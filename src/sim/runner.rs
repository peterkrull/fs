//! Runs a [`Simulation`] on a thread of its own, either in real time with commands from the
//! user, or in lockstep with ArduPilot. Either way, the rendering never holds up the
//! simulation, and only looks at the latest [`Snapshot`].

use std::{
    io,
    net::SocketAddr,
    sync::{Arc, Mutex, MutexGuard},
    thread::JoinHandle,
    time::{Duration, Instant},
};

use super::{Loads, RigidBody, STEP, Sensors, Simulation, Status, propulsion::RotorState};
use crate::sitl::{self, Frame, FrameTracker, Link};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Stepped in real time, with the actuators commanded through [`SimHandle::set_commands`].
    RealTime,
    /// Stepped by ArduPilot SITL, which commands the actuators.
    Lockstep { port: u16 },
}

#[derive(Debug, Clone, Copy)]
pub enum Request {
    Reset,
    Launch,
}

/// The state of the simulation after its latest step.
#[derive(Debug, Clone)]
pub struct Snapshot {
    pub time: f64,
    pub status: Status,
    pub body: RigidBody,
    pub sensors: Sensors,
    pub loads: Loads,
    pub actuators: Vec<f64>,
    pub rotors: Vec<RotorState>,
    pub link: Option<LinkStatus>,
    /// Simulated time over wall clock time, recently.
    pub speed: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct LinkStatus {
    pub peer: Option<SocketAddr>,
    pub port: u16,
    pub frame_rate: u16,
    pub frames: u64,
    /// Whether frames arrived recently.
    pub active: bool,
}

struct Shared {
    snapshot: Snapshot,
    /// An aircraft to carry on with, see [`Simulation::replace_aircraft`].
    replacement: Option<Simulation>,
    commands: Vec<f64>,
    ground: f64,
    requests: Vec<Request>,
    stop: bool,
}

pub struct SimHandle {
    shared: Arc<Mutex<Shared>>,
    thread: Option<JoinHandle<()>>,
    pub mode: Mode,
}

impl SimHandle {
    pub fn spawn(simulation: Simulation, mode: Mode) -> io::Result<Self> {
        let link = match mode {
            Mode::Lockstep { port } => Some(Link::bind(port)?),
            Mode::RealTime => None,
        };
        let shared = Arc::new(Mutex::new(Shared {
            snapshot: snapshot(&simulation, None, 0.0),
            replacement: None,
            commands: simulation.actuators.commands().to_vec(),
            ground: simulation.ground,
            requests: Vec::new(),
            stop: false,
        }));
        let thread = {
            let shared = shared.clone();
            std::thread::Builder::new()
                .name("simulation".into())
                .spawn(move || match link {
                    Some(link) => run_lockstep(simulation, link, mode, &shared),
                    None => run_real_time(simulation, &shared),
                })?
        };
        Ok(Self {
            shared,
            thread: Some(thread),
            mode,
        })
    }

    pub fn snapshot(&self) -> Snapshot {
        self.lock().snapshot.clone()
    }

    /// Normalized actuator commands, used in real time mode.
    pub fn set_commands(&self, commands: &[f64]) {
        self.lock().commands = commands.to_vec();
    }

    /// The height of the terrain below the aircraft.
    pub fn set_ground(&self, height: f64) {
        self.lock().ground = height;
    }

    /// Carry on with another aircraft from where the current one is, such as after its
    /// definition was edited.
    pub fn replace_aircraft(&self, simulation: Simulation) {
        self.lock().replacement = Some(simulation);
    }

    pub fn request(&self, request: Request) {
        self.lock().requests.push(request);
    }

    fn lock(&self) -> MutexGuard<'_, Shared> {
        self.shared
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}

impl Drop for SimHandle {
    fn drop(&mut self) {
        self.lock().stop = true;
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

/// Apply what the owner of the handle asked for, returning whether to stop.
fn sync_inputs(simulation: &mut Simulation, shared: &Mutex<Shared>, commands: bool) -> bool {
    let mut shared = shared.lock().unwrap_or_else(|p| p.into_inner());
    if let Some(next) = shared.replacement.take() {
        simulation.replace_aircraft(next);
    }
    simulation.ground = shared.ground;
    if commands {
        simulation.actuators.set_commands(&shared.commands);
    }
    for request in shared.requests.drain(..) {
        match request {
            Request::Reset => simulation.reset(),
            Request::Launch => simulation.launch(),
        }
    }
    shared.stop
}

fn publish(simulation: &Simulation, shared: &Mutex<Shared>, link: Option<LinkStatus>, speed: f64) {
    let snapshot = snapshot(simulation, link, speed);
    shared.lock().unwrap_or_else(|p| p.into_inner()).snapshot = snapshot;
}

fn snapshot(simulation: &Simulation, link: Option<LinkStatus>, speed: f64) -> Snapshot {
    Snapshot {
        time: simulation.time,
        status: simulation.status,
        body: simulation.body,
        sensors: simulation.sensors(),
        loads: simulation.loads.clone(),
        actuators: simulation.actuators.positions().to_vec(),
        rotors: simulation.rotor_states.clone(),
        link,
        speed,
    }
}

/// Tracks how fast simulated time passes compared to the wall clock.
struct SpeedMeter {
    wall: Instant,
    simulated: f64,
    speed: f64,
}

impl SpeedMeter {
    fn new(time: f64) -> Self {
        Self {
            wall: Instant::now(),
            simulated: time,
            speed: 0.0,
        }
    }

    fn update(&mut self, time: f64) -> f64 {
        let elapsed = self.wall.elapsed().as_secs_f64();
        if elapsed > 0.5 {
            self.speed = (time - self.simulated) / elapsed;
            self.wall = Instant::now();
            self.simulated = time;
        }
        self.speed
    }
}

fn run_real_time(mut simulation: Simulation, shared: &Mutex<Shared>) {
    let start = Instant::now();
    let start_time = simulation.time;
    let mut meter = SpeedMeter::new(simulation.time);
    loop {
        if sync_inputs(&mut simulation, shared, true) {
            return;
        }
        // Catch up with the wall clock, but give up on falling far behind
        let target = start_time + start.elapsed().as_secs_f64();
        let mut steps = 0;
        while simulation.time < target && steps < 50 {
            simulation.step(STEP);
            steps += 1;
        }
        if steps == 50 {
            simulation.time = simulation.time.max(target);
        }
        let speed = meter.update(simulation.time);
        publish(&simulation, shared, None, speed);
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn run_lockstep(mut simulation: Simulation, mut link: Link, mode: Mode, shared: &Mutex<Shared>) {
    let Mode::Lockstep { port } = mode else {
        return;
    };
    let mut tracker = FrameTracker::default();
    let mut meter = SpeedMeter::new(simulation.time);
    let mut status = LinkStatus {
        port,
        ..Default::default()
    };
    let mut reply = String::new();
    let mut last_frame = Instant::now();

    log::info!("Waiting for ArduPilot on UDP port {port}");
    loop {
        if sync_inputs(&mut simulation, shared, false) {
            return;
        }

        if let Some((packet, from)) = link.receive() {
            if status.peer != Some(from) {
                log::info!("ArduPilot connected from {from}");
            }
            status.peer = Some(from);
            status.frame_rate = packet.frame_rate;
            last_frame = Instant::now();

            match tracker.track(packet.frame_count) {
                Frame::Repeat if !reply.is_empty() => {
                    link.send(from, &reply);
                    continue;
                }
                Frame::Restart => {
                    log::info!("ArduPilot restarted, resetting the simulation");
                    simulation.time = 0.0;
                    simulation.reset();
                    meter = SpeedMeter::new(0.0);
                }
                _ => {}
            }

            simulation.actuators.set_pwm(&packet.pwm);
            let (steps, dt) = packet.steps();
            for _ in 0..steps {
                simulation.step(dt);
            }
            status.frames += 1;
            reply = sitl::sensor_json(&simulation.sensors());
            link.send(from, &reply);
        }

        status.active = last_frame.elapsed() < Duration::from_secs(1);
        let speed = meter.update(simulation.time);
        publish(&simulation, shared, Some(status), speed);
    }
}
