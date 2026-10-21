//! The flight simulation in the viewer.
//!
//! The simulation runs on a thread of its own, see [`crate::sim::runner`]. Each frame, the
//! viewer takes the latest snapshot of it, to place the aircraft, follow it with the camera, and
//! draw the debug views. In real time mode the viewer sends the pilot's controls to it; in
//! lockstep mode ArduPilot does.

mod debug;
mod radio;
mod visual;

use bevy::{
    input::{
        gamepad::{Gamepad, GamepadAxis},
        mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll},
    },
    math::{DVec2, DVec3},
    prelude::*,
    window::{CursorGrabMode, CursorOptions, PrimaryWindow},
};

use crate::{
    camera::{CameraMode, CameraSystems, Controls, View},
    geo,
    options::Options,
    sim::{
        Simulation, Status,
        config::{AircraftConfig, Sticks, Watcher},
        runner::{Mode, Request, SimHandle, Snapshot},
        weather::{Ground, Probe},
    },
    tiles::Tiles,
};

pub use debug::DebugView;
pub use radio::RadioMap;

pub struct FlightPlugin {
    pub options: Options,
}

impl Plugin for FlightPlugin {
    fn build(&self, app: &mut App) {
        let options = &self.options;
        let Some(path) = &options.aircraft else {
            return;
        };
        let flight = match Flight::new(path, options) {
            Ok(flight) => flight,
            Err(err) => {
                error!("Unable to start the flight simulation: {err}");
                return;
            }
        };
        if let Mode::Lockstep { port } = options.mode {
            info!("{}", arduplane_command(path, options, port));
        }

        // Far enough behind to see the whole aircraft
        let span = flight
            .model
            .aero
            .strips
            .iter()
            .map(|s| s.a.y.abs().max(s.b.y.abs()))
            .fold(0.0, f64::max)
            * 2.0;
        app.insert_resource(Chase {
            distance: (1.5 * span).max(Chase::default().distance),
            ..default()
        });
        app.insert_resource(flight)
            .add_message::<AircraftReloaded>()
            .insert_resource(options.radio.unwrap_or_default())
            .insert_resource(CameraMode::Chase)
            .init_resource::<Pilot>()
            .init_resource::<DebugView>()
            .add_systems(Startup, (visual::spawn, spawn_panel, debug::startup))
            .add_systems(
                Update,
                (
                    (
                        reload,
                        visual::spawn.run_if(on_message::<AircraftReloaded>),
                        take_snapshot,
                        keys.run_if(resource_exists::<Controls>),
                    )
                        .chain()
                        .before(CameraSystems),
                    (chase, keep_in_view).chain().in_set(CameraSystems),
                    radio::configure,
                    (pilot, visual::update, debug::draw, update_panel).after(CameraSystems),
                ),
            );
    }
}

/// How to start ArduPlane for the simulation. Without `--serial0 tcp:0`, ArduPlane waits for a
/// ground station to connect to TCP port 5760 before it starts, and sends nothing until then.
fn arduplane_command(aircraft: &std::path::Path, options: &Options, port: u16) -> String {
    let s = options.spawn;
    let mut defaults = "Tools/autotest/models/plane.parm".to_string();
    // The parameters for an aircraft are kept next to its definition
    if let Ok(parameters) = aircraft.with_extension("parm").canonicalize() {
        defaults.push_str(&format!(",{}", parameters.display()));
    }
    // Every instance of ArduPilot adds 10 to its ports, starting from 9002 for the simulator
    // and 5760 for the ground station
    let (ports, ground_station) = match port.checked_sub(crate::sitl::PORT) {
        Some(0) => (String::new(), 5760),
        Some(offset) if offset % 10 == 0 => (format!(" -I {}", offset / 10), 5760 + offset),
        _ => (format!(" --sim-port-out {port}"), 5760),
    };
    format!(
        "Listening for ArduPilot on UDP port {port}. Start ArduPlane from the ArduPilot \
         repository with:\n\n    build/sitl/bin/arduplane --model JSON:127.0.0.1{ports} \
         --serial0 tcp:0 --home {},{},{},{} --defaults {defaults}\n\n\
         Then connect a ground station to TCP port {ground_station}.",
        s.lat, s.lon, s.altitude, s.heading,
    )
}

/// How often the aircraft definition is checked for changes, in seconds.
const RELOAD_INTERVAL: f32 = 0.25;

/// Sent when the aircraft was reloaded from its edited definition.
#[derive(Message)]
pub struct AircraftReloaded;

/// The simulation, and a copy of its aircraft on the main thread to look up its geometry.
#[derive(Resource)]
pub struct Flight {
    handle: SimHandle,
    pub model: Simulation,
    pub snapshot: Snapshot,
    watcher: Watcher,
    weather_watcher: Option<Watcher>,
    options: Options,
    /// Why an edited definition could not be loaded, until it can.
    reload_error: Option<String>,
}

impl Flight {
    fn new(path: &std::path::Path, options: &Options) -> Result<Self, String> {
        let watcher = Watcher::new(path);
        let config = AircraftConfig::load(path)?;
        let spawn = options.spawn_for(&config);
        let weather = options.build_weather()?;
        let model = Simulation::new(config.clone(), spawn, weather.clone())?;
        let simulation = Simulation::new(config, spawn, weather)?;
        let handle = SimHandle::spawn(simulation, options.mode).map_err(|err| format!("{err}"))?;
        let snapshot = handle.snapshot();
        Ok(Self {
            handle,
            model,
            snapshot,
            watcher,
            weather_watcher: options.weather.as_ref().map(Watcher::new),
            options: options.clone(),
            reload_error: None,
        })
    }

    /// Carry on with another aircraft, from where the current one is.
    fn replace_aircraft(&mut self, config: AircraftConfig) -> Result<(), String> {
        let spawn = self.options.spawn_for(&config);
        // Simulations keep their weather when replacing their aircraft
        let weather = self.model.weather.clone();
        let model = Simulation::new(config.clone(), spawn, weather.clone())?;
        let simulation = Simulation::new(config, spawn, weather)?;
        self.handle.replace_aircraft(simulation);
        self.model = model;
        Ok(())
    }

    pub fn request(&self, request: Request) {
        self.handle.request(request);
    }

    /// Where a point given in the model frame currently is, in ECEF.
    pub fn to_ecef(&self, point: DVec3) -> DVec3 {
        let body = &self.snapshot.body;
        body.position + body.attitude * (point - self.model.center_of_gravity)
    }
}

/// Load the aircraft or the weather again whenever its definition is saved. A definition with
/// errors is reported, and the aircraft flies on as it was.
fn reload(
    time: Res<Time>,
    mut next: Local<f32>,
    mut flight: ResMut<Flight>,
    mut reloaded: MessageWriter<AircraftReloaded>,
) {
    let now = time.elapsed_secs();
    if now < *next {
        return;
    }
    *next = now + RELOAD_INTERVAL;
    let flight = &mut *flight;
    let options = &flight.options;
    if let Some(weather) = flight
        .weather_watcher
        .as_mut()
        .and_then(|watcher| watcher.poll(|_| options.build_weather()))
    {
        match weather {
            Ok(weather) => {
                info!("Reloaded the weather from {:?}", options.weather);
                flight.handle.replace_weather(weather.clone());
                flight.model.weather = weather;
                flight.reload_error = None;
            }
            Err(err) => {
                error!("Unable to reload the weather: {err}");
                flight.reload_error = Some(err);
            }
        }
    }
    let Some(config) = flight.watcher.poll(|path| AircraftConfig::load(path)) else {
        return;
    };
    match config.and_then(|config| flight.replace_aircraft(config)) {
        Ok(()) => {
            info!("Reloaded the aircraft from {:?}", flight.watcher.path());
            flight.reload_error = None;
            reloaded.write(AircraftReloaded);
        }
        Err(err) => {
            error!("Unable to reload the aircraft: {err}");
            flight.reload_error = Some(err);
        }
    }
}

fn take_snapshot(mut flight: ResMut<Flight>, tiles: Res<Tiles>) {
    flight.snapshot = flight.handle.snapshot();
    let (lon_lat, _) = geo::from_ecef(flight.snapshot.body.position);
    let height = |lon_lat: DVec2| tiles.height_at(lon_lat, 15).map(f64::from);
    let Some(ground) = height(lon_lat) else {
        return;
    };
    // The slope over the distance the ridge lift is reckoned over
    let distance = flight.model.weather.config().ridge_lift.scale.max(10.0);
    let east = DVec2::X * distance / (geo::EARTH_RADIUS * lon_lat.y.cos());
    let north = DVec2::Y * distance / geo::EARTH_RADIUS;
    let around = [east, -east, north, -north].map(|offset| height(lon_lat + offset));
    let slope = match around {
        [Some(e), Some(w), Some(n), Some(s)] => DVec2::new(e - w, n - s) / (2.0 * distance),
        _ => DVec2::ZERO,
    };
    flight.handle.set_ground(Ground {
        height: ground,
        slope,
    });
}

fn keys(
    keys: Res<ButtonInput<KeyCode>>,
    flight: Res<Flight>,
    mut mode: ResMut<CameraMode>,
    mut debug: ResMut<DebugView>,
    mut pilot: ResMut<Pilot>,
) {
    if keys.just_pressed(KeyCode::KeyL) {
        flight.handle.request(Request::Launch);
    }
    if keys.just_pressed(KeyCode::KeyR) {
        flight.handle.request(Request::Reset);
        *pilot = Pilot::default();
    }
    if keys.just_pressed(KeyCode::Tab) {
        *mode = match *mode {
            CameraMode::Free => CameraMode::Chase,
            CameraMode::Chase => CameraMode::Free,
        };
    }
    if keys.just_pressed(KeyCode::F1) {
        debug.forces = !debug.forces;
    }
    if keys.just_pressed(KeyCode::F2) {
        debug.schema = !debug.schema;
    }
    if keys.just_pressed(KeyCode::F3) {
        debug.model = !debug.model;
    }
    if keys.just_pressed(KeyCode::F4) {
        debug.weather = !debug.weather;
    }
}

/// The controls of the pilot, when flying without a flight controller.
#[derive(Resource, Default)]
struct Pilot {
    roll: f64,
    pitch: f64,
    yaw: f64,
    throttle: f64,
    /// The channels of a connected RC radio, if any, of which those that have not reported yet
    /// are `None`.
    radio: Option<[Option<f64>; radio::CHANNELS]>,
}

/// How fast the keys move the controls, in full travel per second.
const KEY_RATE: f64 = 4.0;
const THROTTLE_RATE: f64 = 0.5;
const DEADZONE: f32 = 0.08;

fn pilot(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    // Without it, the keyboard is ignored, but controllers are not
    controls: Option<Res<Controls>>,
    mut gamepads: Query<(&Gamepad, Option<&mut radio::Reported>)>,
    radio_map: Res<RadioMap>,
    mode: Res<CameraMode>,
    flight: Res<Flight>,
    mut pilot: ResMut<Pilot>,
) {
    if flight.handle.mode != Mode::RealTime {
        return;
    }

    // A radio sets the controls directly, and takes precedence
    pilot.radio = gamepads
        .iter_mut()
        .find_map(|(gamepad, reported)| Some(radio::channels(gamepad, reported?.into_inner())));
    if let Some(channels) = pilot.radio {
        [pilot.roll, pilot.pitch, pilot.yaw, pilot.throttle] = radio_map.controls(&channels);
        send_controls(&flight, &pilot);
        return;
    }

    let dt = time.delta_secs_f64();
    let keyboard = controls.is_some();
    let axis = |pos: KeyCode, neg: KeyCode| match keyboard {
        true => keys.pressed(pos) as i32 as f64 - keys.pressed(neg) as i32 as f64,
        false => 0.0,
    };
    // Keys move the controls gradually, and center them when released
    let toward =
        |value: f64, target: f64| value + (target - value).clamp(-KEY_RATE * dt, KEY_RATE * dt);

    // The arrows are free in either camera mode, the letters only when they do not fly the camera
    let chase = *mode == CameraMode::Chase;
    let mut roll = axis(KeyCode::ArrowRight, KeyCode::ArrowLeft);
    let mut pitch = axis(KeyCode::ArrowDown, KeyCode::ArrowUp);
    let mut yaw = if chase {
        axis(KeyCode::KeyD, KeyCode::KeyA)
    } else {
        0.0
    };
    let mut throttle = if chase {
        axis(KeyCode::KeyW, KeyCode::KeyS)
    } else {
        0.0
    };

    // Mode 2 gamepad: throttle and yaw on the left stick, roll and pitch on the right
    for (gamepad, _) in gamepads.iter().filter(|(_, reported)| reported.is_none()) {
        let stick = |axis| {
            gamepad
                .get(axis)
                .filter(|v: &f32| v.abs() > DEADZONE)
                .map(f64::from)
        };
        roll = stick(GamepadAxis::RightStickX).unwrap_or(roll);
        pitch = stick(GamepadAxis::RightStickY).map(|v| -v).unwrap_or(pitch);
        yaw = stick(GamepadAxis::LeftStickX).unwrap_or(yaw);
        throttle = stick(GamepadAxis::LeftStickY).unwrap_or(throttle);
    }

    pilot.roll = toward(pilot.roll, roll);
    pilot.pitch = toward(pilot.pitch, pitch);
    pilot.yaw = toward(pilot.yaw, yaw);
    pilot.throttle = (pilot.throttle + throttle * THROTTLE_RATE * dt).clamp(0.0, 1.0);
    send_controls(&flight, &pilot);
}

/// Mix the controls of the pilot into the actuators, as the aircraft defines.
fn send_controls(flight: &Flight, pilot: &Pilot) {
    let sticks = Sticks {
        roll: pilot.roll,
        pitch: pilot.pitch,
        yaw: pilot.yaw,
        throttle: pilot.throttle,
    };
    flight
        .handle
        .set_commands(&flight.model.config.mix(&sticks));
}

/// The camera following the aircraft, which the mouse orbits and the scroll wheel zooms.
#[derive(Resource)]
pub struct Chase {
    pub distance: f64,
    /// Offsets from behind the aircraft, in radians.
    pub yaw: f64,
    pub pitch: f64,
    /// Heading of the aircraft, smoothed so that the camera does not jerk around.
    pub heading: Option<f64>,
}

impl Default for Chase {
    fn default() -> Self {
        Self {
            distance: 5.0,
            yaw: 0.0,
            pitch: 12f64.to_radians(),
            heading: None,
        }
    }
}

fn chase(
    time: Res<Time>,
    mode: Res<CameraMode>,
    flight: Res<Flight>,
    tiles: Res<Tiles>,
    mouse: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    cursor: Single<&CursorOptions, With<PrimaryWindow>>,
    mut chase: ResMut<Chase>,
    mut view: ResMut<View>,
) {
    if *mode != CameraMode::Chase {
        chase.heading = None;
        return;
    }
    let dt = time.delta_secs_f64();
    let body = &flight.snapshot.body;
    let target = body.position;
    let (east, north, up) = geo::enu(target);
    let forward = body.attitude * DVec3::X;
    let heading = forward.dot(east).atan2(forward.dot(north));
    let smoothed = chase.heading.map_or(heading, |previous| {
        let delta = crate::sim::airfoil::wrap_angle(heading - previous);
        previous + delta * (1.0 - (-dt * 3.0).exp())
    });
    chase.heading = Some(smoothed);

    if cursor.grab_mode != CursorGrabMode::None || mouse.pressed(MouseButton::Right) {
        let delta = motion.delta.as_dvec2() * 0.004;
        chase.yaw += delta.x;
        chase.pitch = (chase.pitch + delta.y).clamp(-80f64.to_radians(), 85f64.to_radians());
    }
    if scroll.delta.y != 0.0 {
        chase.distance = (chase.distance * 0.85f64.powf(scroll.delta.y as f64)).clamp(1.0, 2000.0);
    }

    let yaw = smoothed + chase.yaw;
    let behind = -(north * yaw.cos() + east * yaw.sin());
    let mut position =
        target + (behind * chase.pitch.cos() + up * chase.pitch.sin()) * chase.distance;

    // Stay above the terrain
    let (lon_lat, height) = geo::from_ecef(position);
    let ground = tiles.height_at(lon_lat, 15).unwrap_or(0.0) as f64;
    if height < ground + 1.0 {
        position = geo::to_ecef(lon_lat, ground + 1.0);
    }

    view.position = position;
    view.forward = (target - position).normalize();
    view.ground = ground;
}

/// Keep the near plane of the camera closer than the aircraft, wherever the camera is.
fn keep_in_view(flight: Res<Flight>, mut view: ResMut<View>) {
    let radius = flight
        .model
        .aero
        .strips
        .iter()
        .flat_map(|s| s.leading.into_iter().chain(s.trailing))
        .map(|p| (p - flight.model.center_of_gravity).length())
        .fold(0.5, f64::max);
    let distance = (flight.snapshot.body.position - view.position).length();
    view.nearest = (distance - radius).max(0.05);
}

#[derive(Component)]
struct Panel;

fn spawn_panel(mut commands: Commands) {
    commands.spawn((
        Text::default(),
        TextFont::from_font_size(14.0),
        TextShadow {
            offset: Vec2::splat(1.0),
            color: Color::BLACK.with_alpha(0.6),
        },
        TextLayout::justify(Justify::Right),
        Node {
            position_type: PositionType::Absolute,
            // Below the statistics of the globe, which may span the width of narrow windows
            top: Val::Px(120.0),
            right: Val::Px(12.0),
            ..default()
        },
        Panel,
    ));
}

fn update_panel(
    time: Res<Time>,
    mut last: Local<f32>,
    flight: Res<Flight>,
    pilot: Res<Pilot>,
    radio_map: Res<RadioMap>,
    debug: Res<DebugView>,
    mode: Res<CameraMode>,
    mut panel: Single<&mut Text, With<Panel>>,
) {
    let now = time.elapsed_secs();
    if now - *last < 0.1 {
        return;
    }
    *last = now;

    let snapshot = &flight.snapshot;
    let s = &snapshot.sensors;
    let config = &flight.model.config;

    let link = match (flight.handle.mode, snapshot.link) {
        (Mode::RealTime, _) => "manual control".to_string(),
        (Mode::Lockstep { port }, Some(link)) if link.active => format!(
            "ArduPilot on UDP {port}: {} Hz, frame {}",
            link.frame_rate, link.frames
        ),
        (Mode::Lockstep { port }, _) => format!(
            "waiting for ArduPilot on UDP {port}\n\
             arduplane waits for a ground station on TCP 5760 to start,\n\
             unless started with --serial0 tcp:0"
        ),
    };
    let status = match snapshot.status {
        Status::Held => "HELD, press L to launch",
        Status::Flying => "FLYING",
        Status::Crashed => "CRASHED, press R to reset",
    };
    let actuators: Vec<String> = config
        .actuators
        .iter()
        .zip(&snapshot.actuators)
        .map(|(a, v)| format!("{} {v:+.2}", a.name))
        .collect();
    let rpm: Vec<String> = snapshot
        .rotors
        .iter()
        .map(|r| format!("{:.0}", r.speed * 60.0 / std::f64::consts::TAU))
        .collect();
    let ground_speed = DVec2::new(s.velocity_ned.x, s.velocity_ned.y).length();
    let wind = s.wind_ned;
    let weather = &flight.model.weather;
    let probe = Probe::new(snapshot.body.position, s.time, Ground::UNKNOWN);
    let air = weather.air(&probe);
    let sea_level = Probe::new(geo::to_ecef(probe.lon_lat, 0.0), s.time, Ground::UNKNOWN);

    let reload_error = match &flight.reload_error {
        Some(err) => format!("\nnot reloaded: {err}"),
        None => String::new(),
    };
    let mut text = format!(
        "{}{reload_error}\n{link}   {:.1}x real time\n{status}\n\n\
         airspeed {:.1} m/s   ground speed {:.1} m/s   climb {:+.1} m/s\n\
         altitude {:.0} m   above ground {:.0} m\n\
         wind from {:03.0} at {:.1} m/s   vertical {:+.1} m/s\n\
         sea level {:.0} hPa   air {:.0} C {:.3} kg/m3\n\
         roll {:+.0}   pitch {:+.0}   heading {:.0} deg\n\
         {}\nrpm {}   battery {:.1} V {:.1} A\n\
         accel {:+.2} {:+.2} {:+.2} m/s2   gyro {:+.2} {:+.2} {:+.2} rad/s\n\
         {:.5} N  {:.5} E",
        config.name,
        snapshot.speed,
        s.airspeed,
        ground_speed,
        -s.velocity_ned.z,
        s.altitude,
        s.height_above_ground,
        (-wind.y).atan2(-wind.x).to_degrees().rem_euclid(360.0),
        DVec2::new(wind.x, wind.y).length(),
        -wind.z,
        weather.air(&sea_level).pressure / 100.0,
        air.temperature - 273.15,
        air.density,
        s.euler.x.to_degrees(),
        s.euler.y.to_degrees(),
        s.euler.z.to_degrees().rem_euclid(360.0),
        actuators.join("   "),
        rpm.join(" "),
        s.battery_voltage,
        s.battery_current,
        s.accel.x,
        s.accel.y,
        s.accel.z,
        s.gyro.x,
        s.gyro.y,
        s.gyro.z,
        s.latitude,
        s.longitude,
    );
    text.push_str(&format!(
        "\n\nL: launch   R: reset   Tab: {} camera\nF1: forces{}   F2: aerodynamic model{}   F3: {} aircraft   F4: weather{}",
        match *mode {
            CameraMode::Chase => "free",
            CameraMode::Free => "chase",
        },
        if debug.forces { " (on)" } else { "" },
        if debug.schema { " (on)" } else { "" },
        if debug.model { "hide" } else { "show" },
        if debug.weather { " (on)" } else { "" },
    ));
    if flight.handle.mode == Mode::RealTime {
        match pilot.radio {
            Some(channels) => {
                let values: Vec<String> = channels
                    .iter()
                    .map(|c| c.map_or("--".into(), |c| format!("{c:+.2}")))
                    .collect();
                text.push_str(&format!(
                    "\nRC radio {}: {}",
                    radio_map.describe(),
                    values.join(" ")
                ));
                if !radio_map.synced(&channels) {
                    text.push_str(
                        "\nMove all sticks to read their positions, throttle closed until then",
                    );
                }
            }
            None => text.push_str("\nArrows: roll and pitch   A D: yaw   W S: throttle"),
        }
    }
    panel.0 = text;
}
