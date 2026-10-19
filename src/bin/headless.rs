//! The flight simulation without graphics, over flat ground, for testing flight controllers.

use std::time::Duration;

use bevy_worldmap::{
    options::{Options, USAGE},
    sim::{
        Simulation, Status,
        config::{ActuatorKind, AircraftConfig},
        runner::{Request, SimHandle},
    },
};

const HEADLESS_USAGE: &str = "
  --ground HEIGHT          height of the flat ground above sea level (default: 100 m below spawn)
  --launch-after SECONDS   launch once this much simulated time has passed
  --duration SECONDS       stop after this much simulated time";

fn main() {
    if let Err(err) = run() {
        eprintln!("{err}\n\n{USAGE}{HEADLESS_USAGE}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let options = Options::parse(std::env::args().skip(1))?;
    let mut ground = options.spawn.altitude - 100.0;
    let mut launch_after = None;
    let mut duration = f64::INFINITY;
    let mut rest = options.rest.iter();
    while let Some(arg) = rest.next() {
        let mut number = || {
            rest.next()
                .and_then(|v| v.parse::<f64>().ok())
                .ok_or_else(|| format!("{arg} needs a number"))
        };
        match arg.as_str() {
            "--ground" => ground = number()?,
            "--launch-after" => launch_after = Some(number()?),
            "--duration" => duration = number()?,
            _ => return Err(format!("unknown option {arg}")),
        }
    }

    let path = options.aircraft.ok_or("an aircraft is needed")?;
    let config = AircraftConfig::load(&path)?;
    println!("Simulating {} in {:?} mode", config.name, options.mode);
    let throttle = config
        .actuators
        .iter()
        .position(|a| a.kind == ActuatorKind::Throttle);
    let simulation = Simulation::new(config, options.spawn, Box::new(options.wind))?;
    let handle = SimHandle::spawn(simulation, options.mode).map_err(|err| err.to_string())?;
    handle.set_ground(ground);

    let mut launched = false;
    loop {
        std::thread::sleep(Duration::from_secs(1));
        let snapshot = handle.snapshot();
        if !launched && launch_after.is_some_and(|t| snapshot.time >= t) {
            handle.request(Request::Launch);
            launched = true;
            println!("Launched");
        }

        let s = &snapshot.sensors;
        let link = snapshot
            .link
            .map(|l| {
                format!(
                    "  frames {} at {} Hz{}",
                    l.frames,
                    l.frame_rate,
                    if l.active { "" } else { " (idle)" }
                )
            })
            .unwrap_or_default();
        println!(
            "t {:7.1}  {:8}  alt {:7.1}  agl {:6.1}  airspeed {:5.1}  roll {:6.1}  pitch {:6.1}  heading {:5.1}  throttle {:4.2}  {:.1}x{link}",
            snapshot.time,
            format!("{:?}", snapshot.status),
            s.altitude,
            s.height_above_ground,
            s.airspeed,
            s.euler.x.to_degrees(),
            s.euler.y.to_degrees(),
            s.euler.z.to_degrees().rem_euclid(360.0),
            throttle
                .and_then(|i| snapshot.actuators.get(i))
                .copied()
                .unwrap_or(0.0),
            snapshot.speed,
        );

        if snapshot.time >= duration || snapshot.status == Status::Crashed {
            return match snapshot.status {
                Status::Crashed => Err("crashed".into()),
                _ => Ok(()),
            };
        }
    }
}
