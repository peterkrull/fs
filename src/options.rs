//! Command line options shared by the viewer and the headless simulator.

use std::path::PathBuf;

use crate::{
    flight::RadioMap,
    sim::{Spawn, environment::ConstantWind, runner::Mode},
    sitl,
};

pub const USAGE: &str = "\
Options:
  --aircraft PATH          aircraft definition (default: assets/aircraft/uav.ron)
  --spawn LAT,LON,ALT,HDG  where the aircraft starts: degrees, meters above sea level, degrees
  --launch-speed M/S       airspeed when launched (default: 14)
  --wind FROM,SPEED        constant wind from a heading in degrees, in m/s
  --sitl [PORT]            run in lockstep with ArduPilot SITL (default port: 9002)
  --radio ORDER            use game controllers as RC radios, with this channel order of
                           aileron, elevator, throttle and rudder, lowercase to reverse
                           (default: AETR, for EdgeTX radios only)
  --no-aircraft            only show the globe (viewer only)";

#[derive(Debug, Clone)]
pub struct Options {
    pub aircraft: Option<PathBuf>,
    pub spawn: Spawn,
    pub wind: ConstantWind,
    pub mode: Mode,
    pub radio: Option<RadioMap>,
    /// Options not recognized here, for the binary to handle.
    pub rest: Vec<String>,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            aircraft: Some(PathBuf::from("assets/aircraft/uav.ron")),
            // Above Interlaken, between the lakes of Thun and Brienz
            spawn: Spawn {
                lat: 46.6863,
                lon: 7.8632,
                altitude: 700.0,
                heading: 70.0,
                speed: 14.0,
            },
            wind: ConstantWind::default(),
            mode: crate::sim::runner::Mode::RealTime,
            radio: None,
            rest: Vec::new(),
        }
    }
}

impl Options {
    pub fn parse(args: impl IntoIterator<Item = String>) -> Result<Self, String> {
        let mut options = Self::default();
        let mut args = args.into_iter().peekable();
        while let Some(arg) = args.next() {
            let mut value = |name: &str| args.next().ok_or_else(|| format!("{name} needs a value"));
            match arg.as_str() {
                "--aircraft" => options.aircraft = Some(PathBuf::from(value("--aircraft")?)),
                "--no-aircraft" => options.aircraft = None,
                "--spawn" => {
                    let v = numbers(&value("--spawn")?, 4)?;
                    let speed = options.spawn.speed;
                    options.spawn = Spawn {
                        lat: v[0],
                        lon: v[1],
                        altitude: v[2],
                        heading: v[3],
                        speed,
                    };
                }
                "--launch-speed" => options.spawn.speed = numbers(&value("--launch-speed")?, 1)?[0],
                "--wind" => {
                    let v = numbers(&value("--wind")?, 2)?;
                    options.wind = ConstantWind {
                        from: v[0],
                        speed: v[1],
                    };
                }
                "--radio" => options.radio = Some(RadioMap::parse(&value("--radio")?)?),
                "--sitl" => {
                    let port = match args.peek().and_then(|p| p.parse().ok()) {
                        Some(port) => {
                            args.next();
                            port
                        }
                        None => sitl::PORT,
                    };
                    options.mode = Mode::Lockstep { port };
                }
                _ => options.rest.push(arg),
            }
        }
        Ok(options)
    }
}

fn numbers(text: &str, count: usize) -> Result<Vec<f64>, String> {
    let values: Vec<f64> = text
        .split(',')
        .map(|v| v.trim().parse().map_err(|_| format!("not a number: {v}")))
        .collect::<Result<_, _>>()?;
    match values.len() == count {
        true => Ok(values),
        false => Err(format!(
            "expected {count} comma separated numbers, got {text}"
        )),
    }
}
