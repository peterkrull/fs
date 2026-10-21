//! Checks the 35% scale Sbach 342 of `assets/aircraft/sbach35.ron`: a fast, fully aerobatic
//! model, so unlike the two small aircraft in `src/sim/mod.rs` it is checked at its own speeds.
//!
//! The model is based on the XtremeAir Sbach 342 (7.50 m span, 11.25 m², 850 kg aerobatic mass,
//! 450°/s roll rate) at 35% scale, with the mass and wing loading of a 2.7 m class RC model. It
//! should be stable hands off, trim near 24 m/s, and respond like a large-scale aerobat: a full
//! aileron deflection rolls it at several hundred degrees per second, and the elevator and
//! rudder are powerful enough for snaps.

use bevy::math::{DMat3, DVec3};
use bevy_worldmap::sim::{
    STEP, Sensors, Simulation, Spawn, Status, aero,
    config::{self, AircraftConfig},
    weather::Weather,
};

const LAUNCH_SPEED: f64 = 27.0;

fn aircraft() -> Simulation {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/aircraft/sbach35.ron");
    let config = AircraftConfig::load(path).unwrap();
    let spawn = Spawn {
        lat: 46.0,
        lon: 7.0,
        altitude: 1000.0,
        heading: 0.0,
        speed: LAUNCH_SPEED,
    };
    Simulation::new(config, spawn, Weather::calm()).unwrap()
}

fn sticks(roll: f64, pitch: f64, yaw: f64, throttle: f64) -> config::Sticks {
    config::Sticks {
        roll,
        pitch,
        yaw,
        throttle,
    }
}

/// Launch with fixed sticks and fly for a while, keeping the wings level if asked.
fn fly(sim: &mut Simulation, mut sticks: config::Sticks, level: bool, seconds: f64) -> Sensors {
    sim.actuators.set_commands(&sim.config.mix(&sticks));
    sim.actuators.settle();
    sim.launch();
    for step in 0..(seconds / STEP) as usize {
        if level && step % 10 == 0 {
            let sensors = sim.sensors();
            sticks.roll = -0.5 * sensors.euler.x - 0.1 * sensors.gyro.x;
            sim.actuators.set_commands(&sim.config.mix(&sticks));
        }
        sim.step(STEP);
    }
    assert_eq!(sim.status, Status::Flying);
    sim.sensors()
}

/// The center of gravity is ahead of the neutral point, with enough margin to be flown hands
/// off but little enough to stay crisp, and the model trims near its cruise speed.
#[test]
fn stable_and_trimmed_at_cruise() {
    let mut sim = aircraft();
    let actuators = vec![0.0; sim.actuators.names().len()];
    let mut at = |alpha: f64| {
        let alpha = alpha.to_radians();
        sim.aero.reset();
        let loads = sim.aero.loads(&aero::Flow {
            velocity: DVec3::new(-alpha.cos(), 0.0, -alpha.sin()) * LAUNCH_SPEED,
            gradient: DMat3::ZERO,
            rates: DVec3::ZERO,
            density: 1.225,
            actuators: &actuators,
            dt: STEP,
            slipstreams: &[],
        });
        (-loads.force.z, loads.moment.y)
    };
    let ((l1, m1), (l2, m2)) = (at(2.0), at(6.0));
    // Moving the reference point back by the margin removes the change in moment
    let margin = -(m2 - m1) / (l2 - l1);
    // The angle of attack without a pitching moment, and the speed to fly it level
    let trim = 2.0 - m1 / (m2 - m1) * 4.0;
    let lift = at(trim).0;
    let speed = LAUNCH_SPEED * (sim.config.mass * 9.81 / lift).sqrt();
    println!("static margin {margin:.3} m, trimmed at {trim:.1} deg and {speed:.1} m/s");
    assert!(
        (0.02..0.15).contains(&margin),
        "static margin {margin} m is not crisp but stable"
    );
    assert!(
        (20.0..30.0).contains(&speed),
        "trim speed {speed} m/s is not near cruise"
    );
}

/// With the sticks centered and the engine off, it settles into a stable glide.
#[test]
fn glides_hands_off() {
    let sensors = fly(&mut aircraft(), sticks(0.0, 0.0, 0.0, 0.0), false, 30.0);
    let (roll, pitch) = (sensors.euler.x.to_degrees(), sensors.euler.y.to_degrees());
    println!(
        "glide: {:.1} m/s, {:.1} m/s sink, pitch {pitch:.1} deg, roll {roll:.1} deg",
        sensors.airspeed, sensors.velocity_ned.z
    );
    assert!(roll.abs() < 10.0, "roll {roll}");
    assert!(pitch.abs() < 15.0, "pitch {pitch}");
    assert!(
        (20.0..30.0).contains(&sensors.airspeed),
        "airspeed {}",
        sensors.airspeed
    );
    assert!(
        sensors.velocity_ned.z < 8.0,
        "sink {}",
        sensors.velocity_ned.z
    );
    assert!(
        sensors.gyro.length() < 0.1,
        "still rotating {}",
        sensors.gyro
    );
}

/// Each stick turns the model the right way, and by a lot.
#[test]
fn controls_respond() {
    let rates = |sticks| {
        let mut sim = aircraft();
        sim.actuators.set_commands(&sim.config.mix(&sticks));
        sim.actuators.settle();
        sim.launch();
        for _ in 0..(0.3 / STEP) as usize {
            sim.step(STEP);
        }
        sim.sensors().gyro
    };
    let pitch = rates(sticks(0.0, 0.5, 0.0, 0.6)).y;
    let roll = rates(sticks(0.5, 0.0, 0.0, 0.6)).x;
    let yaw = rates(sticks(0.0, 0.0, 0.5, 0.6)).z;
    println!("pitch {pitch:.2} roll {roll:.2} yaw {yaw:.2} rad/s after 0.3 s");
    assert!(pitch > 0.5, "pitch rate {pitch}");
    assert!(roll > 2.0, "roll rate {roll}");
    assert!(yaw > 0.3, "yaw rate {yaw}");
}

/// Full aileron gives the several hundred degrees per second of a large-scale aerobat.
#[test]
fn rolls_fast() {
    let mut sim = aircraft();
    let sensors = fly(&mut sim, sticks(1.0, 0.0, 0.0, 0.6), false, 1.5);
    let rate = sensors.gyro.x.to_degrees();
    println!(
        "full aileron: {rate:.0} deg/s at {:.1} m/s",
        sensors.airspeed
    );
    assert!(rate > 300.0, "roll rate {rate} deg/s");
}

/// At full throttle it has enough power to climb steeply.
#[test]
fn full_throttle_climbs() {
    let mut sim = aircraft();
    let mut sticks = sticks(0.0, 0.0, 0.0, 1.0);
    sim.actuators.set_commands(&sim.config.mix(&sticks));
    sim.actuators.settle();
    sim.launch();
    let mut integral = 0.0;
    for _ in 0..(10.0 / STEP) as usize {
        let sensors = sim.sensors();
        let error = 20f64.to_radians() - sensors.euler.y;
        integral += error * STEP;
        sticks.roll = -0.5 * sensors.euler.x - 0.1 * sensors.gyro.x;
        sticks.pitch = 2.0 * error - 0.3 * sensors.gyro.y + 0.5 * integral;
        sim.actuators.set_commands(&sim.config.mix(&sticks));
        sim.step(STEP);
    }
    let sensors = sim.sensors();
    println!(
        "full throttle: {:.1} m/s, {:.1} m/s climb",
        sensors.airspeed, -sensors.velocity_ned.z
    );
    assert!(
        sensors.velocity_ned.z < -2.0,
        "vertical speed {}",
        sensors.velocity_ned.z
    );
}
