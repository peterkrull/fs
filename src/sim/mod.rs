//! The flight simulation engine.
//!
//! An aircraft is defined by an [`AircraftConfig`], and simulated as a rigid body with
//! aerodynamic, propulsive and gravitational forces acting on it. The state is kept in `f64`
//! in earth-centered, earth-fixed coordinates, and the body frame is forward-right-down.
//!
//! The engine does not depend on rendering, and runs at a fixed step of [`STEP`] seconds,
//! either in real time, or in lockstep with a flight controller, see [`crate::sitl`].

pub mod actuator;
pub mod aero;
pub mod airfoil;
pub mod config;
pub mod environment;
pub mod propulsion;
pub mod runner;

use bevy::math::{DMat3, DQuat, DVec2, DVec3, EulerRot};

use crate::geo;
use actuator::Actuators;
use aero::{AeroLoads, Aerodynamics, Flow, Slipstream};
use config::AircraftConfig;
use environment::{WindField, air_density, gravity, ned_to_ecef, ned_to_ecef_at};
use propulsion::{RotorModel, RotorState};

/// Duration of a simulation step in seconds.
pub const STEP: f64 = 0.001;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Held in place at the spawn point, as if in the hand before a launch.
    Held,
    Flying,
    /// Touched the ground, and stopped there.
    Crashed,
}

/// Where the aircraft starts, with angles in degrees and the altitude above sea level.
#[derive(Debug, Clone, Copy)]
pub struct Spawn {
    pub lat: f64,
    pub lon: f64,
    pub altitude: f64,
    pub heading: f64,
    /// Airspeed the aircraft is launched with.
    pub speed: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct RigidBody {
    /// Position of the center of gravity in ECEF.
    pub position: DVec3,
    pub velocity: DVec3,
    /// Rotation from the body frame to ECEF.
    pub attitude: DQuat,
    /// Angular velocity in the body frame.
    pub rates: DVec3,
}

/// What the sensors of a flight controller would measure, without noise.
#[derive(Debug, Clone, Copy, Default)]
pub struct Sensors {
    pub time: f64,
    /// Angular velocity in the body frame, rad/s.
    pub gyro: DVec3,
    /// Specific force in the body frame, m/s². Reads `-g` on the `z` axis when level and still.
    pub accel: DVec3,
    /// Degrees.
    pub latitude: f64,
    pub longitude: f64,
    /// Meters above sea level.
    pub altitude: f64,
    pub velocity_ned: DVec3,
    /// Rotation from the body frame to north-east-down.
    pub attitude_ned: DQuat,
    /// Roll, pitch and yaw in radians.
    pub euler: DVec3,
    /// Airspeed along the body `x` axis, as a pitot tube would measure.
    pub airspeed: f64,
    pub wind_ned: DVec3,
    pub battery_voltage: f64,
    pub battery_current: f64,
    pub height_above_ground: f64,
}

/// All forces on the aircraft from the last step, in the body frame.
#[derive(Debug, Clone, Default)]
pub struct Loads {
    pub aero: AeroLoads,
    pub rotors: Vec<DVec3>,
    pub force: DVec3,
    pub moment: DVec3,
}

pub struct Simulation {
    pub config: AircraftConfig,
    pub aero: Aerodynamics,
    pub rotors: Vec<RotorModel>,
    pub rotor_states: Vec<RotorState>,
    pub actuators: Actuators,
    pub center_of_gravity: DVec3,
    mass: f64,
    inertia: DMat3,
    inertia_inverse: DMat3,
    /// Points of the aircraft checked for contact with the ground, relative to the center of
    /// gravity.
    extremities: Vec<DVec3>,
    pub body: RigidBody,
    pub status: Status,
    pub time: f64,
    pub spawn: Spawn,
    pub wind: Box<dyn WindField>,
    /// Height of the terrain below the aircraft, provided by whoever knows the terrain.
    pub ground: f64,
    pub loads: Loads,
    specific_force: DVec3,
    battery_current: f64,
}

impl Simulation {
    pub fn new(
        config: AircraftConfig,
        spawn: Spawn,
        wind: Box<dyn WindField>,
    ) -> Result<Self, String> {
        if config.mass <= 0.0 {
            return Err("the mass must be positive".into());
        }
        let actuators = Actuators::new(config.actuators.clone());
        let names = actuators.names();
        let find = |name: &str| {
            names
                .iter()
                .position(|n| n == name)
                .ok_or_else(|| format!("unknown actuator {name}"))
        };
        let aero = Aerodynamics::new(&config, &names)?;
        let rotors = config
            .rotors
            .iter()
            .map(|r| RotorModel::new(r, find(&r.throttle)?))
            .collect::<Result<Vec<_>, _>>()?;

        let i = config.inertia;
        let inertia = DMat3::from_cols(
            DVec3::new(i.ixx, 0.0, -i.ixz),
            DVec3::new(0.0, i.iyy, 0.0),
            DVec3::new(-i.ixz, 0.0, i.izz),
        );
        if inertia.determinant() <= 0.0 {
            return Err("the inertia must be positive definite".into());
        }
        let center_of_gravity = DVec3::from(config.center_of_gravity);

        let mut extremities: Vec<DVec3> = aero
            .strips
            .iter()
            .flat_map(|s| s.leading.into_iter().chain(s.trailing))
            .collect();
        for body in &config.bodies {
            let nose = DVec3::from(body.nose);
            let radius = body.profile.iter().map(|(_, r)| *r).fold(0.0, f64::max);
            for t in [0.0, 0.5, 1.0] {
                extremities.push(nose - DVec3::X * (body.length * t) + DVec3::Z * radius);
            }
        }
        extremities.extend(rotors.iter().map(|r| r.position));
        for point in &mut extremities {
            *point -= center_of_gravity;
        }

        let mut simulation = Self {
            rotor_states: vec![RotorState::default(); rotors.len()],
            mass: config.mass,
            inertia,
            inertia_inverse: inertia.inverse(),
            extremities,
            config,
            aero,
            rotors,
            actuators,
            center_of_gravity,
            body: spawn_body(&spawn),
            status: Status::Held,
            time: 0.0,
            spawn,
            wind,
            ground: f64::NEG_INFINITY,
            loads: Loads::default(),
            specific_force: DVec3::ZERO,
            battery_current: 0.0,
        };
        simulation.reset();
        Ok(simulation)
    }

    /// Back to the spawn point, held in place.
    pub fn reset(&mut self) {
        self.body = spawn_body(&self.spawn);
        self.status = Status::Held;
        self.rotor_states.fill(RotorState::default());
        self.aero.reset();
        self.specific_force = self.held_specific_force();
    }

    /// Carry on with another aircraft, such as an edited definition of this one, from where
    /// this one is: in flight or held, at the same time, place and attitude, in the same wind.
    /// Actuators and rotors keep their state where their names match.
    pub fn replace_aircraft(&mut self, mut next: Simulation) {
        next.body = self.body;
        next.status = self.status;
        next.time = self.time;
        // From the same place, but at the launch speed of the new aircraft
        next.spawn = Spawn {
            speed: next.spawn.speed,
            ..self.spawn
        };
        next.ground = self.ground;
        next.specific_force = self.specific_force;
        next.battery_current = self.battery_current;
        next.actuators.carry_over(&self.actuators);
        for (rotor, state) in next.rotors.iter().zip(&mut next.rotor_states) {
            if let Some(i) = self.rotors.iter().position(|r| r.name == rotor.name) {
                *state = self.rotor_states[i];
            }
        }
        std::mem::swap(&mut next.wind, &mut self.wind);
        *self = next;
    }

    /// Release the aircraft at its launch speed.
    pub fn launch(&mut self) {
        if self.status == Status::Held {
            self.status = Status::Flying;
            self.body.velocity = self.body.attitude * DVec3::X * self.spawn.speed;
            self.aero.reset();
        }
    }

    pub fn step(&mut self, dt: f64) {
        self.actuators.step(dt);

        let body = self.body;
        let wind = self.wind.velocity(body.position, self.time);
        let to_body = body.attitude.inverse();
        let air = to_body * (wind - body.velocity);
        let (_, height) = geo::from_ecef(body.position);
        let density = air_density(height);

        // The battery sags under the current of the previous step
        let battery = &self.config.battery;
        let voltage =
            (battery.voltage - battery.internal_resistance * self.battery_current).max(0.0);
        let mut force = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        let mut current = 0.0;
        let mut rotor_forces = Vec::with_capacity(self.rotors.len());
        let mut slipstreams: Vec<Slipstream> = Vec::with_capacity(self.rotors.len());
        for (rotor, state) in self.rotors.iter().zip(&mut self.rotor_states) {
            let offset = rotor.position - self.center_of_gravity;
            let rotor_air = air - body.rates.cross(offset);
            let throttle = self.actuators.positions()[rotor.throttle];
            let output = rotor.step(
                state,
                throttle,
                voltage,
                rotor_air,
                density,
                self.center_of_gravity,
                dt,
            );
            force += output.force;
            moment += output.moment;
            current += output.current;
            rotor_forces.push(output.force);
            slipstreams.push(output.slipstream);
        }
        self.battery_current = current;

        let aero = self.aero.loads(&Flow {
            velocity: air,
            rates: body.rates,
            density,
            actuators: self.actuators.positions(),
            dt,
            slipstreams: &slipstreams,
        });
        force += aero.force;
        moment += aero.moment;
        self.loads = Loads {
            aero,
            rotors: rotor_forces,
            force,
            moment,
        };

        match self.status {
            Status::Flying => {
                self.specific_force = force / self.mass;
                self.integrate(force, moment, dt);
                if self.touches_ground() {
                    self.status = Status::Crashed;
                    self.body.velocity = DVec3::ZERO;
                    self.body.rates = DVec3::ZERO;
                }
            }
            Status::Held | Status::Crashed => {
                self.specific_force = self.held_specific_force();
            }
        }
        self.time += dt;
    }

    fn integrate(&mut self, force: DVec3, moment: DVec3, dt: f64) {
        let body = &mut self.body;
        let acceleration = body.attitude * (force / self.mass) + gravity(body.position);
        body.velocity += acceleration * dt;
        body.position += body.velocity * dt;

        let momentum: DVec3 = self
            .rotors
            .iter()
            .zip(&self.rotor_states)
            .map(|(rotor, state)| rotor.momentum(state))
            .sum();
        let angular = self.inertia * body.rates + momentum;
        let angular_acceleration = self.inertia_inverse * (moment - body.rates.cross(angular));
        body.rates += angular_acceleration * dt;
        body.attitude = (body.attitude * DQuat::from_scaled_axis(body.rates * dt)).normalize();
    }

    fn touches_ground(&self) -> bool {
        let body = &self.body;
        let surface = geo::EARTH_RADIUS + self.ground;
        // Most of the time the aircraft is far above the ground
        if body.position.length() - surface > 100.0 {
            return false;
        }
        self.extremities
            .iter()
            .any(|p| (body.position + body.attitude * *p).length() < surface)
    }

    /// What the accelerometer reads while the aircraft is held still.
    fn held_specific_force(&self) -> DVec3 {
        self.body.attitude.inverse() * -gravity(self.body.position)
    }

    pub fn sensors(&self) -> Sensors {
        let body = &self.body;
        let (lon_lat, altitude) = geo::from_ecef(body.position);
        let to_ned = ned_to_ecef_at(lon_lat).inverse();
        let attitude_ned = to_ned * body.attitude;
        let (yaw, pitch, roll) = attitude_ned.to_euler(EulerRot::ZYX);
        let wind = self.wind.velocity(body.position, self.time);
        let air = body.attitude.inverse() * (wind - body.velocity);
        let battery = &self.config.battery;
        Sensors {
            time: self.time,
            gyro: body.rates,
            accel: self.specific_force,
            latitude: lon_lat.y.to_degrees(),
            longitude: lon_lat.x.to_degrees(),
            altitude,
            velocity_ned: to_ned * body.velocity,
            attitude_ned,
            euler: DVec3::new(roll, pitch, yaw),
            airspeed: (-air.x).max(0.0),
            wind_ned: to_ned * wind,
            battery_voltage: battery.voltage - battery.internal_resistance * self.battery_current,
            battery_current: self.battery_current,
            height_above_ground: altitude - self.ground,
        }
    }

    /// Position in ECEF of a point given in the model frame.
    pub fn model_to_ecef(&self, point: DVec3) -> DVec3 {
        self.body.position + self.body.attitude * (point - self.center_of_gravity)
    }
}

fn spawn_body(spawn: &Spawn) -> RigidBody {
    let lon_lat = DVec2::new(spawn.lon.to_radians(), spawn.lat.to_radians());
    let position = geo::to_ecef(lon_lat, spawn.altitude);
    RigidBody {
        position,
        velocity: DVec3::ZERO,
        attitude: ned_to_ecef(position) * DQuat::from_rotation_z(spawn.heading.to_radians()),
        rates: DVec3::ZERO,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use environment::ConstantWind;

    fn spawn() -> Spawn {
        Spawn {
            lat: 46.0,
            lon: 7.0,
            altitude: 1000.0,
            heading: 90.0,
            speed: 15.0,
        }
    }

    #[test]
    fn held_reads_gravity() {
        let mut sim = aircraft("uav");
        sim.step(STEP);
        let sensors = sim.sensors();
        assert!(
            (sensors.accel - DVec3::new(0.0, 0.0, -9.8)).length() < 0.02,
            "{}",
            sensors.accel
        );
        // Heading east, level
        assert!(
            (sensors.euler - DVec3::new(0.0, 0.0, 90f64.to_radians())).length() < 1e-9,
            "{}",
            sensors.euler
        );
        assert!((sensors.altitude - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn free_fall_reads_zero() {
        let config = AircraftConfig::parse(
            r#"(name: "ball", mass: 1.0, center_of_gravity: (0.0, 0.0, 0.0),
                inertia: (ixx: 0.1, iyy: 0.1, izz: 0.1), actuators: [])"#,
        )
        .unwrap();
        let mut sim = Simulation::new(config, spawn(), Box::new(ConstantWind::default())).unwrap();
        sim.launch();
        sim.body.velocity = DVec3::ZERO;
        for _ in 0..1000 {
            sim.step(STEP);
        }
        let sensors = sim.sensors();
        assert!(sensors.accel.length() < 1e-9);
        // Fallen half g t² after a second
        assert!(
            (sensors.altitude - (1000.0 - 0.5 * 9.80)).abs() < 0.05,
            "{}",
            sensors.altitude
        );
        assert!((sensors.velocity_ned.z - 9.80).abs() < 0.02);
    }

    /// Every aircraft in the assets, which should all pass the same tests of their flight.
    const AIRCRAFT: [&str; 2] = ["uav", "flying_wing"];

    fn aircraft(name: &str) -> Simulation {
        let path = format!("{}/assets/aircraft/{name}.ron", env!("CARGO_MANIFEST_DIR"));
        let config = AircraftConfig::load(path).unwrap();
        Simulation::new(config, spawn(), Box::new(ConstantWind::default())).unwrap()
    }

    /// What the pilot does while flying with otherwise fixed sticks.
    #[derive(Clone, Copy, PartialEq)]
    enum Hands {
        Off,
        /// Roll to keep the wings level.
        Level,
        /// Keep the wings level, and pitch to hold this pitch angle in radians.
        Climb(f64),
    }

    fn fly(
        sim: &mut Simulation,
        mut sticks: config::Sticks,
        hands: Hands,
        seconds: f64,
    ) -> Sensors {
        sim.actuators.set_commands(&sim.config.mix(&sticks));
        sim.actuators.settle();
        sim.launch();
        for step in 0..(seconds * 1000.0) as usize {
            if hands != Hands::Off && step % 10 == 0 {
                let sensors = sim.sensors();
                sticks.roll = -0.5 * sensors.euler.x - 0.1 * sensors.gyro.x;
                if let Hands::Climb(pitch) = hands {
                    sticks.pitch = 2.0 * (pitch - sensors.euler.y) - 0.3 * sensors.gyro.y;
                }
                sim.actuators.set_commands(&sim.config.mix(&sticks));
            }
            sim.step(STEP);
        }
        assert_eq!(sim.status, Status::Flying);
        sim.sensors()
    }

    fn sticks(roll: f64, pitch: f64, yaw: f64, throttle: f64) -> config::Sticks {
        config::Sticks {
            roll,
            pitch,
            yaw,
            throttle,
        }
    }

    /// The center of gravity must be ahead of the neutral point, and the aircraft trimmed for
    /// a reasonable speed with its controls centered.
    #[test]
    fn statically_stable_and_trimmed() {
        for name in AIRCRAFT {
            let mut sim = aircraft(name);
            let actuators = vec![0.0; sim.actuators.names().len()];
            let mut at = |alpha: f64| {
                let alpha = f64::to_radians(alpha);
                sim.aero.reset();
                let loads = sim.aero.loads(&aero::Flow {
                    velocity: DVec3::new(-alpha.cos(), 0.0, -alpha.sin()) * 15.0,
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
            let speed = 15.0 * (sim.config.mass * 9.81 / lift).sqrt();
            println!(
                "{name}: static margin {margin:.3} m, trimmed at {trim:.1} deg and {speed:.1} m/s"
            );
            assert!(margin > 0.01, "{name}: static margin {margin}");
            assert!((10.0..20.0).contains(&speed), "{name}: trim speed {speed}");
        }
    }

    /// Gliding with the controls centered, the aircraft should settle into steady flight by
    /// itself, which needs it to be statically and dynamically stable.
    #[test]
    fn glides_hands_off() {
        for name in AIRCRAFT {
            let sensors = fly(
                &mut aircraft(name),
                sticks(0.0, 0.0, 0.0, 0.0),
                Hands::Off,
                30.0,
            );
            let (roll, pitch) = (sensors.euler.x.to_degrees(), sensors.euler.y.to_degrees());
            assert!(
                roll.abs() < 1.0 && pitch.abs() < 12.0,
                "{name}: roll {roll} pitch {pitch}"
            );
            assert!(
                (9.0..20.0).contains(&sensors.airspeed),
                "{name}: airspeed {}",
                sensors.airspeed
            );
            assert!(
                sensors.gyro.length() < 0.05,
                "{name}: still rotating {}",
                sensors.gyro
            );
            let ratio = sensors.airspeed / sensors.velocity_ned.z;
            assert!(ratio > 8.0, "{name}: glide ratio {ratio}");
        }
    }

    /// A banked glide should not steepen into a spiral dive.
    #[test]
    fn spiral_does_not_diverge() {
        for name in AIRCRAFT {
            let mut sim = aircraft(name);
            sim.body.attitude *= DQuat::from_rotation_x(10f64.to_radians());
            let sensors = fly(&mut sim, sticks(0.0, 0.0, 0.0, 0.0), Hands::Off, 30.0);
            let roll = sensors.euler.x.to_degrees();
            assert!(roll.abs() < 10.0, "{name}: roll {roll}");
        }
    }

    /// Under power, the reaction torque of the motor rolls the aircraft, which the pilot
    /// counters.
    #[test]
    fn cruises_level() {
        for name in AIRCRAFT {
            let sensors = fly(
                &mut aircraft(name),
                sticks(0.0, 0.0, 0.0, 0.6),
                Hands::Level,
                30.0,
            );
            let (roll, pitch) = (sensors.euler.x.to_degrees(), sensors.euler.y.to_degrees());
            assert!(
                roll.abs() < 3.0 && pitch.abs() < 10.0,
                "{name}: roll {roll} pitch {pitch}"
            );
            assert!(
                (10.0..22.0).contains(&sensors.airspeed),
                "{name}: airspeed {}",
                sensors.airspeed
            );
            assert!(
                sensors.velocity_ned.z.abs() < 1.5,
                "{name}: vertical speed {}",
                sensors.velocity_ned.z
            );
        }
    }

    /// Hands off, the flying wing pitches up into a loop at full throttle, so the pilot holds a
    /// climbing attitude.
    #[test]
    fn full_throttle_climbs() {
        for name in AIRCRAFT {
            let climb = Hands::Climb(10f64.to_radians());
            let sensors = fly(&mut aircraft(name), sticks(0.0, 0.0, 0.0, 1.0), climb, 20.0);
            assert!(
                sensors.velocity_ned.z < -1.0,
                "{name}: vertical speed {}",
                sensors.velocity_ned.z
            );
        }
    }

    /// Each stick turns the aircraft the right way.
    #[test]
    fn controls_respond() {
        for name in AIRCRAFT {
            let rates = |sticks| fly(&mut aircraft(name), sticks, Hands::Off, 0.2).gyro;
            let pitch = rates(sticks(0.0, 0.5, 0.0, 0.6)).y;
            let roll = rates(sticks(0.5, 0.0, 0.0, 0.6)).x;
            let yaw = rates(sticks(0.0, 0.0, 0.5, 0.6)).z;
            println!("{name}: pitch {pitch:.2} roll {roll:.2} yaw {yaw:.2} rad/s after 0.2 s");
            assert!(pitch > 0.1, "{name}: pitch rate {pitch}");
            assert!(roll > 0.2, "{name}: roll rate {roll}");
            assert!(yaw > 0.03, "{name}: yaw rate {yaw}");
        }
    }

    /// Reloading an aircraft carries on its flight, and keeps the state of what it shares with
    /// the new one.
    #[test]
    fn replaced_aircraft_flies_on() {
        let mut sim = aircraft("uav");
        fly(&mut sim, sticks(0.0, 0.0, 0.0, 0.6), Hands::Off, 1.0);
        let (body, time, rpm) = (sim.body, sim.time, sim.rotor_states[0].speed);
        let throttle = sim.actuators.positions()[2];

        let mut config = sim.config.clone();
        config.mass *= 1.1;
        config.actuators.swap(0, 1);
        sim.replace_aircraft(
            Simulation::new(config, spawn(), Box::new(ConstantWind::default())).unwrap(),
        );
        assert_eq!(sim.status, Status::Flying);
        assert_eq!((sim.body.position, sim.time), (body.position, time));
        assert_eq!(sim.rotor_states[0].speed, rpm);
        assert_eq!(sim.actuators.positions()[2], throttle);
        assert!((sim.mass - 2.2).abs() < 1e-9);
        sim.step(STEP);
        assert_eq!(sim.status, Status::Flying);
    }

    #[test]
    fn crashes_into_the_ground() {
        let mut sim = aircraft("uav");
        sim.ground = 995.0;
        sim.launch();
        for _ in 0..60_000 {
            sim.step(STEP);
        }
        assert_eq!(sim.status, Status::Crashed);
    }
}
