//! Propellers and rotors driven by brushless motors.
//!
//! The speed of each rotor is a state of its own, driven by the torque of its motor against
//! the aerodynamic torque of the propeller. This gives the throttle a realistic delay, which
//! grows with the inertia of the propeller, and the airframe the reaction torque of the motor.

use std::f64::consts::{PI, TAU};

use bevy::math::DVec3;

use super::config::{Rotor, Spin};

pub struct RotorModel {
    pub name: String,
    pub position: DVec3,
    pub axis: DVec3,
    pub diameter: f64,
    /// `1` if the rotor turns positively about its axis, `-1` otherwise.
    pub spin: f64,
    ct: f64,
    cp: f64,
    zero_thrust_advance: f64,
    pub inertia: f64,
    /// Speed constant in rad/s per volt.
    kv: f64,
    resistance: f64,
    no_load_current: f64,
    pub throttle: usize,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct RotorState {
    /// Angular speed in rad/s.
    pub speed: f64,
    /// Angle of the blades, for drawing.
    pub angle: f64,
    pub current: f64,
    pub thrust: f64,
}

pub struct RotorOutput {
    pub force: DVec3,
    /// Torque about the center of gravity, including the reaction torque of the motor.
    pub moment: DVec3,
    /// Current drawn from the battery.
    pub current: f64,
}

impl RotorModel {
    pub fn new(config: &Rotor, throttle: usize) -> Result<Self, String> {
        let axis = DVec3::from(config.axis)
            .try_normalize()
            .ok_or_else(|| format!("rotor {}: the axis must not be zero", config.name))?;
        Ok(Self {
            name: config.name.clone(),
            position: DVec3::from(config.position),
            axis,
            diameter: config.diameter,
            // Turning clockwise when looking along the axis is a positive rotation about it
            spin: match config.spin {
                Spin::Clockwise => 1.0,
                Spin::CounterClockwise => -1.0,
            },
            ct: config.ct,
            cp: config.cp,
            zero_thrust_advance: config.zero_thrust_advance,
            inertia: config.inertia,
            kv: config.motor.kv * TAU / 60.0,
            resistance: config.motor.resistance,
            no_load_current: config.motor.no_load_current,
            throttle,
        })
    }

    /// Angular momentum of the rotor in the body frame.
    pub fn momentum(&self, state: &RotorState) -> DVec3 {
        self.axis * (self.inertia * state.speed * self.spin)
    }

    /// Advance the rotor by `dt`, given its throttle, the battery voltage, and the velocity of
    /// the air relative to the rotor in the body frame.
    pub fn step(
        &self,
        state: &mut RotorState,
        throttle: f64,
        voltage: f64,
        air: DVec3,
        density: f64,
        center_of_gravity: DVec3,
        dt: f64,
    ) -> RotorOutput {
        let revolutions = state.speed / TAU;
        let d = self.diameter;

        // Moving along the thrust axis lowers the angle of attack of the blades
        let inflow = -air.dot(self.axis);
        let advance = match revolutions > 0.5 {
            true => (inflow / (revolutions * d)).max(0.0),
            false => 0.0,
        };
        let ratio = advance / self.zero_thrust_advance;
        let ct = self.ct * (1.0 - ratio).max(-0.5);
        let cp = self.cp * (1.0 - 0.6 * ratio * ratio).max(0.1);
        let thrust = ct * density * revolutions * revolutions * d.powi(4);
        let drag_torque = cp * density * revolutions * revolutions * d.powi(5) / TAU;

        // The speed controller as an ideal voltage source, which does not brake the motor
        let current = ((throttle * voltage - state.speed / self.kv) / self.resistance).max(0.0);
        let motor_torque = (current - self.no_load_current).max(0.0) / self.kv;

        state.speed = (state.speed + (motor_torque - drag_torque) / self.inertia * dt).max(0.0);
        state.angle = (state.angle + state.speed * self.spin * dt) % (2.0 * PI);
        state.current = current;
        state.thrust = thrust;

        let force = self.axis * thrust;
        let reaction = -self.axis * (self.spin * motor_torque);
        RotorOutput {
            force,
            moment: (self.position - center_of_gravity).cross(force) + reaction,
            current: current * throttle,
        }
    }
}
