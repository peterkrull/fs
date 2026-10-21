//! Propellers and rotors driven by brushless motors or piston engines.
//!
//! The speed of each rotor is a state of its own, driven by the torque of its motor against
//! the aerodynamic torque of the propeller. This gives the throttle a realistic delay, which
//! grows with the inertia of the propeller, and the airframe the reaction torque of the motor.

use std::f64::consts::{PI, TAU};

use bevy::math::DVec3;

use super::{
    aero::Slipstream,
    config::{Rotor, Spin},
};

/// Thrust and power coefficients by advance ratio.
enum Coefficients {
    /// Thrust falling linearly to zero at `zero_thrust_advance`, and power quadratically.
    Parametric {
        ct: f64,
        cp: f64,
        zero_thrust_advance: f64,
    },
    Table {
        advance: Vec<f64>,
        ct: Vec<f64>,
        cp: Vec<f64>,
    },
}

impl Coefficients {
    fn at(&self, advance: f64) -> (f64, f64) {
        match self {
            Coefficients::Parametric {
                ct,
                cp,
                zero_thrust_advance,
            } => {
                let ratio = advance.max(0.0) / zero_thrust_advance;
                (
                    ct * (1.0 - ratio).max(-0.5),
                    cp * (1.0 - 0.6 * ratio * ratio).max(0.1),
                )
            }
            Coefficients::Table {
                advance: js,
                ct,
                cp,
            } => {
                let clamped = advance.clamp(js[0], js[js.len() - 1]);
                let i = js.partition_point(|j| *j <= clamped).clamp(1, js.len() - 1);
                let t = (clamped - js[i - 1]) / (js[i] - js[i - 1]);
                let lerp = |v: &[f64]| v[i - 1] + (v[i] - v[i - 1]) * t;
                (lerp(ct), lerp(cp))
            }
        }
    }
}

/// What turns the rotor.
enum Drive {
    /// A brushless motor, with its speed controller as an ideal voltage source.
    Electric {
        /// Speed constant in rad/s per volt.
        kv: f64,
        resistance: f64,
        no_load_current: f64,
    },
    /// A piston engine, with its torque at full throttle and its rated speed in rad/s.
    Piston {
        torque: f64,
        speed: f64,
        friction: f64,
    },
}

pub struct RotorModel {
    pub name: String,
    pub position: DVec3,
    pub axis: DVec3,
    pub diameter: f64,
    /// `1` if the rotor turns positively about its axis, `-1` otherwise.
    pub spin: f64,
    coefficients: Coefficients,
    pub inertia: f64,
    drive: Drive,
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
    /// Torque of the motor or engine on the rotor, in N m.
    pub torque: f64,
}

pub struct RotorOutput {
    pub force: DVec3,
    /// Torque about the center of gravity, including the reaction torque of the motor.
    pub moment: DVec3,
    /// Current drawn from the battery.
    pub current: f64,
    pub slipstream: Slipstream,
}

impl RotorModel {
    pub fn new(config: &Rotor, throttle: usize) -> Result<Self, String> {
        let error = |message: &str| format!("rotor {}: {message}", config.name);
        let axis = DVec3::from(config.axis)
            .try_normalize()
            .ok_or_else(|| error("the axis must not be zero"))?;
        let coefficients = match &config.table {
            Some(table) => {
                let n = table.advance.len();
                if n < 2 || table.ct.len() != n || table.cp.len() != n {
                    return Err(error(
                        "propeller tables need at least two rows of equal length",
                    ));
                }
                if table.advance.windows(2).any(|w| w[1] <= w[0]) {
                    return Err(error("propeller table advance ratios must be increasing"));
                }
                Coefficients::Table {
                    advance: table.advance.clone(),
                    ct: table.ct.clone(),
                    cp: table.cp.clone(),
                }
            }
            None if config.ct > 0.0 && config.cp > 0.0 && config.zero_thrust_advance > 0.0 => {
                Coefficients::Parametric {
                    ct: config.ct,
                    cp: config.cp,
                    zero_thrust_advance: config.zero_thrust_advance,
                }
            }
            None => return Err(error("needs ct, cp and zero_thrust_advance, or a table")),
        };
        let drive = match (&config.motor, &config.engine) {
            (Some(motor), None) => Drive::Electric {
                kv: motor.kv * TAU / 60.0,
                resistance: motor.resistance,
                no_load_current: motor.no_load_current,
            },
            (None, Some(engine)) => {
                let speed = engine.rpm * TAU / 60.0;
                Drive::Piston {
                    torque: engine.power / speed,
                    speed,
                    friction: engine.friction,
                }
            }
            _ => return Err(error("needs either a motor or an engine")),
        };
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
            coefficients,
            inertia: config.inertia,
            drive,
            throttle,
        })
    }

    /// Angular momentum of the rotor in the body frame.
    pub fn momentum(&self, state: &RotorState) -> DVec3 {
        self.axis * (self.inertia * state.speed * self.spin)
    }

    /// The current the rotor draws from the battery at a battery voltage `v`, as `a v - b` where
    /// that is positive, given as `(a, b)`, or `None` if it does not draw from the battery. The
    /// speed controller passes the throttle times the battery voltage on to the motor, and draws
    /// the throttle times the current of the motor.
    pub fn battery_load(&self, state: &RotorState, throttle: f64) -> Option<(f64, f64)> {
        match self.drive {
            Drive::Electric { kv, resistance, .. } => Some((
                throttle * throttle / resistance,
                throttle * state.speed / kv / resistance,
            )),
            Drive::Piston { .. } => None,
        }
    }

    /// Advance the rotor by `dt`, given its throttle, the battery voltage, the velocity of the
    /// air relative to the rotor in the body frame, and its density.
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
            true => inflow / (revolutions * d),
            false => 0.0,
        };
        let (ct, cp) = self.coefficients.at(advance);
        let thrust = ct * density * revolutions * revolutions * d.powi(4);
        // Negative where the air drives the propeller, as it windmills
        let drag_torque = cp * density * revolutions * revolutions * d.powi(5) / TAU;

        let (motor_torque, current) = match self.drive {
            // The speed controller as an ideal voltage source, which does not brake the motor
            Drive::Electric {
                kv,
                resistance,
                no_load_current,
            } => {
                let current = ((throttle * voltage - state.speed / kv) / resistance).max(0.0);
                ((current - no_load_current).max(0.0) / kv, current)
            }
            Drive::Piston {
                torque,
                speed,
                friction,
            } => {
                // The brake power at full throttle and rated speed falls with the density of the
                // air, after Gagg and Ferrar. The friction, which comes off the power the
                // cylinders make, grows with the speed instead.
                let charge = (1.132 * density / 1.225 - 0.132).max(0.0);
                let indicated = throttle * torque * (charge + friction);
                (indicated - torque * friction * state.speed / speed, 0.0)
            }
        };

        state.speed = (state.speed + (motor_torque - drag_torque) / self.inertia * dt).max(0.0);
        state.angle = (state.angle + state.speed * self.spin * dt) % (2.0 * PI);
        state.current = current;
        state.thrust = thrust;
        state.torque = motor_torque;

        // After momentum theory, the rotor speeds up the air through its disc by the velocity
        // that carries the thrust away as momentum
        let area = PI * d * d / 4.0;
        let axial = inflow.max(0.0);
        let induced = match thrust > 0.0 {
            true => 0.5 * (-axial + (axial * axial + 2.0 * thrust / (density * area)).sqrt()),
            false => 0.0,
        };

        let force = self.axis * thrust;
        let reaction = -self.axis * (self.spin * motor_torque);
        RotorOutput {
            force,
            moment: (self.position - center_of_gravity).cross(force) + reaction,
            current: current * throttle,
            slipstream: Slipstream {
                position: self.position,
                axis: self.axis,
                radius: d / 2.0,
                induced,
                inflow: axial,
            },
        }
    }
}
/// The voltage of a battery, of an open circuit voltage and an internal resistance, under
/// loads that each draw a current of `a v - b`, where that is positive, at a voltage `v`, as
/// [`RotorModel::battery_load`] gives them. The voltage falls the more current the loads draw,
/// and they draw more the higher it is, so a single voltage balances the two.
pub fn battery_voltage(open: f64, resistance: f64, loads: &[(f64, f64)]) -> f64 {
    let current = |v: f64| loads.iter().map(|(a, b)| (a * v - b).max(0.0)).sum::<f64>();
    let (mut low, mut high) = (0.0, open.max(0.0));
    for _ in 0..60 {
        let mid = 0.5 * (low + high);
        match mid > open - resistance * current(mid) {
            true => high = mid,
            false => low = mid,
        }
    }
    0.5 * (low + high)
}
