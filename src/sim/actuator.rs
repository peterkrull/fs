//! Servos and speed controllers, which follow their commands with a delay and a limited rate.

use super::config::{Actuator, ActuatorKind};

pub struct Actuators {
    configs: Vec<Actuator>,
    /// Commanded positions, normalized to `-1..1` for servos and `0..1` for throttles.
    commands: Vec<f64>,
    positions: Vec<f64>,
}

impl Actuators {
    pub fn new(configs: Vec<Actuator>) -> Self {
        let n = configs.len();
        Self {
            configs,
            commands: vec![0.0; n],
            positions: vec![0.0; n],
        }
    }

    pub fn names(&self) -> Vec<String> {
        self.configs.iter().map(|c| c.name.clone()).collect()
    }

    pub fn configs(&self) -> &[Actuator] {
        &self.configs
    }

    pub fn positions(&self) -> &[f64] {
        &self.positions
    }

    pub fn commands(&self) -> &[f64] {
        &self.commands
    }

    /// Set the normalized commands, in the order of the actuators.
    pub fn set_commands(&mut self, commands: &[f64]) {
        for ((command, value), config) in self.commands.iter_mut().zip(commands).zip(&self.configs)
        {
            *command = clamp(config.kind, *value);
        }
    }

    /// Set the commands from the pulse widths of the servo outputs of a flight controller, in
    /// microseconds. Actuators without a channel, or whose channel is not driven, keep their
    /// command.
    pub fn set_pwm(&mut self, pwm: &[u16]) {
        for (command, config) in self.commands.iter_mut().zip(&self.configs) {
            let Some(&pulse) = config.channel.and_then(|c| pwm.get(c.wrapping_sub(1))) else {
                continue;
            };
            if pulse == 0 {
                continue;
            }
            let (pulse, p) = (pulse as f64, config.pwm);
            let value = match config.kind {
                ActuatorKind::Servo if pulse >= p.trim => (pulse - p.trim) / (p.max - p.trim),
                ActuatorKind::Servo => (pulse - p.trim) / (p.trim - p.min),
                ActuatorKind::Throttle => (pulse - p.min) / (p.max - p.min),
            };
            let value = match (config.reversed, config.kind) {
                (false, _) => value,
                (true, ActuatorKind::Servo) => -value,
                (true, ActuatorKind::Throttle) => 1.0 - value,
            };
            *command = clamp(config.kind, value);
        }
    }

    pub fn step(&mut self, dt: f64) {
        for ((position, command), config) in self
            .positions
            .iter_mut()
            .zip(&self.commands)
            .zip(&self.configs)
        {
            let response = (dt / config.time_constant.max(1e-6)).min(1.0);
            let max_change = config.slew_rate * dt;
            *position += ((command - *position) * response).clamp(-max_change, max_change);
        }
    }

    /// Move all actuators to their commanded positions at once.
    pub fn settle(&mut self) {
        self.positions.copy_from_slice(&self.commands);
    }
}

fn clamp(kind: ActuatorKind, value: f64) -> f64 {
    match kind {
        ActuatorKind::Servo => value.clamp(-1.0, 1.0),
        ActuatorKind::Throttle => value.clamp(0.0, 1.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::config::Pwm;

    fn servo() -> Actuator {
        Actuator {
            name: "servo".into(),
            kind: ActuatorKind::Servo,
            channel: Some(2),
            time_constant: 0.02,
            slew_rate: 10.0,
            pwm: Pwm::default(),
            reversed: false,
            input: vec![],
        }
    }

    #[test]
    fn pwm_mapping() {
        let mut actuators = Actuators::new(vec![servo()]);
        actuators.set_pwm(&[0, 1750]);
        assert_eq!(actuators.commands(), &[0.5]);
        actuators.set_pwm(&[0, 1000]);
        assert_eq!(actuators.commands(), &[-1.0]);
        // Undriven channels keep the last command
        actuators.set_pwm(&[0, 0]);
        assert_eq!(actuators.commands(), &[-1.0]);
    }

    #[test]
    fn rate_limited() {
        let mut actuators = Actuators::new(vec![servo()]);
        actuators.set_commands(&[1.0]);
        for _ in 0..50 {
            actuators.step(0.001);
        }
        // Limited by the slew rate of 10 per second for the first 50 ms
        assert!((actuators.positions()[0] - 0.5).abs() < 1e-9);
        for _ in 0..500 {
            actuators.step(0.001);
        }
        assert!((actuators.positions()[0] - 1.0).abs() < 1e-6);
    }
}
