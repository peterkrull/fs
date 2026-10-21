//! Discrete gusts: balls of faster air that come and go, and drift with the wind.
//!
//! Each gust rises and falls with the shape of one minus a cosine over its duration, like the
//! discrete gusts of MIL-HDBK-1797, and fades out smoothly towards its edge. Where and when gusts
//! come is random, from a grid in space, of cells as large as the radius of a gust, and in time,
//! of slots as long as a gust lasts. Each cell has some gusts in each slot by chance, somewhere
//! in the cell and starting sometime in the slot, so the gusts that reach a point can only come
//! from the cells next to it, in that slot or the one before.

use std::f64::consts::TAU;

use bevy::math::{DQuat, DVec3};
use serde::Deserialize;

use super::random::{self, Random};

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct GustConfig {
    /// The most a gust adds to the speed of the wind aloft, in m/s. Zero for no gusts.
    pub speed: f64,
    /// Mean time between the gusts that pass a place, in s, counting those that reach at
    /// least half their strength there.
    pub interval: f64,
    /// How long a gust takes to pass, in s.
    pub duration: f64,
    /// Radius of a gust, in m.
    pub radius: f64,
}

impl Default for GustConfig {
    fn default() -> Self {
        Self {
            speed: 0.0,
            interval: 20.0,
            duration: 4.0,
            radius: 200.0,
        }
    }
}

/// The volume over which a gust has at least half its strength, in cells: a ball of the radius
/// where the falloff `(1 - d²)²` is a half.
const REACH: f64 = 4.0 / 3.0 * std::f64::consts::PI * 0.158_5;

#[derive(Debug, Clone)]
pub struct Gusts {
    config: GustConfig,
    seed: u64,
    /// Mean number of gusts in a cell in a slot.
    density: f64,
}

impl Gusts {
    pub fn new(config: GustConfig, seed: u64) -> Self {
        let config = GustConfig {
            duration: config.duration.max(0.1),
            radius: config.radius.max(1.0),
            interval: config.interval.max(0.1),
            ..config
        };
        Self {
            density: config.duration / (config.interval * REACH),
            config,
            seed,
        }
    }

    /// The velocity the gusts add, in north-east-down, at an ECEF position in the air that has
    /// drifted with the wind, and a time. Gusts blow roughly along the wind aloft, given in
    /// north-east-down, or in any direction without wind.
    pub fn velocity(&self, position: DVec3, time: f64, wind: DVec3) -> DVec3 {
        let GustConfig {
            speed,
            duration,
            radius,
            ..
        } = self.config;
        let cell = (position / radius).floor();
        let slot = (time / duration).floor() as i64;
        let mut velocity = DVec3::ZERO;
        for neighbor in 0..27 {
            let c = cell
                + DVec3::new(
                    (neighbor % 3 - 1) as f64,
                    (neighbor / 3 % 3 - 1) as f64,
                    (neighbor / 9 - 1) as f64,
                );
            for s in [slot - 1, slot] {
                let mut random = random::cell(self.seed, &[c.x as i64, c.y as i64, c.z as i64, s]);
                let count = self.density.floor() as usize
                    + usize::from(random.uniform() < self.density.fract());
                for _ in 0..count {
                    let center =
                        (c + DVec3::new(random.uniform(), random.uniform(), random.uniform()))
                            * radius;
                    let start = (s as f64 + random.uniform()) * duration;
                    let strength = speed * random.range(0.5, 1.0);
                    let direction = direction(&mut random, wind);
                    let age = (time - start) / duration;
                    let distance = (position - center).length() / radius;
                    if (0.0..1.0).contains(&age) && distance < 1.0 {
                        let envelope = 0.5 * (1.0 - (TAU * age).cos());
                        let falloff = (1.0 - distance * distance).powi(2);
                        velocity += direction * (strength * envelope * falloff);
                    }
                }
            }
        }
        velocity
    }
}

/// Mostly along the wind, turned by up to 30 degrees, and a little up or down.
fn direction(random: &mut Random, wind: DVec3) -> DVec3 {
    let along = match wind.x.hypot(wind.y) > 0.1 {
        true => DVec3::new(wind.x, wind.y, 0.0).normalize(),
        false => DVec3::X,
    };
    let turn = match wind.x.hypot(wind.y) > 0.1 {
        true => random.range(-30.0, 30.0).to_radians(),
        false => random.range(0.0, TAU),
    };
    DQuat::from_rotation_z(turn) * along + DVec3::Z * random.range(-0.3, 0.3)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EAST: DVec3 = DVec3::new(0.0, 8.0, 0.0);

    fn gusts() -> Gusts {
        Gusts::new(
            GustConfig {
                speed: 5.0,
                ..Default::default()
            },
            3,
        )
    }

    /// The gusts at a point over time, every tenth of a second.
    fn series(gusts: &Gusts, seconds: f64) -> Vec<DVec3> {
        let position = DVec3::new(4.2e6, 6.0e5, 4.7e6);
        (0..(seconds * 10.0) as usize)
            .map(|i| gusts.velocity(position, i as f64 * 0.1, EAST))
            .collect()
    }

    #[test]
    fn come_and_go_along_the_wind() {
        let gusts = gusts();
        let series = series(&gusts, 4000.0);
        // As strong as configured, though no stronger, and continuous
        let strongest = series.iter().map(|v| v.length()).fold(0.0, f64::max);
        assert!(strongest > 3.0, "{strongest}");
        assert!(series.iter().all(|v| v.length() < 5.0 * 1.05 * 2.0));
        for pair in series.windows(2) {
            assert!((pair[1] - pair[0]).length() < 1.0);
        }
        // Mostly along the wind, from the west
        let sum: DVec3 = series.iter().sum();
        assert!(sum.y > 3.0 * sum.x.abs(), "{sum}");
        // Calm in between
        let calm = series.iter().filter(|v| v.length() < 0.01).count();
        assert!(calm > series.len() / 4, "{calm}");
        // Gusts about as often as configured, counting when they first reach half their mean
        // strength
        let arrivals = series
            .windows(2)
            .filter(|p| p[0].length() < 1.875 && p[1].length() >= 1.875)
            .count();
        let expected = 4000.0 / 20.0;
        assert!(
            arrivals as f64 > 0.7 * expected && (arrivals as f64) < 1.4 * expected,
            "{arrivals}"
        );
    }

    #[test]
    fn none_without_speed() {
        let gusts = Gusts::new(GustConfig::default(), 3);
        assert!(series(&gusts, 200.0).iter().all(|v| *v == DVec3::ZERO));
    }
}
