//! Thermals: columns of rising air over the ground the sun heats, ringed by sinking air.
//!
//! The shape of a thermal follows the model of Allen (2006, "Updraft model for development of
//! autonomous soaring uninhabited air vehicles", AIAA 2006-1510): its updraft grows with the
//! cube root of the height and fades towards the top of the convective layer, and its radius
//! grows with the height too. Across the thermal, the updraft is `(1 - ρ²) exp(-ρ²)` of that at
//! its core, at a distance of `ρ` radii, which sinks outside the radius as much as it rises
//! inside, so that the air going up comes down nearby.
//!
//! Thermals come and go at random. They are placed on a grid of longitude and latitude, of
//! cells as large as the spacing between thermals, and of slots in time as long as they last.
//! Each cell has a thermal in each slot, somewhere in the cell and starting sometime in the slot.
//! They drift with the wind.

use std::f64::consts::PI;

use bevy::math::DVec2;
use serde::Deserialize;

use super::random;
use crate::geo;

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct ThermalConfig {
    /// Updraft at the core of an average thermal, at its strongest height, in m/s. Zero for
    /// no thermals.
    pub strength: f64,
    /// Height above the ground the thermals rise to, the top of the convective layer, in m.
    pub top: f64,
    /// Mean distance between thermals, in m.
    pub spacing: f64,
    /// How long a thermal lasts, in s.
    pub lifetime: f64,
}

impl Default for ThermalConfig {
    fn default() -> Self {
        Self {
            strength: 0.0,
            top: 1500.0,
            spacing: 1500.0,
            lifetime: 900.0,
        }
    }
}

/// The air sinks around a thermal out to this many radii.
const REACH: f64 = 3.0;
/// The largest value of the vertical profile `x^⅓ (1 - 1.1 x)`, at `x = 1 / 4.4`.
const PROFILE_PEAK: f64 = 0.457_8;

/// A thermal at some time.
#[derive(Debug, Clone, Copy)]
pub struct Thermal {
    /// Longitude and latitude of its core, in radians, in the air that has drifted with the
    /// wind.
    pub center: DVec2,
    /// Updraft at its core at the strongest height, in m/s, as it is at that time.
    pub strength: f64,
}

#[derive(Debug, Clone)]
pub struct Thermals {
    config: ThermalConfig,
    seed: u64,
}

impl Thermals {
    pub fn new(config: ThermalConfig, seed: u64) -> Self {
        Self {
            config: ThermalConfig {
                top: config.top.max(1.0),
                spacing: config.spacing.max(10.0),
                lifetime: config.lifetime.max(1.0),
                ..config
            },
            seed,
        }
    }

    /// Radius of the thermals at a height above the ground, in m.
    pub fn radius(&self, above_ground: f64) -> f64 {
        let ThermalConfig { top, spacing, .. } = self.config;
        let x = (above_ground / top).clamp(0.0, 1.0);
        (0.102 * x.cbrt() * top * (1.0 - 0.25 * x)).clamp(10.0, spacing / REACH)
    }

    /// The updraft at a longitude and latitude in the air that has drifted with the wind, at a
    /// height above the ground and a time, in m/s.
    pub fn updraft(&self, lon_lat: DVec2, above_ground: f64, time: f64) -> f64 {
        let x = above_ground / self.config.top;
        if !(0.0..1.0).contains(&x) {
            return 0.0;
        }
        let profile = (x.cbrt() * (1.0 - 1.1 * x)).max(0.0) / PROFILE_PEAK;
        let radius = self.radius(above_ground);
        let mut updraft = 0.0;
        for thermal in self.near(lon_lat, time) {
            let r = distance(lon_lat, thermal.center) / radius;
            if r < REACH {
                updraft += thermal.strength * (1.0 - r * r) * (-r * r).exp();
            }
        }
        updraft * profile
    }

    /// The thermals there may be within a spacing of a longitude and latitude, at a time.
    pub fn near(&self, lon_lat: DVec2, time: f64) -> impl Iterator<Item = Thermal> + '_ {
        let ThermalConfig {
            strength,
            spacing,
            lifetime,
            ..
        } = self.config;
        let cell = spacing / geo::EARTH_RADIUS;
        let row = (lon_lat.y / cell).floor() as i64;
        let slot = (time / lifetime).floor() as i64;
        (row - 1..=row + 1).flat_map(move |row| {
            // Cells as wide as they are high, however far north
            let width = cell / ((row as f64 + 0.5) * cell).cos().max(0.05);
            let column = (lon_lat.x / width).floor() as i64;
            (column - 1..=column + 1).flat_map(move |column| {
                [slot - 1, slot].into_iter().filter_map(move |slot| {
                    let mut random = random::cell(self.seed, &[row, column, slot]);
                    let center = DVec2::new(
                        (column as f64 + random.uniform()) * width,
                        (row as f64 + random.uniform()) * cell,
                    );
                    let start = (slot as f64 + random.uniform()) * lifetime;
                    let age = (time - start) / lifetime;
                    let strength = strength * random.range(0.6, 1.4);
                    (0.0..1.0).contains(&age).then(|| Thermal {
                        center,
                        strength: strength * envelope(age),
                    })
                })
            })
        })
    }
}

/// How strong a thermal is over its life: growing over its first tenth, and fading over its
/// last.
fn envelope(age: f64) -> f64 {
    let ramp = |t: f64| {
        let t = t.clamp(0.0, 1.0);
        0.5 - 0.5 * (PI * t).cos()
    };
    ramp(age / 0.1) * ramp((1.0 - age) / 0.1)
}

/// Distance between two nearby points given by their longitude and latitude, in m.
fn distance(a: DVec2, b: DVec2) -> f64 {
    let east = (a.x - b.x) * (0.5 * (a.y + b.y)).cos();
    let north = a.y - b.y;
    east.hypot(north) * geo::EARTH_RADIUS
}

#[cfg(test)]
mod tests {
    use super::*;

    fn thermals() -> Thermals {
        Thermals::new(
            ThermalConfig {
                strength: 3.0,
                ..Default::default()
            },
            9,
        )
    }

    /// Samples over an area of 30 km square, every 50 m.
    fn area(thermals: &Thermals, above_ground: f64, time: f64) -> Vec<f64> {
        let origin = DVec2::new(0.15, 0.8);
        let step = 50.0 / geo::EARTH_RADIUS;
        (0..600)
            .flat_map(|i| (0..600).map(move |j| (i, j)))
            .map(|(i, j)| {
                let at = origin + DVec2::new(i as f64 * step / origin.y.cos(), j as f64 * step);
                thermals.updraft(at, above_ground, time)
            })
            .collect()
    }

    #[test]
    fn rise_and_sink_as_much() {
        let thermals = thermals();
        let samples = area(&thermals, 400.0, 5000.0);
        let strongest = samples.iter().copied().fold(0.0, f64::max);
        let mean = samples.iter().sum::<f64>() / samples.len() as f64;
        assert!(
            strongest > 2.5 && strongest < 3.0 * 1.4 * 1.05,
            "{strongest}"
        );
        assert!(mean.abs() < 0.05, "{mean}");
        // Sinking between the thermals, though more gently than they rise
        let deepest = samples.iter().copied().fold(0.0, f64::min);
        assert!(deepest < -0.2 && deepest > -strongest, "{deepest}");
        // About one thermal per spacing squared, counting the samples in their cores
        let cores = samples.iter().filter(|w| **w > 0.5).count() as f64;
        let radius = thermals.radius(400.0);
        let expected = (30_000.0 / 1500.0f64).powi(2) * PI * (0.6 * radius / 50.0).powi(2);
        assert!(
            cores > 0.4 * expected && cores < 2.0 * expected,
            "{cores} {expected}"
        );
    }

    #[test]
    fn only_in_the_convective_layer() {
        let thermals = thermals();
        assert!(area(&thermals, 1500.0, 5000.0).iter().all(|w| *w == 0.0));
        assert!(
            area(&thermals, f64::INFINITY, 5000.0)
                .iter()
                .all(|w| *w == 0.0)
        );
        let low = area(&thermals, 20.0, 5000.0);
        let mid = area(&thermals, 400.0, 5000.0);
        let max = |v: &[f64]| v.iter().copied().fold(0.0, f64::max);
        assert!(max(&low) < max(&mid));
    }

    #[test]
    fn come_and_go() {
        let thermals = thermals();
        let before = area(&thermals, 400.0, 0.0);
        let after = area(&thermals, 400.0, 2000.0);
        // Strong updrafts in different places, more than two lifetimes apart
        let strong = |w: &f64| *w > 1.0;
        let either = before
            .iter()
            .zip(&after)
            .filter(|(a, b)| strong(a) || strong(b))
            .count();
        let both = before
            .iter()
            .zip(&after)
            .filter(|(a, b)| strong(a) && strong(b))
            .count();
        assert!(either > 1000 && both < either / 10, "{both} of {either}");
    }
}
