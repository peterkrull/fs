//! Turbulence with the von Kármán spectrum, and the intensities and scales of the low altitude
//! model of MIL-HDBK-1797.
//!
//! The turbulence is a sum of random waves, a field that is frozen in the air and drifts with the
//! wind. Each wave flows across its direction, so that the air does not compress, and their
//! strengths follow the energy spectrum of von Kármán, which gives the longitudinal and lateral
//! spectra of the same name along any line through the field.
//!
//! The scale of the turbulence shrinks towards the ground, where the eddies are smaller than
//! the height. A frozen field has a single scale, so there is one per octave of scales, each
//! scaled to unit intensity, and the field of a scale in between blends the two closest.
//! Blending independent fields by the cosine and sine of an angle keeps their intensity.

use std::f64::consts::{FRAC_PI_2, TAU};

use bevy::math::{DQuat, DVec3};

use super::random::Random;

/// Scale of the field of the first octave, in m.
const SMALLEST_SCALE: f64 = 4.0;
const OCTAVES: usize = 9;
const WAVES: usize = 64;
/// The shortest waves, which are shorter than the parts of any aircraft.
const LARGEST_WAVENUMBER: f64 = TAU / 0.5;
const FOOT: f64 = 0.3048;

#[derive(Debug, Clone)]
pub struct Turbulence {
    /// Standard deviation of the vertical gusts, in m/s.
    intensity: f64,
    /// The field of each octave of scales.
    octaves: Vec<Vec<Wave>>,
}

#[derive(Debug, Clone, Copy)]
struct Wave {
    /// Wave vector in ECEF, in rad/m.
    wave: DVec3,
    /// Velocity at the crests, across the wave vector.
    amplitude: DVec3,
    phase: f64,
}

impl Turbulence {
    pub fn new(intensity: f64, seed: u64) -> Self {
        let mut random = Random::new(seed);
        let octaves = (0..OCTAVES)
            .map(|octave| field(SMALLEST_SCALE * 2f64.powi(octave as i32), &mut random))
            .collect();
        Self { intensity, octaves }
    }

    /// The gusts in north-east-down, at an ECEF position in the air that has drifted with it,
    /// at a height above the ground.
    pub fn velocity(&self, position: DVec3, ned: DQuat, above_ground: f64) -> DVec3 {
        let scales = Scales::at(above_ground);
        let along = ned.inverse() * self.blend(position, scales.horizontal);
        let vertical = match scales.vertical == scales.horizontal {
            true => along.z,
            false => (ned.inverse() * self.blend(position, scales.vertical)).z,
        };
        let horizontal = self.intensity * scales.horizontal_intensity;
        DVec3::new(
            along.x * horizontal,
            along.y * horizontal,
            vertical * self.intensity,
        )
    }

    /// The field of unit intensity of a scale.
    fn blend(&self, position: DVec3, scale: f64) -> DVec3 {
        let octave = (scale / SMALLEST_SCALE)
            .log2()
            .clamp(0.0, (OCTAVES - 1) as f64);
        let below = (octave as usize).min(OCTAVES - 2);
        let (sin, cos) = ((octave - below as f64) * FRAC_PI_2).sin_cos();
        let mut velocity = DVec3::ZERO;
        for (waves, weight) in [(below, cos), (below + 1, sin)] {
            if weight > 1e-6 {
                velocity += sum(&self.octaves[waves], position) * weight;
            }
        }
        velocity
    }
}

fn sum(waves: &[Wave], position: DVec3) -> DVec3 {
    waves
        .iter()
        .map(|w| w.amplitude * (w.wave.dot(position) + w.phase).cos())
        .sum()
}

/// Random waves of a field of unit intensity in each direction, with the von Kármán energy
/// spectrum of an integral scale in m.
fn field(scale: f64, random: &mut Random) -> Vec<Wave> {
    // Logarithmically spaced wavenumbers, from well below the peak of the spectrum
    let smallest = 0.1 / scale;
    let ratio = (LARGEST_WAVENUMBER / smallest).ln() / WAVES as f64;
    let mut waves: Vec<(f64, Wave)> = (0..WAVES)
        .map(|i| {
            let k = smallest * ((i as f64 + random.uniform()) * ratio).exp();
            let kl = 1.339 * k * scale;
            let energy = kl.powi(4) / (1.0 + kl * kl).powf(17.0 / 6.0) * k * ratio;
            let direction = random.direction();
            let across = direction.cross(random.direction()).normalize();
            let wave = Wave {
                wave: direction * k,
                amplitude: across,
                phase: random.range(0.0, TAU),
            };
            (energy, wave)
        })
        .collect();
    // Each wave has a mean square velocity of half its amplitude squared. A unit intensity in
    // each of three directions makes them add up to 3.
    let total: f64 = waves.iter().map(|(energy, _)| energy).sum();
    for (energy, wave) in &mut waves {
        wave.amplitude *= (6.0 * *energy / total).sqrt();
    }
    waves.into_iter().map(|(_, wave)| wave).collect()
}

/// The scales and intensities of the turbulence at a height above the ground, after the low
/// altitude model of MIL-HDBK-1797 below 1000 ft, its medium and high altitude scale above
/// 2000 ft, and a blend in between.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Scales {
    /// Integral scale of the horizontal gusts, in m.
    horizontal: f64,
    /// Integral scale of the vertical gusts, in m.
    vertical: f64,
    /// Intensity of the horizontal gusts relative to the vertical ones.
    horizontal_intensity: f64,
}

impl Scales {
    fn at(above_ground: f64) -> Self {
        let h = above_ground.max(10.0 * FOOT) / FOOT;
        if h <= 1000.0 {
            let ratio = 0.177 + 0.000823 * h;
            Self {
                horizontal: h * FOOT / ratio.powf(1.2),
                vertical: h * FOOT,
                horizontal_intensity: 1.0 / ratio.powf(0.4),
            }
        } else {
            let t = ((h - 1000.0) / 1000.0).min(1.0);
            let scale = (1000.0 + t * 750.0) * FOOT;
            Self {
                horizontal: scale,
                vertical: scale,
                horizontal_intensity: 1.0,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use bevy::math::DVec2;

    use super::*;
    use crate::{geo, sim::environment::ned_to_ecef_at};

    /// The mean square velocity of each direction, along a long line through the field.
    fn mean_square(turbulence: &Turbulence, above_ground: f64) -> DVec3 {
        let lon_lat = DVec2::new(0.1, 0.8);
        let ned = ned_to_ecef_at(lon_lat);
        let start = geo::to_ecef(lon_lat, 500.0);
        let n = 200_000;
        (0..n)
            .map(|i| {
                let v = turbulence.velocity(start + ned * DVec3::X * (i as f64), ned, above_ground);
                v * v
            })
            .sum::<DVec3>()
            / n as f64
    }

    #[test]
    fn intensity_at_altitude() {
        let turbulence = Turbulence::new(1.5, 7);
        let rms = mean_square(&turbulence, 1000.0).map(f64::sqrt);
        // Random waves make each direction differ a little from the intensity
        for v in rms.to_array() {
            assert!((v / 1.5 - 1.0).abs() < 0.25, "{rms}");
        }
        assert!(
            (rms.length() / (1.5 * 3f64.sqrt()) - 1.0).abs() < 0.12,
            "{rms}"
        );
    }

    #[test]
    fn stronger_horizontal_gusts_near_the_ground() {
        let turbulence = Turbulence::new(1.0, 11);
        let rms = mean_square(&turbulence, 20.0).map(f64::sqrt);
        let horizontal = (rms.x * rms.x + rms.y * rms.y).sqrt() / 2f64.sqrt();
        assert!(horizontal > 1.4 * rms.z, "{rms}");
    }

    #[test]
    fn scales_after_the_handbook() {
        let at = |ft: f64| Scales::at(ft * FOOT);
        // L_w = h and L_u = h / (0.177 + 0.000823 h)^1.2 below 1000 ft
        let low = at(100.0);
        assert!((low.vertical / FOOT - 100.0).abs() < 1e-9);
        assert!((low.horizontal / FOOT - 100.0 / 0.2593f64.powf(1.2)).abs() < 0.1);
        // Continuous at 1000 ft, and 1750 ft above 2000 ft
        let top = at(1000.0);
        assert!((top.horizontal / FOOT - 1000.0).abs() < 0.5);
        assert!((top.horizontal_intensity - 1.0).abs() < 1e-3);
        assert!((at(3000.0).horizontal / FOOT - 1750.0).abs() < 1e-9);
        assert_eq!(at(0.0), at(10.0));
    }

    /// The longitudinal correlation of von Kármán turbulence, averaged over a few fields, as a
    /// single field of random waves differs from it by chance.
    #[test]
    fn longitudinal_correlation() {
        let lon_lat = DVec2::new(0.1, 0.8);
        let ned = ned_to_ecef_at(lon_lat);
        let start = geo::to_ecef(lon_lat, 2000.0);
        let fields: Vec<Turbulence> = (1..=4).map(|seed| Turbulence::new(1.0, seed)).collect();
        let correlation = |lag: f64| {
            let n = 50_000;
            let mut sums = (0.0, 0.0);
            for turbulence in &fields {
                let along = |x: f64| {
                    turbulence
                        .velocity(start + ned * DVec3::X * x, ned, 2000.0)
                        .x
                };
                for i in 0..n {
                    let x = i as f64 * 6.0;
                    sums.0 += along(x) * along(x + lag);
                    sums.1 += along(x) * along(x);
                }
            }
            sums.0 / sums.1
        };
        // 2^(2/3) / Γ(1/3) ξ^(1/3) K_1/3(ξ), with ξ = r / (1.339 L) and L = 1750 ft
        for (lag, expected) in [(100.0, 0.749), (533.0, 0.347), (2000.0, 0.037)] {
            let actual = correlation(lag);
            assert!((actual - expected).abs() < 0.1, "{lag} m: {actual}");
        }
    }
}
