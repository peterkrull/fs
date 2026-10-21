//! Reproducible randomness for the weather: a stream of numbers from a seed, and numbers hashed
//! from where they are needed, so that any part of the weather can be found without generating
//! what comes before it.

use bevy::math::DVec3;

/// SplitMix64, which is fast, and good enough for the weather.
#[derive(Debug, Clone)]
pub struct Random(u64);

impl Random {
    pub fn new(seed: u64) -> Self {
        Self(seed)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        mix(self.0)
    }

    /// Uniform in `0..1`.
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn range(&mut self, min: f64, max: f64) -> f64 {
        min + (max - min) * self.uniform()
    }

    /// Uniform over the directions.
    pub fn direction(&mut self) -> DVec3 {
        let z = self.range(-1.0, 1.0);
        let (sin, cos) = self.range(0.0, std::f64::consts::TAU).sin_cos();
        let r = (1.0 - z * z).sqrt();
        DVec3::new(r * cos, r * sin, z)
    }
}

/// A stream of random numbers for a seed and the coordinates of a cell, such as of a grid in
/// space and time.
pub fn cell(seed: u64, coordinates: &[i64]) -> Random {
    let mut state = mix(seed ^ 0x5851_f42d_4c95_7f2d);
    for c in coordinates {
        state = mix(state ^ (*c as u64).wrapping_mul(0x2545_f491_4f6c_dd1d));
    }
    Random::new(state)
}

fn mix(mut z: u64) -> u64 {
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uniform_in_range() {
        let mut random = Random::new(3);
        let values: Vec<f64> = (0..10_000).map(|_| random.uniform()).collect();
        assert!(values.iter().all(|v| (0.0..1.0).contains(v)));
        let mean = values.iter().sum::<f64>() / values.len() as f64;
        assert!((mean - 0.5).abs() < 0.01, "{mean}");
    }

    #[test]
    fn cells_differ_and_repeat() {
        let a = cell(1, &[2, 3]).next_u64();
        assert_eq!(a, cell(1, &[2, 3]).next_u64());
        assert_ne!(a, cell(1, &[3, 2]).next_u64());
        assert_ne!(a, cell(2, &[2, 3]).next_u64());
    }
}
