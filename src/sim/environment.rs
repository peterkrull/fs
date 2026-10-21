//! Gravity and the standard atmosphere. The wind is in [`super::weather`].

use bevy::math::{DMat3, DQuat, DVec2, DVec3};

use crate::geo;

/// Standard gravity at the surface of the spherical earth.
const SURFACE_GRAVITY: f64 = 9.80665;

/// Gravitational acceleration in ECEF at an ECEF position.
pub fn gravity(position: DVec3) -> DVec3 {
    let r = position.length();
    let mu = SURFACE_GRAVITY * geo::EARTH_RADIUS * geo::EARTH_RADIUS;
    -position * (mu / (r * r * r))
}

/// The state of the air.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Air {
    /// Static pressure in Pa.
    pub pressure: f64,
    /// Temperature in K.
    pub temperature: f64,
    /// Density in kg/m³.
    pub density: f64,
}

/// Specific gas constant of dry air, in J/(kg K).
const GAS_CONSTANT: f64 = 287.053;
const SEA_LEVEL_PRESSURE: f64 = 101_325.0;
const SEA_LEVEL_TEMPERATURE: f64 = 288.15;

/// The air at a height above sea level, after the International Standard Atmosphere, with a
/// pressure in Pa and temperature in K at sea level, which may differ from the standard ones.
/// The temperature falls by 6.5 K per km up to the tropopause at 11 km, and stays the same above.
pub fn standard_air(height: f64, sea_level_pressure: f64, sea_level_temperature: f64) -> Air {
    const LAPSE_RATE: f64 = 0.0065;
    const TROPOPAUSE: f64 = 11_000.0;
    let exponent = SURFACE_GRAVITY / (GAS_CONSTANT * LAPSE_RATE);
    let troposphere = |h: f64| {
        let temperature = sea_level_temperature - LAPSE_RATE * h;
        let pressure = sea_level_pressure * (temperature / sea_level_temperature).powf(exponent);
        (pressure, temperature)
    };
    let (pressure, temperature) = match height < TROPOPAUSE {
        true => troposphere(height.max(-1000.0)),
        false => {
            let (pressure, temperature) = troposphere(TROPOPAUSE);
            let scale = GAS_CONSTANT * temperature / SURFACE_GRAVITY;
            (
                pressure * (-(height - TROPOPAUSE) / scale).exp(),
                temperature,
            )
        }
    };
    Air {
        pressure,
        temperature,
        density: pressure / (GAS_CONSTANT * temperature),
    }
}

/// Density of the air in kg/m³ at a height above sea level, after the International Standard
/// Atmosphere.
pub fn air_density(height: f64) -> f64 {
    standard_air(height, SEA_LEVEL_PRESSURE, SEA_LEVEL_TEMPERATURE).density
}

/// Rotation from the local north-east-down frame at an ECEF position to ECEF.
pub fn ned_to_ecef(position: DVec3) -> DQuat {
    let (lon_lat, _) = geo::from_ecef(position);
    ned_to_ecef_at(lon_lat)
}

pub fn ned_to_ecef_at(lon_lat: DVec2) -> DQuat {
    let up = geo::up(lon_lat);
    let east = DVec3::new(-lon_lat.x.sin(), lon_lat.x.cos(), 0.0);
    let north = up.cross(east);
    DQuat::from_mat3(&DMat3::from_cols(north, east, -up)).normalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gravity_at_surface() {
        let g = gravity(DVec3::new(0.0, 0.0, geo::EARTH_RADIUS));
        assert!((g - DVec3::new(0.0, 0.0, -SURFACE_GRAVITY)).length() < 1e-9);
    }

    #[test]
    fn density() {
        assert!((air_density(0.0) - 1.225).abs() < 1e-5);
        assert!((air_density(5000.0) - 0.7364).abs() < 0.001);
        assert!((air_density(15000.0) - 0.1948).abs() < 0.002);
    }

    #[test]
    fn warmer_air_is_thinner() {
        let standard = standard_air(1000.0, 101_325.0, 288.15);
        assert!((standard.pressure - 89_875.0).abs() < 5.0, "{standard:?}");
        assert!((standard.temperature - 281.65).abs() < 1e-9);
        let warm = standard_air(1000.0, 101_325.0, 308.15);
        assert!(warm.density < 0.95 * standard.density);
        // Lower pressure at sea level is lower pressure aloft too
        let low = standard_air(1000.0, 99_000.0, 288.15);
        assert!((low.pressure / standard.pressure - 99_000.0 / 101_325.0).abs() < 1e-9);
    }

    #[test]
    fn ned_frame() {
        let lon_lat = DVec2::new(0.3, 0.8);
        let q = ned_to_ecef_at(lon_lat);
        let down = q * DVec3::Z;
        assert!((down + geo::up(lon_lat)).length() < 1e-12);
        // North points towards the pole
        assert!((q * DVec3::X).z > 0.0);
    }
}
