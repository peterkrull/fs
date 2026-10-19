//! Gravity, the atmosphere and the wind.

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

/// Density of the air in kg/m³ at a height above sea level, after the International Standard
/// Atmosphere.
pub fn air_density(height: f64) -> f64 {
    const SEA_LEVEL: f64 = 1.225;
    const TROPOPAUSE: f64 = 11_000.0;
    let temperature_ratio = |h: f64| 1.0 - 0.0065 * h / 288.15;
    if height < TROPOPAUSE {
        SEA_LEVEL * temperature_ratio(height.max(-1000.0)).powf(4.2559)
    } else {
        let tropopause = SEA_LEVEL * temperature_ratio(TROPOPAUSE).powf(4.2559);
        tropopause * (-(height - TROPOPAUSE) / 6341.6).exp()
    }
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

/// The movement of the air. Implementations may vary in space and time, to model gusts,
/// turbulence or wind shear.
pub trait WindField: Send + Sync {
    /// Velocity of the air in ECEF at an ECEF position and simulation time.
    fn velocity(&self, position: DVec3, time: f64) -> DVec3;
}

/// Wind of constant speed from a constant direction.
#[derive(Debug, Clone, Copy, Default)]
pub struct ConstantWind {
    /// Direction the wind blows from, in degrees clockwise from north.
    pub from: f64,
    /// Speed in m/s.
    pub speed: f64,
}

impl WindField for ConstantWind {
    fn velocity(&self, position: DVec3, _time: f64) -> DVec3 {
        let (sin, cos) = self.from.to_radians().sin_cos();
        let ned = DVec3::new(-cos, -sin, 0.0) * self.speed;
        ned_to_ecef(position) * ned
    }
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
        assert!((air_density(0.0) - 1.225).abs() < 1e-9);
        assert!((air_density(5000.0) - 0.7364).abs() < 0.001);
        assert!((air_density(15000.0) - 0.1948).abs() < 0.002);
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

    #[test]
    fn westerly_wind_blows_east() {
        let position = geo::to_ecef(DVec2::new(0.2, 0.7), 0.0);
        let wind = ConstantWind {
            from: 270.0,
            speed: 5.0,
        }
        .velocity(position, 0.0);
        let ned = ned_to_ecef(position).inverse() * wind;
        assert!((ned - DVec3::new(0.0, 5.0, 0.0)).length() < 1e-9, "{ned}");
    }
}
