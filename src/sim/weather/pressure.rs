//! Areas of high and low pressure, and the wind that blows around them.
//!
//! The pressure at sea level is that of the standard atmosphere, plus a bell shaped bump or dip
//! for each pressure system, which may move. Away from the ground, where friction does not slow
//! it down, the wind blows along the isobars, as fast as the Coriolis effect balances the
//! pressure gradient: around lows counterclockwise in the northern hemisphere, and clockwise in
//! the southern. That is the geostrophic wind. It overestimates the wind around tight lows, so
//! it is limited, and it vanishes at the equator, so the Coriolis effect is taken to be at
//! least that at 15 degrees of latitude.

use bevy::math::{DQuat, DVec2, DVec3};
use serde::Deserialize;

use crate::geo;

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PressureSystem {
    /// Where the center is at the start, in degrees.
    pub lat: f64,
    pub lon: f64,
    /// How much higher the pressure at the center is than around it, in hPa: positive for a
    /// high, and negative for a low.
    pub pressure: f64,
    /// Distance from the center at which the pressure changes the fastest, in km.
    pub radius: f64,
    /// Direction the system moves towards, in degrees clockwise from north.
    pub course: f64,
    /// Speed the system moves at, in m/s.
    pub speed: f64,
}

impl Default for PressureSystem {
    fn default() -> Self {
        Self {
            lat: 0.0,
            lon: 0.0,
            pressure: -10.0,
            radius: 300.0,
            course: 0.0,
            speed: 0.0,
        }
    }
}

/// Angular velocity of the rotation of the earth, in rad/s.
const EARTH_ROTATION: f64 = 7.292_115e-5;
/// The geostrophic wind is limited to below this, in m/s.
const MAX_WIND: f64 = 50.0;

impl PressureSystem {
    /// Where the center is at a time.
    fn center(&self, time: f64) -> DVec2 {
        let (sin, cos) = self.course.to_radians().sin_cos();
        let distance = self.speed * time / geo::EARTH_RADIUS;
        let lat = self.lat.to_radians();
        DVec2::new(
            self.lon.to_radians() + distance * sin / lat.cos().max(0.01),
            lat + distance * cos,
        )
    }

    /// The pressure it adds at a position on the surface, given by the direction up from the
    /// center of the earth, in Pa, and its gradient in north-east-down, in Pa/m.
    fn at(&self, up: DVec3, ned: DQuat, time: f64) -> (f64, DVec3) {
        let center = geo::up(self.center(time));
        let distance = up.angle_between(center) * geo::EARTH_RADIUS;
        let radius = self.radius.max(1.0) * 1000.0;
        let pressure = self.pressure * 100.0 * (-0.5 * (distance / radius).powi(2)).exp();
        // Along the surface away from the center
        let away = ned.inverse() * (up * up.dot(center) - center);
        let away = DVec3::new(away.x, away.y, 0.0).normalize_or_zero();
        (pressure, away * (-pressure * distance / (radius * radius)))
    }
}

/// The pressure the systems add at sea level, in Pa, and their geostrophic wind, in
/// north-east-down, at a probe, for a density of the air at sea level.
pub fn field(
    systems: &[PressureSystem],
    lon_lat: DVec2,
    ned: DQuat,
    time: f64,
    density: f64,
) -> (f64, DVec3) {
    if systems.is_empty() {
        return (0.0, DVec3::ZERO);
    }
    let up = geo::up(lon_lat);
    let (pressure, gradient) = systems
        .iter()
        .map(|s| s.at(up, ned, time))
        .fold((0.0, DVec3::ZERO), |(p, g), (sp, sg)| (p + sp, g + sg));
    let latitude = lon_lat.y;
    let coriolis = 2.0
        * EARTH_ROTATION
        * latitude.sin().abs().max(15f64.to_radians().sin())
        * if latitude < 0.0 { -1.0 } else { 1.0 };
    // Up cross the gradient, over the density and the Coriolis parameter
    let wind = DVec3::new(gradient.y, -gradient.x, 0.0) / (density * coriolis);
    let speed = wind.length();
    let limited = match speed > 1e-9 {
        true => wind * (MAX_WIND * (speed / MAX_WIND).tanh() / speed),
        false => DVec3::ZERO,
    };
    (pressure, limited)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::environment::ned_to_ecef_at;

    fn low(lat: f64) -> PressureSystem {
        PressureSystem {
            lat,
            lon: 10.0,
            pressure: -20.0,
            radius: 300.0,
            ..Default::default()
        }
    }

    fn at(systems: &[PressureSystem], lat: f64, lon: f64, time: f64) -> (f64, DVec3) {
        let lon_lat = DVec2::new(lon.to_radians(), lat.to_radians());
        field(systems, lon_lat, ned_to_ecef_at(lon_lat), time, 1.225)
    }

    #[test]
    fn deepest_at_the_center() {
        let (pressure, wind) = at(&[low(50.0)], 50.0, 10.0, 0.0);
        assert!((pressure + 2000.0).abs() < 1e-6);
        assert!(wind.length() < 1e-6);
        // Half way to the radius, the pressure is up by an eighth of the depth
        let (pressure, _) = at(&[low(50.0)], 50.0 + 150.0 / 111.195, 10.0, 0.0);
        assert!(
            (pressure / -2000.0 - (-0.125f64).exp()).abs() < 1e-3,
            "{pressure}"
        );
    }

    #[test]
    fn counterclockwise_around_lows_in_the_north() {
        // South of the low, the wind blows east
        let lat = 50.0 - 300.0 / 111.195;
        let (_, wind) = at(&[low(50.0)], lat, 10.0, 0.0);
        assert!(wind.y > 10.0 && wind.x.abs() < 0.05 * wind.y, "{wind}");
        // Close to the geostrophic wind: 2000 Pa e^-½ / 300 km / (1.225 f)
        let f = 2.0 * EARTH_ROTATION * lat.to_radians().sin();
        let expected = 2000.0 * (-0.5f64).exp() / 300e3 / (1.225 * f);
        let limited = MAX_WIND * (expected / MAX_WIND).tanh();
        assert!((wind.y - limited).abs() < 0.2, "{wind} {limited}");

        // And clockwise in the south: south of a southern low, the wind blows west
        let (_, wind) = at(&[low(-50.0)], -50.0 - 300.0 / 111.195, 10.0, 0.0);
        assert!(wind.y < -10.0, "{wind}");

        // Clockwise around a northern high, blowing west south of it
        let high = PressureSystem {
            pressure: 20.0,
            ..low(50.0)
        };
        let (_, wind) = at(&[high], lat, 10.0, 0.0);
        assert!(wind.y < -10.0, "{wind}");
    }

    #[test]
    fn moves_along_its_course() {
        let moving = PressureSystem {
            course: 90.0,
            speed: 10.0,
            ..low(50.0)
        };
        // A day later, 864 km further east
        let center = moving.center(86_400.0);
        let east = (center.x - 10f64.to_radians()) * 50f64.to_radians().cos() * geo::EARTH_RADIUS;
        assert!((east - 864e3).abs() < 1.0, "{east}");
        assert!((center.y - 50f64.to_radians()).abs() < 1e-12);
    }
}
