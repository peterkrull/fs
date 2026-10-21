//! The weather: wind that varies in space and time.
//!
//! The weather is a pure function of where and when it is sampled. It keeps no state that the
//! simulation advances, so that sampling it twice gives the same air, and two simulations built
//! from the same [`WeatherConfig`] fly through the same weather. What looks random about it comes
//! from the seed of its configuration.
//!
//! The wind is made up of layers that add up:
//! - the wind aloft: a wind that is the same everywhere, and that around the areas of high and
//!   low pressure, see [`pressure`], which the boundary layer slows down and turns near the
//!   ground,
//! - discrete gusts, which add to the wind aloft, see [`gusts`],
//! - ridge lift, where the wind blows up or down the slope of the terrain,
//! - thermals, see [`thermals`],
//! - turbulence, see [`turbulence`].
//!
//! The pressure systems also change the pressure, and so the density, of the air.

mod gusts;
mod pressure;
mod random;
mod thermals;
mod turbulence;

use std::path::Path;

use bevy::math::{DQuat, DVec2, DVec3};
use serde::Deserialize;

use super::environment::{Air, ned_to_ecef_at, standard_air};
use crate::geo;
pub use gusts::GustConfig;
use gusts::Gusts;
pub use pressure::PressureSystem;
pub use thermals::ThermalConfig;
use thermals::Thermals;
use turbulence::Turbulence;

/// What the weather is like, as read from a RON file. Everything has a default, so that `()` is
/// calm air.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct WeatherConfig {
    /// Seeds the random parts of the weather: the same seed gives the same weather.
    pub seed: u64,
    /// The wind above the boundary layer.
    pub wind: Wind,
    pub boundary_layer: BoundaryLayer,
    /// Standard deviation of the vertical gusts of the turbulence, in m/s. The horizontal
    /// gusts are as strong, and stronger near the ground. Light turbulence is about 0.75,
    /// moderate 1.5 and severe 2.3, see [`turbulence_intensity`].
    pub turbulence: f64,
    pub gusts: GustConfig,
    pub sea_level: SeaLevel,
    pub pressure_systems: Vec<PressureSystem>,
    pub thermals: ThermalConfig,
    pub ridge_lift: RidgeLift,
}

/// The wind blowing up the slopes of the terrain, which lifts the air near the ground. Only the
/// slope of the terrain right below is known, so the lift is that of the wind following the
/// slope there, fading with the height above it.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct RidgeLift {
    /// Distance over which the slope of the terrain is measured, and the height above the
    /// terrain over which its lift fades, in m. Zero for no ridge lift.
    pub scale: f64,
}

impl Default for RidgeLift {
    fn default() -> Self {
        Self { scale: 150.0 }
    }
}

/// The air at sea level, away from the pressure systems.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct SeaLevel {
    /// Pressure in hPa.
    pub pressure: f64,
    /// Temperature in °C.
    pub temperature: f64,
}

impl Default for SeaLevel {
    fn default() -> Self {
        Self {
            pressure: 1013.25,
            temperature: 15.0,
        }
    }
}

/// The intensity of turbulence of a severity: `light`, `moderate` or `severe`, or a number in
/// m/s. These are the low altitude intensities of MIL-HDBK-1797, of a wind of 15, 30 and 45
/// knots at 20 ft.
pub fn turbulence_intensity(severity: &str) -> Result<f64, String> {
    match severity {
        "light" => Ok(0.77),
        "moderate" => Ok(1.54),
        "severe" => Ok(2.31),
        _ => severity
            .parse::<f64>()
            .ok()
            .filter(|v| *v >= 0.0)
            .ok_or_else(|| format!("unknown turbulence {severity}")),
    }
}

impl WeatherConfig {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path).map_err(|err| format!("{path:?}: {err}"))?;
        Self::parse(&text).map_err(|err| format!("{path:?}: {err}"))
    }

    pub fn parse(text: &str) -> Result<Self, String> {
        ron::Options::default()
            .with_default_extension(ron::extensions::Extensions::IMPLICIT_SOME)
            .from_str(text)
            .map_err(|err| err.to_string())
    }
}

/// Wind of a speed from a direction.
#[derive(Debug, Clone, Copy, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Wind {
    /// Direction the wind blows from, in degrees clockwise from north.
    pub from: f64,
    /// Speed in m/s.
    pub speed: f64,
}

impl Wind {
    /// Velocity of the air in north-east-down.
    pub fn ned(&self) -> DVec3 {
        let (sin, cos) = self.from.to_radians().sin_cos();
        DVec3::new(-cos, -sin, 0.0) * self.speed
    }
}

/// The layer of air the terrain slows down by friction. Its wind grows with the logarithm of
/// the height, and turns towards low pressure near the ground: to the left of the wind aloft in
/// the northern hemisphere, and to the right in the southern.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct BoundaryLayer {
    /// Height above the terrain where the wind reaches its speed aloft, in m. Zero for no
    /// boundary layer.
    pub height: f64,
    /// Roughness length of the terrain, in m: 0.0002 for open water, 0.03 for grass, 0.1 for
    /// farmland, 0.5 for scattered trees and buildings, and 1 or more for forests and cities.
    pub roughness: f64,
    /// How much the wind at the ground is turned from the wind aloft, in degrees.
    pub turning: f64,
}

impl Default for BoundaryLayer {
    fn default() -> Self {
        Self {
            height: 300.0,
            roughness: 0.1,
            turning: 20.0,
        }
    }
}

impl BoundaryLayer {
    /// The wind at a height above the terrain, for a wind aloft in north-east-down, and the
    /// sign of the latitude.
    fn apply(&self, aloft: DVec3, above_ground: f64, latitude: f64) -> DVec3 {
        if self.height <= 0.0 || above_ground >= self.height {
            return aloft;
        }
        let roughness = self.roughness.clamp(1e-5, self.height * 0.5);
        let speed = ((above_ground.max(roughness) / roughness).ln()
            / (self.height / roughness).ln())
        .clamp(0.0, 1.0);
        // Turning towards the left, counterclockwise seen from above, in the north
        let turning = self.turning.to_radians()
            * (1.0 - above_ground / self.height).clamp(0.0, 1.0)
            * latitude.signum();
        let horizontal = DQuat::from_rotation_z(-turning) * DVec3::new(aloft.x, aloft.y, 0.0);
        (horizontal + DVec3::Z * aloft.z) * speed
    }
}

/// The terrain below where the weather is sampled.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ground {
    /// Height above sea level in m, negative infinity where unknown.
    pub height: f64,
    /// How steep the terrain is: its rise per meter towards the east and the north.
    pub slope: DVec2,
}

impl Ground {
    pub const UNKNOWN: Self = Self {
        height: f64::NEG_INFINITY,
        slope: DVec2::ZERO,
    };

    pub fn flat(height: f64) -> Self {
        Self {
            height,
            slope: DVec2::ZERO,
        }
    }
}

/// Where and when the weather is sampled.
#[derive(Debug, Clone, Copy)]
pub struct Probe {
    /// ECEF position.
    pub position: DVec3,
    /// Simulation time in seconds.
    pub time: f64,
    /// Longitude and latitude in radians.
    pub lon_lat: DVec2,
    /// Height above sea level in m.
    pub height: f64,
    /// Height above the terrain in m, infinite where the terrain is unknown.
    pub above_ground: f64,
    /// Slope of the terrain below.
    pub slope: DVec2,
    /// Rotation from north-east-down to ECEF.
    pub ned: DQuat,
}

impl Probe {
    /// At an ECEF position and simulation time, over some terrain.
    pub fn new(position: DVec3, time: f64, ground: Ground) -> Self {
        let (lon_lat, height) = geo::from_ecef(position);
        let above_ground = match ground.height.is_finite() {
            true => (height - ground.height).max(0.0),
            false => f64::INFINITY,
        };
        Self {
            position,
            time,
            lon_lat,
            height,
            above_ground,
            slope: ground.slope,
            ned: ned_to_ecef_at(lon_lat),
        }
    }
}

/// The weather a [`WeatherConfig`] describes, ready to be sampled.
#[derive(Debug, Clone)]
pub struct Weather {
    config: WeatherConfig,
    turbulence: Option<Turbulence>,
    gusts: Option<Gusts>,
    thermals: Option<Thermals>,
    /// The velocity in north-east-down that what is frozen in the air drifts with.
    drift: DVec3,
    /// The air at sea level away from the pressure systems.
    sea_level: Air,
}

impl Default for Weather {
    fn default() -> Self {
        Self::calm()
    }
}

impl Weather {
    /// The weather of a configuration, in which what is frozen in the air drifts with the wind
    /// aloft at the start, at an origin given by its longitude and latitude in radians.
    pub fn new(config: WeatherConfig, origin: DVec2) -> Self {
        let turbulence = (config.turbulence > 0.0)
            .then(|| Turbulence::new(config.turbulence, config.seed ^ 0x7475_7262));
        let gusts =
            (config.gusts.speed > 0.0).then(|| Gusts::new(config.gusts, config.seed ^ 0x6775_7374));
        let thermals = (config.thermals.strength > 0.0)
            .then(|| Thermals::new(config.thermals, config.seed ^ 0x7468_726d));
        let SeaLevel {
            pressure,
            temperature,
        } = config.sea_level;
        let mut weather = Self {
            sea_level: standard_air(0.0, pressure * 100.0, temperature + 273.15),
            config,
            turbulence,
            gusts,
            thermals,
            drift: DVec3::ZERO,
        };
        weather.drift = weather.aloft(origin, ned_to_ecef_at(origin), 0.0);
        weather
    }

    /// Still air.
    pub fn calm() -> Self {
        Self::new(WeatherConfig::default(), DVec2::ZERO)
    }

    pub fn config(&self) -> &WeatherConfig {
        &self.config
    }

    /// Velocity of the air in ECEF.
    pub fn wind(&self, probe: &Probe) -> DVec3 {
        let drifted = self.drifted(probe);
        let mut aloft = self.aloft(probe.lon_lat, probe.ned, probe.time);
        if let Some(gusts) = &self.gusts {
            aloft += gusts.velocity(drifted, probe.time, aloft);
        }
        let mut ned = self
            .config
            .boundary_layer
            .apply(aloft, probe.above_ground, probe.lon_lat.y);
        let ridge = self.config.ridge_lift.scale;
        if ridge > 0.0 && probe.above_ground.is_finite() {
            let up = ned.x * probe.slope.y + ned.y * probe.slope.x;
            ned.z -= up * (-probe.above_ground / ridge).exp();
        }
        if let Some(thermals) = &self.thermals {
            let (lon_lat, _) = geo::from_ecef(drifted);
            ned.z -= thermals.updraft(lon_lat, probe.above_ground, probe.time);
        }
        if let Some(turbulence) = &self.turbulence {
            ned += turbulence.velocity(drifted, probe.ned, probe.above_ground);
        }
        probe.ned * ned
    }

    /// The thermals near a probe: where their cores are at its height, in ECEF, and their
    /// radius and updraft there.
    pub fn thermals(&self, probe: &Probe) -> Vec<(DVec3, f64, f64)> {
        let Some(thermals) = &self.thermals else {
            return Vec::new();
        };
        let (lon_lat, _) = geo::from_ecef(self.drifted(probe));
        let drift = probe.ned * (self.drift * probe.time);
        let radius = thermals.radius(probe.above_ground);
        thermals
            .near(lon_lat, probe.time)
            .map(|thermal| {
                let updraft = thermals.updraft(thermal.center, probe.above_ground, probe.time);
                (
                    geo::to_ecef(thermal.center, probe.height) + drift,
                    radius,
                    updraft,
                )
            })
            .collect()
    }

    /// The pressure, temperature and density of the air.
    pub fn air(&self, probe: &Probe) -> Air {
        let (pressure, _) = self.pressure_field(probe.lon_lat, probe.ned, probe.time);
        let sea_level = self.sea_level;
        standard_air(
            probe.height,
            sea_level.pressure + pressure,
            sea_level.temperature,
        )
    }

    /// The wind above the boundary layer, in north-east-down.
    fn aloft(&self, lon_lat: DVec2, ned: DQuat, time: f64) -> DVec3 {
        self.config.wind.ned() + self.pressure_field(lon_lat, ned, time).1
    }

    /// The pressure the pressure systems add at sea level, and their wind.
    fn pressure_field(&self, lon_lat: DVec2, ned: DQuat, time: f64) -> (f64, DVec3) {
        let density = self.sea_level.density;
        pressure::field(&self.config.pressure_systems, lon_lat, ned, time, density)
    }

    /// Where the air at a probe was at the start, for what is frozen in the air and drifts
    /// with it. Everything drifts with the same wind, also in the boundary layer and wherever
    /// the wind aloft differs, as anything that drifted at different speeds in different places
    /// would be torn apart over time.
    fn drifted(&self, probe: &Probe) -> DVec3 {
        probe.position - probe.ned * (self.drift * probe.time)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn probe(above_ground: f64) -> Probe {
        let lon_lat = DVec2::new(0.2, 0.7);
        Probe::new(
            geo::to_ecef(lon_lat, 500.0 + above_ground),
            0.0,
            Ground::flat(500.0),
        )
    }

    fn westerly() -> Weather {
        weather(WeatherConfig {
            wind: Wind {
                from: 270.0,
                speed: 5.0,
            },
            ..Default::default()
        })
    }

    fn weather(config: WeatherConfig) -> Weather {
        Weather::new(config, DVec2::new(0.2, 0.7))
    }

    #[test]
    fn westerly_wind_blows_east() {
        let p = probe(1000.0);
        let ned = p.ned.inverse() * westerly().wind(&p);
        assert!((ned - DVec3::new(0.0, 5.0, 0.0)).length() < 1e-9, "{ned}");
    }

    #[test]
    fn examples_load() {
        let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/weather");
        let mut count = 0;
        for entry in std::fs::read_dir(dir).unwrap() {
            let path = entry.unwrap().path();
            if path.extension().is_some_and(|e| e == "ron") {
                WeatherConfig::load(&path).unwrap();
                count += 1;
            }
        }
        assert!(count >= 3);
    }

    #[test]
    fn calm_by_default() {
        let config = WeatherConfig::parse("()").unwrap();
        let p = probe(50.0);
        assert_eq!(weather(config).wind(&p), DVec3::ZERO);
    }

    #[test]
    fn boundary_layer_slows_and_backs_the_wind() {
        let weather = westerly();
        let at = |h: f64| {
            let p = probe(h);
            p.ned.inverse() * weather.wind(&p)
        };
        let surface = at(10.0);
        // A logarithmic profile over farmland: ln(100) / ln(3000)
        assert!((surface.length() / 5.0 - 0.575).abs() < 0.01, "{surface}");
        // In the northern hemisphere, from the left of the wind aloft: from the west-southwest,
        // blowing towards the north of east
        let from = (-surface.y)
            .atan2(-surface.x)
            .to_degrees()
            .rem_euclid(360.0);
        assert!(
            (from - (270.0 - 20.0 * (1.0 - 10.0 / 300.0))).abs() < 1e-6,
            "{from}"
        );
        assert!(surface.x > 0.0);
        // Faster with height, and still at the ground
        assert!(at(100.0).length() > surface.length());
        assert!(at(0.0).length() < 1e-9);
        assert!((at(300.0) - DVec3::new(0.0, 5.0, 0.0)).length() < 1e-9);
    }

    #[test]
    fn unknown_ground_is_far_below() {
        let position = geo::to_ecef(DVec2::new(0.2, 0.7), 10.0);
        let p = Probe::new(position, 0.0, Ground::UNKNOWN);
        let ned = p.ned.inverse() * westerly().wind(&p);
        assert!((ned - DVec3::new(0.0, 5.0, 0.0)).length() < 1e-9);
    }

    #[test]
    fn turbulence_drifts_with_the_wind() {
        let weather = weather(WeatherConfig {
            turbulence: 1.0,
            ..westerly().config
        });
        let p = probe(1000.0);
        let gust = weather.wind(&p) - westerly().wind(&p);
        assert!(gust.length() > 1e-3);
        let downwind = Probe::new(
            p.position + p.ned * DVec3::new(0.0, 50.0, 0.0),
            10.0,
            Ground::flat(500.0),
        );
        let later = weather.wind(&downwind) - westerly().wind(&downwind);
        // But for the north-east-down frame turning a little over the distance
        assert!((later - gust).length() < 1e-3, "{gust} {later}");
        // The same weather from the same configuration
        let again = Weather::new(weather.config().clone(), DVec2::new(0.2, 0.7));
        assert_eq!(again.wind(&p), weather.wind(&p));
    }

    /// The air south of a low in the north is thinner, and blows east, and near the ground
    /// across the isobars into the low.
    #[test]
    fn lows_thin_the_air_and_blow_around() {
        let low = PressureSystem {
            lat: 0.7f64.to_degrees() + 300.0 / 111.195,
            lon: 0.2f64.to_degrees(),
            pressure: -20.0,
            radius: 300.0,
            ..Default::default()
        };
        let weather = weather(WeatherConfig {
            pressure_systems: vec![low],
            ..Default::default()
        });
        let p = probe(1000.0);
        let calm = Weather::calm().air(&p);
        let air = weather.air(&p);
        assert!(air.pressure < calm.pressure - 1000.0, "{air:?}");
        assert!(air.density < calm.density);
        let aloft = p.ned.inverse() * weather.wind(&p);
        assert!(aloft.y > 10.0 && aloft.x.abs() < 0.1 * aloft.y, "{aloft}");
        let p = probe(10.0);
        let surface = p.ned.inverse() * weather.wind(&p);
        assert!(surface.x > 0.2 * surface.y, "{surface}");
        // What is frozen in the air drifts with that wind
        assert!((weather.drift - aloft).length() < 0.5, "{}", weather.drift);
    }

    #[test]
    fn wind_up_a_slope_lifts() {
        let lon_lat = DVec2::new(0.2, 0.7);
        let rising_east = Ground {
            height: 500.0,
            slope: DVec2::new(0.3, 0.0),
        };
        let at = |above_ground: f64| {
            let position = geo::to_ecef(lon_lat, 500.0 + above_ground);
            let p = Probe::new(position, 0.0, rising_east);
            p.ned.inverse() * westerly().wind(&p)
        };
        // Following the slope near the ground
        let low = at(10.0);
        let expected = low.y * 0.3 * (-10.0f64 / 150.0).exp();
        assert!((-low.z - expected).abs() < 1e-9, "{low}");
        assert!(-at(300.0).z < -low.z * 0.3);
        // But not where the terrain is unknown
        let p = Probe::new(geo::to_ecef(lon_lat, 510.0), 0.0, Ground::UNKNOWN);
        assert!(westerly().wind(&p).dot(p.ned * DVec3::Z).abs() < 1e-12);
    }

    /// The thermals drawn in the viewer are where the updraft is, and drift with the wind.
    #[test]
    fn thermals_where_the_updraft_is() {
        let weather = weather(WeatherConfig {
            thermals: ThermalConfig {
                strength: 3.0,
                ..Default::default()
            },
            ..westerly().config
        });
        let p = probe(400.0);
        let thermals = weather.thermals(&p);
        assert!(!thermals.is_empty());
        let strongest = thermals.iter().max_by(|a, b| a.2.total_cmp(&b.2)).unwrap();
        let (center, radius, updraft) = *strongest;
        assert!(updraft > 1.0 && radius > 50.0, "{updraft} {radius}");
        // At the height of the probe, and rising there
        let (_, height) = geo::from_ecef(center);
        assert!((height - 900.0).abs() < 1e-6);
        let at = |position: DVec3, time: f64| {
            let p = Probe::new(position, time, Ground::flat(500.0));
            -(p.ned.inverse() * weather.wind(&p)).z
        };
        assert!(
            (at(center, 0.0) - updraft).abs() < 0.1,
            "{}",
            at(center, 0.0)
        );
        // A minute later, 300 m further east
        let later = center + p.ned * DVec3::new(0.0, 300.0, 0.0);
        assert!(
            (at(later, 60.0) - updraft).abs() < 0.15,
            "{}",
            at(later, 60.0)
        );
    }

    #[test]
    fn southern_hemisphere_veers() {
        let lon_lat = DVec2::new(0.2, -0.7);
        let p = Probe::new(geo::to_ecef(lon_lat, 10.0), 0.0, Ground::flat(0.0));
        let ned = p.ned.inverse() * westerly().wind(&p);
        // Turned to the right, towards the south of east
        assert!(ned.x < 0.0 && ned.y > 0.0, "{ned}");
    }
}
