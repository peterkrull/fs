//! The schema of aircraft definitions, which are written in RON.
//!
//! All positions and directions are in the model frame: `x` forward, `y` right and `z` down
//! (FRD), in meters, relative to any origin. Angles are in degrees.

use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
    time::SystemTime,
};

use serde::Deserialize;

pub type Vec3 = [f64; 3];

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AircraftConfig {
    pub name: String,
    /// Total mass in kilograms.
    pub mass: f64,
    pub center_of_gravity: Vec3,
    /// Moments of inertia about the center of gravity, in body axes.
    pub inertia: Inertia,
    /// Airspeed in m/s to launch the aircraft at, unless another is given on the command line.
    #[serde(default)]
    pub launch_speed: Option<f64>,
    /// Airfoils by name, referred to by the lifting surfaces.
    #[serde(default)]
    pub airfoils: BTreeMap<String, Airfoil>,
    #[serde(default)]
    pub surfaces: Vec<Surface>,
    #[serde(default)]
    pub bodies: Vec<Body>,
    #[serde(default)]
    pub drag: Vec<DragElement>,
    #[serde(default)]
    pub rotors: Vec<Rotor>,
    #[serde(default)]
    pub battery: Battery,
    pub actuators: Vec<Actuator>,
    #[serde(default)]
    pub visual: Visual,
}

impl AircraftConfig {
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

/// Watches an aircraft definition for changes. It compares modification times, which works
/// both with editors that write the file in place and with those that replace it.
pub struct Watcher {
    path: PathBuf,
    /// Modification time of the definition when it was last loaded, and when last polled.
    loaded: Option<SystemTime>,
    polled: Option<SystemTime>,
}

impl Watcher {
    /// Starts out with the definition as it is now, assuming it was just loaded.
    pub fn new(path: impl Into<PathBuf>) -> Self {
        let path = path.into();
        let modified = modified(&path);
        Self {
            path,
            loaded: modified,
            polled: modified,
        }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// The definition, if it changed since it was last loaded. To not read a file that is
    /// still being written, it is only loaded once it is unchanged since the previous poll.
    pub fn poll(&mut self) -> Option<Result<AircraftConfig, String>> {
        let modified = modified(&self.path);
        let settled = modified == self.polled;
        self.polled = modified;
        if modified.is_none() || modified == self.loaded || !settled {
            return None;
        }
        self.loaded = modified;
        Some(AircraftConfig::load(&self.path))
    }
}

fn modified(path: &Path) -> Option<SystemTime> {
    std::fs::metadata(path).and_then(|m| m.modified()).ok()
}

/// Moments of inertia in kg m². `ixz` is the product of inertia `∫ x z dm`.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Inertia {
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
    #[serde(default)]
    pub ixz: f64,
}

/// The aerodynamic characteristics of a two dimensional section, as a function of its angle of
/// attack. Beyond the attached flow range, both kinds blend into the behavior of a flat plate,
/// so that any angle of attack, including flying backwards, is handled.
#[derive(Debug, Clone, Deserialize)]
pub enum Airfoil {
    /// Linear lift with a quadratic drag polar, up to the stall angles.
    Parametric {
        /// Lift slope per radian. Thin airfoil theory gives 2π, real airfoils a bit less.
        #[serde(default = "default_lift_slope")]
        lift_slope: f64,
        /// Angle of attack without lift, negative for cambered airfoils.
        #[serde(default)]
        zero_lift_angle: f64,
        /// Minimum profile drag coefficient.
        cd_min: f64,
        /// Profile drag grows with `cd_k * cl²`.
        #[serde(default = "default_cd_k")]
        cd_k: f64,
        /// Pitching moment about the quarter chord.
        #[serde(default)]
        cm: f64,
        /// Angles of attack at which the flow separates.
        stall_positive: f64,
        stall_negative: f64,
        #[serde(default)]
        shape: AirfoilShape,
    },
    /// Measured or computed (for example with XFoil) coefficients by angle of attack.
    Table {
        alpha: Vec<f64>,
        cl: Vec<f64>,
        cd: Vec<f64>,
        cm: Vec<f64>,
        #[serde(default)]
        shape: AirfoilShape,
    },
}

impl Airfoil {
    pub fn shape(&self) -> AirfoilShape {
        match self {
            Airfoil::Parametric { shape, .. } | Airfoil::Table { shape, .. } => *shape,
        }
    }
}

/// The shape of the airfoil as a NACA 4-digit profile, used for drawing.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AirfoilShape {
    /// Maximum thickness as a fraction of the chord.
    pub thickness: f64,
    /// Maximum camber as a fraction of the chord.
    #[serde(default)]
    pub camber: f64,
    /// Position of the maximum camber as a fraction of the chord.
    #[serde(default = "default_camber_position")]
    pub camber_position: f64,
}

impl Default for AirfoilShape {
    fn default() -> Self {
        Self {
            thickness: 0.12,
            camber: 0.0,
            camber_position: 0.4,
        }
    }
}

/// A wing, tail or fin, made of sections attached end to end from the root outwards.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Surface {
    pub name: String,
    pub airfoil: String,
    /// Leading edge at the root.
    pub root: Vec3,
    pub root_chord: f64,
    /// Angle of the root chord to the `x` axis, positive with the leading edge up.
    #[serde(default)]
    pub incidence: f64,
    /// Whether the surface is mirrored to the other side of the `x`-`z` plane.
    #[serde(default)]
    pub mirror: bool,
    pub sections: Vec<Section>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Section {
    /// Length along the span, measured in the plane of the dihedral.
    pub span: f64,
    pub tip_chord: f64,
    /// Sweep of the leading edge, positive backwards.
    #[serde(default)]
    pub sweep: f64,
    /// Angle of the span above the `x`-`y` plane. A vertical fin has a dihedral of 90°.
    #[serde(default)]
    pub dihedral: f64,
    /// Change in incidence from the root to the tip of the section, negative for washout.
    #[serde(default)]
    pub twist: f64,
    /// Incidence at the root of the section, if it does not continue that of the previous
    /// section, such as for a winglet.
    #[serde(default)]
    pub incidence: Option<f64>,
    /// Number of spanwise cells the section is divided into.
    #[serde(default = "default_cells")]
    pub cells: usize,
    /// Airfoil of the section, if it differs from that of the surface.
    #[serde(default)]
    pub airfoil: Option<String>,
    #[serde(default)]
    pub controls: Vec<Control>,
}

/// A hinged trailing edge flap, such as an aileron, elevator or rudder.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Control {
    pub actuator: String,
    /// Deflection at full positive actuator travel. Positive deflections move the trailing edge
    /// to the lower side of the surface, which increases its lift.
    pub deflection: f64,
    /// Chord of the flap as a fraction of the chord of the surface.
    pub chord_fraction: f64,
    /// Spanwise extent within the section, as fractions from its root to its tip.
    #[serde(default = "default_extent")]
    pub extent: (f64, f64),
    /// How the flap on the mirrored side of the surface moves.
    #[serde(default)]
    pub mirror: Mirror,
}

#[derive(Debug, Clone, Default, Deserialize)]
pub enum Mirror {
    /// Deflects the same way, like an elevator.
    #[default]
    Symmetric,
    /// Deflects the opposite way, like an aileron.
    Antisymmetric,
    /// Is driven by another actuator, with the same deflection.
    Actuator(String),
}

/// A fuselage, pod or boom: a body of revolution along the `x` axis.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Body {
    pub name: String,
    /// The front of the body, which extends backwards from it.
    pub nose: Vec3,
    pub length: f64,
    /// Radius along the body, as `(fraction of the length, radius)` from nose to tail.
    pub profile: Vec<(f64, f64)>,
    #[serde(default = "default_body_cells")]
    pub cells: usize,
    /// Drag coefficient of the flow across the body, based on its side area.
    #[serde(default = "default_cd_crossflow")]
    pub cd_crossflow: f64,
    /// Drag coefficient of the flow along the body, based on its largest cross section.
    #[serde(default = "default_cd_axial")]
    pub cd_axial: f64,
}

/// Drag of anything not modeled otherwise, such as landing gear or the arms of a multicopter.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DragElement {
    pub name: String,
    pub position: Vec3,
    /// Drag coefficient times reference area in m², for flow along each axis.
    pub cd_area: Vec3,
}

/// A propeller or rotor, driven by an electric motor or a piston engine.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Rotor {
    pub name: String,
    pub position: Vec3,
    /// Direction of the thrust.
    pub axis: Vec3,
    pub diameter: f64,
    /// Direction of rotation, as seen from behind, looking in the direction of the thrust.
    pub spin: Spin,
    /// Static thrust coefficient, `T = ct ρ n² D⁴` with `n` in revolutions per second.
    /// Unless the coefficients are given as a `table`.
    #[serde(default)]
    pub ct: f64,
    /// Static power coefficient, `P = cp ρ n³ D⁵`.
    #[serde(default)]
    pub cp: f64,
    /// Advance ratio `J = V / (n D)` at which the thrust vanishes, about the pitch over the
    /// diameter of the propeller. Below it, the thrust falls linearly, and the power
    /// quadratically.
    #[serde(default)]
    pub zero_thrust_advance: f64,
    /// Measured thrust and power coefficients by advance ratio, in place of the above.
    #[serde(default)]
    pub table: Option<PropellerTable>,
    /// Moment of inertia of the propeller and the rotating parts of the motor, in kg m².
    pub inertia: f64,
    /// What drives the rotor: a brushless `motor`, or a piston `engine`.
    #[serde(default)]
    pub motor: Option<Motor>,
    #[serde(default)]
    pub engine: Option<Engine>,
    /// The actuator setting the throttle of the motor or engine.
    pub throttle: String,
}

/// Coefficients of a propeller by advance ratio `J = V / (n D)`, as `ct` and `cp` above.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PropellerTable {
    pub advance: Vec<f64>,
    pub ct: Vec<f64>,
    pub cp: Vec<f64>,
}

/// A piston engine, whose torque at full throttle is about the same at any speed, and falls
/// with the density of the air, as it takes in less of it. The throttle sets the torque,
/// like the manifold pressure does. At zero throttle the engine produces no power, so the
/// propeller windmills against its friction, as with a failed engine.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Engine {
    /// Rated brake power at sea level in watts, at `rpm`.
    pub power: f64,
    pub rpm: f64,
    /// Friction and pumping losses at the rated speed, as a fraction of the rated torque,
    /// which grow with the speed.
    #[serde(default = "default_friction")]
    pub friction: f64,
}

#[derive(Debug, Clone, Copy, Deserialize, PartialEq, Eq)]
pub enum Spin {
    Clockwise,
    CounterClockwise,
}

/// A brushless motor, with its speed controller as an ideal voltage source.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Motor {
    /// Speed constant in rpm per volt.
    pub kv: f64,
    /// Winding resistance in ohm.
    pub resistance: f64,
    #[serde(default)]
    pub no_load_current: f64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Battery {
    /// Open circuit voltage.
    pub voltage: f64,
    pub internal_resistance: f64,
}

impl Default for Battery {
    fn default() -> Self {
        Self {
            voltage: 12.6,
            internal_resistance: 0.02,
        }
    }
}

/// A servo or speed controller, following its command with a first order response limited
/// in rate.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Actuator {
    pub name: String,
    #[serde(default)]
    pub kind: ActuatorKind,
    /// Servo output of the flight controller driving it, counting from 1.
    #[serde(default)]
    pub channel: Option<usize>,
    /// Time constant of the response in seconds.
    pub time_constant: f64,
    /// Fastest rate of change, in units of the normalized range per second.
    pub slew_rate: f64,
    #[serde(default)]
    pub pwm: Pwm,
    #[serde(default)]
    pub reversed: bool,
    /// How the pilot drives the actuator when flying without a flight controller, as a sum of
    /// stick axes times their gains. Mixes, such as for elevons, are sums of several axes.
    #[serde(default)]
    pub input: Vec<(Axis, f64)>,
}

impl AircraftConfig {
    /// Actuator commands for the controls of a pilot, as the actuators mix them.
    pub fn mix(&self, sticks: &Sticks) -> Vec<f64> {
        self.actuators
            .iter()
            .map(|actuator| {
                actuator
                    .input
                    .iter()
                    .map(|(axis, gain)| gain * sticks.get(*axis))
                    .sum()
            })
            .collect()
    }
}

/// The positions of the sticks of a pilot, see [`Axis`].
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Sticks {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
    pub throttle: f64,
}

impl Sticks {
    pub fn get(&self, axis: Axis) -> f64 {
        match axis {
            Axis::Roll => self.roll,
            Axis::Pitch => self.pitch,
            Axis::Yaw => self.yaw,
            Axis::Throttle => self.throttle,
        }
    }
}

/// Axes of the controls of a pilot. Positive values roll right, pitch up, yaw right, and
/// open the throttle.
#[derive(Debug, Clone, Copy, Deserialize, PartialEq, Eq)]
pub enum Axis {
    Roll,
    Pitch,
    Yaw,
    Throttle,
}

#[derive(Debug, Clone, Copy, Default, Deserialize, PartialEq, Eq)]
pub enum ActuatorKind {
    /// Normalized to `-1..1`.
    #[default]
    Servo,
    /// Normalized to `0..1`.
    Throttle,
}

/// Pulse widths in microseconds.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Pwm {
    pub min: f64,
    pub trim: f64,
    pub max: f64,
}

impl Default for Pwm {
    fn default() -> Self {
        Self {
            min: 1000.0,
            trim: 1500.0,
            max: 2000.0,
        }
    }
}

/// How the aircraft is drawn.
#[derive(Debug, Clone, Default, Deserialize)]
pub enum Visual {
    /// Generated from the geometry of the definition.
    #[default]
    Generated,
    /// A glTF model, relative to the assets directory. Its origin is placed at `offset` in the
    /// model frame, and its axes follow the glTF convention of `+z` forward and `+y` up.
    Gltf {
        path: String,
        #[serde(default)]
        offset: Vec3,
        #[serde(default = "default_scale")]
        scale: f64,
    },
}

fn default_lift_slope() -> f64 {
    0.9 * std::f64::consts::TAU
}
fn default_cd_k() -> f64 {
    0.01
}
fn default_camber_position() -> f64 {
    0.4
}
fn default_cells() -> usize {
    8
}
fn default_extent() -> (f64, f64) {
    (0.0, 1.0)
}
fn default_body_cells() -> usize {
    12
}
fn default_cd_crossflow() -> f64 {
    1.2
}
fn default_cd_axial() -> f64 {
    0.15
}
fn default_friction() -> f64 {
    0.1
}
fn default_scale() -> f64 {
    1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    const BALL: &str = r#"(name: "ball", mass: 1.0, center_of_gravity: (0.0, 0.0, 0.0),
        inertia: (ixx: 0.1, iyy: 0.1, izz: 0.1), actuators: [])"#;

    #[test]
    fn watcher_loads_saved_definitions_once_settled() {
        let dir = std::env::temp_dir().join(format!("worldmap-watcher-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("ball.ron");
        let write = |text: &str, seconds: u64| {
            std::fs::write(&path, text).unwrap();
            let time = SystemTime::UNIX_EPOCH + std::time::Duration::from_secs(seconds);
            std::fs::File::options()
                .write(true)
                .open(&path)
                .unwrap()
                .set_modified(time)
                .unwrap();
        };
        write(BALL, 1000);
        let mut watcher = Watcher::new(&path);
        assert!(watcher.poll().is_none());

        // Saved, but only loaded once it stays the same for a poll
        write(&BALL.replace("1.0,", "2.0,"), 2000);
        assert!(watcher.poll().is_none());
        let config = watcher.poll().unwrap().unwrap();
        assert_eq!(config.mass, 2.0);
        assert!(watcher.poll().is_none());

        // Errors are reported, and the next save is loaded again
        write("(name: ", 3000);
        assert!(watcher.poll().is_none());
        assert!(watcher.poll().unwrap().is_err());
        write(BALL, 4000);
        assert!(watcher.poll().is_none());
        assert!(watcher.poll().unwrap().is_ok());

        std::fs::remove_dir_all(&dir).unwrap();
    }
}
