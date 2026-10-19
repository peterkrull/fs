//! The schema of aircraft definitions, which are written in RON.
//!
//! All positions and directions are in the model frame: `x` forward, `y` right and `z` down
//! (FRD), in meters, relative to any origin. Angles are in degrees.

use std::{collections::BTreeMap, path::Path};

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

/// A propeller or rotor, driven by an electric motor.
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
    pub ct: f64,
    /// Static power coefficient, `P = cp ρ n³ D⁵`.
    pub cp: f64,
    /// Advance ratio `J = V / (n D)` at which the thrust vanishes, about the pitch over the
    /// diameter of the propeller.
    pub zero_thrust_advance: f64,
    /// Moment of inertia of the propeller and the rotating parts of the motor, in kg m².
    pub inertia: f64,
    pub motor: Motor,
    /// The actuator setting the throttle of the motor.
    pub throttle: String,
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
fn default_scale() -> f64 {
    1.0
}
