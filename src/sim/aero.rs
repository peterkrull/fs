//! Aerodynamics of the lifting surfaces and bodies.
//!
//! Lifting surfaces are divided into spanwise strips, each with the section characteristics of
//! its airfoil. The strips interact through a nonlinear lifting line (after Phillips & Snyder):
//! every strip carries a horseshoe vortex whose strength follows from the lift of the strip,
//! and whose trailing vortices induce a downwash on all strips. This captures the loss of lift
//! towards the tips, induced drag, the downwash of the wing on the tail, and the movement of the
//! center of lift as the flow and the control surfaces change, without simulating the flow
//! itself. The vortex strengths are solved iteratively, starting from those of the previous
//! step, which makes a few iterations per step enough.
//!
//! Bodies are divided into cells along their length, each with drag from the flow across and
//! along it.

use std::f64::consts::PI;

use bevy::math::DVec3;

use super::{
    airfoil::{AirfoilModel, Flap},
    config::{AircraftConfig, Mirror},
};

/// Iterations of the lifting line per step, starting from the previous solution.
const ITERATIONS: usize = 3;
/// Iterations to start from scratch.
const INITIAL_ITERATIONS: usize = 200;

/// A spanwise strip of a lifting surface. Its vectors are in the model frame.
#[derive(Debug, Clone)]
pub struct Strip {
    pub surface: usize,
    pub airfoil: usize,
    /// Midpoint of the quarter chord line.
    pub center: DVec3,
    /// Ends of the quarter chord line, which carries the bound vortex from `a` to `b`.
    pub a: DVec3,
    pub b: DVec3,
    /// Leading and trailing edge at the `a` and `b` ends.
    pub leading: [DVec3; 2],
    pub trailing: [DVec3; 2],
    /// Direction from the leading to the trailing edge.
    pub chord_dir: DVec3,
    /// Direction in which positive lift acts, at zero angle of attack.
    pub normal: DVec3,
    /// Direction of the bound vortex, `normal × chord_dir`.
    pub span_dir: DVec3,
    pub chord: f64,
    pub area: f64,
    pub control: Option<StripControl>,
}

#[derive(Debug, Clone)]
pub struct StripControl {
    pub actuator: usize,
    /// Deflection in radians at full positive actuator travel.
    pub gain: f64,
    pub flap: Flap,
    pub chord_fraction: f64,
}

/// A slice of a body.
#[derive(Debug, Clone)]
pub struct BodyCell {
    pub body: usize,
    pub center: DVec3,
    pub length: f64,
    pub radius: f64,
    pub cd_crossflow: f64,
    /// Share of the axial drag of the body, as drag coefficient times area.
    pub axial_area: f64,
}

/// The aerodynamic forces on one strip, in the body frame.
#[derive(Debug, Clone, Copy, Default)]
pub struct StripLoad {
    pub lift: DVec3,
    pub drag: DVec3,
    pub moment: DVec3,
    pub alpha: f64,
    pub deflection: f64,
    pub separation: f64,
}

#[derive(Debug, Clone, Default)]
pub struct AeroLoads {
    pub strips: Vec<StripLoad>,
    pub cells: Vec<DVec3>,
    pub drag_elements: Vec<DVec3>,
    /// Total force and moment about the center of gravity.
    pub force: DVec3,
    pub moment: DVec3,
    /// Where the resultant of the lifting surfaces acts, in the model frame.
    pub center_of_lift: Option<DVec3>,
}

/// The air around the aircraft, in the body frame.
pub struct Flow<'a> {
    /// Velocity of the air relative to the center of gravity.
    pub velocity: DVec3,
    /// Angular velocity of the aircraft.
    pub rates: DVec3,
    pub density: f64,
    /// Positions of the actuators.
    pub actuators: &'a [f64],
}

impl Flow<'_> {
    /// Velocity of the air relative to a point of the aircraft, given relative to the center
    /// of gravity.
    fn at(&self, offset: DVec3) -> DVec3 {
        self.velocity - self.rates.cross(offset)
    }
}

pub struct Aerodynamics {
    pub strips: Vec<Strip>,
    pub cells: Vec<BodyCell>,
    pub airfoils: Vec<AirfoilModel>,
    pub drag_elements: Vec<(DVec3, DVec3)>,
    center_of_gravity: DVec3,
    /// Velocity induced at the center of strip `i` by a unit vortex on strip `j`, at `i * n + j`.
    influence: Vec<DVec3>,
    /// Relaxation of the iteration of each strip, against its influence on itself.
    relaxation: Vec<f64>,
    /// Vortex strength of each strip.
    circulation: Vec<f64>,
    converged: bool,
}

impl Aerodynamics {
    pub fn new(config: &AircraftConfig, actuators: &[String]) -> Result<Self, String> {
        let center_of_gravity = DVec3::from(config.center_of_gravity);
        let airfoil_names: Vec<&String> = config.airfoils.keys().collect();
        let airfoils = config
            .airfoils
            .iter()
            .map(|(name, airfoil)| {
                AirfoilModel::new(airfoil).map_err(|err| format!("airfoil {name}: {err}"))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let find_airfoil = |name: &str| {
            airfoil_names
                .iter()
                .position(|n| *n == name)
                .ok_or_else(|| format!("unknown airfoil {name}"))
        };
        let find_actuator = |name: &str| {
            actuators
                .iter()
                .position(|n| n == name)
                .ok_or_else(|| format!("unknown actuator {name}"))
        };

        let mut strips = Vec::new();
        for (index, surface) in config.surfaces.iter().enumerate() {
            let mut le = DVec3::from(surface.root);
            let mut chord = surface.root_chord;
            let mut incidence = surface.incidence.to_radians();
            for section in &surface.sections {
                let airfoil = find_airfoil(section.airfoil.as_deref().unwrap_or(&surface.airfoil))?;
                let dihedral = section.dihedral.to_radians();
                let span = DVec3::new(0.0, dihedral.cos(), -dihedral.sin());
                let sweep = section.sweep.to_radians().tan();
                let twist = section.twist.to_radians();
                let station = |t: f64| Station {
                    leading: le + span * (section.span * t) - DVec3::X * (section.span * t * sweep),
                    chord: chord + (section.tip_chord - chord) * t,
                    incidence: incidence + twist * t,
                    span,
                };

                for cell in 0..section.cells {
                    let (t0, t1) = (
                        cell as f64 / section.cells as f64,
                        (cell + 1) as f64 / section.cells as f64,
                    );
                    let tm = (t0 + t1) / 2.0;
                    let control = section
                        .controls
                        .iter()
                        .find(|c| tm >= c.extent.0 && tm <= c.extent.1);

                    let right = strip(index, airfoil, station(t0), station(t1), station(tm));
                    let mut sides =
                        vec![(right, control.map(|c| (c.actuator.as_str(), c.deflection)))];
                    if surface.mirror {
                        let left = sides[0].0.mirrored();
                        let left_control = control.map(|c| match &c.mirror {
                            Mirror::Symmetric => (c.actuator.as_str(), c.deflection),
                            Mirror::Antisymmetric => (c.actuator.as_str(), -c.deflection),
                            Mirror::Actuator(name) => (name.as_str(), c.deflection),
                        });
                        sides.push((left, left_control));
                    }
                    for (mut strip, side_control) in sides {
                        if let (Some(control), Some((actuator, deflection))) =
                            (control, side_control)
                        {
                            strip.control = Some(StripControl {
                                actuator: find_actuator(actuator)?,
                                gain: deflection.to_radians(),
                                flap: Flap::new(control.chord_fraction),
                                chord_fraction: control.chord_fraction,
                            });
                        }
                        strips.push(strip);
                    }
                }

                le = station(1.0).leading;
                chord = section.tip_chord;
                incidence += twist;
            }
        }

        let mut cells = Vec::new();
        for (index, body) in config.bodies.iter().enumerate() {
            let nose = DVec3::from(body.nose);
            let radius_at = |t: f64| interpolate_profile(&body.profile, t);
            let max_radius = body.profile.iter().map(|(_, r)| *r).fold(0.0, f64::max);
            let length = body.length / body.cells as f64;
            for cell in 0..body.cells {
                let t = (cell as f64 + 0.5) / body.cells as f64;
                cells.push(BodyCell {
                    body: index,
                    center: nose - DVec3::X * (body.length * t),
                    length,
                    radius: radius_at(t),
                    cd_crossflow: body.cd_crossflow,
                    axial_area: body.cd_axial * PI * max_radius * max_radius / body.cells as f64,
                });
            }
        }

        let drag_elements = config
            .drag
            .iter()
            .map(|d| (DVec3::from(d.position), DVec3::from(d.cd_area)))
            .collect();

        let n = strips.len();
        let mut influence = vec![DVec3::ZERO; n * n];
        for (i, target) in strips.iter().enumerate() {
            for (j, source) in strips.iter().enumerate() {
                influence[i * n + j] = horseshoe(target.center, source, i == j);
            }
        }
        // Damp each strip against its own influence on itself, which would otherwise make the
        // iteration overshoot for strips that are narrow compared to their chord
        let relaxation = strips
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let gain = 0.5
                    * s.chord
                    * airfoils[s.airfoil].lift_slope()
                    * (-influence[i * n + i].dot(s.normal)).max(0.0);
                1.0 / (1.0 + gain)
            })
            .collect();

        Ok(Self {
            strips,
            cells,
            airfoils,
            drag_elements,
            center_of_gravity,
            influence,
            relaxation,
            circulation: vec![0.0; n],
            converged: false,
        })
    }

    /// Forget the vortex strengths, for example after the aircraft was teleported.
    pub fn reset(&mut self) {
        self.circulation.fill(0.0);
        self.converged = false;
    }

    pub fn loads(&mut self, flow: &Flow) -> AeroLoads {
        let n = self.strips.len();
        let freestream: Vec<DVec3> = self
            .strips
            .iter()
            .map(|s| flow.at(s.center - self.center_of_gravity))
            .collect();
        let deflections: Vec<f64> = self
            .strips
            .iter()
            .map(|s| {
                s.control
                    .as_ref()
                    .map_or(0.0, |c| c.gain * flow.actuators[c.actuator])
            })
            .collect();

        let iterations = if self.converged {
            ITERATIONS
        } else {
            INITIAL_ITERATIONS
        };
        let mut targets = vec![0.0; n];
        for _ in 0..iterations {
            for i in 0..n {
                let section = self.section(i, freestream[i], deflections[i]);
                targets[i] = 0.5 * section.speed * self.strips[i].chord * section.cl;
            }
            for ((circulation, relaxation), target) in self
                .circulation
                .iter_mut()
                .zip(&self.relaxation)
                .zip(&targets)
            {
                *circulation += relaxation * (target - *circulation);
            }
        }
        self.converged = true;

        let mut loads = AeroLoads {
            strips: Vec::with_capacity(n),
            ..Default::default()
        };
        let mut lift_force = DVec3::ZERO;
        let mut lift_moment = DVec3::ZERO;
        let mut lift_height = (0.0, 0.0);
        for i in 0..n {
            let strip = &self.strips[i];
            let section = self.section(i, freestream[i], deflections[i]);
            let q = 0.5 * flow.density * section.speed * section.speed * strip.area;
            let (lift, drag, moment) = match section.speed > 1e-6 {
                true => {
                    let flow_dir = section.in_plane / section.speed;
                    (
                        flow_dir.cross(strip.span_dir) * (q * section.cl),
                        flow_dir * (q * section.cd),
                        strip.span_dir * (q * strip.chord * section.cm),
                    )
                }
                false => (DVec3::ZERO, DVec3::ZERO, DVec3::ZERO),
            };
            let offset = strip.center - self.center_of_gravity;
            let force = lift + drag;
            let torque = offset.cross(force) + moment;
            loads.force += force;
            loads.moment += torque;
            lift_force += force;
            lift_moment += torque;
            lift_height.0 += lift.length() * offset.z;
            lift_height.1 += lift.length();
            loads.strips.push(StripLoad {
                lift,
                drag,
                moment,
                alpha: section.alpha,
                deflection: deflections[i],
                separation: section.separation,
            });
        }

        // The point in the plane of the lifting surfaces about which they exert no pitching
        // or rolling moment
        if lift_force.z.abs() > 1e-3 {
            let z = lift_height.0 / lift_height.1.max(1e-9);
            let x = -lift_moment.y / lift_force.z;
            let y = lift_moment.x / lift_force.z;
            loads.center_of_lift = Some(self.center_of_gravity + DVec3::new(x, y, z));
        }

        for cell in &self.cells {
            let offset = cell.center - self.center_of_gravity;
            let air = flow.at(offset);
            let axial = air.x;
            let across = DVec3::new(0.0, air.y, air.z);
            let crossflow = across
                * (0.5
                    * flow.density
                    * across.length()
                    * 2.0
                    * cell.radius
                    * cell.length
                    * cell.cd_crossflow);
            let along = DVec3::X * (0.5 * flow.density * axial * axial.abs() * cell.axial_area);
            let force = crossflow + along;
            loads.force += force;
            loads.moment += offset.cross(force);
            loads.cells.push(force);
        }

        for (position, cd_area) in &self.drag_elements {
            let offset = *position - self.center_of_gravity;
            let air = flow.at(offset);
            let force = 0.5 * flow.density * air * air.abs() * *cd_area;
            loads.force += force;
            loads.moment += offset.cross(force);
            loads.drag_elements.push(force);
        }

        loads
    }

    /// The flow over a strip, including the velocity induced by all vortices.
    fn section(&self, i: usize, freestream: DVec3, deflection: f64) -> Section {
        let n = self.strips.len();
        let strip = &self.strips[i];
        let induced: DVec3 = self.influence[i * n..(i + 1) * n]
            .iter()
            .zip(&self.circulation)
            .map(|(v, g)| *v * *g)
            .sum();
        let air = freestream + induced;
        let along = air.dot(strip.chord_dir);
        let up = air.dot(strip.normal);
        let in_plane = strip.chord_dir * along + strip.normal * up;
        let speed = in_plane.length();
        let alpha = up.atan2(along);

        let (flap_alpha, flap_cm, flap_cd) = strip
            .control
            .as_ref()
            .map_or((0.0, 0.0, 0.0), |c| c.flap.effect(deflection));
        let c = self.airfoils[strip.airfoil].coefficients(alpha + flap_alpha);
        Section {
            in_plane,
            speed,
            alpha,
            cl: c.cl,
            cd: c.cd + flap_cd,
            cm: c.cm + flap_cm * (1.0 - c.separation),
            separation: c.separation,
        }
    }
}

struct Section {
    in_plane: DVec3,
    speed: f64,
    alpha: f64,
    cl: f64,
    cd: f64,
    cm: f64,
    separation: f64,
}

struct Station {
    leading: DVec3,
    chord: f64,
    incidence: f64,
    span: DVec3,
}

impl Station {
    /// Chord direction and lift normal, with the leading edge raised by the incidence.
    fn axes(&self) -> (DVec3, DVec3) {
        let chord = -DVec3::X;
        let normal = chord.cross(self.span);
        let (sin, cos) = self.incidence.sin_cos();
        (chord * cos - normal * sin, chord * sin + normal * cos)
    }

    fn quarter_chord(&self) -> DVec3 {
        self.leading + self.axes().0 * (0.25 * self.chord)
    }

    fn trailing(&self) -> DVec3 {
        self.leading + self.axes().0 * self.chord
    }
}

fn strip(surface: usize, airfoil: usize, inner: Station, outer: Station, mid: Station) -> Strip {
    let (chord_dir, normal) = mid.axes();
    let span_dir = normal.cross(chord_dir);
    let (mut a, mut b) = (inner.quarter_chord(), outer.quarter_chord());
    let mut leading = [inner.leading, outer.leading];
    let mut trailing = [inner.trailing(), outer.trailing()];
    if (b - a).dot(span_dir) < 0.0 {
        std::mem::swap(&mut a, &mut b);
        leading.swap(0, 1);
        trailing.swap(0, 1);
    }
    let chord = (inner.chord + outer.chord) / 2.0;
    Strip {
        surface,
        airfoil,
        center: (a + b) / 2.0,
        a,
        b,
        leading,
        trailing,
        chord_dir,
        normal,
        span_dir,
        chord,
        area: chord * (b - a).length(),
        control: None,
    }
}

impl Strip {
    /// The same strip on the other side of the `x`-`z` plane.
    fn mirrored(&self) -> Strip {
        let m = |v: DVec3| DVec3::new(v.x, -v.y, v.z);
        let chord_dir = m(self.chord_dir);
        let normal = m(self.normal);
        let span_dir = normal.cross(chord_dir);
        // Mirroring reverses the direction from `a` to `b`, so swap them back
        Strip {
            surface: self.surface,
            airfoil: self.airfoil,
            center: m(self.center),
            a: m(self.b),
            b: m(self.a),
            leading: [m(self.leading[1]), m(self.leading[0])],
            trailing: [m(self.trailing[1]), m(self.trailing[0])],
            chord_dir,
            normal,
            span_dir,
            chord: self.chord,
            area: self.area,
            control: None,
        }
    }
}

/// Velocity induced at `point` by a horseshoe vortex of unit strength on `strip`: a bound vortex
/// from `a` to `b`, with trailing vortices along the `-x` axis. The bound vortex does not induce
/// a velocity on its own midpoint.
fn horseshoe(point: DVec3, strip: &Strip, own: bool) -> DVec3 {
    // Regularizes the velocity close to vortex lines, such as where the wake of the wing
    // passes the tail
    let core = 0.1 * (strip.b - strip.a).length();
    let downstream = -DVec3::X;
    let mut velocity =
        trailing(point, strip.b, downstream, core) - trailing(point, strip.a, downstream, core);
    if !own {
        velocity += segment(point, strip.a, strip.b, core);
    }
    velocity
}

/// Velocity induced by a unit vortex segment from `p1` to `p2` (Biot-Savart).
fn segment(point: DVec3, p1: DVec3, p2: DVec3, core: f64) -> DVec3 {
    let r1 = point - p1;
    let r2 = point - p2;
    let r0 = p2 - p1;
    let cross = r1.cross(r2);
    let denominator = cross.length_squared() + (core * r0.length()).powi(2);
    let (l1, l2) = (r1.length(), r2.length());
    if denominator < 1e-12 || l1 < 1e-9 || l2 < 1e-9 {
        return DVec3::ZERO;
    }
    cross * (r0.dot(r1 / l1 - r2 / l2) / (4.0 * PI * denominator))
}

/// Velocity induced by a unit semi-infinite vortex from `start` in the direction `dir`.
fn trailing(point: DVec3, start: DVec3, dir: DVec3, core: f64) -> DVec3 {
    let r = point - start;
    let length = r.length();
    if length < 1e-9 {
        return DVec3::ZERO;
    }
    let denominator = dir.cross(r).length_squared() + core * core;
    dir.cross(r) * ((1.0 + dir.dot(r / length)) / (4.0 * PI * denominator))
}

fn interpolate_profile(profile: &[(f64, f64)], t: f64) -> f64 {
    match profile.iter().position(|(s, _)| *s >= t) {
        None => profile.last().map_or(0.0, |(_, r)| *r),
        Some(0) => profile[0].1,
        Some(i) => {
            let ((s0, r0), (s1, r1)) = (profile[i - 1], profile[i]);
            r0 + (r1 - r0) * (t - s0) / (s1 - s0).max(1e-9)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::config::AircraftConfig;

    /// A rectangular wing of aspect ratio 8 with a thin, symmetric airfoil.
    fn wing(cells: usize) -> Aerodynamics {
        let config = AircraftConfig::parse(&format!(
            r#"(
                name: "wing", mass: 1.0, center_of_gravity: (0.0, 0.0, 0.0),
                inertia: (ixx: 1.0, iyy: 1.0, izz: 1.0),
                airfoils: {{ "thin": Parametric(lift_slope: 6.283185, cd_min: 0.0, cd_k: 0.0,
                    stall_positive: 30.0, stall_negative: -30.0) }},
                surfaces: [(name: "wing", airfoil: "thin", root: (0.25, 0.0, 0.0), root_chord: 1.0,
                    mirror: true, sections: [(span: 4.0, tip_chord: 1.0, cells: {cells})])],
                actuators: [],
            )"#
        ))
        .unwrap();
        Aerodynamics::new(&config, &[]).unwrap()
    }

    fn coefficients(aero: &mut Aerodynamics, alpha: f64) -> (f64, f64) {
        // Flying forward, with the air coming from ahead and below
        let speed = 20.0;
        let flow = Flow {
            velocity: DVec3::new(-alpha.cos(), 0.0, -alpha.sin()) * speed,
            rates: DVec3::ZERO,
            density: 1.225,
            actuators: &[],
        };
        let loads = aero.loads(&flow);
        let q = 0.5 * 1.225 * speed * speed * 8.0;
        let lift_dir = DVec3::new(alpha.sin(), 0.0, -alpha.cos());
        let drag_dir = -DVec3::new(alpha.cos(), 0.0, alpha.sin());
        (loads.force.dot(lift_dir) / q, loads.force.dot(drag_dir) / q)
    }

    #[test]
    fn lift_slope_of_finite_wing() {
        let mut aero = wing(16);
        let alpha = 4f64.to_radians();
        let (cl, cdi) = coefficients(&mut aero, alpha);
        // Lifting line theory gives about 4.9 per radian for a rectangular wing of aspect ratio 8
        let slope = cl / alpha;
        assert!((4.6..5.2).contains(&slope), "lift slope {slope}");
        // Induced drag close to that of an elliptic wing, cl² / (π AR), but a bit more
        let elliptic = cl * cl / (PI * 8.0);
        assert!(
            cdi > elliptic * 0.98 && cdi < elliptic * 1.25,
            "{cdi} vs {elliptic}"
        );
    }

    #[test]
    fn lift_falls_off_towards_the_tips() {
        let mut aero = wing(16);
        let speed = 20.0;
        let alpha = 5f64.to_radians();
        let flow = Flow {
            velocity: DVec3::new(-alpha.cos(), 0.0, -alpha.sin()) * speed,
            rates: DVec3::ZERO,
            density: 1.225,
            actuators: &[],
        };
        let loads = aero.loads(&flow);
        // Strips alternate between the right and left side, from the root outwards
        let root = loads.strips[0].lift.length();
        let tip = loads.strips[30].lift.length();
        assert!(tip < root * 0.75, "root {root} tip {tip}");
        let (right, left) = (
            loads.strips[10].lift.length(),
            loads.strips[11].lift.length(),
        );
        assert!(
            (right - left).abs() < 1e-6 * right,
            "asymmetric {right} {left}"
        );
        // Lift up, centered on the wing
        assert!(loads.force.z < 0.0);
        assert!(loads.moment.x.abs() < 1e-6 * loads.force.length());
    }

    #[test]
    fn converges_with_more_cells() {
        let alpha = 4f64.to_radians();
        let (coarse, _) = coefficients(&mut wing(8), alpha);
        let (fine, _) = coefficients(&mut wing(24), alpha);
        assert!((coarse - fine).abs() < 0.02 * fine, "{coarse} vs {fine}");
    }

    #[test]
    fn rolling_is_damped() {
        let mut aero = wing(12);
        let flow = Flow {
            velocity: DVec3::new(-20.0, 0.0, 0.0),
            rates: DVec3::new(1.0, 0.0, 0.0),
            density: 1.225,
            actuators: &[],
        };
        let loads = aero.loads(&flow);
        assert!(loads.moment.x < 0.0, "roll moment {}", loads.moment.x);
    }
}
