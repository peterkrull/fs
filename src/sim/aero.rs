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
//! The trailing vortices leave along the flow, and are realigned with it when it turns, such as
//! with the angle of attack or the sideslip. This matters for the tail, which sees less downwash
//! the further the wake of the wing passes from it. Downstream of a strip, its trailing vortices
//! have the strength it had when they were shed, so a tail sees the downwash of the wing from
//! the moment before, which damps pitching.
//!
//! Beyond the stall, where the lift of a section falls with its angle of attack, the lifting
//! line has many solutions, among them a zigzag of strong and weak strips along the span. The
//! vortex strengths of stalled strips are smoothed along the span to pick the smooth solution.
//!
//! Each strip sees the flow in the section across its bound vortex, after simple sweep theory:
//! the flow along a swept bound vortex does not add to its lift. This lowers the lift slope of
//! swept surfaces, which the lifting line alone does not, and makes a swept wing roll away from
//! a sideslip like one with dihedral.
//!
//! Surfaces and bodies behind a rotor are in its slipstream, which momentum theory describes:
//! the air speeds up through the disc, and further behind it to twice that, as the stream
//! contracts.
//!
//! Bodies are divided into cells along their length, each with drag from the flow across and
//! along it. After slender body theory, the flow across a body also pushes on it where it
//! widens, and pulls where it narrows. On a closed body these forces cancel, but turn it across
//! the flow (the Munk moment), which makes a fuselage destabilizing in pitch and yaw.

use std::{collections::VecDeque, f64::consts::PI};

use bevy::math::{DMat3, DVec3};

use super::{
    airfoil::{AirfoilModel, Flap},
    config::{AircraftConfig, Mirror},
};

/// Iterations of the lifting line per step, starting from the previous solution.
const ITERATIONS: usize = 3;
/// Iterations to start from scratch.
const INITIAL_ITERATIONS: usize = 200;
/// The wake is realigned with the flow once their directions differ by more than this many
/// radians.
const WAKE_TOLERANCE: f64 = 1.0 * PI / 180.0;
/// Below this airspeed in m/s, the wake keeps its direction.
const WAKE_MIN_SPEED: f64 = 1.0;
/// Strips further downstream of another strip than this many of its chords see its trailing
/// vortices as they were shed, see [`Lagged`].
const LAG_CHORDS: f64 = 1.0;
/// The longest the trailing vortices take to reach a strip downstream, in seconds.
const MAX_LAG: f64 = 0.3;
/// Points along each strip at which the slipstreams are sampled.
const SLIPSTREAM_SAMPLES: usize = 5;
/// How strongly the vortex strengths of stalled strips are smoothed along the span, relative to
/// how much the lift of their section falls with the angle of attack.
const STALL_SMOOTHING: f64 = 4.0;
/// Step in angle of attack in radians to find the lift slope of stalled sections.
const SLOPE_STEP: f64 = 1e-3;

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
    /// Direction of the bound vortex from `a` to `b`, which is swept with the surface.
    pub bound: DVec3,
    /// Chord direction and lift normal of the section across the bound vortex, in which the
    /// flow over a swept surface is evaluated.
    pub section_chord_dir: DVec3,
    pub section_normal: DVec3,
    /// Cosine of the sweep, between the bound vortex and the spanwise direction.
    pub sweep_cos: f64,
    /// Streamwise chord.
    pub chord: f64,
    /// Planform area, the chord times the width of the strip across it.
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
    /// How much the cross section grows over the cell, in m², times the apparent mass factor of
    /// the body.
    pub growth: f64,
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
    /// How the wind changes across the aircraft: the change of its velocity with the position
    /// relative to the center of gravity, as gusts hit one part harder than another.
    pub gradient: DMat3,
    /// Angular velocity of the aircraft.
    pub rates: DVec3,
    pub density: f64,
    /// Positions of the actuators.
    pub actuators: &'a [f64],
    /// Time since the previous step, in seconds.
    pub dt: f64,
    pub slipstreams: &'a [Slipstream],
}

/// The stream of air a rotor drives, after momentum theory. Positions and directions are in
/// the model frame.
#[derive(Debug, Clone, Copy, Default)]
pub struct Slipstream {
    /// Center of the disc.
    pub position: DVec3,
    /// Direction of the thrust, opposite to the slipstream.
    pub axis: DVec3,
    pub radius: f64,
    /// How much the rotor speeds up the air through its disc, in m/s.
    pub induced: f64,
    /// Speed of the air flowing into the disc along its axis.
    pub inflow: f64,
}

impl Slipstream {
    /// The velocity the slipstream adds to the air at a point.
    pub fn velocity_at(&self, point: DVec3) -> DVec3 {
        let offset = point - self.position;
        let behind = -offset.dot(self.axis);
        if self.induced <= 0.0 || behind <= 0.0 {
            return DVec3::ZERO;
        }
        // The air keeps speeding up behind the disc, to twice its speed through the disc, and
        // the stream contracts as it does
        let speedup = self.induced * (1.0 + behind / behind.hypot(self.radius));
        let radius = self.radius * ((self.inflow + self.induced) / (self.inflow + speedup)).sqrt();
        // With a soft edge, so that what is at the edge does not jump in and out of it
        let distance = (offset + self.axis * behind).length();
        let inside = 1.0 - ((distance / radius - 0.9) / 0.2).clamp(0.0, 1.0);
        -self.axis * (speedup * inside * inside * (3.0 - 2.0 * inside))
    }
}

impl Flow<'_> {
    /// The velocity the slipstreams add at a point in the model frame.
    fn slipstream(&self, point: DVec3) -> DVec3 {
        self.slipstreams.iter().map(|s| s.velocity_at(point)).sum()
    }
}

impl Flow<'_> {
    /// Velocity of the air relative to a point of the aircraft, given relative to the center
    /// of gravity.
    fn at(&self, offset: DVec3) -> DVec3 {
        self.velocity + self.gradient * offset - self.rates.cross(offset)
    }
}

pub struct Aerodynamics {
    pub strips: Vec<Strip>,
    pub cells: Vec<BodyCell>,
    pub airfoils: Vec<AirfoilModel>,
    pub drag_elements: Vec<(DVec3, DVec3)>,
    center_of_gravity: DVec3,
    /// Direction the trailing vortices leave in, for which the influences were computed.
    wake: DVec3,
    /// Velocity induced at the center of strip `i` by a unit vortex on strip `j`, at `i * n + j`,
    /// leaving out the trailing vortices in `lagged`.
    influence: Vec<DVec3>,
    lagged: Vec<Lagged>,
    /// Relaxation of the iteration of each strip, against its influence on itself.
    relaxation: Vec<f64>,
    /// Upwash induced at each strip by its own unit vortex, against its lift, which is positive.
    self_influence: Vec<f64>,
    /// The strips on either side of each strip along the span of its surface, if any.
    neighbors: Vec<[Option<usize>; 2]>,
    /// Vortex strength of each strip.
    circulation: Vec<f64>,
    /// Recent vortex strengths of all strips, `n` per step, and the time of each step.
    history: VecDeque<f64>,
    history_times: VecDeque<f64>,
    time: f64,
    /// Velocity induced at each strip by the lagged trailing vortices in this step.
    lagged_induced: Vec<DVec3>,
    converged: bool,
}

/// The trailing vortices of strip `source`, which reach strip `target` `distance` downstream,
/// so that they were shed that long ago.
struct Lagged {
    target: usize,
    source: usize,
    distance: f64,
    /// Velocity induced at the target by unit trailing vortices.
    velocity: DVec3,
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
                if let Some(root) = section.incidence {
                    incidence = root.to_radians();
                }
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
            let apparent_mass = apparent_mass_factor(body.length / (2.0 * max_radius).max(1e-9));
            let area = |t: f64| PI * radius_at(t).powi(2);
            for cell in 0..body.cells {
                let t = (cell as f64 + 0.5) / body.cells as f64;
                let (front, back) = (
                    cell as f64 / body.cells as f64,
                    (cell + 1) as f64 / body.cells as f64,
                );
                cells.push(BodyCell {
                    body: index,
                    center: nose - DVec3::X * (body.length * t),
                    length,
                    radius: radius_at(t),
                    cd_crossflow: body.cd_crossflow,
                    axial_area: body.cd_axial * PI * max_radius * max_radius / body.cells as f64,
                    growth: apparent_mass * (area(back) - area(front)),
                });
            }
        }

        let drag_elements = config
            .drag
            .iter()
            .map(|d| (DVec3::from(d.position), DVec3::from(d.cd_area)))
            .collect();

        let n = strips.len();
        let neighbors = strips
            .iter()
            .enumerate()
            .map(|(i, strip)| {
                let at = |point: DVec3, end: fn(&Strip) -> DVec3| {
                    strips.iter().enumerate().position(|(j, other)| {
                        j != i
                            && other.surface == strip.surface
                            && (end(other) - point).length() < 1e-6
                    })
                };
                [at(strip.a, |s| s.b), at(strip.b, |s| s.a)]
            })
            .collect();
        let mut aerodynamics = Self {
            strips,
            cells,
            airfoils,
            drag_elements,
            center_of_gravity,
            wake: -DVec3::X,
            influence: Vec::new(),
            lagged: Vec::new(),
            relaxation: Vec::new(),
            self_influence: Vec::new(),
            neighbors,
            circulation: vec![0.0; n],
            history: VecDeque::new(),
            history_times: VecDeque::new(),
            time: 0.0,
            lagged_induced: vec![DVec3::ZERO; n],
            converged: false,
        };
        aerodynamics.align_wake(-DVec3::X);
        Ok(aerodynamics)
    }

    /// Compute the influences of the strips on each other, with the trailing vortices leaving
    /// in the direction `downstream`.
    fn align_wake(&mut self, downstream: DVec3) {
        let strips = &self.strips;
        let n = strips.len();
        self.wake = downstream;
        self.influence.clear();
        self.influence.resize(n * n, DVec3::ZERO);
        self.lagged.clear();
        for (i, target) in strips.iter().enumerate() {
            for (j, source) in strips.iter().enumerate() {
                let width = (source.b - source.a).length();
                let (bound, trailing) =
                    horseshoe(target.center, source, i == j, downstream, 0.1 * width);
                let distance = (target.center - source.center).dot(downstream);
                self.influence[i * n + j] = bound;
                match distance > LAG_CHORDS * source.chord {
                    // Far downstream, the trailing vortices of a strip stand for the part of the
                    // vortex sheet behind it, as wide as the strip. Seen from closer than that,
                    // such as by a tail in the plane of the wing, they would make the downwash
                    // of the sheet jagged, enough to stall single strips of the tail, so their
                    // cores are as wide as the strip.
                    true => self.lagged.push(Lagged {
                        target: i,
                        source: j,
                        distance,
                        velocity: horseshoe(target.center, source, false, downstream, width).1,
                    }),
                    false => self.influence[i * n + j] += trailing,
                }
            }
        }
        self.self_influence = strips
            .iter()
            .enumerate()
            .map(|(i, s)| (-self.influence[i * n + i].dot(s.section_normal)).max(0.0))
            .collect();
        // Damp each strip against its own influence on itself, which would otherwise make the
        // iteration overshoot for strips that are narrow compared to their chord
        self.relaxation = strips
            .iter()
            .zip(&self.self_influence)
            .map(|(s, influence)| {
                let gain =
                    0.5 * s.chord * s.sweep_cos * self.airfoils[s.airfoil].lift_slope() * influence;
                1.0 / (1.0 + gain)
            })
            .collect();
    }

    /// Forget the vortex strengths, for example after the aircraft was teleported.
    pub fn reset(&mut self) {
        self.circulation.fill(0.0);
        self.history.clear();
        self.history_times.clear();
        self.converged = false;
    }

    /// The velocity induced by the lagged trailing vortices, with the strengths they were shed
    /// with, or with the current strengths when there is no history yet.
    fn update_lagged(&mut self, airspeed: f64) {
        let mut induced = std::mem::take(&mut self.lagged_induced);
        induced.fill(DVec3::ZERO);
        for lagged in &self.lagged {
            let delay = (lagged.distance / airspeed.max(1e-3)).min(MAX_LAG);
            let circulation = self.past_circulation(lagged.source, self.time - delay);
            induced[lagged.target] += lagged.velocity * circulation;
        }
        self.lagged_induced = induced;
    }

    /// Vortex strength of a strip at a time, interpolated in the history.
    fn past_circulation(&self, strip: usize, time: f64) -> f64 {
        let n = self.strips.len();
        let times = &self.history_times;
        let at = |k: usize| self.history[k * n + strip];
        match times.partition_point(|t| *t <= time) {
            _ if times.is_empty() => self.circulation[strip],
            0 => at(0),
            k if k == times.len() => at(k - 1),
            k => {
                let t = (time - times[k - 1]) / (times[k] - times[k - 1]).max(1e-12);
                at(k - 1) + (at(k) - at(k - 1)) * t
            }
        }
    }

    pub fn loads(&mut self, flow: &Flow) -> AeroLoads {
        let n = self.strips.len();
        self.time += flow.dt;
        let airspeed = flow.velocity.length();
        if airspeed > WAKE_MIN_SPEED {
            let downstream = flow.velocity / airspeed;
            if downstream.angle_between(self.wake) > WAKE_TOLERANCE {
                self.align_wake(downstream);
            }
        }
        // Without a history, the trailing vortices downstream are as strong as they are now
        let history = !self.history_times.is_empty();
        if history {
            self.update_lagged(airspeed);
        }

        // Along each strip, which may be partly in a slipstream
        let freestream: Vec<DVec3> = self
            .strips
            .iter()
            .map(|s| {
                let slipstream = match flow.slipstreams.is_empty() {
                    true => DVec3::ZERO,
                    false => {
                        (0..SLIPSTREAM_SAMPLES)
                            .map(|k| {
                                let t = (k as f64 + 0.5) / SLIPSTREAM_SAMPLES as f64;
                                flow.slipstream(s.a + (s.b - s.a) * t)
                            })
                            .sum::<DVec3>()
                            / SLIPSTREAM_SAMPLES as f64
                    }
                };
                flow.at(s.center - self.center_of_gravity) + slipstream
            })
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
        let mut rates = vec![0.0; n];
        for _ in 0..iterations {
            if !history {
                self.update_lagged(airspeed);
            }
            for i in 0..n {
                let section = self.section(i, freestream[i], deflections[i]);
                let strip = &self.strips[i];
                let chord = strip.chord * strip.sweep_cos;
                // Beyond the stall, where the lift falls with the angle of attack, the lifting
                // line has many solutions, such as a zigzag along the span. Smoothing the vortex
                // strengths there, as much as the lift falls, picks the smooth one.
                let smoothing = STALL_SMOOTHING
                    * 0.5
                    * chord
                    * (-section.slope).max(0.0)
                    * self.self_influence[i];
                let circulation = self.circulation[i];
                let [previous, next] =
                    self.neighbors[i].map(|j| j.map_or(circulation, |j| self.circulation[j]));
                targets[i] = 0.5 * section.speed * chord * section.cl
                    + smoothing * (previous + next - 2.0 * circulation);
                rates[i] = 1.0 / (1.0 / self.relaxation[i] + 2.0 * smoothing);
            }
            for ((circulation, rate), target) in
                self.circulation.iter_mut().zip(&rates).zip(&targets)
            {
                *circulation += rate * (target - *circulation);
            }
        }
        self.converged = true;
        self.history_times.push_back(self.time);
        self.history.extend(&self.circulation);
        while self
            .history_times
            .front()
            .is_some_and(|t| *t < self.time - MAX_LAG - 0.01)
        {
            self.history_times.pop_front();
            self.history.drain(..n);
        }

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
            // The section across the bound vortex has a chord shorter by the cosine of the sweep,
            // and is as long as the bound vortex, which together make up the planform area
            let half_rho = 0.5 * flow.density * strip.area;
            let q = half_rho * section.speed * section.speed;
            let (lift, moment) = match section.speed > 1e-6 {
                true => (
                    (section.across / section.speed).cross(strip.bound) * (q * section.cl),
                    strip.bound * (q * strip.chord * strip.sweep_cos * section.cm),
                ),
                false => (DVec3::ZERO, DVec3::ZERO),
            };
            // Skin friction acts along the whole flow, while the drag that comes with lift and
            // separation acts across the bound vortex
            let drag = section.air * (half_rho * section.air.length() * section.cd_zero)
                + section.across * (half_rho * section.speed * (section.cd - section.cd_zero));
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
            // Over the cross section of the cell, of which the slipstream may cover part
            let slipstream = match flow.slipstreams.is_empty() {
                true => DVec3::ZERO,
                false => {
                    let r = cell.radius * std::f64::consts::FRAC_1_SQRT_2;
                    [
                        DVec3::ZERO,
                        DVec3::Y * r,
                        -DVec3::Y * r,
                        DVec3::Z * r,
                        -DVec3::Z * r,
                    ]
                    .iter()
                    .map(|offset| flow.slipstream(cell.center + *offset))
                    .sum::<DVec3>()
                        / 5.0
                }
            };
            let air = flow.at(offset) + slipstream;
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
            // The flow across a widening cell pushes it along with the flow, as it flows along
            let potential = across * (flow.density * cell.growth * -axial);
            let force = crossflow + along + potential;
            loads.force += force;
            loads.moment += offset.cross(force);
            loads.cells.push(force);
        }

        for (position, cd_area) in &self.drag_elements {
            let offset = *position - self.center_of_gravity;
            let air = flow.at(offset) + flow.slipstream(*position);
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
            .sum::<DVec3>()
            + self.lagged_induced[i];
        let air = freestream + induced;
        // Flow along the bound vortex does not change the lift of the section (simple sweep
        // theory), which makes the lift of swept surfaces depend on the sideslip
        let across = air - strip.bound * air.dot(strip.bound);
        let along = across.dot(strip.section_chord_dir);
        let up = across.dot(strip.section_normal);
        let speed = across.length();
        let alpha = up.atan2(along);

        // The flap turns about its hinge, which follows the sweep, so its deflection is the same
        // in the section across it
        let (flap_alpha, flap_cm, flap_cd) = strip
            .control
            .as_ref()
            .map_or((0.0, 0.0, 0.0), |c| c.flap.effect(deflection));
        let airfoil = &self.airfoils[strip.airfoil];
        let c = airfoil.swept_coefficients(alpha, strip.sweep_cos, flap_alpha);
        // Only needed where the flow separates, as the lift may fall there
        let slope = match c.separation > 1e-3 {
            true => {
                let ahead =
                    airfoil.swept_coefficients(alpha + SLOPE_STEP, strip.sweep_cos, flap_alpha);
                (ahead.cl - c.cl) / SLOPE_STEP
            }
            false => 0.0,
        };
        Section {
            slope,
            air,
            across,
            speed,
            alpha,
            cl: c.cl,
            cd: c.cd + flap_cd,
            cd_zero: airfoil.cd_zero(),
            cm: c.cm + flap_cm * (1.0 - c.separation),
            separation: c.separation,
        }
    }
}

struct Section {
    /// Local velocity of the air, including the velocity induced by the vortices.
    air: DVec3,
    /// The part of it across the bound vortex, and its magnitude.
    across: DVec3,
    speed: f64,
    /// Angle of attack in the section across the bound vortex.
    alpha: f64,
    cl: f64,
    /// Lift slope of the section per radian, where the flow separates.
    slope: f64,
    cd: f64,
    cd_zero: f64,
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
    Strip::new(
        surface, airfoil, a, b, leading, trailing, chord_dir, normal, chord,
    )
}

impl Strip {
    fn new(
        surface: usize,
        airfoil: usize,
        a: DVec3,
        b: DVec3,
        leading: [DVec3; 2],
        trailing: [DVec3; 2],
        chord_dir: DVec3,
        normal: DVec3,
        chord: f64,
    ) -> Strip {
        let bound = (b - a).normalize();
        let sweep_cos = chord_dir.cross(bound).length();
        let section_chord_dir = (chord_dir - bound * chord_dir.dot(bound)).normalize();
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
            bound,
            section_chord_dir,
            section_normal: section_chord_dir.cross(bound),
            sweep_cos,
            chord,
            area: chord * chord_dir.cross(b - a).length(),
            control: None,
        }
    }

    /// The same strip on the other side of the `x`-`z` plane.
    fn mirrored(&self) -> Strip {
        let m = |v: DVec3| DVec3::new(v.x, -v.y, v.z);
        // Mirroring reverses the direction from `a` to `b`, so swap them back
        Strip::new(
            self.surface,
            self.airfoil,
            m(self.b),
            m(self.a),
            [m(self.leading[1]), m(self.leading[0])],
            [m(self.trailing[1]), m(self.trailing[0])],
            m(self.chord_dir),
            m(self.normal),
            self.chord,
        )
    }
}

/// Velocity induced at `point` by a horseshoe vortex of unit strength on `strip`: a bound vortex
/// from `a` to `b`, and trailing vortices, given separately. The trailing vortices run along the
/// chord to the trailing edge, and leave from there in the direction `downstream`. The bound
/// vortex does not induce a velocity on its own midpoint. The vortex lines have a core radius,
/// which regularizes the velocity close to them.
fn horseshoe(
    point: DVec3,
    strip: &Strip,
    own: bool,
    downstream: DVec3,
    core: f64,
) -> (DVec3, DVec3) {
    // Leaving along the flow right from the bound vortex would skew the trailing vortices next
    // to the neighbouring strips in a sideslip, and make even a straight wing roll strongly
    let leg = |start: DVec3, edge: DVec3| {
        segment(point, start, edge, core) + trailing(point, edge, downstream, core)
    };
    let trailing = leg(strip.b, strip.trailing[1]) - leg(strip.a, strip.trailing[0]);
    let bound = match own {
        true => DVec3::ZERO,
        false => segment(point, strip.a, strip.b, core),
    };
    (bound, trailing)
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

/// Lamb's apparent mass factor `k2 - k1` of a prolate spheroid, by its length over its diameter,
/// which scales the forces of slender body theory to those of bodies of finite length.
fn apparent_mass_factor(fineness: f64) -> f64 {
    if fineness <= 1.0 + 1e-6 {
        return 0.0;
    }
    let e = (1.0 - 1.0 / (fineness * fineness)).sqrt();
    let log = ((1.0 + e) / (1.0 - e)).ln();
    let alpha = 2.0 * (1.0 - e * e) / e.powi(3) * (0.5 * log - e);
    let beta = 1.0 / (e * e) - (1.0 - e * e) / (2.0 * e.powi(3)) * log;
    beta / (2.0 - beta) - alpha / (2.0 - alpha)
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
        swept_wing(cells, 4.0, 0.0)
    }

    /// A wing of constant chord 1 with a thin, symmetric airfoil, of span `2 * half_span`.
    fn swept_wing(cells: usize, half_span: f64, sweep: f64) -> Aerodynamics {
        let config = AircraftConfig::parse(&format!(
            r#"(
                name: "wing", mass: 1.0, center_of_gravity: (0.0, 0.0, 0.0),
                inertia: (ixx: 1.0, iyy: 1.0, izz: 1.0),
                airfoils: {{ "thin": Parametric(lift_slope: 6.283185, cd_min: 0.0, cd_k: 0.0,
                    stall_positive: 30.0, stall_negative: -30.0) }},
                surfaces: [(name: "wing", airfoil: "thin", root: (0.25, 0.0, 0.0), root_chord: 1.0,
                    mirror: true, sections: [(span: {half_span}, tip_chord: 1.0, sweep: {sweep},
                    cells: {cells})])],
                actuators: [],
            )"#
        ))
        .unwrap();
        Aerodynamics::new(&config, &[]).unwrap()
    }

    /// Flying forward and sideslipping to the right, with the air coming from ahead, below and
    /// the right.
    fn flow(alpha: f64, beta: f64) -> Flow<'static> {
        let (sa, ca) = alpha.sin_cos();
        let (sb, cb) = beta.sin_cos();
        Flow {
            velocity: -DVec3::new(ca * cb, sb, sa * cb) * 20.0,
            gradient: DMat3::ZERO,
            rates: DVec3::ZERO,
            density: 1.225,
            actuators: &[],
            dt: 0.001,
            slipstreams: &[],
        }
    }

    fn coefficients(aero: &mut Aerodynamics, alpha: f64) -> (f64, f64) {
        let speed = 20.0;
        let loads = aero.loads(&flow(alpha, 0.0));
        let area: f64 = aero.strips.iter().map(|s| s.area).sum();
        let q = 0.5 * 1.225 * speed * speed * area;
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
            gradient: DMat3::ZERO,
            rates: DVec3::ZERO,
            density: 1.225,
            actuators: &[],
            dt: 0.001,
            slipstreams: &[],
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

    /// Sweep lowers the lift slope, which the formula of Helmbold and DATCOM gives within a few
    /// percent of panel methods.
    #[test]
    fn sweep_lowers_the_lift_slope() {
        let alpha = 4f64.to_radians();
        for sweep in [30.0, 45.0] {
            let (cl, _) = coefficients(&mut swept_wing(20, 3.0, sweep), alpha);
            let slope = cl / alpha;
            let (aspect, tan) = (6.0, f64::to_radians(sweep).tan());
            let expected =
                2.0 * PI * aspect / (2.0 + (aspect * aspect * (1.0 + tan * tan) + 4.0).sqrt());
            assert!(
                (slope / expected - 1.0).abs() < 0.12,
                "sweep {sweep}: lift slope {slope} vs {expected}"
            );
        }
    }

    /// The wing sweeping into a sideslip flies across a lower sweep, and the other one across a
    /// higher sweep, so a swept wing rolls away from the sideslip like one with dihedral.
    #[test]
    fn sweep_acts_as_dihedral() {
        let roll = |sweep: f64| {
            let mut aero = swept_wing(16, 3.0, sweep);
            aero.loads(&flow(5f64.to_radians(), 5f64.to_radians()))
                .moment
                .x
        };
        let (straight, swept) = (roll(0.0), roll(30.0));
        println!("rolling moment in sideslip: straight {straight:.1}, swept {swept:.1}");
        // The skewed wake of the straight wing rolls it a little the other way
        assert!(
            swept - straight < -5.0 * straight.abs(),
            "{straight} vs {swept}"
        );
    }

    /// The wing of [`wing`], with a tail four chords behind it, in its plane.
    fn wing_and_tail() -> Aerodynamics {
        let config = AircraftConfig::parse(
            r#"(
                name: "wing and tail", mass: 1.0, center_of_gravity: (0.0, 0.0, 0.0),
                inertia: (ixx: 1.0, iyy: 1.0, izz: 1.0),
                airfoils: { "thin": Parametric(lift_slope: 6.283185, cd_min: 0.0, cd_k: 0.0,
                    stall_positive: 30.0, stall_negative: -30.0) },
                surfaces: [
                    (name: "wing", airfoil: "thin", root: (0.25, 0.0, 0.0), root_chord: 1.0,
                        mirror: true, sections: [(span: 4.0, tip_chord: 1.0, cells: 12)]),
                    (name: "tail", airfoil: "thin", root: (-3.75, 0.0, 0.0), root_chord: 0.5,
                        mirror: true, sections: [(span: 1.0, tip_chord: 0.5, cells: 4)]),
                ],
                actuators: [],
            )"#,
        )
        .unwrap();
        Aerodynamics::new(&config, &[]).unwrap()
    }

    /// The downwash at the tail, in radians, and the pitching moment, in steady flow.
    fn downwash(aero: &mut Aerodynamics, alpha: f64) -> f64 {
        aero.reset();
        let loads = aero.loads(&flow(alpha, 0.0));
        let tail: Vec<f64> = aero
            .strips
            .iter()
            .zip(&loads.strips)
            .filter(|(strip, _)| strip.surface == 1)
            .map(|(_, load)| load.alpha)
            .collect();
        alpha - tail.iter().sum::<f64>() / tail.len() as f64
    }

    /// The wake leaves along the flow, so at a higher angle of attack it passes further above
    /// the tail, which sees less downwash for the lift of the wing.
    #[test]
    fn wake_rises_above_the_tail() {
        let mut aero = wing_and_tail();
        let (low, high) = (4f64.to_radians(), 12f64.to_radians());
        let gradient_low = downwash(&mut aero, low) / low;
        let gradient_high = downwash(&mut aero, high) / high;
        let wake = -DVec3::new(high.cos(), 0.0, high.sin());
        assert!(aero.wake.angle_between(wake) < WAKE_TOLERANCE);
        // Only a little, as the wake rises by less than a tenth of the span
        assert!(
            gradient_high < 0.99 * gradient_low,
            "downwash per angle of attack {gradient_low} at 4 deg, {gradient_high} at 12 deg"
        );
    }

    /// A tail in the plane of the wing, with strips narrower than those of the wing, which the
    /// trailing vortices of the wing pass close to, sees a smooth downwash along its span.
    #[test]
    fn smooth_downwash_on_a_tail_in_the_wake() {
        let config = AircraftConfig::parse(
            r#"(
                name: "wing and tail", mass: 1.0, center_of_gravity: (-0.35, 0.0, 0.0),
                inertia: (ixx: 1.0, iyy: 1.0, izz: 1.0),
                airfoils: {
                    "cambered": Parametric(lift_slope: 6.283185, zero_lift_angle: -4.0,
                        cd_min: 0.0, cd_k: 0.0, stall_positive: 13.0, stall_negative: -8.0),
                    "thin": Parametric(lift_slope: 6.283185, cd_min: 0.0, cd_k: 0.0,
                        stall_positive: 11.0, stall_negative: -11.0),
                },
                surfaces: [
                    (name: "wing", airfoil: "cambered", root: (-0.25, 0.07, 0.0), root_chord: 0.3,
                        mirror: true, sections: [(span: 0.53, tip_chord: 0.15, cells: 15)]),
                    (name: "tail", airfoil: "thin", root: (-0.93, 0.0, 0.0), root_chord: 0.15,
                        mirror: true, sections: [(span: 0.25, tip_chord: 0.11, cells: 8)]),
                ],
                actuators: [],
            )"#,
        )
        .unwrap();
        let mut aero = Aerodynamics::new(&config, &[]).unwrap();
        // With the wake of the cambered wing in the plane of the tail
        let steady = flow(0.0, 0.0);
        let loads = (0..300).map(|_| aero.loads(&steady)).last().unwrap();
        // Along the right half of the tail, from the root out
        let mut tail: Vec<(f64, f64)> = aero
            .strips
            .iter()
            .zip(&loads.strips)
            .filter(|(strip, _)| strip.surface == 1 && strip.center.y > 0.0)
            .map(|(strip, load)| (strip.center.y, load.alpha.to_degrees()))
            .collect();
        tail.sort_by(|a, b| a.0.total_cmp(&b.0));
        for three in tail.windows(3) {
            let kink = (three[0].1 - 2.0 * three[1].1 + three[2].1).abs();
            assert!(kink < 1.0, "angles of attack along the tail: {tail:?}");
        }
    }

    /// The tail sees the downwash the wing shed a moment ago, so right after the angle of
    /// attack increases, the tail lifts more than it will once the wake has caught up.
    #[test]
    fn tail_sees_the_downwash_of_the_past() {
        let mut aero = wing_and_tail();
        let (before, after) = (flow(2f64.to_radians(), 0.0), flow(6f64.to_radians(), 0.0));
        for _ in 0..500 {
            aero.loads(&before);
        }
        let first = aero.loads(&after).moment.y;
        // The wake takes 4 m at 20 m/s, 200 steps, to reach the tail
        for _ in 0..300 {
            aero.loads(&after);
        }
        let steady = aero.loads(&after).moment.y;
        assert!(
            first < steady - 0.05 * steady.abs(),
            "pitching moment {first} at first, {steady} once steady"
        );
    }

    /// A spheroid of fineness 10 at an angle of attack turns nose up with the moment Munk found,
    /// `½ ρ V² vol (k2 - k1) sin 2α`.
    #[test]
    fn munk_moment() {
        assert!((apparent_mass_factor(10.0) - 0.9395).abs() < 0.001);
        let profile: Vec<String> = (0..=40)
            .map(|i| {
                let t = i as f64 / 40.0;
                format!("({t}, {})", 0.1 * (1.0 - (2.0 * t - 1.0).powi(2)).sqrt())
            })
            .collect();
        let config = AircraftConfig::parse(&format!(
            r#"(name: "spheroid", mass: 1.0, center_of_gravity: (-1.0, 0.0, 0.0),
                inertia: (ixx: 1.0, iyy: 1.0, izz: 1.0), actuators: [],
                bodies: [(name: "spheroid", nose: (0.0, 0.0, 0.0), length: 2.0,
                    profile: [{}], cells: 40)])"#,
            profile.join(", ")
        ))
        .unwrap();
        let mut aero = Aerodynamics::new(&config, &[]).unwrap();
        let alpha = 5f64.to_radians();
        let moment = aero.loads(&flow(alpha, 0.0)).moment.y;
        let volume = 4.0 / 3.0 * PI * 0.1 * 0.1;
        let expected = 0.5 * 1.225 * 400.0 * volume * 0.9395 * (2.0 * alpha).sin();
        assert!(
            (moment / expected - 1.0).abs() < 0.05,
            "{moment} vs {expected}"
        );
    }

    /// The slipstream of a static rotor speeds up to twice its speed through the disc, and
    /// contracts to `1 / √2` of the disc.
    #[test]
    fn slipstream_after_momentum_theory() {
        let slipstream = Slipstream {
            position: DVec3::ZERO,
            axis: DVec3::X,
            radius: 0.5,
            induced: 5.0,
            inflow: 0.0,
        };
        let far = |offset: f64| slipstream.velocity_at(DVec3::new(-100.0, offset, 0.0));
        assert!((far(0.0) + DVec3::X * 10.0).length() < 0.01, "{}", far(0.0));
        assert!(far(0.32).length() > 9.9 && far(0.4).length() < 1e-9);
        let near = slipstream.velocity_at(DVec3::new(-1e-6, 0.0, 0.0));
        assert!((near.x + 5.0).abs() < 0.01, "{near}");
        // Nothing ahead of the disc
        assert_eq!(
            slipstream.velocity_at(DVec3::new(0.1, 0.0, 0.0)),
            DVec3::ZERO
        );
    }

    /// With the aircraft standing still, the part of a wing in the slipstream of a rotor
    /// ahead of it lifts, and the rest does not.
    #[test]
    fn surfaces_lift_in_the_slipstream() {
        let config = AircraftConfig::parse(
            r#"(name: "wing", mass: 1.0, center_of_gravity: (0.0, 0.0, 0.0),
                inertia: (ixx: 1.0, iyy: 1.0, izz: 1.0), actuators: [],
                airfoils: { "thin": Parametric(lift_slope: 6.283185, cd_min: 0.01,
                    stall_positive: 15.0, stall_negative: -15.0) },
                surfaces: [(name: "wing", airfoil: "thin", root: (0.25, 0.0, 0.0),
                    root_chord: 1.0, incidence: 5.0, mirror: true,
                    sections: [(span: 4.0, tip_chord: 1.0, cells: 16)])])"#,
        )
        .unwrap();
        let mut aero = Aerodynamics::new(&config, &[]).unwrap();
        let slipstreams = [Slipstream {
            position: DVec3::new(1.0, 0.0, 0.0),
            axis: DVec3::X,
            radius: 0.6,
            induced: 5.0,
            inflow: 0.0,
        }];
        let flow = Flow {
            velocity: DVec3::ZERO,
            slipstreams: &slipstreams,
            ..flow(0.0, 0.0)
        };
        let loads = aero.loads(&flow);
        assert!(loads.force.z < -3.0, "lift {}", -loads.force.z);
        // Outside, only the vortices of the part inside move the air
        let (mut inside, mut outside) = (0.0f64, 0.0f64);
        for (strip, load) in aero.strips.iter().zip(&loads.strips) {
            match strip.a.y.abs().min(strip.b.y.abs()) > 0.6 {
                true => outside = outside.max(load.lift.length()),
                false => inside = inside.max(load.lift.length()),
            }
        }
        assert!(outside < 0.1 * inside, "{outside} outside, {inside} inside");
    }

    /// Beyond the stall, the vortex strengths are smooth along the span, rather than zigzag,
    /// and a rectangular wing stalls from the root outwards.
    #[test]
    fn stall_is_smooth() {
        let config = AircraftConfig::parse(
            r#"(name: "wing", mass: 1.0, center_of_gravity: (0.0, 0.0, 0.0),
                inertia: (ixx: 1.0, iyy: 1.0, izz: 1.0), actuators: [],
                airfoils: { "cambered": Parametric(lift_slope: 5.8, zero_lift_angle: -3.0,
                    cd_min: 0.01, stall_positive: 12.0, stall_negative: -10.0) },
                surfaces: [(name: "wing", airfoil: "cambered", root: (0.25, 0.0, 0.0),
                    root_chord: 1.0, mirror: true,
                    sections: [(span: 4.0, tip_chord: 1.0, cells: 16)])])"#,
        )
        .unwrap();
        for alpha in [13.0, 15.0] {
            let mut aero = Aerodynamics::new(&config, &[]).unwrap();
            let flow = flow(f64::to_radians(alpha), 0.0);
            for _ in 0..1000 {
                aero.loads(&flow);
            }
            let g = &aero.circulation;
            let peak = g.iter().fold(0.0f64, |m, g| m.max(g.abs()));
            let roughness = (0..g.len())
                .filter_map(|i| {
                    let [previous, next] = aero.neighbors[i];
                    Some((g[i] - (g[previous?] + g[next?]) / 2.0).abs() / peak)
                })
                .fold(0.0, f64::max);
            // Attached flow has about 0.04, from the curvature towards the tips
            assert!(roughness < 0.08, "{alpha} deg: roughness {roughness}");
            // The root has stalled, while further out the wing still lifts more
            assert!(g[0] < 0.95 * peak, "{alpha} deg: {g:?}");
        }
    }

    #[test]
    fn rolling_is_damped() {
        let mut aero = wing(12);
        let flow = Flow {
            velocity: DVec3::new(-20.0, 0.0, 0.0),
            gradient: DMat3::ZERO,
            rates: DVec3::new(1.0, 0.0, 0.0),
            density: 1.225,
            actuators: &[],
            dt: 0.001,
            slipstreams: &[],
        };
        let loads = aero.loads(&flow);
        assert!(loads.moment.x < 0.0, "roll moment {}", loads.moment.x);
    }
}
