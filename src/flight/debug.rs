//! Debug views of the simulation: the forces on the aircraft, and the aerodynamic model the
//! simulation uses, drawn over or instead of the 3D model.

use bevy::{math::DVec3, prelude::*};

use super::Flight;
use crate::camera::View;

#[derive(Resource)]
pub struct DebugView {
    /// Arrows of the forces acting on each part of the aircraft.
    pub forces: bool,
    /// The strips, bodies and rotors the simulation models the aircraft with.
    pub schema: bool,
    /// The 3D model of the aircraft.
    pub model: bool,
}

impl Default for DebugView {
    fn default() -> Self {
        Self {
            forces: false,
            schema: false,
            model: true,
        }
    }
}

const LIFT: Color = Color::srgb(0.2, 0.9, 0.3);
const STALLED: Color = Color::srgb(1.0, 0.25, 0.2);
const DRAG: Color = Color::srgb(0.95, 0.2, 0.9);
const BODY: Color = Color::srgb(0.3, 0.6, 1.0);
const THRUST: Color = Color::srgb(1.0, 0.85, 0.1);
const TOTAL: Color = Color::srgb(0.2, 0.95, 0.95);
const OUTLINE: Color = Color::srgb(0.9, 0.9, 0.9);
const CONTROL: Color = Color::srgb(1.0, 0.55, 0.1);
const VORTEX: Color = Color::srgb(0.4, 0.5, 1.0);

pub(super) fn startup(mut store: ResMut<GizmoConfigStore>) {
    // Drawn over the aircraft, so that the forces inside the wings are visible
    let (config, _) = store.config_mut::<DefaultGizmoConfigGroup>();
    config.depth_bias = -1.0;
    config.line.width = 2.5;
}

pub(super) fn draw(
    flight: Res<Flight>,
    view: Res<View>,
    debug: Res<DebugView>,
    mut gizmos: Gizmos,
) {
    if !debug.forces && !debug.schema {
        return;
    }
    let model = &flight.model;
    let snapshot = &flight.snapshot;
    let attitude = snapshot.body.attitude;
    // Positions in the model frame and directions in the body frame, into the rendered world
    let at = |point: DVec3| (flight.to_ecef(point) - view.position).as_vec3();
    let dir = |vector: DVec3| (attitude * vector).as_vec3();

    // The strips and body cells draw their forces per meter of span or length, so that the
    // arrows do not depend on how finely the aircraft is divided, and their tips trace the
    // distribution of the load. The weight spread evenly over the span gives arrows a tenth of
    // the span long. Point forces, such as of landing gear, draw as if spread over a twentieth
    // of the span.
    let strips = &model.aero.strips;
    let span = (strips
        .iter()
        .map(|s| s.a.y.abs().max(s.b.y.abs()))
        .fold(0.0, f64::max)
        * 2.0)
        .max(0.5);
    let weight = model.config.mass * 9.81;
    let load_scale = 0.1 * span * span / weight;
    let point_scale = load_scale * 20.0 / span;
    let total_scale = 0.3 * span / weight;

    if debug.forces {
        let loads = &snapshot.loads;
        for (strip, load) in strips.iter().zip(&loads.aero.strips) {
            let p = at(strip.center);
            let color = if load.separation > 0.5 { STALLED } else { LIFT };
            // Its width across the chord
            let scale = load_scale * strip.chord / strip.area.max(1e-9);
            arrow(&mut gizmos, p, dir(load.lift * scale), color);
            arrow(&mut gizmos, p, dir(load.drag * scale), DRAG);
        }
        for (cell, force) in model.aero.cells.iter().zip(&loads.aero.cells) {
            arrow(
                &mut gizmos,
                at(cell.center),
                dir(*force * (load_scale / cell.length.max(1e-9))),
                BODY,
            );
        }
        for ((position, _), force) in model
            .aero
            .drag_elements
            .iter()
            .zip(&loads.aero.drag_elements)
        {
            arrow(&mut gizmos, at(*position), dir(*force * point_scale), BODY);
        }
        for (rotor, force) in model.rotors.iter().zip(&loads.rotors) {
            arrow(
                &mut gizmos,
                at(rotor.position),
                dir(*force * total_scale),
                THRUST,
            );
        }

        gizmos.sphere(
            Isometry3d::from_translation(at(model.center_of_gravity)),
            0.02 * span as f32,
            THRUST,
        );
        if let Some(center) = loads.aero.center_of_lift {
            let p = at(center);
            gizmos.sphere(Isometry3d::from_translation(p), 0.02 * span as f32, TOTAL);
            let lift: DVec3 = loads.aero.strips.iter().map(|s| s.lift + s.drag).sum();
            arrow(&mut gizmos, p, dir(lift * total_scale), TOTAL);
        }
    }

    if debug.schema {
        for (strip, load) in strips.iter().zip(&snapshot.loads.aero.strips) {
            let [la, lb] = strip.leading.map(at);
            let [ta, tb] = strip.trailing.map(at);
            match &strip.control {
                None => {
                    gizmos.linestrip([la, lb, tb, ta, la], OUTLINE);
                }
                Some(control) => {
                    // The hinge, and the flap deflected about it
                    let x = 1.0 - control.chord_fraction;
                    let (ha, hb) = (
                        strip.leading[0] + (strip.trailing[0] - strip.leading[0]) * x,
                        strip.leading[1] + (strip.trailing[1] - strip.leading[1]) * x,
                    );
                    let axis = (hb - ha).normalize();
                    let rotation = bevy::math::DQuat::from_axis_angle(axis, load.deflection);
                    let ta = at(ha + rotation * (strip.trailing[0] - ha));
                    let tb = at(hb + rotation * (strip.trailing[1] - hb));
                    let (ha, hb) = (at(ha), at(hb));
                    gizmos.linestrip([ha, la, lb, hb], OUTLINE);
                    gizmos.linestrip([ha, ta, tb, hb, ha], CONTROL);
                }
            }
            // The bound vortex along the quarter chord
            gizmos.line(at(strip.a), at(strip.b), VORTEX);
        }
        for cell in &model.aero.cells {
            let rotation = Quat::from_rotation_arc(Vec3::Z, dir(DVec3::X));
            let r = cell.radius.max(0.005) as f32;
            for offset in [-0.5, 0.5] {
                let p = at(cell.center + DVec3::X * (cell.length * offset));
                gizmos.circle(Isometry3d::new(p, rotation), r, BODY);
            }
        }
        for rotor in &model.rotors {
            let rotation = Quat::from_rotation_arc(Vec3::Z, dir(rotor.axis));
            gizmos.circle(
                Isometry3d::new(at(rotor.position), rotation),
                rotor.diameter as f32 / 2.0,
                THRUST,
            );
        }
        for (position, _) in &model.aero.drag_elements {
            gizmos.cross(Isometry3d::from_translation(at(*position)), 0.03, BODY);
        }
        let cg = at(model.center_of_gravity);
        gizmos.cross(Isometry3d::from_translation(cg), 0.05 * span as f32, THRUST);
    }
}

fn arrow(gizmos: &mut Gizmos, start: Vec3, vector: Vec3, color: Color) {
    if vector.length() > 1e-3 {
        gizmos
            .arrow(start, start + vector, color)
            .with_tip_length(vector.length().min(0.05) * 0.5);
    }
}
