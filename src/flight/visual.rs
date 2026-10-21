//! The 3D model of the aircraft, generated from its definition, or loaded from glTF.
//!
//! The model is built in the model frame of the definition, and placed by the transform of its
//! root, which maps the model frame into the rendered world. Control surfaces are separate
//! meshes, which rotate about their hinge lines as the actuators move.

use bevy::{
    asset::RenderAssetUsages, math::DVec3, mesh::PrimitiveTopology, prelude::*,
    world_serialization::WorldAssetRoot,
};

use super::{DebugView, Flight};
use crate::{
    atmosphere::LightProbe,
    camera::View,
    sim::{
        aero::Strip,
        config::{AirfoilShape, Visual},
    },
};

/// Spinning faster than this many revolutions per second, a rotor is drawn as a blurred disc.
const BLUR_SPEED: f64 = 15.0;
const CHORD_POINTS: usize = 14;
const RING_POINTS: usize = 20;

#[derive(Component)]
pub(super) struct AircraftRoot;

/// The part of the model that hides when the debug view asks for it.
#[derive(Component)]
pub(super) struct Model;

#[derive(Component)]
pub(super) struct ControlPiece {
    strip: usize,
    axis: Vec3,
}

#[derive(Component)]
pub(super) struct Blades {
    rotor: usize,
    base: Quat,
}

#[derive(Component)]
pub(super) struct Disc(usize);

/// Spawn the model of the aircraft, replacing the previous one after a reload.
pub(super) fn spawn(
    mut commands: Commands,
    flight: Res<Flight>,
    previous: Query<Entity, With<AircraftRoot>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    assets: Res<AssetServer>,
) {
    for entity in &previous {
        commands.entity(entity).despawn();
    }
    let model = &flight.model;
    let material = |materials: &mut Assets<StandardMaterial>, color: Color| {
        materials.add(StandardMaterial {
            base_color: color,
            perceptual_roughness: 0.55,
            double_sided: true,
            cull_mode: None,
            ..default()
        })
    };

    let root = commands
        .spawn((
            AircraftRoot,
            LightProbe,
            Transform::default(),
            Visibility::default(),
        ))
        .id();
    let parts = commands
        .spawn((
            Model,
            Transform::default(),
            Visibility::default(),
            ChildOf(root),
        ))
        .id();

    match &model.config.visual {
        Visual::Generated => {
            let shapes: Vec<AirfoilShape> =
                model.config.airfoils.values().map(|a| a.shape()).collect();
            let shape = |strip: &Strip| shapes[strip.airfoil];

            let mut wing = MeshBuilder::default();
            for strip in &model.aero.strips {
                let hinge = strip
                    .control
                    .as_ref()
                    .map_or(1.0, |c| 1.0 - c.chord_fraction);
                wing.airfoil(strip, shape(strip), 0.0, hinge, DVec3::ZERO);
            }
            commands.spawn((
                Mesh3d(meshes.add(wing.build())),
                MeshMaterial3d(material(&mut materials, Color::srgb(0.82, 0.84, 0.88))),
                ChildOf(parts),
            ));

            let control_material = material(&mut materials, Color::srgb(0.95, 0.45, 0.12));
            for (index, strip) in model.aero.strips.iter().enumerate() {
                let Some(control) = &strip.control else {
                    continue;
                };
                let x = 1.0 - control.chord_fraction;
                let (a, b) = (
                    hinge_point(strip, shape(strip), 0, x),
                    hinge_point(strip, shape(strip), 1, x),
                );
                let center = (a + b) / 2.0;
                let mut piece = MeshBuilder::default();
                piece.airfoil(strip, shape(strip), x, 1.0, center);
                commands.spawn((
                    Mesh3d(meshes.add(piece.build())),
                    MeshMaterial3d(control_material.clone()),
                    Transform::from_translation(center.as_vec3()),
                    ControlPiece {
                        strip: index,
                        axis: (b - a).normalize().as_vec3(),
                    },
                    ChildOf(parts),
                ));
            }

            let mut bodies = MeshBuilder::default();
            for body in &model.config.bodies {
                bodies.body(DVec3::from(body.nose), body.length, &body.profile);
            }
            commands.spawn((
                Mesh3d(meshes.add(bodies.build())),
                MeshMaterial3d(material(&mut materials, Color::srgb(0.95, 0.95, 0.95))),
                ChildOf(parts),
            ));
        }
        Visual::Gltf {
            path,
            offset,
            scale,
        } => {
            // glTF models face +z with +y up, the model frame faces +x with +z down
            let axes = Quat::from_mat3(&Mat3::from_cols(
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(0.0, 0.0, -1.0),
                Vec3::X,
            ));
            commands.spawn((
                WorldAssetRoot(assets.load(GltfAssetLabel::Scene(0).from_asset(path.clone()))),
                Transform::from_translation(DVec3::from(*offset).as_vec3())
                    .with_rotation(axes)
                    .with_scale(Vec3::splat(*scale as f32)),
                ChildOf(parts),
            ));
        }
    }

    // Rotors are drawn either way, as they spin
    let blade_color = Color::srgb(0.08, 0.08, 0.09);
    let blade_material = material(&mut materials, blade_color);
    // The blades smeared over the disc, lit as they are
    let disc_material = materials.add(StandardMaterial {
        base_color: blade_color.with_alpha(0.25),
        alpha_mode: AlphaMode::Blend,
        perceptual_roughness: 0.55,
        double_sided: true,
        cull_mode: None,
        ..default()
    });
    for (index, rotor) in model.rotors.iter().enumerate() {
        let axis = rotor.axis.as_vec3();
        let base = Quat::from_rotation_arc(Vec3::Z, axis);
        let position = rotor.position.as_vec3();
        let d = rotor.diameter as f32;
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(d, d * 0.07, d * 0.02))),
            MeshMaterial3d(blade_material.clone()),
            Transform::from_translation(position).with_rotation(base),
            Blades { rotor: index, base },
            ChildOf(root),
        ));
        commands.spawn((
            Mesh3d(meshes.add(Circle::new(d / 2.0))),
            MeshMaterial3d(disc_material.clone()),
            Transform::from_translation(position).with_rotation(base),
            Visibility::Hidden,
            Disc(index),
            ChildOf(root),
        ));
    }
}

#[expect(clippy::type_complexity, reason = "disjoint queries of transforms")]
pub(super) fn update(
    flight: Res<Flight>,
    view: Res<View>,
    debug: Res<DebugView>,
    mut root: Single<&mut Transform, With<AircraftRoot>>,
    mut model: Single<&mut Visibility, (With<Model>, Without<Disc>)>,
    mut pieces: Query<(&ControlPiece, &mut Transform), (Without<AircraftRoot>, Without<Blades>)>,
    mut blades: Query<
        (&Blades, &mut Transform, &mut Visibility),
        (
            Without<AircraftRoot>,
            Without<ControlPiece>,
            Without<Model>,
            Without<Disc>,
        ),
    >,
    mut discs: Query<(&Disc, &mut Visibility), (Without<Blades>, Without<Model>)>,
) {
    let body = &flight.snapshot.body;
    // The origin of the model frame, relative to the camera
    let origin = flight.to_ecef(DVec3::ZERO) - view.position;
    root.translation = origin.as_vec3();
    root.rotation = body.attitude.as_quat();
    model.set_if_neq(match debug.model {
        true => Visibility::Inherited,
        false => Visibility::Hidden,
    });

    let strips = &flight.snapshot.loads.aero.strips;
    for (piece, mut transform) in &mut pieces {
        let deflection = strips.get(piece.strip).map_or(0.0, |s| s.deflection);
        transform.rotation = Quat::from_axis_angle(piece.axis, deflection as f32);
    }

    for (blade, mut transform, mut visibility) in &mut blades {
        // Right after a reload, the model may already be that of the new aircraft
        let (Some(state), Some(rotor)) = (
            flight.snapshot.rotors.get(blade.rotor),
            flight.model.rotors.get(blade.rotor),
        ) else {
            continue;
        };
        let axis = rotor.axis.as_vec3();
        transform.rotation = Quat::from_axis_angle(axis, state.angle as f32) * blade.base;
        let blurred = state.speed / std::f64::consts::TAU > BLUR_SPEED;
        visibility.set_if_neq(if blurred {
            Visibility::Hidden
        } else {
            Visibility::Inherited
        });
    }
    for (disc, mut visibility) in &mut discs {
        let Some(state) = flight.snapshot.rotors.get(disc.0) else {
            continue;
        };
        let blurred = state.speed / std::f64::consts::TAU > BLUR_SPEED;
        visibility.set_if_neq(if blurred {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        });
    }
}

/// The point on the camber line at `x` along the chord, at end `end` of a strip.
fn hinge_point(strip: &Strip, shape: AirfoilShape, end: usize, x: f64) -> DVec3 {
    let leading = strip.leading[end];
    let chord = strip.trailing[end] - leading;
    leading + chord * x + strip.normal * (camber(shape, x) * chord.length())
}

fn camber(shape: AirfoilShape, x: f64) -> f64 {
    let (m, p) = (shape.camber, shape.camber_position.clamp(0.05, 0.95));
    match x < p {
        true => m / (p * p) * (2.0 * p * x - x * x),
        false => m / ((1.0 - p) * (1.0 - p)) * (1.0 - 2.0 * p + 2.0 * p * x - x * x),
    }
}

/// Half the thickness of a NACA 4-digit section, closed at the trailing edge.
fn thickness(shape: AirfoilShape, x: f64) -> f64 {
    let x = x.clamp(0.0, 1.0);
    5.0 * shape.thickness
        * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x + 0.2843 * x.powi(3)
            - 0.1036 * x.powi(4))
}

/// Builds triangle meshes with flat shading.
#[derive(Default)]
struct MeshBuilder {
    positions: Vec<[f32; 3]>,
}

impl MeshBuilder {
    fn triangle(&mut self, a: DVec3, b: DVec3, c: DVec3, origin: DVec3) {
        for p in [a, b, c] {
            self.positions.push((p - origin).as_vec3().to_array());
        }
    }

    fn quad(&mut self, a: DVec3, b: DVec3, c: DVec3, d: DVec3, origin: DVec3) {
        self.triangle(a, b, c, origin);
        self.triangle(a, c, d, origin);
    }

    /// The part of a strip between `x0` and `x1` along its chord, relative to `origin`.
    fn airfoil(&mut self, strip: &Strip, shape: AirfoilShape, x0: f64, x1: f64, origin: DVec3) {
        let section = |end: usize, x: f64, side: f64| {
            let leading = strip.leading[end];
            let chord = strip.trailing[end] - leading;
            let offset = camber(shape, x) + side * thickness(shape, x);
            leading + chord * x + strip.normal * (offset * chord.length())
        };
        // Cosine spacing puts more points around the rounded leading edge
        let xs: Vec<f64> = (0..CHORD_POINTS)
            .map(|i| {
                let t = i as f64 / (CHORD_POINTS - 1) as f64;
                x0 + (x1 - x0) * (1.0 - (t * std::f64::consts::PI).cos()) / 2.0
            })
            .collect();
        for side in [1.0, -1.0] {
            for w in xs.windows(2) {
                self.quad(
                    section(0, w[0], side),
                    section(1, w[0], side),
                    section(1, w[1], side),
                    section(0, w[1], side),
                    origin,
                );
            }
        }
        // Close the faces at the hinge
        for x in [x0, x1] {
            if x > 0.0 && x < 1.0 {
                self.quad(
                    section(0, x, 1.0),
                    section(1, x, 1.0),
                    section(1, x, -1.0),
                    section(0, x, -1.0),
                    origin,
                );
            }
        }
        // Close the ends of the strip. Where strips meet, their ends lie inside the surface
        // and are never seen, but this also closes the tips, and the side of a strip which a
        // deflected control surface next to it uncovers.
        for end in [0, 1] {
            for w in xs.windows(2) {
                self.quad(
                    section(end, w[0], 1.0),
                    section(end, w[1], 1.0),
                    section(end, w[1], -1.0),
                    section(end, w[0], -1.0),
                    origin,
                );
            }
        }
    }

    /// A body of revolution backwards along the `x` axis from `nose`.
    fn body(&mut self, nose: DVec3, length: f64, profile: &[(f64, f64)]) {
        let ring = |t: f64, radius: f64, i: usize| {
            let angle = i as f64 / RING_POINTS as f64 * std::f64::consts::TAU;
            nose - DVec3::X * (length * t) + DVec3::new(0.0, angle.cos(), angle.sin()) * radius
        };
        for w in profile.windows(2) {
            let ((t0, r0), (t1, r1)) = (w[0], w[1]);
            for i in 0..RING_POINTS {
                self.quad(
                    ring(t0, r0, i),
                    ring(t0, r0, i + 1),
                    ring(t1, r1, i + 1),
                    ring(t1, r1, i),
                    DVec3::ZERO,
                );
            }
        }
        // Close the ends which are not points
        for &(t, radius) in [profile.first(), profile.last()].into_iter().flatten() {
            if radius > 0.0 {
                let center = nose - DVec3::X * (length * t);
                for i in 0..RING_POINTS {
                    self.triangle(center, ring(t, radius, i), ring(t, radius, i + 1), DVec3::ZERO);
                }
            }
        }
    }

    fn build(self) -> Mesh {
        let mut mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::RENDER_WORLD,
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, self.positions);
        mesh.compute_flat_normals();
        mesh
    }
}
