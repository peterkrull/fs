//! The sun, and the sky around the camera.

use bevy::{camera::visibility::NoFrustumCulling, light::NotShadowCaster, math::DVec2, prelude::*};

use crate::{
    camera::{CameraSystems, Controls, View},
    geo,
    material::SkyMaterial,
};

/// Radius of the sphere the sky is drawn on, which must enclose all of the visible terrain.
const SKY_RADIUS: f32 = 1e9;

pub struct AtmospherePlugin;

impl Plugin for AtmospherePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Sun {
            lon: 40f64.to_radians(),
            lat: 12f64.to_radians(),
        })
        .insert_resource(ClearColor(Color::BLACK))
        .add_systems(Startup, spawn)
        .add_systems(
            Update,
            (move_sun.run_if(resource_exists::<Controls>), update)
                .chain()
                .after(CameraSystems),
        );
    }
}

/// Where the sun is straight overhead, in radians.
#[derive(Resource)]
pub struct Sun {
    pub lon: f64,
    pub lat: f64,
}

impl Sun {
    pub fn direction(&self) -> Vec3 {
        geo::up(DVec2::new(self.lon, self.lat)).as_vec3()
    }
}

#[derive(Component)]
struct Sky(Handle<SkyMaterial>);

fn spawn(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<SkyMaterial>>,
) {
    // The shaders only read the direction of the light
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::default(),
    ));

    let material = materials.add(SkyMaterial { camera: Vec4::ZERO });
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(1.0).mesh().uv(64, 32))),
        MeshMaterial3d(material.clone()),
        Transform::from_scale(Vec3::splat(SKY_RADIUS)),
        NoFrustumCulling,
        NotShadowCaster,
        Sky(material),
    ));
}

fn move_sun(time: Res<Time>, keys: Res<ButtonInput<KeyCode>>, mut sun: ResMut<Sun>) {
    // Half an hour of earth's rotation per second
    let rate = std::f64::consts::TAU / 48.0 * time.delta_secs_f64();
    if keys.pressed(KeyCode::BracketLeft) {
        sun.lon -= rate;
    }
    if keys.pressed(KeyCode::BracketRight) {
        sun.lon += rate;
    }
}

fn update(
    sun: Res<Sun>,
    view: Res<View>,
    sky: Single<&Sky>,
    mut light: Single<&mut Transform, With<DirectionalLight>>,
    mut materials: ResMut<Assets<SkyMaterial>>,
) {
    let direction = sun.direction();
    // The light shines along its forward direction, away from the sun
    **light = Transform::default().looking_to(-direction, Vec3::Z);

    if let Some(mut material) = materials.get_mut(&sky.0) {
        material.camera = view.position.as_vec3().extend(0.0);
    }
}
