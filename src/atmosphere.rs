//! The sun, the haze, and the sky around the camera.

use bevy::{
    camera::{Exposure, visibility::NoFrustumCulling},
    light::NotShadowCaster,
    math::DVec2,
    prelude::*,
};

use crate::{
    camera::{CameraSystems, Controls, View},
    geo,
    material::SkyMaterial,
    scattering::{ScatteringLuts, ScatteringParams, ScatteringPlugin, SkyLight},
};

/// Radius of the sphere the sky is drawn on, which must enclose all of the visible terrain.
const SKY_RADIUS: f32 = 1e9;
/// Average reflectance of the ground below objects, as in `scattering.wgsl`.
const GROUND_ALBEDO: f32 = 0.3;

pub struct AtmospherePlugin;

impl Plugin for AtmospherePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(ScatteringPlugin)
            .insert_resource(Sun {
                lon: 40f64.to_radians(),
                lat: 12f64.to_radians(),
            })
            .init_resource::<Haze>()
            .insert_resource(ClearColor(Color::BLACK))
            .add_systems(Startup, spawn)
            .add_systems(
                Update,
                (
                    (move_sun, change_haze).run_if(resource_exists::<Controls>),
                    update,
                    light_objects,
                )
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

/// The aerosols in the air, as their vertical optical depth at 550 nm from sea level: about
/// 0.05 on a clear day, 0.1 to 0.2 on a typical one, and 0.5 and more in thick haze.
#[derive(Resource)]
pub struct Haze(pub f32);

impl Haze {
    pub const MIN: f32 = 0.01;
    pub const MAX: f32 = 2.0;
}

impl Default for Haze {
    fn default() -> Self {
        Self(0.1)
    }
}

/// Objects drawn with Bevy's materials, such as the aircraft, are lit by the sun and the sky as
/// they are where the entity with this component is, or at the camera without one.
#[derive(Component)]
pub struct LightProbe;

#[derive(Component)]
struct Sky(Handle<SkyMaterial>);

fn spawn(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<SkyMaterial>>,
    luts: Res<ScatteringLuts>,
) {
    // The shaders only read the direction of the light
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::default(),
    ));

    let material = materials.add(SkyMaterial::new(&luts));
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

/// Doubles or halves the haze every second.
fn change_haze(time: Res<Time>, keys: Res<ButtonInput<KeyCode>>, mut haze: ResMut<Haze>) {
    let rate = time.delta_secs() * std::f32::consts::LN_2;
    let factor = match (keys.pressed(KeyCode::Period), keys.pressed(KeyCode::Comma)) {
        (true, false) => rate.exp(),
        (false, true) => (-rate).exp(),
        _ => return,
    };
    haze.0 = (haze.0 * factor).clamp(Haze::MIN, Haze::MAX);
}

fn update(
    sun: Res<Sun>,
    haze: Res<Haze>,
    view: Res<View>,
    mut params: ResMut<ScatteringParams>,
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
    *params = ScatteringParams {
        radius: view.position.length() as f32,
        sun_mu: view.up().as_vec3().dot(direction),
        aerosols: haze.0,
    };
}

/// Lights Bevy's materials like the terrain: the directional light is the sunlight as it reaches
/// the probe, and the ambient light averages the light of the sky from above and that reflected
/// by the ground from below.
fn light_objects(
    sky_light: Res<SkyLight>,
    sun: Res<Sun>,
    view: Res<View>,
    probe: Option<Single<&Transform, With<LightProbe>>>,
    mut light: Single<&mut DirectionalLight>,
    mut ambient: ResMut<GlobalAmbientLight>,
) {
    // Positions are relative to the camera
    let offset = probe.map_or(Vec3::ZERO, |transform| transform.translation);
    let position = (view.position + offset.as_dvec3()).as_vec3();
    let direction = sun.direction();
    let Some((sunlight, sky)) = sky_light.at(position, direction) else {
        return;
    };

    // Bevy scales lights by the exposure of the camera, where the shaders of the atmosphere
    // output their light directly
    let exposure = Exposure::default().exposure();
    light.color = Color::linear_rgb(sunlight.x, sunlight.y, sunlight.z);
    light.illuminance = 1.0 / exposure;

    let up = position.normalize();
    let ground = GROUND_ALBEDO * (sunlight * up.dot(direction).max(0.0) + sky);
    // Bevy's ambient light is radiance, where the tables hold irradiance
    let radiance = (sky + ground) * 0.5 / std::f32::consts::PI;
    ambient.color = Color::linear_rgb(radiance.x, radiance.y, radiance.z);
    ambient.brightness = 1.0 / exposure;
}
