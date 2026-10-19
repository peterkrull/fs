//! A first person camera to fly around the globe with.
//!
//! The camera is the origin of the rendered world: it stays at `(0, 0, 0)`, while everything
//! else is positioned relative to it. Its actual position is kept in `f64` ECEF coordinates in
//! [`View`], as `f32` can not represent positions on the earth to better than about a meter.

use bevy::{
    camera::Hdr,
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll},
    math::{DQuat, DVec2, DVec3},
    post_process::bloom::Bloom,
    prelude::*,
    window::{CursorGrabMode, CursorOptions, PrimaryWindow},
};

use crate::{geo, tiles::Tiles};

/// Radians of rotation per pixel of mouse movement.
const MOUSE_SENSITIVITY: f64 = 0.002;
/// Closest the camera may get to the terrain.
const MIN_CLEARANCE: f64 = 2.0;
const MAX_HEIGHT: f64 = 40_000_000.0;

pub struct CameraPlugin {
    pub start: StartView,
}

/// Where the camera starts, with angles in degrees and the height in meters above sea level.
#[derive(Clone, Copy, Debug)]
pub struct StartView {
    pub lat: f64,
    pub lon: f64,
    pub height: f64,
    pub heading: f64,
    pub pitch: f64,
}

impl Default for StartView {
    /// Overlooking the Alps from the north.
    fn default() -> Self {
        Self {
            lat: 46.35,
            lon: 7.6,
            height: 9_000.0,
            heading: 165.0,
            pitch: -18.0,
        }
    }
}

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(View::new(self.start))
            .insert_resource(Controls)
            .init_resource::<CameraMode>()
            .add_systems(Startup, spawn_camera)
            .add_systems(
                Update,
                (
                    grab_cursor,
                    fly.run_if(
                        resource_exists::<Controls>.and_then(resource_equals(CameraMode::Free)),
                    ),
                    spin.run_if(resource_exists::<Spin>),
                )
                    .chain()
                    .in_set(CameraSystems),
            )
            .add_systems(PostUpdate, sync_camera.before(TransformSystems::Propagate));
    }
}

#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct CameraSystems;

/// Turn the camera at this many degrees per second, to test the streaming without a person at
/// the controls.
#[derive(Resource)]
pub struct Spin(pub f64);

/// Whether the camera flies freely, or follows the aircraft.
#[derive(Resource, Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum CameraMode {
    #[default]
    Free,
    Chase,
}

/// The camera only responds to input while this resource exists.
#[derive(Resource)]
pub struct Controls;

/// The camera's actual position and orientation.
#[derive(Resource)]
pub struct View {
    pub position: DVec3,
    pub forward: DVec3,
    /// Vertical field of view in radians.
    pub fov: f64,
    pub aspect: f64,
    pub viewport_height: f64,
    /// Height of the terrain below the camera.
    pub ground: f64,
    /// Speed multiplier, as a power of two.
    pub speed_exp: i32,
    /// Distance to the closest object that must not be clipped by the near plane, besides
    /// the terrain.
    pub nearest: f64,
}

impl View {
    fn new(start: StartView) -> Self {
        let lon_lat = DVec2::new(start.lon.to_radians(), start.lat.to_radians());
        let position = geo::to_ecef(lon_lat, start.height);
        let (east, north, up) = geo::enu(position);
        let (heading, pitch) = (start.heading.to_radians(), start.pitch.to_radians());
        let horizontal = north * heading.cos() + east * heading.sin();
        Self {
            position,
            forward: (horizontal * pitch.cos() + up * pitch.sin()).normalize(),
            fov: 60f64.to_radians(),
            aspect: 16.0 / 9.0,
            viewport_height: 1080.0,
            ground: 0.0,
            speed_exp: 0,
            nearest: f64::INFINITY,
        }
    }

    pub fn up(&self) -> DVec3 {
        self.position.normalize()
    }

    pub fn right(&self) -> DVec3 {
        self.forward.cross(self.up()).normalize()
    }

    /// Longitude and latitude in radians, and height above sea level.
    pub fn geodetic(&self) -> (DVec2, f64) {
        geo::from_ecef(self.position)
    }

    pub fn height_above_ground(&self) -> f64 {
        self.position.length() - geo::EARTH_RADIUS - self.ground
    }

    /// The speed scales with the height, so that the ground appears to move equally fast.
    pub fn speed(&self) -> f64 {
        self.height_above_ground().max(5.0) * 0.8 * 2f64.powf(self.speed_exp as f64 / 2.0)
    }

    /// Heading from north and pitch above the horizon, in degrees.
    pub fn heading_pitch(&self) -> (f64, f64) {
        let (east, north, up) = geo::enu(self.position);
        let heading = self.forward.dot(east).atan2(self.forward.dot(north));
        let pitch = self.forward.dot(up).clamp(-1.0, 1.0).asin();
        (heading.to_degrees().rem_euclid(360.0), pitch.to_degrees())
    }
}

fn spawn_camera(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Hdr,
        Bloom::NATURAL,
        Projection::Perspective(PerspectiveProjection {
            fov: 60f32.to_radians(),
            near: 0.5,
            // Only used for culling, the projection itself has no far plane
            far: 1e12,
            ..default()
        }),
        Transform::default(),
    ));
}

fn grab_cursor(
    mouse: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    mut cursor: Single<&mut CursorOptions, With<PrimaryWindow>>,
) {
    if mouse.just_pressed(MouseButton::Left) {
        cursor.grab_mode = CursorGrabMode::Locked;
        cursor.visible = false;
    }
    if keys.just_pressed(KeyCode::Escape) {
        cursor.grab_mode = CursorGrabMode::None;
        cursor.visible = true;
    }
}

fn fly(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    cursor: Single<&CursorOptions, With<PrimaryWindow>>,
    tiles: Res<Tiles>,
    mut view: ResMut<View>,
) {
    let dt = time.delta_secs_f64();
    let up = view.up();

    // Look around while the cursor is grabbed, or while the right mouse button is held
    if cursor.grab_mode != CursorGrabMode::None || mouse.pressed(MouseButton::Right) {
        let delta = motion.delta.as_dvec2() * MOUSE_SENSITIVITY;
        view.forward = DQuat::from_axis_angle(up, -delta.x) * view.forward;
        let pitched = DQuat::from_axis_angle(view.right(), -delta.y) * view.forward;
        if pitched.dot(up).abs() < 0.995 {
            view.forward = pitched;
        }
    }

    if scroll.delta.y != 0.0 {
        view.speed_exp = (view.speed_exp + scroll.delta.y.signum() as i32).clamp(-12, 12);
    }

    let axis = |pos: KeyCode, neg: KeyCode| {
        keys.pressed(pos) as i32 as f64 - keys.pressed(neg) as i32 as f64
    };
    let direction = view.forward * axis(KeyCode::KeyW, KeyCode::KeyS)
        + view.right() * axis(KeyCode::KeyD, KeyCode::KeyA)
        + up * (axis(KeyCode::Space, KeyCode::KeyC) + axis(KeyCode::KeyE, KeyCode::KeyQ));
    let boost = match keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]) {
        true => 5.0,
        false => 1.0,
    };
    let mut position = view.position + direction.normalize_or_zero() * view.speed() * boost * dt;

    // Stay above the terrain, and within sight of the earth
    let (lon_lat, _) = geo::from_ecef(position);
    view.ground = tiles.height_at(lon_lat, 15).unwrap_or(0.0) as f64;
    let min = geo::EARTH_RADIUS + view.ground + MIN_CLEARANCE;
    position = position.normalize() * position.length().clamp(min, geo::EARTH_RADIUS + MAX_HEIGHT);

    // Keep the view level with the horizon as the camera moves around the globe
    view.forward = (DQuat::from_rotation_arc(up, position.normalize()) * view.forward).normalize();
    view.position = position;
}

fn spin(time: Res<Time>, spin: Res<Spin>, mut view: ResMut<View>) {
    let angle = spin.0.to_radians() * time.delta_secs_f64();
    view.forward = DQuat::from_axis_angle(view.up(), angle) * view.forward;
}

fn sync_camera(mut view: ResMut<View>, camera: Single<(&mut Transform, &mut Projection, &Camera)>) {
    let (mut transform, mut projection, camera) = camera.into_inner();
    *transform = Transform::default().looking_to(view.forward.as_vec3(), view.up().as_vec3());

    if let Projection::Perspective(perspective) = projection.as_mut() {
        // The depth buffer is reversed and has no far plane, so precision is plentiful
        // even with a near plane this close
        let near = (view.height_above_ground() * 0.05).min(view.nearest * 0.5);
        perspective.near = near.clamp(0.02, 1000.0) as f32;
        view.fov = perspective.fov as f64;
        view.aspect = perspective.aspect_ratio as f64;
    }
    if let Some(size) = camera.physical_viewport_size() {
        view.viewport_height = size.y as f64;
    }
}
