//! The look of the image after it is rendered: an exposure that adapts to the light like an eye,
//! and a light grade on top.

use bevy::{
    math::cubic_splines::LinearSpline,
    post_process::{
        auto_exposure::{AutoExposure, AutoExposureCompensationCurve, AutoExposurePlugin},
        bloom::Bloom,
        effect_stack::Vignette,
    },
    prelude::*,
    render::view::ColorGrading,
};

use crate::camera::Controls;

/// Range of the metered luminance, in stops of the light the shaders output: from deep twilight
/// to looking into the sky near the sun.
const METERED: std::ops::RangeInclusive<f32> = -9.0..=4.0;

pub struct PostProcessPlugin;

impl Plugin for PostProcessPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(AutoExposurePlugin)
            .init_resource::<PostProcess>()
            .add_systems(
                Update,
                (
                    toggle.run_if(resource_exists::<Controls>),
                    apply.run_if(resource_changed::<PostProcess>),
                )
                    .chain(),
            );
    }
}

/// Whether the camera adapts its exposure and grades the image, or shows the light as is.
#[derive(Resource)]
pub struct PostProcess {
    pub enabled: bool,
    compensation: Handle<AutoExposureCompensationCurve>,
}

impl FromWorld for PostProcess {
    fn from_world(world: &mut World) -> Self {
        // By the average luminance, in stops. The exposure only follows a change in the light by
        // 40%, so dusk still looks darker than noon, and noon keeps the exposure it had without
        // adapting, at an average luminance of about -1.
        let compensation = |stops: f32| vec2(stops, 0.6 * stops - 0.4);
        let curve = AutoExposureCompensationCurve::from_curve(LinearSpline::new([
            compensation(*METERED.start()),
            compensation(*METERED.end()),
        ]))
        .expect("the compensation curve is monotonic and continuous");
        Self {
            enabled: true,
            compensation: world.resource_mut::<Assets<_>>().add(curve),
        }
    }
}

fn toggle(keys: Res<ButtonInput<KeyCode>>, mut post: ResMut<PostProcess>) {
    if keys.just_pressed(KeyCode::KeyP) {
        post.enabled = !post.enabled;
    }
}

fn apply(post: Res<PostProcess>, camera: Single<Entity, With<Camera3d>>, mut commands: Commands) {
    let mut camera = commands.entity(*camera);
    if !post.enabled {
        camera
            .remove::<(AutoExposure, Vignette)>()
            .insert((Bloom::NATURAL, ColorGrading::default()));
        return;
    }

    let mut grading = ColorGrading::default();
    grading.global.post_saturation = 1.1;
    grading.midtones.contrast = 1.1;
    camera.insert((
        AutoExposure {
            range: METERED,
            compensation_curve: post.compensation.clone(),
            ..default()
        },
        Bloom::NATURAL,
        grading,
        Vignette {
            intensity: 1.0,
            radius: 1.2,
            smoothness: 2.0,
            ..default()
        },
    ));
}
