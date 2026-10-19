//! A Google Earth-lite: a globe with streamed imagery and elevation.

mod atmosphere;
mod camera;
mod fetch;
mod geo;
mod hud;
mod material;
mod mesh;
mod sources;
mod terrain;
mod tiles;

use bevy::{
    prelude::*,
    render::view::screenshot::{Screenshot, save_to_disk},
};

use camera::StartView;

fn main() {
    let start = start_view().unwrap_or_default();

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "World map".into(),
            ..default()
        }),
        ..default()
    }))
    .add_plugins((
        material::MaterialsPlugin,
        tiles::TilesPlugin,
        camera::CameraPlugin { start },
        terrain::TerrainPlugin,
        atmosphere::AtmospherePlugin,
        hud::HudPlugin,
    ));

    if let Some(path) = std::env::var_os("WORLDMAP_SCREENSHOT") {
        let after = std::env::var("WORLDMAP_SCREENSHOT_AFTER")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(15.0);
        app.add_systems(Update, screenshot_and_exit(path.into(), after))
            .add_systems(Startup, |mut commands: Commands| {
                commands.remove_resource::<camera::Controls>()
            });
    }

    if let Ok(imagery) = std::env::var("WORLDMAP_IMAGERY") {
        app.world_mut()
            .resource_mut::<terrain::TerrainSettings>()
            .imagery = match imagery.as_str() {
            "osm" => sources::Source::OpenStreetMap,
            _ => sources::Source::Satellite,
        };
    }
    if let Some(spin) = std::env::var("WORLDMAP_SPIN")
        .ok()
        .and_then(|s| s.parse().ok())
    {
        app.insert_resource(camera::Spin(spin));
    }
    if let Some(lon) = std::env::var("WORLDMAP_SUN_LON")
        .ok()
        .and_then(|s| s.parse::<f64>().ok())
    {
        app.world_mut().resource_mut::<atmosphere::Sun>().lon = lon.to_radians();
    }

    app.run();
}

/// The start view can be set with `WORLDMAP_START="lat,lon,height,heading,pitch"`, in degrees
/// and meters above sea level.
fn start_view() -> Option<StartView> {
    let spec = std::env::var("WORLDMAP_START").ok()?;
    let values: Vec<f64> = spec
        .split(',')
        .map(|v| v.trim().parse().ok())
        .collect::<Option<_>>()?;
    let default = StartView::default();
    Some(StartView {
        lat: *values.first()?,
        lon: *values.get(1)?,
        height: values.get(2).copied().unwrap_or(default.height),
        heading: values.get(3).copied().unwrap_or(default.heading),
        pitch: values.get(4).copied().unwrap_or(default.pitch),
    })
}

/// For checking the rendering without a person at the controls.
fn screenshot_and_exit(
    path: std::path::PathBuf,
    after: f32,
) -> impl FnMut(Commands, Res<Time>, MessageWriter<AppExit>, Local<Option<f32>>) {
    move |mut commands, time, mut exit, mut taken| {
        let now = time.elapsed_secs();
        match *taken {
            None if now > after => {
                commands
                    .spawn(Screenshot::primary_window())
                    .observe(save_to_disk(path.clone()));
                *taken = Some(now);
            }
            Some(at) if now > at + 1.0 => {
                exit.write(AppExit::Success);
            }
            _ => {}
        }
    }
}
