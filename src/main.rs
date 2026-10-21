//! A Google Earth-lite: a globe with streamed imagery and elevation.

use bevy_worldmap::{
    atmosphere, camera, flight, hud, material,
    options::{Options, USAGE},
    post,
    sim::runner::Request,
    sources, terrain, tiles,
};

use bevy::{
    prelude::*,
    render::view::screenshot::{Screenshot, save_to_disk},
};

use camera::StartView;

fn main() {
    let options = match Options::parse(std::env::args().skip(1)) {
        Ok(options) if options.rest.is_empty() => options,
        Ok(options) => exit_with_usage(&format!("unknown options {:?}", options.rest)),
        Err(err) => exit_with_usage(&err),
    };
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
        post::PostProcessPlugin,
        hud::HudPlugin,
        flight::FlightPlugin { options },
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
    // For checking the flight views without a person at the controls
    if let Some(after) = std::env::var("WORLDMAP_LAUNCH")
        .ok()
        .and_then(|s| s.parse::<f32>().ok())
    {
        app.add_systems(
            Update,
            move |time: Res<Time>, flight: Option<Res<flight::Flight>>, mut done: Local<bool>| {
                if !*done
                    && time.elapsed_secs() > after
                    && let Some(flight) = flight
                {
                    flight.request(Request::Launch);
                    *done = true;
                }
            },
        );
    }
    if let Some([yaw, pitch, distance]) = std::env::var("WORLDMAP_CHASE").ok().and_then(|s| {
        let v: Vec<f64> = s.split(',').filter_map(|v| v.trim().parse().ok()).collect();
        <[f64; 3]>::try_from(v).ok()
    }) {
        app.insert_resource(flight::Chase {
            distance,
            yaw: yaw.to_radians(),
            pitch: pitch.to_radians(),
            heading: None,
        });
    }
    if let Ok(views) = std::env::var("WORLDMAP_DEBUG") {
        app.insert_resource(flight::DebugView {
            forces: views.contains("forces"),
            schema: views.contains("schema"),
            model: !views.contains("nomodel"),
            weather: views.contains("weather"),
        });
    }
    if let Some(lon) = std::env::var("WORLDMAP_SUN_LON")
        .ok()
        .and_then(|s| s.parse::<f64>().ok())
    {
        app.world_mut().resource_mut::<atmosphere::Sun>().lon = lon.to_radians();
    }
    if let Some(haze) = std::env::var("WORLDMAP_HAZE")
        .ok()
        .and_then(|s| s.parse::<f32>().ok())
    {
        app.insert_resource(atmosphere::Haze(haze));
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

fn exit_with_usage(error: &str) -> ! {
    eprintln!("{error}\n\n{USAGE}");
    std::process::exit(1);
}
