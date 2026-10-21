//! On-screen information, controls, and the attribution required by the tile sources.

use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    prelude::*,
};

use crate::{
    atmosphere::Haze,
    camera::{Controls, View},
    sources::Source,
    terrain::{Terrain, TerrainSettings},
    tiles::Tiles,
};

const HELP: &str = "\
Click: capture mouse   Esc: release   Right drag: look   Scroll: zoom or speed
Free camera: W A S D fly   Space / C: up / down   Shift: faster
T: toggle imagery   [ ]: move the sun   , .: haze   - =: level of detail
F: freeze tiles   H: hide help";

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FrameTimeDiagnosticsPlugin::default())
            .add_systems(Startup, spawn)
            .add_systems(
                Update,
                (controls.run_if(resource_exists::<Controls>), update),
            );
    }
}

#[derive(Component)]
struct Stats;

#[derive(Component)]
struct Attribution;

#[derive(Component)]
struct Help;

fn spawn(mut commands: Commands) {
    let font = TextFont::from_font_size(14.0);
    let shadow = TextShadow {
        offset: Vec2::splat(1.0),
        color: Color::BLACK.with_alpha(0.6),
    };
    let at = |top: Option<f32>, bottom: Option<f32>, left: Option<f32>, right: Option<f32>| Node {
        position_type: PositionType::Absolute,
        top: top.map_or(Val::Auto, Val::Px),
        bottom: bottom.map_or(Val::Auto, Val::Px),
        left: left.map_or(Val::Auto, Val::Px),
        right: right.map_or(Val::Auto, Val::Px),
        ..default()
    };

    commands.spawn((
        Text::default(),
        font.clone(),
        shadow,
        at(Some(10.0), None, Some(12.0), None),
        Stats,
    ));
    commands.spawn((
        Text::new(HELP),
        font.clone(),
        shadow,
        at(None, Some(10.0), Some(12.0), None),
        Help,
    ));
    commands.spawn((
        Text::default(),
        font.with_font_size(12.0),
        shadow,
        TextLayout::justify(Justify::Right),
        at(None, Some(10.0), None, Some(12.0)),
        Attribution,
    ));
}

fn controls(
    keys: Res<ButtonInput<KeyCode>>,
    mut settings: ResMut<TerrainSettings>,
    mut help: Single<&mut Visibility, With<Help>>,
) {
    if keys.just_pressed(KeyCode::KeyT) {
        let index = Source::IMAGERY
            .iter()
            .position(|s| *s == settings.imagery)
            .unwrap_or(0);
        settings.imagery = Source::IMAGERY[(index + 1) % Source::IMAGERY.len()];
    }
    if keys.just_pressed(KeyCode::Minus) {
        settings.lod_threshold = (settings.lod_threshold * 1.25).min(8.0);
    }
    if keys.just_pressed(KeyCode::Equal) {
        settings.lod_threshold = (settings.lod_threshold / 1.25).max(0.25);
    }
    if keys.just_pressed(KeyCode::KeyF) {
        settings.freeze = !settings.freeze;
    }
    if keys.just_pressed(KeyCode::KeyH) {
        help.toggle_visible_hidden();
    }
}

fn update(
    time: Res<Time>,
    mut last: Local<f32>,
    diagnostics: Res<DiagnosticsStore>,
    view: Res<View>,
    terrain: Res<Terrain>,
    tiles: Res<Tiles>,
    settings: Res<TerrainSettings>,
    haze: Res<Haze>,
    mut stats: Single<&mut Text, (With<Stats>, Without<Attribution>)>,
    mut attribution: Single<&mut Text, (With<Attribution>, Without<Stats>)>,
) {
    // Avoid relayouting the text every frame
    let now = time.elapsed_secs();
    if now - *last < 0.2 {
        return;
    }
    *last = now;

    let fps = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|fps| fps.smoothed())
        .unwrap_or(0.0);
    let (lon_lat, height) = view.geodetic();
    let (heading, pitch) = view.heading_pitch();
    let s = terrain.stats;
    stats.0 = format!(
        "{fps:.0} fps\n\
         {}   height {} (above ground {})\n\
         heading {heading:.0} deg   pitch {pitch:.0} deg   speed {}/s\n\
         {}   detail {:.2} px/texel   haze {:.2}{}\n\
         tiles drawn {}   max zoom {}   in memory {}   building {}   coarsened {}\n\
         imagery {}   elevation {}   downloading {}   queued {}",
        coordinates(lon_lat.x.to_degrees(), lon_lat.y.to_degrees()),
        distance(height),
        distance(view.height_above_ground()),
        distance(view.speed()),
        settings.imagery.name(),
        settings.lod_threshold,
        haze.0,
        if settings.freeze { "   FROZEN" } else { "" },
        s.rendered,
        s.max_zoom,
        s.nodes,
        s.building,
        s.coarsened,
        tiles.imagery_count(),
        tiles.elevation_count(),
        tiles.downloading(),
        tiles.pending(),
    );

    attribution.0 = format!(
        "{}\n{}",
        settings.imagery.attribution(),
        Source::Terrain.attribution()
    );
}

fn coordinates(lon: f64, lat: f64) -> String {
    let ns = if lat >= 0.0 { 'N' } else { 'S' };
    let ew = if lon >= 0.0 { 'E' } else { 'W' };
    format!("{:.5} {ns}  {:.5} {ew}", lat.abs(), lon.abs())
}

fn distance(meters: f64) -> String {
    match meters.abs() {
        m if m < 10_000.0 => format!("{meters:.0} m"),
        _ => format!("{:.1} km", meters / 1000.0),
    }
}
