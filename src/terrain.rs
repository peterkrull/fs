//! Selection and drawing of the terrain tiles.
//!
//! Every frame a quadtree of tiles is traversed from the root, culling the tiles which are
//! outside the view or below the horizon. A tile is split into its four children while its
//! imagery would appear coarser than [`TerrainSettings::lod_threshold`] pixels per texel,
//! as seen from the camera. This takes the distance into account, as well as how foreshortened
//! the tile is: at grazing angles the imagery is compressed in one direction, so a coarser
//! level of detail suffices, see [`TerrainSettings::anisotropy`].
//!
//! A tile is only split once the meshes of all four of its children are ready to draw, so that
//! there is never a hole in the terrain. Children outside the view are kept as well, so that
//! turning the camera never makes a tile fall back to its coarser parent. Meanwhile, tiles are
//! drawn with the best imagery and elevation available, which may be that of an ancestor, and
//! are updated as better data streams in.
//!
//! Parents and children are never drawn at the same time, so coarse geometry can not poke
//! through finer geometry. Instead, a tile fades in from its parent after a split: its geometry
//! morphs from that of the parent, and its imagery blends over that of the parent. Before a
//! merge, the children fade out into their parent the same way. Tiles also blend into better
//! imagery as it arrives, and morph into better elevation. Imagery of different zoom levels
//! often differs in tone, which would otherwise make every change of the level of detail stand
//! out.
//!
//! Only changes to terrain which was in view in the previous frame fade. Terrain that the
//! camera just turned or moved to appears at once with the most detail available, as it was
//! not seen before.

use std::{
    collections::{HashMap, HashSet},
    f64::consts::TAU,
};

use bevy::{
    camera::visibility::NoFrustumCulling,
    light::NotShadowCaster,
    math::{DVec2, DVec3},
    prelude::*,
    tasks::{AsyncComputeTaskPool, Task, futures::check_ready},
};

use crate::{
    camera::{CameraSystems, View},
    fetch::Key,
    geo::{self, EARTH_RADIUS, MAX_HEIGHT, TileId},
    material::{TerrainMaterial, TerrainParams},
    mesh::{self, Elevation, TileMesh},
    scattering::ScatteringLuts,
    sources::Source,
    tiles::Tiles,
};

/// The most meshes being built at once.
const MAX_BUILDING: usize = 48;
/// Tiles not wanted for this many seconds are dropped, along with their meshes.
const EVICT_SECONDS: f64 = 5.0;
/// A split tile is only merged again once its detail drops this much below the threshold,
/// so that tiles at the threshold do not alternate between levels.
const SPLIT_HYSTERESIS: f64 = 0.8;
/// New meshes are only drawn after this many frames, once they are uploaded to the GPU.
/// Drawing them earlier would leave a hole in the terrain for a frame.
const SETTLE_FRAMES: u64 = 2;
/// Color of tiles without any imagery yet.
const PLACEHOLDER: Color = Color::srgb(0.18, 0.22, 0.28);
/// Seconds over which a tile fades in from its parent, or out into it, and from one imagery to
/// the next.
const FADE_SECONDS: f32 = 0.5;
/// A split tile whose detail drops below this fraction of the threshold merges without fading
/// out its children, which are then too small for it to show. This lets the level of detail
/// keep up with a camera moving away quickly.
const MERGE_AT_ONCE: f64 = 0.25;
/// A tile coarser than it should be fades in faster, by how many times coarser it is, up to
/// this much faster. The level of detail then catches up quickly with a camera moving closer,
/// or after the start, while changes around the threshold stay gradual.
const MAX_FADE_SPEEDUP: f64 = 8.0;

pub struct TerrainPlugin;

impl Plugin for TerrainPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(TerrainSettings::default())
            .init_resource::<Terrain>()
            .add_systems(Startup, spawn_caps)
            .add_systems(
                Update,
                (select_tiles, poll_meshes, update_tiles, update_caps)
                    .chain()
                    .after(CameraSystems),
            );
    }
}

#[derive(Resource)]
pub struct TerrainSettings {
    pub imagery: Source,
    /// Tiles are split while a texel of their imagery would cover more than this many pixels.
    pub lod_threshold: f64,
    /// How much less detail tiles seen at grazing angles get, from `0` for none, to `1` for
    /// as much less as they are foreshortened.
    pub anisotropy: f64,
    /// Stop updating which tiles are drawn, to inspect the culling from elsewhere.
    pub freeze: bool,
}

impl Default for TerrainSettings {
    fn default() -> Self {
        Self {
            imagery: Source::Satellite,
            lod_threshold: 1.2,
            anisotropy: 0.5,
            freeze: false,
        }
    }
}

#[derive(Default, Clone, Copy)]
pub struct Stats {
    pub rendered: usize,
    pub nodes: usize,
    pub building: usize,
    pub max_zoom: u8,
    /// Number of times a drawn tile was replaced by a coarser one. When the camera only rotates,
    /// the level of detail should not change, so this should stay put.
    pub coarsened: usize,
}

#[derive(Resource, Default)]
pub struct Terrain {
    nodes: HashMap<TileId, Node>,
    rendered: HashSet<TileId>,
    queue: Vec<Build>,
    building: usize,
    frame: u64,
    /// What the tiles were seen from in the previous frame.
    last_eye: Option<Eye>,
    pub stats: Stats,
}

struct Build {
    rebuild: bool,
    zoom: u8,
    distance: f64,
    tile: TileId,
}

struct Node {
    bounds: Bounds,
    built: Option<Built>,
    task: Option<Task<TileMesh>>,
    /// The imagery the tile is drawn with, which its material keeps alive, the imagery of the
    /// level above that it blends into, and the imagery it is fading in from.
    imagery: Option<Shown>,
    coarser: Option<Shown>,
    fade: Option<Fade>,
    /// How far the tile has faded in from its parent, from `0`, where it looks like its
    /// parent, to `1`.
    morph: f32,
    /// The time the tile was last wanted, in seconds.
    wanted: f64,
    /// The last frame in which the tile was in view.
    visited: u64,
    /// Whether the tile was in view in the previous frame, so that changes to it are seen and
    /// fade, rather than happen at once.
    seen: bool,
    /// Whether the tile was drawn as its children in the last frame in which it was visited.
    split: bool,
    /// Whether the tile is drawn in this frame, but was not in the last.
    fresh: bool,
    /// Which edges border a coarser tile, and are moved onto it, see [`coarser_edges`].
    edges: [bool; 4],
    /// What the material was last set to, so that it is only written when that changes.
    drawn: Option<Drawn>,
}

#[derive(Clone)]
struct Shown {
    key: Key,
    image: Handle<Image>,
}

/// Blending from one imagery into the next.
struct Fade {
    /// The previous imagery, or the placeholder, and the imagery above it.
    from: Option<Shown>,
    from_coarser: Option<Shown>,
    progress: f32,
}

#[derive(PartialEq)]
struct Drawn {
    imagery: Option<Key>,
    coarser: Option<Key>,
    previous: Option<Key>,
    previous_coarser: Option<Key>,
    morph: f32,
    fade: f32,
    edges: [bool; 4],
    /// The detail threshold and anisotropy.
    lod: (f64, f64),
}

struct Built {
    entity: Entity,
    material: Handle<TerrainMaterial>,
    center: DVec3,
    heights: (f32, f32),
    /// The elevation tile of the latest mesh, which may still be waiting to be swapped in, and
    /// how many of its neighbours it was built with.
    elevation: Option<TileId>,
    neighbours: usize,
    /// The frame from which the mesh can be drawn.
    ready: u64,
    swap: Option<Swap>,
}

/// A rebuilt mesh waiting to replace the current one.
struct Swap {
    mesh: Handle<Mesh>,
    heights: (f32, f32),
    frame: u64,
}

impl Node {
    fn new(tile: TileId) -> Self {
        Self {
            bounds: Bounds::new(tile),
            built: None,
            task: None,
            imagery: None,
            coarser: None,
            fade: None,
            morph: 1.0,
            wanted: 0.0,
            visited: 0,
            seen: false,
            split: false,
            fresh: false,
            edges: [false; 4],
            drawn: None,
        }
    }

    fn ready(&self, frame: u64) -> bool {
        self.built
            .as_ref()
            .is_some_and(|built| frame >= built.ready)
    }

    /// The range of heights within the tile, or `fallback` while it is not known.
    fn heights(&self, fallback: (f32, f32)) -> (f32, f32) {
        self.built.as_ref().map_or(fallback, |built| built.heights)
    }
}

/// The extent of a tile on the globe.
struct Bounds {
    west: f64,
    east: f64,
    south: f64,
    north: f64,
    /// The up direction at the middle of the tile, and the largest angle from it to the edge.
    center: DVec3,
    angle: f64,
    /// Up directions at the corners, the middle of the edges, and the center.
    samples: [DVec3; 9],
}

impl Bounds {
    fn new(tile: TileId) -> Self {
        let (west, east, south, north) = tile.bounds();
        let mut samples = [DVec3::ZERO; 9];
        for (i, sample) in samples.iter_mut().enumerate() {
            let (u, v) = ((i % 3) as f64 / 2.0, (i / 3) as f64 / 2.0);
            *sample = geo::up(geo::unit_to_lon_lat(tile.unit(u, v)));
        }
        let center = samples[4];
        let angle = samples
            .iter()
            .map(|s| s.dot(center).clamp(-1.0, 1.0).acos())
            .fold(0.0, f64::max);
        Self {
            west,
            east,
            south,
            north,
            center,
            // The edges bulge out between the samples
            angle: angle * 1.05,
            samples,
        }
    }
}

/// What the tiles are seen from.
#[derive(Clone)]
struct Eye {
    position: DVec3,
    lon_lat: DVec2,
    height: f64,
    up: DVec3,
    /// Angle from the camera to its horizon, seen from the center of the earth.
    horizon: f64,
    /// Inward normals of the left, right, bottom and top planes of the view frustum.
    planes: [DVec3; 4],
    pixels_per_radian: f64,
    threshold: f64,
    anisotropy: f64,
    imagery: Source,
    /// How far tiles fade in or out in this frame.
    fade_step: f32,
    /// Seconds since the start.
    now: f64,
}

impl Eye {
    fn new(view: &View, settings: &TerrainSettings, dt: f32, now: f64) -> Self {
        let (lon_lat, height) = view.geodetic();
        let (forward, right) = (view.forward, view.right());
        let up = right.cross(forward);
        let tan_y = (view.fov / 2.0).tan();
        let tan_x = tan_y * view.aspect;
        let planes = [
            (forward - right * tan_x).cross(up).normalize(),
            up.cross(forward + right * tan_x).normalize(),
            right.cross(forward - up * tan_y).normalize(),
            (forward + up * tan_y).cross(right).normalize(),
        ];
        let radius = view.position.length();
        Self {
            position: view.position,
            lon_lat,
            height,
            up: view.up(),
            horizon: (EARTH_RADIUS / radius).min(1.0).acos(),
            planes,
            pixels_per_radian: view.viewport_height / (2.0 * tan_y),
            threshold: settings.lod_threshold,
            anisotropy: settings.anisotropy,
            imagery: settings.imagery,
            fade_step: dt / FADE_SECONDS,
            now,
        }
    }

    fn visible(&self, tile: TileId, bounds: &Bounds, (min, max): (f32, f32)) -> bool {
        if tile.zoom < 2 {
            return true;
        }

        // Entirely beyond the horizon, which is lowest where the terrain is highest
        let to_tile = self.up.dot(bounds.center).clamp(-1.0, 1.0).acos();
        let beyond = (EARTH_RADIUS / (EARTH_RADIUS + max as f64)).acos();
        if to_tile - bounds.angle > self.horizon + beyond {
            return false;
        }

        // Entirely outside the view, using a bounding sphere of the samples at both heights
        let points = bounds.samples.iter().flat_map(|s| {
            [
                *s * (EARTH_RADIUS + min as f64),
                *s * (EARTH_RADIUS + max as f64),
            ]
        });
        let center = points.clone().sum::<DVec3>() / 18.0;
        let bulge = EARTH_RADIUS * (1.0 - (bounds.angle / 2.0).cos());
        let radius = points.map(|p| p.distance(center)).fold(0.0, f64::max) + bulge;
        let offset = center - self.position;
        self.planes.iter().all(|plane| plane.dot(offset) > -radius)
    }

    /// Distance from the camera to the closest point of the tile, and how many pixels a texel
    /// of its imagery covers there.
    fn detail(&self, tile: TileId, bounds: &Bounds, (min, max): (f32, f32)) -> (f64, f64) {
        let lon = geo::clamp_angle(self.lon_lat.x, bounds.west, bounds.east);
        let lat = self.lon_lat.y.clamp(bounds.south, bounds.north);
        let height = self.height.clamp(min as f64, max as f64);
        let closest = DVec2::new(lon, lat);
        let offset = self.position - geo::to_ecef(closest, height);
        let distance = offset.length().max(1.0);

        // The scale of the mercator projection varies with the latitude
        let texel = TAU * EARTH_RADIUS * lat.cos() / geo::tiles_at(tile.zoom) as f64 / 256.0;
        let facing = (offset / distance).dot(geo::up(closest)).abs();
        let foreshortening = 1.0 - self.anisotropy * (1.0 - facing);
        (
            distance,
            texel / distance * self.pixels_per_radian * foreshortening,
        )
    }
}

impl Terrain {
    /// Select the tiles to draw within `tile`, which is in view of `eye`, and was in view of
    /// `last_eye` if [`Node::seen`]. While `merging`, its parent is waiting for it to fade out.
    fn visit(
        &mut self,
        eye: &Eye,
        last_eye: Option<&Eye>,
        tiles: &mut Tiles,
        tile: TileId,
        heights: (f32, f32),
        merging: bool,
        out: &mut Vec<TileId>,
    ) {
        let frame = self.frame;
        let node = self.nodes.entry(tile).or_insert_with(|| Node::new(tile));
        node.wanted = eye.now;
        // Whether the tile was split is only known if it was visited, as its parent may have
        // been drawn instead
        let visited_before = node.visited + 1 == frame;
        node.visited = frame;
        let heights = node.heights(heights);
        node.seen = last_eye.is_some_and(|last| last.visible(tile, &node.bounds, heights));
        let (distance, detail) = eye.detail(tile, &node.bounds, heights);

        tiles.request(
            (eye.imagery, tile.ancestor(eye.imagery.max_zoom())),
            distance as f32,
        );
        let elevation = mesh::elevation_tile(tile, Source::Terrain.max_zoom());
        tiles.request((Source::Terrain, elevation), distance as f32);

        // Rebuild the mesh when better elevation data is available, or more of the data around
        // it, which its edges are interpolated with
        if node.task.is_none() {
            let best = tiles.best_elevation(elevation).map(|(id, _)| id);
            let rebuild = match &node.built {
                None => Some(false),
                Some(built) if best.map(|id| id.zoom) > built.elevation.map(|id| id.zoom) => {
                    Some(true)
                }
                Some(built)
                    if best.is_some_and(|id| {
                        let around = Elevation::neighbour_ids(id)
                            .filter(|(_, id)| tiles.has_elevation(*id))
                            .count();
                        best == built.elevation && around > built.neighbours
                    }) =>
                {
                    Some(true)
                }
                Some(_) => None,
            };
            if let Some(rebuild) = rebuild {
                self.queue.push(Build {
                    rebuild,
                    zoom: tile.zoom,
                    distance,
                    tile,
                });
            }
        }
        let ready = node.ready(frame);
        let (split, morph) = (node.split && visited_before, node.morph);
        let threshold = match split {
            true => eye.threshold * SPLIT_HYSTERESIS,
            false => eye.threshold,
        };
        let refine = tile.zoom < eye.imagery.max_zoom() && detail > threshold;
        // Only a tile that has faded in splits, so that its children fade in from what is
        // drawn. A split tile that no longer needs its children keeps them while they fade out.
        let fade_out = split && !refine && detail > eye.threshold * MERGE_AT_ONCE;

        if (refine && (split || morph >= 1.0)) || fade_out {
            let children = tile.children();
            let mut visible = [false; 4];
            let mut seen = [false; 4];
            let mut children_ready = true;
            let mut faded_out = true;
            for (i, child) in children.into_iter().enumerate() {
                let node = self.nodes.entry(child).or_insert_with(|| Node::new(child));
                node.wanted = eye.now;
                let child_heights = node.heights(heights);
                visible[i] = eye.visible(child, &node.bounds, child_heights);
                seen[i] =
                    last_eye.is_some_and(|last| last.visible(child, &node.bounds, child_heights));
                faded_out &= !visible[i] || (!node.split && node.morph <= 0.0);
                if !node.ready(frame) {
                    children_ready = false;
                    if node.built.is_none() && node.task.is_none() {
                        self.queue.push(Build {
                            rebuild: false,
                            zoom: child.zoom,
                            distance,
                            tile: child,
                        });
                    }
                }
            }
            if children_ready && !(fade_out && faded_out) {
                // Children only fade in where the tile was in view. Where the camera just
                // turned or moved to, they appear at once.
                for (i, child) in children.into_iter().enumerate() {
                    let Some(node) = self.nodes.get_mut(&child) else {
                        continue;
                    };
                    if !split {
                        node.morph = if seen[i] { 0.0 } else { 1.0 };
                    } else if visible[i] && !seen[i] {
                        // Drawn the way the rest of the tile is, which is faded out into the
                        // tile when it is merging
                        node.morph = if fade_out { 0.0 } else { 1.0 };
                    }
                }
                if fade_out {
                    // The children fade out into the imagery the tile will be drawn with
                    let best = tiles.best_imagery(eye.imagery, tile);
                    if let Some(node) = self.nodes.get_mut(&tile)
                        && let Some(best) = better_imagery(&node.imagery, eye.imagery, best)
                    {
                        node.imagery = best;
                    }
                }
                self.set_split(tile, true);
                for (child, _) in children.into_iter().zip(visible).filter(|(_, v)| *v) {
                    self.visit(eye, last_eye, tiles, child, heights, fade_out, out);
                }
                return;
            }
        }

        self.set_split(tile, false);
        if ready {
            let fresh = !self.rendered.contains(&tile);
            if let Some(node) = self.nodes.get_mut(&tile) {
                let (target, step) = match merging {
                    true => (0.0, eye.fade_step),
                    false => {
                        let speedup = (detail / eye.threshold).clamp(1.0, MAX_FADE_SPEEDUP);
                        (1.0, eye.fade_step * speedup as f32)
                    }
                };
                node.morph += (target - node.morph).clamp(-step, step);
                node.fresh = fresh;
            }
            out.push(tile);
        }
    }

    fn set_split(&mut self, tile: TileId, split: bool) {
        if let Some(node) = self.nodes.get_mut(&tile) {
            node.split = split;
        }
    }

    /// Start building the most important meshes, coarse tiles before fine ones.
    fn start_builds(&mut self, tiles: &mut Tiles) {
        let mut queue = std::mem::take(&mut self.queue);
        queue.sort_by(|a, b| {
            (a.rebuild, a.zoom, a.distance)
                .partial_cmp(&(b.rebuild, b.zoom, b.distance))
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let pool = AsyncComputeTaskPool::get();
        for Build { tile, .. } in queue.drain(..) {
            if self.building >= MAX_BUILDING {
                break;
            }
            let Some(node) = self.nodes.get_mut(&tile) else {
                continue;
            };
            if node.task.is_some() {
                continue;
            }
            let elevation = elevation_of(tiles, tile);
            if let Some(built) = &node.built
                && built.elevation == elevation.as_ref().map(|e| e.id)
                && built.neighbours == elevation.as_ref().map_or(0, Elevation::neighbours)
            {
                continue;
            }
            // What the parent is drawn with, to fade in from
            let parent_elevation = tile.parent().and_then(|parent| elevation_of(tiles, parent));
            node.task =
                Some(pool.spawn(async move {
                    mesh::build_tile_mesh(tile, elevation, parent_elevation)
                }));
            self.building += 1;
        }
        self.queue = queue;
    }
}

/// The best elevation data available for a tile, with that around it.
fn elevation_of(tiles: &mut Tiles, tile: TileId) -> Option<Elevation> {
    let (id, grid) =
        tiles.best_elevation(mesh::elevation_tile(tile, Source::Terrain.max_zoom()))?;
    Some(Elevation::new(id, grid, |id| tiles.elevation_grid(id)))
}

fn select_tiles(
    mut terrain: ResMut<Terrain>,
    mut tiles: ResMut<Tiles>,
    view: Res<View>,
    settings: Res<TerrainSettings>,
    time: Res<Time>,
) {
    let terrain = terrain.as_mut();
    terrain.frame += 1;
    if settings.freeze {
        // Keep the drawn tiles and their data
        let now = time.elapsed_secs_f64();
        for tile in &terrain.rendered {
            if let Some(node) = terrain.nodes.get_mut(tile) {
                node.wanted = now;
            }
        }
        return;
    }

    let eye = Eye::new(&view, &settings, time.delta_secs(), time.elapsed_secs_f64());
    let last_eye = terrain.last_eye.take();
    let mut rendered = Vec::new();
    terrain.visit(
        &eye,
        last_eye.as_ref(),
        &mut tiles,
        TileId::ROOT,
        (0.0, MAX_HEIGHT),
        false,
        &mut rendered,
    );
    terrain.start_builds(&mut tiles);
    terrain.last_eye = Some(eye);

    let current: HashSet<TileId> = rendered.into_iter().collect();
    terrain.stats.coarsened += terrain
        .rendered
        .iter()
        .filter(|tile| {
            tile.lineage()
                .skip(1)
                .any(|parent| current.contains(&parent))
        })
        .count();
    terrain.stats.max_zoom = current.iter().map(|t| t.zoom).max().unwrap_or(0);
    for tile in &current {
        let edges = coarser_edges(*tile, &current);
        if let Some(node) = terrain.nodes.get_mut(tile) {
            node.edges = edges;
        }
    }
    terrain.rendered = current;
}

/// Which edges of a drawn tile border a coarser drawn tile, in the order of the texture
/// coordinates: `u = 0`, `u = 1`, `v = 0` and `v = 1`. The coarser tile covers the whole edge.
fn coarser_edges(tile: TileId, rendered: &HashSet<TileId>) -> [bool; 4] {
    let beyond = |u: f64, v: f64| {
        let unit = tile.unit(u, v);
        if !(0.0..1.0).contains(&unit.y) {
            return false;
        }
        let unit = DVec2::new(unit.x.rem_euclid(1.0), unit.y);
        TileId::containing(unit, tile.zoom)
            .lineage()
            .find(|t| rendered.contains(t))
            .is_some_and(|neighbour| neighbour.zoom < tile.zoom)
    };
    const OUT: f64 = 1e-3;
    [
        beyond(-OUT, 0.5),
        beyond(1.0 + OUT, 0.5),
        beyond(0.5, -OUT),
        beyond(0.5, 1.0 + OUT),
    ]
}

fn poll_meshes(
    mut commands: Commands,
    mut terrain: ResMut<Terrain>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<TerrainMaterial>>,
    luts: Res<ScatteringLuts>,
) {
    let terrain = terrain.as_mut();
    for node in terrain.nodes.values_mut() {
        let Some(task) = &mut node.task else { continue };
        let Some(tile_mesh) = check_ready(task) else {
            continue;
        };
        node.task = None;
        terrain.building -= 1;

        let mesh = meshes.add(tile_mesh.mesh);
        let planet_center = (-tile_mesh.center).as_vec3();
        let heights = (tile_mesh.min_height, tile_mesh.max_height);
        let ready = terrain.frame + SETTLE_FRAMES;

        match &mut node.built {
            // The origin of a tile never changes, so only the mesh needs to be swapped
            Some(built) => {
                built.elevation = tile_mesh.elevation;
                built.neighbours = tile_mesh.neighbours;
                built.swap = Some(Swap {
                    mesh,
                    heights,
                    frame: ready,
                });
            }
            None => {
                let material = materials.add(TerrainMaterial::new(
                    TerrainParams::new(planet_center, PLACEHOLDER),
                    &luts,
                ));
                let entity = commands
                    .spawn((
                        Mesh3d(mesh),
                        MeshMaterial3d(material.clone()),
                        Transform::default(),
                        Visibility::Hidden,
                        // Culling is done by the terrain itself
                        NoFrustumCulling,
                        NotShadowCaster,
                    ))
                    .id();
                node.built = Some(Built {
                    entity,
                    material,
                    center: tile_mesh.center,
                    heights,
                    elevation: tile_mesh.elevation,
                    neighbours: tile_mesh.neighbours,
                    ready,
                    swap: None,
                });
                node.drawn = None;
            }
        }
    }
}

fn update_tiles(
    mut commands: Commands,
    mut terrain: ResMut<Terrain>,
    mut tiles: ResMut<Tiles>,
    mut materials: ResMut<Assets<TerrainMaterial>>,
    mut entities: Query<(&mut Transform, &mut Visibility)>,
    view: Res<View>,
    settings: Res<TerrainSettings>,
    time: Res<Time>,
) {
    let terrain = terrain.as_mut();
    let frame = terrain.frame;

    for (tile, node) in terrain.nodes.iter_mut() {
        let Some(built) = &mut node.built else {
            continue;
        };
        if let Some(swap) = built.swap.take_if(|swap| frame >= swap.frame) {
            commands.entity(built.entity).insert(Mesh3d(swap.mesh));
            built.heights = swap.heights;
        }
        let Ok((mut transform, mut visibility)) = entities.get_mut(built.entity) else {
            continue;
        };
        if !terrain.rendered.contains(tile) {
            visibility.set_if_neq(Visibility::Hidden);
            continue;
        }
        visibility.set_if_neq(Visibility::Visible);
        transform.translation = (built.center - view.position).as_vec3();
    }

    let fade_step = time.delta_secs() / FADE_SECONDS;
    let source = settings.imagery;
    for tile in &terrain.rendered {
        // What the parent was drawn with, which the tile fades in from
        let parent = tile
            .parent()
            .and_then(|parent| terrain.nodes.get(&parent))
            .map(|parent| (parent.imagery.clone(), parent.coarser.clone()));
        let Some(node) = terrain.nodes.get_mut(tile) else {
            continue;
        };
        let Some(built) = &node.built else {
            continue;
        };

        // The imagery the tile blends into, which is that of its parent, or its own if that is
        // no finer
        let better = better_imagery(&node.imagery, source, tiles.best_imagery(source, *tile));
        let imagery = better.clone().unwrap_or_else(|| node.imagery.clone());
        let best_coarser = match &imagery {
            Some(own) if own.key.1.zoom < tile.zoom => Some((own.key.1, own.image.clone())),
            Some(_) => tile
                .parent()
                .and_then(|parent| tiles.best_imagery(source, parent)),
            None => None,
        };
        let better_coarser = better_imagery(&node.coarser, source, best_coarser);
        let changed = better.is_some() || better_coarser.is_some();
        let coarser = better_coarser.unwrap_or_else(|| node.coarser.clone());

        // A tile takes the best imagery at once where it was not in view, or while it starts to
        // fade in from its parent. Otherwise it blends into better imagery, once it has faded
        // in, which includes where its children just faded out into it.
        if !node.seen || (node.fresh && node.morph < 1.0) {
            node.fade = None;
            (node.imagery, node.coarser) = (imagery, coarser);
        } else if node.morph >= 1.0 && changed {
            node.fade = Some(Fade {
                from: std::mem::replace(&mut node.imagery, imagery),
                from_coarser: std::mem::replace(&mut node.coarser, coarser),
                progress: 0.0,
            });
        }
        if let Some(fade) = &mut node.fade {
            fade.progress += fade_step;
            if fade.progress >= 1.0 {
                node.fade = None;
            }
        }

        let (previous, previous_coarser, previous_zoom, weight) = match &node.fade {
            _ if node.morph < 1.0 => {
                let (imagery, coarser) = parent.unwrap_or_default();
                (imagery, coarser, tile.zoom.saturating_sub(1), node.morph)
            }
            Some(fade) => (
                fade.from.clone(),
                fade.from_coarser.clone(),
                tile.zoom,
                fade.progress,
            ),
            None => (None, None, tile.zoom, 1.0),
        };
        // Without imagery above, the imagery blends into itself
        let coarser = node.coarser.clone().or_else(|| node.imagery.clone());
        let previous_coarser = previous_coarser.or_else(|| previous.clone());
        let key = |shown: &Option<Shown>| shown.as_ref().map(|shown| shown.key);
        let drawn = Drawn {
            imagery: key(&node.imagery),
            coarser: key(&coarser),
            previous: key(&previous),
            previous_coarser: key(&previous_coarser),
            morph: node.morph,
            edges: node.edges,
            fade: weight,
            lod: (settings.lod_threshold, settings.anisotropy),
        };
        if node.drawn.as_ref() == Some(&drawn) {
            continue;
        }
        let Some(mut material) = materials.get_mut(&built.material) else {
            continue;
        };
        let params = &mut material.params;
        (params.uv_transform, params.base_color) = imagery_params(*tile, &node.imagery);
        (params.previous_uv_transform, params.previous_color) = imagery_params(*tile, &previous);
        params.coarser_uv_transform = imagery_params(*tile, &coarser).0;
        params.previous_coarser_uv_transform = imagery_params(*tile, &previous_coarser).0;
        params.fade = Vec4::new(ease(node.morph), ease(weight), 0.0, 0.0);
        params.edges = Vec4::from_array(node.edges.map(|edge| edge as u8 as f32));
        params.lod = Vec4::new(
            settings.lod_threshold as f32,
            settings.anisotropy as f32,
            tile.zoom as f32,
            previous_zoom as f32,
        );
        let image = |shown: Option<Shown>| shown.map(|shown| shown.image);
        material.imagery = image(node.imagery.clone());
        material.coarser = image(coarser);
        material.previous = image(previous);
        material.previous_coarser = image(previous_coarser);
        node.drawn = Some(drawn);
    }

    // Drop tiles which have not been wanted for a while
    terrain.nodes.retain(|tile, node| {
        let keep = tile.zoom <= 2 || time.elapsed_secs_f64() - node.wanted < EVICT_SECONDS;
        if !keep {
            if let Some(built) = &node.built {
                commands.entity(built.entity).despawn();
            }
            if node.task.is_some() {
                terrain.building -= 1;
            }
        }
        keep
    });

    terrain.stats.rendered = terrain.rendered.len();
    terrain.stats.nodes = terrain.nodes.len();
    terrain.stats.building = terrain.building;
}

/// The imagery to replace `current` with, if `best` is better. The material of a tile keeps
/// its imagery alive, so a tile never reverts to coarser imagery because it was evicted from
/// the store, unless the source changed.
fn better_imagery(
    current: &Option<Shown>,
    source: Source,
    best: Option<(TileId, Handle<Image>)>,
) -> Option<Option<Shown>> {
    let best = best.map(|(id, image)| Shown {
        key: (source, id),
        image,
    });
    let (current, best_key) = (
        current.as_ref().map(|c| c.key),
        best.as_ref().map(|b| b.key),
    );
    let coarser = current.is_some_and(|(current_source, current)| {
        current_source == source && best_key.is_none_or(|(_, best)| best.zoom < current.zoom)
    });
    (best_key != current && !coarser).then_some(best)
}

/// The texture transform and color of a tile drawn with some imagery, or the placeholder.
fn imagery_params(tile: TileId, imagery: &Option<Shown>) -> (Vec4, Vec4) {
    match imagery {
        Some(Shown { key: (_, id), .. }) => {
            let (offset, scale) = tile.uv_in_ancestor(*id);
            (
                Vec4::new(offset.x as f32, offset.y as f32, scale as f32, 0.0),
                Vec4::ONE,
            )
        }
        None => (
            Vec4::new(0.0, 0.0, 1.0, 0.0),
            PLACEHOLDER.to_linear().to_vec4(),
        ),
    }
}

/// Eases in and out of fades.
fn ease(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// One of the caps closing the globe beyond the reach of the Web Mercator tiles.
#[derive(Component)]
struct PoleCap {
    north: bool,
    center: DVec3,
    material: Handle<TerrainMaterial>,
}

#[derive(Default)]
struct CapState {
    elevation_version: u64,
    imagery: Option<Source>,
    built: f64,
}

fn spawn_caps(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<TerrainMaterial>>,
    luts: Res<ScatteringLuts>,
) {
    for north in [true, false] {
        let (mesh, center) = mesh::build_cap_mesh(north, |_| 0.0);
        let material = materials.add(TerrainMaterial::new(
            TerrainParams::new((-center).as_vec3(), Color::WHITE),
            &luts,
        ));
        commands.spawn((
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(material.clone()),
            Transform::default(),
            NoFrustumCulling,
            NotShadowCaster,
            PoleCap {
                north,
                center,
                material,
            },
        ));
    }
}

fn cap_color(source: Source, north: bool) -> Color {
    match (source, north) {
        (Source::OpenStreetMap, true) => Color::srgb_u8(170, 211, 223),
        (Source::OpenStreetMap, false) => Color::srgb_u8(242, 242, 242),
        (_, true) => Color::srgb(0.78, 0.82, 0.86),
        (_, false) => Color::srgb(0.9, 0.92, 0.95),
    }
}

fn update_caps(
    mut caps: Query<(&mut PoleCap, &mut Transform, &mut Mesh3d)>,
    mut materials: ResMut<Assets<TerrainMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut state: Local<CapState>,
    tiles: Res<Tiles>,
    view: Res<View>,
    settings: Res<TerrainSettings>,
    time: Res<Time>,
) {
    // Follow the rims of the coarser tiles as their elevation streams in
    let now = time.elapsed_secs_f64();
    let rebuild = tiles.elevation_version != state.elevation_version && now - state.built > 1.0;
    let recolor = state.imagery != Some(settings.imagery);

    for (mut cap, mut transform, mut mesh3d) in &mut caps {
        if rebuild {
            let lat = geo::max_mercator_lat() * if cap.north { 1.0 } else { -1.0 };
            let (mesh, center) = mesh::build_cap_mesh(cap.north, |lon| {
                tiles.height_at(DVec2::new(lon, lat), 4).unwrap_or(0.0)
            });
            mesh3d.0 = meshes.add(mesh);
            cap.center = center;
        }
        if (rebuild || recolor)
            && let Some(mut material) = materials.get_mut(&cap.material)
        {
            material.params.planet_center = (-cap.center).as_vec3().extend(0.0);
            material.params.base_color =
                cap_color(settings.imagery, cap.north).to_linear().to_vec4();
        }
        transform.translation = (cap.center - view.position).as_vec3();
    }

    if rebuild {
        state.elevation_version = tiles.elevation_version;
        state.built = now;
    }
    state.imagery = Some(settings.imagery);
}
