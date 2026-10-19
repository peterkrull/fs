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
//! through finer geometry.

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
    atmosphere::Sun,
    camera::{CameraSystems, View},
    geo::{self, EARTH_RADIUS, MAX_HEIGHT, TileId},
    material::{TerrainMaterial, TerrainParams},
    mesh::{self, TileMesh},
    sources::Source,
    tiles::Tiles,
};

/// The most meshes being built at once.
const MAX_BUILDING: usize = 48;
/// Tiles not wanted for this many frames are dropped.
const EVICT_FRAMES: u64 = 180;
/// A split tile is only merged again once its detail drops this much below the threshold,
/// so that tiles at the threshold do not alternate between levels.
const SPLIT_HYSTERESIS: f64 = 0.8;
/// New meshes are only drawn after this many frames, once they are uploaded to the GPU.
/// Drawing them earlier would leave a hole in the terrain for a frame.
const SETTLE_FRAMES: u64 = 2;
/// Color of tiles without any imagery yet.
const PLACEHOLDER: Color = Color::srgb(0.18, 0.22, 0.28);

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
    imagery: Option<(Source, TileId)>,
    wanted: u64,
    /// Whether the tile was drawn as its children in the last frame.
    split: bool,
}

struct Built {
    entity: Entity,
    material: Handle<TerrainMaterial>,
    center: DVec3,
    heights: (f32, f32),
    /// The elevation tile of the latest mesh, which may still be waiting to be swapped in.
    elevation: Option<TileId>,
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
            wanted: 0,
            split: false,
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
}

impl Eye {
    fn new(view: &View, settings: &TerrainSettings) -> Self {
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
    fn visit(
        &mut self,
        eye: &Eye,
        tiles: &mut Tiles,
        tile: TileId,
        heights: (f32, f32),
        out: &mut Vec<TileId>,
    ) {
        let frame = self.frame;
        let node = self.nodes.entry(tile).or_insert_with(|| Node::new(tile));
        node.wanted = frame;
        let heights = node.heights(heights);
        let (distance, detail) = eye.detail(tile, &node.bounds, heights);

        tiles.request(
            (eye.imagery, tile.ancestor(eye.imagery.max_zoom())),
            distance as f32,
        );
        let elevation = mesh::elevation_tile(tile, Source::Terrain.max_zoom());
        tiles.request((Source::Terrain, elevation), distance as f32);

        // Rebuild the mesh when better elevation data is available
        if node.task.is_none() {
            let best = tiles.best_elevation(elevation).map(|(id, _)| id.zoom);
            let rebuild = match &node.built {
                None => Some(false),
                Some(built) if best > built.elevation.map(|id| id.zoom) => Some(true),
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
        let threshold = match node.split {
            true => eye.threshold * SPLIT_HYSTERESIS,
            false => eye.threshold,
        };

        if tile.zoom < eye.imagery.max_zoom() && detail > threshold {
            let children = tile.children();
            let mut visible = [false; 4];
            let mut children_ready = true;
            for (i, child) in children.into_iter().enumerate() {
                let node = self.nodes.entry(child).or_insert_with(|| Node::new(child));
                node.wanted = frame;
                visible[i] = eye.visible(child, &node.bounds, node.heights(heights));
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
            if children_ready {
                self.set_split(tile, true);
                for (child, _) in children.into_iter().zip(visible).filter(|(_, v)| *v) {
                    self.visit(eye, tiles, child, heights, out);
                }
                return;
            }
        }

        self.set_split(tile, false);
        if ready {
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
            let elevation =
                tiles.best_elevation(mesh::elevation_tile(tile, Source::Terrain.max_zoom()));
            if let Some(built) = &node.built
                && built.elevation == elevation.as_ref().map(|(id, _)| *id)
            {
                continue;
            }
            node.task = Some(pool.spawn(async move { mesh::build_tile_mesh(tile, elevation) }));
            self.building += 1;
        }
        self.queue = queue;
    }
}

fn select_tiles(
    mut terrain: ResMut<Terrain>,
    mut tiles: ResMut<Tiles>,
    view: Res<View>,
    settings: Res<TerrainSettings>,
) {
    let terrain = terrain.as_mut();
    terrain.frame += 1;
    if settings.freeze {
        // Keep the drawn tiles and their data
        let frame = terrain.frame;
        for tile in &terrain.rendered {
            if let Some(node) = terrain.nodes.get_mut(tile) {
                node.wanted = frame;
            }
        }
        return;
    }

    let eye = Eye::new(&view, &settings);
    let mut rendered = Vec::new();
    terrain.visit(
        &eye,
        &mut tiles,
        TileId::ROOT,
        (0.0, MAX_HEIGHT),
        &mut rendered,
    );
    terrain.start_builds(&mut tiles);

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
    terrain.rendered = current;
}

fn poll_meshes(
    mut commands: Commands,
    mut terrain: ResMut<Terrain>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<TerrainMaterial>>,
    sun: Res<Sun>,
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
        let planet_center = (-tile_mesh.center).as_vec3().extend(0.0);
        let heights = (tile_mesh.min_height, tile_mesh.max_height);
        let ready = terrain.frame + SETTLE_FRAMES;

        match &mut node.built {
            // The origin of a tile never changes, so only the mesh needs to be swapped
            Some(built) => {
                built.elevation = tile_mesh.elevation;
                built.swap = Some(Swap {
                    mesh,
                    heights,
                    frame: ready,
                });
            }
            None => {
                let material = materials.add(TerrainMaterial {
                    params: TerrainParams {
                        uv_transform: Vec4::new(0.0, 0.0, 1.0, 0.0),
                        planet_center,
                        base_color: PLACEHOLDER.to_linear().to_vec4(),
                        sun: sun.direction().extend(0.0),
                    },
                    imagery: None,
                });
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
                    ready,
                    swap: None,
                });
                node.imagery = None;
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
    sun: Res<Sun>,
) {
    let terrain = terrain.as_mut();
    let frame = terrain.frame;

    if sun.is_changed() {
        for built in terrain
            .nodes
            .values()
            .filter_map(|node| node.built.as_ref())
        {
            if let Some(mut material) = materials.get_mut(&built.material) {
                material.params.sun = sun.direction().extend(0.0);
            }
        }
    }

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

        // The material keeps its imagery alive, so a tile never reverts to coarser imagery
        // because it was evicted from the store
        let best = tiles.best_imagery(settings.imagery, *tile);
        let best_id = best.as_ref().map(|(id, _)| (settings.imagery, *id));
        let coarser = match (node.imagery, best_id) {
            (Some((source, current)), best) => {
                source == settings.imagery && best.is_none_or(|(_, best)| best.zoom < current.zoom)
            }
            (None, _) => false,
        };
        if best_id != node.imagery
            && !coarser
            && let Some(mut material) = materials.get_mut(&built.material)
        {
            node.imagery = best_id;
            match best {
                Some((id, image)) => {
                    let (offset, scale) = tile.uv_in_ancestor(id);
                    material.params.uv_transform =
                        Vec4::new(offset.x as f32, offset.y as f32, scale as f32, 0.0);
                    material.params.base_color = Vec4::ONE;
                    material.imagery = Some(image);
                }
                None => {
                    material.params.base_color = PLACEHOLDER.to_linear().to_vec4();
                    material.imagery = None;
                }
            }
        }
    }

    // Drop tiles which have not been wanted for a while
    terrain.nodes.retain(|tile, node| {
        let keep = tile.zoom <= 2 || frame.saturating_sub(node.wanted) < EVICT_FRAMES;
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
    sun: Res<Sun>,
) {
    for north in [true, false] {
        let (mesh, center) = mesh::build_cap_mesh(north, |_| 0.0);
        let material = materials.add(TerrainMaterial {
            params: TerrainParams {
                uv_transform: Vec4::new(0.0, 0.0, 1.0, 0.0),
                planet_center: (-center).as_vec3().extend(0.0),
                base_color: Vec4::ONE,
                sun: sun.direction().extend(0.0),
            },
            imagery: None,
        });
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
    sun: Res<Sun>,
) {
    // Follow the rims of the coarser tiles as their elevation streams in
    let now = time.elapsed_secs_f64();
    let rebuild = tiles.elevation_version != state.elevation_version && now - state.built > 1.0;
    let recolor = state.imagery != Some(settings.imagery) || sun.is_changed();

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
            material.params.sun = sun.direction().extend(0.0);
        }
        transform.translation = (cap.center - view.position).as_vec3();
    }

    if rebuild {
        state.elevation_version = tiles.elevation_version;
        state.built = now;
    }
    state.imagery = Some(settings.imagery);
}
