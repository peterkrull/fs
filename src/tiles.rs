//! The store of streamed tiles, which the terrain draws from.
//!
//! Every frame the terrain requests the tiles it would like to have, after which they are
//! fetched in order of priority: coarser tiles first, and then those closest to the camera.
//! While a tile is not available, the terrain uses the closest ancestor that is.

use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

use bevy::{
    asset::RenderAssetUsages,
    image::{ImageFilterMode, ImageSampler, ImageSamplerDescriptor},
    math::DVec2,
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
};

use crate::{
    fetch::{Fetcher, HeightGrid, Imagery, Key, Outcome, Payload},
    geo::{self, TileId},
    sources::Source,
};

/// Decoded imagery is kept on the GPU with mipmaps, about 350 KB per tile.
const IMAGERY_BUDGET: usize = 768;
/// Height grids are 256 KB each.
const ELEVATION_BUDGET: usize = 384;
/// Tiles at or above this zoom level are never evicted, as a last resort to draw.
const KEEP_ZOOM: u8 = 2;

pub struct TilesPlugin;

impl Plugin for TilesPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Tiles::new())
            .add_systems(Last, stream_tiles);
    }
}

struct Stored<T> {
    value: T,
    used: u64,
    added: u64,
}

/// Images are only handed out after this many frames, once they are uploaded to the GPU.
/// Bevy prepares materials without waiting for their images, and a material whose image is
/// not ready is not drawn, which would leave a hole in the terrain for a frame.
const SETTLE_FRAMES: u64 = 2;

#[derive(Clone, Copy)]
struct Request {
    key: Key,
    distance: f32,
}

#[derive(Resource)]
pub struct Tiles {
    fetcher: Fetcher,
    imagery: HashMap<Key, Stored<Handle<Image>>>,
    elevation: HashMap<TileId, Stored<Arc<HeightGrid>>>,
    missing: HashSet<Key>,
    requests: Vec<Request>,
    frame: u64,
    /// Incremented whenever new elevation data arrives.
    pub elevation_version: u64,
}

impl Tiles {
    fn new() -> Self {
        Self {
            fetcher: Fetcher::new(),
            imagery: HashMap::new(),
            elevation: HashMap::new(),
            missing: HashSet::new(),
            requests: Vec::new(),
            frame: 0,
            elevation_version: 0,
        }
    }

    /// Ask for a tile to be fetched, or kept if it already is.
    pub fn request(&mut self, key: Key, distance: f32) {
        self.requests.push(Request { key, distance });
    }

    /// The best imagery tile available to draw `tile` with, which is either the tile itself
    /// or one of its ancestors.
    pub fn best_imagery(
        &mut self,
        source: Source,
        tile: TileId,
    ) -> Option<(TileId, Handle<Image>)> {
        let frame = self.frame;
        tile.ancestor(source.max_zoom()).lineage().find_map(|id| {
            let stored = self
                .imagery
                .get_mut(&(source, id))
                .filter(|stored| frame >= stored.added + SETTLE_FRAMES)?;
            stored.used = frame;
            Some((id, stored.value.clone()))
        })
    }

    /// The best height data available for `tile`, which is either the tile itself or one of
    /// its ancestors.
    pub fn best_elevation(&mut self, tile: TileId) -> Option<(TileId, Arc<HeightGrid>)> {
        let frame = self.frame;
        tile.ancestor(Source::Terrain.max_zoom())
            .lineage()
            .find_map(|id| {
                let stored = self.elevation.get_mut(&id)?;
                stored.used = frame;
                Some((id, stored.value.clone()))
            })
    }

    /// The height of the terrain at a longitude and latitude, using the best data available
    /// up to `max_zoom`.
    pub fn height_at(&self, lon_lat: DVec2, max_zoom: u8) -> Option<f32> {
        let unit = geo::lon_lat_to_unit(lon_lat);
        let tile = TileId::containing(unit, max_zoom.min(Source::Terrain.max_zoom()));
        tile.lineage().find_map(|id| {
            let grid = &self.elevation.get(&id)?.value;
            Some(grid.sample(id.local(unit)))
        })
    }

    pub fn downloading(&self) -> usize {
        self.fetcher.downloading()
    }

    pub fn pending(&self) -> usize {
        self.fetcher.pending()
    }

    pub fn imagery_count(&self) -> usize {
        self.imagery.len()
    }

    pub fn elevation_count(&self) -> usize {
        self.elevation.len()
    }

    fn insert(&mut self, key: Key, payload: Payload, images: &mut Assets<Image>) {
        let used = self.frame;
        match payload {
            Payload::Imagery(imagery) => {
                let value = images.add(imagery_image(imagery));
                self.imagery.insert(
                    key,
                    Stored {
                        value,
                        used,
                        added: used,
                    },
                );
            }
            Payload::Elevation(grid) => {
                let value = Arc::new(grid);
                self.elevation.insert(
                    key.1,
                    Stored {
                        value,
                        used,
                        added: used,
                    },
                );
                self.elevation_version += 1;
            }
        }
    }

    /// Whether the source does not have the tile, which is also the case when it does not have
    /// one of its ancestors.
    fn is_missing(&self, (source, tile): Key) -> bool {
        tile.lineage()
            .any(|id| self.missing.contains(&(source, id)))
    }

    fn is_stored(&mut self, key: Key) -> bool {
        let frame = self.frame;
        let stored = match key.0 {
            Source::Terrain => self.elevation.get_mut(&key.1).map(|s| &mut s.used),
            _ => self.imagery.get_mut(&key).map(|s| &mut s.used),
        };
        stored.map(|used| *used = frame).is_some()
    }

    /// Start the most important requests that can be started, and cancel those no longer wanted.
    fn dispatch(&mut self) {
        let mut requests = std::mem::take(&mut self.requests);
        requests.sort_by(|a, b| {
            (a.key.1.zoom, a.distance)
                .partial_cmp(&(b.key.1.zoom, b.distance))
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut seen = HashSet::new();
        for Request { key, .. } in requests.iter().copied() {
            if !seen.insert(key) || self.is_missing(key) || self.is_stored(key) {
                continue;
            }
            if !self.fetcher.touch(key, self.frame) && self.fetcher.can_start(key) {
                self.fetcher.start(key, self.frame);
            }
        }

        self.fetcher.cancel_unwanted(self.frame);
        requests.clear();
        self.requests = requests;
    }

    /// Drop the least recently used tiles when over budget. Tiles still drawn by the terrain
    /// keep their image alive through their material until they are no longer drawn.
    fn evict(&mut self) {
        fn evict<K: Copy + Eq + std::hash::Hash, T>(
            map: &mut HashMap<K, Stored<T>>,
            budget: usize,
            frame: u64,
            zoom: impl Fn(&K) -> u8,
        ) {
            if map.len() <= budget {
                return;
            }
            let mut candidates: Vec<_> = map
                .iter()
                .filter(|(key, stored)| zoom(key) > KEEP_ZOOM && stored.used + 1 < frame)
                .map(|(key, stored)| (stored.used, *key))
                .collect();
            candidates.sort_unstable_by_key(|(used, _)| *used);
            let excess = map.len() - budget * 7 / 8;
            for (_, key) in candidates.into_iter().take(excess) {
                map.remove(&key);
            }
        }
        evict(
            &mut self.imagery,
            IMAGERY_BUDGET,
            self.frame,
            |(_, tile)| tile.zoom,
        );
        evict(&mut self.elevation, ELEVATION_BUDGET, self.frame, |tile| {
            tile.zoom
        });
    }
}

fn stream_tiles(mut tiles: ResMut<Tiles>, mut images: ResMut<Assets<Image>>) {
    for fetched in tiles.fetcher.receive() {
        match fetched.outcome {
            Outcome::Loaded(payload) => tiles.insert(fetched.key, payload, &mut images),
            Outcome::Missing => {
                tiles.missing.insert(fetched.key);
            }
            Outcome::Failed => {}
        }
    }

    tiles.dispatch();
    tiles.evict();
    tiles.frame += 1;
}

fn imagery_image(imagery: Imagery) -> Image {
    let mut image = Image::new_uninit(
        Extent3d {
            width: imagery.width,
            height: imagery.height,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    );
    image.data = Some(imagery.data);
    image.texture_descriptor.mip_level_count = imagery.mip_levels;
    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        mag_filter: ImageFilterMode::Linear,
        min_filter: ImageFilterMode::Linear,
        mipmap_filter: ImageFilterMode::Linear,
        anisotropy_clamp: 16,
        ..default()
    });
    image
}
