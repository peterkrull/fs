//! Fetching and decoding of tiles in the background.
//!
//! Requests are started by the main thread in order of priority, and each one runs as a task
//! on a small tokio runtime. A task first looks for the tile in the disk cache, and otherwise
//! downloads it. Downloads are limited by a semaphore per source, and one shared by all
//! sources, so that the number of simultaneous requests never exceeds the limits set out in
//! [`Source::max_in_flight`] and [`MAX_IN_FLIGHT`]. Tasks for tiles that are no longer wanted
//! are aborted, which also cancels their download.

use std::{
    collections::HashMap,
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    },
    time::{Duration, Instant, SystemTime},
};

use bevy::math::DVec2;
use crossbeam_channel::{Receiver, Sender};
use tokio::{runtime::Runtime, sync::Semaphore, task::JoinHandle};

use crate::{geo::TileId, sources::Source};

/// The maximum number of simultaneous downloads, across all sources.
pub const MAX_IN_FLIGHT: usize = 8;

/// Tiles are refetched once their cached copy on disk is older than this.
const CACHE_MAX_AGE: Duration = Duration::from_secs(30 * 24 * 60 * 60);

/// Requests not wanted for this many frames are aborted.
const CANCEL_FRAMES: u64 = 10;

const USER_AGENT: &str = concat!(
    "bevy-worldmap/",
    env!("CARGO_PKG_VERSION"),
    " (3D globe viewer, built with Bevy)"
);

pub type Key = (Source, TileId);

/// Decoded RGBA8 pixels of a tile, including a full chain of mipmaps.
pub struct Imagery {
    pub width: u32,
    pub height: u32,
    pub mip_levels: u32,
    pub data: Vec<u8>,
}

/// Height samples of a tile in meters, row by row from the north-west corner. Where the source
/// has no data, the heights are NaN until [`crate::tiles::Tiles`] fills them in.
pub struct HeightGrid {
    pub size: usize,
    pub heights: Vec<f32>,
}

impl HeightGrid {
    /// Bilinearly interpolated height at `local`, where the tile spans `0..1` on each axis.
    pub fn sample(&self, local: DVec2) -> f32 {
        let max = (self.size - 1) as f64;
        let px = (local.x * self.size as f64 - 0.5).clamp(0.0, max);
        let py = (local.y * self.size as f64 - 0.5).clamp(0.0, max);
        let (x0, y0) = (px.floor() as usize, py.floor() as usize);
        let (x1, y1) = ((x0 + 1).min(self.size - 1), (y0 + 1).min(self.size - 1));
        let (fx, fy) = ((px - x0 as f64) as f32, (py - y0 as f64) as f32);
        let at = |x: usize, y: usize| self.heights[y * self.size + x];
        let top = at(x0, y0) + (at(x1, y0) - at(x0, y0)) * fx;
        let bottom = at(x0, y1) + (at(x1, y1) - at(x0, y1)) * fx;
        top + (bottom - top) * fy
    }
}

pub enum Payload {
    Imagery(Imagery),
    Elevation(HeightGrid),
}

pub enum Outcome {
    Loaded(Payload),
    /// The source does not have this tile.
    Missing,
    Failed,
}

pub struct Fetched {
    pub key: Key,
    pub outcome: Outcome,
}

struct Pending {
    task: JoinHandle<()>,
    wanted: u64,
}

pub struct Fetcher {
    runtime: Runtime,
    client: reqwest::Client,
    cache_dir: Option<Arc<PathBuf>>,
    global: Arc<Semaphore>,
    per_source: HashMap<Source, Arc<Semaphore>>,
    pending: HashMap<Key, Pending>,
    /// Number of failed attempts, and when to try again.
    backoff: HashMap<Key, (u32, Instant)>,
    downloading: Arc<AtomicUsize>,
    sender: Sender<Fetched>,
    receiver: Receiver<Fetched>,
}

impl Default for Fetcher {
    fn default() -> Self {
        Self::new()
    }
}

impl Fetcher {
    pub fn new() -> Self {
        // Decoding runs on the blocking pool, so leave cores for rendering
        let cores = std::thread::available_parallelism().map_or(2, |n| n.get());
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .max_blocking_threads((cores / 2).max(1))
            .thread_name("tile-fetch")
            .enable_all()
            .build()
            .expect("a tokio runtime");

        let client = reqwest::Client::builder()
            .user_agent(USER_AGENT)
            .timeout(Duration::from_secs(20))
            .build()
            .expect("a valid HTTP client");

        let cache_dir = std::env::var_os("XDG_CACHE_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|home| PathBuf::from(home).join(".cache")))
            .map(|dir| Arc::new(dir.join("bevy-worldmap")));
        log::info!("Caching tiles in {cache_dir:?}");

        let per_source = [Source::Satellite, Source::OpenStreetMap, Source::Terrain]
            .into_iter()
            .map(|source| (source, Arc::new(Semaphore::new(source.max_in_flight()))))
            .collect();

        let (sender, receiver) = crossbeam_channel::unbounded();
        Self {
            runtime,
            client,
            cache_dir,
            global: Arc::new(Semaphore::new(MAX_IN_FLIGHT)),
            per_source,
            pending: HashMap::new(),
            backoff: HashMap::new(),
            downloading: Arc::new(AtomicUsize::new(0)),
            sender,
            receiver,
        }
    }

    /// Number of downloads currently in progress.
    pub fn downloading(&self) -> usize {
        self.downloading.load(Ordering::Relaxed)
    }

    /// Number of requests either downloading, reading from disk, or waiting for their turn.
    pub fn pending(&self) -> usize {
        self.pending.len()
    }

    /// Mark a request as still wanted, returning whether one is pending.
    pub fn touch(&mut self, key: Key, frame: u64) -> bool {
        match self.pending.get_mut(&key) {
            Some(pending) => {
                pending.wanted = frame;
                true
            }
            None => false,
        }
    }

    /// Whether a request for `key` can be started now. Only a few requests are queued per source,
    /// so that requests started later with a higher priority do not have to wait for long.
    pub fn can_start(&self, key: Key) -> bool {
        let (source, _) = key;
        let queued = self.pending.keys().filter(|(s, _)| *s == source).count();
        queued < source.max_in_flight() * 2
            && self
                .backoff
                .get(&key)
                .is_none_or(|(_, retry)| Instant::now() >= *retry)
    }

    pub fn start(&mut self, key: Key, frame: u64) {
        let job = Job {
            key,
            client: self.client.clone(),
            cache_dir: self.cache_dir.clone(),
            global: self.global.clone(),
            source: self.per_source[&key.0].clone(),
            downloading: self.downloading.clone(),
        };
        let sender = self.sender.clone();
        let task = self.runtime.spawn(async move {
            let outcome = job.run().await;
            let _ = sender.send(Fetched { key, outcome });
        });
        self.pending.insert(
            key,
            Pending {
                task,
                wanted: frame,
            },
        );
    }

    /// Abort requests which have not been wanted for a while.
    pub fn cancel_unwanted(&mut self, frame: u64) {
        self.pending.retain(|_, pending| {
            let keep = frame.saturating_sub(pending.wanted) <= CANCEL_FRAMES;
            if !keep {
                pending.task.abort();
            }
            keep
        });
    }

    /// Tiles which finished fetching since the last call.
    pub fn receive(&mut self) -> Vec<Fetched> {
        let fetched: Vec<_> = self.receiver.try_iter().collect();
        for Fetched { key, outcome } in &fetched {
            self.pending.remove(key);
            match outcome {
                Outcome::Failed => {
                    let attempts = self.backoff.get(key).map_or(0, |(n, _)| *n) + 1;
                    let delay = Duration::from_secs(1 << attempts.min(6));
                    log::warn!("Failed to fetch {key:?}, retrying in {delay:?}");
                    self.backoff
                        .insert(*key, (attempts, Instant::now() + delay));
                }
                _ => {
                    self.backoff.remove(key);
                }
            }
        }
        fetched
    }
}

struct Job {
    key: Key,
    client: reqwest::Client,
    cache_dir: Option<Arc<PathBuf>>,
    global: Arc<Semaphore>,
    source: Arc<Semaphore>,
    downloading: Arc<AtomicUsize>,
}

/// Decrements the download counter when the download ends, also when it is aborted.
struct Downloading(Arc<AtomicUsize>);

impl Drop for Downloading {
    fn drop(&mut self) {
        self.0.fetch_sub(1, Ordering::Relaxed);
    }
}

impl Job {
    async fn run(self) -> Outcome {
        let (source, tile) = self.key;
        let path = self.cache_dir.as_ref().map(|dir| {
            dir.join(source.cache_name())
                .join(tile.zoom.to_string())
                .join(tile.x.to_string())
                .join(tile.y.to_string())
        });

        // An empty file marks a tile the source does not have
        if let Some(path) = &path
            && let Ok(meta) = tokio::fs::metadata(path).await
            && meta
                .modified()
                .ok()
                .and_then(|modified| SystemTime::now().duration_since(modified).ok())
                .is_some_and(|age| age < CACHE_MAX_AGE)
            && let Ok(bytes) = tokio::fs::read(path).await
        {
            return match bytes.is_empty() {
                true => Outcome::Missing,
                false => decode(source, bytes).await,
            };
        }

        let bytes = match self.download().await {
            Ok(Some(bytes)) => bytes,
            Ok(None) => {
                if let Some(path) = &path {
                    write_cache(path, &[]).await;
                }
                return Outcome::Missing;
            }
            Err(err) => {
                log::debug!("Unable to fetch {:?}: {err}", self.key);
                return Outcome::Failed;
            }
        };

        if let Some(path) = &path {
            write_cache(path, &bytes).await;
        }
        decode(source, bytes).await
    }

    /// Download the tile, or `None` if the source does not have it.
    async fn download(&self) -> Result<Option<Vec<u8>>, reqwest::Error> {
        let (source, tile) = self.key;
        let _global = self
            .global
            .acquire()
            .await
            .expect("semaphore is never closed");
        let _source = self
            .source
            .acquire()
            .await
            .expect("semaphore is never closed");
        self.downloading.fetch_add(1, Ordering::Relaxed);
        let _downloading = Downloading(self.downloading.clone());

        let response = self.client.get(source.tile_url(tile)).send().await?;
        if response.status() == reqwest::StatusCode::NOT_FOUND {
            return Ok(None);
        }
        let bytes = response.error_for_status()?.bytes().await?;
        Ok(Some(bytes.to_vec()))
    }
}

/// Write to a temporary file first, so that an aborted task never leaves a partial tile behind.
async fn write_cache(path: &std::path::Path, bytes: &[u8]) {
    let Some(dir) = path.parent() else { return };
    let tmp = path.with_extension("tmp");
    let result = async {
        tokio::fs::create_dir_all(dir).await?;
        tokio::fs::write(&tmp, bytes).await?;
        tokio::fs::rename(&tmp, path).await
    }
    .await;
    if let Err(err) = result {
        log::warn!("Unable to cache tile at {path:?}: {err}");
    }
}

async fn decode(source: Source, bytes: Vec<u8>) -> Outcome {
    let decoded = tokio::task::spawn_blocking(move || {
        let image = image::load_from_memory(&bytes).ok()?;
        Some(match source {
            Source::Terrain => match decode_terrarium(image.into_rgb8()) {
                Some(grid) => Outcome::Loaded(Payload::Elevation(grid)),
                None => Outcome::Missing,
            },
            _ => Outcome::Loaded(Payload::Imagery(build_mipmaps(image.into_rgba8()))),
        })
    })
    .await;

    match decoded {
        Ok(Some(outcome)) => outcome,
        _ => {
            log::warn!("Unable to decode a {} tile", source.name());
            Outcome::Failed
        }
    }
}

/// Terrarium tiles encode the height as `(red * 256 + green + blue / 256) - 32768` meters.
/// <https://github.com/tilezen/joerd/blob/master/docs/formats.md#terrarium>
///
/// The tiles include the depth of the oceans, which is clamped to sea level, as the imagery
/// shows the surface of the water.
///
/// Black pixels, which would be 32768 m below sea level, are where the source has no data. At
/// its finest zoom levels, it has whole tiles of them in places, such as over the mountains of
/// Sardinia, which are `None`, as if the source did not have the tile.
fn decode_terrarium(image: image::RgbImage) -> Option<HeightGrid> {
    let size = image.width().min(image.height()) as usize;
    let mut heights = Vec::with_capacity(size * size);
    for y in 0..size as u32 {
        for x in 0..size as u32 {
            let [r, g, b] = image.get_pixel(x, y).0;
            heights.push(match [r, g, b] {
                [0, 0, 0] => f32::NAN,
                _ => (r as f32 * 256.0 + g as f32 + b as f32 / 256.0 - 32768.0).max(0.0),
            });
        }
    }
    (!heights.iter().all(|h| h.is_nan())).then_some(HeightGrid { size, heights })
}

/// Build the full chain of mipmaps, averaging in linear color space. This is needed for
/// tiles to be sampled without aliasing when seen from afar, or at grazing angles.
fn build_mipmaps(image: image::RgbaImage) -> Imagery {
    let to_linear: Vec<f32> = (0..256)
        .map(|v| {
            let c = v as f32 / 255.0;
            match c <= 0.04045 {
                true => c / 12.92,
                false => ((c + 0.055) / 1.055).powf(2.4),
            }
        })
        .collect();
    let to_srgb = |c: f32| {
        let c = match c <= 0.003_130_8 {
            true => c * 12.92,
            false => 1.055 * c.powf(1.0 / 2.4) - 0.055,
        };
        (c * 255.0 + 0.5).clamp(0.0, 255.0) as u8
    };

    let (width, height) = image.dimensions();
    let mut data = image.into_raw();
    let (mut w, mut h, mut offset) = (width as usize, height as usize, 0);
    let mut mip_levels = 1;

    while w > 1 || h > 1 {
        let (nw, nh) = ((w / 2).max(1), (h / 2).max(1));
        let mut next = Vec::with_capacity(nw * nh * 4);
        for y in 0..nh {
            for x in 0..nw {
                let (x0, x1) = (x * 2, (x * 2 + 1).min(w - 1));
                let (y0, y1) = (y * 2, (y * 2 + 1).min(h - 1));
                let px = |x: usize, y: usize, c: usize| data[offset + (y * w + x) * 4 + c];
                for c in 0..3 {
                    let sum = to_linear[px(x0, y0, c) as usize]
                        + to_linear[px(x1, y0, c) as usize]
                        + to_linear[px(x0, y1, c) as usize]
                        + to_linear[px(x1, y1, c) as usize];
                    next.push(to_srgb(sum / 4.0));
                }
                let alpha = px(x0, y0, 3) as u32
                    + px(x1, y0, 3) as u32
                    + px(x0, y1, 3) as u32
                    + px(x1, y1, 3) as u32;
                next.push((alpha / 4) as u8);
            }
        }
        offset = data.len();
        data.extend_from_slice(&next);
        (w, h) = (nw, nh);
        mip_levels += 1;
    }

    Imagery {
        width,
        height,
        mip_levels,
        data,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mipmap_chain() {
        let image = image::RgbaImage::from_pixel(256, 256, image::Rgba([255, 0, 0, 255]));
        let imagery = build_mipmaps(image);
        assert_eq!(imagery.mip_levels, 9);
        let expected: usize = (0..9).map(|l| (256 >> l) * (256 >> l) * 4).sum();
        assert_eq!(imagery.data.len(), expected);
        assert_eq!(&imagery.data[imagery.data.len() - 4..], &[255, 0, 0, 255]);
    }

    #[test]
    fn terrarium_gaps() {
        // 1000 m, and no data
        let mut image = image::RgbImage::from_pixel(4, 4, image::Rgb([131, 232, 0]));
        image.put_pixel(1, 2, image::Rgb([0, 0, 0]));
        let grid = decode_terrarium(image).unwrap();
        assert_eq!(grid.heights[0], 1000.0);
        assert!(grid.heights[2 * 4 + 1].is_nan());

        let empty = image::RgbImage::from_pixel(4, 4, image::Rgb([0, 0, 0]));
        assert!(decode_terrarium(empty).is_none());
    }

    #[test]
    fn height_sampling() {
        let grid = HeightGrid {
            size: 2,
            heights: vec![0.0, 10.0, 20.0, 30.0],
        };
        assert_eq!(grid.sample(DVec2::new(0.0, 0.0)), 0.0);
        assert_eq!(grid.sample(DVec2::new(0.5, 0.5)), 15.0);
        assert_eq!(grid.sample(DVec2::new(1.0, 1.0)), 30.0);
    }
}
