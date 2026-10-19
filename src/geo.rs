//! Geodesy on a spherical earth, and the Web Mercator tile grid draped over it.
//!
//! Positions are kept as earth-centered, earth-fixed (ECEF) coordinates in `f64`,
//! with `+Z` towards the north pole and `+X` towards longitude 0. They are only
//! converted to `f32` relative to the camera, see [`crate::camera`].

use bevy::math::{DVec2, DVec3};
use std::f64::consts::{PI, TAU};

/// Mean radius of the earth, which is modeled as a sphere.
pub const EARTH_RADIUS: f64 = 6_371_000.0;

/// Highest point on earth, used to bound tiles of which no height data is known yet.
pub const MAX_HEIGHT: f32 = 8_900.0;

/// The latitude at which the Web Mercator projection is cut off, in radians.
pub fn max_mercator_lat() -> f64 {
    PI.sinh().atan()
}

/// Identifies a tile in the Web Mercator tile grid.
/// See: <https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames>
#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy, PartialOrd, Ord)]
pub struct TileId {
    /// Zoom level, where 0 is a single tile covering the whole world.
    pub zoom: u8,
    pub x: u32,
    pub y: u32,
}

impl TileId {
    pub const ROOT: Self = TileId {
        zoom: 0,
        x: 0,
        y: 0,
    };

    pub fn new(zoom: u8, x: u32, y: u32) -> Self {
        let max = tiles_at(zoom) - 1;
        TileId {
            zoom,
            x: x.min(max),
            y: y.min(max),
        }
    }

    /// The tile containing `unit`, at the given zoom level.
    pub fn containing(unit: DVec2, zoom: u8) -> Self {
        let n = tiles_at(zoom) as f64;
        let x = (unit.x * n).floor().clamp(0.0, n - 1.0) as u32;
        let y = (unit.y * n).floor().clamp(0.0, n - 1.0) as u32;
        TileId { zoom, x, y }
    }

    pub fn parent(&self) -> Option<TileId> {
        Some(TileId {
            zoom: self.zoom.checked_sub(1)?,
            x: self.x / 2,
            y: self.y / 2,
        })
    }

    /// The ancestor of this tile at `zoom`, or the tile itself if it is not deeper than `zoom`.
    pub fn ancestor(&self, zoom: u8) -> TileId {
        if zoom >= self.zoom {
            return *self;
        }
        let shift = self.zoom - zoom;
        TileId {
            zoom,
            x: self.x >> shift,
            y: self.y >> shift,
        }
    }

    /// This tile, followed by all of its ancestors up to the root.
    pub fn lineage(self) -> impl Iterator<Item = TileId> {
        std::iter::successors(Some(self), TileId::parent)
    }

    /// The four tiles of the next zoom level covering this tile, in north-west,
    /// north-east, south-west, south-east order.
    pub fn children(&self) -> [TileId; 4] {
        let (zoom, x, y) = (self.zoom + 1, self.x * 2, self.y * 2);
        [
            TileId { zoom, x, y },
            TileId { zoom, x: x + 1, y },
            TileId { zoom, x, y: y + 1 },
            TileId {
                zoom,
                x: x + 1,
                y: y + 1,
            },
        ]
    }

    /// The position of `(u, v)` within the tile in unit mercator coordinates, where
    /// `(0, 0)` is the north-west corner of the world, and `(1, 1)` the south-east.
    pub fn unit(&self, u: f64, v: f64) -> DVec2 {
        let n = tiles_at(self.zoom) as f64;
        DVec2::new((self.x as f64 + u) / n, (self.y as f64 + v) / n)
    }

    /// The position of `unit` relative to this tile, where the tile spans `0..1` on each axis.
    pub fn local(&self, unit: DVec2) -> DVec2 {
        let n = tiles_at(self.zoom) as f64;
        DVec2::new(unit.x * n - self.x as f64, unit.y * n - self.y as f64)
    }

    /// The offset and scale that maps this tile's `0..1` texture coordinates
    /// into those of its `ancestor`.
    pub fn uv_in_ancestor(&self, ancestor: TileId) -> (DVec2, f64) {
        let scale = 1.0 / (1u64 << (self.zoom - ancestor.zoom)) as f64;
        (ancestor.local(self.unit(0.0, 0.0)), scale)
    }

    /// Longitude and latitude range of the tile in radians, as `(west, east, south, north)`.
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        let nw = unit_to_lon_lat(self.unit(0.0, 0.0));
        let se = unit_to_lon_lat(self.unit(1.0, 1.0));
        (nw.x, se.x, se.y, nw.y)
    }

    /// Width of the tile along its middle latitude in meters.
    pub fn width_meters(&self) -> f64 {
        let lat = unit_to_lon_lat(self.unit(0.5, 0.5)).y;
        TAU * EARTH_RADIUS * lat.cos() / tiles_at(self.zoom) as f64
    }
}

/// Number of tiles along each axis at a zoom level.
pub fn tiles_at(zoom: u8) -> u32 {
    1 << zoom
}

/// Convert unit mercator coordinates to longitude and latitude in radians.
pub fn unit_to_lon_lat(unit: DVec2) -> DVec2 {
    let lon = unit.x * TAU - PI;
    let lat = (PI * (1.0 - 2.0 * unit.y)).sinh().atan();
    DVec2::new(lon, lat)
}

/// Convert longitude and latitude in radians to unit mercator coordinates.
pub fn lon_lat_to_unit(lon_lat: DVec2) -> DVec2 {
    let lat = lon_lat.y.clamp(-max_mercator_lat(), max_mercator_lat());
    let x = (lon_lat.x + PI) / TAU;
    let y = (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / PI) / 2.0;
    DVec2::new(x.rem_euclid(1.0), y.clamp(0.0, 1.0))
}

/// The direction straight up from the surface, at a longitude and latitude in radians.
pub fn up(lon_lat: DVec2) -> DVec3 {
    let (sin_lon, cos_lon) = lon_lat.x.sin_cos();
    let (sin_lat, cos_lat) = lon_lat.y.sin_cos();
    DVec3::new(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat)
}

/// ECEF position of a point at a longitude and latitude in radians, and a height in meters.
pub fn to_ecef(lon_lat: DVec2, height: f64) -> DVec3 {
    up(lon_lat) * (EARTH_RADIUS + height)
}

/// Longitude and latitude in radians, and height in meters, of an ECEF position.
pub fn from_ecef(pos: DVec3) -> (DVec2, f64) {
    let r = pos.length();
    let lon = pos.y.atan2(pos.x);
    let lat = (pos.z / r).clamp(-1.0, 1.0).asin();
    (DVec2::new(lon, lat), r - EARTH_RADIUS)
}

/// The east, north and up directions at an ECEF position, which must not be at a pole.
pub fn enu(pos: DVec3) -> (DVec3, DVec3, DVec3) {
    let up = pos.normalize();
    let east = DVec3::Z.cross(up).normalize();
    let north = up.cross(east);
    (east, north, up)
}

/// Clamp `value` to the angular range `min..max`, taking wrap-around into account.
pub fn clamp_angle(value: f64, min: f64, max: f64) -> f64 {
    let center = (min + max) / 2.0;
    let half = (max - min) / 2.0;
    let delta = (value - center + PI).rem_euclid(TAU) - PI;
    center + delta.clamp(-half, half)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mercator_roundtrip() {
        let lon_lat = DVec2::new(0.3, -0.7);
        let back = unit_to_lon_lat(lon_lat_to_unit(lon_lat));
        assert!((back - lon_lat).length() < 1e-9);
    }

    #[test]
    fn ecef_roundtrip() {
        let lon_lat = DVec2::new(-2.1, 0.9);
        let (back, h) = from_ecef(to_ecef(lon_lat, 1234.5));
        assert!((back - lon_lat).length() < 1e-9);
        assert!((h - 1234.5).abs() < 1e-6);
    }

    #[test]
    fn uv_in_ancestor() {
        let tile = TileId::new(3, 5, 2);
        let (offset, scale) = tile.uv_in_ancestor(TileId::new(1, 1, 0));
        assert_eq!(scale, 0.25);
        assert!((offset - DVec2::new(0.25, 0.5)).length() < 1e-12);
    }

    #[test]
    fn clamp_wraps() {
        let v = clamp_angle(3.1, -3.0, -2.0);
        assert!((v - -3.0).abs() < 1e-12);
    }
}
