//! Generation of the terrain meshes.
//!
//! Each tile is a regular grid in Web Mercator space, so that its texture coordinates map
//! linearly onto the imagery tiles. Vertices are positioned relative to the center of the
//! tile in `f64` before being converted to `f32`, which keeps them precise at any zoom level.
//!
//! Neighbouring tiles may differ in zoom level, and in the elevation data their heights are
//! sampled from, so their edges do not line up exactly. To never leave a gap between them,
//! every edge has a skirt: a strip hanging down from the edge, deep enough to cover the
//! largest difference in height between neighbours.

use std::sync::Arc;

use bevy::{
    asset::RenderAssetUsages,
    math::{DVec2, DVec3},
    mesh::{Indices, PrimitiveTopology},
    prelude::*,
};

use crate::{
    fetch::HeightGrid,
    geo::{self, TileId},
};

/// Number of grid cells along each edge of a tile. With 256 pixel tiles, elevation data three
/// zoom levels coarser gives one height sample per vertex.
pub const GRID: usize = 32;

/// How many zoom levels coarser the elevation data of a tile is.
pub const ELEVATION_ZOOM_OFFSET: u8 = 3;

pub struct TileMesh {
    pub mesh: Mesh,
    /// The point the vertex positions are relative to.
    pub center: DVec3,
    pub min_height: f32,
    pub max_height: f32,
    /// The elevation tile the heights were sampled from.
    pub elevation: Option<TileId>,
}

/// The elevation tile from which the heights of `tile` are best sampled.
pub fn elevation_tile(tile: TileId, max_zoom: u8) -> TileId {
    tile.ancestor(
        tile.zoom
            .saturating_sub(ELEVATION_ZOOM_OFFSET)
            .min(max_zoom),
    )
}

pub fn build_tile_mesh(tile: TileId, elevation: Option<(TileId, Arc<HeightGrid>)>) -> TileMesh {
    let n = GRID + 1;
    let mut lon_lats = Vec::with_capacity(n * n);
    let mut heights = Vec::with_capacity(n * n);
    let mut uvs = Vec::with_capacity(n * n + GRID * 4);

    for j in 0..n {
        for i in 0..n {
            let (u, v) = (i as f64 / GRID as f64, j as f64 / GRID as f64);
            let unit = tile.unit(u, v);
            let height = elevation
                .as_ref()
                .map_or(0.0, |(id, grid)| grid.sample(id.local(unit)));
            lon_lats.push(geo::unit_to_lon_lat(unit));
            heights.push(height);
            uvs.push([u as f32, v as f32]);
        }
    }

    let min_height = heights.iter().copied().fold(f32::INFINITY, f32::min);
    let max_height = heights.iter().copied().fold(f32::NEG_INFINITY, f32::max);
    // Fixed for each tile, so that a rebuilt mesh can be swapped in without touching its
    // material or transform
    let center = geo::to_ecef(geo::unit_to_lon_lat(tile.unit(0.5, 0.5)), 0.0);

    let ecef: Vec<DVec3> = lon_lats
        .iter()
        .zip(&heights)
        .map(|(lon_lat, h)| geo::to_ecef(*lon_lat, *h as f64))
        .collect();

    // Normals from central differences, one-sided at the edges
    let at = |i: usize, j: usize| ecef[j.min(GRID) * n + i.min(GRID)];
    let mut normals = Vec::with_capacity(n * n);
    for j in 0..n {
        for i in 0..n {
            let east = at(i + 1, j) - at(i.saturating_sub(1), j);
            let south = at(i, j + 1) - at(i, j.saturating_sub(1));
            let normal = south
                .cross(east)
                .try_normalize()
                .unwrap_or(ecef[j * n + i].normalize());
            normals.push(normal.as_vec3().to_array());
        }
    }

    let mut positions: Vec<[f32; 3]> = ecef
        .iter()
        .map(|p| (*p - center).as_vec3().to_array())
        .collect();

    let mut indices: Vec<u16> = Vec::with_capacity(GRID * GRID * 6 + GRID * 4 * 12);
    for j in 0..GRID {
        for i in 0..GRID {
            let nw = (j * n + i) as u16;
            let (ne, sw, se) = (nw + 1, nw + n as u16, nw + n as u16 + 1);
            indices.extend_from_slice(&[nw, sw, ne, ne, sw, se]);
        }
    }

    // The boundary of the grid as a closed loop
    let mut boundary: Vec<usize> = Vec::with_capacity(GRID * 4 + 1);
    boundary.extend(0..GRID);
    boundary.extend((0..GRID).map(|j| j * n + GRID));
    boundary.extend((0..GRID).map(|i| GRID * n + GRID - i));
    boundary.extend((0..GRID).map(|j| (GRID - j) * n));
    boundary.push(0);

    let depth = skirt_depth(tile);
    let skirt_start = positions.len();
    for &index in &boundary[..boundary.len() - 1] {
        let bottom = ecef[index] - ecef[index].normalize() * depth;
        positions.push((bottom - center).as_vec3().to_array());
        normals.push(normals[index]);
        uvs.push(uvs[index]);
    }
    let count = boundary.len() - 1;
    for k in 0..count {
        let (a, b) = (boundary[k] as u16, boundary[k + 1] as u16);
        let (sa, sb) = (
            (skirt_start + k) as u16,
            (skirt_start + (k + 1) % count) as u16,
        );
        // Both windings, so that the skirt is visible from either side
        indices.extend_from_slice(&[a, sa, b, b, sa, sb, a, b, sa, b, sb, sa]);
    }

    let mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    .with_inserted_indices(Indices::U16(indices));

    TileMesh {
        mesh,
        center,
        min_height,
        max_height,
        elevation: elevation.map(|(id, _)| id),
    }
}

/// Deep enough to cover the difference between neighbours, which grows with the size of a tile.
fn skirt_depth(tile: TileId) -> f64 {
    tile.width_meters() * 0.05 + 10.0
}

/// The Web Mercator tiles stop at about 85° latitude, beyond which a cap of a single color
/// closes the globe. Its rim follows the heights of the tiles it borders.
pub fn build_cap_mesh(north: bool, rim_height: impl Fn(f64) -> f32) -> (Mesh, DVec3) {
    const SEGMENTS: usize = 128;
    const RINGS: usize = 8;

    let sign = if north { 1.0 } else { -1.0 };
    let rim_lat = geo::max_mercator_lat() * sign;
    let pole_lat = std::f64::consts::FRAC_PI_2 * sign;

    let rim: Vec<f32> = (0..SEGMENTS)
        .map(|s| rim_height(segment_lon(s, SEGMENTS)))
        .collect();
    let pole_height = rim.iter().sum::<f32>() / SEGMENTS as f32;
    let center = geo::to_ecef(DVec2::new(0.0, pole_lat), pole_height as f64);

    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut normals: Vec<[f32; 3]> = Vec::new();
    let mut push = |pos: DVec3| {
        positions.push((pos - center).as_vec3().to_array());
        normals.push(pos.normalize().as_vec3().to_array());
        positions.len() as u16 - 1
    };

    // Rings from the rim towards the pole, followed by the pole and a skirt below the rim
    for r in 0..RINGS {
        let t = r as f64 / RINGS as f64;
        let lat = rim_lat + (pole_lat - rim_lat) * t;
        for (s, rim_height) in rim.iter().enumerate() {
            let height = *rim_height as f64 * (1.0 - t) + pole_height as f64 * t;
            push(geo::to_ecef(
                DVec2::new(segment_lon(s, SEGMENTS), lat),
                height,
            ));
        }
    }
    let pole = push(center);
    let skirt = pole + 1;
    for (s, rim_height) in rim.iter().enumerate() {
        let height = *rim_height as f64 - 20_000.0;
        push(geo::to_ecef(
            DVec2::new(segment_lon(s, SEGMENTS), rim_lat),
            height,
        ));
    }

    let mut indices: Vec<u16> = Vec::new();
    let ring = |r: usize, s: usize| (r * SEGMENTS + s % SEGMENTS) as u16;
    let mut quad = |a: u16, b: u16, c: u16, d: u16| {
        // Both windings, as the order flips between the hemispheres
        indices.extend_from_slice(&[a, b, c, b, d, c, a, c, b, b, c, d]);
    };
    for r in 0..RINGS - 1 {
        for s in 0..SEGMENTS {
            quad(
                ring(r, s),
                ring(r, s + 1),
                ring(r + 1, s),
                ring(r + 1, s + 1),
            );
        }
    }
    for s in 0..SEGMENTS {
        let (a, b) = (ring(RINGS - 1, s), ring(RINGS - 1, s + 1));
        quad(a, b, pole, pole);
        let (sa, sb) = (skirt + s as u16, skirt + ((s + 1) % SEGMENTS) as u16);
        quad(ring(0, s), ring(0, s + 1), sa, sb);
    }

    let uvs = vec![[0.0f32, 0.0]; positions.len()];
    let mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    .with_inserted_indices(Indices::U16(indices));
    (mesh, center)
}

fn segment_lon(s: usize, segments: usize) -> f64 {
    s as f64 / segments as f64 * std::f64::consts::TAU - std::f64::consts::PI
}
