//! Generation of the terrain meshes.
//!
//! Each tile is a regular grid in Web Mercator space, so that its texture coordinates map
//! linearly onto the imagery tiles. Vertices are positioned relative to the center of the
//! tile in `f64` before being converted to `f32`, which keeps them precise at any zoom level.
//!
//! Neighbouring tiles may differ in zoom level, and in the elevation data their heights are
//! sampled from, so their edges do not line up exactly. Where a neighbour is one level coarser,
//! the terrain moves the edge onto the surface of the parent, which is that of the neighbour.
//! To never leave a gap otherwise, every edge has a skirt: a strip hanging down from the edge,
//! down to the lowest a neighbour up to a few levels coarser could be there, which in steep
//! terrain can be far below.
//!
//! So that a tile can fade in from its parent, and out into it again, every vertex also
//! carries where it would be on the mesh of the parent, see [`ATTRIBUTE_PARENT_POSITION`].

use std::sync::Arc;

use bevy::{
    asset::RenderAssetUsages,
    math::{DVec2, DVec3},
    mesh::{Indices, MeshVertexAttribute, PrimitiveTopology, VertexFormat},
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

/// Where the vertex would be on the mesh of the parent tile. The grid of the parent has a
/// vertex at every other vertex of its children, and the vertices in between lie on its edges
/// and diagonals.
pub const ATTRIBUTE_PARENT_POSITION: MeshVertexAttribute = MeshVertexAttribute::new(
    "ParentPosition",
    0x7e44_a1d0_0000_0001,
    VertexFormat::Float32x3,
);
/// The normal of the parent tile at [`ATTRIBUTE_PARENT_POSITION`].
pub const ATTRIBUTE_PARENT_NORMAL: MeshVertexAttribute = MeshVertexAttribute::new(
    "ParentNormal",
    0x7e44_a1d0_0000_0002,
    VertexFormat::Float32x3,
);

pub struct TileMesh {
    pub mesh: Mesh,
    /// The point the vertex positions are relative to.
    pub center: DVec3,
    pub min_height: f32,
    pub max_height: f32,
    /// The elevation tile the heights were sampled from, and how many of its neighbours.
    pub elevation: Option<TileId>,
    pub neighbours: usize,
}

/// Elevation data for a tile: the grid covering it, and those around it where available.
///
/// Each grid covers its tile with pixels, whose centers are half a pixel in from its edges.
/// Within that half pixel, heights are interpolated with the pixels of the neighbouring grid,
/// so that tiles on either side of the edge between two grids agree on their heights there.
/// Clamping to the edge pixels instead would leave a step of a whole pixel between them, which
/// on a cliff is tens of meters. Without a neighbour, the height is extrapolated from the last
/// two pixels, which both sides agree on where the terrain is evenly sloped.
#[derive(Clone)]
pub struct Elevation {
    pub id: TileId,
    /// The grids of the tile and its neighbours, at `(dx + 1) + 3 * (dy + 1)`.
    grids: [Option<Arc<HeightGrid>>; 9],
}

impl Elevation {
    pub fn new(
        id: TileId,
        grid: Arc<HeightGrid>,
        mut neighbour: impl FnMut(TileId) -> Option<Arc<HeightGrid>>,
    ) -> Self {
        let mut grids: [Option<Arc<HeightGrid>>; 9] = Default::default();
        for (index, neighbour_id) in Self::neighbour_ids(id) {
            grids[index] = neighbour(neighbour_id).filter(|other| other.size == grid.size);
        }
        grids[4] = Some(grid);
        Self { id, grids }
    }

    /// The tiles around a tile, by their index into the grids, wrapping around in longitude.
    pub fn neighbour_ids(id: TileId) -> impl Iterator<Item = (usize, TileId)> {
        let count = geo::tiles_at(id.zoom) as i64;
        (0..9).filter(|index| *index != 4).filter_map(move |index| {
            let (dx, dy) = (index as i64 % 3 - 1, index as i64 / 3 - 1);
            let y = id.y as i64 + dy;
            (0..count).contains(&y).then(|| {
                let x = (id.x as i64 + dx).rem_euclid(count);
                (index, TileId::new(id.zoom, x as u32, y as u32))
            })
        })
    }

    /// How many neighbours there are grids of.
    pub fn neighbours(&self) -> usize {
        self.grids.iter().flatten().count() - 1
    }

    /// Bilinearly interpolated height at a point in unit mercator coordinates.
    pub fn sample(&self, unit: DVec2) -> f32 {
        let center = self.grids[4].as_ref().expect("the grid of the tile");
        let size = center.size as i64;
        let local = self.id.local(unit);
        let px = (local.x * size as f64 - 0.5).clamp(-0.5, size as f64 - 0.5);
        let py = (local.y * size as f64 - 0.5).clamp(-0.5, size as f64 - 0.5);
        let (x0, y0) = (px.floor() as i64, py.floor() as i64);
        let (fx, fy) = ((px - x0 as f64) as f32, (py - y0 as f64) as f32);
        let own = |x: i64, y: i64| center.heights[(y * size + x) as usize];
        // Pixels up to one beyond the edges of the grid
        let at = |x: i64, y: i64| {
            let (cx, cy) = (x.clamp(0, size - 1), y.clamp(0, size - 1));
            let (dx, dy) = (x - cx, y - cy);
            if dx == 0 && dy == 0 {
                return own(x, y);
            }
            match &self.grids[((dx + 1) + 3 * (dy + 1)) as usize] {
                Some(grid) => grid.heights[((y - dy * size) * size + (x - dx * size)) as usize],
                None => 2.0 * own(cx, cy) - own(cx - dx, cy - dy),
            }
        };
        let top = at(x0, y0) + (at(x0 + 1, y0) - at(x0, y0)) * fx;
        let bottom = at(x0, y0 + 1) + (at(x0 + 1, y0 + 1) - at(x0, y0 + 1)) * fx;
        top + (bottom - top) * fy
    }
}

/// The elevation tile from which the heights of `tile` are best sampled.
pub fn elevation_tile(tile: TileId, max_zoom: u8) -> TileId {
    tile.ancestor(
        tile.zoom
            .saturating_sub(ELEVATION_ZOOM_OFFSET)
            .min(max_zoom),
    )
}

/// The mesh of a tile, with heights from `elevation`, and those of its parent from
/// `parent_elevation`, which is what the parent is drawn with.
pub fn build_tile_mesh(
    tile: TileId,
    elevation: Option<Elevation>,
    parent_elevation: Option<Elevation>,
) -> TileMesh {
    let n = GRID + 1;
    let mut lon_lats = Vec::with_capacity(n * n);
    let mut heights = Vec::with_capacity(n * n);
    let mut parent_heights = Vec::with_capacity(n * n);
    let mut uvs = Vec::with_capacity(n * n + GRID * 4);

    let sample = |elevation: &Option<Elevation>, unit: DVec2| {
        elevation
            .as_ref()
            .map_or(0.0, |elevation| elevation.sample(unit))
    };
    for j in 0..n {
        for i in 0..n {
            let (u, v) = (i as f64 / GRID as f64, j as f64 / GRID as f64);
            let unit = tile.unit(u, v);
            let height = sample(&elevation, unit);
            lon_lats.push(geo::unit_to_lon_lat(unit));
            heights.push(height);
            parent_heights.push(match parent_elevation {
                Some(_) => sample(&parent_elevation, unit),
                None => height,
            });
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
    let mut normals: Vec<[f32; 3]> = grid_normals(&ecef, 1)
        .iter()
        .map(|v| v.as_vec3().to_array())
        .collect();

    // The parent has the vertices with even indices. Its triangles are split along the
    // diagonal from their north-east to their south-west corner, as below.
    let parent_vertices: Vec<DVec3> = lon_lats
        .iter()
        .zip(&parent_heights)
        .map(|(lon_lat, h)| geo::to_ecef(*lon_lat, *h as f64))
        .collect();
    let parent_ecef = on_parent(&parent_vertices);
    let mut parent_normals: Vec<[f32; 3]> = on_parent(&grid_normals(&parent_vertices, 2))
        .iter()
        .map(|v| v.normalize_or(DVec3::Z).as_vec3().to_array())
        .collect();

    let relative = |p: &DVec3| (*p - center).as_vec3().to_array();
    let mut positions: Vec<[f32; 3]> = ecef.iter().map(relative).collect();
    let mut parent_positions: Vec<[f32; 3]> = parent_ecef.iter().map(relative).collect();

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

    let depths = skirt_depths(tile, &heights, &parent_heights);
    let skirt_start = positions.len();
    for &index in &boundary[..boundary.len() - 1] {
        let depth = depths[index];
        let bottom = ecef[index] - ecef[index].normalize() * depth;
        positions.push(relative(&bottom));
        let parent_bottom = parent_ecef[index] - parent_ecef[index].normalize() * depth;
        parent_positions.push(relative(&parent_bottom));
        normals.push(normals[index]);
        parent_normals.push(parent_normals[index]);
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
    .with_inserted_attribute(ATTRIBUTE_PARENT_POSITION, parent_positions)
    .with_inserted_attribute(ATTRIBUTE_PARENT_NORMAL, parent_normals)
    .with_inserted_indices(Indices::U16(indices));

    TileMesh {
        mesh,
        center,
        min_height,
        max_height,
        neighbours: elevation.as_ref().map_or(0, Elevation::neighbours),
        elevation: elevation.map(|elevation| elevation.id),
    }
}

/// Normals of a square grid of vertices from central differences between vertices `step`
/// apart, one-sided at the edges.
fn grid_normals(grid: &[DVec3], step: usize) -> Vec<DVec3> {
    let n = GRID + 1;
    let at = |i: usize, j: usize| grid[j.min(GRID) * n + i.min(GRID)];
    let mut normals = Vec::with_capacity(n * n);
    for j in 0..n {
        for i in 0..n {
            let east = at(i + step, j) - at(i.saturating_sub(step), j);
            let south = at(i, j + step) - at(i, j.saturating_sub(step));
            let normal = south
                .cross(east)
                .try_normalize()
                .unwrap_or(grid[j * n + i].normalize());
            normals.push(normal);
        }
    }
    normals
}

/// Values on a square grid as the grid of the parent tile interpolates them: its vertices
/// are those with even indices, and its triangles are split from north-east to south-west.
fn on_parent(grid: &[DVec3]) -> Vec<DVec3> {
    let n = GRID + 1;
    let at = |i: usize, j: usize| grid[j * n + i];
    let mut parent = Vec::with_capacity(n * n);
    for j in 0..n {
        for i in 0..n {
            parent.push(match (i % 2, j % 2) {
                (0, 0) => at(i, j),
                (1, 0) => (at(i - 1, j) + at(i + 1, j)) / 2.0,
                (0, _) => (at(i, j - 1) + at(i, j + 1)) / 2.0,
                _ => (at(i + 1, j - 1) + at(i - 1, j + 1)) / 2.0,
            });
        }
    }
    parent
}

/// How deep the skirt hangs below each vertex on the edge of a tile, by index into the grid.
///
/// A coarser neighbour has a vertex at every second, fourth or eighth vertex of the edge, and
/// is straight in between, with heights from this or coarser elevation data. A finer neighbour
/// follows the terrain between the vertices, which is about as rough as the change from one
/// vertex to the next. The skirt reaches below all of these, and some more for the difference
/// between neighbouring elevation tiles, which grows with the size of a tile.
fn skirt_depths(tile: TileId, heights: &[f32], parent_heights: &[f32]) -> Vec<f64> {
    const COARSER_LEVELS: u32 = 3;
    let n = GRID + 1;
    let base = tile.width_meters() * 0.05 + 10.0;
    let edges: [Vec<usize>; 4] = [
        (0..n).collect(),
        (0..n).map(|i| GRID * n + i).collect(),
        (0..n).map(|j| j * n).collect(),
        (0..n).map(|j| j * n + GRID).collect(),
    ];
    let mut depths = vec![base; n * n];
    for edge in &edges {
        let along = |heights: &[f32], m: usize| heights[edge[m]] as f64;
        for (m, &index) in edge.iter().enumerate() {
            let mut lowest = along(heights, m);
            for level in 0..=COARSER_LEVELS {
                let step = 1 << level;
                let (lower, t) = (m / step * step, (m % step) as f64 / step as f64);
                let upper = (lower + step).min(GRID);
                for heights in [heights, parent_heights] {
                    let (a, b) = (along(heights, lower), along(heights, upper));
                    lowest = lowest.min(a + (b - a) * t);
                }
            }
            let rough = [m.saturating_sub(1), (m + 1).min(GRID)]
                .map(|k| (along(heights, k) - along(heights, m)).abs())
                .into_iter()
                .fold(0.0, f64::max);
            let below = along(heights, m) - lowest + rough;
            depths[index] = depths[index].max(below + base);
        }
    }
    depths
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
    // The caps never fade, and are their own parents
    .with_inserted_attribute(ATTRIBUTE_PARENT_POSITION, positions.clone())
    .with_inserted_attribute(ATTRIBUTE_PARENT_NORMAL, normals.clone())
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    .with_inserted_indices(Indices::U16(indices));
    (mesh, center)
}

fn segment_lon(s: usize, segments: usize) -> f64 {
    s as f64 / segments as f64 * std::f64::consts::TAU - std::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;

    fn positions(mesh: &Mesh, attribute: MeshVertexAttribute, center: DVec3) -> Vec<DVec3> {
        match mesh.attribute(attribute) {
            Some(bevy::mesh::VertexAttributeValues::Float32x3(values)) => values
                .iter()
                .map(|v| Vec3::from(*v).as_dvec3() + center)
                .collect(),
            _ => panic!("missing {}", attribute.name),
        }
    }

    /// Tiles on either side of the edge between two elevation grids agree on the heights along
    /// it, with or without the grid of the other side.
    #[test]
    fn heights_are_seamless_across_grids() {
        let size = 256;
        // Sloping steeply to the east, and curving, so extrapolation is not exact
        let terrain = |x: f64, y: f64| (40.0 * x + 0.05 * x * x + 3.0 * y) as f32;
        let grid = |offset: f64| {
            let heights = (0..size * size)
                .map(|i| terrain((i % size) as f64 + 0.5 + offset, (i / size) as f64 + 0.5))
                .collect();
            Arc::new(HeightGrid { size, heights })
        };
        let (west_id, east_id) = (TileId::new(10, 500, 300), TileId::new(10, 501, 300));
        let (west, east) = (grid(0.0), grid(size as f64));
        let with = |id: TileId, own: &Arc<HeightGrid>, other: TileId, theirs: &Arc<HeightGrid>| {
            Elevation::new(id, own.clone(), |n| (n == other).then(|| theirs.clone()))
        };
        let connected = [
            with(west_id, &west, east_id, &east),
            with(east_id, &east, west_id, &west),
        ];
        let alone = [
            Elevation::new(west_id, west.clone(), |_| None),
            Elevation::new(east_id, east.clone(), |_| None),
        ];
        for v in [0.1, 0.5, 0.9] {
            let edge = east_id.unit(0.0, v);
            let [a, b] = connected.each_ref().map(|e| e.sample(edge));
            assert!((a - b).abs() < 1e-3, "connected: {a} vs {b}");
            assert!((a - terrain(256.0, v * 256.0)).abs() < 0.05);
            let [a, b] = alone.each_ref().map(|e| e.sample(edge));
            // Clamping to the edge pixels would leave a step of a pixel, about 66 m here
            assert!((a - b).abs() < 0.2, "alone: {a} vs {b}");
        }
    }

    /// Where a child says it would be on its parent, the mesh of the parent is.
    #[test]
    fn children_know_where_their_parent_is() {
        // Rugged terrain, with the same elevation data for the parent and its children
        let size = 64;
        let heights = (0..size * size)
            .map(|i| ((i * 7919) % 1000) as f32)
            .collect();
        let grid = Arc::new(HeightGrid { size, heights });
        let parent = TileId::new(12, 2130, 1440);
        let elevation = Some(Elevation::new(parent.ancestor(9), grid, |_| None));

        let built = build_tile_mesh(parent, elevation.clone(), None);
        let on_parent = positions(&built.mesh, Mesh::ATTRIBUTE_POSITION, built.center);
        let indices: Vec<usize> = built.mesh.indices().unwrap().iter().collect();
        let n = GRID + 1;
        // The diagonal of each cell of the parent is the edge its two triangles share
        let (nw, ne, sw) = (0, 1, n);
        assert!(indices[..6].contains(&ne) && indices[..6].contains(&sw));
        assert_eq!(indices[..6].iter().filter(|i| **i == nw).count(), 1);

        for (index, child) in parent.children().into_iter().enumerate() {
            let built = build_tile_mesh(child, elevation.clone(), elevation.clone());
            let expected = positions(&built.mesh, ATTRIBUTE_PARENT_POSITION, built.center);
            let (ox, oy) = ((index % 2) * GRID / 2, (index / 2) * GRID / 2);
            for j in 0..n {
                for i in 0..n {
                    // The vertex of the parent at or before the child vertex, and the one after
                    let (pi, pj) = (ox + i / 2, oy + j / 2);
                    let at = |i: usize, j: usize| on_parent[j * n + i];
                    let parent_position = match (i % 2, j % 2) {
                        (0, 0) => at(pi, pj),
                        (1, 0) => (at(pi, pj) + at(pi + 1, pj)) / 2.0,
                        (0, _) => (at(pi, pj) + at(pi, pj + 1)) / 2.0,
                        _ => (at(pi + 1, pj) + at(pi, pj + 1)) / 2.0,
                    };
                    let error = (expected[j * n + i] - parent_position).length();
                    assert!(error < 0.05, "child {index} at {i}, {j}: {error} m off");
                }
            }
        }
    }
}
