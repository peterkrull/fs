//! The HTTP tile sources. Make sure to follow the terms of use of each source.

use crate::geo::TileId;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Source {
    /// <https://www.arcgis.com/home/item.html?id=10df2279f9684e4a9f6a7f08febac2a9>
    Satellite,
    /// <https://operations.osmfoundation.org/policies/tiles/>
    OpenStreetMap,
    /// Terrarium encoded elevation tiles from the AWS Open Data registry.
    /// <https://registry.opendata.aws/terrain-tiles/>
    Terrain,
}

impl Source {
    pub const IMAGERY: [Source; 2] = [Source::Satellite, Source::OpenStreetMap];

    /// Sources respond with 404 for tiles they do not have, after which the terrain makes do
    /// with the imagery of an ancestor. Without `blankTile=false`, ArcGIS would instead respond
    /// with a "Map data not yet available" tile where its imagery is not this detailed.
    pub fn tile_url(self, tile: TileId) -> String {
        let TileId { zoom, x, y } = tile;
        match self {
            Source::Satellite => format!(
                "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{zoom}/{y}/{x}?blankTile=false"
            ),
            Source::OpenStreetMap => format!("https://tile.openstreetmap.org/{zoom}/{x}/{y}.png"),
            Source::Terrain => format!(
                "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{zoom}/{x}/{y}.png"
            ),
        }
    }

    pub fn max_zoom(self) -> u8 {
        match self {
            Source::Satellite => 19,
            Source::OpenStreetMap => 19,
            Source::Terrain => 15,
        }
    }

    /// The maximum number of simultaneous downloads from this source. The tile usage policy
    /// of OpenStreetMap asks to keep it low, as it is run on donated servers.
    pub fn max_in_flight(self) -> usize {
        match self {
            Source::Satellite => 4,
            Source::OpenStreetMap => 2,
            Source::Terrain => 4,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Source::Satellite => "Satellite",
            Source::OpenStreetMap => "OpenStreetMap",
            Source::Terrain => "Terrain",
        }
    }

    /// Directory name within the disk cache.
    pub fn cache_name(self) -> &'static str {
        match self {
            // Earlier versions cached the "Map data not yet available" tiles
            Source::Satellite => "arcgis-world-imagery-v2",
            Source::OpenStreetMap => "openstreetmap",
            Source::Terrain => "terrarium",
        }
    }

    pub fn attribution(self) -> &'static str {
        match self {
            Source::Satellite => {
                "Imagery: Esri, Maxar, Earthstar Geographics, and the GIS User Community"
            }
            Source::OpenStreetMap => "Map data: © OpenStreetMap contributors",
            Source::Terrain => "Elevation: Mapzen Terrain Tiles, AWS Open Data",
        }
    }
}
