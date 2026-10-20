//! Materials of the terrain and the sky, which share the atmosphere model in `atmosphere.wgsl`.

use bevy::{
    asset::embedded_asset,
    mesh::{MeshVertexBufferLayoutRef, VertexBufferLayout},
    pbr::{MaterialPipeline, MaterialPipelineKey},
    prelude::*,
    render::render_resource::{
        AsBindGroup, RenderPipelineDescriptor, ShaderType, SpecializedMeshPipelineError,
    },
    shader::{ShaderRef, load_shader_library},
};

use crate::mesh::{ATTRIBUTE_PARENT_NORMAL, ATTRIBUTE_PARENT_POSITION};

pub struct MaterialsPlugin;

impl Plugin for MaterialsPlugin {
    fn build(&self, app: &mut App) {
        load_shader_library!(app, "shaders/atmosphere.wgsl");
        embedded_asset!(app, "shaders/terrain.wgsl");
        embedded_asset!(app, "shaders/sky.wgsl");
        app.add_plugins((
            MaterialPlugin::<TerrainMaterial>::default(),
            MaterialPlugin::<SkyMaterial>::default(),
        ));
    }
}

#[derive(Clone, Copy, Debug, ShaderType)]
pub struct TerrainParams {
    /// Offset in `xy` and scale in `z` from the texture coordinates of the mesh to those of the
    /// imagery, which may belong to an ancestor of the tile.
    pub uv_transform: Vec4,
    /// The same for the previous imagery.
    pub previous_uv_transform: Vec4,
    /// The center of the planet relative to the mesh, in `xyz`.
    pub planet_center: Vec4,
    /// Multiplied with the imagery, or used in its place when there is none.
    pub base_color: Vec4,
    pub previous_color: Vec4,
    /// Direction towards the sun in `xyz`. The lights of the view are not available to vertex
    /// shaders, where the atmosphere is evaluated.
    pub sun: Vec4,
    /// In `x`, how far the geometry has faded in from that of the parent tile, and in `y`, the
    /// imagery from the previous imagery, both from `0` to `1`.
    pub fade: Vec4,
    /// Which edges are moved onto the parent, where they border a coarser tile: those at
    /// `u = 0`, `u = 1`, `v = 0` and `v = 1`, as `1`.
    pub edges: Vec4,
}

impl TerrainParams {
    pub fn new(planet_center: Vec3, base_color: Color, sun: Vec3) -> Self {
        Self {
            uv_transform: Vec4::new(0.0, 0.0, 1.0, 0.0),
            previous_uv_transform: Vec4::new(0.0, 0.0, 1.0, 0.0),
            planet_center: planet_center.extend(0.0),
            base_color: base_color.to_linear().to_vec4(),
            previous_color: base_color.to_linear().to_vec4(),
            sun: sun.extend(0.0),
            fade: Vec4::ONE,
            edges: Vec4::ZERO,
        }
    }
}

/// The terrain, which fades from imagery to imagery, and from the geometry of the parent tile
/// to its own, with [`TerrainParams::fade`].
#[derive(Asset, TypePath, AsBindGroup, Clone, Debug)]
pub struct TerrainMaterial {
    #[uniform(0)]
    pub params: TerrainParams,
    #[texture(1)]
    #[sampler(2)]
    pub imagery: Option<Handle<Image>>,
    #[texture(3)]
    #[sampler(4)]
    pub previous: Option<Handle<Image>>,
}

impl Material for TerrainMaterial {
    fn vertex_shader() -> ShaderRef {
        "embedded://bevy_worldmap/shaders/terrain.wgsl".into()
    }

    fn fragment_shader() -> ShaderRef {
        "embedded://bevy_worldmap/shaders/terrain.wgsl".into()
    }

    fn enable_prepass() -> bool {
        false
    }

    fn enable_shadows() -> bool {
        false
    }

    fn specialize(
        _pipeline: &MaterialPipeline,
        descriptor: &mut RenderPipelineDescriptor,
        layout: &MeshVertexBufferLayoutRef,
        _key: MaterialPipelineKey<Self>,
    ) -> Result<(), SpecializedMeshPipelineError> {
        let vertex: VertexBufferLayout = layout.0.get_layout(&[
            Mesh::ATTRIBUTE_POSITION.at_shader_location(0),
            Mesh::ATTRIBUTE_NORMAL.at_shader_location(1),
            Mesh::ATTRIBUTE_UV_0.at_shader_location(2),
            ATTRIBUTE_PARENT_POSITION.at_shader_location(3),
            ATTRIBUTE_PARENT_NORMAL.at_shader_location(4),
        ])?;
        descriptor.vertex.buffers = vec![vertex];
        Ok(())
    }
}

#[derive(Asset, TypePath, AsBindGroup, Clone, Debug)]
pub struct SkyMaterial {
    /// Position of the camera relative to the center of the planet, in `xyz`.
    #[uniform(0)]
    pub camera: Vec4,
}

impl Material for SkyMaterial {
    fn fragment_shader() -> ShaderRef {
        "embedded://bevy_worldmap/shaders/sky.wgsl".into()
    }

    fn enable_prepass() -> bool {
        false
    }

    fn enable_shadows() -> bool {
        false
    }

    /// The sky is a sphere around the camera, seen from the inside.
    fn specialize(
        _pipeline: &MaterialPipeline,
        descriptor: &mut RenderPipelineDescriptor,
        _layout: &MeshVertexBufferLayoutRef,
        _key: MaterialPipelineKey<Self>,
    ) -> Result<(), SpecializedMeshPipelineError> {
        descriptor.primitive.cull_mode = None;
        Ok(())
    }
}
