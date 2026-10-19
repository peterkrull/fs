//! Materials of the terrain and the sky, which share the atmosphere model in `atmosphere.wgsl`.

use bevy::{
    asset::embedded_asset,
    mesh::MeshVertexBufferLayoutRef,
    pbr::{MaterialPipeline, MaterialPipelineKey},
    prelude::*,
    render::render_resource::{
        AsBindGroup, RenderPipelineDescriptor, ShaderType, SpecializedMeshPipelineError,
    },
    shader::{ShaderRef, load_shader_library},
};

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
    /// The center of the planet relative to the mesh, in `xyz`.
    pub planet_center: Vec4,
    /// Multiplied with the imagery, or used in its place when there is none.
    pub base_color: Vec4,
    /// Direction towards the sun in `xyz`. The lights of the view are not available to vertex
    /// shaders, where the atmosphere is evaluated.
    pub sun: Vec4,
}

#[derive(Asset, TypePath, AsBindGroup, Clone, Debug)]
pub struct TerrainMaterial {
    #[uniform(0)]
    pub params: TerrainParams,
    #[texture(1)]
    #[sampler(2)]
    pub imagery: Option<Handle<Image>>,
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
