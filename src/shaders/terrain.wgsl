#import bevy_pbr::{
    mesh_functions,
    mesh_view_bindings::view,
    view_transformations::position_world_to_clip,
}
#import worldmap::atmosphere::{
    PI, SUN_INTENSITY, aerial_perspective, sun_transmittance, sky_irradiance,
}

struct TerrainParams {
    uv_transform: vec4<f32>,
    planet_center: vec4<f32>,
    base_color: vec4<f32>,
    sun: vec4<f32>,
}

@group(#{MATERIAL_BIND_GROUP}) @binding(0) var<uniform> params: TerrainParams;
@group(#{MATERIAL_BIND_GROUP}) @binding(1) var imagery: texture_2d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(2) var imagery_sampler: sampler;

struct Vertex {
    @builtin(instance_index) instance_index: u32,
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) planet_position: vec3<f32>,
    @location(3) sun_light: vec3<f32>,
    @location(4) inscattered: vec3<f32>,
    @location(5) transmittance: vec3<f32>,
}

// The atmosphere is evaluated per vertex, as it varies slowly over the surface
@vertex
fn vertex(vertex: Vertex) -> VertexOutput {
    let world_from_local = mesh_functions::get_world_from_local(vertex.instance_index);
    let world = mesh_functions::mesh_position_local_to_world(world_from_local, vec4<f32>(vertex.position, 1.0)).xyz;
    // Meshes are only ever translated
    let center = (world_from_local * vec4<f32>(params.planet_center.xyz, 1.0)).xyz;
    let sun = params.sun.xyz;

    var out: VertexOutput;
    out.clip_position = position_world_to_clip(world);
    out.uv = params.uv_transform.xy + vertex.uv * params.uv_transform.z;
    out.normal = vertex.normal;
    out.planet_position = world - center;
    out.sun_light = sun_transmittance(out.planet_position, sun);

    let scattering = aerial_perspective(view.world_position - center, world - view.world_position, sun);
    out.inscattered = scattering.inscattered;
    out.transmittance = scattering.transmittance;
    return out;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let sun = params.sun.xyz;
    let albedo = textureSample(imagery, imagery_sampler, in.uv).rgb * params.base_color.rgb;

    // The imagery already has shading baked in, so the relief is only shaded lightly
    let normal = normalize(in.normal);
    let up = normalize(in.planet_position);
    let diffuse = max(dot(normal, sun), 0.0) * 0.7 + max(dot(up, sun), 0.0) * 0.3;
    let irradiance = in.sun_light * SUN_INTENSITY * diffuse
        + sky_irradiance(in.planet_position, normal, sun);

    let color = albedo / PI * irradiance;
    return vec4<f32>(color * in.transmittance + in.inscattered, 1.0);
}
