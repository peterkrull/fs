#import bevy_pbr::{
    mesh_functions,
    mesh_view_bindings::{view, lights},
    view_transformations::position_world_to_clip,
}
#import worldmap::atmosphere::{
    PI, SUN_INTENSITY, aerial_perspective, sun_transmittance, sky_irradiance,
}

struct TerrainParams {
    uv_transform: vec4<f32>,
    previous_uv_transform: vec4<f32>,
    planet_center: vec4<f32>,
    base_color: vec4<f32>,
    previous_color: vec4<f32>,
    // Geometry faded in from the parent tile in x, imagery from the previous imagery in y
    fade: vec4<f32>,
    // Edges at u = 0, u = 1, v = 0 and v = 1 which are moved onto the parent, as 1
    edges: vec4<f32>,
}

@group(#{MATERIAL_BIND_GROUP}) @binding(0) var<uniform> params: TerrainParams;
@group(#{MATERIAL_BIND_GROUP}) @binding(1) var imagery: texture_2d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(2) var imagery_sampler: sampler;
@group(#{MATERIAL_BIND_GROUP}) @binding(3) var previous: texture_2d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(4) var previous_sampler: sampler;
@group(#{MATERIAL_BIND_GROUP}) @binding(5) var transmittance_lut: texture_2d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(6) var irradiance_lut: texture_2d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(7) var aerial_inscattering: texture_3d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(8) var aerial_transmittance: texture_3d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(9) var lut_sampler: sampler;

struct Vertex {
    @builtin(instance_index) instance_index: u32,
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) parent_position: vec3<f32>,
    @location(4) parent_normal: vec3<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) planet_position: vec3<f32>,
    // From the camera
    @location(3) offset: vec3<f32>,
    // The camera relative to the center of the planet, the same over the tile
    @location(4) @interpolate(flat) camera: vec3<f32>,
    @location(5) previous_uv: vec2<f32>,
}

@vertex
fn vertex(vertex: Vertex) -> VertexOutput {
    let world_from_local = mesh_functions::get_world_from_local(vertex.instance_index);
    // Where the tile borders a coarser one, its edge lies on the parent, and so on the neighbour
    let on_edge = vec4<f32>(
        f32(vertex.uv.x <= 0.0),
        f32(vertex.uv.x >= 1.0),
        f32(vertex.uv.y <= 0.0),
        f32(vertex.uv.y >= 1.0),
    );
    let snapped = on_edge * params.edges;
    let morph = params.fade.x * (1.0 - max(max(snapped.x, snapped.y), max(snapped.z, snapped.w)));
    let position = mix(vertex.parent_position, vertex.position, morph);
    let world = mesh_functions::mesh_position_local_to_world(world_from_local, vec4<f32>(position, 1.0)).xyz;
    // Meshes are only ever translated
    let center = (world_from_local * vec4<f32>(params.planet_center.xyz, 1.0)).xyz;

    var out: VertexOutput;
    out.clip_position = position_world_to_clip(world);
    out.uv = params.uv_transform.xy + vertex.uv * params.uv_transform.z;
    out.previous_uv = params.previous_uv_transform.xy + vertex.uv * params.previous_uv_transform.z;
    out.normal = mix(vertex.parent_normal, vertex.normal, morph);
    out.planet_position = world - center;
    out.offset = world - view.world_position;
    out.camera = view.world_position - center;
    return out;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let sun = lights.directional_lights[0].direction_to_light;
    var albedo = textureSample(imagery, imagery_sampler, in.uv).rgb * params.base_color.rgb;
    // The fade is the same over the whole tile, so this branch is uniform
    if params.fade.y < 1.0 {
        let previous = textureSample(previous, previous_sampler, in.previous_uv).rgb * params.previous_color.rgb;
        albedo = mix(previous, albedo, params.fade.y);
    }

    // The imagery already has shading baked in, so the relief is only shaded lightly
    let normal = normalize(in.normal);
    let r = length(in.planet_position);
    let up = in.planet_position / r;
    let diffuse = max(dot(normal, sun), 0.0) * 0.7 + max(dot(up, sun), 0.0) * 0.3;
    let sun_light = sun_transmittance(transmittance_lut, lut_sampler, r, dot(up, sun));
    let irradiance = sun_light * SUN_INTENSITY * diffuse
        + sky_irradiance(irradiance_lut, lut_sampler, in.planet_position, normal, sun);

    let dist = length(in.offset);
    let scattering = aerial_perspective(
        aerial_inscattering, aerial_transmittance, lut_sampler, in.camera, in.offset / dist, sun, dist,
    );
    let color = albedo / PI * irradiance;
    return vec4<f32>(color * scattering.transmittance + scattering.inscattered, 1.0);
}
