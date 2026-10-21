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
    coarser_uv_transform: vec4<f32>,
    previous_coarser_uv_transform: vec4<f32>,
    // Detail threshold, anisotropy, and the zoom of the tiles of the imagery and the previous
    // imagery
    lod: vec4<f32>,
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
@group(#{MATERIAL_BIND_GROUP}) @binding(10) var coarser: texture_2d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(11) var previous_coarser: texture_2d<f32>;

const EARTH_RADIUS: f32 = 6371000.0;

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
    @location(6) coarser_uv: vec2<f32>,
    @location(7) previous_coarser_uv: vec2<f32>,
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
    out.coarser_uv = params.coarser_uv_transform.xy + vertex.uv * params.coarser_uv_transform.z;
    out.previous_coarser_uv = params.previous_coarser_uv_transform.xy
        + vertex.uv * params.previous_coarser_uv_transform.z;
    out.normal = mix(vertex.parent_normal, vertex.normal, morph);
    out.planet_position = world - center;
    out.offset = world - view.world_position;
    out.camera = view.world_position - center;
    return out;
}

// The zoom level whose imagery would appear exactly as detailed as the detail threshold here,
// as the level of detail is chosen in `terrain.rs`: tiles of zoom `z` are drawn where this is
// between `z - 1` and `z`.
fn ideal_zoom(up: vec3<f32>, dir: vec3<f32>, dist: f32) -> f32 {
    let pixels_per_radian = view.viewport.w * view.clip_from_view[1][1] * 0.5;
    // Texels of the Web Mercator projection shrink towards the poles
    let texel = 2.0 * PI * EARTH_RADIUS * max(length(up.xy), 1e-6) / 256.0;
    let foreshortening = 1.0 - params.lod.y * (1.0 - abs(dot(dir, up)));
    return log2(texel * pixels_per_radian * foreshortening / (params.lod.x * dist));
}

// The imagery `own`, whose low frequencies blend by `weight` into those of the imagery `above`,
// of a zoom level `levels` coarser. The detail of `own` is kept, so that the imagery stays as
// sharp, while it takes the tone of `above`. Both are low-passed to the size of the texels of
// `above`, where those are larger than the pixels.
fn blend_above(
    own: texture_2d<f32>,
    above: texture_2d<f32>,
    imagery_sampler: sampler,
    uv: vec2<f32>,
    above_uv: vec2<f32>,
    levels: f32,
    weight: f32,
) -> vec3<f32> {
    let size = f32(textureDimensions(own).x);
    let footprint = max(length(dpdx(uv)), length(dpdy(uv))) * size;
    let level = log2(max(footprint, 1e-6));
    let color = textureSample(own, imagery_sampler, uv).rgb;
    let own_low = textureSampleLevel(own, imagery_sampler, uv, max(level, levels)).rgb;
    let above_low = textureSampleLevel(above, imagery_sampler, above_uv, max(level - levels, 0.0)).rgb;
    return max(color + weight * (above_low - own_low), vec3<f32>(0.0));
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let sun = lights.directional_lights[0].direction_to_light;
    let r = length(in.planet_position);
    let up = in.planet_position / r;
    let dist = length(in.offset);
    let dir = in.offset / dist;

    // The imagery takes on the tone of that of the level above towards where that level is
    // drawn, and there matches it, so that imagery of different zoom levels meets without a
    // seam, even where it is from different sources
    let zoom = ideal_zoom(up, dir, dist);
    var albedo = blend_above(
        imagery,
        coarser,
        imagery_sampler,
        in.uv,
        in.coarser_uv,
        log2(params.uv_transform.z / params.coarser_uv_transform.z),
        clamp(params.lod.z - zoom, 0.0, 1.0),
    ) * params.base_color.rgb;
    // The fade is the same over the whole tile, so this branch is uniform
    if params.fade.y < 1.0 {
        let previous = blend_above(
            previous,
            previous_coarser,
            previous_sampler,
            in.previous_uv,
            in.previous_coarser_uv,
            log2(params.previous_uv_transform.z / params.previous_coarser_uv_transform.z),
            clamp(params.lod.w - zoom, 0.0, 1.0),
        ) * params.previous_color.rgb;
        albedo = mix(previous, albedo, params.fade.y);
    }

    // The imagery already has shading baked in, so the relief is only shaded lightly
    let normal = normalize(in.normal);
    let diffuse = max(dot(normal, sun), 0.0) * 0.7 + max(dot(up, sun), 0.0) * 0.3;
    let sun_light = sun_transmittance(transmittance_lut, lut_sampler, r, dot(up, sun));
    let irradiance = sun_light * SUN_INTENSITY * diffuse
        + sky_irradiance(irradiance_lut, lut_sampler, in.planet_position, normal, sun);

    let scattering = aerial_perspective(
        aerial_inscattering, aerial_transmittance, lut_sampler, in.camera, dir, sun, dist,
    );
    let color = albedo / PI * irradiance;
    return vec4<f32>(color * scattering.transmittance + scattering.inscattered, 1.0);
}
