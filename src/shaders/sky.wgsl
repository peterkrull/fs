#import bevy_pbr::{
    forward_io::VertexOutput,
    mesh_view_bindings::{view, lights},
}
#import worldmap::atmosphere::{
    PI, PLANET_RADIUS, ATMOSPHERE_RADIUS, SUN_INTENSITY,
    ray_sphere, transmittance_uv, sun_transmittance, sky_irradiance, aerial_perspective,
}

// Position of the camera relative to the center of the planet
@group(#{MATERIAL_BIND_GROUP}) @binding(0) var<uniform> camera: vec4<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(1) var transmittance_lut: texture_2d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(2) var irradiance_lut: texture_2d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(3) var aerial_inscattering: texture_3d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(4) var aerial_transmittance: texture_3d<f32>;
@group(#{MATERIAL_BIND_GROUP}) @binding(5) var lut_sampler: sampler;

// Angular radius of the sun
const SUN_RADIUS: f32 = 0.00465;
const SUN_DISK: f32 = 400.0;
// Fallback for where the terrain has not been loaded yet
const OCEAN: vec3<f32> = vec3<f32>(0.01, 0.025, 0.06);

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let origin = camera.xyz;
    let dir = normalize(in.world_position.xyz - view.world_position);
    let sun = lights.directional_lights[0].direction_to_light;

    let atmosphere = ray_sphere(origin, dir, ATMOSPHERE_RADIUS);
    let inside = atmosphere.y > 0.0 && atmosphere.x < atmosphere.y;
    let planet = ray_sphere(origin, dir, PLANET_RADIUS);
    let ground = planet.x < planet.y && planet.x > 0.0;

    // The far end of the volume holds the whole sky
    var dist = 1e30;
    if ground {
        dist = planet.x;
    }
    var color = vec3<f32>(0.0);
    if inside {
        let scattering = aerial_perspective(
            aerial_inscattering, aerial_transmittance, lut_sampler, origin, dir, sun, dist,
        );
        color = scattering.inscattered;
        if ground {
            let p = origin + dir * dist;
            let up = normalize(p);
            let irradiance = sun_transmittance(transmittance_lut, lut_sampler, PLANET_RADIUS, dot(up, sun))
                * SUN_INTENSITY * max(dot(up, sun), 0.0)
                + sky_irradiance(irradiance_lut, lut_sampler, p, up, sun);
            color += scattering.transmittance * OCEAN / PI * irradiance;
        }
    }

    if !ground {
        // Through the atmosphere from where the ray enters it, if the camera is above it
        var transmittance = vec3<f32>(1.0);
        if inside {
            let start = origin + dir * max(atmosphere.x, 0.0);
            let r = length(start);
            let uv = transmittance_uv(r, dot(start / r, dir), textureDimensions(transmittance_lut));
            transmittance = textureSampleLevel(transmittance_lut, lut_sampler, uv, 0.0).rgb;
        }
        let sun_disk = smoothstep(cos(SUN_RADIUS * 1.2), cos(SUN_RADIUS), dot(dir, sun));
        color += transmittance * sun_disk * SUN_DISK;
    }

    return vec4<f32>(color, 1.0);
}
