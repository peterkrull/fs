#import bevy_pbr::{
    forward_io::VertexOutput,
    mesh_view_bindings::{view, lights},
}
#import worldmap::atmosphere::{
    PI, PLANET_RADIUS, ATMOSPHERE_RADIUS, SUN_INTENSITY,
    ray_sphere, integrate, sun_transmittance, sky_irradiance,
}

// Position of the camera relative to the center of the planet
@group(#{MATERIAL_BIND_GROUP}) @binding(0) var<uniform> camera: vec4<f32>;

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

    var color = vec3<f32>(0.0);
    var transmittance = vec3<f32>(1.0);
    var ground = false;

    let atmosphere = ray_sphere(origin, dir, ATMOSPHERE_RADIUS);
    if atmosphere.y > 0.0 && atmosphere.x < atmosphere.y {
        let planet = ray_sphere(origin, dir, PLANET_RADIUS);
        let t0 = max(atmosphere.x, 0.0);
        var t1 = atmosphere.y;
        if planet.x < planet.y && planet.x > 0.0 {
            t1 = planet.x;
            ground = true;
        }
        let scattering = integrate(origin, dir, t0, t1, sun, 16);
        color = scattering.inscattered;
        transmittance = scattering.transmittance;

        if ground {
            let p = origin + dir * t1;
            let up = normalize(p);
            let irradiance = sun_transmittance(p, sun) * SUN_INTENSITY * max(dot(up, sun), 0.0)
                + sky_irradiance(p, up, sun);
            color += transmittance * OCEAN / PI * irradiance;
        }
    }

    if !ground {
        let sun_disk = smoothstep(cos(SUN_RADIUS * 1.2), cos(SUN_RADIUS), dot(dir, sun));
        color += transmittance * sun_disk * SUN_DISK;
    }

    return vec4<f32>(color, 1.0);
}
