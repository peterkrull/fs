// The lookup tables of the atmosphere, after Hillaire, "A Scalable and Production Ready Sky and
// Atmosphere Rendering Technique" (2020). Sunlight is scattered by air molecules (Rayleigh) and
// aerosols (Mie), in layers of exponentially decreasing density, and absorbed by a layer of
// ozone. Light scattered more than once is approximated with a table of its contribution by
// height and sun angle, as if it scattered equally in all directions.
//
// The transmittance, multiple scattering and sky irradiance tables depend only on the
// atmosphere, and are computed once it changes. The scattering volume depends on the camera
// and the sun, and is computed every frame.

#import worldmap::atmosphere::{
    PI, PLANET_RADIUS, ATMOSPHERE_RADIUS, SUN_INTENSITY, Scattering,
    ray_sphere, top_horizon, distance_to_top, sun_height_uv, sun_transmittance,
    volume_radius, volume_elevations, volume_range, volume_top,
}

struct Params {
    // Distance of the camera from the center of the planet
    radius: f32,
    // Cosine of the zenith angle of the sun at the camera
    sun_mu: f32,
    // Vertical optical depth of the aerosols at 550 nm, from sea level
    aerosols: f32,
}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var lut_sampler: sampler;
@group(0) @binding(2) var transmittance_lut: texture_2d<f32>;
@group(0) @binding(3) var multiscattering_lut: texture_2d<f32>;
@group(0) @binding(4) var transmittance_out: texture_storage_2d<rgba16float, write>;
@group(0) @binding(5) var multiscattering_out: texture_storage_2d<rgba16float, write>;
@group(0) @binding(6) var irradiance_out: texture_storage_2d<rgba16float, write>;
@group(0) @binding(7) var inscattering_out: texture_storage_3d<rgba16float, write>;
@group(0) @binding(8) var aerial_transmittance_out: texture_storage_3d<rgba16float, write>;

// Scattering of air at sea level for red, green and blue light (680, 550 and 440 nm)
const RAYLEIGH_SCATTERING: vec3<f32> = vec3<f32>(5.802e-6, 13.558e-6, 33.1e-6);
const RAYLEIGH_HEIGHT: f32 = 8000.0;
// Aerosols scatter shorter wavelengths a little more, with an Angstrom exponent of 1.3
const MIE_SPECTRUM: vec3<f32> = vec3<f32>(0.759, 1.0, 1.337);
const MIE_HEIGHT: f32 = 1200.0;
// Fraction of the light aerosols take out of a ray which they scatter, rather than absorb
const MIE_ALBEDO: f32 = 0.9;
const MIE_G: f32 = 0.8;
const OZONE_ABSORPTION: vec3<f32> = vec3<f32>(0.650e-6, 1.881e-6, 0.085e-6);
// Average reflectance of the ground, for the light it scatters back up into the atmosphere
const GROUND_ALBEDO: f32 = 0.3;

// Steps along each ray of the tables
const TRANSMITTANCE_STEPS: u32 = 40u;
const MULTISCATTERING_STEPS: u32 = 20u;
const IRRADIANCE_STEPS: u32 = 16u;
// Steps between consecutive distances of the volume
const VOLUME_STEPS: u32 = 2u;
// Directions per texel of the multiple scattering and irradiance tables, one per invocation
const DIRECTIONS: u32 = 64u;

struct Medium {
    rayleigh: vec3<f32>,
    mie: vec3<f32>,
    extinction: vec3<f32>,
}

fn medium(r: f32) -> Medium {
    let h = max(r - PLANET_RADIUS, 0.0);
    let mie_extinction = MIE_SPECTRUM * (params.aerosols / MIE_HEIGHT) * exp(-h / MIE_HEIGHT);
    let ozone = max(0.0, 1.0 - abs(h - 25000.0) / 15000.0);

    var out: Medium;
    out.rayleigh = RAYLEIGH_SCATTERING * exp(-h / RAYLEIGH_HEIGHT);
    out.mie = mie_extinction * MIE_ALBEDO;
    out.extinction = out.rayleigh + mie_extinction + OZONE_ABSORPTION * ozone;
    return out;
}

fn rayleigh_phase(nu: f32) -> f32 {
    return 3.0 / (16.0 * PI) * (1.0 + nu * nu);
}

// Cornette-Shanks approximation of Mie scattering
fn mie_phase(nu: f32) -> f32 {
    let g2 = MIE_G * MIE_G;
    let k = 3.0 / (8.0 * PI) * (1.0 - g2) / (2.0 + g2);
    return k * (1.0 + nu * nu) / pow(1.0 + g2 - 2.0 * MIE_G * nu, 1.5);
}

// The inverse of `texel_coord`, for the texel at `index`
fn texel_unit(index: u32, size: u32) -> f32 {
    return f32(index) / f32(size - 1u);
}

// Light scattered more than once, per unit of scattering
fn multiscattering(r: f32, mu_s: f32) -> vec3<f32> {
    let uv = sun_height_uv(r, mu_s, textureDimensions(multiscattering_lut));
    return textureSampleLevel(multiscattering_lut, lut_sampler, uv, 0.0).rgb;
}

// Adds the light scattered towards `origin` from between the distances `t0` and `t1` along
// the ray to `acc`, for a sun of unit brightness. Each step integrates the light scattered
// within it analytically, as if the medium were the same throughout.
fn march(origin: vec3<f32>, dir: vec3<f32>, sun: vec3<f32>, t0: f32, t1: f32, steps: u32, acc: Scattering) -> Scattering {
    var out = acc;
    let dt = (t1 - t0) / f32(steps);
    let nu = dot(dir, sun);
    let rayleigh_p = rayleigh_phase(nu);
    let mie_p = mie_phase(nu);

    for (var i = 0u; i < steps; i++) {
        let p = origin + dir * (t0 + dt * (f32(i) + 0.5));
        let r = length(p);
        let mu_s = dot(p, sun) / r;
        let m = medium(r);
        let single = sun_transmittance(transmittance_lut, lut_sampler, r, mu_s)
            * (m.rayleigh * rayleigh_p + m.mie * mie_p);
        let source = single + multiscattering(r, mu_s) * (m.rayleigh + m.mie);
        let step_transmittance = exp(-m.extinction * dt);
        out.inscattered += out.transmittance * source * (1.0 - step_transmittance)
            / max(m.extinction, vec3<f32>(1e-12));
        out.transmittance *= step_transmittance;
    }
    return out;
}

// Sums a value over the invocations of a workgroup of `DIRECTIONS`
var<workgroup> sums: array<vec4<f32>, DIRECTIONS>;

fn workgroup_sum(value: vec4<f32>, index: u32) -> vec4<f32> {
    // Until all have read the previous sum
    workgroupBarrier();
    sums[index] = value;
    workgroupBarrier();
    for (var stride = DIRECTIONS / 2u; stride > 0u; stride /= 2u) {
        if index < stride {
            sums[index] += sums[index + stride];
        }
        workgroupBarrier();
    }
    return sums[0];
}

@compute @workgroup_size(8, 8, 1)
fn transmittance(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(transmittance_out);
    if any(id.xy >= size) {
        return;
    }

    // The inverse of `transmittance_uv`
    let h = top_horizon();
    let rho = h * texel_unit(id.y, size.y);
    let r = sqrt(rho * rho + PLANET_RADIUS * PLANET_RADIUS);
    let d_min = ATMOSPHERE_RADIUS - r;
    let d_max = rho + h;
    let d = d_min + texel_unit(id.x, size.x) * (d_max - d_min);
    var mu = 1.0;
    if d > 0.0 {
        mu = clamp((h * h - rho * rho - d * d) / (2.0 * r * d), -1.0, 1.0);
    }

    let dt = distance_to_top(r, mu) / f32(TRANSMITTANCE_STEPS);
    let along = vec2<f32>(sqrt(1.0 - mu * mu), mu);
    var optical_depth = vec3<f32>(0.0);
    for (var i = 0u; i < TRANSMITTANCE_STEPS; i++) {
        let p = vec2<f32>(0.0, r) + along * (dt * (f32(i) + 0.5));
        optical_depth += medium(length(p)).extinction * dt;
    }
    // The alpha channel tells the copies read back to the CPU which aerosols they are for
    textureStore(transmittance_out, id.xy, vec4<f32>(exp(-optical_depth), params.aerosols));
}

// Height and zenith cosine of the sun of a texel of the multiple scattering and irradiance
// tables, the inverse of `sun_height_uv`
fn sun_height(texel: vec2<u32>, size: vec2<u32>) -> vec2<f32> {
    // Kept off the very bottom and top, where rays graze the planet or leave at once
    let r = mix(PLANET_RADIUS + 10.0, ATMOSPHERE_RADIUS - 10.0, texel_unit(texel.y, size.y));
    let mu_s = texel_unit(texel.x, size.x) * 2.0 - 1.0;
    return vec2<f32>(r, mu_s);
}

// One workgroup per texel, with an invocation for each direction in which the light is
// followed: the light scattered once more at a point is that which arrives there from all
// directions, after scattering once, and from the ground. Following Hillaire, it is assumed
// that all orders of scattering beyond this arrive equally from all directions, as a geometric
// series of the fraction `transfer` of the light which is scattered back to the point.
@compute @workgroup_size(DIRECTIONS, 1, 1)
fn multiscattering_table(
    @builtin(workgroup_id) texel: vec3<u32>,
    @builtin(local_invocation_index) index: u32,
) {
    let size = textureDimensions(multiscattering_out);
    let rm = sun_height(texel.xy, size);
    let origin = vec3<f32>(0.0, 0.0, rm.x);
    let sun = vec3<f32>(sqrt(1.0 - rm.y * rm.y), 0.0, rm.y);

    // Evenly over the sphere
    let cell = vec2<f32>(f32(index % 8u), f32(index / 8u)) + 0.5;
    let cos_theta = 1.0 - cell.y / 4.0;
    let sin_theta = sqrt(1.0 - cos_theta * cos_theta);
    let phi = cell.x / 8.0 * 2.0 * PI;
    let dir = vec3<f32>(sin_theta * cos(phi), sin_theta * sin(phi), cos_theta);

    let ground = ray_sphere(origin, dir, PLANET_RADIUS);
    let hits_ground = ground.x > 0.0 && ground.x <= ground.y;
    var t_max = ray_sphere(origin, dir, ATMOSPHERE_RADIUS).y;
    if hits_ground {
        t_max = ground.x;
    }

    let isotropic = 1.0 / (4.0 * PI);
    let dt = t_max / f32(MULTISCATTERING_STEPS);
    var throughput = vec3<f32>(1.0);
    var light = vec3<f32>(0.0);
    var transfer = vec3<f32>(0.0);
    for (var i = 0u; i < MULTISCATTERING_STEPS; i++) {
        let p = origin + dir * (dt * (f32(i) + 0.5));
        let r = length(p);
        let m = medium(r);
        let scattering = m.rayleigh + m.mie;
        let step_transmittance = exp(-m.extinction * dt);
        let integral = throughput * (1.0 - step_transmittance) / max(m.extinction, vec3<f32>(1e-12));
        let sunlight = sun_transmittance(transmittance_lut, lut_sampler, r, dot(p, sun) / r);
        light += integral * sunlight * scattering * isotropic;
        transfer += integral * scattering;
        throughput *= step_transmittance;
    }
    if hits_ground {
        let p = origin + dir * t_max;
        let mu_s = dot(normalize(p), sun);
        let sunlight = sun_transmittance(transmittance_lut, lut_sampler, PLANET_RADIUS, mu_s);
        light += throughput * sunlight * max(mu_s, 0.0) * GROUND_ALBEDO / PI;
    }

    // Averaged over the sphere, with the isotropic phase function
    let total_light = workgroup_sum(vec4<f32>(light, 0.0), index).rgb / f32(DIRECTIONS);
    let total_transfer = workgroup_sum(vec4<f32>(transfer, 0.0), index).rgb / f32(DIRECTIONS);
    if index == 0u {
        let result = total_light / max(vec3<f32>(1.0) - total_transfer, vec3<f32>(1e-3));
        textureStore(multiscattering_out, texel.xy, vec4<f32>(result, 1.0));
    }
}

// The light of the sky on a horizontal surface, by height and zenith cosine of the sun. One
// workgroup per texel, with an invocation for each direction of the sky, distributed by the
// cosine of their angle to the surface.
@compute @workgroup_size(DIRECTIONS, 1, 1)
fn irradiance_table(
    @builtin(workgroup_id) texel: vec3<u32>,
    @builtin(local_invocation_index) index: u32,
) {
    let size = textureDimensions(irradiance_out);
    let rm = sun_height(texel.xy, size);
    let origin = vec3<f32>(0.0, 0.0, rm.x);
    let sun = vec3<f32>(sqrt(1.0 - rm.y * rm.y), 0.0, rm.y);

    let cell = vec2<f32>(f32(index % 8u), f32(index / 8u)) + 0.5;
    let sin_theta = sqrt(cell.y / 8.0);
    let cos_theta = sqrt(1.0 - sin_theta * sin_theta);
    let phi = cell.x / 8.0 * 2.0 * PI;
    let dir = vec3<f32>(sin_theta * cos(phi), sin_theta * sin(phi), cos_theta);

    var none: Scattering;
    none.inscattered = vec3<f32>(0.0);
    none.transmittance = vec3<f32>(1.0);
    let t_max = ray_sphere(origin, dir, ATMOSPHERE_RADIUS).y;
    let radiance = march(origin, dir, sun, 0.0, t_max, IRRADIANCE_STEPS, none).inscattered;

    // With the directions distributed by the cosine, the irradiance is pi times the average
    let total = workgroup_sum(vec4<f32>(radiance, 0.0), index).rgb / f32(DIRECTIONS);
    if index == 0u {
        textureStore(irradiance_out, texel.xy, vec4<f32>(total * PI * SUN_INTENSITY, params.aerosols));
    }
}

// The scattering volume, as described in `atmosphere.wgsl`, with an invocation for each
// direction, marching outwards through the distances.
@compute @workgroup_size(8, 8, 1)
fn volume(@builtin(global_invocation_id) id: vec3<u32>) {
    let size = textureDimensions(inscattering_out);
    if any(id.xy >= size.xy) {
        return;
    }

    let r = volume_radius(params.radius);
    let origin = vec3<f32>(0.0, 0.0, r);
    let sun = vec3<f32>(sqrt(max(0.0, 1.0 - params.sun_mu * params.sun_mu)), 0.0, params.sun_mu);

    // The inverse of `volume_uvw`
    let azimuth = texel_unit(id.x, size.x) * PI;
    let bounds = volume_elevations(r);
    let half = size.y / 2u;
    let below = id.y >= half;
    var elevation: f32;
    if below {
        let y = f32(id.y - half) / f32(half - 1u);
        elevation = bounds.x - y * y * (bounds.x + PI * 0.5);
    } else {
        let x = 1.0 - f32(id.y) / f32(half - 1u);
        elevation = bounds.x + x * x * (bounds.y - bounds.x);
    }
    let dir = vec3<f32>(cos(elevation) * cos(azimuth), cos(elevation) * sin(azimuth), sin(elevation));

    // Rays in the lower half end on the ground, the one along the horizon where it touches
    // the planet, and those in the upper half never do, even where they graze it.
    let atmosphere = ray_sphere(origin, dir, ATMOSPHERE_RADIUS);
    var end = atmosphere.y;
    if below {
        let ground = ray_sphere(origin, dir, PLANET_RADIUS);
        if ground.x <= ground.y {
            end = min(end, ground.x);
        } else {
            end = min(end, sqrt((r - PLANET_RADIUS) * (r + PLANET_RADIUS)));
        }
    }
    let start = max(atmosphere.x, 0.0);

    let range = volume_range(r);
    let top = volume_top(r);
    var acc: Scattering;
    acc.inscattered = vec3<f32>(0.0);
    acc.transmittance = vec3<f32>(1.0);
    var previous = 0.0;
    for (var k = 0u; k < size.z; k++) {
        let x = texel_unit(k, size.z);
        var dist = range.x * pow(range.y / range.x, x);
        if below {
            // Where the ray descends to the height of this point, or failing that along the
            // horizon, where it touches the planet
            let height = top * (1.0 - x) * (1.0 - x);
            let sphere = ray_sphere(origin, dir, PLANET_RADIUS + height);
            dist = end;
            if sphere.x <= sphere.y {
                dist = clamp(sphere.x, 0.0, end);
            }
        }
        let t0 = max(previous, start);
        let t1 = min(dist, end);
        if t1 > t0 {
            acc = march(origin, dir, sun, t0, t1, VOLUME_STEPS, acc);
        }
        previous = dist;
        textureStore(inscattering_out, vec3<u32>(id.xy, k), vec4<f32>(acc.inscattered * SUN_INTENSITY, 1.0));
        textureStore(aerial_transmittance_out, vec3<u32>(id.xy, k), vec4<f32>(acc.transmittance, 1.0));
    }
}
