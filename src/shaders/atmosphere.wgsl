// Single scattering of sunlight in the atmosphere, by ray marching through Rayleigh and Mie
// scattering layers of exponentially decreasing density, and a layer of absorbing ozone.
//
// All positions are relative to the center of the planet, in meters.

#define_import_path worldmap::atmosphere

const PI: f32 = 3.14159265;
const PLANET_RADIUS: f32 = 6371000.0;
const ATMOSPHERE_RADIUS: f32 = 6471000.0;

const RAYLEIGH_SCATTERING: vec3<f32> = vec3<f32>(5.802e-6, 13.558e-6, 33.1e-6);
const RAYLEIGH_HEIGHT: f32 = 8000.0;
const MIE_SCATTERING: f32 = 3.996e-6;
const MIE_EXTINCTION: f32 = 4.44e-6;
const MIE_HEIGHT: f32 = 1200.0;
const MIE_G: f32 = 0.8;
const OZONE_ABSORPTION: vec3<f32> = vec3<f32>(0.650e-6, 1.881e-6, 0.085e-6);

// Brightness of the sun, in the units the shaders output before tonemapping
const SUN_INTENSITY: f32 = 10.0;

struct Scattering {
    // Light scattered towards the viewer along the ray
    inscattered: vec3<f32>,
    // Fraction of the light from the end of the ray which reaches the viewer
    transmittance: vec3<f32>,
}

// Distances along the ray to where it enters and exits a sphere at the origin, where the
// entry is after the exit if the ray misses it.
fn ray_sphere(origin: vec3<f32>, dir: vec3<f32>, radius: f32) -> vec2<f32> {
    let b = dot(origin, dir);
    // Factored to avoid cancellation between the large squares
    let r = length(origin);
    let c = (r - radius) * (r + radius);
    let discriminant = b * b - c;
    if discriminant < 0.0 {
        return vec2<f32>(1e30, -1e30);
    }
    let s = sqrt(discriminant);
    return vec2<f32>(-b - s, -b + s);
}

// Relative density of the Rayleigh, Mie and ozone layers at a height above the surface
fn densities(height: f32) -> vec3<f32> {
    let h = max(height, 0.0);
    return vec3<f32>(
        exp(-h / RAYLEIGH_HEIGHT),
        exp(-h / MIE_HEIGHT),
        max(0.0, 1.0 - abs(h - 25000.0) / 15000.0),
    );
}

fn extinction(optical_depth: vec3<f32>) -> vec3<f32> {
    return RAYLEIGH_SCATTERING * optical_depth.x
        + vec3<f32>(MIE_EXTINCTION * optical_depth.y)
        + OZONE_ABSORPTION * optical_depth.z;
}

fn rayleigh_phase(mu: f32) -> f32 {
    return 3.0 / (16.0 * PI) * (1.0 + mu * mu);
}

// Cornette-Shanks approximation of Mie scattering
fn mie_phase(mu: f32) -> f32 {
    let g2 = MIE_G * MIE_G;
    let k = 3.0 / (8.0 * PI) * (1.0 - g2) / (2.0 + g2);
    return k * (1.0 + mu * mu) / pow(1.0 + g2 - 2.0 * MIE_G * mu, 1.5);
}

// Fraction of sunlight which reaches `pos`, fading out softly where the planet is in the way.
fn sun_transmittance(pos: vec3<f32>, sun: vec3<f32>) -> vec3<f32> {
    let r = length(pos);
    let mu = dot(pos / r, sun);
    let ratio = min(PLANET_RADIUS / r, 1.0);
    let horizon = -sqrt(max(0.0, 1.0 - ratio * ratio));
    let visible = smoothstep(horizon - 0.01, horizon + 0.01, mu);
    if visible <= 0.0 {
        return vec3<f32>(0.0);
    }

    let path = ray_sphere(pos, sun, ATMOSPHERE_RADIUS).y;
    if path <= 0.0 {
        return vec3<f32>(visible);
    }
    let steps = 6;
    let dt = path / f32(steps);
    var optical_depth = vec3<f32>(0.0);
    for (var i = 0; i < steps; i++) {
        let p = pos + sun * (dt * (f32(i) + 0.5));
        optical_depth += densities(length(p) - PLANET_RADIUS) * dt;
    }
    return exp(-extinction(optical_depth)) * visible;
}

// Light scattered along the ray from `origin` between the distances `t0` and `t1`
fn integrate(origin: vec3<f32>, dir: vec3<f32>, t0: f32, t1: f32, sun: vec3<f32>, steps: i32) -> Scattering {
    let dt = (t1 - t0) / f32(steps);
    var optical_depth = vec3<f32>(0.0);
    var rayleigh = vec3<f32>(0.0);
    var mie = vec3<f32>(0.0);

    for (var i = 0; i < steps; i++) {
        let p = origin + dir * (t0 + dt * (f32(i) + 0.5));
        let density = densities(length(p) - PLANET_RADIUS) * dt;
        let view = exp(-extinction(optical_depth + density * 0.5));
        let light = sun_transmittance(p, sun) * view;
        rayleigh += light * density.x;
        mie += light * density.y;
        optical_depth += density;
    }

    let mu = dot(dir, sun);
    var out: Scattering;
    out.inscattered = SUN_INTENSITY * (
        rayleigh * RAYLEIGH_SCATTERING * rayleigh_phase(mu)
        + mie * MIE_SCATTERING * mie_phase(mu)
    );
    out.transmittance = exp(-extinction(optical_depth));
    return out;
}

// The atmosphere between the camera and a point in front of it. `camera` is relative to the
// planet, while `offset` is the vector from the camera to the point.
fn aerial_perspective(camera: vec3<f32>, offset: vec3<f32>, sun: vec3<f32>) -> Scattering {
    let dist = length(offset);
    let dir = offset / max(dist, 1e-3);
    let atmosphere = ray_sphere(camera, dir, ATMOSPHERE_RADIUS);
    let t0 = max(atmosphere.x, 0.0);
    let t1 = min(dist, atmosphere.y);
    if t1 <= t0 {
        var none: Scattering;
        none.inscattered = vec3<f32>(0.0);
        none.transmittance = vec3<f32>(1.0);
        return none;
    }
    return integrate(camera, dir, t0, t1, sun, 8);
}

// Rough estimate of the light from the sky on a surface, which mostly matters at dusk and in
// the shade of mountains.
fn sky_irradiance(pos: vec3<f32>, normal: vec3<f32>, sun: vec3<f32>) -> vec3<f32> {
    let up = normalize(pos);
    let sun_height = dot(up, sun);
    let day = smoothstep(-0.15, 0.25, sun_height);
    let facing = 0.5 + 0.5 * dot(normal, up);
    return SUN_INTENSITY * 0.12 * day * facing * vec3<f32>(0.55, 0.7, 1.0);
}
