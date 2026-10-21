// The atmosphere, as seen through the lookup tables computed in `scattering.wgsl`: the
// transmittance to the top of the atmosphere, the light of the sky on a horizontal surface, and
// the light scattered towards the camera and the transmittance along every direction from it,
// by distance.
//
// All positions are relative to the center of the planet, in meters.

#define_import_path worldmap::atmosphere

const PI: f32 = 3.14159265;
const PLANET_RADIUS: f32 = 6371000.0;
const ATMOSPHERE_RADIUS: f32 = 6471000.0;

// Brightness of the sun, in the units the shaders output before tonemapping
const SUN_INTENSITY: f32 = 10.0;

// Closest distance to the camera in the scattering volume, which fades out towards it
const VOLUME_NEAR: f32 = 100.0;

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

// Distance from the planet to the top of the atmosphere along a tangent to the planet
fn top_horizon() -> f32 {
    return sqrt((ATMOSPHERE_RADIUS - PLANET_RADIUS) * (ATMOSPHERE_RADIUS + PLANET_RADIUS));
}

// Distance to the top of the atmosphere from radius `r`, along a ray at the zenith cosine `mu`
fn distance_to_top(r: f32, mu: f32) -> f32 {
    let discriminant = (ATMOSPHERE_RADIUS - r) * (ATMOSPHERE_RADIUS + r) + r * r * mu * mu;
    return max(0.0, -r * mu + sqrt(max(discriminant, 0.0)));
}

// Texture coordinate of `x` from 0 to 1, where 0 and 1 are the centers of the first and the
// last texels, so that both ends are sampled exactly.
fn texel_coord(x: f32, size: u32) -> f32 {
    return (0.5 + x * f32(size - 1u)) / f32(size);
}

// The transmittance table is indexed by the distance to the top of the atmosphere, after
// Bruneton, which gives the most resolution near the horizon.
fn transmittance_uv(r: f32, mu: f32, size: vec2<u32>) -> vec2<f32> {
    let h = top_horizon();
    let rho = sqrt(max(0.0, (r - PLANET_RADIUS) * (r + PLANET_RADIUS)));
    let d = distance_to_top(r, mu);
    let d_min = ATMOSPHERE_RADIUS - r;
    let d_max = rho + h;
    let x_mu = clamp((d - d_min) / (d_max - d_min), 0.0, 1.0);
    let x_r = clamp(rho / h, 0.0, 1.0);
    return vec2<f32>(texel_coord(x_mu, size.x), texel_coord(x_r, size.y));
}

// Tables indexed by the cosine of the zenith angle of the sun and the height
fn sun_height_uv(r: f32, mu_s: f32, size: vec2<u32>) -> vec2<f32> {
    let x_r = clamp((r - PLANET_RADIUS) / (ATMOSPHERE_RADIUS - PLANET_RADIUS), 0.0, 1.0);
    return vec2<f32>(texel_coord(mu_s * 0.5 + 0.5, size.x), texel_coord(x_r, size.y));
}

// Fraction of sunlight which reaches radius `r` with the sun at the zenith cosine `mu_s`,
// fading out softly where the planet is in the way.
fn sun_transmittance(lut: texture_2d<f32>, lut_sampler: sampler, r: f32, mu_s: f32) -> vec3<f32> {
    let ratio = min(PLANET_RADIUS / r, 1.0);
    let horizon = -sqrt(max(0.0, 1.0 - ratio * ratio));
    let visible = smoothstep(horizon - 0.01, horizon + 0.01, mu_s);
    let uv = transmittance_uv(r, max(mu_s, horizon), textureDimensions(lut));
    return textureSampleLevel(lut, lut_sampler, uv, 0.0).rgb * visible;
}

// Light from the sky on a surface facing `normal`, from the table of the light on a horizontal
// surface, which is scaled down as the surface turns away from the sky.
fn sky_irradiance(lut: texture_2d<f32>, lut_sampler: sampler, pos: vec3<f32>, normal: vec3<f32>, sun: vec3<f32>) -> vec3<f32> {
    let r = length(pos);
    let up = pos / r;
    let uv = sun_height_uv(r, dot(up, sun), textureDimensions(lut));
    let horizontal = textureSampleLevel(lut, lut_sampler, uv, 0.0).rgb;
    return horizontal * (0.5 + 0.5 * dot(normal, up));
}

// The scattering volume holds, for every direction from the camera, the light scattered
// towards it and the transmittance to points along that direction. As the atmosphere is
// symmetric around the planet, it only depends on the height of the camera and that of the
// sun, and not on where the camera looks.
//
// Directions are indexed by their azimuth from the sun, from 0 to 180 degrees, as either side
// is the same, and by their elevation. The upper half of the elevations runs from the zenith,
// or from the highest ray which touches the atmosphere when above it, down to the horizon,
// and the lower half from the horizon down to the nadir. Both halves are spaced quadratically
// from the horizon, where the light changes the most, and their texels meet at the horizon,
// so that the sky never blends with the ground.
//
// Points along rays in the upper half are indexed by their distance, from `VOLUME_NEAR` to
// the farthest the atmosphere can be seen, spaced logarithmically. Rays in the lower half
// descend all the way to the ground, and their points are indexed by height instead, from the
// camera or the top of the atmosphere down to the ground, spaced quadratically towards it.
// This keeps the dense air near the ground, where most of the light is scattered, finely
// resolved however far away it is, such as when seen from orbit.

// The camera may be below the sphere of the planet, in a valley below sea level
fn volume_radius(r: f32) -> f32 {
    return max(r, PLANET_RADIUS + 1.0);
}

// Elevations of the horizon and the top of the upper half, in radians
fn volume_elevations(r: f32) -> vec2<f32> {
    let horizon = -acos(PLANET_RADIUS / r);
    var top = PI * 0.5;
    if r > ATMOSPHERE_RADIUS {
        top = -acos(ATMOSPHERE_RADIUS / r);
    }
    return vec2<f32>(horizon, top);
}

// Nearest and farthest distances in the volume, where the farthest is that of the far end of
// a ray past the horizon.
fn volume_range(r: f32) -> vec2<f32> {
    let near = max(r - ATMOSPHERE_RADIUS, 0.0) + VOLUME_NEAR;
    let far = sqrt((r - PLANET_RADIUS) * (r + PLANET_RADIUS)) + top_horizon();
    return vec2<f32>(near, max(far, near * 2.0));
}

// Height of the first point along the rays in the lower half
fn volume_top(r: f32) -> f32 {
    return min(r, ATMOSPHERE_RADIUS) - PLANET_RADIUS;
}

// Row of the volume, from 0 to its height minus 1, at an elevation in radians
fn volume_row(elevation: f32, r: f32, rows: u32) -> f32 {
    let bounds = volume_elevations(r);
    let half = f32(rows / 2u);
    if elevation >= bounds.x {
        let x = clamp((elevation - bounds.x) / (bounds.y - bounds.x), 0.0, 1.0);
        return (1.0 - sqrt(x)) * (half - 1.0);
    }
    let y = clamp((bounds.x - elevation) / (bounds.x + PI * 0.5), 0.0, 1.0);
    return half + sqrt(y) * (half - 1.0);
}

// Coordinates in the volume of the direction `dir` from the camera at `camera`, with the sun
// towards `sun`, at the distance `dist`.
fn volume_uvw(camera: vec3<f32>, dir: vec3<f32>, sun: vec3<f32>, dist: f32, size: vec3<u32>) -> vec3<f32> {
    let r = volume_radius(length(camera));
    let up = normalize(camera);
    let mu = clamp(dot(dir, up), -1.0, 1.0);
    let elevation = asin(mu);

    let across = dir - up * mu;
    let sun_across = sun - up * dot(sun, up);
    let lengths = length(across) * length(sun_across);
    var cos_azimuth = 1.0;
    if lengths > 1e-6 {
        cos_azimuth = clamp(dot(across, sun_across) / lengths, -1.0, 1.0);
    }
    let u = texel_coord(acos(cos_azimuth) / PI, size.x);
    let v = (volume_row(elevation, r, size.y) + 0.5) / f32(size.y);

    var x: f32;
    if elevation < volume_elevations(r).x {
        let height = length(camera + dir * dist) - PLANET_RADIUS;
        x = 1.0 - sqrt(clamp(height / volume_top(r), 0.0, 1.0));
    } else {
        let range = volume_range(r);
        x = clamp(log(dist / range.x) / log(range.y / range.x), 0.0, 1.0);
    }
    return vec3<f32>(u, v, texel_coord(x, size.z));
}

// The atmosphere between the camera and a point `dist` away from it in the direction `dir`
fn aerial_perspective(
    inscattering: texture_3d<f32>,
    transmittance: texture_3d<f32>,
    volume_sampler: sampler,
    camera: vec3<f32>,
    dir: vec3<f32>,
    sun: vec3<f32>,
    dist: f32,
) -> Scattering {
    let uvw = volume_uvw(camera, dir, sun, dist, textureDimensions(inscattering));
    // Fade out towards the camera, closer than the nearest distance in the upper half. The
    // lower half starts at the camera.
    let r = volume_radius(length(camera));
    var fade = 1.0;
    if asin(clamp(dot(dir, normalize(camera)), -1.0, 1.0)) >= volume_elevations(r).x {
        fade = clamp(dist / volume_range(r).x, 0.0, 1.0);
    }

    var out: Scattering;
    out.inscattered = textureSampleLevel(inscattering, volume_sampler, uvw, 0.0).rgb * fade;
    let t = textureSampleLevel(transmittance, volume_sampler, uvw, 0.0).rgb;
    out.transmittance = mix(vec3<f32>(1.0), t, fade);
    return out;
}
