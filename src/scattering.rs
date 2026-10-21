//! The lookup tables of the atmosphere, computed on the GPU, as described in
//! `shaders/scattering.wgsl`, and read by the sky and the terrain through
//! `shaders/atmosphere.wgsl`.
//!
//! The work per frame is fixed: one pass over the scattering volume, whose size does not depend
//! on the view or the window, and a few texture lookups for each pixel of the sky and the
//! terrain. The other tables are only computed when the atmosphere changes.
//!
//! Those tables are also read back to the CPU when they change, as [`SkyLight`], to light
//! objects drawn with Bevy's own materials, such as the aircraft, with the same light.

use bevy::{
    asset::{RenderAssetUsages, embedded_asset, load_embedded_asset},
    core_pipeline::schedule::camera_driver,
    image::ImageSampler,
    prelude::*,
    render::{
        Render, RenderApp, RenderStartup, RenderSystems,
        extract_resource::{ExtractResource, ExtractResourcePlugin},
        gpu_readback::{Readback, ReadbackComplete},
        render_asset::RenderAssets,
        render_resource::{binding_types::*, *},
        renderer::{RenderContext, RenderDevice, RenderGraph, RenderQueue},
        texture::GpuImage,
    },
};

/// By distance to the top of the atmosphere and height.
const TRANSMITTANCE_SIZE: UVec2 = UVec2::new(256, 64);
/// By zenith cosine of the sun and height.
const MULTISCATTERING_SIZE: UVec2 = UVec2::new(32, 32);
/// By zenith cosine of the sun and height.
const IRRADIANCE_SIZE: UVec2 = UVec2::new(64, 16);
/// By azimuth from the sun, elevation and distance from the camera.
const VOLUME_SIZE: UVec3 = UVec3::new(128, 128, 64);

// The same as in `atmosphere.wgsl`
const PLANET_RADIUS: f32 = 6_371_000.0;
const ATMOSPHERE_RADIUS: f32 = 6_471_000.0;
const SUN_INTENSITY: f32 = 10.0;

pub struct ScatteringPlugin;

impl Plugin for ScatteringPlugin {
    fn build(&self, app: &mut App) {
        embedded_asset!(app, "shaders/scattering.wgsl");
        app.init_resource::<ScatteringLuts>()
            .init_resource::<ScatteringParams>()
            .init_resource::<SkyLight>()
            .add_systems(Update, read_back)
            .add_plugins((
                ExtractResourcePlugin::<ScatteringLuts>::default(),
                ExtractResourcePlugin::<ScatteringParams>::default(),
            ));

        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app
            .init_resource::<Computed>()
            .add_systems(RenderStartup, init_pipelines)
            .add_systems(
                Render,
                prepare_bind_groups.in_set(RenderSystems::PrepareBindGroups),
            )
            .add_systems(RenderGraph, compute.before(camera_driver));
    }
}

/// The tables, for the materials to read.
#[derive(Resource, Clone, ExtractResource)]
pub struct ScatteringLuts {
    pub transmittance: Handle<Image>,
    pub multiscattering: Handle<Image>,
    pub irradiance: Handle<Image>,
    pub aerial_inscattering: Handle<Image>,
    pub aerial_transmittance: Handle<Image>,
}

impl FromWorld for ScatteringLuts {
    fn from_world(world: &mut World) -> Self {
        let mut images = world.resource_mut::<Assets<Image>>();
        let mut add = |size: UVec3, dimension| {
            let size = Extent3d {
                width: size.x,
                height: size.y,
                depth_or_array_layers: size.z,
            };
            let mut image = Image::new_uninit(
                size,
                dimension,
                TextureFormat::Rgba16Float,
                RenderAssetUsages::RENDER_WORLD,
            );
            image.texture_descriptor.usage = TextureUsages::COPY_DST
                | TextureUsages::COPY_SRC
                | TextureUsages::STORAGE_BINDING
                | TextureUsages::TEXTURE_BINDING;
            image.sampler = ImageSampler::linear();
            images.add(image)
        };
        Self {
            transmittance: add(TRANSMITTANCE_SIZE.extend(1), TextureDimension::D2),
            multiscattering: add(MULTISCATTERING_SIZE.extend(1), TextureDimension::D2),
            irradiance: add(IRRADIANCE_SIZE.extend(1), TextureDimension::D2),
            aerial_inscattering: add(VOLUME_SIZE, TextureDimension::D3),
            aerial_transmittance: add(VOLUME_SIZE, TextureDimension::D3),
        }
    }
}

/// What the tables are computed for.
#[derive(Resource, Clone, Copy, Default, ExtractResource, ShaderType)]
pub struct ScatteringParams {
    /// Distance of the camera from the center of the planet.
    pub radius: f32,
    /// Cosine of the zenith angle of the sun at the camera.
    pub sun_mu: f32,
    /// Vertical optical depth of the aerosols at 550 nm, from sea level.
    pub aerosols: f32,
}

#[derive(Resource)]
struct ScatteringPipelines {
    transmittance: (BindGroupLayoutDescriptor, CachedComputePipelineId),
    multiscattering: (BindGroupLayoutDescriptor, CachedComputePipelineId),
    irradiance: (BindGroupLayoutDescriptor, CachedComputePipelineId),
    volume: (BindGroupLayoutDescriptor, CachedComputePipelineId),
    sampler: Sampler,
}

fn init_pipelines(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    pipeline_cache: Res<PipelineCache>,
    render_device: Res<RenderDevice>,
) {
    let shader = load_embedded_asset!(asset_server.as_ref(), "shaders/scattering.wgsl");
    let params = || uniform_buffer::<ScatteringParams>(false);
    let lut_sampler = || sampler(SamplerBindingType::Filtering);
    let lut = || texture_2d(TextureSampleType::Float { filterable: true });
    let out_2d =
        || texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly);
    let out_3d =
        || texture_storage_3d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly);

    // The bindings are numbered as in the shader, where each entry point uses some of them
    let pipeline = |entry_point: &'static str, entries: &[BindGroupLayoutEntry]| {
        let layout = BindGroupLayoutDescriptor::new(entry_point, entries);
        let id = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
            label: Some(entry_point.into()),
            layout: vec![layout.clone()],
            shader: shader.clone(),
            entry_point: Some(entry_point.into()),
            ..default()
        });
        (layout, id)
    };
    let stages = ShaderStages::COMPUTE;
    commands.insert_resource(ScatteringPipelines {
        transmittance: pipeline(
            "transmittance",
            &BindGroupLayoutEntries::with_indices(stages, ((0, params()), (4, out_2d()))),
        ),
        multiscattering: pipeline(
            "multiscattering_table",
            &BindGroupLayoutEntries::with_indices(
                stages,
                ((0, params()), (1, lut_sampler()), (2, lut()), (5, out_2d())),
            ),
        ),
        irradiance: pipeline(
            "irradiance_table",
            &BindGroupLayoutEntries::with_indices(
                stages,
                (
                    (0, params()),
                    (1, lut_sampler()),
                    (2, lut()),
                    (3, lut()),
                    (6, out_2d()),
                ),
            ),
        ),
        volume: pipeline(
            "volume",
            &BindGroupLayoutEntries::with_indices(
                stages,
                (
                    (0, params()),
                    (1, lut_sampler()),
                    (2, lut()),
                    (3, lut()),
                    (7, out_3d()),
                    (8, out_3d()),
                ),
            ),
        ),
        sampler: render_device.create_sampler(&SamplerDescriptor {
            label: Some("scattering_lut_sampler"),
            mag_filter: FilterMode::Linear,
            min_filter: FilterMode::Linear,
            ..default()
        }),
    });
}

#[derive(Resource)]
struct ScatteringBindGroups {
    transmittance: BindGroup,
    multiscattering: BindGroup,
    irradiance: BindGroup,
    volume: BindGroup,
}

fn prepare_bind_groups(
    mut commands: Commands,
    mut buffer: Local<UniformBuffer<ScatteringParams>>,
    pipelines: Res<ScatteringPipelines>,
    pipeline_cache: Res<PipelineCache>,
    luts: Res<ScatteringLuts>,
    params: Res<ScatteringParams>,
    images: Res<RenderAssets<GpuImage>>,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
) {
    let (
        Some(transmittance),
        Some(multiscattering),
        Some(irradiance),
        Some(inscattering),
        Some(aerial_transmittance),
    ) = (
        images.get(&luts.transmittance),
        images.get(&luts.multiscattering),
        images.get(&luts.irradiance),
        images.get(&luts.aerial_inscattering),
        images.get(&luts.aerial_transmittance),
    )
    else {
        return;
    };

    buffer.set(*params);
    buffer.write_buffer(&render_device, &render_queue);
    let sampler = &pipelines.sampler;
    let layout = |(layout, _): &(BindGroupLayoutDescriptor, _)| {
        pipeline_cache.get_bind_group_layout(layout)
    };

    commands.insert_resource(ScatteringBindGroups {
        transmittance: render_device.create_bind_group(
            "transmittance",
            &layout(&pipelines.transmittance),
            &BindGroupEntries::with_indices(((0, &*buffer), (4, &transmittance.texture_view))),
        ),
        multiscattering: render_device.create_bind_group(
            "multiscattering_table",
            &layout(&pipelines.multiscattering),
            &BindGroupEntries::with_indices((
                (0, &*buffer),
                (1, sampler),
                (2, &transmittance.texture_view),
                (5, &multiscattering.texture_view),
            )),
        ),
        irradiance: render_device.create_bind_group(
            "irradiance_table",
            &layout(&pipelines.irradiance),
            &BindGroupEntries::with_indices((
                (0, &*buffer),
                (1, sampler),
                (2, &transmittance.texture_view),
                (3, &multiscattering.texture_view),
                (6, &irradiance.texture_view),
            )),
        ),
        volume: render_device.create_bind_group(
            "volume",
            &layout(&pipelines.volume),
            &BindGroupEntries::with_indices((
                (0, &*buffer),
                (1, sampler),
                (2, &transmittance.texture_view),
                (3, &multiscattering.texture_view),
                (7, &inscattering.texture_view),
                (8, &aerial_transmittance.texture_view),
            )),
        ),
    });
}

/// The aerosols the tables which only depend on the atmosphere were last computed for.
#[derive(Resource, Default)]
struct Computed {
    aerosols: Option<f32>,
}

fn compute(
    mut render_context: RenderContext,
    bind_groups: Option<Res<ScatteringBindGroups>>,
    pipelines: Res<ScatteringPipelines>,
    pipeline_cache: Res<PipelineCache>,
    params: Res<ScatteringParams>,
    mut computed: ResMut<Computed>,
) {
    let Some(bind_groups) = bind_groups else {
        return;
    };
    let get = |(_, id): &(_, CachedComputePipelineId)| pipeline_cache.get_compute_pipeline(*id);
    let (Some(transmittance), Some(multiscattering), Some(irradiance), Some(volume)) = (
        get(&pipelines.transmittance),
        get(&pipelines.multiscattering),
        get(&pipelines.irradiance),
        get(&pipelines.volume),
    ) else {
        return;
    };

    let mut pass = render_context
        .command_encoder()
        .begin_compute_pass(&ComputePassDescriptor {
            label: Some("scattering"),
            timestamp_writes: None,
        });

    if computed.aerosols != Some(params.aerosols) {
        pass.set_pipeline(transmittance);
        pass.set_bind_group(0, &bind_groups.transmittance, &[]);
        pass.dispatch_workgroups(
            TRANSMITTANCE_SIZE.x.div_ceil(8),
            TRANSMITTANCE_SIZE.y.div_ceil(8),
            1,
        );

        // A workgroup per texel
        pass.set_pipeline(multiscattering);
        pass.set_bind_group(0, &bind_groups.multiscattering, &[]);
        pass.dispatch_workgroups(MULTISCATTERING_SIZE.x, MULTISCATTERING_SIZE.y, 1);

        pass.set_pipeline(irradiance);
        pass.set_bind_group(0, &bind_groups.irradiance, &[]);
        pass.dispatch_workgroups(IRRADIANCE_SIZE.x, IRRADIANCE_SIZE.y, 1);

        computed.aerosols = Some(params.aerosols);
    }

    pass.set_pipeline(volume);
    pass.set_bind_group(0, &bind_groups.volume, &[]);
    pass.dispatch_workgroups(VOLUME_SIZE.x.div_ceil(8), VOLUME_SIZE.y.div_ceil(8), 1);
}

/// The light of the sun and the sky anywhere in the atmosphere, from copies of the
/// transmittance and sky irradiance tables.
#[derive(Resource, Default)]
pub struct SkyLight {
    transmittance: Option<Table>,
    irradiance: Option<Table>,
}

impl SkyLight {
    /// The sunlight at `position`, relative to the center of the planet, with the sun towards
    /// `sun`, and the light of the sky on a horizontal surface there, in the units the shaders
    /// output. `None` until the tables have been read back.
    pub fn at(&self, position: Vec3, sun: Vec3) -> Option<(Vec3, Vec3)> {
        let (transmittance, irradiance) = (self.transmittance.as_ref()?, self.irradiance.as_ref()?);
        let r = position.length();
        let mu_s = position.dot(sun) / r;

        // As `sun_transmittance` in `atmosphere.wgsl`
        let ratio = (PLANET_RADIUS / r).min(1.0);
        let horizon = -(1.0 - ratio * ratio).max(0.0).sqrt();
        let x = ((mu_s - horizon + 0.01) / 0.02).clamp(0.0, 1.0);
        let visible = x * x * (3.0 - 2.0 * x);
        let uv = transmittance_uv(r, mu_s.max(horizon), transmittance.size);
        let sunlight = transmittance.sample(uv) * visible * SUN_INTENSITY;

        let sky = irradiance.sample(sun_height_uv(r, mu_s, irradiance.size));
        Some((sunlight, sky))
    }
}

struct Table {
    size: UVec2,
    texels: Vec<Vec3>,
    /// The aerosols the table was computed for, which the shader stores in the alpha channel.
    aerosols: f32,
}

impl Table {
    /// From the texels of an `Rgba16Float` texture, whose rows may be padded.
    fn decode(data: &[u8], size: UVec2) -> Option<Self> {
        let row = data.len() / size.y as usize;
        if row < size.x as usize * 8 {
            return None;
        }
        let channel = |at: usize| half::f16::from_le_bytes([data[at], data[at + 1]]).to_f32();
        let texel = |x: u32, y: u32| y as usize * row + x as usize * 8;
        let texels = (0..size.y)
            .flat_map(|y| (0..size.x).map(move |x| (x, y)))
            .map(|(x, y)| {
                let at = texel(x, y);
                Vec3::new(channel(at), channel(at + 2), channel(at + 4))
            })
            .collect();
        Some(Self {
            size,
            texels,
            aerosols: channel(texel(0, 0) + 6),
        })
    }

    /// Bilinearly, as the GPU does.
    fn sample(&self, uv: Vec2) -> Vec3 {
        let p = (uv * self.size.as_vec2() - 0.5).clamp(Vec2::ZERO, (self.size - 1).as_vec2());
        let i = p.floor().as_uvec2().min(self.size - 2);
        let f = p - i.as_vec2();
        let at = |x: u32, y: u32| self.texels[(y * self.size.x + x) as usize];
        let top = at(i.x, i.y).lerp(at(i.x + 1, i.y), f.x);
        let bottom = at(i.x, i.y + 1).lerp(at(i.x + 1, i.y + 1), f.x);
        top.lerp(bottom, f.y)
    }

    fn is_for(&self, aerosols: f32) -> bool {
        // Within the precision of the alpha channel
        (self.aerosols - aerosols).abs() <= aerosols * 2e-3
    }
}

// As in `atmosphere.wgsl`

fn texel_coord(x: f32, size: u32) -> f32 {
    (0.5 + x * (size - 1) as f32) / size as f32
}

fn transmittance_uv(r: f32, mu: f32, size: UVec2) -> Vec2 {
    let h = ((ATMOSPHERE_RADIUS - PLANET_RADIUS) * (ATMOSPHERE_RADIUS + PLANET_RADIUS)).sqrt();
    let rho = ((r - PLANET_RADIUS) * (r + PLANET_RADIUS)).max(0.0).sqrt();
    let discriminant = (ATMOSPHERE_RADIUS - r) * (ATMOSPHERE_RADIUS + r) + r * r * mu * mu;
    let d = (-r * mu + discriminant.max(0.0).sqrt()).max(0.0);
    let d_min = ATMOSPHERE_RADIUS - r;
    let d_max = rho + h;
    let x_mu = ((d - d_min) / (d_max - d_min)).clamp(0.0, 1.0);
    let x_r = (rho / h).clamp(0.0, 1.0);
    Vec2::new(texel_coord(x_mu, size.x), texel_coord(x_r, size.y))
}

fn sun_height_uv(r: f32, mu_s: f32, size: UVec2) -> Vec2 {
    let x_r = ((r - PLANET_RADIUS) / (ATMOSPHERE_RADIUS - PLANET_RADIUS)).clamp(0.0, 1.0);
    Vec2::new(
        texel_coord(mu_s * 0.5 + 0.5, size.x),
        texel_coord(x_r, size.y),
    )
}

#[derive(Component)]
struct Reading;

/// Reads the tables back while they are not those for the current aerosols. They are read
/// every frame until they are, as the GPU may not have computed them yet.
fn read_back(
    mut commands: Commands,
    luts: Res<ScatteringLuts>,
    params: Res<ScatteringParams>,
    sky_light: Res<SkyLight>,
    reading: Query<(), With<Reading>>,
) {
    let current = |table: &Option<Table>| table.as_ref().is_some_and(|t| t.is_for(params.aerosols));
    if !reading.is_empty() || (current(&sky_light.transmittance) && current(&sky_light.irradiance))
    {
        return;
    }

    for (image, size, irradiance) in [
        (&luts.transmittance, TRANSMITTANCE_SIZE, false),
        (&luts.irradiance, IRRADIANCE_SIZE, true),
    ] {
        commands.spawn((Readback::texture(image.clone()), Reading)).observe(
            move |event: On<ReadbackComplete>,
                  mut commands: Commands,
                  params: Res<ScatteringParams>,
                  mut sky_light: ResMut<SkyLight>| {
                // Not computed at all yet
                let Some(table) = Table::decode(&event.data, size).filter(|t| t.aerosols > 0.0)
                else {
                    return;
                };
                if table.is_for(params.aerosols) {
                    commands.entity(event.entity).despawn();
                }
                match irradiance {
                    true => sky_light.irradiance = Some(table),
                    false => sky_light.transmittance = Some(table),
                }
            },
        );
    }
}
