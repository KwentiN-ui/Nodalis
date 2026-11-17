//! The time loop on the graphics card (feature `gpu`): the kernels of `gpu.wgsl`, the same
//! steps as [`super::run_cpu`] in single precision. The card runs time steps until a frame
//! or a point of the history is due (or the material is gone), then waits; the processor
//! reads the sums, writes the frame and the history and lets it go on. In between the
//! processor only reads the small control block after each batch of steps, for the progress
//! and to stop when asked.
//!
//! When the card is used: `PLXS_GPU=off` never, `PLXS_GPU=on` always (any adapter, software
//! renderers included: tests), otherwise on a discrete card from [`GPU_FROM`] particles,
//! as the setting "Use graphics card" of the application says.

use std::time::Instant;

use bytemuck::{Pod, Zeroable};
use plxs_core::particles::{ParticleFrame, ParticleHistory, Side, WallLaw};
use plxs_gpu::{Double, Gpu, Options, wgpu};
use plxs_linalg::amg::GpuMode;

use super::{HISTORY_POINTS, Load, Particle, Setup, Status, Tally, frame};
use crate::law::{Kind, PointState};
use crate::math::{IDENTITY, add, norm, scale};

/// From this many particles the automatic mode takes a discrete graphics card.
const GPU_FROM: usize = 20_000;

const SOURCE: &str = include_str!("gpu.wgsl");
const GROUP: usize = 64;
const SCAN: usize = 256;
const TILE: usize = 4;
const GRID_TILES: usize = 8;
const LOAD: usize = 8;
const SUMS: usize = 8;
/// The slots of the loads in a bit mask: walls, six sides, the loss.
const MAX_SLOTS: usize = 32;
/// Time steps per submission at most.
const BATCH: usize = 256;
/// Seconds a submission of time steps should take at most.
const SUBMISSION: f64 = 0.25;
/// The storage buffers of the largest kernel (`g2p`).
const STORAGE_BUFFERS: u32 = 13;

/// The speed the card reports for a particle whose motion is no number any more.
const UNSTABLE: f32 = 1e38;

const FRAME: u32 = 1;
const HISTORY: u32 = 2;
const BROKEN: u32 = 4;
const EMPTY: u32 = 8;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
struct Params {
    n: [u32; 4],
    tiles: [u32; 4],
    periodic: [u32; 4],
    lo: [u32; 4],
    hi: [u32; 4],
    origin: [f32; 4],
    box_lo: [f32; 4],
    box_hi: [f32; 4],
    limits: [f32; 4],
    sides0: [u32; 4],
    sides1: [u32; 4],
    friction0: [f32; 4],
    friction1: [f32; 4],
    volume: f32,
    slots: u32,
    particle_groups: u32,
    grid_groups: u32,
    offsets: [u32; 4],
    scan_levels: [u32; 4],
    scan_off: [u32; 4],
    scan_len: [u32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
struct Control {
    running: u32,
    reason: u32,
    steps: u32,
    hit: u32,
    dt: f32,
    tl: f32,
    dt_max: f32,
    dt_min: f32,
    frame_at: f32,
    history: f32,
    base: f32,
    max_speed: f32,
    top_speed: f32,
    kinetic: f32,
    potential: f32,
    stored: f32,
    dissipated: [f32; 2],
    loss: [f32; 2],
    work: [f32; 2],
    out_mass: [f32; 2],
    out_energy: [f32; 2],
    samples: u32,
    gone: u32,
    pad: [u32; 2],
}

/// A particle as the card holds it (`Particle` in `gpu.wgsl`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
struct CardParticle {
    x: [f32; 4],
    v: [f32; 4],
    c: [[f32; 4]; 3],
    b: [f32; 4],
    tau: [f32; 4],
    bt: [f32; 4],
    meta: [u32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
struct CardLaw {
    a: [f32; 4],
    b: [f32; 4],
    c: [f32; 4],
    d: [u32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
struct CardWall {
    origin: [f32; 4],
    dims: [u32; 4],
    center: [f32; 4],
    velocity: [f32; 4],
    angular: [f32; 4],
    extra: [u32; 4],
}

/// Bytes of a record and a node on the card (`Record`, `Node`) and of a pose.
const RECORD: u64 = 128;
const NODE: u64 = 64;
const POSE: u64 = 96;

/// One dispatch of a time step: its words in the indirect arguments.
struct Pass {
    pipeline: wgpu::ComputePipeline,
    bind: wgpu::BindGroup,
    at: u64,
}

/// The bindings of `gpu.wgsl` by number.
const UNIFORM: u32 = 0;

/// Whether binding `b` of `gpu.wgsl` is read only.
fn read_only(b: u32) -> bool {
    matches!(b, 12 | 13 | 15 | 16 | 20)
}

/// The particle analysis on an open card.
pub(super) struct Card {
    gpu: Gpu,
    params: Params,
    ctl: wgpu::Buffer,
    particles: wgpu::Buffer,
    partials: wgpu::Buffer,
    loads: wgpu::Buffer,
    args: wgpu::Buffer,
    control: (wgpu::ComputePipeline, wgpu::BindGroup),
    body_sums: (wgpu::ComputePipeline, wgpu::BindGroup, (u32, u32)),
    passes: Vec<Pass>,
    read_ctl: wgpu::Buffer,
    read_loads: wgpu::Buffer,
    read_bodies: wgpu::Buffer,
    read_particles: wgpu::Buffer,
    /// Particles, slots of the loads, bodies, workgroups of the particles.
    count: usize,
    slots: usize,
    bodies: usize,
    particle_groups: usize,
    /// The temperatures on the card are relative to this one.
    t_ref: f64,
    /// The mass of a particle of each body, and of all particles at the start.
    body_mass: Vec<f64>,
    mass: f64,
    /// Node 0 of the grid in the model.
    origin: [f64; 3],
    /// Per particle the processor's copy, for what the card does not hold.
    template: Vec<Particle>,
}

impl Card {
    /// The card for the run, if the mode and the card allow; `notes` says where the time
    /// steps run and, where a card was wanted, why not on it.
    pub(super) fn open_for(
        setup: &Setup<'_>,
        particles: &[Particle],
        notes: &mut Vec<String>,
    ) -> Option<Self> {
        let worth = particles.len() >= GPU_FROM;
        if setup.model.domain.space != plxs_core::particles::Space::Solid {
            if worth {
                notes.push(
                    "Particles computed on the processor: the card does not compute 2D models \
                     yet"
                    .into(),
                );
            }
            return None;
        }
        if !setup.feeds.is_empty() {
            if worth {
                notes.push(
                    "Particles computed on the processor: the card does not compute inlets yet"
                        .into(),
                );
            }
            return None;
        }
        if particles.iter().any(|p| p.grains[0] > 0.0) {
            if worth {
                notes.push(
                    "Particles computed on the processor: the card does not compute the \
                     segregation of grain mixtures yet"
                        .into(),
                );
            }
            return None;
        }
        let any = match GpuMode::from_environment() {
            GpuMode::Off => {
                if worth {
                    notes.push(
                        "Particles computed on the processor: the graphics card is switched \
                         off in the settings or by PLXS_GPU"
                            .into(),
                    );
                }
                return None;
            }
            GpuMode::Automatic if !worth => return None,
            GpuMode::Automatic => false,
            GpuMode::On => true,
        };
        match Self::new(setup, particles, any) {
            Ok(card) => {
                notes.push(format!(
                    "Particles computed on {} (single precision)",
                    card.gpu.name
                ));
                Some(card)
            }
            Err(e) => {
                notes.push(format!("Particles computed on the processor: {e}"));
                None
            }
        }
    }

    fn new(setup: &Setup<'_>, particles: &[Particle], any: bool) -> Result<Self, String> {
        let model = setup.model;
        let grid = setup.grid;
        let walls = setup.walls;
        let slots = walls.len() + 7;
        if slots > MAX_SLOTS {
            return Err(format!(
                "more than {} walls on the graphics card",
                MAX_SLOTS - 7
            ));
        }
        let gpu = Gpu::open(Options {
            label: "plxs particles",
            any_adapter: any,
            double: Double::IfAvailable,
            timestamps: false,
            memory_share: true,
        })?;
        if gpu.limits.max_storage_buffers_per_shader_stage < STORAGE_BUFFERS {
            return Err(format!(
                "{} binds only {} storage buffers per shader",
                gpu.name, gpu.limits.max_storage_buffers_per_shader_stage
            ));
        }
        let count = particles.len();
        let cells = grid.count();
        let heat = model.heat.is_some();
        let origin = grid.origin;
        // The reference of the temperatures: their mean at the start.
        let t_ref = if heat {
            particles.iter().map(|p| p.temperature).sum::<f64>() / count.max(1) as f64
        } else {
            0.0
        };
        let rel = |x: [f64; 3]| -> [f32; 3] { std::array::from_fn(|a| (x[a] - origin[a]) as f32) };

        // The tables: flow factors, amplitudes, distance fields.
        let mut tables: Vec<f32> = Vec::new();
        let mut laws = Vec::new();
        let volume = particles[0].volume;
        for (m, law) in setup.laws.iter().enumerate() {
            let material = &model.materials[m];
            let thermal = model.heat.and(material.thermal);
            let start = tables.len() as u32;
            for p in setup.factors[m] {
                tables.extend([(p[0] - t_ref) as f32, p[1] as f32]);
            }
            let (kind, values, grain) = match law.kind {
                Kind::Elastic => (0.0, [0.0; 3], 0.0),
                Kind::ViscoPlastic {
                    yield_stress,
                    consistency,
                    exponent,
                } => (1.0, [yield_stress, consistency, exponent], 0.0),
                Kind::Fluid { viscosity } => (2.0, [viscosity, 0.0, 0.0], 0.0),
                Kind::Granular {
                    friction,
                    friction_limit,
                    grain,
                    inertial,
                    ..
                } => (3.0, [friction, friction_limit, inertial], grain),
            };
            laws.push(CardLaw {
                a: [law.density as f32, law.bulk as f32, law.shear as f32, kind],
                b: [
                    values[0] as f32,
                    values[1] as f32,
                    values[2] as f32,
                    thermal.map_or(0.0, |t| t.specific_heat * law.density) as f32,
                ],
                c: [
                    thermal.map_or(0.0, |t| t.conductivity) as f32,
                    (law.density * volume) as f32,
                    grain as f32,
                    0.0,
                ],
                d: [start, setup.factors[m].len() as u32, 0, 0],
            });
        }
        let mut card_walls = Vec::new();
        for (w, wall) in walls.iter().enumerate() {
            let start = tables.len() as u32;
            for p in &wall.motion.amplitude {
                tables.extend([p[0] as f32, p[1] as f32]);
            }
            let field_start = tables.len() as u32;
            tables.extend_from_slice(wall.field.values());
            let (law, friction) = match wall.law {
                WallLaw::Stick => (1.0, 0.0),
                WallLaw::Slip => (2.0, 0.0),
                WallLaw::Coulomb { friction } => (3.0, friction as f32),
            };
            let lattice = rel(wall.field.origin);
            let center = rel(wall.motion.center);
            let temperature = setup.wall_temperature[w];
            card_walls.push(CardWall {
                origin: [
                    lattice[0],
                    lattice[1],
                    lattice[2],
                    wall.field.spacing as f32,
                ],
                dims: [
                    wall.field.dims[0] as u32,
                    wall.field.dims[1] as u32,
                    wall.field.dims[2] as u32,
                    field_start,
                ],
                center: [center[0], center[1], center[2], law],
                velocity: [
                    wall.motion.velocity[0] as f32,
                    wall.motion.velocity[1] as f32,
                    wall.motion.velocity[2] as f32,
                    friction,
                ],
                angular: [
                    wall.motion.angular[0] as f32,
                    wall.motion.angular[1] as f32,
                    wall.motion.angular[2] as f32,
                    temperature.map_or(0.0, |t| (t - t_ref) as f32),
                ],
                extra: [
                    start,
                    wall.motion.amplitude.len() as u32,
                    u32::from(temperature.is_some()),
                    0,
                ],
            });
        }
        // An empty array is still bound: one wall that no count reaches.
        if card_walls.is_empty() {
            card_walls.push(CardWall::default());
        }
        let bodies: Vec<[f32; 4]> = (model.bodies.iter())
            .map(|b| {
                let g = add(model.gravity, b.gravity);
                [g[0] as f32, g[1] as f32, g[2] as f32, 0.0]
            })
            .collect();
        let body_mass: Vec<f64> = (model.bodies.iter())
            .map(|b| {
                setup
                    .laws
                    .get(b.material)
                    .map_or(0.0, |l| l.density * volume)
            })
            .collect();
        let card_particles: Vec<CardParticle> = particles
            .iter()
            .map(|p| {
                let x = rel(p.x);
                let t = p.temperature - t_ref;
                let hi = t as f32;
                let lo = (t - f64::from(hi)) as f32;
                let b = p.state.b;
                CardParticle {
                    x: [x[0], x[1], x[2], hi],
                    v: [p.v[0] as f32, p.v[1] as f32, p.v[2] as f32, lo],
                    c: std::array::from_fn(|r| {
                        let w = [p.state.j, p.state.plastic, f64::from(p.shear_rate)][r];
                        [
                            p.c[r][0] as f32,
                            p.c[r][1] as f32,
                            p.c[r][2] as f32,
                            w as f32,
                        ]
                    }),
                    b: [
                        b[0][0] as f32,
                        b[1][1] as f32,
                        b[2][2] as f32,
                        b[0][1] as f32,
                    ],
                    tau: [
                        p.tau[0][0] as f32,
                        p.tau[1][1] as f32,
                        p.tau[2][2] as f32,
                        p.tau[0][1] as f32,
                    ],
                    bt: [
                        b[1][2] as f32,
                        b[0][2] as f32,
                        p.tau[1][2] as f32,
                        p.tau[0][2] as f32,
                    ],
                    meta: [
                        p.id,
                        u32::from(p.material) | u32::from(p.body) << 16,
                        u32::from(p.gone),
                        0,
                    ],
                }
            })
            .collect();

        // The sizes.
        let tiles: [usize; 3] = std::array::from_fn(|a| grid.n[a].div_ceil(TILE));
        let tile_count = tiles[0] * tiles[1] * tiles[2];
        let grid_groups = tile_count.div_ceil(GRID_TILES);
        let particle_groups = count.div_ceil(GROUP);
        let mut scan_len = vec![cells + 1];
        while *scan_len.last().unwrap_or(&0) > SCAN {
            let next = scan_len[scan_len.len() - 1].div_ceil(SCAN);
            scan_len.push(next);
        }
        if scan_len.len() > 4 {
            return Err("the background grid is too large for the graphics card".into());
        }
        let mut scan_off = vec![0usize];
        for l in 1..scan_len.len() {
            scan_off.push(scan_off[l - 1] + scan_len[l - 1]);
        }
        let scan_total = scan_off[scan_off.len() - 1] + scan_len[scan_len.len() - 1];
        let loads_at = grid_groups * slots * LOAD;
        let body_rows = particle_groups * bodies.len().max(1) * 3;
        let loads_at = loads_at.max(body_rows);
        let sums_at = loads_at + slots * LOAD;
        let partial_words = sums_at + particle_groups * SUMS;
        let side = |s: usize| -> (u32, f32) {
            match model.domain.sides[s] {
                Side::Open => (0, 0.0),
                Side::Wall(WallLaw::Stick) => (1, 0.0),
                Side::Wall(WallLaw::Slip) => (2, 0.0),
                Side::Wall(WallLaw::Coulomb { friction }) => (3, friction as f32),
            }
        };
        let finite = |x: f64| if x.is_finite() { x as f32 } else { f32::MAX };
        let box_lo = rel(model.domain.min);
        let box_hi = rel(model.domain.max);
        let mut levels = [0u32; 4];
        let mut offs = [0u32; 4];
        for l in 0..scan_len.len() {
            levels[l] = scan_len[l] as u32;
            offs[l] = scan_off[l] as u32;
        }
        let params = Params {
            n: [
                grid.n[0] as u32,
                grid.n[1] as u32,
                grid.n[2] as u32,
                cells as u32,
            ],
            tiles: [
                tiles[0] as u32,
                tiles[1] as u32,
                tiles[2] as u32,
                tile_count as u32,
            ],
            periodic: [
                u32::from(grid.periodic[0]),
                u32::from(grid.periodic[1]),
                u32::from(grid.periodic[2]),
                count as u32,
            ],
            lo: [
                grid.lo[0] as u32,
                grid.lo[1] as u32,
                grid.lo[2] as u32,
                walls.len() as u32,
            ],
            hi: [
                grid.hi[0] as u32,
                grid.hi[1] as u32,
                grid.hi[2] as u32,
                u32::from(heat),
            ],
            origin: [
                origin[0] as f32,
                origin[1] as f32,
                origin[2] as f32,
                grid.h as f32,
            ],
            box_lo: [
                box_lo[0],
                box_lo[1],
                box_lo[2],
                model.control.courant as f32,
            ],
            box_hi: [box_hi[0], box_hi[1], box_hi[2], setup.wave as f32],
            limits: [
                finite(setup.viscosity_limit),
                finite(setup.heat_limit),
                model.heat.map_or(0.0, |h| h.wall_transfer) as f32,
                if model.control.smooth_volume {
                    1.0
                } else {
                    0.0
                },
            ],
            sides0: [side(0).0, side(1).0, side(2).0, side(3).0],
            sides1: [side(4).0, side(5).0, 0, 0],
            friction0: [side(0).1, side(1).1, side(2).1, side(3).1],
            friction1: [side(4).1, side(5).1, 0.0, 0.0],
            volume: volume as f32,
            slots: slots as u32,
            particle_groups: particle_groups as u32,
            grid_groups: grid_groups as u32,
            offsets: [loads_at as u32, sums_at as u32, 0, 0],
            scan_levels: [scan_len.len() as u32, 0, 0, 0],
            scan_off: offs,
            scan_len: levels,
        };

        // The memory, then the buffers.
        let words = |n: usize| (n * 4) as u64;
        let bytes = [
            count as u64 * (std::mem::size_of::<CardParticle>() as u64 + RECORD + 8),
            words(2 * (cells + 1)) + words(scan_total),
            cells as u64 * (NODE + 16 + if heat { 8 } else { 0 }),
            words(tables.len() + partial_words),
        ];
        let total: u64 = bytes.iter().sum();
        gpu.reserve(total)?;
        let largest = (count as u64 * std::mem::size_of::<CardParticle>() as u64)
            .max(cells as u64 * NODE)
            .max(count as u64 * RECORD);
        if !gpu.fits(largest) {
            return Err(format!(
                "{} binds at most {:.0} MB in one buffer, the particles need {:.0} MB",
                gpu.name,
                gpu.limits.max_storage_buffer_binding_size as f64 / 1048576.0,
                largest as f64 / 1048576.0
            ));
        }
        let built = gpu.validated(|| Buffers {
            params: gpu.uniform("particle params", bytemuck::bytes_of(&params)),
            ctl: gpu.storage("particle control", std::mem::size_of::<Control>() as u64),
            particles: gpu.init("particles", bytemuck::cast_slice(&card_particles)),
            records: gpu.storage("particle records", count as u64 * RECORD),
            keys: gpu.storage("particle keys", words(count)),
            counts: gpu.init("cell counts", &vec![0u8; (cells + 1) * 4]),
            fill: gpu.init("cell fill", &vec![0u8; (cells + 1) * 4]),
            scan: gpu.storage("cell starts", words(scan_total)),
            order: gpu.storage("particle order", words(count)),
            nodes: gpu.storage("grid nodes", cells as u64 * NODE),
            values: gpu.storage("grid values", cells as u64 * 16),
            heat: gpu.storage("grid heat", if heat { cells as u64 * 8 } else { 16 }),
            laws: gpu.init("particle laws", bytemuck::cast_slice(&laws)),
            walls: gpu.init("particle walls", bytemuck::cast_slice(&card_walls)),
            poses: gpu.storage("wall poses", (walls.len() as u64 * POSE).max(POSE)),
            tables: gpu.init("particle tables", bytemuck::cast_slice(&tables)),
            bodies: gpu.init("particle bodies", bytemuck::cast_slice(&bodies)),
            partials: gpu.storage("particle partials", words(partial_words)),
            loads: gpu.storage("wall loads", words(slots * LOAD)),
        })?;
        if let Some(e) = gpu.take_error() {
            return Err(e);
        }

        // The passes of a step, in order, with their dispatches.
        let mut plan: Vec<u32> = Vec::new();
        let mut steps: Vec<Planned> = Vec::new();
        let particle_threads = gpu.fold(count, GROUP);
        steps.push(("key", vec![0, 2, 4, 5], particle_threads, None));
        for l in 0..scan_len.len() {
            let groups = gpu.fold(scan_len[l], SCAN);
            steps.push(("scan_local", vec![0, 5, 7], groups, Some(l)));
        }
        for l in (0..scan_len.len().saturating_sub(1)).rev() {
            let groups = gpu.fold(scan_len[l], SCAN);
            steps.push(("scan_add", vec![0, 7], groups, Some(l)));
        }
        steps.push(("scatter", vec![0, 4, 6, 7, 8], particle_threads, None));
        steps.push(("cells", vec![0, 5, 6, 7, 8], gpu.fold(cells, GROUP), None));
        steps.push((
            "record",
            vec![0, 1, 2, 3, 7, 8, 12, 16],
            particle_threads,
            None,
        ));
        let tile_groups = gpu.fold(tile_count * GROUP, GROUP);
        steps.push(("gather", vec![0, 3, 7, 9], tile_groups, None));
        steps.push((
            "grid",
            vec![0, 1, 9, 10, 11, 13, 14, 15, 17],
            gpu.fold(grid_groups * GROUP, GROUP),
            None,
        ));
        steps.push(("reduce_loads", vec![0, 17], (slots as u32, 1), None));
        if heat {
            steps.push(("conduct", vec![0, 1, 9, 11], tile_groups, None));
        }
        steps.push((
            "g2p",
            vec![0, 1, 2, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17],
            particle_threads,
            None,
        ));
        steps.push(("finish", vec![0, 1, 17, 18], (1, 1), None));

        let args = gpu.buffer(&wgpu::BufferDescriptor {
            label: Some("particle dispatches"),
            size: (12 * steps.len() as u64).max(16),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::INDIRECT
                | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let resource = |b: u32| -> wgpu::BindingResource<'_> {
            let buffer = match b {
                0 => &built.params,
                1 => &built.ctl,
                2 => &built.particles,
                3 => &built.records,
                4 => &built.keys,
                5 => &built.counts,
                6 => &built.fill,
                7 => &built.scan,
                8 => &built.order,
                9 => &built.nodes,
                10 => &built.values,
                11 => &built.heat,
                12 => &built.laws,
                13 => &built.walls,
                14 => &built.poses,
                15 => &built.tables,
                16 => &built.bodies,
                17 => &built.partials,
                18 => &built.loads,
                _ => &args,
            };
            buffer.as_entire_binding()
        };
        let made = gpu.validated(|| {
            let make = |entry: &str,
                        bindings: &[u32],
                        level: Option<usize>,
                        plan_buffer: Option<&wgpu::Buffer>| {
                let layout = layout(&gpu, entry, bindings);
                let constants: Vec<(&str, f64)> =
                    level.map(|l| ("LEVEL", l as f64)).into_iter().collect();
                let pipeline = gpu.pipeline(SOURCE, entry, &layout, &constants);
                let entries: Vec<wgpu::BindGroupEntry> = bindings
                    .iter()
                    .map(|&b| wgpu::BindGroupEntry {
                        binding: b,
                        resource: match (b, plan_buffer) {
                            (20, Some(p)) => p.as_entire_binding(),
                            _ => resource(b),
                        },
                    })
                    .collect();
                let bind = gpu.device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some(entry),
                    layout: &layout,
                    entries: &entries,
                });
                (pipeline, bind)
            };
            let mut passes = Vec::new();
            for (k, (entry, bindings, groups, level)) in steps.iter().enumerate() {
                let (pipeline, bind) = make(entry, bindings, *level, None);
                plan.extend([groups.0, groups.1, 1]);
                passes.push(Pass {
                    pipeline,
                    bind,
                    at: 12 * k as u64,
                });
            }
            let plan_buffer = gpu.init("particle plan", bytemuck::cast_slice(&plan));
            let control = make(
                "control",
                &[0, 1, 13, 14, 15, 19, 20],
                None,
                Some(&plan_buffer),
            );
            let body_sums = make("body_sums", &[0, 2, 12, 16, 17], None, None);
            (passes, control, body_sums)
        })?;
        let (passes, control, body_sums) = made;
        if let Some(e) = gpu.take_error() {
            return Err(e);
        }
        let read_ctl = gpu.staging(
            "particle control read",
            std::mem::size_of::<Control>() as u64,
        );
        let read_loads = gpu.staging("wall loads read", words(slots * LOAD));
        let read_bodies = gpu.staging("body sums read", words(body_rows));
        let read_particles = gpu.staging(
            "particles read",
            count as u64 * std::mem::size_of::<CardParticle>() as u64,
        );
        Ok(Self {
            params,
            ctl: built.ctl,
            particles: built.particles,
            partials: built.partials,
            loads: built.loads,
            args,
            control,
            body_sums: (body_sums.0, body_sums.1, particle_threads),
            passes,
            read_ctl,
            read_loads,
            read_bodies,
            read_particles,
            count,
            slots,
            bodies: bodies.len(),
            particle_groups,
            t_ref,
            body_mass,
            mass: particles.iter().map(|p| p.mass).sum(),
            origin,
            template: particles.to_vec(),
            gpu,
        })
    }

    /// The name of the card, e.g. "NVIDIA GeForce RTX 4070 (discrete)".
    #[allow(dead_code)]
    pub(super) fn name(&self) -> &str {
        &self.gpu.name
    }

    /// Runs `steps` time steps (fewer once the card waits) and reads the control block.
    fn steps(&self, steps: usize) -> Result<Control, String> {
        let mut encoder = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("particles"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("particle steps"),
                timestamp_writes: None,
            });
            for _ in 0..steps {
                pass.set_pipeline(&self.control.0);
                pass.set_bind_group(0, &self.control.1, &[]);
                pass.dispatch_workgroups(1, 1, 1);
                for p in &self.passes {
                    pass.set_pipeline(&p.pipeline);
                    pass.set_bind_group(0, &p.bind, &[]);
                    pass.dispatch_workgroups_indirect(&self.args, p.at);
                }
            }
        }
        encoder.copy_buffer_to_buffer(
            &self.ctl,
            0,
            &self.read_ctl,
            0,
            std::mem::size_of::<Control>() as u64,
        );
        self.gpu.queue.submit([encoder.finish()]);
        let ctl = self
            .gpu
            .map(&self.read_ctl, |b| *bytemuck::from_bytes::<Control>(b))?;
        if let Some(e) = self.gpu.take_error() {
            return Err(e);
        }
        Ok(ctl)
    }

    /// The loads summed since the last reset, and the body sums when `bodies` (particles
    /// left, the sum of their temperatures, the highest temperature per body).
    fn read_sums(&self, bodies: bool) -> Result<(Vec<f32>, Vec<[f64; 3]>), String> {
        let mut encoder = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("particle sums"),
            });
        let rows = self.particle_groups * self.bodies.max(1) * 3;
        if bodies {
            {
                let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                    label: Some("body sums"),
                    timestamp_writes: None,
                });
                pass.set_pipeline(&self.body_sums.0);
                pass.set_bind_group(0, &self.body_sums.1, &[]);
                pass.dispatch_workgroups(self.body_sums.2.0, self.body_sums.2.1, 1);
            }
            encoder.copy_buffer_to_buffer(
                &self.partials,
                0,
                &self.read_bodies,
                0,
                (rows * 4) as u64,
            );
        }
        encoder.copy_buffer_to_buffer(
            &self.loads,
            0,
            &self.read_loads,
            0,
            (self.slots * LOAD * 4) as u64,
        );
        self.gpu.queue.submit([encoder.finish()]);
        let loads = self.gpu.map(&self.read_loads, |b| {
            bytemuck::cast_slice::<u8, f32>(b)[..self.slots * LOAD].to_vec()
        })?;
        let mut sums = vec![[0.0, 0.0, f64::NEG_INFINITY]; self.bodies];
        if bodies && self.bodies > 0 {
            self.gpu.map(&self.read_bodies, |b| {
                let values = bytemuck::cast_slice::<u8, f32>(b);
                for g in 0..self.particle_groups {
                    for (k, s) in sums.iter_mut().enumerate() {
                        let at = (g * self.bodies + k) * 3;
                        s[0] += f64::from(values[at]);
                        s[1] += f64::from(values[at + 1]);
                        s[2] = s[2].max(f64::from(values[at + 2]));
                    }
                }
            })?;
        }
        if let Some(e) = self.gpu.take_error() {
            return Err(e);
        }
        Ok((loads, sums))
    }

    /// The particles that are left, as the processor's frame takes them.
    fn read_particles(&self) -> Result<Vec<Particle>, String> {
        let size = (self.count * std::mem::size_of::<CardParticle>()) as u64;
        let mut encoder = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("particles read"),
            });
        encoder.copy_buffer_to_buffer(&self.particles, 0, &self.read_particles, 0, size);
        self.gpu.queue.submit([encoder.finish()]);
        self.gpu.map(&self.read_particles, |b| {
            let card: &[CardParticle] = bytemuck::cast_slice(b);
            (card[..self.count].iter())
                .zip(&self.template)
                .filter(|(c, _)| c.meta[2] == 0)
                .map(|(c, t)| {
                    let x = std::array::from_fn(|a| self.origin[a] + f64::from(c.x[a]));
                    let tau = sym([c.tau[0], c.tau[1], c.tau[2]], [c.tau[3], c.bt[2], c.bt[3]]);
                    Particle {
                        x,
                        v: std::array::from_fn(|a| f64::from(c.v[a])),
                        c: [[0.0; 3]; 3],
                        state: PointState {
                            b: IDENTITY,
                            j: f64::from(c.c[0][3]),
                            plastic: f64::from(c.c[1][3]),
                        },
                        tau,
                        shear_rate: c.c[2][3],
                        temperature: self.t_ref + f64::from(c.x[3]) + f64::from(c.v[3]),
                        ..*t
                    }
                })
                .collect()
        })
    }

    /// The time loop: as [`super::run_cpu`].
    pub(super) fn run(
        self,
        setup: &Setup<'_>,
        particles: &[Particle],
        history: &mut ParticleHistory,
        progress: &mut dyn FnMut(Status),
        on_frame: &mut dyn FnMut(ParticleFrame),
        notes: &mut Vec<String>,
    ) -> Result<Tally, String> {
        let model = setup.model;
        let control = &model.control;
        let end = control.end_time;
        let frames = control.frames.max(1);
        let frame_dt = end / frames as f64;
        let history_dt = end / HISTORY_POINTS as f64;
        let (potential0, energy0, slowest) = (setup.potential0, setup.energy0, setup.slowest);
        let mut scale_energy = energy0.abs();
        let mut dissipated = 0.0;
        let mut wall_loss = 0.0;
        let mut wall_work = 0.0;
        let mut outflow_energy = 0.0;
        let mut wall_force = vec![Load::default(); history.walls.len()];
        let mut wall_samples = 0usize;
        let mut t = 0.0f64;
        let mut steps = 0usize;
        let mut next_frame = 1usize;
        let mut next_history = 0usize;
        let mut dt_range = (0.0f64, f64::INFINITY);
        let mut max_mach: f64 = 0.0;
        let mut last_report = Instant::now();
        let mut stopped = false;
        let mut gone = 0u32;
        let mut ctl = Control {
            max_speed: particles.iter().map(|p| norm(p.v)).fold(0.0, f64::max) as f32,
            dt_min: f32::MAX,
            ..Default::default()
        };
        // The seconds of a time step on the card, and the time step of the last steps: for
        // the size of a batch.
        let mut per_step: Option<f64> = None;
        let mut dt_last = f64::from(self.params.box_lo[3]) * setup.grid.h
            / (setup.wave + f64::from(ctl.max_speed));
        'run: while t < end * (1.0 - 1e-12) {
            if plxs_core::CancelToken::current_cancelled() {
                stopped = true;
                break;
            }
            let target = next_frame as f64 * frame_dt;
            let next_point = next_history as f64 * history_dt;
            ctl = Control {
                running: 1,
                frame_at: (target - t) as f32,
                history: (next_point - t) as f32,
                base: t as f32,
                max_speed: ctl.max_speed,
                dt_min: f32::MAX,
                gone,
                ..Default::default()
            };
            self.gpu
                .queue
                .write_buffer(&self.ctl, 0, bytemuck::bytes_of(&ctl));
            self.gpu
                .queue
                .write_buffer(&self.loads, 0, &vec![0u8; self.slots * LOAD * 4]);
            let event = (target.min(next_point.max(t)) - t).max(0.0);
            loop {
                let ahead = (event - f64::from(ctl.tl)) / dt_last.max(1e-300);
                let mut batch = (ahead.ceil() as usize + 1).clamp(1, BATCH);
                // A submission stays short: the driver resets a card that does not answer
                // for a few seconds (two on Windows), and the run should stop soon when asked.
                batch = match per_step {
                    Some(p) => batch.min(((SUBMISSION / p) as usize).max(1)),
                    None => batch.min(4),
                };
                let before = ctl.steps;
                let started = Instant::now();
                ctl = self.steps(batch)?;
                let done = ctl.steps.saturating_sub(before);
                if done > 0 {
                    per_step = Some(started.elapsed().as_secs_f64() / f64::from(done));
                }
                if ctl.steps > 0 && ctl.dt > 0.0 {
                    dt_last = f64::from(ctl.dt);
                }
                if last_report.elapsed().as_secs_f64() > 0.2 {
                    last_report = Instant::now();
                    progress(Status {
                        time: t + f64::from(ctl.tl),
                        end_time: end,
                        steps: steps + ctl.steps as usize,
                        particles: self.count - ctl.gone as usize,
                        time_step: f64::from(ctl.dt),
                        on_card: true,
                    });
                }
                if ctl.running == 0 {
                    break;
                }
                if plxs_core::CancelToken::current_cancelled() {
                    stopped = true;
                    t += f64::from(ctl.tl);
                    steps += ctl.steps as usize;
                    break 'run;
                }
            }
            if ctl.reason & BROKEN != 0 {
                return Err(if ctl.max_speed >= UNSTABLE {
                    super::UNSTABLE_MESSAGE.into()
                } else {
                    "the time step became zero".into()
                });
            }
            steps += ctl.steps as usize;
            gone = ctl.gone;
            if ctl.steps > 0 {
                dt_range = (
                    dt_range.0.max(f64::from(ctl.dt_max)),
                    dt_range.1.min(f64::from(ctl.dt_min)),
                );
            }
            t = if ctl.reason & FRAME != 0 {
                target
            } else {
                t + f64::from(ctl.tl)
            };
            let pair = |p: [f32; 2]| f64::from(p[0]) + f64::from(p[1]);
            dissipated += pair(ctl.dissipated);
            wall_loss += pair(ctl.loss);
            wall_work += pair(ctl.work);
            outflow_energy += pair(ctl.out_energy);
            let max_speed = f64::from(ctl.max_speed);
            max_mach = max_mach.max(f64::from(ctl.top_speed) / slowest);
            let kinetic = f64::from(ctl.kinetic);
            let stored = f64::from(ctl.stored);
            let potential = f64::from(ctl.potential) - potential0;
            let total = kinetic + potential + stored + dissipated + wall_loss + outflow_energy;
            scale_energy = scale_energy
                .max(kinetic + potential.abs() + stored + dissipated + wall_loss.abs())
                .max(wall_work.abs());
            let record = ctl.reason & HISTORY != 0 || t >= end * (1.0 - 1e-12);
            let (loads, bodies) = self.read_sums(record)?;
            for (slot, f) in wall_force.iter_mut().enumerate() {
                let l = &loads[slot * LOAD..(slot + 1) * LOAD];
                let v = |k: usize| f64::from(l[k]);
                f.add(&Load {
                    force: [v(0), v(1), v(2)],
                    moment: [v(3), v(4), v(5)],
                    work: v(6),
                    heat: v(7),
                });
            }
            wall_samples += ctl.samples as usize;
            if record {
                next_history = ((t / history_dt).floor() as usize + 1).max(next_history + 1);
                history.time.push(t);
                history.time_step.push(f64::from(ctl.dt));
                history.kinetic.push(kinetic);
                history.potential.push(potential);
                history.elastic.push(stored);
                history.dissipated.push(dissipated);
                history.wall_loss.push(wall_loss + wall_work);
                history.wall_work.push(wall_work);
                history.outflow_energy.push(outflow_energy);
                history.balance_error.push(energy0 - total);
                // The masses from the particles left, in double precision: the mass balance
                // closes.
                let masses: Vec<f64> = (bodies.iter().zip(&self.body_mass))
                    .map(|(b, m)| b[0] * m)
                    .collect();
                let mass: f64 = masses.iter().sum();
                history.mass.push(mass);
                history.outflow.push(self.mass - mass);
                history.mach.push(max_speed / slowest);
                for ((h, &[count, sum, hottest]), &mass) in
                    history.bodies.iter_mut().zip(&bodies).zip(&masses)
                {
                    h.mass.push(mass);
                    let mean = h.temperature_mean.last().copied().unwrap_or(0.0);
                    let (mean, hottest) = if count > 0.0 {
                        (self.t_ref + sum / count, self.t_ref + hottest)
                    } else {
                        (mean, h.temperature_max.last().copied().unwrap_or(mean))
                    };
                    h.temperature_mean.push(mean);
                    h.temperature_max.push(hottest);
                }
                let n = wall_samples.max(1) as f64;
                for (h, f) in history.walls.iter_mut().zip(&mut wall_force) {
                    h.force.push(scale(f.force, 1.0 / n));
                    h.moment.push(scale(f.moment, 1.0 / n));
                    let done = h.work.last().copied().unwrap_or(0.0) + f.work;
                    h.work.push(done);
                    let given = h.heat.last().copied().unwrap_or(0.0) + f.heat;
                    h.heat.push(given);
                    *f = Load::default();
                }
                wall_samples = 0;
            }
            if ctl.reason & FRAME != 0 {
                on_frame(frame(t, &self.read_particles()?, setup.grid));
                next_frame += 1;
            }
            if ctl.reason & EMPTY != 0 {
                notes.push("All material left the model.".into());
                break;
            }
        }
        Ok(Tally {
            t,
            steps,
            dt_range,
            max_mach,
            stopped,
            scale_energy,
            particles: self.count - gone as usize,
        })
    }
}

/// A pass of a time step as planned: its entry point, its bindings, its workgroups and the
/// level of the scan it does.
type Planned = (&'static str, Vec<u32>, (u32, u32), Option<usize>);

/// The buffers of the card.
struct Buffers {
    params: wgpu::Buffer,
    ctl: wgpu::Buffer,
    particles: wgpu::Buffer,
    records: wgpu::Buffer,
    keys: wgpu::Buffer,
    counts: wgpu::Buffer,
    fill: wgpu::Buffer,
    scan: wgpu::Buffer,
    order: wgpu::Buffer,
    nodes: wgpu::Buffer,
    values: wgpu::Buffer,
    heat: wgpu::Buffer,
    laws: wgpu::Buffer,
    walls: wgpu::Buffer,
    poses: wgpu::Buffer,
    tables: wgpu::Buffer,
    bodies: wgpu::Buffer,
    partials: wgpu::Buffer,
    loads: wgpu::Buffer,
}

/// A bind group layout with the bindings of `gpu.wgsl` numbered `bindings`.
fn layout(gpu: &Gpu, label: &str, bindings: &[u32]) -> wgpu::BindGroupLayout {
    let entries: Vec<wgpu::BindGroupLayoutEntry> = bindings
        .iter()
        .map(|&b| wgpu::BindGroupLayoutEntry {
            binding: b,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: if b == UNIFORM {
                    wgpu::BufferBindingType::Uniform
                } else {
                    wgpu::BufferBindingType::Storage {
                        read_only: read_only(b),
                    }
                },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        })
        .collect();
    gpu.device
        .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some(label),
            entries: &entries,
        })
}

/// The symmetric matrix of the diagonal `d` and the off-diagonals `o` (01, 12, 02).
fn sym(d: [f32; 3], o: [f32; 3]) -> [[f64; 3]; 3] {
    let [d0, d1, d2] = d.map(f64::from);
    let [o01, o12, o02] = o.map(f64::from);
    [[d0, o01, o02], [o01, d1, o12], [o02, o12, d2]]
}
