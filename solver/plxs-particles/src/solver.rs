//! The time loop of the material point method: MLS-MPM with quadratic B-splines and the
//! affine (APIC) transfer of momentum.
//!
//! A step: (1) the particles spread mass, affine momentum and the force of their stress to
//! the nodes of the background grid (one transfer, the force folded into the affine part,
//! Hu et al. 2018, "A moving least squares material point method with displacement
//! discontinuity and two-way rigid body coupling", eq. 16 to 18); (2) the nodes add gravity
//! and the walls set their velocity after the wall law; (3) the particles gather velocity
//! and velocity gradient (`C`, Jiang et al. 2015, "The affine particle-in-cell method"),
//! update their stress and move. With F-bar the particles also spread their volume ratio
//! and gather its mass-weighted average back, which replaces their own.
//!
//! The grid is dense over the box of the model, padded by two cells on each side that is
//! not periodic, and built anew every step. The particles are sorted into blocks of
//! [`BLOCK`]³ cells; each block spreads into a patch of its own in parallel, and the patches
//! are added to the grid one after the other, so the results do not depend on the threads.

#![allow(clippy::needless_range_loop)]

use std::time::Instant;

use plxs_core::particles::{
    BodyHistory, Domain, FlowTables, PARTICLE_FIELDS, ParticleFrame, ParticleHistory, ParticleLaw,
    ParticleModel, ParticleReport, ParticleResults, RigidMotion, Side, Space, WallHistory, WallLaw,
};
use rayon::prelude::*;

use crate::distance::{DistanceField, ParityRows};
use crate::law::{Law, PointState};
use crate::math::{
    IDENTITY, Mat3, Vec3, add, cross, det, deviator, dot, mat_vec, norm, rotation, scale, sub,
    transpose,
};

#[cfg(feature = "gpu")]
#[path = "gpu.rs"]
mod gpu;
#[path = "inlet.rs"]
mod inlet;

/// The error of a run whose motion is no number any more.
const UNSTABLE_MESSAGE: &str = "the motion became unstable (speeds that are no number); a \
     smaller cell, a lower wave speed or a stiffer material may help";

/// Cells per block and axis of the sorting.
const BLOCK: usize = 4;
/// Nodes per axis of the patch of a block: the block's cells and the reach of the stencil.
const PATCH: usize = BLOCK + 2;
/// How many points the history keeps over the run.
const HISTORY_POINTS: usize = 1000;
/// The width of the band of exact distances around a wall, in cells.
const WALL_BAND: f64 = 3.0;
/// Nodes up to this distance in front of a wall, in cells, take the wall's velocity: the
/// velocity of the material then reaches the wall's halfway between the last node held and
/// the first free one, on average at the surface itself (nodes held only inside the wall
/// would put the wall half a cell behind its surface).
const WALL_REACH: f64 = 0.2;
/// The share of the way a particle's volume ratio moves towards the averaged one per step.
/// How fast a particle's temperature follows the grid's, in units of α / h².
const SUBCELL_CONDUCTION: f64 = 0.3;
const VOLUME_RELAX: f64 = 0.05;
/// In a 2D model, a particle whose area in the plane grew to this many times the area of
/// the particle spacing squared splits in two. Material that flows towards the axis of an
/// axisymmetric model spreads its particles in the plane (a ring of the same volume nearer
/// the axis is a larger section): without splitting they part and the material tears.
const SPLIT_AREA: f64 = 2.0;
/// The smallest radius, in cells, the ring terms of a body of revolution divide by.
const RADIUS_FLOOR: f64 = 0.25;
/// The speed at which fine grains sink through coarse ones, `C d_fine ln(d_coarse/d_fine)`
/// times the shear rate (Schlick et al. 2015, J. Fluid Mech. 765, 632).
const SEGREGATION: f64 = 0.26;
/// The diffusion of grains in a shear flow, `C γ̇ d²` with the mean diameter (Utter &
/// Behringer 2004, Phys. Rev. E 69, 031308).
const GRAIN_DIFFUSION: f64 = 0.042;

/// How a run is going, for a progress display.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Status {
    pub time: f64,
    pub end_time: f64,
    pub steps: usize,
    pub particles: usize,
    pub time_step: f64,
    /// Whether the time steps run on the graphics card.
    pub on_card: bool,
}

#[derive(Clone, Copy, Debug)]
struct Particle {
    /// The number of the particle, counted from 0 in the order they were made.
    id: u32,
    x: Vec3,
    v: Vec3,
    c: Mat3,
    state: PointState,
    tau: Mat3,
    mass: f64,
    volume: f64,
    material: u16,
    body: u16,
    shear_rate: f32,
    temperature: f64,
    /// The acceleration of gravity on the particle's body.
    gravity: Vec3,
    /// Mass times specific heat (zero: the temperature stays).
    heat_capacity: f64,
    conductivity: f64,
    /// The volume fraction of fine grains of a mixture of two sizes.
    fine: f64,
    /// The fine and the coarse grain diameter of a mixture of two sizes (zero: the
    /// particle does not segregate).
    grains: [f64; 2],
    /// The part of the particle it was made as that it still is: halved by every split.
    share: f32,
    /// Left the box through an open side.
    gone: bool,
    /// The inlet whose layer the particle is in, counted from 1; 0 for none.
    inlet: u16,
    /// Where the successor of a particle that left the layer of an inlet is made, and the
    /// inlet.
    spawn: Option<(Vec3, u16)>,
}

/// The background grid.
struct Grid {
    /// The position of node 0.
    origin: Vec3,
    h: f64,
    /// Nodes per axis.
    n: [usize; 3],
    periodic: [bool; 3],
    space: Space,
    /// The index of the first and last node on or beyond the box, per axis (sides).
    lo: [usize; 3],
    hi: [usize; 3],
}

impl Grid {
    fn count(&self) -> usize {
        self.n[0] * self.n[1] * self.n[2]
    }

    fn index(&self, i: [usize; 3]) -> usize {
        i[0] + self.n[0] * (i[1] + self.n[1] * i[2])
    }

    fn position(&self, i: [usize; 3]) -> Vec3 {
        std::array::from_fn(|a| self.origin[a] + i[a] as f64 * self.h)
    }

    /// The depth along z of what a cell at radius `r` (x) stands for: the cell in 3D, a
    /// unit length in plane strain, the ring in a body of revolution.
    fn depth(&self, r: f64) -> f64 {
        match self.space {
            Space::Solid => self.h,
            Space::PlaneStrain => 1.0,
            Space::Axisymmetric => std::f64::consts::TAU * self.radius(r),
        }
    }

    /// Whether the grid reaches beyond the axis of a body of revolution.
    fn mirrors_axis(&self) -> bool {
        self.space == Space::Axisymmetric
            && !self.periodic[0]
            && self.origin[0].abs() > 0.0
            && (self.origin[0] + self.lo[0] as f64 * self.h).abs() < 1e-9 * self.h
    }

    /// A radius kept off the axis, where the ring terms divide by it.
    fn radius(&self, r: f64) -> f64 {
        r.max(RADIUS_FLOOR * self.h)
    }

    /// The first node of the stencil of `x` (unwrapped) and the weights.
    fn stencil(&self, x: Vec3) -> ([isize; 3], [[f64; 3]; 3], Vec3) {
        let mut base = [0isize; 3];
        let mut w = [[0.0; 3]; 3];
        let mut frac = [0.0; 3];
        for a in 0..3 {
            let u = (x[a] - self.origin[a]) / self.h;
            // floor() is a library call on the baseline x86-64 target; the shift keeps the
            // truncation a floor for the half cell a periodic axis reaches below zero.
            let b = ((u + 7.5) as isize - 8) as f64;
            let f = u - b;
            base[a] = b as isize;
            frac[a] = f;
            w[a] = [
                0.5 * (1.5 - f) * (1.5 - f),
                0.75 - (f - 1.0) * (f - 1.0),
                0.5 * (f - 0.5) * (f - 0.5),
            ];
        }
        (base, w, frac)
    }

    /// The offsets into the node arrays of the three nodes of a stencil along each axis.
    fn offsets(&self, base: [isize; 3]) -> [[usize; 3]; 3] {
        let stride = [1, self.n[0], self.n[0] * self.n[1]];
        std::array::from_fn(|a| {
            std::array::from_fn(|s| self.wrap(a, base[a] + s as isize) * stride[a])
        })
    }

    /// The node index of an unwrapped node position along an axis.
    fn wrap(&self, a: usize, i: isize) -> usize {
        if self.periodic[a] {
            i.rem_euclid(self.n[a] as isize) as usize
        } else {
            i.clamp(0, self.n[a] as isize - 1) as usize
        }
    }
}

/// Where a wall is at a time and how fast it moves.
struct Pose {
    rotation: Mat3,
    /// The factor of the amplitude on the velocities.
    factor: f64,
    /// The center of the motion where it is.
    center: Vec3,
}

/// What the material does to a wall over a time step: force, moment about the origin and
/// the work the wall does on the material.
#[derive(Clone, Copy, Default)]
struct Load {
    force: Vec3,
    moment: Vec3,
    work: f64,
    /// Heat the wall gave the material by transfer.
    heat: f64,
}

impl Load {
    fn add(&mut self, other: &Load) {
        self.force = add(self.force, other.force);
        self.moment = add(self.moment, other.moment);
        self.work += other.work;
        self.heat += other.heat;
    }
}

/// A rigid wall as the solver uses it.
struct Wall {
    field: DistanceField,
    law: WallLaw,
    motion: RigidMotion,
}

impl Wall {
    /// The pose of the body at the end `t` of a time step of `dt`, with the velocity of
    /// the middle of the step: a step that ends on a kink of the amplitude moves with the
    /// segment it lies in, not with the next one.
    fn pose(&self, t: f64, dt: f64) -> Pose {
        let moved = self.motion.moved_time(t);
        let shift = scale(self.motion.velocity, moved);
        Pose {
            rotation: rotation(scale(self.motion.angular, moved)),
            factor: self.motion.factor(t - 0.5 * dt),
            center: add(self.motion.center, shift),
        }
    }

    /// The distance, the outward unit normal and the wall's velocity at `x` at the pose;
    /// `None` far outside.
    fn probe(&self, x: Vec3, pose: &Pose) -> Option<(f64, Vec3, Vec3)> {
        let r = &pose.rotation;
        let body = add(
            mat_vec(&transpose(r), sub(x, pose.center)),
            self.motion.center,
        );
        let (phi, gradient) = self.field.sample(body)?;
        let g = norm(gradient);
        let normal = if g > 1e-12 {
            mat_vec(r, scale(gradient, 1.0 / g))
        } else {
            [0.0; 3]
        };
        let velocity = scale(
            add(
                self.motion.velocity,
                cross(self.motion.angular, sub(x, pose.center)),
            ),
            pose.factor,
        );
        Some((phi, normal, velocity))
    }

    /// The box in world coordinates that holds the body at the pose (its lattice box,
    /// turned).
    fn world_box(&self, pose: &Pose) -> (Vec3, Vec3) {
        let (lo, hi) = self.field.bounds();
        let (r, center) = (&pose.rotation, pose.center);
        let mut out = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for corner in 0..8 {
            let p = [
                if corner & 1 == 0 { lo[0] } else { hi[0] },
                if corner & 2 == 0 { lo[1] } else { hi[1] },
                if corner & 4 == 0 { lo[2] } else { hi[2] },
            ];
            let w = add(mat_vec(r, sub(p, self.motion.center)), center);
            for a in 0..3 {
                out.0[a] = out.0[a].min(w[a]);
                out.1[a] = out.1[a].max(w[a]);
            }
        }
        out
    }
}

/// A velocity after a wall law: `v` relative to the wall's velocity `wall` at a node in
/// or on the wall, with the outward normal `n` (zero: stick).
fn wall_velocity(law: WallLaw, v: Vec3, wall: Vec3, n: Vec3) -> Vec3 {
    let rel = sub(v, wall);
    let vn = dot(rel, n);
    let out = match law {
        WallLaw::Stick => [0.0; 3],
        _ if n == [0.0; 3] => [0.0; 3],
        _ if vn >= 0.0 => rel,
        WallLaw::Slip => sub(rel, scale(n, vn)),
        WallLaw::Coulomb { friction } => {
            let tangent = sub(rel, scale(n, vn));
            let t = norm(tangent);
            if t <= -friction * vn {
                [0.0; 3]
            } else {
                scale(tangent, 1.0 + friction * vn / t)
            }
        }
    };
    add(out, wall)
}

/// What one node of the grid holds while particles spread to it.
#[derive(Clone, Copy, Default)]
struct NodeSum {
    mass: f64,
    momentum: Vec3,
    /// The force of the stresses on the node (in the momentum already), for the energy
    /// the walls take.
    force: Vec3,
    /// The weight of the particles, mass times the gravity of their bodies.
    weight: Vec3,
    /// Mass times volume ratio, for F-bar.
    volume: f64,
    /// Heat capacity, heat (capacity times temperature), mass times conductivity and the
    /// volume of the particles: for the temperature.
    heat_capacity: f64,
    heat: f64,
    conduction: f64,
    space: f64,
    /// The mass of the particles of grain mixtures, its fine part, and mass times the
    /// speed of segregation per unit of the gravity's direction and the grain diffusivity:
    /// for the segregation.
    mixture: f64,
    fine: f64,
    sinking: f64,
    diffusion: f64,
}

/// The sums of a step over the particles.
#[derive(Clone, Copy, Default)]
struct Sums {
    kinetic: f64,
    potential: f64,
    stored: f64,
    dissipated: f64,
    max_speed: f64,
    out_mass: f64,
    out_energy: f64,
}

impl Sums {
    fn join(self, o: Self) -> Self {
        Self {
            kinetic: self.kinetic + o.kinetic,
            potential: self.potential + o.potential,
            stored: self.stored + o.stored,
            dissipated: self.dissipated + o.dissipated,
            max_speed: self.max_speed.max(o.max_speed),
            out_mass: self.out_mass + o.out_mass,
            out_energy: self.out_energy + o.out_energy,
        }
    }
}

/// Solves a particle analysis. `progress` hears of the run now and then; the run stops
/// early when the [`plxs_core::CancelToken`] of the calling thread is cancelled (the
/// results then end at the last frame, [`ParticleReport::stopped`]).
pub fn solve(
    model: &ParticleModel,
    progress: &mut dyn FnMut(Status),
) -> Result<ParticleResults, String> {
    let mut frames = Vec::new();
    let mut results = solve_streaming(model, progress, &mut |frame| frames.push(frame))?;
    results.frames = frames;
    Ok(results)
}

/// [`solve`] that hands every frame to `on_frame` as soon as it is computed instead of
/// keeping it: the results it returns have no frames. A run of many particles and frames
/// then needs the memory of one frame only.
pub fn solve_streaming(
    model: &ParticleModel,
    progress: &mut dyn FnMut(Status),
    on_frame: &mut dyn FnMut(ParticleFrame),
) -> Result<ParticleResults, String> {
    let started = Instant::now();
    let grid = grid_of(&model.domain)?;
    if model.materials.is_empty() {
        return Err("the particle analysis has no material".into());
    }
    let mut notes = Vec::new();
    let mut laws = Vec::new();
    let mut factor: f64 = 1.0;
    for m in &model.materials {
        if m.density.is_nan() || m.density <= 0.0 {
            return Err(format!("material {} has no density", m.name));
        }
        let law = Law::new(m.density, &m.law);
        let (law, f) = match model.control.sound_speed {
            Some(c) => law.slowed(c),
            None => (law, 1.0),
        };
        if f > 1.0 {
            notes.push(format!(
                "Wave speed of material {} slowed by a factor {f:.1} to {:.4} (bulk modulus {:.4e})",
                m.name,
                law.wave_speed(),
                law.bulk
            ));
        }
        factor = factor.max(f);
        laws.push(law);
    }
    let walls: Vec<Wall> = (model.walls.iter())
        .map(|w| Wall {
            field: DistanceField::new(
                &w.shape,
                grid.h,
                (WALL_BAND + 2.0) * grid.h,
                WALL_BAND * grid.h,
                grid.origin,
            ),
            law: w.law,
            motion: w.motion.clone(),
        })
        .collect();
    let feeds = (model.inlets.iter())
        .map(|i| inlet::Feed::new(i, &grid))
        .collect::<Result<Vec<_>, _>>()?;
    if let Some(i) = model.inlets.iter().find(|i| i.body >= model.bodies.len()) {
        return Err(format!(
            "inlet {} feeds a body the model does not have",
            i.name
        ));
    }
    let mut particles = fill(model, &grid, &laws, &walls)?;
    inlet::prime(model, &grid, &laws, &feeds, &walls, &mut particles);
    if particles.is_empty() {
        return Err("no particles: the material bodies lie outside the box or inside walls".into());
    }
    let wave = laws.iter().map(Law::wave_speed).fold(0.0, f64::max);
    let slowest = laws
        .iter()
        .map(Law::wave_speed)
        .fold(f64::INFINITY, f64::min);
    // The values of the flow over the temperature, per material.
    let tables: Vec<&FlowTables> = model.materials.iter().map(|m| &m.flow_tables).collect();
    // The highest viscosity of a fluid at any temperature limits the time step.
    let highest = |m: usize, l: &Law| {
        (tables[m].viscosity.iter().map(|p| p[1])).fold(l.explicit_viscosity(), f64::max)
    };
    let viscosity_limit = (laws.iter().enumerate())
        .filter(|(_, l)| l.explicit_viscosity() > 0.0)
        .map(|(m, l)| l.density * grid.h * grid.h / (8.0 * highest(m, l)))
        .fold(f64::INFINITY, f64::min);
    // Explicit conduction over the grid is stable below h² / (6 α).
    let heat_limit = match model.heat {
        None => f64::INFINITY,
        Some(_) => (model.materials.iter())
            .filter_map(|m| {
                let t = m.thermal?;
                let alpha = t.conductivity / (m.density * t.specific_heat);
                (alpha > 0.0).then(|| grid.h * grid.h / (7.5 * alpha))
            })
            .fold(f64::INFINITY, f64::min),
    };
    let wall_temperature: Vec<Option<f64>> = model.walls.iter().map(|w| w.temperature).collect();
    let mut names: Vec<String> = model.walls.iter().map(|w| w.name.clone()).collect();
    for side in ["X min", "X max", "Y min", "Y max", "Z min", "Z max"] {
        names.push(format!("Box {side}"));
    }
    let mut results = ParticleResults::default();
    let mut history = ParticleHistory {
        walls: names
            .iter()
            .map(|n| WallHistory {
                name: n.clone(),
                ..Default::default()
            })
            .collect(),
        bodies: (model.bodies.iter())
            .map(|b| BodyHistory {
                name: b.name.clone(),
                ..Default::default()
            })
            .collect(),
        ..Default::default()
    };
    let initial = particle_sums(&particles);
    let setup = Setup {
        model,
        grid: &grid,
        laws: &laws,
        walls: &walls,
        tables: &tables,
        wave,
        slowest,
        viscosity_limit,
        heat_limit,
        wall_temperature: &wall_temperature,
        feeds: &feeds,
        potential0: initial.potential,
        energy0: initial.kinetic + initial.stored,
    };
    let initial_count = particles.len();
    #[cfg(feature = "gpu")]
    let card = gpu::Card::open_for(&setup, &particles, &mut notes);
    on_frame(frame(0.0, &particles, &grid));
    #[cfg(feature = "gpu")]
    let on_card = card.is_some();
    #[cfg(not(feature = "gpu"))]
    let on_card = false;
    #[cfg(feature = "gpu")]
    let tally = match card {
        Some(card) => card.run(
            &setup,
            &particles,
            &mut history,
            progress,
            on_frame,
            &mut notes,
        )?,
        None => run_cpu(
            &setup,
            particles,
            &mut history,
            progress,
            on_frame,
            &mut notes,
        )?,
    };
    #[cfg(not(feature = "gpu"))]
    let tally = run_cpu(
        &setup,
        particles,
        &mut history,
        progress,
        on_frame,
        &mut notes,
    )?;
    let Tally {
        t,
        steps,
        dt_range,
        max_mach,
        stopped,
        scale_energy,
        particles: left,
    } = tally;
    progress(Status {
        time: t,
        end_time: model.control.end_time,
        steps,
        particles: left,
        time_step: dt_range.1,
        on_card,
    });
    if max_mach > 0.1 && factor > 1.0 {
        notes.push(format!(
            "The material moved at up to {max_mach:.2} times the slowed wave speed; a value \
             above 0.1 makes the slowed waves change the result"
        ));
    }
    let cells = std::array::from_fn(|a| {
        ((model.domain.max[a] - model.domain.min[a]) / grid.h).round() as usize
    });
    // The balance error relative to the largest energy of the run.
    for e in &mut history.balance_error {
        *e /= scale_energy.max(1e-300);
    }
    results.history = history;
    results.report = ParticleReport {
        particles: initial_count,
        cells,
        steps,
        time_step: (dt_range.0, if steps > 0 { dt_range.1 } else { 0.0 }),
        sound_speed_factor: factor,
        max_mach,
        seconds: started.elapsed().as_secs_f64(),
        stopped,
        notes,
    };
    Ok(results)
}

/// What the solvers share before the time loop.
struct Setup<'a> {
    model: &'a ParticleModel,
    grid: &'a Grid,
    laws: &'a [Law],
    walls: &'a [Wall],
    /// The values of the flow over the temperature, per material.
    tables: &'a [&'a FlowTables],
    /// The fastest and the slowest wave speed of the materials.
    wave: f64,
    slowest: f64,
    viscosity_limit: f64,
    heat_limit: f64,
    wall_temperature: &'a [Option<f64>],
    feeds: &'a [inlet::Feed],
    /// The potential energy and the kinetic plus stored energy at the start.
    potential0: f64,
    energy0: f64,
}

/// How the time loop ended.
struct Tally {
    t: f64,
    steps: usize,
    /// The largest and the smallest time step.
    dt_range: (f64, f64),
    max_mach: f64,
    stopped: bool,
    /// The largest energy of the run, the scale of the balance error.
    scale_energy: f64,
    /// The particles left in the model.
    particles: usize,
}

/// The time loop on the processor.
fn run_cpu(
    setup: &Setup<'_>,
    mut particles: Vec<Particle>,
    history: &mut ParticleHistory,
    progress: &mut dyn FnMut(Status),
    on_frame: &mut dyn FnMut(ParticleFrame),
    notes: &mut Vec<String>,
) -> Result<Tally, String> {
    let Setup {
        model,
        grid,
        laws,
        walls,
        tables,
        wave,
        slowest,
        viscosity_limit,
        heat_limit,
        wall_temperature,
        feeds,
        potential0,
        energy0,
    } = *setup;
    let control = &model.control;
    let end = control.end_time;
    let frames = control.frames.max(1);
    let frame_dt = end / frames as f64;
    let history_dt = end / HISTORY_POINTS as f64;
    let mut scale_energy = energy0.abs();
    let mut dissipated = 0.0;
    let mut wall_loss = 0.0;
    // The work of the moving walls on the material.
    let mut wall_work = 0.0;
    let mut outflow_energy = 0.0;
    let mut outflow_mass = 0.0;
    let mut inflow_energy = 0.0;
    let mut inflow_mass = 0.0;
    // The force of the pressure inlets on the nodes in a step.
    let mut pushed: Vec<(usize, Vec3)> = Vec::new();
    let mut wall_force = vec![Load::default(); history.walls.len()];
    let mut wall_samples = 0usize;

    let mut t = 0.0;
    let mut steps = 0usize;
    let mut next_frame = 1usize;
    let mut next_history = 0usize;
    let mut dt_range = (0.0f64, f64::INFINITY);
    let mut max_mach: f64 = 0.0;
    let mut max_speed = particles.iter().map(|p| norm(p.v)).fold(0.0, f64::max);
    let mut last_report = Instant::now();
    let mut stopped = false;
    let mut next_id = particles.iter().map(|p| p.id + 1).max().unwrap_or(0);
    let mut nodes = vec![NodeSum::default(); grid.count()];
    // Per node: the velocity at the end of the step and the averaged volume ratio.
    let mut values = vec![[0.0; 4]; grid.count()];
    // Per node: the heat the walls gave in the step, and the change of the temperature.
    let mut node_heat = vec![0.0; grid.count()];
    let mut node_dt = vec![0.0; grid.count()];
    // Per node: the change of the fine fraction of grain mixtures.
    let mut node_fine = vec![0.0; grid.count()];
    let mixtures = particles.iter().any(|p| p.grains[0] > 0.0);

    while t < end * (1.0 - 1e-12) {
        if plxs_core::CancelToken::current_cancelled() {
            stopped = true;
            break;
        }
        let target = next_frame as f64 * frame_dt;
        let mut dt = control.courant * grid.h / (wave + max_speed);
        dt = dt.min(viscosity_limit).min(heat_limit);
        if t + dt > target {
            dt = target - t;
        } else if t + 2.0 * dt > target {
            dt = 0.5 * (target - t);
        }
        if !max_speed.is_finite() {
            return Err(UNSTABLE_MESSAGE.into());
        }
        if dt.is_nan() || dt <= 0.0 {
            return Err("the time step became zero".into());
        }
        dt_range = (dt_range.0.max(dt), dt_range.1.min(dt));

        // (1) Particles to grid.
        spread(grid, &particles, dt, &mut nodes);
        let mirrored = grid.mirrors_axis();
        if mirrored {
            fold_axis(grid, &mut nodes);
        }
        // (1b) The pressure of the inlets on the nodes behind their layers.
        pushed.clear();
        for feed in feeds {
            feed.push(grid, t, |at| nodes[at].mass, &mut pushed);
        }
        for &(at, f) in &pushed {
            nodes[at].weight = add(nodes[at].weight, f);
        }
        // (2) The grid.
        let pose: Vec<_> = walls.iter().map(|w| w.pose(t + dt, dt)).collect();
        let boxes: Vec<_> = walls
            .iter()
            .zip(&pose)
            .map(|(w, p)| w.world_box(p))
            .collect();
        let side_index = walls.len();
        let heat = model.heat;
        let (loss, forces) = (values.par_iter_mut().zip(nodes.par_iter()))
            .zip(node_heat.par_iter_mut())
            .enumerate()
            .fold(
                || (0.0, vec![Load::default(); side_index + 6]),
                |(mut loss, mut forces), (at, ((value, node), q))| {
                    *q = 0.0;
                    if node.mass <= 0.0 {
                        *value = [0.0; 4];
                        return (loss, forces);
                    }
                    let i = [
                        at % grid.n[0],
                        (at / grid.n[0]) % grid.n[1],
                        at / (grid.n[0] * grid.n[1]),
                    ];
                    let before = scale(add(node.momentum, scale(node.weight, dt)), 1.0 / node.mass);
                    let mut after = before;
                    let x = grid.position(i);
                    // The wall the node is deepest in.
                    let mut owner = None;
                    let mut deepest = 0.0;
                    for (w, wall) in walls.iter().enumerate() {
                        let (lo, hi) = boxes[w];
                        if (0..3).any(|a| x[a] < lo[a] || x[a] > hi[a]) {
                            continue;
                        }
                        if let Some((phi, n, vw)) = wall.probe(x, &pose[w])
                            && phi <= WALL_REACH * grid.h
                            && (owner.is_none() || phi < deepest)
                        {
                            deepest = phi;
                            owner = Some((w, wall.law, n, vw));
                        }
                    }
                    if let Some((w, law, n, vw)) = owner {
                        after = wall_velocity(law, after, vw, n);
                        record(&mut forces[w], node.mass, before, after, dt, x, vw);
                        // Heat transfer with a wall that has a temperature, never past it.
                        if let (Some(heat), Some(tw)) = (heat, wall_temperature[w])
                            && node.heat_capacity > 0.0
                        {
                            let gap = tw - node.heat / node.heat_capacity;
                            let flow = heat.wall_transfer * grid.h * grid.depth(x[0]) * gap * dt;
                            let most = 0.5 * node.heat_capacity * gap.abs();
                            let flow = flow.clamp(-most, most);
                            *q += flow;
                            forces[w].heat += flow;
                        }
                    }
                    for a in 0..3 {
                        if grid.periodic[a] {
                            continue;
                        }
                        for (side, beyond) in
                            [(2 * a, i[a] <= grid.lo[a]), (2 * a + 1, i[a] >= grid.hi[a])]
                        {
                            if !beyond {
                                continue;
                            }
                            if let Side::Wall(law) = model.domain.sides[side] {
                                let mut n = [0.0; 3];
                                n[a] = if side % 2 == 0 { 1.0 } else { -1.0 };
                                let was = after;
                                after = wall_velocity(law, after, [0.0; 3], n);
                                record(
                                    &mut forces[side_index + side],
                                    node.mass,
                                    was,
                                    after,
                                    dt,
                                    x,
                                    [0.0; 3],
                                );
                            }
                        }
                    }
                    // Nothing crosses the axis of a body of revolution, nor leaves it.
                    if grid.space == Space::Axisymmetric && x[0] <= 1e-9 * grid.h && after[0] != 0.0
                    {
                        let was = after;
                        after[0] = 0.0;
                        record(
                            &mut forces[side_index],
                            node.mass,
                            was,
                            after,
                            dt,
                            x,
                            [0.0; 3],
                        );
                    }
                    // What the walls took: the kinetic energy they removed, less the work the
                    // forces on the node (stresses, gravity) would have done with the velocity
                    // removed, which the material never did: the stored and potential
                    // energies follow the particles.
                    if after != before {
                        let pull = add(node.force, node.weight);
                        let own = sub(before, scale(pull, dt / node.mass));
                        let taken = 0.5 * node.mass * (dot(own, own) - dot(after, after))
                            + dt * dot(pull, after);
                        loss += taken;
                        // What the walls take by friction heats the material there.
                        if heat.is_some() {
                            *q += taken.max(0.0);
                        }
                    }
                    *value = [after[0], after[1], after[2], node.volume / node.mass];
                    (loss, forces)
                },
            )
            .reduce(
                || (0.0, vec![Load::default(); side_index + 6]),
                |(l1, mut f1), (l2, f2)| {
                    for (a, b) in f1.iter_mut().zip(&f2) {
                        a.add(b);
                    }
                    (l1 + l2, f1)
                },
            );
        wall_loss += loss;
        for (w, (sum, f)) in wall_force.iter_mut().zip(&forces).enumerate() {
            let mut f = *f;
            // A wall's moment about its center of motion, where it is.
            if let Some(p) = pose.get(w) {
                f.moment = sub(f.moment, cross(p.center, f.force));
            }
            wall_work += f.work;
            sum.add(&f);
        }
        wall_samples += 1;
        // (2a) The inlets: the work of the pressure, and the speed of the layers of the
        // velocity inlets, with the energy it gives.
        for &(at, f) in &pushed {
            let v = &values[at];
            inflow_energy += dt * (f[0] * v[0] + f[1] * v[1] + f[2] * v[2]);
        }
        for feed in feeds {
            for &(at, from) in feed.behind() {
                if nodes[at].mass > 0.0 {
                    let v = values[from];
                    values[at][..3].copy_from_slice(&v[..3]);
                }
            }
            for (at, v) in feed.held(t + dt) {
                let m = nodes[at].mass;
                if m <= 0.0 {
                    continue;
                }
                let value = &mut values[at];
                let before = [value[0], value[1], value[2]];
                inflow_energy += 0.5 * m * (dot(v, v) - dot(before, before));
                value[..3].copy_from_slice(&v);
            }
        }

        // (2b) The heat: conduction between neighbouring nodes, and what the walls gave.
        if heat.is_some() {
            conduct(grid, &nodes, &node_heat, dt, &mut node_dt);
        }
        // (2c) The grains of mixtures: segregation and diffusion.
        if mixtures {
            segregate(grid, &nodes, dt, &mut node_fine);
        }
        if mirrored {
            mirror_axis(grid, &mut values, &mut node_dt, &mut node_fine);
        }
        let node_dt = &node_dt;
        let node_fine = &node_fine;
        let node_sums = &nodes;

        // (3) Grid to particles.
        let smooth = control.smooth_volume;
        let pose_end = &pose;
        let domain = &model.domain;
        let sums = particles
            .par_iter_mut()
            .map(|p| {
                let (base, w, frac) = grid.stencil(p.x);
                let offsets = grid.offsets(base);
                let d: [[f64; 3]; 3] =
                    std::array::from_fn(|a| std::array::from_fn(|s| (s as f64 - frac[a]) * grid.h));
                let mut v = [0.0; 3];
                let mut b = [[0.0; 3]; 3];
                let mut jbar = 0.0;
                let mut warmer = 0.0;
                let mut finer = 0.0;
                // The new temperature of the nodes around, weighted over the filled ones.
                let mut around = [0.0; 2];
                for sk in 0..3 {
                    for sj in 0..3 {
                        let wjk = w[1][sj] * w[2][sk];
                        let row = offsets[1][sj] + offsets[2][sk];
                        for si in 0..3 {
                            let weight = w[0][si] * wjk;
                            let node = values[offsets[0][si] + row];
                            let dn = [d[0][si], d[1][sj], d[2][sk]];
                            for r in 0..3 {
                                let wv = weight * node[r];
                                v[r] += wv;
                                b[r][0] += wv * dn[0];
                                b[r][1] += wv * dn[1];
                                b[r][2] += wv * dn[2];
                            }
                            jbar += weight * node[3];
                            if mixtures {
                                finer += weight * node_fine[offsets[0][si] + row];
                            }
                            if heat.is_some() {
                                let at = offsets[0][si] + row;
                                warmer += weight * node_dt[at];
                                let n = &node_sums[at];
                                if n.heat_capacity > 0.0 {
                                    around[0] += weight * (n.heat / n.heat_capacity + node_dt[at]);
                                    around[1] += weight;
                                }
                            }
                        }
                    }
                }
                let inv = 4.0 / (grid.h * grid.h);
                let mut c: Mat3 = b.map(|row| row.map(|x| x * inv));
                if grid.space != Space::Solid {
                    // The plane flow: nothing moves or changes along z, but the ring of a
                    // body of revolution stretches with its radius.
                    v[2] = 0.0;
                    for a in 0..3 {
                        c[2][a] = 0.0;
                        c[a][2] = 0.0;
                    }
                    if grid.space == Space::Axisymmetric {
                        c[2][2] = v[0] / grid.radius(p.x[0]);
                    }
                }
                let f: Mat3 =
                    std::array::from_fn(|r| std::array::from_fn(|s| IDENTITY[r][s] + dt * c[r][s]));
                // F-bar: the stress takes the volume ratio averaged over the neighbourhood,
                // carried on by the particle's own increment; the particle keeps its own.
                let source = if smooth && jbar > 0.0 {
                    jbar
                } else {
                    p.state.j
                };
                let j_new = source * det(&f);
                if p.grains[0] > 0.0 {
                    p.fine = (p.fine + finer).clamp(0.0, 1.0);
                }
                let m = p.material as usize;
                let mut law = laws[m].mixed(p.fine);
                if !tables[m].is_empty() {
                    law = law.at_temperature(tables[m], p.temperature);
                }
                let update = law.update(&mut p.state, &f, j_new, &c, dt);
                // The temperature: what the grid conducted and the walls gave, and the
                // heat of the particle's own flow.
                if p.heat_capacity > 0.0 {
                    p.temperature += warmer;
                    // The grid only sees the average of the particles in a cell: what a
                    // particle differs from it is conducted away within the cell, at the
                    // rate α / h² of the particle's own diffusivity.
                    if around[1] > 0.0 {
                        let alpha = p.conductivity * p.volume / p.heat_capacity;
                        let pull = (SUBCELL_CONDUCTION * alpha * dt / (grid.h * grid.h)).min(1.0);
                        p.temperature += pull * (around[0] / around[1] - p.temperature);
                    }
                    p.temperature += update.dissipated * p.volume / p.heat_capacity;
                }
                if smooth && jbar > 0.0 {
                    // The particle's own volume ratio drifts where neighbouring particles
                    // part and close (the average does not see it): it is pulled a little
                    // towards the one of the stress each step.
                    p.state.j *= (j_new / p.state.j.max(1e-300)).powf(VOLUME_RELAX);
                }
                p.tau = update.tau;
                p.c = c;
                p.v = v;
                p.x = add(p.x, scale(v, dt));
                p.shear_rate = update.shear_rate as f32;
                // Out of the walls.
                for (wall, pose) in walls.iter().zip(pose_end) {
                    if let Some((phi, n, _)) = wall.probe(p.x, pose)
                        && phi < 0.0
                    {
                        p.x = sub(p.x, scale(n, phi));
                    }
                }
                // The box: wrap, hold, or let go.
                for a in 0..3 {
                    let (lo, hi) = (domain.min[a], domain.max[a]);
                    if grid.periodic[a] {
                        p.x[a] = lo + (p.x[a] - lo).rem_euclid(hi - lo);
                        continue;
                    }
                    for (side, out) in [(2 * a, p.x[a] < lo), (2 * a + 1, p.x[a] > hi)] {
                        if out {
                            match domain.sides[side] {
                                Side::Wall(_) => p.x[a] = p.x[a].clamp(lo, hi),
                                Side::Open => p.gone = true,
                            }
                        }
                    }
                }
                if !feeds.is_empty() && !p.gone {
                    inlet::pass(p, feeds, grid.h);
                }
                let speed = norm(p.v);
                let energy = 0.5 * p.mass * speed * speed - p.mass * dot(p.gravity, p.x)
                    + update.stored * p.volume;
                let mut sums = Sums {
                    dissipated: update.dissipated * p.volume,
                    ..Sums::default()
                };
                if p.gone {
                    sums.out_mass = p.mass;
                    sums.out_energy = energy;
                } else {
                    sums.kinetic = 0.5 * p.mass * speed * speed;
                    sums.potential = -p.mass * dot(p.gravity, p.x);
                    sums.stored = update.stored * p.volume;
                    // A speed that is no number stops the run.
                    sums.max_speed = if speed.is_nan() { f64::INFINITY } else { speed };
                }
                sums
            })
            .reduce(Sums::default, Sums::join);
        dissipated += sums.dissipated;
        if sums.out_mass > 0.0 {
            outflow_mass += sums.out_mass;
            outflow_energy += sums.out_energy;
            particles.retain(|p| !p.gone);
        }
        if !feeds.is_empty() {
            let (mass, energy) = inlet::spawn(
                model,
                grid,
                walls,
                pose_end,
                &mut particles,
                &mut next_id,
                particle_energy,
            );
            inflow_mass += mass;
            inflow_energy += energy;
        }
        split(grid, domain, walls, pose_end, &mut particles, &mut next_id);

        t += dt;
        steps += 1;
        let now = sums;
        max_speed = now.max_speed;
        max_mach = max_mach.max(max_speed / slowest);
        let potential = now.potential - potential0;
        let total = now.kinetic + potential + now.stored + dissipated + wall_loss + outflow_energy
            - inflow_energy;
        scale_energy = scale_energy
            .max(now.kinetic + potential.abs() + now.stored + dissipated + wall_loss.abs())
            .max(wall_work.abs())
            .max(inflow_energy.abs())
            .max(outflow_energy.abs());

        if t >= next_history as f64 * history_dt * (1.0 - 1e-12) || t >= end * (1.0 - 1e-12) {
            next_history = (t / history_dt).floor() as usize + 1;
            history.time.push(t);
            history.time_step.push(dt);
            history.kinetic.push(now.kinetic);
            history.potential.push(potential);
            history.elastic.push(now.stored);
            history.dissipated.push(dissipated);
            // What the walls took out is what they dissipated less what they put in.
            history.wall_loss.push(wall_loss + wall_work);
            history.wall_work.push(wall_work);
            history.outflow_energy.push(outflow_energy);
            history.balance_error.push(energy0 - total);
            history.mass.push(particles.iter().map(|p| p.mass).sum());
            history.outflow.push(outflow_mass);
            history.inflow.push(inflow_mass);
            history.inflow_energy.push(inflow_energy);
            history.mach.push(max_speed / slowest);
            // Per body: mass, mass times temperature, highest temperature.
            let count = history.bodies.len();
            let per_body = (particles.par_iter())
                .fold(
                    || vec![(0.0, 0.0, f64::NEG_INFINITY); count],
                    |mut sums, p| {
                        let s = &mut sums[p.body as usize];
                        *s = (
                            s.0 + p.mass,
                            s.1 + p.mass * p.temperature,
                            s.2.max(p.temperature),
                        );
                        sums
                    },
                )
                .reduce(
                    || vec![(0.0, 0.0, f64::NEG_INFINITY); count],
                    |a, b| {
                        (a.iter().zip(&b))
                            .map(|(a, b)| (a.0 + b.0, a.1 + b.1, a.2.max(b.2)))
                            .collect()
                    },
                );
            for (h, (mass, heat, hottest)) in history.bodies.iter_mut().zip(per_body) {
                h.mass.push(mass);
                let mean = h.temperature_mean.last().copied().unwrap_or(0.0);
                // A body that left the model keeps its last temperature.
                let (mean, hottest) = if mass > 0.0 {
                    (heat / mass, hottest)
                } else {
                    (mean, h.temperature_max.last().copied().unwrap_or(mean))
                };
                h.temperature_mean.push(mean);
                h.temperature_max.push(hottest);
            }
            let n = wall_samples.max(1) as f64;
            for (h, f) in history.walls.iter_mut().zip(&mut wall_force) {
                let (force, moment) = in_plane(grid.space, f);
                h.force.push(scale(force, 1.0 / n));
                h.moment.push(scale(moment, 1.0 / n));
                let done = h.work.last().copied().unwrap_or(0.0) + f.work;
                h.work.push(done);
                let given = h.heat.last().copied().unwrap_or(0.0) + f.heat;
                h.heat.push(given);
                *f = Load::default();
            }
            wall_samples = 0;
        }
        if t >= target * (1.0 - 1e-12) {
            on_frame(frame(t, &particles, grid));
            next_frame += 1;
        }
        if last_report.elapsed().as_secs_f64() > 0.2 {
            last_report = Instant::now();
            progress(Status {
                time: t,
                end_time: end,
                steps,
                particles: particles.len(),
                time_step: dt,
                on_card: false,
            });
        }
        if particles.is_empty() {
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
        particles: particles.len(),
    })
}

/// The change of the temperature of every node in a step: the heat `given` there, plus
/// what flows in from the six neighbours by conduction. Between two nodes the conductance is
/// `k h` times the smaller filling of the two, so that a node near a free surface, with little
/// capacity, conducts as little and the explicit step stays stable.
fn conduct(grid: &Grid, nodes: &[NodeSum], given: &[f64], dt: f64, change: &mut [f64]) {
    // The temperature and the conductivity times the filling of a node.
    let state = |n: &NodeSum, x: f64| {
        if n.heat_capacity <= 0.0 || n.mass <= 0.0 {
            return None;
        }
        let filling = (n.space / (grid.h * grid.h * grid.depth(x))).min(1.0);
        Some((n.heat / n.heat_capacity, n.conduction / n.mass * filling))
    };
    change.par_iter_mut().enumerate().for_each(|(at, out)| {
        let node = &nodes[at];
        let i = [
            at % grid.n[0],
            (at / grid.n[0]) % grid.n[1],
            at / (grid.n[0] * grid.n[1]),
        ];
        let x = grid.position(i)[0];
        let Some((t, k)) = state(node, x) else {
            *out = 0.0;
            return;
        };
        let mut flow = 0.0;
        for a in 0..3 {
            for step in [-1isize, 1] {
                let next = i[a] as isize + step;
                if !grid.periodic[a] && (next < 0 || next >= grid.n[a] as isize) {
                    continue;
                }
                let mut j = i;
                j[a] = grid.wrap(a, next);
                if j == i {
                    continue;
                }
                let xj = grid.position(j)[0];
                if let Some((tj, kj)) = state(&nodes[grid.index(j)], xj) {
                    flow += k.min(kj) * grid.depth(0.5 * (x + xj)) * (tj - t);
                }
            }
        }
        *out = (flow * dt + given[at]) / node.heat_capacity;
    });
}

/// The segregation coefficients of a particle: the speed at which its fine grains sink per
/// unit of shear rate, times the shear rate, and the diffusivity of its grains (zero for
/// particles of one grain size).
fn grain_motion(p: &Particle) -> (f64, f64) {
    let [fine, coarse] = p.grains;
    if fine <= 0.0 {
        return (0.0, 0.0);
    }
    let rate = f64::from(p.shear_rate);
    let mean = p.fine * fine + (1.0 - p.fine) * coarse;
    (
        SEGREGATION * fine * (coarse / fine).max(1.0).ln() * rate,
        GRAIN_DIFFUSION * rate * mean * mean,
    )
}

/// The change of the fine fraction of every node of grain mixtures in a step (Gray &
/// Thornton 2005; Fan et al. 2014): the fine grains sink along the gravity at the speed
/// `S γ̇ (1 − φ)` relative to the mixture, the coarse ones rise, and both diffuse at
/// `D = C γ̇ d̄²`. Between two neighbouring nodes the fine grains of the upper one swap
/// with the coarse ones of the lower one, at the rate `S γ̇ φ_upper (1 − φ_lower)`, which
/// keeps the fraction between 0 and 1; the diffusion follows the difference. As for the
/// conduction the exchange is weighted with the smaller mass of the two.
fn segregate(grid: &Grid, nodes: &[NodeSum], dt: f64, change: &mut [f64]) {
    // The fine fraction, the speed of segregation, the diffusivity and the direction of
    // the gravity at a node.
    let state = |n: &NodeSum| {
        if n.mixture <= 0.0 {
            return None;
        }
        let g = norm(n.weight);
        let down = if g > 0.0 {
            scale(n.weight, 1.0 / g)
        } else {
            [0.0; 3]
        };
        Some((
            n.fine / n.mixture,
            n.sinking / n.mixture,
            n.diffusion / n.mixture,
            down,
            n.mixture,
        ))
    };
    change.par_iter_mut().enumerate().for_each(|(at, out)| {
        *out = 0.0;
        let Some((phi, speed, diffusion, down, mass)) = state(&nodes[at]) else {
            return;
        };
        let i = [
            at % grid.n[0],
            (at / grid.n[0]) % grid.n[1],
            at / (grid.n[0] * grid.n[1]),
        ];
        let mut inflow = 0.0;
        for a in 0..3 {
            for step in [-1isize, 1] {
                let next = i[a] as isize + step;
                if !grid.periodic[a] && (next < 0 || next >= grid.n[a] as isize) {
                    continue;
                }
                let mut j = i;
                j[a] = grid.wrap(a, next);
                if j == i {
                    continue;
                }
                let Some((phj, speed_j, diffusion_j, down_j, mass_j)) =
                    state(&nodes[grid.index(j)])
                else {
                    continue;
                };
                // The speed of the fine grains from this node towards the neighbour.
                let toward = 0.5 * (speed * down[a] + speed_j * down_j[a]) * step as f64;
                let swap = if toward > 0.0 {
                    toward * phi * (1.0 - phj)
                } else {
                    toward * phj * (1.0 - phi)
                };
                let spread = 0.5 * (diffusion + diffusion_j) * (phi - phj) / grid.h;
                inflow -= mass.min(mass_j) / grid.h * (swap + spread);
            }
        }
        *out = inflow * dt / mass;
    });
}

/// The force and the moment on a wall that a 2D model has: in the plane, and about z; of a
/// body of revolution only the force along its axis, the rest cancels around the ring.
fn in_plane(space: Space, load: &Load) -> (Vec3, Vec3) {
    let [fx, fy, _] = load.force;
    match space {
        Space::Solid => (load.force, load.moment),
        Space::PlaneStrain => ([fx, fy, 0.0], [0.0, 0.0, load.moment[2]]),
        Space::Axisymmetric => ([0.0, fy, 0.0], [0.0; 3]),
    }
}

/// Adds the force a node exerts on a wall, `m (v_before − v_after) / Δt`, its moment
/// about the origin and the work the wall, at velocity `wall`, does on it in the step.
fn record(sum: &mut Load, mass: f64, before: Vec3, after: Vec3, dt: f64, x: Vec3, wall: Vec3) {
    let force = scale(sub(before, after), mass / dt);
    sum.force = add(sum.force, force);
    sum.moment = add(sum.moment, cross(x, force));
    sum.work -= dot(force, wall) * dt;
}

fn particle_sums(particles: &[Particle]) -> Sums {
    // The stored energy is summed from the stress and the volume ratio: τ:ε/2 per unit
    // initial volume for the Hencky law equals the strain energy.
    particles
        .par_iter()
        .map(|p| {
            let speed = norm(p.v);
            let stored = stored_from_stress(p);
            Sums {
                kinetic: 0.5 * p.mass * speed * speed,
                potential: -p.mass * dot(p.gravity, p.x),
                stored: stored * p.volume,
                max_speed: speed,
                ..Sums::default()
            }
        })
        .reduce(Sums::default, Sums::join)
}

/// The kinetic, potential and stored energy of a particle.
fn particle_energy(p: &Particle) -> f64 {
    let speed = norm(p.v);
    0.5 * p.mass * speed * speed - p.mass * dot(p.gravity, p.x) + stored_from_stress(p) * p.volume
}

/// The strain energy per unit initial volume, `½ τ : ε` of the Hencky law, with the
/// logarithmic strain from the state.
fn stored_from_stress(p: &Particle) -> f64 {
    let j = p.state.j.max(1e-300);
    if p.state.b == IDENTITY {
        // A fluid: the pressure part only.
        let tau_mean = (p.tau[0][0] + p.tau[1][1] + p.tau[2][2]) / 3.0;
        return 0.5 * tau_mean * j.ln();
    }
    let (lambda, vectors) = plxs_materials::finite::eigen_symmetric(&p.state.b);
    let mut energy = 0.0;
    for a in 0..3 {
        let e = 0.5 * lambda[a].max(1e-300).ln();
        let v = [vectors[0][a], vectors[1][a], vectors[2][a]];
        energy += 0.5 * e * dot(v, mat_vec(&p.tau, v));
    }
    energy
}

/// The particles spread mass, affine momentum and the force of their stress to the grid.
fn spread(grid: &Grid, particles: &[Particle], dt: f64, nodes: &mut [NodeSum]) {
    let blocks: [usize; 3] = std::array::from_fn(|a| grid.n[a].div_ceil(BLOCK) + 1);
    let block_of = |p: &Particle| {
        let (base, _, _) = grid.stencil(p.x);
        let b: [usize; 3] = std::array::from_fn(|a| {
            let i = if grid.periodic[a] {
                base[a].rem_euclid(grid.n[a] as isize)
            } else {
                base[a].max(0)
            };
            (i as usize / BLOCK).min(blocks[a] - 1)
        });
        b[0] + blocks[0] * (b[1] + blocks[1] * b[2])
    };
    // Counting sort of the particles by block.
    let keys: Vec<usize> = particles.par_iter().map(block_of).collect();
    let count = blocks[0] * blocks[1] * blocks[2];
    let mut start = vec![0usize; count + 1];
    for &k in &keys {
        start[k + 1] += 1;
    }
    for i in 0..count {
        start[i + 1] += start[i];
    }
    let mut order = vec![0usize; particles.len()];
    let mut fill = start.clone();
    for (p, &k) in keys.iter().enumerate() {
        order[fill[k]] = p;
        fill[k] += 1;
    }
    let busy: Vec<usize> = (0..count).filter(|&b| start[b + 1] > start[b]).collect();
    let inv = 4.0 / (grid.h * grid.h);
    let patches: Vec<(usize, Vec<NodeSum>)> = busy
        .par_iter()
        .map(|&b| {
            let corner: [isize; 3] = [
                ((b % blocks[0]) * BLOCK) as isize,
                (((b / blocks[0]) % blocks[1]) * BLOCK) as isize,
                ((b / (blocks[0] * blocks[1])) * BLOCK) as isize,
            ];
            let mut patch = vec![NodeSum::default(); PATCH * PATCH * PATCH];
            for &p in &order[start[b]..start[b + 1]] {
                let p = &particles[p];
                let (base, w, frac) = grid.stencil(p.x);
                // The stencil relative to the block, wrapped like the block key.
                let rel: [usize; 3] = std::array::from_fn(|a| {
                    let i = if grid.periodic[a] {
                        base[a].rem_euclid(grid.n[a] as isize)
                    } else {
                        base[a].max(0)
                    };
                    (i - corner[a]) as usize
                });
                // Affine momentum with the stress force folded in (MLS-MPM).
                let mut q = [[0.0; 3]; 3];
                for r in 0..3 {
                    for c in 0..3 {
                        q[r][c] = p.mass * p.c[r][c] - dt * p.volume * inv * p.tau[r][c];
                    }
                }
                let mv = scale(p.v, p.mass);
                let mj = p.mass * p.state.j;
                let (sinking, diffusion) = grain_motion(p);
                let stress = p.tau.map(|row| row.map(|t| -p.volume * inv * t));
                // The hoop stress of a ring pulls it towards the axis.
                let hoop = if grid.space == Space::Axisymmetric {
                    -p.volume * p.tau[2][2] / grid.radius(p.x[0])
                } else {
                    0.0
                };

                for (sk, wk) in w[2].iter().enumerate() {
                    for (sj, wj) in w[1].iter().enumerate() {
                        for (si, wi) in w[0].iter().enumerate() {
                            let weight = wi * wj * wk;
                            let d = [
                                (si as f64 - frac[0]) * grid.h,
                                (sj as f64 - frac[1]) * grid.h,
                                (sk as f64 - frac[2]) * grid.h,
                            ];
                            let qd = mat_vec(&q, d);
                            let fd = mat_vec(&stress, d);
                            let at =
                                (rel[0] + si) + PATCH * ((rel[1] + sj) + PATCH * (rel[2] + sk));
                            let node = &mut patch[at];
                            node.mass += weight * p.mass;
                            for r in 0..3 {
                                node.weight[r] += weight * p.mass * p.gravity[r];
                            }
                            node.volume += weight * mj;
                            node.heat_capacity += weight * p.heat_capacity;
                            node.heat += weight * p.heat_capacity * p.temperature;
                            node.conduction += weight * p.mass * p.conductivity;
                            node.space += weight * p.volume * p.state.j;
                            if p.grains[0] > 0.0 {
                                let wm = weight * p.mass;
                                node.mixture += wm;
                                node.fine += wm * p.fine;
                                node.sinking += wm * sinking;
                                node.diffusion += wm * diffusion;
                            }
                            for r in 0..3 {
                                node.momentum[r] += weight * (mv[r] + qd[r]);
                                node.force[r] += weight * fd[r];
                            }
                            node.momentum[0] += weight * dt * hoop;
                            node.force[0] += weight * hoop;
                        }
                    }
                }
            }
            (b, patch)
        })
        .collect();
    nodes.par_iter_mut().for_each(|n| *n = NodeSum::default());
    for (b, patch) in patches {
        let corner: [isize; 3] = [
            ((b % blocks[0]) * BLOCK) as isize,
            (((b / blocks[0]) % blocks[1]) * BLOCK) as isize,
            ((b / (blocks[0] * blocks[1])) * BLOCK) as isize,
        ];
        for k in 0..PATCH {
            let gk = corner[2] + k as isize;
            if !grid.periodic[2] && gk >= grid.n[2] as isize {
                continue;
            }
            let gk = grid.wrap(2, gk);
            for j in 0..PATCH {
                let gj = corner[1] + j as isize;
                if !grid.periodic[1] && gj >= grid.n[1] as isize {
                    continue;
                }
                let gj = grid.wrap(1, gj);
                for i in 0..PATCH {
                    let src = &patch[i + PATCH * (j + PATCH * k)];
                    if src.mass == 0.0 {
                        continue;
                    }
                    let gi = corner[0] + i as isize;
                    if !grid.periodic[0] && gi >= grid.n[0] as isize {
                        continue;
                    }
                    let gi = grid.wrap(0, gi);
                    let dst = &mut nodes[grid.index([gi, gj, gk])];
                    dst.mass += src.mass;
                    dst.volume += src.volume;
                    dst.momentum = add(dst.momentum, src.momentum);
                    dst.force = add(dst.force, src.force);
                    dst.weight = add(dst.weight, src.weight);
                    dst.heat_capacity += src.heat_capacity;
                    dst.heat += src.heat;
                    dst.conduction += src.conduction;
                    dst.space += src.space;
                    dst.mixture += src.mixture;
                    dst.fine += src.fine;
                    dst.sinking += src.sinking;
                    dst.diffusion += src.diffusion;
                }
            }
        }
    }
}

/// The nodes beyond the axis of a body of revolution stand for the nodes as far before it,
/// the material being the same on both sides (the velocity along the radius turned): what
/// the particles give them goes to those.
fn fold_axis(grid: &Grid, nodes: &mut [NodeSum]) {
    let axis = grid.lo[0];
    for k in 0..grid.n[2] {
        for j in 0..grid.n[1] {
            for g in 1..=axis {
                let ghost = grid.index([axis - g, j, k]);
                let src = std::mem::take(&mut nodes[ghost]);
                let dst = &mut nodes[grid.index([axis + g, j, k])];
                let turn = |v: Vec3| [-v[0], v[1], v[2]];
                dst.mass += src.mass;
                dst.volume += src.volume;
                dst.momentum = add(dst.momentum, turn(src.momentum));
                dst.force = add(dst.force, turn(src.force));
                dst.weight = add(dst.weight, turn(src.weight));
                dst.heat_capacity += src.heat_capacity;
                dst.heat += src.heat;
                dst.conduction += src.conduction;
                dst.space += src.space;
                dst.mixture += src.mixture;
                dst.fine += src.fine;
                dst.sinking += src.sinking;
                dst.diffusion += src.diffusion;
            }
        }
    }
}

/// The nodes beyond the axis take the values of their mirror images back.
fn mirror_axis(grid: &Grid, values: &mut [[f64; 4]], heat: &mut [f64], fine: &mut [f64]) {
    let axis = grid.lo[0];
    for k in 0..grid.n[2] {
        for j in 0..grid.n[1] {
            for g in 1..=axis {
                let (ghost, image) = (grid.index([axis - g, j, k]), grid.index([axis + g, j, k]));
                let v = values[image];
                values[ghost] = [-v[0], v[1], v[2], v[3]];
                heat[ghost] = heat[image];
                fine[ghost] = fine[image];
            }
        }
    }
}

/// The grid of a domain.
fn grid_of(domain: &Domain) -> Result<Grid, String> {
    let h = domain.cell;
    if h.is_nan() || h <= 0.0 {
        return Err("the cell size of the particle analysis must be positive".into());
    }
    let mut n = [0; 3];
    let mut origin = [0.0; 3];
    let mut lo = [0; 3];
    let mut hi = [0; 3];
    for a in 0..3 {
        let span = domain.max[a] - domain.min[a];
        if span.is_nan() || span <= 0.0 {
            return Err("the box of the particle analysis is empty".into());
        }
        let cells = span / h;
        let slab = a == 2 && domain.space != Space::Solid;
        if slab && (!domain.periodic[2] || (cells - 1.0).abs() > 1e-6) {
            return Err(
                "the box of a 2D particle analysis is one cell thick along z and periodic there"
                    .into(),
            );
        }
        if domain.periodic[a] && !slab {
            let whole = cells.round();
            if (cells - whole).abs() > 1e-6 * cells || whole < 3.0 {
                return Err(format!(
                    "along the periodic axis {} the box must span a whole number of cells, at \
                     least three",
                    ["x", "y", "z"][a]
                ));
            }
            n[a] = whole as usize;
            origin[a] = domain.min[a];
            lo[a] = 0;
            hi[a] = n[a];
        } else if slab {
            // One node: the three of every stencil fold onto it and the terms along z
            // cancel, which leaves the plane flow.
            n[a] = 1;
            origin[a] = domain.min[a];
            lo[a] = 0;
            hi[a] = n[a];
        } else {
            let cells = cells.ceil() as usize;
            n[a] = cells + 5;
            origin[a] = domain.min[a] - 2.0 * h;
            lo[a] = 2;
            hi[a] = 2 + cells;
        }
    }
    if domain.space == Space::Axisymmetric {
        if domain.min[0] < -1e-9 * h {
            return Err(
                "the box of a body of revolution starts at the axis or beyond it: x, the \
                 radius, is not below zero"
                    .into(),
            );
        }
        if domain.min[0] < 1e-9 * h && domain.sides[0] != Side::Wall(WallLaw::Slip) {
            return Err("the side of the box on the axis of revolution is frictionless".into());
        }
    }
    let count = n[0] as f64 * n[1] as f64 * n[2] as f64;
    if count > 4e8 {
        return Err(format!(
            "the background grid would have {count:.2e} nodes; choose a larger cell size"
        ));
    }
    Ok(Grid {
        origin,
        h,
        n,
        periodic: domain.periodic,
        space: domain.space,
        lo,
        hi,
    })
}

/// The particles of the material bodies: two per cell and axis where a body is, outside
/// the walls; where bodies overlap, the first one fills.
fn fill(
    model: &ParticleModel,
    grid: &Grid,
    laws: &[Law],
    walls: &[Wall],
) -> Result<Vec<Particle>, String> {
    let d = &model.domain;
    let s = 0.5 * grid.h;
    // A 2D slab holds one layer of particles, in its middle.
    let slab = grid.space != Space::Solid;
    let step = |a: usize| if slab && a == 2 { grid.h } else { s };
    let counts: [usize; 3] =
        std::array::from_fn(|a| ((d.max[a] - d.min[a]) / step(a) - 1e-9).ceil().max(0.0) as usize);
    let at = |a: usize, i: usize| d.min[a] + (i as f64 + 0.5) * step(a);
    let mut taken = vec![false; counts[0] * counts[1] * counts[2]];
    let mut particles = Vec::new();
    let pose: Vec<_> = walls.iter().map(|w| w.pose(0.0, 0.0)).collect();
    for (b, body) in model.bodies.iter().enumerate() {
        if laws.get(body.material).is_none() {
            return Err(format!("body {} has no material", body.name));
        }
        let Some((lo, hi)) = body.shape.bounds() else {
            continue;
        };
        let range = |a: usize| {
            let first = ((lo[a] - d.min[a]) / s - 0.5).floor().max(0.0) as usize;
            let last = (((hi[a] - d.min[a]) / s - 0.5).ceil().max(0.0) as usize + 1).min(counts[a]);
            first..last.max(first)
        };
        let (rx, ry) = (range(0), range(1));
        let rz = if slab { 0..1 } else { range(2) };
        let rows = ParityRows::new(
            &body.shape,
            [at(1, ry.start), at(2, rz.start)],
            s,
            [ry.len(), rz.len()],
        );
        for k in rz.clone() {
            for j in ry.clone() {
                for i in rx.clone() {
                    let x = [at(0, i), at(1, j), at(2, k)];
                    if !rows.inside(j - ry.start, k - rz.start, x[0]) {
                        continue;
                    }
                    let slot = i + counts[0] * (j + counts[1] * k);
                    if taken[slot] {
                        continue;
                    }
                    let in_wall = walls
                        .iter()
                        .zip(&pose)
                        .any(|(w, pose)| w.probe(x, pose).is_some_and(|(phi, _, _)| phi < 0.0));
                    if in_wall {
                        continue;
                    }
                    taken[slot] = true;
                    let id = particles.len() as u32;
                    particles.push(new_particle(model, grid, laws, b, x, id));
                }
            }
        }
    }
    Ok(particles)
}

/// A particle of body `b` at `x` that rests, as the bodies are filled.
fn new_particle(
    model: &ParticleModel,
    grid: &Grid,
    laws: &[Law],
    b: usize,
    x: Vec3,
    id: u32,
) -> Particle {
    let body = &model.bodies[b];
    let law = &laws[body.material];
    let s = 0.5 * grid.h;
    // Without heat the temperature stays: no capacity.
    let thermal = model.heat.and(model.materials[body.material].thermal);
    let (fine, grains) = match model.materials[body.material].law {
        ParticleLaw::Granular {
            grain,
            fine_grain,
            fine_fraction,
            ..
        } if fine_grain > 0.0 => (fine_fraction, [fine_grain, grain]),
        _ => (0.0, [0.0; 2]),
    };
    // A 2D slab holds one layer, standing for the depth of its radius.
    let volume = if grid.space == Space::Solid {
        s * s * s
    } else {
        s * s * grid.depth(x[0])
    };
    Particle {
        id,
        x,
        v: body.velocity,
        c: [[0.0; 3]; 3],
        state: PointState::default(),
        tau: [[0.0; 3]; 3],
        mass: law.density * volume,
        volume,
        material: body.material as u16,
        body: b as u16,
        shear_rate: 0.0,
        temperature: body.temperature,
        gravity: add(model.gravity, body.gravity),
        heat_capacity: thermal.map_or(0.0, |t| t.specific_heat * law.density * volume),
        conductivity: thermal.map_or(0.0, |t| t.conductivity),
        fine,
        grains,
        share: 1.0,
        gone: false,
        inlet: 0,
        spawn: None,
    }
}

/// Splits the particles of a 2D model whose area in the plane grew beyond [`SPLIT_AREA`]
/// times the spacing squared in two halves, side by side along the direction the
/// material stretches fastest; the new halves are numbered from `next_id` on. Mass, volume,
/// heat capacity and momentum are shared; stress, velocity and its gradient are taken over.
fn split(
    grid: &Grid,
    domain: &plxs_core::particles::Domain,
    walls: &[Wall],
    pose: &[Pose],
    particles: &mut Vec<Particle>,
    next_id: &mut u32,
) {
    if grid.space == Space::Solid {
        return;
    }
    let s = 0.5 * grid.h;
    let area = |p: &Particle| p.volume * p.state.j / grid.depth(p.x[0]);
    let limit = SPLIT_AREA * s * s;
    // The layers of the inlets are made anew as the material moves on.
    let splits = |p: &Particle| p.inlet == 0 && area(p) > limit;
    if !particles.par_iter().any(splits) {
        return;
    }
    let count = particles.len();
    for k in 0..count {
        let a = area(&particles[k]);
        if !splits(&particles[k]) {
            continue;
        }
        let p = &mut particles[k];
        // The direction of the fastest stretch in the plane: the eigenvector of the
        // largest eigenvalue of the symmetric velocity gradient.
        let (xx, yy, xy) = (p.c[0][0], p.c[1][1], 0.5 * (p.c[0][1] + p.c[1][0]));
        let angle = 0.5 * (2.0 * xy).atan2(xx - yy);
        let along = [angle.cos(), angle.sin(), 0.0];
        // The halves sit at the middles of the halves of the particle's square.
        let offset = scale(along, 0.25 * a.sqrt());
        p.mass *= 0.5;
        p.volume *= 0.5;
        p.heat_capacity *= 0.5;
        p.share *= 0.5;
        let mut half = *p;
        half.id = *next_id;
        *next_id += 1;
        p.x = sub(p.x, offset);
        half.x = add(half.x, offset);
        for q in [&mut particles[k], &mut half] {
            place(grid, domain, walls, pose, &mut q.x);
        }
        particles.push(half);
    }
}

/// Moves a point of a split particle back into the material: out of the walls, across
/// the axis of an axisymmetric model, into the box.
fn place(
    grid: &Grid,
    domain: &plxs_core::particles::Domain,
    walls: &[Wall],
    pose: &[Pose],
    x: &mut Vec3,
) {
    for (wall, pose) in walls.iter().zip(pose) {
        if let Some((phi, n, _)) = wall.probe(*x, pose)
            && phi < 0.0
        {
            *x = sub(*x, scale(n, phi));
        }
    }
    if grid.space == Space::Axisymmetric {
        x[0] = x[0].abs();
    }
    for a in 0..3 {
        if !grid.periodic[a] {
            x[a] = x[a].clamp(domain.min[a], domain.max[a]);
        }
    }
}

/// A frame of the particles.
fn frame(time: f64, particles: &[Particle], grid: &Grid) -> ParticleFrame {
    let mut fields = vec![Vec::with_capacity(particles.len()); PARTICLE_FIELDS.len()];
    for p in particles {
        let j = p.state.j.max(1e-300);
        let pressure = -(p.tau[0][0] + p.tau[1][1] + p.tau[2][2]) / (3.0 * j);
        let dev = deviator(&p.tau);
        let j2 = 0.5 * crate::math::contract(&dev, &dev);
        let values = [
            pressure,
            j2.sqrt() / j,
            p.state.plastic,
            f64::from(p.shear_rate),
            j,
            p.temperature,
            p.fine,
            f64::from(p.body),
        ];
        for (f, v) in fields.iter_mut().zip(values) {
            f.push(v as f32);
        }
    }
    ParticleFrame {
        time,
        ids: particles.iter().map(|p| p.id).collect(),
        positions: particles.iter().map(|p| p.x.map(|x| x as f32)).collect(),
        velocities: particles.iter().map(|p| p.v.map(|x| x as f32)).collect(),
        fields,
        shares: if grid.space != Space::Solid {
            // In 2D the area in the plane: it grows where material of a body of revolution
            // nears the axis, until the particle splits.
            let s = 0.5 * grid.h;
            (particles.iter())
                .map(|p| (p.volume * p.state.j / (grid.depth(p.x[0]) * s * s)) as f32)
                .collect()
        } else if particles.iter().any(|p| p.share < 1.0) {
            particles.iter().map(|p| p.share).collect()
        } else {
            Vec::new()
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A column of a mixture at a uniform speed of segregation `q` and without diffusion
    /// separates as Gray & Thornton (2005) solved it: from the top a layer of coarse grains
    /// grows downwards at `q (1 − φ0)`, from the floor one of fine grains upwards at `q φ0`;
    /// they meet after `H/q`, the fine layer then `φ0 H` thick. The fine grains are kept.
    #[test]
    fn a_column_of_a_mixture_separates_at_the_speed_of_segregation() {
        let (count, h, q, phi0) = (60usize, 1e-3, 0.01, 0.3);
        let grid = Grid {
            origin: [0.0; 3],
            h,
            n: [1, 1, count],
            periodic: [true, true, false],
            space: Space::Solid,
            lo: [0; 3],
            hi: [0, 0, count - 1],
        };
        let mut phi = vec![phi0; count];
        let height = count as f64 * h;
        let dt = 0.2 * h / q;
        let mut change = vec![0.0; count];
        let mut t = 0.0;
        let mut checked = false;
        while t < 1.2 * height / q {
            let nodes: Vec<NodeSum> = (phi.iter())
                .map(|&f| NodeSum {
                    mass: 1.0,
                    weight: [0.0, 0.0, -1.0],
                    mixture: 1.0,
                    fine: f,
                    sinking: q,
                    ..Default::default()
                })
                .collect();
            segregate(&grid, &nodes, dt, &mut change);
            for (f, d) in phi.iter_mut().zip(&change) {
                *f += d;
                assert!((-1e-12..=1.0 + 1e-12).contains(f), "{f}");
            }
            t += dt;
            let total: f64 = phi.iter().sum();
            assert!(
                (total - phi0 * count as f64).abs() < 1e-9,
                "fine grains lost"
            );
            // Half way: the layers where the exact solution has them.
            if !checked && t >= 0.5 * height / q {
                checked = true;
                let coarse = (phi.iter().rev()).take_while(|&&f| f < 0.05).count() as f64 * h;
                let fine = phi.iter().take_while(|&&f| f > 0.95).count() as f64 * h;
                let (coarse_exact, fine_exact) = (q * (1.0 - phi0) * t, q * phi0 * t);
                eprintln!(
                    "t {t:.3}: coarse {coarse:.4} ({coarse_exact:.4}) fine {fine:.4} ({fine_exact:.4})"
                );
                assert!((coarse - coarse_exact).abs() < 0.1 * coarse_exact + 3.0 * h);
                assert!((fine - fine_exact).abs() < 0.1 * fine_exact + 3.0 * h);
            }
        }
        let fine = phi.iter().filter(|&&f| f > 0.5).count();
        eprintln!("fine layer {fine} of {count} nodes");
        assert!((fine as f64 - phi0 * count as f64).abs() <= 1.0);
    }
}
