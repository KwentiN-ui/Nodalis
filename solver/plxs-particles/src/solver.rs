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
    Domain, PARTICLE_FIELDS, ParticleFrame, ParticleHistory, ParticleModel, ParticleReport,
    ParticleResults, RigidMotion, Side, BodyHistory, WallHistory, WallLaw, table_value,
};
use rayon::prelude::*;

use crate::distance::{DistanceField, ParityRows};
use crate::law::{Law, PointState};
use crate::math::{
    IDENTITY, Mat3, Vec3, add, cross, det, deviator, dot, mat_vec, norm, rotation, scale, sub,
    transpose,
};

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

/// How a run is going, for a progress display.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Status {
    pub time: f64,
    pub end_time: f64,
    pub steps: usize,
    pub particles: usize,
    pub time_step: f64,
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
    /// Left the box through an open side.
    gone: bool,
}

/// The background grid.
struct Grid {
    /// The position of node 0.
    origin: Vec3,
    h: f64,
    /// Nodes per axis.
    n: [usize; 3],
    periodic: [bool; 3],
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
    /// The pose of the body at time `t`.
    fn pose(&self, t: f64) -> Pose {
        let moved = self.motion.moved_time(t);
        let shift = scale(self.motion.velocity, moved);
        Pose {
            rotation: rotation(scale(self.motion.angular, moved)),
            factor: self.motion.factor(t),
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
            ),
            law: w.law,
            motion: w.motion.clone(),
        })
        .collect();
    let mut particles = fill(model, &grid, &laws, &walls)?;
    if particles.is_empty() {
        return Err("no particles: the material bodies lie outside the box or inside walls".into());
    }
    let wave = laws.iter().map(Law::wave_speed).fold(0.0, f64::max);
    let slowest = laws
        .iter()
        .map(Law::wave_speed)
        .fold(f64::INFINITY, f64::min);
    // The factors of the flow over the temperature, per material.
    let factors: Vec<&[[f64; 2]]> = (model.materials.iter())
        .map(|m| m.flow_factor.as_slice())
        .collect();
    let highest = |m: usize| (factors[m].iter().map(|p| p[1])).fold(1.0f64, f64::max);
    let viscosity_limit = (laws.iter().enumerate())
        .filter(|(_, l)| l.explicit_viscosity() > 0.0)
        .map(|(m, l)| l.density * grid.h * grid.h / (8.0 * l.explicit_viscosity() * highest(m)))
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
    let control = &model.control;
    let end = control.end_time;
    let frames = control.frames.max(1);
    let frame_dt = end / frames as f64;
    let history_dt = end / HISTORY_POINTS as f64;

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
    let potential0 = initial.potential;
    let energy0 = initial.kinetic + initial.stored;
    let mut scale_energy = energy0.abs();
    let mut dissipated = 0.0;
    let mut wall_loss = 0.0;
    // The work of the moving walls on the material.
    let mut wall_work = 0.0;
    let mut outflow_energy = 0.0;
    let mut outflow_mass = 0.0;
    let mut wall_force = vec![Load::default(); names.len()];
    let mut wall_samples = 0usize;

    let initial_count = particles.len();
    on_frame(frame(0.0, &particles));
    let mut t = 0.0;
    let mut steps = 0usize;
    let mut next_frame = 1usize;
    let mut next_history = 0usize;
    let mut dt_range = (0.0f64, f64::INFINITY);
    let mut max_mach: f64 = 0.0;
    let mut max_speed = particles.iter().map(|p| norm(p.v)).fold(0.0, f64::max);
    let mut last_report = Instant::now();
    let mut stopped = false;
    let mut nodes = vec![NodeSum::default(); grid.count()];
    // Per node: the velocity at the end of the step and the averaged volume ratio.
    let mut values = vec![[0.0; 4]; grid.count()];
    // Per node: the heat the walls gave in the step, and the change of the temperature.
    let mut node_heat = vec![0.0; grid.count()];
    let mut node_dt = vec![0.0; grid.count()];

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
        if dt.is_nan() || dt <= 0.0 {
            return Err("the time step became zero".into());
        }
        dt_range = (dt_range.0.max(dt), dt_range.1.min(dt));

        // (1) Particles to grid.
        spread(&grid, &particles, dt, &mut nodes);
        // (2) The grid.
        let pose: Vec<_> = walls.iter().map(|w| w.pose(t + dt)).collect();
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
                            let flow = heat.wall_transfer * grid.h * grid.h * gap * dt;
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

        // (2b) The heat: conduction between neighbouring nodes, and what the walls gave.
        if heat.is_some() {
            conduct(&grid, &nodes, &node_heat, dt, &mut node_dt);
        }
        let node_dt = &node_dt;
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
                let c: Mat3 = b.map(|row| row.map(|x| x * inv));
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
                let law = &laws[p.material as usize];
                let factor = table_value(factors[p.material as usize], p.temperature);
                let update = law.update(&mut p.state, &f, j_new, &c, dt, factor);
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
                    sums.max_speed = speed;
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

        t += dt;
        steps += 1;
        let now = sums;
        max_speed = now.max_speed;
        max_mach = max_mach.max(max_speed / slowest);
        let potential = now.potential - potential0;
        let total = now.kinetic + potential + now.stored + dissipated + wall_loss + outflow_energy;
        scale_energy = scale_energy
            .max(now.kinetic + potential.abs() + now.stored + dissipated + wall_loss.abs())
            .max(wall_work.abs());

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
            history.mach.push(max_speed / slowest);
            // Per body: mass, mass times temperature, highest temperature.
            let count = history.bodies.len();
            let per_body = (particles.par_iter())
                .fold(
                    || vec![(0.0, 0.0, f64::NEG_INFINITY); count],
                    |mut sums, p| {
                        let s = &mut sums[p.body as usize];
                        *s = (s.0 + p.mass, s.1 + p.mass * p.temperature, s.2.max(p.temperature));
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
        if t >= target * (1.0 - 1e-12) {
            on_frame(frame(t, &particles));
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
            });
        }
        if particles.is_empty() {
            notes.push("All material left the model.".into());
            break;
        }
    }
    progress(Status {
        time: t,
        end_time: end,
        steps,
        particles: particles.len(),
        time_step: dt_range.1,
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

/// The change of the temperature of every node in a step: the heat `given` there, plus
/// what flows in from the six neighbours by conduction. Between two nodes the conductance is
/// `k h` times the smaller filling of the two, so that a node near a free surface, with little
/// capacity, conducts as little and the explicit step stays stable.
fn conduct(grid: &Grid, nodes: &[NodeSum], given: &[f64], dt: f64, change: &mut [f64]) {
    let cell = grid.h * grid.h * grid.h;
    // The temperature and the conductivity times the filling of a node.
    let state = |n: &NodeSum| {
        if n.heat_capacity <= 0.0 || n.mass <= 0.0 {
            return None;
        }
        let filling = (n.space / cell).min(1.0);
        Some((n.heat / n.heat_capacity, n.conduction / n.mass * filling))
    };
    change.par_iter_mut().enumerate().for_each(|(at, out)| {
        let node = &nodes[at];
        let Some((t, k)) = state(node) else {
            *out = 0.0;
            return;
        };
        let i = [
            at % grid.n[0],
            (at / grid.n[0]) % grid.n[1],
            at / (grid.n[0] * grid.n[1]),
        ];
        let mut flow = 0.0;
        for a in 0..3 {
            for step in [-1isize, 1] {
                let next = i[a] as isize + step;
                if !grid.periodic[a] && (next < 0 || next >= grid.n[a] as isize) {
                    continue;
                }
                let mut j = i;
                j[a] = grid.wrap(a, next);
                if let Some((tj, kj)) = state(&nodes[grid.index(j)]) {
                    flow += k.min(kj) * grid.h * (tj - t);
                }
            }
        }
        *out = (flow * dt + given[at]) / node.heat_capacity;
    });
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
                let stress = p.tau.map(|row| row.map(|t| -p.volume * inv * t));
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
                            for r in 0..3 {
                                node.momentum[r] += weight * (mv[r] + qd[r]);
                                node.force[r] += weight * fd[r];
                            }
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
                }
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
        if domain.periodic[a] {
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
        } else {
            let cells = cells.ceil() as usize;
            n[a] = cells + 5;
            origin[a] = domain.min[a] - 2.0 * h;
            lo[a] = 2;
            hi[a] = 2 + cells;
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
    let counts: [usize; 3] =
        std::array::from_fn(|a| ((d.max[a] - d.min[a]) / s - 1e-9).ceil().max(0.0) as usize);
    let at = |a: usize, i: usize| d.min[a] + (i as f64 + 0.5) * s;
    let mut taken = vec![false; counts[0] * counts[1] * counts[2]];
    let mut particles = Vec::new();
    let pose: Vec<_> = walls.iter().map(|w| w.pose(0.0)).collect();
    for (b, body) in model.bodies.iter().enumerate() {
        let Some(law) = laws.get(body.material) else {
            return Err(format!("body {} has no material", body.name));
        };
        // Without heat the temperature stays: no capacity.
        let thermal = model.heat.and(model.materials[body.material].thermal);
        let Some((lo, hi)) = body.shape.bounds() else {
            continue;
        };
        let range = |a: usize| {
            let first = ((lo[a] - d.min[a]) / s - 0.5).floor().max(0.0) as usize;
            let last = (((hi[a] - d.min[a]) / s - 0.5).ceil().max(0.0) as usize + 1).min(counts[a]);
            first..last.max(first)
        };
        let (rx, ry, rz) = (range(0), range(1), range(2));
        let rows = ParityRows::new(
            &body.shape,
            [at(1, ry.start), at(2, rz.start)],
            s,
            [ry.len(), rz.len()],
        );
        let volume = s * s * s;
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
                    particles.push(Particle {
                        id: particles.len() as u32,
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
                        heat_capacity: thermal
                            .map_or(0.0, |t| t.specific_heat * law.density * volume),
                        conductivity: thermal.map_or(0.0, |t| t.conductivity),
                        gone: false,
                    });
                }
            }
        }
    }
    Ok(particles)
}

/// A frame of the particles.
fn frame(time: f64, particles: &[Particle]) -> ParticleFrame {
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
    }
}
