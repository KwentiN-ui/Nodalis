//! The inlets: reservoirs behind a surface that material enters the model from or leaves
//! it into.
//!
//! The layer of [`Inlet::DEPTH`] cells in front of the surface of a feeding inlet belongs to
//! its reservoir. Its particles are material like any other, but the layer is kept full: a
//! particle of the layer that moves on into the model leaves a new one behind it, one layer
//! depth back; what passes the surface backwards returns to the reservoir and leaves the
//! model. A pressure
//! inlet pushes the back of the layer with its pressure (a traction on the nodes there),
//! a velocity inlet holds the nodes of the layer at its speed. An open inlet only takes
//! what passes its surface out of the model.

use plxs_core::particles::{Inlet, InletKind, ParticleModel, Space, table_value};

use super::{Grid, Particle, Pose, Wall, place};
use crate::law::Law;
use crate::math::{Vec3, add, cross, dot, norm, scale, sub};

/// A triangle of the surface of an inlet.
struct Face {
    a: Vec3,
    /// The edges from `a`, and the dual vectors that give the barycentric coordinates.
    e: [Vec3; 2],
    dual: [Vec3; 2],
    /// The unit normal into the model.
    n: Vec3,
}

/// An inlet ready for the time loop.
pub(super) struct Feed {
    pub(super) kind: InletKind,
    amplitude: Vec<[f64; 2]>,
    pub(super) body: usize,
    faces: Vec<Face>,
    /// The bounds of the surface widened by the layer and a cell.
    lo: Vec3,
    hi: Vec3,
    /// Of a pressure inlet: the nodes that the points of the reservoir behind its surface
    /// reach and the force they put on each per unit of pressure.
    pressed: Vec<(usize, Vec3)>,
    /// Of a velocity inlet: the nodes of its layer and the normal there.
    nodes: Vec<(usize, Vec3)>,
    /// Of a pressure inlet: the nodes behind the surface that the layer reaches, and the
    /// node in front whose velocity each takes.
    behind: Vec<(usize, usize)>,
}

/// Where a point lies relative to an inlet: its distance in front of the surface and the
/// normal of the face it lies over.
#[derive(Clone, Copy)]
pub(super) struct Spot {
    pub(super) d: f64,
    pub(super) n: Vec3,
}

impl Feed {
    pub(super) fn new(inlet: &Inlet, grid: &Grid) -> Result<Self, String> {
        let h = grid.h;
        let depth = Inlet::DEPTH * h;
        let points = &inlet.shape.points;
        let mut faces = Vec::new();
        for t in &inlet.shape.triangles {
            let [a, b, c] = t.map(|i| points[i as usize]);
            let e = [sub(b, a), sub(c, a)];
            let normal = cross(e[0], e[1]);
            let area2 = norm(normal);
            if area2 <= 1e-12 * h * h {
                continue;
            }
            let n = scale(normal, 1.0 / area2);
            // The dual basis in the plane: dot(dual[i], e[j]) = δij.
            let g = [
                [dot(e[0], e[0]), dot(e[0], e[1])],
                [dot(e[0], e[1]), dot(e[1], e[1])],
            ];
            let det = g[0][0] * g[1][1] - g[0][1] * g[1][0];
            let dual = [
                scale(sub(scale(e[0], g[1][1]), scale(e[1], g[0][1])), 1.0 / det),
                scale(sub(scale(e[1], g[0][0]), scale(e[0], g[1][0])), 1.0 / det),
            ];
            faces.push(Face { a, e, dual, n });
        }
        if faces.is_empty() {
            return Err(format!("inlet {} has no surface", inlet.name));
        }
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for f in &faces {
            for p in [f.a, add(f.a, f.e[0]), add(f.a, f.e[1])] {
                for a in 0..3 {
                    lo[a] = lo[a].min(p[a] - depth - h);
                    hi[a] = hi[a].max(p[a] + depth + h);
                }
            }
        }
        let mut feed = Feed {
            kind: inlet.kind,
            amplitude: inlet.amplitude.clone(),
            body: inlet.body,
            faces,
            lo,
            hi,
            pressed: Vec::new(),
            nodes: Vec::new(),
            behind: Vec::new(),
        };
        match inlet.kind {
            InletKind::Pressure(_) => {
                feed.pressed = pressed(grid, &feed.reservoir(grid, inlet));
                feed.behind = feed.behind_nodes(grid);
            }
            InletKind::Velocity(_) => feed.nodes = feed.layer_nodes(grid),
            InletKind::Open => {}
        }
        Ok(feed)
    }

    /// The points of the reservoir: six layers behind the surface, half a cell apart (as
    /// deep as the nodes with mass of the layer reach and their stencils beyond),
    /// with the volume of a particle there (in 2D per unit length or of the whole ring,
    /// like the particles).
    fn reservoir(&self, grid: &Grid, inlet: &Inlet) -> Vec<(Vec3, f64)> {
        let s = 0.5 * grid.h;
        let (zlo, zhi) = (inlet.shape.points.iter())
            .fold((f64::INFINITY, f64::NEG_INFINITY), |r, p| {
                (r.0.min(p[2]), r.1.max(p[2]))
            });
        let mut out = Vec::new();
        for f in &self.faces {
            let longest = norm(f.e[0])
                .max(norm(f.e[1]))
                .max(norm(sub(f.e[1], f.e[0])));
            let m = (longest / s).ceil().max(1.0) as usize;
            let area = 0.5 * norm(cross(f.e[0], f.e[1])) / (m * m) as f64;
            let at = |u: f64, v: f64| add(f.a, add(scale(f.e[0], u), scale(f.e[1], v)));
            let k = 1.0 / m as f64;
            for i in 0..m {
                for j in 0..m - i {
                    let (u, v) = (i as f64 * k, j as f64 * k);
                    // The upright and, but in the last row, the upside-down sub-triangle.
                    let mut centres = vec![at(u + k / 3.0, v + k / 3.0)];
                    if i + j + 1 < m {
                        centres.push(at(u + 2.0 * k / 3.0, v + 2.0 * k / 3.0));
                    }
                    for x in centres {
                        let a = if grid.space == Space::Solid {
                            area
                        } else {
                            // The surface spans the slab: its area per depth of the slab.
                            area * grid.depth(x[0]) / (zhi - zlo).max(1e-300)
                        };
                        for layer in [0.5, 1.5, 2.5, 3.5, 4.5, 5.5] {
                            out.push((sub(x, scale(f.n, layer * s)), a * s));
                        }
                    }
                }
            }
        }
        out
    }

    /// The nodes over the surface between `from` and `to` in front of it: index, position
    /// and where they lie.
    fn nodes_between(&self, grid: &Grid, from: f64, to: f64) -> Vec<([usize; 3], Spot)> {
        let range = |a: usize| {
            let first = ((self.lo[a] - grid.origin[a]) / grid.h).floor().max(0.0) as usize;
            let last =
                (((self.hi[a] - grid.origin[a]) / grid.h).ceil() as usize + 1).min(grid.n[a]);
            first..last.max(first)
        };
        let mut out = Vec::new();
        for k in range(2) {
            for j in range(1) {
                for i in range(0) {
                    let mut x = grid.position([i, j, k]);
                    if grid.space != Space::Solid {
                        // The one node of the slab stands for its middle.
                        x[2] = 0.5 * (self.lo[2] + self.hi[2]);
                    }
                    if let Some(spot) = self.locate(x, grid.h)
                        && spot.d >= from
                        && spot.d <= to
                    {
                        out.push(([i, j, k], spot));
                    }
                }
            }
        }
        out
    }

    /// The nodes of the layer in front of the surface.
    fn layer_nodes(&self, grid: &Grid) -> Vec<(usize, Vec3)> {
        let depth = Inlet::DEPTH * grid.h;
        (self.nodes_between(grid, 0.0, depth).into_iter())
            .map(|(i, spot)| (grid.index(i), spot.n))
            .collect()
    }

    /// The nodes behind the surface that the layer reaches, each with the node in front
    /// of the surface whose velocity it takes.
    fn behind_nodes(&self, grid: &Grid) -> Vec<(usize, usize)> {
        let h = grid.h;
        (self.nodes_between(grid, -1.5 * h, -1e-9 * h).into_iter())
            .filter_map(|(i, spot)| {
                let x = grid.position(i);
                let ahead = add(x, scale(spot.n, -spot.d + 0.5 * h));
                let j: [usize; 3] = std::array::from_fn(|a| {
                    if grid.n[a] == 1 {
                        0
                    } else {
                        ((ahead[a] - grid.origin[a]) / h)
                            .round()
                            .clamp(0.0, (grid.n[a] - 1) as f64) as usize
                    }
                });
                (j != i).then(|| (grid.index(i), grid.index(j)))
            })
            .collect()
    }

    /// The factor of the amplitude at time `t`.
    pub(super) fn factor(&self, t: f64) -> f64 {
        table_value(&self.amplitude, t)
    }

    /// Where `x` lies over the surface, if it does and is within a cell behind it or the
    /// layer and a cell in front of it.
    pub(super) fn locate(&self, x: Vec3, h: f64) -> Option<Spot> {
        if (0..3).any(|a| x[a] < self.lo[a] || x[a] > self.hi[a]) {
            return None;
        }
        let reach = Inlet::DEPTH * h + h;
        let mut best: Option<Spot> = None;
        for f in &self.faces {
            let r = sub(x, f.a);
            let d = dot(r, f.n);
            if d < -h || d > reach {
                continue;
            }
            let (u, v) = (dot(r, f.dual[0]), dot(r, f.dual[1]));
            let eps = 1e-9;
            if u < -eps || v < -eps || u + v > 1.0 + eps {
                continue;
            }
            if best.is_none_or(|b| d.abs() < b.d.abs()) {
                best = Some(Spot { d, n: f.n });
            }
        }
        best
    }

    /// The force of the pressure on the nodes around the back of the layer at time `t`,
    /// pushed onto `forces` as (node, force). The reservoir behind the surface is material
    /// at the pressure: points behind the surface, as many and as large as the particles
    /// of six layers, which carry the stress of the pressure but no mass. On the nodes
    /// that hold material they act as the reservoir's material would, so that the stress
    /// of the layer meets its like across the surface and the nodes behind it, with
    /// little mass, stay at rest under the two.
    pub(super) fn push(
        &self,
        t: f64,
        mass: impl Fn(usize) -> f64,
        forces: &mut Vec<(usize, Vec3)>,
    ) {
        let InletKind::Pressure(p) = self.kind else {
            return;
        };
        let p = p * self.factor(t);
        if p == 0.0 {
            return;
        }
        for &(at, unit) in &self.pressed {
            if mass(at) > 0.0 {
                forces.push((at, scale(unit, p)));
            }
        }
    }

    /// The nodes behind the surface of a pressure inlet with the node whose velocity they
    /// take: they hold only the edge of the layer, little mass between the stress of the
    /// layer and that of the reservoir, and would take any speed the two leave over (the
    /// viscous stress of the layer, which the reservoir lacks).
    pub(super) fn behind(&self) -> &[(usize, usize)] {
        &self.behind
    }

    /// The nodes of a velocity inlet's layer and the velocity they take at time `t`.
    pub(super) fn held(&self, t: f64) -> impl Iterator<Item = (usize, Vec3)> + '_ {
        let speed = match self.kind {
            InletKind::Velocity(v) => v * self.factor(t),
            _ => 0.0,
        };
        self.nodes.iter().map(move |&(at, n)| (at, scale(n, speed)))
    }
}

/// What happens to a particle that moved at an inlet: it leaves through the surface
/// (`gone`), or it leaves the layer of a feeding inlet into the model, which marks where
/// its successor is made (`spawn`). Material of the model that moves back into a layer
/// does not join it: a particle that went back and forth would leave a successor every
/// time.
pub(super) fn pass(p: &mut Particle, feeds: &[Feed], h: f64) {
    let depth = Inlet::DEPTH * h;
    if p.inlet > 0 {
        let k = p.inlet as usize - 1;
        match feeds[k].locate(p.x, h) {
            Some(spot) if spot.d < 0.0 => p.gone = true,
            Some(spot) if spot.d < depth => {}
            Some(spot) => {
                p.inlet = 0;
                p.spawn = Some((sub(p.x, scale(spot.n, depth)), k as u16 + 1));
            }
            // Out of the layer sideways: material of the model from now on.
            None => p.inlet = 0,
        }
        return;
    }
    if feeds
        .iter()
        .any(|f| f.locate(p.x, h).is_some_and(|s| s.d < 0.0))
    {
        p.gone = true;
    }
}

/// Fills the layers of the feeding inlets at the start: what material is there is taken
/// out, and the layer is filled with the inlet's material on the lattice of the particles.
pub(super) fn prime(
    model: &ParticleModel,
    grid: &Grid,
    laws: &[Law],
    feeds: &[Feed],
    walls: &[Wall],
    particles: &mut Vec<Particle>,
) {
    let depth = Inlet::DEPTH * grid.h;
    let in_layer = |x: Vec3| {
        (feeds.iter().enumerate())
            .filter(|(_, f)| f.kind.feeds())
            .find_map(|(k, f)| {
                f.locate(x, grid.h)
                    .filter(|s| (0.0..depth).contains(&s.d))
                    .map(|_| k)
            })
    };
    particles.retain(|p| in_layer(p.x).is_none());
    let d = &model.domain;
    let s = 0.5 * grid.h;
    let slab = grid.space != Space::Solid;
    let pose: Vec<Pose> = walls.iter().map(|w| w.pose(0.0, 0.0)).collect();
    let mut next = particles.iter().map(|p| p.id + 1).max().unwrap_or(0);
    for (k, feed) in feeds.iter().enumerate() {
        if !feed.kind.feeds() {
            continue;
        }
        let range = |a: usize| {
            let lo = feed.lo[a].max(d.min[a]);
            let hi = feed.hi[a].min(d.max[a]);
            let first = ((lo - d.min[a]) / s - 0.5).floor().max(0.0) as usize;
            let last = ((hi - d.min[a]) / s - 0.5).ceil().max(0.0) as usize + 1;
            first..last.max(first)
        };
        let rz = if slab { 0..1 } else { range(2) };
        for kz in rz {
            for j in range(1) {
                for i in range(0) {
                    let z = if slab {
                        0.5 * (d.min[2] + d.max[2])
                    } else {
                        d.min[2] + (kz as f64 + 0.5) * s
                    };
                    let x = [
                        d.min[0] + (i as f64 + 0.5) * s,
                        d.min[1] + (j as f64 + 0.5) * s,
                        z,
                    ];
                    if (0..3).any(|a| x[a] < d.min[a] || x[a] > d.max[a]) || in_layer(x) != Some(k)
                    {
                        continue;
                    }
                    let in_wall = (walls.iter().zip(&pose))
                        .any(|(w, pose)| w.probe(x, pose).is_some_and(|(phi, _, _)| phi < 0.0));
                    if in_wall {
                        continue;
                    }
                    let mut p = super::new_particle(model, grid, laws, feed.body, x, next);
                    next += 1;
                    p.inlet = k as u16 + 1;
                    if let InletKind::Velocity(v) = feed.kind
                        && let Some(spot) = feed.locate(x, grid.h)
                    {
                        p.v = scale(spot.n, v * feed.factor(0.0));
                    }
                    particles.push(p);
                }
            }
        }
    }
}

/// Makes the successors of the particles that left the layers into the model, numbered
/// from `next_id` on; returns their mass and energy.
#[allow(clippy::too_many_arguments)]
pub(super) fn spawn(
    model: &ParticleModel,
    grid: &Grid,
    walls: &[Wall],
    pose: &[Pose],
    particles: &mut Vec<Particle>,
    next_id: &mut u32,
    energy: impl Fn(&Particle) -> f64,
) -> (f64, f64) {
    let mut mass = 0.0;
    let mut brought = 0.0;
    let count = particles.len();
    for k in 0..count {
        let Some((mut x, inlet)) = particles[k].spawn.take() else {
            continue;
        };
        place(grid, &model.domain, walls, pose, &mut x);
        let mut q = particles[k];
        let ratio = grid.depth(x[0]) / grid.depth(q.x[0]);
        q.id = *next_id;
        *next_id += 1;
        q.x = x;
        q.mass *= ratio;
        q.volume *= ratio;
        q.heat_capacity *= ratio;
        let body = &model.bodies[q.body as usize];
        q.temperature = body.temperature;
        q.share = 1.0;
        q.inlet = inlet;
        mass += q.mass;
        brought += energy(&q);
        particles.push(q);
    }
    (mass, brought)
}

/// The force per unit of pressure that the points of a reservoir, at their places and of
/// their volumes, put on each node they reach.
fn pressed(grid: &Grid, ghosts: &[(Vec3, f64)]) -> Vec<(usize, Vec3)> {
    let inv = 4.0 / (grid.h * grid.h);
    let plane = grid.space != Space::Solid;
    let mut sums: std::collections::BTreeMap<usize, Vec3> = std::collections::BTreeMap::new();
    for &(x, volume) in ghosts {
        let (base, w, _) = grid.stencil(x);
        let offsets = grid.offsets(base);
        // The hoop stress of the ring of a body of revolution.
        let hoop = if grid.space == Space::Axisymmetric {
            volume / grid.radius(x[0])
        } else {
            0.0
        };
        for sk in 0..grid.reach_z() {
            for sj in 0..3 {
                for si in 0..3 {
                    let at = offsets[0][si] + offsets[1][sj] + offsets[2][sk];
                    let s = [si, sj, sk];
                    let node: Vec3 = std::array::from_fn(|a| {
                        grid.origin[a] + (base[a] + s[a] as isize) as f64 * grid.h
                    });
                    // Nodes beyond the axis of a body of revolution only mirror others.
                    if grid.space == Space::Axisymmetric && node[0] < -1e-9 * grid.h {
                        continue;
                    }
                    let weight = w[0][si] * w[1][sj] * w[2][sk];
                    let mut f = scale(sub(node, x), volume * inv * weight);
                    f[0] += hoop * weight;
                    if plane {
                        f[2] = 0.0;
                    }
                    let sum = sums.entry(at).or_insert([0.0; 3]);
                    *sum = [sum[0] + f[0], sum[1] + f[1], sum[2] + f[2]];
                }
            }
        }
    }
    sums.into_iter().collect()
}
