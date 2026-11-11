//! Follower loads (`*CLOAD, FOLLOWER=`): concentrated loads whose direction turns with the
//! structure under large deformations. A load turns with one rotation `R`: the rigid
//! rotation that fits the motion of a group of nodes best, or the rotation vector held in
//! the translations of a rotation node (rigid bodies, beam and shell knots).
//!
//! The load vector of [`crate::load_vector`] holds the loads in their reference
//! directions; a procedure that follows the deformation adds `(R − I) F` with
//! [`Turning::add`] at the displacements it iterates on. The load stiffness (the derivative
//! of `R` by the displacements) is left out of the tangent, as for the follower pressures
//! of membranes.
//!
//! The best-fit rotation is Kabsch's: with the centred reference positions `X_i` and the
//! centred current ones `x_i`, `H = Σ x_i X_iᵀ = R C` for a rigid motion (`C = Σ X_i X_iᵀ`),
//! and `R = Σ u_k v_kᵀ` from the singular vectors of `H` (eigenvectors `v_k` of `HᵀH`,
//! `u_k = H v_k / σ_k`), the third pair completed by cross products so that `R` stays a
//! rotation. Nodes in a plane determine the rotation fully; nodes on a line do not fix the
//! turn about the line, they take the smallest rotation that turns the line.
//!
//! No CalculiX code was consulted.

use plxs_core::mesh::NodeRef;
use plxs_core::step::Follower;
use plxs_core::{Model, Step};
use plxs_materials::finite::eigen_symmetric;

use crate::DofMap;
use crate::dofs::amplitude_factor;

type Mat3 = [[f64; 3]; 3];

/// The follower loads of a step, grouped by what they turn with.
#[derive(Clone, Debug, Default)]
pub struct FollowerLoads {
    groups: Vec<Group>,
}

#[derive(Clone, Debug)]
struct Group {
    follower: Follower,
    source: Source,
    items: Vec<Item>,
}

#[derive(Clone, Debug)]
enum Source {
    /// The DOFs of the translations of the nodes and their positions relative to their
    /// centre; `line` is the direction of nodes that lie on a line.
    Nodes {
        dofs: Vec<[Option<u32>; 3]>,
        reference: Vec<[f64; 3]>,
        line: Option<[f64; 3]>,
    },
    /// The DOFs holding the rotation vector.
    Rotation([Option<u32>; 3]),
    /// Nothing to turn with (a single node of a solid): the load keeps its direction.
    Fixed,
}

/// One component of a load at a node: the DOFs of its block (translations or rotations),
/// which of them it acts on in the reference, its value and amplitude.
#[derive(Clone, Debug)]
struct Item {
    dofs: [Option<u32>; 3],
    column: usize,
    value: f64,
    amplitude: Option<String>,
}

impl FollowerLoads {
    /// The follower loads of `step` with the DOFs of `dofs`. Loads without
    /// [`plxs_core::step::ConcentratedLoad::follower`] are left out.
    pub fn new(model: &Model, dofs: &DofMap, step: &Step) -> Self {
        let mut groups: Vec<Group> = Vec::new();
        for load in &step.loads.concentrated {
            let Some(follower) = &load.follower else {
                continue;
            };
            if !(1..=6).contains(&load.dof) {
                continue;
            }
            let group = match groups.iter().position(|g| g.follower == *follower) {
                Some(g) => g,
                None => {
                    groups.push(Group {
                        follower: follower.clone(),
                        source: source(model, dofs, follower),
                        items: Vec::new(),
                    });
                    groups.len() - 1
                }
            };
            let block = if load.dof <= 3 { 0 } else { 3 };
            let column = (load.dof as usize - 1) % 3;
            for node in nodes_of(model, &load.region) {
                let Some(index) = model.nodes.index(node) else {
                    continue;
                };
                let item_dofs = std::array::from_fn(|k| dofs.dof(index, block + k));
                if item_dofs[column].is_none() {
                    continue;
                }
                groups[group].items.push(Item {
                    dofs: item_dofs,
                    column,
                    value: load.value,
                    amplitude: load.amplitude.clone(),
                });
            }
        }
        groups.retain(|g| !g.items.is_empty() && !matches!(g.source, Source::Fixed));
        Self { groups }
    }

    pub fn is_empty(&self) -> bool {
        self.groups.is_empty()
    }

    /// The DOFs whose displacements the rotations depend on, ascending.
    pub fn dofs(&self) -> Vec<u32> {
        let mut out: Vec<u32> = (self.groups.iter())
            .flat_map(|g| match &g.source {
                Source::Nodes { dofs, .. } => dofs.iter().flatten().flatten().copied().collect(),
                Source::Rotation(d) => d.iter().flatten().copied().collect::<Vec<u32>>(),
                Source::Fixed => Vec::new(),
            })
            .collect();
        out.sort_unstable();
        out.dedup();
        out
    }

    /// The loads at a time: their values times their amplitudes.
    pub fn at(
        &self,
        model: &Model,
        step: &Step,
        time: f64,
        total: f64,
        period: f64,
    ) -> Turning<'_> {
        let factors = (self.groups.iter())
            .map(|g| {
                (g.items.iter())
                    .map(|item| {
                        let amplitude = item.amplitude.as_deref();
                        item.value * amplitude_factor(model, step, amplitude, time, total, period)
                    })
                    .collect()
            })
            .collect();
        Turning {
            loads: self,
            values: factors,
        }
    }
}

/// The follower loads at a time ([`FollowerLoads::at`]).
#[derive(Clone, Debug)]
pub struct Turning<'a> {
    loads: &'a FollowerLoads,
    /// The value of every item of every group.
    values: Vec<Vec<f64>>,
}

impl Turning<'_> {
    pub fn is_empty(&self) -> bool {
        self.loads.is_empty()
    }

    /// Adds `sign (R − I) F` to `f`: how far the loads turned from their reference
    /// directions at the displacements `u` (by DOF).
    pub fn add(&self, u: impl Fn(u32) -> f64, sign: f64, f: &mut [f64]) {
        for (group, values) in self.loads.groups.iter().zip(&self.values) {
            let r = rotation(&group.source, &u);
            for (item, &value) in group.items.iter().zip(values) {
                if value == 0.0 {
                    continue;
                }
                for (k, dof) in item.dofs.iter().enumerate() {
                    let Some(dof) = dof else {
                        continue;
                    };
                    let delta = if k == item.column { 1.0 } else { 0.0 };
                    f[*dof as usize] += sign * value * (r[k][item.column] - delta);
                }
            }
        }
    }

    /// [`Self::add`] with the displacements of all DOFs.
    pub fn add_at(&self, u: &[f64], sign: f64, f: &mut [f64]) {
        self.add(|d| u.get(d as usize).copied().unwrap_or(0.0), sign, f);
    }
}

fn nodes_of(model: &Model, region: &NodeRef) -> Vec<u32> {
    model.nodes_of(region)
}

fn source(model: &Model, dofs: &DofMap, follower: &Follower) -> Source {
    match follower {
        Follower::Rotation(node) => match model.nodes.index(*node) {
            Some(index) => Source::Rotation(std::array::from_fn(|k| dofs.dof(index, k))),
            None => Source::Fixed,
        },
        Follower::Nodes(region) => {
            let indices: Vec<u32> = (nodes_of(model, region).into_iter())
                .filter_map(|n| model.nodes.index(n))
                .collect();
            // A single node turns with its own rotations if it has them.
            if let [index] = indices[..] {
                let rotations: [Option<u32>; 3] = std::array::from_fn(|k| dofs.dof(index, 3 + k));
                return if rotations.iter().all(Option::is_some) {
                    Source::Rotation(rotations)
                } else {
                    Source::Fixed
                };
            }
            if indices.len() < 2 {
                return Source::Fixed;
            }
            let positions: Vec<[f64; 3]> = (indices.iter())
                .map(|&i| model.nodes.coordinates[i as usize])
                .collect();
            let centre = mean(&positions);
            let reference: Vec<[f64; 3]> = positions.iter().map(|p| sub(*p, centre)).collect();
            let mut c = [[0.0; 3]; 3];
            for x in &reference {
                add_outer(&mut c, *x, *x);
            }
            let (values, vectors) = eigen_symmetric(&c);
            let order = descending(values);
            if values[order[0]] <= 0.0 {
                return Source::Fixed;
            }
            let line =
                (values[order[1]] <= 1e-10 * values[order[0]]).then(|| column(&vectors, order[0]));
            Source::Nodes {
                dofs: (indices.iter())
                    .map(|&i| std::array::from_fn(|k| dofs.dof(i, k)))
                    .collect(),
                reference,
                line,
            }
        }
    }
}

/// The rotation of a source at the displacements `u`.
fn rotation(source: &Source, u: &impl Fn(u32) -> f64) -> Mat3 {
    let value = |d: &Option<u32>| d.map_or(0.0, u);
    match source {
        Source::Fixed => IDENTITY,
        Source::Rotation(dofs) => rotation_matrix(dofs.each_ref().map(value)),
        Source::Nodes {
            dofs,
            reference,
            line,
        } => {
            let current: Vec<[f64; 3]> = (reference.iter().zip(dofs))
                .map(|(x, d)| add(*x, d.each_ref().map(value)))
                .collect();
            let centre = mean(&current);
            let mut h = [[0.0; 3]; 3];
            for (x, r) in current.iter().zip(reference) {
                add_outer(&mut h, sub(*x, centre), *r);
            }
            match line {
                Some(direction) => {
                    let Some(turned) = normalize(apply(&h, *direction)) else {
                        return IDENTITY;
                    };
                    smallest_rotation(*direction, turned)
                }
                None => kabsch(&h),
            }
        }
    }
}

/// The rotation of the polar decomposition of `h`, for nodes not on a line.
fn kabsch(h: &Mat3) -> Mat3 {
    let mut hth = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            hth[i][j] = (0..3).map(|k| h[k][i] * h[k][j]).sum();
        }
    }
    let (values, vectors) = eigen_symmetric(&hth);
    let order = descending(values);
    let v1 = column(&vectors, order[0]);
    let v2 = column(&vectors, order[1]);
    let Some(u1) = normalize(apply(h, v1)) else {
        return IDENTITY;
    };
    let h2 = apply(h, v2);
    let Some(u2) = normalize(sub(h2, scale(u1, dot(u1, h2)))) else {
        return IDENTITY;
    };
    let (u3, v3) = (cross(u1, u2), cross(v1, v2));
    let mut r = [[0.0; 3]; 3];
    for (u, v) in [(u1, v1), (u2, v2), (u3, v3)] {
        add_outer(&mut r, u, v);
    }
    r
}

/// The rotation by the smallest angle that turns the unit vector `a` into the unit vector
/// `b`.
fn smallest_rotation(a: [f64; 3], b: [f64; 3]) -> Mat3 {
    let axis = cross(a, b);
    let s = norm(axis);
    let c = dot(a, b);
    if s < 1e-14 {
        if c > 0.0 {
            return IDENTITY;
        }
        // Half a turn about any axis normal to `a`.
        let other = if a[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let n = normalize(cross(a, other)).unwrap_or([0.0, 0.0, 1.0]);
        return rotation_matrix(scale(n, std::f64::consts::PI));
    }
    rotation_matrix(scale(axis, s.atan2(c) / s))
}

/// The matrix of the rotation vector `theta` (Rodrigues' formula).
pub fn rotation_matrix(theta: [f64; 3]) -> Mat3 {
    let angle = norm(theta);
    let (a, b) = if angle < 1e-4 {
        let a2 = angle * angle;
        (1.0 - a2 / 6.0, 0.5 - a2 / 24.0)
    } else {
        (angle.sin() / angle, (1.0 - angle.cos()) / (angle * angle))
    };
    let [x, y, z] = theta;
    let k = [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]];
    let mut r = IDENTITY;
    for i in 0..3 {
        for j in 0..3 {
            let kk: f64 = (0..3).map(|m| k[i][m] * k[m][j]).sum();
            r[i][j] += a * k[i][j] + b * kk;
        }
    }
    r
}

const IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn descending(values: [f64; 3]) -> [usize; 3] {
    let mut order = [0, 1, 2];
    order.sort_by(|&a, &b| values[b].total_cmp(&values[a]));
    order
}

fn column(m: &Mat3, k: usize) -> [f64; 3] {
    [m[0][k], m[1][k], m[2][k]]
}

fn apply(m: &Mat3, v: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| dot(m[i], v))
}

fn add_outer(m: &mut Mat3, a: [f64; 3], b: [f64; 3]) {
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] += a[i] * b[j];
        }
    }
}

fn mean(points: &[[f64; 3]]) -> [f64; 3] {
    let sum = points.iter().fold([0.0; 3], |s, p| add(s, *p));
    scale(sum, 1.0 / points.len().max(1) as f64)
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] + b[i])
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    a.map(|v| v * s)
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn normalize(a: [f64; 3]) -> Option<[f64; 3]> {
    let n = norm(a);
    (n > 1e-300).then(|| scale(a, 1.0 / n))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: &Mat3, b: &Mat3) -> bool {
        (0..3).all(|i| (0..3).all(|j| (a[i][j] - b[i][j]).abs() < 1e-9))
    }

    fn turned(points: &[[f64; 3]], r: &Mat3, shift: [f64; 3]) -> Vec<[f64; 3]> {
        points.iter().map(|p| add(apply(r, *p), shift)).collect()
    }

    fn fit(points: &[[f64; 3]], moved: &[[f64; 3]]) -> Mat3 {
        let centre = mean(points);
        let reference: Vec<[f64; 3]> = points.iter().map(|p| sub(*p, centre)).collect();
        let mut c = [[0.0; 3]; 3];
        for x in &reference {
            add_outer(&mut c, *x, *x);
        }
        let (values, vectors) = eigen_symmetric(&c);
        let order = descending(values);
        let line =
            (values[order[1]] <= 1e-10 * values[order[0]]).then(|| column(&vectors, order[0]));
        let u: Vec<f64> = (points.iter().zip(moved))
            .flat_map(|(p, m)| sub(*m, *p))
            .collect();
        let source = Source::Nodes {
            dofs: (0..points.len() as u32)
                .map(|i| std::array::from_fn(|k| Some(3 * i + k as u32)))
                .collect(),
            reference,
            line,
        };
        rotation(&source, &|d| u[d as usize])
    }

    #[test]
    fn a_rigid_motion_of_a_solid_or_a_plane_is_found_exactly() {
        let r = rotation_matrix([0.3, -1.1, 2.0]);
        let cube: Vec<[f64; 3]> = (0..8)
            .map(|k| {
                [
                    (k & 1) as f64,
                    ((k >> 1) & 1) as f64 * 2.0,
                    ((k >> 2) & 1) as f64,
                ]
            })
            .collect();
        assert!(close(&fit(&cube, &turned(&cube, &r, [5.0, -2.0, 1.0])), &r));
        let face = &cube[..4];
        assert!(close(&fit(face, &turned(face, &r, [0.0, 3.0, 0.0])), &r));
        // A plane turned half around keeps a proper rotation.
        let half = rotation_matrix([std::f64::consts::PI, 0.0, 0.0]);
        assert!(close(&fit(face, &turned(face, &half, [0.0; 3])), &half));
    }

    #[test]
    fn nodes_on_a_line_take_the_smallest_rotation_of_the_line() {
        let line: Vec<[f64; 3]> = (0..4).map(|k| [k as f64, 0.0, 0.0]).collect();
        // Turned about z: the smallest rotation is that rotation.
        let r = rotation_matrix([0.0, 0.0, 0.7]);
        assert!(close(&fit(&line, &turned(&line, &r, [1.0, 1.0, 0.0])), &r));
        // A twist about the line itself does not show.
        let twist = rotation_matrix([0.9, 0.0, 0.0]);
        assert!(close(
            &fit(&line, &turned(&line, &twist, [0.0; 3])),
            &IDENTITY
        ));
    }

    #[test]
    fn rodrigues_turns_x_into_y_about_z() {
        let r = rotation_matrix([0.0, 0.0, std::f64::consts::FRAC_PI_2]);
        let x = apply(&r, [1.0, 0.0, 0.0]);
        assert!((x[0]).abs() < 1e-12 && (x[1] - 1.0).abs() < 1e-12);
    }
}
