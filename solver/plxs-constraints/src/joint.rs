//! Joints (`*JOINT`): kinematic couplings between two points, each the reference node
//! (translations) and rotation node (rotation vector) of a rigid body or deformable coupling,
//! or ground. A joint holds chosen components of the relative motion of the second end
//! against the first in the joint frame:
//!
//! - translation `i`: `e_i · (x_B(p) − x_A(p)) = 0`, with `x_E(p) = X_E + u_E + R(θ_E)(p − X_E)`
//!   the place of the material point at the joint origin `p` on end `E`, and `e_i = R(θ_A) e⁰_i`
//!   the frame axis turned with the first end;
//! - rotation `i`: `e⁰_i · log(R(θ_A)ᵀ R(θ_B)) = 0`, the component of the relative rotation
//!   vector in the frame of the first end.
//!
//! In small rotations the rows are linear (`R(θ) r ≈ r + θ × r`, `log ≈ θ_B − θ_A`) and
//! eliminated like any other constraint. Under NLGEOM they are linearized at the current
//! rotations in every Newton iteration and the dependent DOFs are put back on `c = 0`
//! afterwards (see [`ModelConstraints::at_rotations`](crate::ModelConstraints)), like the
//! members of rigid bodies. With one free rotation (hinge, cylinder, planar joint) the rows
//! are exact; with two or three free rotations the rotation rows hold components of the
//! relative rotation vector.
//!
//! Sources: Shabana, "Dynamics of Multibody Systems", chapter 3 (joint constraints); Géradin
//! and Cardona, "Flexible Multibody Dynamics", chapter 4 (rotation vector and its logarithm).

use crate::set::{norm, rotate};

/// Degrees of freedom per node of the primary numbering.
const PER_NODE: u32 = 3;

/// A joint prepared on dense node indices.
#[derive(Clone, Debug, PartialEq)]
pub struct JointData {
    /// Reference and rotation node (dense) of each end with the reference node's
    /// undeformed position; `None` for ground.
    pub ends: [Option<End>; 2],
    pub origin: [f64; 3],
    pub axes: [[f64; 3]; 3],
    pub fixed: [bool; 6],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct End {
    pub reference: u32,
    pub rotation: u32,
    pub position: [f64; 3],
}

/// One held degree of freedom of a joint, a row of the model's constraints.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointRow {
    /// Index into [`crate::ModelConstraints::constraints`].
    pub constraint: usize,
    /// Index into [`crate::ModelConstraints::joints`].
    pub joint: usize,
    /// 0..3 translations, 3..6 rotations, in the joint frame.
    pub dof: usize,
}

impl JointData {
    /// The global DOFs the joint acts on: translations of the reference nodes and the
    /// rotation vectors of the rotation nodes, first end first.
    pub fn dofs(&self) -> Vec<u32> {
        let mut dofs = Vec::with_capacity(12);
        for end in self.ends.iter().flatten() {
            for node in [end.reference, end.rotation] {
                dofs.extend((0..PER_NODE).map(|d| node * PER_NODE + d));
            }
        }
        dofs
    }

    /// The six residuals at the displacements `u` (global directions, three per node).
    pub fn residual(&self, u: &[f64]) -> [f64; 6] {
        let place = |end: &Option<End>| -> ([f64; 3], [f64; 3]) {
            match end {
                None => (self.origin, [0.0; 3]),
                Some(e) => {
                    let theta = node_vector(u, e.rotation);
                    let shift = node_vector(u, e.reference);
                    let r = sub(self.origin, e.position);
                    let turned = rotate(theta, r);
                    let x = [
                        e.position[0] + shift[0] + turned[0],
                        e.position[1] + shift[1] + turned[1],
                        e.position[2] + shift[2] + turned[2],
                    ];
                    (x, theta)
                }
            }
        };
        let (xa, ta) = place(&self.ends[0]);
        let (xb, tb) = place(&self.ends[1]);
        let gap = sub(xb, xa);
        let relative = relative_rotation(ta, tb);
        std::array::from_fn(|i| {
            let axis = self.axes[i % 3];
            if i < 3 {
                dot(rotate(ta, axis), gap)
            } else {
                dot(axis, relative)
            }
        })
    }

    /// The rows of the joint in small rotations, on global DOFs: `(dof, coefficient)` for
    /// each joint DOF.
    pub fn linear_rows(&self) -> [Vec<(u32, f64)>; 6] {
        let mut rows: [Vec<(u32, f64)>; 6] = Default::default();
        for (side, end) in self.ends.iter().enumerate() {
            let Some(end) = end else {
                continue;
            };
            let sign = if side == 0 { -1.0 } else { 1.0 };
            let r = sub(self.origin, end.position);
            for i in 0..3 {
                let e = self.axes[i];
                // e · (u + θ × r) = e · u + θ · (r × e)
                let lever = cross(r, e);
                for d in 0..3 {
                    push(
                        &mut rows[i],
                        end.reference * PER_NODE + d as u32,
                        sign * e[d],
                    );
                    push(
                        &mut rows[i],
                        end.rotation * PER_NODE + d as u32,
                        sign * lever[d],
                    );
                    push(
                        &mut rows[3 + i],
                        end.rotation * PER_NODE + d as u32,
                        sign * e[d],
                    );
                }
            }
        }
        rows
    }

    /// The rows linearized at `u`: the derivatives of [`JointData::residual`] by central
    /// differences.
    pub fn rows_at(&self, u: &[f64]) -> [Vec<(u32, f64)>; 6] {
        let dofs = self.dofs();
        let mut rows: [Vec<(u32, f64)>; 6] = Default::default();
        let mut work = u.to_vec();
        let needed = dofs.iter().map(|&d| d as usize + 1).max().unwrap_or(0);
        if work.len() < needed {
            work.resize(needed, 0.0);
        }
        let scale = 1.0 + self.size();
        for &dof in &dofs {
            let at = dof as usize;
            let rotation = self.is_rotation(dof);
            let h = if rotation {
                1e-6 * (1.0 + norm(node_vector(&work, dof / PER_NODE)))
            } else {
                1e-6 * scale
            };
            let keep = work[at];
            work[at] = keep + h;
            let plus = self.residual(&work);
            work[at] = keep - h;
            let minus = self.residual(&work);
            work[at] = keep;
            for i in 0..6 {
                let a = (plus[i] - minus[i]) / (2.0 * h);
                if a.abs() > 1e-10 {
                    rows[i].push((dof, a));
                }
            }
        }
        rows
    }

    /// The DOF a row is solved for: the dominant term on the second end, or on the first
    /// against ground.
    pub fn preferred(&self, dof: usize) -> Option<u32> {
        let end = self.ends[1].or(self.ends[0])?;
        let e = self.axes[dof % 3];
        let node = if dof < 3 { end.reference } else { end.rotation };
        let d = (0..3).max_by(|&a, &b| e[a].abs().total_cmp(&e[b].abs()))?;
        Some(node * PER_NODE + d as u32)
    }

    /// Puts the dependent DOFs `dependent` (global, one per held row in `rows`) back on to
    /// the joint's constraints by a few Newton steps, the other DOFs fixed.
    pub fn project(&self, u: &mut [f64], rows: &[usize], dependent: &[u32]) {
        let k = rows.len();
        if k == 0 || dependent.len() != k {
            return;
        }
        let tolerance = 1e-12 * (1.0 + self.size());
        for _ in 0..10 {
            let c = self.residual(u);
            let r: Vec<f64> = rows.iter().map(|&i| c[i]).collect();
            if r.iter().all(|v| v.abs() <= tolerance) {
                return;
            }
            let jacobian = self.rows_at(u);
            let mut a = vec![vec![0.0; k]; k];
            for (row, &i) in rows.iter().enumerate() {
                for &(dof, value) in &jacobian[i] {
                    if let Some(col) = dependent.iter().position(|&d| d == dof) {
                        a[row][col] = value;
                    }
                }
            }
            let Some(step) = solve(a, r) else {
                return;
            };
            for (col, &dof) in dependent.iter().enumerate() {
                if let Some(v) = u.get_mut(dof as usize) {
                    *v -= step[col];
                }
            }
        }
    }

    fn is_rotation(&self, dof: u32) -> bool {
        let node = dof / PER_NODE;
        self.ends
            .iter()
            .flatten()
            .any(|e| e.rotation == node && e.reference != node)
    }

    /// A length of the joint for tolerances: the distance of its ends from the origin.
    fn size(&self) -> f64 {
        (self.ends.iter().flatten())
            .map(|e| norm(sub(self.origin, e.position)))
            .fold(0.0, f64::max)
    }
}

fn push(row: &mut Vec<(u32, f64)>, dof: u32, a: f64) {
    if a == 0.0 {
        return;
    }
    match row.iter_mut().find(|(d, _)| *d == dof) {
        Some((_, v)) => *v += a,
        None => row.push((dof, a)),
    }
}

/// The rotation vector of `R(a)ᵀ R(b)`.
pub fn relative_rotation(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    let ra = matrix(a);
    let rb = matrix(b);
    let mut q = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            q[i][j] = (0..3).map(|k| ra[k][i] * rb[k][j]).sum();
        }
    }
    log(q)
}

/// The rotation matrix of a rotation vector (columns are the turned unit vectors).
fn matrix(theta: [f64; 3]) -> [[f64; 3]; 3] {
    let columns = [
        rotate(theta, [1.0, 0.0, 0.0]),
        rotate(theta, [0.0, 1.0, 0.0]),
        rotate(theta, [0.0, 0.0, 1.0]),
    ];
    let mut m = [[0.0; 3]; 3];
    for (j, c) in columns.iter().enumerate() {
        for i in 0..3 {
            m[i][j] = c[i];
        }
    }
    m
}

/// The rotation vector of a rotation matrix, for angles below π.
fn log(q: [[f64; 3]; 3]) -> [f64; 3] {
    let skew = [q[2][1] - q[1][2], q[0][2] - q[2][0], q[1][0] - q[0][1]];
    let cos = ((q[0][0] + q[1][1] + q[2][2] - 1.0) / 2.0).clamp(-1.0, 1.0);
    let sin = 0.5 * norm(skew);
    let angle = sin.atan2(cos);
    // angle / (2 sin(angle)), by its series near zero.
    let factor = if angle < 1e-4 {
        0.5 + angle * angle / 12.0
    } else {
        angle / (2.0 * angle.sin())
    };
    [skew[0] * factor, skew[1] * factor, skew[2] * factor]
}

/// Solves a small dense system by Gaussian elimination with partial pivoting.
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        let pivot = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() < 1e-12 {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        let (above, below) = a.split_at_mut(col + 1);
        let pivot_row = &above[col];
        for (offset, row) in below.iter_mut().enumerate() {
            let f = row[col] / pivot_row[col];
            for (value, &p) in row.iter_mut().zip(pivot_row).skip(col) {
                *value -= f * p;
            }
            b[col + 1 + offset] -= f * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let s: f64 = (row + 1..n).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - s) / a[row][row];
    }
    Some(x)
}

fn node_vector(u: &[f64], node: u32) -> [f64; 3] {
    let at = node as usize * PER_NODE as usize;
    match u.get(at..at + 3) {
        Some(v) => [v[0], v[1], v[2]],
        None => [0.0; 3],
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
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

#[cfg(test)]
mod tests {
    use super::*;

    fn hinge() -> JointData {
        JointData {
            ends: [
                Some(End {
                    reference: 0,
                    rotation: 1,
                    position: [0.0, 0.0, 0.0],
                }),
                Some(End {
                    reference: 2,
                    rotation: 3,
                    position: [2.0, 0.0, 0.0],
                }),
            ],
            origin: [1.0, 0.0, 0.0],
            axes: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            fixed: [true, true, true, true, true, false],
        }
    }

    #[test]
    fn the_rows_at_rest_are_the_linear_rows() {
        let joint = hinge();
        let linear = joint.linear_rows();
        let at = joint.rows_at(&[0.0; 12]);
        for i in 0..6 {
            for &(dof, a) in &linear[i] {
                let b = at[i].iter().find(|(d, _)| *d == dof).map_or(0.0, |t| t.1);
                assert!((a - b).abs() < 1e-8, "row {i} dof {dof}: {a} vs {b}");
            }
            assert_eq!(linear[i].len(), at[i].len(), "row {i}");
        }
    }

    #[test]
    fn a_hinge_turned_about_its_axis_stays_closed() {
        let joint = hinge();
        // Both bodies turned by 120° about z through the joint origin.
        let angle = 2.0 * std::f64::consts::PI / 3.0;
        let theta = [0.0, 0.0, angle];
        let mut u = vec![0.0; 12];
        let a_ref = sub(rotate(theta, [-1.0, 0.0, 0.0]), [-1.0, 0.0, 0.0]);
        let b_ref = sub(rotate(theta, [1.0, 0.0, 0.0]), [1.0, 0.0, 0.0]);
        u[0..3].copy_from_slice(&a_ref);
        u[3..6].copy_from_slice(&theta);
        u[6..9].copy_from_slice(&b_ref);
        // The second body turns further about the (turned) hinge axis.
        u[9..12].copy_from_slice(&[0.0, 0.0, angle + 0.3]);
        let b_turned = sub(
            rotate([0.0, 0.0, angle + 0.3], [1.0, 0.0, 0.0]),
            [1.0, 0.0, 0.0],
        );
        u[6..9].copy_from_slice(&b_turned);
        let c = joint.residual(&u);
        for (i, v) in c.iter().enumerate().take(5) {
            assert!(v.abs() < 1e-12, "dof {i}: {v}");
        }
        assert!((c[5] - 0.3).abs() < 1e-12);
    }

    #[test]
    fn projection_closes_an_opened_joint() {
        let mut joint = hinge();
        joint.axes = [[0.0, 0.6, 0.8], [1.0, 0.0, 0.0], [0.0, 0.8, -0.6]];
        let mut u = vec![0.0; 12];
        u[3..6].copy_from_slice(&[0.4, -0.2, 0.7]);
        u[0..3].copy_from_slice(&[0.1, 0.05, -0.2]);
        let rows: Vec<usize> = (0..6).filter(|&i| joint.fixed[i]).collect();
        let dependent: Vec<u32> = rows
            .iter()
            .map(|&i| joint.preferred(i).unwrap())
            .collect::<Vec<_>>();
        assert_eq!(dependent.len(), 5);
        joint.project(&mut u, &rows, &dependent);
        let c = joint.residual(&u);
        for &i in &rows {
            assert!(c[i].abs() < 1e-10, "dof {i}: {}", c[i]);
        }
    }

    #[test]
    fn relative_rotation_of_equal_rotations_is_zero() {
        let t = [0.3, -1.2, 0.8];
        let r = relative_rotation(t, t);
        assert!(norm(r) < 1e-12);
        let r = relative_rotation([0.0; 3], t);
        assert!(norm(sub(r, t)) < 1e-12);
    }
}
