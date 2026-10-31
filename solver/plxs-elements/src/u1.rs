//! The general beam `U1` (`*USER ELEMENT, TYPE=U1` with `*BEAM SECTION, SECTION=GENERAL`):
//! a straight two-node beam with six DOFs per node, computed natively rather than expanded.
//!
//! The stiffness follows from the flexibility of the element clamped at its first node,
//! taken from CalculiX 2.21 as a black box (cantilevers under end forces and moments, shear
//! factor and element length varied; no CalculiX code was consulted):
//!
//! * axial `L / (E A)`, torsion `L / (G (I11 + I22))`;
//! * bending with the deflection along the local 1-direction (stiffness `E I11`) and along
//!   the 2-direction (`E I22`): end moment `L / (E I)`, coupling `L² / (2 E I)`, and under
//!   an end force `L³ / (3 E I) + s (φ - 3) / (1 + φ)` with `s = L / (k G A)` and
//!   `φ = 12 E I / (k G A L²)`. This is not the Timoshenko value `L³ / (3 E I) + s`; it is
//!   what ccx computes (to six digits for k from 0.003 to 100).
//!
//! The full 12 × 12 matrix is `Qᵀ K_q Q`, `Q` mapping the nodal DOFs to the deformation of
//! the second node relative to the rigid motion of the first; rigid-body motions are free of
//! forces by construction.

use crate::expand::{cross, dot, normalize, scale, sub};

/// Section and material values of a U1 element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct U1Section {
    pub area: f64,
    /// Second moment for bending with the deflection along the 1-direction.
    pub i11: f64,
    /// Second moment for bending with the deflection along the 2-direction.
    pub i22: f64,
    /// Shear factor `k` (shear stiffness `k G A`).
    pub shear_factor: f64,
    pub young: f64,
    pub poisson: f64,
}

/// The local axes: tangent, 1-direction (the section vector made orthogonal) and
/// 2-direction `t × e1`.
pub fn axes(xa: [f64; 3], xb: [f64; 3], n1: [f64; 3]) -> Option<([f64; 3], [f64; 3], [f64; 3])> {
    let t = normalize(sub(xb, xa));
    let e1 = sub(n1, scale(t, dot(n1, t)));
    if dot(e1, e1) < 1e-20 * dot(n1, n1).max(1e-300) {
        return None;
    }
    let e1 = normalize(e1);
    Some((t, e1, cross(t, e1)))
}

/// The stiffness matrix (row-major 12 × 12) on `[u_a, θ_a, u_b, θ_b]`, or `None` when the
/// section vector is parallel to the axis or the element has no length.
pub fn stiffness(xa: [f64; 3], xb: [f64; 3], n1: [f64; 3], s: &U1Section) -> Option<Vec<f64>> {
    let d = sub(xb, xa);
    let length = dot(d, d).sqrt();
    if length <= 0.0 {
        return None;
    }
    let (t, e1, e2) = axes(xa, xb, n1)?;
    let g = s.young / (2.0 * (1.0 + s.poisson));
    let kga = s.shear_factor * g * s.area;
    let bending = |ei: f64| -> [[f64; 2]; 2] {
        let phi = 12.0 * ei / (kga * length * length);
        let shear = length / kga * (phi - 3.0) / (1.0 + phi);
        let f = [
            [
                length.powi(3) / (3.0 * ei) + shear,
                length * length / (2.0 * ei),
            ],
            [length * length / (2.0 * ei), length / ei],
        ];
        let det = f[0][0] * f[1][1] - f[0][1] * f[1][0];
        [
            [f[1][1] / det, -f[0][1] / det],
            [-f[1][0] / det, f[0][0] / det],
        ]
    };
    let k1 = bending(s.young * s.i11);
    let k2 = bending(s.young * s.i22);
    // Relative deformation q (6) = Q (6 × 12) · D: d = u_b - u_a + (L t) × θ_a, r = θ_b - θ_a;
    // q = [d·t, r·t, d·e1, r·e2, d·e2, -r·e1] (each deflection with the rotation whose
    // positive sense makes a positive slope).
    let lt = scale(t, length);
    let row = |dir: [f64; 3], rot: bool, sign: f64| -> [f64; 12] {
        let mut q = [0.0; 12];
        for i in 0..3 {
            if rot {
                q[3 + i] = -sign * dir[i];
                q[9 + i] = sign * dir[i];
            } else {
                q[i] = -sign * dir[i];
                q[6 + i] = sign * dir[i];
            }
        }
        if !rot {
            // dir · (L t × θ_a) = θ_a · (dir × L t)
            let c = cross(dir, lt);
            for i in 0..3 {
                q[3 + i] += sign * c[i];
            }
        }
        q
    };
    let q = [
        row(t, false, 1.0),
        row(t, true, 1.0),
        row(e1, false, 1.0),
        row(e2, true, 1.0),
        row(e2, false, 1.0),
        row(e1, true, -1.0),
    ];
    let mut kq = [[0.0; 6]; 6];
    kq[0][0] = s.young * s.area / length;
    kq[1][1] = g * (s.i11 + s.i22) / length;
    for a in 0..2 {
        for b in 0..2 {
            kq[2 + a][2 + b] = k1[a][b];
            kq[4 + a][4 + b] = k2[a][b];
        }
    }
    let mut k = vec![0.0; 144];
    for i in 0..12 {
        for j in 0..12 {
            let mut v = 0.0;
            for a in 0..6 {
                if q[a][i] == 0.0 {
                    continue;
                }
                for b in 0..6 {
                    v += q[a][i] * kq[a][b] * q[b][j];
                }
            }
            k[i * 12 + j] = v;
        }
    }
    Some(k)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn section() -> U1Section {
        U1Section {
            area: 952.0,
            i11: 1.25469333e5,
            i22: 1.55435733e6,
            shear_factor: 3.69747899e-1,
            young: 210000.0,
            poisson: 0.3,
        }
    }

    /// Clamps node a and solves for an end load at node b.
    fn tip(load: [f64; 6]) -> [f64; 6] {
        let k = stiffness([0.0; 3], [100.0, 0.0, 0.0], [0.0, 0.0, 1.0], &section()).unwrap();
        let mut m = [[0.0; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                m[i][j] = k[(6 + i) * 12 + 6 + j];
            }
        }
        // Gauss elimination.
        let mut b = load;
        for c in 0..6 {
            let p = (c..6)
                .max_by(|&x, &y| m[x][c].abs().total_cmp(&m[y][c].abs()))
                .unwrap();
            m.swap(c, p);
            b.swap(c, p);
            for r in 0..6 {
                if r != c {
                    let f = m[r][c] / m[c][c];
                    let pivot = m[c];
                    for (v, p) in m[r].iter_mut().zip(pivot) {
                        *v -= f * p;
                    }
                    b[r] -= f * b[c];
                }
            }
        }
        std::array::from_fn(|i| b[i] / m[i][i])
    }

    #[test]
    fn rigid_motions_are_free() {
        let k = stiffness(
            [1.0, 2.0, 3.0],
            [4.0, 6.0, 3.5],
            [0.0, 0.0, 1.0],
            &section(),
        )
        .unwrap();
        // Rotation about z through the origin: u = θ × x.
        let (xa, xb) = ([1.0, 2.0, 3.0], [4.0, 6.0, 3.5]);
        let w = [0.1, -0.2, 0.3];
        let mut u = [0.0; 12];
        let ua = cross(w, xa);
        let ub = cross(w, xb);
        u[..3].copy_from_slice(&ua);
        u[3..6].copy_from_slice(&w);
        u[6..9].copy_from_slice(&ub);
        u[9..].copy_from_slice(&w);
        let largest = k.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        for i in 0..12 {
            let f: f64 = (0..12).map(|j| k[i * 12 + j] * u[j]).sum();
            assert!(f.abs() < 1e-12 * largest, "{i}: {f}");
        }
    }

    #[test]
    fn cantilever_matches_the_ccx_flexibility() {
        let s = section();
        let ei = s.young * s.i11;
        let u = tip([0.0, 0.0, 100.0, 0.0, 0.0, 0.0]);
        // Rotation about -y (the 2-direction for e1 = z) is the slope.
        assert!((u[4] + 100.0 * 1e4 / (2.0 * ei)).abs() < 1e-12);
        let g = s.young / 2.6;
        let kga = s.shear_factor * g * s.area;
        let phi = 12.0 * ei / (kga * 1e4);
        let w = 100.0 * (1e6 / (3.0 * ei) + 100.0 / kga * (phi - 3.0) / (1.0 + phi));
        assert!((u[2] - w).abs() < 1e-12 * w.abs().max(1.0));
        let axial = tip([1000.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        assert!((axial[0] - 1000.0 * 100.0 / (s.young * s.area)).abs() < 1e-15);
    }
}
