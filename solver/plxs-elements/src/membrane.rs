//! The three-node membrane `M3D3`, computed natively as a constant-strain plane stress
//! triangle in its own plane: CalculiX 2.21 gives exactly these displacements, nodal
//! stresses (element values averaged at the nodes) and strains (with the thickness strain
//! `ε33 = -ν / (1 - ν) (ε11 + ε22)`), as found with ccx as a black box. Out of its plane
//! the element has no stiffness.

use crate::expand::{cross, dot, normalize, sub};

/// Local frame of a triangle: rows e1 (first edge), e2, normal; `None` when degenerate.
fn frame(x: &[[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let a = sub(x[1], x[0]);
    let b = sub(x[2], x[0]);
    let n = cross(a, b);
    if dot(n, n) <= 1e-24 * dot(a, a).max(1e-300) * dot(b, b).max(1e-300) {
        return None;
    }
    let e1 = normalize(a);
    let n = normalize(n);
    Some([e1, cross(n, e1), n])
}

/// Strain-displacement matrix, area and frame of a triangle.
type Local = ([[f64; 6]; 3], f64, [[f64; 3]; 3]);

/// Strain-displacement matrix (3 × 6 on the local in-plane DOFs, engineering shear),
/// the area and the frame.
fn local(x: &[[f64; 3]; 3]) -> Option<Local> {
    let r = frame(x)?;
    let p: Vec<[f64; 2]> = x
        .iter()
        .map(|&xi| {
            let d = sub(xi, x[0]);
            [dot(d, r[0]), dot(d, r[1])]
        })
        .collect();
    let area = 0.5
        * ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[2][0] - p[0][0]) * (p[1][1] - p[0][1]));
    if area <= 0.0 {
        return None;
    }
    let b = [p[1][1] - p[2][1], p[2][1] - p[0][1], p[0][1] - p[1][1]];
    let c = [p[2][0] - p[1][0], p[0][0] - p[2][0], p[1][0] - p[0][0]];
    let mut bm = [[0.0; 6]; 3];
    for i in 0..3 {
        bm[0][2 * i] = b[i] / (2.0 * area);
        bm[1][2 * i + 1] = c[i] / (2.0 * area);
        bm[2][2 * i] = c[i] / (2.0 * area);
        bm[2][2 * i + 1] = b[i] / (2.0 * area);
    }
    Some((bm, area, r))
}

fn plane_stress(young: f64, poisson: f64) -> [[f64; 3]; 3] {
    let f = young / (1.0 - poisson * poisson);
    [
        [f, f * poisson, 0.0],
        [f * poisson, f, 0.0],
        [0.0, 0.0, f * (1.0 - poisson) / 2.0],
    ]
}

/// Local in-plane DOF k (node k / 2, direction k % 2) of global DOF j (node j / 3, xyz).
fn transform(r: &[[f64; 3]; 3]) -> [[f64; 9]; 6] {
    let mut t = [[0.0; 9]; 6];
    for node in 0..3 {
        for d in 0..2 {
            for k in 0..3 {
                t[2 * node + d][3 * node + k] = r[d][k];
            }
        }
    }
    t
}

/// The stiffness (row-major 9 × 9 on the nodes' x, y, z), or `None` for a degenerate
/// triangle.
pub fn stiffness(x: &[[f64; 3]; 3], young: f64, poisson: f64, thickness: f64) -> Option<Vec<f64>> {
    let (b, area, r) = local(x)?;
    let d = plane_stress(young, poisson);
    let t = transform(&r);
    // Local 6 × 6: Bᵀ D B A t.
    let mut kl = [[0.0; 6]; 6];
    for i in 0..6 {
        for j in 0..6 {
            let mut v = 0.0;
            for a in 0..3 {
                for c in 0..3 {
                    v += b[a][i] * d[a][c] * b[c][j];
                }
            }
            kl[i][j] = v * area * thickness;
        }
    }
    let mut k = vec![0.0; 81];
    for i in 0..9 {
        for j in 0..9 {
            let mut v = 0.0;
            for a in 0..6 {
                if t[a][i] == 0.0 {
                    continue;
                }
                for c in 0..6 {
                    v += t[a][i] * kl[a][c] * t[c][j];
                }
            }
            k[i * 9 + j] = v;
        }
    }
    Some(k)
}

/// Strain and stress tensors in global axes (`[xx, yy, zz, xy, yz, zx]`, tensor shear) of
/// the displacements `u` (9: the nodes' x, y, z).
pub fn strain_stress(
    x: &[[f64; 3]; 3],
    u: &[f64],
    young: f64,
    poisson: f64,
) -> Option<([f64; 6], [f64; 6])> {
    let (b, _, r) = local(x)?;
    let t = transform(&r);
    let ul: Vec<f64> = (0..6)
        .map(|a| (0..9).map(|j| t[a][j] * u[j]).sum())
        .collect();
    let eps: [f64; 3] = std::array::from_fn(|a| (0..6).map(|j| b[a][j] * ul[j]).sum());
    let d = plane_stress(young, poisson);
    let sig: [f64; 3] = std::array::from_fn(|a| (0..3).map(|c| d[a][c] * eps[c]).sum());
    let e33 = -poisson / (1.0 - poisson) * (eps[0] + eps[1]);
    let strain = [
        [eps[0], eps[2] / 2.0, 0.0],
        [eps[2] / 2.0, eps[1], 0.0],
        [0.0, 0.0, e33],
    ];
    let stress = [
        [sig[0], sig[2], 0.0],
        [sig[2], sig[1], 0.0],
        [0.0, 0.0, 0.0],
    ];
    let global = |m: [[f64; 3]; 3]| -> [f64; 6] {
        let g = |i: usize, j: usize| -> f64 {
            let mut v = 0.0;
            for a in 0..3 {
                for c in 0..3 {
                    v += m[a][c] * r[a][i] * r[c][j];
                }
            }
            v
        };
        [g(0, 0), g(1, 1), g(2, 2), g(0, 1), g(1, 2), g(2, 0)]
    };
    Some((global(strain), global(stress)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rigid_motions_are_free_and_uniform_strain_is_exact() {
        let x = [[0.0, 0.0, 0.0], [2.0, 0.5, 1.0], [0.3, 1.5, -0.4]];
        let k = stiffness(&x, 210000.0, 0.3, 2.0).unwrap();
        let w = [0.1, -0.2, 0.3];
        let mut u = [0.0; 9];
        for (n, xn) in x.iter().enumerate() {
            let r = cross(w, *xn);
            for d in 0..3 {
                u[3 * n + d] = r[d] + [1.0, 2.0, 3.0][d];
            }
        }
        let largest = k.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        for i in 0..9 {
            let f: f64 = (0..9).map(|j| k[i * 9 + j] * u[j]).sum();
            assert!(f.abs() < 1e-10 * largest, "{i}: {f}");
        }
        // Uniform in-plane stretch in the element plane: uniaxial stress along e1.
        let r = frame(&x).unwrap();
        let mut u = [0.0; 9];
        for (n, xn) in x.iter().enumerate() {
            let s = 1e-3 * dot(sub(*xn, x[0]), r[0]);
            for d in 0..3 {
                u[3 * n + d] = s * r[0][d];
            }
        }
        let (strain, stress) = strain_stress(&x, &u, 210000.0, 0.3).unwrap();
        let trace = strain[0] + strain[1] + strain[2];
        // ε11 = 1e-3, ε22 = 0, ε33 = -0.3/0.7e-3.
        assert!((trace - (1e-3 - 0.3 / 0.7 * 1e-3)).abs() < 1e-12);
        let s_trace = stress[0] + stress[1] + stress[2];
        assert!((s_trace - 210000.0 / 0.91 * 1e-3 * 1.3).abs() < 1e-6);
    }
}
