//! Anisotropic linear elasticity (`*ELASTIC, TYPE=ENGINEERING CONSTANTS | ORTHO | ANISO`):
//! the 6 × 6 matrix in the material axes, turned into the global axes of an orientation.
//!
//! Sources: Jones, *Mechanics of Composite Materials* (1999), chapter 2 (the compliance of
//! an orthotropic material from its engineering constants, the restrictions on them, the
//! transformation of stiffness under a rotation of axes); Hill (1952) for the Voigt
//! average of the moduli. The CalculiX manual gives the order of the constants on the data
//! lines. No CalculiX code was consulted.
//!
//! Conventions as in the crate: engineering strains `[11, 22, 33, 12, 23, 31]`.

// Index loops read like the formulas of the matrices.
#![allow(clippy::needless_range_loop)]

use plxs_core::material::{AnisotropicElastic, Material};
use plxs_core::{Error, Result};

use crate::{LinearElastic, MaterialInput, MaterialLaw, MaterialOutput};

/// Index of a symmetric tensor component `(i, j)` in the Voigt order of the crate.
const fn voigt(i: usize, j: usize) -> usize {
    match (i, j) {
        (0, 0) => 0,
        (1, 1) => 1,
        (2, 2) => 2,
        (0, 1) | (1, 0) => 3,
        (1, 2) | (2, 1) => 4,
        _ => 5,
    }
}

/// The stiffness of the constants in the material axes. Fails when the material is not
/// stable (the matrix is not positive definite), which also refuses impossible Poisson's
/// ratios of engineering constants.
pub fn material_matrix(name: &str, elastic: &AnisotropicElastic) -> Result<[[f64; 6]; 6]> {
    let bad = |what: String| Error::input("*ELASTIC", format!("material {name}: {what}"));
    let d = match elastic {
        AnisotropicElastic::EngineeringConstants(c) => {
            let [e1, e2, e3, nu12, nu13, nu23, g12, g13, g23] = *c;
            if [e1, e2, e3, g12, g13, g23]
                .iter()
                .any(|&v| v <= 0.0 || !v.is_finite())
            {
                return Err(bad(
                    "the Young's and shear moduli of the engineering constants must be \
                     positive"
                        .into(),
                ));
            }
            let mut s = [[0.0; 6]; 6];
            s[0][0] = 1.0 / e1;
            s[1][1] = 1.0 / e2;
            s[2][2] = 1.0 / e3;
            s[0][1] = -nu12 / e1;
            s[1][0] = s[0][1];
            s[0][2] = -nu13 / e1;
            s[2][0] = s[0][2];
            s[1][2] = -nu23 / e2;
            s[2][1] = s[1][2];
            s[3][3] = 1.0 / g12;
            s[4][4] = 1.0 / g23;
            s[5][5] = 1.0 / g13;
            invert(&s).ok_or_else(|| {
                bad("the Poisson's ratios make the material unstable (compliance singular)".into())
            })?
        }
        AnisotropicElastic::Orthotropic(c) => {
            let [
                d1111,
                d1122,
                d2222,
                d1133,
                d2233,
                d3333,
                d1212,
                d1313,
                d2323,
            ] = *c;
            let mut d = [[0.0; 6]; 6];
            d[0][0] = d1111;
            d[0][1] = d1122;
            d[1][1] = d2222;
            d[0][2] = d1133;
            d[1][2] = d2233;
            d[2][2] = d3333;
            d[3][3] = d1212;
            d[5][5] = d1313;
            d[4][4] = d2323;
            symmetrize_upper(&mut d);
            d
        }
        AnisotropicElastic::Anisotropic(c) => {
            // The data lines run over the upper triangle of the tensor pairs 11, 22, 33,
            // 12, 13, 23 column by column.
            const PAIRS: [(usize, usize); 6] = [(0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2)];
            let mut d = [[0.0; 6]; 6];
            let mut k = 0;
            for col in 0..6 {
                for row in 0..=col {
                    let (a, b) = PAIRS[row];
                    let (p, q) = PAIRS[col];
                    let (r, c2) = (voigt(a, b), voigt(p, q));
                    d[r][c2] = c[k];
                    d[c2][r] = c[k];
                    k += 1;
                }
            }
            d
        }
    };
    if !positive_definite(&d) {
        return Err(bad(
            "the elastic constants are not positive definite: the material would give \
             energy under some strain"
                .into(),
        ));
    }
    Ok(d)
}

fn symmetrize_upper(d: &mut [[f64; 6]; 6]) {
    for i in 0..6 {
        for j in 0..i {
            d[i][j] = d[j][i];
        }
    }
}

/// The stiffness `d` of material axes `axes` (rows: the unit material axes in global
/// coordinates) in the global axes: `T^T d T` with `T` turning global engineering strains
/// into material ones.
pub fn rotate(d: &[[f64; 6]; 6], axes: &[[f64; 3]; 3]) -> [[f64; 6]; 6] {
    // Column c of T: the material strain of the global unit engineering strain c.
    let mut t = [[0.0; 6]; 6];
    for c in 0..6 {
        let mut eps = [[0.0; 3]; 3];
        match c {
            0..=2 => eps[c][c] = 1.0,
            _ => {
                let (i, j) = match c {
                    3 => (0, 1),
                    4 => (1, 2),
                    _ => (2, 0),
                };
                eps[i][j] = 0.5;
                eps[j][i] = 0.5;
            }
        }
        // ε' = R ε Rᵀ with R's rows the material axes.
        let mut local = [[0.0; 3]; 3];
        for a in 0..3 {
            for b in 0..3 {
                let mut sum = 0.0;
                for i in 0..3 {
                    for j in 0..3 {
                        sum += axes[a][i] * eps[i][j] * axes[b][j];
                    }
                }
                local[a][b] = sum;
            }
        }
        t[0][c] = local[0][0];
        t[1][c] = local[1][1];
        t[2][c] = local[2][2];
        t[3][c] = 2.0 * local[0][1];
        t[4][c] = 2.0 * local[1][2];
        t[5][c] = 2.0 * local[2][0];
    }
    let mut out = [[0.0; 6]; 6];
    for i in 0..6 {
        for j in 0..6 {
            let mut sum = 0.0;
            for k in 0..6 {
                for l in 0..6 {
                    sum += t[k][i] * d[k][l] * t[l][j];
                }
            }
            out[i][j] = sum;
        }
    }
    out
}

/// The isotropic constants closest to an anisotropic stiffness by the Voigt average of its
/// bulk and shear moduli: what contact penalties and other estimates that want one modulus
/// take.
pub fn isotropic_equivalent(d: &[[f64; 6]; 6]) -> LinearElastic {
    let diagonal = d[0][0] + d[1][1] + d[2][2];
    let off = d[0][1] + d[1][2] + d[0][2];
    let shear = d[3][3] + d[4][4] + d[5][5];
    let bulk = (diagonal + 2.0 * off) / 9.0;
    let g = (diagonal - off + 3.0 * shear) / 15.0;
    let young = 9.0 * bulk * g / (3.0 * bulk + g);
    let poisson = (3.0 * bulk - 2.0 * g) / (2.0 * (3.0 * bulk + g));
    LinearElastic { young, poisson }
}

/// The inverse of a symmetric positive definite 6 × 6 matrix by Gauss-Jordan elimination,
/// `None` when it is singular.
pub fn invert(m: &[[f64; 6]; 6]) -> Option<[[f64; 6]; 6]> {
    let mut a = *m;
    let mut inv = [[0.0; 6]; 6];
    for (i, row) in inv.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    let scale = m.iter().flatten().fold(0.0_f64, |s, v| s.max(v.abs()));
    for col in 0..6 {
        let pivot = (col..6).max_by(|&x, &y| a[x][col].abs().total_cmp(&a[y][col].abs()))?;
        if a[pivot][col].abs() <= 1e-14 * scale || !a[pivot][col].is_finite() {
            return None;
        }
        a.swap(col, pivot);
        inv.swap(col, pivot);
        let p = a[col][col];
        for j in 0..6 {
            a[col][j] /= p;
            inv[col][j] /= p;
        }
        for r in 0..6 {
            if r != col {
                let f = a[r][col];
                if f != 0.0 {
                    for j in 0..6 {
                        a[r][j] -= f * a[col][j];
                        inv[r][j] -= f * inv[col][j];
                    }
                }
            }
        }
    }
    Some(inv)
}

/// Whether a symmetric matrix is positive definite (Cholesky).
pub fn positive_definite(d: &[[f64; 6]; 6]) -> bool {
    let mut l = [[0.0; 6]; 6];
    for i in 0..6 {
        for j in 0..=i {
            let mut sum = d[i][j];
            for k in 0..j {
                sum -= l[i][k] * l[j][k];
            }
            if i == j {
                if sum <= 0.0 || !sum.is_finite() {
                    return false;
                }
                l[i][i] = sum.sqrt();
            } else {
                l[i][j] = sum / l[j][j];
            }
        }
    }
    true
}

/// Linear elasticity with a general stiffness in the global axes (anisotropic materials,
/// turned by their orientation).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AnisotropicLinear {
    pub d: [[f64; 6]; 6],
}

impl AnisotropicLinear {
    /// The law of an anisotropic material in the axes `axes` (rows, global coordinates);
    /// `None` axes are the global ones. `Ok(None)` when the material is not anisotropic.
    pub fn of(material: &Material, axes: Option<&[[f64; 3]; 3]>) -> Result<Option<Self>> {
        let Some(elastic) = &material.anisotropic else {
            return Ok(None);
        };
        let d = material_matrix(&material.name, elastic)?;
        Ok(Some(Self {
            d: match axes {
                Some(axes) => rotate(&d, axes),
                None => d,
            },
        }))
    }
}

impl MaterialLaw for AnisotropicLinear {
    fn state_size(&self) -> usize {
        0
    }

    fn is_nonlinear(&self) -> bool {
        false
    }

    fn stress(&self, input: &MaterialInput, _state: &mut [f64]) -> MaterialOutput {
        let mut stress = [0.0; 6];
        for (r, s) in stress.iter_mut().enumerate() {
            *s = (0..6).map(|c| self.d[r][c] * input.strain[c]).sum();
        }
        MaterialOutput {
            stress,
            tangent: self.d,
        }
    }

    fn elastic_matrix(&self) -> [[f64; 6]; 6] {
        self.d
    }

    fn kernel_law(&self) -> Option<crate::KernelLaw> {
        Some(crate::KernelLaw::Elastic)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::isotropic_matrix;

    fn close(a: &[[f64; 6]; 6], b: &[[f64; 6]; 6], tol: f64) -> bool {
        a.iter()
            .flatten()
            .zip(b.iter().flatten())
            .all(|(x, y)| (x - y).abs() <= tol)
    }

    #[test]
    fn isotropic_engineering_constants_give_hooke() {
        let (e, nu) = (210_000.0, 0.3);
        let g = e / (2.0 * (1.0 + nu));
        let d = material_matrix(
            "M",
            &AnisotropicElastic::EngineeringConstants([e, e, e, nu, nu, nu, g, g, g]),
        )
        .unwrap();
        assert!(close(&d, &isotropic_matrix(e, nu), 1e-6));
    }

    #[test]
    fn engineering_constants_give_their_compliance() {
        // E1 = 3000, E2 = 2000, E3 = 1000, ν12 = 0.3, ν13 = 0.2, ν23 = 0.25.
        let c = [3000.0, 2000.0, 1000.0, 0.3, 0.2, 0.25, 800.0, 600.0, 400.0];
        let d = material_matrix("M", &AnisotropicElastic::EngineeringConstants(c)).unwrap();
        let s = invert(&d).unwrap();
        assert!((1.0 / s[0][0] - 3000.0).abs() < 1e-6);
        assert!((1.0 / s[2][2] - 1000.0).abs() < 1e-6);
        assert!((-s[0][1] / s[0][0] - 0.3).abs() < 1e-9);
        assert!((1.0 / s[4][4] - 400.0).abs() < 1e-6, "G23");
        assert!((1.0 / s[5][5] - 600.0).abs() < 1e-6, "G13");
    }

    #[test]
    fn unstable_poisson_ratios_are_refused() {
        let c = [1000.0, 1000.0, 1000.0, 0.9, 0.9, 0.9, 300.0, 300.0, 300.0];
        assert!(material_matrix("M", &AnisotropicElastic::EngineeringConstants(c)).is_err());
    }

    #[test]
    fn aniso_reads_the_upper_triangle_in_calculix_order() {
        let mut c = [0.0; 21];
        // D1111, D1122, D2222, D1133, D2233, D3333 and the shear diagonal.
        c[0] = 10.0;
        c[1] = 2.0;
        c[2] = 11.0;
        c[3] = 3.0;
        c[4] = 4.0;
        c[5] = 12.0;
        c[9] = 5.0; // D1212
        c[14] = 6.0; // D1313
        c[20] = 7.0; // D2323
        c[6] = 0.5; // D1112
        c[15] = 0.25; // D1123
        let d = material_matrix("M", &AnisotropicElastic::Anisotropic(c)).unwrap();
        assert_eq!(d[0][2], 3.0);
        assert_eq!(d[1][2], 4.0);
        assert_eq!(d[3][3], 5.0);
        assert_eq!(d[5][5], 6.0);
        assert_eq!(d[4][4], 7.0);
        assert_eq!(d[0][3], 0.5);
        assert_eq!(d[0][4], 0.25);
        assert_eq!(d[4][0], 0.25);
    }

    #[test]
    fn rotating_about_z_by_90_degrees_swaps_the_in_plane_axes() {
        let c = [3000.0, 2000.0, 1000.0, 0.3, 0.2, 0.25, 800.0, 600.0, 400.0];
        let d = material_matrix("M", &AnisotropicElastic::EngineeringConstants(c)).unwrap();
        // Material axis 1 along global y, 2 along -x.
        let axes = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let g = rotate(&d, &axes);
        assert!((g[1][1] - d[0][0]).abs() < 1e-9);
        assert!((g[0][0] - d[1][1]).abs() < 1e-9);
        assert!(
            (g[4][4] - d[5][5]).abs() < 1e-9,
            "yz shear of the global is 13"
        );
        assert!(g[0][3].abs() < 1e-9);
        // An isotropic matrix is unchanged by any rotation.
        let iso = isotropic_matrix(1000.0, 0.3);
        let s = 0.5_f64.sqrt();
        let tilted = [[s, s, 0.0], [-0.5, 0.5, s], [0.5, -0.5, s]];
        assert!(close(&rotate(&iso, &tilted), &iso, 1e-9));
    }

    #[test]
    fn the_voigt_average_of_hooke_is_hooke() {
        let e = isotropic_equivalent(&isotropic_matrix(1000.0, 0.25));
        assert!((e.young - 1000.0).abs() < 1e-9);
        assert!((e.poisson - 0.25).abs() < 1e-12);
    }
}
