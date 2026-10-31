//! Internal forces and tangent stiffness of the solid elements for nonlinear analyses:
//! material nonlinearity on the small strain, and geometric nonlinearity in the total
//! Lagrangian formulation (Green-Lagrange strain, second Piola-Kirchhoff stress, material
//! plus initial-stress stiffness; Bathe, *Finite Element Procedures*, 6.3; Zienkiewicz &
//! Taylor vol. 2, chapter 10).
//!
//! All formulations go through one displacement-gradient operator `L` (9 × ndof, `H = L u`
//! with `H_kc = ∂u_k/∂X_c`): the standard elements take the shape function derivatives, the
//! uniform-strain element (C3D8R) their volume average plus the hourglass control on the
//! displacements, and the incompatible-mode element (C3D8I) folds the condensed mode
//! amplitudes `α = -X u` into `L` (`L_eff = L_nodes - L_modes X`), which reproduces the
//! condensed linear stiffness exactly and keeps the enhanced kinematics for nonlinear
//! materials and large displacements. No CalculiX code was consulted.

use crate::solid::{Formulation, SolidElement};

/// What a material law needs at an integration point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointKinematics {
    /// `F = I + ∂u/∂X` (identity in geometrically linear analyses).
    pub deformation_gradient: [[f64; 3]; 3],
    /// Engineering strain `[xx, yy, zz, xy, yz, zx]`: the small strain, or the Green-Lagrange
    /// strain under NLGEOM (`2 E_ij` for the shear components).
    pub strain: [f64; 6],
    /// Integration weight times Jacobian determinant (the volume the point stands for).
    pub weight: f64,
}

impl PointKinematics {
    /// `J = det F`.
    pub fn jacobian(&self) -> f64 {
        det3(&self.deformation_gradient)
    }

    /// The Cauchy stress `σ = J⁻¹ F S Fᵀ` of a second Piola-Kirchhoff stress `S`
    /// (`[xx, yy, zz, xy, yz, zx]`).
    pub fn cauchy(&self, s: &[f64; 6]) -> [f64; 6] {
        let f = &self.deformation_gradient;
        let sm = [[s[0], s[3], s[5]], [s[3], s[1], s[4]], [s[5], s[4], s[2]]];
        let j = det3(f);
        let mut fs = [[0.0; 3]; 3];
        for i in 0..3 {
            for k in 0..3 {
                fs[i][k] = (0..3).map(|m| f[i][m] * sm[m][k]).sum();
            }
        }
        let mut sigma = [[0.0; 3]; 3];
        for i in 0..3 {
            for k in 0..3 {
                sigma[i][k] = (0..3).map(|m| fs[i][m] * f[k][m]).sum::<f64>() / j;
            }
        }
        [
            sigma[0][0],
            sigma[1][1],
            sigma[2][2],
            0.5 * (sigma[0][1] + sigma[1][0]),
            0.5 * (sigma[1][2] + sigma[2][1]),
            0.5 * (sigma[2][0] + sigma[0][2]),
        ]
    }
}

fn det3(a: &[[f64; 3]; 3]) -> f64 {
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
}

/// The displacement-gradient operator at one point: `l[(k * 3 + c) * ndof + col]`.
struct Gradient {
    l: Vec<f64>,
    weight: f64,
}

impl SolidElement {
    /// The gradient operators of all integration points (one for the uniform-strain
    /// element) and, for C3D8R, the hourglass stiffness to add.
    fn gradients(
        &self,
        coords: &[[f64; 3]],
        d_elastic: &[[f64; 6]; 6],
    ) -> (Vec<Gradient>, Option<Vec<f64>>) {
        let ndof = self.dof_count();
        let nodal = |dndx: &[[f64; 3]], weight: f64| -> Gradient {
            let mut l = vec![0.0; 9 * ndof];
            for (a, g) in dndx.iter().enumerate() {
                for k in 0..3 {
                    for c in 0..3 {
                        l[(k * 3 + c) * ndof + 3 * a + k] = g[c];
                    }
                }
            }
            Gradient { l, weight }
        };
        match self.formulation {
            Formulation::Standard => (
                self.rule
                    .points
                    .iter()
                    .zip(&self.rule.weights)
                    .map(|(&p, &w)| {
                        let point = self.point(p, coords);
                        nodal(&point.dndx, point.det * w)
                    })
                    .collect(),
                None,
            ),
            Formulation::Reduced => {
                let (mean, volume) = self.mean_gradients(coords);
                let mut hourglass = vec![0.0; ndof * ndof];
                self.hourglass_stiffness(coords, d_elastic, &mean, volume, &mut hourglass);
                (vec![nodal(&mean, volume)], Some(hourglass))
            }
            Formulation::Incompatible => {
                let (x, inv0, det0) = self.incompatible_coupling(coords, d_elastic);
                let gradients = self
                    .rule
                    .points
                    .iter()
                    .zip(&self.rule.weights)
                    .map(|(&p, &w)| {
                        let point = self.point(p, coords);
                        let mut g = nodal(&point.dndx, point.det * w);
                        let modes = Self::mode_gradients(p, &inv0, det0, point.det);
                        // L_eff[(k,c)][j] = L[(k,c)][j] − Σ_m dM_m/dx_c X[3m+k][j].
                        for (m, dm) in modes.iter().enumerate() {
                            for k in 0..3 {
                                let row = &x[(3 * m + k) * ndof..(3 * m + k + 1) * ndof];
                                for (c, dmc) in dm.iter().enumerate() {
                                    let l = &mut g.l[(k * 3 + c) * ndof..(k * 3 + c + 1) * ndof];
                                    for (lj, xj) in l.iter_mut().zip(row) {
                                        *lj -= dmc * xj;
                                    }
                                }
                            }
                        }
                        g
                    })
                    .collect();
                (gradients, None)
            }
        }
    }

    /// Internal nodal forces (added to `fe`, 3 per node) and, when `ke` is given, the
    /// tangent stiffness (row-major, overwritten) for the nodal displacements `u`. For every
    /// integration point `material(point, kinematics)` returns the stress (second
    /// Piola-Kirchhoff under NLGEOM) and the tangent on engineering strains; the caller
    /// keeps whatever state it updates there. `d_elastic` serves the hourglass control and
    /// the incompatible modes.
    #[allow(clippy::too_many_arguments)]
    pub fn nonlinear<M>(
        &self,
        coords: &[[f64; 3]],
        u: &[f64],
        nlgeom: bool,
        d_elastic: &[[f64; 6]; 6],
        mut material: M,
        fe: &mut [f64],
        mut ke: Option<&mut Vec<f64>>,
    ) where
        M: FnMut(usize, &PointKinematics) -> ([f64; 6], [[f64; 6]; 6]),
    {
        let ndof = self.dof_count();
        if let Some(ke) = ke.as_deref_mut() {
            ke.clear();
            ke.resize(ndof * ndof, 0.0);
        }
        let (gradients, hourglass) = self.gradients(coords, d_elastic);
        let mut b = vec![0.0; 6 * ndof];
        let mut db = vec![0.0; 6 * ndof];
        for (ip, gradient) in gradients.iter().enumerate() {
            let l = &gradient.l;
            // H = L u, F = I + H.
            let mut h = [[0.0; 3]; 3];
            for k in 0..3 {
                for c in 0..3 {
                    let row = &l[(k * 3 + c) * ndof..(k * 3 + c + 1) * ndof];
                    h[k][c] = row.iter().zip(u).map(|(a, b)| a * b).sum();
                }
            }
            let mut f = [[0.0; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    f[i][j] = if i == j { 1.0 } else { 0.0 } + if nlgeom { h[i][j] } else { 0.0 };
                }
            }
            const IJ: [(usize, usize); 6] = [(0, 0), (1, 1), (2, 2), (0, 1), (1, 2), (2, 0)];
            let mut strain = [0.0; 6];
            for (r, &(i, j)) in IJ.iter().enumerate() {
                let mut e = h[i][j] + h[j][i];
                if nlgeom {
                    e += (0..3).map(|k| h[k][i] * h[k][j]).sum::<f64>();
                }
                strain[r] = if i == j { 0.5 * e } else { e };
            }
            // B_L = ∂E/∂u: normal rows Σ_k F_ki L_(k,i), shear rows Σ_k (F_ki L_(k,j) + F_kj L_(k,i)).
            for (r, &(i, j)) in IJ.iter().enumerate() {
                let out = &mut b[r * ndof..(r + 1) * ndof];
                out.iter_mut().for_each(|v| *v = 0.0);
                for k in 0..3 {
                    let li = &l[(k * 3 + i) * ndof..(k * 3 + i + 1) * ndof];
                    let lj = &l[(k * 3 + j) * ndof..(k * 3 + j + 1) * ndof];
                    if i == j {
                        for (o, a) in out.iter_mut().zip(li) {
                            *o += f[k][i] * a;
                        }
                    } else {
                        for ((o, a), c) in out.iter_mut().zip(lj).zip(li) {
                            *o += f[k][i] * a + f[k][j] * c;
                        }
                    }
                }
            }
            let kinematics = PointKinematics {
                deformation_gradient: f,
                strain,
                weight: gradient.weight,
            };
            let (s, d) = material(ip, &kinematics);
            let w = gradient.weight;
            for (c, o) in fe.iter_mut().enumerate() {
                *o += w * (0..6).map(|r| b[r * ndof + c] * s[r]).sum::<f64>();
            }
            let Some(ke) = ke.as_deref_mut() else {
                continue;
            };
            // Material stiffness Bᵀ D B.
            for r in 0..6 {
                for c in 0..ndof {
                    db[r * ndof + c] = (0..6).map(|q| d[r][q] * b[q * ndof + c]).sum();
                }
            }
            for a in 0..ndof {
                for c in a..ndof {
                    let v: f64 = (0..6).map(|r| b[r * ndof + a] * db[r * ndof + c]).sum();
                    ke[a * ndof + c] += w * v;
                }
            }
            if nlgeom {
                // Initial-stress stiffness Lᵀ (S ⊗ I) L: for every direction k,
                // Σ_{c,c'} L_(k,c)ᵀ S_cc' L_(k,c').
                let sm = [[s[0], s[3], s[5]], [s[3], s[1], s[4]], [s[5], s[4], s[2]]];
                let mut g = vec![0.0; 3 * ndof];
                for k in 0..3 {
                    for c in 0..3 {
                        let row = &mut g[c * ndof..(c + 1) * ndof];
                        row.iter_mut().for_each(|v| *v = 0.0);
                        for cp in 0..3 {
                            let lk = &l[(k * 3 + cp) * ndof..(k * 3 + cp + 1) * ndof];
                            for (o, a) in row.iter_mut().zip(lk) {
                                *o += sm[c][cp] * a;
                            }
                        }
                    }
                    for c in 0..3 {
                        let lk = &l[(k * 3 + c) * ndof..(k * 3 + c + 1) * ndof];
                        let gc = &g[c * ndof..(c + 1) * ndof];
                        for a in 0..ndof {
                            if lk[a] == 0.0 {
                                continue;
                            }
                            for cc in a..ndof {
                                ke[a * ndof + cc] += w * lk[a] * gc[cc];
                            }
                        }
                    }
                }
            }
        }
        if let Some(ke) = ke.as_deref_mut() {
            for a in 0..ndof {
                for c in 0..a {
                    ke[a * ndof + c] = ke[c * ndof + a];
                }
            }
        }
        if let Some(hg) = hourglass {
            for a in 0..ndof {
                let row = &hg[a * ndof..(a + 1) * ndof];
                fe[a] += row.iter().zip(u).map(|(k, u)| k * u).sum::<f64>();
            }
            if let Some(ke) = ke {
                for (k, h) in ke.iter_mut().zip(&hg) {
                    *k += h;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid::solid;
    use plxs_core::ElementType;

    fn isotropic(young: f64, poisson: f64) -> [[f64; 6]; 6] {
        let lambda = young * poisson / ((1.0 + poisson) * (1.0 - 2.0 * poisson));
        let mu = young / (2.0 * (1.0 + poisson));
        let mut d = [[0.0; 6]; 6];
        for i in 0..3 {
            d[i][..3].fill(lambda);
            d[i][i] = lambda + 2.0 * mu;
            d[i + 3][i + 3] = mu;
        }
        d
    }

    fn linear(
        d: &[[f64; 6]; 6],
    ) -> impl FnMut(usize, &PointKinematics) -> ([f64; 6], [[f64; 6]; 6]) + '_ {
        move |_, k| {
            let mut s = [0.0; 6];
            for (r, sr) in s.iter_mut().enumerate() {
                *sr = (0..6).map(|c| d[r][c] * k.strain[c]).sum();
            }
            (s, *d)
        }
    }

    /// A distorted element of each type: the reference nodes, perturbed deterministically.
    fn distorted(kind: ElementType) -> (SolidElement, Vec<[f64; 3]>) {
        let e = solid(kind).unwrap();
        let mut coords: Vec<[f64; 3]> = e.family.node_coordinates().to_vec();
        for (i, c) in coords.iter_mut().enumerate() {
            let s = (i as f64 * 0.731).sin();
            c[0] += 0.08 * s;
            c[1] -= 0.05 * (i as f64 * 1.37).cos();
            c[2] += 0.06 * s * s;
            for v in c.iter_mut() {
                *v *= 2.5;
            }
        }
        (e, coords)
    }

    fn displacements(ndof: usize, scale: f64) -> Vec<f64> {
        (0..ndof)
            .map(|i| scale * ((i as f64 * 0.37).sin() + 0.5 * (i as f64 * 1.11).cos()))
            .collect()
    }

    #[test]
    fn geometrically_linear_path_reproduces_the_linear_stiffness() {
        let d = isotropic(210_000.0, 0.3);
        for kind in [
            ElementType::C3D8,
            ElementType::C3D8R,
            ElementType::C3D8I,
            ElementType::C3D20,
            ElementType::C3D20R,
            ElementType::C3D4,
            ElementType::C3D10,
            ElementType::C3D6,
            ElementType::C3D15,
        ] {
            let (e, coords) = distorted(kind);
            let ndof = e.dof_count();
            let mut k_lin = Vec::new();
            e.stiffness(&coords, &d, &mut k_lin);
            let u = displacements(ndof, 1e-3);
            let mut fe = vec![0.0; ndof];
            let mut ke = Vec::new();
            e.nonlinear(&coords, &u, false, &d, linear(&d), &mut fe, Some(&mut ke));
            let scale = k_lin.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            for (a, b) in k_lin.iter().zip(&ke) {
                assert!(
                    (a - b).abs() < 1e-10 * scale,
                    "{kind}: K differs: {a} vs {b}"
                );
            }
            let fscale = fe.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            for a in 0..ndof {
                let ku: f64 = (0..ndof).map(|c| k_lin[a * ndof + c] * u[c]).sum();
                assert!((ku - fe[a]).abs() < 1e-10 * fscale, "{kind}: f_int differs");
            }
        }
    }

    #[test]
    fn the_tangent_is_the_derivative_of_the_internal_force_under_nlgeom() {
        let d = isotropic(1000.0, 0.3);
        for kind in [
            ElementType::C3D8,
            ElementType::C3D8R,
            ElementType::C3D8I,
            ElementType::C3D10,
        ] {
            let (e, coords) = distorted(kind);
            let ndof = e.dof_count();
            let u = displacements(ndof, 0.2);
            let mut fe = vec![0.0; ndof];
            let mut ke = Vec::new();
            e.nonlinear(&coords, &u, true, &d, linear(&d), &mut fe, Some(&mut ke));
            let h = 1e-6;
            let scale = ke.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            for j in 0..ndof {
                let mut up = u.clone();
                up[j] += h;
                let mut um = u.clone();
                um[j] -= h;
                let mut fp = vec![0.0; ndof];
                let mut fm = vec![0.0; ndof];
                e.nonlinear(&coords, &up, true, &d, linear(&d), &mut fp, None);
                e.nonlinear(&coords, &um, true, &d, linear(&d), &mut fm, None);
                for i in 0..ndof {
                    let fd = (fp[i] - fm[i]) / (2.0 * h);
                    assert!(
                        (fd - ke[i * ndof + j]).abs() < 1e-6 * scale,
                        "{kind}: K[{i}][{j}] = {} but finite difference {fd}",
                        ke[i * ndof + j]
                    );
                }
            }
        }
    }

    #[test]
    fn a_rigid_rotation_is_stress_free_under_nlgeom() {
        let d = isotropic(1000.0, 0.3);
        let (e, coords) = distorted(ElementType::C3D8);
        let angle = 0.7_f64;
        let (s, c) = angle.sin_cos();
        let u: Vec<f64> = coords
            .iter()
            .flat_map(|x| {
                let rx = c * x[0] - s * x[1];
                let ry = s * x[0] + c * x[1];
                [rx - x[0], ry - x[1], 0.0]
            })
            .collect();
        let mut fe = vec![0.0; 24];
        let mut max_strain = 0.0_f64;
        e.nonlinear(
            &coords,
            &u,
            true,
            &d,
            |_, k| {
                max_strain = max_strain.max(k.strain.iter().fold(0.0_f64, |m, v| m.max(v.abs())));
                ([0.0; 6], d)
            },
            &mut fe,
            None,
        );
        assert!(max_strain < 1e-12, "{max_strain}");
    }

    #[test]
    fn cauchy_stress_of_a_uniaxial_stretch() {
        let k = PointKinematics {
            deformation_gradient: [[1.2, 0.0, 0.0], [0.0, 0.9, 0.0], [0.0, 0.0, 0.9]],
            strain: [0.0; 6],
            weight: 1.0,
        };
        let sigma = k.cauchy(&[100.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        // σ = J⁻¹ F S Fᵀ: σ_xx = 1.2² · 100 / (1.2 · 0.81).
        assert!((sigma[0] - 1.44 * 100.0 / 0.972).abs() < 1e-9, "{sigma:?}");
        assert!(sigma[1].abs() < 1e-12 && sigma[3].abs() < 1e-12);
    }
}
