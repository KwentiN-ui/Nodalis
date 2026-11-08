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

use crate::plane::Expansion;
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

#[inline(always)]
fn dot6(a: &[f64; 6], b: &[f64; 6]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// The cofactor matrix `det(m) m⁻ᵀ`.
fn cofactors(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            let (i1, i2) = ((i + 1) % 3, (i + 2) % 3);
            let (j1, j2) = ((j + 1) % 3, (j + 2) % 3);
            m[i1][j1] * m[i2][j2] - m[i1][j2] * m[i2][j1]
        })
    })
}

fn det3(a: &[[f64; 3]; 3]) -> f64 {
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
}

/// The integration points of an element as [`SolidElement::nodal_gradients`] gives them.
#[derive(Clone, Debug, PartialEq)]
pub struct NodalGradients {
    /// Per point: the weight times the Jacobian determinant (the volume the point stands
    /// for), and the Cartesian derivatives of the shape functions per node.
    pub points: Vec<(f64, Vec<[f64; 3]>)>,
    /// For C3D8R: `ε` of its hourglass stiffness `ε Σ_α γ_α γ_αᵀ ⊗ I`, whose vectors
    /// `γ_α = h_α − (h_α · z) ∂N/∂z` take the base vectors `h_α` of
    /// `SolidElement::HOURGLASS_BASE`, the nodal `z` coordinates and the (only) point's
    /// derivatives.
    pub hourglass: Option<f64>,
}

/// The integration points of any solid element as general displacement-gradient
/// operators ([`SolidElement::dense_operators`]), for the graphics card.
#[derive(Clone, Debug, PartialEq)]
pub struct DenseOperators {
    /// Per point: the volume it stands for (with the expansion's scale) and the operator
    /// `L` (`H_kc = Σ_j L[(3k + c) · ndof + j] u_j`).
    pub points: Vec<(f64, Vec<f64>)>,
    /// A constant stiffness to add (row-major `ndof × ndof`): the condensed hourglass
    /// control of an expanded reduced-integration element.
    pub matrix: Option<Vec<f64>>,
}

/// The displacement-gradient operator at one point: `l[(k * 3 + c) * ndof + col]`.
struct Gradient {
    l: Vec<f64>,
    weight: f64,
}

impl SolidElement {
    /// The gradient operators of all integration points (one for the uniform-strain
    /// element) and, for C3D8R, the hourglass stiffness to add.
    ///
    /// With `condense` (the expansion of a 2D element, `coords` those of its 3D element)
    /// the operators act on the 2D DOFs (`L T`); the hourglass stiffness stays on the 3D
    /// DOFs.
    fn gradients(
        &self,
        coords: &[[f64; 3]],
        d_elastic: &[[f64; 6]; 6],
        condense: Option<&Expansion>,
    ) -> (Vec<Gradient>, Option<Vec<f64>>) {
        let ndof = condense.map_or(self.dof_count_3d(), |x| 3 * x.nodes);
        let nodal = |dndx: &[[f64; 3]], weight: f64| -> Gradient {
            let mut l = vec![0.0; 9 * ndof];
            match condense {
                None => {
                    for (a, g) in dndx.iter().enumerate() {
                        for k in 0..3 {
                            for c in 0..3 {
                                l[(k * 3 + c) * ndof + 3 * a + k] = g[c];
                            }
                        }
                    }
                }
                Some(x) => {
                    let map = x.dof_map();
                    for (a, g) in dndx.iter().enumerate() {
                        for k in 0..3 {
                            let Some((d, factor)) = map[3 * a + k] else {
                                continue;
                            };
                            for c in 0..3 {
                                l[(k * 3 + c) * ndof + d] += factor * g[c];
                            }
                        }
                    }
                }
            }
            Gradient { l, weight }
        };
        match self.formulation {
            Formulation::Standard => (
                self.rule
                    .weights
                    .iter()
                    .enumerate()
                    .map(|(ip, &w)| {
                        let (det, dndx) = self.rule_point(ip, coords);
                        nodal(&dndx, det * w)
                    })
                    .collect(),
                None,
            ),
            Formulation::Reduced => {
                let (volume, mean) = self.mean_gradients(coords);
                // On the 3D DOFs also for an expanded element (condensed by the caller).
                let n3 = self.dof_count_3d();
                let mut hourglass = vec![0.0; n3 * n3];
                self.hourglass_stiffness(coords, d_elastic, &mean, volume, &mut hourglass);
                (vec![nodal(&mean, volume)], Some(hourglass))
            }
            Formulation::Incompatible => {
                let (x, inv0) = self.incompatible_coupling(coords, d_elastic);
                let gradients = self
                    .rule
                    .points
                    .iter()
                    .zip(&self.rule.weights)
                    .map(|(&p, &w)| {
                        let point = self.point(p, coords);
                        let mut g = nodal(&point.dndx, point.det * w);
                        let modes = Self::mode_gradients(p, &inv0);
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

    /// Whether the displacement gradient is `Σ_a u_a ⊗ ∇N_a` at every point: not for the
    /// incompatible-mode element and the expanded plane elements.
    pub fn has_nodal_gradients(&self) -> bool {
        self.expansion().is_none() && self.formulation != Formulation::Incompatible
    }

    /// The integration points of an element whose displacement gradient is
    /// `Σ_a u_a ⊗ ∇N_a` at every point, as the graphics card takes them: `None` for the
    /// incompatible-mode element and for expanded plane elements, whose operators are not of
    /// that form.
    pub fn nodal_gradients(
        &self,
        coords: &[[f64; 3]],
        d_elastic: &[[f64; 6]; 6],
    ) -> Option<NodalGradients> {
        if !self.has_nodal_gradients() {
            return None;
        }
        match self.formulation {
            Formulation::Standard => Some(NodalGradients {
                points: (0..self.rule.len())
                    .map(|ip| {
                        let (det, dndx) = self.rule_point(ip, coords);
                        (det * self.rule.weights[ip], dndx)
                    })
                    .collect(),
                hourglass: None,
            }),
            Formulation::Reduced => {
                let (volume, mean) = self.mean_gradients(coords);
                let epsilon = self.hourglass_coefficient(d_elastic, &mean, volume);
                Some(NodalGradients {
                    points: vec![(volume, mean)],
                    hourglass: Some(epsilon),
                })
            }
            Formulation::Incompatible => None,
        }
    }

    /// The integration points as general operators on the element's DOFs (three per node
    /// of the element, for plane and axisymmetric elements those of the 2D nodes), the
    /// forces being `Σ_points w Lᵀ P` plus the constant matrix times the displacements, as
    /// [`SolidElement::nonlinear`] computes them.
    pub fn dense_operators(
        &self,
        coords: &[[f64; 3]],
        d_elastic: &[[f64; 6]; 6],
    ) -> DenseOperators {
        let Some(x) = self.expansion() else {
            let (gradients, hourglass) = self.gradients(coords, d_elastic, None);
            return DenseOperators {
                points: gradients.into_iter().map(|g| (g.weight, g.l)).collect(),
                matrix: hourglass,
            };
        };
        let (gradients, hourglass) = self.gradients(&x.coordinates(coords), d_elastic, Some(x));
        let n2 = self.dof_count();
        DenseOperators {
            points: gradients
                .into_iter()
                .map(|g| (x.scale * g.weight, g.l))
                .collect(),
            matrix: hourglass.map(|hg| {
                let mut out = vec![0.0; n2 * n2];
                x.condense_matrix(&hg, &mut out);
                out
            }),
        }
    }

    /// Internal nodal forces (added to `fe`, 3 per node) and, when `ke` is given, the
    /// tangent stiffness (row-major, overwritten) for the nodal displacements `u`. For every
    /// integration point `material(point, kinematics)` returns the stress (second
    /// Piola-Kirchhoff under NLGEOM) and the tangent on engineering strains; the caller
    /// keeps whatever state it updates there. `d_elastic` serves the hourglass control and
    /// the incompatible modes.
    ///
    /// Plane and axisymmetric elements integrate their expanded 3D element on the 2D DOFs
    /// directly: the gradient operators of the integration points are condensed (`L T` for
    /// the expansion `u₃ = T u₂`), which gives `Tᵀ f₃` and `Tᵀ K₃ T` at a fraction of the
    /// cost of the 3D element matrix.
    #[allow(clippy::too_many_arguments)]
    pub fn nonlinear<M>(
        &self,
        coords: &[[f64; 3]],
        u: &[f64],
        nlgeom: bool,
        d_elastic: &[[f64; 6]; 6],
        material: M,
        fe: &mut [f64],
        ke: Option<&mut Vec<f64>>,
    ) where
        M: FnMut(usize, &PointKinematics) -> ([f64; 6], [[f64; 6]; 6]),
    {
        self.nonlinear_with_volume(coords, u, nlgeom, d_elastic, material, fe, ke, None);
    }

    /// [`SolidElement::nonlinear`], and with `current` the current volume and its
    /// gradient `∂V/∂u = Σ_points w cof(F) : L` (`J F⁻ᵀ ∇N` integrated, for an element of
    /// nodal gradients), added to it: the volumetric rate of the formulation itself, which
    /// the bulk viscosity of the explicit steps takes for the elements whose operators are
    /// not nodal gradients.
    #[allow(clippy::too_many_arguments)]
    pub fn nonlinear_with_volume<M>(
        &self,
        coords: &[[f64; 3]],
        u: &[f64],
        nlgeom: bool,
        d_elastic: &[[f64; 6]; 6],
        material: M,
        fe: &mut [f64],
        ke: Option<&mut Vec<f64>>,
        current: Option<&mut (f64, Vec<f64>)>,
    ) where
        M: FnMut(usize, &PointKinematics) -> ([f64; 6], [[f64; 6]; 6]),
    {
        let Some(x) = self.expansion() else {
            let (gradients, hourglass) = self.gradients(coords, d_elastic, None);
            let ndof = self.dof_count_3d();
            let hourglass = hourglass.as_deref();
            self.integrate(
                &gradients, hourglass, ndof, 1.0, u, nlgeom, material, fe, ke, current,
            );
            return;
        };
        let (gradients, hourglass) = self.gradients(&x.coordinates(coords), d_elastic, Some(x));
        let n2 = self.dof_count();
        let hourglass = hourglass.map(|hg| {
            let mut out = vec![0.0; n2 * n2];
            x.condense_matrix(&hg, &mut out);
            // The condensation scaled it already.
            out.iter_mut().for_each(|v| *v /= x.scale);
            out
        });
        let hourglass = hourglass.as_deref();
        self.integrate(
            &gradients, hourglass, n2, x.scale, u, nlgeom, material, fe, ke, current,
        );
    }

    /// Integrates the forces and the tangent over the points `gradients` (operators of
    /// `ndof` columns) for the displacements `u` of those `ndof` DOFs, every contribution
    /// scaled by `scale`; `hourglass` is a constant stiffness to add.
    #[allow(clippy::too_many_arguments)]
    fn integrate<M>(
        &self,
        gradients: &[Gradient],
        hourglass: Option<&[f64]>,
        ndof: usize,
        scale: f64,
        u: &[f64],
        nlgeom: bool,
        mut material: M,
        fe: &mut [f64],
        mut ke: Option<&mut Vec<f64>>,
        mut current: Option<&mut (f64, Vec<f64>)>,
    ) where
        M: FnMut(usize, &PointKinematics) -> ([f64; 6], [[f64; 6]; 6]),
    {
        debug_assert_eq!(u.len(), ndof);
        debug_assert_eq!(fe.len(), ndof);
        if let Some(ke) = ke.as_deref_mut() {
            ke.clear();
            ke.resize(ndof * ndof, 0.0);
        }
        // Column-major operators: `bt[col]` holds the six strain rows of one DOF, so the
        // products below are dot products of short fixed-size arrays.
        let mut bt = vec![[0.0; 6]; ndof];
        let mut dbt = vec![[0.0; 6]; ndof];
        let mut g = vec![[0.0; 3]; ndof];
        const IJ: [(usize, usize); 6] = [(0, 0), (1, 1), (2, 2), (0, 1), (1, 2), (2, 0)];
        for (ip, gradient) in gradients.iter().enumerate() {
            // The nine rows `L_(k,c)` of the operator.
            let rows: Vec<&[f64]> = gradient.l.chunks_exact(ndof).collect();
            let lrow = |k: usize, c: usize| rows[k * 3 + c];
            // H = L u, F = I + H.
            let mut h = [[0.0; 3]; 3];
            for (k, hk) in h.iter_mut().enumerate() {
                for (c, hkc) in hk.iter_mut().enumerate() {
                    *hkc = lrow(k, c).iter().zip(u).map(|(a, b)| a * b).sum();
                }
            }
            let mut f = [[0.0; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    f[i][j] = if i == j { 1.0 } else { 0.0 } + if nlgeom { h[i][j] } else { 0.0 };
                }
            }
            let mut strain = [0.0; 6];
            for (r, &(i, j)) in IJ.iter().enumerate() {
                let mut e = h[i][j] + h[j][i];
                if nlgeom {
                    e += (0..3).map(|k| h[k][i] * h[k][j]).sum::<f64>();
                }
                strain[r] = if i == j { 0.5 * e } else { e };
            }
            // B_L = ∂E/∂u: normal rows Σ_k F_ki L_(k,i), shear rows Σ_k (F_ki L_(k,j) + F_kj L_(k,i)).
            bt.iter_mut().for_each(|col| *col = [0.0; 6]);
            for (k, fk) in f.iter().enumerate() {
                for (r, &(i, j)) in IJ.iter().enumerate() {
                    let (li, lj) = (lrow(k, i), lrow(k, j));
                    if i == j {
                        for (col, a) in bt.iter_mut().zip(li) {
                            col[r] += fk[i] * a;
                        }
                    } else {
                        for ((col, a), c) in bt.iter_mut().zip(lj).zip(li) {
                            col[r] += fk[i] * a + fk[j] * c;
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
            let w = scale * gradient.weight;
            if let Some((volume, dv)) = current.as_deref_mut() {
                // J F⁻ᵀ is the cofactor matrix of F (of the full H, also for a
                // geometrically linear step).
                let fc: [[f64; 3]; 3] = std::array::from_fn(|i| {
                    std::array::from_fn(|j| if i == j { 1.0 } else { 0.0 } + h[i][j])
                });
                let cof = cofactors(&fc);
                *volume += w * (0..3).map(|c| fc[0][c] * cof[0][c]).sum::<f64>();
                dv.resize(ndof, 0.0);
                for (k, row) in cof.iter().enumerate() {
                    for (c, value) in row.iter().enumerate() {
                        let factor = w * value;
                        for (o, l) in dv.iter_mut().zip(lrow(k, c)) {
                            *o += factor * l;
                        }
                    }
                }
            }
            for (o, col) in fe.iter_mut().zip(&bt) {
                *o += w * dot6(col, &s);
            }
            let Some(ke) = ke.as_deref_mut() else {
                continue;
            };
            // Material stiffness Bᵀ D B (upper triangle).
            for (dcol, col) in dbt.iter_mut().zip(&bt) {
                for (r, drow) in d.iter().enumerate() {
                    dcol[r] = dot6(drow, col);
                }
            }
            for (a, ba) in bt.iter().enumerate() {
                let row = &mut ke[a * ndof..(a + 1) * ndof];
                for (c, dbc) in dbt.iter().enumerate().skip(a) {
                    row[c] += w * dot6(ba, dbc);
                }
            }
            if nlgeom {
                // Initial-stress stiffness Lᵀ (S ⊗ I) L: for every direction k,
                // Σ_{c,c'} L_(k,c)ᵀ S_cc' L_(k,c').
                // (`S` is symmetric: its columns are its rows.)
                for k in 0..3 {
                    g.iter_mut().for_each(|col| *col = [0.0; 3]);
                    for (cp, s_cp) in [[s[0], s[3], s[5]], [s[3], s[1], s[4]], [s[5], s[4], s[2]]]
                        .iter()
                        .enumerate()
                    {
                        for (gc, a) in g.iter_mut().zip(lrow(k, cp)) {
                            for (gcc, scc) in gc.iter_mut().zip(s_cp) {
                                *gcc += scc * a;
                            }
                        }
                    }
                    let (l0, l1, l2) = (lrow(k, 0), lrow(k, 1), lrow(k, 2));
                    for a in 0..ndof {
                        let la = [l0[a], l1[a], l2[a]];
                        if la == [0.0; 3] {
                            continue;
                        }
                        let row = &mut ke[a * ndof..(a + 1) * ndof];
                        for (cc, gc) in g.iter().enumerate().skip(a) {
                            row[cc] += w * (la[0] * gc[0] + la[1] * gc[1] + la[2] * gc[2]);
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
                fe[a] += scale * row.iter().zip(u).map(|(k, u)| k * u).sum::<f64>();
            }
            if let Some(ke) = ke {
                for (k, h) in ke.iter_mut().zip(hg) {
                    *k += scale * h;
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

    /// The dense operators reproduce `nonlinear` (forces of a Saint Venant-Kirchhoff
    /// material under NLGEOM), and the current volume of the reference configuration is the
    /// one of `volume_mean_gradients`, for 3D, incompatible-mode and expanded 2D elements.
    #[test]
    fn dense_operators_give_the_forces_and_the_volume() {
        let d = isotropic(210_000.0, 0.3);
        let mut elements: Vec<(SolidElement, Vec<[f64; 3]>)> = [
            ElementType::C3D8,
            ElementType::C3D8R,
            ElementType::C3D8I,
            ElementType::C3D20R,
            ElementType::C3D10,
        ]
        .into_iter()
        .map(distorted)
        .collect();
        for kind in [
            ElementType::CPE4,
            ElementType::CPS4R,
            ElementType::CAX4,
            ElementType::CAX8R,
            ElementType::CPE6,
        ] {
            let e = crate::plane::plane(kind, Some(0.3)).unwrap();
            let corners: &[[f64; 2]] = if kind == ElementType::CPE6 {
                &[[1.0, 0.0], [2.0, 0.1], [1.4, 1.1]]
            } else {
                &[[1.0, 0.0], [2.0, 0.1], [2.2, 1.0], [0.9, 1.2]]
            };
            let n = corners.len();
            // Corners, then the mid-side nodes (slightly off the middle).
            let coords: Vec<[f64; 3]> = (0..kind.node_count())
                .map(|i| {
                    if i < n {
                        [corners[i][0], corners[i][1], 0.0]
                    } else {
                        let (a, b) = (corners[i - n], corners[(i - n + 1) % n]);
                        [0.5 * (a[0] + b[0]) + 0.02, 0.5 * (a[1] + b[1]) - 0.01, 0.0]
                    }
                })
                .collect();
            elements.push((e, coords));
        }
        for (e, coords) in elements {
            let ndof = e.dof_count();
            let kind = e.kind;
            let operators = e.dense_operators(&coords, &d);
            let mut u = displacements(ndof, 1e-2);
            if e.expansion().is_some() {
                // In-plane motion only.
                for (i, v) in u.iter_mut().enumerate() {
                    if i % 3 == 2 {
                        *v = 0.0;
                    }
                }
            }
            let mut expected = vec![0.0; ndof];
            let mut current = (0.0, Vec::new());
            e.nonlinear_with_volume(
                &coords,
                &u,
                true,
                &d,
                linear(&d),
                &mut expected,
                None,
                Some(&mut current),
            );
            let mut f = vec![0.0; ndof];
            let mut volume = 0.0;
            for (w, l) in &operators.points {
                let mut h = [[0.0; 3]; 3];
                for (k, hk) in h.iter_mut().enumerate() {
                    for (c, hkc) in hk.iter_mut().enumerate() {
                        *hkc = (0..ndof).map(|j| l[(3 * k + c) * ndof + j] * u[j]).sum();
                    }
                }
                let hh = |i: usize, j: usize| (0..3).map(|k| h[k][i] * h[k][j]).sum::<f64>();
                let strain = [
                    h[0][0] + 0.5 * hh(0, 0),
                    h[1][1] + 0.5 * hh(1, 1),
                    h[2][2] + 0.5 * hh(2, 2),
                    h[0][1] + h[1][0] + hh(0, 1),
                    h[1][2] + h[2][1] + hh(1, 2),
                    h[2][0] + h[0][2] + hh(2, 0),
                ];
                let s: Vec<f64> = (0..6).map(|r| dot6(&d[r], &strain)).collect();
                let sm = [[s[0], s[3], s[5]], [s[3], s[1], s[4]], [s[5], s[4], s[2]]];
                // P = F S.
                for k in 0..3 {
                    for c in 0..3 {
                        let p: f64 = (0..3)
                            .map(|m| (if k == m { 1.0 } else { 0.0 } + h[k][m]) * sm[m][c])
                            .sum();
                        for j in 0..ndof {
                            f[j] += w * p * l[(3 * k + c) * ndof + j];
                        }
                    }
                }
                volume += w;
            }
            if let Some(m) = &operators.matrix {
                for a in 0..ndof {
                    f[a] += (0..ndof).map(|j| m[a * ndof + j] * u[j]).sum::<f64>();
                }
            }
            let scale = expected.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            for (a, b) in expected.iter().zip(&f) {
                assert!((a - b).abs() < 1e-10 * scale, "{kind}: {a} vs {b}");
            }
            // The reference configuration.
            let (v0, g0) = e.volume_mean_gradients(&coords);
            assert!((volume - v0).abs() < 1e-10 * v0, "{kind}: {volume} vs {v0}");
            let mut reference = (0.0, Vec::new());
            e.nonlinear_with_volume(
                &coords,
                &vec![0.0; ndof],
                true,
                &d,
                linear(&d),
                &mut vec![0.0; ndof],
                None,
                Some(&mut reference),
            );
            assert!((reference.0 - v0).abs() < 1e-10 * v0, "{kind}");
            // The incompatible modes add their own (small) volume change in a distorted
            // element.
            let tolerance = if kind == ElementType::C3D8I {
                1e-2
            } else {
                1e-9
            };
            let gscale = g0.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            for (a, b) in reference.1.iter().zip(&g0) {
                assert!(
                    (a - v0 * b).abs() < tolerance * v0 * gscale,
                    "{kind}: {a} vs {}",
                    v0 * b
                );
            }
            assert!(current.0 > 0.0);
        }
    }

    #[test]
    fn nodal_gradients_give_the_stiffness_times_the_displacements() {
        let d = isotropic(210_000.0, 0.3);
        for kind in [
            ElementType::C3D8,
            ElementType::C3D8R,
            ElementType::C3D20R,
            ElementType::C3D4,
            ElementType::C3D10,
            ElementType::C3D15,
        ] {
            let (e, coords) = distorted(kind);
            let ndof = e.dof_count();
            let mut k = Vec::new();
            e.stiffness(&coords, &d, &mut k);
            let u = displacements(ndof, 1e-3);
            let gradients = e.nodal_gradients(&coords, &d).unwrap();
            let mut f = vec![0.0; ndof];
            for (w, g) in &gradients.points {
                let mut h = [[0.0; 3]; 3];
                for (a, ga) in g.iter().enumerate() {
                    for (k, hk) in h.iter_mut().enumerate() {
                        for (c, hkc) in hk.iter_mut().enumerate() {
                            *hkc += u[3 * a + k] * ga[c];
                        }
                    }
                }
                let strain = [
                    h[0][0],
                    h[1][1],
                    h[2][2],
                    h[0][1] + h[1][0],
                    h[1][2] + h[2][1],
                    h[2][0] + h[0][2],
                ];
                let s: Vec<f64> = (0..6).map(|r| dot6(&d[r], &strain)).collect();
                let sm = [[s[0], s[3], s[5]], [s[3], s[1], s[4]], [s[5], s[4], s[2]]];
                for (a, ga) in g.iter().enumerate() {
                    for k in 0..3 {
                        f[3 * a + k] += w * (0..3).map(|c| sm[k][c] * ga[c]).sum::<f64>();
                    }
                }
            }
            if let Some(epsilon) = gradients.hourglass {
                let g = &gradients.points[0].1;
                for h in &SolidElement::HOURGLASS_BASE {
                    let hz: f64 = (0..8).map(|a| h[a] * coords[a][2]).sum();
                    let gamma: Vec<f64> = (0..8).map(|a| h[a] - hz * g[a][2]).collect();
                    for k in 0..3 {
                        let q: f64 = (0..8).map(|a| gamma[a] * u[3 * a + k]).sum();
                        for a in 0..8 {
                            f[3 * a + k] += epsilon * gamma[a] * q;
                        }
                    }
                }
            }
            let scale = f.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            for a in 0..ndof {
                let ku: f64 = (0..ndof).map(|c| k[a * ndof + c] * u[c]).sum();
                assert!(
                    (ku - f[a]).abs() < 1e-10 * scale,
                    "{kind}: {ku} vs {}",
                    f[a]
                );
            }
        }
        let (e, coords) = distorted(ElementType::C3D8I);
        assert!(e.nodal_gradients(&coords, &d).is_none());
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
