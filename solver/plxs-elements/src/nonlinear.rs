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

fn det3(a: &[[f64; 3]; 3]) -> f64 {
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
}

/// The displacement-gradient operators of the integration points, in one buffer:
/// `l[(ip * 9 + k * 3 + c) * ndof + col]`.
struct Gradients {
    l: Vec<f64>,
    weights: Vec<f64>,
    ndof: usize,
}

impl Gradients {
    fn new(points: usize, ndof: usize) -> Self {
        Self {
            l: vec![0.0; points * 9 * ndof],
            weights: vec![0.0; points],
            ndof,
        }
    }

    /// The nine rows of the operator at point `ip`.
    fn point_mut(&mut self, ip: usize) -> &mut [f64] {
        let size = 9 * self.ndof;
        &mut self.l[ip * size..(ip + 1) * size]
    }
}

/// The DOFs the operators act on, which the tangent's products are restricted to: other DOFs
/// (the out-of-plane DOF of plane and axisymmetric elements) have zero rows and columns.
/// `of[k]`: those the rows `L_(k,c)` of direction `k` act on, which the initial-stress
/// stiffness of that direction couples (a third of the DOFs of a solid). Taken from the
/// structure of the operators, so a superset of their nonzero columns.
struct Columns {
    all: Vec<usize>,
    of: [Vec<usize>; 3],
}

impl SolidElement {
    fn columns(&self, condense: Option<&Expansion>) -> Columns {
        // C3D8I folds the modes into every column of `L`.
        let mixed = self.formulation == Formulation::Incompatible;
        let mut of: [Vec<usize>; 3] = Default::default();
        let all: Vec<usize> = match condense {
            None => {
                let ndof = self.dof_count_3d();
                for (k, columns) in of.iter_mut().enumerate() {
                    *columns = if mixed {
                        (0..ndof).collect()
                    } else {
                        (k..ndof).step_by(3).collect()
                    };
                }
                (0..ndof).collect()
            }
            Some(x) => {
                let mut all = Vec::new();
                for (i, m) in x.dof_map().iter().enumerate() {
                    if let Some((d, _)) = *m {
                        all.push(d);
                        of[i % 3].push(d);
                    }
                }
                for list in of.iter_mut().chain([&mut all]) {
                    list.sort_unstable();
                    list.dedup();
                }
                if mixed {
                    of = std::array::from_fn(|_| all.clone());
                }
                all
            }
        };
        Columns { all, of }
    }
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
    ) -> (Gradients, Option<Vec<f64>>) {
        let ndof = condense.map_or(self.dof_count_3d(), |x| 3 * x.nodes);
        let nodal = |dndx: &[[f64; 3]], l: &mut [f64]| match condense {
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
        };
        match self.formulation {
            Formulation::Standard => {
                let mut gradients = Gradients::new(self.rule.weights.len(), ndof);
                for (ip, &w) in self.rule.weights.iter().enumerate() {
                    let (det, dndx) = self.rule_point(ip, coords);
                    nodal(&dndx, gradients.point_mut(ip));
                    gradients.weights[ip] = det * w;
                }
                (gradients, None)
            }
            Formulation::Reduced => {
                let (volume, mean) = self.mean_gradients(coords);
                // On the 3D DOFs, which the caller condenses.
                let n3 = self.dof_count_3d();
                let mut hourglass = vec![0.0; n3 * n3];
                self.hourglass_stiffness(coords, d_elastic, &mean, volume, &mut hourglass);
                let mut gradients = Gradients::new(1, ndof);
                nodal(&mean, gradients.point_mut(0));
                gradients.weights[0] = volume;
                (gradients, Some(hourglass))
            }
            Formulation::Incompatible => {
                let (x, inv0) = self.incompatible_coupling(coords, d_elastic);
                let mut gradients = Gradients::new(self.rule.weights.len(), ndof);
                for (ip, (&p, &w)) in self.rule.points.iter().zip(&self.rule.weights).enumerate() {
                    let point = self.point(p, coords);
                    gradients.weights[ip] = point.det * w;
                    let g = gradients.point_mut(ip);
                    nodal(&point.dndx, g);
                    let modes = Self::mode_gradients(p, &inv0);
                    // L_eff[(k,c)][j] = L[(k,c)][j] − Σ_m dM_m/dx_c X[3m+k][j].
                    for (m, dm) in modes.iter().enumerate() {
                        for k in 0..3 {
                            let row = &x[(3 * m + k) * ndof..(3 * m + k + 1) * ndof];
                            for (c, dmc) in dm.iter().enumerate() {
                                let l = &mut g[(k * 3 + c) * ndof..(k * 3 + c + 1) * ndof];
                                for (lj, xj) in l.iter_mut().zip(row) {
                                    *lj -= dmc * xj;
                                }
                            }
                        }
                    }
                }
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
        let Some(x) = self.expansion() else {
            let (gradients, hourglass) = self.gradients(coords, d_elastic, None);
            let hourglass = hourglass.as_deref();
            let columns = self.columns(None);
            self.integrate(
                &gradients, &columns, hourglass, 1.0, u, nlgeom, material, fe, ke,
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
        let columns = self.columns(Some(x));
        self.integrate(
            &gradients, &columns, hourglass, x.scale, u, nlgeom, material, fe, ke,
        );
    }

    /// Integrates the forces and the tangent over the points `gradients` (operators of
    /// `ndof` columns) for the displacements `u` of those `ndof` DOFs, every contribution
    /// scaled by `scale`; `hourglass` is a constant stiffness to add. The tangent's products
    /// run over `columns` only.
    #[allow(clippy::too_many_arguments)]
    fn integrate<M>(
        &self,
        gradients: &Gradients,
        columns: &Columns,
        hourglass: Option<&[f64]>,
        scale: f64,
        u: &[f64],
        nlgeom: bool,
        mut material: M,
        fe: &mut [f64],
        mut ke: Option<&mut Vec<f64>>,
    ) where
        M: FnMut(usize, &PointKinematics) -> ([f64; 6], [[f64; 6]; 6]),
    {
        let ndof = gradients.ndof;
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
        let active = &columns.all;
        const IJ: [(usize, usize); 6] = [(0, 0), (1, 1), (2, 2), (0, 1), (1, 2), (2, 0)];
        let points = gradients.l.chunks_exact(9 * ndof).zip(&gradients.weights);
        for (ip, (operator, &weight)) in points.enumerate() {
            // The nine rows `L_(k,c)` of the operator.
            let lrow = |k: usize, c: usize| &operator[(k * 3 + c) * ndof..(k * 3 + c + 1) * ndof];
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
                weight,
            };
            let (s, d) = material(ip, &kinematics);
            let w = scale * weight;
            for (o, col) in fe.iter_mut().zip(&bt) {
                *o += w * dot6(col, &s);
            }
            let Some(ke) = ke.as_deref_mut() else {
                continue;
            };
            // Material stiffness Bᵀ D B (upper triangle).
            for &c in active {
                for (r, drow) in d.iter().enumerate() {
                    dbt[c][r] = dot6(drow, &bt[c]);
                }
            }
            for (i, &a) in active.iter().enumerate() {
                let ba = &bt[a];
                let row = &mut ke[a * ndof..(a + 1) * ndof];
                for &c in &active[i..] {
                    row[c] += w * dot6(ba, &dbt[c]);
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
                    let columns = &columns.of[k];
                    for (i, &a) in columns.iter().enumerate() {
                        let la = [l0[a], l1[a], l2[a]];
                        if la == [0.0; 3] {
                            continue;
                        }
                        let row = &mut ke[a * ndof..(a + 1) * ndof];
                        for &cc in &columns[i..] {
                            let gc = &g[cc];
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

    /// Nodes of a distorted 2D quadrilateral (4 or 8 nodes) away from the axis.
    fn distorted_quad(nodes: usize) -> Vec<[f64; 3]> {
        let corners = [[1.0, 0.0], [2.1, 0.1], [2.0, 1.2], [0.9, 1.0]];
        let mut coords: Vec<[f64; 3]> = corners.iter().map(|c| [c[0], c[1], 0.0]).collect();
        if nodes == 8 {
            for a in 0..4 {
                let (p, q) = (corners[a], corners[(a + 1) % 4]);
                coords.push([0.5 * (p[0] + q[0]) + 0.03, 0.5 * (p[1] + q[1]) - 0.02, 0.0]);
            }
        }
        coords
    }

    #[test]
    fn the_tangent_of_plane_and_axisymmetric_elements_is_the_derivative_of_their_force() {
        // The tangent skips the DOFs the expanded element does not move (the out-of-plane
        // DOF); the finite differences cover all of them.
        let d = isotropic(1000.0, 0.3);
        for kind in [
            ElementType::CAX8,
            ElementType::CAX4,
            ElementType::CAX4R,
            ElementType::CPE8R,
            ElementType::CPE4,
        ] {
            let e = crate::plane::plane(kind, None).unwrap();
            let coords = distorted_quad(e.node_count());
            let ndof = e.dof_count();
            let u = displacements(ndof, 0.1);
            let mut fe = vec![0.0; ndof];
            let mut ke = Vec::new();
            e.nonlinear(&coords, &u, true, &d, linear(&d), &mut fe, Some(&mut ke));
            let h = 1e-6;
            let scale = ke.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            assert!(scale > 0.0);
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
