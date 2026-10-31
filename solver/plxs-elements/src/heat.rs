//! Heat conduction on the solid elements (Fourier's law, isotropic conductivity) and the
//! scalar operators the thermal procedures need: conductivity and heat capacity matrices,
//! temperature gradients at the integration points, body heat sources, and the integration
//! points of a face for surface fluxes, films and radiation.
//!
//! Formulation: the standard Galerkin weak form of the heat equation
//! `ρc Ṫ - ∇·(k ∇T) = r` with the element's shape functions for the temperature
//! (Zienkiewicz & Taylor, vol. 1, chapter 7; Lewis, Nithiarasu & Seetharamu, "Fundamentals
//! of the Finite Element Method for Heat and Fluid Flow", chapters 3 and 7). The capacity
//! matrix is the consistent one: CalculiX' transient results are reproduced by it (checked
//! against ccx on single distorted elements, see `docs/solver/SCHNITTSTELLEN.md`).
//!
//! Integration uses the element's rule, except for the one-point C3D8R, whose conduction
//! matrix would be singular (hourglass modes): it is integrated with 2×2×2 points, while its
//! fluxes are output at its single point like its stresses.
//!
//! No CalculiX code was consulted.

use crate::shape::{IntegrationRule, face_rule, face_shape};
use crate::solid::{Formulation, SolidElement};

/// An integration point of a face: the element's local node indices on the face with their
/// shape function values, and the area weight `w · |dA|`.
#[derive(Clone, Debug, PartialEq)]
pub struct FacePoint {
    pub nodes: Vec<usize>,
    pub n: Vec<f64>,
    pub area: f64,
    /// Coordinates of the point.
    pub x: [f64; 3],
}

/// The 3D view of an element for the scalar operators: expanded coordinates and, per 3D
/// node, its 2D node (identity for solids) with the factor on volumes and areas.
struct Scalar3d {
    coords: Vec<[f64; 3]>,
    map: Vec<usize>,
    scale: f64,
}

impl Scalar3d {
    fn expand(&self, nodal: &[f64]) -> Vec<f64> {
        self.map.iter().map(|&a| nodal[a]).collect()
    }
}

impl SolidElement {
    fn scalar_3d(&self, coords: &[[f64; 3]]) -> Scalar3d {
        match self.expansion() {
            Some(x) => Scalar3d {
                coords: x.coordinates(coords),
                map: x.scalar_map(),
                scale: x.scale,
            },
            None => Scalar3d {
                coords: coords.to_vec(),
                map: (0..coords.len()).collect(),
                scale: 1.0,
            },
        }
    }

    fn nodal_3d(&self, nodal: &[f64]) -> Vec<f64> {
        match self.expansion() {
            Some(x) => x.scalar_map().iter().map(|&a| nodal[a]).collect(),
            None => nodal.to_vec(),
        }
    }

    /// The integration rule of the thermal matrices.
    pub fn thermal_rule(&self) -> IntegrationRule {
        if self.formulation == Formulation::Reduced {
            IntegrationRule::hex(2)
        } else {
            self.rule.clone()
        }
    }

    /// Shape function values at the points of `rule` (`[point][node]`).
    /// For plane and axisymmetric elements the values belong to the nodes of the expanded
    /// element.
    pub fn shape_values(&self, rule: &IntegrationRule) -> Vec<Vec<f64>> {
        let count = self.family.node_count();
        rule.points
            .iter()
            .map(|&p| {
                let mut n = vec![0.0; count];
                let mut dn = vec![[0.0; 3]; count];
                self.family.shape(p, &mut n, &mut dn);
                n
            })
            .collect()
    }

    /// The temperatures at which material properties (conductivity, heat capacity,
    /// expansion) are evaluated at the points of `rule`: interpolated with the shape
    /// functions, except for linear hexahedra (C3D8, C3D8I, C3D8R), which use the element's
    /// mean temperature at all points. That is what CalculiX does, found with ccx: its heat
    /// flux in a C3D8 is `k(T_mean) ∇T`, and its thermal strains in a C3D8 follow from the
    /// mean temperature.
    pub fn material_temperatures(&self, rule: &IntegrationRule, nodal: &[f64]) -> Vec<f64> {
        let nodal = &self.nodal_3d(nodal)[..];
        if self.family == crate::Family::Hex8 {
            vec![nodal.iter().sum::<f64>() / nodal.len() as f64; rule.len()]
        } else {
            self.shape_values(rule)
                .iter()
                .map(|n| n.iter().zip(nodal).map(|(a, b)| a * b).sum())
                .collect()
        }
    }

    /// Interpolates nodal values to the points of the element's own rule (where strains
    /// and stresses live).
    pub fn at_integration_points(&self, nodal: &[f64]) -> Vec<f64> {
        let nodal = &self.nodal_3d(nodal)[..];
        self.shape_values(&self.rule)
            .iter()
            .map(|n| n.iter().zip(nodal).map(|(a, b)| a * b).sum())
            .collect()
    }

    /// Conductivity matrix `∫ k ∇Nᵀ ∇N dV` (row-major n × n) with the conductivity `k` at
    /// the points of [`Self::thermal_rule`] and the matching matrix of the heat capacity
    /// `∫ ρc N Nᵀ dV` when `capacity` is given (`ρc` per point).
    pub fn heat_matrices(
        &self,
        coords: &[[f64; 3]],
        conductivity: &[f64],
        capacity: Option<&[f64]>,
        k_out: &mut Vec<f64>,
        c_out: &mut Vec<f64>,
    ) {
        let x = self.scalar_3d(coords);
        let n3 = self.family.node_count();
        let mut k3 = vec![0.0; n3 * n3];
        let mut c3 = vec![0.0; if capacity.is_some() { n3 * n3 } else { 0 }];
        let rule = self.thermal_rule();
        for (g, (&p, &w)) in rule.points.iter().zip(&rule.weights).enumerate() {
            let point = self.point(p, &x.coords);
            let f = point.det * w;
            let kf = f * conductivity[g];
            for i in 0..n3 {
                let gi = point.dndx[i];
                for j in i..n3 {
                    let gj = point.dndx[j];
                    let v = kf * (gi[0] * gj[0] + gi[1] * gj[1] + gi[2] * gj[2]);
                    k3[i * n3 + j] += v;
                    if i != j {
                        k3[j * n3 + i] += v;
                    }
                }
            }
            if let Some(c) = capacity {
                let cf = f * c[g];
                for i in 0..n3 {
                    for j in i..n3 {
                        let v = cf * point.n[i] * point.n[j];
                        c3[i * n3 + j] += v;
                        if i != j {
                            c3[j * n3 + i] += v;
                        }
                    }
                }
            }
        }
        let n = self.node_count();
        let condense = |m3: &[f64], out: &mut Vec<f64>| {
            out.clear();
            out.resize(n * n, 0.0);
            for (i, &a) in x.map.iter().enumerate() {
                for (j, &b) in x.map.iter().enumerate() {
                    out[a * n + b] += x.scale * m3[i * n3 + j];
                }
            }
        };
        condense(&k3, k_out);
        if capacity.is_some() {
            condense(&c3, c_out);
        } else {
            c_out.clear();
        }
    }

    /// Temperature gradients at the points of the element's own rule (where the heat flux
    /// is output) for nodal temperatures `t`.
    pub fn temperature_gradients(&self, coords: &[[f64; 3]], t: &[f64]) -> Vec<[f64; 3]> {
        let x = self.scalar_3d(coords);
        let t = &x.expand(t)[..];
        self.rule
            .points
            .iter()
            .map(|&p| {
                let point = self.point(p, &x.coords);
                let mut g = [0.0; 3];
                for (d, ti) in point.dndx.iter().zip(t) {
                    for k in 0..3 {
                        g[k] += d[k] * ti;
                    }
                }
                g
            })
            .collect()
    }

    /// Equivalent nodal heat of a body heat source per unit volume `r` (`*DFLUX, BF`),
    /// added to `out` (one entry per node).
    pub fn body_heat(&self, coords: &[[f64; 3]], r: f64, out: &mut [f64]) {
        let x = self.scalar_3d(coords);
        let rule = self.thermal_rule();
        for (&p, &w) in rule.points.iter().zip(&rule.weights) {
            let point = self.point(p, &x.coords);
            let f = point.det * w * r * x.scale;
            for (&a, ni) in x.map.iter().zip(&point.n) {
                out[a] += ni * f;
            }
        }
    }

    /// The integration points of face `face` (1-based CalculiX number) for fluxes, films
    /// and radiation. The rules are those CalculiX uses for these loads, found by comparing
    /// films on single distorted elements with ccx: one point on linear triangles and on the
    /// quadrilaterals of C3D6, three points on quadratic triangles, 3×3 on the
    /// quadrilaterals of C3D20 and 2×2 on all other quadrilaterals.
    ///
    /// For plane and axisymmetric elements `face` is the edge (F1..F4 of the input); the
    /// points carry the 2D nodes and the area of the full circumference.
    pub fn face_points(&self, coords: &[[f64; 3]], face: u8) -> Vec<FacePoint> {
        if let Some(x) = self.expansion() {
            let map = x.scalar_map();
            let mut points = self.face_points_3d(&x.coordinates(coords), x.face(face));
            for p in &mut points {
                p.nodes.iter_mut().for_each(|n| *n = map[*n]);
                p.area *= x.scale;
            }
            return points;
        }
        self.face_points_3d(coords, face)
    }

    fn face_points_3d(&self, coords: &[[f64; 3]], face: u8) -> Vec<FacePoint> {
        let faces = self.family.faces();
        let f = &faces[face as usize - 1];
        let quadratic = self.family.is_quadratic();
        let nodes: Vec<usize> = if quadratic {
            f.corners.iter().chain(f.mids).copied().collect()
        } else {
            f.corners.to_vec()
        };
        let (points, weights) = match (f.corners.len(), self.family) {
            // Linear triangles (C3D4, C3D6 ends) and the quadrilaterals of the linear wedge
            // take one point at the face centre.
            (3, _) if !quadratic => (vec![[1.0 / 3.0, 1.0 / 3.0]], vec![0.5]),
            (4, crate::Family::Wedge6) => (vec![[0.0, 0.0]], vec![4.0]),
            // Quadratic triangles: the 3-point interior rule.
            (3, _) => face_rule(3, false),
            // Quadrilaterals: 3×3 only on the fully integrated C3D20 (also as the expansion
            // of CPS8, CPE8, CAX8), 2×2 otherwise (C3D8, C3D8R, C3D8I, C3D20R, C3D15).
            (_, crate::Family::Hex20)
                if crate::plane::expanded_type(self.kind).unwrap_or(self.kind)
                    == plxs_core::ElementType::C3D20 =>
            {
                face_rule(4, true)
            }
            _ => face_rule(4, false),
        };
        let mut out = Vec::with_capacity(points.len());
        let mut n = vec![0.0; nodes.len()];
        let mut dn = vec![[0.0; 2]; nodes.len()];
        for (p, w) in points.iter().zip(weights) {
            face_shape(f.corners.len(), quadratic, *p, &mut n, &mut dn);
            let mut t1 = [0.0; 3];
            let mut t2 = [0.0; 3];
            let mut x = [0.0; 3];
            for (i, &node) in nodes.iter().enumerate() {
                for k in 0..3 {
                    t1[k] += dn[i][0] * coords[node][k];
                    t2[k] += dn[i][1] * coords[node][k];
                    x[k] += n[i] * coords[node][k];
                }
            }
            let normal = [
                t1[1] * t2[2] - t1[2] * t2[1],
                t1[2] * t2[0] - t1[0] * t2[2],
                t1[0] * t2[1] - t1[1] * t2[0],
            ];
            let area =
                (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt() * w;
            out.push(FacePoint {
                nodes: nodes.clone(),
                n: n.clone(),
                area,
                x,
            });
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use plxs_core::ElementType;

    use crate::solid;

    fn unit_cube() -> Vec<[f64; 3]> {
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ]
    }

    #[test]
    fn conduction_through_a_cube_is_fourier() {
        // A linear temperature field T = 10 x conducts k * 10 through the unit cube; the
        // nodal heat K·T is -k*10/4 on each of the four nodes at x = 0 and +k*10/4 at x = 1.
        let e = solid(ElementType::C3D8).unwrap();
        let coords = unit_cube();
        let (mut k, mut c) = (Vec::new(), Vec::new());
        e.heat_matrices(&coords, &[3.0; 8], Some(&[2.0; 8]), &mut k, &mut c);
        let t: Vec<f64> = coords.iter().map(|x| 10.0 * x[0]).collect();
        for i in 0..8 {
            let q: f64 = (0..8).map(|j| k[i * 8 + j] * t[j]).sum();
            let expected = if coords[i][0] == 0.0 { -7.5 } else { 7.5 };
            assert!((q - expected).abs() < 1e-12, "node {i}: {q}");
        }
        // Capacity: total ρc V.
        let total: f64 = c.iter().sum();
        assert!((total - 2.0).abs() < 1e-12);
        let g = e.temperature_gradients(&coords, &t);
        assert!(
            g.iter()
                .all(|g| (g[0] - 10.0).abs() < 1e-12 && g[1].abs() < 1e-12)
        );
    }

    #[test]
    fn faces_and_bodies_integrate_their_area_and_volume() {
        for kind in [ElementType::C3D8, ElementType::C3D20R, ElementType::C3D10] {
            let e = solid(kind).unwrap();
            let coords: Vec<[f64; 3]> = e
                .family
                .node_coordinates()
                .iter()
                .map(|p| [2.0 * p[0], 3.0 * p[1], 0.5 * p[2]])
                .collect();
            let mut heat = vec![0.0; e.node_count()];
            e.body_heat(&coords, 1.0, &mut heat);
            let volume = e.volume(&coords);
            assert!((heat.iter().sum::<f64>() - volume).abs() < 1e-12 * volume.max(1.0));
            let points = e.face_points(&coords, 1);
            let area: f64 = points.iter().map(|p| p.area).sum();
            let weighted: f64 = points
                .iter()
                .map(|p| p.area * p.n.iter().sum::<f64>())
                .sum();
            assert!(area > 0.0 && (weighted - area).abs() < 1e-12 * area);
        }
    }
}
