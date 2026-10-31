//! Cyclic symmetry (`*TIE, CYCLIC SYMMETRY` with `*CYCLIC SYMMETRY MODEL`).
//!
//! The model is one sector of a structure of `N` equal sectors around an axis. Every node
//! of the dependent (slave) side, rotated back by the sector angle `φ = 2π/N`, lies on the
//! independent (master) side; it is coupled to the master face it lands on with the face's
//! shape function values, as a `*TIE` does.
//!
//! - Without a nodal diameter (static steps, frequency steps without
//!   `*SELECT CYCLIC SYMMETRY MODES`) the whole structure deforms like the sector:
//!   `u_s = R(φ) Σ w_i u_i` (CalculiX manual, `*CYCLIC SYMMETRY MODEL`: nodal diameter 0).
//! - A mode of nodal diameter `k` is complex, `c = a + i b`, and the sector's neighbours see
//!   it rotated and shifted in phase: `c_s = e^{-ikφ} R(φ) Σ w_i c_i` (manual, "Cyclic
//!   symmetry calculations"). Real and imaginary parts are two copies of the sector, `a` on
//!   the DOFs `0..n`, `b` on `n..2n`; the constraint couples them:
//!   `a_s = cos(kφ) R a_m + sin(kφ) R b_m`, `b_s = cos(kφ) R b_m − sin(kφ) R a_m`.
//!
//! The direction of the angle is fixed by the geometry: the axis given in the input is
//! oriented so that the slave side is the master side rotated by `+φ` about it (CalculiX's
//! phase convention was read off its results: a mode's slave side lags the master side by
//! `kφ` whichever way the axis points in the input, and the `.dat` prints the opposite of
//! that axis as the reference direction of the turning direction).
//!
//! No CalculiX code was consulted.

use plxs_core::{Error, Model, Result};

use crate::tie::{self, Coupling};

/// The prepared cyclic symmetry of a model.
#[derive(Clone, Debug)]
pub struct CyclicSymmetry {
    pub sectors: u32,
    /// Sectors written to the `.frd` (`NGRAPH`).
    pub ngraph: u32,
    /// The sector angle `2π/N`.
    pub angle: f64,
    /// A point on the axis.
    pub origin: [f64; 3],
    /// Unit axis, oriented so that the slave side is the master side rotated by `+angle`.
    pub axis: [f64; 3],
    /// `R(+angle)` about `axis`.
    pub rotation: [[f64; 3]; 3],
    /// The slave nodes and the master nodes and weights they follow (before the rotation).
    pub couplings: Vec<Coupling>,
    /// Slave nodes that found no master face.
    pub missed: Vec<u32>,
    /// The label index of the constraints (`ModelConstraints::labels`).
    pub origin_label: u32,
}

/// The rotation by `angle` about the unit vector `axis` (Rodrigues).
pub fn rotation_about(axis: [f64; 3], angle: f64) -> [[f64; 3]; 3] {
    let (s, c) = angle.sin_cos();
    let [x, y, z] = axis;
    let t = 1.0 - c;
    [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
}

fn apply(r: &[[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
        r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
        r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
    ]
}

/// `x` rotated by `r` about the axis through `origin`.
pub fn rotate_point(r: &[[f64; 3]; 3], origin: [f64; 3], x: [f64; 3]) -> [f64; 3] {
    let d = apply(r, [x[0] - origin[0], x[1] - origin[1], x[2] - origin[2]]);
    [d[0] + origin[0], d[1] + origin[1], d[2] + origin[2]]
}

impl CyclicSymmetry {
    /// Projects the slave side of the model's cyclic symmetry tie, rotated back by the
    /// sector angle, on to the master side. Both orientations of the axis are tried; the one
    /// that ties more slave nodes (closer, on a tie) wins.
    pub fn new(model: &Model, origin_label: u32) -> Result<Option<CyclicSymmetry>> {
        let Some(definition) = &model.constraints.cyclic_symmetry else {
            return Ok(None);
        };
        let Some(t) = model
            .constraints
            .ties
            .iter()
            .find(|t| t.name == definition.tie)
        else {
            return Err(Error::undefined(
                "*CYCLIC SYMMETRY MODEL",
                "tie",
                &definition.tie,
            ));
        };
        if definition.sectors == 0 {
            return Err(Error::input("*CYCLIC SYMMETRY MODEL", "N must be positive"));
        }
        let [p, q] = definition.axis;
        let d = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
        let length = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if length == 0.0 {
            return Err(Error::input(
                "*CYCLIC SYMMETRY MODEL",
                "the two points of the axis coincide",
            ));
        }
        let unit = d.map(|c| c / length);
        let angle = 2.0 * std::f64::consts::PI / f64::from(definition.sectors);
        let slaves = tie::surface_nodes(model, "*TIE", &t.slave)?;
        let mut best: Option<(usize, f64, CyclicSymmetry)> = None;
        for sign in [1.0, -1.0] {
            let axis = unit.map(|c| c * sign);
            let back = rotation_about(axis, -angle);
            let mut coords = model.nodes.coordinates.clone();
            for &s in &slaves {
                coords[s as usize] = rotate_point(&back, p, coords[s as usize]);
            }
            let (couplings, missed) = tie::couplings(model, t, &coords)?;
            let distance: f64 = couplings.iter().map(|c| c.distance).sum();
            let candidate = CyclicSymmetry {
                sectors: definition.sectors,
                ngraph: definition.ngraph.max(1),
                angle,
                origin: p,
                axis,
                rotation: rotation_about(axis, angle),
                couplings,
                missed,
                origin_label,
            };
            let tied = candidate.couplings.len();
            if best
                .as_ref()
                .is_none_or(|(n, dist, _)| tied > *n || (tied == *n && distance < *dist))
            {
                best = Some((tied, distance, candidate));
            }
        }
        Ok(best.map(|(_, _, c)| c))
    }

    /// The nodal diameter 0 constraints `u_s − R Σ w_i u_i = 0` in global components on the
    /// primary numbering (`node * 3 + d`).
    pub fn real_constraints(&self) -> Vec<(Vec<(u32, f64)>, u32)> {
        let mut out = Vec::new();
        for c in &self.couplings {
            for d in 0..3u32 {
                let mut terms = vec![(c.slave * 3 + d, 1.0)];
                for &(m, w) in &c.masters {
                    for e in 0..3u32 {
                        let a = self.rotation[d as usize][e as usize] * w;
                        if a != 0.0 {
                            terms.push((m * 3 + e, -a));
                        }
                    }
                }
                out.push((terms, c.slave * 3 + d));
            }
        }
        out
    }

    /// The constraints of nodal diameter `k` on the doubled numbering (`b` copy offset by
    /// `n` DOFs), in global components.
    pub fn complex_constraints(&self, k: u32, n: u32) -> Vec<(Vec<(u32, f64)>, u32)> {
        let (sin, cos) = (f64::from(k) * self.angle).sin_cos();
        let mut out = Vec::new();
        for c in &self.couplings {
            for d in 0..3u32 {
                // a_s − cos R a_m − sin R b_m = 0
                let mut real = vec![(c.slave * 3 + d, 1.0)];
                // b_s − cos R b_m + sin R a_m = 0
                let mut imaginary = vec![(n + c.slave * 3 + d, 1.0)];
                for &(m, w) in &c.masters {
                    for e in 0..3u32 {
                        let r = self.rotation[d as usize][e as usize] * w;
                        if r == 0.0 {
                            continue;
                        }
                        let (am, bm) = (m * 3 + e, n + m * 3 + e);
                        push(&mut real, am, -cos * r);
                        push(&mut real, bm, -sin * r);
                        push(&mut imaginary, bm, -cos * r);
                        push(&mut imaginary, am, sin * r);
                    }
                }
                out.push((real, c.slave * 3 + d));
                out.push((imaginary, n + c.slave * 3 + d));
            }
        }
        out
    }

    /// The highest nodal diameter (`N/2`, rounded down).
    pub fn max_diameter(&self) -> u32 {
        self.sectors / 2
    }
}

fn push(terms: &mut Vec<(u32, f64)>, dof: u32, a: f64) {
    if a == 0.0 {
        return;
    }
    match terms.iter_mut().find(|(t, _)| *t == dof) {
        Some((_, v)) => *v += a,
        None => terms.push((dof, a)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rodrigues_rotates_x_into_y_about_z() {
        let r = rotation_about([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2);
        let v = apply(&r, [1.0, 0.0, 0.0]);
        assert!((v[0]).abs() < 1e-15 && (v[1] - 1.0).abs() < 1e-15 && v[2].abs() < 1e-15);
    }
}
