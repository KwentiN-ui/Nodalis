//! Local coordinate systems of nodes (`*TRANSFORM`).
//!
//! CalculiX manual, `*TRANSFORM`: a rectangular system (`TYPE=R`) is given by a point `a` on
//! the local x' axis and a point `b` in the local x'-y' plane; a cylindrical one (`TYPE=C`) by
//! two points on its axis, x' radial, z' along the axis from `a` to `b`, y' tangential, right
//! handed. The last transformation defined for a node takes effect. SPCs, `*EQUATION` terms
//! and concentrated loads at such a node refer to the local directions.

use std::collections::HashMap;

use plxs_core::Model;
use plxs_core::constraints::{Transform, TransformKind};

/// A rotation whose columns are the local axes in global coordinates: `u_global = R u_local`,
/// `R[d][k]` is the global component `d` of local axis `k`.
pub type Rotation = [[f64; 3]; 3];

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub(crate) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalized(a: [f64; 3]) -> Option<[f64; 3]> {
    let len = dot(a, a).sqrt();
    (len > 0.0).then(|| a.map(|c| c / len))
}

fn columns(e1: [f64; 3], e2: [f64; 3], e3: [f64; 3]) -> Rotation {
    [
        [e1[0], e2[0], e3[0]],
        [e1[1], e2[1], e3[1]],
        [e1[2], e2[2], e3[2]],
    ]
}

/// Any unit vector perpendicular to `axis` (unit).
fn perpendicular(axis: [f64; 3]) -> [f64; 3] {
    let helper = if axis[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    normalized(cross(axis, helper)).unwrap_or([1.0, 0.0, 0.0])
}

/// The rotation of a transformation at a point.
pub fn rotation(transform: &Transform, x: [f64; 3]) -> Option<Rotation> {
    match transform.kind {
        TransformKind::Rectangular => {
            let e1 = normalized(transform.a)?;
            let e3 = normalized(cross(transform.a, transform.b))?;
            let e2 = cross(e3, e1);
            Some(columns(e1, e2, e3))
        }
        TransformKind::Cylindrical => {
            let e3 = normalized(sub(transform.b, transform.a))?;
            let r = sub(x, transform.a);
            let radial = sub(r, e3.map(|c| c * dot(r, e3)));
            // A node on the axis has no radial direction; any perpendicular one will do.
            let e1 = normalized(radial)
                .filter(|_| dot(radial, radial).sqrt() > 1e-12 * dot(r, r).sqrt().max(1e-30))
                .unwrap_or_else(|| perpendicular(e3));
            let e2 = cross(e3, e1);
            Some(columns(e1, e2, e3))
        }
    }
}

/// The rotations of all transformed nodes by dense node index; the last transformation of a
/// node wins. Degenerate definitions (points on a line) are reported as warnings.
pub fn node_rotations(model: &Model, warnings: &mut Vec<String>) -> HashMap<u32, Rotation> {
    let mut rotations = HashMap::new();
    for transform in &model.constraints.transforms {
        let Some(nodes) = model.sets.node_sets.get(&transform.nset) else {
            warnings.push(format!(
                "*WARNING in *TRANSFORM: node set {} does not exist",
                transform.nset
            ));
            continue;
        };
        for &id in nodes {
            let Some(index) = model.nodes.index(id) else {
                continue;
            };
            match rotation(transform, model.nodes.coordinates[index as usize]) {
                Some(r) => {
                    rotations.insert(index, r);
                }
                None => warnings.push(format!(
                    "*WARNING in *TRANSFORM: the points of the system of node set {} are degenerate",
                    transform.nset
                )),
            }
        }
    }
    rotations
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rectangular_system_from_two_points() {
        // Manual example: x' = global y, y' = global z.
        let t = Transform {
            nset: "N".into(),
            kind: TransformKind::Rectangular,
            a: [0.0, 1.0, 0.0],
            b: [0.0, 0.0, 1.0],
        };
        let r = rotation(&t, [0.0; 3]).unwrap();
        // Columns: e1 = (0,1,0), e2 = (0,0,1), e3 = (1,0,0).
        assert_eq!([r[0][0], r[1][0], r[2][0]], [0.0, 1.0, 0.0]);
        assert_eq!([r[0][1], r[1][1], r[2][1]], [0.0, 0.0, 1.0]);
        assert_eq!([r[0][2], r[1][2], r[2][2]], [1.0, 0.0, 0.0]);
    }

    #[test]
    fn cylindrical_system_is_radial_tangential_axial() {
        let t = Transform {
            nset: "N".into(),
            kind: TransformKind::Cylindrical,
            a: [0.0, 0.0, 0.0],
            b: [0.0, 0.0, 1.0],
        };
        let r = rotation(&t, [0.0, 2.0, 5.0]).unwrap();
        let col = |k: usize| [r[0][k], r[1][k], r[2][k]];
        assert!(dot(sub(col(0), [0.0, 1.0, 0.0]), sub(col(0), [0.0, 1.0, 0.0])) < 1e-24);
        assert!(dot(sub(col(1), [-1.0, 0.0, 0.0]), sub(col(1), [-1.0, 0.0, 0.0])) < 1e-24);
        assert!(dot(sub(col(2), [0.0, 0.0, 1.0]), sub(col(2), [0.0, 0.0, 1.0])) < 1e-24);
    }
}
