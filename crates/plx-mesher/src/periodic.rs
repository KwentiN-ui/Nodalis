//! Matching meshes on the cut faces of a cyclic symmetric sector ([`PeriodicFaces`]).
//!
//! A cyclic symmetry ties every node of the slave cut face to the master face it lands on
//! when rotated back by one sector. Meshed one after the other, the two cut faces get nodes
//! that miss each other by a fraction of an element, and the tie then passes the forces of
//! the quadratic faces on unevenly: the stresses in the first row of elements show stripes.
//! Gmsh copies the mesh of the master side on to the slave side instead (`setPeriodic`), so
//! that every slave node lies on a master node.
//!
//! Which entities are copies of each other is found geometrically: an entity that the
//! rotation of a cyclic symmetry takes on to another one, with its centre, size and corners.
//! That also finds the cut faces' edges among the faces the sweeps mesh, which are copies of
//! the part's faces.

use std::collections::BTreeSet;

use plx_model::PeriodicFaces;

use crate::gmsh::{Gmsh, GmshError};

/// A rotation about an axis through `origin`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Rotation {
    origin: [f64; 3],
    matrix: [[f64; 3]; 3],
}

impl Rotation {
    /// The rotation by `angle` about the unit vector `axis` (Rodrigues).
    fn new(origin: [f64; 3], axis: [f64; 3], angle: f64) -> Self {
        let (s, c) = angle.sin_cos();
        let [x, y, z] = axis;
        let t = 1.0 - c;
        Self {
            origin,
            matrix: [
                [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
            ],
        }
    }

    fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let d = [
            p[0] - self.origin[0],
            p[1] - self.origin[1],
            p[2] - self.origin[2],
        ];
        let m = &self.matrix;
        std::array::from_fn(|i| m[i][0] * d[0] + m[i][1] * d[1] + m[i][2] * d[2] + self.origin[i])
    }

    /// The affine transformation as Gmsh takes it: `4 × 4`, row by row.
    fn transform(&self) -> [f64; 16] {
        let m = &self.matrix;
        let o = self.origin;
        let mut out = [0.0; 16];
        for i in 0..3 {
            out[4 * i..4 * i + 3].copy_from_slice(&m[i]);
            out[4 * i + 3] = o[i] - (m[i][0] * o[0] + m[i][1] * o[1] + m[i][2] * o[2]);
        }
        out[15] = 1.0;
        out
    }
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// The rotations that take the master faces of each cyclic symmetry on to its slave faces,
/// for the symmetries with faces of both sides in Gmsh's model. Symmetries whose faces do not
/// close the full circle are left out: the solver input reports them.
pub(crate) fn rotations(
    gmsh: &Gmsh,
    periodic: &[PeriodicFaces],
) -> Result<Vec<Rotation>, GmshError> {
    if periodic.is_empty() {
        return Ok(Vec::new());
    }
    let faces: BTreeSet<i32> = gmsh.entities(2)?.into_iter().collect();
    // The centre of the faces of one side that the model has, weighted by their areas.
    let centre = |tags: &[i32]| -> Result<Option<[f64; 3]>, GmshError> {
        let mut sum = [0.0; 3];
        let mut total = 0.0;
        for &tag in tags.iter().filter(|t| faces.contains(t)) {
            let area = gmsh.mass(2, tag)?;
            let c = gmsh.center_of_mass(2, tag)?;
            for d in 0..3 {
                sum[d] += area * c[d];
            }
            total += area;
        }
        Ok((total > 0.0).then(|| sum.map(|v| v / total)))
    };
    let mut out = Vec::new();
    for p in periodic {
        let (Some(master), Some(slave)) = (centre(&p.master)?, centre(&p.slave)?) else {
            continue;
        };
        let Ok(sectors) = p.cyclic.sectors(&[master], &[slave]) else {
            continue;
        };
        let d = p.cyclic.axis_direction;
        let length = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if length.is_nan() || length <= 0.0 {
            continue;
        }
        let axis = d.map(|v| v / length);
        let angle = std::f64::consts::TAU / f64::from(sectors);
        // The way round that takes the master side on to the slave side.
        let rotation = [angle, -angle]
            .map(|a| Rotation::new(p.cyclic.axis_point, axis, a))
            .into_iter()
            .min_by(|a, b| {
                distance(a.apply(master), slave).total_cmp(&distance(b.apply(master), slave))
            });
        if let Some(rotation) = rotation
            && !out.contains(&rotation)
        {
            out.push(rotation);
        }
    }
    Ok(out)
}

/// What an entity is recognised by: its centre, its length or area and its corners.
struct Shape {
    centre: [f64; 3],
    mass: f64,
    corners: Vec<[f64; 3]>,
}

fn shape(gmsh: &Gmsh, dim: i32, tag: i32) -> Result<Shape, GmshError> {
    let mut points = BTreeSet::new();
    if dim == 1 {
        points.extend(gmsh.adjacencies(1, tag)?.1);
    } else {
        for curve in gmsh.adjacencies(dim, tag)?.1 {
            points.extend(gmsh.adjacencies(1, curve)?.1);
        }
    }
    Ok(Shape {
        centre: gmsh.center_of_mass(dim, tag)?,
        mass: gmsh.mass(dim, tag)?,
        corners: (points.into_iter())
            .map(|p| gmsh.point(p))
            .collect::<Result<_, _>>()?,
    })
}

/// Makes the meshes of the entities among `candidates` (curves or faces, of dimension `dim`)
/// that one of the rotations takes on to another of them match, before they are meshed: the
/// one it lands on gets a copy of the mesh of the other. Returns the number of such pairs.
pub(crate) fn match_rotated(
    gmsh: &Gmsh,
    dim: i32,
    candidates: &[i32],
    rotations: &[Rotation],
) -> Result<usize, GmshError> {
    if rotations.is_empty() || candidates.len() < 2 {
        return Ok(0);
    }
    let (lo, hi) = gmsh.bounding_box()?;
    let tolerance = 1e-6 * distance(lo, hi).max(f64::MIN_POSITIVE);
    let shapes: Vec<Shape> = (candidates.iter())
        .map(|&tag| shape(gmsh, dim, tag))
        .collect::<Result<_, _>>()?;
    let lands_on = |rotation: &Rotation, a: &Shape, b: &Shape| -> bool {
        distance(rotation.apply(a.centre), b.centre) <= tolerance
            && (a.mass - b.mass).abs() <= 1e-6 * a.mass.abs().max(b.mass.abs())
            && a.corners.len() == b.corners.len()
            && (a.corners.iter()).all(|&c| {
                let c = rotation.apply(c);
                b.corners.iter().any(|&d| distance(c, d) <= tolerance)
            })
    };
    // An entity is either a master or a slave, so that no mesh is copied from a copy.
    let mut taken = BTreeSet::new();
    let mut pairs = 0;
    for rotation in rotations {
        let (mut slaves, mut masters) = (Vec::new(), Vec::new());
        for (i, a) in shapes.iter().enumerate() {
            if taken.contains(&i) {
                continue;
            }
            let found = (shapes.iter().enumerate())
                .find(|&(j, b)| j != i && !taken.contains(&j) && lands_on(rotation, a, b));
            if let Some((j, _)) = found {
                taken.insert(i);
                taken.insert(j);
                masters.push(candidates[i]);
                slaves.push(candidates[j]);
            }
        }
        if !slaves.is_empty() {
            gmsh.set_periodic(dim, &slaves, &masters, &rotation.transform())?;
            pairs += slaves.len();
        }
    }
    Ok(pairs)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_transform_rotates_about_the_axis_through_the_origin() {
        let r = Rotation::new(
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            std::f64::consts::FRAC_PI_2,
        );
        let p = r.apply([2.0, 0.0, 5.0]);
        assert!(distance(p, [1.0, 1.0, 5.0]) < 1e-12, "{p:?}");
        let t = r.transform();
        let q: [f64; 3] = std::array::from_fn(|i| {
            t[4 * i] * 2.0 + t[4 * i + 1] * 0.0 + t[4 * i + 2] * 5.0 + t[4 * i + 3]
        });
        assert!(distance(q, p) < 1e-12, "{q:?}");
        assert_eq!(t[12..], [0.0, 0.0, 0.0, 1.0]);
    }
}
