//! The plane of a symmetry boundary condition, found from the nodes the user picked.
//!
//! A symmetry boundary condition stores only its region and, optionally, a normal the user
//! entered; the plane is fitted to the nodes of the region whenever it is needed, so it
//! follows the mesh after remeshing. The input file holds the plane's normal displacement
//! and the rotations about the two in-plane axes at zero.

use std::fmt;

use std::collections::{BTreeSet, HashSet};

use plx_mesh::{ElementFamily, ElementId, FeMesh};

use crate::{BoundaryKind, FeModel, ModelSpace, Region, SectionKind};

/// How far the nodes may lie off the plane, relative to the extent of the region, before the
/// region no longer counts as planar.
const PLANAR_TOLERANCE: f64 = 1e-4;
/// Below this a component of the unit normal counts as zero, so the plane is a coordinate
/// plane and needs no local coordinate system.
const AXIS_TOLERANCE: f64 = 1e-7;

/// The plane a symmetry boundary condition holds its nodes in.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SymmetryPlane {
    /// Centroid of the nodes of the region.
    pub point: [f64; 3],
    /// Unit normal; its largest component is positive.
    pub normal: [f64; 3],
    /// Largest distance of a node from the plane.
    pub deviation: f64,
    /// Extent of the region (diagonal of its bounding box).
    pub size: f64,
}

/// Why a region has no symmetry plane.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SymmetryError {
    /// The region has no nodes of the mesh, e.g. a reference point or an empty selection.
    NoNodes,
    /// A single node, or nodes on a line without shells: the plane is not fixed by them.
    Undetermined,
    /// The normal the user entered is the zero vector.
    ZeroNormal,
    /// The normal the user entered points out of the plane of a 2D model.
    NormalOutOfPlane,
}

impl fmt::Display for SymmetryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            SymmetryError::NoNodes => "the region has no nodes of the mesh",
            SymmetryError::Undetermined => {
                "the selection does not determine a plane; select a face or enter the normal"
            }
            SymmetryError::ZeroNormal => "the normal must not be the zero vector",
            SymmetryError::NormalOutOfPlane => "the normal of a 2D model must lie in the x-y plane",
        })
    }
}

impl SymmetryPlane {
    /// Whether all nodes lie on the plane.
    pub fn is_planar(&self) -> bool {
        self.deviation <= PLANAR_TOLERANCE * self.size
    }

    /// The coordinate axis (0 to 2) the normal points along, if it is a coordinate plane.
    pub fn axis(&self) -> Option<usize> {
        (0..3).find(|&k| self.normal[k] == 1.0)
    }

    /// Two unit vectors in the plane that form a right-handed system with the normal, the
    /// local y and z axes of the plane. In a 2D model the first lies in the x-y plane.
    pub fn in_plane_axes(&self) -> ([f64; 3], [f64; 3]) {
        let n = self.normal;
        // The coordinate axis least aligned with the normal gives a stable first tangent;
        // the z axis is preferred so that the tangent of a 2D plane stays in x-y.
        let helper = if n[2].abs() < 0.9 {
            [0.0, 0.0, 1.0]
        } else {
            [1.0, 0.0, 0.0]
        };
        let t1 = normalized(cross(helper, n)).unwrap_or([0.0, 1.0, 0.0]);
        let t2 = cross(n, t1);
        (t1, t2)
    }

    /// Why the plane makes no sense as a symmetry plane although the solver would accept it.
    pub fn warning(&self, space: ModelSpace) -> Option<String> {
        if !self.is_planar() {
            return Some(format!(
                "the nodes are not on one plane: up to {} off it ({:.2} % of the region's extent)",
                short(self.deviation),
                100.0 * self.deviation / self.size.max(f64::MIN_POSITIVE)
            ));
        }
        if space == ModelSpace::Axisymmetric && self.axis() != Some(1) {
            return Some(
                "in an axisymmetric model only a plane perpendicular to the axis (normal along y) \
                 is a symmetry plane"
                    .into(),
            );
        }
        None
    }
}

/// Degrees of freedom (0-based, 3 to 5 the rotations) a symmetry plane normal to the
/// coordinate axis `axis` holds at zero, of `dofs` a node has: the displacement along the
/// axis and the rotations about the other two axes.
pub fn symmetry_dofs(axis: usize, dofs: usize) -> Vec<usize> {
    let mut out = vec![axis];
    if dofs == 6 {
        out.extend((0..3).filter(|&k| k != axis).map(|k| k + 3));
    }
    out.retain(|&d| d < dofs);
    out
}

/// The symmetry plane of the nodes of `region`: with `normal` the plane through their
/// centroid with that normal, otherwise the plane fitted to them. Nodes on a line, such as
/// the edge of a shell, take the plane through the line that is perpendicular to the shells;
/// in a 2D model the plane through the line perpendicular to the x-y plane.
pub fn symmetry_plane(
    mesh: &FeMesh,
    region: &Region,
    space: ModelSpace,
    normal: Option<[f64; 3]>,
) -> Result<SymmetryPlane, SymmetryError> {
    let nodes = region.nodes(mesh);
    let mut points: Vec<[f64; 3]> = nodes.iter().filter_map(|&n| mesh.node(n)).collect();
    if points.is_empty() {
        return Err(SymmetryError::NoNodes);
    }
    if space.is_2d() {
        for p in &mut points {
            p[2] = 0.0;
        }
    }
    let count = points.len() as f64;
    let point: [f64; 3] = std::array::from_fn(|k| points.iter().map(|p| p[k]).sum::<f64>() / count);
    let size = {
        let lo: [f64; 3] =
            std::array::from_fn(|k| points.iter().map(|p| p[k]).fold(f64::INFINITY, f64::min));
        let hi: [f64; 3] = std::array::from_fn(|k| {
            (points.iter().map(|p| p[k])).fold(f64::NEG_INFINITY, f64::max)
        });
        (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt()
    };
    let normal = match normal {
        Some(n) => {
            let n = normalized(n).ok_or(SymmetryError::ZeroNormal)?;
            if space.is_2d() && n[2].abs() > AXIS_TOLERANCE {
                return Err(SymmetryError::NormalOutOfPlane);
            }
            n
        }
        None => fitted_normal(mesh, region, space, &points, point, size)?,
    };
    let normal = canonical(normal);
    let deviation = (points.iter())
        .map(|p| dot(sub(*p, point), normal).abs())
        .fold(0.0, f64::max);
    Ok(SymmetryPlane {
        point,
        normal,
        deviation,
        size,
    })
}

/// The beam elements a symmetry region holds on its plane.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BeamsOnPlane {
    /// Line elements of beam sections with all their nodes in the region.
    pub total: usize,
    /// Those with a rectangular, circular, pipe or box profile, which the solver expands
    /// into solids: holding the expanded section on the plane at every node locks its
    /// bending, and it cannot be halved.
    pub expanded: usize,
}

impl FeModel {
    /// The beam elements whose nodes all lie in `nodes`, each with whether its profile is
    /// one the solver expands into solids.
    fn beams_within(
        &self,
        mesh: &FeMesh,
        nodes: &HashSet<u32>,
    ) -> impl Iterator<Item = (ElementId, bool)> {
        let mut out = Vec::new();
        if !nodes.is_empty() {
            for section in &self.sections {
                let SectionKind::Beam(beam) = &section.kind else {
                    continue;
                };
                let expanded = (beam.cross_section)
                    .and_then(|c| self.cross_section(c))
                    .is_some_and(|c| c.profile.is_native());
                for id in section.region.elements(mesh) {
                    let Some(element) = mesh.element(id) else {
                        continue;
                    };
                    if element.shape.family() == ElementFamily::Line
                        && element.nodes.iter().all(|n| nodes.contains(n))
                    {
                        out.push((id, expanded));
                    }
                }
            }
        }
        out.into_iter()
    }

    /// The beams a symmetry boundary condition on `region` holds on its plane.
    pub fn beams_on_plane(&self, mesh: &FeMesh, region: &Region) -> BeamsOnPlane {
        let nodes: HashSet<u32> = region.nodes(mesh).into_iter().collect();
        let mut out = BeamsOnPlane::default();
        for (_, expanded) in self.beams_within(mesh, &nodes) {
            out.total += 1;
            out.expanded += usize::from(expanded);
        }
        out
    }

    /// The beams a symmetry plane cuts in half: the line elements of beam sections whose
    /// nodes all lie in the region of an active symmetry boundary condition that halves
    /// beams (`BoundaryKind::Symmetry::halve_beams`), in any mechanical step. A beam on the
    /// plane of a half model has half its section in the model: the solver gets half of it.
    /// Beams the solver expands into solids are left out; they cannot be halved.
    pub fn halved_beams(&self, mesh: &FeMesh) -> BTreeSet<ElementId> {
        let mut on_plane: HashSet<u32> = HashSet::new();
        for step in self.steps.iter().filter(|s| s.kind.is_mechanical()) {
            for bc in step.boundary_conditions.iter().filter(|b| b.active) {
                if let BoundaryKind::Symmetry {
                    halve_beams: true, ..
                } = bc.kind
                {
                    on_plane.extend(bc.region.nodes(mesh));
                }
            }
        }
        (self.beams_within(mesh, &on_plane))
            .filter(|&(_, expanded)| !expanded)
            .map(|(id, _)| id)
            .collect()
    }
}

/// Normal of the plane that fits the points best.
fn fitted_normal(
    mesh: &FeMesh,
    region: &Region,
    space: ModelSpace,
    points: &[[f64; 3]],
    center: [f64; 3],
    size: f64,
) -> Result<[f64; 3], SymmetryError> {
    if size == 0.0 {
        return Err(SymmetryError::Undetermined);
    }
    let mut covariance = [[0.0; 3]; 3];
    for p in points {
        let d = sub(*p, center);
        for a in 0..3 {
            for b in 0..3 {
                covariance[a][b] += d[a] * d[b];
            }
        }
    }
    let (values, vectors) = eigen(covariance);
    // Sorted from the largest spread to the smallest.
    let mut order = [0, 1, 2];
    order.sort_by(|&a, &b| values[b].total_cmp(&values[a]));
    let spread = |i: usize| (values[order[i]].max(0.0) / points.len() as f64).sqrt();
    let collinear = spread(1) <= 1e-6 * size;
    if !collinear && !space.is_2d() {
        return Ok(vectors[order[2]]);
    }
    // On a line, or a 2D model: the plane through the main direction perpendicular to the
    // shells or to the x-y plane.
    let line = vectors[order[0]];
    let across = if space.is_2d() {
        Some([0.0, 0.0, 1.0])
    } else {
        shell_normal(mesh, region)
    };
    across
        .and_then(|across| normalized(cross(line, across)))
        .ok_or(SymmetryError::Undetermined)
}

/// Mean normal of the shell elements at the nodes of the region.
fn shell_normal(mesh: &FeMesh, region: &Region) -> Option<[f64; 3]> {
    let nodes: std::collections::HashSet<_> = region.nodes(mesh).into_iter().collect();
    let mut sum: Option<[f64; 3]> = None;
    for element in mesh.elements() {
        if element.shape.family() != ElementFamily::Surface
            || element.is_plane()
            || !element.nodes.iter().any(|n| nodes.contains(n))
        {
            continue;
        }
        let corners: Vec<[f64; 3]> = (element.shape.faces().first())
            .map(|face| face.corners)
            .unwrap_or_default()
            .iter()
            .filter_map(|&i| element.nodes.get(i).and_then(|&n| mesh.node(n)))
            .collect();
        let normal = match corners.as_slice() {
            [a, b, c] => cross(sub(*b, *a), sub(*c, *a)),
            [a, b, c, d] => cross(sub(*c, *a), sub(*d, *b)),
            _ => continue,
        };
        let Some(normal) = normalized(normal) else {
            continue;
        };
        sum = Some(match sum {
            None => normal,
            // Shells may be oriented either way; only the line through them counts.
            Some(s) => {
                let sign = if dot(s, normal) < 0.0 { -1.0 } else { 1.0 };
                std::array::from_fn(|k| s[k] + sign * normal[k])
            }
        });
    }
    sum.and_then(normalized)
}

/// The normal with its largest component positive, snapped onto a coordinate axis when it
/// is one.
fn canonical(n: [f64; 3]) -> [f64; 3] {
    let largest = (0..3)
        .max_by(|&a, &b| n[a].abs().total_cmp(&n[b].abs()))
        .unwrap_or(0);
    if (0..3).all(|k| k == largest || n[k].abs() <= AXIS_TOLERANCE) {
        return std::array::from_fn(|k| if k == largest { 1.0 } else { 0.0 });
    }
    let sign = n[largest].signum();
    n.map(|c| sign * c)
}

/// Eigenvalues and unit eigenvectors of a symmetric 3x3 matrix (Jacobi rotations).
fn eigen(mut a: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let off = a[0][1].powi(2) + a[0][2].powi(2) + a[1][2].powi(2);
        let scale = (0..3).map(|i| a[i][i].powi(2)).sum::<f64>();
        if off <= 1e-30 * scale.max(f64::MIN_POSITIVE) {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q] == 0.0 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for row in &mut a {
                let (akp, akq) = (row[p], row[q]);
                row[p] = c * akp - s * akq;
                row[q] = s * akp + c * akq;
            }
            let (row_p, row_q) = (a[p], a[q]);
            a[p] = std::array::from_fn(|k| c * row_p[k] - s * row_q[k]);
            a[q] = std::array::from_fn(|k| s * row_p[k] + c * row_q[k]);
            for row in &mut v {
                let (vp, vq) = (row[p], row[q]);
                row[p] = c * vp - s * vq;
                row[q] = s * vp + c * vq;
            }
        }
    }
    let values = [a[0][0], a[1][1], a[2][2]];
    let vectors = std::array::from_fn(|j| [v[0][j], v[1][j], v[2][j]]);
    (values, vectors)
}

/// A length for a message, with four significant digits.
fn short(value: f64) -> String {
    format!("{value:.4}")
        .trim_end_matches('0')
        .trim_end_matches('.')
        .to_string()
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|k| a[k] - b[k])
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|k| a[k] * b[k]).sum()
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalized(a: [f64; 3]) -> Option<[f64; 3]> {
    let length = dot(a, a).sqrt();
    (length > 1e-12 && length.is_finite()).then(|| a.map(|c| c / length))
}

#[cfg(test)]
mod tests {
    use plx_mesh::{Element, ElementShape};

    use super::*;

    fn mesh(points: &[[f64; 3]]) -> FeMesh {
        let mut mesh = FeMesh::default();
        for (i, p) in points.iter().enumerate() {
            mesh.set_node(i as u32 + 1, *p);
        }
        mesh
    }

    fn all(points: &[[f64; 3]]) -> Region {
        Region::Nodes((1..=points.len() as u32).collect())
    }

    #[test]
    fn a_coordinate_plane_is_snapped_onto_its_axis() {
        let points = [
            [2.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [2.0, 1.0, 1.0],
            [2.0, 0.0, 3.0],
        ];
        let plane =
            symmetry_plane(&mesh(&points), &all(&points), ModelSpace::ThreeD, None).unwrap();
        assert_eq!(plane.normal, [1.0, 0.0, 0.0]);
        assert_eq!(plane.axis(), Some(0));
        assert!(plane.is_planar());
        assert_eq!(plane.warning(ModelSpace::ThreeD), None);
    }

    #[test]
    fn a_skewed_plane_is_fitted() {
        // The plane x + y = 1.
        let points = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 2.0],
            [0.5, 0.5, 1.0],
        ];
        let plane =
            symmetry_plane(&mesh(&points), &all(&points), ModelSpace::ThreeD, None).unwrap();
        let h = 0.5f64.sqrt();
        for (a, b) in plane.normal.iter().zip([h, h, 0.0]) {
            assert!((a - b).abs() < 1e-12, "{:?}", plane.normal);
        }
        assert_eq!(plane.axis(), None);
        assert!(plane.is_planar());
        let (t1, t2) = plane.in_plane_axes();
        assert!(dot(t1, plane.normal).abs() < 1e-12 && dot(t2, plane.normal).abs() < 1e-12);
        assert!((dot(cross(plane.normal, t1), t2) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn nodes_off_the_plane_give_a_warning() {
        let points = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.5, 0.5, 0.2],
        ];
        let plane =
            symmetry_plane(&mesh(&points), &all(&points), ModelSpace::ThreeD, None).unwrap();
        assert!(!plane.is_planar());
        assert!(
            plane
                .warning(ModelSpace::ThreeD)
                .unwrap()
                .contains("not on one plane")
        );
    }

    #[test]
    fn an_entered_normal_is_checked_against_the_nodes() {
        let points = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let (m, r) = (mesh(&points), all(&points));
        let along = symmetry_plane(&m, &r, ModelSpace::ThreeD, Some([0.0, 0.0, -2.0])).unwrap();
        assert_eq!(along.normal, [0.0, 0.0, 1.0]);
        assert!(along.is_planar());
        let across = symmetry_plane(&m, &r, ModelSpace::ThreeD, Some([1.0, 0.0, 0.0])).unwrap();
        assert!(!across.is_planar());
        assert_eq!(
            symmetry_plane(&m, &r, ModelSpace::ThreeD, Some([0.0; 3])),
            Err(SymmetryError::ZeroNormal)
        );
    }

    #[test]
    fn a_line_of_a_2d_model_is_a_plane_across_x_y() {
        let points = [[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 2.0, 0.0]];
        let (m, r) = (mesh(&points), all(&points));
        let plane = symmetry_plane(&m, &r, ModelSpace::PlaneStress, None).unwrap();
        let h = 0.5f64.sqrt();
        assert!(
            (dot(plane.normal, [h, -h, 0.0]).abs() - 1.0).abs() < 1e-12,
            "{plane:?}"
        );
        assert!(
            plane
                .warning(ModelSpace::Axisymmetric)
                .unwrap()
                .contains("axisymmetric")
        );
        assert_eq!(
            symmetry_plane(&m, &r, ModelSpace::PlaneStress, Some([0.0, 0.0, 1.0])),
            Err(SymmetryError::NormalOutOfPlane)
        );
    }

    #[test]
    fn a_shell_edge_takes_the_plane_perpendicular_to_the_shells() {
        // Shells in the x-y plane; the edge x = 0 lies along y.
        let points = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        let mut m = mesh(&points);
        m.add_element(Element {
            id: 1,
            type_name: "S4".into(),
            shape: ElementShape::Quad4,
            nodes: vec![1, 2, 3, 4],
        })
        .unwrap();
        let edge = Region::Nodes(vec![1, 4]);
        let plane = symmetry_plane(&m, &edge, ModelSpace::ThreeD, None).unwrap();
        assert_eq!(plane.normal, [1.0, 0.0, 0.0]);
        // Without shells the line leaves the plane open.
        let bare = mesh(&points);
        assert_eq!(
            symmetry_plane(&bare, &edge, ModelSpace::ThreeD, None),
            Err(SymmetryError::Undetermined)
        );
        assert_eq!(
            symmetry_plane(&bare, &Region::Nodes(vec![9]), ModelSpace::ThreeD, None),
            Err(SymmetryError::NoNodes)
        );
    }
}
