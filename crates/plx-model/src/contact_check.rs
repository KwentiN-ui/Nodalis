//! The contact check before solving, Ansys' contact tool: how the slave side of each contact
//! region lies to its master side in the mesh as it is, whether it touches (closed), is
//! near or far apart (open), and how large the gaps and overlaps are. It tells a bonded
//! region that ties nothing, or a frictional one that starts open and lets a part fly
//! away, before the solver runs.

use std::collections::{BTreeMap, HashMap};

use plx_mesh::{ElementId, FeMesh, NodeId};

use crate::{ContactRegion, FeModel, Region, Resolved};

/// How the sides of a contact region lie to each other before the analysis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactStatus {
    /// Every slave node touches the master side, within the tolerance.
    Closed,
    /// Some slave nodes touch, others are apart.
    PartlyClosed,
    /// No slave node touches, but some are within ten tolerances of the master side.
    NearOpen,
    /// The sides are far apart.
    FarOpen,
}

impl ContactStatus {
    pub fn name(self) -> &'static str {
        match self {
            Self::Closed => "Closed",
            Self::PartlyClosed => "Partly closed",
            Self::NearOpen => "Near open",
            Self::FarOpen => "Far open",
        }
    }
}

/// The contact check of one region.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactCheck {
    pub status: ContactStatus,
    /// Nodes of the slave side, and those touching the master side.
    pub slave_nodes: usize,
    pub touching: usize,
    /// Distance within which a node touches: the tie tolerance or adjustment distance of
    /// the region, else a quarter of its mesh size.
    pub tolerance: f64,
    /// The smallest gap of the nodes that do not touch.
    pub min_gap: Option<f64>,
    /// The largest gap of the nodes that touch, and the deepest overlap.
    pub max_gap: Option<f64>,
    pub max_penetration: Option<f64>,
    /// The elements that moving the slave nodes on to the master side at the start (the
    /// adjustment of a bonded region or contact) turns inside out, as the solver finds them.
    pub folded: Vec<ElementId>,
}

impl ContactCheck {
    /// Share of the slave nodes that touch, 0 to 1.
    pub fn touching_share(&self) -> f64 {
        if self.slave_nodes == 0 {
            0.0
        } else {
            self.touching as f64 / self.slave_nodes as f64
        }
    }
}

type Point = [f64; 3];

fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn length(a: Point) -> f64 {
    dot(a, a).sqrt()
}

/// A piece of the master side: a triangle, or a segment for the edges of 2D elements and
/// shells. `normal` points out of the material where it is known.
struct Piece {
    points: Vec<Point>,
    normal: Option<Point>,
    min: Point,
    max: Point,
}

/// The closest point to `p` on the triangle `a b c` (Ericson, Real-Time Collision
/// Detection, 5.1.5).
pub(crate) fn closest_on_triangle(p: Point, a: Point, b: Point, c: Point) -> Point {
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return [a[0] + v * ab[0], a[1] + v * ab[1], a[2] + v * ab[2]];
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return [a[0] + w * ac[0], a[1] + w * ac[1], a[2] + w * ac[2]];
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let bc = sub(c, b);
        return [b[0] + w * bc[0], b[1] + w * bc[1], b[2] + w * bc[2]];
    }
    let denom = 1.0 / (va + vb + vc);
    let (v, w) = (vb * denom, vc * denom);
    [
        a[0] + ab[0] * v + ac[0] * w,
        a[1] + ab[1] * v + ac[1] * w,
        a[2] + ab[2] * v + ac[2] * w,
    ]
}

fn closest_on_segment(p: Point, a: Point, b: Point) -> Point {
    let ab = sub(b, a);
    let t = (dot(sub(p, a), ab) / dot(ab, ab).max(f64::MIN_POSITIVE)).clamp(0.0, 1.0);
    [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]]
}

impl Piece {
    /// Signed distance of `p`: positive outside the material (a gap), negative inside (an
    /// overlap); without a known normal the distance.
    /// With the closest point of the piece.
    fn closest(&self, p: Point) -> (f64, Point) {
        let q = match self.points[..] {
            [a, b] => closest_on_segment(p, a, b),
            [a, b, c] => closest_on_triangle(p, a, b, c),
            _ => return (f64::INFINITY, p),
        };
        let d = sub(p, q);
        let distance = length(d);
        match self.normal {
            Some(n) if dot(d, n) < 0.0 => (-distance, q),
            _ => (distance, q),
        }
    }
}

/// The master side cut into pieces, with a grid of cells for finding the pieces near a
/// point.
struct Master {
    pieces: Vec<Piece>,
    cell: f64,
    grid: HashMap<[i64; 3], Vec<usize>>,
    /// The box around all pieces.
    min: Point,
    max: Point,
}

/// Distance of `p` to the box from `min` to `max`, 0 inside.
fn box_distance(p: Point, min: Point, max: Point) -> f64 {
    let d = [0, 1, 2].map(|k| (min[k] - p[k]).max(p[k] - max[k]).max(0.0));
    length(d)
}

impl Master {
    fn new(mesh: &FeMesh, faces: &[(ElementId, u8)], reach: f64) -> Self {
        let mut pieces = Vec::new();
        let mut size = 0.0;
        for &(element_id, face) in faces {
            let Some(element) = mesh.element(element_id) else {
                continue;
            };
            let Some(topology) = element.faces().get(usize::from(face).saturating_sub(1)) else {
                continue;
            };
            let corners: Vec<Point> = (topology.corners.iter())
                .filter_map(|&local| mesh.node(*element.nodes.get(local)?))
                .collect();
            if corners.len() != topology.corners.len() || corners.len() < 2 {
                continue;
            }
            // Outward: away from the centre of the solid element; shells and edges have no
            // inside.
            let solid = !element.is_shell() && !element.is_plane() && corners.len() >= 3;
            let normal = solid.then(|| {
                let mut n = [0.0; 3];
                for k in 0..corners.len() {
                    let (a, b) = (corners[k], corners[(k + 1) % corners.len()]);
                    n[0] += (a[1] - b[1]) * (a[2] + b[2]);
                    n[1] += (a[2] - b[2]) * (a[0] + b[0]);
                    n[2] += (a[0] - b[0]) * (a[1] + b[1]);
                }
                let centre = |points: &[Point]| {
                    let k = points.len() as f64;
                    [0, 1, 2].map(|d| points.iter().map(|p| p[d]).sum::<f64>() / k)
                };
                let nodes: Vec<Point> = (element.nodes.iter())
                    .filter_map(|&id| mesh.node(id))
                    .collect();
                if dot(n, sub(centre(&corners), centre(&nodes))) < 0.0 {
                    n = n.map(|v| -v);
                }
                n
            });
            let triangles: Vec<Vec<Point>> = if corners.len() == 2 {
                vec![corners.clone()]
            } else {
                (1..corners.len() - 1)
                    .map(|k| vec![corners[0], corners[k], corners[k + 1]])
                    .collect()
            };
            for points in triangles {
                let mut min = [f64::INFINITY; 3];
                let mut max = [f64::NEG_INFINITY; 3];
                for p in &points {
                    for d in 0..3 {
                        min[d] = min[d].min(p[d]);
                        max[d] = max[d].max(p[d]);
                    }
                }
                size += length(sub(max, min));
                pieces.push(Piece {
                    points,
                    normal,
                    min,
                    max,
                });
            }
        }
        // At least the reach, so that a search looks into the few cells around its point.
        let cell = (size / pieces.len().max(1) as f64).max(reach).max(1e-12);
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for piece in &pieces {
            for d in 0..3 {
                min[d] = min[d].min(piece.min[d]);
                max[d] = max[d].max(piece.max[d]);
            }
        }
        let mut master = Self {
            pieces,
            cell,
            grid: HashMap::new(),
            min,
            max,
        };
        for (i, piece) in master.pieces.iter().enumerate() {
            let (lo, hi) = (master.key(piece.min), master.key(piece.max));
            for x in lo[0]..=hi[0] {
                for y in lo[1]..=hi[1] {
                    for z in lo[2]..=hi[2] {
                        master.grid.entry([x, y, z]).or_default().push(i);
                    }
                }
            }
        }
        master
    }

    fn key(&self, p: Point) -> [i64; 3] {
        p.map(|v| (v / self.cell).floor() as i64)
    }

    /// The signed distance of `p` to the nearest piece within `radius`, if any, and the
    /// closest point of that piece.
    fn closest(&self, p: Point, radius: f64) -> Option<(f64, Point)> {
        if box_distance(p, self.min, self.max) > radius {
            return None;
        }
        let (lo, hi) = (
            self.key(p.map(|v| v - radius)),
            self.key(p.map(|v| v + radius)),
        );
        let mut best: Option<(f64, Point)> = None;
        for x in lo[0]..=hi[0] {
            for y in lo[1]..=hi[1] {
                for z in lo[2]..=hi[2] {
                    // A piece in several cells is met again, but its box is no nearer then.
                    for &i in self.grid.get(&[x, y, z]).into_iter().flatten() {
                        let piece = &self.pieces[i];
                        let bound = best.map_or(radius, |(b, _)| b.abs());
                        if box_distance(p, piece.min, piece.max) > bound {
                            continue;
                        }
                        let (d, q) = piece.closest(p);
                        if d.abs() <= radius && best.is_none_or(|(b, _)| d.abs() < b.abs()) {
                            best = Some((d, q));
                        }
                    }
                }
            }
        }
        best
    }
}

/// Nearer than this many tolerances counts as near open.
const NEAR: f64 = 10.0;

impl FeModel {
    /// The contact check of every contact region, in their order; `None` for a region
    /// whose sides have no element faces.
    pub fn check_contacts(&self, mesh: &FeMesh) -> Vec<Option<ContactCheck>> {
        let resolved = self.resolve_contacts(mesh);
        (self.contact_regions.iter().zip(&resolved.items))
            .map(|(region, resolved)| self.check_resolved(mesh, region, resolved))
            .collect()
    }

    /// The contact check of `region`, also one not in the model yet.
    pub fn check_contact(&self, region: &ContactRegion, mesh: &FeMesh) -> Option<ContactCheck> {
        self.check_resolved(mesh, region, &self.resolve_contact(region, mesh))
    }

    fn check_resolved(
        &self,
        mesh: &FeMesh,
        region: &ContactRegion,
        resolved: &Resolved,
    ) -> Option<ContactCheck> {
        let (master, slave) = if resolved.a_is_master {
            (&region.side_a, &region.side_b)
        } else {
            (&region.side_b, &region.side_a)
        };
        let tolerance =
            (resolved.tolerance()).or(resolved.mesh_size.map(|h| crate::TOLERANCE_FACTOR * h))?;
        // A rope touches with its surface, the contact radius off its axis.
        let offset = if resolved.node_to_surface {
            self.contact_radius(slave, mesh).unwrap_or(0.0)
        } else {
            0.0
        };
        // Rope nodes stay on their axis, the radius off the master.
        let adjust = resolved.adjusts() && offset == 0.0;
        check(mesh, master, slave, tolerance, offset, adjust)
    }
}

/// The check of the slave nodes against the master faces; `offset` is subtracted from each
/// distance, the radius of a rope whose axis the nodes are on. With `adjust` the slave nodes
/// within the tolerance are moved on to the master as the solver moves them, to find the
/// elements that turns inside out.
fn check(
    mesh: &FeMesh,
    master: &Region,
    slave: &Region,
    tolerance: f64,
    offset: f64,
    adjust: bool,
) -> Option<ContactCheck> {
    let faces = master.faces(mesh);
    let nodes = slave.nodes(mesh);
    if faces.is_empty() || nodes.is_empty() || tolerance <= 0.0 {
        return None;
    }
    let radius = NEAR * tolerance + offset;
    let master = Master::new(mesh, &faces, radius);
    let mut check = ContactCheck {
        status: ContactStatus::FarOpen,
        slave_nodes: nodes.len(),
        touching: 0,
        tolerance,
        min_gap: None,
        max_gap: None,
        max_penetration: None,
        folded: Vec::new(),
    };
    let mut near = false;
    let mut moved: BTreeMap<NodeId, Point> = BTreeMap::new();
    for (n, p) in nodes.iter().filter_map(|&n| Some((n, mesh.node(n)?))) {
        let Some((d, q)) = master.closest(p, radius).map(|(d, q)| (d - offset, q)) else {
            continue;
        };
        if d <= tolerance {
            check.touching += 1;
            if adjust {
                moved.insert(n, q);
            }
            if d < 0.0 {
                check.max_penetration = Some(check.max_penetration.map_or(-d, |m| m.max(-d)));
            } else {
                check.max_gap = Some(check.max_gap.map_or(d, |m| m.max(d)));
            }
        } else {
            near = true;
            check.min_gap = Some(check.min_gap.map_or(d, |m| m.min(d)));
        }
    }
    check.folded = folded(mesh, &moved);
    check.status = if check.touching == check.slave_nodes {
        ContactStatus::Closed
    } else if check.touching > 0 {
        ContactStatus::PartlyClosed
    } else if near {
        ContactStatus::NearOpen
    } else {
        ContactStatus::FarOpen
    };
    Some(check)
}

/// The elements sound in the mesh that the nodes moved to `moved` turn inside out or fold.
fn folded(mesh: &FeMesh, moved: &BTreeMap<NodeId, Point>) -> Vec<ElementId> {
    if moved.is_empty() {
        return Vec::new();
    }
    (mesh.elements().iter())
        .filter(|e| e.nodes.iter().any(|n| moved.contains_key(n)))
        .filter(|e| e.min_jacobian(mesh).is_some_and(|j| j > 0.0))
        .filter(|e| {
            let at = |n: NodeId| moved.get(&n).copied().or_else(|| mesh.node(n));
            e.min_jacobian_at(at).is_some_and(|j| j <= 0.0)
        })
        .map(|e| e.id)
        .collect()
}

#[cfg(test)]
mod tests {
    use plx_mesh::{Element, ElementShape, Part};

    use super::*;
    use crate::{ContactBehavior, InterfaceTreatment};

    /// Two unit cubes of C3D8 along x, the second starting `overlap` before the first ends,
    /// and a bonded region between the face x = 1 of the first and the near face of the
    /// second.
    fn overlapping_cubes(overlap: f64) -> (FeMesh, ContactRegion) {
        let mut mesh = FeMesh::default();
        let corners = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ];
        for cube in 0..2u32 {
            let shift = cube as f64 * (1.0 - overlap);
            for (k, c) in corners.iter().enumerate() {
                mesh.set_node(cube * 8 + k as u32 + 1, [c[0] + shift, c[1], c[2]]);
            }
            mesh.add_element(Element {
                id: cube + 1,
                type_name: "C3D8".into(),
                shape: ElementShape::Hex8,
                nodes: (1..=8).map(|n| cube * 8 + n).collect(),
            })
            .unwrap();
            mesh.parts
                .push(Part::new(format!("CUBE-{}", cube + 1), vec![cube + 1]));
        }
        // The face of each cube whose corners all lie at x.
        let face = |element: u32, x: f64| -> (ElementId, u8) {
            let e = mesh.element(element).unwrap();
            let index = (e.faces().iter())
                .position(|f| {
                    (f.corners.iter())
                        .all(|&c| (mesh.node(e.nodes[c]).unwrap()[0] - x).abs() < 1e-9)
                })
                .unwrap();
            (element, index as u8 + 1)
        };
        let mut region = ContactRegion::new("BONDED", ContactBehavior::Bonded);
        region.side_a = Region::Faces(vec![face(1, 1.0)]);
        region.side_b = Region::Faces(vec![face(2, 1.0 - overlap)]);
        region.settings.tolerance = Some(1.5);
        (mesh, region)
    }

    #[test]
    fn an_adjustment_across_more_than_the_elements_folds_them() {
        let (mesh, region) = overlapping_cubes(1.2);
        let model = FeModel::default();
        let check = model.check_contact(&region, &mesh).unwrap();
        assert_eq!(check.folded.len(), 1);
    }

    #[test]
    fn a_small_adjustment_or_none_folds_nothing() {
        let model = FeModel::default();
        let (mesh, region) = overlapping_cubes(0.1);
        assert!(
            model
                .check_contact(&region, &mesh)
                .unwrap()
                .folded
                .is_empty()
        );
        let (mesh, mut region) = overlapping_cubes(1.2);
        region.settings.interface = Some(InterfaceTreatment::AsIs);
        assert!(
            model
                .check_contact(&region, &mesh)
                .unwrap()
                .folded
                .is_empty()
        );
    }
}
