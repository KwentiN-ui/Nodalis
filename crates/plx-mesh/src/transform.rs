//! Mesh tools of the Mesh menu that change an existing mesh: transforming parts
//! (translate, rotate, mirror, scale), merging coincident nodes and renumbering.

use std::collections::{BTreeMap, BTreeSet};

use crate::cad::CadMap;
use crate::element::ElementShape;
use crate::mesh::{Element, ElementId, FeMesh, NodeId, SurfaceDefinition};

/// A rigid or affine map of the node coordinates of a part.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum MeshTransform {
    Translate([f64; 3]),
    /// Right-handed rotation by `angle` degrees about the axis through `point`.
    Rotate {
        point: [f64; 3],
        axis: [f64; 3],
        angle: f64,
    },
    /// Reflection at the plane through `point` with the given normal.
    Mirror {
        point: [f64; 3],
        normal: [f64; 3],
    },
    /// Scaling about `center` by a factor per axis.
    Scale {
        center: [f64; 3],
        factors: [f64; 3],
    },
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalized(v: [f64; 3]) -> Option<[f64; 3]> {
    let length = dot(v, v).sqrt();
    (length > 0.0).then(|| v.map(|c| c / length))
}

impl MeshTransform {
    /// Why the transform cannot be applied, such as a zero axis.
    pub fn problem(&self) -> Option<&'static str> {
        match self {
            Self::Rotate { axis, .. } if normalized(*axis).is_none() => {
                Some("The axis must not be the zero vector.")
            }
            Self::Mirror { normal, .. } if normalized(*normal).is_none() => {
                Some("The normal must not be the zero vector.")
            }
            Self::Scale { factors, .. } if factors.contains(&0.0) => {
                Some("A scale factor must not be zero.")
            }
            _ => None,
        }
    }

    /// Whether the map turns elements inside out, so that their node order is inverted.
    pub fn inverts(&self) -> bool {
        match self {
            Self::Mirror { .. } => true,
            Self::Scale { factors, .. } => factors.iter().filter(|f| **f < 0.0).count() % 2 == 1,
            _ => false,
        }
    }

    /// The map that takes every image back to its point, so that a transform can be undone.
    pub fn inverse(&self) -> Self {
        match *self {
            Self::Translate(d) => Self::Translate(d.map(|c| -c)),
            Self::Rotate { point, axis, angle } => Self::Rotate {
                point,
                axis,
                angle: -angle,
            },
            Self::Mirror { .. } => *self,
            Self::Scale { center, factors } => Self::Scale {
                center,
                factors: factors.map(|f| if f == 0.0 { f } else { 1.0 / f }),
            },
        }
    }

    /// The image of a point.
    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        match *self {
            Self::Translate(d) => [p[0] + d[0], p[1] + d[1], p[2] + d[2]],
            Self::Rotate { point, axis, angle } => {
                let Some(n) = normalized(axis) else {
                    return p;
                };
                let r = [p[0] - point[0], p[1] - point[1], p[2] - point[2]];
                let (sin, cos) = angle.to_radians().sin_cos();
                // Rodrigues' formula.
                let along = dot(r, n);
                let c = cross(n, r);
                std::array::from_fn(|k| {
                    point[k] + r[k] * cos + c[k] * sin + n[k] * along * (1.0 - cos)
                })
            }
            Self::Mirror { point, normal } => {
                let Some(n) = normalized(normal) else {
                    return p;
                };
                let distance = dot([p[0] - point[0], p[1] - point[1], p[2] - point[2]], n);
                std::array::from_fn(|k| p[k] - 2.0 * distance * n[k])
            }
            Self::Scale { center, factors } => {
                std::array::from_fn(|k| center[k] + (p[k] - center[k]) * factors[k])
            }
        }
    }
}

/// The node order of an element reflected in a plane, so that its Jacobian stays positive
/// and a shell's normal is the mirror image of the original.
fn inverted_order(shape: ElementShape) -> &'static [usize] {
    match shape {
        ElementShape::Line2 => &[0, 1],
        ElementShape::Line3 => &[0, 1, 2],
        ElementShape::Tri3 => &[0, 2, 1],
        ElementShape::Tri6 => &[0, 2, 1, 5, 4, 3],
        ElementShape::Quad4 => &[0, 3, 2, 1],
        ElementShape::Quad8 => &[0, 3, 2, 1, 7, 6, 5, 4],
        ElementShape::Tet4 => &[0, 2, 1, 3],
        ElementShape::Tet10 => &[0, 2, 1, 3, 6, 5, 4, 7, 9, 8],
        ElementShape::Wedge6 => &[0, 2, 1, 3, 5, 4],
        ElementShape::Wedge15 => &[0, 2, 1, 3, 5, 4, 8, 7, 6, 11, 10, 9, 12, 14, 13],
        ElementShape::Hex8 => &[0, 3, 2, 1, 4, 7, 6, 5],
        ElementShape::Hex20 => &[
            0, 3, 2, 1, 4, 7, 6, 5, 11, 10, 9, 8, 15, 14, 13, 12, 16, 19, 18, 17,
        ],
    }
}

impl Element {
    /// Reverses the orientation of the element and returns where each face went: the new
    /// 1-based face number of each old one, in the order of the old faces.
    pub fn invert(&mut self) -> Vec<u8> {
        let old_faces: Vec<BTreeSet<NodeId>> = (self.faces().iter())
            .map(|f| f.corners.iter().map(|&i| self.nodes[i]).collect())
            .collect();
        let order = inverted_order(self.shape);
        self.nodes = order.iter().map(|&i| self.nodes[i]).collect();
        let new_faces: Vec<BTreeSet<NodeId>> = (self.faces().iter())
            .map(|f| f.corners.iter().map(|&i| self.nodes[i]).collect())
            .collect();
        let mut moved: Vec<u8> = (old_faces.iter())
            .map(|old| {
                let position = new_faces.iter().position(|new| new == old);
                position.map_or(0, |p| p as u8 + 1)
            })
            .collect();
        // The bottom and the top of a shell have the same nodes; turned over, they swap.
        if self.is_shell() {
            moved[0] = 2;
            moved[1] = 1;
        }
        moved
    }
}

/// Face numbers of elements that changed, from [`FeMesh::transform_parts`]: the new 1-based
/// face number of each old one.
pub type FaceRenumbering = BTreeMap<ElementId, Vec<u8>>;

fn renumber_face(renumbering: &FaceRenumbering, element: ElementId, face: u8) -> u8 {
    match renumbering.get(&element) {
        Some(faces) => faces
            .get(usize::from(face).wrapping_sub(1))
            .copied()
            .filter(|f| *f > 0)
            .unwrap_or(face),
        None => face,
    }
}

impl FeMesh {
    /// Nodes used by the elements of the parts, ascending.
    pub fn part_nodes(&self, parts: &[usize]) -> Vec<NodeId> {
        let mut nodes = BTreeSet::new();
        for part in parts.iter().filter_map(|&p| self.parts.get(p)) {
            for element in part.elements.iter().filter_map(|&e| self.element(e)) {
                nodes.extend(element.nodes.iter().copied());
            }
        }
        nodes.into_iter().collect()
    }

    /// Moves the nodes of the parts by the transform. A reflection inverts their elements,
    /// so that CalculiX still finds positive Jacobians; the element faces of surfaces and
    /// of the CAD map follow, and the returned renumbering says where the faces of each
    /// inverted element went, for regions kept outside the mesh.
    pub fn transform_parts(
        &mut self,
        parts: &[usize],
        transform: &MeshTransform,
    ) -> FaceRenumbering {
        for id in self.part_nodes(parts) {
            if let Some(index) = self.node_index(id) {
                self.coords[index] = transform.apply(self.coords[index]);
            }
        }
        let mut renumbering = FaceRenumbering::new();
        if transform.inverts() {
            let elements: BTreeSet<ElementId> = (parts.iter())
                .filter_map(|&p| self.parts.get(p))
                .flat_map(|p| p.elements.iter().copied())
                .collect();
            for element in &mut self.elements {
                if elements.contains(&element.id) {
                    let faces = element.invert();
                    if faces
                        .iter()
                        .enumerate()
                        .any(|(i, &f)| usize::from(f) != i + 1)
                    {
                        renumbering.insert(element.id, faces);
                    }
                }
            }
            for surface in self.surfaces.values_mut() {
                if let SurfaceDefinition::ElementFaces(faces) = surface {
                    for (element, face) in faces.iter_mut() {
                        *face = renumber_face(&renumbering, *element, *face);
                    }
                }
            }
            for faces in self.cad.faces.values_mut() {
                for (element, face) in faces.iter_mut() {
                    *face = renumber_face(&renumbering, *element, *face);
                }
            }
        }
        renumbering
    }

    /// Nodes of the parts (all parts if none are given) lying within `tolerance` of a node
    /// with a lower id: each mapped to the lowest node at its place, ready for
    /// [`FeMesh::merge_nodes`].
    pub fn coincident_nodes(&self, parts: &[usize], tolerance: f64) -> BTreeMap<NodeId, NodeId> {
        let ids: Vec<NodeId> = if parts.is_empty() {
            self.node_ids.clone()
        } else {
            self.part_nodes(parts)
        };
        let mut candidates: Vec<(NodeId, [f64; 3])> = (ids.into_iter())
            .filter_map(|id| Some((id, self.node(id)?)))
            .collect();
        // A sweep along x: only nodes within the tolerance in x can be within it in space.
        candidates.sort_by(|a, b| a.1[0].total_cmp(&b.1[0]).then(a.0.cmp(&b.0)));
        let mut replaced = BTreeMap::new();
        for i in 0..candidates.len() {
            let (node, p) = candidates[i];
            if replaced.contains_key(&node) {
                continue;
            }
            for &(other, q) in &candidates[i + 1..] {
                if q[0] - p[0] > tolerance {
                    break;
                }
                let d = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
                if dot(d, d).sqrt() <= tolerance && !replaced.contains_key(&other) {
                    let (kept, dropped) = if other < node {
                        (other, node)
                    } else {
                        (node, other)
                    };
                    let kept = replaced.get(&kept).copied().unwrap_or(kept);
                    replaced.insert(dropped, kept);
                }
            }
        }
        replaced
    }

    /// Numbers the nodes and elements anew from the start numbers, in ascending order of
    /// their old ids, and returns the old to new maps so that regions outside the mesh can
    /// follow.
    pub fn renumber(
        &mut self,
        first_node: NodeId,
        first_element: ElementId,
    ) -> (BTreeMap<NodeId, NodeId>, BTreeMap<ElementId, ElementId>) {
        let mut sorted_nodes = self.node_ids.clone();
        sorted_nodes.sort_unstable();
        let nodes: BTreeMap<NodeId, NodeId> = (sorted_nodes.into_iter().enumerate())
            .map(|(i, old)| (old, first_node + i as NodeId))
            .collect();
        let mut sorted_elements: Vec<ElementId> = self.elements.iter().map(|e| e.id).collect();
        sorted_elements.sort_unstable();
        let elements: BTreeMap<ElementId, ElementId> = (sorted_elements.into_iter().enumerate())
            .map(|(i, old)| (old, first_element + i as ElementId))
            .collect();
        let node = |n: NodeId| nodes.get(&n).copied().unwrap_or(n);
        let element = |e: ElementId| elements.get(&e).copied().unwrap_or(e);
        for id in &mut self.node_ids {
            *id = node(*id);
        }
        self.node_lookup = (self.node_ids.iter().copied())
            .enumerate()
            .map(|(index, id)| (id, index))
            .collect();
        for e in &mut self.elements {
            e.id = element(e.id);
            for n in &mut e.nodes {
                *n = node(*n);
            }
        }
        self.element_lookup = (self.elements.iter())
            .enumerate()
            .map(|(index, e)| (e.id, index))
            .collect();
        for set in self.node_sets.values_mut() {
            set.iter_mut().for_each(|n| *n = node(*n));
            set.sort_unstable();
        }
        for set in self.element_sets.values_mut() {
            set.iter_mut().for_each(|e| *e = element(*e));
            set.sort_unstable();
        }
        for surface in self.surfaces.values_mut() {
            match surface {
                SurfaceDefinition::ElementFaces(faces) => {
                    faces.iter_mut().for_each(|(e, _)| *e = element(*e));
                }
                SurfaceDefinition::Nodes(ns) => {
                    ns.iter_mut().for_each(|n| *n = node(*n));
                    ns.sort_unstable();
                }
            }
        }
        for part in &mut self.parts {
            part.elements.iter_mut().for_each(|e| *e = element(*e));
        }
        self.cad = CadMap {
            nodes: (self.cad.nodes.iter())
                .map(|(&entity, ns)| {
                    let mut ns: Vec<NodeId> = ns.iter().map(|&n| node(n)).collect();
                    ns.sort_unstable();
                    (entity, ns)
                })
                .collect(),
            faces: (self.cad.faces.iter())
                .map(|(&tag, faces)| (tag, faces.iter().map(|&(e, f)| (element(e), f)).collect()))
                .collect(),
            segments: (self.cad.segments.iter())
                .map(|(&edge, segments)| (edge, segments.iter().map(|s| s.map(node)).collect()))
                .collect(),
        };
        (nodes, elements)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::Part;

    /// Corner nodes of the unit shapes and the corner pairs of their midside nodes, in
    /// CalculiX order.
    fn corners(shape: ElementShape) -> (&'static [[f64; 3]], &'static [[usize; 2]]) {
        const HEX: [[f64; 3]; 8] = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ];
        const HEX_EDGES: [[usize; 2]; 12] = [
            [0, 1],
            [1, 2],
            [2, 3],
            [3, 0],
            [4, 5],
            [5, 6],
            [6, 7],
            [7, 4],
            [0, 4],
            [1, 5],
            [2, 6],
            [3, 7],
        ];
        const TET: [[f64; 3]; 4] = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        const TET_EDGES: [[usize; 2]; 6] = [[0, 1], [1, 2], [2, 0], [0, 3], [1, 3], [2, 3]];
        const WEDGE: [[f64; 3]; 6] = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [0.0, 1.0, 1.0],
        ];
        const WEDGE_EDGES: [[usize; 2]; 9] = [
            [0, 1],
            [1, 2],
            [2, 0],
            [3, 4],
            [4, 5],
            [5, 3],
            [0, 3],
            [1, 4],
            [2, 5],
        ];
        const TRI: [[f64; 3]; 3] = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        const TRI_EDGES: [[usize; 2]; 3] = [[0, 1], [1, 2], [2, 0]];
        const QUAD: [[f64; 3]; 4] = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        const QUAD_EDGES: [[usize; 2]; 4] = [[0, 1], [1, 2], [2, 3], [3, 0]];
        match shape {
            ElementShape::Hex8 => (&HEX, &[]),
            ElementShape::Hex20 => (&HEX, &HEX_EDGES),
            ElementShape::Tet4 => (&TET, &[]),
            ElementShape::Tet10 => (&TET, &TET_EDGES),
            ElementShape::Wedge6 => (&WEDGE, &[]),
            ElementShape::Wedge15 => (&WEDGE, &WEDGE_EDGES),
            ElementShape::Tri3 => (&TRI, &[]),
            ElementShape::Tri6 => (&TRI, &TRI_EDGES),
            ElementShape::Quad4 => (&QUAD, &[]),
            ElementShape::Quad8 => (&QUAD, &QUAD_EDGES),
            _ => unreachable!(),
        }
    }

    /// A mesh of one element of the shape, nodes 1.., slightly sheared so that no symmetry
    /// hides a wrong node order.
    fn single(shape: ElementShape, type_name: &str) -> FeMesh {
        let (corners, edges) = corners(shape);
        let shear = |p: [f64; 3]| [p[0] + 0.2 * p[1] + 0.1 * p[2], p[1] + 0.1 * p[2], p[2]];
        let mut points: Vec<[f64; 3]> = corners.iter().map(|&p| shear(p)).collect();
        for [a, b] in edges {
            let (a, b) = (points[*a], points[*b]);
            points.push(std::array::from_fn(|k| 0.5 * (a[k] + b[k])));
        }
        let mut mesh = FeMesh::default();
        for (i, p) in points.iter().enumerate() {
            mesh.set_node(i as NodeId + 1, *p);
        }
        mesh.add_element(Element {
            id: 1,
            type_name: type_name.into(),
            shape,
            nodes: (1..=points.len() as NodeId).collect(),
        })
        .unwrap();
        mesh.parts.push(Part {
            bodies: Vec::new(),
            name: "PART".into(),
            elements: vec![1],
        });
        mesh
    }

    #[test]
    fn mirrored_elements_keep_a_positive_jacobian() {
        let mirror = MeshTransform::Mirror {
            point: [0.0; 3],
            normal: [1.0, 2.0, 0.5],
        };
        for (shape, name) in [
            (ElementShape::Hex8, "C3D8"),
            (ElementShape::Hex20, "C3D20"),
            (ElementShape::Tet4, "C3D4"),
            (ElementShape::Tet10, "C3D10"),
            (ElementShape::Wedge6, "C3D6"),
            (ElementShape::Wedge15, "C3D15"),
        ] {
            let mut mesh = single(shape, name);
            let before = mesh.elements()[0].min_jacobian(&mesh).unwrap();
            assert!(before > 0.0, "{name}: {before}");
            mesh.transform_parts(&[0], &mirror);
            let after = mesh.elements()[0].min_jacobian(&mesh).unwrap();
            assert!(after > 0.0, "{name} inverted: {after}");
            assert!((after - before).abs() < 1e-9, "{name}: {before} vs {after}");
        }
        // Plane elements are checked in the xy plane, mirrored at a plane normal to it.
        let mirror = MeshTransform::Mirror {
            point: [0.5, 0.0, 0.0],
            normal: [1.0, 0.0, 0.0],
        };
        for (shape, name) in [
            (ElementShape::Tri3, "CPS3"),
            (ElementShape::Tri6, "CPS6"),
            (ElementShape::Quad4, "CPS4"),
            (ElementShape::Quad8, "CPS8"),
        ] {
            let mut mesh = single(shape, name);
            mesh.transform_parts(&[0], &mirror);
            let after = mesh.elements()[0].min_jacobian(&mesh).unwrap();
            assert!(after > 0.0, "{name} inverted: {after}");
        }
    }

    #[test]
    fn a_mirrored_hex_keeps_its_bottom_face_and_renumbers_the_sides() {
        let mut mesh = single(ElementShape::Hex8, "C3D8");
        mesh.surfaces.insert(
            "SIDES".into(),
            SurfaceDefinition::ElementFaces(vec![(1, 1), (1, 3), (1, 4)]),
        );
        let renumbering = mesh.transform_parts(
            &[0],
            &MeshTransform::Mirror {
                point: [0.0; 3],
                normal: [0.0, 0.0, 1.0],
            },
        );
        // Corners 0 3 2 1: the bottom (S1) and top (S2) stay, the side through nodes 1 2
        // (old S4) is now the one through the second and third corner (S3).
        assert_eq!(renumbering[&1], vec![1, 2, 6, 5, 4, 3]);
        assert_eq!(
            mesh.surfaces["SIDES"],
            SurfaceDefinition::ElementFaces(vec![(1, 1), (1, 6), (1, 5)])
        );
        assert_eq!(mesh.node(5).unwrap()[2], -1.0);
    }

    #[test]
    fn rotation_and_scaling_move_the_points() {
        let rotate = MeshTransform::Rotate {
            point: [1.0, 0.0, 0.0],
            axis: [0.0, 0.0, 2.0],
            angle: 90.0,
        };
        let p = rotate.apply([2.0, 0.0, 5.0]);
        assert!(
            (p[0] - 1.0).abs() < 1e-12 && (p[1] - 1.0).abs() < 1e-12 && (p[2] - 5.0).abs() < 1e-12
        );
        let scale = MeshTransform::Scale {
            center: [1.0, 1.0, 1.0],
            factors: [2.0, -1.0, 1.0],
        };
        assert_eq!(scale.apply([2.0, 2.0, 2.0]), [3.0, 0.0, 2.0]);
        assert!(scale.inverts());
        assert!(!rotate.inverts());
        let bad = MeshTransform::Rotate {
            point: [0.0; 3],
            axis: [0.0; 3],
            angle: 1.0,
        };
        assert!(bad.problem().is_some());
    }

    #[test]
    fn coincident_nodes_are_merged_to_the_lowest_id() {
        let mut mesh = single(ElementShape::Hex8, "C3D8");
        mesh.set_node(20, mesh.node(3).unwrap());
        let mut near = mesh.node(1).unwrap();
        near[1] += 1e-4;
        mesh.set_node(21, near);
        mesh.set_node(22, [5.0, 5.0, 5.0]);
        let replaced = mesh.coincident_nodes(&[], 1e-3);
        assert_eq!(replaced, BTreeMap::from([(20, 3), (21, 1)]));
        assert!(mesh.coincident_nodes(&[], 1e-6).contains_key(&20));
        assert!(!mesh.coincident_nodes(&[], 1e-6).contains_key(&21));
        // Only the part's nodes when parts are given: node 20 is not in one.
        assert_eq!(mesh.coincident_nodes(&[0], 1e-3), BTreeMap::new());
    }

    #[test]
    fn renumbering_follows_into_sets_surfaces_and_the_cad_map() {
        let mut mesh = single(ElementShape::Tet4, "C3D4");
        mesh.set_node(10, [3.0, 3.0, 3.0]);
        mesh.add_element(Element {
            id: 7,
            type_name: "C3D4".into(),
            shape: ElementShape::Tet4,
            nodes: vec![2, 3, 4, 10],
        })
        .unwrap();
        mesh.parts[0].elements.push(7);
        mesh.node_sets.insert("N".into(), vec![4, 10]);
        mesh.element_sets.insert("E".into(), vec![7]);
        (mesh.surfaces).insert("S".into(), SurfaceDefinition::ElementFaces(vec![(7, 2)]));
        mesh.cad.nodes.insert(crate::CadEntity::Vertex(1), vec![10]);
        mesh.cad.faces.insert(1, vec![(7, 2)]);
        mesh.cad.segments.insert(1, vec![[4, 10]]);
        let (nodes, elements) = mesh.renumber(100, 50);
        assert_eq!(nodes[&10], 104);
        assert_eq!(elements, BTreeMap::from([(1, 50), (7, 51)]));
        assert_eq!(mesh.node(104), Some([3.0, 3.0, 3.0]));
        assert_eq!(mesh.element(51).unwrap().nodes, vec![101, 102, 103, 104]);
        assert_eq!(mesh.node_sets["N"], vec![103, 104]);
        assert_eq!(mesh.element_sets["E"], vec![51]);
        assert_eq!(
            mesh.surfaces["S"],
            SurfaceDefinition::ElementFaces(vec![(51, 2)])
        );
        assert_eq!(mesh.parts[0].elements, vec![50, 51]);
        assert_eq!(mesh.cad.nodes[&crate::CadEntity::Vertex(1)], vec![104]);
        assert_eq!(mesh.cad.faces[&1], vec![(51, 2)]);
        assert_eq!(mesh.cad.segments[&1], vec![[103, 104]]);
        assert_eq!(mesh.node_index(100), Some(0));
    }
}
