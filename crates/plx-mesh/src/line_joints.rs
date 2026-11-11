//! The search for line ends meeting at a point. Beams and trusses meshed from separate CAD
//! edges meet at the same points but have nodes of their own there, as parts share no
//! nodes; CalculiX connects elements only through shared nodes or equations between them.
//! The contact search finds such ends, and the user ties them as node ties.

use std::collections::{BTreeMap, BTreeSet};

use crate::element::ElementShape;
use crate::mesh::{FeMesh, NodeId};

/// Ends of line elements of different parts that lie on one point.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LineJoint {
    /// The nodes at the point, ascending.
    pub nodes: Vec<NodeId>,
    /// The parts of the lines, by index.
    pub parts: BTreeSet<usize>,
}

/// Finds the ends of line elements (beams, trusses) of the searched parts that lie within
/// `distance` of an end of another part. Ends on a node that a solid or shell element uses
/// are left out, since lines on a solid are tied to it otherwise.
pub fn find_line_joints(mesh: &FeMesh, searched: &[bool], distance: f64) -> Vec<LineJoint> {
    let mut other_nodes = BTreeSet::new();
    let mut ends: BTreeMap<NodeId, usize> = BTreeMap::new();
    for (part, entry) in mesh.parts.iter().enumerate() {
        let searched = searched.get(part).copied().unwrap_or(false);
        for element in entry.elements.iter().filter_map(|&id| mesh.element(id)) {
            match element.shape {
                ElementShape::Line2 | ElementShape::Line3 => {
                    if searched {
                        ends.insert(element.nodes[0], part);
                        ends.insert(element.nodes[element.nodes.len() - 1], part);
                    }
                }
                _ => other_nodes.extend(element.nodes.iter().copied()),
            }
        }
    }
    let mut candidates: Vec<(NodeId, usize, [f64; 3])> = (ends.into_iter())
        .filter(|(n, _)| !other_nodes.contains(n))
        .filter_map(|(n, part)| Some((n, part, mesh.node(n)?)))
        .collect();
    // A sweep along x: only ends within the distance in x can be within it in space.
    candidates.sort_by(|a, b| a.2[0].total_cmp(&b.2[0]).then(a.0.cmp(&b.0)));
    let mut taken = vec![false; candidates.len()];
    let mut joints = Vec::new();
    for i in 0..candidates.len() {
        if taken[i] {
            continue;
        }
        let (node, part, p) = candidates[i];
        let mut joint = LineJoint {
            nodes: vec![node],
            parts: BTreeSet::from([part]),
        };
        for j in i + 1..candidates.len() {
            let (other, other_part, q) = candidates[j];
            if q[0] - p[0] > distance {
                break;
            }
            if !taken[j] && other_part != part && separation(p, q) <= distance {
                taken[j] = true;
                joint.nodes.push(other);
                joint.parts.insert(other_part);
            }
        }
        if joint.nodes.len() > 1 {
            joint.nodes.sort_unstable();
            joints.push(joint);
        }
    }
    joints
}

fn separation(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::{Element, Part};

    fn line(id: u32, nodes: Vec<NodeId>) -> Element {
        Element {
            id,
            type_name: "B31".into(),
            shape: ElementShape::Line2,
            nodes,
        }
    }

    /// Three bars: two meeting at (10, 0, 0) with nodes of their own there, the third
    /// starting at the far end of the second, where a tetrahedron has a corner too.
    fn loose_bars() -> FeMesh {
        let mut mesh = FeMesh::default();
        for (id, coords) in [
            (1, [0.0, 0.0, 0.0]),
            (2, [10.0, 0.0, 0.0]),
            (3, [10.0, 0.0, 1e-9]),
            (4, [10.0, 10.0, 0.0]),
            (5, [10.0, 10.0, 0.0]),
            (6, [11.0, 10.0, 0.0]),
            (7, [10.0, 11.0, 0.0]),
            (8, [10.0, 10.0, 1.0]),
            (9, [10.0, 10.0, 0.0]),
            (10, [20.0, 10.0, 0.0]),
        ] {
            mesh.set_node(id, coords);
        }
        mesh.add_element(line(1, vec![1, 2])).unwrap();
        mesh.add_element(line(2, vec![3, 4])).unwrap();
        mesh.add_element(line(4, vec![9, 10])).unwrap();
        mesh.add_element(Element {
            id: 3,
            type_name: "C3D4".into(),
            shape: ElementShape::Tet4,
            nodes: vec![5, 6, 7, 8],
        })
        .unwrap();
        for (name, elements) in [("LINE-1", 1), ("LINE-2", 2), ("SOLID-1", 3), ("LINE-3", 4)] {
            mesh.parts.push(Part {
                bodies: Vec::new(),
                name: name.into(),
                elements: vec![elements],
            });
        }
        mesh
    }

    #[test]
    fn ends_on_one_point_are_a_joint() {
        let mesh = loose_bars();
        let joints = find_line_joints(&mesh, &[true; 4], 1e-6);
        assert_eq!(
            joints,
            [
                LineJoint {
                    nodes: vec![2, 3],
                    parts: BTreeSet::from([0, 1]),
                },
                // The tetrahedron's corner at (10, 10, 0) is not a line end.
                LineJoint {
                    nodes: vec![4, 9],
                    parts: BTreeSet::from([1, 3]),
                },
            ]
        );
        // Parts not searched do not take part; ends apart stay apart.
        let joints = find_line_joints(&mesh, &[true, true, true, false], 1e-6);
        assert_eq!(joints.len(), 1);
        let mut apart = mesh.clone();
        apart.set_node(3, [10.0, 0.5, 0.0]);
        assert_eq!(find_line_joints(&apart, &[true; 4], 0.1).len(), 1);
        assert_eq!(find_line_joints(&apart, &[true; 4], 1.0).len(), 2);
    }
}
