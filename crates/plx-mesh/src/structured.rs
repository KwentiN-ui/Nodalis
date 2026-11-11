//! Structured meshes of simple shapes: a box of hexahedra and a rectangle of quadrilaterals,
//! made by three nested loops without a mesher. Their sides are node sets and surfaces named
//! after the axis and side they lie on (`X-`, `X+`, `Y-` ...), so that boundary conditions
//! and loads need no selection. Their sides, edges and corners are also CAD faces, edges and
//! vertices, see [`cad_map`], so that a selection by geometry survives the mesh being made
//! anew with other divisions.

use std::collections::{BTreeMap, BTreeSet};

use crate::cad::{CadEntity, CadMap};
use crate::element::ElementShape;
use crate::mesh::{Element, FeMesh, NodeId, Part, SurfaceDefinition};

/// Names of the sides of a structured mesh, by axis: the side at the smallest coordinate,
/// then the one at the largest.
pub const SIDES: [[&str; 2]; 3] = [["X-", "X+"], ["Y-", "Y+"], ["Z-", "Z+"]];

/// Corners of a hexahedron in CalculiX order, as steps of the grid.
const HEX_CORNERS: [[usize; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [1, 1, 0],
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [1, 1, 1],
    [0, 1, 1],
];
/// Midside nodes of a 20-node hexahedron in CalculiX order, in half steps of the grid.
const HEX_MIDS: [[usize; 3]; 12] = [
    [1, 0, 0],
    [2, 1, 0],
    [1, 2, 0],
    [0, 1, 0],
    [1, 0, 2],
    [2, 1, 2],
    [1, 2, 2],
    [0, 1, 2],
    [0, 0, 1],
    [2, 0, 1],
    [2, 2, 1],
    [0, 2, 1],
];
const QUAD_CORNERS: [[usize; 3]; 4] = [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]];
const QUAD_MIDS: [[usize; 3]; 4] = [[1, 0, 0], [2, 1, 0], [1, 2, 0], [0, 1, 0]];

/// A box from `origin` with edge lengths `size`, divided into `divisions` hexahedra along
/// each axis, of the element type `type_name`; `quadratic` elements have midside nodes
/// (20-node hexahedra). Its elements form the part `part`.
pub fn box_mesh(
    origin: [f64; 3],
    size: [f64; 3],
    divisions: [u32; 3],
    type_name: &str,
    quadratic: bool,
    part: &str,
) -> FeMesh {
    structured(origin, size, divisions, 3, type_name, quadratic, part)
}

/// A rectangle in the XY plane from `origin` with edge lengths `size`, divided into
/// `divisions` quadrilaterals, of the plane element type `type_name` (CPS, CPE or CAX);
/// `quadratic` elements have midside nodes (8-node quadrilaterals).
pub fn rectangle_mesh(
    origin: [f64; 2],
    size: [f64; 2],
    divisions: [u32; 2],
    type_name: &str,
    quadratic: bool,
    part: &str,
) -> FeMesh {
    let origin = [origin[0], origin[1], 0.0];
    let size = [size[0], size[1], 0.0];
    let divisions = [divisions[0], divisions[1], 0];
    structured(origin, size, divisions, 2, type_name, quadratic, part)
}

fn structured(
    origin: [f64; 3],
    size: [f64; 3],
    divisions: [u32; 3],
    dimensions: usize,
    type_name: &str,
    quadratic: bool,
    part: &str,
) -> FeMesh {
    let n: [usize; 3] = std::array::from_fn(|axis| {
        if axis < dimensions {
            divisions[axis].max(1) as usize
        } else {
            0
        }
    });
    // Points of the grid: nodes at the corners, and in between for quadratic elements.
    let step = if quadratic { 2 } else { 1 };
    let points = n.map(|n| n * step + 1);
    let index = |p: [usize; 3]| p[0] + points[0] * (p[1] + points[1] * p[2]);
    let (corners, mids, shape) = match (dimensions, quadratic) {
        (3, false) => (&HEX_CORNERS[..], &[][..], ElementShape::Hex8),
        (3, true) => (&HEX_CORNERS[..], &HEX_MIDS[..], ElementShape::Hex20),
        (_, false) => (&QUAD_CORNERS[..], &[][..], ElementShape::Quad4),
        (_, true) => (&QUAD_CORNERS[..], &QUAD_MIDS[..], ElementShape::Quad8),
    };
    // The grid points of each element's nodes.
    let mut cells = Vec::with_capacity(n[0] * n[1] * n[2].max(1));
    for k in 0..n[2].max(1) {
        for j in 0..n[1] {
            for i in 0..n[0] {
                let base = [i * step, j * step, k * step];
                let at = |offset: &[usize; 3], scale: usize| {
                    index(std::array::from_fn(|a| base[a] + offset[a] * scale))
                };
                let nodes: Vec<usize> = (corners.iter().map(|c| at(c, step)))
                    .chain(mids.iter().map(|m| at(m, 1)))
                    .collect();
                cells.push(nodes);
            }
        }
    }
    // Node ids in the order of the grid, for the points elements use.
    let mut ids: Vec<NodeId> = vec![0; points.iter().product()];
    for &point in cells.iter().flatten() {
        ids[point] = 1;
    }
    let mut mesh = FeMesh::default();
    let mut next: NodeId = 0;
    for (point, id) in ids.iter_mut().enumerate() {
        if *id == 0 {
            continue;
        }
        next += 1;
        *id = next;
        let grid = [
            point % points[0],
            point / points[0] % points[1],
            point / (points[0] * points[1]),
        ];
        let coords = std::array::from_fn(|a| {
            let cells = (points[a] - 1).max(1) as f64;
            origin[a] + size[a] * grid[a] as f64 / cells
        });
        mesh.set_node(next, coords);
    }
    let mut elements = Vec::with_capacity(cells.len());
    for (number, cell) in cells.iter().enumerate() {
        let id = number as u32 + 1;
        let element = Element {
            id,
            type_name: type_name.to_owned(),
            shape,
            nodes: cell.iter().map(|&p| ids[p]).collect(),
        };
        mesh.add_element(element)
            .expect("structured elements have their nodes");
        elements.push(id);
    }
    // Sides: the nodes on them, and the element faces whose corners all lie on them.
    for axis in 0..dimensions {
        for (end, name) in SIDES[axis].iter().enumerate() {
            let on_side = |point: usize| {
                let along = [
                    point % points[0],
                    point / points[0] % points[1],
                    point / (points[0] * points[1]),
                ][axis];
                along == if end == 0 { 0 } else { points[axis] - 1 }
            };
            let nodes: Vec<NodeId> = (ids.iter().enumerate())
                .filter(|(point, id)| **id != 0 && on_side(*point))
                .map(|(_, id)| *id)
                .collect();
            let mut faces = Vec::new();
            for (cell, element) in cells.iter().zip(mesh.elements()) {
                for (face, topology) in element.faces().iter().enumerate() {
                    if topology.corners.iter().all(|&c| on_side(cell[c])) {
                        faces.push((element.id, face as u8 + 1));
                    }
                }
            }
            mesh.node_sets.insert((*name).to_owned(), nodes);
            mesh.surfaces
                .insert((*name).to_owned(), SurfaceDefinition::ElementFaces(faces));
        }
    }
    mesh.parts.push(Part {
        bodies: Vec::new(),
        name: part.to_uppercase(),
        elements,
    });
    mesh.cad = cad_map(&mesh, &ids, &cells, points, dimensions, step);
    mesh
}

/// The shape's sides, edges and corners as CAD entities, numbered from its axes so that they
/// stay the same for any division. A box has the faces X- 1, X+ 2, Y- 3, Y+ 4, Z- 5 and
/// Z+ 6, its edges along X 1 to 4, along Y 5 to 8 and along Z 9 to 12, and its corners 1 to
/// 8 (1 + X end + 2 Y end + 4 Z end). A rectangle is face 1 with the edges X- 1, X+ 2, Y- 3
/// and Y+ 4 and the corners 1 to 4.
fn cad_map(
    mesh: &FeMesh,
    ids: &[NodeId],
    cells: &[Vec<usize>],
    points: [usize; 3],
    dimensions: usize,
    step: usize,
) -> CadMap {
    let grid = |point: usize| {
        [
            point % points[0],
            point / points[0] % points[1],
            point / (points[0] * points[1]),
        ]
    };
    // The sides a grid point lies on, as 2 * axis + end.
    let sides = |point: usize| -> Vec<usize> {
        let g = grid(point);
        (0..dimensions)
            .flat_map(|axis| {
                let at_start = (g[axis] == 0).then_some(2 * axis);
                let at_end = (g[axis] == points[axis] - 1).then_some(2 * axis + 1);
                [at_start, at_end]
            })
            .flatten()
            .collect()
    };
    let tag = |n: usize| n as i32 + 1;
    // The edge on two sides of a box, and the axis it runs along.
    let box_edge = |a: usize, b: usize| {
        let along = 3 - a / 2 - b / 2;
        (tag(4 * along + a % 2 + 2 * (b % 2)), along)
    };
    let corner = |sides: &[usize]| tag(sides.iter().map(|s| (s % 2) << (s / 2)).sum());
    let mut map = CadMap::default();
    let mut nodes: BTreeMap<CadEntity, BTreeSet<NodeId>> = BTreeMap::new();
    // The points on each edge, and the axis it runs along, for its segments.
    let mut edges: BTreeMap<i32, (usize, Vec<usize>)> = BTreeMap::new();
    for (point, &id) in ids.iter().enumerate().filter(|(_, id)| **id != 0) {
        let on = sides(point);
        let mut entities = Vec::new();
        if dimensions == 3 {
            entities.extend(on.iter().map(|&s| CadEntity::Face(tag(s))));
            for (i, &a) in on.iter().enumerate() {
                for &b in &on[i + 1..] {
                    let (edge, along) = box_edge(a, b);
                    entities.push(CadEntity::Edge(edge));
                    edges
                        .entry(edge)
                        .or_insert((along, Vec::new()))
                        .1
                        .push(point);
                }
            }
        } else {
            entities.push(CadEntity::Face(1));
            for &s in &on {
                entities.push(CadEntity::Edge(tag(s)));
                edges
                    .entry(tag(s))
                    .or_insert((1 - s / 2, Vec::new()))
                    .1
                    .push(point);
            }
        }
        if on.len() == dimensions {
            entities.push(CadEntity::Vertex(corner(&on)));
        }
        for entity in entities {
            nodes.entry(entity).or_default().insert(id);
        }
    }
    map.nodes = (nodes.into_iter())
        .map(|(entity, ids)| (entity, ids.into_iter().collect()))
        .collect();
    // Element faces on the faces: the solid faces on each side of a box, every element of a
    // rectangle.
    for (cell, element) in cells.iter().zip(mesh.elements()) {
        if dimensions == 3 {
            for (k, topology) in element.faces().iter().enumerate() {
                let on: Vec<Vec<usize>> =
                    topology.corners.iter().map(|&c| sides(cell[c])).collect();
                for &side in &on[0] {
                    if on.iter().all(|s| s.contains(&side)) {
                        let faces = map.faces.entry(tag(side)).or_default();
                        faces.push((element.id, k as u8 + 1));
                    }
                }
            }
        } else {
            map.faces.entry(1).or_default().push((element.id, 1));
        }
    }
    // Segments from corner to corner of the elements along each edge.
    for (edge, (along, mut on_edge)) in edges {
        on_edge.retain(|&p| grid(p)[along] % step == 0);
        on_edge.sort_by_key(|&p| grid(p)[along]);
        let segments = (on_edge.windows(2))
            .map(|pair| [ids[pair[0]], ids[pair[1]]])
            .collect();
        map.segments.insert(edge, segments);
    }
    map
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_box_has_its_nodes_elements_and_sides() {
        let mesh = box_mesh([0.0; 3], [2.0, 1.0, 1.0], [2, 1, 1], "C3D8", false, "Box");
        assert_eq!(mesh.node_count(), 12);
        assert_eq!(mesh.element_count(), 2);
        assert_eq!(mesh.bounds(), Some(([0.0; 3], [2.0, 1.0, 1.0])));
        assert_eq!(mesh.parts[0].name, "BOX");
        // A positive volume: the corners go round counter-clockwise seen from above.
        let element = &mesh.elements()[0];
        let p = |i: usize| mesh.node(element.nodes[i]).unwrap();
        assert_eq!(p(1), [1.0, 0.0, 0.0]);
        assert_eq!(p(3), [0.0, 1.0, 0.0]);
        assert_eq!(p(4), [0.0, 0.0, 1.0]);
        assert_eq!(mesh.node_sets["X-"].len(), 4);
        assert_eq!(mesh.node_sets["X+"].len(), 4);
        assert_eq!(mesh.node_sets["Z+"].len(), 6);
        let faces = |name: &str| match &mesh.surfaces[name] {
            SurfaceDefinition::ElementFaces(faces) => faces.clone(),
            other => panic!("{other:?}"),
        };
        // CalculiX's faces of a hexahedron: S1 at the bottom, S2 at the top, S3 to S6
        // around it starting at Y-.
        assert_eq!(faces("Z-"), [(1, 1), (2, 1)]);
        assert_eq!(faces("Z+"), [(1, 2), (2, 2)]);
        assert_eq!(faces("Y-"), [(1, 3), (2, 3)]);
        assert_eq!(faces("X+"), [(2, 4)]);
        assert_eq!(faces("Y+"), [(1, 5), (2, 5)]);
        assert_eq!(faces("X-"), [(1, 6)]);
    }

    #[test]
    fn quadratic_boxes_leave_out_face_and_cell_centres() {
        let mesh = box_mesh([1.0, 2.0, 3.0], [1.0; 3], [2, 2, 2], "C3D20R", true, "Box");
        assert_eq!(mesh.element_count(), 8);
        // The corners of the cells and the middles of their edges.
        let corners = 3 * 3 * 3;
        let edges = 3 * (2 * 3 * 3);
        assert_eq!(mesh.node_count(), corners + edges);
        assert_eq!(mesh.bounds(), Some(([1.0, 2.0, 3.0], [2.0, 3.0, 4.0])));
        let element = &mesh.elements()[0];
        // The midside node between corners 1 and 2.
        let mid = mesh.node(element.nodes[8]).unwrap();
        assert_eq!(mid, [1.25, 2.0, 3.0]);
        // The side's nodes include the midside ones: 5 x 5 less the 4 face centres.
        assert_eq!(mesh.node_sets["X-"].len(), 21);
    }

    #[test]
    fn a_rectangle_has_edges_as_sides() {
        let mesh = rectangle_mesh([0.0, 0.0], [3.0, 1.0], [3, 2], "CPS8", true, "Rectangle");
        assert_eq!(mesh.element_count(), 6);
        assert_eq!(mesh.node_count(), 7 * 5 - 6);
        assert!(!mesh.node_sets.contains_key("Z-"));
        assert_eq!(mesh.node_sets["Y+"].len(), 7);
        let SurfaceDefinition::ElementFaces(faces) = &mesh.surfaces["X+"] else {
            panic!()
        };
        // Edge faces of a plane element: S2 from corner 2 to 3.
        assert_eq!(faces, &[(3, 2), (6, 2)]);
        assert!(mesh.coords().iter().all(|c| c[2] == 0.0));
    }

    #[test]
    fn the_sides_of_a_box_are_cad_faces_that_stay_when_it_is_divided_anew() {
        let coarse = box_mesh([0.0; 3], [2.0, 1.0, 1.0], [2, 1, 1], "C3D8", false, "Box");
        let fine = box_mesh([0.0; 3], [2.0, 1.0, 1.0], [4, 2, 2], "C3D20R", true, "Box");
        for mesh in [&coarse, &fine] {
            let cad = &mesh.cad;
            assert_eq!(cad.faces.len(), 6);
            assert_eq!(cad.segments.len(), 12);
            let vertices = cad
                .nodes
                .keys()
                .filter(|e| matches!(e, CadEntity::Vertex(_)));
            assert_eq!(vertices.count(), 8);
            // Face X+ is the surface X+, its nodes the node set.
            let SurfaceDefinition::ElementFaces(faces) = &mesh.surfaces["X+"] else {
                panic!()
            };
            assert_eq!(&cad.faces[&2], faces);
            assert_eq!(cad.nodes[&CadEntity::Face(2)], mesh.node_sets["X+"]);
            // The corner at the far end of X, Y and Z, and the edge along X at Y+ Z+.
            let far = cad.nodes[&CadEntity::Vertex(8)][0];
            assert_eq!(mesh.node(far), Some([2.0, 1.0, 1.0]));
            let edge = &cad.nodes[&CadEntity::Edge(4)];
            assert!(edge.iter().all(|&n| {
                let [_, y, z] = mesh.node(n).unwrap();
                y == 1.0 && z == 1.0
            }));
        }
        assert_eq!(coarse.cad.segments[&4].len(), 2);
        assert_eq!(fine.cad.segments[&4].len(), 4);
    }

    #[test]
    fn a_rectangle_is_one_cad_face_with_its_sides_as_edges() {
        let mesh = rectangle_mesh([0.0, 0.0], [3.0, 1.0], [3, 2], "CPS4", false, "Rectangle");
        let cad = &mesh.cad;
        assert_eq!(cad.faces[&1].len(), mesh.element_count());
        assert_eq!(cad.nodes[&CadEntity::Face(1)].len(), mesh.node_count());
        assert_eq!(cad.segments.len(), 4);
        assert_eq!(cad.segments[&2].len(), 2);
        assert_eq!(cad.nodes[&CadEntity::Edge(2)], mesh.node_sets["X+"]);
        let corner = cad.nodes[&CadEntity::Vertex(4)][0];
        assert_eq!(mesh.node(corner), Some([3.0, 1.0, 0.0]));
    }
}
