//! Thicken Shell Mesh: the face mesh of a shell part is offset along its normals in layers,
//! so that the shell becomes a solid of wedges and hexahedra. The normals are turned to one
//! side of the shell first. Where faces of the shell meet at an angle, the nodes move so that
//! the solid keeps its thickness on each of them, as the offset faces would meet.
//!
//! Selections picked on the geometry stay: a face of the shell takes the side of the solid
//! nearest to the shell's surface, an edge or a vertex all nodes through the thickness at it.

use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};

use plx_mesh::{CadEntity, CadMap, Element, ElementId, ElementShape, FeMesh, NodeId, Part};
use plx_model::ThickenShellMesh;

use crate::extrude::{Point, add, cross, dot, norm, scale, sub};

/// Normals of faces that meet at less than this angle count as one, as for a smooth
/// surface split into several faces.
const SAME_NORMAL_DEGREES: f64 = 5.0;

/// Offsets longer than this many thicknesses would be made at very sharp edges, where the
/// offset faces meet far away; they are shortened.
const LONGEST_OFFSET: f64 = 4.0;

/// A shell element as corners and midside nodes, turned to the side of the shell.
struct ShellElement {
    corners: Vec<NodeId>,
    mids: Vec<NodeId>,
    /// The CAD face the element lies on, if known.
    face: Option<i32>,
}

/// The shell mesh of one part as a solid, numbered from 1, with its CAD map.
pub(crate) fn thicken(
    shell: &FeMesh,
    part: &str,
    thicken: &ThickenShellMesh,
) -> Result<FeMesh, String> {
    thicken.check()?;
    let face_of: HashMap<ElementId, i32> = (shell.cad.faces.iter())
        .flat_map(|(&face, items)| items.iter().map(move |&(e, _)| (e, face)))
        .collect();
    let mut elements = Vec::new();
    for element in shell.elements() {
        let (corners, quadratic) = match element.shape {
            ElementShape::Tri3 => (3, false),
            ElementShape::Tri6 => (3, true),
            ElementShape::Quad4 => (4, false),
            ElementShape::Quad8 => (4, true),
            shape => return Err(format!("{shape:?} elements cannot be thickened")),
        };
        let mids = if quadratic { corners } else { 0 };
        elements.push(ShellElement {
            corners: element.nodes[..corners].to_vec(),
            mids: element.nodes[corners..corners + mids].to_vec(),
            face: face_of.get(&element.id).copied(),
        });
    }
    if elements.is_empty() {
        return Err("The shell has no elements".into());
    }
    let quadratic = elements.iter().any(|e| !e.mids.is_empty());
    let position = |id: NodeId| shell.node(id).ok_or(format!("Node {id} is missing"));
    orient(&mut elements, &position)?;
    let offsets = node_offsets(&elements, &position)?;

    // The heights of the node levels along the offsets: the layers' bottoms and tops, for
    // quadratic elements also their middles.
    let layers = thicken.layers as usize;
    let bottom = thicken.offset - 0.5 * thicken.thickness;
    let level_count = if quadratic {
        2 * layers + 1
    } else {
        layers + 1
    };
    let step = thicken.thickness / (level_count - 1) as f64;
    let heights: Vec<f64> = (0..level_count).map(|k| bottom + k as f64 * step).collect();
    let per_layer = if quadratic { 2 } else { 1 };

    let mut mesh = FeMesh::default();
    let mut nodes: HashMap<(NodeId, usize), NodeId> = HashMap::new();
    let mut node = |mesh: &mut FeMesh, id: NodeId, level: usize| -> Result<NodeId, String> {
        if let Some(&new) = nodes.get(&(id, level)) {
            return Ok(new);
        }
        let new = NodeId::try_from(nodes.len() + 1).map_err(|_| "Too many nodes".to_string())?;
        let at = add(position(id)?, scale(offsets[&id], heights[level]));
        mesh.set_node(new, at);
        nodes.insert((id, level), new);
        Ok(new)
    };
    let mut part_elements = Part {
        name: part.to_string(),
        elements: Vec::new(),
    };
    for layer in 0..layers {
        let (low, high) = (per_layer * layer, per_layer * (layer + 1));
        for element in &elements {
            let (shape, type_name) = match (element.corners.len(), quadratic) {
                (3, false) => (ElementShape::Wedge6, "C3D6"),
                (3, true) => (ElementShape::Wedge15, "C3D15"),
                (_, false) => (ElementShape::Hex8, "C3D8"),
                (_, true) => (ElementShape::Hex20, "C3D20"),
            };
            let mut ids = Vec::with_capacity(shape.node_count());
            for level in [low, high] {
                for &c in &element.corners {
                    ids.push(node(&mut mesh, c, level)?);
                }
            }
            if quadratic {
                for level in [low, high] {
                    for &m in &element.mids {
                        ids.push(node(&mut mesh, m, level)?);
                    }
                }
                for &c in &element.corners {
                    ids.push(node(&mut mesh, c, low + 1)?);
                }
            }
            let id = ElementId::try_from(mesh.element_count() + 1)
                .map_err(|_| "Too many elements".to_string())?;
            mesh.add_element(Element {
                id,
                type_name: type_name.into(),
                shape,
                nodes: ids,
            })
            .map_err(|e| e.to_string())?;
            part_elements.elements.push(id);
        }
    }
    mesh.parts.push(part_elements);

    // Elements that fold over: the solid is too thick for the shell's curvature or corners.
    for element in mesh.elements() {
        if element.min_jacobian(&mesh).is_some_and(|j| j <= 0.0) {
            let at = mesh.node(element.nodes[0]).unwrap_or_default();
            return Err(format!(
                "The thickened elements fold over near ({:.4}, {:.4}, {:.4}): the thickness \
                 is too large for the curvature or the corners of the shell there",
                at[0], at[1], at[2]
            ));
        }
    }

    // The side of the solid nearest to the shell's surface; at offset 0 the side the
    // normals point to.
    let top = level_count - 1;
    let side = if heights[0].abs() < heights[top].abs() - 1e-9 * thicken.thickness {
        0
    } else {
        top
    };
    mesh.cad = cad_map(&shell.cad, &mesh, &elements, &nodes, side, level_count);
    Ok(mesh)
}

/// Turns the elements so that neighbours' normals point to the same side of the shell, each
/// connected piece to the side most of its area faces as meshed.
fn orient(
    elements: &mut [ShellElement],
    position: &impl Fn(NodeId) -> Result<Point, String>,
) -> Result<(), String> {
    // The elements at each edge, by its sorted corners, with whether they run along it from
    // the smaller node to the larger.
    let mut edges: HashMap<[NodeId; 2], Vec<(usize, bool)>> = HashMap::new();
    for (index, element) in elements.iter().enumerate() {
        let c = &element.corners;
        for k in 0..c.len() {
            let (a, b) = (c[k], c[(k + 1) % c.len()]);
            edges
                .entry([a.min(b), a.max(b)])
                .or_default()
                .push((index, a < b));
        }
    }
    if let Some((edge, _)) = edges.iter().find(|(_, at)| at.len() > 2) {
        let at = position(edge[0])?;
        return Err(format!(
            "More than two faces of the shell meet at an edge near ({:.4}, {:.4}, {:.4}), \
             so it has no inside and outside to thicken to",
            at[0], at[1], at[2]
        ));
    }
    let area = |element: &ShellElement| -> Result<f64, String> {
        Ok(norm(element_normal(&corner_points(element, position)?)))
    };
    let mut flip: Vec<Option<bool>> = vec![None; elements.len()];
    for seed in 0..elements.len() {
        if flip[seed].is_some() {
            continue;
        }
        flip[seed] = Some(false);
        let mut piece = vec![seed];
        let mut queue = VecDeque::from([seed]);
        while let Some(index) = queue.pop_front() {
            let flipped = flip[index] == Some(true);
            let c = elements[index].corners.clone();
            for k in 0..c.len() {
                let (a, b) = (c[k], c[(k + 1) % c.len()]);
                let forward = (a < b) != flipped;
                for &(other, along) in &edges[&[a.min(b), a.max(b)]] {
                    if other == index {
                        continue;
                    }
                    // Neighbours of the same side run along their common edge the other way.
                    let wanted = along == forward;
                    match flip[other] {
                        None => {
                            flip[other] = Some(wanted);
                            piece.push(other);
                            queue.push_back(other);
                        }
                        Some(f) if f != wanted => {
                            let at = position(a)?;
                            return Err(format!(
                                "The shell has no inside and outside near ({:.4}, {:.4}, \
                                 {:.4}), like a Moebius strip, so it cannot be thickened",
                                at[0], at[1], at[2]
                            ));
                        }
                        Some(_) => {}
                    }
                }
            }
        }
        let mut turned = 0.0;
        let mut kept = 0.0;
        for &index in &piece {
            let a = area(&elements[index])?;
            if flip[index] == Some(true) {
                turned += a;
            } else {
                kept += a;
            }
        }
        if turned > kept {
            for &index in &piece {
                flip[index] = flip[index].map(|f| !f);
            }
        }
    }
    for (element, flip) in elements.iter_mut().zip(flip) {
        if flip == Some(true) {
            // Corners 0, n-1, ..., 1 and the midside nodes between them.
            element.corners[1..].reverse();
            element.mids.reverse();
        }
    }
    Ok(())
}

fn corner_points(
    element: &ShellElement,
    position: &impl Fn(NodeId) -> Result<Point, String>,
) -> Result<Vec<Point>, String> {
    element.corners.iter().map(|&c| position(c)).collect()
}

/// The normal of an element by its corners, as long as twice its area (for a triangle) or
/// the product of its diagonals.
fn element_normal(p: &[Point]) -> Point {
    if p.len() == 4 {
        cross(sub(p[2], p[0]), sub(p[3], p[1]))
    } else {
        cross(sub(p[1], p[0]), sub(p[2], p[0]))
    }
}

fn unit(a: Point) -> Option<Point> {
    let length = norm(a);
    (length > 0.0 && length.is_finite()).then(|| scale(a, 1.0 / length))
}

/// The direction and distance each node moves per unit of height through the thickness: the
/// normal inside a face, where faces meet at an angle the point where the faces offset by 1
/// meet. Midside nodes move as the middle of their corners.
fn node_offsets(
    elements: &[ShellElement],
    position: &impl Fn(NodeId) -> Result<Point, String>,
) -> Result<HashMap<NodeId, Point>, String> {
    // The normal of each CAD face at each node, weighted by the elements' angles there.
    let mut normals: BTreeMap<NodeId, BTreeMap<Option<i32>, Point>> = BTreeMap::new();
    for element in elements {
        let p = corner_points(element, position)?;
        let Some(n) = unit(element_normal(&p)) else {
            continue;
        };
        let count = p.len();
        for k in 0..count {
            let (before, after) = (p[(k + count - 1) % count], p[(k + 1) % count]);
            let (u, v) = (unit(sub(before, p[k])), unit(sub(after, p[k])));
            let angle = match (u, v) {
                (Some(u), Some(v)) => dot(u, v).clamp(-1.0, 1.0).acos(),
                _ => 0.0,
            };
            let sum = (normals.entry(element.corners[k]).or_default())
                .entry(element.face)
                .or_insert([0.0; 3]);
            *sum = add(*sum, scale(n, angle));
        }
    }
    let same = SAME_NORMAL_DEGREES.to_radians().cos();
    let mut offsets = HashMap::new();
    for (&node, faces) in &normals {
        // Faces that meet smoothly count as one.
        let mut groups: Vec<Point> = Vec::new();
        for n in faces.values().filter_map(|&n| unit(n)) {
            match (groups.iter_mut()).find(|g| unit(**g).is_some_and(|g| dot(g, n) >= same)) {
                Some(group) => *group = add(*group, n),
                None => groups.push(n),
            }
        }
        let groups: Vec<Point> = groups.into_iter().filter_map(unit).collect();
        let offset = match groups.as_slice() {
            [] => {
                let at = position(node)?;
                return Err(format!(
                    "The shell has no normal at ({:.4}, {:.4}, {:.4})",
                    at[0], at[1], at[2]
                ));
            }
            [n] => *n,
            [a, b] => scale(add(*a, *b), 1.0 / (1.0 + dot(*a, *b)).max(1e-12)),
            more => meeting_point(more),
        };
        let length = norm(offset);
        let offset = if length.is_finite() && length <= LONGEST_OFFSET {
            offset
        } else {
            // Faces folded back onto each other: their mean normal, if they have one.
            let mean = groups.iter().fold([0.0; 3], |s, &n| add(s, n));
            match unit(mean) {
                Some(mean) if length.is_finite() => scale(mean, LONGEST_OFFSET.min(length)),
                Some(mean) => mean,
                None => {
                    let at = position(node)?;
                    return Err(format!(
                        "Faces of the shell fold back onto each other at ({:.4}, {:.4}, \
                         {:.4}), so it cannot be thickened there",
                        at[0], at[1], at[2]
                    ));
                }
            }
        };
        offsets.insert(node, offset);
    }
    for element in elements {
        let count = element.corners.len();
        for (k, &mid) in element.mids.iter().enumerate() {
            let (a, b) = (element.corners[k], element.corners[(k + 1) % count]);
            let (Some(&va), Some(&vb)) = (offsets.get(&a), offsets.get(&b)) else {
                continue;
            };
            // The direction between the two, as long as they are on average.
            let length = 0.5 * (norm(va) + norm(vb));
            let middle = unit(add(va, vb)).map_or(scale(add(va, vb), 0.5), |u| scale(u, length));
            offsets.insert(mid, middle);
        }
    }
    Ok(offsets)
}

/// The point where the planes `n · x = 1` meet, of three or more unit normals; the least
/// squares point nearest to the origin if they do not meet in one.
fn meeting_point(normals: &[Point]) -> Point {
    let mut a = [[0.0; 3]; 3];
    let mut b = [0.0; 3];
    for n in normals {
        for i in 0..3 {
            b[i] += n[i];
            for j in 0..3 {
                a[i][j] += n[i] * n[j];
            }
        }
    }
    // A little regularisation picks the shortest of several solutions.
    let trace = a[0][0] + a[1][1] + a[2][2];
    for (i, row) in a.iter_mut().enumerate() {
        row[i] += 1e-9 * trace;
    }
    let det = |m: &[[f64; 3]; 3]| dot(m[0], cross(m[1], m[2]));
    let d = det(&a);
    if d.abs() < f64::MIN_POSITIVE {
        return [f64::INFINITY; 3];
    }
    // Cramer's rule; `a` is symmetric, so its columns are its rows.
    let mut x = [0.0; 3];
    for (k, value) in x.iter_mut().enumerate() {
        let mut m = a;
        for (row, &bi) in m.iter_mut().zip(&b) {
            row[k] = bi;
        }
        *value = det(&m) / d;
    }
    x
}

/// Where the shell's faces, edges and vertices lie in the solid: a face on the side of the
/// solid at node level `side`, an edge or a vertex through the whole thickness.
fn cad_map(
    shell: &CadMap,
    mesh: &FeMesh,
    elements: &[ShellElement],
    nodes: &HashMap<(NodeId, usize), NodeId>,
    side: usize,
    level_count: usize,
) -> CadMap {
    let mut solid_faces: HashMap<Vec<NodeId>, (ElementId, u8)> = HashMap::new();
    for element in mesh.elements() {
        for (k, face) in element.faces().iter().enumerate() {
            let mut corners: Vec<NodeId> = face.corners.iter().map(|&i| element.nodes[i]).collect();
            corners.sort_unstable();
            solid_faces.insert(corners, (element.id, (k + 1) as u8));
        }
    }
    let at = |id: NodeId, level: usize| nodes.get(&(id, level)).copied();
    let column = |id: NodeId| (0..level_count).filter_map(move |level| at(id, level));
    let mut map = CadMap::default();
    for element in elements {
        let Some(face) = element.face else {
            continue;
        };
        let corners: Option<Vec<NodeId>> = element.corners.iter().map(|&c| at(c, side)).collect();
        if let Some(mut corners) = corners {
            corners.sort_unstable();
            if let Some(&found) = solid_faces.get(&corners) {
                map.faces.entry(face).or_default().push(found);
            }
        }
        let entry = map.nodes.entry(CadEntity::Face(face)).or_default();
        entry.extend((element.corners.iter().chain(&element.mids)).filter_map(|&n| at(n, side)));
    }
    for (&entity, ids) in &shell.nodes {
        if matches!(entity, CadEntity::Edge(_) | CadEntity::Vertex(_)) {
            map.nodes
                .insert(entity, ids.iter().flat_map(|&n| column(n)).collect());
        }
    }
    for (&edge, segments) in &shell.segments {
        let lines = (segments.iter())
            .filter_map(|&[a, b]| Some([at(a, side)?, at(b, side)?]))
            .collect();
        map.segments.insert(edge, lines);
    }
    for ids in map.nodes.values_mut() {
        let unique: BTreeSet<NodeId> = ids.iter().copied().collect();
        *ids = unique.into_iter().collect();
    }
    map
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An L of two square faces of side 2 meeting at x = 0: face 1 in the plane z = 0 at
    /// x from 0 to 2, face 2 in the plane x = 0 at z from 0 to 2, each of 2 x 2 quadrilaterals
    /// with normals pointing to +z and +x, the inside of the L. `quadratic`: Quad8 elements.
    /// The elements of face 2 are turned the other way when `mixed`.
    fn l_shell(quadratic: bool, mixed: bool) -> FeMesh {
        let mut mesh = FeMesh::default();
        let n = if quadratic { 5 } else { 3 };
        let step = 2.0 / (n - 1) as f64;
        // Node (i, j) of face 1 at (i, j, 0), of face 2 at (0, j, i); i = 0 is shared.
        let id = |face: usize, i: usize, j: usize| -> NodeId {
            if i == 0 {
                (1 + j) as NodeId
            } else {
                (1 + n + (face - 1) * n * (n - 1) + (i - 1) * n + j) as NodeId
            }
        };
        for face in 1..=2 {
            for i in 0..n {
                for j in 0..n {
                    let (a, b) = (i as f64 * step, j as f64 * step);
                    let at = if face == 1 { [a, b, 0.0] } else { [0.0, b, a] };
                    mesh.set_node(id(face, i, j), at);
                }
            }
        }
        let mut cad = CadMap::default();
        let mut elements = Vec::new();
        let s = if quadratic { 2 } else { 1 };
        for face in 1..=2 {
            for i in (0..n - 1).step_by(s) {
                for j in (0..n - 1).step_by(s) {
                    // Counterclockwise about +z for face 1 in (x, y), about +x for face 2 in
                    // (y, z): (i, j) -> (i + s, j) ...
                    let mut corners = if face == 1 {
                        vec![(i, j), (i + s, j), (i + s, j + s), (i, j + s)]
                    } else {
                        vec![(i, j), (i, j + s), (i + s, j + s), (i + s, j)]
                    };
                    if face == 2 && mixed {
                        corners[1..].reverse();
                    }
                    let mut nodes: Vec<NodeId> =
                        corners.iter().map(|&(a, b)| id(face, a, b)).collect();
                    let mut shape = ElementShape::Quad4;
                    let mut type_name = "S4";
                    if quadratic {
                        for k in 0..4 {
                            let (a, b) = (corners[k], corners[(k + 1) % 4]);
                            nodes.push(id(face, (a.0 + b.0) / 2, (a.1 + b.1) / 2));
                        }
                        (shape, type_name) = (ElementShape::Quad8, "S8");
                    }
                    let element = ElementId::try_from(mesh.element_count() + 1).unwrap();
                    mesh.add_element(Element {
                        id: element,
                        type_name: type_name.into(),
                        shape,
                        nodes,
                    })
                    .unwrap();
                    elements.push(element);
                    cad.faces.entry(face as i32).or_default().push((element, 1));
                }
            }
        }
        // The shared edge 1 along y at x = z = 0 and its vertex 1 at the origin.
        cad.nodes
            .insert(CadEntity::Edge(1), (0..n).map(|j| id(1, 0, j)).collect());
        cad.segments.insert(
            1,
            (0..n - 1)
                .step_by(s)
                .map(|j| [id(1, 0, j), id(1, 0, j + s)])
                .collect(),
        );
        cad.nodes.insert(CadEntity::Vertex(1), vec![id(1, 0, 0)]);
        mesh.parts.push(Part {
            name: "SHELL-1".into(),
            elements,
        });
        mesh.cad = cad;
        mesh
    }

    /// The volume of linear hexahedra with planar faces: six tetrahedra about the diagonal
    /// from corner 0 to corner 6.
    fn volume(mesh: &FeMesh) -> f64 {
        let ring = [1, 2, 3, 7, 4, 5, 1];
        mesh.elements()
            .iter()
            .map(|e| {
                let p = |k: usize| mesh.node(e.nodes[k]).unwrap();
                let (a, d) = (p(0), p(6));
                (ring.windows(2))
                    .map(|w| dot(sub(p(w[0]), a), cross(sub(p(w[1]), a), sub(d, a))) / 6.0)
                    .sum::<f64>()
            })
            .sum()
    }

    fn item(thickness: f64, layers: u32, offset: f64) -> ThickenShellMesh {
        ThickenShellMesh {
            parts: vec!["SHELL-1".into()],
            thickness,
            layers,
            offset,
        }
    }

    #[test]
    fn an_l_becomes_hexahedra_of_its_thickness_on_both_legs() {
        for mixed in [false, true] {
            let shell = l_shell(false, mixed);
            let solid = thicken(&shell, "SHELL-1", &item(0.2, 2, 0.0)).unwrap();
            assert_eq!(solid.element_count(), 8 * 2);
            assert!(solid.elements().iter().all(|e| e.type_name == "C3D8"));
            assert_eq!(solid.node_count(), 15 * 3);
            // The corner keeps the thickness on both legs: it moves by ±0.1 in x and z.
            let corner: Vec<[f64; 3]> = (solid.node_ids().iter())
                .filter_map(|&n| solid.node(n))
                .filter(|p| p[1] == 0.0 && (p[0] - p[2]).abs() < 1e-12 && p[0].abs() <= 0.1 + 1e-12)
                .collect();
            assert_eq!(corner.len(), 3, "{corner:?}");
            // The volume of two legs 2 x 2 x 0.2, the overlap of the corner counted once:
            // the inner leg is 1.9 long, the outer 2.1, on average 2.
            assert!(
                (volume(&solid) - 2.0 * 2.0 * 2.0 * 0.2).abs() < 1e-9,
                "{}",
                volume(&solid)
            );
            assert!(
                solid
                    .elements()
                    .iter()
                    .all(|e| e.min_jacobian(&solid).unwrap() > 0.0)
            );
        }
    }

    #[test]
    fn the_offset_moves_the_solid_along_the_normals() {
        let shell = l_shell(false, false);
        let solid = thicken(&shell, "SHELL-1", &item(0.2, 1, 0.1)).unwrap();
        // On face 1 the solid reaches from z = 0 to 0.2, towards the inside of the L.
        let mut far: Vec<f64> = (solid.node_ids().iter())
            .filter_map(|&n| solid.node(n))
            .filter(|p| (p[0] - 2.0).abs() < 1e-12 && p[1] == 0.0)
            .map(|p| p[2])
            .collect();
        far.sort_by(f64::total_cmp);
        assert_eq!(far.len(), 2);
        assert!(
            far[0].abs() < 1e-12 && (far[1] - 0.2).abs() < 1e-12,
            "{far:?}"
        );
        // The faces of the geometry are the side on the shell, at z = 0 for face 1.
        let faces = &solid.cad.faces[&1];
        assert_eq!(faces.len(), 4);
        for &n in &solid.cad.nodes[&CadEntity::Face(1)] {
            assert!(solid.node(n).unwrap()[2].abs() < 1e-12);
        }
        // The edge and the vertex reach through the thickness.
        assert_eq!(solid.cad.nodes[&CadEntity::Edge(1)].len(), 3 * 2);
        assert_eq!(solid.cad.nodes[&CadEntity::Vertex(1)].len(), 2);
        assert_eq!(solid.cad.segments[&1].len(), 2);
    }

    #[test]
    fn quadratic_shells_become_twenty_node_hexahedra() {
        let shell = l_shell(true, false);
        let solid = thicken(&shell, "SHELL-1", &item(0.2, 1, 0.0)).unwrap();
        assert_eq!(solid.element_count(), 8);
        assert!(solid.elements().iter().all(|e| e.type_name == "C3D20"));
        assert!(
            solid
                .elements()
                .iter()
                .all(|e| e.min_jacobian(&solid).unwrap() > 0.0)
        );
        // 15 corner nodes on three levels, 22 midside nodes on the two sides; the grid's
        // element centres are no nodes of the elements.
        assert_eq!(solid.node_count(), 15 * 3 + 22 * 2);
    }

    #[test]
    fn too_thick_folds_and_is_refused() {
        let shell = l_shell(false, false);
        // The inner side of the corner would lie beyond the far end of the legs.
        let error = thicken(&shell, "SHELL-1", &item(5.0, 1, 0.0)).unwrap_err();
        assert!(error.contains("fold over"), "{error}");
        assert!(thicken(&shell, "SHELL-1", &item(0.0, 1, 0.0)).is_err());
        assert!(thicken(&shell, "SHELL-1", &item(0.1, 0, 0.0)).is_err());
    }
}
