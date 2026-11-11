//! Extrude Mesh: a part that is a straight extrusion of some of its faces is
//! meshed on those faces with Gmsh, mostly with quadrilaterals, and the face mesh is swept
//! through the part in layers. Quadrilaterals become hexahedra, triangles wedges.
//!
//! The sweep is built here rather than by Gmsh, so that the geometry stays as it is: every
//! face, edge and vertex of the part keeps its tag, and the mesh records where each lies.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use plx_mesh::{CadMap, Element, ElementId, ElementShape, FeMesh, NodeId, Part};
use plx_model::{ExtrudeMesh, Geometry, Recombination};

use crate::gmsh::{Gmsh, GmshError};
use crate::{
    GeneratedMesh, LINE2, LINE3, MeshOptions, POINT, QUAD4, QUAD8, TRI3, TRI6, generate_faces,
    node_coords,
};

pub(crate) type Point = [f64; 3];

pub(crate) fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(crate) fn add(a: Point, b: Point) -> Point {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub(crate) fn scale(a: Point, s: f64) -> Point {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub(crate) fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub(crate) fn cross(a: Point, b: Point) -> Point {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub(crate) fn norm(a: Point) -> f64 {
    dot(a, a).sqrt()
}

/// How the faces, edges and vertices of a part that is an extrusion relate: each source
/// entity has its copy at the other end, each boundary edge of the source sweeps a side face
/// and each boundary vertex a lateral edge.
#[derive(Debug)]
struct Sweep {
    /// The extrusion, from the source to the target end.
    vector: Point,
    source: Vec<i32>,
    /// Target face of each source face.
    target_faces: BTreeMap<i32, i32>,
    /// Target edge of each edge of the source faces.
    target_edges: BTreeMap<i32, i32>,
    /// Target vertex of each vertex of the source faces.
    target_vertices: BTreeMap<i32, i32>,
    /// Side faces swept by each boundary edge of the source.
    side_faces: BTreeMap<i32, i32>,
    /// Lateral edge swept by each boundary vertex of the source.
    lateral_edges: BTreeMap<i32, i32>,
}

/// The part's topology, read before the mesh is made.
pub(crate) struct Topology {
    pub faces: Vec<i32>,
    pub edges: BTreeSet<i32>,
    /// Edges of each face.
    pub face_edges: BTreeMap<i32, Vec<i32>>,
    /// Vertices of each edge, one for a closed edge.
    pub edge_vertices: BTreeMap<i32, Vec<i32>>,
    pub points: BTreeMap<i32, Point>,
    pub boxes: BTreeMap<(i32, i32), (Point, Point)>,
    pub lengths: BTreeMap<i32, f64>,
    /// Size of the part, for tolerances.
    pub size: f64,
}

impl Topology {
    pub fn read(gmsh: &Gmsh, volume: i32) -> Result<Self, GmshError> {
        let faces = gmsh.adjacencies(3, volume)?.1;
        let mut face_edges = BTreeMap::new();
        let mut edges = BTreeSet::new();
        for &face in &faces {
            let own = gmsh.adjacencies(2, face)?.1;
            edges.extend(own.iter().copied());
            face_edges.insert(face, own);
        }
        let mut edge_vertices = BTreeMap::new();
        let mut points = BTreeMap::new();
        let mut lengths = BTreeMap::new();
        let mut boxes = BTreeMap::new();
        for &edge in &edges {
            let vertices = gmsh.adjacencies(1, edge)?.1;
            for &v in &vertices {
                if let std::collections::btree_map::Entry::Vacant(entry) = points.entry(v) {
                    entry.insert(gmsh.point(v)?);
                }
            }
            edge_vertices.insert(edge, vertices);
            lengths.insert(edge, gmsh.mass(1, edge)?);
            boxes.insert((1, edge), gmsh.entity_bounding_box(1, edge)?);
        }
        for &face in &faces {
            boxes.insert((2, face), gmsh.entity_bounding_box(2, face)?);
        }
        let (min, max) = gmsh.entity_bounding_box(3, volume)?;
        Ok(Self {
            faces,
            edges,
            face_edges,
            edge_vertices,
            points,
            boxes,
            lengths,
            size: norm(sub(max, min)).max(f64::MIN_POSITIVE),
        })
    }

    fn tolerance(&self) -> f64 {
        1e-5 * self.size
    }

    fn same_point(&self, a: Point, b: Point) -> bool {
        norm(sub(a, b)) <= self.tolerance()
    }

    /// Whether the entity's bounding box, shifted by the vector, is the other's.
    fn shifted(&self, entity: (i32, i32), other: (i32, i32), vector: Point) -> bool {
        let (Some(a), Some(b)) = (self.boxes.get(&entity), self.boxes.get(&other)) else {
            return false;
        };
        // OpenCASCADE's boxes of curved shapes are not tight; their tolerance is a little
        // looser.
        let tolerance = 1e-3 * self.size;
        (0..3).all(|k| {
            (a.0[k] + vector[k] - b.0[k]).abs() <= tolerance
                && (a.1[k] + vector[k] - b.1[k]).abs() <= tolerance
        })
    }

    /// How the part is an extrusion of the source faces, or why it is none.
    fn sweep(&self, source: &[i32]) -> Result<Sweep, String> {
        let source_set: BTreeSet<i32> = source.iter().copied().collect();
        let mut counts: BTreeMap<i32, usize> = BTreeMap::new();
        for face in source {
            for &edge in self.face_edges.get(face).into_iter().flatten() {
                *counts.entry(edge).or_default() += 1;
            }
        }
        let source_edges: BTreeSet<i32> = counts.keys().copied().collect();
        let boundary: Vec<i32> = (counts.iter())
            .filter(|&(_, &n)| n == 1)
            .map(|(&e, _)| e)
            .collect();
        if boundary.is_empty() {
            return Err("the source faces have no boundary".into());
        }
        let vertices_of = |edges: &mut dyn Iterator<Item = &i32>| -> BTreeSet<i32> {
            edges
                .flat_map(|e| self.edge_vertices.get(e).into_iter().flatten().copied())
                .collect()
        };
        let source_vertices = vertices_of(&mut source_edges.iter());
        let boundary_vertices = vertices_of(&mut boundary.iter());
        // The lateral edges: straight, from each boundary vertex, all the same vector.
        let laterals = |vertex: i32| -> Vec<(i32, Point)> {
            let start = self.points[&vertex];
            (self.edges.iter())
                .filter(|e| !source_edges.contains(e))
                .filter_map(|&edge| {
                    let ends = self.edge_vertices.get(&edge)?;
                    let other = match ends[..] {
                        [a, b] if a == vertex => b,
                        [a, b] if b == vertex => a,
                        _ => return None,
                    };
                    let vector = sub(self.points[&other], start);
                    let straight = (self.lengths[&edge] - norm(vector)).abs() <= self.tolerance();
                    straight.then_some((edge, vector))
                })
                .collect()
        };
        let first = *boundary_vertices
            .first()
            .ok_or("the source faces have no vertices")?;
        let mut found = None;
        'candidates: for (_, vector) in laterals(first) {
            if norm(vector) <= self.tolerance() {
                continue;
            }
            let mut lateral_edges = BTreeMap::new();
            for &vertex in &boundary_vertices {
                let Some((edge, _)) =
                    (laterals(vertex).into_iter()).find(|(_, v)| self.same_point(*v, vector))
                else {
                    continue 'candidates;
                };
                lateral_edges.insert(vertex, edge);
            }
            found = Some((vector, lateral_edges));
            break;
        }
        let (vector, lateral_edges) =
            found.ok_or("no straight edges of the same length and direction leave the source")?;
        let lateral_set: BTreeSet<i32> = lateral_edges.values().copied().collect();

        let mut target_vertices = BTreeMap::new();
        for &vertex in &source_vertices {
            let at = add(self.points[&vertex], vector);
            let (&target, _) = (self.points.iter())
                .find(|&(_, &p)| self.same_point(p, at))
                .ok_or_else(|| format!("vertex {vertex} has no copy at the other end"))?;
            target_vertices.insert(vertex, target);
        }
        let mut target_edges = BTreeMap::new();
        for &edge in &source_edges {
            let ends: BTreeSet<i32> = self.edge_vertices[&edge]
                .iter()
                .map(|v| target_vertices[v])
                .collect();
            let target = (self.edges.iter())
                .filter(|e| !source_edges.contains(e) && !lateral_set.contains(e))
                .find(|&&e| {
                    let own: BTreeSet<i32> = self.edge_vertices[&e].iter().copied().collect();
                    own == ends && self.shifted((1, edge), (1, e), vector)
                })
                .ok_or_else(|| format!("curve {edge} has no copy at the other end"))?;
            target_edges.insert(edge, *target);
        }
        let target_edge_set: BTreeSet<i32> = target_edges.values().copied().collect();
        let mut side_faces = BTreeMap::new();
        for &edge in &boundary {
            let sides: Vec<i32> = (self.faces.iter())
                .filter(|f| !source_set.contains(f))
                .filter(|f| self.face_edges[f].contains(&edge))
                .copied()
                .collect();
            match sides[..] {
                [side] => side_faces.insert(edge, side),
                _ => return Err(format!("curve {edge} does not bound one side face")),
            };
        }
        let side_set: BTreeSet<i32> = side_faces.values().copied().collect();
        // Side faces lie between the ends: bounded only by source, target and lateral edges.
        for side in &side_set {
            if let Some(edge) = self.face_edges[side].iter().find(|e| {
                !source_edges.contains(e)
                    && !target_edge_set.contains(e)
                    && !lateral_set.contains(e)
            }) {
                return Err(format!(
                    "surface {side} has curve {edge} across the extrusion"
                ));
            }
        }
        let rest: Vec<i32> = (self.faces.iter())
            .filter(|f| !source_set.contains(f) && !side_set.contains(f))
            .copied()
            .collect();
        let mut target_faces = BTreeMap::new();
        for &face in source {
            let target = (rest.iter())
                .filter(|t| !target_faces.values().any(|v| v == *t))
                .find(|&&t| self.shifted((2, face), (2, t), vector))
                .ok_or_else(|| format!("surface {face} has no copy at the other end"))?;
            target_faces.insert(face, *target);
        }
        if rest.len() != source.len() {
            return Err("the part has faces that are neither ends nor sides".into());
        }
        Ok(Sweep {
            vector,
            source: source.to_vec(),
            target_faces,
            target_edges,
            target_vertices,
            side_faces,
            lateral_edges,
        })
    }
}

/// The source faces for an item: its faces on the part, else the first face, by most edges,
/// that the part is an extrusion of.
fn find_sweep(topology: &Topology, extrude: &ExtrudeMesh, part: &str) -> Result<Sweep, String> {
    let error = |reason: String| format!("{part}: {reason}");
    let chosen: Vec<i32> = (extrude.faces.iter())
        .filter(|f| topology.faces.contains(f))
        .copied()
        .collect();
    if !chosen.is_empty() {
        return topology.sweep(&chosen).map_err(|reason| {
            error(format!(
                "The part is not an extrusion of the selected faces: {reason}. Mesh it with \
                 tetrahedra or select the faces at one end."
            ))
        });
    }
    if !extrude.faces.is_empty() {
        return Err(error(
            "None of the selected source faces lies on the part".into(),
        ));
    }
    let mut candidates = topology.faces.clone();
    candidates.sort_by_key(|f| (std::cmp::Reverse(topology.face_edges[f].len()), *f));
    (candidates.iter())
        .find_map(|&face| topology.sweep(&[face]).ok())
        .ok_or_else(|| {
            error(
                "No face was found that the part is a straight extrusion of. Select the source \
                 faces or mesh the part with tetrahedra."
                    .into(),
            )
        })
}

/// One element of the source mesh: its corner and midside nodes as Gmsh tags, turned so
/// that its normal points along the extrusion.
pub(crate) struct SourceElement {
    pub face: i32,
    pub corners: Vec<usize>,
    pub mids: Vec<usize>,
}

/// How the source mesh moves through the part: straight, or about an axis.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Motion {
    Translate(Point),
    /// About the unit axis through the origin, by the angle in radians.
    Rotate {
        origin: Point,
        axis: Point,
        angle: f64,
    },
}

impl Motion {
    /// Where a point of the source is at a fraction of the motion.
    pub fn at(&self, p: Point, t: f64) -> Point {
        match *self {
            Motion::Translate(vector) => add(p, scale(vector, t)),
            Motion::Rotate {
                origin,
                axis,
                angle,
            } => {
                let (sin, cos) = (t * angle).sin_cos();
                let v = sub(p, origin);
                let rotated = add(
                    add(scale(v, cos), scale(cross(axis, v), sin)),
                    scale(axis, dot(axis, v) * (1.0 - cos)),
                );
                add(origin, rotated)
            }
        }
    }

    /// The direction a point of the source moves in.
    pub fn direction(&self, p: Point) -> Point {
        match *self {
            Motion::Translate(vector) => vector,
            Motion::Rotate { origin, axis, .. } => cross(axis, sub(p, origin)),
        }
    }

    /// Whether the motion ends where it began: a full revolution.
    pub fn closes(&self) -> bool {
        matches!(*self, Motion::Rotate { angle, .. } if (angle - std::f64::consts::TAU).abs() < 1e-9)
    }
}

/// The elements of the source mesh on the faces, turned so that their normals point along
/// the motion, and the normal of a planar source.
pub(crate) fn source_elements(
    gmsh: &Gmsh,
    faces: &[i32],
    quadratic: bool,
    coords: &HashMap<usize, Point>,
    motion: &Motion,
) -> Result<(Vec<SourceElement>, Option<Point>), GmshError> {
    let position = |tag: usize| {
        coords
            .get(&tag)
            .copied()
            .ok_or_else(|| GmshError::Other(format!("Node {tag} is missing")))
    };
    let types: [(i32, usize); 2] = if quadratic {
        [(TRI6, 3), (QUAD8, 4)]
    } else {
        [(TRI3, 3), (QUAD4, 4)]
    };
    let mut elements = Vec::new();
    let mut planar_normal: Option<Point> = None;
    let mut planar = true;
    for &face in faces {
        for (gmsh_type, corners) in types {
            let n = if quadratic { 2 * corners } else { corners };
            let (_, tags) = gmsh.elements(gmsh_type, face)?;
            for element in tags.chunks_exact(n) {
                let mut c = element[..corners].to_vec();
                let mut m = element[corners..].to_vec();
                let p: Vec<Point> = c.iter().map(|&t| position(t)).collect::<Result<_, _>>()?;
                let normal = if corners == 4 {
                    cross(sub(p[2], p[0]), sub(p[3], p[1]))
                } else {
                    cross(sub(p[1], p[0]), sub(p[2], p[0]))
                };
                let centre = scale(
                    p.iter().fold([0.0; 3], |a, &b| add(a, b)),
                    1.0 / p.len() as f64,
                );
                if dot(normal, motion.direction(centre)) < 0.0 {
                    // Reversed: corners 0, n-1, ..., 1 and the midside nodes between them.
                    c[1..].reverse();
                    m.reverse();
                }
                let length = norm(normal);
                if length > 0.0 {
                    let unit = scale(normal, 1.0 / length);
                    match planar_normal {
                        None => planar_normal = Some(unit),
                        Some(first) => planar &= norm(cross(first, unit)) < 1e-6,
                    }
                }
                elements.push(SourceElement {
                    face,
                    corners: c,
                    mids: m,
                });
            }
        }
    }
    Ok((elements, planar_normal.filter(|_| planar)))
}

/// Sweeps the source elements through the levels: hexahedra from quadrilaterals, wedges
/// from triangles, one part.
pub(crate) fn sweep_mesh<'a>(
    part: &str,
    coords: &'a HashMap<usize, Point>,
    elements: &[SourceElement],
    motion: Motion,
    levels: &[f64],
    quadratic: bool,
) -> Result<(FeMesh, Builder<'a>), GmshError> {
    let mut builder = Builder::new(coords, motion, levels, quadratic);
    let mut mesh = FeMesh::default();
    let mut part_elements = Part {
        bodies: Vec::new(),
        name: part.to_string(),
        elements: Vec::new(),
    };
    for layer in 0..levels.len() {
        for element in elements {
            let (shape, type_name) = match (element.corners.len(), quadratic) {
                (3, false) => (ElementShape::Wedge6, "C3D6"),
                (3, true) => (ElementShape::Wedge15, "C3D15"),
                (_, false) => (ElementShape::Hex8, "C3D8"),
                (_, true) => (ElementShape::Hex20, "C3D20"),
            };
            let (bottom, top) = builder.layer_levels(layer);
            let mut nodes = Vec::with_capacity(shape.node_count());
            for &c in &element.corners {
                nodes.push(builder.node(&mut mesh, c, bottom));
            }
            for &c in &element.corners {
                nodes.push(builder.node(&mut mesh, c, top));
            }
            if quadratic {
                for &m in &element.mids {
                    nodes.push(builder.node(&mut mesh, m, bottom));
                }
                for &m in &element.mids {
                    nodes.push(builder.node(&mut mesh, m, top));
                }
                for &c in &element.corners {
                    nodes.push(builder.node(&mut mesh, c, bottom + 1));
                }
            }
            let id = ElementId::try_from(mesh.element_count() + 1)
                .map_err(|_| GmshError::Other("too many elements".into()))?;
            mesh.add_element(Element {
                id,
                type_name: type_name.into(),
                shape,
                nodes,
            })
            .map_err(|e| GmshError::Other(e.to_string()))?;
            part_elements.elements.push(id);
        }
    }
    mesh.parts.push(part_elements);
    Ok((mesh, builder))
}

/// Meshes a solid as an extrusion of its source faces. The other solids are gone from the
/// Gmsh model already. The inner error says why the part is no extrusion of the source; the
/// Gmsh model may then have lost the solid.
pub(crate) fn extrude_part_mesh(
    gmsh: &Gmsh,
    geometry: &Geometry,
    part: &str,
    volume: i32,
    extrude: &ExtrudeMesh,
) -> Result<Result<GeneratedMesh, String>, GmshError> {
    let topology = Topology::read(gmsh, volume)?;
    let sweep = match find_sweep(&topology, extrude, part) {
        Ok(sweep) => sweep,
        Err(reason) => return Ok(Err(reason)),
    };
    let part_volume = gmsh.mass(3, volume)?;
    let source_area: f64 = (sweep.source.iter())
        .map(|&f| gmsh.mass(2, f))
        .sum::<Result<f64, _>>()?;

    // Only the source faces are meshed.
    let others: Vec<(i32, i32)> = std::iter::once((3, volume))
        .chain(
            (topology.faces.iter())
                .filter(|f| !sweep.source.contains(f))
                .map(|&f| (2, f)),
        )
        .collect();
    gmsh.remove_only(&others)?;
    let mut options = MeshOptions::of_part(geometry, part);
    options.algorithms.0 = extrude.algorithm_2d;
    options.quads = extrude.recombination != Recombination::None;
    options.recombination = match extrude.recombination {
        Recombination::None | Recombination::Simple => 0.0,
        Recombination::Blossom => 1.0,
        Recombination::SimpleFullQuad => 2.0,
        Recombination::BlossomFullQuad => 3.0,
    };
    let mut notes = Vec::new();
    let sized = sweep.source.iter().copied().collect();
    generate_faces(
        gmsh,
        geometry,
        &mut options,
        &sweep.source,
        &sized,
        &mut notes,
    )?;
    let quadratic = options.order == 2;
    let coords = node_coords(gmsh)?;
    let motion = Motion::Translate(sweep.vector);
    let (elements, planar_normal) =
        source_elements(gmsh, &sweep.source, quadratic, &coords, &motion)?;
    if elements.is_empty() {
        return Err(GmshError::Other(format!(
            "{part}: Gmsh created no elements on the source faces"
        )));
    }
    // A planar source is checked once more: the part's volume is its area times the height.
    if let Some(normal) = planar_normal {
        let expected = source_area * dot(normal, sweep.vector).abs();
        if (expected - part_volume).abs() > 1e-3 * part_volume.abs().max(f64::MIN_POSITIVE) {
            return Ok(Err(format!(
                "{part}: The part is not a straight extrusion of its source faces (volume \
                 {part_volume:.6e}, area times height {expected:.6e}). Mesh it with tetrahedra."
            )));
        }
    }

    let max_size = geometry.parameters(part).max_size;
    let levels = (extrude.layers.levels(max_size, norm(sweep.vector)))
        .map_err(|reason| GmshError::Other(format!("{part}: {reason}")))?;
    let (mut mesh, builder) = sweep_mesh(part, &coords, &elements, motion, &levels, quadratic)?;
    mesh.cad = builder.cad_map(gmsh, &mesh, &elements, &sweep)?;
    let mut warnings = gmsh.warnings()?;
    warnings.extend(notes.into_iter().map(|note| format!("{part}: {note}")));
    Ok(Ok(GeneratedMesh { mesh, warnings }))
}

/// Makes the nodes of the swept mesh, numbered from 1, level by level.
pub(crate) struct Builder<'a> {
    coords: &'a HashMap<usize, Point>,
    motion: Motion,
    /// Height of each node level as a fraction of the extrusion: the source, then for
    /// quadratic elements the middle and the top of each layer, for linear ones its top.
    heights: Vec<f64>,
    quadratic: bool,
    nodes: HashMap<(usize, usize), NodeId>,
}

impl<'a> Builder<'a> {
    fn new(
        coords: &'a HashMap<usize, Point>,
        motion: Motion,
        levels: &[f64],
        quadratic: bool,
    ) -> Self {
        let mut heights = vec![0.0];
        for &level in levels {
            if quadratic {
                let below = *heights.last().unwrap_or(&0.0);
                heights.push(0.5 * (below + level));
            }
            heights.push(level);
        }
        Self {
            coords,
            motion,
            heights,
            quadratic,
            nodes: HashMap::new(),
        }
    }

    /// The node levels of the bottom and the top of a layer.
    fn layer_levels(&self, layer: usize) -> (usize, usize) {
        if self.quadratic {
            (2 * layer, 2 * layer + 2)
        } else {
            (layer, layer + 1)
        }
    }

    fn top(&self) -> usize {
        self.heights.len() - 1
    }

    /// Whether a level is the bottom or top of a layer, not the middle.
    fn is_layer_level(&self, level: usize) -> bool {
        !self.quadratic || level.is_multiple_of(2)
    }

    /// The node at a source node's place on a level, made on first use.
    fn node(&mut self, mesh: &mut FeMesh, tag: usize, level: usize) -> NodeId {
        // A full revolution ends on its source.
        let level = if level == self.top() && self.motion.closes() {
            0
        } else {
            level
        };
        if let Some(&id) = self.nodes.get(&(tag, level)) {
            return id;
        }
        let id = NodeId::try_from(self.nodes.len() + 1).unwrap_or(NodeId::MAX);
        let base = self.coords.get(&tag).copied().unwrap_or_default();
        mesh.set_node(id, self.motion.at(base, self.heights[level]));
        self.nodes.insert((tag, level), id);
        id
    }

    /// The node if it was made.
    fn existing(&self, tag: usize, level: usize) -> Option<NodeId> {
        self.nodes.get(&(tag, level)).copied()
    }

    /// Where the faces, edges and vertices of the part lie in the swept mesh.
    fn cad_map(
        &self,
        gmsh: &Gmsh,
        mesh: &FeMesh,
        elements: &[SourceElement],
        sweep: &Sweep,
    ) -> Result<CadMap, GmshError> {
        let top = self.top();
        let mut solid_faces: HashMap<Vec<NodeId>, (ElementId, u8)> = HashMap::new();
        for element in mesh.elements() {
            for (k, face) in element.faces().iter().enumerate() {
                let mut corners: Vec<NodeId> =
                    face.corners.iter().map(|&i| element.nodes[i]).collect();
                corners.sort_unstable();
                solid_faces.insert(corners, (element.id, (k + 1) as u8));
            }
        }
        let lookup = |nodes: &[Option<NodeId>]| -> Option<(ElementId, u8)> {
            let mut corners: Vec<NodeId> = nodes.iter().copied().collect::<Option<_>>()?;
            corners.sort_unstable();
            solid_faces.get(&corners).copied()
        };
        let mut map = CadMap::default();
        let add_nodes = |map: &mut CadMap, entity, nodes: &mut dyn Iterator<Item = NodeId>| {
            let entry = map.nodes.entry(entity).or_default();
            entry.extend(nodes);
            entry.sort_unstable();
            entry.dedup();
        };

        // The ends.
        for &source in &sweep.source {
            let target = sweep.target_faces[&source];
            for (face, level) in [(source, 0), (target, top)] {
                let own = elements.iter().filter(|e| e.face == source);
                let mut faces = Vec::new();
                let mut nodes = Vec::new();
                for element in own {
                    let corners: Vec<_> = (element.corners.iter())
                        .map(|&c| self.existing(c, level))
                        .collect();
                    faces.extend(lookup(&corners));
                    nodes.extend(
                        (element.corners.iter().chain(&element.mids))
                            .filter_map(|&t| self.existing(t, level)),
                    );
                }
                map.faces.entry(face).or_default().extend(faces);
                add_nodes(
                    &mut map,
                    plx_mesh::CadEntity::Face(face),
                    &mut nodes.into_iter(),
                );
            }
        }

        // The edges at both ends and the side faces they sweep.
        let line = if self.quadratic {
            (LINE3, 3)
        } else {
            (LINE2, 2)
        };
        for (&edge, &target) in &sweep.target_edges {
            let (_, tags) = gmsh.elements(line.0, edge)?;
            let segments: Vec<&[usize]> = tags.chunks_exact(line.1).collect();
            for (cad_edge, level) in [(edge, 0), (target, top)] {
                let mut lines = Vec::new();
                let mut nodes = Vec::new();
                for segment in &segments {
                    if let (Some(a), Some(b)) = (
                        self.existing(segment[0], level),
                        self.existing(segment[1], level),
                    ) {
                        lines.push([a, b]);
                    }
                    nodes.extend(segment.iter().filter_map(|&t| self.existing(t, level)));
                }
                map.segments.entry(cad_edge).or_default().extend(lines);
                add_nodes(
                    &mut map,
                    plx_mesh::CadEntity::Edge(cad_edge),
                    &mut nodes.into_iter(),
                );
            }
            let Some(&side) = sweep.side_faces.get(&edge) else {
                continue;
            };
            let mut faces = Vec::new();
            let mut nodes = Vec::new();
            for segment in &segments {
                let (a, b) = (segment[0], segment[1]);
                for level in 0..=top {
                    if self.is_layer_level(level) {
                        nodes.extend(segment.iter().filter_map(|&t| self.existing(t, level)));
                    } else {
                        nodes.extend([a, b].iter().filter_map(|&t| self.existing(t, level)));
                    }
                }
                let step = if self.quadratic { 2 } else { 1 };
                for level in (0..top).step_by(step) {
                    let next = level + step;
                    faces.extend(lookup(&[
                        self.existing(a, level),
                        self.existing(b, level),
                        self.existing(b, next),
                        self.existing(a, next),
                    ]));
                }
            }
            map.faces.entry(side).or_default().extend(faces);
            add_nodes(
                &mut map,
                plx_mesh::CadEntity::Face(side),
                &mut nodes.into_iter(),
            );
        }

        // The vertices at both ends and the lateral edges between them.
        for (&vertex, &target) in &sweep.target_vertices {
            let (_, tags) = gmsh.elements(POINT, vertex)?;
            let Some(&tag) = tags.first() else {
                continue;
            };
            for (cad_vertex, level) in [(vertex, 0), (target, top)] {
                if let Some(node) = self.existing(tag, level) {
                    map.nodes
                        .insert(plx_mesh::CadEntity::Vertex(cad_vertex), vec![node]);
                }
            }
            if let Some(&edge) = sweep.lateral_edges.get(&vertex) {
                let column: Vec<NodeId> = (0..=top).filter_map(|l| self.existing(tag, l)).collect();
                let step = if self.quadratic { 2 } else { 1 };
                let segments: Vec<[NodeId; 2]> = (0..top)
                    .step_by(step)
                    .filter_map(|l| Some([self.existing(tag, l)?, self.existing(tag, l + step)?]))
                    .collect();
                map.segments.insert(edge, segments);
                add_nodes(
                    &mut map,
                    plx_mesh::CadEntity::Edge(edge),
                    &mut column.into_iter(),
                );
            }
        }
        Ok(map)
    }
}
