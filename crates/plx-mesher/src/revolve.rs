//! Hexahedra for bodies of revolution: a part whose edges are circles about one axis and
//! profile curves in planes through it is meshed on a meridian section, its end face or a
//! section cut from it, and the section mesh is turned about the axis in steps. A part that
//! closes on itself, a full revolution, gets a section made for it; a segment is turned from
//! one end face to the other.
//!
//! Parts that touch their axis are left to the tetrahedra: their elements at the axis would
//! collapse to pyramids, which CalculiX lacks.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::f64::consts::{PI, TAU};

use plx_mesh::{CadEntity, CadMap, ElementId, FeMesh, NodeId};
use plx_model::Geometry;

use crate::extrude::{
    Motion, Point, Topology, add, cross, dot, norm, scale, source_elements, sub, sweep_mesh,
};
use crate::gmsh::{Gmsh, GmshError};
use crate::{GeneratedMesh, MeshOptions, generate_faces, node_coords};

/// What a curve of the part is in relation to the axis.
#[derive(Clone, Copy, Debug)]
enum Curve {
    /// A circle, with points along it: about the axis, or a profile curve.
    Circle {
        centre: Point,
        normal: Point,
        points: [Point; 5],
    },
    /// Any other curve, with points along it.
    Other([Point; 5]),
}

impl Curve {
    fn points(&self) -> [Point; 5] {
        match *self {
            Curve::Circle { points, .. } | Curve::Other(points) => points,
        }
    }
}

/// The circle through three points: its centre and unit normal, or `None` if they lie on
/// a line.
fn circle(p0: Point, p1: Point, p2: Point) -> Option<(Point, Point)> {
    let (a, b) = (sub(p1, p0), sub(p2, p0));
    let n = cross(a, b);
    let nn = dot(n, n);
    if nn <= 1e-24 * dot(a, a) * dot(b, b) {
        return None;
    }
    let offset = add(scale(cross(b, n), dot(a, a)), scale(cross(n, a), dot(b, b)));
    Some((add(p0, scale(offset, 0.5 / nn)), scale(n, 1.0 / nn.sqrt())))
}

fn curve(gmsh: &Gmsh, edge: i32, tolerance: f64) -> Result<Curve, GmshError> {
    let (min, max) = gmsh.parameter_bounds(1, edge)?;
    let (Some(&u0), Some(&u1)) = (min.first(), max.first()) else {
        return Err(GmshError::Other(format!("Curve {edge} has no parameters")));
    };
    let parameters: Vec<f64> = [0.0, 0.25, 0.5, 0.75, 1.0]
        .iter()
        .map(|t| u0 + t * (u1 - u0))
        .collect();
    let points = gmsh.values(1, edge, &parameters)?;
    let Ok(points): Result<[Point; 5], _> = points.try_into() else {
        return Err(GmshError::Other(format!("Curve {edge} has no points")));
    };
    if let Some((centre, normal)) = circle(points[0], points[1], points[2]) {
        let radius = norm(sub(points[0], centre));
        let on_circle = points.iter().all(|&p| {
            (norm(sub(p, centre)) - radius).abs() <= tolerance
                && dot(sub(p, centre), normal).abs() <= tolerance
        });
        if on_circle {
            return Ok(Curve::Circle {
                centre,
                normal,
                points,
            });
        }
    }
    Ok(Curve::Other(points))
}

/// The axis of revolution of a part, with the unit radial direction of each point off it.
struct Axis {
    origin: Point,
    direction: Point,
}

impl Axis {
    /// The vector from the axis to the point, perpendicular to it.
    fn radial(&self, p: Point) -> Point {
        let v = sub(p, self.origin);
        sub(v, scale(self.direction, dot(v, self.direction)))
    }

    /// The angle of a point about the axis from a reference direction.
    fn angle(&self, reference: Point, p: Point) -> f64 {
        let r = self.radial(p);
        let y = cross(self.direction, reference);
        dot(r, y).atan2(dot(r, reference)).rem_euclid(TAU)
    }
}

/// How a part is a body of revolution, or why it is none.
struct Revolution {
    axis: Axis,
    /// The faces to mesh and turn: an end face, or a section cut for a full revolution.
    source: Vec<i32>,
    /// The turn in radians, about the axis's direction.
    angle: f64,
    /// Largest distance of the part from the axis.
    radius: f64,
}

fn revolution(
    gmsh: &Gmsh,
    topology: &Topology,
    volume: i32,
) -> Result<Result<Revolution, String>, GmshError> {
    let tolerance = 1e-6 * topology.size;
    let mut curves = BTreeMap::new();
    for &edge in &topology.edges {
        curves.insert(edge, curve(gmsh, edge, 10.0 * tolerance)?);
    }
    // The axis is the one most circles turn about; circles in planes through it, rounds of
    // the profile, turn about others.
    let circles: Vec<(Point, Point)> = (curves.values())
        .filter_map(|c| match *c {
            Curve::Circle { centre, normal, .. } => Some((centre, normal)),
            Curve::Other(_) => None,
        })
        .collect();
    let about = |(centre, normal): (Point, Point), (c, n): (Point, Point)| {
        let d = sub(c, centre);
        norm(cross(normal, n)) <= 1e-6
            && norm(sub(d, scale(normal, dot(d, normal)))) <= 10.0 * tolerance
    };
    let Some(&(centre, normal)) = circles
        .iter()
        .max_by_key(|&&a| circles.iter().filter(|&&b| about(a, b)).count())
    else {
        return Ok(Err("it has no circular edges".into()));
    };
    let axis = Axis {
        origin: centre,
        direction: normal,
    };
    let distance = |p: Point| norm(axis.radial(p));
    // Every circle about the axis turns the profile; every other curve lies in a plane
    // through the axis.
    let mut meridian_edges = BTreeSet::new();
    for (&edge, curve) in &curves {
        if let Curve::Circle {
            centre: c,
            normal: n,
            ..
        } = *curve
            && about((centre, normal), (c, n))
        {
            continue;
        }
        let mut first: Option<Point> = None;
        for p in curve.points() {
            let r = axis.radial(p);
            let length = norm(r);
            if length <= 10.0 * tolerance {
                return Ok(Err("it touches its axis".into()));
            }
            let unit = scale(r, 1.0 / length);
            match first {
                None => first = Some(unit),
                Some(f) if norm(cross(f, unit)) > 1e-6 || dot(f, unit) < 0.0 => {
                    return Ok(Err(format!(
                        "curve {edge} is neither a circle about the axis nor in a plane \
                         through it"
                    )));
                }
                Some(_) => {}
            }
        }
        meridian_edges.insert(edge);
    }
    if topology
        .points
        .values()
        .any(|&p| distance(p) <= 10.0 * tolerance)
    {
        return Ok(Err("it touches its axis".into()));
    }
    // The vertices and the points along the profile curves are the farthest from the axis
    // any part of it gets: a circle keeps its distance.
    let radius = (topology.points.values().copied())
        .chain(curves.values().flat_map(Curve::points))
        .map(distance)
        .fold(0.0, f64::max);

    // Faces in a plane through the axis: the ends of a segment.
    let reference = {
        let Some(&p) = topology.points.values().next() else {
            return Ok(Err("it has no vertices".into()));
        };
        let r = axis.radial(p);
        scale(r, 1.0 / norm(r))
    };
    let mut ends: Vec<(f64, Vec<i32>)> = Vec::new();
    for &face in &topology.faces {
        let edges = &topology.face_edges[&face];
        if edges.is_empty() || !edges.iter().all(|e| meridian_edges.contains(e)) {
            continue;
        }
        let Some(&vertex) = edges
            .iter()
            .flat_map(|e| topology.edge_vertices[e].iter())
            .next()
        else {
            continue;
        };
        let angle = axis.angle(reference, topology.points[&vertex]);
        match ends.iter_mut().find(|(a, _)| {
            let d = (a - angle).rem_euclid(TAU);
            d.min(TAU - d) < 1e-6
        }) {
            Some((_, faces)) => faces.push(face),
            None => ends.push((angle, vec![face])),
        }
    }
    let part_volume = gmsh.mass(3, volume)?;
    let (source, angle, direction) = match &ends[..] {
        [] => {
            // A full revolution: the section in the half-plane through the reference.
            let size = 4.0 * topology.size;
            let rectangle = gmsh.add_rectangle([0.0, -size, 0.0], [size, 2.0 * size], 0.0)?;
            let e1 = reference;
            let n = axis.direction;
            let e2 = cross(e1, n);
            let o = sub(axis.origin, [0.0; 3]);
            let matrix = [
                e1[0], n[0], e2[0], o[0], //
                e1[1], n[1], e2[1], o[1], //
                e1[2], n[2], e2[2], o[2],
            ];
            gmsh.affine_transform(&[(2, rectangle)], &matrix)?;
            let section: Vec<i32> = (gmsh.intersect(&[(2, rectangle)], &[(3, volume)])?)
                .into_iter()
                .filter(|&(d, _)| d == 2)
                .map(|(_, t)| t)
                .collect();
            if section.is_empty() {
                return Ok(Err("no section through the axis was found".into()));
            }
            (section, TAU, n)
        }
        [(a, first), (b, _)] => {
            // A segment from one end to the other, through the part's centre of mass.
            let centre = gmsh.center_of_mass(3, volume)?;
            let m = axis.angle(reference, centre);
            let span = (b - a).rem_euclid(TAU);
            if (m - a).rem_euclid(TAU) < span {
                (first.clone(), span, axis.direction)
            } else {
                (first.clone(), TAU - span, scale(axis.direction, -1.0))
            }
        }
        _ => {
            return Ok(Err(
                "it has faces in more than two planes through its axis".into()
            ));
        }
    };
    // Pappus: the volume is the section's area times the path of its centroid.
    let mut area = 0.0;
    let mut moment = [0.0; 3];
    for &face in &source {
        let a = gmsh.mass(2, face)?;
        area += a;
        moment = add(moment, scale(gmsh.center_of_mass(2, face)?, a));
    }
    if area <= 0.0 {
        return Ok(Err("its section has no area".into()));
    }
    let expected = angle * distance(scale(moment, 1.0 / area)) * area;
    if (expected - part_volume).abs() > 1e-3 * part_volume.abs() {
        return Ok(Err(format!(
            "its volume {part_volume:.6e} is not that of its section turned about the axis \
             ({expected:.6e})"
        )));
    }
    Ok(Ok(Revolution {
        axis: Axis {
            origin: axis.origin,
            direction,
        },
        source,
        angle,
        radius,
    }))
}

/// Meshes a solid as a body of revolution. The other solids are gone from the Gmsh model
/// already. The inner error says why the part is none; the Gmsh model may then have
/// changed.
pub(crate) fn revolve_part_mesh(
    gmsh: &Gmsh,
    geometry: &Geometry,
    part: &str,
    volume: i32,
) -> Result<Result<GeneratedMesh, String>, GmshError> {
    let topology = Topology::read(gmsh, volume)?;
    let revolution = match revolution(gmsh, &topology, volume)? {
        Ok(revolution) => revolution,
        Err(reason) => return Ok(Err(format!("{part}: no body of revolution: {reason}"))),
    };
    let mut options = MeshOptions::of_part(geometry, part);
    options.algorithms.0 = plx_model::Algorithm2d::QuasiStructuredQuad;
    options.quads = true;
    options.visible_only = true;
    // Only the section is meshed.
    let mut all = Vec::new();
    for dim in 0..=3 {
        all.extend(gmsh.entities(dim)?.into_iter().map(|t| (dim, t)));
    }
    gmsh.set_visibility(&all, false)?;
    let source: Vec<(i32, i32)> = revolution.source.iter().map(|&f| (2, f)).collect();
    gmsh.set_visibility(&source, true)?;
    let own: BTreeSet<i32> = (revolution.source.iter())
        .filter(|f| topology.faces.contains(f))
        .copied()
        .collect();
    let mut notes = Vec::new();
    generate_faces(
        gmsh,
        geometry,
        &mut options,
        &revolution.source,
        &own,
        &mut notes,
    )?;
    gmsh.set_number("Mesh.MeshOnlyVisible", 0.0)?;
    let quadratic = options.order == 2;
    let coords = node_coords(gmsh)?;
    let motion = Motion::Rotate {
        origin: revolution.axis.origin,
        axis: revolution.axis.direction,
        angle: revolution.angle,
    };
    let (elements, _) = source_elements(gmsh, &revolution.source, quadratic, &coords, &motion)?;
    if elements.is_empty() {
        return Err(GmshError::Other(format!(
            "{part}: Gmsh created no elements on the section"
        )));
    }
    // A solid section reaches the axis even where no edge of the part does.
    let on_axis = (elements.iter()).flat_map(|e| &e.corners).any(|c| {
        coords
            .get(c)
            .is_some_and(|&p| norm(revolution.axis.radial(p)) <= 1e-6 * topology.size)
    });
    if on_axis {
        return Ok(Err(format!(
            "{part}: no body of revolution: it touches its axis"
        )));
    }
    let max_size = geometry.parameters(part).max_size;
    let steps = ((revolution.angle * revolution.radius / max_size).round())
        .max((revolution.angle / (PI / 8.0)).ceil())
        .clamp(1.0, 100_000.0) as u32;
    let levels: Vec<f64> = (1..=steps)
        .map(|k| f64::from(k) / f64::from(steps))
        .collect();
    let (mut mesh, _) = sweep_mesh(part, &coords, &elements, motion, &levels, quadratic)?;
    mesh.cad = classify(gmsh, &mesh, &topology)?;
    let mut warnings = gmsh.warnings()?;
    warnings.extend(notes.into_iter().map(|note| format!("{part}: {note}")));
    Ok(Ok(GeneratedMesh { mesh, warnings }))
}

/// Where the faces, edges and vertices of the part lie in a mesh made without recording
/// them: each outer element face belongs to the nearest CAD face, each segment between them
/// to the CAD edge both its nodes lie on, each vertex to the node on it.
pub(crate) fn classify(
    gmsh: &Gmsh,
    mesh: &FeMesh,
    topology: &Topology,
) -> Result<CadMap, GmshError> {
    let tolerance = 1e-5 * topology.size;
    let margin = 0.05 * topology.size;
    let position = |id: NodeId| mesh.node(id).unwrap_or_default();
    // The outer element faces, found once.
    let mut count: HashMap<Vec<NodeId>, (usize, ElementId, u8)> = HashMap::new();
    for element in mesh.elements() {
        for (k, face) in element.faces().iter().enumerate() {
            let mut corners: Vec<NodeId> = face.corners.iter().map(|&i| element.nodes[i]).collect();
            corners.sort_unstable();
            count
                .entry(corners)
                .and_modify(|c| c.0 += 1)
                .or_insert((1, element.id, (k + 1) as u8));
        }
    }
    let inside = |p: Point, entity: (i32, i32), margin: f64| {
        topology.boxes.get(&entity).is_some_and(|(min, max)| {
            (0..3).all(|k| p[k] >= min[k] - margin && p[k] <= max[k] + margin)
        })
    };
    let mut map = CadMap::default();
    // CAD face of each outer face, and of each segment the outer faces either side.
    let mut segment_faces: HashMap<(NodeId, NodeId), (Vec<i32>, Option<NodeId>)> = HashMap::new();
    let mut boundary_nodes = BTreeSet::new();
    let mut outer: Vec<(ElementId, u8)> = count
        .values()
        .filter(|c| c.0 == 1)
        .map(|c| (c.1, c.2))
        .collect();
    outer.sort_unstable();
    for (id, k) in outer {
        let Some(element) = mesh.element(id) else {
            continue;
        };
        let topo = &element.faces()[usize::from(k) - 1];
        let quadratic = element.shape.is_quadratic();
        let corners: Vec<NodeId> = topo.corners.iter().map(|&i| element.nodes[i]).collect();
        let mids: Vec<NodeId> = if quadratic {
            topo.mids.iter().map(|&i| element.nodes[i]).collect()
        } else {
            Vec::new()
        };
        let all: Vec<NodeId> = corners.iter().chain(&mids).copied().collect();
        let centre = scale(
            all.iter().fold([0.0; 3], |a, &n| add(a, position(n))),
            1.0 / all.len() as f64,
        );
        let mut best: Option<(f64, i32)> = None;
        let mut candidates: Vec<i32> = (topology.faces.iter())
            .filter(|&&f| inside(centre, (2, f), margin))
            .copied()
            .collect();
        if candidates.is_empty() {
            candidates = topology.faces.clone();
        }
        // The surface of a face may reach beyond it: a face the point projects outside of
        // is only taken if no other is near.
        for face in candidates {
            let (closest, parameters) = gmsh.projection(2, face, centre)?;
            let mut d = norm(sub(closest, centre));
            if !gmsh.is_inside(2, face, &parameters)? {
                d += topology.size;
            }
            if best.is_none_or(|(b, _)| d < b) {
                best = Some((d, face));
            }
        }
        let Some((_, face)) = best else {
            continue;
        };
        map.faces.entry(face).or_default().push((id, k));
        map.nodes
            .entry(CadEntity::Face(face))
            .or_default()
            .extend(&all);
        boundary_nodes.extend(&all);
        let n = corners.len();
        for i in 0..n {
            let (a, b) = (corners[i], corners[(i + 1) % n]);
            let key = (a.min(b), a.max(b));
            let entry = segment_faces
                .entry(key)
                .or_insert((Vec::new(), mids.get(i).copied()));
            entry.0.push(face);
        }
    }
    // Nodes on each CAD edge, and the segments along it.
    for &edge in &topology.edges {
        let adjacent: BTreeSet<i32> = (topology.faces.iter())
            .filter(|f| topology.face_edges[f].contains(&edge))
            .copied()
            .collect();
        let mut on_edge = BTreeSet::new();
        for &node in &boundary_nodes {
            let p = position(node);
            if inside(p, (1, edge), 10.0 * tolerance)
                && norm(sub(gmsh.closest_point(1, edge, p)?, p)) <= tolerance
            {
                on_edge.insert(node);
            }
        }
        if on_edge.is_empty() {
            continue;
        }
        let mut segments = Vec::new();
        let mut nodes: BTreeSet<NodeId> = BTreeSet::new();
        for (&(a, b), (faces, mid)) in &segment_faces {
            if !(on_edge.contains(&a) && on_edge.contains(&b)) {
                continue;
            }
            let sides: BTreeSet<i32> = faces.iter().copied().collect();
            if sides != adjacent {
                continue;
            }
            segments.push([a, b]);
            nodes.extend([a, b]);
            nodes.extend(*mid);
        }
        if segments.is_empty() {
            continue;
        }
        segments.sort_unstable();
        map.segments.insert(edge, segments);
        map.nodes
            .insert(CadEntity::Edge(edge), nodes.into_iter().collect());
    }
    for (&vertex, &p) in &topology.points {
        let nearest = (boundary_nodes.iter())
            .map(|&n| (norm(sub(position(n), p)), n))
            .min_by(|a, b| a.0.total_cmp(&b.0));
        if let Some((d, node)) = nearest
            && d <= tolerance
        {
            map.nodes.insert(CadEntity::Vertex(vertex), vec![node]);
        }
    }
    for nodes in map.nodes.values_mut() {
        nodes.sort_unstable();
        nodes.dedup();
    }
    Ok(map)
}
