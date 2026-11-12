//! Selections of CAD faces and edges on the Geometry tab, e.g. for a local mesh size: what
//! a click selects in the modes of the selection window. The surface angle mode grows the
//! clicked face across edges where the next face continues at less than the angle, the
//! edge angle mode the clicked edge across vertices. The surface angle mode grows on into
//! other visible parts, over edges lying on edges of theirs.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use glam::DVec3;
use plx_mesh::{CadEntity, ElementShape, FeMesh, NodeId};

use crate::model::{Hit, Model};
use crate::selection::{GeometryRegion, Picker, SelectBy};
use crate::viewport::{BoxSelect, Preview};

/// Lookups from the CAD faces and edges of a geometry display to its triangles and lines,
/// built once per display.
#[derive(Debug, Default)]
pub struct GeometryIndex {
    /// The triangles of each face by their nodes.
    face_triangles: HashMap<i32, Vec<[NodeId; 3]>>,
    /// The points along each edge in order.
    edge_points: HashMap<i32, Vec<NodeId>>,
    face_edges: BTreeMap<i32, Vec<i32>>,
    edge_faces: HashMap<i32, Vec<i32>>,
    edge_vertices: BTreeMap<i32, Vec<i32>>,
    vertex_edges: HashMap<i32, Vec<i32>>,
    /// The part drawing each face, the first one where several do.
    face_part: HashMap<i32, usize>,
    /// The edges of other parts lying on each edge, along a stretch at least.
    edge_twins: HashMap<i32, Vec<i32>>,
}

impl GeometryIndex {
    pub fn new(mesh: &FeMesh, entities: &[CadEntity], topology: &plx_mesher::CadTopology) -> Self {
        let entity = |id: plx_mesh::ElementId| {
            let index = usize::try_from(id).ok()?.checked_sub(1)?;
            entities.get(index).copied()
        };
        let mut face_triangles: HashMap<i32, Vec<[NodeId; 3]>> = HashMap::new();
        let mut edge_points: HashMap<i32, Vec<NodeId>> = HashMap::new();
        let mut face_part: HashMap<i32, usize> = HashMap::new();
        // A face or edge shared by parts is drawn by each of them; the first one counts.
        for (part_index, part) in mesh.parts.iter().enumerate() {
            let mut triangles: HashMap<i32, Vec<[NodeId; 3]>> = HashMap::new();
            let mut points: HashMap<i32, Vec<NodeId>> = HashMap::new();
            for element in part.elements.iter().filter_map(|&id| mesh.element(id)) {
                match (entity(element.id), element.shape) {
                    (Some(CadEntity::Face(tag)), ElementShape::Tri3) => {
                        let n = &element.nodes;
                        triangles.entry(tag).or_default().push([n[0], n[1], n[2]]);
                    }
                    // The segments of an edge follow each other along it.
                    (Some(CadEntity::Edge(tag)), ElementShape::Line2) => {
                        let line = points.entry(tag).or_default();
                        if line.is_empty() {
                            line.push(element.nodes[0]);
                        }
                        line.push(element.nodes[1]);
                    }
                    _ => {}
                }
            }
            for (tag, triangles) in triangles {
                face_part.entry(tag).or_insert(part_index);
                face_triangles.entry(tag).or_insert(triangles);
            }
            for (tag, points) in points {
                edge_points.entry(tag).or_insert(points);
            }
        }
        let mut edge_faces: HashMap<i32, Vec<i32>> = HashMap::new();
        for (&face, edges) in &topology.face_edges {
            for &edge in edges {
                edge_faces.entry(edge).or_default().push(face);
            }
        }
        let mut vertex_edges: HashMap<i32, Vec<i32>> = HashMap::new();
        for (&edge, vertices) in &topology.edge_vertices {
            for &vertex in vertices {
                vertex_edges.entry(vertex).or_default().push(edge);
            }
        }
        let edge_twins = edge_twins(mesh, &edge_points);
        Self {
            face_part,
            edge_twins,
            face_triangles,
            edge_points,
            face_edges: topology.face_edges.clone(),
            edge_faces,
            edge_vertices: topology.edge_vertices.clone(),
            vertex_edges,
        }
    }

    /// The faces reached from `start` across edges where the faces on both sides meet at
    /// no more than `angle` degrees, on into other parts over edges lying on each other,
    /// of the parts for which `visible` holds.
    fn smooth_faces(
        &self,
        mesh: &FeMesh,
        start: i32,
        angle: f64,
        visible: impl Fn(usize) -> bool,
    ) -> BTreeSet<i32> {
        let limit = angle.to_radians();
        let shown = |face: &i32| self.face_part.get(face).is_none_or(|&p| visible(p));
        let mut seen = BTreeSet::from([start]);
        let mut stack = vec![start];
        while let Some(face) = stack.pop() {
            for &edge in self.face_edges.get(&face).into_iter().flatten() {
                let twins = self.edge_twins.get(&edge).into_iter().flatten();
                let next = (std::iter::once(&edge).chain(twins))
                    .flat_map(|e| self.edge_faces.get(e).into_iter().flatten());
                for &next in next {
                    if !seen.contains(&next)
                        && shown(&next)
                        && self
                            .face_angle(mesh, face, next, edge)
                            .is_some_and(|a| a <= limit)
                    {
                        seen.insert(next);
                        stack.push(next);
                    }
                }
            }
        }
        seen
    }

    /// The angle between two faces along their common edge in radians, the largest of a
    /// few points along it; the orientation of the faces does not count.
    fn face_angle(&self, mesh: &FeMesh, a: i32, b: i32, edge: i32) -> Option<f64> {
        let points = self.edge_points.get(&edge)?;
        let n = points.len();
        let mut samples: Vec<usize> = vec![n / 4, n / 2, (3 * n) / 4];
        samples.dedup();
        let mut largest: Option<f64> = None;
        for index in samples {
            let point = DVec3::from(mesh.node(*points.get(index)?)?);
            let (Some(na), Some(nb)) = (
                self.normal_near(mesh, a, point),
                self.normal_near(mesh, b, point),
            ) else {
                continue;
            };
            let angle = na.dot(nb).abs().min(1.0).acos();
            largest = Some(largest.map_or(angle, |l| l.max(angle)));
        }
        largest
    }

    /// The normal of the triangle of the face nearest to the point.
    fn normal_near(&self, mesh: &FeMesh, face: i32, point: DVec3) -> Option<DVec3> {
        let corners = |t: &[NodeId; 3]| -> Option<[DVec3; 3]> {
            Some([
                DVec3::from(mesh.node(t[0])?),
                DVec3::from(mesh.node(t[1])?),
                DVec3::from(mesh.node(t[2])?),
            ])
        };
        (self.face_triangles.get(&face)?.iter())
            .filter_map(corners)
            .filter_map(|[a, b, c]| {
                let normal = (b - a).cross(c - a).try_normalize()?;
                let distance = closest_on_triangle(point, a, b, c).distance_squared(point);
                Some((distance, normal))
            })
            .min_by(|x, y| x.0.total_cmp(&y.0))
            .map(|(_, normal)| normal)
    }

    /// The edges reached from `start` across vertices where the edges on both sides bend
    /// by no more than `angle` degrees.
    fn smooth_edges(&self, mesh: &FeMesh, start: i32, angle: f64) -> BTreeSet<i32> {
        let limit = angle.to_radians();
        let mut seen = BTreeSet::from([start]);
        let mut stack = vec![start];
        while let Some(edge) = stack.pop() {
            for &vertex in self.edge_vertices.get(&edge).into_iter().flatten() {
                for &next in self.vertex_edges.get(&vertex).into_iter().flatten() {
                    if !seen.contains(&next)
                        && self.bend(mesh, edge, next).is_some_and(|a| a <= limit)
                    {
                        seen.insert(next);
                        stack.push(next);
                    }
                }
            }
        }
        seen
    }

    /// How much the path bends from one edge into the next at the vertex they meet at, in
    /// radians: 0 where the second continues the first straight on.
    fn bend(&self, mesh: &FeMesh, a: i32, b: i32) -> Option<f64> {
        let ends = |edge: i32| -> Option<[(DVec3, DVec3); 2]> {
            let points = self.edge_points.get(&edge)?;
            let point = |i: usize| Some(DVec3::from(mesh.node(*points.get(i)?)?));
            let n = points.len();
            if n < 2 {
                return None;
            }
            // Each end with the direction leaving the edge there.
            let (first, second) = (point(0)?, point(1)?);
            let (last, before) = (point(n - 1)?, point(n - 2)?);
            Some([
                (first, (first - second).try_normalize()?),
                (last, (last - before).try_normalize()?),
            ])
        };
        let (ends_a, ends_b) = (ends(a)?, ends(b)?);
        // The ends that meet are the ones nearest to each other.
        let (out_a, out_b) = (ends_a.iter())
            .flat_map(|ea| ends_b.iter().map(move |eb| (ea, eb)))
            .min_by(|x, y| {
                (x.0.0.distance_squared(x.1.0)).total_cmp(&y.0.0.distance_squared(y.1.0))
            })
            .map(|(ea, eb)| (ea.1, eb.1))?;
        // Straight on, the direction leaving the first edge points into the second.
        Some((out_a.dot(-out_b)).clamp(-1.0, 1.0).acos())
    }

    /// The edge bounding the face nearest to the point in render coordinates.
    fn nearest_edge(&self, model: &Model, face: i32, point: glam::Vec3) -> Option<i32> {
        (self.face_edges.get(&face)?.iter())
            .filter_map(|&edge| Some((edge, self.edge_distance(model, edge, point)?)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(edge, _)| edge)
    }

    fn edge_distance(&self, model: &Model, edge: i32, point: glam::Vec3) -> Option<f32> {
        let position = |id: NodeId| Some(model.render_position(model.mesh.node_index(id)?));
        let points: Vec<glam::Vec3> = (self.edge_points.get(&edge)?.iter())
            .filter_map(|&id| position(id))
            .collect();
        (points.windows(2))
            .map(|w| {
                let ab = w[1] - w[0];
                let t = ((point - w[0]).dot(ab) / ab.length_squared().max(f32::MIN_POSITIVE))
                    .clamp(0.0, 1.0);
                (w[0] + ab * t).distance(point)
            })
            .min_by(f32::total_cmp)
    }
}

/// For each edge the other edges lying on it along a stretch, see [`crate::seams::overlap`]:
/// where parts meet that do not share their edges. Edges whose boxes overlap are compared
/// segment by segment.
fn edge_twins(mesh: &FeMesh, edge_points: &HashMap<i32, Vec<NodeId>>) -> HashMap<i32, Vec<i32>> {
    struct Polyline {
        tag: i32,
        points: Vec<DVec3>,
        min: DVec3,
        max: DVec3,
    }
    let mut lines: Vec<Polyline> = (edge_points.iter())
        .filter_map(|(&tag, ids)| {
            let points: Vec<DVec3> = (ids.iter())
                .filter_map(|&id| Some(DVec3::from(mesh.node(id)?)))
                .collect();
            if points.len() < 2 {
                return None;
            }
            let longest = (points.windows(2))
                .map(|w| w[0].distance(w[1]))
                .fold(0.0, f64::max);
            let margin = DVec3::splat(0.05 * longest);
            let min = points.iter().fold(DVec3::INFINITY, |m, p| m.min(*p)) - margin;
            let max = points.iter().fold(DVec3::NEG_INFINITY, |m, p| m.max(*p)) + margin;
            Some(Polyline {
                tag,
                points,
                min,
                max,
            })
        })
        .collect();
    lines.sort_by(|a, b| a.min.x.total_cmp(&b.min.x));
    let mut twins: HashMap<i32, Vec<i32>> = HashMap::new();
    for (i, a) in lines.iter().enumerate() {
        for b in &lines[i + 1..] {
            if b.min.x > a.max.x {
                break;
            }
            if b.min.cmpgt(a.max).any() || a.min.cmpgt(b.max).any() {
                continue;
            }
            let on_each_other = (a.points.windows(2)).any(|s| {
                (b.points.windows(2)).any(|t| crate::seams::overlap(s[0], s[1], t[0], t[1]))
            });
            if on_each_other {
                twins.entry(a.tag).or_default().push(b.tag);
                twins.entry(b.tag).or_default().push(a.tag);
            }
        }
    }
    twins
}

/// The point of the triangle nearest to `p`.
fn closest_on_triangle(p: DVec3, a: DVec3, b: DVec3, c: DVec3) -> DVec3 {
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let (d1, d2) = (ab.dot(ap), ac.dot(ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let (d3, d4) = (ab.dot(bp), ac.dot(bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + ab * (d1 / (d1 - d3));
    }
    let cp = p - c;
    let (d5, d6) = (ab.dot(cp), ac.dot(cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + ac * (d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denom = 1.0 / (va + vb + vc);
    a + ab * (vb * denom) + ac * (vc * denom)
}

/// The CAD face under the hit point.
pub fn face_at(model: &Model, hit: &Hit) -> Option<i32> {
    if hit.line {
        return None;
    }
    let face = model.skin(hit.part).faces.get(hit.face)?;
    match model.cad_entity(model.mesh.elements()[face.element].id)? {
        CadEntity::Face(tag) => Some(tag),
        _ => None,
    }
}

/// The CAD edge of a hit line, else the edge of the face under the hit nearest to it.
pub fn edge_at(model: &Model, hit: &Hit) -> Option<i32> {
    if hit.line {
        let element = *model.skin(hit.part).line_elements.get(hit.face)?;
        return match model.cad_entity(model.mesh.elements()[element].id)? {
            CadEntity::Edge(tag) => Some(tag),
            _ => None,
        };
    }
    let face = face_at(model, hit)?;
    model.geometry_index().nearest_edge(model, face, hit.point)
}

/// What a click on `hit` selects in the mode of the selection window. `precision` is the
/// pick tolerance in render units at the hit, for edges.
pub fn pick(
    model: &Model,
    picker: &Picker,
    region: GeometryRegion,
    hit: &Hit,
    precision: f32,
) -> BTreeSet<CadEntity> {
    let edges = region == GeometryRegion::FacesAndEdges;
    let index = model.geometry_index();
    match picker.select_by {
        SelectBy::GeometryPart => {
            let part = &model.mesh.parts[hit.part];
            (part.elements.iter())
                .filter_map(|&id| model.cad_entity(id))
                .filter(|e| match e {
                    CadEntity::Face(_) => true,
                    // A line part has edges only.
                    CadEntity::Edge(_) => edges && hit.line,
                    CadEntity::Vertex(_) => false,
                })
                .collect()
        }
        SelectBy::GeometrySurfaceAngle => face_at(model, hit)
            .map(|face| {
                let visible = |p: usize| model.parts.get(p).is_some_and(|info| info.visible);
                (index
                    .smooth_faces(&model.mesh, face, picker.angle(), visible)
                    .into_iter())
                .map(CadEntity::Face)
                .collect()
            })
            .unwrap_or_default(),
        SelectBy::GeometryEdgeAngle if edges => edge_at(model, hit)
            .map(|edge| {
                (index
                    .smooth_edges(&model.mesh, edge, picker.angle())
                    .into_iter())
                .map(CadEntity::Edge)
                .collect()
            })
            .unwrap_or_default(),
        _ if edges => crate::meshing::cad_pick(model, hit, precision)
            .into_iter()
            .collect(),
        _ => face_at(model, hit)
            .map(CadEntity::Face)
            .into_iter()
            .collect(),
    }
}

/// The faces of the visible parts inside a selection box, or crossing it when it was
/// dragged from right to left.
pub fn pick_box(model: &Model, area: &BoxSelect) -> BTreeSet<CadEntity> {
    // For each face whether all and whether any of its corners lie inside.
    let mut inside: BTreeMap<i32, (bool, bool)> = BTreeMap::new();
    for (part, info) in model.mesh.parts.iter().zip(&model.parts) {
        if !info.visible {
            continue;
        }
        for element in part
            .elements
            .iter()
            .filter_map(|&id| model.mesh.element(id))
        {
            let Some(CadEntity::Face(tag)) = model.cad_entity(element.id) else {
                continue;
            };
            let state = inside.entry(tag).or_insert((true, false));
            for &id in &element.nodes {
                let Some(n) = model.mesh.node_index(id) else {
                    continue;
                };
                let contained = area.contains(model.render_position(n));
                state.0 &= contained;
                state.1 |= contained;
            }
        }
    }
    (inside.into_iter())
        .filter(|&(_, (all, any))| if area.crossing { any } else { all })
        .map(|(tag, _)| CadEntity::Face(tag))
        .collect()
}

/// The faces of the visible parts.
pub fn visible_faces(model: &Model) -> BTreeSet<CadEntity> {
    (model.mesh.parts.iter().zip(&model.parts))
        .filter(|(_, info)| info.visible)
        .flat_map(|(part, _)| &part.elements)
        .filter_map(|&id| model.cad_entity(id))
        .filter(|e| matches!(e, CadEntity::Face(_)))
        .collect()
}

/// What a click at the hit would select, drawn as outlines.
pub fn preview(
    model: &Model,
    picker: &Picker,
    region: GeometryRegion,
    hit: &Hit,
    precision: f32,
) -> Preview {
    crate::meshing::entities_preview(model, &pick(model, picker, region, hit, precision))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The geometry display of a test file, `None` where Gmsh is not available.
    fn view(file: &str) -> Option<Model> {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(file);
        match crate::model::load(&path, plx_model::UnitSystem::Unitless) {
            Ok(loaded) => loaded.geometry_view,
            Err(error) if std::env::var_os("NODALIS_REQUIRE_GMSH").is_none() => {
                eprintln!("Gmsh not available, test skipped: {error}");
                None
            }
            Err(error) => panic!("{error}"),
        }
    }

    #[test]
    fn angle_modes_grow_across_smooth_faces_and_edges() {
        // A shaft whose cylinder is split into three faces along straight seam edges.
        let Some(view) = view("welle_mit_teilflaeche.step") else {
            return;
        };
        let index = view.geometry_index();
        let mesh = &view.mesh;
        let cylinder = BTreeSet::from([4, 5, 6]);
        for face in [4, 5, 6] {
            assert_eq!(index.smooth_faces(mesh, face, 30.0, |_| true), cylinder);
        }
        // The end faces meet the cylinder at a right angle.
        assert_eq!(
            index.smooth_faces(mesh, 7, 30.0, |_| true),
            BTreeSet::from([7])
        );
        assert_eq!(index.smooth_faces(mesh, 4, 95.0, |_| true).len(), 7);
        let seams = BTreeSet::from([5, 8, 10]);
        assert_eq!(index.smooth_edges(mesh, 8, 30.0), seams);
        // The circles at the ends turn away from the seams at a right angle.
        assert_eq!(index.smooth_edges(mesh, 6, 30.0), BTreeSet::from([6]));
    }

    #[test]
    fn edges_of_touching_parts_are_twins() {
        // Edge 1 of one part along x from 0 to 10, edge 2 of a neighbour from 4 to 10 in
        // two pieces, edge 3 from 10 on and edge 4 beside it: only 1 and 2 lie on each other.
        let mut mesh = FeMesh::default();
        let points = [
            (1, [0.0, 0.0, 0.0]),
            (2, [10.0, 0.0, 0.0]),
            (3, [4.0, 0.0, 0.0]),
            (4, [7.0, 0.0, 0.0]),
            (5, [10.0, 0.0, 0.0]),
            (6, [20.0, 0.0, 0.0]),
            (7, [0.0, 1.0, 0.0]),
            (8, [10.0, 1.0, 0.0]),
        ];
        for (id, at) in points {
            mesh.set_node(id, at);
        }
        let edges = HashMap::from([
            (1, vec![1, 2]),
            (2, vec![3, 4, 5]),
            (3, vec![5, 6]),
            (4, vec![7, 8]),
        ]);
        let twins = edge_twins(&mesh, &edges);
        assert_eq!(twins.get(&1), Some(&vec![2]));
        assert_eq!(twins.get(&2), Some(&vec![1]));
        assert_eq!((twins.get(&3), twins.get(&4)), (None, None));
    }

    #[test]
    fn closest_point_on_a_triangle() {
        let (a, b, c) = (DVec3::ZERO, DVec3::X, DVec3::Y);
        let inside = DVec3::new(0.25, 0.25, 1.0);
        assert_eq!(
            closest_on_triangle(inside, a, b, c),
            DVec3::new(0.25, 0.25, 0.0)
        );
        assert_eq!(closest_on_triangle(DVec3::new(-1.0, -1.0, 0.0), a, b, c), a);
        assert_eq!(
            closest_on_triangle(DVec3::new(0.5, -1.0, 0.0), a, b, c),
            DVec3::new(0.5, 0.0, 0.0)
        );
        assert_eq!(
            closest_on_triangle(DVec3::new(1.0, 1.0, 0.0), a, b, c),
            DVec3::new(0.5, 0.5, 0.0)
        );
    }
}
