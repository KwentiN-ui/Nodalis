//! The disassembly direction method, in three stages.
//!
//! 1. Contact graph: groups of connected parts are the nodes, detected interfaces the edges.
//!    Two surface patches touch when the bounding boxes of their faces, enlarged by half the
//!    contact tolerance, overlap; two nearly planar patches must also face each other.
//! 2. Directions: every interface face proposes a direction, a plane its normal and a bore or
//!    shaft its axis, oriented the way that needs the shorter travel. An axis outranks any
//!    plane, since a bore lets a shaft out only one way. Two parts held by the same bolt can
//!    only come apart along that bolt.
//! 3. Layout: the group with the most interfaces stays and the rest hangs on it as a tree
//!    along the strongest interfaces. Each group inherits the offset of its parent and adds
//!    its own travel until its box clears all boxes placed before it, so subassemblies
//!    unfold as chains and stacks keep their order.

use std::collections::HashMap;

use glam::{DMat3, DVec3};
use plx_mesh::ElementFamily;

use super::{Aabb, Assembly, Direction, round_significant};

const DEFAULT_TOLERANCE_FACTOR: f64 = 1e-3;
const MIN_NORMAL_COHERENCE: f64 = 0.7;
const MAX_NORMAL_ANGLE_DEG: f64 = 30.0;
const MIN_GRID_CELL_COUNT: usize = 64;
const MAX_CACHED_GRIDS: usize = 256;
const MERGE_ANGLE_DEG: f64 = 15.0;
const MAX_AXIS_DEVIATION_ANGLE_DEG: f64 = 30.0;
const MAX_PENETRATION_ANGLE_DEG: f64 = 15.0;
const MIN_DIRECTION_CONFIDENCE: f64 = 0.6;
const CLEARANCE_FACTOR: f64 = 0.1;
const MIN_CLEARANCE_FACTOR: f64 = 0.01;
const DIRECTION_GROUP_DIGITS: i32 = 2;
const MIN_FASTENER_AXIAL_INTERFACES: usize = 2;
const MAX_FASTENER_VOLUME_FACTOR: f64 = 0.2;
/// Relative tolerances of the surface recognition.
const PLANE_REL_TOL: f64 = 1e-3;
const NORMAL_RANK_REL_TOL: f64 = 0.02;
const FIT_REL_TOL: f64 = 5e-3;
/// Faceted meshes of curved faces deviate more from the exact surface than CAD geometry.
const REVOLUTION_REL_TOL: f64 = 0.02;
const TINY: f64 = 1e-12;

/// Step offsets of every group, or `None` when the model gives the method nothing to work
/// with and the caller falls back to another method.
pub fn layout(
    assembly: &Assembly,
    groups: &[Vec<usize>],
    direction: Direction,
    tolerance: f64,
) -> Option<Vec<Vec<DVec3>>> {
    if groups.len() < 2 {
        return None;
    }
    let tolerance = if tolerance > 0.0 {
        tolerance
    } else {
        let mut bounds = Aabb::EMPTY;
        for part in groups.iter().flatten() {
            bounds.include(&assembly.part_box(*part));
        }
        let diagonal = bounds.diagonal();
        if diagonal <= 0.0 {
            DEFAULT_TOLERANCE_FACTOR
        } else {
            diagonal * DEFAULT_TOLERANCE_FACTOR
        }
    };
    let graph = Graph::new(assembly, groups, tolerance);
    if graph.interfaces.is_empty() {
        return None;
    }
    Some(place(&graph, direction))
}

/// A surface patch of a part, prepared for the contact detection.
struct Face {
    /// Corner node indices of each element face.
    cells: Vec<Vec<usize>>,
    /// For each cell, a point inside the element it belongs to, which orients its normal out
    /// of the material; `None` for shell and membrane elements.
    insides: Vec<Option<DVec3>>,
    area: f64,
    /// Length of the summed area vector over the summed areas: one for a plane, near zero for
    /// a closed bore.
    coherence: f64,
    normal: DVec3,
    tolerance_box: Aabb,
    cell_boxes: Vec<Aabb>,
}

impl Face {
    fn new(
        coords: &[[f64; 3]],
        cells: Vec<Vec<usize>>,
        insides: Vec<Option<DVec3>>,
        tolerance: f64,
    ) -> Self {
        let mut area_vector = DVec3::ZERO;
        let mut area = 0.0;
        let mut tolerance_box = Aabb::EMPTY;
        let mut cell_boxes = Vec::with_capacity(cells.len());
        for (cell, inside) in cells.iter().zip(&insides) {
            let vector = oriented_area_vector(coords, cell, *inside);
            area_vector += vector;
            area += vector.length();
            let mut bounds = Aabb::EMPTY;
            for &node in cell {
                bounds.include_point(DVec3::from(coords[node]));
            }
            bounds.inflate(0.5 * tolerance);
            tolerance_box.include(&bounds);
            cell_boxes.push(bounds);
        }
        Self {
            coherence: if area > 0.0 {
                area_vector.length() / area
            } else {
                0.0
            },
            normal: area_vector.normalize_or_zero(),
            area,
            tolerance_box,
            cell_boxes,
            cells,
            insides,
        }
    }
}

/// Area vector of a polygon, pointing away from the inside point when there is one.
fn oriented_area_vector(coords: &[[f64; 3]], cell: &[usize], inside: Option<DVec3>) -> DVec3 {
    let point = |i: usize| DVec3::from(coords[cell[i % cell.len()]]);
    let vector: DVec3 = (0..cell.len())
        .map(|i| point(i).cross(point(i + 1)))
        .sum::<DVec3>()
        * 0.5;
    match inside {
        Some(inside) => {
            let center = (0..cell.len()).map(point).sum::<DVec3>() / cell.len() as f64;
            if vector.dot(center - inside) < 0.0 {
                -vector
            } else {
                vector
            }
        }
        None => vector,
    }
}

/// Corner nodes of the cells of one surface patch and a point inside each cell's element.
type Patch = (Vec<Vec<usize>>, Vec<Option<DVec3>>);

/// A group of connected parts moving as one rigid body, with the surface patches of its
/// parts.
struct Cluster {
    name: String,
    bounds: Aabb,
    tolerance_box: Aabb,
    faces: Vec<Face>,
}

impl Cluster {
    fn volume(&self) -> f64 {
        round_significant(self.bounds.volume(), 6)
    }
}

/// The contact between two groups.
struct Interface {
    first: usize,
    second: usize,
    first_faces: Vec<usize>,
    second_faces: Vec<usize>,
    /// Unit direction along which the second group separates from the first one.
    direction: Option<DVec3>,
    confidence: f64,
    /// The direction is the axis of a bore rather than the normal of a plane.
    axial: bool,
    /// The group that runs through the other one, the one whose curved faces turn their back
    /// to the axis.
    shaft: Option<usize>,
}

impl Interface {
    fn other(&self, cluster: usize) -> usize {
        if cluster == self.first {
            self.second
        } else {
            self.first
        }
    }

    /// A shaft of one group runs through a hole of the other.
    fn is_axial_mount(&self) -> bool {
        self.axial && self.shaft.is_some()
    }

    /// The direction along which the given group separates from the other one.
    fn direction_of(&self, cluster: usize) -> Option<DVec3> {
        let d = self.direction?;
        Some(if cluster == self.second { d } else { -d })
    }

    fn area(&self, clusters: &[Cluster]) -> f64 {
        let sum = |cluster: usize, faces: &[usize]| -> f64 {
            faces.iter().map(|&f| clusters[cluster].faces[f].area).sum()
        };
        0.5 * (sum(self.first, &self.first_faces) + sum(self.second, &self.second_faces))
    }
}

struct Graph {
    clusters: Vec<Cluster>,
    interfaces: Vec<Interface>,
    by_cluster: Vec<Vec<usize>>,
}

impl Graph {
    fn new(assembly: &Assembly, groups: &[Vec<usize>], tolerance: f64) -> Self {
        let coords = assembly.coords();
        let clusters: Vec<Cluster> = groups
            .iter()
            .map(|group| cluster(assembly, group, tolerance))
            .collect();
        let mut interfaces = Vec::new();
        let mut grids = HashMap::new();
        for i in 0..clusters.len() {
            for j in i + 1..clusters.len() {
                if !clusters[i]
                    .tolerance_box
                    .intersects(&clusters[j].tolerance_box)
                {
                    continue;
                }
                if let Some(interface) = find_interface(&clusters, i, j, &mut grids) {
                    interfaces.push(interface);
                }
            }
        }
        resolve_directions(coords, &clusters, &mut interfaces);
        let mut by_cluster = vec![Vec::new(); clusters.len()];
        for (index, interface) in interfaces.iter().enumerate() {
            by_cluster[interface.first].push(index);
            by_cluster[interface.second].push(index);
        }
        Self {
            clusters,
            interfaces,
            by_cluster,
        }
    }

    fn interfaces_of(&self, cluster: usize) -> impl Iterator<Item = &Interface> {
        self.by_cluster[cluster]
            .iter()
            .map(|&i| &self.interfaces[i])
    }

    fn interface(&self, a: usize, b: usize) -> Option<&Interface> {
        self.interfaces_of(a).find(|i| i.other(a) == b)
    }

    fn degree(&self, cluster: usize) -> usize {
        self.by_cluster[cluster].len()
    }
}

fn cluster(assembly: &Assembly, group: &[usize], tolerance: f64) -> Cluster {
    let coords = assembly.coords();
    let mesh = assembly.mesh;
    let mut parts = group.to_vec();
    parts.sort_by(|&a, &b| assembly.parts[a].name.cmp(assembly.parts[b].name));
    let name = match parts.len() {
        1 => assembly.parts[parts[0]].name.to_string(),
        n => format!("{} (+{})", assembly.parts[parts[0]].name, n - 1),
    };
    let mut bounds = Aabb::EMPTY;
    let mut faces = Vec::new();
    for &part in &parts {
        bounds.include(&assembly.part_box(part));
        let skin = assembly.parts[part].skin;
        let mut patches: Vec<Patch> = Vec::new();
        for face in &skin.faces {
            if face.region >= patches.len() {
                patches.resize_with(face.region + 1, Default::default);
            }
            let element = mesh.elements().get(face.element);
            let inside = element
                .filter(|e| e.shape.family() == ElementFamily::Solid)
                .and_then(|e| {
                    let nodes: Vec<DVec3> = (e.nodes.iter())
                        .filter_map(|&id| mesh.node(id))
                        .map(DVec3::from)
                        .collect();
                    (!nodes.is_empty()).then(|| nodes.iter().sum::<DVec3>() / nodes.len() as f64)
                });
            patches[face.region].0.push(face.corners.clone());
            patches[face.region].1.push(inside);
        }
        faces.extend(
            patches
                .into_iter()
                .filter(|(cells, _)| !cells.is_empty())
                .map(|(cells, insides)| Face::new(coords, cells, insides, tolerance)),
        );
    }
    let mut tolerance_box = bounds;
    tolerance_box.inflate(0.5 * tolerance);
    Cluster {
        name,
        bounds,
        tolerance_box,
        faces,
    }
}

fn find_interface(
    clusters: &[Cluster],
    first: usize,
    second: usize,
    grids: &mut HashMap<(usize, usize), CellGrid>,
) -> Option<Interface> {
    let max_dot = -MAX_NORMAL_ANGLE_DEG.to_radians().cos();
    let mut interface: Option<Interface> = None;
    for (i, a) in clusters[first].faces.iter().enumerate() {
        for (j, b) in clusters[second].faces.iter().enumerate() {
            if !a.tolerance_box.intersects(&b.tolerance_box) {
                continue;
            }
            if a.coherence >= MIN_NORMAL_COHERENCE
                && b.coherence >= MIN_NORMAL_COHERENCE
                && a.normal.dot(b.normal) > max_dot
            {
                continue;
            }
            if !faces_touch((first, i, a), (second, j, b), grids) {
                continue;
            }
            let interface = interface.get_or_insert_with(|| Interface {
                first,
                second,
                first_faces: Vec::new(),
                second_faces: Vec::new(),
                direction: None,
                confidence: 0.0,
                axial: false,
                shaft: None,
            });
            if !interface.first_faces.contains(&i) {
                interface.first_faces.push(i);
            }
            if !interface.second_faces.contains(&j) {
                interface.second_faces.push(j);
            }
        }
    }
    interface
}

fn faces_touch(
    a: (usize, usize, &Face),
    b: (usize, usize, &Face),
    grids: &mut HashMap<(usize, usize), CellGrid>,
) -> bool {
    let (grid_face, query) = if a.2.cell_boxes.len() >= b.2.cell_boxes.len() {
        (a, b.2)
    } else {
        (b, a.2)
    };
    let boxes = &grid_face.2.cell_boxes;
    if boxes.len() < MIN_GRID_CELL_COUNT {
        return (query.cell_boxes.iter()).any(|q| boxes.iter().any(|g| q.intersects(g)));
    }
    let key = (grid_face.0, grid_face.1);
    if !grids.contains_key(&key) && grids.len() >= MAX_CACHED_GRIDS {
        grids.clear();
    }
    let grid = grids.entry(key).or_insert_with(|| CellGrid::new(boxes));
    query.cell_boxes.iter().any(|q| grid.intersects(boxes, q))
}

/// A uniform spatial hash of the cell boxes of one face, so that proximity queries do not
/// depend on the number of cells. The spacing is the largest cell diagonal.
struct CellGrid {
    spacing: f64,
    cells: HashMap<[i64; 3], Vec<usize>>,
}

impl CellGrid {
    fn new(boxes: &[Aabb]) -> Self {
        let spacing = boxes.iter().map(Aabb::diagonal).fold(0.0, f64::max);
        let mut grid = Self {
            spacing: if spacing > 0.0 { spacing } else { 1.0 },
            cells: HashMap::new(),
        };
        for (index, b) in boxes.iter().enumerate() {
            for key in grid.keys(b) {
                grid.cells.entry(key).or_default().push(index);
            }
        }
        grid
    }

    fn keys(&self, b: &Aabb) -> Vec<[i64; 3]> {
        let index = |v: f64| (v / self.spacing).floor().clamp(-1e15, 1e15) as i64;
        let (min, max) = (b.min.to_array().map(index), b.max.to_array().map(index));
        let mut keys = Vec::new();
        for i in min[0]..=max[0] {
            for j in min[1]..=max[1] {
                for k in min[2]..=max[2] {
                    keys.push([i, j, k]);
                }
            }
        }
        keys
    }

    fn intersects(&self, boxes: &[Aabb], query: &Aabb) -> bool {
        self.keys(query).iter().any(|key| {
            self.cells
                .get(key)
                .is_some_and(|ids| ids.iter().any(|&i| boxes[i].intersects(query)))
        })
    }
}

/// What a surface patch is, as far as the direction of taking a part off goes.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Shape {
    Plane,
    /// A cylinder, cone or extrusion; `convex` when it is the outside of a shaft. The radius
    /// is zero where none was recognized.
    Axis {
        axis: DVec3,
        radius: f64,
        convex: bool,
    },
    Sphere,
    Other,
}

/// One proposed direction, weighted by the face area.
struct Candidate {
    direction: DVec3,
    weight: f64,
    is_axis: bool,
    radius: f64,
    face_count: usize,
}

fn resolve_directions(coords: &[[f64; 3]], clusters: &[Cluster], interfaces: &mut [Interface]) {
    let mut shapes: HashMap<(usize, usize), Shape> = HashMap::new();
    for interface in interfaces.iter_mut() {
        resolve(coords, clusters, interface, &mut shapes);
    }
    apply_shared_shaft_directions(clusters, interfaces);
}

fn resolve(
    coords: &[[f64; 3]],
    clusters: &[Cluster],
    interface: &mut Interface,
    shapes: &mut HashMap<(usize, usize), Shape>,
) {
    let (first, second) = (&clusters[interface.first], &clusters[interface.second]);
    let mut candidates = Vec::new();
    let mut shaft_area = [0.0, 0.0];
    let sides = [
        (interface.first, &interface.first_faces),
        (interface.second, &interface.second_faces),
    ];
    for (side, (cluster, faces)) in sides.into_iter().enumerate() {
        for &f in faces {
            let face = &clusters[cluster].faces[f];
            let shape = *shapes
                .entry((cluster, f))
                .or_insert_with(|| classify(coords, face));
            let (direction, is_axis, radius) = match shape {
                Shape::Sphere => continue,
                Shape::Plane => (face.normal, false, 0.0),
                Shape::Axis {
                    axis,
                    radius,
                    convex,
                } => {
                    if convex {
                        shaft_area[side] += face.area;
                    }
                    (axis, true, radius)
                }
                Shape::Other if face.coherence >= MIN_NORMAL_COHERENCE => (face.normal, false, 0.0),
                Shape::Other => continue,
            };
            let Some(direction) = direction.try_normalize() else {
                continue;
            };
            candidates.push(Candidate {
                direction: oriented(direction, first, second),
                weight: face.area,
                is_axis,
                radius,
                face_count: 1,
            });
        }
    }
    let merged = merge_candidates(candidates);
    let Some(winner) = winner(&merged) else {
        return;
    };
    let mut total = 0.0;
    let mut agreeing = 0.0;
    for candidate in &merged {
        // A plane does not vote against a winning axis.
        if winner.is_axis && !candidate.is_axis {
            continue;
        }
        total += candidate.weight;
        if agrees(candidate, winner.direction) {
            agreeing += candidate.weight;
        }
    }
    if total <= TINY {
        return;
    }
    let confidence = agreeing / total;
    if !winner.is_axis && confidence < MIN_DIRECTION_CONFIDENCE {
        return;
    }
    interface.direction = Some(winner.direction);
    interface.confidence = confidence;
    interface.axial = winner.is_axis;
    interface.shaft = if shaft_area[0] > shaft_area[1] {
        Some(interface.first)
    } else if shaft_area[1] > shaft_area[0] {
        Some(interface.second)
    } else {
        None
    };
}

/// Two parts pinned together by the same bolt can only come apart along that bolt: an
/// interface that is no shaft in a hole takes over the axis of a shaft running through both
/// of its groups.
fn apply_shared_shaft_directions(clusters: &[Cluster], interfaces: &mut [Interface]) {
    let mut mounts: Vec<Vec<usize>> = vec![Vec::new(); clusters.len()];
    for (index, interface) in interfaces.iter().enumerate() {
        if let (true, Some(shaft)) = (interface.is_axial_mount(), interface.shaft) {
            mounts[interface.other(shaft)].push(index);
        }
    }
    let mut updates = Vec::new();
    for (index, interface) in interfaces.iter().enumerate() {
        if interface.is_axial_mount() {
            continue;
        }
        let mut shared: Option<usize> = None;
        for &a in &mounts[interface.first] {
            for &b in &mounts[interface.second] {
                if interfaces[a].shaft != interfaces[b].shaft {
                    continue;
                }
                if shared
                    .is_none_or(|s| interfaces[a].area(clusters) > interfaces[s].area(clusters))
                {
                    shared = Some(a);
                }
            }
        }
        if let Some(s) = shared
            && let Some(direction) = interfaces[s].direction
        {
            let direction = oriented(
                direction,
                &clusters[interface.first],
                &clusters[interface.second],
            );
            updates.push((
                index,
                direction,
                interfaces[s].confidence,
                interfaces[s].shaft,
            ));
        }
    }
    for (index, direction, confidence, shaft) in updates {
        let interface = &mut interfaces[index];
        interface.direction = Some(direction);
        interface.confidence = confidence;
        interface.axial = false;
        interface.shaft = shaft;
    }
}

/// The sign along which the second group needs the shorter travel to clear the first one;
/// it does not rely on face normals, so shells work as well.
fn oriented(direction: DVec3, first: &Cluster, second: &Cluster) -> DVec3 {
    let positive = travel_to_clear(&second.bounds, &first.bounds, direction);
    let negative = travel_to_clear(&second.bounds, &first.bounds, -direction);
    if negative < positive {
        -direction
    } else if positive < negative {
        direction
    } else if (second.bounds.center() - first.bounds.center()).dot(direction) < 0.0 {
        -direction
    } else {
        direction
    }
}

fn travel_to_clear(moving: &Aabb, fixed: &Aabb, direction: DVec3) -> f64 {
    let (moving_min, _) = moving.projection(direction);
    let (_, fixed_max) = fixed.projection(direction);
    (fixed_max - moving_min).max(0.0)
}

fn merge_candidates(candidates: Vec<Candidate>) -> Vec<Candidate> {
    let min_dot = MERGE_ANGLE_DEG.to_radians().cos();
    let mut merged: Vec<Candidate> = Vec::new();
    for candidate in candidates {
        let into = merged.iter_mut().find(|m| {
            m.is_axis == candidate.is_axis && m.direction.dot(candidate.direction) >= min_dot
        });
        match into {
            Some(m) => {
                m.weight += candidate.weight;
                m.face_count += candidate.face_count;
                m.radius = m.radius.max(candidate.radius);
            }
            None => merged.push(candidate),
        }
    }
    merged
}

/// The strongest axis whenever there is one, else the heaviest plane. Of two axes the one
/// most bores agree on wins, then the wider one, then the larger area.
fn winner(candidates: &[Candidate]) -> Option<&Candidate> {
    let mut best_axis: Option<&Candidate> = None;
    let mut best_plane: Option<&Candidate> = None;
    for candidate in candidates {
        if candidate.is_axis {
            let stronger = best_axis.is_none_or(|best| {
                if candidate.face_count != best.face_count {
                    candidate.face_count > best.face_count
                } else if candidate.radius > 0.0
                    && best.radius > 0.0
                    && candidate.radius != best.radius
                {
                    candidate.radius > best.radius
                } else {
                    candidate.weight > best.weight
                }
            });
            if stronger {
                best_axis = Some(candidate);
            }
        } else if best_plane.is_none_or(|best| candidate.weight > best.weight) {
            best_plane = Some(candidate);
        }
    }
    best_axis.or(best_plane)
}

/// An axis has to be followed, so it must stay nearly parallel; a plane only blocks motion
/// into its material.
fn agrees(candidate: &Candidate, direction: DVec3) -> bool {
    let dot = candidate.direction.dot(direction);
    if candidate.is_axis {
        dot >= MAX_AXIS_DEVIATION_ANGLE_DEG.to_radians().cos()
    } else {
        dot >= -MAX_PENETRATION_ANGLE_DEG.to_radians().sin()
    }
}

/// Recognizes planes, surfaces with an axis and spheres from the corner nodes of a patch.
fn classify(coords: &[[f64; 3]], face: &Face) -> Shape {
    let mut index: HashMap<usize, usize> = HashMap::new();
    let mut points: Vec<DVec3> = Vec::new();
    let mut node_normals: Vec<DVec3> = Vec::new();
    let mut cell_normals = Vec::with_capacity(face.cells.len());
    let mut cell_areas = Vec::with_capacity(face.cells.len());
    let mut total_area = 0.0;
    for (cell, inside) in face.cells.iter().zip(&face.insides) {
        let vector = oriented_area_vector(coords, cell, *inside);
        let area = vector.length();
        let normal = vector.normalize_or_zero();
        total_area += area;
        cell_normals.push(normal);
        cell_areas.push(area);
        for &node in cell {
            let i = *index.entry(node).or_insert_with(|| {
                points.push(DVec3::from(coords[node]));
                node_normals.push(DVec3::ZERO);
                points.len() - 1
            });
            node_normals[i] += normal * area;
        }
    }
    let n = points.len();
    if n < 3 || total_area <= 0.0 {
        return Shape::Other;
    }
    for normal in &mut node_normals {
        *normal = normal.normalize_or_zero();
    }
    let centroid = points.iter().sum::<DVec3>() / n as f64;
    let extent = points
        .iter()
        .map(|p| p.distance(centroid))
        .fold(0.0, f64::max);
    if extent <= 0.0 {
        return Shape::Other;
    }
    // A plane: the points hardly deviate from their best fitting plane.
    let mut covariance = DMat3::ZERO;
    for p in &points {
        let d = *p - centroid;
        covariance += outer(d, d);
    }
    let (values, _) = symmetric_eigen(covariance);
    if (values[2].max(0.0) / n as f64).sqrt() <= PLANE_REL_TOL * extent {
        return Shape::Plane;
    }
    // A cylinder, cone or extrusion: the cell normals lie in one plane, normal to the axis.
    let mean = (cell_normals.iter().zip(&cell_areas))
        .map(|(normal, area)| *normal * *area)
        .sum::<DVec3>()
        / total_area;
    let mut normal_covariance = DMat3::ZERO;
    for (normal, area) in cell_normals.iter().zip(&cell_areas) {
        let d = *normal - mean;
        normal_covariance += outer(d, d) * *area;
    }
    let (values, vectors) = symmetric_eigen(normal_covariance * (1.0 / total_area));
    let coplanar = values[0] > 1e-30
        && values[2] <= NORMAL_RANK_REL_TOL * values[0]
        && values[1] > NORMAL_RANK_REL_TOL * values[0];
    if coplanar {
        let axis = vectors[2];
        if let Some(shape) = revolution(&points, &node_normals, axis, centroid, extent) {
            return shape;
        }
    }
    if is_sphere(&points, centroid, extent) {
        return Shape::Sphere;
    }
    if coplanar {
        return Shape::Axis {
            axis: vectors[2],
            radius: 0.0,
            convex: false,
        };
    }
    Shape::Other
}

/// A cylinder or cone around the axis direction: the distance from the axis is constant or
/// changes linearly along it.
fn revolution(
    points: &[DVec3],
    normals: &[DVec3],
    axis: DVec3,
    centroid: DVec3,
    extent: f64,
) -> Option<Shape> {
    let axis = axis.try_normalize()?;
    let q = axis_point(points, normals, axis, centroid)?;
    let (mut z, mut r) = (
        Vec::with_capacity(points.len()),
        Vec::with_capacity(points.len()),
    );
    for p in points {
        let d = *p - q;
        let along = d.dot(axis);
        z.push(along);
        r.push((d - axis * along).length());
    }
    let n = points.len() as f64;
    let mean_r = r.iter().sum::<f64>() / n;
    if mean_r <= TINY {
        return None;
    }
    let tolerance = REVOLUTION_REL_TOL * mean_r;
    let radius_residual = (r.iter().map(|ri| (ri - mean_r).powi(2)).sum::<f64>() / n).sqrt();
    let accepted = radius_residual <= tolerance || {
        // A cone: the meridian is a straight line.
        let mean_z = z.iter().sum::<f64>() / n;
        let szz: f64 = z.iter().map(|zi| (zi - mean_z).powi(2)).sum();
        let span =
            z.iter().copied().fold(f64::MIN, f64::max) - z.iter().copied().fold(f64::MAX, f64::min);
        span > 1e-6 * extent && szz > 0.0 && {
            let szr: f64 = z
                .iter()
                .zip(&r)
                .map(|(zi, ri)| (zi - mean_z) * (ri - mean_r))
                .sum();
            let alpha = szr / szz;
            let beta = mean_r - alpha * mean_z;
            let line = (z.iter().zip(&r))
                .map(|(zi, ri)| (ri - alpha * zi - beta).powi(2))
                .sum::<f64>()
                / n;
            line.sqrt() <= tolerance
        }
    };
    if !accepted {
        return None;
    }
    // The outside of a shaft turns its normals away from the axis.
    let mut sum = 0.0;
    for (p, normal) in points.iter().zip(normals) {
        let d = *p - q;
        if let Some(radial) = (d - axis * d.dot(axis)).try_normalize() {
            sum += radial.dot(*normal);
        }
    }
    Some(Shape::Axis {
        axis,
        radius: mean_r,
        convex: sum >= 0.0,
    })
}

/// The point of the axis nearest to the centroid: the lines along the node normals pass
/// through the axis, weighted by how far they are from parallel to it.
fn axis_point(points: &[DVec3], normals: &[DVec3], axis: DVec3, centroid: DVec3) -> Option<DVec3> {
    let mut m = DMat3::ZERO;
    let mut rhs = DVec3::ZERO;
    let mut count = 0;
    for (p, normal) in points.iter().zip(normals) {
        let k = normal.cross(axis);
        if k.abs().element_sum() < 1e-8 {
            continue;
        }
        m += outer(k, k);
        rhs += k * k.dot(*p);
        count += 1;
    }
    if count < 3 {
        return None;
    }
    // The matrix is singular along the axis; fix that component at the centroid.
    let scale = (m.x_axis.x + m.y_axis.y + m.z_axis.z) / 3.0;
    if scale <= 1e-30 {
        return None;
    }
    m += outer(axis, axis) * scale;
    rhs += axis * (scale * axis.dot(centroid));
    if m.determinant().abs() <= 1e-30 {
        return None;
    }
    Some(m.inverse() * rhs)
}

/// An algebraic (Kasa) sphere fit: |p - c|² = R² is linear in c and R² - |c|².
fn is_sphere(points: &[DVec3], centroid: DVec3, extent: f64) -> bool {
    if points.len() < 4 {
        return false;
    }
    let mut m = [[0.0; 4]; 4];
    let mut rhs = [0.0; 4];
    for p in points {
        let d = (*p - centroid) / extent;
        let row = [2.0 * d.x, 2.0 * d.y, 2.0 * d.z, 1.0];
        let value = d.length_squared();
        for j in 0..4 {
            for k in 0..4 {
                m[j][k] += row[j] * row[k];
            }
            rhs[j] += row[j] * value;
        }
    }
    let Some(c) = solve4(m, rhs) else {
        return false;
    };
    let radius2 = c[3] + c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
    if radius2 <= 0.0 {
        return false;
    }
    let radius = radius2.sqrt() * extent;
    let center = centroid + DVec3::new(c[0], c[1], c[2]) * extent;
    let residual = (points
        .iter()
        .map(|p| (p.distance(center) - radius).powi(2))
        .sum::<f64>()
        / points.len() as f64)
        .sqrt();
    residual <= FIT_REL_TOL * radius.min(extent)
}

/// Gaussian elimination with partial pivoting.
fn solve4(mut m: [[f64; 4]; 4], mut rhs: [f64; 4]) -> Option<[f64; 4]> {
    for col in 0..4 {
        let pivot = (col..4).max_by(|&a, &b| m[a][col].abs().total_cmp(&m[b][col].abs()))?;
        if m[pivot][col].abs() < 1e-14 {
            return None;
        }
        m.swap(col, pivot);
        rhs.swap(col, pivot);
        let pivot_row = m[col];
        for row in col + 1..4 {
            let f = m[row][col] / pivot_row[col];
            for (value, pivot_value) in m[row].iter_mut().zip(pivot_row).skip(col) {
                *value -= f * pivot_value;
            }
            rhs[row] -= f * rhs[col];
        }
    }
    let mut x = [0.0; 4];
    for row in (0..4).rev() {
        let sum: f64 = (row + 1..4).map(|k| m[row][k] * x[k]).sum();
        x[row] = (rhs[row] - sum) / m[row][row];
    }
    Some(x)
}

fn outer(a: DVec3, b: DVec3) -> DMat3 {
    DMat3::from_cols(a * b.x, a * b.y, a * b.z)
}

/// Eigenvalues of a symmetric 3×3 matrix in descending order with their unit eigenvectors
/// (Jacobi rotations).
fn symmetric_eigen(m: DMat3) -> ([f64; 3], [DVec3; 3]) {
    let mut a = [
        m.x_axis.to_array(),
        m.y_axis.to_array(),
        m.z_axis.to_array(),
    ];
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let off = a[0][1].powi(2) + a[0][2].powi(2) + a[1][2].powi(2);
        if off < 1e-30 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-300 {
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
            for k in 0..3 {
                a[p][k] = c * row_p[k] - s * row_q[k];
                a[q][k] = s * row_p[k] + c * row_q[k];
            }
            for row in &mut v {
                let (vp, vq) = (row[p], row[q]);
                row[p] = c * vp - s * vq;
                row[q] = s * vp + c * vq;
            }
        }
    }
    let mut order = [0, 1, 2];
    order.sort_by(|&i, &j| a[j][j].total_cmp(&a[i][i]));
    let values = order.map(|i| a[i][i]);
    let vectors = order.map(|i| DVec3::new(v[0][i], v[1][i], v[2][i]).normalize_or_zero());
    (values, vectors)
}

/// The step offsets of every group.
fn place(graph: &Graph, direction: Direction) -> Vec<Vec<DVec3>> {
    let clusters = &graph.clusters;
    let mut assembly_box = Aabb::EMPTY;
    for c in clusters {
        assembly_box.include(&c.bounds);
    }
    let diagonal = assembly_box.diagonal();
    let center = assembly_box.center();
    let root = root_cluster(graph);
    let mut offsets: Vec<Option<DVec3>> = vec![None; clusters.len()];
    let mut steps: Vec<Vec<DVec3>> = vec![Vec::new(); clusters.len()];
    let mut placed = vec![clusters[root].bounds];
    // The root does not move and takes no step of its own.
    offsets[root] = Some(DVec3::ZERO);
    let fasteners = fastener_flags(graph);
    let parents = spanning_tree(graph, root, &fasteners);
    for (parent, child) in placement_order(graph, root, &parents, direction, center) {
        let parent_offset = offsets[parent].unwrap_or(DVec3::ZERO);
        let travel_direction = travel_direction(graph, parent, child, direction, center);
        let travel = travel(
            &clusters[child],
            parent_offset,
            travel_direction,
            &placed,
            diagonal,
        );
        let parent_steps = steps[parent].clone();
        place_cluster(
            child,
            &clusters[child],
            parent_offset,
            parent_steps,
            travel_direction.map(|d| d * travel),
            &mut offsets,
            &mut steps,
            &mut placed,
        );
    }
    // The groups no interface connects to the root, from the largest.
    let mut unplaced: Vec<usize> = (0..clusters.len())
        .filter(|&c| offsets[c].is_none())
        .collect();
    unplaced.sort_by(|&a, &b| {
        clusters[b]
            .volume()
            .total_cmp(&clusters[a].volume())
            .then_with(|| clusters[a].name.cmp(&clusters[b].name))
    });
    for c in unplaced {
        let radial = radial_direction(&clusters[c], center, center);
        let travel_direction = radial.and_then(|d| filtered(d, direction));
        let travel = travel(
            &clusters[c],
            DVec3::ZERO,
            travel_direction,
            &placed,
            diagonal,
        );
        place_cluster(
            c,
            &clusters[c],
            DVec3::ZERO,
            Vec::new(),
            travel_direction.map(|d| d * travel),
            &mut offsets,
            &mut steps,
            &mut placed,
        );
    }
    steps
}

#[allow(clippy::too_many_arguments)]
fn place_cluster(
    index: usize,
    cluster: &Cluster,
    parent_offset: DVec3,
    mut parent_steps: Vec<DVec3>,
    step: Option<DVec3>,
    offsets: &mut [Option<DVec3>],
    steps: &mut [Vec<DVec3>],
    placed: &mut Vec<Aabb>,
) {
    let step = step.filter(|s| *s != DVec3::ZERO).unwrap_or(DVec3::ZERO);
    let offset = parent_offset + step;
    offsets[index] = Some(offset);
    parent_steps.push(step);
    steps[index] = parent_steps;
    let mut moved = cluster.bounds;
    moved.translate(offset);
    placed.push(moved);
}

/// The group with the most interfaces, the largest of them on a tie.
fn root_cluster(graph: &Graph) -> usize {
    let clusters = &graph.clusters;
    let mut root = 0;
    for c in 1..clusters.len() {
        let (degree, best) = (graph.degree(c), graph.degree(root));
        if degree > best || (degree == best && is_larger(&clusters[c], &clusters[root])) {
            root = c;
        }
    }
    root
}

fn is_larger(a: &Cluster, b: &Cluster) -> bool {
    match a.volume().total_cmp(&b.volume()) {
        std::cmp::Ordering::Greater => true,
        std::cmp::Ordering::Less => false,
        std::cmp::Ordering::Equal => a.name < b.name,
    }
}

/// Bolts, pins and washers: threaded through more than one part while small next to the
/// largest of them.
fn fastener_flags(graph: &Graph) -> Vec<bool> {
    (0..graph.clusters.len())
        .map(|c| {
            let mut axial = 0;
            let mut max_volume: f64 = 0.0;
            for interface in graph.interfaces_of(c).filter(|i| i.axial) {
                axial += 1;
                max_volume = max_volume.max(graph.clusters[interface.other(c)].bounds.volume());
            }
            axial >= MIN_FASTENER_AXIAL_INTERFACES
                && graph.clusters[c].bounds.volume() < MAX_FASTENER_VOLUME_FACTOR * max_volume
        })
        .collect()
}

/// The group every group hangs on, grown from the root one interface at a time, always along
/// the strongest interface left.
fn spanning_tree(graph: &Graph, root: usize, fasteners: &[bool]) -> Vec<Option<usize>> {
    let clusters = &graph.clusters;
    let mut parents = vec![None; clusters.len()];
    let mut in_tree = vec![false; clusters.len()];
    in_tree[root] = true;
    let mut tree = vec![root];
    loop {
        let mut best: Option<(&Interface, usize, usize)> = None;
        for &parent in &tree {
            for interface in graph.interfaces_of(parent) {
                let child = interface.other(parent);
                if in_tree[child] {
                    continue;
                }
                let stronger = best.is_none_or(|(best_interface, best_parent, best_child)| {
                    is_stronger(
                        graph,
                        (interface, parent, child),
                        (best_interface, best_parent, best_child),
                        fasteners,
                    )
                });
                if stronger {
                    best = Some((interface, parent, child));
                }
            }
        }
        let Some((_, parent, child)) = best else {
            break;
        };
        parents[child] = Some(parent);
        in_tree[child] = true;
        tree.push(child);
    }
    parents
}

fn is_stronger(
    graph: &Graph,
    (interface, parent, child): (&Interface, usize, usize),
    (best, best_parent, best_child): (&Interface, usize, usize),
    fasteners: &[bool],
) -> bool {
    let clusters = &graph.clusters;
    let rank = interface_rank(graph, interface, parent, child, fasteners[parent]);
    let best_rank = interface_rank(graph, best, best_parent, best_child, fasteners[best_parent]);
    if rank != best_rank {
        return rank > best_rank;
    }
    // Of two parts that could hold this one, the larger is the casing it is mounted to.
    let (volume, best_volume) = (clusters[parent].volume(), clusters[best_parent].volume());
    if volume != best_volume {
        return volume > best_volume;
    }
    let area = round_significant(interface.area(clusters), 6);
    let best_area = round_significant(best.area(clusters), 6);
    if area != best_area {
        return area > best_area;
    }
    clusters[child].name < clusters[best_child].name
}

/// How well an interface holds the child on the parent: a shaft of the parent through the
/// child first, then the child inserted into a hole of the parent, then any contact with a
/// direction. A fastener carries only parts no larger than itself.
fn interface_rank(
    graph: &Graph,
    interface: &Interface,
    parent: usize,
    child: usize,
    parent_is_fastener: bool,
) -> u8 {
    let clusters = &graph.clusters;
    if parent_is_fastener && clusters[child].volume() > clusters[parent].volume() {
        0
    } else if interface.direction.is_none() {
        1
    } else if !interface.is_axial_mount() {
        2
    } else if interface.shaft == Some(parent) {
        4
    } else {
        3
    }
}

/// Parent and child of every tree edge, level by level so that a parent is placed before its
/// children.
fn placement_order(
    graph: &Graph,
    root: usize,
    parents: &[Option<usize>],
    direction: Direction,
    center: DVec3,
) -> Vec<(usize, usize)> {
    let mut children = vec![Vec::new(); parents.len()];
    for (child, parent) in parents.iter().enumerate() {
        if let Some(parent) = parent {
            children[*parent].push(child);
        }
    }
    let mut order = Vec::new();
    let mut queue = std::collections::VecDeque::from([root]);
    while let Some(c) = queue.pop_front() {
        for child in sorted_children(graph, c, &children[c], direction, center) {
            queue.push_back(child);
            order.push((c, child));
        }
    }
    order
}

/// The children grouped by the direction they travel along and within a group in the order
/// they are stacked along it, so that a stack keeps its order when spread out.
fn sorted_children(
    graph: &Graph,
    parent: usize,
    children: &[usize],
    direction: Direction,
    center: DVec3,
) -> Vec<usize> {
    let key = |child: usize| -> [f64; 5] {
        match travel_direction(graph, parent, child, direction, center) {
            Some(d) => {
                let rounded = d.to_array().map(|c| {
                    let f = 10f64.powi(DIRECTION_GROUP_DIGITS);
                    (c * f).round() / f
                });
                let (min, max) = graph.clusters[child].bounds.projection(d);
                [rounded[0], rounded[1], rounded[2], max, -min]
            }
            None => [0.0; 5],
        }
    };
    let mut sorted = children.to_vec();
    sorted.sort_by(|&a, &b| {
        let (ka, kb) = (key(a), key(b));
        (ka.iter().zip(&kb))
            .map(|(x, y)| x.total_cmp(y))
            .find(|o| o.is_ne())
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| graph.clusters[a].name.cmp(&graph.clusters[b].name))
    });
    sorted
}

fn travel_direction(
    graph: &Graph,
    parent: usize,
    child: usize,
    direction: Direction,
    center: DVec3,
) -> Option<DVec3> {
    let along_interface = graph
        .interface(parent, child)
        .and_then(|i| i.direction_of(child))
        .and_then(|d| filtered(d, direction));
    along_interface.or_else(|| {
        let from = graph.clusters[parent].bounds.center();
        radial_direction(&graph.clusters[child], from, center).and_then(|d| filtered(d, direction))
    })
}

fn radial_direction(cluster: &Cluster, from: DVec3, assembly_center: DVec3) -> Option<DVec3> {
    let c = cluster.bounds.center();
    [c - from, c - assembly_center]
        .into_iter()
        .find(|d| d.length_squared() > TINY)
}

fn filtered(v: DVec3, direction: Direction) -> Option<DVec3> {
    let f = direction.filter(v);
    (f.length_squared() > TINY).then(|| f.normalize())
}

/// How far the group moves to clear all placed boxes, plus some clearance.
fn travel(
    cluster: &Cluster,
    parent_offset: DVec3,
    direction: Option<DVec3>,
    placed: &[Aabb],
    assembly_diagonal: f64,
) -> f64 {
    let Some(direction) = direction else {
        return 0.0;
    };
    let mut moving = cluster.bounds;
    moving.translate(parent_offset);
    let clear = placed
        .iter()
        .map(|p| separation(&moving, p, direction))
        .filter(|s| s.is_finite())
        .fold(0.0, f64::max);
    clear
        + (CLEARANCE_FACTOR * cluster.bounds.diagonal())
            .max(MIN_CLEARANCE_FACTOR * assembly_diagonal)
}

/// The smallest distance along the direction at which the moving box stops overlapping the
/// fixed one, or infinity when the direction never separates them.
fn separation(moving: &Aabb, fixed: &Aabb, direction: DVec3) -> f64 {
    let mut best = f64::INFINITY;
    for axis in 0..3 {
        let component = direction[axis];
        let s = if component > TINY {
            (fixed.max[axis] - moving.min[axis]) / component
        } else if component < -TINY {
            (fixed.min[axis] - moving.max[axis]) / component
        } else if moving.max[axis] <= fixed.min[axis] || moving.min[axis] >= fixed.max[axis] {
            0.0
        } else {
            f64::INFINITY
        };
        best = best.min(s.max(0.0));
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exploded::{Method, Parameters, PartShape};
    use plx_mesh::{Element, ElementShape, FeMesh, Part, extract_part_skin};

    /// A hex mesh of a ring (or a disc when `r0` is zero, with a tiny hole) around the z
    /// axis, from `z0` to `z1`, added to the mesh as a part.
    fn ring(mesh: &mut FeMesh, name: &str, r: (f64, f64), z: (f64, f64)) {
        let (nr, nt, nz) = (2, 24, 2);
        let base = mesh.node_count() as u32;
        let id =
            |i: usize, j: usize, k: usize| base + 1 + (k * (nr + 1) * nt + j * (nr + 1) + i) as u32;
        for k in 0..=nz {
            for j in 0..nt {
                for i in 0..=nr {
                    let radius = r.0 + (r.1 - r.0) * i as f64 / nr as f64;
                    let angle = std::f64::consts::TAU * j as f64 / nt as f64;
                    let height = z.0 + (z.1 - z.0) * k as f64 / nz as f64;
                    mesh.set_node(
                        id(i, j, k),
                        [radius * angle.cos(), radius * angle.sin(), height],
                    );
                }
            }
        }
        let first = mesh.element_count() as u32 + 1;
        let mut elements = Vec::new();
        for k in 0..nz {
            for j in 0..nt {
                let jn = (j + 1) % nt;
                for i in 0..nr {
                    let e = first + elements.len() as u32;
                    mesh.add_element(Element {
                        id: e,
                        type_name: "C3D8".into(),
                        shape: ElementShape::Hex8,
                        nodes: vec![
                            id(i, j, k),
                            id(i + 1, j, k),
                            id(i + 1, jn, k),
                            id(i, jn, k),
                            id(i, j, k + 1),
                            id(i + 1, j, k + 1),
                            id(i + 1, jn, k + 1),
                            id(i, jn, k + 1),
                        ],
                    })
                    .unwrap();
                    elements.push(e);
                }
            }
        }
        mesh.parts.push(Part {
            bodies: Vec::new(),
            name: name.into(),
            elements,
        });
    }

    /// Two plates with a bore, a bolt through both and a nut below them.
    fn bolted_joint() -> FeMesh {
        let mut mesh = FeMesh::default();
        ring(&mut mesh, "PLATE_A", (5.0, 30.0), (0.0, 5.0));
        ring(&mut mesh, "PLATE_B", (5.0, 30.0), (5.0, 10.0));
        ring(&mut mesh, "BOLT", (0.5, 5.0), (-4.0, 13.0));
        ring(&mut mesh, "NUT", (5.0, 9.0), (-4.0, 0.0));
        mesh
    }

    fn offsets(mesh: &FeMesh, parameters: &Parameters) -> Vec<DVec3> {
        let skins: Vec<_> = (mesh.parts.iter())
            .map(|p| extract_part_skin(mesh, p, 30.0))
            .collect();
        let nodes: Vec<Vec<usize>> = (mesh.parts.iter())
            .map(|p| {
                let mut nodes: Vec<usize> = (p.elements.iter())
                    .flat_map(|&e| mesh.element(e).unwrap().nodes.clone())
                    .map(|id| mesh.node_index(id).unwrap())
                    .collect();
                nodes.sort_unstable();
                nodes.dedup();
                nodes
            })
            .collect();
        let assembly = Assembly {
            mesh,
            parts: (mesh.parts.iter().enumerate())
                .map(|(i, p)| PartShape {
                    name: &p.name,
                    nodes: &nodes[i],
                    skin: &skins[i],
                })
                .collect(),
        };
        assembly
            .layout(parameters)
            .offsets(parameters.scale(), parameters.sequential)
    }

    #[test]
    fn bolted_joint_comes_apart_along_the_bolt() {
        let mesh = bolted_joint();
        let parameters = Parameters {
            scale_factor: 1.0,
            magnification: 1.0,
            ..Parameters::default()
        };
        let offsets = offsets(&mesh, &parameters);
        // Every part moves along the bolt axis only.
        for offset in &offsets {
            assert!(
                offset.x.abs() < 1e-6 && offset.y.abs() < 1e-6,
                "{offsets:?}"
            );
        }
        let z: Vec<f64> = offsets.iter().map(|o| o.z).collect();
        let [a, b, bolt, nut] = [z[0], z[1], z[2], z[3]];
        // One part stays, and the stack keeps its order: nut below plate A below plate B.
        assert!(z.contains(&0.0), "{z:?}");
        assert!(nut < a && a < b, "{z:?}");
        // The bolt comes out of the bores entirely: its bottom clears the top plate's top.
        assert!(bolt != a && bolt != b, "{z:?}");
        assert_ne!(offsets, vec![DVec3::ZERO; 4]);
    }

    #[test]
    fn falls_back_to_assembly_center_without_contact() {
        let mut mesh = FeMesh::default();
        ring(&mut mesh, "A", (5.0, 10.0), (0.0, 1.0));
        ring(&mut mesh, "B", (5.0, 10.0), (5.0, 6.0));
        let disassembly = offsets(&mesh, &Parameters::default());
        let center = offsets(
            &mesh,
            &Parameters {
                method: Method::AssemblyCenter,
                ..Parameters::default()
            },
        );
        assert_eq!(disassembly, center);
    }

    #[test]
    fn recognizes_bores_and_planes() {
        let mut mesh = FeMesh::default();
        ring(&mut mesh, "PLATE", (5.0, 30.0), (0.0, 5.0));
        let skin = extract_part_skin(&mesh, &mesh.parts[0], 30.0);
        let coords = mesh.coords();
        let mut shapes = Vec::new();
        let regions = skin.faces.iter().map(|f| f.region).max().unwrap() + 1;
        for region in 0..regions {
            let cells: Vec<Vec<usize>> = (skin.faces.iter())
                .filter(|f| f.region == region)
                .map(|f| f.corners.clone())
                .collect();
            let insides = vec![None; cells.len()];
            shapes.push(classify(coords, &Face::new(coords, cells, insides, 0.01)));
        }
        let planes = shapes.iter().filter(|s| **s == Shape::Plane).count();
        let axes: Vec<_> = (shapes.iter())
            .filter_map(|s| match s {
                Shape::Axis { axis, radius, .. } => Some((*axis, *radius)),
                _ => None,
            })
            .collect();
        assert_eq!(planes, 2, "{shapes:?}");
        assert_eq!(axes.len(), 2, "{shapes:?}");
        for (axis, radius) in axes {
            assert!(axis.z.abs() > 0.999, "{axis}");
            assert!(radius > 4.9, "{radius}");
        }
    }

    #[test]
    fn eigen_decomposition_sorts_descending() {
        let m = DMat3::from_cols(
            DVec3::new(2.0, 1.0, 0.0),
            DVec3::new(1.0, 2.0, 0.0),
            DVec3::new(0.0, 0.0, 5.0),
        );
        let (values, vectors) = symmetric_eigen(m);
        assert!((values[0] - 5.0).abs() < 1e-9);
        assert!((values[1] - 3.0).abs() < 1e-9);
        assert!((values[2] - 1.0).abs() < 1e-9);
        assert!((vectors[0].z.abs() - 1.0).abs() < 1e-9);
        let expected = DVec3::new(1.0, -1.0, 0.0).normalize();
        assert!((vectors[2].dot(expected).abs() - 1.0).abs() < 1e-9);
    }
}
