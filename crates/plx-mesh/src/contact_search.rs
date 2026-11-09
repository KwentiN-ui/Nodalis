//! The search behind the contact detection: finds surfaces of different parts that lie on each
//! other and decides which side is master and which slave, following
//! `CaeMesh.ContactSearch` and `ContactGraph`.
//!
//! Surfaces are the smooth patches of the part skins, which stand for the CAD faces the way
//! geometry surfaces do: of solids their outer side, of shells and membranes the
//! top and the bottom side, and the free edges of shells in chains between corners. In 2D
//! models the surfaces are chains of outline edges of the plane elements, split at corners,
//! the way the solver takes the edges as faces. Two surfaces are in contact where triangles of
//! both face each other within the search distance and angle; an edge is searched as a strip
//! across it, perpendicular to its shell, or along z in 2D. The pairs found are grouped (by parts,
//! by the contact graph or not at all). Master is the side with the coarser mesh, then the
//! stiffer material, then the larger area, the usual rule for CalculiX contact; a surface
//! touching several better masters is the slave of all of them at once, so that it is a
//! slave only once, which the solver requires of ties.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};

use crate::element::ElementFamily;
use crate::mesh::{ElementId, FeMesh};
use crate::skin::PartSkin;

/// A surface patch: part index, which side or edge of the part, and the number of the patch
/// (the region of the part's skin, or the chain of edges).
pub type SurfaceId = (usize, SurfaceKind, usize);

/// What a contact surface is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SurfaceKind {
    /// The outer side of solid elements.
    Solid,
    /// The top (positive normal, CalculiX face S2) of shells and membranes.
    ShellTop,
    /// Their bottom (S1).
    ShellBottom,
    /// Free edges of shells and membranes (S3 and up), and the outline of 2D elements.
    ShellEdge,
}

impl SurfaceKind {
    /// The name in the search's table.
    pub fn label(self) -> &'static str {
        match self {
            SurfaceKind::Solid => "Solid",
            SurfaceKind::ShellTop | SurfaceKind::ShellBottom => "Shell",
            SurfaceKind::ShellEdge => "Shell edge",
        }
    }
}

/// Which surfaces are searched.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GeometryFilter {
    pub solid: bool,
    /// Top and bottom of shells and membranes.
    pub shell: bool,
    /// Free edges of shells and membranes, and the outline of 2D elements.
    pub shell_edge: bool,
}

impl Default for GeometryFilter {
    fn default() -> Self {
        Self {
            solid: true,
            shell: true,
            shell_edge: true,
        }
    }
}

impl GeometryFilter {
    fn takes(&self, kind: SurfaceKind) -> bool {
        match kind {
            SurfaceKind::Solid => self.solid,
            SurfaceKind::ShellTop | SurfaceKind::ShellBottom => self.shell,
            SurfaceKind::ShellEdge => self.shell_edge,
        }
    }
}

/// How the pairs of touching surfaces become contact pairs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum GroupBy {
    /// One contact pair per pair of touching surfaces.
    None,
    /// One contact pair per pair of touching parts.
    #[default]
    Parts,
    /// Surfaces sharing mesh nodes are merged, so that a node is a slave only once.
    Graph,
}

/// Search parameters of the dialog.
#[derive(Clone, Debug, PartialEq)]
pub struct SearchParameters {
    /// Largest gap between touching surfaces.
    pub distance: f64,
    /// Largest deviation of the surfaces from facing each other, in degrees.
    pub angle_deg: f64,
    pub group_by: GroupBy,
    /// Young's modulus of each part's material, where known, for choosing the master.
    pub stiffness: Vec<Option<f64>>,
    pub filter: GeometryFilter,
    /// Of each part, whether its shell faces are the skin of a solid with their normals
    /// pointing out of it (the display of CAD solids, see [`crate::orient_outward`]): only
    /// their top side can touch, so that faces of two solids lying side by side in one
    /// plane, e.g. on a plane of symmetry, do not count as touching. Parts left out are
    /// two-sided.
    pub one_sided: Vec<bool>,
    /// Of each part, whether its shell faces are the skin of a solid (the display of CAD
    /// solids): the borders of its faces are no free edges of shells and are not searched.
    /// Drawn out across their faces like the edges of shells, they would reach into the
    /// faces of the neighbours beside a face and join faces that do not touch, e.g. two
    /// faces of one part meeting at an edge, into the pairs. Parts left out have edges.
    pub solid_skins: Vec<bool>,
}

/// A master and a slave side of a contact pair found, each a set of surfaces.
#[derive(Clone, Debug, PartialEq)]
pub struct MasterSlaveItem {
    pub master_name: String,
    pub slave_name: String,
    pub master: BTreeSet<SurfaceId>,
    pub slave: BTreeSet<SurfaceId>,
}

impl MasterSlaveItem {
    /// The name of the pair, `<master>_to_<slave>`, shortened to 35 characters.
    pub fn name(&self) -> String {
        let join = |a: &str, b: &str| format!("{a}_to_{b}");
        let mut name = join(&self.master_name, &self.slave_name);
        if name.len() > 35 {
            name = if self.master_name.len() > self.slave_name.len() {
                join("Item", &self.slave_name)
            } else {
                join(&self.master_name, "Item")
            };
        }
        if name.len() > 35 {
            name = join("Item", "Item");
        }
        name
    }

    /// What touches, e.g. "Shell edge-Solid": the kind of the
    /// master's and of the slave's first surface.
    pub fn geometry(&self) -> String {
        let kind = |side: &BTreeSet<SurfaceId>| side.first().map_or("", |id| id.1.label());
        format!("{}-{}", kind(&self.master), kind(&self.slave))
    }
}

/// One surface patch prepared for the search.
struct Surface {
    id: SurfaceId,
    /// Corner points of each face, normals pointing out of the material. An edge is drawn
    /// out into a quadrilateral, so that it is searched like a face: across its shell, or
    /// along z in 2D.
    faces: Vec<Vec<[f64; 3]>>,
    /// Bounds of each face, enlarged by half the search distance.
    face_bounds: Vec<Bounds>,
    bounds: Bounds,
    nodes: BTreeSet<usize>,
    /// Sum over the faces of their mean edge length, for the mesh size of the surface.
    edge_lengths: f64,
    /// The other side of a surface shared with another part, which is no contact.
    internal: bool,
}

#[derive(Clone, Copy, Debug)]
struct Bounds {
    min: [f64; 3],
    max: [f64; 3],
}

impl Bounds {
    fn empty() -> Self {
        Self {
            min: [f64::INFINITY; 3],
            max: [f64::NEG_INFINITY; 3],
        }
    }

    fn include(&mut self, p: [f64; 3]) {
        for (k, value) in p.into_iter().enumerate() {
            self.min[k] = self.min[k].min(value);
            self.max[k] = self.max[k].max(value);
        }
    }

    fn union(&mut self, other: &Bounds) {
        self.include(other.min);
        self.include(other.max);
    }

    fn inflate(&mut self, by: f64) {
        for k in 0..3 {
            self.min[k] -= by;
            self.max[k] += by;
        }
    }

    fn intersects(&self, other: &Bounds) -> bool {
        (0..3).all(|k| self.min[k] <= other.max[k] && other.min[k] <= self.max[k])
    }

    fn intersection(&self, other: &Bounds) -> Bounds {
        let mut out = *self;
        for k in 0..3 {
            out.min[k] = self.min[k].max(other.min[k]);
            out.max[k] = self.max[k].min(other.max[k]);
        }
        out
    }
}

/// Contact pairs between the solid surfaces of the parts marked in `searched`; `skins` are
/// the skins of all parts of the mesh, in the order of its parts.
pub fn find_contact_pairs(
    mesh: &FeMesh,
    skins: &[PartSkin],
    searched: &[bool],
    parameters: &SearchParameters,
) -> Vec<MasterSlaveItem> {
    let surfaces = contact_surfaces(mesh, skins, searched, parameters);
    let angle = parameters.angle_deg.to_radians();
    let mut touching = Vec::new();
    for i in 0..surfaces.len() {
        for j in i + 1..surfaces.len() {
            if surfaces_touch(&surfaces[i], &surfaces[j], parameters.distance, angle) {
                touching.push((surfaces[i].id, surfaces[j].id));
            }
        }
    }
    let areas = surface_areas(&surfaces);
    let names: Vec<&str> = mesh.parts.iter().map(|p| p.name.as_str()).collect();
    let nodes: HashMap<SurfaceId, BTreeSet<usize>> =
        (surfaces.iter()).map(|s| (s.id, s.nodes.clone())).collect();
    let mesh_sizes: HashMap<SurfaceId, f64> = (surfaces.iter())
        .map(|s| (s.id, s.edge_lengths / s.faces.len().max(1) as f64))
        .collect();
    let context = Context {
        nodes: &nodes,
        areas: &areas,
        mesh_sizes: &mesh_sizes,
        stiffness: &parameters.stiffness,
        names: &names,
    };
    match parameters.group_by {
        GroupBy::None => {
            let items = (touching.iter())
                .map(|&(a, b)| item_of(&context, [a].into(), [b].into()))
                .collect::<Vec<_>>();
            ContactGraph::new(&context, &items, false).master_slave_items()
        }
        GroupBy::Parts => {
            let mut by_parts: BTreeMap<_, (BTreeSet<SurfaceId>, BTreeSet<SurfaceId>)> =
                BTreeMap::new();
            for &(a, b) in &touching {
                // Sides by part order; the contact graph decides which is master. Edges of
                // shells touching a part make a pair of their own.
                let (master, slave) = if a.0 < b.0 { (a, b) } else { (b, a) };
                let edge = |id: SurfaceId| id.1 == SurfaceKind::ShellEdge;
                let key = (master.0, edge(master), slave.0, edge(slave));
                let entry = by_parts.entry(key).or_default();
                entry.0.insert(master);
                entry.1.insert(slave);
            }
            let items = (by_parts.into_values())
                .map(|(master, slave)| item_of(&context, master, slave))
                .collect::<Vec<_>>();
            ContactGraph::new(&context, &items, false).master_slave_items()
        }
        GroupBy::Graph => {
            let items = (touching.iter())
                .map(|&(a, b)| item_of(&context, [a].into(), [b].into()))
                .collect::<Vec<_>>();
            ContactGraph::new(&context, &items, true).grouped_master_slave_items()
        }
    }
}

/// Element faces (element id, CalculiX face number) of surface patches.
pub fn surface_faces(
    mesh: &FeMesh,
    skins: &[PartSkin],
    surfaces: &BTreeSet<SurfaceId>,
) -> Vec<(ElementId, u8)> {
    let mut faces: Vec<(ElementId, u8)> = (patch_faces(mesh, skins).into_iter())
        .filter(|f| surfaces.contains(&f.surface))
        .map(|f| (f.element, f.face))
        .collect();
    faces.sort_unstable();
    faces
}

/// An element face on a contact surface.
struct PatchFace {
    surface: SurfaceId,
    element: ElementId,
    /// CalculiX face number, S1 = 1.
    face: u8,
    /// Corner node indices in order, the normal pointing out of the material; the two ends
    /// of an edge, its outside to the right when looking along the normal of its element.
    corners: Vec<usize>,
    /// Corner and midside node indices.
    nodes: Vec<usize>,
    /// The normal of the shell an edge belongs to, across which it is drawn out; `None`
    /// for faces and for the edges of 2D elements, which are drawn out along z.
    across: Option<[f64; 3]>,
}

/// The faces of all surface patches: those of solid elements from the skins, both sides of
/// shells, and the outline edges of shells and of the plane elements of 2D models.
fn patch_faces(mesh: &FeMesh, skins: &[PartSkin]) -> Vec<PatchFace> {
    let elements = mesh.elements();
    let mut faces = Vec::new();
    for (part, skin) in skins.iter().enumerate() {
        for face in &skin.faces {
            let element = &elements[face.element];
            if face.corners.len() < 3 {
                continue;
            }
            let nodes: Vec<usize> = face.corners.iter().chain(&face.mids).copied().collect();
            if element.shape.family() == ElementFamily::Solid {
                faces.push(PatchFace {
                    surface: (part, SurfaceKind::Solid, face.region),
                    element: element.id,
                    face: face.face as u8 + 1,
                    // Element faces are numbered with their normal into the element.
                    corners: face.corners.iter().rev().copied().collect(),
                    nodes,
                    across: None,
                });
            } else if element.is_shell() {
                // The top faces along the element's normal, the bottom against it.
                faces.push(PatchFace {
                    surface: (part, SurfaceKind::ShellTop, face.region),
                    element: element.id,
                    face: 2,
                    corners: face.corners.clone(),
                    nodes: nodes.clone(),
                    across: None,
                });
                faces.push(PatchFace {
                    surface: (part, SurfaceKind::ShellBottom, face.region),
                    element: element.id,
                    face: 1,
                    corners: face.corners.iter().rev().copied().collect(),
                    nodes,
                    across: None,
                });
            }
        }
    }
    for (part, entry) in mesh.parts.iter().enumerate() {
        faces.extend(outline(mesh, part, &entry.elements));
    }
    faces
}

/// Largest angle between neighbouring outline edges of one surface, like the feature angle
/// of the skins.
const OUTLINE_FEATURE_ANGLE_DEG: f64 = 30.0;

/// The outline edges of the plane elements and of the shells of a part, grouped into chains
/// between corners.
fn outline(mesh: &FeMesh, part: usize, ids: &[ElementId]) -> Vec<PatchFace> {
    let coords = mesh.coords();
    let mut edges: Vec<PatchFace> = Vec::new();
    let mut count: HashMap<(usize, usize), usize> = HashMap::new();
    for element in ids.iter().filter_map(|&id| mesh.element(id)) {
        if !element.is_plane() && !element.is_shell() {
            continue;
        }
        let Some(nodes) = (element.nodes.iter())
            .map(|&id| mesh.node_index(id))
            .collect::<Option<Vec<usize>>>()
        else {
            continue;
        };
        let corners = element.shape.edges().len();
        let corner_nodes = &nodes[..corners];
        // A shell's edges are outward to the right looking along its normal. Of 2D elements
        // the counter-clockwise ones have their outward edge normals to the right of the
        // edge direction; clockwise ones are walked the other way round.
        let (across, forward) = if element.is_shell() {
            (Some(crate::skin::face_normal(coords, corner_nodes)), true)
        } else {
            let area: f64 = (0..corners)
                .map(|k| {
                    let (p, q) = (coords[nodes[k]], coords[nodes[(k + 1) % corners]]);
                    p[0] * q[1] - q[0] * p[1]
                })
                .sum();
            (None, area >= 0.0)
        };
        for (number, edge) in element.edge_faces() {
            let [a, b] = [edge.corners[0], edge.corners[1]].map(|l| nodes[l]);
            *count.entry((a.min(b), a.max(b))).or_default() += 1;
            let mut all = vec![a, b];
            if element.shape.is_quadratic() {
                all.extend(edge.mids.iter().map(|&l| nodes[l]));
            }
            edges.push(PatchFace {
                surface: (part, SurfaceKind::ShellEdge, 0),
                element: element.id,
                face: number,
                corners: if forward { vec![a, b] } else { vec![b, a] },
                nodes: all,
                across: across.map(normalize),
            });
        }
    }
    edges.retain(|e| {
        let (a, b) = (e.corners[0], e.corners[1]);
        count[&(a.min(b), a.max(b))] == 1
    });
    // Neighbouring edges meeting at a small angle belong to the same surface.
    let mut parent: Vec<usize> = (0..edges.len()).collect();
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let mut at_node: HashMap<usize, Vec<usize>> = HashMap::new();
    for (i, edge) in edges.iter().enumerate() {
        for &node in &edge.corners {
            at_node.entry(node).or_default().push(i);
        }
    }
    let direction = |e: &PatchFace| normalize(sub(coords[e.corners[1]], coords[e.corners[0]]));
    let limit = OUTLINE_FEATURE_ANGLE_DEG.to_radians().cos();
    for touching in at_node.values() {
        if let [i, j] = touching[..]
            && dot(direction(&edges[i]), direction(&edges[j])) >= limit
        {
            let (ri, rj) = (root(&mut parent, i), root(&mut parent, j));
            parent[ri] = rj;
        }
    }
    let mut numbers: BTreeMap<usize, usize> = BTreeMap::new();
    for (i, edge) in edges.iter_mut().enumerate() {
        let r = root(&mut parent, i);
        let next = numbers.len();
        let number = *numbers.entry(r).or_insert(next);
        edge.surface = (part, SurfaceKind::ShellEdge, number);
    }
    edges
}

/// The surface patches of the searched parts.
fn contact_surfaces(
    mesh: &FeMesh,
    skins: &[PartSkin],
    searched: &[bool],
    parameters: &SearchParameters,
) -> Vec<Surface> {
    let distance = parameters.distance;
    let coords = mesh.coords();
    // Edges of 2D elements are drawn out by a length of the model's size.
    let depth = model_size(coords).max(distance * 10.0);
    let mut patches: BTreeMap<SurfaceId, Surface> = BTreeMap::new();
    for face in patch_faces(mesh, skins) {
        if !searched.get(face.surface.0).copied().unwrap_or(false)
            || !parameters.filter.takes(face.surface.1)
            || face.surface.1 == SurfaceKind::ShellBottom
                && parameters.one_sided.get(face.surface.0) == Some(&true)
            || face.surface.1 == SurfaceKind::ShellEdge
                && parameters.solid_skins.get(face.surface.0) == Some(&true)
        {
            continue;
        }
        let surface = patches.entry(face.surface).or_insert_with(|| Surface {
            id: face.surface,
            faces: Vec::new(),
            face_bounds: Vec::new(),
            bounds: Bounds::empty(),
            nodes: BTreeSet::new(),
            edge_lengths: 0.0,
            internal: false,
        });
        let mut points: Vec<[f64; 3]> = face.corners.iter().map(|&n| coords[n]).collect();
        surface.edge_lengths += mean_edge_length(&points);
        if let [a, b] = points[..] {
            points = match face.across {
                // A strip as wide as the edge is long, centred on the shell.
                Some(normal) => {
                    let half = scale(normal, 0.5 * length(sub(b, a)));
                    vec![sub(a, half), sub(b, half), add(b, half), add(a, half)]
                }
                None => {
                    let up = |p: [f64; 3]| [p[0], p[1], p[2] + depth];
                    vec![a, b, up(b), up(a)]
                }
            };
        }
        let mut bounds = Bounds::empty();
        for &point in &points {
            bounds.include(point);
        }
        bounds.inflate(distance * 0.5);
        surface.bounds.union(&bounds);
        surface.nodes.extend(&face.nodes);
        surface.faces.push(points);
        surface.face_bounds.push(bounds);
    }
    let mut surfaces: Vec<Surface> = patches.into_values().collect();
    // The same faces in two parts are the interface of parts meshed together; the top and
    // the bottom of a shell are not.
    for i in 0..surfaces.len() {
        for j in i + 1..surfaces.len() {
            let (a, b) = (&surfaces[i], &surfaces[j]);
            if a.id.1 == b.id.1
                && a.nodes.len() == b.nodes.len()
                && a.bounds.intersects(&b.bounds)
                && a.nodes == b.nodes
            {
                surfaces[i].internal = true;
                surfaces[j].internal = true;
            }
        }
    }
    surfaces
}

/// Mean length of the edges of a polygon; of a line, its length.
fn mean_edge_length(points: &[[f64; 3]]) -> f64 {
    match points {
        [] | [_] => 0.0,
        [a, b] => length(sub(*b, *a)),
        _ => {
            let total: f64 = (0..points.len())
                .map(|i| length(sub(points[(i + 1) % points.len()], points[i])))
                .sum();
            total / points.len() as f64
        }
    }
}

/// The diagonal of the bounding box of the nodes.
fn model_size(coords: &[[f64; 3]]) -> f64 {
    let mut bounds = Bounds::empty();
    for &point in coords {
        bounds.include(point);
    }
    if coords.is_empty() {
        return 0.0;
    }
    length(sub(bounds.max, bounds.min))
}

fn surface_areas(surfaces: &[Surface]) -> HashMap<SurfaceId, f64> {
    surfaces
        .iter()
        .map(|s| {
            let area = (s.faces.iter())
                .map(|points| {
                    triangles(points)
                        .map(|t| 0.5 * length(cross(sub(t[1], t[0]), sub(t[2], t[0]))))
                        .sum::<f64>()
                })
                .sum();
            (s.id, area)
        })
        .collect()
}

/// Whether two surfaces face each other somewhere within the distance and angle.
fn surfaces_touch(a: &Surface, b: &Surface, distance: f64, angle: f64) -> bool {
    if a.internal || b.internal || a.id.0 == b.id.0 || !a.bounds.intersects(&b.bounds) {
        return false;
    }
    let common = a.bounds.intersection(&b.bounds);
    for (face_a, bounds_a) in a.faces.iter().zip(&a.face_bounds) {
        if !bounds_a.intersects(&common) {
            continue;
        }
        for (face_b, bounds_b) in b.faces.iter().zip(&b.face_bounds) {
            if !bounds_a.intersects(bounds_b) {
                continue;
            }
            for t1 in triangles(face_a) {
                for t2 in triangles(face_b) {
                    if triangles_touch(&t1, &t2, distance, angle) {
                        return true;
                    }
                }
            }
        }
    }
    false
}

/// The triangles of a face: one, or two of a quadrilateral.
fn triangles(points: &[[f64; 3]]) -> impl Iterator<Item = [[f64; 3]; 3]> + '_ {
    (1..points.len() - 1).map(move |k| [points[0], points[k], points[k + 1]])
}

/// Whether two triangles of solid surfaces touch; solid surfaces which may penetrate each
/// other: the triangles face each other within the angle, are closer than the distance, lie
/// one above the other, and still are after shrinking them, so that touching only at a corner
/// or an edge does not count.
fn triangles_touch(t1: &[[f64; 3]; 3], t2: &[[f64; 3]; 3], distance: f64, angle: f64) -> bool {
    let n1 = normalize(cross(sub(t1[1], t1[0]), sub(t1[2], t1[0])));
    let n2 = normalize(cross(sub(t2[1], t2[0]), sub(t2[2], t2[0])));
    let between = std::f64::consts::PI - dot(n1, n2).clamp(-1.0, 1.0).acos();
    if between >= angle {
        return false;
    }
    let (dist, p, q) = triangle_distance(t1, t2);
    if dist >= distance {
        return false;
    }
    if dist > 0.0 {
        let pq = normalize(sub(q, p));
        // The closest points must lie one above the other, within about 5 degrees.
        if dot(pq, n1).abs() < 0.995 && dot(pq, n2).abs() < 0.995 {
            return false;
        }
    }
    let s1 = shrink(t1, 3.0 * distance);
    let s2 = shrink(t2, 3.0 * distance);
    triangle_distance(&s1, &s2).0 <= distance
}

/// Moves the corners towards the centre by `by`, at most nine tenths of the way.
fn shrink(t: &[[f64; 3]; 3], by: f64) -> [[f64; 3]; 3] {
    let center = scale(add(add(t[0], t[1]), t[2]), 1.0 / 3.0);
    t.map(|p| {
        let to_center = sub(center, p);
        let len = length(to_center);
        let k = if len > 0.0 {
            (by / len).clamp(0.0, 0.9)
        } else {
            0.0
        };
        add(p, scale(to_center, k))
    })
}

/// Distance between two triangles with the closest points on each.
fn triangle_distance(t1: &[[f64; 3]; 3], t2: &[[f64; 3]; 3]) -> (f64, [f64; 3], [f64; 3]) {
    for (a, b) in [(t1, t2), (t2, t1)] {
        for k in 0..3 {
            if let Some(point) = segment_triangle_intersection(a[k], a[(k + 1) % 3], b) {
                return (0.0, point, point);
            }
        }
    }
    let mut best = (f64::INFINITY, t1[0], t2[0]);
    let mut consider = |p: [f64; 3], q: [f64; 3]| {
        let d = length(sub(q, p));
        if d < best.0 {
            best = (d, p, q);
        }
    };
    for i in 0..3 {
        for j in 0..3 {
            let (p, q) = segment_segment(t1[i], t1[(i + 1) % 3], t2[j], t2[(j + 1) % 3]);
            consider(p, q);
        }
        consider(t1[i], point_triangle(t1[i], t2));
        consider(point_triangle(t2[i], t1), t2[i]);
    }
    best
}

/// Where a segment crosses a triangle, if it does.
fn segment_triangle_intersection(p: [f64; 3], q: [f64; 3], t: &[[f64; 3]; 3]) -> Option<[f64; 3]> {
    let dir = sub(q, p);
    let e1 = sub(t[1], t[0]);
    let e2 = sub(t[2], t[0]);
    let h = cross(dir, e2);
    let det = dot(e1, h);
    let scale_len = length(e1) * length(e2) * length(dir);
    if det.abs() <= 1e-12 * scale_len {
        return None;
    }
    let s = sub(p, t[0]);
    let u = dot(s, h) / det;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let qv = cross(s, e1);
    let v = dot(dir, qv) / det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let along = dot(e2, qv) / det;
    (0.0..=1.0)
        .contains(&along)
        .then(|| add(p, scale(dir, along)))
}

/// Closest points of two segments (Ericson, Real-Time Collision Detection 5.1.9).
fn segment_segment(p1: [f64; 3], q1: [f64; 3], p2: [f64; 3], q2: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let d1 = sub(q1, p1);
    let d2 = sub(q2, p2);
    let r = sub(p1, p2);
    let a = dot(d1, d1);
    let e = dot(d2, d2);
    let f = dot(d2, r);
    let (s, t);
    if a <= f64::EPSILON && e <= f64::EPSILON {
        return (p1, p2);
    }
    if a <= f64::EPSILON {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = dot(d1, r);
        if e <= f64::EPSILON {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = dot(d1, d2);
            let denom = a * e - b * b;
            let mut s0 = if denom > 0.0 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t0 = (b * s0 + f) / e;
            if t0 < 0.0 {
                t0 = 0.0;
                s0 = (-c / a).clamp(0.0, 1.0);
            } else if t0 > 1.0 {
                t0 = 1.0;
                s0 = ((b - c) / a).clamp(0.0, 1.0);
            }
            s = s0;
            t = t0;
        }
    }
    (add(p1, scale(d1, s)), add(p2, scale(d2, t)))
}

/// Closest point of a triangle to a point (Ericson 5.1.5).
fn point_triangle(p: [f64; 3], t: &[[f64; 3]; 3]) -> [f64; 3] {
    let [a, b, c] = *t;
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return add(a, scale(ab, d1 / (d1 - d3)));
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return add(a, scale(ac, d2 / (d2 - d6)));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return add(b, scale(sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))));
    }
    let denom = 1.0 / (va + vb + vc);
    add(a, add(scale(ab, vb * denom), scale(ac, vc * denom)))
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: [f64; 3], k: f64) -> [f64; 3] {
    a.map(|c| c * k)
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

fn length(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn normalize(a: [f64; 3]) -> [f64; 3] {
    let len = length(a);
    if len > 0.0 { scale(a, 1.0 / len) } else { a }
}

/// What the grouping needs to know about the surfaces.
struct Context<'a> {
    /// Mesh nodes of each surface patch.
    nodes: &'a HashMap<SurfaceId, BTreeSet<usize>>,
    areas: &'a HashMap<SurfaceId, f64>,
    /// Mean edge length of the faces of each surface patch.
    mesh_sizes: &'a HashMap<SurfaceId, f64>,
    /// Young's modulus of each part, where known.
    stiffness: &'a [Option<f64>],
    names: &'a [&'a str],
}

/// How well a side of a contact suits being the master; see [`Priority::master_first`].
#[derive(Clone, Copy, Debug, PartialEq)]
struct Priority {
    /// Only edges of shells, which are slaves of faces: the edge has a single row of nodes.
    edge: bool,
    /// Mean edge length of the faces, weighted by area.
    mesh_size: f64,
    /// The largest Young's modulus of the parts, 0 if unknown.
    stiffness: f64,
    area: f64,
}

impl Priority {
    /// Mesh sizes within this factor of each other count as the same.
    const MESH_SIZE_STEP: f64 = 1.25;

    /// The mesh size on a logarithmic scale in steps of [`Self::MESH_SIZE_STEP`].
    fn mesh_step(&self) -> i64 {
        if self.mesh_size > 0.0 {
            (self.mesh_size.ln() / Self::MESH_SIZE_STEP.ln()).round() as i64
        } else {
            i64::MIN
        }
    }

    /// Orders the better master first: a face before an edge of a shell, then the coarser mesh, so that the finer mesh's nodes
    /// are the slaves that cannot pass through the master; then the stiffer material; then
    /// the larger area.
    fn master_first(&self, other: &Self) -> std::cmp::Ordering {
        (self.edge.cmp(&other.edge))
            .then(other.mesh_step().cmp(&self.mesh_step()))
            .then(other.stiffness.total_cmp(&self.stiffness))
            .then(other.area.total_cmp(&self.area))
    }
}

impl Context<'_> {
    /// Name of merged items: the part name if all surfaces are of one part, else
    /// the next free "Merged-n".
    fn name(&self, ids: &BTreeSet<SurfaceId>, taken: &[String]) -> String {
        let parts: BTreeSet<usize> = ids.iter().map(|id| id.0).collect();
        if parts.len() == 1 {
            let part = *parts.iter().next().expect("one part");
            return self.names.get(part).copied().unwrap_or("Part").to_string();
        }
        (1..)
            .map(|n| format!("Merged-{n}"))
            .find(|name| !taken.contains(name))
            .expect("unbounded range")
    }

    fn priority(&self, ids: &BTreeSet<SurfaceId>) -> Priority {
        let area_of = |id: &SurfaceId| self.areas.get(id).copied().unwrap_or(0.0);
        let area: f64 = ids.iter().map(area_of).sum();
        let weighted: f64 = (ids.iter())
            .map(|id| area_of(id) * self.mesh_sizes.get(id).copied().unwrap_or(0.0))
            .sum();
        let stiffness = (ids.iter())
            .filter_map(|id| self.stiffness.get(id.0).copied().flatten())
            .fold(0.0, f64::max);
        Priority {
            edge: !ids.is_empty() && ids.iter().all(|id| id.1 == SurfaceKind::ShellEdge),
            mesh_size: if area > 0.0 { weighted / area } else { 0.0 },
            stiffness,
            area,
        }
    }

    /// Mesh nodes of a surface patch.
    fn nodes(&self, id: SurfaceId) -> BTreeSet<usize> {
        self.nodes.get(&id).cloned().unwrap_or_default()
    }
}

fn item_of(
    context: &Context,
    master: BTreeSet<SurfaceId>,
    slave: BTreeSet<SurfaceId>,
) -> MasterSlaveItem {
    MasterSlaveItem {
        master_name: context.name(&master, &[]),
        slave_name: context.name(&slave, &[]),
        master,
        slave,
    }
}

/// A node of the contact graph: surfaces that act together as one side.
#[derive(Clone, Debug)]
struct GraphNode {
    id: usize,
    items: BTreeSet<SurfaceId>,
    priority: Priority,
}

impl GraphNode {
    /// The better master first, see [`Priority::master_first`]; ties by id.
    fn master_first(&self, other: &Self) -> std::cmp::Ordering {
        (self.priority.master_first(&other.priority)).then(self.id.cmp(&other.id))
    }
}

/// An undirected graph: nodes in order, and per node its neighbours
/// in the order the edges were added, an edge added twice listed twice.
#[derive(Clone, Debug, Default)]
struct Graph {
    order: Vec<usize>,
    neighbours: HashMap<usize, Vec<usize>>,
}

impl Graph {
    fn add_node(&mut self, node: usize) {
        self.order.push(node);
        self.neighbours.entry(node).or_default();
    }

    fn add_edge(&mut self, a: usize, b: usize) {
        self.neighbours.entry(a).or_default().push(b);
        self.neighbours.entry(b).or_default().push(a);
    }

    fn neighbours(&self, node: usize) -> &[usize] {
        self.neighbours.get(&node).map_or(&[], Vec::as_slice)
    }

    fn subgraph(&self, nodes: &[usize]) -> Graph {
        Graph {
            order: nodes.to_vec(),
            neighbours: (nodes.iter())
                .map(|&n| (n, self.neighbours(n).to_vec()))
                .collect(),
        }
    }

    /// Connected parts, starting from the best masters.
    fn connected(&self, nodes: &HashMap<usize, GraphNode>) -> Vec<Graph> {
        let mut order = self.order.clone();
        order.sort_by(|a, b| nodes[a].master_first(&nodes[b]));
        let mut visited = HashSet::new();
        let mut graphs = Vec::new();
        for &start in &order {
            if visited.contains(&start) {
                continue;
            }
            let mut nodes = Vec::new();
            let mut queue = VecDeque::from([start]);
            while let Some(node) = queue.pop_front() {
                if visited.insert(node) {
                    nodes.push(node);
                    queue.extend(self.neighbours(node));
                }
            }
            graphs.push(self.subgraph(&nodes));
        }
        graphs
    }
}

/// Sides of contact pairs as nodes, the pairs as edges.
struct ContactGraph<'a> {
    context: &'a Context<'a>,
    nodes: HashMap<usize, GraphNode>,
    graph: Graph,
}

impl<'a> ContactGraph<'a> {
    fn new(context: &'a Context<'a>, items: &[MasterSlaveItem], merge_shared_nodes: bool) -> Self {
        let groups = merge_shared_nodes.then(|| node_groups(context, items));
        let overlap = |a: &BTreeSet<SurfaceId>, b: &BTreeSet<SurfaceId>| {
            if !a.is_disjoint(b) {
                return true;
            }
            let Some(groups) = &groups else {
                return false;
            };
            let first: HashSet<SurfaceId> = a.iter().map(|id| groups[id]).collect();
            b.iter().any(|id| first.contains(&groups[id]))
        };
        // Sides that share surfaces become one node.
        let mut sets: Vec<BTreeSet<SurfaceId>> = Vec::new();
        for item in items {
            for side in [&item.master, &item.slave] {
                let mut merged = side.clone();
                sets.retain(|set| {
                    if overlap(set, &merged) {
                        merged.extend(set.iter().copied());
                        false
                    } else {
                        true
                    }
                });
                sets.push(merged);
            }
        }
        let mut nodes = HashMap::new();
        let mut graph = Graph::default();
        for items in sets {
            let id = graph.order.len() + 1;
            let priority = context.priority(&items);
            nodes.insert(
                id,
                GraphNode {
                    id,
                    items,
                    priority,
                },
            );
            graph.add_node(id);
        }
        for item in items {
            let find = |side: &BTreeSet<SurfaceId>| {
                (graph.order.iter())
                    .copied()
                    .find(|n| !nodes[n].items.is_disjoint(side))
            };
            if let (Some(master), Some(slave)) = (find(&item.master), find(&item.slave))
                && master != slave
            {
                graph.add_edge(master, slave);
            }
        }
        Self {
            context,
            nodes,
            graph,
        }
    }

    /// Each side is the slave of its neighbours that are better masters, all of them in
    /// one pair, so that it is a slave only once; the best master of each connected part is
    /// no slave.
    fn master_slave_items(&self) -> Vec<MasterSlaveItem> {
        let mut items = Vec::new();
        let mut taken = Vec::new();
        for graph in self.graph.connected(&self.nodes) {
            let ordered = self.ordered(&graph);
            let rank: HashMap<usize, usize> =
                ordered.iter().enumerate().map(|(i, &n)| (n, i)).collect();
            for &node in &ordered {
                let masters: BTreeSet<usize> = (graph.neighbours(node).iter())
                    .copied()
                    .filter(|n| rank[n] < rank[&node])
                    .collect();
                if !masters.is_empty() {
                    let masters: Vec<usize> = masters.into_iter().collect();
                    items.push(self.grouped_item(&masters, &[node], &mut taken));
                }
            }
        }
        items
    }

    /// The nodes of a graph, the best master first.
    fn ordered(&self, graph: &Graph) -> Vec<usize> {
        let mut ordered = graph.order.clone();
        ordered.sort_by(|a, b| self.nodes[a].master_first(&self.nodes[b]));
        ordered
    }

    /// Groups the pairs by master and slave: a two-colourable part becomes one pair with
    /// the better master side as master; otherwise each node is the slave of its better
    /// neighbours, slaves with the same masters in one pair.
    fn grouped_master_slave_items(&self) -> Vec<MasterSlaveItem> {
        let mut items = Vec::new();
        let mut taken = Vec::new();
        for graph in self.graph.connected(&self.nodes) {
            if let Some((first, second)) = self.bipartite(&graph) {
                if first.is_empty() || second.is_empty() {
                    continue;
                }
                let priority = |nodes: &[usize]| {
                    let ids = (nodes.iter())
                        .flat_map(|n| self.nodes[n].items.iter().copied())
                        .collect();
                    self.context.priority(&ids)
                };
                if priority(&first).master_first(&priority(&second)).is_le() {
                    items.push(self.grouped_item(&first, &second, &mut taken));
                } else {
                    items.push(self.grouped_item(&second, &first, &mut taken));
                }
            } else {
                self.by_node_size(&graph, &mut items, &mut taken);
            }
        }
        items
    }

    fn bipartite(&self, graph: &Graph) -> Option<(Vec<usize>, Vec<usize>)> {
        let mut color: HashMap<usize, bool> = HashMap::new();
        let (mut first, mut second) = (Vec::new(), Vec::new());
        for &start in &graph.order {
            if color.contains_key(&start) {
                continue;
            }
            color.insert(start, false);
            let mut queue = VecDeque::from([start]);
            while let Some(node) = queue.pop_front() {
                let current = color[&node];
                if current {
                    second.push(node);
                } else {
                    first.push(node);
                }
                for &neighbour in graph.neighbours(node) {
                    match color.get(&neighbour) {
                        Some(&c) if c == current => return None,
                        Some(_) => {}
                        None => {
                            color.insert(neighbour, !current);
                            queue.push_back(neighbour);
                        }
                    }
                }
            }
        }
        Some((first, second))
    }

    fn by_node_size(
        &self,
        graph: &Graph,
        items: &mut Vec<MasterSlaveItem>,
        taken: &mut Vec<String>,
    ) {
        let ordered = self.ordered(graph);
        let rank: HashMap<usize, usize> =
            ordered.iter().enumerate().map(|(i, &n)| (n, i)).collect();
        let mut groups: Vec<(BTreeSet<usize>, Vec<usize>)> = Vec::new();
        for &node in &ordered {
            let masters: BTreeSet<usize> = (graph.neighbours(node).iter())
                .copied()
                .filter(|n| rank[n] < rank[&node])
                .collect();
            if masters.is_empty() {
                continue;
            }
            match groups.iter_mut().find(|(m, _)| *m == masters) {
                Some((_, slaves)) => slaves.push(node),
                None => groups.push((masters, vec![node])),
            }
        }
        for (masters, slaves) in groups {
            let masters: Vec<usize> = masters.into_iter().collect();
            items.push(self.grouped_item(&masters, &slaves, taken));
        }
    }

    fn grouped_item(
        &self,
        masters: &[usize],
        slaves: &[usize],
        taken: &mut Vec<String>,
    ) -> MasterSlaveItem {
        let collect = |nodes: &[usize]| -> BTreeSet<SurfaceId> {
            (nodes.iter())
                .flat_map(|n| self.nodes[n].items.iter().copied())
                .collect()
        };
        let (master, slave) = (collect(masters), collect(slaves));
        let master_name = self.context.name(&master, taken);
        taken.push(master_name.clone());
        let slave_name = self.context.name(&slave, taken);
        taken.push(slave_name.clone());
        MasterSlaveItem {
            master_name,
            slave_name,
            master,
            slave,
        }
    }
}

/// Groups of surfaces connected by shared mesh nodes, as union-find roots.
fn node_groups(context: &Context, items: &[MasterSlaveItem]) -> HashMap<SurfaceId, SurfaceId> {
    let all: BTreeSet<SurfaceId> = (items.iter())
        .flat_map(|i| i.master.iter().chain(&i.slave).copied())
        .collect();
    let mut parent: HashMap<SurfaceId, SurfaceId> = all.iter().map(|&id| (id, id)).collect();
    fn root(parent: &HashMap<SurfaceId, SurfaceId>, mut id: SurfaceId) -> SurfaceId {
        while parent[&id] != id {
            id = parent[&id];
        }
        id
    }
    let mut owner: HashMap<usize, SurfaceId> = HashMap::new();
    for &id in &all {
        for node in context.nodes(id) {
            match owner.get(&node) {
                Some(&other) => {
                    let (a, b) = (root(&parent, other), root(&parent, id));
                    if a != b {
                        let (low, high) = if a < b { (a, b) } else { (b, a) };
                        parent.insert(high, low);
                    }
                }
                None => {
                    owner.insert(node, id);
                }
            }
        }
    }
    all.iter().map(|&id| (id, root(&parent, id))).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Element, ElementShape, Part, extract_part_skin};

    /// Unit hex blocks: part i is the cube at `origins[i]`, with own nodes.
    fn blocks(origins: &[[f64; 3]]) -> FeMesh {
        let mut mesh = FeMesh::default();
        for (b, origin) in origins.iter().enumerate() {
            let base = 8 * b as u32;
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
            for (k, c) in corners.iter().enumerate() {
                mesh.set_node(base + k as u32 + 1, add(*origin, *c));
            }
            mesh.add_element(Element {
                id: b as u32 + 1,
                type_name: "C3D8".into(),
                shape: ElementShape::Hex8,
                nodes: (base + 1..=base + 8).collect(),
            })
            .unwrap();
            mesh.parts.push(Part {
                name: format!("PART-{}", b + 1),
                elements: vec![b as u32 + 1],
            });
        }
        mesh
    }

    fn search(mesh: &FeMesh, group_by: GroupBy) -> Vec<MasterSlaveItem> {
        search_with(mesh, group_by, Vec::new())
    }

    fn search_with(
        mesh: &FeMesh,
        group_by: GroupBy,
        stiffness: Vec<Option<f64>>,
    ) -> Vec<MasterSlaveItem> {
        let skins: Vec<PartSkin> = (mesh.parts.iter())
            .map(|p| extract_part_skin(mesh, p, 30.0))
            .collect();
        let parameters = SearchParameters {
            distance: 0.01,
            angle_deg: 35.0,
            group_by,
            stiffness,
            filter: GeometryFilter::default(),
            one_sided: Vec::new(),
            solid_skins: Vec::new(),
        };
        find_contact_pairs(mesh, &skins, &vec![true; mesh.parts.len()], &parameters)
    }

    /// A 2D plane strain model: a beam of 4 × 1 CPE4 quads from (0, 0) to (4, 1) and a disc
    /// of CPE3 triangles below it touching its bottom edge at (1, 0), with clockwise elements
    /// in the disc.
    fn beam_on_disc() -> FeMesh {
        let mut mesh = FeMesh::default();
        for i in 0..5 {
            mesh.set_node(i + 1, [i as f64, 0.0, 0.0]);
            mesh.set_node(i + 6, [i as f64, 1.0, 0.0]);
        }
        for i in 0..4 {
            mesh.add_element(Element {
                id: i + 1,
                type_name: "CPE4".into(),
                shape: ElementShape::Quad4,
                nodes: vec![i + 1, i + 2, i + 7, i + 6],
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "BEAM".into(),
            elements: (1..=4).collect(),
        });
        let (center, radius, n) = ([1.0, -0.5], 0.5, 24);
        mesh.set_node(100, [center[0], center[1], 0.0]);
        for k in 0..n {
            // The first rim node is the top of the disc, where it touches the beam.
            let phi = std::f64::consts::FRAC_PI_2 + std::f64::consts::TAU * k as f64 / n as f64;
            let point = [
                center[0] + radius * phi.cos(),
                center[1] + radius * phi.sin(),
                0.0,
            ];
            mesh.set_node(101 + k, point);
        }
        for k in 0..n {
            mesh.add_element(Element {
                id: 101 + k,
                type_name: "CPE3".into(),
                shape: ElementShape::Tri3,
                nodes: vec![100, 101 + (k + 1) % n, 101 + k],
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "DISC".into(),
            elements: (101..101 + n).collect(),
        });
        mesh
    }

    #[test]
    fn finds_the_edges_where_a_disc_touches_a_beam_in_2d() {
        let mesh = beam_on_disc();
        let items = search(&mesh, GroupBy::Parts);
        assert_eq!(items.len(), 1, "{items:?}");
        let item = &items[0];
        assert_eq!(item.name(), "BEAM_to_DISC");
        let skins: Vec<PartSkin> = (mesh.parts.iter())
            .map(|p| extract_part_skin(&mesh, p, 30.0))
            .collect();
        // The bottom edges of the beam and the rim of the disc, edges S1 of both.
        let master = surface_faces(&mesh, &skins, &item.master);
        assert_eq!(master, [(1, 1), (2, 1), (3, 1), (4, 1)]);
        let slave = surface_faces(&mesh, &skins, &item.slave);
        assert_eq!(slave.len(), 24);
        assert!(
            slave
                .iter()
                .all(|&(element, face)| element > 100 && face == 2)
        );
    }

    #[test]
    fn a_disc_below_a_gap_is_no_contact_in_2d() {
        let mut mesh = beam_on_disc();
        for k in 0..25 {
            let id = 100 + k;
            let p = mesh.coords()[mesh.node_index(id).unwrap()];
            mesh.set_node(id, [p[0], p[1] - 0.1, 0.0]);
        }
        assert!(search(&mesh, GroupBy::Parts).is_empty());
    }

    #[test]
    fn finds_the_faces_where_two_blocks_touch() {
        // The second block sits on the first with a small gap, shifted sideways by half.
        let mesh = blocks(&[[0.0, 0.0, 0.0], [0.5, 0.0, 1.005]]);
        let items = search(&mesh, GroupBy::Parts);
        assert_eq!(items.len(), 1);
        let item = &items[0];
        assert_eq!(item.name(), "PART-1_to_PART-2");
        let skins: Vec<PartSkin> = (mesh.parts.iter())
            .map(|p| extract_part_skin(&mesh, p, 30.0))
            .collect();
        // The top face S2 of the first block and the bottom face S1 of the second.
        let mut faces = surface_faces(&mesh, &skins, &item.master);
        faces.extend(surface_faces(&mesh, &skins, &item.slave));
        faces.sort_unstable();
        assert_eq!(faces, [(1, 2), (2, 1)]);
    }

    #[test]
    fn blocks_apart_or_side_by_side_at_a_corner_are_no_contact() {
        assert!(search(&blocks(&[[0.0; 3], [0.0, 0.0, 1.5]]), GroupBy::Parts).is_empty());
        // Touching along an edge only.
        let edge = blocks(&[[0.0; 3], [1.0, 0.0, 1.0]]);
        assert!(search(&edge, GroupBy::Parts).is_empty());
    }

    #[test]
    fn a_chain_of_blocks_makes_every_block_a_slave_once() {
        let mesh = blocks(&[[0.0; 3], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]]);
        for group_by in [GroupBy::None, GroupBy::Parts, GroupBy::Graph] {
            let items = search(&mesh, group_by);
            assert_eq!(items.len(), 2, "{group_by:?}");
            let slaves: Vec<_> = items.iter().map(|i| i.slave.clone()).collect();
            assert_ne!(slaves[0], slaves[1], "{group_by:?}");
        }
    }

    /// A unit cube at `origin` meshed with `n` × `n` × `n` hexes as part `name`.
    fn add_cube(mesh: &mut FeMesh, name: &str, origin: [f64; 3], n: usize) {
        add_box(mesh, name, origin, [1.0; 3], [n; 3]);
    }

    /// A box at `origin` of the size meshed with `n` hexes along each axis as part `name`.
    fn add_box(mesh: &mut FeMesh, name: &str, origin: [f64; 3], size: [f64; 3], n: [usize; 3]) {
        let first_node = mesh.coords().len() as u32 + 1;
        let first_element = mesh.elements().len() as u32 + 1;
        let id = |i: usize, j: usize, k: usize| {
            first_node + (i + (n[0] + 1) * (j + (n[1] + 1) * k)) as u32
        };
        let h = [0, 1, 2].map(|a| size[a] / n[a] as f64);
        for k in 0..=n[2] {
            for j in 0..=n[1] {
                for i in 0..=n[0] {
                    let offset = [i as f64 * h[0], j as f64 * h[1], k as f64 * h[2]];
                    mesh.set_node(id(i, j, k), add(origin, offset));
                }
            }
        }
        let mut elements = Vec::new();
        for k in 0..n[2] {
            for j in 0..n[1] {
                for i in 0..n[0] {
                    let element = first_element + elements.len() as u32;
                    let nodes = vec![
                        id(i, j, k),
                        id(i + 1, j, k),
                        id(i + 1, j + 1, k),
                        id(i, j + 1, k),
                        id(i, j, k + 1),
                        id(i + 1, j, k + 1),
                        id(i + 1, j + 1, k + 1),
                        id(i, j + 1, k + 1),
                    ];
                    mesh.add_element(Element {
                        id: element,
                        type_name: "C3D8".into(),
                        shape: ElementShape::Hex8,
                        nodes,
                    })
                    .unwrap();
                    elements.push(element);
                }
            }
        }
        mesh.parts.push(Part {
            name: name.into(),
            elements,
        });
    }

    /// A plate of `n` × `n` S4 shells of the size in the plane through `origin` spanned by
    /// the axes `u` and `v`, with its own nodes, as part `name`.
    fn add_plate(mesh: &mut FeMesh, name: &str, origin: [f64; 3], [u, v]: [[f64; 3]; 2], n: usize) {
        let first_node = mesh.coords().len() as u32 + 1;
        let first_element = mesh.elements().len() as u32 + 1;
        let id = |i: usize, j: usize| first_node + (i + (n + 1) * j) as u32;
        for j in 0..=n {
            for i in 0..=n {
                let (a, b) = (i as f64 / n as f64, j as f64 / n as f64);
                mesh.set_node(id(i, j), add(origin, add(scale(u, a), scale(v, b))));
            }
        }
        let mut elements = Vec::new();
        for j in 0..n {
            for i in 0..n {
                let element = first_element + elements.len() as u32;
                mesh.add_element(Element {
                    id: element,
                    type_name: "S4".into(),
                    shape: ElementShape::Quad4,
                    nodes: vec![id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)],
                })
                .unwrap();
                elements.push(element);
            }
        }
        mesh.parts.push(Part {
            name: name.into(),
            elements,
        });
    }

    fn faces_of(mesh: &FeMesh, ids: &BTreeSet<SurfaceId>) -> Vec<(ElementId, u8)> {
        let skins: Vec<PartSkin> = (mesh.parts.iter())
            .map(|p| extract_part_skin(mesh, p, 30.0))
            .collect();
        surface_faces(mesh, &skins, ids)
    }

    #[test]
    fn the_edge_of_a_shell_standing_on_a_block_is_the_slave_of_its_face() {
        // A block of 2 x 2 x 1 hexes from z = 1 to 2, and a vertical plate of 3 x 3 shells
        // in x = 0.5 below it, its top edge on the bottom face of the block.
        let mut mesh = FeMesh::default();
        add_box(&mut mesh, "BLOCK", [0.0, 0.0, 1.0], [1.0; 3], [2, 2, 1]);
        add_plate(
            &mut mesh,
            "PLATE",
            [0.5, 0.0, 0.0],
            [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            3,
        );
        for group_by in [GroupBy::None, GroupBy::Parts, GroupBy::Graph] {
            let items = search(&mesh, group_by);
            assert_eq!(items.len(), 1, "{group_by:?}: {items:?}");
            let item = &items[0];
            assert_eq!(item.name(), "BLOCK_to_PLATE");
            assert_eq!(item.geometry(), "Solid-Shell edge");
            // The bottom faces S1 of the block, and the top edges of the top row of shells,
            // from node 3 to 4: face S5.
            assert_eq!(
                faces_of(&mesh, &item.master),
                [(1, 1), (2, 1), (3, 1), (4, 1)]
            );
            assert_eq!(faces_of(&mesh, &item.slave), [(11, 5), (12, 5), (13, 5)]);
        }
        let only_solids = GeometryFilter {
            shell_edge: false,
            ..GeometryFilter::default()
        };
        let skins: Vec<PartSkin> = (mesh.parts.iter())
            .map(|p| extract_part_skin(&mesh, p, 30.0))
            .collect();
        let parameters = SearchParameters {
            distance: 0.01,
            angle_deg: 35.0,
            group_by: GroupBy::Parts,
            stiffness: Vec::new(),
            filter: only_solids,
            one_sided: Vec::new(),
            solid_skins: Vec::new(),
        };
        assert!(find_contact_pairs(&mesh, &skins, &[true, true], &parameters).is_empty());
    }

    #[test]
    fn a_shell_on_a_block_touches_it_with_the_side_facing_it() {
        // A coarse plate on top of a block faces it with its bottom, a plate below the block
        // with its top; being coarser, the plate is the master.
        for (z, face, kind) in [
            (1.0, 1, SurfaceKind::ShellBottom),
            (0.0, 2, SurfaceKind::ShellTop),
        ] {
            let mut mesh = FeMesh::default();
            add_cube(&mut mesh, "BLOCK", [0.0; 3], 4);
            add_plate(
                &mut mesh,
                "PLATE",
                [0.0, 0.0, z],
                [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                1,
            );
            let items = search(&mesh, GroupBy::Parts);
            assert_eq!(items.len(), 1, "{z}: {items:?}");
            let item = &items[0];
            assert_eq!(item.master.first().unwrap().1, kind);
            assert_eq!(item.geometry(), "Shell-Solid");
            assert_eq!(faces_of(&mesh, &item.master), [(65, face)]);
        }
    }

    #[test]
    fn the_coarser_mesh_is_the_master() {
        // The finely meshed cube comes first, so the part order alone would make it master.
        let mut mesh = FeMesh::default();
        add_cube(&mut mesh, "FINE", [0.0, 0.0, 1.0], 3);
        add_cube(&mut mesh, "COARSE", [0.0; 3], 1);
        for group_by in [GroupBy::None, GroupBy::Parts, GroupBy::Graph] {
            let items = search_with(&mesh, group_by, vec![Some(210e3), Some(70e3)]);
            assert_eq!(items.len(), 1, "{group_by:?}");
            assert_eq!(items[0].name(), "COARSE_to_FINE", "{group_by:?}");
        }
    }

    #[test]
    fn with_the_same_mesh_the_stiffer_material_is_the_master() {
        let mesh = blocks(&[[0.0; 3], [0.0, 0.0, 1.0]]);
        let items = search_with(&mesh, GroupBy::Parts, vec![Some(70e3), Some(210e3)]);
        assert_eq!(items[0].name(), "PART-2_to_PART-1");
        let items = search_with(&mesh, GroupBy::Parts, vec![Some(210e3), Some(70e3)]);
        assert_eq!(items[0].name(), "PART-1_to_PART-2");
    }

    #[test]
    fn a_surface_on_two_masters_is_the_slave_of_both_at_once() {
        // A finely meshed plate lying on two coarse blocks: its bottom face touches both.
        let mut mesh = FeMesh::default();
        add_cube(&mut mesh, "LEFT", [0.0; 3], 1);
        add_cube(&mut mesh, "RIGHT", [2.0, 0.0, 0.0], 1);
        add_box(
            &mut mesh,
            "PLATE",
            [0.0, 0.0, 1.0],
            [3.0, 1.0, 0.25],
            [12, 4, 1],
        );
        for group_by in [GroupBy::None, GroupBy::Parts, GroupBy::Graph] {
            let items = search(&mesh, group_by);
            assert_eq!(items.len(), 1, "{group_by:?}: {items:?}");
            assert_eq!(items[0].slave_name, "PLATE", "{group_by:?}");
            let masters: BTreeSet<usize> = items[0].master.iter().map(|id| id.0).collect();
            assert_eq!(masters, [0, 1].into(), "{group_by:?}");
        }
    }

    #[test]
    fn long_names_are_shortened() {
        let item = MasterSlaveItem {
            master_name: "A_VERY_LONG_PART_NAME_INDEED_REALLY".into(),
            slave_name: "B".into(),
            master: BTreeSet::new(),
            slave: BTreeSet::new(),
        };
        assert_eq!(item.name(), "Item_to_B");
    }
}
