//! Contact detection, Ansys' connection groups: a search for touching surfaces with its
//! parameters kept in the model, so that it can run again after the geometry or the mesh
//! changed. Each pair of touching surfaces found becomes a [`ContactRegion`] of the
//! detection's type, and ends of beams and trusses of different parts that meet become a
//! [`NodeTie`]. Detecting again replaces the regions and node ties found before, except
//! those the user changed since.
//!
//! A detection runs on the display of the CAD geometry when the model has one, Ansys'
//! detection on the geometry: the regions take the whole CAD faces that touch, so they exist
//! before the parts are meshed and stay the same when they are meshed again.

use std::collections::{BTreeSet, HashSet};

use std::collections::BTreeMap;

use plx_mesh::{
    CadEntity, FeMesh, GeometryFilter, GroupBy, LineJoint, PartSkin, SearchParameters, SurfaceId,
    SurfaceKind, display_cad_map, find_contact_pairs, find_line_joints, surface_faces,
};
use serde::{Deserialize, Serialize};

use crate::connection::parts_of;
use crate::{ContactBehavior, ContactRegion, FeModel, NodeTie, Region};

/// The parts of the two sides of a region, the smaller set first.
type PartPair = (BTreeSet<usize>, BTreeSet<usize>);

/// A search for contact regions and the parameters it runs with.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ContactDetection {
    /// The internal identity other items refer to; the user sees only the name.
    pub id: crate::Id,
    pub name: String,
    /// Names of the parts searched; empty searches all visible parts.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub parts: Vec<String>,
    /// Largest gap between surfaces that touch; `None` derives it from the model size.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Largest angle between the surfaces facing each other, in degrees.
    pub angle_deg: f64,
    pub filter: DetectionFilter,
    pub grouping: DetectionGrouping,
    /// The type of the regions found.
    pub behavior: ContactBehavior,
}

impl ContactDetection {
    /// The diagonal of the box around the parts the detection searches, all parts if it
    /// names none. An automatic tolerance is 0.5 % of it.
    pub fn searched_diagonal(&self, mesh: &FeMesh) -> f64 {
        let (mut min, mut max) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        let searched =
            (mesh.parts.iter()).filter(|p| self.parts.is_empty() || self.parts.contains(&p.name));
        for element in searched.flat_map(|p| &p.elements) {
            let Some(element) = mesh.element(*element) else {
                continue;
            };
            for p in element.nodes.iter().filter_map(|&n| mesh.node(n)) {
                for d in 0..3 {
                    min[d] = min[d].min(p[d]);
                    max[d] = max[d].max(p[d]);
                }
            }
        }
        if min[0] > max[0] {
            return 0.0;
        }
        (0..3)
            .map(|d| (max[d] - min[d]).powi(2))
            .sum::<f64>()
            .sqrt()
    }

    pub fn new(name: impl Into<String>) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            parts: Vec::new(),
            tolerance: None,
            angle_deg: 35.0,
            filter: DetectionFilter::default(),
            grouping: DetectionGrouping::default(),
            behavior: ContactBehavior::Bonded,
        }
    }
}

/// Which surfaces a detection searches.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DetectionFilter {
    /// Outer faces of solids.
    pub solid: bool,
    /// Top and bottom of shells and membranes.
    pub shell: bool,
    /// Free edges of shells and membranes.
    pub shell_edge: bool,
    /// Ends of beams and trusses of different parts at one point, tied as node ties.
    #[serde(default = "crate::active")]
    pub line_end: bool,
}

impl Default for DetectionFilter {
    fn default() -> Self {
        Self {
            solid: true,
            shell: true,
            shell_edge: true,
            line_end: true,
        }
    }
}

/// How touching surfaces are gathered into regions.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum DetectionGrouping {
    /// One region per pair of touching parts.
    #[default]
    Parts,
    /// One region per pair of touching surfaces (CAD faces).
    Faces,
    /// Surfaces sharing nodes are merged, so that every node is on the slave side once.
    Graph,
}

impl DetectionGrouping {
    pub const ALL: [Self; 3] = [Self::Parts, Self::Faces, Self::Graph];

    pub fn name(self) -> &'static str {
        match self {
            Self::Parts => "Parts",
            Self::Faces => "Faces",
            Self::Graph => "Graph",
        }
    }
}

/// What a detection did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DetectionReport {
    /// The tolerance it searched with.
    pub tolerance: f64,
    /// Regions found and added.
    pub added: usize,
    /// Regions of the previous detection that were replaced.
    pub replaced: usize,
    /// Regions the user changed, kept although found again.
    pub kept: Vec<String>,
    /// Regions the user changed that the detection no longer finds.
    pub lost: Vec<String>,
    /// Node ties found at line ends, those the user changed and kept included.
    pub node_ties: usize,
}

impl DetectionReport {
    /// One line for the output window.
    pub fn summary(&self, name: &str) -> String {
        let mut text = format!(
            "{name}: {} contact region{} found",
            self.added + self.kept.len(),
            if self.added + self.kept.len() == 1 {
                ""
            } else {
                "s"
            }
        );
        if self.node_ties > 0 {
            text += &format!(
                ", {} node tie{} at line ends",
                self.node_ties,
                if self.node_ties == 1 { "" } else { "s" }
            );
        }
        if !self.kept.is_empty() {
            text += &format!(", {} changed by you kept", self.kept.len());
        }
        if !self.lost.is_empty() {
            text += &format!(
                "; no longer found but kept because you changed them: {}",
                self.lost.join(", ")
            );
        }
        text
    }
}

/// Default tolerance of a detection: 0.5 % of the diagonal of the parts searched, but at
/// least half their mean element size, so that the faces of different meshes on a curved
/// surface still count as touching.
pub fn auto_detection_tolerance(mesh: &FeMesh, skins: &[PartSkin], searched: &[bool]) -> f64 {
    tolerance_of(mesh, skins, searched, true)
}

/// Default tolerance of a detection on the display of the CAD geometry: 0.5 % of the
/// diagonal of the parts searched. Its triangles say nothing about the mesh to come, and
/// the long thin ones along curved faces would make the tolerance far too large.
pub fn auto_geometry_detection_tolerance(
    display: &FeMesh,
    skins: &[PartSkin],
    searched: &[bool],
) -> f64 {
    tolerance_of(display, skins, searched, false)
}

fn tolerance_of(mesh: &FeMesh, skins: &[PartSkin], searched: &[bool], by_elements: bool) -> f64 {
    let coords = mesh.coords();
    let (mut min, mut max) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    let (mut length, mut edges) = (0.0, 0usize);
    let skins = skins.iter().zip(searched).filter(|(_, s)| **s);
    for face in skins.flat_map(|(skin, _)| &skin.faces) {
        for (k, &corner) in face.corners.iter().enumerate() {
            let p = coords[corner];
            for d in 0..3 {
                min[d] = min[d].min(p[d]);
                max[d] = max[d].max(p[d]);
            }
            let q = coords[face.corners[(k + 1) % face.corners.len()]];
            length += (0..3).map(|d| (q[d] - p[d]).powi(2)).sum::<f64>().sqrt();
            edges += 1;
        }
    }
    if edges == 0 {
        // Only beams and trusses: 0.5 % of the diagonal of their nodes, for the line ends.
        let searched = (mesh.parts.iter().zip(searched)).filter(|(_, s)| **s);
        for element in searched.flat_map(|(p, _)| &p.elements) {
            let Some(element) = mesh.element(*element) else {
                continue;
            };
            for p in element.nodes.iter().filter_map(|&n| mesh.node(n)) {
                for d in 0..3 {
                    min[d] = min[d].min(p[d]);
                    max[d] = max[d].max(p[d]);
                }
            }
        }
        if min[0] > max[0] {
            return 0.0;
        }
    }
    let diagonal = (0..3)
        .map(|d| (max[d] - min[d]).powi(2))
        .sum::<f64>()
        .sqrt();
    if by_elements && edges > 0 {
        (0.005 * diagonal).max(0.5 * length / edges as f64)
    } else {
        0.005 * diagonal
    }
}

/// The display of CAD geometry a detection runs on, see [`FeModel::detect_contacts_on_geometry`].
pub struct DetectionGeometry<'a> {
    /// The display mesh: triangles on the CAD faces, lines along the CAD edges.
    pub mesh: &'a FeMesh,
    pub skins: &'a [PartSkin],
    /// The CAD face or edge each element shows; element ids count from 1.
    pub entities: &'a [CadEntity],
    /// The CAD edges bounding each CAD face.
    pub face_edges: &'a BTreeMap<i32, Vec<i32>>,
    /// Of each part, the dimension of its CAD shape: 3 a solid, 2 faces, 1 edges.
    pub part_dimensions: &'a [u8],
}

/// What a detection on the display of CAD geometry knows of its parts.
#[derive(Clone, Copy)]
struct Display<'a> {
    /// Of each part, the dimension of its CAD shape.
    part_dimensions: &'a [u8],
    /// Of each part, whether its faces were turned outwards, see
    /// [`plx_mesh::SearchParameters::one_sided`].
    one_sided: &'a [bool],
}

impl FeModel {
    /// Runs the detection `index`: finds the touching surfaces of the parts it searches
    /// (`visible` says which parts are shown, for a detection of all parts) and replaces its
    /// regions with the ones found. Regions the user changed since stay as they are.
    pub fn detect_contacts(
        &mut self,
        index: usize,
        mesh: &FeMesh,
        skins: &[PartSkin],
        visible: &[bool],
    ) -> Option<DetectionReport> {
        self.detect(index, mesh, skins, visible, None)
    }

    /// [`Self::detect_contacts`] on the display of the CAD geometry: the sides of the
    /// regions are the CAD faces, or the CAD edges of shells, that touch anywhere, so that
    /// the regions are found before the parts are meshed and keep to the geometry when they
    /// are meshed again. The tie tolerance is left to the mesh unless the detection sets one.
    pub fn detect_contacts_on_geometry(
        &mut self,
        index: usize,
        geometry: &DetectionGeometry,
        visible: &[bool],
    ) -> Option<DetectionReport> {
        let mut mesh = geometry.mesh.clone();
        mesh.cad = display_cad_map(&mesh, geometry.entities, geometry.face_edges);
        // The display does not tell which side of the faces of a solid is outside: turned
        // outwards, only that side touches. Else two faces of neighbouring solids in one
        // plane, e.g. on a plane of symmetry, would touch each other's back.
        let one_sided: Vec<bool> = (0..mesh.parts.len())
            .map(|part| {
                geometry.part_dimensions.get(part) == Some(&3)
                    && plx_mesh::orient_outward(&mut mesh, part)
            })
            .collect();
        let skins: Vec<PartSkin> = if one_sided.contains(&true) {
            (mesh.parts.iter())
                .zip(geometry.skins)
                .zip(&one_sided)
                .map(|((part, skin), &turned)| {
                    if turned {
                        plx_mesh::extract_part_skin(&mesh, part, 180.0)
                    } else {
                        skin.clone()
                    }
                })
                .collect()
        } else {
            geometry.skins.to_vec()
        };
        let display = Display {
            part_dimensions: geometry.part_dimensions,
            one_sided: &one_sided,
        };
        self.detect(index, &mesh, &skins, visible, Some(display))
    }

    /// The detection on a mesh, or on a display of CAD geometry with the dimensions of its
    /// parts.
    fn detect(
        &mut self,
        index: usize,
        mesh: &FeMesh,
        skins: &[PartSkin],
        visible: &[bool],
        display: Option<Display>,
    ) -> Option<DetectionReport> {
        let detection = self.contact_detections.get(index)?.clone();
        let searched = searched_parts(&detection, mesh, visible);
        let tolerance = (detection.tolerance)
            .unwrap_or_else(|| tolerance_of(mesh, skins, &searched, display.is_none()));
        let young_of = self.young_of_elements(mesh);
        let stiffness = (mesh.parts.iter())
            .map(|part| {
                (part.elements.iter())
                    .filter_map(|e| young_of.get(e).copied())
                    .reduce(f64::max)
            })
            .collect();
        let filter = if self.properties.space.is_2d() {
            GeometryFilter {
                solid: false,
                shell: false,
                shell_edge: true,
            }
        } else if display.is_some() {
            // The display shows the faces of solids as triangles, searched like shells.
            GeometryFilter {
                solid: false,
                shell: detection.filter.solid || detection.filter.shell,
                shell_edge: detection.filter.shell_edge,
            }
        } else {
            GeometryFilter {
                solid: detection.filter.solid,
                shell: detection.filter.shell,
                shell_edge: detection.filter.shell_edge,
            }
        };
        let parameters = SearchParameters {
            distance: tolerance,
            angle_deg: detection.angle_deg,
            group_by: match detection.grouping {
                DetectionGrouping::Parts => GroupBy::Parts,
                DetectionGrouping::Faces => GroupBy::None,
                DetectionGrouping::Graph => GroupBy::Graph,
            },
            stiffness,
            filter,
            one_sided: display.map_or(Vec::new(), |d| d.one_sided.to_vec()),
        };
        let mut items = find_contact_pairs(mesh, skins, &searched, &parameters);
        if let Some(dimensions) = display.map(|d| d.part_dimensions) {
            // Of solids only their faces count, which the filter asks for as solid faces;
            // the borders of their faces are no edges of shells.
            let wanted = |id: &SurfaceId| match (dimensions.get(id.0), id.1) {
                (Some(3), SurfaceKind::ShellEdge) => false,
                (Some(3), _) => detection.filter.solid,
                (_, SurfaceKind::ShellEdge) => true,
                _ => detection.filter.shell || self.properties.space.is_2d(),
            };
            for item in &mut items {
                item.master.retain(wanted);
                item.slave.retain(wanted);
            }
        }
        // Surfaces of whole CAD faces are kept by geometry, so that they survive remeshing;
        // on the geometry every face touched is taken whole.
        let region = |surfaces: &BTreeSet<SurfaceId>| {
            let faces = surface_faces(mesh, skins, surfaces);
            if display.is_some() {
                let entities = mesh.cad_entities_touched(&faces);
                return if entities.is_empty() {
                    Region::Faces(Vec::new())
                } else {
                    Region::Geometry(entities)
                };
            }
            match mesh.whole_cad_faces(&faces) {
                Some(entities) => Region::Geometry(entities),
                None => Region::Faces(faces),
            }
        };
        let mut report = DetectionReport {
            tolerance,
            ..DetectionReport::default()
        };
        let ours = |r: &ContactRegion| r.detected_by == Some(detection.id);
        let before = self.contact_regions.len();
        // A region found again keeps the id of the one it replaces, so that what refers to
        // it (a contact history output, a contact change of a step) stays on it.
        let replaced: Vec<(String, crate::Id)> = (self.contact_regions.iter())
            .filter(|r| ours(r) && !r.edited)
            .map(|r| (r.name.clone(), r.id))
            .collect();
        self.contact_regions.retain(|r| !ours(r) || r.edited);
        report.replaced = before - self.contact_regions.len();
        // The parts each kept region connects, to recognise it among the regions found.
        let key = |a: &Region, b: &Region| {
            let (a, b) = (parts_of(a, mesh), parts_of(b, mesh));
            if a <= b { (a, b) } else { (b, a) }
        };
        let edited: Vec<(String, PartPair)> = (self.contact_regions.iter().filter(|r| ours(r)))
            .map(|r| (r.name.clone(), key(&r.side_a, &r.side_b)))
            .collect();
        let mut found_again = HashSet::new();
        for item in &items {
            let mut found = ContactRegion::new("", detection.behavior);
            found.side_a = region(&item.master);
            found.side_b = region(&item.slave);
            let empty = |r: &Region| matches!(r, Region::Faces(f) if f.is_empty());
            if empty(&found.side_a) || empty(&found.side_b) {
                continue;
            }
            let parts = key(&found.side_a, &found.side_b);
            if let Some((name, _)) = edited.iter().find(|(_, k)| *k == parts) {
                found_again.insert(name.clone());
                continue;
            }
            // The side found as master is side A; its tolerance is the one it was found
            // with, so that a tie catches every node the detection saw touching. Found on
            // the geometry, the mesh to come sets it unless the detection does.
            found.settings.tolerance = match display {
                Some(_) => detection.tolerance,
                None => Some(tolerance),
            };
            found.detected_by = Some(detection.id);
            let name = found.name_from_definition(mesh);
            let taken = self.contact_pair_names();
            found.name = if taken.iter().any(|t| t.eq_ignore_ascii_case(&name)) {
                crate::next_name(&name, taken)
            } else {
                name
            };
            if let Some((_, id)) = replaced.iter().find(|(n, _)| *n == found.name) {
                found.id = *id;
            }
            self.contact_regions.push(found);
            report.added += 1;
        }
        for (name, _) in edited {
            if found_again.contains(&name) {
                report.kept.push(name);
            } else {
                report.lost.push(name);
            }
        }
        // The display of the geometry has no nodes of the mesh to tie; the caller ties the
        // line ends of the mesh, see [`Self::detect_line_ends`].
        if display.is_none() {
            self.detect_line_ends(index, mesh, visible, &mut report);
        }
        Some(report)
    }

    /// The node ties of the detection `index`: ends of beams and trusses of different parts
    /// within the report's tolerance of each other are tied, rigidly. They replace the node
    /// ties the detection found before, except those the user changed since, and are counted
    /// in `report`. With the detection's line ends switched off, its node ties are removed.
    pub fn detect_line_ends(
        &mut self,
        index: usize,
        mesh: &FeMesh,
        visible: &[bool],
        report: &mut DetectionReport,
    ) {
        let Some(detection) = self.contact_detections.get(index).cloned() else {
            return;
        };
        let ours = |t: &NodeTie| t.detected_by == Some(detection.id);
        self.node_ties.retain(|t| !ours(t) || t.edited);
        if !detection.filter.line_end {
            return;
        }
        let edited: Vec<BTreeSet<usize>> = (self.node_ties.iter().filter(|t| ours(t)))
            .map(|t| node_parts(&t.region, mesh))
            .collect();
        report.node_ties += edited.len();
        let searched = searched_parts(&detection, mesh, visible);
        for joint in find_line_joints(mesh, &searched, report.tolerance) {
            if edited.contains(&joint.parts) {
                continue;
            }
            let parts: Vec<&str> = (joint.parts.iter())
                .filter_map(|&p| mesh.parts.get(p))
                .map(|p| p.name.as_str())
                .collect();
            let name = format!("Node Tie - {}", parts.join(", "));
            let taken = self.node_ties.iter().map(|t| t.name.as_str());
            let name = if taken.clone().any(|t| t.eq_ignore_ascii_case(&name)) {
                crate::next_name(&name, taken)
            } else {
                name
            };
            self.node_ties.push(NodeTie {
                region: joint_region(mesh, &joint),
                detected_by: Some(detection.id),
                ..NodeTie::new(name)
            });
            report.node_ties += 1;
        }
    }
}

/// Which parts of `mesh` the detection searches: the parts it names, or the visible ones.
fn searched_parts(detection: &ContactDetection, mesh: &FeMesh, visible: &[bool]) -> Vec<bool> {
    (mesh.parts.iter().enumerate())
        .map(|(i, part)| {
            if detection.parts.is_empty() {
                visible.get(i).copied().unwrap_or(true)
            } else {
                detection.parts.contains(&part.name)
            }
        })
        .collect()
}

/// The parts with elements on the nodes of `region`, to recognise a node tie by.
fn node_parts(region: &Region, mesh: &FeMesh) -> BTreeSet<usize> {
    let nodes: BTreeSet<_> = region.nodes(mesh).into_iter().collect();
    (mesh.parts.iter().enumerate())
        .filter(|(_, p)| {
            (p.elements.iter().filter_map(|&e| mesh.element(e)))
                .any(|e| e.nodes.iter().any(|n| nodes.contains(n)))
        })
        .map(|(i, _)| i)
        .collect()
}

/// The nodes of a line joint as a region: the CAD vertices they lie on, if those have no
/// other nodes, so that the tie survives remeshing; else the nodes themselves.
fn joint_region(mesh: &FeMesh, joint: &LineJoint) -> Region {
    let vertices: BTreeSet<CadEntity> = (mesh.cad.nodes.iter())
        .filter(|(entity, nodes)| {
            matches!(entity, CadEntity::Vertex(_)) && nodes.iter().any(|n| joint.nodes.contains(n))
        })
        .map(|(&entity, _)| entity)
        .collect();
    let covered: BTreeSet<_> = (vertices.iter())
        .flat_map(|v| mesh.cad.nodes[v].iter().copied())
        .collect();
    if !vertices.is_empty() && covered == joint.nodes.iter().copied().collect() {
        Region::Geometry(vertices.into_iter().collect())
    } else {
        Region::Nodes(joint.nodes.clone())
    }
}
