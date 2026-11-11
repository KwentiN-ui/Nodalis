use std::collections::BTreeSet;

use plx_mesh::{CadEntity, ElementId, FeMesh, NodeId, SurfaceDefinition};
use serde::{Deserialize, Serialize};

/// Where a section, boundary condition or load applies.
///
/// Selections made in the GUI are stored as they were picked; the sets the solver needs are
/// derived from them when writing the input file.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Region {
    /// Whole parts by name.
    Parts(Vec<String>),
    /// Picked nodes.
    Nodes(Vec<NodeId>),
    /// Picked element faces as (element, CalculiX face number 1..6).
    Faces(Vec<(ElementId, u8)>),
    /// Picked element edges of 3D parts by their end nodes, e.g. the edge of a shell a
    /// surface traction acts on.
    Edges(Vec<[NodeId; 2]>),
    /// A node set of the input file.
    NodeSet(String),
    /// An element set of the input file.
    ElementSet(String),
    /// A surface of the input file.
    Surface(String),
    /// Faces, edges and vertices of the CAD geometry the mesh was generated from, picked
    /// on the geometry. They are found on the mesh again after remeshing.
    Geometry(Vec<CadEntity>),
    /// A reference point by its id, for boundary conditions and loads on a rigid body: the
    /// body's reference node, which exists only in the input file, so the region has no
    /// nodes of the mesh.
    ReferencePoint(crate::Id),
    /// The point of a remote displacement boundary condition or a remote force by the id of
    /// the boundary condition or load, for history outputs of its reaction forces and
    /// moments. Like a reference point it exists only in the input file.
    RemotePoint(crate::Id),
}

impl Region {
    /// Whether nothing is picked: no faces, nodes, edges, parts or CAD entities, or a set of
    /// no name.
    pub fn is_empty(&self) -> bool {
        match self {
            Region::Parts(v) => v.is_empty(),
            Region::Nodes(v) => v.is_empty(),
            Region::Faces(v) => v.is_empty(),
            Region::Edges(v) => v.is_empty(),
            Region::NodeSet(n) | Region::ElementSet(n) | Region::Surface(n) => n.is_empty(),
            Region::Geometry(v) => v.is_empty(),
            _ => false,
        }
    }

    /// Elements of the region, for sections. Node and face selections have none.
    pub fn elements(&self, mesh: &FeMesh) -> Vec<ElementId> {
        let mut ids: BTreeSet<ElementId> = BTreeSet::new();
        match self {
            Region::Parts(names) => {
                for part in mesh.parts.iter().filter(|p| names.contains(&p.name)) {
                    ids.extend(part.elements.iter().copied());
                }
            }
            Region::ElementSet(name) => {
                ids.extend(mesh.element_sets.get(name).into_iter().flatten().copied());
            }
            Region::Faces(faces) => ids.extend(faces.iter().map(|(e, _)| *e)),
            Region::Geometry(entities) => {
                ids.extend(mesh.cad_faces(entities).iter().map(|(e, _)| *e));
                ids.extend(mesh.cad_line_elements(entities));
            }
            Region::Nodes(_) | Region::NodeSet(_) | Region::Surface(_) => {}
            Region::Edges(_) | Region::ReferencePoint(_) | Region::RemotePoint(_) => {}
        }
        ids.into_iter().collect()
    }

    /// Nodes of the region in ascending order, for boundary conditions and point loads.
    pub fn nodes(&self, mesh: &FeMesh) -> Vec<NodeId> {
        let mut ids: BTreeSet<NodeId> = BTreeSet::new();
        let element_nodes = |ids: &mut BTreeSet<NodeId>, elements: &[ElementId]| {
            for element in elements.iter().filter_map(|&e| mesh.element(e)) {
                ids.extend(element.nodes.iter().copied());
            }
        };
        match self {
            Region::Parts(_) | Region::ElementSet(_) => {
                element_nodes(&mut ids, &self.elements(mesh));
            }
            Region::Nodes(nodes) => ids.extend(nodes.iter().copied()),
            Region::NodeSet(name) => {
                ids.extend(mesh.node_sets.get(name).into_iter().flatten().copied());
            }
            Region::Faces(faces) => ids.extend(face_nodes(mesh, faces)),
            Region::Edges(_) => ids.extend(self.edges(mesh).into_iter().flatten()),
            Region::Geometry(entities) => ids.extend(mesh.cad_nodes(entities)),
            Region::Surface(name) => match mesh.surfaces.get(name) {
                Some(SurfaceDefinition::Nodes(nodes)) => ids.extend(nodes.iter().copied()),
                Some(SurfaceDefinition::ElementFaces(faces)) => {
                    ids.extend(face_nodes(mesh, faces));
                }
                None => {}
            },
            Region::ReferencePoint(_) | Region::RemotePoint(_) => {}
        }
        ids.into_iter().collect()
    }

    /// Whether the region is made of line elements (beams, ropes) rather than faces: a
    /// rope touching a surface with its nodes.
    pub fn is_lines(&self, mesh: &FeMesh) -> bool {
        self.faces(mesh).is_empty()
            && (self.elements(mesh).iter())
                .filter_map(|&e| mesh.element(e))
                .any(|e| e.shape.family() == plx_mesh::ElementFamily::Line)
    }

    /// Element faces of the region, for pressure loads.
    pub fn faces(&self, mesh: &FeMesh) -> Vec<(ElementId, u8)> {
        match self {
            Region::Faces(faces) => faces.clone(),
            Region::Geometry(entities) => mesh.cad_faces(entities),
            Region::Surface(name) => match mesh.surfaces.get(name) {
                Some(SurfaceDefinition::ElementFaces(faces)) => faces.clone(),
                _ => Vec::new(),
            },
            _ => Vec::new(),
        }
    }

    /// Element edges of the region, for loads along edges: the picked edges or those along
    /// picked CAD edges, each as its end nodes and the midside node of quadratic elements.
    pub fn edges(&self, mesh: &FeMesh) -> Vec<Vec<NodeId>> {
        match self {
            Region::Edges(segments) => mesh.edge_lines(segments),
            Region::Geometry(entities) => {
                let segments: Vec<[NodeId; 2]> = (entities.iter())
                    .filter_map(|e| match e {
                        CadEntity::Edge(tag) => mesh.cad.segments.get(tag),
                        _ => None,
                    })
                    .flatten()
                    .copied()
                    .collect();
                mesh.edge_lines(&segments)
            }
            _ => Vec::new(),
        }
    }

    /// Short description for the GUI, e.g. "3 nodes", with the names of the items of
    /// `model` it refers to.
    pub fn describe(&self, model: &crate::FeModel) -> String {
        match self {
            Region::Parts(names) => names.join(", "),
            Region::Nodes(nodes) => format!("{} nodes", nodes.len()),
            Region::Faces(faces) => format!("{} element faces", faces.len()),
            Region::Edges(edges) => format!("{} element edges", edges.len()),
            Region::NodeSet(name) | Region::ElementSet(name) | Region::Surface(name) => {
                name.clone()
            }
            Region::Geometry(entities) => describe_entities(entities),
            Region::ReferencePoint(id) => (model.reference_point(*id))
                .map_or_else(|| "(missing reference point)".into(), |r| r.name.clone()),
            Region::RemotePoint(id) => (model.remote_item_name(*id))
                .map_or_else(|| "(missing point)".into(), str::to_string),
        }
    }

    /// Area-weighted centre of the region's element faces, of its nodes when it has no
    /// faces, e.g. picked edges or vertices; `None` without either.
    pub fn centroid(&self, mesh: &FeMesh) -> Option<[f64; 3]> {
        if let Some(centre) = crate::features::face_centroid(mesh, &self.faces(mesh)) {
            return Some(centre);
        }
        let points: Vec<[f64; 3]> = (self.nodes(mesh).into_iter())
            .filter_map(|id| mesh.node(id))
            .collect();
        let count = points.len() as f64;
        (!points.is_empty()).then(|| {
            let sum = points
                .iter()
                .fold([0.0; 3], |s, p| [0, 1, 2].map(|k| s[k] + p[k]));
            sum.map(|v| v / count)
        })
    }

    /// The reference point the region stands for, if it is one.
    pub fn reference_point(&self) -> Option<crate::Id> {
        match self {
            Region::ReferencePoint(id) => Some(*id),
            _ => None,
        }
    }

    /// The remote displacement whose point the region stands for, if it is one.
    pub fn remote_point(&self) -> Option<crate::Id> {
        match self {
            Region::RemotePoint(id) => Some(*id),
            _ => None,
        }
    }

    /// Whether the region refers to node or element numbers, which remeshing invalidates.
    pub fn by_mesh_ids(&self) -> bool {
        matches!(self, Region::Nodes(_) | Region::Faces(_) | Region::Edges(_))
    }
}

/// E.g. "2 faces, 1 edge".
pub fn describe_entities(entities: &[CadEntity]) -> String {
    let count = |f: fn(&CadEntity) -> bool| entities.iter().filter(|e| f(e)).count();
    let parts = [
        (count(|e| matches!(e, CadEntity::Face(_))), "face", "faces"),
        (count(|e| matches!(e, CadEntity::Edge(_))), "edge", "edges"),
        (
            count(|e| matches!(e, CadEntity::Vertex(_))),
            "vertex",
            "vertices",
        ),
    ];
    let text: Vec<String> = (parts.iter())
        .filter(|(n, ..)| *n > 0)
        .map(|&(n, one, many)| format!("{n} {}", if n == 1 { one } else { many }))
        .collect();
    let text = if text.is_empty() {
        "no geometry".into()
    } else {
        text.join(", ")
    };
    match entities.iter().filter(|e| e.is_lost()).count() {
        0 => text,
        lost => format!("{text} ({lost} not found)"),
    }
}

/// All nodes (corners and midside) of element faces.
fn face_nodes<'a>(
    mesh: &'a FeMesh,
    faces: &'a [(ElementId, u8)],
) -> impl Iterator<Item = NodeId> + 'a {
    faces.iter().flat_map(move |&(element, face)| {
        let element = mesh.element(element);
        let nodes: Vec<NodeId> = element
            .and_then(|e| {
                let topology = e.faces().get(usize::from(face).checked_sub(1)?)?;
                let quadratic = e.shape.is_quadratic();
                Some(
                    topology
                        .corners
                        .iter()
                        .chain(if quadratic { topology.mids } else { &[] })
                        .filter_map(|&local| e.nodes.get(local).copied())
                        .collect(),
                )
            })
            .unwrap_or_default();
        nodes
    })
}
