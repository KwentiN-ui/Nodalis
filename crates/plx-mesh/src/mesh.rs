use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::cad::CadMap;
use crate::element::{ElementFamily, ElementShape, FaceTopology};
use crate::fast_map::FastMap;

pub type NodeId = u32;
pub type ElementId = u32;

#[derive(Clone, Debug, PartialEq)]
pub struct Element {
    pub id: ElementId,
    /// Type name as written in the input file, e.g. `C3D10` or `S8R`.
    pub type_name: String,
    pub shape: ElementShape,
    pub nodes: Vec<NodeId>,
}

impl Element {
    /// Whether this is a plane stress, plane strain or axisymmetric element of a 2D model.
    /// Its faces are its edges then, see [`Self::faces`].
    pub fn is_plane(&self) -> bool {
        let name = self.type_name.as_bytes();
        ["CPS", "CPE", "CAX"]
            .iter()
            .any(|p| name.len() >= 3 && name[..3].eq_ignore_ascii_case(p.as_bytes()))
    }

    /// Whether this is a shell or membrane element of a 3D model.
    pub fn is_shell(&self) -> bool {
        self.shape.family() == ElementFamily::Surface && !self.is_plane()
    }

    /// Faces in CalculiX face order. A 2D element's faces are its edges, as CalculiX numbers
    /// them in surfaces and distributed loads (S1 = P1 = edge from node 1 to 2). A shell has
    /// its bottom (S1, NEG) and top (S2, POS), both in the element's node order, and then
    /// its edges (S3 = edge from node 1 to 2).
    pub fn faces(&self) -> &'static [FaceTopology] {
        if self.is_plane() {
            self.shape.edges()
        } else if self.is_shell() {
            self.shape.shell_faces()
        } else {
            self.shape.faces()
        }
    }

    /// The faces that are edges of the element, with their CalculiX face numbers: all faces
    /// of a 2D element, S3 and up of a shell; none of a solid or line element.
    pub fn edge_faces(&self) -> impl Iterator<Item = (u8, &'static FaceTopology)> {
        let first = if self.is_shell() { 2 } else { 0 };
        let faces: &'static [FaceTopology] = if self.is_plane() || self.is_shell() {
            self.faces()
        } else {
            &[]
        };
        (faces.iter().enumerate())
            .skip(first)
            .map(|(k, face)| (k as u8 + 1, face))
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SurfaceDefinition {
    /// Element faces as (element, 1-based face number S1…S6).
    ElementFaces(Vec<(ElementId, u8)>),
    Nodes(Vec<NodeId>),
}

/// A named group of elements shown and hidden together.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Part {
    pub name: String,
    pub elements: Vec<ElementId>,
}

#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum MeshError {
    #[error("Element {id} of type {type_name} needs {expected} nodes, but has {actual}")]
    WrongNodeCount {
        id: ElementId,
        type_name: String,
        expected: usize,
        actual: usize,
    },
    #[error("Element {0} is defined twice")]
    DuplicateElement(ElementId),
}

/// Finite element mesh with nodes, elements, sets and parts.
///
/// Set, surface and part names are stored upper case, because the solver treats them case-insensitively.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(try_from = "MeshFile", into = "MeshFile")]
pub struct FeMesh {
    pub(crate) node_ids: Vec<NodeId>,
    pub(crate) coords: Vec<[f64; 3]>,
    pub(crate) node_lookup: FastMap<NodeId, usize>,
    pub(crate) elements: Vec<Element>,
    pub(crate) element_lookup: FastMap<ElementId, usize>,
    pub node_sets: BTreeMap<String, Vec<NodeId>>,
    pub element_sets: BTreeMap<String, Vec<ElementId>>,
    pub surfaces: BTreeMap<String, SurfaceDefinition>,
    pub parts: Vec<Part>,
    /// Where the CAD entities lie, for a mesh generated from geometry.
    pub cad: CadMap,
}

impl FeMesh {
    /// Adds a node or moves an existing one; returns true if the id was already defined.
    pub fn set_node(&mut self, id: NodeId, coords: [f64; 3]) -> bool {
        match self.node_lookup.get(&id) {
            Some(&index) => {
                self.coords[index] = coords;
                true
            }
            None => {
                self.node_lookup.insert(id, self.node_ids.len());
                self.node_ids.push(id);
                self.coords.push(coords);
                false
            }
        }
    }

    pub fn add_element(&mut self, element: Element) -> Result<(), MeshError> {
        let expected = element.shape.node_count();
        if element.nodes.len() != expected {
            return Err(MeshError::WrongNodeCount {
                id: element.id,
                type_name: element.type_name,
                expected,
                actual: element.nodes.len(),
            });
        }
        if self.element_lookup.contains_key(&element.id) {
            return Err(MeshError::DuplicateElement(element.id));
        }
        self.element_lookup.insert(element.id, self.elements.len());
        self.elements.push(element);
        Ok(())
    }

    /// Separates the mesh along a cut: the given elements get copies of the given nodes, at
    /// the same place, under the new ids of `copies`, and no longer share them with the rest.
    pub fn split_nodes(
        &mut self,
        elements: &BTreeSet<ElementId>,
        copies: &BTreeMap<NodeId, NodeId>,
    ) {
        for (&node, &copy) in copies {
            if let Some(coords) = self.node(node) {
                self.set_node(copy, coords);
            }
        }
        for element in (self.elements.iter_mut()).filter(|e| elements.contains(&e.id)) {
            for node in &mut element.nodes {
                if let Some(&copy) = copies.get(node) {
                    *node = copy;
                }
            }
        }
    }

    pub fn node_count(&self) -> usize {
        self.node_ids.len()
    }

    pub fn element_count(&self) -> usize {
        self.elements.len()
    }

    /// A hash of the nodes and elements, the same for the same mesh in every program run,
    /// to tell whether results of two runs share their mesh.
    pub fn fingerprint(&self) -> u64 {
        let mut hash = Fnv::new();
        for (id, coords) in self.node_ids.iter().zip(&self.coords) {
            hash.add(&id.to_le_bytes());
            for c in coords {
                hash.add(&c.to_le_bytes());
            }
        }
        for element in &self.elements {
            hash.add(&element.id.to_le_bytes());
            for node in &element.nodes {
                hash.add(&node.to_le_bytes());
            }
        }
        hash.finish()
    }

    /// A hash of everything of the mesh that goes into the input file: nodes, elements with
    /// their types, parts, sets and surfaces. Unlike [`FeMesh::fingerprint`] it changes when
    /// only an element type or a set changes, so that results computed on another state of
    /// the mesh are told apart.
    pub fn input_fingerprint(&self) -> u64 {
        let mut hash = Fnv::new();
        hash.add(&self.fingerprint().to_le_bytes());
        for element in &self.elements {
            hash.add_str(&element.type_name);
        }
        hash.add_debug(&self.parts);
        hash.add_debug(&self.node_sets);
        hash.add_debug(&self.element_sets);
        hash.add_debug(&self.surfaces);
        hash.finish()
    }

    pub fn node_ids(&self) -> &[NodeId] {
        &self.node_ids
    }

    /// Coordinates in insertion order, parallel to [`Self::node_ids`].
    pub fn coords(&self) -> &[[f64; 3]] {
        &self.coords
    }

    /// Enlarges the mesh by `factor` about the origin, e.g. when its length unit changes.
    pub fn scale(&mut self, factor: f64) {
        for point in &mut self.coords {
            *point = point.map(|c| c * factor);
        }
    }

    pub fn node_index(&self, id: NodeId) -> Option<usize> {
        self.node_lookup.get(&id).copied()
    }

    pub fn node(&self, id: NodeId) -> Option<[f64; 3]> {
        self.node_index(id).map(|index| self.coords[index])
    }

    pub fn elements(&self) -> &[Element] {
        &self.elements
    }

    pub fn element_index(&self, id: ElementId) -> Option<usize> {
        self.element_lookup.get(&id).copied()
    }

    pub fn element(&self, id: ElementId) -> Option<&Element> {
        self.element_index(id).map(|index| &self.elements[index])
    }

    /// The element edges between the end nodes of `segments`, as the end nodes followed by
    /// the midside node of a quadratic element: the lines of a picked edge of a shell or a
    /// solid. A segment no element has as an edge is left out.
    pub fn edge_lines(&self, segments: &[[NodeId; 2]]) -> Vec<Vec<NodeId>> {
        let wanted: BTreeSet<[NodeId; 2]> = segments
            .iter()
            .map(|&[a, b]| [a.min(b), a.max(b)])
            .collect();
        let mut lines: BTreeMap<[NodeId; 2], Vec<NodeId>> = BTreeMap::new();
        for element in &self.elements {
            let quadratic = element.shape.is_quadratic();
            let node = |local: usize| element.nodes.get(local).copied();
            for face in element.faces() {
                let corners = face.corners;
                // A 2D element's faces are its edges already.
                let count = if corners.len() == 2 { 1 } else { corners.len() };
                for k in 0..count {
                    let ends = (node(corners[k]), node(corners[(k + 1) % corners.len()]));
                    let (Some(a), Some(b)) = ends else {
                        continue;
                    };
                    let key = [a.min(b), a.max(b)];
                    if !wanted.contains(&key) || lines.contains_key(&key) {
                        continue;
                    }
                    let mut line = vec![a, b];
                    if quadratic && let Some(m) = face.mids.get(k).and_then(|&l| node(l)) {
                        line.push(m);
                    }
                    lines.insert(key, line);
                }
            }
        }
        lines.into_values().collect()
    }

    /// Axis-aligned bounding box of all nodes.
    pub fn bounds(&self) -> Option<([f64; 3], [f64; 3])> {
        let mut coords = self.coords.iter();
        let first = *coords.next()?;
        Some(coords.fold((first, first), |(mut min, mut max), c| {
            for axis in 0..3 {
                min[axis] = min[axis].min(c[axis]);
                max[axis] = max[axis].max(c[axis]);
            }
            (min, max)
        }))
    }

    /// Element node references that point to undefined nodes, as (element, node).
    pub fn missing_nodes(&self) -> Vec<(ElementId, NodeId)> {
        self.elements
            .iter()
            .flat_map(|e| e.nodes.iter().map(move |&n| (e.id, n)))
            .filter(|&(_, n)| !self.node_lookup.contains_key(&n))
            .collect()
    }

    /// Changes the type names of the elements, e.g. when the model space changes.
    pub fn retype_elements(&mut self, mut type_name: impl FnMut(&str, ElementShape) -> String) {
        for element in &mut self.elements {
            element.type_name = type_name(&element.type_name, element.shape);
        }
    }

    /// Gives one element another type name; false if there is no element of the id.
    pub fn set_element_type(&mut self, id: ElementId, type_name: String) -> bool {
        match self.element_index(id) {
            Some(index) => {
                self.elements[index].type_name = type_name;
                true
            }
            None => false,
        }
    }

    /// Reverses the node order of triangles and quadrilaterals for which `flip` holds, which
    /// turns their normal around; corners and midside nodes stay matched.
    pub fn flip_surface_elements(&mut self, mut flip: impl FnMut(&Element) -> bool) {
        for element in &mut self.elements {
            let order: &[usize] = match element.shape {
                ElementShape::Tri3 => &[0, 2, 1],
                ElementShape::Tri6 => &[0, 2, 1, 5, 4, 3],
                ElementShape::Quad4 => &[0, 3, 2, 1],
                ElementShape::Quad8 => &[0, 3, 2, 1, 7, 6, 5, 4],
                _ => continue,
            };
            if flip(element) {
                element.nodes = order.iter().map(|&k| element.nodes[k]).collect();
            }
        }
    }

    /// Replaces nodes by others everywhere they are referred to (elements, sets, surfaces,
    /// the CAD map) and drops them, e.g. to join line parts at a point.
    pub fn merge_nodes(&mut self, replaced: &BTreeMap<NodeId, NodeId>) {
        if replaced.is_empty() {
            return;
        }
        let new = |n: NodeId| replaced.get(&n).copied().unwrap_or(n);
        let relist = |nodes: &[NodeId]| -> Vec<NodeId> {
            let mut nodes: Vec<NodeId> = nodes.iter().map(|&n| new(n)).collect();
            nodes.sort_unstable();
            nodes.dedup();
            nodes
        };
        for element in &mut self.elements {
            for node in &mut element.nodes {
                *node = new(*node);
            }
        }
        for nodes in self.node_sets.values_mut() {
            *nodes = relist(nodes);
        }
        for surface in self.surfaces.values_mut() {
            if let SurfaceDefinition::Nodes(nodes) = surface {
                *nodes = relist(nodes);
            }
        }
        self.cad = CadMap {
            nodes: (self.cad.nodes.iter())
                .map(|(&entity, nodes)| (entity, relist(nodes)))
                .collect(),
            faces: self.cad.faces.clone(),
            segments: (self.cad.segments.iter())
                .map(|(&edge, segments)| (edge, segments.iter().map(|s| s.map(new)).collect()))
                .collect(),
        };
        let (ids, coords): (Vec<NodeId>, Vec<[f64; 3]>) = (self.node_ids.iter().copied())
            .zip(self.coords.iter().copied())
            .filter(|(id, _)| !replaced.contains_key(id))
            .unzip();
        self.node_ids = ids;
        self.coords = coords;
        self.node_lookup = (self.node_ids.iter().copied())
            .enumerate()
            .map(|(index, id)| (id, index))
            .collect();
    }

    /// Adds another mesh as new parts, for the import of a mesh into a model: the
    /// other mesh is numbered after the highest node and element numbers this one has, so
    /// that nothing that refers to the existing parts points into the new ones. A part, set
    /// or surface whose name is taken gets a number appended (`NAME-2`). Returns the parts
    /// as added, with their new names.
    pub fn append(&mut self, other: FeMesh) -> Vec<String> {
        let node_offset = self.node_ids.iter().copied().max().unwrap_or(0);
        let element_offset = self.elements.iter().map(|e| e.id).max().unwrap_or(0);
        let mut used: BTreeSet<String> = (self.parts.iter().map(|p| &p.name))
            .chain(self.node_sets.keys())
            .chain(self.element_sets.keys())
            .chain(self.surfaces.keys())
            .map(|n| n.to_uppercase())
            .collect();
        let mut fresh = |name: &str| -> String {
            let mut new = name.to_string();
            for number in 2.. {
                if !used.contains(&new.to_uppercase()) {
                    break;
                }
                new = format!("{name}-{number}");
            }
            used.insert(new.to_uppercase());
            new
        };
        for (&id, &coords) in other.node_ids.iter().zip(&other.coords) {
            self.set_node(id + node_offset, coords);
        }
        for element in other.elements {
            // The elements come from a valid mesh whose numbers are shifted past this one's.
            let _ = self.add_element(Element {
                id: element.id + element_offset,
                nodes: element.nodes.iter().map(|n| n + node_offset).collect(),
                ..element
            });
        }
        let shifted = |ids: &[ElementId]| -> Vec<ElementId> {
            ids.iter().map(|e| e + element_offset).collect()
        };
        let mut renamed = BTreeMap::new();
        let mut added = Vec::new();
        for part in other.parts {
            let name = fresh(&part.name);
            renamed.insert(part.name.clone(), name.clone());
            added.push(name.clone());
            self.parts.push(Part {
                name,
                elements: shifted(&part.elements),
            });
        }
        for (name, elements) in other.element_sets {
            // A part's element set keeps the part's name.
            let name = renamed.get(&name).cloned().unwrap_or_else(|| fresh(&name));
            self.element_sets.insert(name, shifted(&elements));
        }
        for (name, nodes) in other.node_sets {
            let nodes = nodes.iter().map(|n| n + node_offset).collect();
            self.node_sets.insert(fresh(&name), nodes);
        }
        for (name, surface) in other.surfaces {
            let surface = match surface {
                SurfaceDefinition::ElementFaces(faces) => SurfaceDefinition::ElementFaces(
                    faces
                        .iter()
                        .map(|&(e, f)| (e + element_offset, f))
                        .collect(),
                ),
                SurfaceDefinition::Nodes(nodes) => {
                    SurfaceDefinition::Nodes(nodes.iter().map(|n| n + node_offset).collect())
                }
            };
            self.surfaces.insert(fresh(&name), surface);
        }
        self.cad
            .extend(other.cad.offset(node_offset, element_offset));
        added
    }

    /// Whether a part, set or surface already has this name; the solver ignores case.
    pub fn name_in_use(&self, name: &str) -> bool {
        let same = |n: &String| n.eq_ignore_ascii_case(name);
        self.parts.iter().any(|p| same(&p.name))
            || self.node_sets.keys().any(same)
            || self.element_sets.keys().any(same)
            || self.surfaces.keys().any(same)
    }

    /// Removes the parts named `names` (in any case) with their elements and the nodes no
    /// other element uses; sets, surfaces and the CAD map
    /// lose them too. The rest keep their numbers. In place, as a large mesh takes long to
    /// copy.
    pub fn remove_parts(&mut self, names: &[String]) {
        let gone = |p: &Part| names.iter().any(|n| p.name.eq_ignore_ascii_case(n));
        let mut removed = vec![false; self.elements.len()];
        for part in self.parts.iter().filter(|p| gone(p)) {
            for id in &part.elements {
                if let Some(&index) = self.element_lookup.get(id) {
                    removed[index] = true;
                }
            }
        }
        self.parts.retain(|p| !gone(p));
        self.remove_flagged(&removed, true);
    }

    /// Removes the elements `ids` from the mesh, its parts, sets, surfaces and CAD map but
    /// keeps every node, so that results of the nodes keep their order; parts left empty go.
    pub fn remove_elements(&mut self, ids: &BTreeSet<ElementId>) {
        let removed: Vec<bool> = self.elements.iter().map(|e| ids.contains(&e.id)).collect();
        if !removed.contains(&true) {
            return;
        }
        for part in &mut self.parts {
            part.elements.retain(|id| !ids.contains(id));
        }
        self.parts.retain(|p| !p.elements.is_empty());
        self.remove_flagged(&removed, false);
    }

    /// Removes the elements flagged by index and, with `nodes`, the nodes only they use.
    fn remove_flagged(&mut self, removed: &[bool], nodes: bool) {
        if !removed.contains(&true) {
            return;
        }
        // The nodes of the removed elements that no kept element uses go.
        let mut used = vec![false; self.node_ids.len()];
        let mut touched = vec![false; self.node_ids.len()];
        for (element, &removed) in self.elements.iter().zip(removed) {
            let flags = if removed { &mut touched } else { &mut used };
            for node in &element.nodes {
                if let Some(&index) = self.node_lookup.get(node) {
                    flags[index] = true;
                }
            }
        }
        let node_gone: Vec<bool> = (touched.iter().zip(&used))
            .map(|(&touched, &used)| nodes && touched && !used)
            .collect();
        // Only sets, surfaces and the CAD map need the ids.
        if !(self.node_sets.is_empty()
            && self.element_sets.is_empty()
            && self.surfaces.is_empty()
            && self.cad.is_empty())
        {
            let elements: BTreeSet<ElementId> = (self.elements.iter().zip(removed))
                .filter(|(_, removed)| **removed)
                .map(|(e, _)| e.id)
                .collect();
            let nodes: BTreeSet<NodeId> = (self.node_ids.iter().zip(&node_gone))
                .filter(|(_, gone)| **gone)
                .map(|(&id, _)| id)
                .collect();
            for set in self.node_sets.values_mut() {
                set.retain(|n| !nodes.contains(n));
            }
            for set in self.element_sets.values_mut() {
                set.retain(|e| !elements.contains(e));
            }
            for surface in self.surfaces.values_mut() {
                match surface {
                    SurfaceDefinition::ElementFaces(faces) => {
                        faces.retain(|(e, _)| !elements.contains(e));
                    }
                    SurfaceDefinition::Nodes(set) => set.retain(|n| !nodes.contains(n)),
                }
            }
            self.cad = self.cad.without(&elements, &nodes);
        }
        let mut index = 0;
        self.elements.retain(|_| {
            index += 1;
            !removed[index - 1]
        });
        let mut index = 0;
        self.node_ids.retain(|_| {
            index += 1;
            !node_gone[index - 1]
        });
        let mut index = 0;
        self.coords.retain(|_| {
            index += 1;
            !node_gone[index - 1]
        });
        self.element_lookup =
            FastMap::with_capacity_and_hasher(self.elements.len(), Default::default());
        for (index, element) in self.elements.iter().enumerate() {
            self.element_lookup.insert(element.id, index);
        }
        self.node_lookup =
            FastMap::with_capacity_and_hasher(self.node_ids.len(), Default::default());
        for (index, &id) in self.node_ids.iter().enumerate() {
            self.node_lookup.insert(id, index);
        }
    }

    /// Merges parts into the first of them (by index), as the Merge of mesh parts:
    /// the others' elements join it and their element sets of the same name join its set.
    /// Returns the name kept and the names that went.
    pub fn merge_parts(&mut self, indices: &[usize]) -> Option<(String, Vec<String>)> {
        let mut indices = indices.to_vec();
        indices.sort_unstable();
        indices.dedup();
        let (&target, others) = indices.split_first()?;
        if others.is_empty() || indices.last().is_some_and(|&i| i >= self.parts.len()) {
            return None;
        }
        let kept = self.parts[target].name.clone();
        let mut gone = Vec::new();
        for &index in others.iter().rev() {
            let part = self.parts.remove(index);
            self.parts[target].elements.extend(part.elements);
            if let Some(elements) = self.element_sets.remove(&part.name) {
                self.element_sets
                    .entry(kept.clone())
                    .or_default()
                    .extend(elements);
            }
            gone.push(part.name);
        }
        gone.reverse();
        Some((kept, gone))
    }

    /// Renames a part together with the element set of the same name that an input file
    /// defines for it. Returns the old name.
    pub fn rename_part(&mut self, index: usize, name: &str) -> Option<String> {
        let part = self.parts.get_mut(index)?;
        let old = std::mem::replace(&mut part.name, name.to_string());
        if let Some(elements) = self.element_sets.remove(&old) {
            self.element_sets.insert(name.to_string(), elements);
        }
        Some(old)
    }

    /// Groups the elements into the parts of the mesh they were computed from
    /// maps the parts of the model onto its results: each part takes its elements this mesh
    /// has, in the order of `parts`; elements no part claims stay in the parts they were in.
    /// Returns false, leaving the parts alone, when none of the elements is claimed.
    pub fn adopt_parts(&mut self, parts: &[Part]) -> bool {
        let mut claimed = vec![false; self.elements.len()];
        let mut adopted = Vec::new();
        for part in parts {
            let elements: Vec<ElementId> = (part.elements.iter().copied())
                .filter(|&id| {
                    self.element_index(id)
                        .is_some_and(|index| !std::mem::replace(&mut claimed[index], true))
                })
                .collect();
            if !elements.is_empty() {
                adopted.push(Part {
                    name: part.name.clone(),
                    elements,
                });
            }
        }
        if adopted.is_empty() {
            return false;
        }
        for part in std::mem::take(&mut self.parts) {
            let elements: Vec<ElementId> = (part.elements.into_iter())
                .filter(|&id| self.element_index(id).is_some_and(|index| !claimed[index]))
                .collect();
            if elements.is_empty() {
                continue;
            }
            let mut name = part.name;
            let base = name.clone();
            let same = |a: &Part, n: &str| a.name.eq_ignore_ascii_case(n);
            for number in 2.. {
                if !adopted.iter().any(|a| same(a, &name)) {
                    break;
                }
                name = format!("{base}-{number}");
            }
            adopted.push(Part { name, elements });
        }
        self.parts = adopted;
        true
    }
}

/// How a mesh is stored in project files: nodes and elements without lookup tables; element
/// shapes follow from the type names.
#[derive(Serialize, Deserialize)]
struct MeshFile {
    pub(crate) node_ids: Vec<NodeId>,
    pub(crate) coords: Vec<[f64; 3]>,
    elements: Vec<(ElementId, String, Vec<NodeId>)>,
    #[serde(default)]
    node_sets: BTreeMap<String, Vec<NodeId>>,
    #[serde(default)]
    element_sets: BTreeMap<String, Vec<ElementId>>,
    #[serde(default)]
    surfaces: BTreeMap<String, SurfaceDefinition>,
    #[serde(default)]
    parts: Vec<Part>,
    #[serde(default, skip_serializing_if = "CadMap::is_empty")]
    cad: CadMap,
}

impl From<FeMesh> for MeshFile {
    fn from(mesh: FeMesh) -> Self {
        Self {
            node_ids: mesh.node_ids,
            coords: mesh.coords,
            elements: (mesh.elements.into_iter())
                .map(|e| (e.id, e.type_name, e.nodes))
                .collect(),
            node_sets: mesh.node_sets,
            element_sets: mesh.element_sets,
            surfaces: mesh.surfaces,
            parts: mesh.parts,
            cad: mesh.cad,
        }
    }
}

impl TryFrom<MeshFile> for FeMesh {
    type Error = String;

    fn try_from(file: MeshFile) -> Result<Self, String> {
        if file.node_ids.len() != file.coords.len() {
            return Err("Node numbers and coordinates do not match".into());
        }
        let mut mesh = FeMesh::default();
        for (id, coords) in file.node_ids.into_iter().zip(file.coords) {
            mesh.set_node(id, coords);
        }
        for (id, type_name, nodes) in file.elements {
            let shape = ElementShape::from_type_name(&type_name)
                .ok_or_else(|| format!("Element {id}: unknown type {type_name}"))?;
            mesh.add_element(Element {
                id,
                type_name,
                shape,
                nodes,
            })
            .map_err(|e| e.to_string())?;
        }
        mesh.node_sets = file.node_sets;
        mesh.element_sets = file.element_sets;
        mesh.surfaces = file.surfaces;
        mesh.parts = file.parts;
        mesh.cad = file.cad;
        Ok(mesh)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn edge_lines_find_element_edges_with_their_midside_nodes() {
        use crate::{Element, ElementShape};
        let mut mesh = super::FeMesh::default();
        for (id, p) in (1..).zip([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]) {
            mesh.set_node(id, p);
        }
        for id in 5..=10 {
            mesh.set_node(id, [0.1 * f64::from(id), 0.5, 0.5]);
        }
        mesh.add_element(Element {
            id: 1,
            type_name: "C3D10".into(),
            shape: ElementShape::Tet10,
            nodes: (1..=10).collect(),
        })
        .unwrap();
        // C3D10: 5 between 1-2, 7 between 3-1, 9 between 2-4; 1-99 is no edge.
        let lines = mesh.edge_lines(&[[2, 1], [1, 3], [4, 2], [1, 99]]);
        let found: Vec<[u32; 3]> = (lines.iter())
            .map(|l| [l[0].min(l[1]), l[0].max(l[1]), l[2]])
            .collect();
        assert_eq!(found, vec![[1, 2, 5], [1, 3, 7], [2, 4, 9]]);
    }

    #[test]
    fn merging_parts_keeps_the_first_and_joins_its_element_set() {
        let mut mesh = super::FeMesh::default();
        for (name, elements) in [("A", vec![1, 2]), ("B", vec![3]), ("C", vec![4, 5])] {
            mesh.parts.push(super::Part {
                name: name.into(),
                elements: elements.clone(),
            });
            mesh.element_sets.insert(name.into(), elements);
        }
        assert_eq!(mesh.merge_parts(&[1]), None);
        let (kept, gone) = mesh.merge_parts(&[2, 0]).unwrap();
        assert_eq!((kept.as_str(), gone), ("A", vec!["C".to_string()]));
        assert_eq!(mesh.parts.len(), 2);
        assert_eq!(mesh.parts[0].elements, [1, 2, 4, 5]);
        assert_eq!(mesh.parts[1].name, "B");
        assert_eq!(mesh.element_sets["A"], [1, 2, 4, 5]);
        assert!(!mesh.element_sets.contains_key("C"));
    }

    use super::*;

    #[test]
    fn removed_elements_leave_their_nodes_and_empty_parts_go() {
        let mut mesh = FeMesh::default();
        for id in 1..=3 {
            mesh.set_node(id, [id as f64, 0.0, 0.0]);
        }
        for id in 1..=3 {
            mesh.add_element(Element {
                id,
                type_name: "T3D2".into(),
                shape: ElementShape::Line2,
                nodes: vec![id, id % 3 + 1],
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "A".into(),
            elements: vec![1, 2],
        });
        mesh.parts.push(Part {
            name: "GAP".into(),
            elements: vec![3],
        });
        mesh.element_sets.insert("ALL".into(), vec![1, 2, 3]);
        mesh.remove_elements(&BTreeSet::from([2, 3]));
        assert_eq!(mesh.element_count(), 1);
        assert_eq!(mesh.element(1).map(|e| e.id), Some(1));
        assert_eq!(mesh.element(3), None);
        assert_eq!(mesh.node_ids(), [1, 2, 3]);
        assert_eq!(mesh.parts.len(), 1);
        assert_eq!(mesh.parts[0].elements, [1]);
        assert_eq!(mesh.element_sets["ALL"], [1]);
    }

    #[test]
    fn adopted_parts_regroup_the_elements() {
        let mut mesh = FeMesh::default();
        for id in 1..=4 {
            mesh.set_node(id, [id as f64, 0.0, 0.0]);
        }
        for id in 1..=5 {
            mesh.add_element(Element {
                id,
                type_name: "T3D2".into(),
                shape: ElementShape::Line2,
                nodes: vec![1, 2],
            })
            .unwrap();
        }
        let part = |name: &str, elements: Vec<ElementId>| Part {
            name: name.into(),
            elements,
        };
        mesh.parts = vec![part("Steel", vec![1, 2, 3, 4]), part("Wood", vec![5])];
        // Nothing of the source in the mesh: the parts stay.
        assert!(!mesh.adopt_parts(&[part("Other", vec![9])]));
        assert_eq!(mesh.parts[0].name, "Steel");
        // Element 9 is not in the mesh, 4 and 5 are claimed by no source part.
        let source = [part("Solid-1", vec![1, 3, 9]), part("steel", vec![2, 1])];
        assert!(mesh.adopt_parts(&source));
        let parts: Vec<_> = (mesh.parts.iter())
            .map(|p| (p.name.as_str(), p.elements.clone()))
            .collect();
        assert_eq!(
            parts,
            [
                ("Solid-1", vec![1, 3]),
                ("steel", vec![2]),
                ("Steel-2", vec![4]),
                ("Wood", vec![5]),
            ]
        );
    }

    #[test]
    fn an_appended_mesh_is_numbered_on_and_renamed() {
        let beam = |first: NodeId, id: ElementId, part: &str| {
            let mut mesh = FeMesh::default();
            mesh.set_node(first, [0.0; 3]);
            mesh.set_node(first + 1, [1.0, 0.0, 0.0]);
            mesh.add_element(Element {
                id,
                type_name: "B31".into(),
                shape: ElementShape::Line2,
                nodes: vec![first, first + 1],
            })
            .unwrap();
            mesh.parts.push(Part {
                name: part.into(),
                elements: vec![id],
            });
            mesh.element_sets.insert(part.into(), vec![id]);
            mesh.node_sets.insert("ENDS".into(), vec![first, first + 1]);
            mesh
        };
        let mut mesh = beam(1, 1, "BEAM");
        let added = mesh.append(beam(1, 1, "BEAM"));
        assert_eq!(added, ["BEAM-2"]);
        assert_eq!(mesh.node_ids(), [1, 2, 3, 4]);
        assert_eq!(mesh.element(2).unwrap().nodes, [3, 4]);
        assert_eq!(mesh.parts[1].name, "BEAM-2");
        assert_eq!(mesh.parts[1].elements, [2]);
        assert_eq!(mesh.element_sets["BEAM-2"], [2]);
        assert_eq!(mesh.element_sets["BEAM"], [1]);
        assert_eq!(mesh.node_sets["ENDS"], [1, 2]);
        assert_eq!(mesh.node_sets["ENDS-2"], [3, 4]);
        // Into an empty mesh, the numbers stay.
        let mut empty = FeMesh::default();
        assert_eq!(empty.append(beam(5, 7, "BEAM")), ["BEAM"]);
        assert_eq!(empty.node_ids(), [5, 6]);
        assert!(empty.element(7).is_some());
    }

    #[test]
    fn the_fingerprint_changes_with_the_mesh() {
        let mut mesh = FeMesh::default();
        mesh.set_node(1, [0.0, 0.0, 0.0]);
        mesh.set_node(2, [1.0, 0.0, 0.0]);
        let same = mesh.clone();
        assert_eq!(mesh.fingerprint(), same.fingerprint());
        mesh.set_node(2, [1.0, 0.5, 0.0]);
        assert_ne!(mesh.fingerprint(), same.fingerprint());
    }

    #[test]
    fn merged_nodes_vanish_everywhere() {
        use crate::cad::CadEntity;
        let mut mesh = FeMesh::default();
        for (id, x) in [(1, 0.0), (2, 10.0), (3, 10.0), (4, 20.0)] {
            mesh.set_node(id, [x, 0.0, 0.0]);
        }
        for (id, nodes) in [(1, vec![1, 2]), (2, vec![3, 4])] {
            mesh.add_element(Element {
                id,
                type_name: "B31".into(),
                shape: ElementShape::Line2,
                nodes,
            })
            .unwrap();
        }
        mesh.node_sets.insert("ENDS".into(), vec![2, 3, 4]);
        mesh.surfaces
            .insert("S".into(), SurfaceDefinition::Nodes(vec![3]));
        mesh.cad.nodes.insert(CadEntity::Vertex(2), vec![2, 3]);
        mesh.cad.segments.insert(2, vec![[3, 4]]);
        mesh.merge_nodes(&BTreeMap::from([(3, 2)]));
        assert_eq!(mesh.node_ids(), [1, 2, 4]);
        assert_eq!(mesh.node_index(4), Some(2));
        assert!(mesh.node(3).is_none());
        assert_eq!(mesh.element(2).unwrap().nodes, [2, 4]);
        assert_eq!(mesh.node_sets["ENDS"], [2, 4]);
        assert_eq!(mesh.surfaces["S"], SurfaceDefinition::Nodes(vec![2]));
        assert_eq!(mesh.cad.nodes[&CadEntity::Vertex(2)], [2]);
        assert_eq!(mesh.cad.segments[&2], [[2, 4]]);
        assert!(mesh.missing_nodes().is_empty());
    }

    #[test]
    fn meshes_survive_serialization() {
        let mut mesh = FeMesh::default();
        for (id, x) in [(1, 0.0), (2, 1.0), (3, 0.5), (4, 0.25)] {
            mesh.set_node(id, [x, x * 2.0, 0.1]);
        }
        mesh.add_element(tet(7, vec![1, 2, 3, 4])).unwrap();
        mesh.parts.push(Part {
            name: "SOLID".into(),
            elements: vec![7],
        });
        mesh.surfaces
            .insert("TOP".into(), SurfaceDefinition::ElementFaces(vec![(7, 2)]));
        mesh.cad
            .nodes
            .insert(crate::CadEntity::Face(3), vec![1, 2, 3]);
        mesh.cad.faces.insert(3, vec![(7, 1)]);
        let text = ron::to_string(&mesh).unwrap();
        let read: FeMesh = ron::from_str(&text).unwrap();
        assert_eq!(read.node_ids(), mesh.node_ids());
        assert_eq!(read.coords(), mesh.coords());
        assert_eq!(read.elements(), mesh.elements());
        assert_eq!(read.element_index(7), Some(0));
        assert_eq!(read.cad, mesh.cad);
        assert_eq!((read.parts, read.surfaces), (mesh.parts, mesh.surfaces));
        let broken = text.replace("C3D4", "XYZ");
        assert!(ron::from_str::<FeMesh>(&broken).is_err());
    }

    fn tet(id: ElementId, nodes: Vec<NodeId>) -> Element {
        Element {
            id,
            type_name: "C3D4".into(),
            shape: ElementShape::Tet4,
            nodes,
        }
    }

    #[test]
    fn renamed_parts_take_their_element_set_along() {
        let mut mesh = FeMesh::default();
        mesh.parts.push(Part {
            name: "SOLID".into(),
            elements: vec![7],
        });
        mesh.element_sets.insert("SOLID".into(), vec![7]);
        mesh.node_sets.insert("FIX".into(), vec![1]);
        assert!(mesh.name_in_use("fix") && mesh.name_in_use("Solid"));
        assert_eq!(mesh.rename_part(0, "BRACKET").as_deref(), Some("SOLID"));
        assert_eq!(mesh.parts[0].name, "BRACKET");
        assert_eq!(mesh.element_sets.get("BRACKET"), Some(&vec![7]));
        assert!(!mesh.name_in_use("SOLID"));
        assert_eq!(mesh.rename_part(1, "X"), None);
    }

    #[test]
    fn nodes_are_found_by_id() {
        let mut mesh = FeMesh::default();
        assert!(!mesh.set_node(10, [1.0, 2.0, 3.0]));
        assert!(!mesh.set_node(3, [0.0, 0.0, 0.0]));
        assert!(mesh.set_node(10, [1.0, 2.0, 4.0]));
        assert_eq!(mesh.node_count(), 2);
        assert_eq!(mesh.node(10), Some([1.0, 2.0, 4.0]));
        assert_eq!(mesh.node(7), None);
        assert_eq!(mesh.bounds(), Some(([0.0, 0.0, 0.0], [1.0, 2.0, 4.0])));
    }

    #[test]
    fn elements_are_validated() {
        let mut mesh = FeMesh::default();
        mesh.add_element(tet(1, vec![1, 2, 3, 4])).unwrap();
        assert_eq!(
            mesh.add_element(tet(1, vec![1, 2, 3, 4])),
            Err(MeshError::DuplicateElement(1))
        );
        assert!(matches!(
            mesh.add_element(tet(2, vec![1, 2, 3])),
            Err(MeshError::WrongNodeCount {
                expected: 4,
                actual: 3,
                ..
            })
        ));
        for (id, x) in [(1, 0.0), (2, 1.0), (3, 2.0)] {
            mesh.set_node(id, [x, 0.0, 0.0]);
        }
        assert_eq!(mesh.missing_nodes(), vec![(1, 4)]);
        assert_eq!(mesh.element(1).map(|e| e.shape), Some(ElementShape::Tet4));
    }
}

/// FNV-1a: a hash that is the same in every program run, unlike the standard hasher, so that
/// it can be saved and compared later.
#[derive(Clone, Copy, Debug)]
pub struct Fnv(u64);

impl Default for Fnv {
    fn default() -> Self {
        Self::new()
    }
}

impl Fnv {
    pub fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }

    pub fn add(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3);
        }
    }

    /// Adds a text followed by a separator, so that "ab" + "c" differs from "a" + "bc".
    pub fn add_str(&mut self, text: &str) {
        self.add(text.as_bytes());
        self.add(&[0xff]);
    }

    /// Adds a value by its debug text, which holds all its data and is the same in every
    /// run as long as the value has no hash maps.
    pub fn add_debug(&mut self, value: &impl std::fmt::Debug) {
        self.add_str(&format!("{value:?}"));
    }

    pub fn finish(self) -> u64 {
        self.0
    }
}
