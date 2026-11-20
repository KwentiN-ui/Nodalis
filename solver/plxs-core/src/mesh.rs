//! Nodes, sets and surfaces. Elements are in [`crate::element`].

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

/// A node number as the user wrote it.
pub type NodeId = u32;
/// An element number as the user wrote it.
pub type ElementId = u32;

/// The nodes of the model: user ids and coordinates, densely packed in the order of their
/// definition, with an index from id to position.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Nodes {
    pub ids: Vec<NodeId>,
    pub coordinates: Vec<[f64; 3]>,
    #[serde(skip)]
    index: HashMap<NodeId, u32>,
}

impl Nodes {
    /// Adds a node, or replaces the coordinates of one defined already (CalculiX takes the
    /// last definition). Returns its dense index.
    pub fn insert(&mut self, id: NodeId, coordinates: [f64; 3]) -> u32 {
        if let Some(&i) = self.index.get(&id) {
            self.coordinates[i as usize] = coordinates;
            return i;
        }
        let i = self.ids.len() as u32;
        self.ids.push(id);
        self.coordinates.push(coordinates);
        self.index.insert(id, i);
        i
    }

    /// Dense index of a node id.
    pub fn index(&self, id: NodeId) -> Option<u32> {
        self.index.get(&id).copied()
    }

    pub fn coordinates(&self, id: NodeId) -> Option<[f64; 3]> {
        self.index(id).map(|i| self.coordinates[i as usize])
    }

    pub fn len(&self) -> usize {
        self.ids.len()
    }

    pub fn is_empty(&self) -> bool {
        self.ids.is_empty()
    }

    /// Rebuilds the id index, needed after deserializing.
    pub fn rebuild_index(&mut self) {
        self.index = (self.ids.iter().enumerate())
            .map(|(i, &id)| (id, i as u32))
            .collect();
    }

    /// Ids and coordinates together.
    pub fn iter(&self) -> impl Iterator<Item = (NodeId, [f64; 3])> + '_ {
        self.ids
            .iter()
            .copied()
            .zip(self.coordinates.iter().copied())
    }
}

/// Node sets, element sets and surfaces, referred to by their handles ([`crate::NodeSetId`],
/// [`crate::ElementSetId`], [`crate::SurfaceId`]): the index in their list. Sets keep the
/// order of definition without duplicates; CalculiX's `GENERATE` ranges are expanded when
/// read. Names are labels only.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Sets {
    pub node_sets: Vec<NodeSet>,
    pub element_sets: Vec<ElementSet>,
    pub surfaces: Vec<NamedSurface>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct NodeSet {
    /// Label for messages and the input file.
    pub name: String,
    pub nodes: Vec<NodeId>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ElementSet {
    /// Label for messages and the input file.
    pub name: String,
    pub elements: Vec<ElementId>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NamedSurface {
    /// Label for messages and the input file.
    pub name: String,
    pub surface: Surface,
}

impl Sets {
    /// The sets and surfaces with their labels only, no members: what results keep to name
    /// the sets their history is of.
    pub fn labels(&self) -> Sets {
        Sets {
            node_sets: (self.node_sets.iter())
                .map(|s| NodeSet {
                    name: s.name.clone(),
                    nodes: Vec::new(),
                })
                .collect(),
            element_sets: (self.element_sets.iter())
                .map(|s| ElementSet {
                    name: s.name.clone(),
                    elements: Vec::new(),
                })
                .collect(),
            surfaces: (self.surfaces.iter())
                .map(|s| NamedSurface {
                    name: s.name.clone(),
                    surface: Surface::Node(Vec::new()),
                })
                .collect(),
        }
    }

    /// The members of a node set, none for a handle out of range.
    pub fn nodes(&self, id: crate::NodeSetId) -> &[NodeId] {
        self.node_sets.get(id.index()).map_or(&[], |s| &s.nodes)
    }

    /// The members of an element set, none for a handle out of range.
    pub fn elements(&self, id: crate::ElementSetId) -> &[ElementId] {
        (self.element_sets.get(id.index())).map_or(&[], |s| &s.elements)
    }

    pub fn surface(&self, id: crate::SurfaceId) -> Option<&Surface> {
        self.surfaces.get(id.index()).map(|s| &s.surface)
    }

    pub fn node_set_name(&self, id: crate::NodeSetId) -> &str {
        self.node_sets.get(id.index()).map_or("?", |s| &s.name)
    }

    pub fn element_set_name(&self, id: crate::ElementSetId) -> &str {
        (self.element_sets.get(id.index())).map_or("?", |s| &s.name)
    }

    pub fn surface_name(&self, id: crate::SurfaceId) -> &str {
        self.surfaces.get(id.index()).map_or("?", |s| &s.name)
    }

    /// The name of the set of a history output.
    pub fn history_set_name(&self, set: crate::step::HistorySet) -> &str {
        match set {
            crate::step::HistorySet::Nodes(set) => self.node_set_name(set),
            crate::step::HistorySet::Elements(set) => self.element_set_name(set),
        }
    }

    /// A new node set.
    pub fn add_node_set(
        &mut self,
        name: impl Into<String>,
        ids: impl IntoIterator<Item = NodeId>,
    ) -> crate::NodeSetId {
        let mut nodes = Vec::new();
        append_unique(&mut nodes, ids);
        self.node_sets.push(NodeSet {
            name: name.into(),
            nodes,
        });
        crate::NodeSetId::at(self.node_sets.len() - 1)
    }

    /// A new element set.
    pub fn add_element_set(
        &mut self,
        name: impl Into<String>,
        ids: impl IntoIterator<Item = ElementId>,
    ) -> crate::ElementSetId {
        let mut elements = Vec::new();
        append_unique(&mut elements, ids);
        self.element_sets.push(ElementSet {
            name: name.into(),
            elements,
        });
        crate::ElementSetId::at(self.element_sets.len() - 1)
    }

    /// A new surface.
    pub fn add_surface(&mut self, name: impl Into<String>, surface: Surface) -> crate::SurfaceId {
        self.surfaces.push(NamedSurface {
            name: name.into(),
            surface,
        });
        crate::SurfaceId::at(self.surfaces.len() - 1)
    }

    /// Appends ids to a node set; ids already in the set are skipped.
    pub fn add_nodes(&mut self, id: crate::NodeSetId, ids: impl IntoIterator<Item = NodeId>) {
        if let Some(set) = self.node_sets.get_mut(id.index()) {
            append_unique(&mut set.nodes, ids);
        }
    }

    pub fn add_elements(
        &mut self,
        id: crate::ElementSetId,
        ids: impl IntoIterator<Item = ElementId>,
    ) {
        if let Some(set) = self.element_sets.get_mut(id.index()) {
            append_unique(&mut set.elements, ids);
        }
    }

    /// The node set labelled `name`, the first of them. For readers of input files and
    /// tests: references hold handles.
    pub fn node_set_named(&self, name: &str) -> Option<crate::NodeSetId> {
        (self.node_sets.iter().position(|s| s.name == name)).map(crate::NodeSetId::at)
    }

    /// The element set labelled `name`, see [`Self::node_set_named`].
    pub fn element_set_named(&self, name: &str) -> Option<crate::ElementSetId> {
        (self.element_sets.iter().position(|s| s.name == name)).map(crate::ElementSetId::at)
    }

    /// The surface labelled `name`, see [`Self::node_set_named`].
    pub fn surface_named(&self, name: &str) -> Option<crate::SurfaceId> {
        (self.surfaces.iter().position(|s| s.name == name)).map(crate::SurfaceId::at)
    }

    /// The members of the node set labelled `name`, see [`Self::node_set_named`].
    pub fn named_nodes(&self, name: &str) -> Option<&Vec<NodeId>> {
        (self.node_sets.iter().find(|s| s.name == name)).map(|s| &s.nodes)
    }

    /// The members of the element set labelled `name`, see [`Self::node_set_named`].
    pub fn named_elements(&self, name: &str) -> Option<&Vec<ElementId>> {
        (self.element_sets.iter().find(|s| s.name == name)).map(|s| &s.elements)
    }

    /// The surface labelled `name`, see [`Self::node_set_named`].
    pub fn named_surface(&self, name: &str) -> Option<&Surface> {
        (self.surfaces.iter().find(|s| s.name == name)).map(|s| &s.surface)
    }
}

fn append_unique<T: Copy + Ord + std::hash::Hash>(
    set: &mut Vec<T>,
    ids: impl IntoIterator<Item = T>,
) {
    let mut known: std::collections::HashSet<T> = set.iter().copied().collect();
    for id in ids {
        if known.insert(id) {
            set.push(id);
        }
    }
}

/// A surface: element faces (for loads, contact, ties) or nodes (for node-to-surface contact
/// slaves and ties).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Surface {
    Element(Vec<SurfaceFace>),
    Node(Vec<NodeId>),
}

/// One face of an element in a surface: the element and the CalculiX face number (`S1`..`S6`
/// of solids, `S1`/`S2`/`S3`.. of shells and 2D elements, `SPOS`/`SNEG` as `1`/`2`), kept as
/// the number after the `S`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SurfaceFace {
    pub element: ElementId,
    pub face: u8,
}

/// Where a boundary condition, load or output applies: a single node or element, or a set.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NodeRef {
    Node(NodeId),
    Set(crate::NodeSetId),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ElementRef {
    Element(ElementId),
    Set(crate::ElementSetId),
}

impl NodeRef {
    /// The node or the name of the set, for messages.
    pub fn describe(&self, sets: &Sets) -> String {
        match self {
            NodeRef::Node(id) => id.to_string(),
            NodeRef::Set(set) => match sets.node_sets.get(set.index()) {
                Some(set) => set.name.clone(),
                None => format!("#{}", set.0),
            },
        }
    }
}

impl ElementRef {
    /// The element or the name of the set, for messages.
    pub fn describe(&self, sets: &Sets) -> String {
        match self {
            ElementRef::Element(id) => id.to_string(),
            ElementRef::Set(set) => match sets.element_sets.get(set.index()) {
                Some(set) => set.name.clone(),
                None => format!("#{}", set.0),
            },
        }
    }
}
