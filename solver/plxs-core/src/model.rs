//! The model: everything outside the steps, plus the analysis as the whole input.

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use crate::amplitude::Amplitude;
use crate::constraints::Constraints;
use crate::contact::Contact;
use crate::element::Element;
use crate::material::Material;
use crate::mesh::{ElementId, NodeId, NodeRef, Nodes, Sets};
use crate::section::Section;
use crate::step::Step;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Model {
    /// `*HEADING`.
    pub heading: String,
    pub nodes: Nodes,
    pub elements: Vec<Element>,
    pub sets: Sets,
    pub materials: Vec<Material>,
    pub sections: Vec<Section>,
    pub constraints: Constraints,
    pub contact: Contact,
    pub amplitudes: Vec<Amplitude>,
    pub initial: InitialConditions,
    pub physical: PhysicalConstants,
    /// `*DESIGN VARIABLES, TYPE=COORDINATE`: node set of the design nodes (sensitivity).
    pub design_variables: Option<DesignVariables>,
    /// `*RESTART, READ`: continue from the state after step `step` of the previous run.
    pub restart_read: Option<RestartRead>,
    #[serde(skip)]
    element_index: HashMap<ElementId, u32>,
}

impl Model {
    /// Adds an element, or replaces one with the same id. Returns its dense index.
    pub fn insert_element(&mut self, element: Element) -> u32 {
        if let Some(&i) = self.element_index.get(&element.id) {
            self.elements[i as usize] = element;
            return i;
        }
        let i = self.elements.len() as u32;
        self.element_index.insert(element.id, i);
        self.elements.push(element);
        i
    }

    /// Dense index of an element id.
    pub fn element_index(&self, id: ElementId) -> Option<u32> {
        self.element_index.get(&id).copied()
    }

    pub fn element(&self, id: ElementId) -> Option<&Element> {
        self.element_index(id).map(|i| &self.elements[i as usize])
    }

    /// Rebuilds the id indices of nodes and elements, needed after deserializing.
    pub fn rebuild_indices(&mut self) {
        self.nodes.rebuild_index();
        self.element_index = (self.elements.iter().enumerate())
            .map(|(i, e)| (e.id, i as u32))
            .collect();
    }

    pub fn material(&self, name: &str) -> Option<&Material> {
        self.materials.iter().find(|m| m.name == name)
    }

    pub fn amplitude(&self, name: &str) -> Option<&Amplitude> {
        self.amplitudes.iter().find(|a| a.name == name)
    }

    /// The node ids a reference stands for: the node itself or the members of the set.
    pub fn nodes_of(&self, region: &NodeRef) -> Vec<NodeId> {
        match region {
            NodeRef::Node(id) => vec![*id],
            NodeRef::Set(name) => self.sets.node_sets.get(name).cloned().unwrap_or_default(),
        }
    }

    /// The element ids a reference stands for.
    pub fn elements_of(&self, region: &crate::mesh::ElementRef) -> Vec<ElementId> {
        match region {
            crate::mesh::ElementRef::Element(id) => vec![*id],
            crate::mesh::ElementRef::Set(name) => self
                .sets
                .element_sets
                .get(name)
                .cloned()
                .unwrap_or_default(),
        }
    }
}

/// `*INITIAL CONDITIONS`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct InitialConditions {
    /// `TYPE=TEMPERATURE`: values per node or set.
    pub temperatures: Vec<(NodeRef, f64)>,
    /// `TYPE=VELOCITY`: (node or set, DOF, value).
    pub velocities: Vec<(NodeRef, u8, f64)>,
    /// `TYPE=DISPLACEMENT`, `STRESS`, `PLASTIC STRAIN`...: not written by Nodalis.
    pub displacements: Vec<(NodeRef, u8, f64)>,
}

/// `*PHYSICAL CONSTANTS`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PhysicalConstants {
    pub absolute_zero: Option<f64>,
    pub stefan_boltzmann: Option<f64>,
    pub newton_gravity: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DesignVariables {
    pub nset: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RestartRead {
    pub step: Option<u32>,
}

/// A whole input: the model and its steps. What `plxs-input` returns and a session runs.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Analysis {
    pub model: Model,
    pub steps: Vec<Step>,
}
