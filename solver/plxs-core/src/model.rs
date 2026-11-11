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
    /// The `.eig` file of the job next to its input file, when there is one: the modes of an
    /// earlier run for modal steps without a frequency step of their own (as CalculiX reads
    /// `<job>.eig`).
    #[serde(skip)]
    pub stored_modes_file: Option<std::path::PathBuf>,
    /// The fibres of expanded shells: the nodes through the thickness of every shell node
    /// (bottom, middle, top; the ones the elements use), for the selective mass scaling of
    /// explicit steps. Set by the expansion, empty otherwise.
    #[serde(default)]
    pub shell_fibres: Vec<Vec<NodeId>>,
    /// Whether shells, membranes, beams or trusses were expanded into solids: the
    /// translations of the expanded model carry bending through thin elements, which the
    /// multigrid of the iterative solver does not fit (the automatic equation solver stays
    /// direct while the factor fits). Set by the expansion.
    #[serde(default)]
    pub expanded_structure: bool,
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

    /// The body of every node (dense node index to a dense body number): the connected
    /// components of the mesh over its continuum and structural elements. Springs and gaps
    /// connect nothing, since they couple bodies only weakly or one-sidedly, and contact
    /// (no element at all) neither: two bodies that touch stay two bodies. A node of no
    /// element is a body of its own. The multigrid of the iterative solver never groups
    /// the nodes of different bodies into one aggregate, so that its coarse spaces hold
    /// the rigid motion of every body on its own.
    pub fn node_bodies(&self) -> Vec<u32> {
        let n = self.nodes.len();
        let mut parent: Vec<u32> = (0..n as u32).collect();
        fn root(parent: &mut [u32], mut i: u32) -> u32 {
            while parent[i as usize] != i {
                let p = parent[i as usize];
                parent[i as usize] = parent[p as usize];
                i = p;
            }
            i
        }
        for element in &self.elements {
            if matches!(
                element.kind.family(),
                crate::element::ElementFamily::Spring | crate::element::ElementFamily::Gap
            ) {
                continue;
            }
            let mut first: Option<u32> = None;
            for &id in &element.nodes {
                let Some(i) = self.nodes.index(id) else {
                    continue;
                };
                match first {
                    None => first = Some(i),
                    Some(f) => {
                        let (a, b) = (root(&mut parent, f), root(&mut parent, i));
                        if a != b {
                            parent[b as usize] = a;
                        }
                    }
                }
            }
        }
        let mut number = vec![u32::MAX; n];
        let mut count = 0;
        for i in 0..n as u32 {
            let r = root(&mut parent, i) as usize;
            if number[r] == u32::MAX {
                number[r] = count;
                count += 1;
            }
            number[i as usize] = number[r];
        }
        number
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

impl Analysis {
    /// Whether a modal step (modal dynamics, steady state dynamics, complex frequency) uses
    /// modes that no `*FREQUENCY, STORAGE=YES` step before it computes: the modes of an
    /// earlier run, [`Model::stored_modes_file`].
    pub fn needs_stored_modes(&self) -> bool {
        use crate::step::Procedure;
        let mut stored = false;
        for step in &self.steps {
            match &step.procedure {
                Procedure::Frequency(f) if f.storage => stored = true,
                Procedure::ModalDynamics(_)
                | Procedure::SteadyStateDynamics(_)
                | Procedure::ComplexFrequency(_)
                    if !stored =>
                {
                    return true;
                }
                _ => {}
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::element::ElementType;

    #[test]
    fn bodies_are_the_components_over_the_elements_without_springs() {
        let mut model = Model::default();
        for id in 1..=9 {
            model.nodes.insert(id, [id as f64, 0.0, 0.0]);
        }
        // Two tetrahedra that share a node, a third one apart, joined by a spring only,
        // and a node of no element.
        for (id, kind, nodes) in [
            (1, ElementType::C3D4, vec![1, 2, 3, 4]),
            (2, ElementType::C3D4, vec![4, 5, 6, 7]),
            (3, ElementType::C3D4, vec![8, 9, 1, 2]),
        ] {
            model.insert_element(Element {
                id,
                kind,
                nodes,
                section: None,
            });
        }
        let mut model2 = model.clone();
        model2.elements[2].nodes = vec![8, 9, 9, 8];
        model2.insert_element(Element {
            id: 4,
            kind: ElementType::SPRING2,
            nodes: vec![7, 8],
            section: None,
        });
        let bodies = model.node_bodies();
        assert!(bodies.iter().all(|&b| b == 0), "{bodies:?}");
        let bodies = model2.node_bodies();
        assert_eq!(bodies[..7], [0; 7]);
        assert_eq!(bodies[7..], [1, 1]);
        let mut model3 = model2.clone();
        model3.nodes.insert(10, [0.0; 3]);
        assert_eq!(model3.node_bodies()[9], 2);
    }
}
