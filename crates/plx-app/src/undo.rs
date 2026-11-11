//! Undo and redo of what the user changed in the FE model workspace:
//! Edit > Undo and Redo.
//!
//! Everything the user sets up ends up in the model's data: the FE model, the meshing
//! settings of the geometry and the mesh. Rather than each change knowing how to revert
//! itself, the history keeps the data as it was before each change: at the end of every frame
//! it compares the data with how it last saw it. The mesh is large, so it is kept only for
//! changes that replaced it; [`crate::model::Model::set_mesh`] hands the mesh it replaces to
//! the history.
//!
//! Changes of the geometry's shapes (an import, deleted geometry parts, a new unit system)
//! cannot be undone; the history starts anew after them.

use plx_mesh::FeMesh;
use plx_model::{FeModel, Geometry, MeshSetupItem, MeshingParameters};

use crate::model::Model;

/// Changes that can be undone at most.
const DEPTH: usize = 100;
/// Changes that replaced the mesh that can be undone at most, as each keeps a mesh.
const MESH_DEPTH: usize = 10;

/// The meshing settings of the geometry.
#[derive(Clone, PartialEq)]
struct Meshing {
    parameters: MeshingParameters,
    items: Vec<MeshSetupItem>,
    compounds: Vec<plx_model::CompoundPart>,
}

impl Meshing {
    fn of(geometry: Option<&Geometry>) -> Option<Self> {
        geometry.map(|g| Self {
            parameters: g.meshing,
            items: g.mesh_items.clone(),
            compounds: g.compounds.clone(),
        })
    }
}

/// What the shapes of the geometry are, to notice when they change.
#[derive(PartialEq)]
struct Shapes {
    source: String,
    brep: String,
    part_names: Vec<String>,
}

impl Shapes {
    fn of(geometry: Option<&Geometry>) -> Option<Self> {
        geometry.map(|g| Self {
            source: g.source.clone(),
            brep: g.brep.clone(),
            part_names: g.part_names.clone(),
        })
    }

    fn matches(this: &Option<Self>, geometry: Option<&Geometry>) -> bool {
        match (this, geometry) {
            (None, None) => true,
            (Some(s), Some(g)) => {
                s.source == g.source && s.part_names == g.part_names && s.brep == g.brep
            }
            _ => false,
        }
    }
}

/// The data a change can be undone or redone to; the mesh only if the change replaced it.
struct State {
    fe: FeModel,
    meshing: Option<Meshing>,
    mesh: Option<FeMesh>,
}

/// The data as the history last saw it.
struct Seen {
    fe: FeModel,
    meshing: Option<Meshing>,
    shapes: Option<Shapes>,
}

/// The changes of a model that can be undone and redone.
#[derive(Default)]
pub struct History {
    undo: Vec<State>,
    redo: Vec<State>,
    /// `None` until the history follows the model, from the first [`record`] on.
    seen: Option<Seen>,
    /// The mesh before it was first replaced since the last [`record`].
    replaced_mesh: Option<FeMesh>,
}

impl History {
    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }

    /// Whether the history follows the model's changes.
    pub fn follows(&self) -> bool {
        self.seen.is_some()
    }

    /// Keeps the mesh a new one replaced, for undoing the change; only for a model the
    /// history follows.
    pub fn mesh_replaced(&mut self, old: FeMesh) {
        if self.seen.is_some() && self.replaced_mesh.is_none() {
            self.replaced_mesh = Some(old);
        }
    }

    /// Drops the oldest changes beyond the depth of the history.
    fn trim(&mut self) {
        let with_mesh = self.undo.iter().filter(|s| s.mesh.is_some()).count();
        if with_mesh > MESH_DEPTH {
            // Changes before the oldest mesh can no longer be reached without it.
            let oldest = self.undo.iter().position(|s| s.mesh.is_some());
            self.undo.drain(..=oldest.unwrap_or(0));
        }
        if self.undo.len() > DEPTH {
            let excess = self.undo.len() - DEPTH;
            self.undo.drain(..excess);
        }
    }
}

/// Notes what changed since the last call as one change that can be undone, and counts it
/// in the model's revision; called once at the end of every frame, after everything the
/// user did in it.
pub fn record(model: &mut Model) {
    let geometry = model.geometry.as_ref();
    let history = &mut model.history;
    let replaced = history.replaced_mesh.take();
    let shapes = match history.seen.take() {
        Some(seen) if Shapes::matches(&seen.shapes, geometry) => {
            let meshing = Meshing::of(geometry);
            if replaced.is_none() && seen.fe == model.fe && seen.meshing == meshing {
                history.seen = Some(seen);
                return;
            }
            history.undo.push(State {
                fe: seen.fe,
                meshing: seen.meshing,
                mesh: replaced,
            });
            history.redo.clear();
            history.trim();
            seen.shapes
        }
        _ => {
            history.undo.clear();
            history.redo.clear();
            Shapes::of(geometry)
        }
    };
    history.seen = Some(Seen {
        fe: model.fe.clone(),
        meshing: Meshing::of(geometry),
        shapes,
    });
    model.revision += 1;
}

/// Reverts the last change; false if there is none.
pub fn undo(model: &mut Model) -> bool {
    let Some(state) = model.history.undo.pop() else {
        return false;
    };
    let current = restore(model, state);
    model.history.redo.push(current);
    true
}

/// Makes the last undone change again; false if there is none.
pub fn redo(model: &mut Model) -> bool {
    let Some(state) = model.history.redo.pop() else {
        return false;
    };
    let current = restore(model, state);
    model.history.undo.push(current);
    model.history.trim();
    true
}

/// Puts the model's data back to `state`; returns the data it had, to go back to it.
fn restore(model: &mut Model, state: State) -> State {
    let meshing = Meshing::of(model.geometry.as_ref());
    let sets = model.fe.mesh_sets.clone();
    let fe = std::mem::replace(&mut model.fe, state.fe);
    if let (Some(geometry), Some(restored)) = (&mut model.geometry, state.meshing) {
        geometry.meshing = restored.parameters;
        geometry.mesh_items = restored.items;
        geometry.compounds = restored.compounds;
    }
    let mesh = match state.mesh {
        Some(mesh) => {
            // Parts that stay keep their colour and visibility.
            let kept: Vec<(String, [f32; 3], bool)> = (model.parts.iter())
                .map(|p| (p.name.clone(), p.color, p.visible))
                .collect();
            model.set_mesh(mesh);
            for part in &mut model.parts {
                if let Some((_, color, visible)) = kept.iter().find(|k| k.0 == part.name) {
                    part.color = *color;
                    part.visible = *visible;
                }
            }
            // The history follows the model, so it kept the mesh just replaced.
            model.history.replaced_mesh.take()
        }
        None => {
            if model.fe.mesh_sets != sets {
                model.fe.sync_mesh_sets(&sets, &mut model.mesh);
            }
            None
        }
    };
    if let Some(seen) = &mut model.history.seen {
        seen.fe = model.fe.clone();
        seen.meshing = Meshing::of(model.geometry.as_ref());
    }
    State { fe, meshing, mesh }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use plx_mesh::{Element, ElementShape, Part};
    use plx_model::Material;

    use super::*;

    fn model() -> Model {
        let mut mesh = FeMesh::default();
        for (id, coords) in [
            (1, [0.0, 0.0, 0.0]),
            (2, [1.0, 0.0, 0.0]),
            (3, [0.0, 1.0, 0.0]),
            (4, [0.0, 0.0, 1.0]),
        ] {
            mesh.set_node(id, coords);
        }
        mesh.add_element(Element {
            id: 1,
            type_name: "C3D4".into(),
            shape: ElementShape::Tet4,
            nodes: vec![1, 2, 3, 4],
        })
        .unwrap();
        mesh.parts.push(Part {
            bodies: Vec::new(),
            name: "PART-1".into(),
            elements: vec![1],
        });
        let mut model = Model::new(Path::new("test.inp"), mesh);
        record(&mut model);
        model
    }

    fn material(name: &str) -> Material {
        Material {
            name: name.into(),
            ..Material::default()
        }
    }

    #[test]
    fn changes_are_undone_and_redone_in_order() {
        let mut model = model();
        assert!(!model.history.can_undo());
        model.fe.materials.push(material("STEEL"));
        record(&mut model);
        model.fe.materials.push(material("ALUMINIUM"));
        record(&mut model);
        assert!(undo(&mut model));
        assert_eq!(model.fe.materials.len(), 1);
        assert!(undo(&mut model));
        assert!(model.fe.materials.is_empty());
        assert!(!undo(&mut model));
        assert!(redo(&mut model));
        assert!(redo(&mut model));
        assert_eq!(model.fe.materials.len(), 2);
        assert!(!redo(&mut model));
    }

    #[test]
    fn frames_without_changes_record_nothing() {
        let mut model = model();
        record(&mut model);
        record(&mut model);
        assert!(!model.history.can_undo());
        model.fe.materials.push(material("STEEL"));
        record(&mut model);
        assert!(undo(&mut model));
        // The undo itself is no change to record.
        record(&mut model);
        assert!(!model.history.can_undo());
        assert!(model.history.can_redo());
    }

    #[test]
    fn a_new_change_drops_what_could_be_redone() {
        let mut model = model();
        model.fe.materials.push(material("STEEL"));
        record(&mut model);
        undo(&mut model);
        model.fe.materials.push(material("COPPER"));
        record(&mut model);
        assert!(!model.history.can_redo());
    }

    #[test]
    fn a_renamed_part_gets_its_name_back() {
        let mut model = model();
        model.rename_part(0, "BRACKET").unwrap();
        record(&mut model);
        assert_eq!(model.parts[0].name, "BRACKET");
        assert!(undo(&mut model));
        assert_eq!(model.parts[0].name, "PART-1");
        assert_eq!(model.mesh.parts[0].name, "PART-1");
        assert!(redo(&mut model));
        assert_eq!(model.mesh.parts[0].name, "BRACKET");
    }

    #[test]
    fn a_replaced_mesh_comes_back() {
        let mut model = model();
        model.edit_mesh(|mesh, _| mesh.parts.clear());
        record(&mut model);
        assert!(model.parts.is_empty());
        assert!(undo(&mut model));
        assert_eq!(model.parts.len(), 1);
        assert!(redo(&mut model));
        assert!(model.parts.is_empty());
    }

    #[test]
    fn the_history_keeps_a_limited_number_of_meshes() {
        let mut model = model();
        for _ in 0..MESH_DEPTH + 3 {
            model.edit_mesh(|_, _| {});
            record(&mut model);
        }
        let meshes = model
            .history
            .undo
            .iter()
            .filter(|s| s.mesh.is_some())
            .count();
        assert!(meshes <= MESH_DEPTH);
    }
}
