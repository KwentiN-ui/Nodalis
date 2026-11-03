//! Node sets, element sets and surfaces the user defines in the FE Model tree.
//! They keep the selection they were made from, so that they are found on a new
//! mesh again; the members are written into the mesh's sets, which the input file lists.

use plx_mesh::{FeMesh, SurfaceDefinition};
use serde::{Deserialize, Serialize};

use crate::{FeModel, Region};

/// What a user-defined set holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SetKind {
    Nodes,
    Elements,
    /// Element faces, in 2D models the edges of the plane elements.
    Surface,
}

impl SetKind {
    pub fn label(self) -> &'static str {
        match self {
            SetKind::Nodes => "Node Set",
            SetKind::Elements => "Element Set",
            SetKind::Surface => "Surface",
        }
    }

    /// The default name prefix.
    pub fn prefix(self) -> &'static str {
        match self {
            SetKind::Nodes => "Node_Set",
            SetKind::Elements => "Element_Set",
            SetKind::Surface => "Surface",
        }
    }

    /// Names of the mesh's sets of this kind.
    pub fn names(self, mesh: &FeMesh) -> Vec<&str> {
        match self {
            SetKind::Nodes => mesh.node_sets.keys().map(String::as_str).collect(),
            SetKind::Elements => mesh.element_sets.keys().map(String::as_str).collect(),
            SetKind::Surface => mesh.surfaces.keys().map(String::as_str).collect(),
        }
    }

    /// The region that refers to the set of this kind by name.
    pub fn reference(self, name: &str) -> Region {
        match self {
            SetKind::Nodes => Region::NodeSet(name.to_owned()),
            SetKind::Elements => Region::ElementSet(name.to_owned()),
            SetKind::Surface => Region::Surface(name.to_owned()),
        }
    }

    /// Takes the set of the name out of the mesh.
    pub fn remove(self, mesh: &mut FeMesh, name: &str) -> bool {
        match self {
            SetKind::Nodes => mesh.node_sets.remove(name).is_some(),
            SetKind::Elements => mesh.element_sets.remove(name).is_some(),
            SetKind::Surface => mesh.surfaces.remove(name).is_some(),
        }
    }
}

/// A node set, element set or surface defined by a selection.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MeshSet {
    pub name: String,
    pub kind: SetKind,
    /// What the user picked: nodes, faces, CAD entities or whole parts.
    pub region: Region,
}

impl MeshSet {
    pub fn new(name: impl Into<String>, kind: SetKind, region: Region) -> Self {
        Self {
            name: name.into(),
            kind,
            region,
        }
    }

    /// Writes the members of the set into the mesh's sets of its kind. A selection that
    /// finds nothing on the mesh, e.g. picked nodes of an older mesh, leaves no set, so that
    /// items using it show their missing reference.
    pub fn write_into(&self, mesh: &mut FeMesh) {
        self.kind.remove(mesh, &self.name);
        match self.kind {
            SetKind::Nodes => {
                let mut nodes = self.region.nodes(mesh);
                nodes.retain(|&n| mesh.node(n).is_some());
                if !nodes.is_empty() {
                    mesh.node_sets.insert(self.name.clone(), nodes);
                }
            }
            SetKind::Elements => {
                let mut elements = self.region.elements(mesh);
                elements.retain(|&e| mesh.element(e).is_some());
                if !elements.is_empty() {
                    mesh.element_sets.insert(self.name.clone(), elements);
                }
            }
            SetKind::Surface => {
                let mut faces = self.region.faces(mesh);
                faces.retain(|&(e, _)| mesh.element(e).is_some());
                if !faces.is_empty() {
                    let surface = SurfaceDefinition::ElementFaces(faces);
                    mesh.surfaces.insert(self.name.clone(), surface);
                }
            }
        }
    }
}

impl FeModel {
    /// The user-defined set of the kind and name, if there is one.
    pub fn mesh_set(&self, kind: SetKind, name: &str) -> Option<&MeshSet> {
        (self.mesh_sets.iter()).find(|s| s.kind == kind && s.name == name)
    }

    /// Brings the mesh's sets up to date with the user-defined ones after they changed:
    /// `before` are the user-defined sets as they were, whose old members are removed.
    pub fn sync_mesh_sets(&self, before: &[MeshSet], mesh: &mut FeMesh) {
        for set in before {
            set.kind.remove(mesh, &set.name);
        }
        self.write_mesh_sets(mesh);
    }

    /// Writes all user-defined sets into the mesh, e.g. into a newly generated one.
    pub fn write_mesh_sets(&self, mesh: &mut FeMesh) {
        for set in &self.mesh_sets {
            set.write_into(mesh);
        }
    }

    /// Follows a renamed set: regions on it keep referring to it.
    pub fn rename_set(&mut self, kind: SetKind, old: &str, new: &str) {
        let old_reference = kind.reference(old);
        let new_reference = kind.reference(new);
        let follow = |region: &mut Region| {
            if *region == old_reference {
                *region = new_reference.clone();
            }
        };
        self.regions_mut().for_each(follow);
        (self.initial_conditions.iter_mut()).for_each(|i| follow(&mut i.region));
    }
}

#[cfg(test)]
mod tests {
    use plx_mesh::{Element, ElementShape, Part};

    use super::*;

    fn mesh() -> FeMesh {
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
            name: "Solid".into(),
            elements: vec![1],
        });
        mesh
    }

    #[test]
    fn sets_are_written_into_the_mesh_and_follow_changes() {
        let mut mesh = mesh();
        let mut fe = FeModel {
            mesh_sets: vec![
                MeshSet::new("Tip", SetKind::Nodes, Region::Nodes(vec![4])),
                MeshSet::new(
                    "All",
                    SetKind::Elements,
                    Region::Parts(vec!["Solid".into()]),
                ),
                MeshSet::new("Bottom", SetKind::Surface, Region::Faces(vec![(1, 1)])),
            ],
            ..Default::default()
        };
        fe.write_mesh_sets(&mut mesh);
        assert_eq!(mesh.node_sets["Tip"], vec![4]);
        assert_eq!(mesh.element_sets["All"], vec![1]);
        assert_eq!(
            mesh.surfaces["Bottom"],
            SurfaceDefinition::ElementFaces(vec![(1, 1)])
        );

        // Renamed and changed: the old name is gone, regions follow.
        let before = fe.mesh_sets.clone();
        fe.sections.push(crate::Section {
            name: "Section-1".into(),
            material: "Steel".into(),
            region: Region::ElementSet("All".into()),
            thickness: 1.0,
            kind: crate::SectionKind::Solid,
        });
        fe.mesh_sets[1].name = "Everything".into();
        fe.rename_set(SetKind::Elements, "All", "Everything");
        fe.mesh_sets[0].region = Region::Nodes(vec![1, 2]);
        fe.sync_mesh_sets(&before, &mut mesh);
        assert!(!mesh.element_sets.contains_key("All"));
        assert_eq!(mesh.element_sets["Everything"], vec![1]);
        assert_eq!(mesh.node_sets["Tip"], vec![1, 2]);
        assert_eq!(
            fe.sections[0].region,
            Region::ElementSet("Everything".into())
        );

        // A selection the mesh no longer has leaves no set.
        let before = fe.mesh_sets.clone();
        fe.mesh_sets[0].region = Region::Nodes(vec![99]);
        fe.sync_mesh_sets(&before, &mut mesh);
        assert!(!mesh.node_sets.contains_key("Tip"));
    }
}
