//! Node sets, element sets and surfaces the user defines in the FE Model tree.
//! They keep the selection they were made from, so that they are found on a new
//! mesh again; the members are written into the mesh's sets, which the input file lists.

pub use plx_mesh::SetKind;
use plx_mesh::{FeMesh, SurfaceDefinition};
use serde::{Deserialize, Serialize};

use crate::{FeModel, Region};

/// A node set, element set or surface defined by a selection.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MeshSet {
    /// The internal identity others refer to; the user sees only the name.
    pub id: crate::Id,
    pub name: String,
    pub kind: SetKind,
    /// What the user picked: nodes, faces, CAD entities or whole parts.
    pub region: Region,
    /// Whether a Geometry link brought the set with the meshes: it changes where it comes
    /// from, and goes when the link brings the meshes again, unlike the analysis' own sets.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub linked: bool,
}

impl MeshSet {
    pub fn new(name: impl Into<String>, kind: SetKind, region: Region) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            kind,
            region,
            linked: false,
        }
    }

    /// Writes the members of the set into the mesh's sets of its kind. A selection that
    /// finds nothing on the mesh, e.g. picked nodes of an older mesh, leaves no set, so that
    /// items using it show their missing reference.
    pub fn write_into(&self, mesh: &mut FeMesh) {
        self.kind.remove(mesh, &self.name);
        mesh.user_sets.remove(&self.id);
        match self.kind {
            SetKind::Nodes => {
                let mut nodes = self.region.nodes(mesh);
                nodes.retain(|&n| mesh.node(n).is_some());
                if !nodes.is_empty() {
                    mesh.node_sets.insert(self.name.clone(), nodes);
                    self.mark(mesh);
                }
            }
            SetKind::Elements => {
                let mut elements = self.region.elements(mesh);
                elements.retain(|&e| mesh.element(e).is_some());
                if !elements.is_empty() {
                    mesh.element_sets.insert(self.name.clone(), elements);
                    self.mark(mesh);
                }
            }
            SetKind::Surface => {
                let mut faces = self.region.faces(mesh);
                faces.retain(|&(e, _)| mesh.element(e).is_some());
                if !faces.is_empty() {
                    let surface = SurfaceDefinition::ElementFaces(faces);
                    mesh.surfaces.insert(self.name.clone(), surface);
                    self.mark(mesh);
                }
            }
        }
    }
}

impl MeshSet {
    /// Notes in the mesh that its set of the name is this one, for regions on it by id.
    fn mark(&self, mesh: &mut FeMesh) {
        (mesh.user_sets).insert(self.id, (self.kind, self.name.clone()));
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
            mesh.user_sets.remove(&set.id);
        }
        self.write_mesh_sets(mesh);
    }

    /// Makes the regions on the mesh's set of the kind and name refer to the user-defined set
    /// of the id that takes its place, e.g. when a set of an input file is edited.
    pub fn adopt_set(&mut self, kind: SetKind, name: &str, id: crate::Id) {
        let old = Region::named(kind, name);
        let adopt = |region: &mut Region| {
            if *region == old {
                *region = Region::Set(id);
            }
        };
        self.regions_mut().for_each(adopt);
        (self.initial_conditions.iter_mut()).for_each(|i| adopt(&mut i.region));
    }

    /// Writes all user-defined sets into the mesh, e.g. into a newly generated one.
    pub fn write_mesh_sets(&self, mesh: &mut FeMesh) {
        for set in &self.mesh_sets {
            set.write_into(mesh);
        }
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
            id: Default::default(),
            bodies: Vec::new(),
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
                    Region::Parts(mesh.ids_of(&["Solid"])),
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

        // Renamed and changed: the old name is gone, regions on the set keep it by id.
        let before = fe.mesh_sets.clone();
        let all = Region::Set(fe.mesh_sets[1].id);
        fe.sections.push(crate::Section {
            id: crate::Id::new(),
            orientation: None,
            name: "Section-1".into(),
            material: None,
            region: all.clone(),
            thickness: 1.0,
            kind: crate::SectionKind::Solid,
        });
        fe.mesh_sets[1].name = "Everything".into();
        fe.mesh_sets[0].region = Region::Nodes(vec![1, 2]);
        fe.sync_mesh_sets(&before, &mut mesh);
        assert!(!mesh.element_sets.contains_key("All"));
        assert_eq!(mesh.element_sets["Everything"], vec![1]);
        assert_eq!(mesh.node_sets["Tip"], vec![1, 2]);
        assert_eq!(fe.sections[0].region.elements(&mesh), vec![1]);
        assert_eq!(all.describe(&fe, &mesh), "Everything");

        // A selection the mesh no longer has leaves no set.
        let before = fe.mesh_sets.clone();
        fe.mesh_sets[0].region = Region::Nodes(vec![99]);
        fe.sync_mesh_sets(&before, &mut mesh);
        assert!(!mesh.node_sets.contains_key("Tip"));
        let tip = Region::Set(fe.mesh_sets[0].id);
        assert!(tip.nodes(&mesh).is_empty());
        assert!(tip.missing_reference(&mesh).is_some());
    }
}
