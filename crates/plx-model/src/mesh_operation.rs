//! Mesh operations: changes of the mesh that are part of how it is made, listed under Mesh >
//! Mesh Operations. Each is applied to the parts it names whenever they are meshed anew, and
//! it can be edited or deleted later, which undoes it on the mesh there is.

use plx_mesh::{FaceRenumbering, FeMesh, MeshTransform};
use serde::{Deserialize, Serialize};

use crate::FeModel;
use crate::convert::Conversion;
use crate::units::Quantity;

/// A change of the mesh parts, applied in the order of the list after meshing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MeshOperation {
    pub name: String,
    /// The mesh parts it changes, by name.
    pub parts: Vec<String>,
    pub kind: MeshOperationKind,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum MeshOperationKind {
    /// Translate, rotate, mirror or scale the parts.
    Transform(MeshTransform),
}

impl MeshOperationKind {
    /// The name of the item type, also the prefix of new items' names.
    pub fn type_name(&self) -> &'static str {
        match self {
            MeshOperationKind::Transform(_) => "Transform",
        }
    }
}

impl MeshOperation {
    /// The indices of the parts of `mesh` it names; names without a part are left out.
    pub fn part_indices(&self, mesh: &FeMesh) -> Vec<usize> {
        (mesh.parts.iter().enumerate())
            .filter(|(_, p)| self.parts.contains(&p.name))
            .map(|(i, _)| i)
            .collect()
    }

    /// Applies the operation to the parts of `mesh` it names; returns where the faces of
    /// elements went that it turned inside out.
    pub fn apply(&self, mesh: &mut FeMesh) -> FaceRenumbering {
        let parts = self.part_indices(mesh);
        match &self.kind {
            MeshOperationKind::Transform(transform) => mesh.transform_parts(&parts, transform),
        }
    }

    /// Takes the operation back on the parts of `mesh` it names, see [`Self::apply`].
    pub fn undo(&self, mesh: &mut FeMesh) -> FaceRenumbering {
        let parts = self.part_indices(mesh);
        match &self.kind {
            MeshOperationKind::Transform(transform) => {
                mesh.transform_parts(&parts, &transform.inverse())
            }
        }
    }

    pub(crate) fn convert_units(&mut self, c: &Conversion) {
        let length = |v: &mut [f64; 3]| v.iter_mut().for_each(|x| c.value(x, Quantity::Length));
        match &mut self.kind {
            MeshOperationKind::Transform(transform) => match transform {
                MeshTransform::Translate(d) => length(d),
                MeshTransform::Rotate { point, .. } | MeshTransform::Mirror { point, .. } => {
                    length(point)
                }
                MeshTransform::Scale { center, .. } => length(center),
            },
        }
    }
}

impl FeModel {
    /// Applies the mesh operations to a mesh made anew, e.g. the meshes of parts Gmsh just
    /// made: each operation acts on those of its parts the mesh has.
    pub fn apply_mesh_operations(&self, mesh: &mut FeMesh) {
        for operation in &self.mesh_operations {
            operation.apply(mesh);
        }
    }

    /// Makes `operations` the model's mesh operations and changes `mesh`, which has the
    /// current ones applied, to match: the operations from the first that differs on are
    /// undone, last first, and the new ones applied. Faces of the model's regions follow
    /// elements turned inside out.
    pub fn set_mesh_operations(&mut self, mesh: &mut FeMesh, operations: Vec<MeshOperation>) {
        let same = (self.mesh_operations.iter().zip(&operations))
            .take_while(|(old, new)| old == new)
            .count();
        let old = std::mem::take(&mut self.mesh_operations);
        for operation in old[same..].iter().rev() {
            let faces = operation.undo(mesh);
            self.renumber_faces(&faces);
        }
        for operation in &operations[same..] {
            let faces = operation.apply(mesh);
            self.renumber_faces(&faces);
        }
        self.mesh_operations = operations;
    }

    /// Follows a renamed mesh part in the mesh operations.
    pub(crate) fn rename_mesh_operation_parts(&mut self, old: &str, new: &str) {
        for operation in &mut self.mesh_operations {
            for name in operation.parts.iter_mut().filter(|n| n.as_str() == old) {
                *name = new.to_string();
            }
            let mut seen = std::collections::BTreeSet::new();
            operation.parts.retain(|name| seen.insert(name.clone()));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_mesh::{Element, ElementShape, Part};

    /// Two single tetrahedra, parts A and B.
    fn two_parts() -> FeMesh {
        let mut mesh = FeMesh::default();
        let corners = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        for (offset, name) in [(0, "A"), (4, "B")] {
            for (i, c) in corners.iter().enumerate() {
                let shift = if offset == 0 { 0.0 } else { 5.0 };
                mesh.set_node(offset + i as u32 + 1, [c[0] + shift, c[1], c[2]]);
            }
            let id = offset / 4 + 1;
            mesh.add_element(Element {
                id,
                type_name: "C3D4".into(),
                shape: ElementShape::Tet4,
                nodes: (1..=4).map(|n| offset + n).collect(),
            })
            .unwrap();
            mesh.parts.push(Part {
                name: name.into(),
                elements: vec![id],
            });
        }
        mesh
    }

    fn transform(name: &str, parts: &[&str], transform: MeshTransform) -> MeshOperation {
        MeshOperation {
            name: name.into(),
            parts: parts.iter().map(|p| p.to_string()).collect(),
            kind: MeshOperationKind::Transform(transform),
        }
    }

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        (0..3).all(|k| (a[k] - b[k]).abs() < 1e-12)
    }

    #[test]
    fn editing_an_operation_undoes_the_later_ones_and_applies_them_again() {
        let original = two_parts();
        let mut mesh = original.clone();
        let mut fe = FeModel::default();
        let shift = transform(
            "Transform-1",
            &["A"],
            MeshTransform::Translate([0.0, 2.0, 0.0]),
        );
        let turn = transform(
            "Transform-2",
            &["A", "B"],
            MeshTransform::Rotate {
                point: [0.0; 3],
                axis: [0.0, 0.0, 1.0],
                angle: 90.0,
            },
        );
        fe.set_mesh_operations(&mut mesh, vec![shift.clone(), turn.clone()]);
        // Node 2 of A at (1, 0, 0): shifted to (1, 2, 0), turned to (-2, 1, 0).
        assert!(close(mesh.node(2).unwrap(), [-2.0, 1.0, 0.0]));
        // Node 5 of B at (5, 0, 0): turned only.
        assert!(close(mesh.node(5).unwrap(), [0.0, 5.0, 0.0]));

        // A mesh made anew gets the same.
        let mut fresh = original.clone();
        fe.apply_mesh_operations(&mut fresh);
        for id in 1..=8 {
            assert!(close(fresh.node(id).unwrap(), mesh.node(id).unwrap()));
        }

        // The shift edited: it is undone below the rotation, which is applied again.
        let longer = transform(
            "Transform-1",
            &["A"],
            MeshTransform::Translate([0.0, 3.0, 0.0]),
        );
        fe.set_mesh_operations(&mut mesh, vec![longer, turn.clone()]);
        assert!(close(mesh.node(2).unwrap(), [-3.0, 1.0, 0.0]));
        assert!(close(mesh.node(5).unwrap(), [0.0, 5.0, 0.0]));

        // Both deleted: the mesh is as it was meshed.
        fe.set_mesh_operations(&mut mesh, Vec::new());
        for id in 1..=8 {
            assert!(close(mesh.node(id).unwrap(), original.node(id).unwrap()));
        }
        assert!(fe.mesh_operations.is_empty());
    }

    #[test]
    fn a_deleted_mirror_restores_the_node_order() {
        let original = two_parts();
        let mut mesh = original.clone();
        let mut fe = FeModel::default();
        let mirror = transform(
            "Transform-1",
            &["B"],
            MeshTransform::Mirror {
                point: [0.0; 3],
                normal: [1.0, 0.0, 0.0],
            },
        );
        fe.set_mesh_operations(&mut mesh, vec![mirror]);
        assert!(close(mesh.node(5).unwrap(), [-5.0, 0.0, 0.0]));
        assert_ne!(
            mesh.element(2).unwrap().nodes,
            original.element(2).unwrap().nodes
        );
        fe.set_mesh_operations(&mut mesh, Vec::new());
        assert_eq!(
            mesh.element(2).unwrap().nodes,
            original.element(2).unwrap().nodes
        );
        assert!(close(mesh.node(5).unwrap(), [5.0, 0.0, 0.0]));
    }

    #[test]
    fn a_renamed_part_stays_in_the_operation() {
        let mut fe = FeModel::default();
        fe.mesh_operations.push(transform(
            "Transform-1",
            &["A", "B"],
            MeshTransform::Translate([1.0; 3]),
        ));
        fe.rename_part("A", "C");
        assert_eq!(fe.mesh_operations[0].parts, ["C", "B"]);
    }
}
