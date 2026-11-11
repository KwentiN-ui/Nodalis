//! Element types the user chose for the parts of the mesh, per
//! part: another CalculiX formulation of the same shape, such as `C3D8R` instead of `C3D8`.

use plx_mesh::{ElementFamily, ElementShape, FeMesh};
use serde::{Deserialize, Serialize};

use crate::{FeModel, ModelSpace};

/// The element type the user chose for the elements of one shape in a part. It is kept by the part's name, so a part meshed anew
/// gets it again.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PartElementType {
    pub part: String,
    /// The CalculiX type; its shape tells which elements of the part it is for. Plane
    /// elements follow the model space: `CPE4R` is written as `CAX4R` in an axisymmetric
    /// model.
    pub type_name: String,
}

impl PartElementType {
    fn shape(&self) -> Option<ElementShape> {
        ElementShape::from_type_name(&self.type_name)
    }
}

/// The CalculiX types an element of the shape can have in the model space:
/// the same nodes in another formulation. Line elements get their type from their beam
/// or truss section and have none to choose from. Linear surface elements of a 3D model can
/// be shells or membranes; a 2D model has no membranes. CalculiX 2.23 computes quadratic
/// membranes wrongly (M3D8 is 1.7 times too soft in a tension test, M3D8R has hourglass
/// modes, M3D6 is off by a factor of 5), so they are not offered.
pub fn element_type_options(shape: ElementShape, space: ModelSpace) -> Vec<String> {
    use ElementShape::*;
    let names: &[&str] = match shape {
        Line2 | Line3 => &[],
        Tri3 => &["S3", "M3D3"],
        Tri6 => &["S6"],
        Quad4 => &["S4", "S4R", "M3D4", "M3D4R"],
        Quad8 => &["S8", "S8R"],
        Tet4 => &["C3D4"],
        Tet10 => &["C3D10", "C3D10T"],
        Wedge6 => &["C3D6"],
        Wedge15 => &["C3D15"],
        Hex8 => &["C3D8", "C3D8R", "C3D8I"],
        Hex20 => &["C3D20", "C3D20R"],
    };
    // Shells become the plane elements of a 2D model space.
    (names.iter())
        .filter(|name| !space.is_2d() || !name.starts_with("M3D"))
        .map(|name| space.element_type(name, shape))
        .collect()
}

impl FeModel {
    /// The element type chosen for the elements of the shape in the part, in the model's
    /// space.
    pub fn element_type(&self, part: &str, shape: ElementShape) -> Option<String> {
        (self.element_types.iter())
            .find(|c| c.part == part && c.shape() == Some(shape))
            .map(|c| self.properties.space.element_type(&c.type_name, shape))
    }

    /// Chooses the element type of the elements of its shape in the part; false for a name
    /// the solver has no shape for.
    pub fn set_element_type(&mut self, part: &str, type_name: &str) -> bool {
        let type_name = type_name.trim().to_ascii_uppercase();
        let Some(shape) = ElementShape::from_type_name(&type_name) else {
            return false;
        };
        if shape.family() == ElementFamily::Line {
            return false;
        }
        self.element_types
            .retain(|c| c.part != part || c.shape() != Some(shape));
        self.element_types.push(PartElementType {
            part: part.to_string(),
            type_name,
        });
        true
    }

    /// Gives the elements of the mesh's parts the types chosen for them; true when any
    /// changed. Parts without a choice keep the types they have.
    pub fn apply_element_types(&self, mesh: &mut FeMesh) -> bool {
        if self.element_types.is_empty() {
            return false;
        }
        let mut changes = Vec::new();
        for part in &mesh.parts {
            let choices: Vec<(ElementShape, String)> = (self.element_types.iter())
                .filter(|c| c.part == part.name)
                .filter_map(|c| {
                    let shape = c.shape()?;
                    Some((
                        shape,
                        self.properties.space.element_type(&c.type_name, shape),
                    ))
                })
                .collect();
            if choices.is_empty() {
                continue;
            }
            for element in part.elements.iter().filter_map(|&id| mesh.element(id)) {
                let chosen = choices.iter().find(|(shape, _)| *shape == element.shape);
                if let Some((_, type_name)) = chosen
                    && !type_name.eq_ignore_ascii_case(&element.type_name)
                {
                    changes.push((element.id, type_name.clone()));
                }
            }
        }
        for (id, type_name) in &changes {
            mesh.set_element_type(*id, type_name.clone());
        }
        !changes.is_empty()
    }

    /// Writes the linear shells of membrane sections as membranes, see
    /// [`crate::membrane_type`]; true when any changed. The user need not choose an M3D
    /// element type for a part with a membrane section.
    pub fn apply_membrane_sections(&self, mesh: &mut FeMesh) -> bool {
        let mut changes = Vec::new();
        for section in &self.sections {
            if !matches!(section.kind, crate::SectionKind::Membrane { .. }) {
                continue;
            }
            for element in
                (section.region.elements(mesh).into_iter()).filter_map(|id| mesh.element(id))
            {
                if let Some(type_name) = crate::membrane_type(&element.type_name) {
                    changes.push((element.id, type_name.to_string()));
                }
            }
        }
        for (id, type_name) in &changes {
            mesh.set_element_type(*id, type_name.clone());
        }
        !changes.is_empty()
    }

    /// Follows a renamed part with the element types chosen for it. Merged parts keep the
    /// choices of the part merged into.
    pub(crate) fn rename_element_types(&mut self, old: &str, new: &str) {
        let kept: Vec<Option<ElementShape>> = (self.element_types.iter())
            .filter(|c| c.part == new)
            .map(PartElementType::shape)
            .collect();
        self.element_types
            .retain(|c| c.part != old || !kept.contains(&c.shape()));
        for choice in self.element_types.iter_mut().filter(|c| c.part == old) {
            choice.part = new.to_string();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_mesh::{Element, Part};

    fn mesh() -> FeMesh {
        let mut mesh = FeMesh::default();
        for id in 1..=8 {
            mesh.set_node(id, [f64::from(id), 0.0, 0.0]);
        }
        let quad = |id, type_name: &str| Element {
            id,
            type_name: type_name.into(),
            shape: ElementShape::Quad4,
            nodes: vec![1, 2, 3, 4],
        };
        mesh.add_element(quad(1, "CPS4")).unwrap();
        mesh.add_element(quad(2, "CPS4")).unwrap();
        mesh.add_element(Element {
            id: 3,
            type_name: "CPS3".into(),
            shape: ElementShape::Tri3,
            nodes: vec![5, 6, 7],
        })
        .unwrap();
        mesh.parts = vec![
            Part {
                bodies: Vec::new(),
                name: "A".into(),
                elements: vec![1, 3],
            },
            Part {
                bodies: Vec::new(),
                name: "B".into(),
                elements: vec![2],
            },
        ];
        mesh
    }

    #[test]
    fn options_have_the_same_nodes_and_follow_the_model_space() {
        use ElementShape::*;
        assert_eq!(
            element_type_options(Hex8, ModelSpace::ThreeD),
            ["C3D8", "C3D8R", "C3D8I"]
        );
        assert_eq!(
            element_type_options(Tet10, ModelSpace::PlaneStrain),
            ["C3D10", "C3D10T"]
        );
        assert_eq!(
            element_type_options(Quad8, ModelSpace::Axisymmetric),
            ["CAX8", "CAX8R"]
        );
        assert_eq!(
            element_type_options(Tri6, ModelSpace::PlaneStrain),
            ["CPE6"]
        );
        assert_eq!(
            element_type_options(Quad4, ModelSpace::ThreeD),
            ["S4", "S4R", "M3D4", "M3D4R"]
        );
        assert!(element_type_options(Line3, ModelSpace::ThreeD).is_empty());
        for space in ModelSpace::ALL {
            for shape in [
                Tri3, Tri6, Quad4, Quad8, Tet4, Tet10, Wedge6, Wedge15, Hex8, Hex20,
            ] {
                for name in element_type_options(shape, space) {
                    assert_eq!(ElementShape::from_type_name(&name), Some(shape), "{name}");
                }
            }
        }
    }

    #[test]
    fn chosen_types_apply_to_their_part_and_shape_only() {
        let mut model = FeModel::default();
        model.properties.space = ModelSpace::PlaneStress;
        assert!(model.set_element_type("A", "cps4r"));
        assert!(!model.set_element_type("A", "XYZ"));
        assert!(!model.set_element_type("A", "B32"));
        let mut mesh = mesh();
        assert!(model.apply_element_types(&mut mesh));
        let types: Vec<&str> = mesh
            .elements()
            .iter()
            .map(|e| e.type_name.as_str())
            .collect();
        assert_eq!(types, ["CPS4R", "CPS4", "CPS3"]);
        assert!(!model.apply_element_types(&mut mesh));

        // A new choice for the shape replaces the old one.
        model.set_element_type("A", "CPS4");
        assert_eq!(model.element_types.len(), 1);

        // The choice follows the model space, keeping the reduced integration.
        model.set_element_type("A", "CPS4R");
        model.properties.space = ModelSpace::Axisymmetric;
        assert_eq!(
            model.element_type("A", ElementShape::Quad4).as_deref(),
            Some("CAX4R")
        );
        model.apply_element_types(&mut mesh);
        assert_eq!(mesh.elements()[0].type_name, "CAX4R");
    }

    #[test]
    fn choices_follow_renamed_and_merged_parts() {
        let mut model = FeModel::default();
        model.set_element_type("A", "C3D8R");
        model.set_element_type("A", "C3D20R");
        model.set_element_type("B", "C3D8I");
        model.rename_part("A", "C");
        assert!(model.element_types.iter().all(|c| c.part != "A"));
        // B merged into C: C keeps its C3D8R.
        model.rename_part("B", "C");
        assert_eq!(
            model.element_type("C", ElementShape::Hex8).as_deref(),
            Some("C3D8R")
        );
        assert_eq!(model.element_types.len(), 2);
        let saved = ron::to_string(&model).unwrap();
        assert_eq!(ron::from_str::<FeModel>(&saved).unwrap(), model);
    }
}
