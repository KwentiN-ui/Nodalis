//! The element table: for every element of the model its formulation, dense node indices,
//! global DOFs and material matrix, resolved once per model.

use std::collections::HashMap;

use plxs_core::section::SectionKind;
use plxs_core::state::IntegrationPointField;
use plxs_core::{Element, ElementFamily, ElementType, Error, Model, Result};
use plxs_elements::{SolidElement, solid};
use plxs_materials::{LinearElastic, MaterialLaw, law_of};

use crate::DofMap;

/// One element of the table.
#[derive(Clone, Debug)]
pub struct ElementEntry {
    /// Index into [`ElementTable::formulations`].
    pub formulation: u32,
    /// Index into [`ElementTable::materials`].
    pub material: u32,
    /// Dense node indices in element order.
    pub nodes: Vec<u32>,
    /// Global DOFs in element order (3 per node).
    pub dofs: Vec<u32>,
    /// Density, when the material has one.
    pub density: Option<f64>,
}

/// Where the integration points of every element sit in a flat per-point array.
#[derive(Clone, Debug, Default)]
pub struct IntegrationPointLayout {
    /// `offsets[e]..offsets[e+1]` are the points of element `e`.
    pub offsets: Vec<u32>,
}

impl IntegrationPointLayout {
    pub fn total(&self) -> usize {
        self.offsets.last().copied().unwrap_or(0) as usize
    }

    pub fn field(&self, per_point: usize) -> IntegrationPointField {
        IntegrationPointField {
            per_point,
            offsets: self.offsets.clone(),
            values: vec![0.0; self.total() * per_point],
        }
    }
}

/// The resolved elements of a model.
pub struct ElementTable {
    pub entries: Vec<ElementEntry>,
    pub formulations: Vec<SolidElement>,
    /// Elastic constants and matrix per (section, temperature) combination.
    pub materials: Vec<(LinearElastic, [[f64; 6]; 6])>,
    /// The material law of every entry of `materials` (elastic, plastic, creeping), for
    /// small strains and for NLGEOM steps (finite strains).
    pub laws: Vec<Box<dyn MaterialLaw>>,
    pub laws_finite: Vec<Box<dyn MaterialLaw>>,
    pub dofs: DofMap,
    pub layout: IntegrationPointLayout,
}

impl ElementTable {
    /// Resolves the elements of a model at a uniform reference temperature. Elements the
    /// solver cannot compute yet are an error naming the type.
    pub fn new(model: &Model, temperature: f64) -> Result<Self> {
        let dofs = DofMap::new(model.nodes.len(), 3);
        let mut formulation_index: HashMap<ElementType, u32> = HashMap::new();
        let mut formulations = Vec::new();
        let mut material_index: HashMap<u32, u32> = HashMap::new();
        let mut materials = Vec::new();
        let mut laws: Vec<Box<dyn MaterialLaw>> = Vec::new();
        let mut laws_finite: Vec<Box<dyn MaterialLaw>> = Vec::new();
        let mut entries = Vec::with_capacity(model.elements.len());
        let mut offsets = Vec::with_capacity(model.elements.len() + 1);
        offsets.push(0u32);
        for element in &model.elements {
            if element.kind.family() != ElementFamily::Solid {
                return Err(Error::Unsupported(format!(
                    "element type {} is not implemented yet (element {})",
                    element.kind, element.id
                )));
            }
            let formulation = match formulation_index.get(&element.kind) {
                Some(&i) => i,
                None => {
                    let f = solid(element.kind).ok_or_else(|| {
                        Error::Unsupported(format!(
                            "element type {} is not implemented yet",
                            element.kind
                        ))
                    })?;
                    formulations.push(f);
                    let i = formulations.len() as u32 - 1;
                    formulation_index.insert(element.kind, i);
                    i
                }
            };
            let section_index = element.section.ok_or_else(|| {
                Error::input("*ELEMENT", format!("element {} has no section", element.id))
            })?;
            let (material, density) = match material_index.get(&section_index) {
                Some(&i) => (i, section_density(model, section_index)),
                None => {
                    let section = &model.sections[section_index as usize];
                    let name = match &section.kind {
                        SectionKind::Solid { material, .. } => material.as_str(),
                        _ => {
                            return Err(Error::input(
                                "*SOLID SECTION",
                                format!("element {} needs a solid section", element.id),
                            ));
                        }
                    };
                    let mat = model
                        .material(name)
                        .ok_or_else(|| Error::undefined("*SOLID SECTION", "material", name))?;
                    let elastic = LinearElastic::of(mat, temperature)?;
                    materials.push((elastic, elastic.matrix()));
                    laws.push(law_of(mat, temperature, false)?);
                    laws_finite.push(law_of(mat, temperature, true)?);
                    let i = materials.len() as u32 - 1;
                    material_index.insert(section_index, i);
                    (i, plxs_materials::density(mat, temperature))
                }
            };
            let nodes = Self::node_indices(model, element)?;
            let dof_list = nodes
                .iter()
                .flat_map(|&n| (0..3).map(move |d| n * 3 + d as u32))
                .collect();
            let points = formulations[formulation as usize].integration_points() as u32;
            offsets.push(offsets.last().unwrap() + points);
            entries.push(ElementEntry {
                formulation,
                material,
                nodes,
                dofs: dof_list,
                density,
            });
        }
        Ok(Self {
            entries,
            formulations,
            materials,
            laws,
            laws_finite,
            dofs,
            layout: IntegrationPointLayout { offsets },
        })
    }

    fn node_indices(model: &Model, element: &Element) -> Result<Vec<u32>> {
        element
            .nodes
            .iter()
            .map(|&id| {
                model.nodes.index(id).ok_or_else(|| {
                    Error::undefined("*ELEMENT", "node", format!("{id} (element {})", element.id))
                })
            })
            .collect()
    }

    pub fn formulation(&self, entry: &ElementEntry) -> &SolidElement {
        &self.formulations[entry.formulation as usize]
    }

    pub fn material_matrix(&self, entry: &ElementEntry) -> &[[f64; 6]; 6] {
        &self.materials[entry.material as usize].1
    }

    /// The material law of an element, in its finite-strain form for NLGEOM steps.
    pub fn law(&self, entry: &ElementEntry, finite_strain: bool) -> &dyn MaterialLaw {
        if finite_strain {
            self.laws_finite[entry.material as usize].as_ref()
        } else {
            self.laws[entry.material as usize].as_ref()
        }
    }

    /// Whether any element has a plastic or creeping material (the step then iterates even
    /// without NLGEOM, as CalculiX does).
    pub fn has_nonlinear_material(&self) -> bool {
        self.laws.iter().any(|l| l.is_nonlinear())
    }

    /// Coordinates of the element's nodes.
    pub fn coordinates(&self, model: &Model, entry: &ElementEntry) -> Vec<[f64; 3]> {
        entry
            .nodes
            .iter()
            .map(|&n| model.nodes.coordinates[n as usize])
            .collect()
    }

    /// Element displacements gathered from the global vector.
    pub fn gather(&self, entry: &ElementEntry, u: &[f64]) -> Vec<f64> {
        entry.dofs.iter().map(|&d| u[d as usize]).collect()
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// The density of the material of a section, if it has one.
fn section_density(model: &Model, section_index: u32) -> Option<f64> {
    let section = &model.sections[section_index as usize];
    let name = section.kind.material()?;
    model
        .material(name)
        .and_then(|m| plxs_materials::density(m, 0.0))
}
