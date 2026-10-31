//! The element table: for every element of the model its formulation, dense node indices,
//! global DOFs and material matrix, resolved once per model.

use std::collections::HashMap;

use plxs_core::section::SectionKind;
use plxs_core::state::IntegrationPointField;
use plxs_core::{Element, ElementFamily, ElementType, Error, Model, Result};
use plxs_elements::{SolidElement, plane, solid};
use plxs_materials::LinearElastic;

use rayon::prelude::*;

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
    pub dofs: DofMap,
    pub layout: IntegrationPointLayout,
    /// A material without `*ELASTIC`: allowed for pure heat transfer, where it gets zero
    /// elastic constants; [`Self::require_elastic`] refuses it for mechanical steps.
    pub missing_elastic: Option<String>,
}

impl ElementTable {
    /// Fails when a material lacks `*ELASTIC`: mechanical steps need it, heat transfer not.
    pub fn require_elastic(&self) -> Result<()> {
        match &self.missing_elastic {
            Some(name) => Err(Error::input(
                "*MATERIAL",
                format!("material {name} has no *ELASTIC definition"),
            )),
            None => Ok(()),
        }
    }

    /// Resolves the elements of a model at a uniform reference temperature. Elements the
    /// solver cannot compute yet are an error naming the type.
    pub fn new(model: &Model, temperature: f64) -> Result<Self> {
        let mut dofs = DofMap::new(model.nodes.len(), 3);
        // Formulations per type and, for plane elements, thickness.
        let mut formulation_index: HashMap<(ElementType, u64), u32> = HashMap::new();
        // Per node: whether some element uses its third DOF, whether it is internal.
        let mut uses_z = vec![false; model.nodes.len()];
        let mut connected = vec![false; model.nodes.len()];

        let mut formulations = Vec::new();
        let mut material_index: HashMap<u32, u32> = HashMap::new();
        let mut materials = Vec::new();
        let mut missing_elastic: Option<String> = None;
        let mut entries = Vec::with_capacity(model.elements.len());
        let mut offsets = Vec::with_capacity(model.elements.len() + 1);
        offsets.push(0u32);
        for element in &model.elements {
            let family = element.kind.family();
            let is_plane = matches!(
                family,
                ElementFamily::PlaneStress
                    | ElementFamily::PlaneStrain
                    | ElementFamily::Axisymmetric
            );
            if family != ElementFamily::Solid && !is_plane {
                return Err(Error::Unsupported(format!(
                    "element type {} is not implemented yet (element {})",
                    element.kind, element.id
                )));
            }
            let thickness = if is_plane {
                element
                    .section
                    .and_then(|s| match &model.sections.get(s as usize)?.kind {
                        SectionKind::Solid { thickness, .. } => *thickness,
                        _ => None,
                    })
            } else {
                None
            };
            let key = (element.kind, thickness.map_or(0, f64::to_bits));
            let formulation = match formulation_index.get(&key) {
                Some(&i) => i,
                None => {
                    let made = if is_plane {
                        plane(element.kind, thickness)
                    } else {
                        solid(element.kind)
                    };
                    let f = made.ok_or_else(|| {
                        Error::Unsupported(format!(
                            "element type {} is not implemented yet",
                            element.kind
                        ))
                    })?;
                    formulations.push(f);
                    let i = formulations.len() as u32 - 1;
                    formulation_index.insert(key, i);
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
                    let elastic = if mat.elastic.is_none() {
                        missing_elastic.get_or_insert_with(|| mat.name.clone());
                        LinearElastic {
                            young: 0.0,
                            poisson: 0.0,
                        }
                    } else {
                        LinearElastic::of(mat, temperature)?
                    };
                    materials.push((elastic, elastic.matrix()));
                    let i = materials.len() as u32 - 1;
                    material_index.insert(section_index, i);
                    (i, plxs_materials::density(mat, temperature))
                }
            };
            let nodes = Self::node_indices(model, element)?;
            let f = &formulations[formulation as usize];
            for &n in &nodes {
                connected[n as usize] = true;
                uses_z[n as usize] |= f.uses_out_of_plane();
            }
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
        // DOFs no element works on are held at zero: nodes outside every element, and the
        // out-of-plane DOF of nodes that only plane strain or axisymmetric elements use.
        for n in 0..model.nodes.len() {
            let base = n as u32 * 3;
            if !connected[n] {
                dofs.unused.extend([base, base + 1, base + 2]);
            } else if !uses_z[n] {
                dofs.unused.push(base + 2);
            }
        }
        Ok(Self {
            entries,
            formulations,
            materials,
            dofs,
            layout: IntegrationPointLayout { offsets },
            missing_elastic,
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

    /// Displacements and nodal forces as they are reported (`.frd`, `.dat`): what the
    /// solver computes, except at the nodes of plane and axisymmetric elements, which report
    /// the thickness average of their expanded nodes' displacements and the sum of their
    /// forces, as CalculiX does (see `plxs_elements::plane`). The internal out-of-plane
    /// unknown of plane stress nodes is no displacement and reported as zero.
    pub fn reported(&self, model: &Model, u: &[f64], forces: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let mut u_out = u.to_vec();
        let mut f_out = forces.to_vec();
        if self.formulations.iter().all(|f| f.expansion().is_none()) {
            return (u_out, f_out);
        }
        let reported: Vec<Option<plxs_elements::ReportedNodal>> = self
            .entries
            .par_iter()
            .map(|entry| {
                let f = self.formulation(entry);
                f.expansion()?;
                let coords = self.coordinates(model, entry);
                f.reported_nodal(&coords, self.material_matrix(entry), &self.gather(entry, u))
            })
            .collect();
        // Plane nodes: swap the element's share of the solver's forces for the reported one.
        for (entry, r) in self.entries.iter().zip(&reported) {
            let Some(r) = r else { continue };
            for a in 0..entry.nodes.len() {
                for k in 0..3 {
                    let dof = entry.dofs[3 * a + k] as usize;
                    u_out[dof] = r.displacements[a][k];
                    f_out[dof] += r.forces[a][k] - r.condensed[3 * a + k];
                }
            }
        }
        (u_out, f_out)
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
