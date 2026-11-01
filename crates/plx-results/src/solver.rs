//! Results of the built-in solver `plxs`, taken from memory instead of a `.frd` file
//! ([`from_solver`]), and kept as a compact binary file (`.plxr`, postcard) in the work
//! directory and in projects.
//!
//! [`from_solver`] gives the same mesh, increments, fields and components as reading the
//! `.frd` file the solver would write (see `plx-io::frd`): the same element types and node
//! orders, parts by material, component names after [`crate::names`], and derived components.
//! Only the coordinates and values are exact rather than rounded to the file's five digits.

use std::collections::BTreeMap;
use std::path::Path;

use plx_mesh::{Element, FeMesh, Part};
use plxs_core::results::IncrementKind;
use plxs_core::{Model, Results};
use serde::{Deserialize, Serialize};

use crate::names::{element_kind, rename_component, reorder_nodes};
use crate::{AnalysisKind, Component, Field, Increment, add_derived_components};

/// The extension of a results file of the built-in solver.
pub const EXTENSION: &str = "plxr";

/// The first bytes of a `.plxr` file; the last two count the format, which is not converted:
/// results of another format are solved again.
const MAGIC: &[u8; 8] = b"PLXRES\x00\x01";

/// What a run of the built-in solver computed, as PrePoLix keeps it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SolverResults {
    /// The mesh the results are on: nodes, elements, sections and materials of the solver's
    /// output model, with shells and beams expanded as the `.frd` file shows them. Sets,
    /// constraints, contact and loads are left out.
    pub model: Model,
    pub results: Results,
    /// The version of the solver that computed them.
    pub version: String,
}

/// Why a `.plxr` file could not be read.
#[derive(Debug)]
pub struct ReadError(pub String);

impl std::fmt::Display for ReadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ReadError {}

impl SolverResults {
    /// The results on the mesh of `model`, the solver's output model, of which only what
    /// the results need is kept.
    pub fn new(model: &Model, results: Results, version: &str) -> Self {
        let mut kept = Model::default();
        kept.heading = model.heading.clone();
        kept.nodes = model.nodes.clone();
        kept.elements = model.elements.clone();
        kept.materials = model.materials.clone();
        kept.sections = model.sections.clone();
        kept.rebuild_indices();
        Self {
            model: kept,
            results,
            version: version.to_owned(),
        }
    }

    /// The results as the bytes of a `.plxr` file.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = MAGIC.to_vec();
        // Serializing plain data into memory does not fail.
        bytes.extend(postcard::to_stdvec(self).unwrap_or_default());
        bytes
    }

    /// Reads the bytes of a `.plxr` file.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, ReadError> {
        let data = bytes
            .strip_prefix(MAGIC.as_slice())
            .ok_or_else(|| ReadError("not results of this PrePoLix version: solve again".into()))?;
        let mut results: SolverResults = postcard::from_bytes(data)
            .map_err(|e| ReadError(format!("results not readable: {e}")))?;
        results.model.rebuild_indices();
        Ok(results)
    }

    pub fn write(&self, path: &Path) -> std::io::Result<()> {
        std::fs::write(path, self.to_bytes())
    }

    pub fn read(path: &Path) -> Result<Self, ReadError> {
        let bytes =
            std::fs::read(path).map_err(|e| ReadError(format!("{}: {e}", path.display())))?;
        Self::from_bytes(&bytes).map_err(|e| ReadError(format!("{}: {e}", path.display())))
    }
}

/// Whether a file is a results file of the built-in solver (`.plxr`).
pub fn is_solver_results(path: &Path) -> bool {
    path.extension()
        .is_some_and(|e| e.eq_ignore_ascii_case(EXTENSION))
}

/// The mesh and increments of results of the built-in solver, as reading its `.frd` file
/// gives them.
#[derive(Debug, Default)]
pub struct SolverImport {
    pub mesh: FeMesh,
    pub increments: Vec<Increment>,
    /// Material names by number, as the `.frd` file numbers them.
    pub materials: BTreeMap<i32, String>,
    pub warnings: Vec<String>,
}

/// The mesh and increments of `solved`, the same as reading the `.frd` file the solver would
/// write for them, without writing and parsing it.
pub fn from_solver(solved: &SolverResults) -> SolverImport {
    let model = &solved.model;
    let mut import = SolverImport::default();
    for (i, material) in model.materials.iter().enumerate() {
        import.materials.insert(i as i32 + 1, material.name.clone());
    }

    // Nodes that belong to elements, ascending by id, as the `.frd` file has them.
    for i in plxs_output::frd::written_nodes(model) {
        let (id, xyz) = (model.nodes.ids[i], model.nodes.coordinates[i]);
        if import.mesh.set_node(id, xyz) {
            import
                .warnings
                .push(format!("Node {id} is defined more than once"));
        }
    }

    // Elements ascending by id, grouped into parts by the material of their section.
    let mut order: Vec<usize> = (0..model.elements.len()).collect();
    order.sort_by_key(|&i| model.elements[i].id);
    let mut by_material: BTreeMap<i32, Vec<u32>> = BTreeMap::new();
    for i in order {
        let element = &model.elements[i];
        let Some((number, frd_order)) = plxs_output::frd::frd_element(element.kind) else {
            continue;
        };
        let Some((shape, type_name)) = element_kind(i32::from(number)) else {
            continue;
        };
        let frd_nodes: Vec<u32> = (frd_order.iter())
            .filter_map(|&k| element.nodes.get(k).copied())
            .collect();
        if frd_nodes.len() < shape.node_count() {
            import
                .warnings
                .push(format!("Element {} is incomplete", element.id));
            continue;
        }
        let nodes = reorder_nodes(shape, &frd_nodes[..shape.node_count()]);
        let material = (element.section)
            .and_then(|s| model.sections.get(s as usize))
            .and_then(|s| s.kind.material())
            .and_then(|name| model.materials.iter().position(|m| m.name == name))
            .map_or(1, |i| i as i32 + 1);
        let added = import.mesh.add_element(Element {
            id: element.id,
            type_name: type_name.to_string(),
            shape,
            nodes,
        });
        match added {
            Ok(()) => by_material.entry(material).or_default().push(element.id),
            Err(error) => import.warnings.push(error.to_string()),
        }
    }
    for (material, elements) in by_material {
        let name = (import.materials.get(&material).cloned())
            .unwrap_or_else(|| format!("MATERIAL-{material}"));
        let parts = &mut import.mesh.parts;
        match parts.iter_mut().find(|p| p.name == name) {
            Some(part) => part.elements.extend(elements),
            None => parts.push(Part { name, elements }),
        }
    }

    let mesh = &import.mesh;
    for (ordinal, written) in solved.results.increments.iter().enumerate() {
        let kind = match written.kind {
            IncrementKind::Time => AnalysisKind::Static,
            IncrementKind::Dynamic => AnalysisKind::Dynamic,
            IncrementKind::Frequency => AnalysisKind::Frequency,
            IncrementKind::Buckling => AnalysisKind::Buckling,
            IncrementKind::ComplexFrequency => AnalysisKind::ComplexFrequency,
        };
        // Time increments carry the total time, modes their frequency or load factor.
        let value = match written.kind {
            IncrementKind::Time | IncrementKind::Dynamic => written.total_time,
            _ => written.value,
        };
        // An increment counted 0 takes the number the `.frd` block header gives it.
        let increment = match (written.increment, written.kind) {
            (0, IncrementKind::Time | IncrementKind::Dynamic) => ordinal as u32 + 1,
            (number, _) => number,
        };
        let fields = written.fields.iter().map(|field| {
            let index: Vec<Option<usize>> =
                field.nodes.iter().map(|&id| mesh.node_index(id)).collect();
            let components = (field.components.iter().zip(&field.values))
                .map(|(name, column)| {
                    let mut values = vec![f32::NAN; mesh.node_count()];
                    for (at, value) in index.iter().zip(column) {
                        if let Some(at) = at {
                            values[*at] = *value as f32;
                        }
                    }
                    Component {
                        name: rename_component(name).to_string(),
                        values,
                        derived: false,
                    }
                })
                .collect();
            let mut field = Field::new(field.name.clone(), components);
            add_derived_components(&mut field);
            field
        });
        let target = (import.increments.iter_mut())
            .find(|i| i.step == written.step && i.increment == increment && i.kind == kind);
        match target {
            Some(target) => target.fields.extend(fields),
            None => import.increments.push(Increment {
                step: written.step,
                increment,
                kind,
                value,
                fields: fields.collect(),
                nodal_diameter: written.nodal_diameter,
            }),
        }
    }
    import
}
