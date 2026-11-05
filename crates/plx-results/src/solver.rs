//! Results of the built-in solver `plxs`, taken from memory instead of a `.frd` file
//! ([`from_solver`]), and kept as a compact binary file (`.ndlr`) in the work directory and
//! in projects: compressed in chunks after HDF5's layout, see [`plxs_output::packed`].
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
use plxs_output::packed::{self, Array, Packable};
use serde::{Deserialize, Serialize};

use crate::names::{element_kind, rename_component, reorder_nodes};
use crate::{AnalysisKind, Component, Field, Increment, add_derived_components};

/// The extension of a results file of the built-in solver.
pub const EXTENSION: &str = "ndlr";

/// The first bytes of a `.ndlr` file; the last two count the format, which is not converted:
/// results of another format are solved again.
const MAGIC: &[u8; 8] = b"PLXRES\x00\x04";

/// What a run of the built-in solver computed, as Nodalis keeps it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SolverResults {
    /// The mesh the results are on: nodes, elements, sections and materials of the solver's
    /// output model, with shells and beams expanded as the `.frd` file shows them. Sets,
    /// constraints, contact and loads are left out.
    pub model: Model,
    pub results: Results,
    /// The version of the solver that computed them.
    pub version: String,
    /// The steps of the FE model the solver's steps were lowered from.
    pub steps: Vec<crate::steps::StepRecord>,
    /// How the solver got through the increments it finished.
    pub convergence: Vec<crate::steps::IncrementRecord>,
    /// The solver step the run stopped in without converging.
    pub unconverged_step: Option<u32>,
}

/// Why a `.ndlr` file could not be read.
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
        // What the results are expanded to the sectors of a cyclic symmetric model with.
        kept.constraints.cyclic_symmetry = model.constraints.cyclic_symmetry.clone();
        kept.rebuild_indices();
        Self {
            model: kept,
            results,
            version: version.to_owned(),
            ..Self::default()
        }
    }

    /// The results as the bytes of a `.ndlr` file.
    pub fn to_bytes(&self) -> Vec<u8> {
        packed::pack(MAGIC, self.clone())
    }

    /// Reads the bytes of a `.ndlr` file.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, ReadError> {
        let mut results: SolverResults =
            packed::unpack(MAGIC, bytes).map_err(|e| ReadError(e.0))?;
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

/// The arrays of the mesh (node ids and coordinates) and of the results.
impl Packable for SolverResults {
    fn arrays(&mut self, visit: &mut dyn FnMut(Array<'_>)) {
        visit(Array::Ids(&mut self.model.nodes.ids));
        visit(Array::F64x3(&mut self.model.nodes.coordinates));
        self.results.arrays(visit);
    }
}

/// Whether a file is a results file of the built-in solver (`.ndlr`).
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
    // The modes of the eigenvalue steps join the increments as fields of their own.
    let with_modes = with_mode_increments(&solved.results);
    let results = with_modes.as_ref().unwrap_or(&solved.results);
    // A cyclic symmetric model on all the sectors the `.frd` file has.
    let expanded = plxs_output::frd::expand_sectors(&solved.model, results);
    let (model, results) = match &expanded {
        Some((model, results)) => (model, results),
        None => (&solved.model, results),
    };
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
    // The step, set and increment of the last buckling data set, see `buckling` below.
    let mut buckling: Option<(u32, usize, u32)> = None;
    // Eigenfrequency sensitivities of each step so far.
    let mut senfreq: BTreeMap<u32, u32> = BTreeMap::new();
    // The data set number of the `.frd` block header, which counts a mode of a cyclic
    // symmetric model twice (its real and imaginary solution, of which one is written).
    let mut set = 0u32;
    for (ordinal, written) in results.increments.iter().enumerate() {
        set += 1;
        // The `.frd` block header: time increments carry the total time, the others their
        // frequency, load factor or value; the data sets are counted from 1.
        let value = match written.kind {
            IncrementKind::Time | IncrementKind::Dynamic => written.total_time,
            _ => written.value,
        };
        let modal = matches!(
            written.kind,
            IncrementKind::Frequency | IncrementKind::ComplexFrequency
        );
        let nodal_diameter = written.nodal_diameter.filter(|_| modal);
        for field in &written.fields {
            // Numbered as the `.frd` reader numbers the blocks (`plx-io::frd`).
            let sensitivity = field.name == "NORM" || field.name.starts_with("SEN");
            let kind = match written.kind {
                IncrementKind::Time => AnalysisKind::Static,
                IncrementKind::Dynamic | IncrementKind::SteadyState => AnalysisKind::Dynamic,
                IncrementKind::Frequency => AnalysisKind::Frequency,
                IncrementKind::Buckling => AnalysisKind::Buckling,
                IncrementKind::ComplexFrequency => AnalysisKind::ComplexFrequency,
                IncrementKind::Sensitivity if sensitivity => AnalysisKind::Sensitivity,
                IncrementKind::Sensitivity => AnalysisKind::Other(3),
            };
            let header_increment = match written.kind {
                IncrementKind::SteadyState | IncrementKind::ComplexFrequency => 0,
                _ => written.increment,
            };
            let increment = match kind {
                // Each nodal diameter counts its modes anew; cosine and sine part come in
                // pairs, of which the first is kept.
                AnalysisKind::Frequency if nodal_diameter.is_some() => set.div_ceil(2),
                AnalysisKind::Frequency | AnalysisKind::ComplexFrequency => written.increment,
                // The static solution of a buckle step (value 0) is increment 0, the modes
                // count from 1 by data set.
                AnalysisKind::Buckling => {
                    let number = match buckling {
                        Some((step, at, number)) if step == written.step && at == ordinal => number,
                        Some((step, _, number)) if step == written.step => number + 1,
                        _ => u32::from(value != 0.0),
                    };
                    buckling = Some((written.step, ordinal, number));
                    number
                }
                AnalysisKind::Sensitivity if field.name == "SENFREQ" => {
                    let count = senfreq.entry(written.step).or_default();
                    *count += 1;
                    *count
                }
                // Steady state points are told apart by the data set number only.
                _ if header_increment == 0 => set,
                _ => header_increment,
            };
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
            // The imaginary part of a cyclic symmetric mode only turned it into the other
            // sectors.
            if field.name == MODE_IMAGINARY {
                continue;
            }
            let mut converted = Field::new(field.name.clone(), components);
            add_derived_components(&mut converted);
            let target = (import.increments.iter_mut())
                .find(|i| i.step == written.step && i.increment == increment && i.kind == kind);
            match target {
                Some(target) => {
                    if target.value == 0.0 {
                        target.value = value;
                    }
                    target.fields.push(converted);
                }
                None => import.increments.push(Increment {
                    step: written.step,
                    increment,
                    kind,
                    value,
                    fields: vec![converted],
                    nodal_diameter,
                }),
            }
        }
        if written.nodal_diameter.is_some() {
            set += 1;
        }
    }
    import
}

/// The field of a mode shape of an eigenvalue step, normalized to a largest displacement of
/// 1, see [`plxs_core::results::ModeSet`].
pub const MODE: &str = "MODE";
/// Magnitudes and phase angles of a complex mode (`MAG1`..., `PHA1`... in degrees), which
/// make it whirl: `u_k = MAG_k cos(ωt + PHA_k)`.
pub const MODE_PHASES: &str = "PMODE";
/// The imaginary part of a mode shape, while the sectors of a cyclic symmetric model are
/// made of it.
const MODE_IMAGINARY: &str = "MODEI";

/// The results with the modes of their eigenvalue steps as increments of one field each
/// ([`MODE`], with [`MODE_PHASES`] for complex modes and [`MODE_IMAGINARY`] for cyclic
/// symmetric ones), after the increments of the steps before; `None` without modes.
fn with_mode_increments(results: &Results) -> Option<Results> {
    use plxs_core::results::{ModeKind, ResultField, ResultsIncrement};
    if results
        .modes
        .iter()
        .all(|set| (results.increments.iter()).any(|i| i.step == set.step))
    {
        return None;
    }
    let mut out = Results {
        increments: Vec::new(),
        history: Vec::new(),
        modes: Vec::new(),
    };
    let mut modes: Vec<ResultsIncrement> = Vec::new();
    // A step that asked for fields (an input file run by the solver alone) shows them as
    // CalculiX writes them.
    let written = |step: u32| results.increments.iter().any(|i| i.step == step);
    for set in results.modes.iter().filter(|set| !written(set.step)) {
        let field = |name: &str, components: &[&str], values: Vec<Vec<f64>>| ResultField {
            name: name.into(),
            components: components.iter().map(|c| c.to_string()).collect(),
            nodes: set.nodes.clone(),
            values,
        };
        let columns = |rows: &[[f32; 3]]| -> Vec<Vec<f64>> {
            (0..3)
                .map(|k| rows.iter().map(|r| f64::from(r[k])).collect())
                .collect()
        };
        for mode in &set.modes {
            let mut fields = vec![field(MODE, &["D1", "D2", "D3"], columns(&mode.shape))];
            match (&mode.imaginary, set.kind) {
                (Some(im), ModeKind::ComplexFrequency) => {
                    let mut values: Vec<Vec<f64>> =
                        (0..6).map(|_| Vec::with_capacity(im.len())).collect();
                    for (re, im) in mode.shape.iter().zip(im) {
                        for k in 0..3 {
                            let (a, b) = (f64::from(re[k]), f64::from(im[k]));
                            values[k].push(a.hypot(b));
                            // Round-off magnitudes get the phase 0.
                            let phase = if a.hypot(b) <= 1e-6 {
                                0.0
                            } else {
                                b.atan2(a).to_degrees()
                            };
                            values[k + 3].push(phase);
                        }
                    }
                    let names = ["MAG1", "MAG2", "MAG3", "PHA1", "PHA2", "PHA3"];
                    fields.push(field(MODE_PHASES, &names, values));
                }
                (Some(im), _) => {
                    fields.push(field(MODE_IMAGINARY, &["D1", "D2", "D3"], columns(im)));
                }
                (None, _) => {}
            }
            modes.push(ResultsIncrement {
                step: set.step,
                increment: mode.number,
                kind: match set.kind {
                    ModeKind::Frequency => IncrementKind::Frequency,
                    ModeKind::Buckling => IncrementKind::Buckling,
                    ModeKind::ComplexFrequency => IncrementKind::ComplexFrequency,
                },
                value: mode.value,
                total_time: 0.0,
                nodal_diameter: mode.nodal_diameter,
                fields,
            });
        }
    }
    // The modes of a step follow its other increments (a buckle step's reference state).
    let mut written = results.increments.iter().cloned().peekable();
    for mode in modes {
        while let Some(increment) = written.next_if(|i| i.step <= mode.step) {
            out.increments.push(increment);
        }
        out.increments.push(mode);
    }
    out.increments.extend(written);
    Some(out)
}
