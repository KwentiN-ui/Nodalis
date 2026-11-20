//! Results of the built-in solver `plxs`, taken from memory instead of a `.frd` file
//! ([`from_solver`]), and kept as a compact binary file (`.ndlr`) in the work directory and
//! in projects: compressed in chunks after HDF5's layout, see [`plxs_output::packed`].
//!
//! [`from_solver`] gives the same mesh, increments, fields and components as reading the
//! `.frd` file the solver would write (see `plx-io::frd`): the same element types and node
//! orders, parts by material, component names after [`crate::names`], and derived components.
//! Only the coordinates and values are exact rather than rounded to the file's five digits.

use std::collections::{BTreeMap, HashMap};
use std::io::Write;
use std::path::Path;
use std::sync::Arc;

use plx_mesh::{Element, FeMesh, Part};
use plxs_core::results::{
    IncrementKind, ModeKind, ModeShape, ResultField, ResultsIncrement, plane,
};
use plxs_core::{Model, Results};
use plxs_output::frd::imaginary_field_name;
use plxs_output::packed::{self, Array, Packable};
use serde::{Deserialize, Serialize};

use crate::names::{element_kind, rename_component, reorder_nodes};
use crate::{AnalysisKind, Component, Field, Increment, LazyField, Values, add_derived_components};

/// The extension of a results file of the built-in solver.
pub const EXTENSION: &str = "ndlr";

/// The first bytes of a `.ndlr` file; the last two count the format, which is not converted:
/// results of another format are solved again.
const MAGIC: &[u8; 8] = b"PLXRES\x00\x0b";

/// What a run of the built-in solver computed, as Nodalis keeps it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SolverResults {
    /// The mesh the results are on: nodes, elements, sections and materials of the solver's
    /// output model, with shells and beams expanded as the `.frd` file shows them, and the
    /// names of its sets and surfaces. Their members, constraints, contact and loads are
    /// left out.
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
    /// the results need is kept; without the out-of-plane components of a 2D model (see
    /// [`plxs_core::results::plane`]), which [`from_solver`] fills in as zeros.
    pub fn new(model: &Model, mut results: Results, version: &str) -> Self {
        results.drop_out_of_plane(model);
        let mut kept = Model::default();
        kept.heading = model.heading.clone();
        kept.nodes = model.nodes.clone();
        kept.elements = model.elements.clone();
        kept.materials = model.materials.clone();
        kept.sections = model.sections.clone();
        // What the results are expanded to the sectors of a cyclic symmetric model with.
        kept.constraints.cyclic_symmetry = model.constraints.cyclic_symmetry.clone();
        // The names of the sets the history is of.
        kept.sets = model.sets.labels();
        kept.rebuild_indices();
        Self {
            model: kept,
            results,
            version: version.to_owned(),
            ..Self::default()
        }
    }

    /// The results as the bytes of a `.ndlr` file. The arrays are packed in place, without
    /// a copy of the results (hundreds of frames of a dynamic step are gigabytes).
    pub fn to_bytes(&mut self) -> Vec<u8> {
        packed::pack_in_place(MAGIC, self)
    }

    /// Reads the bytes of a `.ndlr` file.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, ReadError> {
        let mut results: SolverResults =
            packed::unpack(MAGIC, bytes).map_err(|e| ReadError(e.0))?;
        results.model.rebuild_indices();
        Ok(results)
    }

    /// Writes the `.ndlr` file, streaming the arrays as they are packed (the memory beyond
    /// the results is one array and its compressed chunks).
    pub fn write(&mut self, path: &Path) -> std::io::Result<()> {
        let file = std::fs::File::create(path)?;
        let mut out = std::io::BufWriter::new(file);
        packed::pack_to(MAGIC, self, &mut out)?;
        out.flush()?;
        out.into_inner()?.sync_all()
    }

    /// [`SolverResults::write`] of results whose increments are in `spool` (and none in
    /// [`SolverResults::results`]): the `.ndlr` file is written beside `path` as
    /// `<path>.tmp`, with the spooled arrays copied as they are stored, and renamed to `path`
    /// once it is complete. The increments of the steps `skip` are left out; the out-of-plane
    /// components of a 2D model are left out as [`SolverResults::new`] does, by all spooled
    /// increments.
    pub fn write_spooled(
        &mut self,
        spool: plxs_output::spool::Spool,
        skip: &[u32],
        path: &Path,
    ) -> std::io::Result<()> {
        // What a 2D model leaves out: decided by the values of every increment.
        let mut plane = None;
        if plxs_core::results::plane::is_plane(&self.model) {
            let mut zeros = plxs_core::results::plane::OutOfPlane::default();
            for index in 0..spool.len() {
                for field in &spool.read(index)?.fields {
                    zeros.add(field);
                }
            }
            plane = Some(zeros);
        }
        let partial = path.with_extension(format!("{EXTENSION}.tmp"));
        // A file that cannot be written whole, e.g. on a full disk, leaves no part behind.
        let written = (|| -> std::io::Result<()> {
            let file = std::fs::File::create(&partial)?;
            let mut out = packed::Streaming::new(MAGIC, std::io::BufWriter::new(file))?;
            // Node ids and coordinates are written from memory.
            let mut written = vec![None, None];
            let mut increments = Vec::new();
            for spooled in spool.increments() {
                if skip.contains(&spooled.skeleton.step) {
                    continue;
                }
                let mut increment = spooled.skeleton.clone();
                let mut arrays = spooled.arrays.iter();
                for field in &mut increment.fields {
                    let keep = plane.as_ref().and_then(|p| p.kept(field));
                    let nodes = arrays.next().copied();
                    written.push(
                        nodes
                            .map(|a| spool.raw(a).and_then(|r| out.raw(&r)))
                            .transpose()?,
                    );
                    let columns: Vec<usize> =
                        arrays.by_ref().take(field.values.len()).copied().collect();
                    for (k, column) in columns.into_iter().enumerate() {
                        if keep.as_ref().is_none_or(|keep| keep[k]) {
                            written.push(Some(out.raw(&spool.raw(column)?)?));
                        }
                    }
                    if let Some(keep) = &keep {
                        let mut kept = keep.iter();
                        field.components.retain(|_| *kept.next().unwrap_or(&true));
                        let mut kept = keep.iter();
                        field.values.retain(|_| *kept.next().unwrap_or(&true));
                    }
                }
                increments.push(increment);
            }
            drop(spool);
            // Increments in memory come after the spooled ones and are written from there.
            let spooled = increments.len();
            let in_memory = std::mem::replace(&mut self.results.increments, increments);
            self.results.increments.extend(in_memory);
            let mut count = 0;
            self.arrays(&mut |_| count += 1);
            written.resize(count, None);
            let finished = out.finish(self, &written);
            // The spooled increments stay out of memory: what is kept is their file.
            self.results.increments.drain(..spooled);
            let mut out = finished?;
            out.flush()?;
            out.into_inner().map_err(|e| e.into_error())?.sync_all()?;
            std::fs::rename(&partial, path)
        })();
        if written.is_err() {
            let _ = std::fs::remove_file(&partial);
        }
        written
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

/// Where the arrays of results of the built-in solver are read from: the results in
/// memory, or a `.ndlr` file ([`ResultsFile`]), read field by field as they are wanted.
pub trait ArraySource: Send + Sync {
    /// Field `field` of increment `ordinal` of [`Results::increments`], with its nodes and
    /// values.
    fn field(&self, ordinal: usize, field: usize) -> Result<ResultField, String>;
    /// Mode `mode` of mode set `set` of [`Results::modes`], with the nodes of the set.
    fn mode(&self, set: usize, mode: usize) -> Result<(Vec<u32>, ModeShape), String>;
}

const MISSING: &str = "the results file does not match the results shown";

impl ArraySource for SolverResults {
    fn field(&self, ordinal: usize, field: usize) -> Result<ResultField, String> {
        (self.results.increments.get(ordinal))
            .and_then(|i| i.fields.get(field))
            .cloned()
            .ok_or_else(|| MISSING.to_owned())
    }

    fn mode(&self, set: usize, mode: usize) -> Result<(Vec<u32>, ModeShape), String> {
        let set = self.results.modes.get(set).ok_or(MISSING)?;
        let mode = set.modes.get(mode).ok_or(MISSING)?;
        Ok((set.nodes.clone(), mode.clone()))
    }
}

/// A `.ndlr` file opened to read its fields one by one: what it holds besides the arrays,
/// and where each array is.
pub struct ResultsFile {
    archive: packed::Archive,
    /// The results with their arrays empty.
    skeleton: Results,
    /// Of every field of every increment, the number of the array of its nodes; its
    /// columns follow.
    fields: Vec<Vec<usize>>,
    /// Of every mode set, the array of its nodes, and of every mode those of its shape and
    /// imaginary part.
    modes: Vec<(usize, Vec<ModeArrays>)>,
}

/// The arrays of a mode: its shape and, if it has one, its imaginary part.
type ModeArrays = (usize, Option<usize>);

impl std::fmt::Debug for ResultsFile {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ResultsFile")
            .field("archive", &self.archive)
            .finish_non_exhaustive()
    }
}

impl ResultsFile {
    fn new(archive: packed::Archive, skeleton: Results) -> Self {
        // The mesh's node ids and coordinates come first, see `Packable for SolverResults`.
        let mut next = 2;
        let mut take = |count: usize| {
            let at = next;
            next += count;
            at
        };
        let fields = (skeleton.increments.iter())
            .map(|i| i.fields.iter().map(|f| take(1 + f.values.len())).collect())
            .collect();
        let modes = (skeleton.modes.iter())
            .map(|set| {
                let nodes = take(1);
                let modes = (set.modes.iter())
                    .map(|m| (take(1), m.imaginary.as_ref().map(|_| take(1))))
                    .collect();
                (nodes, modes)
            })
            .collect();
        Self {
            archive,
            skeleton,
            fields,
            modes,
        }
    }
}

impl ArraySource for ResultsFile {
    fn field(&self, ordinal: usize, field: usize) -> Result<ResultField, String> {
        let mut read = (self.skeleton.increments.get(ordinal))
            .and_then(|i| i.fields.get(field))
            .cloned()
            .ok_or(MISSING)?;
        let at = self.fields[ordinal][field];
        let error = |e: packed::PackError| e.0;
        self.archive
            .read(at, Array::Ids(&mut read.nodes))
            .map_err(error)?;
        for (k, column) in read.values.iter_mut().enumerate() {
            self.archive
                .read(at + 1 + k, Array::F64(column))
                .map_err(error)?;
        }
        Ok(read)
    }

    fn mode(&self, set: usize, mode: usize) -> Result<(Vec<u32>, ModeShape), String> {
        let error = |e: packed::PackError| e.0;
        let mut read = (self.skeleton.modes.get(set))
            .and_then(|s| s.modes.get(mode))
            .cloned()
            .ok_or(MISSING)?;
        let (nodes_at, modes) = &self.modes[set];
        let (shape_at, imaginary_at) = modes[mode];
        let mut nodes = Vec::new();
        self.archive
            .read(*nodes_at, Array::Ids(&mut nodes))
            .map_err(error)?;
        self.archive
            .read(shape_at, Array::F32x3(&mut read.shape))
            .map_err(error)?;
        if let (Some(at), Some(imaginary)) = (imaginary_at, &mut read.imaginary) {
            self.archive
                .read(at, Array::F32x3(imaginary))
                .map_err(error)?;
        }
        Ok((nodes, read))
    }
}

impl SolverResults {
    /// Opens a `.ndlr` file to read its fields as they are wanted: the results with their
    /// mesh, the rest of their arrays empty, and the file they are read from
    /// ([`from_source`]).
    pub fn open(path: &Path) -> Result<(Self, Arc<ResultsFile>), ReadError> {
        let failed = |e: String| ReadError(format!("{}: {e}", path.display()));
        let file = std::fs::File::open(path).map_err(|e| failed(e.to_string()))?;
        let archive = packed::Archive::open(MAGIC, Box::new(file)).map_err(|e| failed(e.0))?;
        Self::from_archive(archive).map_err(|e| failed(e.0))
    }

    /// [`SolverResults::open`] of a `.ndlr` file read from `source`, such as a project file.
    pub fn open_source(
        source: Box<dyn packed::Source>,
    ) -> Result<(Self, Arc<ResultsFile>), ReadError> {
        let archive = packed::Archive::open(MAGIC, source).map_err(|e| ReadError(e.0))?;
        Self::from_archive(archive)
    }

    /// [`SolverResults::open`] of the bytes of a `.ndlr` file in memory.
    pub fn open_bytes(bytes: Vec<u8>) -> Result<(Self, Arc<ResultsFile>), ReadError> {
        let archive = packed::Archive::from_bytes(MAGIC, bytes).map_err(|e| ReadError(e.0))?;
        Self::from_archive(archive)
    }

    fn from_archive(archive: packed::Archive) -> Result<(Self, Arc<ResultsFile>), ReadError> {
        let mut solved: SolverResults = archive.skeleton().map_err(|e| ReadError(e.0))?;
        let nodes = &mut solved.model.nodes;
        (archive.read(0, Array::Ids(&mut nodes.ids)))
            .and_then(|()| archive.read(1, Array::F64x3(&mut nodes.coordinates)))
            .map_err(|e| ReadError(e.0))?;
        solved.model.rebuild_indices();
        let file = ResultsFile::new(archive, solved.results.clone());
        Ok((solved, Arc::new(file)))
    }
}

/// The mesh and increments of `solved`, the same as reading the `.frd` file the solver would
/// write for them, without writing and parsing it. The fields are taken from `solved` when
/// they are wanted.
pub fn from_solver(solved: &SolverResults) -> SolverImport {
    from_source(solved, Arc::new(solved.clone()))
}

/// Where a written increment of [`from_source`] comes from.
#[derive(Clone, Copy, Debug)]
enum Origin {
    /// An increment of [`Results::increments`].
    Increment(usize),
    /// A mode of [`Results::modes`]: set and mode.
    Mode(usize, usize),
}

/// What the fields of [`from_source`] are read and converted with.
struct Context {
    source: Arc<dyn ArraySource>,
    /// The model the results are on, for a cyclic symmetric model, whose results are
    /// expanded to its sectors.
    cyclic: Option<Model>,
    /// The index in the mesh of every node id.
    index: HashMap<u32, usize>,
    nodes: usize,
    /// The components the mode fields keep in a 2D model, by field.
    plane_modes: Option<Vec<(String, Vec<String>)>>,
}

impl Context {
    /// The fields of a written increment that field `field` needs: itself, and of a mode
    /// of a cyclic symmetric model the other part of the mode, which turns it into the
    /// other sectors.
    fn read(&self, written: &Written, field: usize) -> Result<ResultsIncrement, String> {
        let name = &written.fields[field].name;
        let needed: Vec<usize> = (0..written.fields.len())
            .filter(|&f| {
                let other = &written.fields[f].name;
                f == field
                    || (self.cyclic.is_some()
                        && written.nodal_diameter.is_some()
                        && (imaginary_field_name(name) == *other
                            || imaginary_field_name(other) == *name))
            })
            .collect();
        let fields = match written.origin {
            Origin::Increment(ordinal) => (needed.iter())
                .map(|&f| self.source.field(ordinal, f))
                .collect::<Result<Vec<_>, _>>()?,
            Origin::Mode(set, mode) => {
                let (nodes, shape) = self.source.mode(set, mode)?;
                let mut fields = mode_fields(written.mode_kind, &nodes, &shape);
                if let Some(kept) = &self.plane_modes {
                    for field in &mut fields {
                        keep_components(field, kept);
                    }
                }
                let wanted: Vec<&str> = needed
                    .iter()
                    .map(|&f| written.fields[f].name.as_str())
                    .collect();
                fields.retain(|f| wanted.contains(&f.name.as_str()));
                fields
            }
        };
        Ok(ResultsIncrement {
            step: written.step,
            increment: written.increment,
            kind: written.kind,
            value: written.value,
            total_time: written.total_time,
            nodal_diameter: written.nodal_diameter,
            fields,
        })
    }

    /// The components of part `part` of field `field` of a written increment, converted as
    /// [`from_source`] converts them.
    fn load(&self, written: &Written, field: usize, part: usize) -> Result<Vec<Vec<f32>>, String> {
        let increment = self.read(written, field)?;
        let name = &written.fields[field].name;
        let fields = match &self.cyclic {
            Some(model) => plxs_output::frd::expand_increment(model, &increment),
            None => increment.fields,
        };
        let read = fields.iter().find(|f| f.name == *name).ok_or(MISSING)?;
        let mut converted = convert(read, |id| self.index.get(&id).copied(), self.nodes);
        add_derived_components(&mut converted);
        let part = (crate::split_rotations(converted).into_iter().nth(part)).ok_or(MISSING)?;
        Ok((part.components.into_iter())
            .map(|c| c.values.to_vec())
            .collect())
    }
}

/// An increment as the `.frd` file writes it: of [`Results::increments`], or a mode of an
/// eigenvalue step, with its fields without their values.
struct Written {
    step: u32,
    increment: u32,
    kind: IncrementKind,
    value: f64,
    total_time: f64,
    nodal_diameter: Option<u32>,
    fields: Vec<ResultField>,
    origin: Origin,
    /// The kind of the modes of a mode.
    mode_kind: plxs_core::results::ModeKind,
}

/// The mesh and increments of `solved`, of which only the mesh is needed: the fields are
/// read from `source` when they are wanted, see [`crate::values`].
pub fn from_source(solved: &SolverResults, source: Arc<dyn ArraySource>) -> SolverImport {
    let written = written_increments(&solved.results);
    // A cyclic symmetric model on all the sectors the `.frd` file has.
    let expanded = plxs_output::frd::expand_sectors(&solved.model, &Results::default());
    let model = expanded.as_ref().map_or(&solved.model, |(model, _)| model);
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
    let mut by_material: BTreeMap<i32, Vec<u32>> = BTreeMap::new();
    for i in plxs_output::frd::written_elements(model) {
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
            .map_or(1, |material| material.index() as i32 + 1);
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
            None => parts.push(Part {
                id: Default::default(),
                bodies: Vec::new(),
                name,
                elements,
            }),
        }
    }

    let mut written = written;
    let plane_modes = (plane::is_plane(&solved.model))
        .then(|| plane_mode_components(&solved.model, &written, &*source));
    if let Some(kept) = &plane_modes {
        for written in written
            .iter_mut()
            .filter(|w| matches!(w.origin, Origin::Mode(..)))
        {
            for field in &mut written.fields {
                keep_components(field, kept);
            }
        }
    }
    let context = Arc::new(Context {
        plane_modes,
        source,
        cyclic: expanded.is_some().then(|| solved.model.clone()),
        index: (import.mesh.node_ids().iter().enumerate())
            .map(|(i, &id)| (id, i))
            .collect(),
        nodes: import.mesh.node_count(),
    });
    let written: Vec<Arc<Written>> = written.into_iter().map(Arc::new).collect();
    let nodes = import.mesh.node_count();

    // The step, set and increment of the last buckling data set, see `buckling` below.
    let mut buckling: Option<(u32, usize, u32)> = None;
    // Eigenfrequency sensitivities of each step so far.
    let mut senfreq: BTreeMap<u32, u32> = BTreeMap::new();
    // The data set number of the `.frd` block header, which counts a mode of a cyclic
    // symmetric model twice (its real and imaginary solution, of which one is written).
    let mut set = 0u32;
    for (ordinal, written) in written.iter().enumerate() {
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
        for (index, field) in written.fields.iter().enumerate() {
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
            // The imaginary part of a cyclic symmetric mode only turned it into the other
            // sectors.
            if field.name == MODE_IMAGINARY {
                continue;
            }
            // The fields without values: their components, read when they are wanted.
            let mut converted = convert(field, |_| None, 0);
            add_derived_components(&mut converted);
            let parts = crate::split_rotations(converted);
            let fields = (parts.into_iter().enumerate()).map(|(part, mut shown)| {
                let loader = {
                    let (context, written) = (Arc::clone(&context), Arc::clone(written));
                    Box::new(move || context.load(&written, index, part))
                };
                let lazy = LazyField::new(nodes, shown.components.len(), loader);
                for (k, component) in shown.components.iter_mut().enumerate() {
                    component.values = Values::lazy(&lazy, k);
                }
                shown
            });
            let target = (import.increments.iter_mut())
                .find(|i| i.step == written.step && i.increment == increment && i.kind == kind);
            match target {
                Some(target) => {
                    if target.value == 0.0 {
                        target.value = value;
                    }
                    target.fields.extend(fields);
                }
                None => import.increments.push(Increment {
                    step: written.step,
                    increment,
                    kind,
                    value,
                    fields: fields.collect(),
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

/// A field of the solver on the nodes of the mesh, `count` values per component: values
/// of nodes `index_of` does not find in the mesh are left out, those of nodes without a
/// value are NaN. With the out-of-plane components a 2D model leaves out, as zeros.
fn convert(field: &ResultField, index_of: impl Fn(u32) -> Option<usize>, count: usize) -> Field {
    let index: Vec<Option<usize>> = field.nodes.iter().map(|&id| index_of(id)).collect();
    // The out-of-plane components a 2D model leaves out come back as zeros, unlisted.
    let names: Vec<&str> = match plane::full_components(&field.components) {
        Some(full) => full.to_vec(),
        None => field.components.iter().map(String::as_str).collect(),
    };
    let components = (names.into_iter())
        .map(|name| {
            let column = (field.components.iter())
                .position(|c| c == name)
                .map(|k| &field.values[k]);
            let mut values = vec![f32::NAN; count];
            for (n, at) in index.iter().enumerate() {
                if let Some(at) = at {
                    values[*at] = column.map_or(0.0, |c| c[n] as f32);
                }
            }
            Component {
                name: rename_component(name).to_string(),
                values: values.into(),
                derived: false,
                out_of_plane: column.is_none(),
            }
        })
        .collect();
    Field::new(field.name.clone(), components)
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

/// The increments the `.frd` file writes, without the values of their fields: those of the
/// results, with the modes of their eigenvalue steps as increments of one field each
/// ([`MODE`], with [`MODE_PHASES`] for complex modes and [`MODE_IMAGINARY`] for cyclic
/// symmetric ones) after the increments of the steps before.
fn written_increments(results: &Results) -> Vec<Written> {
    let empty = |field: &ResultField| ResultField {
        name: field.name.clone(),
        components: field.components.clone(),
        nodes: Vec::new(),
        values: vec![Vec::new(); field.values.len()],
    };
    let mut increments = (results.increments.iter().enumerate())
        .map(|(ordinal, i)| Written {
            step: i.step,
            increment: i.increment,
            kind: i.kind,
            value: i.value,
            total_time: i.total_time,
            nodal_diameter: i.nodal_diameter,
            fields: i.fields.iter().map(empty).collect(),
            origin: Origin::Increment(ordinal),
            mode_kind: ModeKind::Frequency,
        })
        .peekable();
    // A step that asked for fields (an input file run by the solver alone) shows them as
    // CalculiX writes them.
    let written = |step: u32| results.increments.iter().any(|i| i.step == step);
    let mut out = Vec::new();
    for (s, set) in results.modes.iter().enumerate() {
        if written(set.step) {
            continue;
        }
        for (m, mode) in set.modes.iter().enumerate() {
            // The modes of a step follow its other increments (a buckle step's reference
            // state).
            while let Some(increment) = increments.next_if(|i| i.step <= set.step) {
                out.push(increment);
            }
            let shape = ModeShape {
                shape: Vec::new(),
                imaginary: mode.imaginary.as_ref().map(|_| Vec::new()),
                ..mode.clone()
            };
            out.push(Written {
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
                fields: mode_fields(set.kind, &[], &shape),
                origin: Origin::Mode(s, m),
                mode_kind: set.kind,
            });
        }
    }
    out.extend(increments);
    out
}

/// The fields of a mode with the nodes `nodes` of its set, see [`written_increments`].
fn mode_fields(kind: ModeKind, nodes: &[u32], mode: &ModeShape) -> Vec<ResultField> {
    let field = |name: &str, components: &[&str], values: Vec<Vec<f64>>| ResultField {
        name: name.into(),
        components: components.iter().map(|c| c.to_string()).collect(),
        nodes: nodes.to_vec(),
        values,
    };
    let columns = |rows: &[[f32; 3]]| -> Vec<Vec<f64>> {
        (0..3)
            .map(|k| rows.iter().map(|r| f64::from(r[k])).collect())
            .collect()
    };
    let mut fields = vec![field(MODE, &["D1", "D2", "D3"], columns(&mode.shape))];
    match (&mode.imaginary, kind) {
        (Some(im), ModeKind::ComplexFrequency) => {
            let mut values: Vec<Vec<f64>> = (0..6).map(|_| Vec::with_capacity(im.len())).collect();
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
    fields
}

/// Keeps the components of `field` that `kept` lists for its name.
fn keep_components(field: &mut ResultField, kept: &[(String, Vec<String>)]) {
    let Some((_, names)) = kept.iter().find(|(n, _)| *n == field.name) else {
        return;
    };
    let mut values = std::mem::take(&mut field.values).into_iter();
    let (components, values): (Vec<String>, Vec<Vec<f64>>) =
        (std::mem::take(&mut field.components).into_iter())
            .zip(&mut values)
            .filter(|(c, _)| names.contains(c))
            .unzip();
    field.components = components;
    field.values = values;
}

/// The components of the mode fields of a 2D model without the out-of-plane ones that are
/// zero in every mode ([`Results::drop_out_of_plane`]), by field. Reads all modes.
fn plane_mode_components(
    model: &Model,
    written: &[Written],
    source: &dyn ArraySource,
) -> Vec<(String, Vec<String>)> {
    let mut modes = Results::default();
    for written in written {
        let Origin::Mode(set, mode) = written.origin else {
            continue;
        };
        let Ok((nodes, shape)) = source.mode(set, mode) else {
            continue;
        };
        modes.increments.push(ResultsIncrement {
            step: written.step,
            increment: written.increment,
            kind: written.kind,
            value: written.value,
            total_time: 0.0,
            nodal_diameter: written.nodal_diameter,
            fields: mode_fields(written.mode_kind, &nodes, &shape),
        });
    }
    modes.drop_out_of_plane(model);
    let mut kept: Vec<(String, Vec<String>)> = Vec::new();
    for field in modes.increments.iter().flat_map(|i| &i.fields) {
        if !kept.iter().any(|(n, _)| *n == field.name) {
            kept.push((field.name.clone(), field.components.clone()));
        }
    }
    kept
}
