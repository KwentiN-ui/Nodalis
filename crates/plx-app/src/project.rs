//! The project window, Ansys Workbench's project schematic: the analyses of the project as
//! blocks of a node graph, each with the state of its cells. An analysis opens in a window of
//! its own, the [`AnalysisWindow`], which looks like the classic main window.

use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::path::{Path, PathBuf};
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};

use crate::node_picker::{self, NodePicker};
use egui_snarl::ui::{AnyPins, PinInfo, SnarlStyle, SnarlViewer, SnarlWidget};
use egui_snarl::{InPin, NodeId, OutPin, Snarl};
use plx_io::project::{self as file, Link, Project, ProjectAnalysis, ProjectNode};
use plx_model::{
    MathOp, MeasureKind, MeshPrimitive, ModelProperties, ModelSpace, POINT_AXES, Param, ParamSlot,
    Parameter, PointRun, Quantity, StudyTable, UnitSystem, ValueKind,
};

use crate::app::{AnalysisWindow, Request, Show};
use crate::cad_editor::{CadEditorResult, CadOperationEditor};
use crate::campbell::CampbellWindow;
use crate::command::BlockTile;
use crate::links;
use crate::material_set::{Collection, MaterialSetEditor, MaterialSetResult};
use crate::measure::{MeasureEditor, MeasureEditorResult};
use crate::mesh_block::{MeshBlockEditor, MeshBlockResult};
use crate::model::PROJECT_EXTENSION;
use crate::model_properties::{DialogResult, ModelPropertiesDialog};
use crate::node_icons::{self, NodeIcon};
use crate::parameter_set::{ParameterSetEditor, ParameterSetResult};
use crate::settings::{self, Settings, SettingsWindow, WindowResult};
use crate::solution_monitor::{BackgroundSolve, Outcome};
use crate::status::{self, AnalysisStatus, CellState};
use crate::study::{PointView, StudyEdit, StudyEditor, StudyView};
use crate::tree::TreeView;
use crate::tree_icons::StepGlyph;
use cad::{CAD_LENGTH, CadBlock};
use deformed::{DEFORM_FACTOR, DEFORM_INCREMENT, DeformedBlock};

mod cad;
mod deformed;
mod frames;
mod history;
mod minimap;
mod numbers;
mod saving;
mod workspace;

/// Storage key of the project file open at exit, reopened at the next start.
const RECENT_KEY: &str = "recent_projects";
/// How many projects Open Recent lists.
const RECENT_MAX: usize = 10;

/// What a block of the schematic is, without its data: for menus, texts and decisions that
/// only depend on the kind.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Analysis,
    Materials,
    CrossSections,
    Mesh,
    RefineMesh,
    DeformedMesh,
    CadOperation,
    Parameters,
    Measure,
    Study,
    MeshExport,
    Campbell,
    File,
    Note,
    Math,
    Switch,
}

impl Kind {
    /// Whether the block is a source of data rather than an analysis.
    fn is_source(self) -> bool {
        self != Self::Analysis
    }

    /// What the block is called in the output and in dialogs.
    fn noun(self) -> &'static str {
        match self {
            Self::Analysis => "analysis",
            Self::Materials => "materials block",
            Self::CrossSections => "cross-sections block",
            Self::Mesh => "mesh block",
            Self::RefineMesh => "refine mesh block",
            Self::DeformedMesh => "deformed mesh block",
            Self::CadOperation => "CAD operation block",
            Self::Parameters => "parameters block",
            Self::Measure => "measure block",
            Self::Study => "parameter study",
            Self::MeshExport => "mesh export block",
            Self::Campbell => "Campbell block",
            Self::File => "file block",
            Self::Note => "note",
            Self::Math => "math block",
            Self::Switch => "switch block",
        }
    }
}

/// Whether a solve started.
enum Start {
    Started,
    /// Not now: its window is busy or still loading.
    Later,
    /// Not at all; the output tells why.
    Refused,
}

/// A block of the schematic: its name, the rows of it that are open, and what it is with
/// the data of that kind.
struct Node {
    id: u64,
    name: String,
    /// Rows of its block that are open: the setup and its categories.
    expanded: BTreeSet<Vec<String>>,
    block: Block,
}

/// A block with the data of its kind.
enum Block {
    Analysis(Box<AnalysisBlock>),
    /// A collection of materials, a source of data for analyses that is not solved.
    Materials(Box<CollectionBlock>),
    /// A collection of cross-sections, like the materials.
    CrossSections(Box<CollectionBlock>),
    /// A structured mesh made from a few parameters.
    Mesh(Box<MeshBlock>),
    /// The mesh linked to it made again with a factor on its fineness.
    RefineMesh(Box<RefineBlock>),
    /// The mesh of an analysis moved by the displacements of a step of its results.
    DeformedMesh(Box<DeformedBlock>),
    /// A fillet, chamfer or push/pull of the geometry linked to it.
    CadOperation(Box<CadBlock>),
    /// Named numbers that value links give to other blocks.
    Parameters(ParametersBlock),
    /// A key figure computed from the results of a step linked to it.
    Measure(MeasureBlock),
    /// A table of design points, each solved in turn with the values it gives to other
    /// blocks.
    Study(Box<StudyBlock>),
    /// A sink that writes the mesh linked to it to a file.
    MeshExport(Box<ExportBlock>),
    /// A sweep of the analysis linked to it over the rotational speed, with its Campbell
    /// diagram.
    Campbell(CampbellBlock),
    /// A file outside the project read as a source: geometry, a mesh or results.
    File(Box<FileBlock>),
    /// A text for the reader, which gives and takes nothing.
    Note(String),
    /// A number computed from others, or picked from several.
    Math(NumberBlock),
    Switch(NumberBlock),
}

/// A Math or a Switch block: the numbers of its inputs, which value links give or the block
/// holds, and what they are. The inputs of a Math block are those of its operation; those of
/// a Switch block are its index and then its choices.
#[derive(Clone)]
struct NumberBlock {
    units: UnitSystem,
    /// `None` until a link gives it the kind of the other end. Of a product or quotient,
    /// the kind of its result.
    kind: Option<ValueKind>,
    /// Of a product or quotient, the kinds of A and B, which the kind of the result follows
    /// from (see [`MathOp::settle_kinds`]); `None` while open.
    operands: [Option<ValueKind>; 2],
    operation: MathOp,
    values: Vec<f64>,
}

/// The row of the result of a Math or Switch block.
const RESULT: &str = "Result";
/// The input of a Switch block that picks the choice.
const INDEX: &str = "index";

impl NumberBlock {
    /// The inputs of a Math block, or the index and the choices of a Switch block: key of
    /// the row, label, the kind a link must bring (`None`: that of the block, still open),
    /// and the value.
    fn inputs(&self, switch: bool) -> Vec<(String, String, Option<ValueKind>, f64)> {
        let value = |i: usize| self.values.get(i).copied().unwrap_or(0.0);
        if switch {
            let mut inputs = vec![(
                INDEX.to_owned(),
                "Index".to_owned(),
                Some(ValueKind::Count),
                value(0),
            )];
            for i in 1..self.values.len().max(1) {
                inputs.push((i.to_string(), format!("Choice {i}"), self.kind, value(i)));
            }
            return inputs;
        }
        (self.operation.inputs().iter().enumerate())
            .map(|(i, label)| {
                let kind = if self.operation.combines() {
                    self.operands.get(i).copied().flatten()
                } else {
                    self.kind
                };
                (
                    ["a", "b", "c"][i].to_owned(),
                    (*label).to_owned(),
                    kind,
                    value(i),
                )
            })
            .collect()
    }

    /// The kind of the number of the row of `path`: its result or an input; `Some(None)`
    /// while it has none yet.
    fn kind_at(&self, switch: bool, path: &[String]) -> Option<Option<ValueKind>> {
        match path {
            [cell, name] if cell == links::VALUE && name == RESULT => Some(self.kind),
            [cell, key] if cell == links::PARAMETERS => (self.inputs(switch).into_iter())
                .find(|(k, ..)| k == key)
                .map(|(_, _, kind, _)| kind),
            _ => None,
        }
    }

    /// The kinds of A, B and the result of a product or quotient.
    fn kinds(&self) -> [Option<ValueKind>; 3] {
        [self.operands[0], self.operands[1], self.kind]
    }

    /// The kinds of A, B and the result, with the one missing found from the other two;
    /// unchanged with an error when they do not fit together.
    fn settle(&mut self) -> Result<(), String> {
        let [a, b, result] = self.operation.settle_kinds(self.kinds())?;
        self.operands = [a, b];
        self.kind = result;
        Ok(())
    }

    /// The block after the number of the row of `path` became of `kind`, with the kinds
    /// that follow from it; an error says why it cannot.
    fn with_kind(&self, switch: bool, path: &[String], kind: ValueKind) -> Result<Self, String> {
        let mut block = self.clone();
        match path {
            [cell, key] if cell == links::PARAMETERS && !switch && self.operation.combines() => {
                let index = ["a", "b"].iter().position(|k| k == key);
                if let Some(slot) = index.and_then(|i| block.operands.get_mut(i)) {
                    *slot = Some(kind);
                }
            }
            [cell, key] if cell == links::PARAMETERS && key == INDEX && switch => {}
            _ => block.kind = Some(kind),
        }
        block.settle()?;
        Ok(block)
    }

    /// The number it gives.
    fn result(&self, switch: bool) -> Option<f64> {
        let value = if switch {
            plx_model::switch(*self.values.first()?, self.values.get(1..)?)
        } else {
            self.operation.apply(&self.values)
        }?;
        Some(self.kind.map_or(value, |k| k.clean(value)))
    }

    /// Gives the input `key` the value; whether it changed.
    fn set(&mut self, switch: bool, key: &str, value: f64) -> bool {
        let Some((at, (_, _, kind, _))) =
            (self.inputs(switch).into_iter().enumerate()).find(|(_, (k, ..))| k == key)
        else {
            return false;
        };
        let value = kind.map_or(value, |k| k.clean(value));
        if self.values.len() <= at {
            self.values.resize(at + 1, 0.0);
        }
        let changed = self.values[at] != value;
        self.values[at] = value;
        changed
    }
}

/// An analysis: its data, or the window that holds them while it is open.
struct AnalysisBlock {
    /// The analysis' data while its window is closed; an open window holds it.
    data: Option<ProjectAnalysis>,
    window: Option<Box<AnalysisWindow>>,
    /// Whether its window is a window of its own; otherwise the project window shows it in
    /// the workspaces Geometry, FE Model and Results.
    own_window: bool,
    /// Where the window opens the first time: over the project window.
    window_position: Option<egui::Pos2>,
    /// The state of its cells, as last found.
    status: Option<AnalysisStatus>,
    /// Its model space, shown in the header of its block.
    space: ModelSpace,
    /// What it takes from other blocks that is not ready yet; found by
    /// [`ProjectApp::sync_links`].
    upstream: crate::status::Upstream,
    /// Solved in the background, its window closed.
    solving: bool,
    /// The revision of its open window as last seen, see [`AnalysisWindow::revision`].
    seen: Option<u64>,
    /// Its links were not brought up to date because its window was busy: they are once it
    /// is idle.
    sync_missed: bool,
}

/// The model data a block that is no analysis gives to links or takes from them, in the form
/// links read and fill: the materials of a materials block, the mesh of a mesh block or of a
/// mesh export block.
#[derive(Default)]
struct LinkedModel {
    geometry: Option<plx_model::Geometry>,
    mesh: plx_mesh::FeMesh,
    fe: plx_model::FeModel,
    mesh_inputs: BTreeMap<String, u64>,
}

impl LinkedModel {
    fn data(&self) -> links::Data<'_> {
        links::Data {
            geometry: self.geometry.as_ref(),
            mesh: &self.mesh,
            fe: &self.fe,
            mesh_inputs: &self.mesh_inputs,
        }
    }

    fn data_mut(&mut self) -> ModelMut<'_> {
        ModelMut {
            geometry: &mut self.geometry,
            mesh: &mut self.mesh,
            fe: &mut self.fe,
            mesh_inputs: &mut self.mesh_inputs,
        }
    }
}

/// What links fill in a block whose model is not in an open window.
struct ModelMut<'a> {
    geometry: &'a mut Option<plx_model::Geometry>,
    mesh: &'a mut plx_mesh::FeMesh,
    fe: &'a mut plx_model::FeModel,
    mesh_inputs: &'a mut BTreeMap<String, u64>,
}

impl ModelMut<'_> {
    /// Takes over a mesh with what belongs to it.
    fn take_mesh(&mut self, copy: links::MeshCopy) {
        let links::MeshCopy {
            geometry,
            mesh,
            mesh_inputs,
            mesh_sets,
            element_types,
            mesh_operations,
        } = copy;
        if let Some(tags) = links::geometry_tags(self.geometry.as_ref(), geometry.as_ref()) {
            links::follow_geometry(self.fe, None, &tags);
        }
        *self.geometry = geometry;
        *self.mesh = mesh;
        *self.mesh_inputs = mesh_inputs;
        self.fe.mesh_sets = mesh_sets;
        self.fe.element_types = element_types;
        self.fe.mesh_operations = mesh_operations;
    }
}

/// A materials or cross-sections block: its items, in the setup of a model, and the units
/// they are shown in.
struct CollectionBlock {
    model: LinkedModel,
    units: UnitSystem,
}

/// A mesh block: what it meshes, the mesh it made and the fingerprint of what that was made
/// from.
struct MeshBlock {
    model: LinkedModel,
    space: ModelSpace,
    units: UnitSystem,
    primitive: MeshPrimitive,
    generated: Option<u64>,
}

/// A refine mesh block: the factor, what it refines, and the mesh it gives with the
/// fingerprint of what that was made from.
struct RefineBlock {
    model: LinkedModel,
    factor: f64,
    /// What the link of its Mesh row brings, found by [`ProjectApp::sync_links`].
    input: Option<RefineInput>,
    /// Fingerprint of the divisions its mesh was made from, see [`MeshPrimitive::fingerprint`].
    generated: Option<u64>,
    /// Fingerprint of the geometry and mesh it gave as last taken, with its factor.
    taken: Option<u64>,
}

/// What a refine mesh block makes its mesh from.
enum RefineInput {
    /// The parameters of a mesh block, times the factors of the refine mesh blocks between:
    /// the block makes the mesh with its divisions refined.
    Primitive {
        primitive: MeshPrimitive,
        space: ModelSpace,
        factor: f64,
    },
    /// Geometry with its mesh setup: the analysis the mesh goes to meshes it again, its
    /// element sizes divided by the factor.
    Geometry { space: ModelSpace, refinement: f64 },
    /// A mesh it cannot make again, with the name of where it comes from.
    Fixed(String),
}

impl RefineBlock {
    /// What it meshes when it refines the parameters of a mesh block.
    fn meshed(&self) -> Option<(MeshPrimitive, ModelSpace)> {
        match &self.input {
            Some(RefineInput::Primitive {
                primitive,
                space,
                factor,
            }) => Some((primitive.refined(factor * self.factor), *space)),
            _ => None,
        }
    }

    /// Makes its mesh from the parameters of the mesh block it refines; returns the number
    /// of elements, or why it cannot be made.
    fn generate(&mut self) -> Result<usize, String> {
        let (primitive, space) = match &self.input {
            Some(RefineInput::Geometry { .. }) => {
                return Err("geometry is meshed by the analysis the mesh goes to".into());
            }
            Some(RefineInput::Fixed(source)) => {
                return Err(format!("the mesh of {source} cannot be made again"));
            }
            None => return Err("no mesh is linked to it".into()),
            Some(RefineInput::Primitive { .. }) => self.meshed().ok_or("no mesh")?,
        };
        if let Some(problem) = primitive.problem(space) {
            return Err(problem);
        }
        let mesh = primitive.generate(space, &primitive.label().to_uppercase());
        let count = mesh.element_count();
        self.generated = Some(primitive.fingerprint(space));
        self.model = LinkedModel {
            mesh,
            ..Default::default()
        };
        self.model.fe.properties.space = space;
        Ok(count)
    }
}

/// A parameters block: its numbers.
struct ParametersBlock {
    units: UnitSystem,
    parameters: Vec<Parameter>,
}

/// What a measure block computes, and the value it last computed with the fingerprint of the
/// results and the definition it was computed from, see [`ProjectApp::measure_key`].
struct MeasureBlock {
    units: UnitSystem,
    kind: MeasureKind,
    value: Option<f64>,
    evaluated: Option<u64>,
    /// Why the last evaluation gave no value.
    error: Option<String>,
}

/// A parameter study: its table, the fingerprint of the blocks it runs as they are now, see
/// [`ProjectApp::study_body_key`], and the point being solved while it runs.
struct StudyBlock {
    units: UnitSystem,
    table: StudyTable,
    body: Option<u64>,
    active: Option<usize>,
    /// How its last run ended.
    report: Option<String>,
}

/// A mesh export block: the mesh linked to it, the file it writes to, the fingerprint of the
/// mesh and path it last wrote, and why its last write failed.
struct ExportBlock {
    model: LinkedModel,
    path: String,
    written: Option<u64>,
    error: Option<String>,
}

/// What a file block reads, by the extension of its file.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FileKind {
    /// CAD geometry: STEP, IGES, BREP or STL.
    Geometry,
    /// The mesh of a CalculiX input file.
    Mesh,
    /// The results of each step of a CalculiX results file.
    Results,
}

impl FileKind {
    fn of(path: &str) -> Option<Self> {
        let path = Path::new(path.trim());
        let extension = path.extension()?.to_str()?.to_ascii_lowercase();
        match extension.as_str() {
            "inp" => Some(Self::Mesh),
            "frd" => Some(Self::Results),
            _ if plx_mesher::is_cad_file(path) => Some(Self::Geometry),
            _ => None,
        }
    }
}

/// A file block: the file, what was read from it and the fingerprint of the file then.
struct FileBlock {
    model: LinkedModel,
    path: String,
    /// The unit system the numbers of an input file and the lengths of geometry are in.
    units: UnitSystem,
    /// The model space of the mesh read, for links that need the same.
    space: Option<ModelSpace>,
    /// The steps of a results file: the name of each row with its number in the file.
    steps: Vec<(String, u32)>,
    /// The parts of a geometry file, which can be linked one by one.
    parts: Vec<String>,
    /// The geometry with only some of its parts, by those parts, for the analyses that link
    /// only them; made on worker threads, as Gmsh removes the other parts.
    subsets: BTreeMap<Vec<String>, Subset>,
    /// Fingerprint of the file as last read, see [`file_print`], and why that read failed.
    read: Option<u64>,
    error: Option<String>,
    /// The file being read on a worker thread, with its fingerprint.
    reading: Option<(u64, Receiver<Result<FileRead, String>>)>,
}

/// What a worker read from the file of a file block.
struct FileRead {
    model: LinkedModel,
    space: Option<ModelSpace>,
    steps: Vec<(String, u32)>,
    parts: Vec<String>,
    summary: String,
}

/// The geometry of a file block with only some of its parts.
enum Subset {
    Making(Receiver<Result<plx_model::Geometry, String>>),
    Made(plx_model::Geometry),
    Failed(String),
}

impl FileBlock {
    /// What was read is gone, or another file is read: so are the parts made of it.
    fn forget(&mut self) {
        self.model = LinkedModel::default();
        self.steps.clear();
        self.parts.clear();
        self.subsets.clear();
    }

    /// The geometry with only `parts`: the whole geometry when those are all its parts;
    /// `None` while it is being made, which starts here. Why it could not be made is an
    /// error.
    fn subset(&mut self, parts: &[String]) -> Result<Option<plx_model::Geometry>, String> {
        let Some(geometry) = &self.model.geometry else {
            return Ok(None);
        };
        if let Some(part) = parts.iter().find(|p| !self.parts.contains(p)) {
            return Err(format!("the file has no part {part}"));
        }
        if self.parts.iter().all(|p| parts.contains(p)) {
            return Ok(Some(geometry.clone()));
        }
        // In the order of the file, so that the same parts are made once.
        let key: Vec<String> = (self.parts.iter())
            .filter(|p| parts.contains(p))
            .cloned()
            .collect();
        match self.subsets.get(&key) {
            Some(Subset::Made(geometry)) => return Ok(Some(geometry.clone())),
            Some(Subset::Failed(why)) => return Err(why.clone()),
            Some(Subset::Making(_)) => return Ok(None),
            None => {}
        }
        let others: Vec<String> = (self.parts.iter())
            .filter(|p| !key.contains(p))
            .cloned()
            .collect();
        let (sender, receiver) = std::sync::mpsc::channel();
        let geometry = geometry.clone();
        std::thread::spawn(move || {
            let made = plx_mesher::delete_parts_renumbered(&geometry, &others)
                .map_err(|e| e.to_string())
                .and_then(|(made, _)| made.ok_or_else(|| "no part is left".to_owned()));
            let _ = sender.send(made);
        });
        self.subsets.insert(key, Subset::Making(receiver));
        Ok(None)
    }

    /// Takes the subsets made since; whether any was.
    fn poll_subsets(&mut self) -> bool {
        let mut done = false;
        for subset in self.subsets.values_mut() {
            if let Subset::Making(receiver) = subset {
                *subset = match receiver.try_recv() {
                    Ok(Ok(geometry)) => Subset::Made(geometry),
                    Ok(Err(why)) => Subset::Failed(why),
                    Err(std::sync::mpsc::TryRecvError::Empty) => continue,
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                        Subset::Failed("making it stopped unexpectedly".into())
                    }
                };
                done = true;
            }
        }
        done
    }

    fn making_subsets(&self) -> bool {
        (self.subsets.values()).any(|s| matches!(s, Subset::Making(_)))
    }
}

/// A Campbell block: its sweep and diagram, and the fingerprint of the analysis linked to it
/// as last found by [`ProjectApp::sync_links`], see [`crate::campbell::sweep_fingerprint`].
struct CampbellBlock {
    /// Taken out while the project window shows it or starts its sweep.
    window: Option<Box<CampbellWindow>>,
    input: Option<u64>,
}

impl Node {
    fn new(id: u64, name: String, block: Block) -> Self {
        // A materials block shows its materials at first.
        let expanded = match block {
            Block::Materials(_) => BTreeSet::from([material_set_path()]),
            Block::CrossSections(_) => BTreeSet::from([cross_section_set_path()]),
            _ => BTreeSet::new(),
        };
        let mut node = Self {
            id,
            name,
            expanded,
            block,
        };
        node.refresh_closed();
        node
    }

    /// An analysis with these data.
    fn analysis(id: u64, name: String, data: ProjectAnalysis) -> Self {
        let space = data.model.properties.space;
        let block = AnalysisBlock {
            data: Some(data),
            window: None,
            own_window: false,
            window_position: None,
            status: None,
            space,
            upstream: Default::default(),
            solving: false,
            seen: None,
            sync_missed: false,
        };
        Self::new(id, name, Block::Analysis(Box::new(block)))
    }

    /// A mesh block meshing `primitive`, with `mesh` made from `generated`.
    fn mesh_block(
        id: u64,
        name: String,
        space: ModelSpace,
        units: UnitSystem,
        primitive: MeshPrimitive,
        mesh: plx_mesh::FeMesh,
        generated: Option<u64>,
    ) -> Self {
        let mut model = LinkedModel {
            mesh,
            ..Default::default()
        };
        model.fe.properties.space = space;
        let block = MeshBlock {
            model,
            space,
            units,
            primitive,
            generated,
        };
        Self::new(id, name, Block::Mesh(Box::new(block)))
    }

    /// A refine mesh block with this factor, holding `mesh` made from `generated`.
    fn refine_block(
        id: u64,
        name: String,
        factor: f64,
        mesh: plx_mesh::FeMesh,
        generated: Option<u64>,
    ) -> Self {
        let block = RefineBlock {
            model: LinkedModel {
                mesh,
                ..Default::default()
            },
            factor,
            input: None,
            generated,
            taken: None,
        };
        Self::new(id, name, Block::RefineMesh(Box::new(block)))
    }

    /// A mesh export block writing to `path`, which it wrote with `written`.
    fn mesh_export(id: u64, name: String, path: String, written: Option<u64>) -> Self {
        let block = ExportBlock {
            model: LinkedModel::default(),
            path,
            written,
            error: None,
        };
        Self::new(id, name, Block::MeshExport(Box::new(block)))
    }

    /// A file block reading `path`, in `units`.
    fn file_block(id: u64, name: String, path: String, units: UnitSystem) -> Self {
        let block = FileBlock {
            model: LinkedModel::default(),
            path,
            units,
            space: None,
            steps: Vec::new(),
            parts: Vec::new(),
            subsets: BTreeMap::new(),
            read: None,
            error: None,
            reading: None,
        };
        Self::new(id, name, Block::File(Box::new(block)))
    }

    /// A Campbell block with its sweep and diagram.
    fn campbell_block(id: u64, name: String, window: CampbellWindow) -> Self {
        let block = CampbellBlock {
            window: Some(Box::new(window)),
            input: None,
        };
        Self::new(id, name, Block::Campbell(block))
    }

    fn kind(&self) -> Kind {
        match &self.block {
            Block::Analysis(_) => Kind::Analysis,
            Block::Materials(_) => Kind::Materials,
            Block::CrossSections(_) => Kind::CrossSections,
            Block::Mesh(_) => Kind::Mesh,
            Block::RefineMesh(_) => Kind::RefineMesh,
            Block::DeformedMesh(_) => Kind::DeformedMesh,
            Block::CadOperation(_) => Kind::CadOperation,
            Block::Parameters(_) => Kind::Parameters,
            Block::Measure(_) => Kind::Measure,
            Block::Study(_) => Kind::Study,
            Block::MeshExport(_) => Kind::MeshExport,
            Block::Campbell(_) => Kind::Campbell,
            Block::File(_) => Kind::File,
            Block::Note(_) => Kind::Note,
            Block::Math(_) => Kind::Math,
            Block::Switch(_) => Kind::Switch,
        }
    }

    fn as_analysis(&self) -> Option<&AnalysisBlock> {
        match &self.block {
            Block::Analysis(a) => Some(a),
            _ => None,
        }
    }

    fn as_analysis_mut(&mut self) -> Option<&mut AnalysisBlock> {
        match &mut self.block {
            Block::Analysis(a) => Some(a),
            _ => None,
        }
    }

    /// What a collection block holds.
    fn collection(&self) -> Option<Collection> {
        match self.block {
            Block::Materials(_) => Some(Collection::Materials),
            Block::CrossSections(_) => Some(Collection::CrossSections),
            _ => None,
        }
    }

    fn as_collection(&self) -> Option<&CollectionBlock> {
        match &self.block {
            Block::Materials(m) | Block::CrossSections(m) => Some(m),
            _ => None,
        }
    }

    fn as_collection_mut(&mut self) -> Option<&mut CollectionBlock> {
        match &mut self.block {
            Block::Materials(m) | Block::CrossSections(m) => Some(m),
            _ => None,
        }
    }

    fn as_mesh(&self) -> Option<&MeshBlock> {
        match &self.block {
            Block::Mesh(m) => Some(m),
            _ => None,
        }
    }

    fn as_mesh_mut(&mut self) -> Option<&mut MeshBlock> {
        match &mut self.block {
            Block::Mesh(m) => Some(m),
            _ => None,
        }
    }

    fn as_refine(&self) -> Option<&RefineBlock> {
        match &self.block {
            Block::RefineMesh(r) => Some(r),
            _ => None,
        }
    }

    fn as_refine_mut(&mut self) -> Option<&mut RefineBlock> {
        match &mut self.block {
            Block::RefineMesh(r) => Some(r),
            _ => None,
        }
    }

    fn as_deformed(&self) -> Option<&DeformedBlock> {
        match &self.block {
            Block::DeformedMesh(d) => Some(d),
            _ => None,
        }
    }

    fn as_deformed_mut(&mut self) -> Option<&mut DeformedBlock> {
        match &mut self.block {
            Block::DeformedMesh(d) => Some(d),
            _ => None,
        }
    }

    fn as_cad(&self) -> Option<&CadBlock> {
        match &self.block {
            Block::CadOperation(c) => Some(c),
            _ => None,
        }
    }

    fn as_cad_mut(&mut self) -> Option<&mut CadBlock> {
        match &mut self.block {
            Block::CadOperation(c) => Some(c),
            _ => None,
        }
    }

    fn as_measure(&self) -> Option<&MeasureBlock> {
        match &self.block {
            Block::Measure(m) => Some(m),
            _ => None,
        }
    }

    fn as_measure_mut(&mut self) -> Option<&mut MeasureBlock> {
        match &mut self.block {
            Block::Measure(m) => Some(m),
            _ => None,
        }
    }

    fn as_study(&self) -> Option<&StudyBlock> {
        match &self.block {
            Block::Study(s) => Some(s),
            _ => None,
        }
    }

    fn as_study_mut(&mut self) -> Option<&mut StudyBlock> {
        match &mut self.block {
            Block::Study(s) => Some(s),
            _ => None,
        }
    }

    fn as_parameters(&self) -> Option<&ParametersBlock> {
        match &self.block {
            Block::Parameters(p) => Some(p),
            _ => None,
        }
    }

    fn as_parameters_mut(&mut self) -> Option<&mut ParametersBlock> {
        match &mut self.block {
            Block::Parameters(p) => Some(p),
            _ => None,
        }
    }

    fn as_export(&self) -> Option<&ExportBlock> {
        match &self.block {
            Block::MeshExport(e) => Some(e),
            _ => None,
        }
    }

    fn as_export_mut(&mut self) -> Option<&mut ExportBlock> {
        match &mut self.block {
            Block::MeshExport(e) => Some(e),
            _ => None,
        }
    }

    fn as_file(&self) -> Option<&FileBlock> {
        match &self.block {
            Block::File(f) => Some(f),
            _ => None,
        }
    }

    fn as_file_mut(&mut self) -> Option<&mut FileBlock> {
        match &mut self.block {
            Block::File(f) => Some(f),
            _ => None,
        }
    }

    fn as_campbell(&self) -> Option<&CampbellBlock> {
        match &self.block {
            Block::Campbell(c) => Some(c),
            _ => None,
        }
    }

    fn as_campbell_mut(&mut self) -> Option<&mut CampbellBlock> {
        match &mut self.block {
            Block::Campbell(c) => Some(c),
            _ => None,
        }
    }

    /// The open window of an analysis.
    /// A Math or Switch block, and whether it is a switch.
    fn as_number(&self) -> Option<(&NumberBlock, bool)> {
        match &self.block {
            Block::Math(n) => Some((n, false)),
            Block::Switch(n) => Some((n, true)),
            _ => None,
        }
    }

    fn as_number_mut(&mut self) -> Option<(&mut NumberBlock, bool)> {
        match &mut self.block {
            Block::Math(n) => Some((n, false)),
            Block::Switch(n) => Some((n, true)),
            _ => None,
        }
    }

    /// For a Math or Switch block, the kind of the number of the row of `path`: its result
    /// or an input; `Some(None)` while the block has no kind yet.
    fn number_kind(&self, path: &[String]) -> Option<Option<ValueKind>> {
        let (number, switch) = self.as_number()?;
        number.kind_at(switch, path)
    }

    fn window(&self) -> Option<&AnalysisWindow> {
        self.as_analysis()?.window.as_deref()
    }

    fn window_mut(&mut self) -> Option<&mut AnalysisWindow> {
        self.as_analysis_mut()?.window.as_deref_mut()
    }

    /// The state of an analysis' cells, as last found.
    fn status(&self) -> Option<&AnalysisStatus> {
        self.as_analysis()?.status.as_ref()
    }

    /// The state of an analysis' cells as it is now, for what is drawn in every frame:
    /// found again by an open window only when the analysis changed.
    fn fresh_status(&mut self) -> Option<AnalysisStatus> {
        match self.window_mut() {
            Some(window) => window.status(),
            None => self.status().cloned(),
        }
    }

    /// The state of an analysis' cells found now, for decisions: an open window may have
    /// taken what links brought in this frame.
    fn current_status(&self) -> Option<AnalysisStatus> {
        match self.window() {
            Some(window) => window.current_status(),
            None => self.status().cloned(),
        }
    }

    /// The model space shown in the header of the block, of an analysis or a mesh block.
    fn space(&self) -> Option<ModelSpace> {
        match &self.block {
            Block::Analysis(a) => Some(a.space),
            Block::Mesh(m) => Some(m.space),
            Block::RefineMesh(r) => match &r.input {
                Some(
                    RefineInput::Primitive { space, .. } | RefineInput::Geometry { space, .. },
                ) => Some(*space),
                _ => None,
            },
            Block::File(f) => f.space,
            Block::DeformedMesh(d) => d.space,
            _ => None,
        }
    }

    /// The unit system the block shows its values in; an analysis' is that of its model.
    fn units(&self) -> UnitSystem {
        match &self.block {
            Block::Analysis(_) => {
                (self.link_data()).map_or_else(UnitSystem::default, |d| d.fe.properties.units)
            }
            Block::Materials(m) | Block::CrossSections(m) => m.units,
            Block::Mesh(m) => m.units,
            Block::Parameters(p) => p.units,
            Block::Measure(m) => m.units,
            Block::Study(s) => s.units,
            Block::File(f) => f.units,
            Block::Math(n) | Block::Switch(n) => n.units,
            Block::CadOperation(c) => c.units,
            Block::MeshExport(_)
            | Block::Campbell(_)
            | Block::RefineMesh(_)
            | Block::DeformedMesh(_)
            | Block::Note(_) => UnitSystem::default(),
        }
    }

    /// Fingerprint of the mesh a mesh export block holds and the path it writes to; `None`
    /// while no mesh is linked to it.
    fn export_print(&self) -> Option<u64> {
        let export = self.as_export()?;
        let data = export.model.data();
        if data.mesh.element_count() == 0 {
            return None;
        }
        let mut hash = plx_mesh::Fnv::new();
        hash.add(&links::fingerprint(data, &[links::MESH.to_owned()]).to_le_bytes());
        hash.add_str(&export.path);
        Some(hash.finish())
    }

    /// The state of a mesh export block's file; `base` is the folder of the project file.
    fn export_state(&self, base: Option<&Path>) -> CellState {
        let (Some(export), Some(print)) = (self.as_export(), self.export_print()) else {
            return CellState::Missing;
        };
        if export.path.trim().is_empty() {
            CellState::Missing
        } else if export.error.is_some() {
            CellState::Failed
        } else if export.written == Some(print) && export_file(&export.path, base).is_file() {
            CellState::UpToDate
        } else {
            CellState::OutOfDate
        }
    }

    /// Writes the mesh of a mesh export block as an input file; returns the file.
    fn write_export(&mut self, base: Option<&Path>) -> Result<PathBuf, String> {
        let print = self
            .export_print()
            .ok_or("no mesh is linked to it, drag the mesh of a block to its Mesh row")?;
        let name = self.name.clone();
        let export = self.as_export_mut().ok_or("not a mesh export block")?;
        if export.path.trim().is_empty() {
            return Err("no file is chosen".into());
        }
        let file = export_file(&export.path, base);
        let text = plx_io::inp::write_inp(&export.model.mesh, &export.model.fe, &name)
            .map_err(|e| e.to_string())?;
        std::fs::write(&file, text).map_err(|e| format!("{} not written: {e}", file.display()))?;
        export.written = Some(print);
        Ok(file)
    }

    /// The state of a file block's data: missing without a file, out of date while the file
    /// changed since it was read; `base` is the folder of the project file.
    fn file_state(&self, base: Option<&Path>) -> CellState {
        let Some(file) = self.as_file() else {
            return CellState::Missing;
        };
        if file.reading.is_some() {
            return CellState::Running;
        }
        let print = file_print(&export_file(&file.path, base));
        match print {
            None => CellState::Missing,
            Some(_) if file.error.is_some() => CellState::Failed,
            Some(print) if Some(print) == file.read => CellState::UpToDate,
            Some(_) => CellState::OutOfDate,
        }
    }

    /// The state of the mesh of a mesh block or a refine mesh block: out of date once its
    /// parameters or its factor changed. A refine mesh block that refines geometry gives it
    /// as it is, to be meshed by the analysis it goes to.
    fn mesh_state(&self) -> CellState {
        if let Some(refine) = self.as_refine() {
            return match &refine.input {
                None | Some(RefineInput::Fixed(_)) => CellState::Missing,
                _ if refine.model.mesh.element_count() == 0 => CellState::OutOfDate,
                Some(RefineInput::Geometry { .. }) => CellState::UpToDate,
                Some(RefineInput::Primitive { .. }) => {
                    let made = refine.meshed().map(|(p, space)| p.fingerprint(space));
                    if made.is_some() && made == refine.generated {
                        CellState::UpToDate
                    } else {
                        CellState::OutOfDate
                    }
                }
            };
        }
        let Some(block) = self.as_mesh() else {
            return CellState::Missing;
        };
        if block.model.mesh.element_count() == 0 {
            CellState::Missing
        } else if Some(block.primitive.fingerprint(block.space)) == block.generated {
            CellState::UpToDate
        } else {
            CellState::OutOfDate
        }
    }

    /// Makes the mesh of a mesh block or a refine mesh block from its parameters; returns
    /// the number of elements, or why it cannot be made.
    fn generate(&mut self) -> Result<usize, String> {
        match &mut self.block {
            Block::Mesh(m) => m.generate(),
            Block::RefineMesh(r) => r.generate(),
            _ => Err("not a mesh block".into()),
        }
    }

    /// A block read from the project file.
    fn read(node: ProjectNode) -> Self {
        match node {
            ProjectNode::Analysis(analysis) => {
                Self::analysis(analysis.id, analysis.name.clone(), *analysis)
            }
            ProjectNode::Materials(set) => {
                let mut model = LinkedModel::default();
                model.fe.materials = set.materials;
                model.fe.exposed = set.exposed;
                let block = CollectionBlock {
                    model,
                    units: set.units,
                };
                Self::new(set.id, set.name, Block::Materials(Box::new(block)))
            }
            ProjectNode::CrossSections(set) => {
                let mut model = LinkedModel::default();
                model.fe.cross_sections = set.cross_sections;
                model.fe.exposed = set.exposed;
                let block = CollectionBlock {
                    model,
                    units: set.units,
                };
                Self::new(set.id, set.name, Block::CrossSections(Box::new(block)))
            }
            ProjectNode::Mesh(block) => {
                let block = *block;
                Self::mesh_block(
                    block.id,
                    block.name,
                    block.space,
                    block.units,
                    block.primitive,
                    block.mesh,
                    block.generated,
                )
            }
            ProjectNode::RefineMesh(block) => {
                let block = *block;
                let factor = if block.factor.is_finite() && block.factor > 0.0 {
                    block.factor
                } else {
                    1.0
                };
                Self::refine_block(block.id, block.name, factor, block.mesh, block.generated)
            }
            ProjectNode::DeformedMesh(block) => {
                let (id, name) = (block.id, block.name.clone());
                let deformed = DeformedBlock::read(*block);
                Self::new(id, name, Block::DeformedMesh(Box::new(deformed)))
            }
            ProjectNode::CadOperation(block) => {
                let block = *block;
                let cad =
                    CadBlock::new(block.operation, block.units, block.result, block.made_from);
                Self::new(block.id, block.name, Block::CadOperation(Box::new(cad)))
            }
            ProjectNode::Parameters(set) => {
                let block = ParametersBlock {
                    units: set.units,
                    parameters: set.parameters,
                };
                Self::new(set.id, set.name, Block::Parameters(block))
            }
            ProjectNode::Measure(block) => {
                let measure = MeasureBlock {
                    units: block.units,
                    kind: block.kind,
                    value: block.value,
                    evaluated: block.evaluated,
                    error: None,
                };
                Self::new(block.id, block.name, Block::Measure(measure))
            }
            ProjectNode::Study(block) => {
                let study = StudyBlock {
                    units: block.units,
                    table: block.table,
                    body: None,
                    active: None,
                    report: None,
                };
                Self::new(block.id, block.name, Block::Study(Box::new(study)))
            }
            ProjectNode::MeshExport(e) => Self::mesh_export(e.id, e.name, e.path, e.written),
            ProjectNode::File(f) => Self::file_block(f.id, f.name, f.path, f.units),
            ProjectNode::Campbell(block) => {
                let block = *block;
                let window = CampbellWindow::saved(
                    block.setup,
                    block.points,
                    block.focus,
                    block.fingerprint,
                );
                Self::campbell_block(block.id, block.name, window)
            }
            ProjectNode::Note(note) => Self::new(note.id, note.name, Block::Note(note.text)),
            ProjectNode::Math(math) => {
                let block = NumberBlock {
                    units: math.units,
                    kind: math.kind,
                    operands: math.operands,
                    operation: math.operation,
                    values: math.values,
                };
                Self::new(math.id, math.name, Block::Math(block))
            }
            ProjectNode::Switch(switch) => {
                let mut values = vec![switch.index];
                values.extend(switch.choices);
                let block = NumberBlock {
                    units: switch.units,
                    kind: switch.kind,
                    operands: [None; 2],
                    operation: MathOp::default(),
                    values,
                };
                Self::new(switch.id, switch.name, Block::Switch(block))
            }
        }
    }

    /// The block as the project file keeps it, from the window of an analysis if it is
    /// open; `position` is that of its block.
    fn written(&mut self, position: [f32; 2]) -> Option<ProjectNode> {
        let (id, name) = (self.id, self.name.clone());
        Some(match &mut self.block {
            Block::Analysis(analysis) => {
                let mut data = match &mut analysis.window {
                    Some(window) => window.data()?,
                    None => analysis.data.clone()?,
                };
                data.id = id;
                data.name = name;
                data.position = position;
                ProjectNode::Analysis(Box::new(data))
            }
            Block::Materials(m) => ProjectNode::Materials(file::MaterialSet {
                id,
                name,
                position,
                units: m.units,
                materials: m.model.fe.materials.clone(),
                exposed: m.model.fe.exposed.clone(),
            }),
            Block::CrossSections(m) => ProjectNode::CrossSections(file::CrossSectionSet {
                id,
                name,
                position,
                units: m.units,
                cross_sections: m.model.fe.cross_sections.clone(),
                exposed: m.model.fe.exposed.clone(),
            }),
            Block::Mesh(m) => ProjectNode::Mesh(Box::new(file::MeshBlock {
                id,
                name,
                position,
                space: m.space,
                units: m.units,
                primitive: m.primitive.clone(),
                mesh: m.model.mesh.clone(),
                generated: m.generated,
            })),
            // The mesh of geometry is not kept: the link brings it again.
            Block::RefineMesh(r) => ProjectNode::RefineMesh(Box::new(file::RefineMesh {
                id,
                name,
                position,
                factor: r.factor,
                mesh: match r.generated {
                    Some(_) => r.model.mesh.clone(),
                    None => Default::default(),
                },
                generated: r.generated,
            })),
            // The mesh it made is kept, so that the project opens without reading the results.
            Block::DeformedMesh(d) => {
                ProjectNode::DeformedMesh(Box::new(d.written(id, name, position)))
            }
            // The geometry it made is kept, so that the project meshes without making it again.
            Block::CadOperation(c) => {
                ProjectNode::CadOperation(Box::new(file::CadOperationBlock {
                    id,
                    name,
                    position,
                    units: c.units,
                    operation: c.operation.clone(),
                    result: c.model.geometry.clone(),
                    made_from: c.made_from,
                }))
            }
            Block::Parameters(p) => ProjectNode::Parameters(file::ParameterSet {
                id,
                name,
                position,
                units: p.units,
                parameters: p.parameters.clone(),
            }),
            Block::Measure(m) => ProjectNode::Measure(file::MeasureBlock {
                id,
                name,
                position,
                units: m.units,
                kind: m.kind.clone(),
                value: m.value,
                evaluated: m.evaluated,
            }),
            Block::Study(s) => ProjectNode::Study(file::StudyBlock {
                id,
                name,
                position,
                units: s.units,
                table: s.table.clone(),
            }),
            // It keeps no mesh: the link brings it again.
            Block::MeshExport(e) => ProjectNode::MeshExport(file::MeshExport {
                id,
                name,
                position,
                path: e.path.clone(),
                written: e.written,
            }),
            // It keeps only the path: the file is read again.
            Block::File(f) => ProjectNode::File(file::FileSource {
                id,
                name,
                position,
                path: f.path.clone(),
                units: f.units,
            }),
            // It keeps no model: the link brings it again.
            Block::Campbell(c) => {
                let window = c.window.as_ref()?;
                ProjectNode::Campbell(Box::new(file::CampbellBlock {
                    id,
                    name,
                    position,
                    setup: window.setup().clone(),
                    points: window.points().to_vec(),
                    focus: window.focus(),
                    fingerprint: window.fingerprint(),
                }))
            }
            Block::Note(text) => ProjectNode::Note(file::Note {
                id,
                name,
                position,
                text: text.clone(),
            }),
            Block::Math(math) => ProjectNode::Math(file::MathBlock {
                id,
                name,
                position,
                units: math.units,
                operation: math.operation,
                kind: math.kind,
                operands: math.operands,
                values: math.values.clone(),
            }),
            Block::Switch(switch) => ProjectNode::Switch(file::SwitchBlock {
                id,
                name,
                position,
                units: switch.units,
                kind: switch.kind,
                index: switch.values.first().copied().unwrap_or(1.0),
                choices: switch.values.get(1..).unwrap_or_default().to_vec(),
            }),
        })
    }

    /// A copy of the block as a new one, `id` named `name`: its definition without what it
    /// computed.
    fn copied(&mut self, id: u64, name: String) -> Option<Self> {
        let block = match &mut self.block {
            Block::Analysis(analysis) => {
                let mut data = match &mut analysis.window {
                    Some(window) => window.data()?,
                    None => analysis.data.clone()?,
                };
                data.id = id;
                data.name = name.clone();
                return Some(Self::analysis(id, name, data));
            }
            Block::Materials(m) => {
                let mut model = LinkedModel::default();
                model.fe.materials = m.model.fe.materials.clone();
                Block::Materials(Box::new(CollectionBlock {
                    model,
                    units: m.units,
                }))
            }
            Block::CrossSections(m) => {
                let mut model = LinkedModel::default();
                model.fe.cross_sections = m.model.fe.cross_sections.clone();
                Block::CrossSections(Box::new(CollectionBlock {
                    model,
                    units: m.units,
                }))
            }
            Block::Mesh(m) => {
                let mut model = LinkedModel {
                    mesh: m.model.mesh.clone(),
                    ..Default::default()
                };
                model.fe.properties.space = m.space;
                Block::Mesh(Box::new(MeshBlock {
                    model,
                    space: m.space,
                    units: m.units,
                    primitive: m.primitive.clone(),
                    generated: m.generated,
                }))
            }
            Block::RefineMesh(r) => {
                return Some(Self::refine_block(
                    id,
                    name,
                    r.factor,
                    Default::default(),
                    None,
                ));
            }
            Block::DeformedMesh(d) => {
                Block::DeformedMesh(Box::new(DeformedBlock::new(d.factor, d.increment)))
            }
            Block::CadOperation(c) => Block::CadOperation(Box::new(CadBlock::new(
                c.operation.clone(),
                c.units,
                None,
                None,
            ))),
            Block::Parameters(p) => Block::Parameters(ParametersBlock {
                units: p.units,
                parameters: p.parameters.clone(),
            }),
            Block::Measure(m) => Block::Measure(MeasureBlock {
                units: m.units,
                kind: m.kind.clone(),
                value: None,
                evaluated: None,
                error: None,
            }),
            Block::Study(s) => {
                let mut table = s.table.clone();
                table.reset();
                Block::Study(Box::new(StudyBlock {
                    units: s.units,
                    table,
                    body: None,
                    active: None,
                    report: None,
                }))
            }
            Block::MeshExport(e) => {
                return Some(Self::mesh_export(id, name, e.path.clone(), None));
            }
            Block::File(f) => return Some(Self::file_block(id, name, f.path.clone(), f.units)),
            Block::Campbell(c) => {
                let setup = c.window.as_ref()?.setup().clone();
                let window = CampbellWindow::saved(setup, Vec::new(), None, None);
                return Some(Self::campbell_block(id, name, window));
            }
            Block::Note(text) => Block::Note(text.clone()),
            Block::Math(math) => Block::Math(math.clone()),
            Block::Switch(switch) => Block::Switch(switch.clone()),
        };
        Some(Self::new(id, name, block))
    }

    /// Brings the state of an analysis up to date after the project changed its model
    /// through [`Self::exposing_fe_mut`]: an open window counts the change, a closed analysis
    /// finds its state again.
    fn model_changed(&mut self) {
        match self.window_mut() {
            Some(window) => window.note_change(),
            None => self.refresh_closed(),
        }
    }

    /// Finds the state of a closed analysis from its data, once after it changed.
    fn refresh_closed(&mut self) {
        let Some(analysis) = self.as_analysis_mut() else {
            return;
        };
        if let Some(data) = &analysis.data {
            analysis.status = Some(status::analysis_status(
                data.geometry.as_ref(),
                &data.mesh,
                &data.model,
                &data.mesh_inputs,
                data.results.as_ref(),
                &analysis.upstream,
                analysis.solving,
            ));
        }
    }

    /// What links read of the block; `None` while the window of an analysis loads it, and
    /// for blocks that hold no model.
    fn link_data(&self) -> Option<links::Data<'_>> {
        match &self.block {
            Block::Analysis(analysis) => match (&analysis.window, &analysis.data) {
                (Some(window), _) => window.model().map(|m| links::Data {
                    geometry: m.geometry.as_ref(),
                    mesh: &m.mesh,
                    fe: &m.fe,
                    mesh_inputs: &m.mesh_inputs,
                }),
                (None, Some(data)) => Some(links::Data {
                    geometry: data.geometry.as_ref(),
                    mesh: &data.mesh,
                    fe: &data.model,
                    mesh_inputs: &data.mesh_inputs,
                }),
                _ => None,
            },
            Block::Materials(m) | Block::CrossSections(m) => Some(m.model.data()),
            Block::Mesh(m) => Some(m.model.data()),
            Block::RefineMesh(r) => Some(r.model.data()),
            Block::DeformedMesh(d) => d.link_data(),
            Block::CadOperation(c) => c.link_data(),
            Block::MeshExport(e) => Some(e.model.data()),
            Block::File(f) => Some(f.model.data()),
            Block::Parameters(_)
            | Block::Measure(_)
            | Block::Study(_)
            | Block::Campbell(_)
            | Block::Note(_)
            | Block::Math(_)
            | Block::Switch(_) => None,
        }
    }

    /// What links fill in the block while no window holds its model.
    fn closed_model_mut(&mut self) -> Option<ModelMut<'_>> {
        match &mut self.block {
            Block::Analysis(analysis) if analysis.window.is_none() => {
                let data = analysis.data.as_mut()?;
                Some(ModelMut {
                    geometry: &mut data.geometry,
                    mesh: &mut data.mesh,
                    fe: &mut data.model,
                    mesh_inputs: &mut data.mesh_inputs,
                })
            }
            Block::Materials(m) | Block::CrossSections(m) => Some(m.model.data_mut()),
            Block::Mesh(m) => Some(m.model.data_mut()),
            Block::MeshExport(e) => Some(e.model.data_mut()),
            _ => None,
        }
    }

    /// The model whose numbers can be exposed as parameters: of an analysis or a materials
    /// block; `None` while the window of an analysis loads it.
    fn exposing_fe(&self) -> Option<&plx_model::FeModel> {
        match self.kind() {
            Kind::Analysis | Kind::Materials | Kind::CrossSections => {
                self.link_data().map(|d| d.fe)
            }
            _ => None,
        }
    }

    fn exposing_fe_mut(&mut self) -> Option<&mut plx_model::FeModel> {
        match &mut self.block {
            Block::Analysis(analysis) => match (&mut analysis.window, &mut analysis.data) {
                (Some(window), _) => window.fe_mut(),
                (None, Some(data)) => Some(&mut data.model),
                _ => None,
            },
            Block::Materials(m) | Block::CrossSections(m) => Some(&mut m.model.fe),
            _ => None,
        }
    }

    /// Sets the exposed parameter of `key`; returns whether its value changed.
    fn set_exposed(&mut self, key: u32, value: f64) -> bool {
        let Some(fe) = self.exposing_fe_mut() else {
            return false;
        };
        let Some(path) = fe.exposed(key).map(|e| e.path.clone()) else {
            return false;
        };
        // A number of a mesh operation moves the mesh of the analysis along.
        let changed = match &mut self.block {
            Block::Analysis(analysis) => match (&mut analysis.window, &mut analysis.data) {
                (Some(window), _) => window.set_value(&path, value),
                (None, Some(data)) => data.model.set_value_meshed(&mut data.mesh, &path, value),
                _ => false,
            },
            _ => self
                .exposing_fe_mut()
                .is_some_and(|fe| fe.set_value(&path, value)),
        };
        if changed {
            self.model_changed();
        }
        changed
    }

    /// The factor of a refine mesh block; `None` for other blocks.
    fn refinement(&self) -> Option<f64> {
        self.as_refine().map(|r| r.factor)
    }

    /// Sets the factor of a refine mesh block; its mesh is out of date once it changes.
    fn set_refinement(&mut self, value: f64) {
        if let Some(refine) = self.as_refine_mut()
            && value.is_finite()
            && value > 0.0
        {
            refine.factor = value;
        }
    }

    /// Whether it has results, but made from data that changed since or being made anew.
    fn results_out_of_date(&self) -> bool {
        self.bound_results().is_some()
            && self.status().is_some_and(|s| {
                matches!(
                    s.solution,
                    status::CellState::OutOfDate | status::CellState::Running
                )
            })
    }

    /// Whether the results of `step` that the analysis keeps were made from data that
    /// changed since. The solver solves the steps one after the other, so they depend only on
    /// the model and the steps up to `step`: steps added or changed after it leave them up to
    /// date, while the analysis itself is out of date.
    fn step_results_out_of_date(&self, step: &str) -> bool {
        let (Some(bound), Some(status), Some(data)) =
            (self.bound_results(), self.status(), self.link_data())
        else {
            return self.results_out_of_date();
        };
        if status.mesh == status::CellState::OutOfDate || !status.stale_results.is_empty() {
            return true;
        }
        match data.fe.steps.iter().position(|s| s.name == step) {
            // Results solved before they kept the fingerprints of their steps count whole.
            Some(index) if !bound.steps.is_empty() => {
                let inputs = crate::solve_fingerprint::step_inputs(data.mesh, data.fe);
                bound.steps.get(index) != inputs.get(index)
            }
            _ => self.results_out_of_date(),
        }
    }

    /// Whether what `link` takes of the analysis' results is out of date: the results of its
    /// step for results and state links, all of them for anything else. Eigenmodes count
    /// whole, as the file holding them is written anew by every frequency step.
    fn out_of_date_for(&self, link: &Link) -> bool {
        let by_step = links::is_state(link)
            || (links::is_results(link)
                && links::ResultsUse::of(link) != Some(links::ResultsUse::Modes));
        match link.item.get(1) {
            Some(step) if by_step => self.step_results_out_of_date(step),
            _ => self.results_out_of_date(),
        }
    }

    /// The fingerprint naming the copy of the results of `step` a link takes: the one of the
    /// step, so that a run that solved only later steps anew keeps the copy.
    fn step_key(&self, step: &str) -> u64 {
        let Some(bound) = self.bound_results() else {
            return 0;
        };
        (self.link_data())
            .and_then(|data| data.fe.steps.iter().position(|s| s.name == step))
            .and_then(|index| bound.steps.get(index).copied())
            .or(bound.input)
            .unwrap_or_default()
    }

    /// The results the analysis keeps.
    fn bound_results(&self) -> Option<&plx_io::project::BoundResults> {
        let analysis = self.as_analysis()?;
        match (&analysis.window, &analysis.data) {
            (Some(window), _) => window.model()?.bound_results.as_ref(),
            (None, Some(data)) => data.results.as_ref(),
            _ => None,
        }
    }
}

impl MeshBlock {
    /// Makes its mesh from its parameters; returns the number of elements, or why it cannot
    /// be made.
    fn generate(&mut self) -> Result<usize, String> {
        let primitive = self.primitive.clone();
        if let Some(problem) = primitive.problem(self.space) {
            return Err(problem);
        }
        let mesh = primitive.generate(self.space, &primitive.label().to_uppercase());
        let count = mesh.element_count();
        self.generated = Some(primitive.fingerprint(self.space));
        self.model.mesh = mesh;
        self.model.fe.properties.space = self.space;
        Ok(count)
    }
}

/// A parameter study under way: the points left to solve, the one being solved with when
/// it started, and whether Update Project was asked for it.
struct StudyRun {
    id: u64,
    /// The fingerprint of the blocks as they stood at the current point when it started,
    /// which every point it solves takes, see [`ProjectApp::study_body_key`].
    body: Option<u64>,
    points: VecDeque<usize>,
    current: Option<(usize, Instant)>,
    updating: bool,
    /// How many points it solved, and how many of those failed.
    solved: usize,
    failed: usize,
}

/// What a link copies into its target.
enum Payload {
    Setup(Box<plx_model::FeModel>),
    /// What a link of a category takes: the source's model, the names of the items it takes
    /// and those linked items of the target that come before them, see
    /// [`links::apply_bundle`].
    Bundle {
        fe: Box<plx_model::FeModel>,
        names: Vec<String>,
        ahead: Vec<String>,
    },
    Geometry(Option<plx_model::Geometry>),
    Mesh(Box<links::MeshCopy>),
    /// The step of the source's results and their file, see [`links::apply_results`].
    Results(u32, PathBuf),
    /// What a state link takes: the mesh with its geometry when it changed, the source's
    /// model when the setup or the steps up to `step` changed.
    Chain {
        mesh: Option<Box<links::MeshCopy>>,
        fe: Option<Box<plx_model::FeModel>>,
        step: usize,
    },
}

/// What the user did in the schematic, done after it is drawn.
enum Action {
    New(egui::Pos2),
    /// A new materials block at the position.
    AddMaterials(egui::Pos2),
    AddCrossSections(egui::Pos2),
    /// A new mesh block meshing the primitive at the position.
    AddMesh(egui::Pos2, MeshPrimitive),
    /// A new refine mesh block at the position.
    AddRefineMesh(egui::Pos2),
    /// A new mesh export block at the position.
    AddMeshExport(egui::Pos2),
    /// A new Campbell block at the position.
    AddCampbell(egui::Pos2),
    /// A new file block at the position.
    AddFile(egui::Pos2),
    /// Read the file of a file block again.
    ReadFile(u64),
    /// The file a mesh export block writes to was edited in its block.
    SetExportPath(u64, String),
    /// Write the file of a mesh export block now.
    ExportNow(u64),
    /// Parameters of a mesh block changed in its block.
    SetPrimitive(u64, MeshPrimitive),
    /// A new parameters block at the position.
    AddParameters(egui::Pos2),
    /// A new measure block at the position.
    AddMeasure(egui::Pos2),
    /// A new parameter study at the position.
    AddStudy(egui::Pos2),
    /// Solve the points of a study that are not up to date, or forget what they gave.
    RunStudy(u64),
    ResetStudy(u64),
    /// The value of a parameter of a parameters block, by its index, changed in its block.
    SetParameter(u64, usize, f64),
    /// The value of an exposed parameter of an analysis or a materials block, by its key,
    /// changed in its block.
    SetExposed(u64, u32, f64),
    /// An exposed parameter was renamed in its block.
    RenameExposed(u64, u32, String),
    /// An exposed parameter is to be no longer exposed, with its links.
    Unexpose(u64, u32),
    /// The factor of a refine mesh block changed in its block.
    SetRefinement(u64, f64),
    /// The length of a CAD operation changed in its block.
    SetCadLength(u64, f64),
    /// A coordinate of the point of a measure block, by its key in [`POINT_AXES`].
    SetMeasurePoint(u64, &'static str, f64),
    /// A new CAD operation block at the position.
    AddCadOperation(egui::Pos2),
    /// A new deformed mesh block at the position.
    AddDeformedMesh(egui::Pos2),
    /// The factor or the increment of a deformed mesh block changed in its block.
    SetDeformFactor(u64, f64),
    SetDeformIncrement(u64, f64),

    /// Make the mesh of a mesh block anew.
    Generate(u64),
    /// Open the analysis in the project window, in the workspace of a cell if given.
    Open(u64, Option<Show>),
    /// Open the analysis in a window of its own.
    OpenInWindow(u64),
    Solve(u64),
    /// Update the block and what it takes data from: its out-of-date symbol was clicked.
    Update(u64),
    Rename(u64),
    /// Open or close a row of the block.
    Toggle(u64, Vec<String>),
    /// Link `item` of `source` to `slot` of `target`.
    Link {
        source: u64,
        item: Vec<String>,
        target: u64,
        slot: Vec<String>,
    },
    /// A new analysis that continues `source` after `step`, its block at the position.
    Continue {
        source: u64,
        step: String,
        position: egui::Pos2,
    },
    /// Split the analysis after the step: the steps after it go to a new analysis that
    /// continues it.
    Split(u64, String),
    /// Give the own steps of the analysis to the one it continues, and remove it.
    Merge(u64),
    /// Remove the links of a pin, or the one between two pins.
    Unlink {
        source: Option<(u64, Vec<String>)>,
        target: Option<(u64, Vec<String>)>,
    },
    /// A new note at the position.
    AddNote(egui::Pos2),
    /// The text of a note was edited in its block.
    SetNote(u64, String),
    /// Duplicate the blocks, or delete them once the user agrees.
    DuplicateAll(Vec<u64>),
    DeleteAll(Vec<u64>),
    /// Show the blocks compact, or all in full if they are all compact.
    Compact(Vec<u64>),
    /// Put the blocks in a new frame, or take them out of theirs.
    Frame(Vec<u64>),
    Unframe(Vec<u64>),
    /// A frame, by its index, was dragged by the vector, given a label or a colour, or is
    /// to be removed without its blocks.
    MoveFrame(usize, egui::Vec2),
    FrameLabel(usize, String),
    FrameColor(usize, [u8; 3]),
    RemoveFrame(usize),
    /// Show the blocks to add at the position on the screen and in the schematic.
    Pick(egui::Pos2, egui::Pos2),
    /// A new Math or Switch block at the position.
    AddMath(egui::Pos2),
    AddSwitch(egui::Pos2),
    /// The operation of a Math block, the value of one of the inputs of a Math or Switch
    /// block by key, or the number of choices of a Switch block changed in its block.
    SetOperation(u64, MathOp),
    SetNumberInput(u64, String, f64),
    SetChoices(u64, usize),
}

/// A project read from its file, with notes on its conversion, or why it could not be read.
type Opened = Result<(Project, Vec<String>), String>;

pub struct ProjectApp {
    render_state: egui_wgpu::RenderState,
    settings: Settings,
    settings_window: Option<SettingsWindow>,
    updater: crate::update::Updater,
    output: Vec<String>,
    /// What the status bar tells of the output while its tab is not shown.
    output_news: crate::output_news::OutputNews,
    /// The project file; `None` until the project is saved.
    path: Option<PathBuf>,
    nodes: Vec<Node>,
    links: Vec<Link>,
    /// The schematic: a block per analysis, by its id.
    graph: Snarl<u64>,
    /// Counts the graphs shown, so that the view centres on the blocks of each new one.
    graph_generation: u64,
    /// Name typed for an analysis being renamed, with why it cannot be taken.
    rename: Option<(u64, String)>,
    /// Blocks to delete once the user agrees.
    confirm_delete: Vec<u64>,
    /// The toast over the schematic, such as "Deleted Beam" with Undo.
    toast: Option<crate::toast::Toast>,
    /// The frames around groups of blocks.
    frames: Vec<file::Frame>,
    /// The blocks shown compact.
    compact: BTreeSet<u64>,
    /// The size each block was drawn at in the last frame, in the schematic.
    block_sizes: HashMap<u64, egui::Vec2>,
    /// The blocks selected in the schematic.
    selected: Vec<u64>,
    /// The frame whose label is being edited, by index, with the text typed.
    frame_label: Option<(usize, String)>,
    /// The note whose text takes the keyboard in the next frame.
    focus_note: Option<u64>,
    /// The positions of the blocks in the last frame, and the blocks moved while a button
    /// is down, to put those dropped onto a frame in it.
    last_positions: Vec<(u64, [f32; 2])>,
    moving: BTreeSet<u64>,
    /// The editor of a materials block.
    material_set: Option<MaterialSetEditor>,
    /// The editor of a mesh block.
    mesh_block: Option<MeshBlockEditor>,
    /// The editor of a CAD operation block.
    cad_editor: Option<CadOperationEditor>,
    /// The 3D view of the CAD operation editor, made when it is first opened and kept for
    /// the next.
    cad_viewport: Option<crate::viewport::Viewport>,
    /// The editor of a parameters block.
    parameter_set: Option<ParameterSetEditor>,
    /// The editor of a measure block.
    measure_editor: Option<MeasureEditor>,
    /// Measures being computed on worker threads, by block, with the fingerprint they are
    /// computed for.
    measuring: HashMap<u64, (u64, Receiver<Result<f64, String>>)>,
    /// The editor of a parameter study.
    study_editor: Option<StudyEditor>,
    /// The study being run, and the studies Update Project runs after it.
    looping: Option<StudyRun>,
    loops_due: VecDeque<u64>,
    /// Update Project made meshes anew and goes on once links took them.
    update_after_sync: bool,
    /// The blocks the update is kept to: a block whose out-of-date symbol was clicked and
    /// those it takes data from; `None` for Update Project, which updates them all.
    update_scope: Option<Vec<u64>>,
    /// The overview of blocks to add, open after a double-click into the schematic.
    picker: Option<NodePicker>,
    /// Show All Blocks was asked for: the view fits the blocks in the next frame.
    show_all: bool,
    /// Whether the minimap of the schematic is shown.
    show_minimap: bool,
    /// The point of the schematic the view is to centre on next, from the minimap.
    look_at: Option<egui::Pos2>,
    /// The part of the schematic the minimap spans while the pointer is down on it, so the map
    /// holds still under the pointer while the view follows.
    minimap_bounds: Option<egui::Rect>,
    /// Where the pins of the schematic were drawn on the screen in the last frame.
    pin_rects: Vec<(PinKey, egui::Rect)>,
    /// The pin a wire is being dragged from in the schematic.
    dragging: Option<PinKey>,
    /// The properties of a new analysis being asked for, with where its block goes.
    new_analysis: Option<(egui::Pos2, ModelPropertiesDialog)>,
    /// A project file being read on a worker thread, with notes on its conversion.
    opening: Option<(PathBuf, Receiver<Opened>)>,
    /// The project files opened or saved last, the latest first; see Open Recent.
    recent: Vec<PathBuf>,
    /// The save running on a worker thread, and what autosave needs to know.
    saver: saving::Saver,
    /// Analyses Update Project still has to solve, in order, and those it solves.
    update_queue: VecDeque<u64>,
    updating: Vec<u64>,
    /// Mesh export blocks Update Project writes once the analyses are done, and whether an
    /// analysis failed meanwhile.
    export_queue: Vec<u64>,
    update_failed: bool,
    /// The solves of the analyses, running and done, for the solution monitor.
    solves: Vec<BackgroundSolve>,
    /// Analyses Solve Again asked for: solved even if their results are up to date.
    forced: BTreeSet<u64>,
    /// The solution monitor, with the analysis whose output it shows and its filter.
    monitor: Monitor,
    /// Files given on the command line, imported into the first analysis once it is open.
    startup_import: Vec<PathBuf>,
    /// Fingerprint of the geometry or mesh a link last copied into its target, by target and
    /// path: copied again once the source's differs.
    applied: HashMap<(u64, Vec<String>), u64>,
    /// Whether links have to bring their targets up to date: something changed that they
    /// may take.
    sync_due: bool,
    /// What the user changed in the project window, to undo and redo it.
    history: history::History,
    /// Global results being written for submodels, by the folder they are written to;
    /// `None` once writing failed.
    restoring: HashMap<PathBuf, Option<Receiver<Result<PathBuf, String>>>>,
    /// Problems of links already reported in the output, each once.
    warned: BTreeSet<String>,
    /// When the files of the file blocks were last looked at for changes.
    files_checked: Option<Instant>,
    /// Whether the solution and results of each analysis were up to date as the links last
    /// took them, see [`ProjectApp::freshness`].
    synced_freshness: Vec<(u64, CellState, CellState)>,
    /// The title last given to the window: setting it asks egui for another frame, so it
    /// is set only when it changes, else the app would draw without pause.
    shown_title: String,
    /// Whether the window shows the Project workspace; otherwise the active analysis in
    /// the workspace its window shows.
    in_project: bool,
    /// The analysis the workspaces Geometry, FE Model and Results act on.
    active: Option<u64>,
    /// The tab of each workspace and whether the ribbon is folded.
    ribbon: crate::ribbon::RibbonState,
    /// The command search (Ctrl+K).
    search: Option<NodePicker>,
    /// The kind of block hanging at the pointer until a click places it.
    placing: Option<BlockTile>,
}

impl ProjectApp {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let render_state = cc
            .wgpu_render_state
            .clone()
            .ok_or("Nodalis requires the wgpu backend of eframe")?;
        crate::crash_log::watch_gpu(&render_state.device);
        crate::style::apply(&cc.egui_ctx);
        (cc.egui_ctx).add_plugin(crate::screenshot::ImmediateCapture::new(
            render_state.clone(),
        ));
        let adapter = render_state.adapter.get_info();
        let output = vec![format!(
            "Nodalis {} started, graphics: {} ({:?})",
            env!("CARGO_PKG_VERSION"),
            adapter.name,
            adapter.backend
        )];
        let mut settings: Settings = cc
            .storage
            .and_then(|s| eframe::get_value(s, settings::STORAGE_KEY))
            .unwrap_or_default();
        settings.upgrade();
        settings.solver.apply_gpu();
        crate::results::set_significant_digits(settings.post.digits);
        crate::style::set_theme(&cc.egui_ctx, settings.graphics.theme);
        plx_mesher::set_library_path(settings.gmsh.library());
        let mut app = Self {
            render_state,
            settings,
            settings_window: None,
            updater: Default::default(),
            output,
            path: None,
            nodes: Vec::new(),
            links: Vec::new(),
            graph: Snarl::new(),
            graph_generation: 0,
            rename: None,
            confirm_delete: Vec::new(),
            toast: None,
            frames: Vec::new(),
            compact: BTreeSet::new(),
            block_sizes: HashMap::new(),
            selected: Vec::new(),
            frame_label: None,
            focus_note: None,
            last_positions: Vec::new(),
            moving: BTreeSet::new(),
            material_set: None,
            mesh_block: None,
            cad_editor: None,
            cad_viewport: None,
            parameter_set: None,
            measure_editor: None,
            measuring: HashMap::new(),
            study_editor: None,
            looping: None,
            loops_due: VecDeque::new(),
            update_after_sync: false,
            update_scope: None,
            picker: None,
            show_all: false,
            show_minimap: true,
            look_at: None,
            minimap_bounds: None,
            pin_rects: Vec::new(),
            dragging: None,
            new_analysis: None,
            opening: None,
            recent: cc.storage.map(stored_recent).unwrap_or_default(),
            saver: Default::default(),
            update_queue: VecDeque::new(),
            updating: Vec::new(),
            shown_title: String::new(),
            export_queue: Vec::new(),
            update_failed: false,
            solves: Vec::new(),
            forced: BTreeSet::new(),
            monitor: Monitor::default(),
            output_news: Default::default(),
            startup_import: Vec::new(),
            applied: HashMap::new(),
            sync_due: true,
            history: Default::default(),
            restoring: HashMap::new(),
            files_checked: None,
            synced_freshness: Vec::new(),
            warned: BTreeSet::new(),
            in_project: true,
            active: None,
            ribbon: Default::default(),
            search: None,
            placing: None,
        };
        if app.settings.updates.check_at_startup && app.updater.installation().checks_at_startup() {
            app.updater.check(&cc.egui_ctx, false);
        }
        // A project on the command line opens; other files, such as a STEP or input file,
        // are imported into a new project's first analysis.
        let paths: Vec<PathBuf> = std::env::args_os().skip(1).map(PathBuf::from).collect();
        let (projects, files): (Vec<PathBuf>, Vec<PathBuf>) = paths
            .into_iter()
            .partition(|p| crate::model::is_project_file(p));
        if let Some(project) = projects.into_iter().next() {
            app.open(&cc.egui_ctx, project);
        } else if !files.is_empty() {
            let name = (files[0].file_stem())
                .map_or_else(|| "Analysis-1".into(), |s| s.to_string_lossy().into_owned());
            let properties = app.settings.new_model.clone();
            let id = app.add_analysis(&name, egui::pos2(40.0, 40.0), properties);
            app.startup_import = files;
            app.open_window(&cc.egui_ctx, id, None);
        } else {
            // The last project is not loaded again: Open Recent lists it.
            app.add_first_analysis();
        }
        Ok(app)
    }

    fn node(&mut self, id: u64) -> Option<&mut Node> {
        self.nodes.iter_mut().find(|n| n.id == id)
    }

    fn project_name(&self) -> String {
        (self.path.as_ref())
            .and_then(|p| p.file_stem())
            .map_or_else(|| "Untitled".into(), |s| s.to_string_lossy().into_owned())
    }

    /// An id no block had: those of deleted blocks stay theirs, for undo.
    fn next_id(&self) -> u64 {
        let highest = self.nodes.iter().map(|n| n.id).max().unwrap_or(0);
        highest.max(self.history.highest_id) + 1
    }

    /// Adds an empty analysis with these model properties; returns its id.
    fn add_analysis(
        &mut self,
        name: &str,
        position: egui::Pos2,
        properties: ModelProperties,
    ) -> u64 {
        let id = self.next_id();
        let name = block_name(name);
        let mut data = ProjectAnalysis {
            id,
            name: name.clone(),
            ..Default::default()
        };
        data.model.properties = properties;
        self.nodes.push(Node::analysis(id, name, data));
        self.graph.insert_node(position, id);
        id
    }

    /// Adds an empty materials block; returns its id.
    fn add_materials(&mut self, position: egui::Pos2) -> u64 {
        self.add_collection(position, Collection::Materials)
    }

    /// Adds an empty materials or cross-sections block; returns its id.
    fn add_collection(&mut self, position: egui::Pos2, collection: Collection) -> u64 {
        let id = self.next_id();
        let name = block_name(collection.title());
        let block = Box::new(CollectionBlock {
            model: LinkedModel::default(),
            units: self.settings.new_model.units,
        });
        let block = match collection {
            Collection::Materials => Block::Materials(block),
            Collection::CrossSections => Block::CrossSections(block),
        };
        self.nodes.push(Node::new(id, name.clone(), block));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: double-click it to add {}, then drag them to analyses.",
            collection.title().to_lowercase()
        ));
        id
    }

    /// Adds a mesh block meshing `primitive` and makes its mesh; returns its id.
    fn add_mesh(&mut self, position: egui::Pos2, primitive: MeshPrimitive) -> u64 {
        let id = self.next_id();
        let name = block_name(primitive.label());
        let space = match &primitive {
            MeshPrimitive::Box { .. } => ModelSpace::ThreeD,
            MeshPrimitive::Rectangle { .. } => match self.settings.new_model.space {
                ModelSpace::ThreeD => ModelSpace::PlaneStress,
                space => space,
            },
        };
        let units = self.settings.new_model.units;
        let mesh = Default::default();
        let mut node = Node::mesh_block(id, name.clone(), space, units, primitive, mesh, None);
        let made = node.generate();
        self.nodes.push(node);
        self.graph.insert_node(position, id);
        self.output.push(match made {
            Ok(count) => format!(
                "{name} created: {}, {count} elements. Double-click it to change its size, then \
                 drag its mesh to analyses.",
                space.label()
            ),
            Err(why) => format!("{name} created without a mesh: {why}"),
        });
        id
    }

    /// Adds a refine mesh block with the factor 1; returns its id.
    /// Adds a CAD operation block, a fillet of 1 mm; returns its id.
    fn add_cad_operation(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = block_name("CAD Operation");
        let cad = CadBlock::new(Default::default(), UnitSystem::default(), None, None);
        self.nodes.push(Node::new(
            id,
            name.clone(),
            Block::CadOperation(Box::new(cad)),
        ));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: drag geometry to its Geometry row, double-click it to pick the \
             edges, and drag its geometry on to analyses."
        ));
        id
    }

    fn add_refine_mesh(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = block_name("Refine Mesh");
        self.nodes.push(Node::refine_block(
            id,
            name.clone(),
            1.0,
            Default::default(),
            None,
        ));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: drag the mesh of a mesh block or of an analysis to its Mesh row, \
             and its mesh on to analyses."
        ));
        id
    }

    /// Adds a parameters block with one length; returns its id.
    fn add_parameters(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = block_name("Parameters");
        let block = ParametersBlock {
            units: self.settings.new_model.units,
            parameters: vec![Parameter::new("Length", ValueKind::LENGTH, 0.1)],
        };
        self.nodes
            .push(Node::new(id, name.clone(), Block::Parameters(block)));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: drag the pin of a parameter to a parameter of a mesh block or to an \
             exposed parameter of an analysis; double-click it to add, rename or remove \
             parameters."
        ));
        id
    }

    /// Adds a note, whose text takes the keyboard at once.
    fn add_note(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = block_name("Note");
        self.nodes
            .push(Node::new(id, name, Block::Note(String::new())));
        self.graph.insert_node(position, id);
        self.focus_note = Some(id);
        id
    }

    /// Opens the editor of a parameters block.
    fn edit_parameters(&mut self, id: u64) {
        let Some(node) = self.nodes.iter().find(|n| n.id == id) else {
            return;
        };
        let Some(block) = node.as_parameters() else {
            return;
        };
        self.parameter_set = Some(ParameterSetEditor::new(
            id,
            &node.name,
            &block.parameters,
            block.units,
        ));
    }

    fn parameter_set_window(&mut self, ctx: &egui::Context) {
        let Some(editor) = &mut self.parameter_set else {
            return;
        };
        let id = editor.id;
        match editor.show(ctx) {
            ParameterSetResult::Open => {}
            ParameterSetResult::Cancel => self.parameter_set = None,
            ParameterSetResult::Ok(rows, units) => {
                self.checkpoint(&[]);
                self.parameter_set = None;
                // Links follow a renamed parameter; those of a removed one go.
                let kept: Vec<(String, String)> = (rows.iter())
                    .filter_map(|(old, p)| Some((old.clone()?, p.name.clone())))
                    .collect();
                self.links.retain_mut(|link| {
                    if link.source != id || !links::is_value(link) {
                        return true;
                    }
                    let Some(name) = link.item.get(1) else {
                        return false;
                    };
                    match kept.iter().find(|(old, _)| old == name) {
                        Some((_, new)) => {
                            link.item[1] = new.clone();
                            true
                        }
                        None => false,
                    }
                });
                if let Some(block) = self.node(id).and_then(Node::as_parameters_mut) {
                    block.parameters = rows.into_iter().map(|(_, p)| p).collect();
                    block.units = units;
                }
                self.sync_due = true;
            }
        }
    }

    /// Adds a measure block of the largest von Mises stress; returns its id.
    fn add_measure(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = block_name("Measure");
        let block = MeasureBlock {
            units: self.settings.new_model.units,
            kind: MeasureKind::default(),
            value: None,
            evaluated: None,
            error: None,
        };
        self.nodes
            .push(Node::new(id, name.clone(), Block::Measure(block)));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: drag the results of a step to its Results row; double-click it \
             to choose what it measures."
        ));
        id
    }

    /// Opens the editor of a measure block, with the node sets of the mesh it measures on.
    fn edit_measure(&mut self, id: u64) {
        let Some(node) = self.nodes.iter().find(|n| n.id == id) else {
            return;
        };
        let Some(measure) = node.as_measure() else {
            return;
        };
        let units = measure.units;
        let node_sets = self
            .measure_link(id)
            .and_then(|l| self.nodes.iter().find(|n| n.id == l.source))
            .and_then(Node::link_data)
            .map(|d| measure_regions(&d).into_keys().collect())
            .unwrap_or_default();
        let offered = (self.measured(id)).map(|(source, step, bound)| {
            let dir = self.measure_dir(id, source, bound, "-fields");
            crate::measure::read_offered(bound.clone(), step, dir)
        });
        self.measure_editor = Some(MeasureEditor::new(
            id,
            &node.name,
            measure.kind.clone(),
            units,
            node_sets,
            offered,
        ));
    }

    /// Where a measure block restores the results it reads: one folder per results, with
    /// `suffix` for a reader that must not share it.
    fn measure_dir(
        &self,
        id: u64,
        source: &Node,
        bound: &plx_io::project::BoundResults,
        suffix: &str,
    ) -> std::path::PathBuf {
        (self.settings.solver.work_dir())
            .join(crate::analysis::job_name(&self.project_name()))
            .join(crate::analysis::job_dir(id))
            .join(format!(
                "{}-{:016x}{suffix}",
                crate::analysis::job_dir(source.id),
                bound.input.unwrap_or_default()
            ))
    }

    fn measure_window(&mut self, ctx: &egui::Context) {
        let Some(editor) = &mut self.measure_editor else {
            return;
        };
        let id = editor.id;
        match editor.show(ctx) {
            MeasureEditorResult::Open => {}
            MeasureEditorResult::Cancel => self.measure_editor = None,
            MeasureEditorResult::Ok(kind, units) => {
                self.checkpoint(&[]);
                self.measure_editor = None;
                let at_point = kind.point().is_some();
                if let Some(measure) = self.node(id).and_then(Node::as_measure_mut) {
                    measure.kind = kind;
                    measure.units = units;
                }
                // Without a point the block has no coordinate rows to link to.
                if !at_point {
                    self.links.retain(|l| {
                        l.target != id
                            || !matches!(links::slot(l), [cell, _] if cell == links::PARAMETERS)
                    });
                }
                self.sync_due = true;
            }
        }
    }

    /// The link of the results a measure block measures; value links to its coordinates
    /// are others.
    fn measure_link(&self, id: u64) -> Option<&Link> {
        (self.links.iter())
            .find(|l| l.target == id && matches!(links::slot(l), [cell] if cell == links::MEASURE))
    }

    /// The results a measure block measures: the analysis, the step's number for CalculiX
    /// and the results it keeps; `None` without a link or results.
    fn measured(&self, id: u64) -> Option<(&Node, u32, &plx_io::project::BoundResults)> {
        let link = self.measure_link(id)?;
        let source = self.nodes.iter().find(|n| n.id == link.source)?;
        let step = links::global_step_number(source.link_data()?.fe, link.item.get(1)?)?;
        Some((source, step, source.bound_results()?))
    }

    /// The fingerprint of what a measure block computes its value from: the results it
    /// measures and its definition; `None` while it has no results to measure.
    fn measure_key(&self, id: u64) -> Option<u64> {
        let node = self.nodes.iter().find(|n| n.id == id)?;
        let kind = &node.as_measure()?.kind;
        let (_, step, bound) = self.measured(id)?;
        let mut print = plx_mesh::Fnv::new();
        print.add(&bound.input.unwrap_or_default().to_le_bytes());
        for file in &bound.files {
            print.add(file.name.as_bytes());
            print.add(&file.size.to_le_bytes());
        }
        print.add(&step.to_le_bytes());
        print.add(format!("{kind:?}").as_bytes());
        Some(print.finish())
    }

    /// The state of the value of a measure block: up to date once computed from the results
    /// as they are, and those are up to date.
    fn measure_state(&self, id: u64) -> CellState {
        let Some(node) = self.nodes.iter().find(|n| n.id == id) else {
            return CellState::Missing;
        };
        let Some(measure) = node.as_measure() else {
            return CellState::Missing;
        };
        let Some(key) = self.measure_key(id) else {
            return CellState::Missing;
        };
        let fresh = self
            .measure_link(id)
            .zip(self.measured(id))
            .is_some_and(|(link, (source, _, _))| !source.out_of_date_for(link));
        match measure.value {
            _ if measure.evaluated != Some(key) => CellState::OutOfDate,
            None => CellState::Missing,
            Some(_) if fresh => CellState::UpToDate,
            Some(_) => CellState::OutOfDate,
        }
    }

    /// Takes the values the workers of measure blocks computed; returns whether one came.
    fn poll_measures(&mut self) -> bool {
        let done: Vec<(u64, u64, Result<f64, String>)> = (self.measuring.iter())
            .filter_map(|(&id, (key, receiver))| match receiver.try_recv() {
                Ok(result) => Some((id, *key, result)),
                Err(std::sync::mpsc::TryRecvError::Empty) => None,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    Some((id, *key, Err("the computation stopped".into())))
                }
            })
            .collect();
        let came = !done.is_empty();
        for (id, key, result) in done {
            self.measuring.remove(&id);
            let Some(node) = self.node(id) else {
                continue;
            };
            let name = node.name.clone();
            let Some(measure) = node.as_measure_mut() else {
                continue;
            };
            let units = measure.units;
            measure.evaluated = Some(key);
            let line = match result {
                Ok(value) => {
                    measure.value = Some(value);
                    measure.error = None;
                    format!(
                        "{name}: {} = {}",
                        measure.kind.describe(),
                        crate::measure::shown(&measure.kind, value, units)
                    )
                }
                Err(why) => {
                    measure.value = None;
                    let line = format!("{name} has no value: {why}.");
                    measure.error = Some(why);
                    line
                }
            };
            self.output.push(line);
        }
        came
    }

    /// Computes the values of measure blocks whose results or definition changed, on worker
    /// threads; [`ProjectApp::poll_measures`] takes them.
    fn evaluate_measures(&mut self) {
        let measures: Vec<u64> = (self.nodes.iter())
            .filter(|n| n.kind() == Kind::Measure)
            .map(|n| n.id)
            .collect();
        for id in measures {
            let Some(key) = self.measure_key(id) else {
                continue;
            };
            let evaluated = (self.nodes.iter().find(|n| n.id == id))
                .and_then(Node::as_measure)
                .and_then(|m| m.evaluated);
            if evaluated == Some(key) || self.measuring.get(&id).is_some_and(|(k, _)| *k == key) {
                continue;
            }
            let Some((source, step, bound)) = self.measured(id) else {
                continue;
            };
            let Some((kind, units)) = (self.nodes.iter().find(|n| n.id == id))
                .and_then(Node::as_measure)
                .map(|m| (m.kind.clone(), m.units))
            else {
                continue;
            };
            let node_sets = (source.link_data())
                .map(|d| measure_regions(&d))
                .unwrap_or_default();
            let dir = self.measure_dir(id, source, bound, "");
            let job = crate::measure::Job {
                kind,
                step,
                results: bound.clone(),
                node_sets,
                units,
                dir,
            };
            self.measuring.insert(id, (key, crate::measure::start(job)));
        }
    }

    /// Adds a parameter study without columns; returns its id.
    fn add_study(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = block_name("Parameter Study");
        let block = StudyBlock {
            units: self.settings.new_model.units,
            table: StudyTable::default(),
            body: None,
            active: None,
            report: None,
        };
        self.nodes
            .push(Node::new(id, name.clone(), Block::Study(Box::new(block))));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: drag its Inputs pin to the numbers it varies, such as exposed \
             parameters of analyses, and the value of a Measure block to its Outputs pin; \
             double-click it for its table of design points."
        ));
        id
    }

    fn edit_study(&mut self, id: u64) {
        let Some(node) = self.nodes.iter().find(|n| n.id == id) else {
            return;
        };
        if node.as_study().is_some() {
            self.study_editor = Some(StudyEditor::new(id, &node.name));
        }
    }

    fn study_window(&mut self, ctx: &egui::Context) {
        let Some(id) = self.study_editor.as_ref().map(|e| e.id) else {
            return;
        };
        let view = self.study_view(id);
        let Some(editor) = &mut self.study_editor else {
            return;
        };
        for edit in editor.show(ctx, &view) {
            self.edit_study_table(id, edit);
        }
    }

    /// Applies a change of the editor of a study; its links follow renamed and removed
    /// columns.
    fn edit_study_table(&mut self, id: u64, edit: StudyEdit) {
        match edit {
            StudyEdit::Close => {
                self.study_editor = None;
                return;
            }
            StudyEdit::Run => return self.start_study(id),
            StudyEdit::Reset => return self.reset_study(id),
            _ if self.busy() => return,
            _ => {}
        }
        self.checkpoint(&[]);
        // A renamed column, input or not, with its old and new name; a removed one.
        let mut renamed = None;
        let mut removed = None;
        let mut added = None;
        let Some(study) = self.node(id).and_then(Node::as_study_mut) else {
            return;
        };
        let table = &mut study.table;
        match edit {
            StudyEdit::Units(units) => study.units = units,
            StudyEdit::SetInput {
                point,
                column,
                value,
            } => table.set_input(point, column, value),
            StudyEdit::AddPoint => table.add_point(),
            StudyEdit::AddRows { rows, replace } => added = Some(table.add_rows(rows, replace)),
            StudyEdit::Duplicate(point) => table.duplicate(point),
            StudyEdit::RemovePoint(point) => table.remove_point(point),
            StudyEdit::SetCurrent(point) if point < table.points.len() => table.current = point,
            StudyEdit::RenameInput(column, ref name)
            | StudyEdit::RenameOutput(column, ref name) => {
                let input = matches!(edit, StudyEdit::RenameInput(..));
                let name = name.clone();
                let new = table.unique_name(&name);
                let columns = if input {
                    &mut table.inputs
                } else {
                    &mut table.outputs
                };
                if let Some(column) = columns.get_mut(column) {
                    let old = std::mem::replace(&mut column.name, new.clone());
                    renamed = Some((input, old, new));
                }
            }
            StudyEdit::RemoveInput(column) => {
                if let Some(c) = table.inputs.get(column) {
                    removed = Some((true, c.name.clone()));
                    table.remove_input(column);
                }
            }
            StudyEdit::RemoveOutput(column) => {
                if let Some(c) = table.outputs.get(column) {
                    removed = Some((false, c.name.clone()));
                    table.remove_output(column);
                }
            }
            _ => {}
        }
        let column_link = |link: &Link, input: bool, name: &str| {
            if input {
                link.source == id
                    && matches!(link.item.as_slice(), [c, n] if c == links::LOOP && n == name)
            } else {
                link.target == id
                    && matches!(links::slot(link), [c, n] if c == links::OUTPUTS && n == name)
            }
        };
        if let Some((input, old, new)) = renamed {
            for link in &mut self.links {
                if column_link(link, input, &old) {
                    if input {
                        link.item[1] = new.clone();
                    } else {
                        link.slot[1] = new.clone();
                    }
                }
            }
        }
        if let Some((input, name)) = removed {
            self.links.retain(|l| !column_link(l, input, &name));
        }
        if let Some(added) = added {
            let name = self.name_of(id);
            self.output.push(format!("{name}: {added} points added."));
        }
        self.sync_due = true;
    }

    /// What the editor of a study shows: its columns and points as text in the units of
    /// their sources.
    fn study_view(&self, id: u64) -> StudyView {
        let Some(study) = (self.nodes.iter().find(|n| n.id == id)).and_then(Node::as_study) else {
            return StudyView::default();
        };
        let table = &study.table;
        let running = self.looping.as_ref().is_some_and(|run| run.id == id);
        StudyView {
            units: study.units,
            inputs: (table.inputs.iter())
                .map(|c| (c.name.clone(), c.kind))
                .collect(),
            outputs: (table.outputs.iter())
                .map(|c| (c.name.clone(), self.output_shown(id, &c.name, 0.0).1))
                .collect(),
            points: (table.points.iter())
                .map(|point| PointView {
                    inputs: point.inputs.clone(),
                    outputs: (table.outputs.iter().enumerate())
                        .map(|(i, column)| {
                            (point.outputs.get(i).copied().flatten())
                                .map(|v| self.output_shown(id, &column.name, v).0)
                        })
                        .collect(),
                    state: point.state(study.body, table.outputs.len()),
                    seconds: (point.run != PointRun::NotSolved).then_some(point.seconds),
                    elements: point.elements,
                })
                .collect(),
            current: table.current,
            running: study.active.filter(|_| running),
            busy: self.busy(),
        }
    }

    /// The link that gives the output column `name` of a study.
    fn output_link(&self, id: u64, name: &str) -> Option<&Link> {
        (self.links.iter()).find(|l| {
            l.target == id && matches!(links::slot(l), [c, n] if c == links::OUTPUTS && n == name)
        })
    }

    /// The value the output column `name` of a study reads now: that of an up-to-date
    /// Measure block, or a number of another block.
    fn output_value(&self, id: u64, name: &str) -> Option<f64> {
        let link = self.output_link(id, name)?;
        let source = self.nodes.iter().find(|n| n.id == link.source)?;
        match source.as_measure() {
            Some(measure) => {
                (self.measure_state(source.id) == CellState::UpToDate).then_some(measure.value)?
            }
            None => self.value_of(source.id, &link.item),
        }
    }

    /// An output of a study as text, in the units of its source.
    fn shown_output(&self, id: u64, name: &str, value: f64) -> String {
        let (value, unit) = self.output_shown(id, name, value);
        format!("{} {unit}", crate::results::format_value(value as f32))
            .trim()
            .to_owned()
    }

    /// An output of a study as its source shows it: the number and its unit.
    fn output_shown(&self, id: u64, name: &str, value: f64) -> (f64, String) {
        let source = (self.output_link(id, name))
            .and_then(|link| Some((self.nodes.iter().find(|n| n.id == link.source)?, link)));
        let quantity = |quantity, units: UnitSystem| {
            (
                units.shown(value, quantity),
                units.unit(quantity).to_owned(),
            )
        };
        match source {
            Some((node, _)) if node.as_measure().is_some() => {
                let measure = node.as_measure().expect("a measure");
                let units = measure.units;
                match measure.kind.field() {
                    ("FREQ", _) => quantity(plx_model::Quantity::Frequency, units),
                    (field, component) => (
                        crate::results::shown_field_value(value, field, component, units),
                        crate::results::field_unit(field, component, units).unwrap_or_default(),
                    ),
                }
            }
            Some((node, link)) => {
                match value_kind(&self.nodes, node.id, &link.item).and_then(ValueKind::quantity) {
                    Some(q) => quantity(q, node.units()),
                    None => (value, String::new()),
                }
            }
            None => (value, String::new()),
        }
    }

    /// The feedback links of a study: where its inputs go.
    fn feedback_of(&self, id: u64) -> Vec<&Link> {
        (self.links.iter())
            .filter(|l| l.source == id && links::is_feedback(l))
            .collect()
    }

    /// The blocks a study runs: those after the numbers its inputs go to and before the
    /// blocks its outputs come from, in the order of the links.
    fn study_body(&self, id: u64) -> Vec<u64> {
        let mut after: BTreeSet<u64> = self.feedback_of(id).iter().map(|l| l.target).collect();
        let mut queue: Vec<u64> = after.iter().copied().collect();
        while let Some(at) = queue.pop() {
            for link in self
                .links
                .iter()
                .filter(|l| l.source == at && !links::is_feedback(l))
            {
                if link.target != id && after.insert(link.target) {
                    queue.push(link.target);
                }
            }
        }
        let mut before: BTreeSet<u64> = (self.links.iter())
            .filter(|l| l.target == id && !links::is_feedback(l))
            .map(|l| l.source)
            .collect();
        let mut queue: Vec<u64> = before.iter().copied().collect();
        while let Some(at) = queue.pop() {
            for link in self
                .links
                .iter()
                .filter(|l| l.target == at && !links::is_feedback(l))
            {
                if link.source != id && before.insert(link.source) {
                    queue.push(link.source);
                }
            }
        }
        let ids: Vec<u64> = self.nodes.iter().map(|n| n.id).collect();
        links::order(&ids, &self.links)
            .into_iter()
            .filter(|n| after.contains(n) && before.contains(n))
            .collect()
    }

    /// The fingerprint of the blocks a study runs without the values it gives them, which
    /// change from point to point: a point solved at another fingerprint is out of date.
    /// `None` while it runs no block, or one is loading.
    fn study_body_key(&self, id: u64) -> Option<u64> {
        let body = self.study_body(id);
        if body.is_empty() {
            return None;
        }
        // The rows that take their number from the study or from a block it runs.
        let driven: Vec<(u64, &[String])> = (self.links.iter())
            .filter(|l| links::is_value(l) && (l.source == id || body.contains(&l.source)))
            .map(|l| (l.target, links::slot(l)))
            .collect();
        let mut print = plx_mesh::Fnv::new();
        for &at in &body {
            let node = self.nodes.iter().find(|n| n.id == at)?;
            let driven = |cell: &str, key: &str| {
                (driven.iter()).any(|(target, slot)| {
                    *target == at && matches!(slot, [c, k] if c == cell && k == key)
                })
            };
            print.add(&at.to_le_bytes());
            match &node.block {
                Block::Analysis(_) | Block::Materials(_) | Block::CrossSections(_) => {
                    let data = node.link_data()?;
                    // A mesh the study makes anew counts by the block that makes it.
                    let remeshed = (self.links.iter()).any(|l| {
                        l.target == at
                            && body.contains(&l.source)
                            && (l.item.first())
                                .is_some_and(|c| c == links::MESH || c == links::GEOMETRY)
                    });
                    let mesh = if node.collection().is_some() {
                        StudyMesh::None
                    } else if remeshed {
                        StudyMesh::Remeshed
                    } else {
                        StudyMesh::Own
                    };
                    let exposed = |key: u32| driven(links::PARAMETERS, &key.to_string());
                    print.add(&study_analysis_print(data, mesh, exposed).to_le_bytes());
                }
                Block::Mesh(mesh) => {
                    let mut primitive = mesh.primitive.clone();
                    for param in mesh.primitive.params() {
                        if driven(links::PARAMETERS, param.key) {
                            primitive.set_value(param.key, 0.0);
                        }
                    }
                    print.add(format!("{primitive:?}").as_bytes());
                }
                Block::RefineMesh(refine) if !driven(links::PARAMETERS, REFINEMENT) => {
                    print.add(&refine.factor.to_le_bytes());
                }
                Block::DeformedMesh(deformed) => {
                    if !driven(links::PARAMETERS, DEFORM_FACTOR) {
                        print.add(&deformed.factor.to_le_bytes());
                    }
                    if !driven(links::PARAMETERS, DEFORM_INCREMENT) {
                        print.add(&deformed.increment.to_le_bytes());
                    }
                }
                Block::CadOperation(cad) => {
                    cad.body_print(driven(links::PARAMETERS, CAD_LENGTH), &mut print);
                }
                Block::Parameters(parameters) => {
                    for parameter in &parameters.parameters {
                        print.add(format!("{}{:?}", parameter.name, parameter.kind).as_bytes());
                        if !driven(links::VALUE, &parameter.name) {
                            print.add(&parameter.value.to_le_bytes());
                        }
                    }
                }
                Block::Measure(measure) => {
                    let mut kind = measure.kind.clone();
                    for axis in POINT_AXES {
                        if driven(links::PARAMETERS, axis) {
                            kind.set_coordinate(axis, 0.0);
                        }
                    }
                    print.add(format!("{kind:?}").as_bytes());
                }
                Block::Math(number) | Block::Switch(number) => {
                    print.add(
                        format!(
                            "{:?}{:?}{:?}",
                            number.operation, number.kind, number.operands
                        )
                        .as_bytes(),
                    );
                    let switch = node.kind() == Kind::Switch;
                    for (key, _, _, value) in number.inputs(switch) {
                        if !driven(links::PARAMETERS, &key) {
                            print.add(&value.to_le_bytes());
                        }
                    }
                }
                _ => {}
            }
        }
        Some(print.finish())
    }

    /// Notes the fingerprint of the blocks of every study, once something changed.
    fn refresh_study_bodies(&mut self) {
        let ids: Vec<u64> = (self.nodes.iter())
            .filter(|n| n.kind() == Kind::Study)
            .map(|n| n.id)
            .collect();
        for id in ids {
            // A running study keeps the fingerprint it started with: between its points the
            // blocks stand at other values, and the picks that follow the shapes with them.
            if self.looping.as_ref().is_some_and(|run| run.id == id) {
                continue;
            }
            let body = self.study_body_key(id);
            if let Some(study) = self.node(id).and_then(Node::as_study_mut) {
                study.body = body;
            }
        }
    }

    /// The elements of the largest mesh of the analyses a study runs.
    fn study_elements(&self, id: u64) -> Option<u64> {
        (self.study_body(id).into_iter())
            .filter_map(|at| self.nodes.iter().find(|n| n.id == at))
            .filter(|n| n.kind() == Kind::Analysis)
            .filter_map(Node::link_data)
            .map(|d| d.mesh.element_count() as u64)
            .max()
    }

    /// Whether a study has what it needs to run: inputs that go somewhere, outputs that
    /// come from somewhere, and points.
    fn study_runnable(&self, id: u64) -> bool {
        let has_points = (self.nodes.iter().find(|n| n.id == id))
            .and_then(Node::as_study)
            .is_some_and(|s| !s.table.points.is_empty());
        has_points
            && !self.feedback_of(id).is_empty()
            && (self.links.iter()).any(|l| {
                l.target == id && links::slot(l).first().is_some_and(|c| c == links::OUTPUTS)
            })
    }

    /// The state of a study: up to date while all its points are, missing without columns.
    fn study_state(&self, id: u64) -> CellState {
        let Some(study) = (self.nodes.iter().find(|n| n.id == id)).and_then(Node::as_study) else {
            return CellState::Missing;
        };
        if self.looping.as_ref().is_some_and(|run| run.id == id) {
            return CellState::Running;
        }
        if !self.study_runnable(id) {
            return CellState::Missing;
        }
        let (solved, _, all) = study.table.summary(study.body);
        if solved == all {
            CellState::UpToDate
        } else {
            CellState::OutOfDate
        }
    }

    /// The rows of a study: the Outputs it reads and their columns, the Inputs it gives and
    /// theirs, and how far its points are.
    fn study_rows(
        &self,
        node: &Node,
        incoming: &HashMap<u64, Vec<(Vec<String>, String)>>,
    ) -> Vec<Row> {
        let Some(study) = node.as_study() else {
            return Vec::new();
        };
        let table = &study.table;
        let row = |path: Vec<String>, text: String, depth, icon, tip: &str| Row {
            path,
            text,
            depth,
            icon,
            state: None,
            expander: None,
            input: false,
            output: false,
            linked_from: None,
            collection: false,
            partial: false,
            widget: None,
            tip: tip.to_owned(),
        };
        let current = table.current();
        let mut rows = vec![Row {
            input: true,
            collection: true,
            ..row(
                vec![links::OUTPUTS.to_owned()],
                format!("Outputs ({})", table.outputs.len()),
                0,
                NodeIcon::Results,
                "The numbers it reads once the points are solved: drop the value of a \
                 Measure block here to add a column.",
            )
        }];
        for (i, column) in table.outputs.iter().enumerate() {
            let path = vec![links::OUTPUTS.to_owned(), column.name.clone()];
            let linked_from = (incoming.get(&node.id).into_iter().flatten())
                .find(|(slot, _)| *slot == path)
                .map(|(_, source)| source.clone());
            let value = (current.and_then(|p| p.outputs.get(i).copied().flatten()))
                .map_or_else(String::new, |v| {
                    format!("  {}", self.shown_output(node.id, &column.name, v))
                });
            rows.push(Row {
                input: true,
                linked_from,
                ..row(
                    path,
                    format!("{}{value}", column.name),
                    1,
                    NodeIcon::Results,
                    "An output column; its value at the current point.",
                )
            });
        }
        rows.push(Row {
            output: true,
            collection: true,
            ..row(
                vec![links::LOOP.to_owned()],
                format!("Inputs ({})", table.inputs.len()),
                0,
                NodeIcon::Setup,
                "The numbers it varies: drag this pin to an exposed parameter of an analysis, \
                 a number of a mesh block or the Factor of a Refine Mesh block to add a \
                 column. The wires are dashed, as they lead back.",
            )
        });
        for (i, column) in table.inputs.iter().enumerate() {
            let value =
                (current.and_then(|p| p.inputs.get(i).copied())).map_or_else(String::new, |v| {
                    match column.kind {
                        Some(kind) => format!("  {}", crate::study::shown(kind, v, study.units)),
                        None => format!("  {}", crate::results::format_value(v as f32)),
                    }
                });
            rows.push(Row {
                output: true,
                ..row(
                    vec![links::LOOP.to_owned(), column.name.clone()],
                    format!("{}{value}", column.name),
                    1,
                    NodeIcon::Setup,
                    "An input column; its value at the current point. Drag its pin to more \
                     numbers to give them the same value.",
                )
            });
        }
        let (solved, failed, all) = table.summary(study.body);
        let running = (self.looping.as_ref())
            .filter(|run| run.id == node.id)
            .and(study.active);
        let text = match running {
            Some(point) => format!("Solving DP {point} …"),
            None if failed > 0 => format!("{all} points, {solved} solved, {failed} failed"),
            None => format!("{all} points, {solved} solved"),
        };
        let report = study
            .report
            .clone()
            .unwrap_or_else(|| "Double-click the block for its table of design points.".into());
        rows.push(Row {
            state: Some(self.study_state(node.id)),
            ..row(Vec::new(), text, 0, NodeIcon::Solution, &report)
        });
        rows
    }

    /// Update Project: runs the studies that are not up to date, each of which updates the
    /// project for every point, or else updates the project.
    fn update_all(&mut self) {
        self.update_scope = None;
        self.loops_due = (self.nodes.iter())
            .filter(|n| n.kind() == Kind::Study)
            .map(|n| n.id)
            .filter(|&id| self.study_state(id) == CellState::OutOfDate)
            .collect();
        match self.loops_due.pop_front() {
            Some(id) => self.start_study(id),
            None => self.update_project(),
        }
    }

    /// Starts solving the points of a study that are not up to date.
    fn start_study(&mut self, id: u64) {
        if self.looping.is_some() {
            return;
        }
        let name = self.name_of(id);
        if !self.study_runnable(id) {
            self.output.push(format!(
                "{name} cannot run: drag its Inputs pin to the numbers it varies and the \
                 value of a Measure block to its Outputs pin."
            ));
            return;
        }
        let body = self.study_body_key(id);
        let Some(study) = self.node(id).and_then(Node::as_study_mut) else {
            return;
        };
        study.body = body;
        let points = study.table.to_solve(body);
        if points.is_empty() {
            self.output
                .push(format!("{name}: all points are up to date."));
            return self.next_study();
        }
        study.report = None;
        self.output
            .push(format!("{name}: solving {} design points", points.len()));
        // A study updates the whole project for every point.
        self.update_scope = None;
        self.looping = Some(StudyRun {
            id,
            body,
            points: points.into(),
            current: None,
            updating: false,
            solved: 0,
            failed: 0,
        });
    }

    /// Forgets what the points of a study gave; the inputs stay.
    fn reset_study(&mut self, id: u64) {
        if self.looping.as_ref().is_some_and(|run| run.id == id) {
            return;
        }
        self.checkpoint(&[]);
        if let Some(study) = self.node(id).and_then(Node::as_study_mut) {
            study.table.reset();
            study.report = None;
        }
        self.sync_due = true;
    }

    /// Works through a study: gives the inputs of the next point, has the project updated,
    /// and reads the outputs once all is done.
    fn run_study(&mut self) {
        let Some(run) = &self.looping else {
            return;
        };
        let id = run.id;
        match run.current {
            None => self.next_point(id),
            Some(_) if !run.updating => {
                // The inputs went to their numbers; then the update.
                if !self.sync_due {
                    if let Some(run) = &mut self.looping {
                        run.updating = true;
                    }
                    self.update_project();
                }
            }
            Some((point, started)) => {
                let idle = !self.update_after_sync
                    && !self.sync_due
                    && self.update_queue.is_empty()
                    && self.updating.is_empty()
                    && self.measuring.is_empty()
                    && !self.deforming()
                    && !self.restoring.values().any(Option::is_some);
                if !idle {
                    return;
                }
                let names: Vec<String> = (self.nodes.iter().find(|n| n.id == id))
                    .and_then(Node::as_study)
                    .map(|s| s.table.outputs.iter().map(|c| c.name.clone()).collect())
                    .unwrap_or_default();
                let outputs: Vec<Option<f64>> =
                    names.iter().map(|n| self.output_value(id, n)).collect();
                let shown: Vec<String> = (names.iter().zip(&outputs))
                    .map(|(name, value)| match value {
                        Some(v) => format!("{name} {}", self.shown_output(id, name, *v)),
                        None => format!("{name} missing"),
                    })
                    .collect();
                let failed = outputs.iter().any(Option::is_none);
                // A study started while a block was loading takes the fingerprint now.
                let body = match self.looping.as_ref().and_then(|run| run.body) {
                    Some(body) => Some(body),
                    None => self.study_body_key(id),
                };
                let elements = self.study_elements(id);
                self.output.push(format!(
                    "{}: DP {point}: {}",
                    self.name_of(id),
                    shown.join(", ")
                ));
                if let Some(study) = self.node(id).and_then(Node::as_study_mut)
                    && let Some(row) = study.table.points.get_mut(point)
                {
                    row.outputs = outputs;
                    row.run = if failed {
                        PointRun::Failed
                    } else {
                        PointRun::Solved
                    };
                    row.seconds = started.elapsed().as_secs_f64();
                    row.elements = elements;
                    row.fingerprint = body;
                    study.body = body;
                }
                if let Some(run) = &mut self.looping {
                    run.body = body;
                    run.current = None;
                    run.updating = false;
                    run.solved += 1;
                    run.failed += usize::from(failed);
                }
            }
        }
    }

    /// Gives the inputs of the next point of the study, or ends it.
    fn next_point(&mut self, id: u64) {
        let next = self.looping.as_mut().and_then(|run| run.points.pop_front());
        let Some(point) = next else {
            return self.finish_study(id);
        };
        let Some(study) = self.node(id).and_then(Node::as_study_mut) else {
            self.looping = None;
            return;
        };
        if point >= study.table.points.len() {
            return;
        }
        study.active = Some(point);
        if let Some(run) = &mut self.looping {
            run.current = Some((point, Instant::now()));
            run.updating = false;
        }
        self.sync_due = true;
    }

    /// Ends a study: the blocks stand at its current point again, and the next study of
    /// Update Project starts.
    fn finish_study(&mut self, id: u64) {
        let Some(run) = self.looping.take() else {
            return;
        };
        let name = self.name_of(id);
        let report = match run.failed {
            0 => format!("{} points solved.", run.solved),
            failed => format!(
                "{} points solved, {failed} of them failed: the solution monitor and the \
                 output tell why.",
                run.solved
            ),
        };
        self.output.push(format!("{name}: {report}"));
        if let Some(study) = self.node(id).and_then(Node::as_study_mut) {
            study.active = None;
            study.report = Some(report);
        }
        self.sync_due = true;
        self.next_study();
    }

    /// Starts the next study Update Project runs, or the files of the export blocks once
    /// none is left.
    fn next_study(&mut self) {
        if let Some(next) = self.loops_due.pop_front() {
            self.start_study(next);
        } else {
            // What the study made goes to the files of the export blocks.
            self.queue_exports(false);
        }
    }

    /// A link from or to a study: from its Inputs row it adds an input column, which the
    /// points give the number it goes to as it is now; to its Outputs row an output column.
    /// A column without a kind takes that of the number. Returns the item and slot of the
    /// column.
    fn study_column(
        &mut self,
        source: u64,
        item: Vec<String>,
        target: u64,
        slot: Vec<String>,
    ) -> (Vec<String>, Vec<String>) {
        let kind = value_kind(&self.nodes, target, &slot);
        if matches!(item.as_slice(), [cell] if cell == links::LOOP) {
            let mut label = self.parameter_label(target, &slot);
            // A column of an input of a Math or Switch block goes by the block too.
            if let Some(node) = (self.nodes.iter().find(|n| n.id == target))
                && node.as_number().is_some()
            {
                label = format!("{} {label}", node.name);
            }
            let value = self.slot_value(target, &slot).unwrap_or(0.0);
            if let Some(study) = self.node(source).and_then(Node::as_study_mut) {
                let name = study.table.add_input(&label, kind, value);
                return (vec![links::LOOP.to_owned(), name], slot);
            }
        }
        if let [cell, name] = item.as_slice()
            && cell == links::LOOP
            && let Some(study) = self.node(source).and_then(Node::as_study_mut)
            && let Some(column) = study.table.input(name)
            && study.table.inputs[column].kind.is_none()
        {
            study.table.inputs[column].kind = kind;
        }
        if matches!(slot.as_slice(), [cell] if cell == links::OUTPUTS) {
            let label = match item.get(1).map(String::as_str) {
                Some("Result") | None => self.name_of(source),
                Some(name) => name.to_owned(),
            };
            if let Some(study) = self.node(target).and_then(Node::as_study_mut) {
                let name = study.table.add_output(&label);
                return (item, vec![links::OUTPUTS.to_owned(), name]);
            }
        }
        (item, slot)
    }

    /// The number at `slot` of a block now: of an exposed parameter, a number of a mesh
    /// block, the factor of a refine mesh block or a parameter of a parameters block.
    fn slot_value(&self, id: u64, slot: &[String]) -> Option<f64> {
        let node = self.nodes.iter().find(|n| n.id == id)?;
        if let (Some((number, switch)), [_, key]) = (node.as_number(), slot) {
            return (number.inputs(switch).into_iter())
                .find(|(k, ..)| k == key)
                .map(|(.., value)| value);
        }
        match slot {
            [cell, name] if cell == links::VALUE => (node.as_parameters()?.parameters.iter())
                .find(|p| &p.name == name)
                .map(|p| p.value),
            [cell, key] if cell == links::PARAMETERS && key == REFINEMENT => node.refinement(),
            [cell, key] if cell == links::PARAMETERS && node.as_deformed().is_some() => {
                let block = node.as_deformed()?;
                match key.as_str() {
                    DEFORM_FACTOR => Some(block.factor),
                    DEFORM_INCREMENT => Some(f64::from(block.increment)),
                    _ => None,
                }
            }
            [cell, key] if cell == links::PARAMETERS && node.as_measure().is_some() => {
                let axis = POINT_AXES.iter().position(|a| a == key)?;
                Some(node.as_measure()?.kind.point()?[axis])
            }
            [cell, key] if cell == links::PARAMETERS && node.as_cad().is_some() => {
                (key == CAD_LENGTH).then(|| node.as_cad().map(|c| c.operation.value))?
            }
            [cell, key] if cell == links::PARAMETERS => match node.exposing_fe() {
                Some(fe) => fe.value_at(&fe.exposed(key.parse().ok()?)?.path),
                None => node.as_mesh()?.primitive.value(key),
            },
            _ => None,
        }
    }

    /// Gives the numbers of value links to the parameters they go to; a mesh block whose
    /// parameters change is out of date and made anew by Update Project.
    fn apply_values(&mut self) {
        // A parameter of a parameters block can take its value from another one: the values
        // pass on until none changes, as links make no loop.
        for _ in 0..=self.nodes.len() {
            let mut changed = false;
            let values: Vec<(u64, Vec<String>, f64)> = (self.links.iter())
                .filter(|l| links::is_value(l))
                .filter_map(|link| {
                    let value = self.value_of(link.source, &link.item)?;
                    Some((link.target, links::slot(link).to_vec(), value))
                })
                .collect();
            for (target, slot, value) in values {
                let Some(node) = self.node(target) else {
                    continue;
                };
                if let Some((number, switch)) = node.as_number_mut() {
                    if let [_, key] = slot.as_slice() {
                        changed |= number.set(switch, key, value);
                    }
                    continue;
                }
                match slot.as_slice() {
                    [cell, name] if cell == links::VALUE => {
                        if let Some(parameter) = (node.as_parameters_mut())
                            .and_then(|p| p.parameters.iter_mut().find(|p| &p.name == name))
                        {
                            let value = parameter.kind.clean(value);
                            changed |= parameter.value != value;
                            parameter.value = value;
                        }
                    }
                    [_, key] if key == REFINEMENT => node.set_refinement(value),
                    [_, key] if node.as_deformed().is_some() => {
                        if let Some(block) = node.as_deformed_mut() {
                            match key.as_str() {
                                DEFORM_FACTOR => block.set_factor(value),
                                DEFORM_INCREMENT => block.set_increment(value),
                                _ => {}
                            }
                        }
                    }
                    [_, key] if node.as_measure().is_some() => {
                        if let Some(measure) = node.as_measure_mut() {
                            measure.kind.set_coordinate(key, value);
                        }
                    }
                    [_, key] if key == CAD_LENGTH && node.as_cad().is_some() => {
                        if let Some(cad) = node.as_cad_mut()
                            && value.is_finite()
                        {
                            cad.operation.value = value;
                        }
                    }
                    [_, key] => {
                        if let Some(mesh) = node.as_mesh_mut() {
                            mesh.primitive.set_value(key, value);
                        } else if let Ok(key) = key.parse() {
                            node.set_exposed(key, value);
                        }
                    }
                    _ => {}
                }
            }
            if !changed {
                break;
            }
        }
    }

    /// Stops exposing a parameter of a block, and removes the links that set it.
    fn unexpose(&mut self, id: u64, key: u32) {
        let slot = vec![links::PARAMETERS.to_owned(), key.to_string()];
        self.links
            .retain(|l| !(l.target == id && links::slot(l) == slot.as_slice()));
        let name = self.name_of(id);
        if let Some(fe) = self.node(id).and_then(Node::exposing_fe_mut)
            && let Some(at) = fe.exposed.iter().position(|e| e.key == key)
        {
            let exposed = fe.exposed.remove(at);
            self.output
                .push(format!("{name}: {} is no longer exposed.", exposed.name));
        }
        self.sync_due = true;
    }

    /// The number a block gives at `item`, `["Value", name]`.
    fn value_of(&self, id: u64, item: &[String]) -> Option<f64> {
        let node = self.nodes.iter().find(|n| n.id == id)?;
        let [cell, name] = item else {
            return None;
        };
        if let Some((number, switch)) = node.as_number() {
            return (cell == links::VALUE && name == RESULT)
                .then(|| number.result(switch))
                .flatten();
        }
        if let Some(measure) = node.as_measure() {
            let fresh = self.measure_state(id) == CellState::UpToDate;
            return (cell == links::VALUE && name == RESULT && fresh)
                .then_some(measure.value)
                .flatten();
        }
        if cell == links::LOOP {
            let study = node.as_study()?;
            let column = study.table.input(name)?;
            let point = study.active.unwrap_or(study.table.current);
            return study.table.points.get(point)?.inputs.get(column).copied();
        }
        (node.as_parameters()?.parameters.iter())
            .find(|p| &p.name == name)
            .map(|p| p.value)
    }

    /// The label of the parameter row `slot`, `["Parameters", key]`, of a block.
    fn parameter_label(&self, id: u64, slot: &[String]) -> String {
        parameter_label(&self.nodes, id, slot)
    }

    /// Adds a mesh export block with no file yet; returns its id.
    fn add_mesh_export(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = block_name("Mesh Export");
        self.nodes
            .push(Node::mesh_export(id, name.clone(), String::new(), None));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: drag the mesh of a block to its Mesh row and type the file to \
             write; Update Project writes it."
        ));
        id
    }

    /// Adds a file block and asks for its file; returns its id.
    fn add_file(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = block_name("File");
        let units = self.settings.new_model.units;
        self.nodes
            .push(Node::file_block(id, name.clone(), String::new(), units));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: choose a results file (.frd), an input file (.inp) or geometry \
             (STEP, IGES, BREP, STL) and drag what it reads to an analysis."
        ));
        self.choose_source_file(id);
        id
    }

    /// Asks for the file a file block reads.
    fn choose_source_file(&mut self, id: u64) {
        let base = self.export_base();
        let Some(file) = self
            .nodes
            .iter()
            .find(|n| n.id == id)
            .and_then(Node::as_file)
        else {
            return;
        };
        let current = export_file(&file.path, base.as_deref());
        let mut dialog = rfd::FileDialog::new()
            .set_title("File to Read")
            .add_filter(
                "Results, input files and geometry",
                &[
                    "frd", "inp", "step", "stp", "iges", "igs", "brep", "brp", "stl",
                ],
            )
            .add_filter("CalculiX results (*.frd)", &["frd"])
            .add_filter("CalculiX input (*.inp)", &["inp"])
            .add_filter(
                "Geometry (*.step, *.iges, *.brep, *.stl)",
                &["step", "stp", "iges", "igs", "brep", "brp", "stl"],
            );
        if let Some(dir) = current.parent().filter(|d| d.is_dir()).or(base.as_deref()) {
            dialog = dialog.set_directory(dir);
        }
        let Some(chosen) = dialog.pick_file() else {
            return;
        };
        // A file next to the project is kept relative, so the project can be moved.
        let shown = match &base {
            Some(base) => chosen.strip_prefix(base).unwrap_or(&chosen),
            None => &chosen,
        };
        let shown = shown.to_string_lossy().into_owned();
        self.set_source_path(id, shown);
    }

    /// Lets a file block read another file.
    fn set_source_path(&mut self, id: u64, path: String) {
        if let Some(file) = self.node(id).and_then(Node::as_file_mut) {
            file.path = path;
            file.forget();
            file.read = None;
            file.error = None;
            file.reading = None;
        }
        self.files_checked = None;
    }

    /// Reads the files of the file blocks again where they changed, on worker threads, and
    /// takes what they read; the links bring it to the analyses.
    fn refresh_files(&mut self, ctx: &egui::Context) {
        let base = self.export_base();
        let look = (self.files_checked).is_none_or(|t| t.elapsed() >= FILE_CHECK_INTERVAL);
        if look {
            self.files_checked = Some(Instant::now());
        }
        let mut lines = Vec::new();
        let mut changed = false;
        for node in &mut self.nodes {
            let name = node.name.clone();
            let Some(file) = node.as_file_mut() else {
                continue;
            };
            if let Some((print, receiver)) = &file.reading {
                let print = *print;
                match receiver.try_recv() {
                    Ok(Ok(read)) => {
                        lines.push(format!("{name}: {}", read.summary));
                        file.forget();
                        file.model = read.model;
                        file.space = read.space;
                        file.steps = read.steps;
                        file.parts = read.parts;
                        file.error = None;
                    }
                    Ok(Err(why)) => {
                        lines.push(format!("{name}: not read: {why}"));
                        file.forget();
                        file.error = Some(why);
                    }
                    Err(std::sync::mpsc::TryRecvError::Empty) => {
                        ctx.request_repaint_after(Duration::from_millis(100));
                        continue;
                    }
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                        file.error = Some("reading it failed".into());
                    }
                }
                file.read = Some(print);
                file.reading = None;
                changed = true;
                continue;
            }
            if file.poll_subsets() {
                changed = true;
            }
            if file.making_subsets() {
                ctx.request_repaint_after(Duration::from_millis(100));
            }
            if !look || file.path.trim().is_empty() {
                continue;
            }
            let path = export_file(&file.path, base.as_deref());
            match file_print(&path) {
                // Gone: what it gave is gone too.
                None => {
                    if file.read.take().is_some() {
                        lines.push(format!("{name}: {} is missing", path.display()));
                        file.forget();
                        file.error = None;
                        changed = true;
                    }
                }
                Some(print) if Some(print) == file.read => {}
                Some(print) => {
                    let (sender, receiver) = std::sync::mpsc::channel();
                    let (units, repaint) = (file.units, ctx.clone());
                    std::thread::spawn(move || {
                        let _ = sender.send(read_source(&path, units));
                        repaint.request_repaint();
                    });
                    file.reading = Some((print, receiver));
                }
            }
        }
        // Looked at again soon, for files changed outside.
        if self.nodes.iter().any(|n| n.as_file().is_some()) {
            ctx.request_repaint_after(FILE_CHECK_INTERVAL);
        }
        self.output.extend(lines);
        if changed {
            self.sync_due = true;
        }
    }

    /// Adds a Campbell block; returns its id.
    fn add_campbell(&mut self, position: egui::Pos2) -> u64 {
        let window = CampbellWindow::new(&plx_model::FeModel::default());
        let id = self.insert_campbell(position, window);
        let name = self.name_of(id);
        self.output.push(format!(
            "{name} created: drag the Setup of an analysis with a Complex Frequency step to \
             its Analysis row; Update Project or its diagram then \
             sweeps it."
        ));
        id
    }

    /// Inserts a Campbell block with `window`; returns its id.
    fn insert_campbell(&mut self, position: egui::Pos2, window: CampbellWindow) -> u64 {
        let id = self.next_id();
        let name = block_name("Campbell");
        self.nodes.push(Node::campbell_block(id, name, window));
        self.graph.insert_node(position, id);
        id
    }

    /// Opens the Campbell block that sweeps the analysis `analysis`; one is made and linked
    /// first if there is none.
    fn open_campbell_of(&mut self, ctx: &egui::Context, analysis: u64) {
        let existing = (self.links.iter())
            .find(|l| l.source == analysis && links::slot(l) == [links::ANALYSIS])
            .map(|l| l.target);
        let id = match existing {
            Some(id) => id,
            None => {
                let position = (self.graph_node(analysis))
                    .and_then(|g| self.graph.get_node_info(g))
                    .map_or_else(|| self.free_position(), |i| i.pos + egui::vec2(300.0, 0.0));
                // The sweep goes up to the speed of the analysis at first.
                let fe = (self.nodes.iter().find(|n| n.id == analysis))
                    .and_then(Node::link_data)
                    .map(|d| d.fe.clone())
                    .unwrap_or_default();
                let id = self.insert_campbell(position, CampbellWindow::new(&fe));
                self.link(
                    analysis,
                    vec![links::SETUP.to_owned()],
                    id,
                    vec![links::ANALYSIS.to_owned()],
                );
                id
            }
        };
        self.open_window(ctx, id, None);
    }

    /// The analysis a Campbell block sweeps.
    fn campbell_source(&self, id: u64) -> Option<&Node> {
        let link =
            (self.links.iter()).find(|l| l.target == id && links::slot(l) == [links::ANALYSIS])?;
        self.nodes.iter().find(|n| n.id == link.source)
    }

    /// The state of the diagram of a Campbell block: up to date while the analysis is as it
    /// was swept, its speed left out.
    fn campbell_state(&self, id: u64) -> CellState {
        let Some(node) = self.nodes.iter().find(|n| n.id == id) else {
            return CellState::Missing;
        };
        let Some(CampbellBlock {
            window: Some(window),
            input,
        }) = node.as_campbell()
        else {
            return CellState::Missing;
        };
        if window.is_running() {
            CellState::Running
        } else if window.points().is_empty() || self.campbell_source(id).is_none() {
            CellState::Missing
        } else if input.is_some() && window.fingerprint() == *input {
            CellState::UpToDate
        } else {
            CellState::OutOfDate
        }
    }

    /// Starts the sweep of a Campbell block, or continues it with `keep`.
    fn start_campbell(&mut self, id: u64, keep: bool) -> Result<(), String> {
        let mut window = (self.node(id).and_then(Node::as_campbell_mut))
            .and_then(|c| c.window.take())
            .ok_or_else(|| "not a Campbell block".to_string())?;
        let result = self.start_sweep(id, &mut window, keep);
        if let Some(campbell) = self.node(id).and_then(Node::as_campbell_mut) {
            campbell.window = Some(window);
        }
        result
    }

    /// Starts the sweep of the Campbell block `id`, whose window is `window`, on the model of
    /// the analysis linked to it, in a folder of its own in that of the project.
    fn start_sweep(&self, id: u64, window: &mut CampbellWindow, keep: bool) -> Result<(), String> {
        let source = self.campbell_source(id).ok_or_else(|| {
            "No analysis is linked: drag the Setup of an analysis to the Analysis row of the \
             block."
                .to_string()
        })?;
        let data = (source.link_data()).ok_or_else(|| format!("{} is loading.", source.name))?;
        let dir = (self.settings.solver.work_dir())
            .join(crate::analysis::job_name(&self.project_name()))
            .join(crate::analysis::job_dir(id));
        let solver = &self.settings.solver;
        window.start(solver, (data.mesh, data.fe), dir, keep)
    }

    /// Takes the finished speeds of the Campbell blocks and shows their open diagrams.
    fn campbell_windows(&mut self, ctx: &egui::Context) {
        let ids: Vec<u64> = (self.nodes.iter())
            .filter(|n| n.kind() == Kind::Campbell)
            .map(|n| n.id)
            .collect();
        for id in ids {
            let Some(mut window) =
                (self.node(id).and_then(Node::as_campbell_mut)).and_then(|c| c.window.take())
            else {
                continue;
            };
            window.poll();
            if window.visible {
                let can_start = (self.campbell_source(id))
                    .and_then(Node::link_data)
                    .is_some_and(|d| crate::campbell::applies(d.fe) && d.mesh.element_count() > 0);
                let name = self.name_of(id);
                let open = window.show(ctx, (&name, id), can_start, |window, keep| {
                    self.start_sweep(id, window, keep)
                });
                window.visible = open;
            }
            if let Some(speed) = window.take_analysis_speed() {
                let note = self.set_analysis_speed(id, speed);
                self.output.push(note.clone());
                window.note(note);
            }
            if window.is_running() {
                ctx.request_repaint_after(Duration::from_millis(250));
            }
            if let Some(campbell) = self.node(id).and_then(Node::as_campbell_mut) {
                campbell.window = Some(window);
            }
        }
    }

    /// Sets the analysis the Campbell block `id` sweeps to turn at `speed`, so that solving it
    /// gives the results at that speed; the diagram stays up to date, as the sweep sets the
    /// speed itself. Returns what happened, for the output.
    fn set_analysis_speed(&mut self, id: u64, speed: f64) -> String {
        let shown = crate::campbell::shown(crate::campbell::rpm(speed));
        let Some(source) = self.campbell_source(id).map(|n| n.id) else {
            return "Speed not set: no analysis is linked to the block.".into();
        };
        let name = self.name_of(source);
        let Some(node) = self.node(source) else {
            return format!("Speed not set: {name} is gone.");
        };
        let Some(fe) = node.exposing_fe_mut() else {
            return format!("Speed not set: {name} is loading.");
        };
        if crate::campbell::rotor_speed(fe).is_none() {
            return format!("Speed not set: {name} has no active Complex Frequency step.");
        }
        crate::campbell::set_speed(fe, speed);
        node.model_changed();
        self.sync_due = true;
        format!(
            "{name}: Complex Frequency step set to {shown} rpm; solve it for the results there."
        )
    }

    /// Starts the sweeps of the Campbell blocks whose diagram is not up to date; returns
    /// whether any started.
    fn update_campbells(&mut self) -> bool {
        let due: Vec<u64> = (self.nodes.iter())
            .filter(|n| n.kind() == Kind::Campbell)
            .map(|n| n.id)
            .filter(|&id| {
                self.in_scope(id)
                    && self.campbell_source(id).is_some()
                    && matches!(
                        self.campbell_state(id),
                        CellState::Missing | CellState::OutOfDate
                    )
            })
            .collect();
        let mut started = false;
        for id in due {
            let name = self.name_of(id);
            match self.start_campbell(id, false) {
                Ok(()) => {
                    self.output.push(format!("{name}: sweep started"));
                    started = true;
                }
                Err(why) => self.output.push(format!("{name} not swept: {why}")),
            }
        }
        started
    }

    /// The folder relative export paths are taken from: that of the project file.
    fn export_base(&self) -> Option<PathBuf> {
        self.path.as_deref()?.parent().map(Path::to_path_buf)
    }

    /// Writes the file of a mesh export block, unless it is up to date and `force` is off;
    /// returns whether it is fine afterwards.
    fn run_export(&mut self, id: u64, force: bool) -> bool {
        let base = self.export_base();
        let Some(node) = self.node(id) else {
            return true;
        };
        if !force && node.export_state(base.as_deref()) == CellState::UpToDate {
            return true;
        }
        let name = node.name.clone();
        let (line, error) = match node.write_export(base.as_deref()) {
            Ok(file) => (format!("{name}: {} written", file.display()), None),
            Err(why) => (format!("{name}: not written: {why}"), Some(why)),
        };
        let ok = error.is_none();
        if let Some(export) = node.as_export_mut() {
            export.error = error;
        }
        self.output.push(line);
        ok
    }

    /// Queues the mesh export blocks that have a file, for [`Self::run_exports`]; with
    /// `changed_only` those whose file is up to date stay out, which is only known while no
    /// analysis is to be solved before them.
    fn queue_exports(&mut self, changed_only: bool) {
        self.export_queue.clear();
        self.update_failed = false;
        let base = self.export_base();
        let ids: Vec<u64> = (self.nodes.iter())
            .map(|n| n.id)
            .filter(|&id| self.in_scope(id))
            .collect();
        for id in links::order(&ids, &self.links) {
            let Some(node) = (self.nodes.iter()).find(|n| n.id == id) else {
                continue;
            };
            let Some(export) = node.as_export() else {
                continue;
            };
            if export.path.trim().is_empty() {
                self.output
                    .push(format!("{} skipped: no file is chosen.", node.name));
            } else if !changed_only || node.export_state(base.as_deref()) != CellState::UpToDate {
                self.export_queue.push(id);
            }
        }
    }

    /// Writes the files of the mesh export blocks Update Project queued, once the analyses
    /// before them are done and their links have brought the meshes.
    fn run_exports(&mut self) {
        if self.export_queue.is_empty()
            || !self.updating.is_empty()
            || !self.update_queue.is_empty()
            || self.looping.is_some()
            || self.sync_due
            || self.update_after_sync
            || self.deforming()
            || self.restoring.values().any(Option::is_some)
        {
            return;
        }
        let mut failed = std::mem::take(&mut self.update_failed);
        for id in std::mem::take(&mut self.export_queue) {
            failed |= !self.run_export(id, false);
        }
        self.output.push(if failed {
            "Update finished with errors".into()
        } else {
            self.updated_line()
        });
    }

    /// Asks for the file a mesh export block writes to.
    fn choose_export_file(&mut self, id: u64) {
        let base = self.export_base();
        let Some(node) = self.nodes.iter().find(|n| n.id == id) else {
            return;
        };
        let Some(export) = node.as_export() else {
            return;
        };
        let current = export_file(&export.path, base.as_deref());
        let name = Path::new(&export.path).file_name().map_or_else(
            || format!("{}.inp", node.name),
            |n| n.to_string_lossy().into(),
        );
        let mut dialog = rfd::FileDialog::new()
            .set_title("Mesh Export File")
            .add_filter("CalculiX input (*.inp)", &["inp"])
            .set_file_name(name);
        if let Some(dir) = current.parent().filter(|d| d.is_dir()).or(base.as_deref()) {
            dialog = dialog.set_directory(dir);
        }
        let Some(mut file) = dialog.save_file() else {
            return;
        };
        if file.extension().is_none() {
            file.set_extension("inp");
        }
        // A file next to the project is kept relative, so the project can be moved.
        let shown = match &base {
            Some(base) => file.strip_prefix(base).unwrap_or(&file),
            None => &file,
        };
        let shown = shown.to_string_lossy().into_owned();
        if let Some(export) = self.node(id).and_then(Node::as_export_mut) {
            export.path = shown;
            export.error = None;
        }
    }

    /// Makes the meshes of the mesh blocks that are out of date anew; returns whether it
    /// made any. A block whose mesh cannot be made stays out of date, and so do the
    /// analyses that take their mesh from it.
    fn generate_meshes(&mut self) -> bool {
        let (made, lines) = generate_stale_meshes(&mut self.nodes, None);
        self.output.extend(lines);
        if made {
            self.sync_due = true;
        }
        made
    }

    /// The mesh blocks before the analysis `id` whose mesh is out of date, see
    /// [`stale_meshes`].
    fn stale_meshes(&self, id: u64) -> Vec<String> {
        stale_meshes(&self.nodes, &self.links, id)
    }

    /// Makes the mesh of one mesh block anew.
    fn generate_mesh(&mut self, id: u64) {
        let Some(node) = self.node(id) else {
            return;
        };
        let line = match node.as_cad_mut().map(CadBlock::generate) {
            Some(Ok(made)) => format!("{}: {made}", node.name),
            Some(Err(why)) => format!("{}: the operation failed: {why}", node.name),
            None => match node.generate() {
                Ok(count) => format!("{}: mesh made, {count} elements", node.name),
                Err(why) => format!("{}: no mesh made: {why}", node.name),
            },
        };
        self.output.push(line);
        self.sync_due = true;
    }

    fn edit_mesh(&mut self, id: u64) {
        let linked = self.links.iter().any(|l| l.source == id);
        let Some(node) = self.nodes.iter().find(|n| n.id == id) else {
            return;
        };
        let Some(mesh) = node.as_mesh() else {
            return;
        };
        self.mesh_block = Some(MeshBlockEditor::new(
            id,
            &node.name,
            mesh.primitive.clone(),
            mesh.space,
            mesh.units,
            linked,
        ));
    }

    fn edit_cad(&mut self, id: u64) {
        let length = [links::PARAMETERS.to_owned(), CAD_LENGTH.to_owned()];
        let length_linked = (self.links.iter()).any(|l| {
            l.target == id && (l.slot == length || (l.slot.is_empty() && l.item == length))
        });
        let Some(node) = self.nodes.iter().find(|n| n.id == id) else {
            return;
        };
        let Some(cad) = node.as_cad() else {
            return;
        };
        let Some(input) = cad.input.clone() else {
            let line = format!(
                "{}: link a geometry to its Geometry row first, then pick the edges on it.",
                node.name
            );
            self.output.push(line);
            return;
        };
        self.cad_editor = Some(CadOperationEditor::new(
            id,
            &node.name,
            input,
            &cad.operation,
            cad.units,
            length_linked,
        ));
    }

    fn cad_editor_window(&mut self, ctx: &egui::Context) {
        let Some(editor) = &mut self.cad_editor else {
            return;
        };
        let render_state = &self.render_state;
        let viewport = (self.cad_viewport)
            .get_or_insert_with(|| crate::viewport::Viewport::new(render_state.clone()));
        let id = editor.id;
        match editor.show(ctx, viewport) {
            CadEditorResult::Open => {}
            CadEditorResult::Cancel => self.cad_editor = None,
            CadEditorResult::Ok(operation, made) => {
                self.checkpoint(&[]);
                self.cad_editor = None;
                if let Some(node) = self.node(id) {
                    if let Some(cad) = node.as_cad_mut() {
                        cad.operation = operation;
                        // The preview made it for this very input and operation.
                        if let (Some(made), Some(input)) = (made, &cad.input) {
                            let print = cad.operation.fingerprint(input);
                            cad.take_result(made, print);
                        }
                    }
                    if node
                        .as_cad()
                        .is_some_and(|c| c.state() == CellState::OutOfDate)
                    {
                        let line = format!(
                            "{}: the geometry is out of date; Update Project or Update \
                             Geometry makes it again.",
                            node.name
                        );
                        self.output.push(line);
                    }
                }
                self.sync_due = true;
            }
        }
    }

    fn mesh_block_window(&mut self, ctx: &egui::Context) {
        let Some(editor) = &mut self.mesh_block else {
            return;
        };
        let id = editor.id;
        match editor.show(ctx) {
            MeshBlockResult::Open => {}
            MeshBlockResult::Cancel => self.mesh_block = None,
            MeshBlockResult::Ok(primitive, space, units) => {
                self.checkpoint(&[]);
                self.mesh_block = None;
                if let Some(node) = self.node(id) {
                    if let Some(mesh) = node.as_mesh_mut() {
                        mesh.primitive = primitive;
                        mesh.space = space;
                        mesh.units = units;
                    }
                    if node.mesh_state() == CellState::OutOfDate {
                        let line = format!(
                            "{}: the mesh is out of date; Update Project or Generate Mesh \
                             makes it anew.",
                            node.name
                        );
                        self.output.push(line);
                    }
                }
                self.sync_due = true;
            }
        }
    }

    /// Opens the editor of a materials or cross-sections block, with the items of the
    /// analyses to import.
    fn edit_collection(&mut self, id: u64) {
        let Some(node) = self.nodes.iter().find(|n| n.id == id) else {
            return;
        };
        let (Some(collection), Some(block)) = (node.collection(), node.as_collection()) else {
            return;
        };
        let analyses = (self.nodes.iter())
            .filter(|n| n.kind() == Kind::Analysis)
            .filter_map(|n| Some((n.name.clone(), collection.copy(n.link_data()?.fe))))
            .collect();
        let linked = (self.links.iter())
            .filter(|l| l.target == id)
            .flat_map(|l| links::filled_paths(l, &self.name_of(l.source)))
            .collect::<Vec<_>>();
        self.material_set = Some(MaterialSetEditor::new(
            id,
            &node.name,
            collection,
            &block.model.fe,
            block.units,
            analyses,
            links::linked_parameters(&linked),
        ));
    }

    fn material_set_window(&mut self, ctx: &egui::Context) {
        let Some(editor) = &mut self.material_set else {
            return;
        };
        let id = editor.id;
        match editor.show(ctx) {
            MaterialSetResult::Open => {}
            MaterialSetResult::Cancel => self.material_set = None,
            MaterialSetResult::Ok(edited) => {
                self.checkpoint(&[]);
                self.material_set = None;
                let node = self.node(id);
                let collection = node.as_ref().and_then(|n| n.collection());
                if let (Some(block), Some(collection)) =
                    (node.and_then(Node::as_collection_mut), collection)
                {
                    collection.take(&mut block.model.fe, &edited);
                    block.model.fe.exposed = edited.exposed;
                    block.units = edited.properties.units;
                }

                // The analyses that link them take them now.
                self.sync_due = true;
            }
        }
    }

    /// The analysis' block in the schematic.
    fn graph_node(&self, id: u64) -> Option<NodeId> {
        (self.graph.nodes_ids_data())
            .find(|(_, node)| node.value == id)
            .map(|(graph_id, _)| graph_id)
    }

    fn open_window(&mut self, ctx: &egui::Context, id: u64, view: Option<Show>) {
        match self.nodes.iter().find(|n| n.id == id).map(Node::kind) {
            Some(Kind::Analysis) => {}
            Some(Kind::Materials | Kind::CrossSections) => {
                // Asked for from an analysis window: the editor is in the project window.
                ctx.send_viewport_cmd_to(egui::ViewportId::ROOT, egui::ViewportCommand::Focus);
                return self.edit_collection(id);
            }
            Some(Kind::Mesh) => {
                ctx.send_viewport_cmd_to(egui::ViewportId::ROOT, egui::ViewportCommand::Focus);
                return self.edit_mesh(id);
            }
            // Its numbers are edited in its block.
            Some(Kind::RefineMesh | Kind::DeformedMesh) => return,
            Some(Kind::CadOperation) => {
                ctx.send_viewport_cmd_to(egui::ViewportId::ROOT, egui::ViewportCommand::Focus);
                return self.edit_cad(id);
            }
            Some(Kind::Parameters) => return self.edit_parameters(id),
            Some(Kind::Measure) => return self.edit_measure(id),
            Some(Kind::Study) => return self.edit_study(id),
            Some(Kind::MeshExport) => {
                ctx.send_viewport_cmd_to(egui::ViewportId::ROOT, egui::ViewportCommand::Focus);
                return self.choose_export_file(id);
            }
            Some(Kind::File) => {
                ctx.send_viewport_cmd_to(egui::ViewportId::ROOT, egui::ViewportCommand::Focus);
                return self.choose_source_file(id);
            }
            Some(Kind::Campbell) => {
                ctx.send_viewport_cmd_to(egui::ViewportId::ROOT, egui::ViewportCommand::Focus);
                if let Some(window) =
                    (self.node(id).and_then(Node::as_campbell_mut)).and_then(|c| c.window.as_mut())
                {
                    window.visible = true;
                }
                return;
            }
            Some(Kind::Note) => {
                self.focus_note = Some(id);
                return;
            }
            // Its numbers are edited in its block.
            Some(Kind::Math | Kind::Switch) => return,
            None => return,
        }
        self.open_analysis(ctx, id, view, false);
    }

    /// Opens the analysis: in the project window, which then shows it in the workspace of
    /// its window, or with `own` in a window of its own. An analysis already open in a
    /// window of its own is brought to the front.
    fn open_analysis(&mut self, ctx: &egui::Context, id: u64, view: Option<Show>, own: bool) {
        // One in a window of its own stays out of the project window: it is brought to the
        // front there, and the project window keeps its active analysis.
        if !own && !self.in_own_window(id) {
            self.active = Some(id);
        }
        let shown_here = !self.in_project && self.active == Some(id);
        if let Some(analysis) = self.node(id).and_then(Node::as_analysis_mut)
            && let Some(window) = &mut analysis.window
            && !analysis.own_window
        {
            if let Some(view) = view {
                window.show_workspace(view);
            }
            if own {
                // Moved out into a window of its own.
                analysis.own_window = true;
                if shown_here {
                    self.in_project = true;
                }
            } else {
                self.in_project = false;
            }
            return;
        }
        let path = self.model_path();
        let folder = crate::analysis::job_name(&self.project_name());
        let (render_state, settings) = (self.render_state.clone(), self.settings.clone());
        let at = ctx.input(|i| i.viewport().outer_rect).map(|r| r.min);
        let import = std::mem::take(&mut self.startup_import);
        // What it takes from others is known before it is solved.
        let incoming = self.incoming(id);
        let upstream = crate::status::Upstream {
            stale_meshes: self.stale_meshes(id),
            ..Default::default()
        };
        self.sync_due = true;
        let Some(node) = self.node(id) else {
            return;
        };
        let name = node.name.clone();
        let Some(analysis) = node.as_analysis_mut() else {
            return;
        };
        if analysis.window.is_some() {
            // Brought to the front, also when minimized (Windows focuses only a window that
            // is not). Wayland lets no program take the focus: there the taskbar marks it.
            let viewport = analysis_viewport(id);
            for command in [
                egui::ViewportCommand::Minimized(false),
                egui::ViewportCommand::Focus,
                egui::ViewportCommand::RequestUserAttention(egui::UserAttentionType::Informational),
            ] {
                ctx.send_viewport_cmd_to(viewport, command);
            }
            if let (Some(view), Some(window)) = (view, &mut analysis.window) {
                window.show_workspace(view.clone());
            }
            return;
        }
        let Some(data) = analysis.data.take() else {
            return;
        };
        let mut window = AnalysisWindow::new(render_state, settings, &name);
        window.set_project_folder(folder);
        window.set_links(incoming, upstream);
        if let Some(view) = view {
            window.show_workspace(view.clone());
        }
        window.load(ctx, path, data);
        if !import.is_empty() {
            window.import_after_load(import);
        }
        analysis.window = Some(Box::new(window));
        analysis.window_position = at;
        analysis.own_window = own;
        if !own {
            self.in_project = false;
        }
    }

    /// The path the analyses' models take: the project file, or a name for an unsaved one.
    fn model_path(&self) -> PathBuf {
        (self.path.clone())
            .unwrap_or_else(|| PathBuf::from(format!("Untitled.{PROJECT_EXTENSION}")))
    }

    /// Closes the analysis' window; its data goes back to the project. Not while the window
    /// works on something that would be lost.
    fn close_window(&mut self, id: u64) -> bool {
        let Some(node) = self.node(id) else {
            return true;
        };
        let name = node.name.clone();
        let Some(analysis) = node.as_analysis_mut() else {
            return true;
        };
        let Some(window) = &mut analysis.window else {
            return true;
        };
        if let Some(reason) = window.busy() {
            let line = format!("{name} stays open while {reason}.");
            window.message(line.clone());
            self.output.push(line);
            return false;
        }
        analysis.data = window.data();
        analysis.window = None;
        analysis.own_window = false;
        analysis.seen = None;
        node.refresh_closed();
        self.sync_due = true;
        true
    }

    fn close_all_windows(&mut self) -> bool {
        let ids: Vec<u64> = self.nodes.iter().map(|n| n.id).collect();
        ids.into_iter().all(|id| self.close_window(id))
    }

    /// Starts a new, empty project.
    fn new_project(&mut self) {
        if !self.close_all_windows() {
            return;
        }
        self.nodes.clear();
        self.links.clear();
        self.frames.clear();
        self.compact.clear();
        self.frame_label = None;
        self.applied.clear();
        self.restoring.clear();
        self.sync_due = true;
        self.graph = Snarl::new();
        self.graph_generation += 1;
        self.path = None;
        self.forget_solves();
        self.history.clear();
        self.in_project = true;
        self.active = None;
        self.add_first_analysis();
        self.forget_edits();
        self.output.push("New project".into());
    }

    /// A new project starts with an analysis, with the model space and units of the
    /// settings, so that Geometry, FE Model and Results can be used right away.
    fn add_first_analysis(&mut self) {
        let properties = ModelProperties {
            kind: plx_model::ModelKind::General,
            global_results: None,
            ..self.settings.new_model.clone()
        };
        let id = self.add_analysis("Analysis-1", egui::pos2(40.0, 40.0), properties);
        self.active = Some(id);
    }

    fn open_dialog(&mut self, ctx: &egui::Context) {
        let picked = rfd::FileDialog::new()
            .set_title("Open Project")
            .add_filter("Nodalis project (*.ndl)", &["ndl", "NDL"])
            .pick_file();
        if let Some(path) = picked {
            self.open(ctx, path);
        }
    }

    /// Puts `path` first in the list of Open Recent.
    fn remember(&mut self, path: &Path) {
        self.recent = with_recent(&self.recent, path);
    }

    /// The panel of Open Recent: the recent projects that still exist, the latest first.
    /// Returns the one picked.
    fn recent_panel(&mut self, ui: &mut egui::Ui) -> Option<PathBuf> {
        self.recent.retain(|path| path.is_file());
        if self.recent.is_empty() {
            ui.weak("No recent projects");
            return None;
        }
        let mut picked = None;
        for path in &self.recent {
            let name = (path.file_name()).map_or_else(String::new, |n| n.to_string_lossy().into());
            let open = ui.add_enabled(self.opening.is_none(), egui::Button::new(name));
            if open.on_hover_text(path.display().to_string()).clicked() {
                picked = Some(path.clone());
                ui.close();
            }
        }
        picked
    }

    /// Reads the project file on a worker thread; it replaces the open project once read.
    fn open(&mut self, ctx: &egui::Context, path: PathBuf) {
        if self.opening.is_some() {
            return;
        }
        let (sender, receiver) = std::sync::mpsc::channel();
        let (file, ctx) = (path.clone(), ctx.clone());
        std::thread::spawn(move || {
            // A panic while reading is reported like any other error, instead of leaving
            // the project window waiting for the file forever.
            let opened = std::panic::catch_unwind(|| read_opened(&file))
                .unwrap_or_else(|panic| Err(panic_message(panic.as_ref())));
            let _ = sender.send(opened);
            ctx.request_repaint();
        });
        self.opening = Some((path, receiver));
    }

    fn poll_opening(&mut self) {
        let Some((path, receiver)) = &self.opening else {
            return;
        };
        let Some(result) = received(receiver) else {
            return;
        };
        let path = path.clone();
        self.opening = None;
        match result {
            Ok((project, notes)) => {
                if !self.close_all_windows() {
                    return;
                }
                self.output.extend(notes);
                self.nodes.clear();
                self.history.clear();
                self.graph = Snarl::new();
                self.graph_generation += 1;
                // The view of the schematic is not saved: it starts out showing all blocks.
                self.show_all = true;
                self.links = project.links;
                self.frames = project.frames;
                self.compact = project.compact.into_iter().collect();
                self.frame_label = None;
                self.applied.clear();
                self.restoring.clear();
                self.sync_due = true;
                for node in project.nodes {
                    let [x, y] = node.position();
                    self.graph.insert_node(egui::pos2(x, y), node.id());
                    self.nodes.push(Node::read(node));
                }
                let analyses = (self.nodes.iter())
                    .filter(|n| n.kind() == Kind::Analysis)
                    .count();
                let sources = self.nodes.len() - analyses;
                self.output.push(if sources == 0 {
                    format!("{} opened: {analyses} analyses", path.display())
                } else {
                    let other = if sources == 1 { "block" } else { "blocks" };
                    format!(
                        "{} opened: {analyses} analyses, {sources} other {other}",
                        path.display()
                    )
                });
                self.remember(&path);
                self.path = Some(path);
                self.forget_solves();
                self.forget_edits();
            }
            Err(error) => self.output.push(format!("Not opened: {error}")),
        }
    }

    /// The project as it is saved, with the analyses' data from their windows; `results`
    /// false leaves out the results the analyses keep.
    fn project(&mut self, results: bool) -> Project {
        let positions: Vec<(u64, egui::Pos2)> = (self.graph.nodes_pos_ids())
            .map(|(_, pos, id)| (*id, pos))
            .collect();
        let mut nodes = Vec::with_capacity(self.nodes.len());
        for node in &mut self.nodes {
            let position = (positions.iter())
                .find(|(id, _)| *id == node.id)
                .map_or([0.0; 2], |(_, pos)| [pos.x, pos.y]);
            let Some(mut written) = node.written(position) else {
                continue;
            };
            if let ProjectNode::Analysis(data) = &mut written
                && !results
            {
                data.results = None;
            }
            nodes.push(written);
        }
        Project {
            nodes,
            links: self.links.clone(),
            frames: self.frames.clone(),
            compact: self.compact.iter().copied().collect(),
            format: plx_model::PROJECT_FORMAT,
        }
    }

    fn duplicate(&mut self, id: u64) {
        let position = (self.graph_node(id))
            .and_then(|g| self.graph.get_node_info(g))
            .map_or(egui::pos2(40.0, 40.0), |info| {
                info.pos + egui::vec2(40.0, 260.0)
            });
        let Some(name) = self.node(id).map(|n| format!("Copy of {}", n.name)) else {
            return;
        };
        let new_id = self.next_id();
        let name = block_name(&name);
        let Some(copy) = self.node(id).and_then(|n| n.copied(new_id, name.clone())) else {
            return;
        };
        self.nodes.push(copy);
        self.graph.insert_node(position, new_id);
        // The copy goes into the frame of the block, and looks like it.
        if let Some(frame) = frames::frame_of(&self.frames, id) {
            self.frames[frame].nodes.push(new_id);
        }
        if self.compact.contains(&id) {
            self.compact.insert(new_id);
        }
        self.output.push(format!("{name} created"));
    }

    /// Deletes the block; undo brings it back.
    fn delete(&mut self, id: u64) {
        if let Some(node) = self.detach(id) {
            self.output.push(format!("{} deleted", node.name));
            self.keep_deleted(node);
        }
    }

    /// Takes the block out of the project, with its links and what works on it; `None` if
    /// its window cannot close now.
    fn detach(&mut self, id: u64) -> Option<Node> {
        if !self.close_window(id) {
            return None;
        }
        for solve in self.solves.iter_mut().filter(|s| s.id == id) {
            solve.stop();
        }
        self.solves.retain(|s| s.id != id);
        self.updating.retain(|&solving| solving != id);
        if let Some(graph_id) = self.graph_node(id) {
            self.graph.remove_node(graph_id);
        }
        let node = (self.nodes.iter().position(|n| n.id == id)).map(|i| self.nodes.remove(i));
        self.links.retain(|l| l.source != id && l.target != id);
        self.applied.retain(|(target, _), _| *target != id);
        self.leave_frames(&[id]);
        self.compact.remove(&id);
        self.selected.retain(|&s| s != id);
        if self.material_set.as_ref().is_some_and(|e| e.id == id) {
            self.material_set = None;
        }
        if self.mesh_block.as_ref().is_some_and(|e| e.id == id) {
            self.mesh_block = None;
        }
        if self.cad_editor.as_ref().is_some_and(|e| e.id == id) {
            self.cad_editor = None;
        }
        if self.parameter_set.as_ref().is_some_and(|e| e.id == id) {
            self.parameter_set = None;
        }
        if self.measure_editor.as_ref().is_some_and(|e| e.id == id) {
            self.measure_editor = None;
        }
        self.measuring.remove(&id);
        if self.study_editor.as_ref().is_some_and(|e| e.id == id) {
            self.study_editor = None;
        }
        if self.looping.as_ref().is_some_and(|run| run.id == id) {
            self.looping = None;
        }
        self.loops_due.retain(|&due| due != id);
        self.update_queue.retain(|&queued| queued != id);
        node
    }

    /// Updates the block `id` and the blocks it takes data from, as Update Project does,
    /// and leaves the others as they are; a parameter study solves its design points.
    fn update_block(&mut self, id: u64) {
        if self.busy() {
            return;
        }
        if self.node(id).is_some_and(|n| n.kind() == Kind::Study) {
            self.start_study(id);
            return;
        }
        let mut scope = links::upstream(&self.links, id);
        scope.push(id);
        self.output.push(format!(
            "Updating {} and what it takes data from",
            self.name_of(id)
        ));
        self.update_scope = Some(scope);
        self.update_project();
    }

    /// The deformed mesh blocks before the analysis `id`.
    fn deformed_before(&self, id: u64) -> Vec<u64> {
        (links::upstream(&self.links, id).into_iter())
            .filter(|&b| (self.nodes.iter()).any(|n| n.id == b && n.kind() == Kind::DeformedMesh))
            .collect()
    }

    /// Whether the analysis `id` takes a deformed mesh made from the results of an analysis
    /// queued to be solved.
    fn after_deformed(&self, id: u64) -> bool {
        (self.deformed_before(id).into_iter()).any(|d| {
            (links::upstream(&self.links, d).iter()).any(|s| self.update_queue.contains(s))
        })
    }

    /// The line an update that went well ends with.
    fn updated_line(&self) -> String {
        match self.update_scope.as_ref().and_then(|scope| scope.last()) {
            Some(&id) => format!("{} updated", self.name_of(id)),
            None => "Project updated".into(),
        }
    }

    /// Whether the update under way takes in the block `id`.
    fn in_scope(&self, id: u64) -> bool {
        (self.update_scope.as_ref()).is_none_or(|scope| scope.contains(&id))
    }

    /// Ansys Workbench's Update Project: solves every analysis that is not up to date, one
    /// after the other; only those of [`Self::update_scope`] when it is set.
    fn update_project(&mut self) {
        // Mesh blocks first: what links their meshes takes the new one before it is found
        // out of date, so the update goes on once the links are brought up to date.
        let (made, lines) = generate_stale_meshes(&mut self.nodes, self.update_scope.as_deref());
        self.output.extend(lines);
        if made {
            self.sync_due = true;
        }
        // A deformed mesh being made decides whether the analyses after it are up to date.
        if self.sync_due || self.deforming() {
            self.update_after_sync = true;
            return;
        }
        let mut skipped = Vec::new();
        self.update_queue.clear();
        // Sources before their targets; a submodel is solved again after its global model.
        let ids: Vec<u64> = (self.nodes.iter())
            .map(|n| n.id)
            .filter(|&id| self.in_scope(id))
            .collect();
        for id in links::order(&ids, &self.links) {
            let Some(node) = self.nodes.iter().find(|n| n.id == id) else {
                continue;
            };
            let after_source = (self.links.iter()).any(|l| {
                l.target == id && links::is_results(l) && self.update_queue.contains(&l.source)
            });
            // Its mesh is made from the results of an analysis solved before it: it has none
            // yet, or an old one, until then.
            let after_deformed = self.after_deformed(id);
            match &node.current_status() {
                Some(status) if after_deformed && status.setup != CellState::Missing => {
                    self.update_queue.push_back(node.id)
                }
                Some(status) if (status.needs_solve() || after_source) && status.solvable() => {
                    self.update_queue.push_back(node.id)
                }
                Some(status) if status.needs_solve() => skipped.push(node.name.clone()),
                _ => {}
            }
        }
        for name in skipped {
            self.output.push(format!(
                "{name} skipped: it has no mesh or geometry, or no step yet."
            ));
        }
        // The files of the mesh export blocks come after everything they may depend on, and
        // after a parameter study: it updates the project at every design point.
        let mut swept = false;
        if self.looping.is_none() && self.loops_due.is_empty() {
            self.queue_exports(self.update_queue.is_empty());
            // A sweep runs the model of its analysis by itself, beside the solves.
            swept = self.update_campbells();
        }
        if self.update_queue.is_empty() && self.export_queue.is_empty() && !swept {
            self.output.push("Nothing to update.".into());
        }
    }

    /// Works through Update Project: solves the next analyses, as many at a time as the
    /// settings allow; an analysis waits for those it takes data from. An analysis whose
    /// window is closed is solved in the background without opening it; one whose window is
    /// open is solved there and shows its results.
    fn run_update(&mut self, ctx: &egui::Context) {
        for id in std::mem::take(&mut self.updating) {
            let background = (self.solves.iter().rev()).find(|s| s.id == id);
            let Some(node) = self.nodes.iter_mut().find(|n| n.id == id) else {
                continue;
            };
            let outcome = match background {
                Some(solve) if solve.active() => {
                    self.updating.push(id);
                    continue;
                }
                Some(solve) => solve.outcome(),
                None => None,
            };
            // Done: solved, or stopped by an error the monitor shows.
            let failed = matches!(outcome, Some(Outcome::Failed | Outcome::Stopped))
                || match node.window() {
                    Some(window) => window.current_status().is_some_and(|s| s.needs_solve()),
                    None => node.status().is_some_and(|s| s.needs_solve()),
                };
            let name = node.name.clone();
            if failed {
                // What depends on it is not solved; independent analyses go on.
                let dependent = self.dependents(id);
                let skipped: Vec<String> = (self.nodes.iter())
                    .filter(|n| {
                        dependent.contains(&n.id)
                            && (self.update_queue.contains(&n.id)
                                || self.export_queue.contains(&n.id))
                    })
                    .map(|n| n.name.clone())
                    .collect();
                self.update_queue
                    .retain(|queued| !dependent.contains(queued));
                self.export_queue
                    .retain(|queued| !dependent.contains(queued));
                self.output.push(if skipped.is_empty() {
                    format!("{name} could not be solved; the solution monitor tells why.")
                } else {
                    format!(
                        "{name} could not be solved; {} not updated. The solution monitor \
                         tells why.",
                        skipped.join(", ")
                    )
                });
            } else {
                self.output.push(format!("{name} solved"));
            }
            if self.updating.is_empty() && self.update_queue.is_empty() {
                if self.export_queue.is_empty() {
                    self.output
                        .push(if failed || std::mem::take(&mut self.update_failed) {
                            "Update finished with errors".into()
                        } else {
                            self.updated_line()
                        });
                } else {
                    // The export blocks write their files, then the update is done.
                    self.update_failed |= failed;
                }
            }
            // The analyses after it take its new results before they are solved.
            self.sync_due = true;
        }
        if self.sync_due || self.restoring.values().any(Option::is_some) {
            return;
        }
        // A deformed mesh that waits to be made from new results is made first.
        let pending: Vec<u64> = if self.update_queue.is_empty() {
            Vec::new()
        } else {
            (self.nodes.iter())
                .filter(|n| n.as_deformed().is_some_and(DeformedBlock::pending))
                .map(|n| n.id)
                .collect()
        };
        if pending.iter().any(|&id| {
            (self.nodes.iter().find(|n| n.id == id))
                .and_then(Node::as_deformed)
                .is_some_and(|d| d.making.is_none())
        }) {
            self.sync_due = true;
        }
        let parallel = self.settings.solver.parallel_runs.max(1) as usize;
        while self.updating.len() < parallel {
            // The first queued analysis whose sources are neither queued nor solving, nor
            // a deformed mesh being made.
            let busy = |source: u64| {
                self.update_queue.contains(&source)
                    || self.updating.contains(&source)
                    || pending.contains(&source)
            };
            let ready = self.update_queue.iter().position(|&id| {
                !(self.links.iter()).any(|l| l.target == id && l.source != id && busy(l.source))
                    && !(self.deformed_before(id).iter()).any(|&d| {
                        pending.contains(&d)
                            || links::upstream(&self.links, d).iter().any(|&s| busy(s))
                    })
            });
            let Some(index) = ready else {
                return;
            };
            let Some(id) = self.update_queue.remove(index) else {
                return;
            };
            match self.start_solve(ctx, id) {
                Start::Started => self.updating.push(id),
                Start::Refused => {}
                Start::Later => {
                    // Its window is still busy: tried again in the next frame.
                    self.update_queue.insert(index, id);
                    return;
                }
            }
        }
    }

    /// Starts solving a copy of the analysis' data: of the project's, or of its open
    /// window's.
    fn start_solve(&mut self, ctx: &egui::Context, id: u64) -> Start {
        let incoming = self.incoming(id);
        let path = self.model_path();
        let mut work_dir = self.settings.solver.work_dir();
        work_dir.push(crate::analysis::job_name(&self.project_name()));
        // The analyses solved at the same time share the solver's one pool.
        let settings = self.settings.solver.clone();
        let force = self.forced.remove(&id);
        let Some(node) = self.nodes.iter_mut().find(|n| n.id == id) else {
            return Start::Refused;
        };
        let name = node.name.clone();
        let Some(analysis) = node.as_analysis_mut() else {
            return Start::Refused;
        };
        if analysis.window.as_ref().is_some_and(|w| w.busy().is_some()) {
            return Start::Later;
        }
        if !analysis.upstream.is_ready() {
            // Update Project could not bring them up to date: the old data is not solved.
            let upstream = &analysis.upstream;
            let mut reasons = Vec::new();
            if !upstream.stale_meshes.is_empty() {
                // Mesh blocks and CAD operations it takes its mesh or geometry from.
                reasons.push(format!(
                    "{} is not up to date",
                    upstream.stale_meshes.join(", ")
                ));
            }
            if !upstream.stale_results.is_empty() {
                reasons.push(format!(
                    "the results of {} are out of date",
                    upstream.stale_results.join(", ")
                ));
            }
            if !upstream.missing_results.is_empty() {
                reasons.push(format!(
                    "{} has no results yet",
                    upstream.missing_results.join(", ")
                ));
            }
            self.output
                .push(format!("{name} not solved: {}.", reasons.join("; ")));
            return Start::Refused;
        }
        let data = match &mut analysis.window {
            Some(window) => window.data(),
            None => analysis.data.clone(),
        };
        let Some(data) = data else {
            return Start::Later;
        };
        let data = ProjectAnalysis {
            id,
            name: name.clone(),
            ..data
        };
        work_dir.push(crate::analysis::job_dir(id));
        analysis.solving = true;
        node.refresh_closed();
        let solve = BackgroundSolve::start(
            ctx,
            path,
            data,
            settings,
            work_dir,
            incoming.continuation,
            force,
        );
        // A run of the same analysis before is replaced in the monitor.
        self.solves.retain(|s| s.id != id);
        self.solves.push(solve);
        self.monitor.show(id);
        crate::layout::fold_output(ctx, false);
        Start::Started
    }

    /// Polls the background solves and puts what a finished one made into its analysis.
    fn poll_solves(&mut self, ctx: &egui::Context) {
        let mut finished = Vec::new();
        for solve in self.solves.iter_mut().filter(|s| s.active()) {
            solve.poll(ctx);
            if !solve.active() {
                finished.push(solve.id);
            }
        }
        let mut lost = Vec::new();
        for id in finished {
            let Some(solve) = self.solves.iter_mut().find(|s| s.id == id) else {
                continue;
            };
            let solved = solve.take_solved();
            let Some(node) = self.node(id) else {
                continue;
            };
            let name = node.name.clone();
            let Some(analysis) = node.as_analysis_mut() else {
                continue;
            };
            analysis.solving = false;
            let Some(mut solved) = solved else {
                continue;
            };
            // What the solve made belongs to the mesh it started from: a mesh made or linked
            // meanwhile keeps the analysis out of date instead.
            let mesh = match (&analysis.window, &analysis.data) {
                (Some(window), _) => window.model().map(|m| m.mesh.fingerprint()),
                (None, Some(data)) => Some(data.mesh.fingerprint()),
                (None, None) => None,
            };
            if solved.mesh_print.is_some() && mesh != solved.mesh_print {
                if solved.meshed.is_some() || solved.results.is_some() {
                    lost.push(format!(
                        "{name}: the mesh changed while it was solved; its new results are not \
                         kept. Solve it again."
                    ));
                }
                solved.meshed = None;
                solved.results = None;
            }
            match (&mut analysis.window, &mut analysis.data) {
                (Some(window), _) => window.take_solved(solved),
                (None, Some(data)) => {
                    if let Some((mesh, mesh_inputs)) = solved.meshed {
                        data.mesh = mesh;
                        data.mesh_inputs = mesh_inputs;
                        // Reference points at the centre of a selection follow the new mesh.
                        data.model.update_reference_points(&data.mesh);
                    }
                    if let Some(mut results) = solved.results {
                        // What was defined on the results before goes over to the new ones;
                        // what refers to nodes or elements only if the mesh stayed the same.
                        let previous = (data.results.as_mut()).and_then(|r| r.settings.take());
                        results.settings = previous.map(|mut settings| {
                            if settings.mesh != Some(data.mesh.fingerprint()) {
                                settings.without_mesh_references();
                            }
                            settings
                        });
                        data.results = Some(results);
                    }
                }
                (None, None) => {}
            }
            node.refresh_closed();
            self.sync_due = true;
        }
        self.output.extend(lost);
    }

    /// Stops Update Project: nothing more is started, and what runs is stopped.
    fn stop_update(&mut self) {
        if let Some(run) = self.looping.take() {
            self.loops_due.clear();
            let name = self.name_of(run.id);
            if let Some(study) = self.node(run.id).and_then(Node::as_study_mut) {
                let line = format!("Stopped after {} points.", run.solved);
                study.report = Some(line.clone());
                study.active = None;
                self.output.push(format!("{name}: {line}"));
            }
            self.sync_due = true;
        }
        self.update_queue.clear();
        self.export_queue.clear();
        self.update_failed = false;
        self.updating.clear();
        self.forced.clear();
        for solve in &mut self.solves {
            solve.stop();
        }
        self.output.push("Update stopped".into());
    }

    /// Stops what runs and forgets the runs, for another project.
    fn forget_solves(&mut self) {
        for solve in &mut self.solves {
            solve.stop();
        }
        self.solves.clear();
        self.forced.clear();
        self.update_queue.clear();
        self.export_queue.clear();
        self.updating.clear();
    }

    /// The lines of the solution monitor's list: the analyses solved, then those queued.
    fn monitor_rows(&self) -> Vec<MonitorRow> {
        let mut rows: Vec<MonitorRow> = (self.solves.iter())
            .map(|solve| MonitorRow {
                id: solve.id,
                name: self.name_of(solve.id),
                state: solve.state_text().to_owned(),
                seconds: Some(solve.seconds()),
                progress: solve.progress(),
                active: solve.active(),
            })
            .collect();
        for &id in &self.update_queue {
            rows.push(MonitorRow {
                id,
                name: self.name_of(id),
                state: "queued".into(),
                seconds: None,
                progress: None,
                active: true,
            });
        }
        rows
    }

    /// The monitor for the whole project, in the Analyses tab below the schematic:
    /// the analyses being solved with their state on the left, the output of the one
    /// picked on the right.
    fn monitor_panel(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        let rows = self.monitor_rows();
        let selected = self.monitor.selected;
        let mut filter = self.monitor.filter.clone();
        let (mut pick, mut stop, mut stop_all, mut open_analysis, mut clear) =
            (None, None, false, None, false);
        let solve = selected.and_then(|id| self.solves.iter().find(|s| s.id == id));
        let output: Vec<&str> = solve.map(|s| s.output()).unwrap_or_default();
        let dropped = solve.map_or(0, |s| s.dropped());
        let profile = solve.filter(|s| !s.active()).and_then(|s| s.profile());
        let steps = solve.map(|s| s.step_lines().to_vec()).unwrap_or_default();
        ui.horizontal(|ui| {
            let row = rows.iter().find(|r| Some(r.id) == selected);
            let active = row.is_some_and(|r| r.active);
            if ui
                .add_enabled(active, egui::Button::new("Stop"))
                .on_hover_text("Stops the solve of the analysis picked in the list")
                .clicked()
            {
                stop = selected;
            }
            let any = rows.iter().any(|r| r.active);
            if ui
                .add_enabled(any, egui::Button::new("Stop All"))
                .on_hover_text("Stops Update Project and every solve")
                .clicked()
            {
                stop_all = true;
            }
            let tip = "Opens the window of the analysis picked in the list, with its results";
            if ui
                .add_enabled(row.is_some(), egui::Button::new("Open Analysis"))
                .on_hover_text(tip)
                .clicked()
            {
                open_analysis = selected;
            }
            let finished = rows.iter().any(|r| !r.active);
            if ui
                .add_enabled(finished, egui::Button::new("Clear Finished"))
                .clicked()
            {
                clear = true;
            }
            ui.separator();
            if let Some(row) = row {
                let info = crate::solver_profile::ReportInfo {
                    name: &row.name,
                    state: &row.state,
                    seconds: row.seconds.unwrap_or(0.0),
                    steps: &steps,
                };
                crate::solver_profile::buttons(ui, profile.as_ref(), &info);
                ui.separator();
            }
            ui.label("Filter:");
            ui.add(
                egui::TextEdit::singleline(&mut filter)
                    .hint_text("Show only lines containing ...")
                    .desired_width(180.0),
            );
            if ui
                .add_enabled(!filter.is_empty(), egui::Button::new("Clear"))
                .clicked()
            {
                filter.clear();
            }
            crate::analysis::dropped_note(ui, dropped);
        });
        ui.horizontal_top(|ui| {
            let height = ui.available_height();
            ui.allocate_ui_with_layout(
                egui::vec2(420.0, height),
                egui::Layout::top_down(egui::Align::Min),
                |ui| {
                egui::ScrollArea::vertical()
                    .id_salt("monitor list")
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        egui::Grid::new("monitor rows")
                            .num_columns(5)
                            .min_col_width(16.0)
                            .striped(true)
                            .spacing([16.0, 4.0])
                            .show(ui, |ui| {
                                for title in ["", "Analysis", "Status", "Time", "Progress"] {
                                    ui.strong(title);
                                }
                                ui.end_row();
                                if rows.is_empty() {
                                    ui.label("");
                                    ui.weak("Nothing solved yet: Update Project solves the analyses that are not up to date.");
                                    ui.end_row();
                                }
                                for row in &rows {
                                    let size = egui::vec2(16.0, 16.0);
                                    if row.active && row.state != "queued" {
                                        ui.add_sized(size, egui::Spinner::new());
                                    } else {
                                        ui.allocate_space(size);
                                    }
                                    let label =
                                        ui.selectable_label(selected == Some(row.id), &row.name);
                                    if label.clicked() {
                                        pick = Some(row.id);
                                    }
                                    if label.double_clicked() {
                                        open_analysis = Some(row.id);
                                    }
                                    ui.label(&row.state);
                                    ui.label(
                                        row.seconds
                                            .map_or_else(String::new, |s| format!("{s:.1} s")),
                                    );
                                    ui.label(row.progress.as_deref().unwrap_or(""));
                                    ui.end_row();
                                }
                            });
                    });
                },
            );
            ui.separator();
            // The lines go one below the other, not in the row of the list.
            ui.vertical(|ui| crate::solver_profile::with_output(ui, profile.as_ref(), |ui| {
                let area = egui::ScrollArea::both()
                    .id_salt(("monitor output", selected))
                    .auto_shrink([false, false])
                    .stick_to_bottom(true);
                if filter.is_empty() {
                    crate::virtual_table::lines(ui, area, &output);
                } else {
                    let needle = filter.to_lowercase();
                    let shown: Vec<&str> = (output.iter().copied())
                        .filter(|line| line.to_lowercase().contains(&needle))
                        .collect();
                    crate::virtual_table::lines(ui, area, &shown);
                }
            }));
        });
        self.monitor.filter = filter;
        if pick.is_some() {
            self.monitor.selected = pick;
        }
        if let Some(id) = stop {
            self.stop_solve(id);
        }
        if stop_all {
            self.stop_update();
        }
        if clear {
            self.solves.retain(BackgroundSolve::active);
        }
        if let Some(id) = open_analysis {
            self.open_window(
                &ctx,
                id,
                Some(Show {
                    view: TreeView::Results,
                    step: None,
                }),
            );
        }
    }

    /// Stops the solve of one analysis; a queued one is taken out of the queue.
    fn stop_solve(&mut self, id: u64) {
        self.update_queue.retain(|&queued| queued != id);
        self.forced.remove(&id);
        if let Some(solve) = (self.solves.iter_mut()).find(|s| s.id == id && s.active()) {
            solve.stop();
        }
    }

    /// F5, as Solve in an analysis window: Update Project, unless it is under way.
    fn solve_shortcut(&mut self) {
        if !self.busy() {
            self.update_all();
        }
    }

    /// Whether Update Project is under way, or a parameter study.
    fn busy(&self) -> bool {
        !self.updating.is_empty()
            || !self.update_queue.is_empty()
            || !self.export_queue.is_empty()
            || self.looping.is_some()
    }

    /// Switches the output pane between the monitor and the output; a hidden pane opens on
    /// the monitor.
    fn toggle_monitor(&mut self, ctx: &egui::Context) {
        let hidden = crate::layout::output_folded(ctx);
        self.monitor.shown = hidden || !self.monitor.shown;
        crate::layout::fold_output(ctx, false);
        if self.monitor.shown && self.monitor.selected.is_none() {
            self.monitor.selected = self.updating.first().copied();
        }
    }

    /// The analyses that take data from `id`, directly or through others.
    fn dependents(&self, id: u64) -> Vec<u64> {
        let mut found = Vec::new();
        let mut queue = vec![id];
        while let Some(source) = queue.pop() {
            for link in (self.links.iter()).filter(|l| l.source == source && !links::is_feedback(l))
            {
                if !found.contains(&link.target) && link.target != id {
                    found.push(link.target);
                    queue.push(link.target);
                }
            }
        }
        found
    }

    fn menu_bar(&mut self, ui: &mut egui::Ui) {
        {
            ui.menu_button("File", |ui| {
                if ui.button("New Project").clicked() {
                    self.new_project();
                }
                let open = egui::Button::new("Open …").shortcut_text("Ctrl+O");
                if ui.add_enabled(self.opening.is_none(), open).clicked() {
                    self.open_dialog(ui.ctx());
                }
                let tip = "Opens one of the projects opened last that still exist";
                let mut picked = None;
                ui.add_enabled_ui(self.opening.is_none(), |ui| {
                    ui.menu_button("Open Recent", |ui| picked = self.recent_panel(ui))
                        .response
                        .on_hover_text(tip);
                });
                if let Some(path) = picked {
                    self.open(ui.ctx(), path);
                    ui.close();
                }
                let save = egui::Button::new("Save").shortcut_text("Ctrl+S");
                if ui.add(save).clicked() {
                    self.save(false);
                }
                let save_as = egui::Button::new("Save As …").shortcut_text("Ctrl+Shift+S");
                if ui.add(save_as).clicked() {
                    self.save(true);
                }
                let tip = "Saves a copy with only the meshes and the model definitions, \
                           without the results the analyses keep. The open project keeps them.";
                if ui
                    .button("Save Without Results …")
                    .on_hover_text(tip)
                    .clicked()
                {
                    self.save_without_results();
                }
                ui.separator();
                if ui.button("Exit").clicked() {
                    ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
                }
            });
            ui.menu_button("Edit", |ui| {
                let ctx = ui.ctx().clone();
                let undo = egui::Button::new("Undo").shortcut_text("Ctrl+Z");
                if ui
                    .add_enabled(self.history.can_undo(), undo)
                    .on_hover_text(UNDO_TIP)
                    .clicked()
                {
                    self.undo(&ctx);
                }
                let redo = egui::Button::new("Redo").shortcut_text("Ctrl+Y");
                if ui.add_enabled(self.history.can_redo(), redo).clicked() {
                    self.redo(&ctx);
                }
                ui.separator();
                if ui.button("Settings …").clicked() {
                    self.settings_window = Some(SettingsWindow::new(&self.settings));
                }
            });
            ui.menu_button("Help", |ui| {
                let tip = "Opens the user guide in the web browser";
                if ui.button("User Guide").on_hover_text(tip).clicked() {
                    let line = crate::help::open_user_guide();
                    self.output.push(line);
                }
                let tip = "Opens the folder with the session log and the crash reports";
                if ui.button("Open Log Folder").on_hover_text(tip).clicked() {
                    self.output.push(crate::crash_log::open_log_dir());
                }
                let updater = &mut self.updater;
                if ui
                    .add_enabled(!updater.busy(), egui::Button::new("Check for Updates …"))
                    .on_hover_text("Asks GitHub whether a newer version of Nodalis is available")
                    .clicked()
                {
                    updater.check(ui.ctx(), true);
                }
            });
        }
    }

    /// A place for a new block: right of the last one.
    fn free_position(&self) -> egui::Pos2 {
        (self.graph.nodes_pos())
            .map(|(pos, _)| pos)
            .max_by(|a, b| a.x.total_cmp(&b.x))
            .map_or(egui::pos2(40.0, 40.0), |pos| pos + egui::vec2(260.0, 0.0))
    }

    fn schematic(&mut self, ui: &mut egui::Ui) -> Vec<Action> {
        let mut incoming: HashMap<u64, Vec<(Vec<String>, String)>> = HashMap::new();
        let mut continuations: HashMap<u64, Continuation> = HashMap::new();
        let mut bundles: HashMap<u64, Vec<Bundle>> = HashMap::new();
        for link in &self.links {
            if let Some(source) = self.nodes.iter().find(|n| n.id == link.source) {
                (incoming.entry(link.target).or_default())
                    .extend(links::filled_paths(link, &source.name));
                if links::is_state(link) {
                    continuations.insert(
                        link.target,
                        Continuation {
                            source: source.name.clone(),
                            step: link.item.get(1).cloned().unwrap_or_default(),
                            inherited: link.inherited.clone(),
                        },
                    );
                }
                if links::is_bundle(link) {
                    bundles.entry(link.target).or_default().push(Bundle {
                        path: link.item.clone(),
                        source: source.name.clone(),
                        taken: link.inherited.len(),
                        excluded: link.excluded.len(),
                    });
                }
            }
        }
        let base = self.export_base();
        let mut blocks: Vec<BlockView> = (self.nodes.iter())
            .map(|n| {
                let fe = n.link_data().map(|d| d.fe);
                let rows = match (n.kind(), &n.status()) {
                    (Kind::Materials | Kind::CrossSections, _) => fe
                        .zip(n.collection())
                        .map(|(fe, collection)| {
                            let mut rows = collection_rows(collection, fe, &n.expanded);
                            let incoming = incoming.get(&n.id).map_or(&[][..], Vec::as_slice);
                            rows.extend(exposed_rows(fe, &n.expanded, incoming));
                            rows
                        })
                        .unwrap_or_default(),
                    (Kind::Mesh, _) => {
                        mesh_rows(n, incoming.get(&n.id).map_or(&[][..], Vec::as_slice))
                    }
                    (Kind::MeshExport, _) => export_rows(
                        n,
                        base.as_deref(),
                        incoming.get(&n.id).map_or(&[][..], Vec::as_slice),
                    ),
                    (Kind::File, _) => file_rows(n, base.as_deref()),
                    (Kind::Parameters, _) => {
                        parameter_rows(n, incoming.get(&n.id).map_or(&[][..], Vec::as_slice))
                    }
                    (Kind::Study, _) => self.study_rows(n, &incoming),
                    (Kind::Campbell, _) => campbell_rows(
                        n,
                        incoming.get(&n.id).map_or(&[][..], Vec::as_slice),
                        self.campbell_state(n.id),
                    ),
                    (Kind::Measure, _) => measure_rows(
                        n,
                        incoming.get(&n.id).map_or(&[][..], Vec::as_slice),
                        self.measure_state(n.id),
                        self.measuring.contains_key(&n.id),
                    ),
                    (Kind::RefineMesh, _) => {
                        refine_rows(n, incoming.get(&n.id).map_or(&[][..], Vec::as_slice))
                    }
                    (Kind::DeformedMesh, _) => deformed::deformed_rows(
                        n,
                        incoming.get(&n.id).map_or(&[][..], Vec::as_slice),
                        n.as_deformed()
                            .map_or(CellState::Missing, DeformedBlock::state),
                        (deformed::deformed_link(&self.links, n.id))
                            .and_then(|l| self.nodes.iter().find(|s| s.id == l.source))
                            .map_or_else(UnitSystem::default, Node::units),
                    ),
                    (Kind::CadOperation, _) => {
                        cad::cad_rows(n, incoming.get(&n.id).map_or(&[][..], Vec::as_slice))
                    }
                    (Kind::Analysis, Some(status)) => block_rows(
                        status,
                        fe,
                        &n.expanded,
                        incoming.get(&n.id).map_or(&[][..], Vec::as_slice),
                        bundles.get(&n.id).map_or(&[][..], Vec::as_slice),
                        continuations.get(&n.id),
                    ),
                    (Kind::Math | Kind::Switch, _) => {
                        numbers::number_rows(n, incoming.get(&n.id).map_or(&[][..], Vec::as_slice))
                    }
                    (Kind::Analysis, None) | (Kind::Note, _) => Vec::new(),
                };
                let continues = (self.links.iter())
                    .find(|l| l.target == n.id && links::is_state(l))
                    .map(|l| self.name_of(l.source));
                let chain = match (n.kind(), fe) {
                    (Kind::Analysis, Some(fe)) => chain_marks(fe, &rows),
                    _ => HashMap::new(),
                };
                BlockView {
                    id: n.id,
                    name: n.name.clone(),
                    export_path: (n.as_export().map(|e| &e.path))
                        .or(n.as_file().map(|f| &f.path))
                        .cloned()
                        .unwrap_or_default(),
                    kind: n.kind(),
                    space: n.space().unwrap_or_default(),
                    has_space: n.space().is_some(),
                    steps: (fe.into_iter().flat_map(|fe| &fe.steps))
                        .map(|s| s.name.clone())
                        .collect(),
                    continues,
                    rows,
                    loading: n.window().is_some_and(|w| w.model().is_none()),
                    open: n.window().is_some(),
                    primitive: n.as_mesh().map(|m| m.primitive.clone()),
                    refinement: n.refinement().unwrap_or(1.0),
                    deform: n
                        .as_deformed()
                        .map_or((1.0, 1), |d| (d.factor, d.increment)),
                    cad_length: n.as_cad().map_or(0.0, |c| c.operation.value),
                    point: n.as_measure().and_then(|m| m.kind.point()),
                    parameters: (n.as_parameters()).map_or_else(Vec::new, |p| p.parameters.clone()),
                    exposed: n.exposing_fe().map_or_else(Vec::new, ExposedView::of),
                    units: n.units(),
                    note: match &n.block {
                        Block::Note(text) => text.clone(),
                        _ => String::new(),
                    },
                    compact: self.compact.contains(&n.id),
                    number: n.as_number().map(|(b, switch)| (b.clone(), switch)),
                    framed: frames::frame_of(&self.frames, n.id).is_some(),
                    chain,
                }
            })
            .collect();
        // A compact block keeps the rows its wires end at.
        if blocks.iter().any(|b| b.compact) {
            let ends = self.wired_rows(&blocks);
            for block in blocks.iter_mut().filter(|b| b.compact) {
                let kept: BTreeSet<usize> = (ends.iter())
                    .flat_map(|&((source, output), (target, input))| {
                        [
                            (source == block.id).then_some(output),
                            (target == block.id).then_some(input),
                        ]
                    })
                    .flatten()
                    .collect();
                let mut index = 0;
                block.rows.retain(|_| {
                    index += 1;
                    kept.contains(&(index - 1))
                });
                for row in &mut block.rows {
                    row.expander = None;
                }
            }
        }
        self.sync_wires(&blocks);
        let verdicts = self.drag_verdicts(ui.ctx(), &blocks);
        let mut viewer = Viewer {
            drag: self.dragging.clone(),
            verdicts,
            nodes: blocks,
            actions: Vec::new(),
            to_global: Default::default(),
            show_all: std::mem::take(&mut self.show_all).then(|| ui.max_rect()),
            look_at: self.look_at.take().map(|p| (ui.max_rect(), p)),
            feedback: (self.links.iter())
                .filter(|l| links::is_feedback(l))
                .map(|l| (l.source, l.item.clone(), l.target, l.slot.clone()))
                .collect(),
            rects: Vec::new(),
            busy: self.busy(),
            frames: self.frames.clone(),
            sizes: std::mem::take(&mut self.block_sizes),
            frame_label: self.frame_label.take(),
            drawn: HashMap::new(),
            focus_note: self.focus_note.take(),
            selected: self.selected.clone(),
        };
        let snarl_id = egui::Id::new(("project schematic", self.graph_generation));
        // Text on the blocks is not selectable: a drag on it moves the block.
        let selectable = ui.style().interaction.selectable_labels;
        ui.style_mut().interaction.selectable_labels = false;
        let response = SnarlWidget::new()
            .id(snarl_id)
            .style(schematic_style())
            .show(&mut self.graph, &mut viewer, ui);
        ui.style_mut().interaction.selectable_labels = selectable;
        // Blocks not drawn this frame keep their last size.
        self.block_sizes = std::mem::take(&mut viewer.sizes);
        self.block_sizes.extend(viewer.drawn.drain());
        self.frame_label = viewer.frame_label.take();
        let selected: Vec<u64> = (egui_snarl::ui::get_selected_nodes(snarl_id, ui.ctx()))
            .into_iter()
            .filter_map(|g| self.graph.get_node(g).copied())
            .collect();
        // A click on a block selects it alone, as in Blender; Shift adds to the selection.
        let mut selected = selected;
        let layer = egui::LayerId::new(ui.layer_id().order, snarl_id);
        if let Some(id) = self.clicked_block(ui, layer, viewer.to_global) {
            if let Some(node) = self.graph_node(id) {
                egui_snarl::ui::set_selected_nodes(snarl_id, ui.ctx(), &[node]);
            }
            selected = vec![id];
        }
        // An analysis selected alone becomes the one Geometry, FE Model and Results show,
        // unless it is open in a window of its own.
        if selected != self.selected
            && let [id] = selected[..]
            && self
                .nodes
                .iter()
                .any(|n| n.id == id && n.kind() == Kind::Analysis)
            && !self.in_own_window(id)
        {
            self.active = Some(id);
        }
        self.selected = selected;
        let clip = viewer.to_global.inverse() * ui.clip_rect();
        draw_feedback(ui.ctx(), &viewer.feedback, &viewer.rects, clip);
        self.pin_rects = ui.ctx().data_mut(|d| {
            d.remove_temp::<Vec<(PinKey, egui::Rect)>>(egui::Id::new(PIN_RECTS))
                .unwrap_or_default()
        });
        self.show_refusal(ui, &viewer.verdicts);
        // A double-click or right-click into empty space shows the blocks to add there, as in
        // Blender.
        // The response lies on the zoomed and panned layer of the schematic, so its pointer
        // position is in the schematic; the picker needs the one on the screen.
        if self.placing.is_none()
            && (response.double_clicked() || response.secondary_clicked())
            && let Some(screen) = ui.ctx().input(|i| i.pointer.interact_pos())
        {
            let graph = viewer.to_global.inverse() * screen;
            self.picker = Some(NodePicker::new(screen, graph));
        }
        self.schematic_keys(ui, ui.max_rect(), viewer.to_global.inverse());
        self.minimap(ui.ctx(), ui.max_rect(), viewer.to_global);
        if let Some(picker) = &mut self.picker {
            let kinds = block_kinds();
            let entries: Vec<_> = kinds.iter().map(|k| k.entry()).collect();
            match picker.show(ui.ctx(), &entries) {
                node_picker::Outcome::Open => {}
                node_picker::Outcome::Closed => self.picker = None,
                node_picker::Outcome::Picked(i) => {
                    viewer.actions.push(kinds[i].action(picker.graph));
                    self.picker = None;
                }
            }
        }
        self.place_block(ui, viewer.to_global.inverse(), &mut viewer.actions);
        if self.nodes.is_empty() {
            ui.put(
                egui::Rect::from_center_size(ui.max_rect().center(), egui::vec2(420.0, 60.0)),
                egui::Label::new(
                    egui::RichText::new(
                        "Place an analysis, materials or a mesh with the tiles of the ribbon,\n\
                         or double-click here to search the blocks. Double-click an analysis\n\
                         to open it in Geometry, FE Model and Results.",
                    )
                    .weak(),
                )
                // Clicks go through to the schematic below.
                .selectable(false),
            );
        }
        viewer.actions
    }

    /// The block a plain click went to in this frame, if any.
    fn clicked_block(
        &self,
        ui: &egui::Ui,
        layer: egui::LayerId,
        to_global: egui::emath::TSTransform,
    ) -> Option<u64> {
        let ctx = ui.ctx();
        let (clicked, modifiers, pos) = ctx.input(|i| {
            (
                i.pointer.primary_clicked(),
                i.modifiers,
                i.pointer.interact_pos(),
            )
        });
        let pos = pos?;
        if !clicked || modifiers.shift || modifiers.command || self.placing.is_some() {
            return None;
        }
        // Not through a window or menu over the schematic.
        let over = ctx.layer_id_at(pos);
        if !ui.max_rect().contains(pos) || (over != Some(layer) && over != Some(ui.layer_id())) {
            return None;
        }
        let at = to_global.inverse() * pos;
        (self.graph.nodes_pos_ids())
            .filter(|(_, min, id)| {
                let size = self
                    .block_sizes
                    .get(id)
                    .copied()
                    .unwrap_or(egui::vec2(220.0, 60.0));
                egui::Rect::from_min_size(*min, size).contains(at)
            })
            .map(|(_, _, id)| *id)
            .last()
    }

    /// Finds the pin a wire is dragged from, by where the drag started, and for each pin of the
    /// other side why the wire cannot end there; empty while no wire is dragged.
    fn drag_verdicts(
        &mut self,
        ctx: &egui::Context,
        blocks: &[BlockView],
    ) -> HashMap<PinKey, Option<String>> {
        let (down, origin, dragging) = ctx.input(|i| {
            let p = &i.pointer;
            (
                p.primary_down(),
                p.press_origin(),
                p.is_decidedly_dragging(),
            )
        });
        if !down {
            self.dragging = None;
        } else if self.dragging.is_none()
            && dragging
            && let Some(origin) = origin
        {
            self.dragging = (self.pin_rects.iter())
                .find(|(_, rect)| rect.expand(2.0).contains(origin))
                .map(|(key, _)| key.clone());
        }
        let Some(drag) = &self.dragging else {
            return HashMap::new();
        };
        let mut verdicts = HashMap::new();
        for block in blocks {
            for row in &block.rows {
                let other = if drag.output { row.input } else { row.output };
                if !other {
                    continue;
                }
                let key = PinKey {
                    id: block.id,
                    path: row.path.clone(),
                    output: !drag.output,
                };
                let why = if drag.output {
                    refusal(
                        &self.nodes,
                        &self.links,
                        drag.id,
                        &drag.path,
                        block.id,
                        &row.path,
                    )
                } else {
                    refusal(
                        &self.nodes,
                        &self.links,
                        block.id,
                        &row.path,
                        drag.id,
                        &drag.path,
                    )
                };
                verdicts.insert(key, why);
            }
        }
        verdicts
    }

    /// Says why the wire being dragged cannot end at the pin under the pointer.
    fn show_refusal(&self, ui: &egui::Ui, verdicts: &HashMap<PinKey, Option<String>>) {
        let Some(pointer) = ui.ctx().pointer_hover_pos() else {
            return;
        };
        let refused = (self.pin_rects.iter())
            .filter(|(_, rect)| rect.expand(2.0).contains(pointer))
            .find_map(|(key, _)| verdicts.get(key).cloned().flatten());
        if let Some(why) = refused {
            let id = egui::Id::new("schematic link refusal");
            egui::Tooltip::always_open(
                ui.ctx().clone(),
                ui.layer_id(),
                id,
                egui::PopupAnchor::Pointer,
            )
            .show(|ui| ui.label(format!("Cannot link: {why}")));
        }
    }

    /// Lets the wires of the schematic show the project's links, each between the rows its
    /// ends are shown in.
    /// The rows the wires of the links end at: block and row of the source, block and row
    /// of the target.
    fn wired_rows(&self, blocks: &[BlockView]) -> Vec<((u64, usize), (u64, usize))> {
        let block = |id: u64| blocks.iter().find(|b| b.id == id);
        let mut wired = Vec::new();
        for link in &self.links {
            let (Some(source), Some(target)) = (block(link.source), block(link.target)) else {
                continue;
            };
            // A link of a category is a bundle of links of its items: a wire per item
            // between the rows that show it, one wire while both categories are closed.
            let paths: Vec<(Vec<String>, Vec<String>)> =
                if links::is_bundle(link) && !link.inherited.is_empty() {
                    (link.inherited.iter())
                        .map(|name| {
                            let mut path = link.item.clone();
                            path.push(name.clone());
                            (path.clone(), path)
                        })
                        .collect()
                } else {
                    vec![(links::output_path(link), links::slot(link).to_vec())]
                };
            for (output, input) in paths {
                let output = shown_row(&source.rows, &output);
                // The global results of a submodel end at its Global results row while the
                // input of the step is not shown.
                let input = match shown_row(&target.rows, &input) {
                    Some(row) if target.rows[row].path != input => (target.rows.iter())
                        .position(|r| {
                            r.path == [links::GLOBAL_RESULTS] && links::ends_at(&r.path, link)
                        })
                        .or(Some(row)),
                    row => row,
                };
                if let (Some(output), Some(input)) = (output, input) {
                    wired.push(((link.source, output), (link.target, input)));
                }
            }
        }
        wired
    }

    fn sync_wires(&mut self, blocks: &[BlockView]) {
        let mut wanted = std::collections::HashSet::new();
        for ((source, output), (target, input)) in self.wired_rows(blocks) {
            if let (Some(from), Some(to)) = (self.graph_node(source), self.graph_node(target)) {
                wanted.insert((
                    egui_snarl::OutPinId { node: from, output },
                    egui_snarl::InPinId { node: to, input },
                ));
            }
        }
        let wires: Vec<_> = self.graph.wires().collect();
        for wire in wires {
            if !wanted.remove(&wire) {
                self.graph.disconnect(wire.0, wire.1);
            }
        }
        for (from, to) in wanted {
            self.graph.connect(from, to);
        }
    }

    fn name_of(&self, id: u64) -> String {
        (self.nodes.iter().find(|n| n.id == id)).map_or_else(String::new, |n| n.name.clone())
    }

    /// Links `item` of `source` to `slot` of `target`, in place of the links that fill the
    /// same part of the target.
    fn link(&mut self, source: u64, item: Vec<String>, target: u64, slot: Vec<String>) {
        let (from, to) = (self.name_of(source), self.name_of(target));
        if source == target {
            return;
        }
        if let Some(why) = refusal(&self.nodes, &self.links, source, &item, target, &slot) {
            self.output.push(format!("Not linked: {why}"));
            return;
        }
        let (item, slot) = self.study_column(source, item, target, slot);
        let feedback = item.first().is_some_and(|c| c == links::LOOP);
        if slot.first().is_some_and(|c| c == links::MEASURE) {
            self.link_measure(source, item, target);
            return;
        }
        if slot.first().is_some_and(|c| c == links::STEP_RESULT) {
            self.link_deformed(source, item, target);
            return;
        }
        if slot.first().is_some_and(|c| c == links::ANALYSIS) {
            self.link_campbell(source, item, target);
            return;
        }
        if slot.first().is_some_and(|c| c == links::STATE) {
            self.link_state(source, item, target);
            return;
        }
        let value = item.first().is_some_and(|c| c == links::VALUE) || feedback;
        let objective = slot.first().is_some_and(|c| c == links::OUTPUTS);
        let results = item.first().is_some_and(|c| c == links::RESULTS);
        let part = matches!(item.as_slice(), [cell, _] if cell == links::GEOMETRY);
        // A category or item links into the same place of the target; a step's results go
        // to the step they are dropped on, a part of a geometry to the Geometry row.
        let slot = if results {
            // What the results are used for follows from the steps at both ends.
            let fe = |id: u64| {
                (self.nodes.iter().find(|n| n.id == id))
                    .and_then(Node::link_data)
                    .map(|d| d.fe.clone())
            };
            let (Some(source_fe), Some(target_fe)) = (fe(source), fe(target)) else {
                return;
            };
            let step = item
                .get(1)
                .and_then(|n| source_fe.steps.iter().find(|s| &s.name == n));
            let Some(step) = step else {
                return;
            };
            match links::results_slot(step, &target_fe, &slot) {
                Ok(slot) => slot,
                Err(why) => {
                    self.output.push(format!(
                        "Not linked: {} of {from} cannot go to {} of {to}: {why}.",
                        links::label(&item),
                        links::label(&slot)
                    ));
                    return;
                }
            }
        } else if value || part {
            slot
        } else {
            item.clone()
        };
        // An item of a category the target takes from the same source already: taken
        // again if it was left out.
        if let [cell, category, name] = slot.as_slice()
            && cell == links::SETUP
            && !results
            && let Some(bundle) = self.links.iter_mut().find(|l| {
                l.source == source
                    && l.target == target
                    && links::is_bundle(l)
                    && links::setup_category(l) == Some(category)
            })
        {
            let text = match bundle.excluded.iter().position(|n| n == name) {
                Some(at) => {
                    bundle.excluded.remove(at);
                    format!("{to} takes {name} from {from} again, with the rest of {category}.")
                }
                None => format!("{to} takes {name} from {from} already, with all of {category}."),
            };
            self.output.push(text);
            self.sync_due = true;
            return;
        }
        // What the source has of the category a category or an item goes to.
        let source_entries = match slot.as_slice() {
            [cell, category, ..] if cell == links::SETUP && !results => (self.nodes.iter())
                .find(|n| n.id == source)
                .and_then(Node::link_data)
                .map(|d| links::entries(d.fe, category))
                .unwrap_or_default(),
            _ => Vec::new(),
        };
        let usage = |link: &Link| links::ResultsUse::of(link);
        let new_usage = usage(&Link {
            source,
            target,
            item: item.clone(),
            slot: slot.clone(),
            ..Default::default()
        });
        let overlaps = |link: &Link| {
            if link.target != target || links::is_results(link) != results {
                return false;
            }
            if results {
                return usage(link) == new_usage
                    && match new_usage {
                        // A submodel reads one global results file.
                        Some(links::ResultsUse::Submodel) => {
                            link.source != source || links::slot(link) == slot.as_slice()
                        }
                        // A step takes one temperature field.
                        Some(links::ResultsUse::Temperature) => {
                            links::slot(link) == slot.as_slice()
                        }
                        // The model takes one set of eigenmodes and of initial temperatures.
                        _ => true,
                    };
            }
            let other = links::slot(link);
            // The parts of a geometry linked from the same source go together.
            if part && links::is_geometry_part(link) && link.source == source {
                return link.item == item;
            }
            // A category is a collection: links of categories and items of it from other
            // sources stay, and the later one wins an item both have.
            if let ([a, b, ..], [c, d, ..]) = (other, slot.as_slice())
                && a == links::SETUP
                && c == links::SETUP
            {
                return b == d
                    && match (other.len(), slot.len()) {
                        (3, 3) => other == slot.as_slice(),
                        (2, 2) => link.source == source,
                        (3, 2) => link.source == source || source_entries.contains(&other[2]),
                        _ => false,
                    };
            }
            let mesh_and_geometry = |a: &[String], b: &[String]| {
                a.first().is_some_and(|c| c == links::MESH)
                    && b.first().is_some_and(|c| c == links::GEOMETRY)
            };
            links::covers(other, &slot)
                || links::covers(&slot, other)
                || mesh_and_geometry(other, &slot)
                || mesh_and_geometry(&slot, other)
        };
        let replaced: Vec<Link> = self.links.iter().filter(|l| overlaps(l)).cloned().collect();
        self.links.retain(|l| !overlaps(l));
        for link in replaced {
            self.output.push(format!(
                "{to} no longer takes {} from {}.",
                links::label(&link.item),
                self.name_of(link.source)
            ));
        }
        // An item a category link of another source offers too comes from here now.
        if let [_, category, name] = slot.as_slice()
            && !results
        {
            let others: Vec<String> = (self.links.iter())
                .filter(|l| {
                    l.target == target
                        && links::is_bundle(l)
                        && links::setup_category(l) == Some(category)
                        && l.inherited.contains(name)
                })
                .map(|l| self.name_of(l.source))
                .collect();
            for other in others {
                self.output.push(format!(
                    "{to} takes {name} from {from} in place of {other}."
                ));
            }
        }
        let (what, step) = (links::label(&item), slot.get(2).map_or("", String::as_str));
        let text = match new_usage {
            Some(links::ResultsUse::Submodel) => {
                format!("{to} takes the displacements of {what} of {from} in {step} (submodel).")
            }
            Some(links::ResultsUse::Temperature) => {
                format!("{to} takes the temperatures of {what} of {from} in {step}.")
            }
            Some(links::ResultsUse::Modes) => format!(
                "{to} takes the eigenmodes of {what} of {from} for its modal and steady state \
                 dynamics steps."
            ),
            Some(links::ResultsUse::InitialTemperatures) => {
                format!("{to} starts at the temperatures of {what} of {from}.")
            }
            None if objective => format!("{to} reads the value of {from}."),
            None if feedback => format!(
                "{} of {to} takes the value of {} of {from}, which varies it.",
                self.parameter_label(target, &slot),
                item.get(1).map_or("", String::as_str)
            ),
            None if value => format!(
                "{} of {to} takes the value of {} of {from}.",
                self.parameter_label(target, &slot),
                item.get(1).map_or("", String::as_str)
            ),
            None => format!("{to} takes {} from {from}.", links::label(&item)),
        };
        self.output.push(text);
        self.applied.retain(|(id, _), _| *id != target);
        self.links.push(Link {
            source,
            target,
            slot: if results || value || part {
                slot
            } else {
                Vec::new()
            },
            item,
            ..Default::default()
        });
        self.sync_due = true;
    }

    /// Makes the measure block `target` measure the results of a step of `source`, in place
    /// of those it measured.
    fn link_measure(&mut self, source: u64, item: Vec<String>, target: u64) {
        let (from, to) = (self.name_of(source), self.name_of(target));
        let replaced: Vec<Link> = (self.links.iter())
            .filter(|l| l.target == target)
            .cloned()
            .collect();
        self.links.retain(|l| l.target != target);
        for link in replaced {
            self.output.push(format!(
                "{to} no longer measures {} of {}.",
                links::label(&link.item),
                self.name_of(link.source)
            ));
        }
        self.output
            .push(format!("{to} measures {} of {from}.", links::label(&item)));
        self.links.push(Link {
            source,
            target,
            item,
            slot: vec![links::MEASURE.to_owned()],
            ..Default::default()
        });
        self.sync_due = true;
    }

    /// Makes the deformed mesh block `target` take the displacements of the results of a step
    /// of `source`, in place of those it took; value links to its numbers stay.
    fn link_deformed(&mut self, source: u64, item: Vec<String>, target: u64) {
        let (from, to) = (self.name_of(source), self.name_of(target));
        let replaced = |l: &Link| {
            l.target == target && matches!(links::slot(l), [cell] if cell == links::STEP_RESULT)
        };
        for link in self.links.iter().filter(|l| replaced(l)) {
            self.output.push(format!(
                "{to} no longer takes {} of {}.",
                links::label(&link.item),
                self.name_of(link.source)
            ));
        }
        self.links.retain(|l| !replaced(l));
        self.output.push(format!(
            "{to} deforms the mesh of {from} by the displacements of {}.",
            links::label(&item)
        ));
        self.links.push(Link {
            source,
            target,
            item,
            slot: vec![links::STEP_RESULT.to_owned()],
            ..Default::default()
        });
        self.sync_due = true;
    }

    /// Makes the Campbell block `target` sweep the analysis `source`, whose setup `item` is,
    /// in place of the one it swept.
    fn link_campbell(&mut self, source: u64, item: Vec<String>, target: u64) {
        let (from, to) = (self.name_of(source), self.name_of(target));
        let kind = |id: u64| self.nodes.iter().find(|n| n.id == id).map(|n| n.kind());
        if kind(source) != Some(Kind::Analysis) || kind(target) != Some(Kind::Campbell) {
            self.output.push(format!(
                "Not linked: {to} sweeps an analysis, and {from} is none."
            ));
            return;
        }
        let replaced: Vec<Link> = (self.links.iter())
            .filter(|l| l.target == target)
            .cloned()
            .collect();
        self.links.retain(|l| l.target != target);
        for link in replaced {
            self.output.push(format!(
                "{to} no longer sweeps {}.",
                self.name_of(link.source)
            ));
        }
        self.output.push(format!(
            "{to} sweeps {from} over the speed of its centrifugal loads."
        ));
        self.links.push(Link {
            source,
            target,
            item,
            slot: vec![links::ANALYSIS.to_owned()],
            ..Default::default()
        });
        self.sync_due = true;
    }

    /// Makes `target` continue `source` after the step whose results `item` names: it takes
    /// the source's model and its steps up to that one, in place of all it took before.
    fn link_state(&mut self, source: u64, item: Vec<String>, target: u64) {
        let (from, to) = (self.name_of(source), self.name_of(target));
        let Some(step) = item.get(1).cloned() else {
            return;
        };
        let replaced: Vec<Link> = (self.links.iter())
            .filter(|l| l.target == target)
            .cloned()
            .collect();
        self.links.retain(|l| l.target != target);
        for link in replaced {
            self.output.push(format!(
                "{to} no longer takes {} from {}.",
                links::label(&link.item),
                self.name_of(link.source)
            ));
        }
        self.output.push(format!(
            "{to} continues {from} from the state after {step}: it takes the model of {from} \
             and its steps up to {step}; its own steps follow."
        ));
        self.applied.retain(|(id, _), _| *id != target);
        self.links.push(Link {
            source,
            target,
            item: vec![links::STATE.to_owned(), step],
            slot: vec![links::STATE.to_owned()],
            ..Default::default()
        });
        self.sync_due = true;
    }

    /// A new analysis that continues `source` after `step`, with its model space and units.
    fn continue_from(&mut self, source: u64, step: String, position: egui::Pos2) -> Option<u64> {
        let properties = self.node(source)?.link_data()?.fe.properties.clone();
        let name = format!("{} continued", self.name_of(source));
        let id = self.add_analysis(&name, position, properties);
        self.output.push(format!("{} created", self.name_of(id)));
        let item = vec![links::RESULTS.to_owned(), step];
        self.link(source, item, id, vec![links::STATE.to_owned()]);
        Some(id)
    }

    /// Splits the analysis after `step`: the steps after it go to a new analysis that
    /// continues it, and what took their results takes them from there.
    fn split(&mut self, id: u64, step: String) {
        let Some(fe) = self
            .node(id)
            .and_then(|n| n.link_data())
            .map(|d| d.fe.clone())
        else {
            return;
        };
        let Some(index) = fe.steps.iter().position(|s| s.name == step) else {
            return;
        };
        if index + 1 >= fe.steps.len() {
            return;
        }
        if let Some(window) = self.node(id).and_then(|n| n.window())
            && let Some(reason) = window.busy()
        {
            self.output.push(format!("Not split while {reason}."));
            return;
        }
        let moved: Vec<String> = fe.steps[index + 1..]
            .iter()
            .map(|s| s.name.clone())
            .collect();
        let position = (self.graph_node(id))
            .and_then(|g| self.graph.get_node_info(g))
            .map_or(self.free_position(), |info| {
                info.pos + egui::vec2(420.0, 0.0)
            });
        let Some(new) = self.continue_from(id, step.clone(), position) else {
            return;
        };
        let kept = match self.node(new).and_then(Node::closed_model_mut) {
            Some(data) => links::split_after(&fe, index, data.fe),
            None => links::split_after(&fe, index, &mut plx_model::FeModel::default()),
        };
        // What took the results or state of a moved step takes them from the new analysis.
        for link in &mut self.links {
            let moved_step = link.item.get(1).is_some_and(|s| moved.contains(s));
            if link.source == id && link.target != new && moved_step {
                link.source = new;
            }
        }
        let path = [links::SETUP.to_owned(), "Steps".to_owned()];
        self.set_steps(id, &kept, &path);
        self.output.push(format!(
            "{} split after {step}: {} continues it with {}.",
            self.name_of(id),
            self.name_of(new),
            moved.join(", ")
        ));
    }

    /// Gives the own steps of the analysis to the one it continues and removes it; only when
    /// it continues from the last step of that one.
    fn merge(&mut self, id: u64) {
        let Some(link) = (self.links.iter())
            .find(|l| l.target == id && links::is_state(l))
            .cloned()
        else {
            return;
        };
        let (name, into) = (self.name_of(id), self.name_of(link.source));
        let source = self
            .node(link.source)
            .and_then(|n| n.link_data())
            .map(|d| d.fe.clone());
        let target = self
            .node(id)
            .and_then(|n| n.link_data())
            .map(|d| d.fe.clone());
        let (Some(mut source), Some(target)) = (source, target) else {
            return;
        };
        if source.steps.last().map(|s| &s.name) != link.item.get(1) {
            self.output.push(format!(
                "Not merged: {name} continues {into} from a step before its last; only an \
                 analysis that continues from the last step can be merged into it."
            ));
            return;
        }
        if !self.close_window(id) {
            return;
        }
        let moved = links::merge_into(&mut source, &target, &link);
        let path = [links::SETUP.to_owned(), "Steps".to_owned()];
        self.set_steps(link.source, &source, &path);
        // What took results or states of the merged analysis takes them from the other one.
        for other in &mut self.links {
            if other.source == id {
                other.source = link.source;
            }
        }
        self.delete(id);
        self.output.push(format!(
            "{name} merged into {into}: {into} solves {} too.",
            moved.join(", ")
        ));
        self.sync_due = true;
    }

    /// Replaces the steps of an analysis with those of `fe`.
    fn set_steps(&mut self, id: u64, fe: &plx_model::FeModel, path: &[String]) {
        let Some(node) = self.node(id) else {
            return;
        };
        match node.window_mut() {
            Some(window) => window.apply_setup(fe, path),
            None => {
                if let Some(data) = (node.as_analysis_mut()).and_then(|a| a.data.as_mut()) {
                    links::apply_setup(&mut data.model, fe, path);
                }
                node.refresh_closed();
            }
        }
        self.sync_due = true;
    }

    /// Leaves out an item of the categories that links take as a whole, named by the pin of
    /// its row in the source or in the target: the target keeps its copy, which is its own
    /// from then on.
    fn exclude(
        &mut self,
        source: Option<&(u64, Vec<String>)>,
        target: Option<&(u64, Vec<String>)>,
    ) {
        let item = |end: Option<&(u64, Vec<String>)>, id: u64, category: &[String]| {
            end.map(|(at, path)| {
                (*at == id && path.len() == category.len() + 1 && links::covers(category, path))
                    .then(|| path[category.len()].clone())
            })
        };
        let mut left_out = Vec::new();
        for link in self.links.iter_mut().filter(|l| links::is_bundle(l)) {
            let name = match (
                item(source, link.source, &link.item),
                item(target, link.target, &link.item),
            ) {
                (Some(Some(a)), Some(Some(b))) if a == b => a,
                (Some(Some(a)), None) | (None, Some(Some(a))) => a,
                _ => continue,
            };
            if link.inherited.contains(&name) && !link.excluded.contains(&name) {
                link.excluded.push(name.clone());
                link.inherited.retain(|n| *n != name);
                left_out.push((link.source, link.target, name));
            }
        }
        for (source, target, name) in left_out {
            self.output.push(format!(
                "{} no longer takes {name} from {}; it keeps its copy.",
                self.name_of(target),
                self.name_of(source)
            ));
            self.sync_due = true;
        }
    }

    /// Removes the links that `remove` picks; the targets keep their copies. A target that
    /// loses its last results link is no submodel any more.
    fn unlink(&mut self, remove: impl Fn(&Link) -> bool) {
        let removed: Vec<Link> = self.links.iter().filter(|l| remove(l)).cloned().collect();
        self.links.retain(|l| !remove(l));
        for link in removed {
            let target = link.target;
            self.output.push(format!(
                "{} no longer takes {} from {}; it keeps its copy.",
                self.name_of(target),
                links::label(&link.item),
                self.name_of(link.source)
            ));
            self.applied
                .retain(|(id, path), _| *id != target || path != &link.item);
            let submodel = (self.links.iter()).any(|l| {
                l.target == target && links::ResultsUse::of(l) == Some(links::ResultsUse::Submodel)
            });
            if links::is_results(&link)
                && let Some(node) = self.node(target)
            {
                match node.window_mut() {
                    Some(window) => window.clear_results(&link, submodel),
                    None => {
                        if let Some(data) = (node.as_analysis_mut()).and_then(|a| a.data.as_mut()) {
                            links::clear_results(&mut data.model, &link, submodel);
                        }
                        node.refresh_closed();
                    }
                }
            }
        }
        self.sync_due = true;
    }

    /// Brings what the analyses take through links up to date with their sources once
    /// something changed, in the order of the links so that a chain is done in one go.
    fn sync_links(&mut self, ctx: &egui::Context) {
        self.poll_restoring();
        if self.restoring.values().any(Option::is_some) {
            ctx.request_repaint_after(Duration::from_millis(100));
        }
        self.refresh_files(ctx);
        // A new value of a measure goes to the parameter study that waits for it.
        if !self.measuring.is_empty() {
            ctx.request_repaint_after(Duration::from_millis(100));
            if self.poll_measures() {
                self.sync_due = true;
            }
        }
        if self.deforming() {
            ctx.request_repaint_after(Duration::from_millis(100));
            if self.poll_deformed() {
                self.sync_due = true;
            }
        }
        // Results that went out of date, or up to date again, change what the analyses that
        // take them are: however that came about, the links pass it on.
        if self.freshness() != self.synced_freshness {
            self.sync_due = true;
        }
        if !self.sync_due {
            return;
        }
        self.sync_due = false;
        self.refresh_number_kinds();
        self.apply_values();
        self.evaluate_measures();
        self.evaluate_deformed();
        let ids: Vec<u64> = self.nodes.iter().map(|n| n.id).collect();
        for target in links::order(&ids, &self.links) {
            // A deformed mesh block reads the results of its step itself, see
            // [`Self::evaluate_deformed`].
            if (self.nodes.iter()).any(|n| n.id == target && n.kind() == Kind::DeformedMesh) {
                continue;
            }
            if (self.nodes.iter()).any(|n| n.id == target && n.kind() == Kind::RefineMesh) {
                take_refine_input(&mut self.nodes, &self.links, target);
                continue;
            }
            if (self.nodes.iter()).any(|n| n.id == target && n.kind() == Kind::CadOperation) {
                cad::take_cad_input(&mut self.nodes, &self.links, target);
                continue;
            }
            let to_target: Vec<Link> = (self.links.iter())
                .filter(|l| l.target == target)
                .cloned()
                .collect();
            let mut upstream = crate::status::Upstream::default();
            for link in &to_target {
                let source = self.name_of(link.source);
                if let Some(payload) = self.payload(link, &mut upstream)
                    && let Some(inherited) = self.apply(target, &source, link, payload)
                    && let Some(own) = self.links.iter_mut().find(|l| *l == link)
                {
                    own.inherited = inherited;
                }
            }
            if self
                .nodes
                .iter()
                .any(|n| n.id == target && n.kind() == Kind::Campbell)
            {
                let input = (self.campbell_source(target))
                    .and_then(Node::link_data)
                    .map(|d| crate::campbell::sweep_fingerprint(d.mesh, d.fe));
                if let Some(campbell) = self.node(target).and_then(Node::as_campbell_mut) {
                    campbell.input = input;
                }
                continue;
            }
            let incoming = self.incoming(target);
            upstream.stale_meshes = self.stale_meshes(target);
            if let Some(node) = self.node(target) {
                let Block::Analysis(analysis) = &mut node.block else {
                    continue;
                };
                // A busy window takes nothing now; what it would miss waits for it.
                if let Some(window) = &analysis.window
                    && (window.busy().is_some() || window.model().is_none())
                {
                    analysis.sync_missed = true;
                    continue;
                }
                analysis.upstream = upstream.clone();
                match &mut analysis.window {
                    Some(window) => {
                        window.set_links(incoming, upstream);
                        // Analyses after it in this pass see its state as it is now.
                        if let Some(status) = window.status() {
                            analysis.status = Some(status);
                        }
                    }
                    None => node.refresh_closed(),
                }
            }
        }
        self.synced_freshness = self.freshness();
        self.refresh_study_bodies();
        ctx.request_repaint();
    }

    /// Per analysis, the state of its solution and results as its block shows them.
    fn freshness(&self) -> Vec<(u64, CellState, CellState)> {
        (self.nodes.iter())
            .filter_map(|n| n.status().map(|s| (n.id, s.solution, s.results)))
            .collect()
    }

    /// What links fill in `target`, and from where it continues and where others continue
    /// it.
    fn incoming(&self, target: u64) -> links::Incoming {
        let mut incoming = links::Incoming::default();
        for link in self.links.iter().filter(|l| l.target == target) {
            let source = self.name_of(link.source);
            incoming.paths.extend(links::filled_paths(link, &source));
            if links::is_state(link) {
                (incoming.continuation.inherited_steps).extend(
                    link.inherited
                        .iter()
                        .map(|step| (step.clone(), source.clone())),
                );
            }
        }
        let fe = |id: u64| {
            (self.nodes.iter().find(|n| n.id == id))
                .and_then(Node::link_data)
                .map(|d| d.fe)
        };
        let dir =
            (self.settings.solver.work_dir()).join(crate::analysis::job_name(&self.project_name()));
        // The analyses it continues, nearest first.
        let mut at = target;
        while let Some(link) = (self.links.iter()).find(|l| l.target == at && links::is_state(l)) {
            at = link.source;
            if at == target {
                break;
            }
            let job = crate::analysis::job_dir(at);
            incoming.continuation.upstream.push((dir.join(&job), job));
        }
        // The own steps of those that continue it, and of those that continue them.
        if let Some(own) = fe(target) {
            for link in self
                .links
                .iter()
                .filter(|l| l.source == target && links::is_state(l))
            {
                let Some(index) = links::state_step(own, link) else {
                    continue;
                };
                let mut steps = Vec::new();
                let mut queue = vec![link.clone()];
                while let Some(link) = queue.pop() {
                    if let Some(fe) = fe(link.target) {
                        steps.extend(links::own_steps(fe, &link));
                    }
                    queue.extend(
                        (self.links.iter())
                            .filter(|l| l.source == link.target && links::is_state(l))
                            .cloned(),
                    );
                }
                incoming.continuation.dependent_steps.push((index, steps));
            }
        }
        incoming
    }

    /// What `link` has to copy into its target to bring it up to date with the source;
    /// `None` when it is up to date or cannot be brought up to date now. Sources of results
    /// links that have no results go to `missing`.
    fn payload(&mut self, link: &Link, upstream: &mut crate::status::Upstream) -> Option<Payload> {
        let source = self.nodes.iter().find(|n| n.id == link.source)?;
        let target = self.nodes.iter().find(|n| n.id == link.target)?;
        // A Campbell block reads the analysis when its sweep starts.
        if target.kind() == Kind::Campbell {
            return None;
        }
        if let Some(window) = target.window()
            && (window.busy().is_some() || window.model().is_none())
        {
            return None;
        }
        let data = source.link_data();
        let path = &link.item;
        match path.first().map(String::as_str) {
            Some(links::RESULTS) if source.as_file().is_some() => {
                let file = source.as_file()?;
                let step = path.get(1)?;
                let &(_, global) = file.steps.iter().find(|(name, _)| name == step)?;
                if file.reading.is_some() || file.error.is_some() || file.read.is_none() {
                    upstream.missing_results.push(source.name.clone());
                    return None;
                }
                let usage = links::ResultsUse::of(link)?;
                if usage == links::ResultsUse::Modes {
                    return None;
                }
                if usage != links::ResultsUse::Submodel
                    && target.link_data()?.mesh.input_fingerprint()
                        != file.model.mesh.input_fingerprint()
                {
                    let line = format!(
                        "{} cannot take the {} of {}: the meshes differ. Temperatures are \
                         taken node by node.",
                        target.name,
                        links::label(&link.item),
                        source.name
                    );
                    if self.warned.insert(line.clone()) {
                        self.output.push(line);
                    }
                    return None;
                }
                // A copy named after the file as it was read, so that a changed file makes
                // the target out of date.
                let base = self.export_base();
                let original = export_file(&file.path, base.as_deref());
                let name = original.file_name()?.to_owned();
                let dir = (self.settings.solver.work_dir())
                    .join(crate::analysis::job_name(&self.project_name()))
                    .join(crate::analysis::job_dir(target.id))
                    .join(format!(
                        "File-{}-{:016x}",
                        crate::analysis::job_dir(source.id),
                        file.read.unwrap_or_default()
                    ));
                let copy = dir.join(name);
                if !copy.is_file()
                    && let Err(why) = std::fs::create_dir_all(&dir)
                        .and_then(|_| std::fs::copy(&original, &copy).map(|_| ()))
                {
                    let line = format!("{}: {} not copied: {why}", source.name, original.display());
                    if self.warned.insert(line.clone()) {
                        self.output.push(line);
                    }
                    upstream.missing_results.push(source.name.clone());
                    return None;
                }
                Some(Payload::Results(global, copy))
            }
            Some(links::RESULTS) => {
                let step = path.get(1)?;
                let Some(bound) = source.bound_results() else {
                    upstream.missing_results.push(source.name.clone());
                    return None;
                };
                if source.out_of_date_for(link) {
                    // Its results were made from data that changed since: the target waits
                    // for it to be solved again rather than take them.
                    upstream.stale_results.push(source.name.clone());
                    return None;
                }
                let data = data?;
                let global = links::global_step_number(data.fe, step)?;
                let usage = links::ResultsUse::of(link)?;
                let key = source.step_key(step);
                if usage != links::ResultsUse::Submodel
                    && target.link_data()?.mesh.input_fingerprint() != data.mesh.input_fingerprint()
                {
                    // Temperatures and eigenmodes are taken node by node.
                    let line = format!(
                        "{} cannot take the {} of {}: their meshes differ. Link the mesh, or \
                         continue from a state, so that both have the same nodes.",
                        target.name,
                        links::label(&link.item),
                        source.name
                    );
                    if self.warned.insert(line.clone()) {
                        self.output.push(line);
                    }
                    return None;
                }
                if usage == links::ResultsUse::Modes {
                    // The solver writes them into the run's own folder, where the next run of
                    // the source overwrites them. The target takes a copy in a folder named
                    // after the solved model, so that new modes make it out of date.
                    let project = crate::analysis::job_name(&self.project_name());
                    let job = crate::analysis::job_dir(source.id);
                    let solved = (self.settings.solver.work_dir().join(&project).join(&job))
                        .join(format!("{job}.eig"));
                    let dir = (self.settings.solver.work_dir().join(project))
                        .join(crate::analysis::job_dir(target.id))
                        .join(format!(
                            "Modes-{job}-{:016x}",
                            bound.input.unwrap_or_default()
                        ));
                    let file = dir.join(format!("{job}.eig"));
                    // From the source's run, or else from its kept results, as on another
                    // computer.
                    let copied = || {
                        std::fs::create_dir_all(&dir).is_ok()
                            && std::fs::copy(&solved, &file).is_ok()
                    };
                    let name = format!("{job}.eig");
                    let restored = || {
                        plx_io::project::restore_file(bound, &name, &file).is_ok_and(|done| done)
                    };
                    if !file.is_file() && !copied() && !restored() {
                        upstream.missing_results.push(source.name.clone());
                        return None;
                    }
                    return Some(Payload::Results(global, file));
                }
                // CalculiX's results file, which results of the built-in solver are
                // written as for this.
                let frd = bound.calculix_results_name()?;
                let dir = (self.settings.solver.work_dir())
                    .join(crate::analysis::job_name(&self.project_name()))
                    .join(crate::analysis::job_dir(target.id))
                    .join(format!(
                        "Global-{}-{:016x}",
                        crate::analysis::job_dir(source.id),
                        key
                    ));
                let file = dir.join(&frd);
                if !file.is_file() {
                    match self.restoring.get(&dir) {
                        Some(Some(_)) => upstream.missing_results.push(source.name.clone()),
                        Some(None) => {}
                        None => {
                            let (sender, receiver) = std::sync::mpsc::channel();
                            let (bound, folder) = (bound.clone(), dir.clone());
                            std::thread::spawn(move || {
                                let written =
                                    plx_io::project::restore_calculix_results(&bound, &folder)
                                        .map_err(|e| e.to_string());
                                let _ = sender.send(written);
                            });
                            self.restoring.insert(dir, Some(receiver));
                            upstream.missing_results.push(source.name.clone());
                        }
                    }
                }
                Some(Payload::Results(global, file))
            }
            Some(links::STATE) => {
                if source.bound_results().is_none() {
                    // Solved first, so that this one continues from its states.
                    upstream.missing_results.push(source.name.clone());
                } else if source.out_of_date_for(link) {
                    // Its states belong to data that changed since.
                    upstream.stale_results.push(source.name.clone());
                }
                let (data, own) = (data?, target.link_data()?);
                let step = links::state_step(data.fe, link)?;
                let mut print = plx_mesh::Fnv::new();
                for cell in [links::GEOMETRY, links::MESH] {
                    let path = [cell.to_owned()];
                    print.add(&links::fingerprint(data, &path).to_le_bytes());
                }
                // The mesh copy carries the mesh setup too.
                print.add_debug(&data.geometry.map(|g| (&g.meshing, &g.mesh_items)));
                let print = print.finish();
                let key = (link.target, vec![links::STATE.to_owned()]);
                let mesh = (self.applied.get(&key) != Some(&print))
                    .then(|| Box::new(links::MeshCopy::of(data)));
                let fe = (!links::chain_up_to_date(data.fe, step, own.fe, link))
                    .then(|| Box::new(data.fe.clone()));
                if mesh.is_none() && fe.is_none() {
                    return None;
                }
                self.applied.insert(key, print);
                Some(Payload::Chain { mesh, fe, step })
            }
            Some(links::GEOMETRY) if links::is_geometry_part(link) => {
                // The first of the part links of a source to the target takes them all.
                let first = (self.links.iter()).find(|l| {
                    l.source == link.source && l.target == link.target && links::is_geometry_part(l)
                });
                if first != Some(link) {
                    return None;
                }
                let parts = links::linked_parts(&self.links, link.source, link.target);
                let source = self.node(link.source)?;
                let name = source.name.clone();
                let file = source.as_file_mut()?;
                let read = file.read?;
                let geometry = match file.subset(&parts) {
                    Ok(geometry) => geometry?,
                    Err(why) => {
                        let line = format!("{name}: the parts linked are not taken: {why}.");
                        if self.warned.insert(line.clone()) {
                            self.output.push(line);
                        }
                        return None;
                    }
                };
                let mut print = plx_mesh::Fnv::new();
                print.add(&read.to_le_bytes());
                print.add_debug(&parts);
                let print = print.finish();
                let key = (link.target, vec![links::GEOMETRY.to_owned()]);
                if self.applied.get(&key) == Some(&print) {
                    return None;
                }
                self.applied.insert(key, print);
                Some(Payload::Geometry(Some(geometry)))
            }
            Some(links::GEOMETRY | links::MESH) => {
                let data = data?;
                if !links::exists(data, path) {
                    return None;
                }
                let print = links::fingerprint(data, path);
                let key = (link.target, path.clone());
                if self.applied.get(&key) == Some(&print) {
                    return None;
                }
                self.applied.insert(key, print);
                Some(if path[0] == links::MESH {
                    Payload::Mesh(Box::new(links::MeshCopy::of(data)))
                } else {
                    Payload::Geometry(data.geometry.cloned())
                })
            }
            Some(links::SETUP) => {
                let (data, own) = (data?, target.link_data()?);
                if let Some(category) = links::setup_category(link) {
                    // The links of a category and its items to the same target: a later one
                    // wins an item both offer.
                    let at = self.links.iter().position(|l| l == link)?;
                    let same = |l: &&Link| {
                        l.target == link.target && links::setup_category(l) == Some(category)
                    };
                    let offers = |l: &Link| -> Vec<String> {
                        let Some(fe) = (self.nodes.iter())
                            .find(|n| n.id == l.source)
                            .and_then(Node::link_data)
                            .map(|d| d.fe)
                        else {
                            return Vec::new();
                        };
                        let present = links::entries(fe, category);
                        (links::offered(fe, l).into_iter())
                            .filter(|name| present.contains(name))
                            .collect()
                    };
                    let later: Vec<String> = self.links[at + 1..]
                        .iter()
                        .filter(same)
                        .flat_map(offers)
                        .collect();
                    if links::is_bundle(link) {
                        let names: Vec<String> = (links::offered(data.fe, link).into_iter())
                            .filter(|name| !later.contains(name))
                            .collect();
                        if links::bundle_up_to_date(data.fe, own.fe, link, &names) {
                            return None;
                        }
                        let ahead = self.links[..at]
                            .iter()
                            .filter(same)
                            .flat_map(offers)
                            .collect();
                        return Some(Payload::Bundle {
                            fe: Box::new(data.fe.clone()),
                            names,
                            ahead,
                        });
                    }
                    if link.item.get(2).is_some_and(|name| later.contains(name)) {
                        return None;
                    }
                }
                if !links::exists(data, path)
                    || links::fingerprint(data, path) == links::fingerprint(own, path)
                {
                    return None;
                }
                Some(Payload::Setup(Box::new(data.fe.clone())))
            }
            _ => None,
        }
    }

    /// Copies what a link takes into its target, see [`ProjectApp::payload`]; returns the
    /// steps a state link took.
    fn apply(
        &mut self,
        target: u64,
        source: &str,
        link: &Link,
        payload: Payload,
    ) -> Option<Vec<String>> {
        let node = self.node(target)?;
        let path = &link.item;
        if let Payload::Chain { mesh, fe, step } = payload {
            let mut inherited = None;
            match node.window_mut() {
                Some(window) => {
                    if let Some(mesh) = mesh {
                        window.apply_mesh(*mesh);
                    }
                    if let Some(fe) = fe {
                        inherited = Some(window.apply_chain(&fe, step, link));
                    }
                    window.message(format!("Model and steps taken from {source}"));
                }
                None => {
                    let mut data = node.closed_model_mut()?;
                    if let Some(mesh) = mesh {
                        data.take_mesh(*mesh);
                    }
                    if let Some(fe) = fe {
                        inherited = Some(links::apply_chain(data.fe, &fe, step, link));
                    }
                }
            }
            return inherited;
        }
        if let Payload::Bundle { fe, names, ahead } = payload {
            let taken = match node.window_mut() {
                Some(window) => {
                    let taken = window.apply_bundle(&fe, link, &names, &ahead);
                    window.message(format!("{} taken from {source}", links::label(path)));
                    taken
                }
                None => {
                    let data = node.closed_model_mut()?;
                    links::apply_bundle(data.fe, &fe, link, &names, &ahead)
                }
            };
            return Some(taken);
        }
        if let Some(window) = node.window_mut() {
            match payload {
                Payload::Setup(fe) => {
                    window.apply_setup(&fe, path);
                    window.message(format!("{} taken from {source}", links::label(path)));
                }
                Payload::Geometry(geometry) => {
                    window.apply_geometry(geometry);
                    window.message(format!("Geometry taken from {source}"));
                }
                Payload::Mesh(mesh) => {
                    window.apply_mesh(*mesh);
                    window.message(format!("Mesh taken from {source}"));
                }
                Payload::Results(global, file) => window.apply_results(link, global, file),
                Payload::Chain { .. } | Payload::Bundle { .. } => {}
            }
            return None;
        }
        let mut data = node.closed_model_mut()?;
        match payload {
            Payload::Setup(fe) => links::apply_setup(data.fe, &fe, path),
            Payload::Geometry(geometry) => {
                if let Some(tags) = links::take_geometry(data.geometry, geometry)
                    && let Some(lost) = links::follow_geometry(data.fe, Some(data.mesh), &tags)
                {
                    let line = format!("{}: {lost}.", node.name);
                    self.output.push(line);
                }
            }
            Payload::Mesh(mesh) => data.take_mesh(*mesh),
            Payload::Results(global, file) => {
                links::apply_results(data.fe, link, global, file);
            }
            Payload::Chain { .. } | Payload::Bundle { .. } => {}
        }
        None
    }

    /// Picks up global results written for submodels.
    fn poll_restoring(&mut self) {
        let mut done = Vec::new();
        for (dir, receiver) in &self.restoring {
            if let Some(receiver) = receiver
                && let Ok(result) = receiver.try_recv()
            {
                done.push((dir.clone(), result));
            }
        }
        for (dir, result) in done {
            match result {
                Ok(_) => {
                    self.restoring.remove(&dir);
                }
                Err(error) => {
                    self.output
                        .push(format!("Global results not written: {error}"));
                    // Not tried again until the project is opened again.
                    self.restoring.insert(dir, None);
                }
            }
            self.sync_due = true;
        }
    }

    /// Solves the analyses whose results `id` needs and that have none yet, in the order of
    /// the links, then `id`.
    fn solve_chain(&mut self, id: u64) {
        // The solve waits for the links to bring a new mesh.
        self.generate_meshes();
        let mut sources = Vec::new();
        let mut taken = Vec::new();
        let mut queue = vec![id];
        while let Some(target) = queue.pop() {
            for link in self.links.iter().filter(|l| l.target == target) {
                let solved_first = links::is_results(link) || links::is_state(link);
                if solved_first {
                    taken.push(link);
                    if !sources.contains(&link.source) {
                        sources.push(link.source);
                        queue.push(link.source);
                    }
                }
            }
        }
        // Those with results made from data that changed since are solved again too, unless
        // only steps after the ones taken changed; a file block is read, not solved.
        sources.retain(|s| {
            (self.nodes.iter()).any(|n| {
                n.id == *s
                    && n.kind() == Kind::Analysis
                    && (n.bound_results().is_none()
                        || (taken.iter()).any(|l| l.source == *s && n.out_of_date_for(l)))
            })
        });
        sources.push(id);
        let chain = links::order(&sources, &self.links);
        self.update_queue.retain(|queued| !chain.contains(queued));
        for id in chain.into_iter().rev() {
            self.update_queue.push_front(id);
        }
    }

    /// The analyses whose data `action` rewrites, which undo has to keep; `None` for an
    /// action that changes nothing undo brings back, such as opening a window or solving.
    fn rewritten_by(&self, action: &Action) -> Option<Vec<u64>> {
        Some(match action {
            Action::New(_)
            | Action::Open(..)
            | Action::OpenInWindow(_)
            | Action::Solve(_)
            | Action::Update(_)
            | Action::Rename(_)
            | Action::Toggle(..)
            | Action::Generate(_)
            | Action::ExportNow(_)
            | Action::ReadFile(_)
            | Action::RunStudy(_)
            | Action::DeleteAll(_)
            | Action::Pick(..)
            // Blocks moved are noticed on their own.
            | Action::MoveFrame(..) => return None,
            Action::Link { target, .. } => vec![*target],
            Action::SetExposed(id, ..)
            | Action::RenameExposed(id, ..)
            | Action::Unexpose(id, _) => {
                vec![*id]
            }
            Action::Unlink { source, target } => (self.links.iter())
                .filter(|l| {
                    source.as_ref().is_some_and(|(id, _)| l.source == *id)
                        || target.as_ref().is_some_and(|(id, _)| l.target == *id)
                })
                .map(|l| l.target)
                .collect(),
            Action::Split(id, _) => vec![*id],
            // The analysis it continues takes its steps.
            Action::Merge(id) => (self.links.iter())
                .filter(|l| l.target == *id && links::is_state(l))
                .map(|l| l.source)
                .collect(),
            Action::AddMaterials(_)
            | Action::AddCrossSections(_)
            | Action::AddMesh(..)
            | Action::AddRefineMesh(_)
            | Action::SetRefinement(..)
            | Action::SetCadLength(..)
            | Action::SetMeasurePoint(..)
            | Action::AddCadOperation(_)
            | Action::AddDeformedMesh(_)
            | Action::SetDeformFactor(..)
            | Action::SetDeformIncrement(..)

            | Action::AddMeshExport(_)
            | Action::AddCampbell(_)
            | Action::AddFile(_)
            | Action::AddParameters(_)
            | Action::AddMeasure(_)
            | Action::AddStudy(_)
            | Action::SetExportPath(..)
            | Action::SetPrimitive(..)
            | Action::ResetStudy(_)
            | Action::SetParameter(..)
            | Action::Continue { .. }
            | Action::AddNote(_)
            | Action::SetNote(..)
            | Action::DuplicateAll(_)
            | Action::Compact(_)
            | Action::Frame(_)
            | Action::Unframe(_)
            | Action::FrameLabel(..)
            | Action::FrameColor(..)
            | Action::RemoveFrame(_)
            | Action::AddMath(_)
            | Action::AddSwitch(_)
            | Action::SetOperation(..)
            | Action::SetNumberInput(..)
            | Action::SetChoices(..) => Vec::new(),
        })
    }

    fn act(&mut self, ctx: &egui::Context, action: Action) {
        if let Some(rewritten) = self.rewritten_by(&action) {
            self.checkpoint(&rewritten);
        }
        match action {
            Action::New(pos) => self.ask_new_analysis(pos),
            Action::AddMaterials(pos) => {
                self.add_materials(pos);
            }
            Action::AddCrossSections(pos) => {
                self.add_collection(pos, Collection::CrossSections);
            }
            Action::AddMesh(pos, primitive) => {
                self.add_mesh(pos, primitive);
            }
            Action::AddParameters(pos) => {
                self.add_parameters(pos);
            }
            Action::AddMeasure(pos) => {
                self.add_measure(pos);
            }
            Action::AddStudy(pos) => {
                self.add_study(pos);
            }
            Action::RunStudy(id) => self.start_study(id),
            Action::ResetStudy(id) => self.reset_study(id),
            Action::SetParameter(id, index, value) => {
                if let Some(parameter) = (self.node(id).and_then(Node::as_parameters_mut))
                    .and_then(|p| p.parameters.get_mut(index))
                {
                    parameter.value = parameter.kind.clean(value);
                }
                self.sync_due = true;
            }
            Action::SetRefinement(id, value) => {
                if let Some(node) = self.node(id) {
                    node.set_refinement(value);
                }
                self.sync_due = true;
            }
            Action::SetCadLength(id, value) => {
                if let Some(cad) = self.node(id).and_then(Node::as_cad_mut)
                    && value.is_finite()
                {
                    cad.operation.value = value;
                }
                self.sync_due = true;
            }
            Action::SetMeasurePoint(id, key, value) => {
                if let Some(measure) = self.node(id).and_then(Node::as_measure_mut) {
                    measure.kind.set_coordinate(key, value);
                }
                self.sync_due = true;
            }
            Action::AddCadOperation(pos) => {
                self.add_cad_operation(pos);
            }
            Action::AddDeformedMesh(pos) => {
                self.add_deformed_mesh(pos);
            }
            Action::SetDeformFactor(id, value) => {
                if let Some(block) = self.node(id).and_then(Node::as_deformed_mut) {
                    block.set_factor(value);
                }
                self.sync_due = true;
            }
            Action::SetDeformIncrement(id, value) => {
                if let Some(block) = self.node(id).and_then(Node::as_deformed_mut) {
                    block.set_increment(value);
                }
                self.sync_due = true;
            }

            Action::SetExposed(id, key, value) => {
                if let Some(node) = self.node(id) {
                    node.set_exposed(key, value);
                }
                self.sync_due = true;
            }
            Action::RenameExposed(id, key, name) => {
                let taken = (self.nodes.iter().find(|n| n.id == id))
                    .and_then(Node::exposing_fe)
                    .is_some_and(|fe| fe.exposed.iter().any(|e| e.name == name && e.key != key));
                if taken {
                    self.output
                        .push(format!("{name} is the name of another parameter."));
                } else if let Some(exposed) = (self.node(id))
                    .and_then(Node::exposing_fe_mut)
                    .and_then(|fe| fe.exposed.iter_mut().find(|e| e.key == key))
                {
                    exposed.name = name;
                }
            }
            Action::Unexpose(id, key) => self.unexpose(id, key),
            Action::AddRefineMesh(pos) => {
                self.add_refine_mesh(pos);
            }
            Action::AddMeshExport(pos) => {
                self.add_mesh_export(pos);
            }
            Action::AddCampbell(pos) => {
                self.add_campbell(pos);
            }
            Action::AddFile(pos) => {
                self.add_file(pos);
            }
            Action::ReadFile(id) => {
                if let Some(file) = self.node(id).and_then(Node::as_file_mut) {
                    file.read = None;
                    file.error = None;
                }
                self.files_checked = None;
            }
            Action::SetExportPath(id, path) => {
                if let Some(export) = self.node(id).and_then(Node::as_export_mut) {
                    export.path = path;
                    export.error = None;
                } else if self.node(id).and_then(Node::as_file_mut).is_some() {
                    self.set_source_path(id, path);
                }
            }
            Action::ExportNow(id) => {
                // The mesh the link brings comes first.
                self.sync_due = true;
                self.sync_links(ctx);
                self.run_export(id, true);
            }
            Action::Generate(id) => self.generate_mesh(id),
            Action::SetPrimitive(id, primitive) => {
                if let Some(mesh) = self.node(id).and_then(Node::as_mesh_mut) {
                    mesh.primitive = primitive;
                }
                // Refine mesh blocks after it refine the new parameters.
                self.sync_due = true;
            }
            Action::Open(id, view) => self.open_window(ctx, id, view),
            Action::OpenInWindow(id) => self.open_analysis(ctx, id, None, true),
            Action::Solve(id) => self.solve_chain(id),
            Action::Update(id) => self.update_block(id),
            Action::Rename(id) => {
                let name = self
                    .nodes
                    .iter()
                    .find(|n| n.id == id)
                    .map(|n| n.name.clone());
                self.rename = name.map(|name| (id, name));
            }
            Action::DeleteAll(ids) => self.confirm_delete = ids,
            Action::DuplicateAll(ids) => {
                for id in ids {
                    self.duplicate(id);
                }
            }
            Action::AddNote(pos) => {
                self.add_note(pos);
            }
            Action::SetNote(id, text) => {
                if let Some(Block::Note(note)) = self.node(id).map(|n| &mut n.block) {
                    *note = text;
                }
            }
            Action::Compact(ids) => self.toggle_compact(&ids),
            Action::Frame(ids) => self.frame_blocks(&ids),
            Action::Unframe(ids) => self.leave_frames(&ids),
            Action::MoveFrame(index, delta) => self.move_frame(index, delta),
            Action::FrameLabel(index, label) => {
                if let Some(frame) = self.frames.get_mut(index) {
                    frame.label = label;
                }
            }
            Action::FrameColor(index, color) => {
                if let Some(frame) = self.frames.get_mut(index) {
                    frame.color = color;
                }
            }
            Action::RemoveFrame(index) => {
                if index < self.frames.len() {
                    self.frames.remove(index);
                }
                self.frame_label = None;
            }
            Action::Pick(screen, graph) => self.picker = Some(NodePicker::new(screen, graph)),
            Action::AddMath(pos) => {
                self.add_math(pos);
            }
            Action::AddSwitch(pos) => {
                self.add_switch(pos);
            }
            Action::SetOperation(id, operation) => self.set_operation(id, operation),
            Action::SetNumberInput(id, key, value) => {
                if let Some((number, switch)) = self.node(id).and_then(Node::as_number_mut) {
                    number.set(switch, &key, value);
                }
                self.sync_due = true;
            }
            Action::SetChoices(id, count) => self.set_choices(id, count),
            Action::Toggle(id, path) => {
                if let Some(node) = self.node(id)
                    && !node.expanded.remove(&path)
                {
                    node.expanded.insert(path);
                }
            }
            Action::Link {
                source,
                item,
                target,
                slot,
            } => self.link(source, item, target, slot),
            Action::Continue {
                source,
                step,
                position,
            } => {
                self.continue_from(source, step, position);
            }
            Action::Split(id, step) => self.split(id, step),
            Action::Merge(id) => self.merge(id),
            Action::Unlink { source, target } => {
                self.exclude(source.as_ref(), target.as_ref());
                self.unlink(|link| {
                    source.as_ref().is_none_or(|(id, path)| {
                        link.source == *id && links::covers(path, &links::output_path(link))
                    }) && target
                        .as_ref()
                        .is_none_or(|(id, path)| link.target == *id && links::ends_at(path, link))
                })
            }
        }
    }

    fn rename_window(&mut self, ctx: &egui::Context) {
        let Some((id, mut name)) = self.rename.take() else {
            return;
        };
        let mut open = true;
        let (mut done, mut cancelled) = (false, false);
        let valid = !name.trim().is_empty();
        let noun = (self.nodes.iter().find(|n| n.id == id)).map_or("analysis", |n| n.kind().noun());
        egui::Window::new(format!("Rename {}", title_case(noun)))
            .id(crate::ids::window(ctx, "rename"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Name:");
                    let edit = ui.text_edit_singleline(&mut name);
                    // Asking for the focus on the frame it is lost would keep Enter from
                    // being seen.
                    if !edit.has_focus() && !edit.lost_focus() {
                        edit.request_focus();
                    }
                    if edit.lost_focus() {
                        if ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                            done = valid;
                        } else if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                            cancelled = true;
                        }
                    }
                });
                ui.horizontal(|ui| {
                    if ui.add_enabled(valid, egui::Button::new("OK")).clicked() {
                        done = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancelled = true;
                    }
                });
            });
        if cancelled {
            open = false;
        }
        if done {
            self.checkpoint(&[]);
            let name = name.trim().to_owned();
            if let Some(node) = self.node(id) {
                node.name = name.clone();
                if let Some(window) = node.window_mut() {
                    window.rename(&name);
                }
            }
            if let Some(editor) = self.material_set.as_mut().filter(|e| e.id == id) {
                editor.rename(&name);
            }
            if let Some(editor) = self.mesh_block.as_mut().filter(|e| e.id == id) {
                editor.rename(&name);
            }
            if let Some(editor) = self.cad_editor.as_mut().filter(|e| e.id == id) {
                editor.rename(&name);
            }
            if let Some(editor) = self.parameter_set.as_mut().filter(|e| e.id == id) {
                editor.rename(&name);
            }
            if let Some(editor) = self.measure_editor.as_mut().filter(|e| e.id == id) {
                editor.rename(&name);
            }
            if let Some(editor) = self.study_editor.as_mut().filter(|e| e.id == id) {
                editor.rename(&name);
            }
        } else if open {
            self.rename = Some((id, name));
        }
    }

    /// Deletes the blocks asked to be deleted, without asking: Undo brings them back, which
    /// the toast over the schematic offers (decisions 3 and 16 of the GUI concept).
    fn confirm_delete_window(&mut self, _ctx: &egui::Context) {
        self.confirm_delete
            .retain(|id| self.nodes.iter().any(|n| n.id == *id));
        let names: Vec<String> = (self.confirm_delete.iter())
            .filter_map(|id| self.nodes.iter().find(|n| n.id == *id))
            .map(|n| n.name.clone())
            .collect();
        let text = match names.as_slice() {
            [] => return,
            [name] => format!("Deleted {name}"),
            _ => format!("Deleted {} blocks", names.len()),
        };
        self.toast = Some(crate::toast::Toast::new(text, true));
        self.checkpoint(&[]);
        for id in std::mem::take(&mut self.confirm_delete) {
            self.delete(id);
        }
    }

    /// Asks for the model space, unit system and type of a new analysis, whose block goes
    /// to `position`; the last choice is proposed.
    fn ask_new_analysis(&mut self, position: egui::Pos2) {
        let properties = ModelProperties {
            kind: plx_model::ModelKind::General,
            global_results: None,
            ..self.settings.new_model.clone()
        };
        self.new_analysis = Some((position, ModelPropertiesDialog::create(properties)));
    }

    fn new_analysis_window(&mut self, ctx: &egui::Context) {
        let Some((position, dialog)) = &mut self.new_analysis else {
            return;
        };
        let position = *position;
        match dialog.show(ctx, None, None) {
            DialogResult::Open => {}
            DialogResult::Cancel => self.new_analysis = None,
            DialogResult::Ok(properties) => {
                self.checkpoint(&[]);
                self.new_analysis = None;
                self.settings.new_model = ModelProperties {
                    kind: plx_model::ModelKind::General,
                    global_results: None,
                    ..properties.clone()
                };
                let space = properties.space;
                let id = self.add_analysis("Analysis-1", position, properties);
                let name = self.name_of(id);
                self.output.push(format!(
                    "{name} created: {}, {}",
                    space.label(),
                    self.settings.new_model.units.label()
                ));
            }
        }
    }

    fn settings_window(&mut self, ctx: &egui::Context) {
        let Some(window) = &mut self.settings_window else {
            return;
        };
        let new = match window.show(ctx) {
            WindowResult::Open => return,
            WindowResult::Apply(settings) => settings,
            WindowResult::Ok(settings) => {
                self.settings_window = None;
                settings
            }
            WindowResult::Cancel => {
                self.settings_window = None;
                return;
            }
        };
        if new.gmsh != self.settings.gmsh && !plx_mesher::set_library_path(new.gmsh.library()) {
            self.output
                .push("The changed Gmsh library is loaded after a restart".into());
        }
        if new.graphics.theme != self.settings.graphics.theme {
            crate::style::set_theme(ctx, new.graphics.theme);
        }
        // The View tab owns these toggles; the window's copy may be older than a click there.
        let mut new = new;
        let live = &self.settings.graphics;
        new.graphics.shell_thickness = live.shell_thickness;
        new.graphics.beam_profiles = live.beam_profiles;
        new.graphics.show_linked = live.show_linked;
        new.graphics.checklist = live.checklist;
        self.settings = new;
        self.share_settings();
    }

    /// Gives every analysis window the program's settings.
    fn share_settings(&mut self) {
        for node in &mut self.nodes {
            if let Some(window) = node.window_mut() {
                window.set_settings(&self.settings);
            }
        }
    }

    /// Shows the open analyses' windows and acts on what they ask.
    fn analysis_windows(&mut self, ctx: &egui::Context) {
        let project = self.project_name();
        let embedded = self.embedded().filter(|_| !self.in_project);
        let mut requests = Vec::new();
        let mut closing = Vec::new();
        let mut settings = None;
        let mut changed = false;
        for node in &mut self.nodes {
            let (id, name) = (node.id, &node.name);
            let Block::Analysis(analysis) = &mut node.block else {
                continue;
            };
            let Some(window) = &mut analysis.window else {
                continue;
            };
            // The window shows the project's solve of its analysis.
            if let Some(solve) = self.solves.iter().rev().find(|s| s.id == id) {
                window.show_run(solve);
            }
            if !analysis.own_window {
                // Shown in the project window, or parked there while another one is shown:
                // it goes on with what it does in the background.
                if embedded != Some(id) {
                    window.tick(ctx);
                }
            } else {
                let mut builder = egui::ViewportBuilder::default()
                    .with_title(format!("{name} - {project} - Nodalis"))
                    .with_app_id("nodalis")
                    .with_inner_size([1400.0, 900.0])
                    .with_min_inner_size([640.0, 400.0])
                    .with_icon(crate::window_icon());
                if let Some(position) = analysis.window_position {
                    builder = builder.with_position(position);
                }
                ctx.show_viewport_immediate(analysis_viewport(id), builder, |ui, _| {
                    window.ui(ui);
                    if ui.input(|i| i.viewport().close_requested()) {
                        if window.busy().is_some() {
                            ui.ctx()
                                .send_viewport_cmd(egui::ViewportCommand::CancelClose);
                        }
                        closing.push(id);
                    }
                });
            }
            requests.extend(window.take_requests().into_iter().map(|r| (id, r)));
            if window.settings() != &self.settings {
                settings = Some(window.settings().clone());
            }
            // What links take from it follows its changes.
            let revision = window.revision();
            if revision != analysis.seen {
                // A window that is done loading has not changed the analysis.
                if analysis.seen.is_some() && revision.is_some() {
                    self.saver.window_edits += 1;
                }
                analysis.seen = revision;
                changed = true;
            }
            // The block shows the state live.
            if let Some(status) = window.status() {
                analysis.status = Some(status);
            }
            // A window that missed a pass of the links takes it once it is idle.
            if analysis.sync_missed && window.busy().is_none() && window.model().is_some() {
                analysis.sync_missed = false;
                changed = true;
            }
            if let Some(model) = window.model() {
                analysis.space = model.fe.properties.space;
            }
        }
        if changed {
            self.sync_due = true;
        }
        if let Some(settings) = settings {
            self.settings = settings;
            self.share_settings();
        }
        for id in closing {
            self.close_window(id);
        }
        for (id, request) in requests {
            match request {
                Request::Save(save_as) => self.save(save_as),
                Request::SaveWithoutResults => self.save_without_results(),
                Request::OpenProject(path) => self.open(ctx, path),
                Request::SolveChain(force) => {
                    if force {
                        self.forced.insert(id);
                    }
                    self.solve_chain(id);
                }
                Request::StopSolve => self.stop_solve(id),
                Request::OpenAnalysis(path) => {
                    let source = (self.links.iter())
                        .find(|l| l.target == id && links::covers(&path, links::slot(l)))
                        .map(|l| l.source);
                    if let Some(source) = source {
                        self.open_window(ctx, source, None);
                    }
                }
                Request::Campbell => self.open_campbell_of(ctx, id),
                Request::Close => {
                    if self.close_window(id) && self.active == Some(id) {
                        self.in_project = true;
                    }
                }
                // Unlinking what a state link took makes the analysis independent.
                Request::Unlink(path) => {
                    self.exclude(None, Some(&(id, path.clone())));
                    self.unlink(|l| {
                        l.target == id
                            && (links::covers(&path, links::slot(l)) || links::is_state(l))
                    });
                }
            }
        }
    }
}

/// Reads a project file for the project window.
fn read_opened(file: &Path) -> Opened {
    let project = plx_io::project::read_project(file).map_err(|e| e.to_string())?;
    Ok((project, Vec::new()))
}

/// What the thread reading a project sent, once it is done; a thread that ended without
/// sending anything failed.
fn received(receiver: &Receiver<Opened>) -> Option<Opened> {
    match receiver.try_recv() {
        Ok(opened) => Some(opened),
        Err(std::sync::mpsc::TryRecvError::Empty) => None,
        Err(std::sync::mpsc::TryRecvError::Disconnected) => {
            Some(Err("reading the file stopped unexpectedly".into()))
        }
    }
}

/// The message of a panic, as far as it has one.
fn panic_message(panic: &(dyn std::any::Any + Send)) -> String {
    let message = (panic.downcast_ref::<&str>().copied())
        .or_else(|| panic.downcast_ref::<String>().map(String::as_str))
        .unwrap_or("unknown error");
    format!("reading the file failed: {message}")
}

/// The workspace a double-click on the row at `path` opens the analysis in: Geometry and
/// Mesh in the Geometry workspace, the setup and the steps in the FE Model workspace,
/// Results in the Results workspace.
fn workspace_of(path: &[String]) -> Option<Show> {
    let show = |view| Some(Show { view, step: None });
    match path.first().map(String::as_str) {
        Some(links::GEOMETRY | links::MESH) => show(TreeView::Geometry),
        // A step of the setup and its inputs select the step.
        Some(links::SETUP) if path.len() > 2 && path[1] == links::STEPS => Some(Show {
            view: TreeView::FeModel,
            step: Some(path[2].clone()),
        }),
        Some(links::SETUP) => show(TreeView::FeModel),
        // The rows of the steps stand for the steps of the setup.
        Some(links::RESULTS) if path.len() > 1 => Some(Show {
            view: TreeView::FeModel,
            step: Some(path[1].clone()),
        }),
        Some(links::RESULTS) => show(TreeView::Results),
        _ => None,
    }
}

/// The analysis window's viewport.
fn analysis_viewport(id: u64) -> egui::ViewportId {
    egui::ViewportId::from_hash_of(("analysis", id))
}

impl eframe::App for ProjectApp {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, settings::STORAGE_KEY, &self.settings);
        eframe::set_value(storage, RECENT_KEY, &self.recent);
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        crate::style::sync(&ctx);
        self.poll_opening();
        self.poll_saving(&ctx);
        self.check_active();
        if crate::crash_log::take_device_lost() {
            self.output.push(
                "The graphics device was lost (driver reset or GPU switched off). \
                 Save the project and restart Nodalis if the view stays empty."
                    .into(),
            );
            self.toast = Some(crate::toast::Toast::new(
                "Graphics device lost: save the project and restart Nodalis",
                false,
            ));
        }
        let project = self.in_project;
        let title = format!("{} - Nodalis", self.project_name());
        if title != self.shown_title {
            self.shown_title = title.clone();
            ctx.send_viewport_cmd(egui::ViewportCommand::Title(title));
        }
        // In the analysis workspaces F5 solves the analysis shown, Ctrl+Z undoes its changes.
        if project && ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::F5)) {
            self.solve_shortcut();
        }
        if ctx.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, egui::Key::O)) {
            self.open_dialog(&ctx);
        }
        let shift_command = egui::Modifiers::COMMAND | egui::Modifiers::SHIFT;
        if ctx.input_mut(|i| i.consume_key(shift_command, egui::Key::S)) {
            self.save(true);
        }
        if ctx.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, egui::Key::S)) {
            self.save(false);
        }
        let typing = ctx.memory(|m| m.focused().is_some());
        if !typing && project {
            if ctx.input_mut(|i| i.consume_key(shift_command, egui::Key::Z))
                || ctx.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, egui::Key::Y))
            {
                self.redo(&ctx);
            } else if ctx.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, egui::Key::Z)) {
                self.undo(&ctx);
            }
        }
        if !typing
            && project
            && ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Home))
        {
            self.show_all = true;
        }
        let dropped: Vec<PathBuf> = ctx.input(|i| {
            (i.raw.dropped_files.iter())
                .map(|f| f.path().to_path_buf())
                .filter(|p| crate::model::is_project_file(p))
                .collect()
        });
        if let Some(path) = dropped.into_iter().next() {
            self.open(&ctx, path);
        }
        self.app_bar(ui);
        // The analysis shown here, in Geometry, FE Model or Results.
        if let Some(id) = self.embedded().filter(|_| !self.in_project) {
            self.embedded_ui(ui, id);
        } else {
            self.project_ui(ui);
        }
        self.rename_window(&ctx);
        self.confirm_delete_window(&ctx);
        self.material_set_window(&ctx);
        self.mesh_block_window(&ctx);
        self.cad_editor_window(&ctx);
        self.parameter_set_window(&ctx);
        self.measure_window(&ctx);
        self.study_window(&ctx);
        self.campbell_windows(&ctx);
        self.new_analysis_window(&ctx);
        self.settings_window(&ctx);
        self.updater.show(&ctx, &mut self.output);
        self.sync_links(&ctx);
        if self.update_after_sync && !self.sync_due {
            self.update_after_sync = false;
            self.update_project();
        }
        self.poll_solves(&ctx);
        self.run_update(&ctx);
        self.run_study();
        self.run_exports();
        self.analysis_windows(&ctx);
        self.join_dropped(&ctx);
        self.record_change(&ctx);
    }
}

impl ProjectApp {
    /// The Project workspace: its ribbon, the schematic and the solution monitor.
    fn project_ui(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        self.project_ribbon(ui);
        egui::Panel::bottom("project status").show(ui, |ui| {
            ui.horizontal(|ui| {
                // The tabs of the output pane: a click shows the tab, a click on the shown
                // tab hides the pane.
                let shown = (!crate::layout::output_folded(&ctx))
                    .then_some(usize::from(self.monitor.shown));
                let (unread, newest) =
                    (self.output_news).update(&ctx, &self.output, shown == Some(0));
                let titles = [
                    crate::output_news::OutputNews::title(unread).into(),
                    "Analyses".into(),
                ];
                let mut clicked = crate::style::status_tabs(ui, &titles, shown);
                if let Some(line) = newest
                    && crate::output_news::OutputNews::line(ui, line)
                {
                    clicked = Some(0);
                }
                if let Some(index) = clicked {
                    if shown == Some(index) {
                        crate::layout::fold_output(&ctx, true);
                    } else {
                        self.monitor.shown = index == 1;
                        crate::layout::fold_output(&ctx, false);
                    }
                }
                ui.separator();
                if let Some((path, _)) = &self.opening {
                    ui.spinner();
                    ui.label(format!("Opening {} …", path.display()));
                } else if let Some(status) = crate::save_status::get(&ctx) {
                    ui.spinner();
                    ui.label(status);
                } else {
                    let file = (self.path.as_ref())
                        .map_or_else(|| "not saved yet".into(), |p| p.display().to_string());
                    ui.label(format!("Project: {file}"));
                }
            });
        });
        let pane = egui::Frame::new()
            .fill(crate::style::window())
            .stroke(egui::Stroke::new(1.0, crate::style::border()));
        // The output pane is hidden until one of its tabs in the status bar is clicked or a
        // solve starts.
        if !crate::layout::output_folded(&ctx) {
            egui::Panel::bottom("project output")
                .resizable(true)
                .default_size(170.0)
                .size_range(70.0..=500.0)
                .frame(egui::Frame::new().fill(crate::style::control()))
                .show(ui, |ui| {
                    pane.inner_margin(4).show(ui, |ui| {
                        ui.set_min_size(ui.available_size());
                        if self.monitor.shown {
                            self.monitor_panel(ui);
                        } else {
                            egui::ScrollArea::vertical()
                                .auto_shrink([false, false])
                                .stick_to_bottom(true)
                                .show(ui, |ui| {
                                    for line in &self.output {
                                        ui.label(line);
                                    }
                                });
                        }
                    });
                });
        }
        let central = egui::CentralPanel::no_frame().show(ui, |ui| self.schematic(ui));
        for action in central.inner {
            self.act(&ctx, action);
        }
        if let Some(cmd) = crate::ribbon::floating_bar(
            &ctx,
            "schematic",
            central.response.rect,
            crate::ribbon::SCHEMATIC_BAR,
            &mut workspace::ProjectHost(self),
        ) {
            self.run_command(&ctx, cmd);
        }
        if let Some(toast) = &mut self.toast {
            match crate::toast::show(&ctx, toast, central.response.rect) {
                crate::toast::Event::None => {}
                crate::toast::Event::Gone => self.toast = None,
                crate::toast::Event::Undo => {
                    self.toast = None;
                    self.undo(&ctx);
                }
            }
        }
    }
}

/// A row of a block: a cell of the analysis, a category or item of its setup, or a step
/// whose results it has. Each row has a pin on either side to link it.
#[derive(Clone, Debug)]
struct Row {
    path: Vec<String>,
    text: String,
    depth: usize,
    icon: NodeIcon,
    state: Option<CellState>,
    /// Whether the row opens to show what is in it, and whether it is open.
    expander: Option<bool>,
    /// Whether the row takes a link (left pin) and gives one (right pin).
    input: bool,
    output: bool,
    /// The analysis a link fills the row from.
    linked_from: Option<String>,
    /// Whether the row is a collection, whose pin takes any number of wires, and whether a
    /// category link to it leaves out items.
    collection: bool,
    partial: bool,
    /// What the row edits in place, for a parameter of a source block.
    widget: Option<RowWidget>,
    tip: String,
}

/// What a row of a block edits in place, like the input rows of a Blender node.
#[derive(Clone, Copy, Debug)]
enum RowWidget {
    /// A number a mesh block is made from. Its row is where a numeric input will link to.
    Param(Param),
    /// The element type of a mesh block.
    Element,
    /// The refinement of a mesh block.
    Refinement,
    /// The length of a CAD operation, such as a fillet radius.
    CadLength,
    /// The factor and the increment of a deformed mesh block.
    DeformFactor,
    DeformIncrement,
    /// A coordinate of the point of a measure block, by axis.
    Coordinate(usize),
    /// A number of a parameters block, by its index.
    Value(usize),
    /// An exposed parameter of an analysis or a materials block, by its key.
    Exposed(u32),
    /// The file a mesh export block writes to.
    ExportPath,
    /// The operation of a Math block, one of the inputs of a Math or Switch block by index,
    /// and the number of choices of a Switch block.
    Operation,
    Input(usize),
    Choices,
}

/// A link of a whole category to a block, for its row: from where, how many items it takes
/// and how many it leaves out.
struct Bundle {
    path: Vec<String>,
    source: String,
    taken: usize,
    excluded: usize,
}

/// The state link into an analysis: what it continues from and the steps it takes over.
struct Continuation {
    source: String,
    step: String,
    inherited: Vec<String>,
}

/// Path of the row that lists the steps a continuation takes over from its source.
const INHERITED: &str = "Inherited steps";

/// How a block of the schematic looks: what the viewer needs of an analysis.
struct BlockView {
    id: u64,
    name: String,
    kind: Kind,
    space: ModelSpace,
    /// Whether the data it gives has a model space, `space`.
    has_space: bool,
    /// Its steps, to split it after one, and the analysis it continues, to merge into.
    steps: Vec<String>,
    continues: Option<String>,
    rows: Vec<Row>,
    loading: bool,
    open: bool,
    /// What a mesh block meshes, and the units its parameters are shown in.
    primitive: Option<MeshPrimitive>,
    /// The factor of a refine mesh block.
    refinement: f64,
    /// The factor and the increment of a deformed mesh block.
    deform: (f64, u32),
    /// The length of a CAD operation block.
    cad_length: f64,
    /// The point of a measure block that measures at a point.
    point: Option<[f64; 3]>,
    /// The numbers of a parameters block.
    parameters: Vec<Parameter>,
    /// The exposed parameters of an analysis or a materials block.
    exposed: Vec<ExposedView>,
    units: UnitSystem,
    /// What a mesh export block writes to.
    export_path: String,
    /// The text of a note.
    note: String,
    /// Whether it shows only its header and the rows with a wire, and is in a frame.
    compact: bool,
    framed: bool,
    /// A Math or Switch block, and whether it is a switch.
    number: Option<(NumberBlock, bool)>,
    /// The mark of the step chain at the rows of steps, by row path.
    chain: HashMap<Vec<String>, ChainMark>,
}

/// Where a row of a step lies on the chain of steps, for the trace at the left of the rows
/// of an analysis block: a solid line where the state flows, a dot at a step that passes its
/// state on, a side branch to a ring at a step that only reads it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct ChainMark {
    /// An active step that passes its state on.
    continues: bool,
    /// An active step that leaves the state as it found it.
    probing: bool,
    /// The probing step is computed about the state the steps before leave.
    branch: bool,
    /// An active step that passes its state on lies above and below the row.
    above: bool,
    below: bool,
}

/// The chain marks of the rows of steps of an analysis: under Setup > Steps (and the inputs
/// below an open step) and under Results, each list on its own.
fn chain_marks(fe: &plx_model::FeModel, rows: &[Row]) -> HashMap<Vec<String>, ChainMark> {
    let mut marks = HashMap::new();
    let continuing = |s: usize| fe.steps[s].active && fe.steps[s].kind.continues_state();
    for results in [false, true] {
        let listed: Vec<(&Vec<String>, usize)> = (rows.iter())
            .filter_map(|row| {
                let name = match row.path.as_slice() {
                    [setup, steps, name] if !results && setup == links::SETUP => {
                        (steps == links::STEPS).then_some(name)?
                    }
                    [group, name] if results && group == links::RESULTS => name,
                    _ => return None,
                };
                let index = fe.steps.iter().position(|s| &s.name == name)?;
                Some((&row.path, index))
            })
            .collect();
        for (k, &(path, s)) in listed.iter().enumerate() {
            let step = &fe.steps[s];
            let probing = step.active && !step.kind.continues_state();
            let branch = probing
                && (fe.step_handover(s).base_state()).is_some_and(|base| base != "Unloaded model");
            marks.insert(
                path.clone(),
                ChainMark {
                    continues: continuing(s),
                    probing,
                    branch,
                    above: listed[..k].iter().any(|&(_, p)| continuing(p)),
                    below: listed[k + 1..].iter().any(|&(_, p)| continuing(p)),
                },
            );
        }
    }
    // The inputs below an open step: the line passes them on to the next step.
    for row in rows.iter().filter(|r| r.path.len() == 4) {
        if let Some(mark) = marks.get(&row.path[..3]).copied() {
            let through = mark.below;
            marks.insert(
                row.path.clone(),
                ChainMark {
                    above: through,
                    below: through,
                    ..ChainMark::default()
                },
            );
        }
    }
    marks
}

/// Where the step `name` starts and what it passes on, for the tooltips of its rows.
fn chain_tip(fe: Option<&plx_model::FeModel>, name: &str) -> String {
    let Some(fe) = fe else {
        return String::new();
    };
    let Some(index) = fe.steps.iter().position(|s| s.name == name) else {
        return String::new();
    };
    let handover = fe.step_handover(index);
    format!(
        "\n{}. Passes on: {}.",
        handover.summary(),
        handover.passes_on
    )
}

/// Paints the chain trace of a row of a step in `rect`; `gap` is the space between rows,
/// which the lines bridge.
fn paint_chain(painter: &egui::Painter, rect: egui::Rect, depth: usize, mark: ChainMark, gap: f32) {
    let color = crate::style::adapt(egui::Color32::from_rgb(40, 150, 60));
    let stroke = egui::Stroke::new(2.0, color);
    let x = rect.left() + 2.0 + 14.0 * depth as f32 - 10.0;
    let middle = rect.center().y;
    let top = rect.top() - gap * 0.5 - 0.5;
    let bottom = rect.bottom() + gap * 0.5 + 0.5;
    if mark.continues {
        if mark.above {
            painter.line_segment([egui::pos2(x, top), egui::pos2(x, middle)], stroke);
        }
        if mark.below {
            painter.line_segment([egui::pos2(x, middle), egui::pos2(x, bottom)], stroke);
        }
        painter.circle_filled(egui::pos2(x, middle), 3.5, color);
        return;
    }
    if mark.above && mark.below {
        painter.line_segment([egui::pos2(x, top), egui::pos2(x, bottom)], stroke);
    } else if mark.above && mark.branch {
        painter.line_segment([egui::pos2(x, top), egui::pos2(x, middle)], stroke);
    }
    if mark.probing {
        let ring = egui::pos2(x + 6.5, middle);
        if mark.branch && mark.above {
            let thin = egui::Stroke::new(1.5, color);
            painter.line_segment([egui::pos2(x, middle), ring - egui::vec2(3.0, 0.0)], thin);
        }
        painter.circle_stroke(ring, 2.7, egui::Stroke::new(1.5, color));
    }
}

/// An exposed parameter as its row shows it: its value and kind, `None` when its field is
/// gone.
#[derive(Clone, Debug)]
struct ExposedView {
    key: u32,
    name: String,
    value: Option<f64>,
    kind: Option<ValueKind>,
}

impl ExposedView {
    fn of(fe: &plx_model::FeModel) -> Vec<Self> {
        (fe.exposed.iter())
            .map(|e| Self {
                key: e.key,
                name: e.name.clone(),
                value: fe.value_at(&e.path),
                kind: fe.value_kind(&e.path),
            })
            .collect()
    }
}

/// The rows of the exposed parameters of a block: a row that opens them, open at first, and
/// a row with a pin for each, where value links set it.
fn exposed_rows(
    fe: &plx_model::FeModel,
    expanded: &BTreeSet<Vec<String>>,
    incoming: &[(Vec<String>, String)],
) -> Vec<Row> {
    if fe.exposed.is_empty() {
        return Vec::new();
    }
    let path = vec![links::PARAMETERS.to_owned()];
    // Open unless it was closed.
    let open = !expanded.contains(&path);
    let mut rows = vec![Row {
        text: format!("{} ({})", EXPOSED, fe.exposed.len()),
        depth: 0,
        icon: NodeIcon::Setup,
        state: None,
        expander: Some(open),
        input: false,
        output: false,
        linked_from: None,
        collection: false,
        partial: false,
        widget: None,
        tip: "Exposed Parameters: numbers of the dialogs exposed with their P button. A value \
              link from a Parameters block or a parameter study sets each of them."
            .into(),
        path: path.clone(),
    }];
    if !open {
        return rows;
    }
    for exposed in &fe.exposed {
        let mut path = path.clone();
        path.push(exposed.key.to_string());
        let gone = fe.value_at(&exposed.path).is_none();
        let item = &exposed.path.item;
        let tip = if gone {
            format!(
                "{} '{}' no longer has this number: remove the parameter with a right-click, \
                 or give the item its name again.",
                item.noun(),
                item.name()
            )
        } else {
            format!(
                "{}: {} of {} '{}'. Drag a number of a Parameters block or a parameter study \
                 here to set it; right-click to rename or remove it.",
                exposed.name,
                exposed.path.field.label().to_lowercase(),
                item.noun().to_lowercase(),
                item.name()
            )
            .replace(":  of", ": the value of")
        };
        rows.push(Row {
            text: exposed.name.clone(),
            depth: 1,
            icon: NodeIcon::Setup,
            state: gone.then_some(CellState::Missing),
            expander: None,
            input: !gone,
            output: false,
            linked_from: (incoming.iter())
                .find(|(slot, _)| *slot == path)
                .map(|(_, source)| source.clone()),
            collection: false,
            partial: false,
            widget: Some(RowWidget::Exposed(exposed.key)),
            tip,
            path,
        });
    }
    rows
}

/// The name of the rows of the exposed parameters of a block.
const EXPOSED: &str = "Exposed Parameters";

/// The rows of an analysis' block: its cells, the setup opened as far as `expanded` says, and
/// the steps whose results can drive a submodel.
fn block_rows(
    status: &AnalysisStatus,
    fe: Option<&plx_model::FeModel>,
    expanded: &BTreeSet<Vec<String>>,
    incoming: &[(Vec<String>, String)],
    bundles: &[Bundle],
    continuation: Option<&Continuation>,
) -> Vec<Row> {
    let from = |path: &[String]| {
        (incoming.iter())
            .find(|(slot, _)| links::covers(slot, path))
            .map(|(_, source)| source.clone())
    };
    let cell = |name: &str, icon, state: CellState, linkable: bool| {
        let path = vec![name.to_owned()];
        Row {
            text: name.to_owned(),
            depth: 0,
            icon,
            state: Some(state),
            expander: None,
            input: linkable,
            output: linkable,
            linked_from: from(&path),
            collection: false,
            partial: false,
            widget: None,
            tip: format!("{name}: {}", state.text()),
            path,
        }
    };
    let state = vec![links::STATE.to_owned()];
    let continues = from(&state);
    let mut rows = vec![Row {
        text: match &continues {
            Some(source) => match continuation {
                Some(c) => format!("After {} of {}", c.step, c.source),
                None => format!("State of {source}"),
            },
            None => "State".to_owned(),
        },
        depth: 0,
        icon: NodeIcon::State,
        state: None,
        expander: None,
        input: true,
        output: false,
        tip: "Drop the results of a step of another analysis here: this analysis then \
                  continues from the state after that step. It takes the model and the steps \
                  up to it, and the solver solves only its own steps."
            .into(),
        linked_from: continues,
        collection: false,
        partial: false,
        widget: None,
        path: state,
    }];
    // A submodel takes the results of its global model here, like the State row.
    let global: Vec<&str> = (incoming.iter())
        .filter(|(slot, _)| links::ResultsUse::of_slot(slot) == Some(links::ResultsUse::Submodel))
        .map(|(_, source)| source.as_str())
        .collect();
    let submodel = fe.is_some_and(|fe| fe.properties.kind == plx_model::ModelKind::Submodel);
    if submodel || !global.is_empty() {
        let linked_from = (!global.is_empty()).then(|| global.join(", "));
        let text = match &linked_from {
            Some(source) => format!("Global results of {source}"),
            None => links::GLOBAL_RESULTS.to_owned(),
        };
        rows.push(Row {
            text,
            depth: 0,
            icon: NodeIcon::Results,
            // Missing until a global analysis gives them.
            state: global.is_empty().then_some(CellState::Missing),
            expander: None,
            input: true,
            output: false,
            tip: "Global results: drop the results of a step of the global analysis here. \
                  The submodel boundary conditions read its displacements on the cut \
                  boundary; open a step under Setup > Steps to give it the results of \
                  another global step."
                .into(),
            linked_from,
            collection: false,
            partial: false,
            widget: None,
            path: vec![links::GLOBAL_RESULTS.to_owned()],
        });
    }
    let mut geometry = cell(links::GEOMETRY, NodeIcon::Geometry, status.geometry, true);
    if !status.stale_parts.is_empty() {
        (geometry.tip).push_str(&format!(" ({})", status.stale_parts.join(", ")));
    }
    rows.push(geometry);
    rows.push(cell(links::MESH, NodeIcon::Mesh, status.mesh, true));
    let setup = vec![links::SETUP.to_owned()];
    let setup_open = expanded.contains(&setup);
    let mut row = cell(links::SETUP, NodeIcon::Setup, status.setup, true);
    row.expander = Some(setup_open);
    rows.push(row);
    if setup_open {
        for category in links::CATEGORIES {
            let path = vec![links::SETUP.to_owned(), category.to_owned()];
            let open = expanded.contains(&path);
            let entries = fe
                .map(|fe| links::entries(fe, category))
                .unwrap_or_default();
            let linked: Vec<&Bundle> = bundles.iter().filter(|b| b.path == path).collect();
            let mut tip = if category == "Initial Conditions" {
                "Initial Conditions: drag to another analysis to link them all, or drop the \
                 results of a thermal step here to start at its temperatures"
                    .to_owned()
            } else {
                format!("{category}: drag to another analysis to link them all")
            };
            tip.push_str(
                ".\nA collection: it takes any number of links, and its own items stay. \
                 Right-click an item's pin to leave it out of a linked category.",
            );
            for bundle in &linked {
                tip.push_str(&format!(
                    "\n{} of {} items from {}.",
                    bundle.taken,
                    bundle.taken + bundle.excluded,
                    bundle.source
                ));
            }
            let sources: Vec<&str> = linked.iter().map(|b| b.source.as_str()).collect();
            rows.push(Row {
                text: if entries.is_empty() {
                    category.to_owned()
                } else {
                    format!("{category} ({})", entries.len())
                },
                depth: 1,
                icon: category_icon(category),
                state: None,
                expander: (!entries.is_empty()).then_some(open),
                input: true,
                output: true,
                linked_from: from(&path)
                    .or_else(|| (!sources.is_empty()).then(|| sources.join(", "))),
                collection: links::is_collection(&path),
                partial: linked.iter().any(|b| b.excluded > 0),
                widget: None,
                tip,
                path: path.clone(),
            });
            if open {
                for name in entries {
                    let mut path = path.clone();
                    path.push(name.clone());
                    // A step has inputs for the results of other analyses below it.
                    let step = (category == links::STEPS)
                        .then(|| fe.and_then(|fe| fe.steps.iter().find(|s| s.name == name)))
                        .flatten();
                    let inputs = step.map(links::step_inputs).unwrap_or_default();
                    let step_open = expanded.contains(&path);
                    let tip = if step.is_some() {
                        format!(
                            "{name}: drag to another analysis to link its definition. Open \
                             it for its inputs, or drop the results of a step of another \
                             analysis here: they go to the input that fits.{}",
                            chain_tip(fe, &name)
                        )
                    } else {
                        format!("{name}: drag to another analysis to link it")
                    };
                    rows.push(Row {
                        tip,
                        text: name.clone(),
                        depth: 2,
                        icon: match step {
                            Some(step) => NodeIcon::StepType(StepGlyph::from(&step.kind)),
                            None => category_icon(category),
                        },
                        state: None,
                        expander: (!inputs.is_empty()).then_some(step_open),
                        input: true,
                        output: true,
                        linked_from: from(&path),
                        collection: false,
                        partial: false,
                        widget: None,
                        path: path.clone(),
                    });
                    for input in inputs.into_iter().filter(|_| step_open) {
                        let mut path = path.clone();
                        path.push(input.to_owned());
                        rows.push(Row {
                            tip: step_input_tip(&name, input),
                            text: input.to_owned(),
                            depth: 3,
                            icon: NodeIcon::Results,
                            state: None,
                            expander: None,
                            input: true,
                            output: false,
                            linked_from: from(&path),
                            collection: false,
                            partial: false,
                            widget: None,
                            path,
                        });
                    }
                }
            }
        }
    }
    if let Some(fe) = fe {
        rows.extend(exposed_rows(fe, expanded, incoming));
    }
    rows.push(cell("Solution", NodeIcon::Solution, status.solution, false));
    rows.push(cell("Results", NodeIcon::Results, status.results, false));
    let taken = continuation.map_or(&[][..], |c| c.inherited.as_slice());
    if let Some(c) = continuation.filter(|c| !c.inherited.is_empty()) {
        let path = vec![INHERITED.to_owned()];
        let open = expanded.contains(&path);
        let count = c.inherited.len();
        rows.push(Row {
            text: format!(
                "{count} step{} from {}",
                if count == 1 { "" } else { "s" },
                c.source
            ),
            depth: 1,
            icon: NodeIcon::Link,
            state: None,
            expander: Some(open),
            input: false,
            output: false,
            linked_from: Some(c.source.clone()),
            collection: false,
            partial: false,
            widget: None,
            tip: format!(
                "These steps are taken over from {} and solved there, not in this analysis. \
                 Their results are in {}.",
                c.source, c.source
            ),
            path: path.clone(),
        });
        for name in c.inherited.iter().filter(|_| open) {
            let mut path = path.clone();
            path.push(name.clone());
            rows.push(Row {
                text: name.clone(),
                depth: 2,
                icon: step_icon(fe, name),
                state: None,
                expander: None,
                input: false,
                output: false,
                linked_from: None,
                collection: false,
                partial: false,
                widget: None,
                tip: format!("{name} is solved in {}", c.source),
                path,
            });
        }
    }
    for step in fe
        .map(|fe| fe.steps.as_slice())
        .unwrap_or_default()
        .iter()
        .filter(|s| !taken.contains(&s.name))
    {
        let path = vec![links::RESULTS.to_owned(), step.name.clone()];
        let tip = if step.active {
            format!(
                "Results of {}, what it produces: drag them to an input of a step under \
                 Setup > Steps of another analysis (its Temperatures, Eigenmodes or Global \
                 results), to the State row of another analysis, which continues from them, \
                 or to empty space to continue in a new analysis.{}",
                step.name,
                chain_tip(fe, &step.name)
            )
        } else {
            format!("{} is deactivated and has no results", step.name)
        };
        rows.push(Row {
            text: step.name.clone(),
            depth: 1,
            icon: NodeIcon::StepType(StepGlyph::from(&step.kind)),
            state: None,
            expander: None,
            // Results only give; what a step takes is an input of it under Setup.
            input: false,
            output: step.active,
            linked_from: from(&path),
            collection: false,
            partial: false,
            widget: None,
            tip,
            path,
        });
    }
    rows
}

/// The rows of a materials or cross-sections block: all items, then each of them while that
/// row is open, to link to analyses.
fn collection_rows(
    collection: Collection,
    fe: &plx_model::FeModel,
    expanded: &BTreeSet<Vec<String>>,
) -> Vec<Row> {
    let path = collection_path(collection);
    let open = expanded.contains(&path);
    let names = collection.names(fe);
    let title = collection.title();
    let icon = category_icon(title);
    let noun = title.to_lowercase();
    let mut rows = vec![Row {
        text: format!("{title} ({})", names.len()),
        depth: 0,
        icon,
        state: None,
        expander: (!names.is_empty()).then_some(open),
        input: false,
        output: !names.is_empty(),
        linked_from: None,
        collection: false,
        partial: false,
        widget: None,
        tip: format!(
            "All {noun}: drag them to the {title} row of an analysis, which then takes each \
             of them, also those added here later"
        ),
        path: path.clone(),
    }];
    for name in names.into_iter().filter(|_| open) {
        let mut path = path.clone();
        path.push(name.to_owned());
        rows.push(Row {
            tip: format!("{name}: drag it to the {title} row of an analysis to link it"),
            text: name.to_owned(),
            depth: 1,
            icon,
            state: None,
            expander: None,
            input: false,
            output: true,
            linked_from: None,
            collection: false,
            partial: false,
            widget: None,
            path,
        });
    }
    rows
}

/// Makes the meshes of the mesh blocks and refine mesh blocks among `nodes` that are out of
/// date anew, of those in `only` when it is given; returns whether it made any, and a line on
/// each.
fn generate_stale_meshes(nodes: &mut [Node], only: Option<&[u64]>) -> (bool, Vec<String>) {
    let mut lines = Vec::new();
    let mut made = false;
    for node in (nodes.iter_mut()).filter(|n| only.is_none_or(|ids| ids.contains(&n.id))) {
        let stale = match node.kind() {
            Kind::Mesh => node.mesh_state() != CellState::UpToDate,
            // One without a mesh to refine has nothing to make.
            Kind::RefineMesh => node.mesh_state() == CellState::OutOfDate,
            Kind::CadOperation => node
                .as_cad()
                .is_some_and(|c| c.state() == CellState::OutOfDate),
            _ => false,
        };
        if !stale {
            continue;
        }
        if let Some(cad) = node.as_cad_mut() {
            lines.push(match cad.generate() {
                Ok(line) => {
                    made = true;
                    format!("{}: {line}", node.name)
                }
                Err(why) => format!("{}: the operation failed: {why}", node.name),
            });
            continue;
        }
        lines.push(match node.generate() {
            Ok(count) => {
                made = true;
                format!("{}: mesh made, {count} elements", node.name)
            }
            Err(why) => format!("{}: no mesh made: {why}", node.name),
        });
    }
    (made, lines)
}

/// The mesh blocks and refine mesh blocks before the analysis `id` whose mesh is out of
/// date: what takes its mesh from them, directly or through other analyses, holds a mesh
/// that is not the block's.
fn stale_meshes(nodes: &[Node], links: &[Link], id: u64) -> Vec<String> {
    (links::upstream(links, id).into_iter())
        .filter_map(|source| nodes.iter().find(|n| n.id == source))
        .filter(|n| match n.kind() {
            Kind::Mesh | Kind::RefineMesh => n.mesh_state() != CellState::UpToDate,
            Kind::DeformedMesh => {
                (n.as_deformed()).is_some_and(|d| d.state() != CellState::UpToDate)
            }
            Kind::CadOperation => n.as_cad().is_some_and(|c| c.state() != CellState::UpToDate),
            _ => false,
        })
        .map(|n| n.name.clone())
        .collect()
}

/// The row of a mesh block: its mesh, to link to analyses, with its state, and its
/// parameters, which value links fill as `incoming` says.
fn mesh_rows(node: &Node, incoming: &[(Vec<String>, String)]) -> Vec<Row> {
    let state = node.mesh_state();
    let Some(block) = node.as_mesh() else {
        return Vec::new();
    };
    let mesh = Some(&block.model.mesh);
    let count = mesh.map_or(0, plx_mesh::FeMesh::element_count);
    let element = (mesh.and_then(|m| m.elements().first())).map_or("", |e| e.type_name.as_str());
    let sides = mesh.map_or_else(String::new, |m| {
        m.node_sets.keys().cloned().collect::<Vec<_>>().join(", ")
    });
    let mut tip = format!("Mesh: {}", state.text());
    if state == CellState::OutOfDate {
        tip = "Mesh: out of date: Update Project or Generate Mesh makes it again".into();
    }
    if let Some(problem) = block.primitive.problem(block.space) {
        tip.push_str(&format!("\nIt cannot be made: {problem}"));
    }
    if count > 0 {
        tip.push_str(&format!(
            "\n{count} elements {element}, sides {sides} as node sets and surfaces.\nDrag it \
             to the Mesh row of an analysis to link it."
        ));
    }
    let mut rows = vec![Row {
        path: vec![links::MESH.to_owned()],
        text: links::MESH.to_owned(),
        depth: 0,
        icon: NodeIcon::Mesh,
        state: Some(state),
        expander: None,
        input: false,
        output: count > 0,
        linked_from: None,
        collection: false,
        partial: false,
        widget: None,
        tip,
    }];
    // Its parameters below, like the inputs of a Blender node under its outputs.
    let primitive = &block.primitive;
    let row = |key: &str, text: &str, widget: RowWidget, tip: String| {
        let path = vec![PARAMETERS.to_owned(), key.to_owned()];
        Row {
            text: text.to_owned(),
            depth: 0,
            icon: NodeIcon::Mesh,
            state: None,
            expander: None,
            // A number of another block can give it its value.
            input: !matches!(widget, RowWidget::Element),
            output: false,
            linked_from: (incoming.iter())
                .find(|(slot, _)| *slot == path)
                .map(|(_, source)| source.clone()),
            collection: false,
            partial: false,
            widget: Some(widget),
            tip,
            path,
        }
    };
    for param in primitive.params() {
        let tip = format!(
            "{}; the mesh is out of date once it changes. Drop a parameter of a Parameters \
             block on its pin to give it that value.",
            param.tip
        );
        rows.push(row(param.key, param.label, RowWidget::Param(*param), tip));
    }
    let tip = match primitive {
        MeshPrimitive::Box { .. } => "Element type of the hexahedra".to_owned(),
        MeshPrimitive::Rectangle { .. } => format!(
            "Element type: linear (4-node) or quadratic (8-node) quadrilaterals, with reduced \
             integration (R) or not; the family follows from the model space, {}",
            block.space.label()
        ),
    };
    rows.push(row("element", "Element", RowWidget::Element, tip));
    rows
}

/// Brings what the refine mesh block `id` refines up to date with the block linked to
/// its Mesh row. The parameters of a mesh block it meshes itself, with its factor on the
/// divisions; geometry it gives on with the element sizes divided by its factor, for the
/// analysis the mesh goes to to mesh it.
fn take_refine_input(nodes: &mut [Node], links: &[Link], id: u64) {
    let Some((taken, own_factor)) = (nodes.iter().find(|n| n.id == id))
        .and_then(Node::as_refine)
        .map(|r| (r.taken, r.factor))
    else {
        return;
    };
    let link = links
        .iter()
        .find(|l| l.target == id && l.item.first().is_some_and(|c| c == links::MESH));
    let mut copy = None;
    let input = match link.and_then(|l| nodes.iter().find(|n| n.id == l.source)) {
        None => None,
        Some(source) => match &source.block {
            Block::Mesh(m) => Some(RefineInput::Primitive {
                primitive: m.primitive.clone(),
                space: m.space,
                factor: 1.0,
            }),
            Block::RefineMesh(r) => match &r.input {
                Some(RefineInput::Primitive {
                    primitive,
                    space,
                    factor,
                }) => Some(RefineInput::Primitive {
                    primitive: primitive.clone(),
                    space: *space,
                    factor: factor * r.factor,
                }),
                None => Some(RefineInput::Fixed(source.name.clone())),
                Some(RefineInput::Fixed(name)) => Some(RefineInput::Fixed(name.clone())),
                Some(RefineInput::Geometry { .. }) => None,
            },
            _ => None,
        }
        .or_else(|| {
            // An analysis loading in its window is taken once it has loaded.
            let data = source.link_data()?;
            Some(match data.geometry {
                Some(geometry) if data.mesh.element_count() > 0 => {
                    let mut print = plx_mesh::Fnv::new();
                    let path = [links::MESH.to_owned()];
                    print.add(&links::fingerprint(data, &path).to_le_bytes());
                    print.add(&own_factor.to_bits().to_le_bytes());
                    let print = print.finish();
                    copy = (taken != Some(print)).then(|| (print, links::MeshCopy::of(data)));
                    RefineInput::Geometry {
                        space: source.space().unwrap_or_default(),
                        refinement: geometry.refinement,
                    }
                }
                _ => RefineInput::Fixed(source.name.clone()),
            })
        }),
    };
    if link.is_some() && input.is_none() {
        return;
    }
    let Some(refine) = (nodes.iter_mut().find(|n| n.id == id)).and_then(Node::as_refine_mut) else {
        return;
    };
    match &input {
        Some(RefineInput::Geometry { space, refinement }) => {
            if let Some((print, mut copy)) = copy {
                if let Some(geometry) = &mut copy.geometry {
                    geometry.refinement = refinement * refine.factor;
                }
                refine.model.data_mut().take_mesh(copy);
                refine.model.fe.properties.space = *space;
                refine.taken = Some(print);
                refine.generated = None;
            }
        }
        Some(RefineInput::Primitive { .. }) => {
            let fresh = refine.generated.is_none();
            refine.input = input;
            refine.taken = None;
            // The first mesh is made at once, so that it can be linked on.
            if fresh {
                let _ = refine.generate();
            }
            return;
        }
        None | Some(RefineInput::Fixed(_)) => {
            refine.model = LinkedModel::default();
            refine.generated = None;
            refine.taken = None;
        }
    }
    refine.input = input;
}

/// The rows of a refine mesh block: the mesh it takes and gives, with its state, and the
/// factor, which a value link may give.
fn refine_rows(node: &Node, incoming: &[(Vec<String>, String)]) -> Vec<Row> {
    let state = node.mesh_state();
    let Some(block) = node.as_refine() else {
        return Vec::new();
    };
    let linked = |path: &[String]| {
        (incoming.iter())
            .find(|(slot, _)| slot.as_slice() == path)
            .map(|(_, source)| source.clone())
    };
    let mesh = &block.model.mesh;
    let count = mesh.element_count();
    let mut tip = "Mesh: drag the mesh of a mesh block or of an analysis that meshes geometry \
                   here; drag the pin on the right to the Mesh row of an analysis for the \
                   refined mesh."
        .to_owned();
    match &block.input {
        None => {}
        Some(RefineInput::Fixed(source)) => tip.push_str(&format!(
            "\nThe mesh of {source} has no parameters or geometry to mesh it again with."
        )),
        Some(RefineInput::Geometry { .. }) => tip.push_str(
            "\nIt refines geometry: every element size of the mesh setup is divided by the \
             factor, and the analysis the mesh goes to meshes it with them when it is solved.",
        ),
        Some(RefineInput::Primitive { .. }) => {
            tip.push_str(&format!("\nMesh: {}", state.text()));
            if state == CellState::OutOfDate {
                tip.push_str(": Update Project or Generate Mesh makes it again");
            }
            if let Some(problem) = (block.meshed()).and_then(|(p, space)| p.problem(space)) {
                tip.push_str(&format!("\nIt cannot be made: {problem}"));
            }
            if count > 0 {
                tip.push_str(&format!(
                    "\n{count} elements, the divisions times the factor, rounded."
                ));
            }
        }
    }
    let path = vec![links::MESH.to_owned()];
    let factor = vec![PARAMETERS.to_owned(), REFINEMENT.to_owned()];
    vec![
        Row {
            text: links::MESH.to_owned(),
            depth: 0,
            icon: NodeIcon::Mesh,
            state: (block.input.is_some()).then_some(state),
            expander: None,
            input: true,
            output: count > 0,
            linked_from: linked(&path),
            collection: false,
            partial: false,
            widget: None,
            tip,
            path,
        },
        Row {
            text: "Factor".to_owned(),
            depth: 0,
            icon: NodeIcon::Mesh,
            state: None,
            expander: None,
            input: true,
            output: false,
            linked_from: linked(&factor),
            collection: false,
            partial: false,
            widget: Some(RowWidget::Refinement),
            tip: "Factor: the divisions of a mesh block are multiplied by it, rounded, and the \
                  element sizes of geometry divided by it; 1 leaves the mesh as it is. Drop \
                  an input of a parameter study here to vary the mesh in its design points."
                .to_owned(),
            path: factor,
        },
    ]
}

/// Why `item` of the block `source` cannot go to `slot` of the block `target`; `None`
/// when the link may be made. The type of the two pins comes first, then the model space
/// of what they carry, then what the blocks and the links between them allow.
fn refusal(
    nodes: &[Node],
    existing: &[Link],
    source: u64,
    item: &[String],
    target: u64,
    slot: &[String],
) -> Option<String> {
    let node = |id: u64| nodes.iter().find(|n| n.id == id);
    let name = |id: u64| node(id).map_or_else(String::new, |n| n.name.clone());
    let (from, to) = (name(source), name(target));
    let (Some(source_node), Some(target_node)) = (node(source), node(target)) else {
        return Some(format!("{from} or {to} is gone."));
    };
    if source == target {
        return Some(format!("{from} cannot take data from itself."));
    }
    if !links::compatible(item, slot) {
        let kind = |path: &[String]| links::DataType::of(path).map_or("nothing", |t| t.label());
        return Some(format!(
            "{} of {from} is {} and {} of {to} takes {}.",
            links::label(item),
            with_article(kind(item)),
            links::label(slot),
            with_article(kind(slot)),
        ));
    }
    // A refine mesh block takes the model space of the mesh it refines.
    let target_space = match target_node.kind() {
        Kind::RefineMesh | Kind::DeformedMesh => None,
        _ => target_node.space(),
    };
    // Data of a model goes only between blocks of the same model space; materials and
    // the other blocks without one take it from any.
    if links::DataType::of(item).is_some_and(links::DataType::has_space)
        && let (Some(a), Some(b)) = (source_node.space(), target_space)
        && a != b
    {
        return Some(format!(
            "{from} is {} and {to} is {}: linked blocks need the same model space.",
            a.label(),
            b.label()
        ));
    }
    let feedback = item.first().is_some_and(|c| c == links::LOOP);
    if !feedback && links::makes_cycle(existing, source, target) {
        return Some(format!(
            "{to} comes before {from} already, and a link cannot go back."
        ));
    }
    let first = slot.first().map(String::as_str);
    if first == Some(links::MEASURE) {
        return None;
    }
    if first == Some(links::STEP_RESULT) {
        return (source_node.kind() != Kind::Analysis)
            .then(|| format!("{to} deforms the mesh of an analysis, and {from} is none."));
    }
    if target_node.kind() == Kind::RefineMesh && first == Some(links::MESH) {
        let remeshed = matches!(source_node.kind(), Kind::Mesh | Kind::RefineMesh)
            || source_node
                .link_data()
                .is_some_and(|d| d.geometry.is_some());
        if !remeshed {
            return Some(format!(
                "{to} makes a mesh again from the parameters of a mesh block or from \
                 geometry, and the mesh of {from} has neither."
            ));
        }
    }
    let refines_geometry = (source_node.as_refine())
        .is_some_and(|r| matches!(r.input, Some(RefineInput::Geometry { .. })));
    if refines_geometry && target_node.kind() == Kind::MeshExport {
        return Some(format!(
            "{from} refines geometry, which the analysis its mesh goes to meshes; link the \
             Mesh of that analysis to {to}."
        ));
    }
    if first == Some(links::ANALYSIS) {
        return (source_node.kind() != Kind::Analysis || target_node.kind() != Kind::Campbell)
            .then(|| format!("{to} sweeps an analysis, and {from} is none."));
    }
    if first != Some(links::STATE)
        && let Some(chain) = existing
            .iter()
            .find(|l| l.target == target && links::is_state(l))
    {
        return Some(format!(
            "{to} continues {} and takes its model from there; unlink it first.",
            name(chain.source)
        ));
    }
    if first == Some(links::OUTPUTS) {
        return (target_node.kind() != Kind::Study
            || item.first().is_none_or(|c| c != links::VALUE))
        .then(|| {
            format!(
                "{to} reads numbers, and {} of {from} is none.",
                links::label(item)
            )
        });
    }
    if item.first().is_some_and(|c| c == links::VALUE)
        && let Some(why) = numbers::number_refusal(nodes, existing, source, item, target, slot)
    {
        return why;
    }
    if item.first().is_some_and(|c| c == links::VALUE) || feedback {
        let (given, taken) = (
            value_kind(nodes, source, item),
            value_kind(nodes, target, slot),
        );
        // The Inputs row of a study, or a column of it without a kind yet, takes the kind
        // of what it gives to; so does a Math or Switch block without a kind, at either end.
        let open =
            |id: u64, path: &[String]| node(id).is_some_and(|n| numbers::open_number(n, path));
        let any = (feedback && given.is_none() && (taken.is_some() || open(target, slot)))
            || (open(source, item) && (taken.is_some() || open(target, slot)))
            || (open(target, slot) && given.is_some());
        if !any && (given.is_none() || given != taken) {
            let kind = |k: Option<ValueKind>| k.map_or("no number", ValueKind::label);
            return Some(format!(
                "{} of {from} is a {} and {} of {to} a {}.",
                item.get(1).map_or("", String::as_str),
                kind(given).to_lowercase(),
                parameter_label(nodes, target, slot),
                kind(taken).to_lowercase()
            ));
        }
    }
    if item.first().is_some_and(|c| c == links::RESULTS) && first != Some(links::STATE) {
        // What the results are used for follows from the steps at both ends.
        let (Some(source_data), Some(target_data)) =
            (source_node.link_data(), target_node.link_data())
        else {
            return Some(format!("{from} or {to} is still loading."));
        };
        let step =
            (item.get(1)).and_then(|n| source_data.fe.steps.iter().find(|s| &s.name == n))?;
        if let Err(why) = links::results_slot(step, target_data.fe, slot) {
            return Some(format!(
                "{} of {from} cannot go to {} of {to}: {why}.",
                links::label(item),
                links::label(slot)
            ));
        }
    }
    None
}

/// The label of the parameter row `slot`, `["Parameters", key]`, of a block.
fn parameter_label(nodes: &[Node], id: u64, slot: &[String]) -> String {
    let key = slot.get(1).map_or("", String::as_str);
    if key == REFINEMENT {
        return "Factor".into();
    }
    let node = nodes.iter().find(|n| n.id == id);
    if node.and_then(Node::as_deformed).is_some() {
        match key {
            DEFORM_FACTOR => return "Factor".into(),
            DEFORM_INCREMENT => return "Mode".into(),
            _ => {}
        }
    }
    if node.and_then(Node::as_measure).is_some() && POINT_AXES.contains(&key) {
        return key.to_uppercase();
    }
    if let Some((number, switch)) = node.and_then(Node::as_number) {
        let label = (number.inputs(switch).into_iter())
            .find(|(k, ..)| k == key)
            .map_or_else(|| key.to_owned(), |(_, label, ..)| label);
        return label;
    }
    if let Some(exposed) = (node.and_then(Node::exposing_fe))
        .zip(key.parse().ok())
        .and_then(|(fe, key)| fe.exposed(key))
    {
        return exposed.name.clone();
    }
    if slot.first().is_some_and(|c| c == links::VALUE) {
        return key.to_owned();
    }
    node.and_then(Node::as_mesh)
        .and_then(|m| m.primitive.params().iter().find(|param| param.key == key))
        .map_or_else(|| key.to_owned(), |param| param.label.to_owned())
}

/// What kind of number a block gives at `item` or takes at `slot`.
fn value_kind(nodes: &[Node], id: u64, path: &[String]) -> Option<ValueKind> {
    let node = nodes.iter().find(|n| n.id == id)?;
    if node.as_number().is_some() {
        return node.number_kind(path).flatten();
    }
    if let Some(measure) = node.as_measure() {
        return match path {
            [cell, key] if cell == links::PARAMETERS => (measure.kind.point().is_some()
                && POINT_AXES.contains(&key.as_str()))
            .then_some(ValueKind::LENGTH),
            _ => (path == [links::VALUE, RESULT])
                .then(|| crate::measure::value_kind(&measure.kind))
                .flatten(),
        };
    }
    match path {
        [cell, name] if cell == links::VALUE => (node.as_parameters()?.parameters.iter())
            .find(|p| &p.name == name)
            .map(|p| p.kind),
        // The Inputs row of a study adds a column of any kind; a column without one yet
        // takes any too.
        [cell] if cell == links::LOOP => None,
        [cell, name] if cell == links::LOOP => {
            let table = &node.as_study()?.table;
            table.inputs[table.input(name)?].kind
        }
        [cell, key] if cell == links::PARAMETERS && key == REFINEMENT => Some(ValueKind::Factor),
        [cell, key] if cell == links::PARAMETERS && node.as_deformed().is_some() => {
            match key.as_str() {
                DEFORM_FACTOR => Some(ValueKind::Factor),
                DEFORM_INCREMENT => Some(ValueKind::Count),
                _ => None,
            }
        }
        [cell, key]
            if cell == links::PARAMETERS && key == CAD_LENGTH && node.as_cad().is_some() =>
        {
            Some(ValueKind::Quantity(Quantity::Length))
        }
        [cell, key] if cell == links::PARAMETERS => match node.exposing_fe() {
            Some(fe) => fe.value_kind(&fe.exposed(key.parse().ok()?)?.path),
            None => node.as_mesh()?.primitive.value_kind(key),
        },

        _ => None,
    }
}

/// `word` with a capital first letter.
fn capitalized(word: &str) -> String {
    let mut chars = word.chars();
    chars
        .next()
        .map_or_else(String::new, |c| c.to_uppercase().chain(chars).collect())
}

/// `word` with its indefinite article: "a mesh", "an analysis"; "nothing" stays as it is.
fn with_article(word: &str) -> String {
    match word.chars().next() {
        _ if word == "nothing" => word.to_owned(),
        Some('a' | 'e' | 'i' | 'o' | 'u') => format!("an {word}"),
        _ => format!("a {word}"),
    }
}

/// The file a relative export path names: next to the project file, or in the working folder
/// while the project has none.
/// How often the files of file blocks are looked at for changes.
const FILE_CHECK_INTERVAL: Duration = Duration::from_millis(1000);

/// What tells a change of a file: its path, size and time of modification; `None` when it is
/// not there.
fn file_print(path: &Path) -> Option<u64> {
    let metadata = std::fs::metadata(path).ok().filter(|m| m.is_file())?;
    let mut hash = plx_mesh::Fnv::new();
    hash.add_str(&path.to_string_lossy());
    hash.add(&metadata.len().to_le_bytes());
    if let Ok(time) = metadata.modified()
        && let Ok(since) = time.duration_since(std::time::UNIX_EPOCH)
    {
        hash.add(&since.as_nanos().to_le_bytes());
    }
    Some(hash.finish())
}

/// Reads the file of a file block: CAD geometry with lengths in `units`, the mesh of an input
/// file whose numbers are in `units`, or the mesh and steps of a results file.
fn read_source(path: &Path, units: UnitSystem) -> Result<FileRead, String> {
    let text = path.to_string_lossy();
    let kind = FileKind::of(&text).ok_or_else(|| {
        format!(
            "{} is no results file (.frd), input file (.inp) or geometry",
            path.display()
        )
    })?;
    let mut model = LinkedModel::default();
    model.fe.properties.units = units;
    match kind {
        FileKind::Geometry => {
            let import = plx_mesher::import_cad_files(&[path.to_path_buf()], units)
                .map_err(|e| e.to_string())?;
            let parts = plx_mesher::part_names(&import.geometry).map_err(|e| e.to_string())?;
            let summary = match parts.len() {
                1 => format!("geometry read from {}", path.display()),
                n => format!("geometry of {n} parts read from {}", path.display()),
            };
            model.geometry = Some(import.geometry);
            Ok(FileRead {
                model,
                space: None,
                steps: Vec::new(),
                parts,
                summary,
            })
        }
        FileKind::Mesh => {
            let loaded = crate::model::load(path, units)?;
            let mesh = loaded.model.mesh;
            let space = ModelSpace::of_mesh(&mesh);
            let summary = format!(
                "{} nodes and {} elements read from {}",
                mesh.node_count(),
                mesh.element_count(),
                path.display()
            );
            model.fe.properties.space = space;
            model.mesh = mesh;
            Ok(FileRead {
                model,
                space: Some(space),
                steps: Vec::new(),
                parts: Vec::new(),
                summary,
            })
        }
        FileKind::Results => {
            let import = plx_io::frd::read_frd(path).map_err(|e| e.to_string())?;
            let mut steps = Vec::new();
            for increment in &import.increments {
                if steps.iter().any(|(_, n)| *n == increment.step) {
                    continue;
                }
                let name = format!("Step-{}", increment.step);
                // The rows take the kind of their step, for what their results can be used.
                let has = |field: &str| increment.fields.iter().any(|f| f.name == field);
                let mut step = match increment.kind {
                    plx_results::AnalysisKind::Frequency => plx_model::Step::new_frequency(&name),
                    _ if has("NDTEMP") && !has("DISP") => plx_model::Step::new_heat_transfer(&name),
                    _ => plx_model::Step::new_static(&name),
                };
                // The eigenmodes are kept in a file of their own that it has not.
                if let plx_model::StepKind::Frequency(frequency) = &mut step.kind {
                    frequency.storage = false;
                }
                model.fe.steps.push(step);
                steps.push((name, increment.step));
            }
            if steps.is_empty() {
                return Err(format!("{} holds no results", path.display()));
            }
            let mesh = import.mesh;
            let space = ModelSpace::of_mesh(&mesh);
            let summary = format!(
                "{} step(s) of results on {} nodes read from {}",
                steps.len(),
                mesh.node_count(),
                path.display()
            );
            model.fe.properties.space = space;
            model.mesh = mesh;
            Ok(FileRead {
                model,
                space: Some(space),
                steps,
                parts: Vec::new(),
                summary,
            })
        }
    }
}

/// The rows of a file block: the file, then what it gives: its geometry, its mesh or the
/// results of each step.
fn file_rows(node: &Node, base: Option<&Path>) -> Vec<Row> {
    let state = node.file_state(base);
    let Some(file) = node.as_file() else {
        return Vec::new();
    };
    let row = |path: Vec<String>, text: String, icon, state, output, tip: String| Row {
        path,
        text,
        depth: 0,
        icon,
        state,
        expander: None,
        input: false,
        output,
        linked_from: None,
        collection: false,
        partial: false,
        widget: None,
        tip,
    };
    let mut status = match state {
        CellState::Missing if file.path.trim().is_empty() => "no file is chosen".to_owned(),
        CellState::Missing => "the file is missing".to_owned(),
        CellState::Running => "being read".to_owned(),
        CellState::Failed => format!("not read: {}", file.error.as_deref().unwrap_or("")),
        CellState::OutOfDate => "the file changed and is read again".to_owned(),
        _ => "read".to_owned(),
    };
    let mut path_row = row(
        vec![PARAMETERS.to_owned(), "File".to_owned()],
        "File".to_owned(),
        NodeIcon::Export,
        None,
        false,
        "The file to read: CalculiX results (.frd), a CalculiX input file (.inp) or geometry \
         (STEP, IGES, BREP, STL). A relative path starts at the folder of the project file. \
         It is read again whenever it changes. Double-click the block to choose it."
            .to_owned(),
    );
    path_row.widget = Some(RowWidget::ExportPath);
    let ready = state == CellState::UpToDate;
    let mut rows = Vec::new();
    match FileKind::of(&file.path) {
        Some(FileKind::Geometry) => {
            status.push_str(". Drag it to the Geometry row of an analysis to link it.");
            let path = vec![links::GEOMETRY.to_owned()];
            let mut geometry = row(
                path.clone(),
                links::GEOMETRY.to_owned(),
                NodeIcon::Geometry,
                Some(state),
                ready,
                format!("Geometry: {status}"),
            );
            // An assembly opens to its parts, which link one by one.
            let open = node.expanded.contains(&path);
            if file.parts.len() > 1 {
                geometry.expander = Some(open);
                geometry.tip.push_str(&format!(
                    " It has {} parts: open it to link only some of them.",
                    file.parts.len()
                ));
            }
            rows.push(geometry);
            for part in file.parts.iter().filter(|_| open && file.parts.len() > 1) {
                let mut part_row = row(
                    vec![links::GEOMETRY.to_owned(), part.clone()],
                    part.clone(),
                    NodeIcon::Geometry,
                    None,
                    ready,
                    format!(
                        "Part {part} of the file: drag it to the Geometry row of an analysis \
                         to link only this part. The parts linked to the same analysis go \
                         there together."
                    ),
                );
                part_row.depth = 1;
                rows.push(part_row);
            }
        }
        Some(FileKind::Mesh) => {
            let mesh = &file.model.mesh;
            if ready {
                status.push_str(&format!(
                    ": {} nodes, {} elements, {} node sets and {} surfaces. Drag it to the \
                     Mesh row of an analysis to link it.",
                    mesh.node_count(),
                    mesh.element_count(),
                    mesh.node_sets.len(),
                    mesh.surfaces.len()
                ));
            }
            rows.push(row(
                vec![links::MESH.to_owned()],
                links::MESH.to_owned(),
                NodeIcon::Mesh,
                Some(state),
                ready,
                format!("Mesh: {status}"),
            ));
        }
        Some(FileKind::Results) => {
            rows.push(row(
                vec![links::RESULTS.to_owned()],
                links::RESULTS.to_owned(),
                NodeIcon::Results,
                Some(state),
                false,
                format!("Results: {status}"),
            ));
            for (name, number) in &file.steps {
                let step = file.model.fe.steps.iter().find(|s| &s.name == name);
                let mut step_row = row(
                    vec![links::RESULTS.to_owned(), name.clone()],
                    name.clone(),
                    step.map_or(NodeIcon::Results, |s| {
                        NodeIcon::StepType(StepGlyph::from(&s.kind))
                    }),
                    None,
                    ready,
                    format!(
                        "Results of step {number} of the file: drag them to the Global results \
                         row of a submodel, or to the Temperatures of a step of an analysis \
                         with the same mesh"
                    ),
                );
                step_row.depth = 1;
                rows.push(step_row);
            }
        }
        None => {}
    }
    rows.push(path_row);
    rows
}

fn export_file(path: &str, base: Option<&Path>) -> PathBuf {
    let path = Path::new(path.trim());
    match base {
        Some(base) if path.is_relative() => base.join(path),
        _ => path.to_path_buf(),
    }
}

/// The rows of a mesh export block: the mesh it takes, with the state of its file, and the
/// file it writes.
fn export_rows(node: &Node, base: Option<&Path>, incoming: &[(Vec<String>, String)]) -> Vec<Row> {
    let state = node.export_state(base);
    let Some(export) = node.as_export() else {
        return Vec::new();
    };
    let mesh = [links::MESH.to_owned()];
    let source = (incoming.iter())
        .find(|(slot, _)| links::covers(slot, &mesh))
        .map(|(_, source)| source.clone());
    let mut tip = match state {
        CellState::Missing if export.path.trim().is_empty() => "Mesh: no file is chosen".to_owned(),
        CellState::Missing => "Mesh: nothing is linked yet, drag the mesh of a block here".into(),
        CellState::UpToDate => "Mesh: the file is up to date".into(),
        CellState::OutOfDate => {
            "Mesh: the file is not written yet or the mesh changed since: Update Project or \
             Export Now writes it"
                .into()
        }
        _ => "Mesh: the file could not be written".into(),
    };
    if let Some(why) = &export.error {
        tip.push_str(&format!("\n{why}"));
    }
    let count = export.model.mesh.element_count();
    if count > 0 {
        tip.push_str(&format!(
            "\n{count} elements, written as a CalculiX input file."
        ));
    }
    vec![
        Row {
            path: mesh.to_vec(),
            text: links::MESH.to_owned(),
            depth: 0,
            icon: NodeIcon::Mesh,
            state: Some(state),
            expander: None,
            input: true,
            output: false,
            linked_from: source,
            collection: false,
            partial: false,
            widget: None,
            tip,
        },
        Row {
            path: vec![PARAMETERS.to_owned(), "File".to_owned()],
            text: "File".to_owned(),
            depth: 0,
            icon: NodeIcon::Export,
            state: None,
            expander: None,
            input: false,
            output: false,
            linked_from: None,
            collection: false,
            partial: false,
            widget: Some(RowWidget::ExportPath),
            tip: "The CalculiX input file (.inp) to write, with the nodes, elements, node sets \
                  and surfaces of the mesh. A relative path starts at the folder of the \
                  project file. Double-click the block to choose it."
                .into(),
        },
    ]
}

/// The first part of the path of a parameter row of a mesh block.
const PARAMETERS: &str = links::PARAMETERS;

/// The regions a measure block can take its value in, by name, with their nodes on the mesh
/// as it is now: the node sets of the mesh and of the model, and the regions of the boundary
/// conditions and loads, such as "Fixed (boundary condition)". Those picked on the geometry
/// or on the sides of a mesh block are found on each new mesh again.
fn measure_regions(data: &links::Data) -> BTreeMap<String, Vec<plx_mesh::NodeId>> {
    let mut regions: BTreeMap<String, Vec<plx_mesh::NodeId>> = data.mesh.node_sets.clone();
    let mut add = |name: String, region: &plx_model::Region| {
        let nodes = region.nodes(data.mesh);
        if !nodes.is_empty() {
            regions.entry(name).or_insert(nodes);
        }
    };
    for set in &data.fe.mesh_sets {
        add(set.name.clone(), &set.region);
    }
    for step in &data.fe.steps {
        for bc in &step.boundary_conditions {
            add(format!("{} (boundary condition)", bc.name), &bc.region);
        }
        for load in &step.loads {
            add(format!("{} (load)", load.name), &load.region);
        }
    }
    regions
}

/// The key of the factor of a refine mesh block, the parameter row it takes numbers at.
/// Which mesh of an analysis counts for the fingerprint of the blocks a study runs.
#[derive(Clone, Copy, PartialEq)]
enum StudyMesh {
    /// Its own mesh, which the study leaves as it is.
    Own,
    /// A mesh it makes anew from what a block of the study gives it: not the mesh, which
    /// changes from point to point, but how it is made.
    Remeshed,
    /// None, as for a materials block.
    None,
}

/// The fingerprint of an analysis that a study runs, see [`ProjectApp::study_body_key`]:
/// its model without the exposed values the study gives, `driven`, and its mesh as `mesh`
/// says.
fn study_analysis_print(data: links::Data, mesh: StudyMesh, driven: impl Fn(u32) -> bool) -> u64 {
    let mut fe = data.fe.clone();
    for exposed in &data.fe.exposed {
        if driven(exposed.key) {
            fe.set_value(&exposed.path, 0.0);
        }
    }
    let empty = plx_mesh::FeMesh::default();
    // The parts that driven mesh operations move change from point to point: their nodes
    // count without their coordinates.
    let moved: Vec<usize> = (data.fe.exposed.iter())
        .filter(|e| driven(e.key))
        .filter_map(|e| match &e.path.item {
            plx_model::ItemRef::MeshOperation(name) => {
                data.fe.mesh_operations.iter().find(|o| &o.name == name)
            }
            _ => None,
        })
        .flat_map(|o| o.part_indices(data.mesh))
        .collect();
    let still;
    let fe_mesh = match mesh {
        StudyMesh::Own if moved.is_empty() => data.mesh,
        StudyMesh::Own => {
            let mut copy = data.mesh.clone();
            for node in data.mesh.part_nodes(&moved) {
                copy.set_node(node, [0.0; 3]);
            }
            still = copy;
            &still
        }
        StudyMesh::Remeshed | StudyMesh::None => &empty,
    };
    let mut print = plx_mesh::Fnv::new();
    print.add(&plx_model::fingerprint::solve_input(fe_mesh, &fe).to_le_bytes());
    // How it meshes: the element sizes, the mesh setup and the refinement.
    if mesh != StudyMesh::None
        && let Some(geometry) = data.geometry
    {
        print.add_debug(&geometry.meshing);
        print.add_debug(&geometry.mesh_items);
        print.add(&geometry.refinement.to_le_bytes());
    }
    print.finish()
}

const REFINEMENT: &str = "refinement";

/// The rows of a parameters block: a number each, edited in place, with a pin to give it.
fn parameter_rows(node: &Node, incoming: &[(Vec<String>, String)]) -> Vec<Row> {
    let Some(block) = node.as_parameters() else {
        return Vec::new();
    };
    (block.parameters.iter().enumerate())
        .map(|(index, parameter)| Row {
            path: vec![links::VALUE.to_owned(), parameter.name.clone()],
            text: parameter.name.clone(),
            depth: 0,
            icon: NodeIcon::Setup,
            state: None,
            expander: None,
            input: true,
            output: true,
            linked_from: (incoming.iter())
                .find(|(slot, _)| {
                    slot.len() == 2 && slot[0] == links::VALUE && slot[1] == parameter.name
                })
                .map(|(_, source)| source.clone()),
            collection: false,
            partial: false,
            widget: Some(RowWidget::Value(index)),
            tip: format!(
                "{}, a {}: drag its pin to a parameter of another block, such as a length of \
                 a mesh block or an exposed parameter of an analysis, which then takes this \
                 value. Its left pin takes the value of another parameter of the same kind.",
                parameter.name,
                parameter.kind.label().to_lowercase()
            ),
        })
        .collect()
}

/// The rows of a measure block: the results it takes, what it measures and the value it
/// gives.
fn measure_rows(
    node: &Node,
    incoming: &[(Vec<String>, String)],
    state: CellState,
    busy: bool,
) -> Vec<Row> {
    let Some(measure) = node.as_measure() else {
        return Vec::new();
    };
    let units = measure.units;
    let row = |path: &[&str], text: String, icon, tip: String| Row {
        path: path.iter().map(|p| p.to_string()).collect(),
        text,
        depth: 0,
        icon,
        state: None,
        expander: None,
        input: false,
        output: false,
        linked_from: None,
        collection: false,
        partial: false,
        widget: None,
        tip,
    };
    let path = [links::MEASURE.to_owned()];
    let linked_from = (incoming.iter())
        .find(|(slot, _)| slot.as_slice() == path)
        .map(|(_, source)| source.clone());
    let results = Row {
        input: true,
        linked_from,
        ..row(
            &[links::MEASURE],
            "Results".into(),
            NodeIcon::Results,
            "The results of a step to measure: drag the pin of a step under Results of an \
             analysis here."
                .into(),
        )
    };
    let what = row(
        &[],
        measure.kind.describe(),
        NodeIcon::Setup,
        "What the block measures; double-click the block to change it.".into(),
    );
    let text = match (measure.value, busy) {
        (_, true) => "Computing …".to_owned(),
        (Some(value), false) => crate::measure::shown(&measure.kind, value, units),
        (None, false) => "No value".to_owned(),
    };
    let mut tip = "The value, in the units of the block: drag its pin to the Outputs of a \
                   parameter study, which reads it. It is computed anew whenever the results or the definition change."
        .to_owned();
    if let Some(why) = &measure.error {
        tip.push_str(&format!("\nNo value: {why}."));
    }
    let value = Row {
        output: true,
        state: Some(state),
        ..row(&[links::VALUE, "Result"], text, NodeIcon::Results, tip)
    };
    let mut rows = vec![results, what];
    if measure.kind.point().is_some() {
        for (axis, key) in POINT_AXES.into_iter().enumerate() {
            let path = [links::PARAMETERS, key];
            let linked_from = (incoming.iter())
                .find(|(slot, _)| slot.iter().eq(path.iter()))
                .map(|(_, source)| source.clone());
            let label = key.to_uppercase();
            rows.push(Row {
                input: true,
                linked_from,
                widget: Some(RowWidget::Coordinate(axis)),
                ..row(
                    &path,
                    label.clone(),
                    NodeIcon::Setup,
                    format!(
                        "{label} of the point the field is read at, a length. Type it, or \
                         drag the pin of a number here: a parameter, a Math block or an \
                         input of a parameter study."
                    ),
                )
            });
        }
    }
    rows.push(value);
    rows
}

/// The rows of a Campbell block: the analysis it sweeps, how far its sweep is and the
/// critical speeds of its diagram.
fn campbell_rows(node: &Node, incoming: &[(Vec<String>, String)], state: CellState) -> Vec<Row> {
    let Some(window) = node.as_campbell().and_then(|c| c.window.as_deref()) else {
        return Vec::new();
    };
    let row = |path: &[&str], text: String, icon, tip: String| Row {
        path: path.iter().map(|p| p.to_string()).collect(),
        text,
        depth: 0,
        icon,
        state: None,
        expander: None,
        input: false,
        output: false,
        linked_from: None,
        collection: false,
        partial: false,
        widget: None,
        tip,
    };
    let path = [links::ANALYSIS.to_owned()];
    let linked_from = (incoming.iter())
        .find(|(slot, _)| slot.as_slice() == path)
        .map(|(_, source)| source.clone());
    let tip = match state {
        CellState::Missing if linked_from.is_none() => {
            "The analysis to sweep: drag the Setup of an analysis with a Complex Frequency \
             step here."
        }
        CellState::Missing => "Not swept yet: Update Project or Start sweep in its diagram.",
        CellState::UpToDate => "The diagram is up to date with the analysis.",
        CellState::Running => "The sweep is running.",
        _ => {
            "The analysis changed since the sweep: Update Project or Start sweep in its diagram \
             sweeps it again."
        }
    };
    let analysis = Row {
        input: true,
        state: Some(state),
        linked_from,
        ..row(
            &[links::ANALYSIS],
            "Analysis".into(),
            NodeIcon::Setup,
            tip.into(),
        )
    };
    let mut rows = vec![
        analysis,
        row(
            &["Campbell", "Speeds"],
            window.progress(),
            NodeIcon::Solution,
            "The speeds swept so far; double-click the block for the diagram.".into(),
        ),
    ];
    const SHOWN: usize = 5;
    let critical = window.critical_speeds();
    for (index, speed) in critical.iter().enumerate().take(SHOWN) {
        let order = speed.order_label();
        rows.push(row(
            &["Campbell", "Critical", &index.to_string()],
            format!(
                "{order}: {} rpm",
                crate::numeric::format_physical(crate::campbell::rpm(speed.speed).round())
            ),
            NodeIcon::Results,
            if speed.order == 0.0 {
                format!(
                    "Critical speed: the backward wave of mode {} ({}) comes to rest here, a \
                     standing wave in the stationary frame.",
                    speed.mode,
                    speed.branch_label()
                )
            } else {
                format!(
                    "Critical speed: mode {} ({}) crosses the {order} engine order line here.",
                    speed.mode,
                    speed.branch_label()
                )
            },
        ));
    }
    if critical.len() > SHOWN {
        rows.push(row(
            &["Campbell", "More"],
            format!("{} more critical speeds", critical.len() - SHOWN),
            NodeIcon::Results,
            "The diagram lists them all.".into(),
        ));
    }
    rows
}

/// Tooltip of an Add Mesh entry.
/// A kind of block the schematic adds: the menus and the picker list these, in this order.
enum BlockKind {
    Analysis,
    Materials,
    CrossSections,
    Mesh(MeshPrimitive),
    RefineMesh,
    DeformedMesh,
    CadOperation,
    Parameters,
    Math,
    Switch,
    Measure,
    Study,
    Campbell,
    MeshExport,
    File,
    Note,
}

fn block_kinds() -> Vec<BlockKind> {
    vec![
        BlockKind::Analysis,
        BlockKind::Materials,
        BlockKind::CrossSections,
        BlockKind::Mesh(MeshPrimitive::default_box()),
        BlockKind::Mesh(MeshPrimitive::default_rectangle()),
        BlockKind::RefineMesh,
        BlockKind::DeformedMesh,
        BlockKind::CadOperation,
        BlockKind::Parameters,
        BlockKind::Math,
        BlockKind::Switch,
        BlockKind::Measure,
        BlockKind::Study,
        BlockKind::Campbell,
        BlockKind::MeshExport,
        BlockKind::File,
        BlockKind::Note,
    ]
}

impl BlockKind {
    fn entry(&self) -> node_picker::Entry {
        let (category, label, keywords, tip) = match self {
            Self::Analysis => (
                "Analysis",
                "Analysis",
                "new model static frequency buckle dynamic heat steps solve calculix",
                ANALYSIS_TIP,
            ),
            Self::Materials => (
                "Materials",
                "Materials",
                "library elastic density",
                MATERIALS_TIP,
            ),
            Self::CrossSections => (
                "Cross-Sections",
                "Cross-Sections",
                "beam profile section rectangle circle pipe box tube area moment of inertia",
                CROSS_SECTIONS_TIP,
            ),
            Self::Mesh(primitive) => {
                let keywords = match primitive {
                    MeshPrimitive::Box { .. } => "block cube hexahedra 3D structured",
                    MeshPrimitive::Rectangle { .. } => "square quadrilaterals 2D structured",
                };
                ("Mesh", primitive.label(), keywords, mesh_tip(primitive))
            }
            Self::RefineMesh => (
                "Mesh",
                "Refine Mesh",
                "finer coarser factor divisions element size convergence study",
                "Adds a block that makes the mesh linked to it again, finer or coarser by a \
                 factor: the divisions of a mesh block times the factor, or the element sizes \
                 of geometry divided by it. A parameter study over its factor shows whether \
                 the results converge.",
            ),
            Self::DeformedMesh => (
                "Mesh",
                "Deformed Mesh",
                "imperfection buckling mode shape displaced displacement scale factor \
                 nonlinear initial geometry eigenmode",
                "Adds a block that gives the mesh of an analysis with every node moved by its \
                 displacement in one mode or increment of a step, times a factor: link the \
                 results of a buckle step to it, and its mesh to a nonlinear static analysis, \
                 for a buckling mode as the imperfection.",
            ),
            Self::CadOperation => (
                "Geometry",
                "CAD Operation",
                "fillet chamfer round bevel edge radius push pull offset face move thickness \
                 cad geometry modify parametric opencascade",
                "Adds a block that changes the geometry linked to it: a fillet or a chamfer of \
                 picked edges, or planar faces moved along their normal. Its radius, distance \
                 or offset is a row with a pin, so a parameter study can vary it; each design \
                 point makes the geometry again, and the analyses after it mesh it anew.",
            ),
            Self::Parameters => (
                "Parameters",
                "Parameters",
                "number value variable length count factor",
                PARAMETERS_TIP,
            ),
            Self::Math => (
                "Parameters",
                "Math",
                "add subtract multiply divide minimum maximum clamp number formula derived \
                 calculate expression",
                "Adds a block that computes a number from two others, like Blender's Math \
                 node: add, subtract, multiply or divide by a factor, the smaller or the larger, \
                 or a value kept between two limits. For parameters derived from others, such \
                 as a length twice another one.",
            ),
            Self::Switch => (
                "Parameters",
                "Switch",
                "choose select variant case index option number",
                "Adds a block that gives one of several numbers, picked by its index counted \
                 from 1. With the index as an input of a parameter study, the design points \
                 run through the variants.",
            ),
            Self::Measure => (
                "Measure",
                "Measure",
                "result value maximum stress displacement eigenfrequency key figure objective",
                MEASURE_TIP,
            ),
            Self::Study => (
                "Study",
                "Parameter Study",
                "design points table sweep variation sensitivity mesh convergence optimizer loop",
                STUDY_TIP,
            ),
            Self::Campbell => (
                "Campbell",
                "Campbell",
                "rotor speed rpm whirl critical speed complex frequency coriolis sweep diagram",
                "Adds a block that sweeps the analysis linked to it over the speed of its \
                 Complex Frequency step and keeps the Campbell diagram with the critical speeds. \
                 Link the Setup of an analysis with a Complex Frequency step to its Analysis \
                 row.",
            ),
            Self::MeshExport => (
                "Export",
                "Mesh Export",
                "write save file inp calculix output sink",
                "Adds a block that writes the mesh linked to it to a CalculiX input file when \
                 the project is updated, after the analyses, studies and blocks before it.",
            ),
            Self::File => (
                "File",
                "File",
                "import read external frd results inp mesh step iges brep stl geometry \
                 submodel global",
                "Adds a block that reads a file outside the project: the results of each step \
                 of a results file (.frd), for the global results of a submodel or \
                 temperatures, the mesh of an input file (.inp), or geometry (STEP, IGES, \
                 BREP, STL). It reads the file again when it changes.",
            ),
            Self::Note => (
                "Note",
                "Note",
                "text comment remark label annotation",
                "Adds a note: a text in the schematic for whoever reads the project. It has no \
                 pins. Frames (Ctrl+J) group blocks under a name.",
            ),
        };
        node_picker::Entry {
            category,
            label,
            keywords,
            tip,
        }
    }

    /// Adds its block at the position.
    fn action(&self, pos: egui::Pos2) -> Action {
        match self {
            Self::Analysis => Action::New(pos),
            Self::Materials => Action::AddMaterials(pos),
            Self::CrossSections => Action::AddCrossSections(pos),
            Self::Mesh(primitive) => Action::AddMesh(pos, primitive.clone()),
            Self::RefineMesh => Action::AddRefineMesh(pos),
            Self::DeformedMesh => Action::AddDeformedMesh(pos),
            Self::CadOperation => Action::AddCadOperation(pos),
            Self::Parameters => Action::AddParameters(pos),
            Self::Math => Action::AddMath(pos),
            Self::Switch => Action::AddSwitch(pos),
            Self::Measure => Action::AddMeasure(pos),
            Self::Study => Action::AddStudy(pos),
            Self::MeshExport => Action::AddMeshExport(pos),
            Self::Campbell => Action::AddCampbell(pos),
            Self::File => Action::AddFile(pos),
            Self::Note => Action::AddNote(pos),
        }
    }
}

fn mesh_tip(primitive: &MeshPrimitive) -> &'static str {
    match primitive {
        MeshPrimitive::Box { .. } => {
            "Adds a block that meshes a box with hexahedra, for 3D analyses; its sides are \
             node sets and surfaces X-, X+, Y-, Y+, Z-, Z+."
        }
        MeshPrimitive::Rectangle { .. } => {
            "Adds a block that meshes a rectangle with quadrilaterals, for 2D analyses; its \
             sides are node sets and surfaces X-, X+, Y-, Y+."
        }
    }
}

/// The row of all materials of a materials block, open when the block is added.
fn material_set_path() -> Vec<String> {
    collection_path(Collection::Materials)
}

fn cross_section_set_path() -> Vec<String> {
    collection_path(Collection::CrossSections)
}

/// The row of all items of a collection block, as the category of an analysis' setup.
fn collection_path(collection: Collection) -> Vec<String> {
    vec![links::SETUP.to_owned(), collection.title().to_owned()]
}

/// "materials block" as "Materials Block", for window titles.
fn title_case(text: &str) -> String {
    (text.split(' '))
        .map(|word| {
            let mut chars = word.chars();
            chars.next().map_or_else(String::new, |first| {
                first.to_uppercase().chain(chars).collect()
            })
        })
        .collect::<Vec<_>>()
        .join(" ")
}

/// The row a path is shown in: the row itself, or the closed row it lies in.
fn shown_row(rows: &[Row], path: &[String]) -> Option<usize> {
    (rows.iter().enumerate())
        .filter(|(_, row)| links::covers(&row.path, path))
        .max_by_key(|(_, row)| row.path.len())
        .map(|(index, _)| index)
}

/// The symbol of the type of the step called `name`.
fn step_icon(fe: Option<&plx_model::FeModel>, name: &str) -> NodeIcon {
    (fe.and_then(|fe| fe.steps.iter().find(|s| s.name == name))).map_or(NodeIcon::Step, |s| {
        NodeIcon::StepType(StepGlyph::from(&s.kind))
    })
}

/// The symbol of each category of the setup.
fn category_icon(category: &str) -> NodeIcon {
    match category {
        "Features" => NodeIcon::Features,
        "Materials" => NodeIcon::Materials,
        "Sections" => NodeIcon::Sections,
        crate::tree::CROSS_SECTIONS => NodeIcon::CrossSections,
        "Constraints" => NodeIcon::Constraints,
        "Contacts" => NodeIcon::Contacts,
        "Amplitudes" => NodeIcon::Amplitudes,
        "Initial Conditions" => NodeIcon::InitialConditions,
        _ => NodeIcon::Steps,
    }
}

/// What an input of a step under Setup takes, for its tooltip.
fn step_input_tip(step: &str, input: &str) -> String {
    let what = match input {
        links::TEMPERATURES => format!(
            "the temperatures of a thermal step of another analysis, as a defined field of \
             {step}"
        ),
        links::EIGENMODES => format!(
            "the eigenmodes of a frequency step of another analysis (with Storage on), which \
             the modal or steady state dynamics of {step} use"
        ),
        _ => format!(
            "the results of a step of another analysis, the global model: {step} reads the \
             displacements on the cut boundary from them, as a submodel"
        ),
    };
    format!("{input}: takes {what}. Drop the results of a step here.")
}

/// Colour of the wires of a link: references in blue, results in orange, states in green,
/// numbers in violet.
fn wire_color(path: &[String]) -> egui::Color32 {
    if path.first().is_some_and(|c| {
        c == links::VALUE || c == links::PARAMETERS || c == links::LOOP || c == links::OUTPUTS
    }) {
        crate::style::adapt(egui::Color32::from_rgb(140, 80, 200))
    } else if path.first().is_some_and(|c| c == links::STATE) {
        crate::style::adapt(egui::Color32::from_rgb(40, 150, 60))
    } else if path.first().is_some_and(|c| {
        c == links::RESULTS
            || c == links::MEASURE
            || c == links::STEP_RESULT
            || c == links::GLOBAL_RESULTS
    }) || links::is_step_input(path)
    {
        crate::style::adapt(egui::Color32::from_rgb(230, 130, 20))
    } else {
        crate::style::adapt(egui::Color32::from_rgb(40, 110, 200))
    }
}

/// The tint of the header band of a block of `kind`, like the node groups of Blender; `None`
/// for the default one.
fn header_fill(kind: Kind) -> Option<egui::Color32> {
    Some(match kind {
        Kind::Materials | Kind::CrossSections => crate::style::materials_header(),
        Kind::Mesh | Kind::RefineMesh | Kind::DeformedMesh | Kind::File => {
            crate::style::mesh_header()
        }
        Kind::Parameters | Kind::Math | Kind::Switch => crate::style::parameters_header(),
        Kind::Measure => crate::style::measure_header(),
        Kind::Study | Kind::Campbell => crate::style::study_header(),
        Kind::MeshExport => crate::style::export_header(),
        Kind::Note => crate::style::note_header(),
        _ => return None,
    })
}

struct Viewer {
    nodes: Vec<BlockView>,
    actions: Vec<Action>,
    /// From the schematic to the screen.
    to_global: egui::emath::TSTransform,
    /// Where the schematic is shown, when its view is to fit all blocks.
    show_all: Option<egui::Rect>,
    /// Where the schematic is shown and the point of it the view is to centre on.
    look_at: Option<(egui::Rect, egui::Pos2)>,
    /// The feedback wires of parameter studies: study, item, target and slot.
    feedback: Vec<(u64, Vec<String>, u64, Vec<String>)>,
    /// Where the blocks were drawn in this pass, which feedback wires go around below.
    rects: Vec<egui::Rect>,
    /// The pin a wire is being dragged from.
    drag: Option<PinKey>,
    /// Why the wire being dragged cannot end at each pin of the other side; `None` for the
    /// pins it fits.
    verdicts: HashMap<PinKey, Option<String>>,
    /// An update or loop is under way: the out-of-date symbols cannot be clicked.
    busy: bool,
    /// The frames, drawn behind the blocks, the size of each block as last drawn, and the
    /// frame whose label is being edited.
    frames: Vec<file::Frame>,
    sizes: HashMap<u64, egui::Vec2>,
    frame_label: Option<(usize, String)>,
    /// The size of each block as drawn now.
    drawn: HashMap<u64, egui::Vec2>,
    /// The note whose text takes the keyboard.
    focus_note: Option<u64>,
    /// The blocks selected, which the items of a block's menu work on if it is one of them.
    selected: Vec<u64>,
}

impl Viewer {
    fn block(&self, id: u64) -> Option<&BlockView> {
        self.nodes.iter().find(|n| n.id == id)
    }

    /// `pin` of the row of `path` in the block `id`, marked as fitting the wire being dragged
    /// or not, and kept where it is drawn.
    fn typed(&self, mut pin: BlockPin, id: u64, path: &[String], output: bool) -> BlockPin {
        let key = PinKey {
            id,
            path: path.to_vec(),
            output,
        };
        if self.drag.as_ref().is_some_and(|d| d.output != output) {
            pin.fits = self.verdicts.get(&key).map(Option::is_none);
        }
        pin.key = Some(key);
        pin
    }

    /// The analysis and step whose results a wire dropped on empty space comes from.
    fn dropped_results(&self, pins: AnyPins, snarl: &Snarl<u64>) -> Option<(u64, String)> {
        let AnyPins::Out([pin]) = pins else {
            return None;
        };
        let (id, row) = self.row(snarl, pin.node, pin.output)?;
        match row.path.as_slice() {
            [cell, step] if cell == links::RESULTS && row.output => Some((id, step.clone())),
            _ => None,
        }
    }

    /// A row that edits a number of a block in place: its label, and the field at the
    /// right; read-only, in the link colour, while a value link gives it.
    fn edit_row(&mut self, ui: &mut egui::Ui, id: u64, row: &Row, widget: RowWidget) {
        let Some(block) = self.block(id) else {
            return;
        };
        if let RowWidget::ExportPath = widget {
            let mut path = block.export_path.clone();
            let (rect, label) =
                ui.allocate_exact_size(egui::vec2(ROW_WIDTH, ROW_HEIGHT), egui::Sense::hover());
            ui.painter().text(
                egui::pos2(rect.left() + 30.0, rect.center().y),
                egui::Align2::LEFT_CENTER,
                &row.text,
                egui::TextStyle::Body.resolve(ui.style()),
                ui.visuals().text_color(),
            );
            label.on_hover_text(&row.tip);
            let field = egui::Rect::from_min_max(
                egui::pos2(rect.left() + 62.0, rect.top() + 1.0),
                egui::pos2(rect.right() - 4.0, rect.bottom() - 1.0),
            );
            let mut ui = ui.new_child(
                egui::UiBuilder::new()
                    .max_rect(field)
                    .id_salt((id, &row.path)),
            );
            let response = ui
                .add_sized(
                    field.size(),
                    egui::TextEdit::singleline(&mut path).hint_text("folder/mesh.inp"),
                )
                .on_hover_text(&row.tip);
            if response.changed() {
                self.actions.push(Action::SetExportPath(id, path));
            }
            return;
        }
        let (units, space, refinement) = (block.units, block.space, block.refinement);
        let cad_length = block.cad_length;
        let deform = block.deform;
        let point = block.point;
        let parameters = block.parameters.clone();
        let exposed = block.exposed.clone();
        let block_primitive = block.primitive.clone();
        let sense = match widget {
            RowWidget::Exposed(_) => egui::Sense::click(),
            _ => egui::Sense::hover(),
        };
        let link_color = wire_color(&[links::VALUE.to_owned()]);
        // The fields of a Math or Switch block leave room for the label of its operation.
        let width = match widget {
            RowWidget::Operation | RowWidget::Input(_) | RowWidget::Choices => 106.0,
            _ => 118.0,
        };
        // The label senses the pointer left of the field only, so that the field shows its
        // own tooltip, with its full value, also while it is read-only.
        let (label_id, rect) = ui.allocate_space(egui::vec2(ROW_WIDTH, ROW_HEIGHT));
        let label = ui.interact(rect.with_max_x(rect.right() - 4.0 - width), label_id, sense);
        // A long label, such as the name of an exposed parameter, ends in an ellipsis before
        // the field; the tooltip has it in full.
        let mut job = egui::text::LayoutJob::simple_singleline(
            row.text.clone(),
            egui::TextStyle::Body.resolve(ui.style()),
            if row.linked_from.is_some() {
                link_color
            } else {
                ui.visuals().text_color()
            },
        );
        job.wrap = egui::text::TextWrapping {
            max_width: (ROW_WIDTH - 36.0 - width).max(0.0),
            max_rows: 1,
            break_anywhere: true,
            overflow_character: Some('…'),
        };
        let galley = ui.painter().layout_job(job);
        let elided = galley.elided;
        ui.painter().galley(
            egui::pos2(rect.left() + 30.0, rect.center().y - galley.size().y / 2.0),
            galley,
            ui.visuals().text_color(),
        );
        let mut tip = if elided {
            format!("{}\n{}", row.text, row.tip)
        } else {
            row.tip.clone()
        };
        if let Some(source) = &row.linked_from {
            tip.push_str(&format!(
                "\nIts value comes from {source}; change it there."
            ));
        }
        let label = label.on_hover_text(&tip);
        if let RowWidget::Exposed(key) = widget {
            let name = (exposed.iter().find(|e| e.key == key)).map_or("", |e| e.name.as_str());
            // The name being typed is kept from frame to frame until the field lets go of the
            // keyboard, and then becomes the parameter's.
            let draft = egui::Id::new(("exposed name draft", id, key));
            let mut text =
                (ui.data(|d| d.get_temp::<String>(draft))).unwrap_or_else(|| name.to_owned());
            let studies: Vec<(u64, String)> = (self.nodes.iter())
                .filter(|n| n.kind == Kind::Study)
                .map(|n| (n.id, n.name.clone()))
                .collect();
            let linked = row.linked_from.is_some();
            label.context_menu(|ui| {
                ui.label("Name");
                let edit = ui.text_edit_singleline(&mut text);
                if edit.has_focus() {
                    ui.data_mut(|d| d.insert_temp(draft, text.clone()));
                }
                if edit.lost_focus() {
                    ui.data_mut(|d| d.remove::<String>(draft));
                    if !text.trim().is_empty() && text.trim() != name {
                        (self.actions).push(Action::RenameExposed(id, key, text.trim().to_owned()));
                        ui.close();
                    }
                }
                ui.separator();
                let add = |ui: &mut egui::Ui| {
                    for (study, name) in &studies {
                        if ui.button(name).clicked() {
                            return Some(*study);
                        }
                    }
                    None
                };
                let tip = if studies.is_empty() {
                    "Add a parameter study to the project first."
                } else if linked {
                    "Its value comes from a link already; remove it first."
                } else {
                    "Makes it an input of the study: a column of its table."
                };
                let menu = ui.add_enabled_ui(!studies.is_empty() && !linked, |ui| {
                    ui.menu_button("Add to Study", add).inner.flatten()
                });
                menu.response.on_hover_text(tip);
                if let Some(study) = menu.inner {
                    self.actions.push(Action::Link {
                        source: study,
                        item: vec![links::LOOP.to_owned()],
                        target: id,
                        slot: row.path.clone(),
                    });
                    ui.close();
                }
                if ui.button("Remove Parameter").clicked() {
                    self.actions.push(Action::Unexpose(id, key));
                    ui.close();
                }
            });
        }
        let field = egui::Rect::from_min_max(
            egui::pos2(rect.right() - 4.0 - width, rect.top() + 1.0),
            egui::pos2(rect.right() - 4.0, rect.bottom() - 1.0),
        );
        let mut ui = ui.new_child(
            egui::UiBuilder::new()
                .max_rect(field)
                .id_salt((id, &row.path)),
        );
        ui.spacing_mut().interact_size.y = field.height();
        match widget {
            RowWidget::Operation | RowWidget::Input(_) | RowWidget::Choices => {
                self.edit_number_row(&mut ui, id, row, widget, field, &tip);
                return;
            }
            RowWidget::Exposed(key) => {
                let Some(exposed) = exposed.iter().find(|e| e.key == key) else {
                    return;
                };
                let (Some(mut value), Some(kind)) = (exposed.value, exposed.kind) else {
                    ui.weak("gone");
                    return;
                };
                let before = value;
                let drag = crate::parameter_set::field(&mut value, kind, units);
                number_field(&mut ui, field, row, drag, &tip);
                if value != before {
                    self.actions.push(Action::SetExposed(id, key, value));
                }
                return;
            }
            RowWidget::Value(index) => {
                let Some(mut parameter) = parameters.get(index).cloned() else {
                    return;
                };
                let before = parameter.value;
                let drag = crate::parameter_set::field(&mut parameter.value, parameter.kind, units);
                number_field(&mut ui, field, row, drag, &tip);
                if parameter.value != before {
                    self.actions
                        .push(Action::SetParameter(id, index, parameter.value));
                }
                return;
            }
            RowWidget::CadLength => {
                let mut value = cad_length;
                let drag = crate::numeric::quantity(&mut value, units, Quantity::Length);
                number_field(&mut ui, field, row, drag, &tip);
                if value != cad_length {
                    self.actions.push(Action::SetCadLength(id, value));
                }
                return;
            }
            RowWidget::Coordinate(axis) => {
                let Some(point) = point else {
                    return;
                };
                let mut value = point[axis];
                let drag = crate::numeric::quantity(&mut value, units, Quantity::Length);
                number_field(&mut ui, field, row, drag, &tip);
                if value != point[axis] {
                    (self.actions).push(Action::SetMeasurePoint(id, POINT_AXES[axis], value));
                }
                return;
            }
            RowWidget::DeformFactor => {
                let mut value = deform.0;
                let drag = crate::numeric::drag_value(&mut value).speed(0.01);
                number_field(&mut ui, field, row, drag, &tip);
                if value != deform.0 {
                    self.actions.push(Action::SetDeformFactor(id, value));
                }
                return;
            }
            RowWidget::DeformIncrement => {
                let mut value = deform.1;
                let drag = crate::numeric::count(&mut value)
                    .range(1..=100_000)
                    .speed(0.05);
                number_field(&mut ui, field, row, drag, &tip);
                if value != deform.1 {
                    (self.actions).push(Action::SetDeformIncrement(id, f64::from(value)));
                }
                return;
            }
            RowWidget::Refinement => {
                let mut value = refinement;
                let drag = crate::numeric::drag_value(&mut value)
                    .range(0.05..=20.0)
                    .speed(0.01);
                number_field(&mut ui, field, row, drag, &tip);
                if value != refinement {
                    self.actions.push(Action::SetRefinement(id, value));
                }
                return;
            }
            RowWidget::Param(_) | RowWidget::Element | RowWidget::ExportPath => {}
        }
        let Some(mut primitive) = block_primitive else {
            return;
        };
        let before = primitive.clone();
        if row.linked_from.is_some() {
            ui.disable();
        }
        let response = match widget {
            RowWidget::ExportPath => return,
            RowWidget::Param(param) => match primitive.slot(param.key) {
                Some(ParamSlot::Length(value)) => {
                    let drag = crate::numeric::quantity(value, units, Quantity::Length);
                    number_field(&mut ui, field, row, drag, &tip);
                    return self.set_primitive(id, primitive, before);
                }
                Some(ParamSlot::Count(value)) => {
                    let drag = crate::numeric::count(value).range(1..=10_000).speed(0.1);
                    number_field(&mut ui, field, row, drag, &tip);
                    return self.set_primitive(id, primitive, before);
                }
                None => return,
            },
            RowWidget::Element => match &mut primitive {
                MeshPrimitive::Box { element, .. } => {
                    egui::ComboBox::from_id_salt(("mesh block element", id))
                        .selected_text(element.as_str())
                        .width(field.width())
                        .show_ui(&mut ui, |ui| {
                            for choice in MeshPrimitive::BOX_ELEMENTS {
                                ui.selectable_value(element, choice.to_owned(), choice);
                            }
                        })
                        .response
                }
                MeshPrimitive::Rectangle {
                    quadratic, reduced, ..
                } => {
                    // The four types of the model space's family, by order and integration.
                    let name = |quadratic: bool, reduced: bool| {
                        let choice = MeshPrimitive::Rectangle {
                            origin: [0.0; 2],
                            size: [1.0; 2],
                            divisions: [1; 2],
                            quadratic,
                            reduced,
                        };
                        choice.element_type(space)
                    };
                    egui::ComboBox::from_id_salt(("mesh block element", id))
                        .selected_text(name(*quadratic, *reduced))
                        .width(field.width())
                        .show_ui(&mut ui, |ui| {
                            for (q, r) in
                                [(false, false), (false, true), (true, false), (true, true)]
                            {
                                let selected = (*quadratic, *reduced) == (q, r);
                                if ui.selectable_label(selected, name(q, r)).clicked() {
                                    (*quadratic, *reduced) = (q, r);
                                }
                            }
                        })
                        .response
                }
            },
            RowWidget::Refinement
            | RowWidget::DeformFactor
            | RowWidget::DeformIncrement
            | RowWidget::CadLength
            | RowWidget::Coordinate(_)
            | RowWidget::Value(_)
            | RowWidget::Exposed(_)
            | RowWidget::Operation
            | RowWidget::Input(_)
            | RowWidget::Choices => return,
        };
        response.on_hover_text(&tip);
        self.set_primitive(id, primitive, before);
    }

    /// Sets the edited mesh primitive of a block, if it changed.
    fn set_primitive(&mut self, id: u64, primitive: MeshPrimitive, before: MeshPrimitive) {
        if primitive != before {
            self.actions.push(Action::SetPrimitive(id, primitive));
        }
    }

    fn row(&self, snarl: &Snarl<u64>, node: NodeId, index: usize) -> Option<(u64, &Row)> {
        let id = *snarl.get_node(node)?;
        Some((id, self.block(id)?.rows.get(index)?))
    }
}

/// Width of the rows of a block.
const ROW_WIDTH: f32 = 210.0;
const ROW_HEIGHT: f32 = 20.0;

/// The field of a number in a row of a block, the one every number of every block shows:
/// as many digits as fit before its unit ([`crate::numeric::fitted`]), the full value in
/// the tooltip; read-only while a value link gives it.
fn number_field(
    ui: &mut egui::Ui,
    field: egui::Rect,
    row: &Row,
    drag: egui::DragValue<'_>,
    tip: &str,
) -> egui::Response {
    if row.linked_from.is_some() {
        ui.disable();
    }
    crate::numeric::fitted(ui, field.size(), drag, tip)
}

impl SnarlViewer<u64> for Viewer {
    fn title(&mut self, node: &u64) -> String {
        self.block(*node)
            .map_or_else(String::new, |b| b.name.clone())
    }

    fn show_header(
        &mut self,
        node: NodeId,
        _inputs: &[InPin],
        _outputs: &[OutPin],
        ui: &mut egui::Ui,
        snarl: &mut Snarl<u64>,
    ) {
        let id = snarl[node];
        let Some(block) = self.block(id) else {
            return;
        };
        let compact = block.compact;
        // A triangle as in Blender: pointing down while the block shows all its rows.
        let (rect, toggle) = ui.allocate_exact_size(egui::vec2(10.0, 16.0), egui::Sense::click());
        let ink = if toggle.hovered() {
            ui.visuals().strong_text_color()
        } else {
            ui.visuals().weak_text_color()
        };
        let c = rect.center();
        let points = if compact {
            vec![
                c + egui::vec2(-2.0, -4.0),
                c + egui::vec2(3.0, 0.0),
                c + egui::vec2(-2.0, 4.0),
            ]
        } else {
            vec![
                c + egui::vec2(-4.0, -2.0),
                c + egui::vec2(4.0, -2.0),
                c + egui::vec2(0.0, 3.0),
            ]
        };
        ui.painter()
            .add(egui::Shape::convex_polygon(points, ink, egui::Stroke::NONE));
        let toggle_tip = if compact {
            "Shows all rows of the block (H)"
        } else {
            "Shows only the header and the rows with a wire (H)"
        };
        let toggled = toggle.on_hover_text(toggle_tip).clicked();
        let (rect, icon) = ui.allocate_exact_size(egui::vec2(16.0, 16.0), egui::Sense::hover());
        match block.kind {
            Kind::Analysis | Kind::Mesh => {
                paint_space(ui.painter(), rect, block.space, ui.visuals().text_color());
                icon.on_hover_text(format!("Model space: {}", block.space.label()));
            }
            Kind::RefineMesh => {
                let ink = ui.visuals().text_color();
                node_icons::paint(ui.painter(), rect.min, NodeIcon::Mesh, ink);
                icon.on_hover_text("Makes the mesh linked to it again, finer or coarser");
            }
            Kind::DeformedMesh => {
                let ink = ui.visuals().text_color();
                node_icons::paint(ui.painter(), rect.min, NodeIcon::Mesh, ink);
                icon.on_hover_text(
                    "Gives the mesh of an analysis deformed by the displacements of a step",
                );
            }
            Kind::CadOperation => {
                let ink = ui.visuals().text_color();
                node_icons::paint(ui.painter(), rect.min, NodeIcon::CadOperation, ink);
                icon.on_hover_text("Changes the geometry linked to it, e.g. rounds edges");
            }
            Kind::Materials => {
                let ink = ui.visuals().text_color();
                node_icons::paint(ui.painter(), rect.min, NodeIcon::Materials, ink);
                icon.on_hover_text("Materials for analyses to link");
            }
            Kind::CrossSections => {
                let ink = ui.visuals().text_color();
                node_icons::paint(ui.painter(), rect.min, NodeIcon::CrossSections, ink);
                icon.on_hover_text("Beam cross-sections for analyses to link");
            }
            Kind::Parameters => {
                let ink = ui.visuals().text_color();
                node_icons::paint(ui.painter(), rect.min, NodeIcon::Setup, ink);
                icon.on_hover_text("Numbers for other blocks to link");
            }
            Kind::Measure => {
                let ink = ui.visuals().text_color();
                node_icons::paint(ui.painter(), rect.min, NodeIcon::Results, ink);
                icon.on_hover_text("A key figure of the results of a step");
            }
            Kind::Study => {
                let ink = ui.visuals().text_color();
                node_icons::paint(ui.painter(), rect.min, NodeIcon::Solution, ink);
                icon.on_hover_text("A table of design points, each solved in turn");
            }
            Kind::MeshExport => {
                let ink = ui.visuals().text_color();
                node_icons::paint(ui.painter(), rect.min, NodeIcon::Export, ink);
                icon.on_hover_text("Writes the mesh linked to it to a file");
            }
            Kind::Campbell => {
                let ink = ui.visuals().text_color();
                node_icons::paint(ui.painter(), rect.min, NodeIcon::Results, ink);
                icon.on_hover_text("Sweeps an analysis over the rotational speed");
            }
            Kind::File => {
                let ink = ui.visuals().text_color();
                let glyph = match FileKind::of(&block.export_path) {
                    Some(FileKind::Geometry) => NodeIcon::Geometry,
                    Some(FileKind::Mesh) => NodeIcon::Mesh,
                    _ => NodeIcon::Results,
                };
                node_icons::paint(ui.painter(), rect.min, glyph, ink);
                icon.on_hover_text("Reads a file outside the project for analyses to link");
            }
            Kind::Note => {
                let ink = ui.visuals().text_color();
                node_icons::paint(ui.painter(), rect.min, NodeIcon::Note, ink);
                icon.on_hover_text("A note for the reader; it gives and takes nothing");
            }
            Kind::Math => {
                let ink = ui.visuals().text_color();
                node_icons::paint(ui.painter(), rect.min, NodeIcon::Math, ink);
                icon.on_hover_text("Computes a number from others");
            }
            Kind::Switch => {
                let ink = ui.visuals().text_color();
                node_icons::paint(ui.painter(), rect.min, NodeIcon::Switch, ink);
                icon.on_hover_text("Gives one of several numbers, picked by its index");
            }
        }
        let text = egui::RichText::new(&block.name).strong();
        let label = ui.add(
            egui::Label::new(text)
                .selectable(false)
                .sense(egui::Sense::click()),
        );
        if block.open {
            ui.weak("(open)");
        }
        if block.loading {
            ui.spinner();
        }
        let (steps, continues, kind) = (block.steps.clone(), block.continues.clone(), block.kind);
        let layout = Layout {
            targets: if self.selected.contains(&id) {
                self.selected.clone()
            } else {
                vec![id]
            },
            compact,
            framed: block.framed,
        };
        if toggled {
            self.actions.push(Action::Compact(vec![id]));
        }
        if label.double_clicked() {
            // A compact block opens to show its rows first, as in Blender.
            self.actions.push(if compact {
                Action::Compact(vec![id])
            } else {
                Action::Open(id, None)
            });
        }
        let tip = match kind {
            Kind::Analysis => {
                "Double-click opens the analysis in Geometry, FE Model or Results; its menu \
                 opens it in a window of its own."
            }
            Kind::Materials => "Double-click edits the materials.",
            Kind::CrossSections => "Double-click edits the cross-sections.",
            Kind::Mesh => "Double-click edits the size and division of the mesh.",
            Kind::RefineMesh => "Its factor makes the mesh finer or coarser.",
            Kind::DeformedMesh => "Its mode and factor are edited in the block.",
            Kind::CadOperation => "Double-click picks the edges and shows the result.",
            Kind::MeshExport => "Double-click chooses the file to write.",
            Kind::Parameters => "Double-click adds, renames or removes parameters.",
            Kind::Measure => "Double-click chooses what it measures.",
            Kind::Study => "Double-click opens its table of design points.",
            Kind::Campbell => "Double-click opens its Campbell diagram and the sweep.",
            Kind::File => "Double-click chooses the file to read.",
            Kind::Note => "Double-click edits its text.",
            Kind::Math => "Its operation and numbers are edited in the block.",
            Kind::Switch => "Its index and choices are edited in the block.",
        };
        // The name takes the clicks of the header, so it opens the block's menu too.
        label.on_hover_text(tip).context_menu(|ui| {
            node_menu(
                ui,
                id,
                kind,
                &steps,
                continues.as_deref(),
                &layout,
                &mut self.actions,
            )
        });
    }

    fn inputs(&mut self, node: &u64) -> usize {
        self.block(*node).map_or(0, |b| b.rows.len())
    }

    fn show_input(
        &mut self,
        pin: &InPin,
        ui: &mut egui::Ui,
        snarl: &mut Snarl<u64>,
    ) -> impl egui_snarl::ui::SnarlPin + 'static {
        let Some((id, row)) = self.row(snarl, pin.id.node, pin.id.input) else {
            return BlockPin::single(hidden_pin());
        };
        let row = row.clone();
        if let Some(widget) = row.widget {
            self.edit_row(ui, id, &row, widget);
            if !row.input {
                return BlockPin::single(hidden_pin());
            }
            // A number of another block can give it its value.
            let color = wire_color(&row.path);
            let info = PinInfo::circle().with_fill(color).with_wire_color(color);
            if self
                .feedback
                .iter()
                .any(|(_, _, target, slot)| *target == id && *slot == row.path)
            {
                let pin = BlockPin::anchored(info, feedback_end(id, &row.path));
                return self.typed(pin, id, &row.path, false);
            }
            return self.typed(BlockPin::single(info), id, &row.path, false);
        }
        let (rect, response) =
            ui.allocate_exact_size(egui::vec2(ROW_WIDTH, ROW_HEIGHT), egui::Sense::click());
        let painter = ui.painter();
        if response.hovered() {
            painter.rect_filled(rect, 2.0, crate::style::hover_fill());
        }
        if let Some(&mark) = self.block(id).and_then(|b| b.chain.get(&row.path)) {
            // Results rows of steps sit one level up from those under Setup > Steps.
            let depth = if row.path.len() == 2 { 1 } else { 2 };
            paint_chain(painter, rect, depth, mark, ui.spacing().item_spacing.y);
        }
        let mut x = rect.left() + 2.0 + 14.0 * row.depth as f32;
        let middle = rect.center().y;
        let mut toggled = false;
        if let Some(open) = row.expander {
            let arrow =
                egui::Rect::from_center_size(egui::pos2(x + 5.0, middle), egui::vec2(10.0, 10.0));
            let color = ui.visuals().text_color();
            let points = if open {
                vec![
                    arrow.left_top() + egui::vec2(1.0, 2.0),
                    arrow.right_top() + egui::vec2(-1.0, 2.0),
                    arrow.center_bottom() - egui::vec2(0.0, 1.0),
                ]
            } else {
                vec![
                    arrow.left_top() + egui::vec2(2.0, 1.0),
                    arrow.right_center() - egui::vec2(1.0, 0.0),
                    arrow.left_bottom() + egui::vec2(2.0, -1.0),
                ]
            };
            painter.add(egui::Shape::convex_polygon(
                points,
                color,
                egui::Stroke::NONE,
            ));
            let pointer = response.interact_pointer_pos();
            toggled = response.clicked() && pointer.is_some_and(|p| p.x < arrow.right() + 2.0);
        }
        x += 12.0;
        let dim = row.path.first().is_some_and(|c| c == INHERITED) && row.path.len() > 1;
        let mut faded = ui.painter().clone();
        if dim {
            faded.multiply_opacity(0.45);
        }
        let ink = ui
            .visuals()
            .text_color()
            .gamma_multiply(if dim { 0.45 } else { 0.75 });
        node_icons::paint(&faded, egui::pos2(x, middle - 7.0), row.icon, ink);
        x += 20.0;
        let color = if dim {
            ui.visuals().text_color().gamma_multiply(0.5)
        } else if row.linked_from.is_some() {
            crate::style::adapt(egui::Color32::from_rgb(40, 110, 200))
        } else {
            ui.visuals().text_color()
        };
        painter.text(
            egui::pos2(x, middle),
            egui::Align2::LEFT_CENTER,
            &row.text,
            egui::TextStyle::Body.resolve(ui.style()),
            color,
        );
        let mut update = None;
        if let Some(state) = row.state {
            let symbol = egui::pos2(rect.right() - 18.0, middle - 8.0);
            // Out of date, or a solution that is not there yet.
            let solved = matches!(
                row.path.first().map(String::as_str),
                Some("Solution" | "Results")
            ) && row.path.len() == 1;
            let due = state == CellState::OutOfDate || (state == CellState::Missing && solved);
            if due && !self.busy {
                // Click: the block is updated with what it takes data from, nothing else.
                let area = egui::Rect::from_min_size(symbol, egui::vec2(16.0, 16.0)).expand(2.0);
                let clicked = ui
                    .interact(
                        area,
                        ui.id().with(("update", id, &row.path)),
                        egui::Sense::click(),
                    )
                    .on_hover_cursor(egui::CursorIcon::PointingHand)
                    .on_hover_text(
                        "Update this block: solves it and the blocks it takes data from that \
                         are out of date, and nothing else.",
                    );
                if clicked.hovered() {
                    painter.rect_filled(area, 3.0, ui.visuals().widgets.hovered.weak_bg_fill);
                }
                update = Some(clicked.clicked());
            }
            status::paint(painter, symbol, state);
        }
        if update == Some(true) {
            self.actions.push(Action::Update(id));
        }
        let mut tip = row.tip.clone();
        if let Some(source) = &row.linked_from {
            tip.push_str(&format!("\nLinked from {source}."));
        }
        if let Some(data) = links::DataType::of(&row.path)
            && (row.input || row.output)
        {
            // The type of the pin, and the model space of what it carries where it has one.
            let space = (self.block(id))
                .filter(|b| data.has_space() && b.has_space)
                .map_or(String::new(), |b| format!(", {}", b.space.label()));
            tip.push_str(&format!("\nPin: {}{space}.", capitalized(data.label())));
        }
        if row.input || row.output {
            tip.push_str("\nRight-click a pin to remove its links.");
        }
        let response = response.on_hover_text(tip);
        if toggled || (row.expander.is_some() && response.double_clicked()) {
            self.actions.push(Action::Toggle(id, row.path.clone()));
        } else if response.double_clicked() {
            self.actions.push(Action::Open(id, workspace_of(&row.path)));
        }
        let color = wire_color(&row.path);
        if row.path.first().is_some_and(|c| c == links::STATE) {
            let pin = BlockPin::single(PinInfo::square().with_fill(color).with_wire_color(color));
            self.typed(pin, id, &row.path, false)
        } else if row.input {
            // A wire takes the mixed colour of its two pins: a wire from the results of a
            // step is orange on both ends, whichever row takes them.
            let results = (pin.remotes.iter()).any(|remote| {
                self.row(snarl, remote.node, remote.output)
                    .is_some_and(|(_, row)| row.path.first().is_some_and(|c| c == links::RESULTS))
            });
            let wire = if results {
                wire_color(&[links::RESULTS.to_owned()])
            } else {
                color
            };
            let info = PinInfo::circle().with_fill(color).with_wire_color(wire);
            let pin = BlockPin {
                collection: row.collection,
                partial: row.partial,
                ..BlockPin::single(info)
            };
            self.typed(pin, id, &row.path, false)
        } else {
            BlockPin::single(hidden_pin())
        }
    }

    fn outputs(&mut self, node: &u64) -> usize {
        self.block(*node).map_or(0, |b| b.rows.len())
    }

    fn show_output(
        &mut self,
        pin: &OutPin,
        ui: &mut egui::Ui,
        snarl: &mut Snarl<u64>,
    ) -> impl egui_snarl::ui::SnarlPin + 'static {
        ui.allocate_exact_size(egui::vec2(0.0, ROW_HEIGHT), egui::Sense::hover());
        match self.row(snarl, pin.id.node, pin.id.output) {
            Some((id, row)) if row.output && row.path.first().is_some_and(|c| c == links::LOOP) => {
                let color = wire_color(&row.path);
                let info = PinInfo::circle().with_fill(color).with_wire_color(color);
                let shown = if pin.remotes.is_empty() {
                    BlockPin::single(info)
                } else {
                    BlockPin::anchored(info, feedback_start(id, &row.path))
                };
                self.typed(shown, id, &row.path, true)
            }
            Some((id, row)) if row.output => {
                let color = wire_color(&row.path);
                // A wire takes the mixed colour of its two pins, so the results pin of a step
                // whose wires all go to a State row gives them the green of a state link.
                let state = vec![links::STATE.to_owned()];
                let continues = !pin.remotes.is_empty()
                    && (pin.remotes.iter()).all(|remote| {
                        self.row(snarl, remote.node, remote.input)
                            .is_some_and(|(_, row)| row.path == state)
                    });
                let wire = if continues { wire_color(&state) } else { color };
                let shown =
                    BlockPin::single(PinInfo::circle().with_fill(color).with_wire_color(wire));
                self.typed(shown, id, &row.path, true)
            }
            _ => BlockPin::single(hidden_pin()),
        }
    }

    // Only a note has a body: its text.
    fn has_body(&mut self, node: &u64) -> bool {
        self.block(*node)
            .is_some_and(|b| b.kind == Kind::Note && !b.compact)
    }

    fn show_body(
        &mut self,
        node: NodeId,
        _inputs: &[InPin],
        _outputs: &[OutPin],
        ui: &mut egui::Ui,
        snarl: &mut Snarl<u64>,
    ) {
        let id = snarl[node];
        let Some(note) = self.block(id).map(|b| b.note.clone()) else {
            return;
        };
        // The text being typed is kept until the field lets go of the keyboard, and then
        // becomes the note's, as one change to undo.
        let draft = egui::Id::new(("note draft", id));
        let mut text = ui
            .data(|d| d.get_temp::<String>(draft))
            .unwrap_or_else(|| note.clone());
        ui.set_min_width(ROW_WIDTH);
        let edit = ui.add(
            egui::TextEdit::multiline(&mut text)
                .desired_width(ROW_WIDTH)
                .desired_rows(3)
                .frame(egui::Frame::NONE)
                .hint_text("Double-click to write a note"),
        );
        if self.focus_note == Some(id) {
            edit.request_focus();
        }
        if edit.has_focus() {
            ui.data_mut(|d| d.insert_temp(draft, text.clone()));
        }
        if edit.lost_focus() {
            ui.data_mut(|d| d.remove::<String>(draft));
            if text != note {
                self.actions.push(Action::SetNote(id, text));
            }
        }
    }

    fn final_node_rect(
        &mut self,
        node: NodeId,
        rect: egui::Rect,
        _ui: &mut egui::Ui,
        snarl: &mut Snarl<u64>,
    ) {
        self.rects.push(rect);
        if let Some(id) = snarl.get_node(node) {
            self.drawn.insert(*id, rect.size());
        }
    }

    /// A grid of dots behind the blocks, like the background of a node editor. Zoomed out,
    /// the dots thin out and keep their size on the screen; see [`dot_grid`].
    fn draw_background(
        &mut self,
        _background: Option<&egui_snarl::ui::BackgroundPattern>,
        viewport: &egui::Rect,
        _snarl_style: &SnarlStyle,
        _style: &egui::Style,
        painter: &egui::Painter,
        snarl: &Snarl<u64>,
    ) {
        let (spacing, radius) = dot_grid(self.to_global.scaling);
        let color = crate::style::schematic_dot();
        let first = |v: f32| (v / spacing).floor() as i32;
        let last = |v: f32| (v / spacing).ceil() as i32;
        for i in first(viewport.left())..=last(viewport.right()) {
            for j in first(viewport.top())..=last(viewport.bottom()) {
                let center = egui::pos2(i as f32 * spacing, j as f32 * spacing);
                painter.circle_filled(center, radius, color);
            }
        }
        self.frames(painter, snarl);
    }

    fn connect(&mut self, from: &OutPin, to: &InPin, snarl: &mut Snarl<u64>) {
        let source = self.row(snarl, from.id.node, from.id.output);
        let target = self.row(snarl, to.id.node, to.id.input);
        if let (Some((source, item)), Some((target, slot))) = (source, target)
            && item.output
            && slot.input
        {
            self.actions.push(Action::Link {
                source,
                item: item.path.clone(),
                target,
                slot: slot.path.clone(),
            });
        }
    }

    fn disconnect(&mut self, from: &OutPin, to: &InPin, snarl: &mut Snarl<u64>) {
        let source = self.row(snarl, from.id.node, from.id.output);
        let target = self.row(snarl, to.id.node, to.id.input);
        if let (Some((source, item)), Some((target, slot))) = (source, target) {
            self.actions.push(Action::Unlink {
                source: Some((source, item.path.clone())),
                target: Some((target, slot.path.clone())),
            });
        }
    }

    fn drop_outputs(&mut self, pin: &OutPin, snarl: &mut Snarl<u64>) {
        if let Some((source, item)) = self.row(snarl, pin.id.node, pin.id.output) {
            self.actions.push(Action::Unlink {
                source: Some((source, item.path.clone())),
                target: None,
            });
        }
    }

    fn drop_inputs(&mut self, pin: &InPin, snarl: &mut Snarl<u64>) {
        if let Some((target, slot)) = self.row(snarl, pin.id.node, pin.id.input) {
            self.actions.push(Action::Unlink {
                source: None,
                target: Some((target, slot.path.clone())),
            });
        }
    }

    // No graph menu: a right-click into empty space opens the block search, as a
    // double-click does.
    fn has_graph_menu(&mut self, _pos: egui::Pos2, _snarl: &mut Snarl<u64>) -> bool {
        false
    }

    fn current_transform(
        &mut self,
        to_global: &mut egui::emath::TSTransform,
        snarl: &mut Snarl<u64>,
    ) {
        if let Some(view) = self.show_all.take() {
            // The blocks, each about as large as it is drawn.
            let blocks = (snarl.nodes_pos_ids()).fold(egui::Rect::NOTHING, |all, (_, pos, id)| {
                let rows = self.block(*id).map_or(0, |b| b.rows.len() + b.steps.len());
                let size = (self.sizes.get(id).copied())
                    .unwrap_or(egui::vec2(260.0, 60.0 + 22.0 * rows as f32));
                all.union(egui::Rect::from_min_size(pos, size))
            });
            // With the label bands of the frames above them.
            let blocks = (self.frames.iter())
                .filter_map(|f| frames::frame_rect(f, snarl, &self.sizes))
                .fold(blocks, egui::Rect::union);
            if blocks.is_finite() {
                let blocks = blocks.expand(40.0);
                let scale = (view.width() / blocks.width())
                    .min(view.height() / blocks.height())
                    .clamp(MIN_SCALE, 1.0);
                *to_global = egui::emath::TSTransform::new(
                    view.center().to_vec2() - blocks.center().to_vec2() * scale,
                    scale,
                );
            }
        }
        if let Some((view, point)) = self.look_at.take() {
            to_global.translation = view.center().to_vec2() - point.to_vec2() * to_global.scaling;
        }
        self.to_global = *to_global;
    }

    /// Header bands tinted by the kind of block, like the node groups of Blender.
    fn header_frame(
        &mut self,
        default: egui::Frame,
        node: NodeId,
        _inputs: &[InPin],
        _outputs: &[OutPin],
        snarl: &Snarl<u64>,
    ) -> egui::Frame {
        let kind = snarl
            .get_node(node)
            .and_then(|id| self.block(*id))
            .map(|b| b.kind);
        kind.and_then(header_fill)
            .map_or(default, |fill| default.fill(fill))
    }

    fn has_node_menu(&mut self, _node: &u64) -> bool {
        true
    }

    fn show_node_menu(
        &mut self,
        node: NodeId,
        _inputs: &[InPin],
        _outputs: &[OutPin],
        ui: &mut egui::Ui,
        snarl: &mut Snarl<u64>,
    ) {
        let id = snarl[node];
        let block = self.block(id);
        let steps = block.map(|b| b.steps.clone()).unwrap_or_default();
        let continues = block.and_then(|b| b.continues.clone());
        let kind = block.map_or(Kind::Analysis, |b| b.kind);
        let layout = Layout {
            targets: if self.selected.contains(&id) {
                self.selected.clone()
            } else {
                vec![id]
            },
            compact: block.is_some_and(|b| b.compact),
            framed: block.is_some_and(|b| b.framed),
        };
        node_menu(
            ui,
            id,
            kind,
            &steps,
            continues.as_deref(),
            &layout,
            &mut self.actions,
        );
    }

    fn has_dropped_wire_menu(&mut self, src_pins: AnyPins, snarl: &mut Snarl<u64>) -> bool {
        self.dropped_results(src_pins, snarl).is_some()
    }

    fn show_dropped_wire_menu(
        &mut self,
        pos: egui::Pos2,
        ui: &mut egui::Ui,
        src_pins: AnyPins,
        snarl: &mut Snarl<u64>,
    ) {
        let Some((source, step)) = self.dropped_results(src_pins, snarl) else {
            ui.close();
            return;
        };
        let tip = "A new analysis that continues from the state after this step: it takes the \
                   model and the steps up to it, and its own steps are solved from there.";
        if ui.button("Continue from here").on_hover_text(tip).clicked() {
            self.actions.push(Action::Continue {
                source,
                step,
                position: pos,
            });
            ui.close();
        }
    }
}

/// A symbol of the model space: a cube for 3D, a thin plate for plane stress, a long prism
/// for plane strain and a profile turned about its axis for axisymmetric models.
fn paint_space(painter: &egui::Painter, rect: egui::Rect, space: ModelSpace, color: egui::Color32) {
    let stroke = egui::Stroke::new(1.2, color);
    let p = |x: f32, y: f32| rect.min + egui::vec2(x, y);
    let line = |points: &[egui::Pos2]| {
        painter.add(egui::Shape::line(points.to_vec(), stroke));
    };
    match space {
        ModelSpace::ThreeD => {
            line(&[
                p(2.0, 6.0),
                p(11.0, 6.0),
                p(11.0, 15.0),
                p(2.0, 15.0),
                p(2.0, 6.0),
            ]);
            line(&[p(2.0, 6.0), p(6.0, 2.0), p(15.0, 2.0), p(11.0, 6.0)]);
            line(&[p(15.0, 2.0), p(15.0, 11.0), p(11.0, 15.0)]);
        }
        ModelSpace::PlaneStress => {
            // A thin plate, loaded in its plane.
            let face = [p(1.0, 4.0), p(13.0, 4.0), p(13.0, 14.0), p(1.0, 14.0)];
            painter.add(egui::Shape::convex_polygon(
                face.to_vec(),
                color.gamma_multiply(0.25),
                stroke,
            ));
            line(&[p(1.0, 4.0), p(3.0, 2.0), p(15.0, 2.0), p(13.0, 4.0)]);
            line(&[p(15.0, 2.0), p(15.0, 12.0), p(13.0, 14.0)]);
        }
        ModelSpace::PlaneStrain => {
            // A long body: a small section drawn out far into the depth.
            line(&[
                p(1.0, 9.0),
                p(7.0, 9.0),
                p(7.0, 15.0),
                p(1.0, 15.0),
                p(1.0, 9.0),
            ]);
            line(&[p(1.0, 9.0), p(9.0, 1.0), p(15.0, 1.0), p(7.0, 9.0)]);
            line(&[p(15.0, 1.0), p(15.0, 7.0), p(7.0, 15.0)]);
        }
        ModelSpace::Axisymmetric => {
            // The axis of revolution, the profile beside it and the turn about the axis.
            let dash = egui::Stroke::new(1.0, color);
            painter.extend(egui::Shape::dashed_line(
                &[p(3.0, 0.5), p(3.0, 15.5)],
                dash,
                2.0,
                1.5,
            ));
            let profile = [p(7.0, 5.0), p(12.0, 5.0), p(12.0, 13.0), p(7.0, 13.0)];
            painter.add(egui::Shape::convex_polygon(
                profile.to_vec(),
                color.gamma_multiply(0.25),
                stroke,
            ));
            let arc: Vec<egui::Pos2> = (0..=12)
                .map(|i| {
                    let t = std::f32::consts::PI * i as f32 / 12.0;
                    p(3.0 + 6.0 * t.cos(), 2.5 + 1.8 * t.sin())
                })
                .collect();
            line(&arc);
        }
    }
}

/// The input pin of a row: round for a single value, which takes one wire, a new one in
/// place of the old; a tall rounded rectangle for a collection, which takes any number of
/// wires, like Blender's multi-input sockets.
struct BlockPin {
    info: PinInfo,
    collection: bool,
    /// A category link to the collection leaves out items: drawn lighter.
    partial: bool,
    /// Where the pin is drawn is kept under this id, for a wire drawn by the project window:
    /// the dashed feedback of a loop. The wire of the schematic is then not drawn.
    anchor: Option<egui::Id>,
    /// The block and row of the pin, whose place on the screen is kept for the project window
    /// to find the pin a wire is dragged from, see [`PIN_RECTS`].
    key: Option<PinKey>,
    /// Whether the wire being dragged may end at the pin: drawn with a ring if so, faded if
    /// not; `None` while no wire is dragged, and for the pin it comes from.
    fits: Option<bool>,
}

/// A pin of the schematic: its block, the path of its row and whether it is an output.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
struct PinKey {
    id: u64,
    path: Vec<String>,
    output: bool,
}

/// Where the pins were drawn on the screen in this pass, see [`BlockPin::key`].
const PIN_RECTS: &str = "schematic pin rects";

impl BlockPin {
    fn single(info: PinInfo) -> Self {
        Self {
            info,
            collection: false,
            partial: false,
            anchor: None,
            key: None,
            fits: None,
        }
    }

    /// A pin of a feedback wire, whose position is kept under `anchor`.
    fn anchored(info: PinInfo, anchor: egui::Id) -> Self {
        Self {
            anchor: Some(anchor),
            ..Self::single(info)
        }
    }
}

impl egui_snarl::ui::SnarlPin for BlockPin {
    fn pin_rect(&self, x: f32, y0: f32, y1: f32, size: f32) -> egui::Rect {
        let size = if self.collection {
            egui::vec2(size * 0.7, size * 1.4)
        } else {
            egui::vec2(size, size)
        };
        egui::Rect::from_center_size(egui::pos2(x, (y0 + y1) * 0.5), size)
    }

    fn draw(
        mut self,
        snarl_style: &SnarlStyle,
        style: &egui::Style,
        rect: egui::Rect,
        painter: &egui::Painter,
    ) -> egui_snarl::ui::PinWireInfo {
        let ctx = painter.ctx();
        if let Some(key) = self.key.take() {
            let to_global = ctx.layer_transform_to_global(painter.layer_id());
            let screen = to_global.map_or(rect, |t| t * rect);
            let id = egui::Id::new(PIN_RECTS);
            ctx.data_mut(|d| {
                d.get_temp_mut_or_default::<Vec<(PinKey, egui::Rect)>>(id)
                    .push((key, screen))
            });
        }
        match self.fits.take() {
            Some(false) => {
                let mut faded = painter.clone();
                faded.multiply_opacity(0.2);
                self.draw_shape(snarl_style, style, rect, &faded)
            }
            Some(true) => {
                let ring = egui::Stroke::new(2.0, style.visuals.selection.bg_fill);
                let radius = rect.width().max(rect.height()) * 0.5 + 3.0;
                painter.circle_stroke(rect.center(), radius, ring);
                self.draw_shape(snarl_style, style, rect, painter)
            }
            None => self.draw_shape(snarl_style, style, rect, painter),
        }
    }
}

impl BlockPin {
    fn draw_shape(
        self,
        snarl_style: &SnarlStyle,
        style: &egui::Style,
        rect: egui::Rect,
        painter: &egui::Painter,
    ) -> egui_snarl::ui::PinWireInfo {
        if let Some(anchor) = self.anchor {
            let pass = painter.ctx().cumulative_pass_nr();
            let at = (painter.layer_id(), rect.center(), pass);
            painter.ctx().data_mut(|d| d.insert_temp(anchor, at));
            let mut info = self.info.draw(snarl_style, style, rect, painter);
            info.color = egui::Color32::TRANSPARENT;
            return info;
        }
        if !self.collection {
            return self.info.draw(snarl_style, style, rect, painter);
        }
        let fill = self.info.get_fill(snarl_style, style);
        let stroke = self.info.get_stroke(snarl_style, style);
        let hovered = (painter.ctx().pointer_hover_pos()).is_some_and(|p| rect.contains(p));
        let shown = if hovered || self.partial {
            fill.lerp_to_gamma(egui::Color32::WHITE, 0.45)
        } else {
            fill
        };
        painter.rect(
            rect,
            rect.width() * 0.5,
            shown,
            stroke,
            egui::StrokeKind::Middle,
        );
        if self.partial {
            // A dot: not all items of the category come through the link.
            painter.circle_filled(rect.center(), rect.width() * 0.18, fill);
        }
        egui_snarl::ui::PinWireInfo {
            color: self.info.wire_color.unwrap_or(fill),
            style: (self.info.wire_style).unwrap_or(egui_snarl::ui::WireStyle::Bezier5),
        }
    }
}

/// Where the pin of an input of a study is kept, see [`BlockPin::anchor`].
fn feedback_start(study: u64, item: &[String]) -> egui::Id {
    egui::Id::new(("study input pin", study, item))
}

/// Where the pin of a parameter a loop varies is kept.
fn feedback_end(target: u64, slot: &[String]) -> egui::Id {
    egui::Id::new(("loop parameter pin", target, slot))
}

/// Draws the feedback wires of loops dashed, between the pins drawn in this pass: from the
/// input of the study out to the right, below the blocks in its way back to the left and into
/// the parameter, so that the loop reads as one.
fn draw_feedback(
    ctx: &egui::Context,
    feedback: &[(u64, Vec<String>, u64, Vec<String>)],
    rects: &[egui::Rect],
    clip: egui::Rect,
) {
    let pass = ctx.cumulative_pass_nr();
    let at = |id: egui::Id| {
        ctx.data(|d| d.get_temp::<(egui::LayerId, egui::Pos2, u64)>(id))
            .filter(|(_, _, drawn)| *drawn == pass)
    };
    let color = wire_color(&[links::LOOP.to_owned()]);
    for (index, (study, item, target, slot)) in feedback.iter().enumerate() {
        let (Some((layer, from, _)), Some((_, to, _))) = (
            at(feedback_start(*study, item)),
            at(feedback_end(*target, slot)),
        ) else {
            continue;
        };
        // Several loops run below one another.
        let gap = 30.0 + 14.0 * index as f32;
        let (out, back) = (from.x + 24.0, to.x - 24.0);
        let below = feedback_level(
            from.y.max(to.y) + gap,
            back.min(out)..=back.max(out),
            gap,
            rects,
        );
        let corners = [
            from,
            egui::pos2(out, from.y),
            egui::pos2(out, below),
            egui::pos2(back, below),
            egui::pos2(back, to.y),
            to,
        ];
        // The painter of the layer clips to the screen as if it were not zoomed or panned.
        let mut painter = ctx.layer_painter(layer);
        painter.set_clip_rect(clip);
        painter.extend(egui::Shape::dashed_line(
            &rounded(&corners, 10.0),
            egui::Stroke::new(2.0, color),
            8.0,
            5.0,
        ));
    }
}

/// The height at which a feedback wire runs back between `x` from `start`: below the blocks
/// it would cross there, and only those, so that blocks elsewhere do not move it.
fn feedback_level(
    start: f32,
    x: std::ops::RangeInclusive<f32>,
    gap: f32,
    rects: &[egui::Rect],
) -> f32 {
    let mut level = start;
    // Going below one block can run into the next one under it.
    while let Some(bottom) = (rects.iter())
        .filter(|r| r.right() >= *x.start() && r.left() <= *x.end())
        .filter(|r| r.top() - gap * 0.5 <= level && level < r.bottom() + gap)
        .map(|r| r.bottom())
        .reduce(f32::max)
    {
        level = bottom + gap;
    }
    level
}

/// A path through `corners` with each inner corner rounded by up to `radius`.
fn rounded(corners: &[egui::Pos2], radius: f32) -> Vec<egui::Pos2> {
    let mut points = Vec::new();
    for (i, &corner) in corners.iter().enumerate() {
        let (Some(&before), Some(&after)) = (corners.get(i.wrapping_sub(1)), corners.get(i + 1))
        else {
            points.push(corner);
            continue;
        };
        let into = before - corner;
        let out = after - corner;
        let r = radius.min(into.length() * 0.5).min(out.length() * 0.5);
        let (a, b) = (
            corner + into.normalized() * r,
            corner + out.normalized() * r,
        );
        for step in 0..=6 {
            let t = step as f32 / 6.0;
            // A quadratic curve through the corner.
            points.push(a.lerp(corner, t).lerp(corner.lerp(b, t), t));
        }
    }
    points
}

/// The pin of a row that takes or gives no link: there, but not to be seen.
fn hidden_pin() -> PinInfo {
    PinInfo::circle()
        .with_fill(egui::Color32::TRANSPARENT)
        .with_stroke(egui::Stroke::NONE)
}

/// The context menu of an analysis' block.
/// How a block lies in the schematic, for its menu: the blocks its layout items work on,
/// itself or the selected blocks it is one of, whether it is compact and in a frame.
struct Layout {
    targets: Vec<u64>,
    compact: bool,
    framed: bool,
}

/// The items of a block's menu that change how it looks in the schematic, with their keys.
fn layout_menu(ui: &mut egui::Ui, layout: &Layout, actions: &mut Vec<Action>) {
    let mut entry = |ui: &mut egui::Ui, text: &str, key: &str, action: Action| {
        if ui.add(egui::Button::new(text).shortcut_text(key)).clicked() {
            actions.push(action);
            ui.close();
        }
    };
    let targets = layout.targets.clone();
    let compact = if layout.compact {
        "Show All Rows"
    } else {
        "Compact View"
    };
    entry(ui, compact, "H", Action::Compact(targets.clone()));
    entry(ui, "Frame Blocks", "Ctrl+J", Action::Frame(targets.clone()));
    if layout.framed {
        entry(ui, "Remove from Frame", "Alt+P", Action::Unframe(targets));
    }
}

fn node_menu(
    ui: &mut egui::Ui,
    id: u64,
    kind: Kind,
    steps: &[String],
    continues: Option<&str>,
    layout: &Layout,
    actions: &mut Vec<Action>,
) {
    let targets = layout.targets.clone();
    let mut entry = |ui: &mut egui::Ui, text: &str, key: &str, action: Action| {
        if ui.add(egui::Button::new(text).shortcut_text(key)).clicked() {
            actions.push(action);
            ui.close();
        }
    };
    if kind.is_source() {
        // Sources have no steps and are not solved.
        if kind == Kind::MeshExport {
            entry(ui, "Export Now", "", Action::ExportNow(id));
            entry(ui, "Choose File …", "", Action::Open(id, None));
        } else if kind == Kind::File {
            entry(ui, "Choose File …", "", Action::Open(id, None));
            entry(ui, "Read Again", "", Action::ReadFile(id));
        } else if kind == Kind::Campbell {
            entry(ui, "Open Diagram …", "", Action::Open(id, None));
        } else if !matches!(kind, Kind::RefineMesh | Kind::DeformedMesh) {
            entry(ui, "Edit …", "", Action::Open(id, None));
        }
        if matches!(kind, Kind::Mesh | Kind::RefineMesh) {
            entry(ui, "Generate Mesh", "", Action::Generate(id));
        }
        if kind == Kind::CadOperation {
            entry(ui, "Update Geometry", "", Action::Generate(id));
        }
        if kind == Kind::Study {
            entry(ui, "Run", "", Action::RunStudy(id));
            entry(ui, "Reset", "", Action::ResetStudy(id));
        }
        ui.separator();
        layout_menu(ui, layout, actions);
        ui.separator();
        edit_entries(ui, id, &targets, actions);
        return;
    }
    entry(ui, "Open", "", Action::Open(id, None));
    entry(ui, "Open in New Window", "", Action::OpenInWindow(id));
    entry(ui, "Solve", "", Action::Solve(id));
    ui.separator();
    let split = steps.len() > 1;
    ui.add_enabled_ui(split, |ui| {
        ui.menu_button("Split After", |ui| {
            for step in &steps[..steps.len().saturating_sub(1)] {
                entry(ui, step, "", Action::Split(id, step.clone()));
            }
        })
        .response
        .on_hover_text(
            "The steps after the one picked go to a new analysis that continues this one \
             from the state after it.",
        );
    });
    if let Some(source) = continues {
        entry(ui, &format!("Merge into {source}"), "", Action::Merge(id));
    }
    ui.separator();
    layout_menu(ui, layout, actions);
    ui.separator();
    edit_entries(ui, id, &targets, actions);
}

/// Rename, Duplicate and Delete in a block's menu; the last two work on `targets`.
fn edit_entries(ui: &mut egui::Ui, id: u64, targets: &[u64], actions: &mut Vec<Action>) {
    let mut entry = |ui: &mut egui::Ui, text: &str, key: &str, action: Action| {
        if ui.add(egui::Button::new(text).shortcut_text(key)).clicked() {
            actions.push(action);
            ui.close();
        }
    };
    entry(ui, "Rename …", "F2", Action::Rename(id));
    entry(
        ui,
        "Duplicate",
        "Shift+D",
        Action::DuplicateAll(targets.to_vec()),
    );
    entry(ui, "Delete", "X", Action::DeleteAll(targets.to_vec()));
}

/// Spacing of the dots of the schematic's background, at true size.
const DOT_SPACING: f32 = 24.0;

/// The least gap of the background dots on the screen, in points.
const MIN_DOT_GAP: f32 = 16.0;

/// Spacing and radius of the background dots in the schematic at a zoom of `scale`.
///
/// Zoomed out, every second row and column of dots goes, as often as it takes to keep them
/// `MIN_DOT_GAP` apart on the screen, and the dots keep their size on the screen. At a fixed
/// spacing a schematic zoomed out to a tenth drew a hundred times as many dots, over 100 000
/// in each frame, which made panning and dragging stutter.
fn dot_grid(scale: f32) -> (f32, f32) {
    let scale = scale.max(MIN_SCALE);
    let mut spacing = DOT_SPACING;
    while spacing * scale < MIN_DOT_GAP {
        spacing *= 2.0;
    }
    (spacing, 1.0 / scale.min(1.0))
}

/// The zoom of the schematic: from a tenth of its true size to four times it; Show All
/// magnifies no more than to its true size.
const MAX_SCALE: f32 = 4.0;
const MIN_SCALE: f32 = 0.1;

fn schematic_style() -> SnarlStyle {
    let border = egui::Stroke::new(1.0, crate::style::border());
    SnarlStyle {
        collapsible: Some(false),
        // A double-click into empty space shows the blocks to add instead.
        centering: Some(false),
        // Blocks with rounded corners and a tinted header, like the nodes of a node editor.
        node_frame: Some(
            egui::Frame::new()
                .fill(crate::style::window())
                .stroke(border)
                .corner_radius(6.0)
                .inner_margin(egui::Margin::symmetric(5, 3)),
        ),
        // Pins on the edge of the block, like the sockets of a Blender node. Inside the
        // frame, egui_snarl 0.12 widens a block by the pin size for every row: the space it
        // keeps for an output pin pushes the next row's right edge out.
        pin_placement: Some(egui_snarl::ui::PinPlacement::Edge),
        header_frame: Some(
            egui::Frame::new()
                .fill(crate::style::node_header())
                .corner_radius(egui::CornerRadius {
                    nw: 6,
                    ne: 6,
                    sw: 0,
                    se: 0,
                })
                .inner_margin(egui::Margin::symmetric(6, 4)),
        ),
        bg_frame: Some(egui::Frame::new().fill(crate::style::control())),
        bg_pattern: Some(egui_snarl::ui::BackgroundPattern::NoPattern),
        pin_size: Some(9.0),
        pin_stroke: Some(egui::Stroke::new(1.0, crate::style::pin_edge())),
        wire_width: Some(2.0),
        max_scale: Some(MAX_SCALE),
        min_scale: Some(MIN_SCALE),
        // A row per pin, all of the same height, so that the rows line up with their pins.
        node_layout: Some(egui_snarl::ui::NodeLayout {
            kind: egui_snarl::ui::NodeLayoutKind::Coil,
            min_pin_row_height: ROW_HEIGHT,
            equal_pin_row_heights: true,
        }),
        ..SnarlStyle::new()
    }
}

/// Tooltip of Add Materials.
const ANALYSIS_TIP: &str = "Adds an analysis: a model of its own with geometry, mesh, setup \
     and steps, solved with the built-in solver. Double-click its block to open it in Geometry, FE Model and Results.";

const PARAMETERS_TIP: &str = "Adds a block of named numbers: drag the pin of one to a \
                              parameter of another block, such as a length of a mesh block, \
                              which then takes its value.";
const MEASURE_TIP: &str = "Adds a block that computes a key figure from the results of a \
                           step linked to it, such as the largest von Mises stress, the \
                           displacement of a node or an eigenfrequency.";
const STUDY_TIP: &str = "Adds a parameter study: a table of design points, each a set of \
                         values it gives to exposed parameters of analyses or numbers of \
                         other blocks, solved in turn, with the values of Measure blocks it \
                         reads back.";
const MATERIALS_TIP: &str = "Adds a block of materials kept in the project: analyses link \
     them one by one or all at once, so that they are defined in one place.";

/// Tooltip of Add Cross-Sections.
const CROSS_SECTIONS_TIP: &str = "Adds a block of beam cross-sections kept in the project: \
     analyses link them one by one or all at once, and their dimensions can be exposed as \
     parameters, so that a parameter study varies the profile of every analysis that links it.";

/// Tooltip of Undo.
const UNDO_TIP: &str = "Undoes the last change of the project: blocks added, deleted, renamed or \
                        moved, links, and what the blocks hold. Changes inside an analysis are \
                        undone in its window.";
/// The project window's solution monitor: which analysis' output it shows, and the text
/// the output is filtered by.
#[derive(Default)]
struct Monitor {
    /// The output pane shows the Analyses tab instead of the messages.
    shown: bool,
    selected: Option<u64>,
    filter: String,
}

impl Monitor {
    /// A solve of `id` starts: the monitor shows its output.
    fn show(&mut self, id: u64) {
        self.shown = true;
        self.selected = Some(id);
    }
}

/// A line of the solution monitor's list.
struct MonitorRow {
    id: u64,
    name: String,
    state: String,
    seconds: Option<f32>,
    progress: Option<String>,
    active: bool,
}

/// The recent project files stored at the last exit that still exist.
fn stored_recent(storage: &dyn eframe::Storage) -> Vec<PathBuf> {
    let mut recent = eframe::get_value::<Vec<PathBuf>>(storage, RECENT_KEY).unwrap_or_default();
    recent.retain(|path| path.is_file());
    recent.truncate(RECENT_MAX);
    recent
}

/// `recent` with `path` first, once, and at most [`RECENT_MAX`] entries.
fn with_recent(recent: &[PathBuf], path: &Path) -> Vec<PathBuf> {
    let same = |other: &Path| other == path;
    let mut list = vec![path.to_path_buf()];
    list.extend(recent.iter().filter(|p| !same(p)).cloned());
    list.truncate(RECENT_MAX);
    list
}

/// Asks for a project file to save to, proposing the open one's name with `suffix`.
fn pick_project_file(current: Option<&Path>, title: &str, suffix: &str) -> Option<PathBuf> {
    let stem = (current.and_then(Path::file_stem))
        .map_or_else(|| "Project".into(), |s| s.to_string_lossy().into_owned());
    let mut dialog = rfd::FileDialog::new()
        .set_title(title)
        .add_filter("Nodalis project (*.ndl)", &[PROJECT_EXTENSION])
        .set_file_name(format!("{stem}{suffix}.{PROJECT_EXTENSION}"));
    if let Some(dir) = current.and_then(Path::parent) {
        dialog = dialog.set_directory(dir);
    }
    let mut path = dialog.save_file()?;
    if path.extension().is_none() {
        path.set_extension(PROJECT_EXTENSION);
    }
    Some(path)
}

/// Prefixes an output message with the local time, `HH:MM:SS`.
fn timestamped(message: String) -> String {
    format!("{} {message}", chrono::Local::now().format("%H:%M:%S"))
}

/// The name a new block starts with: blocks may share a name, they are told apart by their
/// ids, so it is always `base`.
fn block_name(base: &str) -> String {
    base.to_owned()
}

#[cfg(test)]
mod tests {
    /// Zoomed out, the background has about as many dots as at true size, not a hundred
    /// times as many, and they stay as large on the screen.
    #[test]
    fn background_dots_thin_out_when_zoomed_out() {
        use super::{DOT_SPACING, MAX_SCALE, MIN_DOT_GAP, MIN_SCALE, dot_grid};
        assert_eq!(dot_grid(1.0), (DOT_SPACING, 1.0));
        let mut scale = MIN_SCALE;
        while scale <= MAX_SCALE {
            let (spacing, radius) = dot_grid(scale);
            let gap = spacing * scale;
            assert!(gap >= MIN_DOT_GAP, "{scale}: {gap}");
            assert!(
                gap < 2.0 * MIN_DOT_GAP || spacing == DOT_SPACING,
                "{scale}: {gap}"
            );
            assert!(radius * scale >= 1.0 - 1e-4, "{scale}: {radius}");
            scale *= 1.1;
        }
    }

    #[test]
    fn feedback_wires_go_below_only_the_blocks_in_their_way() {
        use super::feedback_level;
        let block =
            |x: f32, y: f32| egui::Rect::from_min_size(egui::pos2(x, y), egui::vec2(100.0, 80.0));
        // The study and the analysis it varies, side by side.
        let mut rects = vec![block(0.0, 0.0), block(200.0, 0.0)];
        assert_eq!(feedback_level(70.0, -24.0..=324.0, 30.0, &rects), 110.0);
        // A block far below, or off to the side, leaves the wire where it is.
        rects.push(block(100.0, 600.0));
        rects.push(block(600.0, 0.0));
        rects.push(block(-400.0, 200.0));
        assert_eq!(feedback_level(70.0, -24.0..=324.0, 30.0, &rects), 110.0);
        // A block in its way, and one right under that, take it further down.
        rects.push(block(100.0, 100.0));
        rects.push(block(150.0, 200.0));
        assert_eq!(feedback_level(70.0, -24.0..=324.0, 30.0, &rects), 310.0);
    }

    #[test]
    fn rows_open_the_analysis_in_their_workspace() {
        use super::{Show, TreeView, workspace_of};
        let view = |view| Some(Show { view, step: None });
        let path = |p: &[&str]| p.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        assert_eq!(workspace_of(&path(&["Geometry"])), view(TreeView::Geometry));
        assert_eq!(workspace_of(&path(&["Mesh"])), view(TreeView::Geometry));
        assert_eq!(
            workspace_of(&path(&["Setup", "Steps"])),
            view(TreeView::FeModel)
        );
        assert_eq!(workspace_of(&path(&["Results"])), view(TreeView::Results));
        assert_eq!(
            workspace_of(&path(&["Results", "Step-1"])),
            Some(Show {
                view: TreeView::FeModel,
                step: Some("Step-1".into())
            })
        );
        assert_eq!(
            workspace_of(&path(&["Setup", "Steps", "Step-2", "Temperatures"])),
            Some(Show {
                view: TreeView::FeModel,
                step: Some("Step-2".into())
            })
        );
        assert_eq!(workspace_of(&path(&["State"])), None);
    }

    use super::*;
    use cad::cad_rows;
    use std::collections::HashMap;

    #[derive(Default)]
    struct MemoryStorage(HashMap<String, String>);

    impl eframe::Storage for MemoryStorage {
        fn get_string(&self, key: &str) -> Option<String> {
            self.0.get(key).cloned()
        }

        fn set_string(&mut self, key: &str, value: String) {
            self.0.insert(key.to_owned(), value);
        }

        fn remove_string(&mut self, key: &str) {
            self.0.remove(key);
        }

        fn flush(&mut self) {}
    }

    #[test]
    fn a_reader_that_fails_ends_the_opening() {
        // Still reading: nothing yet.
        let (sender, receiver) = std::sync::mpsc::channel::<Opened>();
        assert!(received(&receiver).is_none());
        // A reader thread that panicked drops its sender without sending; the project
        // window used to wait for it forever.
        let reader = std::thread::spawn(move || {
            let _sender = sender;
            panic!("broken file");
        });
        assert!(reader.join().is_err());
        assert!(matches!(received(&receiver), Some(Err(_))));
        // A panic caught on the reader thread carries its message.
        let panic = std::panic::catch_unwind(|| panic!("index out of bounds")).unwrap_err();
        assert_eq!(
            panic_message(panic.as_ref()),
            "reading the file failed: index out of bounds"
        );
        let file = std::env::temp_dir().join("nodalis-missing-project.ndl");
        assert!(read_opened(&file).is_err());
    }

    #[test]
    fn a_materials_block_keeps_its_materials_and_gives_them_as_setup() {
        let set = file::MaterialSet {
            id: 4,
            name: "Materials".into(),
            position: [5.0, 6.0],
            units: plx_model::UnitSystem::MKgSC,
            materials: vec![plx_model::Material {
                name: "Steel".into(),
                density: Some(7850.0),
                ..Default::default()
            }],
            exposed: Vec::new(),
        };
        let mut node = Node::read(ProjectNode::Materials(set.clone()));
        assert_eq!(node.kind(), Kind::Materials);
        assert!(node.status().is_none(), "a source is not solved");
        // Analyses link its materials like those of another analysis.
        let data = node.link_data().unwrap();
        let path = vec![links::SETUP.to_owned(), "Materials".to_owned()];
        assert!(links::exists(data, &path));
        let mut steel = path.clone();
        steel.push("Steel".into());
        assert!(links::exists(data, &steel));
        assert!(links::compatible(&steel, &steel) && links::compatible(&path, &path));
        let mut target = plx_model::FeModel::default();
        links::apply_setup(&mut target, data.fe, &steel);
        assert_eq!(target.materials, set.materials);
        let rows = collection_rows(Collection::Materials, data.fe, &node.expanded);
        assert_eq!(rows.len(), 2);
        assert!(rows.iter().all(|r| r.output && !r.input));
        assert_eq!(rows[1].path, steel);
        match node.written([5.0, 6.0]) {
            Some(ProjectNode::Materials(written)) => assert_eq!(written, set),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_cross_sections_block_gives_its_profiles_with_their_exposed_dimensions() {
        use plx_model::{BeamProfile, CrossSection, Field, ItemRef, ValuePath};
        let set = file::CrossSectionSet {
            id: 7,
            name: "Cross-Sections".into(),
            position: [1.0, 2.0],
            units: plx_model::UnitSystem::MmTonSC,
            cross_sections: vec![CrossSection {
                id: plx_model::Id::new(),
                name: "Girder".into(),
                profile: BeamProfile::Rect { a: 0.02, b: 0.04 },
            }],
            exposed: Vec::new(),
        };
        let mut node = Node::read(ProjectNode::CrossSections(set.clone()));
        assert_eq!(node.kind(), Kind::CrossSections);
        assert!(node.status().is_none(), "a source is not solved");
        // Its height is a parameter of the block, which a study sets.
        let height = ValuePath::new(ItemRef::CrossSection("Girder".into()), Field::BeamB);
        let fe = node.exposing_fe_mut().unwrap();
        let key = fe.expose(height.clone());
        assert!(fe.set_value(&height, 0.05));
        assert!(node.exposing_fe().unwrap().exposed(key).is_some());
        // Analyses link the profile like the materials of a materials block.
        let data = node.link_data().unwrap();
        let path = vec![
            links::SETUP.to_owned(),
            crate::tree::CROSS_SECTIONS.to_owned(),
        ];
        assert!(links::exists(data, &path));
        let mut girder = path.clone();
        girder.push("Girder".into());
        let mut target = plx_model::FeModel::default();
        links::apply_setup(&mut target, data.fe, &girder);
        assert_eq!(
            target.cross_sections[0].profile,
            BeamProfile::Rect { a: 0.02, b: 0.05 }
        );
        let rows = collection_rows(Collection::CrossSections, data.fe, &node.expanded);
        assert_eq!(rows.len(), 2, "open at first");
        assert_eq!(rows[0].text, "Cross-Sections (1)");
        assert_eq!(rows[1].path, girder);
        match node.written([1.0, 2.0]) {
            Some(ProjectNode::CrossSections(written)) => {
                assert_eq!(written.exposed.len(), 1);
                assert_eq!(written.cross_sections, target.cross_sections);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_mesh_block_is_out_of_date_once_its_parameters_change() {
        let block = file::MeshBlock {
            id: 2,
            name: "Box".into(),
            position: [0.0; 2],
            space: ModelSpace::ThreeD,
            units: plx_model::UnitSystem::MmTonSC,
            primitive: MeshPrimitive::default_box(),
            mesh: Default::default(),
            generated: None,
        };
        let mut node = Node::read(ProjectNode::Mesh(Box::new(block)));
        assert_eq!(node.kind(), Kind::Mesh);
        assert_eq!(node.mesh_state(), CellState::Missing);
        assert_eq!(node.generate(), Ok(40));
        assert_eq!(node.mesh_state(), CellState::UpToDate);
        let data = node.link_data().unwrap();
        let mesh = vec![links::MESH.to_owned()];
        assert!(links::exists(data, &mesh));
        assert_eq!(links::MeshCopy::of(data).mesh.node_sets.len(), 6);
        let rows = mesh_rows(&node, &[]);
        assert!(rows[0].output && !rows[0].input);
        // Its parameters are rows below the mesh, the element type last; the numbers take
        // value links, the element type does not.
        let params: Vec<String> = (rows[1..].iter()).map(|row| row.path.join("/")).collect();
        assert_eq!(params[0], "Parameters/x");
        assert_eq!(params[6], "Parameters/nx");
        assert_eq!(params.last().unwrap(), "Parameters/element");
        // The factor is a block of its own.
        assert!(!params.iter().any(|p| p.ends_with(REFINEMENT)));
        let (numbers, element) = rows[1..].split_at(rows.len() - 2);
        assert!(
            numbers
                .iter()
                .all(|r| r.widget.is_some() && r.input && !r.output)
        );
        assert!(!element[0].input && !element[0].output);
        // What the rows edit: the count of elements along X and the element type.
        let primitive = &mut node.as_mesh_mut().unwrap().primitive;
        assert!(primitive.set("nx", plx_model::ParamValue::Count(20)));
        if let MeshPrimitive::Box { element, .. } = primitive {
            *element = "C3D8I".into();
        }
        assert_eq!(node.mesh_state(), CellState::OutOfDate);
        assert!(mesh_rows(&node, &[])[0].tip.contains("out of date"));
        match node.written([1.0, 2.0]) {
            Some(ProjectNode::Mesh(written)) => {
                assert_eq!(written.mesh.element_count(), 40);
                assert_eq!(written.primitive, node.as_mesh().unwrap().primitive.clone());
                assert_eq!(written.position, [1.0, 2.0]);
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(node.generate(), Ok(80));
        assert_eq!(node.mesh_state(), CellState::UpToDate);
    }

    #[test]
    fn a_cad_operation_block_makes_its_geometry_and_fails_visibly() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/platte_mit_loch.step");
        let input = match plx_mesher::import_cad(&path, plx_model::UnitSystem::MmTonSC) {
            Ok(import) => import.geometry,
            Err(error) if std::env::var_os("NODALIS_REQUIRE_GMSH").is_none() => {
                eprintln!("Gmsh not available, test skipped: {error}");
                return;
            }
            Err(error) => panic!("{error}"),
        };
        // A straight edge of the plate.
        let edge = (input.signatures.iter())
            .find(|s| matches!(s.entity, plx_mesh::CadEntity::Edge(_)) && s.kind == "Line")
            .unwrap()
            .clone();
        let block = file::CadOperationBlock {
            id: 3,
            name: "Fillet".into(),
            position: [0.0; 2],
            units: plx_model::UnitSystem::MmTonSC,
            operation: plx_model::CadOperation {
                entities: vec![edge],
                value: 0.5e-3,
                ..Default::default()
            },
            result: None,
            made_from: None,
        };
        let mut node = Node::read(ProjectNode::CadOperation(Box::new(block)));
        assert_eq!(node.kind(), Kind::CadOperation);
        let cad = node.as_cad_mut().unwrap();
        assert_eq!(cad.state(), CellState::Missing);
        assert!(cad.link_data().is_none() || cad.model.geometry.is_none());
        cad.input = Some(input.clone());
        assert_eq!(cad.state(), CellState::OutOfDate);
        assert!(cad.generate().is_ok());
        assert_eq!(cad.state(), CellState::UpToDate);
        let made = cad.model.geometry.clone().unwrap();
        assert_ne!(made.brep, input.brep);
        assert_eq!(made.part_names, input.part_names);
        assert!(cad.link_data().is_some_and(|d| d.geometry.is_some()));
        // Rows: the geometry in and out, what it does, and the radius with an input pin.
        let rows = cad_rows(&node, &[]);
        assert!(rows[0].input && rows[0].output);
        assert_eq!(rows[1].text, "Fillet: 1 edge");
        assert_eq!(rows[2].path, [links::PARAMETERS, CAD_LENGTH]);
        assert!(rows[2].input);
        // A radius far larger than the plate: the block fails, says why, and gives nothing
        // on, so that what comes after it is skipped.
        let cad = node.as_cad_mut().unwrap();
        cad.operation.value = 1.0;
        assert_eq!(cad.state(), CellState::OutOfDate);
        assert!(cad.generate().is_err());
        assert_eq!(cad.state(), CellState::Failed);
        assert!(cad.error().is_some());
        assert!(cad.link_data().is_none());
        assert!(cad_rows(&node, &[])[0].tip.contains("failed"));
        // Back to a radius that works: out of date again, then made.
        let cad = node.as_cad_mut().unwrap();
        cad.operation.value = 0.5e-3;
        assert_eq!(cad.state(), CellState::UpToDate);
        // What it keeps in the project file.
        match node.written([0.0; 2]) {
            Some(ProjectNode::CadOperation(written)) => {
                assert!(written.result.is_some() && written.made_from.is_some());
                assert_eq!(written.operation.value, 0.5e-3);
            }
            other => panic!("{other:?}"),
        }
    }

    /// A box mesh block, made, with the id 2.
    fn made_box() -> Node {
        let mut node = Node::mesh_block(
            2,
            "Box".into(),
            ModelSpace::ThreeD,
            plx_model::UnitSystem::MmTonSC,
            MeshPrimitive::default_box(),
            Default::default(),
            None,
        );
        assert_eq!(node.generate(), Ok(40));
        node
    }

    fn mesh_link(source: u64, target: u64) -> Link {
        Link {
            source,
            target,
            item: vec![links::MESH.to_owned()],
            ..Default::default()
        }
    }

    #[test]
    fn a_refine_mesh_block_refines_the_divisions_of_a_mesh_block() {
        let mut nodes = vec![
            made_box(),
            Node::refine_block(4, "Refine Mesh".into(), 1.0, Default::default(), None),
            Node::analysis(1, "Analysis-1".into(), ProjectAnalysis::default()),
        ];
        assert_eq!(nodes[1].kind(), Kind::RefineMesh);
        // Nothing linked: nothing to give.
        assert_eq!(nodes[1].mesh_state(), CellState::Missing);
        let rows = refine_rows(&nodes[1], &[]);
        assert!(rows[0].input && !rows[0].output);
        let links = [mesh_link(2, 4), mesh_link(4, 1)];
        // Linked, it makes its mesh at once, so that it can be linked on.
        take_refine_input(&mut nodes, &links, 4);
        assert_eq!(nodes[1].mesh_state(), CellState::UpToDate);
        assert_eq!(nodes[1].link_data().unwrap().mesh.element_count(), 40);
        assert_eq!(nodes[1].space(), Some(ModelSpace::ThreeD));
        // The mesh row takes and gives a mesh, the factor takes numbers.
        let rows = refine_rows(&nodes[1], &[]);
        assert!(rows[0].input && rows[0].output);
        assert_eq!(rows[1].path, [PARAMETERS, REFINEMENT]);
        assert!(rows[1].input && matches!(rows[1].widget, Some(RowWidget::Refinement)));
        assert_eq!(
            value_kind(&nodes, 4, &rows[1].path),
            Some(ValueKind::Factor)
        );
        // A factor of 1.5 makes 15 x 3 x 3 elements; the analysis after it is out of date
        // until the block makes them.
        nodes[1].set_refinement(1.5);
        assert_eq!(nodes[1].mesh_state(), CellState::OutOfDate);
        assert_eq!(stale_meshes(&nodes, &links, 1), ["Refine Mesh"]);
        let (made, lines) = generate_stale_meshes(&mut nodes, None);
        assert!(made, "{lines:?}");
        assert_eq!(nodes[1].link_data().unwrap().mesh.element_count(), 135);
        assert!(stale_meshes(&nodes, &links, 1).is_empty());
        // The mesh block keeps its own divisions.
        assert_eq!(nodes[0].link_data().unwrap().mesh.element_count(), 40);
        // New divisions of the mesh block are refined too.
        let primitive = &mut nodes[0].as_mesh_mut().unwrap().primitive;
        assert!(primitive.set("nx", plx_model::ParamValue::Count(20)));
        take_refine_input(&mut nodes, &links, 4);
        assert_eq!(nodes[1].mesh_state(), CellState::OutOfDate);
        generate_stale_meshes(&mut nodes, None);
        assert_eq!(
            nodes[1].link_data().unwrap().mesh.element_count(),
            30 * 3 * 3
        );
        // It keeps its mesh in the project file, up to date when read back.
        let written = nodes[1].written([1.0, 2.0]).unwrap();
        let mut read = vec![nodes.remove(0), Node::read(written)];
        assert_eq!(read[1].mesh_state(), CellState::Missing);
        take_refine_input(&mut read, &links, 4);
        assert_eq!(read[1].refinement(), Some(1.5));
        assert_eq!(read[1].mesh_state(), CellState::UpToDate);
        // A second block refines the refined divisions: 1.5 * 2 = 3 times as fine.
        read.push(Node::refine_block(
            5,
            "Finer".into(),
            2.0,
            Default::default(),
            None,
        ));
        let chain = [mesh_link(2, 4), mesh_link(4, 5)];
        take_refine_input(&mut read, &chain, 5);
        assert_eq!(
            read[2].link_data().unwrap().mesh.element_count(),
            60 * 6 * 6
        );
        // Unlinked, it gives nothing.
        take_refine_input(&mut read, &[], 4);
        assert_eq!(read[1].mesh_state(), CellState::Missing);
        assert_eq!(read[1].link_data().unwrap().mesh.element_count(), 0);
    }

    #[test]
    fn a_refine_mesh_block_divides_the_element_sizes_of_geometry() {
        let geometry = plx_model::Geometry {
            source: "block.step".into(),
            brep: "shapes".into(),
            meshing: Default::default(),
            mesh_items: Vec::new(),
            part_names: Vec::new(),
            refinement: 1.0,
            signatures: Vec::new(),
        };
        let mesh = MeshPrimitive::default_box().generate(ModelSpace::ThreeD, "Solid-1");
        let space = ModelSpace::ThreeD;
        let mesh_inputs = BTreeMap::from([(
            "SOLID-1".to_owned(),
            plx_model::fingerprint::mesh_input(&geometry, space, "SOLID-1"),
        )]);
        let mut model = plx_model::FeModel::default();
        model.properties.space = space;
        let data = ProjectAnalysis {
            id: 3,
            name: "Analysis".into(),
            geometry: Some(geometry),
            mesh,
            model,
            mesh_inputs,
            ..Default::default()
        };
        let mut nodes = vec![
            Node::read(ProjectNode::Analysis(Box::new(data.clone()))),
            Node::refine_block(4, "Refine Mesh".into(), 2.0, Default::default(), None),
            Node::read(ProjectNode::Analysis(Box::new(ProjectAnalysis {
                id: 5,
                name: "Fine".into(),
                ..data
            }))),
        ];
        // An analysis has no Refine row of its own.
        let rows = block_rows(
            nodes[0].status().unwrap(),
            None,
            &BTreeSet::new(),
            &[],
            &[],
            None,
        );
        assert!(!rows.iter().any(|r| r.path.contains(&REFINEMENT.to_owned())));
        assert_eq!(nodes[0].refinement(), None);
        let links = [mesh_link(3, 4), mesh_link(4, 5)];
        assert_eq!(
            refusal(&nodes, &[], 3, &links[0].item, 4, &links[0].item),
            None
        );
        take_refine_input(&mut nodes, &links, 4);
        // It gives the geometry with the sizes divided by its factor; the analysis the mesh
        // goes to meshes it, so its mesh is out of date there.
        assert_eq!(nodes[1].mesh_state(), CellState::UpToDate);
        let given = nodes[1].link_data().unwrap();
        assert_eq!(given.geometry.unwrap().refinement, 2.0);
        let print = links::fingerprint(given, &links[1].item);
        let copy = links::MeshCopy::of(given);
        let mut analysis = nodes[2].closed_model_mut().unwrap();
        analysis.take_mesh(copy);
        nodes[2].refresh_closed();
        assert_eq!(nodes[2].status().unwrap().mesh, CellState::OutOfDate);
        // A new factor gives a new mesh to the links.
        nodes[1].set_refinement(3.0);
        take_refine_input(&mut nodes, &links, 4);
        let given = nodes[1].link_data().unwrap();
        assert_eq!(given.geometry.unwrap().refinement, 3.0);
        assert_ne!(links::fingerprint(given, &links[1].item), print);
        // It cannot go to a mesh export block: the analysis makes the mesh.
        nodes.push(Node::mesh_export(6, "Export".into(), String::new(), None));
        let why = refusal(&nodes, &links, 4, &links[0].item, 6, &links[0].item).unwrap();
        assert!(why.contains("refines geometry"), "{why}");
        // A mesh without geometry or parameters cannot be made again.
        let read = ProjectAnalysis {
            id: 7,
            name: "Read".into(),
            mesh: MeshPrimitive::default_box().generate(space, "Solid-1"),
            ..Default::default()
        };
        nodes.push(Node::read(ProjectNode::Analysis(Box::new(read))));
        let why = refusal(&nodes, &[], 7, &links[0].item, 4, &links[0].item).unwrap();
        assert!(why.contains("neither"), "{why}");
    }

    #[test]
    fn a_measure_takes_the_regions_of_the_model_on_each_new_mesh() {
        let mut fe = plx_model::FeModel::default();
        let mut step = plx_model::Step::new_static("Step-1");
        step.boundary_conditions.push(plx_model::BoundaryCondition {
            id: plx_model::Id::new(),
            distribution: None,
            name: "Fixed".into(),
            active: true,
            visible: true,
            region: plx_model::Region::Geometry(vec![plx_mesh::CadEntity::Face(2)]),
            kind: plx_model::BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        fe.steps.push(step);
        for refinement in [1.0, 2.0] {
            let mesh = (MeshPrimitive::default_box().refined(refinement))
                .generate(ModelSpace::ThreeD, "Box");
            let data = links::Data {
                geometry: None,
                mesh: &mesh,
                fe: &fe,
                mesh_inputs: &BTreeMap::new(),
            };
            let regions = measure_regions(&data);
            // The face picked on the box is its side X+ on the coarse and the fine mesh.
            assert_eq!(regions["Fixed (boundary condition)"], mesh.node_sets["X+"]);
            assert!(regions.contains_key("X-"));
        }
    }

    #[test]
    fn a_parameters_block_reads_back_and_gives_its_values() {
        let set = file::ParameterSet {
            id: 4,
            name: "Parameters".into(),
            position: [3.0, 4.0],
            units: plx_model::UnitSystem::MmTonSC,
            parameters: vec![
                Parameter::new("Length", ValueKind::LENGTH, 0.3),
                Parameter::new("Cells", ValueKind::Count, 12.0),
            ],
        };
        let node = Node::read(ProjectNode::Parameters(set.clone()));
        assert_eq!(node.kind(), Kind::Parameters);
        let rows = parameter_rows(&node, &[]);
        assert_eq!(rows[1].path, ["Value", "Cells"]);
        // A parameter gives its value and takes one from another parameter.
        assert!(rows.iter().all(|r| r.output && r.input));
        let mut node = node;
        match node.written([3.0, 4.0]) {
            Some(ProjectNode::Parameters(written)) => assert_eq!(written, set),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_note_reads_back_and_its_copy_keeps_the_text() {
        let note = file::Note {
            id: 7,
            name: "Note".into(),
            position: [1.0, 2.0],
            text: "Check the bolt preload".into(),
        };
        let mut node = Node::read(ProjectNode::Note(note.clone()));
        assert_eq!(node.kind(), Kind::Note);
        assert!(node.link_data().is_none());
        match node.written([1.0, 2.0]) {
            Some(ProjectNode::Note(written)) => assert_eq!(written, note),
            other => panic!("{other:?}"),
        }
        let copy = node.copied(8, "Copy of Note".into()).unwrap();
        assert!(matches!(&copy.block, Block::Note(text) if text == &note.text));
    }

    #[test]
    fn math_and_switch_blocks_compute_and_take_numbers_of_their_kind() {
        let parameters = Node::read(ProjectNode::Parameters(file::ParameterSet {
            id: 1,
            name: "Parameters".into(),
            parameters: vec![
                Parameter::new("Length", ValueKind::LENGTH, 0.1),
                Parameter::new("Cells", ValueKind::Count, 4.0),
            ],
            ..Default::default()
        }));
        let math = Node::read(ProjectNode::Math(file::MathBlock {
            id: 2,
            name: "Math".into(),
            operation: MathOp::Multiply,
            values: vec![0.1, 2.0],
            ..Default::default()
        }));
        let switch = Node::read(ProjectNode::Switch(file::SwitchBlock {
            id: 3,
            name: "Switch".into(),
            index: 2.0,
            choices: vec![5.0, 7.0],
            ..Default::default()
        }));
        let (number, is_switch) = math.as_number().unwrap();
        assert!(!is_switch);
        assert_eq!(number.result(false), Some(0.2));
        assert_eq!(switch.as_number().unwrap().0.result(true), Some(7.0));
        let rows = numbers::number_rows(&math, &[]);
        assert_eq!(rows.last().unwrap().path, [links::VALUE, RESULT]);
        assert!(rows.last().unwrap().output);
        let mut nodes = vec![parameters, math, switch];
        let value = |name: &str| vec![links::VALUE.to_owned(), name.to_owned()];
        let input = |key: &str| vec![links::PARAMETERS.to_owned(), key.to_owned()];
        // Without links, A and B of a product take numbers of any kind.
        assert_eq!(
            refusal(&nodes, &[], 1, &value("Length"), 2, &input("a")),
            None
        );
        assert_eq!(
            refusal(&nodes, &[], 1, &value("Length"), 2, &input("b")),
            None
        );
        // The index of a switch is a count; its result goes on to a block of any kind.
        assert_eq!(
            refusal(&nodes, &[], 1, &value("Cells"), 3, &input(INDEX)),
            None
        );
        assert_eq!(
            refusal(&nodes, &[], 3, &value(RESULT), 2, &input("a")),
            None
        );
        // A sum takes only numbers of the kind its links give it.
        if let Some((number, _)) = nodes[1].as_number_mut() {
            number.operation = MathOp::Add;
        }
        let mut links = vec![Link {
            source: 1,
            target: 2,
            item: value("Cells"),
            slot: input("a"),
            ..Default::default()
        }];
        numbers::refresh_kinds(&mut nodes, &mut links);
        assert!(refusal(&nodes, &links, 1, &value("Length"), 2, &input("b")).is_some());
        assert_eq!(
            value_kind(&nodes, 2, &value(RESULT)),
            Some(ValueKind::Count)
        );
        assert_eq!(parameter_label(&nodes, 3, &input("2")), "Choice 2");
        // A count is kept whole.
        let (number, _) = nodes[2].as_number_mut().unwrap();
        number.kind = Some(ValueKind::Count);
        assert!(number.set(true, "1", 2.4));
        assert_eq!(number.values[1], 2.0);
        assert!(!number.set(true, "9", 1.0));
    }

    #[test]
    fn a_quotient_of_a_force_and_a_length_is_a_stiffness() {
        let force = ValueKind::Quantity(Quantity::Force);
        let stiffness = ValueKind::Quantity(Quantity::ForcePerLength);
        let parameters = Node::read(ProjectNode::Parameters(file::ParameterSet {
            id: 1,
            name: "Parameters".into(),
            parameters: vec![
                Parameter::new("RF3", force, 4912.0),
                Parameter::new("U3", ValueKind::LENGTH, 0.01),
                Parameter::new("Stiffness", stiffness, 1.0),
                Parameter::new("Area", ValueKind::Quantity(Quantity::Area), 1.0),
            ],
            ..Default::default()
        }));
        let math = Node::read(ProjectNode::Math(file::MathBlock {
            id: 2,
            name: "Math".into(),
            operation: MathOp::Divide,
            values: vec![0.0, 0.0],
            ..Default::default()
        }));
        let sum = Node::read(ProjectNode::Math(file::MathBlock {
            id: 3,
            name: "Sum".into(),
            ..Default::default()
        }));
        let mut nodes = vec![parameters, math, sum];
        let value = |name: &str| vec![links::VALUE.to_owned(), name.to_owned()];
        let input = |key: &str| vec![links::PARAMETERS.to_owned(), key.to_owned()];
        let link = |item: Vec<String>, slot: Vec<String>, from_math: bool| Link {
            source: if from_math { 2 } else { 1 },
            target: if from_math { 3 } else { 2 },
            item,
            slot,
            ..Default::default()
        };
        let result = value(RESULT);
        // The force goes to A, then the length to B: the result is a force per length.
        assert_eq!(refusal(&nodes, &[], 1, &value("RF3"), 2, &input("a")), None);
        let mut links = vec![link(value("RF3"), input("a"), false)];
        numbers::refresh_kinds(&mut nodes, &mut links);
        assert_eq!(value_kind(&nodes, 2, &input("a")), Some(force));
        // Unlinked, B is a factor until a length goes to it.
        assert_eq!(value_kind(&nodes, 2, &input("b")), Some(ValueKind::Factor));
        assert_eq!(value_kind(&nodes, 2, &result), Some(force));
        assert_eq!(
            refusal(&nodes, &links, 1, &value("U3"), 2, &input("b")),
            None
        );
        links.push(link(value("U3"), input("b"), false));
        let (changed, removed) = numbers::refresh_kinds(&mut nodes, &mut links);
        assert!(changed && !removed);
        assert_eq!(value_kind(&nodes, 2, &result), Some(stiffness));
        // Of a block with a force at A and a stiffness at its result, B is a length.
        links.pop();
        links.push(Link {
            source: 1,
            target: 3,
            item: value("Stiffness"),
            slot: input("a"),
            ..Default::default()
        });
        links.push(link(result.clone(), input("b"), true));
        numbers::refresh_kinds(&mut nodes, &mut links);
        assert_eq!(value_kind(&nodes, 2, &input("b")), Some(ValueKind::LENGTH));
        assert!(refusal(&nodes, &links, 1, &value("Area"), 2, &input("b")).is_some());
        // With a length at B, the result goes on only to a force per length.
        links.truncate(1);
        links.push(link(value("U3"), input("b"), false));
        numbers::refresh_kinds(&mut nodes, &mut links);
        let sum_of_areas = Link {
            source: 1,
            target: 3,
            item: value("Area"),
            slot: input("a"),
            ..Default::default()
        };
        links.push(sum_of_areas);
        numbers::refresh_kinds(&mut nodes, &mut links);
        let why = refusal(&nodes, &links, 2, &result, 3, &input("b")).unwrap();
        assert!(why.contains("force per length"), "{why}");
        links.pop();
        // A product of a force and a length is a moment.
        if let Some((number, _)) = nodes[1].as_number_mut() {
            number.operation = MathOp::Multiply;
        }
        numbers::refresh_kinds(&mut nodes, &mut links);
        assert_eq!(
            value_kind(&nodes, 2, &result),
            Some(ValueKind::Quantity(Quantity::Moment))
        );
        // A sum of a force and a length is none: the link of B goes.
        if let Some((number, _)) = nodes[1].as_number_mut() {
            number.operation = MathOp::Add;
        }
        let (_, removed) = numbers::refresh_kinds(&mut nodes, &mut links);
        assert!(removed);
        assert_eq!(links.len(), 1);
        assert_eq!(value_kind(&nodes, 2, &result), Some(force));
        // Unlinked, its numbers have no kind again.
        links.clear();
        numbers::refresh_kinds(&mut nodes, &mut links);
        assert_eq!(value_kind(&nodes, 2, &result), None);
    }

    #[test]
    fn intermediate_results_have_any_unit_and_convert_back() {
        let force = ValueKind::Quantity(Quantity::Force);
        let parameters = Node::read(ProjectNode::Parameters(file::ParameterSet {
            id: 1,
            name: "Parameters".into(),
            parameters: vec![
                Parameter::new("F", force, 2.0),
                Parameter::new("L", ValueKind::LENGTH, 3.0),
                Parameter::new("E", ValueKind::Quantity(Quantity::Energy), 1.0),
            ],
            ..Default::default()
        }));
        let math = |id, operation| {
            Node::read(ProjectNode::Math(file::MathBlock {
                id,
                name: format!("Math-{id}"),
                operation,
                values: vec![0.0, 0.0],
                ..Default::default()
            }))
        };
        let mut nodes = vec![
            parameters,
            math(2, MathOp::Multiply),
            math(3, MathOp::Add),
            math(4, MathOp::Multiply),
            math(5, MathOp::Divide),
        ];
        let value = |name: &str| vec![links::VALUE.to_owned(), name.to_owned()];
        let input = |key: &str| vec![links::PARAMETERS.to_owned(), key.to_owned()];
        let link = |source, item: Vec<String>, target, key: &str| Link {
            source,
            target,
            item,
            slot: input(key),
            ..Default::default()
        };
        let result = value(RESULT);
        let mut links = vec![
            link(1, value("F"), 2, "a"),
            link(1, value("L"), 2, "b"),
            link(1, value("E"), 3, "a"),
            link(1, value("F"), 4, "a"),
            link(1, value("F"), 4, "b"),
        ];
        numbers::refresh_kinds(&mut nodes, &mut links);
        // A force times a length, a moment, adds to an energy: the unit is the same.
        assert_eq!(
            value_kind(&nodes, 2, &result),
            Some(ValueKind::Quantity(Quantity::Moment))
        );
        assert_eq!(refusal(&nodes, &links, 2, &result, 3, &input("b")), None);
        // A force times a force has a unit no quantity has, and goes on to Math blocks...
        let squared = value_kind(&nodes, 4, &result);
        assert_eq!(
            squared.map(ValueKind::describe).as_deref(),
            Some("quantity in kg² m²/s^4")
        );
        assert_eq!(refusal(&nodes, &links, 4, &result, 5, &input("a")), None);
        links.push(link(4, result.clone(), 5, "a"));
        links.push(link(1, value("F"), 5, "b"));
        let (_, removed) = numbers::refresh_kinds(&mut nodes, &mut links);
        assert!(!removed);
        // ... where over a force it is a force again.
        assert_eq!(value_kind(&nodes, 5, &result), Some(force));
        // A sum of an energy takes it no more.
        let why = refusal(&nodes, &links, 4, &result, 3, &input("b")).unwrap();
        assert!(why.contains("kg² m²/s^4"), "{why}");
    }

    #[test]
    fn a_frame_lies_around_its_blocks_with_its_label_above() {
        let mut graph = Snarl::new();
        graph.insert_node(egui::pos2(0.0, 0.0), 1);
        graph.insert_node(egui::pos2(300.0, 100.0), 2);
        graph.insert_node(egui::pos2(900.0, 900.0), 3);
        let sizes = HashMap::from([(1, egui::vec2(200.0, 80.0)), (2, egui::vec2(220.0, 120.0))]);
        let frame = file::Frame {
            label: "Load cases".into(),
            color: frames::FRAME_COLORS[1].1,
            nodes: vec![1, 2],
        };
        let rect = frames::frame_rect(&frame, &graph, &sizes).unwrap();
        assert!(rect.contains(egui::pos2(10.0, 10.0)));
        assert!(rect.contains(egui::pos2(510.0, 210.0)));
        assert!(!rect.contains(egui::pos2(900.0, 900.0)));
        // The label band lies above the blocks.
        assert!(rect.top() < -30.0);
        let empty = file::Frame {
            nodes: vec![5],
            ..frame
        };
        assert!(frames::frame_rect(&empty, &graph, &sizes).is_none());
        assert_eq!(frames::frame_of(&[empty], 5), Some(0));
    }

    #[test]
    fn an_exposed_transform_moves_the_mesh_of_a_closed_analysis() {
        let mut mesh = plx_mesh::FeMesh::default();
        mesh.set_node(1, [0.0; 3]);
        mesh.set_node(2, [1.0, 0.0, 0.0]);
        mesh.add_element(plx_mesh::Element {
            id: 1,
            type_name: "T3D2".into(),
            shape: plx_mesh::ElementShape::Line2,
            nodes: vec![1, 2],
        })
        .unwrap();
        mesh.parts.push(plx_mesh::Part {
            name: "A".into(),
            elements: vec![1],
        });
        let mut model = plx_model::FeModel::default();
        let shift = plx_model::MeshOperation {
            name: "Transform-1".into(),
            parts: vec!["A".into()],
            kind: plx_model::MeshOperationKind::Transform(plx_mesh::MeshTransform::Translate([
                0.0, 1.0, 0.0,
            ])),
        };
        model.set_mesh_operations(&mut mesh, vec![shift]);
        let path = plx_model::ValuePath::new(
            plx_model::ItemRef::MeshOperation("Transform-1".into()),
            plx_model::Field::Y,
        );
        let key = model.expose(path.clone());
        let data = ProjectAnalysis {
            mesh,
            model,
            ..Default::default()
        };
        let mut analysis = Node::analysis(1, "Analysis-1".into(), data);
        let rows = exposed_rows(analysis.exposing_fe().unwrap(), &BTreeSet::new(), &[]);
        assert_eq!(rows[1].text, "Transform-1 Y");
        assert!(analysis.set_exposed(key, 4.0));
        let data = analysis.link_data().unwrap();
        assert_eq!(data.fe.value_at(&path), Some(4.0));
        assert_eq!(data.mesh.node(2), Some([1.0, 4.0, 0.0]));
        // A study that drives the transform runs the same body at each of its points.
        let print = |node: &Node| {
            study_analysis_print(node.link_data().unwrap(), StudyMesh::Own, |k| k == key)
        };
        let before = print(&analysis);
        assert!(analysis.set_exposed(key, 7.0));
        assert_eq!(print(&analysis), before);
    }

    #[test]
    fn exposed_parameters_take_numbers_of_their_kind() {
        let path = |p: &[&str]| p.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        let mut model = plx_model::FeModel::default();
        let mut step = plx_model::Step::new_static("Step-1");
        step.loads.push(plx_model::Load {
            id: plx_model::Id::new(),
            distribution: None,
            name: "Force-1".into(),
            active: true,
            visible: true,
            region: plx_model::Region::Nodes(vec![1]),
            kind: plx_model::LoadKind::force([0.0, -10.0, 0.0]),
            amplitude: None,
            factor_amplitude: None,
            tables: Default::default(),
            system: None,
            follower: false,
        });
        model.steps.push(step);
        let force = plx_model::ValuePath::new(
            plx_model::ItemRef::Load {
                step: "Step-1".into(),
                name: "Force-1".into(),
            },
            plx_model::Field::Y,
        );
        let key = model.expose(force.clone());
        let data = ProjectAnalysis {
            model,
            ..Default::default()
        };
        let mut analysis = Node::analysis(1, "Analysis-1".into(), data);
        let parameters = Node::new(
            2,
            "Parameters".into(),
            Block::Parameters(ParametersBlock {
                units: Default::default(),
                parameters: vec![
                    Parameter::new("Load", ValueKind::Quantity(Quantity::Force), -50.0),
                    Parameter::new("Width", ValueKind::LENGTH, 0.1),
                ],
            }),
        );
        let fe = analysis.exposing_fe().unwrap();
        let rows = exposed_rows(fe, &BTreeSet::new(), &[]);
        assert_eq!(rows[0].path, [links::PARAMETERS]);
        assert_eq!(rows[1].path, path(&["Parameters", &key.to_string()]));
        assert!(rows[1].input && rows[1].text == "Force-1 Y");
        assert!(analysis.set_exposed(key, -50.0));
        assert!(!analysis.set_exposed(key, -50.0));
        let fe = analysis.exposing_fe().unwrap();
        assert_eq!(fe.value_at(&force), Some(-50.0));
        let nodes = vec![analysis, parameters];
        let slot = path(&["Parameters", &key.to_string()]);
        assert_eq!(
            refusal(&nodes, &[], 2, &path(&["Value", "Load"]), 1, &slot),
            None
        );
        let length = refusal(&nodes, &[], 2, &path(&["Value", "Width"]), 1, &slot).unwrap();
        assert!(
            length.contains("a length") && length.contains("a force"),
            "{length}"
        );
        assert_eq!(parameter_label(&nodes, 1, &slot), "Force-1 Y");
        // The Inputs row of a study takes any kind, a column only its own.
        let mut nodes = nodes;
        let mut table = plx_model::StudyTable::default();
        table.add_input("Width", Some(ValueKind::LENGTH), 0.1);
        table.add_input("Any", None, 0.0);
        nodes.push(Node::new(
            3,
            "Study".into(),
            Block::Study(Box::new(StudyBlock {
                units: Default::default(),
                table,
                body: None,
                active: None,
                report: None,
            })),
        ));
        let refused = |item: &[&str]| refusal(&nodes, &[], 3, &path(item), 1, &slot);
        assert_eq!(refused(&["Loop"]), None);
        assert_eq!(refused(&["Loop", "Any"]), None);
        assert!(refused(&["Loop", "Width"]).is_some());
    }

    #[test]
    fn a_measure_block_reads_back_and_takes_results() {
        let block = file::MeasureBlock {
            id: 5,
            name: "Measure".into(),
            position: [5.0, 6.0],
            units: plx_model::UnitSystem::MmTonSC,
            kind: MeasureKind::Eigenfrequency { mode: 2 },
            value: Some(12.5),
            evaluated: Some(7),
        };
        let mut node = Node::read(ProjectNode::Measure(block.clone()));
        assert_eq!(node.kind(), Kind::Measure);
        let incoming = vec![(vec!["Measure".to_owned()], "Modal".to_owned())];
        let rows = measure_rows(&node, &incoming, CellState::UpToDate, false);
        assert_eq!(rows[0].path, ["Measure"]);
        assert!(rows[0].input && !rows[0].output);
        assert_eq!(rows[0].linked_from.as_deref(), Some("Modal"));
        assert_eq!(rows[2].path, ["Value", "Result"]);
        assert!(rows[2].output && !rows[2].input);
        assert_eq!(rows[2].text, "12.5 Hz");
        // The results of a step go to its Results row; nothing else does.
        let results = vec!["Results".to_owned(), "Step-1".to_owned()];
        assert!(links::compatible(&results, &rows[0].path));
        assert!(!links::compatible(&["Mesh".to_owned()], &rows[0].path));
        match node.written([5.0, 6.0]) {
            Some(ProjectNode::Measure(written)) => assert_eq!(written, block),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_measure_at_a_point_takes_lengths_at_its_coordinates() {
        let path = |p: &[&str]| p.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        let block = file::MeasureBlock {
            id: 5,
            name: "Measure".into(),
            kind: MeasureKind::PointValue {
                field: "DISP".into(),
                component: "U2".into(),
                point: [0.1, 0.02, 0.0],
            },
            ..Default::default()
        };
        let node = Node::read(ProjectNode::Measure(block));
        let incoming = vec![(path(&["Parameters", "x"]), "Parameters".to_owned())];
        let rows = measure_rows(&node, &incoming, CellState::Missing, false);
        let coordinates: Vec<_> = rows.iter().filter(|r| r.input).skip(1).collect();
        assert_eq!(coordinates.len(), 3);
        assert_eq!(coordinates[0].path, path(&["Parameters", "x"]));
        assert_eq!(coordinates[0].text, "X");
        assert_eq!(coordinates[0].linked_from.as_deref(), Some("Parameters"));
        assert!(coordinates[2].linked_from.is_none());
        assert_eq!(rows.last().unwrap().path, path(&["Value", "Result"]));
        let parameters = Node::new(
            2,
            "Parameters".into(),
            Block::Parameters(ParametersBlock {
                units: Default::default(),
                parameters: vec![
                    Parameter::new("Load", ValueKind::Quantity(Quantity::Force), -50.0),
                    Parameter::new("Length", ValueKind::LENGTH, 0.3),
                ],
            }),
        );
        let nodes = vec![node, parameters];
        let x = path(&["Parameters", "x"]);
        assert_eq!(value_kind(&nodes, 5, &x), Some(ValueKind::LENGTH));
        assert_eq!(parameter_label(&nodes, 5, &x), "X");
        assert_eq!(
            refusal(&nodes, &[], 2, &path(&["Value", "Length"]), 5, &x),
            None
        );
        assert!(refusal(&nodes, &[], 2, &path(&["Value", "Load"]), 5, &x).is_some());
        // Another measure has no coordinates to link to.
        let mut extreme = nodes;
        if let Some(measure) = extreme[0].as_measure_mut() {
            measure.kind = MeasureKind::default();
        }
        assert_eq!(value_kind(&extreme, 5, &x), None);
        assert_eq!(
            measure_rows(&extreme[0], &[], CellState::Missing, false).len(),
            3
        );
    }

    #[test]
    fn a_math_block_takes_the_value_of_a_measure_block() {
        let measure = |id, kind| {
            Node::read(ProjectNode::Measure(file::MeasureBlock {
                id,
                name: format!("Measure-{id}"),
                kind,
                value: Some(0.25),
                ..Default::default()
            }))
        };
        let stress = MeasureKind::default();
        let frequency = MeasureKind::Eigenfrequency { mode: 1 };
        let math = Node::read(ProjectNode::Math(file::MathBlock {
            id: 3,
            name: "Math".into(),
            operation: MathOp::Multiply,
            values: vec![0.0, 2.0],
            ..Default::default()
        }));
        let mut nodes = vec![measure(1, stress), measure(2, frequency), math];
        let value = vec![links::VALUE.to_owned(), RESULT.to_owned()];
        let input = vec![links::PARAMETERS.to_owned(), "a".to_owned()];
        assert_eq!(
            value_kind(&nodes, 1, &value),
            Some(ValueKind::Quantity(Quantity::Pressure))
        );
        assert_eq!(
            value_kind(&nodes, 2, &value),
            Some(ValueKind::Quantity(Quantity::Frequency))
        );
        // A sum without links takes the value, and then only values of its kind.
        if let Some((number, _)) = nodes[2].as_number_mut() {
            number.operation = MathOp::Add;
        }
        assert_eq!(refusal(&nodes, &[], 1, &value, 3, &input), None);
        let mut links = vec![Link {
            source: 1,
            target: 3,
            item: value.clone(),
            slot: input.clone(),
            ..Default::default()
        }];
        numbers::refresh_kinds(&mut nodes, &mut links);
        let b = vec![links::PARAMETERS.to_owned(), "b".to_owned()];
        assert_eq!(refusal(&nodes, &links, 1, &value, 3, &b), None);
        assert!(refusal(&nodes, &links, 2, &value, 3, &b).is_some());
    }

    #[test]
    fn later_steps_leave_the_results_of_earlier_steps_up_to_date() {
        let mut model = plx_model::FeModel::default();
        model.steps.push(plx_model::Step::new_static("Step-1"));
        let mesh = plx_mesh::FeMesh::default();
        let results = plx_io::project::BoundResults {
            input: Some(crate::solve_fingerprint::solve_input(&mesh, &model)),
            steps: crate::solve_fingerprint::step_inputs(&mesh, &model),
            ..Default::default()
        };
        model.steps.push(plx_model::Step::new_static("Step-2"));
        let data = ProjectAnalysis {
            mesh,
            model,
            results: Some(results),
            ..Default::default()
        };
        let mut node = Node::analysis(1, "Global".into(), data);
        node.refresh_closed();
        // The analysis has a step to solve anew ...
        assert!(node.results_out_of_date());
        // ... but a submodel taking the results of the first step keeps them.
        let link = |step: &str, input: &str| Link {
            source: 1,
            target: 2,
            item: vec![links::RESULTS.into(), step.into()],
            slot: ["Setup", "Steps", "Step-1", input]
                .map(String::from)
                .to_vec(),
            ..Default::default()
        };
        assert!(!node.out_of_date_for(&link("Step-1", "Global results")));
        assert!(node.out_of_date_for(&link("Step-2", "Global results")));
        // Eigenmodes are written anew by every frequency step.
        assert!(node.out_of_date_for(&link("Step-1", "Eigenmodes")));
        // A renamed first step keeps them: the solver computes the same.
        if let Some(data) = node.as_analysis_mut().and_then(|a| a.data.as_mut()) {
            data.model.steps[0].name = "Load".into();
        }
        node.refresh_closed();
        assert!(!node.out_of_date_for(&link("Load", "Global results")));
        // A changed first step does not.
        if let Some(data) = node.as_analysis_mut().and_then(|a| a.data.as_mut()) {
            data.model.steps[0].active = false;
        }
        node.refresh_closed();
        assert!(node.out_of_date_for(&link("Load", "Global results")));
    }

    #[test]
    fn results_made_from_changed_data_are_out_of_date() {
        // Never solved: nothing to take, and nothing out of date either.
        let mut node = Node::analysis(1, "Static".into(), ProjectAnalysis::default());
        node.refresh_closed();
        assert!(node.bound_results().is_none() && !node.results_out_of_date());
        // Results whose model changed since: a results link waits for them to be made anew,
        // and Solve of the target solves this one first.
        let data = ProjectAnalysis {
            results: Some(Default::default()),
            ..Default::default()
        };
        let mut node = Node::analysis(1, "Static".into(), data);
        node.refresh_closed();
        assert!(node.results_out_of_date());
    }

    #[test]
    fn analyses_taking_the_mesh_of_a_changed_block_are_out_of_date() {
        let block = file::MeshBlock {
            id: 2,
            name: "Box".into(),
            position: [0.0; 2],
            space: ModelSpace::ThreeD,
            units: plx_model::UnitSystem::MmTonSC,
            primitive: MeshPrimitive::default_box(),
            mesh: Default::default(),
            generated: None,
        };
        let mut nodes = vec![
            Node::read(ProjectNode::Mesh(Box::new(block))),
            Node::analysis(1, "Analysis-1".into(), ProjectAnalysis::default()),
            Node::analysis(3, "Analysis-2".into(), ProjectAnalysis::default()),
        ];
        let link = |source, target| Link {
            source,
            target,
            item: vec![links::MESH.to_owned()],
            ..Default::default()
        };
        // Analysis-2 takes the mesh of Analysis-1, which takes that of the block.
        let links = [link(2, 1), link(1, 3)];
        assert_eq!(stale_meshes(&nodes, &links, 3), ["Box"]);
        assert!(generate_stale_meshes(&mut nodes, None).0);
        assert!(stale_meshes(&nodes, &links, 3).is_empty());
        // New divisions: the analyses hold the old mesh until the block makes the new one.
        let primitive = &mut nodes[0].as_mesh_mut().unwrap().primitive;
        assert!(primitive.set("nx", plx_model::ParamValue::Count(20)));
        assert_eq!(stale_meshes(&nodes, &links, 1), ["Box"]);
        assert_eq!(stale_meshes(&nodes, &links, 3), ["Box"]);
        // A mesh that cannot be made leaves the block out of date and is not reported as
        // made, so that Update Project does not try again and again.
        nodes[0].as_mesh_mut().unwrap().primitive = MeshPrimitive::default_rectangle();
        let (made, lines) = generate_stale_meshes(&mut nodes, None);
        assert!(!made);
        assert!(lines[0].contains("no mesh made"), "{lines:?}");
        assert_eq!(stale_meshes(&nodes, &links, 3), ["Box"]);
    }

    #[test]
    fn updating_one_block_makes_only_the_meshes_before_it() {
        let block = |id, name: &str| {
            Node::read(ProjectNode::Mesh(Box::new(file::MeshBlock {
                id,
                name: name.into(),
                position: [0.0; 2],
                space: ModelSpace::ThreeD,
                units: plx_model::UnitSystem::MmTonSC,
                primitive: MeshPrimitive::default_box(),
                mesh: Default::default(),
                generated: None,
            })))
        };
        let mut nodes = vec![block(1, "Box"), block(2, "Box-2")];
        let (made, lines) = generate_stale_meshes(&mut nodes, Some(&[2]));
        assert!(made);
        assert_eq!(lines.len(), 1, "{lines:?}");
        assert_ne!(nodes[0].mesh_state(), CellState::UpToDate);
        assert_eq!(nodes[1].mesh_state(), CellState::UpToDate);
    }

    #[test]
    fn pins_take_only_data_of_their_type_and_model_space() {
        let path = |p: &[&str]| p.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        let analysis = |id, space| {
            let mut model = plx_model::FeModel::default();
            model.properties.space = space;
            let data = ProjectAnalysis {
                model,
                ..Default::default()
            };
            Node::analysis(id, format!("Analysis-{id}"), data)
        };
        let mesh = |id, space| {
            let primitive = MeshPrimitive::default_rectangle();
            let units = plx_model::UnitSystem::default();
            let mesh = Default::default();
            Node::mesh_block(id, "Plate".into(), space, units, primitive, mesh, None)
        };
        let parameters = Node::new(
            5,
            "Parameters".into(),
            Block::Parameters(ParametersBlock {
                units: Default::default(),
                parameters: vec![Parameter::new("Width", ValueKind::LENGTH, 0.1)],
            }),
        );
        let nodes = vec![
            analysis(1, ModelSpace::ThreeD),
            analysis(2, ModelSpace::PlaneStress),
            mesh(3, ModelSpace::PlaneStress),
            parameters,
        ];
        let why = |source, item: &[&str], target, slot: &[&str]| {
            refusal(&nodes, &[], source, &path(item), target, &path(slot))
        };
        // The mesh of a plane stress block goes to the plane stress analysis, not to the 3D one.
        assert_eq!(why(3, &["Mesh"], 2, &["Mesh"]), None);
        let space = why(3, &["Mesh"], 1, &["Mesh"]).unwrap();
        assert!(space.contains("same model space"), "{space}");
        assert!(why(2, &["Setup"], 1, &["Setup"]).is_some());
        // A pin of another type refuses it, whatever the model space.
        let kind = why(3, &["Mesh"], 2, &["Setup"]).unwrap();
        assert_eq!(
            kind,
            "Mesh of Plate is a mesh and Setup of Analysis-2 takes a setup."
        );
        // A number has no model space: it goes to a length of the mesh block.
        let width = path(&["Value", "Width"]);
        let slot = path(&["Parameters", "lx"]);
        assert_eq!(refusal(&nodes, &[], 5, &width, 3, &slot), None);
        assert!(refusal(&nodes, &[], 5, &width, 3, &path(&["Parameters", "nx"])).is_some());
        // A link cannot go back: the analysis comes after the mesh block once linked.
        let link = Link {
            source: 3,
            target: 2,
            item: path(&["Mesh"]),
            ..Default::default()
        };
        let back = refusal(&nodes, &[link], 2, &path(&["Mesh"]), 3, &path(&["Mesh"]));
        assert!(back.is_some());
    }

    #[test]
    fn a_file_block_gives_the_results_of_each_step_of_its_file() {
        let frd = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/kragbalken_c3d8.frd");
        let read = read_source(&frd, UnitSystem::default()).unwrap();
        assert_eq!(read.steps, [("Step-1".to_owned(), 1)]);
        assert_eq!(read.model.fe.steps[0].name, "Step-1");
        assert!(read.model.mesh.node_count() > 0);
        let path = frd.to_string_lossy().into_owned();
        let mut node = Node::file_block(1, "File".into(), path.clone(), UnitSystem::default());
        assert_eq!(node.file_state(None), CellState::OutOfDate, "not read yet");
        let file = node.as_file_mut().unwrap();
        (file.model, file.steps, file.space) = (read.model, read.steps, read.space);
        file.read = file_print(&frd);
        assert_eq!(node.file_state(None), CellState::UpToDate);
        let rows = file_rows(&node, None);
        let step = rows
            .iter()
            .find(|r| r.path == ["Results", "Step-1"])
            .unwrap();
        assert!(step.output && !step.input);
        // The project file keeps only its path.
        match node.written([1.0, 2.0]) {
            Some(ProjectNode::File(written)) => assert_eq!(written.path, path),
            other => panic!("{other:?}"),
        }
        // A submodel takes them as its global results.
        let mut model = plx_model::FeModel::default();
        model.properties.kind = plx_model::ModelKind::Submodel;
        model.steps.push(plx_model::Step::new_static("Step-1"));
        let data = ProjectAnalysis {
            model,
            ..Default::default()
        };
        let nodes = [node, Node::analysis(2, "Detail".into(), data)];
        let global = [links::GLOBAL_RESULTS.to_owned()];
        let item = ["Results".to_owned(), "Step-1".to_owned()];
        assert_eq!(refusal(&nodes, &[], 1, &item, 2, &global), None);
        // Other files say what it reads.
        let error = read_source(Path::new("notes.txt"), UnitSystem::default()).err();
        assert!(error.is_some_and(|e| e.contains("no results file")));
    }

    #[test]
    fn the_parts_of_a_geometry_file_link_one_by_one() {
        match plx_mesher::self_test() {
            Ok(_) => {}
            Err(error) if std::env::var_os("NODALIS_REQUIRE_GMSH").is_none() => {
                eprintln!("Gmsh not available, test skipped: {error}");
                return;
            }
            Err(error) => panic!("Gmsh is required: {error}"),
        }
        let step = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/zwei_bloecke.step");
        let read = read_source(&step, UnitSystem::default()).unwrap();
        assert_eq!(read.parts.len(), 2, "{:?}", read.parts);
        let path = step.to_string_lossy().into_owned();
        let mut node = Node::file_block(1, "File".into(), path, UnitSystem::default());
        let file = node.as_file_mut().unwrap();
        (file.model, file.parts) = (read.model, read.parts.clone());
        file.read = file_print(&step);
        // The Geometry row opens to the parts, each with an output of its own.
        let geometry = [links::GEOMETRY.to_owned()];
        let rows = file_rows(&node, None);
        assert_eq!(rows[0].expander, Some(false));
        assert_eq!(rows.iter().filter(|r| r.path[0] == geometry[0]).count(), 1);
        node.expanded.insert(geometry.to_vec());
        let rows = file_rows(&node, None);
        let part = |name: &str| vec![links::GEOMETRY.to_owned(), name.to_owned()];
        for name in &read.parts {
            let row = rows.iter().find(|r| r.path == part(name)).unwrap();
            assert!(row.output && !row.input && row.depth == 1);
        }
        // A part goes to the Geometry row of an analysis, and only there.
        let first = part(&read.parts[0]);
        assert!(links::compatible(&first, &geometry));
        assert!(!links::compatible(&first, &[links::MESH.to_owned()]));
        assert!(!links::compatible(&first, &first));
        // The parts linked from the same source go together, in the order of the links.
        let link = |source, item: Vec<String>| Link {
            source,
            target: 2,
            item,
            slot: geometry.to_vec(),
            ..Default::default()
        };
        let all = [
            link(1, part(&read.parts[1])),
            link(3, part("OTHER")),
            link(1, first.clone()),
        ];
        assert_eq!(
            links::linked_parts(&all, 1, 2),
            [read.parts[1].as_str(), read.parts[0].as_str()]
        );
        // All parts are the whole geometry; one part is made without the others.
        let file = node.as_file_mut().unwrap();
        let whole = file.subset(&read.parts).unwrap().unwrap();
        assert_eq!(Some(&whole), file.model.geometry.as_ref());
        assert!(file.subset(&["OTHER".to_owned()]).is_err());
        let one = &read.parts[..1];
        assert_eq!(file.subset(one), Ok(None), "made on a worker");
        let made = loop {
            file.poll_subsets();
            match file.subset(one) {
                Ok(None) => std::thread::sleep(Duration::from_millis(20)),
                made => break made.unwrap().unwrap(),
            }
        };
        assert_eq!(plx_mesher::part_names(&made).unwrap(), one);
    }

    #[test]
    fn a_submodel_has_a_global_results_row() {
        let mut model = plx_model::FeModel::default();
        model.steps.push(plx_model::Step::new_static("Step-1"));
        let data = ProjectAnalysis {
            model: model.clone(),
            ..Default::default()
        };
        let node = Node::analysis(1, "Detail".into(), data);
        let status = node.status().unwrap();
        let global = |fe: &plx_model::FeModel, incoming: &[(Vec<String>, String)]| {
            let rows = block_rows(status, Some(fe), &BTreeSet::new(), incoming, &[], None);
            rows.into_iter().find(|r| r.path == [links::GLOBAL_RESULTS])
        };
        // A general model has none; a submodel has an input for the global results.
        assert!(global(&model, &[]).is_none());
        model.properties.kind = plx_model::ModelKind::Submodel;
        let row = global(&model, &[]).unwrap();
        assert!(row.input && !row.output && row.state == Some(CellState::Missing));
        // Once linked, it names the global analysis, whichever step takes the results.
        let slot = ["Setup", "Steps", "Step-1", "Global results"]
            .map(String::from)
            .to_vec();
        let row = global(&model, &[(slot, "Global".into())]).unwrap();
        assert_eq!(row.text, "Global results of Global");
        assert_eq!(row.linked_from.as_deref(), Some("Global"));
        assert!(row.state.is_none());
    }

    #[test]
    fn the_rows_of_steps_trace_the_chain() {
        let mut model = plx_model::FeModel::default();
        model.steps.push(plx_model::Step::new_static("Preload"));
        let mut modes = plx_model::Step::new_frequency("Modes");
        if let plx_model::StepKind::Frequency(settings) = &mut modes.kind {
            settings.base = plx_model::Base::PreviousState;
        }
        model.steps.push(modes);
        model.steps.push(plx_model::Step::new_static("Unload"));
        model.steps.push(plx_model::Step::new_frequency("Free"));
        let data = ProjectAnalysis {
            model: model.clone(),
            ..Default::default()
        };
        let node = Node::analysis(1, "Beam".into(), data);
        let status = node.status().unwrap();
        let open = [
            vec!["Setup".to_owned()],
            vec!["Setup".into(), "Steps".into()],
        ];
        let rows = block_rows(status, Some(&model), &open.into(), &[], &[], None);
        let marks = chain_marks(&model, &rows);
        for group in [vec!["Setup", "Steps"], vec!["Results"]] {
            let mark = |step: &str| {
                let mut path: Vec<String> = group.iter().map(|p| p.to_string()).collect();
                path.push(step.to_owned());
                marks[&path]
            };
            // The state flows from Preload past Modes to Unload.
            let preload = mark("Preload");
            assert!(preload.continues && !preload.above && preload.below);
            let modes = mark("Modes");
            assert!(modes.probing && modes.branch && modes.above && modes.below);
            let unload = mark("Unload");
            assert!(unload.continues && unload.above && !unload.below);
            // A frequency step about the unloaded model stands apart.
            let free = mark("Free");
            assert!(free.probing && !free.branch && !free.below);
        }
        // The tooltip of a step row says where it starts.
        let row = (rows.iter())
            .find(|r| r.path == ["Results", "Modes"])
            .unwrap();
        assert!(row.tip.contains("Base: End of Preload"), "{}", row.tip);
    }

    #[test]
    fn a_study_counts_how_its_analyses_mesh() {
        let geometry = |brep: &str, max_size: f64| plx_model::Geometry {
            source: "part.step".into(),
            brep: brep.into(),
            meshing: plx_model::MeshingParameters {
                max_size,
                ..Default::default()
            },
            mesh_items: Vec::new(),
            part_names: Vec::new(),
            refinement: 1.0,
            signatures: Vec::new(),
        };
        let fe = plx_model::FeModel::default();
        let inputs = std::collections::BTreeMap::new();
        let mut meshed = plx_mesh::FeMesh::default();
        meshed.set_node(1, [0.0; 3]);
        let print = |geometry: &plx_model::Geometry, mesh: &plx_mesh::FeMesh, kind| {
            let data = links::Data {
                geometry: Some(geometry),
                mesh,
                fe: &fe,
                mesh_inputs: &inputs,
            };
            study_analysis_print(data, kind, |_| false)
        };
        let empty = plx_mesh::FeMesh::default();
        let coarse = geometry("a", 10.0);
        let fine = geometry("a", 5.0);
        // An analysis that meshes the shapes a block of the study gives it anew: the shapes
        // and the mesh change from point to point and do not count, how it meshes does.
        let remeshed = print(&coarse, &empty, StudyMesh::Remeshed);
        assert_eq!(
            print(&geometry("b", 10.0), &meshed, StudyMesh::Remeshed),
            remeshed
        );
        assert_ne!(print(&fine, &empty, StudyMesh::Remeshed), remeshed);
        let mut setup = coarse.clone();
        setup.mesh_items.push(plx_model::MeshSetupItem {
            name: "Local-1".into(),
            kind: plx_model::MeshSetupKind::LocalMeshSize {
                faces: vec![1],
                edges: Vec::new(),
                size: 1.0,
            },
        });
        assert_ne!(print(&setup, &empty, StudyMesh::Remeshed), remeshed);
        // Its own mesh counts as it is.
        assert_ne!(
            print(&coarse, &meshed, StudyMesh::Own),
            print(&coarse, &empty, StudyMesh::Own)
        );
        // A materials block has no mesh.
        assert_eq!(
            print(&coarse, &empty, StudyMesh::None),
            print(&fine, &meshed, StudyMesh::None)
        );
    }

    #[test]
    fn a_study_reads_back_with_its_points() {
        let mut table = plx_model::StudyTable::default();
        table.add_input("Force", None, 2.0);
        table.add_output("Stress");
        table.add_point();
        table.points[0].run = PointRun::Solved;
        table.points[0].outputs = vec![Some(3.0)];
        table.points[0].fingerprint = Some(5);
        let block = file::StudyBlock {
            id: 7,
            name: "Study".into(),
            position: [7.0, 8.0],
            units: plx_model::UnitSystem::MmTonSC,
            table,
        };
        let mut node = Node::read(ProjectNode::Study(block.clone()));
        assert_eq!(node.kind(), Kind::Study);
        match node.written([7.0, 8.0]) {
            Some(ProjectNode::Study(written)) => assert_eq!(written, block),
            other => panic!("{other:?}"),
        }
        // A copy keeps its columns and points, without what they gave.
        let copy = node.copied(8, "Copy".into()).unwrap();
        let study = copy.as_study().unwrap();
        assert_eq!((copy.id, copy.kind()), (8, Kind::Study));
        assert_eq!(study.units, block.units);
        assert_eq!(study.table.points.len(), 2);
        assert_eq!(study.table.points[0].run, PointRun::NotSolved);
        assert_eq!(study.table.points[0].outputs, [None]);
    }

    /// A mesh block with its mesh made, and a mesh export block that took it over.
    fn export_of_a_box(path: &Path) -> (Node, Node) {
        let block = file::MeshBlock {
            id: 1,
            name: "Box".into(),
            position: [0.0; 2],
            space: ModelSpace::ThreeD,
            units: Default::default(),
            primitive: MeshPrimitive::default_box(),
            mesh: Default::default(),
            generated: None,
        };
        let mut source = Node::read(ProjectNode::Mesh(Box::new(block)));
        source.generate().unwrap();
        let mut export = Node::mesh_export(
            2,
            "Mesh Export".into(),
            path.to_string_lossy().into_owned(),
            None,
        );
        // What the link does.
        let copy = links::MeshCopy::of(source.link_data().unwrap());
        export.closed_model_mut().unwrap().take_mesh(copy);
        (source, export)
    }

    #[test]
    fn a_mesh_export_block_writes_the_mesh_as_an_input_file_once() {
        let dir = std::env::temp_dir().join(format!("plx-export-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("box.inp");
        let (source, mut export) = export_of_a_box(&file);
        assert_eq!(export.export_state(None), CellState::OutOfDate);
        let written = export.write_export(None).unwrap();
        assert_eq!(written, file);
        assert_eq!(export.export_state(None), CellState::UpToDate);
        // What it wrote reads back with the same mesh and the sides as sets.
        let text = std::fs::read_to_string(&file).unwrap();
        let read = plx_io::inp::read_inp_str(&text, None).unwrap();
        let made = &source.as_mesh().unwrap().model.mesh;
        assert_eq!(read.mesh.element_count(), made.element_count());
        assert_eq!(read.mesh.node_ids().len(), made.node_ids().len());
        for (name, nodes) in &made.node_sets {
            assert_eq!(
                read.mesh.node_sets.get(name),
                Some(nodes),
                "node set {name}"
            );
        }
        assert_eq!(read.mesh.surfaces.len(), made.surfaces.len());
        let upper = text.to_uppercase();
        assert!(upper.contains("*NSET") && upper.contains("*SURFACE"));
        // A relative path starts at the folder of the project file; a new mesh, a new path,
        // or a deleted file makes it out of date again.
        export.as_export_mut().unwrap().path = "box.inp".into();
        assert_eq!(export.export_state(Some(&dir)), CellState::OutOfDate);
        export.write_export(Some(&dir)).unwrap();
        assert_eq!(export.export_state(Some(&dir)), CellState::UpToDate);
        std::fs::remove_file(&file).unwrap();
        assert_eq!(export.export_state(Some(&dir)), CellState::OutOfDate);
        export.write_export(Some(&dir)).unwrap();
        (export.as_export_mut().unwrap().model.mesh).set_node(1, [9.0, 9.0, 9.0]);
        assert_eq!(export.export_state(Some(&dir)), CellState::OutOfDate);
        // The block is saved with the path and what it wrote, but without the mesh.
        match export.written([3.0, 4.0]) {
            Some(ProjectNode::MeshExport(saved)) => {
                assert_eq!(
                    (saved.path.as_str(), saved.position),
                    ("box.inp", [3.0, 4.0])
                );
                assert!(saved.written.is_some());
            }
            other => panic!("{other:?}"),
        }
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn a_mesh_export_block_says_why_it_cannot_write() {
        let (_, mut export) = export_of_a_box(Path::new("/nonexistent-folder/box.inp"));
        let why = export.write_export(None).unwrap_err();
        assert!(why.contains("not written"), "{why}");
        export.as_export_mut().unwrap().path = String::new();
        assert!(export.write_export(None).unwrap_err().contains("no file"));
        assert_eq!(export.export_state(None), CellState::Missing);
        // Nothing linked yet.
        let mut empty = Node::mesh_export(3, "Empty".into(), "x.inp".into(), None);
        assert!(empty.write_export(None).unwrap_err().contains("no mesh"));
        let rows = export_rows(&empty, None, &[]);
        assert!(rows[0].input && !rows[0].output && rows[1].widget.is_some());
    }

    #[test]
    fn a_campbell_block_keeps_its_sweep_and_lists_the_critical_speeds() {
        use plx_model::campbell::{Mode, Setup, SpeedPoint, Whirl};
        // A mode that falls through the 1x line between 600 and 1000 rad/s.
        let points: Vec<SpeedPoint> = (1..=5)
            .map(|i| SpeedPoint {
                speed: 200.0 * i as f64,
                modes: vec![Mode {
                    frequency: 200.0 - 10.0 * i as f64,
                    whirl: Whirl::Backward,
                    unstable: false,
                    nodal_diameter: None,
                }],
            })
            .collect();
        let setup = Setup {
            max_speed: 1000.0,
            ..Setup::default()
        };
        let window = CampbellWindow::saved(setup.clone(), points.clone(), None, Some(9));
        let mut node = Node::campbell_block(4, "Campbell".into(), window);
        let saved = match node.written([1.0, 2.0]) {
            Some(ProjectNode::Campbell(saved)) => *saved,
            other => panic!("{other:?}"),
        };
        assert_eq!(
            (&saved.setup, &saved.points, saved.fingerprint),
            (&setup, &points, Some(9))
        );
        let read = Node::read(ProjectNode::Campbell(Box::new(saved)));
        assert_eq!(read.kind(), Kind::Campbell);
        let incoming = [(vec![links::ANALYSIS.to_owned()], "Rotor".to_owned())];
        let rows = campbell_rows(&read, &incoming, CellState::UpToDate);
        assert!(rows[0].input && rows[0].linked_from.as_deref() == Some("Rotor"));
        assert_eq!(rows[1].text, "5 speeds, 1910 to 9549 rpm");
        // The 1x line f = speed / 2 pi meets the mode at 200 / (1 / 2 pi + 0.05) = 956 rad/s.
        let texts: Vec<&str> = rows.iter().map(|r| r.text.as_str()).collect();
        assert_eq!(texts[2..], ["1x: 9131 rpm"]);
        assert!(rows[2].tip.contains("mode 1 (backward)"), "{}", rows[2].tip);
    }

    #[test]
    fn recent_projects_keep_their_order_and_only_existing_files() {
        let dir = std::env::temp_dir();
        let (a, b) = (
            dir.join("nodalis_test_recent_a.ndl"),
            dir.join("nodalis_test_recent_b.ndl"),
        );
        std::fs::write(&a, "").unwrap();
        std::fs::write(&b, "").unwrap();
        let gone = dir.join("nodalis_test_recent_gone.ndl");
        let list = with_recent(&with_recent(&[], &a), &b);
        assert_eq!(list, [b.clone(), a.clone()]);
        // Opening one again moves it to the front and does not list it twice.
        assert_eq!(with_recent(&list, &a), [a.clone(), b.clone()]);
        let many: Vec<PathBuf> = (0..20).map(|i| dir.join(format!("r{i}.ndl"))).collect();
        assert_eq!(with_recent(&many, &a).len(), RECENT_MAX);
        let mut storage = MemoryStorage::default();
        eframe::set_value(&mut storage, RECENT_KEY, &vec![gone, a.clone(), b.clone()]);
        assert_eq!(stored_recent(&storage), [a.clone(), b.clone()]);
        assert!(stored_recent(&MemoryStorage::default()).is_empty());
        std::fs::remove_file(&a).unwrap();
        std::fs::remove_file(&b).unwrap();
    }
}
