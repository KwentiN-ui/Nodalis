//! Project files (`.ndl`): the analyses of a project, each with its geometry, mesh, FE model
//! in RON, written by serde, and the results the user bound to it.

use std::collections::BTreeMap;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use base64::Engine;
use base64::engine::general_purpose::STANDARD as BASE64;
use flate2::Compression;
use flate2::read::DeflateDecoder;
use flate2::write::DeflateEncoder;
use plx_mesh::FeMesh;
use plx_model::{FeModel, Geometry, PROJECT_FORMAT};
use plx_results::ResultsSettings;
use serde::{Deserialize, Serialize};

/// A nodalis project as saved in a `.ndl` file: its analyses, like the systems of an Ansys
/// Workbench project, and the blocks that give them data. It stores
/// what the user defined, not the CalculiX input files.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Project {
    pub format: u32,
    /// The blocks of the project schematic: analyses and the sources of data they take.
    #[serde(default)]
    pub nodes: Vec<ProjectNode>,
    /// Data one block takes from another.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub links: Vec<Link>,
    /// Named frames around groups of blocks, drawn behind them.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub frames: Vec<Frame>,
    /// The blocks shown compact: only their header and the rows with a wire.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub compact: Vec<u64>,
}

impl Default for Project {
    fn default() -> Self {
        Self {
            format: PROJECT_FORMAT,
            nodes: Vec::new(),
            links: Vec::new(),
            frames: Vec::new(),
            compact: Vec::new(),
        }
    }
}

impl Project {
    /// The analyses among the blocks.
    pub fn analyses(&self) -> impl Iterator<Item = &ProjectAnalysis> {
        self.nodes.iter().filter_map(|node| match node {
            ProjectNode::Analysis(analysis) => Some(&**analysis),
            _ => None,
        })
    }

    pub fn analyses_mut(&mut self) -> impl Iterator<Item = &mut ProjectAnalysis> {
        self.nodes.iter_mut().filter_map(|node| match node {
            ProjectNode::Analysis(analysis) => Some(&mut **analysis),
            _ => None,
        })
    }
}

/// A block of the project schematic: an analysis, or a source of data that analyses link
/// to, which has no steps and is not solved.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ProjectNode {
    Analysis(Box<ProjectAnalysis>),
    /// A collection of materials kept in the project, unlike the user's material library.
    Materials(MaterialSet),
    /// A collection of beam cross-sections kept in the project.
    CrossSections(CrossSectionSet),
    /// A mesh made from a few parameters, such as a box.
    Mesh(Box<MeshBlock>),
    /// The mesh linked to it, made again finer or coarser by a factor.
    RefineMesh(Box<RefineMesh>),
    /// The mesh of an analysis moved by the displacements of a step, times a factor: e.g. a
    /// buckling mode as the imperfection of a nonlinear analysis.
    DeformedMesh(Box<DeformedMesh>),
    /// A fillet or chamfer of the geometry linked to it.
    CadOperation(Box<CadOperationBlock>),
    /// Named numbers that value links give to other blocks.
    Parameters(ParameterSet),
    /// A key figure computed from the results of a step.
    Measure(MeasureBlock),
    /// A table of design points, each a set of values it gives to other blocks and the
    /// numbers it read back once they were solved.
    Study(StudyBlock),
    /// A sink that writes the mesh it takes to a file.
    MeshExport(MeshExport),
    /// A sweep of an analysis over the rotational speed, with its Campbell diagram.
    Campbell(Box<CampbellBlock>),
    /// A file outside the project read as a source: geometry, a mesh or results.
    File(FileSource),
    /// A text in the schematic, without pins.
    Note(Note),
    /// A number computed from two or three others.
    Math(MathBlock),
    /// One of several numbers, picked by its index.
    Switch(SwitchBlock),
}

impl ProjectNode {
    /// Stays the same when the block is renamed, for links to it.
    pub fn id(&self) -> u64 {
        match self {
            Self::Analysis(a) => a.id,
            Self::Materials(m) => m.id,
            Self::CrossSections(c) => c.id,
            Self::Mesh(m) => m.id,
            Self::RefineMesh(r) => r.id,
            Self::DeformedMesh(d) => d.id,
            Self::CadOperation(c) => c.id,
            Self::Parameters(p) => p.id,
            Self::Measure(m) => m.id,
            Self::Study(o) => o.id,
            Self::MeshExport(e) => e.id,
            Self::Campbell(c) => c.id,
            Self::File(f) => f.id,
            Self::Note(n) => n.id,
            Self::Math(m) => m.id,
            Self::Switch(s) => s.id,
        }
    }

    pub fn name(&self) -> &str {
        match self {
            Self::Analysis(a) => &a.name,
            Self::Materials(m) => &m.name,
            Self::CrossSections(c) => &c.name,
            Self::Mesh(m) => &m.name,
            Self::RefineMesh(r) => &r.name,
            Self::DeformedMesh(d) => &d.name,
            Self::CadOperation(c) => &c.name,
            Self::Parameters(p) => &p.name,
            Self::Measure(m) => &m.name,
            Self::Study(o) => &o.name,
            Self::MeshExport(e) => &e.name,
            Self::Campbell(c) => &c.name,
            Self::File(f) => &f.name,
            Self::Note(n) => &n.name,
            Self::Math(m) => &m.name,
            Self::Switch(s) => &s.name,
        }
    }

    /// Position of the block in the project schematic.
    pub fn position(&self) -> [f32; 2] {
        match self {
            Self::Analysis(a) => a.position,
            Self::Materials(m) => m.position,
            Self::CrossSections(c) => c.position,
            Self::Mesh(m) => m.position,
            Self::RefineMesh(r) => r.position,
            Self::DeformedMesh(d) => d.position,
            Self::CadOperation(c) => c.position,
            Self::Parameters(p) => p.position,
            Self::Measure(m) => m.position,
            Self::Study(o) => o.position,
            Self::MeshExport(e) => e.position,
            Self::Campbell(c) => c.position,
            Self::File(f) => f.position,
            Self::Note(n) => n.position,
            Self::Math(m) => m.position,
            Self::Switch(s) => s.position,
        }
    }
}

/// A block that applies a CAD operation to the geometry linked to it and gives the result.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CadOperationBlock {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    /// The unit system its length is shown and typed in.
    #[serde(default)]
    pub units: plx_model::UnitSystem,
    pub operation: plx_model::CadOperation,
    /// The geometry as last made, kept so that the project opens and meshes without making
    /// it again.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub result: Option<Geometry>,
    /// Fingerprint of the input and the operation the result was made from, see
    /// [`plx_model::CadOperation::fingerprint`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub made_from: Option<u64>,
}

/// A block that makes a structured mesh for analyses to link, with its sides as node sets
/// and surfaces.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MeshBlock {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    /// The model space of the analyses that link it.
    pub space: plx_model::ModelSpace,
    /// The unit system its parameters are shown in; they are stored in SI units.
    #[serde(default)]
    pub units: plx_model::UnitSystem,
    pub primitive: plx_model::MeshPrimitive,
    /// The mesh as last made.
    #[serde(default)]
    pub mesh: FeMesh,
    /// Fingerprint of the parameters the mesh was made from, see
    /// [`plx_model::MeshPrimitive::fingerprint`]; the mesh is out of date once it differs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub generated: Option<u64>,
}

/// A block that makes the mesh linked to it again with a factor on its fineness: the
/// divisions of a mesh block times the factor, or the element sizes of geometry divided by
/// it. A parameter study refines a mesh through its factor.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RefineMesh {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    #[serde(default = "one")]
    pub factor: f64,
    /// The mesh as last made from the divisions of a mesh block; the mesh of geometry is
    /// made by the analysis it goes to, and the link brings it again.
    #[serde(default)]
    pub mesh: FeMesh,
    /// Fingerprint of the divisions and the factor the mesh was made from, see
    /// [`plx_model::MeshPrimitive::fingerprint`]; the mesh is out of date once it differs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub generated: Option<u64>,
}

fn one() -> f64 {
    1.0
}

fn first() -> u32 {
    1
}

/// A block that gives the mesh of the analysis whose step results are linked to it, each
/// node moved by its displacement in one increment or mode of the step times a factor. Its
/// mesh goes on to the Mesh row of another analysis, e.g. a buckling mode as the
/// imperfection of a nonlinear static analysis.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DeformedMesh {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    /// What the displacements are multiplied by.
    #[serde(default = "one")]
    pub factor: f64,
    /// The increment of the step, or its mode for a buckling or frequency step, counted
    /// from 1.
    #[serde(default = "first")]
    pub increment: u32,
    /// The mesh as last made, with the sets and element types of the analysis it comes
    /// from, kept so that the project opens without reading the results again.
    #[serde(default)]
    pub mesh: FeMesh,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub mesh_sets: Vec<plx_model::MeshSet>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub element_types: Vec<plx_model::PartElementType>,
    #[serde(default)]
    pub space: plx_model::ModelSpace,
    /// Fingerprint of the results, the mesh and the numbers the mesh was made from.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub made_from: Option<u64>,
    /// The largest displacement of a node as made, the displacement times the factor.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub largest: Option<f64>,
}

/// A block that computes a key figure from the results of a step another block links to it,
/// such as the largest von Mises stress.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MeasureBlock {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    /// The unit system its value is shown in; it is stored in SI units.
    #[serde(default)]
    pub units: plx_model::UnitSystem,
    pub kind: plx_model::MeasureKind,
    /// The value as last computed, and the fingerprint of the results and the definition it
    /// was computed from: out of date once that differs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub value: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub evaluated: Option<u64>,
}

/// A parameter study: the numbers it gives to other blocks and reads back, as columns of a
/// table of design points, each solved in turn.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct StudyBlock {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    /// The unit system its inputs are shown in; they are stored in SI units.
    #[serde(default)]
    pub units: plx_model::UnitSystem,
    #[serde(default)]
    pub table: plx_model::StudyTable,
}

/// A block of named numbers, which value links give to other blocks: the size of a mesh
/// block, for example, so that several blocks share one value.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ParameterSet {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    /// The unit system its lengths are shown in; they are stored in SI units.
    #[serde(default)]
    pub units: plx_model::UnitSystem,
    pub parameters: Vec<plx_model::Parameter>,
}

/// A block that reads a file outside the project for analyses to link: geometry (STEP,
/// IGES, BREP, STL), a mesh (`.inp`) or the results of each step of a results file (`.frd`).
/// It keeps only the path: the file is read again when the project is opened and whenever
/// it changes.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FileSource {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    /// The file; a relative path is taken from the folder of the project file.
    #[serde(default)]
    pub path: String,
    /// The unit system the numbers of an input file and the lengths of geometry are in.
    #[serde(default)]
    pub units: plx_model::UnitSystem,
}

/// A note in the project schematic: a text for the reader, which gives and takes nothing.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Note {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    #[serde(default)]
    pub text: String,
}

/// A Math block: a number computed from the numbers of its inputs, which value links give or
/// the block holds.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MathBlock {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    /// The unit system the block shows its numbers in.
    #[serde(default)]
    pub units: plx_model::UnitSystem,
    #[serde(default)]
    pub operation: plx_model::MathOp,
    /// What its numbers are; `None` until a link gives it the kind of the other end.
    #[serde(default)]
    pub kind: Option<plx_model::ValueKind>,
    /// The values of its inputs, in SI units.
    #[serde(default)]
    pub values: Vec<f64>,
}

/// A Switch block: gives the number of its choice the index picks, counted from 1.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SwitchBlock {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    #[serde(default)]
    pub units: plx_model::UnitSystem,
    /// What its choices are; `None` until a link gives it the kind of the other end.
    #[serde(default)]
    pub kind: Option<plx_model::ValueKind>,
    #[serde(default)]
    pub index: f64,
    /// The choices, in SI units.
    #[serde(default)]
    pub choices: Vec<f64>,
}

/// A frame of the project schematic: a named, tinted rectangle around the blocks it holds,
/// like a frame of Blender's node editor. It follows its blocks when they move.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    pub label: String,
    /// The tint of the frame, red, green and blue.
    #[serde(default)]
    pub color: [u8; 3],
    /// The ids of the blocks in the frame.
    #[serde(default)]
    pub nodes: Vec<u64>,
}

/// A block that writes the mesh linked to it to a file when the project is updated. It keeps
/// no mesh: the link brings it again when the project is opened.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MeshExport {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    /// The file to write: an input file for CalculiX. A relative path is taken from the
    /// folder of the project file.
    #[serde(default)]
    pub path: String,
    /// Fingerprint of the mesh and the path as last written; nothing is written while it
    /// is the same.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub written: Option<u64>,
}

/// A block that sweeps the analysis linked to it over the rotational speed of its centrifugal
/// loads and keeps the complex modes of each speed for the Campbell diagram, until the
/// analysis changes.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CampbellBlock {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    #[serde(default)]
    pub setup: plx_model::campbell::Setup,
    /// The speeds of the last sweep with their modes, in the order they were computed.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub points: Vec<plx_model::campbell::SpeedPoint>,
    /// Speed range marked for refinement, in radians per time.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub focus: Option<[f64; 2]>,
    /// Fingerprint of the analysis the speeds were computed with, its speed left out: the
    /// diagram is out of date once that differs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fingerprint: Option<u64>,
}

/// A block that holds materials for analyses to link, item by item or all of them.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MaterialSet {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    /// The unit system its values are shown in; they are stored in SI units like those of an
    /// analysis.
    #[serde(default)]
    pub units: plx_model::UnitSystem,
    #[serde(default)]
    pub materials: Vec<plx_model::Material>,
    /// Fields of the materials exposed as parameters, which value links set.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub exposed: Vec<plx_model::Exposed>,
}

/// A block that holds beam cross-sections for analyses to link, item by item or all of them.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CrossSectionSet {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub position: [f32; 2],
    /// The unit system its dimensions are shown in; they are stored in SI units.
    #[serde(default)]
    pub units: plx_model::UnitSystem,
    #[serde(default)]
    pub cross_sections: Vec<plx_model::CrossSection>,
    /// Dimensions of the cross-sections exposed as parameters, which value links set.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub exposed: Vec<plx_model::Exposed>,
}

/// An analysis of a project: what a whole project was before projects held several, with
/// its name and its place in the project schematic.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ProjectAnalysis {
    /// Stays the same when the analysis is renamed, for links to it.
    pub id: u64,
    pub name: String,
    /// Position of its block in the project schematic.
    #[serde(default)]
    pub position: [f32; 2],
    /// CAD geometry the mesh is generated from; analyses made from an input file have none.
    #[serde(default)]
    pub geometry: Option<Geometry>,
    pub mesh: FeMesh,
    #[serde(default)]
    pub model: FeModel,
    /// Fingerprint of the meshing inputs of each part of the geometry when it was meshed,
    /// see [`plx_model::fingerprint::mesh_input`].
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub mesh_inputs: BTreeMap<String, u64>,
    /// Results of a finished run the analysis keeps, restored when it is opened.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub results: Option<BoundResults>,
}

/// Data an analysis takes from another one instead of having its own, like Ansys Workbench's
/// links between cells, but down to single items of the model tree.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Link {
    pub source: u64,
    pub target: u64,
    /// The path of what is taken in the source's block: a cell such as `["Mesh"]`, a
    /// category of the setup such as `["Setup", "Materials"]`, or one item of it such as
    /// `["Setup", "Materials", "Steel"]`; `["Results", step]` is the results of a step.
    pub item: Vec<String>,
    /// Where the target takes it, when that is not the same place: the input of a step that
    /// takes the results of a step, such as `["Setup", "Steps", step, "Temperatures"]`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub slot: Vec<String>,
    /// A state link (`["State", step]`): the steps of the source up to that step the target
    /// took last time, which come before its own steps. A link of a whole category: the
    /// names of the items it took last time.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub inherited: Vec<String>,
    /// A link of a whole category, which is a bundle of links of its items: the items the
    /// target does not take, but keeps its own copies of. New items of the source are taken
    /// as they come.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub excluded: Vec<String>,
}

/// The results of a finished analysis kept in the project file: the input file the solver
/// ran and its results, those of the built-in solver in [`BoundResults::solver`], those of
/// CalculiX as its files (`.frd`, history output in `.dat`, eigenmodes in `.eig`).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BoundResults {
    /// The analysis the results belong to.
    pub analysis: String,
    /// Fingerprint of what the run solved, see [`plx_model::fingerprint::solve_input`];
    /// the results are out of date once the analysis' fingerprint differs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub input: Option<u64>,
    /// Fingerprint of what the results of each step were solved from, see
    /// [`plx_model::fingerprint::step_inputs`]: the results of a step stay up to date while
    /// only later steps change.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub steps: Vec<u64>,
    /// The solver reported errors but wrote results.
    #[serde(default)]
    pub with_errors: bool,
    pub files: Vec<BoundFile>,
    /// The results of the built-in solver, a `.ndlr` file ([`plx_results::SolverResults`]),
    /// read without a `.frd` file; CalculiX's results are among [`BoundResults::files`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub solver: Option<BoundFile>,
    /// What the user defined on the results, restored with them; as of the last save.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub settings: Option<ResultsSettings>,
}

/// A file of [`BoundResults`]: its name and its content, base64 encoded so that it is a
/// string in the project's RON. The results of the built-in solver (`.ndlr`) are compressed
/// already and stored as they are, the other files deflated.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct BoundFile {
    pub name: String,
    /// Size of the file itself.
    pub size: u64,
    pub data: String,
}

impl BoundResults {
    fn all_files(&self) -> impl Iterator<Item = &BoundFile> {
        self.files.iter().chain(&self.solver)
    }

    /// Size of the files as stored in the project.
    pub fn stored_size(&self) -> u64 {
        self.all_files().map(|f| f.data.len() as u64).sum()
    }

    /// Size of the files themselves.
    pub fn size(&self) -> u64 {
        self.all_files().map(|f| f.size).sum()
    }

    /// The results of the built-in solver, read in memory; `None` for results of CalculiX.
    pub fn solver_results(&self) -> Result<Option<plx_results::SolverResults>, ProjectError> {
        let Some(file) = &self.solver else {
            return Ok(None);
        };
        let bytes = unpack(file, Path::new(&file.name))?;
        plx_results::SolverResults::from_bytes(&bytes)
            .map(Some)
            .map_err(|e| ProjectError::Format {
                path: PathBuf::from(&file.name),
                message: e.to_string(),
            })
    }

    /// The text of the bound file with extension `extension`, such as the input file.
    pub fn text(&self, extension: &str) -> Option<String> {
        let file = self.files.iter().find(|f| {
            Path::new(&f.name)
                .extension()
                .is_some_and(|e| e.eq_ignore_ascii_case(extension))
        })?;
        let bytes = unpack(file, Path::new(&file.name)).ok()?;
        Some(String::from_utf8_lossy(&bytes).into_owned())
    }

    /// The name of the results as a CalculiX results file (`.frd`): the bound one, or the
    /// one [`restore_calculix_results`] writes for results of the built-in solver.
    pub fn calculix_results_name(&self) -> Option<String> {
        if let Some(file) = &self.solver {
            return Some(
                Path::new(&file.name)
                    .with_extension("frd")
                    .display()
                    .to_string(),
            );
        }
        (self.files.iter())
            .find(|f| is_frd(Path::new(&f.name)))
            .map(|f| f.name.clone())
    }

    /// Reads the results in memory: those of the built-in solver directly, those of CalculiX
    /// from their files written into `dir` first.
    pub fn read(&self, dir: &Path) -> Result<ReadResults, String> {
        if let Some(solved) = self.solver_results().map_err(|e| e.to_string())? {
            let input = self.text("inp");
            let history = crate::solver::history(&solved, input.as_deref());
            return Ok(ReadResults {
                results: crate::solver::import(&solved),
                history,
                input,
            });
        }
        let frd = restore_results(self, dir).map_err(|e| e.to_string())?;
        let results = crate::frd::read_frd(&frd).map_err(|e| e.to_string())?;
        let history = crate::dat::read_dat(&frd.with_extension("dat")).ok();
        let input = std::fs::read_to_string(frd.with_extension("inp")).ok();
        Ok(ReadResults {
            results,
            history,
            input,
        })
    }
}

/// Bound results read in memory, see [`BoundResults::read`].
pub struct ReadResults {
    pub results: crate::frd::FrdImport,
    pub history: Option<crate::dat::DatImport>,
    /// The input file the solver ran.
    pub input: Option<String>,
}

/// The content of a bound file; `path` names it in errors.
fn unpack(file: &BoundFile, path: &Path) -> Result<Vec<u8>, ProjectError> {
    let format = |message: String| ProjectError::Format {
        path: path.to_owned(),
        message,
    };
    let compressed = BASE64
        .decode(&file.data)
        .map_err(|e| format(format!("bound file {}: {e}", file.name)))?;
    if !deflated(Path::new(&file.name)) {
        return Ok(compressed);
    }
    let mut content = Vec::with_capacity(file.size as usize);
    DeflateDecoder::new(compressed.as_slice())
        .read_to_end(&mut content)
        .map_err(|e| format(format!("bound file {}: {e}", file.name)))?;
    Ok(content)
}

#[derive(Debug, thiserror::Error)]
pub enum ProjectError {
    #[error("{path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("{path}: not a valid project file: {message}")]
    Format { path: PathBuf, message: String },
    #[error("{path} comes from a newer Nodalis version (format {format})")]
    Newer { path: PathBuf, format: u32 },
    #[error(
        "{path} comes from an older Nodalis version (format {format}), which cannot be \
         opened: build the project anew, or import the input files of its analyses"
    )]
    Older { path: PathBuf, format: u32 },
}

/// Writes the project; a crash while saving leaves an existing file intact.
pub fn save_project(path: &Path, project: &Project) -> Result<(), ProjectError> {
    let text = ron::to_string(project).map_err(|e| ProjectError::Format {
        path: path.to_owned(),
        message: e.to_string(),
    })?;
    let io = |source| ProjectError::Io {
        path: path.to_owned(),
        source,
    };
    let temporary = path.with_extension("plx.tmp");
    std::fs::write(&temporary, text).map_err(io)?;
    std::fs::rename(&temporary, path).map_err(io)
}

/// The oldest format that can still be opened. While Nodalis is being designed, a new
/// format is not converted: older projects are refused.
pub const FIRST_PROJECT_FORMAT: u32 = 16;

/// The format of a project file, read before the rest, which depends on it.
#[derive(Deserialize)]
struct Header {
    format: u32,
}

/// The format of a project file as written, `(format:6,...`: the first field. Reading it as a
/// [`Header`] goes through the whole file without types, which takes minutes for a large
/// mesh, as RON looks ahead through the rest of the file at every number.
fn leading_format(text: &str) -> Option<u32> {
    let text = text.trim_start();
    let rest = text
        .strip_prefix("Project")
        .unwrap_or(text)
        .trim_start()
        .strip_prefix('(')?;
    let rest = rest.trim_start().strip_prefix("format")?;
    let rest = rest.trim_start().strip_prefix(':')?.trim_start();
    let end = rest
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}

pub fn read_project(path: &Path) -> Result<Project, ProjectError> {
    let text = std::fs::read_to_string(path).map_err(|source| ProjectError::Io {
        path: path.to_owned(),
        source,
    })?;
    let format_error = |e: ron::error::SpannedError| ProjectError::Format {
        path: path.to_owned(),
        message: e.to_string(),
    };
    let format = match leading_format(&text) {
        Some(format) => format,
        None => ron::from_str::<Header>(&text).map_err(format_error)?.format,
    };
    if format > PROJECT_FORMAT {
        return Err(ProjectError::Newer {
            path: path.to_owned(),
            format,
        });
    }
    if format < FIRST_PROJECT_FORMAT {
        return Err(ProjectError::Older {
            path: path.to_owned(),
            format,
        });
    }
    let mut project: Project = ron::from_str(&text).map_err(format_error)?;
    for analysis in project.analyses_mut() {
        analysis.model.migrate();
    }
    Ok(project)
}

/// Reads the files of a finished analysis into [`BoundResults`]; files that do not exist are
/// left out, the results file (`.frd` of CalculiX or `.ndlr` of the built-in solver) has to
/// be there.
pub fn bind_results(
    analysis: &str,
    files: &[PathBuf],
    with_errors: bool,
) -> Result<BoundResults, ProjectError> {
    let mut bound = BoundResults {
        analysis: analysis.to_owned(),
        input: None,
        steps: Vec::new(),
        with_errors,
        files: Vec::new(),
        solver: None,
        settings: None,
    };
    for path in files {
        let io = |source| ProjectError::Io {
            path: path.to_owned(),
            source,
        };
        let content = match std::fs::read(path) {
            Ok(content) => content,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound && !is_results_file(path) => {
                continue;
            }
            Err(e) => return Err(io(e)),
        };
        let size = content.len() as u64;
        let compressed = if deflated(path) {
            // Level 3: a third of the time of the default level for a few percent more.
            let mut encoder = DeflateEncoder::new(Vec::new(), Compression::new(3));
            encoder.write_all(&content).map_err(io)?;
            encoder.finish().map_err(io)?
        } else {
            content
        };
        let file = BoundFile {
            name: file_name(path).unwrap_or_default(),
            size,
            data: BASE64.encode(compressed),
        };
        if plx_results::solver::is_solver_results(path) {
            bound.solver = Some(file);
        } else {
            bound.files.push(file);
        }
    }
    Ok(bound)
}

/// Writes the files of bound results into `dir`; returns the results file: `.ndlr` for
/// results of the built-in solver, `.frd` for those of CalculiX.
pub fn restore_results(bound: &BoundResults, dir: &Path) -> Result<PathBuf, ProjectError> {
    let format = |message: String| ProjectError::Format {
        path: dir.to_owned(),
        message,
    };
    let io = |path: &Path, source| ProjectError::Io {
        path: path.to_owned(),
        source,
    };
    std::fs::create_dir_all(dir).map_err(|e| io(dir, e))?;
    // Files of results restored here before, which the results read next to their results
    // file: a history output (`.dat`) the bound results do not have would be taken for
    // theirs, a `.frd` of CalculiX for the results of the built-in solver.
    for file in bound
        .all_files()
        .filter(|f| is_results_file(Path::new(&f.name)))
    {
        for extension in ["inp", "dat", "frd", plx_results::solver::EXTENSION] {
            let old = dir.join(Path::new(&file.name).with_extension(extension));
            if old.parent() == Some(dir) && old.is_file() {
                std::fs::remove_file(&old).map_err(|e| io(&old, e))?;
            }
        }
    }
    let mut results = None;
    for file in bound.all_files() {
        // Only a plain file name: a project cannot write elsewhere.
        let name = Path::new(&file.name);
        if file_name(name).as_deref() != Some(file.name.as_str()) {
            return Err(format(format!(
                "bound file name {:?} is not valid",
                file.name
            )));
        }
        let path = dir.join(name);
        let content = unpack(file, &path)?;
        std::fs::write(&path, content).map_err(|e| io(&path, e))?;
        if is_results_file(&path) {
            results = Some(path);
        }
    }
    results.ok_or_else(|| format("the bound results have no results file (.frd)".into()))
}

/// Writes the bound results into `dir` as CalculiX's files and returns the `.frd` file:
/// the bound files of CalculiX's results, or those exported from the results of the
/// built-in solver, for what reads CalculiX's files.
pub fn restore_calculix_results(bound: &BoundResults, dir: &Path) -> Result<PathBuf, ProjectError> {
    let restored = restore_results(bound, dir)?;
    if !plx_results::solver::is_solver_results(&restored) {
        return Ok(restored);
    }
    let format = |message: String| ProjectError::Format {
        path: restored.clone(),
        message,
    };
    let solved = plx_results::SolverResults::read(&restored).map_err(|e| format(e.to_string()))?;
    let frd = restored.with_extension("frd");
    crate::solver::export_calculix(&solved, &frd).map_err(|source| ProjectError::Io {
        path: frd.clone(),
        source,
    })?;
    Ok(frd)
}

/// Writes the bound file named `name`, such as the eigenmodes (`.eig`) of a frequency
/// step, to `path`; false if the bound results have no such file.
pub fn restore_file(bound: &BoundResults, name: &str, path: &Path) -> Result<bool, ProjectError> {
    let Some(file) = bound.files.iter().find(|f| f.name == name) else {
        return Ok(false);
    };
    let content = unpack(file, path)?;
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|source| ProjectError::Io {
            path: dir.to_owned(),
            source,
        })?;
    }
    std::fs::write(path, content).map_err(|source| ProjectError::Io {
        path: path.to_owned(),
        source,
    })?;
    Ok(true)
}

/// Whether a bound file is stored deflated: all but the results of the built-in solver,
/// which their file compresses already (deflating them again takes seconds for large
/// results and saves next to nothing).
fn deflated(path: &Path) -> bool {
    !plx_results::solver::is_solver_results(path)
}

/// Whether a file holds results: `.frd` of CalculiX or `.ndlr` of the built-in solver.
fn is_results_file(path: &Path) -> bool {
    is_frd(path) || plx_results::solver::is_solver_results(path)
}

fn is_frd(path: &Path) -> bool {
    path.extension()
        .is_some_and(|e| e.eq_ignore_ascii_case("frd"))
}

fn file_name(path: &Path) -> Option<String> {
    path.file_name().map(|n| n.to_string_lossy().into_owned())
}

#[cfg(test)]
mod tests {
    use plx_model::{
        Algorithm2d, Algorithm3d, Constraint, ContactPair, Friction, InteractionProperty, Material,
        MeshSetupItem, MeshSetupKind, MeshingParameters, Region, Step, SurfaceBehavior,
        SurfaceInteraction, Tie,
    };

    use super::*;
    use crate::inp::read_inp;

    /// A project with one analysis of this data.
    fn single(
        geometry: Option<&Geometry>,
        mesh: &FeMesh,
        model: &FeModel,
        results: Option<&BoundResults>,
    ) -> Project {
        Project {
            nodes: vec![ProjectNode::Analysis(Box::new(ProjectAnalysis {
                id: 1,
                name: "Analysis-1".into(),
                geometry: geometry.cloned(),
                mesh: mesh.clone(),
                model: model.clone(),
                results: results.cloned(),
                ..Default::default()
            }))],
            ..Default::default()
        }
    }

    /// A block of hexahedra from `origin` with `size` in `n` elements per direction, as a part
    /// of the mesh with node and element ids from `first`; returns its faces on the bottom (z
    /// min, S1) and on the top (S2).
    fn add_block(
        mesh: &mut FeMesh,
        name: &str,
        first: u32,
        origin: [f64; 3],
        size: [f64; 3],
        n: [u32; 3],
    ) -> [Vec<(u32, u8)>; 2] {
        let node = |i: u32, j: u32, k: u32| first + i + (n[0] + 1) * (j + (n[1] + 1) * k);
        for k in 0..=n[2] {
            for j in 0..=n[1] {
                for i in 0..=n[0] {
                    let at = [i, j, k].map(f64::from);
                    let coords = [0, 1, 2].map(|d| origin[d] + size[d] * at[d] / f64::from(n[d]));
                    mesh.set_node(node(i, j, k), coords);
                }
            }
        }
        let (mut elements, mut bottom, mut top) = (Vec::new(), Vec::new(), Vec::new());
        for k in 0..n[2] {
            for j in 0..n[1] {
                for i in 0..n[0] {
                    let id = first + i + n[0] * (j + n[1] * k);
                    let mut nodes = Vec::new();
                    for kk in [k, k + 1] {
                        nodes.extend([
                            node(i, j, kk),
                            node(i + 1, j, kk),
                            node(i + 1, j + 1, kk),
                            node(i, j + 1, kk),
                        ]);
                    }
                    mesh.add_element(plx_mesh::Element {
                        id,
                        type_name: "C3D8".into(),
                        shape: plx_mesh::ElementShape::Hex8,
                        nodes,
                    })
                    .unwrap();
                    elements.push(id);
                    if k == 0 {
                        bottom.push((id, 1));
                    }
                    if k + 1 == n[2] {
                        top.push((id, 2));
                    }
                }
            }
        }
        mesh.parts.push(plx_mesh::Part {
            name: name.into(),
            elements,
        });
        [bottom, top]
    }

    /// Writes a project of a base plate, a block bonded onto it and a cap resting on the
    /// block with friction, for the user guide's screenshots of contact regions:
    /// `PLX_CONTACT_DEMO=path cargo test contact_demo -- --ignored`.
    #[test]
    #[ignore]
    fn contact_demo_project() {
        let Some(path) = std::env::var_os("PLX_CONTACT_DEMO") else {
            return;
        };
        let mut mesh = FeMesh::default();
        let mm = 1e-3;
        let [base_bottom, base_top] = add_block(
            &mut mesh,
            "Base",
            1,
            [0.0, 0.0, 0.0],
            [80.0 * mm, 80.0 * mm, 10.0 * mm],
            [16, 16, 2],
        );
        let [block_bottom, block_top] = add_block(
            &mut mesh,
            "Block",
            10_001,
            [20.0 * mm, 20.0 * mm, 10.0 * mm],
            [40.0 * mm, 40.0 * mm, 20.0 * mm],
            [12, 12, 6],
        );
        let [cap_bottom, cap_top] = add_block(
            &mut mesh,
            "Cap",
            20_001,
            [25.0 * mm, 25.0 * mm, 30.0 * mm],
            [30.0 * mm, 30.0 * mm, 8.0 * mm],
            [6, 6, 2],
        );
        // Only where the block stands on the base.
        let inside = |mesh: &FeMesh, face: &(u32, u8)| {
            let element = mesh.element(face.0).unwrap();
            let x: f64 = (element.nodes.iter())
                .map(|&n| mesh.node(n).unwrap()[0])
                .sum::<f64>()
                / 8.0;
            let y: f64 = (element.nodes.iter())
                .map(|&n| mesh.node(n).unwrap()[1])
                .sum::<f64>()
                / 8.0;
            (20.0 * mm..60.0 * mm).contains(&x) && (20.0 * mm..60.0 * mm).contains(&y)
        };
        let base_top: Vec<_> = base_top.into_iter().filter(|f| inside(&mesh, f)).collect();
        let mut model = FeModel::default();
        model.materials.push(Material {
            name: "Steel".into(),
            density: Some(7850.0),
            elastic: Some(plx_model::Elastic {
                young: 210e9,
                poisson: 0.3,
            }),
            ..Default::default()
        });
        model.sections.push(plx_model::Section {
            name: "Solid_Section-1".into(),
            material: "Steel".into(),
            region: Region::Parts(vec!["Base".into(), "Block".into(), "Cap".into()]),
            thickness: 1.0,
            kind: plx_model::SectionKind::Solid,
        });
        let mut bonded = plx_model::ContactRegion::new(
            "Bonded - Base to Block",
            plx_model::ContactBehavior::Bonded,
        );
        bonded.side_a = Region::Faces(base_top);
        bonded.side_b = Region::Faces(block_bottom);
        let mut frictional = plx_model::ContactRegion::new(
            "Frictional - Block to Cap",
            plx_model::ContactBehavior::Frictional { mu: 0.15 },
        );
        frictional.side_a = Region::Faces(block_top);
        frictional.side_b = Region::Faces(cap_bottom.clone());
        frictional.settings.interface = Some(plx_model::InterfaceTreatment::AdjustToTouch);
        // Without regions, to show the contact detection.
        if std::env::var_os("PLX_CONTACT_DEMO_DETECT").is_none() {
            model.contact_regions = vec![bonded, frictional];
        }
        let mut step = plx_model::Step::new_static("Step-1");
        step.boundary_conditions.push(plx_model::BoundaryCondition {
            distribution: None,
            name: "Fixed-1".into(),
            active: true,
            visible: true,
            region: Region::Faces(base_bottom),
            kind: plx_model::BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        // The cap held sideways at its top, pressed down.
        let cap_nodes = Region::Faces(cap_top.clone()).nodes(&mesh);
        step.boundary_conditions.push(plx_model::BoundaryCondition {
            distribution: None,
            name: "Guide-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(cap_nodes),
            kind: plx_model::BoundaryKind::displacement([
                Some(0.0),
                Some(0.0),
                None,
                None,
                None,
                None,
            ]),
            amplitude: None,
        });
        step.loads.push(plx_model::Load {
            distribution: None,
            name: "Pressure-1".into(),
            active: true,
            visible: true,
            region: Region::Faces(cap_top),
            kind: plx_model::LoadKind::Pressure(5e6),
            amplitude: None,
            factor_amplitude: None,
        });
        // A second step pressing harder, to show the contacts of a step.
        if std::env::var_os("PLX_CONTACT_DEMO_STEPS").is_some() {
            let mut second = step.clone();
            second.name = "Step-2".into();
            second.loads[0].kind = plx_model::LoadKind::Pressure(10e6);
            model.steps.push(step);
            model.steps.push(second);
        } else {
            model.steps.push(step);
        }
        let project = single(None, &mesh, &model, None);
        save_project(std::path::Path::new(&path), &project).unwrap();
    }

    #[test]
    fn the_format_is_read_from_the_start_of_the_file() {
        // Reading it as a Header instead took minutes for a project with a large mesh.
        let project = single(None, &FeMesh::default(), &FeModel::default(), None);
        let path = std::env::temp_dir().join(format!("plx-format-{}.ndl", std::process::id()));
        save_project(&path, &project).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(leading_format(&text), Some(PROJECT_FORMAT));
        assert_eq!(leading_format("Project( format : 3, nodes: [])"), Some(3));
        assert_eq!(leading_format("(nodes: [], format: 3)"), None);
        assert_eq!(leading_format("(format: x)"), None);
    }

    #[test]
    fn projects_read_back_what_was_saved() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let mesh = read_inp(&root.join("testdata/block_c3d20r.inp"))
            .unwrap()
            .mesh;
        let model = FeModel {
            bolts: Vec::new(),
            cross_sections: Vec::new(),
            distributions: Vec::new(),
            properties: Default::default(),
            materials: vec![Material {
                name: "Steel".into(),
                density: Some(7.85e-9),
                elastic: None,
                ..Default::default()
            }],
            sections: Vec::new(),
            steps: vec![
                Step::new_static("Step-1"),
                Step {
                    contact_changes: vec![plx_model::ContactChange {
                        region: "Region-1".into(),
                        active: Some(false),
                        friction: Some(0.3),
                    }],
                    ..Step::new_coupled("Step-2")
                },
            ],
            reference_points: vec![plx_model::ReferencePoint::new("RP-1", [1.0, 2.0, 3.0])],
            coordinate_systems: vec![plx_model::CoordinateSystem {
                kind: plx_model::CoordinateSystemKind::Cylindrical,
                ..plx_model::CoordinateSystem::new("Coordinate_System-1")
            }],
            planes: vec![plx_model::Plane {
                source: plx_model::PlaneSource::ThreePoints {
                    points: [
                        plx_model::PointRef::ReferencePoint("RP-1".into()),
                        plx_model::PointRef::Coordinates([1.0, 0.0, 0.0]),
                        plx_model::PointRef::Coordinates([0.0, 1.0, 0.0]),
                    ],
                },
                ..plx_model::Plane::new("Plane-1")
            }],
            result_planes: vec![plx_model::ResultPlane {
                name: "Plane_Result-1".into(),
                plane: "Plane-1".into(),
            }],
            result_paths: vec![plx_model::ResultPath {
                start: plx_model::PointRef::ReferencePoint("RP-1".into()),
                ..plx_model::ResultPath::new("Path-1")
            }],
            mesh_sets: vec![plx_model::MeshSet::new(
                "Corner",
                plx_model::SetKind::Nodes,
                Region::Nodes(vec![1]),
            )],
            element_types: vec![plx_model::PartElementType {
                part: "EALL".into(),
                type_name: "C3D20".into(),
            }],
            exposed: vec![plx_model::Exposed {
                key: 1,
                name: "Steel density".into(),
                path: plx_model::ValuePath::new(
                    plx_model::ItemRef::Material("Steel".into()),
                    plx_model::Field::Density,
                ),
            }],
            constraints: Vec::new(),
            ties: vec![Tie {
                master: Region::Faces(vec![(1, 2)]),
                slave: Region::Surface("TOP".into()),
                ..Tie::new("Tie-1")
            }],
            node_ties: vec![plx_model::NodeTie {
                region: Region::Nodes(vec![1, 2]),
                rotations: false,
                ..plx_model::NodeTie::new("Node_Tie-1")
            }],
            surface_interactions: vec![SurfaceInteraction {
                name: "Surface_Interaction-1".into(),
                properties: vec![
                    InteractionProperty::SurfaceBehavior(SurfaceBehavior::Tabular(vec![
                        [0.0, 0.0],
                        [1e5, 1.0],
                    ])),
                    InteractionProperty::Friction(Friction::default()),
                ],
            }],
            contact_pairs: vec![ContactPair {
                adjustment_size: Some(0.01),
                clearance: None,
                master_color: [255, 0, 0],
                ..ContactPair::new("Contact_Pair-1", "Surface_Interaction-1")
            }],
            contact_regions: vec![plx_model::ContactRegion {
                side_a: Region::Parts(vec!["A".into()]),
                side_b: Region::Faces(vec![(1, 2)]),
                settings: plx_model::ContactSettings {
                    normal_stiffness: Some(plx_model::Stiffness::Factor(20.0)),
                    sides: Some(plx_model::SideRoles::BIsMaster),
                    tolerance: Some(1e-4),
                    ..Default::default()
                },
                detected_by: Some("Contacts".into()),
                edited: true,
                ..plx_model::ContactRegion::new(
                    "Frictional - A to B",
                    plx_model::ContactBehavior::Frictional { mu: 0.2 },
                )
            }],
            contact_detections: vec![plx_model::ContactDetection {
                parts: vec!["A".into()],
                tolerance: Some(2e-4),
                grouping: plx_model::DetectionGrouping::Graph,
                behavior: plx_model::ContactBehavior::Rough,
                ..plx_model::ContactDetection::new("Contacts")
            }],
            contact_defaults: plx_model::ContactDefaults {
                formulation: Some(plx_model::ContactMethod::Mortar),
                stiffness_factor: 5.0,
                weak_springs: false,
            },
            amplitudes: vec![plx_model::Amplitude {
                time_span: plx_model::AmplitudeTime::Total,
                points: vec![[0.0, 0.0], [1.0, 2.5]],
                ..plx_model::Amplitude::new("Amplitude-1")
            }],
            initial_conditions: vec![plx_model::InitialCondition {
                distribution: None,
                name: "Initial_Temperature-1".into(),
                active: true,
                region: Region::Parts(vec!["A".into()]),
                kind: plx_model::InitialConditionKind::Temperature(20.0),
            }],
        };
        let dir = std::env::temp_dir().join(format!("plx-project-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("block.ndl");
        let geometry = Geometry {
            source: "block.step".into(),
            brep: "DBRep_DrawableShape\n\"quoted\" lines\n".into(),
            meshing: MeshingParameters::for_diagonal(100.0),
            mesh_items: vec![
                MeshSetupItem {
                    name: "Local_Mesh_Size-1".into(),
                    kind: MeshSetupKind::LocalMeshSize {
                        faces: vec![3, 4],
                        edges: vec![7],
                        size: 0.5,
                    },
                },
                MeshSetupItem {
                    name: "Tetrahedral_Gmsh-1".into(),
                    kind: MeshSetupKind::TetrahedralGmsh {
                        parts: vec!["SOLID-1".into()],
                        algorithm_2d: Algorithm2d::Delaunay,
                        algorithm_3d: Algorithm3d::Hxt,
                    },
                },
                MeshSetupItem {
                    name: "Extrude_Mesh-1".into(),
                    kind: MeshSetupKind::ExtrudeMesh(plx_model::ExtrudeMesh {
                        parts: vec!["SOLID-1".into()],
                        faces: vec![2],
                        layers: plx_model::ExtrudeLayers::MultiLayered {
                            sizes: vec![0.2, 0.8],
                            elements: vec![1, 3],
                        },
                        ..Default::default()
                    }),
                },
            ],
            part_names: vec!["SOLID-1".into()],
            refinement: 1.0,
            signatures: Vec::new(),
        };
        let mut saved = single(Some(&geometry), &mesh, &model, None);
        (saved.analyses_mut().next().unwrap())
            .mesh_inputs
            .insert("SOLID-1".into(), 42);
        // A second, empty analysis next to it.
        saved
            .nodes
            .push(ProjectNode::Analysis(Box::new(ProjectAnalysis {
                id: 2,
                name: "Modal".into(),
                position: [300.0, 20.0],
                ..Default::default()
            })));
        saved.links.push(Link {
            source: 1,
            target: 2,
            item: vec!["Setup".into(), "Materials".into(), "Steel".into()],
            ..Default::default()
        });
        saved.links.push(Link {
            source: 1,
            target: 2,
            item: vec!["Setup".into(), "Sections".into()],
            inherited: vec!["Solid".into(), "Shell".into()],
            excluded: vec!["Beam".into()],
            ..Default::default()
        });
        saved.links.push(Link {
            source: 1,
            target: 2,
            item: vec!["Results".into(), "Step-1".into()],
            slot: ["Setup", "Steps", "Step-2", "Global results"]
                .map(Into::into)
                .to_vec(),
            ..Default::default()
        });
        saved.links.push(Link {
            source: 1,
            target: 2,
            item: vec!["State".into(), "Step-1".into()],
            slot: vec!["State".into()],
            inherited: vec!["Step-1".into()],
            ..Default::default()
        });
        save_project(&path, &saved).unwrap();
        let project = read_project(&path).unwrap();
        assert_eq!(project.links, saved.links);
        let mut analyses: Vec<ProjectAnalysis> = project.analyses().cloned().collect();
        let second = analyses.pop().unwrap();
        assert_eq!((second.id, second.name.as_str()), (2, "Modal"));
        assert_eq!(second.position, [300.0, 20.0]);
        assert_eq!(second.mesh.element_count(), 0);
        let project = analyses.pop().unwrap();
        assert_eq!(project.model, model);
        assert_eq!(project.geometry, Some(geometry));
        assert_eq!(project.mesh.elements(), mesh.elements());
        assert_eq!(project.mesh.coords(), mesh.coords());
        assert_eq!(project.mesh.node_sets, mesh.node_sets);
        assert_eq!(project.mesh_inputs["SOLID-1"], 42);

        let newer = std::fs::read_to_string(&path).unwrap().replacen(
            &format!("format:{PROJECT_FORMAT}"),
            "format:99",
            1,
        );
        std::fs::write(&path, newer).unwrap();
        assert!(matches!(
            read_project(&path),
            Err(ProjectError::Newer { format: 99, .. })
        ));
        // Projects with a single model, from before analyses, are not read.
        let older = "(format:1,geometry:None,mesh:(),model:())";
        std::fs::write(&path, older).unwrap();
        assert!(matches!(
            read_project(&path),
            Err(ProjectError::Older { format: 1, .. })
        ));
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn cross_section_blocks_read_back() {
        let dir = std::env::temp_dir().join(format!("plx-cross-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("cross.ndl");
        let set = CrossSectionSet {
            id: 4,
            name: "Cross-Sections".into(),
            position: [10.0, 200.0],
            units: plx_model::UnitSystem::MmTonSC,
            cross_sections: vec![plx_model::CrossSection {
                name: "Tube".into(),
                profile: plx_model::BeamProfile::Pipe {
                    radius: 0.02,
                    thickness: 0.002,
                },
            }],
            exposed: vec![plx_model::Exposed {
                key: 1,
                name: "Tube Radius".into(),
                path: plx_model::ValuePath::new(
                    plx_model::ItemRef::CrossSection("Tube".into()),
                    plx_model::Field::Radius,
                ),
            }],
        };
        let mut saved = single(None, &FeMesh::default(), &FeModel::default(), None);
        saved.nodes.push(ProjectNode::CrossSections(set.clone()));
        save_project(&path, &saved).unwrap();
        let project = read_project(&path).unwrap();
        match &project.nodes[1] {
            ProjectNode::CrossSections(read) => assert_eq!(read, &set),
            other => panic!("{other:?}"),
        }
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn source_blocks_read_back_and_older_projects_are_refused() {
        let dir = std::env::temp_dir().join(format!("plx-nodes-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("nodes.ndl");
        let set = MaterialSet {
            id: 3,
            name: "Materials".into(),
            position: [10.0, 400.0],
            units: plx_model::UnitSystem::MKgSC,
            materials: vec![Material {
                name: "Steel".into(),
                density: Some(7850.0),
                ..Default::default()
            }],
            exposed: vec![plx_model::Exposed {
                key: 1,
                name: "Steel density".into(),
                path: plx_model::ValuePath::new(
                    plx_model::ItemRef::Material("Steel".into()),
                    plx_model::Field::Density,
                ),
            }],
        };
        let mut saved = single(None, &FeMesh::default(), &FeModel::default(), None);
        saved.nodes.push(ProjectNode::Materials(set.clone()));
        let primitive = plx_model::MeshPrimitive::default_box();
        let space = plx_model::ModelSpace::ThreeD;
        saved.nodes.push(ProjectNode::Mesh(Box::new(MeshBlock {
            id: 5,
            name: "Box".into(),
            position: [0.0, 0.0],
            space,
            units: plx_model::UnitSystem::MmTonSC,
            mesh: primitive.generate(space, "Box"),
            generated: Some(primitive.fingerprint(space)),
            primitive: primitive.clone(),
        })));
        let refined = primitive.refined(1.5);
        saved
            .nodes
            .push(ProjectNode::RefineMesh(Box::new(RefineMesh {
                id: 10,
                name: "Refine Mesh".into(),
                position: [100.0, 0.0],
                factor: 1.5,
                mesh: refined.generate(space, "Box"),
                generated: Some(refined.fingerprint(space)),
            })));
        let parameters = ParameterSet {
            id: 6,
            name: "Parameters".into(),
            position: [0.0, 300.0],
            units: plx_model::UnitSystem::MmTonSC,
            parameters: vec![plx_model::Parameter::new(
                "length",
                plx_model::ValueKind::LENGTH,
                0.25,
            )],
        };
        saved
            .nodes
            .push(ProjectNode::Parameters(parameters.clone()));
        saved.links.push(Link {
            source: 6,
            target: 5,
            item: vec!["Value".into(), "length".into()],
            slot: vec!["Parameters".into(), "lx".into()],
            ..Default::default()
        });
        let export = MeshExport {
            id: 7,
            name: "Mesh Export".into(),
            position: [300.0, 20.0],
            path: "out/box.inp".into(),
            written: Some(42),
        };
        saved.nodes.push(ProjectNode::MeshExport(export.clone()));
        let campbell = CampbellBlock {
            id: 8,
            name: "Campbell".into(),
            position: [400.0, 20.0],
            setup: plx_model::campbell::Setup {
                max_speed: 1000.0,
                ..Default::default()
            },
            points: vec![plx_model::campbell::SpeedPoint {
                speed: 500.0,
                modes: vec![plx_model::campbell::Mode {
                    frequency: 12.5,
                    whirl: plx_model::campbell::Whirl::Backward,
                    unstable: false,
                }],
            }],
            focus: Some([100.0, 200.0]),
            fingerprint: Some(7),
        };
        saved
            .nodes
            .push(ProjectNode::Campbell(Box::new(campbell.clone())));
        let file = FileSource {
            id: 9,
            name: "Global results".into(),
            position: [500.0, 20.0],
            path: "results/global.frd".into(),
            units: plx_model::UnitSystem::MmTonSC,
        };
        saved.nodes.push(ProjectNode::File(file.clone()));
        let note = Note {
            id: 10,
            name: "Note".into(),
            position: [600.0, 20.0],
            text: "Loads from the test bench".into(),
        };
        saved.nodes.push(ProjectNode::Note(note.clone()));
        let math = MathBlock {
            id: 11,
            name: "Math".into(),
            position: [700.0, 20.0],
            units: plx_model::UnitSystem::MmTonSC,
            operation: plx_model::MathOp::Multiply,
            kind: Some(plx_model::ValueKind::LENGTH),
            values: vec![0.1, 2.0],
        };
        saved.nodes.push(ProjectNode::Math(math.clone()));
        let switch = SwitchBlock {
            id: 12,
            name: "Switch".into(),
            position: [800.0, 20.0],
            units: plx_model::UnitSystem::MmTonSC,
            kind: None,
            index: 2.0,
            choices: vec![1.0, 2.0, 3.0],
        };
        saved.nodes.push(ProjectNode::Switch(switch.clone()));
        saved.frames.push(Frame {
            label: "Sources".into(),
            color: [96, 172, 96],
            nodes: vec![3, 9],
        });
        saved.compact = vec![1, 9];
        saved.links.push(Link {
            source: 3,
            target: 1,
            item: vec!["Setup".into(), "Materials".into()],
            slot: Vec::new(),
            inherited: Vec::new(),
            excluded: Vec::new(),
        });
        save_project(&path, &saved).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.contains(&format!("format:{PROJECT_FORMAT}")));
        assert!(!text.contains("analyses:"), "{text}");
        let project = read_project(&path).unwrap();
        assert_eq!(project.nodes.len(), 11);
        match (&project.nodes[9], &project.nodes[10]) {
            (ProjectNode::Math(a), ProjectNode::Switch(b)) => {
                assert_eq!(a, &math);
                assert_eq!(b, &switch);
            }
            other => panic!("{other:?}"),
        }
        match &project.nodes[7] {
            ProjectNode::File(read) => assert_eq!(read, &file),
            other => panic!("{other:?}"),
        }
        match &project.nodes[8] {
            ProjectNode::Note(read) => assert_eq!(read, &note),
            other => panic!("{other:?}"),
        }
        assert_eq!(project.frames, saved.frames);
        assert_eq!(project.compact, vec![1, 9]);
        match &project.nodes[5] {
            ProjectNode::MeshExport(read) => assert_eq!(read, &export),
            other => panic!("{other:?}"),
        }
        match &project.nodes[6] {
            ProjectNode::Campbell(read) => assert_eq!(**read, campbell),
            other => panic!("{other:?}"),
        }
        match &project.nodes[4] {
            ProjectNode::Parameters(read) => assert_eq!(read, &parameters),
            other => panic!("{other:?}"),
        }
        match &project.nodes[3] {
            ProjectNode::RefineMesh(block) => {
                assert_eq!(block.factor, 1.5);
                assert_eq!(block.generated, Some(refined.fingerprint(space)));
                assert!(block.mesh.element_count() > 40);
            }
            other => panic!("{other:?}"),
        }
        match &project.nodes[2] {
            ProjectNode::Mesh(block) => {
                assert_eq!(block.primitive, primitive);
                assert_eq!(block.generated, Some(primitive.fingerprint(space)));
                assert_eq!(block.mesh.element_count(), 40);
                assert_eq!(block.mesh.surfaces.len(), 6);
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(
            (project.nodes[0].id(), project.nodes[0].name()),
            (1, "Analysis-1")
        );
        match &project.nodes[1] {
            ProjectNode::Materials(read) => assert_eq!(read, &set),
            other => panic!("{other:?}"),
        }
        assert_eq!(project.links, saved.links);

        // A project of an older format is refused, not converted.
        let older = "(format:5,nodes:[],links:[])";
        std::fs::write(&path, older).unwrap();
        assert!(matches!(
            read_project(&path),
            Err(ProjectError::Older { format: 5, .. })
        ));
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn steps_of_older_projects_are_active() {
        let mut step = Step::new_static("Step-1");
        step.active = false;
        let text = ron::to_string(&step).unwrap();
        assert!(text.contains("active:false,"), "{text}");
        let older: Step = ron::from_str(&text.replace("active:false,", "")).unwrap();
        assert!(older.active);
    }

    #[test]
    fn ties_of_older_projects_move_to_the_contact_pairs() {
        // Ties and node ties were constraints; a project saved then still opens with them.
        let mut tie = plx_model::NodeTie::new("Node_Tie-1");
        tie.region = Region::Nodes(vec![3, 7]);
        let older = FeModel {
            constraints: vec![
                Constraint::Tie(Tie::new("Tie-1")),
                Constraint::NodeTie(tie.clone()),
            ],
            ..FeModel::default()
        };
        let mut model: FeModel = ron::from_str(&ron::to_string(&older).unwrap()).unwrap();
        model.migrate();
        assert_eq!(model.constraints, []);
        assert_eq!(model.ties, [Tie::new("Tie-1")]);
        assert_eq!(model.node_ties, [tie]);
    }

    #[test]
    fn hot_spots_of_older_projects_are_skipped() {
        // Hot spots were part of the FE model for a while; they are defined on the results
        // now, and projects saved with them still open.
        let text = ron::to_string(&FeModel::default()).unwrap();
        let hot_spots = "hot_spots:[(name:\"Hot_Spot-1\",toe:Nodes([3,7]),\
            direction:(1.0,0.0,0.0),thickness:10.0,extrapolation:Custom([2.0,6.0]),\
            component:Perpendicular)],";
        let older = text.replacen(
            "reference_points:",
            &format!("{hot_spots}reference_points:"),
            1,
        );
        assert_ne!(older, text);
        let model: FeModel = ron::from_str(&older).unwrap();
        assert_eq!(model, FeModel::default());
    }

    #[test]
    fn path_plots_saved_before_hot_spot_plots_still_load() {
        let older = "(path:\"Path-1\",offset:None,size:(400.0,240.0),on_path:false,\
            height:80.0,title:\"\",x_label:\"Distance\",y_label:\"\",y_range:None)";
        let plot: plx_results::saved_view::SavedPathPlot = ron::from_str(older).unwrap();
        assert!(!plot.hot_spot);
        let saved = ron::to_string(&plot).unwrap();
        assert_eq!(
            ron::from_str::<plx_results::saved_view::SavedPathPlot>(&saved).unwrap(),
            plot
        );
    }

    #[test]
    fn table_plots_keep_missing_values() {
        let plot = plx_results::saved_view::SavedPathPlot {
            path: "SP_A.STRESSES.MISES".into(),
            offset: Some([20.0, 30.0]),
            size: [400.0, 240.0],
            on_path: false,
            height: 80.0,
            title: String::new(),
            x_label: "Increment".into(),
            y_label: String::new(),
            y_range: None,
            hot_spot: false,
            table: Some(plx_results::saved_view::SavedTablePlot {
                x_label: "Increment".into(),
                x: vec![1.0, 2.0, 3.0],
                curves: vec![("12_1".into(), vec![0.5, f64::NAN, 1.5])],
            }),
        };
        let saved = ron::to_string(&plot).unwrap();
        let loaded: plx_results::saved_view::SavedPathPlot = ron::from_str(&saved).unwrap();
        let table = loaded.table.unwrap();
        assert_eq!(table.x, [1.0, 2.0, 3.0]);
        let values = &table.curves[0].1;
        assert!(values[0] == 0.5 && values[1].is_nan() && values[2] == 1.5);
    }

    #[test]
    fn bound_results_are_restored_from_the_project() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let mesh = read_inp(&root.join("testdata/block_c3d20r.inp"))
            .unwrap()
            .mesh;
        let dir = std::env::temp_dir().join(format!("plx-bound-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let frd = dir.join("Analysis-1.frd");
        let text = "    1C\n".repeat(1000) + " 9999\n";
        std::fs::write(&frd, &text).unwrap();
        // No history output: the missing .dat is left out.
        let files = [frd.with_extension("dat"), frd.clone()];
        let bound = bind_results("Analysis-1", &files, false).unwrap();
        assert_eq!(bound.files.len(), 1);
        assert_eq!(bound.size(), text.len() as u64);
        assert!(
            bound.stored_size() < bound.size() / 10,
            "{}",
            bound.stored_size()
        );
        assert!(bind_results("Analysis-1", &[dir.join("missing.frd")], false).is_err());

        let path = dir.join("block.ndl");
        let model = FeModel::default();
        let mut bound = bound;
        bound.settings = Some(ResultsSettings {
            result_paths: vec![plx_model::ResultPath::new("Path-1")],
            transformations: vec![plx_results::Transformation::symmetry(
                plx_results::transformation::SymmetryPlane::Y,
            )],
            shown: Some(("STRESS".into(), "MISES".into())),
            // The colour scale the results were shown with.
            view: plx_results::saved_view::SavedView {
                color_scale: Some(plx_model::ColorScale {
                    spectrum: plx_model::ColorSpectrum::Viridis,
                    manual: Some([-1.5, 250.0]),
                    ..Default::default()
                }),
                ..Default::default()
            },
            ..Default::default()
        });
        bound.input = Some(7);
        save_project(&path, &single(None, &mesh, &model, Some(&bound))).unwrap();
        let project = read_project(&path).unwrap();
        let analysis = project.analyses().next().unwrap();
        assert_eq!(analysis.results.as_ref(), Some(&bound));

        // History output restored there before belongs to other results.
        std::fs::create_dir_all(dir.join("restored")).unwrap();
        std::fs::write(dir.join("restored/Analysis-1.dat"), "old").unwrap();
        let restored = restore_results(&bound, &dir.join("restored")).unwrap();
        assert_eq!(restored, dir.join("restored/Analysis-1.frd"));
        assert_eq!(std::fs::read_to_string(&restored).unwrap(), text);
        assert!(!dir.join("restored/Analysis-1.dat").exists());

        // Without bound results the project file has no trace of them.
        save_project(&path, &single(None, &mesh, &model, None)).unwrap();
        assert!(!std::fs::read_to_string(&path).unwrap().contains("files:"));
        let project = read_project(&path).unwrap();
        assert_eq!(project.analyses().next().unwrap().results, None);

        let mut evil = bound.clone();
        evil.files[0].name = "../evil.frd".into();
        assert!(restore_results(&evil, &dir.join("restored")).is_err());
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn results_of_the_built_in_solver_are_bound_as_their_file() {
        let dir = std::env::temp_dir().join(format!("plx-bound-ndlr-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut solved = plx_results::SolverResults::default();
        solved.version = "test".into();
        solved.model.nodes.insert(4, [1.0, 2.0, 3.0]);
        let path = dir.join("Analysis-1.ndlr");
        solved.write(&path).unwrap();
        let bound = bind_results("Analysis-1", &[path.clone()], false).unwrap();
        let file = bound.solver.as_ref().unwrap();
        // Compressed by its own format already: stored as it is, not deflated again.
        let written = std::fs::read(&path).unwrap();
        assert_eq!(BASE64.decode(&file.data).unwrap(), written);
        assert_eq!(bound.solver_results().unwrap().unwrap(), solved);
        let restored = restore_results(&bound, &dir.join("restored")).unwrap();
        assert_eq!(std::fs::read(restored).unwrap(), written);
        std::fs::remove_dir_all(dir).unwrap();
    }
}
