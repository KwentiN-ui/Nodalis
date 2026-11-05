use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use glam::{DAffine3, DVec3, Vec3};
use plx_io::frd::{FrdImport, read_frd};
use plx_io::inp::{InpImport, read_inp};
use plx_mesh::{
    ElementFamily, ElementId, ElementShape, FeMesh, NodeId, PartSkin, extract_part_skin,
};
use plx_mesher::{CadEntity, GeometryDisplay};
#[cfg(test)]
use plx_model::SectionKind;
use plx_model::{BeamOrientation, FeModel, Finding, Geometry, MeshCheck, UnitSystem};
use plx_render::{
    BeamShape, ClipPlane, RenderMesh, SectionCells, SectionValues, ShellThickness, Vertex,
    beam_mesh, lighten, mesh_wireframe, part_color, part_render_mesh, section_mesh, section_values,
    thick_shell_mesh, wireframe_edges,
};

use crate::exploded::{Assembly, Explosion, Parameters, PartShape};
use crate::results::ResultsView;

/// Angle between neighbouring faces above which their common edge counts as a feature edge
/// and the shading across it stays sharp.
const FEATURE_ANGLE_DEG: f64 = 30.0;
/// Within a surface patch, coarse meshes of curved surfaces can fold more than the feature
/// angle between neighbouring faces; shading still blends across such folds.
const SMOOTH_ANGLE_DEG: f64 = 60.0;

/// Extension of nodalis project files.
pub const PROJECT_EXTENSION: &str = "ndl";

/// Whether a file is a nodalis project, by its extension.
pub fn is_project_file(path: &Path) -> bool {
    path.extension()
        .is_some_and(|e| e.eq_ignore_ascii_case(PROJECT_EXTENSION))
}

/// Colour of selected faces and nodes.
pub const HIGHLIGHT_COLOR: [f32; 3] = [1.0, 0.0, 0.0];

/// Colour of the second region of an item, such as the slave surface of a contact pair:
/// The secondary highlight colour, violet.
pub const SECONDARY_HIGHLIGHT_COLOR: [f32; 3] = [238.0 / 255.0, 130.0 / 255.0, 238.0 / 255.0];

/// Summary of one part, computed once on load so the GUI never iterates large meshes.
pub struct PartInfo {
    pub name: String,
    pub color: [f32; 3],
    pub element_count: usize,
    pub node_count: usize,
    pub element_types: Vec<(String, usize)>,
    /// Bounding box relative to [`Model::origin`].
    pub bounds: Option<(Vec3, Vec3)>,
    pub visible: bool,
    /// Whether the part takes part in the colour scale of results (colour
    /// contours on); a part without them is plain grey and leaves the legend range alone.
    pub contours: bool,
    /// Dimension of the CAD shape of a geometry part: 3 solid, 2 face, 1 edge.
    pub dimension: Option<u8>,
}

/// A loaded input file with everything the GUI shows about it.
pub struct Model {
    pub path: PathBuf,
    pub mesh: FeMesh,
    pub parts: Vec<PartInfo>,
    pub warnings: Vec<String>,
    /// What was changed on reading, printed as notes in the output.
    pub notes: Vec<String>,
    pub skipped_keywords: BTreeMap<String, usize>,
    pub included_files: usize,
    pub load_time: Duration,
    /// Results read from an `.frd` file, with what the user currently looks at. A model
    /// with results is a results file: it lives in the Results workspace and has no FE model
    /// to set up.
    pub results: Option<ResultsView>,
    /// The analysis set up on this mesh.
    pub fe: FeModel,
    /// CAD geometry the mesh is generated from.
    pub geometry: Option<Geometry>,
    /// Results of the analysis bound to the project, saved with it.
    pub bound_results: Option<plx_io::project::BoundResults>,
    /// Fingerprint of the meshing inputs of each part of the geometry when it was last
    /// meshed; a part whose inputs differ now has an out-of-date mesh.
    pub mesh_inputs: BTreeMap<String, u64>,
    /// Where the bound results were written to be opened, once they were.
    pub bound_results_file: Option<PathBuf>,
    /// The display of CAD geometry rather than a mesh: element edges are not drawn.
    is_geometry: bool,
    /// For the display of CAD geometry, the CAD face or edge of each element; element ids
    /// count from 1.
    cad_entities: Vec<CadEntity>,
    /// Solids of the CAD geometry a geometry display shows.
    pub geometry_solids: usize,
    /// How the faces, edges and vertices of a geometry display meet.
    geometry_topology: plx_mesher::CadTopology,
    /// Lookups for selections on a geometry display, built when first needed.
    geometry_index: std::sync::OnceLock<crate::geometry_selection::GeometryIndex>,
    /// Hot spots defined on a results file, with their values.
    pub hot_spots: crate::hot_spots::HotSpots,
    /// Keys of the features of these results that come from the FE model, see
    /// [`FeModel::add_missing_features`]; the others were made on the results.
    pub inherited_features: Vec<String>,
    /// Plots of result paths shown in the 3D view.
    pub path_plots: Vec<crate::path_plot::PathPlot>,
    /// Faces and parts drawn in the highlight colour.
    pub highlight: Highlight,
    /// The section view plane in render coordinates; picking ignores what it cuts off.
    pub clip: Option<ClipPlane>,
    /// Centre of the mesh; render positions are relative to it.
    origin: DVec3,
    skins: Vec<PartSkin>,
    /// Elements of each part prepared for section cuts, built when first needed.
    section_cells: std::sync::OnceLock<Vec<SectionCells>>,
    /// The exploded view: parts drawn and picked moved apart.
    pub explosion: Explosion,
    /// What the query tool left in the view; they refer to the mesh by ids, so a new mesh
    /// drops them.
    pub query_annotations: Vec<crate::query::Annotation>,
    /// Offsets of single nodes by index, the motion preview of a boundary condition dialog:
    /// the nodes are drawn moved, the rest of the mesh stays put.
    pub motion: Option<Vec<(usize, [f64; 3])>>,
    /// The whole part a cyclic symmetry tie dialog's sector stands for, drawn as the
    /// wireframe of the other sectors.
    pub cyclic_preview: Option<CyclicPreview>,
    /// Node indices of each part and the part of each node, built when first needed.
    part_nodes: std::sync::OnceLock<PartNodes>,
    /// Nodes and elements of each part's skin, built when first needed.
    skin_lookup: std::sync::OnceLock<SkinLookup>,
    /// What shows each CAD entity of a geometry display, built when first needed.
    cad_display: std::sync::OnceLock<CadDisplay>,
    /// Lookups to the CAD entities of a mesh generated from geometry, built when first
    /// needed.
    cad_index: std::sync::OnceLock<crate::cad_selection::CadIndex>,
    /// The model checks of the last FE model checked, see [`Model::findings`].
    checks: std::sync::Mutex<Option<Checks>>,
    /// What the user changed, to undo and redo it.
    pub history: crate::undo::History,
    /// Counts the changes of the FE model, the meshing settings and the mesh, see
    /// [`crate::undo::record`]; the project brings its links up to date when it changes.
    pub revision: u64,
    /// Shell and membrane sections whose elements are drawn as the solid of their
    /// thickness and offset, see [`Model::shell_sections`]; empty draws them as surfaces.
    pub thick_shells: Vec<plx_model::Section>,
    /// Beam and truss sections whose elements are drawn as their profile swept along the
    /// line, with the profile of a beam's cross-section, see [`Model::line_sections`]; empty
    /// draws them as lines.
    pub thick_lines: Vec<(plx_model::Section, Option<plx_model::BeamProfile>)>,
    /// Results: the shell, membrane, beam and truss sections of the input CalculiX ran.
    input_sections: InputSections,
    /// The global results of a submodel, see [`ResultsView::show_global_contours`].
    pub global: Option<Box<GlobalModel>>,
}

/// The results of the global model of a submodel, whose deformed mesh can be drawn as a
/// wireframe around the submodel's results.
pub struct GlobalModel {
    pub model: Model,
    /// The increment the submodel's boundary follows: the last of the global step it reads.
    pub increment: usize,
}

impl GlobalModel {
    /// The global results `model`; `step` is the global step the submodel reads, counted
    /// from 1.
    pub fn new(model: Model, step: u32) -> Option<Self> {
        let view = model.results.as_ref()?;
        let increment = (view.increments.iter())
            .rposition(|i| i.step == step)
            .or_else(|| view.increments.len().checked_sub(1))?;
        Some(Self { model, increment })
    }
}

/// The sections of the input CalculiX ran, in the units the results show, see
/// [`plx_io::inp::InpImport::sections`], with the element sets they refer to. The sets stay
/// out of the mesh: the results tree would list them.
#[derive(Default)]
struct InputSections {
    sections: Vec<plx_model::Section>,
    cross_sections: Vec<plx_model::CrossSection>,
    sets: BTreeMap<String, Vec<ElementId>>,
}

/// Elements drawn as the solid they stand for, by element index: shells thickened, lines
/// with their profile.
#[derive(Default)]
struct SolidElements {
    shells: HashMap<usize, ShellThickness>,
    lines: HashMap<usize, BeamShape>,
}

/// Findings of the model checks with what they were computed for.
struct Checks {
    mesh: MeshCheck,
    fe: FeModel,
    findings: Vec<Finding>,
    /// The contact regions the bolts replace, with the bolt.
    replaced: std::collections::BTreeMap<usize, usize>,
}

/// Which nodes belong to which part; a node shared by parts belongs to the first of them.
struct PartNodes {
    nodes: Vec<Vec<usize>>,
    part_of: Vec<u32>,
}

impl PartNodes {
    fn new(mesh: &FeMesh) -> Self {
        let mut part_of = vec![u32::MAX; mesh.node_count()];
        let nodes = (mesh.parts.iter().enumerate())
            .map(|(index, part)| {
                let mut nodes: Vec<usize> = (part.elements.iter())
                    .filter_map(|&id| mesh.element(id))
                    .flat_map(|e| e.nodes.iter())
                    .filter_map(|&id| mesh.node_index(id))
                    .collect();
                nodes.sort_unstable();
                nodes.dedup();
                for &node in &nodes {
                    if part_of[node] == u32::MAX {
                        part_of[node] = index as u32;
                    }
                }
                nodes
            })
            .collect();
        Self { nodes, part_of }
    }
}

/// What the skins of the parts are made of, for picking and redrawing single parts.
struct SkinLookup {
    /// The nodes of the faces and lines of each part's skin.
    nodes: Vec<Vec<usize>>,
    /// The part whose skin shows an element, by element index; `u32::MAX` for none.
    element_part: Vec<u32>,
}

impl SkinLookup {
    fn new(skins: &[PartSkin], elements: usize) -> Self {
        let mut element_part = vec![u32::MAX; elements];
        let nodes = (skins.iter().enumerate())
            .map(|(index, skin)| {
                let elements = (skin.faces.iter().map(|f| f.element))
                    .chain(skin.line_elements.iter().copied());
                for element in elements {
                    element_part[element] = index as u32;
                }
                let corners = (skin.faces.iter()).flat_map(|f| &f.corners);
                let nodes = corners.chain(skin.lines.iter().flatten()).copied();
                // Marked in the range of the part's nodes, which is cheaper than sorting.
                let (lo, hi) =
                    (nodes.clone()).fold((usize::MAX, 0), |(lo, hi), n| (lo.min(n), hi.max(n + 1)));
                let mut used = vec![false; hi.saturating_sub(lo)];
                for node in nodes {
                    used[node - lo] = true;
                }
                (used.iter().enumerate())
                    .filter(|(_, used)| **used)
                    .map(|(k, _)| lo + k)
                    .collect()
            })
            .collect();
        Self {
            nodes,
            element_part,
        }
    }
}

/// The skin faces and line segments that show each CAD face and edge of a geometry
/// display, so that highlighting entities doesn't search the whole display every frame.
#[derive(Default)]
struct CadDisplay {
    faces: HashMap<i32, Vec<(ElementId, u8)>>,
    edges: HashMap<i32, Vec<[NodeId; 2]>>,
}

/// What is shown selected: whole parts, element faces and nodes.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Highlight {
    pub parts: HashSet<usize>,
    /// Parts drawn with a red outline only, as a selected part is shown.
    pub outlines: HashSet<usize>,
    /// Element faces as (element, CalculiX face number).
    pub faces: HashSet<(ElementId, u8)>,
    /// Element faces in the secondary highlight colour, e.g. slave surfaces.
    pub secondary_faces: HashSet<(ElementId, u8)>,
    /// Nodes, drawn as points over the scene.
    pub nodes: Vec<NodeId>,
    /// Lines between two nodes drawn over the scene, such as the edges of 2D elements.
    pub lines: Vec<[NodeId; 2]>,
    /// Lines in the secondary highlight colour, e.g. the slave edges of a 2D contact.
    pub secondary_lines: Vec<[NodeId; 2]>,
}

impl Highlight {
    /// Whole parts, such as those selected in the tree or clicked in the 3D view.
    pub fn parts(parts: impl IntoIterator<Item = usize>) -> Self {
        Self {
            outlines: parts.into_iter().collect(),
            ..Self::default()
        }
    }

    /// Adds what `other` highlights.
    pub fn extend(&mut self, other: Highlight) {
        self.parts.extend(other.parts);
        self.outlines.extend(other.outlines);
        self.faces.extend(other.faces);
        self.secondary_faces.extend(other.secondary_faces);
        self.nodes.extend(other.nodes);
        self.lines.extend(other.lines);
        self.secondary_lines.extend(other.secondary_lines);
    }
}

/// A sector repeated about an axis, previewed while its cyclic symmetry tie is edited.
#[derive(Clone, Debug, PartialEq)]
pub struct CyclicPreview {
    pub axis_point: [f64; 3],
    pub axis_direction: [f64; 3],
    pub sectors: u32,
}

/// A visible face or line under the mouse.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    pub part: usize,
    /// Index into the part's skin faces, or into its skin lines when `line` is set.
    pub face: usize,
    /// Hit point relative to the model origin.
    pub point: Vec3,
    /// A segment of a line element (beam, truss, CAD edge) was hit instead of a face.
    pub line: bool,
}

/// Node coordinates as drawn, normalized contour values and whether the shape is deformed.
type ShownState<'a> = (std::borrow::Cow<'a, [[f64; 3]]>, Option<Vec<f32>>, bool);

/// Result of loading on a worker thread: the model plus one GPU-ready mesh per part of what
/// is shown first, the geometry if the model has no mesh yet.
pub struct LoadedModel {
    pub model: Model,
    pub render_meshes: Vec<RenderMesh>,
    /// Display of the model's CAD geometry.
    pub geometry_view: Option<Model>,
}

/// Imports one or several CAD files into one geometry, in the length unit of `units`. The
/// model takes the path of the first file.
pub fn load_geometry(paths: &[PathBuf], units: UnitSystem) -> Result<LoadedModel, String> {
    let start = Instant::now();
    let path = paths.first().ok_or("No file selected")?;
    let import = plx_mesher::import_cad_files(paths, units).map_err(|e| e.to_string())?;
    let mut model = Model::new(path, FeMesh::default());
    model.fe.properties.units = units;
    model.geometry = Some(import.geometry);
    model.warnings = import.warnings;
    model.notes = import.notes;
    let view = Model::geometry_view(path, import.display);
    let render_meshes = view.render_meshes();
    model.load_time = start.elapsed();
    Ok(LoadedModel {
        model,
        render_meshes,
        geometry_view: Some(view),
    })
}

/// The input file CalculiX ran for a results file: named like it, next to it.
fn companion_input(results: &Path) -> Option<PathBuf> {
    ["inp", "INP"]
        .map(|e| results.with_extension(e))
        .into_iter()
        .find(|p| p.is_file())
}

/// Groups the elements of results into the parts of the input file they were computed from,
/// with the part names as the input writes them, and drops the springs and gaps the
/// constraints were written as: the solver makes each gap section a material of its own, which
/// would show as a part per gap. Returns whether the input defines contact.
fn adopt_input_parts(
    mesh: &mut FeMesh,
    increments: &mut [plx_results::Increment],
    input: &Path,
    units: UnitSystem,
    sections: &mut InputSections,
) -> bool {
    match read_inp(input) {
        Ok(mut import) => {
            mesh.remove_elements(&import.skipped_elements);
            adopt_general_beams(mesh, increments, &import.mesh);
            *sections = shown_sections(&mut import, units);
            let parts: Vec<_> = (import.mesh.parts.into_iter())
                .map(|mut part| {
                    if let Some(name) = import.part_names.get(&part.name) {
                        part.name = name.clone();
                    }
                    part
                })
                .collect();
            mesh.adopt_parts(&parts);
            ["CONTACTPAIR", "CONTACTINCLUSION"]
                .iter()
                .any(|k| import.skipped_keywords.contains_key(*k))
        }
        Err(_) => plx_io::inp::defines_contact(input).unwrap_or(false),
    }
}

/// Adds the user elements U1 of general beam sections, which the solver leaves out of the
/// results, from the input it ran: their nodes are there, so they are drawn as lines. Their
/// stresses and strains are section forces, not stresses, so the nodes that belong to them
/// only show none.
fn adopt_general_beams(
    mesh: &mut FeMesh,
    increments: &mut [plx_results::Increment],
    input: &FeMesh,
) {
    let mut beam_nodes = std::collections::BTreeSet::new();
    for element in input.elements() {
        let missing = mesh.element(element.id).is_none()
            && element.nodes.iter().all(|&n| mesh.node(n).is_some());
        if missing && element.type_name.eq_ignore_ascii_case("U1") {
            beam_nodes.extend(element.nodes.iter().copied());
            let _ = mesh.add_element(element.clone());
        }
    }
    if beam_nodes.is_empty() {
        return;
    }
    for element in mesh.elements() {
        if !element.type_name.eq_ignore_ascii_case("U1") {
            for node in &element.nodes {
                beam_nodes.remove(node);
            }
        }
    }
    let indices: Vec<usize> = (beam_nodes.iter())
        .filter_map(|&n| mesh.node_index(n))
        .collect();
    for field in increments.iter_mut().flat_map(|i| &mut i.fields) {
        if matches!(field.rank, plx_results::FieldRank::Tensor(_)) {
            for component in &mut field.components {
                for &index in &indices {
                    if let Some(value) = component.values.get_mut(index) {
                        *value = f32::NAN;
                    }
                }
            }
        }
    }
}

/// The sections of the input CalculiX ran that let the results draw their elements as
/// solids, with their dimensions converted from the stored SI units into `units`.
fn shown_sections(import: &mut InpImport, units: UnitSystem) -> InputSections {
    let length = plx_model::STORED.factor_to(plx_model::Quantity::Length, units);
    let mut sections = InputSections {
        sections: std::mem::take(&mut import.sections),
        cross_sections: std::mem::take(&mut import.cross_sections),
        sets: BTreeMap::new(),
    };
    for cross_section in &mut sections.cross_sections {
        cross_section.profile.scale(length);
    }
    for section in &mut sections.sections {
        match &mut section.kind {
            plx_model::SectionKind::Shell { thickness, .. }
            | plx_model::SectionKind::Membrane { thickness, .. } => *thickness *= length,
            plx_model::SectionKind::Truss { area } => *area *= length * length,
            plx_model::SectionKind::Beam(_) | plx_model::SectionKind::Solid => {}
        }
        if let plx_model::Region::ElementSet(set) = &section.region
            && let Some(elements) = import.mesh.element_sets.remove(set)
        {
            sections.sets.insert(set.clone(), elements);
        }
    }
    sections
}

/// Loads an input, results or CAD file for a model shown in `units`: the mesh of an input file
/// and CAD geometry are stored in SI units, see [`plx_model::STORED`], and results are
/// converted from them into `units`. Project files hold several analyses and are
/// opened by the project window.
pub fn load(path: &Path, units: UnitSystem) -> Result<LoadedModel, String> {
    let start = Instant::now();
    let extension = |e: &str| path.extension().is_some_and(|x| x.eq_ignore_ascii_case(e));
    if plx_mesher::is_cad_file(path) {
        return load_geometry(&[path.to_path_buf()], units);
    }
    if extension(PROJECT_EXTENSION) {
        return Err(format!(
            "{} is a project; it opens in the project window",
            path.display()
        ));
    }
    let mut fe = FeModel::default();
    let mut contact = None;
    let mut input_sections = InputSections::default();
    // The history output of results of the built-in solver, which come without a `.dat`
    // file; `None` for CalculiX's results, which read theirs from the file.
    let mut solver_history = None;
    // The steps of the FE model the built-in solver recorded with its results.
    let mut step_records = None;
    let solver_results = plx_results::solver::is_solver_results(path);
    let (mesh, warnings, skipped_keywords, included_files, increments) =
        if extension("frd") || solver_results {
            let FrdImport {
                mut mesh,
                mut increments,
                warnings,
                date,
                time,
                ..
            } = if solver_results {
                // Taken from memory as the solver left them, without a `.frd` file to parse.
                let solved = plx_results::SolverResults::read(path).map_err(|e| e.to_string())?;
                step_records = Some((
                    solved.steps.clone(),
                    solved.convergence.clone(),
                    solved.unconverged_step,
                ));
                let input = companion_input(path).and_then(|p| std::fs::read_to_string(p).ok());
                solver_history = Some(plx_io::solver::history(&solved, input.as_deref()));
                plx_io::solver::import(&solved)
            } else {
                read_frd(path).map_err(|e| format!("{}: {e}", path.display()))?
            };
            // CalculiX ran in the stored SI units; the results show in the analysis' units.
            crate::results::convert_results(&mut mesh, &mut increments, units);
            // The results group the elements by material; the input it ran, named like the
            // results, knows the parts.
            contact = companion_input(path).map(|input| {
                adopt_input_parts(
                    &mut mesh,
                    &mut increments,
                    &input,
                    units,
                    &mut input_sections,
                )
            });
            (
                mesh,
                warnings,
                BTreeMap::new(),
                0,
                Some((increments, date, time)),
            )
        } else {
            let InpImport {
                mut mesh,
                warnings,
                skipped_keywords,
                files,
                ..
            } = read_inp(path).map_err(|e| e.to_string())?;
            // The numbers of an input file are in the analysis' units; the mesh is stored in SI
            // units.
            let length = units.factor_to(plx_model::Quantity::Length, plx_model::STORED);
            if length != 1.0 {
                mesh.scale(length);
            }
            // Plane stress, plane strain or axisymmetric elements make a 2D model.
            fe.properties.space = plx_model::ModelSpace::of_mesh(&mesh);
            let included = files.len().saturating_sub(1);
            (mesh, warnings, skipped_keywords, included, None)
        };
    if mesh.element_count() == 0 {
        return Err(format!(
            "{} contains no elements that can be displayed",
            path.display()
        ));
    }
    let mut model = Model::new(path, mesh);
    model.fe = fe;
    model.input_sections = input_sections;
    model.warnings = warnings;
    model.skipped_keywords = skipped_keywords;
    model.included_files = included_files;
    model.results = increments.map(|(increments, date, time)| {
        let mut view = ResultsView::new(increments, model.mesh.bounds());
        // The solver writes contact into the results only on request; the input it ran tells.
        if let Some(contact) = contact {
            view.contact |= contact;
            view.follow_default_deformation();
        }
        view.date = date;
        view.time = time;
        if let Some((records, convergence, unconverged)) = &step_records {
            view.set_step_records(records, convergence, *unconverged);
        }
        view
    });
    // History outputs CalculiX printed next to the results, or
    // those of the built-in solver.
    if let Some(view) = &mut model.results {
        let dat = ["dat", "DAT"]
            .map(|e| path.with_extension(e))
            .into_iter()
            .find(|p| p.is_file() && solver_history.is_none());
        let history = match (solver_history, dat) {
            (Some(history), _) => history.map(Ok),
            (None, Some(dat)) => Some(plx_io::dat::read_dat(&dat).map_err(|e| (dat, e))),
            (None, None) => None,
        };
        if let Some(history) = history {
            match history {
                Ok(mut import) => {
                    crate::results::convert_history(&mut import.sets, units);
                    let warnings = view.set_file_history(import.sets, &model.mesh);
                    model.warnings.extend(import.warnings);
                    model.warnings.extend(warnings);
                }
                Err((dat, error)) => model.warnings.push(format!("{}: {error}", dat.display())),
            }
        }
    }
    let render_meshes = model.render_meshes();
    for (part, render) in model.parts.iter_mut().zip(&render_meshes) {
        part.bounds = render.bounds();
    }
    model.load_time = start.elapsed();
    Ok(LoadedModel {
        model,
        render_meshes,
        geometry_view: None,
    })
}

/// Builds the model of an analysis of a project; `path` is the project file. An analysis
/// without a mesh opens on its geometry, if it has one.
pub fn load_analysis(path: &Path, analysis: plx_io::project::ProjectAnalysis) -> LoadedModel {
    let start = Instant::now();
    let plx_io::project::ProjectAnalysis {
        geometry,
        mesh,
        model: fe,
        mesh_inputs,
        results,
        ..
    } = analysis;
    let mut warnings = Vec::new();
    // The geometry is shown on the Geometry tab; without Gmsh only the mesh is.
    let geometry_view = match &geometry {
        Some(geometry) => match plx_mesher::tessellate(geometry) {
            Ok(display) => Some(Model::geometry_view(path, display)),
            Err(error) => {
                warnings.push(format!("Geometry cannot be displayed: {error}"));
                None
            }
        },
        None => None,
    };
    let mut model = Model::new(path, mesh);
    model.fe = fe;
    model.geometry = geometry;
    model.bound_results = results;
    model.mesh_inputs = mesh_inputs;
    model.warnings = warnings;
    let render_meshes = match &geometry_view {
        Some(view) if model.mesh.element_count() == 0 => view.render_meshes(),
        _ => model.render_meshes(),
    };
    for (part, render) in model.parts.iter_mut().zip(&render_meshes) {
        part.bounds = render.bounds();
    }
    model.load_time = start.elapsed();
    LoadedModel {
        model,
        render_meshes,
        geometry_view,
    }
}

impl Model {
    /// What the user defined on these results, for results bound to a project.
    pub fn results_settings(&self) -> Option<plx_results::ResultsSettings> {
        let view = self.results.as_ref()?;
        Some(plx_results::ResultsSettings {
            hot_spots: self.hot_spots.definitions.clone(),
            reference_points: self.fe.reference_points.clone(),
            coordinate_systems: self.fe.coordinate_systems.clone(),
            planes: self.fe.planes.clone(),
            result_paths: self.fe.result_paths.clone(),
            result_planes: self.fe.result_planes.clone(),
            plain_part_names: (self.parts.iter())
                .filter(|part| !part.contours)
                .map(|part| part.name.clone())
                .collect(),
            mesh: Some(self.mesh.fingerprint()),
            inherited_features: self.inherited_features.clone(),
            ..view.settings()
        })
        .map(|mut settings| {
            let saved = &mut settings.view;
            saved.hidden_parts = (self.parts.iter())
                .filter(|p| !p.visible)
                .map(|p| p.name.clone())
                .collect();
            saved.annotations = self.query_annotations.iter().map(Into::into).collect();
            saved.path_plots = self.path_plots.iter().map(|p| p.saved()).collect();
            settings
        })
    }

    /// Takes saved settings of bound results; returns the outputs that failed. History
    /// outputs may put the positions of `moving_loads`, those of the FE model, beside their
    /// values.
    pub fn restore_results_settings(
        &mut self,
        settings: &plx_results::ResultsSettings,
        moving_loads: Vec<plx_model::LoadStations>,
    ) -> Vec<String> {
        self.fe.reference_points = settings.reference_points.clone();
        self.fe.coordinate_systems = settings.coordinate_systems.clone();
        self.fe.planes = settings.planes.clone();
        self.fe.result_paths = settings.result_paths.clone();
        self.fe.result_planes = settings.result_planes.clone();
        self.inherited_features = settings.inherited_features.clone();
        self.hot_spots.definitions = settings.hot_spots.clone();
        let view = &settings.view;
        for part in &mut self.parts {
            part.visible = !view.hidden_parts.contains(&part.name);
        }
        self.query_annotations = view.annotations.iter().cloned().map(Into::into).collect();
        self.path_plots = (view.path_plots.iter())
            .map(crate::path_plot::PathPlot::from_saved)
            .collect();
        let plain: Vec<usize> = if settings.plain_part_names.is_empty()
            && settings.mesh == Some(self.mesh.fingerprint())
        {
            settings.plain_parts.clone()
        } else {
            (self.parts.iter().enumerate())
                .filter(|(_, part)| settings.plain_part_names.contains(&part.name))
                .map(|(index, _)| index)
                .collect()
        };
        self.set_contours(&plain, false);
        // History outputs of hot spots need their values first.
        let reports = crate::hot_spots::evaluate(self);
        match &mut self.results {
            Some(view) => {
                view.hot_spot_reports = reports;
                view.moving_loads = moving_loads;
                view.restore(settings, &self.mesh)
            }
            None => Vec::new(),
        }
    }

    /// A model of the mesh with summaries and skins of its parts.
    pub fn new(path: &Path, mesh: FeMesh) -> Self {
        let skins = (mesh.parts.iter())
            .map(|part| extract_part_skin(&mesh, part, FEATURE_ANGLE_DEG))
            .collect();
        Self::with_skins(path, mesh, skins)
    }

    fn with_skins(path: &Path, mesh: FeMesh, skins: Vec<PartSkin>) -> Self {
        let origin = mesh.bounds().map_or(DVec3::ZERO, |(min, max)| {
            (DVec3::from(min) + DVec3::from(max)) * 0.5
        });
        let mut parts = Vec::with_capacity(mesh.parts.len());
        for (index, part) in mesh.parts.iter().enumerate() {
            let mut types: BTreeMap<&str, usize> = BTreeMap::new();
            let mut nodes = std::collections::HashSet::new();
            for element in part.elements.iter().filter_map(|&id| mesh.element(id)) {
                *types.entry(&element.type_name).or_default() += 1;
                nodes.extend(element.nodes.iter().copied());
            }
            parts.push(PartInfo {
                name: part.name.clone(),
                color: part_color(index),
                element_count: part.elements.len(),
                node_count: nodes.len(),
                element_types: types.into_iter().map(|(t, n)| (t.to_string(), n)).collect(),
                bounds: None,
                visible: true,
                contours: true,
                dimension: None,
            });
        }
        let mut model = Self {
            path: path.to_path_buf(),
            mesh,
            parts,
            warnings: Vec::new(),
            notes: Vec::new(),
            skipped_keywords: BTreeMap::new(),
            included_files: 0,
            load_time: Duration::ZERO,
            results: None,
            fe: FeModel::default(),
            geometry: None,
            bound_results: None,
            mesh_inputs: BTreeMap::new(),
            bound_results_file: None,
            is_geometry: false,
            cad_entities: Vec::new(),
            geometry_solids: 0,
            geometry_topology: Default::default(),
            geometry_index: Default::default(),
            hot_spots: Default::default(),
            inherited_features: Vec::new(),
            path_plots: Vec::new(),
            highlight: Highlight::default(),
            clip: None,
            origin,
            skins,
            section_cells: Default::default(),
            explosion: Explosion::default(),
            query_annotations: Vec::new(),
            motion: None,
            cyclic_preview: None,
            part_nodes: Default::default(),
            skin_lookup: Default::default(),
            cad_display: Default::default(),
            checks: Default::default(),
            cad_index: Default::default(),
            history: Default::default(),
            revision: 0,
            thick_shells: Vec::new(),
            thick_lines: Vec::new(),
            input_sections: InputSections::default(),
            global: None,
        };
        let meshes = model.render_meshes();
        for (part, render) in model.parts.iter_mut().zip(&meshes) {
            part.bounds = render.bounds();
        }
        model
    }

    /// The CAD geometry for the Geometry tab: its faces shaded each on its own, outlined by
    /// its edges, without the triangles of the display mesh.
    pub fn geometry_view(path: &Path, display: GeometryDisplay) -> Self {
        // Only the CAD edges, drawn as line elements, outline the geometry.
        let skins = (display.mesh.parts.iter())
            .map(|part| extract_part_skin(&display.mesh, part, 180.0))
            .collect();
        Self::geometry_view_with_skins(path, display, skins)
    }

    /// [`Model::geometry_view`] with the skins of the parts already extracted; their faces
    /// take the CAD faces as regions here.
    fn geometry_view_with_skins(
        path: &Path,
        display: GeometryDisplay,
        mut skins: Vec<PartSkin>,
    ) -> Self {
        let mesh = display.mesh;
        for face in skins.iter_mut().flat_map(|skin| &mut skin.faces) {
            let id = mesh.elements()[face.element].id;
            if let Some(CadEntity::Face(tag)) = display.entities.get(id as usize - 1) {
                face.region = *tag as usize;
            }
        }
        let mut model = Self::with_skins(path, mesh, skins);
        model.is_geometry = true;
        for (part, dimension) in model.parts.iter_mut().zip(display.part_dimensions) {
            part.dimension = Some(dimension);
        }
        model.cad_entities = display.entities;
        model.geometry_solids = display.solids;
        model.geometry_topology = display.topology;
        model
    }

    /// The view of `smaller`, the geometry left after deleting the parts `deleted`, made
    /// from this one, see [`GeometryDisplay::without_parts`]: the parts that stay keep their
    /// triangles and skins, which take long to make anew for a large assembly. `None` if
    /// the geometry has to be tessellated anew.
    pub fn geometry_view_without(
        mut self,
        deleted: &[String],
        smaller: &Geometry,
        tags: &BTreeMap<CadEntity, CadEntity>,
    ) -> Option<Self> {
        let path = self.path.clone();
        let kept: Vec<bool> = (self.parts.iter())
            .map(|p| !deleted.iter().any(|d| p.name.eq_ignore_ascii_case(d)))
            .collect();
        let skins = std::mem::take(&mut self.skins);
        let elements: Vec<ElementId> = self.mesh.elements().iter().map(|e| e.id).collect();
        let nodes = self.mesh.node_ids().to_vec();
        let display = (self.into_geometry_display()).without_parts(deleted, smaller, tags)?;
        // The skins index the elements and nodes, which move up in the smaller mesh.
        let element: Vec<Option<usize>> = (elements.iter())
            .map(|&id| display.mesh.element_index(id))
            .collect();
        let node: Vec<Option<usize>> = (nodes.iter())
            .map(|&id| display.mesh.node_index(id))
            .collect();
        let node = |index: &mut usize| -> Option<()> {
            *index = (*node.get(*index)?)?;
            Some(())
        };
        let element = |index: &mut usize| -> Option<()> {
            *index = (*element.get(*index)?)?;
            Some(())
        };
        let mut kept_skins = Vec::new();
        for (mut skin, _) in skins.into_iter().zip(kept).filter(|(_, kept)| *kept) {
            for face in &mut skin.faces {
                element(&mut face.element)?;
                face.corners.iter_mut().try_for_each(node)?;
                face.mids.iter_mut().try_for_each(node)?;
            }
            for edge in &mut skin.edges {
                node(&mut edge.a)?;
                node(&mut edge.b)?;
                edge.mid.as_mut().map_or(Some(()), node)?;
            }
            skin.lines.iter_mut().flatten().try_for_each(node)?;
            skin.line_elements.iter_mut().try_for_each(element)?;
            kept_skins.push(skin);
        }
        if kept_skins.len() != display.mesh.parts.len() {
            return None;
        }
        Some(Self::geometry_view_with_skins(&path, display, kept_skins))
    }

    /// The geometry display this view was made from, see [`Model::geometry_view`].
    pub fn into_geometry_display(self) -> GeometryDisplay {
        let topology = self.geometry_topology;
        GeometryDisplay {
            part_dimensions: self
                .parts
                .iter()
                .map(|p| p.dimension.unwrap_or(3))
                .collect(),
            mesh: self.mesh,
            entities: self.cad_entities,
            solids: self.geometry_solids,
            faces: topology.face_edges.len(),
            edges: topology.edge_vertices.len(),
            topology,
        }
    }

    /// The CAD face or edge each element of the geometry display shows; element ids count
    /// from 1.
    pub fn cad_entities(&self) -> &[CadEntity] {
        &self.cad_entities
    }

    /// The CAD edges bounding each CAD face of the geometry display.
    pub fn face_edges(&self) -> &BTreeMap<i32, Vec<i32>> {
        &self.geometry_topology.face_edges
    }

    /// Of each part of the geometry display, the dimension of its CAD shape: 3 a solid, 2
    /// faces, 1 edges.
    pub fn part_dimensions(&self) -> Vec<u8> {
        self.parts
            .iter()
            .map(|p| p.dimension.unwrap_or(3))
            .collect()
    }

    /// The CAD face or edge an element of the geometry display shows.
    pub fn cad_entity(&self, element: ElementId) -> Option<CadEntity> {
        let index = usize::try_from(element).ok()?.checked_sub(1)?;
        self.cad_entities.get(index).copied()
    }

    /// The skin faces that show the CAD faces `tags` of a geometry display, as (element,
    /// CalculiX face number), in every part that draws them.
    pub fn cad_face_items(&self, tags: &BTreeSet<i32>) -> HashSet<(ElementId, u8)> {
        let display = self.cad_display();
        (tags.iter())
            .filter_map(|tag| display.faces.get(tag))
            .flatten()
            .copied()
            .collect()
    }

    /// The segments of a geometry display that show the CAD edges `tags`, as node pairs.
    pub fn cad_edge_segments(&self, tags: &BTreeSet<i32>) -> Vec<[NodeId; 2]> {
        let display = self.cad_display();
        (tags.iter())
            .filter_map(|tag| display.edges.get(tag))
            .flatten()
            .copied()
            .collect()
    }

    fn cad_display(&self) -> &CadDisplay {
        self.cad_display.get_or_init(|| {
            let mut display = CadDisplay::default();
            if self.cad_entities.is_empty() {
                return display;
            }
            let elements = self.mesh.elements();
            for face in self.skins.iter().flat_map(|skin| &skin.faces) {
                let id = elements[face.element].id;
                if let Some(CadEntity::Face(tag)) = self.cad_entity(id) {
                    let items = display.faces.entry(tag).or_default();
                    items.push((id, face.face as u8 + 1));
                }
            }
            for element in elements.iter().filter(|e| e.shape == ElementShape::Line2) {
                if let Some(CadEntity::Edge(tag)) = self.cad_entity(element.id) {
                    let segment = [element.nodes[0], element.nodes[1]];
                    display.edges.entry(tag).or_default().push(segment);
                }
            }
            display
        })
    }

    /// Changes the mesh in place, e.g. by a mesh tool, and rebuilds what the GUI derives
    /// from it; the parts keep their colours and visibility.
    pub fn edit_mesh(&mut self, edit: impl FnOnce(&mut FeMesh, &mut FeModel)) {
        // A copy if the history follows the model, so that it keeps the mesh as it was.
        let mut mesh = if self.history.follows() {
            self.mesh.clone()
        } else {
            std::mem::take(&mut self.mesh)
        };
        edit(&mut mesh, &mut self.fe);
        let kept: Vec<([f32; 3], bool)> = self.parts.iter().map(|p| (p.color, p.visible)).collect();
        self.set_mesh(mesh);
        for (part, (color, visible)) in self.parts.iter_mut().zip(kept) {
            part.color = color;
            part.visible = visible;
        }
    }

    /// Replaces the mesh, e.g. by a newly generated one; the FE model stays.
    pub fn set_mesh(&mut self, mut mesh: FeMesh) {
        // The user's sets are found on the new mesh by their selections, and its parts get
        // the element types chosen for them, also when they were meshed anew.
        self.fe.write_mesh_sets(&mut mesh);
        self.fe.apply_element_types(&mut mesh);
        let mut fresh = Self::new(&self.path, mesh);
        std::mem::swap(&mut self.mesh, &mut fresh.mesh);
        self.history.mesh_replaced(std::mem::take(&mut fresh.mesh));
        self.revision += 1;
        self.parts = fresh.parts;
        self.skins = fresh.skins;
        self.origin = fresh.origin;
        self.highlight = Highlight::default();
        self.section_cells = Default::default();
        self.part_nodes = Default::default();
        self.skin_lookup = Default::default();
        self.cad_display = Default::default();
        self.checks = Default::default();
        self.cad_index = Default::default();
        self.geometry_index = Default::default();
        self.query_annotations.clear();
        // Reference points at the centre of a selection follow the new mesh.
        self.fe.update_reference_points(&self.mesh);
        // The exploded view stays on and is laid out anew for the new mesh.
        self.explosion.clear_layout();
        if let Some(parameters) = self.explosion.applied.clone() {
            let offsets = self.explosion_offsets(&parameters);
            self.explosion.show(offsets, false);
        } else {
            self.explosion.show(Vec::new(), false);
        }
    }

    /// Problems of the FE model that make CalculiX abort or give useless results. The checks
    /// run again when the FE model changed; the part that depends on the mesh only once per
    /// mesh.
    pub fn findings(&self) -> Vec<Finding> {
        let mut cache = self.checks.lock().unwrap_or_else(|e| e.into_inner());
        let space = self.fe.properties.space;
        if let Some(checks) = cache.as_ref()
            && checks.fe == self.fe
            && checks.mesh.is_current(&self.mesh, space)
        {
            return checks.findings.clone();
        }
        let mesh = match cache.take() {
            Some(checks) if checks.mesh.is_current(&self.mesh, space) => checks.mesh,
            _ => MeshCheck::new(&self.mesh, space),
        };
        let findings = self.fe.check(&self.mesh, &mesh);
        let replaced = self.fe.replaced_contact_regions(&self.mesh);
        *cache = Some(Checks {
            mesh,
            fe: self.fe.clone(),
            findings: findings.clone(),
            replaced,
        });
        findings
    }

    /// The contact regions the bolts replace, with the bolt; computed with the checks.
    pub fn replaced_regions(&self) -> std::collections::BTreeMap<usize, usize> {
        self.findings();
        let cache = self.checks.lock().unwrap_or_else(|e| e.into_inner());
        cache
            .as_ref()
            .map(|c| c.replaced.clone())
            .unwrap_or_default()
    }

    /// Lookups for selections on the geometry display, see [`crate::geometry_selection`].
    pub fn geometry_index(&self) -> &crate::geometry_selection::GeometryIndex {
        (self.geometry_index).get_or_init(|| {
            crate::geometry_selection::GeometryIndex::new(
                &self.mesh,
                &self.cad_entities,
                &self.geometry_topology,
            )
        })
    }

    pub fn cad_index(&self) -> &crate::cad_selection::CadIndex {
        (self.cad_index).get_or_init(|| crate::cad_selection::CadIndex::new(&self.mesh))
    }

    /// Whether the mesh was generated from CAD geometry and knows where its entities lie.
    pub fn has_cad(&self) -> bool {
        !self.mesh.cad.is_empty()
    }

    fn part_nodes(&self) -> &PartNodes {
        self.part_nodes.get_or_init(|| PartNodes::new(&self.mesh))
    }

    /// The part a node belongs to; of several parts that share it, the first.
    pub fn node_part(&self, id: NodeId) -> Option<usize> {
        let index = self.mesh.node_index(id)?;
        let part = *self.part_nodes().part_of.get(index)?;
        (part != u32::MAX).then_some(part as usize)
    }

    fn skin_lookup(&self) -> &SkinLookup {
        (self.skin_lookup).get_or_init(|| SkinLookup::new(&self.skins, self.mesh.elements().len()))
    }

    /// The offsets of all parts for an exploded view; the layout is kept until parameters
    /// it depends on change.
    pub fn explosion_offsets(&mut self, parameters: &Parameters) -> Vec<glam::DVec3> {
        self.explosion_layout(parameters)
            .offsets(parameters.scale(), parameters.sequential)
    }

    /// The layout of an exploded view, computed when first needed.
    pub fn explosion_layout(&mut self, parameters: &Parameters) -> &crate::exploded::Layout {
        let part_nodes = self.part_nodes.get_or_init(|| PartNodes::new(&self.mesh));
        let assembly = Assembly {
            mesh: &self.mesh,
            parts: (self.mesh.parts.iter().enumerate())
                .map(|(i, part)| PartShape {
                    name: &part.name,
                    nodes: &part_nodes.nodes[i],
                    skin: &self.skins[i],
                })
                .collect(),
        };
        self.explosion.layout(&assembly, parameters)
    }

    /// How far a node is moved by the exploded view.
    pub fn explosion_offset(&self, index: usize) -> DVec3 {
        let offsets = self.explosion.offsets();
        if offsets.is_empty() {
            return DVec3::ZERO;
        }
        let part = self
            .part_nodes()
            .part_of
            .get(index)
            .copied()
            .unwrap_or(u32::MAX);
        offsets.get(part as usize).copied().unwrap_or(DVec3::ZERO)
    }

    /// Node coordinates moved by the exploded view, without any deformation.
    pub fn exploded_coords(&self) -> std::borrow::Cow<'_, [[f64; 3]]> {
        let coords = self.mesh.coords();
        if !self.explosion.is_shown() {
            return std::borrow::Cow::Borrowed(coords);
        }
        let offsets = self.explosion.offsets();
        let part_of = &self.part_nodes().part_of;
        std::borrow::Cow::Owned(
            (coords.iter().zip(part_of))
                .map(|(p, &part)| {
                    let o = offsets.get(part as usize).copied().unwrap_or(DVec3::ZERO);
                    [p[0] + o.x, p[1] + o.y, p[2] + o.z]
                })
                .collect(),
        )
    }

    /// Whether this shows CAD geometry rather than a mesh.
    pub fn is_geometry(&self) -> bool {
        self.is_geometry
    }

    /// Whether this is a results file rather than an FE model.
    pub fn is_results(&self) -> bool {
        self.results.is_some()
    }

    /// Node coordinates as drawn: moved by the exploded view and with the shown deformation
    /// of results.
    pub fn shown_coords(&self) -> std::borrow::Cow<'_, [[f64; 3]]> {
        self.deformed_coords().0
    }

    /// Node coordinates as drawn and whether a deformation is shown.
    fn deformed_coords(&self) -> (std::borrow::Cow<'_, [[f64; 3]]>, bool) {
        let mut coords = self.exploded_coords();
        if let Some(motion) = &self.motion {
            let moved = coords.to_mut();
            for &(node, offset) in motion {
                if let Some(p) = moved.get_mut(node) {
                    *p = [0, 1, 2].map(|k| p[k] + offset[k]);
                }
            }
        }
        let Some(view) = &self.results else {
            return (coords, false);
        };
        let scale = (view.scale() * view.amplitude()) as f64;
        match view.shown_displacements() {
            Some(displacements) if scale != 0.0 => (
                std::borrow::Cow::Owned(
                    coords
                        .iter()
                        .zip(displacements.iter())
                        .map(|(p, d)| [0, 1, 2].map(|k| p[k] + scale * d[k] as f64))
                        .collect(),
                ),
                true,
            ),
            _ => (coords, false),
        }
    }

    /// Turns the colour contours of parts on or off and refreshes the nodes the results leave
    /// out of the legend: those used by plain parts only.
    pub fn set_contours(&mut self, parts: &[usize], contours: bool) {
        for &part in parts {
            if let Some(info) = self.parts.get_mut(part) {
                info.contours = contours;
            }
        }
        self.update_plain_nodes();
    }

    fn update_plain_nodes(&mut self) {
        let Some(view) = &mut self.results else {
            return;
        };
        if self.parts.iter().all(|p| p.contours) {
            view.plain_nodes.clear();
            return;
        }
        let count = self.mesh.coords().len();
        let mut plain = vec![false; count];
        let mut coloured = vec![false; count];
        for (index, info) in self.parts.iter().enumerate() {
            let target = if info.contours {
                &mut coloured
            } else {
                &mut plain
            };
            for id in self.mesh.part_nodes(&[index]) {
                if let Some(node) = self.mesh.node_index(id) {
                    target[node] = true;
                }
            }
        }
        // A node shared with a part that has contours keeps its colour.
        view.plain_nodes = plain
            .into_iter()
            .zip(coloured)
            .map(|(plain, coloured)| plain && !coloured)
            .collect();
    }

    /// Node coordinates as drawn and the normalized contour values, if a result is shown;
    /// the flag tells whether the shape is deformed and the undeformed outline is wanted.
    fn shown_state(&self) -> ShownState<'_> {
        let (coords, is_deformed) = self.deformed_coords();
        let mut scalars = None;
        let mut deformed = false;
        if let Some(view) = &self.results {
            if is_deformed {
                deformed = view.show_undeformed;
            }
            if let (Some(values), Some(legend)) = (view.shown_values(), view.legend()) {
                scalars = Some(legend.normalize(&values));
            }
        }
        (coords, scalars, deformed)
    }

    /// The transformed copies of the results ([`ResultsView::transformations`]): per copy
    /// its transformation and its normalized contour values.
    fn transformed_copies(&self) -> Vec<(DAffine3, Option<Vec<f32>>)> {
        let Some(view) = &self.results else {
            return Vec::new();
        };
        let legend = view.legend();
        (view.instances().into_iter().skip(1))
            .map(|instance| {
                let scalars = (view.shown_values_on(&instance))
                    .zip(legend.as_ref())
                    .map(|(values, legend)| legend.normalize(&values));
                (instance, scalars)
            })
            .collect()
    }

    /// GPU-ready meshes of all parts, deformed and coloured by the selected result if any,
    /// with the transformed copies of the results.
    pub fn render_meshes(&self) -> Vec<RenderMesh> {
        let (coords, scalars, deformed) = self.shown_state();
        let arrows = self.part_arrows();
        // Coloured arrows take the contour colours, the surfaces under them turn grey.
        let surface = |scalars: Option<Vec<f32>>| {
            scalars.filter(|_| !arrows.as_ref().is_some_and(|a| a.colored))
        };
        let mut meshes = self.part_meshes(&coords, surface(scalars.clone()).as_deref(), deformed);
        let lic = self.lic_values();
        if let Some((values, _)) = &lic {
            for (index, mesh) in meshes.iter_mut().enumerate() {
                if self.shows_lic(index) {
                    mesh.set_directions(values);
                }
            }
        }
        if let Some(arrows) = &arrows {
            for (index, mesh) in meshes.iter_mut().enumerate() {
                mesh.glyphs = arrows.lines(
                    index,
                    &coords,
                    scalars.as_deref(),
                    &DAffine3::IDENTITY,
                    self.origin,
                );
            }
        }
        let copies = self.transformed_copies();
        let solids = if copies.is_empty() {
            SolidElements::default()
        } else {
            self.solid_elements()
        };
        for (instance, scalars) in copies {
            let moved = transform_coords(&coords, &instance);
            let undeformed = deformed.then(|| transform_coords(&self.exploded_coords(), &instance));
            let parts = meshes.iter_mut().zip(&self.parts).zip(&self.skins);
            for (index, ((mesh, info), skin)) in parts.enumerate() {
                let surface_scalars = surface(scalars.clone());
                let mut copy = part_render_mesh(
                    &moved,
                    skin,
                    self.origin,
                    info.color,
                    |_| None,
                    SMOOTH_ANGLE_DEG,
                    surface_scalars.as_deref(),
                );
                let surface_scalars = surface_scalars.as_deref();
                self.append_solids(&mut copy, index, &moved, surface_scalars, &solids, false);
                if let Some(arrows) = &arrows {
                    copy.glyphs =
                        arrows.lines(index, &moved, scalars.as_deref(), &instance, self.origin);
                }
                if let Some((values, tensor)) = &lic
                    && self.shows_lic(index)
                {
                    copy.set_directions(&crate::lic::turned(values, &instance, *tensor));
                }
                if let Some(undeformed) = &undeformed {
                    copy.wireframe_edges = wireframe_edges(undeformed, skin, self.origin);
                }
                mesh.append(copy);
            }
        }
        meshes
    }

    /// The vector source of LIC per node, if the results view shows it, and whether it is a
    /// tensor. Only parts with colour contours show streaks ([`Model::shows_lic`]).
    fn lic_values(&self) -> Option<(Vec<plx_render::Directions>, bool)> {
        let view = self.results.as_ref()?;
        let values = crate::lic::node_values(view)?;
        let tensor = view.vector_source()?.is_line_field();
        Some((values, tensor))
    }

    fn shows_lic(&self, part: usize) -> bool {
        self.parts.get(part).is_some_and(|p| p.contours)
    }

    /// The arrows of the shown vector source, if the results view draws them: spread over
    /// the undeformed surface of the parts with colour contours.
    fn part_arrows(&self) -> Option<crate::arrows::PartArrows> {
        let view = self.results.as_ref()?;
        let reference = self.exploded_coords();
        let with_arrows = |part: usize| self.parts.get(part).is_some_and(|p| p.contours);
        crate::arrows::PartArrows::new(view, &reference, &self.skins, with_arrows)
    }

    /// [`Model::render_meshes`] followed, for a submodel, by the wireframe of its global model.
    pub fn scene_meshes(&self) -> Vec<RenderMesh> {
        let mut meshes = self.render_meshes();
        meshes.extend(self.global_wireframe());
        meshes.extend(self.cyclic_wireframe());
        meshes
    }

    /// The other sectors of a cyclic symmetry preview: the outline of all parts rotated
    /// about the axis by each multiple of the sector angle, as a mesh of lines only.
    fn cyclic_wireframe(&self) -> Option<RenderMesh> {
        let preview = self
            .cyclic_preview
            .as_ref()
            .filter(|_| self.results.is_none())?;
        let axis = DVec3::from(preview.axis_direction).try_normalize()?;
        let point = DVec3::from(preview.axis_point);
        let coords = self.exploded_coords();
        let mut mesh = RenderMesh::default();
        for k in 1..preview.sectors {
            let angle = std::f64::consts::TAU * f64::from(k) / f64::from(preview.sectors);
            let instance = DAffine3::from_translation(point)
                * DAffine3::from_axis_angle(axis, angle)
                * DAffine3::from_translation(-point);
            let turned = transform_coords(&coords, &instance);
            for skin in &self.skins {
                mesh.wireframe_edges
                    .extend(wireframe_edges(&turned, skin, self.origin));
            }
        }
        Some(mesh)
    }

    /// The edges of the global model of a submodel, deformed by the same scale as the
    /// results, as a mesh of lines only: it neither shows in the tree nor in the legend.
    fn global_wireframe(&self) -> Option<RenderMesh> {
        let (Some(global), Some(view)) = (&self.global, &self.results) else {
            return None;
        };
        if !view.show_global_contours {
            return None;
        }
        let model = &global.model;
        let increment = (model.results.as_ref())?.increments.get(global.increment)?;
        let mut coords = model.mesh.coords().to_vec();
        let scale = (view.scale() * view.amplitude()) as f64;
        if scale != 0.0
            && let Some(displacements) = increment.displacements()
        {
            for (p, d) in coords.iter_mut().zip(&displacements) {
                *p = [0, 1, 2].map(|k| p[k] + scale * d[k] as f64);
            }
        }
        // Render positions are relative to the submodel's origin.
        let mut mesh = RenderMesh::default();
        for skin in &model.skins {
            mesh.wireframe_edges
                .extend(mesh_wireframe(&coords, skin, self.origin));
        }
        Some(mesh)
    }

    fn part_meshes(
        &self,
        coords: &[[f64; 3]],
        scalars: Option<&[f32]>,
        deformed: bool,
    ) -> Vec<RenderMesh> {
        let undeformed = if deformed {
            self.exploded_coords()
        } else {
            std::borrow::Cow::Borrowed(&[][..])
        };
        let solids = self.solid_elements();
        (0..self.parts.len().min(self.skins.len()))
            .map(|index| self.part_mesh(index, coords, scalars, &undeformed, deformed, &solids))
            .collect()
    }

    /// The mesh of one part as [`Model::part_meshes`] builds it.
    fn part_mesh(
        &self,
        index: usize,
        coords: &[[f64; 3]],
        scalars: Option<&[f32]>,
        undeformed: &[[f64; 3]],
        deformed: bool,
        solids: &SolidElements,
    ) -> RenderMesh {
        let (info, skin) = (&self.parts[index], &self.skins[index]);
        let mut mesh = part_render_mesh(
            coords,
            skin,
            self.origin,
            info.color,
            self.face_highlight(index, skin),
            SMOOTH_ANGLE_DEG,
            scalars,
        );
        self.highlight_outline(index, skin, &mut mesh);
        self.append_solids(&mut mesh, index, coords, scalars, solids, true);
        if deformed {
            mesh.wireframe_edges = wireframe_edges(undeformed, skin, self.origin);
        }
        if self.is_geometry {
            mesh.mesh_edges.clear();
        }
        mesh
    }

    /// The mesh of one part as [`Model::scene_meshes`] draws it, so that a change of the
    /// highlight redraws only the parts it concerns. `None` for results, whose parts are
    /// drawn with contours, copies and arrows: they are built all at once.
    pub fn part_scene_mesh(&self, index: usize) -> Option<RenderMesh> {
        if self.results.is_some() || index >= self.parts.len().min(self.skins.len()) {
            return None;
        }
        let coords = self.shown_coords();
        let solids = self.solid_elements();
        Some(self.part_mesh(index, &coords, None, &[], false, &solids))
    }

    /// The parts drawn differently with the highlight `new` than with `old`; `None` when
    /// the scene has to be built anew as a whole, see [`Model::part_scene_mesh`].
    pub fn highlight_changed_parts(
        &self,
        old: &Highlight,
        new: &Highlight,
    ) -> Option<BTreeSet<usize>> {
        if self.results.is_some() {
            return None;
        }
        let mut parts: BTreeSet<usize> = (old.parts.symmetric_difference(&new.parts))
            .chain(old.outlines.symmetric_difference(&new.outlines))
            .copied()
            .collect();
        let part_of = &self.skin_lookup().element_part;
        let faces = (old.faces.symmetric_difference(&new.faces)).chain(
            old.secondary_faces
                .symmetric_difference(&new.secondary_faces),
        );
        // A face of an element on no skin isn't drawn.
        let changed = (faces.filter_map(|&(element, _)| self.mesh.element_index(element)))
            .filter_map(|index| part_of.get(index).copied())
            .filter(|&part| part != u32::MAX);
        parts.extend(changed.map(|part| part as usize));
        Some(parts)
    }

    /// The sections whose elements can be drawn as the solid they stand for: those of a 3D
    /// FE model, or for results those of the input CalculiX ran.
    fn solid_sections(&self) -> impl Iterator<Item = &plx_model::Section> {
        let three_d = self.fe.properties.space == plx_model::ModelSpace::ThreeD;
        let sections = match &self.results {
            _ if self.is_geometry => &[][..],
            Some(_) => &self.input_sections.sections[..],
            None if three_d => &self.fe.sections[..],
            None => &[][..],
        };
        sections.iter()
    }

    /// The shell and membrane sections whose elements can be drawn as the solid they stand
    /// for, see [`Model::thick_shells`].
    pub fn shell_sections(&self) -> impl Iterator<Item = &plx_model::Section> {
        self.solid_sections().filter(|s| {
            matches!(
                s.kind,
                plx_model::SectionKind::Shell { .. } | plx_model::SectionKind::Membrane { .. }
            )
        })
    }

    /// The beam and truss sections whose elements can be drawn as their profile swept along
    /// the line, see [`Model::thick_lines`]. Results hold beams as lines only where the
    /// input asked for `OUTPUT=2D`, as it does with shells; else the solver expanded them.
    /// A beam section comes with the profile of its cross-section.
    pub fn line_sections(
        &self,
    ) -> impl Iterator<Item = (&plx_model::Section, Option<&plx_model::BeamProfile>)> {
        let cross_sections = match &self.results {
            Some(_) => &self.input_sections.cross_sections[..],
            None => &self.fe.cross_sections[..],
        };
        (self.solid_sections())
            .filter(|s| {
                matches!(
                    s.kind,
                    plx_model::SectionKind::Beam(_) | plx_model::SectionKind::Truss { .. }
                )
            })
            .map(move |s| {
                let profile = match &s.kind {
                    plx_model::SectionKind::Beam(beam) => (cross_sections.iter())
                        .find(|c| c.name == beam.cross_section)
                        .map(|c| &c.profile),
                    _ => None,
                };
                (s, profile)
            })
    }

    /// The elements of [`Model::thick_shells`] and [`Model::thick_lines`] with what they are
    /// drawn as, by element index.
    fn solid_elements(&self) -> SolidElements {
        let mut solids = SolidElements::default();
        if self.is_geometry {
            return solids;
        }
        let of_family = |id: ElementId, family: ElementFamily| {
            (self.mesh.element_index(id))
                .filter(|&index| self.mesh.elements()[index].shape.family() == family)
        };
        let elements = |section: &plx_model::Section| match &section.region {
            plx_model::Region::ElementSet(set) if self.results.is_some() => {
                (self.input_sections.sets.get(set).cloned()).unwrap_or_default()
            }
            region => region.elements(&self.mesh),
        };
        for section in &self.thick_shells {
            let (plx_model::SectionKind::Shell { thickness, offset }
            | plx_model::SectionKind::Membrane { thickness, offset }) = section.kind
            else {
                continue;
            };
            let shell = ShellThickness { thickness, offset };
            for id in elements(section) {
                if let Some(index) = of_family(id, ElementFamily::Surface) {
                    solids.shells.insert(index, shell);
                }
            }
        }
        for (section, profile) in &self.thick_lines {
            for id in elements(section) {
                let Some(index) = of_family(id, ElementFamily::Line) else {
                    continue;
                };
                let shape = match &section.kind {
                    plx_model::SectionKind::Truss { area } => BeamShape::truss(*area),
                    plx_model::SectionKind::Beam(beam) => profile.as_ref().map(|profile| {
                        let element = &self.mesh.elements()[index];
                        let tangent = plx_model::line_tangent(&self.mesh, element);
                        // The solver refuses a normal along the beam; the model checks say so.
                        let normal = (beam.orientation.normal_for(tangent))
                            .or_else(|| BeamOrientation::Automatic.normal_for(tangent))
                            .unwrap_or([0.0, 0.0, 1.0]);
                        BeamShape {
                            profile: profile.clone(),
                            normal: DVec3::from(normal),
                            offset: beam.offset,
                        }
                    }),
                    _ => None,
                };
                if let Some(shape) = shape {
                    solids.lines.insert(index, shape);
                }
            }
        }
        solids
    }

    /// Adds the solids of the thick shells and lines of a part to its mesh; `highlight`
    /// colours them like the part's highlighted faces.
    fn append_solids(
        &self,
        mesh: &mut RenderMesh,
        index: usize,
        coords: &[[f64; 3]],
        scalars: Option<&[f32]>,
        solids: &SolidElements,
        highlight: bool,
    ) {
        let (info, skin) = (&self.parts[index], &self.skins[index]);
        if !solids.shells.is_empty() {
            let faces = self.face_highlight(index, skin);
            mesh.append(thick_shell_mesh(
                coords,
                skin,
                self.origin,
                info.color,
                |face| faces(face).filter(|_| highlight),
                |face| solids.shells.get(&skin.faces[face].element).copied(),
                SMOOTH_ANGLE_DEG,
                scalars,
            ));
        }
        if !solids.lines.is_empty() {
            let whole = highlight && self.highlight.parts.contains(&index);
            mesh.append(beam_mesh(
                coords,
                skin,
                self.origin,
                info.color,
                |_| whole.then_some(HIGHLIGHT_COLOR),
                |segment| solids.lines.get(&skin.line_elements[segment]).cloned(),
                scalars,
            ));
        }
    }

    /// Section faces of all parts where a plane in model coordinates cuts them, deformed and
    /// coloured like the parts.
    pub fn section_meshes(
        &self,
        point: DVec3,
        normal: DVec3,
        lighten_colors: bool,
    ) -> Vec<RenderMesh> {
        let cells = self.section_cells.get_or_init(|| {
            self.mesh
                .parts
                .iter()
                .map(|part| SectionCells::new(&self.mesh, part))
                .collect()
        });
        let (coords, scalars, _) = self.shown_state();
        let color = |info: &PartInfo| {
            if lighten_colors {
                lighten(info.color)
            } else {
                info.color
            }
        };
        let lic = self.lic_values();
        let mut meshes: Vec<RenderMesh> = cells
            .iter()
            .zip(&self.parts)
            .enumerate()
            .map(|(index, (cells, info))| {
                let color = color(info);
                let mut mesh = section_mesh(
                    cells,
                    &coords,
                    self.origin,
                    point,
                    normal,
                    color,
                    scalars.as_deref(),
                );
                if let Some((values, _)) = &lic
                    && self.shows_lic(index)
                {
                    mesh.set_directions(values);
                }
                mesh
            })
            .collect();
        // The copies are cut by the same plane.
        for (instance, scalars) in self.transformed_copies() {
            let moved = transform_coords(&coords, &instance);
            let parts = meshes.iter_mut().zip(cells).zip(&self.parts);
            for (index, ((mesh, cells), info)) in parts.enumerate() {
                let color = color(info);
                let scalars = scalars.as_deref();
                let mut copy =
                    section_mesh(cells, &moved, self.origin, point, normal, color, scalars);
                if let Some((values, tensor)) = &lic
                    && self.shows_lic(index)
                {
                    copy.set_directions(&crate::lic::turned(values, &instance, *tensor));
                }
                mesh.append(copy);
            }
        }
        meshes
    }

    /// The shown result values where a plane in model coordinates cuts the visible parts:
    /// of the parts as shown, or of the undeformed mesh, for values at true positions.
    pub fn section_values(
        &self,
        point: DVec3,
        normal: DVec3,
        shown: bool,
    ) -> Option<SectionValues> {
        let values = self.results.as_ref()?.shown_values()?;
        let cells = self.section_cells.get_or_init(|| {
            self.mesh
                .parts
                .iter()
                .map(|part| SectionCells::new(&self.mesh, part))
                .collect()
        });
        let coords = if shown {
            self.deformed_coords().0
        } else {
            std::borrow::Cow::Borrowed(self.mesh.coords())
        };
        (cells.iter().zip(&self.parts))
            .filter(|(_, info)| info.visible)
            .map(|(cells, _)| section_values(cells, &coords, point, normal, &values))
            .reduce(|mut all, part| {
                all.merge(part);
                all
            })
    }

    /// The model origin in global coordinates; render positions are relative to it.
    pub fn origin(&self) -> DVec3 {
        self.origin
    }

    /// Draws the outline of an outlined part red and wide; a highlighted part whose faces turn
    /// red gets its line elements red and wide, as they have no faces to colour.
    fn highlight_outline(&self, part: usize, skin: &PartSkin, mesh: &mut RenderMesh) {
        let red = |vertex: &Vertex| Vertex {
            color: HIGHLIGHT_COLOR,
            ..*vertex
        };
        if self.highlight.outlines.contains(&part) {
            mesh.wide_edges = mesh.feature_edges.iter().map(red).collect();
        } else if self.highlight.parts.contains(&part) {
            // The segments of line elements come last in the feature edges.
            let lines = mesh.feature_edges.len() - 2 * skin.lines.len();
            mesh.wide_edges = mesh.feature_edges[lines..].iter().map(red).collect();
        }
    }

    /// The colour of each highlighted skin face of a part, by its index in the skin.
    fn face_highlight(&self, part: usize, skin: &PartSkin) -> impl Fn(usize) -> Option<[f32; 3]> {
        let whole = self.highlight.parts.contains(&part);
        let none = self.highlight.faces.is_empty() && self.highlight.secondary_faces.is_empty();
        let elements = self.mesh.elements();
        move |index| {
            if whole {
                return Some(HIGHLIGHT_COLOR);
            }
            if none {
                return None;
            }
            let face = &skin.faces[index];
            let key = (elements[face.element].id, face.face as u8 + 1);
            if self.highlight.faces.contains(&key) {
                Some(HIGHLIGHT_COLOR)
            } else {
                (self.highlight.secondary_faces.contains(&key)).then_some(SECONDARY_HIGHLIGHT_COLOR)
            }
        }
    }

    /// All element faces on the surface of the parts.
    pub fn skin_faces(&self) -> impl Iterator<Item = (ElementId, u8)> + '_ {
        let elements = self.mesh.elements();
        self.skins
            .iter()
            .flat_map(|skin| &skin.faces)
            .map(|f| (elements[f.element].id, f.face as u8 + 1))
    }

    /// The segments of all line elements (beams, trusses, CAD edges) with their element.
    pub fn line_segments(&self) -> impl Iterator<Item = (ElementId, [NodeId; 2])> + '_ {
        let (elements, ids) = (self.mesh.elements(), self.mesh.node_ids());
        (self.skins.iter())
            .flat_map(|skin| skin.lines.iter().zip(&skin.line_elements))
            .map(move |(&[a, b], &element)| (elements[element].id, [ids[a], ids[b]]))
    }

    /// All element faces on the surface with the ids of their corner nodes.
    pub fn skin_faces_with_corners(
        &self,
    ) -> impl Iterator<Item = ((ElementId, u8), Vec<NodeId>)> + '_ {
        let (elements, ids) = (self.mesh.elements(), self.mesh.node_ids());
        self.skins
            .iter()
            .flat_map(|skin| &skin.faces)
            .map(move |f| {
                let corners = f.corners.iter().map(|&n| ids[n]).collect();
                ((elements[f.element].id, f.face as u8 + 1), corners)
            })
    }

    pub fn skin(&self, part: usize) -> &PartSkin {
        &self.skins[part]
    }

    /// Skins of all parts, in the order of the mesh's parts.
    pub fn skins(&self) -> &[PartSkin] {
        &self.skins
    }

    /// Position of a node relative to the model origin as drawn, where picking happens.
    pub fn render_position(&self, index: usize) -> Vec3 {
        self.node_position(index).unwrap_or(Vec3::ZERO)
    }

    /// All nodes of the visible parts.
    pub fn visible_nodes(&self) -> BTreeSet<NodeId> {
        let elements = self.mesh.elements();
        self.parts
            .iter()
            .zip(&self.mesh.parts)
            .filter(|(info, _)| info.visible)
            .flat_map(|(_, part)| &part.elements)
            .filter_map(|&id| self.mesh.element_index(id))
            .flat_map(|index| elements[index].nodes.iter().copied())
            .collect()
    }

    /// All surface faces of the visible parts.
    pub fn visible_faces(&self) -> BTreeSet<(ElementId, u8)> {
        let elements = self.mesh.elements();
        self.skins
            .iter()
            .zip(&self.parts)
            .filter(|(_, info)| info.visible)
            .flat_map(|(skin, _)| &skin.faces)
            .map(|f| (elements[f.element].id, f.face as u8 + 1))
            .collect()
    }

    /// Edges of 2D elements and shells on the outline of their part, as (part, (element,
    /// CalculiX face number), corner and midside nodes): the faces of 2D models, and the
    /// free edges of shells.
    pub fn outline_edges(&self) -> Vec<(usize, (ElementId, u8), Vec<NodeId>)> {
        let mut edges = Vec::new();
        for (index, part) in self.mesh.parts.iter().enumerate() {
            let mut count: std::collections::HashMap<(NodeId, NodeId), usize> =
                std::collections::HashMap::new();
            let mut found = Vec::new();
            for element in part.elements.iter().filter_map(|&id| self.mesh.element(id)) {
                for (number, edge) in element.edge_faces() {
                    let [a, b] = [edge.corners[0], edge.corners[1]].map(|l| element.nodes[l]);
                    *count.entry((a.min(b), a.max(b))).or_default() += 1;
                    let mut nodes = vec![a, b];
                    if element.shape.is_quadratic() {
                        nodes.extend(edge.mids.iter().map(|&l| element.nodes[l]));
                    }
                    found.push(((element.id, number), nodes));
                }
            }
            for (face, nodes) in found {
                let key = (nodes[0].min(nodes[1]), nodes[0].max(nodes[1]));
                if count[&key] == 1 {
                    edges.push((index, face, nodes));
                }
            }
        }
        edges
    }

    /// Whether the model consists of 2D elements, whose faces are their edges.
    pub fn is_plane(&self) -> bool {
        self.mesh.elements().first().is_some_and(|e| e.is_plane())
    }

    /// Renames a part as CalculiX needs it: upper case, unique among parts, sets and
    /// surfaces. Sections, boundary conditions and loads on the part follow.
    pub fn rename_part(&mut self, index: usize, name: &str) -> Result<(), String> {
        let name = name.trim().to_ascii_uppercase();
        if name.is_empty() {
            return Err("The name must not be empty.".into());
        }
        if name.len() > 80 {
            return Err("The name must be at most 80 characters long.".into());
        }
        if !(name.chars()).all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-') {
            return Err("Allowed are letters without accents, digits, _ and -.".into());
        }
        let current = self.parts.get(index).ok_or("The part no longer exists.")?;
        if current.name == name {
            return Ok(());
        }
        if self.mesh.name_in_use(&name) {
            return Err(format!("The name {name} is already used."));
        }
        if self.history.follows() {
            self.history.mesh_replaced(self.mesh.clone());
        }
        if let Some(old) = self.mesh.rename_part(index, &name) {
            self.fe.rename_part(&old, &name);
        }
        self.parts[index].name = name;
        Ok(())
    }

    /// Gives the elements of the part that have the shape of `type_name` this type,
    /// as the part properties do; the choice is kept for the part when it is meshed anew.
    pub fn set_element_type(&mut self, index: usize, type_name: &str) -> Result<(), String> {
        let part = self.parts.get(index).ok_or("The part no longer exists.")?;
        let part = part.name.clone();
        if !self.fe.set_element_type(&part, type_name) {
            return Err(format!("{type_name} is not an element type of a part"));
        }
        self.edit_mesh(|_, _| {});
        Ok(())
    }

    /// The nearest visible face hit by a ray, both relative to the model origin.
    pub fn pick(&self, origin: Vec3, direction: Vec3) -> Option<Hit> {
        let coords = self.shown_coords();
        let position = |node: usize| (DVec3::from(coords[node]) - self.origin).as_vec3();
        let mut best: Option<(f32, Hit)> = None;
        // Only the parts whose box the ray passes, nearest first, until a hit lies in front
        // of the next box: a click in a large assembly tests the faces of a few parts.
        for (entry, part) in self.parts_along_ray(&coords, origin, direction, |_| 0.0) {
            if best.is_some_and(|(nearest, _)| nearest < entry) {
                break;
            }
            let skin = &self.skins[part];
            for (index, face) in skin.faces.iter().enumerate() {
                let a = position(face.corners[0]);
                for pair in face.corners[1..].windows(2) {
                    let (b, c) = (position(pair[0]), position(pair[1]));
                    let Some(t) = ray_triangle(origin, direction, a, b, c) else {
                        continue;
                    };
                    let point = origin + direction * t;
                    if self.clip.is_some_and(|clip| clip.distance(point) < 0.0) {
                        continue;
                    }
                    if best.is_none_or(|(nearest, _)| t < nearest) {
                        best = Some((
                            t,
                            Hit {
                                part,
                                face: index,
                                point,
                                line: false,
                            },
                        ));
                    }
                }
            }
        }
        best.map(|(_, hit)| hit)
    }

    /// The nearest visible line segment within the pick tolerance of a ray; `tolerance`
    /// gives the tolerance at a point of the ray. Returns the distance along the ray too.
    pub fn pick_line(
        &self,
        origin: Vec3,
        direction: Vec3,
        tolerance: impl Fn(Vec3) -> f32,
    ) -> Option<(f32, Hit)> {
        let coords = self.shown_coords();
        let position = |node: usize| (DVec3::from(coords[node]) - self.origin).as_vec3();
        let u = direction.normalize_or_zero();
        let mut best: Option<(f32, Hit)> = None;
        for (entry, part) in self.parts_along_ray(&coords, origin, u, &tolerance) {
            if best.is_some_and(|(nearest, _)| nearest < entry) {
                break;
            }
            let skin = &self.skins[part];
            for (index, &[a, b]) in skin.lines.iter().enumerate() {
                let (a, b) = (position(a), position(b));
                let Some((t, s)) = ray_segment(origin, u, a, b) else {
                    continue;
                };
                let on_ray = origin + u * t;
                let on_line = a + (b - a) * s;
                if on_ray.distance(on_line) > tolerance(on_ray)
                    || self.clip.is_some_and(|clip| clip.distance(on_line) < 0.0)
                {
                    continue;
                }
                if best.is_none_or(|(nearest, _)| t < nearest) {
                    best = Some((
                        t,
                        Hit {
                            part,
                            face: index,
                            point: on_line,
                            line: true,
                        },
                    ));
                }
            }
        }
        best
    }

    /// The visible parts whose bounding box a ray passes, with the distance along the ray
    /// (in units of `direction`) at which it enters the box, nearest first. Boxes are
    /// widened by `tolerance`, the pick tolerance at a point of the ray.
    fn parts_along_ray(
        &self,
        coords: &[[f64; 3]],
        origin: Vec3,
        direction: Vec3,
        tolerance: impl Fn(Vec3) -> f32,
    ) -> Vec<(f32, usize)> {
        let skin_nodes = &self.skin_lookup().nodes;
        let unit = direction.normalize_or_zero();
        let mut parts: Vec<(f32, usize)> = (self.parts.iter().enumerate())
            .filter(|(index, info)| info.visible && *index < self.skins.len())
            .filter_map(|(index, _)| {
                let nodes = skin_nodes.get(index)?;
                let (min, max) = (nodes.iter())
                    .map(|&n| (DVec3::from(coords[n]) - self.origin).as_vec3())
                    .fold(None, |bounds: Option<(Vec3, Vec3)>, p| {
                        Some(bounds.map_or((p, p), |(lo, hi)| (lo.min(p), hi.max(p))))
                    })?;
                // The tolerance grows at most linearly along the ray, so its larger value
                // at the ends of the stretch through the box bounds it; doubled, it also
                // covers points of the ray a tolerance beyond the box.
                let far = (0..8)
                    .map(|k| {
                        let corner = Vec3::select(
                            glam::BVec3::new(k & 1 != 0, k & 2 != 0, k & 4 != 0),
                            max,
                            min,
                        );
                        (corner - origin).dot(unit)
                    })
                    .fold(0.0f32, f32::max);
                let margin = 2.0 * tolerance(origin).max(tolerance(origin + unit * far))
                    // Rounding of the positions, also for flat boxes.
                    + 1e-5 * (max - min).length();
                let entry = ray_box(origin, direction, min - margin, max + margin)?;
                Some((entry, index))
            })
            .collect();
        parts.sort_by(|a, b| a.0.total_cmp(&b.0));
        parts
    }

    /// What a click hits: the nearest face, or a line in front of it or where no face is.
    pub fn pick_click(&self, click: &crate::viewport::Click) -> Option<Hit> {
        let face = self.pick(click.origin, click.direction);
        let line = self.pick_line(click.origin, click.direction, |p| click.precision_at(p));
        match (face, line) {
            (Some(face), Some((t, line))) => {
                let face_t = (face.point - click.origin).dot(click.direction.normalize_or_zero());
                // A line on a face, such as a CAD edge, wins by the pick tolerance.
                Some(if t <= face_t + click.precision_at(line.point) {
                    line
                } else {
                    face
                })
            }
            (face, line) => face.or(line.map(|(_, hit)| hit)),
        }
    }

    /// The node of the hit face or line segment nearest to the hit point.
    pub fn hit_node(&self, hit: &Hit) -> NodeId {
        let coords = self.shown_coords();
        let distance = |node: usize| {
            ((DVec3::from(coords[node]) - self.origin).as_vec3() - hit.point).length_squared()
        };
        let skin = &self.skins[hit.part];
        let nearest = if hit.line {
            let [a, b] = skin.lines[hit.face];
            if distance(a) <= distance(b) { a } else { b }
        } else {
            let face = &skin.faces[hit.face];
            face.corners
                .iter()
                .chain(&face.mids)
                .copied()
                .min_by(|&a, &b| distance(a).total_cmp(&distance(b)))
                .unwrap_or(face.corners[0])
        };
        self.mesh.node_ids()[nearest]
    }

    /// The element (by index) whose face or line segment was hit.
    pub fn hit_element(&self, hit: &Hit) -> Option<usize> {
        let skin = self.skins.get(hit.part)?;
        if hit.line {
            skin.line_elements.get(hit.face).copied()
        } else {
            skin.faces.get(hit.face).map(|f| f.element)
        }
    }

    /// The edges of the hit face, or the hit line segment, in render coordinates.
    pub fn hit_outline(&self, hit: &Hit) -> Vec<[Vec3; 2]> {
        let Some(skin) = self.skins.get(hit.part) else {
            return Vec::new();
        };
        if hit.line {
            return (skin.lines.get(hit.face).iter())
                .filter_map(|&&[a, b]| Some([self.node_position(a)?, self.node_position(b)?]))
                .collect();
        }
        skin.faces
            .get(hit.face)
            .map_or_else(Vec::new, |face| self.face_outline(&face.corners))
    }

    /// The visible edges of an element in render coordinates: its skin faces and line
    /// segments.
    pub fn element_outline(&self, element: usize) -> Vec<[Vec3; 2]> {
        let mut edges = Vec::new();
        for skin in &self.skins {
            for face in skin.faces.iter().filter(|f| f.element == element) {
                edges.extend(self.face_outline(&face.corners));
            }
            for (segment, &owner) in skin.lines.iter().zip(&skin.line_elements) {
                if owner == element
                    && let (Some(a), Some(b)) = (
                        self.node_position(segment[0]),
                        self.node_position(segment[1]),
                    )
                {
                    edges.push([a, b]);
                }
            }
        }
        edges
    }

    fn face_outline(&self, corners: &[usize]) -> Vec<[Vec3; 2]> {
        (0..corners.len())
            .filter_map(|i| {
                let (a, b) = (corners[i], corners[(i + 1) % corners.len()]);
                Some([self.node_position(a)?, self.node_position(b)?])
            })
            .collect()
    }

    /// Where a node is drawn, in model coordinates: with the exploded view and the shown
    /// deformation.
    pub fn shown_node(&self, id: NodeId) -> Option<[f64; 3]> {
        let index = self.mesh.node_index(id)?;
        self.shown_coords().get(index).copied()
    }

    /// The unit system the model shows its values in: of the results, or of the FE model.
    pub fn units(&self) -> UnitSystem {
        self.results
            .as_ref()
            .map_or(self.fe.properties.units, |v| v.units)
    }

    /// The unit system the model's values are stored in: SI units for an FE model; results
    /// are converted into the units they show when they are read.
    pub fn stored_units(&self) -> UnitSystem {
        match &self.results {
            Some(view) => view.units,
            None => plx_model::STORED,
        }
    }

    /// A value of the model as it is shown.
    pub fn shown(&self, value: f64, quantity: plx_model::Quantity) -> f64 {
        self.stored_units().convert(value, quantity, self.units())
    }

    /// Where a node is drawn, relative to the model origin, including the shown deformation.
    pub fn node_position(&self, index: usize) -> Option<Vec3> {
        let mut p = DVec3::from(*self.mesh.coords().get(index)?) + self.explosion_offset(index);
        if let Some(view) = &self.results {
            let scale = (view.scale() * view.amplitude()) as f64;
            let displacement = view.shown_displacement(index);
            if let (Some(d), true) = (displacement, scale != 0.0) {
                p += scale * DVec3::new(d[0] as f64, d[1] as f64, d[2] as f64);
            }
        }
        Some((p - self.origin).as_vec3())
    }

    /// Where a node is drawn on the given item of the results ([`ResultsView::instances`]).
    pub fn node_position_on(&self, index: usize, item: usize) -> Option<Vec3> {
        let position = self.node_position(index)?;
        if item == 0 {
            return Some(position);
        }
        let instance = *self.results.as_ref()?.instances().get(item)?;
        let global = position.as_dvec3() + self.origin;
        Some((instance.transform_point3(global) - self.origin).as_vec3())
    }

    /// A point given in model coordinates, in render coordinates.
    pub fn to_render(&self, point: [f64; 3]) -> Vec3 {
        (DVec3::from(point) - self.origin).as_vec3()
    }

    /// A point in render coordinates, in model coordinates.
    pub fn model_point(&self, point: Vec3) -> DVec3 {
        point.as_dvec3() + self.origin
    }

    /// The global origin in render coordinates.
    pub fn global_origin(&self) -> Vec3 {
        (-self.origin).as_vec3()
    }

    pub fn file_name(&self) -> String {
        self.path.file_name().map_or_else(
            || self.path.display().to_string(),
            |n| n.to_string_lossy().into_owned(),
        )
    }

    /// Bounding box of all visible parts, relative to the model origin.
    pub fn visible_bounds(&self) -> Option<(Vec3, Vec3)> {
        self.parts
            .iter()
            .filter(|p| p.visible)
            .filter_map(|p| p.bounds)
            .reduce(|(min_a, max_a), (min_b, max_b)| (min_a.min(min_b), max_a.max(max_b)))
    }
}

fn transform_coords(coords: &[[f64; 3]], instance: &DAffine3) -> Vec<[f64; 3]> {
    (coords.iter())
        .map(|&p| instance.transform_point3(DVec3::from(p)).to_array())
        .collect()
}

/// Closest approach of a ray (unit direction `u`, `t >= 0`) and a segment `a`-`b`: the
/// distance `t` along the ray and the parameter `s` in 0..=1 along the segment. `None` when
/// the closest point of the segment lies behind the ray's origin.
fn ray_segment(origin: Vec3, u: Vec3, a: Vec3, b: Vec3) -> Option<(f32, f32)> {
    let v = b - a;
    let w = origin - a;
    let (uv, vv, uw, vw) = (u.dot(v), v.dot(v), u.dot(w), v.dot(w));
    let denominator = vv - uv * uv;
    let s = if denominator <= f32::EPSILON * vv.max(f32::MIN_POSITIVE) {
        // Parallel: the segment's start is as close as any point of it.
        0.0
    } else {
        ((vw - uv * uw) / denominator).clamp(0.0, 1.0)
    };
    let t = (a + v * s - origin).dot(u);
    (t >= 0.0).then_some((t, s))
}

/// Distance along the ray (`t >= 0`) at which it enters a box, 0 when it starts inside.
fn ray_box(origin: Vec3, direction: Vec3, min: Vec3, max: Vec3) -> Option<f32> {
    let (mut near, mut far) = (0.0f32, f32::INFINITY);
    for axis in 0..3 {
        let (o, d) = (origin[axis], direction[axis]);
        if d == 0.0 {
            if o < min[axis] || o > max[axis] {
                return None;
            }
            continue;
        }
        let (a, b) = ((min[axis] - o) / d, (max[axis] - o) / d);
        near = near.max(a.min(b));
        far = far.min(a.max(b));
        if near > far {
            return None;
        }
    }
    Some(near)
}

/// Distance along the ray to a triangle (Möller-Trumbore), seen from either side.
fn ray_triangle(origin: Vec3, direction: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Option<f32> {
    let (ab, ac) = (b - a, c - a);
    let p = direction.cross(ac);
    let det = ab.dot(p);
    if det.abs() < f32::EPSILON * ab.length() * ac.length() {
        return None;
    }
    let s = origin - a;
    let u = s.dot(p) / det;
    let q = s.cross(ab);
    let v = direction.dot(q) / det;
    let t = ac.dot(q) / det;
    (u >= 0.0 && v >= 0.0 && u + v <= 1.0 && t >= 0.0).then_some(t)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Results of the built-in solver open from their `.ndlr` file like CalculiX's from the
    /// `.frd`: with the parts of the input file beside them and the history output, without
    /// any `.frd` or `.dat` file.
    #[test]
    fn results_of_the_built_in_solver_open_without_a_frd_file() {
        let dir = std::env::temp_dir().join(format!("plx-ndlr-{}", std::process::id()));
        let path = crate::solver_check::solve_beam_built_in(&dir);
        assert!(plx_results::solver::is_solver_results(&path));
        assert!(!path.with_extension("frd").exists() && !path.with_extension("dat").exists());
        let loaded = load(&path, UnitSystem::Unitless).unwrap();
        let view = loaded.model.results.as_ref().unwrap();
        let disp = view.increments[0].field("DISP").unwrap();
        assert!(disp.component("ALL").is_some());
        let read = plx_io::inp::read_inp(&path.with_extension("inp")).unwrap();
        let names = |parts: &[plx_mesh::Part]| -> Vec<String> {
            parts.iter().map(|p| p.name.clone()).collect()
        };
        assert_eq!(names(&loaded.model.mesh.parts).len(), read.mesh.parts.len());
        assert!(!view.history.is_empty(), "{:?}", loaded.model.warnings);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn general_beams_are_taken_from_the_input_without_their_section_forces() {
        let node_mesh = || {
            let mut mesh = FeMesh::default();
            for id in 1..=4 {
                mesh.set_node(id, [f64::from(id), 0.0, 0.0]);
            }
            mesh
        };
        // The results hold a truss on nodes 3 and 4, the input two U1 beams besides it.
        let mut mesh = node_mesh();
        let element = |id, type_name: &str, nodes: Vec<u32>| plx_mesh::Element {
            id,
            type_name: type_name.into(),
            shape: plx_mesh::ElementShape::Line2,
            nodes,
        };
        mesh.add_element(element(3, "T3D2", vec![3, 4])).unwrap();
        let mut input = node_mesh();
        input.add_element(element(1, "U1", vec![1, 2])).unwrap();
        input.add_element(element(2, "U1", vec![2, 3])).unwrap();
        input.add_element(element(3, "T3D2", vec![3, 4])).unwrap();
        let component = |name: &str| plx_results::Component {
            name: name.into(),
            values: vec![1.0; 4],
            derived: false,
        };
        let stress = ["SXX", "SYY", "SZZ", "SXY", "SYZ", "SZX"]
            .map(component)
            .to_vec();
        let mut increments = vec![plx_results::Increment {
            step: 1,
            increment: 1,
            kind: plx_results::AnalysisKind::Static,
            value: 1.0,
            fields: vec![
                plx_results::Field::new("STRESS", stress),
                plx_results::Field::new("DISP", ["D1", "D2", "D3"].map(component).to_vec()),
            ],
            nodal_diameter: None,
        }];
        adopt_general_beams(&mut mesh, &mut increments, &input);
        assert_eq!(mesh.element_count(), 3);
        assert_eq!(mesh.element(1).unwrap().type_name, "U1");
        let sxx = &increments[0].fields[0].components[0].values;
        // Node 3 belongs to the truss too and keeps its stress.
        assert!(sxx[0].is_nan() && sxx[1].is_nan());
        assert_eq!(sxx[2..], [1.0, 1.0]);
        assert_eq!(increments[0].fields[1].components[0].values, vec![1.0; 4]);
    }

    fn testdata(name: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name)
    }

    #[test]
    fn loads_testdata_with_part_summaries() {
        let loaded = load(&testdata("platte_mit_stuetzen.inp"), UnitSystem::Unitless).unwrap();
        let model = &loaded.model;
        assert_eq!(model.included_files, 1);
        assert_eq!(model.parts.len(), 2);
        assert_eq!(model.parts[0].element_types, [("S4R".to_string(), 16)]);
        assert_eq!(model.parts[0].node_count, 25);
        assert_eq!(model.parts[1].node_count, 4);
        assert_eq!(loaded.render_meshes.len(), 2);
        let (min, max) = model.visible_bounds().unwrap();
        assert_eq!(max - min, Vec3::new(20.0, 20.0, 15.0));
    }

    #[test]
    fn highlighted_line_parts_are_drawn_red_and_wide() {
        let mut model = load(&testdata("rahmen_b32.inp"), UnitSystem::Unitless)
            .unwrap()
            .model;
        assert!(model.render_meshes()[0].wide_edges.is_empty());
        // A section region of parts colours the faces, of which line parts have none.
        model.highlight.parts.insert(0);
        let meshes = model.render_meshes();
        let segments = model.skin(0).lines.len();
        assert!(segments > 0);
        assert_eq!(meshes[0].wide_edges.len(), 2 * segments);
        assert!(
            meshes[0]
                .wide_edges
                .iter()
                .all(|v| v.color == HIGHLIGHT_COLOR)
        );
        assert!(meshes[1].wide_edges.is_empty());
        // Nodes and element sets on lines show their segments.
        let nodes = model.mesh.node_ids()[..3].to_vec();
        let region = plx_model::Region::Nodes(nodes);
        let highlight = crate::setup::region_highlight(&model, &region);
        assert!(!highlight.lines.is_empty());
    }

    #[test]
    fn renamed_parts_keep_their_references() {
        let mut model = load(&testdata("platte_mit_stuetzen.inp"), UnitSystem::Unitless)
            .unwrap()
            .model;
        let (first, second) = (model.parts[0].name.clone(), model.parts[1].name.clone());
        model.fe.sections.push(plx_model::Section {
            name: "Section-1".into(),
            material: "Steel".into(),
            region: plx_model::Region::Parts(vec![first.clone()]),
            thickness: 1.0,
            kind: SectionKind::Solid,
        });
        assert!(model.rename_part(0, "").is_err());
        assert!(model.rename_part(0, "zwei wörter").is_err());
        assert!(model.rename_part(0, &second.to_lowercase()).is_err());
        assert_eq!(model.rename_part(0, " Platte-1 "), Ok(()));
        assert_eq!(model.parts[0].name, "PLATTE-1");
        assert_eq!(model.mesh.parts[0].name, "PLATTE-1");
        assert!(model.mesh.element_sets.contains_key("PLATTE-1"));
        assert!(!model.mesh.element_sets.contains_key(&first));
        assert_eq!(
            model.fe.sections[0].region,
            plx_model::Region::Parts(vec!["PLATTE-1".into()])
        );
        assert_eq!(model.rename_part(0, "platte-1"), Ok(()));
    }

    #[test]
    fn chosen_element_types_survive_a_new_mesh() {
        let mut model = load(&testdata("kragbalken_c3d8.inp"), UnitSystem::Unitless)
            .unwrap()
            .model;
        let original = model.mesh.clone();
        assert!(model.set_element_type(0, "C3D10").is_ok());
        assert!(model.set_element_type(0, "XYZ").is_err());
        assert_eq!(model.set_element_type(0, "c3d8i"), Ok(()));
        assert_eq!(model.parts[0].element_types, [("C3D8I".to_string(), 40)]);
        // Meshed anew with the type the mesher gives, the part gets its choice again.
        model.set_mesh(original);
        assert_eq!(model.parts[0].element_types, [("C3D8I".to_string(), 40)]);
    }

    /// Saves the model as the only analysis of a project and opens that analysis again.
    fn save_and_reopen(path: &Path, model: &Model) -> LoadedModel {
        let project = plx_io::project::Project {
            nodes: vec![plx_io::project::ProjectNode::Analysis(Box::new(
                plx_io::project::ProjectAnalysis {
                    id: 1,
                    name: "Analysis-1".into(),
                    geometry: model.geometry.clone(),
                    mesh: model.mesh.clone(),
                    model: model.fe.clone(),
                    ..Default::default()
                },
            ))],
            ..Default::default()
        };
        plx_io::project::save_project(path, &project).unwrap();
        let project = plx_io::project::read_project(path).unwrap();
        load_analysis(path, project.analyses().next().unwrap().clone())
    }

    #[test]
    fn projects_open_with_their_fe_model() {
        let mut model = load(&testdata("platte_mit_stuetzen.inp"), UnitSystem::Unitless)
            .unwrap()
            .model;
        model.fe.steps.push(plx_model::Step::new_static("Step-1"));
        let dir = std::env::temp_dir().join(format!("nodalis-model-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("platte.ndl");
        let loaded = save_and_reopen(&path, &model);
        std::fs::remove_dir_all(&dir).unwrap();
        assert_eq!(loaded.model.path, path);
        assert!(!loaded.model.is_results());
        assert_eq!(loaded.model.fe, model.fe);
        assert_eq!(loaded.model.parts.len(), 2);
    }

    #[test]
    fn hidden_parts_do_not_count_for_fitting() {
        let mut model = load(&testdata("platte_mit_stuetzen.inp"), UnitSystem::Unitless)
            .unwrap()
            .model;
        model.parts[1].visible = false;
        let (min, max) = model.visible_bounds().unwrap();
        assert_eq!(max.z - min.z, 0.0);
    }

    #[test]
    fn global_contours_add_the_deformed_edges_of_the_global_model_only_when_shown() {
        let frd = testdata("kragbalken_c3d8.frd");
        let mut model = load(&frd, UnitSystem::Unitless).unwrap().model;
        let global = load(&frd, UnitSystem::Unitless).unwrap().model;
        let parts = model.render_meshes().len();
        model.global = GlobalModel::new(global, 1).map(Box::new);
        assert_eq!(model.scene_meshes().len(), parts);
        model.results.as_mut().unwrap().show_global_contours = true;
        let meshes = model.scene_meshes();
        assert_eq!(meshes.len(), parts + 1);
        let wireframe = &meshes[parts];
        assert!(wireframe.vertices.is_empty() && !wireframe.wireframe_edges.is_empty());
        // Deformed like the results: the beam bends below its undeformed bottom at z = -5,
        // relative to the origin of the results.
        let lowest = (wireframe.wireframe_edges.iter())
            .map(|v| v.position[2])
            .fold(f32::MAX, f32::min);
        assert!(lowest < -5.0, "{lowest}");
    }

    #[test]
    fn loads_frd_results_with_deformation() {
        let loaded = load(&testdata("kragbalken_c3d8.frd"), UnitSystem::Unitless).unwrap();
        let model = &loaded.model;
        // The part of the input next to the results, not the material the file groups by.
        assert_eq!(model.parts[0].name, "EALL");
        let view = model.results.as_ref().unwrap();
        assert_eq!(view.current().unwrap().1.name, "ALL");
        assert!(view.scale() > 1.0);
        // The beam bends downwards, so the deformed bounds reach below the undeformed ones.
        let undeformed_min_z = -5.0;
        let (min, _) = model.visible_bounds().unwrap();
        assert!(min.z < undeformed_min_z, "{min}");
        assert!(
            loaded.render_meshes[0]
                .vertices
                .iter()
                .all(|v| v.scalar >= 0.0)
        );
    }

    #[test]
    fn contact_in_the_input_next_to_the_results_opens_them_in_true_scale() {
        let dir = std::env::temp_dir().join("nodalis_contact_results");
        std::fs::create_dir_all(&dir).unwrap();
        let frd = dir.join("job.frd");
        std::fs::copy(testdata("kragbalken_c3d8.frd"), &frd).unwrap();
        std::fs::write(dir.join("job.inp"), "*CONTACT PAIR, INTERACTION=I\nA, B\n").unwrap();
        let model = load(&frd, UnitSystem::Unitless).unwrap().model;
        std::fs::remove_dir_all(&dir).ok();
        let view = model.results.unwrap();
        assert!(view.contact);
        assert_eq!(view.deformation, crate::results::Deformation::TrueScale);
    }

    #[test]
    fn transformed_copies_are_drawn_with_mirrored_values() {
        use plx_results::transformation::{SymmetryPlane, Transformation};
        let mut model = load(&testdata("kragbalken_c3d8.frd"), UnitSystem::Unitless)
            .unwrap()
            .model;
        let single = model.render_meshes()[0].vertices.len();
        let view = model.results.as_mut().unwrap();
        let fields = &view.current_increment().unwrap().fields;
        let field = fields.iter().position(|f| f.name == "DISP").unwrap();
        let components = &fields[field].components;
        let component = components.iter().position(|c| c.name == "U3").unwrap();
        (view.field, view.component) = (field, component);
        let (min, max) = view.legend().map(|l| (l.min, l.max)).unwrap();
        view.transformations = vec![Transformation::symmetry(SymmetryPlane::Z)];
        // Mirrored at z, the beam bending down shows a copy bending up.
        let legend = view.legend().unwrap();
        assert_eq!((legend.min, legend.max), (min.min(-max), max.max(-min)));
        assert_eq!(view.maximum().map(|m| (m.1, m.2)), Some((-min, 1)));
        assert_eq!(model.render_meshes()[0].vertices.len(), 2 * single);
        let node = model.mesh.coords().len() - 1;
        let (a, b) = (
            model.node_position_on(node, 0).unwrap(),
            model.node_position_on(node, 1).unwrap(),
        );
        let global_z = |p: Vec3| p.z as f64 + model.origin().z;
        assert!((global_z(a) + global_z(b)).abs() < 1e-3, "{a} {b}");
    }

    #[test]
    fn a_new_highlight_redraws_only_the_parts_it_concerns() {
        let mut model = load(&testdata("platte_mit_stuetzen.inp"), UnitSystem::Unitless)
            .unwrap()
            .model;
        assert_eq!(model.parts.len(), 2);
        let selected = Highlight::parts([1]);
        let changed = model.highlight_changed_parts(&model.highlight, &selected);
        assert_eq!(changed, Some(BTreeSet::from([1])));
        model.highlight = selected.clone();
        // The part drawn on its own is the part of the whole scene.
        let scene = model.scene_meshes();
        for (part, whole) in scene.iter().enumerate() {
            let single = model.part_scene_mesh(part).unwrap();
            assert_eq!(single.vertices, whole.vertices);
            assert_eq!(single.wide_edges, whole.wide_edges);
        }
        // A face of the plate concerns the plate; the beams lose their outline.
        let face = model.skin_faces().next().unwrap();
        let faces = Highlight {
            faces: HashSet::from([face]),
            ..Highlight::default()
        };
        let changed = model.highlight_changed_parts(&selected, &faces);
        assert_eq!(changed, Some(BTreeSet::from([0, 1])));
        assert_eq!(
            model.highlight_changed_parts(&faces, &faces),
            Some(BTreeSet::new())
        );
    }

    #[test]
    fn picking_skips_parts_off_the_ray_but_finds_the_nearest_face() {
        let model = load(&testdata("platte_mit_stuetzen.inp"), UnitSystem::Unitless)
            .unwrap()
            .model;
        let coords = model.shown_coords();
        let position = |node: usize| (DVec3::from(coords[node]) - model.origin).as_vec3();
        // The nearest face of all parts, without boxes.
        let nearest = |origin: Vec3, direction: Vec3| {
            (model.skins.iter().flat_map(|skin| &skin.faces))
                .flat_map(|face| {
                    let a = position(face.corners[0]);
                    (face.corners[1..].windows(2)).filter_map(move |pair| {
                        ray_triangle(origin, direction, a, position(pair[0]), position(pair[1]))
                    })
                })
                .min_by(f32::total_cmp)
        };
        let (min, max) = ((0..coords.len()).map(position))
            .fold((Vec3::MAX, Vec3::MIN), |(lo, hi), p| (lo.min(p), hi.max(p)));
        let size = (max - min).length();
        let mut hits = 0;
        for direction in [
            Vec3::NEG_Z,
            Vec3::Z,
            Vec3::NEG_X,
            Vec3::new(-1.0, -0.5, -2.0),
        ] {
            for i in 0..=10 {
                for j in 0..=10 {
                    let (u, v) = (i as f32 / 10.0, j as f32 / 10.0);
                    let target = min + (max - min) * Vec3::new(u, v, 0.5);
                    let origin = target - direction.normalize() * size;
                    let expected = nearest(origin, direction);
                    let hit = model.pick(origin, direction);
                    assert_eq!(hit.is_some(), expected.is_some(), "{origin} {direction}");
                    if let (Some(hit), Some(t)) = (hit, expected) {
                        hits += 1;
                        let found = (hit.point - origin).length() / direction.length();
                        assert!((found - t).abs() <= 1e-4 * size, "{found} {t}");
                    }
                }
            }
        }
        assert!(hits > 50, "{hits}");
    }

    /// Times a click on a part of a large assembly and the redrawn scene with it
    /// highlighted: `NODALIS_PICK_BENCH=file.step cargo test --release -p plx-app
    /// pick_bench -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn pick_bench() {
        use std::time::Instant;
        let Some(path) = std::env::var_os("NODALIS_PICK_BENCH") else {
            return;
        };
        let start = Instant::now();
        let mut view = load(std::path::Path::new(&path), UnitSystem::Unitless)
            .unwrap()
            .geometry_view
            .unwrap();
        let skin_faces: usize = view.skins.iter().map(|s| s.faces.len()).sum();
        eprintln!(
            "loaded in {:?}: {} parts, {} nodes, {} skin faces",
            start.elapsed(),
            view.parts.len(),
            view.mesh.coords().len(),
            skin_faces
        );
        let start = Instant::now();
        let meshes = view.scene_meshes();
        eprintln!("first scene: {:?}", start.elapsed());
        let (min, max) = view.visible_bounds().unwrap();
        // A click straight down onto the middle of each of a few parts, so that the
        // highlight moves from part to part.
        for (round, part) in (0..meshes.len())
            .step_by(meshes.len().div_ceil(6))
            .enumerate()
        {
            let Some((lo, hi)) = meshes[part].bounds() else {
                continue;
            };
            let middle = (lo + hi) * 0.5;
            let origin = Vec3::new(middle.x, middle.y, max.z + (max - min).length());
            let click = crate::viewport::Click {
                origin,
                direction: Vec3::NEG_Z,
                side_origin: origin + Vec3::X * (max - min).length() * 1e-3,
                side_direction: Vec3::NEG_Z,
                shift: false,
                ctrl: false,
            };
            let start = Instant::now();
            let hit = view.pick_click(&click);
            let picked = start.elapsed();
            let start = Instant::now();
            let highlight = Highlight::parts(hit.map(|h| h.part));
            let changed = view
                .highlight_changed_parts(&view.highlight, &highlight)
                .unwrap();
            view.highlight = highlight;
            for &part in &changed {
                view.part_scene_mesh(part).unwrap();
            }
            let partial = start.elapsed();
            let start = Instant::now();
            let all = view.scene_meshes();
            eprintln!(
                "click {round}: pick {picked:?}, parts {changed:?} rebuilt in {partial:?}, \
                 whole scene {:?} ({} meshes)",
                start.elapsed(),
                all.len()
            );
        }
        drop(meshes);
    }

    /// Times deleting half the parts of a large assembly from its geometry:
    /// `NODALIS_DELETE_BENCH=file.step cargo test --release -p plx-app delete_bench --
    /// --ignored --nocapture`.
    #[test]
    #[ignore]
    fn delete_bench() {
        use std::time::Instant;
        let Some(path) = std::env::var_os("NODALIS_DELETE_BENCH") else {
            return;
        };
        let start = Instant::now();
        let loaded = load(std::path::Path::new(&path), UnitSystem::Unitless).unwrap();
        let geometry = loaded.model.geometry.unwrap();
        let view = loaded.geometry_view.unwrap();
        eprintln!(
            "loaded in {:?}: {} parts",
            start.elapsed(),
            view.parts.len()
        );
        let names: Vec<String> = (view.parts.iter().step_by(2))
            .map(|p| p.name.clone())
            .collect();
        let start = Instant::now();
        let (smaller, tags) = plx_mesher::delete_parts_renumbered(&geometry, &names).unwrap();
        eprintln!(
            "{} parts deleted in {:?} ({} tags)",
            names.len(),
            start.elapsed(),
            tags.len()
        );
        let smaller = smaller.unwrap();
        let start = Instant::now();
        let view = view.geometry_view_without(&names, &smaller, &tags).unwrap();
        eprintln!("view kept in {:?}", start.elapsed());
        let start = Instant::now();
        let meshes = view.scene_meshes();
        eprintln!("scene in {:?} ({} meshes)", start.elapsed(), meshes.len());
    }

    #[test]
    fn picking_finds_faces_and_nodes() {
        let model = load(&testdata("kragbalken_c3d8.inp"), UnitSystem::Unitless)
            .unwrap()
            .model;
        // The beam spans 0..100 x 0..10 x 0..10; look straight down at x = 95, y = 5.
        let origin = Vec3::new(95.0, 5.0, 50.0) - model.origin.as_vec3();
        let hit = model.pick(origin, Vec3::NEG_Z).unwrap();
        assert!((hit.point.z + model.origin.z as f32 - 10.0).abs() < 1e-4);
        let node = model.mesh.node(model.hit_node(&hit)).unwrap();
        assert_eq!(node[2], 10.0);
        assert!(model.pick(origin, Vec3::Z).is_none());
    }

    #[test]
    fn picking_finds_lines_within_the_tolerance() {
        use plx_mesh::{Element, ElementShape, Part};
        // A beam of two quadratic lines along x from 0 to 100 at y = z = 0.
        let mut mesh = FeMesh::default();
        for i in 0..5u32 {
            mesh.set_node(i + 1, [25.0 * f64::from(i), 0.0, 0.0]);
        }
        for (id, nodes) in [(1, vec![1, 2, 3]), (2, vec![3, 4, 5])] {
            mesh.add_element(Element {
                id,
                type_name: "B32".into(),
                shape: ElementShape::Line3,
                nodes,
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "BEAM".into(),
            elements: vec![1, 2],
        });
        let model = Model::new(Path::new("balken.inp"), mesh);
        assert!(
            model.pick(Vec3::ZERO, Vec3::NEG_Z).is_none(),
            "lines have no faces"
        );
        // Looking down 2 units beside the beam at x = 60: within a tolerance of 3, not 1.
        let origin = Vec3::new(60.0, 2.0, 50.0) - model.origin.as_vec3();
        let (t, hit) = model
            .pick_line(origin, Vec3::NEG_Z, |_| 3.0)
            .expect("line within tolerance");
        assert!(hit.line && hit.part == 0);
        assert!((t - 50.0).abs() < 1e-4, "{t}");
        assert!((hit.point.x + model.origin.x as f32 - 60.0).abs() < 1e-4);
        // Node 3 at x = 50 is the nearest node of the hit segment (50..75).
        assert_eq!(model.hit_node(&hit), 3);
        assert!(model.pick_line(origin, Vec3::NEG_Z, |_| 1.0).is_none());
        // Behind the origin nothing is hit.
        assert!(model.pick_line(origin, Vec3::Z, |_| 3.0).is_none());
    }

    /// Whether Gmsh can be used; tests that need it pass with a note otherwise, unless
    /// `NODALIS_REQUIRE_GMSH` is set.
    fn gmsh_available() -> bool {
        match plx_mesher::self_test() {
            Ok(_) => true,
            Err(error) if std::env::var_os("NODALIS_REQUIRE_GMSH").is_none() => {
                eprintln!("Gmsh not available, test skipped: {error}");
                false
            }
            Err(error) => panic!("Gmsh is required: {error}"),
        }
    }

    #[test]
    fn step_files_open_as_geometry_and_mesh_into_the_project() {
        if !gmsh_available() {
            return;
        }
        let loaded = load(&testdata("zwei_bloecke.step"), UnitSystem::Unitless).unwrap();
        let view = loaded.geometry_view.unwrap();
        assert!(view.is_geometry());
        assert_eq!(view.parts.len(), 2);
        // The display shows the CAD edges but no triangle edges, and each CAD face is its
        // own patch: a block has six.
        assert_eq!(loaded.render_meshes.len(), 2);
        assert!(loaded.render_meshes[0].mesh_edges.is_empty());
        assert!(!loaded.render_meshes[0].feature_edges.is_empty());
        let patches: BTreeSet<usize> = view.skin(0).faces.iter().map(|f| f.region).collect();
        assert_eq!(patches.len(), 6);

        let mut model = loaded.model;
        assert_eq!(model.mesh.element_count(), 0);
        let geometry = model.geometry.clone().unwrap();
        let mesh = plx_mesher::generate_mesh(&geometry).unwrap().mesh;
        model.set_mesh(mesh);
        assert_eq!(model.parts.len(), 2);
        assert!(model.visible_bounds().is_some());

        // The project keeps geometry and mesh; it opens on its mesh with the geometry beside.
        let dir = std::env::temp_dir().join(format!("nodalis-geometry-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("bloecke.ndl");
        let reopened = save_and_reopen(&path, &model);
        std::fs::remove_dir_all(&dir).unwrap();
        assert_eq!(reopened.model.geometry, Some(geometry));
        assert!(!model.mesh.cad.is_empty());
        assert_eq!(reopened.model.mesh.cad, model.mesh.cad);
        assert_eq!(
            reopened.model.mesh.element_count(),
            model.mesh.element_count()
        );
        assert!(reopened.geometry_view.is_some());
        assert!(!reopened.render_meshes[0].mesh_edges.is_empty());
    }

    #[test]
    fn deleting_geometry_parts_keeps_the_skins_of_the_others() {
        if !gmsh_available() {
            return;
        }
        let path = testdata("zwei_bloecke.step");
        let import = plx_mesher::import_cad(&path, UnitSystem::Unitless).unwrap();
        let deleted = ["SOLID-1".to_string()];
        let (smaller, tags) =
            plx_mesher::delete_parts_renumbered(&import.geometry, &deleted).unwrap();
        let smaller = smaller.unwrap();
        let display = (import.display.clone())
            .without_parts(&deleted, &smaller, &tags)
            .unwrap();
        let made = Model::geometry_view(&path, display);
        let kept = Model::geometry_view(&path, import.display)
            .geometry_view_without(&deleted, &smaller, &tags)
            .unwrap();
        assert_eq!(kept.parts.len(), 1);
        assert_eq!(kept.parts[0].name, "SOLID-2");
        assert_eq!(kept.skins, made.skins);
        assert_eq!(kept.cad_entities, made.cad_entities);
        assert_eq!(kept.mesh.node_ids(), made.mesh.node_ids());
        assert_eq!(kept.parts[0].dimension, Some(3));
        assert_eq!(kept.geometry_solids, 1);
        // Faces of the block that stays take their new tags as regions.
        let regions: BTreeSet<usize> = kept.skin(0).faces.iter().map(|f| f.region).collect();
        assert_eq!(regions.len(), 6);
        assert!(regions.iter().all(|&r| (1..=6).contains(&r)));
    }

    /// Meshes the plate with a hole, sets up a tension test and checks CalculiX's stress.
    #[test]
    fn generated_meshes_solve_with_calculix() {
        if !gmsh_available() {
            eprintln!("Gmsh missing, test skipped");
            return;
        }
        use plx_model::{
            BoundaryCondition, BoundaryKind, Elastic, Material, Region, Section, Step,
        };
        let mut model = load(&testdata("platte_mit_loch.step"), UnitSystem::Unitless)
            .unwrap()
            .model;
        let mut geometry = model.geometry.clone().unwrap();
        geometry.meshing.max_size = 4.0;
        model.set_mesh(plx_mesher::generate_mesh(&geometry).unwrap().mesh);
        let nodes_at = |x: f64| -> Vec<NodeId> {
            (model.mesh.node_ids().iter().zip(model.mesh.coords()))
                .filter(|(_, c)| (c[0] - x).abs() < 1e-6)
                .map(|(id, _)| *id)
                .collect()
        };
        let (fixed, pulled) = (nodes_at(0.0), nodes_at(100.0));
        let mut step = Step::new_static("Step-1");
        step.boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Fix".into(),
            active: true,
            visible: true,
            region: Region::Nodes(fixed),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        step.boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Pull".into(),
            active: true,
            visible: true,
            region: Region::Nodes(pulled),
            kind: BoundaryKind::displacement([Some(0.1), None, None, None, None, None]),
            amplitude: None,
        });
        model.fe = FeModel {
            properties: Default::default(),
            materials: vec![Material {
                name: "Steel".into(),
                density: None,
                elastic: Some(Elastic {
                    young: 210000.0,
                    poisson: 0.3,
                }),
                ..Default::default()
            }],
            sections: vec![Section {
                name: "Section-1".into(),
                material: "Steel".into(),
                region: Region::Parts(vec!["SOLID-1".into()]),
                thickness: 1.0,
                kind: SectionKind::Solid,
            }],
            steps: vec![step],
            ..FeModel::default()
        };
        let inp = plx_io::inp::write_inp(&model.mesh, &model.fe, "Platte").unwrap();
        let dir = std::env::temp_dir().join(format!("nodalis-ccx-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("platte.inp"), inp).unwrap();
        plxs_session::run_input_file(&dir, "platte").unwrap_or_else(|e| panic!("{e}"));
        let results = read_frd(&dir.join("platte.frd")).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        // A strain of 0.1 / 100 gives about 210 MPa in the net section away from the hole;
        // the hole raises the peak, CalculiX's extrapolation keeps it below a few times that.
        let stress = results.increments.last().unwrap().field("STRESS").unwrap();
        let sxx = &stress.component("S11").unwrap().values;
        let max = sxx.iter().copied().fold(f32::MIN, f32::max);
        assert!(max > 300.0 && max < 1500.0, "largest stress {max}");
    }

    /// Shells of a shell section are drawn as the solid of its thickness, placed by its
    /// offset, in 3D FE models only.
    #[test]
    fn shell_sections_are_drawn_thick() {
        let mut mesh = FeMesh::default();
        for (id, c) in [
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [10.0, 10.0, 0.0],
            [0.0, 10.0, 0.0],
        ]
        .into_iter()
        .enumerate()
        {
            mesh.set_node(id as u32 + 1, c);
        }
        mesh.add_element(plx_mesh::Element {
            id: 1,
            type_name: "S4".into(),
            shape: plx_mesh::ElementShape::Quad4,
            nodes: vec![1, 2, 3, 4],
        })
        .unwrap();
        mesh.parts.push(plx_mesh::Part {
            name: "Plate".into(),
            elements: vec![1],
        });
        let mut model = Model::new(Path::new("plate.inp"), mesh);
        model.fe.sections.push(plx_model::Section {
            name: "Shell_Section-1".into(),
            material: "Steel".into(),
            region: plx_model::Region::Parts(vec!["Plate".into()]),
            thickness: 1.0,
            kind: SectionKind::Shell {
                thickness: 2.0,
                offset: 0.5,
            },
        });
        let z_range = |model: &Model| {
            let (min, max) = model.render_meshes()[0].bounds().unwrap();
            let origin = model.origin.z as f32;
            (min.z + origin, max.z + origin)
        };
        assert_eq!(z_range(&model), (0.0, 0.0));
        model.thick_shells = model.shell_sections().cloned().collect();
        assert_eq!(model.thick_shells.len(), 1);
        // The nodes lie on the top face, the solid below them.
        let (low, high) = z_range(&model);
        assert!(
            (low + 2.0).abs() < 1e-5 && high.abs() < 1e-5,
            "{low} {high}"
        );
        model.fe.properties.space = plx_model::ModelSpace::PlaneStress;
        assert_eq!(model.shell_sections().count(), 0);
    }

    /// Beams of a beam section are drawn with their profile, turned by its orientation, in 3D
    /// FE models only.
    #[test]
    fn beam_sections_are_drawn_with_their_profile() {
        let mut mesh = FeMesh::default();
        mesh.set_node(1, [0.0, 0.0, 0.0]);
        mesh.set_node(2, [10.0, 0.0, 0.0]);
        mesh.add_element(plx_mesh::Element {
            id: 1,
            type_name: "B31".into(),
            shape: plx_mesh::ElementShape::Line2,
            nodes: vec![1, 2],
        })
        .unwrap();
        mesh.parts.push(plx_mesh::Part {
            name: "Beam".into(),
            elements: vec![1],
        });
        let mut model = Model::new(Path::new("beam.inp"), mesh);
        model.fe.sections.push(plx_model::Section {
            name: "Beam_Section-1".into(),
            material: "Steel".into(),
            region: plx_model::Region::Parts(vec!["Beam".into()]),
            thickness: 1.0,
            kind: SectionKind::Beam(plx_model::BeamSection {
                cross_section: "Rect".into(),
                orientation: BeamOrientation::Direction([0.0, 1.0, 0.0]),
                offset: [0.0; 2],
            }),
        });
        let extent = |model: &Model| {
            let (min, max) = model.render_meshes()[0].bounds().unwrap();
            max - min
        };
        assert_eq!(extent(&model), Vec3::new(10.0, 0.0, 0.0));
        model.fe.cross_sections.push(plx_model::CrossSection {
            name: "Rect".into(),
            profile: plx_model::BeamProfile::Rect { a: 4.0, b: 2.0 },
        });
        model.thick_lines = (model.line_sections())
            .map(|(s, p)| (s.clone(), p.cloned()))
            .collect();
        assert_eq!(model.thick_lines.len(), 1);
        // 4 along the normal y, 2 along x cross y = z.
        let size = extent(&model);
        assert!((size - Vec3::new(10.0, 4.0, 2.0)).length() < 1e-5, "{size}");
        model.fe.properties.space = plx_model::ModelSpace::PlaneStress;
        assert_eq!(model.line_sections().count(), 0);
    }

    /// Results draw shells with the thickness, beams with the profile and trusses with the
    /// area of the input CalculiX ran, in the units they show, coloured by the results.
    #[test]
    fn results_draw_shells_beams_and_trusses_of_the_input_as_solids() {
        let dir = std::env::temp_dir().join(format!("plx-thick-results-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        // A plate 1 by 1 m, 0.02 m thick with its nodes on the top face, and beside it a held
        // truss of 1e-4 m^2 and a held beam 0.1 by 0.04 m, which shells keep a line in the
        // results (OUTPUT=2D).
        let input = "\
*Node
1, 0, 0, 0
2, 1, 0, 0
3, 1, 1, 0
4, 0, 1, 0
5, 2, 0, 0
6, 2, 0, -1
7, 3, 0, 0
8, 3, 0, 1
*Element, Type=S4, Elset=Plate
1, 1, 2, 3, 4
*Element, Type=T3D2, Elset=Bar
2, 5, 6
*Element, Type=B31, Elset=Post
3, 7, 8
*Material, Name=Steel
*Elastic
2.1e11, 0.3
*Shell section, Elset=Plate, Material=Steel, Offset=0.5
0.02
*Solid section, Elset=Bar, Material=Steel
1e-4
*Beam section, Elset=Post, Material=Steel, Section=RECT
0.1, 0.04
1, 0, 0
*Step
*Static
*Boundary
1, 1, 6
2, 1, 6
4, 1, 6
5, 1, 3
6, 1, 3
7, 1, 6
8, 1, 6
*Cload
3, 3, -1000
*Node file, Output=2D
U
*El file, Output=2D
S
*End step
";
        std::fs::write(dir.join("thick.inp"), input).unwrap();
        plxs_session::run_input_file(&dir, "thick").unwrap_or_else(|e| panic!("{e}"));
        let mut model = load(&dir.join("thick.frd"), UnitSystem::MmTonSC)
            .unwrap()
            .model;
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(model.shell_sections().count(), 1);
        assert_eq!(model.line_sections().count(), 2);
        // The sets of the input stay out of the results tree.
        assert!(model.mesh.element_sets.is_empty());
        if let Some(view) = model.results.as_mut() {
            view.deformation = crate::results::Deformation::Undeformed;
        }
        let z_range = |model: &Model| {
            let meshes = model.render_meshes();
            let (min, max) = (meshes.iter().filter_map(RenderMesh::bounds))
                .reduce(|(a, b), (c, d)| (a.min(c), b.max(d)))
                .unwrap();
            let origin = model.origin.z as f32;
            (min.z + origin, max.z + origin)
        };
        let thin = z_range(&model);
        model.thick_shells = model.shell_sections().cloned().collect();
        model.thick_lines = (model.line_sections())
            .map(|(s, p)| (s.clone(), p.cloned()))
            .collect();
        let thick = z_range(&model);
        // In millimetres: the bar reaches down to -1000 and the post up to 1000 either way.
        for range in [thin, thick] {
            assert!(
                (range.0 + 1000.0).abs() < 1e-2 && (range.1 - 1000.0).abs() < 1e-2,
                "{range:?}"
            );
        }
        let plate = model.parts.iter().position(|p| p.name == "Plate").unwrap();
        let mesh = &model.render_meshes()[plate];
        let lowest = (mesh.vertices.iter().map(|v| v.position[2])).fold(f32::MAX, f32::min);
        assert!(
            (lowest + model.origin.z as f32 + 20.0).abs() < 1e-3,
            "{lowest}"
        );
        // The solids take the contour values of their nodes.
        assert!(mesh.vertices.iter().all(|v| v.scalar >= 0.0));
        let bar = model.parts.iter().position(|p| p.name == "Bar").unwrap();
        let mesh = &model.render_meshes()[bar];
        let x = (mesh
            .vertices
            .iter()
            .map(|v| v.position[0] + model.origin.x as f32))
        .fold((f32::MAX, f32::MIN), |(lo, hi), x| (lo.min(x), hi.max(x)));
        let radius = (100.0 / std::f32::consts::PI).sqrt();
        assert!((x.1 - x.0 - 2.0 * radius).abs() < 1e-2, "{x:?}");
        let post = model.parts.iter().position(|p| p.name == "Post").unwrap();
        let (min, max) = model.render_meshes()[post].bounds().unwrap();
        let size = max - min;
        assert!(
            (size - Vec3::new(100.0, 40.0, 1000.0)).length() < 1e-2,
            "{size}"
        );
    }

    /// A flat disk in a 3D model is meshed as shells and thickened by a shell section; thin,
    /// clamped at its rim and loaded by its weight it sags as a Kirchhoff plate does.
    #[test]
    fn surfaces_in_3d_models_solve_as_shells() {
        if !gmsh_available() {
            eprintln!("Gmsh missing, test skipped");
            return;
        }
        use plx_model::{
            BoundaryCondition, BoundaryKind, Elastic, Load, LoadKind, Material, ModelSpace, Region,
            Section, Step,
        };
        let mut model = load(&testdata("membran.step"), UnitSystem::Unitless)
            .unwrap()
            .model;
        let mut geometry = model.geometry.clone().unwrap();
        geometry.meshing.max_size = 2.0;
        geometry.meshing.quad_dominated = true;
        let mut mesh = plx_mesher::generate_mesh(&geometry).unwrap().mesh;
        ModelSpace::ThreeD
            .prepare_generated_mesh(&mut mesh)
            .unwrap();
        assert!(mesh.elements().iter().all(|e| e.type_name == "S8"));
        model.set_mesh(mesh);
        let (radius, thickness, young, poisson) = (20.0, 0.25, 210000.0, 0.3);
        let (density, gravity) = (7.85e-9, 9810.0);
        let rim: Vec<NodeId> = (model.mesh.node_ids().iter().zip(model.mesh.coords()))
            .filter(|(_, c)| (c[0].hypot(c[1]) - radius).abs() < 1e-6)
            .map(|(id, _)| *id)
            .collect();
        assert!(rim.len() > 20);
        let part = model.mesh.parts[0].name.clone();
        let mut step = Step::new_static("Step-1");
        step.boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Clamped".into(),
            active: true,
            visible: true,
            region: Region::Nodes(rim),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        step.loads.push(Load {
            distribution: None,
            name: "Weight".into(),
            active: true,
            visible: true,
            region: Region::Parts(vec![part.clone()]),
            kind: LoadKind::Gravity([0.0, 0.0, -gravity]),
            amplitude: None,
            factor_amplitude: None,
        });
        let solid = Section {
            name: "Section-1".into(),
            material: "Steel".into(),
            region: Region::Parts(vec![part]),
            thickness: 1.0,
            kind: SectionKind::Solid,
        };
        model.fe = FeModel {
            materials: vec![Material {
                name: "Steel".into(),
                density: Some(density),
                elastic: Some(Elastic { young, poisson }),
                ..Default::default()
            }],
            sections: vec![solid.clone()],
            steps: vec![step],
            ..FeModel::default()
        };
        // A solid section would leave the shells without a thickness.
        assert!(plx_io::inp::write_inp(&model.mesh, &model.fe, "Membran").is_err());
        model.fe.sections[0].kind = SectionKind::Shell {
            thickness,
            offset: 0.0,
        };
        let inp = plx_io::inp::write_inp(&model.mesh, &model.fe, "Membran").unwrap();
        assert!(inp.contains("*Shell section"), "{inp}");
        let dir = std::env::temp_dir().join(format!("nodalis-shell-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("membran.inp"), inp).unwrap();
        plxs_session::run_input_file(&dir, "membran").unwrap_or_else(|e| panic!("{e}"));
        let results = read_frd(&dir.join("membran.frd")).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        let disp = results.increments.last().unwrap().field("DISP").unwrap();
        let sag = (disp.component("U3").unwrap().values.iter())
            .copied()
            .fold(0.0f32, f32::min);
        // Centre deflection of a clamped circular plate: q a^4 / (64 D).
        let q = density * gravity * thickness;
        let stiffness = young * thickness.powi(3) / (12.0 * (1.0 - poisson * poisson));
        let expected = -q * radius.powi(4) / (64.0 * stiffness);
        assert!(
            (f64::from(sag) / expected - 1.0).abs() < 0.02,
            "sag {sag}, plate theory {expected}"
        );
    }

    /// Parts sharing a section share a material, which is all an `.frd` file groups the
    /// elements by; the results take the parts of the input they were computed from.
    #[test]
    fn results_list_the_parts_of_their_input() {
        use plx_model::{
            BoundaryCondition, BoundaryKind, Elastic, Material, Region, Section, Step,
        };
        // Three cubes in a row, one part each, sharing the nodes where they touch.
        let mut mesh = FeMesh::default();
        let mut id = 0;
        for x in 0..=3 {
            for (y, z) in [(0, 0), (1, 0), (1, 1), (0, 1)] {
                id += 1;
                mesh.set_node(id, [x as f64, y as f64, z as f64]);
            }
        }
        let names = ["Solid-4", "SOLID-5", "SOLID-6"];
        for (index, name) in names.iter().enumerate() {
            let first = 4 * index as NodeId;
            let nodes = (1..=8).map(|n| first + n).collect();
            let id = index as ElementId + 1;
            (mesh.add_element(plx_mesh::Element {
                id,
                type_name: "C3D8".into(),
                shape: plx_mesh::ElementShape::Hex8,
                nodes,
            }))
            .unwrap();
            mesh.parts.push(plx_mesh::Part {
                name: name.to_string(),
                elements: vec![id],
            });
        }
        let mut step = Step::new_static("Step-1");
        for (name, nodes, kind) in [
            (
                "Fix",
                vec![1, 2, 3, 4],
                BoundaryKind::displacement([Some(0.0); 6]),
            ),
            (
                "Pull",
                vec![13, 14, 15, 16],
                BoundaryKind::displacement([Some(0.01), None, None, None, None, None]),
            ),
        ] {
            step.boundary_conditions.push(BoundaryCondition {
                distribution: None,
                name: name.into(),
                active: true,
                visible: true,
                region: Region::Nodes(nodes),
                kind,
                amplitude: None,
            });
        }
        let fe = FeModel {
            materials: vec![Material {
                name: "S235".into(),
                elastic: Some(Elastic {
                    young: 210000.0,
                    poisson: 0.3,
                }),
                ..Default::default()
            }],
            sections: vec![Section {
                name: "Section-1".into(),
                material: "S235".into(),
                region: Region::Parts(names.map(String::from).to_vec()),
                thickness: 1.0,
                kind: SectionKind::Solid,
            }],
            steps: vec![step],
            ..FeModel::default()
        };
        let inp = plx_io::inp::write_inp(&mesh, &fe, "Bloecke").unwrap();
        let dir = std::env::temp_dir().join(format!("nodalis-parts-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("bloecke.inp"), inp).unwrap();
        plxs_session::run_input_file(&dir, "bloecke").unwrap_or_else(|e| panic!("{e}"));
        let frd = dir.join("bloecke.frd");
        let alone = read_frd(&frd).unwrap().mesh;
        let parts = |mesh: &FeMesh| -> Vec<(String, Vec<ElementId>)> {
            (mesh.parts.iter())
                .map(|p| (p.name.clone(), p.elements.clone()))
                .collect()
        };
        assert_eq!(parts(&alone), [("S235".to_string(), vec![1, 2, 3])]);
        let results = load(&frd, UnitSystem::Unitless).unwrap().model;
        // Without the input the results keep the grouping of the file.
        std::fs::remove_file(dir.join("bloecke.inp")).unwrap();
        let without_input = load(&frd, UnitSystem::Unitless).unwrap().model;
        std::fs::remove_dir_all(&dir).unwrap();
        let expected: Vec<_> = (names.iter().zip(1..))
            .map(|(name, id)| (name.to_string(), vec![id]))
            .collect();
        assert_eq!(parts(&results.mesh), expected);
        assert_eq!(results.parts.len(), 3);
        assert_eq!(parts(&without_input.mesh), parts(&alone));
    }

    /// The solver makes a material of each `*Gap` section; the gaps are not parts of the results.
    #[test]
    fn results_leave_out_the_gaps_of_their_input() {
        let dir = std::env::temp_dir().join(format!("nodalis-gaps-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        for extension in ["inp", "frd"] {
            let name = format!("gap_stuetzen.{extension}");
            std::fs::copy(testdata(&name), dir.join(&name)).unwrap();
        }
        let frd = dir.join("gap_stuetzen.frd");
        let results = load(&frd, UnitSystem::Unitless).unwrap().model;
        // Without the input the gaps are one part.
        std::fs::remove_file(dir.join("gap_stuetzen.inp")).unwrap();
        let without_input = load(&frd, UnitSystem::Unitless).unwrap().model;
        std::fs::remove_dir_all(&dir).unwrap();
        let names =
            |mesh: &FeMesh| -> Vec<String> { mesh.parts.iter().map(|p| p.name.clone()).collect() };
        assert_eq!(names(&results.mesh), ["SOLID"]);
        assert_eq!(results.mesh.element_count(), 1);
        // The ground nodes of the gaps stay, so the nodal results keep their order.
        assert_eq!(results.mesh.node_count(), 12);
        assert_eq!(names(&without_input.mesh), ["STEEL", "GAP"]);
    }

    #[test]
    fn file_without_elements_is_rejected() {
        assert!(load(&testdata("platte_knoten.inp"), UnitSystem::Unitless).is_err());
    }

    #[test]
    fn bound_results_open_with_what_was_defined_on_them() {
        let mut model = load(&testdata("kragbalken_c3d8.frd"), UnitSystem::Unitless)
            .unwrap()
            .model;
        let settings = plx_results::ResultsSettings {
            field_outputs: vec![plx_results::FieldOutput {
                name: "SCALED".into(),
                kind: plx_results::FieldOutputKind::Equation {
                    equation: "=DISP.ALL * 1000".into(),
                    unit: "um".into(),
                },
            }],
            transformations: vec![plx_results::Transformation::symmetry(
                plx_results::transformation::SymmetryPlane::X,
            )],
            hot_spots: vec![plx_results::hot_spot::HotSpot::new("Hot_Spot-1")],
            reference_points: vec![plx_model::ReferencePoint::new("RP-1", [1.0, 2.0, 3.0])],
            result_paths: vec![plx_model::ResultPath::new("Path-1")],
            plain_part_names: vec![model.parts[0].name.clone()],
            mesh: Some(model.mesh.fingerprint()),
            increment: 0,
            shown: Some(("SCALED".into(), "VALUE".into())),
            view: plx_results::saved_view::SavedView {
                deformation: Some(plx_results::saved_view::SavedDeformation::TrueScale),
                user_scale: Some(2.5),
                mode_deformation: Some(plx_results::saved_view::SavedDeformation::Automatic(2.0)),
                mode_amplitude: Some(4.0),
                show_undeformed: Some(false),
                levels: Some(12),
                color_scale: Some(plx_model::ColorScale {
                    spectrum: plx_model::ColorSpectrum::CoolWarm,
                    levels: 12,
                    manual: Some([0.5, 3.0]),
                    ..Default::default()
                }),
                hidden_parts: vec![model.parts[0].name.clone()],
                annotations: vec![
                    plx_results::saved_view::SavedAnnotation::Node(99),
                    plx_results::saved_view::SavedAnnotation::Distance([1, 99]),
                ],
                path_plots: vec![
                    crate::path_plot::PathPlot::new("Path-1".into()).saved(),
                    crate::path_plot::PathPlot::hot_spot("Hot_Spot-1".into()).saved(),
                    crate::path_plot::PathPlot::table(crate::xy_plot::XyData {
                        title: "SP_A.STRESSES.MISES".into(),
                        x_label: "Increment".into(),
                        x: vec![1.0, 2.0],
                        curves: vec![("12_1".into(), vec![0.5, 1.5])],
                        guides: Vec::new(),
                        y_max: None,
                        ..Default::default()
                    })
                    .saved(),
                ],
                vector_display: Some(plx_results::display::VectorDisplay {
                    representation: plx_results::display::Representation::Arrows,
                    ..Default::default()
                }),
                // Camera and mesh edges belong to the 3D view, not to the results.
                camera: None,
                mesh_edges: None,
            },
            ..Default::default()
        };
        assert_eq!(
            model.restore_results_settings(&settings, Vec::new()),
            Vec::<String>::new()
        );
        assert!(!model.parts[0].contours);
        let view = model.results.as_ref().unwrap();
        let (field, component) = view.current().unwrap();
        assert_eq!(
            (field.name.as_str(), component.name.as_str()),
            ("SCALED", "VALUE")
        );
        assert_eq!(model.results_settings(), Some(settings.clone()));
        // Projects saved before part names were kept name parts by index, which only fit
        // results of the same mesh.
        let mut legacy = settings;
        legacy.plain_part_names.clear();
        legacy.plain_parts = vec![0];
        legacy.mesh = Some(model.mesh.fingerprint() ^ 1);
        model.set_contours(&[0], true);
        model.restore_results_settings(&legacy, Vec::new());
        assert!(model.parts[0].contours);
        legacy.mesh = Some(model.mesh.fingerprint());
        model.restore_results_settings(&legacy, Vec::new());
        assert!(!model.parts[0].contours);
    }
}
