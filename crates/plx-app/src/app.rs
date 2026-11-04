use std::collections::{BTreeSet, HashMap};
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, Sender, channel};

use plx_io::project::ProjectAnalysis;
use plx_model::UnitSystem;
use plx_render::StandardView;
use plx_results::hot_spot::{HotSpot, HotSpotPath};

use crate::analysis::{Analysis, MonitorEvent, RunView};
use crate::animation::{AnimationKind, ColorLimits, Playback};
use crate::animation_export::{self, Format};
use crate::contact_search::{ContactSearchDialog, SearchResult};
use crate::exploded::{ExplodedDialog, ExplodedResult};
use crate::features::{FeatureDialog, FeatureKind, FeatureResult};
use crate::field_output_dialog::{DialogAction, FieldOutputDialog};
use crate::history_output_dialog::HistoryOutputDialog;
use crate::history_table::HistoryTable;
use crate::hot_spot_dialog::HotSpotDialog;
use crate::icons::{self, Icon};
use crate::material_library::{LibraryResult, MaterialLibraryEditor};
use crate::mesh_tools::{MeshTool, MeshToolDialog, MeshToolResult};
use crate::meshing::{
    MeshItemEditor, MeshItemResult, MeshSetupResult, MeshSetupWindow, MeshingJob,
};
use crate::model::{self, Highlight, LoadedModel, Model};
use crate::model_properties::{DialogResult, ModelPropertiesDialog, geometry_check};
use crate::numeric;
use crate::overlay::AnnotationCommand;
use crate::overlay::{Marker, Overlay};
use crate::path_plot::{self, PathPlot, PlotEvent};
use crate::properties;
use crate::query::{QueryAction, QueryClick, QueryWindow};
use crate::results::{Deformation, ResultsView, format_legend_value};
use crate::screenshot::{self, Screenshot};
use crate::section::{PlaneDefinition, SectionDialog, SectionResult, SectionView};
use crate::selection::Operation;
use crate::settings::{self, Settings, SettingsWindow, WindowResult};
use crate::setup::{Editor, EditorResult, NewItem};
use crate::sound::{self, ModeSound};
use crate::symbols;
use crate::transformation_dialog::{TransformationAction, TransformationDialog};
use crate::tree::{self, TreeItem, TreeResponse, TreeState, TreeView};
use crate::viewport::{BoxSelect, Click, ViewCommand, Viewport};
use plx_render::RenderMesh;

mod commands;

enum LoadEvent {
    Started(PathBuf),
    Finished(PathBuf, Result<Box<LoadedModel>, String>),
    /// CAD files added to the geometry of the open model.
    Added(Vec<PathBuf>, Result<Box<plx_mesher::CadAddition>, String>),
    /// Meshes of input files, each read from its file, to add as parts of the open model.
    MeshesRead(Vec<(PathBuf, Result<Box<plx_io::inp::InpImport>, String>)>),
    /// A project file was picked or dropped; the project window opens it.
    Project(PathBuf),
}

/// What an analysis window asks of the project window, which holds the project.
#[derive(Clone, Debug, PartialEq)]
pub enum Request {
    /// Save the project, to a file the user picks with `true`.
    Save(bool),
    /// Save a copy of the project without the results of its analyses.
    SaveWithoutResults,
    /// Open this project file instead of the open project.
    OpenProject(PathBuf),
    /// Solve the analyses this one takes results or states from and that have none or old
    /// ones, then this one: Solve, or Solve Again with `true`, which runs the solver even if
    /// the results are up to date.
    SolveChain(bool),
    /// Stop the solve of this analysis.
    StopSolve,
    /// Open the analysis of this name, which a linked item comes from.
    OpenAnalysis(String),
    /// Stop taking the item at this path from another analysis; the analysis keeps its
    /// copy.
    Unlink(Vec<String>),
    /// Open the Campbell block of this analysis, made and linked first if it has none.
    Campbell,
    /// Close the analysis shown in the project window: its data goes back to the project.
    Close,
}

struct Workbench {
    settings: Settings,
    /// Open settings window with its unsaved draft.
    settings_window: Option<SettingsWindow>,
    /// Name of the analysis the window shows, also of its input and results files.
    analysis_name: String,
    /// Folder of the project in the work directory; each analysis of the project runs in a
    /// folder of its own in it.
    project_folder: String,
    /// What the window asks of the project window.
    requests: Vec<Request>,
    /// What the analysis takes from other analyses of the project; the window shows it but
    /// does not change it.
    incoming: crate::links::Incoming,
    /// What the analysis takes from other blocks that is not ready yet.
    upstream: crate::status::Upstream,
    /// A linked item was double-clicked: where it comes from, with its path.
    linked_prompt: Option<(Vec<String>, String)>,
    /// The state of the analysis' cells and the revision of the analysis it was found for;
    /// finding it takes the fingerprints of mesh and model, so it is found again only when
    /// the analysis changed.
    status: Option<(u64, crate::status::AnalysisStatus)>,
    /// Open dialog of the model properties.
    model_dialog: Option<ModelPropertiesDialog>,
    viewport: Viewport,
    /// The FE model workspace: mesh and analysis set up from an input or project file.
    model: Option<Model>,
    /// The CAD geometry of the FE model as shown on the Geometry tab.
    geometry: Option<Model>,
    /// Open window of the default meshing parameters.
    mesh_setup: Option<MeshSetupWindow>,
    /// Open dialog of a mesh setup item.
    mesh_item_editor: Option<MeshItemEditor>,
    meshing: Option<MeshingJob>,
    /// CAD faces and edges Gmsh named when meshing failed, shown red on the geometry.
    mesh_failure: BTreeSet<plx_mesher::CadEntity>,
    /// The results workspace: every results file opened in this session.
    /// One of them is shown on the Results tab.
    results: Vec<Model>,
    current_result: usize,
    /// Camera of the workspace not shown, so switching tabs keeps each view.
    parked_camera: Option<plx_render::Camera>,
    tree: TreeState,
    /// Item whose properties window is open.
    dialog: Option<TreeItem>,
    /// Name typed in the properties window of a part, with why it cannot be taken.
    part_name: (String, Option<String>),
    /// Element types of the part in the properties window, as the user picks them.
    part_types: Vec<String>,
    /// Part the open context menu of the 3D view belongs to.
    menu_part: Option<usize>,
    /// Which of the three trees is shown.
    tree_view: TreeView,
    /// The workspace the project window asked for while the results bound to the analysis are
    /// still to open; they must not switch to the Results tab then.
    pinned_view: Option<TreeView>,
    output: Vec<String>,
    /// The output pane shows the Analysis tab, with the output of the last solver run,
    /// instead of the messages.
    analysis_tab: bool,
    view_command: Option<ViewCommand>,
    /// The result selection or deformation changed; the scene must be rebuilt.
    results_changed: bool,
    /// The highlight of the shown scene changed, drawn differently on these parts: only
    /// their meshes are rebuilt, so that a click in a large assembly doesn't build all parts
    /// anew. Without any, only the highlighted nodes and lines over the scene changed.
    highlight_changed: Option<BTreeSet<usize>>,
    /// Only the animation frame changed; frames already built are reused.
    frame_changed: bool,
    /// Scene of each animation frame shown so far, cleared when anything else changes.
    frame_cache: std::collections::HashMap<usize, Vec<RenderMesh>>,
    /// Open dialog creating or editing an item of the FE model.
    editor: Option<Editor>,
    /// The step new boundary conditions, loads, outputs and defined fields go into, by name;
    /// `None` or a name no longer there means the last step.
    active_step: Option<String>,
    /// The selection the active step last followed, so that it follows only a new one.
    step_selection: Option<TreeItem>,
    /// The open editors of the properties panel, the newest last; only one stays open.
    docked: Vec<crate::dock::Docked>,
    /// Items to delete once the user agrees: several parts, or one item.
    confirm_delete: Option<(TreeView, Vec<TreeItem>)>,
    /// Remove Results waits for the user's confirmation.
    confirm_remove_results: bool,
    /// The toast shown over the 3D view, such as "Deleted Force-1" with Undo.
    toast: Option<crate::toast::Toast>,
    /// The output pane shows the Problems tab.
    problems_tab: bool,
    /// The properties panel shows the checklist while no item is edited.
    checklist_shown: bool,
    /// The checklist lying over the 3D view of a narrow window waits until it is asked for.
    checklist_fold: crate::layout::Fold,
    /// Whether the checklist was shown in the last frame, for its ribbon tile.
    checklist_visible: bool,
    /// Whether the Analysis tab was asked for in the last frame, to notice when a solve
    /// brings it to the front.
    analysis_tab_before: bool,
    /// `problems_tab` in the last frame: a folded output pane opens when it turns on.
    problems_tab_before: bool,
    /// Open search for contact pairs.
    contact_search: Option<ContactSearchDialog>,
    /// The table of all contact regions is open.
    contact_worksheet: bool,
    /// Open mesh tool: transform parts, merge nodes or renumber.
    mesh_tool: Option<MeshToolDialog>,
    /// The Query tool, while its window is open.
    query: Option<QueryWindow>,
    /// Open material library editor.
    material_library: Option<MaterialLibraryEditor>,
    /// Open dialog creating or editing a field output derived from the shown results.
    field_output_dialog: Option<FieldOutputDialog>,
    /// Open dialog creating or editing a history output of the shown results.
    history_dialog: Option<HistoryOutputDialog>,
    /// Open table of a history output component.
    history_table: Option<HistoryTable>,
    /// Diagram of a table selection.
    history_plot: Option<crate::xy_plot::XyData>,
    /// The tree selection whose region is highlighted.
    /// `None` until it is computed, so a dialog's highlight is cleared once it closes.
    highlighted: Option<TreeSelection>,
    /// The last Check Model run of the window.
    analysis: Option<Analysis>,
    /// The last solve of the analysis, which the project runs; shown in the Analysis tab.
    project_run: Option<RunView>,
    /// The project's solve just brought new results: they take over what was defined on
    /// the results before, once they are restored.
    carry_to_restored: Option<plx_results::ResultsSettings>,
    /// Results being bound to or restored from the project file of the model at this path.
    bound_work: Option<(PathBuf, std::sync::mpsc::Receiver<BoundWork>)>,
    /// The global results of a submodel being read, for the results at this path.
    global_work: Option<(PathBuf, std::sync::mpsc::Receiver<GlobalWork>)>,
    /// What was defined on the results before the analysis ran again, for its new results
    /// at this path.
    carried_settings: Option<(PathBuf, plx_results::ResultsSettings)>,
    /// Problems the solver reported when the last analysis failed, shown in the tree.
    solver_findings: Vec<plx_model::Finding>,
    /// The findings of a tree item whose warning sign was clicked, explained in a window.
    findings_window: Option<Vec<plx_model::Finding>>,
    /// Results file the user asked to open; read by the app on a worker thread.
    open_results: Option<PathBuf>,
    /// The results file asked for last; see [`is_latest_results`].
    results_asked: Option<PathBuf>,
    screenshot: Screenshot,
    /// Animation export taking its frames from the 3D view.
    recording: Option<animation_export::Recording>,
    /// Exported animation being encoded, with its file name.
    animation_encoding: Option<(String, animation_export::Encoder)>,
    /// Open dialog of a hot spot definition of the current results.
    hot_spot_dialog: Option<HotSpotDialog>,
    /// Hot spot whose paths are shown, edited or selected, with the results file and its
    /// paths there.
    hot_spot_preview: Option<(HotSpot, usize, Vec<HotSpotPath>)>,
    /// The toe node whose path the hot spot dialog plots, as last drawn.
    hot_spot_plotted: Option<plx_mesh::NodeId>,
    /// Audio output of the sound window, opened when it first plays.
    audio: Option<sound::Player>,
    /// The section view, while it is on; it cuts whatever the 3D view shows.
    section: Option<SectionView>,
    section_dialog: Option<SectionDialog>,
    /// Open dialog of the transformations of the current results.
    transformation_dialog: Option<TransformationDialog>,
    /// The cut shown in the scene of the given version with the visible parts, and whether
    /// it shows the section faces alone, to rebuild it on changes.
    section_shown: Option<(u64, Vec<bool>, SectionView, bool)>,
    /// The boundary conditions and loads, with the shown parts and the exploded view, whose
    /// symbols are drawn.
    symbols_shown: Option<(Vec<symbols::Item>, Vec<bool>, u64)>,
    /// Open exploded view dialog with the model it edits.
    exploded_dialog: Option<(ExplodedDialog, ShownModel)>,
    /// The exploded view last applied, where the next one starts.
    last_exploded: crate::exploded::Parameters,
    /// Open dialog of a feature or of results on one.
    feature_dialog: Option<FeatureDialog>,
    /// The Edit Annotation dialog of a path plot in the 3D view.
    plot_editor: Option<path_plot::EditDialog>,
    /// The Color Scale dialog of a results file, with the scale it last applied.
    color_scale_dialog: Option<(
        crate::color_scale_dialog::ColorScaleDialog,
        plx_model::ColorScale,
    )>,
    /// The Export VTU or Export VTK dialog of a results file.
    vtk_export_dialog: Option<crate::vtk_export_dialog::VtkExportDialog>,
    /// The shown values on the plane of the shown plane result, in model coordinates: where
    /// the plane cuts the parts as shown, and where it cuts the undeformed mesh.
    section_values: Option<plx_render::SectionValues>,
    plane_values: Option<plx_render::SectionValues>,
}

/// Which model the 3D view shows: the geometry, the FE model or one of the results.
#[derive(Clone, Copy, Debug, PartialEq)]
struct ShownModel(TreeView, usize);

/// The window of one analysis of the project: the main window with the geometry, the
/// FE model and the results of the analysis.
pub struct AnalysisWindow {
    workbench: Workbench,
    load_events: (Sender<LoadEvent>, Receiver<LoadEvent>),
    loading: Option<PathBuf>,
    /// Files to import once the analysis is loaded.
    import_when_loaded: Vec<PathBuf>,
    /// The workspace to switch to once the analysis is loaded.
    show_when_loaded: Option<Show>,
    /// The tab of each workspace and whether the ribbon is folded.
    ribbon: crate::ribbon::RibbonState,
    /// The command search, in a window of its own; the project window has its own.
    search: Option<crate::node_picker::NodePicker>,
}

impl AnalysisWindow {
    pub fn new(render_state: egui_wgpu::RenderState, settings: Settings, name: &str) -> Self {
        Self {
            workbench: Workbench {
                settings,
                settings_window: None,
                analysis_name: name.to_owned(),
                project_folder: String::new(),
                requests: Vec::new(),
                incoming: Default::default(),
                upstream: Default::default(),
                linked_prompt: None,
                status: None,
                model_dialog: None,
                viewport: Viewport::new(render_state),
                model: None,
                geometry: None,
                mesh_setup: None,
                mesh_item_editor: None,
                meshing: None,
                mesh_failure: BTreeSet::new(),
                results: Vec::new(),
                current_result: 0,
                parked_camera: None,
                tree: TreeState::default(),
                dialog: None,
                part_name: Default::default(),
                part_types: Vec::new(),
                menu_part: None,
                tree_view: TreeView::FeModel,
                pinned_view: None,
                output: Vec::new(),
                analysis_tab: false,
                view_command: None,
                results_changed: false,
                highlight_changed: None,
                frame_changed: false,
                frame_cache: Default::default(),
                editor: None,
                active_step: None,
                step_selection: None,
                docked: Vec::new(),
                confirm_delete: None,
                confirm_remove_results: false,
                toast: None,
                problems_tab: false,
                checklist_shown: true,
                checklist_fold: Default::default(),
                checklist_visible: false,
                analysis_tab_before: false,
                problems_tab_before: false,
                contact_search: None,
                contact_worksheet: false,
                mesh_tool: None,
                query: None,
                material_library: None,
                field_output_dialog: None,
                history_dialog: None,
                history_table: None,
                history_plot: None,
                highlighted: None,
                analysis: None,
                project_run: None,
                carry_to_restored: None,
                bound_work: None,
                global_work: None,
                carried_settings: None,
                solver_findings: Vec::new(),
                findings_window: None,
                open_results: None,
                results_asked: None,
                screenshot: Screenshot::default(),
                recording: None,
                animation_encoding: None,
                hot_spot_dialog: None,
                hot_spot_preview: None,
                hot_spot_plotted: None,
                audio: None,
                section: None,
                section_dialog: None,
                transformation_dialog: None,
                feature_dialog: None,
                plot_editor: None,
                color_scale_dialog: None,
                vtk_export_dialog: None,
                section_values: None,
                plane_values: None,
                section_shown: None,
                symbols_shown: None,
                exploded_dialog: None,
                last_exploded: Default::default(),
            },
            load_events: channel(),
            loading: None,
            import_when_loaded: Vec::new(),
            show_when_loaded: None,
            ribbon: Default::default(),
            search: None,
        }
    }

    /// Shows this workspace, as soon as the analysis is loaded.
    pub fn show_workspace(&mut self, show: Show) {
        self.show_when_loaded = Some(show);
    }

    /// Imports these files into the analysis as soon as it is loaded.
    pub fn import_after_load(&mut self, paths: Vec<PathBuf>) {
        self.import_when_loaded = paths;
    }

    /// Tells the window what the analysis takes from other blocks, and what of that is not
    /// ready yet.
    pub fn set_links(
        &mut self,
        incoming: crate::links::Incoming,
        upstream: crate::status::Upstream,
    ) {
        let workbench = &mut self.workbench;
        if workbench.upstream != upstream {
            // The cells follow at once, not when the cached state is found again.
            workbench.status = None;
        }
        workbench.incoming = incoming;
        workbench.upstream = upstream;
    }

    /// Takes over what `path` names of the setup of another analysis, see
    /// [`crate::links::apply_setup`].
    pub fn apply_setup(&mut self, source: &plx_model::FeModel, path: &[String]) {
        if let Some(model) = &mut self.workbench.model {
            crate::links::apply_setup(&mut model.fe, source, path);
            self.workbench.results_changed = true;
            self.workbench.symbols_shown = None;
        }
    }

    /// Takes over the items `names` of a category of another analysis, see
    /// [`crate::links::apply_bundle`]; returns the names taken.
    pub fn apply_bundle(
        &mut self,
        source: &plx_model::FeModel,
        link: &plx_io::project::Link,
        names: &[String],
        ahead: &[String],
    ) -> Vec<String> {
        let Some(model) = &mut self.workbench.model else {
            return link.inherited.clone();
        };
        let taken = crate::links::apply_bundle(&mut model.fe, source, link, names, ahead);
        self.workbench.results_changed = true;
        self.workbench.symbols_shown = None;
        taken
    }

    /// Makes the analysis continue `source` after its step `step`, see
    /// [`crate::links::apply_chain`]; returns the names of the steps taken.
    pub fn apply_chain(
        &mut self,
        source: &plx_model::FeModel,
        step: usize,
        link: &plx_io::project::Link,
    ) -> Vec<String> {
        let Some(model) = &mut self.workbench.model else {
            return link.inherited.clone();
        };
        let inherited = crate::links::apply_chain(&mut model.fe, source, step, link);
        self.workbench.results_changed = true;
        self.workbench.symbols_shown = None;
        inherited
    }

    /// Lets the analysis use the results of another one, see
    /// [`crate::links::apply_results`].
    pub fn apply_results(&mut self, link: &plx_io::project::Link, global: u32, file: PathBuf) {
        if let Some(model) = &mut self.workbench.model
            && crate::links::apply_results(&mut model.fe, link, global, file)
        {
            self.workbench.symbols_shown = None;
        }
    }

    /// Takes back what a removed results link set, see [`crate::links::clear_results`].
    pub fn clear_results(&mut self, link: &plx_io::project::Link, submodel: bool) {
        if let Some(model) = &mut self.workbench.model {
            crate::links::clear_results(&mut model.fe, link, submodel);
            self.workbench.symbols_shown = None;
        }
    }

    /// Takes over the geometry of another analysis, shown on the Geometry tab.
    pub fn apply_geometry(&mut self, geometry: Option<plx_model::Geometry>) {
        self.take_geometry(geometry, true);
    }

    /// [`Self::apply_geometry`]; `follow` has the selections and the mesh follow changed
    /// shapes, which a mesh taken with them does not need.
    fn take_geometry(&mut self, geometry: Option<plx_model::Geometry>, follow: bool) {
        let workbench = &mut self.workbench;
        let Some(model) = &mut workbench.model else {
            return;
        };
        // The analysis keeps its own mesh setup.
        let mut own = model.geometry.take();
        if let Some(tags) = crate::links::take_geometry(&mut own, geometry)
            && follow
        {
            let lost = crate::links::follow_geometry(&mut model.fe, Some(&mut model.mesh), &tags);
            if let Some(lost) = lost {
                (workbench.output).push(format!("{lost}."));
            }
            workbench.symbols_shown = None;
        }
        let geometry = own;
        workbench.geometry = match &geometry {
            Some(geometry) => match plx_mesher::tessellate(geometry) {
                Ok(display) => Some(Model::geometry_view(&model.path, display)),
                Err(error) => {
                    (workbench.output).push(format!("Geometry cannot be displayed: {error}"));
                    None
                }
            },
            None => None,
        };
        model.geometry = geometry;
        workbench.after_parts_changed();
    }

    /// Takes over the mesh of another analysis with the geometry it was meshed from.
    pub fn apply_mesh(&mut self, mesh: crate::links::MeshCopy) {
        let crate::links::MeshCopy {
            geometry,
            mesh,
            mesh_inputs,
            mesh_sets,
            element_types,
        } = mesh;
        let Some(model) = &mut self.workbench.model else {
            return;
        };
        // The new mesh is of the new shapes already: only the selections follow them.
        if let Some(tags) = crate::links::geometry_tags(model.geometry.as_ref(), geometry.as_ref())
        {
            crate::links::follow_geometry(&mut model.fe, None, &tags);
        }
        model.fe.mesh_sets = mesh_sets;
        model.fe.element_types = element_types;
        model.set_mesh(mesh);
        model.mesh_inputs = mesh_inputs;
        self.take_geometry(geometry, false);
        self.workbench.view_command = Some(ViewCommand::Fit);
    }

    /// Shows the project's solve of the analysis in the Analysis tab, as far as it got.
    pub fn show_run(&mut self, solve: &crate::solution_monitor::BackgroundSolve) {
        let workbench = &mut self.workbench;
        let run = match &mut workbench.project_run {
            Some(run) if run.started == Some(solve.started()) => run,
            _ => {
                // A new solve: the output below the 3D view shows it, not an older Check
                // Model.
                if !(workbench.analysis.as_ref()).is_some_and(Analysis::is_running) {
                    workbench.analysis = None;
                }
                if solve.active() {
                    workbench.analysis_tab = true;
                }
                workbench.status = None;
                workbench.project_run.insert(RunView {
                    started: Some(solve.started()),
                    ..Default::default()
                })
            }
        };
        if run.active != solve.active() {
            workbench.status = None;
        }
        run.active = solve.active();
        run.state = solve.state_text();
        run.seconds = solve.seconds();
        run.progress = solve.progress();
        run.copy(&solve.output(), solve.output_total());
    }

    /// The project's solve is done: the analysis takes the mesh it made and the results it
    /// kept, which open on the Results tab, and what the solver reported.
    pub fn take_solved(&mut self, solved: crate::solution_monitor::Solved) {
        let workbench = &mut self.workbench;
        workbench.solver_findings = solved.findings;
        let Some(model) = &mut workbench.model else {
            return;
        };
        if let Some((mesh, mesh_inputs)) = solved.meshed {
            model.set_mesh(mesh);
            model.mesh_inputs = mesh_inputs;
            model.fe.update_reference_points(&model.mesh);
            workbench.after_parts_changed();
            workbench.view_command = Some(ViewCommand::Fit);
        }
        let Some(results) = solved.results else {
            workbench.status = None;
            return;
        };
        // What was defined on the results before goes over to the new ones.
        let path = workbench.model.as_ref().map(|m| m.path.clone());
        let previous = path.and_then(|p| workbench.previous_settings(&p));
        let Some(model) = &mut workbench.model else {
            return;
        };
        model.bound_results = Some(results);
        model.bound_results_file = None;
        workbench.carry_to_restored = previous;
        workbench.status = None;
        workbench.open_bound_results();
    }

    /// Opens the analysis of the project at `path` in the window, built on a worker thread.
    pub fn load(&mut self, ctx: &egui::Context, path: PathBuf, analysis: ProjectAnalysis) {
        let (sender, ctx) = (self.load_events.0.clone(), ctx.clone());
        self.loading = Some(path.clone());
        std::thread::spawn(move || {
            let loaded = model::load_analysis(&path, analysis);
            let _ = sender.send(LoadEvent::Finished(path, Ok(Box::new(loaded))));
            ctx.request_repaint();
        });
    }

    pub fn rename(&mut self, name: &str) {
        self.workbench.analysis_name = name.to_owned();
    }

    /// The folder of the project in the work directory, from the project file's name.
    pub fn set_project_folder(&mut self, folder: String) {
        self.workbench.project_folder = folder;
    }

    pub fn settings(&self) -> &Settings {
        &self.workbench.settings
    }

    pub fn set_settings(&mut self, settings: &Settings) {
        self.workbench.settings = settings.clone();
    }

    /// What the window asked of the project window since the last call.
    pub fn take_requests(&mut self) -> Vec<Request> {
        std::mem::take(&mut self.workbench.requests)
    }

    /// The open model of the analysis; `None` while it is being built.
    pub fn model(&self) -> Option<&Model> {
        self.workbench.model.as_ref()
    }

    /// The FE model of the analysis, for the project window to set its exposed parameters;
    /// a change shows like one the user made.
    pub fn fe_mut(&mut self) -> Option<&mut plx_model::FeModel> {
        let workbench = &mut self.workbench;
        let model = workbench.model.as_mut()?;
        workbench.results_changed = true;
        workbench.symbols_shown = None;
        Some(&mut model.fe)
    }

    /// Whether a Check Model run of the window runs.
    pub fn is_running(&self) -> bool {
        (self.workbench.analysis.as_ref()).is_some_and(Analysis::is_running)
    }

    /// What keeps the window from closing or its data from being taken: work on a worker
    /// thread that would be lost.
    pub fn busy(&self) -> Option<&'static str> {
        let workbench = &self.workbench;
        if self.loading.is_some() {
            Some("it is loading")
        } else if workbench.meshing.is_some() {
            Some("it is meshing")
        } else if self.is_running() {
            Some("its model check is running")
        } else if workbench.bound_work.is_some() {
            Some("it is keeping its result")
        } else {
            None
        }
    }

    /// The data of the analysis as the project saves it; the results keep what was defined
    /// on them, if they are open. `None` while the model is being built.
    pub fn data(&mut self) -> Option<ProjectAnalysis> {
        let workbench = &mut self.workbench;
        let bound_file = workbench.model.as_ref()?.bound_results_file.clone();
        let settings = (workbench.results.iter())
            .find(|r| bound_file.as_ref() == Some(&r.path))
            .map(|open| workbench.view_settings(open));
        let model = workbench.model.as_mut()?;
        if let Some(bound) = &mut model.bound_results
            && let Some(settings) = settings
        {
            bound.settings = settings;
        }
        Some(ProjectAnalysis {
            name: workbench.analysis_name.clone(),
            geometry: model.geometry.clone(),
            mesh: model.mesh.clone(),
            model: model.fe.clone(),
            mesh_inputs: model.mesh_inputs.clone(),
            results: model.bound_results.clone(),
            ..Default::default()
        })
    }

    /// The project was saved to `path`: the analysis belongs to that file now.
    pub fn saved(&mut self, path: &std::path::Path) {
        let workbench = &mut self.workbench;
        if let Some(model) = &mut workbench.model {
            // Results being kept for the model follow it to the new file.
            if let Some((kept_for, _)) = &mut workbench.bound_work
                && *kept_for == model.path
            {
                *kept_for = path.to_path_buf();
            }
            model.path = path.to_path_buf();
        }
    }

    /// Adds a line to the output of the window.
    pub fn message(&mut self, line: String) {
        self.workbench.output.push(line);
    }

    /// The state of the analysis' cells, found again when the analysis changed.
    pub fn status(&mut self) -> Option<crate::status::AnalysisStatus> {
        self.workbench.cached_status()
    }

    /// Changes whenever the data of the analysis changes as the project sees it; `None`
    /// while it loads.
    pub fn revision(&self) -> Option<u64> {
        self.workbench.revision()
    }

    /// The state of the cells found now, not as last found: right after a run, the one
    /// found while it ran may still be cached.
    pub fn current_status(&self) -> Option<crate::status::AnalysisStatus> {
        self.workbench.status()
    }

    /// Geometry > Import, into the model space and units the analysis was made with.
    fn import(&mut self, ctx: &egui::Context) {
        if self.workbench.model.is_some() {
            self.import_dialog(ctx);
        }
    }

    /// Geometry > Import: one or several STEP, IGES or BREP files, read into
    /// one geometry, or input files (.inp) whose mesh becomes mesh parts, such as a mesh
    /// written by "Export Deformed Mesh". A model that already has geometry or a mesh keeps
    /// it; the new parts are added.
    fn import_dialog(&mut self, ctx: &egui::Context) {
        if self.loading.is_some() {
            return;
        }
        let picked = rfd::FileDialog::new()
            .set_title("Import Geometry")
            .add_filter(IMPORT_FILTER.0, IMPORT_FILTER.1)
            .add_filter(GEOMETRY_FILTER.0, GEOMETRY_FILTER.1)
            .add_filter("Input file (*.inp)", &["inp", "INP"])
            .pick_files();
        if let Some(paths) = picked.filter(|p| !p.is_empty()) {
            self.import_paths(paths, ctx);
        }
    }

    /// Imports CAD files into the geometry and the meshes of input files as mesh parts, as
    /// Geometry > Import does; files dropped into the window come here too.
    pub fn import_paths(&mut self, paths: Vec<PathBuf>, ctx: &egui::Context) {
        if self.loading.is_some() {
            return;
        }
        let sender = self.load_events.0.clone();
        let ctx = ctx.clone();
        let units = self.workbench.import_units();
        // What the new parts are added to: the geometry, if any, and the names of the
        // mesh parts they must not take.
        let base = (self.workbench.model.as_ref())
            .filter(|m| m.geometry.is_some() || m.mesh.element_count() > 0)
            .map(|m| {
                let taken: Vec<String> = m.parts.iter().map(|p| p.name.clone()).collect();
                (m.geometry.clone(), taken, m.fe.properties.units)
            });
        std::thread::spawn(move || {
            let (meshes, paths): (Vec<PathBuf>, Vec<PathBuf>) =
                paths.into_iter().partition(|p| !plx_mesher::is_cad_file(p));
            if paths.is_empty() {
                read_meshes_in_background(meshes, sender, ctx);
                return;
            }
            match base {
                Some((geometry, taken, units)) => {
                    let _ = sender.send(LoadEvent::Started(paths[0].clone()));
                    ctx.request_repaint();
                    let result =
                        plx_mesher::add_cad_files(geometry.as_ref(), &paths, units, &taken)
                            .map(Box::new)
                            .map_err(|e| e.to_string());
                    let _ = sender.send(LoadEvent::Added(paths, result));
                    ctx.request_repaint();
                }
                None => load_geometry_in_background(paths, units, sender.clone(), ctx.clone()),
            }
            if !meshes.is_empty() {
                read_meshes_in_background(meshes, sender, ctx);
            }
        });
    }

    /// Opens the results of the analysis, written by the solver or restored from the project.
    fn open_results(&mut self, path: PathBuf, ctx: &egui::Context) {
        let (sender, ctx) = (self.load_events.0.clone(), ctx.clone());
        let units = self.workbench.import_units();
        std::thread::spawn(move || load_in_background(path, units, sender, ctx));
    }

    fn handle_load_events(&mut self) {
        while let Ok(event) = self.load_events.1.try_recv() {
            match event {
                LoadEvent::Started(path) => self.loading = Some(path),
                LoadEvent::Finished(path, result) => {
                    self.loading = None;
                    self.workbench.model_loaded(path, result);
                }
                LoadEvent::Added(paths, result) => {
                    self.loading = None;
                    self.workbench.geometry_added(&paths, result);
                }
                LoadEvent::MeshesRead(files) => {
                    self.loading = None;
                    self.workbench.meshes_added(files);
                }
                LoadEvent::Project(path) => {
                    self.loading = None;
                    self.workbench.requests.push(Request::OpenProject(path));
                }
            }
        }
    }

    /// The menus of the analysis in the app bar: File, Edit and Help; the ribbon holds the
    /// rest. In the project window, Close Analysis gives the analysis back to the project
    /// instead of closing a window.
    pub fn menus(&mut self, ui: &mut egui::Ui, embedded: bool) {
        ui.menu_button("File", |ui| {
            let save = egui::Button::new("Save Project").shortcut_text("Ctrl+S");
            if ui.add(save).clicked() {
                self.workbench.requests.push(Request::Save(false));
            }
            let save_as = egui::Button::new("Save Project As …").shortcut_text("Ctrl+Shift+S");
            if ui.add(save_as).clicked() {
                self.workbench.requests.push(Request::Save(true));
            }
            let without = egui::Button::new("Save Project Without Results …");
            if ui
                .add(without)
                .on_hover_text(
                    "Saves a copy of the project with only the meshes and the model \
                     definitions, without the results its analyses keep. The open \
                     project keeps them.",
                )
                .clicked()
            {
                self.workbench.requests.push(Request::SaveWithoutResults);
            }
            ui.separator();
            ui.menu_button("Export", |ui| self.workbench.export_menu(ui));
            ui.separator();
            let tip = "Closes the analysis; it stays in the project.";
            if ui.button("Close Analysis").on_hover_text(tip).clicked() {
                if embedded {
                    self.workbench.requests.push(Request::Close);
                } else {
                    ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
                }
            }
        });
        ui.menu_button("Edit", |ui| {
            self.workbench.edit_menu(ui);
            ui.separator();
            if ui.button("Settings …").clicked() {
                self.workbench.settings_window =
                    Some(SettingsWindow::new(&self.workbench.settings));
            }
        });
        ui.menu_button("Help", |ui| {
            let tip = "Opens the user guide in the web browser";
            if ui.button("User Guide").on_hover_text(tip).clicked() {
                let line = crate::help::open_user_guide();
                self.workbench.output.push(line);
            }
        });
    }

    /// The status bar; returns whether the meshing was cancelled there.
    fn status_bar(&self, ui: &mut egui::Ui) -> bool {
        let mut cancel = false;
        ui.horizontal(|ui| {
            // Nodalis shows the unit system of the model at the right.
            if let Some(model) = &self.workbench.model {
                let properties = &model.fe.properties;
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(format!(
                        "Unit system: {}   Model space: {}",
                        properties.units.label(),
                        properties.space.label()
                    ));
                    ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                        cancel = self.status_text(ui);
                    });
                });
            } else {
                cancel = self.status_text(ui);
            }
        });
        cancel
    }

    /// What the program does or shows; returns whether the meshing was cancelled.
    fn status_text(&self, ui: &mut egui::Ui) -> bool {
        let mut cancel = false;
        ui.horizontal(|ui| match (&self.loading, self.workbench.shown()) {
            (Some(path), _) => {
                ui.spinner();
                ui.label(format!("Loading {} …", path.display()));
            }
            (None, _) if self.workbench.recording.is_some() => {
                if let Some(recording) = &self.workbench.recording {
                    let (done, all) = recording.progress();
                    ui.spinner();
                    ui.label(format!(
                        "Recording {}: frame {} of {all} (Esc cancels)",
                        recording.file,
                        (done + 1).min(all)
                    ));
                }
            }
            (None, _) if self.workbench.animation_encoding.is_some() => {
                if let Some((file, _)) = &self.workbench.animation_encoding {
                    ui.spinner();
                    ui.label(format!("Writing {file} …"));
                }
            }
            (None, _) if self.workbench.meshing.is_some() => {
                ui.spinner();
                ui.label("Meshing …");
                cancel = (ui.small_button("Cancel"))
                    .on_hover_text("Stop the meshing; the parts keep their old mesh.")
                    .clicked();
            }
            (None, Some(model)) if model.is_geometry() => {
                ui.label(format!(
                    "{}: {} parts",
                    model.file_name(),
                    model.parts.len()
                ));
            }
            (None, Some(model)) => {
                ui.label(format!(
                    "{}: {} nodes, {} elements, {} parts",
                    model.file_name(),
                    model.mesh.node_count(),
                    model.mesh.element_count(),
                    model.parts.len()
                ));
            }
            (None, None) => {
                ui.weak(if self.workbench.tree_view == TreeView::Results {
                    "No results loaded"
                } else {
                    "No model loaded"
                });
            }
        });
        cancel
    }
}

/// Writes the results file `source` as CalculiX's files `frd` and `.dat` beside it: results
/// of the built-in solver (`.ndlr`) are converted, CalculiX's own files copied.
pub fn export_calculix_results(
    source: &std::path::Path,
    frd: &std::path::Path,
) -> Result<(), String> {
    if plx_results::solver::is_solver_results(source) {
        let solved = plx_results::SolverResults::read(source).map_err(|e| e.to_string())?;
        return plx_io::solver::export_calculix(&solved, frd)
            .map_err(|e| format!("{}: {e}", frd.display()));
    }
    if source == frd {
        return Ok(());
    }
    std::fs::copy(source, frd).map_err(|e| format!("{}: {e}", frd.display()))?;
    let (dat, target) = (source.with_extension("dat"), frd.with_extension("dat"));
    if dat.is_file() {
        std::fs::copy(&dat, &target).map_err(|e| format!("{}: {e}", target.display()))?;
    }
    Ok(())
}

/// Whether a file is a results file: CalculiX's (`.frd`) or the built-in solver's (`.ndlr`).
pub fn is_results_file(path: &std::path::Path) -> bool {
    path.extension()
        .is_some_and(|e| e.eq_ignore_ascii_case("frd"))
        || plx_results::solver::is_solver_results(path)
}

/// Moves the parts of a model between two scales of an exploded view: through the levels of a
/// sequential disassembly one after the other, else in one animation.
fn animate_explosion(
    model: &mut Model,
    parameters: &crate::exploded::Parameters,
    from: f64,
    to: f64,
) {
    let sequential =
        parameters.method == crate::exploded::Method::Disassembly && parameters.sequential;
    let stops = if sequential {
        model.explosion_layout(parameters).sequence(from, to)
    } else {
        vec![to]
    };
    let duration = if stops.len() > 1 {
        crate::exploded::STEP_TIME
    } else {
        crate::exploded::ANIMATION_TIME
    };
    let targets = stops
        .into_iter()
        .map(|scale| {
            let offsets = model
                .explosion_layout(parameters)
                .offsets(scale, parameters.sequential);
            (offsets, duration)
        })
        .collect();
    model.explosion.show_sequence(targets, true);
}

/// Shows the exploded view being edited in a dialog.
fn preview_explosion(model: &mut Model, dialog: &mut ExplodedDialog, animate: bool) {
    let parameters = dialog.draft.clone();
    dialog.step_count = model.explosion_layout(&parameters).step_count();
    let offsets = model.explosion_offsets(&parameters);
    model.explosion.show(offsets, animate);
}

/// Loads a file on this thread; CAD geometry in the length unit of `units`.
fn load_in_background(
    path: PathBuf,
    units: UnitSystem,
    sender: Sender<LoadEvent>,
    ctx: egui::Context,
) {
    if model::is_project_file(&path) {
        let _ = sender.send(LoadEvent::Project(path));
        ctx.request_repaint();
        return;
    }
    let _ = sender.send(LoadEvent::Started(path.clone()));
    ctx.request_repaint();
    let result = model::load(&path, units).map(Box::new);
    let _ = sender.send(LoadEvent::Finished(path, result));
    ctx.request_repaint();
}

/// Imports several CAD files into one geometry on the calling thread, reporting like
/// [`load_in_background`] under the first file's path.
fn load_geometry_in_background(
    paths: Vec<PathBuf>,
    units: UnitSystem,
    sender: Sender<LoadEvent>,
    ctx: egui::Context,
) {
    let path = paths[0].clone();
    let _ = sender.send(LoadEvent::Started(path.clone()));
    ctx.request_repaint();
    let result = model::load_geometry(&paths, units).map(Box::new);
    let _ = sender.send(LoadEvent::Finished(path, result));
    ctx.request_repaint();
}

/// Reads the meshes of input files on the calling thread, to be added to the open model.
fn read_meshes_in_background(paths: Vec<PathBuf>, sender: Sender<LoadEvent>, ctx: egui::Context) {
    let _ = sender.send(LoadEvent::Started(paths[0].clone()));
    ctx.request_repaint();
    let files = (paths.into_iter())
        .map(|path| {
            let result = plx_io::inp::read_inp(&path)
                .map(Box::new)
                .map_err(|e| e.to_string());
            (path, result)
        })
        .collect();
    let _ = sender.send(LoadEvent::MeshesRead(files));
    ctx.request_repaint();
}

/// The selected tree item and the parts and items selected along with it.
type TreeSelection = (Option<(TreeView, TreeItem)>, BTreeSet<usize>, Vec<TreeItem>);

/// The output line after deleting parts.
fn deleted_message(names: &[String]) -> String {
    match names {
        [name] => format!("Part {name} deleted"),
        names => format!("Parts {} deleted", names.join(", ")),
    }
}
/// Puts the meshes Gmsh made of parts of `geometry` into the model, as Mesh does; returns the
/// lines that tell what was made, or why the meshes could not be used.
pub(crate) fn merge_part_meshes(
    model: &mut Model,
    geometry: &plx_model::Geometry,
    generated: crate::meshing::PartMeshes,
    started: Option<std::time::Instant>,
) -> Result<Vec<String>, String> {
    let mut output = Vec::new();
    let had_mesh = model.mesh.element_count() > 0;
    let mut mesh = model.mesh.clone();
    let space = model.fe.properties.space;
    for mut part in generated.meshes {
        space
            .prepare_generated_mesh(&mut part.mesh)
            .map_err(|error| format!("Meshing failed: {error}"))?;
        // What each part is meshed from, to tell when its mesh is out of date.
        for p in &part.mesh.parts {
            let input = plx_model::fingerprint::mesh_input(geometry, space, &p.name);
            model.mesh_inputs.insert(p.name.clone(), input);
        }
        for warning in &part.warnings {
            output.push(format!("Gmsh: {warning}"));
        }
        let units = model.fe.properties.units;
        let size = |size: f64| {
            let size = units.shown(size, plx_model::Quantity::Length);
            crate::numeric::format_physical(size)
        };
        for p in &part.mesh.parts {
            let parameters = model.geometry.as_ref().map(|g| g.parameters(&p.name));
            output.push(format!(
                "{}: {} elements (order {}, element size {} to {} {})",
                p.name,
                p.elements.len(),
                if parameters.is_some_and(|s| s.second_order) {
                    2
                } else {
                    1
                },
                size(parameters.map_or(0.0, |s| s.min_size)),
                size(parameters.map_or(0.0, |s| s.max_size)),
                units.unit(plx_model::Quantity::Length),
            ));
        }
        mesh = plx_mesher::merge_part(&mesh, part.mesh);
    }
    model.set_mesh(mesh);
    let lost = lost_selections(&model.fe, &model.mesh);
    if had_mesh && lost > 0 {
        output.push(format!(
            "Note: {lost} selections made of node or element numbers refer to the old mesh \
             and must be selected again"
        ));
    }
    output.push(format!(
        "Mesh created: {} nodes, {} elements, {} parts ({} ms)",
        model.mesh.node_count(),
        model.mesh.element_count(),
        model.parts.len(),
        started.map_or(0, |s| s.elapsed().as_millis())
    ));
    Ok(output)
}

/// How many regions of the FE model consist of node or element numbers that a new mesh no
/// longer has. Regions picked on the geometry are found on the new mesh again.
fn lost_selections(fe: &plx_model::FeModel, mesh: &plx_mesh::FeMesh) -> usize {
    fe.regions()
        .filter(|r| r.by_mesh_ids() && r.missing_reference(mesh).is_some())
        .count()
}

/// File dialog filter of everything Import Geometry takes.
const IMPORT_FILTER: (&str, &[&str]) = (
    "Geometry or mesh (*.step, *.stp, *.iges, *.igs, *.brep, *.inp)",
    &[
        "step", "STEP", "stp", "STP", "iges", "IGES", "igs", "IGS", "brep", "BREP", "inp", "INP",
    ],
);

/// The height of the output pane folded to its tabs.
pub const OUTPUT_TABS: f32 = 26.0;

/// File dialog filter of the CAD formats Gmsh imports.
const GEOMETRY_FILTER: (&str, &[&str]) = (
    "Geometry (*.step, *.stp, *.iges, *.igs, *.brep)",
    &[
        "step", "STEP", "stp", "STP", "iges", "IGES", "igs", "IGS", "brep", "BREP",
    ],
);

impl AnalysisWindow {
    /// Shows the analysis in a window of its own: its app bar without the Project segment,
    /// its ribbon and its workspace.
    pub fn ui(&mut self, ui: &mut egui::Ui) {
        let bar = crate::ribbon::Bar {
            project: false,
            workspace: self.workspace(),
            analyses: Vec::new(),
            active: None,
            active_name: self.workbench.analysis_name.clone(),
            title: self.workbench.analysis_name.clone(),
            pill: self.pill(),
            can_undo: self.can_undo(),
            can_redo: self.can_redo(),
        };
        let events = egui::Panel::top("app bar")
            .exact_size(crate::ribbon::APP_BAR)
            .show(ui, |ui| {
                crate::ribbon::app_bar(ui, &bar, &mut |ui| self.menus(ui, false))
            })
            .inner;
        let ctx = ui.ctx().clone();
        for event in events {
            use crate::ribbon::BarEvent;
            match event {
                BarEvent::Workspace(workspace) => self.set_workspace(workspace),
                BarEvent::Analysis(_) => {}
                BarEvent::Save => self.workbench.requests.push(Request::Save(false)),
                BarEvent::Undo => self.undo(),
                BarEvent::Redo => self.redo(),
                BarEvent::Search(at) => {
                    self.search = Some(
                        crate::node_picker::NodePicker::new(at, at)
                            .with_limit(crate::command::SEARCH_ROWS, crate::command::SEARCH_HINT),
                    );
                }
                BarEvent::Settings => self.open_settings(),
                BarEvent::Help => self.open_user_guide(),
                BarEvent::Pill => self.pill_clicked(),
            }
        }
        let workspace = self.workspace();
        if let Some(search) = &mut self.search {
            let entries = crate::command::search_order(false, workspace);
            let list: Vec<crate::node_picker::Entry> = (entries.iter())
                .map(|e| crate::node_picker::Entry { ..e.entry })
                .collect();
            match search.show(&ctx, &list) {
                crate::node_picker::Outcome::Open => {}
                crate::node_picker::Outcome::Closed => self.search = None,
                crate::node_picker::Outcome::Picked(i) => {
                    self.search = None;
                    let entry = &entries[i];
                    self.run_found(&ctx, entry.workspace, entry.cmd);
                }
            }
        }
        self.workspace_ui(ui);
    }

    /// The properties panel: the editor that is open, or else the checklist.
    fn properties_panel(&mut self, ui: &mut egui::Ui, workspace: crate::command::Workspace) {
        let ctx = ui.ctx().clone();
        let overlay = crate::layout::current(&ctx).overlay;
        let folded = self.workbench.checklist_fold.folded(overlay);
        self.workbench.checklist_visible = self.workbench.checklist_shown && !folded;
        if self.workbench.one_docked() {
            crate::dock::panel(ui, None);
        } else if self.workbench.checklist_visible {
            let steps = self.checklist();
            let problems = self.workbench.problems().len();
            let mut clicked = None;
            let mut draw = |ui: &mut egui::Ui| {
                clicked = crate::checklist::show(ui, &steps, problems);
            };
            if crate::dock::panel(ui, Some(("Checklist", &mut draw))) {
                self.workbench.checklist_shown = false;
            }
            match clicked {
                Some(crate::checklist::Clicked::Run(cmd)) => {
                    self.ribbon.reveal(workspace, cmd);
                    self.run_command(&ctx, cmd);
                }
                Some(crate::checklist::Clicked::Problems) => self.workbench.problems_tab = true,
                None => {}
            }
        }
    }

    /// Shows the analysis inside the project window, below the project's app bar.
    pub fn ui_embedded(&mut self, ui: &mut egui::Ui) {
        self.workspace_ui(ui);
    }

    /// Runs a command the command search found: in its workspace, on its ribbon tab.
    pub fn run_found(
        &mut self,
        ctx: &egui::Context,
        workspace: crate::command::Workspace,
        cmd: crate::command::Cmd,
    ) {
        if workspace.view().is_some()
            && workspace != self.workspace()
            && !matches!(cmd, crate::command::Cmd::View(_))
        {
            self.set_workspace(workspace);
        }
        self.ribbon.reveal(self.workspace(), cmd);
        self.run_command(ctx, cmd);
    }

    /// The ribbon and the workspace: tree, 3D view, output, status bar and dialogs.
    fn workspace_ui(&mut self, ui: &mut egui::Ui) {
        self.handle_load_events();
        let ctx = ui.ctx().clone();
        if self.loading.is_none() && self.workbench.model.is_some() {
            let import = std::mem::take(&mut self.import_when_loaded);
            if !import.is_empty() {
                self.import_paths(import, &ctx);
            }
            if let Some(Show { view, step }) = self.show_when_loaded.take() {
                self.workbench.set_tree_view(view);
                if let (Some(name), Some(model)) = (step, &self.workbench.model)
                    && let Some(i) = model.fe.steps.iter().position(|s| s.name == name)
                {
                    self.workbench.tree.selected = Some((TreeView::FeModel, TreeItem::Step(i)));
                }
                // The results bound to the analysis open after the model, without taking
                // the view away again.
                let bound =
                    (self.workbench.model.as_ref()).is_some_and(|m| m.bound_results.is_some());
                self.workbench.pinned_view = (bound && view != TreeView::Results).then_some(view);
            }
        }
        // Under the System theme, the operating system may switch between light and dark.
        crate::style::sync(&ctx);
        let shift_command = egui::Modifiers::COMMAND | egui::Modifiers::SHIFT;
        if ctx.input_mut(|i| i.consume_key(shift_command, egui::Key::S)) {
            self.workbench.requests.push(Request::Save(true));
        }
        if ctx.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, egui::Key::S)) {
            self.workbench.requests.push(Request::Save(false));
        }
        // A focused text field undoes its own typing.
        if !ctx.egui_wants_keyboard_input() {
            // Ctrl+Z matches with Shift held too, so Redo is checked first.
            if ctx.input_mut(|i| i.consume_key(shift_command, egui::Key::Z)) {
                self.workbench.redo();
            }
            if ctx.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, egui::Key::Z)) {
                self.workbench.undo();
            }
        }
        let dropped: Vec<PathBuf> = ctx.input(|i| {
            (i.raw.dropped_files.iter())
                .map(|f| f.path().to_path_buf())
                .filter(|p| !p.as_os_str().is_empty())
                .collect()
        });
        // Several CAD files dropped together are imported into one geometry, as with
        // Geometry > Import; otherwise the first file opens.
        // A project opens in the project window; geometry and meshes are imported into the
        // analysis. Results only come from the analysis itself.
        let (projects, dropped): (Vec<PathBuf>, Vec<PathBuf>) =
            dropped.into_iter().partition(|p| model::is_project_file(p));
        if let Some(path) = projects.into_iter().next() {
            self.workbench.requests.push(Request::OpenProject(path));
        }
        let (results, dropped): (Vec<PathBuf>, Vec<PathBuf>) =
            dropped.into_iter().partition(|p| is_results_file(p));
        if !results.is_empty() {
            self.workbench.output.push(
                "Results files are not opened on their own: the Results tab shows the \
                 results of the analysis (Solve)."
                    .into(),
            );
        }
        if !dropped.is_empty() {
            self.import_paths(dropped, &ctx);
        }

        let workspace = self.workspace();
        let mut ribbon = std::mem::take(&mut self.ribbon);
        let clicked = egui::Panel::top("ribbon")
            .frame(egui::Frame::new().fill(crate::style::control()))
            .show(ui, |ui| {
                crate::ribbon::show(ui, &mut ribbon, workspace, &mut commands::RibbonHost(self))
            })
            .inner;
        self.ribbon = ribbon;
        if let Some(cmd) = clicked {
            self.run_command(&ctx, cmd);
        }

        self.workbench.animate(&ctx);
        self.workbench.play_sound(&ctx);
        let cancel = egui::Panel::bottom("status").show(ui, |ui| self.status_bar(ui));
        if cancel.inner {
            self.workbench.cancel_meshing();
        }
        // The fixed layout: tree on the left over the full height, 3D view with the
        // output below it; only the separators move.
        let pane = egui::Frame::new()
            .fill(crate::style::window())
            .stroke(egui::Stroke::new(1.0, crate::style::border()));
        // Below the narrow width the tree folds away behind a button of the app bar.
        if !crate::layout::tree_folded(&ctx) {
            egui::Panel::left("tree")
                .resizable(true)
                .default_size(280.0)
                .size_range(160.0..=700.0)
                .frame(egui::Frame::new().fill(crate::style::control()))
                .show(ui, |ui| {
                    // The tree of the workspace; the segments of the app bar switch it.
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.add_space(6.0);
                        ui.strong(workspace.title());
                    });
                    ui.add_space(2.0);
                    pane.inner_margin(4).show(ui, |ui| {
                        ui.set_min_size(ui.available_size());
                        let view = self.workbench.tree_view;
                        self.workbench.model_tree(ui, view);
                    });
                });
        }
        self.workbench.follow_selected_step();
        // The properties panel takes its room before the output pane; lying over the 3D view
        // in a narrow window, it ends above the output pane.
        let overlay = crate::layout::current(&ctx).overlay;
        if !overlay {
            self.properties_panel(ui, workspace);
        }
        // In a small window the output pane shows only its tabs until one is clicked.
        if crate::layout::output_folded(&ctx) {
            egui::Panel::bottom("output folded")
                .resizable(false)
                .exact_size(OUTPUT_TABS)
                .frame(egui::Frame::new().fill(crate::style::control()))
                .show(ui, |ui| self.workbench.output(ui, pane, true));
        } else {
            egui::Panel::bottom("output")
                .resizable(true)
                .default_size(170.0)
                .size_range(70.0..=600.0)
                .frame(egui::Frame::new().fill(crate::style::control()))
                .show(ui, |ui| self.workbench.output(ui, pane, false));
        }
        if overlay {
            self.properties_panel(ui, workspace);
        }
        egui::CentralPanel::no_frame().show(ui, |ui| {
            self.workbench.viewport.selecting = self.workbench.picking()
                || self.workbench.section_picks()
                || self.workbench.transformation_picks()
                || self.workbench.feature_picks();
            self.workbench.viewport.gizmo =
                match (&self.workbench.section_dialog, self.workbench.shown()) {
                    (Some(dialog), Some(model)) => Some(dialog.gizmo(model)),
                    _ => None,
                };
            let response = self.workbench.viewport.ui(ui);
            if let (Some(drag), Some(dialog)) = (response.gizmo, &mut self.workbench.section_dialog)
            {
                dialog.drag(drag);
            }
            self.workbench.path_plot_events(response.path_plots);
            if response.edit_legend {
                self.workbench.open_color_scale();
            }
            if let Some(command) = response.command {
                self.workbench.view_command = Some(command);
            }
            if let Some(click) = response.pivot {
                // Rotate about the model point under the cursor, or about the view centre
                // when the cursor points into empty space.
                let pivot = (self.workbench.shown()).and_then(|model| model.pick_click(&click));
                self.workbench
                    .viewport
                    .set_pivot(pivot.map(|hit| hit.point));
            }
            if let Some(click) = response.click {
                self.workbench.click(click);
            }
            if let Some(view) = &response.response {
                self.workbench.viewport_menu(view, response.secondary_click);
            }
            if let Some(command) = response.annotation {
                self.workbench.annotations(command);
            }
            if let Some(area) = response.box_select {
                self.workbench.box_select(&area);
            }
            if let Some(hover) = response.hover {
                self.workbench.hover(hover);
                ui.ctx().request_repaint();
            }
        });
        let view = self.workbench.viewport.rect;
        // The floating bar, at the bottom of what the properties panel leaves of the view;
        // not in a screenshot.
        if self.workbench.shown().is_some() && !self.workbench.screenshot.capturing(&ctx) {
            let mut free = view;
            if let Some(panel) = crate::dock::over_view(&ctx) {
                free.max.x = free.max.x.min(panel.left());
            }
            let bar = crate::ribbon::VIEW_BAR;
            let host = &mut commands::RibbonHost(self);
            if let Some(cmd) = crate::ribbon::floating_bar(&ctx, "view", free, bar, host) {
                self.ribbon.reveal(workspace, cmd);
                self.run_command(&ctx, cmd);
            }
        }
        let workbench = &mut self.workbench;
        workbench
            .screenshot
            .update(&ctx, view, &mut workbench.output);
        self.workbench.properties_window(&ctx);
        self.workbench.editor_window(&ctx);
        self.workbench.update_motion_preview(&ctx);
        self.workbench.update_cyclic_preview();
        self.workbench.confirm_delete_window(&ctx);
        self.workbench.confirm_remove_results(&ctx);
        self.workbench.toast(&ctx);
        self.workbench.contact_search_window(&ctx);
        self.workbench.contact_worksheet_window(&ctx);
        self.workbench.mesh_tool_window(&ctx);
        self.workbench.query_window(&ctx);
        self.workbench.section_window(&ctx);
        self.workbench.exploded_window(&ctx);
        self.workbench.material_library_window(&ctx);
        self.workbench.mesh_setup_window(&ctx);
        self.workbench.mesh_item_window(&ctx);
        self.workbench.poll_meshing();
        self.workbench.field_output_window(&ctx);
        self.workbench.history_output_window(&ctx);
        self.workbench.hot_spot_dialog_window(&ctx);
        self.workbench.history_table_window(&ctx);
        self.workbench.transformation_window(&ctx);
        self.workbench.feature_window(&ctx);
        self.workbench.plot_editor_window(&ctx);
        self.workbench.color_scale_window(&ctx);
        self.workbench.vtk_export_window(&ctx);
        self.workbench.sync_result_features();
        self.workbench.run_analysis(&ctx);
        self.workbench.poll_bound_results(&ctx);
        self.workbench.poll_global_results(&ctx);
        if let Some(path) = self.workbench.open_results.take() {
            self.workbench.results_asked = Some(path.clone());
            self.open_results(path, &ctx);
        }
        self.workbench.update_highlight();
        self.workbench.update_symbols(&ctx);
        self.workbench.update_hot_spot_preview();
        self.workbench.findings_window(&ctx);
        self.workbench.settings_window(&ctx);
        self.workbench.model_dialog_window(&ctx);
        self.workbench.linked_prompt_window(&ctx);
        self.workbench.update_explosion(&ctx);
        self.workbench.update_thick_shells();
        self.workbench.rebuild_if_results_changed();
        self.workbench.update_section();
        self.workbench.update_feature_overlay();
        self.workbench.update_path_plots();

        if let Some(command) = self.workbench.view_command.take() {
            let bounds = self.workbench.shown().and_then(Model::visible_bounds);
            if matches!(command, ViewCommand::Fit) && bounds.is_none() {
                // Nothing to fit yet, e.g. on the empty Geometry tab of an analysis opened
                // from its mesh: the FE Model tab, which shares the camera, fits instead.
                self.workbench.view_command = Some(command);
            } else {
                self.workbench.viewport.apply(command, bounds);
            }
        }
        // Last, after everything the user changed in this frame.
        if let Some(model) = &mut self.workbench.model {
            crate::undo::record(model);
        }
    }
}

/// What a worker restoring bound results sends back: the analysis, the directory and the
/// results file written there.
struct BoundWork(String, PathBuf, Result<PathBuf, String>);

/// The global results of a submodel, read on a worker thread.
type GlobalWork = Result<Box<model::GlobalModel>, String>;

/// Drops the features of saved results settings that came from the FE model and are gone
/// from `current`.
fn drop_removed_features(
    settings: &mut plx_results::ResultsSettings,
    current: &plx_model::FeModel,
) {
    let mut fe = plx_model::FeModel {
        reference_points: std::mem::take(&mut settings.reference_points),
        coordinate_systems: std::mem::take(&mut settings.coordinate_systems),
        planes: std::mem::take(&mut settings.planes),
        ..plx_model::FeModel::default()
    };
    fe.drop_removed_features(&settings.inherited_features, current);
    let kept = fe.feature_keys();
    settings.inherited_features.retain(|key| kept.contains(key));
    settings.reference_points = fe.reference_points;
    settings.coordinate_systems = fe.coordinate_systems;
    settings.planes = fe.planes;
}

fn saved_camera(saved: plx_results::saved_view::SavedCamera) -> plx_render::Camera {
    plx_render::Camera {
        target: glam::Vec3::from_array(saved.target),
        distance: saved.distance,
        rotation: glam::Quat::from_array(saved.rotation).normalize(),
        scene_radius: saved.scene_radius,
    }
}

/// A file size in kB or MB.
pub(crate) fn file_size(bytes: u64) -> String {
    let kb = bytes as f64 / 1024.0;
    if kb < 1024.0 {
        format!("{kb:.0} kB")
    } else {
        format!("{:.1} MB", kb / 1024.0)
    }
}

/// Geometry imported into a model that has nothing yet keeps its FE model and its file: a
/// project saved before the import is still saved to its own file, not to the CAD file.
fn take_over_empty_model(model: &mut Model, old: &Model) {
    model.fe = old.fe.clone();
    model.path = old.path.clone();
}

/// Whether results read from `loaded` are the ones asked for last. Results are read on
/// worker threads, so those of an earlier request, e.g. the results kept in the project
/// being restored when a new run's results are opened, can arrive after newer ones and
/// would replace them.
fn is_latest_results(asked: Option<&std::path::Path>, loaded: &std::path::Path) -> bool {
    asked.is_none_or(|asked| asked == loaded)
}

impl Workbench {
    fn model_loaded(&mut self, path: PathBuf, result: Result<Box<LoadedModel>, String>) {
        match result {
            Ok(loaded) => {
                let LoadedModel {
                    mut model,
                    render_meshes,
                    geometry_view,
                } = *loaded;
                if model.results.is_some()
                    && !is_latest_results(self.results_asked.as_deref(), &path)
                {
                    self.output.push(format!(
                        "{} not shown: newer results were opened since.",
                        path.display()
                    ));
                    return;
                }
                // Geometry imported into a model that has nothing yet, such as one just
                // created with File > New, has to fit its model space.
                let into_empty = (self.model.as_ref()).filter(|old| {
                    plx_mesher::is_cad_file(&path)
                        && old.mesh.element_count() == 0
                        && old.geometry.is_none()
                });
                let mut turned_faces = Vec::new();
                if let (Some(old), Some(view)) = (into_empty, &geometry_view) {
                    match geometry_check(old.fe.properties.space, view) {
                        Ok(faces) => turned_faces = faces,
                        Err(error) => {
                            self.output
                                .push(format!("Error importing {}: {error}", path.display()));
                            return;
                        }
                    }
                }
                // The scene built on the worker thread is reused unless the settings change it.
                let mut rebuild = false;
                // The camera of bound results as saved, replacing the fit to the model.
                let mut camera = None;
                // Results opened while another workspace stays shown, e.g. an analysis opened
                // from a Setup row of its block.
                let mut results_hidden = false;
                if let Some(view) = &mut model.results {
                    let post = &self.settings.post;
                    let levels = view.color_scale.levels;
                    if (levels, view.show_undeformed) != (post.levels, post.undeformed_outline) {
                        view.color_scale.levels = post.levels;
                        view.show_undeformed = post.undeformed_outline;
                        rebuild = true;
                    }
                }
                match &geometry_view {
                    Some(view) if model.mesh.element_count() == 0 => {
                        // Several files imported together are listed by name.
                        let source = model.geometry.as_ref().map(|g| g.source.as_str());
                        let single = path.file_name().and_then(|n| n.to_str()) == source;
                        let what = match source {
                            Some(names) if !single => names.to_string(),
                            _ => path.display().to_string(),
                        };
                        self.output.push(format!(
                            "{what} imported: {} parts ({} ms)",
                            view.parts.len(),
                            model.load_time.as_millis()
                        ));
                    }
                    _ => self.output.push(format!(
                        "{} loaded: {} nodes, {} elements, {} parts ({} ms)",
                        path.display(),
                        model.mesh.node_count(),
                        model.mesh.element_count(),
                        model.parts.len(),
                        model.load_time.as_millis()
                    )),
                }
                if model.included_files > 0 {
                    self.output
                        .push(format!("{} included file(s) read", model.included_files));
                }
                for warning in &model.warnings {
                    self.output.push(format!("Warning: {warning}"));
                }
                for note in &model.notes {
                    self.output.push(format!("Note: {note}"));
                }
                if !turned_faces.is_empty() {
                    let faces: Vec<String> = turned_faces.iter().map(i32::to_string).collect();
                    self.output.push(format!(
                        "Note: the normal of face {} points in -z; the elements are flipped \
                         when meshing.",
                        faces.join(", ")
                    ));
                }
                if !model.skipped_keywords.is_empty() {
                    let keywords: Vec<String> = model
                        .skipped_keywords
                        .keys()
                        .map(|k| format!("*{k}"))
                        .collect();
                    self.output
                        .push(format!("Not evaluated yet: {}", keywords.join(", ")));
                }
                if let Some(view) = &model.results {
                    self.output.push(format!(
                        "{} result increment(s) read",
                        view.increments.len()
                    ));
                    if !view.file_history.is_empty() {
                        // The built-in solver's results hold their history output.
                        let source = if plx_results::solver::is_solver_results(&model.path) {
                            "with the results"
                        } else {
                            "from the .dat file"
                        };
                        self.output.push(format!(
                            "{} history output(s) read {source}",
                            view.file_history.len()
                        ));
                    }
                }
                self.dialog = None;
                self.field_output_dialog = None;
                self.close_history_windows();
                self.viewport.labels = Default::default();
                let mut model = model;
                if let Some(view) = &mut model.results {
                    // Results are in the unit system of the open FE model, as those of its
                    // analyses are; without one in that of new models.
                    view.units = self
                        .model
                        .as_ref()
                        .map_or(self.settings.new_model.units, |m| m.fe.properties.units);
                }
                if let Some(view) = &model.results {
                    // A results file joins the results collection and leaves the FE model
                    // alone; opening the same file again replaces it.
                    self.tree.selected = Some((
                        TreeView::Results,
                        TreeItem::Component(view.field, view.component),
                    ));
                    let shown = results_workspace(&mut self.pinned_view);
                    self.set_tree_view(shown);
                    results_hidden = self.tree_view != TreeView::Results;
                    // Results of a 2D model are flat in z and seen from the front.
                    if !results_hidden
                        && model
                            .mesh
                            .bounds()
                            .is_some_and(|(min, max)| (max[2] - min[2]).abs() < 1e-9)
                    {
                        self.viewport
                            .apply(ViewCommand::View(StandardView::Front), None);
                    }
                    let mut model = model;
                    // Results get the features of the FE model; reloaded ones keep their own
                    // and their paths.
                    // The Results tab shows the results of the analysis: new results, e.g.
                    // of another run, replace those shown.
                    let previous = (!self.results.is_empty()).then_some(0);
                    // Features taken over from the FE model are not kept when it lost them.
                    let features = self.model.as_ref().map(|project| {
                        let mut features = project.fe.clone();
                        features.convert_features(&plx_model::convert::Conversion::new(
                            plx_model::STORED,
                            model.units(),
                        ));
                        features
                    });
                    let current = features.clone().unwrap_or_default();
                    if let Some(index) = previous {
                        let (old, inherited) = (&self.results[index].fe, &self.results[index]);
                        let mut old = old.clone();
                        if features.is_some() {
                            old.drop_removed_features(&inherited.inherited_features, &current);
                        }
                        let kept = old.feature_keys();
                        model.inherited_features = (inherited.inherited_features.iter())
                            .filter(|key| kept.contains(key))
                            .cloned()
                            .collect();
                        model.fe.add_missing_features(&old);
                        model.fe.result_paths = old.result_paths.clone();
                        model.fe.result_planes = old.result_planes.clone();
                    }
                    // Results of a new run take over what was defined on those before; what
                    // refers to nodes or elements only if the mesh stayed the same.
                    let carried = (self.carried_settings.take())
                        .filter(|(path, _)| *path == model.path)
                        .map(|(_, mut settings)| {
                            if features.is_some() {
                                drop_removed_features(&mut settings, &current);
                            }
                            if settings.mesh != Some(model.mesh.fingerprint()) {
                                let changes = settings.without_mesh_references();
                                (self.output).extend(changes.into_iter().map(|change| {
                                    format!("The mesh changed since the last results: {change}")
                                }));
                            }
                            settings
                        });
                    let carried_over = carried.is_some();
                    // Results bound to the project come back as they were saved.
                    let settings = carried.or_else(|| {
                        (self.model.as_ref())
                            .filter(|_| previous.is_none())
                            .filter(|p| p.bound_results_file.as_ref() == Some(&model.path))
                            .and_then(|p| p.bound_results.as_ref()?.settings.clone())
                    });
                    if let Some(settings) = &settings {
                        let warnings =
                            model.restore_results_settings(settings, self.moving_load_stations());
                        self.output.extend(warnings);
                        camera = settings.view.camera.map(saved_camera);
                        if let Some(edges) = settings.view.mesh_edges {
                            self.viewport.options.mesh_edges = edges;
                        }
                        if let Some(view) = &model.results {
                            self.tree.selected = Some((
                                TreeView::Results,
                                TreeItem::Component(view.field, view.component),
                            ));
                        }
                        rebuild = true;
                    }
                    if let Some(features) = &features {
                        // Results are stored in the units they show; the FE model in SI ones.
                        let added = model.fe.add_missing_features(features);
                        for key in added {
                            if !model.inherited_features.contains(&key) {
                                model.inherited_features.push(key);
                            }
                        }
                    }
                    // Results read again, e.g. after the analysis ran once more, keep their
                    // colour scale.
                    let previous_scale = previous
                        .and_then(|i| self.results[i].results.as_ref())
                        .map(|v| v.color_scale.clone());
                    if let (Some(view), Some(scale)) = (model.results.as_mut(), previous_scale) {
                        view.color_scale = scale;
                    }
                    match previous {
                        Some(index) => {
                            // Results read again, e.g. after the analysis ran once more,
                            // keep their hot spots.
                            let mut model = model;
                            let old = std::mem::take(&mut self.results[index].hot_spots);
                            if !carried_over {
                                model.hot_spots.definitions = old.definitions;
                            }
                            self.results[index] = model;
                            self.current_result = index;
                        }
                        None => {
                            self.results.push(model);
                            self.current_result = self.results.len() - 1;
                        }
                    }
                    self.evaluate_hot_spots();
                    self.load_global_results();
                } else {
                    self.tree.selected = None;
                    self.editor = None;
                    self.highlighted = None;
                    self.mesh_setup = None;
                    self.mesh_item_editor = None;
                    self.meshing = None;
                    // A model without a mesh yet opens on the Geometry tab.
                    let view = if geometry_view.is_some() && model.mesh.element_count() == 0 {
                        TreeView::Geometry
                    } else {
                        TreeView::FeModel
                    };
                    self.set_tree_view(view);
                    // Geometry imported into a model that has nothing yet, such as one just
                    // created with File > New, keeps its model space and units.
                    let mut model = model;
                    if let Some(old) = &self.model
                        && plx_mesher::is_cad_file(&path)
                        && old.mesh.element_count() == 0
                        && old.geometry.is_none()
                    {
                        take_over_empty_model(&mut model, old);
                    }
                    // 2D models lie in the x-y plane and are seen from the front.
                    if model.fe.properties.space.is_2d() {
                        self.viewport
                            .apply(ViewCommand::View(StandardView::Front), None);
                    }
                    self.model = Some(model);
                    self.symbols_shown = None;
                    self.tree.acting_for = None;
                    self.geometry = geometry_view;
                    // Results bound to the project open with it.
                    if self
                        .model
                        .as_ref()
                        .is_some_and(|m| m.bound_results.is_some())
                    {
                        self.open_bound_results();
                    }
                }
                if results_hidden {
                    // The view keeps the workspace shown and its camera; the results take
                    // theirs, in their own units, when the Results tab is chosen, or a fit
                    // without a saved one.
                    self.parked_camera = camera;
                    self.results_changed = true;
                    return;
                }
                self.frame_cache.clear();
                self.viewport.set_parts(&render_meshes);
                let hidden: Vec<usize> = (self.shown().into_iter())
                    .flat_map(|m| m.parts.iter().enumerate())
                    .filter(|(_, p)| !p.visible)
                    .map(|(i, _)| i)
                    .collect();
                for part in hidden {
                    self.viewport.set_part_visible(part, false);
                }
                self.results_changed = rebuild;
                self.update_contour();
                match camera {
                    Some(camera) => {
                        self.viewport.set_camera(camera);
                        self.view_command = None;
                    }
                    None => self.view_command = Some(ViewCommand::Fit),
                }
            }
            Err(error) => self.output.push(format!("Error loading: {error}")),
        }
    }

    /// The pane below the 3D view: the messages, and in the Analysis tab the output of
    /// the solver, so the results stay in view while it runs.
    fn output(&mut self, ui: &mut egui::Ui, pane: egui::Frame, folded: bool) {
        let ctx = ui.ctx().clone();
        // The Analysis tab, brought to the front by a solve, covers the problems.
        if self.analysis_tab && !self.analysis_tab_before {
            self.problems_tab = false;
            crate::layout::fold_output(&ctx, false);
        }
        if self.problems_tab && !self.problems_tab_before {
            crate::layout::fold_output(&ctx, false);
        }
        self.problems_tab_before = self.problems_tab;
        let problems = self.problems();
        let problems_title = format!("Problems ({})", problems.len());
        let selected = if self.problems_tab {
            0
        } else if self.analysis_tab {
            1
        } else {
            2
        };
        let titles = [problems_title.as_str(), "Analysis", "Output"];
        let top = ui.cursor().top();
        let (clicked, selected_tab) = crate::style::tabs(ui, &titles, selected);
        let row = egui::Rect::from_x_y_ranges(ui.max_rect().x_range(), top..=ui.cursor().top());
        if let Some(index) = clicked {
            self.problems_tab = index == 0;
            self.analysis_tab = index == 1;
            if folded {
                crate::layout::fold_output(&ctx, false);
            }
        }
        self.analysis_tab_before = self.analysis_tab;
        let double = ui.input(|i| {
            i.pointer
                .button_double_clicked(egui::PointerButton::Primary)
                && i.pointer.interact_pos().is_some_and(|p| row.contains(p))
        });
        if double && !folded {
            crate::layout::fold_output(&ctx, true);
        }
        if folded {
            crate::style::open_tab(ui, selected_tab);
            return;
        }
        let mut event = MonitorEvent::None;
        let mut picked = None;
        pane.inner_margin(4).show(ui, |ui| {
            ui.set_min_size(ui.available_size());
            if self.problems_tab {
                picked = problems_list(ui, &problems);
            } else if !self.analysis_tab {
                crate::virtual_table::lines(
                    ui,
                    egui::ScrollArea::both()
                        .auto_shrink([false, false])
                        .stick_to_bottom(true),
                    &self.output,
                );
            } else if let Some(run) = &mut self.project_run {
                event = run.monitor(ui, &self.analysis_name);
            } else if let Some(analysis) = &mut self.analysis {
                event = analysis.monitor(ui);
            } else {
                ui.weak(
                    "No analysis run yet: Solve (F5) starts the solver; its output shows here.",
                );
            }
        });
        crate::style::open_tab(ui, selected_tab);
        if let Some((finding, open)) = picked {
            self.pick_problem(ui.ctx(), &finding, open);
        }
        match event {
            MonitorEvent::OpenResults => self.open_bound_results(),
            MonitorEvent::Stop => self.stop(),
            MonitorEvent::None => {}
        }
    }

    /// The problems of the FE model: what the model check finds and what the solver found
    /// in the last run.
    fn problems(&self) -> Vec<plx_model::Finding> {
        let mut findings = (self.model.as_ref()).map_or_else(Vec::new, Model::findings);
        findings.extend(self.solver_findings.iter().cloned());
        // Errors first.
        findings.sort_by_key(|f| f.severity() != plx_model::Severity::Error);
        findings
    }

    /// A problem was clicked: its item is selected in the FE Model tree; a double-click
    /// opens it in the properties panel.
    fn pick_problem(&mut self, ctx: &egui::Context, finding: &plx_model::Finding, open: bool) {
        let (item, _) = tree::tree_items(finding.item);
        if !matches!(item, TreeItem::Model) {
            self.set_tree_view(TreeView::FeModel);
            self.tree.selected = Some((TreeView::FeModel, item.clone()));
            self.tree.reveal = true;
        }
        if open {
            let response = TreeResponse {
                open: Some(item),
                ..Default::default()
            };
            self.tree_response(ctx, TreeView::FeModel, response);
        }
    }

    fn model_tree(&mut self, ui: &mut egui::Ui, view: TreeView) {
        let empty = match view {
            TreeView::Geometry => self.geometry.is_none().then_some("No geometry loaded"),
            TreeView::FeModel => self.model.is_none().then_some("No model loaded"),
            TreeView::Results => self.results.is_empty().then_some("No results loaded"),
        };
        if let Some(text) = empty {
            let hint = if view == TreeView::Geometry {
                "Import Geometry in the ribbon, or drag a STEP, IGES or BREP file into the window."
            } else {
                "Import Geometry in the ribbon, or drag a STEP or .inp file into the window."
            };
            ui.weak(format!("{text}.\n{hint}"));
            ui.separator();
        }
        let mesh_items: Vec<String> = (self.model.as_ref())
            .and_then(|m| m.geometry.as_ref())
            .map(|g| g.mesh_items.iter().map(|i| i.name.clone()).collect())
            .unwrap_or_default();
        self.tree.linked = self.linked_tree_items(view);
        self.tree.hide_linked = !self.settings.graphics.show_linked;
        if view == TreeView::FeModel && !self.tree.linked.is_empty() {
            ui.checkbox(
                &mut self.settings.graphics.show_linked,
                settings::SHOW_LINKED_LABEL,
            )
            .on_hover_text(settings::SHOW_LINKED_TIP);
            ui.separator();
        }
        if view == TreeView::FeModel {
            self.update_acting();
        }
        self.tree.active_step = self.active_step();
        let shown = match view {
            TreeView::Results => self.results.get_mut(self.current_result),
            TreeView::Geometry => self.geometry.as_mut(),
            TreeView::FeModel => self.model.as_mut(),
        };
        let response = tree::show(
            ui,
            view,
            shown,
            &mesh_items,
            &self.solver_findings,
            &mut self.tree,
        );
        self.tree_response(ui.ctx(), view, response);
    }

    /// The items of the tree in `view` that another analysis owns, with its name.
    fn linked_tree_items(&self, view: TreeView) -> HashMap<TreeItem, String> {
        let mut linked = HashMap::new();
        let Some(model) = &self.model else {
            return linked;
        };
        for (path, source) in &self.incoming.paths {
            let items = match view {
                TreeView::Results => Vec::new(),
                TreeView::Geometry
                    if path[0] == crate::links::GEOMETRY || path[0] == crate::links::MESH =>
                {
                    let mut items = vec![TreeItem::Group("Parts")];
                    items.extend(
                        (0..self.geometry.as_ref().map_or(0, |g| g.parts.len()))
                            .map(TreeItem::Part),
                    );
                    // A geometry link takes the shapes only, the mesh setup stays this
                    // analysis' own; a mesh link takes the setup it was meshed with too.
                    if path[0] == crate::links::MESH {
                        items.push(TreeItem::Group("Mesh Setup"));
                        let setup = model.geometry.as_ref().map_or(0, |g| g.mesh_items.len());
                        items.extend((0..setup).map(TreeItem::MeshItem));
                    }
                    items
                }
                TreeView::Geometry => Vec::new(),
                TreeView::FeModel if path[0] == crate::links::MESH => {
                    let mut items = vec![TreeItem::Mesh, TreeItem::Group("Parts")];
                    items.extend((0..model.parts.len()).map(TreeItem::Part));
                    items
                }
                TreeView::FeModel => crate::links::tree_items(&model.fe, path),
            };
            for item in items {
                linked.insert(item, source.clone());
            }
        }
        linked
    }

    /// The analysis a tree item comes from, if another analysis owns it.
    fn linked_source(&self, view: TreeView, item: &TreeItem) -> Option<(Vec<String>, String)> {
        let source = self.tree.linked.get(item)?;
        let path = match view {
            // The mesh setup is linked only with the mesh.
            TreeView::Geometry => match item {
                TreeItem::Group("Mesh Setup") | TreeItem::MeshItem(_) => {
                    vec![crate::links::MESH.to_owned()]
                }
                _ => vec![crate::links::GEOMETRY.to_owned()],
            },
            _ => (self.model.as_ref()).and_then(|m| crate::links::path_of(&m.fe, item))?,
        };
        // The link that covers it: a whole category rather than the item in it.
        let path = (self.incoming.paths.iter())
            .find(|(prefix, s)| s == source && crate::links::covers(prefix, &path))
            .map_or(path, |(prefix, _)| prefix.clone());
        Some((path, source.clone()))
    }

    /// Keeps the user from changing in the tree what another analysis owns: a double-click
    /// asks whether to open that analysis or unlink the item, other changes are refused.
    fn guard_linked(&mut self, view: TreeView, response: &mut TreeResponse) {
        if self.tree.linked.is_empty() {
            return;
        }
        if let Some(item) = &response.open
            && let Some(linked) = self.linked_source(view, item)
        {
            response.open = None;
            self.linked_prompt = Some(linked);
        }
        let mut refused: Option<String> = None;
        let mut check = |item: Option<&TreeItem>, this: &Self| {
            let linked = item.and_then(|item| this.linked_source(view, item));
            if let Some((path, source)) = linked {
                refused = Some(format!(
                    "{} comes from {source}: change it there, or unlink it first.",
                    crate::links::label(&path)
                ));
                true
            } else {
                false
            }
        };
        if check(response.delete.as_ref(), self) {
            response.delete = None;
        }
        if check(response.toggle_active.as_ref(), self) {
            response.toggle_active = None;
        }
        if check(response.swap_master_slave.as_ref(), self) {
            response.swap_master_slave = None;
        }
        if let Some((step, _)) = response.move_step
            && check(Some(&TreeItem::Step(step)), self)
        {
            response.move_step = None;
        }
        if let Some(kind) = &response.create
            && check(new_item_container(kind).as_ref(), self)
        {
            response.create = None;
        }
        let mesh = TreeItem::Mesh;
        let meshing = response.mesh_part.is_some()
            || response.transform_part.is_some()
            || response.merge_parts;
        let mesh_linked =
            (self.incoming.paths.iter()).any(|(path, _)| path[0] == crate::links::MESH);
        if meshing && (check(Some(&mesh), self) || mesh_linked) {
            if refused.is_none() {
                refused = Some("The mesh comes from another analysis: mesh it there.".into());
            }
            response.mesh_part = None;
            response.transform_part = None;
            response.merge_parts = false;
        }
        if let Some(line) = refused {
            self.output.push(line);
        }
    }

    /// Asks what to do about a linked item that was double-clicked.
    fn linked_prompt_window(&mut self, ctx: &egui::Context) {
        let Some((path, source)) = &self.linked_prompt else {
            return;
        };
        let mut close = false;
        let mut open = true;
        egui::Window::new("Linked Item")
            .id(crate::ids::window(ctx, "linked item"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                ui.label(format!(
                    "{} comes from the analysis {source}. Change it there, or unlink it: this \
                     analysis then keeps its own copy.",
                    crate::links::label(path)
                ));
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    if ui.button(format!("Open {source}")).clicked() {
                        self.requests.push(Request::OpenAnalysis(source.clone()));
                        close = true;
                    }
                    if ui.button("Unlink").clicked() {
                        self.requests.push(Request::Unlink(path.clone()));
                        close = true;
                    }
                    if ui.button("Cancel").clicked() {
                        close = true;
                    }
                });
            });
        if close || !open {
            self.linked_prompt = None;
        }
    }

    /// Acts on what the user picked in the tree or in a part's context menu in the 3D view.
    fn tree_response(&mut self, ctx: &egui::Context, view: TreeView, mut response: TreeResponse) {
        self.guard_linked(view, &mut response);
        for (index, visible) in response.visibility {
            if let Some(part) = self.shown_mut().and_then(|m| m.parts.get_mut(index)) {
                part.visible = visible;
            }
            self.viewport.set_part_visible(index, visible);
        }
        if !response.contours.is_empty() {
            for (index, contours) in response.contours {
                if let Some(model) = self.shown_mut() {
                    model.set_contours(&[index], contours);
                }
            }
            self.results_changed = true;
        }
        if let (Some((field, component)), Some(results)) =
            (response.component, self.shown_results_mut())
        {
            results.field = field;
            results.component = component;
            self.results_changed = true;
        }
        if let (Some(shown), Some(results)) = (response.plane_result, self.shown_results_mut()) {
            results.plane_result = shown;
        }
        if let Some(TreeItem::Part(index)) = response.open {
            let name = self.shown().and_then(|m| m.parts.get(index));
            let types = (name.into_iter())
                .flat_map(|p| p.element_types.iter().map(|(t, _)| t.clone()))
                .collect();
            self.part_name = (name.map(|p| p.name.clone()).unwrap_or_default(), None);
            self.part_types = types;
            self.dialog = Some(TreeItem::Part(index));
        } else if let Some(TreeItem::ResultFieldOutput(field)) = response.open {
            self.edit_field_output(field);
        } else if let Some(TreeItem::HistorySet(set)) = response.open {
            self.edit_history_output(set);
        } else if let Some(TreeItem::HotSpot(index)) = response.open {
            self.edit_hot_spot(index);
        } else if let Some(TreeItem::HistoryComponent(set, field, component)) = response.open {
            let same = (self.history_table.as_ref())
                .is_some_and(|t| (t.set, t.field, t.component) == (set, field, component));
            if !same {
                self.history_table = Some(HistoryTable::new(set, field, component));
            }
        } else if let (Some(TreeItem::Model), TreeView::FeModel) = (&response.open, view) {
            self.edit_model_properties();
        } else if let Some(TreeItem::MeshItem(index)) = response.open {
            self.mesh_item_editor = self.model.as_ref().and_then(|m| {
                MeshItemEditor::edit(m.geometry.as_ref()?, index, m.fe.properties.units)
            });
        } else if let Some(feature) = response.open.as_ref().and_then(tree::feature) {
            let results = (view == TreeView::Results).then_some(self.current_result);
            if let Some(model) = self.feature_model(results) {
                self.feature_dialog = FeatureDialog::edit(feature, results, model);
            }
        } else if let Some(item) = response.open {
            let model = self.model.as_ref().filter(|_| view != TreeView::Results);
            match model.and_then(|m| Editor::edit(&item, &m.fe, &m.mesh)) {
                Some(editor) => self.editor = Some(editor),
                None => self.dialog = Some(item),
            }
        }
        if let Some(kind) = response.create {
            self.create(kind);
        }
        if let Some(item) = response.delete {
            // A part or item among several selected ones deletes them all.
            let parts = self.tree.selected_parts(view);
            let selected = self.tree.selected_items(view);
            let items = match item {
                TreeItem::Part(index) if parts.contains(&index) => {
                    parts.into_iter().map(TreeItem::Part).collect()
                }
                item if selected.contains(&item) => selected,
                item => vec![item],
            };
            self.confirm_delete = Some((view, items));
        }
        if let (Some((s, up)), Some(model)) = (response.move_step, self.model.as_mut())
            && let Some(to) = crate::setup::move_step(&mut model.fe, s, up)
        {
            // The moved step stays selected.
            if let Some((_, item @ TreeItem::Step(_))) = &mut self.tree.selected {
                *item = TreeItem::Step(to);
            }
        }
        if let (Some(item), Some(model)) = (response.toggle_active, self.model.as_mut()) {
            // Several selected items all take the state the clicked one changes to.
            let selected = self.tree.selected_items(view);
            match crate::setup::is_active(&mut model.fe, &item) {
                Some(active) if selected.len() > 1 && selected.contains(&item) => {
                    for item in &selected {
                        crate::setup::set_active(&mut model.fe, item, !active);
                    }
                }
                _ => {
                    crate::setup::toggle_active(&mut model.fe, &item);
                }
            }
        }
        if let Some(model) = self.model.as_mut() {
            for (item, visible) in &response.symbol_visibility {
                crate::setup::set_visible(&mut model.fe, item, *visible);
            }
        }
        if let (Some(item), Some(model)) = (response.swap_master_slave, self.model.as_mut()) {
            // Several selected items are all swapped.
            let selected = self.tree.selected_items(view);
            let items = if selected.contains(&item) {
                selected
            } else {
                vec![item]
            };
            let mut swapped = false;
            for item in &items {
                swapped |= crate::setup::swap_master_slave(&mut model.fe, item);
            }
            if swapped {
                // The selection stays, but master and slave colours change places.
                self.highlighted = None;
            }
        }
        if let Some(index) = response.mesh_part {
            // A part among several selected ones meshes them all.
            let selected = self.tree.selected_parts(view);
            let indices: Vec<usize> = if selected.contains(&index) {
                selected.into_iter().collect()
            } else {
                vec![index]
            };
            let names: Vec<String> = match self.geometry.as_ref() {
                Some(g) => (indices.iter())
                    .filter_map(|&i| g.parts.get(i).map(|p| p.name.clone()))
                    .collect(),
                None => Vec::new(),
            };
            if !names.is_empty() {
                self.generate_mesh(ctx, Some(names));
            }
        }
        if let Some(index) = response.detect {
            self.detect_contacts(index);
        }
        if let Some(index) = response.transform_part {
            // A part among several selected ones transforms them all.
            let parts = self.tree.selected_parts(view);
            let parts = if parts.contains(&index) {
                parts.into_iter().collect()
            } else {
                vec![index]
            };
            self.open_mesh_tool(MeshTool::Transform(parts));
        }
        if response.merge_parts {
            let parts: Vec<usize> = self.tree.selected_parts(view).into_iter().collect();
            self.merge_mesh_parts(&parts);
        }
        if let Some(findings) = response.findings {
            self.findings_window = Some(findings);
        }
    }

    /// Explains the findings of the tree item whose warning sign was clicked: what is wrong,
    /// why the solver cannot cope with it and how to fix it.
    fn findings_window(&mut self, ctx: &egui::Context) {
        let Some(findings) = &self.findings_window else {
            return;
        };
        let mut open = true;
        let mut close = false;
        egui::Window::new("Model Check")
            .id(crate::ids::window(ctx, "model check"))
            .open(&mut open)
            .collapsible(false)
            .resizable(true)
            .default_width(460.0)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .max_height(ctx.content_rect().height() * 0.6)
                    .show(ui, |ui| {
                        for (i, finding) in findings.iter().enumerate() {
                            if i > 0 {
                                ui.separator();
                            }
                            let (kind, color) = match finding.severity() {
                                plx_model::Severity::Error => ("Error", crate::style::error()),
                                plx_model::Severity::Warning => {
                                    ("Warning", crate::style::warning())
                                }
                            };
                            ui.horizontal(|ui| {
                                ui.colored_label(color, egui::RichText::new(kind).strong());
                                ui.label(egui::RichText::new(finding.problem.title()).strong());
                            });
                            ui.add(
                                egui::Label::new(egui::RichText::new(&finding.detail).weak())
                                    .wrap(),
                            );
                            ui.add_space(4.0);
                            ui.add(egui::Label::new(finding.problem.explanation()).wrap());
                            ui.add_space(4.0);
                            ui.add(
                                egui::Label::new(format!("Remedy: {}", finding.problem.fix()))
                                    .wrap(),
                            );
                        }
                    });
                close = crate::dialog::work_footer(ui, None) == crate::dialog::Work::Close;
            });
        if !open || close {
            self.findings_window = None;
        }
    }

    /// Evaluates the hot spots of the current results file over all increments and writes
    /// the values next to it.
    fn evaluate_hot_spots(&mut self) {
        let moving_loads = self.moving_load_stations();
        let Some(results) = self.results.get_mut(self.current_result) else {
            return;
        };
        let reports = crate::hot_spots::evaluate(results);
        let hot_spots = &mut results.hot_spots;
        hot_spots.reports = Vec::new();
        hot_spots.file = None;
        if !reports.is_empty() {
            match crate::hot_spots::write(&results.path, &reports) {
                Ok(file) => {
                    self.output
                        .push(format!("Hot spots evaluated: {}", file.display()));
                    hot_spots.file = Some(file);
                }
                Err(error) => self.output.push(error),
            }
            self.output.extend(crate::hot_spots::summary(&reports));
            for report in &reports {
                self.output
                    .extend(report.warnings.iter().map(|w| format!("Warning: {w}")));
            }
            hot_spots.reports = reports.clone();
        }
        // History outputs of the hot spots follow them.
        if let Some(view) = &mut results.results
            && let Some(warnings) = view.set_history_sources(reports, moving_loads, &results.mesh)
        {
            (self.output).extend(warnings.into_iter().map(|w| format!("Warning: {w}")));
            self.history_table = None;
        }
        self.update_contour();
    }

    /// Takes the moving loads of the FE model as they are now into the current results, for
    /// the history outputs.
    fn refresh_moving_loads(&mut self) {
        let moving_loads = self.moving_load_stations();
        let Some(results) = self.results.get_mut(self.current_result) else {
            return;
        };
        if let Some(view) = &mut results.results {
            let reports = view.hot_spot_reports.clone();
            if let Some(warnings) = view.set_history_sources(reports, moving_loads, &results.mesh) {
                (self.output).extend(warnings.into_iter().map(|w| format!("Warning: {w}")));
                self.history_table = None;
            }
        }
    }

    /// Where the moving loads of the FE model stand in the steps of its results, in the
    /// units the results show.
    fn moving_load_stations(&self) -> Vec<plx_model::LoadStations> {
        let Some(project) = &self.model else {
            return Vec::new();
        };
        let units = project.fe.properties.units;
        let mut stations = plx_model::load_stations(&project.fe);
        for (_, position) in stations.iter_mut().flat_map(|s| s.steps.iter_mut()) {
            *position = units.shown(*position, plx_model::Quantity::Length);
        }
        stations
    }

    fn edit_hot_spot(&mut self, index: usize) {
        if let Some(model) = self.results.get(self.current_result) {
            self.hot_spot_dialog =
                HotSpotDialog::edit(&model.hot_spots.definitions, index, &model.mesh);
        }
    }

    fn delete_hot_spot(&mut self, index: usize) {
        let Some(model) = self.results.get_mut(self.current_result) else {
            return;
        };
        if index >= model.hot_spots.definitions.len() {
            return;
        }
        let removed = model.hot_spots.definitions.remove(index);
        self.output
            .push(format!("Hot spot {} deleted", removed.name));
        self.hot_spot_dialog = None;
        self.tree.selected = None;
        self.evaluate_hot_spots();
    }

    fn hot_spot_dialog_window(&mut self, ctx: &egui::Context) {
        let (Some(dialog), Some(model)) = (
            &mut self.hot_spot_dialog,
            self.results.get_mut(self.current_result),
        ) else {
            return;
        };
        let action = dialog.show(ctx, model);
        let plot_name = dialog.plot_name(&model.hot_spots.definitions);
        if dialog.take_plot_toggle() {
            match model
                .path_plots
                .iter()
                .position(|p| p.is_hot_spot(&plot_name))
            {
                Some(index) => {
                    model.path_plots.remove(index);
                }
                None => {
                    model.path_plots.push(PathPlot::hot_spot(plot_name.clone()));
                    dialog.set_plot_key(plot_name.clone());
                }
            }
        }
        let (hot_spot, next) = match action {
            DialogAction::Open => return,
            DialogAction::Cancel => {
                // The plot of a hot spot that was not created goes with it.
                if dialog.edit.is_none() {
                    model.path_plots.retain(|p| !p.is_hot_spot(&plot_name));
                }
                self.hot_spot_dialog = None;
                return;
            }
            DialogAction::Ok { output, next } => (output, next),
        };
        // The plot follows a renamed hot spot.
        for plot in model.path_plots.iter_mut() {
            if plot.is_hot_spot(&plot_name) {
                plot.path = hot_spot.name.clone();
            }
        }
        let definitions = &mut model.hot_spots.definitions;
        let verb = match dialog.edit.filter(|&i| i < definitions.len()) {
            Some(index) => {
                definitions[index] = hot_spot.clone();
                "modified"
            }
            None => {
                definitions.push(hot_spot.clone());
                "created"
            }
        };
        self.output
            .push(format!("Hot spot {} {verb}", hot_spot.name));
        if next {
            dialog.next(&hot_spot);
        } else {
            self.hot_spot_dialog = None;
        }
        self.evaluate_hot_spots();
    }

    /// Finds the paths of the hot spot being edited or selected in the Results tree.
    fn update_hot_spot_preview(&mut self) {
        let current = self.current_result;
        let model = self.results.get(current);
        let wanted = match (&self.hot_spot_dialog, &self.tree.selected) {
            _ if self.tree_view != TreeView::Results => None,
            (Some(dialog), _) => Some(dialog.hot_spot()),
            (None, Some((TreeView::Results, TreeItem::HotSpot(i)))) => {
                model.and_then(|m| m.hot_spots.definitions.get(*i).cloned())
            }
            _ => None,
        };
        let shown = (self.hot_spot_preview.as_ref()).map(|(h, index, _)| (h, *index));
        let plotted = (self.hot_spot_dialog.as_ref()).and_then(HotSpotDialog::plotted_node);
        let plotted_changed = std::mem::replace(&mut self.hot_spot_plotted, plotted) != plotted;
        if wanted.as_ref().map(|h| (h, current)) == shown {
            if plotted_changed {
                self.update_contour();
            }
            return;
        }
        self.hot_spot_preview = match (wanted, model) {
            (Some(hot_spot), Some(model)) => {
                let paths = crate::hot_spots::paths(model, &hot_spot);
                Some((hot_spot, current, paths))
            }
            _ => None,
        };
        self.update_contour();
    }

    /// Hot spot paths drawn over the results, deformed like them: of the hot spot edited or
    /// selected.
    fn overlay_paths(&self) -> Vec<Vec<glam::Vec3>> {
        let model = self.results.get(self.current_result);
        let Some(model) = model.filter(|_| self.tree_view == TreeView::Results) else {
            return Vec::new();
        };
        // The dialog shows the path of one toe node, that of the highest hot spot stress.
        let plotted = (self.hot_spot_dialog.as_ref()).and_then(HotSpotDialog::plotted_node);
        match &self.hot_spot_preview {
            Some((_, index, paths)) if *index == self.current_result => {
                let paths = paths.iter().filter(|p| plotted.is_none_or(|n| p.node == n));
                crate::hot_spots::render_paths(model, paths)
            }
            _ => Vec::new(),
        }
    }

    /// The y axis of an axisymmetric model, from below to above the
    /// shown model, in render coordinates. Results show it when they are flat like the 2D
    /// model they come from.
    fn axis_line(&self, shown: &Model) -> Option<[glam::Vec3; 2]> {
        let space = self.model.as_ref()?.fe.properties.space;
        if space != plx_model::ModelSpace::Axisymmetric {
            return None;
        }
        if shown.is_results() {
            let (min, max) = shown.mesh.bounds()?;
            let flat = (max[2] - min[2]).abs() <= 1e-6 * (max[0] - min[0]).abs().max(1e-30);
            if !flat {
                return None;
            }
        }
        let (min, max) = shown.visible_bounds()?;
        let origin = shown.global_origin();
        let margin = 0.1 * (max.y - min.y).max(max.x - min.x);
        Some([
            glam::Vec3::new(origin.x, min.y - margin, origin.z),
            glam::Vec3::new(origin.x, max.y + margin, origin.z),
        ])
    }

    /// The FE model, which can be set up.
    fn setup_model(&self) -> Option<&Model> {
        self.model.as_ref()
    }

    /// The index of the active step: the one picked in the ribbon or last selected in the
    /// tree, else the last step.
    pub(crate) fn active_step(&self) -> Option<usize> {
        let steps = &self.setup_model()?.fe.steps;
        (self.active_step.as_ref())
            .and_then(|name| steps.iter().position(|s| &s.name == name))
            .or_else(|| steps.len().checked_sub(1))
    }

    /// Makes step `index` the active one.
    pub(crate) fn set_active_step(&mut self, index: usize) {
        let name = (self.setup_model())
            .and_then(|m| m.fe.steps.get(index))
            .map(|s| s.name.clone());
        if name.is_some() {
            self.active_step = name;
        }
    }

    /// The combo box of the active step on the Steps tab of the ribbon.
    pub(crate) fn active_step_combo(&mut self, ui: &mut egui::Ui) {
        let active = self.active_step();
        let steps: Vec<String> = (self.setup_model())
            .map(|m| m.fe.steps.iter().map(|s| s.name.clone()).collect())
            .unwrap_or_default();
        let text = active
            .and_then(|i| steps.get(i))
            .map_or("No step", String::as_str)
            .to_owned();
        let mut picked = None;
        let mut new_step = false;
        ui.vertical(|ui| {
            ui.add_space(6.0);
            ui.weak("New items go into");
            ui.add_enabled_ui(self.setup_model().is_some(), |ui| {
                egui::ComboBox::from_id_salt("active step")
                    .selected_text(text)
                    .width(150.0)
                    .show_ui(ui, |ui| {
                        for (i, name) in steps.iter().enumerate() {
                            if ui.selectable_label(active == Some(i), name).clicked() {
                                picked = Some(i);
                            }
                        }
                        ui.separator();
                        new_step = ui.selectable_label(false, "New Step …").clicked();
                    })
                    .response
                    .on_hover_text(
                        "The active step: new boundary conditions, loads, outputs and \
                         defined fields go into it, and the 3D view shows its symbols.",
                    );
            });
        });
        if let Some(i) = picked {
            self.set_active_step(i);
            // The symbols follow the active step, not a selection in another one.
            if (self.tree.selected.as_ref())
                .is_some_and(|(_, item)| step_of(item).is_some_and(|s| s != i))
            {
                self.tree.selected = None;
            }
        }
        if new_step {
            self.create(NewItem::Step);
        }
    }

    /// The editors of the properties panel that are open.
    fn docked_open(&self) -> Vec<crate::dock::Docked> {
        use crate::dock::Docked;
        let mut open = Vec::new();
        for (is_open, docked) in [
            (self.dialog.is_some(), Docked::Properties),
            (self.editor.is_some(), Docked::Item),
            (self.mesh_item_editor.is_some(), Docked::MeshItem),
            (self.feature_dialog.is_some(), Docked::Feature),
            (self.field_output_dialog.is_some(), Docked::FieldOutput),
            (self.history_dialog.is_some(), Docked::HistoryOutput),
            (self.hot_spot_dialog.is_some(), Docked::HotSpot),
            (self.model_dialog.is_some(), Docked::ModelProperties),
        ] {
            if is_open {
                open.push(docked);
            }
        }
        open
    }

    /// Keeps one editor open in the properties panel, the one opened last; returns whether
    /// one is open.
    fn one_docked(&mut self) -> bool {
        use crate::dock::Docked;
        let open = self.docked_open();
        for closed in crate::dock::one_open(&mut self.docked, &open) {
            match closed {
                Docked::Properties => self.dialog = None,
                Docked::Item => {
                    self.editor = None;
                    self.highlighted = None;
                }
                Docked::MeshItem => self.mesh_item_editor = None,
                Docked::Feature => self.feature_dialog = None,
                Docked::FieldOutput => self.field_output_dialog = None,
                Docked::HistoryOutput => self.history_dialog = None,
                Docked::HotSpot => self.hot_spot_dialog = None,
                Docked::ModelProperties => self.model_dialog = None,
            }
        }
        !self.docked.is_empty()
    }

    /// A newly selected step, or item of a step, in the tree makes that step active.
    fn follow_selected_step(&mut self) {
        let selected = match &self.tree.selected {
            Some((TreeView::FeModel, item)) => Some(item.clone()),
            _ => None,
        };
        if selected == self.step_selection {
            return;
        }
        if let Some(step) = selected.as_ref().and_then(step_of) {
            self.set_active_step(step);
        }
        self.step_selection = selected;
    }

    /// What the 3D view shows: the current results on the Results tab, else the FE model.
    fn shown(&self) -> Option<&Model> {
        match self.tree_view {
            TreeView::Results => self.results.get(self.current_result),
            TreeView::Geometry => self.geometry.as_ref(),
            TreeView::FeModel => self.model.as_ref(),
        }
    }

    fn shown_mut(&mut self) -> Option<&mut Model> {
        match self.tree_view {
            TreeView::Results => self.results.get_mut(self.current_result),
            TreeView::Geometry => self.geometry.as_mut(),
            TreeView::FeModel => self.model.as_mut(),
        }
    }

    fn shown_results_mut(&mut self) -> Option<&mut ResultsView> {
        self.shown_mut()?.results.as_mut()
    }

    /// Whether clicks in the 3D view pick for an open dialog: an item dialog of the FE model
    /// or a history output or hot spot dialog of the results.
    fn picking(&self) -> bool {
        if self.query.is_some() {
            return true;
        }
        match self.tree_view {
            TreeView::Results => {
                (self.history_dialog.as_ref()).is_some_and(HistoryOutputDialog::picks)
                    || (self.hot_spot_dialog.as_ref()).is_some_and(HotSpotDialog::picks)
            }
            TreeView::FeModel => self.editor.as_ref().is_some_and(Editor::picks),
            TreeView::Geometry => {
                (self.mesh_item_editor.as_ref()).is_some_and(MeshItemEditor::picks)
            }
        }
    }

    /// The results shown on the Results tab.
    fn shown_results_view(&self) -> Option<&ResultsView> {
        self.results.get(self.current_result)?.results.as_ref()
    }

    fn close_history_windows(&mut self) {
        self.history_dialog = None;
        self.hot_spot_dialog = None;
        self.history_table = None;
    }

    /// Switches the tree tab. The Results tab is a workspace of its own: the
    /// 3D view swaps between the FE model and the current results, each with its own camera.
    fn set_tree_view(&mut self, view: TreeView) {
        if self.tree_view == view {
            return;
        }
        let was_results = self.tree_view == TreeView::Results;
        if was_results == (view == TreeView::Results) {
            // Geometry and FE model share the camera; the scene changes.
            self.tree_view = view;
            self.results_changed = true;
            return;
        }
        // A fit still pending, e.g. right after loading, belongs to the workspace left.
        if let Some(command) = self.view_command.take() {
            let bounds = self.shown().and_then(Model::visible_bounds);
            self.viewport.apply(command, bounds);
        }
        self.tree_view = view;
        self.dialog = None;
        self.viewport.labels = Default::default();
        match &mut self.parked_camera {
            Some(camera) => self.viewport.swap_camera(camera),
            None => {
                let mut camera = plx_render::Camera::default();
                self.viewport.swap_camera(&mut camera);
                self.parked_camera = Some(camera);
                self.view_command = Some(ViewCommand::Fit);
            }
        }
        self.results_changed = true;
    }

    /// The results row of the tool bar: the controls of the shown result.
    fn results_tool_bar(&mut self, ui: &mut egui::Ui) {
        let enabled = self.tree_view == TreeView::Results && !self.results.is_empty();
        ui.add_enabled_ui(enabled, |ui| self.results_tool_bar_row(ui, enabled));
    }

    fn results_tool_bar_row(&mut self, ui: &mut egui::Ui, enabled: bool) {
        ui.vertical(|ui| {
            // Greyed out, the row shows the current results file, or empty controls.
            let mut placeholder = None;
            let current = self.results.get_mut(self.current_result);
            let has_global = current.as_ref().is_some_and(|m| m.global.is_some());
            let view = match current {
                Some(model) => model.results.as_mut(),
                None => None,
            }
            .unwrap_or_else(|| placeholder.insert(ResultsView::new(Vec::new(), None)));
            let transformed = !view.transformations.is_empty();
            let response = results_tool_bar(ui, view, transformed, has_global);
            if response.changed && enabled {
                self.results_changed = true;
            }
            if response.transformations && enabled {
                self.open_transformation_dialog();
            }
            if response.color_scale && enabled {
                self.open_color_scale();
            }
        });
    }

    fn open_transformation_dialog(&mut self) {
        let index = self.current_result;
        if let Some(model) = self.results.get(index)
            && let Some(view) = &model.results
        {
            let dialog = TransformationDialog::new(index, &view.transformations, model);
            self.transformation_dialog = Some(dialog);
            self.update_contour();
        }
    }

    fn transformation_picks(&self) -> bool {
        (self.transformation_dialog.as_ref())
            .is_some_and(|d| d.picks() && self.tree_view == TreeView::Results)
    }

    fn transformation_window(&mut self, ctx: &egui::Context) {
        if self.tree_view != TreeView::Results {
            self.transformation_dialog = None;
        }
        let Some(dialog) = &mut self.transformation_dialog else {
            return;
        };
        let Some(model) = self.results.get_mut(dialog.result) else {
            self.transformation_dialog = None;
            return;
        };
        let shown = (dialog.lines(model), dialog.points(model));
        let action = dialog.show(ctx, model);
        // The overlay shows the points and axis of the selected item.
        let moved = shown != (dialog.lines(model), dialog.points(model));
        let (transformations, close) = match action {
            TransformationAction::Open => {
                if moved {
                    self.update_contour();
                }
                return;
            }
            TransformationAction::Cancel => {
                self.transformation_dialog = None;
                self.viewport.preview = Default::default();
                self.update_contour();
                return;
            }
            TransformationAction::Apply {
                transformations,
                close,
            } => (transformations, close),
        };
        if let Some(view) = &mut model.results {
            view.transformations = transformations;
            self.results_changed = true;
            // The copies extend the scene; show all of it.
            self.view_command = Some(ViewCommand::Fit);
        }
        if close {
            self.transformation_dialog = None;
            self.viewport.preview = Default::default();
        }
        self.update_contour();
    }

    fn create(&mut self, kind: NewItem) {
        if kind == NewItem::ResultHotSpot {
            if let Some(model) = self.results.get(self.current_result) {
                self.hot_spot_dialog = Some(HotSpotDialog::create(&model.hot_spots.definitions));
                self.set_tree_view(TreeView::Results);
            }
            return;
        }
        if let NewItem::Feature(kind) = kind {
            // Paths read results; features belong to the tab they are created on.
            let on_results = matches!(kind, FeatureKind::ResultPath | FeatureKind::ResultPlane);
            let results =
                (on_results || self.tree_view == TreeView::Results).then_some(self.current_result);
            if let Some(model) = self.feature_model(results) {
                self.feature_dialog = Some(FeatureDialog::create(kind, results, model));
                if results.is_some() {
                    self.set_tree_view(TreeView::Results);
                }
            }
            return;
        }
        if kind == NewItem::ResultHistoryOutput {
            self.refresh_moving_loads();
            if let Some(view) = self.shown_results_view() {
                self.history_dialog = Some(HistoryOutputDialog::create(view));
            }
            return;
        }
        if kind == NewItem::MeshSetupItem {
            if let Some(model) = &self.model
                && let Some(geometry) = &model.geometry
            {
                let units = model.fe.properties.units;
                self.mesh_item_editor = Some(MeshItemEditor::create(geometry, units));
                self.set_tree_view(TreeView::Geometry);
            }
            return;
        }
        if let NewItem::MeshSet(kind) = kind {
            if let Some(model) = self.setup_model() {
                self.editor = Some(Editor::create_set(kind, &model.fe, &model.mesh));
                self.set_tree_view(TreeView::FeModel);
            }
            return;
        }
        if kind == NewItem::ResultFieldOutput {
            if let Some(model) = self.results.get(self.current_result)
                && let Some(view) = &model.results
            {
                self.field_output_dialog = Some(FieldOutputDialog::create(view, &model.mesh));
            }
            return;
        }
        if let Some(model) = self.setup_model() {
            // The solver writes the values of all contacts with one contact field output, so
            // a step that has one opens it.
            let existing = match kind {
                NewItem::ContactFieldOutput(step) => (model.fe.steps.get(step))
                    .and_then(|s| {
                        (s.field_outputs.iter())
                            .position(|f| f.kind == plx_model::OutputKind::Contact)
                    })
                    .map(|i| TreeItem::FieldOutput(step, i)),
                _ => None,
            };
            self.editor = match existing {
                Some(item) => Editor::edit(&item, &model.fe, &model.mesh),
                None => Editor::create(kind, &model.fe),
            };
            self.set_tree_view(TreeView::FeModel);
        }
    }

    fn open_mesh_tool(&mut self, tool: MeshTool) {
        if let Some(model) = self.setup_model() {
            self.mesh_tool = Some(MeshToolDialog::new(tool, model));
            self.editor = None;
            self.set_tree_view(TreeView::FeModel);
        }
    }

    fn mesh_tool_window(&mut self, ctx: &egui::Context) {
        let (Some(dialog), Some(model)) = (&mut self.mesh_tool, &mut self.model) else {
            return;
        };
        match dialog.show(ctx, model) {
            MeshToolResult::Open => {}
            // On an error the dialog shows it and stays open.
            MeshToolResult::Apply => {
                if let Ok(done) = dialog.apply(model) {
                    self.output.push(done);
                    self.mesh_tool = None;
                    self.highlighted = None;
                    // The symbols of loads and supports sit on nodes that moved or changed.
                    self.symbols_shown = None;
                    self.results_changed = true;
                }
            }
            MeshToolResult::Cancel => self.mesh_tool = None,
        }
    }

    fn query_window(&mut self, ctx: &egui::Context) {
        let Some(query) = &mut self.query else {
            return;
        };
        let model = match self.tree_view {
            TreeView::Results => self.results.get(self.current_result),
            TreeView::Geometry => self.geometry.as_ref(),
            TreeView::FeModel => self.model.as_ref(),
        };
        match query.show(ctx, model, &mut self.output) {
            QueryAction::Open => {}
            QueryAction::Changed => self.update_contour(),
            QueryAction::Add(annotation) => {
                if let Some(model) = self.shown_mut() {
                    model.query_annotations.push(annotation);
                }
                self.update_contour();
            }
            QueryAction::RemoveAll => self.annotations(AnnotationCommand::DeleteAll),
            QueryAction::Close => self.close_query(),
        }
    }

    fn open_query(&mut self) {
        self.query = Some(QueryWindow::default());
    }

    /// Closes the Query window; its annotations stay in the view.
    fn close_query(&mut self) {
        self.query = None;
        self.viewport.preview = Default::default();
        self.update_contour();
    }

    fn has_annotations(&self) -> bool {
        self.shown()
            .is_some_and(|model| !model.query_annotations.is_empty())
    }

    /// Deletes query annotations of the shown model, from their context menu or with Remove
    /// Annotations.
    fn annotations(&mut self, command: AnnotationCommand) {
        // The label of an annotation dragged elsewhere keeps its place when one before it
        // goes.
        let labels = &mut self.viewport.labels.annotations;
        match command {
            AnnotationCommand::Delete(index) => {
                let owners = &self.viewport.overlay.annotation_owners;
                if let Some(marker) = owners.iter().position(|&owner| owner == index)
                    && marker < labels.len()
                {
                    labels.remove(marker);
                }
            }
            AnnotationCommand::DeleteAll => labels.clear(),
        }
        let Some(model) = self.shown_mut() else {
            return;
        };
        match command {
            AnnotationCommand::Delete(index) if index < model.query_annotations.len() => {
                model.query_annotations.remove(index);
            }
            AnnotationCommand::Delete(_) => {}
            AnnotationCommand::DeleteAll => model.query_annotations.clear(),
        }
        self.update_contour();
    }

    fn open_mesh_setup(&mut self) {
        if let Some(model) = &self.model
            && let Some(geometry) = &model.geometry
        {
            let units = model.fe.properties.units;
            self.mesh_setup = Some(MeshSetupWindow::new(
                &geometry.meshing,
                units,
                self.geometry.as_ref(),
            ));
        }
    }

    fn mesh_setup_window(&mut self, ctx: &egui::Context) {
        let Some(window) = &mut self.mesh_setup else {
            return;
        };
        let (setup, mesh) = match window.show(ctx) {
            MeshSetupResult::Open => return,
            MeshSetupResult::Cancel => (None, false),
            MeshSetupResult::Ok(setup) => (Some(setup), false),
            MeshSetupResult::Mesh(setup) => (Some(setup), true),
        };
        self.mesh_setup = None;
        if let (Some(setup), Some(geometry)) =
            (setup, self.model.as_mut().and_then(|m| m.geometry.as_mut()))
        {
            geometry.meshing = setup;
        }
        if mesh {
            self.generate_mesh(ctx, None);
        }
    }

    /// The dialog of a mesh setup item; OK stores the item in the geometry.
    fn mesh_item_window(&mut self, ctx: &egui::Context) {
        let (Some(editor), Some(geometry)) = (
            &mut self.mesh_item_editor,
            self.model.as_mut().and_then(|m| m.geometry.as_mut()),
        ) else {
            self.mesh_item_editor = None;
            return;
        };
        let mut remesh = None;
        match editor.show(ctx, geometry, self.geometry.as_ref()) {
            MeshItemResult::Open => return,
            MeshItemResult::Cancel => {}
            result @ (MeshItemResult::Ok(..) | MeshItemResult::OkRemesh(..)) => {
                let (index, item, remeshes) = match result {
                    MeshItemResult::Ok(index, item) => (index, item, false),
                    MeshItemResult::OkRemesh(index, item) => (index, item, true),
                    MeshItemResult::Open | MeshItemResult::Cancel => return,
                };
                if remeshes {
                    // A local mesh size is picked on faces and edges: mesh every part.
                    remesh = Some(item.kind.parts().map(<[String]>::to_vec));
                }
                let index = match index.filter(|&i| i < geometry.mesh_items.len()) {
                    Some(index) => {
                        geometry.mesh_items[index] = item;
                        index
                    }
                    None => {
                        geometry.mesh_items.push(item);
                        geometry.mesh_items.len() - 1
                    }
                };
                self.tree.selected = Some((TreeView::Geometry, TreeItem::MeshItem(index)));
            }
        }
        self.mesh_item_editor = None;
        self.viewport.preview = Default::default();
        if let Some(parts) = remesh {
            self.generate_mesh(ctx, parts);
        }
    }

    /// Meshes the named parts of the geometry, or all with `None`, on a worker thread;
    /// [`Self::poll_meshing`] takes the result.
    fn generate_mesh(&mut self, ctx: &egui::Context, parts: Option<Vec<String>>) {
        if self.meshing.is_some() {
            return;
        }
        let Some(geometry) = self.model.as_ref().and_then(|m| m.geometry.clone()) else {
            return;
        };
        let what = match &parts {
            Some(parts) => parts.join(", "),
            None => geometry.source.clone(),
        };
        self.output.push(format!("Meshing {what} …"));
        self.meshing = Some(MeshingJob::start(geometry, parts, ctx));
        self.mesh_failure.clear();
    }

    /// Cancels the meshing under way: the mesher is killed and the parts keep their mesh.
    fn cancel_meshing(&mut self) {
        if self.meshing.take().is_some() {
            self.output.push(crate::meshing::CANCELLED.into());
        }
    }

    fn poll_meshing(&mut self) {
        let Some(result) = self.meshing.as_ref().and_then(MeshingJob::poll) else {
            return;
        };
        let Some(job) = self.meshing.take() else {
            return;
        };
        let started = Some(job.started);
        let Some(model) = self.model.as_mut() else {
            return;
        };
        match result {
            Ok(generated) => {
                let had_mesh = model.mesh.element_count() > 0;
                match merge_part_meshes(model, &job.geometry, generated, started) {
                    Ok(lines) => self.output.extend(lines),
                    Err(error) => {
                        self.output.push(error);
                        return;
                    }
                }
                self.highlighted = None;
                self.mesh_item_editor = None;
                self.set_tree_view(TreeView::FeModel);
                self.results_changed = true;
                if !had_mesh {
                    self.view_command = Some(ViewCommand::Fit);
                }
            }
            Err(error) => {
                let message = error.to_string();
                self.mesh_failure = plx_mesher::named_entities(&message).into_iter().collect();
                self.output.push(format!("Meshing failed: {message}"));
                // The geometry shows the faces and edges Gmsh names in red.
                if !self.mesh_failure.is_empty() {
                    self.output
                        .push("The affected faces and edges are marked red in the geometry".into());
                    self.tree.selected = None;
                    self.set_tree_view(TreeView::Geometry);
                }
            }
        }
    }

    /// The Material Library Editor, from the context menu of Materials.
    fn open_material_library(&mut self) {
        if let Some(model) = self.setup_model() {
            self.material_library = Some(MaterialLibraryEditor::new(
                &model.fe.materials,
                model.fe.properties.units,
            ));
        }
    }

    fn material_library_window(&mut self, ctx: &egui::Context) {
        let Some(editor) = &mut self.material_library else {
            return;
        };
        match editor.show(ctx) {
            LibraryResult::Open => {}
            LibraryResult::Ok(materials) => {
                self.material_library = None;
                if let Some(materials) = materials
                    && let Some(model) = self.model.as_mut()
                {
                    model.fe.materials = materials;
                    // Material indices may have changed.
                    self.tree.selected = None;
                    self.editor = None;
                }
            }
            LibraryResult::Cancel => self.material_library = None,
        }
    }

    /// Asks before Remove Results, which Undo cannot take back.
    fn confirm_remove_results(&mut self, ctx: &egui::Context) {
        if !self.confirm_remove_results {
            return;
        }
        let question = "Remove the results of the analysis?";
        let consequence = "Saving the project leaves them out; solve again to get them back.";
        match crate::dialog::confirm(
            ctx,
            "confirm remove results",
            question,
            consequence,
            "Remove",
        ) {
            Some(true) => {
                self.confirm_remove_results = false;
                self.remove_results();
            }
            Some(false) => self.confirm_remove_results = false,
            None => {}
        }
    }

    /// Drops the results the analysis keeps; saving the project leaves them out.
    fn remove_results(&mut self) {
        if let Some(model) = &mut self.model
            && model.bound_results.take().is_some()
        {
            model.bound_results_file = None;
            self.output
                .push("Results removed; saving the project leaves them out.".into());
        }
    }

    /// What was defined on the results before those at `path` came: on the open bound
    /// results, on open results at that path, or saved with the bound results.
    fn previous_settings(&self, path: &std::path::Path) -> Option<plx_results::ResultsSettings> {
        let model = self.model.as_ref()?;
        let open = |file: &std::path::Path| self.results.iter().find(|r| r.path == file);
        (model.bound_results_file.as_deref().and_then(open))
            .or_else(|| open(path))
            .and_then(|results| self.view_settings(results))
            .or_else(|| model.bound_results.as_ref()?.settings.clone())
    }

    /// What the user defined on open results, with the camera of the results workspace.
    fn view_settings(&self, results: &Model) -> Option<plx_results::ResultsSettings> {
        let mut settings = results.results_settings()?;
        // The results workspace's camera, in use or parked while another is shown.
        let camera = match self.tree_view {
            TreeView::Results => Some(self.viewport.camera()),
            _ => self.parked_camera.as_ref(),
        };
        settings.view.camera = camera.map(|c| plx_results::saved_view::SavedCamera {
            target: c.target.to_array(),
            distance: c.distance,
            rotation: c.rotation.to_array(),
            scene_radius: c.scene_radius,
        });
        settings.view.mesh_edges = Some(self.viewport.options.mesh_edges);
        Some(settings)
    }

    /// Opens the results bound to the project, written into the work directory first.
    fn open_bound_results(&mut self) {
        if self.bound_work.is_some() {
            return;
        }
        let dir = self.work_dir().join("Kept_Results");
        let Some(model) = &self.model else {
            return;
        };
        let Some(bound) = &model.bound_results else {
            return;
        };
        if let Some(path) = model.bound_results_file.as_ref().filter(|p| p.is_file()) {
            self.open_results = Some(path.clone());
            return;
        }
        let (sender, receiver) = std::sync::mpsc::channel();
        let bound = bound.clone();
        std::thread::spawn(move || {
            let restored =
                plx_io::project::restore_results(&bound, &dir).map_err(|e| e.to_string());
            let _ = sender.send(BoundWork(bound.analysis, dir, restored));
        });
        self.bound_work = Some((model.path.clone(), receiver));
    }

    /// Takes the result of binding or restoring bound results once the worker is done.
    fn poll_bound_results(&mut self, ctx: &egui::Context) {
        let Some((path, receiver)) = &self.bound_work else {
            return;
        };
        let work = match receiver.try_recv() {
            Ok(work) => work,
            Err(std::sync::mpsc::TryRecvError::Empty) => {
                ctx.request_repaint_after(std::time::Duration::from_millis(100));
                return;
            }
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.bound_work = None;
                return;
            }
        };
        // The model may have been closed or replaced meanwhile.
        let model = (self.model.as_mut()).filter(|m| m.path == *path);
        self.bound_work = None;
        let carried = self.carry_to_restored.take();
        match (work, model) {
            (BoundWork(analysis, dir, Ok(file)), Some(model)) => {
                self.output.push(format!(
                    "Results of {analysis} restored to {}",
                    dir.display()
                ));
                model.bound_results_file = Some(file.clone());
                // New results of a solve take over what was defined on those before.
                self.carried_settings = carried.map(|settings| (file.clone(), settings));
                self.open_results = Some(file);
            }
            (BoundWork(_, _, Err(error)), _) => {
                self.output
                    .push(format!("Kept results not restored: {error}"));
            }
            (_, None) => {}
        }
    }

    /// Reads the results of the global model of a submodel, whose edges Show Global
    /// Contours draws around the shown results of the submodel.
    fn load_global_results(&mut self) {
        let Some(fe) = self.model.as_ref().map(|m| &m.fe) else {
            return;
        };
        let Some(path) = (fe.properties.submodel_input()).filter(|_| fe.uses_global_results())
        else {
            return;
        };
        let Some(results) = self.results.get(self.current_result) else {
            return;
        };
        // The global step the first active submodel boundary condition reads.
        let step = (fe.steps.iter().filter(|s| s.active))
            .flat_map(|s| &s.boundary_conditions)
            .find_map(|bc| match bc.kind {
                plx_model::BoundaryKind::Submodel { step, .. } if bc.active => Some(step),
                _ => None,
            })
            .unwrap_or(1);
        if !path.is_file() {
            return;
        }
        let (path, units) = (path.to_path_buf(), self.import_units());
        let (sender, receiver) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let global = model::load(&path, units).and_then(|loaded| {
                model::GlobalModel::new(loaded.model, step)
                    .map(Box::new)
                    .ok_or_else(|| format!("{} holds no results", path.display()))
            });
            let _ = sender.send(global);
        });
        self.global_work = Some((results.path.clone(), receiver));
    }

    /// Keeps the global results of a submodel with its results once they are read.
    fn poll_global_results(&mut self, ctx: &egui::Context) {
        let Some((path, receiver)) = &self.global_work else {
            return;
        };
        let work = match receiver.try_recv() {
            Ok(work) => work,
            Err(std::sync::mpsc::TryRecvError::Empty) => {
                ctx.request_repaint_after(std::time::Duration::from_millis(100));
                return;
            }
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.global_work = None;
                return;
            }
        };
        let path = path.clone();
        self.global_work = None;
        match work {
            Ok(global) => {
                let Some(results) = self.results.iter_mut().find(|m| m.path == path) else {
                    return;
                };
                results.global = Some(global);
                self.results_changed = true;
            }
            Err(error) => self
                .output
                .push(format!("Global contours not available: {error}")),
        }
    }

    /// The folder the analysis runs in: its own in the project's folder of the work
    /// directory, so that the analyses of a project do not overwrite each other's files.
    fn work_dir(&self) -> PathBuf {
        let mut dir = self.settings.solver.work_dir();
        if !self.project_folder.is_empty() {
            dir.push(&self.project_folder);
        }
        dir.join(crate::analysis::job_name(&self.analysis_name))
    }

    /// The state of the cells of the analysis, see [`crate::status`].
    pub fn status(&self) -> Option<crate::status::AnalysisStatus> {
        let model = self.model.as_ref()?;
        Some(crate::status::analysis_status(
            model.geometry.as_ref(),
            &model.mesh,
            &model.fe,
            &model.mesh_inputs,
            model.bound_results.as_ref(),
            &self.upstream,
            (self.project_run.as_ref()).is_some_and(|r| r.active),
        ))
    }

    /// The state of the cells as last found, found again when the analysis changed.
    fn cached_status(&mut self) -> Option<crate::status::AnalysisStatus> {
        let revision = self.revision()?;
        if let Some((seen, status)) = &self.status
            && *seen == revision
        {
            return Some(status.clone());
        }
        let status = self.status()?;
        self.status = Some((revision, status.clone()));
        Some(status)
    }

    /// What tells a change of the analysis: of its FE model, meshing settings and mesh by
    /// [`Model::revision`], of the refinement its geometry is meshed with and of the
    /// results it keeps.
    fn revision(&self) -> Option<u64> {
        let model = self.model.as_ref()?;
        let mut hash = plx_mesh::Fnv::new();
        hash.add(&model.revision.to_le_bytes());
        let refinement = model.geometry.as_ref().map(|g| g.refinement.to_bits());
        hash.add_debug(&refinement);
        let kept = (model.bound_results.as_ref()).map(|b| (b.input, b.stored_size()));
        hash.add_debug(&kept);
        Some(hash.finish())
    }

    /// Ansys Workbench's Update: the project meshes the parts whose mesh is out of date,
    /// then solves if the results are missing or out of date, and keeps the new results.
    /// What the analysis takes from other analyses is made first.
    fn solve(&mut self) {
        self.requests.push(Request::SolveChain(false));
    }

    /// Stops the project's solve of the analysis, or a Check Model run.
    fn stop(&mut self) {
        if (self.project_run.as_ref()).is_some_and(|r| r.active) {
            self.requests.push(Request::StopSolve);
        }
        if let Some(analysis) = (self.analysis.as_mut()).filter(|a| a.is_running()) {
            analysis.kill();
            self.output.push("Model check killed".into());
        }
    }

    /// Whether the project solves the analysis or a Check Model runs.
    fn running(&self) -> bool {
        (self.project_run.as_ref()).is_some_and(|r| r.active)
            || (self.analysis.as_ref()).is_some_and(Analysis::is_running)
    }

    /// Check Model: the solver checks the input file of the analysis without solving it.
    fn check_model(&mut self) {
        if self.running() {
            return;
        }
        if self.setup_model().is_none() {
            return;
        }
        // The run writes its results where those kept last may have been opened from: the
        // kept ones are restored from the project from now on.
        if let Some(model) = &mut self.model {
            model.bound_results_file = None;
        }
        let Some(model) = self.setup_model() else {
            return;
        };
        if model.mesh.element_count() == 0 {
            self.output
                .push("The model has no mesh yet: Geometry > Mesh All Parts".into());
            return;
        }
        let findings = model.findings();
        let work_dir = self.work_dir();
        let name = &self.analysis_name;
        match Analysis::start(
            &self.settings.solver,
            model,
            true,
            name,
            &work_dir,
            &self.incoming.continuation,
        ) {
            Ok(mut analysis) => {
                // Problems the checks found go into the monitor first, so that an abort is
                // explained even before the solver says anything.
                for finding in
                    (findings.iter()).filter(|f| f.severity() == plx_model::Severity::Error)
                {
                    analysis.note(format!(
                        "Model check: {}: {}",
                        finding.problem.title(),
                        finding.detail
                    ));
                }
                self.output
                    .push(format!("Model check started: {}", work_dir.display()));
                self.solver_findings.clear();
                self.analysis = Some(analysis);
                self.project_run = None;
                // The run's output shows below the 3D view, which stays free for results.
                self.analysis_tab = true;
            }
            Err(error) => {
                // A model the analysis cannot be set up with points at the item.
                self.solver_findings = error.finding().into_iter().collect();
                self.output.push(error.message);
            }
        }
    }

    /// Polls the running Check Model.
    fn run_analysis(&mut self, ctx: &egui::Context) {
        if ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::F5)) && !self.running()
        {
            self.solve();
        }
        let Some(analysis) = &mut self.analysis else {
            return;
        };
        if let Some(status) = analysis.poll() {
            self.output
                .push(format!("Analysis {}", Analysis::status_text(status)));
            // The Nodalis solver tells what it found, typed: no reading of its output.
            if let Some(diagnostics) = analysis.diagnostics()
                && let Some(model) = self.model.as_ref()
            {
                self.solver_findings = plx_model::diagnose_solver_check(
                    &diagnostics,
                    &model.mesh,
                    &model.fe,
                    analysis.origins(),
                );
                note_solver_findings(analysis, &self.solver_findings);
            } else if matches!(
                status,
                plx_solve::JobStatus::Failed | plx_solve::JobStatus::FailedWithResults
            ) && let Some(model) = self.model.as_ref()
            {
                self.solver_findings =
                    plx_model::diagnose_solver_output(analysis.output(), &model.mesh);
                for finding in (self.solver_findings.iter())
                    .filter(|f| f.item == plx_model::ModelItem::Analysis)
                {
                    analysis.note(format!(
                        "Possible cause: {}. {}",
                        finding.problem.title(),
                        finding.problem.fix()
                    ));
                }
                if !self.solver_findings.is_empty() {
                    analysis.note(
                        "The warning symbol on the analysis in the tree explains the cause.".into(),
                    );
                    // Errors bring the problems to the front.
                    self.problems_tab = true;
                    self.analysis_tab_before = self.analysis_tab;
                }
            }
            // Warnings that point to wrong results, after any run.
            let warnings = plx_model::diagnose_solver_warnings(analysis.output());
            for finding in &warnings {
                analysis.note(format!(
                    "Warning: {}: {}. {}",
                    finding.problem.title(),
                    finding.detail,
                    finding.problem.fix()
                ));
            }
            self.solver_findings.extend(warnings);
        }
        if analysis.is_running() {
            ctx.request_repaint_after(std::time::Duration::from_millis(200));
        }
    }

    /// Writes the input file of the set-up model to a file the user picks.
    fn export_inp(&mut self) {
        if self.setup_model().is_none() {
            return;
        }
        let Some(model) = self.setup_model() else {
            return;
        };
        let heading = format!("Nodalis: {}", model.file_name());
        let mut fe = model.fe.clone();
        // Checked before asking for the file; the global results are staged once it is known.
        if let Err(error) = plx_io::inp::write_inp(&model.mesh, &fe, &heading) {
            self.output.push(format!("Export not possible: {error}"));
            return;
        }
        let picked = rfd::FileDialog::new()
            .set_title("Export CalculiX Input File")
            .add_filter("Input file (*.inp)", &["inp"])
            .set_file_name(format!(
                "{}.inp",
                crate::analysis::job_name(&self.analysis_name)
            ))
            .save_file();
        if let Some(path) = picked {
            // A submodel's global results go next to the input file, which names them.
            let dir = path.parent().unwrap_or(std::path::Path::new("."));
            let stem = path.file_stem().map(|s| s.to_string_lossy().into_owned());
            if let Err(error) = crate::analysis::stage_global_results(
                &mut fe,
                dir,
                stem.as_deref().unwrap_or("Analysis"),
            ) {
                self.output.push(error);
                return;
            }
            let text = match plx_io::inp::write_inp(&model.mesh, &fe, &heading) {
                Ok(text) => text,
                Err(error) => {
                    self.output.push(format!("Export not possible: {error}"));
                    return;
                }
            };
            if fe.uses_global_results()
                && let Some(global) = fe.properties.submodel_input()
            {
                self.output.push(format!("{} copied", global.display()));
            }
            match std::fs::write(&path, text) {
                Ok(()) => self.output.push(format!("{} written", path.display())),
                Err(error) => self.output.push(format!("{}: {error}", path.display())),
            }
            // The solver reads the result files of defined fields from next to the input file.
            if let Some(dir) = path.parent() {
                let notes = crate::analysis::copy_result_files(&fe, dir);
                self.output.extend(notes);
            }
        }
    }

    /// "Export deformed mesh": the mesh of the current results with the shown
    /// displacements times the deformation scale factor, without materials or steps, so
    /// that another model can import it.
    fn export_deformed_mesh(&mut self) {
        let Some(model) = self.results.get(self.current_result) else {
            return;
        };
        let Some(view) = &model.results else {
            return;
        };
        let Some(displacements) = view.shown_displacements() else {
            self.output
                .push("Export not possible: the results have no displacements".into());
            return;
        };
        let scale = f64::from(view.scale());
        let shown = view
            .current_increment()
            .map(|i| format!(", step {}, increment {}", i.step, i.increment))
            .unwrap_or_default();
        let heading = format!(
            "Nodalis: deformed mesh of {}{shown}, scale factor {scale}",
            model.file_name()
        );
        let text = match plx_io::inp::write_deformed_mesh_inp(
            &model.mesh,
            &displacements,
            scale,
            &heading,
        ) {
            Ok(text) => text,
            Err(error) => {
                self.output.push(format!("Export not possible: {error}"));
                return;
            }
        };
        let stem = model
            .path
            .file_stem()
            .map_or("Results".into(), |s| s.to_string_lossy().into_owned());
        let picked = rfd::FileDialog::new()
            .set_title("Export Deformed Mesh")
            .add_filter("Input file (*.inp)", &["inp"])
            .set_file_name(format!("{stem}_deformed.inp"))
            .save_file();
        if let Some(path) = picked {
            match std::fs::write(&path, text) {
                Ok(()) => self
                    .output
                    .push(format!("Deformed mesh exported to {}", path.display())),
                Err(error) => self.output.push(format!("{}: {error}", path.display())),
            }
        }
    }

    /// File > Export CalculiX Results: the current results as a `.frd` file with the `.dat`
    /// file beside it. Results of the built-in solver are written from memory, those of
    /// CalculiX copied.
    fn export_calculix_results(&mut self) {
        let Some(model) = self.results.get(self.current_result) else {
            return;
        };
        let source = model.path.clone();
        let stem = source
            .file_stem()
            .map_or("Results".into(), |s| s.to_string_lossy().into_owned());
        let picked = rfd::FileDialog::new()
            .set_title("Export CalculiX Results")
            .add_filter("CalculiX results (*.frd)", &["frd"])
            .set_file_name(format!("{stem}.frd"))
            .save_file();
        let Some(path) = picked else {
            return;
        };
        let path = path.with_extension("frd");
        match export_calculix_results(&source, &path) {
            Ok(()) => self
                .output
                .push(format!("Results exported to {}", path.display())),
            Err(error) => self.output.push(format!("Export not possible: {error}")),
        }
    }

    /// Opens the Export VTU or Export VTK dialog of the current results.
    fn open_vtk_export(&mut self, format: plx_io::vtk::VtkFormat) {
        if self.shown_results_view().is_some() {
            self.vtk_export_dialog = Some(crate::vtk_export_dialog::VtkExportDialog::new(
                self.current_result,
                format,
            ));
        }
    }

    /// The Export VTU or Export VTK dialog; Export asks for the file and writes it.
    fn vtk_export_window(&mut self, ctx: &egui::Context) {
        use crate::vtk_export_dialog::DialogResult;
        let Some(mut dialog) = self.vtk_export_dialog.take() else {
            return;
        };
        let Some(model) = self.results.get(dialog.result) else {
            return;
        };
        let Some(view) = &model.results else {
            return;
        };
        match dialog.show(ctx, &view.increments, view.increment) {
            DialogResult::Open => self.vtk_export_dialog = Some(dialog),
            DialogResult::Cancel => {}
            DialogResult::Export => {
                let increments = dialog.scope.select(&view.increments, view.increment);
                let format = dialog.format;
                let several = increments.len() > 1;
                let extension = if several {
                    format.collection_extension()
                } else {
                    format.extension()
                };
                let stem = model
                    .path
                    .file_stem()
                    .map_or("Results".into(), |s| s.to_string_lossy().into_owned());
                let filter = match (several, format) {
                    (false, _) => format!("VTK file (*.{extension})"),
                    (true, plx_io::vtk::VtkFormat::Xml) => "ParaView Data (*.pvd)".into(),
                    (true, plx_io::vtk::VtkFormat::Legacy) => {
                        "VTK file series (*.vtk.series)".into()
                    }
                };
                let filter_extension = extension.rsplit('.').next().unwrap_or(extension);
                let picked = rfd::FileDialog::new()
                    .set_title(dialog.title())
                    .add_filter(filter, &[filter_extension])
                    .set_file_name(format!("{stem}.{extension}"))
                    .save_file();
                let Some(mut path) = picked else {
                    // Back to the dialog to choose again.
                    self.vtk_export_dialog = Some(dialog);
                    return;
                };
                let name = path.file_name().map(|n| n.to_string_lossy().to_lowercase());
                if !name.is_some_and(|n| n.ends_with(&format!(".{extension}"))) {
                    let mut with = path.into_os_string();
                    with.push(format!(".{extension}"));
                    path = with.into();
                }
                let title = format!("Nodalis results of {}", model.file_name());
                let result = plx_io::vtk::export(
                    &path,
                    format,
                    &model.mesh,
                    &increments,
                    dialog.binary,
                    &title,
                );
                match result {
                    Ok(summary) => {
                        let mut line = format!("Results exported to {}", summary.main.display());
                        if summary.files > 1 {
                            line.push_str(&format!(" ({} increments", summary.files));
                            if summary.physical_times == Some(false) {
                                line.push_str(
                                    "; their values do not increase, so the time series is \
                                     numbered 0, 1, 2, ...",
                                );
                            }
                            line.push(')');
                        }
                        self.output.push(line);
                    }
                    Err(error) => self
                        .output
                        .push(format!("Export not possible: {}: {error}", path.display())),
                }
            }
        }
    }

    /// A click in the 3D view picks for the open dialog. Without one, a click on a part
    /// selects it in the tree, and a click into empty space clears the tree
    /// selection and with it the highlighted region.
    fn click(&mut self, click: Click) {
        if let Some(query) = &mut self.query {
            let model = match self.tree_view {
                TreeView::Results => self.results.get(self.current_result),
                TreeView::Geometry => self.geometry.as_ref(),
                TreeView::FeModel => self.model.as_ref(),
            };
            if let Some(model) = model {
                let hit = model.pick_click(&click);
                let precision = hit.as_ref().map_or(0.0, |h| click.precision_at(h.point));
                match query.click(model, hit.as_ref(), precision, &mut self.output) {
                    QueryClick::Nothing => {}
                    QueryClick::Picked => self.update_contour(),
                    QueryClick::Added(annotation) => {
                        if let Some(model) = self.shown_mut() {
                            model.query_annotations.push(annotation);
                        }
                        self.update_contour();
                    }
                }
            }
            return;
        }
        if self.feature_picks()
            && let Some(dialog) = &mut self.feature_dialog
        {
            let model = match dialog.results {
                Some(index) => self.results.get(index),
                None => self.model.as_ref(),
            };
            if let Some(model) = model {
                let hit = model.pick(click.origin, click.direction);
                dialog.click(
                    model,
                    hit.as_ref().map(|h| (h, click.precision_at(h.point))),
                    Operation::from_modifiers(click.shift, click.ctrl),
                );
            }
            return;
        }
        if self.transformation_picks()
            && let (Some(dialog), Some(model)) = (
                &mut self.transformation_dialog,
                self.results.get(self.current_result),
            )
        {
            let hit = model.pick_click(&click);
            dialog.click(
                model,
                hit.as_ref().map(|h| (h, click.precision_at(h.point))),
            );
            return;
        }
        if let Some(dialog) = &mut self.section_dialog
            && dialog.picks()
        {
            let model = match self.tree_view {
                TreeView::Results => self.results.get(self.current_result),
                TreeView::Geometry => self.geometry.as_ref(),
                TreeView::FeModel => self.model.as_ref(),
            };
            if let Some(model) = model {
                let hit = model.pick_click(&click);
                dialog.click(
                    model,
                    hit.as_ref().map(|h| (h, click.precision_at(h.point))),
                );
            }
            return;
        }
        if !self.picking() {
            let hit = self.shown().and_then(|model| model.pick_click(&click));
            let view = self.tree_view;
            match hit {
                // Ctrl adds a part or takes it out again, Shift adds it.
                Some(hit) if click.ctrl => self.tree.toggle_part(view, hit.part),
                Some(hit) if click.shift => self.tree.add_parts(view, [hit.part]),
                Some(hit) => self.select_part(hit.part),
                // A click beside the model with Ctrl or Shift keeps the selection.
                None if click.ctrl || click.shift => {}
                // On the Results tab the tree shows the current field, which stays.
                None if self.tree_view != TreeView::Results => self.tree.selected = None,
                None if matches!(self.tree.selected, Some((_, TreeItem::Part(_)))) => {
                    self.tree.selected = None;
                }
                None => {}
            }
            return;
        }
        if self.tree_view == TreeView::Results {
            if let Some(model) = self.results.get(self.current_result) {
                let hit = model.pick_click(&click);
                let pick = hit.as_ref().map(|hit| (hit, click.precision_at(hit.point)));
                let operation = Operation::from_modifiers(click.shift, click.ctrl);
                if let Some(dialog) = &mut self.hot_spot_dialog {
                    dialog.click(model, pick, operation);
                } else if let Some(dialog) = &mut self.history_dialog {
                    dialog.click(model, pick, operation);
                }
            }
            return;
        }
        if self.tree_view == TreeView::Geometry {
            if let (Some(editor), Some(view)) = (&mut self.mesh_item_editor, &self.geometry) {
                let hit = view.pick_click(&click);
                let pick = hit.as_ref().map(|hit| (hit, click.precision_at(hit.point)));
                editor.click(
                    view,
                    pick,
                    Operation::from_modifiers(click.shift, click.ctrl),
                );
            }
            return;
        }
        let (Some(editor), Some(model)) = (&mut self.editor, &self.model) else {
            return;
        };
        let hit = model.pick_click(&click);
        let pick = hit.as_ref().map(|hit| (hit, click.precision_at(hit.point)));
        editor.click(
            model,
            pick,
            Operation::from_modifiers(click.shift, click.ctrl),
        );
    }

    /// Selects a part clicked in the 3D view in the tree shown. In the FE Model tree the
    /// tree stays where it is and opens what acts on the part instead, which it marks.
    fn select_part(&mut self, index: usize) {
        let view = self.tree_view;
        self.tree.select_part(view, index);
        if view == TreeView::FeModel {
            self.tree.reveal_acting = true;
        } else {
            self.tree.reveal = true;
        }
    }

    /// Finds what acts on the parts selected in the FE Model tree, once for each selection
    /// and change of the model.
    fn update_acting(&mut self) {
        let parts = self.tree.selected_parts(TreeView::FeModel);
        let Some(model) = self.model.as_ref().filter(|_| !parts.is_empty()) else {
            self.tree.acting.clear();
            self.tree.acting_for = None;
            return;
        };
        let key = (parts, model.revision);
        if self.tree.acting_for.as_ref() != Some(&key) {
            self.tree.acting = crate::setup::acting_on_parts(model, &key.0);
            self.tree.acting_for = Some(key);
        }
    }

    /// The context menu of the 3D view: on a part it starts with the part's menu from the
    /// tree, so it makes no difference where the part is right-clicked.
    fn viewport_menu(&mut self, response: &egui::Response, right_click: Option<Click>) {
        if let Some(click) = right_click {
            self.menu_part = None;
            if !self.picking() {
                self.menu_part = (self.shown())
                    .and_then(|model| model.pick_click(&click))
                    .map(|hit| hit.part);
            }
            // A right click on one of several selected parts keeps them, as in the tree.
            if let Some(part) = self.menu_part
                && !self.tree.selected_parts(self.tree_view).contains(&part)
            {
                self.select_part(part);
            }
        }
        let part = (self.menu_part).and_then(|index| {
            let info = self.shown()?.parts.get(index)?;
            Some((index, info.visible, info.contours))
        });
        let mut tree_response = TreeResponse::default();
        let mut command = None;
        let selected = self.tree.selected_parts(self.tree_view);
        response.context_menu(|ui| {
            ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
            if let Some((index, visible, contours)) = part {
                let view = self.tree_view;
                tree::part_menu(
                    ui,
                    index,
                    visible,
                    contours,
                    view,
                    &selected,
                    &mut tree_response,
                );
                ui.separator();
            }
            command = crate::viewport::view_menu(ui);
        });
        if command.is_some() {
            self.view_command = command;
        }
        let view = self.tree_view;
        self.tree_response(&response.ctx, view, tree_response);
    }

    fn box_select(&mut self, area: &BoxSelect) {
        if !self.picking() || self.section_picks() {
            return;
        }
        let operation = Operation::from_modifiers(area.shift, area.ctrl);
        if self.feature_picks()
            && let Some(dialog) = &mut self.feature_dialog
        {
            let model = match dialog.results {
                Some(index) => self.results.get(index),
                None => self.model.as_ref(),
            };
            if let Some(model) = model {
                dialog.box_select(model, area, operation);
            }
            return;
        }
        if self.tree_view == TreeView::Results {
            if let Some(model) = self.results.get(self.current_result) {
                if let Some(dialog) = &mut self.hot_spot_dialog {
                    dialog.box_select(model, area, operation);
                } else if let Some(dialog) = &mut self.history_dialog {
                    dialog.box_select(model, area, operation);
                }
            }
            return;
        }
        if self.tree_view == TreeView::Geometry {
            if let (Some(editor), Some(view)) = (&mut self.mesh_item_editor, &self.geometry) {
                editor.box_select(view, area, operation);
            }
            return;
        }
        if let (Some(editor), Some(model)) = (&mut self.editor, &self.model) {
            editor.box_select(
                model,
                area,
                Operation::from_modifiers(area.shift, area.ctrl),
            );
        }
    }

    /// Shows what a click would select where the mouse rests.
    fn hover(&mut self, hover: Option<Click>) {
        if let Some(query) = &self.query {
            self.viewport.preview = match (hover, self.shown()) {
                (Some(click), Some(model)) => model
                    .pick_click(&click)
                    .map(|hit| query.preview(model, &hit, click.precision_at(hit.point)))
                    .unwrap_or_default(),
                _ => Default::default(),
            };
            return;
        }
        if self.feature_picks()
            && let Some(dialog) = &self.feature_dialog
            && let Some(model) = self.feature_model(dialog.results)
        {
            self.viewport.preview = hover
                .and_then(|click| {
                    let hit = model.pick(click.origin, click.direction)?;
                    Some(dialog.preview(model, &hit, click.precision_at(hit.point)))
                })
                .unwrap_or_default();
            return;
        }
        if self.transformation_picks()
            && let (Some(dialog), Some(model)) = (
                &self.transformation_dialog,
                self.results.get(self.current_result),
            )
        {
            self.viewport.preview = hover
                .and_then(|click| {
                    let hit = model.pick_click(&click)?;
                    Some(dialog.preview(model, &hit, click.precision_at(hit.point)))
                })
                .unwrap_or_default();
            return;
        }
        if let (Some(dialog), Some(model)) = (&self.section_dialog, self.shown())
            && dialog.picks()
        {
            self.viewport.preview = hover
                .and_then(|click| {
                    let hit = model.pick_click(&click)?;
                    Some(dialog.preview(model, &hit, click.precision_at(hit.point)))
                })
                .unwrap_or_default();
            return;
        }
        let hover = hover.filter(|_| self.picking());
        if self.tree_view == TreeView::Results {
            let model = self.results.get(self.current_result);
            self.viewport.preview = match (hover, model) {
                (Some(click), Some(model)) => model
                    .pick_click(&click)
                    .map(|hit| {
                        let precision = click.precision_at(hit.point);
                        match (&self.hot_spot_dialog, &self.history_dialog) {
                            (Some(dialog), _) => dialog.preview(model, &hit, precision),
                            (None, Some(dialog)) => dialog.preview(model, &hit, precision),
                            (None, None) => Default::default(),
                        }
                    })
                    .unwrap_or_default(),
                _ => Default::default(),
            };
            return;
        }
        if self.tree_view == TreeView::Geometry {
            let shown = (self.mesh_item_editor.as_ref()).zip(self.geometry.as_ref());
            self.viewport.preview = match (hover, shown) {
                (Some(click), Some((editor, view))) => view
                    .pick_click(&click)
                    .map(|hit| editor.preview(view, &hit, click.precision_at(hit.point)))
                    .unwrap_or_default(),
                _ => Default::default(),
            };
            return;
        }
        let preview = match (hover, &self.editor, &self.model) {
            (Some(click), Some(editor), Some(model)) => model
                .pick_click(&click)
                .map(|hit| editor.preview(model, &hit, click.precision_at(hit.point)))
                .unwrap_or_default(),
            _ => Default::default(),
        };
        self.viewport.preview = preview;
    }

    /// Opens the dialog of the derived field output that computes the given field.
    fn edit_field_output(&mut self, field: usize) {
        let Some(model) = self.results.get(self.current_result) else {
            return;
        };
        let Some(view) = &model.results else { return };
        if let Some(index) = view.field_output_index(field) {
            self.field_output_dialog = FieldOutputDialog::edit(view, &model.mesh, index);
        }
    }

    fn delete_field_output(&mut self, field: usize) {
        let Some(model) = self.results.get_mut(self.current_result) else {
            return;
        };
        let Some(view) = &mut model.results else {
            return;
        };
        let Some(index) = view.field_output_index(field) else {
            return;
        };
        let name = view.field_outputs[index].name.clone();
        let warnings = view.remove_field_output(index, &model.mesh);
        self.output.push(format!("Field output {name} deleted"));
        for warning in warnings {
            self.output.push(format!("Warning: {warning}"));
        }
        self.tree.selected = None;
        self.field_output_dialog = None;
        self.close_history_windows();
        self.results_changed = true;
    }

    fn field_output_window(&mut self, ctx: &egui::Context) {
        let Some(dialog) = &mut self.field_output_dialog else {
            return;
        };
        let (output, next) = match dialog.show(ctx) {
            DialogAction::Open => return,
            DialogAction::Cancel => {
                self.field_output_dialog = None;
                return;
            }
            DialogAction::Ok { output, next } => (output, next),
        };
        let Some(model) = self.results.get_mut(self.current_result) else {
            self.field_output_dialog = None;
            return;
        };
        let Some(view) = &mut model.results else {
            return;
        };
        let name = output.name.clone();
        let edit = dialog.edit;
        match view.set_field_output(edit, output, &model.mesh) {
            Ok(warnings) => {
                let verb = if edit.is_some() {
                    "modified"
                } else {
                    "created"
                };
                self.output.push(format!("Field output {name} {verb}"));
                for warning in warnings {
                    self.output.push(format!("Warning: {warning}"));
                }
                // Nodalis shows a new field output right away.
                if edit.is_none()
                    && let Some(field) = (view.current_increment())
                        .and_then(|i| i.fields.iter().position(|f| f.name == name))
                {
                    view.field = field;
                    view.component = 0;
                    self.tree.selected = Some((TreeView::Results, TreeItem::Component(field, 0)));
                }
                self.results_changed = true;
                if next {
                    dialog.next(&name);
                } else {
                    self.field_output_dialog = None;
                }
            }
            Err(error) => dialog.error = Some(error),
        }
    }

    fn edit_history_output(&mut self, set: usize) {
        self.refresh_moving_loads();
        let Some(model) = self.results.get(self.current_result) else {
            return;
        };
        let Some(view) = &model.results else { return };
        if let Some(index) = view.history_output_index(set) {
            self.history_dialog = HistoryOutputDialog::edit(view, &model.mesh, index);
        }
    }

    fn delete_history_output(&mut self, set: usize) {
        let Some(model) = self.results.get_mut(self.current_result) else {
            return;
        };
        let Some(view) = &mut model.results else {
            return;
        };
        let Some(index) = view.history_output_index(set) else {
            return;
        };
        let name = view.history_outputs[index].name.clone();
        let warnings = view.remove_history_output(index, &model.mesh);
        self.output.push(format!("History output {name} deleted"));
        for warning in warnings {
            self.output.push(format!("Warning: {warning}"));
        }
        self.tree.selected = None;
        self.close_history_windows();
    }

    fn history_output_window(&mut self, ctx: &egui::Context) {
        let (Some(dialog), Some(model)) = (
            &mut self.history_dialog,
            self.results.get_mut(self.current_result),
        ) else {
            return;
        };
        let (output, next) = match dialog.show(ctx, model) {
            DialogAction::Open => return,
            DialogAction::Cancel => {
                self.history_dialog = None;
                return;
            }
            DialogAction::Ok { output, next } => (output, next),
        };
        let Some(view) = &mut model.results else {
            return;
        };
        let edit = dialog.edit;
        match view.set_history_output(edit, output.clone(), &model.mesh) {
            Ok(warnings) => {
                let verb = if edit.is_some() {
                    "modified"
                } else {
                    "created"
                };
                self.output
                    .push(format!("History output {} {verb}", output.name));
                for warning in warnings {
                    self.output.push(format!("Warning: {warning}"));
                }
                // The table may show another component now.
                self.history_table = None;
                if next {
                    dialog.next(&output, view);
                } else {
                    self.history_dialog = None;
                }
            }
            Err(error) => dialog.error = Some(error),
        }
    }

    fn history_table_window(&mut self, ctx: &egui::Context) {
        if let Some(data) = &self.history_plot {
            let model = match self.tree_view {
                TreeView::Results => self.results.get_mut(self.current_result),
                _ => None,
            };
            let shown = model
                .as_ref()
                .is_some_and(|m| m.path_plots.iter().any(|p| p.is_table(data)));
            let response = crate::xy_plot::window(ctx, data, shown);
            // The annotation belongs to the shown results and stays after the window closes.
            if response.toggle_view
                && let Some(model) = model
            {
                match model.path_plots.iter().position(|p| p.is_table(data)) {
                    Some(index) => {
                        model.path_plots.remove(index);
                    }
                    None => model.path_plots.push(PathPlot::table(data.clone())),
                }
            }
            match response.export {
                Some(crate::xy_plot::Export::Clipboard) => {
                    (self.screenshot).request_region(screenshot::Target::Clipboard, response.plot)
                }
                Some(crate::xy_plot::Export::Png) => {
                    (self.screenshot).request_region(screenshot::Target::File, response.plot)
                }
                Some(crate::xy_plot::Export::Svg) => {
                    let svg =
                        crate::xy_plot::to_svg(data, response.plot.width(), response.plot.height());
                    if let Some(path) = rfd::FileDialog::new()
                        .set_title("Save Plot As")
                        .add_filter("SVG image", &["svg"])
                        .set_file_name("plot.svg")
                        .save_file()
                    {
                        let path = if path.extension().is_none() {
                            path.with_extension("svg")
                        } else {
                            path
                        };
                        self.output.push(match std::fs::write(&path, svg) {
                            Ok(()) => format!("Plot saved: {}", path.display()),
                            Err(err) => {
                                format!("Plot could not be saved ({}): {err}", path.display())
                            }
                        });
                    }
                }
                None => {}
            }
            if response.closed {
                self.history_plot = None;
            }
        }
        let Some(mut table) = self.history_table.take() else {
            return;
        };
        let Some(view) = self.shown_results_view() else {
            return;
        };
        let kind = view
            .current_increment()
            .map_or(plx_results::AnalysisKind::Static, |i| i.kind);
        let unit = (view.history_output_index(table.set))
            .and_then(|i| view.history_outputs.get(i))
            .and_then(|o| o.unit());
        match table.show(ctx, &view.history, kind, unit) {
            crate::history_table::TableAction::Close => return,
            crate::history_table::TableAction::Plot(data) => self.history_plot = Some(data),
            crate::history_table::TableAction::None => {}
        }
        self.history_table = Some(table);
    }

    /// Search Contact Pairs; it replaces an open item dialog.
    fn open_contact_search(&mut self) {
        if let Some(model) = self.setup_model() {
            self.contact_search = Some(ContactSearchDialog::new(&model.fe));
            self.editor = None;
            self.set_tree_view(TreeView::FeModel);
        }
    }

    /// Runs the contact detection `index` on the parts of the model and reports what it
    /// found.
    /// Runs the contact detection `index`: on the CAD geometry when every part comes from
    /// it, see [`crate::connections::detection_display`], else on the mesh.
    fn detect_contacts(&mut self, index: usize) {
        let Some(model) = &mut self.model else {
            return;
        };
        let report = crate::connections::detect(model, self.geometry.as_ref(), index);
        let Some(report) = report else {
            return;
        };
        let name = model.fe.contact_detections[index].name.clone();
        let units = model.fe.properties.units;
        self.output.push(format!(
            "{} (tolerance {})",
            report.summary(&name),
            crate::connections::with_unit(report.tolerance, units, plx_model::Quantity::Length)
        ));
        self.tree.selected = Some((TreeView::FeModel, TreeItem::ContactDetection(index)));
        self.tree.reveal = true;
    }

    /// The table of all contact regions, Ansys' worksheet: rows select the regions in the
    /// tree, and the type, friction, sides and state of all selected ones change at once.
    fn contact_worksheet_window(&mut self, ctx: &egui::Context) {
        if !self.contact_worksheet {
            return;
        }
        let Some(model) = &mut self.model else {
            self.contact_worksheet = false;
            return;
        };
        let mut open = true;
        let results = self.results.get(self.current_result);
        crate::connections::worksheet(ctx, &mut open, model, results, &mut self.tree);
        self.contact_worksheet = open;
    }

    fn contact_search_window(&mut self, ctx: &egui::Context) {
        let (Some(dialog), Some(model)) = (&mut self.contact_search, &mut self.model) else {
            return;
        };
        match dialog.show(ctx, model) {
            SearchResult::Open => {}
            SearchResult::Ok(ties, pairs, joints) => {
                let mut created = format!(
                    "Contact search: {} ties and {} contact pairs created",
                    ties.len(),
                    pairs.len()
                );
                if !joints.is_empty() {
                    created.push_str(&format!(", {} Node Ties", joints.len()));
                }
                // The first new item shows in the tree, even in a collapsed branch.
                let first = if !pairs.is_empty() {
                    Some(TreeItem::ContactPair(model.fe.contact_pairs.len()))
                } else if !ties.is_empty() {
                    Some(TreeItem::Tie(model.fe.ties.len()))
                } else {
                    (!joints.is_empty()).then_some(TreeItem::NodeTie(model.fe.node_ties.len()))
                };
                if let Some(item) = first {
                    self.tree.selected = Some((TreeView::FeModel, item));
                    self.tree.reveal = true;
                }
                model.fe.ties.extend(ties);
                model.fe.contact_pairs.extend(pairs);
                model.fe.node_ties.extend(joints);
                self.output.push(created);
                self.contact_search = None;
                self.highlighted = None;
            }
            SearchResult::Cancel => {
                self.contact_search = None;
                self.highlighted = None;
            }
        }
    }

    /// Deleting, for the context menu and the Delete key alike: in the FE model at once with
    /// a toast offering Undo, elsewhere after a confirmation, since Undo does not reach there.
    fn confirm_delete_window(&mut self, ctx: &egui::Context) {
        let Some((view, items)) = self.confirm_delete.clone() else {
            return;
        };
        // The parts are named in the question.
        let parts: Vec<String> = (items.iter())
            .filter_map(|item| match item {
                TreeItem::Part(index) => (self.tree_model(view))
                    .and_then(|m| m.parts.get(*index))
                    .map(|p| p.name.clone()),
                _ => None,
            })
            .collect();
        let question = match parts.as_slice() {
            [] if items.len() > 1 => format!("Delete {} items?", items.len()),
            [] => "Delete the item?".into(),
            [name] => format!("Delete {name}?"),
            names => format!("Delete {} parts?", names.len()),
        };
        let consequence = match parts.as_slice() {
            [] | [_] => "Undo cannot bring it back here.".to_owned(),
            names => format!("{}. Undo cannot bring them back here.", names.join(", ")),
        };
        // In the FE model Undo brings a deleted item back, so it goes at once and a toast
        // offers Undo. Elsewhere there is no Undo, so the user is asked first.
        let undoable = view == TreeView::FeModel;
        let toast = undoable.then(|| {
            let names: Vec<String> = match parts.as_slice() {
                [] => (items.iter())
                    .filter_map(|item| self.item_name(item))
                    .collect(),
                names => names.to_vec(),
            };
            match names.as_slice() {
                [name] if items.len() == 1 => format!("Deleted {name}"),
                _ => format!("Deleted {} items", items.len()),
            }
        });
        let answer = if undoable {
            Some(true)
        } else {
            crate::dialog::confirm(ctx, "confirm delete", &question, &consequence, "Delete")
        };
        if let (Some(true), Some(text)) = (answer, toast) {
            self.toast = Some(crate::toast::Toast::new(text, true));
        }
        match answer {
            Some(true) => {
                self.confirm_delete = None;
                // The tree may have changed meanwhile, e.g. by another tab's selection.
                let selected = self.tree.selected_parts(view);
                let parts: Vec<usize> = (items.iter())
                    .filter_map(|item| match item {
                        TreeItem::Part(index) => Some(*index),
                        _ => None,
                    })
                    .collect();
                let selected_items = self.tree.selected_items(view);
                if !parts.is_empty() && parts.iter().all(|p| selected.contains(p)) {
                    self.delete_parts(view, &parts);
                } else if items.len() > 1 && items.iter().all(|i| selected_items.contains(i)) {
                    // From the back, so that the indices of the others stay valid.
                    let mut items = items;
                    items.sort();
                    for item in items.into_iter().rev() {
                        self.delete_item(view, item);
                    }
                } else if let [item] = items.as_slice()
                    && self.tree.selected.as_ref() == Some(&(view, item.clone()))
                {
                    self.delete_item(view, item.clone());
                }
            }
            Some(false) => self.confirm_delete = None,
            None => {}
        }
    }

    /// The name of an item of the FE model, as its dialog shows it.
    fn item_name(&self, item: &TreeItem) -> Option<String> {
        let model = self.model.as_ref()?;
        let editor = Editor::edit(item, &model.fe, &model.mesh)?;
        Some(editor.name())
    }

    /// Shows the toast, if there is one, over the bottom of the 3D view.
    fn toast(&mut self, ctx: &egui::Context) {
        let Some(toast) = &mut self.toast else {
            return;
        };
        match crate::toast::show(ctx, toast, self.viewport.rect) {
            crate::toast::Event::None => {}
            crate::toast::Event::Gone => self.toast = None,
            crate::toast::Event::Undo => {
                self.toast = None;
                self.undo();
            }
        }
    }

    /// The model a tree shows.
    fn tree_model(&self, view: TreeView) -> Option<&Model> {
        match view {
            TreeView::Results => self.results.get(self.current_result),
            TreeView::Geometry => self.geometry.as_ref(),
            TreeView::FeModel => self.model.as_ref(),
        }
    }

    fn delete_parts(&mut self, view: TreeView, parts: &[usize]) {
        match view {
            TreeView::Geometry => self.delete_geometry_parts(parts),
            TreeView::FeModel => self.delete_mesh_parts(parts),
            TreeView::Results => {}
        }
    }

    fn delete_item(&mut self, view: TreeView, item: TreeItem) {
        if let TreeItem::Part(index) = item {
            self.delete_parts(view, &[index]);
        } else if let TreeItem::ResultFieldOutput(field) = item {
            self.delete_field_output(field);
        } else if let TreeItem::HistorySet(set) = item {
            self.delete_history_output(set);
        } else if let TreeItem::HotSpot(index) = item {
            self.delete_hot_spot(index);
        } else if let TreeItem::MeshItem(index) = item {
            if let Some(geometry) = self.model.as_mut().and_then(|m| m.geometry.as_mut())
                && index < geometry.mesh_items.len()
            {
                geometry.mesh_items.remove(index);
                self.tree.selected = None;
                self.mesh_item_editor = None;
            }
        } else if let Some(feature) = tree::feature(&item) {
            let model = match view {
                TreeView::Results => self.results.get_mut(self.current_result),
                _ => self.model.as_mut(),
            };
            if let Some(model) = model
                && crate::features::delete(&mut model.fe, feature)
            {
                self.tree.selected = None;
                self.feature_dialog = None;
                if let Some(view) = &mut model.results
                    && feature.kind == FeatureKind::ResultPlane
                {
                    view.plane_result = match view.plane_result {
                        Some(i) if i == feature.index => None,
                        Some(i) if i > feature.index => Some(i - 1),
                        other => other,
                    };
                }
            }
        } else if let Some(kind) = tree::set_kind(&item) {
            if let (Some(model), Some(name)) = (self.model.as_mut(), tree::set_name(&item)) {
                model
                    .fe
                    .mesh_sets
                    .retain(|s| s.kind != kind || s.name != name);
                kind.remove(&mut model.mesh, name);
                self.tree.selected = None;
                self.editor = None;
                self.highlighted = None;
            }
        } else if let Some(model) = self.model.as_mut()
            && crate::setup::delete(&mut model.fe, &item)
        {
            self.tree.selected = None;
            self.editor = None;
        }
    }

    /// The import into a model with geometry or a mesh: the new parts join the
    /// geometry. The parts already there keep their names, and the selections on the
    /// geometry and the mesh follow Gmsh's new numbering of the faces, edges and vertices.
    fn geometry_added(
        &mut self,
        paths: &[PathBuf],
        result: Result<Box<plx_mesher::CadAddition>, String>,
    ) {
        let files: Vec<String> = (paths.iter())
            .map(|p| {
                p.file_name().map_or_else(
                    || p.display().to_string(),
                    |n| n.to_string_lossy().into_owned(),
                )
            })
            .collect();
        let files = files.join(", ");
        let addition = match result {
            Ok(addition) => *addition,
            Err(error) => {
                self.output
                    .push(format!("Error importing {files}: {error}"));
                return;
            }
        };
        let Some(model) = self.model.as_mut() else {
            return;
        };
        let plx_mesher::CadAddition {
            import,
            renumbered,
            added,
        } = addition;
        let view = Model::geometry_view(&model.path, import.display);
        match geometry_check(model.fe.properties.space, &view) {
            Ok(faces) if !faces.is_empty() => {
                let faces: Vec<String> = faces.iter().map(i32::to_string).collect();
                self.output.push(format!(
                    "Note: the normal of face {} points in -z; the elements are flipped \
                     when meshing.",
                    faces.join(", ")
                ));
            }
            Ok(_) => {}
            Err(error) => {
                self.output
                    .push(format!("Error importing {files}: {error}"));
                return;
            }
        }
        if model.geometry.is_some() {
            model.fe.renumber_cad(&renumbered);
            if model.has_cad() {
                let mut mesh = model.mesh.clone();
                mesh.cad = mesh.cad.renumbered(&renumbered);
                model.set_mesh(mesh);
            }
        }
        model.geometry = Some(import.geometry);
        for warning in &import.warnings {
            self.output.push(format!("Warning: {warning}"));
        }
        for note in &import.notes {
            self.output.push(format!("Note: {note}"));
        }
        self.output.push(format!(
            "{files} imported: {} part(s) added ({})",
            added.len(),
            added.join(", ")
        ));
        self.geometry = Some(view);
        self.after_parts_changed();
        self.set_tree_view(TreeView::Geometry);
        self.view_command = Some(ViewCommand::Fit);
    }

    /// The import of a mesh into a model: the mesh of each input file is added as
    /// parts. The parts already there keep their numbers and names; a part, set or surface
    /// of the new mesh whose name is taken gets a number appended.
    fn meshes_added(&mut self, files: Vec<(PathBuf, Result<Box<plx_io::inp::InpImport>, String>)>) {
        let Some(model) = self.model.as_mut() else {
            return;
        };
        let mut any = false;
        for (path, result) in files {
            let name = path.file_name().map_or_else(
                || path.display().to_string(),
                |n| n.to_string_lossy().into_owned(),
            );
            let mut import = match result {
                Ok(import) => *import,
                Err(error) => {
                    self.output.push(format!("Error importing {name}: {error}"));
                    continue;
                }
            };
            if import.mesh.element_count() == 0 {
                self.output.push(format!(
                    "Error importing {name}: it contains no elements that can be displayed"
                ));
                continue;
            }
            // The numbers of the input file are in the analysis' units; the mesh is stored in
            // SI units.
            let length = (model.fe.properties.units)
                .factor_to(plx_model::Quantity::Length, plx_model::STORED);
            if length != 1.0 {
                import.mesh.scale(length);
            }
            // The model space of an analysis is set when it is made: plane stress, plane
            // strain and axisymmetric elements take the type of the analysis' 2D space.
            let space = plx_model::ModelSpace::of_mesh(&import.mesh);
            if space.is_2d() && model.fe.properties.space.is_2d() {
                model.fe.properties.space.convert_mesh(&mut import.mesh);
            } else if space.is_2d() != model.fe.properties.space.is_2d() {
                self.output.push(format!(
                    "Error importing {name}: its elements are {}, the model is {}",
                    space.label(),
                    model.fe.properties.space.label()
                ));
                continue;
            }
            let (nodes, elements) = (import.mesh.node_count(), import.mesh.element_count());
            let mut added = Vec::new();
            model.edit_mesh(|mesh, _| added = mesh.append(import.mesh));
            self.output.push(format!(
                "{name} imported: {nodes} nodes, {elements} elements, {} part(s) added ({})",
                added.len(),
                added.join(", ")
            ));
            for warning in &import.warnings {
                self.output.push(format!("Warning: {warning}"));
            }
            // The heading is not part of a mesh, so skipping it is no news.
            let keywords: Vec<String> = (import.skipped_keywords.keys())
                .filter(|k| *k != "HEADING")
                .map(|k| format!("*{k}"))
                .collect();
            if !keywords.is_empty() {
                self.output.push(format!(
                    "Only the mesh is read; skipped: {}",
                    keywords.join(", ")
                ));
            }
            any = true;
        }
        if any {
            self.after_parts_changed();
            self.set_tree_view(TreeView::FeModel);
            self.view_command = Some(ViewCommand::Fit);
        }
    }

    /// Deleting geometry parts: the geometry loses the solids or faces; a mesh
    /// already generated from them stays, as parts of the FE model.
    fn delete_geometry_parts(&mut self, indices: &[usize]) {
        let names: Vec<String> = (self.geometry.as_ref())
            .map(|g| {
                (indices.iter())
                    .filter_map(|&i| g.parts.get(i))
                    .map(|p| p.name.clone())
                    .collect()
            })
            .unwrap_or_default();
        let Some(model) = self.model.as_mut() else {
            return;
        };
        let Some(geometry) = &model.geometry else {
            return;
        };
        if names.is_empty() {
            return;
        }
        let name = names.join(", ");
        // All in one go, so that Gmsh numbers the faces, edges and vertices anew once.
        let (smaller, tags) = match plx_mesher::delete_parts_renumbered(geometry, &names) {
            Ok((smaller, tags)) => {
                // Gmsh numbers the faces, edges and vertices anew; the mesh and the
                // selections on the geometry follow.
                if model.has_cad() {
                    let mut mesh = model.mesh.clone();
                    mesh.cad = mesh.cad.renumbered(&tags);
                    model.fe.renumber_cad(&tags);
                    model.set_mesh(mesh);
                }
                (smaller, tags)
            }
            Err(error) => {
                self.output
                    .push(format!("{name} cannot be deleted: {error}"));
                return;
            }
        };
        // The display keeps the triangles of the parts that stay; tessellating the
        // geometry anew takes long for a large assembly.
        let view = self.geometry.take();
        self.geometry = smaller.as_ref().and_then(|geometry| {
            let kept = view.and_then(|v| v.geometry_view_without(&names, geometry, &tags));
            if kept.is_some() {
                return kept;
            }
            match plx_mesher::tessellate(geometry) {
                Ok(display) => Some(Model::geometry_view(&model.path, display)),
                Err(error) => {
                    self.output
                        .push(format!("Geometry cannot be displayed: {error}"));
                    None
                }
            }
        });
        model.geometry = smaller;
        self.output.push(deleted_message(&names));
        self.after_parts_changed();
    }

    /// Deleting mesh parts: their elements go, with the nodes no other part has.
    fn delete_mesh_parts(&mut self, indices: &[usize]) {
        let Some(model) = self.model.as_mut() else {
            return;
        };
        let names: Vec<String> = (indices.iter())
            .filter_map(|&i| model.parts.get(i))
            .map(|p| p.name.clone())
            .collect();
        if names.is_empty() {
            return;
        }
        // All in one go: each pass builds the whole mesh anew.
        let mesh = plx_mesher::delete_mesh_parts(&model.mesh, &names);
        model.set_mesh(mesh);
        self.output.push(deleted_message(&names));
        self.after_parts_changed();
    }

    /// Merging mesh parts: the selected parts become one, named after the
    /// first; the model's regions on the others follow.
    fn merge_mesh_parts(&mut self, parts: &[usize]) {
        let Some(model) = self.model.as_mut() else {
            return;
        };
        let mut merged = None;
        model.edit_mesh(|mesh, fe| {
            merged = mesh.merge_parts(parts);
            if let Some((kept, gone)) = &merged {
                for name in gone {
                    fe.rename_part(name, kept);
                }
            }
        });
        let Some((kept, gone)) = merged else {
            return;
        };
        self.output
            .push(format!("Parts {} merged into {kept}", gone.join(", ")));
        self.after_parts_changed();
    }

    /// The Edit menu.
    /// The exports of the File menu.
    fn export_menu(&mut self, ui: &mut egui::Ui) {
        let setup = self.setup_model().is_some();
        let export = egui::Button::new("CalculiX Input File …");
        if ui.add_enabled(setup, export).clicked() {
            self.export_inp();
        }
        let deformed = self.shown_results_view().is_some();
        let export = egui::Button::new("Deformed Mesh (.inp) …");
        let tip = "Writes the mesh of the current results, deformed by the current \
                   deformation scale factor, as an input file with only the mesh.";
        if ui
            .add_enabled(deformed, export)
            .on_hover_text(tip)
            .on_disabled_hover_text("No results are open.")
            .clicked()
        {
            self.export_deformed_mesh();
        }
        let export = egui::Button::new("CalculiX Results (.frd) …");
        let tip = "Writes the current results as CalculiX's result files: the .frd \
                   file and, with history output, the .dat file beside it, for \
                   CalculiX GraphiX, PrePoMax and other programs that read them.";
        if ui
            .add_enabled(deformed, export)
            .on_hover_text(tip)
            .on_disabled_hover_text("No results are open.")
            .clicked()
        {
            self.export_calculix_results();
        }
        let tip = "Writes the current results for ParaView and other VTK programs, \
                   one increment or a time series of several.";
        for (label, format) in [
            ("VTU …", plx_io::vtk::VtkFormat::Xml),
            ("VTK …", plx_io::vtk::VtkFormat::Legacy),
        ] {
            if ui
                .add_enabled(deformed, egui::Button::new(label))
                .on_hover_text(tip)
                .on_disabled_hover_text("No results are open.")
                .clicked()
            {
                self.open_vtk_export(format);
            }
        }
    }

    fn edit_menu(&mut self, ui: &mut egui::Ui) {
        let blocked = self.undo_blocked();
        let history = self.model.as_ref().map(|m| &m.history);
        let can_undo = blocked.is_none() && history.is_some_and(|h| h.can_undo());
        let can_redo = blocked.is_none() && history.is_some_and(|h| h.can_redo());
        let reason = blocked.unwrap_or("Nothing to undo.");
        let undo = egui::Button::new("Undo").shortcut_text("Ctrl+Z");
        if ui
            .add_enabled(can_undo, undo)
            .on_disabled_hover_text(reason)
            .clicked()
        {
            self.undo();
        }
        let reason = blocked.unwrap_or("Nothing to redo.");
        let redo = egui::Button::new("Redo").shortcut_text("Ctrl+Shift+Z");
        if ui
            .add_enabled(can_redo, redo)
            .on_disabled_hover_text(reason)
            .clicked()
        {
            self.redo();
        }
    }

    /// Why the changes of the FE model cannot be undone or redone right now, if so: an open
    /// dialog may refer to what an undo takes away.
    fn undo_blocked(&self) -> Option<&'static str> {
        if self.tree_view == TreeView::Results {
            return Some("Undo and Redo work on the Geometry and FE Model tabs.");
        }
        if self.meshing.is_some() {
            return Some("Wait until meshing has finished.");
        }
        let dialog_open = self.dialog.is_some()
            || self.editor.is_some()
            || self.model_dialog.is_some()
            || self.mesh_setup.is_some()
            || self.mesh_item_editor.is_some()
            || self.confirm_delete.is_some()
            || self.contact_search.is_some()
            || self.mesh_tool.is_some()
            || self.material_library.is_some()
            || self.feature_dialog.is_some();
        dialog_open.then_some("Close the open dialog first.")
    }

    /// Edit > Undo: reverts the last change of the FE model. The tree selection and what is
    /// drawn may refer to items it takes away, so they are built anew.
    fn undo(&mut self) {
        if self.undo_blocked().is_some() {
            return;
        }
        if let Some(model) = &mut self.model
            && crate::undo::undo(model)
        {
            self.output.push("Undo".into());
            self.after_parts_changed();
        }
    }

    /// Edit > Redo: makes the last undone change of the FE model again.
    fn redo(&mut self) {
        if self.undo_blocked().is_some() {
            return;
        }
        if let Some(model) = &mut self.model
            && crate::undo::redo(model)
        {
            self.output.push("Redo".into());
            self.after_parts_changed();
        }
    }

    fn after_parts_changed(&mut self) {
        self.tree.selected = None;
        self.menu_part = None;
        self.dialog = None;
        self.mesh_item_editor = None;
        self.highlighted = None;
        self.symbols_shown = None;
        self.frame_cache.clear();
        self.results_changed = true;
    }

    fn editor_window(&mut self, ctx: &egui::Context) {
        let (Some(editor), Some(model)) = (&mut self.editor, &mut self.model) else {
            return;
        };
        editor.set_linked(crate::links::linked_parameters(&self.incoming.paths));
        // A contact detection searches the geometry, if there is one to search; its derived
        // tolerance is worked out again only when the parts searched change.
        let scope = (editor.detection_parts()).and_then(|parts| {
            let display = crate::connections::detection_display(model, self.geometry.as_ref())?;
            let current = editor.detection_scope();
            Some(match current {
                Some(scope) if scope.auto.0 == parts => scope.clone(),
                _ => crate::connections::DetectionScope::new(model, display, parts),
            })
        });
        editor.set_detection_scope(scope);
        match editor.show(ctx, model) {
            EditorResult::Open => {}
            EditorResult::Ok => {
                if let Some(editor) = self.editor.take() {
                    let created = editor.new_interaction_item(&model.fe);
                    // A new step becomes the active one.
                    if let Some(step) = editor.new_step() {
                        self.active_step = Some(step);
                    }
                    let detection = editor.detection(&model.fe);
                    let sets = model.fe.mesh_sets.clone();
                    if let Some((kind, name)) = editor.replaced_set() {
                        kind.remove(&mut model.mesh, name);
                    }
                    editor.apply(&mut model.fe);
                    if model.fe.mesh_sets != sets {
                        model.fe.sync_mesh_sets(&sets, &mut model.mesh);
                    }
                    // The new item shows in the tree, even in a collapsed branch.
                    if let Some(item) = created {
                        self.tree.selected = Some((TreeView::FeModel, item));
                        self.tree.reveal = true;
                    }
                    if let Some(index) = detection {
                        self.detect_contacts(index);
                    }
                }
                self.highlighted = None;
            }
            EditorResult::Cancel => {
                self.editor = None;
                self.highlighted = None;
            }
        }
    }

    /// The motion preview of a boundary condition dialog moves the nodes of its region in
    /// the 3D view; without one they are back in place.
    fn update_motion_preview(&mut self, ctx: &egui::Context) {
        let Some(model) = &mut self.model else {
            return;
        };
        let now = ctx.input(|i| i.time);
        let motion = (self.editor.as_mut()).and_then(|e| e.motion_offsets(now, model));
        if motion.is_some() {
            ctx.request_repaint_after(std::time::Duration::from_secs_f64(1.0 / 30.0));
        }
        if model.motion != motion {
            model.motion = motion;
            self.results_changed |= self.tree_view == TreeView::FeModel;
        }
    }

    /// A cyclic symmetry tie dialog draws the other sectors of its part in the 3D view.
    fn update_cyclic_preview(&mut self) {
        let Some(model) = &mut self.model else {
            return;
        };
        let preview = (self.editor.as_mut()).and_then(|e| e.cyclic_preview(model));
        if model.cyclic_preview != preview {
            model.cyclic_preview = preview;
            self.results_changed |= self.tree_view == TreeView::FeModel;
        }
    }

    /// Highlights the region of the open dialog, or of the item selected in the tree; a
    /// selected part is outlined in the FE model and in the results alike.
    fn update_highlight(&mut self) {
        let on_results = self.tree_view == TreeView::Results;
        // The current results show the region of an open history output dialog, else the
        // part selected in the Results tree.
        let current = self.current_result;
        for (index, model) in self.results.iter_mut().enumerate() {
            let shown = on_results && index == current;
            let dialog = (self.history_dialog.as_ref()).filter(|_| shown);
            let hot_spot = (self.hot_spot_dialog.as_ref()).filter(|_| shown);
            let transformation = (self.transformation_dialog.as_ref())
                .filter(|d| on_results && d.result == index && index == current);
            let feature = (self.feature_dialog.as_ref())
                .filter(|d| on_results && d.results == Some(index) && index == current);
            let highlight = match (dialog, &self.tree.selected) {
                _ if feature.is_some() => feature.map(|d| d.highlight(model)).unwrap_or_default(),
                _ if transformation.is_some() => transformation
                    .map(TransformationDialog::highlight)
                    .unwrap_or_default(),
                _ if hot_spot.is_some() => hot_spot.map(|d| d.highlight(model)).unwrap_or_default(),
                (Some(dialog), _) => dialog.highlight(model),
                (None, Some((TreeView::Results, TreeItem::Part(_)))) if index == current => {
                    Highlight::parts(self.tree.selected_parts(TreeView::Results))
                }
                _ => Highlight::default(),
            };
            if highlight != model.highlight {
                model.highlight = highlight;
                self.results_changed |= on_results && index == current;
            }
        }
        self.update_geometry_highlight();
        let Some(model) = &mut self.model else {
            return;
        };
        let feature = self.feature_dialog.as_ref().filter(|d| d.results.is_none());
        if (self.section_dialog.is_some() || feature.is_some())
            && let Some(dialog) = &mut self.contact_search
        {
            // Shows its pairs again once the other dialog has closed.
            dialog.forget_highlight();
        }
        let highlight = if let Some(dialog) = &self.section_dialog {
            self.highlighted = None;
            dialog.highlight()
        } else if let Some(dialog) = feature {
            self.highlighted = None;
            dialog.highlight(model)
        } else if let Some(dialog) = &mut self.contact_search {
            self.highlighted = None;
            // Built anew only when the selected pairs change, as for the tree selection.
            match dialog.changed_highlight(model) {
                Some(highlight) => highlight,
                None => return,
            }
        } else if let Some(editor) = &self.editor {
            self.highlighted = None;
            editor.highlight(model)
        } else {
            let parts = self.tree.selected_parts(TreeView::FeModel);
            let items = self.tree.selected_items(TreeView::FeModel);
            let selection = (self.tree.selected.clone(), parts.clone(), items.clone());
            if self.highlighted.as_ref() == Some(&selection) {
                return;
            }
            self.highlighted = Some(selection);
            match &self.tree.selected {
                Some((TreeView::FeModel, TreeItem::Part(_))) => Highlight::parts(parts),
                // Several selected items, such as contact pairs, show together.
                Some((TreeView::FeModel, _)) => {
                    let mut highlight = Highlight::default();
                    for item in &items {
                        highlight.extend(crate::setup::item_highlight(model, item));
                    }
                    highlight
                }
                _ => Default::default(),
            }
        };
        if highlight != model.highlight {
            if self.tree_view == TreeView::FeModel {
                match model.highlight_changed_parts(&model.highlight, &highlight) {
                    Some(parts) => self.highlight_changed.get_or_insert_default().extend(parts),
                    None => self.results_changed = true,
                }
            } else {
                self.results_changed |= !on_results;
            }
            model.highlight = highlight;
        }
    }

    /// The geometry shows the item of an open mesh setup dialog, else the selected part or
    /// mesh setup item.
    fn update_geometry_highlight(&mut self) {
        let Some(view) = &mut self.geometry else {
            return;
        };
        let items = (self.model.as_ref())
            .and_then(|m| m.geometry.as_ref())
            .map_or(&[][..], |g| g.mesh_items.as_slice());
        let highlight = match (&self.mesh_item_editor, &self.tree.selected) {
            (Some(editor), _) => editor.highlight(view),
            (None, Some((TreeView::Geometry, TreeItem::Part(_)))) => {
                Highlight::parts(self.tree.selected_parts(TreeView::Geometry))
            }
            (None, Some((TreeView::Geometry, TreeItem::MeshItem(index)))) => items
                .get(*index)
                .map(|item| crate::meshing::item_highlight(view, &item.kind))
                .unwrap_or_default(),
            // Without a selection, the faces and edges meshing failed on.
            _ => crate::meshing::entities_highlight(view, &self.mesh_failure),
        };
        if highlight != view.highlight {
            if self.tree_view == TreeView::Geometry {
                match view.highlight_changed_parts(&view.highlight, &highlight) {
                    Some(parts) => self.highlight_changed.get_or_insert_default().extend(parts),
                    None => self.results_changed = true,
                }
            }
            view.highlight = highlight;
        }
    }

    /// Draws the symbols of the boundary conditions and loads of one step in the FE model,
    /// for the step chosen to show: the step of the edited or selected
    /// item, else the last step. The selected or edited item is drawn in red.
    fn update_symbols(&mut self, ctx: &egui::Context) {
        // Symbols rest while the exploded view is edited or moves.
        let exploding = self.exploded_dialog.is_some()
            || (self.model.as_ref()).is_some_and(|m| m.explosion.is_animating());
        let items = if exploding {
            Vec::new()
        } else {
            self.symbol_items()
        };
        let shown = (self.model.as_ref())
            .map(|m| m.parts.iter().map(|p| p.visible).collect())
            .unwrap_or_default();
        let version = (self.model.as_ref()).map_or(0, |m| m.explosion.version());
        let key = (items, shown, version);
        if self.symbols_shown.as_ref() == Some(&key) {
            return;
        }
        self.viewport.symbols = (self.model.as_ref())
            .filter(|_| !key.0.is_empty())
            .map(|model| symbols::build(model, &key.0))
            .unwrap_or_default();
        self.symbols_shown = Some(key);
        ctx.request_repaint();
    }

    fn symbol_items(&self) -> Vec<symbols::Item> {
        let Some(model) = self
            .model
            .as_ref()
            .filter(|_| self.tree_view == TreeView::FeModel)
        else {
            return Vec::new();
        };
        let edited = self.editor.as_ref().and_then(Editor::step_item);
        let selected = match &self.tree.selected {
            Some((TreeView::FeModel, item)) => Some(item),
            _ => None,
        };
        // Constraints belong to no step: they are drawn with the symbols of any step.
        let mut items = Vec::new();
        let edited_constraint = self.editor.as_ref().and_then(|e| e.editing_constraint());
        for (i, constraint) in model.fe.constraints.iter().enumerate() {
            if !constraint.active() || edited_constraint == Some(Some(i)) {
                continue;
            }
            // A hidden constraint is still drawn while selected.
            let is_selected =
                edited_constraint.is_none() && selected == Some(&TreeItem::Constraint(i));
            if !constraint.visible() && !is_selected {
                continue;
            }
            if let Some((kind, region)) = symbols::Kind::of_constraint(constraint) {
                items.push(symbols::Item {
                    kind,
                    region,
                    selected: is_selected,
                });
            }
        }
        items.extend(self.editor.as_ref().and_then(Editor::constraint_item));
        let index = (edited.as_ref().map(|(s, ..)| *s))
            .or_else(|| selected.and_then(step_of))
            .or_else(|| self.active_step());
        let Some((index, step)) = index.and_then(|i| Some((i, model.fe.steps.get(i)?))) else {
            return items;
        };
        // The edited item replaces its saved version.
        let replaced = |load: bool, i: usize| {
            edited.as_ref().is_some_and(|(_, edited_index, item)| {
                *edited_index == Some(i) && matches!(item.kind, symbols::Kind::Load(_)) == load
            })
        };
        let is_selected = |item: TreeItem| edited.is_none() && selected == Some(&item);
        // Deactivated items have no symbols, hidden ones only while selected;
        // the edited one is drawn anyway.
        for (i, bc) in step.boundary_conditions.iter().enumerate() {
            let selected = is_selected(TreeItem::BoundaryCondition(index, i));
            if step.active && bc.active && (bc.visible || selected) && !replaced(false, i) {
                items.push(symbols::Item {
                    kind: symbols::Kind::Boundary(bc.kind.clone()),
                    region: bc.region.clone(),
                    selected,
                });
            }
        }
        for (i, load) in step.loads.iter().enumerate() {
            let selected = is_selected(TreeItem::Load(index, i));
            if step.active && load.active && (load.visible || selected) && !replaced(true, i) {
                items.push(symbols::Item {
                    kind: symbols::Kind::Load(load.kind.clone()),
                    region: load.region.clone(),
                    selected,
                });
            }
        }
        items.extend(edited.map(|(_, _, item)| item));
        items
    }

    fn shown_model(&self) -> ShownModel {
        ShownModel(self.tree_view, self.current_result)
    }

    fn model_mut(&mut self, which: ShownModel) -> Option<&mut Model> {
        match which.0 {
            TreeView::Results => self.results.get_mut(which.1),
            TreeView::Geometry => self.geometry.as_mut(),
            TreeView::FeModel => self.model.as_mut(),
        }
    }

    /// Whether the shown model is exploded.
    fn exploded_applied(&self) -> bool {
        self.shown().is_some_and(|m| m.explosion.applied.is_some())
    }

    /// The toolbar button: explodes the shown model by the last used parameters at
    /// half the scale, or puts it back together, animated either way.
    fn toggle_exploded(&mut self) {
        if self.exploded_dialog.is_some() {
            return;
        }
        let last = self.last_exploded.clone();
        let Some(model) = self.shown_mut() else {
            return;
        };
        let (from, to, parameters) = match model.explosion.applied.take() {
            Some(parameters) => (parameters.scale(), 0.0, parameters),
            None if model.parts.len() > 1 => {
                let parameters = crate::exploded::Parameters {
                    scale_factor: 0.5,
                    ..last
                };
                model.explosion.applied = Some(parameters.clone());
                (0.0, parameters.scale(), parameters)
            }
            None => return,
        };
        animate_explosion(model, &parameters, from, to);
        if model.explosion.applied.is_some() {
            self.last_exploded = parameters;
        }
        self.viewport.preview = Default::default();
    }

    /// Opens the exploded view dialog on the shown model and previews it at once.
    fn open_exploded_dialog(&mut self) {
        if self.exploded_dialog.is_some() {
            return;
        }
        let which = self.shown_model();
        let last = self.last_exploded.clone();
        let Some(model) = self.model_mut(which) else {
            return;
        };
        let center = model.mesh.bounds().map_or(glam::DVec3::ZERO, |(a, b)| {
            (glam::DVec3::from(a) + glam::DVec3::from(b)) * 0.5
        });
        let units = (model.stored_units(), model.units());
        let mut dialog = ExplodedDialog::new(model.explosion.applied.clone(), &last, center, units);
        preview_explosion(model, &mut dialog, true);
        self.exploded_dialog = Some((dialog, which));
    }

    fn exploded_window(&mut self, ctx: &egui::Context) {
        let Some((mut dialog, which)) = self.exploded_dialog.take() else {
            return;
        };
        // Switching to another model ends the dialog as if it was cancelled.
        let (result, change) = if which == self.shown_model() {
            dialog.show(ctx)
        } else {
            (ExplodedResult::Cancel, None)
        };
        let Some(model) = self.model_mut(which) else {
            return;
        };
        // Offsets shown without an animation move nothing in the next tick, so rebuild here.
        let mut moved = false;
        if let Some(change) = change {
            let animate = change == crate::exploded::dialog::Change::Parameter;
            preview_explosion(model, &mut dialog, animate);
            moved = !animate;
        }
        match result {
            ExplodedResult::Open => {
                self.exploded_dialog = Some((dialog, which));
                self.results_changed |= moved;
                return;
            }
            ExplodedResult::Ok => {
                let parameters = dialog.draft.clone();
                let offsets = model.explosion_offsets(&parameters);
                let nothing = offsets.iter().all(|o| *o == glam::DVec3::ZERO);
                // A scale factor of zero shows the model assembled, which is no exploded view.
                model.explosion.applied = (!nothing).then(|| parameters.clone());
                if !model.explosion.is_animating() {
                    model.explosion.show(offsets, false);
                    moved = true;
                }
                self.last_exploded = parameters;
            }
            ExplodedResult::Cancel => {
                match &dialog.before {
                    Some(before) => {
                        let offsets = model.explosion_offsets(before);
                        model.explosion.show(offsets, true);
                    }
                    None => model.explosion.show(Vec::new(), true),
                }
                model.explosion.applied = dialog.before.clone();
            }
            ExplodedResult::Disable => {
                model.explosion.show(Vec::new(), true);
                model.explosion.applied = None;
            }
        }
        self.results_changed |= moved;
        self.viewport.preview = Default::default();
    }

    /// Advances the exploded view animations; the shown model is rebuilt when it moved.
    fn update_explosion(&mut self, ctx: &egui::Context) {
        let now = std::time::Instant::now();
        let shown = self.shown_model();
        let mut animating = false;
        let mut moved = false;
        let models = (self.model.iter_mut().map(|m| (TreeView::FeModel, 0, m)))
            .chain(self.geometry.iter_mut().map(|m| (TreeView::Geometry, 0, m)))
            .chain((self.results.iter_mut().enumerate()).map(|(i, m)| (TreeView::Results, i, m)));
        for (view, index, model) in models {
            let changed = model.explosion.tick(now);
            animating |= model.explosion.is_animating();
            let is_shown = view == shown.0 && (view != TreeView::Results || index == shown.1);
            moved |= changed && is_shown;
        }
        if moved {
            self.results_changed = true;
            self.viewport.preview = Default::default();
        }
        if animating {
            ctx.request_repaint();
        }
    }

    fn section_picks(&self) -> bool {
        self.section_dialog
            .as_ref()
            .is_some_and(SectionDialog::picks)
    }

    /// Opens the section view dialog on the current section, or on a new one facing away
    /// from the viewer.
    fn open_section_dialog(&mut self) {
        if self.section_dialog.is_some() {
            return;
        }
        if let Some(model) = self.shown() {
            let forward = self.viewport.camera().forward().as_dvec3();
            let dialog = SectionDialog::new(self.section.clone(), model, forward);
            self.section_dialog = Some(dialog);
        }
    }

    fn section_window(&mut self, ctx: &egui::Context) {
        let Some(mut dialog) = self.section_dialog.take() else {
            return;
        };
        let Some(model) = self.shown() else {
            return;
        };
        match dialog.show(ctx, model) {
            SectionResult::Open => {
                self.section_dialog = Some(dialog);
                return;
            }
            SectionResult::Ok => self.section = Some(dialog.draft),
            SectionResult::Disable => self.section = None,
        }
        self.viewport.preview = Default::default();
    }

    /// The model whose features a dialog edits: a results file, or the FE model.
    fn feature_model(&self, results: Option<usize>) -> Option<&Model> {
        match results {
            Some(index) => self.results.get(index),
            None => self.model.as_ref(),
        }
    }

    fn feature_picks(&self) -> bool {
        (self.feature_dialog.as_ref()).is_some_and(|d| d.picks() && self.feature_dialog_shown())
    }

    /// Whether the open feature dialog belongs to what the 3D view shows.
    fn feature_dialog_shown(&self) -> bool {
        match self.feature_dialog.as_ref().map(|d| d.results) {
            Some(Some(index)) => {
                self.tree_view == TreeView::Results && index == self.current_result
            }
            Some(None) => self.tree_view == TreeView::FeModel,
            None => false,
        }
    }

    fn feature_window(&mut self, ctx: &egui::Context) {
        if self.feature_dialog.is_some() && !self.feature_dialog_shown() {
            self.feature_dialog = None;
            self.viewport.preview = Default::default();
        }
        let Some(mut dialog) = self.feature_dialog.take() else {
            return;
        };
        let model = match dialog.results {
            Some(index) => self.results.get_mut(index),
            None => self.model.as_mut(),
        };
        let Some(model) = model else {
            return;
        };
        let cut = self.plane_values.as_ref();
        let result = dialog.show(ctx, model, cut);
        let plot_name = dialog.plot_name(&model.fe);
        if dialog.take_plot_toggle()
            && let Some(name) = plot_name.clone()
        {
            match model.path_plots.iter().position(|p| p.is_path(&name)) {
                Some(index) => {
                    model.path_plots.remove(index);
                }
                None => {
                    model.path_plots.push(PathPlot::new(name.clone()));
                    dialog.set_plot_key(name);
                }
            }
        }
        match result {
            FeatureResult::Open => self.feature_dialog = Some(dialog),
            FeatureResult::Ok => {
                // The plot follows a renamed path.
                if let (Some(old), Some(path)) = (plot_name, dialog.path_draft()) {
                    for plot in model.path_plots.iter_mut().filter(|p| p.is_path(&old)) {
                        plot.path = path.name.clone();
                    }
                }
                dialog.apply(&mut model.fe, &model.mesh);
                self.viewport.preview = Default::default();
            }
            FeatureResult::Cancel => {
                // The plot of a path that was not created goes with it.
                if let Some(old) = plot_name.filter(|_| dialog.creates()) {
                    model.path_plots.retain(|p| !p.is_path(&old));
                }
                self.viewport.preview = Default::default();
            }
        }
    }

    /// Results take the coordinate systems of their features, for transformed field outputs.
    fn sync_result_features(&mut self) {
        for model in &mut self.results {
            let Some(view) = &mut model.results else {
                continue;
            };
            if view.coordinate_systems == model.fe.coordinate_systems {
                continue;
            }
            let systems = model.fe.coordinate_systems.clone();
            let warnings = view.set_coordinate_systems(systems, &model.mesh);
            self.output.extend(warnings);
            self.results_changed = true;
        }
    }

    /// The plots of result paths in the 3D view, for the shown increment. Plots of deleted
    /// paths are dropped; that of the path in the open dialog follows its edits.
    fn update_path_plots(&mut self) {
        let dialog = self
            .feature_dialog
            .as_ref()
            .filter(|_| self.feature_dialog_shown());
        let Some(model) = (match self.tree_view {
            TreeView::Results => self.results.get_mut(self.current_result),
            _ => None,
        }) else {
            self.viewport.path_plots.clear();
            return;
        };
        let edited = dialog.and_then(|d| Some((d.plot_name(&model.fe)?, d.path_draft()?.clone())));
        let path = |fe: &plx_model::FeModel, name: &str| match &edited {
            Some((edited, draft)) if edited == name => Some(draft.clone()),
            _ => fe.result_paths.iter().find(|p| p.name == name).cloned(),
        };
        let edited_hot_spot = (self.hot_spot_dialog.as_ref())
            .map(|d| (d.plot_name(&model.hot_spots.definitions), d.hot_spot()));
        let hot_spot =
            |definitions: &[plx_results::hot_spot::HotSpot], name: &str| match &edited_hot_spot {
                Some((edited, draft)) if edited == name => Some(draft.clone()),
                _ => definitions.iter().find(|h| h.name == name).cloned(),
            };
        let mut plots = std::mem::take(&mut model.path_plots);
        plots.retain(|plot| match plot.kind {
            path_plot::PlotKind::Path => path(&model.fe, &plot.path).is_some(),
            path_plot::PlotKind::HotSpot => {
                hot_spot(&model.hot_spots.definitions, &plot.path).is_some()
            }
            path_plot::PlotKind::Table => true,
        });
        let boxes = (plots.iter_mut().enumerate())
            .filter_map(|(index, plot)| match plot.kind {
                path_plot::PlotKind::Path => {
                    let path = path(&model.fe, &plot.path)?;
                    plot.plot_box(index, model, &path)
                }
                path_plot::PlotKind::HotSpot => {
                    let hot_spot = hot_spot(&model.hot_spots.definitions, &plot.path)?;
                    plot.hot_spot_box(index, model, &hot_spot)
                }
                path_plot::PlotKind::Table => plot.table_box(index),
            })
            .collect();
        model.path_plots = plots;
        self.viewport.path_plots = boxes;
    }

    /// Applies what the user did to the path plots in the 3D view.
    fn path_plot_events(&mut self, events: Vec<PlotEvent>) {
        let Some(model) = (match self.tree_view {
            TreeView::Results => self.results.get_mut(self.current_result),
            _ => None,
        }) else {
            return;
        };
        for event in events {
            match event {
                PlotEvent::Changed { index, placement } => {
                    if let Some(plot) = model.path_plots.get_mut(index) {
                        plot.placement = placement;
                    }
                }
                PlotEvent::Edit(index) => {
                    let extremes = (self.viewport.path_plots.iter())
                        .find(|b| b.index == index)
                        .and_then(path_plot::PlotBox::extremes);
                    if let (Some(plot), Some(extremes)) = (model.path_plots.get(index), extremes) {
                        self.plot_editor = Some(path_plot::EditDialog::new(
                            self.current_result,
                            index,
                            &plot.placement,
                            extremes,
                            plot.kind == path_plot::PlotKind::Table,
                        ));
                    }
                }
                PlotEvent::Delete(index) => {
                    if index < model.path_plots.len() {
                        model.path_plots.remove(index);
                    }
                }
            }
        }
    }

    /// Opens the Color Scale dialog of the shown results.
    fn open_color_scale(&mut self) {
        if self.tree_view != TreeView::Results {
            return;
        }
        let Some(view) = (self.results.get(self.current_result)).and_then(|m| m.results.as_ref())
        else {
            return;
        };
        let extremes = view.legend().map(|l| (l.min, l.max));
        let dialog = crate::color_scale_dialog::ColorScaleDialog::new(
            self.current_result,
            &view.color_scale,
            extremes,
        );
        self.color_scale_dialog = Some((dialog, view.color_scale.clone()));
    }

    /// The Color Scale dialog; its changes show at once.
    fn color_scale_window(&mut self, ctx: &egui::Context) {
        use crate::color_scale_dialog::DialogResult;
        let Some((mut dialog, applied)) = self.color_scale_dialog.take() else {
            return;
        };
        if self.tree_view != TreeView::Results || dialog.result != self.current_result {
            return;
        }
        let Some(view) = (self.results.get_mut(dialog.result)).and_then(|m| m.results.as_mut())
        else {
            return;
        };
        // The number of colours can also change in the tool bar while the dialog is open.
        if view.color_scale.levels != applied.levels {
            dialog.draft.levels = view.color_scale.levels;
        }
        let result = dialog.show(ctx);
        let scale = match result {
            DialogResult::Open | DialogResult::Ok => dialog.draft.clone(),
            DialogResult::Cancel => dialog.original().clone(),
        };
        if view.color_scale != scale {
            view.color_scale = scale.clone();
            self.results_changed = true;
        }
        if let DialogResult::Open = result {
            self.color_scale_dialog = Some((dialog, scale));
        }
    }

    /// The Edit Annotation dialog of a path plot; its changes show at once.
    fn plot_editor_window(&mut self, ctx: &egui::Context) {
        let Some(mut dialog) = self.plot_editor.take() else {
            return;
        };
        if self.tree_view != TreeView::Results || dialog.result != self.current_result {
            return;
        }
        let Some(plot) = (self.results.get_mut(dialog.result))
            .and_then(|model| model.path_plots.get_mut(dialog.index))
        else {
            return;
        };
        // Moving and resizing with the mouse goes on while the dialog is open.
        dialog.draft.offset = plot.placement.offset;
        dialog.draft.size = plot.placement.size;
        dialog.draft.height = plot.placement.height;
        match dialog.show(ctx) {
            path_plot::EditResult::Open => {
                plot.placement = dialog.draft.clone();
                self.plot_editor = Some(dialog);
            }
            path_plot::EditResult::Ok => plot.placement = dialog.draft.clone(),
            path_plot::EditResult::Cancel => plot.placement = dialog.original().clone(),
        }
    }

    /// Reference points, coordinate systems and the result path drawn over the shown model.
    fn update_feature_overlay(&mut self) {
        let Some(model) = self.shown() else {
            self.viewport.overlay.features.clear();
            self.viewport.overlay.result_path = None;
            return;
        };
        let dialog = self
            .feature_dialog
            .as_ref()
            .filter(|_| self.feature_dialog_shown());
        let selected = match &self.tree.selected {
            Some((view, item)) if *view == self.tree_view => tree::feature(item),
            _ => None,
        };
        let edited = dialog.and_then(FeatureDialog::item);
        let mut features = crate::features::marks(model, selected, edited);
        features.extend(dialog.and_then(|d| d.mark(model)));
        let result_path = match dialog {
            Some(dialog) if dialog.kind() == FeatureKind::ResultPath => dialog.path_line(model),
            _ => selected
                .filter(|s| s.kind == FeatureKind::ResultPath)
                .and_then(|s| model.fe.result_paths.get(s.index))
                .and_then(|p| crate::features::path_line(model, p)),
        };
        self.viewport.overlay.features = features;
        self.viewport.overlay.result_path = result_path;
    }

    /// The plane whose cut the shown results show alone: that of a plane result being
    /// edited, or of the checked one; a point on it and its unit normal.
    fn shown_plane_cut(&self) -> Option<(glam::DVec3, glam::DVec3)> {
        if self.tree_view != TreeView::Results {
            return None;
        }
        let model = self.results.get(self.current_result)?;
        let view = model.results.as_ref()?;
        let edited = (self.feature_dialog.as_ref())
            .filter(|d| d.results == Some(self.current_result))
            .and_then(FeatureDialog::result_plane);
        let name = match edited {
            Some(name) => name,
            None => &model.fe.result_planes.get(view.plane_result?)?.plane,
        };
        let (point, normal) = model.fe.plane(name)?.resolve(&model.fe).ok()?;
        Some((point.into(), normal.into()))
    }

    /// Cuts the scene at the section plane being edited or shown, or at the plane of shown
    /// plane results, when it or the scene changed.
    fn update_section(&mut self) {
        // Planes of features follow them.
        if let Some(fe) = self.shown().map(|m| m.fe.clone()) {
            if let Some(dialog) = &mut self.section_dialog {
                dialog.draft.resolve(&fe);
            }
            if let Some(section) = &mut self.section {
                section.resolve(&fe);
            }
        }
        // The section view rests while the exploded view is edited or moves.
        let exploding = self.exploded_dialog.is_some()
            || self.shown().is_some_and(|m| m.explosion.is_animating());
        // Plane results show the cut alone, in place of the section view.
        let plane = self.shown_plane_cut().map(|(point, normal)| {
            let definition = PlaneDefinition::PointNormal { point, normal };
            let cut = SectionView {
                definition,
                flipped: false,
                lighten: false,
            };
            (cut, true)
        });
        let section = (self.section_dialog.as_ref())
            .map(|d| &d.draft)
            .or(self.section.as_ref())
            .map(|s| (s.clone(), false));
        let visible = (self.shown())
            .map(|m| m.parts.iter().map(|p| p.visible).collect())
            .unwrap_or_default();
        let wanted = plane
            .or(section)
            .filter(|_| !exploding)
            .map(|(cut, alone)| (self.viewport.scene_version(), visible, cut, alone));
        if wanted == self.section_shown {
            return;
        }
        // Only the model in view is cut; picking in the others sees everything.
        let models = self.model.iter_mut().chain(&mut self.geometry);
        for model in models.chain(&mut self.results) {
            model.clip = None;
        }
        let shown = match self.tree_view {
            TreeView::Results => self.results.get_mut(self.current_result),
            TreeView::Geometry => self.geometry.as_mut(),
            TreeView::FeModel => self.model.as_mut(),
        };
        let (mut values, mut plane_values) = (None, None);
        match (shown, &wanted) {
            (Some(model), Some((_, _, section, alone))) => {
                // The model origin is the centre of its bounds, where a principal plane's
                // manipulator sits.
                let anchor = section.anchor(model.origin());
                let normal = section.normal();
                let clip = plx_render::clip_plane(anchor, normal, model.origin());
                let faces = model.section_meshes(anchor, normal, section.lighten);
                model.clip = Some(clip);
                self.viewport.set_section(Some((clip, &faces)));
                let alone = *alone && model.results.is_some();
                self.viewport.set_sections_only(alone);
                if alone {
                    values = model.section_values(anchor, normal, true);
                    plane_values = model.section_values(anchor, normal, false);
                }
            }
            _ => {
                self.viewport.set_section(None);
                self.viewport.set_sections_only(false);
            }
        }
        // The legend of the shown results takes the range on the plane; the others their own.
        let shown_range = values
            .as_ref()
            .map(|v| v.extremes.map(|[min, max]| (min.1, max.1)));
        let on_results = self.tree_view == TreeView::Results;
        for (index, model) in self.results.iter_mut().enumerate() {
            let range = match shown_range {
                Some(range) if on_results && index == self.current_result => range,
                _ => None,
            };
            if let Some(view) = &mut model.results
                && view.section_range != range
            {
                view.section_range = range;
                self.results_changed = true;
            }
        }
        self.section_values = values;
        self.plane_values = plane_values;
        self.section_shown = wanted;
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
        if new != self.settings {
            self.settings = new;
            self.update_contour();
        }
    }

    /// Properties dialog of the double-clicked tree item.
    fn properties_window(&mut self, ctx: &egui::Context) {
        let Some(item) = self.dialog.clone() else {
            return;
        };
        let mut open = true;
        let mut close = false;
        let mut accept = false;
        // Parts of the geometry keep their names, which the mesh setup refers to.
        let part = match item {
            TreeItem::Part(index) if self.tree_view != TreeView::Geometry => Some(index),
            _ => None,
        };
        // The model by its fields, so the window can edit the typed name alongside.
        let shown = match self.tree_view {
            TreeView::Results => self.results.get(self.current_result),
            TreeView::Geometry => self.geometry.as_ref(),
            TreeView::FeModel => self.model.as_ref(),
        };
        let title = properties::title(shown, &item);
        let builder = egui::Window::new(title.clone())
            .id(crate::ids::window(ctx, "properties window"))
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center());
        crate::dock::show(ctx, builder, title, &mut open, |ui| {
            let (name, error) = &mut self.part_name;
            let name = part.map(|_| name);
            // Element types are chosen for the FE model, not for results.
            let types = (part.is_some() && self.tree_view == TreeView::FeModel)
                .then_some(())
                .zip(shown)
                .map(|((), model)| properties::ElementTypes {
                    types: &mut self.part_types,
                    space: model.fe.properties.space,
                });
            properties::show(ui, shown, &item, name, types);
            if let Some(error) = error.as_ref().filter(|_| part.is_some()) {
                ui.add_space(4.0);
                ui.colored_label(crate::style::error(), error);
            }
            ui.add_space(8.0);
            crate::dock::footer(ui, |ui| {
                if part.is_some() {
                    close = ui.button("Cancel").clicked();
                    accept = ui.button("OK").clicked();
                } else {
                    close = ui.button("Close").clicked();
                }
            });
        });
        accept |= part.is_some() && ctx.input(|i| i.key_pressed(egui::Key::Enter));
        if let (true, Some(index)) = (accept, part) {
            let name = self.part_name.0.clone();
            let types = std::mem::take(&mut self.part_types);
            let fe_model = self.tree_view == TreeView::FeModel;
            let mut retyped = Vec::new();
            let renamed = match self.shown_mut() {
                Some(model) => model.rename_part(index, &name).and_then(|()| {
                    let current = (model.parts.get(index))
                        .map(|p| p.element_types.clone())
                        .unwrap_or_default();
                    for ((old, _), new) in current.iter().zip(&types) {
                        if fe_model && !old.eq_ignore_ascii_case(new) {
                            model.set_element_type(index, new)?;
                            retyped.push(format!("{old} to {new}"));
                        }
                    }
                    Ok(())
                }),
                None => Ok(()),
            };
            self.part_types = types;
            if !retyped.is_empty() {
                self.output.push(format!(
                    "{name}: element type changed from {}",
                    retyped.join(", ")
                ));
                self.results_changed = true;
                self.highlighted = None;
            }
            match renamed {
                Ok(()) => close = true,
                Err(error) => self.part_name.1 = Some(error),
            }
        }
        if !open || close || ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.dialog = None;
        }
    }

    /// The unit system of the analysis, in which files without units are read and results
    /// shown.
    fn import_units(&self) -> UnitSystem {
        (self.model.as_ref()).map_or(self.settings.new_model.units, |m| m.fe.properties.units)
    }

    fn edit_model_properties(&mut self) {
        if let Some(model) = &self.model {
            self.model_dialog = Some(ModelPropertiesDialog::edit(model.fe.properties.clone()));
        }
    }

    fn model_dialog_window(&mut self, ctx: &egui::Context) {
        let Some(dialog) = &mut self.model_dialog else {
            return;
        };
        let mesh = self.model.as_ref().map(|m| &m.mesh);
        let result = dialog.show(ctx, mesh, self.geometry.as_ref());
        match result {
            DialogResult::Open => return,
            DialogResult::Cancel => {}
            DialogResult::Ok(properties) => {
                // The model space and the unit system stay as the analysis was made.
                if let Some(model) = &mut self.model {
                    model.fe.properties = plx_model::ModelProperties {
                        space: model.fe.properties.space,
                        units: model.fe.properties.units,
                        ..properties
                    };
                    self.results_changed = true;
                }
            }
        }
        self.model_dialog = None;
    }

    /// Plays the animation and shows its window; marks the scene for rebuilding.
    fn animate(&mut self, ctx: &egui::Context) {
        if let Some((file, encoder)) = &self.animation_encoding {
            match encoder.poll() {
                Some(result) => {
                    self.output
                        .push(result.unwrap_or_else(|err| format!("{file}: {err}")));
                    self.animation_encoding = None;
                }
                // Looked at again soon: nothing else may draw a frame meanwhile.
                None => ctx.request_repaint_after(std::time::Duration::from_millis(100)),
            }
        }
        let model = match self.tree_view {
            TreeView::Results => self.results.get_mut(self.current_result),
            TreeView::Geometry => self.geometry.as_mut(),
            TreeView::FeModel => self.model.as_mut(),
        };
        let Some(view) = model
            .and_then(|m| m.results.as_mut())
            .filter(|view| view.animation.is_some())
        else {
            if self.recording.take().is_some() {
                self.output.push("Animation export cancelled".into());
            }
            return;
        };
        if let Some(recording) = self.recording.take() {
            let ((frame, playing), file) = (recording.restore, recording.file.clone());
            let (recording, step) = recording.update(ctx, self.viewport.rect);
            self.recording = recording;
            let shown = match step {
                animation_export::Step::Wait => return,
                animation_export::Step::Show(frame) => frame,
                animation_export::Step::Finished(encoder) => {
                    self.animation_encoding = Some((file, encoder));
                    frame
                }
                animation_export::Step::Cancelled => {
                    self.output.push("Animation export cancelled".into());
                    frame
                }
            };
            if let Some(animation) = &mut view.animation {
                animation.go_to(shown);
                // Back to the state from before once the recording ended.
                animation.playing = self.recording.is_none() && playing;
            }
            view.show_animation_frame();
            self.frame_changed = true;
            return;
        }
        let Some(animation) = &mut view.animation else {
            return;
        };
        if animation.playing {
            let dt = ctx.input(|i| i.stable_dt).min(0.1);
            if animation.tick(dt) {
                view.show_animation_frame();
                self.frame_changed = true;
            }
            ctx.request_repaint();
        }
        match animation_window(ctx, view, self.animation_encoding.is_some()) {
            WindowEvent::None => {}
            WindowEvent::Export(format) => {
                let ffmpeg = animation_export::ffmpeg_executable(&self.settings.post.ffmpeg);
                if let Some(recording) =
                    start_animation_export(view, format, ffmpeg, &mut self.output)
                {
                    let first = recording.first_frame();
                    self.recording = Some(recording);
                    if let Some(animation) = &mut view.animation {
                        animation.go_to(first);
                    }
                    view.show_animation_frame();
                    self.frame_changed = true;
                }
            }
            WindowEvent::Frame => {
                view.show_animation_frame();
                self.frame_changed = true;
            }
            WindowEvent::Settings => self.results_changed = true,
            WindowEvent::Close => {
                view.stop_animation();
                self.results_changed = true;
            }
        }
    }

    /// Shows the sound window of the shown results and drives its audio output.
    fn play_sound(&mut self, ctx: &egui::Context) {
        let model = match self.tree_view {
            TreeView::Results => self.results.get_mut(self.current_result),
            _ => self.model.as_mut(),
        };
        let Some(view) = model.and_then(|m| m.results.as_mut()) else {
            self.audio = None;
            return;
        };
        let shown = view.increment;
        let Some(sound) = &mut view.sound else {
            // Closing the window releases the audio device.
            self.audio = None;
            if view.superposition.take().is_some() {
                self.results_changed = true;
            }
            return;
        };
        let playing = self.audio.as_ref().is_some_and(|a| a.synth().sounding());
        let actions = sound::window(ctx, sound, shown, playing);
        if actions.play {
            if self.audio.is_none() {
                match sound::Player::open() {
                    Ok(player) => self.audio = Some(player),
                    Err(error) => sound.message = Some(error),
                }
            }
            if let Some(audio) = &self.audio {
                audio.synth().start(sound.tones());
                sound.message = None;
            }
        }
        if let Some(audio) = &self.audio {
            if actions.stop {
                audio.synth().stop();
            } else if actions.changed && playing {
                audio.synth().update(sound.tones());
            }
        }
        if actions.export {
            let picked = rfd::FileDialog::new()
                .set_title("Save Sound")
                .add_filter("WAV file (*.wav)", &["wav"])
                .set_file_name(format!("Modes-Step-{}.wav", sound.step))
                .save_file();
            if let Some(mut path) = picked {
                if path.extension().is_none() {
                    path.set_extension("wav");
                }
                let data = sound::wav(sound.tones(), sound.export_seconds());
                match std::fs::write(&path, data) {
                    Ok(()) => self
                        .output
                        .push(timestamped(format!("{} saved", path.display()))),
                    Err(error) => sound.message = Some(format!("Not saved: {error}")),
                }
            }
        }
        if playing || actions.play {
            // The play button turns back when a struck sound has died away.
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }
        let now = ctx.input(|i| i.time);
        if actions.play {
            sound.started = Some(now);
            sound.shape_clock = (0.0, f64::NEG_INFINITY);
        }
        if actions.play || actions.changed {
            sound.mix = None;
        }
        // A mode clicked in the window swings in the 3D view until another increment is
        // shown; a playing sound with its overlay takes precedence.
        if let Some(increment) = actions.show {
            sound.preview = Some(increment);
            sound.shape_clock = (0.0, f64::NEG_INFINITY);
        }
        if sound
            .preview
            .is_some_and(|p| p != view.increment && actions.show.is_none())
        {
            sound.preview = None;
        }
        let audible = self.audio.as_ref().is_some_and(|a| a.synth().sounding());
        let playing_overlay = sound.show_shape && audible;
        let only = (!playing_overlay).then_some(sound.preview).flatten();
        let overlay = (playing_overlay || only.is_some()) && !actions.close;
        // The overlay takes the place of an animation.
        if overlay && view.animation.is_some() {
            view.stop_animation();
        }
        if overlay
            && let Some((field, component)) =
                (view.current()).map(|(f, c)| (f.name.clone(), c.name.clone()))
            && let Some(sound) = &mut view.sound
        {
            if sound
                .mix
                .as_ref()
                .is_some_and(|m| !m.shows(&field, &component, only))
            {
                sound.mix = None;
            }
            if sound.mix.is_none() {
                let mix = sound::ShapeMix::new(sound, &view.increments, &field, &component, only);
                sound.mix = mix;
            }
            // A new frame at the chosen rate; the swings advance with the chosen speed, so
            // changing it while playing does not make the shape jump.
            let period = 1.0 / sound.shape_fps.max(1.0) as f64;
            let (swings, last) = sound.shape_clock;
            let since = now - last;
            if since >= period || view.superposition.is_none() {
                let step = if last.is_finite() { since } else { 0.0 };
                let swings = swings + step * sound.shape_speed as f64;
                sound.shape_clock = (swings, now);
                let time = now - sound.started.unwrap_or(now);
                view.superposition = sound.mix.as_ref().map(|m| m.frame(swings, time));
                self.results_changed = true;
                ctx.request_repaint_after(std::time::Duration::from_secs_f64(period));
            } else {
                ctx.request_repaint_after(std::time::Duration::from_secs_f64(period - since));
            }
        } else if view.superposition.take().is_some() {
            self.results_changed = true;
        }
        if let Some(increment) = actions.show
            && increment != view.increment
        {
            view.stop_animation();
            view.select_increment(increment);
            self.results_changed = true;
        }
        if actions.close {
            view.sound = None;
            self.audio = None;
        }
    }

    /// Draws the shells and beams of the shown FE model or results as solids or as surfaces
    /// and lines, as the settings ask, again after their sections changed.
    fn update_thick_shells(&mut self) {
        let model = match self.tree_view {
            TreeView::FeModel => self.model.as_mut(),
            TreeView::Results => self.results.get_mut(self.current_result),
            TreeView::Geometry => None,
        };
        let Some(model) = model else {
            return;
        };
        let graphics = &self.settings.graphics;
        let (shells, lines) = (graphics.shell_thickness, graphics.beam_profiles);
        let shells_current = if shells {
            model.shell_sections().eq(&model.thick_shells)
        } else {
            model.thick_shells.is_empty()
        };
        let lines_current = if lines {
            (model.line_sections()).eq((model.thick_lines.iter()).map(|(s, p)| (s, p.as_ref())))
        } else {
            model.thick_lines.is_empty()
        };
        if shells_current && lines_current {
            return;
        }
        model.thick_shells = (model.shell_sections().filter(|_| shells).cloned()).collect();
        model.thick_lines = (model.line_sections().filter(|_| lines))
            .map(|(s, p)| (s.clone(), p.cloned()))
            .collect();
        self.results_changed = true;
    }

    /// Rebuilds the scene after the result selection or deformation changed.
    fn rebuild_if_results_changed(&mut self) {
        let highlighted = self.highlight_changed.take();
        if !(self.results_changed || self.frame_changed)
            && let Some(parts) = highlighted
        {
            self.rebuild_highlighted_parts(parts);
            return;
        }
        let frame_only = !self.results_changed && self.frame_changed;
        if !(std::mem::take(&mut self.results_changed) | std::mem::take(&mut self.frame_changed)) {
            return;
        }
        let model = match self.tree_view {
            TreeView::Results => self.results.get_mut(self.current_result),
            TreeView::Geometry => self.geometry.as_mut(),
            TreeView::FeModel => self.model.as_mut(),
        };
        let Some(model) = model else {
            self.frame_cache.clear();
            self.viewport.set_parts(&[]);
            self.update_contour();
            return;
        };
        let frame = model
            .results
            .as_ref()
            .and_then(|v| v.animation.as_ref())
            .map(|a| a.frame);
        if !frame_only {
            self.frame_cache.clear();
        }
        let fresh;
        let meshes: &Vec<RenderMesh> = match frame {
            Some(frame) => self
                .frame_cache
                .entry(frame)
                .or_insert_with(|| model.scene_meshes()),
            None => {
                fresh = model.scene_meshes();
                &fresh
            }
        };
        self.viewport.set_parts(meshes);
        for (index, (part, mesh)) in model.parts.iter_mut().zip(meshes).enumerate() {
            part.bounds = mesh.bounds();
            self.viewport.set_part_visible(index, part.visible);
        }
        if model.explosion.is_shown()
            && let Some(bounds) = model.visible_bounds()
        {
            self.viewport.cover(bounds);
        }
        self.update_contour();
    }

    /// Rebuilds only the meshes of the parts whose highlight changed, or the whole scene
    /// where the shown model can't draw a single part.
    fn rebuild_highlighted_parts(&mut self, parts: BTreeSet<usize>) {
        let model = match self.tree_view {
            TreeView::Results => self.results.get(self.current_result),
            TreeView::Geometry => self.geometry.as_ref(),
            TreeView::FeModel => self.model.as_ref(),
        };
        let Some(model) = model else {
            return;
        };
        let meshes: Option<Vec<(usize, RenderMesh)>> = (parts.into_iter())
            .map(|index| Some((index, model.part_scene_mesh(index)?)))
            .collect();
        let Some(meshes) = meshes else {
            self.results_changed = true;
            self.rebuild_if_results_changed();
            return;
        };
        for (index, mesh) in &meshes {
            self.viewport.set_part(*index, mesh);
        }
        self.update_contour();
    }

    /// Contour settings and annotations of the 3D view for the current result.
    fn update_contour(&mut self) {
        let model = match self.tree_view {
            TreeView::Results => self.results.get(self.current_result),
            TreeView::Geometry => self.geometry.as_ref(),
            TreeView::FeModel => self.model.as_ref(),
        };
        let Some(model) = model else {
            self.viewport.options.contour = None;
            self.viewport.options.lic = None;
            self.viewport.overlay = Overlay {
                show_view_triad: self.settings.graphics.view_triad,
                ..Overlay::default()
            };
            return;
        };
        let view = model.results.as_ref();
        // Results have their own unit system; the geometry is in that of the FE model.
        let units = match view {
            Some(view) => view.units,
            None => self
                .model
                .as_ref()
                .map_or(model.fe.properties.units, |m| m.fe.properties.units),
        };
        let axis = self.axis_line(model);
        let legend = view.and_then(ResultsView::legend);
        self.viewport.options.contour =
            (legend.as_ref()).map(|legend| plx_render::ContourPalette::new(&legend.palette()));
        self.viewport.options.lic = view.and_then(crate::lic::display);
        let (graphics, post) = (&self.settings.graphics, &self.settings.post);
        // The open transformation dialog shows the points and axis of the selected item.
        let transformation = (self.transformation_dialog.as_ref())
            .filter(|d| self.tree_view == TreeView::Results && d.result == self.current_result);
        let marker = |label: &str, extreme: Option<(usize, f32, usize)>| {
            let (index, value, item) = extreme?;
            Some(Marker {
                position: model.node_position_on(index, item)?,
                text: format!(
                    "{label}: {}\nNode id: {}",
                    format_legend_value(value),
                    model.mesh.node_ids()[index]
                ),
            })
        };
        // On the section plane the extremes lie between nodes.
        let section = (self.section_values.as_ref())
            .and_then(|v| v.extremes)
            .filter(|_| view.is_some_and(|v| v.section_range.is_some()));
        let section_marker = |label: &str, (position, value): (glam::DVec3, f32)| {
            Some(Marker {
                position: model.to_render(position.to_array()),
                text: format!("{label}: {}\nSection plane", format_legend_value(value)),
            })
        };
        let picked = self.query.as_ref().map_or(&[][..], QueryWindow::picked);
        let query = crate::query::marks(model, picked);
        self.viewport.overlay = Overlay {
            legend,
            annotations: query.markers,
            annotation_owners: query.owners,
            status: view
                .filter(|_| post.status_block)
                .map_or_else(Vec::new, |v| v.status_lines(&model.file_name())),
            maximum: view.filter(|_| post.max_label).and_then(|v| match section {
                Some([_, max]) => section_marker("Max", max),
                None => marker("Max", v.maximum()),
            }),
            minimum: view.filter(|_| post.min_label).and_then(|v| match section {
                Some([min, _]) => section_marker("Min", min),
                None => marker("Min", v.minimum()),
            }),
            global_origin: graphics.global_axes.then(|| model.global_origin()),
            show_scale_bar: graphics.scale_bar,
            length_unit: units.unit(plx_model::Quantity::Length),
            length_factor: Some(
                (model.stored_units()).factor_to(plx_model::Quantity::Length, units) as f32,
            ),
            show_view_triad: graphics.view_triad,
            nodes: (model.highlight.nodes.iter())
                .filter_map(|&id| model.node_position(model.mesh.node_index(id)?))
                .chain(transformation.iter().flat_map(|d| d.points(model)))
                .chain(query.points)
                .collect(),
            edges: render_lines(model, &model.highlight.lines),
            secondary_edges: render_lines(model, &model.highlight.secondary_lines),
            axis,
            paths: self.overlay_paths(),
            lines: (transformation
                .map_or_else(Vec::new, |d| d.lines(model))
                .into_iter())
            .chain(query.lines)
            .collect(),
            features: Vec::new(),
            result_path: None,
        };
        self.update_feature_overlay();
    }
}

/// Lines between nodes in render coordinates; lines with an unknown node are left out.
fn render_lines(model: &Model, lines: &[[plx_mesh::NodeId; 2]]) -> Vec<[glam::Vec3; 2]> {
    (lines.iter())
        .filter_map(|ends| {
            let [a, b] = ends.map(|id| {
                model
                    .mesh
                    .node_index(id)
                    .and_then(|n| model.node_position(n))
            });
            Some([a?, b?])
        })
        .collect()
}

/// What the results tool bar asks for.
struct ToolBarResponse {
    /// Anything that affects the scene changed.
    changed: bool,
    /// The transformations button was clicked.
    transformations: bool,
    /// The colour scale button was clicked.
    color_scale: bool,
}

/// The representation of the shown field: contours, or arrows or LIC streaks along its
/// directions, with the principal direction of a tensor field and the options of each.
/// Returns whether anything changed.
fn display_controls(ui: &mut egui::Ui, view: &mut ResultsView) -> bool {
    use plx_results::display::Representation;
    let before = view.vector_display.clone();
    let field = view.current().map(|(f, _)| f.rank);
    let directional = field.is_some_and(|rank| rank != plx_results::FieldRank::Scalar);
    let display = &mut view.vector_display;
    ui.label("Display");
    let shown = if directional {
        display.representation
    } else {
        Representation::Contour
    };
    ui.add_enabled_ui(directional, |ui| {
        egui::ComboBox::from_id_salt("representation")
            .selected_text(shown.label())
            .width(80.0)
            .show_ui(ui, |ui| {
                for choice in Representation::ALL {
                    ui.selectable_value(&mut display.representation, choice, choice.label());
                }
            })
            .response
            .on_disabled_hover_text(
                "Arrows and LIC need a vector or tensor field, such as DISP or STRESS",
            )
    });
    let tensor = field.is_some_and(|rank| rank.is_tensor());
    if tensor && shown != Representation::Contour {
        egui::ComboBox::from_id_salt("principal direction")
            .selected_text(display.principal.label())
            .width(70.0)
            .show_ui(ui, |ui| {
                for choice in plx_results::Principal::ALL {
                    ui.selectable_value(&mut display.principal, choice, choice.label());
                }
            })
            .response
            .on_hover_text("Principal direction the arrows or streaks follow");
    }
    if shown == Representation::Arrows {
        let arrows = &mut display.arrows;
        ui.menu_button("…", |ui| {
            use plx_results::display::ArrowOptions;
            ui.horizontal(|ui| {
                ui.label("Number of arrows");
                ui.add(
                    numeric::drag_value(&mut arrows.count)
                        .range(ArrowOptions::MIN_COUNT..=ArrowOptions::MAX_COUNT)
                        .speed(20.0),
                );
            });
            ui.horizontal(|ui| {
                ui.label("Length");
                ui.add(
                    numeric::drag_value(&mut arrows.length)
                        .range(0.1..=5.0)
                        .speed(0.02),
                )
                .on_hover_text("Length of the longest arrow relative to the arrow spacing");
            });
            ui.checkbox(&mut arrows.by_magnitude, "Length by magnitude");
            ui.checkbox(&mut arrows.colored, "Arrows in contour colors")
                .on_hover_text("Otherwise the arrows are black over the contours");
        })
        .response
        .on_hover_text("Arrow options");
    }
    if shown == Representation::Lic {
        let lic = &mut display.lic;
        ui.menu_button("…", |ui| {
            use plx_results::display::LicOptions;
            egui::Grid::new("lic options")
                .num_columns(2)
                .show(ui, |ui| {
                    ui.label("Streak length");
                    ui.add(
                        numeric::drag_value(&mut lic.length)
                            .range(LicOptions::LENGTH)
                            .speed(0.2),
                    )
                    .on_hover_text("Length of the streaks to either side of a pixel, in pixels");
                    ui.end_row();
                    ui.label("Grain size");
                    ui.add(
                        numeric::drag_value(&mut lic.grain)
                            .range(LicOptions::GRAIN)
                            .speed(0.002),
                    )
                    .on_hover_text(
                        "Size of the noise grains in percent of the model size; never finer \
                     than a pixel",
                    );
                    ui.end_row();
                    ui.label("Contrast");
                    ui.add(
                        numeric::drag_value(&mut lic.contrast)
                            .range(LicOptions::CONTRAST)
                            .speed(0.05),
                    );
                    ui.end_row();
                    ui.label("Mix");
                    ui.add(
                        numeric::drag_value(&mut lic.mix)
                            .range(0.0..=1.0)
                            .speed(0.01),
                    )
                    .on_hover_text("How much of the brightness the streaks set, 0 to 1");
                    ui.end_row();
                });
            if ui.button("Defaults").clicked() {
                *lic = LicOptions::default();
            }
        })
        .response
        .on_hover_text("LIC options");
    }
    view.vector_display != before
}

/// The width of the labels at the start of both rows of the results controls.
const LABEL: f32 = 76.0;

/// The results tool bar: deformation, colour bands, transformations and the increment
/// with its navigation buttons.
fn results_tool_bar(
    ui: &mut egui::Ui,
    view: &mut ResultsView,
    transformed: bool,
    has_global: bool,
) -> ToolBarResponse {
    let mut changed = false;
    let mut transformations = false;
    let mut color_scale = false;
    // Two rows: the deformation and the transformations above, the colours, the display and
    // the increment below, each label in a column of the same width.
    let label = |ui: &mut egui::Ui, text: &str| {
        ui.add_sized(
            [LABEL, 20.0],
            egui::Label::new(text).halign(egui::Align::LEFT),
        );
    };
    ui.spacing_mut().item_spacing.y = 6.0;
    ui.horizontal(|ui| {
        label(ui, "Deformation");
        let before = view.deformation;
        egui::ComboBox::from_id_salt("deformation")
            .selected_text(view.deformation.label())
            .width(150.0)
            .show_ui(ui, |ui| {
                for choice in Deformation::CHOICES {
                    let response =
                        ui.selectable_value(&mut view.deformation, choice, choice.label());
                    view.deformation_chosen |= response.clicked();
                }
            });
        changed |= view.deformation != before;
        ui.label("Factor");
        let user = view.deformation == Deformation::UserDefined;
        let factor = if user {
            view.user_scale
        } else {
            f64::from(view.scale())
        };
        // A text field of fixed width: a long factor is cut off inside the frame instead of
        // widening the field and pushing the items on its right around.
        let id = ui.id().with("deformation factor");
        let focused = ui.memory(|m| m.has_focus(id));
        let mut text = focused
            .then(|| ui.data_mut(|d| d.get_temp::<String>(id)))
            .flatten()
            .unwrap_or_else(|| numeric::format_physical(factor));
        let response = ui
            .add_enabled(
                user,
                egui::TextEdit::singleline(&mut text)
                    .id(id)
                    .desired_width(80.0),
            )
            .on_hover_text(numeric::format_physical(factor));
        if response.has_focus() {
            ui.data_mut(|d| d.insert_temp(id, text.clone()));
        }
        if user && response.lost_focus() {
            ui.data_mut(|d| d.remove_temp::<String>(id));
            if let Some(value) = numeric::evaluate(&text)
                && value != view.user_scale
            {
                view.user_scale = value;
                changed = true;
            }
        }
        changed |= ui
            .checkbox(&mut view.show_undeformed, "Show Undeformed")
            .changed();
        if has_global {
            changed |= ui
                .checkbox(&mut view.show_global_contours, "Show Global Contours")
                .on_hover_text(
                    "Draws the edges of the global model the submodel reads, deformed by the \
                     same scale, around the results",
                )
                .changed();
        }
        ui.separator();
        let tip = "Transformations: symmetries and patterns";
        transformations = icons::button(ui, Icon::Transformation, tip, true, transformed).clicked();
    });
    ui.horizontal(|ui| {
        label(ui, "Color levels");
        changed |= ui
            .add(
                numeric::drag_value(&mut view.color_scale.levels)
                    .range(2..=plx_render::contour::MAX_LEVELS),
            )
            .changed();
        color_scale = (ui.button("…"))
            .on_hover_text("Color scale: spectrum and limits")
            .clicked();
        ui.separator();
        changed |= display_controls(ui, view);
        ui.separator();

        ui.label("Step, Increment");
        let mut increment = view.increment;
        let selected = view
            .current_increment()
            .map(ResultsView::increment_label)
            .unwrap_or_default();
        egui::ComboBox::from_id_salt("increment")
            .selected_text(selected)
            .width(70.0)
            .show_ui(ui, |ui| {
                for (index, inc) in view.increments.iter().enumerate() {
                    ui.selectable_value(&mut increment, index, ResultsView::increment_label(inc));
                }
            });
        ui.spacing_mut().item_spacing.x = 1.0;
        let last = view.increments.len().saturating_sub(1);
        let current = view.increment;
        for (icon, tooltip, target) in [
            (Icon::First, "First Increment", 0),
            (
                Icon::Previous,
                "Previous Increment",
                current.saturating_sub(1),
            ),
            (Icon::Next, "Next Increment", (current + 1).min(last)),
            (Icon::Last, "Last Increment", last),
        ] {
            let enabled = target != current;
            if icons::button(ui, icon, tooltip, enabled, false).clicked() {
                increment = target;
            }
        }
        if increment != view.increment {
            view.switch_increment(increment);
            changed = true;
        }
        ui.add_space(4.0);
        let animating = view.animation.is_some();
        if icons::button(ui, Icon::Animate, "Animation", true, animating).clicked() {
            if animating {
                view.stop_animation();
            } else {
                view.start_animation(AnimationKind::ScaleFactor);
            }
            changed = true;
        }
        // Only modes of a frequency step have a sound; otherwise the button is greyed out.
        let sounding = view.sound.is_some();
        let is_mode = view
            .current_increment()
            .is_some_and(|i| i.kind == plx_results::AnalysisKind::Frequency);
        let tip = "Sound of the mode shapes";
        if icons::button(ui, Icon::Sound, tip, sounding || is_mode, sounding).clicked() {
            view.sound = match view.sound {
                Some(_) => None,
                None => ModeSound::new(&view.increments, view.increment),
            };
        }
    });
    ToolBarResponse {
        changed,
        transformations,
        color_scale,
    }
}

enum WindowEvent {
    None,
    /// Another frame is shown.
    Frame,
    /// Settings changed that affect every frame.
    Settings,
    /// Export of the animation as a file of this format.
    Export(Format),
    Close,
}

/// Asks for the file and starts recording the frames of the animation of `view` in the order
/// of its playback; `None` when cancelled or FFmpeg is missing for a video.
fn start_animation_export(
    view: &mut ResultsView,
    format: Format,
    ffmpeg: PathBuf,
    output: &mut Vec<String>,
) -> Option<animation_export::Recording> {
    let animation = view.animation.as_mut()?;
    if format.needs_ffmpeg()
        && let Err(err) = animation_export::ffmpeg_version(&ffmpeg)
    {
        output.push(format!(
            "{err}. Exporting {} needs FFmpeg: install it or set its path under Edit > \
             Settings > Post-processing; GIF works without it.",
            format.extension().to_uppercase()
        ));
        return None;
    }
    let path = rfd::FileDialog::new()
        .set_title("Export Animation")
        .add_filter(format.filter_name(), &[format.extension()])
        .set_file_name(format!("animation.{}", format.extension()))
        .save_file()?;
    let path = if path.extension().is_none() {
        path.with_extension(format.extension())
    } else {
        path
    };
    let file = path.file_name().map_or_else(
        || path.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    );
    let restore = (animation.frame, animation.playing);
    animation.playing = false;
    let sequence = animation_export::frame_sequence(animation.frame_count(), animation.playback);
    let encoder = animation_export::Encoder::start(animation_export::Job {
        format,
        path,
        fps: animation.fps,
        repeat: animation.playback != Playback::Once,
        ffmpeg,
    });
    Some(animation_export::Recording::new(
        sequence, encoder, file, restore,
    ))
}

/// The animation dialog: kind, frames, speed, playback mode, colour limits and the
/// player controls.
/// `exporting` greys out the export button while an exported animation is being written.
fn animation_window(ctx: &egui::Context, view: &mut ResultsView, exporting: bool) -> WindowEvent {
    let mut event = WindowEvent::None;
    let mut open = true;
    let mut restart = None;
    egui::Window::new("Animation")
        .id(crate::ids::window(ctx, "animation"))
        .open(&mut open)
        .collapsible(false)
        .resizable(false)
        .pivot(egui::Align2::RIGHT_BOTTOM)
        .default_pos(ctx.content_rect().right_bottom() + egui::vec2(-130.0, -230.0))
        .show(ctx, |ui| {
            let Some(animation) = &mut view.animation else {
                return;
            };
            egui::Grid::new("animation settings")
                .num_columns(2)
                .spacing([12.0, 6.0])
                .show(ui, |ui| {
                    ui.label("Type");
                    ui.horizontal(|ui| {
                        for (kind, label) in [
                            (AnimationKind::ScaleFactor, "Scale factor"),
                            (AnimationKind::Increments, "Step increments"),
                            (AnimationKind::Steps, "Steps"),
                        ] {
                            if ui.radio(animation.kind == kind, label).clicked()
                                && animation.kind != kind
                            {
                                restart = Some(kind);
                            }
                        }
                    });
                    ui.end_row();
                    if animation.kind == AnimationKind::ScaleFactor {
                        ui.label("Frames");
                        let frames = numeric::drag_value(&mut animation.frames).range(2..=200);
                        if ui.add(frames).changed() {
                            animation.go_to(animation.frame);
                            event = WindowEvent::Settings;
                        }
                        ui.end_row();
                    }
                    ui.label("Frames per second");
                    ui.add(numeric::drag_value(&mut animation.fps).range(1.0..=60.0));
                    ui.end_row();
                    ui.label("Playback");
                    ui.horizontal(|ui| {
                        for (playback, label) in [
                            (Playback::Once, "Once"),
                            (Playback::Loop, "Loop"),
                            (Playback::Swing, "Swing"),
                        ] {
                            ui.radio_value(&mut animation.playback, playback, label);
                        }
                    });
                    ui.end_row();
                    ui.label("Color scale");
                    ui.horizontal(|ui| {
                        for (limits, label) in [
                            (ColorLimits::CurrentFrame, "Current frame"),
                            (ColorLimits::AllFrames, "All frames"),
                        ] {
                            if ui
                                .radio_value(&mut animation.limits, limits, label)
                                .changed()
                            {
                                event = WindowEvent::Settings;
                            }
                        }
                    });
                    ui.end_row();
                });
            ui.separator();
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 1.0;
                let last = animation.frame_count() - 1;
                let frame = animation.frame;
                let mut target = None;
                if icons::button(ui, Icon::First, "First Frame", frame > 0, false).clicked() {
                    target = Some(0);
                }
                if icons::button(ui, Icon::Previous, "Previous Frame", frame > 0, false).clicked() {
                    target = Some(frame - 1);
                }
                let (icon, tip) = if animation.playing {
                    (Icon::Pause, "Pause")
                } else {
                    (Icon::Animate, "Play")
                };
                if icons::button(ui, icon, tip, true, false).clicked() {
                    if animation.playing {
                        animation.playing = false;
                    } else {
                        animation.play();
                    }
                }
                if icons::button(ui, Icon::Next, "Next Frame", frame < last, false).clicked() {
                    target = Some(frame + 1);
                }
                if icons::button(ui, Icon::Last, "Last Frame", frame < last, false).clicked() {
                    target = Some(last);
                }
                ui.add_space(8.0);
                let mut slider = frame;
                let response = ui.add(egui::Slider::new(&mut slider, 0..=last).show_value(false));
                if response.changed() {
                    target = Some(slider);
                }
                ui.add_space(8.0);
                ui.label(format!("Frame {} of {}", frame + 1, last + 1));
                ui.add_space(8.0);
                let tip = if exporting {
                    "Export Animation (the last export is still being written)"
                } else {
                    "Export Animation"
                };
                let export = icons::button(ui, Icon::Save, tip, !exporting, false);
                egui::Popup::menu(&export).show(|ui| {
                    for format in Format::ALL {
                        if ui.button(format.menu_label()).clicked() {
                            event = WindowEvent::Export(format);
                        }
                    }
                });
                if let Some(target) = target {
                    animation.playing = false;
                    if animation.go_to(target) && matches!(event, WindowEvent::None) {
                        event = WindowEvent::Frame;
                    }
                }
            });
            if crate::dialog::tool_footer(ui, None) == crate::dialog::Tool::Close {
                event = WindowEvent::Close;
            }
        });
    if let Some(kind) = restart {
        view.start_animation(kind);
        event = WindowEvent::Settings;
    }
    if !open {
        event = WindowEvent::Close;
    }
    event
}

/// The list of problems in the Problems tab: errors and warnings with what they are about.
/// Returns the problem clicked, and whether it was double-clicked.
fn problems_list(
    ui: &mut egui::Ui,
    problems: &[plx_model::Finding],
) -> Option<(plx_model::Finding, bool)> {
    if problems.is_empty() {
        ui.weak("No problems found: the model check has nothing to report.");
        return None;
    }
    let mut picked = None;
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            for finding in problems {
                let (kind, color) = match finding.severity() {
                    plx_model::Severity::Error => ("Error", crate::style::error()),
                    plx_model::Severity::Warning => ("Warning", crate::style::warning()),
                };
                let row = ui
                    .horizontal(|ui| {
                        ui.colored_label(color, egui::RichText::new(kind).strong());
                        ui.label(egui::RichText::new(finding.problem.title()).strong());
                        ui.weak(&finding.detail);
                    })
                    .response;
                let row = ui
                    .interact(row.rect, row.id.with("problem"), egui::Sense::click())
                    .on_hover_text(format!(
                        "{}\n\nRemedy: {}\n\nClick to select it in the tree, \
                         double-click to open it.",
                        finding.problem.explanation(),
                        finding.problem.fix()
                    ));
                if row.hovered() {
                    ui.painter().rect_filled(
                        row.rect,
                        2.0,
                        ui.visuals()
                            .widgets
                            .hovered
                            .weak_bg_fill
                            .gamma_multiply(0.3),
                    );
                }
                if row.double_clicked() {
                    picked = Some((finding.clone(), true));
                } else if row.clicked() {
                    picked = Some((finding.clone(), false));
                }
            }
        });
    picked
}

/// The step an item of the FE model tree belongs to.
fn step_of(item: &TreeItem) -> Option<usize> {
    match *item {
        TreeItem::Step(s)
        | TreeItem::StepGroup(s, _)
        | TreeItem::BoundaryCondition(s, _)
        | TreeItem::Load(s, _)
        | TreeItem::FieldOutput(s, _)
        | TreeItem::HistoryOutput(s, _)
        | TreeItem::DefinedField(s, _) => Some(s),
        _ => None,
    }
}

/// Prefixes an output message with the local time, `HH:MM:SS`.
fn timestamped(message: String) -> String {
    format!("{} {message}", chrono::Local::now().format("%H:%M:%S"))
}

/// The container a new item goes into, to find whether another analysis owns it.
fn new_item_container(kind: &NewItem) -> Option<TreeItem> {
    Some(match kind {
        NewItem::Material => TreeItem::Group("Materials"),
        NewItem::Section => TreeItem::Group("Sections"),
        NewItem::CrossSection => TreeItem::Group(crate::tree::CROSS_SECTIONS),
        NewItem::InitialCondition => TreeItem::Group("Initial Conditions"),
        NewItem::Step => TreeItem::Group("Steps"),
        NewItem::BoundaryCondition(s)
        | NewItem::Load(s)
        | NewItem::FieldOutput(s)
        | NewItem::HistoryOutput(s)
        | NewItem::ContactHistoryOutput(s, _)
        | NewItem::ContactFieldOutput(s)
        | NewItem::DefinedField(s) => TreeItem::Step(*s),
        NewItem::Constraint => TreeItem::Group("Constraints"),
        NewItem::SurfaceInteraction
        | NewItem::ContactRegion
        | NewItem::Bolt
        | NewItem::ContactPair
        | NewItem::Tie
        | NewItem::NodeTie => TreeItem::Group("Contacts"),
        NewItem::Amplitude => TreeItem::Group("Amplitudes"),
        NewItem::Distribution => TreeItem::Group("Distributions"),
        NewItem::MeshSetupItem => TreeItem::Group("Mesh Setup"),
        _ => return None,
    })
}

/// What the project window asks of a window it opens: the workspace, and a step to select
/// in the FE Model tree.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Show {
    pub view: TreeView,
    pub step: Option<String>,
}

/// The workspace after a results file opened: the Results tab, unless the project window
/// asked for another one when it opened the analysis. The request is used up.
fn results_workspace(pinned: &mut Option<TreeView>) -> TreeView {
    pinned.take().unwrap_or(TreeView::Results)
}

/// Writes the findings of the solver's model check into the monitor, with the hint where the
/// tree shows them.
pub(crate) fn note_solver_findings(analysis: &mut Analysis, findings: &[plx_model::Finding]) {
    let mut shown = std::collections::HashSet::new();
    for finding in findings {
        // A finding is attached to its step and to each part: say it once.
        if !shown.insert((finding.problem, finding.detail.clone())) {
            continue;
        }
        analysis.note(format!(
            "Model check: {}: {}. {}",
            finding.problem.title(),
            finding.detail,
            finding.problem.fix()
        ));
    }
    if findings.is_empty() {
        analysis.note("Model check: the solver found no problem.".into());
    } else {
        analysis.note(
            "The warning symbols in the tree show where; their tooltips explain the cause.".into(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn carried_settings_lose_reference_points_deleted_from_the_fe_model() {
        let mut settings = plx_results::ResultsSettings {
            reference_points: vec![
                plx_model::ReferencePoint::new("RP-1", [0.0; 3]),
                plx_model::ReferencePoint::new("Mine", [1.0; 3]),
            ],
            inherited_features: vec!["RP:RP-1".into()],
            ..Default::default()
        };
        drop_removed_features(&mut settings, &plx_model::FeModel::default());
        let names: Vec<_> = settings.reference_points.iter().map(|r| &r.name).collect();
        assert_eq!(names, ["Mine"]);
        assert!(settings.inherited_features.is_empty());
    }

    #[test]
    fn bound_results_keep_the_workspace_the_project_window_asked_for() {
        let mut pinned = Some(TreeView::Geometry);
        assert_eq!(results_workspace(&mut pinned), TreeView::Geometry);
        // Results opened later, e.g. by a solve, show the Results tab again.
        assert_eq!(results_workspace(&mut pinned), TreeView::Results);
    }

    #[test]
    fn results_of_an_earlier_request_do_not_replace_newer_ones() {
        let kept = std::path::Path::new("/tmp/Kept_Results/Analysis-1.frd");
        let new = std::path::Path::new("/tmp/Analysis-1/Analysis-1.frd");
        // The kept results were asked for first, the new run's after them.
        assert!(is_latest_results(Some(new), new));
        assert!(!is_latest_results(Some(new), kept));
        assert!(is_latest_results(None, kept));
    }

    #[test]
    fn import_into_saved_new_model_keeps_the_project_file() {
        let saved = Model::new(
            std::path::Path::new("/tmp/neu.ndl"),
            plx_mesh::FeMesh::default(),
        );
        let mut imported = Model::new(
            std::path::Path::new("/tmp/teil.step"),
            plx_mesh::FeMesh::default(),
        );
        take_over_empty_model(&mut imported, &saved);
        assert_eq!(imported.path, std::path::Path::new("/tmp/neu.ndl"));
        assert_eq!(imported.path, saved.path);
    }
}
