//! The commands of the ribbon in an analysis: whether they can run, whether they show
//! pressed, and what they do.

use super::*;
use crate::command::{Cmd, Workspace};
use crate::status::CellState;

impl AnalysisWindow {
    /// The workspace the analysis shows.
    pub fn workspace(&self) -> Workspace {
        Workspace::of(self.workbench.tree_view)
    }

    /// Shows the workspace, as its segment in the app bar does.
    pub fn set_workspace(&mut self, workspace: Workspace) {
        if let Some(view) = workspace.view() {
            self.workbench.pinned_view = None;
            self.workbench.set_tree_view(view);
        }
    }

    /// Goes on with what the analysis does in the background while the project window shows
    /// another workspace: loading, meshing, restoring results.
    pub fn tick(&mut self, ctx: &egui::Context) {
        self.handle_load_events();
        self.workbench.poll_meshing();
        self.workbench.poll_bound_results(ctx);
        self.workbench.poll_global_results(ctx);
    }

    /// The state of the analysis in the app bar's pill.
    pub fn pill(&mut self) -> Option<(CellState, String)> {
        if self.loading.is_some() {
            return Some((CellState::Running, "Loading …".into()));
        }
        if self.workbench.meshing.is_some() {
            return Some((CellState::Running, "Meshing …".into()));
        }
        let status = self.workbench.cached_status()?;
        let problems = self.workbench.solver_findings.len();
        let text = match self.workbench.tree_view {
            TreeView::Geometry if self.workbench.geometry.is_none() => {
                return Some((CellState::Missing, "No geometry".into()));
            }
            TreeView::Geometry => match (status.mesh, status.stale_parts.len()) {
                (CellState::Missing, _) => return Some((CellState::Missing, "No mesh".into())),
                (_, 0) => return Some((status.mesh, "Mesh up to date".into())),
                (_, 1) => {
                    return Some((CellState::OutOfDate, "Mesh out of date for 1 part".into()));
                }
                (_, n) => {
                    return Some((
                        CellState::OutOfDate,
                        format!("Mesh out of date for {n} parts"),
                    ));
                }
            },
            _ => match status.solution {
                CellState::Missing => "Not solved",
                CellState::UpToDate => "Solution up to date",
                CellState::OutOfDate => "Solution out of date",
                CellState::Running => "Solving …",
                CellState::Failed => "Solved with errors",
                CellState::NotUsed => "",
            },
        };
        let text = match problems {
            0 => text.to_string(),
            1 => format!("{text}, 1 problem"),
            n => format!("{text}, {n} problems"),
        };
        Some((status.solution, text))
    }

    /// The pill was clicked: the solver's problems, else the monitor.
    /// The checklist of the workspace shown, for the properties panel.
    pub fn checklist(&self) -> Vec<crate::checklist::Step> {
        let w = &self.workbench;
        match w.tree_view {
            TreeView::Geometry => crate::checklist::geometry(w.geometry.as_ref(), w.model.as_ref()),
            TreeView::FeModel => crate::checklist::fe_model(w.model.as_ref()),
            TreeView::Results => crate::checklist::results(
                !w.results.is_empty()
                    || w.model.as_ref().is_some_and(|m| m.bound_results.is_some()),
            ),
        }
    }

    pub fn pill_clicked(&mut self) {
        // With problems the Problems tab, else the monitor of the solver.
        let workbench = &mut self.workbench;
        if workbench.solver_findings.is_empty() {
            workbench.analysis_tab = true;
            workbench.problems_tab = false;
        } else {
            workbench.problems_tab = true;
        }
    }

    pub fn can_undo(&self) -> bool {
        let workbench = &self.workbench;
        workbench.undo_blocked().is_none()
            && (workbench.model.as_ref()).is_some_and(|m| m.history.can_undo())
    }

    pub fn can_redo(&self) -> bool {
        let workbench = &self.workbench;
        workbench.undo_blocked().is_none()
            && (workbench.model.as_ref()).is_some_and(|m| m.history.can_redo())
    }

    pub fn undo(&mut self) {
        self.workbench.undo();
    }

    pub fn redo(&mut self) {
        self.workbench.redo();
    }

    pub fn open_settings(&mut self) {
        let workbench = &mut self.workbench;
        workbench.settings_window = Some(SettingsWindow::new(&workbench.settings));
    }

    pub fn open_user_guide(&mut self) {
        let line = crate::help::open_user_guide();
        self.workbench.output.push(line);
    }

    /// The parts selected in the tree of the shown workspace.
    fn selected_parts(&self) -> Vec<usize> {
        let workbench = &self.workbench;
        let view = workbench.tree_view;
        let mut parts: Vec<usize> = workbench.tree.selected_parts(view).into_iter().collect();
        if parts.is_empty()
            && let Some((v, TreeItem::Part(index))) = &workbench.tree.selected
            && *v == view
        {
            parts.push(*index);
        }
        parts
    }

    /// Whether the command can run now.
    pub fn command_enabled(&self, cmd: Cmd) -> bool {
        let w = &self.workbench;
        let idle = self.loading.is_none();
        let running = w.running();
        let meshing = w.meshing.is_some();
        let setup = w.setup_model();
        let has_setup = setup.is_some();
        let has_geometry = w.model.as_ref().is_some_and(|m| m.geometry.is_some());
        let has_mesh = setup.is_some_and(|m| m.mesh.element_count() > 0);
        let last_step = (w.active_step()).and_then(|i| setup?.fe.steps.get(i));
        let parts = self.selected_parts();
        let shown = w.shown().is_some();
        let results_open = w.shown_results_view().is_some();
        let kept = w.model.as_ref().is_some_and(|m| m.bound_results.is_some());
        match cmd {
            Cmd::Import | Cmd::ImportMesh => idle,
            Cmd::DefaultMeshParameters
            | Cmd::MeshParameters
            | Cmd::LocalMeshSize
            | Cmd::ExtrudeMesh
            | Cmd::TetrahedralGmsh => has_geometry,
            Cmd::MeshAll => has_geometry && !meshing,
            Cmd::MeshSelected => {
                has_geometry && !meshing && w.tree_view == TreeView::Geometry && !parts.is_empty()
            }
            Cmd::CancelMeshing => meshing,
            Cmd::PartProperties | Cmd::ElementType => parts.len() == 1,
            Cmd::HideShow | Cmd::DeleteParts => !parts.is_empty(),
            Cmd::Transform => w.tree_view == TreeView::FeModel && !parts.is_empty(),
            Cmd::MergeParts => w.tree_view == TreeView::FeModel && parts.len() > 1,
            Cmd::MergeNodes | Cmd::Renumber => has_mesh,
            Cmd::Query => shown,
            Cmd::RemoveAnnotations => w.has_annotations(),
            Cmd::Solve | Cmd::SolveAgain | Cmd::CheckModel => {
                w.model.is_some() && !running && !meshing && idle
            }
            Cmd::Stop => running || meshing,
            Cmd::Monitor => true,
            Cmd::Campbell => setup.is_some_and(|m| crate::campbell::applies(&m.fe)),
            Cmd::NodeSet
            | Cmd::ElementSet
            | Cmd::Surface
            | Cmd::Material
            | Cmd::MaterialLibrary
            | Cmd::Section
            | Cmd::CrossSection
            | Cmd::ModelProperties
            | Cmd::ReferencePoint
            | Cmd::CoordinateSystem
            | Cmd::Plane
            | Cmd::DetectContacts
            | Cmd::ContactRegion
            | Cmd::Bolt
            | Cmd::Constraint
            | Cmd::SurfaceInteraction
            | Cmd::ContactPair
            | Cmd::Tie
            | Cmd::NodeTie
            | Cmd::ContactWorksheet
            | Cmd::ConnectionSettings
            | Cmd::Step
            | Cmd::InitialCondition
            | Cmd::Amplitude
            | Cmd::Distribution
            | Cmd::ExportInp => has_setup,
            Cmd::BoundaryCondition | Cmd::FieldOutput | Cmd::HistoryOutput => last_step.is_some(),
            Cmd::Load => last_step.is_some_and(|s| s.kind.supports_loads()),
            Cmd::DefinedField => last_step.is_some_and(|s| s.kind.supports_defined_fields()),
            Cmd::OpenResults | Cmd::RemoveResults => !running && kept,
            Cmd::ColorScale => w.tree_view == TreeView::Results && results_open,
            Cmd::Deformation
            | Cmd::ShowUndeformed
            | Cmd::GlobalContours
            | Cmd::Representation
            | Cmd::ColorLevels
            | Cmd::Transformations
            | Cmd::ResultStep
            | Cmd::Increment
            | Cmd::FirstIncrement
            | Cmd::PreviousIncrement
            | Cmd::NextIncrement
            | Cmd::LastIncrement
            | Cmd::Animate
            | Cmd::ModeSound => w.results_command_enabled(cmd),
            Cmd::ExportVtu | Cmd::ExportVtk | Cmd::ExportFrd | Cmd::ExportDeformedMesh => {
                results_open
            }
            Cmd::HotSpot
            | Cmd::Path
            | Cmd::PlaneResult
            | Cmd::ResultHistoryOutput
            | Cmd::DerivedFieldOutput => results_open,
            Cmd::SectionView | Cmd::ExplodedView => shown,
            Cmd::SectionOff => w.section.is_some(),
            Cmd::ExplodedOff => w.exploded_applied(),
            Cmd::ZoomToFit
            | Cmd::StandardViews
            | Cmd::View(_)
            | Cmd::AxisView(_)
            | Cmd::IsometricAxis(_)
            | Cmd::VerticalView
            | Cmd::EdgesOnly
            | Cmd::MeshEdges
            | Cmd::ShellThickness
            | Cmd::BeamProfiles
            | Cmd::ShowLinked
            | Cmd::Checklist
            | Cmd::Screenshot
            | Cmd::ScreenshotClipboard
            | Cmd::ScreenshotFile
            | Cmd::Settings
            | Cmd::SolverSettings
            | Cmd::UserGuide
            | Cmd::OpenLogFolder
            | Cmd::SaveProject
            | Cmd::SaveProjectAs
            | Cmd::SaveWithoutResults => true,
            // The project carries out the rest.
            _ => false,
        }
    }

    /// Whether the tile of the command shows pressed.
    pub fn command_checked(&self, cmd: Cmd) -> bool {
        let w = &self.workbench;
        let graphics = &w.settings.graphics;
        match cmd {
            Cmd::EdgesOnly => !w.viewport.options.mesh_edges,
            Cmd::MeshEdges => w.viewport.options.mesh_edges,
            Cmd::ShellThickness => graphics.shell_thickness,
            Cmd::BeamProfiles => graphics.beam_profiles,
            Cmd::ShowLinked => graphics.show_linked,
            Cmd::Checklist => w.checklist_visible,
            Cmd::SectionView => w.section.is_some() || w.section_dialog.is_some(),
            Cmd::ExplodedView => w.exploded_applied() || w.exploded_dialog.is_some(),
            Cmd::Query => w.query.is_some(),
            Cmd::ColorScale => w.color_scale_dialog.is_some(),
            Cmd::ShowUndeformed
            | Cmd::GlobalContours
            | Cmd::Transformations
            | Cmd::Animate
            | Cmd::ModeSound => w.results_command_checked(cmd),
            Cmd::Monitor => w.analysis_tab,
            Cmd::ContactWorksheet => w.contact_worksheet,
            _ => false,
        }
    }

    /// Carries out the command, if it can run.
    pub fn run_command(&mut self, ctx: &egui::Context, cmd: Cmd) {
        if !self.command_enabled(cmd) {
            return;
        }
        let parts = self.selected_parts();
        let view = self.workbench.tree_view;
        let w = &mut self.workbench;
        let last_step = w.active_step().unwrap_or(0);
        match cmd {
            Cmd::Import | Cmd::ImportMesh => self.import(ctx),
            Cmd::DefaultMeshParameters => w.open_mesh_setup(),
            Cmd::MeshParameters | Cmd::LocalMeshSize | Cmd::TetrahedralGmsh | Cmd::ExtrudeMesh => {
                let index = match cmd {
                    Cmd::MeshParameters => 0,
                    Cmd::LocalMeshSize => 1,
                    Cmd::TetrahedralGmsh => 2,
                    _ => 3,
                };
                if let Some(model) = &w.model
                    && let Some(geometry) = &model.geometry
                {
                    let units = model.fe.properties.units;
                    w.mesh_item_editor = Some(MeshItemEditor::create_kind(geometry, units, index));
                    w.set_tree_view(TreeView::Geometry);
                }
            }
            Cmd::MeshAll => w.generate_mesh(ctx, None),
            Cmd::MeshSelected => {
                let response = TreeResponse {
                    mesh_part: parts.first().copied(),
                    ..Default::default()
                };
                w.tree_response(ctx, view, response);
            }
            Cmd::CancelMeshing => w.cancel_meshing(),
            Cmd::PartProperties | Cmd::ElementType => {
                let response = TreeResponse {
                    open: parts.first().map(|&i| TreeItem::Part(i)),
                    ..Default::default()
                };
                w.tree_response(ctx, view, response);
            }
            Cmd::HideShow => {
                // All selected parts take the opposite of the first one's visibility.
                let visible = (w.shown())
                    .and_then(|m| parts.first().and_then(|&i| m.parts.get(i)))
                    .is_some_and(|p| p.visible);
                let response = TreeResponse {
                    visibility: parts.iter().map(|&i| (i, !visible)).collect(),
                    ..Default::default()
                };
                w.tree_response(ctx, view, response);
            }
            Cmd::DeleteParts => {
                let response = TreeResponse {
                    delete: parts.first().map(|&i| TreeItem::Part(i)),
                    ..Default::default()
                };
                w.tree_response(ctx, view, response);
            }
            Cmd::Transform => {
                let response = TreeResponse {
                    transform_part: parts.first().copied(),
                    ..Default::default()
                };
                w.tree_response(ctx, view, response);
            }
            Cmd::MergeParts => {
                let response = TreeResponse {
                    merge_parts: true,
                    ..Default::default()
                };
                w.tree_response(ctx, view, response);
            }
            Cmd::MergeNodes => w.open_mesh_tool(MeshTool::MergeNodes),
            Cmd::Renumber => w.open_mesh_tool(MeshTool::Renumber),
            Cmd::Query => {
                if w.query.is_some() {
                    w.close_query();
                } else {
                    w.open_query();
                }
            }
            Cmd::RemoveAnnotations => w.annotations(AnnotationCommand::DeleteAll),
            Cmd::Solve => w.solve(),
            Cmd::SolveAgain => w.requests.push(Request::SolveChain(true)),
            Cmd::CheckModel => w.check_model(),
            Cmd::Stop => {
                if w.running() {
                    w.stop();
                } else {
                    w.cancel_meshing();
                }
            }
            Cmd::Monitor => w.analysis_tab = !w.analysis_tab,
            Cmd::Campbell => w.requests.push(Request::Campbell),
            Cmd::NodeSet => w.create(NewItem::MeshSet(plx_model::SetKind::Nodes)),
            Cmd::ElementSet => w.create(NewItem::MeshSet(plx_model::SetKind::Elements)),
            Cmd::Surface => w.create(NewItem::MeshSet(plx_model::SetKind::Surface)),
            Cmd::Material => w.create(NewItem::Material),
            Cmd::MaterialLibrary => w.open_material_library(),
            Cmd::Section => w.create(NewItem::Section),
            Cmd::CrossSection => w.create(NewItem::CrossSection),
            Cmd::ModelProperties => w.edit_model_properties(),
            Cmd::ReferencePoint => w.create(NewItem::Feature(FeatureKind::ReferencePoint)),
            Cmd::CoordinateSystem => w.create(NewItem::Feature(FeatureKind::CoordinateSystem)),
            Cmd::Plane => w.create(NewItem::Feature(FeatureKind::Plane)),
            Cmd::DetectContacts => w.create(NewItem::ContactDetection),
            Cmd::ContactRegion => w.create(NewItem::ContactRegion),
            Cmd::Bolt => w.create(NewItem::Bolt),
            Cmd::Constraint => w.create(NewItem::Constraint),
            Cmd::SurfaceInteraction => w.create(NewItem::SurfaceInteraction),
            Cmd::ContactPair => w.create(NewItem::ContactPair),
            Cmd::Tie => w.create(NewItem::Tie),
            Cmd::NodeTie => w.create(NewItem::NodeTie),
            Cmd::ContactWorksheet => w.contact_worksheet = true,
            Cmd::ConnectionSettings => {
                if let Some(model) = w.setup_model() {
                    w.editor = Editor::edit(&TreeItem::Group("Contacts"), &model.fe, &model.mesh);
                }
            }
            Cmd::Step => w.create(NewItem::Step),
            Cmd::BoundaryCondition => w.create(NewItem::BoundaryCondition(last_step)),
            Cmd::Load => w.create(NewItem::Load(last_step)),
            Cmd::InitialCondition => w.create(NewItem::InitialCondition),
            Cmd::DefinedField => w.create(NewItem::DefinedField(last_step)),
            Cmd::Amplitude => w.create(NewItem::Amplitude),
            Cmd::Distribution => w.create(NewItem::Distribution),
            Cmd::FieldOutput => w.create(NewItem::FieldOutput(last_step)),
            Cmd::HistoryOutput => w.create(NewItem::HistoryOutput(last_step)),
            Cmd::OpenResults => w.open_bound_results(),
            Cmd::RemoveResults => w.confirm_remove_results = true,
            Cmd::ColorScale => w.open_color_scale(),
            Cmd::ShowUndeformed
            | Cmd::GlobalContours
            | Cmd::Transformations
            | Cmd::FirstIncrement
            | Cmd::PreviousIncrement
            | Cmd::NextIncrement
            | Cmd::LastIncrement
            | Cmd::Animate
            | Cmd::ModeSound => w.run_results_command(cmd),
            Cmd::ExportVtu => w.open_vtk_export(plx_io::vtk::VtkFormat::Xml),
            Cmd::ExportVtk => w.open_vtk_export(plx_io::vtk::VtkFormat::Legacy),
            Cmd::ExportFrd => w.export_calculix_results(),
            Cmd::ExportDeformedMesh => w.export_deformed_mesh(),
            Cmd::ExportInp => w.export_inp(),
            Cmd::HotSpot => w.create(NewItem::ResultHotSpot),
            Cmd::Path => w.create(NewItem::Feature(FeatureKind::ResultPath)),
            Cmd::PlaneResult => w.create(NewItem::Feature(FeatureKind::ResultPlane)),
            Cmd::ResultHistoryOutput => w.create(NewItem::ResultHistoryOutput),
            Cmd::DerivedFieldOutput => w.create(NewItem::ResultFieldOutput),
            Cmd::ZoomToFit => w.view_command = Some(ViewCommand::Fit),
            Cmd::View(view) => w.view_command = Some(ViewCommand::View(view)),
            Cmd::AxisView(axis) => w.view_command = Some(ViewCommand::AxisView(axis)),
            Cmd::IsometricAxis(axis) => w.view_command = Some(ViewCommand::IsometricAxis(axis)),
            Cmd::VerticalView => w.view_command = Some(ViewCommand::Vertical),
            Cmd::EdgesOnly => w.viewport.options.mesh_edges = false,
            Cmd::MeshEdges => w.viewport.options.mesh_edges = true,
            Cmd::ShellThickness => {
                let graphics = &mut w.settings.graphics;
                graphics.shell_thickness = !graphics.shell_thickness;
            }
            Cmd::BeamProfiles => {
                let graphics = &mut w.settings.graphics;
                graphics.beam_profiles = !graphics.beam_profiles;
            }
            Cmd::Checklist => {
                // A narrow window hides the checklist until it is asked for.
                let overlay = crate::layout::current(ctx).overlay;
                w.checklist_shown = !w.checklist_visible;
                w.checklist_fold.set(overlay, false);
            }
            Cmd::ShowLinked => {
                let graphics = &mut w.settings.graphics;
                graphics.show_linked = !graphics.show_linked;
            }
            Cmd::SectionView => w.open_section_dialog(),
            Cmd::SectionOff => {
                w.section = None;
                w.section_dialog = None;
            }
            Cmd::ExplodedView => w.open_exploded_dialog(),
            Cmd::ExplodedOff => w.toggle_exploded(),
            Cmd::ScreenshotClipboard => w.screenshot.request(screenshot::Target::Clipboard),
            Cmd::ScreenshotFile => w.screenshot.request(screenshot::Target::File),
            Cmd::Settings => self.open_settings(),
            Cmd::SolverSettings => {
                let workbench = &mut self.workbench;
                let page = crate::settings::Page::Solver;
                workbench.settings_window =
                    Some(SettingsWindow::with_page(&workbench.settings, page));
            }
            Cmd::UserGuide => self.open_user_guide(),
            Cmd::OpenLogFolder => {
                let line = crate::crash_log::open_log_dir();
                self.workbench.output.push(line);
            }
            Cmd::SaveProject => w.requests.push(Request::Save(false)),
            Cmd::SaveProjectAs => w.requests.push(Request::Save(true)),
            Cmd::SaveWithoutResults => w.requests.push(Request::SaveWithoutResults),
            _ => {}
        }
    }
}

/// The ribbon of an analysis asks the analysis about its commands.
pub(super) struct RibbonHost<'a>(pub &'a mut AnalysisWindow);

impl crate::ribbon::Host for RibbonHost<'_> {
    fn enabled(&self, cmd: Cmd) -> bool {
        self.0.command_enabled(cmd)
    }

    fn checked(&self, cmd: Cmd) -> bool {
        self.0.command_checked(cmd)
    }

    fn title(&self, cmd: Cmd) -> Option<&'static str> {
        self.0.workbench.results_title(cmd)
    }

    fn value(&self, cmd: Cmd) -> Option<String> {
        self.0.workbench.results_value(cmd)
    }

    fn panel(&mut self, ui: &mut egui::Ui, cmd: Cmd) {
        self.0.workbench.results_panel(ui, cmd);
    }

    fn custom(&mut self, ui: &mut egui::Ui, cmd: Cmd) {
        if cmd == Cmd::ActiveStep {
            self.0.workbench.active_step_combo(ui);
        }
    }
}
