//! The project window as one surface with four workspaces: the app bar on top, the ribbon
//! of the Project workspace made from the command register, and the active analysis shown
//! in the same window in Geometry, FE Model and Results.

use super::*;
use crate::command::{BlockTile, Cmd, SEARCH_HINT, SEARCH_ROWS, Workspace};
use crate::ribbon::{self, BarEvent};

impl BlockTile {
    /// The kind of block the tile places.
    fn kind(self) -> BlockKind {
        match self {
            Self::Analysis => BlockKind::Analysis,
            Self::Materials => BlockKind::Materials,
            Self::CrossSections => BlockKind::CrossSections,
            Self::BoxMesh => BlockKind::Mesh(MeshPrimitive::default_box()),
            Self::RectangleMesh => BlockKind::Mesh(MeshPrimitive::default_rectangle()),
            Self::RefineMesh => BlockKind::RefineMesh,
            Self::CadOperation => BlockKind::CadOperation,
            Self::Transform => BlockKind::Transform,
            Self::File => BlockKind::File,
            Self::Parameters => BlockKind::Parameters,
            Self::Math => BlockKind::Math,
            Self::Switch => BlockKind::Switch,
            Self::Formula => BlockKind::Formula,
            Self::Measure => BlockKind::Measure,
            Self::Study => BlockKind::Study,
            Self::Campbell => BlockKind::Campbell,
            Self::MeshExport => BlockKind::MeshExport,
            Self::Note => BlockKind::Note,
            Self::Plot => BlockKind::Plot,
            Self::Preview => BlockKind::Preview,
        }
    }
}

impl ProjectApp {
    /// The workspace the window shows.
    pub(super) fn workspace(&self) -> Workspace {
        match self.embedded() {
            Some(id) if !self.in_project => (self.nodes.iter().find(|n| n.id == id))
                .and_then(Node::window)
                .map_or(Workspace::Project, AnalysisWindow::workspace),
            _ => Workspace::Project,
        }
    }

    /// The analysis the project window shows in Geometry, FE Model and Results: the active
    /// one, if its window is open here and not in a window of its own.
    pub(super) fn embedded(&self) -> Option<u64> {
        let id = self.active?;
        let node = self.nodes.iter().find(|n| n.id == id)?;
        let analysis = node.as_analysis()?;
        (analysis.window.is_some() && !analysis.own_window).then_some(id)
    }

    /// Whether the analysis is open in a window of its own. It is shown and steered only
    /// there: it is never the active analysis of the project window.
    pub(super) fn in_own_window(&self, id: u64) -> bool {
        (self.nodes.iter().find(|n| n.id == id))
            .and_then(Node::as_analysis)
            .is_some_and(|a| a.window.is_some() && a.own_window)
    }

    /// Keeps the active analysis one of the project's that is not in a window of its own:
    /// the first such one if it is gone or moved out, none if all are out.
    pub(super) fn check_active(&mut self) {
        let candidates = || {
            (self.nodes.iter()).filter(|n| n.kind() == Kind::Analysis && !self.in_own_window(n.id))
        };
        if self
            .active
            .is_none_or(|id| !candidates().any(|n| n.id == id))
        {
            self.active = candidates().next().map(|n| n.id);
        }
        if self.embedded().is_none() {
            self.in_project = true;
        }
    }

    /// Shows a workspace: Project, or the active analysis in Geometry, FE Model or Results,
    /// opened here first if it is not open yet.
    pub(super) fn show_workspace(&mut self, ctx: &egui::Context, workspace: Workspace) {
        let Some(view) = workspace.view() else {
            self.in_project = true;
            return;
        };
        let Some(id) = self.active.filter(|&id| !self.in_own_window(id)) else {
            return;
        };
        let show = Show { view, step: None };
        let open_here = (self.nodes.iter().find(|n| n.id == id))
            .and_then(Node::as_analysis)
            .is_some_and(|a| a.window.is_some() && !a.own_window);
        if open_here {
            self.in_project = false;
            if let Some(window) = self.node(id).and_then(Node::window_mut) {
                window.set_workspace(workspace);
            }
        } else {
            self.open_analysis(ctx, id, Some(show), false);
        }
    }

    /// The state in the app bar: of the project, or of the analysis shown.
    fn pill(&mut self) -> Option<(CellState, String)> {
        if let Some(id) = self.embedded().filter(|_| !self.in_project) {
            return self.node(id).and_then(Node::window_mut)?.pill();
        }
        // Found again only when an analysis changed: finding the state of an open analysis
        // walks its whole mesh and results, too slow for every frame.
        let states: Vec<Option<CellState>> = (self.nodes.iter_mut())
            .filter(|n| n.kind() == Kind::Analysis)
            .map(|n| n.fresh_status().map(|s| s.solution))
            .collect();
        let stale = (states.iter())
            .filter(|s| **s == Some(CellState::OutOfDate))
            .count();
        let solved = (states.iter())
            .filter(|s| **s == Some(CellState::UpToDate))
            .count();
        let count = match states.len() {
            1 => "1 analysis".to_string(),
            n => format!("{n} analyses"),
        };
        Some(if self.busy() {
            (CellState::Running, format!("{count}, updating …"))
        } else if stale > 0 {
            (
                CellState::OutOfDate,
                format!("{count}, {stale} out of date"),
            )
        } else if solved == states.len() && solved > 0 {
            (CellState::UpToDate, format!("{count}, up to date"))
        } else {
            (CellState::Missing, count)
        })
    }

    /// The app bar of the project window, with the menus of the shown workspace.
    pub(super) fn app_bar(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        let workspace = self.workspace();
        let embedded = self.embedded().filter(|_| workspace != Workspace::Project);
        let analyses: Vec<(u64, String)> = (self.nodes.iter())
            .filter(|n| n.kind() == Kind::Analysis)
            .map(|n| (n.id, n.name.clone()))
            .collect();
        let own_windows: Vec<u64> = (analyses.iter())
            .map(|(id, _)| *id)
            .filter(|&id| self.in_own_window(id))
            .collect();
        let active_name = (self.active)
            .and_then(|id| analyses.iter().find(|(a, _)| *a == id))
            .map_or_else(String::new, |(_, name)| name.clone());
        let (can_undo, can_redo) = match embedded
            .and_then(|id| self.nodes.iter().find(|n| n.id == id))
            .and_then(Node::window)
        {
            Some(window) => (window.can_undo(), window.can_redo()),
            None => (self.history.can_undo(), self.history.can_redo()),
        };
        let bar = ribbon::Bar {
            project: true,
            workspace,
            analyses,
            own_windows,
            active: self.active,
            active_name,
            title: self.project_name(),
            pill: self.pill(),
            update: self.updater.indicator_tip(),
            can_undo,
            can_redo,
        };
        let events = egui::Panel::top("project app bar")
            .exact_size(ribbon::APP_BAR)
            .show(ui, |ui| {
                ribbon::app_bar(ui, &bar, &mut |ui| match embedded {
                    Some(id) => {
                        if let Some(window) = self.node(id).and_then(Node::window_mut) {
                            window.menus(ui, true);
                        }
                    }
                    None => self.menu_bar(ui),
                })
            })
            .inner;
        for event in events {
            match event {
                BarEvent::Workspace(workspace) => self.show_workspace(&ctx, workspace),
                BarEvent::Analysis(id) if !self.in_own_window(id) => {
                    self.active = Some(id);
                    if workspace != Workspace::Project {
                        self.show_workspace(&ctx, workspace);
                    }
                }
                BarEvent::Analysis(_) => {}
                BarEvent::Save => self.save(false),
                BarEvent::Undo | BarEvent::Redo => {
                    let undo = event == BarEvent::Undo;
                    match embedded
                        .and_then(|id| self.node(id))
                        .and_then(Node::window_mut)
                    {
                        Some(window) if undo => window.undo(),
                        Some(window) => window.redo(),
                        None if undo => self.undo(&ctx),
                        None => self.redo(&ctx),
                    }
                }
                BarEvent::Search(at) => {
                    self.search = Some(NodePicker::new(at, at).with_limit(SEARCH_ROWS, SEARCH_HINT))
                }
                BarEvent::Settings => self.run_command(&ctx, Cmd::Settings),
                BarEvent::Help => self.run_command(&ctx, Cmd::UserGuide),
                BarEvent::Update => self.updater.open_window(),
                BarEvent::Pill => match embedded
                    .and_then(|id| self.node(id))
                    .and_then(Node::window_mut)
                {
                    Some(window) => window.pill_clicked(),
                    None => self.monitor.shown = true,
                },
            }
        }
        self.command_search(&ctx);
    }

    /// The command search (Ctrl+K) over the commands of all workspaces.
    fn command_search(&mut self, ctx: &egui::Context) {
        let workspace = self.workspace();
        let Some(search) = &mut self.search else {
            return;
        };
        let entries = crate::command::search_order(true, workspace);
        let list: Vec<node_picker::Entry> = entries
            .iter()
            .map(|e| node_picker::Entry { ..e.entry })
            .collect();
        match search.show(ctx, &list) {
            node_picker::Outcome::Open => {}
            node_picker::Outcome::Closed => self.search = None,
            node_picker::Outcome::Picked(i) => {
                self.search = None;
                let (workspace, cmd) = (entries[i].workspace, entries[i].cmd);
                if workspace == Workspace::Project
                    || matches!(
                        cmd,
                        Cmd::Settings | Cmd::SolverSettings | Cmd::UserGuide | Cmd::OpenLogFolder
                    )
                {
                    if !matches!(
                        cmd,
                        Cmd::Settings | Cmd::SolverSettings | Cmd::UserGuide | Cmd::OpenLogFolder
                    ) {
                        self.in_project = true;
                    }
                    self.ribbon.reveal(Workspace::Project, cmd);
                    self.run_command(ctx, cmd);
                } else if self.active.is_some() {
                    if self.embedded().is_none() || self.in_project {
                        self.show_workspace(ctx, workspace);
                    }
                    if let Some(window) = self
                        .embedded()
                        .and_then(|id| self.node(id))
                        .and_then(Node::window_mut)
                    {
                        window.run_found(ctx, workspace, cmd);
                    }
                }
            }
        }
    }

    /// The ribbon of the Project workspace.
    pub(super) fn project_ribbon(&mut self, ui: &mut egui::Ui) {
        let mut state = std::mem::take(&mut self.ribbon);
        let clicked = egui::Panel::top("project ribbon")
            .frame(egui::Frame::new().fill(crate::style::control()))
            .show(ui, |ui| {
                ribbon::show(ui, &mut state, Workspace::Project, &mut ProjectHost(self))
            })
            .inner;
        self.ribbon = state;
        if let Some(cmd) = clicked {
            let ctx = ui.ctx().clone();
            self.run_command(&ctx, cmd);
        }
    }

    /// The selected analysis, if exactly one block is selected and it is one.
    fn selected_analysis(&self) -> Option<u64> {
        match self.selected[..] {
            [id] => (self.nodes.iter())
                .find(|n| n.id == id && n.kind() == Kind::Analysis)
                .map(|n| n.id),
            _ => None,
        }
    }

    fn command_enabled(&self, cmd: Cmd) -> bool {
        let any = !self.selected.is_empty();
        match cmd {
            Cmd::OpenProject => self.opening.is_none(),
            Cmd::OpenRecent => true,
            Cmd::OpenBlock | Cmd::Rename => self.selected.len() == 1,
            Cmd::OpenInNewWindow => self.selected_analysis().is_some(),
            Cmd::SolveBlock => self.selected_analysis().is_some() && !self.busy(),
            Cmd::Duplicate | Cmd::Delete | Cmd::FrameBlocks | Cmd::Compact | Cmd::GroupBlocks => {
                any
            }
            Cmd::Ungroup => (self.selected.iter()).any(|&id| {
                self.nodes
                    .iter()
                    .any(|n| n.id == id && n.kind() == Kind::Group)
            }),
            Cmd::Unframe => {
                (self.selected.iter()).any(|&id| frames::frame_of(&self.frames, id).is_some())
            }
            Cmd::UpdateProject => !self.busy(),
            Cmd::StopUpdate => self.busy(),
            Cmd::NewProject
            | Cmd::SaveProject
            | Cmd::SaveProjectAs
            | Cmd::SaveWithoutResults
            | Cmd::NewAnalysis
            | Cmd::ProjectMonitor
            | Cmd::Place(_)
            | Cmd::ShowAll
            | Cmd::Minimap
            | Cmd::Settings
            | Cmd::SolverSettings
            | Cmd::UserGuide
            | Cmd::OpenLogFolder => true,
            Cmd::CheckForUpdates => !self.updater.busy(),
            _ => false,
        }
    }

    /// Carries out a command of the Project workspace, or one of every workspace such as
    /// Settings.
    pub(super) fn run_command(&mut self, ctx: &egui::Context, cmd: Cmd) {
        if !self.command_enabled(cmd) {
            return;
        }
        let selected = self.selected.clone();
        let first = selected.first().copied();
        match cmd {
            Cmd::NewProject => self.new_project(),
            Cmd::OpenProject => self.open_dialog(ctx),
            Cmd::SaveProject => self.save(false),
            Cmd::SaveProjectAs => self.save(true),
            Cmd::SaveWithoutResults => self.save_without_results(),
            Cmd::NewAnalysis => self.placing = Some(BlockTile::Analysis),
            Cmd::Place(block) => {
                self.placing = (self.placing != Some(block)).then_some(block);
            }
            Cmd::OpenBlock => {
                if let Some(id) = first {
                    self.act(ctx, Action::Open(id, None));
                }
            }
            Cmd::OpenInNewWindow => {
                if let Some(id) = first {
                    self.act(ctx, Action::OpenInWindow(id));
                }
            }
            Cmd::SolveBlock => {
                if let Some(id) = first {
                    self.act(ctx, Action::Solve(id));
                }
            }
            Cmd::Rename => {
                if let Some(id) = first {
                    self.act(ctx, Action::Rename(id));
                }
            }
            Cmd::Duplicate => self.act(ctx, Action::DuplicateAll(selected)),
            Cmd::Delete => self.act(ctx, Action::DeleteAll(selected)),
            Cmd::FrameBlocks => self.act(ctx, Action::Frame(selected)),
            Cmd::GroupBlocks => self.act(ctx, Action::Group(selected)),
            Cmd::Ungroup => self.act(ctx, Action::Ungroup(selected)),
            Cmd::Unframe => self.act(ctx, Action::Unframe(selected)),
            Cmd::Compact => self.act(ctx, Action::Compact(selected)),
            Cmd::ShowAll => self.show_all = true,
            Cmd::Minimap => self.show_minimap = !self.show_minimap,
            Cmd::UpdateProject => self.update_all(),
            Cmd::StopUpdate => self.stop_update(),
            Cmd::ProjectMonitor => self.toggle_monitor(ctx),
            Cmd::Settings => self.settings_window = Some(SettingsWindow::new(&self.settings)),
            Cmd::SolverSettings => {
                let page = crate::settings::Page::Solver;
                self.settings_window = Some(SettingsWindow::with_page(&self.settings, page));
            }
            Cmd::UserGuide => {
                let line = crate::help::open_user_guide();
                self.output.push(line);
            }
            Cmd::OpenLogFolder => self.output.push(crate::crash_log::open_log_dir()),
            Cmd::CheckForUpdates => self.updater.check(ctx, true),
            _ => {}
        }
    }

    /// The analysis shown in this window, below the app bar: its ribbon and workspace.
    pub(super) fn embedded_ui(&mut self, ui: &mut egui::Ui, id: u64) {
        let solve = self.solves.iter().rev().find(|s| s.id == id);
        let node = self.nodes.iter_mut().find(|n| n.id == id);
        if let Some(window) = node.and_then(Node::window_mut) {
            if let Some(solve) = solve {
                window.show_run(solve);
            }
            window.ui_embedded(ui);
        }
    }

    /// While a block hangs at the pointer: draws it there, places it on a click into the
    /// schematic and drops it on Esc.
    pub(super) fn place_block(
        &mut self,
        ui: &egui::Ui,
        to_graph: egui::emath::TSTransform,
        actions: &mut Vec<Action>,
    ) {
        let Some(block) = self.placing else {
            return;
        };
        let ctx = ui.ctx();
        if ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Escape)) {
            self.placing = None;
            return;
        }
        let area = ui.max_rect();
        let Some(pointer) = ctx
            .input(|i| i.pointer.hover_pos())
            .filter(|p| area.contains(*p))
        else {
            return;
        };
        let kind = block.kind();
        let painter = ctx.layer_painter(egui::LayerId::new(
            egui::Order::Foreground,
            egui::Id::new("placed block"),
        ));
        let rect = egui::Rect::from_min_size(pointer, egui::vec2(180.0, 56.0));
        let fill = crate::style::window().gamma_multiply(0.7);
        painter.rect(
            rect,
            6.0,
            fill,
            egui::Stroke::new(1.0, crate::style::highlight()),
            egui::StrokeKind::Inside,
        );
        let header = egui::Rect::from_min_size(rect.min, egui::vec2(rect.width(), 22.0));
        painter.rect_filled(
            header,
            egui::CornerRadius {
                nw: 6,
                ne: 6,
                sw: 0,
                se: 0,
            },
            crate::style::node_header().gamma_multiply(0.8),
        );
        painter.text(
            header.left_center() + egui::vec2(8.0, 0.0),
            egui::Align2::LEFT_CENTER,
            kind.entry().label,
            egui::FontId::proportional(13.0),
            crate::style::text(),
        );
        painter.text(
            rect.left_bottom() + egui::vec2(8.0, -10.0),
            egui::Align2::LEFT_CENTER,
            "Click to place, Esc cancels",
            egui::FontId::proportional(11.0),
            ui.visuals().weak_text_color(),
        );
        ctx.set_cursor_icon(egui::CursorIcon::Copy);
        if ctx.input(|i| i.pointer.primary_clicked()) {
            actions.push(kind.action(to_graph * pointer));
            self.placing = None;
        }
        ctx.request_repaint();
    }
}

/// The ribbon of the Project workspace asks the project about its commands.
pub(super) struct ProjectHost<'a>(pub(super) &'a mut ProjectApp);

impl ribbon::Host for ProjectHost<'_> {
    fn enabled(&self, cmd: Cmd) -> bool {
        self.0.command_enabled(cmd)
    }

    fn checked(&self, cmd: Cmd) -> bool {
        match cmd {
            Cmd::Minimap => self.0.show_minimap,
            Cmd::ProjectMonitor => self.0.monitor.shown,
            Cmd::Place(block) => self.0.placing == Some(block),
            Cmd::NewAnalysis => self.0.placing == Some(BlockTile::Analysis),
            _ => false,
        }
    }

    fn panel(&mut self, ui: &mut egui::Ui, cmd: Cmd) {
        if cmd == Cmd::OpenRecent
            && let Some(path) = self.0.recent_panel(ui)
        {
            self.0.open(&ui.ctx().clone(), path);
        }
    }
}
