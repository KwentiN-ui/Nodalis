//! User settings, grouped as the settings dialog and stored by eframe in the user's
//! data directory (Linux `~/.local/share/nodalis`, Windows `%APPDATA%\nodalis\data`).

use crate::numeric;
use serde::{Deserialize, Serialize};

/// Key of the settings in eframe's storage.
pub const STORAGE_KEY: &str = "settings";

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    pub graphics: Graphics,
    pub post: PostProcessing,
    pub solver: Solver,
    pub gmsh: Gmsh,
    pub updates: Updates,
    /// Model space and unit system last chosen for a new model, proposed for the next one.
    pub new_model: plx_model::ModelProperties,
    /// Version of the stored settings, for [`Settings::upgrade`]; missing (0) in those of
    /// versions before it existed.
    #[serde(default)]
    pub format: u32,
}

/// Label and tooltip of [`Graphics::shell_thickness`], on the View tab.
pub const SHELL_THICKNESS_TIP: &str = "Draws shell and membrane elements of the FE model and the results as solids of their section's thickness, placed by its offset.";

/// Label and tooltip of [`Graphics::beam_profiles`], on the View tab.
pub const BEAM_PROFILES_TIP: &str = "Draws beam elements as their profile swept along the line, turned by the section's orientation and placed by its offsets, and trusses as round bars of their area, in the FE model and the results.";

/// Label and tooltip of [`Graphics::show_linked`], on the View tab and above the tree.
pub const SHOW_LINKED_LABEL: &str = "Show linked items";
pub const SHOW_LINKED_TIP: &str = "Lists the items of the FE Model tree that this analysis takes from another one. They stay in the model; hidden, the tree shows only what is set up here.";

/// [`Settings::format`] of this version.
const FORMAT: u32 = 2;

impl Default for Settings {
    fn default() -> Self {
        Self {
            graphics: Graphics::default(),
            post: PostProcessing::default(),
            solver: Solver::default(),
            gmsh: Gmsh::default(),
            updates: Updates::default(),
            new_model: plx_model::ModelProperties::default(),
            format: FORMAT,
        }
    }
}

impl Settings {
    /// Brings settings stored by an older version up to date.
    pub fn upgrade(&mut self) {
        if self.format < 2 && self.solver.threads == 1 {
            // The old default of one thread: now all cores.
            self.solver.threads = AUTOMATIC_THREADS;
        }
        self.format = FORMAT;
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Graphics {
    /// Light or dark colours, or as the operating system is set.
    pub theme: crate::style::ThemeChoice,
    /// Axis triad at the global origin of the model.
    pub global_axes: bool,
    /// Axis triad in the top right corner showing the view direction.
    pub view_triad: bool,
    pub scale_bar: bool,
    /// Shell and membrane elements of the FE model drawn as the solid of their section's
    /// thickness and offset rather than as surfaces.
    pub shell_thickness: bool,
    /// Beams and trusses drawn with their profile rather than as lines.
    pub beam_profiles: bool,
    /// The FE Model tree lists the items an analysis takes from another one; off, it shows
    /// only its own items.
    pub show_linked: bool,
    /// The timeline of the steps and increments below the 3D view of results.
    pub timeline: bool,
}

impl Default for Graphics {
    fn default() -> Self {
        Self {
            theme: crate::style::ThemeChoice::System,
            global_axes: true,
            view_triad: true,
            scale_bar: true,
            shell_thickness: true,
            beam_profiles: true,
            show_linked: false,
            timeline: true,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PostProcessing {
    /// Label at the node with the largest value (default: on).
    pub max_label: bool,
    /// Label at the node with the smallest value (default: off).
    pub min_label: bool,
    /// Information block with file, step and deformation.
    pub status_block: bool,
    /// Outline of the undeformed shape behind deformed results, for newly opened results.
    pub undeformed_outline: bool,
    /// Colour bands of the legend for newly opened results.
    pub levels: u32,
    /// Significant digits of result values in legend, labels, query and tables.
    pub digits: u32,
    /// FFmpeg for exporting animations as MP4 or MKV; empty for `ffmpeg` on the `PATH`.
    pub ffmpeg: String,
}

impl Default for PostProcessing {
    fn default() -> Self {
        Self {
            max_label: true,
            min_label: false,
            status_block: true,
            undeformed_outline: true,
            levels: plx_render::contour::DEFAULT_LEVELS,
            digits: 4,
            ffmpeg: String::new(),
        }
    }
}

/// How the solver runs: the "Solver" settings page.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Solver {
    /// Threads for the solver: the size of the one thread pool all analyses share
    /// ([`plx_solve::pool`]); [`AUTOMATIC_THREADS`] for all cores.
    pub threads: u32,
    /// Where analyses run; empty for the default, see [`default_work_dir`].
    pub work_dir: String,
    /// Most analyses Update Project solves at the same time.
    pub parallel_runs: u32,
    /// Whether the iterative solver may use a discrete graphics card (it decides by
    /// itself when, see [`plxs_linalg::GpuMode::Automatic`]).
    pub gpu: bool,
}

/// [`Solver::threads`] for as many threads as the computer has cores.
pub const AUTOMATIC_THREADS: u32 = 0;

/// Cores of this computer, at least one.
pub fn cores() -> u32 {
    std::thread::available_parallelism().map_or(1, |n| n.get() as u32)
}

impl Default for Solver {
    fn default() -> Self {
        Self {
            threads: AUTOMATIC_THREADS,
            work_dir: String::new(),
            parallel_runs: 2,
            gpu: true,
        }
    }
}

impl Solver {
    /// Hands [`Self::gpu`] to the solver, for the analyses started from now on.
    pub fn apply_gpu(&self) {
        plxs_linalg::GpuMode::set_preferred(if self.gpu {
            plxs_linalg::GpuMode::Automatic
        } else {
            plxs_linalg::GpuMode::Off
        });
    }

    pub fn work_dir(&self) -> std::path::PathBuf {
        match clean_path(&self.work_dir) {
            "" => default_work_dir(),
            dir => std::path::PathBuf::from(dir),
        }
    }
}

/// Automatic threads, or the number chosen.
fn threads_ui(ui: &mut egui::Ui, threads: &mut u32) {
    ui.horizontal(|ui| {
        let mut automatic = *threads == AUTOMATIC_THREADS;
        let tip = format!(
            "Uses all {} cores of this computer. Analyses solved at the same time share them: the solver hands them out as the analyses need them.",
            cores()
        );
        if ui
            .checkbox(&mut automatic, "Automatic")
            .on_hover_text(tip)
            .changed()
        {
            *threads = if automatic {
                AUTOMATIC_THREADS
            } else {
                cores()
            };
        }
        if automatic {
            ui.weak(format!("{} cores", cores()));
        } else {
            ui.add(numeric::drag_value(threads).range(1..=256));
        }
    });
}

/// The graphics cards the solver finds, detected once on a thread of its own (a driver
/// call that can take a moment) when the solver page is first shown.
fn gpu_detection(ctx: &egui::Context) -> Option<&'static plxs_linalg::gpu::Detection> {
    static DETECTION: std::sync::OnceLock<plxs_linalg::gpu::Detection> = std::sync::OnceLock::new();
    static STARTED: std::sync::Once = std::sync::Once::new();
    STARTED.call_once(|| {
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let _ = DETECTION.set(plxs_linalg::gpu::detect());
            ctx.request_repaint();
        });
    });
    DETECTION.get()
}

/// The switch for the graphics card and what the solver found.
fn gpu_ui(ui: &mut egui::Ui, gpu: &mut bool) {
    ui.vertical(|ui| {
        ui.checkbox(gpu, "Use for the iterative solver").on_hover_text(format!(
            "Iterative solves of {} equations and more run on a discrete graphics card, which \
             has many times the memory bandwidth of the processor. The solver takes at most 60 % \
             of the card's memory and falls back to the processor when the model does not fit. \
             Integrated graphics share the processor's memory and are not used. Applies to \
             analyses started from now on.",
            plxs_linalg::gpu::GPU_FROM
        ));
        match gpu_detection(ui.ctx()) {
            None => {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.weak("Detecting graphics cards …");
                });
            }
            Some(found) => match &found.chosen {
                Some(card) => {
                    let backend = found.backend.as_deref().unwrap_or("");
                    let precision = if found.double_precision {
                        "conjugate gradients and multigrid cycle on the card"
                    } else {
                        "multigrid cycle on the card, conjugate gradients on the processor (no double precision)"
                    };
                    ui.label(format!("{card}, {backend}"));
                    ui.weak(precision);
                }
                None => {
                    let found_text = if found.adapters.is_empty() {
                        "no graphics adapter found".to_owned()
                    } else {
                        format!("found: {}", found.adapters.join(", "))
                    };
                    ui.weak(format!(
                        "No discrete graphics card ({found_text}); the solver runs on the processor."
                    ));
                }
            },
        }
    });
}

/// Where the Gmsh library comes from, for geometry import and meshing.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Gmsh {
    /// The library file; empty to search next to the program and on the system.
    pub library: String,
}

impl Gmsh {
    pub fn library(&self) -> Option<std::path::PathBuf> {
        Some(clean_path(&self.library))
            .filter(|p| !p.is_empty())
            .map(std::path::PathBuf::from)
    }
}

/// The check for a new release on GitHub, see [`crate::update`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Updates {
    /// Ask GitHub for a new release when Nodalis starts.
    pub check_at_startup: bool,
}

impl Default for Updates {
    fn default() -> Self {
        Self {
            check_at_startup: true,
        }
    }
}

/// A path as typed or pasted: without surrounding spaces and the quotes that Windows'
/// "Als Pfad kopieren" adds.
pub fn clean_path(text: &str) -> &str {
    let text = text.trim();
    ['"', '\'']
        .iter()
        .find_map(|&quote| text.strip_prefix(quote)?.strip_suffix(quote))
        .map_or(text, str::trim)
}

/// The default work directory: a `Temp` folder next to the program. Where that is
/// not writable, e.g. for a binary installed to `/usr/bin`, a `nodalis` folder in the
/// system's temporary directory.
pub fn default_work_dir() -> std::path::PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|exe| Some(exe.parent()?.join("Temp")))
        .filter(|dir| is_writable(dir))
        .unwrap_or_else(|| std::env::temp_dir().join("nodalis"))
}

/// Creates the directory if needed and checks that files can be written into it.
fn is_writable(dir: &std::path::Path) -> bool {
    let probe = dir.join(".nodalis-write-test");
    let writable = std::fs::create_dir_all(dir).is_ok() && std::fs::write(&probe, b"").is_ok();
    let _ = std::fs::remove_file(probe);
    writable
}

/// Pages of the settings window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Page {
    Graphics,
    PostProcessing,
    Solver,
    Gmsh,
    Updates,
}

impl Page {
    const ALL: [Page; 5] = [
        Page::Graphics,
        Page::PostProcessing,
        Page::Solver,
        Page::Gmsh,
        Page::Updates,
    ];

    fn title(self) -> &'static str {
        match self {
            Page::Graphics => "Graphics",
            Page::PostProcessing => "Post-processing",
            Page::Solver => "Solver",
            Page::Gmsh => "Gmsh",
            Page::Updates => "Updates",
        }
    }
}

/// The settings window while it is open: a draft that OK or Apply copies into the settings.
pub struct SettingsWindow {
    page: Page,
    draft: Settings,
    default_work_dir: String,
    solver_check: SolverCheck,
    gmsh_check: GmshCheck,
    installation: crate::update::Installation,
}

/// State of the Gmsh test on the settings page.
#[derive(Default)]
enum GmshCheck {
    #[default]
    NotRun,
    Running(std::sync::mpsc::Receiver<Result<String, String>>),
    Done(Result<String, String>),
}

/// State of the solver self test on the settings page.
#[derive(Default)]
enum SolverCheck {
    #[default]
    NotRun,
    Running(
        std::sync::mpsc::Receiver<crate::solver_check::Report>,
        crate::solver_check::SharedProgress,
    ),
    Done(Vec<crate::solver_check::CheckResult>),
}

/// What the user decided in the settings window.
pub enum WindowResult {
    Open,
    /// Take over the draft and keep the window open.
    Apply(Settings),
    /// Take over the draft and close.
    Ok(Settings),
    Cancel,
}

impl SettingsWindow {
    pub fn new(settings: &Settings) -> Self {
        Self {
            page: Page::PostProcessing,
            draft: settings.clone(),
            default_work_dir: default_work_dir().display().to_string(),
            solver_check: SolverCheck::NotRun,
            gmsh_check: GmshCheck::NotRun,
            installation: crate::update::Installation::detect(),
        }
    }

    /// The window opened at `page`.
    pub fn with_page(settings: &Settings, page: Page) -> Self {
        Self {
            page,
            ..Self::new(settings)
        }
    }

    pub fn show(&mut self, ctx: &egui::Context) -> WindowResult {
        let mut open = true;
        let mut result = WindowResult::Open;
        egui::Window::new("Settings")
            .id(crate::ids::window(ctx, "settings"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                ui.horizontal_top(|ui| {
                    egui::Frame::new()
                        .fill(crate::style::window())
                        .stroke(egui::Stroke::new(1.0, crate::style::border()))
                        .inner_margin(4)
                        .show(ui, |ui| {
                            ui.set_min_size(egui::vec2(130.0, 220.0));
                            ui.vertical(|ui| {
                                for page in Page::ALL {
                                    if ui
                                        .selectable_label(self.page == page, page.title())
                                        .clicked()
                                    {
                                        self.page = page;
                                    }
                                }
                            });
                        });
                    ui.add_space(8.0);
                    ui.vertical(|ui| {
                        ui.set_min_width(300.0);
                        ui.heading(self.page.title());
                        ui.add_space(4.0);
                        self.page_ui(ui);
                    });
                });
                if crate::portable::is_portable()
                    && let Some(dir) = crate::portable::data_dir()
                {
                    ui.add_space(4.0);
                    ui.weak(format!(
                        "Portable mode: settings are kept in {}",
                        dir.display()
                    ));
                }
                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("Defaults").clicked() {
                        self.draft = Settings::default();
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("Apply").clicked() {
                            result = WindowResult::Apply(self.draft.clone());
                        }
                        if ui.button("Cancel").clicked() {
                            result = WindowResult::Cancel;
                        }
                        if ui.button("OK").clicked() {
                            result = WindowResult::Ok(self.draft.clone());
                        }
                    });
                });
            });
        if !open {
            result = WindowResult::Cancel;
        }
        result
    }

    fn page_ui(&mut self, ui: &mut egui::Ui) {
        match self.page {
            Page::Graphics => {
                let g = &mut self.draft.graphics;
                ui.horizontal(|ui| {
                    ui.label("Color theme");
                    egui::ComboBox::from_id_salt("color theme")
                        .selected_text(g.theme.label())
                        .show_ui(ui, |ui| {
                            for choice in crate::style::ThemeChoice::ALL {
                                ui.selectable_value(&mut g.theme, choice, choice.label());
                            }
                        });
                })
                .response
                .on_hover_text("System follows the light or dark mode of the operating system.");
                ui.add_space(4.0);
                ui.checkbox(&mut g.global_axes, "Global axes at the origin");
                ui.checkbox(
                    &mut g.view_triad,
                    "Coordinate triad in the corner (view direction)",
                );
                ui.checkbox(&mut g.scale_bar, "Scale bar");
                ui.weak("Shell thickness, beam profiles and linked items are set on the View tab.");
            }
            Page::PostProcessing => {
                let p = &mut self.draft.post;
                ui.checkbox(&mut p.max_label, "Show label at maximum");
                ui.checkbox(&mut p.min_label, "Show label at minimum");
                ui.checkbox(&mut p.status_block, "Show status block");
                ui.checkbox(
                    &mut p.undeformed_outline,
                    "Show undeformed outline (newly opened results)",
                );
                ui.horizontal(|ui| {
                    ui.label("Color levels (newly opened results)");
                    ui.add(
                        numeric::drag_value(&mut p.levels)
                            .range(2..=plx_render::contour::MAX_LEVELS),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Significant digits of result values");
                    ui.add(numeric::drag_value(&mut p.digits).range(crate::results::DIGITS_RANGE))
                        .on_hover_text(
                            "Digits shown in the legend, labels, Query, tables and plots. \
                         The stored values are not rounded.",
                        );
                });
                ui.horizontal(|ui| {
                    ui.label("FFmpeg executable");
                    ui.add(
                        egui::TextEdit::singleline(&mut p.ffmpeg)
                            .hint_text("ffmpeg")
                            .desired_width(200.0),
                    )
                    .on_hover_text(
                        "Writes animations as MP4 or MKV. Leave empty for ffmpeg on the PATH.",
                    );
                });
            }
            Page::Solver => {
                let solver = &mut self.draft.solver;
                egui::Grid::new("solver settings")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| {
                        ui.label("Threads");
                        threads_ui(ui, &mut solver.threads);
                        ui.end_row();
                        ui.label("Work directory");
                        ui.add(
                            egui::TextEdit::singleline(&mut solver.work_dir)
                                .hint_text(self.default_work_dir.as_str()),
                        );
                        ui.end_row();
                        ui.label("Parallel analyses");
                        ui.add(numeric::drag_value(&mut solver.parallel_runs).range(1..=64))
                            .on_hover_text("Update Project solves up to this many independent analyses at the same time; the Campbell sweep runs as many speeds at a time. The solver runs them in one pool of the threads above, so a single large analysis gets all of them. Each analysis needs its own memory.");
                        ui.end_row();
                        ui.label("Graphics card");
                        gpu_ui(ui, &mut solver.gpu);
                        ui.end_row();
                    });
                ui.add_space(8.0);
                self.solver_check_ui(ui);
            }
            Page::Gmsh => self.gmsh_ui(ui),
            Page::Updates => {
                let updates = &mut self.draft.updates;
                ui.checkbox(&mut updates.check_at_startup, "Check for updates at startup")
                    .on_hover_text(
                        "Asks GitHub for the latest release when Nodalis starts and shows a symbol next to the version if there is a newer one. Help > Check for Updates asks at any time.",
                    );
                ui.add_space(4.0);
                ui.add(egui::Label::new(self.installation.description()).wrap());
            }
        }
    }

    /// Gmsh imports STEP, IGES and BREP and meshes; nodalis loads its library at run time.
    fn gmsh_ui(&mut self, ui: &mut egui::Ui) {
        let loaded = plx_mesher::loaded_library();
        egui::Grid::new("gmsh settings")
            .num_columns(2)
            .spacing([12.0, 6.0])
            .show(ui, |ui| {
                ui.label("Library");
                ui.add(
                    egui::TextEdit::singleline(&mut self.draft.gmsh.library)
                        .hint_text("search automatically")
                        .desired_width(260.0),
                );
                ui.end_row();
                ui.label("Loaded");
                match &loaded {
                    Some(info) => {
                        ui.label(format!("Gmsh {}\n{}", info.version, info.path.display()))
                    }
                    None => ui.weak("not yet"),
                };
                ui.end_row();
            });
        ui.add(
            egui::Label::new(
                "If empty, the library is searched next to the executable (libgmsh.so or \
                 gmsh-4.15.dll), then in the system search path. Another library is only \
                 loaded after a restart.",
            )
            .wrap(),
        );
        ui.add_space(8.0);
        if let GmshCheck::Running(receiver) = &self.gmsh_check
            && let Ok(result) = receiver.try_recv()
        {
            self.gmsh_check = GmshCheck::Done(result);
        }
        let running = matches!(self.gmsh_check, GmshCheck::Running(_));
        ui.horizontal(|ui| {
            let button = ui
                .add_enabled(!running, egui::Button::new("Test Gmsh"))
                .on_hover_text("Loads Gmsh and meshes a cube.");
            if button.clicked() {
                if loaded.is_none() {
                    plx_mesher::set_library_path(self.draft.gmsh.library());
                }
                let (sender, receiver) = std::sync::mpsc::channel();
                let ctx = ui.ctx().clone();
                std::thread::spawn(move || {
                    let result = plx_mesher::self_test()
                        .map(|(info, elements)| {
                            let mut text = format!(
                                "Gmsh {} works correctly, cube meshed with {elements} elements.",
                                info.version
                            );
                            if info.untested {
                                let (major, minor) = plx_mesher::gmsh::TESTED_VERSION;
                                text += &format!(" Nodalis is tested with Gmsh {major}.{minor}.");
                            }
                            text
                        })
                        .map_err(|e| e.to_string());
                    let _ = sender.send(result);
                    ctx.request_repaint();
                });
                self.gmsh_check = GmshCheck::Running(receiver);
            }
            if running {
                ui.spinner();
                ui.label("Test running …");
            }
        });
        match &self.gmsh_check {
            GmshCheck::Done(Ok(text)) => {
                ui.colored_label(crate::style::ok(), text);
            }
            GmshCheck::Done(Err(text)) => {
                ui.add(
                    egui::Label::new(egui::RichText::new(text).color(crate::style::error())).wrap(),
                );
            }
            _ => {}
        }
    }

    /// Button and results of the solver self test, run with the settings in the window.
    fn solver_check_ui(&mut self, ui: &mut egui::Ui) {
        if let SolverCheck::Running(receiver, _) = &self.solver_check
            && let Ok(report) = receiver.try_recv()
        {
            self.solver_check = SolverCheck::Done(report.checks);
        }
        let running = matches!(self.solver_check, SolverCheck::Running(..));
        ui.horizontal(|ui| {
            let button = ui
                .add_enabled(!running, egui::Button::new("Test solver"))
                .on_hover_text(
                    "Solves small cantilevers, direct and iterative, and compares the results with beam theory.",
                );
            if button.clicked() {
                let (sender, receiver) = std::sync::mpsc::channel();
                let (threads, work_dir) = (self.draft.solver.threads, self.draft.solver.work_dir());
                let ctx = ui.ctx().clone();
                let progress = crate::solver_check::SharedProgress::default();
                let thread_progress = progress.clone();
                std::thread::spawn(move || {
                    let report = crate::solver_check::run(threads, &work_dir, &thread_progress);
                    let _ = sender.send(report);
                    ctx.request_repaint();
                });
                self.solver_check = SolverCheck::Running(receiver, progress);
            }
            if running {
                ui.spinner();
                ui.label("Test running …");
            }
        });
        if let SolverCheck::Running(_, progress) = &self.solver_check {
            let progress = progress.lock().map(|p| p.clone()).unwrap_or_default();
            running_modal(ui.ctx(), &progress);
        }
        if let SolverCheck::Done(results) = &self.solver_check {
            egui::Grid::new("solver check")
                .num_columns(2)
                .spacing([8.0, 4.0])
                .show(ui, |ui| {
                    for result in results {
                        if result.passed {
                            ui.colored_label(crate::style::ok(), "OK");
                        } else {
                            ui.colored_label(crate::style::error(), "Error");
                        }
                        ui.vertical(|ui| {
                            ui.strong(result.name);
                            ui.add(egui::Label::new(&result.message).wrap());
                        });
                        ui.end_row();
                    }
                });
            if results.iter().all(|r| r.passed) {
                ui.label("The solver works correctly.");
            }
        }
    }
}

/// Modal window over the application while the solver self test runs, which takes a while
/// on some systems.
fn running_modal(ctx: &egui::Context, progress: &crate::solver_check::Progress) {
    egui::Modal::new(crate::ids::window(ctx, "solver check running")).show(ctx, |ui| {
        ui.set_width(300.0);
        ui.heading("Testing the solver");
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            ui.spinner();
            ui.label("The test is running, please wait.");
        });
        ui.add_space(6.0);
        let step = match progress.count {
            Some((done, total)) => format!("{} ({} of {total})", progress.step, done + 1),
            None => progress.step.clone(),
        };
        ui.label(step);
        if let Some((done, total)) = progress.count {
            ui.add(egui::ProgressBar::new(done as f32 / total.max(1) as f32));
        }
    });
    // The test thread cannot wake the window for each step, so keep it fresh.
    ctx.request_repaint_after(std::time::Duration::from_millis(100));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_fields_take_their_defaults() {
        // Settings files of older versions lack newer fields.
        let settings: Settings = ron::from_str("(post: (min_label: true))").unwrap();
        assert!(settings.post.min_label);
        assert!(settings.post.max_label);
        assert!(settings.graphics.global_axes);
        assert!(settings.graphics.shell_thickness);
        assert!(settings.updates.check_at_startup);
    }

    #[test]
    fn paths_lose_quotes_and_spaces() {
        assert_eq!(
            clean_path(r#" "C:\Program Files\Temp" "#),
            r"C:\Program Files\Temp"
        );
        assert_eq!(clean_path("'/opt/work' "), "/opt/work");
        assert_eq!(clean_path("work"), "work");
        assert_eq!(clean_path("\"work"), "\"work");
        let solver = Solver {
            work_dir: " \"/tmp/plx\" ".into(),
            ..Solver::default()
        };
        assert_eq!(solver.work_dir(), std::path::Path::new("/tmp/plx"));
    }

    #[test]
    fn automatic_threads_are_the_default() {
        assert_eq!(Solver::default().threads, AUTOMATIC_THREADS);
        // The old default of one thread becomes automatic, a later choice of one stays.
        let mut old: Settings = ron::from_str("(format: 1, solver: (threads: 1))").unwrap();
        old.upgrade();
        assert_eq!(old.solver.threads, AUTOMATIC_THREADS);
        let mut one = Settings::default();
        one.solver.threads = 1;
        let mut one: Settings = ron::from_str(&ron::to_string(&one).unwrap()).unwrap();
        one.upgrade();
        assert_eq!(one.solver.threads, 1);
    }

    #[test]
    fn default_work_dir_is_next_to_the_binary_if_writable() {
        let dir = default_work_dir();
        let next_to_binary = std::env::current_exe()
            .unwrap()
            .parent()
            .unwrap()
            .join("Temp");
        assert_eq!(dir, next_to_binary, "the test binary's folder is writable");
    }
}
