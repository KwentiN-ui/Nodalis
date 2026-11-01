//! User settings, grouped like PrePoMax's settings dialog and stored by eframe in the user's
//! data directory (Linux `~/.local/share/prepolix`, Windows `%APPDATA%\prepolix\data`).

use crate::numeric;
pub use plx_solve::Engine;
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

/// Label and tooltip of [`Graphics::shell_thickness`], in the settings and the View menu.
pub const SHELL_THICKNESS_LABEL: &str = "Shell thickness";
pub const SHELL_THICKNESS_TIP: &str = "Draws shell and membrane elements of the FE model and the results as solids of their section's thickness, placed by its offset.";

/// Label and tooltip of [`Graphics::beam_profiles`], in the settings and the View menu.
pub const BEAM_PROFILES_LABEL: &str = "Beam profiles";
pub const BEAM_PROFILES_TIP: &str = "Draws beam elements as their profile swept along the line, turned by the section's orientation and placed by its offsets, and trusses as round bars of their area, in the FE model and the results.";

/// Label and tooltip of [`Graphics::show_linked`], in the settings, the View menu and the tree.
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
        if self.format < 1 {
            self.solver.forget_old_default();
        }
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
    /// Axis triad in the bottom right corner showing the view direction.
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
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PostProcessing {
    /// Label at the node with the largest value (PrePoMax: on).
    pub max_label: bool,
    /// Label at the node with the smallest value (PrePoMax: off).
    pub min_label: bool,
    /// Information block with file, step and deformation.
    pub status_block: bool,
    /// Outline of the undeformed shape behind deformed results, for newly opened results.
    pub undeformed_outline: bool,
    /// Colour bands of the legend for newly opened results.
    pub levels: u32,
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
            ffmpeg: String::new(),
        }
    }
}

/// Which solver computes the analyses and how it is run: the "Solver" settings page.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Solver {
    /// The built-in PrePoLix solver or CalculiX as a process.
    pub engine: Engine,
    /// The CalculiX executable; a bare name is looked up on `PATH`, empty for the one shipped
    /// with prepolix (see [`bundled_ccx`]).
    pub executable: String,
    /// Threads for the solver: the size of the one thread pool all analyses of the built-in
    /// solver share ([`plx_solve::pool`]), or `OMP_NUM_THREADS` of each CalculiX process;
    /// [`AUTOMATIC_THREADS`] for all cores.
    pub threads: u32,
    /// Where analyses run; empty for the default, see [`default_work_dir`].
    pub work_dir: String,
    /// Keywords tested on the executable; tested again when it changes.
    pub keywords: Option<DetectedKeywords>,
    /// CalculiX writes the state after each step, so that a run after a change solves only
    /// the steps from the first changed one; see [`plx_job::restart`].
    pub keep_restart_files: bool,
    /// Most the restart files of a project may take, in MB; the oldest are removed beyond.
    pub restart_limit_mb: u64,
    /// Most analyses Update Project solves at the same time.
    pub parallel_runs: u32,
}

/// [`Solver::threads`] for as many threads as the computer has cores.
pub const AUTOMATIC_THREADS: u32 = 0;

/// Cores of this computer, at least one.
pub fn cores() -> u32 {
    std::thread::available_parallelism().map_or(1, |n| n.get() as u32)
}

/// Result of [`crate::solver_check::known_keywords`] for one executable, by normalized
/// keyword, collected over the self test and the analyses run with it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DetectedKeywords {
    pub executable: String,
    /// The version the executable reported, so that an update is tested again.
    pub version: Option<String>,
    pub known: std::collections::BTreeMap<String, bool>,
}

impl Default for Solver {
    fn default() -> Self {
        Self {
            engine: Engine::default(),
            executable: String::new(),
            threads: AUTOMATIC_THREADS,
            work_dir: String::new(),
            keywords: None,
            keep_restart_files: true,
            restart_limit_mb: 4096,
            parallel_runs: 2,
        }
    }
}

impl Solver {
    pub fn work_dir(&self) -> std::path::PathBuf {
        match clean_path(&self.work_dir) {
            "" => default_work_dir(),
            dir => std::path::PathBuf::from(dir),
        }
    }

    /// Those of `keywords` the executable does not know. Keywords not tested on it yet are
    /// tested now, a few hundredths of a second each.
    pub fn unknown_keywords(&mut self, keywords: &[String]) -> Vec<String> {
        use plx_io::inp::keywords::normalized;
        // The built-in solver reads everything the input file writer writes; what it cannot
        // compute yet it reports when it runs.
        if self.engine == Engine::PrePoLix {
            return Vec::new();
        }
        // An empty executable means the bundled CalculiX, see `job_solver`.
        let solver = self.job_solver();
        let version = crate::solver_check::version_text(&solver.executable);
        if (self.keywords.as_ref())
            .is_none_or(|k| k.executable != self.executable || k.version != version)
        {
            self.keywords = Some(DetectedKeywords {
                executable: self.executable.clone(),
                version,
                known: Default::default(),
            });
        }
        let Some(detected) = self.keywords.as_mut() else {
            return Vec::new();
        };
        let untested = (keywords.iter())
            .filter(|k| !detected.known.contains_key(&normalized(k)))
            .cloned()
            .collect::<Vec<_>>();
        if !untested.is_empty() {
            let work_dir = self.work_dir();
            if let Some(known) =
                crate::solver_check::known_keywords(&solver, &work_dir, &untested, &|_, _| {})
                && let Some(detected) = self.keywords.as_mut()
            {
                detected.known.extend(known);
            }
        }
        let known = self.keywords.as_ref().map(|k| &k.known);
        known.map_or_else(Vec::new, |known| {
            crate::solver_check::unknown_of(known, keywords.iter().map(String::as_str))
        })
    }

    /// The executable restart states and tested keywords belong to: the resolved path with the file's size
    /// and modification time, so that another CalculiX at the same path, such as an update of
    /// the bundled one, is probed again.
    pub fn executable_key(&self) -> String {
        let executable = self.job_solver().executable;
        let Ok(meta) = std::fs::metadata(&executable) else {
            return executable.display().to_string();
        };
        let modified = (meta.modified().ok())
            .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
            .map_or(0, |d| d.as_secs());
        format!(
            "{} ({} bytes, {modified})",
            executable.display(),
            meta.len()
        )
    }

    pub fn job_solver(&self) -> plx_job::Solver {
        plx_job::Solver {
            executable: match clean_path(&self.executable) {
                "" => default_executable(),
                path => path.into(),
            },
            threads: self.threads_of(1),
        }
    }

    /// Threads of each of `concurrent` runs solved at the same time. The built-in solver
    /// runs them all in one shared pool of the chosen size ([`AUTOMATIC_THREADS`] for all
    /// cores), where they take the threads as they need them. CalculiX processes cannot
    /// share; with [`AUTOMATIC_THREADS`] each gets an equal share of the cores, so that
    /// together they do not ask for more than the computer has.
    pub fn threads_of(&self, concurrent: usize) -> u32 {
        match (self.engine, self.threads) {
            (Engine::PrePoLix, threads) => threads,
            (Engine::CalculiX, AUTOMATIC_THREADS) => (cores() / concurrent.max(1) as u32).max(1),
            (Engine::CalculiX, threads) => threads,
        }
    }

    /// These settings for one of `concurrent` runs solved at the same time, see
    /// [`Self::threads_of`].
    pub fn shared_by(&self, concurrent: usize) -> Self {
        Self {
            threads: self.threads_of(concurrent),
            ..self.clone()
        }
    }

    /// Settings saved before CalculiX was shipped hold the old default `ccx`; empty means the
    /// same where no CalculiX is bundled, and the bundled one where it is.
    fn forget_old_default(&mut self) {
        if self.executable.trim() == "ccx" {
            self.executable.clear();
        }
    }
}

/// Automatic threads, or the number chosen.
fn threads_ui(ui: &mut egui::Ui, threads: &mut u32) {
    ui.horizontal(|ui| {
        let mut automatic = *threads == AUTOMATIC_THREADS;
        let tip = format!(
            "Uses all {} cores of this computer. Analyses solved at the same time share them: the built-in solver hands them out as the analyses need them, CalculiX processes get an equal share each.",
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

/// Where the shipped CalculiX lies relative to the directory of the executable: Windows
/// installer, Linux AppImage (usr/bin next to usr/lib), and target/ccx/<platform> for
/// target/<profile>/prepolix.
const BUNDLED_LOCATIONS: [&str; 3] = [
    "ccx/bin",
    "../lib/prepolix/ccx/bin",
    if cfg!(windows) {
        "../ccx/windows/bin"
    } else {
        "../ccx/linux/bin"
    },
];

/// The CalculiX shipped with prepolix, if there is one.
pub fn bundled_ccx() -> Option<std::path::PathBuf> {
    let exe = std::env::current_exe().ok()?;
    find_bundled_ccx(exe.parent()?)
}

fn find_bundled_ccx(exe_dir: &std::path::Path) -> Option<std::path::PathBuf> {
    let name = if cfg!(windows) { "ccx.exe" } else { "ccx" };
    BUNDLED_LOCATIONS
        .iter()
        .map(|dir| exe_dir.join(dir).join(name))
        .find(|path| path.is_file())
        // Without `..` in the path shown in the settings and the output.
        .map(|path| {
            std::fs::canonicalize(&path)
                .map(strip_verbatim)
                .unwrap_or(path)
        })
}

/// `canonicalize` gives `\\?\C:\...` on Windows, which the settings would show as is.
fn strip_verbatim(path: std::path::PathBuf) -> std::path::PathBuf {
    match path.to_str().and_then(|p| p.strip_prefix(r"\\?\")) {
        Some(plain) if !plain.starts_with("UNC") => plain.into(),
        _ => path,
    }
}

/// CalculiX when the setting is empty: the bundled one, else `ccx` on the `PATH`.
pub fn default_executable() -> std::path::PathBuf {
    bundled_ccx().unwrap_or_else(|| "ccx".into())
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
    /// Ask GitHub for a new release when PrePoLix starts.
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

/// Why the executable cannot be started, if it is given as a path to a missing file;
/// a bare name is left to the `PATH` lookup.
pub fn missing_executable(executable: &std::path::Path) -> Option<String> {
    let is_path = executable.components().count() > 1 || executable.is_absolute();
    (is_path && !executable.is_file()).then(|| format!("File not found: {}", executable.display()))
}

/// PrePoMax's default work directory: a `Temp` folder next to the program. Where that is
/// not writable, e.g. for a binary installed to `/usr/bin`, a `prepolix` folder in the
/// system's temporary directory.
pub fn default_work_dir() -> std::path::PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|exe| Some(exe.parent()?.join("Temp")))
        .filter(|dir| is_writable(dir))
        .unwrap_or_else(|| std::env::temp_dir().join("prepolix"))
}

/// Creates the directory if needed and checks that files can be written into it.
fn is_writable(dir: &std::path::Path) -> bool {
    let probe = dir.join(".prepolix-write-test");
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
    default_executable: String,
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

/// State of the CalculiX self test on the settings page.
#[derive(Default)]
enum SolverCheck {
    #[default]
    NotRun,
    Running(
        std::sync::mpsc::Receiver<crate::solver_check::Report>,
        crate::solver_check::SharedProgress,
    ),
    Done(Engine, Vec<crate::solver_check::CheckResult>),
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
            default_executable: default_executable().display().to_string(),
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
                ui.checkbox(&mut g.shell_thickness, SHELL_THICKNESS_LABEL)
                    .on_hover_text(SHELL_THICKNESS_TIP);
                ui.checkbox(&mut g.beam_profiles, BEAM_PROFILES_LABEL)
                    .on_hover_text(BEAM_PROFILES_TIP);
                ui.checkbox(&mut g.show_linked, SHOW_LINKED_LABEL)
                    .on_hover_text(SHOW_LINKED_TIP);
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
                let calculix = solver.engine == Engine::CalculiX;
                egui::Grid::new("solver settings")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| {
                        ui.label("Solver");
                        ui.horizontal(|ui| {
                            for engine in Engine::ALL {
                                let tip = match engine {
                                    Engine::PrePoLix => "The solver built into PrePoLix: no external program, progress and results stay in the program. Steps it cannot compute yet are reported when the analysis runs.",
                                    Engine::CalculiX => "CalculiX (ccx) as a separate program, the one shipped with PrePoLix or your own.",
                                };
                                ui.radio_value(&mut solver.engine, engine, engine.name())
                                    .on_hover_text(tip);
                            }
                        });
                        ui.end_row();
                        if calculix {
                            ui.label("Executable");
                            ui.add(
                                egui::TextEdit::singleline(&mut solver.executable)
                                    .hint_text(self.default_executable.as_str()),
                            )
                            .on_hover_text(
                                "Leave empty for the CalculiX shipped with PrePoLix, or ccx on the PATH if there is none.",
                            );
                            ui.end_row();
                        }
                        ui.label("Threads");
                        threads_ui(ui, &mut solver.threads);
                        ui.end_row();
                        ui.label("Work directory");
                        ui.add(
                            egui::TextEdit::singleline(&mut solver.work_dir)
                                .hint_text(self.default_work_dir.as_str()),
                        );
                        ui.end_row();
                        if calculix {
                            ui.label("Restart files");
                            ui.checkbox(&mut solver.keep_restart_files, "Keep the state after each step")
                                .on_hover_text("CalculiX writes the state after each static, dynamic, heat transfer or visco step. Solving again after a change then starts after the last unchanged step instead of from the beginning. Needs disk space next to the results.");
                            ui.end_row();
                            ui.label("Restart files per project");
                            ui.add_enabled(
                                solver.keep_restart_files,
                                numeric::drag_value(&mut solver.restart_limit_mb)
                                    .range(10..=1_000_000)
                                    .suffix(" MB"),
                            )
                            .on_hover_text("Beyond this size the oldest restart files of the project are removed; an analysis without them is solved from the beginning.");
                            ui.end_row();
                        }
                        ui.label("Parallel analyses");
                        ui.add(numeric::drag_value(&mut solver.parallel_runs).range(1..=64))
                            .on_hover_text("Update Project solves up to this many independent analyses at the same time; the Campbell sweep runs as many speeds at a time. The built-in solver runs them in one pool of the threads above, so a single large analysis gets all of them. Each analysis needs its own memory.");
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
                        "Asks GitHub for the latest release when PrePoLix starts and shows a symbol next to the version if there is a newer one. Help > Check for Updates asks at any time.",
                    );
                ui.add_space(4.0);
                ui.add(egui::Label::new(self.installation.description()).wrap());
            }
        }
    }

    /// Gmsh imports STEP, IGES and BREP and meshes; prepolix loads its library at run time.
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
                                text += &format!(" PrePoLix is tested with Gmsh {major}.{minor}.");
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

    /// Button and results of the CalculiX self test, run with the settings in the window.
    fn solver_check_ui(&mut self, ui: &mut egui::Ui) {
        if let SolverCheck::Running(receiver, _) = &self.solver_check
            && let Ok(report) = receiver.try_recv()
        {
            if let Some(known) = report.keywords {
                self.draft.solver.keywords = Some(DetectedKeywords {
                    executable: self.draft.solver.executable.clone(),
                    version: report.version,
                    known,
                });
            }
            self.solver_check = SolverCheck::Done(self.draft.solver.engine, report.checks);
        }
        let running = matches!(self.solver_check, SolverCheck::Running(..));
        ui.horizontal(|ui| {
            let engine = self.draft.solver.engine;
            let tip = match engine {
                Engine::CalculiX => "Starts the executable, solves small cantilevers with the direct and the iterative equation solver, compares the results with beam theory and checks whether it knows every keyword PrePoLix writes.",
                Engine::PrePoLix => "Solves small cantilevers with the built-in solver, direct and iterative, and compares the results with beam theory.",
            };
            let button = ui
                .add_enabled(!running, egui::Button::new("Test solver"))
                .on_hover_text(tip);
            if button.clicked() {
                let (sender, receiver) = std::sync::mpsc::channel();
                let (solver, work_dir) = (self.draft.solver.job_solver(), self.draft.solver.work_dir());
                let ctx = ui.ctx().clone();
                let progress = crate::solver_check::SharedProgress::default();
                let thread_progress = progress.clone();
                std::thread::spawn(move || {
                    let report =
                        crate::solver_check::run(engine, &solver, &work_dir, &thread_progress);
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
            running_modal(ui.ctx(), self.draft.solver.engine, &progress);
        }
        if let SolverCheck::Done(engine, results) = &self.solver_check {
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
                ui.label(format!("The {} works correctly.", engine.name()));
            }
        }
    }
}

/// Modal window over the application while the solver self test runs, which takes a while
/// on some systems.
fn running_modal(ctx: &egui::Context, engine: Engine, progress: &crate::solver_check::Progress) {
    egui::Modal::new(crate::ids::window(ctx, "solver check running")).show(ctx, |ui| {
        ui.set_width(300.0);
        ui.heading(format!("Testing the {}", engine.name()));
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
    fn unknown_keywords_are_remembered() {
        if std::process::Command::new("ccx")
            .arg("-v")
            .output()
            .is_err()
        {
            eprintln!("ccx not found, test skipped");
            return;
        }
        let dir = std::env::temp_dir().join(format!("plx-keywords-{}", std::process::id()));
        let mut solver = Solver {
            work_dir: dir.display().to_string(),
            ..Solver::default()
        };
        let keywords = ["*STATIC".to_string(), "*NOT A KEYWORD".into()];
        assert_eq!(solver.unknown_keywords(&keywords), ["*NOT A KEYWORD"]);
        let detected = solver.keywords.clone().unwrap();
        assert!(detected.version.is_some());
        assert_eq!(detected.known.get("*NOTAKEYWORD"), Some(&false));
        // Tested keywords are not tested again: a cached verdict stands.
        solver
            .keywords
            .as_mut()
            .unwrap()
            .known
            .insert("*STATIC".into(), false);
        assert_eq!(solver.unknown_keywords(&keywords[..1]), ["*STATIC"]);
        let _ = std::fs::remove_dir_all(&dir);
    }

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
            clean_path(r#" "C:\Program Files\ccx.exe" "#),
            r"C:\Program Files\ccx.exe"
        );
        assert_eq!(clean_path("'/opt/ccx' "), "/opt/ccx");
        assert_eq!(clean_path("ccx"), "ccx");
        assert_eq!(clean_path("\"ccx"), "\"ccx");
        let solver = Solver {
            executable: "\"/opt/ccx\"".into(),
            work_dir: " \"/tmp/plx\" ".into(),
            ..Solver::default()
        };
        assert_eq!(
            solver.job_solver().executable,
            std::path::Path::new("/opt/ccx")
        );
        assert_eq!(solver.work_dir(), std::path::Path::new("/tmp/plx"));
    }

    #[test]
    fn automatic_threads_share_the_cores() {
        let solver = Solver {
            engine: Engine::CalculiX,
            ..Solver::default()
        };
        assert_eq!(solver.threads, AUTOMATIC_THREADS);
        assert_eq!(solver.job_solver().threads, cores());
        assert_eq!(solver.threads_of(2), (cores() / 2).max(1));
        assert_eq!(solver.threads_of(10_000), 1);
        let chosen = Solver {
            threads: 3,
            ..solver.clone()
        };
        assert_eq!(chosen.threads_of(4), 3);
        assert_eq!(chosen.shared_by(4).job_solver().threads, 3);
        // The built-in solver's analyses share one pool instead.
        let builtin = Solver {
            engine: Engine::PrePoLix,
            ..Solver::default()
        };
        assert_eq!(builtin.threads_of(4), AUTOMATIC_THREADS);
        assert_eq!(builtin.shared_by(4).threads, AUTOMATIC_THREADS);
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
    fn empty_executable_means_the_bundled_calculix() {
        let solver = Solver::default();
        assert_eq!(solver.job_solver().executable, default_executable());
        // Settings of older versions stored the then default `ccx`.
        let mut old: Settings = ron::from_str("(solver: (executable: \"ccx\"))").unwrap();
        old.upgrade();
        assert_eq!(old.solver.executable, "");
        // Once upgraded, `ccx` is what the user chose, also after a round trip.
        let mut chosen = Settings::default();
        chosen.solver.executable = "ccx".into();
        let mut chosen: Settings = ron::from_str(&ron::to_string(&chosen).unwrap()).unwrap();
        chosen.upgrade();
        assert_eq!(chosen.solver.executable, "ccx");
        let mut own: Settings = ron::from_str("(solver: (executable: \"/opt/ccx\"))").unwrap();
        own.upgrade();
        assert_eq!(own.solver.executable, "/opt/ccx");
    }

    #[test]
    fn replaced_executable_is_probed_again() {
        let dir = std::env::temp_dir().join(format!("plx-ccx-key-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let exe = dir.join("ccx");
        std::fs::write(&exe, "old").unwrap();
        let solver = Solver {
            executable: exe.display().to_string(),
            ..Solver::default()
        };
        let old = solver.executable_key();
        assert_eq!(solver.executable_key(), old);
        // Same path and version, but another build, e.g. the bundled CalculiX of an update.
        std::fs::write(&exe, "new build").unwrap();
        assert_ne!(solver.executable_key(), old);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn bundled_calculix_is_found_next_to_the_program() {
        let root = std::env::temp_dir().join(format!("plx-bundled-ccx-{}", std::process::id()));
        let exe_dir = root.join("usr").join("bin");
        std::fs::create_dir_all(&exe_dir).unwrap();
        assert_eq!(find_bundled_ccx(&exe_dir), None);
        // Where the AppImage carries it.
        let bin = root.join("usr/lib/prepolix/ccx/bin");
        std::fs::create_dir_all(&bin).unwrap();
        let name = if cfg!(windows) { "ccx.exe" } else { "ccx" };
        std::fs::write(bin.join(name), "").unwrap();
        let found = find_bundled_ccx(&exe_dir).unwrap();
        assert!(found.ends_with(std::path::Path::new("lib/prepolix/ccx/bin").join(name)));
        assert!(!found.to_string_lossy().contains(".."));
        // The Windows installer's folder next to the program wins.
        let local = exe_dir.join("ccx").join("bin");
        std::fs::create_dir_all(&local).unwrap();
        std::fs::write(local.join(name), "").unwrap();
        assert!(
            find_bundled_ccx(&exe_dir)
                .unwrap()
                .starts_with(std::fs::canonicalize(&exe_dir).unwrap())
        );
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn missing_files_are_reported_but_names_are_looked_up() {
        assert!(missing_executable(std::path::Path::new("ccx")).is_none());
        let missing = std::env::temp_dir().join("plx-does-not-exist").join("ccx");
        let message = missing_executable(&missing).unwrap();
        assert!(message.starts_with("File not found"), "{message}");
        let exe = std::env::current_exe().unwrap();
        assert!(missing_executable(&exe).is_none());
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
