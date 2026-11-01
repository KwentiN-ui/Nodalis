//! Running the analysis: writes the input file, starts the solver (the built-in one or
//! CalculiX, see [`plx_solve::Run`]) and shows PrePoMax's monitor window with the solver
//! output, progress and the button to open the results.

use std::path::PathBuf;
use std::time::Instant;

use plx_job::JobStatus;
use plx_job::restart::{self, Chain, Plan, StateIndex};
use plx_solve::{Engine, Run};

use crate::model::Model;
use crate::settings::Solver;

/// Lines of solver output kept for the monitor; older ones are dropped, see
/// [`Analysis::dropped`].
pub const MAX_OUTPUT_LINES: usize = 20_000;

pub struct Analysis {
    job: Run,
    /// Name of the analysis, also of its input and results files.
    name: String,
    /// Fingerprint of the model the run solves, see [`plx_model::fingerprint::solve_input`];
    /// kept with its results to tell when they are out of date.
    pub input: u64,
    /// The fingerprint of each step's results, see [`plx_model::fingerprint::step_inputs`].
    pub steps: Vec<u64>,
    output: Vec<String>,
    /// Lines dropped from the front of `output` to keep it at [`MAX_OUTPUT_LINES`].
    dropped: usize,
    /// Text typed into the monitor's filter field; only output lines containing it are shown.
    filter: String,
    started: Instant,
    finished: Option<f32>,
    status: JobStatus,
    /// The keywords of the input file, see [`plx_io::inp::keywords::input_keywords`].
    keywords: Vec<String>,
    /// How the run reuses the state of an earlier one; `None` for a model check.
    restart: Option<RestartRun>,
}

/// What a run needs to start from the state of an earlier run and to keep its own states,
/// see [`plx_job::restart`].
struct RestartRun {
    chain: Chain,
    plan: Plan,
    /// The CalculiX the states belong to, see [`crate::settings::Solver::executable_key`].
    solver_key: String,
    job_solver: plx_job::Solver,
    work_dir: PathBuf,
    job_name: String,
    /// Most the restart files of the project may take, in bytes; `None` keeps none.
    limit: Option<u64>,
}

pub enum MonitorEvent {
    None,
    /// Open the results of the run.
    OpenResults,
    /// Stop the run.
    Stop,
}

/// The project's solve of an analysis as its window shows it, copied from the solve as it
/// goes on.
#[derive(Default)]
pub struct RunView {
    /// When the solve started, which tells one solve from the next.
    pub started: Option<Instant>,
    pub active: bool,
    pub state: &'static str,
    pub seconds: f32,
    pub progress: Option<String>,
    pub lines: Vec<String>,
    /// Lines of the solve copied so far, also those dropped from `lines` since.
    pub copied: usize,
    pub filter: String,
}

impl RunView {
    /// Copies the new lines of `output`, of which `total` were ever written (the first ones
    /// may be dropped already), and keeps the last [`MAX_OUTPUT_LINES`].
    pub fn copy<S: AsRef<str>>(&mut self, output: &[S], total: usize) {
        if total < self.copied {
            self.lines.clear();
            self.copied = 0;
        }
        let new = (total - self.copied).min(output.len());
        (self.lines).extend(
            output[output.len() - new..]
                .iter()
                .map(|l| l.as_ref().to_owned()),
        );
        self.copied = total;
        keep_last(&mut self.lines, MAX_OUTPUT_LINES);
    }

    /// The monitor of the solve, like that of a run: Stop, Results, the filter and the
    /// output.
    pub fn monitor(&mut self, ui: &mut egui::Ui, name: &str) -> MonitorEvent {
        let mut event = MonitorEvent::None;
        ui.horizontal(|ui| {
            if ui
                .add_enabled(self.active, egui::Button::new("Stop"))
                .on_hover_text("Stops the solve")
                .clicked()
            {
                event = MonitorEvent::Stop;
            }
            if ui
                .add_enabled(!self.active, egui::Button::new("Results"))
                .on_hover_text("Opens the results the analysis keeps")
                .clicked()
            {
                event = MonitorEvent::OpenResults;
            }
            ui.separator();
            ui.label("Filter:");
            ui.add(
                egui::TextEdit::singleline(&mut self.filter)
                    .hint_text("Show only lines containing ...")
                    .desired_width(180.0),
            );
            if ui
                .add_enabled(!self.filter.is_empty(), egui::Button::new("Clear"))
                .clicked()
            {
                self.filter.clear();
            }
            ui.separator();
            if self.active {
                ui.spinner();
            }
            ui.label(format!("{name}: {} ({:.1} s)", self.state, self.seconds));
            if let Some(text) = &self.progress {
                ui.separator();
                ui.label(text);
            }
            dropped_note(ui, self.copied - self.lines.len().min(self.copied));
        });
        let area = egui::ScrollArea::both()
            .id_salt("solve output")
            .auto_shrink([false, false])
            .stick_to_bottom(true);
        if self.filter.is_empty() {
            crate::virtual_table::lines(ui, area, &self.lines);
        } else {
            let needle = self.filter.to_lowercase();
            let shown: Vec<&String> = (self.lines.iter())
                .filter(|line| line.to_lowercase().contains(&needle))
                .collect();
            crate::virtual_table::lines(ui, area, &shown);
        }
        event
    }
}

/// The analysis name as the name of its files: CalculiX takes no spaces in them, and the
/// file system no path separators.
pub fn job_name(name: &str) -> String {
    let name: String = (name.trim().chars())
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    if name.is_empty() {
        "Analysis".into()
    } else {
        name
    }
}

/// Copies the results file of a submodel's global model next to the input file `<name>.inp`
/// in `dir`, as `<name>-global.frd`, and lets `fe` refer to the copy: CalculiX reads it from
/// the directory of the input file and cannot take paths with spaces. Does nothing unless a
/// submodel boundary condition needs the file.
pub fn stage_global_results(
    fe: &mut plx_model::FeModel,
    dir: &std::path::Path,
    name: &str,
) -> Result<(), String> {
    if !fe.uses_global_results() {
        return Ok(());
    }
    // Without a file the input file reports the missing global results.
    let Some(source) = fe
        .properties
        .submodel_input()
        .map(std::path::Path::to_path_buf)
    else {
        return Ok(());
    };
    let target = dir.join(format!("{name}-global.frd"));
    let same = |a: &std::path::Path, b: &std::path::Path| {
        a.canonicalize()
            .ok()
            .is_some_and(|a| b.canonicalize().ok() == Some(a))
    };
    if same(&source, &dir.join(format!("{name}.frd"))) {
        return Err(format!(
            "The global results {} are this analysis's own results file, which the run \
             overwrites. Link the results of another analysis to Global results.",
            source.display()
        ));
    }
    if !same(&source, &target) {
        std::fs::create_dir_all(dir)
            .and_then(|()| std::fs::copy(&source, &target))
            .map_err(|e| format!("Global results {} not copied: {e}", source.display()))?;
    }
    fe.properties.global_results = Some(target);
    Ok(())
}

/// Prepares what the model takes from the results of other analyses: the stored eigenmodes
/// are copied to `<name>.eig`, where CalculiX reads them; the initial temperatures are read
/// from their results file and returned as the keyword that sets them.
pub fn stage_linked_results(
    fe: &plx_model::FeModel,
    dir: &std::path::Path,
    name: &str,
) -> Result<Option<String>, String> {
    if let Some(modes) = &fe.properties.stored_modes {
        if !modes.is_file() {
            return Err(format!(
                "The eigenmodes {} are gone: solve the analysis they come from again.",
                modes.display()
            ));
        }
        let target = dir.join(format!("{name}.eig"));
        if *modes != target {
            std::fs::create_dir_all(dir)
                .and_then(|()| std::fs::copy(modes, &target))
                .map_err(|e| format!("Eigenmodes {} not copied: {e}", modes.display()))?;
        }
    }
    let Some((file, step)) = &fe.properties.initial_temperatures else {
        return Ok(None);
    };
    let results =
        plx_io::frd::read_frd(file).map_err(|e| format!("Initial temperatures not read: {e}"))?;
    let temperatures = (results.increments.iter().rev())
        .filter(|i| i.step == *step)
        .find_map(|i| i.field("NDTEMP")?.component("T"))
        .ok_or_else(|| {
            format!(
                "Initial temperatures not read: {} has no temperatures of step {step}.",
                file.display()
            )
        })?;
    let mut keyword = format!(
        "** Initial temperatures: step {step} of {}\n*Initial conditions, Type=Temperature\n",
        file.display()
    );
    for (node, value) in results.mesh.node_ids().iter().zip(&temperatures.values) {
        keyword.push_str(&format!("{node}, {value:e}\n"));
    }
    Ok(Some(keyword))
}

/// The input file with `keyword` inserted after the model definition, before the first step.
fn before_first_step(input: &str, keyword: &str) -> String {
    let at = (input.match_indices('\n'))
        .map(|(i, _)| i + 1)
        .find(|&i| {
            input[i..]
                .get(..5)
                .is_some_and(|k| k.eq_ignore_ascii_case("*step"))
        })
        .unwrap_or(input.len());
    format!("{}{keyword}{}", &input[..at], &input[at..])
}

impl Analysis {
    /// Writes the input file of the analysis `name` into `work_dir` and starts CalculiX on
    /// it. With `check_model` CalculiX only reads and checks the model
    /// (PrePoMax's "Check Model"). `continuation` tells which analyses it continues from the
    /// state of and which continue from its own.
    pub fn start(
        solver: &Solver,
        model: &Model,
        check_model: bool,
        name: &str,
        work_dir: &std::path::Path,
        continuation: &crate::links::Continuation,
    ) -> Result<Self, String> {
        if model.fe.steps.is_empty() && !check_model {
            return Err(
                "Analysis not started: the model has no step (Model > Create Step …).".into(),
            );
        }
        if !check_model && !model.fe.steps.iter().any(|s| s.active) {
            return Err("Analysis not started: all steps are deactivated.".into());
        }
        let heading = format!("PrePoLix: {}", model.file_name());
        let mut fe = model.fe.clone();
        let job_name = job_name(name);
        stage_global_results(&mut fe, work_dir, &job_name)?;
        let initial = stage_linked_results(&fe, work_dir, &job_name)?;
        // The sets of the analyses that continue from its states go into the input file too:
        // CalculiX takes no new sets after reading a state.
        let input = if check_model {
            plx_io::inp::write_check_inp(&model.mesh, &fe, &heading)
        } else {
            let dependent = &continuation.dependent_steps;
            plx_io::inp::write_inp_with_sets_of(&model.mesh, &fe, &heading, dependent)
        }
        .map_err(|e| format!("Input file not written: {e}"))?;
        let input = match &initial {
            Some(keyword) => before_first_step(&input, keyword),
            None => input,
        };
        let overlaps = plx_io::inp::resolved_overlaps(&input);
        // The model check leaves out the steps; their keywords are still tested.
        let keywords = if check_model {
            let full = plx_io::inp::write_inp(&model.mesh, &fe, &heading).unwrap_or_default();
            plx_io::inp::keywords::input_keywords(&full)
        } else {
            plx_io::inp::keywords::input_keywords(&input)
        };
        let copied = copy_result_files(&fe, work_dir);
        let job_solver = solver.job_solver();
        let engine = solver.engine;
        let hint = "Check the executable under Tools > Settings > Solver.";
        if engine == Engine::CalculiX
            && let Some(message) = crate::settings::missing_executable(&job_solver.executable)
        {
            return Err(format!("CalculiX not started: {message}. {hint}"));
        }
        // Restarts from the state of an earlier run are CalculiX's `.rout` files; the
        // built-in solver keeps its state in memory in a later stage.
        let restart = (!check_model && engine == Engine::CalculiX).then(|| {
            let job = (work_dir, job_name.as_str());
            plan_restart(solver, &fe, &input, job, continuation, job_solver.clone())
        });
        let run_input = match &restart {
            Some(RestartRun {
                chain,
                plan: Plan::Restart { after, .. },
                ..
            }) => chain.restart_input(*after),
            Some(run) => run.chain.input.clone(),
            None => input,
        };
        let job =
            Run::start(engine, &job_solver, work_dir, &job_name, &run_input).map_err(|e| {
                match engine {
                    Engine::CalculiX => format!(
                        "CalculiX ({}) not started: {e}. {hint}",
                        job_solver.executable.display()
                    ),
                    Engine::PrePoLix => format!("Solver not started: {e}"),
                }
            })?;
        let mut output = vec![format!(
            "{name} {} in {}",
            if check_model {
                "model check started"
            } else {
                "started"
            },
            work_dir.display()
        )];
        output.extend(copied);
        // Constraints the input file writer had to change so that they do not bind a degree of
        // freedom twice.
        output.extend(
            (overlaps.into_iter()).map(|line| format!("Overlapping constraints resolved: {line}")),
        );
        if let Some(run) = &restart {
            output.push(run.describe());
        }
        Ok(Self {
            output,
            dropped: 0,
            filter: String::new(),
            job,
            name: name.to_owned(),
            input: plx_model::fingerprint::solve_input(&model.mesh, &model.fe),
            steps: plx_model::fingerprint::step_inputs(&model.mesh, &model.fe),
            started: Instant::now(),
            finished: None,
            status: JobStatus::Running,
            keywords,
            restart,
        })
    }

    /// Adds a line of prepolix's own to the monitor output.
    pub fn note(&mut self, line: String) {
        self.collect([line]);
    }

    /// The keywords of the input file CalculiX runs on.
    pub fn keywords(&self) -> &[String] {
        &self.keywords
    }

    /// The solver output collected so far, without the lines [`Self::dropped`].
    pub fn output(&self) -> &[String] {
        &self.output
    }

    /// The number of lines dropped from the front of [`Self::output`] in a long run.
    pub fn dropped(&self) -> usize {
        self.dropped
    }

    /// Adds `lines` to the output and drops the oldest ones over [`MAX_OUTPUT_LINES`].
    fn collect(&mut self, lines: impl IntoIterator<Item = String>) {
        self.output.extend(lines);
        self.dropped += keep_last(&mut self.output, MAX_OUTPUT_LINES);
    }

    pub fn is_running(&self) -> bool {
        self.finished.is_none()
    }

    /// Collects new output; returns the final status once, when the job has just ended.
    pub fn poll(&mut self) -> Option<JobStatus> {
        let lines = self.job.new_output();
        self.collect(lines);
        if self.finished.is_some() {
            return None;
        }
        let status = self.job.poll();
        if status == JobStatus::Running {
            return None;
        }
        let lines = self.job.new_output();
        self.collect(lines);
        if self.solve_in_full_after_failed_restart(status) {
            return None;
        }
        if let Some(run) = &self.restart {
            let notes = run.finish(&self.job, status);
            self.collect(notes);
        }
        self.finished = Some(self.started.elapsed().as_secs_f32());
        self.status = status;
        Some(status)
    }

    /// How long the run has taken so far, or took.
    pub fn seconds(&self) -> f32 {
        (self.finished).unwrap_or_else(|| self.started.elapsed().as_secs_f32())
    }

    /// Where the run is: step, increment, attempt, iteration and time, once CalculiX tells.
    pub fn progress(&self) -> Option<String> {
        let finished = || {
            self.job.progress().map(|p| {
                format!(
                    "Step {}, increment {}, iterations {}, time {}",
                    p.step, p.increment, p.iterations, p.total_time
                )
            })
        };
        // The .sta file only changes when an increment has converged; while the run goes on
        // its output tells about every attempt and iteration.
        let live = || self.job.live_progress().text();
        if self.is_running() {
            live().or_else(finished)
        } else {
            finished().or_else(live)
        }
    }

    pub fn kill(&mut self) {
        self.job.kill();
        self.poll();
    }

    pub fn status_text(status: JobStatus) -> &'static str {
        match status {
            JobStatus::Running => "running",
            JobStatus::Completed => "completed",
            JobStatus::FailedWithResults => "finished with errors, results available",
            JobStatus::Failed => "failed",
            JobStatus::Killed => "killed",
        }
    }

    /// PrePoMax's monitor, in the Analysis tab below the 3D view: status, progress and
    /// filter in one row over the output of the run.
    pub fn monitor(&mut self, ui: &mut egui::Ui) -> MonitorEvent {
        let mut event = MonitorEvent::None;
        // Running until `poll` has joined the results of a restart with the earlier ones.
        let status = self.status;
        ui.horizontal(|ui| {
            if ui
                .add_enabled(status == JobStatus::Running, egui::Button::new("Kill"))
                .clicked()
            {
                event = MonitorEvent::Stop;
            }
            let results = self.job.results().filter(|_| status != JobStatus::Running);
            if ui
                .add_enabled(results.is_some(), egui::Button::new("Results"))
                .on_hover_text("Opens the results of the run")
                .clicked()
            {
                event = MonitorEvent::OpenResults;
            }
            ui.separator();
            ui.label("Filter:");
            ui.add(
                egui::TextEdit::singleline(&mut self.filter)
                    .hint_text("Show only lines containing ...")
                    .desired_width(180.0),
            );
            if ui
                .add_enabled(!self.filter.is_empty(), egui::Button::new("Clear"))
                .clicked()
            {
                self.filter.clear();
            }
            ui.separator();
            if status == JobStatus::Running {
                ui.spinner();
            }
            let seconds = self.seconds();
            ui.label(format!(
                "{}: {} ({seconds:.1} s)",
                self.name,
                Self::status_text(status)
            ));
            if let Some(text) = self.progress() {
                ui.separator();
                ui.label(text);
            }
            dropped_note(ui, self.dropped);
        });
        let area = egui::ScrollArea::both()
            .id_salt("analysis output")
            .auto_shrink([false, false])
            .stick_to_bottom(true);
        if self.filter.is_empty() {
            crate::virtual_table::lines(ui, area, &self.output);
        } else {
            let needle = self.filter.to_lowercase();
            let shown: Vec<&String> = self
                .output
                .iter()
                .filter(|line| line.to_lowercase().contains(&needle))
                .collect();
            crate::virtual_table::lines(ui, area, &shown);
        }
        event
    }

    pub fn results(&self) -> Option<PathBuf> {
        self.job.results()
    }

    /// A run from the state of an earlier one that CalculiX could not finish is solved once
    /// more from the beginning: a restart is only a shortcut, and CalculiX does not take it
    /// after every kind of step. Returns whether the full run started.
    fn solve_in_full_after_failed_restart(&mut self, status: JobStatus) -> bool {
        let Some(run) = &mut self.restart else {
            return false;
        };
        if !matches!(status, JobStatus::Failed | JobStatus::FailedWithResults)
            || !matches!(run.plan, Plan::Restart { .. })
        {
            return false;
        }
        restart::discard(&run.work_dir, &run.job_name);
        run.plan = Plan::Full("the restart failed".into());
        self.output
            .push("The restart failed: CalculiX solves all steps again.".into());
        match Run::start(
            Engine::CalculiX,
            &run.job_solver,
            &run.work_dir,
            &run.job_name,
            &run.chain.input,
        ) {
            Ok(job) => {
                self.job = job;
                self.status = JobStatus::Running;
                true
            }
            Err(error) => {
                self.output.push(format!("CalculiX not started: {error}"));
                false
            }
        }
    }
}

/// Decides whether the job starts from the state of an earlier run, see
/// [`plx_job::restart`], and prepares its work directory for that.
fn plan_restart(
    solver: &Solver,
    fe: &plx_model::FeModel,
    input: &str,
    (work_dir, job_name): (&std::path::Path, &str),
    continuation: &crate::links::Continuation,
    job_solver: plx_job::Solver,
) -> RestartRun {
    // Files the job reads besides its input file count as part of the model.
    let mut external: Vec<PathBuf> = (fe.result_files().into_iter())
        .map(std::path::Path::to_path_buf)
        .collect();
    external.extend(fe.properties.global_results.clone());
    external.extend(fe.properties.stored_modes.clone());
    external.extend((fe.properties.initial_temperatures.iter()).map(|(file, _)| file.clone()));
    let keep = solver.keep_restart_files;
    let chain = Chain::new(input, keep, restart::files_fingerprint(&external));
    let solver_key = solver.executable_key();
    let mut plan = if keep {
        // Its own earlier runs first, then those of the analyses it continues.
        let mut jobs = vec![(work_dir.to_owned(), job_name.to_owned())];
        jobs.extend(continuation.upstream.iter().cloned());
        restart::plan(&chain, &solver_key, &jobs)
    } else {
        StateIndex::clear(work_dir, job_name);
        Plan::Full("restart files are turned off in the settings".into())
    };
    if let Plan::Restart { file, results, .. } = &plan
        && let Err(error) = restart::prepare(work_dir, job_name, file, results)
    {
        restart::discard(work_dir, job_name);
        plan = Plan::Full(format!("the state could not be read: {error}"));
    }
    RestartRun {
        chain,
        plan,
        solver_key,
        job_solver,
        work_dir: work_dir.to_owned(),
        job_name: job_name.to_owned(),
        limit: keep.then(|| solver.restart_limit_mb.saturating_mul(1 << 20)),
    }
}

impl RestartRun {
    /// A line for the monitor on how the run starts.
    fn describe(&self) -> String {
        let steps = self.chain.steps.len();
        match &self.plan {
            Plan::Restart { after, results, .. } => {
                // The state of another analysis, which this one continues.
                let source = (results.parent() != Some(self.work_dir.as_path()))
                    .then(|| results.file_stem())
                    .flatten()
                    .map_or_else(
                        || "the earlier run".to_owned(),
                        |job| format!("{}", job.to_string_lossy()),
                    );
                format!(
                    "Restart after step {after} of {steps}: the steps up to it are unchanged; \
                     CalculiX reads their state from {source} and solves the rest."
                )
            }
            Plan::Full(reason) => format!("Full run: {reason}."),
        }
    }

    /// Joins the results of a restart with the earlier ones, keeps the full input file next
    /// to them and records the states the run wrote. Returns lines for the monitor.
    fn finish(&self, job: &Run, status: JobStatus) -> Vec<String> {
        let mut notes = Vec::new();
        let (dir, name) = (&self.work_dir, self.job_name.as_str());
        if let Plan::Restart { after, time, .. } = &self.plan {
            if job.results().is_some() {
                match restart::join_results(dir, name, *after, *time) {
                    Ok(()) => notes.push(format!(
                        "Results of steps 1 to {after} taken from the earlier run."
                    )),
                    Err(error) => notes.push(format!("Earlier results not joined: {error}")),
                }
            } else {
                restart::discard(dir, name);
            }
            // The input file kept with the results is the full one, which runs on its own.
            let restart_input = dir.join(format!("{name}.restart.inp"));
            let _ = std::fs::rename(job.file("inp"), restart_input);
            if let Err(error) = std::fs::write(job.file("inp"), &self.chain.input) {
                notes.push(format!("Full input file not written: {error}"));
            }
        }
        let Some(limit) = self.limit else {
            return notes;
        };
        let run = restart::Run {
            after: match self.plan {
                Plan::Restart { after, .. } => after,
                Plan::Full(_) => 0,
            },
            completed: status == JobStatus::Completed,
            results: job.results().is_some(),
            sta: std::fs::read_to_string(job.file("sta")).unwrap_or_default(),
        };
        let mut index = StateIndex::read(dir, name);
        if let Err(error) = index.record(&self.chain, &self.solver_key, dir, name, &run) {
            notes.push(format!("Restart files not kept: {error}"));
        }
        if let Some(project) = dir.parent() {
            notes.extend(restart::limit_states(project, limit));
        }
        notes
    }
}

/// Drops the first lines of `lines` over `max`; returns how many.
fn keep_last<T>(lines: &mut Vec<T>, max: usize) -> usize {
    let over = lines.len().saturating_sub(max);
    lines.drain(..over);
    over
}

/// Tells in the monitor's row that the first `dropped` lines of a long output are not kept.
pub fn dropped_note(ui: &mut egui::Ui, dropped: usize) {
    if dropped > 0 {
        ui.separator();
        ui.weak(format!("{dropped} earlier lines not kept"))
            .on_hover_text(format!(
                "The monitor keeps the last {MAX_OUTPUT_LINES} lines"
            ));
    }
}

/// Copies the result files the defined fields read next to the input file, where CalculiX
/// looks for them; a file already there is left alone. Returns a line per file for the
/// monitor output.
pub fn copy_result_files(fe: &plx_model::FeModel, dir: &std::path::Path) -> Vec<String> {
    let mut notes = Vec::new();
    let files = fe.result_files();
    if !files.is_empty()
        && let Err(error) = std::fs::create_dir_all(dir)
    {
        return vec![format!("{} not created: {error}", dir.display())];
    }
    for file in files {
        let Some(name) = file.file_name() else {
            continue;
        };
        let target = dir.join(name);
        if target == file {
            continue;
        }
        match std::fs::copy(file, &target) {
            Ok(_) => notes.push(format!("{} copied to {}", file.display(), dir.display())),
            Err(error) => notes.push(format!("{} not copied: {error}", file.display())),
        }
    }
    notes
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A row of three cubes, fixed at one end and pulled at the other in three steps, the
    /// last one by `last`.
    fn pulled_cubes(last: f64) -> Model {
        use plx_model::{BoundaryCondition, BoundaryKind, Elastic, Material, Region, Section};
        let mut mesh = plx_mesh::FeMesh::default();
        let mut id = 0;
        for x in 0..=3 {
            for (y, z) in [(0, 0), (1, 0), (1, 1), (0, 1)] {
                id += 1;
                mesh.set_node(id, [x as f64, y as f64, z as f64]);
            }
        }
        for index in 0..3 {
            let first = 4 * index;
            (mesh.add_element(plx_mesh::Element {
                id: index + 1,
                type_name: "C3D8".into(),
                shape: plx_mesh::ElementShape::Hex8,
                nodes: (1..=8).map(|n| first + n).collect(),
            }))
            .unwrap();
        }
        mesh.parts.push(plx_mesh::Part {
            name: "Solid-1".into(),
            elements: vec![1, 2, 3],
        });
        let bc = |name: &str, nodes: Vec<u32>, kind| BoundaryCondition {
            distribution: None,
            name: name.into(),
            active: true,
            visible: true,
            region: Region::Nodes(nodes),
            kind,
            amplitude: None,
        };
        // Each node pulled: the end face contracts freely.
        let pull = |u: f64| {
            BoundaryKind::Displacement(plx_model::Displacement {
                behavior: plx_model::Behavior::EachNode,
                ..plx_model::Displacement::new([Some(u), None, None, None, None, None])
            })
        };
        let steps = [0.01, 0.02, last].into_iter().enumerate().map(|(i, u)| {
            let mut step = plx_model::Step::new_static(format!("Step-{}", i + 1));
            step.boundary_conditions = vec![
                bc(
                    "Fix",
                    vec![1, 2, 3, 4],
                    BoundaryKind::displacement([Some(0.0); 6]),
                ),
                bc("Pull", vec![13, 14, 15, 16], pull(u)),
            ];
            step
        });
        let mut model = Model::new(std::path::Path::new("cubes.plx"), mesh);
        model.fe = plx_model::FeModel {
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
                region: Region::Parts(vec!["Solid-1".into()]),
                thickness: 1.0,
                kind: plx_model::SectionKind::Solid,
            }],
            steps: steps.collect(),
            ..Default::default()
        };
        model
    }

    /// Solves the model in `dir`; returns the monitor output.
    fn solve(model: &Model, dir: &std::path::Path) -> Vec<String> {
        solve_as(model, dir, "Cubes", &Default::default())
    }

    fn solve_as(
        model: &Model,
        dir: &std::path::Path,
        name: &str,
        continuation: &crate::links::Continuation,
    ) -> Vec<String> {
        let solver = Solver {
            executable: "ccx".into(),
            ..Default::default()
        };
        let mut analysis = Analysis::start(&solver, model, false, name, dir, continuation).unwrap();
        let status = loop {
            if let Some(status) = analysis.poll() {
                break status;
            }
            std::thread::sleep(std::time::Duration::from_millis(20));
        };
        assert_eq!(status, JobStatus::Completed, "{:?}", analysis.output());
        analysis.output().to_vec()
    }

    #[test]
    fn a_changed_last_step_is_solved_alone() {
        if std::process::Command::new("ccx")
            .arg("-v")
            .output()
            .is_err()
        {
            eprintln!("ccx missing, test skipped");
            return;
        }
        let dir = std::env::temp_dir().join(format!("prepolix-restart-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let output = solve(&pulled_cubes(0.03), &dir.join("Cubes"));
        assert!(
            output.iter().any(|l| l.starts_with("Full run")),
            "{output:?}"
        );
        let output = solve(&pulled_cubes(0.04), &dir.join("Cubes"));
        assert!(
            output
                .iter()
                .any(|l| l.starts_with("Restart after step 2 of 3")),
            "{output:?}"
        );
        // The same results as solving all steps, and the full input file next to them.
        solve(&pulled_cubes(0.04), &dir.join("Full"));
        let restarted = plx_io::frd::read_frd(&dir.join("Cubes/Cubes.frd")).unwrap();
        let full = plx_io::frd::read_frd(&dir.join("Full/Cubes.frd")).unwrap();
        assert_eq!(restarted.increments.len(), 3);
        assert_eq!(
            format!("{:?}", restarted.increments),
            format!("{:?}", full.increments)
        );
        let inp = std::fs::read_to_string(dir.join("Cubes/Cubes.inp")).unwrap();
        assert!(inp.contains("*Node") && !inp.contains("Read"), "{inp}");
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn an_analysis_continues_from_the_state_of_another() {
        if std::process::Command::new("ccx")
            .arg("-v")
            .output()
            .is_err()
        {
            eprintln!("ccx missing, test skipped");
            return;
        }
        let dir = std::env::temp_dir().join(format!("prepolix-continue-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        // The continuing analysis holds the end with a boundary condition of its own, on a
        // set the first analysis does not use itself.
        let mut target = pulled_cubes(0.04);
        let own = &mut target.fe.steps[2];
        own.boundary_conditions[1].name = "Hold".into();
        let own = own.clone();
        let mut source = pulled_cubes(0.04);
        source.fe.steps.truncate(2);
        let continuation = crate::links::Continuation {
            dependent_steps: vec![(1, vec![own])],
            ..Default::default()
        };
        solve_as(&source, &dir.join("Source"), "Source", &continuation);
        let source_inp = std::fs::read_to_string(dir.join("Source/Source.inp")).unwrap();
        assert!(source_inp.contains("Hold"), "{source_inp}");
        assert_eq!(source_inp.matches("*Step").count(), 2);
        let continuation = crate::links::Continuation {
            upstream: vec![(dir.join("Source"), "Source".into())],
            ..Default::default()
        };
        let output = solve_as(&target, &dir.join("Target"), "Target", &continuation);
        assert!(
            output
                .iter()
                .any(|l| l.starts_with("Restart after step 2 of 3") && l.contains("from Source")),
            "{output:?}"
        );
        // Without the set of the continuing analysis in its input, the state cannot be read.
        solve_as(&source, &dir.join("Plain"), "Plain", &Default::default());
        let continuation = crate::links::Continuation {
            upstream: vec![(dir.join("Plain"), "Plain".into())],
            ..Default::default()
        };
        let output = solve_as(&target, &dir.join("Other"), "Target", &continuation);
        assert!(
            output.iter().any(|l| l.starts_with("Full run")),
            "{output:?}"
        );
        solve_as(&target, &dir.join("Full"), "Target", &Default::default());
        let continued = plx_io::frd::read_frd(&dir.join("Target/Target.frd")).unwrap();
        let full = plx_io::frd::read_frd(&dir.join("Full/Target.frd")).unwrap();
        assert_eq!(continued.increments.len(), 3);
        assert_eq!(
            format!("{:?}", continued.increments),
            format!("{:?}", full.increments)
        );
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn eigenmodes_and_temperatures_come_from_other_analyses() {
        use plx_model::{BoundaryCondition, BoundaryKind, Load, LoadKind, Region, Step};
        if std::process::Command::new("ccx")
            .arg("-v")
            .output()
            .is_err()
        {
            eprintln!("ccx missing, test skipped");
            return;
        }
        let dir = std::env::temp_dir().join(format!("prepolix-linked-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let bc = |name: &str, nodes: Vec<u32>, kind| BoundaryCondition {
            distribution: None,
            name: name.into(),
            active: true,
            visible: true,
            region: Region::Nodes(nodes),
            kind,
            amplitude: None,
        };
        let cubes = || {
            let mut model = pulled_cubes(0.0);
            let material = &mut model.fe.materials[0];
            material.density = Some(7.85e-9);
            material.conductivity = Some(50.0);
            material.specific_heat = Some(4.6e8);
            model
        };
        // Eigenmodes of one analysis, the response over time from them in another one.
        let mut frequency = Step::new_frequency("Modes");
        if let plx_model::StepKind::Frequency(f) = &mut frequency.kind {
            f.storage = true;
        }
        frequency.boundary_conditions = vec![bc(
            "Fix",
            vec![1, 2, 3, 4],
            BoundaryKind::displacement([Some(0.0); 6]),
        )];
        let mut modal = Step::new_modal_dynamics("Shake");
        modal.boundary_conditions = frequency.boundary_conditions.clone();
        modal.loads.push(Load {
            distribution: None,
            name: "Push".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![13, 14, 15, 16]),
            kind: LoadKind::force([0.0, 1.0, 0.0]),
            amplitude: None,
            factor_amplitude: None,
        });
        let mut both = cubes();
        both.fe.steps = vec![frequency.clone(), modal.clone()];
        solve_as(&both, &dir.join("Both"), "Both", &Default::default());
        let mut source = cubes();
        source.fe.steps = vec![frequency];
        solve_as(&source, &dir.join("Modes"), "Modes", &Default::default());
        let mut target = cubes();
        target.fe.steps = vec![modal];
        target.fe.properties.stored_modes = Some(dir.join("Modes/Modes.eig"));
        assert!(
            !(target.findings().iter()).any(|f| f.problem == plx_model::Problem::NoStoredModes)
        );
        solve_as(&target, &dir.join("Shake"), "Shake", &Default::default());
        let alone = plx_io::frd::read_frd(&dir.join("Shake/Shake.frd")).unwrap();
        let joint = plx_io::frd::read_frd(&dir.join("Both/Both.frd")).unwrap();
        let last = |frd: &plx_io::frd::FrdImport| {
            format!("{:?}", frd.increments.last().unwrap().displacements())
        };
        assert_eq!(last(&alone), last(&joint));
        // Temperatures of a heat transfer analysis as the start of another one.
        let mut heat = Step::new_heat_transfer("Heat");
        heat.boundary_conditions = vec![
            bc("Hot", vec![1, 2, 3, 4], BoundaryKind::Temperature(100.0)),
            bc(
                "Cold",
                vec![13, 14, 15, 16],
                BoundaryKind::Temperature(20.0),
            ),
        ];
        let mut thermal = cubes();
        thermal.fe.steps = vec![heat];
        solve_as(
            &thermal,
            &dir.join("Thermal"),
            "Thermal",
            &Default::default(),
        );
        let mut fe = cubes().fe;
        fe.properties.initial_temperatures = Some((dir.join("Thermal/Thermal.frd"), 1));
        let keyword = stage_linked_results(&fe, &dir, "Start").unwrap().unwrap();
        let lines: Vec<&str> = keyword.lines().collect();
        assert_eq!(lines[1], "*Initial conditions, Type=Temperature");
        assert_eq!(lines.len(), 2 + 16);
        let value = |node: &str| -> f64 {
            let line = lines
                .iter()
                .find(|l| l.starts_with(&format!("{node}, ")))
                .unwrap();
            line.split(", ").nth(1).unwrap().parse().unwrap()
        };
        assert!((value("1") - 100.0).abs() < 1e-3 && (value("16") - 20.0).abs() < 1e-3);
        assert!((value("8") - 100.0 + 80.0 / 3.0).abs() < 1e-2, "{keyword}");
        let input = before_first_step("*Node\n1, 0, 0, 0\n*Step\n*Static\n", &keyword);
        assert!(input.find("*Initial").unwrap() < input.find("*Step").unwrap());
        std::fs::remove_dir_all(dir).unwrap();
    }

    /// Past the line limit the oldest lines go and the new ones still reach the view: it
    /// stood still once the output had as many lines as it keeps.
    #[test]
    fn long_output_keeps_coming_in() {
        let mut output: Vec<String> = Vec::new();
        let mut dropped = 0;
        let mut view = RunView::default();
        for chunk in 0..5 {
            let new = (0..MAX_OUTPUT_LINES / 2).map(|i| format!("{chunk} {i}"));
            output.extend(new);
            dropped += keep_last(&mut output, MAX_OUTPUT_LINES);
            view.copy(&output, dropped + output.len());
            assert_eq!(view.lines, output);
        }
        assert_eq!(dropped, 3 * MAX_OUTPUT_LINES / 2);
        assert_eq!(view.copied - view.lines.len(), dropped);
        // A new solve starts over.
        view.copy(&["first".to_owned()], 1);
        assert_eq!(view.lines, ["first"]);
    }
}
