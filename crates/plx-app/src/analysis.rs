//! Running the analysis: writes the input file, starts the solver (see [`plx_solve::Run`])
//! and shows PrePoMax's monitor window with the solver output, progress and the button to
//! open the results.

use std::path::PathBuf;
use std::time::Instant;

use plx_solve::{JobStatus, Run};

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
    /// The item of the model each item of the solver's analysis came from, which leads the
    /// solver's diagnostics back to the tree.
    origins: plx_model::Origins,
}

/// Why an analysis did not start, with the item of the model it is about when the lowering
/// named one.
#[derive(Debug)]
pub struct StartError {
    pub message: String,
    pub item: Option<plx_model::ModelItem>,
}

impl StartError {
    /// The error as a finding on its item, for the tree.
    pub fn finding(&self) -> Option<plx_model::Finding> {
        let item = self.item?;
        Some(plx_model::Finding::new(
            item,
            plx_model::Problem::NotLowered,
            self.message.clone(),
        ))
    }
}

impl From<String> for StartError {
    fn from(message: String) -> Self {
        Self {
            message,
            item: None,
        }
    }
}

impl From<&str> for StartError {
    fn from(message: &str) -> Self {
        message.to_owned().into()
    }
}

impl std::fmt::Display for StartError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
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

/// The lowered analysis for the solver, with its input text for reference.
pub fn prepare(lowered: plx_io::inp::Lowered) -> plx_solve::Prepared {
    plx_solve::Prepared {
        text: lowered.text(),
        analysis: lowered.analysis,
        warnings: lowered.warnings,
    }
}

/// Initial temperatures taken from the results of another analysis.
#[derive(Debug)]
pub struct InitialTemperatures {
    /// Where they come from, in words.
    pub source: String,
    /// The temperature of each node.
    pub values: Vec<(u32, f64)>,
}

impl InitialTemperatures {
    /// Sets them as the initial conditions of `analysis`, after those of the model. Every
    /// node has to be one of the model's.
    pub fn apply(&self, analysis: &mut plxs_core::Analysis) -> Result<(), String> {
        let model = &mut analysis.model;
        if let Some(&(node, _)) =
            (self.values.iter()).find(|(n, _)| model.nodes.index(*n).is_none())
        {
            return Err(format!(
                "Initial temperatures not set: node {node} of {} is not in the model.",
                self.source
            ));
        }
        let values = self.values.iter();
        (model.initial.temperatures)
            .extend(values.map(|&(node, value)| (plxs_core::mesh::NodeRef::Node(node), value)));
        Ok(())
    }
}

/// Prepares what the model takes from the results of other analyses: the stored eigenmodes
/// are copied to `<name>.eig`, where the solver reads them; the initial temperatures are
/// read from their results file.
pub fn stage_linked_results(
    fe: &plx_model::FeModel,
    dir: &std::path::Path,
    name: &str,
) -> Result<Option<InitialTemperatures>, String> {
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
    Ok(Some(InitialTemperatures {
        source: format!("step {step} of {}", file.display()),
        values: (results.mesh.node_ids().iter().copied())
            .zip(temperatures.values.iter().map(|&t| f64::from(t)))
            .collect(),
    }))
}

impl Analysis {
    /// Writes the input file of the analysis `name` into `work_dir` and starts the solver on
    /// it. With `check_model` the solver only checks the model (PrePoMax's "Check Model").
    /// `continuation` tells which analyses continue from its state, whose sets its input
    /// file holds too.
    pub fn start(
        solver: &Solver,
        model: &Model,
        check_model: bool,
        name: &str,
        work_dir: &std::path::Path,
        continuation: &crate::links::Continuation,
    ) -> Result<Self, StartError> {
        if model.fe.steps.is_empty() && !check_model {
            return Err(
                "Analysis not started: the model has no step (FE Model > Steps > Step).".into(),
            );
        }
        if !check_model && !model.fe.steps.iter().any(|s| s.active) {
            return Err("Analysis not started: all steps are deactivated.".into());
        }
        let heading = format!("Nodalis: {}", model.file_name());
        let mut fe = model.fe.clone();
        let job_name = job_name(name);
        stage_global_results(&mut fe, work_dir, &job_name)?;
        let initial = stage_linked_results(&fe, work_dir, &job_name)?;
        // The sets of the analyses that continue from its states go into the input file too.
        // The model check reads the whole input with its steps: it checks every step's
        // supports.
        let dependent = &continuation.dependent_steps;
        let mut lowered =
            plx_io::inp::lower_with_sets_unvalidated(&model.mesh, &fe, &heading, dependent)
                .map_err(|e| StartError {
                    message: format!("Analysis not set up: {e}"),
                    item: e.item,
                })?;
        if let Some(initial) = &initial {
            initial.apply(&mut lowered.analysis)?;
        }
        let overlaps = std::mem::take(&mut lowered.resolved_overlaps);
        let origins = std::mem::take(&mut lowered.origins);
        let prepared = prepare(lowered);
        let copied = copy_result_files(&fe, work_dir);
        let job = if check_model {
            Run::check_analysis(solver.threads, work_dir, &job_name, prepared)
        } else {
            Run::start_analysis(solver.threads, work_dir, &job_name, prepared)
        }
        .map_err(|e| format!("Solver not started: {e}"))?;
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
        if let Some(initial) = &initial {
            output.push(format!("Initial temperatures: {}", initial.source));
        }
        // Constraints the input file writer had to change so that they do not bind a degree of
        // freedom twice.
        output.extend(
            (overlaps.into_iter()).map(|line| format!("Overlapping constraints resolved: {line}")),
        );
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
            origins,
        })
    }

    /// The item of the model each item of the solver's analysis came from.
    pub fn origins(&self) -> &plx_model::Origins {
        &self.origins
    }

    /// Adds a line of nodalis's own to the monitor output.
    pub fn note(&mut self, line: String) {
        self.collect([line]);
    }

    /// What the solver found, typed, once the run has finished.
    pub fn diagnostics(&self) -> Option<Vec<plx_solve::Diagnostic>> {
        self.job.diagnostics()
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
        self.finished = Some(self.started.elapsed().as_secs_f32());
        self.status = status;
        Some(status)
    }

    /// How long the run has taken so far, or took.
    pub fn seconds(&self) -> f32 {
        (self.finished).unwrap_or_else(|| self.started.elapsed().as_secs_f32())
    }

    /// Where the run is: step, increment, attempt, iteration and time, once the solver tells.
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
        let mut model = Model::new(std::path::Path::new("cubes.ndl"), mesh);
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

    /// The results of a run as the project reads them.
    fn results(path: &std::path::Path) -> plx_io::frd::FrdImport {
        plx_io::solver::import(&plx_results::SolverResults::read(path).unwrap())
    }

    fn solve_as(
        model: &Model,
        dir: &std::path::Path,
        name: &str,
        continuation: &crate::links::Continuation,
    ) -> Vec<String> {
        let solver = Solver {
            threads: 1,
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

    /// Check Model with the Nodalis solver: cubes held in X only can slide and turn; the
    /// solver's rank test names the part and the free motions, attached to each step.
    #[test]
    fn the_built_in_model_check_names_the_free_part() {
        let mut model = pulled_cubes(0.03);
        for step in &mut model.fe.steps {
            step.boundary_conditions.truncate(1);
            step.boundary_conditions[0].kind =
                plx_model::BoundaryKind::displacement([Some(0.0), None, None, None, None, None]);
        }
        let solver = Solver::default();
        let dir = std::env::temp_dir().join(format!("nodalis-check-{}", std::process::id()));
        let mut analysis =
            Analysis::start(&solver, &model, true, "Cubes", &dir, &Default::default()).unwrap();
        let status = loop {
            if let Some(status) = analysis.poll() {
                break status;
            }
            std::thread::sleep(std::time::Duration::from_millis(20));
        };
        assert_eq!(status, JobStatus::Failed, "{:?}", analysis.output());
        let diagnostics = analysis.diagnostics().expect("typed diagnostics");
        let findings = plx_model::diagnose_solver_check(
            &diagnostics,
            &model.mesh,
            &model.fe,
            analysis.origins(),
        );
        let at = |item| {
            findings
                .iter()
                .find(|f| f.item == item && f.problem == plx_model::Problem::RigidBodyMotion)
        };
        let part = at(plx_model::ModelItem::Part(0)).expect("the part is named");
        assert!(part.detail.contains("Solid-1"), "{}", part.detail);
        // Held in X at one end: Y, Z and the turns about Y and Z (X turns too: one line
        // of nodes would hold it, a face holds it in X only).
        assert!(part.detail.contains("displacement in Y"), "{}", part.detail);
        for step in 0..3 {
            assert!(
                at(plx_model::ModelItem::BoundaryConditions(step)).is_some(),
                "{findings:?}"
            );
        }
        let output = analysis.output().join("\n");
        assert!(
            output.contains("*ERROR in model check (step 1)"),
            "{output}"
        );
    }

    /// The solver names the element it found without a section; the finding points at its
    /// part, which the section leaves out.
    #[test]
    fn the_model_check_points_at_the_part_without_a_section() {
        let mut model = pulled_cubes(0.03);
        model.mesh.parts[0].elements = vec![1, 2];
        model.mesh.parts.push(plx_mesh::Part {
            name: "Solid-2".into(),
            elements: vec![3],
        });
        let solver = Solver::default();
        let dir = std::env::temp_dir().join(format!("nodalis-unsectioned-{}", std::process::id()));
        let mut analysis =
            Analysis::start(&solver, &model, true, "Cubes", &dir, &Default::default()).unwrap();
        let status = loop {
            if let Some(status) = analysis.poll() {
                break status;
            }
            std::thread::sleep(std::time::Duration::from_millis(20));
        };
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(status, JobStatus::Failed, "{:?}", analysis.output());
        let diagnostics = analysis.diagnostics().expect("typed diagnostics");
        let findings = plx_model::diagnose_solver_check(
            &diagnostics,
            &model.mesh,
            &model.fe,
            analysis.origins(),
        );
        let part = (findings.iter())
            .find(|f| f.item == plx_model::ModelItem::Part(1))
            .unwrap_or_else(|| panic!("{findings:?}"));
        assert!(
            part.detail.contains("element 3 has no section"),
            "{}",
            part.detail
        );
    }

    /// A model the analysis cannot be set up with names the item, for the tree.
    #[test]
    fn a_model_that_cannot_be_set_up_names_the_item() {
        let mut model = pulled_cubes(0.03);
        model.fe.steps[1].boundary_conditions[1].region = plx_model::Region::Nodes(Vec::new());
        let dir = std::env::temp_dir().join(format!("nodalis-not-set-up-{}", std::process::id()));
        let error = Analysis::start(
            &Solver::default(),
            &model,
            false,
            "Cubes",
            &dir,
            &Default::default(),
        )
        .err()
        .expect("the empty region stops the lowering");
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(
            error.item,
            Some(plx_model::ModelItem::BoundaryCondition(1, 1)),
            "{error}"
        );
        let finding = error.finding().unwrap();
        assert_eq!(finding.problem, plx_model::Problem::NotLowered);
    }

    #[test]
    fn an_analysis_continuing_another_solves_all_its_steps() {
        let dir = std::env::temp_dir().join(format!("nodalis-continue-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        // The continuing analysis holds the end with a boundary condition of its own, on a
        // set the first analysis does not use itself: its input file has that set too.
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
        assert!(
            source_inp.to_ascii_uppercase().contains("HOLD"),
            "{source_inp}"
        );
        assert_eq!(source_inp.matches("*Step").count(), 2);
        let continuation = crate::links::Continuation {
            upstream: vec![(dir.join("Source"), "Source".into())],
            ..Default::default()
        };
        let output = solve_as(&target, &dir.join("Target"), "Target", &continuation);
        assert!(output.iter().any(|l| l.trim() == "STEP 3"), "{output:?}");
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn eigenmodes_and_temperatures_come_from_other_analyses() {
        use plx_model::{BoundaryCondition, BoundaryKind, Load, LoadKind, Region, Step};
        let dir = std::env::temp_dir().join(format!("nodalis-linked-{}", std::process::id()));
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
        let alone = results(&dir.join("Shake/Shake.ndlr"));
        let joint = results(&dir.join("Both/Both.ndlr"));
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
        // The results file the project writes from the solver's for linking.
        let solved = plx_results::SolverResults::read(&dir.join("Thermal/Thermal.ndlr")).unwrap();
        plx_io::solver::export_calculix(&solved, &dir.join("Thermal/Thermal.frd")).unwrap();
        let mut fe = cubes().fe;
        fe.properties.initial_temperatures = Some((dir.join("Thermal/Thermal.frd"), 1));
        let initial = stage_linked_results(&fe, &dir, "Start").unwrap().unwrap();
        assert_eq!(initial.values.len(), 16);
        let value = |node: u32| initial.values.iter().find(|(n, _)| *n == node).unwrap().1;
        assert!((value(1) - 100.0).abs() < 1e-3 && (value(16) - 20.0).abs() < 1e-3);
        assert!((value(8) - 100.0 + 80.0 / 3.0).abs() < 1e-2, "{initial:?}");
        let mut lowered = plx_io::inp::lower_unvalidated(&cubes().mesh, &fe, "Start").unwrap();
        initial.apply(&mut lowered.analysis).unwrap();
        let temperatures = &lowered.analysis.model.initial.temperatures;
        assert_eq!(temperatures.len(), 16);
        assert!(
            lowered
                .text()
                .contains("*Initial conditions, Type=Temperature")
        );
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
