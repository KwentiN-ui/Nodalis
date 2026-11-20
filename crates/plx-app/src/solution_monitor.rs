//! Solving from the project, as Ansys Workbench's Update Project does: every solve, from
//! Update Project or from Solve in an analysis window, meshes and solves a copy of the
//! analysis' data in the background, and the project window's solution monitor shows the
//! runs with their solver output. An open window shows the run in its Analysis tab and
//! takes the new mesh and results once it is done.

use std::path::PathBuf;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::time::{Duration, Instant};

use plx_io::project::{BoundResults, ProjectAnalysis};
use plx_model::{Finding, Severity};
use plx_solve::JobStatus;

use crate::analysis::Analysis;
use crate::links::Continuation;
use crate::meshing::MeshingJob;
use crate::model::Model;
use crate::settings::Solver;
use crate::status::CellState;

/// What a background solve is doing.
enum Stage {
    /// Building the model on a worker thread, and finding the default equation solver.
    Loading(Receiver<Loaded>),
    Meshing(Box<MeshingJob>),
    /// Writing the input file and starting the solver on a worker thread, which lowers the
    /// whole model and finds the steps it can take over from the last run.
    Starting(Receiver<Started>),
    Running,
    /// Keeping the results in the project; compressing them takes a while.
    Keeping(Receiver<Result<BoundResults, String>>),
    Done(Outcome),
}

struct Loaded {
    model: Model,
    /// Fingerprint of the mesh it started from.
    mesh_print: u64,
    solver: Solver,
    next: Next,
}

/// What a solve does once its model is built, found on the worker thread: finding the state
/// of the analysis takes fingerprints of the whole model.
enum Next {
    /// Mesh the parts first: all of them (`None`) or those out of date.
    Mesh(Option<Vec<String>>),
    /// Nothing to solve: the results are up to date.
    UpToDate,
    /// The solver was started, or could not be.
    Started(Box<Result<Analysis, crate::analysis::StartError>>),
}

/// The model back from the worker that put the new meshes into it and started the solver.
struct Started {
    model: Model,
    /// The lines of the meshes put into the model, none without meshing; an error stops
    /// the solve.
    merged: Result<Vec<String>, String>,
    /// `None` when the meshes could not be put into the model.
    analysis: Option<Result<Analysis, crate::analysis::StartError>>,
}

/// Writes the input file of `model` and starts the solver: the work of the worker threads
/// before the solver runs. Solve Again (`force`) computes every step, also those that did not
/// change.
fn start_solver(
    model: &Model,
    solver: &Solver,
    findings: &[Finding],
    work: &Work,
) -> Result<Analysis, crate::analysis::StartError> {
    if work.force {
        crate::analysis::forget_steps(&work.dir, &crate::analysis::job_dir(work.id));
    }
    let mut analysis = Analysis::start(
        solver,
        model,
        false,
        &crate::analysis::job_dir(work.id),
        &work.dir,
        &work.continuation,
    )?;
    // Problems the checks found go first, so that an abort is explained even before the
    // solver says anything.
    for finding in findings.iter().filter(|f| f.severity() == Severity::Error) {
        analysis.note(format!(
            "Model check: {}: {}",
            finding.problem.title(),
            finding.detail
        ));
    }
    Ok(analysis)
}

/// What the worker threads of a solve need to start the solver.
#[derive(Clone)]
struct Work {
    id: u64,
    dir: PathBuf,
    continuation: Continuation,
    force: bool,
}

/// What a built model needs, by the state of the analysis: meshing, nothing, or solving.
fn next_for(model: &Model, solver: &Solver, findings: &[Finding], work: &Work) -> Next {
    let status = crate::status::analysis_status(
        model.geometry.as_ref(),
        model.mesh(),
        &model.fe,
        model.origins(),
        model.bound_results.as_ref(),
        // The project makes the mesh blocks it takes its mesh from before it starts.
        &Default::default(),
        false,
        None,
    );
    match status.mesh {
        CellState::Missing => return Next::Mesh(None),
        CellState::OutOfDate => return Next::Mesh(Some(status.stale_parts.clone())),
        _ => {}
    }
    if !status.needs_solve() && !work.force {
        return Next::UpToDate;
    }
    Next::Started(Box::new(start_solver(model, solver, findings, work)))
}

/// How a background solve ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Solved and kept, or up to date already.
    Solved,
    /// The solver reported errors, but its results are kept.
    SolvedWithErrors,
    Failed,
    Stopped,
    /// Stopped by the user, who kept the results of the increments done until then.
    StoppedWithResults,
}

impl Outcome {
    pub fn text(self) -> &'static str {
        match self {
            Self::Solved => "solved",
            Self::SolvedWithErrors => "finished with errors, results kept",
            Self::Failed => "failed",
            Self::Stopped => "stopped",
            Self::StoppedWithResults => "stopped, results so far kept",
        }
    }
}

/// What a finished background solve changed in the analysis.
pub struct Solved {
    /// The new mesh with the fingerprints of what its parts were made from, when the solve
    /// meshed the geometry first.
    pub meshed: Option<(plx_mesh::FeMesh, plx_model::PartOrigins)>,
    pub results: Option<BoundResults>,
    /// Problems the solver reported when the run failed.
    pub findings: Vec<Finding>,
    /// Fingerprint of the mesh the solve started from: what it made belongs to the
    /// analysis only while its mesh is still that one.
    pub mesh_print: Option<u64>,
}

/// An analysis meshed and solved from a copy of its data.
pub struct BackgroundSolve {
    pub id: u64,
    pub name: String,
    stage: Stage,
    model: Option<Model>,
    solver: Solver,
    work_dir: PathBuf,
    continuation: Continuation,
    analysis: Option<Analysis>,
    /// Lines before the solver runs (loading, meshing, model checks) and after (keeping the
    /// results), around the solver output.
    before: Vec<String>,
    after: Vec<String>,
    started: Instant,
    finished: Option<f32>,
    meshed: bool,
    results: Option<BoundResults>,
    findings: Vec<Finding>,
    mesh_print: Option<u64>,
    /// Runs the solver even if the results are up to date: Solve Again.
    force: bool,
    /// The solver was stopped keeping the results it had: they are kept as those of a
    /// stopped run.
    stopped_keeping: bool,
}

impl BackgroundSolve {
    /// Starts solving the analysis `data` of the project at `path` in `work_dir`; the model
    /// is built on a worker thread.
    pub fn start(
        ctx: &egui::Context,
        path: PathBuf,
        data: ProjectAnalysis,
        solver: Solver,
        work_dir: PathBuf,
        continuation: Continuation,
        force: bool,
    ) -> Self {
        let name = data.name.clone();
        let (sender, receiver) = std::sync::mpsc::channel();
        let ctx = ctx.clone();
        let probed = solver.clone();
        let work = Work {
            id: data.id,
            dir: work_dir.clone(),
            continuation: continuation.clone(),
            force,
        };
        std::thread::spawn(move || {
            let ProjectAnalysis {
                geometry,
                mesh,
                model: fe,
                origins,
                results,
                ..
            } = data;
            let mut model = Model::new(&path, mesh);
            model.fe = fe;
            model.geometry = geometry;
            model.set_origins(origins);
            model.bound_results = results;
            let findings = if model.mesh().element_count() > 0 {
                model.findings()
            } else {
                Vec::new()
            };
            let mesh_print = model.mesh().fingerprint();
            let next = next_for(&model, &probed, &findings, &work);
            let loaded = Loaded {
                model,
                mesh_print,
                solver: probed,
                next,
            };
            // A solve stopped meanwhile does not take the solver started for it.
            if let Err(std::sync::mpsc::SendError(loaded)) = sender.send(loaded)
                && let Next::Started(started) = loaded.next
                && let Ok(mut analysis) = *started
            {
                analysis.kill();
            }
            ctx.request_repaint();
        });
        Self {
            before: vec![format!("{name}: solving in {}", work_dir.display())],
            id: data.id,
            name,
            stage: Stage::Loading(receiver),
            model: None,
            solver,
            work_dir,
            continuation,
            analysis: None,
            after: Vec::new(),
            started: Instant::now(),
            finished: None,
            meshed: false,
            results: None,
            findings: Vec::new(),
            mesh_print: None,
            force,
            stopped_keeping: false,
        }
    }

    /// Whether it still works: loading, meshing, running or keeping the results.
    pub fn active(&self) -> bool {
        !matches!(self.stage, Stage::Done(_))
    }

    pub fn outcome(&self) -> Option<Outcome> {
        match self.stage {
            Stage::Done(outcome) => Some(outcome),
            _ => None,
        }
    }

    /// What it does now, for the monitor.
    pub fn state_text(&self) -> &'static str {
        match &self.stage {
            Stage::Loading(_) => "preparing",
            Stage::Meshing(_) => "meshing",
            Stage::Starting(_) => "starting",
            Stage::Running if self.is_stopping() => "stopping, writing the results so far",
            Stage::Running => "running",
            Stage::Keeping(_) => "keeping results",
            Stage::Done(outcome) => outcome.text(),
        }
    }

    /// When it started, which tells it from the next solve of the same analysis.
    pub fn started(&self) -> Instant {
        self.started
    }

    pub fn seconds(&self) -> f32 {
        (self.finished).unwrap_or_else(|| self.started.elapsed().as_secs_f32())
    }

    /// Where the solver is, while it runs.
    pub fn progress(&self) -> Option<String> {
        let analysis = self.analysis.as_ref()?;
        analysis.is_running().then(|| analysis.progress()).flatten()
    }

    /// The output so far: nodalis's lines around the solver's.
    pub fn output(&self) -> Vec<&str> {
        let solver = self.analysis.as_ref().map_or(&[][..], |a| a.output());
        (self.before.iter().chain(solver).chain(&self.after))
            .map(String::as_str)
            .collect()
    }

    /// The number of lines [`Self::output`] has had, also those dropped since.
    pub fn output_total(&self) -> usize {
        let solver = (self.analysis.as_ref()).map_or(0, |a| a.dropped() + a.output().len());
        self.before.len() + solver + self.after.len()
    }

    /// Where the time of the solver went, once it has finished with results.
    pub fn profile(&self) -> Option<plx_solve::JobProfile> {
        self.analysis.as_ref()?.profile()
    }

    /// How the solver converges, once it runs.
    pub fn convergence(&self) -> Option<plx_solve::monitor::LiveLog> {
        self.analysis.as_ref().map(Analysis::convergence)
    }

    /// The steps solved, a line each, for the solver report.
    pub fn step_lines(&self) -> &[String] {
        self.analysis.as_ref().map_or(&[], Analysis::step_lines)
    }

    /// The lines dropped from the front of the solver's output in a long run.
    pub fn dropped(&self) -> usize {
        self.analysis
            .as_ref()
            .map_or(0, crate::analysis::Analysis::dropped)
    }

    /// Stops it: the solver is stopped, a model being built or meshed is dropped.
    pub fn stop(&mut self) {
        if !self.active() {
            return;
        }
        if let Some(analysis) = (self.analysis.as_mut()).filter(|a| a.is_running()) {
            analysis.kill();
        }
        // Dropping the meshing job kills the mesher.
        if matches!(self.stage, Stage::Meshing(_)) {
            self.after.push(crate::meshing::CANCELLED.into());
        }
        self.after.push("Stopped".into());
        self.finish(Outcome::Stopped);
    }

    /// The increments the solver finished so far: a stop can keep them as results.
    pub fn increments_done(&self) -> u32 {
        (self.analysis.as_ref())
            .filter(|a| a.is_running())
            .map_or(0, Analysis::increments_done)
    }

    /// Whether the solver was stopped and writes the results it has.
    pub fn is_stopping(&self) -> bool {
        (self.analysis.as_ref()).is_some_and(Analysis::is_stopping)
    }

    /// Stops the solver keeping the increments it finished: the solve goes on until they
    /// are written and kept, then ends as [`Outcome::StoppedWithResults`]. Without a
    /// running solver (loading, meshing), it is [`BackgroundSolve::stop`].
    pub fn stop_keeping_results(&mut self) {
        match (self.analysis.as_mut())
            .filter(|a| a.is_running() && matches!(self.stage, Stage::Running))
        {
            Some(analysis) => {
                analysis.stop_keeping_results();
                self.after
                    .push("Stopped; writing the results of the increments done".into());
            }
            None => self.stop(),
        }
    }

    /// What it changed in the analysis, once done; taken once.
    pub fn take_solved(&mut self) -> Option<Solved> {
        if self.active() {
            return None;
        }
        let meshed =
            (self.meshed.then(|| self.model.take()).flatten()).map(Model::into_mesh_with_origins);
        self.meshed = false;
        Some(Solved {
            meshed,
            results: self.results.take(),
            findings: std::mem::take(&mut self.findings),
            mesh_print: self.mesh_print.take(),
        })
    }

    fn finish(&mut self, outcome: Outcome) {
        self.finished = Some(self.started.elapsed().as_secs_f32());
        self.stage = Stage::Done(outcome);
    }

    fn fail(&mut self, line: String) {
        self.before.push(line);
        self.finish(Outcome::Failed);
    }

    /// Works on: takes the built model or the new mesh, polls the solver and keeps its
    /// results once it ends.
    pub fn poll(&mut self, ctx: &egui::Context) {
        match &mut self.stage {
            Stage::Loading(receiver) => match receiver.try_recv() {
                Ok(loaded) => {
                    let Loaded {
                        model,
                        mesh_print,
                        solver,
                        next,
                    } = loaded;
                    self.model = Some(model);
                    self.mesh_print = Some(mesh_print);
                    self.solver = solver;
                    self.go_on(ctx, next);
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => self.fail("The model was not built.".into()),
            },
            Stage::Meshing(job) => {
                let Some(result) = job.poll() else {
                    return;
                };
                let (geometry, started) = (job.geometry.clone(), job.started);
                match result {
                    // A solve needs every part meshed.
                    Ok(generated) if !generated.failed.is_empty() => {
                        self.meshing_failed(generated.failed.join("\n"))
                    }
                    Ok(generated) => self.run(ctx, Some((geometry, generated, started))),
                    Err(error) => self.meshing_failed(error),
                }
            }
            Stage::Starting(receiver) => match receiver.try_recv() {
                Ok(Started {
                    model,
                    merged,
                    analysis,
                }) => {
                    self.model = Some(model);
                    match merged {
                        Ok(lines) => {
                            self.meshed |= !lines.is_empty();
                            self.before.extend(lines);
                        }
                        Err(error) => return self.meshing_failed(error),
                    }
                    match analysis {
                        Some(started) => self.take_started(started),
                        None => self.fail("The solver was not started.".into()),
                    }
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => {
                    self.fail("The solver was not started.".into());
                }
            },
            Stage::Running => self.poll_run(),
            Stage::Keeping(receiver) => match receiver.try_recv() {
                Ok(Ok(bound)) => {
                    let with_errors = bound.with_errors;
                    self.after.push(format!(
                        "Results of {} kept, {} in the project file once it is saved",
                        self.name,
                        crate::app::file_size(bound.stored_size())
                    ));
                    self.results = Some(bound);
                    self.finish(if self.stopped_keeping {
                        Outcome::StoppedWithResults
                    } else if with_errors {
                        Outcome::SolvedWithErrors
                    } else {
                        Outcome::Solved
                    });
                }
                Ok(Err(error)) => {
                    self.after.push(format!("Results not kept: {error}"));
                    self.finish(Outcome::Failed);
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => {
                    self.after.push("Results not kept".into());
                    self.finish(Outcome::Failed);
                }
            },
            Stage::Done(_) => return,
        }
        if self.active() {
            ctx.request_repaint_after(Duration::from_millis(200));
        }
    }

    /// Goes on with what the worker found the built model needs: meshing first, nothing,
    /// or the solver it started.
    fn go_on(&mut self, ctx: &egui::Context, next: Next) {
        match next {
            Next::Mesh(parts) => {
                let Some(model) = &self.model else {
                    return;
                };
                let Some(geometry) = model.geometry.clone() else {
                    return self.fail("The model has no mesh and no geometry to mesh.".into());
                };
                let what = match &parts {
                    Some(parts) => parts.join(", "),
                    None => geometry.source.clone(),
                };
                self.before.push(format!("Meshing {what} …"));
                self.stage = Stage::Meshing(Box::new(MeshingJob::start(
                    geometry,
                    model.fe.matching_faces(),
                    parts,
                    ctx,
                )));
            }
            Next::UpToDate => {
                self.before
                    .push(format!("{} is up to date; nothing to solve.", self.name));
                self.finish(Outcome::Solved);
            }
            Next::Started(started) => self.take_started(*started),
        }
    }

    fn meshing_failed(&mut self, error: String) {
        let error = if error.starts_with("Meshing") {
            error
        } else {
            format!("Meshing failed: {error}")
        };
        self.fail(error);
    }

    /// The solver runs, or could not be started.
    fn take_started(&mut self, started: Result<Analysis, crate::analysis::StartError>) {
        match started {
            Ok(analysis) => {
                self.analysis = Some(analysis);
                self.stage = Stage::Running;
            }
            Err(error) => {
                self.findings = error.finding().into_iter().collect();
                self.fail(error.message);
            }
        }
    }

    /// Puts the new meshes into the model, writes the input file and starts the solver, on
    /// a worker thread: for a large model, each takes long enough to stop the window.
    fn run(
        &mut self,
        ctx: &egui::Context,
        meshed: Option<(plx_model::Geometry, crate::meshing::PartMeshes, Instant)>,
    ) {
        let Some(mut model) = self.model.take() else {
            return;
        };
        let (sender, receiver) = std::sync::mpsc::channel();
        let solver = self.solver.clone();
        let work = Work {
            id: self.id,
            dir: self.work_dir.clone(),
            continuation: self.continuation.clone(),
            force: self.force,
        };
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let merged = match meshed {
                Some((geometry, generated, started)) => {
                    crate::app::merge_part_meshes(&mut model, &geometry, generated, Some(started))
                }
                None => Ok(Vec::new()),
            };
            let analysis = merged.is_ok().then(|| {
                let findings = model.findings();
                start_solver(&model, &solver, &findings, &work)
            });
            let started = Started {
                model,
                merged,
                analysis,
            };
            // A solve stopped meanwhile does not take the solver started for it.
            if let Err(std::sync::mpsc::SendError(started)) = sender.send(started)
                && let Some(Ok(mut analysis)) = started.analysis
            {
                analysis.kill();
            }
            ctx.request_repaint();
        });
        self.stage = Stage::Starting(receiver);
    }

    fn poll_run(&mut self) {
        let Some(analysis) = &mut self.analysis else {
            return;
        };
        let Some(status) = analysis.poll() else {
            return;
        };
        self.after
            .push(format!("Analysis {}", Analysis::status_text(status)));
        // The Nodalis solver tells the causes of a failed run, typed.
        if let Some(diagnostics) = analysis.diagnostics()
            && let Some(model) = &self.model
        {
            let findings = plx_model::diagnose_solver_check(
                &diagnostics,
                model.mesh(),
                &model.fe,
                analysis.origins(),
            );
            let mut shown = std::collections::HashSet::new();
            for finding in &findings {
                // What does not explain the failure is told as a warning below.
                if !plx_model::explains_failure(finding, &findings) {
                    continue;
                }
                if shown.insert((finding.problem, finding.detail.clone())) {
                    self.after.push(format!(
                        "Cause: {}: {}. {}",
                        finding.problem.title(),
                        finding.detail,
                        finding.problem.fix()
                    ));
                }
            }
            self.findings = findings;
        } else if matches!(status, JobStatus::Failed | JobStatus::FailedWithResults)
            && let Some(model) = &self.model
        {
            let findings = plx_model::diagnose_solver_output(analysis.output(), model.mesh());
            for finding in (findings.iter()).filter(|f| f.item == plx_model::ModelItem::Analysis) {
                self.after.push(format!(
                    "Possible cause: {}. {}",
                    finding.problem.title(),
                    finding.problem.fix()
                ));
            }
            self.findings = findings;
        }
        // Warnings that point to wrong results, after any run.
        let warnings = plx_model::diagnose_solver_warnings(analysis.output());
        for finding in &warnings {
            self.after.push(format!(
                "Warning: {}: {}. {}",
                finding.problem.title(),
                finding.detail,
                finding.problem.fix()
            ));
        }
        self.findings.extend(warnings);
        let results = analysis.results().filter(|_| {
            matches!(
                status,
                JobStatus::Completed | JobStatus::FailedWithResults | JobStatus::StoppedWithResults
            )
        });
        let Some(results_file) = results else {
            let outcome = if status == JobStatus::Killed {
                Outcome::Stopped
            } else {
                Outcome::Failed
            };
            return self.finish(outcome);
        };
        // Kept as the analysis window keeps them, beside the GUI.
        let (sender, receiver) = std::sync::mpsc::channel();
        // The eigenmodes of a frequency step go with them, for analyses that take them; the
        // built-in solver's results file holds the fields and the history output.
        let extensions: &[&str] = if plx_results::solver::is_solver_results(&results_file) {
            &["inp", "eig", plx_results::wall_loads::EXTENSION]
        } else {
            &["inp", "dat", "eig"]
        };
        let mut files = vec![results_file.clone()];
        files.extend(extensions.iter().map(|e| results_file.with_extension(e)));
        let (name, input) = (self.name.clone(), analysis.input);
        let steps = analysis.steps.clone();
        self.stopped_keeping = status == JobStatus::StoppedWithResults;
        // A stopped run's last step may not be finished, as that of a failed run.
        let with_errors = matches!(
            status,
            JobStatus::FailedWithResults | JobStatus::StoppedWithResults
        );
        std::thread::spawn(move || {
            let bound = plx_io::project::bind_results(&name, &files, with_errors)
                .map(|mut bound| {
                    bound.input = Some(input);
                    bound.steps = steps;
                    bound
                })
                .map_err(|e| e.to_string());
            let _ = sender.send(bound);
        });
        self.after
            .push(format!("Keeping the results of {} …", self.name));
        self.stage = Stage::Keeping(receiver);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Polls the solve until it is done, as the project window does in every frame.
    fn finish(solve: &mut BackgroundSolve, ctx: &egui::Context) {
        let started = Instant::now();
        while solve.active() {
            assert!(started.elapsed() < Duration::from_secs(120), "solve hangs");
            solve.poll(ctx);
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    #[test]
    fn solves_and_keeps_the_results_without_a_window() {
        use crate::solver_check::{LENGTH, beam_mesh, beam_model, faces_where};
        let mesh = beam_mesh();
        let tip = faces_where(&mesh, |p| p[0] == LENGTH);
        let fe = beam_model(
            &mesh,
            plx_model::LoadKind::force([0.0, 0.0, -1.0]),
            plx_model::Region::Faces(tip),
        );
        let data = ProjectAnalysis {
            id: 7,
            name: "Beam 1".into(),
            mesh,
            model: fe,
            ..Default::default()
        };
        let dir = std::env::temp_dir().join(format!("nodalis-monitor-{}", std::process::id()));
        let solver = Solver {
            work_dir: dir.display().to_string(),
            ..Default::default()
        };
        let ctx = egui::Context::default();
        let project = dir.join("project.ndl");
        let work_dir = dir.join("project").join("Beam_1");
        let mut solve = BackgroundSolve::start(
            &ctx,
            project.clone(),
            data.clone(),
            solver.clone(),
            work_dir.clone(),
            Continuation::default(),
            false,
        );
        assert_eq!(solve.state_text(), "preparing");
        finish(&mut solve, &ctx);
        let output = solve.output().join("\n");
        assert_eq!(solve.outcome(), Some(Outcome::Solved), "{output}");
        assert!(
            output.contains("Job finished") || output.contains("completed"),
            "{output}"
        );
        let solved = solve.take_solved().expect("results");
        assert!(solved.meshed.is_none());
        let results = solved.results.expect("kept results");
        // The solver keeps its results without CalculiX's files, in the project as its own
        // results file.
        assert!(!work_dir.join("Beam_1.frd").is_file());
        assert!(results.solver.is_some());
        let read = results.read(&dir.join("read")).unwrap();
        assert!(read.results.increments[0].field("DISP").is_some());
        assert!(read.input.is_some());
        // With them the analysis is up to date: a second solve has nothing to do.
        let status = crate::status::analysis_status(
            None,
            &data.mesh,
            &data.model,
            &data.origins,
            Some(&results),
            &Default::default(),
            false,
            None,
        );
        assert!(!status.needs_solve(), "{status:?}");
        let mut again = BackgroundSolve::start(
            &ctx,
            project,
            ProjectAnalysis {
                results: Some(results),
                ..data
            },
            solver,
            work_dir,
            Continuation::default(),
            false,
        );
        finish(&mut again, &ctx);
        assert_eq!(again.outcome(), Some(Outcome::Solved));
        assert!(again.output().iter().any(|l| l.contains("up to date")));
        let again = again.take_solved().expect("done");
        assert!(again.meshed.is_none() && again.results.is_none());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_solve_stopped_keeping_results_keeps_the_increments_done() {
        use crate::solver_check::{LENGTH, beam_mesh, beam_model, faces_where};
        let mesh = beam_mesh();
        let tip = faces_where(&mesh, |p| p[0] == LENGTH);
        let mut fe = beam_model(
            &mesh,
            plx_model::LoadKind::force([0.0, 0.0, -1.0]),
            plx_model::Region::Faces(tip),
        );
        // Many small fixed increments with large deformations and one frame at the end:
        // the stop comes halfway, and the last increment done is written.
        let plx_model::StepKind::Static(settings) = &mut fe.steps[0].kind else {
            panic!("a static step");
        };
        settings.nlgeom = true;
        settings.control.size = plx_model::IncrementSize::Fixed(1.0 / 20_000.0);
        let data = ProjectAnalysis {
            id: 8,
            name: "Beam 2".into(),
            mesh,
            model: fe,
            ..Default::default()
        };
        let dir = std::env::temp_dir().join(format!("nodalis-keep-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let solver = Solver {
            work_dir: dir.display().to_string(),
            ..Default::default()
        };
        let ctx = egui::Context::default();
        let mut solve = BackgroundSolve::start(
            &ctx,
            dir.join("project.ndl"),
            data,
            solver,
            dir.join("project").join("Beam_2"),
            Continuation::default(),
            false,
        );
        let started = Instant::now();
        while solve.increments_done() < 3 {
            assert!(solve.active(), "{}", solve.output().join("\n"));
            assert!(
                started.elapsed() < Duration::from_secs(120),
                "no increment done"
            );
            solve.poll(&ctx);
            std::thread::sleep(Duration::from_millis(5));
        }
        solve.stop_keeping_results();
        assert!(solve.active() && solve.is_stopping());
        assert_eq!(solve.state_text(), "stopping, writing the results so far");
        finish(&mut solve, &ctx);
        let output = solve.output().join("\n");
        assert_eq!(
            solve.outcome(),
            Some(Outcome::StoppedWithResults),
            "{output}"
        );
        let solved = solve.take_solved().expect("done");
        let results = solved.results.expect("kept results");
        assert!(results.with_errors);
        // The step had one frame, at its end: the state where it stopped is written instead.
        let file = std::fs::read_dir(dir.join("project").join("Beam_2"))
            .unwrap()
            .filter_map(Result::ok)
            .map(|e| e.path())
            .find(|p| {
                p.extension()
                    .is_some_and(|e| e == plx_results::solver::EXTENSION)
            })
            .expect("results file");
        let written = plx_results::SolverResults::read(&file).unwrap();
        assert_eq!(written.results.increments.len(), 1, "{output}");
        assert_eq!(written.unconverged_step, Some(1));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_stopped_solve_keeps_nothing() {
        let ctx = egui::Context::default();
        let dir = std::env::temp_dir().join(format!("nodalis-stop-{}", std::process::id()));
        let mut solve = BackgroundSolve::start(
            &ctx,
            dir.join("p.ndl"),
            ProjectAnalysis::default(),
            Solver::default(),
            dir.clone(),
            Continuation::default(),
            false,
        );
        solve.stop();
        assert!(!solve.active());
        assert_eq!(solve.outcome(), Some(Outcome::Stopped));
        let kept = solve.take_solved().expect("done");
        assert!(kept.meshed.is_none() && kept.results.is_none());
        // What the worker still sends is not taken any more.
        solve.poll(&ctx);
        assert_eq!(solve.outcome(), Some(Outcome::Stopped));
    }

    /// Polls the solve until it is done and returns the longest poll, the time the
    /// project window's frame waits for it.
    fn longest_poll(solve: &mut BackgroundSolve, ctx: &egui::Context) -> Duration {
        let started = Instant::now();
        let mut longest = Duration::ZERO;
        while solve.active() {
            assert!(started.elapsed() < Duration::from_secs(1800), "solve hangs");
            let poll = Instant::now();
            let stage = solve.state_text();
            solve.poll(ctx);
            let took = poll.elapsed();
            if took > Duration::from_millis(20) {
                eprintln!("poll while {stage}: {took:?}");
            }
            longest = longest.max(took);
            std::thread::sleep(Duration::from_millis(5));
        }
        longest
    }

    /// How long submitting a large dynamic analysis blocks the frame, and what saving its
    /// results costs: `NODALIS_SUBMIT_BENCH=60 cargo test --release -p plx-app
    /// submit_bench -- --ignored --nocapture` (elements along the beam).
    #[test]
    #[ignore]
    fn submit_bench() {
        use crate::solver_check::{LENGTH, beam_mesh_divided, beam_model, faces_where};
        let along: u32 = (std::env::var("NODALIS_SUBMIT_BENCH").ok())
            .and_then(|n| n.parse().ok())
            .unwrap_or(40);
        let mesh = beam_mesh_divided([along, along / 5, along / 5]);
        let tip = faces_where(&mesh, |p| p[0] == LENGTH);
        let mut fe = beam_model(
            &mesh,
            plx_model::LoadKind::force([0.0, 0.0, -1.0]),
            plx_model::Region::Faces(tip),
        );
        for material in &mut fe.materials {
            material.density = Some(7.85e-9);
        }
        let mut dynamic = plx_model::Step::new_dynamic("Step-1");
        dynamic.boundary_conditions = std::mem::take(&mut fe.steps[0].boundary_conditions);
        dynamic.loads = std::mem::take(&mut fe.steps[0].loads);
        if let plx_model::StepKind::Dynamic(d) = &mut dynamic.kind {
            d.duration = 1e-3;
            d.frames = 40;
        }
        fe.steps = vec![dynamic];
        eprintln!("{} nodes", mesh.node_count());
        let data = ProjectAnalysis {
            id: 7,
            name: "Beam 1".into(),
            mesh,
            model: fe,
            ..Default::default()
        };
        let dir = std::env::temp_dir().join(format!("nodalis-submit-{}", std::process::id()));
        let solver = Solver {
            work_dir: dir.display().to_string(),
            ..Default::default()
        };
        let ctx = egui::Context::default();
        let project = dir.join("project.ndl");
        let work_dir = dir.join("project").join("Beam_1");
        let start = |data: ProjectAnalysis| {
            let started = Instant::now();
            let solve = BackgroundSolve::start(
                &ctx,
                project.clone(),
                data,
                solver.clone(),
                work_dir.clone(),
                Continuation::default(),
                true,
            );
            eprintln!("start: {:?}", started.elapsed());
            solve
        };
        let mut solve = start(data.clone());
        let longest = longest_poll(&mut solve, &ctx);
        let output = solve.output().join("\n");
        assert_eq!(solve.outcome(), Some(Outcome::Solved), "{output}");
        eprintln!("first solve: longest poll {longest:?}");
        let results = solve
            .take_solved()
            .and_then(|s| s.results)
            .expect("results");
        eprintln!("results {} MB", results.stored_size() / 1_000_000);
        let data = ProjectAnalysis {
            results: Some(results),
            ..data
        };
        let cloned = Instant::now();
        let copy = data.clone();
        eprintln!(
            "copy of the analysis with its results: {:?}",
            cloned.elapsed()
        );
        if let Ok(path) = std::env::var("NODALIS_SUBMIT_BENCH_SAVE") {
            let project = plx_io::project::Project {
                nodes: vec![plx_io::project::ProjectNode::Analysis(Box::new(
                    data.clone(),
                ))],
                ..Default::default()
            };
            plx_io::project::save_project(std::path::Path::new(&path), &project).unwrap();
        }
        let mut again = start(copy);
        let longest = longest_poll(&mut again, &ctx);
        eprintln!("solve again: longest poll {longest:?}");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
