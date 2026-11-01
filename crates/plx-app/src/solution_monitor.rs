//! Solving from the project, as Ansys Workbench's Update Project does: every solve, from
//! Update Project or from Solve in an analysis window, meshes and solves a copy of the
//! analysis' data in the background, and the project window's solution monitor shows the
//! runs with their CalculiX output. An open window shows the run in its Analysis tab and
//! takes the new mesh and results once it is done.

use std::path::PathBuf;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::time::{Duration, Instant};

use plx_io::project::{BoundResults, ProjectAnalysis};
use plx_job::JobStatus;
use plx_model::{Finding, Severity};

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
    Meshing(MeshingJob),
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
    findings: Vec<Finding>,
}

/// How a background solve ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Solved and kept, or up to date already.
    Solved,
    /// CalculiX reported errors, but its results are kept.
    SolvedWithErrors,
    Failed,
    Stopped,
}

impl Outcome {
    pub fn text(self) -> &'static str {
        match self {
            Self::Solved => "solved",
            Self::SolvedWithErrors => "finished with errors, results kept",
            Self::Failed => "failed",
            Self::Stopped => "stopped",
        }
    }
}

/// What a finished background solve changed in the analysis.
pub struct Solved {
    /// The new mesh with the fingerprints of what its parts were made from, when the solve
    /// meshed the geometry first.
    pub meshed: Option<(plx_mesh::FeMesh, std::collections::BTreeMap<String, u64>)>,
    pub results: Option<BoundResults>,
    /// Problems CalculiX reported when the run failed.
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
    /// Lines before CalculiX runs (loading, meshing, model checks) and after (keeping the
    /// results), around the solver output.
    before: Vec<String>,
    after: Vec<String>,
    started: Instant,
    finished: Option<f32>,
    meshed: bool,
    results: Option<BoundResults>,
    findings: Vec<Finding>,
    mesh_print: Option<u64>,
    /// Runs CalculiX even if the results are up to date: Solve Again.
    force: bool,
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
        std::thread::spawn(move || {
            let ProjectAnalysis {
                geometry,
                mesh,
                model: fe,
                mesh_inputs,
                results,
                ..
            } = data;
            let mut model = Model::new(&path, mesh);
            model.fe = fe;
            model.geometry = geometry;
            model.mesh_inputs = mesh_inputs;
            model.bound_results = results;
            let findings = if model.mesh.element_count() > 0 {
                model.findings()
            } else {
                Vec::new()
            };
            let mesh_print = model.mesh.fingerprint();
            let _ = sender.send(Loaded {
                model,
                mesh_print,
                solver: probed,
                findings,
            });
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

    /// The solver settings with what was found out about the executable on the way.
    pub fn solver(&self) -> &Solver {
        &self.solver
    }

    /// What it does now, for the monitor.
    pub fn state_text(&self) -> &'static str {
        match &self.stage {
            Stage::Loading(_) => "preparing",
            Stage::Meshing(_) => "meshing",
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

    /// Where CalculiX is, while it runs.
    pub fn progress(&self) -> Option<String> {
        let analysis = self.analysis.as_ref()?;
        analysis.is_running().then(|| analysis.progress()).flatten()
    }

    /// The output so far: nodalis's lines around CalculiX's.
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

    /// The lines dropped from the front of CalculiX's output in a long run.
    pub fn dropped(&self) -> usize {
        self.analysis
            .as_ref()
            .map_or(0, crate::analysis::Analysis::dropped)
    }

    /// Stops it: CalculiX is killed, a model being built or meshed is dropped.
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

    /// What it changed in the analysis, once done; taken once.
    pub fn take_solved(&mut self) -> Option<Solved> {
        if self.active() {
            return None;
        }
        let meshed = (self.meshed.then(|| self.model.take()).flatten())
            .map(|model| (model.mesh.clone(), model.mesh_inputs.clone()));
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

    /// Works on: takes the built model or the new mesh, polls CalculiX and keeps its
    /// results once it ends.
    pub fn poll(&mut self, ctx: &egui::Context) {
        match &mut self.stage {
            Stage::Loading(receiver) => match receiver.try_recv() {
                Ok(loaded) => {
                    let Loaded {
                        model,
                        mesh_print,
                        solver,
                        findings,
                    } = loaded;
                    self.model = Some(model);
                    self.mesh_print = Some(mesh_print);
                    self.solver = solver;
                    self.mesh_or_run(ctx, findings);
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => self.fail("The model was not built.".into()),
            },
            Stage::Meshing(job) => {
                let Some(result) = job.poll() else {
                    return;
                };
                let (geometry, started) = (job.geometry.clone(), job.started);
                let Some(model) = self.model.as_mut() else {
                    return;
                };
                let merged = result.and_then(|generated| {
                    crate::app::merge_part_meshes(model, &geometry, generated, Some(started))
                });
                match merged {
                    Ok(lines) => {
                        self.before.extend(lines);
                        self.meshed = true;
                        let findings = model.findings();
                        self.run(findings);
                    }
                    Err(error) => {
                        let error = if error.starts_with("Meshing") {
                            error
                        } else {
                            format!("Meshing failed: {error}")
                        };
                        self.fail(error);
                    }
                }
            }
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
                    self.finish(if with_errors {
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

    /// Meshes the geometry first if the mesh is missing or out of date; else starts
    /// CalculiX.
    fn mesh_or_run(&mut self, ctx: &egui::Context, findings: Vec<Finding>) {
        let Some(model) = &self.model else {
            return;
        };
        let status = crate::status::analysis_status(
            model.geometry.as_ref(),
            &model.mesh,
            &model.fe,
            &model.mesh_inputs,
            model.bound_results.as_ref(),
            // The project makes the mesh blocks it takes its mesh from before it starts.
            &Default::default(),
            false,
        );
        let parts = match status.mesh {
            CellState::Missing => Some(None),
            CellState::OutOfDate => Some(Some(status.stale_parts.clone())),
            _ => None,
        };
        if let Some(parts) = parts {
            let Some(geometry) = model.geometry.clone() else {
                return self.fail("The model has no mesh and no geometry to mesh.".into());
            };
            let what = match &parts {
                Some(parts) => parts.join(", "),
                None => geometry.source.clone(),
            };
            self.before.push(format!("Meshing {what} …"));
            self.stage = Stage::Meshing(MeshingJob::start(geometry, parts, ctx));
            return;
        }
        if !status.needs_solve() && !self.force {
            self.before
                .push(format!("{} is up to date; nothing to solve.", self.name));
            return self.finish(Outcome::Solved);
        }
        self.run(findings);
    }

    /// Writes the input file and starts CalculiX.
    fn run(&mut self, findings: Vec<Finding>) {
        let Some(model) = &self.model else {
            return;
        };
        let started = Analysis::start(
            &self.solver,
            model,
            false,
            &self.name,
            &self.work_dir,
            &self.continuation,
        );
        let mut analysis = match started {
            Ok(analysis) => analysis,
            Err(error) => return self.fail(error),
        };
        // Problems the checks found go first, so that an abort is explained even before
        // CalculiX says anything.
        for finding in findings.iter().filter(|f| f.severity() == Severity::Error) {
            analysis.note(format!(
                "Model check: {}: {}",
                finding.problem.title(),
                finding.detail
            ));
        }
        // CalculiX skips keywords it does not know with only a warning.
        let unknown = self.solver.unknown_keywords(analysis.keywords());
        if !unknown.is_empty() {
            analysis.note(format!(
                "Model check: CalculiX does not know {}",
                crate::solver_check::by_feature(&unknown).join("; ")
            ));
        }
        self.analysis = Some(analysis);
        self.stage = Stage::Running;
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
            let findings = plx_model::diagnose_solver_check(&diagnostics, &model.mesh, &model.fe);
            let mut shown = std::collections::HashSet::new();
            for finding in &findings {
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
            let findings = plx_model::diagnose_solver_output(analysis.output(), &model.mesh);
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
        let results = analysis
            .results()
            .filter(|_| matches!(status, JobStatus::Completed | JobStatus::FailedWithResults));
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
            &["inp", "eig"]
        } else {
            &["inp", "dat", "eig"]
        };
        let mut files = vec![results_file.clone()];
        files.extend(extensions.iter().map(|e| results_file.with_extension(e)));
        let (name, input) = (self.name.clone(), analysis.input);
        let steps = analysis.steps.clone();
        let with_errors = status == JobStatus::FailedWithResults;
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

    fn ccx_installed() -> bool {
        let found = std::process::Command::new("ccx").arg("-v").output().is_ok();
        if !found {
            eprintln!("ccx not found, test skipped");
        }
        found
    }

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
        if !ccx_installed() {
            return;
        }
        solves_and_keeps_the_results(plx_solve::Engine::CalculiX);
    }

    /// The same with the built-in solver, which needs no program.
    #[test]
    fn solves_with_the_built_in_solver() {
        solves_and_keeps_the_results(plx_solve::Engine::Nodalis);
    }

    fn solves_and_keeps_the_results(engine: plx_solve::Engine) {
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
        let dir =
            std::env::temp_dir().join(format!("nodalis-monitor-{engine:?}-{}", std::process::id()));
        let solver = Solver {
            engine,
            executable: "ccx".into(),
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
        // The built-in solver keeps its results without CalculiX's files, in the project
        // as its own results file.
        let calculix = engine == plx_solve::Engine::CalculiX;
        assert_eq!(work_dir.join("Beam_1.frd").is_file(), calculix);
        assert_eq!(results.solver.is_some(), !calculix);
        let read = results.read(&dir.join("read")).unwrap();
        assert!(read.results.increments[0].field("DISP").is_some());
        assert!(read.input.is_some());
        // With them the analysis is up to date: a second solve has nothing to do.
        let status = crate::status::analysis_status(
            None,
            &data.mesh,
            &data.model,
            &data.mesh_inputs,
            Some(&results),
            &Default::default(),
            false,
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
}
