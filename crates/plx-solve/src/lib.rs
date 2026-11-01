//! Running an analysis with either solver engine behind one interface: the PrePoLix solver
//! `plxs` as a library in this process ([`BuiltinJob`]), or CalculiX as a child process
//! ([`plx_job::Job`]). [`Run`] hides which one it is from the monitor and the project.
//!
//! The built-in run takes the same input text as CalculiX would. Its results stay in memory
//! ([`Run::solver_results`]) and are written as one binary file `<name>.plxr`
//! ([`plx_results::SolverResults`]) instead of CalculiX's `.frd` and `.dat`, which PrePoLix
//! reads without parsing text; the `.sta` file is written as by CalculiX. Its progress comes
//! as typed events, which are rendered into CalculiX's lines for the monitor.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use plx_job::{Job, JobStatus, live::LiveProgress};
use plx_results::SolverResults;
use plxs_core::{CancelToken, Error, ProgressEvent};
use serde::{Deserialize, Serialize};

/// The version of the built-in solver.
pub const VERSION: &str = plxs_session::job::VERSION;

/// Which solver computes the analyses.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Engine {
    /// CalculiX `ccx` as a child process, the shipped one or the user's own.
    #[default]
    CalculiX,
    /// The PrePoLix solver `plxs`, built in.
    PrePoLix,
}

impl Engine {
    pub const ALL: [Engine; 2] = [Engine::PrePoLix, Engine::CalculiX];

    /// The name shown in the interface.
    pub fn name(self) -> &'static str {
        match self {
            Engine::CalculiX => "CalculiX",
            Engine::PrePoLix => "PrePoLix solver",
        }
    }
}

/// Smallest `.frd` that holds results, as in `plx_job`.
const MIN_RESULT_SIZE: u64 = 300;

/// Files of a previous run that would be mistaken for results of this one.
const OLD_FILES: [&str; 8] = ["inp", "dat", "sta", "cvg", "12d", "cel", "frd", "plxr"];

/// An analysis running or finished on either engine.
pub enum Run {
    Process(Job),
    Builtin(BuiltinJob),
}

impl Run {
    /// Writes `<work_dir>/<name>.inp` and starts `input` on `engine`; `solver` tells the
    /// CalculiX executable and the threads of either engine.
    pub fn start(
        engine: Engine,
        solver: &plx_job::Solver,
        work_dir: &Path,
        name: &str,
        input: &str,
    ) -> std::io::Result<Run> {
        match engine {
            Engine::CalculiX => Job::start(solver, work_dir, name, input).map(Run::Process),
            Engine::PrePoLix => {
                BuiltinJob::start(solver.threads, work_dir, name, input).map(Run::Builtin)
            }
        }
    }

    pub fn engine(&self) -> Engine {
        match self {
            Run::Process(_) => Engine::CalculiX,
            Run::Builtin(_) => Engine::PrePoLix,
        }
    }

    pub fn name(&self) -> &str {
        match self {
            Run::Process(job) => job.name(),
            Run::Builtin(job) => job.name(),
        }
    }

    /// Checks whether the run has finished, without waiting.
    pub fn poll(&mut self) -> JobStatus {
        match self {
            Run::Process(job) => job.poll(),
            Run::Builtin(job) => job.poll(),
        }
    }

    /// Waits until the run has finished.
    pub fn wait(&mut self) -> JobStatus {
        match self {
            Run::Process(job) => job.wait(),
            Run::Builtin(job) => job.wait(),
        }
    }

    /// Stops the run.
    pub fn kill(&mut self) {
        match self {
            Run::Process(job) => job.kill(),
            Run::Builtin(job) => job.kill(),
        }
    }

    /// Output lines written since the last call.
    pub fn new_output(&mut self) -> Vec<String> {
        match self {
            Run::Process(job) => job.new_output(),
            Run::Builtin(job) => job.new_output(),
        }
    }

    /// Where the run is by its output so far.
    pub fn live_progress(&self) -> &LiveProgress {
        match self {
            Run::Process(job) => job.live_progress(),
            Run::Builtin(job) => job.live_progress(),
        }
    }

    /// The last increment reported in the `.sta` file.
    pub fn progress(&self) -> Option<plx_job::Progress> {
        let text = std::fs::read_to_string(self.file("sta")).ok()?;
        plx_job::parse_sta(&text)
    }

    /// The results file, once results have been written to it: `.frd` of CalculiX, `.plxr`
    /// of the built-in solver.
    pub fn results(&self) -> Option<PathBuf> {
        match self {
            Run::Process(_) => {
                let path = self.file("frd");
                let size = std::fs::metadata(&path).ok()?.len();
                (size > MIN_RESULT_SIZE).then_some(path)
            }
            Run::Builtin(job) => job.results_written().then(|| job.results_file()),
        }
    }

    /// The results of the built-in solver in memory, once it has finished with results.
    pub fn solver_results(&self) -> Option<Arc<SolverResults>> {
        match self {
            Run::Process(_) => None,
            Run::Builtin(job) => job.solver_results(),
        }
    }

    pub fn file(&self, extension: &str) -> PathBuf {
        match self {
            Run::Process(job) => job.file(extension),
            Run::Builtin(job) => job.file(extension),
        }
    }
}

/// The built-in solver computing one job on a thread of its own.
pub struct BuiltinJob {
    name: String,
    work_dir: PathBuf,
    output: Arc<Mutex<Vec<String>>>,
    /// The results, once the solver has finished with some.
    solved: Arc<Mutex<Option<Arc<SolverResults>>>>,
    live: LiveProgress,
    cancel: CancelToken,
    handle: Option<JoinHandle<Finished>>,
    status: JobStatus,
}

/// How the solver thread ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Finished {
    Done,
    Cancelled,
    Error,
}

impl BuiltinJob {
    /// Writes `<work_dir>/<name>.inp` and starts the solver on it with `threads` threads.
    pub fn start(threads: u32, work_dir: &Path, name: &str, input: &str) -> std::io::Result<Self> {
        std::fs::create_dir_all(work_dir)?;
        for extension in OLD_FILES {
            let path = work_dir.join(format!("{name}.{extension}"));
            if path.exists() {
                std::fs::remove_file(path)?;
            }
        }
        let input_path = work_dir.join(format!("{name}.inp"));
        std::fs::write(&input_path, input)?;
        let output = Arc::new(Mutex::new(vec![
            "************************************************************".to_owned(),
            format!("PrePoLix solver plxs {VERSION}"),
            "************************************************************".to_owned(),
        ]));
        let cancel = CancelToken::new();
        let options = plxs_session::JobOptions {
            threads: threads as usize,
            write_files: false,
            cancel: cancel.clone(),
            ..plxs_session::JobOptions::new(work_dir, name)
        };
        let input = input.to_owned();
        let lines = Arc::clone(&output);
        let solved = Arc::new(Mutex::new(None));
        let keep = Arc::clone(&solved);
        let results_file = work_dir.join(format!("{name}.{}", plx_results::solver::EXTENSION));
        let handle = std::thread::Builder::new()
            .name(format!("plxs {name}"))
            .spawn(move || {
                let say = |line: String| lines.lock().unwrap_or_else(|e| e.into_inner()).push(line);
                let mut progress = |event: ProgressEvent| {
                    for line in plxs_session::format_event(&event) {
                        say(line);
                    }
                };
                let mut kept = None;
                let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    plxs_session::run_job_keeping(
                        &input,
                        Some(&input_path),
                        &options,
                        &mut progress,
                        &mut kept,
                    )
                }));
                // The results so far, also of a step that did not converge.
                if let Some((model, results)) = kept {
                    let results = SolverResults::new(&model, results, VERSION);
                    match results.write(&results_file) {
                        Ok(()) => {
                            *keep.lock().unwrap_or_else(|e| e.into_inner()) =
                                Some(Arc::new(results));
                        }
                        Err(error) => say(format!(
                            "*ERROR: results not written to {}: {error}",
                            results_file.display()
                        )),
                    }
                }
                match run {
                    Ok(Ok(plxs_session::JobOutcome::Finished)) => Finished::Done,
                    Ok(Ok(plxs_session::JobOutcome::Cancelled)) => {
                        say(" *INFO: the run was stopped".into());
                        Finished::Cancelled
                    }
                    Ok(Ok(plxs_session::JobOutcome::Diverged)) => Finished::Error,
                    Ok(Err(error)) => {
                        say(String::new());
                        say(error.to_string());
                        if let Some(location) = error.location() {
                            say(format!(" in {}, line {}", location.file, location.line));
                        }
                        if let Error::Unsupported(_) = error {
                            say(
                                " The PrePoLix solver does not compute this yet; CalculiX does: \
                                 Tools > Settings > Solver."
                                    .into(),
                            );
                        }
                        Finished::Error
                    }
                    Err(panic) => {
                        let message = panic
                            .downcast_ref::<String>()
                            .cloned()
                            .or_else(|| panic.downcast_ref::<&str>().map(|s| (*s).to_owned()))
                            .unwrap_or_else(|| "unknown cause".into());
                        say(format!("*ERROR: the solver failed: {message}"));
                        Finished::Error
                    }
                }
            })?;
        Ok(Self {
            name: name.to_owned(),
            work_dir: work_dir.to_owned(),
            output,
            solved,
            live: LiveProgress::default(),
            cancel,
            handle: Some(handle),
            status: JobStatus::Running,
        })
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn file(&self, extension: &str) -> PathBuf {
        self.work_dir.join(format!("{}.{extension}", self.name))
    }

    /// Checks whether the solver has finished, without waiting.
    pub fn poll(&mut self) -> JobStatus {
        if self.status == JobStatus::Running
            && self.handle.as_ref().is_some_and(JoinHandle::is_finished)
        {
            self.join();
        }
        self.status
    }

    /// Waits until the solver has finished.
    pub fn wait(&mut self) -> JobStatus {
        if self.status == JobStatus::Running {
            self.join();
        }
        self.status
    }

    /// Stops the solver at the next increment and waits for it.
    pub fn kill(&mut self) {
        if self.status == JobStatus::Running {
            self.cancel.cancel();
            self.join();
            self.status = JobStatus::Killed;
        }
    }

    /// Output lines written since the last call.
    pub fn new_output(&mut self) -> Vec<String> {
        let mut output = self.output.lock().unwrap_or_else(|e| e.into_inner());
        let lines = std::mem::take(&mut *output);
        lines.iter().for_each(|line| self.live.read(line));
        lines
    }

    pub fn live_progress(&self) -> &LiveProgress {
        &self.live
    }

    /// The results file `<name>.plxr`.
    pub fn results_file(&self) -> PathBuf {
        self.file(plx_results::solver::EXTENSION)
    }

    /// The results in memory, once the solver has finished with some.
    pub fn solver_results(&self) -> Option<Arc<SolverResults>> {
        if self.status == JobStatus::Running {
            return None;
        }
        self.solved
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }

    fn results_written(&self) -> bool {
        self.status != JobStatus::Running
            && self
                .solved
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .is_some()
            && self.results_file().is_file()
    }

    fn join(&mut self) {
        let Some(handle) = self.handle.take() else {
            return;
        };
        let finished = handle.join().unwrap_or(Finished::Error);
        let results = self
            .solved
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .is_some();
        self.status = match (finished, results) {
            (Finished::Cancelled, _) => JobStatus::Killed,
            (Finished::Done, true) => JobStatus::Completed,
            (Finished::Done, false) => JobStatus::Failed,
            (Finished::Error, true) => JobStatus::FailedWithResults,
            (Finished::Error, false) => JobStatus::Failed,
        };
    }
}

impl Drop for BuiltinJob {
    fn drop(&mut self) {
        self.kill();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One hexahedron fixed at x=0 and pulled at x=1.
    const CUBE: &str = "*NODE, NSET=NALL
1, 0, 0, 0
2, 1, 0, 0
3, 1, 1, 0
4, 0, 1, 0
5, 0, 0, 1
6, 1, 0, 1
7, 1, 1, 1
8, 0, 1, 1
*ELEMENT, TYPE=C3D8, ELSET=EALL
1, 1, 2, 3, 4, 5, 6, 7, 8
*NSET, NSET=FIX
1, 4, 5, 8
*NSET, NSET=PULL
2, 3, 6, 7
*MATERIAL, NAME=STEEL
*ELASTIC
210000, 0.3
*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL
*BOUNDARY
FIX, 1, 3
*STEP
*STATIC
*CLOAD
PULL, 1, 25
*NODE FILE
U, RF
*EL FILE
S
*END STEP
";

    #[test]
    fn built_in_run_keeps_the_results_in_memory() {
        let dir = std::env::temp_dir().join(format!("plx-solve-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let solver = plx_job::Solver {
            executable: PathBuf::from("unused"),
            threads: 1,
        };
        let mut run = Run::start(Engine::PrePoLix, &solver, &dir, "Cube", CUBE).unwrap();
        assert_eq!(run.engine(), Engine::PrePoLix);
        let status = run.wait();
        let output = run.new_output();
        assert_eq!(status, JobStatus::Completed, "{output:#?}");
        assert!(
            output.iter().any(|l| l.contains("Job finished")),
            "{output:#?}"
        );
        assert!(output.iter().any(|l| l.trim() == "STEP 1"), "{output:#?}");
        // The results stay in memory and in the `.plxr` file, without a `.frd` file.
        assert_eq!(run.results(), Some(run.file("plxr")));
        assert!(!run.file("frd").exists());
        let solved = run.solver_results().unwrap();
        let import = plx_results::from_solver(&solved);
        assert_eq!(import.mesh.node_count(), 8);
        let disp = import.increments[0].field("DISP").unwrap();
        let u1 = disp.component("U1").unwrap();
        assert!(u1.values.iter().all(|v| v.is_finite()));
        assert!(u1.values.iter().any(|&v| v > 0.0));
        let read = SolverResults::read(&run.file("plxr")).unwrap();
        assert_eq!(read, *solved);
        assert!(run.file("inp").is_file());
        assert!(run.file("sta").is_file());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn unsupported_procedures_fail_with_a_hint() {
        let dir = std::env::temp_dir().join(format!("plx-solve-dyn-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let input = CUBE
            .replace("*STEP", "*STEP, NLGEOM")
            .replace("*STATIC", "*COUPLED TEMPERATURE-DISPLACEMENT\n0.1, 1.0");
        let mut job = BuiltinJob::start(1, &dir, "Dyn", &input).unwrap();
        assert_eq!(job.wait(), JobStatus::Failed);
        let output = job.new_output();
        assert!(output.iter().any(|l| l.contains("*ERROR")), "{output:#?}");
        assert!(
            output
                .iter()
                .any(|l| l.contains("Tools > Settings > Solver")),
            "{output:#?}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn input_errors_are_reported_in_calculix_wording() {
        let dir = std::env::temp_dir().join(format!("plx-solve-err-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let input = CUBE.replace("*CLOAD", "*CLOUD");
        let mut job = BuiltinJob::start(1, &dir, "Err", &input).unwrap();
        assert_eq!(job.wait(), JobStatus::Failed);
        let output = job.new_output();
        assert!(
            output.iter().any(|l| l.contains("cannot be interpreted")),
            "{output:#?}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }
}
