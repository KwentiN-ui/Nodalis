//! A whole job as the command line and Nodalis run it: read the input, expand shells and
//! beams, compute the steps on a session, write the result files. The command line prints
//! the events in CalculiX's wording ([`format_event`]); Nodalis consumes them directly.

use std::path::{Path, PathBuf};

use plxs_core::step::Procedure;
use plxs_core::{Analysis, CancelToken, Error, Model, Progress, ProgressEvent, Result, Results};

use plxs_core::state::Snapshot;

use std::sync::{Arc, Mutex};

use plxs_output::spool::Spool;

use crate::cache::StepCache;
use crate::restart::{self, RestartFile};
use crate::{Session, SessionOptions, StepOutcome};

/// The version of the solver, written into the result files.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[derive(Clone, Debug)]
pub struct JobOptions {
    /// Threads of the run (0: all cores), when no `pool` is given.
    pub threads: usize,
    /// A pool shared with other runs, see [`SessionOptions::pool`].
    pub pool: Option<std::sync::Arc<rayon::ThreadPool>>,
    /// The job's directory: result files go here, files the input refers to are read here.
    pub work_dir: PathBuf,
    /// Name of the job: the result files are `<name>.frd`, `.dat`, `.sta`.
    pub name: String,
    /// Writes the result files `.frd` and `.dat`; off when the caller takes the results from
    /// memory (see [`run_job_keeping`]). The `.sta` file is written either way.
    pub write_files: bool,
    /// How the numbers of the `.frd` are written.
    pub frd_format: plxs_output::FrdFormat,
    /// Reports the time per phase at the end of the job, as `Info` events.
    pub profile: bool,
    /// Receives the time per phase whenever the results are written: at the end, and after
    /// a step that did not converge or was cancelled. Nodalis shows it in its monitor.
    pub profile_sink: Option<ProfileSink>,
    pub cancel: CancelToken,
    /// Receives the iterations of the iterative linear solver as they happen, see
    /// [`plxs_core::LinearWatch`].
    pub linear_watch: Option<plxs_core::LinearWatch>,
    /// Keeps the state and results after each step, and takes over the steps a run before
    /// computed already ([`crate::cache`]); `None`: every step is computed.
    pub step_cache: Option<StepCache>,
    /// Keeps the increments that are done in this file instead of memory while the run goes
    /// on ([`KeptResults::spool`]), so that the results of a long run need not fit into
    /// memory; only without [`JobOptions::write_files`]. The file is deleted with the
    /// results.
    pub spool: Option<PathBuf>,
}

impl JobOptions {
    /// A job `name` in `work_dir` with the defaults: all cores, result files written, text
    /// `.frd`, no profile.
    pub fn new(work_dir: impl Into<PathBuf>, name: impl Into<String>) -> Self {
        Self {
            threads: 0,
            pool: None,
            work_dir: work_dir.into(),
            name: name.into(),
            write_files: true,
            frd_format: plxs_output::FrdFormat::Ascii,
            profile: false,
            profile_sink: None,
            cancel: CancelToken::new(),
            linear_watch: None,
            step_cache: None,
            spool: None,
        }
    }

    fn output(&self, ext: &str) -> PathBuf {
        self.work_dir.join(format!("{}.{ext}", self.name))
    }
}

/// Where the time of a job went, with the size of the model it solved.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct JobProfile {
    /// Time per phase, summed over the steps.
    pub phases: plxs_procedures::profile::Profile,
    /// Wall-clock time of the job from the expansion of the model on, writing included.
    pub total: std::time::Duration,
    /// Of which writing the results.
    pub writing: std::time::Duration,
    /// Nodes and elements of the model as solved (shells and beams expanded).
    pub nodes: usize,
    pub elements: usize,
    /// Degrees of freedom before constraints are eliminated.
    pub degrees_of_freedom: usize,
    /// Threads the solver computed on.
    pub threads: usize,
}

/// Where a job leaves its [`JobProfile`], see [`JobOptions::profile_sink`].
pub type ProfileSink = std::sync::Arc<std::sync::Mutex<Option<JobProfile>>>;

/// How a job ended.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum JobOutcome {
    /// Every step converged; the result files are complete.
    Finished,
    /// A step did not converge; the results so far are written.
    Diverged,
    /// Cancelled by the caller; the results so far are written.
    Cancelled,
}

/// Reads `input` (the text of an `.inp` file, `*INCLUDE`s relative to `base`) and runs it.
/// Errors are returned after their events; what the model is and does goes to `progress`
/// as `Info` events, like the command line prints it.
pub fn run_job(
    input: &str,
    base: Option<&Path>,
    options: &JobOptions,
    progress: &mut dyn Progress,
) -> Result<JobOutcome> {
    run_job_keeping(input, base, options, progress, &mut None)
}

/// Runs `<dir>/<name>.inp` as the command line `plxs -i <name>` does, on one thread: the
/// result files `.frd`, `.dat` and `.sta` go to `dir`. Returns the output lines, as the error
/// when the job failed or did not finish.
pub fn run_input_file(dir: &Path, name: &str) -> std::result::Result<Vec<String>, String> {
    let input_path = dir.join(format!("{name}.inp"));
    let input = std::fs::read_to_string(&input_path)
        .map_err(|e| format!("{}: {e}", input_path.display()))?;
    let options = JobOptions {
        threads: 1,
        ..JobOptions::new(dir, name)
    };
    let mut lines = Vec::new();
    let outcome = run_job(&input, Some(&input_path), &options, &mut |event| {
        lines.extend(crate::format_event(&event));
    });
    match outcome {
        Ok(JobOutcome::Finished) => Ok(lines),
        Ok(other) => Err(format!("{other:?}\n{}", lines.join("\n"))),
        Err(e) => Err(format!("{e}\n{}", lines.join("\n"))),
    }
}

/// The output model and results of a job: the model with shells and beams expanded as the
/// `.frd` file shows them, and the results on it.
#[derive(Debug)]
pub struct KeptResults {
    pub model: Model,
    /// The results; without their increments where these are in [`KeptResults::spool`].
    pub results: Results,
    /// The increments as written, kept on disk ([`JobOptions::spool`]); they come before
    /// those in [`KeptResults::results`], which has none then.
    pub spool: Option<Spool>,
}

/// A spool shared by the job and its session, which hands increments over to it.
#[derive(Clone)]
struct SharedSpool(Arc<Mutex<Option<Spool>>>);

impl SharedSpool {
    fn lock(&self) -> std::sync::MutexGuard<'_, Option<Spool>> {
        self.0.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl plxs_core::Spill for SharedSpool {
    fn spill(&mut self, increment: plxs_core::ResultsIncrement) {
        if let Some(spool) = self.lock().as_mut() {
            plxs_core::Spill::spill(spool, increment);
        }
    }
}

fn spool_error(path: &Path, message: impl std::fmt::Display) -> Error {
    Error::Io {
        path: path.display().to_string(),
        message: message.to_string(),
    }
}

/// [`run_job`], keeping the results in `kept` whenever they would be written: at the end,
/// and after a step that did not converge or was cancelled.
pub fn run_job_keeping(
    input: &str,
    base: Option<&Path>,
    options: &JobOptions,
    progress: &mut dyn Progress,
    kept: &mut Option<KeptResults>,
) -> Result<JobOutcome> {
    // `*RESTART, READ`: the model and the steps before come from `<job>.rin`.
    let mut snapshot = None;
    let reading = match plxs_input::restart_read(input) {
        None => plxs_input::read_inp_str(input, base),
        Some(step) => {
            let rin = options.output("rin");
            let file = RestartFile::read(&rin)?;
            let Some(found) = file.after(step).cloned() else {
                return Err(Error::Input {
                    keyword: "*RESTART".into(),
                    message: format!(
                        "{} holds no state after step {}",
                        rin.display(),
                        step.map_or_else(|| "any".to_string(), |s| s.to_string())
                    ),
                    location: None,
                });
            };
            let base_analysis = file.base(found.state.steps_done);
            snapshot = Some(found);
            plxs_input::read_inp_continuing(input, base, base_analysis)
        }
    };
    let reading = reading.map_err(|mut errors| {
        for error in errors.iter().skip(1) {
            progress.event(ProgressEvent::Warning(error.to_string()));
        }
        errors.swap_remove(0)
    })?;
    for warning in &reading.warnings {
        progress.event(ProgressEvent::Warning(warning.clone()));
    }
    let analysis = reading.analysis;
    let first = snapshot.as_ref().map_or(0, |s| s.state.steps_done as usize);
    progress.event(ProgressEvent::Info(format!(
        "number of nodes: {}, elements: {}, steps: {}",
        analysis.model.nodes.len(),
        analysis.model.elements.len(),
        analysis.steps.len() - first
    )));
    run_continuing(analysis, snapshot, options, progress, kept)
}

/// Runs a parsed analysis, see [`run_job`].
pub fn run_analysis(
    analysis: Analysis,
    options: &JobOptions,
    progress: &mut dyn Progress,
) -> Result<JobOutcome> {
    run_continuing(analysis, None, options, progress, &mut None)
}

/// Runs a parsed analysis, see [`run_job_keeping`].
pub fn run_analysis_keeping(
    analysis: Analysis,
    options: &JobOptions,
    progress: &mut dyn Progress,
    kept: &mut Option<KeptResults>,
) -> Result<JobOutcome> {
    run_continuing(analysis, None, options, progress, kept)
}

/// Runs a parsed analysis; with a `snapshot` (`*RESTART, READ`) from the state it holds,
/// skipping the steps computed before.
fn run_continuing(
    analysis: Analysis,
    snapshot: Option<Snapshot>,
    options: &JobOptions,
    progress: &mut dyn Progress,
    kept: &mut Option<KeptResults>,
) -> Result<JobOutcome> {
    let started = std::time::Instant::now();
    let first = snapshot.as_ref().map_or(0, |s| s.state.steps_done as usize);
    // A model check only reads the input.
    if (analysis.steps[first..].iter()).all(|s| matches!(s.procedure, Procedure::NoAnalysis)) {
        for index in first..analysis.steps.len() {
            progress.event(ProgressEvent::Step {
                step: index as u32 + 1,
            });
            progress.event(ProgressEvent::Info(
                "*NO ANALYSIS: the model was read and checked, nothing is computed.".into(),
            ));
        }
        progress.event(ProgressEvent::JobDone);
        return Ok(JobOutcome::Finished);
    }
    // What `*RESTART, WRITE` stores: the analysis as read, before the expansion.
    let mut restart_file = (analysis.steps[first..].iter())
        .any(|s| s.restart_write.is_some_and(|w| w.frequency > 0))
        .then(|| RestartFile {
            analysis: Analysis {
                model: analysis.model.clone(),
                steps: Vec::new(),
            },
            snapshots: Vec::new(),
        });
    let source_steps = restart_file.as_ref().map(|_| analysis.steps.clone());
    // The step cache: the keys of the steps as given, and the steps of an earlier run with
    // the same keys. A `*RESTART, READ` run continues from its own state.
    let cache = options.step_cache.as_ref().filter(|_| snapshot.is_none());
    let keys = cache.map(|c| c.keys(&analysis, &options.work_dir));
    // Increments that are done go to a file; the command line writes its `.frd` from
    // memory.
    let spool = match (&options.spool, options.write_files) {
        (Some(path), false) => Some(SharedSpool(Arc::new(Mutex::new(Some(
            Spool::create(path).map_err(|e| spool_error(path, e))?,
        ))))),
        _ => None,
    };
    let mut cached = None;
    if let (Some(cache), Some(keys)) = (cache, &keys) {
        cache.keep_only(keys);
        let mut spooled = spool.as_ref().map(SharedSpool::lock);
        cached = cache.take(keys, spooled.as_mut().and_then(|s| s.as_mut()));
    }
    // Shells, membranes and beams are computed as the solids CalculiX expands them to.
    let expansion = plxs_elements::expand::expand(analysis)?;
    let analysis = expansion.analysis;
    let output_map = expansion.output;
    let session_options = SessionOptions {
        threads: options.threads,
        pool: options.pool.clone(),
        work_dir: options.work_dir.clone(),
        cancel: options.cancel.clone(),
        linear_watch: options.linear_watch.clone(),
    };
    let steps = analysis.steps.len();
    let (mut session, first) = match (snapshot, cached) {
        (Some(snapshot), _) => (
            Session::resume(analysis.model, snapshot, session_options)?,
            first,
        ),
        // A state that does not fit the model (it does, the key says so) is not used.
        (None, Some(cached))
            if restart::model_fingerprint(&analysis.model) == cached.snapshot.model_fingerprint =>
        {
            let taken = cached.steps;
            let mut session = Session::resume(analysis.model, cached.snapshot, session_options)?;
            session.results = cached.results;
            progress.event(ProgressEvent::Info(if taken == steps {
                "no step changed since the last run: the results are taken over".to_string()
            } else {
                format!(
                    "steps 1 to {taken} did not change since the last run: their results are \
                     taken over, the run continues with step {}",
                    taken + 1
                )
            }));
            (session, taken)
        }
        (None, _) => (Session::new(analysis.model, session_options)?, 0),
    };
    if let Some(spool) = &spool {
        session.set_spill(Box::new(spool.clone()));
    }
    // Only when the expansion kept the steps as they were (it always does).
    let keys = keys.filter(|k| k.len() == steps);
    let mut keeping = cache.is_some();
    let output = |ext: &str| options.output(ext);
    let mut write = |session: &mut Session| -> Result<()> {
        let writing = std::time::Instant::now();
        session.spill_increments();
        let mut spooled = spool.as_ref().and_then(|s| s.lock().take());
        if let Some(error) = spooled.as_ref().and_then(Spool::error) {
            return Err(Error::Io {
                path: String::new(),
                message: format!("results not kept: {error}"),
            });
        }
        let (model, results) = output_map.apply(session.model(), session.results());
        // The spooled increments as written: mapped one at a time where the written fields
        // differ from the solver's.
        if let (Some(raw), Some(path)) = (spooled.as_ref(), &options.spool) {
            let written = output_map.written(session.model());
            if !output_map.keeps_fields(&written) {
                let path = path.with_extension("mapped");
                let failed = |e: std::io::Error| spool_error(&path, e);
                let mut mapped = Spool::create(&path).map_err(failed)?;
                for index in 0..raw.len() {
                    let mut increment = raw.read(index).map_err(failed)?;
                    output_map.map_increment(&written, &mut increment);
                    mapped.push(increment).map_err(failed)?;
                }
                spooled = Some(mapped);
            }
        }
        plxs_output::write_sta(&output("sta"), session.sta_rows())?;
        plxs_output::write_cvg(&output("cvg"), session.cvg_rows())?;
        if options.write_files {
            plxs_output::write_frd_as(
                &output("frd"),
                &model,
                &results,
                VERSION,
                options.frd_format,
            )?;
            if !results.history.is_empty() {
                plxs_output::write_dat(&output("dat"), &results)?;
            }
        }
        // A frequency step that stored its modes (`*FREQUENCY, STORAGE=YES`): the `.eig`
        // file holds them in `postcard` encoding, an own format that only `plxs` reads
        // (Nodalis handles the file by name only, also when it keeps the results in
        // memory).
        if let Some(modes) = &session.state().stored_modes {
            let path = output("eig");
            let io = |message: String| Error::Io {
                path: path.display().to_string(),
                message,
            };
            let bytes = postcard::to_stdvec(modes).map_err(|e| io(e.to_string()))?;
            std::fs::write(&path, bytes).map_err(|e| io(e.to_string()))?;
        }
        *kept = Some(KeptResults {
            model,
            results,
            spool: spooled,
        });
        if let Some(sink) = &options.profile_sink {
            let model = session.model();
            *sink.lock().unwrap_or_else(|e| e.into_inner()) = Some(JobProfile {
                phases: session.profile(),
                total: started.elapsed(),
                writing: writing.elapsed(),
                nodes: model.nodes.len(),
                elements: model.elements.len(),
                degrees_of_freedom: session.degrees_of_freedom(),
                threads: session.threads(),
            });
        }
        Ok(())
    };
    for (index, step) in analysis.steps.iter().enumerate().skip(first) {
        for variable in crate::unsupported_outputs(step) {
            progress.event(ProgressEvent::Warning(format!(
                "output variable {variable} is not implemented yet and is left out"
            )));
        }
        if options.cancel.is_cancelled() {
            write(&mut session)?;
            return Ok(JobOutcome::Cancelled);
        }
        let outcome = session.run_step(step, progress)?;
        // A step done, its increments go to the spool: the next step does not change them.
        session.spill_increments();
        match outcome {
            StepOutcome::Converged => {
                if let (true, Some(cache), Some(keys)) = (keeping, cache, &keys) {
                    let snapshot = session.snapshot();
                    let spooled = spool.as_ref().map(SharedSpool::lock);
                    cache.keep(
                        keys[index],
                        session.state().steps_done,
                        &snapshot,
                        session.results_mut(),
                        spooled.as_ref().and_then(|s| s.as_ref()),
                    );
                }
            }
            // The steps after a step that did not finish are not kept: the cache holds a
            // start of the step chain.
            StepOutcome::Partial { .. } => keeping = false,
            StepOutcome::Diverged {
                increment,
                failure,
                message,
                item,
            } => {
                write(&mut session)?;
                // The step that did not converge is the one after those done.
                let error = Error::Numerical {
                    step: session.state().steps_done + 1,
                    increment,
                    failure,
                    message: match failure {
                        plxs_core::Failure::NoConvergence(_) => {
                            format!("no convergence: {message}")
                        }
                        _ => message,
                    },
                };
                return Err(match item {
                    Some(item) => error.concerning(item),
                    None => error,
                });
            }
            StepOutcome::Cancelled => {
                write(&mut session)?;
                return Ok(JobOutcome::Cancelled);
            }
        }
        if let (Some(file), Some(steps), Some(write)) = (
            restart_file.as_mut(),
            source_steps.as_ref(),
            step.restart_write.filter(|w| w.frequency > 0),
        ) {
            // The state at the end of the step; CalculiX's FREQUENCY counts increments,
            // a snapshot inside a step is not needed to continue with the next step.
            file.analysis.steps = steps[..=index].to_vec();
            file.push(session.snapshot(), write.overlay);
            if options.write_files {
                file.write(&output("rout"))?;
            }
        }
    }
    let writing = std::time::Instant::now();
    write(&mut session)?;
    if options.profile {
        for line in session.profile().report(started.elapsed()) {
            progress.event(ProgressEvent::Info(line));
        }
        progress.event(ProgressEvent::Info(format!(
            "(of which writing the result files {:.3} s)",
            writing.elapsed().as_secs_f64()
        )));
    }
    progress.event(ProgressEvent::JobDone);
    Ok(JobOutcome::Finished)
}

/// The lines CalculiX would print for an event: Nodalis's monitor reads them from the
/// command line and from the built-in solver alike.
pub fn format_event(event: &ProgressEvent) -> Vec<String> {
    match event {
        ProgressEvent::Step { step } => vec![format!(" STEP {step}"), String::new()],
        ProgressEvent::Increment {
            increment,
            attempt,
            size,
            step_time,
            ..
        } => vec![
            format!(" increment {increment} attempt {attempt}"),
            format!(" increment size= {size:e}"),
            format!(" actual step time= {step_time:e}"),
        ],
        ProgressEvent::Iteration { iteration, .. } => vec![format!(" iteration {iteration}")],
        ProgressEvent::Cutback { reason, .. } => {
            vec![format!(" *INFO: increment cut back: {reason}")]
        }
        ProgressEvent::Eigenvalue { mode, value, .. } => {
            vec![format!(" eigenvalue {mode}: {value:e}")]
        }
        ProgressEvent::LinearSolve { quiet: true, .. } => Vec::new(),
        ProgressEvent::LinearSolve {
            iterations,
            relative_residual,
            seconds,
            ..
        } => vec![format!(
            " *INFO: iterative solver: {iterations} iterations, relative residual \
             {relative_residual:.1e}, {seconds:.2} s"
        )],
        ProgressEvent::Warning(text) => vec![format!("*WARNING: {text}")],
        ProgressEvent::Info(text) => vec![format!(" *INFO: {text}")],
        ProgressEvent::IncrementDone { .. }
        | ProgressEvent::StepDone { .. }
        | ProgressEvent::Integration { .. } => Vec::new(),
        ProgressEvent::JobDone => vec![" Job finished".into()],
    }
}
