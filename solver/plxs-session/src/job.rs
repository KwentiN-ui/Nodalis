//! A whole job as the command line and Nodalis run it: read the input, expand shells and
//! beams, compute the steps on a session, write the result files. The command line prints
//! the events in CalculiX's wording ([`format_event`]); Nodalis consumes them directly.

use std::path::{Path, PathBuf};

use plxs_core::step::Procedure;
use plxs_core::{Analysis, CancelToken, Error, Model, Progress, ProgressEvent, Result, Results};

use plxs_core::state::Snapshot;

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
    pub cancel: CancelToken,
    /// Keeps the state and results after each step, and takes over the steps a run before
    /// computed already ([`crate::cache`]); `None`: every step is computed.
    pub step_cache: Option<StepCache>,
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
            cancel: CancelToken::new(),
            step_cache: None,
        }
    }

    fn output(&self, ext: &str) -> PathBuf {
        self.work_dir.join(format!("{}.{ext}", self.name))
    }
}

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
pub type KeptResults = (Model, Results);

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
    let mut cached = None;
    if let (Some(cache), Some(keys)) = (cache, &keys) {
        cache.keep_only(keys);
        cached = cache.take(keys);
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
    // Only when the expansion kept the steps as they were (it always does).
    let keys = keys.filter(|k| k.len() == steps);
    let mut keeping = cache.is_some();
    let output = |ext: &str| options.output(ext);
    let mut write = |session: &Session| -> Result<()> {
        let (model, results) = output_map.apply(session.model(), session.results());
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
        *kept = Some((model, results));
        Ok(())
    };
    for (index, step) in analysis.steps.iter().enumerate().skip(first) {
        for variable in crate::unsupported_outputs(step) {
            progress.event(ProgressEvent::Warning(format!(
                "output variable {variable} is not implemented yet and is left out"
            )));
        }
        if options.cancel.is_cancelled() {
            write(&session)?;
            return Ok(JobOutcome::Cancelled);
        }
        match session.run_step(step, progress)? {
            StepOutcome::Converged => {
                if let (true, Some(cache), Some(keys)) = (keeping, cache, &keys) {
                    cache.keep(
                        keys[index],
                        session.state().steps_done,
                        &session.snapshot(),
                        session.results_mut(),
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
            } => {
                write(&session)?;
                // The step that did not converge is the one after those done.
                return Err(Error::Numerical {
                    step: session.state().steps_done + 1,
                    increment,
                    failure,
                    message: match failure {
                        plxs_core::Failure::NoConvergence(_) => {
                            format!("no convergence: {message}")
                        }
                        _ => message,
                    },
                });
            }
            StepOutcome::Cancelled => {
                write(&session)?;
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
    write(&session)?;
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
        ProgressEvent::Warning(text) => vec![format!("*WARNING: {text}")],
        ProgressEvent::Info(text) => vec![format!(" *INFO: {text}")],
        ProgressEvent::IncrementDone { .. }
        | ProgressEvent::StepDone { .. }
        | ProgressEvent::Integration { .. } => Vec::new(),
        ProgressEvent::JobDone => vec![" Job finished".into()],
    }
}
