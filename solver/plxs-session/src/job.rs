//! A whole job as the command line and PrePoLix run it: read the input, expand shells and
//! beams, compute the steps on a session, write the result files. The command line prints
//! the events in CalculiX's wording ([`format_event`]); PrePoLix consumes them directly.

use std::path::{Path, PathBuf};

use plxs_core::step::Procedure;
use plxs_core::{Analysis, CancelToken, Error, Progress, ProgressEvent, Result};

use plxs_core::state::Snapshot;

use crate::restart::RestartFile;
use crate::{Session, SessionOptions, StepOutcome};

/// The version of the solver, written into the result files.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[derive(Clone, Debug)]
pub struct JobOptions {
    /// Threads of the run (0: all cores).
    pub threads: usize,
    /// The job's directory: result files go here, files the input refers to are read here.
    pub work_dir: PathBuf,
    /// Name of the job: the result files are `<name>.frd`, `.dat`, `.sta`.
    pub name: String,
    /// Writes the result files; off when the caller takes the results from memory.
    pub write_files: bool,
    /// How the numbers of the `.frd` are written.
    pub frd_format: plxs_output::FrdFormat,
    /// Reports the time per phase at the end of the job, as `Info` events.
    pub profile: bool,
    pub cancel: CancelToken,
}

impl JobOptions {
    /// A job `name` in `work_dir` with the defaults: all cores, result files written, text
    /// `.frd`, no profile.
    pub fn new(work_dir: impl Into<PathBuf>, name: impl Into<String>) -> Self {
        Self {
            threads: 0,
            work_dir: work_dir.into(),
            name: name.into(),
            write_files: true,
            frd_format: plxs_output::FrdFormat::Ascii,
            profile: false,
            cancel: CancelToken::new(),
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
    run_continuing(analysis, snapshot, options, progress)
}

/// Runs a parsed analysis, see [`run_job`].
pub fn run_analysis(
    analysis: Analysis,
    options: &JobOptions,
    progress: &mut dyn Progress,
) -> Result<JobOutcome> {
    run_continuing(analysis, None, options, progress)
}

/// Runs a parsed analysis; with a `snapshot` (`*RESTART, READ`) from the state it holds,
/// skipping the steps computed before.
fn run_continuing(
    analysis: Analysis,
    snapshot: Option<Snapshot>,
    options: &JobOptions,
    progress: &mut dyn Progress,
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
    // Shells, membranes and beams are computed as the solids CalculiX expands them to.
    let expansion = plxs_elements::expand::expand(analysis)?;
    let analysis = expansion.analysis;
    let output_map = expansion.output;
    let session_options = SessionOptions {
        threads: options.threads,
        work_dir: options.work_dir.clone(),
        cancel: options.cancel.clone(),
    };
    let mut session = match snapshot {
        None => Session::new(analysis.model, session_options)?,
        Some(snapshot) => Session::resume(analysis.model, snapshot, session_options)?,
    };
    let output = |ext: &str| options.output(ext);
    let write = |session: &Session| -> Result<()> {
        if !options.write_files {
            return Ok(());
        }
        let (model, results) = output_map.apply(session.model(), session.results());
        plxs_output::write_frd_as(
            &output("frd"),
            &model,
            &results,
            VERSION,
            options.frd_format,
        )?;
        plxs_output::write_sta(&output("sta"), session.sta_rows())?;
        plxs_output::write_cvg(&output("cvg"), session.cvg_rows())?;
        if !session.results().history.is_empty() {
            plxs_output::write_dat(&output("dat"), session.results())?;
        }
        // A frequency step that stored its modes (`*FREQUENCY, STORAGE=YES`): the `.eig`
        // file holds them in `postcard` encoding, an own format that only `plxs` reads
        // (PrePoLix handles the file by name only).
        if let Some(modes) = &session.state().stored_modes {
            let path = output("eig");
            let io = |message: String| Error::Io {
                path: path.display().to_string(),
                message,
            };
            let bytes = postcard::to_stdvec(modes).map_err(|e| io(e.to_string()))?;
            std::fs::write(&path, bytes).map_err(|e| io(e.to_string()))?;
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
            write(&session)?;
            return Ok(JobOutcome::Cancelled);
        }
        match session.run_step(step, progress)? {
            StepOutcome::Converged | StepOutcome::Partial { .. } => {}
            StepOutcome::Diverged { increment, message } => {
                write(&session)?;
                return Err(Error::Numerical {
                    step: session.state().steps_done,
                    increment,
                    message: format!("no convergence: {message}"),
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

/// The lines CalculiX would print for an event: PrePoLix's monitor reads them from the
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
        ProgressEvent::IncrementDone { .. } | ProgressEvent::StepDone { .. } => Vec::new(),
        ProgressEvent::JobDone => vec![" Job finished".into()],
    }
}
