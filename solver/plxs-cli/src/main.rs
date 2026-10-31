//! `plxs`: the PrePoLix solver on the command line, called like CalculiX:
//!
//! ```text
//! plxs <job>            read <job>.inp, compute, write <job>.frd/.dat/.sta/.cvg
//! plxs -i <job>         the same
//! plxs -v               print the version
//! plxs --threads <n>    number of threads (default: OMP_NUM_THREADS, else all cores)
//! ```
//!
//! stdout follows CalculiX's lines (`STEP n`, `increment n attempt m`, `iteration n`, `Job
//! finished`, `*ERROR ...`), because PrePoLix's job monitor and solver self-test read them;
//! the start line carries the word `CalculiX` for the self-test. stdout is written line by
//! line without buffering so that the monitor sees progress as it happens.
//!
//! M0 computes `*NO ANALYSIS` only (read, validate, report); every other procedure answers
//! `*ERROR: procedure not implemented yet`. Exit code 0 when the job finished, 1 otherwise.

use std::io::Write;
use std::path::{Path, PathBuf};

use plxs_core::step::Procedure;
use plxs_core::{Error, ProgressEvent};

const VERSION: &str = env!("CARGO_PKG_VERSION");

fn main() {
    let code = run(std::env::args().skip(1).collect());
    std::process::exit(code);
}

/// A line to stdout, flushed at once.
fn say(line: &str) {
    let mut out = std::io::stdout().lock();
    let _ = writeln!(out, "{line}");
    let _ = out.flush();
}

fn banner() {
    say("************************************************************");
    say(&format!(
        "PrePoLix solver plxs {VERSION}, CalculiX-compatible input"
    ));
    say(&format!("This is Version {VERSION}"));
    say("************************************************************");
    say("");
}

fn run(args: Vec<String>) -> i32 {
    let mut job: Option<String> = None;
    let mut threads: Option<usize> = None;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "-v" | "--version" => {
                banner();
                return 0;
            }
            "-i" => {
                i += 1;
                job = args.get(i).cloned();
            }
            "--threads" | "-t" => {
                i += 1;
                threads = args.get(i).and_then(|t| t.parse().ok());
            }
            "-h" | "--help" => {
                say("usage: plxs [-i] <job> [--threads <n>] | plxs -v");
                return 0;
            }
            other if !other.starts_with('-') => job = Some(other.to_string()),
            other => {
                say(&format!("*ERROR: unknown option {other}"));
                return 1;
            }
        }
        i += 1;
    }
    let Some(job) = job else {
        say("usage: plxs [-i] <job> [--threads <n>] | plxs -v");
        return 1;
    };
    banner();
    let threads = threads
        .or_else(|| {
            std::env::var("OMP_NUM_THREADS")
                .ok()
                .and_then(|t| t.parse().ok())
        })
        .filter(|&t| t > 0)
        .unwrap_or_else(|| {
            std::thread::available_parallelism()
                .map(|n| n.get())
                .unwrap_or(1)
        });
    say(&format!("Using up to {threads} cpu(s) for the solver."));
    say("");

    let path = input_path(&job);
    let job_dir = path.parent().map(Path::to_path_buf).unwrap_or_default();
    let stem = path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| job.clone());
    let output = |ext: &str| job_dir.join(format!("{stem}.{ext}"));

    let text = match std::fs::read_to_string(&path) {
        Ok(text) => text,
        Err(e) => {
            report_error(&Error::Io {
                path: path.display().to_string(),
                message: e.to_string(),
            });
            return 1;
        }
    };
    // `*RESTART, READ`: the model and the steps before come from `<job>.rin`.
    let mut snapshot = None;
    let reading = match plxs_input::restart_read(&text) {
        None => plxs_input::read_inp_str(&text, Some(&path)),
        Some(step) => {
            let rin = output("rin");
            let file = match plxs_session::restart::RestartFile::read(&rin) {
                Ok(file) => file,
                Err(error) => {
                    report_error(&error);
                    return 1;
                }
            };
            let Some(found) = file.after(step).cloned() else {
                say("");
                say(&format!(
                    "*ERROR in restartread: {} holds no state after step {}",
                    rin.display(),
                    step.map_or_else(|| "any".to_string(), |s| s.to_string())
                ));
                say("");
                return 1;
            };
            let base = file.base(found.state.steps_done);
            snapshot = Some(found);
            plxs_input::read_inp_continuing(&text, Some(&path), base)
        }
    };
    let reading = match reading {
        Ok(reading) => reading,
        Err(errors) => {
            for error in &errors {
                report_error(error);
            }
            return 1;
        }
    };
    for warning in &reading.warnings {
        say(warning);
    }
    let analysis = reading.analysis;
    // Steps already computed by the run the restart continues.
    let first = snapshot.as_ref().map_or(0, |s| s.state.steps_done as usize);
    say(&format!(
        " number of nodes: {}\n number of elements: {}\n number of steps: {}",
        analysis.model.nodes.len(),
        analysis.model.elements.len(),
        analysis.steps.len() - first
    ));
    say("");

    let mut progress = |event: ProgressEvent| print_event(&event);

    // A model check only reads the input.
    if analysis.steps[first..]
        .iter()
        .all(|s| matches!(s.procedure, Procedure::NoAnalysis))
    {
        for index in first..analysis.steps.len() {
            plxs_core::Progress::event(
                &mut progress,
                ProgressEvent::Step {
                    step: index as u32 + 1,
                },
            );
            say(" *NO ANALYSIS: the model was read and checked, nothing is computed.");
        }
        say("");
        say(" Job finished");
        say("");
        return 0;
    }

    // What `*RESTART, WRITE` stores: the analysis as read, before the expansion.
    let mut restart_file = (analysis.steps[first..].iter())
        .any(|s| s.restart_write.is_some_and(|w| w.frequency > 0))
        .then(|| plxs_session::restart::RestartFile {
            analysis: plxs_core::Analysis {
                model: analysis.model.clone(),
                steps: Vec::new(),
            },
            snapshots: Vec::new(),
        });
    let source_steps = restart_file.as_ref().map(|_| analysis.steps.clone());

    // Shells, membranes and beams are computed as the solids CalculiX expands them to.
    let expansion = match plxs_elements::expand::expand(analysis) {
        Ok(e) => e,
        Err(error) => {
            report_error(&error);
            return 1;
        }
    };
    let analysis = expansion.analysis;
    let output_map = expansion.output;

    let options = plxs_session::SessionOptions {
        threads,
        work_dir: job_dir.clone(),
    };
    let session = match snapshot {
        None => plxs_session::Session::new(analysis.model, options),
        Some(snapshot) => plxs_session::Session::resume(analysis.model, snapshot, options),
    };
    let mut session = match session {
        Ok(s) => s,
        Err(error) => {
            report_error(&error);
            return 1;
        }
    };
    for (index, step) in analysis.steps.iter().enumerate().skip(first) {
        for variable in plxs_procedures_unsupported(step) {
            say(&format!(
                "*WARNING: output variable {variable} is not implemented yet and is left out"
            ));
        }
        match session.run_step(step, &mut progress) {
            Ok(plxs_session::StepOutcome::Converged) => {}
            Ok(plxs_session::StepOutcome::Diverged { increment, message }) => {
                say(&format!(
                    "*ERROR: no convergence in increment {increment}: {message}"
                ));
                let _ = write_results(&session, &output_map, &output);
                return 1;
            }
            Ok(plxs_session::StepOutcome::Partial { .. }) => {}
            Err(error) => {
                report_error(&error);
                return 1;
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
            if let Err(error) = file.write(&output("rout")) {
                report_error(&error);
                return 1;
            }
        }
        say("");
    }
    if let Err(error) = write_results(&session, &output_map, &output) {
        report_error(&error);
        return 1;
    }
    say("");
    say(" Job finished");
    say("");
    0
}

fn plxs_procedures_unsupported(step: &plxs_core::Step) -> Vec<String> {
    plxs_session::unsupported_outputs(step)
}

/// Writes `.frd`, `.sta` and, when there is history output, `.dat`.
fn write_results(
    session: &plxs_session::Session,
    map: &plxs_elements::expand::OutputMap,
    output: &dyn Fn(&str) -> PathBuf,
) -> Result<(), Error> {
    let (model, results) = map.apply(session.model(), session.results());
    plxs_output::write_frd(&output("frd"), &model, &results, VERSION)?;
    plxs_output::write_sta(&output("sta"), session.sta_rows())?;
    plxs_output::write_cvg(&output("cvg"), session.cvg_rows())?;
    if !session.results().history.is_empty() {
        plxs_output::write_dat(&output("dat"), session.results())?;
    }
    Ok(())
}

/// `<job>` or `<job>.inp`.
fn input_path(job: &str) -> PathBuf {
    let path = Path::new(job);
    if path
        .extension()
        .is_some_and(|e| e.eq_ignore_ascii_case("inp"))
    {
        path.to_path_buf()
    } else {
        PathBuf::from(format!("{job}.inp"))
    }
}

fn report_error(error: &Error) {
    say("");
    say(&error.to_string());
    if let Some(location) = error.location() {
        say(&format!(" in {}, line {}", location.file, location.line));
    }
    say("");
}

/// Progress events in CalculiX's wording, the lines PrePoLix's monitor reads.
fn print_event(event: &ProgressEvent) {
    match event {
        ProgressEvent::Step { step } => {
            say(&format!(" STEP {step}"));
            say("");
        }
        ProgressEvent::Increment {
            increment,
            attempt,
            size,
            step_time,
            ..
        } => {
            say(&format!(" increment {increment} attempt {attempt}"));
            say(&format!(" increment size= {size:e}"));
            say(&format!(" actual step time= {step_time:e}"));
        }
        ProgressEvent::Iteration { iteration, .. } => say(&format!(" iteration {iteration}")),
        ProgressEvent::Cutback { reason, .. } => {
            say(&format!(" *INFO: increment cut back: {reason}"))
        }
        ProgressEvent::Eigenvalue { mode, value, .. } => {
            say(&format!(" eigenvalue {mode}: {value:e}"))
        }
        ProgressEvent::Warning(text) => say(&format!("*WARNING: {text}")),
        ProgressEvent::Info(text) => say(&format!(" *INFO: {text}")),
        ProgressEvent::IncrementDone { .. } | ProgressEvent::StepDone { .. } => {}
        ProgressEvent::JobDone => say(" Job finished"),
    }
}
