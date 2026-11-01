//! `plxs`: the PrePoLix solver on the command line, called like CalculiX:
//!
//! ```text
//! plxs <job>            read <job>.inp, compute, write <job>.frd/.dat/.sta/.cvg
//! plxs -i <job>         the same
//! plxs -v               print the version
//! plxs --threads <n>    number of threads (default: OMP_NUM_THREADS, else all cores)
//! plxs --check <job>    model check only: free motions, untied nodes, size (no results)
//! ```
//!
//! stdout follows CalculiX's lines (`STEP n`, `increment n attempt m`, `iteration n`, `Job
//! finished`, `*ERROR ...`), because PrePoLix's job monitor and solver self-test read them;
//! the start line carries the word `CalculiX` for the self-test. stdout is written line by
//! line without buffering so that the monitor sees progress as it happens.
//!
//! The run itself is [`plxs_session::run_job`], shared with PrePoLix's built-in solver. Exit
//! code 0 when the job finished, 1 otherwise.

use std::io::Write;
use std::path::{Path, PathBuf};

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
    let mut check = false;
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
            "--check" => check = true,
            "-h" | "--help" => {
                say("usage: plxs [-i] <job> [--threads <n>] [--check] | plxs -v");
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
    let input = match std::fs::read_to_string(&path) {
        Ok(input) => input,
        Err(e) => {
            report_error(&Error::Io {
                path: path.display().to_string(),
                message: e.to_string(),
            });
            return 1;
        }
    };
    let job_dir = path.parent().map(Path::to_path_buf).unwrap_or_default();
    let stem = path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| job.clone());
    let options = plxs_session::JobOptions {
        threads,
        work_dir: job_dir,
        name: stem,
        write_files: true,
        cancel: plxs_core::CancelToken::new(),
    };
    let mut progress = |event: ProgressEvent| {
        for line in plxs_session::format_event(&event) {
            say(&line);
        }
    };
    if check {
        let found = plxs_session::check_input(&input, Some(&path), &options, &mut progress);
        for diagnostic in &found.diagnostics {
            say(&plxs_session::format_diagnostic(diagnostic));
        }
        say(&format!(" model check finished in {:.2} s", found.seconds));
        return i32::from(found.has_errors());
    }
    match plxs_session::run_job(&input, Some(&path), &options, &mut progress) {
        Ok(plxs_session::JobOutcome::Finished) => {
            say("");
            0
        }
        Ok(plxs_session::JobOutcome::Cancelled) => {
            say("*ERROR: the job was cancelled");
            1
        }
        Ok(plxs_session::JobOutcome::Diverged) => 1,
        Err(error) => {
            report_error(&error);
            1
        }
    }
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
