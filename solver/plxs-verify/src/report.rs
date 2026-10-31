//! Running a solver over fixtures and reporting the outcome.

use std::path::Path;
use std::process::Command;
use std::time::{Duration, Instant};

use crate::corpus::load_results;
use crate::{Comparison, Fixture, compare_dat, compare_frd_except, load_reference};

/// How a fixture went.
#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    /// Results within tolerance.
    Pass { max_relative: f64 },
    /// Results differ: the failures of the comparison.
    Fail(Vec<String>),
    /// The solver refused or crashed; the first `*ERROR` line or the exit status.
    Error(String),
    /// No golden files for this fixture; nothing to compare with.
    NoReference,
}

impl Outcome {
    pub fn passed(&self) -> bool {
        matches!(self, Outcome::Pass { .. })
    }

    pub fn label(&self) -> &'static str {
        match self {
            Outcome::Pass { .. } => "pass",
            Outcome::Fail(_) => "FAIL",
            Outcome::Error(_) => "ERROR",
            Outcome::NoReference => "no reference",
        }
    }
}

#[derive(Clone, Debug)]
pub struct Report {
    pub fixture: String,
    pub outcome: Outcome,
    pub duration: Duration,
    /// The first lines of the solver's stdout, for the error report.
    pub log: String,
}

/// Runs `solver` (a `ccx`-compatible command line: `solver -i <name>` in a directory with
/// `<name>.inp`) on a fixture in `work` and compares the results with the golden files.
pub fn run_fixture(solver: &Path, fixture: &Fixture, work: &Path) -> Report {
    let start = Instant::now();
    let name = fixture.id();
    let report = |outcome, log: String| Report {
        fixture: name.clone(),
        outcome,
        duration: start.elapsed(),
        log,
    };
    if !fixture.has_reference() {
        return report(Outcome::NoReference, String::new());
    }
    let reference = match load_reference(fixture) {
        Ok(r) => r,
        Err(e) => return report(Outcome::Error(format!("golden files: {e}")), String::new()),
    };
    if let Err(e) = std::fs::create_dir_all(work) {
        return report(Outcome::Error(e.to_string()), String::new());
    }
    let job = &fixture.meta.name;
    for ext in ["frd", "dat", "sta", "cvg", "12d"] {
        let _ = std::fs::remove_file(work.join(format!("{job}.{ext}")));
    }
    if let Err(e) = std::fs::copy(fixture.inp(), work.join(format!("{job}.inp"))) {
        return report(Outcome::Error(e.to_string()), String::new());
    }
    let output = match Command::new(solver)
        .args(["-i", job])
        .current_dir(work)
        .env("OMP_NUM_THREADS", "1")
        .output()
    {
        Ok(o) => o,
        Err(e) => {
            return report(
                Outcome::Error(format!("{}: {e}", solver.display())),
                String::new(),
            );
        }
    };
    let log = String::from_utf8_lossy(&output.stdout).into_owned();
    let short_log: String = log.lines().take(40).collect::<Vec<_>>().join("\n");
    if !output.status.success() || log.contains("*ERROR") {
        let error = log
            .lines()
            .find(|l| l.contains("*ERROR"))
            .map(|l| l.trim().to_string())
            .unwrap_or_else(|| format!("exit {}", output.status));
        return report(Outcome::Error(error), short_log);
    }
    let results = match load_results(work, job, &fixture.inp()) {
        Ok(r) => r,
        Err(e) => return report(Outcome::Error(format!("results: {e}")), short_log),
    };
    let class = fixture.meta.class;
    let mut comparison: Comparison = compare_frd_except(
        &reference.frd,
        &results.frd,
        class,
        &fixture.meta.deviations,
    );
    match (&reference.dat, &results.dat) {
        (Some(r), Some(c)) => {
            let dat = compare_dat(r, c, class);
            comparison.failures.extend(dat.failures);
            comparison.max_relative = comparison.max_relative.max(dat.max_relative);
            comparison.compared += dat.compared;
        }
        (Some(_), None) => comparison
            .failures
            .push("the reference has a .dat, the candidate none".into()),
        (None, Some(_)) => comparison
            .failures
            .push("the candidate wrote a .dat, the reference has none".into()),
        (None, None) => {}
    }
    let outcome = if comparison.ok() {
        Outcome::Pass {
            max_relative: comparison.max_relative,
        }
    } else {
        Outcome::Fail(comparison.failures)
    };
    report(outcome, short_log)
}

/// Runs the fixtures that `filter` accepts, each in its own directory under `work`.
pub fn run_corpus(
    solver: &Path,
    fixtures: &[Fixture],
    work: &Path,
    filter: impl Fn(&Fixture) -> bool,
) -> Vec<Report> {
    fixtures
        .iter()
        .filter(|f| filter(f))
        .map(|f| run_fixture(solver, f, &work.join(&f.meta.area).join(&f.meta.name)))
        .collect()
}

/// A text table of the reports.
pub fn format_reports(reports: &[Report]) -> String {
    let mut out = String::new();
    let width = reports.iter().map(|r| r.fixture.len()).max().unwrap_or(10);
    for r in reports {
        let detail = match &r.outcome {
            Outcome::Pass { max_relative } => format!("max deviation {max_relative:.1e}"),
            Outcome::Fail(failures) => failures.first().cloned().unwrap_or_default(),
            Outcome::Error(e) => e.clone(),
            Outcome::NoReference => String::new(),
        };
        out.push_str(&format!(
            "{:<width$}  {:<12} {:>7.2}s  {}\n",
            r.fixture,
            r.outcome.label(),
            r.duration.as_secs_f64(),
            detail
        ));
    }
    let passed = reports.iter().filter(|r| r.outcome.passed()).count();
    let with_reference = reports
        .iter()
        .filter(|r| r.outcome != Outcome::NoReference)
        .count();
    out.push_str(&format!("{passed} of {with_reference} fixtures pass\n"));
    out
}

/// Keywords nearly every fixture has; the feature matrix leaves them out to show what a
/// fixture is about.
pub const BASELINE_FEATURES: &[&str] = &[
    "*HEADING",
    "*NODE",
    "*ELEMENT",
    "*NSET",
    "*ELSET",
    "*SURFACE",
    "*MATERIAL",
    "*ELASTIC",
    "*DENSITY",
    "*SOLID SECTION",
    "*STEP",
    "*END STEP",
    "*BOUNDARY",
    "*CLOAD",
    "*DLOAD",
    "*NODE FILE",
    "*EL FILE",
];

/// The feature matrix of `docs/solver/FIXTURES.md`: every fixture with its class, stage,
/// reference and the features beyond [`BASELINE_FEATURES`], as a Markdown table.
pub fn feature_matrix(fixtures: &[Fixture]) -> String {
    let mut out = String::new();
    out.push_str("| Fixture | Class | Stage | Reference | Features |\n|---|---|---|---|---|\n");
    for f in fixtures {
        let features: Vec<&str> = (f.meta.features.iter())
            .map(String::as_str)
            .filter(|f| !BASELINE_FEATURES.contains(f))
            .collect();
        out.push_str(&format!(
            "| `{}` | {} | {} | {} | {} |\n",
            f.id(),
            f.meta.class.letter(),
            f.meta.stage.name(),
            if f.has_reference() {
                f.meta.reference.as_str()
            } else {
                "none"
            },
            features.join(", ").replace('*', "\\*")
        ));
    }
    out
}
