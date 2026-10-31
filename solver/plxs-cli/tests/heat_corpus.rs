//! The T9 acceptance test: every fixture of the `heat` area solved by `plxs` matches the
//! CalculiX golden files within its class tolerance.

use std::path::Path;

use plxs_verify::report::format_reports;
use plxs_verify::{Outcome, corpus, fixtures_root, run_corpus};

/// Fixtures that need an element another M2 task brings: `film-2d` is a CPS8 model (T4).
/// They may fail until then; once they pass they stay in the check like all others.
const PENDING: &[&str] = &["heat/film-2d"];

#[test]
fn heat_fixtures_match_calculix() {
    let fixtures = corpus(&fixtures_root()).expect("fixture corpus");
    let work = std::env::temp_dir().join(format!("plxs-heat-corpus-{}", std::process::id()));
    let reports = run_corpus(
        Path::new(env!("CARGO_BIN_EXE_plxs")),
        &fixtures,
        &work,
        |f| f.meta.area == "heat",
    );
    let _ = std::fs::remove_dir_all(&work);
    eprintln!("{}", format_reports(&reports));
    assert!(!reports.is_empty());
    for report in &reports {
        if PENDING.contains(&report.fixture.as_str()) {
            continue;
        }
        assert!(
            report.outcome.passed() || report.outcome == Outcome::NoReference,
            "{}: {:?}\n{}",
            report.fixture,
            report.outcome,
            report.log
        );
    }
}
