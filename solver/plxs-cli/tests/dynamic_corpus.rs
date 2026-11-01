//! The M3 T13 acceptance test: every fixture of the `dynamic` area (implicit HHT,
//! initial conditions, damping) solved by `plxs` matches the
//! CalculiX golden files within its class tolerance.

use std::path::Path;

use plxs_verify::report::format_reports;
use plxs_verify::{Outcome, corpus, fixtures_root, run_corpus};

#[test]
fn dynamic_fixtures_match_calculix() {
    let fixtures = corpus(&fixtures_root()).expect("fixture corpus");
    let work = std::env::temp_dir().join(format!("plxs-dynamic-corpus-{}", std::process::id()));
    let reports = run_corpus(
        Path::new(env!("CARGO_BIN_EXE_plxs")),
        &fixtures,
        &work,
        |f| f.meta.area == "dynamic",
    );
    let _ = std::fs::remove_dir_all(&work);
    eprintln!("{}", format_reports(&reports));
    assert!(!reports.is_empty());
    for report in &reports {
        assert!(
            report.outcome.passed() || report.outcome == Outcome::NoReference,
            "{}: {:?}\n{}",
            report.fixture,
            report.outcome,
            report.log
        );
    }
}
