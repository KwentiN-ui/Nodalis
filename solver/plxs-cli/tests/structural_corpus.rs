//! The M2 T5 acceptance test: the fixtures of the `structural` area (shells, beams,
//! trusses, U1) solved by `plxs` match their plxs snapshots within their class tolerance.
//! The fixtures with a `known_gap` in their meta file have no snapshot and are skipped.

use std::path::Path;

use plxs_verify::report::format_reports;
use plxs_verify::{corpus, fixtures_root, run_corpus};

#[test]
fn structural_fixtures_are_unchanged() {
    let fixtures = corpus(&fixtures_root()).expect("fixture corpus");
    let work = std::env::temp_dir().join(format!("plxs-structural-{}", std::process::id()));
    let reports = run_corpus(
        Path::new(env!("CARGO_BIN_EXE_plxs")),
        &fixtures,
        &work,
        |f| f.meta.area == "structural",
    );
    let _ = std::fs::remove_dir_all(&work);
    eprintln!("{}", format_reports(&reports));
    assert!(reports.iter().any(|r| r.outcome.compared()));
    for report in &reports {
        assert!(
            report.outcome.passed() || !report.outcome.compared(),
            "{}: {:?}
{}",
            report.fixture,
            report.outcome,
            report.log
        );
    }
}
