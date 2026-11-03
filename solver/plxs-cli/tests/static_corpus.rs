//! The M1 acceptance test: every fixture of the `static` area solved by `plxs` matches its
//! plxs snapshot within its class tolerance.

use std::path::Path;

use plxs_verify::report::format_reports;
use plxs_verify::{corpus, fixtures_root, run_corpus};

#[test]
fn static_fixtures_are_unchanged() {
    let fixtures = corpus(&fixtures_root()).expect("fixture corpus");
    let work = std::env::temp_dir().join(format!("plxs-static-corpus-{}", std::process::id()));
    let reports = run_corpus(
        Path::new(env!("CARGO_BIN_EXE_plxs")),
        &fixtures,
        &work,
        |f| f.meta.area == "static",
    );
    let _ = std::fs::remove_dir_all(&work);
    eprintln!("{}", format_reports(&reports));
    assert!(!reports.is_empty());
    for report in &reports {
        assert!(
            report.outcome.passed() || !report.outcome.compared(),
            "{}: {:?}\n{}",
            report.fixture,
            report.outcome,
            report.log
        );
    }
}
