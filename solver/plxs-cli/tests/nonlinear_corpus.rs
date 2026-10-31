//! The M2 T8 acceptance test: every fixture of the `nonlinear` area that `plxs` can solve
//! matches the CalculiX golden files within its class tolerance. Fixtures with element
//! types of other M2 threads (springs, gaps, membranes) are allowed to be refused until
//! those land; a wrong result is not.

use std::path::Path;

use plxs_verify::report::format_reports;
use plxs_verify::{Outcome, corpus, fixtures_root, run_corpus};

#[test]
fn nonlinear_fixtures_match_calculix() {
    let fixtures = corpus(&fixtures_root()).expect("fixture corpus");
    let work = std::env::temp_dir().join(format!("plxs-nonlinear-corpus-{}", std::process::id()));
    let reports = run_corpus(
        Path::new(env!("CARGO_BIN_EXE_plxs")),
        &fixtures,
        &work,
        |f| f.meta.area == "nonlinear",
    );
    let _ = std::fs::remove_dir_all(&work);
    eprintln!("{}", format_reports(&reports));
    let passed = reports.iter().filter(|r| r.outcome.passed()).count();
    assert!(passed >= 6, "only {passed} nonlinear fixtures pass");
    for report in &reports {
        let acceptable = match &report.outcome {
            Outcome::Pass { .. } | Outcome::NoReference => true,
            Outcome::Error(message) => message.contains("is not implemented yet"),
            Outcome::Fail(_) => false,
        };
        assert!(
            acceptable,
            "{}: {:?}\n{}",
            report.fixture, report.outcome, report.log
        );
    }
}
