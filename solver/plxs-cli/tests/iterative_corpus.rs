//! The iterative equation solver (stage 5 of the ccx replacement): the `static` and `heat`
//! fixtures, every step switched to `SOLVER=ITERATIVE CHOLESKY`, still match the CalculiX
//! golden files within their class tolerance.

use std::path::{Path, PathBuf};

use plxs_verify::report::format_reports;
use plxs_verify::{Outcome, corpus, fixtures_root, run_corpus};

/// Procedures that solve with conjugate gradients when asked to.
const ITERATIVE: [&str; 4] = [
    "*STATIC",
    "*HEAT TRANSFER",
    "*COUPLED TEMPERATURE-DISPLACEMENT",
    "*UNCOUPLED TEMPERATURE-DISPLACEMENT",
];

/// The input with `SOLVER=ITERATIVE CHOLESKY` on each procedure line that names no solver.
fn iterative(input: &str) -> String {
    let mut out = String::with_capacity(input.len() + 256);
    for line in input.lines() {
        let upper = line.trim().to_ascii_uppercase();
        let keyword = upper.split(',').next().unwrap_or("").trim().to_owned();
        out.push_str(line);
        if ITERATIVE.contains(&keyword.as_str()) && !upper.contains("SOLVER") {
            out.push_str(", SOLVER=ITERATIVE CHOLESKY");
        }
        out.push('\n');
    }
    out
}

fn copy_area(from: &Path, to: &Path) {
    std::fs::create_dir_all(to).unwrap();
    for entry in std::fs::read_dir(from).unwrap() {
        let path = entry.unwrap().path();
        if !path.is_file() {
            continue;
        }
        let target = to.join(path.file_name().unwrap());
        if path.extension().is_some_and(|e| e == "inp") {
            let text = std::fs::read_to_string(&path).unwrap();
            std::fs::write(target, iterative(&text)).unwrap();
        } else {
            std::fs::copy(&path, target).unwrap();
        }
    }
}

#[test]
fn static_and_heat_fixtures_match_calculix_with_the_iterative_solver() {
    let root: PathBuf =
        std::env::temp_dir().join(format!("plxs-iterative-corpus-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    for area in ["static", "heat"] {
        copy_area(
            &fixtures_root().join(area),
            &root.join("fixtures").join(area),
        );
    }
    let fixtures = corpus(&root.join("fixtures")).expect("fixture corpus");
    let work = root.join("work");
    let reports = run_corpus(
        Path::new(env!("CARGO_BIN_EXE_plxs")),
        &fixtures,
        &work,
        |_| true,
    );
    eprintln!("{}", format_reports(&reports));
    let iterated = reports
        .iter()
        .filter(|r| r.log.contains("iterative solver"))
        .count();
    let _ = std::fs::remove_dir_all(&root);
    assert!(!reports.is_empty());
    assert!(
        iterated > 10,
        "only {iterated} runs used the iterative solver"
    );
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
