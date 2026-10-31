//! The M2 T6 acceptance test: every fixture of the `constraints` area and the
//! compression-only (`GAPUNI`) fixtures of the `nonlinear` area solved by `plxs` match the
//! CalculiX golden files within their class tolerance, apart from the fixtures listed in
//! [`WAITING`], which need features of other M2 stages.

use std::path::Path;

use plxs_verify::report::format_reports;
use plxs_verify::{Fixture, Outcome, corpus, fixtures_root, run_corpus};

/// Fixtures that need other stages, with the reason. A listed fixture may pass (once the
/// other stage is merged); one that is not listed must.
const WAITING: &[(&str, &str)] = &[
    ("constraints/referenzpunkt-2d", "CPS8 elements (M2 T4)"),
    ("constraints/remote-2d", "CPS8 elements (M2 T4)"),
    ("constraints/remote-2d-deformable", "CPS8 elements (M2 T4)"),
    ("constraints/symmetrie-2d-30", "CPS8 elements (M2 T4)"),
    (
        "constraints/verteilte-temperatur",
        "thermal expansion, *TEMPERATURE and NT output (M2 T9/T10)",
    ),
    (
        "constraints/tet-schraeg",
        "C3D10 nodal stresses 1.7e-5 off ccx everywhere while displacements and forces agree to 1e-6 (element stage, not the constraints)",
    ),
    (
        "constraints/tet-vorspannung",
        "C3D10 nodal stresses 1.8e-5 off ccx, as tet-schraeg",
    ),
];

fn selected(f: &Fixture) -> bool {
    f.meta.area == "constraints"
        || (f.meta.area == "nonlinear" && f.meta.name.starts_with("druck-nur"))
}

#[test]
fn constraint_fixtures_match_calculix() {
    let fixtures = corpus(&fixtures_root()).expect("fixture corpus");
    let work = std::env::temp_dir().join(format!("plxs-constraints-corpus-{}", std::process::id()));
    let reports = run_corpus(
        Path::new(env!("CARGO_BIN_EXE_plxs")),
        &fixtures,
        &work,
        selected,
    );
    let _ = std::fs::remove_dir_all(&work);
    eprintln!("{}", format_reports(&reports));
    assert!(reports.len() >= 46, "{} fixtures", reports.len());
    for report in &reports {
        if WAITING.iter().any(|(name, _)| *name == report.fixture) {
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

/// The plx-io writer re-pivots equations whose first term is held by an SPC, because ccx
/// stops on them. plxs resolves that itself: with the original term order the result is the
/// same as with the writer's.
#[test]
fn held_dependent_dofs_are_repivoted() {
    let fixtures = corpus(&fixtures_root()).expect("fixture corpus");
    let original = fixtures
        .iter()
        .find(|f| f.id() == "constraints/region-no-separation-held")
        .expect("fixture");
    let work = std::env::temp_dir().join(format!("plxs-repivot-{}", std::process::id()));
    let dir = work.join("fixtures").join("constraints");
    std::fs::create_dir_all(&dir).unwrap();
    // Put the held node (1xx, the SPC'd side) first in every equation.
    let input = std::fs::read_to_string(original.inp()).unwrap();
    let mut lines: Vec<String> = Vec::new();
    let mut in_equation = 0;
    let mut swapped = 0;
    for line in input.lines() {
        if line.eq_ignore_ascii_case("*equation") {
            in_equation = 2;
            lines.push(line.to_string());
            continue;
        }
        if in_equation == 2 {
            in_equation = 1;
        } else if in_equation == 1 {
            in_equation = 0;
            let f: Vec<&str> = line.split(',').map(str::trim).collect();
            if f.len() == 6 && f[0].len() == 2 && f[3].len() == 3 {
                lines.push(format!(
                    "{}, {}, {}, {}, {}, {}",
                    f[3], f[4], f[5], f[0], f[1], f[2]
                ));
                swapped += 1;
                continue;
            }
        }
        lines.push(line.to_string());
    }
    assert!(swapped >= 3, "{swapped} equations swapped");
    let base = dir.join("repivot");
    std::fs::write(base.with_extension("inp"), lines.join("\n")).unwrap();
    std::fs::copy(original.ref_frd(), dir.join("repivot.ref.frd.gz")).unwrap();
    let toml = std::fs::read_to_string(original.toml())
        .unwrap()
        .replace("region-no-separation-held", "repivot");
    std::fs::write(base.with_extension("toml"), toml).unwrap();

    let fixtures = corpus(&work.join("fixtures")).unwrap();
    let reports = run_corpus(
        Path::new(env!("CARGO_BIN_EXE_plxs")),
        &fixtures,
        &work.join("run"),
        |_| true,
    );
    let _ = std::fs::remove_dir_all(&work);
    assert_eq!(reports.len(), 1);
    assert!(
        reports[0].outcome.passed(),
        "{:?}\n{}",
        reports[0].outcome,
        reports[0].log
    );
}
