//! The M2 T5 acceptance test: the fixtures of the `structural` area (shells, beams,
//! trusses, U1) that plxs computes solved by `plxs` match the CalculiX golden files within
//! their class tolerance. The rest is listed with the reason it is still open; a fixture
//! that starts passing must move into the checked list.

use std::path::Path;

use plxs_verify::report::format_reports;
use plxs_verify::{corpus, fixtures_root, run_corpus};

/// Fixtures not yet computed like ccx, and why.
const OPEN: &[(&str, &str)] = &[
    (
        "balken_b32r_box",
        "BOX: Simpson rule on the wall centerlines is 2e-4 too stiff, ccx's rule not known",
    ),
    (
        "edge-tie-m3d4",
        "the solid hangs on the membrane like on a hinge: its rotation is a mechanism only the \
         tiny out-of-plane stiffness of the membranes fixes, differently in ccx and plxs",
    ),
    (
        "membran-m3d4",
        "membranes: ccx's M3D4 is stiffer in its plane than the plane stress quadrilateral by \
         a term of the order (thickness / element size)² (1e-4 here)",
    ),
    (
        "membran-m3d4r",
        "membranes: M3D4R formulation of ccx 2.21 unknown",
    ),
    (
        "membran-s4r",
        "membranes: M3D4R formulation of ccx 2.21 unknown",
    ),
    (
        "rohr_b32r",
        "PIPE integration needs a section integration rule in the IR",
    ),
];

#[test]
fn structural_fixtures_match_calculix() {
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
    assert!(reports.len() > OPEN.len());
    for report in &reports {
        let name = report.fixture.rsplit('/').next().unwrap_or(&report.fixture);
        let open = OPEN.iter().any(|(n, _)| *n == name);
        assert!(
            report.outcome.passed() != open,
            "{}: {:?} (listed as open: {open})\n{}",
            report.fixture,
            report.outcome,
            report.log
        );
    }
}
