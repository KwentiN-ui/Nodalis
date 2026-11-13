//! The `plxs` command line: reads fixtures, reports in CalculiX's wording, computes
//! `*NO ANALYSIS`, linear static and eigenvalue steps and refuses every other procedure.

use std::path::PathBuf;
use std::process::Command;

use plxs_verify::{corpus, fixtures_root, run_fixture};

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

#[test]
fn the_version_line_names_calculix_for_the_self_test() {
    let output = Command::new(plxs()).arg("-v").output().unwrap();
    let text = String::from_utf8_lossy(&output.stdout);
    assert!(output.status.success());
    assert!(text.contains("This is Version "), "{text}");
    assert!(text.contains("CalculiX"), "{text}");
}

#[test]
fn a_static_fixture_is_computed_and_other_procedures_are_refused() {
    let fixtures = corpus(&fixtures_root()).unwrap();
    let work = std::env::temp_dir().join(format!("plxs-cli-test-{}", std::process::id()));
    let f = fixtures
        .iter()
        .find(|f| f.meta.name == "kragbalken")
        .unwrap();
    let report = run_fixture(&plxs(), f, &work);
    assert!(
        report.outcome.passed(),
        "{:?}\n{}",
        report.outcome,
        report.log
    );
    assert!(report.log.contains("This is Version"), "{}", report.log);
    // A procedure plxs does not compute (yet) on the same model.
    let text = std::fs::read_to_string(f.inp()).unwrap();
    let dynamic = text
        .replacen("*Static", "*Coupled temperature-displacement", 1)
        .replacen("*Step", "*Step, Nlgeom", 1);
    assert_ne!(dynamic, text);
    std::fs::write(work.join("dynamik.inp"), dynamic).unwrap();
    let output = Command::new(plxs())
        .args(["-i", "dynamik"])
        .current_dir(&work)
        .output()
        .unwrap();
    let text = String::from_utf8_lossy(&output.stdout);
    assert!(text.contains("procedure not implemented yet"), "{text}");
    let _ = std::fs::remove_dir_all(&work);
}

#[test]
fn a_model_check_finishes_and_an_unknown_keyword_cannot_be_interpreted() {
    let work = std::env::temp_dir().join(format!("plxs-cli-check-{}", std::process::id()));
    std::fs::create_dir_all(&work).unwrap();
    let model = "*Node\n1, 0, 0, 0\n2, 1, 0, 0\n3, 0, 1, 0\n4, 0, 0, 1\n*Element, Type=C3D4, Elset=E\n1, 1, 2, 3, 4\n\
*Material, Name=M\n*Elastic\n1, 0.3\n*Solid section, Elset=E, Material=M\n";
    std::fs::write(
        work.join("check.inp"),
        format!("{model}*Step\n*No analysis\n*End step\n"),
    )
    .unwrap();
    let output = Command::new(plxs())
        .args(["-i", "check"])
        .current_dir(&work)
        .output()
        .unwrap();
    let text = String::from_utf8_lossy(&output.stdout);
    assert!(output.status.success(), "{text}");
    assert!(text.contains("Job finished"), "{text}");
    assert!(!text.contains("*ERROR"), "{text}");

    std::fs::write(
        work.join("bad.inp"),
        format!("{model}*Step\n*Nonsense\n*End step\n"),
    )
    .unwrap();
    let output = Command::new(plxs())
        .args(["-i", "bad"])
        .current_dir(&work)
        .output()
        .unwrap();
    let text = String::from_utf8_lossy(&output.stdout);
    assert_eq!(output.status.code(), Some(1));
    assert!(text.contains("cannot be interpreted"), "{text}");
    let _ = std::fs::remove_dir_all(&work);
}

#[test]
fn the_profile_names_the_phases_and_the_reused_analyses() {
    let work = std::env::temp_dir().join(format!("plxs-cli-profile-{}", std::process::id()));
    std::fs::create_dir_all(&work).unwrap();
    let source = fixtures_root().join("nonlinear").join("plastisch.inp");
    std::fs::copy(source, work.join("p.inp")).unwrap();
    let output = Command::new(plxs())
        .args(["--profile", "p"])
        // Each tangent factorized anew: a kept factor would leave nothing to reuse.
        .env("PLXS_KEEP_FACTOR", "0")
        .current_dir(&work)
        .output()
        .unwrap();
    let text = String::from_utf8_lossy(&output.stdout);
    assert!(output.status.success(), "{text}");
    for phase in ["assembly", "factorization", "symbolic analysis", "total"] {
        assert!(text.contains(phase), "{text}");
    }
    // Newton iterations on one pattern analyze it once.
    assert!(text.contains(" reused)"), "{text}");
    let _ = std::fs::remove_dir_all(&work);
}
