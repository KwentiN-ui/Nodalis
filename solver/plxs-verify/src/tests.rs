//! The corpus checks itself: every fixture is complete, parses, and its golden files decode.

use std::path::PathBuf;

use crate::{Class, compare_dat, compare_frd, corpus, fixtures_root, load_reference};

fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

#[test]
fn every_fixture_is_complete_and_within_the_size_budget() {
    let fixtures = corpus(&fixtures_root()).unwrap();
    assert!(fixtures.len() >= 150, "{} fixtures", fixtures.len());
    let mut total = 0;
    for f in &fixtures {
        assert!(f.inp().is_file(), "{}", f.id());
        assert_eq!(
            f.meta.name,
            f.path.file_name().unwrap().to_string_lossy(),
            "{}",
            f.id()
        );
        assert_eq!(
            f.meta.area,
            f.path
                .parent()
                .unwrap()
                .file_name()
                .unwrap()
                .to_string_lossy()
        );
        assert!(!f.meta.features.is_empty(), "{}", f.id());
        if f.meta.reference == "none" {
            assert!(
                f.meta.note.is_some(),
                "{} has no reference and no note why",
                f.id()
            );
            assert!(!f.ref_frd().exists(), "{}", f.id());
        } else {
            assert!(f.ref_frd().is_file(), "{} lacks its golden .frd", f.id());
        }
        for path in [f.inp(), f.toml(), f.ref_frd(), f.ref_dat()] {
            total += std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0);
        }
    }
    assert!(total < 20 << 20, "the corpus takes {} MB", total >> 20);
}

#[test]
fn every_fixture_reads_into_the_ir() {
    for f in corpus(&fixtures_root()).unwrap() {
        let reading =
            plxs_input::read_inp(&f.inp()).unwrap_or_else(|e| panic!("{}: {e:?}", f.id()));
        assert!(!reading.analysis.steps.is_empty(), "{}", f.id());
    }
}

#[test]
fn every_golden_file_decodes_and_matches_itself() {
    let mut with_dat = 0;
    for f in corpus(&fixtures_root())
        .unwrap()
        .iter()
        .filter(|f| f.has_reference())
    {
        let reference = load_reference(f).unwrap_or_else(|e| panic!("{}: {e}", f.id()));
        assert!(
            !reference.frd.increments.is_empty(),
            "{}: no results",
            f.id()
        );
        assert!(reference.frd.mesh.node_count() > 0, "{}", f.id());
        let comparison = compare_frd(&reference.frd, &reference.frd, Class::A);
        assert!(comparison.ok(), "{}: {:?}", f.id(), comparison.failures);
        assert!(comparison.compared > 0);
        if let Some(dat) = &reference.dat {
            with_dat += 1;
            let comparison = compare_dat(dat, dat, Class::A);
            assert!(comparison.ok(), "{}: {:?}", f.id(), comparison.failures);
        }
    }
    assert!(with_dat > 5, "{with_dat} fixtures with history output");
}

#[test]
fn a_changed_value_fails_the_comparison() {
    let fixtures = corpus(&fixtures_root()).unwrap();
    let f = fixtures
        .iter()
        .find(|f| f.meta.name == "kragbalken")
        .unwrap();
    let reference = load_reference(f).unwrap();
    let mut changed = load_reference(f).unwrap();
    let field = &mut changed.frd.increments[0].fields[0];
    // Deviations are relative to the field norm (all components).
    let scale = field
        .components
        .iter()
        .filter(|c| !c.derived)
        .flat_map(|c| c.values.iter())
        .map(|v| v.abs())
        .fold(0.0_f32, f32::max);
    let component = field.components.iter_mut().find(|c| !c.derived).unwrap();
    component.values[0] += 1e-4 * scale;
    let comparison = compare_frd(&reference.frd, &changed.frd, Class::A);
    assert_eq!(comparison.failures.len(), 1, "{:?}", comparison.failures);
    assert!(compare_frd(&reference.frd, &changed.frd, Class::B).ok());
    // A missing element is a structural failure whatever the class.
    let first = changed.frd.mesh.elements()[0].id;
    changed
        .frd
        .mesh
        .remove_elements(&std::collections::BTreeSet::from([first]));
    assert!(!compare_frd(&reference.frd, &changed.frd, Class::C).ok());
}

/// The feature matrix in the documentation is generated from the corpus and must be current.
#[test]
fn the_feature_matrix_in_the_docs_is_current() {
    let fixtures = corpus(&fixtures_root()).unwrap();
    let expected = crate::report::feature_matrix(&fixtures);
    let doc = std::fs::read_to_string(repo().join("docs/solver/FIXTURES.md")).unwrap();
    assert!(
        doc.contains(expected.trim()),
        "docs/solver/FIXTURES.md is stale: run `cargo run -p plxs-verify --bin plxs-fixtures -- matrix` and paste the table"
    );
}
