//! The T17 acceptance tests: the fixtures of the `sensitivity` area match their plxs
//! snapshots within their class tolerance, and the
//! strain energy sensitivity, where plxs deliberately differs from CalculiX, is the exact
//! derivative: moving a design node along its normal changes strain energy and mass in
//! the ratio of their sensitivities (the node area both are divided by cancels).

use std::path::Path;

use plxs_core::progress::NoProgress;
use plxs_core::results::Results;
use plxs_output::dat::DESIGN_RESPONSE_TABLE;
use plxs_session::{Session, SessionOptions};
use plxs_verify::report::format_reports;
use plxs_verify::{corpus, fixtures_root, run_corpus};

#[test]
fn sensitivity_fixtures_are_unchanged() {
    let fixtures = corpus(&fixtures_root()).expect("fixture corpus");
    let work = std::env::temp_dir().join(format!("plxs-sensitivity-corpus-{}", std::process::id()));
    let reports = run_corpus(
        Path::new(env!("CARGO_BIN_EXE_plxs")),
        &fixtures,
        &work,
        |f| f.meta.area == "sensitivity",
    );
    let _ = std::fs::remove_dir_all(&work);
    eprintln!("{}", format_reports(&reports));
    assert!(reports.len() >= 3);
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

/// Runs every step of an analysis in memory.
fn solve(analysis: &plxs_core::Analysis) -> Results {
    let mut session = Session::new(
        analysis.model.clone(),
        SessionOptions {
            threads: 2,
            work_dir: std::env::temp_dir(),
            ..Default::default()
        },
    )
    .expect("session");
    for step in &analysis.steps {
        session.run_step(step, &mut NoProgress).expect("step");
    }
    session.results().clone()
}

/// The value of a design response from the `.dat` table of the last step.
fn response(results: &Results, name: &str) -> f64 {
    let block = results
        .history
        .iter()
        .rev()
        .find(|b| b.quantity == DESIGN_RESPONSE_TABLE && b.set.split(',').any(|n| n == name))
        .expect("design response table");
    let row = block.set.split(',').position(|n| n == name).unwrap();
    block.rows[row].1[0]
}

/// A node field of the sensitivity step by name: node ids and the values of a component.
fn field(results: &Results, name: &str, component: usize) -> (Vec<u32>, Vec<f64>) {
    let f = results
        .increments
        .iter()
        .flat_map(|i| &i.fields)
        .find(|f| f.name == name)
        .expect(name);
    (f.nodes.clone(), f.values[component].clone())
}

#[test]
fn the_strain_energy_sensitivity_is_the_exact_derivative() {
    let path = fixtures_root().join("sensitivity/sensitivitaet.inp");
    let analysis = plxs_input::read_inp(&path).expect("fixture").analysis;
    let results = solve(&analysis);
    let (ids, energy) = field(&results, "SENENER", 0);
    let (_, mass) = field(&results, "SENMASS", 0);
    let normal: Vec<Vec<f64>> = (0..3).map(|k| field(&results, "NORM", k).1).collect();

    // The nodes with the largest strain energy sensitivities.
    let mut order: Vec<usize> = (0..ids.len()).filter(|&i| mass[i] != 0.0).collect();
    order.sort_by(|&a, &b| energy[b].abs().total_cmp(&energy[a].abs()));
    let largest = energy[order[0]].abs();
    for &i in order.iter().take(5) {
        let n = [normal[0][i], normal[1][i], normal[2][i]];
        let h = 1e-3;
        let moved = |s: f64| {
            let mut a = analysis.clone();
            let index = a.model.nodes.index(ids[i]).unwrap() as usize;
            for (k, nk) in n.iter().enumerate() {
                a.model.nodes.coordinates[index][k] += s * nk;
            }
            let r = solve(&a);
            (response(&r, "STRAINENERGY"), response(&r, "MASS"))
        };
        let (e_plus, m_plus) = moved(h);
        let (e_minus, m_minus) = moved(-h);
        let ratio_fd = (e_plus - e_minus) / (m_plus - m_minus);
        let ratio = energy[i] / mass[i];
        assert!(
            (ratio_fd - ratio).abs() <= 1e-3 * (largest / mass[i]).abs(),
            "node {}: finite differences {ratio_fd:e}, sensitivities {ratio:e}",
            ids[i]
        );
    }
}
