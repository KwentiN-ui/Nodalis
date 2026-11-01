//! Results of the built-in solver as the readers of CalculiX's files give them: fields from
//! memory ([`plx_results::from_solver`]), history output, and the `.frd`/`.dat` files for
//! the export and for what still takes CalculiX's files (the global results of a submodel).

use std::path::Path;

use plx_results::SolverResults;

use crate::dat::DatImport;
use crate::frd::FrdImport;

/// The mesh and increments of the results, as [`crate::frd::read_frd`] gives those of a
/// `.frd` file.
pub fn import(solved: &SolverResults) -> FrdImport {
    let import = plx_results::from_solver(solved);
    FrdImport {
        mesh: import.mesh,
        increments: import.increments,
        materials: import.materials,
        date: None,
        time: None,
        warnings: import.warnings,
    }
}

/// The history output of the results, as [`crate::dat::read_dat`] reads it; names `input`
/// shortened for CalculiX are given in full. `None` without history output.
pub fn history(solved: &SolverResults, input: Option<&str>) -> Option<DatImport> {
    if solved.results.history.is_empty() {
        return None;
    }
    // The blocks are named as the `.dat` file names them, so they take the same way.
    let mut text = Vec::new();
    plxs_output::dat::write_dat_to(&mut text, &solved.results).ok()?;
    let mut text = String::from_utf8_lossy(&text).into_owned();
    if let Some(input) = input {
        text = crate::dat::restore_shortened_names(&text, input);
    }
    Some(crate::dat::parse_dat(&text))
}

/// Writes the results as CalculiX's files: `frd` and, with history output, the `.dat` file
/// beside it.
pub fn export_calculix(solved: &SolverResults, frd: &Path) -> std::io::Result<()> {
    let error = |e: plxs_core::Error| std::io::Error::other(e.to_string());
    plxs_output::write_frd(frd, &solved.model, &solved.results, &solved.version).map_err(error)?;
    let dat = frd.with_extension("dat");
    if solved.results.history.is_empty() {
        if dat.is_file() {
            std::fs::remove_file(&dat)?;
        }
    } else {
        plxs_output::write_dat(&dat, &solved.results).map_err(error)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;
    use crate::frd::read_frd;

    fn fixtures(area: &str) -> Vec<PathBuf> {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../solver/fixtures")
            .join(area);
        let mut inputs: Vec<PathBuf> = std::fs::read_dir(dir)
            .unwrap()
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|e| e == "inp"))
            .collect();
        inputs.sort();
        inputs
    }

    /// Runs `input` with the built-in solver, writing its `.frd` and `.dat` files into `dir`
    /// and keeping its results; `None` if the solver does not compute it.
    fn solve(input: &Path, dir: &Path) -> Option<(SolverResults, PathBuf)> {
        let name = input.file_stem().unwrap().to_string_lossy().into_owned();
        let options = plxs_session::JobOptions {
            threads: 1,
            work_dir: dir.to_owned(),
            name: name.clone(),
            write_files: true,
            cancel: plxs_core::CancelToken::new(),
        };
        let text = std::fs::read_to_string(input).unwrap();
        let mut kept = None;
        let outcome =
            plxs_session::run_job_keeping(&text, Some(input), &options, &mut |_| {}, &mut kept);
        let (model, results) = kept.filter(|_| outcome.is_ok())?;
        let solved = SolverResults::new(&model, results, "test");
        Some((solved, dir.join(format!("{name}.frd"))))
    }

    fn close(a: f32, b: f32) -> bool {
        (a.is_nan() && b.is_nan()) || (a - b).abs() <= 2e-5 * a.abs().max(b.abs()).max(1e-6)
    }

    /// The results from memory are what reading the `.frd` and `.dat` files of the same run
    /// gives: mesh, parts, increments, fields, components and values (to the five digits
    /// of the file), history output; for solids, plane elements, expanded shells and beams,
    /// springs, heat transfer.
    #[test]
    fn results_from_memory_match_the_result_files() {
        let dir = std::env::temp_dir().join(format!("plx-io-solver-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut compared = 0;
        let areas = [
            "static",
            "structural",
            "plane",
            "constraints",
            "heat",
            "loads",
        ];
        for input in areas.iter().flat_map(|a| fixtures(a)) {
            let Some((solved, frd)) = solve(&input, &dir) else {
                continue;
            };
            let what = input.display();
            let file = read_frd(&frd).unwrap();
            let memory = import(&solved);
            assert_eq!(memory.mesh.node_ids(), file.mesh.node_ids(), "{what}");
            for (a, b) in memory.mesh.elements().iter().zip(file.mesh.elements()) {
                assert_eq!(
                    (a.id, &a.type_name, &a.nodes),
                    (b.id, &b.type_name, &b.nodes)
                );
            }
            assert_eq!(memory.mesh.parts, file.mesh.parts, "{what}");
            assert_eq!(memory.materials, file.materials, "{what}");
            for (a, b) in memory
                .mesh
                .node_ids()
                .iter()
                .map(|&id| (memory.mesh.node(id).unwrap(), file.mesh.node(id).unwrap()))
            {
                for k in 0..3 {
                    assert!((a[k] - b[k]).abs() <= 1e-5 * a[k].abs().max(1e-3), "{what}");
                }
            }
            assert_eq!(memory.increments.len(), file.increments.len(), "{what}");
            for (a, b) in memory.increments.iter().zip(&file.increments) {
                let head = |i: &plx_results::Increment| (i.step, i.increment, i.kind);
                assert_eq!(head(a), head(b), "{what}");
                assert!(
                    (a.value - b.value).abs() <= 1e-6 * b.value.abs().max(1.0),
                    "{what}"
                );
                let names = |i: &plx_results::Increment| -> Vec<(String, Vec<String>)> {
                    (i.fields.iter())
                        .map(|f| {
                            let c = f.components.iter().map(|c| c.name.clone()).collect();
                            (f.name.clone(), c)
                        })
                        .collect()
                };
                assert_eq!(names(a), names(b), "{what}");
                for (fa, fb) in a.fields.iter().zip(&b.fields) {
                    assert_eq!(fa.rank, fb.rank, "{what}");
                    let field_scale = (fb.components.iter().filter(|c| !c.derived))
                        .filter_map(|c| c.range())
                        .fold(0.0f32, |m, (l, h)| m.max(l.abs()).max(h.abs()));
                    for (ca, cb) in fa.components.iter().zip(&fb.components) {
                        // Values the file rounds to zero below its smallest exponent; derived
                        // values such as principal strains take the file's rounding further.
                        let scale = if cb.derived {
                            field_scale
                        } else {
                            cb.range().map_or(0.0, |(l, h)| l.abs().max(h.abs()))
                        };
                        let tolerance = if cb.derived { 1e-4 } else { 1e-5 };
                        assert_eq!(ca.derived, cb.derived, "{what}");
                        for (&va, &vb) in ca.values.iter().zip(&cb.values) {
                            assert!(
                                close(va, vb) || (va - vb).abs() <= tolerance * scale,
                                "{what} {} {}: {va} vs {vb}",
                                fa.name,
                                ca.name
                            );
                        }
                    }
                }
            }
            let dat = frd.with_extension("dat");
            let from_file = crate::dat::read_dat(&dat).ok();
            let input_text = std::fs::read_to_string(&input).ok();
            let from_memory = history(&solved, input_text.as_deref());
            assert_eq!(
                from_memory.as_ref().map(|d| d.sets.len()),
                from_file.as_ref().map(|d| d.sets.len()),
                "{what}"
            );
            compared += 1;
        }
        let _ = std::fs::remove_dir_all(&dir);
        assert!(compared > 50, "only {compared} fixtures compared");
    }

    #[test]
    fn exported_results_read_like_the_solver_files() {
        let dir = std::env::temp_dir().join(format!("plx-io-export-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let input = fixtures("static").into_iter().next().unwrap();
        let (solved, frd) = solve(&input, &dir).unwrap();
        let exported = dir.join("exported.frd");
        export_calculix(&solved, &exported).unwrap();
        let (a, b) = (read_frd(&exported).unwrap(), read_frd(&frd).unwrap());
        assert_eq!(a.mesh.node_ids(), b.mesh.node_ids());
        assert_eq!(a.increments, b.increments);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
