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
    let mut import = crate::dat::parse_dat(&text);
    if plxs_core::results::plane::is_plane(&solved.model) {
        for set in &mut import.sets {
            drop_out_of_plane(set);
        }
    }
    Some(import)
}

/// Leaves the history components out that are zero out of the plane of a 2D model, also
/// those the solver keeps because they are not at the end of their rows (the thickness
/// stress of plane stress, the thickness strain of plane strain).
fn drop_out_of_plane(set: &mut plx_results::history_output::HistorySet) {
    const OUT_OF_PLANE: &[&str] = &[
        "U3", "V3", "RF3", "Q3", "S33", "S13", "S23", "E33", "E13", "E23",
    ];
    for field in &mut set.fields {
        field.components.retain(|c| {
            !OUT_OF_PLANE.contains(&c.name.as_str())
                || c.entries.iter().any(|e| e.values.iter().any(|v| *v != 0.0))
        });
    }
    set.fields.retain(|f| !f.components.is_empty());
}

/// Writes the results as CalculiX's files: `frd` and, with history output, the `.dat` file
/// beside it.
pub fn export_calculix(solved: &SolverResults, frd: &Path) -> std::io::Result<()> {
    let error = |e: plxs_core::Error| std::io::Error::other(e.to_string());
    // CalculiX's files have every component, also those a 2D model leaves out.
    let mut full = std::borrow::Cow::Borrowed(&solved.results);
    if solved.results.has_out_of_plane_dropped() {
        full.to_mut().restore_out_of_plane();
    }
    plxs_output::write_frd(frd, &solved.model, &full, &solved.version).map_err(error)?;
    let dat = frd.with_extension("dat");
    if solved.results.history.is_empty() {
        if dat.is_file() {
            std::fs::remove_file(&dat)?;
        }
    } else {
        plxs_output::write_dat(&dat, &full).map_err(error)?;
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
            ..plxs_session::JobOptions::new(dir, name.clone())
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
        let areas = [
            "static",
            "structural",
            "plane",
            "constraints",
            "heat",
            "loads",
            "eigen",
            "nonlinear",
            "contact",
            "dynamic",
            "modal",
            "cyclic",
            "sensitivity",
        ];
        // The fixtures are solved side by side (each solve takes one thread), as many as
        // there are cores; a directory per area keeps fixtures of the same name apart.
        let base = &dir;
        let inputs: Vec<(PathBuf, PathBuf)> = (areas.iter())
            .flat_map(|&a| fixtures(a).into_iter().map(move |f| (f, base.join(a))))
            .collect();
        let next = std::sync::atomic::AtomicUsize::new(0);
        let workers = std::thread::available_parallelism().map_or(1, |n| n.get());
        let compared: usize = std::thread::scope(|scope| {
            let handles: Vec<_> = (0..workers)
                .map(|_| {
                    scope.spawn(|| {
                        let mut compared = 0;
                        loop {
                            let i = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                            let Some((input, dir)) = inputs.get(i) else {
                                break;
                            };
                            std::fs::create_dir_all(dir).unwrap();
                            compared += usize::from(compare(input, dir));
                        }
                        compared
                    })
                })
                .collect();
            handles.into_iter().map(|h| h.join().unwrap()).sum()
        });
        let _ = std::fs::remove_dir_all(&dir);
        assert!(compared > 50, "only {compared} fixtures compared");
    }

    /// Solves `input` into `dir` and compares the results from memory with the files;
    /// `false` if the solver does not compute it.
    fn compare(input: &Path, dir: &Path) -> bool {
        let Some((solved, frd)) = solve(input, dir) else {
            return false;
        };
        let mut out_of_plane = 0;
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
                    // Out of the plane of a 2D model: zero, the file's round-off at most.
                    if ca.out_of_plane {
                        let largest = cb.range().map_or(0.0, |(l, h)| l.abs().max(h.abs()));
                        assert!(largest <= 1e-9 * field_scale, "{what} {}", ca.name);
                        out_of_plane += 1;
                        continue;
                    }
                    // The signed largest principal value takes the sign of whichever of
                    // two opposite principal values of (nearly) equal size the rounding
                    // makes larger.
                    let signed = ca.name.starts_with("SGN_");
                    for (&va, &vb) in ca.values.iter().zip(&cb.values) {
                        let (va, vb) = if signed {
                            (va.abs(), vb.abs())
                        } else {
                            (va, vb)
                        };
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
        // A 2D model leaves its third displacements out (each field at most once here).
        let displaced = memory.increments.iter().any(|i| i.field("DISP").is_some());
        if plxs_core::results::plane::is_plane(&solved.model) && displaced {
            assert!(out_of_plane > 0, "{what}");
        }
        let dat = frd.with_extension("dat");
        let from_file = crate::dat::read_dat(&dat).ok();
        let input_text = std::fs::read_to_string(input).ok();
        let from_memory = history(&solved, input_text.as_deref());
        assert_eq!(
            from_memory.as_ref().map(|d| d.sets.len()),
            from_file.as_ref().map(|d| d.sets.len()),
            "{what}"
        );
        true
    }

    /// The history output of a 2D model leaves the zero out-of-plane values out: from
    /// memory without them, in the exported `.dat` file in full as CalculiX prints it.
    #[test]
    fn history_of_a_plane_model_leaves_out_of_plane_values_out() {
        let dir = std::env::temp_dir().join(format!("plx-io-plane-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let fixture = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../solver/fixtures/plane/ebene-cpe4.inp");
        let text = std::fs::read_to_string(fixture).unwrap().replace(
            "*End step",
            "*Node print, Nset=Internal_Selection-1_Displacement_Rotation-1\nU, RF\n\
             *El print, Elset=Internal_Selection-1_Section-1\nS, E\n*End step",
        );
        let input = dir.join("plane.inp");
        std::fs::write(&input, text).unwrap();
        let (solved, _) = solve(&input, &dir).unwrap();
        let import = history(&solved, None).unwrap();
        let set_of = |import: &'_ DatImport, name: &str| {
            let set = import.sets.iter().find(|s| s.name.contains(name));
            set.cloned().unwrap()
        };
        let set = set_of(&import, "DISPLACEMENT");
        let names = |field: &str| -> Vec<String> {
            let field = set.fields.iter().find(|f| f.name == field).unwrap();
            field.components.iter().map(|c| c.name.clone()).collect()
        };
        assert_eq!(names("DISPLACEMENTS"), ["U1", "U2"]);
        assert_eq!(names("FORCES"), ["RF1", "RF2"]);
        let set = set_of(&import, "SECTION");
        let stresses = set.fields.iter().find(|f| f.name == "STRESSES").unwrap();
        let stresses: Vec<&str> = stresses
            .components
            .iter()
            .map(|c| c.name.as_str())
            .collect();
        // Plane strain: no transverse shear, no thickness stress left out (it is not zero).
        assert!(!stresses.contains(&"S13") && !stresses.contains(&"S23"));
        assert!(stresses.contains(&"S33"));
        assert!(stresses.contains(&"S12") && stresses.contains(&"MISES"));
        // The files have every value.
        let frd = dir.join("exported.frd");
        export_calculix(&solved, &frd).unwrap();
        let file = crate::dat::read_dat(&frd.with_extension("dat")).unwrap();
        let displacements = (set_of(&file, "DISPLACEMENT").fields.into_iter())
            .find(|f| f.name == "DISPLACEMENTS")
            .unwrap();
        assert_eq!(displacements.components.len(), 3);
        assert!(file.warnings.is_empty(), "{:?}", file.warnings);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Plane stress carries no stress across the thickness: `S33`, `S13`, `S23` are zero at
    /// every integration point and left out of the fields and the history output; the
    /// thickness strain `E33` stays.
    #[test]
    fn a_plane_stress_model_leaves_its_thickness_stress_out() {
        let dir = std::env::temp_dir().join(format!("plx-io-cps-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let fixture = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../solver/fixtures/plane/ebene-cps4.inp");
        let text = std::fs::read_to_string(fixture).unwrap().replace(
            "*End step",
            "*El print, Elset=Internal_Selection-1_Section-1\nS, E\n*End step",
        );
        let input = dir.join("cps.inp");
        std::fs::write(&input, text).unwrap();
        let (solved, _) = solve(&input, &dir).unwrap();
        let components = |name: &str| -> Vec<String> {
            let increment = solved.results.increments.last().unwrap();
            let field = increment.fields.iter().find(|f| f.name == name).unwrap();
            field.components.clone()
        };
        assert_eq!(components("STRESS"), ["SXX", "SYY", "SXY"]);
        assert_eq!(components("TOSTRAIN"), ["EXX", "EYY", "EZZ", "EXY"]);
        let import = history(&solved, None).unwrap();
        let set = import
            .sets
            .iter()
            .find(|s| s.name.contains("SECTION"))
            .unwrap();
        let names = |field: &str| -> Vec<String> {
            let field = set.fields.iter().find(|f| f.name == field).unwrap();
            field.components.iter().map(|c| c.name.clone()).collect()
        };
        let stresses = names("STRESSES");
        for c in ["S33", "S13", "S23"] {
            assert!(!stresses.iter().any(|s| s == c), "{stresses:?}");
        }
        assert!(stresses.iter().any(|s| s == "S12"));
        assert!(names("STRAINS").iter().any(|s| s == "E33"));
        let _ = std::fs::remove_dir_all(&dir);
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
