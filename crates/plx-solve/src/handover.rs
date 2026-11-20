//! What a run takes from the results of other analyses, given to the solver as values: the
//! global results of a submodel and the temperatures of a defined field are read here from
//! the files the project made of those results, so that the run needs no copy of them next
//! to its input. The eigenmodes of another run stay a file the solver reads from where it
//! is. Only an exported input file names files beside it.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use plxs_core::handover::{NodalTemperatures, Source};
use plxs_core::step::DefinedFieldKind;

/// Makes the sources of `analysis` that name a file, by its name as the input file would,
/// values read from the file of that name among `files`, or, for the eigenmodes, that file
/// itself. A name none of `files` has is left for the solver to find in its directory.
pub fn load(analysis: &mut plxs_core::Analysis, files: &[PathBuf]) -> Result<(), String> {
    let find = |name: &Path| {
        (files.iter())
            .find(|f| f.file_name().is_some_and(|n| Some(n) == name.file_name()))
            .cloned()
    };
    let read =
        |path: &Path| std::fs::read(path).map_err(|e| format!("{} not read: {e}", path.display()));
    if let Some(submodel) = &mut analysis.model.constraints.submodel
        && let Source::Path(name) = &submodel.global
        && let Some(path) = find(name)
    {
        let text = read(&path)?;
        let global = plxs_procedures::submodel::read_global_frd(&String::from_utf8_lossy(&text))
            .map_err(|e| format!("Global results {}: {e}", path.display()))?;
        submodel.global = Source::loaded(global);
    }
    if let Some(Source::Path(name)) = &analysis.model.stored_modes
        && let Some(path) = find(name)
    {
        analysis.model.stored_modes = Some(Source::Path(path));
    }
    // Each file is read once, for all the steps of it the fields take.
    let mut texts: BTreeMap<PathBuf, Vec<u8>> = BTreeMap::new();
    for step in &mut analysis.steps {
        for field in &mut step.fields {
            let DefinedFieldKind::TemperatureFrom { source, bstep, .. } = &mut field.kind else {
                continue;
            };
            let Source::Path(name) = source else {
                continue;
            };
            let Some(path) = find(name) else {
                continue;
            };
            if !texts.contains_key(&path) {
                texts.insert(path.clone(), read(&path)?);
            }
            let number = bstep.unwrap_or(1);
            let values = plxs_procedures::thermal::frd_temperatures(&texts[&path], number)
                .ok_or_else(|| {
                    format!(
                        "{} has no temperatures (NDTEMP) of step {number}",
                        path.display()
                    )
                })?;
            let steps = BTreeMap::from([(number, values.into_iter().collect())]);
            *source = Source::loaded(NodalTemperatures { steps });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use plxs_core::step::{DefinedField, Procedure, Step};

    #[test]
    fn temperatures_of_a_file_go_to_the_solver_as_values() {
        let dir = std::env::temp_dir().join(format!("plx-handover-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("Heat.frd");
        std::fs::write(
            &file,
            "    1PSTEP                         1           1           1
 -4  NDTEMP      1    1
 -5  T           1    1    0    0
 -1         1 1.00000E+01
 -1         2 2.00000E+01
 -3
",
        )
        .unwrap();
        let mut analysis = plxs_core::Analysis::default();
        let mut step = Step::new("Step-1", Procedure::NoAnalysis);
        step.fields.push(DefinedField {
            kind: DefinedFieldKind::TemperatureFrom {
                source: Source::Path("Heat.frd".into()),
                bstep: Some(1),
                estep: None,
                binc: None,
                einc: None,
            },
            amplitude: None,
        });
        analysis.steps.push(step);
        // A name no file has is left to the solver.
        load(&mut analysis, &[dir.join("Other.frd")]).unwrap();
        let source = |analysis: &plxs_core::Analysis| match &analysis.steps[0].fields[0].kind {
            DefinedFieldKind::TemperatureFrom { source, .. } => source.clone(),
            _ => unreachable!(),
        };
        assert_eq!(source(&analysis), Source::Path("Heat.frd".into()));
        load(&mut analysis, &[file]).unwrap();
        let expected = NodalTemperatures {
            steps: BTreeMap::from([(1, BTreeMap::from([(1, 10.0), (2, 20.0)]))]),
        };
        assert_eq!(source(&analysis), Source::loaded(expected));
        let _ = std::fs::remove_dir_all(dir);
    }
}
