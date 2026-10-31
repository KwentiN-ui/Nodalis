//! Maintenance of the fixture corpus (`solver/fixtures`), see `scripts/solver_reference.sh`:
//!
//! ```text
//! plxs-fixtures import <dir> [--source <text>]   copy the .inp files of a directory into the
//!                                                corpus, classified by their keywords
//! plxs-fixtures reference [--ccx <exe>] [--all]  compute golden files with CalculiX for the
//!                                                fixtures without any (or all)
//! plxs-fixtures matrix                           print the feature matrix (docs/solver/FIXTURES.md)
//! plxs-fixtures run --solver <exe> [--area <a>] [--name <n>]
//!                                                run a solver over the corpus and compare
//! ```

use std::path::{Path, PathBuf};

use plxs_verify::{Meta, classify, corpus, fixtures_root};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.first().map(String::as_str) {
        Some("import") => import(&args[1..]),
        Some("reference") => reference(&args[1..]),
        Some("matrix") => {
            let fixtures = corpus(&fixtures_root()).map_err(|e| e.to_string());
            fixtures.map(|f| print!("{}", plxs_verify::report::feature_matrix(&f)))
        }
        Some("run") => run(&args[1..]),
        _ => Err("usage: plxs-fixtures import|reference|matrix|run ...".into()),
    };
    if let Err(e) = result {
        eprintln!("{e}");
        std::process::exit(1);
    }
}

fn option<'a>(args: &'a [String], name: &str) -> Option<&'a str> {
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1))
        .map(String::as_str)
}

fn write_meta(fixture_path: &Path, meta: &Meta) -> Result<(), String> {
    let text = toml::to_string(meta).map_err(|e| e.to_string())?;
    std::fs::write(fixture_path.with_extension("toml"), text).map_err(|e| e.to_string())
}

fn import(args: &[String]) -> Result<(), String> {
    let Some(dir) = args.first() else {
        return Err("import needs the directory with the .inp files".into());
    };
    let source = option(args, "--source").unwrap_or("plx-io writer test");
    let root = fixtures_root();
    let existing = corpus(&root).map_err(|e| e.to_string())?;
    let mut files: Vec<PathBuf> = std::fs::read_dir(dir)
        .map_err(|e| format!("{dir}: {e}"))?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e == "inp"))
        .collect();
    files.sort();
    let mut imported = 0;
    for file in files {
        let name = file
            .file_stem()
            .unwrap()
            .to_string_lossy()
            .to_ascii_lowercase()
            .replace(' ', "-");
        let text = std::fs::read_to_string(&file).map_err(|e| e.to_string())?;
        let mut meta = classify(&name, &text, source);
        // A fixture that exists keeps its place, reference and notes; only its features are
        // refreshed when the input changed.
        if let Some(old) = existing.iter().find(|f| f.meta.name == name) {
            let unchanged =
                std::fs::read_to_string(old.inp()).ok().as_deref() == Some(text.as_str());
            meta.area = old.meta.area.clone();
            meta.note = old.meta.note.clone();
            meta.deviations = old.meta.deviations.clone();
            if unchanged {
                meta.reference = old.meta.reference.clone();
            } else {
                let _ = std::fs::remove_file(old.ref_frd());
                let _ = std::fs::remove_file(old.ref_dat());
            }
            std::fs::write(old.inp(), &text).map_err(|e| e.to_string())?;
            write_meta(&old.path, &meta)?;
            if !unchanged {
                imported += 1;
            }
            continue;
        }
        let area_dir = root.join(&meta.area);
        std::fs::create_dir_all(&area_dir).map_err(|e| e.to_string())?;
        let path = area_dir.join(&name);
        std::fs::write(path.with_extension("inp"), &text).map_err(|e| e.to_string())?;
        write_meta(&path, &meta)?;
        imported += 1;
    }
    println!("{imported} fixtures imported or changed");
    Ok(())
}

fn reference(args: &[String]) -> Result<(), String> {
    let ccx = option(args, "--ccx").unwrap_or("ccx");
    let all = args.iter().any(|a| a == "--all");
    let only = option(args, "--name");
    let root = fixtures_root();
    let fixtures = corpus(&root).map_err(|e| e.to_string())?;
    let work = std::env::temp_dir().join(format!("plxs-reference-{}", std::process::id()));
    let mut made = 0;
    let mut failed = Vec::new();
    for fixture in &fixtures {
        if let Some(only) = only
            && fixture.meta.name != only
        {
            continue;
        }
        if !all && fixture.has_reference() {
            continue;
        }
        let dir = work.join(&fixture.meta.name);
        let mut meta = fixture.meta.clone();
        match plxs_verify::reference::make_reference(ccx, fixture, &dir) {
            Ok(version) => {
                meta.reference = version;
                meta.note = None;
                made += 1;
                println!("{}: reference made", fixture.id());
            }
            Err(e) => {
                meta.reference = "none".into();
                meta.note = Some(format!("CalculiX failed: {e}"));
                failed.push(format!("{}: {e}", fixture.id()));
            }
        }
        write_meta(&fixture.path, &meta)?;
        let _ = std::fs::remove_dir_all(&dir);
    }
    let _ = std::fs::remove_dir_all(&work);
    println!(
        "{made} golden files made, {} fixtures without",
        failed.len()
    );
    for f in failed {
        println!("  {f}");
    }
    Ok(())
}

fn run(args: &[String]) -> Result<(), String> {
    let Some(solver) = option(args, "--solver") else {
        return Err("run needs --solver <executable>".into());
    };
    let solver = std::fs::canonicalize(solver).map_err(|e| format!("{solver}: {e}"))?;
    let area = option(args, "--area");
    let name = option(args, "--name");
    let fixtures = corpus(&fixtures_root()).map_err(|e| e.to_string())?;
    let work = std::env::temp_dir().join(format!("plxs-run-{}", std::process::id()));
    let reports = plxs_verify::run_corpus(&solver, &fixtures, &work, |f| {
        area.is_none_or(|a| f.meta.area == a) && name.is_none_or(|n| f.meta.name == n)
    });
    print!("{}", plxs_verify::report::format_reports(&reports));
    for r in reports.iter().filter(|r| !r.outcome.passed()) {
        if let plxs_verify::Outcome::Fail(failures) = &r.outcome {
            println!("\n{}:", r.fixture);
            for f in failures {
                println!("  {f}");
            }
        }
    }
    let _ = std::fs::remove_dir_all(&work);
    Ok(())
}
