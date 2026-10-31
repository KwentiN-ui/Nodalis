//! The fixture corpus on disk.

use std::io::Read;
use std::path::{Path, PathBuf};

use plx_io::dat::{DatImport, parse_dat, restore_shortened_names};
use plx_io::frd::{FrdImport, read_frd_bytes};

use crate::Meta;

/// `solver/fixtures` of this checkout.
pub fn fixtures_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures")
}

#[derive(Clone, Debug)]
pub struct Fixture {
    pub meta: Meta,
    /// `<area>/<name>` without extension.
    pub path: PathBuf,
}

impl Fixture {
    pub fn inp(&self) -> PathBuf {
        self.path.with_extension("inp")
    }

    pub fn toml(&self) -> PathBuf {
        self.path.with_extension("toml")
    }

    pub fn ref_frd(&self) -> PathBuf {
        self.path.with_extension("ref.frd.gz")
    }

    pub fn ref_dat(&self) -> PathBuf {
        self.path.with_extension("ref.dat")
    }

    /// Files the input reads besides itself (the global results of a submodel), gzipped
    /// as `<file>.gz` in the directory `<name>.files/`; [`stage_files`] puts them into a
    /// job's directory.
    pub fn files_dir(&self) -> PathBuf {
        self.path.with_extension("files")
    }

    /// `area/name`.
    pub fn id(&self) -> String {
        format!("{}/{}", self.meta.area, self.meta.name)
    }

    pub fn has_reference(&self) -> bool {
        self.meta.reference != "none" && self.ref_frd().is_file()
    }
}

/// Every fixture under `root`, by area and name.
pub fn corpus(root: &Path) -> std::io::Result<Vec<Fixture>> {
    let mut fixtures = Vec::new();
    let mut areas: Vec<PathBuf> = std::fs::read_dir(root)?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.is_dir())
        .collect();
    areas.sort();
    for area in areas {
        let mut files: Vec<PathBuf> = std::fs::read_dir(&area)?
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().is_some_and(|e| e == "toml"))
            .collect();
        files.sort();
        for toml_path in files {
            let text = std::fs::read_to_string(&toml_path)?;
            let meta: Meta = toml::from_str(&text)
                .map_err(|e| std::io::Error::other(format!("{}: {e}", toml_path.display())))?;
            fixtures.push(Fixture {
                meta,
                path: toml_path.with_extension(""),
            });
        }
    }
    Ok(fixtures)
}

/// The parsed results of a run: `.frd` and, when present, `.dat`.
#[derive(Debug)]
pub struct FixtureResults {
    pub frd: FrdImport,
    pub dat: Option<DatImport>,
}

/// Loads the golden files of a fixture.
pub fn load_reference(fixture: &Fixture) -> Result<FixtureResults, String> {
    let gz = std::fs::read(fixture.ref_frd())
        .map_err(|e| format!("{}: {e}", fixture.ref_frd().display()))?;
    let mut bytes = Vec::new();
    flate2::read::GzDecoder::new(gz.as_slice())
        .read_to_end(&mut bytes)
        .map_err(|e| format!("{}: {e}", fixture.ref_frd().display()))?;
    let frd =
        read_frd_bytes(&bytes).map_err(|e| format!("{}: {e}", fixture.ref_frd().display()))?;
    let dat = match std::fs::read_to_string(fixture.ref_dat()) {
        Ok(text) => Some(parse_dat_with_input(&text, &fixture.inp())),
        Err(_) => None,
    };
    Ok(FixtureResults { frd, dat })
}

/// Parses a `.dat` text with the long names the input file holds for shortened set names.
pub fn parse_dat_with_input(text: &str, input: &Path) -> DatImport {
    match std::fs::read_to_string(input) {
        Ok(inp) => parse_dat(&restore_shortened_names(text, &inp)),
        Err(_) => parse_dat(text),
    }
}

/// Reads the results a solver left in `dir` for job `name`.
pub fn load_results(dir: &Path, name: &str, input: &Path) -> Result<FixtureResults, String> {
    let frd_path = dir.join(format!("{name}.frd"));
    let frd = plx_io::frd::read_frd(&frd_path).map_err(|e| e.to_string())?;
    let dat = std::fs::read_to_string(dir.join(format!("{name}.dat")))
        .ok()
        .filter(|t| !t.trim().is_empty())
        .map(|t| parse_dat_with_input(&t, input));
    Ok(FixtureResults { frd, dat })
}

/// Copies the files of a fixture's `<name>.files/` directory into `work`, unzipping those
/// that end in `.gz`.
pub fn stage_files(fixture: &Fixture, work: &Path) -> Result<(), String> {
    let dir = fixture.files_dir();
    let Ok(entries) = std::fs::read_dir(&dir) else {
        return Ok(());
    };
    for entry in entries.filter_map(|e| e.ok()) {
        let path = entry.path();
        let name = entry.file_name().to_string_lossy().into_owned();
        let bytes = std::fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        let (target, bytes) = match name.strip_suffix(".gz") {
            Some(stem) => {
                let mut out = Vec::new();
                flate2::read::GzDecoder::new(bytes.as_slice())
                    .read_to_end(&mut out)
                    .map_err(|e| format!("{}: {e}", path.display()))?;
                (stem.to_string(), out)
            }
            None => (name, bytes),
        };
        std::fs::write(work.join(&target), bytes).map_err(|e| format!("{target}: {e}"))?;
    }
    Ok(())
}

/// Gzips bytes for a golden file.
pub fn gzip(bytes: &[u8]) -> std::io::Result<Vec<u8>> {
    use std::io::Write;
    let mut encoder = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::best());
    encoder.write_all(bytes)?;
    encoder.finish()
}
