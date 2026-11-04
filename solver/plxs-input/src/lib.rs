//! Reader for CalculiX input files (`.inp`) into the solver's IR ([`plxs_core::Analysis`]).
//!
//! Scope: every keyword Nodalis's writer emits (concept section 1), spelled as CalculiX
//! documents it. Keywords, parameters and set names are case-insensitive and stored in
//! upper case; data lines may continue on the next line after a trailing comma; `*INCLUDE`
//! pulls in files relative to the including file; comment lines (`**`) are skipped. The
//! carry-over rules of CalculiX between steps (`OP=NEW` and `OP=MOD` of boundary conditions
//! and loads, output requests kept from the previous step) are resolved here, so that every
//! [`plxs_core::Step`] is complete on its own.
//!
//! Errors use CalculiX's wording (`*ERROR reading *BOUNDARY: ...`, `cannot be interpreted`
//! for unknown keywords), because Nodalis's solver self-test reads the text.
//!
//! Source: the keyword chapter of the CalculiX CrunchiX user's manual (2.23). No CalculiX
//! source code was used.

mod carry;
mod lexer;
mod reader;
mod render;
pub mod steps;
#[cfg(test)]
mod tests;

use std::path::Path;

use plxs_core::{Analysis, Error};

pub use lexer::{Keyword, Line, split_fields};
pub use render::{render, render_with_parts, unwritable_names};

/// What reading an input file produces: the analysis and what the reader noticed but did
/// not stop for (`*WARNING` lines on the command line).
#[derive(Debug, Default)]
pub struct Reading {
    pub analysis: Analysis,
    pub warnings: Vec<String>,
}

/// Reads and validates an input file. Included files are resolved relative to it. The first
/// syntax error stops the reading (as CalculiX does); validation reports every missing
/// reference at once.
pub fn read_inp(path: &Path) -> Result<Reading, Vec<Error>> {
    let text = std::fs::read_to_string(path).map_err(|e| {
        vec![Error::Io {
            path: path.display().to_string(),
            message: e.to_string(),
        }]
    })?;
    read_inp_str(&text, Some(path))
}

/// Reads the text of an input file; `path` is the file it came from, used for the location
/// of errors and to find included files (without it `*INCLUDE` resolves relative to the
/// working directory).
pub fn read_inp_str(text: &str, path: Option<&Path>) -> Result<Reading, Vec<Error>> {
    read_with(reader::Reader::new(), text, path)
}

/// Reads the input of a run that continues `base` (`*RESTART, READ`): `base` is the model
/// and the steps up to the restart, read from the restart file; the text holds the steps
/// after them (and may add sets). The steps are numbered on from `base`'s and start with
/// what its last step carries over, so the analysis is the same as one input with all
/// steps. The returned analysis holds `base`'s steps and the new ones.
pub fn read_inp_continuing(
    text: &str,
    path: Option<&Path>,
    base: Analysis,
) -> Result<Reading, Vec<Error>> {
    read_with(reader::Reader::continuing(base), text, path)
}

/// `*RESTART, READ` of an input text, found without reading the model: the run then needs
/// its model from the restart file. `Some(None)` is a `*RESTART, READ` without `STEP=`.
pub fn restart_read(text: &str) -> Option<Option<u32>> {
    for line in text.lines() {
        let line = line.trim_start();
        if !line.starts_with('*') || line.starts_with("**") {
            continue;
        }
        let compact: String = line
            .chars()
            .filter(|c| !c.is_whitespace())
            .collect::<String>()
            .to_ascii_uppercase();
        let mut parts = compact.split(',');
        match parts.next() {
            Some("*RESTART") => {}
            Some("*STEP") => return None,
            _ => continue,
        }
        let mut read = false;
        let mut step = None;
        for part in parts {
            if part == "READ" {
                read = true;
            } else if let Some(value) = part.strip_prefix("STEP=") {
                step = value.parse().ok();
            }
        }
        if read {
            return Some(step);
        }
    }
    None
}

fn read_with(
    reader: reader::Reader,
    text: &str,
    path: Option<&Path>,
) -> Result<Reading, Vec<Error>> {
    let file = path.map(|p| p.display().to_string()).unwrap_or_default();
    let base = path.and_then(|p| p.parent()).map(Path::to_path_buf);
    let lines = lexer::lex(text, &file, base.as_deref()).map_err(|e| vec![e])?;
    let mut reading = reader.read(&lines).map_err(|e| vec![e])?;
    // `<job>.eig` next to the input: modes of an earlier run, as CalculiX reads them.
    if let Some(eig) = path
        .map(|p| p.with_extension("eig"))
        .filter(|p| p.is_file())
    {
        reading.analysis.model.stored_modes_file = Some(eig);
    }
    let warnings = plxs_core::validate(&mut reading.analysis)?;
    reading.warnings.extend(warnings);
    Ok(reading)
}
