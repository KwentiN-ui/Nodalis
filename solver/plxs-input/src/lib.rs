//! Reader for CalculiX input files (`.inp`) into the solver's IR ([`plxs_core::Analysis`]).
//!
//! Scope: every keyword PrePoLix's writer emits (concept section 1), spelled as CalculiX
//! documents it. Keywords, parameters and set names are case-insensitive and stored in
//! upper case; data lines may continue on the next line after a trailing comma; `*INCLUDE`
//! pulls in files relative to the including file; comment lines (`**`) are skipped. The
//! carry-over rules of CalculiX between steps (`OP=NEW` and `OP=MOD` of boundary conditions
//! and loads, output requests kept from the previous step) are resolved here, so that every
//! [`plxs_core::Step`] is complete on its own.
//!
//! Errors use CalculiX's wording (`*ERROR reading *BOUNDARY: ...`, `cannot be interpreted`
//! for unknown keywords), because PrePoLix's solver self-test reads the text.
//!
//! Source: the keyword chapter of the CalculiX CrunchiX user's manual (2.23). No CalculiX
//! source code was used.

mod carry;
mod lexer;
mod reader;
#[cfg(test)]
mod tests;

use std::path::Path;

use plxs_core::{Analysis, Error};

pub use lexer::{Keyword, Line, split_fields};

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
    let file = path.map(|p| p.display().to_string()).unwrap_or_default();
    let base = path.and_then(|p| p.parent()).map(Path::to_path_buf);
    let lines = lexer::lex(text, &file, base.as_deref()).map_err(|e| vec![e])?;
    let mut reading = reader::Reader::new().read(&lines).map_err(|e| vec![e])?;
    let warnings = plxs_core::validate(&mut reading.analysis)?;
    reading.warnings.extend(warnings);
    Ok(reading)
}
