//! Lines of an input file: keyword lines with their parameters, and data lines, with
//! comments removed, continuations joined and included files inlined.

use std::path::{Path, PathBuf};

use plxs_core::Error;

/// A keyword line: the name as CalculiX compares it (upper case, no blanks: `*NODEFILE`)
/// and its parameters.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Keyword {
    pub name: String,
    /// Parameters in order: upper case name, and the value in upper case without blanks
    /// (`SURFACETOSURFACE`), except for file names (`INPUT`, `FILE`), which keep their
    /// case; quotes removed.
    pub parameters: Vec<(String, Option<String>)>,
    pub file: String,
    pub line: usize,
}

impl Keyword {
    /// The value of a parameter (`NSET=X` gives `X`), `None` when absent or without value.
    pub fn value(&self, name: &str) -> Option<&str> {
        let name = name.replace(' ', "");
        self.parameters
            .iter()
            .find(|(n, _)| *n == name)
            .and_then(|(_, v)| v.as_deref())
    }

    /// Whether a parameter (flag or with value) is present.
    pub fn has(&self, name: &str) -> bool {
        let name = name.replace(' ', "");
        self.parameters.iter().any(|(n, _)| *n == name)
    }

    /// The keyword as the user wrote it, for error messages: `*NODE FILE`.
    pub fn display_name(&self) -> String {
        spaced_keyword(&self.name)
    }

    /// A numeric parameter.
    pub fn number(&self, name: &str) -> Result<Option<f64>, Error> {
        match self.value(name) {
            None => Ok(None),
            Some(v) => v
                .trim()
                .parse::<f64>()
                .map(Some)
                .map_err(|_| self.error(format!("parameter {name} has no number: {v}"))),
        }
    }

    pub fn integer(&self, name: &str) -> Result<Option<u32>, Error> {
        match self.number(name)? {
            None => Ok(None),
            Some(v) if v >= 0.0 && v.fract() == 0.0 => Ok(Some(v as u32)),
            Some(_) => Err(self.error(format!(
                "parameter {name} is not a whole number: {}",
                self.value(name).unwrap_or_default()
            ))),
        }
    }

    pub fn error(&self, message: impl Into<String>) -> Error {
        Error::input(self.display_name(), message).at(&self.file, self.line)
    }

    /// Every parameter must be one of `allowed` (names without blanks), else an error names
    /// the first that is not; CalculiX rejects unknown parameters too.
    pub fn check_parameters(&self, allowed: &[&str]) -> Result<(), Error> {
        for (name, _) in &self.parameters {
            if !allowed.iter().any(|a| a.replace(' ', "") == *name) {
                return Err(self.error(format!("unknown parameter {name}")));
            }
        }
        Ok(())
    }
}

/// `*NODEFILE` -> `*NODE FILE`, for the keywords CalculiX writes with blanks.
pub fn spaced_keyword(name: &str) -> String {
    const SPACED: &[&str] = &[
        "*BEAM SECTION",
        "*CHANGE FRICTION",
        "*COMPLEX FREQUENCY",
        "*CONTACT FILE",
        "*CONTACT PAIR",
        "*CONTACT PRINT",
        "*COUPLED TEMPERATURE-DISPLACEMENT",
        "*CYCLIC SYMMETRY MODEL",
        "*DESIGN RESPONSE",
        "*DESIGN VARIABLES",
        "*EL FILE",
        "*EL PRINT",
        "*END STEP",
        "*GAP CONDUCTANCE",
        "*HEAT TRANSFER",
        "*INITIAL CONDITIONS",
        "*MEMBRANE SECTION",
        "*MODAL DAMPING",
        "*MODAL DYNAMICS",
        "*MODEL CHANGE",
        "*NO ANALYSIS",
        "*NODE FILE",
        "*NODE PRINT",
        "*PHYSICAL CONSTANTS",
        "*RIGID BODY",
        "*SELECT CYCLIC SYMMETRY MODES",
        "*SHELL SECTION",
        "*SOLID SECTION",
        "*SPECIFIC HEAT",
        "*STEADY STATE DYNAMICS",
        "*SURFACE BEHAVIOR",
        "*SURFACE INTERACTION",
        "*TIME POINTS",
        "*UNCOUPLED TEMPERATURE-DISPLACEMENT",
        "*USER ELEMENT",
        "*PRE-TENSION SECTION",
    ];
    SPACED
        .iter()
        .find(|s| s.replace(' ', "") == name)
        .map(|s| s.to_string())
        .unwrap_or_else(|| name.to_string())
}

/// A line of the input after lexing.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Line {
    Keyword(Keyword),
    /// A data line (continuations joined), with where it came from.
    Data {
        text: String,
        file: String,
        line: usize,
    },
    /// A comment, kept for the names Nodalis writes (`** Name: Force-1`, `** Step-1 ++`).
    Comment {
        text: String,
        #[allow(dead_code)]
        line: usize,
    },
}

/// Splits a data line into its comma-separated fields, trimmed; a trailing empty field (from
/// a trailing comma) is dropped.
pub fn split_fields(text: &str) -> Vec<&str> {
    let mut fields: Vec<&str> = text.split(',').map(str::trim).collect();
    while fields.last() == Some(&"") {
        fields.pop();
    }
    fields
}

/// Lexes a file's text; `base` is the directory `*INCLUDE` files are relative to.
pub fn lex(text: &str, file: &str, base: Option<&Path>) -> Result<Vec<Line>, Error> {
    let mut out = Vec::new();
    lex_into(text, file, base, &mut out, 0)?;
    Ok(out)
}

fn lex_into(
    text: &str,
    file: &str,
    base: Option<&Path>,
    out: &mut Vec<Line>,
    depth: usize,
) -> Result<(), Error> {
    if depth > 8 {
        return Err(Error::input("*INCLUDE", "includes nested too deeply"));
    }
    let mut lines = text.lines().enumerate().peekable();
    while let Some((index, raw)) = lines.next() {
        let number = index + 1;
        let trimmed = raw.trim();
        if trimmed.is_empty() {
            continue;
        }
        if let Some(comment) = trimmed.strip_prefix("**") {
            out.push(Line::Comment {
                text: comment.trim().to_string(),
                line: number,
            });
            continue;
        }
        // Continuation: a line ending in a comma goes on with the next.
        let mut joined = trimmed.to_string();
        while joined.ends_with(',') {
            match lines.peek() {
                Some((_, next)) if !next.trim().starts_with('*') && !next.trim().is_empty() => {
                    let (_, next) = lines.next().unwrap();
                    joined.push(' ');
                    joined.push_str(next.trim());
                }
                _ => break,
            }
        }
        if joined.starts_with('*') {
            let keyword = parse_keyword(&joined, file, number)?;
            if keyword.name == "*INCLUDE" {
                let Some(input) = keyword.value("INPUT") else {
                    return Err(keyword.error("no INPUT given"));
                };
                let path = match base {
                    Some(base) => base.join(input),
                    None => PathBuf::from(input),
                };
                let included = std::fs::read_to_string(&path)
                    .map_err(|e| keyword.error(format!("cannot read {}: {e}", path.display())))?;
                let name = path.display().to_string();
                lex_into(&included, &name, path.parent(), out, depth + 1)?;
                continue;
            }
            out.push(Line::Keyword(keyword));
        } else {
            out.push(Line::Data {
                text: joined,
                file: file.to_string(),
                line: number,
            });
        }
    }
    Ok(())
}

/// Parses `*NAME, PARAM=VALUE, FLAG`. Quoted values keep their case and may hold commas.
fn parse_keyword(text: &str, file: &str, line: usize) -> Result<Keyword, Error> {
    let fields = split_quoted(text);
    let name = fields[0]
        .split_whitespace()
        .collect::<String>()
        .to_ascii_uppercase();
    let mut parameters = Vec::new();
    for field in &fields[1..] {
        let field = field.trim();
        if field.is_empty() {
            continue;
        }
        match field.split_once('=') {
            Some((key, value)) => {
                let key = key
                    .split_whitespace()
                    .collect::<String>()
                    .to_ascii_uppercase();
                let value = value.trim();
                let value = if let Some(inner) =
                    value.strip_prefix('"').and_then(|v| v.strip_suffix('"'))
                {
                    inner.to_string()
                } else if key == "INPUT" || key == "FILE" {
                    value.to_string()
                } else {
                    // CalculiX ignores blanks in values: `SURFACE TO SURFACE`, `TOTAL TIME`.
                    value
                        .split_whitespace()
                        .collect::<String>()
                        .to_ascii_uppercase()
                };
                parameters.push((key, Some(value)));
            }
            None => {
                let key = field
                    .split_whitespace()
                    .collect::<String>()
                    .to_ascii_uppercase();
                parameters.push((key, None));
            }
        }
    }
    Ok(Keyword {
        name,
        parameters,
        file: file.to_string(),
        line,
    })
}

/// Splits at commas outside double quotes.
fn split_quoted(text: &str) -> Vec<String> {
    let mut fields = Vec::new();
    let mut current = String::new();
    let mut quoted = false;
    for c in text.chars() {
        match c {
            '"' => {
                quoted = !quoted;
                current.push(c);
            }
            ',' if !quoted => fields.push(std::mem::take(&mut current)),
            _ => current.push(c),
        }
    }
    fields.push(current);
    fields
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keywords_are_normalized_and_data_lines_continued() {
        let text = "*Node file, Output=3d, Last iterations\nRF, U\n** comment\n*Element, Type=C3D20\n1, 2, 3,\n4, 5\n*Submodel, Type=Node, Input=\"Global Model.frd\"\n";
        let lines = lex(text, "t.inp", None).unwrap();
        let Line::Keyword(k) = &lines[0] else {
            panic!()
        };
        assert_eq!(k.name, "*NODEFILE");
        assert_eq!(k.value("OUTPUT"), Some("3D"));
        assert!(k.has("LAST ITERATIONS"));
        assert_eq!(k.display_name(), "*NODE FILE");
        assert!(matches!(&lines[1], Line::Data { text, .. } if text == "RF, U"));
        assert!(matches!(&lines[2], Line::Comment { text, .. } if text == "comment"));
        assert!(matches!(&lines[4], Line::Data { text, line: 5, .. } if text == "1, 2, 3, 4, 5"));
        let Line::Keyword(k) = &lines[5] else {
            panic!()
        };
        assert_eq!(k.value("INPUT"), Some("Global Model.frd"));
    }
}
