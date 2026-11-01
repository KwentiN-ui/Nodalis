use std::path::{Path, PathBuf};

use super::InpError;

/// One meaningful input line with its origin, after comments are dropped and includes expanded.
#[derive(Clone, Debug, PartialEq)]
pub struct SourceLine {
    pub file: usize,
    pub number: usize,
    pub text: String,
}

/// Keyword line such as `*ELEMENT, TYPE=C3D8, ELSET=EALL`.
#[derive(Clone, Debug, PartialEq)]
pub struct Keyword {
    /// Upper case keyword without the leading `*`, internal blanks removed (`NODE PRINT` → `NODEPRINT`).
    pub name: String,
    /// Parameters with upper case names; values keep their spelling, flags have an empty value.
    pub params: Vec<(String, String)>,
}

impl Keyword {
    pub fn parse(line: &str) -> Self {
        let mut parts = line.trim_start_matches('*').split(',');
        let name = parts
            .next()
            .unwrap_or_default()
            .split_whitespace()
            .collect::<String>()
            .to_ascii_uppercase();
        let params = parts
            .map(str::trim)
            .filter(|p| !p.is_empty())
            .map(|p| match p.split_once('=') {
                Some((key, value)) => (key.trim().to_ascii_uppercase(), value.trim().to_string()),
                None => (p.to_ascii_uppercase(), String::new()),
            })
            .collect();
        Self { name, params }
    }

    pub fn param(&self, key: &str) -> Option<&str> {
        self.params
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.as_str())
    }

    pub fn has_flag(&self, key: &str) -> bool {
        self.param(key).is_some()
    }
}

pub fn is_keyword(text: &str) -> bool {
    text.starts_with('*')
}

/// Reads a file and all files it includes into a flat line list; `files` receives every path read.
pub fn load(path: &Path, files: &mut Vec<PathBuf>) -> Result<Vec<SourceLine>, InpError> {
    let text = std::fs::read_to_string(path).map_err(|source| InpError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    expand(&text, path.parent(), path.to_path_buf(), files)
}

/// Like [`load`] for text that is already in memory; includes resolve against `base_dir`.
pub fn expand(
    text: &str,
    base_dir: Option<&Path>,
    name: PathBuf,
    files: &mut Vec<PathBuf>,
) -> Result<Vec<SourceLine>, InpError> {
    if files.len() > 256 {
        return Err(InpError::Parse {
            file: name,
            line: 0,
            message: "Too many nested *INCLUDE files".into(),
        });
    }
    let file = files.len();
    files.push(name.clone());
    let mut lines = Vec::new();
    for (index, raw) in text.lines().enumerate() {
        let trimmed = raw.trim();
        // Comments are dropped but Nodalis's profile of a general beam section.
        if trimmed.is_empty()
            || (trimmed.starts_with("**") && !trimmed.starts_with(super::PROFILE_COMMENT.trim()))
        {
            continue;
        }
        if is_keyword(trimmed) {
            let keyword = Keyword::parse(trimmed);
            if keyword.name == "INCLUDE" {
                let Some(input) = keyword.param("INPUT") else {
                    return Err(InpError::Parse {
                        file: name,
                        line: index + 1,
                        message: "*INCLUDE without INPUT=".into(),
                    });
                };
                let input = input.trim_matches('"');
                let include = base_dir.map_or_else(|| PathBuf::from(input), |dir| dir.join(input));
                lines.extend(load(&include, files)?);
                continue;
            }
        }
        lines.push(SourceLine {
            file,
            number: index + 1,
            text: trimmed.to_string(),
        });
    }
    Ok(lines)
}

/// Splits a data line into its comma separated, trimmed fields; a trailing comma adds no field.
pub fn fields(text: &str) -> impl Iterator<Item = &str> {
    let text = text.strip_suffix(',').unwrap_or(text);
    text.split(',').map(str::trim)
}

pub fn parse_f64(field: &str) -> Option<f64> {
    field
        .parse()
        .ok()
        .or_else(|| field.replace(['D', 'd'], "E").parse().ok())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keyword_parameters() {
        let keyword = Keyword::parse("*Element, type=C3D10, ELSET = Eall");
        assert_eq!(keyword.name, "ELEMENT");
        assert_eq!(keyword.param("TYPE"), Some("C3D10"));
        assert_eq!(keyword.param("ELSET"), Some("Eall"));
        let keyword = Keyword::parse("*NSET,NSET=Fix,GENERATE");
        assert!(keyword.has_flag("GENERATE"));
        assert_eq!(Keyword::parse("*Node Print, NSET=N").name, "NODEPRINT");
    }

    #[test]
    fn comments_and_blank_lines_are_dropped() {
        let mut files = Vec::new();
        let lines = expand(
            "** comment\n\n*NODE\n1, 0, 0, 0\n",
            None,
            "a.inp".into(),
            &mut files,
        )
        .unwrap();
        assert_eq!(lines.len(), 2);
        assert_eq!(lines[1].number, 4);
    }

    #[test]
    fn fortran_exponents_are_accepted() {
        assert_eq!(parse_f64("1.5D2"), Some(150.0));
        assert_eq!(parse_f64("-2.e-3"), Some(-0.002));
        assert_eq!(parse_f64("x"), None);
    }

    #[test]
    fn trailing_comma_adds_no_field() {
        assert_eq!(fields("1, 2, 3,").collect::<Vec<_>>(), ["1", "2", "3"]);
    }
}
