//! Material library files: the user's own [`MaterialLibrary`] in readable RON, so that
//! materials can also be added with a text editor.

use std::path::{Path, PathBuf};

use plx_model::MaterialLibrary;
use plx_model::library::LIBRARY_FORMAT;

#[derive(Debug, thiserror::Error)]
pub enum LibraryError {
    #[error("{path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("{path}: not a valid material library: {message}")]
    Format { path: PathBuf, message: String },
    #[error("{path} comes from a newer Nodalis version (format {format})")]
    Newer { path: PathBuf, format: u32 },
}

/// Writes the library, creating its directory; a crash while saving leaves an existing
/// file intact.
pub fn save_library(path: &Path, library: &MaterialLibrary) -> Result<(), LibraryError> {
    let config = ron::ser::PrettyConfig::default();
    let text = ron::ser::to_string_pretty(library, config).map_err(|e| LibraryError::Format {
        path: path.to_owned(),
        message: e.to_string(),
    })?;
    let io = |source| LibraryError::Io {
        path: path.to_owned(),
        source,
    };
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(io)?;
    }
    let temporary = path.with_extension("ron.tmp");
    std::fs::write(&temporary, text).map_err(io)?;
    std::fs::rename(&temporary, path).map_err(io)
}

/// Reads the library; a missing file gives the built-in default library.
pub fn read_library(path: &Path) -> Result<MaterialLibrary, LibraryError> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Ok(MaterialLibrary::default());
        }
        Err(source) => {
            return Err(LibraryError::Io {
                path: path.to_owned(),
                source,
            });
        }
    };
    let mut library: MaterialLibrary = ron::from_str(&text).map_err(|e| LibraryError::Format {
        path: path.to_owned(),
        message: e.to_string(),
    })?;
    if library.format > LIBRARY_FORMAT {
        return Err(LibraryError::Newer {
            path: path.to_owned(),
            format: library.format,
        });
    }
    library.update_defaults();
    Ok(library)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn libraries_read_back_what_was_saved() {
        let dir = std::env::temp_dir().join(format!("plx-library-{}", std::process::id()));
        let path = dir.join("sub").join("materials.ron");
        assert_eq!(read_library(&path).unwrap(), MaterialLibrary::default());

        let mut library = MaterialLibrary::default();
        let category = library.add_category(&[]).unwrap();
        library.rename(&category, "Custom").unwrap();
        save_library(&path, &library).unwrap();
        assert_eq!(read_library(&path).unwrap(), library);

        std::fs::write(&path, "(format: 1, root: 3)").unwrap();
        assert!(matches!(
            read_library(&path),
            Err(LibraryError::Format { .. })
        ));
        std::fs::remove_dir_all(dir).unwrap();
    }
}
