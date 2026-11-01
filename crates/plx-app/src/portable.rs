//! The portable Windows ZIP, like PrePoMax's: a `portable.txt` next to the program makes
//! nodalis keep its settings and material library in a `Settings` folder beside it instead
//! of the user's data directory, so the whole folder can be copied, run from a USB stick or
//! deleted without leaving anything behind.

use std::path::{Path, PathBuf};

/// Marker file next to the executable that switches on the portable mode.
pub const MARKER: &str = "portable.txt";

/// Folder next to the executable for settings and the material library in portable mode.
const DATA_DIR: &str = "Settings";

/// The program folder, if nodalis runs from the portable ZIP.
pub fn portable_root() -> Option<PathBuf> {
    let exe = std::env::current_exe().ok()?;
    portable_root_of(exe.parent()?)
}

fn portable_root_of(exe_dir: &Path) -> Option<PathBuf> {
    exe_dir
        .join(MARKER)
        .is_file()
        .then(|| exe_dir.to_path_buf())
}

/// Whether nodalis runs from the portable ZIP, e.g. so an update offers the ZIP and not
/// the installer.
pub fn is_portable() -> bool {
    portable_root().is_some()
}

/// Where settings and the material library are kept: `Settings` next to the program in
/// portable mode, else eframe's data directory (Linux `~/.local/share/nodalis`, Windows
/// `%APPDATA%\nodalis\data`).
pub fn data_dir() -> Option<PathBuf> {
    match portable_root() {
        Some(root) => Some(root.join(DATA_DIR)),
        None => eframe::storage_dir("nodalis"),
    }
}

/// The eframe settings file in portable mode; `None` leaves eframe's default.
pub fn persistence_path() -> Option<PathBuf> {
    portable_root().map(|root| root.join(DATA_DIR).join("app.ron"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn marker_file_switches_on_portable_mode() {
        let dir = std::env::temp_dir().join(format!("nodalis-portable-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        assert_eq!(portable_root_of(&dir), None);
        std::fs::write(dir.join(MARKER), "portable").unwrap();
        assert_eq!(portable_root_of(&dir), Some(dir.clone()));
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn development_builds_are_not_portable() {
        assert!(!is_portable());
        assert_eq!(data_dir(), eframe::storage_dir("nodalis"));
        assert_eq!(persistence_path(), None);
    }
}
