//! Saving the project on a worker thread, so that the windows do not freeze while a large
//! project is written, and saving it by itself a while after a change (autosave).
//!
//! The project is copied on the UI thread, which takes a few milliseconds even for a large
//! one; turning the copy into text and writing it, which take seconds, run on the worker.
//! Changes made meanwhile are not in the file and wait for the next save.

use std::path::PathBuf;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::time::{Duration, Instant};

use plx_io::project::Project;

use super::{ProjectApp, pick_project_file, timestamped};
use crate::settings;

/// How a save was asked for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum SaveKind {
    /// File > Save, or Save As: the project goes on in the file.
    Save,
    /// The project saved by itself, see [`settings::Saving`].
    Auto,
    /// File > Save Without Results: a copy; the project stays on its own file.
    WithoutResults,
}

/// A save running on its worker thread.
struct Running {
    path: PathBuf,
    kind: SaveKind,
    /// [`ProjectApp::edits`] when the project was copied for it.
    edits: u64,
    /// The project it was copied from, see `ProjectApp::graph_generation`: once another
    /// project is opened, the file belongs to none of the open analyses.
    generation: u64,
    receiver: Receiver<Result<(), String>>,
}

/// What the project window knows about saving.
#[derive(Default)]
pub(super) struct Saver {
    running: Option<Running>,
    /// A save asked for while another one ran, with its file: it starts once that one is done.
    queued: Option<(SaveKind, PathBuf)>,
    /// [`ProjectApp::edits`] as last saved or opened.
    saved_edits: u64,
    /// Since when the project has changes that are not saved; autosave waits the interval from
    /// there.
    dirty_since: Option<Instant>,
    /// Changes of the analysis windows: [`ProjectApp::edits`] counts them with those of the
    /// schematic.
    pub(super) window_edits: u64,
    /// The window closes once the saves are done.
    close_after: bool,
}

/// What autosave does now.
#[derive(Debug, PartialEq)]
pub(super) enum Autosave {
    /// Nothing to save, or autosave is off.
    Idle,
    /// Saves once this time is over.
    Wait(Duration),
    /// Saves now.
    Due,
}

/// Whether autosave saves now: `dirty_for` is how long the project has had changes that are
/// not saved, `None` if it has none.
pub(super) fn autosave(settings: &settings::Saving, dirty_for: Option<Duration>) -> Autosave {
    if !settings.autosave {
        return Autosave::Idle;
    }
    let Some(dirty_for) = dirty_for else {
        return Autosave::Idle;
    };
    match settings.interval().checked_sub(dirty_for) {
        Some(left) if !left.is_zero() => Autosave::Wait(left),
        _ => Autosave::Due,
    }
}

/// Writes the project on a worker thread; the receiver gets the outcome. A panic while
/// writing is reported like any other error.
fn write_in_background(path: PathBuf, project: Project) -> Receiver<Result<(), String>> {
    let (sender, receiver) = std::sync::mpsc::channel();
    let spawned = std::thread::Builder::new()
        .name("save project".into())
        .spawn(move || {
            let written = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                plx_io::project::save_project(&path, &project).map_err(|e| e.to_string())
            }))
            .unwrap_or_else(|panic| {
                let message = (panic.downcast_ref::<&str>().copied())
                    .or_else(|| panic.downcast_ref::<String>().map(String::as_str))
                    .unwrap_or("unknown error");
                Err(format!("writing the file failed: {message}"))
            });
            let _ = sender.send(written);
        });
    if let Err(error) = spawned {
        let (sender, failed) = std::sync::mpsc::channel();
        let _ = sender.send(Err(format!("no thread to save on: {error}")));
        return failed;
    }
    receiver
}

impl ProjectApp {
    /// Changes of the project so far: of the schematic and in the analysis windows. A save
    /// keeps the count it saved, so that autosave knows whether there is anything to save.
    pub(super) fn edits(&self) -> u64 {
        self.history.edits + self.saver.window_edits
    }

    /// Whether the project has changes since it was last saved or opened.
    pub(super) fn edits_unsaved(&self) -> bool {
        self.edits() != self.saver.saved_edits
    }

    /// Whether a save runs or waits for the one running.
    pub(super) fn is_saving(&self) -> bool {
        self.saver.running.is_some() || self.saver.queued.is_some()
    }

    /// The project was opened or made anew: nothing to save yet.
    pub(super) fn forget_edits(&mut self) {
        self.saver.saved_edits = self.edits();
        self.saver.dirty_since = None;
    }

    /// Saves the project to its file, or to one the user picks for a new project or with
    /// `save_as`.
    pub(super) fn save(&mut self, save_as: bool) {
        let path = match &self.path {
            Some(path) if !save_as => path.clone(),
            _ => {
                let Some(path) = pick_project_file(self.path.as_deref(), "Save Project", "") else {
                    return;
                };
                path
            }
        };
        self.save_to(SaveKind::Save, path);
    }

    /// Saves a copy of the project without the results of its analyses, for example to send
    /// it; the open project stays on its own file and keeps its results.
    pub(super) fn save_without_results(&mut self) {
        let Some(path) = pick_project_file(
            self.path.as_deref(),
            "Save Project Without Results",
            "-without-results",
        ) else {
            return;
        };
        self.save_to(SaveKind::WithoutResults, path);
    }

    /// Copies the project and writes it on a worker thread; while another save runs, it
    /// starts after it.
    fn save_to(&mut self, kind: SaveKind, path: PathBuf) {
        if self.saver.running.is_some() {
            if kind != SaveKind::Auto {
                self.output.push(format!(
                    "{} is saved once the save running now is done",
                    path.display()
                ));
                self.saver.queued = Some((kind, path));
            }
            return;
        }
        if let Some(name) = self
            .nodes
            .iter()
            .find(|n| n.window().is_some_and(|w| w.model().is_none()))
        {
            if kind != SaveKind::Auto {
                self.output
                    .push(format!("Not saved: {} is still loading.", name.name));
            }
            return;
        }
        let project = self.project(kind != SaveKind::WithoutResults);
        let receiver = write_in_background(path.clone(), project);
        self.saver.running = Some(Running {
            path,
            kind,
            edits: self.edits(),
            generation: self.graph_generation,
            receiver,
        });
    }

    /// Takes the outcome of a save that is done, starts a queued one or autosave, and
    /// closes the window once the saves are done if that was asked for. Called every frame.
    pub(super) fn poll_saving(&mut self, ctx: &egui::Context) {
        // A close of the window while saving waits for the file.
        if ctx.input(|i| i.viewport().close_requested()) && self.is_saving() {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            if !self.saver.close_after {
                self.output
                    .push("Nodalis closes once the project is saved".into());
            }
            self.saver.close_after = true;
        }
        self.note_edits();
        if let Some(running) = &self.saver.running {
            let outcome = match running.receiver.try_recv() {
                Ok(outcome) => outcome,
                Err(TryRecvError::Empty) => {
                    // Nothing wakes the window when the worker is done but the next frame.
                    ctx.request_repaint_after(Duration::from_millis(100));
                    crate::save_status::set(ctx, self.save_status());
                    return;
                }
                Err(TryRecvError::Disconnected) => Err("saving stopped unexpectedly".into()),
            };
            let Some(running) = self.saver.running.take() else {
                return;
            };
            let saved = outcome.is_ok();
            self.saved(running, outcome);
            if let Some((kind, path)) = self.saver.queued.take() {
                self.save_to(kind, path);
            }
            if self.saver.close_after && self.saver.running.is_none() {
                self.saver.close_after = false;
                if saved {
                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                } else {
                    self.output
                        .push("Nodalis stays open: the project could not be saved".into());
                }
            }
        }
        self.run_autosave(ctx);
        crate::save_status::set(ctx, self.save_status());
    }

    /// Notes when the project got changes that are not saved.
    fn note_edits(&mut self) {
        if !self.edits_unsaved() {
            self.saver.dirty_since = None;
        } else if self.saver.dirty_since.is_none() {
            self.saver.dirty_since = Some(Instant::now());
        }
    }

    /// Starts autosave once the project has had unsaved changes for the interval of the
    /// settings, or wakes the window then.
    fn run_autosave(&mut self, ctx: &egui::Context) {
        if self.saver.running.is_some() || self.opening.is_some() {
            return;
        }
        // A new project has no file to save to yet.
        let Some(path) = self.path.clone() else {
            return;
        };
        let dirty_for = self.saver.dirty_since.map(|since| since.elapsed());
        match autosave(&self.settings.saving, dirty_for) {
            Autosave::Idle => {}
            Autosave::Wait(left) => ctx.request_repaint_after(left),
            // Not in the middle of a drag: the block would be saved half moved.
            Autosave::Due if ctx.input(|i| i.pointer.any_down()) => {}
            Autosave::Due => {
                self.save_to(SaveKind::Auto, path);
                // A save that cannot start now, while an analysis loads, is tried again later.
                if self.saver.running.is_none() {
                    self.saver.dirty_since = Some(Instant::now());
                }
            }
        }
    }

    /// The line of the status bar while saving.
    fn save_status(&self) -> Option<String> {
        let running = self.saver.running.as_ref()?;
        let name = (running.path.file_name()).map_or_else(
            || running.path.display().to_string(),
            |n| n.to_string_lossy().into_owned(),
        );
        Some(match running.kind {
            SaveKind::Auto => format!("Saving {name} (autosave) …"),
            _ => format!("Saving {name} …"),
        })
    }

    /// A save is done: the project belongs to its file now.
    fn saved(&mut self, running: Running, outcome: Result<(), String>) {
        let Running {
            path,
            kind,
            edits,
            generation,
            ..
        } = running;
        if let Err(error) = outcome {
            self.output.push(format!("Not saved: {error}"));
            if kind == SaveKind::Auto {
                self.toast = Some(crate::toast::Toast::new(
                    format!("Autosave failed: {error}"),
                    false,
                ));
                // Tried again after the interval, not in every frame.
                self.saver.dirty_since = Some(Instant::now());
            }
            return;
        }
        if kind == SaveKind::WithoutResults {
            self.output.push(timestamped(format!(
                "{} saved without results",
                path.display()
            )));
            return;
        }
        let line = timestamped(match kind {
            SaveKind::Auto => format!("{} saved (autosave)", path.display()),
            _ => format!("{} saved", path.display()),
        });
        self.output.push(line.clone());
        // Another project was opened meanwhile: the file is not that one's.
        if generation != self.graph_generation {
            return;
        }
        let folder =
            crate::analysis::job_name(&path.file_stem().unwrap_or_default().to_string_lossy());
        for node in &mut self.nodes {
            if let Some(window) = node.window_mut() {
                window.saved(&path);
                window.set_project_folder(folder.clone());
                window.message(line.clone());
            }
        }
        self.remember(&path);
        self.path = Some(path);
        self.saver.saved_edits = edits;
        // Changes made while it was written wait for the next save.
        self.saver.dirty_since = (self.edits() != edits).then(Instant::now);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn empty_project() -> Project {
        Project {
            nodes: Vec::new(),
            links: Vec::new(),
            frames: Vec::new(),
            compact: Vec::new(),
            units: Default::default(),
            format: plx_model::PROJECT_FORMAT,
        }
    }

    fn every(minutes: u32) -> settings::Saving {
        settings::Saving {
            autosave: true,
            minutes,
        }
    }

    #[test]
    fn autosave_waits_the_interval_after_the_first_change() {
        let minute = Duration::from_secs(60);
        assert_eq!(autosave(&every(5), None), Autosave::Idle);
        assert_eq!(
            autosave(&every(5), Some(2 * minute)),
            Autosave::Wait(3 * minute)
        );
        assert_eq!(autosave(&every(5), Some(5 * minute)), Autosave::Due);
        assert_eq!(autosave(&every(5), Some(9 * minute)), Autosave::Due);
    }

    #[test]
    fn autosave_off_never_saves() {
        let off = settings::Saving {
            autosave: false,
            minutes: 1,
        };
        assert_eq!(
            autosave(&off, Some(Duration::from_secs(3600))),
            Autosave::Idle
        );
    }

    #[test]
    fn a_save_on_the_worker_writes_the_file() {
        let dir = std::env::temp_dir().join(format!("nodalis-save-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("worker.ndl");
        let project = empty_project();
        let receiver = write_in_background(path.clone(), project);
        let outcome = receiver.recv_timeout(Duration::from_secs(30)).unwrap();
        assert_eq!(outcome, Ok(()));
        assert!(plx_io::project::read_project(&path).is_ok());
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn a_failed_save_reports_its_error() {
        let path = std::env::temp_dir()
            .join("nodalis-no-such-folder")
            .join("deep")
            .join("x.ndl");
        let receiver = write_in_background(path, empty_project());
        let outcome = receiver.recv_timeout(Duration::from_secs(30)).unwrap();
        assert!(outcome.is_err());
    }
}
