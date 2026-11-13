//! Asking what happens to the changes that are not saved before the project is left: when
//! Nodalis closes, and before a new project or another one replaces it.
//!
//! The question offers Save, Discard and Cancel. Save goes on with what was asked once the
//! file is written, also when a file has to be picked first for a project never saved.

use std::path::PathBuf;

use egui::RichText;

use super::ProjectApp;

/// What leaves the open project.
#[derive(Clone, Debug, PartialEq)]
pub(super) enum Leave {
    /// Closing the project window, which ends Nodalis.
    Close,
    /// File > New Project.
    New,
    /// Opening this project file: File > Open, Open Recent, a dropped file.
    Open(PathBuf),
}

/// What the project window knows about leaving the project.
#[derive(Default)]
pub(super) struct Unsaved {
    /// The question is shown for this.
    asking: Option<Leave>,
    /// Saved for this: it goes on once the save is done.
    after_save: Option<Leave>,
    /// The window closes without asking: the user answered, or nothing was unsaved.
    may_close: bool,
}

/// The answer to the question.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Answer {
    Save,
    Discard,
    Cancel,
}

impl ProjectApp {
    /// Whether leaving the project loses something: changes since it was saved or opened, or
    /// blocks of a project that was never saved.
    pub(super) fn has_unsaved_changes(&self) -> bool {
        self.edits_unsaved() || (self.path.is_none() && !self.nodes.is_empty())
    }

    /// File > New Project, after asking about unsaved changes.
    pub(super) fn ask_new_project(&mut self, ctx: &egui::Context) {
        self.leave(ctx, Leave::New);
    }

    /// Opens `path` in place of the project, after asking about unsaved changes.
    pub(super) fn ask_open(&mut self, ctx: &egui::Context, path: PathBuf) {
        if self.opening.is_some() {
            return;
        }
        self.leave(ctx, Leave::Open(path));
    }

    /// Does `leave` now, or asks first if the project has unsaved changes.
    fn leave(&mut self, ctx: &egui::Context, leave: Leave) {
        if self.unsaved.asking.is_some() || self.unsaved.after_save.is_some() {
            return;
        }
        if self.has_unsaved_changes() {
            self.unsaved.asking = Some(leave);
        } else {
            self.leave_now(ctx, leave);
        }
    }

    fn leave_now(&mut self, ctx: &egui::Context, leave: Leave) {
        match leave {
            Leave::Close => {
                self.unsaved.may_close = true;
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            Leave::New => self.new_project(),
            Leave::Open(path) => self.open(ctx, path),
        }
    }

    /// Keeps the window open when it is closed with unsaved changes, and asks instead. Call it
    /// after [`ProjectApp::poll_saving`], which keeps it open while a save runs.
    pub(super) fn watch_close(&mut self, ctx: &egui::Context) {
        if !ctx.input(|i| i.viewport().close_requested()) || self.unsaved.may_close {
            return;
        }
        if self.is_saving() {
            return;
        }
        if self.unsaved.after_save.is_some() {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            return;
        }
        if self.has_unsaved_changes() {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            // Closing is what the user wants now, more than what was asked before.
            self.unsaved.asking = Some(Leave::Close);
            ctx.request_repaint();
        }
    }

    /// Shows the question while it is asked, and goes on after a save it asked for.
    pub(super) fn unsaved_window(&mut self, ctx: &egui::Context) {
        if self.unsaved.after_save.is_some() && !self.is_saving() {
            let leave = self.unsaved.after_save.take();
            if let Some(leave) = leave {
                if self.has_unsaved_changes() {
                    self.output
                        .push("The project was not saved: it stays open".into());
                } else {
                    self.leave_now(ctx, leave);
                }
            }
        }
        let Some(leave) = self.unsaved.asking.clone() else {
            return;
        };
        let name = self.project_name();
        let question = match self.path {
            Some(_) => format!("Save the changes to {name}?"),
            None => "Save the new project?".to_string(),
        };
        let what = match leave {
            Leave::Close => "Nodalis closes.",
            Leave::New => "A new project replaces it.",
            Leave::Open(_) => "Another project replaces it.",
        };
        let lost = match self.path {
            Some(_) => "Changes that are not saved are lost.",
            None => "The project was never saved and is lost.",
        };
        let consequence = format!("{what} {lost}");
        let Some(answer) = ask(ctx, &question, &consequence) else {
            return;
        };
        self.unsaved.asking = None;
        match answer {
            Answer::Cancel => {}
            Answer::Discard => self.leave_now(ctx, leave),
            Answer::Save => {
                self.save(false);
                // Nothing saves when no file was picked or an analysis still loads: the
                // project stays as it is.
                if self.is_saving() {
                    self.unsaved.after_save = Some(leave);
                }
            }
        }
    }
}

/// The question over the whole window. Enter saves, Escape cancels.
fn ask(ctx: &egui::Context, question: &str, consequence: &str) -> Option<Answer> {
    let mut answer = None;
    egui::Modal::new(crate::ids::window(ctx, "unsaved changes")).show(ctx, |ui| {
        ui.set_max_width(420.0);
        ui.label(RichText::new(question).strong().size(15.0));
        ui.add_space(4.0);
        ui.label(consequence);
        ui.add_space(10.0);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            let save = RichText::new("Save").color(egui::Color32::WHITE);
            let save = egui::Button::new(save).fill(ui.visuals().selection.bg_fill);
            if ui.add(save).clicked() {
                answer = Some(Answer::Save);
            }
            if ui.button("Discard").clicked() {
                answer = Some(Answer::Discard);
            }
            if ui.button("Cancel").clicked() {
                answer = Some(Answer::Cancel);
            }
        });
    });
    if answer.is_none() {
        if ctx.input(|i| i.key_pressed(egui::Key::Enter)) {
            answer = Some(Answer::Save);
        } else if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            answer = Some(Answer::Cancel);
        }
    }
    answer
}
