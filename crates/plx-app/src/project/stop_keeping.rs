//! Asking what happens to the increments a solve has finished when it is stopped: Stop in
//! the analysis window or the solution monitor, and Stop Update.
//!
//! A solve without finished increments stops at once, as before. One with some asks
//! whether their results are kept: Stop and Keep Results writes them and keeps them as the
//! results of a stopped run, the step stopped halfway marked as not finished; Stop and
//! Discard stops as before and keeps nothing; Cancel lets the solve go on. A solve already
//! stopping (writing its results) is stopped at once by another Stop, which drops them.

use egui::RichText;

use super::ProjectApp;

/// What a Stop asked about stops.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Stopping {
    /// The solve of this analysis.
    One(u64),
    /// Update Project, with all its solves.
    All,
}

/// The answer to the question.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Answer {
    Keep,
    Discard,
    Cancel,
}

impl ProjectApp {
    /// The solves a stop of `stopping` would lose finished increments of: their names and
    /// increments.
    fn finished_increments(&self, stopping: Stopping) -> Vec<(String, u32)> {
        (self.solves.iter())
            .filter(|s| s.active() && !s.is_stopping())
            .filter(|s| stopping == Stopping::All || stopping == Stopping::One(s.id))
            .map(|s| (s.name.clone(), s.increments_done()))
            .filter(|&(_, done)| done > 0)
            .collect()
    }

    /// Stop for the solve of one analysis: asks first when it has finished increments.
    pub(super) fn stop_solve(&mut self, id: u64) {
        match self.finished_increments(Stopping::One(id)).is_empty() {
            true => self.stop_solve_now(id, false),
            false => self.stop_asking = Some(Stopping::One(id)),
        }
    }

    /// Stop Update: asks first when a solve has finished increments.
    pub(super) fn stop_update(&mut self) {
        match self.finished_increments(Stopping::All).is_empty() {
            true => self.stop_update_now(false),
            false => self.stop_asking = Some(Stopping::All),
        }
    }

    /// The question, while it is asked; it goes away by itself once nothing it asks about
    /// runs any more.
    pub(super) fn stop_keeping_window(&mut self, ctx: &egui::Context) {
        let Some(stopping) = self.stop_asking else {
            return;
        };
        let finished = self.finished_increments(stopping);
        if finished.is_empty() {
            self.stop_asking = None;
            return;
        }
        let Some(answer) = ask(ctx, &finished) else {
            return;
        };
        self.stop_asking = None;
        let keep = match answer {
            Answer::Keep => true,
            Answer::Discard => false,
            Answer::Cancel => return,
        };
        match stopping {
            Stopping::One(id) => self.stop_solve_now(id, keep),
            Stopping::All => self.stop_update_now(keep),
        }
    }
}

/// What is lost or kept, as a sentence.
fn consequence(finished: &[(String, u32)]) -> String {
    let increments = |n: u32| match n {
        1 => "1 increment".to_owned(),
        n => format!("{n} increments"),
    };
    let done = match finished {
        [(name, n)] => format!("{name} has finished {}.", increments(*n)),
        _ => {
            let list: Vec<String> = (finished.iter())
                .map(|(name, n)| format!("{name} ({})", increments(*n)))
                .collect();
            format!("Finished so far: {}.", list.join(", "))
        }
    };
    format!(
        "{done} Stop and Keep Results writes them and keeps them as the results of a \
         stopped run; a step stopped halfway is marked as not finished. Stop and Discard \
         keeps nothing."
    )
}

/// The question over the whole window. Escape cancels; Enter does nothing, so that no
/// solve is stopped by a key pressed by habit.
fn ask(ctx: &egui::Context, finished: &[(String, u32)]) -> Option<Answer> {
    let question = "Keep the results of the increments done?";
    let mut answer = None;
    egui::Modal::new(crate::ids::window(ctx, "stop keeping results")).show(ctx, |ui| {
        ui.set_max_width(440.0);
        ui.label(RichText::new(question).strong().size(15.0));
        ui.add_space(4.0);
        ui.label(consequence(finished));
        ui.add_space(10.0);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui
                .button("Cancel")
                .on_hover_text("The solve goes on")
                .clicked()
            {
                answer = Some(Answer::Cancel);
            }
            let discard = RichText::new("Stop and Discard").color(egui::Color32::WHITE);
            let discard = egui::Button::new(discard).fill(ui.visuals().error_fg_color);
            if ui.add(discard).clicked() {
                answer = Some(Answer::Discard);
            }
            let keep = RichText::new("Stop and Keep Results").color(egui::Color32::WHITE);
            let keep = egui::Button::new(keep).fill(ui.visuals().selection.bg_fill);
            if ui.add(keep).clicked() {
                answer = Some(Answer::Keep);
            }
        });
    });
    if answer.is_none() && ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
        answer = Some(Answer::Cancel);
    }
    answer
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_question_names_the_increments() {
        let one = consequence(&[("Beam".into(), 1)]);
        assert!(one.starts_with("Beam has finished 1 increment."), "{one}");
        let two = consequence(&[("A".into(), 3), ("B".into(), 1)]);
        assert!(two.starts_with("Finished so far: A (3 increments), B (1 increment)."));
    }
}
