//! The building blocks the windows share, after the four patterns of the GUI concept:
//!
//! - A, item editors, live in the properties panel ([`crate::dock`]).
//! - B, tool windows such as Section View, act at once: Close on the right, and Off on the
//!   left where a state stays after closing ([`tool_footer`]).
//! - C, confirmations, only for what Undo cannot take back: the question as the title, one
//!   line on what follows, Cancel and the verb in red ([`confirm`]).
//! - D, work windows such as the Contact Worksheet: Close at the bottom right, or Cancel and
//!   the verb where the window makes something ([`work_footer`]).

use egui::{Context, RichText, Ui};

/// What the foot of a tool window was asked for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tool {
    None,
    /// Turns off what the tool shows, and closes it.
    Off,
    Close,
}

/// The foot of a tool window: Off on the left when `off` is given (greyed out unless it is
/// true), Close on the right.
pub fn tool_footer(ui: &mut Ui, off: Option<bool>) -> Tool {
    tool_footer_with(ui, off, |_| {})
}

/// [`tool_footer`] with more buttons on the left, after Off.
pub fn tool_footer_with(ui: &mut Ui, off: Option<bool>, left: impl FnOnce(&mut Ui)) -> Tool {
    let mut clicked = Tool::None;
    ui.separator();
    ui.horizontal(|ui| {
        if let Some(enabled) = off
            && ui.add_enabled(enabled, egui::Button::new("Off")).clicked()
        {
            clicked = Tool::Off;
        }
        left(ui);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.button("Close").clicked() {
                clicked = Tool::Close;
            }
        });
    });
    clicked
}

/// What the foot of a work window was asked for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Work {
    None,
    /// The verb: OK, Create 12 Pairs, ...
    Verb,
    /// Close, or Cancel next to a verb.
    Close,
}

/// The foot of a work window, from the right: the verb when given (primary), then Cancel;
/// without a verb only Close. `verb` is its label and whether it can be used.
pub fn work_footer(ui: &mut Ui, verb: Option<(&str, bool)>) -> Work {
    let mut clicked = Work::None;
    ui.separator();
    ui.with_layout(
        egui::Layout::right_to_left(egui::Align::Center),
        |ui| match verb {
            Some((label, enabled)) => {
                if ui.add_enabled(enabled, egui::Button::new(label)).clicked() {
                    clicked = Work::Verb;
                }
                if ui.button("Cancel").clicked() {
                    clicked = Work::Close;
                }
            }
            None => {
                if ui.button("Close").clicked() {
                    clicked = Work::Close;
                }
            }
        },
    );
    clicked
}

/// Close at the bottom right of a work window whose table or plot fills the rest; call it
/// before drawing that. Returns whether Close was clicked.
pub fn close_bar(ui: &mut Ui, id: &str) -> bool {
    let mut close = false;
    egui::Panel::bottom(egui::Id::new(id).with("close bar"))
        .frame(egui::Frame::new())
        .show(ui, |ui| close = work_footer(ui, None) == Work::Close);
    close
}

/// A confirmation over the whole window: `question` as its title, `consequence` below,
/// Cancel and `verb`, drawn in red. Enter does it, Escape
/// cancels. Returns `Some(true)` for the verb, `Some(false)` for Cancel, `None` while open.
pub fn confirm(
    ctx: &Context,
    id: &str,
    question: &str,
    consequence: &str,
    verb: &str,
) -> Option<bool> {
    let mut answer = None;
    egui::Modal::new(crate::ids::window(ctx, id)).show(ctx, |ui| {
        ui.set_max_width(380.0);
        ui.label(RichText::new(question).strong().size(15.0));
        ui.add_space(4.0);
        ui.label(consequence);
        ui.add_space(10.0);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            let red = RichText::new(verb).color(egui::Color32::WHITE);
            let button = egui::Button::new(red).fill(crate::style::error());
            if ui.add(button).clicked() {
                answer = Some(true);
            }
            if ui.button("Cancel").clicked() {
                answer = Some(false);
            }
        });
    });
    if answer.is_none() {
        if ctx.input(|i| i.key_pressed(egui::Key::Enter)) {
            answer = Some(true);
        } else if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            answer = Some(false);
        }
    }
    answer
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One frame with `events`; returns the answer of the confirmation.
    fn frame(ctx: &Context, events: Vec<egui::Event>) -> Option<bool> {
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(800.0, 600.0),
            )),
            events,
            ..Default::default()
        };
        let mut answer = None;
        let mut output = ctx.run_ui(input, |ui| {
            answer = confirm(ui.ctx(), "test", "Remove it?", "It is gone then.", "Remove");
        });
        output.textures_delta.clear();
        answer
    }

    fn key(key: egui::Key) -> egui::Event {
        egui::Event::Key {
            key,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        }
    }

    #[test]
    fn enter_confirms_and_escape_cancels_a_confirmation() {
        let ctx = Context::default();
        assert_eq!(frame(&ctx, Vec::new()), None);
        assert_eq!(frame(&ctx, vec![key(egui::Key::Enter)]), Some(true));
        assert_eq!(frame(&ctx, vec![key(egui::Key::Escape)]), Some(false));
    }
}
