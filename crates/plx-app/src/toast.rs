//! A toast: one line near the bottom of the middle area, above the scale bar, with at most
//! one button, that goes away by itself. A newer toast replaces the shown one.

use egui::{Context, Rect};

/// How long a toast stays.
pub const SECONDS: f64 = 6.0;

pub struct Toast {
    pub text: String,
    /// Offers Undo, for a change that Undo takes back.
    pub undo: bool,
    /// The time it was shown first; set when it is drawn.
    pub since: Option<f64>,
}

impl Toast {
    pub fn new(text: impl Into<String>, undo: bool) -> Self {
        Self {
            text: text.into(),
            undo,
            since: None,
        }
    }
}

pub enum Event {
    None,
    Undo,
    /// The time is up, or the toast was closed.
    Gone,
}

/// Draws the toast at the bottom centre of `area`.
pub fn show(ctx: &Context, toast: &mut Toast, area: Rect) -> Event {
    let now = ctx.input(|i| i.time);
    let since = *toast.since.get_or_insert(now);
    let left = SECONDS - (now - since);
    if left <= 0.0 {
        return Event::Gone;
    }
    ctx.request_repaint_after(std::time::Duration::from_secs_f64(left.min(0.5)));
    let mut event = Event::None;
    egui::Area::new(crate::ids::window(ctx, "toast"))
        .pivot(egui::Align2::CENTER_BOTTOM)
        .fixed_pos(area.center_bottom() - egui::vec2(0.0, 72.0))
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(&toast.text);
                    if toast.undo && ui.button("Undo").clicked() {
                        event = Event::Undo;
                    }
                });
            });
        });
    event
}
