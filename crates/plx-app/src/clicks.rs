//! Double clicks that forgive a slow hand.
//!
//! egui counts a click as the second of a double click only within 0.3 s of the first, and a
//! longer delay is no help: egui then calls every click after a double click a triple click,
//! which is no double click (see `style::click_tolerance`). Here the pairs are counted by the
//! program: two clicks on the same widget within [`DOUBLE_CLICK_SECONDS`] and a few pixels.

use egui::{Context, Id, Pos2, Response};

/// Longest time between the two clicks of a double click.
pub const DOUBLE_CLICK_SECONDS: f64 = 0.6;
/// Furthest the second click may be from the first, in points.
const DOUBLE_CLICK_DISTANCE: f32 = 10.0;

/// The previous click of a widget: time and place, until a double click uses it up.
type Last = (f64, Pos2);
/// The answer of a widget in one frame, so that asking twice in a frame is no second click.
type Answer = (u64, bool);

/// Whether the click at `pos` now is the second of a double click on `key`.
pub fn pair(ctx: &Context, key: Id, pos: Pos2) -> bool {
    let now = ctx.input(|i| i.time);
    let frame = ctx.cumulative_frame_nr();
    let cache = key.with("answer");
    if let Some((at, double)) = ctx.data(|d| d.get_temp::<Answer>(cache))
        && at == frame
    {
        return double;
    }
    let last = ctx.data(|d| d.get_temp::<Last>(key));
    let double = last.is_some_and(|(at, p)| {
        now - at <= DOUBLE_CLICK_SECONDS && p.distance(pos) <= DOUBLE_CLICK_DISTANCE
    });
    ctx.data_mut(|d| {
        if double {
            d.remove::<Last>(key);
        } else {
            d.insert_temp::<Last>(key, (now, pos));
        }
        d.insert_temp::<Answer>(cache, (frame, double));
    });
    double
}

pub trait TolerantDoubleClick {
    /// A double click with the primary button, also a slow one or one of a moving mouse.
    fn double_clicked_tolerant(&self) -> bool;
}

impl TolerantDoubleClick for Response {
    fn double_clicked_tolerant(&self) -> bool {
        if !self.clicked() {
            return false;
        }
        let pos = self.interact_pointer_pos().unwrap_or(self.rect.center());
        let counted = pair(&self.ctx, self.id.with("double"), pos);
        if self.double_clicked() && !counted {
            // egui saw it first: the next click starts a new pair.
            self.ctx
                .data_mut(|d| d.remove::<Last>(self.id.with("double")));
        }
        counted || self.double_clicked()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(ctx: &Context, time: f64, events: Vec<egui::Event>) -> bool {
        let input = egui::RawInput {
            time: Some(time),
            events,
            ..Default::default()
        };
        let mut double = false;
        let mut out = ctx.run_ui(input, |ui| {
            let r = ui.allocate_response(egui::vec2(100.0, 100.0), egui::Sense::click());
            double = r.double_clicked_tolerant();
        });
        out.textures_delta.clear();
        double
    }

    fn click(ctx: &Context, time: &mut f64, pos: Pos2) -> bool {
        let button = |pressed| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: Default::default(),
        };
        *time += 0.05;
        frame(
            ctx,
            *time,
            vec![egui::Event::PointerMoved(pos), button(true)],
        );
        *time += 0.05;
        frame(ctx, *time, vec![button(false)])
    }

    /// Pairs of clicks `gap` seconds apart, `pause` seconds between the pairs, the second click
    /// `shift` points right of the first; every pair must be a double click.
    fn pairs(gap: f64, pause: f64, shift: f32) {
        let ctx = Context::default();
        crate::style::click_tolerance(&ctx);
        let mut time = 1.0;
        frame(&ctx, time, vec![]);
        for pair in 0..6 {
            click(&ctx, &mut time, egui::pos2(20.0, 20.0));
            time += gap;
            frame(&ctx, time, vec![]);
            let second = click(&ctx, &mut time, egui::pos2(20.0 + shift, 20.0));
            assert!(
                second,
                "gap {gap}, pause {pause}, shift {shift}: pair {pair}"
            );
            time += pause;
            frame(&ctx, time, vec![]);
        }
    }

    #[test]
    fn a_slow_double_click_counts() {
        pairs(0.1, 0.7, 0.0);
        pairs(0.35, 0.7, 0.0);
        pairs(0.45, 0.7, 0.0);
    }

    #[test]
    fn a_moving_mouse_counts() {
        pairs(0.1, 0.7, 8.0);
        pairs(0.35, 0.7, 8.0);
    }

    #[test]
    fn pairs_in_a_row_count_every_time() {
        // A short pause: egui calls the clicks after the first pair triple clicks.
        pairs(0.2, 0.5, 0.0);
        pairs(0.35, 0.4, 0.0);
    }

    #[test]
    fn two_widgets_are_no_double_click() {
        let ctx = Context::default();
        let mut time = 1.0;
        frame(&ctx, time, vec![]);
        assert!(!click(&ctx, &mut time, egui::pos2(20.0, 20.0)));
        time += 0.2;
        assert!(!click(&ctx, &mut time, egui::pos2(60.0, 20.0)));
    }
}
