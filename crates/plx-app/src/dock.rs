//! The properties panel on the right of the analysis workspaces. It holds exactly one
//! editor: the dialog of the item being created or edited, or the properties of an item.
//! The panel is laid out before the 3D view; the editor draws into it later in the frame,
//! where its dialog used to open a window of its own. Without the panel, for example in a
//! test, the editor falls back to that window.

use egui::{Context, Id, InnerResponse, Rect, Ui, WidgetText};

/// The default width of the panel.
const WIDTH: f32 = 440.0;

/// The editors that live in the panel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Docked {
    /// The properties of a tree item.
    Properties,
    /// An item of the FE model: material, section, step, boundary condition, load, ...
    Item,
    MeshItem,
    Feature,
    FieldOutput,
    HistoryOutput,
    HotSpot,
    ModelProperties,
}

/// Keeps the open editors in the order they were opened. Returns those to close, so that
/// only the newest stays open.
pub fn one_open(order: &mut Vec<Docked>, open: &[Docked]) -> Vec<Docked> {
    order.retain(|d| open.contains(d));
    for d in open {
        if !order.contains(d) {
            order.push(*d);
        }
    }
    let keep = order.len().saturating_sub(1);
    order.drain(..keep).collect()
}

fn key(ctx: &Context) -> Id {
    crate::ids::window(ctx, "properties panel")
}

/// What the panel shows while no editor is open: a title and its content.
pub type Content<'a> = (&'a str, &'a mut dyn FnMut(&mut Ui));

/// Lays out the panel on the right of `ui`; call it before the central panel. An editor
/// draws into it later in the frame; without one, `empty` fills it under its title, and the
/// return value tells whether its close button was clicked.
pub fn panel(ui: &mut Ui, empty: Option<Content<'_>>) -> bool {
    let ctx = ui.ctx().clone();
    let mut closed = false;
    egui::Panel::right("properties")
        .resizable(true)
        .default_size(WIDTH)
        .size_range(260.0..=900.0)
        .frame(
            egui::Frame::new()
                .fill(crate::style::window())
                .stroke(egui::Stroke::new(1.0, crate::style::border()))
                .inner_margin(8),
        )
        .show(ui, |ui| {
            if let Some((title, add)) = empty {
                closed = head(ui, title);
                egui::ScrollArea::vertical()
                    .auto_shrink(false)
                    .show(ui, |ui| add(ui));
                return;
            }
            let rect = ui.max_rect();
            ui.allocate_rect(rect, egui::Sense::hover());
            // The key first: the context is locked while its data is borrowed.
            let (pass, key) = (ctx.cumulative_pass_nr(), key(&ctx));
            ctx.data_mut(|d| d.insert_temp(key, (pass, rect)));
        });
    closed
}

/// The head of the panel: its title, a close button and a line. Returns whether the
/// button was clicked.
fn head(ui: &mut Ui, title: impl Into<WidgetText>) -> bool {
    let mut close = false;
    ui.horizontal(|ui| {
        ui.label(title.into().strong());
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            close = close_button(ui).on_hover_text("Close").clicked();
        });
    });
    ui.separator();
    close
}

/// The inside of the panel, when it was laid out in this frame.
fn rect(ctx: &Context) -> Option<Rect> {
    let (pass, key) = (ctx.cumulative_pass_nr(), key(ctx));
    ctx.data(|d| d.get_temp::<(u64, Rect)>(key))
        .filter(|(p, _)| *p == pass)
        .map(|(_, rect)| rect)
}

/// Shows an editor in the panel under `title`, with a close button that clears `open`; or,
/// without the panel, in `window`.
pub fn show<R>(
    ctx: &Context,
    window: egui::Window<'_>,
    title: impl Into<WidgetText>,
    open: &mut bool,
    add: impl FnOnce(&mut Ui) -> R,
) -> Option<InnerResponse<Option<R>>> {
    let Some(rect) = rect(ctx) else {
        return window.open(open).show(ctx, add);
    };
    let area = egui::Area::new(crate::ids::window(ctx, "properties panel editor"))
        .fixed_pos(rect.min)
        .order(egui::Order::Middle)
        .constrain(false)
        .show(ctx, |ui| {
            ui.set_min_size(rect.size());
            ui.set_max_size(rect.size());
            if head(ui, title) {
                *open = false;
            }
            // Vertical only: layouts from the right need a bounded width.
            egui::ScrollArea::vertical()
                .auto_shrink(false)
                .show(ui, |ui| {
                    ui.set_max_width(ui.available_width());
                    add(ui)
                })
                .inner
        });
    Some(InnerResponse::new(Some(area.inner), area.response))
}

/// The buttons at the foot of an editor, from the right: in the panel at its right edge,
/// even when a long hint makes the editor wider than the panel.
pub fn footer<R>(ui: &mut Ui, add: impl FnOnce(&mut Ui) -> R) -> InnerResponse<R> {
    let width = match rect(ui.ctx()) {
        Some(rect) if ui.max_rect().right() > rect.right() => {
            (rect.right() - ui.cursor().left()).max(0.0)
        }
        _ => ui.available_width(),
    };
    let layout = egui::Layout::right_to_left(egui::Align::Min);
    ui.allocate_ui_with_layout(egui::vec2(width, 0.0), layout, add)
}

/// A cross drawn with the painter.
fn close_button(ui: &mut Ui) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(egui::vec2(18.0, 18.0), egui::Sense::click());
    let color = if response.hovered() {
        ui.visuals().strong_text_color()
    } else {
        ui.visuals().weak_text_color()
    };
    let r = rect.shrink(5.0);
    let stroke = egui::Stroke::new(1.5, color);
    ui.painter()
        .line_segment([r.left_top(), r.right_bottom()], stroke);
    ui.painter()
        .line_segment([r.right_top(), r.left_bottom()], stroke);
    response
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_the_newest_editor_stays_open() {
        let mut order = Vec::new();
        assert!(one_open(&mut order, &[Docked::Item]).is_empty());
        // Opening the properties of an item closes the item editor.
        let closed = one_open(&mut order, &[Docked::Item, Docked::Properties]);
        assert_eq!(closed, vec![Docked::Item]);
        assert_eq!(order, vec![Docked::Properties]);
        // Closed by the user: nothing left.
        assert!(one_open(&mut order, &[]).is_empty());
        assert!(order.is_empty());
    }
}
