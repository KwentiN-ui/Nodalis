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
///
/// In a narrow window (below [`crate::layout::OVERLAY_WIDTH`]) the panel takes no room: it
/// lies over the right edge of what is left of `ui`, so call it after the output pane then.
pub fn panel(ui: &mut Ui, empty: Option<Content<'_>>) -> bool {
    let ctx = ui.ctx().clone();
    let mut closed = false;
    if crate::layout::current(&ctx).overlay {
        let room = ui.available_rect_before_wrap();
        let width = WIDTH.min(room.width() - 60.0).max(220.0);
        let outer = Rect::from_min_max(egui::pos2(room.right() - width, room.top()), room.max);
        let inner = outer.shrink(8.0);
        let over = crate::ids::window(&ctx, "properties panel over");
        let pass = ctx.cumulative_pass_nr();
        ctx.data_mut(|d| d.insert_temp(over, (pass, outer)));
        let Some((title, add)) = empty else {
            let (pass, key) = (ctx.cumulative_pass_nr(), key(&ctx));
            ctx.data_mut(|d| d.insert_temp(key, (pass, inner, true)));
            return false;
        };
        egui::Area::new(crate::ids::window(&ctx, "properties panel overlay"))
            .fixed_pos(outer.min)
            .order(egui::Order::Middle)
            .constrain(false)
            .show(&ctx, |ui| {
                overlay_frame(ui, outer);
                let mut child = ui.new_child(egui::UiBuilder::new().max_rect(inner));
                closed = head(&mut child, title);
                egui::ScrollArea::vertical()
                    .auto_shrink(false)
                    .show(&mut child, |ui| add(ui));
                ui.allocate_rect(outer, egui::Sense::hover());
            });
        return closed;
    }
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
            ctx.data_mut(|d| d.insert_temp(key, (pass, rect, false)));
        });
    closed
}

/// Where the panel lies over the 3D view in this frame, if it does.
pub fn over_view(ctx: &Context) -> Option<Rect> {
    let (pass, key) = (
        ctx.cumulative_pass_nr(),
        crate::ids::window(ctx, "properties panel over"),
    );
    ctx.data(|d| d.get_temp::<(u64, Rect)>(key))
        .filter(|(p, _)| *p == pass)
        .map(|(_, rect)| rect)
}

/// The background of the panel lying over the 3D view, with a shadow to its left.
fn overlay_frame(ui: &Ui, outer: Rect) {
    let shadow = egui::Shadow {
        offset: [-4, 0],
        blur: 12,
        spread: 0,
        color: egui::Color32::from_black_alpha(90),
    };
    ui.painter().add(shadow.as_shape(outer, 0.0));
    ui.painter().rect(
        outer,
        0.0,
        crate::style::window(),
        egui::Stroke::new(1.0, crate::style::border()),
        egui::StrokeKind::Inside,
    );
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
    place(ctx).map(|(rect, _)| rect)
}

/// The inside of the panel and whether it lies over the 3D view.
fn place(ctx: &Context) -> Option<(Rect, bool)> {
    let (pass, key) = (ctx.cumulative_pass_nr(), key(ctx));
    ctx.data(|d| d.get_temp::<(u64, Rect, bool)>(key))
        .filter(|(p, ..)| *p == pass)
        .map(|(_, rect, overlay)| (rect, overlay))
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
    let Some((rect, overlay)) = place(ctx) else {
        return window.open(open).show(ctx, add);
    };
    let area = egui::Area::new(crate::ids::window(ctx, "properties panel editor"))
        .fixed_pos(rect.min)
        .order(egui::Order::Middle)
        .constrain(false)
        .show(ctx, |ui| {
            if overlay {
                overlay_frame(ui, rect.expand(8.0));
            }
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

    /// One frame of a window `width` points wide with the panel and an editor in it;
    /// returns where the editor drew and where the panel lies over the view.
    fn frame(ctx: &Context, width: f32) -> (Option<Rect>, Option<Rect>) {
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(width, 800.0),
            )),
            ..Default::default()
        };
        let mut drawn = None;
        let mut over = None;
        let mut output = ctx.run_ui(input, |ui| {
            panel(ui, None);
            egui::CentralPanel::default().show(ui, |_| {});
            let mut open = true;
            let id = crate::ids::window(ui.ctx(), "test editor");
            let window = egui::Window::new("Editor").id(id);
            let shown = show(ui.ctx(), window, "Editor", &mut open, |ui| ui.label("x"));
            drawn = shown.map(|r| r.response.rect);
            over = over_view(ui.ctx());
        });
        output.textures_delta.clear();
        (drawn, over)
    }

    #[test]
    fn a_narrow_window_lays_the_panel_over_the_view() {
        let ctx = Context::default();
        // Wide: the panel takes its room and the view keeps the rest.
        let (drawn, over) = frame(&ctx, 1600.0);
        assert!(drawn.is_some_and(|r| r.left() > 1000.0), "{drawn:?}");
        assert_eq!(over, None);
        // Narrow: over the right edge of the view.
        let (drawn, over) = frame(&ctx, 1000.0);
        let over = over.expect("lies over the view");
        assert!(over.right() >= 999.0 && over.width() <= WIDTH, "{over:?}");
        assert!(drawn.is_some_and(|r| over.contains(r.min)), "{drawn:?}");
    }

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
