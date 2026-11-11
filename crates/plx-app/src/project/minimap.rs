//! The minimap of the project schematic: all blocks and frames in small in a corner, with the
//! part the view shows; a click or a drag in it moves the view there.

use super::*;

/// Size of the minimap and its distance from the corner of the schematic.
const SIZE: egui::Vec2 = egui::vec2(220.0, 150.0);
const MARGIN: f32 = 12.0;
/// Space around the blocks in the minimap, in the schematic.
const PADDING: f32 = 60.0;

/// What the minimap shows: the rectangles of the blocks with the tint of their header, those
/// of the frames with theirs, and the part of the schematic in view.
pub(super) struct MapContent {
    pub(super) blocks: Vec<(egui::Rect, egui::Color32)>,
    pub(super) frames: Vec<(egui::Rect, [u8; 3])>,
    pub(super) view: egui::Rect,
}

impl MapContent {
    /// The part of the schematic the minimap spans: all blocks and the view.
    pub(super) fn bounds(&self) -> egui::Rect {
        (self.blocks.iter().map(|(r, _)| *r))
            .chain(self.frames.iter().map(|(r, _)| *r))
            .fold(self.view, egui::Rect::union)
            .expand(PADDING)
    }
}

/// From the schematic to the minimap in `inner`: the bounds fit it, keeping their aspect.
pub(super) fn map_transform(bounds: egui::Rect, inner: egui::Rect) -> egui::emath::TSTransform {
    let scale = (inner.width() / bounds.width()).min(inner.height() / bounds.height());
    egui::emath::TSTransform::new(
        inner.center().to_vec2() - bounds.center().to_vec2() * scale,
        scale,
    )
}

impl ProjectApp {
    /// What the minimap shows with the view `view` of the schematic.
    fn map_content(&self, view: egui::Rect) -> MapContent {
        let blocks = (self.graph.nodes_pos_ids())
            .filter(|(_, _, id)| self.shown.contains(id))
            .filter_map(|(_, pos, id)| {
                let node = self.nodes.iter().find(|n| n.id == *id)?;
                let size = (self.block_sizes.get(id).copied()).unwrap_or(egui::vec2(220.0, 60.0));
                let fill = header_fill(node.kind()).unwrap_or_else(crate::style::node_header);
                Some((egui::Rect::from_min_size(pos, size), fill))
            })
            .collect();
        let frames = (self.frames.iter())
            .filter_map(|f| {
                Some((
                    frames::frame_rect(f, &self.graph, &self.block_sizes, &self.shown)?,
                    f.color,
                ))
            })
            .collect();
        MapContent {
            blocks,
            frames,
            view,
        }
    }

    /// Draws the minimap in the lower right corner of the schematic shown in `schematic`, with
    /// `to_global` from the schematic to the screen; a click or a drag in it centres the view
    /// on that point from the next frame on.
    pub(super) fn minimap(
        &mut self,
        ctx: &egui::Context,
        schematic: egui::Rect,
        to_global: egui::emath::TSTransform,
    ) {
        if !self.show_minimap || self.nodes.is_empty() {
            return;
        }
        let content = self.map_content(to_global.inverse() * schematic);
        let rect =
            egui::Rect::from_min_size(schematic.max - SIZE - egui::vec2(MARGIN, MARGIN), SIZE);
        if !schematic.contains_rect(rect) {
            return;
        }
        let mut look_at = None;
        let mut held = None;
        egui::Area::new(crate::ids::window(ctx, "project minimap"))
            .order(egui::Order::Foreground)
            .fixed_pos(rect.min)
            .show(ctx, |ui| {
                let (rect, response) = ui.allocate_exact_size(SIZE, egui::Sense::click_and_drag());
                let painter = ui.painter_at(rect);
                painter.rect(
                    rect,
                    4.0,
                    crate::style::window(),
                    egui::Stroke::new(1.0, crate::style::border()),
                    egui::StrokeKind::Inside,
                );
                let bounds = self.minimap_bounds.unwrap_or_else(|| content.bounds());
                let map = map_transform(bounds, rect.shrink(6.0));
                for (frame, color) in &content.frames {
                    let [r, g, b] = *color;
                    painter.rect_filled(
                        map * *frame,
                        2.0,
                        egui::Color32::from_rgba_unmultiplied(r, g, b, 70),
                    );
                }
                for (block, fill) in &content.blocks {
                    painter.rect(
                        map * *block,
                        1.5,
                        *fill,
                        egui::Stroke::new(1.0, crate::style::border()),
                        egui::StrokeKind::Inside,
                    );
                }
                painter.rect_stroke(
                    map * content.view,
                    1.0,
                    egui::Stroke::new(1.5, crate::style::highlight()),
                    egui::StrokeKind::Inside,
                );
                let pressed = response.is_pointer_button_down_on() || response.dragged();
                if (pressed || response.clicked())
                    && let Some(pointer) = response.interact_pointer_pos()
                {
                    look_at = Some(map.inverse() * pointer);
                }
                held = pressed.then_some(bounds);
                response.on_hover_text(
                    "All blocks of the project; the outline is the part in view. Click or \
                     drag here to move the view.",
                );
            });
        self.minimap_bounds = held;
        if look_at.is_some() {
            self.look_at = look_at;
            ctx.request_repaint();
        }
    }
}
