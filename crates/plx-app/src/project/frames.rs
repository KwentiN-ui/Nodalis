//! Frames, notes and the compact view of the project schematic, and its keys, like those of
//! Blender's node editor.
//!
//! A frame is a named, tinted rectangle around the blocks it holds; it follows them when
//! they move and is moved with them by dragging it. A compact block shows only its header
//! and the rows with a wire.

use super::*;

/// The tints a frame can take, by name for its menu: Blender's muted frame colours.
pub(super) const FRAME_COLORS: [(&str, [u8; 3]); 7] = [
    ("Gray", [140, 140, 146]),
    ("Red", [204, 88, 88]),
    ("Orange", [214, 142, 64]),
    ("Yellow", [204, 184, 70]),
    ("Green", [96, 172, 96]),
    ("Blue", [84, 136, 206]),
    ("Purple", [152, 108, 204]),
];

/// Space between a frame and the blocks in it.
const FRAME_MARGIN: f32 = 16.0;
/// Height of the band at the top of a frame that holds its label.
const FRAME_LABEL: f32 = 26.0;
/// Size of a block that was not drawn yet.
const UNDRAWN: egui::Vec2 = egui::vec2(220.0, 60.0);

/// The rectangle of `frame` in the schematic: around its blocks where they are now, each as
/// large as it was last drawn; `None` without blocks.
pub(super) fn frame_rect(
    frame: &file::Frame,
    graph: &Snarl<u64>,
    sizes: &HashMap<u64, egui::Vec2>,
    shown: &BTreeSet<u64>,
) -> Option<egui::Rect> {
    // Only the blocks of the group shown: a frame lies in one group.
    let blocks = (graph.nodes_pos_ids())
        .filter(|(_, _, id)| frame.nodes.contains(id) && shown.contains(id))
        .fold(egui::Rect::NOTHING, |all, (_, pos, id)| {
            let size = sizes.get(id).copied().unwrap_or(UNDRAWN);
            all.union(egui::Rect::from_min_size(pos, size))
        });
    (blocks.is_finite() && blocks.is_positive()).then(|| {
        let rect = blocks.expand(FRAME_MARGIN);
        egui::Rect::from_min_max(rect.min - egui::vec2(0.0, FRAME_LABEL), rect.max)
    })
}

/// Fill and outline of a frame of the tint `color`.
fn frame_colors(color: [u8; 3]) -> (egui::Color32, egui::Color32) {
    let [r, g, b] = color;
    let fill = if crate::style::dark() { 46 } else { 56 };
    (
        egui::Color32::from_rgba_unmultiplied(r, g, b, fill),
        egui::Color32::from_rgba_unmultiplied(r, g, b, 150),
    )
}

/// The frame the block `id` is in.
pub(super) fn frame_of(frames: &[file::Frame], id: u64) -> Option<usize> {
    frames.iter().position(|f| f.nodes.contains(&id))
}

impl Viewer {
    /// Draws the frames behind the blocks and takes their clicks and drags: they lie above
    /// the empty schematic and below the blocks.
    pub(super) fn frames(&mut self, painter: &egui::Painter, snarl: &Snarl<u64>) {
        if self.frames.is_empty() {
            return;
        }
        let mut ui = egui::Ui::new(
            painter.ctx().clone(),
            egui::Id::new("schematic frames"),
            egui::UiBuilder::new()
                .layer_id(painter.layer_id())
                .max_rect(painter.clip_rect()),
        );
        ui.set_clip_rect(painter.clip_rect());
        let text_color = ui.visuals().text_color();
        for index in 0..self.frames.len() {
            let Some(rect) = frame_rect(&self.frames[index], snarl, &self.sizes, &self.here) else {
                continue;
            };
            let frame = &self.frames[index];
            let (fill, stroke) = frame_colors(frame.color);
            painter.rect(
                rect,
                8.0,
                fill,
                egui::Stroke::new(1.0, stroke),
                egui::StrokeKind::Inside,
            );
            let response = ui.interact(
                rect,
                egui::Id::new(("schematic frame", index)),
                egui::Sense::click_and_drag(),
            );
            let label = egui::Rect::from_min_size(
                rect.min + egui::vec2(10.0, 4.0),
                egui::vec2((rect.width() - 20.0).max(40.0), FRAME_LABEL - 8.0),
            );
            let font = egui::FontId::proportional(15.0);
            match &mut self.frame_label {
                Some((editing, text)) if *editing == index => {
                    let edit = ui
                        .scope_builder(egui::UiBuilder::new().max_rect(label), |ui| {
                            egui::TextEdit::singleline(text)
                                .font(font)
                                .desired_width(label.width())
                                .show(ui)
                        })
                        .inner;
                    // The label takes the keyboard with all of it selected, to be typed over.
                    if !edit.response.has_focus() && !edit.response.lost_focus() {
                        edit.response.request_focus();
                        let mut state = edit.state;
                        let all = egui::text::CCursorRange::two(
                            egui::text::CCursor::new(0),
                            egui::text::CCursor::new(text.chars().count()),
                        );
                        state.cursor.set_char_range(Some(all));
                        state.store(ui.ctx(), edit.response.id);
                    }
                    let edit = edit.response;
                    let escape = ui.input(|i| i.key_pressed(egui::Key::Escape));
                    if edit.lost_focus() {
                        if !escape {
                            self.actions.push(Action::FrameLabel(index, text.clone()));
                        }
                        self.frame_label = None;
                    }
                }
                _ => {
                    painter
                        .with_clip_rect(label.intersect(painter.clip_rect()))
                        .text(
                            label.left_center(),
                            egui::Align2::LEFT_CENTER,
                            &frame.label,
                            font,
                            text_color,
                        );
                }
            }
            if response.dragged_by(egui::PointerButton::Primary) {
                self.actions
                    .push(Action::MoveFrame(index, response.drag_delta()));
            }
            if response.double_clicked() {
                let on_label = (response.interact_pointer_pos())
                    .is_some_and(|p| p.y < rect.top() + FRAME_LABEL);
                if on_label {
                    self.frame_label = Some((index, frame.label.clone()));
                } else if let Some(pos) = response.interact_pointer_pos() {
                    // As on the empty schematic: the blocks to add there.
                    self.actions.push(Action::Pick(self.to_global * pos, pos));
                }
            }
            let color = frame.color;
            let label_text = frame.label.clone();
            let on_label = (response.hover_pos()).is_some_and(|p| p.y < rect.top() + FRAME_LABEL);
            let response = if on_label {
                response.on_hover_text_at_pointer(
                    "Drag to move the frame with its blocks. Double-click its label to rename \
                     it; right-click for its colour.",
                )
            } else {
                response
            };
            response.context_menu(|ui| {
                if ui.button("Rename").clicked() {
                    self.frame_label = Some((index, label_text.clone()));
                    ui.close();
                }
                ui.menu_button("Color", |ui| {
                    for (name, tint) in FRAME_COLORS {
                        let (_, stroke) = frame_colors(tint);
                        let fill = egui::Color32::from_rgb(tint[0], tint[1], tint[2]);
                        let selected = tint == color;
                        let row = ui.horizontal(|ui| {
                            let (swatch, _) = ui
                                .allocate_exact_size(egui::vec2(14.0, 14.0), egui::Sense::hover());
                            ui.painter().rect(
                                swatch,
                                3.0,
                                fill,
                                egui::Stroke::new(1.0, stroke),
                                egui::StrokeKind::Inside,
                            );
                            ui.selectable_label(selected, name)
                        });
                        if row.inner.clicked() {
                            self.actions.push(Action::FrameColor(index, tint));
                            ui.close();
                        }
                    }
                });
                ui.separator();
                let tip = "Removes the frame; its blocks stay where they are.";
                if ui.button("Remove Frame").on_hover_text(tip).clicked() {
                    self.actions.push(Action::RemoveFrame(index));
                    ui.close();
                }
            });
        }
    }
}

impl ProjectApp {
    /// Puts `ids` in a new frame around them, taking them out of the frames they were in,
    /// and starts editing its label.
    pub(super) fn frame_blocks(&mut self, ids: &[u64]) {
        if ids.is_empty() {
            return;
        }
        self.leave_frames(ids);
        self.frames.push(file::Frame {
            label: "Frame".into(),
            color: FRAME_COLORS[0].1,
            nodes: ids.to_vec(),
        });
        self.frame_label = Some((self.frames.len() - 1, "Frame".into()));
    }

    /// Takes `ids` out of their frames; a frame left without blocks is removed.
    pub(super) fn leave_frames(&mut self, ids: &[u64]) {
        for frame in &mut self.frames {
            frame.nodes.retain(|n| !ids.contains(n));
        }
        self.frames.retain(|f| !f.nodes.is_empty());
        self.frame_label = None;
    }

    /// Shows `ids` compact, or all of them in full if they are all compact.
    pub(super) fn toggle_compact(&mut self, ids: &[u64]) {
        if ids.iter().all(|id| self.compact.contains(id)) {
            for id in ids {
                self.compact.remove(id);
            }
        } else {
            self.compact.extend(ids.iter().copied());
        }
    }

    /// Moves the blocks of a frame by `delta`.
    pub(super) fn move_frame(&mut self, index: usize, delta: egui::Vec2) {
        let Some(frame) = self.frames.get(index) else {
            return;
        };
        for (_, info) in self.graph.nodes_ids_data_mut() {
            if frame.nodes.contains(&info.value) {
                info.pos += delta;
            }
        }
    }

    /// Blocks dropped onto a frame after a drag join it, unless they are in one already.
    pub(super) fn join_dropped(&mut self, ctx: &egui::Context) {
        let positions = self.positions();
        let (down, released) = ctx.input(|i| (i.pointer.any_down(), i.pointer.any_released()));
        if down {
            let moved = (positions.iter())
                .filter(|(id, pos)| {
                    (self.last_positions.iter()).any(|(other, old)| other == id && old != pos)
                })
                .map(|(id, _)| *id);
            self.moving.extend(moved);
        }
        self.last_positions = positions;
        if !released || self.moving.is_empty() {
            return;
        }
        let moving = std::mem::take(&mut self.moving);
        let mut joins = Vec::new();
        for id in moving {
            if frame_of(&self.frames, id).is_some() {
                continue;
            }
            let Some(center) = (self.graph_node(id))
                .and_then(|g| self.graph.get_node_info(g))
                .map(|info| {
                    let size = self.block_sizes.get(&id).copied().unwrap_or(UNDRAWN);
                    info.pos + size / 2.0
                })
            else {
                continue;
            };
            let frame = (self.frames.iter()).position(|frame| {
                frame_rect(frame, &self.graph, &self.block_sizes, &self.shown)
                    .is_some_and(|rect| rect.contains(center))
            });
            if let Some(frame) = frame {
                joins.push((frame, id));
            }
        }
        if !joins.is_empty() {
            self.checkpoint(&[]);
            for (frame, id) in joins {
                self.frames[frame].nodes.push(id);
            }
        }
    }

    /// The keys of the schematic, as in Blender's node editor, while the pointer is over it
    /// and no text takes the keyboard: Shift+A adds a block, X and Delete delete, Shift+D
    /// duplicates, H shows compact, Ctrl+J frames, Alt+P takes out of the frame and F2
    /// renames; on the selected blocks, or the one under the pointer.
    pub(super) fn schematic_keys(
        &mut self,
        ui: &egui::Ui,
        area: egui::Rect,
        to_graph: egui::emath::TSTransform,
    ) {
        let ctx = ui.ctx().clone();
        let Some(pointer) = ctx.input(|i| i.pointer.hover_pos()) else {
            return;
        };
        let over = area.contains(pointer)
            && (ctx.layer_id_at(pointer)).is_none_or(|l| l.order == egui::Order::Background);
        let typing = ctx.memory(|m| m.focused().is_some());
        if !over || typing || self.picker.is_some() {
            return;
        }
        let graph = to_graph * pointer;
        let hovered = (self.graph.nodes_pos_ids())
            .filter(|(_, _, id)| self.shown.contains(id))
            .find_map(|(_, pos, id)| {
                let size = self.block_sizes.get(id).copied().unwrap_or(UNDRAWN);
                egui::Rect::from_min_size(pos, size)
                    .contains(graph)
                    .then_some(*id)
            });
        let targets = match hovered {
            Some(_) if !self.selected.is_empty() => self.selected.clone(),
            Some(id) => vec![id],
            None => self.selected.clone(),
        };
        let key = |modifiers: egui::Modifiers, key: egui::Key| {
            ctx.input_mut(|i| i.consume_key(modifiers, key))
        };
        let shift = egui::Modifiers::SHIFT;
        let none = egui::Modifiers::NONE;
        if key(shift, egui::Key::A) {
            self.picker = Some(NodePicker::new(pointer, graph));
        } else if key(shift, egui::Key::D) {
            self.act(&ctx, Action::DuplicateAll(targets));
        } else if key(none, egui::Key::X) || key(none, egui::Key::Delete) {
            self.act(&ctx, Action::DeleteAll(targets));
        } else if key(none, egui::Key::H) {
            self.act(&ctx, Action::Compact(targets));
        } else if key(
            egui::Modifiers::COMMAND | egui::Modifiers::ALT,
            egui::Key::G,
        ) {
            let groups: Vec<u64> = (targets.iter().copied())
                .filter(|id| {
                    self.nodes
                        .iter()
                        .any(|n| n.id == *id && n.kind() == Kind::Group)
                })
                .collect();
            self.act(&ctx, Action::Ungroup(groups));
        } else if key(egui::Modifiers::COMMAND, egui::Key::G) {
            self.act(&ctx, Action::Group(targets));
        } else if key(egui::Modifiers::COMMAND, egui::Key::Tab) {
            self.act(&ctx, Action::LeaveGroup);
        } else if key(none, egui::Key::Tab) {
            // Tab opens the group under the pointer or selected, else goes out, as in
            // Blender.
            let group = match targets[..] {
                [id] => Some(id),
                _ => hovered,
            }
            .filter(|id| {
                self.nodes
                    .iter()
                    .any(|n| n.id == *id && n.kind() == Kind::Group)
            });
            match group {
                Some(id) => self.act(&ctx, Action::EnterGroup(id)),
                None => self.act(&ctx, Action::LeaveGroup),
            }
        } else if key(egui::Modifiers::COMMAND, egui::Key::J) {
            self.act(&ctx, Action::Frame(targets));
        } else if key(egui::Modifiers::ALT, egui::Key::P) {
            self.act(&ctx, Action::Unframe(targets));
        } else if key(none, egui::Key::F2)
            && let [id] = targets[..]
        {
            self.act(&ctx, Action::Rename(id));
        }
    }
}
