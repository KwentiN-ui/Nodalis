//! Tables that lay out only their visible rows, for result lists with thousands of entries.

use egui::{Align2, Sense, TextStyle, Ui, vec2};

/// Gap between two columns.
const COLUMN_GAP: f32 = 16.0;
/// Space above and below the text of a row.
const ROW_PADDING: f32 = 2.0;

/// A striped table with a fixed header. The rows are painted only where the scroll area shows
/// them, so a table of 10000 rows costs as much per frame as one of 20. Every column is as wide
/// as its header or `sample`, the widest text a cell is expected to hold.
pub fn show(
    ui: &mut Ui,
    id: &str,
    headers: &[&str],
    sample: &str,
    rows: usize,
    max_height: f32,
    cell: impl Fn(usize, usize) -> String,
) {
    let font = TextStyle::Body.resolve(ui.style());
    let visuals = ui.visuals().clone();
    let width = |ui: &Ui, text: &str| {
        let galley =
            ui.fonts_mut(|f| f.layout_no_wrap(text.into(), font.clone(), visuals.text_color()));
        galley.size().x
    };
    let sample_width = width(ui, sample);
    let widths: Vec<f32> = (headers.iter())
        .map(|h| width(ui, h).max(sample_width) + COLUMN_GAP)
        .collect();
    let total = widths.iter().sum::<f32>();
    let row_height = ui.fonts_mut(|f| f.row_height(&font)) + 2.0 * ROW_PADDING;
    let paint_row = |ui: &mut Ui, texts: &mut dyn Iterator<Item = String>, color, stripe: bool| {
        let (rect, _) = ui.allocate_exact_size(vec2(total, row_height), Sense::hover());
        if !ui.is_rect_visible(rect) {
            return;
        }
        let painter = ui.painter();
        if stripe {
            painter.rect_filled(rect, 0.0, visuals.faint_bg_color);
        }
        let mut x = rect.left();
        for (text, w) in texts.zip(&widths) {
            painter.text(
                egui::pos2(x, rect.center().y),
                Align2::LEFT_CENTER,
                text,
                font.clone(),
                color,
            );
            x += w;
        }
    };
    ui.push_id(id, |ui| {
        ui.spacing_mut().item_spacing.y = 0.0;
        let strong = visuals.strong_text_color();
        paint_row(
            ui,
            &mut headers.iter().map(|h| h.to_string()),
            strong,
            false,
        );
        egui::ScrollArea::vertical()
            .max_height(max_height)
            .auto_shrink([false, true])
            .show_rows(ui, row_height, rows, |ui, range| {
                for row in range {
                    let mut texts = (0..headers.len()).map(|column| cell(row, column));
                    paint_row(ui, &mut texts, visuals.text_color(), row % 2 == 1);
                }
            });
    });
}

/// Monospace text lines in `area`, laid out only where they are visible. The solver output of a
/// long run has many thousands of lines.
///
/// The view keeps its own text selection over all lines, so a selection dragged beyond the
/// visible rows scrolls along and copies every line it covers, not only the ones on screen.
/// Ctrl+A selects all lines, Ctrl+C copies the selection, and the context menu copies either.
pub fn lines<S: AsRef<str>>(ui: &mut Ui, area: egui::ScrollArea, lines: &[S]) {
    let font = TextStyle::Monospace.resolve(ui.style());
    let row_height = ui.fonts_mut(|f| f.row_height(&font));
    let text_color = ui.visuals().text_color();
    let selection_color = ui.visuals().selection.bg_fill;
    let id = ui.make_persistent_id("text lines");
    let count = lines.len();
    let mut selection = ui
        .data(|d| d.get_temp::<LineSelection>(id))
        .filter(|s| s.lines == count && s.end().line < count);
    let layout = |ui: &Ui, line: usize| {
        let text = lines[line].as_ref().to_owned();
        ui.fonts_mut(|f| f.layout_no_wrap(text, font.clone(), text_color))
    };
    // The widest line seen so far: the visible lines alone would make the width jump.
    let width_id = id.with("width");
    let mut width = ui.data(|d| d.get_temp::<f32>(width_id)).unwrap_or(0.0);
    let mut copy = None;
    area.show_viewport(ui, |ui, viewport| {
        let top = ui.max_rect().top();
        let left = ui.max_rect().left();
        let first = ((viewport.min.y / row_height).floor().max(0.0) as usize).min(count);
        let last = ((viewport.max.y / row_height).ceil().max(0.0) as usize + 1).min(count);
        let mut galleys = Vec::with_capacity(last - first);
        for line in first..last {
            let galley = layout(ui, line);
            width = width.max(galley.size().x);
            galleys.push(galley);
        }
        let content = egui::Rect::from_min_size(
            egui::pos2(left, top),
            vec2(width.max(ui.max_rect().width()), row_height * count as f32),
        );
        ui.allocate_rect(content, Sense::hover());
        let response = ui.interact(content, id, Sense::click_and_drag());
        if response.hovered() || response.dragged() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Text);
        }

        // Where in the text a point lies; points above or below the lines clamp to the ends.
        let position = |ui: &Ui, pos: egui::Pos2| -> TextPosition {
            if count == 0 {
                return TextPosition::default();
            }
            let row = ((pos.y - top) / row_height).floor();
            if row < 0.0 {
                return TextPosition { line: 0, column: 0 };
            }
            let line = (row as usize).min(count - 1);
            if row as usize >= count {
                return TextPosition {
                    line,
                    column: lines[line].as_ref().chars().count(),
                };
            }
            let galley = match line.checked_sub(first).and_then(|i| galleys.get(i)) {
                Some(galley) => galley.clone(),
                None => layout(ui, line),
            };
            let column = galley
                .cursor_from_pos(vec2(pos.x - left, row_height / 2.0))
                .index
                .into();
            TextPosition { line, column }
        };

        if response.drag_started_by(egui::PointerButton::Primary) || response.clicked() {
            response.request_focus();
            if let Some(pos) = response.interact_pointer_pos() {
                let at = position(ui, pos);
                let anchor = match (ui.input(|i| i.modifiers.shift), &selection) {
                    (true, Some(s)) => s.anchor,
                    _ => at,
                };
                selection = Some(LineSelection {
                    anchor,
                    cursor: at,
                    lines: count,
                });
            }
        }
        if response.double_clicked()
            && let Some(pos) = response.interact_pointer_pos()
        {
            let line = position(ui, pos).line;
            let end = lines.get(line).map_or(0, |l| l.as_ref().chars().count());
            selection = Some(LineSelection {
                anchor: TextPosition { line, column: 0 },
                cursor: TextPosition { line, column: end },
                lines: count,
            });
        }
        if response.dragged_by(egui::PointerButton::Primary)
            && let (Some(pos), Some(s)) = (response.interact_pointer_pos(), &mut selection)
        {
            s.cursor = position(ui, pos);
            // Past the edge of the view the lines scroll along, the farther the faster.
            let clip = ui.clip_rect();
            let beyond = if pos.y < clip.top() {
                clip.top() - pos.y
            } else if pos.y > clip.bottom() {
                clip.bottom() - pos.y
            } else {
                0.0
            };
            if beyond != 0.0 {
                let step =
                    beyond.signum() * (beyond.abs() * 0.5).clamp(row_height, 40.0 * row_height);
                ui.scroll_with_delta_animation(
                    vec2(0.0, step),
                    egui::style::ScrollAnimation::none(),
                );
                ui.ctx().request_repaint();
            }
        }

        if response.has_focus() {
            let (all, copied) = ui.input(|i| {
                // The modifiers of the event itself: a quick Ctrl+A is over within one frame.
                let all = i.events.iter().any(|e| {
                    matches!(e, egui::Event::Key { key: egui::Key::A, pressed: true, modifiers, .. }
                        if modifiers.command)
                });
                let copied = i.events.iter().any(|e| matches!(e, egui::Event::Copy));
                (all, copied)
            });
            if all && count > 0 {
                selection = Some(LineSelection::all(lines));
            }
            if copied {
                copy = selection.as_ref().map(|s| s.text(lines));
            }
        }
        response.context_menu(|ui| {
            let has = selection.as_ref().is_some_and(|s| !s.is_empty());
            if ui.add_enabled(has, egui::Button::new("Copy")).clicked() {
                copy = selection.as_ref().map(|s| s.text(lines));
                ui.close();
            }
            if ui
                .add_enabled(count > 0, egui::Button::new("Copy All"))
                .clicked()
            {
                copy = Some(LineSelection::all(lines).text(lines));
                ui.close();
            }
            if ui
                .add_enabled(count > 0, egui::Button::new("Select All"))
                .clicked()
            {
                selection = Some(LineSelection::all(lines));
                ui.close();
            }
        });

        let painter = ui.painter();
        for (galley, line) in galleys.into_iter().zip(first..last) {
            let origin = egui::pos2(left, top + line as f32 * row_height);
            if let Some((from, to)) = selection.as_ref().and_then(|s| s.columns(line, lines)) {
                let x = |column| {
                    galley
                        .pos_from_cursor(egui::text::CCursor::new(column))
                        .left()
                };
                let (x0, mut x1) = (x(from), x(to));
                // An empty part of a selected line break still shows that the break is in it.
                if selection.as_ref().is_some_and(|s| s.end().line > line) {
                    x1 = x1.max(x0) + row_height / 3.0;
                }
                if x1 > x0 {
                    let rect = egui::Rect::from_x_y_ranges(
                        origin.x + x0..=origin.x + x1,
                        origin.y..=origin.y + row_height,
                    );
                    painter.rect_filled(rect, 0.0, selection_color);
                }
            }
            painter.galley(origin, galley, text_color);
        }
    });
    if let Some(text) = copy {
        ui.ctx().copy_text(text);
    }
    ui.data_mut(|d| {
        d.insert_temp(width_id, width);
        match selection {
            Some(s) => {
                d.insert_temp(id, s);
            }
            None => d.remove::<LineSelection>(id),
        }
    });
}

/// The button that copies all `lines` containing `filter` (ignoring case), the whole output when
/// the filter is empty, also the lines scrolled out of view.
pub fn copy_button<S: AsRef<str>>(ui: &mut Ui, lines: &[S], filter: &str) {
    let tip = match filter.is_empty() {
        true => "Copy the whole output to the clipboard",
        false => "Copy all lines the filter shows to the clipboard",
    };
    let button = ui.add_enabled(!lines.is_empty(), egui::Button::new("Copy"));
    if button.on_hover_text(tip).clicked() {
        let needle = filter.to_lowercase();
        let shown: Vec<&str> = (lines.iter().map(|l| l.as_ref()))
            .filter(|line| needle.is_empty() || line.to_lowercase().contains(&needle))
            .collect();
        ui.ctx().copy_text(shown.join("\n"));
    }
}

/// A place in a list of text lines: the line and the character in it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
struct TextPosition {
    line: usize,
    column: usize,
}

/// The selected text of [`lines`], from where the drag started to where it is now.
#[derive(Clone, Copy, Debug)]
struct LineSelection {
    anchor: TextPosition,
    cursor: TextPosition,
    /// The number of lines the selection was made in: a filter that changes them ends it.
    lines: usize,
}

impl LineSelection {
    fn all<S: AsRef<str>>(lines: &[S]) -> Self {
        let line = lines.len().saturating_sub(1);
        let column = lines.last().map_or(0, |l| l.as_ref().chars().count());
        Self {
            anchor: TextPosition::default(),
            cursor: TextPosition { line, column },
            lines: lines.len(),
        }
    }

    fn start(&self) -> TextPosition {
        self.anchor.min(self.cursor)
    }

    fn end(&self) -> TextPosition {
        self.anchor.max(self.cursor)
    }

    fn is_empty(&self) -> bool {
        self.anchor == self.cursor
    }

    /// The selected characters of `line`, if any of it is selected.
    fn columns<S: AsRef<str>>(&self, line: usize, lines: &[S]) -> Option<(usize, usize)> {
        let (start, end) = (self.start(), self.end());
        if self.is_empty() || line < start.line || line > end.line {
            return None;
        }
        let length = lines[line].as_ref().chars().count();
        let from = if line == start.line {
            start.column.min(length)
        } else {
            0
        };
        let to = if line == end.line {
            end.column.min(length)
        } else {
            length
        };
        Some((from, to))
    }

    /// The selected text, the lines joined by line breaks.
    fn text<S: AsRef<str>>(&self, lines: &[S]) -> String {
        let (start, end) = (self.start(), self.end());
        let mut text = String::new();
        for line in start.line..=end.line.min(lines.len().saturating_sub(1)) {
            if line > start.line {
                text.push('\n');
            }
            if let Some((from, to)) = self.columns(line, lines) {
                let chars = lines[line].as_ref().chars();
                text.extend(chars.skip(from).take(to - from));
            }
        }
        text
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selection_text_spans_all_lines() {
        let lines = ["first line", "second", "third line"];
        let all = LineSelection::all(&lines);
        assert_eq!(all.text(&lines), "first line\nsecond\nthird line");
        let part = LineSelection {
            anchor: TextPosition { line: 2, column: 5 },
            cursor: TextPosition { line: 0, column: 6 },
            lines: 3,
        };
        assert_eq!(part.text(&lines), "line\nsecond\nthird");
        assert_eq!(part.columns(1, &lines), Some((0, 6)));
    }

    #[test]
    fn selection_counts_characters_not_bytes() {
        let lines = ["Spannung σ = 12 MPa"];
        let s = LineSelection {
            anchor: TextPosition { line: 0, column: 9 },
            cursor: TextPosition {
                line: 0,
                column: 10,
            },
            lines: 1,
        };
        assert_eq!(s.text(&lines), "σ");
    }
}
