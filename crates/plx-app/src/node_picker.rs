//! The overview of the blocks the project schematic can add, shown on a double-click into
//! empty space, and the command search. Its rows look like the search of OMEdit: an icon,
//! the name in bold and a grey line of where it belongs under it. Typing searches it with
//! fuzzy matching; empty, the block picker lists Analysis and the blocks used last.

use nucleo_matcher::pattern::{CaseMatching, Normalization, Pattern};
use nucleo_matcher::{Config, Matcher, Utf32Str};

use crate::command::TileIcon;

/// A kind of block the picker offers.
#[derive(Clone)]
pub struct Entry {
    /// The group it is listed under, such as "Mesh".
    pub category: &'static str,
    pub label: &'static str,
    /// More words the search finds it by.
    pub keywords: &'static str,
    pub tip: &'static str,
    /// The symbol at the left of its row.
    pub icon: TileIcon,
    /// The grey line under its name: where it belongs, such as "Project › Sources › Mesh".
    pub detail: String,
}

/// What became of the picker this frame.
pub enum Outcome {
    Open,
    Closed,
    /// The entry at this index was picked.
    Picked(usize),
}

pub struct NodePicker {
    /// Where it was asked for, on the screen and in the schematic.
    pub screen: egui::Pos2,
    pub graph: egui::Pos2,
    query: String,
    /// The highlighted entry, by its place in the list shown.
    selected: usize,
    /// Whether it was shown before, so that the click that opened it does not close it.
    shown: bool,
    /// The most entries it lists.
    limit: usize,
    /// The text in the empty search field.
    hint: &'static str,
    /// What it lists while the search field is empty, such as the blocks used last; `None`
    /// lists all entries.
    start: Option<Vec<usize>>,
    /// The selection moved by a key in the last frame, so the list scrolls to it.
    follow: bool,
}

const WIDTH: f32 = 340.0;
/// The height of a row: the name over the grey line.
const ROW_HEIGHT: f32 = 38.0;
/// The most rows shown at once; more scroll.
const ROWS: usize = 8;
const ICON: f32 = 22.0;

impl NodePicker {
    pub fn new(screen: egui::Pos2, graph: egui::Pos2) -> Self {
        Self {
            screen,
            graph,
            query: String::new(),
            selected: 0,
            shown: false,
            limit: usize::MAX,
            hint: "Search …",
            start: None,
            follow: false,
        }
    }

    /// A picker that lists at most `limit` entries, with this text in the empty field.
    pub fn with_limit(mut self, limit: usize, hint: &'static str) -> Self {
        self.limit = limit;
        self.hint = hint;
        self
    }

    /// A picker that lists only these entries while nothing is typed.
    pub fn with_start(mut self, start: Vec<usize>, hint: &'static str) -> Self {
        self.start = Some(start);
        self.hint = hint;
        self
    }

    pub fn show(&mut self, ctx: &egui::Context, entries: &[Entry]) -> Outcome {
        let empty = self.query.trim().is_empty();
        let mut list = match (&self.start, empty) {
            (Some(start), true) => (start.iter().copied())
                .filter(|&i| i < entries.len())
                .collect(),
            _ => ranked(&self.query, entries),
        };
        list.truncate(self.limit);
        let (up, down, enter, escape) = ctx.input_mut(|i| {
            (
                i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowUp),
                i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowDown),
                i.consume_key(egui::Modifiers::NONE, egui::Key::Enter),
                i.consume_key(egui::Modifiers::NONE, egui::Key::Escape),
            )
        });
        if escape {
            return Outcome::Closed;
        }
        if up {
            self.selected = self.selected.saturating_sub(1);
        }
        if down {
            self.selected += 1;
        }
        self.follow |= up || down;
        self.selected = self.selected.min(list.len().saturating_sub(1));
        if enter && self.shown {
            return list
                .get(self.selected)
                .map_or(Outcome::Closed, |&i| Outcome::Picked(i));
        }
        let mut picked = None;
        let area = egui::Area::new(crate::ids::window(ctx, "node picker"))
            .order(egui::Order::Foreground)
            .fixed_pos(self.screen)
            .constrain(true)
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.set_width(WIDTH);
                    let search = ui.add(
                        egui::TextEdit::singleline(&mut self.query)
                            .hint_text(self.hint)
                            .desired_width(f32::INFINITY),
                    );
                    if !self.shown || !search.has_focus() {
                        search.request_focus();
                    }
                    if search.changed() {
                        self.selected = 0;
                    }
                    ui.add_space(2.0);
                    if list.is_empty() {
                        ui.label(egui::RichText::new("Nothing found").weak());
                        return;
                    }
                    // The list takes the height of its rows, up to [`ROWS`] of them, so that
                    // it does not keep the size of a shorter list from an earlier frame.
                    let height = ROW_HEIGHT * list.len().min(ROWS) as f32;
                    // A solid bar, always shown as in OMEdit, tells that the list goes on.
                    ui.spacing_mut().scroll = egui::style::ScrollStyle::solid();
                    egui::ScrollArea::vertical()
                        .max_height(height)
                        .min_scrolled_height(height)
                        .auto_shrink([false, true])
                        .scroll_bar_visibility(
                            egui::scroll_area::ScrollBarVisibility::VisibleWhenNeeded,
                        )
                        .show(ui, |ui| {
                            ui.spacing_mut().item_spacing.y = 0.0;
                            for (place, &i) in list.iter().enumerate() {
                                let selected = place == self.selected;
                                let response = row(ui, &entries[i], selected);
                                if selected && self.follow {
                                    response.scroll_to_me(None);
                                }
                                if response.hovered()
                                    && ui.ctx().input(|i| i.pointer.delta() != egui::Vec2::ZERO)
                                {
                                    self.selected = place;
                                }
                                if response.clicked() {
                                    picked = Some(i);
                                }
                            }
                        });
                    if self.start.is_some() && empty {
                        ui.add_space(4.0);
                        ui.label(
                            egui::RichText::new(format!(
                                "Type to search all {} blocks",
                                entries.len()
                            ))
                            .small()
                            .weak(),
                        );
                    }
                });
            });
        self.follow = false;
        if let Some(i) = picked {
            return Outcome::Picked(i);
        }
        // A click anywhere else closes it, as a menu does.
        let outside = ctx.input(|i| {
            i.pointer.any_pressed()
                && (i.pointer.interact_pos()).is_some_and(|p| !area.response.rect.contains(p))
        });
        if self.shown && outside {
            return Outcome::Closed;
        }
        self.shown = true;
        Outcome::Open
    }
}

/// Draws the row of `entry`: its icon at the left, its name in bold and the grey line under
/// it, filled with the selection colour when it is the selected one.
fn row(ui: &mut egui::Ui, entry: &Entry, selected: bool) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), ROW_HEIGHT),
        egui::Sense::click(),
    );
    let response = response.on_hover_text(entry.tip);
    if !ui.is_rect_visible(rect) {
        return response;
    }
    let visuals = ui.visuals();
    let (name_color, detail_color) = if selected {
        let text = visuals.selection.stroke.color;
        (text, text.gamma_multiply(0.8))
    } else {
        (visuals.strong_text_color(), visuals.weak_text_color())
    };
    let painter = ui.painter();
    if selected {
        painter.rect_filled(rect, 2.0, visuals.selection.bg_fill);
    } else if response.hovered() {
        painter.rect_filled(rect, 2.0, crate::style::hover_fill());
    }
    let icon = egui::Rect::from_center_size(
        egui::pos2(rect.left() + 8.0 + ICON * 0.5, rect.center().y),
        egui::Vec2::splat(ICON),
    );
    crate::ribbon::paint_icon(painter, entry.icon, icon, true);
    let left = icon.right() + 10.0;
    let size = egui::TextStyle::Body.resolve(ui.style()).size;
    let name = egui::FontId::new(size, crate::style::bold());
    let detail = egui::FontId::proportional(size * 0.86);
    let width = rect.right() - left - 6.0;
    let name = painter.layout(entry.label.into(), name, name_color, f32::INFINITY);
    let detail = painter.layout(entry.detail.clone(), detail, detail_color, f32::INFINITY);
    let name_height = name.size().y;
    let top = rect.center().y - (name_height + detail.size().y) * 0.5;
    let clip = painter.with_clip_rect(egui::Rect::from_x_y_ranges(
        left..=left + width,
        rect.y_range(),
    ));
    clip.galley(egui::pos2(left, top), name, name_color);
    clip.galley(egui::pos2(left, top + name_height), detail, detail_color);
    response
}
/// The entries matching the query, best first; all of them in their order for no query.
pub fn ranked(query: &str, entries: &[Entry]) -> Vec<usize> {
    if query.trim().is_empty() {
        return (0..entries.len()).collect();
    }
    let pattern = Pattern::parse(query, CaseMatching::Ignore, Normalization::Smart);
    let mut matcher = Matcher::new(Config::DEFAULT);
    let mut buf = Vec::new();
    let mut score = |text: &str| pattern.score(Utf32Str::new(text, &mut buf), &mut matcher);
    let mut found: Vec<(u32, usize)> = (entries.iter().enumerate())
        .filter_map(|(i, e)| {
            // Letters spread over several words match too much, so each word is matched on
            // its own, unless the query has several words. A match in the name counts more.
            let name = score(e.label).map(|s| s * 2);
            let words = (std::iter::once(e.category).chain(e.keywords.split_whitespace()))
                .filter_map(&mut score)
                .max();
            let all = (query.trim().contains(char::is_whitespace))
                .then(|| score(&format!("{} {} {}", e.label, e.category, e.keywords)))
                .flatten();
            name.max(words).max(all).map(|s| (s, i))
        })
        .collect();
    found.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
    found.into_iter().map(|(_, i)| i).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entries() -> Vec<Entry> {
        let entry = |category, label, keywords| Entry {
            category,
            label,
            keywords,
            tip: "",
            icon: TileIcon::None,
            detail: category.into(),
        };
        vec![
            entry("Analysis", "Analysis", "static frequency"),
            entry("Materials", "Materials", "library"),
            entry("Mesh", "Box", "hexahedra 3D"),
            entry("Mesh", "Rectangle", "quadrilateral 2D"),
        ]
    }

    #[test]
    fn no_query_lists_all_in_order() {
        assert_eq!(ranked("", &entries()), vec![0, 1, 2, 3]);
        assert_eq!(ranked("  ", &entries()), vec![0, 1, 2, 3]);
    }

    #[test]
    fn fuzzy_query_finds_by_name_group_and_keywords() {
        let entries = entries();
        assert_eq!(ranked("rect", &entries)[0], 3);
        assert_eq!(ranked("rctngl", &entries)[0], 3);
        assert_eq!(ranked("MAT", &entries)[0], 1);
        assert_eq!(ranked("hexa", &entries), vec![2]);
        let mesh = ranked("mesh", &entries);
        assert!(mesh.starts_with(&[2, 3]) || mesh.starts_with(&[3, 2]));
        assert!(ranked("xyzzy", &entries).is_empty());
        // Letters spread over several words do not match.
        assert_eq!(ranked("rect", &entries), vec![3]);
        assert_eq!(ranked("box 3d", &entries), vec![2]);
    }
}
