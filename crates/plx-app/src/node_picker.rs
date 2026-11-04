//! The overview of the blocks the project schematic can add, shown on a double-click into
//! empty space like the add menu of Blender. Typing searches it with fuzzy matching.

use nucleo_matcher::pattern::{CaseMatching, Normalization, Pattern};
use nucleo_matcher::{Config, Matcher, Utf32Str};

/// A kind of block the picker offers.
pub struct Entry {
    /// The group it is listed under, such as "Mesh".
    pub category: &'static str,
    pub label: &'static str,
    /// More words the search finds it by.
    pub keywords: &'static str,
    pub tip: &'static str,
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
}

const WIDTH: f32 = 240.0;

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
        }
    }

    /// A picker that lists at most `limit` entries, with this text in the empty field.
    pub fn with_limit(mut self, limit: usize, hint: &'static str) -> Self {
        self.limit = limit;
        self.hint = hint;
        self
    }

    pub fn show(&mut self, ctx: &egui::Context, entries: &[Entry]) -> Outcome {
        let mut list = ranked(&self.query, entries);
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
                    ui.separator();
                    let grouped = self.query.trim().is_empty();
                    // No scroll area: the area keeps last frame's size as its limit, so
                    // a list in one would stay cut short when it grows again.
                    ui.with_layout(egui::Layout::top_down_justified(egui::Align::LEFT), |ui| {
                        let mut category = "";
                        for (place, &i) in list.iter().enumerate() {
                            let entry = &entries[i];
                            if grouped && entry.category != category {
                                category = entry.category;
                                ui.label(egui::RichText::new(category).small().weak());
                            }
                            let selected = place == self.selected;
                            let button = if grouped {
                                egui::Button::selectable(selected, entry.label)
                            } else {
                                egui::Button::selectable(
                                    selected,
                                    (
                                        entry.label,
                                        egui::Atom::grow(),
                                        if selected {
                                            egui::RichText::new(entry.category)
                                        } else {
                                            egui::RichText::new(entry.category).weak()
                                        },
                                    ),
                                )
                            };
                            let response = ui.add(button).on_hover_text(entry.tip);
                            if response.clicked() {
                                picked = Some(i);
                            }
                        }
                        if list.is_empty() {
                            ui.label(egui::RichText::new("Nothing found").weak());
                        }
                    });
                });
            });
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
