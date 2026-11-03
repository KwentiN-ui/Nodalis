//! The app bar and the ribbon at the top of the window, made from the command register.
//!
//! App bar: the logo, the workspace segments (Project apart from Geometry, FE Model,
//! Results), the analysis the three act on, the menus, Save/Undo/Redo, the title, the state
//! of the project or analysis, the command search and Settings/Help. Ribbon: the tabs of the
//! workspace with their tiles in groups, and the pinned group (Update, Solve) at the right
//! of every tab.

use std::collections::HashMap;

use egui::{Align2, Color32, FontId, Pos2, Rect, Sense, Shape, Stroke, Ui, Vec2, pos2, vec2};

use crate::command::{self, Cmd, Command, Glyph, PINNED, Tile, TileIcon, Workspace};
use crate::status::CellState;
use crate::style::{border, control, highlight, hover_fill, pressed_fill, window};

/// Height of the app bar.
pub const APP_BAR: f32 = 30.0;
const TILE: Vec2 = vec2(58.0, 62.0);
const ICON: f32 = 22.0;

/// What the ribbon keeps between frames: the tab of each workspace and whether it is
/// folded to its tab row.
#[derive(Default)]
pub struct RibbonState {
    tabs: HashMap<Workspace, &'static str>,
    pub collapsed: bool,
}

impl RibbonState {
    pub fn tab(&self, workspace: Workspace) -> &'static str {
        (self.tabs.get(&workspace).copied()).unwrap_or(workspace.tabs()[0])
    }

    pub fn set_tab(&mut self, workspace: Workspace, tab: &'static str) {
        self.tabs.insert(workspace, tab);
    }

    /// Shows the tab a command is on, as the command search does before running it.
    pub fn reveal(&mut self, workspace: Workspace, cmd: Cmd) {
        if let Some(c) = (command::registry().iter())
            .find(|c| c.cmd == cmd && c.workspaces.contains(&workspace) && c.tab != PINNED)
            && workspace.tabs().contains(&c.tab)
        {
            self.set_tab(workspace, c.tab);
        }
    }
}

/// What the ribbon needs to know of the workspace it shows, which also carries its
/// commands out.
pub trait Host {
    fn enabled(&self, cmd: Cmd) -> bool;
    /// Whether the tile shows pressed: a tool that is open or an option that is on.
    fn checked(&self, _cmd: Cmd) -> bool {
        false
    }
    /// Draws a tile of its own, such as the controls of the shown results.
    fn custom(&mut self, _ui: &mut Ui, _cmd: Cmd) {}
    /// A line shown after the groups, such as how to place a block.
    fn hint(&self, _tab: &str) -> Option<&'static str> {
        None
    }
}

/// Shows the ribbon of `workspace`; returns the command clicked.
pub fn show(
    ui: &mut Ui,
    state: &mut RibbonState,
    workspace: Workspace,
    host: &mut dyn Host,
) -> Option<Cmd> {
    let mut run = None;
    let tab = state.tab(workspace);
    // The tab row.
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 2.0;
        ui.add_space(6.0);
        for &title in workspace.tabs() {
            let selected = title == tab;
            let text = egui::RichText::new(title).size(13.0);
            let text = if selected { text.strong() } else { text };
            let response = ui
                .add(egui::Button::selectable(selected, text).min_size(vec2(64.0, 24.0)))
                .on_hover_text("Double-click folds the ribbon to its tabs, or opens it again.");
            if response.clicked() {
                state.set_tab(workspace, title);
            }
            if response.double_clicked() {
                state.collapsed = !state.collapsed;
            }
        }
    });
    if state.collapsed {
        return None;
    }
    let commands: Vec<&Command> = (command::registry().iter())
        .filter(|c| c.workspaces.contains(&workspace))
        .filter(|c| {
            matches!(
                c.tile,
                Tile::Button | Tile::Split | Tile::Menu | Tile::Custom
            )
        })
        .collect();
    let row: Vec<&Command> = commands.iter().copied().filter(|c| c.tab == tab).collect();
    let pinned: Vec<&Command> = commands
        .iter()
        .copied()
        .filter(|c| c.tab == PINNED)
        .collect();
    let frame = egui::Frame::new()
        .fill(window())
        .stroke(Stroke::new(1.0, border()))
        .inner_margin(egui::Margin::symmetric(4, 2));
    let response = frame
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            let height = TILE.y + 16.0;
            let full = Rect::from_min_size(ui.cursor().min, vec2(ui.available_width(), height));
            ui.allocate_rect(full, Sense::hover());
            let pinned_width = if pinned.is_empty() {
                0.0
            } else {
                group_width(ui, &pinned) + 14.0
            };
            let left =
                Rect::from_min_max(full.min, pos2(full.right() - pinned_width, full.bottom()));
            let mut left_ui = ui.new_child(
                egui::UiBuilder::new()
                    .max_rect(left)
                    .layout(egui::Layout::left_to_right(egui::Align::Min)),
            );
            left_ui.set_clip_rect(left.intersect(ui.clip_rect()));
            if let Some(cmd) = groups(&mut left_ui, &row, tab, host) {
                run = Some(cmd);
            }
            if !pinned.is_empty() {
                // The pinned group stays at the right of every tab.
                let x = left.right() + 6.0;
                ui.painter()
                    .vline(x, full.y_range().shrink(4.0), Stroke::new(1.0, border()));
                let right = Rect::from_min_max(pos2(x + 7.0, full.top()), full.max);
                let mut right_ui = ui.new_child(
                    egui::UiBuilder::new()
                        .max_rect(right)
                        .layout(egui::Layout::left_to_right(egui::Align::Min)),
                );
                if let Some(cmd) = group(&mut right_ui, &pinned, host) {
                    run = Some(cmd);
                }
            }
        })
        .response;
    response.context_menu(|ui| {
        if ui.button("Fold Ribbon").clicked() {
            state.collapsed = true;
        }
    });
    run
}

/// The groups of a tab, left to right, in a strip that scrolls when the window is narrow.
fn groups(ui: &mut Ui, row: &[&Command], tab: &str, host: &mut dyn Host) -> Option<Cmd> {
    let mut run = None;
    egui::ScrollArea::horizontal()
        .id_salt(("ribbon groups", tab))
        .scroll_bar_visibility(egui::scroll_area::ScrollBarVisibility::AlwaysHidden)
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                let mut start = 0;
                while start < row.len() {
                    let name = row[start].group;
                    let end = (row[start..].iter().position(|c| c.group != name))
                        .map_or(row.len(), |n| start + n);
                    if start > 0 {
                        ui.add(egui::Separator::default().vertical().spacing(6.0));
                    }
                    if let Some(cmd) = group(ui, &row[start..end], host) {
                        run = Some(cmd);
                    }
                    start = end;
                }
                if let Some(hint) = host.hint(tab) {
                    ui.add_space(12.0);
                    ui.add(egui::Label::new(egui::RichText::new(hint).weak()).wrap());
                }
            });
        });
    run
}

/// The width the tiles of a group take.
fn group_width(ui: &Ui, tiles: &[&Command]) -> f32 {
    let widths: f32 = tiles.iter().map(|c| tile_width(ui, c)).sum();
    widths + (tiles.len().saturating_sub(1)) as f32
}

/// The label of a tile.
fn tile_label(c: &Command) -> String {
    c.label.to_string()
}

/// The room the arrow of a menu tile takes after its last label line.
fn arrow_room(c: &Command) -> f32 {
    if matches!(c.tile, Tile::Menu | Tile::Split) {
        12.0
    } else {
        0.0
    }
}

fn tile_width(ui: &Ui, c: &Command) -> f32 {
    let font = FontId::proportional(11.5);
    let lines = tile_label(c).lines().count();
    (tile_label(c).lines().enumerate())
        .map(|(i, l)| {
            let width = ui
                .ctx()
                .fonts_mut(|f| f.layout_no_wrap(l.into(), font.clone(), Color32::WHITE))
                .size()
                .x;
            if i + 1 == lines {
                width + arrow_room(c)
            } else {
                width
            }
        })
        .fold(TILE.x - 8.0, f32::max)
        + 8.0
}

/// One group: its tiles side by side and its title below them.
fn group(ui: &mut Ui, tiles: &[&Command], host: &mut dyn Host) -> Option<Cmd> {
    let mut run = None;
    let title = tiles.first().map_or("", |c| c.group);
    ui.vertical(|ui| {
        ui.spacing_mut().item_spacing = vec2(1.0, 0.0);
        let row = ui
            .horizontal(|ui| {
                ui.set_height(TILE.y);
                for c in tiles {
                    if let Some(cmd) = tile(ui, c, host) {
                        run = Some(cmd);
                    }
                }
            })
            .response;
        let rect = Rect::from_min_size(
            pos2(row.rect.left(), row.rect.bottom() + 1.0),
            vec2(row.rect.width().max(30.0), 14.0),
        );
        ui.allocate_rect(rect, Sense::hover());
        ui.painter().text(
            rect.center(),
            Align2::CENTER_CENTER,
            title,
            FontId::proportional(11.0),
            ui.visuals().weak_text_color(),
        );
    });
    run
}

/// A tile: the icon over a label of up to two lines. Returns the command to run.
fn tile(ui: &mut Ui, c: &Command, host: &mut dyn Host) -> Option<Cmd> {
    if c.tile == Tile::Custom {
        ui.vertical(|ui| {
            ui.set_height(TILE.y);
            ui.add_space(4.0);
            host.custom(ui, c.cmd);
        });
        return None;
    }
    let enabled = host.enabled(c.cmd) && ui.is_enabled();
    let checked = host.checked(c.cmd);
    let font = FontId::proportional(11.5);
    let label = tile_label(c);
    let width = tile_width(ui, c);
    let sense = if enabled {
        Sense::click()
    } else {
        Sense::hover()
    };
    let (rect, response) = ui.allocate_exact_size(vec2(width, TILE.y), sense);
    let icon_rect = Rect::from_min_size(rect.min, vec2(rect.width(), 32.0));
    let label_rect = Rect::from_min_max(pos2(rect.left(), icon_rect.bottom()), rect.max);
    // A split tile: the icon runs the command, the label opens the menu.
    let (body, arrow) = if c.tile == Tile::Split {
        let id = response.id;
        let body = ui.interact(icon_rect, id.with("body"), sense);
        let arrow = ui.interact(label_rect, id.with("arrow"), sense);
        (body, Some(arrow))
    } else {
        (response.clone(), None)
    };
    if ui.is_rect_visible(rect) {
        let painter = ui.painter();
        let hovered = enabled && response.hovered();
        if checked || hovered {
            let fill = if checked || response.is_pointer_button_down_on() {
                pressed_fill()
            } else {
                hover_fill()
            };
            painter.rect(
                rect.shrink(1.0),
                3.0,
                fill,
                Stroke::new(1.0, highlight()),
                egui::StrokeKind::Inside,
            );
            if let Some(arrow) = &arrow
                && hovered
            {
                let y = label_rect.top();
                painter.hline(rect.x_range().shrink(4.0), y, Stroke::new(1.0, highlight()));
                if arrow.hovered() {
                    painter.rect_filled(label_rect.shrink(2.0), 2.0, pressed_fill());
                }
            }
        }
        let icon =
            Rect::from_center_size(pos2(rect.center().x, rect.top() + 17.0), Vec2::splat(ICON));
        paint_icon(painter, c.icon, icon, enabled);
        let color = if enabled {
            ui.visuals().text_color()
        } else {
            ui.visuals().weak_text_color()
        };
        let lines = label.lines().count();
        for (i, line) in label.lines().enumerate() {
            let room = if i + 1 == lines { arrow_room(c) } else { 0.0 };
            let at = pos2(
                rect.center().x - room / 2.0,
                label_rect.top() + 7.0 + 13.0 * i as f32,
            );
            let text = painter.text(at, Align2::CENTER_CENTER, line, font.clone(), color);
            if room > 0.0 {
                // A small down arrow after the last line marks the menu.
                let c = pos2(text.right() + 6.0, at.y + 1.0);
                painter.add(egui::Shape::convex_polygon(
                    vec![
                        c + vec2(-3.5, -2.0),
                        c + vec2(3.5, -2.0),
                        c + vec2(0.0, 2.5),
                    ],
                    color,
                    Stroke::NONE,
                ));
            }
        }
    }
    let tip = command::tooltip(c);
    let response = response.on_hover_text(&tip).on_disabled_hover_text(tip);
    let mut run = None;
    match c.tile {
        Tile::Split => {
            if body.clicked() {
                run = Some(c.cmd);
            }
            if let Some(arrow) = arrow {
                run = run.or(menu(&arrow, c.cmd, host));
            }
        }
        Tile::Menu => run = menu(&response, c.cmd, host),
        _ => {
            if response.clicked() {
                run = Some(c.cmd);
            }
        }
    }
    run
}

/// The menu of a menu or split tile: the commands registered in it.
fn menu(response: &egui::Response, parent: Cmd, host: &mut dyn Host) -> Option<Cmd> {
    let mut run = None;
    egui::Popup::menu(response).show(|ui| {
        for c in command::registry()
            .iter()
            .filter(|c| c.tile == Tile::In(parent))
        {
            let button = egui::Button::new(c.name()).shortcut_text(c.shortcut);
            if ui
                .add_enabled(host.enabled(c.cmd), button)
                .on_hover_text(c.tip)
                .clicked()
            {
                run = Some(c.cmd);
            }
        }
    });
    run
}

/// Paints the icon of a tile into the square `area`.
pub fn paint_icon(painter: &egui::Painter, icon: TileIcon, area: Rect, enabled: bool) {
    let painter = if enabled {
        painter.clone()
    } else {
        let mut faded = painter.clone();
        faded.multiply_opacity(0.4);
        faded
    };
    match icon {
        TileIcon::Tool(icon) => crate::icons::paint_in(&painter, icon, area, true),
        TileIcon::Node(icon) => {
            let scale = area.width() / 14.0;
            crate::node_icons::paint_scaled(&painter, area.min, icon, crate::style::text(), scale);
        }
        TileIcon::Tree(icon) => {
            let size = crate::tree_icons::SIZE;
            let min = area.center() - Vec2::splat(size * 0.5);
            crate::tree_icons::paint(&painter, pos2(min.x.round(), min.y.round()), icon);
        }
        TileIcon::Shape(glyph) => paint_glyph(&painter, glyph, area, crate::style::text()),
        TileIcon::None => {}
    }
}

/// Paints a glyph of the ribbon on a 16 x 16 grid scaled into `area`.
pub fn paint_glyph(painter: &egui::Painter, glyph: Glyph, area: Rect, color: Color32) {
    let s = area.width() / 16.0;
    let p = |x: f32, y: f32| pos2(area.left() + x * s, area.top() + y * s);
    let stroke = Stroke::new(1.3 * s.sqrt(), color);
    let accent = Stroke::new(
        1.6 * s.sqrt(),
        crate::style::adapt(Color32::from_rgb(70, 140, 215)),
    );
    let line = |points: &[Pos2], stroke: Stroke| {
        painter.add(Shape::line(points.to_vec(), stroke));
    };
    let rect = |x0: f32, y0: f32, x1: f32, y1: f32, stroke: Stroke| {
        painter.rect_stroke(
            Rect::from_min_max(p(x0, y0), p(x1, y1)),
            1.0,
            stroke,
            egui::StrokeKind::Middle,
        );
    };
    match glyph {
        Glyph::Window => {
            rect(1.5, 2.5, 14.5, 13.5, stroke);
            line(&[p(1.5, 5.0), p(14.5, 5.0)], stroke);
            line(&[p(7.0, 11.0), p(12.0, 7.0)], accent);
            line(&[p(9.0, 7.0), p(12.0, 7.0), p(12.0, 10.0)], accent);
        }
        Glyph::Duplicate => {
            rect(1.5, 1.5, 10.5, 10.5, stroke);
            rect(5.5, 5.5, 14.5, 14.5, accent);
        }
        Glyph::Rename => {
            rect(1.5, 4.5, 14.5, 11.5, stroke);
            line(&[p(4.0, 8.0), p(8.0, 8.0)], stroke);
            line(&[p(10.5, 2.5), p(10.5, 13.5)], accent);
            line(&[p(9.0, 2.5), p(12.0, 2.5)], accent);
            line(&[p(9.0, 13.5), p(12.0, 13.5)], accent);
        }
        Glyph::Frame | Glyph::Unframe => {
            let dash = [
                (1.5, 1.5, 6.0, 1.5),
                (10.0, 1.5, 14.5, 1.5),
                (1.5, 14.5, 6.0, 14.5),
                (10.0, 14.5, 14.5, 14.5),
                (1.5, 1.5, 1.5, 6.0),
                (1.5, 10.0, 1.5, 14.5),
                (14.5, 1.5, 14.5, 6.0),
                (14.5, 10.0, 14.5, 14.5),
            ];
            for (x0, y0, x1, y1) in dash {
                line(&[p(x0, y0), p(x1, y1)], stroke);
            }
            rect(5.0, 5.0, 11.0, 11.0, accent);
            if glyph == Glyph::Unframe {
                line(&[p(3.0, 13.0), p(13.0, 3.0)], stroke);
            }
        }
        Glyph::Compact => {
            rect(2.5, 1.5, 13.5, 14.5, stroke);
            line(&[p(2.5, 5.0), p(13.5, 5.0)], stroke);
            line(&[p(8.0, 7.0), p(8.0, 12.5)], accent);
            line(&[p(5.5, 10.0), p(8.0, 12.5), p(10.5, 10.0)], accent);
        }
        Glyph::Minimap => {
            rect(1.5, 1.5, 14.5, 14.5, stroke);
            rect(7.5, 7.5, 13.0, 13.0, accent);
            rect(3.5, 3.5, 6.0, 5.5, stroke);
        }
        Glyph::Merge => {
            line(&[p(2.0, 3.0), p(7.0, 8.0), p(14.0, 8.0)], stroke);
            line(&[p(2.0, 13.0), p(7.0, 8.0)], stroke);
            painter.circle_filled(p(7.0, 8.0), 1.8 * s, accent.color);
        }
        Glyph::Renumber => {
            line(&[p(5.5, 2.0), p(4.0, 14.0)], stroke);
            line(&[p(11.0, 2.0), p(9.5, 14.0)], stroke);
            line(&[p(2.0, 5.5), p(14.0, 5.5)], accent);
            line(&[p(2.0, 10.5), p(14.0, 10.5)], accent);
        }
        Glyph::Library => {
            for x in [2.0, 5.5, 9.0] {
                rect(x, 3.0, x + 2.5, 14.0, stroke);
            }
            line(&[p(12.0, 4.0), p(14.5, 13.5)], accent);
        }
        Glyph::Check => {
            line(&[p(2.0, 8.5), p(6.0, 12.5), p(14.0, 3.5)], accent);
        }
        Glyph::Hide => {
            let eye: Vec<Pos2> = (0..=16)
                .map(|i| {
                    let t = std::f32::consts::PI * i as f32 / 16.0;
                    p(8.0 - 6.5 * t.cos(), 8.0 - 4.0 * t.sin())
                })
                .chain((0..=16).map(|i| {
                    let t = std::f32::consts::PI * i as f32 / 16.0;
                    p(8.0 + 6.5 * t.cos(), 8.0 + 4.0 * t.sin())
                }))
                .collect();
            painter.add(Shape::closed_line(eye, stroke));
            painter.circle_stroke(p(8.0, 8.0), 2.0 * s, stroke);
            line(&[p(2.5, 14.0), p(13.5, 2.0)], accent);
        }
        Glyph::Worksheet => {
            rect(1.5, 2.5, 14.5, 13.5, stroke);
            for y in [6.0, 9.5] {
                line(&[p(1.5, y), p(14.5, y)], stroke);
            }
            line(&[p(6.0, 2.5), p(6.0, 13.5)], accent);
        }
        Glyph::Settings => {
            let gear: Vec<Pos2> = (0..16)
                .map(|i| {
                    let a = std::f32::consts::TAU * i as f32 / 16.0;
                    let r = if i % 2 == 0 { 6.5 } else { 4.8 };
                    p(8.0 + r * a.cos(), 8.0 + r * a.sin())
                })
                .collect();
            painter.add(Shape::closed_line(gear, stroke));
            painter.circle_stroke(p(8.0, 8.0), 2.0 * s, stroke);
        }
        Glyph::Help => {
            painter.circle_stroke(p(8.0, 8.0), 6.5 * s, stroke);
            painter.text(
                p(8.0, 8.3),
                Align2::CENTER_CENTER,
                "?",
                FontId::proportional(10.0 * s),
                color,
            );
        }
        Glyph::Undo | Glyph::Redo => {
            let flip = |x: f32| if glyph == Glyph::Undo { x } else { 16.0 - x };
            line(&[p(flip(3.0), 6.0), p(flip(10.0), 6.0)], stroke);
            line(
                &[
                    p(flip(10.0), 6.0),
                    p(flip(13.0), 8.5),
                    p(flip(10.0), 12.0),
                    p(flip(6.0), 12.0),
                ],
                stroke,
            );
            line(
                &[p(flip(5.5), 3.5), p(flip(3.0), 6.0), p(flip(5.5), 8.5)],
                stroke,
            );
        }
        Glyph::Search => {
            painter.circle_stroke(p(6.5, 6.5), 4.5 * s, stroke);
            line(&[p(10.0, 10.0), p(14.5, 14.5)], stroke);
        }
        Glyph::Save => crate::icons::paint_in(painter, crate::icons::Icon::Save, area, true),
        Glyph::Analysis => {
            rect(1.5, 2.5, 14.5, 13.5, stroke);
            painter.rect_filled(
                Rect::from_min_max(p(1.5, 2.5), p(14.5, 5.5)),
                1.0,
                accent.color,
            );
            for y in [8.0, 11.0] {
                painter.circle_filled(p(4.0, y), 1.1 * s, color);
                line(&[p(6.0, y), p(12.0, y)], stroke);
            }
        }
    }
}

/// What the app bar shows.
pub struct Bar {
    /// With the Project segment: the project window. An analysis window of its own has none.
    pub project: bool,
    pub workspace: Workspace,
    /// The analyses of the project for the analysis combo; empty in an analysis window.
    pub analyses: Vec<(u64, String)>,
    /// The analysis Geometry, FE Model and Results act on.
    pub active: Option<u64>,
    pub active_name: String,
    pub title: String,
    pub pill: Option<(CellState, String)>,
    pub can_undo: bool,
    pub can_redo: bool,
}

/// What was clicked in the app bar.
#[derive(Clone, Debug, PartialEq)]
pub enum BarEvent {
    Workspace(Workspace),
    Analysis(u64),
    Save,
    Undo,
    Redo,
    /// Open the command search below this point of the screen.
    Search(Pos2),
    Settings,
    Help,
    Pill,
}

/// The app bar. `menus` draws the menus after the segments.
pub fn app_bar(ui: &mut Ui, bar: &Bar, menus: &mut dyn FnMut(&mut Ui)) -> Vec<BarEvent> {
    let mut events = Vec::new();
    let full = ui.max_rect();
    let mut left_end = full.left();
    ui.horizontal(|ui| {
        ui.set_height(APP_BAR - 4.0);
        ui.spacing_mut().item_spacing.x = 4.0;
        // The logo.
        let (rect, _) = ui.allocate_exact_size(vec2(20.0, 22.0), Sense::hover());
        ui.painter().text(
            rect.center(),
            Align2::CENTER_CENTER,
            "N",
            FontId::proportional(17.0),
            crate::style::adapt(Color32::from_rgb(70, 140, 215)),
        );
        let analysis_enabled = bar.active.is_some() || !bar.project;
        if bar.project {
            if segment(
                ui,
                "Project",
                bar.workspace == Workspace::Project,
                true,
                "The schematic of the project: analyses and the blocks they take data from",
            )
            .clicked()
            {
                events.push(BarEvent::Workspace(Workspace::Project));
            }
            ui.add_space(8.0);
        }
        ui.spacing_mut().item_spacing.x = 0.0;
        for workspace in Workspace::ANALYSIS {
            let tip = if analysis_enabled {
                match workspace {
                    Workspace::Geometry => "Import and mesh the geometry of the analysis",
                    Workspace::FeModel => "Set up the model of the analysis and solve it",
                    _ => "Look at the results of the analysis",
                }
            } else {
                "Create an analysis in the project"
            };
            if segment(
                ui,
                workspace.title(),
                bar.workspace == workspace,
                analysis_enabled,
                tip,
            )
            .clicked()
            {
                events.push(BarEvent::Workspace(workspace));
            }
        }
        ui.spacing_mut().item_spacing.x = 4.0;
        if bar.project {
            ui.add_space(6.0);
            let text = if bar.active.is_some() {
                bar.active_name.clone()
            } else {
                "(no analysis)".to_string()
            };
            egui::ComboBox::from_id_salt("active analysis")
                .selected_text(text)
                .width(130.0)
                .show_ui(ui, |ui| {
                    for (id, name) in &bar.analyses {
                        if ui.selectable_label(bar.active == Some(*id), name).clicked() {
                            events.push(BarEvent::Analysis(*id));
                        }
                    }
                })
                .response
                .on_hover_text("The analysis Geometry, FE Model and Results act on");
        }
        ui.add_space(6.0);
        // The menus, as a menu bar: hovering moves from one open menu to the next.
        egui::MenuBar::new().ui(ui, |ui| {
            menus(ui);
            ui.add_space(6.0);
            if small(ui, Glyph::Save, "Save Project (Ctrl+S)", true).clicked() {
                events.push(BarEvent::Save);
            }
            if small(ui, Glyph::Undo, "Undo (Ctrl+Z)", bar.can_undo).clicked() {
                events.push(BarEvent::Undo);
            }
            if small(ui, Glyph::Redo, "Redo (Ctrl+Y)", bar.can_redo).clicked() {
                events.push(BarEvent::Redo);
            }
            left_end = ui.cursor().min.x;
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(
                    egui::RichText::new(concat!("v", env!("CARGO_PKG_VERSION")))
                        .color(ui.visuals().weak_text_color()),
                );
                if small(ui, Glyph::Help, "User Guide", true).clicked() {
                    events.push(BarEvent::Help);
                }
                if small(ui, Glyph::Settings, "Settings", true).clicked() {
                    events.push(BarEvent::Settings);
                }
                let search = search_box(ui);
                if search.clicked() {
                    events.push(BarEvent::Search(search.rect.left_bottom()));
                }
                if let Some((state, text)) = &bar.pill
                    && pill(ui, *state, text).clicked()
                {
                    events.push(BarEvent::Pill);
                }
                // The title in the middle, if it fits between both sides.
                let right_start = ui.cursor().max.x;
                let font = FontId::proportional(13.0);
                let color = ui.visuals().text_color();
                let galley = ui.fonts_mut(|f| f.layout_no_wrap(bar.title.clone(), font, color));
                let center = full.center().x;
                let half = galley.size().x * 0.5 + 12.0;
                let (x0, x1) = if center - half > left_end && center + half < right_start {
                    (center - half, center + half)
                } else {
                    (left_end + 12.0, right_start - 12.0)
                };
                if x1 - x0 > galley.size().x {
                    let y = ui.max_rect().center().y - galley.size().y * 0.5;
                    let x = (x0 + x1) * 0.5 - galley.size().x * 0.5;
                    ui.painter().galley(pos2(x, y), galley, color);
                }
            });
        });
    });
    // Ctrl+K opens the command search.
    if ui.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, egui::Key::K)) {
        let at = pos2(full.right() - 330.0, full.bottom());
        events.push(BarEvent::Search(at));
    }
    events
}

/// A workspace segment.
fn segment(ui: &mut Ui, title: &str, selected: bool, enabled: bool, tip: &str) -> egui::Response {
    let text = egui::RichText::new(title).size(13.0);
    let text = if selected { text.strong() } else { text };
    let button = egui::Button::selectable(selected, text)
        .min_size(vec2(76.0, 24.0))
        .stroke(Stroke::new(
            1.0,
            if selected { highlight() } else { border() },
        ));
    ui.add_enabled(enabled, button)
        .on_hover_text(tip)
        .on_disabled_hover_text(tip)
}

/// A small icon button of the app bar.
fn small(ui: &mut Ui, glyph: Glyph, tip: &str, enabled: bool) -> egui::Response {
    let sense = if enabled {
        Sense::click()
    } else {
        Sense::hover()
    };
    let (rect, response) = ui.allocate_exact_size(vec2(24.0, 22.0), sense);
    if enabled && response.hovered() {
        ui.painter().rect_filled(rect, 3.0, hover_fill());
    }
    let color = if enabled {
        crate::style::text()
    } else {
        ui.visuals().weak_text_color()
    };
    let area = Rect::from_center_size(rect.center(), Vec2::splat(16.0));
    paint_glyph(ui.painter(), glyph, area, color);
    response.on_hover_text(tip)
}

/// The box that opens the command search.
fn search_box(ui: &mut Ui) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(vec2(190.0, 22.0), Sense::click());
    let fill = if response.hovered() {
        hover_fill()
    } else {
        ui.visuals().extreme_bg_color
    };
    ui.painter().rect(
        rect,
        3.0,
        fill,
        Stroke::new(1.0, border()),
        egui::StrokeKind::Inside,
    );
    paint_glyph(
        ui.painter(),
        Glyph::Search,
        Rect::from_center_size(pos2(rect.left() + 12.0, rect.center().y), Vec2::splat(12.0)),
        ui.visuals().weak_text_color(),
    );
    ui.painter().text(
        pos2(rect.left() + 22.0, rect.center().y),
        Align2::LEFT_CENTER,
        "Search commands  Ctrl+K",
        FontId::proportional(12.0),
        ui.visuals().weak_text_color(),
    );
    response.on_hover_text("Finds any command of the ribbon and runs it")
}

/// The state of the project or the analysis as a rounded pill with its symbol.
fn pill(ui: &mut Ui, state: CellState, text: &str) -> egui::Response {
    let font = FontId::proportional(12.0);
    let galley =
        ui.fonts_mut(|f| f.layout_no_wrap(text.to_string(), font, ui.visuals().text_color()));
    let size = vec2(galley.size().x + 30.0, 22.0);
    let (rect, response) = ui.allocate_exact_size(size, Sense::click());
    let fill = if response.hovered() {
        hover_fill()
    } else {
        control()
    };
    ui.painter().rect(
        rect,
        11.0,
        fill,
        Stroke::new(1.0, border()),
        egui::StrokeKind::Inside,
    );
    let symbol = crate::status::symbol_rect(Rect::from_center_size(
        pos2(rect.left() + 13.0, rect.center().y),
        Vec2::splat(16.0),
    ));
    crate::status::paint(ui.painter(), symbol.min, state);
    ui.painter().galley(
        pos2(rect.left() + 24.0, rect.center().y - galley.size().y * 0.5),
        galley,
        ui.visuals().text_color(),
    );
    response
}
