//! Table of one history output component, like PrePoMax's history output view: a row per
//! increment, a column per node, element or face.
//!
//! As in PrePoMax's data grid, columns are selected by their header (Ctrl adds one, Shift
//! a range) and cells by dragging; Ctrl+P or the context menu's Plot draws the selection,
//! the first selected column over the x axis and every other one as a curve.

use egui::{Color32, Modifiers, Rect, Sense, Stroke, pos2, vec2};
use plx_results::AnalysisKind;
use plx_results::history_output::{HistoryComponent, HistorySet};

use crate::results::format_value;
use crate::xy_plot::XyData;

/// Columns before the entries: step, increment and time (or frequency).
const FIXED_COLUMNS: usize = 3;
const FIXED_WIDTHS: [f32; FIXED_COLUMNS] = [44.0, 72.0, 96.0];
const COLUMN_WIDTH: f32 = 100.0;
const ROW_HEIGHT: f32 = 20.0;
const HEADER_HEIGHT: f32 = 22.0;

/// Cells chosen in the table: columns in the order they were chosen, and the rows.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Selection {
    pub columns: Vec<usize>,
    /// First and last row, both included; `None` is all rows.
    pub rows: Option<(usize, usize)>,
    /// Where the last plain click was, for Shift clicks and drags: column and row (`None`
    /// on the header).
    anchor: Option<(usize, Option<usize>)>,
}

impl Selection {
    pub fn contains(&self, column: usize, row: usize) -> bool {
        self.columns.contains(&column) && self.rows.is_none_or(|(a, b)| (a..=b).contains(&row))
    }

    /// Columns from `from` to `to`, in that order.
    fn span(from: usize, to: usize) -> Vec<usize> {
        if from <= to {
            (from..=to).collect()
        } else {
            (to..=from).rev().collect()
        }
    }

    /// A press on a header (`row` is `None`) or a cell, with the keyboard modifiers.
    pub fn press(&mut self, column: usize, row: Option<usize>, modifiers: Modifiers) {
        if modifiers.shift
            && let Some((from, from_row)) = self.anchor
        {
            self.columns = Self::span(from, column);
            self.rows = match (from_row, row) {
                (Some(a), Some(b)) => Some((a.min(b), a.max(b))),
                _ => None,
            };
        } else if modifiers.command {
            // Ctrl adds a column, or takes a chosen one out again.
            if let Some(at) = self.columns.iter().position(|&c| c == column) {
                self.columns.remove(at);
            } else {
                self.columns.push(column);
            }
            if self.columns.len() == 1 {
                self.rows = row.map(|r| (r, r));
            }
            self.anchor = Some((column, row));
        } else {
            self.columns = vec![column];
            self.rows = row.map(|r| (r, r));
            self.anchor = Some((column, row));
        }
    }

    /// Dragging from the anchor to a header or cell.
    pub fn drag_to(&mut self, column: usize, row: Option<usize>) {
        let Some((from, from_row)) = self.anchor else {
            return;
        };
        self.columns = Self::span(from, column);
        self.rows = match (from_row, row) {
            (Some(a), Some(b)) => Some((a.min(b), a.max(b))),
            (Some(a), None) => Some((0, a)),
            _ => None,
        };
    }

    fn clear(&mut self) {
        *self = Self::default();
    }
}

/// Which component is shown: indices of set, field and component, and the selection.
#[derive(Clone, Debug, PartialEq)]
pub struct HistoryTable {
    pub set: usize,
    pub field: usize,
    pub component: usize,
    pub selection: Selection,
    /// Keyboard shortcuts go to the table after a click into it, until a click elsewhere.
    active: bool,
    /// Why the selection could not be plotted.
    message: Option<String>,
}

/// What the table asks for.
#[derive(Clone, Debug, PartialEq)]
pub enum TableAction {
    None,
    Close,
    Plot(XyData),
}

/// The values of a table: the columns' titles and a value per row and column.
struct Columns<'a> {
    set: &'a HistorySet,
    field: &'a str,
    component: &'a HistoryComponent,
    kind: AnalysisKind,
}

impl Columns<'_> {
    fn count(&self) -> usize {
        FIXED_COLUMNS + self.component.entries.len()
    }

    fn rows(&self) -> usize {
        self.set.rows.len()
    }

    fn title(&self, column: usize) -> &str {
        match column {
            0 => "Step",
            1 => "Increment",
            2 => self.kind.value_label(),
            c => self
                .component
                .entries
                .get(c - FIXED_COLUMNS)
                .map_or("", |e| e.name.as_str()),
        }
    }

    fn value(&self, row: usize, column: usize) -> f64 {
        let Some(&(step, increment, value)) = self.set.rows.get(row) else {
            return f64::NAN;
        };
        match column {
            0 => step as f64,
            1 => increment as f64,
            2 => value,
            c => (self.component.entries.get(c - FIXED_COLUMNS))
                .and_then(|e| e.values.get(row))
                .copied()
                .unwrap_or(f64::NAN),
        }
    }

    fn text(&self, row: usize, column: usize) -> String {
        let value = self.value(row, column);
        match column {
            0 | 1 => format!("{value}"),
            _ if !value.is_finite() => String::new(),
            _ => format_value(value as f32),
        }
    }

    fn left(&self, column: usize) -> f32 {
        let fixed = FIXED_WIDTHS.iter().take(column).sum::<f32>();
        fixed + column.saturating_sub(FIXED_COLUMNS) as f32 * COLUMN_WIDTH
    }

    fn width(&self, column: usize) -> f32 {
        FIXED_WIDTHS.get(column).copied().unwrap_or(COLUMN_WIDTH)
    }

    /// The column at a distance from the table's left edge.
    fn column_at(&self, x: f32) -> usize {
        let mut left = 0.0;
        for (column, width) in FIXED_WIDTHS.iter().enumerate() {
            if x < left + width {
                return column;
            }
            left += width;
        }
        let column = FIXED_COLUMNS + ((x - left).max(0.0) / COLUMN_WIDTH) as usize;
        column.min(self.count().saturating_sub(1))
    }
}

impl HistoryTable {
    pub fn new(set: usize, field: usize, component: usize) -> Self {
        Self {
            set,
            field,
            component,
            selection: Selection::default(),
            active: false,
            message: None,
        }
    }

    fn columns<'a>(&self, sets: &'a [HistorySet], kind: AnalysisKind) -> Option<Columns<'a>> {
        let set = sets.get(self.set)?;
        let field = set.fields.get(self.field)?;
        Some(Columns {
            set,
            field: &field.name,
            component: field.components.get(self.component)?,
            kind,
        })
    }

    /// The table as tab separated text, e.g. for a spreadsheet: the selected cells, or all
    /// when nothing is selected.
    pub fn text(&self, sets: &[HistorySet], kind: AnalysisKind) -> String {
        let Some(table) = self.columns(sets, kind) else {
            return String::new();
        };
        let columns: Vec<usize> = if self.selection.columns.is_empty() {
            (0..table.count()).collect()
        } else {
            let mut columns = self.selection.columns.clone();
            columns.sort_unstable();
            columns
        };
        let (first, last) = (self.selection.rows).unwrap_or((0, table.rows().saturating_sub(1)));
        let mut lines = vec![
            (columns.iter().map(|&c| table.title(c)))
                .collect::<Vec<_>>()
                .join("\t"),
        ];
        for row in (first..=last).filter(|&r| r < table.rows()) {
            let values: Vec<String> = (columns.iter())
                .map(|&c| {
                    let value = table.value(row, c);
                    if value.is_finite() {
                        value.to_string()
                    } else {
                        String::new()
                    }
                })
                .collect();
            lines.push(values.join("\t"));
        }
        lines.join("\n") + "\n"
    }

    /// The diagram of the selection: the first chosen column over x, the others as curves.
    pub fn plot(&self, sets: &[HistorySet], kind: AnalysisKind) -> Result<XyData, String> {
        let table = self.columns(sets, kind).ok_or("The table has no data.")?;
        let columns = &self.selection.columns;
        if columns.len() < 2 {
            return Err("Select at least two columns for a plot; \
                        the first one selected becomes the x axis."
                .into());
        }
        let rows = match self.selection.rows {
            Some((first, last)) => first..last.min(table.rows().saturating_sub(1)) + 1,
            None => 0..table.rows(),
        };
        let x = rows.clone().map(|r| table.value(r, columns[0])).collect();
        let curves = (columns[1..].iter())
            .map(|&c| {
                let values = rows.clone().map(|r| table.value(r, c)).collect();
                (table.title(c).to_string(), values)
            })
            .collect();
        Ok(XyData {
            title: format!(
                "{}.{}.{}",
                table.set.name, table.field, table.component.name
            ),
            x_label: table.title(columns[0]).to_string(),
            x,
            curves,
            guides: Vec::new(),
            y_max: None,
        })
    }

    /// Shows the window.
    pub fn show(
        &mut self,
        ctx: &egui::Context,
        sets: &[HistorySet],
        kind: AnalysisKind,
        unit: Option<&str>,
    ) -> TableAction {
        let Some(table) = self.columns(sets, kind) else {
            return TableAction::Close;
        };
        let mut action = TableAction::None;
        let mut open = true;
        let mut close = false;
        let title = format!(
            "History Output: {}.{}.{}",
            table.set.name, table.field, table.component.name
        );
        let window = egui::Window::new(title)
            .id(crate::ids::window(ctx, "history table"))
            .open(&mut open)
            .collapsible(false)
            .resizable(true)
            .default_size([640.0, 400.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label(format!(
                        "{} rows, {} entries",
                        table.rows(),
                        table.component.entries.len()
                    ));
                    if let Some(unit) = unit.filter(|u| !u.trim().is_empty() && u.trim() != "/") {
                        ui.label(format!("Unit: {}", unit.trim()));
                    }
                    if ui.button("Copy").clicked() {
                        ui.ctx().copy_text(self.text(sets, kind));
                    }
                    let plottable = self.selection.columns.len() >= 2;
                    if ui
                        .add_enabled(plottable, egui::Button::new("Plot"))
                        .on_hover_text("Ctrl+P")
                        .on_disabled_hover_text("Select at least two columns")
                        .clicked()
                    {
                        action = self.plot_action(sets, kind);
                    }
                });
                ui.weak(
                    "Select columns via the header (Ctrl: more, Shift: range), cells by \
                     dragging. Ctrl+P plots the selection, the first column selected \
                     becomes the x axis.",
                );
                if let Some(message) = &self.message {
                    ui.colored_label(crate::style::error(), message);
                }
                ui.separator();
                close = crate::dialog::close_bar(ui, "history table");
                if let Some(plot) = self.grid(ui, &table, sets, kind) {
                    action = plot;
                }
            });
        if let Some(window) = &window {
            let rect = window.response.rect;
            let outside = ctx.input(|i| {
                i.pointer.any_pressed()
                    && i.pointer.interact_pos().is_some_and(|p| !rect.contains(p))
            });
            if outside {
                self.active = false;
            }
        }
        if self.active {
            let (plot, copy) = ctx.input_mut(|i| {
                let plot = i.consume_key(Modifiers::COMMAND, egui::Key::P);
                let copy = i.events.iter().any(|e| matches!(e, egui::Event::Copy));
                (plot, copy)
            });
            if plot {
                action = self.plot_action(sets, kind);
            }
            if copy {
                ctx.copy_text(self.text(sets, kind));
            }
        }
        if !open || close {
            return TableAction::Close;
        }
        action
    }

    fn plot_action(&mut self, sets: &[HistorySet], kind: AnalysisKind) -> TableAction {
        match self.plot(sets, kind) {
            Ok(data) => {
                self.message = None;
                TableAction::Plot(data)
            }
            Err(message) => {
                self.message = Some(message);
                TableAction::None
            }
        }
    }

    /// The cells, drawn only where they are visible; the header stays at the top.
    fn grid(
        &mut self,
        ui: &mut egui::Ui,
        table: &Columns,
        sets: &[HistorySet],
        kind: AnalysisKind,
    ) -> Option<TableAction> {
        let mut action = None;
        let total = vec2(
            table.left(table.count()),
            HEADER_HEIGHT + table.rows() as f32 * ROW_HEIGHT,
        );
        egui::ScrollArea::both()
            .auto_shrink(false)
            .show_viewport(ui, |ui, viewport| {
                let (rect, response) = ui.allocate_exact_size(total, Sense::click_and_drag());
                let painter = ui.painter_at(viewport.translate(rect.min.to_vec2()));
                let origin = rect.min;
                let header_top = origin.y + viewport.min.y;
                // Header (row None) or cell under a screen position.
                let cell_at = |p: egui::Pos2| {
                    let column = table.column_at(p.x - origin.x);
                    if p.y < header_top + HEADER_HEIGHT {
                        (column, None)
                    } else {
                        let row = ((p.y - origin.y - HEADER_HEIGHT) / ROW_HEIGHT) as usize;
                        (column, Some(row.min(table.rows().saturating_sub(1))))
                    }
                };
                let pressed = ui.input(|i| i.pointer.primary_pressed());
                if pressed
                    && response.hovered()
                    && let Some(p) = ui.input(|i| i.pointer.interact_pos())
                {
                    let (column, row) = cell_at(p);
                    let modifiers = ui.input(|i| i.modifiers);
                    self.selection.press(column, row, modifiers);
                    self.active = true;
                    self.message = None;
                }
                if response.dragged_by(egui::PointerButton::Primary)
                    && !ui.input(|i| i.modifiers.command)
                    && let Some(p) = response.interact_pointer_pos()
                {
                    let (column, row) = cell_at(p);
                    self.selection.drag_to(column, row);
                }
                if response.secondary_clicked()
                    && let Some(p) = response.interact_pointer_pos()
                {
                    // As in PrePoMax, a right click on a cell outside the selection selects it.
                    let (column, row) = cell_at(p);
                    let inside = match row {
                        Some(r) => self.selection.contains(column, r),
                        None => self.selection.columns.contains(&column),
                    };
                    if !inside {
                        self.selection.press(column, row, Modifiers::NONE);
                    }
                    self.active = true;
                }
                response.context_menu(|ui| {
                    if ui.button("Copy              Ctrl+C").clicked() {
                        ui.ctx().copy_text(self.text(sets, kind));
                        ui.close();
                    }
                    let plottable = self.selection.columns.len() >= 2;
                    if ui
                        .add_enabled(plottable, egui::Button::new("Plot              Ctrl+P"))
                        .clicked()
                    {
                        action = Some(self.plot_action(sets, kind));
                        ui.close();
                    }
                    if ui.button("Clear Selection").clicked() {
                        self.selection.clear();
                        ui.close();
                    }
                });
                self.paint(&painter, table, origin, viewport);
            });
        action
    }

    fn paint(&self, painter: &egui::Painter, table: &Columns, origin: egui::Pos2, view: Rect) {
        let font = egui::FontId::proportional(13.0);
        let text = crate::style::plot_text();
        let line = Stroke::new(1.0, crate::style::adapt(Color32::from_gray(215)));
        let selected = crate::style::pressed_fill();
        let first = table.column_at(view.min.x);
        let last = table.column_at(view.max.x);
        let top_row = ((view.min.y - HEADER_HEIGHT).max(0.0) / ROW_HEIGHT) as usize;
        let bottom_row =
            (((view.max.y - HEADER_HEIGHT).max(0.0) / ROW_HEIGHT) as usize + 1).min(table.rows());
        for row in top_row..bottom_row {
            let y = origin.y + HEADER_HEIGHT + row as f32 * ROW_HEIGHT;
            if row % 2 == 1 {
                let band = Rect::from_min_size(
                    pos2(origin.x + view.min.x, y),
                    vec2(view.width(), ROW_HEIGHT),
                );
                painter.rect_filled(band, 0.0, crate::style::faint());
            }
            for column in first..=last {
                let cell = Rect::from_min_size(
                    pos2(origin.x + table.left(column), y),
                    vec2(table.width(column), ROW_HEIGHT),
                );
                if self.selection.contains(column, row) {
                    painter.rect_filled(cell, 0.0, selected);
                }
                painter.text(
                    pos2(cell.right() - 6.0, cell.center().y),
                    egui::Align2::RIGHT_CENTER,
                    table.text(row, column),
                    font.clone(),
                    text,
                );
                painter.vline(cell.right(), cell.y_range(), line);
            }
            painter.hline(
                (origin.x + view.min.x)..=(origin.x + view.max.x),
                y + ROW_HEIGHT,
                line,
            );
        }
        // The header stays at the top of the view.
        let top = origin.y + view.min.y;
        let band = Rect::from_min_size(
            pos2(origin.x + view.min.x, top),
            vec2(view.width(), HEADER_HEIGHT),
        );
        painter.rect_filled(band, 0.0, crate::style::adapt(Color32::from_gray(236)));
        let x_column = self.selection.columns.first().copied();
        for column in first..=last {
            let cell = Rect::from_min_size(
                pos2(origin.x + table.left(column), top),
                vec2(table.width(column), HEADER_HEIGHT),
            );
            if self.selection.columns.contains(&column) {
                let fill = if crate::style::dark() {
                    Color32::from_rgb(40, 95, 150)
                } else {
                    Color32::from_rgb(178, 211, 238)
                };
                painter.rect_filled(cell, 0.0, fill);
            }
            let mut title = table.title(column).to_string();
            if x_column == Some(column) && self.selection.columns.len() > 1 {
                title.push_str(" (X)");
            }
            painter.text(
                cell.center(),
                egui::Align2::CENTER_CENTER,
                title,
                egui::FontId::proportional(13.0),
                crate::style::text(),
            );
            painter.vline(
                cell.right(),
                cell.y_range(),
                Stroke::new(1.0, crate::style::adapt(Color32::from_gray(190))),
            );
        }
        painter.hline(
            band.x_range(),
            band.bottom(),
            Stroke::new(1.0, crate::style::adapt(Color32::from_gray(160))),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_results::history_output::{HistoryEntry, HistoryField};

    fn sets() -> Vec<HistorySet> {
        let entry = |name: &str, values: [f64; 3]| HistoryEntry {
            name: name.into(),
            values: values.to_vec(),
        };
        vec![HistorySet {
            name: "NH_OUTPUT-1".into(),
            rows: vec![(1, 1, 0.25), (1, 2, 0.5), (1, 3, 1.0)],
            fields: vec![HistoryField {
                name: "DISPLACEMENTS".into(),
                components: vec![HistoryComponent {
                    name: "U3".into(),
                    entries: vec![
                        entry("41", [-1.0, -2.0, -4.0]),
                        entry("62", [-0.5, -1.0, f64::NAN]),
                    ],
                }],
            }],
        }]
    }

    #[test]
    fn the_first_chosen_column_becomes_the_x_axis() {
        let sets = sets();
        let mut table = HistoryTable::new(0, 0, 0);
        // Node 41 first, then the time with Ctrl.
        table.selection.press(3, None, Modifiers::NONE);
        table.selection.press(2, None, Modifiers::COMMAND);
        let data = table.plot(&sets, AnalysisKind::Static).unwrap();
        assert_eq!(data.title, "NH_OUTPUT-1.DISPLACEMENTS.U3");
        assert_eq!(data.x_label, "41");
        assert_eq!(data.x, [-1.0, -2.0, -4.0]);
        assert_eq!(data.curves, [("Time".to_string(), vec![0.25, 0.5, 1.0])]);
    }

    #[test]
    fn a_dragged_range_plots_its_rows_against_its_first_column() {
        let sets = sets();
        let mut table = HistoryTable::new(0, 0, 0);
        table.selection.press(2, Some(1), Modifiers::NONE);
        table.selection.drag_to(4, Some(2));
        assert_eq!(table.selection.columns, [2, 3, 4]);
        assert_eq!(table.selection.rows, Some((1, 2)));
        let data = table.plot(&sets, AnalysisKind::Static).unwrap();
        assert_eq!(data.x, [0.5, 1.0]);
        assert_eq!(data.curves.len(), 2);
        assert_eq!(data.curves[0].1, [-2.0, -4.0]);
        assert!(data.curves[1].1[1].is_nan());
        // Dragged from right to left, the right column is x.
        table.selection.press(4, None, Modifiers::NONE);
        table.selection.drag_to(3, None);
        assert_eq!(table.selection.columns, [4, 3]);
    }

    #[test]
    fn one_column_is_not_enough_for_a_plot() {
        let sets = sets();
        let mut table = HistoryTable::new(0, 0, 0);
        table.selection.press(3, None, Modifiers::NONE);
        assert!(table.plot(&sets, AnalysisKind::Static).is_err());
        // Ctrl on a chosen column takes it out again; Shift selects a range.
        table.selection.press(2, None, Modifiers::SHIFT);
        assert_eq!(table.selection.columns, [3, 2]);
        table.selection.press(3, None, Modifiers::COMMAND);
        assert_eq!(table.selection.columns, [2]);
    }

    #[test]
    fn copied_text_holds_the_selection_or_everything() {
        let sets = sets();
        let mut table = HistoryTable::new(0, 0, 0);
        assert_eq!(
            table.text(&sets, AnalysisKind::Static),
            "Step\tIncrement\tTime\t41\t62\n1\t1\t0.25\t-1\t-0.5\n1\t2\t0.5\t-2\t-1\n\
             1\t3\t1\t-4\t\n"
        );
        table.selection.press(4, Some(0), Modifiers::NONE);
        table.selection.drag_to(3, Some(1));
        assert_eq!(
            table.text(&sets, AnalysisKind::Static),
            "41\t62\n-1\t-0.5\n-2\t-1\n"
        );
    }
}
