//! Table of one history output component: a row per
//! increment, a column per node, element or face.
//!
//! Columns are selected by their header (Ctrl adds one, Shift
//! a range) and cells by dragging; Ctrl+P or the context menu's Plot draws the selection,
//! the first selected column over the x axis and every other one as a curve.

use egui::{Color32, Modifiers, Rect, Sense, Stroke, pos2, vec2};
use plx_results::history_output::{HistoryComponent, HistorySet};
use plx_results::steps::ResultSteps;
use plx_results::{AnalysisKind, Increment};

use crate::results::format_value;
use crate::xy_plot::XyData;

/// Columns before the entries: step, increment and time (or frequency).
const FIXED_COLUMNS: usize = 3;
const FIXED_WIDTHS: [f32; FIXED_COLUMNS] = [150.0, 72.0, 96.0];
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

/// The steps of the results and their increments, which name the steps of the rows and
/// tie each row to the increment it shows.
#[derive(Clone, Copy)]
pub struct Places<'a> {
    pub steps: &'a ResultSteps,
    pub increments: &'a [Increment],
}

impl Places<'_> {
    /// The step of a row, numbered from 1, and its name; a step the results only number has
    /// no name.
    fn step(&self, solver: u32) -> Option<(usize, Option<&str>)> {
        let index = self.steps.of_solver_step(solver)?;
        let step = &self.steps.steps[index];
        Some((index + 1, (!step.numbered).then_some(step.name.as_str())))
    }
}

/// What the table asks for.
#[derive(Clone, Debug, PartialEq)]
pub enum TableAction {
    None,
    Close,
    Plot(Box<XyData>),
}

/// The values of a table: the columns' titles and a value per row and column.
struct Columns<'a> {
    set: &'a HistorySet,
    field: &'a str,
    component: &'a HistoryComponent,
    kind: AnalysisKind,
    places: Places<'a>,
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
            // Rows of steps over time have a time, whatever the shown increment is.
            2 if (self.set.rows.first())
                .is_some_and(|r| self.places.steps.times_of(r.0, &[r.2]).is_some()) =>
            {
                "Time"
            }
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
            0 => self
                .places
                .step(step)
                .map_or(step as f64, |(n, _)| n as f64),
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
            0 => match (self.set.rows.get(row)).and_then(|r| self.places.step(r.0)) {
                Some((number, Some(name))) => format!("{number} {name}"),
                _ => format!("{value}"),
            },
            1 => format!("{value}"),
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

/// Step time and total time of each row, the rows of each step together; `None` when a row
/// belongs to a step that does not run over time.
fn row_times(places: Places, rows: &[(u32, u32, f64)]) -> Option<Vec<(f64, f64)>> {
    let mut times = Vec::with_capacity(rows.len());
    for run in rows.chunk_by(|a, b| a.0 == b.0) {
        let values: Vec<f64> = run.iter().map(|r| r.2).collect();
        times.extend(places.steps.times_of(run[0].0, &values)?);
    }
    Some(times)
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

    fn columns<'a>(
        &self,
        sets: &'a [HistorySet],
        kind: AnalysisKind,
        places: Places<'a>,
    ) -> Option<Columns<'a>> {
        let set = sets.get(self.set)?;
        let field = set.fields.get(self.field)?;
        Some(Columns {
            set,
            field: &field.name,
            component: field.components.get(self.component)?,
            kind,
            places,
        })
    }

    /// The table as tab separated text, e.g. for a spreadsheet: the selected cells, or all
    /// when nothing is selected.
    pub fn text(&self, sets: &[HistorySet], kind: AnalysisKind, places: Places) -> String {
        let Some(table) = self.columns(sets, kind, places) else {
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
    pub fn plot(
        &self,
        sets: &[HistorySet],
        kind: AnalysisKind,
        places: Places,
    ) -> Result<XyData, String> {
        let table = (self.columns(sets, kind, places)).ok_or("The table has no data.")?;
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
        let mut data = XyData {
            title: format!(
                "{}.{}.{}",
                table.set.name, table.field, table.component.name
            ),
            x_label: table.title(columns[0]).to_string(),
            x,
            curves,
            guides: Vec::new(),
            y_max: None,
            ..Default::default()
        };
        let row_data: Vec<(u32, u32, f64)> = rows.map(|r| table.set.rows[r]).collect();
        // The increment of the results each row shows.
        data.increments = (row_data.iter())
            .map(|&(step, increment, _)| {
                (places.increments.iter()).position(|i| i.step == step && i.increment == increment)
            })
            .collect();
        // The steps as bands, a band for each run of rows of one step.
        for (row, &(step, _, _)) in row_data.iter().enumerate() {
            let place = places.step(step);
            let name = match place {
                Some((number, Some(name))) => format!("{number} {name}"),
                Some((number, None)) => format!("Step {number}"),
                None => format!("Step {step}"),
            };
            match data.bands.last_mut() {
                Some(band) if band.name == name => band.rows[1] = row,
                _ => data.bands.push(crate::xy_plot::Band {
                    name,
                    rows: [row, row],
                }),
            }
        }
        if data.bands.len() < 2 {
            data.bands.clear();
        }
        // Over the step, increment or time the plot can switch between the total time, the
        // step time and the increments, when all rows run over time.
        let times = row_times(places, &row_data);
        if columns[0] < FIXED_COLUMNS
            && let Some(times) = times.filter(|t| !t.is_empty())
        {
            data.x_axes = vec![
                ("Total time".into(), times.iter().map(|t| t.1).collect()),
                ("Step time".into(), times.iter().map(|t| t.0).collect()),
                (
                    "Increment".into(),
                    (1..=times.len()).map(|n| n as f64).collect(),
                ),
            ];
            data.choose_x_axis(if columns[0] == 1 { 2 } else { 0 });
        }
        Ok(data)
    }

    /// Shows the window.
    pub fn show(
        &mut self,
        ctx: &egui::Context,
        sets: &[HistorySet],
        kind: AnalysisKind,
        places: Places,
        unit: Option<&str>,
    ) -> TableAction {
        let Some(table) = self.columns(sets, kind, places) else {
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
                        ui.ctx().copy_text(self.text(sets, kind, places));
                    }
                    let plottable = self.selection.columns.len() >= 2;
                    if ui
                        .add_enabled(plottable, egui::Button::new("Plot"))
                        .on_hover_text("Ctrl+P")
                        .on_disabled_hover_text("Select at least two columns")
                        .clicked()
                    {
                        action = self.plot_action(sets, kind, places);
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
                if let Some(plot) = self.grid(ui, &table, sets, kind, places) {
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
                action = self.plot_action(sets, kind, places);
            }
            if copy {
                ctx.copy_text(self.text(sets, kind, places));
            }
        }
        if !open || close {
            return TableAction::Close;
        }
        action
    }

    fn plot_action(
        &mut self,
        sets: &[HistorySet],
        kind: AnalysisKind,
        places: Places,
    ) -> TableAction {
        match self.plot(sets, kind, places) {
            Ok(data) => {
                self.message = None;
                TableAction::Plot(Box::new(data))
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
        places: Places,
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
                    // A right click on a cell outside the selection selects it.
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
                        ui.ctx().copy_text(self.text(sets, kind, places));
                        ui.close();
                    }
                    let plottable = self.selection.columns.len() >= 2;
                    if ui
                        .add_enabled(plottable, egui::Button::new("Plot              Ctrl+P"))
                        .clicked()
                    {
                        action = Some(self.plot_action(sets, kind, places));
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
                // Step names read from the left, numbers from the right.
                let (at, align) = if column == 0 {
                    (
                        pos2(cell.left() + 6.0, cell.center().y),
                        egui::Align2::LEFT_CENTER,
                    )
                } else {
                    (
                        pos2(cell.right() - 6.0, cell.center().y),
                        egui::Align2::RIGHT_CENTER,
                    )
                };
                painter.text(at, align, table.text(row, column), font.clone(), text);
                painter.vline(cell.right(), cell.y_range(), line);
            }
            // The rows of a step are grouped: a darker line where the next step begins.
            let rows = &table.set.rows;
            let step_ends = (rows.get(row + 1)).is_some_and(|next| next.0 != rows[row].0);
            let below = if step_ends {
                Stroke::new(1.5, crate::style::adapt(Color32::from_gray(120)))
            } else {
                line
            };
            painter.hline(
                (origin.x + view.min.x)..=(origin.x + view.max.x),
                y + ROW_HEIGHT,
                below,
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

    static NO_STEPS: std::sync::LazyLock<ResultSteps> =
        std::sync::LazyLock::new(ResultSteps::default);

    fn no_places() -> Places<'static> {
        Places {
            steps: &NO_STEPS,
            increments: &[],
        }
    }

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
        let data = table
            .plot(&sets, AnalysisKind::Static, no_places())
            .unwrap();
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
        let data = table
            .plot(&sets, AnalysisKind::Static, no_places())
            .unwrap();
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
        assert!(
            table
                .plot(&sets, AnalysisKind::Static, no_places())
                .is_err()
        );
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
            table.text(&sets, AnalysisKind::Static, no_places()),
            "Step\tIncrement\tTime\t41\t62\n1\t1\t0.25\t-1\t-0.5\n1\t2\t0.5\t-2\t-1\n\
             1\t3\t1\t-4\t\n"
        );
        table.selection.press(4, Some(0), Modifiers::NONE);
        table.selection.drag_to(3, Some(1));
        assert_eq!(
            table.text(&sets, AnalysisKind::Static, no_places()),
            "41\t62\n-1\t-0.5\n-2\t-1\n"
        );
    }

    #[test]
    fn a_plot_over_time_has_the_steps_as_bands_and_knows_their_increments() {
        use plx_results::steps::{StepCategory, StepRecord};
        let increment = |step, increment, value| Increment {
            step,
            increment,
            kind: AnalysisKind::Static,
            value,
            fields: Vec::new(),
            nodal_diameter: None,
        };
        // Field output only at the end of each step.
        let increments = vec![increment(1, 2, 1.0), increment(2, 2, 3.0)];
        let record = |name: &str, steps: &[u32]| StepRecord {
            name: name.into(),
            category: StepCategory::Static,
            nlgeom: false,
            perturbation: false,
            time_period: Some(if name == "Preload" { 1.0 } else { 2.0 }),
            solver_steps: steps.to_vec(),
            inherited_from: None,
        };
        let steps = ResultSteps::new(
            &increments,
            &[record("Preload", &[1]), record("Operating load", &[2])],
            &[],
            None,
        );
        let places = Places {
            steps: &steps,
            increments: &increments,
        };
        let mut sets = sets();
        sets[0].rows = vec![(1, 1, 0.5), (1, 2, 1.0), (2, 1, 2.0), (2, 2, 3.0)];
        sets[0].fields[0].components[0].entries = vec![HistoryEntry {
            name: "41".into(),
            values: vec![-1.0, -2.0, -3.0, -4.0],
        }];
        let mut table = HistoryTable::new(0, 0, 0);
        table.selection.press(2, None, Modifiers::NONE);
        table.selection.press(3, None, Modifiers::COMMAND);
        let columns = table.columns(&sets, AnalysisKind::Static, places).unwrap();
        assert_eq!(columns.text(2, 0), "2 Operating load");
        let mut data = table.plot(&sets, AnalysisKind::Static, places).unwrap();
        assert_eq!(data.x_label, "Total time");
        assert_eq!(data.x, [0.5, 1.0, 2.0, 3.0]);
        assert_eq!(data.increments, [None, Some(0), None, Some(1)]);
        let names: Vec<_> = data
            .bands
            .iter()
            .map(|b| (b.name.as_str(), b.rows))
            .collect();
        assert_eq!(names, [("1 Preload", [0, 1]), ("2 Operating load", [2, 3])]);
        data.choose_x_axis(1);
        assert_eq!(
            (data.x_label.as_str(), data.x.as_slice()),
            ("Step time", &[0.5, 1.0, 1.0, 2.0][..])
        );
        data.choose_x_axis(2);
        assert_eq!(data.x, [1.0, 2.0, 3.0, 4.0]);
        // The built-in solver's history output gives the step time.
        sets[0].rows[2].2 = 0.5;
        sets[0].rows[3].2 = 2.0;
        let data = table.plot(&sets, AnalysisKind::Static, places).unwrap();
        assert_eq!(data.x, [0.5, 1.0, 1.5, 3.0]);
    }
}
