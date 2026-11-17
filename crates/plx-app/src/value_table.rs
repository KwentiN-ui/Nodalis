//! Tables of single values: the "T" button next to a component of a boundary condition or
//! load opens a window with the value as a table over the step time, which replaces the
//! fixed value and the amplitude for that component (see [`plx_model::ValueTable`]). The
//! same button next to a value of the flow of a material gives the value as a table over the
//! temperature.

use egui::{Response, Ui};
use plx_model::{Quantity, UnitSystem, ValueTable};

use crate::amplitude_dialog::{AmplitudeView, points_editor};

/// A value field and its table button. Without a table `add_value` draws the field; with
/// one a button with the number of points takes its place, which opens the table like the
/// "T" button does. The first click on "T" starts a table that ramps from zero to `value`
/// over `period`, as the default ramp of a static step does. `title` names the window, e.g.
/// "U2 of Move".
#[allow(clippy::too_many_arguments)]
pub fn value_with_table(
    ui: &mut Ui,
    id: egui::Id,
    table: &mut Option<ValueTable>,
    title: &str,
    label: &str,
    quantity: Quantity,
    units: UnitSystem,
    value: f64,
    period: f64,
    add_value: impl FnOnce(&mut Ui) -> Response,
) {
    let kind = Kind {
        argument: Quantity::Time,
        follows: "The value follows a table over the step time. Click to edit.",
        on: "The value follows a table over the step time instead of the amplitude. Click to \
             edit it.",
        off: "Define the value as a table over the step time, e.g. a motion that changes \
              direction within the step. It replaces the value and the amplitude for this \
              component only.",
        note: "Step time and value; linear between the points, constant before the first and \
               after the last. Nonlinear static increments end at every point.",
        back: "Back to the fixed value and the amplitude",
    };
    let start = || ValueTable::ramp(value, period.max(f64::MIN_POSITIVE));
    with_table(
        ui, id, table, title, label, quantity, units, &kind, start, add_value,
    );
}

/// [`value_with_table`] for a value of a material over the temperature, such as a viscosity:
/// the first click on "T" starts a table that holds `value` at 20 °C, to be continued.
#[allow(clippy::too_many_arguments)]
pub fn value_over_temperature(
    ui: &mut Ui,
    id: egui::Id,
    table: &mut Option<ValueTable>,
    title: &str,
    label: &str,
    quantity: Quantity,
    units: UnitSystem,
    value: f64,
    add_value: impl FnOnce(&mut Ui) -> Response,
) {
    let kind = Kind {
        argument: Quantity::Temperature,
        follows: "The value follows a table over the temperature. Click to edit.",
        on: "The value follows a table over the temperature of the material. Click to edit it.",
        off: "Define the value as a table over the temperature of the material, for Particle \
              Flow steps that follow the temperature. It replaces the fixed value.",
        note: "Temperature and value; linear between the points, constant below the first and \
               above the last. It acts in Particle Flow steps that follow the temperature.",
        back: "Back to the fixed value, the same at every temperature",
    };
    let start = || ValueTable {
        points: vec![[20.0, value]],
    };
    with_table(
        ui, id, table, title, label, quantity, units, &kind, start, add_value,
    );
}

/// The texts of a kind of table.
struct Kind {
    /// What the table is over.
    argument: Quantity,
    /// The tooltip of the button that stands for the field while the table is there.
    follows: &'static str,
    /// The tooltips of "T" with and without a table.
    on: &'static str,
    off: &'static str,
    /// Under the table in its window.
    note: &'static str,
    /// The tooltip of "Remove Table".
    back: &'static str,
}

#[allow(clippy::too_many_arguments)]
fn with_table(
    ui: &mut Ui,
    id: egui::Id,
    table: &mut Option<ValueTable>,
    title: &str,
    label: &str,
    quantity: Quantity,
    units: UnitSystem,
    kind: &Kind,
    start: impl FnOnce() -> ValueTable,
    add_value: impl FnOnce(&mut Ui) -> Response,
) {
    // Per OS window, so that the same dialog of two analyses keeps its own table window.
    let id = crate::ids::window(ui.ctx(), id);
    let open_id = id.with("open");
    let mut open = ui
        .ctx()
        .data(|d| d.get_temp::<bool>(open_id))
        .unwrap_or(false);
    ui.horizontal(|ui| {
        match table {
            Some(t) => {
                let text = match t.points.len() {
                    1 => "Table, 1 point".to_owned(),
                    n => format!("Table, {n} points"),
                };
                if ui
                    .add(egui::Button::new(text).min_size(egui::vec2(100.0, 0.0)))
                    .on_hover_text(kind.follows)
                    .clicked()
                {
                    open = !open;
                }
            }
            None => {
                add_value(ui);
            }
        }
        let button = egui::Button::new(egui::RichText::new("T").small().strong())
            .selected(table.is_some())
            .min_size(egui::vec2(16.0, 16.0));
        let tip = if table.is_some() { kind.on } else { kind.off };
        if ui.add(button).on_hover_text(tip).clicked() {
            open = !open;
            if open && table.is_none() {
                *table = Some(start());
            }
        }
    });
    if open {
        let view_id = id.with("view");
        let mut view: AmplitudeView = ui.ctx().data(|d| d.get_temp(view_id)).unwrap_or_default();
        let mut remove = false;
        let mut close = false;
        egui::Window::new(format!("Table: {title}"))
            .id(crate::ids::window(ui.ctx(), (id, "window")))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .default_width(360.0)
            .show(ui.ctx(), |ui| {
                if let Some(table) = table.as_mut() {
                    points_editor(
                        ui,
                        &mut table.points,
                        &mut view,
                        units,
                        "value table points",
                        kind.argument,
                        Some((label, quantity)),
                    );
                    let problem = match kind.argument {
                        Quantity::Time => table.points_problem(),
                        _ => temperature_problem(table),
                    };
                    if let Some(problem) = problem {
                        ui.colored_label(crate::style::error(), problem);
                    }
                }
                ui.weak(kind.note);
                ui.horizontal(|ui| {
                    if ui.button("Remove Table").on_hover_text(kind.back).clicked() {
                        remove = true;
                    }
                    if ui.button("Close").clicked() {
                        close = true;
                    }
                });
            });
        if remove {
            *table = None;
        }
        if remove || close || table.is_none() {
            open = false;
        }
        ui.ctx().data_mut(|d| d.insert_temp(view_id, view));
    }
    ui.ctx().data_mut(|d| d.insert_temp(open_id, open));
}

/// Why a table over the temperature cannot be computed, if it cannot.
fn temperature_problem(table: &ValueTable) -> Option<String> {
    if table.points.is_empty() {
        return Some("The table needs at least one point".into());
    }
    if table.points.iter().flatten().any(|v| !v.is_finite()) {
        return Some("The table contains invalid numbers".into());
    }
    if table.points.iter().any(|p| p[1] < 0.0) {
        return Some("The values must not be negative".into());
    }
    (table.points.windows(2))
        .position(|w| w[1][0] < w[0][0])
        .map(|i| format!("The temperature in row {} is smaller than before", i + 2))
}
