//! Time tables of single values: the "T" button next to a component of a boundary
//! condition or load opens a window with the value as a table over the step time, which
//! replaces the fixed value and the amplitude for that component (see
//! [`plx_model::ValueTable`]).

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
                let text = format!("Table, {} points", t.points.len());
                if ui
                    .add(egui::Button::new(text).min_size(egui::vec2(100.0, 0.0)))
                    .on_hover_text("The value follows a table over the step time. Click to edit.")
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
        let tip = if table.is_some() {
            "The value follows a table over the step time instead of the amplitude. \
             Click to edit it."
        } else {
            "Define the value as a table over the step time, e.g. a motion that changes \
             direction within the step. It replaces the value and the amplitude for this \
             component only."
        };
        if ui.add(button).on_hover_text(tip).clicked() {
            open = !open;
            if open && table.is_none() {
                *table = Some(ValueTable::ramp(value, period.max(f64::MIN_POSITIVE)));
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
                        Some((label, quantity)),
                    );
                    if let Some(problem) = table.points_problem() {
                        ui.colored_label(crate::style::error(), problem);
                    }
                }
                ui.weak(
                    "Step time and value; linear between the points, constant before the \
                     first and after the last. The increments of a static step end at every \
                     point.",
                );
                ui.horizontal(|ui| {
                    if ui
                        .button("Remove Table")
                        .on_hover_text("Back to the fixed value and the amplitude")
                        .clicked()
                    {
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
