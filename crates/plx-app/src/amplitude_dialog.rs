//! The amplitude dialog: name, time span and shifts of a tabular amplitude, its points
//! as an editable table and the curve they make; and the amplitude choice of the boundary
//! condition and load dialogs.

use egui::Ui;
use plx_model::units::parse_number;
use plx_model::{Amplitude, AmplitudeTime, FeModel, Quantity, UnitSystem};

use crate::numeric;

/// State of the dialog besides the amplitude itself.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AmplitudeView {
    /// A paste of the clipboard was requested; the next paste replaces the points.
    awaiting_paste: bool,
    /// What the last paste brought, shown under the table.
    paste_note: Option<String>,
}

/// The dialog's form, laid out by itself like the surface interaction dialog.
pub fn amplitude_form(
    ui: &mut Ui,
    amplitude: &mut Amplitude,
    view: &mut AmplitudeView,
    units: UnitSystem,
) {
    egui::Grid::new("amplitude form")
        .max_col_width(crate::dock::column_limit(ui))
        .num_columns(2)
        .spacing([12.0, 6.0])
        .show(ui, |ui| {
            ui.label("Name");
            ui.add(egui::TextEdit::singleline(&mut amplitude.name).desired_width(200.0));
            ui.end_row();
            ui.label("Time span");
            egui::ComboBox::from_id_salt("amplitude time span")
                .selected_text(time_span_label(amplitude.time_span))
                .width(crate::dock::fit_width(ui, 200.0))
                .show_ui(ui, |ui| {
                    for span in [AmplitudeTime::Step, AmplitudeTime::Total] {
                        ui.selectable_value(&mut amplitude.time_span, span, time_span_label(span));
                    }
                });
            ui.end_row();
            ui.label("Shift time");
            ui.add(numeric::quantity(&mut amplitude.shift_time, units, Quantity::Time).speed(0.01));
            ui.end_row();
            ui.label("Shift amplitude");
            ui.add(numeric::drag_value(&mut amplitude.shift_amplitude).speed(0.01));
            ui.end_row();
        });
    ui.add_space(6.0);
    ui.strong("Data Points");
    points_editor(
        ui,
        &mut amplitude.points,
        view,
        units,
        "amplitude points",
        None,
    );
    if let Some(problem) = amplitude.points_problem() {
        ui.colored_label(crate::style::error(), problem);
    }
    ui.weak(
        "Linear between the points, constant before the first and after the last.\n\
         Without an amplitude a load ramps up linearly in a static step.",
    );
}

/// The points of a table over time as an editable table with the curve they make: rows to
/// add, remove, sort and paste from the clipboard. The values are factors (`value` `None`)
/// or of a quantity, shown in the unit system's unit.
pub fn points_editor(
    ui: &mut Ui,
    points: &mut Vec<[f64; 2]>,
    view: &mut AmplitudeView,
    units: UnitSystem,
    id: &str,
    value: Option<(&str, Quantity)>,
) {
    let time_unit = units.unit(Quantity::Time);
    let time_header = if time_unit.is_empty() {
        "Time".to_string()
    } else {
        format!("Time [{time_unit}]")
    };
    let value_header = match value {
        None => "Amplitude [/]".to_string(),
        Some((label, quantity)) => {
            let unit = units.unit(quantity);
            if unit.is_empty() {
                label.to_string()
            } else {
                format!("{label} [{unit}]")
            }
        }
    };
    let mut remove = None;
    let table = egui::ScrollArea::vertical()
        .id_salt(id)
        .max_height(180.0)
        .auto_shrink([false, true])
        .show(ui, |ui| {
            egui::Grid::new((id, "grid"))
                .num_columns(4)
                .striped(true)
                .spacing([12.0, 4.0])
                .show(ui, |ui| {
                    ui.strong("#");
                    ui.strong(&time_header);
                    ui.strong(&value_header);
                    ui.label("");
                    ui.end_row();
                    let removable = points.len() > 1;
                    for (i, [time, factor]) in points.iter_mut().enumerate() {
                        ui.label((i + 1).to_string());
                        ui.add(
                            numeric::without_unit(time, units, Quantity::Time)
                                .speed(0.01)
                                .min_decimals(1),
                        );
                        match value {
                            None => ui.add(numeric::drag_value(factor).speed(0.01).min_decimals(1)),
                            Some((_, quantity)) => ui.add(
                                numeric::without_unit(factor, units, quantity)
                                    .speed(0.01)
                                    .min_decimals(1),
                            ),
                        };
                        if ui
                            .add_enabled(removable, egui::Button::new("Remove").small())
                            .clicked()
                        {
                            remove = Some(i);
                        }
                        ui.end_row();
                    }
                });
        });
    if let Some(i) = remove {
        points.remove(i);
    }
    let table_hovered = ui.rect_contains_pointer(table.inner_rect);
    ui.horizontal(|ui| {
        if ui.button("Add Row").clicked() {
            // Continues the last step in time, as a new row in the table starts empty.
            let next = match points.as_slice() {
                [.., a, b] => [b[0] + (b[0] - a[0]).max(0.0), b[1]],
                [b] => [b[0] + 1.0, b[1]],
                [] => [0.0, 0.0],
            };
            points.push(next);
        }
        if ui
            .button("Paste from Clipboard")
            .on_hover_text(
                "Replaces the table with two columns, time and value, from the \
                 clipboard, e.g. from a spreadsheet. Ctrl+V over the \
                 table does the same.",
            )
            .clicked()
        {
            view.awaiting_paste = true;
            ui.ctx()
                .send_viewport_cmd(egui::ViewportCommand::RequestPaste);
        }
        if ui.button("Sort by Time").clicked() {
            points.sort_by(|a, b| a[0].total_cmp(&b[0]));
        }
    });
    // Ctrl+V over the table, while no text field takes it.
    let typing = ui.ctx().memory(|m| m.focused().is_some());
    let pasted = ui.input(|i| {
        i.events.iter().find_map(|e| match e {
            egui::Event::Paste(text) => Some(text.clone()),
            _ => None,
        })
    });
    if let Some(text) = pasted
        && (view.awaiting_paste || (table_hovered && !typing))
    {
        view.awaiting_paste = false;
        let pasted = parse_points(&text, units, value.map(|(_, q)| q));
        view.paste_note = Some(if pasted.is_empty() {
            "The clipboard contains no number pairs.".into()
        } else {
            format!("{} points pasted.", pasted.len())
        });
        if !pasted.is_empty() {
            *points = pasted;
        }
    }
    if let Some(note) = &view.paste_note {
        ui.weak(note);
    }
    // The curve in the units of the table.
    let (times, values): (Vec<f64>, Vec<f64>) = (points.iter())
        .map(|&[t, a]| {
            let value = value.map_or(a, |(_, q)| units.shown(a, q));
            (units.shown(t, Quantity::Time), value)
        })
        .unzip();
    crate::features::plot(ui, &times, &values, &time_header, true, 220.0);
}

/// The checks of the dialog's OK.
pub fn validate(amplitude: &Amplitude) -> Result<(), String> {
    match amplitude.points_problem() {
        Some(problem) => Err(format!("{problem}.")),
        None => Ok(()),
    }
}

fn time_span_label(span: AmplitudeTime) -> &'static str {
    match span {
        AmplitudeTime::Step => "Step time",
        AmplitudeTime::Total => "Total time",
    }
}

/// Time and amplitude from pasted text: one point per line, the two values separated by a
/// tab, a semicolon, spaces or a comma. Lines that are not two numbers, such as a header,
/// are skipped; a decimal comma is fine when the values are separated otherwise.
fn parse_points(text: &str, units: UnitSystem, value: Option<Quantity>) -> Vec<[f64; 2]> {
    text.lines()
        .filter_map(|line| {
            let line = line.trim();
            let fields: Vec<&str> = if line.contains(['\t', ';']) {
                line.split(['\t', ';']).collect()
            } else if line.split_whitespace().count() >= 2 {
                line.split_whitespace().collect()
            } else {
                line.split(',').collect()
            };
            let fields: Vec<&str> = (fields.into_iter())
                .map(str::trim)
                .filter(|f| !f.is_empty())
                .collect();
            match fields.as_slice() {
                [time, factor] => {
                    let time = units.parse_value(time, Quantity::Time).ok()?;
                    let value = match value {
                        Some(quantity) => units.parse_value(factor, quantity).ok()?,
                        None => parse_number(factor)?,
                    };
                    Some([time, value])
                }
                _ => None,
            }
        })
        .collect()
}

/// The amplitude choice of a boundary condition or load: "Default" or one of the
/// model's amplitudes.
pub fn amplitude_row(
    ui: &mut Ui,
    label: &str,
    id: &str,
    reference: &mut Option<String>,
    fe: &FeModel,
) {
    amplitude_row_with(ui, label, id, reference, fe, |_| {});
}

/// [`amplitude_row`] with more widgets right of the choice, such as the motion preview's play
/// button.
pub fn amplitude_row_with(
    ui: &mut Ui,
    label: &str,
    id: &str,
    reference: &mut Option<String>,
    fe: &FeModel,
    extra: impl FnOnce(&mut Ui),
) {
    ui.label(label);
    let text = reference.clone().unwrap_or_else(|| DEFAULT.into());
    ui.horizontal(|ui| {
        egui::ComboBox::from_id_salt(id)
            .selected_text(text)
            .width(crate::dock::fit_width(ui, 200.0))
            .show_ui(ui, |ui| {
                ui.selectable_value(reference, None, DEFAULT);
                for amplitude in &fe.amplitudes {
                    let name = Some(amplitude.name.clone());
                    ui.selectable_value(reference, name, &amplitude.name);
                }
            });
        extra(ui);
    });
    ui.end_row();
    if let Some(name) = reference.as_deref()
        && fe.amplitude(name).is_none()
    {
        ui.label("");
        ui.colored_label(
            crate::style::error(),
            format!("Amplitude {name} does not exist"),
        );
        ui.end_row();
    }
}

/// The name for no amplitude: CalculiX's ramp in static steps, the full value at once
/// in heat transfer steps.
const DEFAULT: &str = "Default";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pastes_spreadsheet_columns() {
        let units = UnitSystem::default();
        let text = "Time\tAmplitude\n0\t0\n0,5\t1,5\n1\t1\n";
        assert_eq!(
            parse_points(text, units, None),
            [[0.0, 0.0], [0.5, 1.5], [1.0, 1.0]]
        );
        assert_eq!(
            parse_points("0, 0\n2, 1", units, None),
            [[0.0, 0.0], [2.0, 1.0]]
        );
        assert_eq!(
            parse_points("1 2\n3;4", units, None),
            [[1.0, 2.0], [3.0, 4.0]]
        );
        assert!(parse_points("Hello", units, None).is_empty());
    }
}
