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
    let time_unit = units.unit(Quantity::Time);
    egui::Grid::new("amplitude form")
        .num_columns(2)
        .spacing([12.0, 6.0])
        .show(ui, |ui| {
            ui.label("Name");
            ui.add(egui::TextEdit::singleline(&mut amplitude.name).desired_width(200.0));
            ui.end_row();
            ui.label("Time span");
            egui::ComboBox::from_id_salt("amplitude time span")
                .selected_text(time_span_label(amplitude.time_span))
                .width(200.0)
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
    let time_header = if time_unit.is_empty() {
        "Time".to_string()
    } else {
        format!("Time [{time_unit}]")
    };
    let mut remove = None;
    let table = egui::ScrollArea::vertical()
        .id_salt("amplitude points")
        .max_height(180.0)
        .auto_shrink([false, true])
        .show(ui, |ui| {
            egui::Grid::new("amplitude points grid")
                .num_columns(4)
                .striped(true)
                .spacing([12.0, 4.0])
                .show(ui, |ui| {
                    ui.strong("#");
                    ui.strong(&time_header);
                    ui.strong("Amplitude [/]");
                    ui.label("");
                    ui.end_row();
                    let removable = amplitude.points.len() > 1;
                    for (i, [time, factor]) in amplitude.points.iter_mut().enumerate() {
                        ui.label((i + 1).to_string());
                        ui.add(
                            numeric::without_unit(time, units, Quantity::Time)
                                .speed(0.01)
                                .min_decimals(1),
                        );
                        ui.add(numeric::drag_value(factor).speed(0.01).min_decimals(1));
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
        amplitude.points.remove(i);
    }
    let table_hovered = ui.rect_contains_pointer(table.inner_rect);
    ui.horizontal(|ui| {
        if ui.button("Add Row").clicked() {
            // Continues the last step in time, as a new row in the table starts empty.
            let next = match amplitude.points.as_slice() {
                [.., a, b] => [b[0] + (b[0] - a[0]).max(0.0), b[1]],
                [b] => [b[0] + 1.0, b[1]],
                [] => [0.0, 0.0],
            };
            amplitude.points.push(next);
        }
        if ui
            .button("Paste from Clipboard")
            .on_hover_text(
                "Replaces the table with two columns time and amplitude from the \
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
            amplitude.points.sort_by(|a, b| a[0].total_cmp(&b[0]));
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
        let points = parse_points(&text, units);
        view.paste_note = Some(if points.is_empty() {
            "The clipboard contains no number pairs.".into()
        } else {
            format!("{} points pasted.", points.len())
        });
        if !points.is_empty() {
            amplitude.points = points;
        }
    }
    if let Some(note) = &view.paste_note {
        ui.weak(note);
    }
    if let Some(problem) = amplitude.points_problem() {
        ui.colored_label(crate::style::error(), problem);
    }
    let (times, factors): (Vec<f64>, Vec<f64>) =
        amplitude.points.iter().map(|[t, a]| (*t, *a)).unzip();
    crate::features::plot(ui, &times, &factors, &time_header, true, 220.0);
    ui.weak(
        "Linear between the points, constant before the first and after the last.\n\
         Without an amplitude a load ramps up linearly in a static step.",
    );
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
fn parse_points(text: &str, units: UnitSystem) -> Vec<[f64; 2]> {
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
                    Some([time, parse_number(factor)?])
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
            .width(200.0)
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
            parse_points(text, units),
            [[0.0, 0.0], [0.5, 1.5], [1.0, 1.0]]
        );
        assert_eq!(parse_points("0, 0\n2, 1", units), [[0.0, 0.0], [2.0, 1.0]]);
        assert_eq!(parse_points("1 2\n3;4", units), [[1.0, 2.0], [3.0, 4.0]]);
        assert!(parse_points("Hello", units).is_empty());
    }
}
