//! The rows of the load dialog for an excitation: the peak force, the choice of signal with
//! its settings and a plot of the signal over the step.

use egui::Ui;
use egui_plot::{Line, Plot};
use plx_model::units::{Quantity, UnitSystem};
use plx_model::{Excitation, Field, Signal, SignalKind, Sweep};

use crate::expose;
use crate::numeric;

/// The most points of the signal plot.
const PLOT_POINTS: usize = 4000;

/// The rows of an excitation in a step of length `duration`.
pub fn excitation_rows(
    ui: &mut Ui,
    excitation: &mut Excitation,
    duration: f64,
    two_d: bool,
    units: UnitSystem,
) {
    let count = if two_d { 2 } else { 3 };
    if two_d {
        excitation.value[2] = 0.0;
    }
    let labels = ["F1", "F2", "F3"].into_iter().zip(Field::VECTOR);
    for (value, (label, field)) in excitation.value.iter_mut().zip(labels).take(count) {
        ui.label(label)
            .on_hover_text("Peak force: the signal swings between it and its negative.");
        expose::field(ui, field, |ui| {
            ui.add(numeric::quantity(value, units, Quantity::Force).speed(1.0))
        });
        ui.end_row();
    }
    ui.label("");
    ui.checkbox(&mut excitation.each_node, "Apply to all nodes individually");
    ui.end_row();

    ui.label("Signal");
    let current = excitation.signal.kind();
    egui::ComboBox::from_id_salt("excitation signal")
        .selected_text(current.label())
        .width(crate::dock::fit_width(ui, 200.0))
        .show_ui(ui, |ui| {
            for kind in SignalKind::ALL {
                if ui.selectable_label(current == kind, kind.label()).clicked() && current != kind {
                    excitation.signal = kind.signal(&excitation.signal);
                }
            }
        });
    ui.end_row();

    let time = |ui: &mut Ui, label: &str, value: &mut f64, tip: &str| {
        ui.label(label).on_hover_text(tip);
        ui.add(numeric::scaled(
            value,
            units,
            Quantity::Time,
            duration * 0.1,
        ));
        ui.end_row();
    };
    let frequency = |ui: &mut Ui, label: &str, value: &mut f64| {
        ui.label(label);
        ui.add(numeric::scaled(value, units, Quantity::Frequency, 1.0));
        ui.end_row();
    };
    match &mut excitation.signal {
        Signal::Impulse { time: at, width } => {
            time(ui, "Time", at, "Step time the impulse starts.");
            time(
                ui,
                "Width",
                width,
                "Length of the half sine that stands in for the Dirac impulse: the shorter, \
                 the higher the frequencies it excites.",
            );
        }
        Signal::Sine {
            frequency: f,
            phase,
            start,
            end,
        } => {
            frequency(ui, "Frequency", f);
            ui.label("Phase");
            ui.add(numeric::drag_value(phase).speed(1.0).suffix(" deg"));
            ui.end_row();
            time(ui, "Start", start, "Step time the sine starts.");
            end_row(ui, end, duration, units);
        }
        Signal::Chirp {
            from,
            to,
            sweep,
            start,
            end,
        } => {
            frequency(ui, "Start frequency", from);
            frequency(ui, "End frequency", to);
            ui.label("Sweep").on_hover_text(
                "Linear: the same hertz per second. Logarithmic: the same factor per second, \
                 as long per octave at low as at high frequencies.",
            );
            ui.horizontal(|ui| {
                for choice in Sweep::ALL {
                    ui.radio_value(sweep, choice, choice.label());
                }
            });
            ui.end_row();
            time(ui, "Start", start, "Step time the sweep starts.");
            end_row(ui, end, duration, units);
        }
    }

    let signal = &excitation.signal;
    ui.label("");
    match signal.problem(duration) {
        Some(problem) => {
            ui.colored_label(crate::style::error(), problem);
        }
        None => {
            let mut note = format!(
                "Excites up to about {} {}",
                numeric::format_physical(fit(
                    units.shown(signal.highest_frequency(), Quantity::Frequency)
                )),
                units.unit(Quantity::Frequency)
            );
            if let Some(per_force) = signal.impulse_per_force() {
                let peak = excitation.value.iter().map(|v| v * v).sum::<f64>().sqrt();
                let impulse = per_force
                    * units.shown(peak, Quantity::Force)
                    * units.shown(1.0, Quantity::Time);
                note = format!(
                    "Impulse {} {} {}. {note}",
                    numeric::format_physical(fit(impulse)),
                    units.unit(Quantity::Force),
                    units.unit(Quantity::Time),
                );
            }
            ui.weak(format!("{note}; the time step follows it."));
        }
    }
    ui.end_row();
    if signal.problem(duration).is_none() {
        ui.label("");
        signal_plot(ui, signal, duration, units);
        ui.end_row();
    }
}

/// The end of a sine or sweep: the end of the step, or a time of its own.
fn end_row(ui: &mut Ui, end: &mut Option<f64>, duration: f64, units: UnitSystem) {
    ui.label("End")
        .on_hover_text("Step time the signal stops; without, it runs to the end of the step.");
    let mut own = end.is_some();
    let mut value = end.unwrap_or(duration);
    ui.horizontal(|ui| {
        ui.checkbox(&mut own, "");
        if own {
            ui.add(numeric::scaled(
                &mut value,
                units,
                Quantity::Time,
                duration * 0.1,
            ));
        } else {
            ui.weak("End of the step");
        }
    });
    ui.end_row();
    *end = own.then_some(value);
}

/// Six significant digits for a note.
fn fit(value: f64) -> f64 {
    if value == 0.0 || !value.is_finite() {
        return value;
    }
    let digits = 6 - value.abs().log10().ceil() as i32;
    let scale = 10f64.powi(digits);
    (value * scale).round() / scale
}

/// The signal over the step, sampled finely enough to show its oscillations.
fn signal_plot(ui: &mut Ui, signal: &Signal, duration: f64, units: UnitSystem) {
    let ir = signal.to_ir(duration);
    // An impulse is short against the step: the plot shows it with a little before and after.
    let (from, to) = match *signal {
        Signal::Impulse { time, width } => {
            ((time - width).max(0.0_f64.min(time)), time + 2.0 * width)
        }
        _ => (
            0.0_f64.min(ir.start()),
            duration.max(ir.end().min(duration * 2.0)),
        ),
    };
    let mut times = ir.corners(PLOT_POINTS);
    times.retain(|t| (from..=to).contains(t));
    // Zero before and after the signal.
    times.push(from);
    times.push(to);
    for edge in [ir.start(), ir.end()] {
        if (from..=to).contains(&edge) {
            times.push(edge);
        }
    }
    times.sort_by(f64::total_cmp);
    times.dedup();
    let points: Vec<[f64; 2]> = times
        .iter()
        .map(|&t| [units.shown(t, Quantity::Time), ir.value(t)])
        .collect();
    let width = ui.available_width().max(200.0);
    Plot::new(ui.id().with("excitation signal plot"))
        .height(130.0)
        .width(width)
        .x_axis_label(format!("Step time [{}]", units.unit(Quantity::Time)))
        .y_axis_label("Factor")
        .include_y(-1.05)
        .include_y(1.05)
        .allow_scroll(false)
        .allow_drag(false)
        .allow_zoom(false)
        .show(ui, |plot| {
            plot.line(Line::new("Signal", points).width(1.5));
        });
}
