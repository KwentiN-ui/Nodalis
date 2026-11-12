//! The convergence of a solve as it goes on, plotted beside the monitor's output: the
//! residual force and the correction of every Newton iteration over the limits they have to
//! reach, the time and size of the increments with the cutbacks, and the iterations of the
//! iterative linear solver. Like the solution information of other FE programs it shows
//! where a hard nonlinear run struggles while it is still running.

use egui::{Color32, Ui};
use egui_plot::{
    GridInput, GridMark, HLine, HoverPosition, Legend, Line, LineStyle, MarkerShape, Plot,
    PlotPoints, Points, VLine,
};
use plx_solve::monitor::{
    ConvergenceLog, Cutback, IncrementPoint, IterationPoint, LiveLog, StepRecord,
};

use crate::xy_plot;

/// What the plot shows.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum View {
    /// Residual and correction of the Newton iterations over their limits.
    #[default]
    Residuals,
    /// Total time reached per increment.
    Time,
    /// Size of the increments, of the cut back attempts too.
    IncrementSize,
    /// Relative residual of the iterative linear solver per iteration.
    LinearSolver,
}

impl View {
    const ALL: [View; 4] = [
        View::Residuals,
        View::Time,
        View::IncrementSize,
        View::LinearSolver,
    ];

    fn label(self) -> &'static str {
        match self {
            View::Residuals => "Residuals",
            View::Time => "Time",
            View::IncrementSize => "Increment Size",
            View::LinearSolver => "Linear Solver",
        }
    }

    fn tip(self) -> &'static str {
        match self {
            View::Residuals => {
                "Largest residual force and largest correction of every Newton iteration, \
                 each over the limit it has to reach: an increment converges when both are \
                 below 1"
            }
            View::Time => "The total time each converged increment reached; red: cut back",
            View::IncrementSize => {
                "The size of each converged increment; red: attempts that were cut back"
            }
            View::LinearSolver => {
                "The relative residual of the iterative linear solver per iteration, for the \
                 solve going on and the one before"
            }
        }
    }
}

/// Whether the monitors show the plot beside the output; one switch for all of them, kept
/// with the window layout.
pub fn shown(ctx: &egui::Context) -> bool {
    ctx.data_mut(|d| *d.get_persisted_mut_or(shown_id(), true))
}

fn shown_id() -> egui::Id {
    egui::Id::new("convergence plot shown")
}

fn view_id() -> egui::Id {
    egui::Id::new("convergence plot view")
}

/// The button of a monitor's row that switches the plot on and off.
pub fn button(ui: &mut Ui, log: Option<&LiveLog>) {
    let ctx = ui.ctx().clone();
    let mut on = shown(&ctx);
    if ui
        .add_enabled(log.is_some(), egui::Button::selectable(on, "Convergence"))
        .on_hover_text(
            "Plots the residuals of the iterations, the increments and the linear solver \
             while the run goes on",
        )
        .on_disabled_hover_text("The plot shows once the solver runs")
        .clicked()
    {
        on = !on;
        ctx.data_mut(|d| d.insert_persisted(shown_id(), on));
    }
}

/// The output of a monitor with the plot to its right, when there is a log and the plot is
/// switched on.
pub fn with_output(ui: &mut Ui, log: Option<&LiveLog>, output: impl FnOnce(&mut Ui)) {
    let Some(log) = log.filter(|_| shown(ui.ctx())) else {
        output(ui);
        return;
    };
    let width = ui.available_width();
    let plot_width = (width * 0.5).clamp(320.0, 760.0).min(width - 160.0);
    if plot_width < 240.0 {
        output(ui);
        return;
    }
    ui.horizontal_top(|ui| {
        let height = ui.available_height();
        let output_width = (ui.available_width() - plot_width - 12.0).max(80.0);
        ui.allocate_ui_with_layout(
            egui::vec2(output_width, height),
            egui::Layout::top_down(egui::Align::Min),
            output,
        );
        ui.separator();
        ui.allocate_ui_with_layout(
            egui::vec2(ui.available_width(), height),
            egui::Layout::top_down(egui::Align::Min),
            |ui| panel(ui, log),
        );
    });
}

/// The plot with its view picker and a line on where the run is.
pub fn panel(ui: &mut Ui, log: &LiveLog) {
    let ctx = ui.ctx().clone();
    let mut view: View = ctx.data_mut(|d| *d.get_persisted_mut_or_default(view_id()));
    // Copied out, so that the solver is not held up while the plot is drawn.
    let data = Data::new(&log.lock());
    ui.horizontal(|ui| {
        egui::ComboBox::from_id_salt("convergence view")
            .selected_text(view.label())
            .show_ui(ui, |ui| {
                for choice in View::ALL {
                    if ui
                        .selectable_label(view == choice, choice.label())
                        .on_hover_text(choice.tip())
                        .clicked()
                    {
                        view = choice;
                        ctx.data_mut(|d| d.insert_persisted(view_id(), view));
                    }
                }
            })
            .response
            .on_hover_text(view.tip());
        if let Some(fraction) = data.fraction {
            ui.add(
                egui::ProgressBar::new(fraction as f32)
                    .desired_width(70.0)
                    .text(format!("{:.0} %", 100.0 * fraction)),
            )
            .on_hover_text("How far the step got through its time period");
        }
        if let Some(text) = &data.status {
            ui.add(egui::Label::new(text.as_str()).truncate());
        }
    });
    let height = ui.available_height().max(120.0);
    match view {
        View::Residuals => residuals(ui, &data, height),
        View::Time => increments(ui, &data, height, false),
        View::IncrementSize => increments(ui, &data, height, true),
        View::LinearSolver => linear(ui, &data, height),
    }
}

/// Series longer than this are thinned for the plot (every n-th point): the plot has fewer
/// pixels than that anyway.
const SHOWN_POINTS: usize = 6000;

/// Iterations, increments and steps of the plot, copied from the log.
struct Data {
    iterations: Vec<IterationPoint>,
    increments: Vec<IncrementPoint>,
    cutbacks: Vec<Cutback>,
    steps: Vec<StepRecord>,
    linear: Option<(f64, Vec<[f64; 2]>)>,
    linear_before: Option<Vec<[f64; 2]>>,
    solves: Vec<plx_solve::monitor::LinearSolve>,
    fraction: Option<f64>,
    status: Option<String>,
}

/// Every n-th of `points` (and the last one) so that at most [`SHOWN_POINTS`] are left.
fn thinned<T: Clone>(points: &[T]) -> Vec<T> {
    let n = points.len();
    let stride = n.div_ceil(SHOWN_POINTS).max(1);
    (points.iter().enumerate())
        .filter(|(i, _)| i % stride == 0 || i + 1 == n)
        .map(|(_, p)| p.clone())
        .collect()
}

impl Data {
    fn new(log: &ConvergenceLog) -> Self {
        let status = log.current.map(|c| {
            let name = (log.step()).map_or_else(String::new, |s| format!(" {}", s.name));
            let mut text = format!("Step {}{name}", c.step);
            if c.increment > 0 {
                text += &format!(", increment {}", c.increment);
                if c.attempt > 1 {
                    text += &format!(", attempt {}", c.attempt);
                }
                if c.iteration > 0 {
                    text += &format!(", iteration {}", c.iteration);
                }
                text += &format!(", size {}", number(c.size));
            }
            text
        });
        Self {
            iterations: thinned(&log.iterations),
            increments: thinned(&log.increments),
            cutbacks: thinned(&log.cutbacks),
            steps: log.steps.clone(),
            linear: (log.linear.as_ref()).map(|c| (c.tolerance, log10_points(&c.points))),
            linear_before: (log.linear_before.as_ref()).map(|c| log10_points(&c.points)),
            solves: log.solves.clone(),
            fraction: log.step_fraction(),
            status,
        }
    }
}

fn iteration_text(p: &IterationPoint) -> String {
    format!(
        "Step {}, increment {}, attempt {}, iteration {}\nresidual {} (criterion {})\n\
         correction {} (criterion {})",
        p.step,
        p.increment,
        p.attempt,
        p.iteration,
        number(p.residual),
        p.residual_limit.map_or("none".into(), number),
        number(p.correction),
        p.correction_limit.map_or("none".into(), number),
    )
}

fn increment_text(i: &IncrementPoint) -> String {
    format!(
        "Step {}, increment {}: {} iterations{}\nsize {}, step time {}, total time {}",
        i.step,
        i.increment,
        i.iterations,
        if i.attempts > 1 {
            format!(", {} attempts", i.attempts)
        } else {
            String::new()
        },
        number(i.size),
        number(i.step_time),
        number(i.total_time)
    )
}

fn cutback_text(c: &Cutback) -> String {
    format!(
        "Step {}, increment {}, attempt {} cut back\nsize {}: {}",
        c.step,
        c.increment,
        c.attempt,
        number(c.size),
        c.reason
    )
}

fn step_name(s: &StepRecord) -> String {
    format!("Step {} {}", s.number, s.name)
}

/// The steps as lines where they start, `x` of the plot, with their names at the top while
/// there are few.
/// `top` is the largest value of the plot, where the names go (the plot's own bounds would
/// grow with the names in them).
fn step_lines(
    plot_ui: &mut egui_plot::PlotUi,
    steps: &[StepRecord],
    x: fn(&StepRecord) -> u64,
    top: f64,
) {
    for step in steps {
        let at = x(step) as f64 + 0.5;
        if at > 1.0 {
            plot_ui.vline(
                VLine::new("", at)
                    .color(step_color())
                    .width(1.5)
                    .allow_hover(false),
            );
        }
        if steps.len() <= 12 {
            plot_ui.text(
                egui_plot::Text::new(
                    "",
                    egui_plot::PlotPoint::new(at.max(0.5), top),
                    step_name(step),
                )
                .color(step_color())
                .anchor(egui::Align2::LEFT_TOP),
            );
        }
    }
}

fn log10_points(points: &[[f64; 2]]) -> Vec<[f64; 2]> {
    (points.iter())
        .filter(|[_, y]| *y > 0.0 && y.is_finite())
        .map(|[x, y]| [*x, y.log10()])
        .collect()
}

/// A number as the monitor shows it: four significant digits.
fn number(value: f64) -> String {
    if value == 0.0 {
        "0".into()
    } else if (1e-3..1e5).contains(&value.abs()) {
        let digits = (3 - value.abs().log10().floor() as i32).max(0) as usize;
        format!("{value:.digits$}")
    } else {
        format!("{value:.3e}")
    }
}

/// A plot of the monitor: legend, grid, no zoom by scrolling (the output beside scrolls).
fn plot<'a>(id: &str, height: f32, x_label: &str, y_label: &str) -> Plot<'a> {
    Plot::new(id)
        .height(height)
        .legend(
            Legend::default()
                .follow_insertion_order(true)
                .position(egui_plot::Corner::RightTop),
        )
        .x_axis_label(x_label.to_owned())
        .y_axis_label(y_label.to_owned())
        .grid_color(crate::style::plot_grid())
        .allow_scroll(false)
        .allow_double_click_reset(true)
}

/// The y axis of `plot` in decades: the values are `log10` of what is shown.
fn decades<'a>(plot: Plot<'a>) -> Plot<'a> {
    plot.y_grid_spacer(|input: GridInput| {
        let step = input.base_step_size.ceil().max(1.0);
        let (low, high) = input.bounds;
        let mut marks = Vec::new();
        let mut value = (low / step).floor() * step;
        while value <= high {
            marks.push(GridMark {
                value,
                step_size: step,
            });
            value += step;
        }
        marks
    })
    .y_axis_formatter(|mark, _| {
        let exponent = mark.value.round() as i32;
        if exponent == 0 {
            "1".into()
        } else {
            format!("1e{exponent}")
        }
    })
}

fn cutback_color() -> Color32 {
    crate::style::error()
}

fn step_color() -> Color32 {
    crate::style::plot_text().gamma_multiply(0.6)
}

/// The largest y of `points`, 0 without any.
fn top_of<'a>(points: impl Iterator<Item = &'a [f64; 2]>) -> f64 {
    let top = points.map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max);
    if top.is_finite() { top } else { 0.0 }
}

fn empty(ui: &mut Ui, text: &str) {
    ui.add_space(20.0);
    ui.vertical_centered(|ui| ui.weak(text));
}

/// The text of the point nearest to `x` in `points`.
fn near<T>(points: &[T], x: f64, at: impl Fn(&T) -> f64) -> Option<&T> {
    points
        .iter()
        .min_by(|a, b| (at(a) - x).abs().total_cmp(&(at(b) - x).abs()))
}

fn residuals(ui: &mut Ui, data: &Data, height: f32) {
    if data.iterations.is_empty() {
        empty(
            ui,
            if data.steps.is_empty() {
                "Waiting for the first iteration"
            } else {
                "No Newton iterations so far: the steps solved are linear"
            },
        );
        return;
    }
    let series = |pick: fn(&IterationPoint) -> Option<f64>| {
        (data.iterations.iter())
            .filter_map(|p| Some([p.x as f64, pick(p)?.log10()]))
            .collect::<Vec<[f64; 2]>>()
    };
    let residual = series(IterationPoint::residual_ratio);
    let correction = series(IterationPoint::correction_ratio);
    let top = top_of(residual.iter().chain(&correction));
    let few = data.iterations.len() <= 300;
    let plot = decades(plot(
        "convergence residuals",
        height,
        "Iteration (cumulative)",
        "Value / criterion",
    ))
    .label_formatter(|hover| {
        let HoverPosition::NearDataPoint { position, .. } = hover else {
            return None;
        };
        near(&data.iterations, position.x, |p| p.x as f64).map(iteration_text)
    });
    plot.show(ui, |plot_ui| {
        plot_ui.hline(
            HLine::new("Criterion", 0.0)
                .color(crate::style::ok())
                .style(LineStyle::dashed_loose())
                .allow_hover(false),
        );
        step_lines(plot_ui, &data.steps, |s| s.iterations_x, top);
        // Converged increments as thin lines while there are few of them.
        if data.increments.len() <= 200 {
            for increment in &data.increments {
                plot_ui.vline(
                    VLine::new("", increment.iterations_x as f64 + 0.5)
                        .color(crate::style::ok().gamma_multiply(0.25))
                        .width(1.0)
                        .allow_hover(false),
                );
            }
        }
        if data.cutbacks.len() <= 400 {
            for cutback in &data.cutbacks {
                plot_ui.vline(
                    VLine::new("Cutback", cutback.iterations_x as f64 + 0.5)
                        .color(cutback_color())
                        .style(LineStyle::dashed_dense())
                        .width(1.0)
                        .allow_hover(false),
                );
            }
        }
        let curve = |plot_ui: &mut egui_plot::PlotUi, name: &str, points: Vec<[f64; 2]>, c| {
            if points.is_empty() {
                return;
            }
            let color = xy_plot::color(c);
            plot_ui.line(Line::new(name, points.clone()).color(color).width(1.5));
            if few {
                plot_ui.points(
                    Points::new(name, PlotPoints::from(points))
                        .color(color)
                        .radius(2.0),
                );
            }
        };
        curve(plot_ui, "Residual force", residual, 0);
        curve(plot_ui, "Correction", correction, 1);
    });
}

fn increments(ui: &mut Ui, data: &Data, height: f32, size: bool) {
    if data.increments.is_empty() && data.cutbacks.is_empty() {
        empty(ui, "Waiting for the first increment");
        return;
    }
    let value = |time: f64, s: f64| -> Option<f64> {
        if size {
            (s > 0.0).then(|| s.log10())
        } else {
            Some(time)
        }
    };
    let converged: Vec<[f64; 2]> = (data.increments.iter())
        .filter_map(|i| Some([i.x as f64, value(i.total_time, i.size)?]))
        .collect();
    let cut: Vec<[f64; 2]> = (data.cutbacks.iter())
        .filter_map(|c| Some([c.x as f64, value(c.total_time, c.size)?]))
        .collect();
    let top = top_of(converged.iter().chain(&cut));
    let mut plot = plot(
        if size {
            "convergence increment size"
        } else {
            "convergence time"
        },
        height,
        "Increment (cumulative)",
        if size { "Increment size" } else { "Total time" },
    )
    .label_formatter(|hover| {
        let HoverPosition::NearDataPoint {
            plot_name,
            position,
            ..
        } = hover
        else {
            return None;
        };
        if *plot_name == "Cutback" {
            near(&data.cutbacks, position.x, |c| c.x as f64).map(cutback_text)
        } else {
            near(&data.increments, position.x, |i| i.x as f64).map(increment_text)
        }
    });
    if size {
        plot = decades(plot);
    } else {
        plot = plot.include_y(0.0);
    }
    let few = converged.len() <= 300;
    plot.show(ui, |plot_ui| {
        step_lines(plot_ui, &data.steps, |s| s.increments_x, top);
        let color = xy_plot::color(0);
        if !converged.is_empty() {
            plot_ui.line(
                Line::new("Converged", converged.clone())
                    .color(color)
                    .width(1.5),
            );
            if few {
                plot_ui.points(
                    Points::new("Converged", PlotPoints::from(converged))
                        .color(color)
                        .radius(2.0),
                );
            }
        }
        if !cut.is_empty() {
            plot_ui.points(
                Points::new("Cutback", PlotPoints::from(cut))
                    .color(cutback_color())
                    .shape(MarkerShape::Cross)
                    .radius(4.0),
            );
        }
    });
}

fn linear(ui: &mut Ui, data: &Data, height: f32) {
    let Some((tolerance, points)) = &data.linear else {
        empty(
            ui,
            "No iterative linear solve so far: the direct solver needs no iterations",
        );
        return;
    };
    if let Some(last) = data.solves.last() {
        let count = data.solves.len();
        let mean = data
            .solves
            .iter()
            .map(|s| f64::from(s.iterations))
            .sum::<f64>()
            / count as f64;
        ui.weak(format!(
            "{count} solves, the last {} iterations in {:.2} s, {mean:.0} on average",
            last.iterations, last.seconds
        ));
    }
    let plot = decades(plot(
        "convergence linear solver",
        ui.available_height().min(height).max(100.0),
        "Iteration of the solve",
        "Relative residual",
    ))
    .label_formatter(|hover| {
        let HoverPosition::NearDataPoint {
            plot_name,
            position,
            ..
        } = hover
        else {
            return None;
        };
        Some(format!(
            "{plot_name}\niteration {:.0}: {}",
            position.x,
            number(10f64.powf(position.y))
        ))
    });
    plot.show(ui, |plot_ui| {
        if *tolerance > 0.0 {
            plot_ui.hline(
                HLine::new("Tolerance", tolerance.log10())
                    .color(crate::style::ok())
                    .style(LineStyle::dashed_loose())
                    .allow_hover(false),
            );
        }
        if let Some(before) = data.linear_before.as_ref().filter(|p| !p.is_empty()) {
            plot_ui.line(
                Line::new("Solve before", before.clone())
                    .color(crate::style::plot_text().gamma_multiply(0.5))
                    .style(LineStyle::dashed_dense())
                    .width(1.0),
            );
        }
        if !points.is_empty() {
            plot_ui.line(
                Line::new("Current solve", points.clone())
                    .color(xy_plot::color(0))
                    .width(1.5),
            );
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn numbers_show_four_digits() {
        assert_eq!(number(0.0), "0");
        assert_eq!(number(12.3456), "12.35");
        assert_eq!(number(0.0625), "0.06250");
        assert_eq!(number(1.5e-7), "1.500e-7");
    }
}
