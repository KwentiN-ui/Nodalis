//! The timeline below the 3D view of the results: one segment per step of the FE model, as
//! long as its time period or, for a step without time such as a frequency step, as wide as
//! its modes need; a tick or dot per increment, mode or position; cutbacks, steps taken over
//! from another analysis and a step the solver did not finish marked. Clicking or dragging
//! along it shows the increment under the pointer.

use egui::{Align2, Color32, FontId, Pos2, Rect, Sense, Shape, Stroke, Ui, pos2, vec2};
use plx_results::steps::{ResultStep, StepFamily};

use crate::results::{ResultsView, format_value};

/// Height of the timeline's strip.
pub const HEIGHT: f32 = 62.0;
const GAP: f32 = 4.0;
const BAR_TOP: f32 = 20.0;
const BAR_HEIGHT: f32 = 14.0;
/// Width of a segment per entry of a step without time, and its limits.
const ENTRY_WIDTH: f32 = 26.0;
const UNTIMED: std::ops::RangeInclusive<f32> = 90.0..=260.0;
const MIN_TIMED: f32 = 60.0;

/// Where a step and its entries lie along the timeline, in points from its left end.
#[derive(Clone, Debug, PartialEq)]
pub struct Segment {
    pub step: usize,
    pub left: f32,
    pub right: f32,
    /// For a step the solver did not finish: where its last increment lies.
    pub reached: Option<f32>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Layout {
    pub segments: Vec<Segment>,
    /// Each increment of the results with its position.
    pub entries: Vec<(usize, f32)>,
}

impl Layout {
    /// The increment nearest to `x`.
    pub fn nearest(&self, x: f32) -> Option<usize> {
        (self.entries.iter())
            .min_by(|a, b| (a.1 - x).abs().total_cmp(&(b.1 - x).abs()))
            .map(|&(index, _)| index)
    }

    pub fn x_of(&self, index: usize) -> Option<f32> {
        (self.entries.iter())
            .find(|(i, _)| *i == index)
            .map(|&(_, x)| x)
    }
}

/// Lays the steps of `view` out over `width` points.
pub fn layout(view: &ResultsView, width: f32) -> Layout {
    let steps = &view.steps.steps;
    if steps.is_empty() {
        return Layout::default();
    }
    let duration = |s: &ResultStep| (s.end_time - s.start_time).max(0.0);
    let timed = |s: &ResultStep| s.is_timed() && duration(s) > 0.0 && !s.has_positions();
    let fixed = |s: &ResultStep| {
        let entries = if s.has_positions() {
            s.solver_steps.len()
        } else {
            s.increments.len()
        };
        (entries as f32 * ENTRY_WIDTH).clamp(*UNTIMED.start(), *UNTIMED.end())
    };
    let gaps = GAP * (steps.len() - 1) as f32;
    let fixed_total: f32 = steps.iter().filter(|s| !timed(s)).map(fixed).sum();
    let timed_count = steps.iter().filter(|s| timed(s)).count();
    let total_time: f64 = steps.iter().filter(|s| timed(s)).map(duration).sum();
    let free = width - gaps - fixed_total;
    // Without room for the timed steps, every step gets the same width.
    let even = free < MIN_TIMED * timed_count as f32 || (timed_count == 0 && free < 0.0);
    let widths: Vec<f32> = steps
        .iter()
        .map(|s| {
            if even {
                ((width - gaps) / steps.len() as f32).max(1.0)
            } else if timed(s) {
                (free * (duration(s) / total_time) as f32).max(MIN_TIMED)
            } else if timed_count == 0 {
                // Steps without time fill the width by their share of it.
                fixed(s) * (width - gaps) / fixed_total
            } else {
                fixed(s)
            }
        })
        .collect();
    // Minimum widths can overshoot: the whole is scaled back to the width.
    let sum: f32 = widths.iter().sum::<f32>() + gaps;
    let scale = if sum > width {
        (width - gaps) / (sum - gaps)
    } else {
        1.0
    };
    let mut layout = Layout::default();
    let mut left = 0.0;
    for (index, (step, width)) in steps.iter().zip(widths).enumerate() {
        let width = width * scale;
        let right = left + width;
        let count = step.increments.len();
        for (k, &increment) in step.increments.iter().enumerate() {
            let fraction = if step.has_positions() {
                let solver = view.increments[increment].step;
                let position = step.solver_steps.iter().position(|&s| s == solver);
                (position.unwrap_or(0) as f32 + 0.5) / step.solver_steps.len() as f32
            } else if timed(step) {
                let total = view.steps.place(increment).and_then(|p| p.total_time);
                match total {
                    Some(t) => ((t - step.start_time) / duration(step)).clamp(0.0, 1.0) as f32,
                    None => (k + 1) as f32 / count as f32,
                }
            } else {
                (k as f32 + 0.5) / count as f32
            };
            layout.entries.push((increment, left + fraction * width));
        }
        let reached = (!step.converged)
            .then(|| step.increments.last().and_then(|&i| layout.x_of(i)))
            .flatten();
        layout.segments.push(Segment {
            step: index,
            left,
            right,
            reached,
        });
        left = right + GAP;
    }
    layout
}

/// The colour of a step's segment.
fn family_color(family: StepFamily) -> Color32 {
    match family {
        StepFamily::Mechanical => Color32::from_rgb(79, 127, 181),
        StepFamily::Dynamic => Color32::from_rgb(56, 146, 140),
        StepFamily::Thermal => Color32::from_rgb(196, 108, 58),
        StepFamily::Modal => Color32::from_rgb(120, 95, 165),
        StepFamily::Other => Color32::from_rgb(120, 120, 120),
    }
}

/// Shows the timeline in `ui`; returns the increment clicked or dragged to.
pub fn show(ui: &mut Ui, view: &ResultsView) -> Option<usize> {
    let margin = 10.0;
    let (rect, response) =
        ui.allocate_exact_size(vec2(ui.available_width(), HEIGHT), Sense::click_and_drag());
    let inner = rect.shrink2(vec2(margin, 0.0));
    let layout = layout(view, inner.width());
    let painter = ui.painter_at(rect);
    let text = ui.visuals().text_color();
    let weak = ui.visuals().weak_text_color();
    let font = FontId::proportional(11.5);
    let small = FontId::proportional(10.5);
    let x = |v: f32| inner.left() + v;
    let bar_y = rect.top() + BAR_TOP;
    let current = view.current_step_index();
    let time_unit = view.units.unit(plx_model::Quantity::Time);
    let hertz = view.units.unit(plx_model::Quantity::Frequency);
    for segment in &layout.segments {
        let step = &view.steps.steps[segment.step];
        let bar = Rect::from_min_max(
            pos2(x(segment.left), bar_y),
            pos2(x(segment.right), bar_y + BAR_HEIGHT),
        );
        let base = family_color(step.category.family());
        let shown = current == Some(segment.step);
        let fill = if step.inherited_from.is_some() {
            Color32::from_gray(if crate::style::dark() { 70 } else { 170 })
        } else if shown {
            base
        } else {
            base.gamma_multiply(0.6)
        };
        if step.increments.is_empty() {
            dashed_outline(&painter, bar, base);
        } else if let Some(reached) = segment.reached {
            dashed_outline(&painter, bar, base);
            let done = Rect::from_min_max(bar.min, pos2(x(reached), bar.max.y));
            painter.rect_filled(done, 3.0, fill);
            painter.line_segment(
                [
                    pos2(x(reached) + 2.0, bar.top() - 3.0),
                    pos2(x(reached) + 2.0, bar.bottom() + 3.0),
                ],
                Stroke::new(3.0, crate::style::error()),
            );
        } else {
            painter.rect_filled(bar, 3.0, fill);
            if step.inherited_from.is_some() {
                hatch(&painter, bar);
            }
        }
        // The name above the segment, cut to its width.
        let mut name = format!("{} {}", segment.step + 1, step.name);
        if let Some(source) = &step.inherited_from {
            name.push_str(&format!(" (from {source})"));
        }
        let color = if shown { text } else { weak };
        let galley = fit(ui, &name, font.clone(), color, bar.width());
        painter.galley(pos2(bar.left() + 2.0, rect.top() + 3.0), galley, color);
        if segment.reached.is_some() {
            let galley = ui.painter().layout_no_wrap(
                "not converged".into(),
                small.clone(),
                crate::style::error(),
            );
            let at = pos2(
                (x(segment.reached.unwrap_or(0.0)) + 6.0).min(bar.right() - galley.size().x),
                bar.bottom() + 3.0,
            );
            painter.galley(at, galley, crate::style::error());
        }
        // The values below: the times at the step's ends, or the first and last frequency.
        let below = bar.bottom() + 4.0;
        if step.is_timed() && !step.has_positions() && !step.increments.is_empty() {
            let start = format!("{} {time_unit}", format_value(step.start_time as f32));
            painter.text(
                pos2(bar.left(), below),
                Align2::LEFT_TOP,
                start.trim(),
                small.clone(),
                weak,
            );
            if Some(segment.step + 1) == Some(layout.segments.len()) {
                let end = format!("{} {time_unit}", format_value(step.end_time as f32));
                painter.text(
                    pos2(bar.right(), below),
                    Align2::RIGHT_TOP,
                    end.trim(),
                    small.clone(),
                    weak,
                );
            }
        } else if let (Some(&first), Some(&last)) =
            (step.increments.first(), step.increments.last())
            && step.category.family() == StepFamily::Modal
        {
            let value = |i: usize| format_value(view.increments[i].value as f32);
            painter.text(
                pos2(bar.left(), below),
                Align2::LEFT_TOP,
                value(first),
                small.clone(),
                weak,
            );
            if last != first {
                let label = format!("{} {hertz}", value(last));
                painter.text(
                    pos2(bar.right(), below),
                    Align2::RIGHT_TOP,
                    label.trim(),
                    small.clone(),
                    weak,
                );
            }
        }
        // The entries: ticks over time, dots for modes, diamonds for positions.
        let mark = if shown {
            Color32::from_gray(235)
        } else {
            Color32::from_gray(205)
        };
        let cy = bar.center().y;
        let mut shown_positions = Vec::new();
        for &increment in &step.increments {
            let Some(ex) = layout.x_of(increment).map(x) else {
                continue;
            };
            if step.has_positions() {
                if !shown_positions.contains(&(ex as i32)) {
                    shown_positions.push(ex as i32);
                    let d = 4.5;
                    painter.add(Shape::convex_polygon(
                        vec![
                            pos2(ex, cy - d),
                            pos2(ex + d, cy),
                            pos2(ex, cy + d),
                            pos2(ex - d, cy),
                        ],
                        mark,
                        Stroke::NONE,
                    ));
                }
            } else if step.is_timed() {
                painter.line_segment(
                    [pos2(ex, bar.top() + 2.0), pos2(ex, bar.bottom() - 2.0)],
                    Stroke::new(1.2, mark),
                );
            } else {
                painter.circle_filled(pos2(ex, cy), 2.8, mark);
            }
            // A cutback under its increment.
            let attempts = view.steps.place(increment).and_then(|p| p.attempts);
            if attempts.is_some_and(|a| a > 1) {
                let t = bar.bottom() + 1.0;
                painter.add(Shape::convex_polygon(
                    vec![
                        pos2(ex, t),
                        pos2(ex + 4.0, t + 6.0),
                        pos2(ex - 4.0, t + 6.0),
                    ],
                    crate::style::warning(),
                    Stroke::NONE,
                ));
            }
        }
    }
    // The shown increment.
    if let Some(cx) = layout.x_of(view.increment).map(x) {
        let marker = Rect::from_center_size(
            pos2(cx, bar_y + BAR_HEIGHT * 0.5),
            vec2(7.0, BAR_HEIGHT + 10.0),
        );
        painter.rect_filled(marker, 2.0, Color32::WHITE);
        painter.rect_stroke(
            marker,
            2.0,
            Stroke::new(1.0, Color32::from_gray(60)),
            egui::StrokeKind::Inside,
        );
    }
    let pointed = |pos: Option<Pos2>| pos.and_then(|p| layout.nearest(p.x - inner.left()));
    let response = response.on_hover_ui_at_pointer(|ui| {
        if let Some(index) = pointed(ui.ctx().pointer_hover_pos()) {
            ui.label(tooltip(view, index));
        }
    });
    if response.clicked() || response.dragged() {
        return pointed(response.interact_pointer_pos()).filter(|&i| i != view.increment);
    }
    None
}

/// What the tooltip says about an increment: its step, entry, times and attempts.
pub fn tooltip(view: &ResultsView, index: usize) -> String {
    let mut lines = Vec::new();
    if let Some(step) = view.steps.place(index).map(|p| p.step) {
        lines.push(view.step_title(step));
    }
    let count = (view.steps.place(index))
        .and_then(|p| view.entry_count(p.step))
        .map_or_else(String::new, |n| format!(" of {n}"));
    let mut entry = format!("{}{count}", view.entry_label(index));
    let step = view.steps.step_of(index);
    if step.is_some_and(|s| s.category.family() == StepFamily::Modal) {
        let hertz = view.units.unit(plx_model::Quantity::Frequency);
        let value = format_value(view.increments[index].value as f32);
        entry.push_str(&match view.increments[index].kind {
            plx_results::AnalysisKind::Buckling => format!(": factor {value}"),
            _ => format!(": {value} {hertz}"),
        });
    }
    lines.push(entry);
    if let Some(place) = view.steps.place(index) {
        let unit = view.units.unit(plx_model::Quantity::Time);
        if let (Some(step_time), Some(total)) = (place.step_time, place.total_time) {
            lines.push(format!(
                "Step time {} {unit}, total time {} {unit}",
                format_value(step_time as f32),
                format_value(total as f32)
            ));
        }
        match (place.attempts, place.iterations) {
            (Some(attempts), Some(iterations)) if attempts > 1 => lines.push(format!(
                "Cut back: {attempts} attempts, {iterations} iterations"
            )),
            (_, Some(iterations)) => lines.push(format!("{iterations} iterations")),
            _ => {}
        }
    }
    lines.join("\n")
}

/// `text` laid out in one line, cut with an ellipsis to `width`.
fn fit(
    ui: &Ui,
    text: &str,
    font: FontId,
    color: Color32,
    width: f32,
) -> std::sync::Arc<egui::Galley> {
    let mut job = egui::text::LayoutJob::simple_singleline(text.to_owned(), font, color);
    job.wrap = egui::text::TextWrapping::truncate_at_width(width.max(10.0));
    ui.painter().layout_job(job)
}

/// The outline of a segment without results, dashed.
fn dashed_outline(painter: &egui::Painter, bar: Rect, color: Color32) {
    let stroke = Stroke::new(1.0, color);
    let corners = [
        bar.left_top(),
        bar.right_top(),
        bar.right_bottom(),
        bar.left_bottom(),
        bar.left_top(),
    ];
    for pair in corners.windows(2) {
        painter.extend(Shape::dashed_line(&[pair[0], pair[1]], stroke, 4.0, 3.0));
    }
}

/// Diagonal stripes over a segment taken over from another analysis.
fn hatch(painter: &egui::Painter, bar: Rect) {
    let painter = painter.with_clip_rect(bar.intersect(painter.clip_rect()));
    let stroke = Stroke::new(2.0, Color32::from_black_alpha(40));
    let mut x = bar.left() - bar.height();
    while x < bar.right() {
        painter.line_segment(
            [pos2(x, bar.bottom()), pos2(x + bar.height(), bar.top())],
            stroke,
        );
        x += 6.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_results::steps::{StepCategory, StepRecord};
    use plx_results::{AnalysisKind, Increment};

    fn inc(step: u32, increment: u32, kind: AnalysisKind, value: f64) -> Increment {
        Increment {
            step,
            increment,
            kind,
            value,
            fields: Vec::new(),
            nodal_diameter: None,
        }
    }

    fn record(
        name: &str,
        category: StepCategory,
        period: Option<f64>,
        steps: &[u32],
    ) -> StepRecord {
        StepRecord {
            name: name.into(),
            category,
            nlgeom: false,
            perturbation: false,
            time_period: period,
            solver_steps: steps.to_vec(),
            inherited_from: None,
        }
    }

    fn view() -> ResultsView {
        let mut increments = Vec::new();
        for k in 1..=4 {
            increments.push(inc(1, k, AnalysisKind::Static, 0.25 * k as f64));
        }
        for k in 1..=2 {
            increments.push(inc(2, k, AnalysisKind::Static, 1.0 + 1.5 * k as f64));
        }
        for k in 1..=3 {
            increments.push(inc(3, k, AnalysisKind::Frequency, 100.0 * k as f64));
        }
        let mut view = ResultsView::new(increments, None);
        view.set_step_records(
            &[
                record("Preload", StepCategory::Static, Some(1.0), &[1]),
                record("Load", StepCategory::Static, Some(3.0), &[2]),
                record("Modes", StepCategory::Frequency, None, &[3]),
            ],
            &[],
            None,
        );
        view
    }

    #[test]
    fn timed_steps_take_the_width_by_their_time_and_modes_a_fixed_width() {
        let view = view();
        let layout = layout(&view, 1000.0);
        let widths: Vec<f32> = layout.segments.iter().map(|s| s.right - s.left).collect();
        // The modes get their minimum width; the rest is shared 1 : 3.
        assert_eq!(widths[2], 90.0);
        let free = 1000.0 - 2.0 * GAP - 90.0;
        assert!((widths[0] - free * 0.25).abs() < 1e-3, "{widths:?}");
        assert!((widths[1] - free * 0.75).abs() < 1e-3, "{widths:?}");
        // The last increment of the first step lies at its end, the modes in the middle of
        // their thirds.
        assert!((layout.x_of(3).unwrap() - widths[0]).abs() < 1e-3);
        let modes = &layout.segments[2];
        assert!((layout.x_of(7).unwrap() - (modes.left + 45.0)).abs() < 1e-3);
        assert_eq!(layout.nearest(modes.right), Some(8));
    }

    #[test]
    fn a_narrow_timeline_gives_every_step_the_same_width() {
        let layout = layout(&view(), 200.0);
        let widths: Vec<f32> = layout.segments.iter().map(|s| s.right - s.left).collect();
        assert!(
            widths.iter().all(|w| (w - widths[0]).abs() < 1e-3),
            "{widths:?}"
        );
        assert!(layout.segments[2].right <= 200.0 + 1e-3);
    }

    #[test]
    fn the_tooltip_names_step_entry_and_times() {
        let view = view();
        assert_eq!(
            tooltip(&view, 4),
            "2 Load\nIncrement 1 of 2\nStep time 1.5 s, total time 2.5 s"
        );
        assert_eq!(tooltip(&view, 8), "3 Modes\nMode 3 of 3: 300 Hz");
    }
}
