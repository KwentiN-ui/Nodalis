//! The timeline below the 3D view of the results: one segment per step of the FE model, as
//! wide as what can be picked in it needs, its increments, modes or positions; a tick or dot per increment, mode or position; cutbacks, steps taken over
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
/// The room a step needs for each entry: a tick of an increment, a mode with its frequency
/// below it, a position of a moving load; and the least room of a step.
const TICK_WIDTH: f32 = 12.0;
const MODE_WIDTH: f32 = 56.0;
const POSITION_WIDTH: f32 = 26.0;
const MIN_STEP: f32 = 60.0;

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
    /// The increment nearest to `x` within the step under `x`, so that a click into a step
    /// shows one of its increments even where the step before has one nearer; between the
    /// steps, or in a step without increments, the nearest of all.
    pub fn nearest(&self, x: f32) -> Option<usize> {
        let nearest = |entries: &mut dyn Iterator<Item = &(usize, f32)>| {
            entries
                .min_by(|a, b| (a.1 - x).abs().total_cmp(&(b.1 - x).abs()))
                .map(|&(index, _)| index)
        };
        let segment = (self.segments.iter()).find(|s| (s.left..=s.right).contains(&x));
        segment
            .and_then(|s| {
                let inside = |e: &&(usize, f32)| (s.left..=s.right).contains(&e.1);
                nearest(&mut self.entries.iter().filter(inside))
            })
            .or_else(|| nearest(&mut self.entries.iter()))
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
    // Each step takes the width by what can be picked in it, not by its time: a static step
    // of one increment gets little, a frequency step room for the frequency of each mode.
    let need = |s: &ResultStep| {
        let need = if s.has_positions() {
            s.solver_steps.len() as f32 * POSITION_WIDTH
        } else if timed(s) {
            s.increments.len() as f32 * TICK_WIDTH
        } else {
            s.increments.len() as f32 * MODE_WIDTH
        };
        need.max(MIN_STEP)
    };
    let gaps = GAP * (steps.len() - 1) as f32;
    let needs: Vec<f32> = steps.iter().map(need).collect();
    let total: f32 = needs.iter().sum();
    let scale = (width - gaps).max(steps.len() as f32) / total;
    let widths: Vec<f32> = needs.iter().map(|n| n * scale).collect();
    let mut layout = Layout::default();
    let mut left = 0.0;
    for (index, (step, width)) in steps.iter().zip(widths).enumerate() {
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

/// Shows the timeline in `ui`; returns the increment clicked or dragged to. `hide` is set
/// when its context menu hides it.
pub fn show(ui: &mut Ui, view: &ResultsView, hide: &mut bool) -> Option<usize> {
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
        } else if step.category.family() == StepFamily::Modal {
            // The frequency of each mode below its dot, where it fits; the unit after the
            // last one.
            let mut free_from = bar.left();
            let count = step.increments.len();
            for (k, &increment) in step.increments.iter().enumerate() {
                let Some(ex) = layout.x_of(increment).map(x) else {
                    continue;
                };
                let mut label = format_value(view.increments[increment].value as f32);
                if k + 1 == count {
                    label = format!("{label} {hertz}").trim().to_string();
                }
                let galley = ui.painter().layout_no_wrap(label, small.clone(), weak);
                let w = galley.size().x;
                let left = (ex - w * 0.5).clamp(bar.left(), (bar.right() - w).max(bar.left()));
                // The last one is always written, so the range shows.
                let last = k + 1 == count;
                if left < free_from && !last {
                    continue;
                }
                painter.galley(pos2(left.max(free_from), below), galley, weak);
                free_from = left.max(free_from) + w + 6.0;
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
    response.context_menu(|ui| {
        if ui.button("Hide Timeline").clicked() {
            *hide = true;
            ui.close();
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
    fn steps_take_the_width_their_increments_and_modes_need() {
        let view = view();
        let layout = layout(&view, 1000.0);
        let widths: Vec<f32> = layout.segments.iter().map(|s| s.right - s.left).collect();
        // Both static steps need their least width, the three modes room for each frequency:
        // 60 : 60 : 168, scaled to the width.
        let scale = (1000.0 - 2.0 * GAP) / (60.0 + 60.0 + 3.0 * MODE_WIDTH);
        for (width, need) in widths.iter().zip([60.0, 60.0, 3.0 * MODE_WIDTH]) {
            assert!((width - need * scale).abs() < 1e-3, "{widths:?}");
        }
        assert!((layout.segments[2].right - 1000.0).abs() < 1e-3);
        // The last increment of the first step lies at its end, by its time; the modes in the
        // middle of their thirds.
        assert!((layout.x_of(3).unwrap() - widths[0]).abs() < 1e-3);
        let modes = &layout.segments[2];
        assert!((layout.x_of(7).unwrap() - (modes.left + widths[2] / 2.0)).abs() < 1e-3);
        assert_eq!(layout.nearest(modes.right), Some(8));
    }

    #[test]
    fn a_click_into_a_step_shows_one_of_its_increments() {
        // Two steps of one increment each, at their ends: the middle of the second step is
        // nearer to the end of the first, but belongs to the second.
        let increments = vec![
            inc(1, 1, AnalysisKind::Static, 1.0),
            inc(2, 1, AnalysisKind::Static, 2.0),
        ];
        let mut view = ResultsView::new(increments, None);
        view.set_step_records(
            &[
                record("Stiffness", StepCategory::Static, Some(1.0), &[1]),
                record("Stress", StepCategory::Static, Some(1.0), &[2]),
            ],
            &[],
            None,
        );
        let layout = layout(&view, 1000.0);
        let second = &layout.segments[1];
        assert_eq!(layout.nearest(second.left + 10.0), Some(1));
        assert_eq!(layout.nearest(layout.segments[0].left + 10.0), Some(0));
        // In the gap between the steps the nearest of all.
        assert_eq!(layout.nearest(layout.segments[0].right + 1.0), Some(0));
    }

    #[test]
    fn one_static_increment_leaves_the_room_to_the_modes() {
        let mut increments = vec![inc(1, 1, AnalysisKind::Static, 1.0)];
        for k in 1..=10 {
            increments.push(inc(2, k, AnalysisKind::Frequency, 100.0 * k as f64));
        }
        let mut view = ResultsView::new(increments, None);
        view.set_step_records(
            &[
                record("Step-1", StepCategory::Static, Some(1.0), &[1]),
                record("Step-2", StepCategory::Frequency, None, &[2]),
            ],
            &[],
            None,
        );
        let layout = layout(&view, 1600.0);
        let modes = &layout.segments[1];
        assert!(modes.right - modes.left > 1400.0, "{layout:?}");
        // A narrow timeline keeps every step inside it.
        let narrow = super::layout(&view, 200.0);
        assert!(narrow.segments[1].right <= 200.0 + 1e-3);
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
