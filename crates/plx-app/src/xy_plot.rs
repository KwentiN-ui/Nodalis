//! Diagram of curves over a common x axis: a
//! curve per y column, a legend, grid lines at round numbers and the value of the nearest
//! point under the mouse.

use egui::{Color32, Pos2, Rect, Stroke, Ui, pos2, vec2};

use crate::results::format_value;

/// Data of a diagram: x values and the y values of each named curve, row by row.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct XyData {
    pub title: String,
    pub x_label: String,
    pub x: Vec<f64>,
    pub curves: Vec<(String, Vec<f64>)>,
    /// Reference lines over the same x values, drawn dashed in grey and named at their end,
    /// such as the engine order lines of a Campbell diagram.
    pub guides: Vec<(String, Vec<f64>)>,
    /// Upper end of the y axis; None fits the axis to the values.
    pub y_max: Option<f64>,
    /// The x axes the plot can switch between, such as total time, step time and increment,
    /// with `x_axis` the one `x` and `x_label` hold; empty when there is no choice.
    pub x_axes: Vec<(String, Vec<f64>)>,
    pub x_axis: usize,
    /// The steps the rows belong to, drawn as bands behind the curves.
    pub bands: Vec<Band>,
    /// The increment of the results each row shows, if any: the plot marks the one shown in
    /// the 3D view, and a click into it shows another.
    pub increments: Vec<Option<usize>>,
}

/// A step of a history plot: its name and its first and last row.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Band {
    pub name: String,
    pub rows: [usize; 2],
}

impl XyData {
    /// Switches to the x axis `index` of [`XyData::x_axes`].
    pub fn choose_x_axis(&mut self, index: usize) {
        if let Some((label, x)) = self.x_axes.get(index) {
            self.x_label = label.clone();
            self.x = x.clone();
            self.x_axis = index;
        }
    }

    /// The x value of the row that shows the increment `increment`.
    fn x_of_increment(&self, increment: usize) -> Option<f64> {
        let row = self.increments.iter().position(|&i| i == Some(increment))?;
        self.x.get(row).copied().filter(|x| x.is_finite())
    }

    /// The increment of the row nearest to `x` that shows one.
    fn increment_near(&self, x: f64) -> Option<usize> {
        (self.x.iter().zip(&self.increments))
            .filter_map(|(&rx, &i)| Some((rx, i?)))
            .filter(|(rx, _)| rx.is_finite())
            .min_by(|a, b| (a.0 - x).abs().total_cmp(&(b.0 - x).abs()))
            .map(|(_, i)| i)
    }

    /// The x range of each band, when x grows with the rows (as times and increments do,
    /// unlike the step time).
    fn band_ranges(&self) -> Vec<(f64, f64)> {
        let x = &self.x;
        if self.bands.len() < 2 || x.windows(2).any(|w| w[1] < w[0] || w[1].is_nan()) {
            return Vec::new();
        }
        let mut ranges = Vec::new();
        for (index, band) in self.bands.iter().enumerate() {
            let [first, last] = band.rows;
            let (Some(&start), Some(&end)) = (x.get(first), x.get(last)) else {
                return Vec::new();
            };
            // A step starts where the one before ended.
            let start = match index {
                0 => start,
                _ => ranges.last().map_or(start, |&(_, end): &(f64, f64)| end),
            };
            ranges.push((start, end));
        }
        ranges
    }
}

const GUIDE: Color32 = Color32::from_gray(60);

/// Distinct colours for the curves, in the order of the columns.
const COLORS: [Color32; 10] = [
    Color32::from_rgb(31, 119, 180),
    Color32::from_rgb(214, 39, 40),
    Color32::from_rgb(44, 160, 44),
    Color32::from_rgb(255, 127, 14),
    Color32::from_rgb(148, 103, 189),
    Color32::from_rgb(140, 86, 75),
    Color32::from_rgb(227, 119, 194),
    Color32::from_rgb(23, 190, 207),
    Color32::from_rgb(188, 189, 34),
    Color32::from_rgb(127, 127, 127),
];

pub fn color(index: usize) -> Color32 {
    COLORS[index % COLORS.len()]
}

/// Grid values at round steps (1, 2 or 5 times a power of ten) covering `min..=max`.
pub fn ticks(min: f64, max: f64, count: usize) -> Vec<f64> {
    if !(min.is_finite() && max.is_finite()) || max <= min {
        return vec![min];
    }
    let raw = (max - min) / count.max(1) as f64;
    let magnitude = 10f64.powf(raw.log10().floor());
    let step = [1.0, 2.0, 5.0, 10.0]
        .into_iter()
        .map(|f| f * magnitude)
        .find(|s| *s >= raw)
        .unwrap_or(10.0 * magnitude);
    let first = (min / step).ceil() as i64;
    let last = (max / step).floor() as i64;
    (first..=last).map(|k| k as f64 * step).collect()
}

/// Range of the finite values with a margin; a single value gets a range around it.
fn range(values: impl Iterator<Item = f64>) -> Option<(f64, f64)> {
    let (min, max) = values
        .filter(|v| v.is_finite())
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), v| {
            (a.min(v), b.max(v))
        });
    if !(min.is_finite() && max.is_finite()) {
        return None;
    }
    if max - min <= 1e-12 * min.abs().max(max.abs()).max(1e-30) {
        let pad = min.abs().max(1.0) * 0.05;
        return Some((min - pad, max + pad));
    }
    Some((min, max))
}

/// Draws the diagram into the available space of `ui`, with a line at the increment
/// `current`; returns where, and the increment of a click into it.
pub fn show(ui: &mut Ui, data: &XyData, current: Option<usize>) -> (Rect, Option<usize>) {
    let size = ui.available_size().max(vec2(360.0, 220.0));
    let sense = if data.increments.iter().any(Option::is_some) {
        egui::Sense::click_and_drag()
    } else {
        egui::Sense::hover()
    };
    let (rect, response) = ui.allocate_exact_size(size, sense);
    let axes = Axes {
        current,
        ..Axes::default()
    };
    let painter = ui.painter_at(rect);
    paint(&painter, rect, response.hover_pos(), data, &axes);
    let picked = (response.clicked() || response.dragged())
        .then(|| response.interact_pointer_pos())
        .flatten()
        .and_then(|p| pick(rect, p, data, &axes))
        .filter(|&i| Some(i) != current);
    (rect, picked)
}

/// What a viewport annotation changes of a diagram: the label of the value axis and a fixed
/// value range.
#[derive(Default)]
pub struct Axes<'a> {
    /// Written above the value axis; empty for none.
    pub y_label: &'a str,
    /// Values shown, lowest and highest; `None` follows the data.
    pub y_range: Option<[f64; 2]>,
    /// The increment shown in the 3D view, marked by a vertical line.
    pub current: Option<usize>,
}

/// Smallest and largest value of the curves and guides.
pub fn extremes(data: &XyData) -> Option<[f64; 2]> {
    let values = (data.curves.iter().chain(&data.guides)).flat_map(|(_, y)| y.iter().copied());
    range(values).map(|(min, max)| [min, max])
}

/// Where the axes of a diagram in `rect` lie.
fn plot_area(rect: Rect, axes: &Axes) -> Rect {
    let top = if axes.y_label.is_empty() { 12.0 } else { 26.0 };
    Rect::from_min_max(rect.min + vec2(72.0, top), rect.max - vec2(16.0, 40.0))
}

/// The increment of the row nearest to a click at `pos` into the diagram in `rect`.
pub fn pick(rect: Rect, pos: Pos2, data: &XyData, axes: &Axes) -> Option<usize> {
    let area = plot_area(rect, axes);
    let (x0, x1) = range(data.x.iter().copied())?;
    if !area.contains(pos) {
        return None;
    }
    let x = x0 + f64::from((pos.x - area.left()) / area.width()) * (x1 - x0);
    data.increment_near(x)
}

/// Draws the diagram into `rect` with the value under `hover`.
pub fn paint(painter: &egui::Painter, rect: Rect, hover: Option<Pos2>, data: &XyData, axes: &Axes) {
    let painter = painter.with_clip_rect(rect.intersect(painter.clip_rect()));
    painter.rect_filled(rect, 0.0, crate::style::label_fill());
    let font = egui::FontId::proportional(12.0);
    let text = crate::style::plot_text();
    let values = (data.curves.iter().chain(&data.guides)).flat_map(|(_, y)| y.iter().copied());
    let y_range = match (axes.y_range, data.y_max) {
        (Some([low, high]), _) if low.is_finite() && high.is_finite() && high > low => {
            Some((low, high))
        }
        (_, Some(top)) if top.is_finite() && top > 0.0 => {
            range(values).map(|(min, _)| (min.min(0.0), top))
        }
        _ => range(values),
    };
    let (Some((x0, x1)), Some((y0, y1))) = (range(data.x.iter().copied()), y_range) else {
        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "No plottable values",
            font,
            text,
        );
        return;
    };
    let area = plot_area(rect, axes);
    if !axes.y_label.is_empty() {
        painter.text(
            pos2(area.left(), rect.top() + 4.0),
            egui::Align2::LEFT_TOP,
            axes.y_label,
            font.clone(),
            text,
        );
    }
    let to_screen = |x: f64, y: f64| {
        pos2(
            area.left() + ((x - x0) / (x1 - x0)) as f32 * area.width(),
            area.bottom() - ((y - y0) / (y1 - y0)) as f32 * area.height(),
        )
    };
    let grid = Stroke::new(1.0, crate::style::plot_grid());
    let x_count = (area.width() / 90.0).max(2.0) as usize;
    let y_count = (area.height() / 50.0).max(2.0) as usize;
    for y in ticks(y0, y1, y_count) {
        let p = to_screen(x0, y);
        painter.hline(area.x_range(), p.y, grid);
        painter.text(
            pos2(area.left() - 6.0, p.y),
            egui::Align2::RIGHT_CENTER,
            format_value(y as f32),
            font.clone(),
            text,
        );
    }
    for x in ticks(x0, x1, x_count) {
        let p = to_screen(x, y0);
        painter.vline(p.x, area.y_range(), grid);
        painter.text(
            pos2(p.x, area.bottom() + 4.0),
            egui::Align2::CENTER_TOP,
            format_value(x as f32),
            font.clone(),
            text,
        );
    }
    painter.rect_stroke(
        area,
        0.0,
        Stroke::new(1.0, crate::style::adapt(Color32::from_gray(80))),
        egui::StrokeKind::Inside,
    );
    painter.text(
        pos2(area.center().x, rect.bottom() - 4.0),
        egui::Align2::CENTER_BOTTOM,
        &data.x_label,
        font.clone(),
        text,
    );
    // Curves leave the plot area when the y axis is limited.
    let plot = painter.with_clip_rect(area);
    let x_screen = |x: f64| to_screen(x, y0).x;
    for (index, (&(start, end), band)) in data.band_ranges().iter().zip(&data.bands).enumerate() {
        let (left, right) = (x_screen(start), x_screen(end));
        if index % 2 == 1 {
            let shade = Rect::from_x_y_ranges(left..=right, area.y_range());
            plot.rect_filled(shade, 0.0, crate::style::text().gamma_multiply(0.06));
        }
        if index > 0 {
            plot.extend(egui::Shape::dashed_line(
                &[pos2(left, area.top()), pos2(left, area.bottom())],
                Stroke::new(1.0, crate::style::adapt(Color32::from_gray(120))),
                4.0,
                3.0,
            ));
        }
        let name = plot
            .with_clip_rect(Rect::from_x_y_ranges(left..=right, area.y_range()).intersect(area));
        name.text(
            pos2(left + 4.0, area.top() + 3.0),
            egui::Align2::LEFT_TOP,
            &band.name,
            font.clone(),
            crate::style::adapt(Color32::from_gray(110)),
        );
    }
    if let Some(x) = axes.current.and_then(|i| data.x_of_increment(i)) {
        plot.vline(
            x_screen(x),
            area.y_range(),
            Stroke::new(1.5, crate::style::highlight()),
        );
    }
    let markers = data.x.len() <= 60;
    for (name, values) in &data.guides {
        let points: Vec<Pos2> = (data.x.iter().zip(values))
            .filter(|(x, y)| x.is_finite() && y.is_finite())
            .map(|(&x, &y)| to_screen(x, y))
            .collect();
        plot.extend(egui::Shape::dashed_line(
            &points,
            Stroke::new(1.5, crate::style::adapt(GUIDE)),
            8.0,
            5.0,
        ));
        // The name sits on the line short of its visible end, clear of the legend in the
        // upper right corner.
        let first = points.iter().find(|p| area.contains(**p));
        let last = points.iter().rev().find(|p| area.contains(**p));
        if let (Some(&first), Some(&last)) = (first, last) {
            let at = first.lerp(last, 0.8);
            let galley =
                plot.layout_no_wrap(name.clone(), font.clone(), crate::style::adapt(GUIDE));
            let frame = Rect::from_min_size(at + vec2(2.0, -4.0 - galley.size().y), galley.size());
            plot.rect_filled(frame.expand(1.0), 2.0, crate::style::plot_label_fill(220));
            plot.galley(frame.min, galley, crate::style::adapt(GUIDE));
        }
    }
    for (index, (_, values)) in data.curves.iter().enumerate() {
        let color = color(index);
        // Rows without a value break the curve.
        let mut segment: Vec<Pos2> = Vec::new();
        let flush = |segment: &mut Vec<Pos2>| {
            if segment.len() > 1 {
                plot.add(egui::Shape::line(
                    std::mem::take(segment),
                    Stroke::new(2.0, color),
                ));
            }
            segment.clear();
        };
        for (&x, &y) in data.x.iter().zip(values) {
            if x.is_finite() && y.is_finite() {
                let p = to_screen(x, y);
                segment.push(p);
                if markers {
                    plot.circle_filled(p, 2.5, color);
                }
            } else {
                flush(&mut segment);
            }
        }
        flush(&mut segment);
    }
    legend(&painter, area, data, &font);
    if let Some(hover) = hover.filter(|p| area.contains(*p)) {
        let lines: Vec<_> = data.curves.iter().chain(&data.guides).collect();
        let nearest = (lines.iter().enumerate())
            .flat_map(|(c, (_, values))| {
                (data.x.iter().zip(values))
                    .filter(|(x, y)| x.is_finite() && y.is_finite())
                    .map(move |(&x, &y)| (c, x, y))
            })
            .filter(|&(_, x, y)| area.expand(1.0).contains(to_screen(x, y)))
            .min_by(|a, b| {
                let d = |x: f64, y: f64| to_screen(x, y).distance_sq(hover);
                d(a.1, a.2).total_cmp(&d(b.1, b.2))
            });
        if let Some((curve, x, y)) = nearest {
            let p = to_screen(x, y);
            painter.circle_stroke(p, 4.5, Stroke::new(1.5, crate::style::text()));
            let label = format!(
                "{}\n{}: {}\n{}",
                lines[curve].0,
                data.x_label,
                format_value(x as f32),
                format_value(y as f32)
            );
            let galley = painter.layout_no_wrap(label, font, crate::style::text());
            let mut at = p + vec2(8.0, -8.0 - galley.size().y);
            at.x = at.x.min(area.right() - galley.size().x - 4.0);
            at.y = at.y.max(area.top() + 2.0);
            let frame = Rect::from_min_size(at, galley.size()).expand(3.0);
            painter.rect_filled(frame, 2.0, crate::style::plot_label_fill(230));
            painter.rect_stroke(
                frame,
                2.0,
                Stroke::new(1.0, crate::style::adapt(Color32::from_gray(150))),
                egui::StrokeKind::Inside,
            );
            painter.galley(at, galley, crate::style::text());
        }
    }
}

/// Names of the guides and the curves in the upper right corner of the plot area.
fn legend(painter: &egui::Painter, area: Rect, data: &XyData, font: &egui::FontId) {
    let entries: Vec<(&String, Option<Color32>)> = (data.guides.iter().map(|(n, _)| (n, None)))
        .chain((data.curves.iter().enumerate()).map(|(i, (n, _))| (n, Some(color(i)))))
        .collect();
    let shown = entries.len().min(12);
    if shown == 0 {
        return;
    }
    let galleys: Vec<_> = (entries.iter().take(shown))
        .map(|(name, _)| {
            painter.layout_no_wrap((*name).clone(), font.clone(), crate::style::text())
        })
        .collect();
    let width = galleys.iter().map(|g| g.size().x).fold(0.0, f32::max) + 30.0;
    let line = 16.0;
    let extra = (entries.len() > shown) as usize;
    let height = line * (shown + extra) as f32 + 6.0;
    let frame = Rect::from_min_size(
        pos2(area.right() - width - 8.0, area.top() + 8.0),
        vec2(width, height),
    );
    painter.rect_filled(frame, 2.0, crate::style::plot_label_fill(230));
    painter.rect_stroke(
        frame,
        2.0,
        Stroke::new(1.0, crate::style::adapt(Color32::from_gray(160))),
        egui::StrokeKind::Inside,
    );
    for (index, (galley, (_, color))) in galleys.into_iter().zip(&entries).enumerate() {
        let y = frame.top() + 3.0 + line * index as f32 + line / 2.0;
        let sample = [pos2(frame.left() + 5.0, y), pos2(frame.left() + 21.0, y)];
        match color {
            Some(color) => {
                painter.line_segment(sample, Stroke::new(2.0, *color));
            }
            None => {
                painter.extend(egui::Shape::dashed_line(
                    &sample,
                    Stroke::new(1.5, crate::style::adapt(GUIDE)),
                    5.0,
                    3.0,
                ));
            }
        }
        painter.galley(
            pos2(frame.left() + 26.0, y - galley.size().y / 2.0),
            galley,
            crate::style::text(),
        );
    }
    if extra > 0 {
        let y = frame.top() + 3.0 + line * shown as f32 + line / 2.0;
        painter.text(
            pos2(frame.left() + 26.0, y),
            egui::Align2::LEFT_CENTER,
            format!("… {} more", entries.len() - shown),
            font.clone(),
            crate::style::adapt(Color32::from_gray(80)),
        );
    }
}

/// Where the diagram of the window is to be exported.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Export {
    Clipboard,
    Png,
    Svg,
}

/// What the user did in the diagram's window.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindowResponse {
    pub closed: bool,
    /// Show in View or Remove from View was clicked.
    pub toggle_view: bool,
    pub export: Option<Export>,
    /// Where the diagram is drawn, in points.
    pub plot: Rect,
    /// The increment a click into the diagram asks the 3D view to show.
    pub picked: Option<usize>,
}

/// The diagram in its own window; `shown` is whether it is also an
/// annotation in the 3D view.
pub fn window(
    ctx: &egui::Context,
    data: &mut XyData,
    shown: bool,
    current: Option<usize>,
) -> WindowResponse {
    let mut picked = None;
    let mut open = true;
    let mut toggle_view = false;
    let mut export = None;
    let mut plot = Rect::NOTHING;
    let mut close = false;
    egui::Window::new(format!("Plot: {}", data.title))
        .id(crate::ids::window(ctx, "history plot"))
        .open(&mut open)
        .collapsible(false)
        .resizable(true)
        .default_size([640.0, 450.0])
        .pivot(egui::Align2::RIGHT_TOP)
        .default_pos(ctx.content_rect().right_top() + vec2(-24.0, 96.0))
        .min_size([360.0, 270.0])
        .show(ctx, |ui| {
            ui.with_layout(egui::Layout::bottom_up(egui::Align::Min), |ui| {
                ui.horizontal(|ui| {
                    let label = if shown {
                        "Remove from View"
                    } else {
                        "Show in View"
                    };
                    toggle_view = ui
                        .button(label)
                        .on_hover_text(
                            "Shows the plot as an annotation in the 3D view, where it stays \
                             after the window is closed. Drag it to move it, drag its corner \
                             to resize it; right-click it to edit or delete it.",
                        )
                        .clicked();
                    ui.menu_button("Export", |ui| {
                        for (label, choice) in [
                            ("Copy to Clipboard", Export::Clipboard),
                            ("Save as PNG …", Export::Png),
                            ("Save as SVG …", Export::Svg),
                        ] {
                            if ui.button(label).clicked() {
                                export = Some(choice);
                                ui.close();
                            }
                        }
                    });
                    if !data.x_axes.is_empty() {
                        ui.label("X axis");
                        let mut chosen = data.x_axis;
                        egui::ComboBox::from_id_salt("plot x axis")
                            .selected_text(data.x_label.clone())
                            .show_ui(ui, |ui| {
                                for (index, (label, _)) in data.x_axes.iter().enumerate() {
                                    ui.selectable_value(&mut chosen, index, label);
                                }
                            });
                        if chosen != data.x_axis {
                            data.choose_x_axis(chosen);
                        }
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        close = ui.button("Close").clicked();
                    });
                });
                ui.with_layout(egui::Layout::top_down(egui::Align::Min), |ui| {
                    (plot, picked) = show(ui, data, current);
                });
            });
        });
    WindowResponse {
        picked,
        closed: !open || close,
        toggle_view,
        export,
        plot,
    }
}

/// The diagram as an SVG image of `width` by `height` points, laid out like the window's.
pub fn to_svg(data: &XyData, width: f32, height: f32) -> String {
    use std::fmt::Write;
    let (width, height) = (width.max(360.0), height.max(220.0));
    let mut svg = format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{width}\" height=\"{height}\" \
         viewBox=\"0 0 {width} {height}\" font-family=\"Noto Sans, Arial, sans-serif\" \
         font-size=\"12\">\n<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n"
    );
    let values = (data.curves.iter().chain(&data.guides)).flat_map(|(_, y)| y.iter().copied());
    let y_range = match data.y_max {
        Some(top) if top.is_finite() && top > 0.0 => {
            range(values).map(|(min, _)| (min.min(0.0), top))
        }
        _ => range(values),
    };
    let (Some((x0, x1)), Some((y0, y1))) = (range(data.x.iter().copied()), y_range) else {
        let _ = writeln!(
            svg,
            "<text x=\"{}\" y=\"{}\" text-anchor=\"middle\">No plottable values</text>",
            width / 2.0,
            height / 2.0
        );
        return svg + "</svg>\n";
    };
    let area = Rect::from_min_max(pos2(72.0, 12.0), pos2(width - 16.0, height - 40.0));
    let to_screen = |x: f64, y: f64| {
        pos2(
            area.left() + ((x - x0) / (x1 - x0)) as f32 * area.width(),
            area.bottom() - ((y - y0) / (y1 - y0)) as f32 * area.height(),
        )
    };
    let ink = "#282828";
    for y in ticks(y0, y1, (area.height() / 50.0).max(2.0) as usize) {
        let p = to_screen(x0, y);
        let _ = writeln!(
            svg,
            "<line x1=\"{:.2}\" y1=\"{:.2}\" x2=\"{:.2}\" y2=\"{:.2}\" stroke=\"#e1e1e1\"/>\n\
             <text x=\"{:.2}\" y=\"{:.2}\" text-anchor=\"end\" dy=\"0.35em\" \
             fill=\"{ink}\">{}</text>",
            area.left(),
            p.y,
            area.right(),
            p.y,
            area.left() - 6.0,
            p.y,
            escape(&format_value(y as f32))
        );
    }
    for x in ticks(x0, x1, (area.width() / 90.0).max(2.0) as usize) {
        let p = to_screen(x, y0);
        let _ = writeln!(
            svg,
            "<line x1=\"{:.2}\" y1=\"{:.2}\" x2=\"{:.2}\" y2=\"{:.2}\" stroke=\"#e1e1e1\"/>\n\
             <text x=\"{:.2}\" y=\"{:.2}\" text-anchor=\"middle\" fill=\"{ink}\">{}</text>",
            p.x,
            area.top(),
            p.x,
            area.bottom(),
            p.x,
            area.bottom() + 16.0,
            escape(&format_value(x as f32))
        );
    }
    let _ = writeln!(
        svg,
        "<rect x=\"{:.2}\" y=\"{:.2}\" width=\"{:.2}\" height=\"{:.2}\" fill=\"none\" \
         stroke=\"#505050\"/>\n<text x=\"{:.2}\" y=\"{:.2}\" text-anchor=\"middle\" \
         fill=\"{ink}\">{}</text>\n<clipPath id=\"area\"><rect x=\"{:.2}\" y=\"{:.2}\" \
         width=\"{:.2}\" height=\"{:.2}\"/></clipPath>\n<g clip-path=\"url(#area)\" \
         fill=\"none\" stroke-linejoin=\"round\">",
        area.left(),
        area.top(),
        area.width(),
        area.height(),
        area.center().x,
        height - 6.0,
        escape(&data.x_label),
        area.left(),
        area.top(),
        area.width(),
        area.height(),
    );
    // Rows without a value break a line.
    let pieces = |values: &[f64]| {
        let mut pieces: Vec<Vec<Pos2>> = vec![Vec::new()];
        for (&x, &y) in data.x.iter().zip(values) {
            if x.is_finite() && y.is_finite() {
                pieces.last_mut().unwrap().push(to_screen(x, y));
            } else if !pieces.last().unwrap().is_empty() {
                pieces.push(Vec::new());
            }
        }
        pieces
    };
    let points = |piece: &[Pos2]| {
        (piece.iter())
            .map(|p| format!("{:.2},{:.2}", p.x, p.y))
            .collect::<Vec<_>>()
            .join(" ")
    };
    for (_, values) in &data.guides {
        for piece in pieces(values).iter().filter(|p| p.len() > 1) {
            let _ = writeln!(
                svg,
                "<polyline points=\"{}\" stroke=\"{}\" stroke-width=\"1.5\" \
                 stroke-dasharray=\"8 5\"/>",
                points(piece),
                hex(GUIDE)
            );
        }
    }
    let markers = data.x.len() <= 60;
    for (index, (_, values)) in data.curves.iter().enumerate() {
        let color = hex(color(index));
        for piece in pieces(values) {
            if piece.len() > 1 {
                let _ = writeln!(
                    svg,
                    "<polyline points=\"{}\" stroke=\"{color}\" stroke-width=\"2\"/>",
                    points(&piece)
                );
            }
            if markers {
                for p in &piece {
                    let _ = writeln!(
                        svg,
                        "<circle cx=\"{:.2}\" cy=\"{:.2}\" r=\"2.5\" fill=\"{color}\"/>",
                        p.x, p.y
                    );
                }
            }
        }
    }
    svg.push_str("</g>\n");
    // The legend in the upper right corner; text is not measured, about 7 points per letter.
    let entries: Vec<(&String, Option<Color32>)> = (data.guides.iter().map(|(n, _)| (n, None)))
        .chain((data.curves.iter().enumerate()).map(|(i, (n, _))| (n, Some(color(i)))))
        .collect();
    if !entries.is_empty() {
        let longest = (entries.iter()).map(|(n, _)| n.chars().count()).max();
        let w = longest.unwrap_or(0) as f32 * 7.0 + 30.0;
        let line = 16.0;
        let h = line * entries.len() as f32 + 6.0;
        let (left, top) = (area.right() - w - 8.0, area.top() + 8.0);
        let _ = writeln!(
            svg,
            "<rect x=\"{left:.2}\" y=\"{top:.2}\" width=\"{w:.2}\" height=\"{h:.2}\" rx=\"2\" \
             fill=\"white\" fill-opacity=\"0.9\" stroke=\"#a0a0a0\"/>"
        );
        for (k, (name, color)) in entries.iter().enumerate() {
            let y = top + 3.0 + line * k as f32 + line / 2.0;
            let style = match color {
                Some(c) => format!("stroke=\"{}\" stroke-width=\"2\"", hex(*c)),
                None => format!(
                    "stroke=\"{}\" stroke-width=\"1.5\" stroke-dasharray=\"5 3\"",
                    hex(GUIDE)
                ),
            };
            let _ = writeln!(
                svg,
                "<line x1=\"{:.2}\" y1=\"{y:.2}\" x2=\"{:.2}\" y2=\"{y:.2}\" {style}/>\n\
                 <text x=\"{:.2}\" y=\"{y:.2}\" dy=\"0.35em\">{}</text>",
                left + 5.0,
                left + 21.0,
                left + 26.0,
                escape(name)
            );
        }
    }
    svg + "</svg>\n"
}

fn hex(color: Color32) -> String {
    format!("#{:02x}{:02x}{:02x}", color.r(), color.g(), color.b())
}

fn escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ticks_fall_on_round_numbers() {
        assert_eq!(
            ticks(0.0, 1.0, 5),
            [0.0, 0.2, 0.4, 0.6000000000000001, 0.8, 1.0]
        );
        assert_eq!(ticks(-3.0, 17.0, 4), [0.0, 5.0, 10.0, 15.0]);
        assert_eq!(ticks(2.0, 2.0, 4), [2.0]);
    }

    #[test]
    fn svg_breaks_curves_at_missing_values() {
        let data = XyData {
            title: "T".into(),
            x_label: "Time <s>".into(),
            x: vec![0.0, 1.0, 2.0, 3.0, 4.0],
            curves: vec![("11".into(), vec![1.0, 2.0, f64::NAN, 3.0, 4.0])],
            guides: Vec::new(),
            y_max: None,
            ..Default::default()
        };
        let svg = to_svg(&data, 640.0, 400.0);
        assert!(svg.starts_with("<svg") && svg.trim_end().ends_with("</svg>"));
        assert_eq!(svg.matches("<polyline").count(), 2);
        assert_eq!(svg.matches("<circle").count(), 4);
        assert!(svg.contains("Time &lt;s&gt;"));
        assert!(!svg.contains("NaN"));
    }

    #[test]
    fn a_constant_curve_gets_a_range() {
        let (min, max) = range([5.0, 5.0, f64::NAN].into_iter()).unwrap();
        assert!(min < 5.0 && max > 5.0);
        assert!(range([f64::NAN].into_iter()).is_none());
    }
}
