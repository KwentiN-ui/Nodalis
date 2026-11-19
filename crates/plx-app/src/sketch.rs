//! The sketch tool of the cross-section dialog: an outline and its holes drawn as polygons,
//! their points dragged on a grid or typed in, with the centroid and the principal axes
//! following them.

use egui::{Color32, Pos2, Rect, Sense, Stroke, Ui, pos2, vec2};
use plx_model::{Quantity, SectionProperties, Sketch, UnitSystem};

use crate::{cross_section, numeric, style};

/// Size of the drawing area.
const CANVAS: egui::Vec2 = egui::vec2(320.0, 260.0);

/// Distance in points within which the pointer takes a point or an edge.
const PICK: f32 = 8.0;

/// What the sketch tool keeps between frames: the polygon shown in the table and the point
/// picked, the point dragged and the view while it is, so that the drawing holds still.
#[derive(Clone, Copy, Debug, Default)]
struct State {
    ring: usize,
    point: Option<usize>,
    dragged: Option<(usize, usize)>,
    view: Option<View>,
}

/// Where the sketch lies on the screen: `centre` in the model at `screen`, `scale` points
/// per metre, and the grid it snaps to, in metres.
#[derive(Clone, Copy, Debug)]
struct View {
    centre: [f64; 2],
    screen: Pos2,
    scale: f64,
    step: f64,
}

impl View {
    /// The view showing all of `sketch` in `rect`, with a grid of a round step in `units`.
    fn fitting(sketch: &Sketch, rect: Rect, units: UnitSystem) -> Self {
        let (mut min, mut max) = ([f64::MAX; 2], [f64::MIN; 2]);
        for &[u, v] in sketch.loops.iter().flatten() {
            min = [min[0].min(u), min[1].min(v)];
            max = [max[0].max(u), max[1].max(v)];
        }
        let mut span = (max[0] - min[0]).max(max[1] - min[1]);
        if !(span.is_finite() && span > 0.0) {
            (min, max, span) = ([-0.05; 2], [0.05; 2], 0.1);
        }
        let inner = rect.shrink(24.0);
        let scale = f64::from(inner.width().min(inner.height())) / span;
        Self {
            centre: [(min[0] + max[0]) / 2.0, (min[1] + max[1]) / 2.0],
            screen: inner.center(),
            scale,
            step: grid_step(span / 20.0, units),
        }
    }

    fn to_screen(self, [u, v]: [f64; 2]) -> Pos2 {
        pos2(
            self.screen.x + ((u - self.centre[0]) * self.scale) as f32,
            self.screen.y - ((v - self.centre[1]) * self.scale) as f32,
        )
    }

    fn to_model(self, p: Pos2) -> [f64; 2] {
        [
            self.centre[0] + f64::from(p.x - self.screen.x) / self.scale,
            self.centre[1] - f64::from(p.y - self.screen.y) / self.scale,
        ]
    }

    fn snap(self, [u, v]: [f64; 2]) -> [f64; 2] {
        [u, v].map(|c| (c / self.step).round() * self.step)
    }
}

/// A grid step of 1, 2 or 5 times a power of ten in the shown length unit, near `about`
/// metres.
fn grid_step(about: f64, units: UnitSystem) -> f64 {
    let factor = plx_model::STORED.factor_to(Quantity::Length, units);
    let shown = about * factor;
    let power = 10f64.powf(shown.log10().floor());
    let step = [1.0, 2.0, 5.0, 10.0]
        .into_iter()
        .map(|m| m * power)
        .find(|&s| s >= shown)
        .unwrap_or(10.0 * power);
    step / factor
}

/// The name of a polygon of the sketch.
fn ring_name(ring: usize) -> String {
    if ring == 0 {
        "Outline".into()
    } else {
        format!("Hole {ring}")
    }
}

/// The rows of the sketch tool in the cross-section dialog: the drawing with the area and
/// second moments below it, and the points of one polygon to type in.
pub fn rows(ui: &mut Ui, sketch: &mut Sketch, units: UnitSystem) {
    let id = ui.id().with("sketch tool");
    let mut state: State = ui.data(|d| d.get_temp(id)).unwrap_or_default();
    if sketch.loops.is_empty() {
        sketch
            .loops
            .push(vec![[-0.05, -0.05], [0.05, -0.05], [0.0, 0.05]]);
    }
    state.ring = state.ring.min(sketch.loops.len() - 1);

    ui.label("Sketch");
    ui.vertical(|ui| {
        ui.set_max_width(CANVAS.x);
        canvas(ui, sketch, units, &mut state);
        ui.add(
            egui::Label::new(
                egui::RichText::new(
                    "Drag a point to move it, on the grid unless Shift is held. Double-click \
                     an edge to add a point, right-click a point to delete it.",
                )
                .weak(),
            )
            .wrap(),
        );
        match sketch.problem() {
            Some(problem) => {
                ui.colored_label(style::error(), problem);
            }
            // The centroid where it lies in the sketch.
            None => {
                let profile = plx_model::BeamProfile::Sketch(sketch.clone());
                cross_section::property_lines(
                    ui,
                    SectionProperties::of_loops(&sketch.loops),
                    profile.size(),
                    units,
                );
                cross_section::torsion_line(ui, &profile, units);
            }
        }
    });
    ui.end_row();

    ui.label("Polygon");
    ui.horizontal(|ui| {
        egui::ComboBox::from_id_salt("sketch polygon")
            .selected_text(ring_name(state.ring))
            .width(110.0)
            .show_ui(ui, |ui| {
                for ring in 0..sketch.loops.len() {
                    if ui
                        .selectable_label(state.ring == ring, ring_name(ring))
                        .clicked()
                    {
                        state.ring = ring;
                        state.point = None;
                    }
                }
            });
        if ui
            .button("Add Hole")
            .on_hover_text("Adds a small square hole inside the outline, to drag into place.")
            .clicked()
        {
            sketch.loops.push(new_hole(sketch, units));
            state.ring = sketch.loops.len() - 1;
            state.point = None;
        }
        if ui
            .add_enabled(state.ring > 0, egui::Button::new("Delete Hole"))
            .clicked()
        {
            sketch.loops.remove(state.ring);
            state.ring -= 1;
            state.point = None;
        }
    });
    ui.end_row();

    ui.label("Points");
    points_table(ui, sketch, units, &mut state);
    ui.end_row();

    ui.data_mut(|d| d.insert_temp(id, state));
}

/// The drawing: the grid, the filled region, the points to drag and the axes.
fn canvas(ui: &mut Ui, sketch: &mut Sketch, units: UnitSystem, state: &mut State) {
    let (rect, _) = ui.allocate_exact_size(CANVAS, Sense::hover());
    let response = ui.interact(rect, ui.id().with("sketch canvas"), Sense::click_and_drag());
    let view = match state.view.filter(|_| state.dragged.is_some()) {
        Some(view) => view,
        None => View::fitting(sketch, rect, units),
    };
    let pointer = response.interact_pointer_pos().or(response.hover_pos());
    let nearest_point = |sketch: &Sketch, at: Pos2| {
        let mut best = None;
        for (ring, points) in sketch.loops.iter().enumerate() {
            for (index, &p) in points.iter().enumerate() {
                let distance = view.to_screen(p).distance(at);
                if distance <= PICK && best.is_none_or(|(_, _, d)| distance < d) {
                    best = Some((ring, index, distance));
                }
            }
        }
        best.map(|(ring, index, _)| (ring, index))
    };

    if response.drag_started()
        && let Some(at) = pointer
        && let Some((ring, index)) = nearest_point(sketch, at)
    {
        state.dragged = Some((ring, index));
        state.view = Some(view);
        state.ring = ring;
        state.point = Some(index);
    }
    if let Some((ring, index)) = state.dragged {
        if response.dragged()
            && let Some(at) = pointer
            && let Some(point) = sketch.loops.get_mut(ring).and_then(|r| r.get_mut(index))
        {
            let free = view.to_model(at);
            *point = if ui.input(|i| i.modifiers.shift) {
                free
            } else {
                view.snap(free)
            };
        }
        if !response.dragged() {
            state.dragged = None;
            state.view = None;
        }
    }
    if response.clicked()
        && let Some(at) = pointer
        && let Some((ring, index)) = nearest_point(sketch, at)
    {
        state.ring = ring;
        state.point = Some(index);
    }
    if response.double_clicked()
        && let Some(at) = pointer
        && nearest_point(sketch, at).is_none()
        && let Some((ring, index, point)) = nearest_edge(sketch, view, at)
    {
        let point = if ui.input(|i| i.modifiers.shift) {
            point
        } else {
            view.snap(point)
        };
        sketch.loops[ring].insert(index + 1, point);
        state.ring = ring;
        state.point = Some(index + 1);
    }
    if response.secondary_clicked()
        && let Some(at) = pointer
        && let Some((ring, index)) = nearest_point(sketch, at)
        && sketch.loops[ring].len() > 3
    {
        sketch.loops[ring].remove(index);
        state.ring = ring;
        state.point = None;
    }

    if !ui.is_rect_visible(rect) {
        return;
    }
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 2.0, style::window());
    painter.rect_stroke(
        rect,
        2.0,
        Stroke::new(1.0, style::border()),
        egui::StrokeKind::Inside,
    );
    // The grid as dots, if they are not too dense to see.
    let spacing = (view.step * view.scale) as f32;
    if spacing >= 6.0 {
        let [u0, v0] = view.to_model(rect.left_bottom());
        let [u1, v1] = view.to_model(rect.right_top());
        let dot = style::adapt(Color32::from_gray(150));
        let (i0, i1) = (
            (u0 / view.step).ceil() as i64,
            (u1 / view.step).floor() as i64,
        );
        let (j0, j1) = (
            (v0 / view.step).ceil() as i64,
            (v1 / view.step).floor() as i64,
        );
        for i in i0..=i1 {
            for j in j0..=j1 {
                let at = view.to_screen([i as f64 * view.step, j as f64 * view.step]);
                let size = if i == 0 || j == 0 { 1.5 } else { 0.8 };
                painter.circle_filled(at, size, dot);
            }
        }
    }
    let fill = style::adapt(Color32::from_rgb(170, 190, 215));
    let edge = Stroke::new(1.5, style::adapt(Color32::from_rgb(40, 70, 120)));
    let valid = sketch.problem().is_none();
    if valid {
        cross_section::fill_rings(&painter, &sketch.loops, |p| view.to_screen(p), fill, edge);
        let properties = SectionProperties::of_loops(&sketch.loops);
        let centroid = view.to_screen(properties.centroid);
        let (_, angle) = properties.principal();
        cross_section::paint_axes(&painter, rect, centroid, centroid, angle, 50.0);
    } else {
        let wrong = Stroke::new(1.5, style::error());
        for ring in &sketch.loops {
            let points = ring.iter().map(|&p| view.to_screen(p)).collect();
            painter.add(egui::Shape::closed_line(points, wrong));
        }
    }
    // The points, the picked one and the one under the pointer larger.
    let hovered = pointer.and_then(|at| nearest_point(sketch, at));
    for (ring, points) in sketch.loops.iter().enumerate() {
        for (index, &p) in points.iter().enumerate() {
            let picked = state.ring == ring && state.point == Some(index);
            let big = picked || hovered == Some((ring, index));
            let color = if picked {
                style::adapt(Color32::from_rgb(200, 60, 40))
            } else {
                edge.color
            };
            painter.circle(
                view.to_screen(p),
                if big { 4.5 } else { 3.0 },
                style::window(),
                Stroke::new(1.5, color),
            );
        }
    }
    if hovered.is_some() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
    }
}

/// The edge of the sketch nearest to `at` on the screen, if within reach: its polygon, the
/// index of its first point and the point on it nearest to `at`, in the model.
fn nearest_edge(sketch: &Sketch, view: View, at: Pos2) -> Option<(usize, usize, [f64; 2])> {
    let mut best: Option<(usize, usize, [f64; 2], f32)> = None;
    for (ring, points) in sketch.loops.iter().enumerate() {
        let n = points.len();
        for i in 0..n {
            let (a, b) = (
                view.to_screen(points[i]),
                view.to_screen(points[(i + 1) % n]),
            );
            let ab = b - a;
            let t = ((at - a).dot(ab) / ab.length_sq().max(f32::MIN_POSITIVE)).clamp(0.0, 1.0);
            let on = a + ab * t;
            let distance = on.distance(at);
            if distance <= PICK && best.is_none_or(|(.., d)| distance < d) {
                best = Some((ring, i, view.to_model(on), distance));
            }
        }
    }
    best.map(|(ring, index, point, _)| (ring, index, point))
}

/// A small square hole in the material of `sketch`: in the largest triangle of its region, a
/// quarter as wide as the circle in that triangle, on the grid of `units` if it fits there.
fn new_hole(sketch: &Sketch, units: UnitSystem) -> Vec<[f64; 2]> {
    let points: Vec<[f64; 2]> = sketch.loops.iter().flatten().copied().collect();
    let largest = plx_model::triangulate_with_holes(&sketch.loops)
        .into_iter()
        .map(|t| t.map(|k| points[k]))
        .max_by(|a, b| triangle_area(*a).total_cmp(&triangle_area(*b)));
    let Some(triangle @ [a, b, c]) = largest else {
        return vec![[-0.01, -0.01], [-0.01, 0.01], [0.01, 0.01], [0.01, -0.01]];
    };
    let length = |p: [f64; 2], q: [f64; 2]| (p[0] - q[0]).hypot(p[1] - q[1]);
    let perimeter = length(a, b) + length(b, c) + length(c, a);
    // The centre of the inscribed circle, weighted by the opposite sides.
    let (la, lb, lc) = (length(b, c), length(c, a), length(a, b));
    let centre = [0, 1].map(|k| (la * a[k] + lb * b[k] + lc * c[k]) / perimeter);
    let radius = 2.0 * triangle_area(triangle) / perimeter;
    let square = |[u, v]: [f64; 2], h: f64| {
        vec![
            [u - h, v - h],
            [u - h, v + h],
            [u + h, v + h],
            [u + h, v - h],
        ]
    };
    let step = grid_step(radius / 4.0, units);
    let snapped = square(centre.map(|c| (c / step).round() * step), step);
    let mut with = sketch.clone();
    with.loops.push(snapped.clone());
    if with.problem().is_none() {
        snapped
    } else {
        square(centre, radius / 4.0)
    }
}

fn triangle_area([a, b, c]: [[f64; 2]; 3]) -> f64 {
    ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs() / 2.0
}

/// The points of the polygon picked, as fields of their 1- and 2-coordinate, with buttons to
/// add a point after the picked one and to delete it.
fn points_table(ui: &mut Ui, sketch: &mut Sketch, units: UnitSystem, state: &mut State) {
    ui.vertical(|ui| {
        let ring = &mut sketch.loops[state.ring];
        egui::ScrollArea::vertical()
            .id_salt("sketch points")
            .max_height(150.0)
            .show(ui, |ui| {
                egui::Grid::new("sketch points grid")
                    .num_columns(3)
                    .spacing(vec2(6.0, 2.0))
                    .show(ui, |ui| {
                        ui.label("");
                        ui.weak("1");
                        ui.weak("2");
                        ui.end_row();
                        for (index, point) in ring.iter_mut().enumerate() {
                            let picked = state.point == Some(index);
                            if ui
                                .selectable_label(picked, format!("{}", index + 1))
                                .clicked()
                            {
                                state.point = Some(index);
                            }
                            for c in point.iter_mut() {
                                let field = numeric::quantity(c, units, Quantity::Length);
                                if ui.add_sized([100.0, 18.0], field).gained_focus() {
                                    state.point = Some(index);
                                }
                            }
                            ui.end_row();
                        }
                    });
            });
        ui.horizontal(|ui| {
            let after = state.point.unwrap_or(ring.len() - 1).min(ring.len() - 1);
            if ui
                .button("Add Point")
                .on_hover_text("Adds a point halfway to the next one, after the picked point.")
                .clicked()
            {
                let (p, q) = (ring[after], ring[(after + 1) % ring.len()]);
                ring.insert(after + 1, [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0]);
                state.point = Some(after + 1);
            }
            let can_delete = state.point.is_some() && ring.len() > 3;
            if ui
                .add_enabled(can_delete, egui::Button::new("Delete Point"))
                .clicked()
                && let Some(index) = state.point
            {
                ring.remove(index.min(ring.len() - 1));
                state.point = None;
            }
        });
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_steps_are_round_in_the_shown_unit() {
        assert!((grid_step(0.0042, UnitSystem::MmTonSC) - 0.005).abs() < 1e-12);
        assert!((grid_step(0.0011, UnitSystem::MmTonSC) - 0.002).abs() < 1e-12);
        assert!((grid_step(0.3, UnitSystem::MKgSC) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn a_new_hole_lies_inside_the_outline() {
        let angle = plx_model::Sketch::of(&plx_model::BeamProfile::L {
            h: 0.08,
            b: 0.04,
            t: 0.01,
        });
        let mut sketch = angle.clone();
        sketch.loops.push(new_hole(&sketch, UnitSystem::MmTonSC));
        assert_eq!(sketch.problem(), None);
        let area = SectionProperties::of_loops(&sketch.loops).area;
        assert!(area < SectionProperties::of_loops(&angle.loops).area);
        // On the millimetre grid, and a second one beside the first.
        assert!(
            sketch.loops[1]
                .iter()
                .flatten()
                .all(|c| (c * 1e3 - (c * 1e3).round()).abs() < 1e-9)
        );
        sketch.loops.push(new_hole(&sketch, UnitSystem::MmTonSC));
        assert_eq!(sketch.problem(), None);
    }

    #[test]
    fn views_map_the_model_to_the_screen_and_back() {
        let sketch = plx_model::Sketch::of(&plx_model::BeamProfile::Rect { a: 0.02, b: 0.04 });
        let rect = Rect::from_min_size(pos2(10.0, 20.0), CANVAS);
        let view = View::fitting(&sketch, rect, UnitSystem::MmTonSC);
        let p = [0.003, -0.007];
        let back = view.to_model(view.to_screen(p));
        assert!((back[0] - p[0]).abs() < 1e-6 && (back[1] - p[1]).abs() < 1e-6);
        assert!(rect.contains(view.to_screen([0.01, 0.02])));
        assert_eq!(
            view.snap([0.0041, -0.0059]).map(|c| (c * 1e4).round()),
            [40.0, -60.0]
        );
    }
}
