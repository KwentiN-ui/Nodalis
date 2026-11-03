//! Plots of path results and of hot spots shown as annotations in the 3D view, which stay
//! after their dialog is closed and follow the shown increment: either in a box the user
//! moves and resizes, or drawn onto the path itself, which then is the plot's distance axis.
//! A hot spot's plot is the stress along the path of its toe with the read-out points and
//! the extrapolation. A table's plot shows curves from a history output table; it has no
//! path and is always drawn in a box.

use egui::{
    Color32, CursorIcon, Painter, PointerButton, Pos2, Rect, Sense, Stroke, StrokeKind, Ui, Vec2,
    vec2,
};
use glam::Vec3;
use plx_model::ResultPath;
use plx_render::Camera;
use plx_results::hot_spot::HotSpot;
use plx_results::path::{self, PathPoint};

use crate::features::{MARK, Marks};
use crate::model::Model;
use crate::overlay::project;
use crate::results::format_value;
use crate::xy_plot::XyData;

const MARGIN: f32 = 20.0;
/// Space kept free below a new box for the scale bar.
const BOTTOM: f32 = 70.0;
const TITLE_HEIGHT: f32 = 20.0;
const HANDLE: f32 = 12.0;
const MIN_SIZE: Vec2 = vec2(260.0, 160.0);
/// Distance in points from the curve on the path within which it can be grabbed.
const GRAB: f32 = 6.0;
const CURVE: Color32 = Color32::from_rgb(200, 0, 160);
const FILL: Color32 = Color32::from_rgba_premultiplied(25, 0, 20, 32);

/// Where and how a plot is drawn, as set by dragging it and in its edit dialog.
#[derive(Clone, Debug, PartialEq)]
pub struct Placement {
    /// Top left corner of the box relative to that of the view; `None` until it is moved.
    pub offset: Option<Vec2>,
    pub size: Vec2,
    /// Drawn onto the path instead of in a box.
    pub on_path: bool,
    /// On the path: distance in points of the largest value from the path; negative flips the
    /// plot to the other side of the path.
    pub height: f32,
    /// Title; empty for the path's name and the shown component.
    pub title: String,
    pub x_label: String,
    /// Written above the value axis; empty for none.
    pub y_label: String,
    /// Values shown, lowest and highest; `None` fits them to the values.
    pub y_range: Option<[f64; 2]>,
}

impl Default for Placement {
    fn default() -> Self {
        Self {
            offset: None,
            size: vec2(400.0, 240.0),
            on_path: false,
            height: 80.0,
            title: String::new(),
            x_label: "Distance".into(),
            y_label: String::new(),
            y_range: None,
        }
    }
}

/// What a plot shows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlotKind {
    /// The shown result along a result path.
    Path,
    /// The stress along the path of a hot spot, with its extrapolation.
    HotSpot,
    /// Curves from a history output table, which do not change with the increment.
    Table,
}

/// A path or hot spot whose plot the 3D view shows, with where the user put it.
pub struct PathPlot {
    /// Name of the path in the model's features, or of the hot spot.
    pub path: String,
    pub kind: PlotKind,
    pub placement: Placement,
    /// Read-out points of the path as last located.
    located: Option<(ResultPath, Result<Vec<PathPoint>, String>)>,
    /// Paths of the hot spot as last located.
    hot_spot: crate::hot_spots::Located,
    /// The curves of a table's plot.
    pub table: Option<XyData>,
}

impl PathPlot {
    /// The plot as saved with results bound to a project.
    pub fn saved(&self) -> plx_results::saved_view::SavedPathPlot {
        let p = &self.placement;
        plx_results::saved_view::SavedPathPlot {
            path: self.path.clone(),
            offset: p.offset.map(|o| [o.x, o.y]),
            size: [p.size.x, p.size.y],
            on_path: p.on_path,
            height: p.height,
            title: p.title.clone(),
            x_label: p.x_label.clone(),
            y_label: p.y_label.clone(),
            y_range: p.y_range,
            hot_spot: self.kind == PlotKind::HotSpot,
            table: self
                .table
                .as_ref()
                .map(|data| plx_results::saved_view::SavedTablePlot {
                    x_label: data.x_label.clone(),
                    x: data.x.clone(),
                    curves: data.curves.clone(),
                }),
        }
    }

    pub fn from_saved(saved: &plx_results::saved_view::SavedPathPlot) -> Self {
        let mut plot = Self::new(saved.path.clone());
        if saved.hot_spot {
            plot.kind = PlotKind::HotSpot;
        }
        if let Some(table) = &saved.table {
            plot.kind = PlotKind::Table;
            plot.table = Some(XyData {
                title: saved.path.clone(),
                x_label: table.x_label.clone(),
                x: table.x.clone(),
                curves: table.curves.clone(),
                guides: Vec::new(),
                y_max: None,
            });
        }
        plot.placement = Placement {
            offset: saved.offset.map(|[x, y]| vec2(x, y)),
            size: vec2(saved.size[0], saved.size[1]),
            on_path: saved.on_path,
            height: saved.height,
            title: saved.title.clone(),
            x_label: saved.x_label.clone(),
            y_label: saved.y_label.clone(),
            y_range: saved.y_range,
        };
        plot
    }

    pub fn new(path: String) -> Self {
        Self {
            path,
            kind: PlotKind::Path,
            placement: Placement::default(),
            located: None,
            hot_spot: Default::default(),
            table: None,
        }
    }

    /// The plot of the hot spot named `name`.
    pub fn hot_spot(name: String) -> Self {
        Self {
            kind: PlotKind::HotSpot,
            ..Self::new(name)
        }
    }

    /// The plot of curves from a history output table.
    pub fn table(data: XyData) -> Self {
        let mut plot = Self::new(data.title.clone());
        plot.kind = PlotKind::Table;
        plot.placement.x_label = data.x_label.clone();
        plot.table = Some(data);
        plot
    }

    /// Whether this is the plot of these curves of a table.
    pub fn is_table(&self, data: &XyData) -> bool {
        self.kind == PlotKind::Table && self.table.as_ref() == Some(data)
    }

    /// The table's plot as the 3D view draws it.
    pub fn table_box(&self, index: usize) -> Option<PlotBox> {
        let data = self.table.clone()?;
        Some(PlotBox {
            index,
            title: data.title.clone(),
            distances: Vec::new(),
            values: Vec::new(),
            ends: [Vec3::ZERO; 2],
            placement: Placement {
                on_path: false,
                ..self.placement.clone()
            },
            marks: None,
            auto_range: None,
            table: Some(data),
        })
    }

    /// Whether this is the plot of the path named `name`.
    pub fn is_path(&self, name: &str) -> bool {
        self.kind == PlotKind::Path && self.path == name
    }

    /// Whether this is the plot of the hot spot named `name`.
    pub fn is_hot_spot(&self, name: &str) -> bool {
        self.kind == PlotKind::HotSpot && self.path == name
    }

    /// The plot of `hot_spot` in the shown increment, or `None` where it has no values.
    pub fn hot_spot_box(
        &mut self,
        index: usize,
        model: &Model,
        hot_spot: &HotSpot,
    ) -> Option<PlotBox> {
        let plot = self.hot_spot.plot(model, hot_spot).ok()?;
        let range = plot.range();
        Some(PlotBox {
            index,
            title: format!(
                "{}: {} at node {}",
                hot_spot.name,
                plot.component.short(),
                plot.node
            ),
            ends: plot.ends.map(|p| model.to_render(p)),
            distances: plot.distances,
            values: plot.values,
            placement: self.placement.clone(),
            marks: Some(plot.marks),
            auto_range: Some(range),
            table: None,
        })
    }

    /// The plot to draw for the shown component of the model's results along `path`, or
    /// `None` without results or where the path misses the mesh.
    pub fn plot_box(&mut self, index: usize, model: &Model, path: &ResultPath) -> Option<PlotBox> {
        let view = model.results.as_ref()?;
        let (field, component) = view.current()?;
        if self.located.as_ref().is_none_or(|(p, _)| p != path) {
            let points = path
                .samples(&model.fe)
                .map(|samples| path::path_points(&model.mesh, &samples));
            self.located = Some((path.clone(), points));
        }
        let points = self.located.as_ref()?.1.as_ref().ok()?;
        let values = path::interpolate(points, &component.values);
        if !values.iter().any(|v| v.is_finite()) {
            return None;
        }
        let (ends, _) = crate::features::path_line(model, path)?;
        Some(PlotBox {
            index,
            title: format!("{}: {} {}", path.name, field.name, component.name),
            distances: points.iter().map(|p| p.distance).collect(),
            values,
            ends,
            placement: self.placement.clone(),
            marks: None,
            auto_range: None,
            table: None,
        })
    }
}

/// A plot as the 3D view draws it.
pub struct PlotBox {
    /// Index of the plot in the model's list.
    pub index: usize,
    pub title: String,
    pub distances: Vec<f64>,
    pub values: Vec<f64>,
    /// Start and end of the path in render coordinates.
    pub ends: [Vec3; 2],
    pub placement: Placement,
    /// Read-out points and extrapolation of a hot spot.
    pub marks: Option<Marks>,
    /// Value range while the placement's is automatic; `None` fits it to the values.
    pub auto_range: Option<[f64; 2]>,
    /// The curves of a table's plot, drawn instead of the values along a path.
    pub table: Option<XyData>,
}

/// What the user did to a plot.
pub enum PlotEvent {
    /// Moved, resized or switched between box and path.
    Changed {
        index: usize,
        placement: Placement,
    },
    /// The edit dialog is to be opened.
    Edit(usize),
    Delete(usize),
}

impl PlotBox {
    /// The title the user entered, or the path's name and the shown component.
    fn shown_title(&self) -> &str {
        match self.placement.title.trim() {
            "" => &self.title,
            title => title,
        }
    }

    /// The value range shown: the one the user fixed, else the automatic one.
    fn y_range(&self) -> Option<[f64; 2]> {
        self.placement.y_range.or(self.auto_range)
    }

    /// Smallest and largest value shown when the range is automatic.
    pub fn extremes(&self) -> Option<[f64; 2]> {
        if let Some(range) = self.auto_range {
            return Some(range);
        }
        if let Some(data) = &self.table {
            return crate::xy_plot::extremes(data);
        }
        let finite = self.values.iter().copied().filter(|v| v.is_finite());
        let low = finite.clone().min_by(f64::total_cmp)?;
        let high = finite.max_by(f64::total_cmp)?;
        Some([low, high])
    }
}

/// Draws the plots over the view `rect`. Dragging a box moves it, dragging its bottom right
/// corner resizes it; dragging a plot on its path changes its height. The context menu
/// switches between the two and deletes the plot.
pub fn draw(ui: &Ui, rect: Rect, camera: &Camera, boxes: &mut [PlotBox]) -> Vec<PlotEvent> {
    let painter = ui.painter_at(rect);
    let mut events = Vec::new();
    let mut stacked = 0;
    for plot in boxes.iter_mut() {
        let before = plot.placement.clone();
        let id = ui.id().with(("path plot", plot.index));
        let response = if plot.placement.on_path {
            draw_on_path(ui, &painter, rect, camera, plot, id)
        } else {
            stacked += 1;
            draw_box(ui, &painter, rect, plot, id, stacked - 1)
        };
        let Some(response) = response else {
            continue;
        };
        if response.double_clicked() {
            events.push(PlotEvent::Edit(plot.index));
        }
        response.context_menu(|ui| {
            if ui.button("Edit …").clicked() {
                events.push(PlotEvent::Edit(plot.index));
                ui.close();
            }
            let (label, on_path) = if plot.placement.on_path {
                ("Show in Box", false)
            } else {
                ("Show on Path", true)
            };
            if plot.table.is_none() && ui.button(label).clicked() {
                plot.placement.on_path = on_path;
                ui.close();
            }
            if ui.button("Delete").clicked() {
                events.push(PlotEvent::Delete(plot.index));
                ui.close();
            }
        });
        if plot.placement != before {
            events.push(PlotEvent::Changed {
                index: plot.index,
                placement: plot.placement.clone(),
            });
        }
    }
    events
}

/// A plot in a box with its title; returns the box's response.
fn draw_box(
    ui: &Ui,
    painter: &Painter,
    rect: Rect,
    plot: &mut PlotBox,
    id: egui::Id,
    k: usize,
) -> Option<egui::Response> {
    let size = plot
        .placement
        .size
        .max(MIN_SIZE)
        .min((rect.size() - vec2(2.0 * MARGIN, 2.0 * MARGIN)).max(MIN_SIZE));
    // New boxes go to the bottom left corner, each a little above the one before.
    let stagger = 30.0 * k as f32;
    let default = vec2(MARGIN + stagger, rect.height() - size.y - BOTTOM - stagger);
    let mut offset = plot.placement.offset.unwrap_or(default);
    let frame = Rect::from_min_size(rect.min + offset, size);
    let response = ui.interact(frame, id, Sense::click_and_drag());
    let corner = Rect::from_min_size(frame.max - Vec2::splat(HANDLE), Vec2::splat(HANDLE));
    let handle = ui.interact(corner, id.with("resize"), Sense::drag());
    let mut size = size;
    let mut changed = false;
    if handle.dragged_by(PointerButton::Primary) {
        size = (size + handle.drag_delta()).max(MIN_SIZE);
        changed = true;
    } else if response.dragged_by(PointerButton::Primary) {
        offset += response.drag_delta();
        changed = true;
    }
    if handle.hovered() || handle.dragged() {
        ui.ctx().set_cursor_icon(CursorIcon::ResizeNwSe);
    } else if response.dragged() {
        ui.ctx().set_cursor_icon(CursorIcon::Grabbing);
    } else if response.hovered() {
        ui.ctx().set_cursor_icon(CursorIcon::Grab);
    }
    // Kept inside the view.
    offset.x = offset.x.clamp(0.0, (rect.width() - size.x).max(0.0));
    offset.y = offset.y.clamp(0.0, (rect.height() - size.y).max(0.0));
    if changed {
        plot.placement.offset = Some(offset);
        plot.placement.size = size;
    }
    let frame = Rect::from_min_size(rect.min + offset, size);
    painter.rect_filled(frame, 0.0, crate::style::label_fill());
    painter.text(
        frame.min + vec2(6.0, 3.0),
        egui::Align2::LEFT_TOP,
        plot.shown_title(),
        egui::FontId::proportional(13.0),
        crate::style::text(),
    );
    let area = Rect::from_min_max(frame.min + vec2(0.0, TITLE_HEIGHT), frame.max);
    let hover = response.hover_pos().filter(|_| !response.dragged());
    if let Some(data) = &plot.table {
        let data = XyData {
            x_label: plot.placement.x_label.clone(),
            ..data.clone()
        };
        crate::xy_plot::paint(
            painter,
            area.shrink(1.0),
            hover,
            &data,
            &crate::xy_plot::Axes {
                y_label: &plot.placement.y_label,
                y_range: plot.placement.y_range,
            },
        );
    } else {
        crate::features::draw_plot(
            painter,
            area.shrink(1.0),
            hover,
            &plot.distances,
            &plot.values,
            &crate::features::Axes {
                x_label: &plot.placement.x_label,
                y_label: &plot.placement.y_label,
                y_range: plot.y_range(),
                marks: plot.marks.as_ref(),
            },
            false,
        );
    }
    painter.rect_stroke(
        frame,
        0.0,
        Stroke::new(1.0, crate::style::text()),
        StrokeKind::Inside,
    );
    // Grip lines in the resize corner.
    let grip = Stroke::new(1.0, crate::style::adapt(Color32::from_gray(140)));
    for d in [4.0, 8.0] {
        painter.line_segment([frame.max - vec2(d, 2.0), frame.max - vec2(2.0, d)], grip);
    }
    Some(response)
}

/// A plot drawn onto its path: values stand off the path, perpendicular to it on screen, the
/// largest by `height` points; returns the response while the plot is grabbed.
fn draw_on_path(
    ui: &Ui,
    painter: &Painter,
    rect: Rect,
    camera: &Camera,
    plot: &mut PlotBox,
    id: egui::Id,
) -> Option<egui::Response> {
    let [a, b] = plot.ends.map(|p| project(camera, rect, p));
    let along = b - a;
    if along.length() < 1.0 {
        return None;
    }
    // Perpendicular on screen, to the upper side of the path.
    let mut normal = along.normalized().rot90();
    if normal.y > 0.0 || (normal.y == 0.0 && normal.x > 0.0) {
        normal = -normal;
    }
    // A negative height puts the plot on the other side of the path.
    let flip = if plot.placement.height < 0.0 {
        -1.0
    } else {
        1.0
    };
    normal *= flip;
    let length = plot.distances.last().copied().unwrap_or(0.0);
    let extreme = (plot.values.iter().copied())
        .filter(|v| v.is_finite())
        .max_by(|x, y| x.abs().total_cmp(&y.abs()))?;
    let reach = match plot.y_range() {
        Some([low, high]) => low.abs().max(high.abs()),
        None => extreme.abs(),
    };
    let scale = if reach > 0.0 {
        plot.placement.height.abs() / reach as f32
    } else {
        0.0
    };
    let base = |d: f64| a + along * (d / length.max(1e-300)) as f32;
    let points: Vec<Option<(Pos2, Pos2)>> = (plot.distances.iter().zip(&plot.values))
        .map(|(&d, &v)| {
            v.is_finite()
                .then(|| (base(d), base(d) + normal * v as f32 * scale))
        })
        .collect();
    // Translucent area between path and curve, then the curve in gap-separated pieces.
    let mut fill = egui::Mesh::default();
    for pair in points.windows(2) {
        if let [Some((b0, p0)), Some((b1, p1))] = pair {
            let i = fill.vertices.len() as u32;
            for corner in [*b0, *b1, *p1, *p0] {
                fill.colored_vertex(corner, FILL);
            }
            fill.add_triangle(i, i + 1, i + 2);
            fill.add_triangle(i, i + 2, i + 3);
        }
    }
    painter.add(fill);
    let stroke = Stroke::new(2.0, CURVE);
    painter.line_segment([a, b], Stroke::new(1.5, CURVE));
    let mut curve: Vec<Pos2> = Vec::new();
    for point in points.iter().chain([&None]) {
        match point {
            Some((_, p)) => curve.push(*p),
            None if curve.len() > 1 => {
                painter.add(egui::Shape::line(std::mem::take(&mut curve), stroke));
            }
            None => curve.clear(),
        }
    }
    let on_path = |d: f64, v: f64| base(d) + normal * v as f32 * scale;
    if let Some(marks) = &plot.marks {
        let line: Vec<Pos2> = (marks.line.iter())
            .filter(|p| p[1].is_finite())
            .map(|p| on_path(p[0], p[1]))
            .collect();
        painter.extend(egui::Shape::dashed_line(
            &line,
            Stroke::new(1.5, crate::style::viewport_text()),
            6.0,
            4.0,
        ));
        for &[d, s] in marks.support.iter().chain([&[0.0, marks.hot_spot]]) {
            if s.is_finite() {
                painter.circle_filled(on_path(d, s), 4.0, crate::style::lift(MARK));
            }
        }
    }
    // Labels at the smallest and largest value, and the title beside the middle of the path,
    // beyond everything drawn on its side so that it never covers the curve or a label.
    let font = egui::FontId::proportional(13.0);
    let finite = || (plot.values.iter().enumerate()).filter(|(_, v)| v.is_finite());
    let min = finite().min_by(|x, y| x.1.total_cmp(y.1));
    let max = finite().max_by(|x, y| x.1.total_cmp(y.1));
    let title_side = if extreme >= 0.0 { normal } else { -normal };
    // How far a point lies from the path towards the title.
    let reach = |p: Pos2| (p - a).dot(title_side);
    let mut extent = (points.iter().flatten())
        .map(|(_, p)| reach(*p))
        .fold(0.0, f32::max);
    let mut label = |index: usize, value: f64| {
        let Some((_, p)) = points[index] else {
            return;
        };
        let side = if value >= 0.0 { normal } else { -normal };
        let rect = painter.text(
            p + side * 4.0,
            anchor(side),
            format_value(value as f32),
            font.clone(),
            crate::style::viewport_text(),
        );
        let corners = [
            rect.left_top(),
            rect.right_top(),
            rect.left_bottom(),
            rect.right_bottom(),
        ];
        extent = corners.into_iter().map(reach).fold(extent, f32::max);
    };
    if let Some((i, &v)) = max {
        label(i, v);
    }
    if let Some((i, &v)) = min.filter(|(i, _)| Some(*i) != max.map(|m| m.0)) {
        label(i, v);
    }
    let title_at = a + along * 0.5 + title_side * (extent + 8.0);
    let galley = painter.layout_no_wrap(plot.shown_title().to_string(), font, CURVE);
    let mut title = anchor(title_side).anchor_size(title_at, galley.size());
    // Beside a slanted path one end of the title reaches back towards it.
    let corners = [
        title.left_top(),
        title.right_top(),
        title.left_bottom(),
        title.right_bottom(),
    ];
    let nearest = corners.into_iter().map(reach).fold(f32::INFINITY, f32::min);
    if nearest < extent + 8.0 {
        title = title.translate(title_side * (extent + 8.0 - nearest));
    }
    painter.galley(title.min, galley, CURVE);
    // Grabbed where the pointer is near the curve or the path, or while dragged or its menu
    // is open, so that the rest of the view keeps its clicks.
    let pointer = ui.input(|i| i.pointer.hover_pos());
    let near = pointer.is_some_and(|pointer| {
        let segments = (points.windows(2)).filter_map(|pair| match pair {
            [Some((_, p0)), Some((_, p1))] => Some((*p0, *p1)),
            _ => None,
        });
        segments
            .chain([(a, b)])
            .any(|(p0, p1)| distance_to_segment(pointer, p0, p1) < GRAB)
    });
    let menu_open = egui::Popup::is_id_open(ui.ctx(), id.with("popup"));
    // A press on the curve keeps it grabbed until released, also before the pointer has
    // moved far enough for egui to call it a drag.
    let (pressed, down) = ui.input(|i| (i.pointer.primary_pressed(), i.pointer.primary_down()));
    let held_id = id.with("held");
    let held = down && (near && pressed || ui.data(|d| d.get_temp(held_id).unwrap_or(false)));
    ui.data_mut(|d| d.insert_temp(held_id, held));
    let dragged = ui.ctx().is_being_dragged(id);
    if !(near || menu_open || dragged || held) {
        return None;
    }
    let pointer = pointer.unwrap_or(a);
    let response = ui.interact(
        Rect::from_center_size(pointer, Vec2::splat(4.0 * GRAB)),
        id,
        Sense::click_and_drag(),
    );
    if response.dragged_by(PointerButton::Primary) {
        // Dragging across the path turns the height negative and flips the plot.
        let away = response.drag_delta().dot(title_side) * flip;
        plot.placement.height = (plot.placement.height + away).clamp(-2000.0, 2000.0);
        ui.ctx().set_cursor_icon(CursorIcon::Grabbing);
    } else if response.hovered() {
        ui.ctx().set_cursor_icon(CursorIcon::Grab);
    }
    Some(response)
}

/// Text anchor that puts a label on the `side` of a point.
fn anchor(side: Vec2) -> egui::Align2 {
    let h = if side.x > 0.5 {
        egui::Align::Min
    } else if side.x < -0.5 {
        egui::Align::Max
    } else {
        egui::Align::Center
    };
    let v = if side.y > 0.5 {
        egui::Align::Min
    } else if side.y < -0.5 {
        egui::Align::Max
    } else {
        egui::Align::Center
    };
    egui::Align2([h, v])
}

fn distance_to_segment(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_sq().max(1e-12)).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// The Edit Annotation dialog of a path plot: display, height on the path, title, axis
/// labels and value range. Changes show at once; Cancel restores the plot.
pub struct EditDialog {
    /// The results file and the plot in its list.
    pub result: usize,
    pub index: usize,
    original: Placement,
    pub draft: Placement,
    /// Range entered for a fixed value axis, kept while it is automatic.
    range: [f64; 2],
    /// A table's plot, which has no path to be drawn on.
    table: bool,
}

pub enum EditResult {
    Open,
    Ok,
    Cancel,
}

impl EditDialog {
    /// `extremes` are the plot's current values, the start of a fixed range.
    pub fn new(
        result: usize,
        index: usize,
        placement: &Placement,
        extremes: [f64; 2],
        table: bool,
    ) -> Self {
        Self {
            table,
            result,
            index,
            original: placement.clone(),
            draft: placement.clone(),
            // Rounded as the plot labels them, a tidy start for the user's own range.
            range: placement
                .y_range
                .unwrap_or(extremes.map(|v| format_value(v as f32).parse().unwrap_or(v))),
        }
    }

    pub fn original(&self) -> &Placement {
        &self.original
    }

    pub fn show(&mut self, ctx: &egui::Context) -> EditResult {
        let mut result = EditResult::Open;
        let mut open = true;
        let draft = &mut self.draft;
        let table = self.table;
        egui::Window::new("Edit Annotation")
            .id(crate::ids::window(ctx, "path plot edit"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::LEFT_TOP)
            .default_pos(ctx.content_rect().left_top() + vec2(300.0, 90.0))
            .show(ctx, |ui| {
                egui::Grid::new("path plot edit grid")
                    .num_columns(2)
                    .spacing([12.0, 4.0])
                    .show(ui, |ui| {
                        if !table {
                            ui.label("Display");
                            ui.horizontal(|ui| {
                                ui.radio_value(&mut draft.on_path, false, "In Box");
                                ui.radio_value(&mut draft.on_path, true, "On Path");
                            });
                            ui.end_row();
                            ui.label("Height on path");
                            ui.add_enabled(
                                draft.on_path,
                                egui::DragValue::new(&mut draft.height)
                                    .custom_parser(crate::numeric::evaluate)
                                    .range(-2000.0..=2000.0)
                                    .suffix(" px"),
                            );
                            ui.end_row();
                        }
                        ui.label("Title");
                        ui.add(
                            egui::TextEdit::singleline(&mut draft.title)
                                .hint_text(if table {
                                    "Table and component"
                                } else {
                                    "Path and component"
                                })
                                .desired_width(220.0),
                        );
                        ui.end_row();
                        ui.label(if table {
                            "X axis label"
                        } else {
                            "Distance axis label"
                        });
                        ui.add_enabled(
                            !draft.on_path,
                            egui::TextEdit::singleline(&mut draft.x_label).desired_width(220.0),
                        );
                        ui.end_row();
                        ui.label("Value axis label");
                        ui.add_enabled(
                            !draft.on_path,
                            egui::TextEdit::singleline(&mut draft.y_label)
                                .hint_text("None")
                                .desired_width(220.0),
                        );
                        ui.end_row();
                        ui.label("Value range");
                        let mut automatic = draft.y_range.is_none();
                        if ui.checkbox(&mut automatic, "Automatic").changed() {
                            draft.y_range = (!automatic).then_some(self.range);
                        }
                        ui.end_row();
                        for (label, k) in [("Minimum", 0), ("Maximum", 1)] {
                            ui.label(label);
                            let changed = ui
                                .add_enabled(
                                    !automatic,
                                    crate::numeric::drag_value(&mut self.range[k]),
                                )
                                .changed();
                            if changed && !automatic {
                                draft.y_range = Some(self.range);
                            }
                            ui.end_row();
                        }
                    });
                if draft.on_path {
                    ui.weak(
                        "On the path, the larger end of the value range is drawn at the \
                         height.",
                    );
                }
                ui.separator();
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if ui.button("Cancel").clicked() {
                        result = EditResult::Cancel;
                    }
                    if ui.button("OK").clicked() {
                        result = EditResult::Ok;
                    }
                });
            });
        if !open || ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            result = EditResult::Cancel;
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::pos2;

    #[test]
    fn labels_sit_on_the_given_side() {
        assert_eq!(anchor(vec2(0.0, -1.0)), egui::Align2::CENTER_BOTTOM);
        assert_eq!(anchor(vec2(1.0, 0.0)), egui::Align2::LEFT_CENTER);
    }

    #[test]
    fn distance_to_segment_clamps_to_its_ends() {
        let (a, b) = (pos2(0.0, 0.0), pos2(10.0, 0.0));
        assert_eq!(distance_to_segment(pos2(5.0, 3.0), a, b), 3.0);
        assert_eq!(distance_to_segment(pos2(14.0, 3.0), a, b), 5.0);
    }
}
