//! Annotations drawn over the 3D view: legend top left, axis triad top right with the
//! information block left of it, scale bar bottom centre above the floating bar, plus markers at the minimum and
//! maximum and a triad at the global origin. Legend, information block, scale bar and markers
//! can be dragged within the view.

use egui::{
    Align2, Color32, CursorIcon, FontId, Painter, Pos2, Rect, Sense, Stroke, StrokeKind, Ui, Vec2,
    pos2, vec2,
};
use glam::Vec3;
use plx_render::Camera;

use crate::model::SECONDARY_HIGHLIGHT_COLOR;
use crate::results::{Legend, format_legend_value, format_value};

const MARGIN: f32 = 20.0;
/// Length of the axes of the corner triad, and its distance from the corner.
const TRIAD: f32 = 45.0;
const TRIAD_INSET: f32 = 24.0;
/// Legend band height; PrePoMax's boxes are a third wider than high.
const BAND_HEIGHT: f32 = 18.0;
const SCALE_BAR_WIDTH: f32 = 400.0;
/// The room the floating bar in the bottom centre of the view takes, with its margins.
const FLOATING_BAR: f32 = 62.0;
const TRIAD_PICK_RADIUS: f32 = 9.0;
const SCALE_BAR_HEIGHT: f32 = 8.0;
const SCALE_BAR_FIELDS: usize = 5;
const STATUS_PADDING: f32 = 5.0;
const MARKER_PADDING: Vec2 = vec2(6.0, 3.0);

/// Everything drawn over the scene.
#[derive(Default)]
pub struct Overlay {
    pub legend: Option<Legend>,
    /// Lines of the information block in the top right corner; empty to hide it.
    pub status: Vec<String>,
    pub maximum: Option<Marker>,
    pub minimum: Option<Marker>,
    /// Labels left by the query tool, drawn like the markers; a right click offers deleting
    /// them.
    pub annotations: Vec<Marker>,
    /// The query annotation each of `annotations` belongs to.
    pub annotation_owners: Vec<usize>,
    /// Global origin in render coordinates, where the global axis triad is drawn.
    pub global_origin: Option<Vec3>,
    pub show_scale_bar: bool,
    /// Length unit written after the scale bar's last label, empty without units.
    pub length_unit: &'static str,
    /// Factor from the lengths of the view to those the scale bar shows: the model is
    /// stored in SI units; `None` for 1.
    pub length_factor: Option<f32>,
    pub show_view_triad: bool,
    /// Selected nodes in render coordinates, drawn as highlighted points.
    pub nodes: Vec<Vec3>,
    /// Selected lines in render coordinates, such as edges of 2D elements.
    pub edges: Vec<[Vec3; 2]>,
    /// Selected lines in the secondary highlight colour, such as slave edges.
    pub secondary_edges: Vec<[Vec3; 2]>,
    /// The axis of revolution of an axisymmetric model in render coordinates, drawn as a
    /// dash-dotted line between the two points.
    pub axis: Option<[Vec3; 2]>,
    /// Hot spot paths in render coordinates: the toe, then the read-out points.
    pub paths: Vec<Vec<Vec3>>,
    /// Highlighted polylines in render coordinates, e.g. the axis of a pattern.
    pub lines: Vec<Vec<Vec3>>,
    /// Reference points, coordinate systems and planes of the model.
    pub features: Vec<FeatureMark>,
    /// A result path in render coordinates: start, end and its name.
    pub result_path: Option<([Vec3; 2], String)>,
}

/// A feature drawn in the 3D view, in render coordinates.
pub enum FeatureMark {
    Point {
        position: Vec3,
        name: String,
        selected: bool,
    },
    /// A coordinate system's origin and unit axes x, y, z.
    System {
        origin: Vec3,
        axes: [Vec3; 3],
        name: String,
        selected: bool,
    },
    /// A plane as a rectangle on it, with its centre and unit normal.
    Plane {
        corners: [Vec3; 4],
        center: Vec3,
        normal: Vec3,
        name: String,
        selected: bool,
    },
}

/// Annotated point of the model, e.g. the node with the largest result value.
pub struct Marker {
    pub position: Vec3,
    pub text: String,
}

/// What the context menu of a query annotation asked for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnnotationCommand {
    /// Delete the query annotation with this index.
    Delete(usize),
    DeleteAll,
}

/// What was clicked in the overlay.
#[derive(Default)]
pub struct OverlayResponse {
    /// An axis direction clicked in the corner triad.
    pub axis: Option<Vec3>,
    pub annotation: Option<AnnotationCommand>,
    /// The legend was double clicked or its context menu asked for the colour scale.
    pub edit_legend: bool,
}

/// Where the user dragged the labels to, relative to their default places.
#[derive(Default)]
pub struct LabelOffsets {
    pub legend: Vec2,
    pub status: Vec2,
    pub scale_bar: Vec2,
    /// Box position relative to the marked point, once dragged.
    pub maximum: Option<Vec2>,
    pub minimum: Option<Vec2>,
    /// One per annotation of the query tool.
    pub annotations: Vec<Option<Vec2>>,
}

fn font() -> FontId {
    FontId::proportional(14.0)
}

/// Screen position of a point in render coordinates.
pub fn project(camera: &Camera, rect: Rect, point: Vec3) -> Pos2 {
    let ndc = camera.view_proj(rect.aspect_ratio()).project_point3(point);
    pos2(
        rect.center().x + ndc.x * rect.width() * 0.5,
        rect.center().y - ndc.y * rect.height() * 0.5,
    )
}

/// Draws the annotations; returns what was clicked in them.
pub fn draw(
    ui: &Ui,
    rect: Rect,
    camera: &Camera,
    overlay: &Overlay,
    offsets: &mut LabelOffsets,
) -> OverlayResponse {
    let painter = ui.painter_at(rect);
    let mut clicked = OverlayResponse::default();
    if let Some(origin) = overlay.global_origin {
        let center = project(camera, rect, origin);
        if rect.contains(center) {
            triad(&painter, camera, center, 36.0, false);
        }
    }
    if let Some([a, b]) = overlay.axis {
        dash_dotted(&painter, project(camera, rect, a), project(camera, rect, b));
    }
    let secondary = SECONDARY_HIGHLIGHT_COLOR.map(|c| (c * 255.0).round() as u8);
    let secondary = Color32::from_rgb(secondary[0], secondary[1], secondary[2]);
    for (edges, color) in [
        (&overlay.secondary_edges, secondary),
        (&overlay.edges, Color32::RED),
    ] {
        for &[a, b] in edges {
            painter.line_segment(
                [project(camera, rect, a), project(camera, rect, b)],
                Stroke::new(3.0, color),
            );
        }
    }
    let view_proj = camera.view_proj(rect.aspect_ratio());
    for &node in &overlay.nodes {
        let ndc = view_proj.project_point3(node);
        let point = pos2(
            rect.center().x + ndc.x * rect.width() * 0.5,
            rect.center().y - ndc.y * rect.height() * 0.5,
        );
        painter.rect_filled(
            Rect::from_center_size(point, vec2(5.0, 5.0)),
            0.0,
            Color32::RED,
        );
    }
    for line in &overlay.lines {
        let points: Vec<Pos2> = line.iter().map(|&p| project(camera, rect, p)).collect();
        painter.add(egui::Shape::line(
            points,
            egui::Stroke::new(2.0, Color32::RED),
        ));
    }
    for path in &overlay.paths {
        let points: Vec<Pos2> = path.iter().map(|&p| project(camera, rect, p)).collect();
        draw_path(&painter, &points);
    }
    for feature in &overlay.features {
        draw_feature(&painter, camera, rect, feature);
    }
    if let Some(([a, b], name)) = &overlay.result_path {
        let (a, b) = (project(camera, rect, *a), project(camera, rect, *b));
        let stroke = Stroke::new(2.5, crate::style::lift(PATH_COLOR));
        painter.line_segment([a, b], stroke);
        for end in [a, b] {
            painter.circle(end, 4.0, Color32::WHITE, stroke);
        }
        painter.text(
            a + vec2(6.0, -6.0),
            Align2::LEFT_BOTTOM,
            name,
            font(),
            crate::style::lift(PATH_COLOR),
        );
    }
    if overlay.show_scale_bar {
        let (unit, factor) = (overlay.length_unit, overlay.length_factor.unwrap_or(1.0));
        draw_scale_bar(
            ui,
            &painter,
            rect,
            camera,
            (unit, factor),
            &mut offsets.scale_bar,
        );
    }
    if overlay.show_view_triad {
        let corner = triad_center(rect);
        triad(&painter, camera, corner, TRIAD, true);
        clicked.axis = triad_buttons(ui, &painter, camera, corner, TRIAD);
    }
    for (marker, offset, id) in [
        (&overlay.minimum, &mut offsets.minimum, "minimum label"),
        (&overlay.maximum, &mut offsets.maximum, "maximum label"),
    ] {
        if let Some(marker) = marker {
            let point = project(camera, rect, marker.position);
            draw_marker(ui, rect, point, &marker.text, offset, id, Sense::drag());
        }
    }
    offsets.annotations.resize(overlay.annotations.len(), None);
    for (index, (marker, offset)) in (overlay.annotations.iter())
        .zip(&mut offsets.annotations)
        .enumerate()
    {
        let point = project(camera, rect, marker.position);
        let id = format!("annotation {index}");
        let text = &marker.text;
        let sense = Sense::click_and_drag();
        let Some(response) = draw_marker(ui, rect, point, text, offset, &id, sense) else {
            continue;
        };
        // PrePoMax's annotation menu; Remove Annotations of its tool bar as well.
        response.context_menu(|ui| {
            if ui.button("Delete").clicked() {
                clicked.annotation = (overlay.annotation_owners.get(index))
                    .map(|&owner| AnnotationCommand::Delete(owner));
            }
            if ui.button("Delete All Annotations").clicked() {
                clicked.annotation = Some(AnnotationCommand::DeleteAll);
            }
        });
    }
    if let Some(legend) = &overlay.legend {
        let response = draw_legend(ui, &painter, rect, legend, &mut offsets.legend);
        clicked.edit_legend = response.double_clicked();
        response.context_menu(|ui| {
            if ui.button("Color Scale …").clicked() {
                clicked.edit_legend = true;
                ui.close();
            }
        });
    }
    if !overlay.status.is_empty() {
        // Left of the corner triad, which takes the top right corner.
        let room = if overlay.show_view_triad {
            2.0 * (TRIAD_INSET + TRIAD) - MARGIN
        } else {
            0.0
        };
        draw_status(
            ui,
            &painter,
            rect,
            room,
            &overlay.status,
            &mut offsets.status,
        );
    }
    clicked
}

/// A dash-dotted line, the technical drawing's line for axes of symmetry.
fn dash_dotted(painter: &Painter, a: Pos2, b: Pos2) {
    const PATTERN: [(f32, bool); 4] = [(18.0, true), (4.0, false), (3.0, true), (4.0, false)];
    let length = a.distance(b);
    if length < 1.0 {
        return;
    }
    let direction = (b - a) / length;
    let stroke = Stroke::new(1.5, crate::style::lift(AXIS_COLOR));
    let mut position = 0.0;
    'line: loop {
        for (dash, drawn) in PATTERN {
            let end = (position + dash).min(length);
            if drawn {
                painter.line_segment([a + direction * position, a + direction * end], stroke);
            }
            position = end;
            if position >= length {
                break 'line;
            }
        }
    }
}

/// Colour of the axis of revolution.
const AXIS_COLOR: Color32 = Color32::from_rgb(40, 40, 160);

/// Makes the ends of the corner triad clickable: an axis tip, or the end of its faint negative
/// part, sets the view to look down that direction.
fn triad_buttons(
    ui: &Ui,
    painter: &Painter,
    camera: &Camera,
    center: Pos2,
    length: f32,
) -> Option<Vec3> {
    let screen = |axis: Vec3| vec2(axis.dot(camera.right()), -axis.dot(camera.up()));
    let mut ends = Vec::new();
    for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
        let v = screen(axis);
        // Axes pointing at the viewer collapse onto the centre and cannot be told apart.
        if v.length() < 0.2 {
            continue;
        }
        ends.push((axis, center + v * (length + 4.0)));
        ends.push((-axis, center - v * length * 2.0 / 3.0));
    }
    let pointer = ui.input(|i| i.pointer.hover_pos());
    // Nearest end under the pointer; ends towards the viewer win ties.
    let hovered = pointer.and_then(|pointer| {
        ends.iter()
            .filter(|(_, end)| end.distance(pointer) < TRIAD_PICK_RADIUS)
            .min_by(|a, b| {
                let key =
                    |(axis, end): &(Vec3, Pos2)| end.distance(pointer) + axis.dot(camera.forward());
                key(a).total_cmp(&key(b))
            })
            .copied()
    });
    let (axis, end) = hovered?;
    let area = Rect::from_center_size(end, Vec2::splat(2.0 * TRIAD_PICK_RADIUS));
    let response = ui.interact(area, ui.id().with("triad axis"), Sense::click());
    ui.ctx().set_cursor_icon(CursorIcon::PointingHand);
    painter.circle_stroke(end, 7.0, Stroke::new(1.5, Color32::from_rgb(255, 160, 0)));
    response.clicked().then_some(axis)
}

/// A hot spot path: a line from the toe, a square on the toe and rings on the read-out
/// points.
fn draw_path(painter: &egui::Painter, points: &[Pos2]) {
    let color = crate::style::lift(Color32::from_rgb(0, 70, 200));
    let Some((&toe, readouts)) = points.split_first() else {
        return;
    };
    if let Some(&last) = readouts.last() {
        painter.line_segment([toe, last], egui::Stroke::new(1.5, color));
    }
    painter.rect_filled(Rect::from_center_size(toe, vec2(6.0, 6.0)), 0.0, color);
    for &point in readouts {
        painter.circle(point, 3.5, Color32::WHITE, egui::Stroke::new(1.5, color));
    }
}

/// Places a draggable label: its default position plus the user's offset, kept inside the
/// view. Dragging moves the offset. Returns where it sits and its response; labels with a
/// context menu sense clicks as well.
fn drag_label(
    ui: &Ui,
    rect: Rect,
    id: &str,
    default: Pos2,
    size: Vec2,
    offset: &mut Vec2,
    sense: Sense,
) -> (Rect, egui::Response) {
    let frame = Rect::from_min_size(default + *offset, size);
    let response = ui.interact(frame, ui.id().with(id), sense);
    if response.dragged() {
        *offset += response.drag_delta();
        ui.ctx().set_cursor_icon(CursorIcon::Grabbing);
    } else if response.hovered() {
        ui.ctx().set_cursor_icon(CursorIcon::Grab);
    }
    let mut min = default + *offset;
    min.x = min
        .x
        .clamp(rect.left(), (rect.right() - size.x).max(rect.left()));
    min.y = min
        .y
        .clamp(rect.top(), (rect.bottom() - size.y).max(rect.top()));
    *offset = min - default;
    (Rect::from_min_size(min, size), response)
}

/// Title lines, then one box per band with the band limits as labels, maximum on top; a
/// manual scale adds boxes for the values beyond its limits, labelled with their extremes.
/// Returns the legend's response, for its double click and context menu.
fn draw_legend(
    ui: &Ui,
    painter: &Painter,
    rect: Rect,
    legend: &Legend,
    offset: &mut Vec2,
) -> egui::Response {
    let titles: Vec<_> = legend
        .title
        .lines()
        .map(|l| painter.layout_no_wrap(l.to_string(), font(), crate::style::viewport_text()))
        .collect();
    let levels = legend.levels();
    let above = legend.above.as_ref().filter(|o| o.shown);
    let below = legend.below.as_ref().filter(|o| o.shown);
    let mut boxes: Vec<[f32; 3]> = above.iter().map(|o| o.color).collect();
    boxes.extend(legend.colors.iter().rev());
    boxes.extend(below.iter().map(|o| o.color));
    let mut labels: Vec<f32> = above.iter().map(|o| o.value).collect();
    labels.extend((0..=levels).map(|i| {
        let t = 1.0 - i as f32 / levels as f32;
        legend.min + t * (legend.max - legend.min)
    }));
    labels.extend(below.iter().map(|o| o.value));
    let values: Vec<_> = labels
        .into_iter()
        .map(|value| {
            painter.layout_no_wrap(
                format_legend_value(value),
                font(),
                crate::style::viewport_text(),
            )
        })
        .collect();
    let band = vec2(BAND_HEIGHT / 0.75, BAND_HEIGHT);
    let label_height = values.first().map_or(0.0, |g| g.size().y);
    let title_height: f32 = titles.iter().map(|g| g.size().y + 2.0).sum::<f32>() + 6.0;
    let width = titles
        .iter()
        .map(|g| g.size().x)
        .chain(values.iter().map(|g| band.x + 8.0 + g.size().x))
        .fold(0.0, f32::max);
    let size = vec2(
        width,
        title_height + boxes.len() as f32 * band.y + label_height * 0.5,
    );
    let default = rect.left_top() + vec2(MARGIN, MARGIN);
    let sense = Sense::click_and_drag();
    let (frame, response) = drag_label(ui, rect, "legend", default, size, offset, sense);

    let mut top = frame.top();
    for galley in titles {
        let height = galley.size().y;
        painter.galley(
            pos2(frame.left(), top),
            galley,
            crate::style::viewport_text(),
        );
        top += height + 2.0;
    }
    top += 6.0;
    for (i, color) in boxes.into_iter().enumerate() {
        let [r, g, b] = color.map(|c| (c * 255.0).round() as u8);
        let cell = Rect::from_min_size(pos2(frame.left(), top + i as f32 * band.y), band);
        painter.rect(
            cell,
            0.0,
            Color32::from_rgb(r, g, b),
            Stroke::new(1.0, crate::style::viewport_text()),
            StrokeKind::Middle,
        );
    }
    for (i, galley) in values.into_iter().enumerate() {
        let y = top + i as f32 * band.y - galley.size().y * 0.5;
        painter.galley(
            pos2(frame.left() + band.x + 8.0, y),
            galley,
            crate::style::viewport_text(),
        );
    }
    response
}

/// The centre of the corner triad, in the top right corner of the view.
pub fn triad_center(rect: Rect) -> Pos2 {
    rect.right_top() + vec2(-(TRIAD_INSET + TRIAD), TRIAD_INSET + TRIAD)
}

/// Framed text block, by default in the top right corner, `room` points left of it.
fn draw_status(
    ui: &Ui,
    painter: &Painter,
    rect: Rect,
    room: f32,
    lines: &[String],
    offset: &mut Vec2,
) {
    let galleys: Vec<_> = lines
        .iter()
        .map(|l| painter.layout_no_wrap(l.clone(), font(), crate::style::viewport_text()))
        .collect();
    let width = galleys.iter().map(|g| g.size().x).fold(0.0, f32::max);
    let line_height = galleys.first().map_or(0.0, |g| g.size().y) + 2.0;
    let size = vec2(width, line_height * galleys.len() as f32)
        + vec2(2.0 * STATUS_PADDING, STATUS_PADDING);
    let default = pos2(rect.right() - MARGIN - room - size.x, rect.top() + MARGIN);
    let (frame, _) = drag_label(ui, rect, "status", default, size, offset, Sense::drag());
    painter.rect_stroke(
        frame,
        0.0,
        Stroke::new(1.0, crate::style::viewport_text()),
        StrokeKind::Inside,
    );
    for (i, galley) in galleys.into_iter().enumerate() {
        let pos = frame.min
            + vec2(
                STATUS_PADDING,
                STATUS_PADDING * 0.5 + i as f32 * line_height,
            );
        painter.galley(pos, galley, crate::style::viewport_text());
    }
}

/// Bar of five alternating fields whose total length is a round number of model units, by
/// by default centred at the bottom above the floating bar; dragging it moves the bar together with its labels.
fn draw_scale_bar(
    ui: &Ui,
    painter: &Painter,
    rect: Rect,
    camera: &Camera,
    (unit, factor): (&str, f32),
    offset: &mut Vec2,
) {
    let world_per_point = camera.pixel_size(rect.width(), rect.height()) * factor;
    let Some((length, width)) = scale_bar_length(world_per_point, SCALE_BAR_WIDTH) else {
        return;
    };
    let labels: Vec<_> = (0..=SCALE_BAR_FIELDS)
        .map(|i| {
            let value = length * i as f32 / SCALE_BAR_FIELDS as f32;
            // PrePoMax writes the unit after the last label.
            let text = match i == SCALE_BAR_FIELDS && !unit.is_empty() {
                true => format!("{} {unit}", format_value(value)),
                false => format_value(value),
            };
            painter.layout_no_wrap(text, font(), crate::style::viewport_text())
        })
        .collect();
    // The outer labels are centred on the bar ends and stick out by half their width.
    let overhang =
        |galley: Option<&std::sync::Arc<egui::Galley>>| galley.map_or(0.0, |g| g.size().x * 0.5);
    let (left_overhang, right_overhang) = (overhang(labels.first()), overhang(labels.last()));
    let label_height = labels.first().map_or(0.0, |g| g.size().y);
    let size = vec2(
        left_overhang + width + right_overhang,
        label_height + 3.0 + SCALE_BAR_HEIGHT,
    );
    // Above the floating bar in the bottom centre.
    let default = pos2(
        rect.center().x - width * 0.5 - left_overhang,
        rect.bottom() - FLOATING_BAR - size.y,
    );
    let (frame, _) = drag_label(ui, rect, "scale bar", default, size, offset, Sense::drag());
    let left = frame.left() + left_overhang;
    let top = frame.bottom() - SCALE_BAR_HEIGHT;
    let field = width / SCALE_BAR_FIELDS as f32;
    for i in 0..SCALE_BAR_FIELDS {
        let cell = Rect::from_min_size(
            pos2(left + i as f32 * field, top),
            vec2(field, SCALE_BAR_HEIGHT),
        );
        let fill = if i % 2 == 0 {
            crate::style::viewport_text()
        } else {
            crate::style::label_fill()
        };
        painter.rect(
            cell,
            0.0,
            fill,
            Stroke::new(1.0, crate::style::viewport_text()),
            StrokeKind::Middle,
        );
    }
    for (i, galley) in labels.into_iter().enumerate() {
        let x = left + i as f32 * field - galley.size().x * 0.5;
        painter.galley(pos2(x, frame.top()), galley, crate::style::viewport_text());
    }
}

/// Round length near `room` points (steps of 0.2 of a power of ten) and its width
/// in points.
pub fn scale_bar_length(world_per_point: f32, room: f32) -> Option<(f32, f32)> {
    let nominal = room * world_per_point;
    if !(nominal.is_finite() && nominal > 0.0) {
        return None;
    }
    let power = 10f32.powf(nominal.log10().floor());
    let length = (5.0 * nominal / power).round() / 5.0 * power;
    Some((length, length / world_per_point))
}

/// Framed label next to a model point, with an arrow pointing at it. The box follows the point
/// when the view moves; dragging it changes where it sits relative to the point. Returns the
/// label's response, `None` while the point is outside the view.
fn draw_marker(
    ui: &Ui,
    rect: Rect,
    point: Pos2,
    text: &str,
    offset: &mut Option<Vec2>,
    id: &str,
    sense: Sense,
) -> Option<egui::Response> {
    if !rect.contains(point) {
        return None;
    }
    let painter = ui.painter_at(rect);
    let galley = painter.layout_no_wrap(text.to_string(), font(), crate::style::viewport_text());
    let size = galley.size() + 2.0 * MARKER_PADDING;
    // Above right of the point, or mirrored where the view ends.
    let automatic = || {
        let dx = if point.x + 60.0 + size.x < rect.right() {
            60.0
        } else {
            -60.0 - size.x
        };
        let dy = if point.y - 50.0 - size.y > rect.top() {
            -50.0 - size.y
        } else {
            50.0
        };
        vec2(dx, dy)
    };
    let mut relative = offset.unwrap_or_else(automatic);
    let before = relative;
    let (frame, response) = drag_label(ui, rect, id, point, size, &mut relative, sense);
    if relative != before {
        *offset = Some(relative);
    }
    let anchor = pos2(
        point.x.clamp(frame.left(), frame.right()),
        point.y.clamp(frame.top(), frame.bottom()),
    );
    if anchor.distance(point) > 1.0 {
        painter.line_segment(
            [anchor, point],
            Stroke::new(1.0, crate::style::viewport_text()),
        );
        let direction = (point - anchor).normalized();
        let normal = direction.rot90();
        painter.add(egui::Shape::convex_polygon(
            vec![
                point,
                point - direction * 9.0 + normal * 3.0,
                point - direction * 9.0 - normal * 3.0,
            ],
            crate::style::viewport_text(),
            Stroke::NONE,
        ));
    }
    painter.rect(
        frame,
        0.0,
        crate::style::label_fill(),
        Stroke::new(1.0, crate::style::viewport_text()),
        StrokeKind::Inside,
    );
    painter.galley(
        frame.min + MARKER_PADDING,
        galley,
        crate::style::viewport_text(),
    );
    Some(response)
}

/// Colour of result paths.
const PATH_COLOR: Color32 = Color32::from_rgb(200, 0, 160);
/// Colour of reference points and the names of features.
const FEATURE_COLOR: Color32 = Color32::from_rgb(0, 110, 160);
const SELECTED_FEATURE: Color32 = Color32::RED;

fn draw_feature(painter: &Painter, camera: &Camera, rect: Rect, feature: &FeatureMark) {
    match feature {
        FeatureMark::Point {
            position,
            name,
            selected,
        } => {
            let point = project(camera, rect, *position);
            if !rect.contains(point) {
                return;
            }
            let color = if *selected {
                SELECTED_FEATURE
            } else {
                crate::style::lift(FEATURE_COLOR)
            };
            painter.circle(point, 4.5, Color32::WHITE, Stroke::new(2.0, color));
            painter.circle_filled(point, 1.5, color);
            painter.text(
                point + vec2(7.0, -5.0),
                Align2::LEFT_BOTTOM,
                name,
                font(),
                color,
            );
        }
        FeatureMark::System {
            origin,
            axes,
            name,
            selected,
        } => {
            let center = project(camera, rect, *origin);
            if !rect.contains(center) {
                return;
            }
            let [x, y, z] = *axes;
            arrows(
                painter,
                camera,
                center,
                30.0,
                &[
                    (x, "x", crate::style::lift(Color32::from_rgb(255, 0, 0))),
                    (y, "y", crate::style::lift(Color32::from_rgb(0, 200, 0))),
                    (z, "z", crate::style::lift(Color32::from_rgb(0, 0, 255))),
                ],
            );
            let color = if *selected {
                SELECTED_FEATURE
            } else {
                crate::style::lift(FEATURE_COLOR)
            };
            painter.circle_filled(center, 3.0, color);
            painter.text(
                center + vec2(-6.0, 6.0),
                Align2::RIGHT_TOP,
                name,
                font(),
                color,
            );
        }
        FeatureMark::Plane {
            corners,
            center,
            normal,
            name,
            selected,
        } => {
            let color = if *selected {
                SELECTED_FEATURE
            } else {
                crate::style::lift(FEATURE_COLOR)
            };
            let corners: Vec<Pos2> = corners.iter().map(|&c| project(camera, rect, c)).collect();
            let fill = Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), 24);
            painter.add(egui::Shape::convex_polygon(
                corners.clone(),
                fill,
                Stroke::new(if *selected { 2.0 } else { 1.0 }, color),
            ));
            let origin = project(camera, rect, *center);
            arrows(painter, camera, origin, 30.0, &[(*normal, "n", color)]);
            painter.text(
                corners[0] + vec2(4.0, -4.0),
                Align2::LEFT_BOTTOM,
                name,
                font(),
                color,
            );
        }
    }
}

/// Axes X, Y, Z as arrows in screen space, drawn back to front; `full` adds PrePoMax's centre
/// sphere and the shorter, faint negative axes.
fn triad(painter: &Painter, camera: &Camera, center: Pos2, length: f32, full: bool) {
    let axes = [
        (Vec3::X, "X", Color32::from_rgb(255, 0, 0)),
        (Vec3::Y, "Y", Color32::from_rgb(0, 230, 0)),
        (
            Vec3::Z,
            "Z",
            crate::style::lift(Color32::from_rgb(0, 0, 255)),
        ),
    ];
    let screen = |axis: Vec3| vec2(axis.dot(camera.right()), -axis.dot(camera.up()));
    if full {
        let grey = Color32::from_rgba_unmultiplied(90, 90, 90, 128);
        for (axis, _, _) in axes {
            let end = center - screen(axis) * length * 2.0 / 3.0;
            painter.line_segment([center, end], Stroke::new(2.0, grey));
        }
    }
    arrows(painter, camera, center, length, &axes);
    if full {
        painter.circle_filled(center, 3.5, Color32::from_rgb(200, 200, 200));
        painter.circle_stroke(center, 3.5, Stroke::new(0.5, Color32::from_gray(120)));
    }
}

/// Unit directions as arrows in screen space, drawn back to front.
fn arrows(
    painter: &Painter,
    camera: &Camera,
    center: Pos2,
    length: f32,
    axes: &[(Vec3, &str, Color32)],
) {
    let screen = |axis: Vec3| vec2(axis.dot(camera.right()), -axis.dot(camera.up()));
    let mut order: Vec<_> = axes.iter().collect();
    // Farthest first; forward points into the screen.
    order.sort_by(|a, b| {
        let depth = |axis: Vec3| axis.dot(camera.forward());
        depth(b.0).total_cmp(&depth(a.0))
    });
    let cone = length * 14.0 / 45.0;
    for &&(axis, label, color) in &order {
        let v = screen(axis);
        let tip = center + v * length;
        let base = center + v * (length - cone);
        painter.line_segment([center, base], Stroke::new(2.5, color));
        let normal = v.normalized().rot90() * 3.5;
        if v.length() > 0.05 {
            painter.add(egui::Shape::convex_polygon(
                vec![tip, base + normal, base - normal],
                color,
                Stroke::NONE,
            ));
        }
        let label_pos = center + v * length + v.normalized() * 9.0;
        // The light labels of the dark theme get a dark outline, as they may lie on the model.
        if crate::style::dark() {
            for offset in [
                vec2(1.0, 0.0),
                vec2(-1.0, 0.0),
                vec2(0.0, 1.0),
                vec2(0.0, -1.0),
            ] {
                painter.text(
                    label_pos + offset,
                    Align2::CENTER_CENTER,
                    label,
                    font(),
                    crate::style::window(),
                );
            }
        }
        painter.text(
            label_pos,
            Align2::CENTER_CENTER,
            label,
            font(),
            crate::style::viewport_text(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scale_bar_rounds_to_fifths_of_a_decade() {
        // 400 points at 0.37 units each are 148 units: rounded to 140.
        let (length, width) = scale_bar_length(0.37, SCALE_BAR_WIDTH).unwrap();
        assert!((length - 140.0).abs() < 1e-3, "{length}");
        assert!((width - 140.0 / 0.37).abs() < 1e-2);
        assert!(scale_bar_length(0.0, SCALE_BAR_WIDTH).is_none());
    }
}
