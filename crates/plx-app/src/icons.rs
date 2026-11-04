//! Toolbar icons drawn with the egui painter, so that no icon font or emoji is needed.

use egui::{Color32, Pos2, Rect, Response, Sense, Shape, Stroke, Ui, Vec2, pos2, vec2};

use plx_render::StandardView as ViewIcon;

use crate::style::{highlight, hover_fill, pressed_fill};

const BUTTON: Vec2 = vec2(24.0, 22.0);
const ICON: f32 = 16.0;
const OUTLINE: Color32 = Color32::from_rgb(70, 70, 70);
const FACE: Color32 = Color32::from_rgb(232, 232, 232);
const ACCENT: Color32 = Color32::from_rgb(70, 140, 215);

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Icon {
    New,
    Open,
    /// Import of geometry: an arrow pointing into a sheet.
    Import,
    Save,
    Fit,
    /// Standard view; the cube face looked at is highlighted.
    View(ViewIcon),
    /// Vertical view: an arrow standing on the ground line.
    Vertical,
    FeatureEdges,
    MeshEdges,
    First,
    Previous,
    Next,
    Last,
    /// Play triangle; also opens the animation.
    Animate,
    Pause,
    /// Square of a stop button.
    Stop,
    /// Arrow of the material library's copy and move buttons, pointing in this direction
    /// (unit vector, y down).
    Arrow(Vec2),
    /// Camera: screenshot of the 3D view.
    Screenshot,
    /// Loudspeaker with sound waves: the sound of eigenmodes.
    Sound,
    /// Half a cube with its cut face, for the section view.
    SectionView,
    /// Three blocks of a stack pulled apart, for the exploded view.
    ExplodedView,
    /// A shape and its mirror image at a dashed plane: the result transformations.
    Transformation,
    /// Cross of a remove button.
    Remove,
    /// The query: a pipette, which picks values from the model.
    Query,
    /// A label of the query crossed out: remove all annotations.
    RemoveAnnotations,
    /// Lightning bolt of Ansys Workbench's Update: make the analysis up to date.
    Solve,
    /// A window with lines of text: the solver monitor.
    Monitor,
}

/// Flat toolbar button in the Windows style: frame only while hovered or checked.
pub fn button(ui: &mut Ui, icon: Icon, tooltip: &str, enabled: bool, checked: bool) -> Response {
    // Inside a disabled area, e.g. the greyed-out results row, the icon is greyed out too.
    let enabled = enabled && ui.is_enabled();
    let sense = if enabled {
        Sense::click()
    } else {
        Sense::hover()
    };
    let (rect, response) = ui.allocate_exact_size(BUTTON, sense);
    if ui.is_rect_visible(rect) {
        let painter = ui.painter();
        let hovered = enabled && response.hovered();
        if checked || hovered {
            let fill = if checked || response.is_pointer_button_down_on() {
                pressed_fill()
            } else {
                hover_fill()
            };
            painter.rect(
                rect.shrink(1.0),
                0.0,
                fill,
                Stroke::new(1.0, highlight()),
                egui::StrokeKind::Inside,
            );
        }
        let area = Rect::from_center_size(rect.center(), Vec2::splat(ICON));
        let mut shapes = Vec::new();
        paint(&mut shapes, icon, area);
        for shape in &mut shapes {
            map_colors(shape, &crate::style::adapt);
            if !enabled {
                map_colors(shape, &grey);
            }
        }
        painter.extend(shapes);
    }
    response.on_hover_text(tooltip)
}

/// Dialog button with a frame like a text button, showing an icon.
pub fn dialog_button(ui: &mut Ui, icon: Icon, tooltip: &str, enabled: bool) -> Response {
    let response = ui
        .add_enabled(enabled, egui::Button::new("").min_size(vec2(26.0, 24.0)))
        .on_hover_text(tooltip);
    if ui.is_rect_visible(response.rect) {
        let area = Rect::from_center_size(response.rect.center(), Vec2::splat(ICON));
        let mut shapes = Vec::new();
        paint(&mut shapes, icon, area);
        for shape in &mut shapes {
            map_colors(shape, &crate::style::adapt);
            if !enabled {
                map_colors(shape, &grey);
            }
        }
        ui.painter().extend(shapes);
    }
    response
}

/// Paints `icon` into the square `area` of any size, scaled from its 16 x 16 grid: the
/// large icons of the ribbon's tiles.
pub fn paint_in(painter: &egui::Painter, icon: Icon, area: Rect, enabled: bool) {
    let mut shapes = Vec::new();
    paint(
        &mut shapes,
        icon,
        Rect::from_min_size(Pos2::ZERO, Vec2::splat(ICON)),
    );
    let scale = area.width().min(area.height()) / ICON;
    let transform = egui::emath::TSTransform::new(area.min.to_vec2(), scale);
    for shape in &mut shapes {
        shape.transform(transform);
        map_colors(shape, &crate::style::adapt);
        if !enabled {
            map_colors(shape, &grey);
        }
    }
    painter.extend(shapes);
}

/// The colour of a disabled icon: grey and half transparent.
fn grey(c: Color32) -> Color32 {
    let l = ((c.r() as u32 + c.g() as u32 + c.b() as u32) / 3) as u8;
    Color32::from_rgba_unmultiplied(l, l, l, c.a() / 2)
}

fn map_colors(shape: &mut Shape, f: &dyn Fn(Color32) -> Color32) {
    match shape {
        Shape::Path(path) => {
            path.fill = f(path.fill);
            if let egui::epaint::PathStroke {
                color: egui::epaint::ColorMode::Solid(c),
                ..
            } = &mut path.stroke
            {
                *c = f(*c);
            }
        }
        Shape::LineSegment { stroke, .. } => stroke.color = f(stroke.color),
        Shape::Rect(r) => {
            r.fill = f(r.fill);
            r.stroke.color = f(r.stroke.color);
        }
        Shape::Circle(c) => {
            c.fill = f(c.fill);
            c.stroke.color = f(c.stroke.color);
        }
        _ => {}
    }
}

fn line(shapes: &mut Vec<Shape>, points: &[Pos2], width: f32, color: Color32) {
    shapes.push(Shape::line(points.to_vec(), Stroke::new(width, color)));
}

fn polygon(shapes: &mut Vec<Shape>, points: Vec<Pos2>, fill: Color32, stroke: Color32) {
    shapes.push(Shape::convex_polygon(
        points,
        fill,
        Stroke::new(1.0, stroke),
    ));
}

fn paint(shapes: &mut Vec<Shape>, icon: Icon, r: Rect) {
    // Coordinates on a 16 × 16 grid.
    let p = |x: f32, y: f32| pos2(r.left() + x, r.top() + y);
    match icon {
        Icon::Vertical => {
            line(shapes, &[p(2.5, 14.0), p(13.5, 14.0)], 1.5, OUTLINE);
            line(shapes, &[p(8.0, 13.0), p(8.0, 5.0)], 2.0, ACCENT);
            polygon(
                shapes,
                vec![p(8.0, 1.0), p(11.5, 6.0), p(4.5, 6.0)],
                ACCENT,
                ACCENT,
            );
        }
        Icon::New => {
            polygon(
                shapes,
                vec![
                    p(3.5, 1.5),
                    p(10.0, 1.5),
                    p(13.5, 5.0),
                    p(13.5, 14.5),
                    p(3.5, 14.5),
                ],
                Color32::WHITE,
                OUTLINE,
            );
            line(
                shapes,
                &[p(10.0, 1.5), p(10.0, 5.0), p(13.5, 5.0)],
                1.0,
                OUTLINE,
            );
        }
        Icon::Import => {
            polygon(
                shapes,
                vec![
                    p(5.5, 1.5),
                    p(11.0, 1.5),
                    p(14.5, 5.0),
                    p(14.5, 14.5),
                    p(5.5, 14.5),
                ],
                Color32::WHITE,
                OUTLINE,
            );
            line(
                shapes,
                &[p(11.0, 1.5), p(11.0, 5.0), p(14.5, 5.0)],
                1.0,
                OUTLINE,
            );
            line(shapes, &[p(1.0, 9.0), p(7.0, 9.0)], 2.0, ACCENT);
            polygon(
                shapes,
                vec![p(11.0, 9.0), p(6.5, 5.5), p(6.5, 12.5)],
                ACCENT,
                ACCENT,
            );
        }
        Icon::Open => {
            let back = Color32::from_rgb(230, 170, 40);
            let front = Color32::from_rgb(255, 206, 84);
            polygon(
                shapes,
                vec![
                    p(1.0, 3.0),
                    p(6.0, 3.0),
                    p(7.5, 4.5),
                    p(14.0, 4.5),
                    p(14.0, 13.5),
                    p(1.0, 13.5),
                ],
                back,
                Color32::from_rgb(170, 120, 20),
            );
            polygon(
                shapes,
                vec![p(3.5, 7.0), p(15.5, 7.0), p(13.5, 13.5), p(1.0, 13.5)],
                front,
                Color32::from_rgb(170, 120, 20),
            );
        }
        Icon::Save => {
            let blue = Color32::from_rgb(60, 110, 180);
            polygon(
                shapes,
                vec![
                    p(1.5, 1.5),
                    p(12.5, 1.5),
                    p(14.5, 3.5),
                    p(14.5, 14.5),
                    p(1.5, 14.5),
                ],
                blue,
                Color32::from_rgb(30, 60, 110),
            );
            shapes.push(Shape::rect_filled(
                Rect::from_min_max(p(4.0, 1.5), p(11.0, 6.0)),
                0.0,
                Color32::WHITE,
            ));
            shapes.push(Shape::rect_filled(
                Rect::from_min_max(p(8.5, 2.5), p(10.0, 5.0)),
                0.0,
                blue,
            ));
            shapes.push(Shape::rect_filled(
                Rect::from_min_max(p(3.5, 9.0), p(12.5, 14.5)),
                0.0,
                Color32::from_rgb(235, 235, 235),
            ));
        }
        Icon::Fit => {
            shapes.push(Shape::rect_filled(
                Rect::from_min_max(p(5.0, 5.0), p(11.0, 11.0)),
                0.0,
                ACCENT,
            ));
            for (corner, dx, dy) in [
                (p(1.5, 1.5), 1.0, 1.0),
                (p(14.5, 1.5), -1.0, 1.0),
                (p(1.5, 14.5), 1.0, -1.0),
                (p(14.5, 14.5), -1.0, -1.0),
            ] {
                line(
                    shapes,
                    &[
                        corner + vec2(4.0 * dx, 0.0),
                        corner,
                        corner + vec2(0.0, 4.0 * dy),
                    ],
                    1.5,
                    OUTLINE,
                );
            }
        }
        Icon::View(view) => cube(shapes, r, Some(view), false),
        Icon::FeatureEdges => cube(shapes, r, None, false),
        Icon::MeshEdges => cube(shapes, r, None, true),
        Icon::SectionView => section_cube(shapes, r),
        Icon::ExplodedView => exploded_blocks(shapes, r),
        Icon::Animate => {
            let green = Color32::from_rgb(40, 150, 60);
            polygon(
                shapes,
                vec![p(4.0, 2.0), p(14.0, 8.0), p(4.0, 14.0)],
                green,
                Color32::from_rgb(20, 100, 35),
            );
        }
        Icon::Pause => {
            let color = Color32::from_rgb(50, 50, 50);
            for x in [4.0, 9.5] {
                shapes.push(Shape::rect_filled(
                    Rect::from_min_max(p(x, 3.0), p(x + 3.0, 13.0)),
                    0.0,
                    color,
                ));
            }
        }
        Icon::Stop => {
            let color = Color32::from_rgb(50, 50, 50);
            shapes.push(Shape::rect_filled(
                Rect::from_min_max(p(3.5, 3.5), p(12.5, 12.5)),
                1.0,
                color,
            ));
        }
        Icon::Arrow(direction) => {
            // Drawn pointing right, then turned into the direction.
            let c = r.center();
            let normal = direction.rot90();
            let at = |x: f32, y: f32| c + direction * x + normal * y;
            let color = Color32::from_rgb(50, 50, 50);
            polygon(
                shapes,
                vec![at(-6.0, -1.8), at(0.5, -1.8), at(0.5, 1.8), at(-6.0, 1.8)],
                color,
                color,
            );
            polygon(
                shapes,
                vec![at(0.0, -5.0), at(6.0, 0.0), at(0.0, 5.0)],
                color,
                color,
            );
        }
        Icon::Screenshot => {
            let body = Color32::from_rgb(85, 85, 90);
            polygon(
                shapes,
                vec![p(5.0, 4.5), p(6.5, 2.5), p(10.5, 2.5), p(12.0, 4.5)],
                body,
                body,
            );
            shapes.push(Shape::rect_filled(
                Rect::from_min_max(p(1.0, 4.0), p(15.0, 14.0)),
                1.5,
                body,
            ));
            shapes.push(Shape::rect_filled(
                Rect::from_min_max(p(2.5, 5.5), p(4.5, 6.5)),
                0.0,
                Color32::from_rgb(255, 200, 60),
            ));
            shapes.push(Shape::circle_filled(p(8.5, 9.0), 3.8, Color32::WHITE));
            shapes.push(Shape::circle_filled(p(8.5, 9.0), 2.6, ACCENT));
            shapes.push(Shape::circle_filled(p(7.6, 8.1), 0.8, Color32::WHITE));
        }
        Icon::Sound => {
            let body = Color32::from_rgb(70, 70, 75);
            polygon(
                shapes,
                vec![
                    p(1.5, 6.0),
                    p(4.5, 6.0),
                    p(8.5, 2.5),
                    p(8.5, 13.5),
                    p(4.5, 10.0),
                    p(1.5, 10.0),
                ],
                body,
                body,
            );
            let c = p(8.5, 8.0);
            for radius in [3.0f32, 5.5] {
                let arc: Vec<Pos2> = (-4..=4)
                    .map(|i| {
                        let angle = i as f32 * 0.22;
                        c + vec2(angle.cos(), angle.sin()) * radius
                    })
                    .collect();
                line(shapes, &arc, 1.3, ACCENT);
            }
        }
        Icon::Transformation => {
            // The original filled, its mirror image as an outline.
            let shape = |s: f32| {
                vec![
                    p(8.0 - s * 1.5, 3.0),
                    p(8.0 - s * 6.5, 6.0),
                    p(8.0 - s * 6.5, 13.0),
                    p(8.0 - s * 1.5, 13.0),
                ]
            };
            polygon(shapes, shape(1.0), ACCENT, OUTLINE);
            // Reversed, so that the mirror image keeps the winding egui expects.
            let mirrored = shape(-1.0).into_iter().rev().collect();
            polygon(shapes, mirrored, FACE, OUTLINE);
            for y in [1.0f32, 4.5, 8.0, 11.5] {
                line(shapes, &[p(8.0, y), p(8.0, y + 2.0)], 1.2, OUTLINE);
            }
        }
        Icon::Remove => {
            let color = Color32::from_rgb(50, 50, 50);
            line(shapes, &[p(3.5, 3.5), p(12.5, 12.5)], 2.2, color);
            line(shapes, &[p(12.5, 3.5), p(3.5, 12.5)], 2.2, color);
        }
        Icon::Query => pipette(shapes, r),
        Icon::Solve => {
            let amber = Color32::from_rgb(250, 190, 40);
            let edge = Color32::from_rgb(150, 100, 0);
            // A bolt is not convex: two triangles, then its outline.
            polygon(
                shapes,
                vec![p(11.0, 1.0), p(3.5, 9.5), p(9.5, 8.0)],
                amber,
                amber,
            );
            polygon(
                shapes,
                vec![p(6.5, 8.0), p(12.5, 6.5), p(5.0, 15.0)],
                amber,
                amber,
            );
            shapes.push(Shape::closed_line(
                vec![
                    p(11.0, 1.0),
                    p(3.5, 9.5),
                    p(7.8, 8.4),
                    p(5.0, 15.0),
                    p(12.5, 6.5),
                    p(8.2, 7.6),
                ],
                Stroke::new(1.0, edge),
            ));
        }
        Icon::Monitor => {
            let window = Rect::from_min_max(p(1.5, 2.5), p(14.5, 13.5));
            shapes.push(Shape::rect_filled(window, 0.0, Color32::WHITE));
            shapes.push(Shape::rect_filled(
                Rect::from_min_max(p(1.5, 2.5), p(14.5, 5.0)),
                0.0,
                ACCENT,
            ));
            for (y, end) in [(7.0, 12.0), (9.0, 10.0), (11.0, 12.5)] {
                line(shapes, &[p(3.5, y), p(end, y)], 1.0, OUTLINE);
            }
            shapes.push(Shape::rect_stroke(
                window,
                0.0,
                Stroke::new(1.0, OUTLINE),
                egui::StrokeKind::Middle,
            ));
        }
        Icon::RemoveAnnotations => {
            // A query label: framed text with the arrow to its point.
            line(shapes, &[p(3.0, 9.5), p(1.0, 14.5)], 1.0, OUTLINE);
            polygon(
                shapes,
                vec![p(1.0, 14.8), p(0.6, 12.0), p(2.6, 12.8)],
                OUTLINE,
                OUTLINE,
            );
            let label = Rect::from_min_max(p(2.5, 1.5), p(14.5, 9.5));
            shapes.push(Shape::rect_filled(label, 0.0, Color32::WHITE));
            shapes.push(Shape::rect_stroke(
                label,
                0.0,
                Stroke::new(1.0, OUTLINE),
                egui::StrokeKind::Inside,
            ));
            for (y, end) in [(4.0, 12.0), (7.0, 9.5)] {
                line(shapes, &[p(4.5, y), p(end, y)], 1.0, OUTLINE);
            }
            let red = Color32::from_rgb(210, 40, 40);
            line(shapes, &[p(8.5, 8.5), p(14.5, 14.5)], 2.0, red);
            line(shapes, &[p(14.5, 8.5), p(8.5, 14.5)], 2.0, red);
        }
        Icon::First | Icon::Previous | Icon::Next | Icon::Last => {
            let forward = matches!(icon, Icon::Next | Icon::Last);
            let s = if forward { 1.0 } else { -1.0 };
            let c = r.center();
            let tri = |x0: f32| {
                vec![
                    c + vec2(s * (x0 - 3.5), -5.0),
                    c + vec2(s * (x0 + 3.5), 0.0),
                    c + vec2(s * (x0 - 3.5), 5.0),
                ]
            };
            let color = Color32::from_rgb(50, 50, 50);
            let double = matches!(icon, Icon::First | Icon::Last);
            if double {
                polygon(shapes, tri(-1.5), color, color);
                shapes.push(Shape::rect_filled(
                    Rect::from_center_size(c + vec2(s * 4.5, 0.0), vec2(2.0, 10.0)),
                    0.0,
                    color,
                ));
            } else {
                polygon(shapes, tri(0.0), color, color);
            }
        }
    }
}

/// The pipette of the query: a glass tube with blue liquid, a dark rubber bulb, its tip at
/// the lower left.
fn pipette(shapes: &mut Vec<Shape>, r: Rect) {
    // Along the pipette from its tip (0) to the end of the bulb (about 17), and across it.
    let tip = pos2(r.left() + 1.5, r.top() + 14.5);
    let along = vec2(1.0, -1.0).normalized();
    let across = along.rot90();
    let at = |t: f32, n: f32| tip + along * t + across * n;
    let quad =
        |t0: f32, w0: f32, t1: f32, w1: f32| vec![at(t0, -w0), at(t1, -w1), at(t1, w1), at(t0, w0)];
    let glass = Color32::from_rgb(225, 238, 250);
    let rubber = Color32::from_rgb(70, 70, 75);
    // The narrow tip, the tube, the liquid in its lower half, the collar and the bulb.
    polygon(shapes, quad(0.0, 0.5, 3.5, 1.4), ACCENT, OUTLINE);
    polygon(shapes, quad(3.5, 1.9, 11.0, 1.9), glass, OUTLINE);
    shapes.push(Shape::convex_polygon(
        quad(3.9, 1.4, 7.5, 1.4),
        ACCENT,
        Stroke::NONE,
    ));
    polygon(shapes, quad(11.0, 2.9, 12.5, 2.9), rubber, rubber);
    shapes.push(Shape::circle_filled(at(14.8, 0.0), 2.6, rubber));
}

/// Isometric unit cube; x to the lower right, z to the lower left, y up.
/// The cube of the view icons cut in half, its section face in the accent colour inside the
/// outline of the cutting plane.
fn section_cube(shapes: &mut Vec<Shape>, r: Rect) {
    let c = r.center() + vec2(0.0, 0.5);
    let s = 6.2 * 1.25;
    let project = |[x, y, z]: [f32; 3]| {
        let (x, y, z) = (x - 0.5, y - 0.5, z - 0.5);
        c + vec2((x - z) * 0.866 * s, ((x + z) * 0.5 - y) * s)
    };
    let cut = 0.55;
    let quad = |corners: [[f32; 3]; 4]| corners.map(project).to_vec();
    let plane_color = Color32::from_rgb(0, 120, 215);
    // The cutting plane, slightly larger than the cube.
    let plane = quad([
        [cut, -0.15, -0.15],
        [cut, 1.15, -0.15],
        [cut, 1.15, 1.15],
        [cut, -0.15, 1.15],
    ]);
    let mut outline = plane.clone();
    outline.push(plane[0]);
    line(shapes, &outline, 1.0, plane_color);
    let front = quad([
        [0.0, 0.0, 1.0],
        [cut, 0.0, 1.0],
        [cut, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ]);
    let top = quad([
        [0.0, 1.0, 0.0],
        [cut, 1.0, 0.0],
        [cut, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ]);
    let section = quad([
        [cut, 0.0, 0.0],
        [cut, 1.0, 0.0],
        [cut, 1.0, 1.0],
        [cut, 0.0, 1.0],
    ]);
    for (points, fill) in [(front, FACE), (top, FACE), (section, ACCENT)] {
        polygon(shapes, points.clone(), fill, Color32::TRANSPARENT);
        let mut closed = points.clone();
        closed.push(points[0]);
        line(shapes, &closed, 1.0, OUTLINE);
    }
}

/// A base plate with two blocks lifted off it along a blue arrow.
fn exploded_blocks(shapes: &mut Vec<Shape>, r: Rect) {
    let c = r.center();
    let s = 5.0;
    let project = |[x, y, z]: [f32; 3]| c + vec2((x - z) * 0.866 * s, ((x + z) * 0.5 - y) * s);
    // A box from `min` to `max` in the units of `s`, with its three visible faces.
    let block = |shapes: &mut Vec<Shape>, min: [f32; 3], max: [f32; 3], fill: Color32| {
        let [x0, y0, z0] = min;
        let [x1, y1, z1] = max;
        let faces = [
            [[x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]],
            [[x0, y1, z0], [x1, y1, z0], [x1, y1, z1], [x0, y1, z1]],
            [[x1, y0, z0], [x1, y1, z0], [x1, y1, z1], [x1, y0, z1]],
        ];
        for face in faces {
            let points = face.map(project).to_vec();
            polygon(shapes, points.clone(), fill, Color32::TRANSPARENT);
            let mut closed = points.clone();
            closed.push(points[0]);
            line(shapes, &closed, 1.0, OUTLINE);
        }
    };
    block(shapes, [-0.9, -1.6, -0.9], [0.9, -1.1, 0.9], FACE);
    block(shapes, [-0.5, -0.5, -0.5], [0.5, 0.0, 0.5], ACCENT);
    block(shapes, [-0.7, 0.8, -0.7], [0.7, 1.2, 0.7], FACE);
    // The arrow along which the blocks come off, beside the stack.
    let arrow_color = Color32::from_rgb(0, 120, 215);
    let (bottom, top) = (project([1.6, -0.9, -0.4]), project([1.6, 1.0, -0.4]));
    line(shapes, &[bottom, top + vec2(0.0, 2.0)], 1.2, arrow_color);
    polygon(
        shapes,
        vec![
            top + vec2(0.0, -1.0),
            top + vec2(2.2, 2.5),
            top + vec2(-2.2, 2.5),
        ],
        arrow_color,
        arrow_color,
    );
}

fn cube(shapes: &mut Vec<Shape>, r: Rect, view: Option<ViewIcon>, grid: bool) {
    let c = r.center() + vec2(0.0, 0.5);
    let s = 6.2;
    let project = |[x, y, z]: [f32; 3]| {
        // Centre the cube on the origin of the icon.
        let (x, y, z) = (x - 0.5, y - 0.5, z - 0.5);
        c + vec2((x - z) * 0.866 * s * 1.25, ((x + z) * 0.5 - y) * s * 1.25)
    };
    let corner = |i: usize| [(i & 1) as f32, ((i >> 1) & 1) as f32, ((i >> 2) & 1) as f32];
    // Faces as corner indices (bits: x = 1, y = 2, z = 4), visible ones last.
    let faces: [(ViewIcon, [usize; 4], bool); 6] = [
        (ViewIcon::Back, [0, 1, 3, 2], false),
        (ViewIcon::Bottom, [0, 1, 5, 4], false),
        (ViewIcon::Left, [0, 2, 6, 4], false),
        (ViewIcon::Front, [4, 5, 7, 6], true),
        (ViewIcon::Top, [2, 3, 7, 6], true),
        (ViewIcon::Right, [1, 3, 7, 5], true),
    ];
    // A highlighted back face is seen through a wireframe of the front faces.
    let see_through = faces
        .iter()
        .any(|(face, _, visible)| !visible && view == Some(*face));
    let hidden_edge = Color32::from_rgb(150, 150, 150);
    for (face, corners, visible) in faces {
        let points: Vec<Pos2> = corners.iter().map(|&i| project(corner(i))).collect();
        let highlighted = view == Some(face) || (view == Some(ViewIcon::Isometric) && visible);
        if highlighted {
            polygon(shapes, points.clone(), ACCENT, Color32::TRANSPARENT);
        } else if visible && !see_through {
            polygon(shapes, points.clone(), FACE, Color32::TRANSPARENT);
        }
        let mut closed = points.clone();
        closed.push(points[0]);
        if visible {
            line(shapes, &closed, 1.0, OUTLINE);
        } else if highlighted {
            line(shapes, &closed, 1.0, hidden_edge);
        }
        if grid && visible {
            for t in [1.0 / 3.0, 2.0 / 3.0] {
                let lerp = |a: Pos2, b: Pos2| a + (b - a) * t;
                line(
                    shapes,
                    &[lerp(points[0], points[1]), lerp(points[3], points[2])],
                    0.7,
                    OUTLINE,
                );
                line(
                    shapes,
                    &[lerp(points[0], points[3]), lerp(points[1], points[2])],
                    0.7,
                    OUTLINE,
                );
            }
        }
    }
}
