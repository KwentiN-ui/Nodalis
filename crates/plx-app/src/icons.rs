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
    /// The open folder with a small clock: the projects opened last.
    OpenRecent,
    /// Import of geometry: an arrow pointing into a sheet.
    Import,
    Save,
    Fit,
    /// Standard view; the cube face looked at is highlighted.
    View(ViewIcon),
    /// View down an axis: the face looked at straight on, with the letter of the axis.
    AxisView(crate::viewport::Axis),
    /// Isometric view with an axis up: a small cube with the letter of the axis above.
    IsometricAxis(crate::viewport::Axis),
    /// A quarter turn of the view of a 2D model: an arrow around a block, counterclockwise
    /// for `true`.
    Rotate(bool),
    /// The view of a 2D model with y up: an arrow pointing up with a Y.
    DefaultRotation,
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
    /// A face meshed with an even grid: the default mesh parameters.
    MeshDefaults,
    /// A coarse grid with one corner meshed finer: a local mesh size.
    /// Two parts side by side, one meshed finer: mesh parameters of single parts.
    MeshParameters,
    LocalMeshSize,
    /// A face mesh swept upward into layers of hexahedra: the extruded mesh.
    ExtrudeMesh,
    /// A tetrahedron with its hidden edge dashed: the tetrahedral mesher.
    TetMesh,
    /// A bent beam over its straight outline: the deformation of the results.
    Deformation,
    /// The dashed straight outline behind a bent beam: show the undeformed shape.
    Undeformed,
    /// A thin outline around a small coloured block: the global model of a submodel.
    GlobalContours,
    /// A face in colour bands: contours, arrows or streaks.
    Contours,
    /// A legend: a column of colour bands.
    Levels,
    /// A spectrum with its two limit marks: the colour scale.
    ColorScale,
    /// A bar of ticks with a marker on one: the increments of a step.
    Increments,
    /// A bar of three segments with a marker in the middle one: the steps of the results.
    Steps,
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
        Icon::AxisView(axis) => {
            polygon(
                shapes,
                vec![p(2.5, 2.5), p(13.5, 2.5), p(13.5, 13.5), p(2.5, 13.5)],
                ACCENT,
                OUTLINE,
            );
            letter(
                shapes,
                axis,
                Rect::from_min_size(p(5.0, 5.0), vec2(6.0, 6.0)),
                1.6,
                Color32::WHITE,
            );
        }
        Icon::IsometricAxis(axis) => {
            let area = Rect::from_min_size(p(-1.5, 3.5), Vec2::splat(13.0));
            cube_scaled(shapes, area, Some(ViewIcon::Top), false, 0.78);
            letter(
                shapes,
                axis,
                Rect::from_min_size(p(10.0, 0.5), vec2(5.0, 5.5)),
                1.4,
                axis_color(axis),
            );
        }
        Icon::Rotate(left) => {
            let x = |x: f32| if left { x } else { 16.0 - x };
            let q = |px: f32, py: f32| p(x(px), py);
            polygon(
                shapes,
                vec![q(5.5, 7.0), q(10.5, 7.0), q(10.5, 11.0), q(5.5, 11.0)],
                FACE,
                OUTLINE,
            );
            // An arc around the block, counterclockwise on the screen, ending at the top
            // with its head pointing left.
            let (c, radius) = ((8.0, 9.0), 5.5);
            let at = |degrees: f32| {
                let a = degrees.to_radians();
                q(c.0 + radius * a.cos(), c.1 - radius * a.sin())
            };
            let arc: Vec<Pos2> = (0..=20).map(|i| at(165.0 + 13.75 * i as f32)).collect();
            line(shapes, &arc, 1.6, ACCENT);
            // The head goes on along the arc from its end.
            let end = at(440.0);
            let along = (at(441.0) - end).normalized();
            let tip = end + along * 4.0;
            let side = along.rot90();
            polygon(
                shapes,
                vec![tip, end - side * 2.8, end + side * 2.8],
                ACCENT,
                ACCENT,
            );
        }
        Icon::DefaultRotation => {
            line(shapes, &[p(6.0, 14.5), p(6.0, 5.0)], 2.0, ACCENT);
            polygon(
                shapes,
                vec![p(6.0, 1.0), p(9.5, 6.0), p(2.5, 6.0)],
                ACCENT,
                ACCENT,
            );
            letter(
                shapes,
                crate::viewport::Axis::Y,
                Rect::from_min_size(p(9.5, 8.0), vec2(5.0, 6.0)),
                1.4,
                axis_color(crate::viewport::Axis::Y),
            );
        }
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
        Icon::Open | Icon::OpenRecent => {
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
            if icon == Icon::OpenRecent {
                let centre = p(11.5, 11.0);
                shapes.push(Shape::circle_filled(centre, 4.6, Color32::WHITE));
                shapes.push(Shape::circle_stroke(centre, 4.6, Stroke::new(1.2, ACCENT)));
                line(shapes, &[p(11.5, 8.2), centre, p(13.4, 12.0)], 1.2, OUTLINE);
            }
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
        Icon::MeshDefaults => {
            let face = Rect::from_min_max(p(2.5, 2.5), p(13.5, 13.5));
            shapes.push(Shape::rect_filled(face, 0.0, FACE));
            for i in 1..4 {
                let t = 2.5 + 11.0 * i as f32 / 4.0;
                line(shapes, &[p(t, 2.5), p(t, 13.5)], 1.0, ACCENT);
                line(shapes, &[p(2.5, t), p(13.5, t)], 1.0, ACCENT);
            }
            shapes.push(Shape::rect_stroke(
                face,
                0.0,
                Stroke::new(1.0, OUTLINE),
                egui::StrokeKind::Middle,
            ));
        }
        Icon::MeshParameters => {
            // Two parts: the left one meshed coarse, the right one fine.
            let left = Rect::from_min_max(p(1.5, 3.5), p(7.0, 12.5));
            let right = Rect::from_min_max(p(9.0, 3.5), p(14.5, 12.5));
            shapes.push(Shape::rect_filled(left, 0.0, FACE));
            shapes.push(Shape::rect_filled(right, 0.0, FACE));
            line(shapes, &[p(4.25, 3.5), p(4.25, 12.5)], 1.0, ACCENT);
            line(shapes, &[p(1.5, 8.0), p(7.0, 8.0)], 1.0, ACCENT);
            for t in [10.4, 11.75, 13.1] {
                line(shapes, &[p(t, 3.5), p(t, 12.5)], 1.0, ACCENT);
            }
            for t in [5.75, 8.0, 10.25] {
                line(shapes, &[p(9.0, t), p(14.5, t)], 1.0, ACCENT);
            }
            for r in [left, right] {
                shapes.push(Shape::rect_stroke(
                    r,
                    0.0,
                    Stroke::new(1.0, OUTLINE),
                    egui::StrokeKind::Middle,
                ));
            }
        }
        Icon::LocalMeshSize => {
            let face = Rect::from_min_max(p(2.5, 2.5), p(13.5, 13.5));
            shapes.push(Shape::rect_filled(face, 0.0, FACE));
            // The finer corner at the top right.
            shapes.push(Shape::rect_filled(
                Rect::from_min_max(p(8.0, 2.5), p(13.5, 8.0)),
                0.0,
                Color32::from_rgb(200, 222, 245),
            ));
            line(shapes, &[p(8.0, 2.5), p(8.0, 13.5)], 1.0, OUTLINE);
            line(shapes, &[p(2.5, 8.0), p(13.5, 8.0)], 1.0, OUTLINE);
            for t in [5.25, 10.75] {
                line(shapes, &[p(t, 2.5), p(t, 8.0)], 1.0, ACCENT);
                line(shapes, &[p(8.0, t), p(13.5, t)], 1.0, ACCENT);
            }
            for t in [9.375, 12.125] {
                line(shapes, &[p(t, 2.5), p(t, 8.0)], 1.0, ACCENT);
            }
            for t in [3.875, 6.625] {
                line(shapes, &[p(8.0, t), p(13.5, t)], 1.0, ACCENT);
            }
            shapes.push(Shape::rect_stroke(
                face,
                0.0,
                Stroke::new(1.0, OUTLINE),
                egui::StrokeKind::Middle,
            ));
        }
        Icon::ExtrudeMesh => {
            // Three layers of a prism over a slanted face, the face in the accent colour.
            let (dx, dy) = (4.0, -3.0);
            let base = [p(1.5, 11.0), p(9.0, 11.0), p(9.0 + dx, 11.0 + dy)];
            let at = |q: Pos2, h: f32| q + vec2(0.0, -h);
            let height = 7.5;
            // The visible side faces.
            polygon(
                shapes,
                vec![base[0], base[1], at(base[1], height), at(base[0], height)],
                FACE,
                Color32::TRANSPARENT,
            );
            polygon(
                shapes,
                vec![base[1], base[2], at(base[2], height), at(base[1], height)],
                Color32::from_rgb(205, 205, 205),
                Color32::TRANSPARENT,
            );
            // The top: the face mesh that is swept.
            let top = [
                at(base[0], height),
                at(base[1], height),
                at(base[2], height),
            ];
            let fourth = top[0] + vec2(dx, dy);
            polygon(
                shapes,
                vec![top[0], top[1], top[2], fourth],
                ACCENT,
                Color32::TRANSPARENT,
            );
            let mid = |a: Pos2, b: Pos2| a + (b - a) * 0.5;
            line(
                shapes,
                &[mid(top[0], top[1]), mid(fourth, top[2])],
                1.0,
                Color32::WHITE,
            );
            line(
                shapes,
                &[mid(top[0], fourth), mid(top[1], top[2])],
                1.0,
                Color32::WHITE,
            );
            // The layers.
            for h in [height / 3.0, 2.0 * height / 3.0] {
                line(
                    shapes,
                    &[at(base[0], h), at(base[1], h), at(base[2], h)],
                    1.0,
                    OUTLINE,
                );
            }
            // The outline.
            line(
                shapes,
                &[
                    top[0], base[0], base[1], base[2], top[2], fourth, top[0], top[1], top[2],
                ],
                1.0,
                OUTLINE,
            );
            line(shapes, &[base[1], top[1]], 1.0, OUTLINE);
        }
        Icon::TetMesh => {
            let (a, b, c, d) = (p(1.5, 13.0), p(10.5, 14.5), p(14.5, 9.0), p(7.0, 1.5));
            polygon(shapes, vec![a, b, d], FACE, Color32::TRANSPARENT);
            polygon(shapes, vec![b, c, d], ACCENT, Color32::TRANSPARENT);
            // The hidden edge at the back, dashed.
            shapes.extend(Shape::dashed_line(
                &[a, c],
                Stroke::new(1.0, OUTLINE),
                1.5,
                1.5,
            ));
            line(shapes, &[a, b, c, d, a], 1.0, OUTLINE);
            line(shapes, &[b, d], 1.0, OUTLINE);
        }
        Icon::Deformation | Icon::Undeformed => {
            // The straight outline, dashed for Undeformed, and the beam bent down.
            let straight = [
                p(1.5, 4.5),
                p(14.5, 4.5),
                p(14.5, 8.0),
                p(1.5, 8.0),
                p(1.5, 4.5),
            ];
            if icon == Icon::Undeformed {
                for pair in straight.windows(2) {
                    shapes.extend(Shape::dashed_line(
                        pair,
                        Stroke::new(1.0, OUTLINE),
                        1.5,
                        1.5,
                    ));
                }
            }
            let bend = |x: f32| 0.035 * (x - 1.5) * (x - 1.5);
            let xs: Vec<f32> = (0..=8).map(|i| 1.5 + 13.0 * i as f32 / 8.0).collect();
            let top: Vec<Pos2> = xs.iter().map(|&x| p(x, 4.5 + bend(x))).collect();
            let bottom: Vec<Pos2> = xs.iter().rev().map(|&x| p(x, 8.0 + bend(x))).collect();
            let fill = if icon == Icon::Undeformed {
                FACE
            } else {
                ACCENT
            };
            for i in 0..xs.len() - 1 {
                let j = xs.len() - 1 - i;
                polygon(
                    shapes,
                    vec![top[i], top[i + 1], bottom[j - 1], bottom[j]],
                    fill,
                    fill,
                );
            }
            let mut outline = top.clone();
            outline.extend(bottom);
            outline.push(top[0]);
            line(shapes, &outline, 1.0, OUTLINE);
            if icon == Icon::Deformation {
                // The arrow of the displacement at the free end.
                let red = Color32::from_rgb(210, 60, 40);
                line(shapes, &[p(12.5, 1.0), p(12.5, 6.0)], 1.2, red);
                polygon(
                    shapes,
                    vec![p(12.5, 8.0), p(10.8, 5.5), p(14.2, 5.5)],
                    red,
                    red,
                );
            }
        }
        Icon::GlobalContours => {
            let outer = Rect::from_min_max(p(1.5, 1.5), p(14.5, 14.5));
            shapes.extend(Shape::dashed_line(
                &[
                    outer.left_top(),
                    outer.right_top(),
                    outer.right_bottom(),
                    outer.left_bottom(),
                    outer.left_top(),
                ],
                Stroke::new(1.0, OUTLINE),
                2.0,
                1.5,
            ));
            bands(shapes, Rect::from_min_max(p(5.0, 5.0), p(11.0, 11.0)), true);
        }
        Icon::Contours => bands(shapes, Rect::from_min_max(p(1.5, 2.5), p(14.5, 13.5)), true),
        Icon::Levels => {
            bands(shapes, Rect::from_min_max(p(2.5, 1.5), p(7.5, 14.5)), false);
            for i in 0..5 {
                let y = 2.5 + 2.75 * i as f32;
                line(shapes, &[p(9.0, y), p(13.5, y)], 1.0, OUTLINE);
            }
        }
        Icon::ColorScale => {
            let bar = Rect::from_min_max(p(1.5, 4.5), p(14.5, 10.5));
            let colors = spectrum();
            let n = colors.len();
            for (i, color) in colors.into_iter().enumerate() {
                let x0 = bar.left() + bar.width() * i as f32 / n as f32;
                let x1 = bar.left() + bar.width() * (i + 1) as f32 / n as f32;
                shapes.push(Shape::rect_filled(
                    Rect::from_min_max(pos2(x0, bar.top()), pos2(x1, bar.bottom())),
                    0.0,
                    color,
                ));
            }
            shapes.push(Shape::rect_stroke(
                bar,
                0.0,
                Stroke::new(1.0, OUTLINE),
                egui::StrokeKind::Middle,
            ));
            // The two limits, as marks below the bar.
            for x in [4.0, 12.0] {
                polygon(
                    shapes,
                    vec![p(x, 11.0), p(x + 2.0, 14.5), p(x - 2.0, 14.5)],
                    OUTLINE,
                    OUTLINE,
                );
            }
        }
        Icon::Increments => {
            let bar = Rect::from_min_max(p(0.5, 6.0), p(15.5, 10.0));
            shapes.push(Shape::rect_filled(bar, 0.0, FACE));
            shapes.push(Shape::rect_stroke(
                bar,
                0.0,
                Stroke::new(1.0, OUTLINE),
                egui::StrokeKind::Middle,
            ));
            for x in [3.0, 5.5, 8.0, 13.0] {
                line(shapes, &[p(x, 6.0), p(x, 10.0)], 1.0, OUTLINE);
            }
            let marker = Rect::from_min_max(p(9.6, 3.0), p(11.6, 13.0));
            shapes.push(Shape::rect_filled(marker, 0.0, ACCENT));
            shapes.push(Shape::rect_stroke(
                marker,
                0.0,
                Stroke::new(1.0, OUTLINE),
                egui::StrokeKind::Middle,
            ));
        }
        Icon::Steps => {
            for (i, (x0, x1)) in [(0.5, 4.8), (5.8, 10.2), (11.2, 15.5)]
                .into_iter()
                .enumerate()
            {
                let segment = Rect::from_min_max(p(x0, 5.5), p(x1, 10.5));
                let fill = if i == 1 { ACCENT } else { FACE };
                shapes.push(Shape::rect_filled(segment, 1.0, fill));
                shapes.push(Shape::rect_stroke(
                    segment,
                    1.0,
                    Stroke::new(1.0, OUTLINE),
                    egui::StrokeKind::Middle,
                ));
            }
            line(shapes, &[p(8.0, 2.5), p(8.0, 13.5)], 1.5, OUTLINE);
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

/// The colours of a contour from low to high.
fn spectrum() -> Vec<Color32> {
    [
        (40, 60, 230),
        (40, 160, 240),
        (40, 210, 160),
        (120, 220, 40),
        (240, 220, 40),
        (245, 140, 30),
        (225, 45, 35),
    ]
    .into_iter()
    .map(|(r, g, b)| Color32::from_rgb(r, g, b))
    .collect()
}

/// `rect` in colour bands, side by side when `across`, else from the bottom up.
fn bands(shapes: &mut Vec<Shape>, rect: Rect, across: bool) {
    let colors = spectrum();
    let n = colors.len() as f32;
    for (i, color) in colors.into_iter().enumerate() {
        let (a, b) = (i as f32 / n, (i + 1) as f32 / n);
        let band = if across {
            Rect::from_min_max(
                pos2(rect.left() + rect.width() * a, rect.top()),
                pos2(rect.left() + rect.width() * b, rect.bottom()),
            )
        } else {
            Rect::from_min_max(
                pos2(rect.left(), rect.bottom() - rect.height() * b),
                pos2(rect.right(), rect.bottom() - rect.height() * a),
            )
        };
        shapes.push(Shape::rect_filled(band, 0.0, color));
    }
    shapes.push(Shape::rect_stroke(
        rect,
        0.0,
        Stroke::new(1.0, OUTLINE),
        egui::StrokeKind::Middle,
    ));
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

/// The colour of an axis, as in the triad of the 3D view.
fn axis_color(axis: crate::viewport::Axis) -> Color32 {
    match axis {
        crate::viewport::Axis::X => Color32::from_rgb(220, 40, 40),
        crate::viewport::Axis::Y => Color32::from_rgb(20, 160, 20),
        crate::viewport::Axis::Z => Color32::from_rgb(40, 80, 230),
    }
}

/// The letter of an axis drawn with strokes into `r`.
fn letter(
    shapes: &mut Vec<Shape>,
    axis: crate::viewport::Axis,
    r: Rect,
    width: f32,
    color: Color32,
) {
    let (a, b, c, d) = (
        r.left_top(),
        r.right_top(),
        r.left_bottom(),
        r.right_bottom(),
    );
    let middle = r.center();
    match axis {
        crate::viewport::Axis::X => {
            line(shapes, &[a, d], width, color);
            line(shapes, &[b, c], width, color);
        }
        crate::viewport::Axis::Y => {
            line(shapes, &[a, middle, b], width, color);
            line(shapes, &[middle, pos2(middle.x, r.bottom())], width, color);
        }
        crate::viewport::Axis::Z => line(shapes, &[a, b, c, d], width, color),
    }
}

fn cube(shapes: &mut Vec<Shape>, r: Rect, view: Option<ViewIcon>, grid: bool) {
    cube_scaled(shapes, r, view, grid, 1.0);
}

/// The cube of [`cube`], `scale` times as large.
fn cube_scaled(shapes: &mut Vec<Shape>, r: Rect, view: Option<ViewIcon>, grid: bool, scale: f32) {
    let c = r.center() + vec2(0.0, 0.5);
    let s = 6.2 * scale;
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
