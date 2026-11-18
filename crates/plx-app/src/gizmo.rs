//! The plane manipulator of the section view, in the spirit of Onshape's: the plane drawn
//! translucent, an arrow along its normal that shifts it, and two arcs that tilt it around
//! the axes in the plane. Everything is drawn over the scene with the egui painter.

use egui::{Color32, Painter, Pos2, Rect, Shape, Stroke};
use glam::Vec3;
use plx_render::Camera;

use crate::overlay;

/// Screen length of the arrow and radius of the arcs, in points.
const ARROW_LENGTH: f32 = 90.0;
const ARC_RADIUS: f32 = 70.0;
/// How close the pointer has to come to a handle to grab it.
const GRAB_DISTANCE: f32 = 8.0;
/// The arcs run between these angles from the normal towards the in-plane axis.
const ARC_ANGLES_DEG: (f32, f32) = (20.0, 70.0);
const ARC_SEGMENTS: usize = 16;

const PLANE_FILL: Color32 = Color32::from_rgba_premultiplied(0, 10, 17, 20);
const PLANE_BORDER: Color32 = Color32::from_rgb(0, 120, 215);
const ARROW_COLOR: Color32 = Color32::from_rgb(20, 50, 170);
/// Drawn under the handles so they stand out on any contour colour.
const HALO: Color32 = Color32::from_rgba_premultiplied(255, 255, 255, 255);
const ARC_COLORS: [Color32; 2] = [
    Color32::from_rgb(220, 60, 40),
    Color32::from_rgb(40, 160, 60),
];
const ACTIVE_COLOR: Color32 = Color32::from_rgb(255, 175, 0);

/// The plane being edited, in render coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaneGizmo {
    pub point: Vec3,
    /// Unit normal; the arrow points along it.
    pub normal: Vec3,
    /// Half the edge length of the drawn plane, in world units; none drawn when zero.
    pub half_size: f32,
    /// The axes in the plane the arcs turn it around, one arc each for as many as the count
    /// says; two of any orthogonal axes in the plane when `None`.
    pub axes: Option<([Vec3; 2], usize)>,
}

/// How a drag on the manipulator moves the plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GizmoDrag {
    /// Shift along the normal by this distance.
    Translate(f32),
    /// Turn around an axis in the plane through its point, in radians.
    Rotate { axis: Vec3, angle: f32 },
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Handle {
    Arrow,
    /// Rotation around the first or second in-plane axis.
    Arc(usize),
}

/// Which handle the mouse is over or drags.
#[derive(Clone, Copy, Debug, Default)]
pub struct GizmoState {
    hovered: Option<Handle>,
    /// The dragged handle with its rotation axis, fixed while the drag lasts.
    active: Option<(Handle, Vec3)>,
}

impl GizmoState {
    /// Whether the left button drags a handle, so the view must not react to it.
    pub fn dragging(&self) -> bool {
        self.active.is_some()
    }

    /// Whether a click would hit a handle rather than the model.
    pub fn hovered(&self) -> bool {
        self.hovered.is_some()
    }

    /// Grabs, drags and releases handles; returns how the plane moves this frame.
    pub fn interact(
        &mut self,
        gizmo: Option<&PlaneGizmo>,
        camera: &Camera,
        rect: Rect,
        response: &egui::Response,
    ) -> Option<GizmoDrag> {
        let Some(gizmo) = gizmo else {
            *self = Self::default();
            return None;
        };
        let shapes = Shapes::new(gizmo, camera, rect);
        let pointer = response.hover_pos().or(response.interact_pointer_pos());
        if self.active.is_none() {
            self.hovered = pointer.and_then(|p| shapes.handle_at(p));
        }
        // A drag only starts once the mouse moved a little; the handle is the one pressed on.
        let pressed = response.ctx.input(|i| i.pointer.press_origin());
        if response.drag_started_by(egui::PointerButton::Primary)
            && let Some(handle) = pressed.and_then(|p| shapes.handle_at(p))
        {
            let axis = match handle {
                Handle::Arrow => gizmo.normal,
                Handle::Arc(k) => shapes.axes[k],
            };
            self.active = Some((handle, axis));
        }
        let (handle, axis) = self.active?;
        if !response.dragged_by(egui::PointerButton::Primary) {
            self.active = None;
            return None;
        }
        let delta = response.drag_delta();
        if delta == egui::Vec2::ZERO {
            return None;
        }
        let pixel = camera.pixel_size(rect.width(), rect.height());
        let to_screen = |p: Vec3| overlay::project(camera, rect, p);
        match handle {
            Handle::Arrow => {
                // Follow the mouse along the arrow as it appears on screen.
                let length = ARROW_LENGTH * pixel;
                let on_screen = to_screen(gizmo.point + gizmo.normal * length) - shapes.center;
                let distance = if on_screen.length() > 4.0 {
                    delta.dot(on_screen) / on_screen.length_sq() * length
                } else {
                    // Looking along the normal: dragging up moves the plane towards the viewer.
                    -delta.y * pixel
                };
                Some(GizmoDrag::Translate(distance))
            }
            Handle::Arc(_) => {
                // Moving the mouse along the arc turns it; the tangent of the arc where it is
                // grabbed gives the direction on screen.
                let radius = ARC_RADIUS * pixel;
                let middle = 0.5 * (ARC_ANGLES_DEG.0 + ARC_ANGLES_DEG.1).to_radians();
                let side = axis.cross(gizmo.normal);
                let grab =
                    gizmo.point + (gizmo.normal * middle.cos() + side * middle.sin()) * radius;
                let tangent = axis.cross(grab - gizmo.point).normalize_or_zero();
                let step = radius * 0.1;
                let on_screen = to_screen(grab + tangent * step) - to_screen(grab);
                if on_screen.length() < 0.5 {
                    return None;
                }
                let along = delta.dot(on_screen) / on_screen.length_sq() * step;
                Some(GizmoDrag::Rotate {
                    axis,
                    angle: along / radius,
                })
            }
        }
    }

    pub fn draw(&self, painter: &Painter, gizmo: &PlaneGizmo, camera: &Camera, rect: Rect) {
        let shapes = Shapes::new(gizmo, camera, rect);
        if gizmo.half_size > 0.0 {
            painter.add(Shape::convex_polygon(
                shapes.plane.to_vec(),
                PLANE_FILL,
                Stroke::new(1.5, PLANE_BORDER),
            ));
        }
        let highlight = self.active.map(|(h, _)| h).or(self.hovered);
        let style = |handle: Handle, color: Color32| {
            if highlight == Some(handle) {
                Stroke::new(4.0, ACTIVE_COLOR)
            } else {
                Stroke::new(3.0, color)
            }
        };
        let halo = |stroke: Stroke| Stroke::new(stroke.width + 3.0, HALO);
        for (k, arc) in shapes.arcs.iter().enumerate() {
            let stroke = style(Handle::Arc(k), ARC_COLORS[k % ARC_COLORS.len()]);
            painter.add(Shape::line(arc.clone(), halo(stroke)));
            painter.add(Shape::line(arc.clone(), stroke));
            if let Some(&end) = arc.last() {
                painter.circle(end, 4.0, stroke.color, Stroke::new(1.5, HALO));
            }
        }
        let arrow = style(Handle::Arrow, crate::style::lift(ARROW_COLOR));
        let [base, tip] = shapes.arrow;
        painter.line_segment([base, tip], halo(arrow));
        painter.line_segment([base, tip], arrow);
        let direction = (tip - base).normalized();
        if direction.length() > 0.5 {
            let side = direction.rot90() * 6.0;
            let back = tip - direction * 14.0;
            painter.add(Shape::convex_polygon(
                vec![tip + direction * 2.0, back + side, back - side],
                arrow.color,
                Stroke::new(1.5, HALO),
            ));
        }
        painter.circle_filled(base, 4.0, PLANE_BORDER);
    }
}

/// The manipulator projected onto the screen.
struct Shapes {
    center: Pos2,
    plane: [Pos2; 4],
    arrow: [Pos2; 2],
    /// Axes in the plane the arcs turn around.
    axes: Vec<Vec3>,
    arcs: Vec<Vec<Pos2>>,
}

impl Shapes {
    fn new(gizmo: &PlaneGizmo, camera: &Camera, rect: Rect) -> Self {
        let to_screen = |p: Vec3| overlay::project(camera, rect, p);
        let pixel = camera.pixel_size(rect.width(), rect.height());
        let n = gizmo.normal;
        let given = gizmo.axes.map(|(a, _)| a[0].normalize_or_zero());
        let u = given
            .filter(|u| u.length_squared() > 0.5)
            .unwrap_or_else(|| n.any_orthonormal_vector());
        let v = n.cross(u);
        let (p, h) = (gizmo.point, gizmo.half_size);
        let plane = [
            p + (u + v) * h,
            p + (v - u) * h,
            p - (u + v) * h,
            p + (u - v) * h,
        ]
        .map(to_screen);
        let arrow = [p, p + n * ARROW_LENGTH * pixel].map(to_screen);
        let axes = match gizmo.axes {
            Some((axes, count)) => axes[..count.min(2)].to_vec(),
            None => vec![u, v],
        };
        let radius = ARC_RADIUS * pixel;
        let arcs = (axes.iter()).map(|&axis| {
            let side = axis.cross(n);
            let (from, to) = (ARC_ANGLES_DEG.0.to_radians(), ARC_ANGLES_DEG.1.to_radians());
            (0..=ARC_SEGMENTS)
                .map(|i| {
                    let a = from + (to - from) * i as f32 / ARC_SEGMENTS as f32;
                    to_screen(p + (n * a.cos() + side * a.sin()) * radius)
                })
                .collect()
        });
        let arcs = arcs.collect();
        Self {
            center: to_screen(p),
            plane,
            arrow,
            axes,
            arcs,
        }
    }

    fn handle_at(&self, pointer: Pos2) -> Option<Handle> {
        let near = |points: &[Pos2]| {
            points
                .windows(2)
                .map(|s| segment_distance(pointer, s[0], s[1]))
                .fold(f32::INFINITY, f32::min)
        };
        let arcs = (self.arcs.iter().enumerate()).map(|(k, arc)| (Handle::Arc(k), near(arc)));
        std::iter::once((Handle::Arrow, near(&self.arrow)))
            .chain(arcs)
            .filter(|(_, d)| *d <= GRAB_DISTANCE)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(h, _)| h)
    }
}

fn segment_distance(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let t = if ab.length_sq() > 0.0 {
        ((p - a).dot(ab) / ab.length_sq()).clamp(0.0, 1.0)
    } else {
        0.0
    };
    p.distance(a + ab * t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::pos2;

    fn setup() -> (PlaneGizmo, Camera, Rect) {
        let camera = Camera::default();
        let gizmo = PlaneGizmo {
            point: Vec3::ZERO,
            normal: Vec3::X,
            half_size: 0.5,
            axes: None,
        };
        (
            gizmo,
            camera,
            Rect::from_min_size(Pos2::ZERO, egui::vec2(800.0, 600.0)),
        )
    }

    #[test]
    fn arrow_tip_and_arcs_are_grabbed() {
        let (gizmo, camera, rect) = setup();
        let shapes = Shapes::new(&gizmo, &camera, rect);
        assert_eq!(shapes.handle_at(shapes.arrow[1]), Some(Handle::Arrow));
        for k in 0..2 {
            let middle = shapes.arcs[k][ARC_SEGMENTS / 2];
            assert_eq!(shapes.handle_at(middle), Some(Handle::Arc(k)));
        }
        assert_eq!(shapes.handle_at(pos2(5.0, 5.0)), None);
    }

    #[test]
    fn handles_lie_on_the_plane_axes() {
        let (gizmo, camera, rect) = setup();
        let shapes = Shapes::new(&gizmo, &camera, rect);
        for axis in shapes.axes {
            assert!(axis.dot(gizmo.normal).abs() < 1e-6);
            assert!((axis.length() - 1.0).abs() < 1e-6);
        }
        assert!(shapes.center.distance(shapes.arrow[0]) < 1e-3);
    }
}
