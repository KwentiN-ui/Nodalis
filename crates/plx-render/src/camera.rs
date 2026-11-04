use glam::camera::rh::{proj::directx, view::look_at_mat4};
use glam::{Mat3, Mat4, Quat, Vec3};

/// Viewing directions of the view toolbar: the eye looks from the named side at the target.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StandardView {
    Front,
    Back,
    Top,
    Bottom,
    Left,
    Right,
    Isometric,
}

/// Orthographic camera orbiting a target point, as used in CAE viewports.
///
/// `rotation` maps camera space (x right, y up, looking along -z) to world space.
#[derive(Clone, Debug, PartialEq)]
pub struct Camera {
    pub target: Vec3,
    pub distance: f32,
    pub rotation: Quat,
    pub scene_radius: f32,
}

const ORBIT_RADIANS_PER_PIXEL: f32 = 0.008;
const MIN_HALF_EXTENT: f32 = 1e-6;

impl Default for Camera {
    fn default() -> Self {
        let mut camera = Self {
            target: Vec3::ZERO,
            distance: 1.0,
            rotation: Quat::IDENTITY,
            scene_radius: 1.0,
        };
        camera.set_isometric();
        camera
    }
}

impl Camera {
    pub fn eye(&self) -> Vec3 {
        self.target + self.rotation * Vec3::Z * self.distance
    }

    pub fn forward(&self) -> Vec3 {
        self.rotation * Vec3::NEG_Z
    }

    pub fn right(&self) -> Vec3 {
        self.rotation * Vec3::X
    }

    pub fn up(&self) -> Vec3 {
        self.rotation * Vec3::Y
    }

    /// Half of the visible extent along the shorter viewport side, in world units.
    pub fn half_extent(&self) -> f32 {
        (self.distance * 0.5).max(MIN_HALF_EXTENT)
    }

    /// Directions towards the three camera lights in world space. The lights sit at
    /// (-1, 1, 1), (1, 1, 1) and (0, -1, 0) in camera space and shine at the focal point.
    pub fn light_directions(&self) -> [Vec3; 3] {
        [
            Vec3::new(-1.0, 1.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(0.0, -1.0, 0.0),
        ]
        .map(|p| self.rotation * p.normalize())
    }

    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        let aspect = aspect.max(1e-6);
        let (half_width, half_height) = if aspect >= 1.0 {
            (self.half_extent() * aspect, self.half_extent())
        } else {
            (self.half_extent(), self.half_extent() / aspect)
        };
        let depth = self.depth_range() * 0.5;
        let view = look_at_mat4(self.eye(), self.target, self.up());
        let proj = directx::orthographic(
            -half_width,
            half_width,
            -half_height,
            half_height,
            self.distance - depth,
            self.distance + depth,
        );
        proj * view
    }

    /// Distance between the near and far clipping planes in world units.
    pub fn depth_range(&self) -> f32 {
        self.scene_radius.max(self.distance) * 8.0
    }

    /// World size of one pixel for a viewport of the given size.
    pub fn pixel_size(&self, viewport_width_px: f32, viewport_height_px: f32) -> f32 {
        2.0 * self.half_extent() / viewport_width_px.min(viewport_height_px).max(1.0)
    }

    /// Rotates the view around the target by a mouse drag in pixels.
    pub fn orbit(&mut self, dx_px: f32, dy_px: f32) {
        let yaw = Quat::from_axis_angle(self.up(), -dx_px * ORBIT_RADIANS_PER_PIXEL);
        let pitch = Quat::from_axis_angle(self.right(), -dy_px * ORBIT_RADIANS_PER_PIXEL);
        self.rotation = (yaw * pitch * self.rotation).normalize();
    }

    /// Rotates the view like [`Camera::orbit`], but about `pivot`: the pivot keeps its place
    /// on the screen, so a detail far from the target stays in view.
    pub fn orbit_about(&mut self, pivot: Vec3, dx_px: f32, dy_px: f32) {
        let before = self.rotation;
        self.orbit(dx_px, dy_px);
        let turn = self.rotation * before.inverse();
        self.target = pivot + turn * (self.target - pivot);
    }

    /// Moves the target so that the scene follows a mouse drag in pixels.
    pub fn pan(&mut self, dx_px: f32, dy_px: f32, viewport_width_px: f32, viewport_height_px: f32) {
        let world_per_pixel = self.pixel_size(viewport_width_px, viewport_height_px);
        self.target -= self.right() * dx_px * world_per_pixel;
        self.target += self.up() * dy_px * world_per_pixel;
    }

    /// Scales the visible region; factors below 1 zoom in.
    pub fn zoom(&mut self, factor: f32) {
        self.distance =
            (self.distance * factor).clamp(self.scene_radius * 1e-4, self.scene_radius * 1e3);
    }

    /// Zooms like [`Camera::zoom`] while the point under the mouse stays in place; the offset is
    /// measured from the viewport centre in pixels, y downwards.
    pub fn zoom_at(
        &mut self,
        factor: f32,
        offset_x_px: f32,
        offset_y_px: f32,
        viewport_width_px: f32,
        viewport_height_px: f32,
    ) {
        let world_per_pixel = self.pixel_size(viewport_width_px, viewport_height_px);
        let point =
            self.target + (self.right() * offset_x_px - self.up() * offset_y_px) * world_per_pixel;
        let before = self.distance;
        self.zoom(factor);
        self.target = point + (self.target - point) * (self.distance / before);
    }

    /// Centers the view on a bounding box and makes it fill the viewport.
    pub fn fit(&mut self, min: Vec3, max: Vec3) {
        self.target = (min + max) * 0.5;
        self.scene_radius = ((max - min).length() * 0.5).max(MIN_HALF_EXTENT);
        self.distance = self.scene_radius * 2.2;
    }

    pub fn set_isometric(&mut self) {
        self.set_view(StandardView::Isometric);
    }

    /// Looks at the target from one of the standard directions. The isometric view keeps
    /// the global axis that is closest to up on the screen pointing up, so a
    /// model standing on Z is not laid on its side.
    pub fn set_view(&mut self, view: StandardView) {
        use std::f32::consts::{FRAC_PI_2, PI};
        self.rotation = match view {
            StandardView::Front => Quat::IDENTITY,
            StandardView::Back => Quat::from_rotation_y(PI),
            StandardView::Right => Quat::from_rotation_y(FRAC_PI_2),
            StandardView::Left => Quat::from_rotation_y(-FRAC_PI_2),
            StandardView::Top => Quat::from_rotation_x(-FRAC_PI_2),
            StandardView::Bottom => Quat::from_rotation_x(FRAC_PI_2),
            StandardView::Isometric => {
                self.set_isometric_axis(closest_axis(self.up()));
                return;
            }
        };
    }

    /// The "Vertical view": turns the view about the viewing direction until the global
    /// axis closest to the screen's up direction points straight up.
    pub fn set_vertical_view(&mut self) {
        let up = closest_axis(self.up());
        self.orient(self.forward(), up);
    }

    /// The axis view: looks down the global axis from its positive side, so the axis
    /// is the normal of the view plane. Of the global axes perpendicular to it, the one that
    /// turns the current view the least points up.
    pub fn set_axis_view(&mut self, axis: Vec3) {
        let normal = axis.normalize();
        let (right, up) = (self.right(), self.up());
        let candidates = [
            Vec3::X,
            Vec3::NEG_X,
            Vec3::Y,
            Vec3::NEG_Y,
            Vec3::Z,
            Vec3::NEG_Z,
        ];
        let score = |candidate: &Vec3| candidate.cross(normal).dot(right) + candidate.dot(up);
        let Some(view_up) = candidates
            .into_iter()
            .filter(|candidate| candidate.dot(normal).abs() < 0.5)
            .max_by(|a, b| score(a).total_cmp(&score(b)))
        else {
            return;
        };
        self.orient(-normal, view_up);
    }

    /// Isometric view with a global axis pointing up on the screen, seen from above along the
    /// space diagonal nearest to the current viewing direction.
    pub fn set_isometric_axis(&mut self, axis: Vec3) {
        let axis = axis.normalize();
        let eye = -self.forward();
        let sign = |c: f32| if c < 0.0 { -1.0 } else { 1.0 };
        let mut diagonal = Vec3::new(sign(eye.x), sign(eye.y), sign(eye.z));
        // From above: the component along the axis is positive.
        diagonal += axis * (axis.dot(diagonal).abs() - axis.dot(diagonal));
        self.orient(-diagonal.normalize(), axis);
    }

    /// Looks along `forward` with `up` made perpendicular to it.
    fn orient(&mut self, forward: Vec3, up: Vec3) {
        let right = forward.cross(up);
        if right.length_squared() < 1e-8 {
            return;
        }
        let right = right.normalize();
        let up = right.cross(forward).normalize();
        self.rotation = Quat::from_mat3(&Mat3::from_cols(right, up, -forward)).normalize();
    }
}

/// The signed global axis with the largest share of `v`.
fn closest_axis(v: Vec3) -> Vec3 {
    let a = v.abs();
    if a.x >= a.y && a.x >= a.z {
        Vec3::X * v.x.signum()
    } else if a.y >= a.z {
        Vec3::Y * v.y.signum()
    } else {
        Vec3::Z * v.z.signum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-4;

    fn project(camera: &Camera, point: Vec3) -> Vec3 {
        project_with_aspect(camera, point, 1.0)
    }

    fn project_with_aspect(camera: &Camera, point: Vec3, aspect: f32) -> Vec3 {
        camera.view_proj(aspect).project_point3(point)
    }

    #[test]
    fn target_projects_to_viewport_center() {
        let mut camera = Camera::default();
        camera.fit(Vec3::new(1.0, 2.0, 3.0), Vec3::new(5.0, 4.0, 9.0));
        let ndc = project(&camera, camera.target);
        assert!(ndc.x.abs() < EPS && ndc.y.abs() < EPS);
        assert!((0.0..=1.0).contains(&ndc.z));
    }

    #[test]
    fn fitted_box_is_inside_view_volume() {
        let mut camera = Camera::default();
        let (min, max) = (Vec3::splat(-1.0), Vec3::new(3.0, 1.0, 2.0));
        camera.fit(min, max);
        for (step, aspect) in [0.3, 0.8, 1.0, 1.7, 3.0]
            .into_iter()
            .cycle()
            .take(10)
            .enumerate()
        {
            camera.orbit(37.0 * step as f32, -21.0);
            for corner in 0..8 {
                let point = Vec3::new(
                    if corner & 1 == 0 { min.x } else { max.x },
                    if corner & 2 == 0 { min.y } else { max.y },
                    if corner & 4 == 0 { min.z } else { max.z },
                );
                let ndc = project_with_aspect(&camera, point, aspect);
                assert!(ndc.x.abs() <= 1.0 && ndc.y.abs() <= 1.0, "{ndc}");
                assert!((0.0..=1.0).contains(&ndc.z), "{ndc}");
            }
        }
    }

    #[test]
    fn orbit_about_keeps_the_pivot_on_the_screen() {
        let mut camera = Camera::default();
        camera.fit(Vec3::new(-15.0, -1.0, -1.0), Vec3::new(15.0, 1.0, 1.0));
        camera.zoom_at(0.01, 300.0, -40.0, 800.0, 600.0);
        let pivot = Vec3::new(14.0, 0.5, 0.8);
        let before = camera.view_proj(1.5).project_point3(pivot);
        let distance = camera.distance;
        camera.orbit_about(pivot, 120.0, -45.0);
        let after = camera.view_proj(1.5).project_point3(pivot);
        assert!((after.x - before.x).abs() < 1e-3 && (after.y - before.y).abs() < 1e-3);
        assert_eq!(camera.distance, distance);
    }

    #[test]
    fn orbit_keeps_distance_and_orthonormal_axes() {
        let mut camera = Camera::default();
        let distance_before = (camera.eye() - camera.target).length();
        camera.orbit(120.0, 45.0);
        assert!(((camera.eye() - camera.target).length() - distance_before).abs() < EPS);
        assert!(camera.right().dot(camera.up()).abs() < EPS);
        assert!(camera.forward().dot(camera.up()).abs() < EPS);
    }

    #[test]
    fn pan_moves_scene_with_the_mouse() {
        let mut camera = Camera::default();
        let anchor = Vec3::ZERO;
        let before = project(&camera, anchor);
        camera.pan(50.0, 0.0, 500.0, 500.0);
        let after = project(&camera, anchor);
        assert!(
            (after.x - before.x - 0.2).abs() < EPS,
            "{before} -> {after}"
        );
        assert!((after.y - before.y).abs() < EPS);
    }

    #[test]
    fn pan_uses_shorter_viewport_side() {
        let mut camera = Camera::default();
        let aspect = 0.5;
        let before = project_with_aspect(&camera, Vec3::ZERO, aspect);
        camera.pan(25.0, 0.0, 250.0, 500.0);
        let after = project_with_aspect(&camera, Vec3::ZERO, aspect);
        assert!(
            (after.x - before.x - 0.2).abs() < EPS,
            "{before} -> {after}"
        );
    }

    #[test]
    fn pixel_size_shrinks_when_zooming_in() {
        let mut camera = Camera::default();
        let before = camera.pixel_size(800.0, 400.0);
        assert!((before - 2.0 * camera.half_extent() / 400.0).abs() < EPS);
        camera.zoom(0.5);
        assert!((camera.pixel_size(800.0, 400.0) - before * 0.5).abs() < EPS);
    }

    #[test]
    fn standard_views_look_from_the_named_side() {
        let mut camera = Camera::default();
        for (view, eye, up) in [
            (StandardView::Front, Vec3::Z, Vec3::Y),
            (StandardView::Back, Vec3::NEG_Z, Vec3::Y),
            (StandardView::Right, Vec3::X, Vec3::Y),
            (StandardView::Left, Vec3::NEG_X, Vec3::Y),
            (StandardView::Top, Vec3::Y, Vec3::NEG_Z),
            (StandardView::Bottom, Vec3::NEG_Y, Vec3::Z),
        ] {
            camera.set_view(view);
            assert!((camera.forward() + eye).length() < EPS, "{view:?}");
            assert!((camera.up() - up).length() < EPS, "{view:?}");
        }
    }

    #[test]
    fn zoom_in_magnifies() {
        let mut camera = Camera::default();
        let point = camera.target + camera.right();
        let before = project(&camera, point).x;
        camera.zoom(0.5);
        let after = project(&camera, point).x;
        assert!((after - 2.0 * before).abs() < EPS);
    }

    #[test]
    fn vertical_view_snaps_up_to_the_closest_axis() {
        let mut camera = Camera::default();
        camera.orbit(80.0, 0.0);
        camera.orbit(0.0, 30.0);
        let forward = camera.forward();
        camera.set_vertical_view();
        // Y points straight up on the screen; the view direction is kept.
        assert!(
            camera.right().dot(Vec3::Y).abs() < EPS,
            "{:?}",
            camera.right()
        );
        assert!(camera.up().y > 0.0);
        assert!((camera.forward() - forward).length() < EPS);
    }

    #[test]
    fn axis_view_looks_down_the_axis() {
        let mut camera = Camera::default();
        camera.set_axis_view(Vec3::Z);
        // Looking down Z onto the XY plane; Y stays up from the isometric view.
        assert!((camera.forward() - Vec3::NEG_Z).length() < EPS);
        assert!((camera.up() - Vec3::Y).length() < EPS);
        // From the top view the screen up direction -Z is perpendicular to X and stays up.
        camera.set_view(StandardView::Top);
        camera.set_axis_view(Vec3::X);
        assert!((camera.forward() - Vec3::NEG_X).length() < EPS);
        assert!(
            (camera.up() - Vec3::NEG_Z).length() < EPS,
            "{:?}",
            camera.up()
        );
    }

    #[test]
    fn zoom_at_keeps_the_point_under_the_mouse() {
        let mut camera = Camera::default();
        let (width, height) = (400.0, 400.0);
        let pixel = camera.pixel_size(width, height);
        let point = camera.target + (camera.right() * 100.0 - camera.up() * 50.0) * pixel;
        let before = project(&camera, point);
        camera.zoom_at(0.5, 100.0, 50.0, width, height);
        let after = project(&camera, point);
        assert!(
            (before.truncate() - after.truncate()).length() < EPS,
            "{before} {after}"
        );
    }

    #[test]
    fn isometric_axis_looks_along_a_diagonal_from_above() {
        let mut camera = Camera::default();
        camera.set_view(StandardView::Bottom);
        camera.orbit(30.0, 0.0);
        camera.set_isometric_axis(Vec3::Z);
        assert!(camera.right().dot(Vec3::Z).abs() < EPS);
        assert!(camera.up().z > 0.0);
        let f = camera.forward();
        let third = 1.0 / 3.0_f32.sqrt();
        assert!((f.abs() - Vec3::splat(third)).length() < EPS, "{f:?}");
        assert!(f.z < 0.0, "{f:?}");
    }

    #[test]
    fn isometric_view_keeps_the_upright_axis() {
        let third = 1.0 / 3.0_f32.sqrt();
        // A fresh camera looks from (1, 1, 1) with Y up.
        let camera = Camera::default();
        assert!((camera.forward() + Vec3::splat(third)).length() < EPS);
        assert!(camera.right().dot(Vec3::Y).abs() < EPS && camera.up().y > 0.0);
        // A slightly tilted view with Z up stays upright on Z.
        let mut camera = Camera::default();
        camera.set_isometric_axis(Vec3::Z);
        camera.orbit(25.0, -15.0);
        camera.set_view(StandardView::Isometric);
        assert!(camera.right().dot(Vec3::Z).abs() < EPS && camera.up().z > 0.0);
        assert!((camera.forward().abs() - Vec3::splat(third)).length() < EPS);
    }
}
