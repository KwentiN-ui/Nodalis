use egui::{Color32, PointerButton, Pos2, Rect, Sense, Ui, pos2};
use egui_wgpu::RenderState;
use glam::{Mat4, Vec2, Vec3};
use plx_render::wgpu::FilterMode;
use plx_render::{Camera, ClipPlane, DisplayOptions, RenderMesh, StandardView, ViewportRenderer};

use crate::gizmo::{GizmoDrag, GizmoState, PlaneGizmo};
use crate::overlay::{self, LabelOffsets, Overlay};

const ZOOM_PER_SCROLL_POINT: f32 = 0.002;
const ZOOM_PER_DRAG_POINT: f32 = 0.01;

/// The 3D view: owns the GPU renderer and the camera and maps mouse input to camera moves.
///
/// The middle button rotates about the model point under the cursor, with Shift it pans and
/// with Ctrl it zooms; the wheel zooms about the cursor.
pub struct Viewport {
    render_state: RenderState,
    renderer: ViewportRenderer,
    texture: egui::TextureId,
    camera: Camera,
    pub options: DisplayOptions,
    /// Legend, information block and markers drawn over the scene.
    pub overlay: Overlay,
    /// Boundary condition and load symbols, under the annotations.
    pub symbols: Vec<crate::symbols::Symbol>,
    /// Where the user dragged the labels.
    pub labels: LabelOffsets,
    /// Plots of result paths drawn as boxes over the scene.
    pub path_plots: Vec<crate::path_plot::PlotBox>,
    /// A dialog picks in the 3D view: the left button selects and draws selection boxes.
    pub selecting: bool,
    /// What a click at the resting mouse would select, drawn in orange.
    pub preview: Preview,
    /// Where the selection box drag started.
    box_start: Option<Pos2>,
    /// Pointer position and since when it rests there, for the hover preview.
    resting: Option<(Pos2, f64)>,
    /// The resting position last reported as hover.
    hovered: Option<Pos2>,
    /// Where the view was drawn last, in points.
    pub rect: Rect,
    /// The section plane being edited, with its manipulator.
    pub gizmo: Option<PlaneGizmo>,
    gizmo_state: GizmoState,
    /// Counts scene replacements; section faces have to be rebuilt for each new scene.
    scene_version: u64,
    /// The point of the model the camera rotates about while the middle button is held; the
    /// owner sets it from [`ViewportResponse::pivot`]. Without one the camera turns about
    /// its target.
    pivot: Option<Vec3>,
    /// The view of a flat 2D model: the camera looks at the x-y plane from +z and only
    /// pans, zooms and turns in quarter turns about z.
    planar: bool,
}

/// Lines and points in render coordinates drawn as the hover preview.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Preview {
    pub lines: Vec<[Vec3; 2]>,
    pub points: Vec<Vec3>,
}

/// A selection box dragged in the 3D view, in normalised device coordinates.
#[derive(Clone, Copy, Debug)]
pub struct BoxSelect {
    pub view_proj: Mat4,
    pub min: Vec2,
    pub max: Vec2,
    /// Dragged from right to left: items crossing the box count, not only those inside.
    pub crossing: bool,
    pub shift: bool,
    pub ctrl: bool,
}

impl BoxSelect {
    /// Whether a point in render coordinates lies inside the box and in front of the camera.
    pub fn contains(&self, point: Vec3) -> bool {
        let ndc = self.view_proj.project_point3(point);
        (0.0..=1.0).contains(&ndc.z)
            && ndc.x >= self.min.x
            && ndc.x <= self.max.x
            && ndc.y >= self.min.y
            && ndc.y <= self.max.y
    }
}

/// How long the mouse has to rest before the hover preview is shown.
const HOVER_DELAY_SECONDS: f64 = 0.2;

/// The colour of the hover preview.
const PREVIEW_COLOR: Color32 = Color32::from_rgb(255, 175, 0);

/// Colour of the marker at the rotation pivot.
const PIVOT_COLOR: Color32 = Color32::from_rgb(220, 30, 30);

/// A click into the scene as a ray in render coordinates.
#[derive(Clone, Copy, Debug)]
pub struct Click {
    pub origin: Vec3,
    pub direction: Vec3,
    /// The ray through a pixel [`PICK_PRECISION_PIXELS`] to the side, to turn the pick
    /// tolerance into a distance in the scene.
    pub side_origin: Vec3,
    pub side_direction: Vec3,
    pub shift: bool,
    pub ctrl: bool,
}

/// How close to an edge or point a click has to be to pick it.
pub const PICK_PRECISION_PIXELS: f32 = 7.0;

impl Click {
    /// The pick tolerance at a point on the click ray.
    pub fn precision_at(&self, point: Vec3) -> f32 {
        let offset = point - self.side_origin;
        (offset - self.side_direction * offset.dot(self.side_direction)).length()
    }
}

/// What happened in the 3D view this frame.
#[derive(Default)]
pub struct ViewportResponse {
    pub command: Option<ViewCommand>,
    pub click: Option<Click>,
    pub box_select: Option<BoxSelect>,
    /// The mouse came to rest over the scene (`Some(ray)`) or left it (`Some(None)`).
    pub hover: Option<Option<Click>>,
    /// A right click, which opens the context menu; the owner adds what was clicked on.
    pub secondary_click: Option<Click>,
    /// The view's response, for the owner's context menu.
    pub response: Option<egui::Response>,
    /// The section plane manipulator was dragged.
    pub gizmo: Option<GizmoDrag>,
    /// The context menu of a query annotation was used.
    pub annotation: Option<overlay::AnnotationCommand>,
    /// Path plots were moved, resized or deleted.
    pub path_plots: Vec<crate::path_plot::PlotEvent>,
    /// The legend asked for the colour scale dialog.
    pub edit_legend: bool,
    /// The middle button was pressed here: the owner answers with
    /// [`Viewport::set_pivot`] to rotate about the model point under the cursor.
    pub pivot: Option<Click>,
}

/// The view entries of the 3D view's context menu; `planar` for the view of a 2D model,
/// which only turns in the x-y plane.
pub fn view_menu(ui: &mut Ui, planar: bool) -> Option<ViewCommand> {
    let mut command = None;
    if ui.button("Zoom to Fit").clicked() {
        command = Some(ViewCommand::Fit);
    }
    if planar {
        for (entry, label) in [
            (ViewCommand::RotateLeft, "Rotate Left"),
            (ViewCommand::RotateRight, "Rotate Right"),
            (ViewCommand::DefaultRotation, "Default Rotation"),
        ] {
            if ui.button(label).clicked() {
                command = Some(entry);
            }
        }
        return command;
    }
    if ui.button("Vertical View").clicked() {
        command = Some(ViewCommand::Vertical);
    }
    ui.menu_button("View Normal to Axis", |ui| {
        for axis in Axis::ALL {
            if ui.button(axis.label()).clicked() {
                command = Some(ViewCommand::AxisView(axis));
            }
        }
    });
    ui.menu_button("Isometric, Axis Up", |ui| {
        for axis in Axis::ALL {
            if ui.button(axis.label()).clicked() {
                command = Some(ViewCommand::IsometricAxis(axis));
            }
        }
    });
    command
}

/// Camera requests from toolbar, menu or tree, applied by the owner of the model bounds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewCommand {
    Fit,
    View(StandardView),
    /// Turns the closest global axis straight up.
    Vertical,
    /// Looks down the given global axis onto the plane it is normal to.
    AxisView(Axis),
    /// Isometric view with the given global axis up.
    IsometricAxis(Axis),
    /// The view of a 2D model turned a quarter turn: the model turns counterclockwise.
    RotateLeft,
    /// The model turns clockwise.
    RotateRight,
    /// The view of a 2D model with y up.
    DefaultRotation,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    pub const ALL: [Axis; 3] = [Axis::X, Axis::Y, Axis::Z];

    pub fn label(self) -> &'static str {
        match self {
            Axis::X => "X",
            Axis::Y => "Y",
            Axis::Z => "Z",
        }
    }

    fn vector(self) -> Vec3 {
        match self {
            Axis::X => Vec3::X,
            Axis::Y => Vec3::Y,
            Axis::Z => Vec3::Z,
        }
    }
}

impl Viewport {
    pub fn new(render_state: RenderState) -> Self {
        let renderer = ViewportRenderer::new(&render_state.device);
        let texture = render_state.renderer.write().register_native_texture(
            &render_state.device,
            renderer.color_view(),
            FilterMode::Linear,
        );
        Self {
            render_state,
            renderer,
            texture,
            camera: Camera::default(),
            options: DisplayOptions::default(),
            overlay: Overlay::default(),
            symbols: Vec::new(),
            labels: LabelOffsets::default(),
            path_plots: Vec::new(),
            selecting: false,
            preview: Preview::default(),
            box_start: None,
            resting: None,
            hovered: None,
            rect: Rect::NOTHING,
            gizmo: None,
            gizmo_state: GizmoState::default(),
            scene_version: 0,
            pivot: None,
            planar: false,
        }
    }

    /// Sets the point the camera rotates about until the middle button is released; `None`
    /// rotates about the target, e.g. when the cursor points into empty space.
    pub fn set_pivot(&mut self, pivot: Option<Vec3>) {
        self.pivot = pivot;
    }

    /// Makes the view that of a flat 2D model, or a free 3D view again. A 2D view that does
    /// not look at the x-y plane is turned to the front.
    pub fn set_planar(&mut self, planar: bool) {
        self.planar = planar;
        if planar && !self.camera.looks_at_xy_plane() {
            self.camera.set_view(StandardView::Front);
        }
    }

    pub fn planar(&self) -> bool {
        self.planar
    }

    pub fn camera(&self) -> &Camera {
        &self.camera
    }

    /// Cuts the scene at the section plane and shows the section faces of the parts, or shows
    /// the whole scene again.
    pub fn set_section(&mut self, section: Option<(ClipPlane, &[RenderMesh])>) {
        let device = &self.render_state.device;
        match section {
            Some((clip, faces)) => {
                self.renderer.set_clip_plane(Some(clip));
                self.renderer.set_sections(device, faces);
            }
            None => {
                self.renderer.set_clip_plane(None);
                self.renderer.set_sections(device, &[]);
            }
        }
    }

    /// Parts, by index, that the section view leaves whole.
    pub fn set_uncut_parts(&mut self, uncut: &[bool]) {
        self.renderer.set_unclipped_parts(uncut);
    }

    /// Draws only the section faces of the parts while the section view is on.
    pub fn set_sections_only(&mut self, only: bool) {
        self.renderer.set_sections_only(only);
    }

    pub fn set_parts(&mut self, parts: &[RenderMesh]) {
        self.renderer.set_parts(&self.render_state.device, parts);
        self.scene_version += 1;
    }

    /// Replaces the mesh of one part and keeps the others.
    pub fn set_part(&mut self, index: usize, mesh: &RenderMesh) {
        self.renderer
            .set_part(&self.render_state.device, index, mesh);
        self.scene_version += 1;
    }

    pub fn scene_version(&self) -> u64 {
        self.scene_version
    }

    /// Widens the clipping range so that nothing of the box is cut off, e.g. when the parts
    /// are pulled apart; the view itself stays.
    pub fn cover(&mut self, (min, max): (Vec3, Vec3)) {
        let target = self.camera.target;
        let reach = (target - min).abs().max((max - target).abs()).length();
        self.camera.scene_radius = self.camera.scene_radius.max(reach * 0.5);
    }

    /// Exchanges the camera, so the FE model and the results each keep their own view.
    pub fn set_camera(&mut self, camera: Camera) {
        self.camera = camera;
    }

    pub fn swap_camera(&mut self, camera: &mut Camera) {
        std::mem::swap(&mut self.camera, camera);
    }

    pub fn set_part_visible(&mut self, index: usize, visible: bool) {
        self.renderer.set_part_visible(index, visible);
    }

    pub fn apply(&mut self, command: ViewCommand, bounds: Option<(Vec3, Vec3)>) {
        match command {
            ViewCommand::Fit => {
                if let Some((min, max)) = bounds {
                    self.camera.fit(min, max);
                }
            }
            // A 2D model stays seen from the front.
            ViewCommand::View(_) | ViewCommand::AxisView(_) | ViewCommand::IsometricAxis(_)
                if self.planar => {}
            ViewCommand::View(view) => self.camera.set_view(view),
            ViewCommand::Vertical => self.camera.set_vertical_view(),
            ViewCommand::AxisView(axis) => self.camera.set_axis_view(axis.vector()),
            ViewCommand::IsometricAxis(axis) => self.camera.set_isometric_axis(axis.vector()),
            ViewCommand::RotateLeft => self.camera.rotate_quarter(true),
            ViewCommand::RotateRight => self.camera.rotate_quarter(false),
            ViewCommand::DefaultRotation => self.camera.set_view(StandardView::Front),
        }
    }

    /// Draws the scene with its annotations; reports camera commands and clicks.
    pub fn ui(&mut self, ui: &mut Ui) -> ViewportResponse {
        let mut result = ViewportResponse::default();
        let (rect, response) = ui.allocate_exact_size(ui.available_size(), Sense::click_and_drag());
        self.rect = rect;
        let delta = response.drag_delta();
        // The section plane manipulator takes left drags that start on one of its handles.
        result.gizmo =
            self.gizmo_state
                .interact(self.gizmo.as_ref(), &self.camera, rect, &response);
        let free = !self.gizmo_state.dragging();
        let modifiers = ui.input(|i| i.modifiers);
        let ctrl = modifiers.ctrl || modifiers.command;
        // The rotation pivot is picked when the middle button goes down, before the drag
        // starts, so the owner can answer in time.
        let (middle_pressed, middle_down) = ui.input(|i| {
            (
                i.pointer.button_pressed(PointerButton::Middle),
                i.pointer.button_down(PointerButton::Middle),
            )
        });
        if middle_pressed
            && !self.planar
            && response.hovered()
            && let Some(pointer) = ui.input(|i| i.pointer.interact_pos())
        {
            self.pivot = None;
            result.pivot = Some(self.click_at(rect, pointer, modifiers));
        }
        if !middle_down && !response.dragged_by(PointerButton::Middle) {
            self.pivot = None;
        }
        // The mouse: the middle button rotates, with Shift it pans and with Ctrl it zooms;
        // the left button picks and draws selection boxes, the right one opens the context menu.
        // The view of a 2D model does not rotate: the middle button pans.
        if response.dragged_by(PointerButton::Middle) {
            if ctrl {
                self.camera.zoom((delta.y * ZOOM_PER_DRAG_POINT).exp());
            } else if modifiers.shift || self.planar {
                self.camera
                    .pan(delta.x, delta.y, rect.width(), rect.height());
            } else if let Some(pivot) = self.pivot {
                self.camera.orbit_about(pivot, delta.x, delta.y);
            } else {
                self.camera.orbit(delta.x, delta.y);
            }
        }
        if free && self.selecting && response.drag_started_by(PointerButton::Primary) {
            self.box_start = response.interact_pointer_pos();
        }
        if !self.selecting {
            self.box_start = None;
        }
        if response.hovered()
            && let Some(pointer) = ui.input(|i| i.pointer.hover_pos())
        {
            let scroll = ui.input(|input| input.smooth_scroll_delta.y);
            if scroll != 0.0 {
                // Ctrl zooms in finer steps.
                let rate = if ctrl { 0.2 } else { 1.0 } * ZOOM_PER_SCROLL_POINT;
                let offset = pointer - rect.center();
                self.camera.zoom_at(
                    (-scroll * rate).exp(),
                    offset.x,
                    offset.y,
                    rect.width(),
                    rect.height(),
                );
            }
        }
        if response.secondary_clicked()
            && let Some(pointer) = response.interact_pointer_pos()
        {
            result.secondary_click = Some(self.click_at(rect, pointer, modifiers));
        }
        if response.double_clicked() {
            result.command = Some(ViewCommand::Fit);
        } else if response.clicked()
            && !self.gizmo_state.hovered()
            && let Some(pointer) = response.interact_pointer_pos()
        {
            result.click = Some(self.click_at(rect, pointer, modifiers));
        }
        let mut dragged_box = None;
        if let Some(start) = self.box_start {
            let end = ui.input(|i| i.pointer.latest_pos()).unwrap_or(start);
            let crossing = end.x < start.x;
            dragged_box = Some((Rect::from_two_pos(start, end), crossing));
            if response.drag_stopped() || !ui.input(|i| i.pointer.primary_down()) {
                self.box_start = None;
                let area = Rect::from_two_pos(start, end);
                if area.width() > 1.0 && area.height() > 1.0 {
                    let ndc = |p: Pos2| {
                        Vec2::new(
                            (p.x - rect.center().x) / (rect.width() * 0.5),
                            (rect.center().y - p.y) / (rect.height() * 0.5),
                        )
                    };
                    let (a, b) = (ndc(area.left_bottom()), ndc(area.right_top()));
                    result.box_select = Some(BoxSelect {
                        view_proj: self.camera.view_proj(rect.aspect_ratio()),
                        min: a.min(b),
                        max: a.max(b),
                        crossing,
                        shift: modifiers.shift,
                        ctrl: modifiers.command,
                    });
                }
                dragged_box = None;
            }
        }
        let orbiting =
            response.dragged_by(PointerButton::Middle) && !ctrl && !modifiers.shift && !self.planar;
        result.hover = self.hover(ui, rect, &response);
        result.response = Some(response);

        let pixels_per_point = ui.ctx().pixels_per_point();
        let width = (rect.width() * pixels_per_point).round() as u32;
        let height = (rect.height() * pixels_per_point).round() as u32;
        let device = &self.render_state.device;
        if self.renderer.resize(device, width, height) {
            self.render_state
                .renderer
                .write()
                .update_egui_texture_from_wgpu_texture(
                    device,
                    self.renderer.color_view(),
                    FilterMode::Linear,
                    self.texture,
                );
        }
        self.options.background = crate::style::viewport_background();
        self.options.wireframe = crate::style::viewport_wireframe();
        self.renderer
            .render(device, &self.render_state.queue, &self.camera, self.options);
        let painter = ui.painter_at(rect);
        painter.image(
            self.texture,
            rect,
            Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0)),
            Color32::WHITE,
        );
        crate::symbols::draw(&painter, rect, &self.camera, &self.symbols);
        let clicked = overlay::draw(
            ui,
            rect,
            &self.camera,
            &self.overlay,
            &mut self.labels,
            self.planar,
        );
        if let Some(axis) = clicked.axis {
            self.camera.set_axis_view(axis);
            ui.ctx().request_repaint();
        }
        result.annotation = clicked.annotation;
        result.edit_legend = clicked.edit_legend;
        result.path_plots = crate::path_plot::draw(ui, rect, &self.camera, &mut self.path_plots);
        if let Some(pivot) = self.pivot.filter(|_| orbiting) {
            // A small marker shows the point the view turns about.
            let center = overlay::project(&self.camera, rect, pivot);
            painter.circle(
                center,
                4.0,
                PIVOT_COLOR,
                egui::Stroke::new(1.5, Color32::WHITE),
            );
        }
        if let Some(gizmo) = &self.gizmo {
            self.gizmo_state.draw(&painter, gizmo, &self.camera, rect);
        }
        if self.selecting {
            let to_screen = |p: Vec3| overlay::project(&self.camera, rect, p);
            // One mesh of plain quads: large patches have tens of thousands of edges, which
            // egui's anti-aliased line segments would make slow to draw every frame.
            let mut lines = egui::Mesh::default();
            for [a, b] in &self.preview.lines {
                let (a, b) = (to_screen(*a), to_screen(*b));
                if a == b {
                    continue;
                }
                let side = (b - a).normalized().rot90();
                let index = lines.vertices.len() as u32;
                for corner in [a + side, b + side, b - side, a - side] {
                    lines.colored_vertex(corner, PREVIEW_COLOR);
                }
                lines.add_triangle(index, index + 1, index + 2);
                lines.add_triangle(index, index + 2, index + 3);
            }
            painter.add(lines);
            for &point in &self.preview.points {
                painter.rect_filled(
                    Rect::from_center_size(to_screen(point), egui::vec2(7.0, 7.0)),
                    0.0,
                    PREVIEW_COLOR,
                );
            }
        }
        if let Some((area, crossing)) = dragged_box {
            // Nodalis draws a box selecting items inside blue, one selecting crossing items
            // green.
            let (fill, border) = if crossing {
                (
                    Color32::from_rgba_unmultiplied(51, 255, 128, 50),
                    Color32::from_rgb(77, 255, 77),
                )
            } else {
                (
                    Color32::from_rgba_unmultiplied(135, 206, 250, 60),
                    Color32::from_rgb(0, 120, 215),
                )
            };
            painter.rect(
                area,
                0.0,
                fill,
                egui::Stroke::new(1.0, border),
                egui::StrokeKind::Inside,
            );
            ui.ctx().request_repaint();
        }
        result
    }

    /// The pick ray through a point of the view.
    fn click_at(&self, rect: Rect, pointer: Pos2, modifiers: egui::Modifiers) -> Click {
        // Ray through the pixel from the near to the far plane.
        let inverse = self.camera.view_proj(rect.aspect_ratio()).inverse();
        let ray = |pointer: Pos2| {
            let x = (pointer.x - rect.center().x) / (rect.width() * 0.5);
            let y = (rect.center().y - pointer.y) / (rect.height() * 0.5);
            let near = inverse.project_point3(Vec3::new(x, y, 0.0));
            let far = inverse.project_point3(Vec3::new(x, y, 1.0));
            (near, (far - near).normalize_or_zero())
        };
        let (origin, direction) = ray(pointer);
        let (side_origin, side_direction) = ray(pointer + egui::vec2(PICK_PRECISION_PIXELS, 0.0));
        Click {
            origin,
            direction,
            side_origin,
            side_direction,
            shift: modifiers.shift,
            ctrl: modifiers.command,
        }
    }

    /// Reports the pointer once it has rested over the scene, and when it leaves.
    fn hover(&mut self, ui: &Ui, rect: Rect, response: &egui::Response) -> Option<Option<Click>> {
        // No preview while a button is held, e.g. while the camera turns.
        let pressed = ui.input(|i| i.pointer.any_down());
        let pointer = response
            .hover_pos()
            .filter(|_| self.selecting && self.box_start.is_none() && !pressed);
        let Some(pointer) = pointer else {
            self.resting = None;
            return self.hovered.take().map(|_| None);
        };
        let now = ui.input(|i| i.time);
        match self.resting {
            Some((rest, since)) if rest.distance(pointer) < 0.5 => {
                if now - since < HOVER_DELAY_SECONDS {
                    ui.ctx()
                        .request_repaint_after(std::time::Duration::from_secs_f64(
                            HOVER_DELAY_SECONDS - (now - since),
                        ));
                    return None;
                }
            }
            _ => {
                self.resting = Some((pointer, now));
                ui.ctx()
                    .request_repaint_after(std::time::Duration::from_secs_f64(HOVER_DELAY_SECONDS));
                // Moving hides the preview until the mouse rests again.
                return self.hovered.take().map(|_| None);
            }
        }
        if self.hovered == Some(pointer) {
            return None;
        }
        self.hovered = Some(pointer);
        let modifiers = ui.input(|i| i.modifiers);
        Some(Some(self.click_at(rect, pointer, modifiers)))
    }
}
