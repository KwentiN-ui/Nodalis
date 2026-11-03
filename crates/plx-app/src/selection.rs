//! The selection dialog ("Set Selection"): while an item dialog picks a region in the
//! 3D view, this window chooses what a click selects, and keeps the history of picks so that
//! Undo works.
//!
//! A click without modifier replaces the selection, Shift adds, Ctrl removes
//! and Shift+Ctrl keeps the intersection. A click into empty space clears the selection.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use glam::{DVec3, Vec3};
use plx_mesh::{CadEntity, ElementId, NodeId, SkinFace, face_normal};

use crate::model::{Hit, Model};
use crate::numeric;
use crate::viewport::{BoxSelect, Preview};

/// How a pick combines with the selection so far.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Operation {
    Replace,
    Add,
    Subtract,
    Intersect,
}

impl Operation {
    pub fn from_modifiers(shift: bool, ctrl: bool) -> Self {
        match (shift, ctrl) {
            (true, true) => Operation::Intersect,
            (true, false) => Operation::Add,
            (false, true) => Operation::Subtract,
            (false, false) => Operation::Replace,
        }
    }
}

/// The picks of a selection in order; the selected items are the picks replayed.
#[derive(Clone, Debug, PartialEq)]
pub struct History<T> {
    steps: Vec<(Operation, BTreeSet<T>)>,
}

impl<T> Default for History<T> {
    fn default() -> Self {
        Self { steps: Vec::new() }
    }
}

impl<T: Ord + Clone> History<T> {
    /// A history holding a selection made earlier, e.g. of an item being edited.
    pub fn from_items(items: impl IntoIterator<Item = T>) -> Self {
        let items: BTreeSet<T> = items.into_iter().collect();
        let mut history = Self::default();
        if !items.is_empty() {
            history.push(Operation::Replace, items);
        }
        history
    }

    pub fn push(&mut self, operation: Operation, items: BTreeSet<T>) {
        if operation == Operation::Replace {
            self.steps.clear();
        }
        self.steps.push((operation, items));
    }

    pub fn undo(&mut self) {
        self.steps.pop();
    }

    pub fn clear(&mut self) {
        self.steps.clear();
    }

    pub fn can_undo(&self) -> bool {
        !self.steps.is_empty()
    }

    pub fn items(&self) -> BTreeSet<T> {
        let mut items = BTreeSet::new();
        for (operation, picked) in &self.steps {
            match operation {
                Operation::Replace => items = picked.clone(),
                Operation::Add => items.extend(picked.iter().cloned()),
                Operation::Subtract => items.retain(|i| !picked.contains(i)),
                Operation::Intersect => items.retain(|i| picked.contains(i)),
            }
        }
        items
    }
}

/// What the region of the item consists of.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    /// Boundary conditions and forces on points act on nodes.
    Nodes,
    /// Pressure and forces act on element faces.
    Faces,
    /// Contacts and ties in 3D models: element faces, or the edges of shells, which
    /// the solver takes as faces too.
    Surfaces,
    /// In 2D models they act on the outline: the edges of the 2D elements, stored as faces.
    Edges,
    /// Edges of 3D parts, e.g. of a shell for a force along it: the element
    /// edges along the outline, by their end nodes.
    Lines,
    /// A single point, e.g. an end of a path: picked as nodes, but the geometry mode takes
    /// only corner points, as an edge or a face does not define a point.
    Point,
}

/// What a click selects.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SelectBy {
    /// The surface patch under the mouse, or the edge or corner point near the click.
    Geometry,
    GeometryPart,
    GeometryEdgeAngle,
    GeometrySurfaceAngle,
    Node,
    Element,
    Edge,
    Surface,
    Part,
    EdgeAngle,
    FaceAngle,
    Id,
}

const GEOMETRY_MODES: [SelectBy; 4] = [
    SelectBy::Geometry,
    SelectBy::GeometryPart,
    SelectBy::GeometryEdgeAngle,
    SelectBy::GeometrySurfaceAngle,
];

const MESH_MODES: [SelectBy; 8] = [
    SelectBy::Node,
    SelectBy::Element,
    SelectBy::Edge,
    SelectBy::Surface,
    SelectBy::Part,
    SelectBy::EdgeAngle,
    SelectBy::FaceAngle,
    SelectBy::Id,
];

impl SelectBy {
    fn label(self, target: Target) -> &'static str {
        match self {
            SelectBy::Geometry if target == Target::Faces => "Faces",
            SelectBy::Geometry if target == Target::Surfaces => "Faces and shell edges",
            SelectBy::Geometry if matches!(target, Target::Edges | Target::Lines) => "Edges",
            SelectBy::Geometry if target == Target::Point => "Points",
            SelectBy::Geometry => "Faces, edges and points",
            SelectBy::GeometryPart | SelectBy::Part => "Part",
            SelectBy::GeometryEdgeAngle | SelectBy::EdgeAngle => "Edge angle",
            SelectBy::GeometrySurfaceAngle => "Surface angle",
            SelectBy::Node => "Node",
            SelectBy::Element => "Element",
            SelectBy::Edge => "Edge",
            SelectBy::Surface => "Surface",
            SelectBy::FaceAngle => "Surface angle",
            SelectBy::Id => "ID",
        }
    }

    fn is_geometry(self) -> bool {
        GEOMETRY_MODES.contains(&self)
    }

    /// Edges and points only make sense for regions of nodes or contact surfaces, faces not
    /// for edges.
    fn allowed(self, target: Target) -> bool {
        match target {
            Target::Nodes | Target::Point => true,
            Target::Faces => !matches!(
                self,
                SelectBy::Node | SelectBy::Edge | SelectBy::EdgeAngle | SelectBy::GeometryEdgeAngle
            ),
            // Edges pick the edges of shells.
            Target::Surfaces => self != SelectBy::Node,
            Target::Edges => !matches!(
                self,
                SelectBy::Node
                    | SelectBy::Surface
                    | SelectBy::FaceAngle
                    | SelectBy::GeometrySurfaceAngle
            ),
            Target::Lines => matches!(
                self,
                SelectBy::Geometry
                    | SelectBy::GeometryEdgeAngle
                    | SelectBy::Edge
                    | SelectBy::EdgeAngle
            ),
        }
    }

    fn angle_mode(self) -> bool {
        matches!(
            self,
            SelectBy::GeometryEdgeAngle
                | SelectBy::GeometrySurfaceAngle
                | SelectBy::EdgeAngle
                | SelectBy::FaceAngle
        )
    }
}

/// What a selection of CAD entities on the Geometry tab consists of.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeometryRegion {
    /// Faces and edges, e.g. of a local mesh size.
    FacesAndEdges,
    /// Faces only, e.g. the source faces of an extrude mesh.
    Faces,
}

impl GeometryRegion {
    fn allows(self, mode: SelectBy) -> bool {
        match self {
            GeometryRegion::FacesAndEdges => mode.is_geometry(),
            GeometryRegion::Faces => {
                matches!(mode, SelectBy::Geometry | SelectBy::GeometrySurfaceAngle)
            }
        }
    }
}

/// Which modes the selection window offers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum WindowKind {
    /// A region of the FE model picked as the target.
    Region(Target),
    /// Whole parts only.
    Parts,
    /// CAD entities on the Geometry tab: the geometry based modes only.
    Geometry(GeometryRegion),
}

/// Picked items in the id space of the target.
#[derive(Clone, Debug, PartialEq)]
pub enum Items {
    Nodes(BTreeSet<NodeId>),
    Faces(BTreeSet<(ElementId, u8)>),
    /// Element edges of 3D parts by their end nodes, the smaller first.
    Edges(BTreeSet<[NodeId; 2]>),
    /// Faces, edges and vertices of the CAD geometry the mesh was generated from.
    Geometry(BTreeSet<CadEntity>),
}

impl Items {
    /// CAD entities as the nodes or element faces of the target they stand for.
    pub fn resolved(self, mesh: &plx_mesh::FeMesh, target: Target) -> Items {
        match self {
            Items::Geometry(entities) => {
                let entities: Vec<CadEntity> = entities.into_iter().collect();
                match target {
                    Target::Nodes | Target::Point => {
                        Items::Nodes(mesh.cad_nodes(&entities).into_iter().collect())
                    }
                    Target::Faces | Target::Surfaces | Target::Edges => {
                        Items::Faces(mesh.cad_faces(&entities).into_iter().collect())
                    }
                    Target::Lines => Items::Edges(cad_segments(mesh, &entities)),
                }
            }
            items => items,
        }
    }
}

/// The dialog's settings, kept while it is open.
#[derive(Clone, Debug, PartialEq)]
pub struct Picker {
    pub select_by: SelectBy,
    /// Angles in degrees for the angle modes, the default 30°.
    angle: f64,
    ids: String,
    /// Shows the mesh based modes next to the geometry based ones.
    expanded: bool,
    error: Option<String>,
}

/// A button of the dialog the region has to act on.
#[derive(Clone, Debug, PartialEq)]
pub enum PickerAction {
    Undo,
    Clear,
    All,
    Invert,
    Ids(Operation, Vec<u32>),
}

impl Default for Picker {
    fn default() -> Self {
        Self {
            select_by: SelectBy::Geometry,
            angle: 30.0,
            ids: String::new(),
            expanded: false,
            error: None,
        }
    }
}

impl Picker {
    /// The selection window next to the item dialog `anchor`.
    pub fn window(
        &mut self,
        ctx: &egui::Context,
        anchor: egui::Rect,
        target: Target,
        can_undo: bool,
    ) -> Option<PickerAction> {
        if !self.select_by.allowed(target) {
            self.select_by = SelectBy::Geometry;
        }
        self.show(ctx, anchor, WindowKind::Region(target), can_undo)
    }

    /// The selection window of a region of whole parts: only the part mode
    /// is enabled, the mode chosen for other regions stays.
    pub fn parts_window(
        &mut self,
        ctx: &egui::Context,
        anchor: egui::Rect,
        can_undo: bool,
    ) -> Option<PickerAction> {
        self.show(ctx, anchor, WindowKind::Parts, can_undo)
    }

    /// The selection window of CAD faces and edges on the Geometry tab: only the geometry
    /// based modes, the display has no mesh to pick from.
    pub fn geometry_window(
        &mut self,
        ctx: &egui::Context,
        anchor: egui::Rect,
        region: GeometryRegion,
        can_undo: bool,
    ) -> Option<PickerAction> {
        if !region.allows(self.select_by) {
            self.select_by = SelectBy::Geometry;
        }
        self.show(ctx, anchor, WindowKind::Geometry(region), can_undo)
    }

    /// The angle of the angle modes in degrees.
    pub fn angle(&self) -> f64 {
        self.angle
    }

    fn show(
        &mut self,
        ctx: &egui::Context,
        anchor: egui::Rect,
        kind: WindowKind,
        can_undo: bool,
    ) -> Option<PickerAction> {
        let parts = kind == WindowKind::Parts;
        let target = match kind {
            WindowKind::Region(target) => target,
            _ => Target::Faces,
        };
        let geometry = match kind {
            WindowKind::Geometry(region) => Some(region),
            _ => None,
        };
        let mut action = None;
        // egui's own constraint uses a default size before the first layout and pins the
        // window there; keep it on screen with its real size instead. It follows the dialog,
        // so it is not movable: egui keeps a movable window where it first was.
        let id = crate::ids::window(ctx, "selection window");
        let width = egui::AreaState::load(ctx, id)
            .and_then(|s| s.size)
            .map_or(350.0, |s| s.x);
        let screen = ctx.content_rect();
        let pos = egui::pos2(
            (anchor.right() + 8.0)
                .min(screen.right() - width)
                .max(screen.left()),
            anchor.top(),
        );
        egui::Window::new("Selection")
            .id(id)
            .constrain(false)
            .movable(false)
            .collapsible(false)
            .resizable(false)
            .title_bar(true)
            .current_pos(pos)
            .show(ctx, |ui| {
                ui.horizontal_top(|ui| {
                    ui.group(|ui| {
                        ui.vertical(|ui| {
                            ui.strong("Geometry Based");
                            for mode in GEOMETRY_MODES {
                                if parts {
                                    self.part_mode_row(ui, mode);
                                } else if let Some(region) = geometry {
                                    self.geometry_mode_row(ui, mode, region);
                                } else {
                                    self.mode_row(ui, mode, target);
                                }
                            }
                        });
                    });
                    if self.expanded && kind == WindowKind::Region(target) {
                        ui.group(|ui| {
                            ui.vertical(|ui| {
                                ui.strong("Mesh Based");
                                for mode in MESH_MODES {
                                    self.mode_row(ui, mode, target);
                                }
                                ui.add_enabled_ui(self.select_by == SelectBy::Id, |ui| {
                                    ui.add(
                                        egui::TextEdit::singleline(&mut self.ids)
                                            .hint_text("e.g. 1, 5, 10-20")
                                            .desired_width(150.0),
                                    );
                                    ui.horizontal(|ui| {
                                        for (label, operation) in [
                                            ("Add", Operation::Add),
                                            ("Remove", Operation::Subtract),
                                        ] {
                                            if ui.button(label).clicked() {
                                                match parse_ids(&self.ids) {
                                                    Ok(ids) => {
                                                        self.error = None;
                                                        action =
                                                            Some(PickerAction::Ids(operation, ids));
                                                    }
                                                    Err(error) => self.error = Some(error),
                                                }
                                            }
                                        }
                                    });
                                    if matches!(
                                        target,
                                        Target::Faces | Target::Surfaces | Target::Edges
                                    ) {
                                        ui.weak("IDs of elements");
                                    }
                                });
                            });
                        });
                    }
                });
                if let Some(error) = &self.error {
                    ui.colored_label(crate::style::error(), error);
                }
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(can_undo, egui::Button::new("Undo"))
                        .clicked()
                    {
                        action = Some(PickerAction::Undo);
                    }
                    if ui.button("Clear").clicked() {
                        action = Some(PickerAction::Clear);
                    }
                    let mesh = parts || geometry.is_some() || !self.select_by.is_geometry();
                    if ui.add_enabled(mesh, egui::Button::new("All")).clicked() {
                        action = Some(PickerAction::All);
                    }
                    if ui.add_enabled(mesh, egui::Button::new("Invert")).clicked() {
                        action = Some(PickerAction::Invert);
                    }
                    let more = if self.expanded { "Less" } else { "More" };
                    if matches!(kind, WindowKind::Region(_)) && ui.button(more).clicked() {
                        self.expanded = !self.expanded;
                        if !self.expanded && !self.select_by.is_geometry() {
                            self.select_by = SelectBy::Geometry;
                        }
                    }
                });
                ui.weak("Shift: add, Ctrl: remove");
                ui.weak("Drag: box (to the left: also crossing)");
                ui.weak("Middle mouse button: rotate, with Shift pan");
            });
        action
    }

    /// A mode of the parts window: only the part mode is enabled, and always chosen.
    fn part_mode_row(&self, ui: &mut egui::Ui, mode: SelectBy) {
        let part = mode == SelectBy::GeometryPart;
        ui.add_enabled_ui(part, |ui| {
            ui.horizontal(|ui| {
                let _ = ui.radio(part, mode.label(Target::Faces));
                if mode.angle_mode() {
                    let mut angle = self.angle;
                    ui.add_enabled(false, numeric::drag_value(&mut angle).suffix(" °"));
                }
            });
        });
    }

    /// A mode of the geometry window, with the label of what the region consists of.
    fn geometry_mode_row(&mut self, ui: &mut egui::Ui, mode: SelectBy, region: GeometryRegion) {
        let label = match (mode, region) {
            (SelectBy::Geometry, GeometryRegion::FacesAndEdges) => "Faces and edges",
            (SelectBy::Geometry, GeometryRegion::Faces) => "Faces",
            _ => mode.label(Target::Faces),
        };
        self.labelled_mode_row(ui, mode, label, region.allows(mode));
    }

    fn mode_row(&mut self, ui: &mut egui::Ui, mode: SelectBy, target: Target) {
        self.labelled_mode_row(ui, mode, mode.label(target), mode.allowed(target));
    }

    fn labelled_mode_row(&mut self, ui: &mut egui::Ui, mode: SelectBy, label: &str, enabled: bool) {
        ui.add_enabled_ui(enabled, |ui| {
            ui.horizontal(|ui| {
                if ui.radio(self.select_by == mode, label).clicked() {
                    self.select_by = mode;
                }
                if mode.angle_mode() {
                    ui.add_enabled(
                        self.select_by == mode,
                        numeric::drag_value(&mut self.angle)
                            .range(0.0..=180.0)
                            .speed(1.0)
                            .suffix(" °"),
                    );
                }
            });
        });
    }

    /// What a click on `hit` selects. `precision` is the pick tolerance in render units at
    /// the hit, for edges and points.
    pub fn pick(&self, model: &Model, hit: &Hit, target: Target, precision: f32) -> Items {
        if target == Target::Point {
            return self.pick_point(model, hit, precision);
        }
        // On a mesh generated from geometry the geometry mode picks CAD entities, which stay
        // valid when the part is remeshed.
        if self.select_by == SelectBy::Geometry
            && model.has_cad()
            && let Some(entity) = crate::cad_selection::pick(model, hit, target, precision)
        {
            return Items::Geometry(BTreeSet::from([entity]));
        }
        if target == Target::Lines {
            return self.pick_edges(model, hit);
        }
        if hit.line {
            return self.pick_line(model, hit, target, precision);
        }
        let picker = MeshPicker::new(model, hit.part);
        if target == Target::Edges {
            // Edges of the outline whose nodes the same click would select.
            let nodes = match self.select_by {
                SelectBy::Geometry => match picker.nearest_edge(hit.point) {
                    Some(edge) => picker.edge_chain(edge, Some(OUTLINE_ANGLE.to_radians())),
                    None => BTreeSet::new(),
                },
                _ => match self.pick(model, hit, Target::Nodes, precision) {
                    Items::Nodes(nodes) => nodes,
                    Items::Faces(_) | Items::Edges(_) | Items::Geometry(_) => BTreeSet::new(),
                },
            };
            return Items::Faces(outline_edges_within(model, Some(hit.part), &nodes));
        }
        let face = hit.face;
        let angle = self.angle.to_radians();
        let faces: Vec<usize> = match self.select_by {
            SelectBy::Geometry => {
                if target == Target::Surfaces
                    && let Some(edge) = picker.edge_near(face, hit.point, precision)
                {
                    // A click near a free edge of a shell picks the edge, up to its corners.
                    let nodes = picker.edge_chain(edge, Some(OUTLINE_ANGLE.to_radians()));
                    let edges = outline_edges_within(model, Some(hit.part), &nodes);
                    if !edges.is_empty() {
                        return Items::Faces(edges);
                    }
                }
                if target == Target::Nodes {
                    if let Some(node) = picker.corner_near(hit.point, precision) {
                        return Items::Nodes(picker.node_ids([node]));
                    }
                    if let Some(edge) = picker.edge_near(face, hit.point, precision) {
                        // The outline of a 2D model is one loop; its edges end where it bends.
                        let angle = model.is_plane().then(|| OUTLINE_ANGLE.to_radians());
                        return Items::Nodes(picker.edge_chain(edge, angle));
                    }
                }
                picker.patch(face)
            }
            SelectBy::GeometryPart | SelectBy::Part => (0..picker.faces().len()).collect(),
            SelectBy::GeometryEdgeAngle | SelectBy::EdgeAngle | SelectBy::Edge => {
                let limit = (self.select_by != SelectBy::Edge).then_some(angle);
                let nodes = match picker.nearest_edge(hit.point) {
                    Some(edge) => picker.edge_chain(edge, limit),
                    None => BTreeSet::new(),
                };
                if target == Target::Surfaces {
                    // The edges of shells along the picked edge.
                    return Items::Faces(outline_edges_within(model, Some(hit.part), &nodes));
                }
                return Items::Nodes(nodes);
            }
            SelectBy::GeometrySurfaceAngle | SelectBy::FaceAngle => {
                picker.faces_by_angle(face, angle)
            }
            SelectBy::Node => return Items::Nodes(BTreeSet::from([model.hit_node(hit)])),
            SelectBy::Element => {
                let element = picker.faces()[face].element;
                if target == Target::Nodes {
                    let element = &model.mesh.elements()[element];
                    return Items::Nodes(element.nodes.iter().copied().collect());
                }
                (0..picker.faces().len())
                    .filter(|&f| picker.faces()[f].element == element)
                    .collect()
            }
            SelectBy::Surface => vec![face],
            SelectBy::Id => return picker.empty(target),
        };
        match target {
            Target::Faces | Target::Surfaces | Target::Edges => {
                Items::Faces(faces.iter().map(|&f| picker.face_id(f)).collect())
            }
            Target::Lines => Items::Edges(BTreeSet::new()),
            Target::Nodes | Target::Point => Items::Nodes(
                picker.node_ids(
                    faces
                        .iter()
                        .flat_map(|&f| picker.face_nodes(f))
                        .collect::<Vec<_>>(),
                ),
            ),
        }
    }
}

impl Picker {
    /// What a click picks as edges of a 3D part: the chain of outline edges nearest to it,
    /// running to the corners or, in the angle modes, on while it bends less than the angle.
    fn pick_edges(&self, model: &Model, hit: &Hit) -> Items {
        if hit.line {
            return Items::Edges(BTreeSet::new());
        }
        let picker = MeshPicker::new(model, hit.part);
        let Some(edge) = picker.nearest_edge(hit.point) else {
            return Items::Edges(BTreeSet::new());
        };
        let chain = match self.select_by {
            SelectBy::GeometryEdgeAngle | SelectBy::EdgeAngle => {
                picker.chain_edges(edge, Some(self.angle.to_radians()))
            }
            _ => picker.geometry_edge(edge),
        };
        Items::Edges(picker.segments(chain))
    }

    /// What a click picks as a point: in the geometry mode the corner point of the geometry
    /// nearest to the click, wherever on the face or edge it is, else the nodes of the mode.
    fn pick_point(&self, model: &Model, hit: &Hit, precision: f32) -> Items {
        if self.select_by != SelectBy::Geometry {
            return self.pick(model, hit, Target::Nodes, precision);
        }
        if model.has_cad()
            && let Some(vertex) = crate::cad_selection::nearest_corner(model, hit)
        {
            return Items::Geometry(BTreeSet::from([CadEntity::Vertex(vertex)]));
        }
        let corner = if hit.line {
            line_end_near(model, hit, f32::INFINITY)
        } else {
            let picker = MeshPicker::new(model, hit.part);
            (picker.corner_of_patch(hit.face, hit.point)).map(|n| model.mesh.node_ids()[n])
        };
        // A part without corners, like a sphere, has no point but its nodes.
        Items::Nodes(BTreeSet::from([
            corner.unwrap_or_else(|| model.hit_node(hit))
        ]))
    }

    /// What a selection box selects: nodes in the node modes, otherwise element faces (whole
    /// elements in the element mode) inside the box, or crossing it when dragged from right
    /// to left. Only visible parts count.
    pub fn pick_box(&self, model: &Model, area: &BoxSelect, target: Target) -> Items {
        self.pick_box_inner(model, area, target)
    }

    /// What a click on a line segment (beam, truss) selects: the nearest node, the nodes of
    /// the element or of the part's lines; lines have no faces. In the geometry modes a
    /// click within `precision` of an end of the part's lines picks the node there, as a
    /// corner point of a solid, else the nodes of all its lines, as an edge.
    fn pick_line(&self, model: &Model, hit: &Hit, target: Target, precision: f32) -> Items {
        if target != Target::Nodes {
            return Items::Faces(BTreeSet::new());
        }
        let skin = model.skin(hit.part);
        let elements = model.mesh.elements();
        let part_nodes = || -> BTreeSet<NodeId> {
            (skin.line_elements.iter())
                .flat_map(|&e| elements[e].nodes.iter().copied())
                .collect()
        };
        let nodes: BTreeSet<NodeId> = match self.select_by {
            SelectBy::Node => BTreeSet::from([model.hit_node(hit)]),
            SelectBy::GeometryPart | SelectBy::Part => part_nodes(),
            SelectBy::Geometry | SelectBy::GeometryEdgeAngle | SelectBy::GeometrySurfaceAngle => {
                match line_end_near(model, hit, precision) {
                    Some(end) => BTreeSet::from([end]),
                    None => part_nodes(),
                }
            }
            _ => (elements[skin.line_elements[hit.face]].nodes.iter())
                .copied()
                .collect(),
        };
        Items::Nodes(nodes)
    }

    fn pick_box_inner(&self, model: &Model, area: &BoxSelect, target: Target) -> Items {
        let mesh = &model.mesh;
        if target == Target::Lines {
            let mut edges = BTreeSet::new();
            for part in (0..model.parts.len()).filter(|&p| model.parts[p].visible) {
                let picker = MeshPicker::new(model, part);
                let inside = (picker.feature_edges()).filter(|(_, edge)| {
                    let mut nodes = [Some(edge.a), Some(edge.b), edge.mid].into_iter().flatten();
                    if area.crossing {
                        nodes.any(|n| area.contains(model.render_position(n)))
                    } else {
                        nodes.all(|n| area.contains(model.render_position(n)))
                    }
                });
                let inside: Vec<usize> = inside.map(|(i, _)| i).collect();
                edges.extend(picker.segments(inside));
            }
            return Items::Edges(edges);
        }
        if target == Target::Edges {
            let edges = (model.outline_edges().into_iter())
                .filter(|(part, _, _)| model.parts[*part].visible)
                .filter(|(_, _, nodes)| {
                    let mut inside = (nodes.iter())
                        .filter_map(|&id| mesh.node_index(id))
                        .map(|n| area.contains(model.render_position(n)));
                    if area.crossing {
                        inside.any(|i| i)
                    } else {
                        inside.all(|i| i)
                    }
                })
                .map(|(_, face, _)| face)
                .collect();
            return Items::Faces(edges);
        }
        let inside: Vec<bool> = (0..mesh.node_count())
            .map(|n| area.contains(model.render_position(n)))
            .collect();
        let taken = |nodes: &mut dyn Iterator<Item = usize>| {
            let mut nodes = nodes.map(|n| inside[n]);
            if area.crossing {
                nodes.any(|inside| inside)
            } else {
                nodes.all(|inside| inside)
            }
        };
        let node_modes = matches!(
            self.select_by,
            SelectBy::Node
                | SelectBy::Edge
                | SelectBy::EdgeAngle
                | SelectBy::GeometryEdgeAngle
                | SelectBy::Id
        );
        if node_modes && target == Target::Nodes {
            let visible = model.visible_nodes();
            return Items::Nodes(
                visible
                    .into_iter()
                    .filter(|&id| mesh.node_index(id).is_some_and(|n| inside[n]))
                    .collect(),
            );
        }
        let mut faces = Vec::new();
        for part in (0..model.parts.len()).filter(|&p| model.parts[p].visible) {
            let picker = MeshPicker::new(model, part);
            if self.select_by == SelectBy::Element {
                let mut elements: Vec<usize> = picker.faces().iter().map(|f| f.element).collect();
                elements.sort_unstable();
                elements.dedup();
                for element in elements {
                    let nodes = &mesh.elements()[element].nodes;
                    let mut indices = nodes.iter().filter_map(|&id| mesh.node_index(id));
                    if taken(&mut indices) {
                        faces.extend(
                            (0..picker.faces().len())
                                .filter(|&f| picker.faces()[f].element == element)
                                .map(|f| (part, f)),
                        );
                    }
                }
            } else {
                for f in 0..picker.faces().len() {
                    if taken(&mut picker.face_nodes(f)) {
                        faces.push((part, f));
                    }
                }
            }
        }
        match target {
            Target::Faces | Target::Surfaces | Target::Edges => Items::Faces(
                faces
                    .iter()
                    .map(|&(part, f)| MeshPicker::new(model, part).face_id(f))
                    .collect(),
            ),
            Target::Lines => Items::Edges(BTreeSet::new()),
            Target::Nodes | Target::Point => {
                let ids = mesh.node_ids();
                Items::Nodes(
                    faces
                        .iter()
                        .flat_map(|&(part, f)| MeshPicker::new(model, part).face_nodes(f))
                        .map(|n| ids[n])
                        .collect(),
                )
            }
        }
    }
}

/// Whole parts picked in the 3D view, for regions of parts and mesh setup items. As for
/// other selections a click replaces the picked parts, Shift adds and Ctrl removes.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PartPicks {
    history: History<String>,
}

impl PartPicks {
    pub fn from_names(names: impl IntoIterator<Item = String>) -> Self {
        Self {
            history: History::from_items(names),
        }
    }

    pub fn names(&self) -> BTreeSet<String> {
        self.history.items()
    }

    pub fn is_empty(&self) -> bool {
        self.names().is_empty()
    }

    pub fn can_undo(&self) -> bool {
        self.history.can_undo()
    }

    pub fn clear(&mut self) {
        self.history.clear();
    }

    /// A click on the part of the hit, or into empty space.
    pub fn click(&mut self, model: &Model, hit: Option<&Hit>, operation: Operation) {
        match hit.and_then(|hit| model.parts.get(hit.part)) {
            Some(part) => (self.history).push(operation, BTreeSet::from([part.name.clone()])),
            // Nodalis clears the selection on a plain click into empty space.
            None if operation == Operation::Replace => self.clear(),
            None => {}
        }
    }

    /// The visible parts inside a selection box, or crossing it when dragged to the left.
    pub fn box_select(&mut self, model: &Model, area: &BoxSelect, operation: Operation) {
        let names = (0..model.parts.len())
            .filter(|&p| model.parts[p].visible && part_in_box(model, p, area))
            .map(|p| model.parts[p].name.clone())
            .collect();
        self.history.push(operation, names);
    }

    /// Applies a button of the selection window.
    pub fn action(&mut self, model: &Model, action: PickerAction) {
        let visible = || {
            (model.parts.iter())
                .filter(|p| p.visible)
                .map(|p| p.name.clone())
        };
        match action {
            PickerAction::Undo => self.history.undo(),
            PickerAction::Clear => self.clear(),
            PickerAction::All => self.history.push(Operation::Replace, visible().collect()),
            PickerAction::Invert => {
                let picked = self.names();
                let others = visible().filter(|n| !picked.contains(n)).collect();
                self.history.push(Operation::Replace, others);
            }
            PickerAction::Ids(..) => {}
        }
    }

    /// The outline of the part under the mouse.
    pub fn preview(model: &Model, hit: &Hit) -> Preview {
        let target = if hit.line {
            Target::Nodes
        } else if model.is_plane() {
            Target::Edges
        } else {
            Target::Faces
        };
        let picker = Picker {
            select_by: SelectBy::Part,
            ..Picker::default()
        };
        let mut preview = preview(model, &picker.pick(model, hit, target, 0.0));
        if hit.line {
            // A line part shows as its lines; points on every node would hide them.
            preview.points.clear();
        }
        preview
    }

    /// The row of the dialog: the "..." button, the picked parts and a button to clear
    /// them. `active` tells whether clicks in the 3D view pick parts for this row; returns
    /// whether the user wants them to.
    pub fn ui(&mut self, ui: &mut egui::Ui, active: bool) -> bool {
        let mut wanted = false;
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                wanted = crate::setup::pick_button(ui, active);
                let names: Vec<String> = self.names().into_iter().collect();
                let label = match names.len() {
                    0 => "Empty".to_string(),
                    1..=3 => names.join(", "),
                    n => format!("{n} Parts"),
                };
                let response = ui.label(label);
                if names.len() > 3 {
                    response.on_hover_text(names.join("\n"));
                }
                if ui.button("Clear Selection").clicked() {
                    self.clear();
                }
            });
            ui.weak("Click parts in the 3D view.");
        });
        wanted
    }
}

/// Whether the part lies inside the box, or crosses it for a box dragged to the left.
fn part_in_box(model: &Model, part: usize, area: &BoxSelect) -> bool {
    let mesh = &model.mesh;
    let picker = MeshPicker::new(model, part);
    let elements = mesh.elements();
    let mut nodes: Vec<usize> = (0..picker.faces().len())
        .flat_map(|f| picker.face_nodes(f))
        .collect();
    nodes.extend(
        (model.skin(part).line_elements.iter())
            .flat_map(|&e| elements[e].nodes.iter())
            .filter_map(|&id| mesh.node_index(id)),
    );
    if nodes.is_empty() {
        return false;
    }
    let mut inside = nodes
        .into_iter()
        .map(|n| area.contains(model.render_position(n)));
    if area.crossing {
        inside.any(|i| i)
    } else {
        inside.all(|i| i)
    }
}

/// The hover preview of picked items: outlines of faces and points of nodes.
pub fn preview(model: &Model, items: &Items) -> Preview {
    let mut preview = Preview::default();
    match items {
        Items::Geometry(entities) => return crate::cad_selection::preview(model, entities),
        Items::Nodes(nodes) => {
            preview.points = nodes
                .iter()
                .filter_map(|&id| model.mesh.node_index(id))
                .map(|n| model.render_position(n))
                .collect();
            // Segments of line elements between two of the nodes, so a picked line shows.
            preview.lines = (model.line_segments())
                .filter(|(_, line)| line.iter().all(|n| nodes.contains(n)))
                .filter_map(|(_, [a, b])| {
                    let position = |id| Some(model.render_position(model.mesh.node_index(id)?));
                    Some([position(a)?, position(b)?])
                })
                .collect();
        }
        Items::Edges(edges) => {
            preview.lines = (edges.iter())
                .filter_map(|&[a, b]| {
                    let position = |id| Some(model.render_position(model.mesh.node_index(id)?));
                    Some([position(a)?, position(b)?])
                })
                .collect();
        }
        Items::Faces(faces) if model.is_plane() => {
            preview.lines = edge_lines(model, faces)
                .into_iter()
                .filter_map(|[a, b]| {
                    let position = |id| Some(model.render_position(model.mesh.node_index(id)?));
                    Some([position(a)?, position(b)?])
                })
                .collect();
        }
        Items::Faces(faces) => {
            // An edge between two picked faces is drawn once.
            let mut edges = HashSet::new();
            for (part, skin) in (0..model.parts.len()).map(|p| (p, model.skin(p))) {
                let picker = MeshPicker::new(model, part);
                for (index, face) in skin.faces.iter().enumerate() {
                    if faces.contains(&picker.face_id(index)) {
                        edges
                            .extend(corner_edges(&face.corners).map(|(a, b)| (a.min(b), a.max(b))));
                    }
                }
            }
            preview.lines = (edges.into_iter())
                .map(|(a, b)| [model.render_position(a), model.render_position(b)])
                .collect();
            // Edges of shells, which have no face of their own on the skin.
            let shell_edges: BTreeSet<(ElementId, u8)> = (faces.iter())
                .filter(|&&(e, face)| {
                    face >= 3 && model.mesh.element(e).is_some_and(|e| e.is_shell())
                })
                .copied()
                .collect();
            preview
                .lines
                .extend(
                    edge_lines(model, &shell_edges)
                        .into_iter()
                        .filter_map(|[a, b]| {
                            let position =
                                |id| Some(model.render_position(model.mesh.node_index(id)?));
                            Some([position(a)?, position(b)?])
                        }),
                );
        }
    }
    preview
}

/// The mesh segments along CAD edges, the smaller node first.
pub fn cad_segments(mesh: &plx_mesh::FeMesh, entities: &[CadEntity]) -> BTreeSet<[NodeId; 2]> {
    (entities.iter())
        .filter_map(|e| match e {
            CadEntity::Edge(tag) => mesh.cad.segments.get(tag),
            _ => None,
        })
        .flatten()
        .map(|&[a, b]| [a.min(b), a.max(b)])
        .collect()
}

/// All outline edges of the visible parts, for the selection of edges of 3D parts.
pub fn visible_segments(model: &Model) -> BTreeSet<[NodeId; 2]> {
    (0..model.parts.len())
        .filter(|&p| model.parts[p].visible)
        .flat_map(|p| {
            let picker = MeshPicker::new(model, p);
            let edges: Vec<usize> = picker.feature_edges().map(|(i, _)| i).collect();
            picker.segments(edges)
        })
        .collect()
}

/// How far the outline may bend at a node and still count as one edge when the geometry
/// based mode picks edges of a 2D model, the feature angle of the display.
const OUTLINE_ANGLE: f64 = 30.0;

/// Edges of the outline, of one part or of all, whose nodes are all among `nodes`.
pub fn outline_edges_within(
    model: &Model,
    part: Option<usize>,
    nodes: &BTreeSet<NodeId>,
) -> BTreeSet<(ElementId, u8)> {
    (model.outline_edges().into_iter())
        .filter(|(p, _, _)| part.is_none_or(|part| part == *p))
        .filter(|(_, _, edge)| edge.iter().all(|n| nodes.contains(n)))
        .map(|(_, face, _)| face)
        .collect()
}

/// The edges of 2D elements as lines between nodes, split at the midside node.
pub fn edge_lines(model: &Model, faces: &BTreeSet<(ElementId, u8)>) -> Vec<[NodeId; 2]> {
    let mut lines = Vec::new();
    for &(element, face) in faces {
        let Some(element) = model.mesh.element(element) else {
            continue;
        };
        let Some(edge) = (usize::from(face).checked_sub(1)).and_then(|f| element.faces().get(f))
        else {
            continue;
        };
        let [a, b] = [edge.corners[0], edge.corners[1]].map(|l| element.nodes[l]);
        match edge.mids.first().filter(|_| element.shape.is_quadratic()) {
            Some(&mid) => {
                let m = element.nodes[mid];
                lines.extend([[a, m], [m, b]]);
            }
            None => lines.push([a, b]),
        }
    }
    lines
}

/// `1, 5, 10-20` as a list of ids.
fn parse_ids(text: &str) -> Result<Vec<u32>, String> {
    let mut ids = Vec::new();
    for token in text.split([',', ' ', ';']).filter(|t| !t.trim().is_empty()) {
        let invalid = || format!("Not a valid ID: {token}");
        match token.split_once('-') {
            Some((from, to)) => {
                let from: u32 = from.trim().parse().map_err(|_| invalid())?;
                let to: u32 = to.trim().parse().map_err(|_| invalid())?;
                if to < from || to - from > 10_000_000 {
                    return Err(invalid());
                }
                ids.extend(from..=to);
            }
            None => ids.push(token.trim().parse().map_err(|_| invalid())?),
        }
    }
    Ok(ids)
}

/// Geometric queries on the skin of one part.
struct MeshPicker<'a> {
    model: &'a Model,
    part: usize,
}

impl<'a> MeshPicker<'a> {
    fn new(model: &'a Model, part: usize) -> Self {
        Self { model, part }
    }

    fn faces(&self) -> &'a [SkinFace] {
        &self.model.skin(self.part).faces
    }

    fn empty(&self, target: Target) -> Items {
        match target {
            Target::Nodes | Target::Point => Items::Nodes(BTreeSet::new()),
            Target::Faces | Target::Surfaces | Target::Edges => Items::Faces(BTreeSet::new()),
            Target::Lines => Items::Edges(BTreeSet::new()),
        }
    }

    /// Skin edges by the ids of their end nodes, the smaller first.
    fn segments(&self, edges: impl IntoIterator<Item = usize>) -> BTreeSet<[NodeId; 2]> {
        let ids = self.model.mesh.node_ids();
        let skin = &self.model.skin(self.part).edges;
        (edges.into_iter())
            .map(|e| {
                let (a, b) = (ids[skin[e].a], ids[skin[e].b]);
                [a.min(b), a.max(b)]
            })
            .collect()
    }

    fn face_id(&self, face: usize) -> (ElementId, u8) {
        let face = &self.faces()[face];
        (
            self.model.mesh.elements()[face.element].id,
            face.face as u8 + 1,
        )
    }

    fn face_nodes(&self, face: usize) -> impl Iterator<Item = usize> + use<'a> {
        let face = &self.faces()[face];
        face.corners.iter().chain(&face.mids).copied()
    }

    fn node_ids(&self, nodes: impl IntoIterator<Item = usize>) -> BTreeSet<NodeId> {
        let ids = self.model.mesh.node_ids();
        nodes.into_iter().map(|n| ids[n]).collect()
    }

    fn position(&self, node: usize) -> Vec3 {
        self.model.render_position(node)
    }

    fn normal(&self, face: usize) -> DVec3 {
        DVec3::from(face_normal(
            self.model.mesh.coords(),
            &self.faces()[face].corners,
        ))
        .normalize_or_zero()
    }

    /// The smooth patch around a face, bounded by feature edges.
    fn patch(&self, face: usize) -> Vec<usize> {
        let region = self.faces()[face].region;
        (0..self.faces().len())
            .filter(|&f| self.faces()[f].region == region)
            .collect()
    }

    /// Faces reached from `start` across edges where neighbouring faces fold less than `angle`.
    fn faces_by_angle(&self, start: usize, angle: f64) -> Vec<usize> {
        let faces = self.faces();
        let mut by_edge: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
        for (index, face) in faces.iter().enumerate() {
            for (a, b) in corner_edges(&face.corners) {
                by_edge.entry((a.min(b), a.max(b))).or_default().push(index);
            }
        }
        let cos_limit = angle.cos();
        let mut seen = vec![false; faces.len()];
        seen[start] = true;
        let mut stack = vec![start];
        while let Some(face) = stack.pop() {
            let normal = self.normal(face);
            for (a, b) in corner_edges(&faces[face].corners) {
                for &next in &by_edge[&(a.min(b), a.max(b))] {
                    if !seen[next] && normal.dot(self.normal(next)) >= cos_limit {
                        seen[next] = true;
                        stack.push(next);
                    }
                }
            }
        }
        (0..faces.len()).filter(|&f| seen[f]).collect()
    }

    fn feature_edges(&self) -> impl Iterator<Item = (usize, &'a plx_mesh::SkinEdge)> + 'a {
        self.model
            .skin(self.part)
            .edges
            .iter()
            .enumerate()
            .filter(|(_, e)| e.feature)
    }

    /// Number of feature edges at each node they touch.
    fn feature_degree(&self) -> HashMap<usize, usize> {
        let mut degree = HashMap::new();
        for (_, edge) in self.feature_edges() {
            *degree.entry(edge.a).or_insert(0) += 1;
            *degree.entry(edge.b).or_insert(0) += 1;
        }
        degree
    }

    /// A corner of the outline near the point.
    fn corner_near(&self, point: Vec3, precision: f32) -> Option<usize> {
        (self.corners().into_iter())
            .map(|node| (node, self.position(node).distance(point)))
            .filter(|&(_, distance)| distance <= precision)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(node, _)| node)
    }

    /// The corner of the outline nearest to the point among those of the patch of `face`,
    /// or of the whole part if the patch has none.
    fn corner_of_patch(&self, face: usize, point: Vec3) -> Option<usize> {
        let on_patch: HashSet<usize> = (self.patch(face).into_iter())
            .flat_map(|f| self.faces()[f].corners.clone())
            .collect();
        let corners = self.corners();
        let nearest = |nodes: &mut dyn Iterator<Item = usize>| {
            nodes
                .map(|node| (node, self.position(node).distance(point)))
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map(|(node, _)| node)
        };
        nearest(&mut corners.iter().copied().filter(|n| on_patch.contains(n)))
            .or_else(|| nearest(&mut corners.iter().copied()))
    }

    /// The corners of the outline: where other than two feature edges meet, or where the
    /// outline bends, like at the corners of a 2D model or of a shell.
    fn corners(&self) -> Vec<usize> {
        let bends = self.outline_bends();
        self.feature_degree()
            .into_iter()
            .filter(|&(node, degree)| degree != 2 || bends.contains(&node))
            .map(|(node, _)| node)
            .collect()
    }

    /// Nodes where two feature edges meet at more than [`OUTLINE_ANGLE`], the corners of a
    /// 2D model's outline.
    fn outline_bends(&self) -> HashSet<usize> {
        let mut at_node: HashMap<usize, Vec<usize>> = HashMap::new();
        for (_, edge) in self.feature_edges() {
            at_node.entry(edge.a).or_default().push(edge.b);
            at_node.entry(edge.b).or_default().push(edge.a);
        }
        let limit = OUTLINE_ANGLE.to_radians().cos() as f32;
        at_node
            .into_iter()
            .filter(|(node, others)| {
                let [a, b] = others[..] else {
                    return false;
                };
                let here = self.position(*node);
                let incoming = (here - self.position(a)).normalize_or_zero();
                let outgoing = (self.position(b) - here).normalize_or_zero();
                incoming.dot(outgoing) < limit
            })
            .map(|(node, _)| node)
            .collect()
    }

    fn edge_distance(&self, edge: &plx_mesh::SkinEdge, point: Vec3) -> f32 {
        let (a, b) = (self.position(edge.a), self.position(edge.b));
        let ab = b - a;
        let t = ((point - a).dot(ab) / ab.length_squared().max(f32::MIN_POSITIVE)).clamp(0.0, 1.0);
        (a + ab * t).distance(point)
    }

    /// A feature edge of the hit face near the point.
    fn edge_near(&self, face: usize, point: Vec3, precision: f32) -> Option<usize> {
        let corners = &self.faces()[face].corners;
        self.feature_edges()
            .filter(|(_, e)| corners.contains(&e.a) && corners.contains(&e.b))
            .map(|(i, e)| (i, self.edge_distance(e, point)))
            .filter(|&(_, distance)| distance <= precision)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
    }

    /// The feature edge of the part nearest to the point.
    fn nearest_edge(&self, point: Vec3) -> Option<usize> {
        self.feature_edges()
            .map(|(i, e)| (i, self.edge_distance(e, point)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
    }

    /// Nodes of the chain of feature edges through `start`. Without an angle the chain runs
    /// to the corners of the outline; with one it continues wherever the next edge bends less
    /// than the angle.
    fn edge_chain(&self, start: usize, angle: Option<f64>) -> BTreeSet<NodeId> {
        let edges = &self.model.skin(self.part).edges;
        let nodes = self.chain_edges(start, angle).into_iter().flat_map(|e| {
            let edge = &edges[e];
            [Some(edge.a), Some(edge.b), edge.mid].into_iter().flatten()
        });
        self.node_ids(nodes.collect::<Vec<_>>())
    }

    /// The feature edges of the chain through `start`, see [`MeshPicker::edge_chain`].
    fn chain_edges(&self, start: usize, angle: Option<f64>) -> BTreeSet<usize> {
        self.chain_edges_with(start, angle, false)
    }

    /// The feature edges of the chain through `start` as an edge of the geometry: on to
    /// where other edges meet it or it bends by more than [`OUTLINE_ANGLE`], as at the
    /// corners of a flat shell, whose outline is one loop.
    fn geometry_edge(&self, start: usize) -> BTreeSet<usize> {
        self.chain_edges_with(start, Some(OUTLINE_ANGLE.to_radians()), true)
    }

    fn chain_edges_with(
        &self,
        start: usize,
        angle: Option<f64>,
        junctions: bool,
    ) -> BTreeSet<usize> {
        let edges = &self.model.skin(self.part).edges;
        let mut at_node: HashMap<usize, Vec<usize>> = HashMap::new();
        for (index, edge) in self.feature_edges() {
            at_node.entry(edge.a).or_default().push(index);
            at_node.entry(edge.b).or_default().push(index);
        }
        let direction = |edge: usize, from: usize| {
            let e = &edges[edge];
            let to = if e.a == from { e.b } else { e.a };
            (self.position(to) - self.position(from)).normalize_or_zero()
        };
        let cos_limit = angle.map(|a| a.cos() as f32);
        let mut seen = BTreeSet::from([start]);
        let mut stack = vec![start];
        while let Some(edge) = stack.pop() {
            for node in [edges[edge].a, edges[edge].b] {
                let others = &at_node[&node];
                let incoming = -direction(edge, node);
                for &next in others.iter().filter(|&&n| n != edge) {
                    let follow = match cos_limit {
                        // Continue straight on: the next edge leaves in about the direction the
                        // current one arrived.
                        Some(limit) => {
                            incoming.dot(direction(next, node)) >= limit
                                && (!junctions || others.len() == 2)
                        }
                        None => others.len() == 2,
                    };
                    if follow && seen.insert(next) {
                        stack.push(next);
                    }
                }
            }
        }
        seen
    }
}

/// The segments of the outline edge nearest to `point` on a part, from corner to corner as
/// the geometry based selection picks an edge; split at midside nodes.
pub fn edge_segments(model: &Model, part: usize, point: Vec3) -> Vec<[NodeId; 2]> {
    let picker = MeshPicker::new(model, part);
    let Some(start) = picker.nearest_edge(point) else {
        return Vec::new();
    };
    let ids = model.mesh.node_ids();
    let edges = &model.skin(part).edges;
    (picker.chain_edges(start, None).into_iter())
        .flat_map(|e| {
            let edge = &edges[e];
            match edge.mid {
                Some(m) => vec![[ids[edge.a], ids[m]], [ids[m], ids[edge.b]]],
                None => vec![[ids[edge.a], ids[edge.b]]],
            }
        })
        .collect()
}

fn corner_edges(corners: &[usize]) -> impl Iterator<Item = (usize, usize)> + '_ {
    (0..corners.len()).map(|i| (corners[i], corners[(i + 1) % corners.len()]))
}

/// The end of the hit part's lines within `precision` of the click: a node only one of
/// its line elements uses, like a corner point of a solid.
fn line_end_near(model: &Model, hit: &Hit, precision: f32) -> Option<NodeId> {
    let skin = model.skin(hit.part);
    let elements = model.mesh.elements();
    let mut uses: BTreeMap<NodeId, usize> = BTreeMap::new();
    // The skin lists an element once per segment it is drawn with.
    let line_elements: BTreeSet<usize> = skin.line_elements.iter().copied().collect();
    for e in line_elements {
        let nodes = &elements[e].nodes;
        for &n in [nodes[0], nodes[nodes.len() - 1]].iter() {
            *uses.entry(n).or_default() += 1;
        }
    }
    (uses.into_iter())
        .filter(|&(_, count)| count == 1)
        .filter_map(|(n, _)| {
            let position = model.render_position(model.mesh.node_index(n)?);
            Some((n, position.distance(hit.point)))
        })
        .filter(|&(_, d)| d <= precision)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(n, _)| n)
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_model::Region;

    #[test]
    fn history_replays_operations() {
        let mut history = History::default();
        history.push(Operation::Replace, BTreeSet::from([1, 2, 3]));
        history.push(Operation::Add, BTreeSet::from([4]));
        history.push(Operation::Subtract, BTreeSet::from([2]));
        assert_eq!(history.items(), BTreeSet::from([1, 3, 4]));
        history.push(Operation::Intersect, BTreeSet::from([3, 4, 5]));
        assert_eq!(history.items(), BTreeSet::from([3, 4]));
        history.undo();
        assert_eq!(history.items(), BTreeSet::from([1, 3, 4]));
        history.push(Operation::Replace, BTreeSet::from([9]));
        assert_eq!(history.items(), BTreeSet::from([9]));
        history.undo();
        assert!(history.items().is_empty(), "replace starts a new history");
    }

    #[test]
    fn modifiers_choose_the_operation() {
        assert_eq!(Operation::from_modifiers(false, false), Operation::Replace);
        assert_eq!(Operation::from_modifiers(true, false), Operation::Add);
        assert_eq!(Operation::from_modifiers(false, true), Operation::Subtract);
        assert_eq!(Operation::from_modifiers(true, true), Operation::Intersect);
    }

    fn testdata(name: &str) -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name)
    }

    /// The 100 x 10 x 10 cantilever of 10 x 2 x 2 hexahedra, hit from above at x = 95, y = 5,
    /// plus a tolerance of 0.5 mm.
    fn beam_hit(x: f32, y: f32) -> (Model, Hit) {
        let model = crate::model::load(
            &testdata("kragbalken_c3d8.inp"),
            plx_model::UnitSystem::Unitless,
        )
        .unwrap()
        .model;
        // Render coordinates are relative to the model centre.
        let p = model.mesh.coords()[0];
        let first = Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32);
        let origin = Vec3::new(x, y, 50.0) - first + model.render_position(0);
        let hit = model.pick(origin, Vec3::NEG_Z).unwrap();
        (model, hit)
    }

    fn picker(select_by: SelectBy) -> Picker {
        Picker {
            select_by,
            ..Picker::default()
        }
    }

    fn faces(items: Items) -> BTreeSet<(ElementId, u8)> {
        match items {
            Items::Faces(faces) => faces,
            other => panic!("{other:?} instead of faces"),
        }
    }

    fn nodes(items: Items) -> BTreeSet<NodeId> {
        match items {
            Items::Nodes(nodes) => nodes,
            other => panic!("{other:?} instead of nodes"),
        }
    }

    #[test]
    fn part_picks_take_whole_parts_like_any_selection() {
        let (model, hit) = beam_hit(95.0, 5.0);
        let name = model.parts[0].name.clone();
        let mut parts = PartPicks::default();
        parts.click(&model, Some(&hit), Operation::Replace);
        assert_eq!(parts.names(), BTreeSet::from([name.clone()]));
        // Ctrl removes, a plain click into empty space clears.
        parts.click(&model, Some(&hit), Operation::Subtract);
        assert!(parts.is_empty());
        parts.click(&model, Some(&hit), Operation::Add);
        parts.click(&model, None, Operation::Add);
        assert!(!parts.is_empty(), "only a plain click clears");
        parts.click(&model, None, Operation::Replace);
        assert!(parts.is_empty());
        // The buttons of the selection window.
        parts.action(&model, PickerAction::All);
        assert_eq!(parts.names().len(), 1);
        parts.action(&model, PickerAction::Invert);
        assert!(parts.is_empty());
        parts.click(&model, Some(&hit), Operation::Add);
        parts.action(&model, PickerAction::Undo);
        assert!(parts.is_empty());
        // The preview outlines the whole part.
        assert!(!PartPicks::preview(&model, &hit).lines.is_empty());

        // A box over half of the beam takes it only when dragged to the left.
        let area = |crossing| BoxSelect {
            view_proj: glam::Mat4::from_translation(Vec3::new(0.0, 0.0, 0.5))
                * glam::Mat4::from_scale(Vec3::new(1.0 / 60.0, 1.0 / 60.0, -0.005)),
            min: glam::Vec2::new(-0.0001, -1.0),
            max: glam::Vec2::new(1.0, 1.0),
            crossing,
            shift: false,
            ctrl: false,
        };
        let mut parts = PartPicks::default();
        parts.box_select(&model, &area(false), Operation::Replace);
        assert!(parts.is_empty());
        parts.box_select(&model, &area(true), Operation::Replace);
        assert_eq!(parts.names(), BTreeSet::from([name]));
    }

    #[test]
    fn modes_pick_faces_of_the_beam() {
        let (model, hit) = beam_hit(95.0, 5.0);
        let pick = |mode| faces(picker(mode).pick(&model, &hit, Target::Faces, 0.5));
        let single = pick(SelectBy::Surface);
        assert_eq!(single.len(), 1);
        assert_eq!(single.first().unwrap().1, 2, "top face of a hex is S2");
        // The top of the beam is one patch of 10 x 2 element faces.
        assert_eq!(pick(SelectBy::Geometry).len(), 20);
        assert_eq!(pick(SelectBy::FaceAngle).len(), 20);
        assert_eq!(
            pick(SelectBy::Element).len(),
            3,
            "top, end and side face of the element"
        );
        // 4 x 10 x 2 long sides and 2 x 2 x 2 ends.
        assert_eq!(pick(SelectBy::Part).len(), 88);
    }

    #[test]
    fn modes_pick_nodes_of_the_beam() {
        let pick = |x, y, mode| {
            let (model, hit) = beam_hit(x, y);
            let nodes = nodes(picker(mode).pick(&model, &hit, Target::Nodes, 0.5));
            let coords: Vec<[f64; 3]> =
                nodes.iter().map(|&n| model.mesh.node(n).unwrap()).collect();
            coords
        };
        // Inside a face the geometry mode takes the whole top: 11 x 3 nodes.
        assert_eq!(pick(55.0, 5.0, SelectBy::Geometry).len(), 33);
        // Near the long top edge at y = 0 it takes that edge.
        let edge = pick(55.0, 0.3, SelectBy::Geometry);
        assert_eq!(edge.len(), 11);
        assert!(edge.iter().all(|p| p[1] == 0.0 && p[2] == 10.0));
        // Near the corner only the corner.
        assert_eq!(
            pick(99.8, 0.2, SelectBy::Geometry),
            vec![[100.0, 0.0, 10.0]]
        );
        // Edge modes run to the corners; with an angle they stop at the 90° bends too.
        assert_eq!(pick(55.0, 2.0, SelectBy::Edge).len(), 11);
        assert_eq!(pick(55.0, 2.0, SelectBy::EdgeAngle).len(), 11);
        let mut wide = picker(SelectBy::EdgeAngle);
        wide.angle = 95.0;
        let (model, hit) = beam_hit(55.0, 2.0);
        let outline = nodes(wide.pick(&model, &hit, Target::Nodes, 0.5));
        assert!(
            outline.len() > 11,
            "a 95° limit follows the outline around the corners"
        );
    }

    #[test]
    fn edges_of_a_solid_are_picked_as_element_edges() {
        let (model, hit) = beam_hit(55.0, 2.0);
        let edges = |picker: &Picker| match picker.pick(&model, &hit, Target::Lines, 0.5) {
            Items::Edges(edges) => edges,
            other => panic!("{other:?} instead of edges"),
        };
        // The long top edge at y = 0 from corner to corner: ten element edges.
        let top = edges(&picker(SelectBy::Geometry));
        assert_eq!(top.len(), 10);
        for [a, b] in &top {
            let (p, q) = (model.mesh.node(*a).unwrap(), model.mesh.node(*b).unwrap());
            assert!(p[1] == 0.0 && q[1] == 0.0 && p[2] == 10.0 && q[2] == 10.0);
        }
        assert_eq!(edges(&picker(SelectBy::EdgeAngle)), top);
        // Faces and nodes are no edges.
        assert!(!SelectBy::Surface.allowed(Target::Lines));
        assert!(!SelectBy::Node.allowed(Target::Lines));
        // The region takes the edges and spreads a load over all of them by length.
        let region = Region::Edges(top.into_iter().collect());
        let lines = region.edges(&model.mesh);
        assert_eq!(lines.len(), 10);
    }

    /// The side of a flat shell ends at the corners of the plate, although its outline is
    /// one smooth loop of feature edges.
    #[test]
    fn an_edge_of_a_flat_shell_runs_from_corner_to_corner() {
        use plx_mesh::{Element, ElementShape, FeMesh, Part};
        let mut mesh = FeMesh::default();
        let id = |i: u32, j: u32| j * 11 + i + 1;
        for j in 0..=6 {
            for i in 0..=10 {
                mesh.set_node(id(i, j), [10.0 * f64::from(i), 10.0 * f64::from(j), 0.0]);
            }
        }
        let mut elements = Vec::new();
        for j in 0..6 {
            for i in 0..10 {
                let element = j * 10 + i + 1;
                mesh.add_element(Element {
                    id: element,
                    type_name: "S4".into(),
                    shape: ElementShape::Quad4,
                    nodes: vec![id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)],
                })
                .unwrap();
                elements.push(element);
            }
        }
        mesh.parts.push(Part {
            name: "PLATE".into(),
            elements,
        });
        let model = Model::new(std::path::Path::new("platte.inp"), mesh);
        let p = model.mesh.coords()[0];
        let first = Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32);
        let origin = Vec3::new(55.0, 1.0, 50.0) - first + model.render_position(0);
        let hit = model.pick(origin, Vec3::NEG_Z).unwrap();
        let Items::Edges(edges) = picker(SelectBy::Geometry).pick(&model, &hit, Target::Lines, 0.5)
        else {
            panic!("no edges");
        };
        assert_eq!(edges.len(), 10);
        assert!(
            edges
                .iter()
                .flatten()
                .all(|&n| model.mesh.node(n).unwrap()[1] == 0.0)
        );
    }

    /// A beam of two quadratic lines along x from 0 to 100, hit by looking down at x.
    fn line_hit(x: f32) -> (Model, Hit) {
        use plx_mesh::{Element, ElementShape, FeMesh, Part};
        let mut mesh = FeMesh::default();
        for i in 0..5u32 {
            mesh.set_node(i + 1, [25.0 * f64::from(i), 0.0, 0.0]);
        }
        for (id, nodes) in [(1, vec![1, 2, 3]), (2, vec![3, 4, 5])] {
            mesh.add_element(Element {
                id,
                type_name: "B32".into(),
                shape: ElementShape::Line3,
                nodes,
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "BEAM".into(),
            elements: vec![1, 2],
        });
        let model = Model::new(std::path::Path::new("balken.inp"), mesh);
        // Render coordinates are relative to the model centre, node 1 is at the origin.
        let origin = Vec3::new(x, 0.0, 50.0) + model.render_position(0);
        let (_, hit) = model.pick_line(origin, Vec3::NEG_Z, |_| 1.0).unwrap();
        (model, hit)
    }

    #[test]
    fn geometry_mode_picks_the_ends_of_lines_as_points() {
        let pick = |x, mode| {
            let (model, hit) = line_hit(x);
            nodes(picker(mode).pick(&model, &hit, Target::Nodes, 1.0))
        };
        // Within the tolerance of an end the geometry mode takes the node there, as a
        // corner point; elsewhere all nodes of the part's lines, as an edge.
        assert_eq!(pick(99.5, SelectBy::Geometry), BTreeSet::from([5]));
        assert_eq!(pick(0.5, SelectBy::Geometry), BTreeSet::from([1]));
        assert_eq!(
            pick(50.5, SelectBy::Geometry),
            BTreeSet::from([1, 2, 3, 4, 5])
        );
        assert_eq!(pick(60.0, SelectBy::Geometry).len(), 5);
        assert_eq!(pick(60.0, SelectBy::Node), BTreeSet::from([3]));
        assert_eq!(pick(60.0, SelectBy::Element), BTreeSet::from([3, 4, 5]));
        assert_eq!(pick(60.0, SelectBy::Part).len(), 5);
    }

    #[test]
    fn boxes_take_items_inside_or_crossing() {
        let (model, _) = beam_hit(55.0, 5.0);
        // Seen from above, the beam spans -50..50 in x around the model centre; the box
        // covers the half with x >= 0.
        let area = |crossing| BoxSelect {
            // Orthographic view from above: x and y scaled into -1..1, depth around 0.5.
            view_proj: glam::Mat4::from_translation(Vec3::new(0.0, 0.0, 0.5))
                * glam::Mat4::from_scale(Vec3::new(1.0 / 60.0, 1.0 / 60.0, -0.005)),
            min: glam::Vec2::new(-0.0001, -1.0),
            max: glam::Vec2::new(1.0, 1.0),
            crossing,
            shift: false,
            ctrl: false,
        };
        let pick = |mode, crossing, target| picker(mode).pick_box(&model, &area(crossing), target);
        // 5 elements along x times 2 on each of the 4 long sides, plus the 4 end faces.
        assert_eq!(
            faces(pick(SelectBy::Geometry, false, Target::Faces)).len(),
            44
        );
        // Crossing also takes the faces of the elements touching x = 0.
        assert_eq!(
            faces(pick(SelectBy::Geometry, true, Target::Faces)).len(),
            52
        );
        // 6 cross sections of 3 x 3 nodes.
        assert_eq!(nodes(pick(SelectBy::Node, false, Target::Nodes)).len(), 54);
    }

    #[test]
    fn ids_and_ranges_are_parsed() {
        assert_eq!(parse_ids("1, 5 10-12"), Ok(vec![1, 5, 10, 11, 12]));
        assert!(parse_ids("1, x").is_err());
        assert!(parse_ids("5-2").is_err());
    }

    /// A 40 x 20 plate of 4 x 2 plane stress elements in the x-y plane, hit from +z at
    /// (x, y).
    fn plate_hit(x: f32, y: f32) -> (Model, Hit) {
        plate_of_hit("CPS4", x, y)
    }

    /// A plate of 4 x 2 elements of the type, 40 x 20 in the x-y plane, hit from above.
    fn plate_of_hit(type_name: &str, x: f32, y: f32) -> (Model, Hit) {
        let mut mesh = plx_mesh::FeMesh::default();
        let id = |i: u32, j: u32| j * 5 + i + 1;
        for j in 0..=2 {
            for i in 0..=4 {
                mesh.set_node(id(i, j), [f64::from(i) * 10.0, f64::from(j) * 10.0, 0.0]);
            }
        }
        let mut part = plx_mesh::Part {
            name: "PLATE".into(),
            elements: Vec::new(),
        };
        for j in 0..2 {
            for i in 0..4 {
                let element = j * 4 + i + 1;
                mesh.add_element(plx_mesh::Element {
                    id: element,
                    type_name: type_name.into(),
                    shape: plx_mesh::ElementShape::Quad4,
                    nodes: vec![id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)],
                })
                .unwrap();
                part.elements.push(element);
            }
        }
        mesh.parts.push(part);
        let model = Model::new(std::path::Path::new("platte.inp"), mesh);
        let origin = Vec3::new(x, y, 50.0) + model.global_origin();
        let hit = model.pick(origin, Vec3::NEG_Z).unwrap();
        (model, hit)
    }

    #[test]
    fn two_d_loads_pick_edges_of_the_outline() {
        let (model, hit) = plate_hit(1.0, 8.0);
        let pick = |mode| faces(picker(mode).pick(&model, &hit, Target::Edges, 2.0));
        // The left side, edge 4 of the two elements there.
        assert_eq!(pick(SelectBy::Geometry), BTreeSet::from([(1, 4), (5, 4)]));
        assert_eq!(pick(SelectBy::Part).len(), 12);
        // The corner element has two edges on the outline.
        assert_eq!(pick(SelectBy::Element), BTreeSet::from([(1, 1), (1, 4)]));
        // Boundary conditions take the nodes of the left side only, not the whole outline.
        let left = nodes(picker(SelectBy::Geometry).pick(&model, &hit, Target::Nodes, 2.0));
        assert_eq!(left, BTreeSet::from([1, 6, 11]));
        let (model, corner) = plate_hit(0.5, 0.5);
        let picked = picker(SelectBy::Geometry).pick(&model, &corner, Target::Nodes, 2.0);
        assert_eq!(nodes(picked), BTreeSet::from([1]));
        let preview = preview(&model, &Items::Faces(BTreeSet::from([(1, 4), (5, 4)])));
        assert_eq!(preview.lines.len(), 2);
    }

    #[test]
    fn points_snap_to_the_nearest_corner_in_the_geometry_mode() {
        // Inside an element of the 2D plate, far from any corner of the outline: the point
        // is the nearest corner, not the edge or the face.
        let point = |x, y, mode| {
            let (model, hit) = plate_hit(x, y);
            nodes(picker(mode).pick(&model, &hit, Target::Point, 2.0))
        };
        assert_eq!(point(12.0, 8.0, SelectBy::Geometry), BTreeSet::from([1]));
        assert_eq!(point(35.0, 15.0, SelectBy::Geometry), BTreeSet::from([15]));
        assert_eq!(point(1.0, 8.0, SelectBy::Geometry), BTreeSet::from([1]));
        // The mesh based modes keep picking nodes.
        assert_eq!(point(12.0, 8.0, SelectBy::Node), BTreeSet::from([7]));
        // On a line the nearer end.
        let (model, hit) = line_hit(60.0);
        let picked = picker(SelectBy::Geometry).pick(&model, &hit, Target::Point, 1.0);
        assert_eq!(nodes(picked), BTreeSet::from([5]));
    }

    #[test]
    fn points_on_a_solid_are_corners_of_the_hit_face() {
        let (model, hit) = beam_hit(55.0, 5.0);
        let picked = nodes(picker(SelectBy::Geometry).pick(&model, &hit, Target::Point, 0.5));
        assert_eq!(picked.len(), 1);
        let position = model.mesh.node(*picked.first().unwrap()).unwrap();
        // A corner of the top face the click hit.
        let top = (model.mesh.coords().iter())
            .map(|p| p[2])
            .fold(f64::MIN, f64::max);
        assert_eq!((position[0], position[2]), (100.0, top), "{position:?}");
    }

    #[test]
    fn contact_surfaces_pick_edges_of_shells_or_their_faces() {
        // Near the bottom edge of the shell plate the click picks the edge S3 of the bottom
        // row of shells, in the middle the face.
        let (model, hit) = plate_of_hit("S4", 15.0, 1.0);
        let pick = |mode, hit: &Hit| faces(picker(mode).pick(&model, hit, Target::Surfaces, 2.0));
        let bottom = BTreeSet::from([(1, 3), (2, 3), (3, 3), (4, 3)]);
        assert_eq!(pick(SelectBy::Geometry, &hit), bottom);
        assert_eq!(pick(SelectBy::EdgeAngle, &hit), bottom);
        // Without an angle the edge runs on around the flat plate's outline.
        assert_eq!(pick(SelectBy::Edge, &hit).len(), 12);
        let above = Vec3::new(15.0, 6.0, 50.0) + model.global_origin();
        let middle = model.pick(above, Vec3::NEG_Z).unwrap();
        assert_eq!(pick(SelectBy::Geometry, &middle).len(), 8);
        assert!(SelectBy::Edge.allowed(Target::Surfaces));
        assert!(!SelectBy::Edge.allowed(Target::Faces));
    }

    #[test]
    fn points_on_a_shell_are_its_corners() {
        // A 20 x 20 shell plate in the x-y plane with two beams below: all nodes of its
        // outline join two edges, the corners are where it bends.
        let model = crate::model::load(
            &testdata("platte_mit_stuetzen.inp"),
            plx_model::UnitSystem::Unitless,
        )
        .unwrap()
        .model;
        let origin = Vec3::new(13.0, 16.0, 50.0) + model.global_origin();
        let hit = model.pick(origin, Vec3::NEG_Z).unwrap();
        let picked = picker(SelectBy::Geometry).pick(&model, &hit, Target::Point, 0.5);
        assert_eq!(nodes(picked), BTreeSet::from([25]));
    }
}
