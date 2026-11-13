//! The Query tool (Query on the View and Evaluate tabs): picks in the 3D view write what was
//! selected to the output pane, as aligned lines, and leave an annotation in the view.
//! Nodes, elements, edges, surfaces and parts are picked with the selection window, like
//! every region of the model: a plain click starts a query, Shift adds to it and Ctrl removes
//! from it, a box selects many at once. Distances, angles and circles need two or three
//! points, picked one after another in the mode of the selection window. The annotations
//! stay when the window is closed; a right click on one deletes it, Remove Annotations
//! deletes them all.

use std::collections::{BTreeSet, HashMap};

use glam::{DVec3, Vec3};
use plx_mesh::{CadEntity, ElementId, NodeId};
use plx_model::{Quantity, Region};

use crate::model::{Hit, Model};
use crate::overlay::Marker;
use crate::results::{field_unit, format_value};
use crate::selection::{GeometryRegion, Items, Operation, Picker, PickerAction, Target};
use crate::setup::{RegionDraft, Source};
use crate::viewport::{BoxSelect, Preview};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QueryKind {
    Node,
    Element,
    Edge,
    Surface,
    Part,
    Assembly,
    BoundingBox,
    Distance,
    Angle,
    Circle,
}

const KINDS: [(QueryKind, &str); 10] = [
    (QueryKind::Node, "Vertex/Node"),
    (QueryKind::Element, "Facet/Element"),
    (QueryKind::Edge, "Edge"),
    (QueryKind::Surface, "Surface"),
    (QueryKind::Part, "Part"),
    (QueryKind::Assembly, "Assembly"),
    (QueryKind::BoundingBox, "Bounding box size"),
    (QueryKind::Distance, "Distance"),
    (QueryKind::Angle, "Angle"),
    (QueryKind::Circle, "Circle"),
];

impl QueryKind {
    /// How many nodes the query needs before it has something to say; 0 for queries that
    /// pick something else or nothing.
    fn nodes_needed(self) -> usize {
        match self {
            QueryKind::Node => 1,
            QueryKind::Distance => 2,
            QueryKind::Angle | QueryKind::Circle => 3,
            _ => 0,
        }
    }
}

/// What a query left in the 3D view. Nodes are kept by id, so the annotation follows the
/// shown deformation and stays where it belongs when another increment is shown.
#[derive(Clone, Debug, PartialEq)]
pub enum Annotation {
    /// The nodes of one query, which has one label.
    Nodes(Vec<NodeId>),
    /// Elements by index.
    Elements(Vec<usize>),
    /// The segments along the picked edges.
    Edge {
        segments: Vec<[NodeId; 2]>,
    },
    /// The faces of the picked surfaces by their corner nodes.
    Surface {
        faces: Vec<Vec<NodeId>>,
    },
    /// Parts by index.
    Parts(Vec<usize>),
    Distance([NodeId; 2]),
    Angle([NodeId; 3]),
    Circle([NodeId; 3]),
    BoundingBox,
}

impl From<&Annotation> for plx_results::saved_view::SavedAnnotation {
    fn from(annotation: &Annotation) -> Self {
        use plx_results::saved_view::SavedAnnotation as Saved;
        match *annotation {
            Annotation::Nodes(ref nodes) => Saved::Nodes(nodes.clone()),
            Annotation::Elements(ref elements) => Saved::Elements(elements.clone()),
            Annotation::Edge { ref segments } => Saved::Edge {
                segments: segments.clone(),
            },
            Annotation::Surface { ref faces } => Saved::Surface {
                faces: faces.clone(),
            },
            Annotation::Parts(ref parts) => Saved::Parts(parts.clone()),
            Annotation::Distance(nodes) => Saved::Distance(nodes),
            Annotation::Angle(nodes) => Saved::Angle(nodes),
            Annotation::Circle(nodes) => Saved::Circle(nodes),
            Annotation::BoundingBox => Saved::BoundingBox,
        }
    }
}

impl From<plx_results::saved_view::SavedAnnotation> for Annotation {
    fn from(saved: plx_results::saved_view::SavedAnnotation) -> Self {
        use plx_results::saved_view::SavedAnnotation as Saved;
        match saved {
            Saved::Nodes(nodes) => Annotation::Nodes(nodes),
            Saved::Elements(elements) => Annotation::Elements(elements),
            Saved::Edge { segments } => Annotation::Edge { segments },
            Saved::Surface { faces } => Annotation::Surface { faces },
            Saved::Parts(parts) => Annotation::Parts(parts),
            Saved::Distance(nodes) => Annotation::Distance(nodes),
            Saved::Angle(nodes) => Annotation::Angle(nodes),
            Saved::Circle(nodes) => Annotation::Circle(nodes),
            Saved::BoundingBox => Annotation::BoundingBox,
        }
    }
}

/// What the annotations draw: labelled points, lines and highlighted points.
#[derive(Default)]
pub struct QueryMarks {
    pub markers: Vec<Marker>,
    /// The annotation each marker belongs to, as index into the annotations shown.
    pub owners: Vec<usize>,
    pub lines: Vec<Vec<Vec3>>,
    pub points: Vec<Vec3>,
}

pub enum QueryAction {
    Open,
    /// The overlay has to be rebuilt: a query changed what the view shows.
    Changed,
    /// Remove Annotations: all of the shown model.
    RemoveAll,
    Close,
}

/// The sources of the selection of a query: picks in the 3D view, or whole parts.
const PICKS: &[Source] = &[Source::Selection];
const PARTS: &[Source] = &[Source::Parts];

pub struct QueryWindow {
    kind: QueryKind,
    /// What a click selects, as in every other dialog that picks in the 3D view.
    picker: Picker,
    /// The selection of a node, element, edge, surface or part query, picked with the
    /// selection window: a plain click starts a new query, Shift adds, Ctrl removes.
    selection: RegionDraft,
    /// The annotation of the selection, until a plain click starts the next query.
    live: Option<Annotation>,
    /// Points picked so far for a distance, angle or circle, in order.
    picked: Vec<NodeId>,
}

impl Default for QueryWindow {
    fn default() -> Self {
        Self {
            kind: QueryKind::Node,
            picker: Picker::default(),
            selection: draft(QueryKind::Node),
            live: None,
            picked: Vec::new(),
        }
    }
}

/// The empty selection of a query.
fn draft(kind: QueryKind) -> RegionDraft {
    match kind {
        QueryKind::Part => RegionDraft::new(PARTS, Target::Faces),
        _ => RegionDraft::new(PICKS, kind.target()),
    }
}

impl QueryKind {
    /// What the picks of the query select.
    fn target(self) -> Target {
        match self {
            QueryKind::Node => Target::Nodes,
            QueryKind::Edge => Target::Lines,
            QueryKind::Distance | QueryKind::Angle | QueryKind::Circle => Target::Point,
            _ => Target::Faces,
        }
    }

    /// Whether the query picks in the 3D view, with the selection window open.
    fn picks(self) -> bool {
        !matches!(self, QueryKind::Assembly | QueryKind::BoundingBox)
    }

    /// Whether it picks points one after another instead of a selection.
    fn points(self) -> bool {
        self.nodes_needed() > 1
    }
}

impl QueryWindow {
    /// The window with the list of queries and, for those that pick in the 3D view, the
    /// selection window beside it. Choosing Assembly or Bounding box writes the values at
    /// once.
    pub fn show(
        &mut self,
        ctx: &egui::Context,
        mut model: Option<&mut Model>,
        output: &mut Vec<String>,
    ) -> QueryAction {
        let mut action = QueryAction::Open;
        let mut open = true;
        let any = model
            .as_ref()
            .is_some_and(|m| !m.query_annotations.is_empty())
            || !self.picked.is_empty();
        let window = egui::Window::new("Query")
            .id(crate::ids::window(ctx, "query"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::LEFT_TOP)
            .default_pos(ctx.content_rect().left_top() + egui::vec2(300.0, 90.0))
            .show(ctx, |ui| {
                let before = self.kind;
                for (kind, label) in KINDS {
                    ui.selectable_value(&mut self.kind, kind, label);
                }
                if self.kind != before {
                    self.restart();
                    if let Some(model) = model.as_deref_mut() {
                        match self.kind {
                            QueryKind::Assembly => {
                                output.extend(assembly_lines(model));
                                action = QueryAction::Changed;
                            }
                            QueryKind::BoundingBox => {
                                output.extend(bounding_box_lines(model));
                                model.query_annotations.push(Annotation::BoundingBox);
                                action = QueryAction::Changed;
                            }
                            _ if !self.picked.is_empty() => action = QueryAction::Changed,
                            _ => {}
                        }
                    }
                }
                ui.add_space(4.0);
                ui.weak(self.hint());
                let mut remove = false;
                let footer = crate::dialog::tool_footer_with(ui, None, |ui| {
                    let clear = egui::Button::new("Remove Annotations");
                    remove = ui.add_enabled(any, clear).clicked();
                });
                if remove {
                    self.restart();
                    action = QueryAction::RemoveAll;
                }
                if footer == crate::dialog::Tool::Close {
                    action = QueryAction::Close;
                }
            });
        if let (Some(window), Some(model)) = (window, model)
            && self.kind.picks()
        {
            let anchor = window.response.rect;
            if let Some(picked) = self.picker_window(ctx, anchor, model) {
                self.action(model, picked, output);
                action = QueryAction::Changed;
            }
        }
        if !open || ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            action = QueryAction::Close;
        }
        action
    }

    fn hint(&self) -> String {
        match self.kind {
            QueryKind::Assembly | QueryKind::BoundingBox => {
                "The values are written to the output pane.".to_string()
            }
            QueryKind::Distance | QueryKind::Angle | QueryKind::Circle => format!(
                "Pick {} points in the 3D view ({} picked).",
                self.kind.nodes_needed(),
                self.picked.len()
            ),
            _ => "Pick in the 3D view; Shift adds to the query, Ctrl removes.".to_string(),
        }
    }

    /// Forgets the selection and the points picked so far; the annotations stay.
    fn restart(&mut self) {
        self.selection = draft(self.kind);
        self.live = None;
        self.picked.clear();
    }

    /// Whether edges and surfaces are picked as CAD entities of the Geometry tab, with the
    /// geometry based modes of the selection window.
    fn picks_geometry(&self, model: &Model) -> bool {
        model.is_geometry() && matches!(self.kind, QueryKind::Edge | QueryKind::Surface)
    }

    fn picker_window(
        &mut self,
        ctx: &egui::Context,
        anchor: egui::Rect,
        model: &Model,
    ) -> Option<PickerAction> {
        if self.kind.points() {
            return self
                .picker
                .window(ctx, anchor, Target::Point, !self.picked.is_empty());
        }
        let can_undo = self.selection.can_undo();
        match self.kind {
            QueryKind::Part => self.picker.parts_window(ctx, anchor, can_undo),
            QueryKind::Edge if self.picks_geometry(model) => {
                let region = GeometryRegion::FacesAndEdges;
                self.picker.geometry_window(ctx, anchor, region, can_undo)
            }
            QueryKind::Surface if self.picks_geometry(model) => {
                let region = GeometryRegion::Faces;
                self.picker.geometry_window(ctx, anchor, region, can_undo)
            }
            kind => self.picker.window(ctx, anchor, kind.target(), can_undo),
        }
    }

    /// A button of the selection window.
    fn action(&mut self, model: &mut Model, action: PickerAction, output: &mut Vec<String>) {
        if self.kind.points() {
            match action {
                PickerAction::Undo => {
                    self.picked.pop();
                }
                PickerAction::Clear => self.picked.clear(),
                PickerAction::Ids(Operation::Add, ids) => {
                    for id in ids {
                        if model.mesh.node_index(id).is_some() {
                            self.add_point(model, id, output);
                        }
                    }
                }
                // All, Invert and removing ids say nothing about the order of points.
                _ => {}
            }
            return;
        }
        if self.picks_geometry(model) {
            match action {
                PickerAction::All | PickerAction::Invert if self.kind == QueryKind::Surface => {
                    let mut faces = crate::geometry_selection::visible_faces(model);
                    if action == PickerAction::Invert
                        && let Region::Geometry(picked) = self.selection.region()
                    {
                        faces.retain(|f| !picked.contains(f));
                    }
                    let items = Items::Geometry(faces);
                    self.selection.take(&model.mesh, items, Operation::Replace);
                }
                PickerAction::Undo | PickerAction::Clear => self.selection.action(model, action),
                // The buttons that work on nodes and faces of the mesh have nothing to do on
                // the CAD entities of the Geometry tab.
                _ => return,
            }
        } else {
            self.selection.action(model, action);
        }
        self.update(model, false, output);
    }

    /// A click in the 3D view: picks for the query and writes what it found to the output
    /// pane. Returns whether the overlay has to be rebuilt. `precision` is the pick tolerance
    /// at the hit in render units.
    pub fn click(
        &mut self,
        model: &mut Model,
        hit: Option<&Hit>,
        precision: f32,
        operation: Operation,
        output: &mut Vec<String>,
    ) -> bool {
        // A click into empty space keeps the query; Clear in the selection window ends it.
        let Some(hit) = hit else {
            return false;
        };
        if !self.kind.picks() {
            return false;
        }
        if self.kind.points() {
            let Some(node) = self.point(model, hit, precision) else {
                return false;
            };
            self.add_point(model, node, output);
            return true;
        }
        let items = self.pick(model, hit, precision);
        match items {
            Some(items) => self.selection.take(&model.mesh, items, operation),
            None => {
                let pick = Some((hit, precision));
                self.selection.click(model, &self.picker, pick, operation);
            }
        }
        self.update(model, operation == Operation::Replace, output);
        true
    }

    /// A selection box dragged in the 3D view; returns whether the overlay has to be rebuilt.
    pub fn box_select(
        &mut self,
        model: &mut Model,
        area: &BoxSelect,
        operation: Operation,
        output: &mut Vec<String>,
    ) -> bool {
        if !self.kind.picks() || self.kind.points() {
            return false;
        }
        if self.picks_geometry(model) {
            if self.kind == QueryKind::Edge {
                return false;
            }
            let items = Items::Geometry(crate::geometry_selection::pick_box(model, area));
            self.selection.take(&model.mesh, items, operation);
        } else {
            self.selection
                .box_select(model, &self.picker, area, operation);
        }
        self.update(model, operation == Operation::Replace, output);
        true
    }

    /// What a click picks where the selection draft does not know how: the CAD entities of
    /// the Geometry tab, and the lines of a beam part as an edge, which have no outline
    /// edges to pick.
    fn pick(&self, model: &Model, hit: &Hit, precision: f32) -> Option<Items> {
        if self.picks_geometry(model) {
            let region = match self.kind {
                QueryKind::Edge => GeometryRegion::FacesAndEdges,
                _ => GeometryRegion::Faces,
            };
            let mut entities =
                crate::geometry_selection::pick(model, &self.picker, region, hit, precision);
            if self.kind == QueryKind::Edge {
                // A click on a face takes its edge nearest to the click.
                entities.retain(|e| matches!(e, CadEntity::Edge(_)));
                if entities.is_empty()
                    && let Some(edge) = crate::geometry_selection::edge_at(model, hit)
                {
                    entities.insert(CadEntity::Edge(edge));
                }
            }
            return Some(Items::Geometry(entities));
        }
        // Line elements have no faces to pick; the hit segment is its element.
        if self.kind == QueryKind::Element && hit.line {
            let element = model.hit_element(hit)?;
            let id = model.mesh.elements().get(element)?.id;
            return Some(Items::Faces(BTreeSet::from([(id, 1)])));
        }
        let key = |[a, b]: [NodeId; 2]| [a.min(b), a.max(b)];
        if self.kind == QueryKind::Edge && hit.line {
            let segments = pick_edge(model, hit, precision);
            return Some(Items::Edges(segments.into_iter().map(key).collect()));
        }
        if self.kind == QueryKind::Edge
            && let Items::Geometry(entities) =
                (self.picker).pick(model, hit, Target::Lines, precision)
        {
            // A CAD edge the mesh has no segments for, as on results of a mesh from
            // geometry, is taken along the mesh's own edges.
            let entities: Vec<CadEntity> = entities.into_iter().collect();
            if crate::selection::cad_segments(&model.mesh, &entities).is_empty() {
                let segments = crate::selection::edge_segments(model, hit.part, hit.point);
                return Some(Items::Edges(segments.into_iter().map(key).collect()));
            }
        }
        None
    }

    /// The point a click picks for a distance, angle or circle: the node the selection
    /// window's mode selects, or of several the one nearest to the click.
    fn point(&self, model: &Model, hit: &Hit, precision: f32) -> Option<NodeId> {
        let items = (self.picker)
            .pick(model, hit, Target::Point, precision)
            .resolved(&model.mesh, Target::Point);
        let Items::Nodes(nodes) = items else {
            return None;
        };
        let distance = |id: &NodeId| {
            let index = model.mesh.node_index(*id);
            index
                .and_then(|i| model.node_position(i))
                .map_or(f32::INFINITY, |p| p.distance(hit.point))
        };
        nodes
            .into_iter()
            .min_by(|a, b| distance(a).total_cmp(&distance(b)))
    }

    /// Adds a point; with enough of them the query is complete and leaves its annotation.
    fn add_point(&mut self, model: &mut Model, node: NodeId, output: &mut Vec<String>) {
        self.picked.push(node);
        if self.picked.len() < self.kind.nodes_needed() {
            return;
        }
        let picked = std::mem::take(&mut self.picked);
        let annotation = match self.kind {
            QueryKind::Distance => {
                let nodes = [picked[0], picked[1]];
                output.extend(distance_lines(model, nodes));
                Annotation::Distance(nodes)
            }
            QueryKind::Angle => {
                let nodes = [picked[0], picked[1], picked[2]];
                output.extend(angle_lines(model, nodes));
                Annotation::Angle(nodes)
            }
            _ => {
                let nodes = [picked[0], picked[1], picked[2]];
                output.extend(circle_lines(model, nodes));
                Annotation::Circle(nodes)
            }
        };
        model.query_annotations.push(annotation);
    }

    /// Writes the query of the selection to the output pane and shows its annotation in
    /// place of the one before, or as a new one when `new` (a plain click) starts a query.
    fn update(&mut self, model: &mut Model, new: bool, output: &mut Vec<String>) {
        let annotation = self.annotation(model);
        if let Some(annotation) = &annotation {
            output.extend(self.lines(model, annotation));
        }
        let annotations = &mut model.query_annotations;
        let shown =
            (self.live.as_ref()).and_then(|live| annotations.iter().rposition(|a| a == live));
        match (shown, annotation.clone()) {
            // The same pick again does not repeat the annotation.
            (Some(_), Some(a)) if new && self.live.as_ref() == Some(&a) => {}
            (Some(i), Some(a)) if !new => annotations[i] = a,
            (Some(i), None) if !new => {
                annotations.remove(i);
            }
            (_, Some(a)) => annotations.push(a),
            (_, None) => {}
        }
        self.live = annotation;
    }

    /// The annotation of the selection; `None` while nothing is selected.
    fn annotation(&self, model: &Model) -> Option<Annotation> {
        let mesh = &model.mesh;
        let region = self.selection.region();
        let annotation = match self.kind {
            QueryKind::Node => Annotation::Nodes(region.nodes(mesh)),
            QueryKind::Element => {
                let elements: BTreeSet<usize> = (selected_faces(model, &region).into_iter())
                    .filter_map(|(element, _)| mesh.element_index(element))
                    .collect();
                Annotation::Elements(elements.into_iter().collect())
            }
            QueryKind::Edge => Annotation::Edge {
                segments: selected_segments(model, &region),
            },
            QueryKind::Surface => Annotation::Surface {
                faces: face_corners(model, &selected_faces(model, &region)),
            },
            QueryKind::Part => {
                let Region::Parts(names) = region else {
                    return None;
                };
                Annotation::Parts(
                    (0..model.parts.len())
                        .filter(|&p| names.contains(&model.parts[p].name))
                        .collect(),
                )
            }
            _ => return None,
        };
        let empty = match &annotation {
            Annotation::Nodes(nodes) => nodes.is_empty(),
            Annotation::Elements(items) | Annotation::Parts(items) => items.is_empty(),
            Annotation::Edge { segments } => segments.is_empty(),
            Annotation::Surface { faces } => faces.is_empty(),
            _ => false,
        };
        (!empty).then_some(annotation)
    }

    /// The lines the output pane gets for the annotation of a selection.
    fn lines(&self, model: &Model, annotation: &Annotation) -> Vec<String> {
        match annotation {
            Annotation::Nodes(nodes) => match nodes.as_slice() {
                [node] => node_lines(model, *node),
                nodes => nodes_lines(model, nodes),
            },
            Annotation::Elements(elements) => match elements.as_slice() {
                [element] => element_lines(model, *element),
                elements => elements_lines(model, elements),
            },
            Annotation::Edge { segments } => edge_lines(model, segments),
            Annotation::Surface { faces } => surface_lines(model, faces),
            Annotation::Parts(parts) => parts.iter().flat_map(|&p| part_lines(model, p)).collect(),
            _ => Vec::new(),
        }
    }

    /// What a click would select where the mouse rests, as the selection window's mode
    /// selects it.
    pub fn preview(&self, model: &Model, hit: &Hit, precision: f32) -> Preview {
        if !self.kind.picks() {
            return Preview::default();
        }
        if self.kind.points() {
            let Some(node) = self.point(model, hit, precision) else {
                return Preview::default();
            };
            return crate::selection::preview(model, &Items::Nodes(BTreeSet::from([node])));
        }
        if self.kind == QueryKind::Element && hit.line {
            return Preview {
                lines: model.hit_outline(hit),
                points: Vec::new(),
            };
        }
        match self.pick(model, hit, precision) {
            Some(Items::Geometry(entities)) if model.is_geometry() => {
                crate::meshing::entities_preview(model, &entities)
            }
            Some(items) => crate::selection::preview(model, &items),
            None => self.selection.preview(model, &self.picker, hit, precision),
        }
    }

    /// The points picked so far.
    pub fn picked(&self) -> &[NodeId] {
        &self.picked
    }
}

/// The element faces of a selection: picked, or those of CAD faces.
fn selected_faces(model: &Model, region: &Region) -> BTreeSet<(ElementId, u8)> {
    match region {
        Region::Faces(faces) => faces.iter().copied().collect(),
        Region::Geometry(entities) if model.is_geometry() => {
            let tags = (entities.iter())
                .filter_map(|e| match e {
                    CadEntity::Face(tag) => Some(*tag),
                    _ => None,
                })
                .collect();
            model.cad_face_items(&tags).into_iter().collect()
        }
        Region::Geometry(entities) => model.mesh.cad_faces(entities).into_iter().collect(),
        _ => BTreeSet::new(),
    }
}

/// The segments of a selection of edges: picked, or those along CAD edges.
fn selected_segments(model: &Model, region: &Region) -> Vec<[NodeId; 2]> {
    match region {
        Region::Edges(segments) => segments.clone(),
        Region::Geometry(entities) if model.is_geometry() => {
            let tags = (entities.iter())
                .filter_map(|e| match e {
                    CadEntity::Edge(tag) => Some(*tag),
                    _ => None,
                })
                .collect();
            model.cad_edge_segments(&tags)
        }
        Region::Geometry(entities) => {
            (crate::selection::cad_segments(&model.mesh, entities).into_iter()).collect()
        }
        _ => Vec::new(),
    }
}

/// The corner nodes of the faces on the skins of the parts that are among `faces`.
fn face_corners(model: &Model, faces: &BTreeSet<(ElementId, u8)>) -> Vec<Vec<NodeId>> {
    let ids = model.mesh.node_ids();
    let elements = model.mesh.elements();
    (0..model.parts.len())
        .flat_map(|part| model.skin(part).faces.iter())
        .filter(|face| faces.contains(&(elements[face.element].id, face.face as u8 + 1)))
        .map(|face| face.corners.iter().map(|&n| ids[n]).collect())
        .collect()
}

/// The annotations and the nodes picked so far, in render coordinates of the shown model.
pub fn marks(model: &Model, picked: &[NodeId]) -> QueryMarks {
    let mut marks = QueryMarks::default();
    let position = |id: NodeId| model.node_position(model.mesh.node_index(id)?);
    let shown = |id: NodeId| Some(DVec3::from(model.shown_node(id)?));
    // Lengths and areas in true scale, as the output pane has them.
    let true_position = |id: NodeId| {
        let (base, deformed) = base_and_deformed(model, id)?;
        Some(deformed.unwrap_or(base))
    };
    let units = model.units();
    let length = units.unit(Quantity::Length);
    let (length_unit, area_unit) = (length, units.unit(Quantity::Area));
    let s = length_scale(model);
    marks.points = picked.iter().filter_map(|&id| position(id)).collect();
    for (owner, annotation) in model.query_annotations.iter().enumerate() {
        match annotation {
            Annotation::Nodes(nodes) => {
                let points: Vec<(NodeId, Vec3)> = (nodes.iter())
                    .filter_map(|&id| Some((id, position(id)?)))
                    .collect();
                let Some(&(first, p)) = points.first() else {
                    continue;
                };
                marks.points.extend(points.iter().map(|&(_, p)| p));
                let (position, text) = if let [_] = points.as_slice() {
                    let mut text = format!("Node id: {first}");
                    if let Some((value, unit)) = field_value(model, first) {
                        text.push_str(&format!("\nValue: {value} {unit}"));
                    }
                    (p, text)
                } else {
                    let mut text = format!("Nodes: {}", points.len());
                    let ids: Vec<NodeId> = points.iter().map(|&(id, _)| id).collect();
                    if let Some((min, max, unit)) = field_range(model, &ids) {
                        text.push_str(&format!("\nMin: {min} {unit}\nMax: {max} {unit}"));
                    }
                    let all: Vec<Vec3> = points.iter().map(|&(_, p)| p).collect();
                    (nearest_to_middle(&all).unwrap_or(p), text)
                };
                marks.markers.push(Marker { position, text });
            }
            Annotation::Elements(elements) => {
                let mut centers = Vec::new();
                for &index in elements {
                    let outline = model.element_outline(index);
                    centers.extend(centroid(&outline));
                    marks.lines.extend(outline.iter().map(|&[a, b]| vec![a, b]));
                }
                let Some(position) = nearest_to_middle(&centers) else {
                    continue;
                };
                let text = match elements.as_slice() {
                    [index] => {
                        let Some(element) = model.mesh.elements().get(*index) else {
                            continue;
                        };
                        format!("Element id: {}\n{}", element.id, element.type_name)
                    }
                    _ => format!("Elements: {}", centers.len()),
                };
                marks.markers.push(Marker { position, text });
            }
            Annotation::Edge { segments, .. } => {
                let lines = render_segments(model, segments);
                let Some(&[a, b]) = lines.get(lines.len() / 2) else {
                    continue;
                };
                marks.lines.extend(lines.iter().map(|&[a, b]| vec![a, b]));
                let length = edge_length(segments, true_position);
                marks.markers.push(Marker {
                    position: (a + b) * 0.5,
                    text: format!("Length: {} {length_unit}", number(length * s)),
                });
            }
            Annotation::Surface { faces, .. } => {
                let Some(center) = surface_center(model, faces) else {
                    continue;
                };
                let lines = render_segments(model, &outline(faces));
                marks.lines.extend(lines.iter().map(|&[a, b]| vec![a, b]));
                let area = surface_area(faces, true_position);
                marks.markers.push(Marker {
                    position: center,
                    text: format!("Area: {} {area_unit}", number(area * s * s)),
                });
            }
            Annotation::Parts(parts) => {
                for part in parts.iter().filter_map(|&p| model.parts.get(p)) {
                    let Some((min, max)) = part.bounds else {
                        continue;
                    };
                    marks.markers.push(Marker {
                        position: (min + max) * 0.5,
                        text: format!("Part: {}", part.name),
                    });
                }
            }
            Annotation::Distance([a, b]) => {
                let (Some(pa), Some(pb)) = (position(*a), position(*b)) else {
                    continue;
                };
                let distance = match (shown(*a), shown(*b)) {
                    (Some(a), Some(b)) => a.distance(b),
                    _ => continue,
                };
                marks.points.extend([pa, pb]);
                marks.lines.push(vec![pa, pb]);
                marks.markers.push(Marker {
                    position: (pa + pb) * 0.5,
                    text: format!("Distance: {} {length}", number(distance * s)),
                });
            }
            Annotation::Angle([a, b, c]) => {
                let (Some(pa), Some(pb), Some(pc)) = (position(*a), position(*b), position(*c))
                else {
                    continue;
                };
                let angle = match (shown(*a), shown(*b), shown(*c)) {
                    (Some(a), Some(b), Some(c)) => angle_between(a, b, c),
                    _ => continue,
                };
                marks.points.extend([pa, pb, pc]);
                marks.lines.push(vec![pa, pb, pc]);
                marks.markers.push(Marker {
                    position: pb,
                    text: format!("Angle: {} °", number(angle)),
                });
            }
            Annotation::Circle([a, b, c]) => {
                let (Some(pa), Some(pb), Some(pc)) = (position(*a), position(*b), position(*c))
                else {
                    continue;
                };
                let Some((center, radius, axis)) = (match (shown(*a), shown(*b), shown(*c)) {
                    (Some(a), Some(b), Some(c)) => circle_through(a, b, c),
                    _ => None,
                }) else {
                    continue;
                };
                marks.points.extend([pa, pb, pc]);
                marks
                    .lines
                    .push(circle_polyline(model, center, radius, axis));
                marks.markers.push(Marker {
                    position: model.to_render(center.to_array()),
                    text: format!("Radius: {} {length}", number(radius * s)),
                });
            }
            Annotation::BoundingBox => {
                let Some((min, max)) = model.visible_bounds() else {
                    continue;
                };
                let corner = |i: usize| {
                    Vec3::new(
                        if i & 1 == 0 { min.x } else { max.x },
                        if i & 2 == 0 { min.y } else { max.y },
                        if i & 4 == 0 { min.z } else { max.z },
                    )
                };
                for (a, b) in [
                    (0, 1),
                    (1, 3),
                    (3, 2),
                    (2, 0),
                    (4, 5),
                    (5, 7),
                    (7, 6),
                    (6, 4),
                    (0, 4),
                    (1, 5),
                    (2, 6),
                    (3, 7),
                ] {
                    marks.lines.push(vec![corner(a), corner(b)]);
                }
                let size = (max - min).as_dvec3() * s;
                marks.markers.push(Marker {
                    position: (min + max) * 0.5,
                    text: format!(
                        "Size: {} x {} x {} {length}",
                        number(size.x),
                        number(size.y),
                        number(size.z)
                    ),
                });
            }
        }
        marks.owners.resize(marks.markers.len(), owner);
    }
    marks
}

/// A value with six significant digits.
fn number(value: f64) -> String {
    let value: f64 = format!("{value:.5e}").parse().unwrap_or(value);
    if value != 0.0 && !(1e-4..1e7).contains(&value.abs()) {
        format!("{value:e}")
    } else {
        format!("{value}")
    }
}

/// The layout of a query line: the item, its unit in brackets, the names of the
/// values and the values, in columns.
fn row(item: &str, unit: &str, names: &str, values: &[String]) -> String {
    let unit = if unit.is_empty() {
        "[/]".to_string()
    } else {
        format!("[{unit}]")
    };
    let mut line = format!("{item:<16}{unit:>8}{names:>16}");
    for (i, value) in values.iter().enumerate() {
        if i > 0 {
            line.push(',');
        }
        line.push_str(&format!("{value:>16}"));
    }
    line
}

/// Factor from the lengths of the model as stored to those it shows.
fn length_scale(model: &Model) -> f64 {
    (model.stored_units()).factor_to(Quantity::Length, model.units())
}

fn xyz(p: DVec3) -> Vec<String> {
    vec![number(p.x), number(p.y), number(p.z)]
}

/// The value of the shown field at a node and its unit, on the Results tab.
fn field_value(model: &Model, node: NodeId) -> Option<(String, String)> {
    let view = model.results.as_ref()?;
    let (field, component) = view.current()?;
    let index = model.mesh.node_index(node)?;
    let value = *component.values.get(index)?;
    let unit = field_unit(&field.name, &component.name, view.units).unwrap_or_default();
    Some((format_value(value), unit))
}

fn node_lines(model: &Model, node: NodeId) -> Vec<String> {
    let Some(base) = model.mesh.node(node).map(DVec3::from) else {
        return Vec::new();
    };
    let length = model.units().unit(Quantity::Length);
    let base = base * length_scale(model);
    let item = if model.is_geometry() {
        "Vertex"
    } else {
        "Node"
    };
    let mut lines = vec![
        String::new(),
        row(item, "", "id:", &[node.to_string()]),
        row("Base", length, "x, y, z:", &xyz(base)),
    ];
    if let Some(view) = &model.results {
        let displacement = (view.current_increment())
            .and_then(|increment| increment.field("DISP"))
            .and_then(|field| {
                let index = model.mesh.node_index(node)?;
                let component = |name: &str| field.component(name).map_or(0.0, |c| c.values[index]);
                Some(DVec3::new(
                    component("U1") as f64,
                    component("U2") as f64,
                    component("U3") as f64,
                ))
            });
        if let Some(d) = displacement {
            lines.push(row("Deformed", length, "x, y, z:", &xyz(base + d)));
            lines.push(row("Displacement", length, "x, y, z:", &xyz(d)));
        }
        if let Some((value, unit)) = field_value(model, node) {
            let name = view.current().map_or(String::new(), |(field, component)| {
                let shown = view.component_label(&field.name, &component.name);
                format!("{}, {shown}:", field.name)
            });
            lines.push(row("Field value", &unit, &name, &[value]));
        }
    }
    lines.push(String::new());
    lines
}

fn element_lines(model: &Model, index: usize) -> Vec<String> {
    let Some(element) = model.mesh.elements().get(index) else {
        return Vec::new();
    };
    let item = if model.is_geometry() {
        "Facet"
    } else {
        "Element"
    };
    let mut lines = vec![
        String::new(),
        row(item, "", "id:", &[element.id.to_string()]),
    ];
    if !model.is_geometry() {
        lines.push(row(
            "Element type",
            "",
            ":",
            std::slice::from_ref(&element.type_name),
        ));
        let nodes: Vec<String> = element.nodes.iter().map(|n| n.to_string()).collect();
        lines.push(format!("{:<16}{:>8}  {}", "Nodes", "[/]", nodes.join(", ")));
    }
    lines.push(String::new());
    lines
}

/// Several nodes picked together: how many, the centre of their base positions and the
/// smallest and largest value of the shown field among them.
fn nodes_lines(model: &Model, nodes: &[NodeId]) -> Vec<String> {
    let length = model.units().unit(Quantity::Length);
    let s = length_scale(model);
    let item = if model.is_geometry() {
        "Vertices"
    } else {
        "Nodes"
    };
    let bases: Vec<DVec3> = (nodes.iter())
        .filter_map(|&n| base_position(model, n))
        .collect();
    let mut lines = vec![
        String::new(),
        row(item, "", "#:", &[nodes.len().to_string()]),
    ];
    if !bases.is_empty() {
        let center = bases.iter().sum::<DVec3>() / bases.len() as f64 * s;
        lines.push(row("Center", length, "x, y, z:", &xyz(center)));
    }
    if let Some(view) = &model.results
        && let Some((field, component)) = view.current()
        && let Some(extremes) = field_extremes(model, nodes)
    {
        let unit = field_unit(&field.name, &component.name, view.units).unwrap_or_default();
        let shown = view.component_label(&field.name, &component.name);
        let name = format!("{}, {shown}:", field.name);
        let ((min, min_id), (max, max_id)) = extremes;
        lines.push(row("Field min", &unit, &name, &[format_value(min)]));
        lines.push(row("  at node", "", "id:", &[min_id.to_string()]));
        lines.push(row("Field max", &unit, &name, &[format_value(max)]));
        lines.push(row("  at node", "", "id:", &[max_id.to_string()]));
    }
    lines.push(String::new());
    lines
}

/// The smallest and largest value of the shown field at the nodes, with the node of each.
fn field_extremes(model: &Model, nodes: &[NodeId]) -> Option<((f32, NodeId), (f32, NodeId))> {
    let (_, component) = model.results.as_ref()?.current()?;
    let mut extremes: Option<((f32, NodeId), (f32, NodeId))> = None;
    for &node in nodes {
        let Some(&value) = (model.mesh.node_index(node)).and_then(|i| component.values.get(i))
        else {
            continue;
        };
        if !value.is_finite() {
            continue;
        }
        extremes = Some(match extremes {
            None => ((value, node), (value, node)),
            Some((min, max)) => (
                if value < min.0 { (value, node) } else { min },
                if value > max.0 { (value, node) } else { max },
            ),
        });
    }
    extremes
}

/// The range of the shown field at the nodes, formatted, with its unit.
fn field_range(model: &Model, nodes: &[NodeId]) -> Option<(String, String, String)> {
    let view = model.results.as_ref()?;
    let (field, component) = view.current()?;
    let ((min, _), (max, _)) = field_extremes(model, nodes)?;
    let unit = field_unit(&field.name, &component.name, view.units).unwrap_or_default();
    Some((format_value(min), format_value(max), unit))
}

/// Several elements picked together: how many, and of which types.
fn elements_lines(model: &Model, elements: &[usize]) -> Vec<String> {
    let item = if model.is_geometry() {
        "Facets"
    } else {
        "Elements"
    };
    let mut lines = vec![
        String::new(),
        row(item, "", "#:", &[elements.len().to_string()]),
    ];
    if !model.is_geometry() {
        let mut types: Vec<(String, usize)> = Vec::new();
        for element in elements
            .iter()
            .filter_map(|&e| model.mesh.elements().get(e))
        {
            match types.iter_mut().find(|(n, _)| *n == element.type_name) {
                Some((_, count)) => *count += 1,
                None => types.push((element.type_name.clone(), 1)),
            }
        }
        lines.extend(element_type_lines(&types));
    }
    lines.push(String::new());
    lines
}

/// Of the points the one nearest to their middle, where a label lies on what it names.
fn nearest_to_middle(points: &[Vec3]) -> Option<Vec3> {
    let middle = points.iter().sum::<Vec3>() / points.len().max(1) as f32;
    (points.iter().copied()).min_by(|a, b| a.distance(middle).total_cmp(&b.distance(middle)))
}

fn element_type_lines(types: &[(String, usize)]) -> Vec<String> {
    (types.iter())
        .map(|(name, count)| format!("    {name:<24}{count:>10}"))
        .collect()
}

fn part_lines(model: &Model, index: usize) -> Vec<String> {
    let Some(part) = model.parts.get(index) else {
        return Vec::new();
    };
    let length = model.units().unit(Quantity::Length);
    let (elements, nodes) = if model.is_geometry() {
        ("Number of facets:", "Number of vertices:")
    } else {
        ("Number of elements:", "Number of nodes:")
    };
    let mut lines = vec![String::new(), format!("Part name: {}", part.name)];
    if let Some((min, max)) = part.bounds {
        let size = (max - min).as_dvec3() * length_scale(model);
        lines.push(row("Size", length, "x, y, z:", &xyz(size)));
    }
    lines.push(format!("{elements} {}", part.element_count));
    if !model.is_geometry() {
        lines.extend(element_type_lines(&part.element_types));
    }
    lines.push(format!("{nodes} {}", part.node_count));
    lines.push(String::new());
    lines
}

fn assembly_lines(model: &Model) -> Vec<String> {
    let (elements, nodes) = if model.is_geometry() {
        ("Number of facets:", "Number of vertices:")
    } else {
        ("Number of elements:", "Number of nodes:")
    };
    let mut lines = vec![
        String::new(),
        "Assembly".to_string(),
        format!("Number of parts: {}", model.parts.len()),
        format!("{elements} {}", model.mesh.element_count()),
    ];
    if !model.is_geometry() {
        let mut types: Vec<(String, usize)> = Vec::new();
        for (name, count) in model.parts.iter().flat_map(|p| p.element_types.iter()) {
            match types.iter_mut().find(|(n, _)| n == name) {
                Some((_, total)) => *total += count,
                None => types.push((name.clone(), *count)),
            }
        }
        lines.extend(element_type_lines(&types));
    }
    lines.push(format!("{nodes} {}", model.mesh.node_count()));
    lines.push(String::new());
    lines
}

fn bounding_box_lines(model: &Model) -> Vec<String> {
    let Some((min, max)) = model.visible_bounds() else {
        return vec!["Bounding box: nothing is visible".to_string()];
    };
    let s = length_scale(model);
    let (min, max) = (model.model_point(min) * s, model.model_point(max) * s);
    let length = model.units().unit(Quantity::Length);
    let mut lines = vec![String::new(), "Bounding box".to_string()];
    if let Some(view) = &model.results {
        lines.push(row(
            "Def. scale factor",
            "",
            "sf:",
            &[number((view.scale() * view.amplitude()) as f64)],
        ));
    }
    lines.push(row("Min", length, "x, y, z:", &xyz(min)));
    lines.push(row("Max", length, "x, y, z:", &xyz(max)));
    lines.push(row("Center", length, "x, y, z:", &xyz((min + max) * 0.5)));
    lines.push(row("Size", length, "x, y, z:", &xyz(max - min)));
    lines.push(String::new());
    lines
}

/// The base and, with results, the true-scale deformed position of a node.
fn base_and_deformed(model: &Model, node: NodeId) -> Option<(DVec3, Option<DVec3>)> {
    let base = DVec3::from(model.mesh.node(node)?);
    let deformed = model.results.as_ref().and_then(|view| {
        let increment = view.current_increment()?;
        let field = increment.field("DISP")?;
        let index = model.mesh.node_index(node)?;
        let component = |name: &str| field.component(name).map_or(0.0, |c| c.values[index]) as f64;
        Some(base + DVec3::new(component("U1"), component("U2"), component("U3")))
    });
    Some((base, deformed))
}

fn distance_lines(model: &Model, [a, b]: [NodeId; 2]) -> Vec<String> {
    let (Some((base_a, def_a)), Some((base_b, def_b))) =
        (base_and_deformed(model, a), base_and_deformed(model, b))
    else {
        return Vec::new();
    };
    let length = model.units().unit(Quantity::Length);
    let s = length_scale(model);
    let (base_a, base_b) = (base_a * s, base_b * s);
    let (def_a, def_b) = (def_a.map(|d| d * s), def_b.map(|d| d * s));
    let values = |d: DVec3| {
        let mut v = xyz(d);
        v.push(number(d.length()));
        v
    };
    let base = base_b - base_a;
    let mut lines = vec![
        String::new(),
        row("Distance", "", "id1, id2:", &[a.to_string(), b.to_string()]),
        row("Base", length, "dx, dy, dz, D:", &values(base)),
    ];
    if let (Some(def_a), Some(def_b)) = (def_a, def_b) {
        let deformed = def_b - def_a;
        lines.push(row("Deformed", length, "dx, dy, dz, D:", &values(deformed)));
        let mut delta = xyz(deformed - base);
        delta.push(number(deformed.length() - base.length()));
        lines.push(row("Delta", length, "dx, dy, dz, D:", &delta));
    }
    lines.push(String::new());
    lines
}

/// The angle at `b` between the lines to `a` and `c`, in degrees.
fn angle_between(a: DVec3, b: DVec3, c: DVec3) -> f64 {
    let (u, v) = ((a - b).normalize_or_zero(), (c - b).normalize_or_zero());
    u.dot(v).clamp(-1.0, 1.0).acos().to_degrees()
}

fn angle_lines(model: &Model, nodes: [NodeId; 3]) -> Vec<String> {
    let Some(points) = nodes
        .map(|n| base_and_deformed(model, n))
        .into_iter()
        .collect::<Option<Vec<_>>>()
    else {
        return Vec::new();
    };
    let base = angle_between(points[0].0, points[1].0, points[2].0);
    let ids = nodes.map(|n| n.to_string());
    let mut lines = vec![
        String::new(),
        row("Angle", "", "id1, id2, id3:", &ids),
        row("Base", "°", "phi:", &[number(base)]),
    ];
    if let (Some(a), Some(b), Some(c)) = (points[0].1, points[1].1, points[2].1) {
        let deformed = angle_between(a, b, c);
        lines.push(row("Deformed", "°", "phi:", &[number(deformed)]));
        lines.push(row("Delta", "°", "phi:", &[number(deformed - base)]));
    }
    lines.push(String::new());
    lines
}

/// Centre, radius and unit axis of the circle through three points; `None` on a line.
fn circle_through(a: DVec3, b: DVec3, c: DVec3) -> Option<(DVec3, f64, DVec3)> {
    let (u, v) = (b - a, c - a);
    let w = u.cross(v);
    let w2 = w.length_squared();
    if w2 < 1e-24 * (u.length_squared() * v.length_squared()).max(f64::MIN_POSITIVE) {
        return None;
    }
    let center =
        a + (v.length_squared() * w.cross(u) + u.length_squared() * v.cross(w)) / (2.0 * w2);
    Some((center, center.distance(a), w / w2.sqrt()))
}

fn circle_lines(model: &Model, nodes: [NodeId; 3]) -> Vec<String> {
    let Some(points) = nodes
        .map(|n| base_and_deformed(model, n))
        .into_iter()
        .collect::<Option<Vec<_>>>()
    else {
        return Vec::new();
    };
    let length = model.units().unit(Quantity::Length);
    let ids = nodes.map(|n| n.to_string());
    let mut lines = vec![String::new(), row("Circle", "", "id1, id2, id3:", &ids)];
    let describe = |label: &str, axis_label: &str, circle: Option<(DVec3, f64, DVec3)>| match circle
    {
        Some((center, radius, axis)) => {
            let s = length_scale(model);
            let mut values = xyz(center * s);
            values.push(number(radius * s));
            vec![
                row(label, length, "x, y, z, R:", &values),
                row(axis_label, "", "x, y, z:", &xyz(axis)),
            ]
        }
        None => vec![format!("{label}: the nodes lie on a line")],
    };
    lines.extend(describe(
        "Base",
        "Base axis",
        circle_through(points[0].0, points[1].0, points[2].0),
    ));
    if let (Some(a), Some(b), Some(c)) = (points[0].1, points[1].1, points[2].1) {
        lines.extend(describe(
            "Deformed",
            "Deformed axis",
            circle_through(a, b, c),
        ));
    }
    lines.push(String::new());
    lines
}

/// The circle as a closed polyline in render coordinates.
fn circle_polyline(model: &Model, center: DVec3, radius: f64, axis: DVec3) -> Vec<Vec3> {
    let helper = if axis.x.abs() < 0.9 {
        DVec3::X
    } else {
        DVec3::Y
    };
    let u = axis.cross(helper).normalize_or_zero();
    let v = axis.cross(u);
    (0..=72)
        .map(|i| {
            let t = i as f64 * std::f64::consts::TAU / 72.0;
            model.to_render((center + radius * (t.cos() * u + t.sin() * v)).to_array())
        })
        .collect()
}

/// The segments of the edge a click picks, as the geometry based selection does: the CAD
/// edge on a mesh generated from geometry, the CAD edge drawn as lines on the Geometry tab
/// (the nearest one to a click on a face), the lines of a beam part, else the outline edge
/// of the part from corner to corner.
fn pick_edge(model: &Model, hit: &Hit, precision: f32) -> Vec<[NodeId; 2]> {
    if model.has_cad()
        && let Some(CadEntity::Edge(tag)) =
            crate::cad_selection::pick(model, hit, Target::Edges, precision)
    {
        return model
            .mesh
            .cad
            .segments
            .get(&tag)
            .cloned()
            .unwrap_or_default();
    }
    let skin = model.skin(hit.part);
    let segment = if hit.line {
        Some(hit.face)
    } else if model.is_geometry() {
        let distance = |&[a, b]: &[usize; 2]| {
            let (Some(a), Some(b)) = (model.node_position(a), model.node_position(b)) else {
                return f32::INFINITY;
            };
            let ab = b - a;
            let t = ((hit.point - a).dot(ab) / ab.length_squared().max(f32::MIN_POSITIVE))
                .clamp(0.0, 1.0);
            (a + ab * t).distance(hit.point)
        };
        (0..skin.lines.len())
            .min_by(|&i, &j| distance(&skin.lines[i]).total_cmp(&distance(&skin.lines[j])))
    } else {
        None
    };
    let Some(segment) = segment else {
        return crate::selection::edge_segments(model, hit.part, hit.point);
    };
    // A CAD edge is drawn as the line elements of one entity; a beam part as a whole.
    let entity_of = |segment: usize| {
        let element = model.mesh.elements().get(skin.line_elements[segment])?;
        model.cad_entity(element.id)
    };
    let entity = entity_of(segment);
    let ids = model.mesh.node_ids();
    (0..skin.lines.len())
        .filter(|&i| entity.is_none() || entity_of(i) == entity)
        .map(|i| skin.lines[i].map(|n| ids[n]))
        .collect()
}

/// The edges of a surface that only one of its faces has: its outline.
fn outline(faces: &[Vec<NodeId>]) -> Vec<[NodeId; 2]> {
    let edges = |face: &Vec<NodeId>| {
        let face = face.clone();
        (0..face.len()).map(move |i| [face[i], face[(i + 1) % face.len()]])
    };
    let key = |[a, b]: [NodeId; 2]| (a.min(b), a.max(b));
    let mut count: HashMap<(NodeId, NodeId), usize> = HashMap::new();
    for edge in faces.iter().flat_map(edges) {
        *count.entry(key(edge)).or_default() += 1;
    }
    (faces.iter().flat_map(edges))
        .filter(|&edge| count[&key(edge)] == 1)
        .collect()
}

fn render_segments(model: &Model, segments: &[[NodeId; 2]]) -> Vec<[Vec3; 2]> {
    let position = |id: NodeId| model.node_position(model.mesh.node_index(id)?);
    (segments.iter())
        .filter_map(|&[a, b]| Some([position(a)?, position(b)?]))
        .collect()
}

fn edge_length(segments: &[[NodeId; 2]], position: impl Fn(NodeId) -> Option<DVec3>) -> f64 {
    (segments.iter())
        .filter_map(|&[a, b]| Some(position(a)?.distance(position(b)?)))
        .sum()
}

/// The area of the faces, each split into triangles from its first corner.
fn surface_area(faces: &[Vec<NodeId>], position: impl Fn(NodeId) -> Option<DVec3>) -> f64 {
    let mut area = 0.0;
    for face in faces {
        let Some(corners) = face
            .iter()
            .map(|&n| position(n))
            .collect::<Option<Vec<_>>>()
        else {
            continue;
        };
        for i in 1..corners.len().saturating_sub(1) {
            let (u, v) = (corners[i] - corners[0], corners[i + 1] - corners[0]);
            area += 0.5 * u.cross(v).length();
        }
    }
    area
}

/// Where the label of a surface goes: the centre of the face nearest to the middle of the
/// surface, so that it lies on the surface also where it is curved.
fn surface_center(model: &Model, faces: &[Vec<NodeId>]) -> Option<Vec3> {
    let centers: Vec<Vec3> = (faces.iter())
        .filter_map(|face| {
            let corners = (face.iter())
                .map(|&n| model.node_position(model.mesh.node_index(n)?))
                .collect::<Option<Vec<_>>>()?;
            Some(corners.iter().sum::<Vec3>() / corners.len().max(1) as f32)
        })
        .collect();
    let middle = centers.iter().sum::<Vec3>() / centers.len().max(1) as f32;
    (centers.into_iter()).min_by(|a, b| a.distance(middle).total_cmp(&b.distance(middle)))
}

fn base_position(model: &Model, node: NodeId) -> Option<DVec3> {
    Some(base_and_deformed(model, node)?.0)
}

fn deformed_position(model: &Model, node: NodeId) -> Option<DVec3> {
    base_and_deformed(model, node)?.1
}

/// The names of the parts the nodes belong to, joined.
fn part_names(model: &Model, nodes: impl IntoIterator<Item = NodeId>) -> String {
    let parts: BTreeSet<usize> = nodes
        .into_iter()
        .filter_map(|n| model.node_part(n))
        .collect();
    let names: Vec<&str> = (parts.into_iter())
        .filter_map(|p| Some(model.parts.get(p)?.name.as_str()))
        .collect();
    names.join(", ")
}

fn edge_lines(model: &Model, segments: &[[NodeId; 2]]) -> Vec<String> {
    let length = model.units().unit(Quantity::Length);
    let s = length_scale(model);
    let base = edge_length(segments, |n| base_position(model, n)) * s;
    let names = part_names(model, segments.iter().map(|&[a, _]| a));
    let mut lines = vec![
        String::new(),
        format!("Edge on part: {names}"),
        row("Base", length, "L:", &[number(base)]),
    ];
    if model.results.is_some() {
        let deformed = edge_length(segments, |n| deformed_position(model, n)) * s;
        lines.push(row("Deformed", length, "L:", &[number(deformed)]));
        lines.push(row("Delta", length, "L:", &[number(deformed - base)]));
    }
    lines.push(String::new());
    lines
}

fn surface_lines(model: &Model, faces: &[Vec<NodeId>]) -> Vec<String> {
    let unit = model.units().unit(Quantity::Area);
    let s = length_scale(model).powi(2);
    let base = surface_area(faces, |n| base_position(model, n)) * s;
    let names = part_names(model, faces.iter().filter_map(|f| f.first().copied()));
    let mut lines = vec![
        String::new(),
        format!("Surface on part: {names}"),
        row("Base", unit, "A:", &[number(base)]),
    ];
    if model.results.is_some() {
        let deformed = surface_area(faces, |n| deformed_position(model, n)) * s;
        lines.push(row("Deformed", unit, "A:", &[number(deformed)]));
        lines.push(row("Delta", unit, "A:", &[number(deformed - base)]));
    }
    let facets = if model.is_geometry() {
        "Facets"
    } else {
        "Element faces"
    };
    lines.push(row(facets, "", "#:", &[faces.len().to_string()]));
    lines.push(String::new());
    lines
}

fn centroid(lines: &[[Vec3; 2]]) -> Option<Vec3> {
    if lines.is_empty() {
        return None;
    }
    let sum: Vec3 = lines.iter().map(|[a, b]| *a + *b).sum();
    Some(sum / (2 * lines.len()) as f32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn circles_and_angles() {
        let (center, radius, axis) = circle_through(
            DVec3::new(1.0, 0.0, 0.0),
            DVec3::new(0.0, 1.0, 0.0),
            DVec3::new(-1.0, 0.0, 0.0),
        )
        .unwrap();
        assert!(center.length() < 1e-12);
        assert!((radius - 1.0).abs() < 1e-12);
        assert!((axis - DVec3::Z).length() < 1e-12);
        assert!(circle_through(DVec3::ZERO, DVec3::X, 2.0 * DVec3::X).is_none());
        let angle = angle_between(DVec3::X, DVec3::ZERO, DVec3::Y);
        assert!((angle - 90.0).abs() < 1e-12);
    }

    #[test]
    fn surface_area_and_outline() {
        // Two unit squares side by side: area 2, the shared edge is not on the outline.
        let faces = vec![vec![1, 2, 5, 4], vec![2, 3, 6, 5]];
        let position = |n: NodeId| {
            let n = n as f64 - 1.0;
            Some(DVec3::new(n % 3.0, (n / 3.0).floor(), 0.0))
        };
        assert!((surface_area(&faces, position) - 2.0).abs() < 1e-12);
        let edges = outline(&faces);
        assert_eq!(edges.len(), 6);
        assert!(!edges.iter().any(|e| matches!(e, [2, 5] | [5, 2])));
        assert!((edge_length(&edges, position) - 6.0).abs() < 1e-12);
    }

    /// The 100 x 10 x 10 cantilever of 10 x 2 x 2 hexahedra and a hit from above at x, y.
    fn beam_hit(x: f32, y: f32) -> (Model, Hit) {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/kragbalken_c3d8.inp");
        let model = crate::model::load(&path, plx_model::UnitSystem::Unitless)
            .unwrap()
            .model;
        let p = model.mesh.coords()[0];
        let first = Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32);
        let origin = Vec3::new(x, y, 50.0) - first + model.render_position(0);
        let hit = model.pick(origin, Vec3::NEG_Z).unwrap();
        (model, hit)
    }

    fn query(kind: QueryKind) -> QueryWindow {
        QueryWindow {
            kind,
            selection: draft(kind),
            ..QueryWindow::default()
        }
    }

    #[test]
    fn surface_queries_take_the_selection_of_the_picker() {
        let (mut model, hit) = beam_hit(95.0, 5.0);
        let mut output = Vec::new();
        let mut query = query(QueryKind::Surface);
        // The geometry mode picks the whole top of the beam: 100 x 10.
        assert!(query.click(&mut model, Some(&hit), 0.5, Operation::Replace, &mut output));
        let [Annotation::Surface { faces }] = model.query_annotations.as_slice() else {
            panic!("{:?}", model.query_annotations);
        };
        assert_eq!(faces.len(), 20);
        let area = surface_area(faces, |n| base_position(&model, n));
        assert!((area - 1000.0).abs() < 1e-6, "{area}");
        assert!(
            output
                .iter()
                .any(|l| l.starts_with("Element faces") && l.ends_with(" 20"))
        );

        // In the surface mode a plain click starts a new query, Shift adds to it and Ctrl
        // removes from it; the annotation follows.
        query.picker.select_by = crate::selection::SelectBy::Surface;
        query.click(&mut model, Some(&hit), 0.5, Operation::Replace, &mut output);
        assert_eq!(model.query_annotations.len(), 2);
        let (_, other) = beam_hit(85.0, 5.0);
        query.click(&mut model, Some(&other), 0.5, Operation::Add, &mut output);
        let faces = |model: &Model| match model.query_annotations.last() {
            Some(Annotation::Surface { faces }) => faces.len(),
            other => panic!("{other:?}"),
        };
        assert_eq!(model.query_annotations.len(), 2);
        assert_eq!(faces(&model), 2);
        query.click(
            &mut model,
            Some(&hit),
            0.5,
            Operation::Subtract,
            &mut output,
        );
        assert_eq!(faces(&model), 1);
        // The same pick again does not repeat the label; a click into empty space keeps it.
        query.click(
            &mut model,
            Some(&other),
            0.5,
            Operation::Replace,
            &mut output,
        );
        assert!(!query.click(&mut model, None, 0.5, Operation::Replace, &mut output));
        assert_eq!(model.query_annotations.len(), 2);
        // Undo and Clear of the selection window change the query being picked.
        query.click(&mut model, Some(&hit), 0.5, Operation::Add, &mut output);
        assert_eq!(faces(&model), 2);
        query.action(&mut model, PickerAction::Undo, &mut output);
        assert_eq!(faces(&model), 1);
        query.action(&mut model, PickerAction::Clear, &mut output);
        assert_eq!(model.query_annotations.len(), 1, "the first query stays");
    }

    #[test]
    fn edge_queries_take_the_edge_nearest_to_the_click() {
        let (mut model, hit) = beam_hit(95.0, 9.0);
        let mut output = Vec::new();
        let mut edge = query(QueryKind::Edge);
        assert!(!edge.preview(&model, &hit, 0.5).lines.is_empty());
        edge.click(&mut model, Some(&hit), 0.5, Operation::Replace, &mut output);
        let [Annotation::Edge { segments }] = model.query_annotations.as_slice() else {
            panic!("{:?}", model.query_annotations);
        };
        let length = edge_length(segments, |n| base_position(&model, n));
        assert!((length - 100.0).abs() < 1e-6, "{length}");
    }

    #[test]
    fn element_queries_take_the_elements_of_the_picked_faces() {
        let (mut model, hit) = beam_hit(95.0, 5.0);
        let mut output = Vec::new();
        let mut elements = query(QueryKind::Element);
        assert!(!elements.preview(&model, &hit, 0.5).lines.is_empty());
        elements.click(&mut model, Some(&hit), 0.5, Operation::Replace, &mut output);
        assert!(
            matches!(
                model.query_annotations.as_slice(),
                [Annotation::Elements(e)] if e.len() == 20
            ),
            "{:?}",
            model.query_annotations
        );
        elements.picker.select_by = crate::selection::SelectBy::Element;
        elements.click(&mut model, Some(&hit), 0.5, Operation::Replace, &mut output);
        assert!(matches!(
            model.query_annotations.last(),
            Some(Annotation::Elements(e)) if e.len() == 1
        ));
        assert!(output.iter().any(|l| l.starts_with("Element type")));
    }

    #[test]
    fn node_and_point_queries() {
        let (mut model, hit) = beam_hit(95.0, 5.0);
        let mut output = Vec::new();
        // The element mode of a node query takes the nodes of the element.
        let mut nodes = query(QueryKind::Node);
        nodes.picker.select_by = crate::selection::SelectBy::Element;
        nodes.click(&mut model, Some(&hit), 0.5, Operation::Replace, &mut output);
        assert!(matches!(
            model.query_annotations.as_slice(),
            [Annotation::Nodes(n)] if n.len() == 8
        ));
        assert!(
            output
                .iter()
                .any(|l| l.starts_with("Nodes") && l.ends_with('8'))
        );

        // A distance takes the corner points of the geometry mode: across the top.
        let mut distance = query(QueryKind::Distance);
        // Off the middle of the width, so that one corner is nearest to each click.
        let (_, near) = beam_hit(95.0, 2.0);
        let (_, far) = beam_hit(5.0, 2.0);
        distance.click(
            &mut model,
            Some(&near),
            0.5,
            Operation::Replace,
            &mut output,
        );
        assert_eq!(distance.picked().len(), 1);
        distance.click(&mut model, Some(&far), 0.5, Operation::Replace, &mut output);
        assert!(distance.picked().is_empty());
        let Some(Annotation::Distance([a, b])) = model.query_annotations.last() else {
            panic!("{:?}", model.query_annotations);
        };
        let length = base_position(&model, *a)
            .unwrap()
            .distance(base_position(&model, *b).unwrap());
        assert!((length - 100.0).abs() < 1e-6, "{length}");
    }

    #[test]
    fn lines_are_aligned() {
        let line = row("Base", "mm", "x, y, z:", &xyz(DVec3::new(1.0, 2.5, -3.0)));
        assert_eq!(
            line,
            "Base                [mm]        x, y, z:               1,             2.5,              -3"
        );
        assert_eq!(number(0.1234567891), "0.123457");
        assert_eq!(number(12345678.9), "1.23457e7");
        assert_eq!(number(0.0), "0");
    }
}
