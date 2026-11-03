//! PrePoMax's "Create Transformation" dialog of the results tool bar: available types on the
//! left, the active transformations on the right, the properties of the selected one below.

use std::collections::BTreeSet;

use egui::{RichText, Ui};
use glam::{DVec3, Vec3};
use plx_mesh::NodeId;
use plx_results::transformation::{SymmetryPlane, Transformation, TransformationKind};

use crate::icons::{self, Icon};
use crate::model::{Highlight, Hit, Model};
use crate::numeric;
use crate::selection::{Items, Operation, Picker, PickerAction, Target};
use crate::tree_rows::{frame, tree_row};
use crate::viewport::Preview;

/// An entry of the Available tree.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Available {
    Symmetry(SymmetryPlane),
    Linear,
    Circular,
}

impl Available {
    fn create(self) -> Transformation {
        match self {
            Available::Symmetry(plane) => Transformation::symmetry(plane),
            Available::Linear => Transformation::linear_pattern(),
            Available::Circular => Transformation::circular_pattern(),
        }
    }
}

/// Which point of the selected transformation a click in the 3D view sets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Slot {
    First,
    Second,
}

/// Rows of the property grid; the focused one is described below the grid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Row {
    Name,
    Count,
    Angle,
    Point(Slot),
}

/// What the user did in the dialog this frame.
pub enum TransformationAction {
    Open,
    /// Show these transformations (none: the results alone); `close` ends the dialog.
    Apply {
        transformations: Vec<Transformation>,
        close: bool,
    },
    Cancel,
}

pub struct TransformationDialog {
    /// The results file the dialog edits the transformations of.
    pub result: usize,
    active: Vec<Transformation>,
    selected: Option<usize>,
    available: Option<Available>,
    /// Expanded state of the Symmetry and Pattern branches.
    expanded: [bool; 2],
    focus: Row,
    /// Something changed since the transformations were last applied.
    changed: bool,
    picking: Option<Slot>,
    picker: Picker,
    /// Nodes of the last picked point, shown highlighted.
    marked: BTreeSet<NodeId>,
    error: Option<String>,
    /// Drag speed of coordinates, from the model size.
    speed: f64,
    /// Half the diagonal of the model, for the drawn axes.
    size: f64,
    /// Half the size of the model along each axis, for the drawn symmetry planes.
    half_extent: DVec3,
    center: DVec3,
}

impl TransformationDialog {
    pub fn new(result: usize, current: &[Transformation], model: &Model) -> Self {
        let (min, max) = model
            .mesh
            .bounds()
            .map_or((DVec3::ZERO, DVec3::ONE), |(a, b)| (a.into(), b.into()));
        let size = ((max - min).length() * 0.5).max(1e-9);
        Self {
            result,
            active: current.to_vec(),
            selected: (!current.is_empty()).then_some(0),
            available: None,
            expanded: [true, true],
            focus: Row::Name,
            changed: false,
            picking: None,
            picker: Picker::default(),
            marked: BTreeSet::new(),
            error: None,
            speed: size * 0.005,
            size,
            // A little larger than the model, never a line for flat models.
            half_extent: ((max - min) * 0.6).max(DVec3::splat(size * 0.1)),
            center: (min + max) * 0.5,
        }
    }

    /// Whether a click in the 3D view picks a point.
    pub fn picks(&self) -> bool {
        self.picking.is_some()
    }

    pub fn highlight(&self) -> Highlight {
        Highlight {
            nodes: self.marked.iter().copied().collect(),
            ..Highlight::default()
        }
    }

    /// The points of the selected transformation in render coordinates, highlighted like
    /// PrePoMax does.
    pub fn points(&self, model: &Model) -> Vec<Vec3> {
        let Some(transformation) = self.selected.and_then(|i| self.active.get(i)) else {
            return Vec::new();
        };
        let points = match transformation.kind {
            TransformationKind::Symmetry { point, .. } => vec![point],
            TransformationKind::LinearPattern { start, end, .. } => vec![start, end],
            TransformationKind::CircularPattern {
                axis_start,
                axis_end,
                ..
            } => vec![axis_start, axis_end],
        };
        points.into_iter().map(|p| model.to_render(p)).collect()
    }

    /// The selected transformation drawn as lines in render coordinates: the outline of a
    /// symmetry plane, the arrow of a linear pattern and the axis of a circular one.
    pub fn lines(&self, model: &Model) -> Vec<Vec<Vec3>> {
        let Some(transformation) = self.selected.and_then(|i| self.active.get(i)) else {
            return Vec::new();
        };
        let render = |p: DVec3| model.to_render(p.to_array());
        match transformation.kind {
            TransformationKind::Symmetry { plane, point } => {
                let axis = plane.axis();
                let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
                let corner = |a: f64, b: f64| {
                    let mut p = self.center;
                    p[axis] = point[axis];
                    p[u] += a * self.half_extent[u];
                    p[v] += b * self.half_extent[v];
                    render(p)
                };
                let corners = [
                    (-1.0, -1.0),
                    (1.0, -1.0),
                    (1.0, 1.0),
                    (-1.0, 1.0),
                    (-1.0, -1.0),
                ];
                vec![corners.iter().map(|&(a, b)| corner(a, b)).collect()]
            }
            TransformationKind::LinearPattern { start, end, .. } => {
                let (start, end) = (DVec3::from(start), DVec3::from(end));
                let mut paths = vec![vec![render(start), render(end)]];
                if let Some(direction) = (end - start).try_normalize() {
                    let side = direction.any_orthonormal_vector();
                    let tip = 0.05 * self.size;
                    let back = end - direction * tip;
                    paths.push(vec![
                        render(back + side * tip * 0.4),
                        render(end),
                        render(back - side * tip * 0.4),
                    ]);
                }
                paths
            }
            TransformationKind::CircularPattern {
                axis_start,
                axis_end,
                ..
            } => {
                let (start, end) = (DVec3::from(axis_start), DVec3::from(axis_end));
                let Some(direction) = (end - start).try_normalize() else {
                    return Vec::new();
                };
                // The axis through the whole model, the given points marked by its ends.
                let along = (self.center - start).dot(direction);
                vec![vec![
                    render(start + direction * (along - self.size)),
                    render(start + direction * (along + self.size)),
                ]]
            }
        }
    }

    /// A click in the 3D view while picking: the centre of what it selects becomes the point.
    pub fn click(&mut self, model: &Model, pick: Option<(&Hit, f32)>) {
        let Some((hit, precision)) = pick else {
            return;
        };
        let picked = self.picker.pick(model, hit, Target::Nodes, precision);
        if let Items::Nodes(nodes) = picked.resolved(&model.mesh, Target::Nodes) {
            self.take(model, nodes);
        }
    }

    pub fn preview(&self, model: &Model, hit: &Hit, precision: f32) -> Preview {
        let items = self.picker.pick(model, hit, Target::Nodes, precision);
        crate::selection::preview(model, &items)
    }

    /// One node is itself the point; an edge or a face gives its centre, e.g. that of a hole.
    fn take(&mut self, model: &Model, nodes: BTreeSet<NodeId>) {
        let positions: Vec<DVec3> = (nodes.iter())
            .filter_map(|&id| model.mesh.node(id))
            .map(DVec3::from)
            .collect();
        let (Some(slot), false) = (self.picking, positions.is_empty()) else {
            return;
        };
        let picked = (positions.iter().sum::<DVec3>() / positions.len() as f64).to_array();
        let Some(transformation) = self.selected.and_then(|i| self.active.get_mut(i)) else {
            return;
        };
        *point_mut(&mut transformation.kind, slot) = picked;
        self.marked = nodes;
        self.picking = None;
        self.changed = true;
    }

    fn action(&mut self, model: &Model, action: PickerAction) {
        match action {
            PickerAction::Undo | PickerAction::Clear => self.marked.clear(),
            PickerAction::Ids(Operation::Add, ids) => {
                let nodes = (ids.into_iter())
                    .filter(|&id| model.mesh.node_index(id).is_some())
                    .collect();
                self.take(model, nodes);
            }
            _ => {}
        }
    }

    pub fn show(&mut self, ctx: &egui::Context, model: &Model) -> TransformationAction {
        let mut action = TransformationAction::Open;
        let mut open = true;
        let window = egui::Window::new("Transformations")
            .id(crate::ids::window(ctx, "transformation dialog"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::LEFT_TOP)
            .default_pos(ctx.content_rect().left_top() + egui::vec2(300.0, 110.0))
            .show(ctx, |ui| {
                ui.set_width(360.0);
                ui.label(RichText::new("Transformation Types").strong());
                ui.horizontal_top(|ui| self.lists(ui));
                ui.add_space(6.0);
                ui.label(RichText::new("Properties").strong());
                frame().show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.set_min_height(210.0);
                    self.properties(ui);
                });
                ui.add_space(4.0);
                egui::Frame::group(ui.style()).show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.set_height(56.0);
                    if self.selected.is_some() {
                        let (title, text) = self.description();
                        ui.strong(title);
                        ui.label(text);
                    }
                });
                if let Some(error) = &self.error {
                    ui.colored_label(crate::style::error(), error);
                }
                // A tool window: Off shows the results without transformations, Apply
                // shows the edited ones, Close applies them and closes.
                let mut apply = false;
                let footer = crate::dialog::tool_footer_with(ui, Some(true), |ui| {
                    apply = ui.button("Apply").clicked();
                });
                if apply {
                    self.changed = true;
                    action = self.apply(false);
                }
                match footer {
                    crate::dialog::Tool::Off => {
                        // Close then keeps the results as they are, as OK in PrePoMax.
                        self.changed = false;
                        self.error = None;
                        action = TransformationAction::Apply {
                            transformations: Vec::new(),
                            close: true,
                        };
                    }
                    crate::dialog::Tool::Close => action = self.apply(true),
                    crate::dialog::Tool::None => {}
                }
            });
        if let Some(window) = window
            && self.picking.is_some()
        {
            let can_undo = !self.marked.is_empty();
            if let Some(picked) =
                self.picker
                    .window(ctx, window.response.rect, Target::Nodes, can_undo)
            {
                self.action(model, picked);
            }
        }
        if !open || ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            action = TransformationAction::Cancel;
        }
        action
    }

    /// The transformations to show, checked; nothing to do if none changed.
    fn apply(&mut self, close: bool) -> TransformationAction {
        if !self.changed {
            return if close {
                TransformationAction::Cancel
            } else {
                TransformationAction::Open
            };
        }
        if let Err(error) = self.active.iter().try_for_each(Transformation::check) {
            self.error = Some(error);
            return TransformationAction::Open;
        }
        self.error = None;
        self.changed = false;
        TransformationAction::Apply {
            transformations: self.active.clone(),
            close,
        }
    }

    /// The Available tree, the add and remove buttons and the Active list.
    fn lists(&mut self, ui: &mut Ui) {
        let height = 130.0;
        let mut add = false;
        ui.vertical(|ui| {
            ui.label("Available");
            frame().show(ui, |ui| {
                ui.set_width(140.0);
                ui.set_height(height);
                let branches = [
                    (
                        "Symmetry",
                        SymmetryPlane::ALL
                            .iter()
                            .map(|&p| (Available::Symmetry(p), p.label()))
                            .collect::<Vec<_>>(),
                    ),
                    (
                        "Pattern",
                        vec![
                            (Available::Linear, "Linear"),
                            (Available::Circular, "Circular"),
                        ],
                    ),
                ];
                for ((label, entries), expanded) in branches.into_iter().zip(&mut self.expanded) {
                    tree_row(ui, 0, Some(&mut *expanded), false, RichText::new(label));
                    if !*expanded {
                        continue;
                    }
                    for (entry, label) in entries {
                        let selected = self.available == Some(entry);
                        let row = tree_row(ui, 1, None, selected, RichText::new(label));
                        if row.clicked() {
                            self.available = Some(entry);
                        }
                        if row.double_clicked() {
                            add = true;
                        }
                    }
                }
            });
        });
        ui.vertical(|ui| {
            ui.add_space(22.0);
            let can_add = self.available.is_some();
            if icons::dialog_button(ui, Icon::Arrow(egui::vec2(1.0, 0.0)), "Add", can_add).clicked()
            {
                add = true;
            }
            if icons::dialog_button(ui, Icon::Remove, "Remove", self.selected.is_some()).clicked()
                && let Some(index) = self.selected
            {
                self.active.remove(index);
                self.selected = (!self.active.is_empty()).then(|| index.min(self.active.len() - 1));
                self.picking = None;
                self.marked.clear();
                self.changed = true;
            }
        });
        if add && let Some(entry) = self.available {
            self.active.push(entry.create());
            self.selected = Some(self.active.len() - 1);
            self.focus = Row::Name;
            self.picking = None;
            self.marked.clear();
            self.changed = true;
        }
        ui.vertical(|ui| {
            ui.label("Active");
            frame().show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.set_height(height);
                egui::ScrollArea::vertical().show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    for index in 0..self.active.len() {
                        let selected = self.selected == Some(index);
                        let name = RichText::new(&self.active[index].name);
                        if tree_row(ui, 0, None, selected, name).clicked() && !selected {
                            self.selected = Some(index);
                            self.focus = Row::Name;
                            self.picking = None;
                            self.marked.clear();
                        }
                    }
                });
            });
        });
    }

    fn properties(&mut self, ui: &mut Ui) {
        let Some(transformation) = self.selected.and_then(|i| self.active.get_mut(i)) else {
            return;
        };
        let speed = self.speed;
        let mut grid = Grid {
            focus: &mut self.focus,
            picking: &mut self.picking,
            changed: false,
        };
        grid.category(ui, "Data");
        grid.rows(ui, "data", |grid, ui| {
            grid.row(ui, Row::Name, |ui| {
                let edit =
                    egui::TextEdit::singleline(&mut transformation.name).desired_width(170.0);
                let response = ui.add(edit);
                (response.changed(), response.has_focus())
            });
            match &mut transformation.kind {
                TransformationKind::Symmetry { .. } => {}
                TransformationKind::LinearPattern { count, .. } => {
                    grid.count_row(ui, count);
                }
                TransformationKind::CircularPattern { count, angle, .. } => {
                    grid.count_row(ui, count);
                    grid.row(ui, Row::Angle, |ui| {
                        let value = numeric::drag_value(angle).speed(1.0).suffix(" °");
                        let height = ui.spacing().interact_size.y;
                        let response = ui.add_sized([120.0, height], value);
                        (response.changed(), response.has_focus())
                    });
                }
            }
        });
        let slots: &[(Slot, &str)] = match transformation.kind {
            TransformationKind::Symmetry { .. } => &[(Slot::First, "Symmetry Point")],
            TransformationKind::LinearPattern { .. } => {
                &[(Slot::First, "Start Point"), (Slot::Second, "End Point")]
            }
            TransformationKind::CircularPattern { .. } => &[
                (Slot::First, "First Axis Point"),
                (Slot::Second, "Second Axis Point"),
            ],
        };
        for &(slot, title) in slots {
            grid.category(ui, title);
            let point = point_mut(&mut transformation.kind, slot);
            grid.rows(ui, title, |grid, ui| {
                grid.point_rows(ui, slot, point, speed)
            });
        }
        if grid.changed {
            self.changed = true;
            self.marked.clear();
        }
    }

    fn description(&self) -> (&'static str, String) {
        let kind = self
            .selected
            .and_then(|i| self.active.get(i))
            .map(|t| &t.kind);
        match (self.focus, kind) {
            (Row::Name, _) | (_, None) => ("Name", "Name of the transformation.".into()),
            (Row::Count, _) => (
                "Number of items",
                "Number of all items of the pattern, including the original.".into(),
            ),
            (Row::Angle, _) => (
                "Angle",
                "Angle between two neighbouring items, positive by the right-hand rule \
                 about the axis from the first to the second axis point."
                    .into(),
            ),
            (Row::Point(_), Some(TransformationKind::Symmetry { plane, .. })) => (
                "Symmetry point",
                format!(
                    "The symmetry plane passes through this point, normal to the {} axis.",
                    plane.label()
                ),
            ),
            (Row::Point(_), Some(TransformationKind::LinearPattern { .. })) => (
                "Start and end point",
                "Each item is offset from the previous one by the distance from the start to \
                 the end point."
                    .into(),
            ),
            (Row::Point(_), Some(TransformationKind::CircularPattern { .. })) => (
                "Axis points",
                "Two points on the rotation axis of the pattern.".into(),
            ),
        }
    }
}

/// The property grid's rows with the focus and picking state they change.
struct Grid<'a> {
    focus: &'a mut Row,
    picking: &'a mut Option<Slot>,
    changed: bool,
}

impl Grid<'_> {
    /// Category header like PrePoMax's property grid.
    fn category(&self, ui: &mut Ui, title: &str) {
        let header = egui::Frame::new()
            .fill(crate::style::control())
            .inner_margin(egui::Margin::symmetric(4, 1));
        header.show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.strong(title);
        });
    }

    fn rows(&mut self, ui: &mut Ui, id: &str, add: impl FnOnce(&mut Self, &mut Ui)) {
        egui::Grid::new(("transformation properties", id))
            .num_columns(2)
            .striped(true)
            .min_col_width(110.0)
            .spacing([8.0, 4.0])
            .show(ui, |ui| add(self, ui));
    }

    /// One row; the editor returns whether it changed the value and has the focus.
    fn row(&mut self, ui: &mut Ui, row: Row, editor: impl FnOnce(&mut Ui) -> (bool, bool)) {
        self.labelled(ui, row, row_label(row), editor);
    }

    fn labelled(
        &mut self,
        ui: &mut Ui,
        row: Row,
        label: &str,
        editor: impl FnOnce(&mut Ui) -> (bool, bool),
    ) {
        let label = ui.add(egui::Label::new(label).sense(egui::Sense::click()));
        let (changed, focused) = editor(ui);
        if changed || focused || label.clicked() {
            *self.focus = row;
        }
        self.changed |= changed;
        ui.end_row();
    }

    fn count_row(&mut self, ui: &mut Ui, count: &mut u32) {
        self.row(ui, Row::Count, |ui| {
            let height = ui.spacing().interact_size.y;
            let value = numeric::drag_value(count).range(2..=1000);
            let response = ui.add_sized([120.0, height], value);
            (response.changed(), response.has_focus())
        });
    }

    fn point_rows(&mut self, ui: &mut Ui, slot: Slot, point: &mut [f64; 3], speed: f64) {
        let row = Row::Point(slot);
        let picking = *self.picking == Some(slot);
        let mut toggle = false;
        self.labelled(ui, row, "By selection", |ui| {
            toggle = crate::setup::pick_button(ui, picking);
            (false, toggle)
        });
        if toggle {
            *self.picking = if picking { None } else { Some(slot) };
        }
        for (k, axis) in ["X", "Y", "Z"].into_iter().enumerate() {
            self.labelled(ui, row, axis, |ui| {
                let value = numeric::drag_value(&mut point[k])
                    .speed(speed)
                    .max_decimals(6);
                let height = ui.spacing().interact_size.y;
                let response = ui.add_sized([120.0, height], value);
                (response.changed(), response.has_focus())
            });
        }
    }
}

fn row_label(row: Row) -> &'static str {
    match row {
        Row::Name => "Name",
        Row::Count => "Number of items",
        Row::Angle => "Angle",
        Row::Point(_) => "",
    }
}

fn point_mut(kind: &mut TransformationKind, slot: Slot) -> &mut [f64; 3] {
    match (kind, slot) {
        (TransformationKind::Symmetry { point, .. }, _) => point,
        (TransformationKind::LinearPattern { start, .. }, Slot::First) => start,
        (TransformationKind::LinearPattern { end, .. }, Slot::Second) => end,
        (TransformationKind::CircularPattern { axis_start, .. }, Slot::First) => axis_start,
        (TransformationKind::CircularPattern { axis_end, .. }, Slot::Second) => axis_end,
    }
}
