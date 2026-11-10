//! The mesh tools: Transform of mesh parts (translate, rotate, mirror, scale) from
//! the part's context menu, and Merge Coincident Nodes and Renumber All of the Mesh menu.
//!
//! A transform is a mesh operation: it is kept under Mesh > Mesh Operations, applied again
//! whenever its parts are meshed anew, and can be edited or deleted later. Its numbers can
//! be exposed as parameters of the analysis with their P buttons, for the project window to
//! set them through value links.

use egui::Ui;
use plx_mesh::MeshTransform;
use plx_model::{Field, ItemRef, MeshOperation, MeshOperationKind, Quantity, UnitSystem};

use crate::expose;
use crate::model::Model;
use crate::numeric;

/// Which tool the dialog edits.
#[derive(Clone, Debug, PartialEq)]
pub enum MeshTool {
    /// Transform these parts, by index: a new Transform mesh operation.
    Transform(Vec<usize>),
    /// Edit the mesh operation, by index.
    Operation(usize),
    MergeNodes,
    Renumber,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TransformKind {
    Translate,
    Rotate,
    Mirror,
    Scale,
}

const TRANSFORM_KINDS: [(TransformKind, &str); 4] = [
    (TransformKind::Translate, "Translate"),
    (TransformKind::Rotate, "Rotate"),
    (TransformKind::Mirror, "Mirror"),
    (TransformKind::Scale, "Scale"),
];

/// The values of every transform kind, so that switching the kind keeps what was typed.
#[derive(Clone, Debug, PartialEq)]
struct TransformDraft {
    kind: TransformKind,
    translation: [f64; 3],
    point: [f64; 3],
    axis: [f64; 3],
    angle: f64,
    normal: [f64; 3],
    center: [f64; 3],
    factors: [f64; 3],
}

impl TransformDraft {
    /// The draft of a new transform: nothing moves yet.
    fn new() -> Self {
        Self {
            kind: TransformKind::Translate,
            translation: [0.0; 3],
            point: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            angle: 0.0,
            normal: [1.0, 0.0, 0.0],
            center: [0.0; 3],
            factors: [1.0; 3],
        }
    }

    /// The draft of a transform there is; the other kinds keep their defaults.
    fn of(transform: &MeshTransform) -> Self {
        let mut draft = Self::new();
        match *transform {
            MeshTransform::Translate(d) => {
                draft.kind = TransformKind::Translate;
                draft.translation = d;
            }
            MeshTransform::Rotate { point, axis, angle } => {
                draft.kind = TransformKind::Rotate;
                (draft.point, draft.axis, draft.angle) = (point, axis, angle);
            }
            MeshTransform::Mirror { point, normal } => {
                draft.kind = TransformKind::Mirror;
                (draft.point, draft.normal) = (point, normal);
            }
            MeshTransform::Scale { center, factors } => {
                draft.kind = TransformKind::Scale;
                (draft.center, draft.factors) = (center, factors);
            }
        }
        draft
    }

    fn transform(&self) -> MeshTransform {
        match self.kind {
            TransformKind::Translate => MeshTransform::Translate(self.translation),
            TransformKind::Rotate => MeshTransform::Rotate {
                point: self.point,
                axis: self.axis,
                angle: self.angle,
            },
            TransformKind::Mirror => MeshTransform::Mirror {
                point: self.point,
                normal: self.normal,
            },
            TransformKind::Scale => MeshTransform::Scale {
                center: self.center,
                factors: self.factors,
            },
        }
    }
}

pub enum MeshToolResult {
    Open,
    Cancel,
    /// Apply the tool and close.
    Apply,
}

pub struct MeshToolDialog {
    tool: MeshTool,
    /// Name of the mesh operation.
    name: String,
    /// The parts the mesh operation changes, by name.
    parts: Vec<String>,
    transform: TransformDraft,
    tolerance: f64,
    first_node: u32,
    first_element: u32,
    error: Option<String>,
    /// The expose buttons of a transform's numbers, see [`Self::set_linked`].
    expose: Option<expose::Scope>,
    /// The exposed parameters of the analysis that value links give, with where from.
    linked: Vec<(u32, String)>,
}

impl MeshToolDialog {
    pub fn new(tool: MeshTool, model: &Model) -> Self {
        // A tolerance of a thousandth of the mesh size.
        let size = model.mesh.bounds().map_or(1.0, |(min, max)| {
            (0..3)
                .map(|k| max[k] - min[k])
                .fold(0.0, f64::max)
                .max(1e-9)
        });
        let operations = &model.fe.mesh_operations;
        let (name, parts, transform) = match &tool {
            MeshTool::Operation(index) => match operations.get(*index) {
                Some(operation) => {
                    let MeshOperationKind::Transform(transform) = &operation.kind;
                    (
                        operation.name.clone(),
                        operation.parts.clone(),
                        TransformDraft::of(transform),
                    )
                }
                None => (String::new(), Vec::new(), TransformDraft::new()),
            },
            MeshTool::Transform(parts) => (
                plx_model::next_name("Transform", operations.iter().map(|o| o.name.as_str())),
                (parts.iter())
                    .filter_map(|&p| model.parts.get(p))
                    .map(|p| p.name.clone())
                    .collect(),
                TransformDraft::new(),
            ),
            _ => (String::new(), Vec::new(), TransformDraft::new()),
        };
        Self {
            tool,
            name,
            parts,
            transform,
            tolerance: size * 1e-3,
            first_node: 1,
            first_element: 1,
            error: None,
            expose: None,
            linked: Vec::new(),
        }
    }

    /// The exposed parameters value links give, by key, with the blocks they come from: the
    /// dialog shows those numbers read-only.
    pub fn set_linked(&mut self, linked: Vec<(u32, String)>) {
        if self.linked != linked {
            self.linked = linked;
            self.expose = None;
        }
    }

    /// The transform as it is in the model, when the dialog edits one.
    fn original(&self, model: &Model) -> Option<ItemRef> {
        let MeshTool::Operation(index) = self.tool else {
            return None;
        };
        let operation = model.fe.mesh_operations.get(index)?;
        Some(ItemRef::MeshOperation(operation.name.clone()))
    }

    /// Draws the transform's form with the expose buttons of its numbers.
    fn exposing_transform_form(&mut self, ui: &mut Ui, model: &Model) {
        let units = model.fe.properties.units;
        let two_d = model.fe.properties.space.is_2d();
        let mut scope = match self.expose.take() {
            Some(scope) => scope,
            None => expose::Scope::new(&model.fe, self.original(model), &self.linked),
        };
        // The numbers the transform has as the dialog shows it now.
        let name = self.name.trim().to_owned();
        let draft = plx_model::FeModel {
            mesh_operations: vec![MeshOperation {
                name: name.clone(),
                parts: Vec::new(),
                kind: MeshOperationKind::Transform(self.transform.transform()),
            }],
            ..Default::default()
        };
        scope.available = draft.exposable(&ItemRef::MeshOperation(name));
        let transform = &mut self.transform;
        scope.show(ui, |ui| transform_form(ui, transform, two_d, units));
        self.expose = Some(scope);
    }

    fn title(&self) -> &'static str {
        match self.tool {
            MeshTool::Transform(_) => "Transform mesh parts",
            MeshTool::Operation(_) => "Edit transform",
            MeshTool::MergeNodes => "Merge coincident nodes",
            MeshTool::Renumber => "Renumber nodes and elements",
        }
    }

    pub fn show(&mut self, ctx: &egui::Context, model: &Model) -> MeshToolResult {
        let mut result = MeshToolResult::Open;
        let mut open = true;
        egui::Window::new(self.title())
            .id(crate::ids::window(ctx, "mesh tool"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                let units = model.fe.properties.units;
                egui::Grid::new("mesh tool grid")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| match &self.tool {
                        MeshTool::Transform(_) | MeshTool::Operation(_) => {
                            ui.label("Name");
                            ui.add(egui::TextEdit::singleline(&mut self.name).desired_width(160.0));
                            ui.end_row();
                            ui.label("Parts");
                            parts_list(ui, &mut self.parts, model);
                            ui.end_row();
                            self.exposing_transform_form(ui, model);
                        }
                        MeshTool::MergeNodes => {
                            ui.label("Tolerance");
                            ui.add(
                                numeric::quantity(&mut self.tolerance, units, Quantity::Length)
                                    .range(0.0..=f64::MAX),
                            );
                            ui.end_row();
                            ui.label("");
                            ui.weak(
                                "Nodes of all parts closer than the tolerance become one node, \
                                 the one with the lowest number.",
                            );
                            ui.end_row();
                        }
                        MeshTool::Renumber => {
                            ui.label("First node number");
                            ui.add(numeric::drag_value(&mut self.first_node).range(1..=u32::MAX));
                            ui.end_row();
                            ui.label("First element number");
                            ui.add(
                                numeric::drag_value(&mut self.first_element).range(1..=u32::MAX),
                            );
                            ui.end_row();
                            ui.label("");
                            ui.weak("Nodes and elements are numbered in their current order.");
                            ui.end_row();
                        }
                    });
                if let Some(error) = &self.error {
                    ui.add_space(4.0);
                    ui.colored_label(crate::style::error(), error);
                }
                ui.add_space(8.0);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if ui.button("Cancel").clicked() {
                        result = MeshToolResult::Cancel;
                    }
                    if ui.button("OK").clicked() {
                        match self.problem() {
                            Some(problem) => self.error = Some(problem.into()),
                            None => result = MeshToolResult::Apply,
                        }
                    }
                });
            });
        if !open || ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            result = MeshToolResult::Cancel;
        }
        result
    }

    fn problem(&self) -> Option<&'static str> {
        match self.tool {
            MeshTool::Transform(_) | MeshTool::Operation(_) if self.name.trim().is_empty() => {
                Some("The name must not be empty.")
            }
            MeshTool::Transform(_) | MeshTool::Operation(_) if self.parts.is_empty() => {
                Some("Select at least one part.")
            }
            MeshTool::Transform(_) | MeshTool::Operation(_) => self.transform.transform().problem(),
            MeshTool::MergeNodes if self.tolerance.is_nan() || self.tolerance < 0.0 => {
                Some("The tolerance must not be negative.")
            }
            _ => None,
        }
    }

    /// Applies the tool to the model and says what changed, or why nothing did.
    pub fn apply(&mut self, model: &mut Model) -> Result<String, String> {
        let result = self.run(model);
        match &result {
            Err(error) => self.error = Some(error.clone()),
            // The exposed parameters follow the transform, under its new name.
            Ok(_) => {
                if let Some(scope) = self.expose.take() {
                    let item = ItemRef::MeshOperation(self.name.trim().to_owned());
                    scope.apply(&mut model.fe, &item);
                }
            }
        }
        result
    }

    fn run(&self, model: &mut Model) -> Result<String, String> {
        if let Some(problem) = self.problem() {
            return Err(problem.into());
        }
        match &self.tool {
            MeshTool::Transform(_) | MeshTool::Operation(_) => {
                let edited = match self.tool {
                    MeshTool::Operation(index) => Some(index),
                    _ => None,
                };
                let name = self.name.trim().to_owned();
                let operations = &model.fe.mesh_operations;
                if (operations.iter().enumerate())
                    .any(|(i, o)| Some(i) != edited && o.name.eq_ignore_ascii_case(&name))
                {
                    return Err(format!("The name {name} is already used."));
                }
                // The parts in the order of the mesh.
                let parts: Vec<String> = (model.parts.iter())
                    .map(|p| p.name.clone())
                    .filter(|p| self.parts.contains(p))
                    .collect();
                let operation = MeshOperation {
                    name: name.clone(),
                    parts,
                    kind: MeshOperationKind::Transform(self.transform.transform()),
                };
                let nodes = model
                    .mesh
                    .part_nodes(&operation.part_indices(&model.mesh))
                    .len();
                if nodes == 0 {
                    return Err("The parts have no nodes.".into());
                }
                let mut operations = operations.clone();
                match edited {
                    Some(index) if index < operations.len() => operations[index] = operation,
                    Some(_) => return Err("The transform no longer exists.".into()),
                    None => operations.push(operation),
                }
                let names = self.parts.join(", ");
                model.edit_mesh(|mesh, fe| fe.set_mesh_operations(mesh, operations));
                let what = TRANSFORM_KINDS
                    .iter()
                    .find(|(k, _)| *k == self.transform.kind)
                    .map_or("", |(_, label)| label);
                Ok(match edited {
                    Some(_) => format!("{name} changed: {what} {names} ({nodes} nodes)"),
                    None => format!("{name}: {what} {names} ({nodes} nodes moved)"),
                })
            }
            MeshTool::MergeNodes => {
                let replaced = model.mesh.coincident_nodes(&[], self.tolerance);
                if replaced.is_empty() {
                    return Err(format!(
                        "No nodes lie within {} {} of another one.",
                        numeric::format_physical(
                            model
                                .fe
                                .properties
                                .units
                                .shown(self.tolerance, Quantity::Length)
                        ),
                        model.fe.properties.units.unit(Quantity::Length)
                    ));
                }
                let count = replaced.len();
                model.edit_mesh(|mesh, fe| {
                    mesh.merge_nodes(&replaced);
                    fe.merge_nodes(&replaced);
                });
                Ok(format!("{count} nodes merged into coincident ones"))
            }
            MeshTool::Renumber => {
                let (first_node, first_element) = (self.first_node, self.first_element);
                let (nodes, elements) = (model.mesh.node_count(), model.mesh.element_count());
                model.edit_mesh(|mesh, fe| {
                    let (node_map, element_map) = mesh.renumber(first_node, first_element);
                    fe.renumber(&node_map, &element_map);
                });
                Ok(format!(
                    "Renumbered: nodes {first_node} to {}, elements {first_element} to {}",
                    first_node as usize + nodes.saturating_sub(1),
                    first_element as usize + elements.saturating_sub(1)
                ))
            }
        }
    }
}

/// A check box per mesh part; parts the list names that the mesh no longer has stay
/// listed, so that they can be taken out.
fn parts_list(ui: &mut Ui, parts: &mut Vec<String>, model: &Model) {
    let mut names: Vec<String> = model.parts.iter().map(|p| p.name.clone()).collect();
    names.extend(
        parts
            .iter()
            .filter(|p| !names.contains(p))
            .cloned()
            .collect::<Vec<_>>(),
    );
    egui::ScrollArea::vertical()
        .max_height(160.0)
        .auto_shrink([false, true])
        .show(ui, |ui| {
            ui.vertical(|ui| {
                for name in names {
                    let mut checked = parts.contains(&name);
                    if ui.checkbox(&mut checked, &name).changed() {
                        if checked {
                            parts.push(name);
                        } else {
                            parts.retain(|p| *p != name);
                        }
                    }
                }
            });
        });
}

/// A row per component of `vector`, each with its expose button of `fields`.
fn vector_rows(
    ui: &mut Ui,
    vector: &mut [f64; 3],
    (labels, fields): ([&str; 3], [Field; 3]),
    two_d: bool,
    quantity: Option<Quantity>,
    units: UnitSystem,
) {
    let count = if two_d { 2 } else { 3 };
    for ((value, label), field) in vector.iter_mut().zip(labels).zip(fields).take(count) {
        ui.label(label);
        expose::field(ui, field, |ui| match quantity {
            Some(quantity) => ui.add(numeric::quantity(value, units, quantity).speed(0.1)),
            None => ui.add(numeric::drag_value(value).speed(0.1)),
        });
        ui.end_row();
    }
}

/// Drag values of a direction in one row, each with its expose button of `fields`.
fn direction_row(ui: &mut Ui, direction: &mut [f64; 3], fields: [Field; 3], count: usize) {
    ui.horizontal(|ui| {
        for (value, field) in direction.iter_mut().zip(fields).take(count) {
            expose::field(ui, field, |ui| {
                ui.add(numeric::drag_value(value).speed(0.1))
            });
        }
    });
}

fn transform_form(ui: &mut Ui, draft: &mut TransformDraft, two_d: bool, units: UnitSystem) {
    ui.label("Kind");
    let label = TRANSFORM_KINDS
        .iter()
        .find(|(k, _)| *k == draft.kind)
        .map_or("", |(_, label)| label);
    egui::ComboBox::from_id_salt("transform kind")
        .selected_text(label)
        .width(160.0)
        .show_ui(ui, |ui| {
            for (kind, label) in TRANSFORM_KINDS {
                ui.selectable_value(&mut draft.kind, kind, label);
            }
        });
    ui.end_row();
    let length = Some(Quantity::Length);
    let point = (["X", "Y", "Z"], Field::LOCATION);
    match draft.kind {
        TransformKind::Translate => {
            vector_rows(
                ui,
                &mut draft.translation,
                (["dX", "dY", "dZ"], Field::VECTOR),
                two_d,
                length,
                units,
            );
        }
        TransformKind::Rotate => {
            vector_rows(ui, &mut draft.point, point, two_d, length, units);
            if two_d {
                ui.label("Axis");
                ui.label("Z");
                ui.end_row();
                draft.axis = [0.0, 0.0, 1.0];
            } else {
                ui.label("Axis");
                direction_row(ui, &mut draft.axis, Field::AXIS, 3);
                ui.end_row();
            }
            ui.label("Angle");
            expose::field(ui, Field::Angle, |ui| {
                ui.add(
                    numeric::drag_value(&mut draft.angle)
                        .speed(1.0)
                        .suffix(" °"),
                )
            });
            ui.end_row();
        }
        TransformKind::Mirror => {
            vector_rows(ui, &mut draft.point, point, two_d, length, units);
            ui.label("Normal");
            direction_row(
                ui,
                &mut draft.normal,
                Field::NORMAL,
                if two_d { 2 } else { 3 },
            );
            ui.end_row();
            if two_d {
                draft.normal[2] = 0.0;
            }
            ui.label("");
            ui.weak("Mirrored elements are turned inside out again, so they stay valid.");
            ui.end_row();
        }
        TransformKind::Scale => {
            vector_rows(ui, &mut draft.center, point, two_d, length, units);
            vector_rows(
                ui,
                &mut draft.factors,
                (["Factor X", "Factor Y", "Factor Z"], Field::SCALE),
                two_d,
                None,
                units,
            );
        }
    }
    ui.label("");
    ui.weak(
        "Kept under Mesh > Mesh Operations: meshing the parts again applies it again; \
         deleting it moves them back.",
    );
    ui.end_row();
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_model::UnitSystem;

    /// The two blocks of `zwei_bloecke.step`, meshed as Mesh does; `None` without Gmsh.
    fn meshed_blocks() -> Option<Model> {
        if let Err(error) = plx_mesher::self_test() {
            if std::env::var_os("NODALIS_REQUIRE_GMSH").is_some() {
                panic!("Gmsh is required: {error}");
            }
            eprintln!("Gmsh not available, test skipped: {error}");
            return None;
        }
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/zwei_bloecke.step");
        let mut model = crate::model::load(&path, UnitSystem::Unitless)
            .unwrap()
            .model;
        remesh(&mut model, None);
        Some(model)
    }

    /// Meshes the named parts of the geometry anew, or all.
    fn remesh(model: &mut Model, parts: Option<Vec<String>>) {
        let geometry = model.geometry.clone().unwrap();
        let parts = parts.unwrap_or_else(|| plx_mesher::part_names(&geometry).unwrap());
        let meshes = (parts.iter())
            .map(|p| plx_mesher::generate_part_mesh(&geometry, p).unwrap())
            .collect();
        let generated = crate::meshing::PartMeshes { meshes };
        crate::app::merge_part_meshes(model, &geometry, generated, None).unwrap();
    }

    /// The smallest x of the nodes of a part.
    fn min_x(model: &Model, part: usize) -> f64 {
        (model.mesh.part_nodes(&[part]).iter())
            .map(|&n| model.mesh.node(n).unwrap()[0])
            .fold(f64::INFINITY, f64::min)
    }

    #[test]
    fn a_transform_stays_when_the_part_is_meshed_again_and_goes_when_deleted() {
        let Some(mut model) = meshed_blocks() else {
            return;
        };
        let start = min_x(&model, 0);
        let other = min_x(&model, 1);
        let mut dialog = MeshToolDialog::new(MeshTool::Transform(vec![0]), &model);
        assert_eq!(dialog.name, "Transform-1");
        dialog.transform.translation = [100.0, 0.0, 0.0];
        dialog.apply(&mut model).unwrap();
        assert_eq!(model.fe.mesh_operations.len(), 1);
        assert!((min_x(&model, 0) - start - 100.0).abs() < 1e-9);
        assert!((min_x(&model, 1) - other).abs() < 1e-9);

        // Meshed again, the part is moved again rather than back at its place.
        let part = model.parts[0].name.clone();
        remesh(&mut model, Some(vec![part]));
        assert!((min_x(&model, 0) - start - 100.0).abs() < 1e-9);
        remesh(&mut model, None);
        assert!((min_x(&model, 0) - start - 100.0).abs() < 1e-9);
        assert!((min_x(&model, 1) - other).abs() < 1e-9);

        // Edited, it moves the part by the new distance, without meshing.
        let mut dialog = MeshToolDialog::new(MeshTool::Operation(0), &model);
        assert_eq!(dialog.transform.translation, [100.0, 0.0, 0.0]);
        dialog.transform.translation = [40.0, 0.0, 0.0];
        dialog.apply(&mut model).unwrap();
        assert!((min_x(&model, 0) - start - 40.0).abs() < 1e-9);
        assert_eq!(model.fe.mesh_operations.len(), 1);

        // Deleted, the part is back where the geometry has it.
        let mut operations = model.fe.mesh_operations.clone();
        operations.clear();
        model.edit_mesh(|mesh, fe| fe.set_mesh_operations(mesh, operations));
        assert!((min_x(&model, 0) - start).abs() < 1e-9);
        remesh(&mut model, None);
        assert!((min_x(&model, 0) - start).abs() < 1e-9);
    }

    #[test]
    fn a_transform_needs_a_part_and_an_unused_name() {
        let Some(mut model) = meshed_blocks() else {
            return;
        };
        let mut dialog = MeshToolDialog::new(MeshTool::Transform(vec![0]), &model);
        dialog.apply(&mut model).unwrap();
        let mut dialog = MeshToolDialog::new(MeshTool::Transform(vec![1]), &model);
        assert_eq!(dialog.name, "Transform-2");
        dialog.name = "transform-1".into();
        assert!(dialog.apply(&mut model).is_err());
        dialog.name = "Transform-2".into();
        dialog.parts.clear();
        assert!(dialog.apply(&mut model).is_err());
        assert_eq!(model.fe.mesh_operations.len(), 1);
    }
}
