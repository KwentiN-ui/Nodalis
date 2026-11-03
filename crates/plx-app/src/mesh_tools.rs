//! The mesh tools: Transform of mesh parts (translate, rotate, mirror, scale) from
//! the part's context menu, and Merge Coincident Nodes and Renumber All of the Mesh menu.

use egui::Ui;
use plx_mesh::MeshTransform;
use plx_model::{Quantity, UnitSystem};

use crate::model::Model;
use crate::numeric;

/// Which tool the dialog edits.
#[derive(Clone, Debug, PartialEq)]
pub enum MeshTool {
    /// Transform these parts, by index.
    Transform(Vec<usize>),
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
    transform: TransformDraft,
    tolerance: f64,
    first_node: u32,
    first_element: u32,
    error: Option<String>,
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
        Self {
            tool,
            transform: TransformDraft {
                kind: TransformKind::Translate,
                translation: [0.0; 3],
                point: [0.0; 3],
                axis: [0.0, 0.0, 1.0],
                angle: 0.0,
                normal: [1.0, 0.0, 0.0],
                center: [0.0; 3],
                factors: [1.0; 3],
            },
            tolerance: size * 1e-3,
            first_node: 1,
            first_element: 1,
            error: None,
        }
    }

    fn title(&self) -> &'static str {
        match self.tool {
            MeshTool::Transform(_) => "Transform mesh parts",
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
                let two_d = model.fe.properties.space.is_2d();
                egui::Grid::new("mesh tool grid")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| match &self.tool {
                        MeshTool::Transform(parts) => {
                            let names: Vec<&str> = (parts.iter())
                                .filter_map(|&p| model.parts.get(p))
                                .map(|p| p.name.as_str())
                                .collect();
                            ui.label("Parts");
                            ui.label(names.join(", "));
                            ui.end_row();
                            transform_form(ui, &mut self.transform, two_d, units);
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
            MeshTool::Transform(_) => self.transform.transform().problem(),
            MeshTool::MergeNodes if self.tolerance.is_nan() || self.tolerance < 0.0 => {
                Some("The tolerance must not be negative.")
            }
            _ => None,
        }
    }

    /// Applies the tool to the model and says what changed, or why nothing did.
    pub fn apply(&mut self, model: &mut Model) -> Result<String, String> {
        let result = self.run(model);
        if let Err(error) = &result {
            self.error = Some(error.clone());
        }
        result
    }

    fn run(&self, model: &mut Model) -> Result<String, String> {
        if let Some(problem) = self.problem() {
            return Err(problem.into());
        }
        match &self.tool {
            MeshTool::Transform(parts) => {
                let transform = self.transform.transform();
                let nodes = model.mesh.part_nodes(parts).len();
                if nodes == 0 {
                    return Err("The parts have no nodes.".into());
                }
                let names: Vec<String> = (parts.iter())
                    .filter_map(|&p| model.parts.get(p))
                    .map(|p| p.name.clone())
                    .collect();
                model.edit_mesh(|mesh, fe| {
                    let faces = mesh.transform_parts(parts, &transform);
                    fe.renumber_faces(&faces);
                });
                let what = TRANSFORM_KINDS
                    .iter()
                    .find(|(k, _)| *k == self.transform.kind)
                    .map_or("", |(_, label)| label);
                Ok(format!(
                    "{what}: {} ({nodes} nodes moved)",
                    names.join(", ")
                ))
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

fn vector_rows(
    ui: &mut Ui,
    vector: &mut [f64; 3],
    labels: [&str; 3],
    two_d: bool,
    quantity: Option<Quantity>,
    units: UnitSystem,
) {
    let count = if two_d { 2 } else { 3 };
    for (value, label) in vector.iter_mut().zip(labels).take(count) {
        ui.label(label);
        match quantity {
            Some(quantity) => {
                ui.add(numeric::quantity(value, units, quantity).speed(0.1));
            }
            None => {
                ui.add(numeric::drag_value(value).speed(0.1));
            }
        }
        ui.end_row();
    }
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
    match draft.kind {
        TransformKind::Translate => {
            vector_rows(
                ui,
                &mut draft.translation,
                ["dX", "dY", "dZ"],
                two_d,
                length,
                units,
            );
        }
        TransformKind::Rotate => {
            vector_rows(ui, &mut draft.point, ["X", "Y", "Z"], two_d, length, units);
            if two_d {
                ui.label("Axis");
                ui.label("Z");
                ui.end_row();
                draft.axis = [0.0, 0.0, 1.0];
            } else {
                ui.label("Axis");
                ui.horizontal(|ui| {
                    for a in draft.axis.iter_mut() {
                        ui.add(numeric::drag_value(a).speed(0.1));
                    }
                });
                ui.end_row();
            }
            ui.label("Angle");
            ui.add(
                numeric::drag_value(&mut draft.angle)
                    .speed(1.0)
                    .suffix(" °"),
            );
            ui.end_row();
        }
        TransformKind::Mirror => {
            vector_rows(ui, &mut draft.point, ["X", "Y", "Z"], two_d, length, units);
            ui.label("Normal");
            ui.horizontal(|ui| {
                let count = if two_d { 2 } else { 3 };
                for n in draft.normal.iter_mut().take(count) {
                    ui.add(numeric::drag_value(n).speed(0.1));
                }
            });
            ui.end_row();
            if two_d {
                draft.normal[2] = 0.0;
            }
            ui.label("");
            ui.weak("Mirrored elements are turned inside out again, so they stay valid.");
            ui.end_row();
        }
        TransformKind::Scale => {
            vector_rows(ui, &mut draft.center, ["X", "Y", "Z"], two_d, length, units);
            vector_rows(
                ui,
                &mut draft.factors,
                ["Factor X", "Factor Y", "Factor Z"],
                two_d,
                None,
                units,
            );
        }
    }
    ui.label("");
    ui.weak("Only the mesh moves; meshing the geometry again restores its position.");
    ui.end_row();
}
