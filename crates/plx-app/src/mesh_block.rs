//! The editor of a mesh block of the project schematic: the parameters of a box or a
//! rectangle that the block meshes. Its mesh is made again by Update Project or
//! Generate Mesh, so that analyses that link it take the new one.

use plx_model::{MeshPrimitive, ModelSpace, Quantity, UnitSystem};

use crate::numeric;

pub enum MeshBlockResult {
    Open,
    /// Closed with OK: the parameters and the model space.
    Ok(MeshPrimitive, ModelSpace),
    Cancel,
}

pub struct MeshBlockEditor {
    /// The block edited.
    pub id: u64,
    name: String,
    primitive: MeshPrimitive,
    space: ModelSpace,
    /// The unit system of the project, which the lengths are shown and typed in.
    units: UnitSystem,
    /// Whether the model space may change: not while analyses link the mesh, which have
    /// the block's model space.
    space_fixed: bool,
}

impl MeshBlockEditor {
    pub fn new(
        id: u64,
        name: &str,
        primitive: MeshPrimitive,
        space: ModelSpace,
        units: UnitSystem,
        space_fixed: bool,
    ) -> Self {
        Self {
            id,
            name: name.to_owned(),
            primitive,
            space,
            units,
            space_fixed,
        }
    }

    pub fn rename(&mut self, name: &str) {
        name.clone_into(&mut self.name);
    }

    pub fn show(&mut self, ctx: &egui::Context) -> MeshBlockResult {
        let mut result = MeshBlockResult::Open;
        let mut open = true;
        egui::Window::new(format!("{} - {}", self.primitive.label(), self.name))
            .id(crate::ids::window(ctx, ("mesh block", self.id)))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                egui::Grid::new("mesh block form")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| self.form(ui));
                let problem = self.primitive.problem(self.space);
                match &problem {
                    Some(problem) => {
                        ui.colored_label(crate::style::error(), problem);
                    }
                    None => {
                        ui.weak(format!(
                            "{} elements {}; the sides are node sets and surfaces {}.",
                            self.primitive.element_count(),
                            self.primitive.element_type(self.space),
                            sides(&self.primitive)
                        ));
                    }
                }
                ui.separator();
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if ui.button("Cancel").clicked() {
                        result = MeshBlockResult::Cancel;
                    }
                    if ui
                        .add_enabled(problem.is_none(), egui::Button::new("OK"))
                        .on_hover_text(
                            "Keeps the parameters; Update Project or Generate Mesh makes the \
                             mesh anew.",
                        )
                        .clicked()
                    {
                        result = MeshBlockResult::Ok(self.primitive.clone(), self.space);
                    }
                });
            });
        if !open {
            result = MeshBlockResult::Cancel;
        }
        result
    }

    fn form(&mut self, ui: &mut egui::Ui) {
        let units = self.units;
        if let MeshPrimitive::Rectangle { .. } = self.primitive {
            ui.label("Model space");
            ui.add_enabled_ui(!self.space_fixed, |ui| {
                egui::ComboBox::from_id_salt("mesh block space")
                    .selected_text(self.space.label())
                    .width(200.0)
                    .show_ui(ui, |ui| {
                        for space in ModelSpace::ALL.into_iter().filter(|s| s.is_2d()) {
                            ui.selectable_value(&mut self.space, space, space.label())
                                .on_hover_ui(|ui| crate::model_space_sketch::hover(ui, space));
                        }
                    });
            })
            .response
            .on_disabled_hover_text(
                "Analyses link this mesh and have its model space; unlink them to change it.",
            );
            ui.end_row();
        }
        let axes = ["X", "Y", "Z"];
        match &mut self.primitive {
            MeshPrimitive::Box {
                origin,
                size,
                divisions,
                element,
            } => {
                vector_row(ui, "Origin", &axes, origin, units);
                vector_row(ui, "Edge lengths", &["Lx", "Ly", "Lz"], size, units);
                count_row(ui, &["nx", "ny", "nz"], divisions);
                ui.label("Element type");
                egui::ComboBox::from_id_salt("mesh block element")
                    .selected_text(element.as_str())
                    .show_ui(ui, |ui| {
                        for choice in MeshPrimitive::BOX_ELEMENTS {
                            ui.selectable_value(element, choice.to_owned(), choice);
                        }
                    });
                ui.end_row();
            }
            MeshPrimitive::Rectangle {
                origin,
                size,
                divisions,
                quadratic,
                reduced,
            } => {
                vector_row(ui, "Origin", &axes[..2], origin, units);
                vector_row(ui, "Edge lengths", &["Lx", "Ly"], size, units);
                count_row(ui, &["nx", "ny"], divisions);
                ui.label("Elements");
                ui.horizontal(|ui| {
                    ui.checkbox(quadratic, "Quadratic");
                    ui.checkbox(reduced, "Reduced integration");
                });
                ui.end_row();
            }
        }
    }
}

/// A row of lengths, one field per axis.
fn vector_row(
    ui: &mut egui::Ui,
    label: &str,
    axes: &[&str],
    values: &mut [f64],
    units: UnitSystem,
) {
    ui.label(label);
    ui.horizontal(|ui| {
        for (axis, value) in axes.iter().zip(values.iter_mut()) {
            ui.label(*axis);
            ui.add_sized(
                [90.0, 18.0],
                numeric::quantity(value, units, Quantity::Length),
            );
        }
    });
    ui.end_row();
}

/// The number of elements along each axis.
fn count_row(ui: &mut egui::Ui, axes: &[&str], counts: &mut [u32]) {
    ui.label("Elements along");
    ui.horizontal(|ui| {
        for (axis, count) in axes.iter().zip(counts.iter_mut()) {
            ui.label(*axis);
            ui.add_sized(
                [90.0, 18.0],
                crate::numeric::count(count).range(1..=10_000).speed(0.1),
            );
        }
    });
    ui.end_row();
}

/// The names of the sides of the mesh, as a list.
fn sides(primitive: &MeshPrimitive) -> String {
    let axes = match primitive {
        MeshPrimitive::Box { .. } => 3,
        MeshPrimitive::Rectangle { .. } => 2,
    };
    plx_mesh::structured::SIDES[..axes]
        .iter()
        .flatten()
        .copied()
        .collect::<Vec<_>>()
        .join(", ")
}
