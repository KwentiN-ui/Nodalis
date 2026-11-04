//! PrePoMax's "Model Properties" dialog: the model space, the unit system and the physical
//! constants, asked for when an analysis is made. Its model space and unit system stay as
//! they are then; the rest can be edited later from the model's context.

use std::collections::BTreeMap;

use crate::model::Model;
use plx_mesh::{ElementFamily, ElementShape, FeMesh};
use plx_mesher::CadEntity;
use plx_model::{
    BASE_QUANTITIES, DERIVED_QUANTITIES, ModelKind, ModelProperties, ModelSpace, Quantity,
    UnitSystem,
};

/// The open dialog with its draft.
pub struct ModelPropertiesDialog {
    pub draft: ModelProperties,
    /// Editing the properties of an analysis rather than making a new one: its model space
    /// and unit system are fixed then.
    pub editing: bool,
}

pub enum DialogResult {
    Open,
    Ok(ModelProperties),
    Cancel,
}

impl ModelPropertiesDialog {
    /// The properties of a new analysis, starting from `properties`.
    pub fn create(properties: ModelProperties) -> Self {
        Self {
            draft: properties,
            editing: false,
        }
    }

    pub fn edit(properties: ModelProperties) -> Self {
        Self {
            draft: properties,
            editing: true,
        }
    }

    /// `mesh` and `geometry`, the display of the CAD geometry, are those of the model being
    /// edited, which limit the model spaces.
    pub fn show(
        &mut self,
        ctx: &egui::Context,
        mesh: Option<&FeMesh>,
        geometry: Option<&Model>,
    ) -> DialogResult {
        let mut open = true;
        let mut result = DialogResult::Open;
        let error = (mesh.and_then(|mesh| space_error(self.draft.space, mesh)))
            .or_else(|| geometry.and_then(|g| geometry_check(self.draft.space, g).err()));
        let title = "Model Properties";
        let builder = egui::Window::new(title)
            .id(crate::ids::window(ctx, "model properties"))
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center());
        crate::dock::show(ctx, builder, title, &mut open, |ui| {
            ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
            ui.set_width(340.0);
            let draft = &mut self.draft;
            let fixed = self.editing;
            group(ui, "Model Space", |ui| {
                egui::Grid::new("model space")
                    .num_columns(2)
                    .spacing([24.0, 4.0])
                    .show(ui, |ui| {
                        for pair in ModelSpace::ALL.chunks(2) {
                            for &space in pair {
                                let radio =
                                    egui::RadioButton::new(draft.space == space, space.label());
                                let clicked = ui
                                    .add_enabled(!fixed || draft.space == space, radio)
                                    .on_hover_ui(|ui| crate::model_space_sketch::hover(ui, space))
                                    .on_disabled_hover_ui(|ui| {
                                        crate::model_space_sketch::hover(ui, space)
                                    })
                                    .clicked();
                                if clicked && !fixed {
                                    draft.space = space;
                                }
                            }
                            ui.end_row();
                        }
                    });
            });
            group(ui, "Model Type", |ui| model_type(ui, draft));
            group(ui, "Unit System", |ui| {
                egui::Frame::new()
                    .fill(crate::style::window())
                    .stroke(egui::Stroke::new(1.0, crate::style::border()))
                    .inner_margin(4)
                    .show(ui, |ui| {
                        ui.set_min_width(ui.available_width());
                        for units in UnitSystem::ALL {
                            let selected = draft.units == units;
                            let label = egui::Button::selectable(selected, units.label());
                            let clicked = ui.add_enabled(!fixed || selected, label).clicked();
                            if clicked && !fixed {
                                draft.units = units;
                            }
                        }
                    });
            });
            group(ui, "Units", |ui| units_table(ui, draft.units));
            group(ui, "Physical Constants", |ui| constants(ui, draft));
            let note = if fixed {
                "The model space and the unit system are set when the analysis is made."
            } else {
                "The model space and the unit system cannot be changed once the analysis \
                     is made. The unit system only says in which units the analysis shows \
                     and reads its values; they are stored in SI units."
            };
            ui.add(egui::Label::new(egui::RichText::new(note).weak()).wrap());
            if let Some(error) = &error {
                ui.add(
                    egui::Label::new(egui::RichText::new(error).color(crate::style::error()))
                        .wrap(),
                );
            }
            ui.add_space(4.0);
            crate::dock::footer(ui, |ui| {
                if ui.button("Cancel").clicked() {
                    result = DialogResult::Cancel;
                }
                let ok = ui.add_enabled(error.is_none(), egui::Button::new("OK"));
                if ok.clicked() {
                    result = DialogResult::Ok(draft.clone());
                }
            });
        });
        if error.is_none() && ctx.input(|i| i.key_pressed(egui::Key::Enter)) {
            result = DialogResult::Ok(self.draft.clone());
        }
        if !open || ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            result = DialogResult::Cancel;
        }
        result
    }
}

/// PrePoMax's model type. A submodel takes the results of its global model through a link
/// in the project, not from a file picked here.
fn model_type(ui: &mut egui::Ui, draft: &mut ModelProperties) {
    ui.horizontal_wrapped(|ui| {
        for kind in ModelKind::ALL {
            ui.radio_value(&mut draft.kind, kind, kind.label());
        }
    });
    if draft.kind != ModelKind::Submodel {
        return;
    }
    let note = "The submodel takes the displacements of its global model through the Global \
                results row of its block in the project window: drag the results of a step of \
                the global analysis there.";
    ui.add(egui::Label::new(egui::RichText::new(note).weak()).wrap());
}

/// Why the model space does not fit the existing mesh: a 2D model takes no solid elements,
/// a 3D model no plane ones.
pub fn space_error(space: ModelSpace, mesh: &FeMesh) -> Option<String> {
    let has = |family| mesh.elements().iter().any(|e| e.shape.family() == family);
    if space.is_2d() && has(ElementFamily::Solid) {
        Some("The mesh contains solid elements; a 2D model works only with plane elements.".into())
    } else if !space.is_2d() && mesh.elements().iter().any(|e| e.is_plane()) {
        Some("The mesh contains 2D elements; a 3D model does not work with them.".into())
    } else {
        None
    }
}

/// Checks CAD geometry for a 2D model space: it has to be faces in the x-y plane, for an
/// axisymmetric model on the side x >= 0 of the axis of revolution. Returns the faces whose
/// normal points to -z; their elements are turned round when meshed.
pub fn geometry_check(space: ModelSpace, geometry: &Model) -> Result<Vec<i32>, String> {
    if !space.is_2d() {
        return Ok(Vec::new());
    }
    if geometry.geometry_solids > 0 {
        return Err(format!(
            "The geometry contains {} solids; a 2D model needs faces in the \
             x-y plane.",
            geometry.geometry_solids
        ));
    }
    let Some((min, max)) = geometry.mesh.bounds() else {
        return Ok(Vec::new());
    };
    let diagonal = (0..3)
        .map(|k| (max[k] - min[k]).powi(2))
        .sum::<f64>()
        .sqrt();
    let tolerance = 1e-6 * diagonal.max(f64::MIN_POSITIVE);
    if min[2].abs() > tolerance || max[2].abs() > tolerance {
        return Err(format!(
            "The geometry does not lie in the x-y plane (z from {:.4} to {:.4}); a \
             2D model needs faces at z = 0.",
            min[2], max[2]
        ));
    }
    if space == ModelSpace::Axisymmetric && min[0] < -tolerance {
        return Err(format!(
            "The geometry reaches to x = {:.4}; an axisymmetric model lies entirely \
             at x >= 0, the y-axis is the axis of revolution.",
            min[0]
        ));
    }
    // The display triangles of a face follow its orientation.
    let mut turn: BTreeMap<i32, f64> = BTreeMap::new();
    for element in geometry.mesh.elements() {
        let (Some(CadEntity::Face(face)), ElementShape::Tri3) =
            (geometry.cad_entity(element.id), element.shape)
        else {
            continue;
        };
        let points: Vec<[f64; 3]> = (element.nodes.iter())
            .filter_map(|&id| geometry.mesh.node(id))
            .collect();
        if let [a, b, c] = points[..] {
            let cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
            *turn.entry(face).or_default() += cross;
        }
    }
    Ok((turn.into_iter())
        .filter(|&(_, area)| area < 0.0)
        .map(|(face, _)| face)
        .collect())
}

/// A titled frame like a Windows group box.
fn group(ui: &mut egui::Ui, title: &str, contents: impl FnOnce(&mut egui::Ui)) {
    ui.add_space(4.0);
    ui.strong(title);
    egui::Frame::new()
        .stroke(egui::Stroke::new(1.0, crate::style::border()))
        .inner_margin(6)
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            contents(ui);
        });
}

/// Absolute zero and the Stefan-Boltzmann constant, which radiation needs; undefined as in
/// PrePoMax until the user sets them.
fn constants(ui: &mut egui::Ui, draft: &mut ModelProperties) {
    let units = draft.units;
    egui::Grid::new("physical constants")
        .num_columns(2)
        .spacing([12.0, 4.0])
        .show(ui, |ui| {
            for (label, value, quantity) in [
                (
                    "Absolute zero",
                    &mut draft.absolute_zero,
                    Quantity::Temperature,
                ),
                (
                    "Stefan-Boltzmann",
                    &mut draft.stefan_boltzmann,
                    Quantity::StefanBoltzmann,
                ),
            ] {
                let mut set = value.is_some();
                ui.checkbox(&mut set, label);
                let mut number = value.unwrap_or(0.0);
                ui.add_enabled(set, crate::numeric::physical(&mut number, units, quantity));
                *value = set.then_some(number);
                ui.end_row();
            }
        });
    if let Some((zero, sigma)) = ModelProperties::standard_constants(units)
        && ui
            .button("Defaults")
            .on_hover_text("Absolute zero and Stefan-Boltzmann constant in the unit system")
            .clicked()
    {
        draft.absolute_zero = Some(zero);
        draft.stefan_boltzmann = Some(sigma);
    }
    ui.add(
        egui::Label::new(egui::RichText::new("Only needed for thermal radiation.").weak()).wrap(),
    );
}

/// The units of the system, PrePoMax's base and derived units.
fn units_table(ui: &mut egui::Ui, units: UnitSystem) {
    egui::ScrollArea::vertical()
        .max_height(220.0)
        .auto_shrink([false, true])
        .show(ui, |ui| {
            for (title, quantities, values) in [
                ("Base Units", &BASE_QUANTITIES[..], &units.base_units()[..]),
                (
                    "Derived Units",
                    &DERIVED_QUANTITIES[..],
                    &units.derived_units()[..],
                ),
            ] {
                egui::CollapsingHeader::new(title)
                    .default_open(true)
                    .show(ui, |ui| {
                        egui::Grid::new(title)
                            .num_columns(2)
                            .striped(true)
                            .spacing([24.0, 2.0])
                            .show(ui, |ui| {
                                for (quantity, unit) in quantities.iter().zip(values) {
                                    ui.label(quantity.label());
                                    ui.label(*unit);
                                    ui.end_row();
                                }
                            });
                    });
            }
        });
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_mesh::{Element, ElementShape};

    #[test]
    fn the_model_space_has_to_fit_the_mesh() {
        let mut mesh = FeMesh::default();
        assert!(space_error(ModelSpace::Axisymmetric, &mesh).is_none());
        mesh.add_element(Element {
            id: 1,
            type_name: "C3D4".into(),
            shape: ElementShape::Tet4,
            nodes: vec![1, 2, 3, 4],
        })
        .unwrap();
        assert!(space_error(ModelSpace::ThreeD, &mesh).is_none());
        assert!(space_error(ModelSpace::PlaneStress, &mesh).is_some());
        let mut plane = FeMesh::default();
        plane
            .add_element(Element {
                id: 1,
                type_name: "CPE3".into(),
                shape: ElementShape::Tri3,
                nodes: vec![1, 2, 3],
            })
            .unwrap();
        assert!(space_error(ModelSpace::Axisymmetric, &plane).is_none());
        assert!(space_error(ModelSpace::ThreeD, &plane).is_some());
    }

    /// A geometry display of two triangles, face 1 counter-clockwise and face 2 clockwise
    /// seen from +z, moved by `shift`.
    fn two_faces(shift: [f64; 3], solids: usize) -> Model {
        let mut mesh = FeMesh::default();
        let points = [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0]];
        for (id, [x, y]) in (1..).zip(points) {
            mesh.set_node(id, [x + shift[0], y + shift[1], shift[2]]);
        }
        for (id, nodes) in [(1, vec![1, 2, 3]), (2, vec![2, 3, 4])] {
            mesh.add_element(Element {
                id,
                type_name: "S3".into(),
                shape: ElementShape::Tri3,
                nodes,
            })
            .unwrap();
        }
        let display = plx_mesher::GeometryDisplay {
            mesh,
            entities: vec![CadEntity::Face(1), CadEntity::Face(2)],
            part_dimensions: vec![2, 2],
            solids,
            faces: 2,
            edges: 0,
            topology: Default::default(),
        };
        Model::geometry_view(std::path::Path::new("flaechen.brep"), display)
    }

    #[test]
    fn two_d_geometry_lies_in_the_x_y_plane_and_reversed_faces_are_found() {
        let flat = two_faces([0.0; 3], 0);
        assert_eq!(geometry_check(ModelSpace::PlaneStress, &flat), Ok(vec![2]));
        assert_eq!(geometry_check(ModelSpace::ThreeD, &flat), Ok(Vec::new()));
        let raised = two_faces([0.0, 0.0, 0.5], 0);
        assert!(geometry_check(ModelSpace::PlaneStrain, &raised).is_err());
        let left = two_faces([-0.5, 0.0, 0.0], 0);
        assert!(geometry_check(ModelSpace::PlaneStress, &left).is_ok());
        assert!(geometry_check(ModelSpace::Axisymmetric, &left).is_err());
        let solid = two_faces([0.0; 3], 1);
        assert!(geometry_check(ModelSpace::Axisymmetric, &solid).is_err());
        assert!(geometry_check(ModelSpace::ThreeD, &solid).is_ok());
    }
}
