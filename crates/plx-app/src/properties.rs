//! Properties of a tree item as a two-column grid. The grid does not care where it is shown:
//! today in the double-click dialog, it fits a side panel just as well.

use plx_mesh::{ElementShape, SurfaceDefinition};
use plx_model::{ModelSpace, element_type_options};

use crate::model::Model;
use crate::results::format_value;
use crate::tree::TreeItem;

/// Title of the properties window of an item.
pub fn title(model: Option<&Model>, item: &TreeItem) -> String {
    let name = match (item, model) {
        (TreeItem::Model, Some(model)) => model.file_name(),
        (TreeItem::Model, None) => "Model".into(),
        (TreeItem::Part(index), Some(model)) => model
            .parts
            .get(*index)
            .map_or_else(String::new, |p| p.name.clone()),
        (TreeItem::NodeSet(n) | TreeItem::ElementSet(n) | TreeItem::Surface(n), _) => n.clone(),
        (TreeItem::Field(f) | TreeItem::ResultFieldOutput(f), Some(model)) => field_name(model, *f),
        (TreeItem::Component(f, c), Some(model)) => {
            let field = model
                .results
                .as_ref()
                .and_then(|v| v.current_increment())
                .and_then(|i| i.fields.get(*f));
            field
                .and_then(|field| {
                    Some(format!(
                        "{}: {}",
                        field.name,
                        field.components.get(*c)?.name
                    ))
                })
                .unwrap_or_default()
        }
        _ => String::new(),
    };
    format!("Properties: {name}")
}

fn field_name(model: &Model, field: usize) -> String {
    model
        .results
        .as_ref()
        .and_then(|v| v.current_increment())
        .and_then(|i| i.fields.get(field))
        .map_or_else(String::new, |f| f.name.clone())
}

/// Element types of a part being edited in the properties: one per element type of the
/// part, in the order of [`crate::model::PartInfo::element_types`].
pub struct ElementTypes<'a> {
    pub types: &'a mut [String],
    pub space: ModelSpace,
}

/// Shows the properties of `item` in a grid; with `name`, the name can be edited, with
/// `types` the element types of a part.
pub fn show(
    ui: &mut egui::Ui,
    model: Option<&Model>,
    item: &TreeItem,
    name: Option<&mut String>,
    types: Option<ElementTypes>,
) {
    let mut name = name;
    let mut types = types;
    let mut type_row = 0;
    let mut rows: Vec<(&'static str, String)> = Vec::new();
    match model {
        Some(model) => rows_of(model, item, &mut rows),
        None => rows.push(("", "No model loaded".into())),
    }
    if rows.is_empty() {
        rows.push(("", "Not implemented yet".into()));
    }
    egui::Grid::new("properties")
        .max_col_width(crate::dock::column_limit(ui))
        .num_columns(2)
        .striped(true)
        .spacing([16.0, 4.0])
        .show(ui, |ui| {
            for (key, value) in rows {
                ui.label(key);
                let edited_type = match &mut types {
                    Some(edit) if key == "Element type" => {
                        type_row += 1;
                        let space = edit.space;
                        (edit.types.get_mut(type_row - 1)).map(|t| (t, space))
                    }
                    _ => None,
                };
                if let Some((type_name, space)) = edited_type {
                    element_type_combo(ui, type_row, type_name, space, &value);
                } else {
                    match name.as_deref_mut().filter(|_| key == "Name") {
                        Some(name) => {
                            let edit = egui::TextEdit::singleline(name).desired_width(200.0);
                            ui.add(edit);
                        }
                        None => {
                            ui.label(value);
                        }
                    }
                }
                ui.end_row();
            }
        });
}

/// The element type of a part: the CalculiX types with the same nodes, as a combo
/// box with the element count after it. `row` shows `type_name (count)`; a shape with one
/// type only keeps it as text.
fn element_type_combo(
    ui: &mut egui::Ui,
    index: usize,
    type_name: &mut String,
    space: ModelSpace,
    row: &str,
) {
    let count = row.rsplit_once(' ').map_or("", |(_, count)| count);
    let mut options = ElementShape::from_type_name(type_name)
        .map(|shape| element_type_options(shape, space))
        .unwrap_or_default();
    if !(options.iter()).any(|o| o.eq_ignore_ascii_case(type_name)) {
        options.insert(0, type_name.clone());
    }
    if options.len() < 2 {
        ui.label(row);
        return;
    }
    ui.horizontal(|ui| {
        egui::ComboBox::from_id_salt(("element type", index))
            .selected_text(type_name.as_str())
            .width(120.0)
            .show_ui(ui, |ui| {
                for option in options {
                    let selected = option.eq_ignore_ascii_case(type_name);
                    if ui.selectable_label(selected, &option).clicked() {
                        *type_name = option;
                    }
                }
            });
        ui.label(count);
    });
}

fn rows_of(model: &Model, item: &TreeItem, rows: &mut Vec<(&'static str, String)>) {
    let mesh = &model.mesh;
    match item {
        TreeItem::Group(_)
        | TreeItem::Mesh
        | TreeItem::FieldOutputs
        | TreeItem::Material(_)
        | TreeItem::Section(_)
        | TreeItem::CrossSection(_)
        | TreeItem::Constraint(_)
        | TreeItem::SurfaceInteraction(_)
        | TreeItem::ContactRegion(_)
        | TreeItem::ContactDetection(_)
        | TreeItem::Bolt(_)
        | TreeItem::ContactPair(_)
        | TreeItem::Tie(_)
        | TreeItem::NodeTie(_)
        | TreeItem::Amplitude(_)
        | TreeItem::Distribution(_)
        | TreeItem::InitialCondition(_)
        | TreeItem::Step(_)
        | TreeItem::StepGroup(..)
        | TreeItem::BoundaryCondition(..)
        | TreeItem::Load(..)
        | TreeItem::FieldOutput(..)
        | TreeItem::HistoryOutput(..)
        | TreeItem::DefinedField(..)
        | TreeItem::HotSpot(_)
        | TreeItem::ReferencePoint(_)
        | TreeItem::CoordinateSystem(_)
        | TreeItem::ResultPath(_)
        | TreeItem::Plane(_)
        | TreeItem::ResultPlane(_)
        | TreeItem::MeshItem(_)
        | TreeItem::HistorySet(_)
        | TreeItem::HistoryField(..)
        | TreeItem::HistoryComponent(..) => {}
        TreeItem::Model => {
            rows.push(("File", model.path.display().to_string()));
            rows.push(("Load time", format!("{} ms", model.load_time.as_millis())));
            rows.push(("Nodes", mesh.node_count().to_string()));
            rows.push(("Elements", mesh.element_count().to_string()));
            rows.push(("Parts", model.parts.len().to_string()));
            let mut types = std::collections::BTreeMap::<&str, usize>::new();
            for part in &model.parts {
                for (name, count) in &part.element_types {
                    *types.entry(name).or_default() += count;
                }
            }
            for (name, count) in types {
                rows.push(("Element type", format!("{name} ({count})")));
            }
            if let Some((min, max)) = mesh.bounds() {
                let size = [0, 1, 2].map(|k| {
                    let length = model.shown(max[k] - min[k], plx_model::Quantity::Length);
                    format_value(length as f32)
                });
                let unit = model.units().unit(plx_model::Quantity::Length);
                rows.push(("Dimensions", format!("{} {unit}", size.join(" × "))));
            }
            rows.push(("Node Sets", mesh.node_sets.len().to_string()));
            rows.push(("Element Sets", mesh.element_sets.len().to_string()));
            rows.push(("Surfaces", mesh.surfaces.len().to_string()));
        }
        TreeItem::Part(index) => {
            let Some(part) = model.parts.get(*index) else {
                return;
            };
            rows.push(("Name", part.name.clone()));
            rows.push(("Elements", part.element_count.to_string()));
            rows.push(("Nodes", part.node_count.to_string()));
            for (type_name, count) in &part.element_types {
                rows.push(("Element type", format!("{type_name} ({count})")));
            }
            let [r, g, b] = part.color.map(|c| (c * 255.0).round() as u8);
            rows.push(("Color", format!("RGB {r}, {g}, {b}")));
            rows.push(("Visible", if part.visible { "yes" } else { "no" }.into()));
        }
        TreeItem::NodeSet(name) => {
            rows.push(("Name", name.clone()));
            let count = mesh.node_sets.get(name).map_or(0, Vec::len);
            rows.push(("Nodes", count.to_string()));
        }
        TreeItem::ElementSet(name) => {
            rows.push(("Name", name.clone()));
            let count = mesh.element_sets.get(name).map_or(0, Vec::len);
            rows.push(("Elements", count.to_string()));
        }
        TreeItem::Surface(name) => {
            rows.push(("Name", name.clone()));
            match mesh.surfaces.get(name) {
                Some(SurfaceDefinition::ElementFaces(faces)) => {
                    rows.push(("Type", "Element faces".into()));
                    rows.push(("Faces", faces.len().to_string()));
                }
                Some(SurfaceDefinition::Nodes(nodes)) => {
                    rows.push(("Type", "Nodes".into()));
                    rows.push(("Nodes", nodes.len().to_string()));
                }
                None => {}
            }
        }
        TreeItem::Field(f) | TreeItem::ResultFieldOutput(f) => {
            let Some(view) = &model.results else { return };
            let Some(field) = view.current_increment().and_then(|i| i.fields.get(*f)) else {
                return;
            };
            rows.push(("Name", field.name.clone()));
            let names: Vec<&str> = (field.components.iter())
                .filter(|c| !c.out_of_plane)
                .map(|c| c.name.as_str())
                .collect();
            rows.push(("Components", names.join(", ")));
        }
        TreeItem::Component(f, c) => {
            let Some(view) = &model.results else { return };
            let Some(inc) = view.current_increment() else {
                return;
            };
            let Some(component) = inc
                .fields
                .get(*f)
                .and_then(|field| field.components.get(*c))
            else {
                return;
            };
            rows.push((
                "Field",
                format!("{}: {}", inc.fields[*f].name, component.name),
            ));
            if let Some(step) = view.current_step_index() {
                rows.push(("Step", view.step_title(step)));
            }
            let noun = view.current_step().map_or("Increment", |s| s.entry_noun());
            rows.push((noun, view.entry_label(view.increment)));
            let extreme = |pick_max: bool| {
                component
                    .values
                    .iter()
                    .enumerate()
                    .filter(|(_, v)| v.is_finite())
                    .reduce(|a, b| {
                        let better = if pick_max { b.1 > a.1 } else { b.1 < a.1 };
                        if better { b } else { a }
                    })
            };
            for (label, pick_max) in [("Maximum", true), ("Minimum", false)] {
                if let Some((index, value)) = extreme(pick_max) {
                    let node = mesh.node_ids()[index];
                    rows.push((label, format!("{} (node {node})", format_value(*value))));
                }
            }
            rows.push((
                "Computed",
                if component.derived {
                    "yes, by Nodalis"
                } else {
                    "no, from the file"
                }
                .into(),
            ));
        }
    }
}
