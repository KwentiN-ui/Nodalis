//! The "Create History Output" dialog of the Results tree: values of a field at
//! nodes over all increments, an equation of other history outputs, element sizes, or the
//! values of a hot spot beside the position of a moving load.

use egui::{RichText, Ui};
use plx_model::next_name;
use plx_results::history_output::{HistoryOutput, HistoryOutputKind, SizeKind};

use crate::field_output_dialog::DialogAction;
use crate::model::{Highlight, Hit, Model};
use crate::results::ResultsView;
use crate::selection::{Operation, Picker, Target};
use crate::setup::{FACE_SOURCES, NODE_SOURCES, RegionDraft, SOLID_SOURCES};
use crate::tree_rows::{frame, tree_row};
use crate::viewport::{BoxSelect, Preview};

/// The types of the list, in order, with the prefix of their default names.
const TYPES: [(&str, &str); 4] = [
    ("From Field Output", "From_Field"),
    ("From History Output by Equation", "From_Equation"),
    ("From Element Size", "From_Element_Size"),
    ("From Hot Spot", "From_Hot_Spot"),
];
const FROM_FIELD: usize = 0;
const EQUATION: usize = 1;
const SIZE: usize = 2;
const HOT_SPOT: usize = 3;

const NAME: &str = "Name";
const REGION: &str = "Region";
const FIELD: &str = "Field";
const COMPONENTS: &str = "Components";
const EQUATION_ROW: &str = "Equation";
const UNIT: &str = "Unit";
const SIZE_ROW: &str = "Size";
const HOT_SPOT_ROW: &str = "Hot spot";
const MOVING_LOAD_ROW: &str = "Moving load";
const NO_MOVING_LOAD: &str = "None";

fn description(row: &str) -> &'static str {
    match row {
        NAME => "Name of the history output.",
        REGION => {
            "Nodes whose values are output over all increments. With \"Selection in the \
             3D view\" the nodes are picked on the deformed model."
        }
        FIELD => "Field the values are taken from.",
        COMPONENTS => "Components of the field that are output.",
        EQUATION_ROW => {
            "Example: =[From_Field-1.DISP.U3] * 2\nA component is given as \
             HistoryOutput.Field.Component, names with a hyphen in square \
             brackets. The equation is evaluated for every entry and every increment; \
             all components need the same number of entries."
        }
        UNIT => "User-defined unit of the history output.",
        SIZE_ROW => {
            "Volume of the elements or area of the element faces, per increment on the \
             deformed mesh (true scale)."
        }
        HOT_SPOT_ROW => {
            "Hot spot whose values are output over all increments: per toe node the hot spot \
             stress S_HS and the stresses S_1, S_2, ... at the read-out points; with several \
             toe nodes also their maximum (MAX)."
        }
        MOVING_LOAD_ROW => {
            "Moving load of the FE model whose position along its path is output as the \
             first column, one per step, to plot the stress over the position. Steps \
             without the load have no position."
        }
        _ => "",
    }
}

pub struct HistoryOutputDialog {
    /// Index of the edited output; `None` creates a new one.
    pub edit: Option<usize>,
    kind: usize,
    names: [String; 4],
    nodes: RegionDraft,
    field: String,
    components: Vec<String>,
    equation: String,
    unit: String,
    size: SizeKind,
    elements: RegionDraft,
    hot_spot: String,
    moving_load: Option<String>,
    focus: &'static str,
    pub error: Option<String>,
    picker: Picker,
    /// Fields of the results with their components.
    fields: Vec<(String, Vec<String>)>,
    /// `Set.Field.Component` of the history outputs an equation may use.
    history: Vec<String>,
    taken: Vec<String>,
    /// Hot spots of the results and moving loads of the FE model, by name.
    hot_spots: Vec<String>,
    moving_loads: Vec<String>,
}

impl HistoryOutputDialog {
    pub fn create(view: &ResultsView) -> Self {
        let fields: Vec<(String, Vec<String>)> = view
            .current_increment()
            .map(|inc| {
                (inc.fields.iter())
                    .filter(|f| !f.components.is_empty())
                    .map(|f| {
                        let components = f.components.iter().map(|c| c.name.clone()).collect();
                        (f.name.clone(), components)
                    })
                    .collect()
            })
            .unwrap_or_default();
        let taken: Vec<String> = view
            .history_outputs
            .iter()
            .map(|o| o.name.clone())
            .collect();
        let history = view
            .history
            .iter()
            .flat_map(|set| {
                set.fields.iter().flat_map(move |f| {
                    (f.components.iter())
                        .map(move |c| format!("{}.{}.{}", set.name, f.name, c.name))
                })
            })
            .collect();
        let (field, components) = fields
            .iter()
            .find(|(f, _)| f == "DISP")
            .or(fields.first())
            .map(|(f, c)| (f.clone(), vec![c[0].clone()]))
            .unwrap_or_default();
        let names = TYPES.map(|(_, prefix)| next_name(prefix, taken.iter().map(String::as_str)));
        let hot_spots: Vec<String> = (view.hot_spot_reports.iter())
            .map(|r| r.name.clone())
            .collect();
        let moving_loads = view.moving_loads.iter().map(|m| m.load.clone()).collect();
        Self {
            edit: None,
            kind: FROM_FIELD,
            names,
            nodes: RegionDraft::new(NODE_SOURCES, Target::Nodes),
            field,
            components,
            equation: "=".into(),
            unit: "/".into(),
            size: SizeKind::Volume,
            elements: RegionDraft::new(SOLID_SOURCES, Target::Faces),
            hot_spot: hot_spots.first().cloned().unwrap_or_default(),
            moving_load: None,
            focus: NAME,
            error: None,
            picker: Picker::default(),
            fields,
            history,
            taken,
            hot_spots,
            moving_loads,
        }
    }

    pub fn edit(view: &ResultsView, mesh: &plx_mesh::FeMesh, index: usize) -> Option<Self> {
        let output = view.history_outputs.get(index)?.clone();
        let mut dialog = Self::create(view);
        dialog.edit = Some(index);
        dialog.taken.retain(|n| *n != output.name);
        // An equation may only use the outputs before it.
        let before: Vec<&str> = (view.history_outputs[..index].iter())
            .map(|o| o.name.as_str())
            .collect();
        dialog
            .history
            .retain(|h| before.iter().any(|b| h.starts_with(&format!("{b}."))));
        dialog.kind = match output.kind {
            HistoryOutputKind::FromField {
                region,
                field,
                components,
            } => {
                dialog.nodes = RegionDraft::from_region(&region, NODE_SOURCES, Target::Nodes, mesh);
                dialog.field = field;
                dialog.components = components;
                FROM_FIELD
            }
            HistoryOutputKind::FromEquation { equation, unit } => {
                dialog.equation = equation;
                dialog.unit = unit;
                EQUATION
            }
            HistoryOutputKind::ElementSize { region, kind } => {
                dialog.size = kind;
                let sources = match kind {
                    SizeKind::Volume => SOLID_SOURCES,
                    SizeKind::Area => FACE_SOURCES,
                };
                dialog.elements = RegionDraft::from_region(&region, sources, Target::Faces, mesh);
                SIZE
            }
            HistoryOutputKind::FromHotSpot {
                hot_spot,
                moving_load,
            } => {
                dialog.hot_spot = hot_spot;
                dialog.moving_load = moving_load;
                HOT_SPOT
            }
        };
        dialog.names[dialog.kind] = output.name;
        Some(dialog)
    }

    pub fn title(&self) -> &'static str {
        if self.edit.is_some() {
            "Edit History Output"
        } else {
            "Create History Output"
        }
    }

    /// The region being picked in the 3D view, if any.
    fn region(&self) -> Option<&RegionDraft> {
        match self.kind {
            FROM_FIELD => Some(&self.nodes),
            SIZE => Some(&self.elements),
            _ => None,
        }
    }

    fn region_mut(&mut self) -> Option<&mut RegionDraft> {
        match self.kind {
            FROM_FIELD => Some(&mut self.nodes),
            SIZE => Some(&mut self.elements),
            _ => None,
        }
    }

    /// Whether clicks in the 3D view pick for this dialog.
    pub fn picks(&self) -> bool {
        self.region().is_some_and(RegionDraft::picks)
    }

    pub fn click(&mut self, model: &Model, pick: Option<(&Hit, f32)>, operation: Operation) {
        let picker = std::mem::take(&mut self.picker);
        if let Some(region) = self.region_mut() {
            region.click(model, &picker, pick, operation);
        }
        self.picker = picker;
    }

    pub fn box_select(&mut self, model: &Model, area: &BoxSelect, operation: Operation) {
        let picker = std::mem::take(&mut self.picker);
        if let Some(region) = self.region_mut() {
            region.box_select(model, &picker, area, operation);
        }
        self.picker = picker;
    }

    pub fn preview(&self, model: &Model, hit: &Hit, precision: f32) -> Preview {
        self.region()
            .map(|r| r.preview(model, &self.picker, hit, precision))
            .unwrap_or_default()
    }

    pub fn highlight(&self, model: &Model) -> Highlight {
        self.region()
            .map_or_else(Highlight::default, |r| r.highlight(model))
    }

    /// The output as the dialog defines it, or why it cannot be created.
    pub fn output(&self) -> Result<HistoryOutput, String> {
        let name = self.names[self.kind].trim().to_string();
        if name.is_empty() {
            return Err("Please enter a name.".into());
        }
        if self.taken.iter().any(|t| t.eq_ignore_ascii_case(&name)) {
            return Err(format!("The name {name} is already used."));
        }
        if name.contains('.') {
            return Err("The name must not contain a dot.".into());
        }
        if self.region().is_some_and(RegionDraft::is_empty) {
            return Err("The region is empty.".into());
        }
        let kind = match self.kind {
            FROM_FIELD => {
                if self.fields.is_empty() {
                    return Err("There are no field outputs to take values from.".into());
                }
                HistoryOutputKind::FromField {
                    region: self.nodes.region(),
                    field: self.field.clone(),
                    components: self.components.clone(),
                }
            }
            EQUATION => HistoryOutputKind::FromEquation {
                equation: self.equation.clone(),
                unit: self.unit.clone(),
            },
            HOT_SPOT => {
                if self.hot_spot.is_empty() {
                    return Err("There are no hot spots. Create one first.".into());
                }
                HistoryOutputKind::FromHotSpot {
                    hot_spot: self.hot_spot.clone(),
                    moving_load: self.moving_load.clone(),
                }
            }
            _ => HistoryOutputKind::ElementSize {
                region: self.elements.region(),
                kind: self.size,
            },
        };
        Ok(HistoryOutput { name, kind })
    }

    /// Prepares the dialog for the next output after OK - New.
    pub fn next(&mut self, created: &HistoryOutput, view: &ResultsView) {
        self.taken.push(created.name.clone());
        self.names =
            TYPES.map(|(_, prefix)| next_name(prefix, self.taken.iter().map(String::as_str)));
        if let Some(set) = view.history.iter().find(|s| s.name == created.name) {
            for f in &set.fields {
                for c in &f.components {
                    self.history
                        .push(format!("{}.{}.{}", set.name, f.name, c.name));
                }
            }
        }
        self.error = None;
    }

    pub fn show(&mut self, ctx: &egui::Context, model: &Model) -> DialogAction<HistoryOutput> {
        let mut action = DialogAction::Open;
        let mut open = true;
        let window = egui::Window::new(self.title())
            .id(crate::ids::window(ctx, "history output dialog"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .default_width(520.0)
            .default_pos(ctx.content_rect().left_top() + egui::vec2(300.0, 90.0))
            .show(ctx, |ui| {
                ui.set_width(500.0);
                ui.label("Type");
                frame().show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.set_min_height(58.0);
                    for (index, (label, _)) in TYPES.iter().enumerate() {
                        let selected = self.kind == index;
                        let row = ui.add_enabled_ui(self.edit.is_none() || selected, |ui| {
                            tree_row(ui, 0, None, selected, RichText::new(*label))
                        });
                        if row.inner.clicked() && !selected {
                            self.kind = index;
                            self.focus = NAME;
                            self.error = None;
                        }
                    }
                });
                ui.add_space(6.0);
                ui.label("Properties");
                frame().show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.set_min_height(240.0);
                    self.properties(ui, model);
                });
                ui.add_space(4.0);
                egui::Frame::group(ui.style()).show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.set_height(76.0);
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        ui.set_width(ui.available_width());
                        ui.strong(self.focus);
                        ui.label(description(self.focus));
                    });
                });
                if let Some(error) = &self.error {
                    ui.colored_label(crate::style::error(), error);
                }
                ui.add_space(4.0);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if ui.button("Cancel").clicked() {
                        action = DialogAction::Cancel;
                    }
                    let ok = ui.button("OK").clicked();
                    let next = self.edit.is_none() && ui.button("OK - New").clicked();
                    if ok || next {
                        match self.output() {
                            Ok(output) => action = DialogAction::Ok { output, next },
                            Err(error) => self.error = Some(error),
                        }
                    }
                });
            });
        if let Some(window) = window {
            let mut picker = std::mem::take(&mut self.picker);
            if let Some(region) = self.region_mut() {
                region.picker_window(ctx, window.response.rect, &mut picker, model);
            }
            self.picker = picker;
        }
        if !open {
            action = DialogAction::Cancel;
        }
        action
    }

    fn properties(&mut self, ui: &mut Ui, model: &Model) {
        let header = egui::Frame::new()
            .fill(crate::style::control())
            .inner_margin(egui::Margin::symmetric(4, 1));
        header.show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.strong("Data");
        });
        let focus = &mut self.focus;
        egui::Grid::new("history output properties")
            .num_columns(2)
            .striped(true)
            .min_col_width(100.0)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                row(ui, focus, NAME, |ui| {
                    let edit =
                        egui::TextEdit::singleline(&mut self.names[self.kind]).desired_width(200.0);
                    ui.add(edit).has_focus()
                });
                match self.kind {
                    FROM_FIELD => {
                        // The region editor writes its own row.
                        let before = self.nodes.clone();
                        self.nodes.ui(ui, model);
                        if self.nodes != before {
                            *focus = REGION;
                        }
                        let fields = &self.fields;
                        let field = &mut self.field;
                        let components = &mut self.components;
                        row(ui, focus, FIELD, |ui| {
                            let mut changed = false;
                            let response = egui::ComboBox::from_id_salt("history field")
                                .selected_text(field.as_str())
                                .width(200.0)
                                .show_ui(ui, |ui| {
                                    for (name, _) in fields {
                                        changed |= ui
                                            .selectable_value(field, name.clone(), name)
                                            .changed();
                                    }
                                });
                            if changed {
                                let available = fields.iter().find(|(f, _)| f == field);
                                *components = available
                                    .map(|(_, c)| vec![c[0].clone()])
                                    .unwrap_or_default();
                            }
                            response.response.clicked()
                        });
                        row(ui, focus, COMPONENTS, |ui| {
                            let available = fields
                                .iter()
                                .find(|(f, _)| f == field)
                                .map_or(&[][..], |(_, c)| c.as_slice());
                            let mut clicked = false;
                            ui.vertical(|ui| {
                                for name in available {
                                    let mut on = components.contains(name);
                                    if ui.checkbox(&mut on, name).changed() {
                                        clicked = true;
                                        components.retain(|c| c != name);
                                        if on {
                                            components.push(name.clone());
                                            // Keep the order of the field.
                                            components.sort_by_key(|c| {
                                                available.iter().position(|a| a == c)
                                            });
                                        }
                                    }
                                }
                            });
                            clicked
                        });
                    }
                    EQUATION => {
                        row(ui, focus, EQUATION_ROW, |ui| {
                            let edit =
                                egui::TextEdit::singleline(&mut self.equation).desired_width(200.0);
                            ui.add(edit).has_focus()
                        });
                        row(ui, focus, UNIT, |ui| {
                            let edit =
                                egui::TextEdit::singleline(&mut self.unit).desired_width(200.0);
                            ui.add(edit).has_focus()
                        });
                    }
                    HOT_SPOT => {
                        let hot_spots = &self.hot_spots;
                        let hot_spot = &mut self.hot_spot;
                        row(ui, focus, HOT_SPOT_ROW, |ui| {
                            let shown = if hot_spot.is_empty() {
                                "(no hot spots)"
                            } else {
                                hot_spot.as_str()
                            };
                            let response = egui::ComboBox::from_id_salt("history hot spot")
                                .selected_text(shown)
                                .width(200.0)
                                .show_ui(ui, |ui| {
                                    for name in hot_spots {
                                        ui.selectable_value(hot_spot, name.clone(), name);
                                    }
                                });
                            response.response.clicked()
                        });
                        let moving_loads = &self.moving_loads;
                        let moving_load = &mut self.moving_load;
                        row(ui, focus, MOVING_LOAD_ROW, |ui| {
                            let response = egui::ComboBox::from_id_salt("history moving load")
                                .selected_text(moving_load.as_deref().unwrap_or(NO_MOVING_LOAD))
                                .width(200.0)
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(moving_load, None, NO_MOVING_LOAD);
                                    for name in moving_loads {
                                        ui.selectable_value(moving_load, Some(name.clone()), name);
                                    }
                                });
                            response.response.clicked()
                        });
                    }
                    _ => {
                        let size = &mut self.size;
                        let elements = &mut self.elements;
                        row(ui, focus, SIZE_ROW, |ui| {
                            let mut clicked = false;
                            ui.horizontal(|ui| {
                                for (kind, label) in
                                    [(SizeKind::Volume, "Volume"), (SizeKind::Area, "Area")]
                                {
                                    if ui.radio(*size == kind, label).clicked() && *size != kind {
                                        *size = kind;
                                        clicked = true;
                                        let sources = match kind {
                                            SizeKind::Volume => SOLID_SOURCES,
                                            SizeKind::Area => FACE_SOURCES,
                                        };
                                        *elements = RegionDraft::new(sources, Target::Faces);
                                    }
                                }
                            });
                            clicked
                        });
                        self.elements.ui(ui, model);
                    }
                }
            });
        if self.kind == EQUATION {
            ui.add_space(4.0);
            let equation = &mut self.equation;
            ui.add_enabled_ui(!self.history.is_empty(), |ui| {
                ui.menu_button("Insert Component…", |ui| {
                    for name in &self.history {
                        if ui.button(name).clicked() {
                            equation.push_str(&format!("[{name}]"));
                            ui.close();
                        }
                    }
                });
            });
            if self.history.is_empty() {
                ui.weak("There are no history outputs yet that the equation can use.");
            }
        }
    }
}

/// One row of the property grid; clicking its label or using its editor focuses it.
fn row(
    ui: &mut Ui,
    focus: &mut &'static str,
    label: &'static str,
    editor: impl FnOnce(&mut Ui) -> bool,
) {
    let response = ui.add(egui::Label::new(label).sense(egui::Sense::click()));
    if editor(ui) || response.clicked() {
        *focus = label;
    }
    ui.end_row();
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_model::Region;

    fn results() -> Model {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/kragbalken_c3d8.frd");
        crate::model::load(&path, plx_model::UnitSystem::Unitless)
            .unwrap()
            .model
    }

    #[test]
    fn values_at_picked_nodes_then_an_equation_of_them() {
        let mut model = results();
        let view = model.results.as_ref().unwrap();
        let mut dialog = HistoryOutputDialog::create(view);
        assert_eq!(dialog.names[FROM_FIELD], "From_Field-1");
        assert_eq!(dialog.field, "DISP");
        assert!(dialog.output().is_err(), "nothing picked yet");
        dialog.nodes = RegionDraft::from_region(
            &Region::Nodes(vec![1, 99]),
            NODE_SOURCES,
            Target::Nodes,
            &model.mesh,
        );
        let output = dialog.output().unwrap();
        let mesh = model.mesh.clone();
        let view = model.results.as_mut().unwrap();
        view.set_history_output(None, output.clone(), &mesh)
            .unwrap();
        assert_eq!(view.history[0].fields[0].components[0].entries.len(), 2);

        dialog.next(&output, view);
        dialog.kind = EQUATION;
        assert_eq!(dialog.names[EQUATION], "From_Equation-1");
        assert_eq!(dialog.history, ["From_Field-1.DISP.ALL"]);
        dialog.equation = "=[From_Field-1.DISP.ALL] * 2".into();
        view.set_history_output(None, dialog.output().unwrap(), &mesh)
            .unwrap();
        assert_eq!(view.history.len(), 2);

        // Editing the first one keeps its region; deleting it breaks the equation.
        let edit = HistoryOutputDialog::edit(view, &mesh, 0).unwrap();
        assert_eq!(edit.nodes.region(), Region::Nodes(vec![1, 99]));
        assert!(edit.history.is_empty());
        let warnings = view.remove_history_output(0, &mesh);
        assert_eq!(warnings.len(), 1);
        assert!(view.history.is_empty());
    }

    #[test]
    fn hot_spot_values_beside_a_moving_load() {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/kragbalken_c3d20r.frd");
        let mut model = crate::model::load(&path, plx_model::UnitSystem::Unitless)
            .unwrap()
            .model;
        // A toe on the top face near the fixed end, the paths along the beam.
        let toe = (model.mesh.node_ids().iter().zip(model.mesh.coords()))
            .find(|(_, c)| **c == [10.0, 5.0, 10.0])
            .map(|(id, _)| *id)
            .unwrap();
        let mut hot_spot = plx_results::hot_spot::HotSpot::new("Hot_Spot-1");
        hot_spot.toe = Region::Nodes(vec![toe]);
        hot_spot.thickness = 10.0;
        model.hot_spots.definitions.push(hot_spot);
        let reports = crate::hot_spots::evaluate(&model);
        let stations = plx_model::LoadStations {
            load: "Crane-1".into(),
            steps: vec![(1, 50.0)],
        };
        let mesh = model.mesh.clone();
        let view = model.results.as_mut().unwrap();
        view.set_history_sources(reports, vec![stations], &mesh);

        let mut dialog = HistoryOutputDialog::create(view);
        dialog.kind = HOT_SPOT;
        assert_eq!(dialog.names[HOT_SPOT], "From_Hot_Spot-1");
        assert_eq!(dialog.hot_spot, "Hot_Spot-1");
        assert_eq!(dialog.moving_loads, ["Crane-1"]);
        dialog.moving_load = Some("Crane-1".into());
        let output = dialog.output().unwrap();
        view.set_history_output(None, output, &mesh).unwrap();
        let set = view.history.last().unwrap();
        let node = &set.field("HOT_SPOT").unwrap().components[0];
        assert_eq!(node.name, format!("NODE_{toe}"));
        let (position, s_hs) = (&node.entries[0], &node.entries[1]);
        assert_eq!(
            (position.name.as_str(), s_hs.name.as_str()),
            ("POSITION", "S_HS")
        );
        // The tip load bends the top in tension.
        assert!(s_hs.values.iter().all(|v| *v > 0.0), "{s_hs:?}");
        assert!(position.values.iter().all(|v| *v == 50.0));

        // Deleting the hot spot breaks the output; editing keeps the choice.
        let edit = HistoryOutputDialog::edit(view, &mesh, 0).unwrap();
        assert_eq!(edit.kind, HOT_SPOT);
        assert_eq!(edit.moving_load.as_deref(), Some("Crane-1"));
        let warnings = view.set_history_sources(Vec::new(), Vec::new(), &mesh);
        assert_eq!(warnings.map(|w| w.len()), Some(1));
    }
}
