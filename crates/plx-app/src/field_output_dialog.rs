//! The "Create Field Output" dialog of the Results tree: a type list, the properties
//! of the chosen type with a description below, and for limits a table of limit values.

use egui::{RichText, Ui};
use plx_mesh::FeMesh;
use plx_model::next_name;
use plx_results::Principal;
use plx_results::field_output::{FieldOutput, FieldOutputKind, LimitBasis};

use crate::numeric;
use crate::results::ResultsView;
use crate::tree_rows::{frame, tree_row};

/// The types of the list, in order, with the prefix of their default
/// names.
const TYPES: [(&str, &str); 6] = [
    ("Limit", "Limit"),
    ("Envelope", "Envelope"),
    ("Equation", "Equation"),
    ("Coordinate System Transformation", "Transform"),
    ("Principal Direction", "Direction"),
    ("Anisotropy", "Anisotropy"),
];
const LIMIT: usize = 0;
const ENVELOPE: usize = 1;
const EQUATION: usize = 2;
const TRANSFORM: usize = 3;
const DIRECTION: usize = 4;
const ANISOTROPY: usize = 5;

const NO_FIELDS: &str = "There are no field outputs or components from which a field output \
                         can be created.";
const NO_COORDINATE_SYSTEM: &str = "No coordinate system is defined into which the \
                                    field output can be transformed.";
const NO_TENSOR: &str = "The results contain no tensor field, such as STRESS or TOSTRAIN, \
                         whose principal directions could be computed.";

const LIMIT_VALUES: &str = "Limit value of each part or element set, e.g. the yield strength. \
                            It must not be 0.";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Tab {
    Properties,
    LimitValues,
}

/// Rows of the property grid; the focused one is described below the grid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Row {
    Name,
    Field,
    Component,
    Basis,
    Equation,
    Unit,
    CoordinateSystem,
    Principal,
}

impl Row {
    fn label(self) -> &'static str {
        match self {
            Row::Name => "Name",
            Row::Field => "Field",
            Row::Component => "Component",
            Row::Basis => "Limit plot based on",
            Row::Equation => "Equation",
            Row::Unit => "Unit",
            Row::CoordinateSystem => "Coordinate system",
            Row::Principal => "Principal value",
        }
    }

    fn description(self) -> &'static str {
        match self {
            Row::Name => "Name of the field output.",
            Row::Field => "Field from which the field output is computed.",
            Row::Component => "Component from which the field output is computed.",
            Row::Basis => {
                "What the limit values are given for: per part, per element set or one for \
                 all elements. If a node lies in several, the smallest limit applies. \
                 Computed are RATIO = value / limit and SAFETY_FACTOR = limit / value."
            }
            Row::Equation => {
                "Example: =DISP.ALL\nA component is given by its full name \
                 FieldName.Component, names with a hyphen in square brackets: \
                 [Limit-1.RATIO]. The names are case-sensitive. Functions such as Sqrt, Abs, \
                 Max, Min, Pow and If are available."
            }
            Row::Unit => "User-defined unit of the field output, shown in the legend.",
            Row::CoordinateSystem => {
                "Coordinate system from the features of the results into whose directions the \
                 components of a vector or tensor field are transformed; for a \
                 cylindrical one into r, theta, z at every node."
            }
            Row::Principal => {
                "Max, Mid or Min: the principal value whose direction is used. A direction \
                 has no sign, and where its principal value is repeated it is not unique."
            }
        }
    }
}

/// What the user did in the dialog this frame.
pub enum DialogAction<T> {
    Open,
    Cancel,
    /// Create or replace the output; `next` keeps the dialog open for another one (OK - New).
    Ok {
        output: T,
        next: bool,
    },
}

pub struct FieldOutputDialog {
    /// Index of the edited output; `None` creates a new one.
    pub edit: Option<usize>,
    kind: usize,
    /// One draft per type, so the values of each type are kept while switching.
    drafts: [FieldOutput; 6],
    /// Limit values per part, per element set and for all elements.
    part_limits: Vec<(String, f64)>,
    set_limits: Vec<(String, f64)>,
    all_limit: f64,
    tab: Tab,
    focus: Row,
    pub error: Option<String>,
    /// Fields of the results with their components, without the edited output.
    fields: Vec<(String, Vec<String>)>,
    /// Names a new output may not take.
    taken: Vec<String>,
    /// Coordinate systems a field can be transformed into.
    systems: Vec<String>,
    /// Tensor fields, whose principal directions can be computed.
    tensors: Vec<String>,
}

impl FieldOutputDialog {
    pub fn create(view: &ResultsView, mesh: &FeMesh) -> Self {
        Self::new(view, mesh, None)
    }

    pub fn edit(view: &ResultsView, mesh: &FeMesh, index: usize) -> Option<Self> {
        let output = view.field_outputs.get(index)?.clone();
        let mut dialog = Self::new(view, mesh, Some(index));
        dialog.kind = match &output.kind {
            FieldOutputKind::Limit {
                basis,
                limits,
                field,
                component,
            } => {
                let given = |item: &str| limits.iter().find(|(n, _)| n == item).map(|l| l.1);
                match basis {
                    LimitBasis::Parts => (dialog.part_limits.iter_mut())
                        .for_each(|(n, l)| *l = given(n).unwrap_or(*l)),
                    LimitBasis::ElementSets => (dialog.set_limits.iter_mut())
                        .for_each(|(n, l)| *l = given(n).unwrap_or(*l)),
                    LimitBasis::AllElements => {
                        dialog.all_limit = limits.first().map_or(0.0, |l| l.1);
                    }
                }
                // The edited output's own field is not offered, but its source must be.
                dialog.offer(field, component);
                LIMIT
            }
            FieldOutputKind::Envelope { field, component } => {
                dialog.offer(field, component);
                ENVELOPE
            }
            FieldOutputKind::Equation { .. } => EQUATION,
            FieldOutputKind::CoordinateSystemTransform { .. } => TRANSFORM,
            FieldOutputKind::PrincipalDirection { field, .. }
            | FieldOutputKind::Anisotropy { field, .. } => {
                if !dialog.tensors.contains(field) {
                    dialog.tensors.push(field.clone());
                }
                if matches!(output.kind, FieldOutputKind::Anisotropy { .. }) {
                    ANISOTROPY
                } else {
                    DIRECTION
                }
            }
        };
        dialog.drafts[dialog.kind] = output;
        Some(dialog)
    }

    fn new(view: &ResultsView, mesh: &FeMesh, edit: Option<usize>) -> Self {
        let own = edit
            .and_then(|i| view.field_outputs.get(i))
            .map(|o| o.name.clone());
        let tensors: Vec<String> = (view.current_increment().into_iter())
            .flat_map(|inc| &inc.fields)
            .filter(|f| Some(&f.name) != own.as_ref() && f.rank.is_tensor())
            .map(|f| f.name.clone())
            .collect();
        let fields: Vec<(String, Vec<String>)> = view
            .current_increment()
            .map(|inc| {
                (inc.fields.iter())
                    .filter(|f| Some(&f.name) != own.as_ref() && !f.components.is_empty())
                    .map(|f| {
                        let components = f.components.iter().map(|c| c.name.clone()).collect();
                        (f.name.clone(), components)
                    })
                    .collect()
            })
            .unwrap_or_default();
        let mut taken: Vec<String> = fields.iter().map(|(f, _)| f.clone()).collect();
        taken.extend(
            (view.field_outputs.iter())
                .map(|o| o.name.clone())
                .filter(|n| Some(n) != own.as_ref()),
        );
        // Nodalis proposes STRESS.MISES, else the first component there is.
        let (field, component) = fields
            .iter()
            .find(|(f, c)| f == "STRESS" && c.iter().any(|c| c == "MISES"))
            .map(|_| ("STRESS".to_string(), "MISES".to_string()))
            .or_else(|| fields.first().map(|(f, c)| (f.clone(), c[0].clone())))
            .unwrap_or_default();
        let name = |prefix: &str| next_name(prefix, taken.iter().map(String::as_str));
        let systems: Vec<String> = (view.coordinate_systems.iter())
            .map(|c| c.name.clone())
            .collect();
        // Vectors and tensors can be transformed; STRESS or DISP first.
        let transformable = ["STRESS", "DISP"]
            .into_iter()
            .find(|f| fields.iter().any(|(n, _)| n == f))
            .map(String::from)
            .unwrap_or_else(|| field.clone());
        // STRESS first for the force flow.
        let tensor = (tensors.iter())
            .find(|f| *f == "STRESS")
            .or(tensors.first())
            .cloned()
            .unwrap_or_default();
        let drafts = [
            FieldOutput {
                name: name(TYPES[LIMIT].1),
                kind: FieldOutputKind::Limit {
                    field: field.clone(),
                    component: component.clone(),
                    basis: LimitBasis::Parts,
                    limits: Vec::new(),
                },
            },
            FieldOutput {
                name: name(TYPES[ENVELOPE].1),
                kind: FieldOutputKind::Envelope {
                    field: field.clone(),
                    component: component.clone(),
                },
            },
            FieldOutput {
                name: name(TYPES[EQUATION].1),
                kind: FieldOutputKind::Equation {
                    equation: "=".into(),
                    unit: "/".into(),
                },
            },
            FieldOutput {
                name: name(TYPES[TRANSFORM].1),
                kind: FieldOutputKind::CoordinateSystemTransform {
                    field: transformable,
                    coordinate_system: systems.first().cloned().unwrap_or_default(),
                },
            },
            FieldOutput {
                name: name(TYPES[DIRECTION].1),
                kind: FieldOutputKind::PrincipalDirection {
                    field: tensor.clone(),
                    which: Principal::Max,
                },
            },
            FieldOutput {
                name: name(TYPES[ANISOTROPY].1),
                kind: FieldOutputKind::Anisotropy {
                    field: tensor,
                    which: Principal::Max,
                },
            },
        ];
        Self {
            edit,
            kind: LIMIT,
            drafts,
            part_limits: mesh.parts.iter().map(|p| (p.name.clone(), 0.0)).collect(),
            set_limits: mesh.element_sets.keys().map(|n| (n.clone(), 0.0)).collect(),
            all_limit: 0.0,
            tab: Tab::Properties,
            focus: Row::Name,
            error: None,
            fields,
            taken,
            systems,
            tensors,
        }
    }

    /// Keeps a source that is no longer among the fields selectable, e.g. after a rename.
    fn offer(&mut self, field: &str, component: &str) {
        match self.fields.iter_mut().find(|(f, _)| f == field) {
            Some((_, components)) if !components.iter().any(|c| c == component) => {
                components.push(component.to_string());
            }
            Some(_) => {}
            None => (self.fields).push((field.to_string(), vec![component.to_string()])),
        }
    }

    pub fn title(&self) -> &'static str {
        if self.edit.is_some() {
            "Edit Field Output"
        } else {
            "Create Field Output"
        }
    }

    /// Why the chosen type cannot be created, shown instead of its properties.
    fn unavailable(&self) -> Option<&'static str> {
        if self.fields.is_empty() {
            Some(NO_FIELDS)
        } else if self.kind == TRANSFORM && self.systems.is_empty() {
            Some(NO_COORDINATE_SYSTEM)
        } else if matches!(self.kind, DIRECTION | ANISOTROPY) && self.tensors.is_empty() {
            Some(NO_TENSOR)
        } else {
            None
        }
    }

    /// The output as the dialog defines it, or why it cannot be created.
    pub fn output(&self) -> Result<FieldOutput, String> {
        if let Some(reason) = self.unavailable() {
            return Err(reason.into());
        }
        let mut output = self.drafts[self.kind].clone();
        output.name = output.name.trim().to_string();
        if output.name.is_empty() {
            return Err("Please enter a name.".into());
        }
        if self
            .taken
            .iter()
            .any(|t| t.eq_ignore_ascii_case(&output.name))
        {
            return Err(format!("The name {} is already used.", output.name));
        }
        if output.name.contains('.') {
            return Err("The name must not contain a dot.".into());
        }
        if let FieldOutputKind::Limit { basis, limits, .. } = &mut output.kind {
            *limits = match basis {
                LimitBasis::Parts => self.part_limits.clone(),
                LimitBasis::ElementSets => self.set_limits.clone(),
                LimitBasis::AllElements => {
                    vec![(LimitBasis::AllElements.name().to_string(), self.all_limit)]
                }
            };
            if limits.is_empty() {
                return Err("There are no element sets the limit values could apply to.".into());
            }
            if let Some((item, _)) = limits.iter().find(|(_, l)| *l == 0.0) {
                return Err(format!(
                    "Please enter a limit value other than 0 for {item} under Limit Values."
                ));
            }
        }
        Ok(output)
    }

    /// Prepares the dialog for the next output after OK - New: the created name is taken
    /// and every type proposes its next free name.
    pub fn next(&mut self, created: &str) {
        self.taken.push(created.to_string());
        for (draft, (_, prefix)) in self.drafts.iter_mut().zip(TYPES) {
            draft.name = next_name(prefix, self.taken.iter().map(String::as_str));
        }
        self.error = None;
    }

    pub fn show(&mut self, ctx: &egui::Context) -> DialogAction<FieldOutput> {
        let mut action = DialogAction::Open;
        let mut open = true;
        let title = self.title();
        let builder = egui::Window::new(title)
            .id(crate::ids::window(ctx, "field output dialog"))
            .collapsible(false)
            .resizable(false)
            // Anchored at the top, so that a longer description or an error never moves it.
            .default_pos(ctx.content_rect().center() - egui::vec2(175.0, 290.0));
        crate::dock::show(ctx, builder, title, &mut open, |ui| {
            ui.set_width(340.0);
            ui.label("Type");
            frame().show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.set_min_height(76.0);
                for (index, (label, _)) in TYPES.iter().enumerate() {
                    let selected = self.kind == index;
                    let row = ui.add_enabled_ui(self.edit.is_none() || selected, |ui| {
                        tree_row(ui, 0, None, selected, RichText::new(*label))
                    });
                    if row.inner.clicked() && !selected {
                        self.kind = index;
                        self.tab = Tab::Properties;
                        self.focus = Row::Name;
                        self.error = None;
                    }
                }
            });
            ui.add_space(6.0);
            self.tabs(ui);
            frame().show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.set_min_height(220.0);
                match self.tab {
                    Tab::Properties => self.properties(ui),
                    Tab::LimitValues => self.limit_values(ui),
                }
            });
            ui.add_space(4.0);
            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.set_height(76.0);
                egui::ScrollArea::vertical().show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    self.description(ui);
                });
            });
            if let Some(error) = &self.error {
                ui.colored_label(crate::style::error(), error);
            }
            ui.add_space(4.0);
            crate::dock::footer(ui, |ui| {
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
        if !open {
            action = DialogAction::Cancel;
        }
        action
    }

    /// The help box below the property grid: what the focused row means.
    fn description(&self, ui: &mut Ui) {
        if self.unavailable().is_some() {
            return;
        }
        if self.tab == Tab::LimitValues {
            ui.strong("Limit Values");
            ui.label(LIMIT_VALUES);
        } else {
            ui.strong(self.focus.label());
            ui.label(self.focus.description());
        }
    }

    fn tabs(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 0.0;
            let mut tabs = vec![(Tab::Properties, "Properties")];
            if self.kind == LIMIT && self.unavailable().is_none() {
                tabs.push((Tab::LimitValues, "Limit Values"));
            }
            for (tab, label) in tabs {
                // Framed also when inactive, so hovering a tab does not widen it and push the
                // tabs to its right.
                let button =
                    egui::Button::selectable(self.tab == tab, label).frame_when_inactive(true);
                if ui.add(button).clicked() {
                    self.tab = tab;
                }
            }
        });
    }

    fn properties(&mut self, ui: &mut Ui) {
        if let Some(reason) = self.unavailable() {
            ui.label(reason);
            return;
        }
        // Category header of the property grid.
        let header = egui::Frame::new()
            .fill(crate::style::control())
            .inner_margin(egui::Margin::symmetric(4, 1));
        header.show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.strong("Data");
        });
        let fields = &self.fields;
        let systems = &self.systems;
        let tensors = &self.tensors;
        let focus = &mut self.focus;
        let draft = &mut self.drafts[self.kind];
        egui::Grid::new("field output properties")
            .max_col_width(crate::dock::column_limit(ui))
            .num_columns(2)
            .striped(true)
            .min_col_width(130.0)
            .spacing([8.0, 4.0])
            .show(ui, |ui| {
                prop_row(ui, focus, Row::Name, |ui| {
                    let edit = egui::TextEdit::singleline(&mut draft.name).desired_width(180.0);
                    ui.add(edit).has_focus()
                });
                match &mut draft.kind {
                    FieldOutputKind::Limit {
                        field,
                        component,
                        basis,
                        ..
                    } => {
                        source_rows(ui, focus, fields, field, component);
                        prop_row(ui, focus, Row::Basis, |ui| {
                            let response = egui::ComboBox::from_id_salt("limit basis")
                                .selected_text(basis_label(*basis))
                                .width(crate::dock::fit_width(ui, 180.0))
                                .show_ui(ui, |ui| {
                                    for choice in LimitBasis::ALL {
                                        ui.selectable_value(basis, choice, basis_label(choice));
                                    }
                                });
                            response.response.clicked()
                        });
                    }
                    FieldOutputKind::Envelope { field, component } => {
                        source_rows(ui, focus, fields, field, component);
                    }
                    FieldOutputKind::Equation { equation, unit } => {
                        prop_row(ui, focus, Row::Equation, |ui| {
                            let edit = egui::TextEdit::singleline(equation).desired_width(180.0);
                            ui.add(edit).has_focus()
                        });
                        prop_row(ui, focus, Row::Unit, |ui| {
                            let edit = egui::TextEdit::singleline(unit).desired_width(180.0);
                            ui.add(edit).has_focus()
                        });
                    }
                    FieldOutputKind::CoordinateSystemTransform {
                        field,
                        coordinate_system,
                    } => {
                        prop_row(ui, focus, Row::Field, |ui| {
                            let response = egui::ComboBox::from_id_salt("transform field")
                                .selected_text(field.as_str())
                                .width(crate::dock::fit_width(ui, 180.0))
                                .show_ui(ui, |ui| {
                                    for (name, _) in fields {
                                        ui.selectable_value(field, name.clone(), name);
                                    }
                                });
                            response.response.clicked()
                        });
                        prop_row(ui, focus, Row::CoordinateSystem, |ui| {
                            let response = egui::ComboBox::from_id_salt("transform system")
                                .selected_text(coordinate_system.as_str())
                                .width(crate::dock::fit_width(ui, 180.0))
                                .show_ui(ui, |ui| {
                                    for name in systems {
                                        ui.selectable_value(coordinate_system, name.clone(), name);
                                    }
                                });
                            response.response.clicked()
                        });
                    }
                    FieldOutputKind::PrincipalDirection { field, which }
                    | FieldOutputKind::Anisotropy { field, which } => {
                        prop_row(ui, focus, Row::Field, |ui| {
                            let response = egui::ComboBox::from_id_salt("principal field")
                                .selected_text(field.as_str())
                                .width(crate::dock::fit_width(ui, 180.0))
                                .show_ui(ui, |ui| {
                                    for name in tensors {
                                        ui.selectable_value(field, name.clone(), name);
                                    }
                                });
                            response.response.clicked()
                        });
                        prop_row(ui, focus, Row::Principal, |ui| {
                            let response = egui::ComboBox::from_id_salt("principal value")
                                .selected_text(which.label())
                                .width(crate::dock::fit_width(ui, 180.0))
                                .show_ui(ui, |ui| {
                                    for choice in Principal::ALL {
                                        ui.selectable_value(which, choice, choice.label());
                                    }
                                });
                            response.response.clicked()
                        });
                    }
                }
            });
        if let FieldOutputKind::Equation { equation, .. } = &mut draft.kind {
            ui.add_space(4.0);
            ui.menu_button("Insert Component…", |ui| {
                insert_menu(ui, fields, equation);
            });
        }
    }

    fn limit_values(&mut self, ui: &mut Ui) {
        let FieldOutputKind::Limit { basis, .. } = &self.drafts[LIMIT].kind else {
            return;
        };
        let (header, limits): (&str, Vec<(String, &mut f64)>) = match basis {
            LimitBasis::Parts => (
                "Part",
                (self.part_limits.iter_mut())
                    .map(|(n, l)| (n.clone(), l))
                    .collect(),
            ),
            LimitBasis::ElementSets => (
                "Element set",
                (self.set_limits.iter_mut())
                    .map(|(n, l)| (n.clone(), l))
                    .collect(),
            ),
            LimitBasis::AllElements => (
                "Elements",
                vec![(
                    basis_label(LimitBasis::AllElements).into(),
                    &mut self.all_limit,
                )],
            ),
        };
        if limits.is_empty() {
            ui.weak("The results contain no element sets.");
            return;
        }
        egui::ScrollArea::vertical()
            .max_height(210.0)
            .show(ui, |ui| {
                egui::Grid::new("limit values")
                    .num_columns(2)
                    .striped(true)
                    .min_col_width(150.0)
                    .show(ui, |ui| {
                        ui.strong(header);
                        ui.strong("Limit value");
                        ui.end_row();
                        for (name, limit) in limits {
                            ui.label(name);
                            let height = ui.spacing().interact_size.y;
                            ui.add_sized([120.0, height], numeric::drag_value(limit).speed(1.0));
                            ui.end_row();
                        }
                    });
            });
    }
}

fn basis_label(basis: LimitBasis) -> &'static str {
    match basis {
        LimitBasis::Parts => "Parts",
        LimitBasis::ElementSets => "Element sets",
        LimitBasis::AllElements => "All elements",
    }
}

/// One row of the property grid; clicking its label or using its editor focuses it.
fn prop_row(ui: &mut Ui, focus: &mut Row, row: Row, editor: impl FnOnce(&mut Ui) -> bool) {
    let label = ui.add(egui::Label::new(row.label()).sense(egui::Sense::click()));
    if editor(ui) || label.clicked() {
        *focus = row;
    }
    ui.end_row();
}

/// Field and component rows; changing the field keeps the component if the field has it.
fn source_rows(
    ui: &mut Ui,
    focus: &mut Row,
    fields: &[(String, Vec<String>)],
    field: &mut String,
    component: &mut String,
) {
    prop_row(ui, focus, Row::Field, |ui| {
        let mut changed = false;
        let response = egui::ComboBox::from_id_salt("field output field")
            .selected_text(field.as_str())
            .width(crate::dock::fit_width(ui, 180.0))
            .show_ui(ui, |ui| {
                for (name, _) in fields {
                    changed |= ui.selectable_value(field, name.clone(), name).changed();
                }
            });
        if changed
            && let Some((_, components)) = fields.iter().find(|(f, _)| f == field)
            && !components.contains(component)
        {
            *component = components[0].clone();
        }
        response.response.clicked()
    });
    prop_row(ui, focus, Row::Component, |ui| {
        let components = fields
            .iter()
            .find(|(f, _)| f == field)
            .map_or(&[][..], |(_, c)| c.as_slice());
        let response = egui::ComboBox::from_id_salt("field output component")
            .selected_text(component.as_str())
            .width(crate::dock::fit_width(ui, 180.0))
            .show_ui(ui, |ui| {
                for name in components {
                    ui.selectable_value(component, name.clone(), name);
                }
            });
        response.response.clicked()
    });
}

/// Menu entries that append `FIELD.COMPONENT` to the equation.
fn insert_menu(ui: &mut Ui, fields: &[(String, Vec<String>)], equation: &mut String) {
    for (field, components) in fields {
        ui.menu_button(field, |ui| {
            for component in components {
                if ui.button(component).clicked() {
                    let name = if field.contains(|c: char| !c.is_alphanumeric() && c != '_') {
                        format!("[{field}.{component}]")
                    } else {
                        format!("{field}.{component}")
                    };
                    equation.push_str(&name);
                    ui.close();
                }
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn results() -> (ResultsView, FeMesh) {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/kragbalken_c3d8.frd");
        let import = plx_io::frd::read_frd(&path).unwrap();
        let view = ResultsView::new(import.increments, import.mesh.bounds());
        (view, import.mesh)
    }

    #[test]
    fn limit_proposes_stress_and_needs_limit_values() {
        let (mut view, mesh) = results();
        let mut dialog = FieldOutputDialog::create(&view, &mesh);
        assert_eq!(dialog.drafts[LIMIT].name, "Limit-1");
        let FieldOutputKind::Limit {
            field, component, ..
        } = &dialog.drafts[LIMIT].kind
        else {
            panic!()
        };
        assert_eq!((field.as_str(), component.as_str()), ("STRESS", "MISES"));
        assert!(dialog.output().is_err(), "limits are still 0");
        for (_, limit) in &mut dialog.part_limits {
            *limit = 235.0;
        }
        let output = dialog.output().unwrap();
        view.set_field_output(None, output, &mesh).unwrap();
        let field = view.current_increment().unwrap().field("Limit-1").unwrap();
        assert_eq!(field.components[0].name, "RATIO");

        // OK - New proposes the next free name; editing keeps the type and values.
        dialog.next("Limit-1");
        assert_eq!(dialog.drafts[LIMIT].name, "Limit-2");
        let index = view.field_output_index(view.current_increment().unwrap().fields.len() - 1);
        let edit = FieldOutputDialog::edit(&view, &mesh, index.unwrap()).unwrap();
        assert_eq!(edit.kind, LIMIT);
        assert_eq!(edit.part_limits[0].1, 235.0);
        assert!(edit.output().is_ok(), "its own name is free");
        assert!(edit.fields.iter().all(|(f, _)| f != "Limit-1"));
    }

    #[test]
    fn equation_output_keeps_the_shown_component() {
        let (mut view, mesh) = results();
        let shown = view
            .current()
            .map(|(f, c)| (f.name.clone(), c.name.clone()));
        let mut dialog = FieldOutputDialog::create(&view, &mesh);
        dialog.kind = EQUATION;
        assert!(dialog.output().is_ok(), "the dialog checks names only");
        assert!(
            view.set_field_output(None, dialog.output().unwrap(), &mesh)
                .is_err()
        );
        dialog.drafts[EQUATION].kind = FieldOutputKind::Equation {
            equation: "=DISP.ALL * 1000".into(),
            unit: "um".into(),
        };
        view.set_field_output(None, dialog.output().unwrap(), &mesh)
            .unwrap();
        let now = view
            .current()
            .map(|(f, c)| (f.name.clone(), c.name.clone()));
        assert_eq!(now, shown);
        let fields = &view.current_increment().unwrap().fields;
        let index = fields.iter().position(|f| f.name == "Equation-1").unwrap();
        (view.field, view.component) = (index, 0);
        assert!(view.legend().unwrap().title.contains("Unit: um"));
        view.remove_field_output(0, &mesh);
        assert!(
            view.current_increment()
                .unwrap()
                .field("Equation-1")
                .is_none()
        );
        assert_eq!(view.field, 0);
    }

    #[test]
    fn principal_direction_offers_the_tensor_fields() {
        let (mut view, mesh) = results();
        let mut dialog = FieldOutputDialog::create(&view, &mesh);
        assert_eq!(dialog.tensors, ["STRESS"]);
        dialog.kind = DIRECTION;
        let output = dialog.output().unwrap();
        assert_eq!(output.name, "Direction-1");
        view.set_field_output(None, output, &mesh).unwrap();
        let field = view.current_increment().unwrap().field("Direction-1");
        assert_eq!(field.unwrap().rank, plx_results::FieldRank::Vector);
        // The principal value is in the unit of the stress.
        let fields = &view.current_increment().unwrap().fields;
        let index = fields.iter().position(|f| f.name == "Direction-1").unwrap();
        (view.field, view.component) = (index, 4);
        assert!(view.legend().unwrap().title.contains("VALUE\nUnit: MPa"));
        // The new vector field is no tensor; editing keeps the type.
        let dialog = FieldOutputDialog::create(&view, &mesh);
        assert!(!dialog.tensors.iter().any(|t| t == "Direction-1"));
        let edit = FieldOutputDialog::edit(&view, &mesh, 0).unwrap();
        assert_eq!(edit.kind, DIRECTION);
        let mut dialog = FieldOutputDialog::create(&view, &mesh);
        dialog.kind = ANISOTROPY;
        dialog.tensors.clear();
        assert_eq!(dialog.output().unwrap_err(), NO_TENSOR);
    }

    #[test]
    fn transform_needs_a_coordinate_system() {
        let (view, mesh) = results();
        let mut dialog = FieldOutputDialog::create(&view, &mesh);
        dialog.kind = TRANSFORM;
        assert_eq!(dialog.output().unwrap_err(), NO_COORDINATE_SYSTEM);
    }

    #[test]
    fn transform_offers_the_coordinate_systems_of_the_results() {
        let (mut view, mesh) = results();
        view.coordinate_systems = vec![plx_model::CoordinateSystem::new("CS-1")];
        let mut dialog = FieldOutputDialog::create(&view, &mesh);
        dialog.kind = TRANSFORM;
        let output = dialog.output().unwrap();
        assert!(matches!(
            output.kind,
            FieldOutputKind::CoordinateSystemTransform { ref coordinate_system, .. }
                if coordinate_system == "CS-1"
        ));
    }
}
