//! The step overview window: every boundary condition, load, defined field and model-wide
//! item against the steps, Abaqus' load manager ([`plx_model::StepOverview`]). A click on a
//! cell opens the item in that step; its context menu copies the item to the following
//! steps or removes it from them, the explicit "propagate" of other tools.

use egui::{Color32, RichText};
use plx_model::{Cell, FeModel, StepItemKey};

use crate::tree::TreeItem;

/// What the user asked for in the overview.
pub enum Action {
    /// Open the item in its step.
    Open(TreeItem),
    /// Copy the item of the step to every following step that can take it.
    Copy(usize, StepItemKey),
    /// Remove the item from every step after the step.
    Remove(usize, StepItemKey),
}

/// Shows the window while `open`; returns what the user asked for.
pub fn window(ctx: &egui::Context, open: &mut bool, fe: &FeModel) -> Option<Action> {
    let overview = fe.step_overview();
    let mut action = None;
    let mut close = false;
    egui::Window::new("Step Overview")
        .id(crate::ids::window(ctx, "step overview"))
        .open(open)
        .collapsible(false)
        .default_width(720.0)
        .default_height(360.0)
        .resizable(true)
        .show(ctx, |ui| {
            ui.label(
                "What each step has compared with the step it starts from. Click a cell to \
                 open the item in that step; its menu copies it to the following steps or \
                 removes it from them.",
            );
            ui.separator();
            close = crate::dialog::close_bar(ui, "step overview");
            if overview.columns.is_empty() {
                ui.weak("No steps. Create one with Step on the Steps tab.");
                return;
            }
            egui::ScrollArea::both()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    egui::Grid::new("step overview grid")
                        .striped(true)
                        .num_columns(overview.columns.len() + 1)
                        .spacing([18.0, 4.0])
                        .show(ui, |ui| {
                            ui.strong("Item");
                            for (s, column) in overview.columns.iter().enumerate() {
                                let mut text = RichText::new(&column.name).strong();
                                if !column.active {
                                    text = text.weak().strikethrough();
                                }
                                let response = ui
                                    .add(
                                        egui::Label::new(text)
                                            .selectable(false)
                                            .sense(egui::Sense::click()),
                                    )
                                    .on_hover_text(format!(
                                        "{}{}\nClick: open the step.",
                                        column.start,
                                        if column.probing {
                                            "\nLeaves the state as it found it."
                                        } else {
                                            ""
                                        }
                                    ));
                                if response.clicked() {
                                    action = Some(Action::Open(TreeItem::Step(s)));
                                }
                            }
                            ui.end_row();
                            ui.label(RichText::new("Starts from").italics());
                            for column in &overview.columns {
                                let start = (column.start.split_once(": "))
                                    .map_or(column.start.as_str(), |(_, s)| s);
                                let start = if column.probing {
                                    format!("Base: {start}")
                                } else {
                                    start.to_owned()
                                };
                                ui.label(RichText::new(start).italics().weak());
                            }
                            ui.end_row();
                            let mut group = None;
                            for row in &overview.rows {
                                if group != Some(row.group) {
                                    group = Some(row.group);
                                    ui.label(RichText::new(row.group.label()).strong());
                                    ui.end_row();
                                }
                                ui.label(format!("  {}", row.name));
                                for (s, cell) in row.cells.iter().enumerate() {
                                    let text = RichText::new(cell.text()).color(color(ui, cell));
                                    let Some(key) = &row.key else {
                                        ui.label(text);
                                        continue;
                                    };
                                    let response = ui.add(
                                        egui::Label::new(text)
                                            .selectable(false)
                                            .sense(egui::Sense::click()),
                                    );
                                    let response = if cell.defined() {
                                        response.on_hover_text(
                                            "Click: open it in this step. Right-click: copy to \
                                             or remove from the following steps.",
                                        )
                                    } else {
                                        response
                                    };
                                    if response.clicked()
                                        && cell.defined()
                                        && let Some(item) = tree_item(fe, s, key)
                                    {
                                        action = Some(Action::Open(item));
                                    }
                                    let later = s + 1 < overview.columns.len();
                                    if cell.defined() && later {
                                        response.context_menu(|ui| {
                                            ui.style_mut().wrap_mode =
                                                Some(egui::TextWrapMode::Extend);
                                            if ui.button("Copy to Following Steps").clicked() {
                                                action = Some(Action::Copy(s, key.clone()));
                                            }
                                            if ui.button("Remove from Following Steps").clicked() {
                                                action = Some(Action::Remove(s, key.clone()));
                                            }
                                        });
                                    }
                                }
                                ui.end_row();
                            }
                            if overview.rows.is_empty() {
                                ui.weak("No boundary conditions, loads or model-wide items.");
                                ui.end_row();
                            }
                        });
                });
        });
    if close {
        *open = false;
    }
    action
}

/// The colour of a cell: new and changed stand out, what stays is quiet.
fn color(ui: &egui::Ui, cell: &Cell) -> Color32 {
    let weak = ui.visuals().weak_text_color();
    let text = ui.visuals().text_color();
    match cell {
        Cell::Created => crate::style::ok(),
        Cell::Changed | Cell::Action(_) => crate::style::warning(),
        Cell::Removed | Cell::RampsToZero => crate::style::error(),
        Cell::Same | Cell::Off | Cell::ModelWideOff | Cell::Absent => weak,
        Cell::ModelWide => text,
    }
}

/// The tree item of a step item in step `s`.
fn tree_item(fe: &FeModel, s: usize, key: &StepItemKey) -> Option<TreeItem> {
    let step = fe.steps.get(s)?;
    Some(match key {
        StepItemKey::BoundaryCondition(id) => TreeItem::BoundaryCondition(
            s,
            step.boundary_conditions.iter().position(|b| b.id == *id)?,
        ),
        StepItemKey::Load(id) => TreeItem::Load(s, step.loads.iter().position(|l| l.id == *id)?),
        StepItemKey::DefinedField(name) => {
            TreeItem::DefinedField(s, step.defined_fields.iter().position(|f| f.name == *name)?)
        }
    })
}

/// The words of the toast after copying or removing.
pub fn done(fe: &FeModel, action: &Action, count: usize) -> String {
    let (s, key, verb) = match action {
        Action::Copy(s, key) => (*s, key, "Copied"),
        Action::Remove(s, key) => (*s, key, "Removed"),
        Action::Open(_) => return String::new(),
    };
    let name = tree_item(fe, s, key)
        .and_then(|item| match item {
            TreeItem::BoundaryCondition(s, i) => {
                Some(fe.steps[s].boundary_conditions[i].name.clone())
            }
            TreeItem::Load(s, i) => Some(fe.steps[s].loads[i].name.clone()),
            TreeItem::DefinedField(s, i) => Some(fe.steps[s].defined_fields[i].name.clone()),
            _ => None,
        })
        .unwrap_or_default();
    let steps = if count == 1 { "step" } else { "steps" };
    match (verb, count) {
        (_, 0) if verb == "Copied" => format!("No following step can take {name}"),
        (_, 0) => format!("No following step has {name}"),
        ("Copied", _) => format!("Copied {name} to {count} following {steps}"),
        _ => format!("Removed {name} from {count} following {steps}"),
    }
}
