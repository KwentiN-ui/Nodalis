//! The bolt dialog: the two faces of the cut through the shank, the preload, and what the
//! bolt does in each step. Everything else (which side is cut, the tie between the halves,
//! the contact regions it replaces) is derived and shown as "Exported as".

use egui::{RichText, Ui};
use plx_model::{Bolt, BoltAction, FeModel, Preload, Quantity};

use crate::contacts::MasterSlave;
use crate::model::Model;
use crate::numeric;

/// What the dialog keeps besides the bolt: the derivation of the bolt as last edited,
/// computed again only when it changes.
#[derive(Clone, Debug, Default)]
pub struct BoltView {
    exported: Option<Box<(Bolt, String)>>,
}

impl BoltView {
    fn exported(&mut self, model: &Model, bolt: &Bolt) -> &str {
        if self.exported.as_ref().is_none_or(|e| e.0 != *bolt) {
            self.exported = Some(Box::new((bolt.clone(), exported_as(model, bolt))));
        }
        &self.exported.as_ref().unwrap().1
    }
}

/// The bolt dialog below the name. `actions` holds what the bolt does in each step, `None`
/// for the automatic action.
pub fn bolt_form(
    ui: &mut Ui,
    model: &Model,
    bolt: &mut Bolt,
    sides: &mut MasterSlave,
    actions: &mut [Option<BoltAction>],
    view: &mut BoltView,
) {
    let units = model.fe.properties.units;
    sides.ui_named(ui, model, ["Side A", "Side B"]);
    (bolt.side_a, bolt.side_b) = sides.regions();
    ui.label("");
    ui.add(
        egui::Label::new(
            RichText::new(
                "The faces on the two sides of the cut through the shank, or only the \
                 cylindrical face of the shank as side A: Nodalis then cuts the bolt across \
                 its middle.",
            )
            .weak(),
        )
        .wrap(),
    );
    ui.end_row();
    ui.label("Preload by");
    ui.horizontal_wrapped(|ui| {
        let value = bolt.preload.value();
        if ui
            .radio(matches!(bolt.preload, Preload::Force(_)), "Force")
            .clicked()
        {
            bolt.preload = Preload::Force(value);
        }
        if ui
            .radio(matches!(bolt.preload, Preload::Shortening(_)), "Shortening")
            .clicked()
        {
            bolt.preload = Preload::Shortening(value);
        }
    });
    ui.end_row();
    match &mut bolt.preload {
        Preload::Force(value) => {
            ui.label("Preload");
            ui.add(numeric::quantity(value, units, Quantity::Force).speed(1.0));
        }
        Preload::Shortening(value) => {
            ui.label("Shortening");
            ui.add(numeric::quantity(value, units, Quantity::Length).speed(0.001));
        }
    }
    ui.end_row();
    let mut given = bolt.axis.is_some();
    ui.label("Axis");
    ui.checkbox(&mut given, "Given (otherwise the normal of the cut)");
    ui.end_row();
    let mut values = bolt.axis.unwrap_or([0.0, 0.0, 1.0]);
    if given {
        for (value, label) in values.iter_mut().zip(["    X", "    Y", "    Z"]) {
            ui.label(label);
            ui.add(numeric::drag_value(value).speed(0.01));
            ui.end_row();
        }
    }
    bolt.axis = given.then_some(values);
    ui.label("Color");
    ui.color_edit_button_srgb(&mut bolt.color);
    ui.end_row();
    ui.label("Steps").on_hover_text(
        "Auto preloads the bolt in the first step that takes forces and locks it at its \
         length in the steps after it.",
    );
    egui::Grid::new("bolt steps")
        .num_columns(3)
        .spacing([8.0, 4.0])
        .show(ui, |ui| step_rows(ui, &model.fe, bolt, actions, units));
    ui.end_row();
    ui.label("Exported as");
    let text = view.exported(model, bolt).to_owned();
    ui.add(egui::Label::new(RichText::new(text).weak()).wrap());
    ui.end_row();
}

/// One row per step: its name, the action and the value of a load or adjustment.
fn step_rows(
    ui: &mut Ui,
    fe: &FeModel,
    bolt: &Bolt,
    actions: &mut [Option<BoltAction>],
    units: plx_model::UnitSystem,
) {
    for (s, (step, action)) in fe.steps.iter().zip(actions.iter_mut()).enumerate() {
        ui.label(&step.name);
        let Some(auto) = fe.auto_bolt_action(bolt, s) else {
            ui.weak("No preload in this step");
            ui.label("");
            ui.end_row();
            continue;
        };
        let describe = |a: BoltAction| a.name();
        let selected = match action {
            None => format!("Auto ({})", describe(auto)),
            Some(a) => describe(*a).to_owned(),
        };
        egui::ComboBox::from_id_salt(("bolt action", s))
            .selected_text(selected)
            .width(140.0)
            .show_ui(ui, |ui| {
                if ui
                    .selectable_label(action.is_none(), format!("Auto ({})", describe(auto)))
                    .clicked()
                {
                    *action = None;
                }
                let value = match (*action).unwrap_or(auto) {
                    BoltAction::Load(v) | BoltAction::Adjust(v) => v,
                    _ => bolt.preload.value(),
                };
                let choices = [
                    (BoltAction::Load(value), "Pulled by a force."),
                    (
                        BoltAction::Adjust(value),
                        "Shortened by a length since the start of the analysis.",
                    ),
                    (
                        BoltAction::Lock,
                        "Kept at its length at the end of the previous step.",
                    ),
                    (BoltAction::Open, "Loose: no preload."),
                ];
                for (choice, hint) in choices {
                    let current = action.is_some_and(|a| a.name() == choice.name());
                    if ui
                        .selectable_label(current, choice.name())
                        .on_hover_text(hint)
                        .clicked()
                    {
                        *action = Some(choice);
                    }
                }
            });
        match action {
            Some(BoltAction::Load(value)) => {
                ui.add(numeric::quantity(value, units, Quantity::Force).speed(1.0));
            }
            Some(BoltAction::Adjust(value)) => {
                ui.add(numeric::quantity(value, units, Quantity::Length).speed(0.001));
            }
            _ => {
                ui.label("");
            }
        }
        ui.end_row();
    }
}

/// What the bolt becomes in the input file, in words: how its sides are joined and the
/// contact regions it replaces.
fn exported_as(model: &Model, bolt: &Bolt) -> String {
    let mut fe = model.fe.clone();
    let resolved = match fe.resolve_bolt(bolt, &model.mesh) {
        Ok(resolved) => resolved,
        Err(reason) => return format!("Not exported: {reason}."),
    };
    let mut text = resolved.why;
    // The regions this bolt replaces, as if it were in the model already.
    fe.bolts = vec![Bolt {
        active: true,
        ..bolt.clone()
    }];
    let replaced: Vec<&str> = (fe.replaced_contact_regions(&model.mesh).keys())
        .map(|&r| model.fe.contact_regions[r].name.as_str())
        .collect();
    if !replaced.is_empty() {
        text.push_str(&format!(
            "\nReplaces the contact region{} {}.",
            if replaced.len() == 1 { "" } else { "s" },
            replaced.join(", ")
        ));
    }
    text
}

/// The step actions of the bolt named `name`, one per step, for the dialog.
pub fn step_actions(fe: &FeModel, name: &str) -> Vec<Option<BoltAction>> {
    (fe.steps.iter())
        .map(|step| {
            (step.bolt_changes.iter())
                .find(|c| c.bolt == name)
                .map(|c| c.action)
        })
        .collect()
}

/// Stores the step actions of the dialog with the bolt named `name`.
pub fn set_step_actions(fe: &mut FeModel, name: &str, actions: &[Option<BoltAction>]) {
    for (step, action) in fe.steps.iter_mut().zip(actions) {
        step.bolt_changes.retain(|c| c.bolt != name);
        if let Some(action) = action {
            step.bolt_changes.push(plx_model::BoltChange {
                bolt: name.to_owned(),
                action: *action,
            });
        }
    }
}

/// A bolt needs side A; side B may stay empty for a cut inside a mesh or a picked shank.
pub fn validate_bolt(sides: &MasterSlave) -> Result<(), String> {
    let empty = match sides.regions().0 {
        plx_model::Region::Faces(faces) => faces.is_empty(),
        plx_model::Region::Geometry(entities) => entities.is_empty(),
        plx_model::Region::Surface(name) => name.is_empty(),
        _ => false,
    };
    if empty {
        return Err("Please pick the faces of side A of the cut.".into());
    }
    Ok(())
}
