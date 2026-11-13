//! The bolt dialog: the surface of the bolt, the thread it is screwed into, the preload,
//! and what the bolt does in each step. Everything else (the shank, the cut across it, the
//! bolt faces tied to the thread) is derived and shown as "Exported as".

use egui::{RichText, Ui};
use plx_model::{Bolt, BoltAction, FeModel, Id, Preload, Quantity, Region};

use crate::model::{Highlight, Model};
use crate::numeric;
use crate::selection::Target;
use crate::setup::{FACE_SOURCES, RegionDraft};

/// The regions of a bolt while they are edited, and which of them clicks in the 3D view
/// pick.
#[derive(Clone, Debug, PartialEq)]
pub struct BoltSides {
    pub surface: RegionDraft,
    pub thread: RegionDraft,
    /// The thread is picked, not the surface of the bolt.
    pub on_thread: bool,
}

impl BoltSides {
    pub fn new(faces: Target) -> Self {
        Self {
            surface: RegionDraft::new(FACE_SOURCES, faces),
            thread: RegionDraft::new(FACE_SOURCES, faces),
            on_thread: false,
        }
    }

    pub fn of(bolt: &Bolt, faces: Target, mesh: &plx_mesh::FeMesh) -> Self {
        Self {
            surface: RegionDraft::from_region(&bolt.surface, FACE_SOURCES, faces, mesh),
            thread: RegionDraft::from_region(&bolt.thread, FACE_SOURCES, faces, mesh),
            on_thread: false,
        }
    }

    pub fn current(&self) -> &RegionDraft {
        if self.on_thread {
            &self.thread
        } else {
            &self.surface
        }
    }

    pub fn current_mut(&mut self) -> &mut RegionDraft {
        if self.on_thread {
            &mut self.thread
        } else {
            &mut self.surface
        }
    }

    /// The surface of the bolt and the thread.
    pub fn regions(&self) -> (Region, Region) {
        (self.surface.region(), self.thread.region())
    }

    /// The bolt in the primary highlight colour, the thread in the secondary one.
    pub fn highlight(&self, model: &Model) -> Highlight {
        bolt_highlight(model, &self.surface.region(), &self.thread.region())
    }

    fn ui(&mut self, ui: &mut Ui, model: &Model) {
        let on_thread = self.on_thread;
        if self
            .surface
            .ui_labeled(ui, model, "Bolt surface region", "bolt surface", !on_thread)
        {
            self.on_thread = false;
        }
        if self
            .thread
            .ui_labeled(ui, model, "Thread region", "thread", on_thread)
        {
            self.on_thread = true;
        }
    }
}

/// The bolt in the primary highlight colour, the thread in the secondary one.
pub fn bolt_highlight(model: &Model, surface: &Region, thread: &Region) -> Highlight {
    crate::contacts::master_slave_highlight(model, surface, thread)
}

/// What the bolt does in one step in the dialog: the action (`None` automatic) and the
/// amplitude of its load or adjustment.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BoltStep {
    pub action: Option<BoltAction>,
    pub amplitude: Option<Id>,
}

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

/// The bolt dialog below the name. `steps` holds what the bolt does in each step.
pub fn bolt_form(
    ui: &mut Ui,
    model: &Model,
    bolt: &mut Bolt,
    sides: &mut BoltSides,
    steps: &mut [BoltStep],
    view: &mut BoltView,
) {
    let units = model.fe.properties.units;
    sides.ui(ui, model);
    (bolt.surface, bolt.thread) = sides.regions();
    ui.label("");
    // The width of the second column before the steps' grid widens it, for the texts.
    let text_width = ui.available_width();
    ui.add(
        egui::Label::new(
            RichText::new(
                "The bolt surface: the cylindrical face of the shank, or the whole bolt with \
                 its head. The thread: the bore of the tapped hole or of the nut. Nodalis \
                 ties the bolt to the thread, also across a gap, and cuts the shank across \
                 the middle of its free part.",
            )
            .weak(),
        )
        .wrap(),
    );
    ui.end_row();
    ui.label("Preload by");
    crate::layout::wrapped_row(ui, |ui| {
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
    ui.checkbox(&mut given, "Given (otherwise the axis of the shank)");
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
        .show(ui, |ui| step_rows(ui, &model.fe, bolt, steps, units));
    ui.end_row();
    ui.label("Exported as");
    let text = view.exported(model, bolt).to_owned();
    ui.vertical(|ui| {
        ui.set_max_width(text_width);
        ui.add(egui::Label::new(RichText::new(text).weak()).wrap());
    });
    ui.end_row();
}

/// One row per step: its name, the action, the value of a load or adjustment and the
/// amplitude it follows.
fn step_rows(
    ui: &mut Ui,
    fe: &FeModel,
    bolt: &Bolt,
    steps: &mut [BoltStep],
    units: plx_model::UnitSystem,
) {
    for (s, (step, row)) in fe.steps.iter().zip(steps.iter_mut()).enumerate() {
        ui.label(&step.name);
        let Some(auto) = fe.auto_bolt_action(bolt, s) else {
            ui.weak("No preload in this step");
            ui.label("");
            ui.end_row();
            continue;
        };
        let action = &mut row.action;
        let describe = |a: BoltAction| a.name();
        let selected = match action {
            None => format!("Auto ({})", describe(auto)),
            Some(a) => describe(*a).to_owned(),
        };
        egui::ComboBox::from_id_salt(("bolt action", s))
            .selected_text(selected)
            .width(165.0)
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
        // The amplitude of a load or an adjustment, in a row of its own below; a lock or an
        // open bolt has no value to follow one.
        let ramps = matches!(
            (*action).unwrap_or(auto),
            BoltAction::Load(_) | BoltAction::Adjust(_)
        );
        if ramps {
            ui.weak("    Amplitude");
            amplitude_choice(ui, ("bolt amplitude", s), &mut row.amplitude, fe);
            ui.label("");
            ui.end_row();
        }
    }
}

/// The amplitude of a step's load or adjustment: "Default" or one of the model's amplitudes.
fn amplitude_choice(
    ui: &mut Ui,
    id: impl std::hash::Hash + std::fmt::Debug,
    amplitude: &mut Option<Id>,
    fe: &FeModel,
) {
    let text = match *amplitude {
        None => "Default".to_owned(),
        Some(id) => fe.name_or_missing(Some(id)),
    };
    egui::ComboBox::from_id_salt(id)
        .selected_text(text)
        .width(165.0)
        .show_ui(ui, |ui| {
            ui.selectable_value(amplitude, None, "Default")
                .on_hover_text("The step's ramp.");
            for a in &fe.amplitudes {
                ui.selectable_value(amplitude, Some(a.id), &a.name);
            }
        })
        .response
        .on_hover_text("The amplitude the preload or the shortening follows in the step.");
}

/// What the bolt becomes in the input file, in words: where it is cut and how it is tied to
/// the thread.
fn exported_as(model: &Model, bolt: &Bolt) -> String {
    let fe = &model.fe;
    let resolved = match fe.resolve_bolt(bolt, &model.mesh) {
        Ok(resolved) => resolved,
        Err(reason) => return format!("Not exported: {reason}."),
    };
    let mut text = resolved.why;
    match fe.resolve_thread(bolt, &model.mesh) {
        Ok(Some(thread)) => text.push_str(&format!(
            "\nThe thread is tied to {} face{} of the bolt where they are{}.",
            thread.bolt_faces.len(),
            if thread.bolt_faces.len() == 1 {
                ""
            } else {
                "s"
            },
            if thread.gap > 0.0 {
                let units = model.fe.properties.units;
                format!(
                    ", across a gap of {} {}",
                    numeric::format_physical(crate::results::round_significant(
                        units.shown(thread.gap, Quantity::Length),
                        3
                    )),
                    units.unit(Quantity::Length)
                )
            } else {
                String::new()
            }
        )),
        Ok(None) => {}
        Err(reason) => text.push_str(&format!("\nThread not exported: {reason}.")),
    }
    text
}

/// What the bolt does in each step, for the dialog.
pub fn step_actions(fe: &FeModel, bolt: Id) -> Vec<BoltStep> {
    (fe.steps.iter())
        .map(|step| {
            (step.bolt_changes.iter())
                .find(|c| c.bolt == bolt)
                .map(|c| BoltStep {
                    action: (!c.auto).then_some(c.action),
                    amplitude: c.amplitude,
                })
                .unwrap_or_default()
        })
        .collect()
}

/// Stores the steps of the dialog with the bolt: a change for each step that has its own
/// action or an amplitude.
pub fn set_step_actions(fe: &mut FeModel, bolt: Id, steps: &[BoltStep]) {
    for step in fe.steps.iter_mut() {
        step.bolt_changes.retain(|c| c.bolt != bolt);
    }
    for (s, row) in steps.iter().enumerate() {
        if row.action.is_none() && row.amplitude.is_none() {
            continue;
        }
        let Some(step) = fe.steps.get_mut(s) else {
            continue;
        };
        step.bolt_changes.push(plx_model::BoltChange {
            bolt,
            action: row.action.unwrap_or(BoltAction::Open),
            auto: row.action.is_none(),
            amplitude: row.amplitude,
        });
    }
}

/// A bolt needs its surface; without a thread the user joins it there himself.
pub fn validate_bolt(sides: &BoltSides) -> Result<(), String> {
    if sides.surface.region().is_empty() {
        return Err("Please pick the surface of the bolt.".into());
    }
    Ok(())
}
