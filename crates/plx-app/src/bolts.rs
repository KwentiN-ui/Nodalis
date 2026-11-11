//! The bolt dialog: the two faces of the cut through the shank, the thread, the preload,
//! and what the bolt does in each step. Everything else (which side is cut, the tie between
//! the halves, the bolt faces tied to the thread, the contact regions it replaces) is derived
//! and shown as "Exported as".

use egui::{RichText, Ui};
use plx_model::{Bolt, BoltAction, FeModel, Id, Preload, Quantity, Region};

use crate::contacts::{MasterSlave, Side};
use crate::model::{Highlight, Model};
use crate::numeric;
use crate::selection::Target;
use crate::setup::{FACE_SOURCES, RegionDraft, region_highlight};

/// The regions of a bolt while they are edited: the sides of the cut as master and slave,
/// and the thread, and which of them clicks in the 3D view pick.
#[derive(Clone, Debug, PartialEq)]
pub struct BoltSides {
    pub cut: MasterSlave,
    pub thread: RegionDraft,
    /// The thread is picked, not a side of the cut.
    pub on_thread: bool,
}

impl BoltSides {
    pub fn new(faces: Target) -> Self {
        Self {
            cut: MasterSlave::new(faces),
            thread: RegionDraft::new(FACE_SOURCES, faces),
            on_thread: false,
        }
    }

    pub fn of(bolt: &Bolt, faces: Target, mesh: &plx_mesh::FeMesh) -> Self {
        Self {
            cut: MasterSlave::from_regions(&bolt.side_a, &bolt.side_b, faces, mesh),
            thread: RegionDraft::from_region(&bolt.thread, FACE_SOURCES, faces, mesh),
            on_thread: false,
        }
    }

    pub fn current(&self) -> &RegionDraft {
        if self.on_thread {
            &self.thread
        } else {
            self.cut.current()
        }
    }

    pub fn current_mut(&mut self) -> &mut RegionDraft {
        if self.on_thread {
            &mut self.thread
        } else {
            self.cut.current_mut()
        }
    }

    /// Side A, side B and the thread.
    pub fn regions(&self) -> (Region, Region, Region) {
        let (a, b) = self.cut.regions();
        (a, b, self.thread.region())
    }

    /// The sides of the cut as master and slave, the thread with the slave.
    pub fn highlight(&self, model: &Model) -> Highlight {
        bolt_highlight(
            model,
            &self.cut.master.region(),
            &self.cut.slave.region(),
            &self.thread.region(),
        )
    }

    fn ui(&mut self, ui: &mut Ui, model: &Model) {
        let picking = |side| !self.on_thread && self.cut.side == side;
        let (a, b) = (picking(Side::Master), picking(Side::Slave));
        if self
            .cut
            .master
            .ui_labeled(ui, model, "Side A region", "master", a)
        {
            (self.cut.side, self.on_thread) = (Side::Master, false);
        }
        if self
            .cut
            .slave
            .ui_labeled(ui, model, "Side B region", "slave", b)
        {
            (self.cut.side, self.on_thread) = (Side::Slave, false);
        }
        let on_thread = self.on_thread;
        if self
            .thread
            .ui_labeled(ui, model, "Thread region", "thread", on_thread)
        {
            self.on_thread = true;
        }
    }
}

/// The sides of a bolt's cut in the primary and secondary highlight colours, the thread in
/// the secondary one.
pub fn bolt_highlight(model: &Model, a: &Region, b: &Region, thread: &Region) -> Highlight {
    let mut highlight = crate::contacts::master_slave_highlight(model, a, b);
    let thread = region_highlight(model, thread);
    highlight.secondary_faces.extend(thread.faces);
    highlight.secondary_lines.extend(thread.lines);
    highlight.nodes.extend(thread.nodes);
    highlight
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
    (bolt.side_a, bolt.side_b, bolt.thread) = sides.regions();
    ui.label("");
    ui.add(
        egui::Label::new(
            RichText::new(
                "The faces on the two sides of the cut through the shank, or only the \
                 cylindrical face of the shank as side A: Nodalis then cuts the bolt across \
                 its middle. The thread is optional: the faces of the internal thread of \
                 the nut or the tapped part, which Nodalis ties to the nearest faces of the \
                 bolt, also across a gap.",
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
        .show(ui, |ui| step_rows(ui, &model.fe, bolt, steps, units));
    ui.end_row();
    ui.label("Exported as");
    let text = view.exported(model, bolt).to_owned();
    ui.add(egui::Label::new(RichText::new(text).weak()).wrap());
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

/// What the bolt becomes in the input file, in words: how its sides are joined and the
/// contact regions it replaces.
fn exported_as(model: &Model, bolt: &Bolt) -> String {
    let mut fe = model.fe.clone();
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
                    numeric::format_physical(units.shown(thread.gap, Quantity::Length)),
                    units.unit(Quantity::Length)
                )
            } else {
                String::new()
            }
        )),
        Ok(None) => {}
        Err(reason) => text.push_str(&format!("\nThread not exported: {reason}.")),
    }
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

/// A bolt needs side A; side B may stay empty for a cut inside a mesh or a picked shank.
pub fn validate_bolt(sides: &BoltSides) -> Result<(), String> {
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
