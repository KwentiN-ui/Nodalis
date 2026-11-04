//! The checklist the properties panel shows while no item is edited: the steps of the
//! workspace, ticked off when done, each with the button that does it. The same as the
//! problems of the model check, worded positively.

use egui::{RichText, Ui};

use crate::command::{self, Cmd};
use crate::model::Model;

/// One step of the checklist.
pub struct Step {
    pub text: &'static str,
    pub done: bool,
    /// The command that does it, offered as a button while it is not done.
    pub cmd: Option<Cmd>,
}

/// What the checklist asks for.
pub enum Clicked {
    Run(Cmd),
    /// Show the Problems tab.
    Problems,
}

/// The steps of the Geometry workspace.
pub fn geometry(geometry: Option<&Model>, model: Option<&Model>) -> Vec<Step> {
    let has_geometry = geometry.is_some_and(|g| !g.parts.is_empty());
    let has_mesh = model.is_some_and(|m| m.mesh.element_count() > 0);
    vec![
        Step {
            text: "Import a STEP, IGES or BREP file.",
            done: has_geometry,
            cmd: Some(Cmd::Import),
        },
        Step {
            text: "Mesh the parts.",
            done: has_mesh,
            cmd: has_geometry.then_some(Cmd::MeshAll),
        },
    ]
}

/// The steps of the FE Model workspace: what a static analysis needs.
pub fn fe_model(model: Option<&Model>) -> Vec<Step> {
    let fe = model.map(|m| &m.fe);
    let steps = fe.map_or(&[][..], |fe| &fe.steps[..]);
    let has_step = !steps.is_empty();
    let has = |test: fn(&plx_model::FeModel) -> bool| fe.is_some_and(test);
    vec![
        Step {
            text: "Define a material.",
            done: has(|fe| !fe.materials.is_empty()),
            cmd: Some(Cmd::Material),
        },
        Step {
            text: "Assign it to the parts with a section.",
            done: has(|fe| !fe.sections.is_empty()),
            cmd: Some(Cmd::Section),
        },
        Step {
            text: "Create a step.",
            done: has_step,
            cmd: Some(Cmd::Step),
        },
        Step {
            text: "Support the model.",
            done: steps.iter().any(|s| !s.boundary_conditions.is_empty())
                || has(|fe| !fe.constraints.is_empty()),
            cmd: has_step.then_some(Cmd::BoundaryCondition),
        },
        Step {
            text: "Load it.",
            done: steps.iter().any(|s| !s.loads.is_empty()),
            cmd: has_step.then_some(Cmd::Load),
        },
    ]
}

/// The steps of the Results workspace.
pub fn results(solved: bool) -> Vec<Step> {
    vec![
        Step {
            text: "Solve the analysis.",
            done: solved,
            cmd: Some(Cmd::Solve),
        },
        Step {
            text: "Pick a field in the tree, for example STRESS > MISES.",
            done: false,
            cmd: None,
        },
    ]
}

/// Draws the checklist; `problems` is the number of problems of the model check.
pub fn show(ui: &mut Ui, steps: &[Step], problems: usize) -> Option<Clicked> {
    let mut clicked = None;
    ui.weak("What to do next:");
    ui.add_space(6.0);
    egui::Grid::new("checklist")
        .num_columns(2)
        .spacing([8.0, 8.0])
        .show(ui, |ui| {
            for step in steps {
                ui.horizontal(|ui| {
                    tick(ui, step.done);
                    let text = RichText::new(step.text);
                    ui.label(if step.done { text.weak() } else { text });
                });
                match step.cmd.filter(|_| !step.done) {
                    Some(cmd) => {
                        let label = command::registry()
                            .iter()
                            .find(|c| c.cmd == cmd)
                            .map_or_else(String::new, |c| c.name());
                        if ui.button(label).clicked() {
                            clicked = Some(Clicked::Run(cmd));
                        }
                    }
                    None => {
                        ui.label("");
                    }
                }
                ui.end_row();
            }
        });
    if problems > 0 {
        ui.add_space(10.0);
        let text = match problems {
            1 => "1 problem".to_owned(),
            n => format!("{n} problems"),
        };
        if ui
            .link(format!("The model check reports {text}."))
            .clicked()
        {
            clicked = Some(Clicked::Problems);
        }
    }
    clicked
}

/// A tick, or an empty box, drawn with the painter.
fn tick(ui: &mut Ui, done: bool) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(14.0, 14.0), egui::Sense::hover());
    let painter = ui.painter();
    let color = if done {
        crate::style::ok()
    } else {
        ui.visuals().weak_text_color()
    };
    painter.rect_stroke(
        rect.shrink(1.0),
        2.0,
        egui::Stroke::new(1.0, color),
        egui::StrokeKind::Inside,
    );
    if done {
        let r = rect.shrink(3.5);
        let points = vec![
            egui::pos2(r.left(), r.center().y),
            egui::pos2(r.left() + r.width() * 0.4, r.bottom()),
            egui::pos2(r.right(), r.top()),
        ];
        painter.add(egui::Shape::line(points, egui::Stroke::new(1.8, color)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_empty_model_asks_for_everything_and_offers_only_what_it_can() {
        let steps = fe_model(None);
        assert!(steps.iter().all(|s| !s.done));
        // Supports and loads need a step first.
        let offered: Vec<_> = steps.iter().filter_map(|s| s.cmd).collect();
        assert_eq!(offered, vec![Cmd::Material, Cmd::Section, Cmd::Step]);
    }
}
