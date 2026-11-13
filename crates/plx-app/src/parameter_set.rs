//! The editor of a Parameters block of the project schematic: named numbers that value links
//! give to other blocks, such as the size of a mesh block.

use plx_model::{Parameter, UnitSystem, ValueKind};

use crate::numeric;

pub enum ParameterSetResult {
    Open,
    /// Closed with OK: the parameters, each with the name it had before, if it had one, so
    /// that links follow a renamed parameter; and the unit system values are shown in.
    Ok(Vec<(Option<String>, Parameter)>),
    Cancel,
}

pub struct ParameterSetEditor {
    /// The block edited.
    pub id: u64,
    name: String,
    rows: Vec<(Option<String>, Parameter)>,
    /// The unit system of the project, which the values are shown and typed in.
    units: UnitSystem,
}

impl ParameterSetEditor {
    pub fn new(id: u64, name: &str, parameters: &[Parameter], units: UnitSystem) -> Self {
        Self {
            id,
            name: name.to_owned(),
            rows: (parameters.iter())
                .map(|p| (Some(p.name.clone()), p.clone()))
                .collect(),
            units,
        }
    }

    pub fn rename(&mut self, name: &str) {
        name.clone_into(&mut self.name);
    }

    /// Why the parameters cannot be kept: a name missing or given twice.
    fn problem(&self) -> Option<String> {
        for (i, (_, parameter)) in self.rows.iter().enumerate() {
            let name = parameter.name.trim();
            if name.is_empty() {
                return Some("Every parameter needs a name.".into());
            }
            if (self.rows[..i].iter()).any(|(_, p)| p.name.trim().eq_ignore_ascii_case(name)) {
                return Some(format!("{name} is there twice."));
            }
        }
        None
    }

    pub fn show(&mut self, ctx: &egui::Context) -> ParameterSetResult {
        let mut result = ParameterSetResult::Open;
        let mut open = true;
        egui::Window::new(format!("Parameters - {}", self.name))
            .id(crate::ids::window(ctx, ("parameter set", self.id)))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                self.table(ui);
                if ui
                    .button("Add Parameter")
                    .on_hover_text("Adds a length; choose another kind in its row.")
                    .clicked()
                {
                    let name = (1..)
                        .map(|i| format!("Parameter-{i}"))
                        .find(|n| !self.rows.iter().any(|(_, p)| &p.name == n))
                        .unwrap_or_default();
                    self.rows
                        .push((None, Parameter::new(name, ValueKind::LENGTH, 0.1)));
                }
                let problem = self.problem();
                if let Some(problem) = &problem {
                    ui.colored_label(crate::style::error(), problem);
                }
                ui.weak(
                    "Drag the pin of a parameter in the schematic to a parameter of another \
                     block, such as the length of a mesh block or a force exposed by an \
                     analysis: it then takes this value.",
                );
                ui.separator();
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if ui.button("Cancel").clicked() {
                        result = ParameterSetResult::Cancel;
                    }
                    if ui
                        .add_enabled(problem.is_none(), egui::Button::new("OK"))
                        .clicked()
                    {
                        let rows = (self.rows.iter())
                            .map(|(old, p)| {
                                let mut p = p.clone();
                                p.name = p.name.trim().to_owned();
                                p.value = p.kind.clean(p.value);
                                (old.clone(), p)
                            })
                            .collect();
                        result = ParameterSetResult::Ok(rows);
                    }
                });
            });
        if !open {
            result = ParameterSetResult::Cancel;
        }
        result
    }

    fn table(&mut self, ui: &mut egui::Ui) {
        let units = self.units;
        let mut remove = None;
        egui::Grid::new("parameter set table")
            .num_columns(4)
            .spacing([10.0, 6.0])
            .striped(true)
            .show(ui, |ui| {
                ui.strong("Name");
                ui.strong("Kind");
                ui.strong("Value");
                ui.label("");
                ui.end_row();
                for (i, (_, parameter)) in self.rows.iter_mut().enumerate() {
                    ui.add_sized(
                        [140.0, 18.0],
                        egui::TextEdit::singleline(&mut parameter.name),
                    );
                    egui::ComboBox::from_id_salt(("parameter kind", i))
                        .selected_text(parameter.kind.label())
                        .width(170.0)
                        .height(320.0)
                        .show_ui(ui, |ui| {
                            for kind in ValueKind::all() {
                                ui.selectable_value(&mut parameter.kind, kind, kind.label());
                            }
                        });
                    let drag = numeric::of_kind(&mut parameter.value, parameter.kind, units);
                    crate::numeric::fitted(ui, egui::vec2(110.0, 18.0), drag, "");
                    if ui.small_button("Remove").clicked() {
                        remove = Some(i);
                    }
                    ui.end_row();
                }
            });
        if let Some(i) = remove {
            self.rows.remove(i);
        }
    }
}
