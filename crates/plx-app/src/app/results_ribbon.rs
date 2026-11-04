//! The Display tab of the Results workspace: the tiles of the deformation, the contours and
//! the increment, the values their labels show and the panels they open.

use super::*;
use crate::command::Cmd;
use plx_results::display::Representation;

impl Workbench {
    /// Whether the result tile can be used now.
    pub(super) fn results_command_enabled(&self, cmd: Cmd) -> bool {
        let Some(view) = self
            .shown_results_view()
            .filter(|_| self.tree_view == TreeView::Results)
        else {
            return false;
        };
        let (current, last) = (view.increment, view.increments.len().saturating_sub(1));
        match cmd {
            Cmd::GlobalContours => {
                (self.results.get(self.current_result)).is_some_and(|m| m.global.is_some())
            }
            Cmd::Representation => !directional_fields(view).is_empty(),
            Cmd::Increment => !view.increments.is_empty(),
            Cmd::FirstIncrement | Cmd::PreviousIncrement => current > 0,
            Cmd::NextIncrement | Cmd::LastIncrement => current < last,
            // Only modes of a frequency step have a sound.
            Cmd::ModeSound => {
                view.sound.is_some()
                    || (view.current_increment())
                        .is_some_and(|i| i.kind == plx_results::AnalysisKind::Frequency)
            }
            _ => true,
        }
    }

    /// Whether the result tile shows pressed.
    pub(super) fn results_command_checked(&self, cmd: Cmd) -> bool {
        let Some(view) = self.shown_results_view() else {
            return false;
        };
        match cmd {
            Cmd::ShowUndeformed => view.show_undeformed,
            Cmd::GlobalContours => view.show_global_contours,
            Cmd::Transformations => {
                !view.transformations.is_empty() || self.transformation_dialog.is_some()
            }
            Cmd::Animate => view.animation.is_some(),
            Cmd::ModeSound => view.sound.is_some(),
            _ => false,
        }
    }

    /// Carries out a result tile that is a button.
    pub(super) fn run_results_command(&mut self, cmd: Cmd) {
        if cmd == Cmd::Transformations {
            self.open_transformation_dialog();
            return;
        }
        let Some(view) = self.shown_results_mut() else {
            return;
        };
        let last = view.increments.len().saturating_sub(1);
        let current = view.increment;
        match cmd {
            Cmd::ShowUndeformed => view.show_undeformed = !view.show_undeformed,
            Cmd::GlobalContours => view.show_global_contours = !view.show_global_contours,
            Cmd::FirstIncrement => view.switch_increment(0),
            Cmd::PreviousIncrement => view.switch_increment(current.saturating_sub(1)),
            Cmd::NextIncrement => view.switch_increment((current + 1).min(last)),
            Cmd::LastIncrement => view.switch_increment(last),
            Cmd::Animate => {
                if view.animation.is_some() {
                    view.stop_animation();
                } else {
                    view.start_animation(AnimationKind::ScaleFactor);
                }
            }
            Cmd::ModeSound => {
                view.sound = match view.sound {
                    Some(_) => None,
                    None => ModeSound::new(&view.increments, view.increment),
                };
                return;
            }
            _ => return,
        }
        self.results_changed = true;
    }

    /// The value a result tile shows under its name.
    pub(super) fn results_value(&self, cmd: Cmd) -> Option<String> {
        let view = self.shown_results_view()?;
        Some(match cmd {
            Cmd::Deformation => view.deformation.label(),
            Cmd::Representation => view.representation().label().into(),
            Cmd::ColorLevels => view.color_scale.levels.to_string(),
            Cmd::Increment => view.current_increment().map(ResultsView::increment_label)?,
            _ => return None,
        })
    }

    /// Fills the panel a result tile opens.
    pub(super) fn results_panel(&mut self, ui: &mut egui::Ui, cmd: Cmd) {
        let Some(view) = self.shown_results_mut() else {
            ui.weak("No results open");
            return;
        };
        let changed = match cmd {
            Cmd::Deformation => deformation_panel(ui, view),
            Cmd::Representation => display_panel(ui, view),
            Cmd::ColorLevels => {
                let changed = ui
                    .horizontal(|ui| {
                        ui.label("Color levels");
                        ui.add(
                            numeric::drag_value(&mut view.color_scale.levels)
                                .range(2..=plx_render::contour::MAX_LEVELS),
                        )
                        .changed()
                    })
                    .inner;
                ui.weak("2 to 24 bands; Color Scale sets the spectrum and limits.");
                changed
            }
            Cmd::Increment => increment_panel(ui, view),
            _ => false,
        };
        if changed {
            self.results_changed = true;
        }
    }
}

/// The fields of the shown increment that give directions, for arrows or streaks.
fn directional_fields(view: &ResultsView) -> Vec<String> {
    (view.current_increment().into_iter())
        .flat_map(|inc| inc.fields.iter())
        .filter(|f| f.rank != plx_results::FieldRank::Scalar)
        .map(|f| f.name.clone())
        .collect()
}

/// The deformation and its factor, which can be typed for "User defined".
fn deformation_panel(ui: &mut egui::Ui, view: &mut ResultsView) -> bool {
    let before = (view.deformation, view.user_scale);
    for choice in Deformation::CHOICES {
        let selected = view.deformation == choice;
        if ui.radio(selected, choice.label()).clicked() && !selected {
            view.deformation = choice;
            view.deformation_chosen = true;
        }
    }
    ui.separator();
    ui.horizontal(|ui| {
        ui.label("Factor");
        let user = view.deformation == Deformation::UserDefined;
        let factor = if user {
            view.user_scale
        } else {
            f64::from(view.scale())
        };
        let id = ui.id().with("deformation factor");
        let focused = ui.memory(|m| m.has_focus(id));
        let mut text = focused
            .then(|| ui.data_mut(|d| d.get_temp::<String>(id)))
            .flatten()
            .unwrap_or_else(|| numeric::format_physical(factor));
        let response = ui
            .add_enabled(
                user,
                egui::TextEdit::singleline(&mut text)
                    .id(id)
                    .desired_width(90.0),
            )
            .on_disabled_hover_text("Choose User defined to type a factor");
        if response.has_focus() {
            ui.data_mut(|d| d.insert_temp(id, text.clone()));
        }
        if user && (response.lost_focus() || response.changed()) {
            if response.lost_focus() {
                ui.data_mut(|d| d.remove_temp::<String>(id));
            }
            if let Some(value) = numeric::evaluate(&text)
                && value > 0.0
            {
                view.user_scale = value;
            }
        }
    });
    (view.deformation, view.user_scale) != before
}

/// Contours, arrows or streaks; the field whose directions they follow, its principal
/// direction and the options of the arrows or streaks.
fn display_panel(ui: &mut egui::Ui, view: &mut ResultsView) -> bool {
    let before = view.vector_display.clone();
    let directional = directional_fields(view);
    let shown_field = view.current().map(|(f, _)| f.name.clone());
    let directions = view.directions_field().map(|f| (f.name.clone(), f.rank));
    let shown = view.representation();
    let display = &mut view.vector_display;
    for choice in Representation::ALL {
        if ui.radio(shown == choice, choice.label()).clicked() && shown != choice {
            display.representation = choice;
            // Over a scalar contour, the directions come from the first field that has them.
            let scalar = directions
                .as_ref()
                .is_none_or(|(_, r)| *r == plx_results::FieldRank::Scalar);
            if choice != Representation::Contour && scalar {
                display.field = directional.first().cloned();
            }
        }
    }
    let drawn = shown != Representation::Contour;
    ui.separator();
    ui.add_enabled_ui(drawn, |ui| {
        ui.strong("Directions of");
        let shown_has = shown_field
            .as_ref()
            .is_some_and(|n| directional.contains(n));
        let current = directions.as_ref().map(|(n, _)| n.clone());
        if shown_has {
            let follows = current == shown_field;
            if ui.radio(follows, "Shown field").clicked() {
                display.field = None;
            }
        }
        for name in &directional {
            if shown_has && Some(name) == shown_field.as_ref() {
                continue;
            }
            if ui.radio(current.as_ref() == Some(name), name).clicked() {
                display.field = Some(name.clone());
            }
        }
    });
    let tensor = directions.is_some_and(|(_, rank)| rank.is_tensor());
    ui.add_enabled_ui(drawn && tensor, |ui| {
        ui.separator();
        ui.strong("Principal direction");
        ui.horizontal_wrapped(|ui| {
            for choice in plx_results::Principal::ALL {
                ui.radio_value(&mut display.principal, choice, choice.label());
            }
        });
    });
    match shown {
        Representation::Arrows => {
            ui.separator();
            ui.strong("Arrows");
            arrow_options(ui, &mut display.arrows);
        }
        Representation::Lic => {
            ui.separator();
            ui.strong("LIC streaks");
            lic_options(ui, &mut display.lic);
        }
        Representation::Contour => {}
    }
    view.vector_display != before
}

fn arrow_options(ui: &mut egui::Ui, arrows: &mut plx_results::display::ArrowOptions) {
    use plx_results::display::ArrowOptions;
    egui::Grid::new("arrow options")
        .num_columns(2)
        .show(ui, |ui| {
            ui.label("Number of arrows");
            ui.add(
                numeric::drag_value(&mut arrows.count)
                    .range(ArrowOptions::MIN_COUNT..=ArrowOptions::MAX_COUNT)
                    .speed(20.0),
            );
            ui.end_row();
            ui.label("Length");
            ui.add(
                numeric::drag_value(&mut arrows.length)
                    .range(0.1..=5.0)
                    .speed(0.02),
            )
            .on_hover_text("Length of the longest arrow relative to the arrow spacing");
            ui.end_row();
        });
    ui.checkbox(&mut arrows.by_magnitude, "Length by magnitude");
    ui.checkbox(&mut arrows.colored, "Arrows in contour colors")
        .on_hover_text("Otherwise the arrows are black over the contours");
}

fn lic_options(ui: &mut egui::Ui, lic: &mut plx_results::display::LicOptions) {
    use plx_results::display::LicOptions;
    egui::Grid::new("lic options")
        .num_columns(2)
        .show(ui, |ui| {
            ui.label("Streak length");
            ui.add(
                numeric::drag_value(&mut lic.length)
                    .range(LicOptions::LENGTH)
                    .speed(0.2),
            )
            .on_hover_text("Length of the streaks to either side of a pixel, in pixels");
            ui.end_row();
            ui.label("Grain size");
            ui.add(
                numeric::drag_value(&mut lic.grain)
                    .range(LicOptions::GRAIN)
                    .speed(0.002),
            )
            .on_hover_text(
                "Size of the noise grains in percent of the model size; never finer than a \
                 pixel",
            );
            ui.end_row();
            ui.label("Contrast");
            ui.add(
                numeric::drag_value(&mut lic.contrast)
                    .range(LicOptions::CONTRAST)
                    .speed(0.05),
            );
            ui.end_row();
            ui.label("Mix");
            ui.add(
                numeric::drag_value(&mut lic.mix)
                    .range(0.0..=1.0)
                    .speed(0.01),
            )
            .on_hover_text("How much of the brightness the streaks set, 0 to 1");
            ui.end_row();
        });
    if ui.button("Defaults").clicked() {
        *lic = LicOptions::default();
    }
}

/// The increments of the results, the shown one selected.
fn increment_panel(ui: &mut egui::Ui, view: &mut ResultsView) -> bool {
    let mut chosen = None;
    ui.weak("Step, increment");
    egui::ScrollArea::vertical()
        .max_height(320.0)
        .show(ui, |ui| {
            for (index, inc) in view.increments.iter().enumerate() {
                let label = ResultsView::increment_label(inc);
                if ui
                    .selectable_label(index == view.increment, label)
                    .clicked()
                {
                    chosen = Some(index);
                }
            }
        });
    match chosen {
        Some(index) if index != view.increment => {
            view.switch_increment(index);
            true
        }
        _ => false,
    }
}
