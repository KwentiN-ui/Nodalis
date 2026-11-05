//! The Display tab of the Results workspace: the tiles of the deformation, the contours and
//! the increment, the values their labels show and the panels they open.

use super::*;
use crate::command::Cmd;
use crate::results::format_value;
use plx_results::AnalysisKind as AnalysisKindAlias;
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
        let step = view.current_step().map(|s| s.increments.as_slice());
        let (first_in_step, last_in_step) = (
            step.and_then(|s| s.first()).copied().unwrap_or(0),
            step.and_then(|s| s.last()).copied().unwrap_or(last),
        );
        match cmd {
            Cmd::GlobalContours => {
                (self.results.get(self.current_result)).is_some_and(|m| m.global.is_some())
            }
            Cmd::Representation => !directional_fields(view).is_empty(),
            Cmd::ResultStep | Cmd::Increment => !view.increments.is_empty(),
            Cmd::FirstIncrement => current > first_in_step,
            Cmd::PreviousIncrement => current > 0,
            Cmd::NextIncrement => current < last,
            Cmd::LastIncrement => current < last_in_step,
            Cmd::AnimateScaleFactor | Cmd::AnimateStep => !view.increments.is_empty(),
            Cmd::AnimateAllSteps => view.steps.steps.len() > 1,
            Cmd::AnimatePositions => view.current_step().is_some_and(|s| s.has_positions()),
            Cmd::ModeSound => self.results_command_visible(cmd),
            _ => true,
        }
    }

    /// Whether the result tile is in the ribbon: Sound only while modes of a frequency step
    /// are shown, or its window is open.
    pub(super) fn results_command_visible(&self, cmd: Cmd) -> bool {
        match cmd {
            Cmd::ModeSound => (self.shown_results_view())
                .filter(|_| self.tree_view == TreeView::Results)
                .is_some_and(|view| {
                    view.sound.is_some()
                        || (view.current_increment())
                            .is_some_and(|i| i.kind == plx_results::AnalysisKind::Frequency)
                }),
            _ => true,
        }
    }

    /// Whether the result tile shows pressed.
    pub(super) fn results_command_checked(&self, cmd: Cmd) -> bool {
        let Some(view) = self.shown_results_view() else {
            return false;
        };
        let playing = |kind| view.animation.as_ref().is_some_and(|a| a.kind == kind);
        match cmd {
            Cmd::ShowUndeformed => view.show_undeformed,
            Cmd::GlobalContours => view.show_global_contours,
            Cmd::Transformations => {
                !view.transformations.is_empty() || self.transformation_dialog.is_some()
            }
            Cmd::Animate => view.animation.is_some(),
            Cmd::AnimateScaleFactor => playing(AnimationKind::ScaleFactor),
            Cmd::AnimateStep => playing(AnimationKind::Step),
            Cmd::AnimateAllSteps => playing(AnimationKind::AllSteps),
            Cmd::AnimatePositions => playing(AnimationKind::Positions),
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
        let step = view.current_step().map(|s| s.increments.clone());
        let first_in_step = step.as_ref().and_then(|s| s.first()).copied().unwrap_or(0);
        let last_in_step = step
            .as_ref()
            .and_then(|s| s.last())
            .copied()
            .unwrap_or(last);
        match cmd {
            Cmd::ShowUndeformed => view.show_undeformed = !view.show_undeformed,
            Cmd::GlobalContours => view.show_global_contours = !view.show_global_contours,
            Cmd::FirstIncrement => view.switch_increment(first_in_step),
            Cmd::PreviousIncrement => view.switch_increment(current.saturating_sub(1)),
            Cmd::NextIncrement => view.switch_increment((current + 1).min(last)),
            Cmd::LastIncrement => view.switch_increment(last_in_step),
            Cmd::Animate => {
                if view.animation.is_some() {
                    view.stop_animation();
                } else {
                    view.start_animation(view.automatic_animation());
                }
            }
            Cmd::AnimateScaleFactor => view.start_animation(AnimationKind::ScaleFactor),
            Cmd::AnimateStep => view.start_animation(AnimationKind::Step),
            Cmd::AnimateAllSteps => view.start_animation(AnimationKind::AllSteps),
            Cmd::AnimatePositions => view.start_animation(AnimationKind::Positions),
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
            Cmd::Deformation => view.active_deformation().label(),
            Cmd::Representation => view.representation().label().into(),
            Cmd::ColorLevels => view.color_scale.levels.to_string(),
            // A long step name is cut, so the tile keeps its size; the panel has it in full.
            Cmd::ResultStep => {
                let title = view.step_title(view.current_step_index()?);
                match title.char_indices().nth(16) {
                    Some((end, _)) => format!("{} ...", title[..end].trim_end()),
                    None => title,
                }
            }
            Cmd::Increment => view.entry_value()?,
            _ => return None,
        })
    }

    /// The name of a result tile that follows the shown step: "Mode" for the increments of
    /// a frequency step.
    pub(super) fn results_title(&self, cmd: Cmd) -> Option<&'static str> {
        let view = self.shown_results_view()?;
        match cmd {
            Cmd::Increment => view.current_step().map(|s| s.entry_noun()),
            // Modes are scaled on their own.
            Cmd::Deformation => view.shows_mode().then_some("Mode Scale"),
            _ => None,
        }
    }

    /// The timeline of the steps below the 3D view, for results with more than one
    /// increment; it shows the increment clicked or dragged to.
    pub(super) fn timeline(&mut self, ui: &mut egui::Ui) {
        let shown = self.settings.graphics.timeline
            && self.tree_view == TreeView::Results
            && (self.shown_results_view()).is_some_and(|v| v.increments.len() > 1);
        if !shown {
            return;
        }
        let mut hide = false;
        let chosen = egui::Panel::bottom("timeline")
            .resizable(false)
            .exact_size(crate::timeline::HEIGHT)
            .frame(egui::Frame::new().fill(crate::style::control()))
            .show(ui, |ui| {
                self.shown_results_view()
                    .and_then(|v| crate::timeline::show(ui, v, &mut hide))
            })
            .inner;
        if hide {
            self.settings.graphics.timeline = false;
        }
        if let Some(index) = chosen
            && let Some(view) = self.shown_results_mut()
        {
            view.switch_increment(index);
            self.results_changed = true;
        }
    }

    /// Moves through the increments and steps of the results with the keyboard: arrows
    /// for the increments, Home and End within the step, Ctrl+Page Up and Ctrl+Page Down
    /// for the steps.
    pub(super) fn results_keys(&mut self, ctx: &egui::Context) {
        if self.tree_view != TreeView::Results || ctx.egui_wants_keyboard_input() {
            return;
        }
        use egui::{Key, Modifiers};
        let pressed = |modifiers, key| ctx.input_mut(|i| i.consume_key(modifiers, key));
        for (key, cmd) in [
            (Key::ArrowLeft, Cmd::PreviousIncrement),
            (Key::ArrowRight, Cmd::NextIncrement),
            (Key::Home, Cmd::FirstIncrement),
            (Key::End, Cmd::LastIncrement),
        ] {
            if pressed(Modifiers::NONE, key) && self.results_command_enabled(cmd) {
                self.run_results_command(cmd);
            }
        }
        for (key, forward) in [(Key::PageUp, false), (Key::PageDown, true)] {
            if pressed(Modifiers::COMMAND, key)
                && let Some(view) = self.shown_results_mut()
                && let Some(step) = view.neighbour_step(forward)
            {
                view.switch_step(step);
                self.results_changed = true;
            }
        }
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
            Cmd::ResultStep => step_panel(ui, view),
            Cmd::Increment => increment_panel(ui, view),
            _ => false,
        };
        if changed {
            self.results_changed = true;
        }
        if cmd == Cmd::ResultStep {
            ui.separator();
            ui.checkbox(&mut self.settings.graphics.timeline, "Timeline")
                .on_hover_text("Shows the steps as a timeline below the 3D view.");
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

/// The deformation and its factor, which can be typed for "User defined". A mode has a
/// deformation of its own, without a true scale: its size means nothing, so the user types
/// the largest displacement drawn instead of a factor.
fn deformation_panel(ui: &mut egui::Ui, view: &mut ResultsView) -> bool {
    let mode = view.shows_mode();
    let before = (
        view.deformation,
        view.user_scale,
        view.mode_deformation,
        view.mode_amplitude,
    );
    if mode {
        ui.strong("Mode shapes");
    }
    for choice in Deformation::CHOICES {
        if mode && choice == Deformation::TrueScale {
            continue;
        }
        let selected = view.active_deformation() == choice;
        if ui.radio(selected, choice.label()).clicked() && !selected {
            if mode {
                view.mode_deformation = choice;
            } else {
                view.deformation = choice;
                view.deformation_chosen = true;
            }
        }
    }
    ui.separator();
    ui.horizontal(|ui| {
        let user = view.active_deformation() == Deformation::UserDefined;
        let (label, value, hint) = if mode {
            let unit = view.units.unit(plx_model::Quantity::Length);
            (
                format!("Largest displacement [{unit}]"),
                if user {
                    view.mode_amplitude
                } else {
                    f64::from(view.scale())
                },
                "Choose User defined to type the largest displacement drawn",
            )
        } else {
            (
                "Factor".to_owned(),
                if user {
                    view.user_scale
                } else {
                    f64::from(view.scale())
                },
                "Choose User defined to type a factor",
            )
        };
        ui.label(label);
        let id = ui.id().with(("deformation factor", mode));
        let focused = ui.memory(|m| m.has_focus(id));
        let mut text = focused
            .then(|| ui.data_mut(|d| d.get_temp::<String>(id)))
            .flatten()
            .unwrap_or_else(|| numeric::format_physical(value));
        let response = ui
            .add_enabled(
                user,
                egui::TextEdit::singleline(&mut text)
                    .id(id)
                    .desired_width(90.0),
            )
            .on_disabled_hover_text(hint);
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
                if mode {
                    view.mode_amplitude = value;
                } else {
                    view.user_scale = value;
                }
            }
        }
    });
    if mode {
        ui.weak("Mode shapes are kept normalized: the largest displacement is 1.");
    }
    let after = (
        view.deformation,
        view.user_scale,
        view.mode_deformation,
        view.mode_amplitude,
    );
    after != before
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

/// The steps of the results with their kind, entries and time range; choosing one shows
/// its last increment.
fn step_panel(ui: &mut egui::Ui, view: &mut ResultsView) -> bool {
    let mut chosen = None;
    let current = view.current_step_index();
    ui.weak("Steps");
    egui::ScrollArea::vertical()
        .max_height(360.0)
        .show(ui, |ui| {
            egui::Grid::new("result steps")
                .num_columns(2)
                .spacing([16.0, 2.0])
                .show(ui, |ui| {
                    for (index, step) in view.steps.steps.iter().enumerate() {
                        let has = !step.increments.is_empty();
                        let title = view.step_title(index);
                        let response = ui.add_enabled(
                            has,
                            egui::Button::selectable(current == Some(index), title),
                        );
                        if response.clicked() {
                            chosen = Some(index);
                        }
                        let response = ui.add_enabled(
                            has,
                            egui::Label::new(egui::RichText::new(step_summary(view, index)).weak())
                                .sense(egui::Sense::click()),
                        );
                        if response.clicked() {
                            chosen = Some(index);
                        }
                        ui.end_row();
                    }
                });
        });
    ui.separator();
    ui.weak("Ctrl+Page Up / Ctrl+Page Down: step before / after");
    if chosen.is_some() {
        ui.close();
    }
    match chosen {
        Some(index) if Some(index) != current => {
            view.switch_step(index);
            true
        }
        _ => false,
    }
}

/// The kind, the number of entries and the time range of a step, for the step list.
fn step_summary(view: &ResultsView, index: usize) -> String {
    let step = &view.steps.steps[index];
    let mut parts = vec![step.description()];
    if let Some(source) = &step.inherited_from {
        parts.push(format!("from {source}"));
    }
    let count = view.entry_count(index).unwrap_or(0);
    if step.increments.is_empty() {
        parts.push("no results".into());
    } else {
        let noun = step.entry_noun().to_lowercase();
        parts.push(format!(
            "{count} {noun}{}",
            if count == 1 { "" } else { "s" }
        ));
    }
    if step.is_timed() && step.end_time > step.start_time {
        let unit = view.units.unit(plx_model::Quantity::Time);
        parts.push(format!(
            "{} to {} {unit}",
            format_value(step.start_time as f32),
            format_value(step.end_time as f32)
        ));
    }
    if !step.converged {
        parts.push("not converged".into());
    }
    parts.join(" \u{b7} ")
}

/// The increments, modes or positions of the shown step as a table with the values that
/// fit the step: step and total time, frequency or buckling factor; the shown one selected.
fn increment_panel(ui: &mut egui::Ui, view: &mut ResultsView) -> bool {
    use plx_results::steps::StepCategory;
    let Some(step) = view.current_step().cloned() else {
        return false;
    };
    let mut chosen = None;
    let noun = step.entry_noun();
    let plural = match noun {
        "Frequency" => "Frequencies".to_owned(),
        _ => format!("{noun}s"),
    };
    ui.weak(format!("{plural} of {}", step.name));
    let time = view.units.unit(plx_model::Quantity::Time);
    let hertz = view.units.unit(plx_model::Quantity::Frequency);
    let first = step.increments.first().map(|&i| view.increments[i].kind);
    let value_column = match (step.category, first) {
        (StepCategory::SteadyStateDynamics, _) => Some(format!("Frequency ({hertz})")),
        (_, Some(AnalysisKindAlias::Frequency | AnalysisKindAlias::ComplexFrequency)) => {
            Some(format!("Frequency ({hertz})"))
        }
        (_, Some(AnalysisKindAlias::Buckling)) => Some("Buckling factor".to_owned()),
        _ => None,
    };
    let timed = step.is_timed() && value_column.is_none();
    let cutbacks = (step.increments.iter()).any(|&i| {
        view.steps
            .place(i)
            .and_then(|p| p.attempts)
            .is_some_and(|a| a > 1)
    });
    // The header stays above the list as it scrolls, in columns as wide as the list's.
    const COLUMN: f32 = 72.0;
    egui::Grid::new("step increments header")
        .num_columns(4)
        .spacing([14.0, 2.0])
        .min_col_width(COLUMN)
        .show(ui, |ui| {
            let header = |ui: &mut egui::Ui, text: String| {
                ui.add_sized(
                    [COLUMN, 18.0],
                    egui::Label::new(egui::RichText::new(text).weak()),
                );
            };
            header(ui, noun.to_owned());
            if timed {
                header(ui, format!("Step time ({time})"));
                header(ui, format!("Total time ({time})"));
            } else if let Some(column) = &value_column {
                header(ui, column.clone());
            }
            if cutbacks {
                header(ui, "Attempts".into());
            }
            ui.end_row();
        });
    egui::ScrollArea::vertical()
        .max_height(320.0)
        .show(ui, |ui| {
            egui::Grid::new("step increments")
                .num_columns(4)
                .spacing([14.0, 2.0])
                .min_col_width(COLUMN)
                .show(ui, |ui| {
                    for &index in &step.increments {
                        let selected = index == view.increment;
                        let mut first = true;
                        let mut row = |ui: &mut egui::Ui, text: String| {
                            let button = egui::Button::selectable(selected, text);
                            let response = ui.add_sized([COLUMN, 18.0], button);
                            if response.clicked() {
                                chosen = Some(index);
                            }
                            // The shown entry scrolls into view once, when the list opens or
                            // another entry is shown.
                            let seen = crate::ids::window(ui.ctx(), "increment panel shown entry");
                            if selected
                                && std::mem::take(&mut first)
                                && ui.data(|d| d.get_temp::<usize>(seen)) != Some(index)
                            {
                                response.scroll_to_me(Some(egui::Align::Center));
                                ui.data_mut(|d| d.insert_temp(seen, index));
                            }
                        };
                        let label = view.entry_label(index);
                        let number = label
                            .strip_prefix(noun)
                            .map_or(label.clone(), |n| n.trim().to_owned());
                        row(ui, number);
                        let place = view.steps.place(index).copied().unwrap_or_default();
                        if timed {
                            let text =
                                |v: Option<f64>| v.map_or("-".into(), |v| format_value(v as f32));
                            row(ui, text(place.step_time));
                            row(ui, text(place.total_time));
                        } else if value_column.is_some() {
                            row(ui, format_value(view.increments[index].value as f32));
                        }
                        if cutbacks {
                            let attempts = place.attempts.unwrap_or(1);
                            row(
                                ui,
                                if attempts > 1 {
                                    format!("{attempts}, cutback")
                                } else {
                                    "1".into()
                                },
                            );
                        }
                        ui.end_row();
                    }
                });
        });
    if chosen.is_some() {
        ui.close();
    }
    match chosen {
        Some(index) if index != view.increment => {
            view.switch_increment(index);
            true
        }
        _ => false,
    }
}
