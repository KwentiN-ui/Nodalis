//! The unit system of the project, chosen at the bottom right of the project window.
//!
//! Every value is stored in SI units ([`plx_model::STORED`]); the unit system only says in
//! which units the blocks of the schematic, their editors and the analyses show the values
//! and take them typed. The analyses follow it: each keeps it in its model properties, which
//! the analysis window reads, and an open window reads its analysis again once it changes,
//! so that its results are shown in the new units too.

use plx_model::{Quantity, UnitSystem};

use super::ProjectApp;

/// The unit systems the project offers: all but Unitless, which would show the SI values
/// without their units.
pub(super) const CHOICES: [UnitSystem; 5] = [
    UnitSystem::MKgSC,
    UnitSystem::MmTonSC,
    UnitSystem::MmKgSC,
    UnitSystem::MTonSC,
    UnitSystem::InLbSF,
];

/// The unit system a new project starts in: the one chosen last, if the project offers it.
pub(super) fn first(last: UnitSystem) -> UnitSystem {
    if CHOICES.contains(&last) {
        last
    } else {
        UnitSystem::default()
    }
}

/// A few units of the system, to tell it in a tooltip.
pub(super) fn summary(units: UnitSystem) -> String {
    let unit = |q| units.unit(q);
    format!(
        "Length {}, mass {}, time {}, temperature {}\nForce {}, moment {}, stress {}, density {}",
        unit(Quantity::Length),
        unit(Quantity::Mass),
        unit(Quantity::Time),
        unit(Quantity::Temperature),
        unit(Quantity::Force),
        unit(Quantity::Moment),
        unit(Quantity::Pressure),
        unit(Quantity::Density),
    )
}

impl ProjectApp {
    /// The menu button of the unit system, at the right end of the status bar: a text like
    /// the others there, framed only while the pointer is on it.
    pub(super) fn units_menu(&mut self, ui: &mut egui::Ui) {
        let busy = (self.nodes.iter())
            .filter_map(|n| Some((n, n.as_analysis()?.window.as_ref()?.busy()?)))
            .map(|(n, why)| format!("{} cannot change its units while {why}.", n.name))
            .next();
        let mut chosen = None;
        let current = self.units;
        let button = ui.add_enabled_ui(busy.is_none(), |ui| {
            let text = format!("Units: {}", current.label());
            let button = egui::Button::new(text).frame_when_inactive(false);
            egui::containers::menu::MenuButton::from_button(button).ui(ui, |ui| {
                ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                for choice in CHOICES {
                    let response = ui
                        .selectable_label(choice == current, choice.label())
                        .on_hover_text(summary(choice));
                    if response.clicked() {
                        chosen = Some(choice);
                        ui.close();
                    }
                }
            })
        });
        let tip = format!(
            "The unit system the blocks and the analyses of the project show values in and \
             take them typed. Values are stored in SI units: another unit system changes only \
             what is shown.\n{}",
            summary(current)
        );
        let response = button.inner.0;
        match busy {
            Some(why) => response.on_disabled_hover_text(why),
            None => response.on_hover_text(tip),
        };
        if let Some(units) = chosen {
            self.set_units(ui.ctx(), units);
        }
    }

    /// Makes `units` the unit system of the project, as a change that can be undone.
    pub(super) fn set_units(&mut self, ctx: &egui::Context, units: UnitSystem) {
        if units == self.units {
            return;
        }
        self.checkpoint(&[]);
        self.units = units;
        // A new project starts in the units chosen last.
        self.settings.new_model.units = units;
        self.apply_units(ctx);
        self.output
            .push(format!("Unit system of the project: {}", units.label()));
    }

    /// Gives the analyses the unit system of the project. An open window reads its analysis
    /// again, so that what it shows, its results among them, is in the new units.
    pub(super) fn apply_units(&mut self, ctx: &egui::Context) {
        let (units, path) = (self.units, self.model_path());
        for node in &mut self.nodes {
            let (id, name) = (node.id, node.name.clone());
            let Some(analysis) = node.as_analysis_mut() else {
                continue;
            };
            if let Some(data) = &mut analysis.data {
                data.model.properties.units = units;
            }
            let Some(window) = &mut analysis.window else {
                continue;
            };
            if window
                .model()
                .is_none_or(|m| m.fe.properties.units == units)
            {
                continue;
            }
            if let Some(mut data) = window.data() {
                data.id = id;
                data.name = name;
                data.model.properties.units = units;
                window.load(ctx, path.clone(), data);
                analysis.seen = None;
            }
        }
    }

    /// Gives the analyses whose windows are closed the unit system of the project, should
    /// one have come with another: made, pasted or read from a file.
    pub(super) fn keep_units(&mut self) {
        let units = self.units;
        for analysis in self.nodes.iter_mut().filter_map(|n| n.as_analysis_mut()) {
            if let Some(data) = &mut analysis.data
                && data.model.properties.units != units
            {
                data.model.properties.units = units;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_new_project_starts_in_the_units_chosen_last_unless_unitless() {
        assert_eq!(first(UnitSystem::InLbSF), UnitSystem::InLbSF);
        assert_eq!(first(UnitSystem::Unitless), UnitSystem::default());
        assert!(CHOICES.iter().all(|u| u.has_units()));
    }

    #[test]
    fn the_summary_names_the_main_units() {
        let text = summary(UnitSystem::MmTonSC);
        assert!(text.contains("Length mm"), "{text}");
        assert!(text.contains("stress MPa"), "{text}");
    }
}
