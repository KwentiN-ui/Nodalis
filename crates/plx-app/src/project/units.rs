//! The unit system of the project, chosen at the bottom right of the project window.
//!
//! Every value is stored in SI units ([`plx_model::STORED`]); the unit system only says in
//! which units the blocks of the schematic, their editors and the analyses show the values
//! and take them typed. The analyses follow it: each keeps it in its model properties, which
//! the analysis window reads; an open window shows its model in the new units at once and
//! reads its results again in them, keeping what was defined on them. Settings saved with
//! bound results know the units they were taken in and are converted when the results open.

use plx_model::UnitSystem;

use super::ProjectApp;

impl ProjectApp {
    /// The menu button of the unit system at the right end of the status bar.
    pub(super) fn units_menu(&mut self, ui: &mut egui::Ui) {
        let busy = self.units_busy();
        if let Some(units) = crate::units_menu::button(ui, self.units, busy.as_deref()) {
            self.set_units(units);
        }
    }

    /// Why the unit system cannot change now: an open analysis is busy, and would lose
    /// what it works on when it reads its analysis again.
    fn units_busy(&self) -> Option<String> {
        (self.nodes.iter())
            .filter_map(|n| Some((n, n.as_analysis()?.window.as_ref()?.busy()?)))
            .map(|(n, why)| format!("{} cannot change its units while {why}.", n.name))
            .next()
    }

    /// Makes `units` the unit system of the project, as a change that can be undone.
    pub(super) fn set_units(&mut self, units: UnitSystem) {
        if units == self.units {
            return;
        }
        if let Some(why) = self.units_busy() {
            self.output.push(format!("Unit system not changed: {why}"));
            return;
        }
        self.checkpoint(&[]);
        self.units = units;
        // A new project starts in the units chosen last.
        self.settings.new_model.units = units;
        self.apply_units();
        self.output
            .push(format!("Unit system of the project: {}", units.label()));
    }

    /// Gives the analyses the unit system of the project. An open window shows its model in
    /// the new units at once and reads its results again in them, keeping what was defined
    /// on them, the camera and the workspace.
    pub(super) fn apply_units(&mut self) {
        let units = self.units;
        for node in &mut self.nodes {
            let Some(analysis) = node.as_analysis_mut() else {
                continue;
            };
            if let Some(data) = &mut analysis.data {
                data.model.properties.units = units;
            }
            if let Some(window) = &mut analysis.window
                && window.set_units(units)
            {
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
