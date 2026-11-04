//! The dialog of a hot spot definition in the Results tree: the toe is picked on the shown
//! results, its path is drawn while the dialog is open, and below the inputs a plot shows
//! the stress along the path in the shown increment with the read-out points and the
//! extrapolation to the toe, so that the hot spot stress can be checked at a glance.

use egui::Ui;
use plx_model::Quantity;
use plx_results::hot_spot::{Extrapolation, HotSpot, HotSpotComponent, extrapolation_weights};

use crate::field_output_dialog::DialogAction;
use crate::hot_spots::{Located, short};
use crate::model::{Highlight, Hit, Model};
use crate::numeric;
use crate::selection::{Operation, Picker, Target};
use crate::setup::{NODE_SOURCES, RegionDraft, Source};
use crate::table_export;
use crate::viewport::{BoxSelect, Preview};

pub struct HotSpotDialog {
    /// Index of the edited definition; `None` creates a new one.
    pub edit: Option<usize>,
    hot_spot: HotSpot,
    toe: RegionDraft,
    /// Own read-out distances as typed, e.g. "2, 6".
    distances: String,
    /// Names of the other definitions.
    taken: Vec<String>,
    pub error: Option<String>,
    picker: Picker,
    /// Paths of the definition as entered, for the plot.
    located: Located,
    /// The button to show or remove the plot in the 3D view was clicked.
    plot_toggled: bool,
    /// Name of the hot spot when its plot was put into the 3D view, by which the plot refers
    /// to it while the dialog is open.
    plot_key: Option<String>,
    /// Height of the dialog's rows other than the plot in the last frame.
    rows_height: Option<f32>,
    /// The plot was drawn in the last frame.
    plotted: bool,
    /// The toe node whose path the plot shows.
    plotted_node: Option<plx_mesh::NodeId>,
}

/// Height of the plot until the dialog is resized, and the least it shrinks to.
const PLOT_HEIGHT: f32 = 240.0;
const MIN_PLOT_HEIGHT: f32 = 150.0;

impl HotSpotDialog {
    /// A new definition next to `existing`; it takes over the settings of the last one, so
    /// several welds of one plate are defined quickly.
    pub fn create(existing: &[HotSpot]) -> Self {
        let taken: Vec<String> = existing.iter().map(|h| h.name.clone()).collect();
        let name = plx_model::next_name("Hot_Spot", taken.iter().map(String::as_str));
        let hot_spot = match existing.last() {
            Some(last) => HotSpot {
                name,
                toe: plx_model::Region::Nodes(Vec::new()),
                ..last.clone()
            },
            None => HotSpot::new(name),
        };
        let distances = match &hot_spot.extrapolation {
            Extrapolation::Custom(d) => format_distances(d),
            _ => String::new(),
        };
        Self {
            edit: None,
            hot_spot,
            toe: RegionDraft::new(NODE_SOURCES, Target::Nodes),
            distances,
            taken,
            error: None,
            picker: Picker::default(),
            located: Located::default(),
            plot_toggled: false,
            plot_key: None,
            rows_height: None,
            plotted: false,
            plotted_node: None,
        }
    }

    /// The name by which a plot in the 3D view refers to the edited hot spot: its name when
    /// the plot was added, else its stored name, or for a new one the name entered.
    pub fn plot_name(&self, existing: &[HotSpot]) -> String {
        let stored = self.edit.and_then(|i| existing.get(i));
        (self.plot_key.clone())
            .or_else(|| stored.map(|h| h.name.clone()))
            .unwrap_or_else(|| self.hot_spot.name.clone())
    }

    /// The toe node whose path the plot shows, the one the 3D view draws.
    pub fn plotted_node(&self) -> Option<plx_mesh::NodeId> {
        self.plotted_node
    }

    /// Whether the plot is to be shown or removed in the 3D view; resets the request.
    pub fn take_plot_toggle(&mut self) -> bool {
        std::mem::take(&mut self.plot_toggled)
    }

    pub fn set_plot_key(&mut self, name: String) {
        self.plot_key = Some(name);
    }

    pub fn edit(existing: &[HotSpot], index: usize, mesh: &plx_mesh::FeMesh) -> Option<Self> {
        let hot_spot = existing.get(index)?.clone();
        let mut dialog = Self::create(existing);
        dialog.edit = Some(index);
        dialog.taken.remove(index);
        dialog.toe = RegionDraft::from_region(&hot_spot.toe, NODE_SOURCES, Target::Nodes, mesh);
        dialog.distances = match &hot_spot.extrapolation {
            Extrapolation::Custom(d) => format_distances(d),
            _ => String::new(),
        };
        dialog.hot_spot = hot_spot;
        Some(dialog)
    }

    pub fn title(&self) -> &'static str {
        if self.edit.is_some() {
            "Edit Hot Spot"
        } else {
            "Create Hot Spot"
        }
    }

    /// Whether clicks in the 3D view pick toe nodes.
    pub fn picks(&self) -> bool {
        self.toe.source == Source::Selection
    }

    pub fn click(&mut self, model: &Model, pick: Option<(&Hit, f32)>, operation: Operation) {
        self.toe.click(model, &self.picker, pick, operation);
    }

    pub fn box_select(&mut self, model: &Model, area: &BoxSelect, operation: Operation) {
        self.toe.box_select(model, &self.picker, area, operation);
    }

    pub fn preview(&self, model: &Model, hit: &Hit, precision: f32) -> Preview {
        self.toe.preview(model, &self.picker, hit, precision)
    }

    pub fn highlight(&self, model: &Model) -> Highlight {
        self.toe.highlight(model)
    }

    /// The definition as currently entered, for drawing its paths.
    pub fn hot_spot(&self) -> HotSpot {
        HotSpot {
            toe: self.toe.region(),
            ..self.hot_spot.clone()
        }
    }

    /// The definition, or why it cannot be created.
    pub fn output(&self) -> Result<HotSpot, String> {
        let hot_spot = self.hot_spot();
        let name = hot_spot.name.trim();
        if name.is_empty() {
            return Err("Please enter a name.".into());
        }
        if self.taken.iter().any(|t| t.eq_ignore_ascii_case(name)) {
            return Err(format!("The name {name} is already used."));
        }
        if self.toe.is_empty() {
            return Err("Please select a node or an edge at the weld toe.".into());
        }
        validate(&hot_spot)?;
        Ok(HotSpot {
            name: name.to_string(),
            ..hot_spot
        })
    }

    /// Prepares the dialog for the next definition after OK - New: same settings, new toe.
    pub fn next(&mut self, created: &HotSpot) {
        self.taken.push(created.name.clone());
        self.hot_spot.name =
            plx_model::next_name("Hot_Spot", self.taken.iter().map(String::as_str));
        self.toe = RegionDraft::new(NODE_SOURCES, Target::Nodes);
        self.error = None;
        self.plot_key = None;
    }

    pub fn show(&mut self, ctx: &egui::Context, model: &Model) -> DialogAction<HotSpot> {
        let mut action = DialogAction::Open;
        let mut open = true;
        let title = self.title();
        let builder = egui::Window::new(title)
            .id(crate::ids::window(ctx, "hot spot dialog"))
            .collapsible(false)
            .resizable(true)
            .pivot(egui::Align2::LEFT_TOP)
            .default_pos(ctx.content_rect().left_top() + egui::vec2(300.0, 90.0));
        let window = crate::dock::show(ctx, builder, title, &mut open, |ui| {
            ui.set_min_width(420.0);
            // The plot takes the height the window is resized to beyond its other rows.
            let plot_height = match self.rows_height {
                Some(rows) => (ui.max_rect().height() - rows).max(MIN_PLOT_HEIGHT),
                None => PLOT_HEIGHT,
            };
            egui::Grid::new("hot spot form")
                .max_col_width(crate::dock::column_limit(ui))
                .num_columns(2)
                .spacing([12.0, 6.0])
                .show(ui, |ui| {
                    form(
                        ui,
                        model,
                        &mut self.hot_spot,
                        &mut self.toe,
                        &mut self.distances,
                    )
                });
            ui.separator();
            self.results(ui, model, plot_height);
            if let Some(error) = &self.error {
                ui.colored_label(crate::style::error(), error);
            }
            ui.separator();
            crate::dock::footer(ui, |ui| {
                if ui.button("Cancel").clicked() {
                    action = DialogAction::Cancel;
                }
                let ok = ui.button("OK").clicked();
                let next = self.edit.is_none() && ui.button("OK - New").clicked();
                if ok || next {
                    match self.output() {
                        Ok(output) => action = DialogAction::Ok { output, next },
                        Err(error) => self.error = Some(error),
                    }
                }
            });
            // Only a drawn plot tells the height of the rest.
            self.rows_height = self.plotted.then(|| ui.min_rect().height() - plot_height);
        });
        if let Some(window) = window
            && self.toe.source == Source::Selection
        {
            let can_undo = self.toe.can_undo();
            let picked = self
                .picker
                .window(ctx, window.response.rect, Target::Nodes, can_undo);
            if let Some(picked) = picked {
                self.toe.action(model, picked);
            }
        }
        if !open {
            action = DialogAction::Cancel;
        }
        action
    }

    /// The stress along the path in the shown increment, its read-out points and the
    /// extrapolation to the toe, with buttons to put the plot into the 3D view or export it.
    fn results(&mut self, ui: &mut Ui, model: &Model, plot_height: f32) {
        self.plotted = false;
        self.plotted_node = None;
        let plot = match self.located.plot(model, &self.hot_spot()) {
            Ok(plot) => plot,
            Err(reason) => {
                ui.weak(reason);
                return;
            }
        };
        let increment = (model.results.as_ref())
            .map(crate::results::ResultsView::shown_label)
            .unwrap_or_default();
        ui.horizontal(|ui| {
            ui.strong(plot.component.short());
            ui.label(increment);
        });
        let mut summary = format!(
            "S_hs = {} at node {}",
            crate::results::format_value(plot.marks.hot_spot as f32),
            plot.node
        );
        if plot.paths > 1 {
            summary += &format!(" (highest of {} toe nodes)", plot.paths);
        }
        ui.label(summary);
        let readings: Vec<String> = (plot.marks.support.iter())
            .map(|[d, s]| {
                format!(
                    "S({}) = {}",
                    short(*d),
                    crate::results::format_value(*s as f32)
                )
            })
            .collect();
        ui.weak(readings.join("   "));
        let width = ui.available_width().max(360.0);
        let (rect, response) =
            ui.allocate_exact_size(egui::vec2(width, plot_height), egui::Sense::hover());
        crate::features::draw_plot(
            &ui.painter_at(rect),
            rect,
            response.hover_pos(),
            &plot.distances,
            &plot.values,
            &crate::features::Axes {
                x_label: "Distance",
                y_range: Some(plot.range()),
                marks: Some(&plot.marks),
                ..Default::default()
            },
            false,
        );
        self.plotted = true;
        self.plotted_node = Some(plot.node);
        let shown = (model.path_plots.iter())
            .any(|p| p.is_hot_spot(&self.plot_name(&model.hot_spots.definitions)));
        ui.horizontal(|ui| {
            let label = if shown {
                "Remove from View"
            } else {
                "Show in View"
            };
            if ui
                .button(label)
                .on_hover_text(
                    "Shows the plot as an annotation in the 3D view, where it stays after \
                     the dialog is closed. Drag it to move it, drag its corner to resize it; \
                     right-click it to show it on the path or to delete it.",
                )
                .clicked()
            {
                self.plot_toggled = true;
            }
            if ui.button("Copy to Clipboard").clicked() {
                ui.ctx().copy_text(plot.to_csv('\t'));
            }
            for (label, format) in [
                ("Save as CSV…", table_export::Format::Csv),
                ("Save as JSON…", table_export::Format::Json),
            ] {
                if ui.button(label).clicked()
                    && let Some(file) =
                        table_export::save_dialog(label, &self.hot_spot.name, format)
                {
                    self.error = table_export::write(&file, &plot.to_csv(';'), ';', format)
                        .err()
                        .map(|e| format!("{}: {e}", file.display()));
                }
            }
        });
        for warning in &plot.warnings {
            ui.colored_label(crate::style::warning(), warning);
        }
    }
}

fn form(
    ui: &mut Ui,
    model: &Model,
    hot_spot: &mut HotSpot,
    toe: &mut RegionDraft,
    text: &mut String,
) {
    ui.label("Name");
    ui.add(egui::TextEdit::singleline(&mut hot_spot.name).desired_width(200.0));
    ui.end_row();
    toe.ui_labeled(ui, model, "Weld toe", "toe", true);
    ui.label("");
    ui.weak("A node at the weld toe, or an edge: its node with the\nhighest hot spot stress is evaluated.");
    ui.end_row();
    let units = crate::hot_spots::units(model);
    ui.label("Extrapolation");
    let custom = matches!(hot_spot.extrapolation, Extrapolation::Custom(_));
    egui::ComboBox::from_id_salt("hot spot extrapolation")
        .selected_text(hot_spot.extrapolation.label())
        .width(240.0)
        .show_ui(ui, |ui| {
            for method in Extrapolation::IIW {
                let label = method.label();
                ui.selectable_value(&mut hot_spot.extrapolation, method, label);
            }
            if ui
                .selectable_label(custom, "Custom read-out points")
                .clicked()
                && !custom
            {
                let distances = hot_spot.distances_in(units);
                *text = format_distances(&distances);
                hot_spot.extrapolation = Extrapolation::Custom(distances);
            }
        });
    ui.end_row();
    if let Extrapolation::Custom(distances) = &mut hot_spot.extrapolation {
        let unit = units.unit(Quantity::Length);
        ui.label(if unit.is_empty() {
            "Distances".to_string()
        } else {
            format!("Distances [{unit}]")
        });
        let edit = egui::TextEdit::singleline(text)
            .hint_text("e.g. 2, 6, 10")
            .desired_width(200.0);
        if ui.add(edit).changed() {
            *distances = parse_distances(text);
        }
        ui.end_row();
    }
    ui.label("Plate thickness t");
    ui.add_enabled(
        hot_spot.extrapolation.uses_thickness(),
        numeric::quantity(&mut hot_spot.thickness, units, Quantity::Length)
            .range(0.0..=f64::MAX)
            .speed(0.1),
    );
    ui.end_row();
    ui.label("Read-out points");
    let distances = hot_spot.distances();
    let weights = extrapolation_weights(&distances);
    let mut formula = String::from("S_hs =");
    for (i, (d, w)) in distances.iter().zip(&weights).enumerate() {
        let sign = match (i, *w < 0.0) {
            (0, false) => "",
            (0, true) => " -",
            (_, false) => " +",
            (_, true) => " -",
        };
        formula += &format!("{sign} {:.3} S({})", w.abs(), short(*d));
    }
    ui.label(formula);
    ui.end_row();
    ui.label("Stress");
    egui::ComboBox::from_id_salt("hot spot component")
        .selected_text(hot_spot.component.label())
        .width(240.0)
        .show_ui(ui, |ui| {
            for component in HotSpotComponent::ALL {
                ui.selectable_value(&mut hot_spot.component, component, component.label());
            }
        });
    ui.end_row();
    ui.label("Path direction");
    ui.vertical(|ui| {
        ui.horizontal(|ui| {
            for (value, label) in hot_spot.direction.iter_mut().zip(["X", "Y", "Z"]) {
                ui.label(label);
                ui.add(numeric::drag_value(value).speed(0.05));
            }
            if ui
                .button("Flip")
                .on_hover_text("Reverses the path direction.")
                .clicked()
            {
                hot_spot.direction = hot_spot.direction.map(|v| if v == 0.0 { 0.0 } else { -v });
            }
        });
        ui.weak("Away from the weld toe; rotated into the plate surface across the weld.");
    });
    ui.end_row();
}

fn validate(hot_spot: &HotSpot) -> Result<(), String> {
    let distances = hot_spot.distances();
    if hot_spot.extrapolation.uses_thickness()
        && hot_spot.thickness.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater)
    {
        return Err("The plate thickness must be greater than zero.".into());
    }
    if distances.len() < 2 {
        return Err("Enter at least two read-out points.".into());
    }
    let mut sorted = distances.clone();
    sorted.sort_by(f64::total_cmp);
    if sorted[0] <= 0.0 || sorted.windows(2).any(|w| w[0] == w[1]) {
        return Err("The distances must be positive and different.".into());
    }
    if hot_spot.direction.iter().all(|&v| v == 0.0) {
        return Err("Please enter a path direction.".into());
    }
    Ok(())
}

fn format_distances(distances: &[f64]) -> String {
    let parts: Vec<String> = distances.iter().map(|&d| short(d)).collect();
    parts.join(", ")
}

/// Distances separated by spaces or semicolons; a comma right after a number separates
/// as well, a comma inside one is a decimal comma ("2, 6" are two, "2,5" is one).
fn parse_distances(text: &str) -> Vec<f64> {
    text.split(|c: char| c == ';' || c.is_whitespace())
        .map(|part| part.trim_end_matches(','))
        .filter(|part| !part.is_empty())
        .filter_map(numeric::parse_number)
        .collect()
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;

    #[test]
    fn definitions_are_checked() {
        let mut dialog = HotSpotDialog::create(&[]);
        assert_eq!(dialog.hot_spot.name, "Hot_Spot-1");
        assert!(dialog.output().is_err(), "no toe nodes");
        dialog.toe.take(
            &plx_mesh::FeMesh::default(),
            crate::selection::Items::Nodes(BTreeSet::from([7])),
            Operation::Replace,
        );
        dialog.hot_spot.extrapolation = Extrapolation::Custom(vec![3.0, 3.0]);
        assert!(dialog.output().is_err(), "equal distances");
        dialog.hot_spot.extrapolation = Extrapolation::IiwTypeBCoarse;
        let created = dialog.output().unwrap();
        assert_eq!(created.toe, plx_model::Region::Nodes(vec![7]));
        // OK - New keeps the settings for the next weld.
        dialog.next(&created);
        assert_eq!(dialog.hot_spot.name, "Hot_Spot-2");
        assert_eq!(dialog.hot_spot.extrapolation, Extrapolation::IiwTypeBCoarse);
        assert!(dialog.toe.is_empty());
        let edit = HotSpotDialog::edit(&[created], 0, &plx_mesh::FeMesh::default()).unwrap();
        assert!(edit.output().is_ok(), "its own name is no duplicate");
    }

    #[test]
    fn distances_accept_lists_and_decimal_commas() {
        assert_eq!(parse_distances("2, 6, 10"), [2.0, 6.0, 10.0]);
        assert_eq!(parse_distances("0,4; 1,5"), [0.4, 1.5]);
        assert_eq!(parse_distances("4 8 x 12"), [4.0, 8.0, 12.0]);
        assert_eq!(format_distances(&[0.4 * 12.0, 12.0]), "4.8, 12");
    }
}
