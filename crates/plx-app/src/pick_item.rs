//! Combo boxes that choose an item of the model by name and keep its id, so that the choice
//! follows the item when it is renamed.

use egui::Ui;
use plx_model::{FeModel, GLOBAL, Id};

/// Chooses among `items`, given by id and name; `none` names the choice of no item, which
/// the list leaves out without it. Returns whether the choice changed.
pub fn item_combo<'a>(
    ui: &mut Ui,
    salt: impl std::hash::Hash + std::fmt::Debug,
    chosen: &mut Option<Id>,
    items: impl IntoIterator<Item = (Id, &'a str)>,
    none: Option<&str>,
    width: f32,
) -> bool {
    let items: Vec<(Id, &str)> = items.into_iter().collect();
    let shown = match *chosen {
        None => none.unwrap_or_default().to_string(),
        Some(id) => (items.iter().find(|(i, _)| *i == id))
            .map_or_else(|| "(missing)".to_string(), |(_, name)| name.to_string()),
    };
    let before = *chosen;
    egui::ComboBox::from_id_salt(salt)
        .selected_text(shown)
        .width(crate::dock::fit_width(ui, width))
        .show_ui(ui, |ui| {
            if let Some(none) = none {
                ui.selectable_value(chosen, None, none);
            }
            for (id, name) in items {
                ui.selectable_value(chosen, Some(id), name);
            }
        });
    *chosen != before
}

/// Chooses a coordinate system of the features or the global one (`None`).
pub fn system_combo(
    ui: &mut Ui,
    salt: impl std::hash::Hash + std::fmt::Debug,
    system: &mut Option<Id>,
    fe: &FeModel,
    width: f32,
) -> bool {
    // Systems that move with a selection are marked.
    let labels: Vec<(Id, String)> = (fe.coordinate_systems.iter())
        .map(|c| match c.moves(&fe.coordinate_systems) {
            true => (c.id, format!("{} (moving)", c.name)),
            false => (c.id, c.name.clone()),
        })
        .collect();
    let systems = labels.iter().map(|(id, name)| (*id, name.as_str()));
    item_combo(ui, salt, system, systems, Some(GLOBAL), width)
}
