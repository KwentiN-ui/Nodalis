//! Editing a part filter: the terms it includes and excludes as small buttons that remove
//! them, a menu to add parts by name, name pattern, material or part group, and how many
//! parts it matches. Built for models of hundreds of parts, where a list of checkboxes no
//! longer works.

use std::sync::Arc;

use egui::{RichText, Ui};
use plx_mesh::FeMesh;
use plx_model::{FeModel, PartContext, PartFilter, PartTerm};

/// The parts of `mesh` as filters see them, worked out again only when the parts, the
/// sections or the part groups change.
pub fn cached_context(ctx: &egui::Context, fe: &FeModel, mesh: &FeMesh) -> Arc<PartContext> {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    mesh.elements().len().hash(&mut hasher);
    for part in &mesh.parts {
        part.name.hash(&mut hasher);
        part.elements.len().hash(&mut hasher);
    }
    for section in &fe.sections {
        section.material.hash(&mut hasher);
        format!("{:?}", std::mem::discriminant(&section.region)).hash(&mut hasher);
        if let plx_model::Region::Parts(names) = &section.region {
            names.hash(&mut hasher);
        }
    }
    format!("{:?}", fe.part_groups).hash(&mut hasher);
    let key = hasher.finish();
    let id = egui::Id::new(("part filter context", mesh as *const FeMesh as usize));
    let cached: Option<(u64, Arc<PartContext>)> = ctx.data(|d| d.get_temp(id));
    if let Some((k, context)) = cached
        && k == key
    {
        return context;
    }
    let context = Arc::new(fe.part_context(mesh));
    ctx.data_mut(|d| d.insert_temp(id, (key, context.clone())));
    context
}

/// Edits `filter` in one wrapped row: the terms it includes, `all` when it includes none,
/// then "except" and the terms it excludes. Below, how many parts of `context` it matches,
/// with their names on hover. Returns whether it changed.
pub fn filter_editor(
    ui: &mut Ui,
    id: &str,
    fe: &FeModel,
    filter: &mut PartFilter,
    context: &PartContext,
    all: &str,
) -> bool {
    let mut changed = false;
    ui.vertical(|ui| {
        ui.horizontal_wrapped(|ui| {
            changed |= terms(ui, fe, &mut filter.include);
            if filter.include.is_empty() {
                ui.label(all);
            }
            changed |= add_menu(
                ui,
                &format!("{id} include"),
                "+",
                fe,
                context,
                &mut filter.include,
            );
            ui.weak("except");
            changed |= terms(ui, fe, &mut filter.exclude);
            changed |= add_menu(
                ui,
                &format!("{id} exclude"),
                "+",
                fe,
                context,
                &mut filter.exclude,
            );
        });
        let matched: Vec<&str> = (0..context.len())
            .filter(|&p| filter.matches(p, context))
            .map(|p| context.names[p].as_str())
            .collect();
        let mut names = matched
            .iter()
            .take(40)
            .copied()
            .collect::<Vec<_>>()
            .join("\n");
        if matched.len() > 40 {
            names += &format!("\n... and {} more", matched.len() - 40);
        }
        ui.label(
            RichText::new(format!(
                "Matches {} of {} parts",
                matched.len(),
                context.len()
            ))
            .weak(),
        )
        .on_hover_text(if names.is_empty() {
            "None".into()
        } else {
            names
        });
    });
    changed
}

/// The terms as small buttons; clicking one removes it.
fn terms(ui: &mut Ui, fe: &FeModel, terms: &mut Vec<PartTerm>) -> bool {
    let mut remove = None;
    for (i, term) in terms.iter().enumerate() {
        if ui
            .add(egui::Button::new(term.describe(fe)).small())
            .on_hover_text("Click to remove.")
            .clicked()
        {
            remove = Some(i);
        }
    }
    if let Some(i) = remove {
        terms.remove(i);
    }
    remove.is_some()
}

/// The menu adding a term: a name pattern, a part from a searchable list, a material or a
/// part group.
fn add_menu(
    ui: &mut Ui,
    id: &str,
    label: &str,
    fe: &FeModel,
    context: &PartContext,
    terms: &mut Vec<PartTerm>,
) -> bool {
    let mut added = None;
    let text_id = ui.id().with((id, "pattern"));
    let mut picked_added = false;
    ui.menu_button(label, |ui| {
        ui.set_min_width(220.0);
        ui.label("Name pattern")
            .on_hover_text("* stands for any text, ? for one character; without them, names containing the text match. Case is ignored.");
        let mut text: String = ui.data(|d| d.get_temp(text_id)).unwrap_or_default();
        let response = ui.add(egui::TextEdit::singleline(&mut text).hint_text("e.g. Bolt*"));
        ui.data_mut(|d| d.insert_temp(text_id, text.clone()));
        let pattern = text.trim();
        if !pattern.is_empty() {
            let matching: Vec<&str> = (context.names.iter())
                .filter(|n| plx_model::name_matches(pattern, n))
                .map(|n| n.as_str())
                .collect();
            ui.weak(format!("{} parts match", matching.len()));
            let enter = response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if ui.button("Add Pattern").clicked() || enter {
                added = Some(PartTerm::Pattern(pattern.to_string()));
            }
            ui.separator();
            ui.weak("Or one of them:");
            egui::ScrollArea::vertical()
                .max_height(220.0)
                .id_salt((id, "parts"))
                .show(ui, |ui| {
                    for name in matching.iter().take(500) {
                        if ui.selectable_label(false, *name).clicked() {
                            added = Some(PartTerm::Part(name.to_string()));
                        }
                    }
                });
        }
        ui.separator();
        let picked = selected_part_names(ui.ctx());
        if ui
            .add_enabled(
                !picked.is_empty(),
                egui::Button::new(format!("Selected Parts ({})", picked.len())),
            )
            .on_hover_text(
                "Adds the parts selected in the 3D view or the FE Model tree; Ctrl-click \
                 parts there to select several.",
            )
            .clicked()
        {
            let mut changed = false;
            for name in picked {
                let term = PartTerm::Part(name);
                if !terms.contains(&term) {
                    terms.push(term);
                    changed = true;
                }
            }
            ui.close();
            picked_added = changed;
            return;
        }
        ui.menu_button("Material", |ui| {
            if fe.materials.is_empty() {
                ui.weak("No materials");
            }
            for material in &fe.materials {
                if ui.button(&material.name).clicked() {
                    added = Some(PartTerm::Material(material.id));
                }
            }
        });
        ui.menu_button("Part Group", |ui| {
            if fe.part_groups.is_empty() {
                ui.weak("No part groups; create them under Connections.");
            }
            for group in &fe.part_groups {
                if ui.button(&group.name).clicked() {
                    added = Some(PartTerm::Group(group.id));
                }
            }
        });
        if added.is_some() {
            ui.data_mut(|d| d.insert_temp(text_id, String::new()));
            ui.close();
        }
    });
    match added {
        Some(term) if !terms.contains(&term) => {
            terms.push(term);
            true
        }
        _ => picked_added,
    }
}

fn selected_parts_id() -> egui::Id {
    egui::Id::new("part filter selected parts")
}

/// Tells the filter menus the names of the parts selected in the 3D view or the tree.
pub fn set_selected_part_names(ctx: &egui::Context, names: Vec<String>) {
    ctx.data_mut(|d| d.insert_temp(selected_parts_id(), names));
}

fn selected_part_names(ctx: &egui::Context) -> Vec<String> {
    ctx.data(|d| d.get_temp(selected_parts_id()))
        .unwrap_or_default()
}
