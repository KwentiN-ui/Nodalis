//! The contact matrix: the contact regions between part groups, one cell per pair of
//! groups with the number of regions of each type. On a model of hundreds of parts it says
//! at a glance which groups touch and how, where the worksheet lists every region. A cell
//! selects its regions; its menu changes their type at once.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use egui::{RichText, Ui};
use plx_model::ContactBehavior;

use crate::connections::{Change, change_regions};
use crate::model::Model;
use crate::tree::{TreeItem, TreeState, TreeView};

/// Of each contact region, the parts of its two sides, worked out again when the regions or
/// the mesh change.
fn region_parts(
    ctx: &egui::Context,
    model: &Model,
) -> Arc<Vec<(BTreeSet<usize>, BTreeSet<usize>)>> {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    model.mesh.elements().len().hash(&mut hasher);
    model.mesh.parts.len().hash(&mut hasher);
    for r in &model.fe.contact_regions {
        r.id.hash(&mut hasher);
        format!("{:?}{:?}", r.side_a, r.side_b)
            .len()
            .hash(&mut hasher);
    }
    let key = hasher.finish();
    let id = egui::Id::new("contact matrix region parts");
    type Cached = (u64, Arc<Vec<(BTreeSet<usize>, BTreeSet<usize>)>>);
    if let Some((k, parts)) = ctx.data(|d| d.get_temp::<Cached>(id))
        && k == key
    {
        return parts;
    }
    let parts: Arc<Vec<_>> = Arc::new(
        (model.fe.contact_regions.iter())
            .map(|r| r.parts(&model.mesh))
            .collect(),
    );
    ctx.data_mut(|d| d.insert_temp(id, (key, parts.clone())));
    parts
}

/// The rows and columns of the matrix: the part groups, then "Other" for the parts in
/// none; without groups, the parts themselves. Each part belongs to the first group it
/// matches.
struct Axes {
    names: Vec<String>,
    /// Of each part, its row.
    row_of: Vec<usize>,
    by_groups: bool,
}

fn axes(ctx: &egui::Context, model: &Model) -> Axes {
    let fe = &model.fe;
    let context = crate::part_filter_ui::cached_context(ctx, fe, &model.mesh);
    if fe.part_groups.is_empty() {
        return Axes {
            names: context.names.clone(),
            row_of: (0..context.len()).collect(),
            by_groups: false,
        };
    }
    let mut names: Vec<String> = fe.part_groups.iter().map(|g| g.name.clone()).collect();
    let other = names.len();
    let row_of: Vec<usize> = (0..context.len())
        .map(|p| {
            (fe.part_groups.iter())
                .position(|g| g.filter.matches(p, &context))
                .unwrap_or(other)
        })
        .collect();
    if row_of.contains(&other) {
        names.push("Other".into());
    }
    Axes {
        names,
        row_of,
        by_groups: true,
    }
}

/// One cell: the regions between two rows by type.
#[derive(Default)]
struct Cell {
    regions: Vec<usize>,
    by_type: BTreeMap<&'static str, usize>,
}

/// Shows the matrix window. Returns true when a cell asks for the worksheet.
pub fn window(
    ctx: &egui::Context,
    open: &mut bool,
    model: &mut Model,
    tree: &mut TreeState,
) -> bool {
    let axes = axes(ctx, model);
    let parts = region_parts(ctx, model);
    let n = axes.names.len();
    let mut cells: BTreeMap<(usize, usize), Cell> = BTreeMap::new();
    let mut unknown = 0;
    for (i, region) in model.fe.contact_regions.iter().enumerate() {
        let Some((a, b)) = parts.get(i) else {
            continue;
        };
        let rows = |parts: &BTreeSet<usize>| -> BTreeSet<usize> {
            parts
                .iter()
                .filter_map(|&p| axes.row_of.get(p).copied())
                .collect()
        };
        let (ra, rb) = (rows(a), rows(b));
        // A side spanning several rows counts in the first of them.
        let (Some(&ra), Some(&rb)) = (ra.first(), rb.first()) else {
            unknown += 1;
            continue;
        };
        let key = (ra.min(rb), ra.max(rb));
        let cell = cells.entry(key).or_default();
        cell.regions.push(i);
        *cell.by_type.entry(region.behavior.name()).or_default() += 1;
    }
    let mut worksheet = false;
    let mut change: Option<(Vec<usize>, Change)> = None;
    let mut close = false;
    egui::Window::new("Contact Matrix")
        .id(crate::ids::window(ctx, "contact matrix"))
        .open(open)
        .collapsible(false)
        .default_width(640.0)
        .resizable(true)
        .show(ctx, |ui| {
            if axes.by_groups {
                ui.weak(
                    "Contact regions between part groups by type. Each part counts in the first \
                     group it matches. Click a cell to select its regions and open the \
                     worksheet; right-click to change them.",
                );
            } else {
                ui.weak(
                    "No part groups: rows are the parts. Create part groups (Part Group on the \
                     Connections tab) to see how groups of parts touch.",
                );
            }
            if unknown > 0 {
                ui.weak(format!(
                    "{unknown} regions have no parts in the mesh yet (found on the geometry \
                     before meshing)."
                ));
            }
            close = crate::dialog::close_bar(ui, "contact matrix");
            egui::ScrollArea::both()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    egui::Grid::new("contact matrix grid")
                        .striped(true)
                        .num_columns(n + 1)
                        .spacing([10.0, 4.0])
                        .show(ui, |ui| {
                            ui.label("");
                            for name in &axes.names {
                                ui.strong(name);
                            }
                            ui.end_row();
                            for r in 0..n {
                                ui.strong(&axes.names[r]);
                                for c in 0..n {
                                    if c < r {
                                        ui.label("");
                                        continue;
                                    }
                                    match cells.get(&(r, c)) {
                                        Some(cell) => cell_ui(
                                            ui,
                                            model,
                                            cell,
                                            tree,
                                            &mut worksheet,
                                            &mut change,
                                        ),
                                        None => {
                                            ui.weak("-");
                                        }
                                    }
                                }
                                ui.end_row();
                            }
                        });
                });
        });
    if close {
        *open = false;
    }
    if let Some((regions, change)) = change {
        change_regions(model, regions, change, tree);
    }
    worksheet
}

fn cell_ui(
    ui: &mut Ui,
    model: &Model,
    cell: &Cell,
    tree: &mut TreeState,
    worksheet: &mut bool,
    change: &mut Option<(Vec<usize>, Change)>,
) {
    let text = if cell.by_type.len() == 1 {
        let (name, count) = cell.by_type.iter().next().unwrap();
        format!("{name} {count}")
    } else {
        format!("Mixed {}", cell.regions.len())
    };
    let hover = (cell.by_type.iter())
        .map(|(name, count)| format!("{count} {name}"))
        .collect::<Vec<_>>()
        .join("\n");
    let items: Vec<TreeItem> = cell
        .regions
        .iter()
        .map(|&i| TreeItem::ContactRegion(i))
        .collect();
    let selected = {
        let current = tree.selected_items(TreeView::FeModel);
        !items.is_empty() && items.iter().all(|i| current.contains(i))
    };
    let response = ui
        .selectable_label(selected, RichText::new(text))
        .on_hover_text(hover);
    if response.clicked() {
        tree.select_items(TreeView::FeModel, items.clone());
        *worksheet = true;
    }
    response.context_menu(|ui| {
        ui.weak(format!("{} regions", cell.regions.len()));
        ui.menu_button("Set Type", |ui| {
            for behavior in ContactBehavior::ALL {
                if ui.button(behavior.name()).clicked() {
                    *change = Some((cell.regions.clone(), Change::Behavior(behavior)));
                    ui.close();
                }
            }
        });
        if ui.button("Activate").clicked() {
            *change = Some((cell.regions.clone(), Change::Active(true)));
            ui.close();
        }
        if ui.button("Deactivate").clicked() {
            *change = Some((cell.regions.clone(), Change::Active(false)));
            ui.close();
        }
        if ui.button("Select").clicked() {
            tree.select_items(TreeView::FeModel, items.clone());
            ui.close();
        }
    });
    let _ = model;
}
