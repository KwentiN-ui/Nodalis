//! The rows of the load dialog for the loads on the walls of a Particle Flow step
//! ([`plx_model::WallLoads`]): which walls, which time of the run, and what reaches the
//! faces of the region.

use std::cell::RefCell;
use std::collections::HashMap;

use egui::Ui;
use plx_model::units::{Quantity, UnitSystem};
use plx_model::{Region, WallLoadTime, WallLoads};

use crate::model::Model;
use crate::numeric;

thread_local! {
    /// What the loads give the faces, by what they were worked out of: reading the run and
    /// spreading its points takes a while.
    static SUMMARIES: RefCell<HashMap<String, Result<Summary, String>>> = RefCell::default();
}

/// What a wall load gives the faces of its region.
#[derive(Clone, Debug)]
struct Summary {
    /// The walls of the run.
    walls: Vec<String>,
    end_time: f64,
    frames: usize,
    total: [f64; 3],
    mapped: [f64; 3],
    nodes: usize,
    points: usize,
    missed: usize,
}

fn summary(model: &Model, region: &Region, load: &WallLoads) -> Result<Summary, String> {
    let file = load
        .file()
        .ok_or_else(|| "Link a Particle Flow step to the Wall loads of the step.".to_string())?;
    let modified = std::fs::metadata(file).and_then(|m| m.modified()).ok();
    let key = format!("{file:?}{modified:?}{region:?}{load:?}");
    if let Some(found) = SUMMARIES.with(|s| s.borrow().get(&key).cloned()) {
        return found;
    }
    let made = (|| {
        let loads = plx_io::wall_loads::read(file)?;
        let mapped = plx_io::wall_loads::nodal_forces(model.mesh(), region, load, &loads);
        let mut summary = Summary {
            walls: loads.walls.iter().map(|w| w.name.clone()).collect(),
            end_time: loads.end_time(),
            frames: loads.times.len(),
            total: [0.0; 3],
            mapped: [0.0; 3],
            nodes: 0,
            points: 0,
            missed: 0,
        };
        let sets = mapped?;
        let count = sets.len().max(1) as f64;
        let mut nodes = std::collections::BTreeSet::new();
        for (_, m) in &sets {
            for a in 0..3 {
                summary.total[a] += m.total[a] / count;
                summary.mapped[a] += m.mapped[a] / count;
            }
            nodes.extend(m.forces.keys().copied());
            summary.points += m.points;
            summary.missed += m.missed;
        }
        summary.nodes = nodes.len();
        Ok(summary)
    })();
    SUMMARIES.with(|s| {
        let mut s = s.borrow_mut();
        if s.len() > 32 {
            s.clear();
        }
        s.insert(key, made.clone());
    });
    made
}

/// The rows of `load`, on the faces of `region`.
pub fn rows(ui: &mut Ui, load: &mut WallLoads, region: &Region, model: &Model, units: UnitSystem) {
    let found = summary(model, region, load);
    // The walls: all, or those ticked.
    if let Ok(summary) = &found
        && summary.walls.len() > 1
    {
        ui.label("Walls");
        ui.vertical(|ui| {
            for name in &summary.walls {
                let mut on = load.takes(name);
                if ui.checkbox(&mut on, name).changed() {
                    let mut taken: Vec<String> = (summary.walls.iter())
                        .filter(|w| load.takes(w))
                        .cloned()
                        .collect();
                    if on {
                        taken.push(name.clone());
                    } else {
                        taken.retain(|w| w != name);
                    }
                    load.walls = if taken.len() == summary.walls.len() {
                        Vec::new()
                    } else {
                        taken
                    };
                }
            }
        });
        ui.end_row();
    }
    let end = found.as_ref().map_or(1.0, |s| s.end_time);
    ui.label("Time of the run");
    let mut index = load.time.index();
    egui::ComboBox::from_id_salt("wall load time")
        .selected_text(WallLoadTime::LABELS[index])
        .show_ui(ui, |ui| {
            for (k, label) in WallLoadTime::LABELS.iter().enumerate() {
                ui.selectable_value(&mut index, k, *label);
            }
        });
    ui.end_row();
    if index != load.time.index() {
        load.time = match index {
            0 => WallLoadTime::Last,
            1 => WallLoadTime::At(end),
            2 => WallLoadTime::Mean {
                from: 0.5 * end,
                to: None,
            },
            _ => WallLoadTime::OverTime,
        };
    }
    match &mut load.time {
        WallLoadTime::Last => {
            ui.label("");
            ui.weak("The last frame: the mean load since the frame before it.");
            ui.end_row();
        }
        WallLoadTime::At(time) => {
            ui.label("Time");
            ui.add(numeric::physical(time, units, Quantity::Time));
            ui.end_row();
            ui.label("");
            ui.weak("The frame nearest the time: the mean load since the frame before it.");
            ui.end_row();
        }
        WallLoadTime::Mean { from, to } => {
            ui.label("From");
            ui.add(numeric::physical(from, units, Quantity::Time));
            ui.end_row();
            ui.label("To");
            ui.horizontal(|ui| {
                let mut to_end = to.is_none();
                if ui.checkbox(&mut to_end, "End").changed() {
                    *to = (!to_end).then_some(end);
                }
                if let Some(to) = to {
                    ui.add(numeric::physical(to, units, Quantity::Time));
                }
            });
            ui.end_row();
            ui.label("");
            ui.weak("The mean over the frames between the times: a steady flow without the noise of single frames.");
            ui.end_row();
        }
        WallLoadTime::OverTime => {
            ui.label("");
            ui.weak(
                "Each frame acts at its time of the run as time of the step, linear in \
                 between; after the last frame its load stays.",
            );
            ui.end_row();
        }
    }
    match &found {
        Ok(summary) => {
            let unit = units.unit(Quantity::Force);
            let shown = |f: [f64; 3]| {
                let f = f.map(|v| units.shown(v, Quantity::Force));
                format!(
                    "{}, {}, {} {unit}",
                    crate::results::format_value(f[0] as f32),
                    crate::results::format_value(f[1] as f32),
                    crate::results::format_value(f[2] as f32)
                )
            };
            ui.label("Run");
            ui.weak(format!(
                "{} frames to time {}",
                summary.frames,
                crate::results::format_value(units.shown(summary.end_time, Quantity::Time) as f32)
            ));
            ui.end_row();
            ui.label("Force of the walls");
            ui.weak(shown(summary.total));
            ui.end_row();
            ui.label("Force on the faces");
            ui.weak(shown(summary.mapped));
            ui.end_row();
            ui.label("");
            ui.weak(format!("On {} nodes of the region.", summary.nodes));
            ui.end_row();
            if summary.missed > 0 {
                ui.label("");
                ui.colored_label(
                    ui.visuals().warn_fg_color,
                    format!(
                        "{} of {} points of the walls lie near no face of the region; their \
                         force is left out. Select the faces of the tool parts.",
                        summary.missed, summary.points
                    ),
                );
                ui.end_row();
            }
        }
        Err(e) => {
            ui.label("");
            ui.colored_label(ui.visuals().error_fg_color, e);
            ui.end_row();
        }
    }
}
