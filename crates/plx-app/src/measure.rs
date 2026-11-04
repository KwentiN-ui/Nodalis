//! Computing the key figure of a Measure block from the results of a step, on a worker thread,
//! and the editor of a Measure block.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::mpsc::Receiver;

use plx_io::frd::FrdImport;
use plx_io::project::BoundResults;
use plx_mesh::{NodeId, PointLocation};
use plx_model::{Extreme, MeasureKind, MeasureRegion, UnitSystem};
use plx_results::AnalysisKind;

/// The value of `kind` in the results of step `step` (CalculiX's numbering, from 1): of its
/// last increment, or of a mode. `node_sets` are those of the mesh the results belong to.
#[cfg(test)]
pub fn evaluate(
    kind: &MeasureKind,
    step: u32,
    frd: &FrdImport,
    dat: Option<&plx_io::dat::DatImport>,
    node_sets: &BTreeMap<String, Vec<NodeId>>,
    units: UnitSystem,
) -> Result<f64, String> {
    evaluate_expanded(kind, step, frd, dat, node_sets, &BTreeMap::new(), units)
}

/// The value of `kind` in the results of step `step` (CalculiX's numbering, from 1), as
/// [`evaluate`] gives it, on results in which the solver expanded shells and beams: `expanded` gives the
/// nodes of the results each of their nodes became (see [`plx_io::frd::expanded_nodes`]). A
/// region takes the values of all of them, so its extremes are those on the faces of the
/// shells; a node takes their mean, or their sum for forces.
pub fn evaluate_expanded(
    kind: &MeasureKind,
    step: u32,
    frd: &FrdImport,
    dat: Option<&plx_io::dat::DatImport>,
    node_sets: &BTreeMap<String, Vec<NodeId>>,
    expanded: &BTreeMap<NodeId, Vec<NodeId>>,
    units: UnitSystem,
) -> Result<f64, String> {
    let indices = |node: NodeId| plx_io::frd::node_indices(&frd.mesh, expanded, node);
    let increments: Vec<_> = frd.increments.iter().filter(|i| i.step == step).collect();
    let component = |field: &str, component: &str| {
        let increment = (increments.iter().rev())
            .find(|i| i.field(field).is_some())
            .ok_or_else(|| format!("step {step} has no field {field} in the results"))?;
        let values = (increment.field(field).and_then(|f| f.component(component)))
            .ok_or_else(|| format!("{field} has no component {component}"))?;
        Ok::<_, String>(&values.values)
    };
    match kind {
        MeasureKind::FieldExtreme {
            field,
            component: name,
            extreme,
            region,
        } => {
            let values = component(field, name)?;
            let picked: Vec<f64> = match region {
                MeasureRegion::All => values.iter().map(|&v| f64::from(v)).collect(),
                MeasureRegion::NodeSet(set) => {
                    let nodes = (node_sets.get(set))
                        .ok_or_else(|| format!("the mesh has no node set {set}"))?;
                    (nodes.iter())
                        .flat_map(|&id| indices(id))
                        .filter_map(|index| values.get(index))
                        .map(|&v| f64::from(v))
                        .collect()
                }
            };
            extreme
                .of(picked)
                .ok_or_else(|| "no values in the region".to_owned())
        }
        MeasureKind::NodeValue {
            field,
            component: name,
            node,
        } => {
            let values = component(field, name)?;
            let at = indices(*node);
            if at.is_empty() {
                return Err(format!("the results have no node {node}"));
            }
            let picked = (at.iter())
                .map(|&index| values.get(index).map(|&v| f64::from(v)))
                .collect::<Option<Vec<f64>>>()
                .ok_or_else(|| format!("no value at node {node}"))?;
            let sum: f64 = picked.iter().sum();
            // Forces of the expanded nodes add up to the force at the node.
            Ok(if matches!(field.as_str(), "FORC" | "RF") {
                sum
            } else {
                sum / picked.len() as f64
            })
        }
        MeasureKind::PointValue {
            field,
            component: name,
            point,
        } => {
            let values = component(field, name)?;
            match frd.mesh.locate(*point) {
                PointLocation::Found { weights, .. } => {
                    let value: f64 = (weights.iter())
                        .map(|&(index, w)| {
                            values.get(index).map_or(f64::NAN, |&v| f64::from(v)) * w
                        })
                        .sum();
                    if value.is_finite() {
                        Ok(value)
                    } else {
                        Err(format!(
                            "{field} has no value at a node of the element at the point"
                        ))
                    }
                }
                PointLocation::Outside { distance } => Err(format!(
                    "the point lies outside the mesh, {} from its nearest node",
                    length(distance, units)
                )),
                PointLocation::Empty => Err("the results have no solid or plane elements".into()),
            }
        }
        MeasureKind::History {
            set,
            field,
            component,
        } => {
            let dat = dat.ok_or("the run printed no history output (.dat)")?;
            let history = (dat.sets.iter())
                .find(|s| s.name.eq_ignore_ascii_case(set))
                .ok_or_else(|| format!("no history output of {set}"))?;
            let row = (history.rows.iter())
                .rposition(|(s, _, _)| *s == step)
                .ok_or_else(|| format!("{set} has nothing of step {step}"))?;
            let entry = (history.field(field).and_then(|f| f.component(component)))
                .and_then(|c| c.entries.first())
                .ok_or_else(|| format!("{set} has no {field} {component}"))?;
            entry
                .values
                .get(row)
                .copied()
                .ok_or_else(|| "the history output ends early".to_owned())
        }
        MeasureKind::Eigenfrequency { mode } => {
            let modes: Vec<f64> = (increments.iter())
                .filter(|i| i.kind == AnalysisKind::Frequency)
                .map(|i| i.value)
                .collect();
            if modes.is_empty() {
                return Err(format!("step {step} is no frequency step"));
            }
            (mode.checked_sub(1))
                .and_then(|i| modes.get(i as usize))
                .copied()
                .ok_or_else(|| format!("the step has {} modes, not {mode}", modes.len()))
        }
    }
}

/// A length in SI units as a message shows it, in `units`.
fn length(metres: f64, units: UnitSystem) -> String {
    let shown = units.shown(metres, plx_model::Quantity::Length);
    let unit = units.unit(plx_model::Quantity::Length);
    format!("{} {unit}", crate::results::format_value(shown as f32))
}

/// What a worker thread needs to compute a measure.
pub struct Job {
    pub kind: MeasureKind,
    pub step: u32,
    pub results: BoundResults,
    pub node_sets: BTreeMap<String, Vec<NodeId>>,
    /// The units its messages give lengths in.
    pub units: UnitSystem,
    /// Where the files of CalculiX's results are written to be read; those of the built-in
    /// solver are read in memory.
    pub dir: PathBuf,
}

/// Computes the measure on a worker thread; the receiver gets the value or why there is none.
pub fn start(job: Job) -> Receiver<Result<f64, String>> {
    let (sender, receiver) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let _ = sender.send(run(&job));
    });
    receiver
}

fn run(job: &Job) -> Result<f64, String> {
    // Results of the built-in solver are read in memory, CalculiX's from their files.
    let read = job.results.read(&job.dir)?;
    let frd = read.results;
    // The input the solver ran knows the shells and beams it expanded.
    let expanded = (read.input.as_deref())
        .and_then(|text| plx_io::inp::read_inp_str(text, None).ok())
        .map(|input| plx_io::frd::expanded_nodes(&input.mesh, &frd.mesh))
        .unwrap_or_default();
    evaluate_expanded(
        &job.kind,
        job.step,
        &frd,
        read.history.as_ref(),
        &job.node_sets,
        &expanded,
        job.units,
    )
}

/// Fields by name, each with the names of its components.
pub type Fields = Vec<(String, Vec<String>)>;

/// The fields and components the results of a step offer, to pick from in the editor.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Offered {
    /// Nodal fields of the step and their components, in the order of the results.
    pub fields: Fields,
    /// History output: the sets, each with its fields and their components.
    pub history: Vec<(String, Fields)>,
}

impl Offered {
    /// The fields of a history set, if the results have it.
    pub fn history_fields(&self, set: &str) -> Option<&[(String, Vec<String>)]> {
        (self.history.iter())
            .find(|(name, _)| name.eq_ignore_ascii_case(set))
            .map(|(_, fields)| fields.as_slice())
    }
}

/// What the results of step `step` offer to measure.
pub fn offered(step: u32, frd: &FrdImport, dat: Option<&plx_io::dat::DatImport>) -> Offered {
    let mut fields: Fields = Vec::new();
    for increment in frd.increments.iter().filter(|i| i.step == step) {
        for field in &increment.fields {
            let components = field.components.iter().map(|c| c.name.clone()).collect();
            match fields.iter_mut().find(|(name, _)| *name == field.name) {
                Some(entry) => entry.1 = components,
                None => fields.push((field.name.clone(), components)),
            }
        }
    }
    let history = (dat.iter().flat_map(|d| &d.sets))
        .filter(|set| set.rows.iter().any(|(s, _, _)| *s == step))
        .map(|set| {
            let fields = (set.fields.iter())
                .map(|f| {
                    let components = f.components.iter().map(|c| c.name.clone()).collect();
                    (f.name.clone(), components)
                })
                .collect();
            (set.name.clone(), fields)
        })
        .collect();
    Offered { fields, history }
}

/// Reads what the results offer to measure on a worker thread, into `dir`.
pub fn read_offered(
    results: BoundResults,
    step: u32,
    dir: PathBuf,
) -> Receiver<Result<Offered, String>> {
    let (sender, receiver) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let read = || {
            let read = results.read(&dir)?;
            Ok(offered(step, &read.results, read.history.as_ref()))
        };
        let _ = sender.send(read());
    });
    receiver
}

/// The kind of number the value is, for value links from the block to Math blocks and
/// parameters.
pub fn value_kind(kind: &MeasureKind) -> Option<plx_model::ValueKind> {
    match kind.field() {
        ("FREQ", _) => Some(plx_model::ValueKind::Quantity(
            plx_model::Quantity::Frequency,
        )),
        (field, component) => crate::results::field_value_kind(field, component),
    }
}

/// The value as the block shows it: in `units`, with its unit.
pub fn shown(kind: &MeasureKind, value: f64, units: UnitSystem) -> String {
    let (field, component) = kind.field();
    if field == "FREQ" {
        let value = units.shown(value, plx_model::Quantity::Frequency);
        let unit = units.unit(plx_model::Quantity::Frequency);
        return format!("{} {unit}", crate::results::format_value(value as f32))
            .trim()
            .to_owned();
    }
    let value = crate::results::shown_field_value(value, field, component, units);
    let unit = crate::results::field_unit(field, component, units).unwrap_or_default();
    format!("{} {unit}", crate::results::format_value(value as f32))
        .trim()
        .to_owned()
}

pub enum MeasureEditorResult {
    Open,
    Ok(MeasureKind, UnitSystem),
    Cancel,
}

/// The editor of a Measure block: what it computes.
pub struct MeasureEditor {
    pub id: u64,
    name: String,
    kind: MeasureKind,
    units: UnitSystem,
    /// The node sets of the mesh of the results linked to it, to pick from.
    node_sets: Vec<String>,
    /// The fields and components of the results linked to it, to pick from.
    choices: Choices,
}

/// What the editor offers for field and component: free text without linked results, the
/// names in the results once they are read.
enum Choices {
    Typed,
    Reading(Receiver<Result<Offered, String>>),
    Read(Offered),
    Failed(String),
}

impl MeasureEditor {
    pub fn new(
        id: u64,
        name: &str,
        kind: MeasureKind,
        units: UnitSystem,
        node_sets: Vec<String>,
        offered: Option<Receiver<Result<Offered, String>>>,
    ) -> Self {
        Self {
            id,
            name: name.to_owned(),
            kind,
            units,
            node_sets,
            choices: offered.map_or(Choices::Typed, Choices::Reading),
        }
    }

    pub fn rename(&mut self, name: &str) {
        name.clone_into(&mut self.name);
    }

    pub fn show(&mut self, ctx: &egui::Context) -> MeasureEditorResult {
        if let Choices::Reading(receiver) = &self.choices {
            match receiver.try_recv() {
                Ok(Ok(offered)) => self.choices = Choices::Read(offered),
                Ok(Err(why)) => self.choices = Choices::Failed(why),
                Err(std::sync::mpsc::TryRecvError::Empty) => ctx.request_repaint(),
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.choices = Choices::Failed("the reading stopped".into());
                }
            }
        }
        let mut result = MeasureEditorResult::Open;
        let mut open = true;
        egui::Window::new(format!("Measure - {}", self.name))
            .id(crate::ids::window(ctx, ("measure", self.id)))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                egui::Grid::new("measure form")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| self.form(ui));
                if matches!(self.kind, MeasureKind::PointValue { .. }) {
                    ui.weak(
                        "The point is given by the X, Y and Z rows of the block: type the \
                         coordinates there, or link numbers to them, such as a parameter of a \
                         study. The field is interpolated with the shape functions of the \
                         element that contains the point. A point just outside the mesh, \
                         within 5 % of the size of the nearest element, takes the value at the \
                         nearest point of that element; farther out the block has no value.",
                    );
                }
                ui.weak(match &self.choices {
                    Choices::Read(_) => {
                        "Fields and components as the linked results have them. The value is \
                         that of the last increment of the step whose results are linked to \
                         the block."
                            .to_owned()
                    }
                    Choices::Reading(_) => "Reading the fields of the linked results...".into(),
                    Choices::Typed => {
                        "Field and component as the Results workspace names them, such as \
                         STRESS MISES, DISP ALL or NDTEMP T. Link the results of a step to the \
                         block to pick them from a list."
                            .to_owned()
                    }
                    Choices::Failed(why) => format!("The linked results were not read: {why}"),
                });
                ui.separator();
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if ui.button("Cancel").clicked() {
                        result = MeasureEditorResult::Cancel;
                    }
                    if ui.button("OK").clicked() {
                        result = MeasureEditorResult::Ok(self.kind.clone(), self.units);
                    }
                });
            });
        if !open {
            result = MeasureEditorResult::Cancel;
        }
        result
    }

    fn form(&mut self, ui: &mut egui::Ui) {
        ui.label("Measure");
        let current = self.kind.label();
        egui::ComboBox::from_id_salt("measure kind")
            .selected_text(current)
            .width(200.0)
            .show_ui(ui, |ui| {
                for choice in MeasureKind::choices() {
                    let label = choice.label();
                    if ui.selectable_label(label == current, label).clicked() && label != current {
                        self.kind = choice;
                    }
                }
            });
        ui.end_row();
        ui.label("Shown in");
        egui::ComboBox::from_id_salt("measure units")
            .selected_text(self.units.label())
            .width(200.0)
            .show_ui(ui, |ui| {
                for choice in UnitSystem::ALL {
                    ui.selectable_value(&mut self.units, choice, choice.label());
                }
            });
        ui.end_row();
        let offered = match &self.choices {
            Choices::Read(offered) => Some(offered),
            _ => None,
        };
        match &mut self.kind {
            MeasureKind::FieldExtreme {
                field,
                component,
                extreme,
                region,
            } => {
                field_and_component(ui, offered.map(|o| o.fields.as_slice()), field, component);
                ui.label("Take");
                egui::ComboBox::from_id_salt("measure extreme")
                    .selected_text(extreme.label())
                    .width(200.0)
                    .show_ui(ui, |ui| {
                        for choice in Extreme::ALL {
                            ui.selectable_value(extreme, choice, choice.label());
                        }
                    });
                ui.end_row();
                ui.label("Region");
                let shown = match region {
                    MeasureRegion::All => "All nodes".to_owned(),
                    MeasureRegion::NodeSet(set) => set.clone(),
                };
                egui::ComboBox::from_id_salt("measure region")
                    .selected_text(shown)
                    .width(200.0)
                    .show_ui(ui, |ui| {
                        ui.selectable_value(region, MeasureRegion::All, "All nodes");
                        for set in &self.node_sets {
                            ui.selectable_value(region, MeasureRegion::NodeSet(set.clone()), set);
                        }
                    });
                ui.end_row();
            }
            MeasureKind::NodeValue {
                field,
                component,
                node,
            } => {
                field_and_component(ui, offered.map(|o| o.fields.as_slice()), field, component);
                ui.label("Node");
                ui.add(crate::numeric::count(node).range(1..=u32::MAX));
                ui.end_row();
            }
            MeasureKind::PointValue {
                field, component, ..
            } => {
                field_and_component(ui, offered.map(|o| o.fields.as_slice()), field, component);
            }
            MeasureKind::History {
                set,
                field,
                component,
            } => {
                let sets: Option<Vec<String>> =
                    offered.map(|o| o.history.iter().map(|(name, _)| name.clone()).collect());
                if pick(ui, "Set", set, sets.as_deref())
                    && let Some(fields) = offered.and_then(|o| o.history_fields(set))
                    && !fields.iter().any(|(name, _)| name == field)
                    && let Some((first, components)) = fields.first()
                {
                    first.clone_into(field);
                    components
                        .first()
                        .unwrap_or(&String::new())
                        .clone_into(component);
                }
                let fields = offered.map(|o| o.history_fields(set).unwrap_or_default());
                field_and_component(ui, fields, field, component);
            }
            MeasureKind::Eigenfrequency { mode } => {
                ui.label("Mode");
                ui.add(crate::numeric::count(mode).range(1..=10_000));
                ui.end_row();
            }
        }
    }
}

/// The Field and Component rows: lists of what `offered` has, or free text without it. A
/// field picked from the list keeps the component if it has one of that name, otherwise takes
/// its first.
fn field_and_component(
    ui: &mut egui::Ui,
    offered: Option<&[(String, Vec<String>)]>,
    field: &mut String,
    component: &mut String,
) {
    let fields: Option<Vec<String>> =
        offered.map(|list| list.iter().map(|(name, _)| name.clone()).collect());
    let components_of = |field: &str| -> Vec<String> {
        (offered.into_iter().flatten())
            .find(|(name, _)| name == field)
            .map(|(_, components)| components.clone())
            .unwrap_or_default()
    };
    if pick(ui, "Field", field, fields.as_deref()) {
        let components = components_of(field);
        if !components.contains(component)
            && let Some(first) = components.first()
        {
            first.clone_into(component);
        }
    }
    let components = offered.map(|_| components_of(field));
    pick(ui, "Component", component, components.as_deref());
}

/// A row with a list of `options` to pick `value` from, or a text field without options; true
/// when the value changed. A value the options lack is shown in the warning color.
fn pick(ui: &mut egui::Ui, label: &str, value: &mut String, options: Option<&[String]>) -> bool {
    ui.label(label);
    let changed = match options {
        None => (ui.add_sized([200.0, 18.0], egui::TextEdit::singleline(value))).changed(),
        Some(options) => {
            let known = options.contains(value);
            let shown = if value.is_empty() {
                egui::RichText::new("(none)")
            } else if known {
                egui::RichText::new(value.as_str())
            } else {
                egui::RichText::new(value.as_str()).color(ui.visuals().warn_fg_color)
            };
            let mut changed = false;
            let combo = egui::ComboBox::from_id_salt(("measure pick", label))
                .selected_text(shown)
                .width(200.0)
                .show_ui(ui, |ui| {
                    for option in options {
                        if ui.selectable_label(option == value, option).clicked() && option != value
                        {
                            option.clone_into(value);
                            changed = true;
                        }
                    }
                });
            if !known {
                combo.response.on_hover_text(if options.is_empty() {
                    "The linked results have nothing to pick here."
                } else {
                    "Not in the linked results."
                });
            }
            changed
        }
    };
    ui.end_row();
    changed
}

#[cfg(test)]
mod tests {
    use super::*;

    fn testdata(name: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name)
    }

    /// Results of the built-in solver kept in a project are measured in memory: nothing is
    /// written to the folder CalculiX's results are restored to.
    #[test]
    fn results_of_the_built_in_solver_are_measured_in_memory() {
        let dir = std::env::temp_dir().join(format!("plx-measure-ndlr-{}", std::process::id()));
        let file = crate::solver_check::solve_beam_built_in(&dir.join("run"));
        let files = [file.clone(), file.with_extension("inp")];
        let bound = plx_io::project::bind_results("Beam", &files, false).unwrap();
        assert!(bound.solver.is_some());
        let restore = dir.join("restore");
        let job = Job {
            kind: MeasureKind::FieldExtreme {
                field: "DISP".into(),
                component: "U3".into(),
                extreme: Extreme::Min,
                region: MeasureRegion::All,
            },
            step: 1,
            results: bound.clone(),
            node_sets: BTreeMap::new(),
            units: UnitSystem::Unitless,
            dir: restore.clone(),
        };
        let value = run(&job).unwrap();
        assert!(value < 0.0, "{value}");
        let offered = read_offered(bound, 1, restore.clone())
            .recv()
            .unwrap()
            .unwrap();
        assert!(offered.fields.iter().any(|(name, _)| name == "DISP"));
        assert!(!offered.history.is_empty());
        assert!(!restore.exists());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn measures_read_extremes_nodes_and_sets() {
        let frd = plx_io::frd::read_frd(&testdata("kragbalken_c3d8.frd")).unwrap();
        let step = frd.increments.last().unwrap().step;
        let mises = MeasureKind::default();
        let max = evaluate(
            &mises,
            step,
            &frd,
            None,
            &BTreeMap::new(),
            UnitSystem::default(),
        )
        .unwrap();
        assert!(max > 0.0);
        let first = frd.mesh.node_ids()[0];
        let sets = BTreeMap::from([("ONE".to_owned(), vec![first])]);
        let in_set = MeasureKind::FieldExtreme {
            field: "STRESS".into(),
            component: "MISES".into(),
            extreme: Extreme::Max,
            region: MeasureRegion::NodeSet("ONE".into()),
        };
        let at = MeasureKind::NodeValue {
            field: "STRESS".into(),
            component: "MISES".into(),
            node: first,
        };
        let one = evaluate(&in_set, step, &frd, None, &sets, UnitSystem::default()).unwrap();
        assert_eq!(
            one,
            evaluate(&at, step, &frd, None, &sets, UnitSystem::default()).unwrap()
        );
        assert!(one <= max);
        // Errors say why.
        let missing = MeasureKind::FieldExtreme {
            field: "NOPE".into(),
            component: "X".into(),
            extreme: Extreme::Max,
            region: MeasureRegion::All,
        };
        assert!(evaluate(&missing, step, &frd, None, &sets, UnitSystem::default()).is_err());
        let eigen = MeasureKind::Eigenfrequency { mode: 1 };
        assert!(evaluate(&eigen, step, &frd, None, &sets, UnitSystem::default()).is_err());
    }

    /// On shells the solver expanded, a node set takes the stresses on both faces and a node
    /// the mean displacement through the thickness.
    #[test]
    fn measures_on_expanded_shells_take_the_expanded_nodes() {
        // A cantilever strip 100 x 20 x 2 of S4 shells, clamped at x = 0, 10 N down at the
        // tip: 75 MPa bending stress at the clamp after beam theory.
        let mut text = String::from("*Node\n");
        for i in 0..=20 {
            for j in 0..=2 {
                let id = i * 3 + j + 1;
                text.push_str(&format!(
                    "{id}, {}, {}, 0\n",
                    5.0 * i as f64,
                    10.0 * j as f64
                ));
            }
        }
        text.push_str("*Element, Type=S4, Elset=Strip\n");
        for i in 0..20 {
            for j in 0..2 {
                let n = i * 3 + j + 1;
                let e = i * 2 + j + 1;
                text.push_str(&format!("{e}, {n}, {}, {}, {}\n", n + 3, n + 4, n + 1));
            }
        }
        text.push_str(
            "*Material, Name=S\n*Elastic\n210000, 0.3\n\
             *Shell section, Elset=Strip, Material=S\n2\n\
             *Step\n*Static\n*Boundary\n1, 1, 6\n2, 1, 6\n3, 1, 6\n\
             *Cload\n61, 3, -2.5\n62, 3, -5\n63, 3, -2.5\n\
             *Node file\nU, RF\n*El file\nS\n*End step\n",
        );
        let dir = std::env::temp_dir().join(format!("plx-measure-shell-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("strip.inp"), &text).unwrap();
        plxs_session::run_input_file(&dir, "strip").unwrap_or_else(|e| panic!("{e}"));
        let frd = plx_io::frd::read_frd(&dir.join("strip.frd")).unwrap();
        let input = plx_io::inp::read_inp(&dir.join("strip.inp")).unwrap().mesh;
        let _ = std::fs::remove_dir_all(&dir);
        let expanded = plx_io::frd::expanded_nodes(&input, &frd.mesh);
        let step = frd.increments.last().unwrap().step;
        let sets = BTreeMap::from([("CLAMP".to_owned(), vec![1, 2, 3])]);
        let measure = |kind: &MeasureKind| {
            evaluate_expanded(
                kind,
                step,
                &frd,
                None,
                &sets,
                &expanded,
                UnitSystem::default(),
            )
        };
        let s11 = |extreme| MeasureKind::FieldExtreme {
            field: "STRESS".into(),
            component: "S11".into(),
            extreme,
            region: MeasureRegion::NodeSet("CLAMP".into()),
        };
        let top = measure(&s11(Extreme::Max)).unwrap();
        let bottom = measure(&s11(Extreme::Min)).unwrap();
        assert!(top > 60.0 && bottom < -60.0, "{top} {bottom}");
        // The tip sags by F L^3 / (3 E I) = 10 * 100^3 / (3 * 210000 * 20 * 8 / 12).
        let tip = measure(&MeasureKind::NodeValue {
            field: "DISP".into(),
            component: "U3".into(),
            node: 62,
        })
        .unwrap();
        let expected = -10.0 * 1e6 / (3.0 * 210_000.0 * 20.0 * 8.0 / 12.0);
        assert!(
            (tip - expected).abs() < 0.05 * expected.abs(),
            "{tip} vs {expected}"
        );
        // The clamp holds the 10 N.
        let held: f64 = (1..=3)
            .map(|node| {
                measure(&MeasureKind::NodeValue {
                    field: "FORC".into(),
                    component: "F3".into(),
                    node,
                })
                .unwrap()
            })
            .sum();
        assert!((held - 10.0).abs() < 1e-3, "{held}");
    }

    #[test]
    fn a_measure_at_a_node_equals_the_measure_at_its_point() {
        let frd = plx_io::frd::read_frd(&testdata("kragbalken_c3d8.frd")).unwrap();
        let step = frd.increments.last().unwrap().step;
        let sets = BTreeMap::new();
        for (index, &node) in frd.mesh.node_ids().iter().enumerate().step_by(7) {
            let at_node = MeasureKind::NodeValue {
                field: "DISP".into(),
                component: "U2".into(),
                node,
            };
            let at_point = MeasureKind::PointValue {
                field: "DISP".into(),
                component: "U2".into(),
                point: frd.mesh.coords()[index],
            };
            let a = evaluate(&at_node, step, &frd, None, &sets, UnitSystem::default()).unwrap();
            let b = evaluate(&at_point, step, &frd, None, &sets, UnitSystem::default()).unwrap();
            assert!(
                (a - b).abs() <= 1e-9 * a.abs().max(1e-12),
                "node {node}: {a} != {b}"
            );
        }
        // Between two nodes the value lies between theirs; far away there is none.
        let coords = frd.mesh.coords();
        let (low, high) = coords.iter().fold(
            ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]),
            |(mut low, mut high), c| {
                for k in 0..3 {
                    low[k] = low[k].min(c[k]);
                    high[k] = high[k].max(c[k]);
                }
                (low, high)
            },
        );
        let middle: [f64; 3] = std::array::from_fn(|k| 0.5 * (low[k] + high[k]));
        let at = |point| MeasureKind::PointValue {
            field: "DISP".into(),
            component: "ALL".into(),
            point,
        };
        assert!(
            evaluate(&at(middle), step, &frd, None, &sets, UnitSystem::default())
                .unwrap()
                .is_finite()
        );
        let far: [f64; 3] = std::array::from_fn(|k| high[k] + 10.0 * (high[k] - low[k]));
        let why = evaluate(&at(far), step, &frd, None, &sets, UnitSystem::default()).unwrap_err();
        assert!(why.contains("outside the mesh"), "{why}");
    }

    #[test]
    fn the_results_offer_their_fields_and_components() {
        let frd = plx_io::frd::read_frd(&testdata("kragbalken_c3d8.frd")).unwrap();
        let step = frd.increments.last().unwrap().step;
        let offered = offered(step, &frd, None);
        let stress = (offered.fields.iter())
            .find(|(name, _)| name == "STRESS")
            .map(|(_, components)| components)
            .unwrap();
        assert!(stress.iter().any(|c| c == "MISES"));
        let disp = offered
            .fields
            .iter()
            .find(|(name, _)| name == "DISP")
            .unwrap();
        assert!(disp.1.iter().any(|c| c == "ALL"));
        assert!(offered.history.is_empty());
        assert_eq!(super::offered(step + 1, &frd, None), Offered::default());
    }
}
