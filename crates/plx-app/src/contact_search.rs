//! PrePoMax's "Search Contact Pairs": finds the surfaces of different parts that touch and
//! turns them into ties or contact pairs. The pairs found are listed in a table; the ones
//! selected there are shown in the 3D view, master in the primary and slave in the secondary
//! highlight colour, and can be edited together, swapped or merged before OK creates them.

use std::collections::{BTreeSet, HashSet};

use egui::Ui;
use plx_mesh::{
    CadEntity, ElementId, GeometryFilter, GroupBy, LineJoint, MasterSlaveItem, NodeId,
    SearchParameters, SurfaceId, find_contact_pairs, find_line_joints, surface_faces,
};
use plx_model::{
    ContactMethod, ContactPair, FeModel, NodeTie, Quantity, Region, Tie, UnitSystem, next_name,
};

use crate::model::{Highlight, Model};
use crate::numeric;
use crate::selection::edge_lines;
use crate::setup::shown_faces;

/// What a pair found becomes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PairType {
    Tie,
    Contact,
}

impl PairType {
    fn label(self) -> &'static str {
        match self {
            PairType::Tie => "Tie",
            PairType::Contact => "Contact",
        }
    }
}

/// The methods PrePoMax offers in the search, and mortar.
const METHODS: [ContactMethod; 3] = [
    ContactMethod::NodeToSurface,
    ContactMethod::SurfaceToSurface,
    ContactMethod::Mortar,
];

/// A row of the table: a pair found with the settings it is created with, or line ends
/// meeting at a point, which become a node tie.
#[derive(Clone, Debug)]
struct Row {
    name: String,
    item: MasterSlaveItem,
    /// Line ends on one point; `item` then only names their parts.
    joint: Option<LineJoint>,
    /// A node tie ties the rotations too.
    rotations: bool,
    kind: PairType,
    interaction: String,
    method: ContactMethod,
    adjust: bool,
    distance: f64,
    selected: bool,
    /// The pair is created by the verb of the window.
    checked: bool,
}

impl Row {
    /// What touches, as in PrePoMax's table: solids, shells, their edges, the edges of a 2D
    /// model, or line ends.
    fn geometry(&self) -> String {
        if self.joint.is_some() {
            "Line end-Line end".into()
        } else {
            self.item.geometry()
        }
    }

    /// The type cell: a node tie for line ends, else the pair type.
    fn type_label(&self) -> &'static str {
        if self.joint.is_some() {
            "Node tie"
        } else {
            self.kind.label()
        }
    }
}

/// Width of the table of the pairs found.
const TABLE_WIDTH: f32 = 640.0;
/// Width of the properties of the selected pairs beside the table.
const PROPERTIES_WIDTH: f32 = 290.0;
/// Width of the value column of the properties.
const PROPERTY_VALUE_WIDTH: f32 = 150.0;
/// Width of the window's content: the table, the gap and the properties. Fixed, because egui
/// never shrinks a window by itself and keeps its size in the saved settings, so a window
/// that once grew would stay wide for good.
const CONTENT_WIDTH: f32 = TABLE_WIDTH + PROPERTY_GAP + PROPERTIES_WIDTH;
/// Space between the table and the properties, item spacing included.
const PROPERTY_GAP: f32 = 24.0;

pub struct ContactSearchDialog {
    distance: f64,
    angle: f64,
    group_by: GroupBy,
    ignore_hidden: bool,
    /// Which surfaces are searched; in 2D models always the edges.
    filter: GeometryFilter,
    /// Also searches the ends of beams and trusses meeting at a point.
    line_ends: bool,
    kind: PairType,
    interaction: String,
    method: ContactMethod,
    adjust: bool,
    rows: Vec<Row>,
    /// Names of the parts when the search ran, for naming merged pairs.
    part_names: Vec<String>,
    /// The search ran and found nothing.
    searched: bool,
    /// A 2D model, whose surfaces are the edges of its elements.
    plane: bool,
    error: Option<String>,
    /// What the 3D view last showed, keyed by the selected rows: finding the faces of the
    /// surfaces walks the skins of all parts, too slow for every frame of a large model.
    shown: Option<Vec<ShownPair>>,
}

/// Master and slave surfaces of a selected pair, or the nodes of a line joint.
type ShownPair = (BTreeSet<SurfaceId>, BTreeSet<SurfaceId>, Vec<NodeId>);

pub enum SearchResult {
    Open,
    /// Create these ties, contact pairs and node ties.
    Ok(Vec<Tie>, Vec<ContactPair>, Vec<NodeTie>),
    Cancel,
}

impl ContactSearchDialog {
    pub fn new(fe: &FeModel) -> Self {
        Self {
            // PrePoMax's 0.01 mm.
            distance: UnitSystem::MmTonSC.stored(0.01, Quantity::Length),
            angle: 35.0,
            group_by: GroupBy::Parts,
            ignore_hidden: true,
            filter: GeometryFilter::default(),
            line_ends: true,
            kind: PairType::Tie,
            interaction: (fe.surface_interactions.first())
                .map(|s| s.name.clone())
                .unwrap_or_default(),
            method: ContactMethod::SurfaceToSurface,
            adjust: false,
            rows: Vec::new(),
            part_names: Vec::new(),
            searched: false,
            plane: fe.properties.space.is_2d(),
            error: None,
            shown: None,
        }
    }

    /// The pairs selected in the table for the 3D view, when they changed since the last call.
    pub fn changed_highlight(&mut self, model: &Model) -> Option<Highlight> {
        let shown = (self.rows.iter().filter(|r| r.selected))
            .map(|r| {
                let nodes = r
                    .joint
                    .as_ref()
                    .map(|j| j.nodes.clone())
                    .unwrap_or_default();
                (r.item.master.clone(), r.item.slave.clone(), nodes)
            })
            .collect();
        if self.shown.as_ref() == Some(&shown) {
            return None;
        }
        self.shown = Some(shown);
        Some(self.highlight(model))
    }

    /// The next call of `changed_highlight` builds the highlight anew.
    pub fn forget_highlight(&mut self) {
        self.shown = None;
    }

    /// The pairs selected in the table, for the 3D view.
    fn highlight(&self, model: &Model) -> Highlight {
        let mut master = BTreeSet::new();
        let mut slave = BTreeSet::new();
        let mut nodes = Vec::new();
        for row in self.rows.iter().filter(|r| r.selected) {
            if let Some(joint) = &row.joint {
                nodes.extend(joint.nodes.iter().copied());
                continue;
            }
            master.extend(row.item.master.iter().copied());
            slave.extend(row.item.slave.iter().copied());
        }
        let master = surface_faces(&model.mesh, model.skins(), &master);
        let slave = surface_faces(&model.mesh, model.skins(), &slave);
        if self.plane {
            // The faces of 2D elements are edges, drawn as lines.
            let lines = |faces: Vec<_>| edge_lines(model, &faces.into_iter().collect());
            return Highlight {
                nodes,
                lines: lines(master),
                secondary_lines: lines(slave),
                ..Highlight::default()
            };
        }
        let (faces, lines) = shown_faces(model, master);
        let (secondary_faces, secondary_lines) = shown_faces(model, slave);
        Highlight {
            nodes,
            faces,
            secondary_faces,
            lines,
            secondary_lines,
            ..Highlight::default()
        }
    }

    fn search(&mut self, model: &Model) {
        let parameters = SearchParameters {
            distance: self.distance,
            angle_deg: self.angle,
            group_by: self.group_by,
            stiffness: part_stiffness(model),
            filter: if self.plane {
                GeometryFilter {
                    solid: false,
                    shell: false,
                    shell_edge: true,
                }
            } else {
                self.filter
            },
        };
        let searched: Vec<bool> = (model.parts.iter())
            .map(|p| p.visible || !self.ignore_hidden)
            .collect();
        let items = find_contact_pairs(&model.mesh, model.skins(), &searched, &parameters);
        self.rows = items
            .into_iter()
            .map(|item| Row {
                checked: true,
                name: item.name(),
                item,
                joint: None,
                rotations: true,
                kind: self.kind,
                interaction: self.interaction.clone(),
                method: self.method,
                adjust: self.adjust,
                distance: self.distance,
                selected: false,
            })
            .collect();
        if self.line_ends {
            let joints = find_line_joints(&model.mesh, &searched, self.distance);
            for (n, joint) in joints.into_iter().enumerate() {
                let parts: Vec<&str> = (joint.parts.iter())
                    .filter_map(|&p| model.parts.get(p))
                    .map(|p| p.name.as_str())
                    .collect();
                self.rows.push(Row {
                    checked: true,
                    name: format!("Node_Tie-{}", n + 1),
                    item: MasterSlaveItem {
                        master_name: parts.join("_"),
                        slave_name: String::new(),
                        master: BTreeSet::new(),
                        slave: BTreeSet::new(),
                    },
                    joint: Some(joint),
                    rotations: true,
                    kind: self.kind,
                    interaction: self.interaction.clone(),
                    method: self.method,
                    adjust: false,
                    distance: self.distance,
                    selected: false,
                });
            }
        }
        self.part_names = model.parts.iter().map(|p| p.name.clone()).collect();
        self.searched = true;
    }

    pub fn show(&mut self, ctx: &egui::Context, model: &Model) -> SearchResult {
        let mut result = SearchResult::Open;
        let mut open = true;
        egui::Window::new("Search Contact Pairs")
            .id(crate::ids::window(ctx, "contact search"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .min_width(CONTENT_WIDTH)
            .max_width(CONTENT_WIDTH)
            .default_pos(ctx.content_rect().left_top() + egui::vec2(300.0, 90.0))
            .show(ctx, |ui| {
                self.parameters(ui, model);
                ui.separator();
                ui.label("Contact pairs");
                // Fixed widths: a table sized by the window would grow the window whenever
                // the properties beside it need more room, without end.
                ui.horizontal_top(|ui| {
                    let size = egui::vec2(TABLE_WIDTH, 340.0);
                    ui.allocate_ui(size, |ui| self.table(ui, model.fe.properties.units));
                    ui.add_space(PROPERTY_GAP - 2.0 * ui.spacing().item_spacing.x);
                    ui.vertical(|ui| {
                        ui.set_width(PROPERTIES_WIDTH);
                        self.properties(ui, model);
                    });
                });
                if let Some(error) = &self.error {
                    ui.colored_label(crate::style::error(), error);
                }
                // A work window: Cancel and the verb, which names what it makes.
                let count = self.rows.iter().filter(|r| r.checked).count();
                let verb = match count {
                    1 => "Create 1 Pair".to_owned(),
                    n => format!("Create {n} Pairs"),
                };
                match crate::dialog::work_footer(ui, Some((&verb, count > 0))) {
                    crate::dialog::Work::Verb => match self.create(model) {
                        Ok((ties, pairs, joints)) => result = SearchResult::Ok(ties, pairs, joints),
                        Err(error) => self.error = Some(error),
                    },
                    crate::dialog::Work::Close => result = SearchResult::Cancel,
                    crate::dialog::Work::None => {}
                }
            });
        if !open {
            result = SearchResult::Cancel;
        }
        result
    }

    /// The four groups above the table, as in PrePoMax.
    fn parameters(&mut self, ui: &mut Ui, model: &Model) {
        ui.horizontal_top(|ui| {
            group(ui, "Search Parameters", |ui| {
                egui::Grid::new("search parameters").show(ui, |ui| {
                    ui.label("Distance").on_hover_text(
                        "Two surfaces count as touching when their gap is smaller than this.",
                    );
                    let units = model.fe.properties.units;
                    ui.add(
                        numeric::quantity(&mut self.distance, units, Quantity::Length)
                            .range(0.0..=f64::MAX)
                            .speed(0.001),
                    );
                    ui.end_row();
                    ui.label("Angle").on_hover_text(
                        "Largest angle between the normals of two touching faces. 35 degrees \
                         skips faces that only meet at a sharp edge.",
                    );
                    ui.add(
                        numeric::drag_value(&mut self.angle)
                            .range(0.0..=180.0)
                            .speed(1.0)
                            .suffix(" °"),
                    );
                    ui.end_row();
                    ui.label("Group by").on_hover_text(
                        "None: one pair per touching face pair. Parts: one pair per pair of \
                         parts. Graph: merges surfaces so that each node is slave only once.",
                    );
                    egui::ComboBox::from_id_salt("group by")
                        .selected_text(group_label(self.group_by))
                        .show_ui(ui, |ui| {
                            for choice in [GroupBy::None, GroupBy::Parts, GroupBy::Graph] {
                                ui.selectable_value(
                                    &mut self.group_by,
                                    choice,
                                    group_label(choice),
                                );
                            }
                        });
                    ui.end_row();
                });
            });
            group(ui, "Geometry Filter", |ui| {
                if self.plane {
                    // The edges of the elements are the surfaces of a 2D model.
                    let (mut solid, mut shell, mut edge) = (false, false, true);
                    let why = "In a 2D model the element edges are the surfaces.";
                    ui.add_enabled(false, egui::Checkbox::new(&mut solid, "Solid"))
                        .on_disabled_hover_text(why);
                    ui.add_enabled(false, egui::Checkbox::new(&mut shell, "Shell"))
                        .on_disabled_hover_text(why);
                    ui.add_enabled(false, egui::Checkbox::new(&mut edge, "Shell edge"))
                        .on_disabled_hover_text(why);
                } else {
                    let filter = &mut self.filter;
                    ui.checkbox(&mut filter.solid, "Solid")
                        .on_hover_text("Faces of solid parts");
                    ui.checkbox(&mut filter.shell, "Shell")
                        .on_hover_text("Top and bottom of shell and membrane parts");
                    ui.checkbox(&mut filter.shell_edge, "Shell edge")
                        .on_hover_text(
                            "Free edges of shell and membrane parts, e.g. where a shell meets a \
                         solid face",
                        );
                }
                ui.checkbox(&mut self.line_ends, "Line end")
                    .on_hover_text("Ends of beams and trusses that lie on one point");
                ui.checkbox(&mut self.ignore_hidden, "Ignore hidden parts");
            });
            group(ui, "Contact Pair Parameters", |ui| {
                egui::Grid::new("pair parameters").show(ui, |ui| {
                    ui.label("Type").on_hover_text(
                        "Tie glues the surfaces together without sliding or separation. \
                         Contact lets them touch, slide and separate, as defined by the \
                         Surface Interaction.",
                    );
                    egui::ComboBox::from_id_salt("pair type")
                        .selected_text(self.kind.label())
                        .show_ui(ui, |ui| {
                            for kind in [PairType::Tie, PairType::Contact] {
                                ui.selectable_value(&mut self.kind, kind, kind.label());
                            }
                        });
                    ui.end_row();
                    let contact = self.kind == PairType::Contact;
                    ui.label("Surface Interaction");
                    ui.add_enabled_ui(contact, |ui| {
                        interaction_combo(ui, "search interaction", &mut self.interaction, model);
                    });
                    ui.end_row();
                    ui.label("Method").on_hover_text(self.method.description());
                    ui.add_enabled_ui(contact, |ui| {
                        method_combo(ui, "search method", &mut self.method);
                    });
                    ui.end_row();
                    ui.label("Adjust mesh").on_hover_text(
                        "Moves slave nodes onto the master surface at the start, so that \
                         meshing gaps and overlaps do not disturb the contact.",
                    );
                    yes_no(ui, "search adjust", &mut self.adjust);
                    ui.end_row();
                });
            });
            ui.vertical(|ui| {
                ui.add_space(70.0);
                if ui
                    .add_sized([90.0, 24.0], egui::Button::new("Search"))
                    .clicked()
                {
                    self.error = None;
                    self.search(model);
                }
            });
        });
    }

    fn table(&mut self, ui: &mut Ui, units: UnitSystem) {
        egui::Frame::new()
            .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
            .fill(ui.visuals().extreme_bg_color)
            .inner_margin(4.0)
            .show(ui, |ui| {
                ui.set_min_size(ui.available_size());
                // Solid scroll bars take their own space instead of covering the last row.
                ui.spacing_mut().scroll = egui::style::ScrollStyle::solid();
                egui::ScrollArea::both().show(ui, |ui| {
                    ui.vertical(|ui| self.rows_grid(ui, units));
                });
            });
    }

    fn rows_grid(&mut self, ui: &mut Ui, units: UnitSystem) {
        // Selected rows are filled across all columns, as in PrePoMax's table.
        let selected: Vec<bool> = self.rows.iter().map(|r| r.selected).collect();
        let fill = ui.visuals().selection.bg_fill;
        let text_color = ui.visuals().selection.stroke.color;
        egui::Grid::new("contact pairs")
            .striped(true)
            .min_col_width(16.0)
            .spacing([12.0, 4.0])
            .with_row_color(move |row, _| {
                let index = row.checked_sub(1)?;
                selected.get(index).copied().filter(|&s| s).map(|_| fill)
            })
            .show(ui, |ui| {
                // The header's checkbox checks or unchecks all pairs.
                let mut all = !self.rows.is_empty() && self.rows.iter().all(|r| r.checked);
                let any = self.rows.iter().any(|r| r.checked);
                let partial = any && !all;
                let header = egui::Checkbox::new(&mut all, "").indeterminate(partial);
                if ui
                    .add(header)
                    .on_hover_text("Select or deselect all contact pairs")
                    .changed()
                {
                    for row in &mut self.rows {
                        row.checked = all;
                    }
                }
                for title in [
                    "Name",
                    "Geometry",
                    "Type",
                    "Surface interaction",
                    "Method",
                    "Adjust",
                    &format!("Distance [{}]", units.unit(Quantity::Length)),
                ] {
                    ui.strong(title);
                }
                ui.end_row();
                let mut clicked = None;
                let mut menu = None;
                let mut toggled = None;
                for (i, row) in self.rows.iter().enumerate() {
                    let mut checked = row.checked;
                    let checkbox = ui
                        .add(egui::Checkbox::without_text(&mut checked))
                        .on_hover_text("Create on OK");
                    if checkbox.changed() {
                        toggled = Some((i, checked));
                    }
                    let contact = row.kind == PairType::Contact;
                    let cells = [
                        row.name.clone(),
                        row.geometry(),
                        row.type_label().to_string(),
                        if contact {
                            row.interaction.clone()
                        } else {
                            String::new()
                        },
                        if contact {
                            row.method.name().into()
                        } else {
                            String::new()
                        },
                        if row.adjust { "Yes" } else { "No" }.to_string(),
                        numeric::format_physical(units.shown(row.distance, Quantity::Length)),
                    ];
                    for cell in cells {
                        let mut text = egui::RichText::new(cell);
                        if row.selected {
                            text = text.color(text_color);
                        }
                        let label = egui::Label::new(text).sense(egui::Sense::click());
                        let response = ui.add(label);
                        if response.clicked() {
                            clicked = Some(i);
                        }
                        if response.secondary_clicked() && !row.selected {
                            clicked = Some(i);
                        }
                        response.context_menu(|ui| menu = self.row_menu(ui));
                    }
                    ui.end_row();
                }
                if let Some((i, checked)) = toggled {
                    self.rows[i].checked = checked;
                }
                if let Some(i) = clicked {
                    let (shift, ctrl) =
                        ui.input(|input| (input.modifiers.shift, input.modifiers.command));
                    self.select(i, shift, ctrl);
                }
                match menu {
                    Some(RowAction::Swap) => self.swap(),
                    Some(RowAction::Merge) => self.merge(),
                    None => {}
                }
            });
        if self.rows.is_empty() {
            ui.weak(if self.searched {
                "No contact pairs found."
            } else {
                "Use \"Search\" to find the contact pairs."
            });
        }
    }

    fn row_menu(&self, ui: &mut Ui) -> Option<RowAction> {
        let mut action = None;
        if ui.button("Swap Master/Slave").clicked() {
            action = Some(RowAction::Swap);
        }
        let several = (self.rows.iter())
            .filter(|r| r.selected && r.joint.is_none())
            .count()
            > 1;
        if ui
            .add_enabled(several, egui::Button::new("Merge by Master/Slave"))
            .clicked()
        {
            action = Some(RowAction::Merge);
        }
        action
    }

    /// A click selects a row; Ctrl toggles it, Shift extends to it from the last selected.
    fn select(&mut self, index: usize, shift: bool, ctrl: bool) {
        if ctrl {
            self.rows[index].selected = !self.rows[index].selected;
        } else if shift && let Some(anchor) = self.rows.iter().position(|r| r.selected) {
            let (a, b) = (anchor.min(index), anchor.max(index));
            for (i, row) in self.rows.iter_mut().enumerate() {
                row.selected = (a..=b).contains(&i);
            }
        } else {
            for (i, row) in self.rows.iter_mut().enumerate() {
                row.selected = i == index;
            }
        }
    }

    fn swap(&mut self) {
        for row in (self.rows.iter_mut()).filter(|r| r.selected && r.joint.is_none()) {
            row.item.swap();
            row.name = row.item.name();
        }
    }

    /// PrePoMax's "Merge by Master/Slave": the selected pairs become one.
    fn merge(&mut self) {
        let selected: Vec<usize> = (self.rows.iter().enumerate())
            .filter(|(_, r)| r.selected && r.joint.is_none())
            .map(|(i, _)| i)
            .collect();
        let Some((&first, rest)) = selected.split_first() else {
            return;
        };
        for &i in rest {
            let (master, slave) = (
                self.rows[i].item.master.clone(),
                self.rows[i].item.slave.clone(),
            );
            self.rows[first].item.master.extend(master);
            self.rows[first].item.slave.extend(slave);
        }
        let taken: Vec<String> = (self.rows.iter())
            .flat_map(|r| [r.item.master_name.clone(), r.item.slave_name.clone()])
            .collect();
        let row = &mut self.rows[first];
        row.item.master_name = side_name(&row.item.master, &taken, &self.part_names);
        let mut taken = taken;
        taken.push(row.item.master_name.clone());
        row.item.slave_name = side_name(&row.item.slave, &taken, &self.part_names);
        row.name = row.item.name();
        for &i in rest.iter().rev() {
            self.rows.remove(i);
        }
    }

    /// The properties of the selected rows, edited together like PrePoMax's property grid.
    fn properties(&mut self, ui: &mut Ui, model: &Model) {
        let selected: Vec<usize> = (self.rows.iter().enumerate())
            .filter(|(_, r)| r.selected)
            .map(|(i, _)| i)
            .collect();
        let Some(&first) = selected.first() else {
            ui.weak("Select a contact pair in the table to edit it.");
            return;
        };
        let mut row = self.rows[first].clone();
        // The values keep a fixed width, so that long names do not widen the column.
        ui.spacing_mut().combo_width = PROPERTY_VALUE_WIDTH;
        if row.joint.is_some() {
            egui::Grid::new("joint properties")
                .num_columns(2)
                .show(ui, |ui| {
                    if selected.len() == 1 {
                        ui.label("Name");
                        ui.add(
                            egui::TextEdit::singleline(&mut row.name)
                                .desired_width(PROPERTY_VALUE_WIDTH),
                        );
                        ui.end_row();
                    }
                    ui.label("Type");
                    ui.label("Node tie");
                    ui.end_row();
                    ui.label("Rotations");
                    yes_no(ui, "row rotations", &mut row.rotations);
                    ui.end_row();
                });
            ui.add_space(8.0);
            ui.weak(
                "The nodes of the line ends follow each other as equations. Rotations: \
                 rigid instead of hinged; trusses have none.",
            );
            let old = self.rows[first].clone();
            for &i in &selected {
                let target = &mut self.rows[i];
                if i == first {
                    target.name = row.name.clone();
                }
                if row.rotations != old.rotations && target.joint.is_some() {
                    target.rotations = row.rotations;
                }
            }
            return;
        }
        egui::Grid::new("pair properties")
            .num_columns(2)
            .show(ui, |ui| {
                if selected.len() == 1 {
                    ui.label("Name");
                    ui.add(
                        egui::TextEdit::singleline(&mut row.name)
                            .desired_width(PROPERTY_VALUE_WIDTH),
                    );
                    ui.end_row();
                }
                ui.label("Type");
                egui::ComboBox::from_id_salt("row type")
                    .selected_text(row.kind.label())
                    .show_ui(ui, |ui| {
                        for kind in [PairType::Tie, PairType::Contact] {
                            ui.selectable_value(&mut row.kind, kind, kind.label());
                        }
                    });
                ui.end_row();
                if row.kind == PairType::Contact {
                    ui.label("Surface Interaction");
                    interaction_combo(ui, "row interaction", &mut row.interaction, model);
                    ui.end_row();
                    ui.label("Method");
                    method_combo(ui, "row method", &mut row.method);
                    ui.end_row();
                }
                ui.label("Adjust");
                yes_no(ui, "row adjust", &mut row.adjust);
                ui.end_row();
                ui.label("Distance");
                ui.add(
                    numeric::quantity(
                        &mut row.distance,
                        model.fe.properties.units,
                        Quantity::Length,
                    )
                    .range(0.0..=f64::MAX)
                    .speed(0.001),
                );
                ui.end_row();
            });
        ui.add_space(8.0);
        ui.weak(match row.kind {
            PairType::Tie => "Distance: position tolerance of the tie.",
            PairType::Contact => "Distance: range in which Adjust moves the slave nodes.",
        });
        let old = self.rows[first].clone();
        for &i in &selected {
            let target = &mut self.rows[i];
            if i == first {
                target.name = row.name.clone();
            }
            if target.joint.is_some() {
                continue;
            }
            if row.kind != old.kind {
                target.kind = row.kind;
            }
            if row.interaction != old.interaction {
                target.interaction = row.interaction.clone();
            }
            if row.method != old.method {
                target.method = row.method;
            }
            if row.adjust != old.adjust {
                target.adjust = row.adjust;
            }
            if row.distance != old.distance {
                target.distance = row.distance;
            }
        }
    }

    /// Ties, contact pairs and node ties of the rows, with names not yet taken in the model.
    #[allow(clippy::type_complexity)]
    fn create(&self, model: &Model) -> Result<(Vec<Tie>, Vec<ContactPair>, Vec<NodeTie>), String> {
        let fe = &model.fe;
        let mut ties = Vec::new();
        let mut pairs = Vec::new();
        let mut joints = Vec::new();
        let mut joint_names: Vec<String> = fe.node_ties.iter().map(|t| t.name.clone()).collect();
        let mut tie_names: Vec<String> = fe.ties.iter().map(|t| t.name.clone()).collect();
        let mut pair_names: Vec<String> = fe.contact_pairs.iter().map(|c| c.name.clone()).collect();
        let unique = |name: &str, taken: &mut Vec<String>| {
            let name = if taken.iter().any(|t| t.eq_ignore_ascii_case(name)) {
                next_name(name, taken.iter().map(String::as_str))
            } else {
                name.to_string()
            };
            taken.push(name.clone());
            name
        };
        for row in self.rows.iter().filter(|r| r.checked) {
            if let Some(joint) = &row.joint {
                joints.push(NodeTie {
                    region: joint_region(model, joint),
                    rotations: row.rotations,
                    ..NodeTie::new(unique(&row.name, &mut joint_names))
                });
                continue;
            }
            // Surfaces of whole CAD faces are kept by geometry, so that they survive
            // remeshing.
            let region = |surface| {
                let faces = surface_faces(&model.mesh, model.skins(), surface);
                match model.mesh.whole_cad_faces(&faces) {
                    Some(entities) => Region::Geometry(entities),
                    None => Region::Faces(faces),
                }
            };
            let (master, slave) = (region(&row.item.master), region(&row.item.slave));
            match row.kind {
                PairType::Tie => ties.push(Tie {
                    position_tolerance: Some(row.distance),
                    adjust: row.adjust,
                    master,
                    slave,
                    ..Tie::new(unique(&row.name, &mut tie_names))
                }),
                PairType::Contact => {
                    if !(fe.surface_interactions.iter()).any(|s| s.name == row.interaction) {
                        return Err(format!(
                            "{}: please select a Surface Interaction; create one under Surface \
                             Interactions first.",
                            row.name
                        ));
                    }
                    pairs.push(ContactPair {
                        method: row.method,
                        adjust: row.adjust,
                        adjustment_size: Some(row.distance),
                        clearance: None,
                        master,
                        slave,
                        ..ContactPair::new(unique(&row.name, &mut pair_names), &row.interaction)
                    });
                }
            }
        }
        Ok((ties, pairs, joints))
    }
}

enum RowAction {
    Swap,
    Merge,
}

/// The nodes of a joint as a region: the CAD vertices they lie on, if those have no other
/// nodes, so that the tie survives remeshing; else the nodes themselves.
fn joint_region(model: &Model, joint: &LineJoint) -> Region {
    let vertices: BTreeSet<CadEntity> = (model.mesh.cad.nodes.iter())
        .filter(|(entity, nodes)| {
            matches!(entity, CadEntity::Vertex(_)) && nodes.iter().any(|n| joint.nodes.contains(n))
        })
        .map(|(&entity, _)| entity)
        .collect();
    let covered: BTreeSet<_> = (vertices.iter())
        .flat_map(|v| model.mesh.cad.nodes[v].iter().copied())
        .collect();
    if !vertices.is_empty() && covered == joint.nodes.iter().copied().collect() {
        Region::Geometry(vertices.into_iter().collect())
    } else {
        Region::Nodes(joint.nodes.clone())
    }
}

/// Name of a merged side: its part, or the next free "Merged-n".
fn side_name(ids: &BTreeSet<plx_mesh::SurfaceId>, taken: &[String], parts: &[String]) -> String {
    let of: BTreeSet<usize> = ids.iter().map(|id| id.0).collect();
    if let (1, Some(name)) = (of.len(), of.first().and_then(|&p| parts.get(p))) {
        return name.clone();
    }
    (1..)
        .map(|n| format!("Merged-{n}"))
        .find(|name| !taken.contains(name))
        .expect("unbounded range")
}

fn group(ui: &mut Ui, title: &str, body: impl FnOnce(&mut Ui)) {
    ui.group(|ui| {
        ui.vertical(|ui| {
            ui.strong(title);
            body(ui);
        });
    });
}

fn group_label(group_by: GroupBy) -> &'static str {
    match group_by {
        GroupBy::None => "None",
        GroupBy::Parts => "Parts",
        GroupBy::Graph => "Graph",
    }
}

fn interaction_combo(ui: &mut Ui, id: &str, interaction: &mut String, model: &Model) {
    let text = if interaction.is_empty() {
        "Missing"
    } else {
        interaction.as_str()
    };
    egui::ComboBox::from_id_salt(id)
        .selected_text(text)
        .show_ui(ui, |ui| {
            for candidate in &model.fe.surface_interactions {
                let name = candidate.name.clone();
                ui.selectable_value(interaction, name, &candidate.name);
            }
        });
}

fn method_combo(ui: &mut Ui, id: &str, method: &mut ContactMethod) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(method.name())
        .show_ui(ui, |ui| {
            for choice in METHODS {
                ui.selectable_value(method, choice, choice.name())
                    .on_hover_text(choice.description());
            }
        });
}

fn yes_no(ui: &mut Ui, id: &str, value: &mut bool) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(if *value { "Yes" } else { "No" })
        .show_ui(ui, |ui| {
            ui.selectable_value(value, true, "Yes");
            ui.selectable_value(value, false, "No");
        });
}

/// Young's modulus of each part, the largest of the materials of its sections, if any.
fn part_stiffness(model: &Model) -> Vec<Option<f64>> {
    let (fe, mesh) = (&model.fe, &model.mesh);
    let young = |name: &str| {
        (fe.materials.iter())
            .find(|m| m.name == name)
            .and_then(|m| m.elastic)
            .map(|e| e.young)
    };
    let assigned: Vec<(HashSet<ElementId>, f64)> = (fe.sections.iter())
        .filter_map(|s| {
            Some((
                s.region.elements(mesh).into_iter().collect(),
                young(&s.material)?,
            ))
        })
        .collect();
    (mesh.parts.iter())
        .map(|part| {
            (assigned.iter())
                .filter(|(elements, _)| part.elements.iter().any(|e| elements.contains(e)))
                .map(|&(_, young)| young)
                .reduce(f64::max)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use plx_mesh::{Element, ElementShape, FeMesh, Part};

    use super::*;

    /// Two unit cubes of one hexahedron each, one on top of the other, as separate parts
    /// with their own nodes.
    fn stacked_cubes() -> Model {
        let mut mesh = FeMesh::default();
        let square = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        for part in 0..2u32 {
            let base = part * 8;
            for (i, z) in [part, part + 1].into_iter().enumerate() {
                for (j, (x, y)) in square.into_iter().enumerate() {
                    mesh.set_node(base + (i * 4 + j) as u32 + 1, [x, y, z as f64]);
                }
            }
            mesh.add_element(Element {
                id: part + 1,
                type_name: "C3D8".into(),
                shape: ElementShape::Hex8,
                nodes: (1..=8).map(|n| base + n).collect(),
            })
            .unwrap();
            mesh.parts.push(Part {
                name: format!("PART-{}", part + 1),
                elements: vec![part + 1],
            });
        }
        Model::new(std::path::Path::new("cubes.inp"), mesh)
    }

    #[test]
    fn highlight_is_built_only_when_the_selected_pairs_change() {
        let model = stacked_cubes();
        let mut dialog = ContactSearchDialog::new(&model.fe);
        assert_eq!(dialog.changed_highlight(&model), Some(Highlight::default()));
        assert_eq!(dialog.changed_highlight(&model), None);
        dialog.search(&model);
        assert_eq!(dialog.rows.len(), 1);
        // Found pairs are not selected, so the view stays as it is.
        assert_eq!(dialog.changed_highlight(&model), None);
        dialog.select(0, false, false);
        let shown = dialog.changed_highlight(&model).unwrap();
        assert_eq!((shown.faces.len(), shown.secondary_faces.len()), (1, 1));
        assert_eq!(dialog.changed_highlight(&model), None);
        dialog.swap();
        assert!(dialog.changed_highlight(&model).is_some());
        dialog.forget_highlight();
        assert!(dialog.changed_highlight(&model).is_some());
    }
}
