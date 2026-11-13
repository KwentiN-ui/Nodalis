//! The forms of the interaction dialogs: tie constraints, surface interactions and
//! contact pairs. Clicks in the 3D view fill the master or the slave surface, whichever
//! field's "..." button is pressed; in 2D models the surfaces are element edges. The master
//! shows in the primary, the slave in the secondary highlight colour.

use crate::clicks::TolerantDoubleClick as _;
use egui::{RichText, Ui};
use plx_mesh::FeMesh;
use plx_model::{
    ContactMethod, ContactPair, CyclicSymmetry, FeModel, Friction, GapConductance,
    InteractionProperty, Quantity, Region, SectorDisplay, SurfaceBehavior, SurfaceInteraction, Tie,
    UnitSystem,
};

use crate::icons::{self, Icon};
use crate::model::{CyclicPreview, Highlight, Model};
use crate::numeric;
use crate::selection::Target;
use crate::setup::{FACE_SOURCES, RegionDraft, region_highlight};
use crate::tree_rows::{frame, tree_row};

/// Which of the two regions clicks in the 3D view pick.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Side {
    Master,
    Slave,
}

/// Master and slave region of a tie or contact pair while they are edited.
#[derive(Clone, Debug, PartialEq)]
pub struct MasterSlave {
    pub master: RegionDraft,
    pub slave: RegionDraft,
    /// The region picked in the 3D view.
    pub side: Side,
}

impl MasterSlave {
    /// `faces` is what the 3D view picks: element faces, or element edges in 2D models.
    pub fn new(faces: Target) -> Self {
        Self {
            master: RegionDraft::new(FACE_SOURCES, faces),
            slave: RegionDraft::new(FACE_SOURCES, faces),
            side: Side::Master,
        }
    }

    pub fn from_regions(master: &Region, slave: &Region, faces: Target, mesh: &FeMesh) -> Self {
        Self {
            master: RegionDraft::from_region(master, FACE_SOURCES, faces, mesh),
            slave: RegionDraft::from_region(slave, FACE_SOURCES, faces, mesh),
            side: Side::Master,
        }
    }

    /// For contact pairs: the slave may also be whole parts, so that a rope or beam part
    /// touches the master with its nodes (node to surface contact).
    pub fn new_pair(faces: Target) -> Self {
        Self {
            slave: RegionDraft::new(PART_SOURCES, faces),
            ..Self::new(faces)
        }
    }

    /// [`Self::from_regions`] for a contact pair, see [`Self::new_pair`].
    pub fn pair_from_regions(
        master: &Region,
        slave: &Region,
        faces: Target,
        mesh: &FeMesh,
    ) -> Self {
        Self {
            slave: RegionDraft::from_region(slave, PART_SOURCES, faces, mesh),
            ..Self::from_regions(master, slave, faces, mesh)
        }
    }

    /// For contact regions: either side may also be whole parts, a rope or beam part
    /// touching the surface of the other side.
    pub fn new_region(faces: Target) -> Self {
        Self {
            master: RegionDraft::new(PART_SOURCES, faces),
            slave: RegionDraft::new(PART_SOURCES, faces),
            side: Side::Master,
        }
    }

    /// [`Self::from_regions`] for a contact region, see [`Self::new_region`].
    pub fn region_from_regions(a: &Region, b: &Region, faces: Target, mesh: &FeMesh) -> Self {
        Self {
            master: RegionDraft::from_region(a, PART_SOURCES, faces, mesh),
            slave: RegionDraft::from_region(b, PART_SOURCES, faces, mesh),
            side: Side::Master,
        }
    }

    pub fn current(&self) -> &RegionDraft {
        match self.side {
            Side::Master => &self.master,
            Side::Slave => &self.slave,
        }
    }

    pub fn current_mut(&mut self) -> &mut RegionDraft {
        match self.side {
            Side::Master => &mut self.master,
            Side::Slave => &mut self.slave,
        }
    }

    pub fn regions(&self) -> (Region, Region) {
        (self.master.region(), self.slave.region())
    }

    pub fn ui(&mut self, ui: &mut Ui, model: &Model) {
        self.ui_named(ui, model, ["Master", "Slave"]);
    }

    /// The two regions under other names, such as the sides A and B of a contact region,
    /// whose master is derived.
    pub fn ui_named(&mut self, ui: &mut Ui, model: &Model, [first, second]: [&str; 2]) {
        let master = self.side == Side::Master;
        let label = |name: &str| format!("{name} region");
        if (self.master).ui_labeled(ui, model, &label(first), "master", master) {
            self.side = Side::Master;
        }
        if (self.slave).ui_labeled(ui, model, &label(second), "slave", !master) {
            self.side = Side::Slave;
        }
    }

    /// The master in the primary, the slave in the secondary highlight colour.
    pub fn highlight(&self, model: &Model) -> Highlight {
        master_slave_highlight(model, &self.master.region(), &self.slave.region())
    }

    pub fn validate(&self) -> Result<(), String> {
        self.validate_named(["master", "slave"])
    }

    pub fn validate_named(&self, [first, second]: [&str; 2]) -> Result<(), String> {
        if self.master.is_empty() {
            return Err(format!("The {first} region is empty."));
        }
        if self.slave.is_empty() {
            return Err(format!("The {second} region is empty."));
        }
        Ok(())
    }
}

/// Picked faces, a surface or whole parts: the slave side of a contact pair and either side
/// of a contact region, which may be a rope or beam part.
const PART_SOURCES: &[crate::setup::Source] = &[
    crate::setup::Source::Selection,
    crate::setup::Source::Surface,
    crate::setup::Source::Parts,
];

/// Master faces in the primary, slave faces in the secondary highlight colour.
pub fn master_slave_highlight(model: &Model, master: &Region, slave: &Region) -> Highlight {
    let mut highlight = region_highlight(model, master);
    let slave = region_highlight(model, slave);
    highlight.secondary_faces = slave.faces;
    highlight.secondary_lines = slave.lines;
    highlight.nodes.extend(slave.nodes);
    highlight
}

const ADJUST_HELP: &str = "Moves slave nodes onto the master surface before the analysis, so \
    that small gaps and overlaps from meshing do not start the contact with a jump or a \
    force. Only the start geometry changes. Without an adjust distance only nodes that \
    penetrate are moved; with it, all slave nodes within the distance.";

/// A value that may be left to CalculiX: a check box and the number.
fn optional_row(
    ui: &mut Ui,
    label: &str,
    help: &str,
    value: &mut Option<f64>,
    default: f64,
    (units, quantity): (UnitSystem, Quantity),
) {
    let mut set = value.is_some();
    ui.checkbox(&mut set, label).on_hover_text(help);
    let mut number_value = value.unwrap_or(default);
    ui.add_enabled(set, numeric::physical(&mut number_value, units, quantity));
    *value = set.then_some(number_value);
    ui.end_row();
}

/// The properties of a tie below the constraint dialog's type list.
pub fn tie_form(
    ui: &mut Ui,
    model: &Model,
    tie: &mut Tie,
    regions: &mut MasterSlave,
    sectors: &mut CyclicSectors,
) {
    let length = (model.fe.properties.units, Quantity::Length);
    optional_row(
        ui,
        "Position tolerance",
        "Slave nodes farther than this from the master surface are not tied. Untick to \
         let the solver choose a tolerance from the mesh.",
        &mut tie.position_tolerance,
        UnitSystem::MmTonSC.stored(0.05, Quantity::Length),
        length,
    );
    ui.label("");
    ui.checkbox(&mut tie.adjust, "Move slave nodes onto master (Adjust)")
        .on_hover_text(ADJUST_HELP);
    ui.end_row();
    regions.ui(ui, model);
    if !model.fe.properties.space.is_2d() || tie.cyclic.is_some() {
        cyclic_symmetry_rows(ui, model, &mut tie.cyclic);
        if tie.cyclic.is_some() {
            sector_rows(ui, model, tie, regions, sectors);
        }
    }
}

/// The cyclic symmetry tie: the axis about which the sector repeats.
fn cyclic_symmetry_rows(ui: &mut Ui, model: &Model, cyclic: &mut Option<CyclicSymmetry>) {
    ui.label("");
    let mut on = cyclic.is_some();
    ui.checkbox(&mut on, "Cyclic symmetry").on_hover_text(
        "The slave side is the master side rotated by one sector about the axis. Pick the \
         two cut faces of the sector as master and slave.",
    );
    ui.end_row();
    if on != cyclic.is_some() {
        *cyclic = on.then(CyclicSymmetry::default);
    }
    let Some(cyclic) = cyclic else {
        return;
    };
    let units = model.fe.properties.units;
    ui.label(RichText::new("Axis point").strong());
    ui.end_row();
    for (value, label) in cyclic.axis_point.iter_mut().zip(["X", "Y", "Z"]) {
        ui.label(label);
        ui.add(numeric::quantity(value, units, Quantity::Length).speed(1.0));
        ui.end_row();
    }
    ui.label(RichText::new("Axis direction").strong());
    ui.end_row();
    for (value, label) in cyclic.axis_direction.iter_mut().zip(["X", "Y", "Z"]) {
        ui.label(label);
        ui.add(numeric::drag_value(value).speed(0.1));
        ui.end_row();
    }
}

/// The number of sectors found from the cut faces, and what the results show of them.
fn sector_rows(
    ui: &mut Ui,
    model: &Model,
    tie: &mut Tie,
    regions: &MasterSlave,
    sectors: &mut CyclicSectors,
) {
    ui.label("Sectors");
    match sectors.get(model, tie, regions) {
        Some(Ok(n)) => {
            let angle = 360.0 / f64::from(*n);
            ui.label(format!("{n} ({} degrees each)", round_angle(angle)))
                .on_hover_text(
                    "360 degrees over the angle between the cut faces about the axis. The \
                     3D view previews the whole part.",
                );
        }
        Some(Err(problem)) => {
            ui.add(
                egui::Label::new(RichText::new(problem.as_str()).color(crate::style::error()))
                    .wrap(),
            );
        }
        None => {
            ui.label("");
        }
    }
    ui.end_row();
    let Some(cyclic) = &mut tie.cyclic else {
        return;
    };
    ui.label("Results show");
    crate::layout::wrapped_row(ui, |ui| {
        ui.radio_value(&mut cyclic.display, SectorDisplay::Single, "Single sector")
            .on_hover_text("Only the modelled sector (NGRAPH=1).");
        ui.radio_value(&mut cyclic.display, SectorDisplay::All, "All sectors")
            .on_hover_text("The results are expanded to the whole part (NGRAPH=N).");
    });
    ui.end_row();
}

/// A sector angle for the dialog: 30, 22.5 or 51.4286 degrees.
fn round_angle(angle: f64) -> f64 {
    (angle * 1e4).round() / 1e4
}

/// The number of sectors of the cyclic symmetry in a tie dialog, worked out again only when
/// its sides or its axis change.
#[derive(Default)]
pub struct CyclicSectors {
    key: Option<(Region, Region, CyclicSymmetry)>,
    sectors: Option<Result<u32, String>>,
}

impl CyclicSectors {
    /// `None` for a tie without cyclic symmetry.
    pub fn get(
        &mut self,
        model: &Model,
        tie: &Tie,
        regions: &MasterSlave,
    ) -> Option<&Result<u32, String>> {
        let cyclic = tie.cyclic.as_ref()?;
        let (master, slave) = regions.regions();
        let key = (master, slave, cyclic.clone());
        if self.key.as_ref() != Some(&key) {
            let mut sides = Tie::new("");
            (sides.master, sides.slave) = (key.0.clone(), key.1.clone());
            sides.cyclic = Some(key.2.clone());
            self.sectors = sides.cyclic_sectors(&model.mesh);
            self.key = Some(key);
        }
        self.sectors.as_ref()
    }

    /// The whole part the dialog's sector stands for, drawn in the 3D view while it is open.
    pub fn preview(
        &mut self,
        model: &Model,
        tie: &Tie,
        regions: &MasterSlave,
    ) -> Option<CyclicPreview> {
        let sectors = *self.get(model, tie, regions)?.as_ref().ok()?;
        let cyclic = tie.cyclic.as_ref()?;
        Some(CyclicPreview {
            axis_point: cyclic.axis_point,
            axis_direction: cyclic.axis_direction,
            sectors,
        })
    }
}

pub fn contact_pair_form(
    ui: &mut Ui,
    model: &Model,
    pair: &mut ContactPair,
    regions: &mut MasterSlave,
) {
    ui.label("Surface Interaction");
    let interactions = (model.fe.surface_interactions.iter()).map(|i| (i.id, i.name.as_str()));
    let salt = "contact interaction";
    crate::pick_item::item_combo(ui, salt, &mut pair.interaction, interactions, None, 200.0);
    ui.end_row();
    ui.label("Method");
    egui::ComboBox::from_id_salt("contact method")
        .selected_text(pair.method.name())
        .width(crate::dock::fit_width(ui, 200.0))
        .show_ui(ui, |ui| {
            for method in ContactMethod::ALL {
                ui.selectable_value(&mut pair.method, method, method.name())
                    .on_hover_text(method.summary());
            }
        });
    ui.end_row();
    ui.label("");
    ui.add(egui::Label::new(RichText::new(pair.method.description()).weak()).wrap());
    ui.end_row();
    ui.label("");
    let node_to_surface = pair.method == ContactMethod::NodeToSurface;
    ui.add_enabled(
        node_to_surface,
        egui::Checkbox::new(&mut pair.small_sliding, "Small sliding"),
    )
    .on_hover_text(
        "Slave nodes are paired with master faces only at the start of each increment and \
         the pairing is kept during the iterations. Faster and more stable when the surfaces \
         slide little compared with the element size. Node to surface only.",
    );
    if !node_to_surface {
        pair.small_sliding = false;
    }
    ui.end_row();
    ui.label("");
    ui.checkbox(&mut pair.adjust, "Move slave nodes onto master (Adjust)")
        .on_hover_text(ADJUST_HELP);
    ui.end_row();
    if pair.adjust {
        let length = (model.fe.properties.units, Quantity::Length);
        optional_row(
            ui,
            "Adjust distance",
            "Slave nodes within this distance of the master surface are moved onto it. \
             Untick to move only nodes that penetrate the master surface.",
            &mut pair.adjustment_size,
            UnitSystem::MmTonSC.stored(0.01, Quantity::Length),
            length,
        );
    }
    regions.ui(ui, model);
}

pub fn validate_contact_pair(pair: &ContactPair, fe: &FeModel) -> Result<(), String> {
    if pair
        .interaction
        .and_then(|i| fe.surface_interaction(i))
        .is_none()
    {
        return Err(
            "Please select a Surface Interaction; create one under Surface Interactions first."
                .into(),
        );
    }
    Ok(())
}

/// A row's title and description, shown below the property grid.
type Description = (&'static str, &'static str);

/// What the surface interaction dialog shows: the model selected in each list and the
/// focused property.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct InteractionView {
    /// The model selected in the Selected list, whose properties are shown.
    selected: Option<usize>,
    /// The model selected in the Available list, added by the arrow button.
    available: Option<usize>,
    focus: Option<Description>,
}

impl InteractionView {
    pub fn new(interaction: &SurfaceInteraction) -> Self {
        Self {
            selected: (!interaction.properties.is_empty()).then_some(0),
            ..Self::default()
        }
    }
}

/// The "Create surface interaction" dialog: the name, the lists of available and
/// selected interaction models, the properties of the selected model and their description.
pub fn interaction_dialog(
    ui: &mut Ui,
    interaction: &mut SurfaceInteraction,
    view: &mut InteractionView,
    units: UnitSystem,
) {
    // As wide as the title, which grows with the name.
    ui.set_min_width(420.0);
    group(ui, "Data", |ui| {
        let mut focus = None;
        egui::Grid::new("interaction data")
            .max_col_width(crate::dock::column_limit(ui))
            .num_columns(2)
            .min_col_width(110.0)
            .show(ui, |ui| {
                let label = ui.add(egui::Label::new("Name").sense(egui::Sense::click()));
                let edit = ui.add(
                    egui::TextEdit::singleline(&mut interaction.name)
                        .desired_width(ui.available_width()),
                );
                if label.clicked() || edit.has_focus() {
                    focus = Some(("Name", "Name of the Surface Interaction."));
                }
            });
        if focus.is_some() {
            view.focus = focus;
        }
    });
    group(ui, "Interaction Models", |ui| {
        model_lists(ui, interaction, view)
    });
    ui.add_space(4.0);
    ui.strong("Properties");
    let was_tied = interaction.is_tied();
    frame().show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.set_min_height(150.0);
        match view
            .selected
            .and_then(|i| interaction.properties.get_mut(i))
        {
            Some(property) => {
                category(ui, property.name());
                let mut rows = Rows {
                    focus: &mut view.focus,
                    units,
                };
                egui::Grid::new(("interaction properties", property.name()))
                    .num_columns(2)
                    .striped(true)
                    .min_col_width(110.0)
                    .spacing([8.0, 4.0])
                    .show(ui, |ui| match property {
                        InteractionProperty::SurfaceBehavior(b) => rows.surface_behavior(ui, b),
                        InteractionProperty::Friction(f) => rows.friction(ui, f),
                        InteractionProperty::GapConductance(g) => rows.gap_conductance(ui, g),
                    });
            }
            None => {
                ui.weak("A model from the Selected list shows its properties here.");
            }
        }
    });
    // The solver aborts on a tied contact without friction, so choosing Tied brings it along.
    if !was_tied && interaction.add_friction_if_tied() {
        view.focus = Some(TIED_FRICTION);
        view.available = None;
    }
    ui.add_space(4.0);
    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.set_height(60.0);
        let selected = view.selected.and_then(|i| interaction.properties.get(i));
        if let Some((title, text)) = view.focus.or_else(|| selected.map(model_description)) {
            ui.strong(title);
            ui.label(text);
        }
    });
}

/// Why a tied surface behavior comes with friction, shown when the friction is added.
const TIED_FRICTION: Description = (
    "Friction added for Tied",
    "The solver needs Friction with a Tied surface behavior: its stick slope is the stiffness of \
     the tie along the surfaces, the friction coefficient is irrelevant. Tied works only with \
     the method Surface to surface.",
);

/// The Available list, the add and remove buttons and the Selected list.
fn model_lists(ui: &mut Ui, interaction: &mut SurfaceInteraction, view: &mut InteractionView) {
    let height = 72.0;
    let all = InteractionProperty::all();
    let taken = |interaction: &SurfaceInteraction, model: &InteractionProperty| {
        (interaction.properties.iter())
            .any(|p| std::mem::discriminant(p) == std::mem::discriminant(model))
    };
    let mut add = false;
    ui.horizontal_top(|ui| {
        ui.vertical(|ui| {
            ui.label("Available");
            frame().show(ui, |ui| {
                ui.set_width(140.0);
                ui.set_height(height);
                for (i, model) in all.iter().enumerate() {
                    let used = taken(interaction, model);
                    let selected = view.available == Some(i);
                    let row = ui
                        .add_enabled_ui(!used, |ui| {
                            tree_row(ui, 0, None, selected, RichText::new(model.name()))
                        })
                        .inner;
                    if row.clicked() {
                        view.available = Some(i);
                        view.focus = None;
                    }
                    if row.double_clicked_tolerant() {
                        add = true;
                    }
                }
            });
        });
        ui.vertical(|ui| {
            ui.add_space(22.0);
            let can_add = view
                .available
                .and_then(|i| all.get(i))
                .is_some_and(|model| !taken(interaction, model));
            let arrow = Icon::Arrow(egui::vec2(1.0, 0.0));
            if icons::dialog_button(ui, arrow, "Add", can_add).clicked() {
                add = true;
            }
            let can_remove = view.selected.is_some();
            if icons::dialog_button(ui, Icon::Remove, "Remove", can_remove).clicked()
                && let Some(index) = view.selected
            {
                interaction.properties.remove(index);
                let len = interaction.properties.len();
                view.selected = (len > 0).then(|| index.min(len - 1));
                view.focus = None;
            }
        });
        if add
            && let Some(model) = view.available.and_then(|i| all.get(i))
            && !taken(interaction, model)
        {
            interaction.properties.push(model.clone());
            view.selected = Some(interaction.properties.len() - 1);
            // The next model still available is ready for the next click on the arrow.
            view.available = (all.iter()).position(|model| !taken(interaction, model));
            view.focus = None;
        }
        ui.vertical(|ui| {
            ui.label("Selected");
            frame().show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.set_height(height);
                for (i, property) in interaction.properties.iter().enumerate() {
                    let selected = view.selected == Some(i);
                    if tree_row(ui, 0, None, selected, RichText::new(property.name())).clicked() {
                        view.selected = Some(i);
                        view.focus = None;
                    }
                }
            });
        });
    });
}

fn model_description(property: &InteractionProperty) -> Description {
    match property {
        InteractionProperty::SurfaceBehavior(_) => (
            "Surface Behavior",
            "Contact pressure as a function of the overclosure (penetration) of the surfaces. \
             Hard is the usual rigid contact; the other models \
             define a soft contact with your own curve.",
        ),
        InteractionProperty::Friction(_) => (
            "Friction",
            "Coulomb friction: the tangential force is limited to the friction coefficient \
             times the contact pressure. Adds nonlinearity; use small coefficients first \
             and enough increments.",
        ),
        InteractionProperty::GapConductance(_) => (
            "Gap Conductance",
            "Heat conduction across the gap between the contact surfaces, for thermal and \
             coupled steps. Without it the contact is thermally insulating.",
        ),
    }
}

/// Width of the number fields of the property grid.
fn field_size(ui: &Ui) -> egui::Vec2 {
    egui::vec2(120.0, ui.spacing().interact_size.y)
}

/// Category header of the property grid.
fn category(ui: &mut Ui, title: &str) {
    let header = egui::Frame::new()
        .fill(crate::style::control())
        .inner_margin(egui::Margin::symmetric(4, 1));
    header.show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.strong(title);
    });
}

/// A titled frame like a Windows group box.
fn group(ui: &mut Ui, title: &str, contents: impl FnOnce(&mut Ui)) {
    ui.add_space(4.0);
    ui.strong(title);
    egui::Frame::new()
        .stroke(egui::Stroke::new(1.0, crate::style::border()))
        .inner_margin(6)
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            contents(ui);
        });
}

/// Rows of the property grid; a click on a row or its editor shows its description.
struct Rows<'a> {
    focus: &'a mut Option<Description>,
    units: UnitSystem,
}

impl Rows<'_> {
    fn row(
        &mut self,
        ui: &mut Ui,
        description: Description,
        editor: impl FnOnce(&mut Ui) -> egui::Response,
    ) {
        let label = ui.add(egui::Label::new(description.0).sense(egui::Sense::click()));
        let response = editor(ui);
        if label.clicked() || response.clicked() || response.has_focus() || response.changed() {
            *self.focus = Some(description);
        }
        ui.end_row();
    }

    fn value(&mut self, ui: &mut Ui, description: Description, value: &mut f64, of: Quantity) {
        let units = self.units;
        self.row(ui, description, |ui| {
            ui.add_sized(field_size(ui), numeric::physical(value, units, of))
        });
    }

    /// A value the solver may leave out; the checkbox writes it.
    fn optional(
        &mut self,
        ui: &mut Ui,
        description: Description,
        value: &mut Option<f64>,
        default: f64,
        of: Quantity,
    ) {
        let units = self.units;
        let mut set = value.is_some();
        let mut number_value = value.unwrap_or(default);
        let label = ui.checkbox(&mut set, description.0);
        let size = field_size(ui);
        let response = ui
            .add_enabled_ui(set, |ui| {
                ui.add_sized(size, numeric::physical(&mut number_value, units, of))
            })
            .inner;
        if label.changed() || response.has_focus() || response.changed() {
            *self.focus = Some(description);
        }
        *value = set.then_some(number_value);
        ui.end_row();
    }

    fn surface_behavior(&mut self, ui: &mut Ui, behavior: &mut SurfaceBehavior) {
        let description = (
            "Pressure-overclosure",
            "Hard: practically no penetration, the usual choice; in penalty contact a linear \
             spring K = 10 E/h (E the softer material, h the slave element size). \
             Linear: pressure grows linearly with \
             the overclosure, a soft penalty contact. Exponential: pressure rises \
             exponentially as the gap closes, which helps convergence from open contact. \
             Tabular: your own curve. Tied: the surfaces stay connected with the stiffness K \
             (see also Ties); needs Friction and the method Surface to surface.",
        );
        self.row(ui, description, |ui| {
            egui::ComboBox::from_id_salt("pressure overclosure")
                .selected_text(behavior.keyword())
                .show_ui(ui, |ui| {
                    for kind in SurfaceBehavior::kinds() {
                        let same =
                            std::mem::discriminant(&kind) == std::mem::discriminant(behavior);
                        if ui.selectable_label(same, kind.keyword()).clicked() && !same {
                            // Switching the kind starts with its default values.
                            *behavior = kind;
                        }
                    }
                })
                .response
        });
        match behavior {
            SurfaceBehavior::Hard => {}
            SurfaceBehavior::Linear { k, sigma_inf, c0 } => {
                let k_text =
                    "Slope of the curve, about 5 to 50 times the Young's modulus per length.";
                self.value(ui, ("K", k_text), k, Quantity::ForcePerVolume);
                let sigma_text = "Tensile stress at a large gap, about 0.25 % of the largest \
                                  expected equivalent stress.";
                self.value(
                    ui,
                    ("Sigma infinity", sigma_text),
                    sigma_inf,
                    Quantity::Pressure,
                );
                let c0_text = "Optional parameter c0 of the linear curve, see \
                               *SURFACE BEHAVIOR in the CalculiX manual.";
                let c0_default = UnitSystem::MmTonSC.stored(1.0, Quantity::Length);
                self.optional(ui, ("c0", c0_text), c0, c0_default, Quantity::Length);
            }
            SurfaceBehavior::Exponential { c0, p0 } => {
                let c0_text = "Gap at which the contact pressure has dropped to 1 % of p0.";
                self.value(ui, ("c0", c0_text), c0, Quantity::Length);
                self.value(
                    ui,
                    ("p0", "Contact pressure at zero gap."),
                    p0,
                    Quantity::Pressure,
                );
            }
            SurfaceBehavior::Tabular(rows) => {
                let text = "One contact pressure and its overclosure per row.";
                self.table(
                    ui,
                    ("Table", text),
                    rows,
                    [
                        ("Pressure", Quantity::Pressure),
                        ("Overclosure", Quantity::Length),
                    ],
                );
            }
            SurfaceBehavior::Tied { k } => {
                self.value(
                    ui,
                    ("K", "Stiffness of the connection."),
                    k,
                    Quantity::ForcePerVolume,
                );
                ui.label("");
                ui.weak("Needs Friction (stick slope)\nand Surface to surface pairs.");
                ui.end_row();
            }
        }
    }

    fn friction(&mut self, ui: &mut Ui, friction: &mut Friction) {
        let text = "Friction coefficient mu, greater than zero.";
        self.row(ui, ("Friction coefficient", text), |ui| {
            let value = numeric::drag_value(&mut friction.coefficient)
                .range(0.0..=f64::MAX)
                .speed(0.01);
            ui.add_sized(field_size(ui), value)
        });
        let text = "Slope lambda of the shear stress over the slip in the stick range; \
                    if empty, 1/100 of the normal contact stiffness, or all of it for a \
                    Tied surface behavior.";
        self.optional(
            ui,
            ("Stick slope", text),
            &mut friction.stick_slope,
            UnitSystem::MmTonSC.stored(1e5, Quantity::ForcePerVolume),
            Quantity::ForcePerVolume,
        );
    }

    fn gap_conductance(&mut self, ui: &mut Ui, conductance: &mut GapConductance) {
        let text = "Constant conductance or table over pressure and temperature.";
        self.row(ui, ("Kind", text), |ui| {
            crate::layout::wrapped_row(ui, |ui| {
                let constant = matches!(conductance, GapConductance::Constant(_));
                let mut response = ui.radio(constant, "Constant");
                if response.clicked() && !constant {
                    *conductance = GapConductance::Constant(0.0);
                }
                let table = ui.radio(!constant, "Table");
                if table.clicked() && constant {
                    *conductance = GapConductance::Tabular(vec![[0.0; 3]]);
                }
                response |= table;
                response
            })
            .inner
        });
        match conductance {
            GapConductance::Constant(value) => {
                let text = "Heat flux per area and temperature difference across the gap.";
                self.value(
                    ui,
                    ("Conductance", text),
                    value,
                    Quantity::HeatTransferCoefficient,
                );
            }
            GapConductance::Tabular(rows) => {
                let text = "One conductance per row with the contact pressure and the temperature \
                            it applies to.";
                self.table(
                    ui,
                    ("Table", text),
                    rows,
                    [
                        ("Conductance", Quantity::HeatTransferCoefficient),
                        ("Pressure", Quantity::Pressure),
                        ("Temperature", Quantity::Temperature),
                    ],
                );
            }
        }
    }

    /// A small editable table with a row to add and buttons to remove rows.
    fn table<const N: usize>(
        &mut self,
        ui: &mut Ui,
        description: Description,
        rows: &mut Vec<[f64; N]>,
        columns: [(&str, Quantity); N],
    ) {
        let units = self.units;
        self.row(ui, description, |ui| {
            ui.vertical(|ui| {
                let mut response = ui.response();
                egui::Grid::new(("interaction table", description.1)).show(ui, |ui| {
                    for (title, quantity) in columns {
                        let unit = units.unit(quantity);
                        if unit.is_empty() {
                            ui.strong(title);
                        } else {
                            ui.strong(format!("{title} [{unit}]"));
                        }
                    }
                    ui.end_row();
                    let mut remove = None;
                    let rows_len = rows.len();
                    for (i, row) in rows.iter_mut().enumerate() {
                        for (value, (_, quantity)) in row.iter_mut().zip(columns) {
                            // The unit stands in the column title.
                            let field = numeric::without_unit(value, units, quantity)
                                .speed(0.0)
                                .custom_formatter(|v, _| numeric::format_physical(v));
                            response |= ui.add(field);
                        }
                        if ui
                            .add_enabled(rows_len > 1, egui::Button::new("x").small())
                            .on_hover_text("Remove row")
                            .clicked()
                        {
                            remove = Some(i);
                        }
                        ui.end_row();
                    }
                    if let Some(i) = remove.filter(|_| rows.len() > 1) {
                        rows.remove(i);
                    }
                });
                if ui.small_button("Add Row").clicked() {
                    let last = rows.last().copied().unwrap_or([0.0; N]);
                    rows.push(last);
                }
                response
            })
            .inner
        });
    }
}

pub fn validate_interaction(interaction: &SurfaceInteraction) -> Result<(), String> {
    if interaction.properties.is_empty() {
        return Err("Please add at least one Interaction Model.".into());
    }
    for property in &interaction.properties {
        if let InteractionProperty::Friction(friction) = property
            && friction.coefficient <= 0.0
        {
            return Err("The friction coefficient must be greater than 0.".into());
        }
    }
    if interaction.is_tied() && !interaction.has_friction() {
        return Err(
            "A Tied surface behavior needs Friction: the solver takes the stiffness of \
                    the tie along the surfaces from its stick slope."
                .into(),
        );
    }
    Ok(())
}
