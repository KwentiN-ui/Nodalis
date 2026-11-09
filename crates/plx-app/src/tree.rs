//! The model tree in the three views Geometry, FE Model and Results, with their
//! node names. Nodes for features nodalis does not support yet are shown as empty
//! placeholders, so that the structure is already the familiar one.

use std::collections::{BTreeSet, HashMap, HashSet};

use egui::collapsing_header::CollapsingState;
use egui::epaint::Mesh;
use egui::{Color32, Pos2, Rect, Response, Shape, Ui, Vec2, WidgetText, pos2, vec2};
use plx_model::{FeModel, Finding, ModelItem, SetKind, Severity};

use crate::features::{FeatureItem, FeatureKind};
use crate::model::{Model, PartInfo};
use crate::setup::{ContactItem, NewItem};
use crate::tree_icons::{self, StepGlyph, TreeIcon};

/// Which of the three trees is shown.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TreeView {
    Geometry,
    FeModel,
    Results,
}

/// A node of a tree; placeholders and group nodes are identified by their name.
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum TreeItem {
    Group(&'static str),
    Model,
    Mesh,
    Part(usize),
    /// An item of the geometry's mesh setup.
    MeshItem(usize),
    NodeSet(String),
    ElementSet(String),
    Surface(String),
    Material(usize),
    Section(usize),
    /// A beam profile, under Sections.
    CrossSection(usize),
    Constraint(usize),
    SurfaceInteraction(usize),
    /// Two surfaces in contact by the behavior the user wants, under Connections.
    ContactRegion(usize),
    /// A search for contact regions under Connections, holding the regions it found.
    ContactDetection(usize),
    /// A bolt preloaded across a cut, under Connections.
    Bolt(usize),
    ContactPair(usize),
    /// A tie of two surfaces, listed with the contact pairs.
    Tie(usize),
    /// A node tie, listed with the contact pairs.
    NodeTie(usize),
    Amplitude(usize),
    Distribution(usize),
    InitialCondition(usize),
    Step(usize),
    /// A container inside a step, such as "BCs".
    StepGroup(usize, &'static str),
    BoundaryCondition(usize, usize),
    Load(usize, usize),
    FieldOutput(usize, usize),
    /// A history output of a step, by step and index.
    HistoryOutput(usize, usize),
    /// A defined field of a step, by step and index.
    DefinedField(usize, usize),
    /// Features of the FE model or of the shown results, by index.
    ReferencePoint(usize),
    CoordinateSystem(usize),
    Plane(usize),
    /// A result path of the shown results, by index.
    ResultPath(usize),
    /// Results on a plane of the shown results, by index.
    ResultPlane(usize),
    FieldOutputs,
    /// Field of the current increment, by index.
    Field(usize),
    /// Field of the current increment computed from a derived field output, by index.
    ResultFieldOutput(usize),
    /// A computed history output, by index of its data.
    HistorySet(usize),
    HistoryField(usize, usize),
    HistoryComponent(usize, usize, usize),
    Component(usize, usize),
    /// A hot spot definition of the current results, by index.
    HotSpot(usize),
}

/// The group of the exposed parameters in the FE Model tree.
pub const EXPOSED: &str = "Exposed Parameters";

/// The tree item of an item of the model a value path names.
pub fn item_of(fe: &FeModel, item: &plx_model::ItemRef) -> Option<TreeItem> {
    use plx_model::ItemRef;
    let step = |name: &str| fe.steps.iter().position(|s| s.name == name);
    Some(match item {
        ItemRef::Load { step: s, name } => {
            let s = step(s)?;
            TreeItem::Load(s, fe.steps[s].loads.iter().position(|l| &l.name == name)?)
        }
        ItemRef::BoundaryCondition { step: s, name } => {
            let s = step(s)?;
            let list = &fe.steps[s].boundary_conditions;
            TreeItem::BoundaryCondition(s, list.iter().position(|b| &b.name == name)?)
        }
        ItemRef::DefinedField { step: s, name } => {
            let s = step(s)?;
            let list = &fe.steps[s].defined_fields;
            TreeItem::DefinedField(s, list.iter().position(|d| &d.name == name)?)
        }
        ItemRef::Material(name) => {
            TreeItem::Material(fe.materials.iter().position(|m| &m.name == name)?)
        }
        ItemRef::Section(name) => {
            TreeItem::Section(fe.sections.iter().position(|s| &s.name == name)?)
        }
        ItemRef::CrossSection(name) => {
            TreeItem::CrossSection(fe.cross_sections.iter().position(|c| &c.name == name)?)
        }
        ItemRef::Step(name) => TreeItem::Step(step(name)?),
        ItemRef::InitialCondition(name) => TreeItem::InitialCondition(
            (fe.initial_conditions.iter()).position(|c| &c.name == name)?,
        ),
    })
}

/// Selection shared by the three trees; an item is selected in one view only.
#[derive(Default)]
pub struct TreeState {
    pub selected: Option<(TreeView, TreeItem)>,
    /// The step new items go into, marked "(active)" when there are several.
    pub active_step: Option<usize>,
    /// The last click on a row: its item and the frame time, for `Tree::double_click`.
    last_click: Option<(TreeView, TreeItem, f64)>,
    /// Expand (true) or collapse an item with all its descendants in the next frame.
    expand: Option<(TreeView, TreeItem, bool)>,
    /// The selection was made in the 3D view: open its branches and scroll it into view.
    pub reveal: bool,
    /// Parts selected along with the selected part, by Ctrl or Shift click;
    /// they count only while a part of the same view is selected.
    more_parts: BTreeSet<usize>,
    /// Where the range of a Shift click starts: the part last clicked without Shift.
    anchor: Option<usize>,
    /// Items selected along with the selected item, by Ctrl or Shift click: siblings in one
    /// container, such as many contact pairs to delete at once. They count only while the
    /// item they were selected with, the first, stays selected.
    more_items: Option<(TreeItem, Vec<TreeItem>)>,
    /// Where the range of a Shift click on items starts.
    item_anchor: Option<TreeItem>,
    /// Items another analysis of the project owns, with its name: they are marked and
    /// cannot be changed here.
    pub linked: HashMap<TreeItem, String>,
    /// Leave the linked items out of the FE Model tree; categories stay, so that items of
    /// the analysis itself can be added to them.
    pub hide_linked: bool,
    /// What acts on the parts selected in the FE Model tree, such as their contact pairs and
    /// loads: marked in the tree. Kept up to date by the app for the parts and the model
    /// revision in `acting_for`.
    pub acting: BTreeSet<TreeItem>,
    pub acting_for: Option<(BTreeSet<usize>, u64)>,
    /// The parts were clicked in the 3D view: open the branches of what acts on them in
    /// the next frame, while the tree stays where it is.
    pub reveal_acting: bool,
}

impl TreeState {
    /// The parts selected in `view`, in ascending order; none unless a part is selected
    /// there.
    pub fn selected_parts(&self, view: TreeView) -> BTreeSet<usize> {
        match &self.selected {
            Some((selected, TreeItem::Part(index))) if *selected == view => {
                let mut parts = self.more_parts.clone();
                parts.insert(*index);
                parts
            }
            _ => BTreeSet::new(),
        }
    }

    /// The items selected in `view`: the selected one first, then those selected along
    /// with it. Parts are in [`Self::selected_parts`].
    pub fn selected_items(&self, view: TreeView) -> Vec<TreeItem> {
        let Some((selected, lead)) = &self.selected else {
            return Vec::new();
        };
        if *selected != view {
            return Vec::new();
        }
        let mut items = vec![lead.clone()];
        if let Some((owner, more)) = &self.more_items
            && owner == lead
        {
            items.extend(more.iter().cloned());
        }
        items
    }

    /// Selects one item alone, by a plain click.
    pub fn select_item(&mut self, view: TreeView, item: TreeItem) {
        self.selected = Some((view, item.clone()));
        self.more_items = None;
        self.item_anchor = Some(item);
    }

    /// Ctrl click (adds or takes out `item`) or Shift click (the items from the last one
    /// clicked to `item`, added with Ctrl) on an item of a container listing `siblings`.
    pub fn select_sibling(
        &mut self,
        view: TreeView,
        item: TreeItem,
        siblings: &[TreeItem],
        ctrl: bool,
        shift: bool,
    ) {
        let current: Vec<TreeItem> = (self.selected_items(view).into_iter())
            .filter(|i| siblings.contains(i))
            .collect();
        let mut chosen = if shift {
            let anchor = (self.item_anchor.clone())
                .filter(|a| siblings.contains(a) && !current.is_empty())
                .unwrap_or_else(|| item.clone());
            let position = |i: &TreeItem| siblings.iter().position(|s| s == i).unwrap_or(0);
            let (a, b) = (position(&anchor), position(&item));
            let mut chosen = if ctrl { current } else { Vec::new() };
            for sibling in &siblings[a.min(b)..=a.max(b)] {
                if !chosen.contains(sibling) {
                    chosen.push(sibling.clone());
                }
            }
            self.item_anchor = Some(anchor);
            chosen
        } else {
            let mut chosen = current;
            match chosen.iter().position(|i| *i == item) {
                Some(index) => {
                    chosen.remove(index);
                }
                None => chosen.push(item.clone()),
            }
            self.item_anchor = Some(item.clone());
            chosen
        };
        let lead = if chosen.contains(&item) {
            item
        } else if let Some(last) = chosen.last() {
            last.clone()
        } else {
            self.selected = None;
            self.more_items = None;
            return;
        };
        chosen.retain(|i| *i != lead);
        self.selected = Some((view, lead.clone()));
        self.more_items = Some((lead, chosen));
    }

    /// Selects one part alone, by a plain click.
    pub fn select_part(&mut self, view: TreeView, index: usize) {
        self.select_parts(view, BTreeSet::from([index]), index);
        self.anchor = Some(index);
    }

    /// Ctrl click: adds a part to the selection, or takes it out when it is selected.
    pub fn toggle_part(&mut self, view: TreeView, index: usize) {
        let mut parts = self.selected_parts(view);
        if !parts.remove(&index) {
            parts.insert(index);
        }
        self.select_parts(view, parts, index);
        self.anchor = Some(index);
    }

    /// Adds parts to the selection; the last becomes the selected one.
    pub fn add_parts(&mut self, view: TreeView, added: impl IntoIterator<Item = usize>) {
        let mut parts = self.selected_parts(view);
        let mut lead = None;
        for index in added {
            parts.insert(index);
            lead = Some(index);
        }
        if let Some(lead) = lead {
            self.select_parts(view, parts, lead);
        }
    }

    /// Shift click in the tree: the parts from the anchor to `index`, added to the selection
    /// with Ctrl.
    pub fn select_range(&mut self, view: TreeView, index: usize, add: bool) {
        let anchor = (self.anchor)
            .filter(|_| !self.selected_parts(view).is_empty())
            .unwrap_or(index);
        let range = anchor.min(index)..=anchor.max(index);
        let mut parts = if add {
            self.selected_parts(view)
        } else {
            BTreeSet::new()
        };
        parts.extend(range);
        self.select_parts(view, parts, index);
        self.anchor = Some(anchor);
    }

    /// Selects `parts` with `lead` as the selected item, or the last part when `lead` is not
    /// among them; nothing when no part is left.
    fn select_parts(&mut self, view: TreeView, mut parts: BTreeSet<usize>, lead: usize) {
        let lead = if parts.contains(&lead) {
            Some(lead)
        } else {
            parts.last().copied()
        };
        match lead {
            Some(lead) => {
                parts.remove(&lead);
                self.selected = Some((view, TreeItem::Part(lead)));
                self.more_parts = parts;
            }
            None => {
                self.selected = None;
                self.more_parts.clear();
            }
        }
    }
}

/// What the user did in the tree this frame.
#[derive(Default)]
pub struct TreeResponse {
    pub visibility: Vec<(usize, bool)>,
    /// Colour contours of results parts turned on (`true`) or off.
    pub contours: Vec<(usize, bool)>,
    /// A result component was picked (field, component).
    pub component: Option<(usize, usize)>,
    /// An item was double-clicked: show its properties.
    pub open: Option<TreeItem>,
    /// Create a new item, such as a contact output from the menu of a contact.
    pub create: Option<NewItem>,
    /// Delete an item, by its context menu or the Delete key; the app asks first.
    pub delete: Option<TreeItem>,
    /// Activate a deactivated step, boundary condition or load, or deactivate an active one.
    pub toggle_active: Option<TreeItem>,
    /// Move a step one place up (`true`) or down in the list of steps.
    pub move_step: Option<(usize, bool)>,
    /// Swap master and slave of a tie, spring connection or contact pair.
    pub swap_master_slave: Option<TreeItem>,
    /// Hide (`false`) or show the symbols of constraints, boundary conditions or loads.
    pub symbol_visibility: Vec<(TreeItem, bool)>,
    /// Mesh one part of the geometry, by index.
    pub mesh_part: Option<usize>,
    /// Transform a mesh part (translate, rotate, mirror, scale), by index.
    pub transform_part: Option<usize>,
    /// Merge the selected mesh parts into one.
    pub merge_parts: bool,
    /// Run this contact detection again.
    pub detect: Option<usize>,
    /// The warning sign of an item was clicked: explain its findings.
    pub findings: Option<Vec<Finding>>,
    /// Show the results on a plane, by index, or none.
    pub plane_result: Option<Option<usize>>,
}

/// Tree label with a fixed size: highlight and hover frame are painted over the same area, so
/// that hovering never moves the rows below (egui's selectable label grows by its frame).
/// The tinted frame behind the label of an item that acts on the selected part.
fn acting_mark(rect: Rect) -> Shape {
    let color = crate::style::highlight();
    let fill = Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), 48);
    let stroke = egui::Stroke::new(1.0, color);
    Shape::Rect(egui::epaint::RectShape::new(
        rect,
        0.0,
        fill,
        stroke,
        egui::StrokeKind::Inside,
    ))
}

/// Longest time between the two clicks of a double click in the tree.
const DOUBLE_CLICK_SECONDS: f64 = 0.8;

pub(crate) fn row_label(
    ui: &mut Ui,
    selected: bool,
    color: Option<Color32>,
    text: impl Into<WidgetText>,
) -> Response {
    let padding = egui::vec2(3.0, 1.0);
    let mut text = text.into();
    if let Some(color) = color.filter(|_| !selected) {
        text = text.color(color);
    }
    let galley = text.into_galley(
        ui,
        Some(egui::TextWrapMode::Extend),
        f32::INFINITY,
        egui::TextStyle::Body,
    );
    let (rect, response) =
        ui.allocate_exact_size(galley.size() + 2.0 * padding, egui::Sense::click());
    if ui.is_rect_visible(rect) {
        let visuals = ui.visuals();
        let text_color = if selected {
            visuals.selection.stroke.color
        } else {
            visuals.text_color()
        };
        if selected {
            ui.painter()
                .rect_filled(rect, 0.0, visuals.selection.bg_fill);
        } else if response.hovered() {
            ui.painter().rect(
                rect,
                0.0,
                crate::style::hover_fill(),
                egui::Stroke::new(1.0, crate::style::highlight()),
                egui::StrokeKind::Inside,
            );
        }
        ui.painter().galley(rect.min + padding, galley, text_color);
    }
    response
}

/// Text colour of invalid items and of the containers holding them, Windows' red.
const INVALID: Color32 = Color32::from_rgb(255, 0, 0);

/// Text colour of deactivated items.
const INACTIVE: Color32 = Color32::from_rgb(128, 128, 128);

/// Items that can be deactivated, as far as Nodalis has them.
/// Materials and surface interactions cannot be deactivated yet.
fn can_deactivate(item: &TreeItem) -> bool {
    matches!(
        item,
        TreeItem::Step(_)
            | TreeItem::BoundaryCondition(..)
            | TreeItem::Load(..)
            | TreeItem::FieldOutput(..)
            | TreeItem::HistoryOutput(..)
            | TreeItem::DefinedField(..)
            | TreeItem::Constraint(_)
            | TreeItem::ContactRegion(_)
            | TreeItem::Bolt(_)
            | TreeItem::ContactPair(_)
            | TreeItem::Tie(_)
            | TreeItem::NodeTie(_)
            | TreeItem::InitialCondition(_)
    )
}

/// Warning sign next to an item with findings; a click explains them.
/// The mark of an item another analysis owns: two chain links.
fn link_sign(ui: &mut Ui) -> Response {
    ui.add_space(3.0);
    let (rect, response) =
        ui.allocate_exact_size(Vec2::splat(tree_icons::SIZE), egui::Sense::hover());
    if ui.is_rect_visible(rect) {
        let color = crate::style::adapt(Color32::from_rgb(40, 110, 200));
        let stroke = egui::Stroke::new(1.6, color);
        let c = rect.center();
        for offset in [-2.6, 2.6] {
            let link = egui::Rect::from_center_size(
                c + egui::vec2(offset, -offset * 0.8),
                egui::vec2(7.5, 5.0),
            );
            ui.painter()
                .rect_stroke(link, 2.5, stroke, egui::StrokeKind::Middle);
        }
    }
    response
}

fn warning_sign(ui: &mut Ui) -> Response {
    ui.add_space(3.0);
    let (rect, response) =
        ui.allocate_exact_size(Vec2::splat(tree_icons::SIZE), egui::Sense::click());
    if ui.is_rect_visible(rect) {
        tree_icons::paint(ui.painter(), rect.min, TreeIcon::Warning);
    }
    response
}

/// Tree item of an item of the FE model and the containers it is shown in, innermost first.
pub(crate) fn tree_items(item: ModelItem) -> (TreeItem, Vec<TreeItem>) {
    let step = |s: usize, group: &'static str| {
        vec![
            TreeItem::StepGroup(s, group),
            TreeItem::Step(s),
            TreeItem::Group("Steps"),
            TreeItem::Model,
        ]
    };
    match item {
        ModelItem::Section(i) => (
            TreeItem::Section(i),
            vec![TreeItem::Group("Sections"), TreeItem::Model],
        ),
        ModelItem::CrossSection(i) => (
            TreeItem::CrossSection(i),
            vec![
                TreeItem::Group(CROSS_SECTIONS),
                TreeItem::Group("Sections"),
                TreeItem::Model,
            ],
        ),
        ModelItem::Constraint(i) => (
            TreeItem::Constraint(i),
            vec![TreeItem::Group("Constraints"), TreeItem::Model],
        ),
        ModelItem::ContactPair(i) => (TreeItem::ContactPair(i), advanced_contacts()),
        ModelItem::Tie(i) => (TreeItem::Tie(i), advanced_contacts()),
        ModelItem::NodeTie(i) => (TreeItem::NodeTie(i), advanced_contacts()),
        ModelItem::ContactRegion(i) => (
            TreeItem::ContactRegion(i),
            vec![
                TreeItem::Group(CONTACT_REGIONS),
                TreeItem::Group("Contacts"),
                TreeItem::Model,
            ],
        ),
        ModelItem::Bolt(i) => (
            TreeItem::Bolt(i),
            vec![
                TreeItem::Group(BOLTS),
                TreeItem::Group("Contacts"),
                TreeItem::Model,
            ],
        ),
        ModelItem::BoundaryCondition(s, i) => (TreeItem::BoundaryCondition(s, i), step(s, "BCs")),
        ModelItem::Load(s, i) => (TreeItem::Load(s, i), step(s, "Loads")),
        ModelItem::HistoryOutput(s, i) => {
            (TreeItem::HistoryOutput(s, i), step(s, "History Outputs"))
        }
        ModelItem::DefinedField(s, i) => (TreeItem::DefinedField(s, i), step(s, "Defined Fields")),
        ModelItem::Amplitude(i) => (
            TreeItem::Amplitude(i),
            vec![TreeItem::Group("Amplitudes"), TreeItem::Model],
        ),
        ModelItem::Distribution(i) => (
            TreeItem::Distribution(i),
            vec![TreeItem::Group("Distributions"), TreeItem::Model],
        ),
        ModelItem::Material(i) => (
            TreeItem::Material(i),
            vec![TreeItem::Group("Materials"), TreeItem::Model],
        ),
        ModelItem::Part(i) => (
            TreeItem::Part(i),
            vec![TreeItem::Group("Parts"), TreeItem::Mesh, TreeItem::Model],
        ),
        ModelItem::Step(s) => (
            TreeItem::Step(s),
            vec![TreeItem::Group("Steps"), TreeItem::Model],
        ),
        ModelItem::BoundaryConditions(s) => {
            let mut containers = step(s, "BCs");
            containers.remove(0);
            (TreeItem::StepGroup(s, "BCs"), containers)
        }
        // The analysis has no item; what the solver reported shows on the model.
        ModelItem::Analysis => (TreeItem::Model, vec![]),
        ModelItem::InitialCondition(i) => (
            TreeItem::InitialCondition(i),
            vec![TreeItem::Group("Initial Conditions"), TreeItem::Model],
        ),
    }
}

/// The containers of contact pairs, ties and node ties, innermost first.
fn advanced_contacts() -> Vec<TreeItem> {
    vec![
        TreeItem::Group("Contact Pairs"),
        TreeItem::Group(ADVANCED_CONTACTS),
        TreeItem::Group("Contacts"),
        TreeItem::Model,
    ]
}

/// The container of the cross-sections, under Sections.
pub const CROSS_SECTIONS: &str = "Cross-Sections";
/// The container of the contact regions, under Connections (the group "Contacts").
pub const CONTACT_REGIONS: &str = "Contact Regions";
/// The container of the bolts, under Connections.
pub const BOLTS: &str = "Bolts";
/// The container of the keyword-near contact objects under Connections: surface
/// interactions, contact pairs, ties and node ties.
pub const ADVANCED_CONTACTS: &str = "Advanced";

/// What double-clicking a container creates.
fn creates(item: &TreeItem) -> Option<NewItem> {
    match *item {
        TreeItem::Group("Materials") => Some(NewItem::Material),
        TreeItem::Group("Sections") => Some(NewItem::Section),
        TreeItem::Group(CROSS_SECTIONS) => Some(NewItem::CrossSection),
        TreeItem::Group("Steps") => Some(NewItem::Step),
        TreeItem::Group("Constraints") => Some(NewItem::Constraint),
        TreeItem::Group("Surface Interactions") => Some(NewItem::SurfaceInteraction),
        TreeItem::Group("Contact Pairs") => Some(NewItem::ContactPair),
        TreeItem::Group(CONTACT_REGIONS) => Some(NewItem::ContactRegion),
        TreeItem::Group(BOLTS) => Some(NewItem::Bolt),
        TreeItem::Group("Amplitudes") => Some(NewItem::Amplitude),
        TreeItem::Group("Distributions") => Some(NewItem::Distribution),
        TreeItem::Group("Initial Conditions") => Some(NewItem::InitialCondition),
        TreeItem::StepGroup(step, "BCs") => Some(NewItem::BoundaryCondition(step)),
        TreeItem::StepGroup(step, "Loads") => Some(NewItem::Load(step)),
        TreeItem::StepGroup(step, "Field Outputs") => Some(NewItem::FieldOutput(step)),
        TreeItem::StepGroup(step, "History Outputs") => Some(NewItem::HistoryOutput(step)),
        TreeItem::StepGroup(step, "Defined Fields") => Some(NewItem::DefinedField(step)),
        TreeItem::Group(HOT_SPOTS) => Some(NewItem::ResultHotSpot),
        TreeItem::Group(REFERENCE_POINTS) => Some(NewItem::Feature(FeatureKind::ReferencePoint)),
        TreeItem::Group(COORDINATE_SYSTEMS) => {
            Some(NewItem::Feature(FeatureKind::CoordinateSystem))
        }
        TreeItem::Group(PLANES) => Some(NewItem::Feature(FeatureKind::Plane)),
        TreeItem::Group(PATHS) => Some(NewItem::Feature(FeatureKind::ResultPath)),
        TreeItem::Group(PLANE_RESULTS) => Some(NewItem::Feature(FeatureKind::ResultPlane)),
        TreeItem::Group("Mesh Setup") => Some(NewItem::MeshSetupItem),
        TreeItem::FieldOutputs => Some(NewItem::ResultFieldOutput),
        TreeItem::Group("History Outputs") => Some(NewItem::ResultHistoryOutput),
        _ => None,
    }
}

/// What double-clicking a container of the mesh's sets creates; in the FE Model tree only.
fn creates_set(item: &TreeItem) -> Option<NewItem> {
    match *item {
        TreeItem::Group("Node Sets") => Some(NewItem::MeshSet(SetKind::Nodes)),
        TreeItem::Group("Element Sets") => Some(NewItem::MeshSet(SetKind::Elements)),
        TreeItem::Group("Surfaces") => Some(NewItem::MeshSet(SetKind::Surface)),
        _ => None,
    }
}

/// The kind of a node set, element set or surface of the mesh.
pub fn set_kind(item: &TreeItem) -> Option<SetKind> {
    match item {
        TreeItem::NodeSet(_) => Some(SetKind::Nodes),
        TreeItem::ElementSet(_) => Some(SetKind::Elements),
        TreeItem::Surface(_) => Some(SetKind::Surface),
        _ => None,
    }
}

pub fn set_name(item: &TreeItem) -> Option<&str> {
    match item {
        TreeItem::NodeSet(n) | TreeItem::ElementSet(n) | TreeItem::Surface(n) => Some(n),
        _ => None,
    }
}

/// Whether double-clicking opens a dialog. Containers such as "Mesh" or
/// "Parts" have none: they create their kind of item or open and close.
fn has_properties(item: &TreeItem) -> bool {
    !matches!(
        item,
        TreeItem::Group(_)
            | TreeItem::Mesh
            | TreeItem::StepGroup(..)
            | TreeItem::FieldOutputs
            | TreeItem::HistoryField(..)
    )
}

/// Items of the FE model that have an edit dialog.
/// Items the context menu and the Delete key remove; the parts of results are not.
fn deletable(view: TreeView, item: &TreeItem) -> bool {
    is_fe_item(item)
        || (matches!(item, TreeItem::Part(_)) && view != TreeView::Results)
        || feature(item).is_some()
        || matches!(
            item,
            TreeItem::ResultFieldOutput(_)
                | TreeItem::HistorySet(_)
                | TreeItem::HotSpot(_)
                | TreeItem::MeshItem(_)
        )
        || (view == TreeView::FeModel && set_kind(item).is_some())
}

fn is_fe_item(item: &TreeItem) -> bool {
    matches!(
        item,
        TreeItem::Material(_)
            | TreeItem::Section(_)
            | TreeItem::CrossSection(_)
            | TreeItem::Constraint(_)
            | TreeItem::SurfaceInteraction(_)
            | TreeItem::ContactRegion(_)
            | TreeItem::ContactDetection(_)
            | TreeItem::Bolt(_)
            | TreeItem::ContactPair(_)
            | TreeItem::Tie(_)
            | TreeItem::NodeTie(_)
            | TreeItem::Amplitude(_)
            | TreeItem::Distribution(_)
            | TreeItem::InitialCondition(_)
            | TreeItem::Step(_)
            | TreeItem::BoundaryCondition(..)
            | TreeItem::Load(..)
            | TreeItem::FieldOutput(..)
            | TreeItem::HistoryOutput(..)
            | TreeItem::DefinedField(..)
    )
}

/// "Name (n)" when there are entries, the label of a container.
fn counted(name: &str, count: usize) -> String {
    if count > 0 {
        format!("{name} ({count})")
    } else {
        name.to_string()
    }
}

/// Colour of the dotted lines that connect the nodes, as in the Windows tree view.
const LINE: Color32 = Color32::from_rgb(160, 160, 160);

/// Height of the connector line below the top of a row's icon: the dotted images
/// have their line in pixel row 9.
const LINE_OFFSET: f32 = 9.0;

/// Dotted line between two points on a horizontal or vertical, one screen pixel wide with a
/// dot on every second pixel, so that crossing lines share their dots. Drawn as a plain mesh,
/// as egui would blur rectangles of a single pixel.
fn dotted(mesh: &mut Mesh, ppp: f32, a: Pos2, b: Pos2) {
    let px = |v: f32| (v * ppp).floor() as i32;
    let color = crate::style::adapt(LINE);
    let mut dot = |x: i32, y: i32| {
        let min = pos2(x as f32 / ppp, y as f32 / ppp);
        mesh.add_colored_rect(Rect::from_min_size(min, Vec2::splat(1.0 / ppp)), color);
    };
    let (x0, y0, x1, y1) = (px(a.x), px(a.y), px(b.x), px(b.y));
    if x0 == x1 {
        let first = y0.min(y1);
        for y in (first + first.rem_euclid(2)..=y0.max(y1)).step_by(2) {
            dot(x0, y);
        }
    } else {
        let first = x0.min(x1);
        for x in (first + first.rem_euclid(2)..=x0.max(x1)).step_by(2) {
            dot(x, y0);
        }
    }
}

/// The plus and minus box of the classic Windows tree view, on whole pixels.
/// Whether `item` lies in the branch `branch`, which opens to reveal it.
fn contains(branch: &TreeItem, item: &TreeItem) -> bool {
    use TreeItem::*;
    match branch {
        Model => !matches!(item, Model),
        Mesh | Group("Parts") => matches!(item, Part(_)),
        Group("Constraints") => matches!(item, Constraint(_)),
        Group("Sections") => matches!(item, Section(_) | CrossSection(_) | Group(CROSS_SECTIONS)),
        Group(CROSS_SECTIONS) => matches!(item, CrossSection(_)),
        Group("Amplitudes") => matches!(item, Amplitude(_)),
        Group("Distributions") => matches!(item, Distribution(_)),
        Group("Contacts") => matches!(
            item,
            SurfaceInteraction(_)
                | ContactRegion(_)
                | ContactDetection(_)
                | Bolt(_)
                | ContactPair(_)
                | Tie(_)
                | NodeTie(_)
                | Group("Surface Interactions")
                | Group("Contact Pairs")
                | Group(CONTACT_REGIONS)
                | Group(BOLTS)
                | Group(ADVANCED_CONTACTS)
        ),
        Group(CONTACT_REGIONS) => matches!(item, ContactRegion(_)),
        Group(BOLTS) => matches!(item, Bolt(_)),
        Group(ADVANCED_CONTACTS) => matches!(
            item,
            SurfaceInteraction(_)
                | ContactPair(_)
                | Tie(_)
                | NodeTie(_)
                | Group("Surface Interactions")
                | Group("Contact Pairs")
        ),
        Group("Surface Interactions") => matches!(item, SurfaceInteraction(_)),
        Group("Contact Pairs") => matches!(item, ContactPair(_) | Tie(_) | NodeTie(_)),
        Group("Steps") => matches!(item, BoundaryCondition(..) | Load(..)),
        Step(s) | StepGroup(s, "BCs") if matches!(item, BoundaryCondition(t, _) if t == s) => true,
        Step(s) | StepGroup(s, "Loads") if matches!(item, Load(t, _) if t == s) => true,
        _ => false,
    }
}

fn expander(ui: &mut Ui, openness: f32, response: &Response) {
    let ppp = ui.pixels_per_point();
    let snap = |v: f32| (v * ppp).floor() / ppp;
    let center = response.rect.center();
    // Centred on the connector line of the row.
    let min = pos2(
        snap(center.x) - 4.0,
        snap(center.y - 8.0 + LINE_OFFSET) - 4.0,
    );
    let mut mesh = Mesh::default();
    let mut fill = |x: f32, y: f32, w: f32, h: f32, color: Color32| {
        let rect = Rect::from_min_size(min + vec2(x, y), vec2(w, h));
        mesh.add_colored_rect(rect, crate::style::adapt(color));
    };
    fill(0.0, 0.0, 9.0, 9.0, Color32::from_rgb(145, 145, 145));
    fill(1.0, 1.0, 7.0, 7.0, Color32::WHITE);
    fill(2.0, 4.0, 5.0, 1.0, Color32::BLACK);
    if openness < 0.5 {
        fill(4.0, 2.0, 1.0, 5.0, Color32::BLACK);
    }
    ui.painter().add(Shape::mesh(mesh));
}

/// Whether the context menu of `group` shows and hides the symbol of `item`.
fn shows_symbol_of(group: &TreeItem, item: &TreeItem) -> bool {
    match (group, item) {
        (TreeItem::Group("Constraints"), TreeItem::Constraint(_)) => true,
        (TreeItem::StepGroup(s, "BCs"), TreeItem::BoundaryCondition(t, _))
        | (TreeItem::StepGroup(s, "Loads"), TreeItem::Load(t, _)) => s == t,
        _ => false,
    }
}

/// Icon of a part after its element types, telling solid, shell and beam parts apart.
fn part_icon(part: &PartInfo) -> TreeIcon {
    if !part.visible {
        return TreeIcon::Hidden;
    }
    match part.dimension {
        Some(3) => return TreeIcon::Solid,
        Some(2) => return TreeIcon::Shell,
        Some(1) => return TreeIcon::Wire,
        _ => {}
    }
    let is = |prefixes: &[&str]| {
        (part.element_types.iter())
            .any(|(name, _)| prefixes.iter().any(|p| name.to_uppercase().starts_with(p)))
    };
    if is(&["C3D", "F3D"]) {
        TreeIcon::Solid
    } else if is(&["S", "M3D", "CPS", "CPE", "CAX"]) {
        TreeIcon::Shell
    } else if is(&["B", "T"]) {
        TreeIcon::Wire
    } else {
        TreeIcon::Solid
    }
}

/// Context menu of a part, the same in the tree and in the 3D view. A part of the geometry
/// is meshed from here. On one of several selected parts, hiding and
/// deleting take them all.
pub fn part_menu(
    ui: &mut Ui,
    index: usize,
    visible: bool,
    contours: bool,
    view: TreeView,
    selected: &BTreeSet<usize>,
    response: &mut TreeResponse,
) {
    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
    if view == TreeView::Geometry {
        if ui.button("Create Mesh").clicked() {
            response.mesh_part = Some(index);
        }
        ui.separator();
    }
    if ui.button("Properties …").clicked() {
        response.open = Some(TreeItem::Part(index));
    }
    if view == TreeView::FeModel && ui.button("Transform …").clicked() {
        response.transform_part = Some(index);
    }
    if view == TreeView::FeModel
        && selected.len() > 1
        && selected.contains(&index)
        && ui.button("Merge").clicked()
    {
        response.merge_parts = true;
    }
    let label = if visible { "Hide" } else { "Show" };
    if ui.button(label).clicked() {
        if selected.contains(&index) {
            (response.visibility).extend(selected.iter().map(|&i| (i, !visible)));
        } else {
            response.visibility.push((index, !visible));
        }
    }
    if view == TreeView::Results {
        // A part without colour contours is grey and not part of the legend range.
        let label = if contours {
            "Color Contours Off"
        } else {
            "Color Contours On"
        };
        if ui.button(label).clicked() {
            if selected.contains(&index) {
                (response.contours).extend(selected.iter().map(|&i| (i, !contours)));
            } else {
                response.contours.push((index, !contours));
            }
        }
    }
    if deletable(view, &TreeItem::Part(index)) {
        ui.separator();
        if ui.button("Delete").clicked() {
            response.delete = Some(TreeItem::Part(index));
        }
    }
}

/// A row of a branch: the height of its connector line and the left edge of the row.
#[derive(Clone, Copy)]
struct Row {
    y: f32,
    left: f32,
}

struct Tree<'a> {
    view: TreeView,
    state: &'a mut TreeState,
    /// The row drawn last was double-clicked.
    doubled: bool,
    response: TreeResponse,
    /// Rows of the open branches, innermost last, for the connector lines.
    levels: Vec<Vec<Row>>,
    /// Inside an item being expanded or collapsed: the state all branches take.
    forced_open: Option<bool>,
    /// Problems found by the model checks, by item: shown with a warning sign, and red when
    /// the solver would abort.
    findings: HashMap<TreeItem, Vec<Finding>>,
    /// Containers holding items with errors, shown red so that they are found when collapsed.
    holds_invalid: HashSet<TreeItem>,
    /// Containers that cannot take items, with the reason, such as the loads of a frequency
    /// step.
    closed: HashMap<TreeItem, &'static str>,
    /// Deactivated items, shown gray with the no-entry sign.
    inactive: HashSet<TreeItem>,
    /// Items with a master and a slave, whose menu offers to swap them.
    master_slave: HashSet<TreeItem>,
    /// The detection each detected contact region is shown under.
    detected: HashMap<usize, usize>,
    /// The contact regions bolts replace, with the bolt.
    replaced: std::collections::BTreeMap<usize, usize>,
    /// Constraints, boundary conditions and loads with a symbol in the 3D view, and whether
    /// it is shown; hidden ones get the hide icon.
    symbols: HashMap<TreeItem, bool>,
    /// Number of steps, to disable "Move Down" on the last one.
    step_count: usize,
    /// Names of the steps, for the menus that create an item in one of them.
    step_names: Vec<String>,
    /// The parts listed in the tree for the menu of "Parts".
    part_indices: Vec<usize>,
    /// The type of every step, which gives it its icon and tooltip.
    step_types: Vec<StepGlyph>,
    /// The items of the container being drawn, which Ctrl and Shift select together.
    siblings: Vec<TreeItem>,
}

impl Tree<'_> {
    /// A menu entry creating an item in a step: a button with one step, a submenu of the
    /// steps with several, disabled without any.
    fn step_item_menu(
        &mut self,
        ui: &mut egui::Ui,
        label: &str,
        hint: &str,
        create: impl Fn(usize) -> NewItem,
    ) {
        match &self.step_names[..] {
            [] => {
                ui.add_enabled(false, egui::Button::new(format!("{label} …")))
                    .on_disabled_hover_text("Create a step first.");
            }
            [_] => {
                if ui
                    .button(format!("{label} …"))
                    .on_hover_text(hint)
                    .clicked()
                {
                    self.response.create = Some(create(0));
                }
            }
            names => {
                let mut chosen = None;
                ui.menu_button(label, |ui| {
                    for (step, name) in names.iter().enumerate() {
                        if ui.button(name).on_hover_text(hint).clicked() {
                            chosen = Some(step);
                        }
                    }
                })
                .response
                .on_hover_text(hint);
                if let Some(step) = chosen {
                    self.response.create = Some(create(step));
                }
            }
        }
    }

    fn is_selected(&self, item: &TreeItem) -> bool {
        match item {
            TreeItem::Part(index) => self.state.selected_parts(self.view).contains(index),
            _ => self.state.selected_items(self.view).contains(item),
        }
    }

    /// Tooltip of a step: its type, which the name of the step does not tell.
    fn step_type_hint(&self, item: &TreeItem) -> Option<String> {
        let TreeItem::Step(s) = item else { return None };
        let glyph = self.step_types.get(*s)?;
        Some(format!("Step type: {}", glyph.name()))
    }

    /// Whether the click on the row of `item` is the second of a double click. egui measures
    /// 0.3 s between clicks in frame time, so a slow frame after the first click (the
    /// selection redraws the 3D view) swallowed the second one; here the time is longer.
    fn double_click(&mut self, ui: &Ui, item: &TreeItem, response: &Response) -> bool {
        if !response.clicked() && !response.double_clicked() {
            return false;
        }
        let now = ui.input(|i| i.time);
        let last = self.state.last_click.take();
        let double = response.double_clicked()
            || last.is_some_and(|(view, last, at)| {
                view == self.view && last == *item && now - at <= DOUBLE_CLICK_SECONDS
            });
        if !double {
            self.state.last_click = Some((self.view, item.clone(), now));
        }
        double
    }

    /// Selectable label of an item: a click selects it, a double click opens its properties.
    fn label(&mut self, ui: &mut Ui, item: TreeItem, text: impl Into<WidgetText>) -> Response {
        let findings = self.findings.get(&item).cloned();
        let error = (findings.iter().flatten()).any(|f| f.severity() == Severity::Error);
        let red = error || self.holds_invalid.contains(&item);
        let inactive = self.inactive.contains(&item);
        let color = if red {
            Some(INVALID)
        } else {
            inactive.then_some(INACTIVE)
        };
        let selected = self.is_selected(&item);
        // Drawn under the label once its size is known.
        let mark = ui.painter().add(Shape::Noop);
        let mut response = row_label(ui, selected, color, text);
        if !selected && self.view == TreeView::FeModel && self.state.acting.contains(&item) {
            ui.painter().set(mark, acting_mark(response.rect));
            response = response.on_hover_text("Acts on the selected part");
        }
        if let Some(source) = self.state.linked.get(&item) {
            let tip = format!(
                "Linked from {source}: it changes there. Double-click to open {source} or \
                 unlink it."
            );
            link_sign(ui).on_hover_text(&tip);
            response = response.on_hover_text(tip);
        }
        let step_type = self.step_type_hint(&item);
        if findings.is_none()
            && let Some(hint) = &step_type
        {
            response = response.on_hover_text(hint);
        }
        if let Some(findings) = findings {
            let mut hover: Vec<String> = (step_type.iter().cloned())
                .chain(
                    findings
                        .iter()
                        .map(|f| format!("{}: {}", f.problem.title(), f.detail)),
                )
                .collect();
            hover.push("Click the warning sign to explain the problem.".into());
            let hover = hover.join("\n");
            if warning_sign(ui).on_hover_text(&hover).clicked() {
                self.response.findings = Some(findings);
            }
            response = response.on_hover_text(hover);
        }
        // A right click selects the item its context menu belongs to; on a
        // part of several selected ones it keeps them, so that the menu deletes them all.
        let double = self.double_click(ui, &item, &response);
        self.doubled = double;
        if response.clicked() || response.double_clicked() || response.secondary_clicked() {
            let (ctrl, shift) = ui.input(|i| (i.modifiers.command, i.modifiers.shift));
            match item {
                TreeItem::Part(_) if response.secondary_clicked() && self.is_selected(&item) => {}
                // Several parts are selected with Ctrl and Shift, as in a Windows tree.
                TreeItem::Part(index) if response.clicked() && shift => {
                    self.state.select_range(self.view, index, ctrl);
                }
                TreeItem::Part(index) if response.clicked() && ctrl => {
                    self.state.toggle_part(self.view, index);
                }
                TreeItem::Part(index) => self.state.select_part(self.view, index),
                // Siblings in a container likewise, e.g. contact pairs; a right click on
                // one of them keeps them for its menu.
                _ if response.secondary_clicked() && self.is_selected(&item) => {}
                _ if response.clicked() && (ctrl || shift) && self.siblings.contains(&item) => {
                    let siblings = self.siblings.clone();
                    (self.state).select_sibling(self.view, item.clone(), &siblings, ctrl, shift);
                }
                _ => self.state.select_item(self.view, item.clone()),
            }
        }
        let lead =
            (self.state.selected.as_ref()).is_some_and(|(v, s)| *v == self.view && *s == item);
        if self.state.reveal && lead {
            response.scroll_to_me(None);
        }
        let closed = self.closed.get(&item).copied();
        if let Some(reason) = closed {
            response = response.on_hover_text(reason);
        }
        let creates = (creates(&item))
            .or_else(|| (self.view == TreeView::FeModel).then(|| creates_set(&item))?)
            .filter(|_| closed.is_none());
        if item == TreeItem::Group("Parts") && !self.part_indices.is_empty() {
            response.context_menu(|ui| {
                ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                for (label, visible) in [("Show All", true), ("Hide All", false)] {
                    if ui.button(label).clicked() {
                        (self.response.visibility)
                            .extend(self.part_indices.iter().map(|&i| (i, visible)));
                    }
                }
            });
        }
        if double {
            // Items are created from the ribbon; containers only open or close, see
            // `branch`.
            if has_properties(&item) {
                self.response.open = Some(item.clone());
            }
        }
        let editable = is_fe_item(&item)
            || feature(&item).is_some()
            || (self.view == TreeView::FeModel && set_kind(&item).is_some())
            || matches!(
                item,
                TreeItem::ResultFieldOutput(_)
                    | TreeItem::HistorySet(_)
                    | TreeItem::HotSpot(_)
                    | TreeItem::MeshItem(_)
            );
        // The menu of one of several selected items acts on all of them.
        let several = self.is_selected(&item) && self.state.selected_items(self.view).len() > 1;
        if creates.is_some() || editable {
            response.context_menu(|ui| {
                ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                // Tools such as the material library are on the
                // ribbon only; the menu holds what acts on this item. A separator only
                // goes below something.
                let top = ui.cursor().top();
                let separator = |ui: &mut Ui| {
                    if ui.cursor().top() > top + 1.0 {
                        ui.separator();
                    }
                };
                if let TreeItem::ContactDetection(d) = item
                    && ui
                        .button("Detect Again")
                        .on_hover_text(
                            "Searches again and replaces the regions found before, except \
                             those you changed since.",
                        )
                        .clicked()
                {
                    self.response.detect = Some(d);
                }
                let members: Vec<TreeItem> = (self.symbols.keys())
                    .filter(|member| shows_symbol_of(&item, member))
                    .cloned()
                    .collect();
                if !members.is_empty() {
                    separator(ui);
                    for (label, visible) in [("Show All", true), ("Hide All", false)] {
                        if ui.button(label).clicked() {
                            (self.response.symbol_visibility)
                                .extend(members.iter().map(|m| (m.clone(), visible)));
                        }
                    }
                }
                if creates.is_some() {
                    separator(ui);
                    if ui.button("Expand All").clicked() {
                        self.state.expand = Some((self.view, item.clone(), true));
                    }
                    if ui.button("Collapse All").clicked() {
                        self.state.expand = Some((self.view, item.clone(), false));
                    }
                }
                if editable {
                    separator(ui);
                    if ui.button("Edit …").clicked() {
                        self.response.open = Some(item.clone());
                    }
                    if let Some(&visible) = self.symbols.get(&item) {
                        let label = if visible { "Hide" } else { "Show" };
                        if ui.button(label).clicked() {
                            self.response
                                .symbol_visibility
                                .push((item.clone(), !visible));
                        }
                    }
                    if can_deactivate(&item) {
                        separator(ui);
                        let label = match (inactive, several) {
                            (true, false) => "Activate",
                            (false, false) => "Deactivate",
                            (true, true) => "Activate Selected",
                            (false, true) => "Deactivate Selected",
                        };
                        if ui.button(label).clicked() {
                            self.response.toggle_active = Some(item.clone());
                        }
                        separator(ui);
                    }
                    if let TreeItem::Step(s) = item {
                        if ui
                            .add_enabled(s > 0, egui::Button::new("Move Up"))
                            .clicked()
                        {
                            self.response.move_step = Some((s, true));
                        }
                        if ui
                            .add_enabled(s + 1 < self.step_count, egui::Button::new("Move Down"))
                            .clicked()
                        {
                            self.response.move_step = Some((s, false));
                        }
                        separator(ui);
                    }
                    let contact = match item {
                        TreeItem::ContactRegion(i) => Some(ContactItem::Region(i)),
                        TreeItem::ContactPair(i) => Some(ContactItem::Pair(i)),
                        _ => None,
                    };
                    if let Some(contact) = contact.filter(|_| !several) {
                        self.step_item_menu(
                            ui,
                            "Create Contact History Output",
                            "Prints the contact forces, slip and pressure of this contact into \
                             the .dat file at every increment of the step.",
                            |step| NewItem::ContactHistoryOutput(step, contact),
                        );
                        self.step_item_menu(
                            ui,
                            "Create Contact Field Output",
                            "Writes the contact pressure, slip and gap of all contacts into the \
                             .frd file; a step that writes them already opens its output.",
                            NewItem::ContactFieldOutput,
                        );
                        separator(ui);
                    }
                    if self.master_slave.contains(&item) {
                        let label = match (several, &item) {
                            (true, TreeItem::ContactRegion(_)) => "Flip Sides of Selected",
                            (false, TreeItem::ContactRegion(_)) => "Flip Sides",
                            (true, _) => "Swap Master/Slave of Selected",
                            (false, _) => "Swap Master/Slave",
                        };
                        if ui.button(label).clicked() {
                            self.response.swap_master_slave = Some(item.clone());
                        }
                        separator(ui);
                    }
                    let label = if several { "Delete Selected" } else { "Delete" };
                    if deletable(self.view, &item) && ui.button(label).clicked() {
                        self.response.delete = Some(item.clone());
                    }
                }
            });
        }
        response
    }

    /// The image of a node; containers without an own image get the dotted line.
    fn icon(&self, item: &TreeItem, open: bool) -> TreeIcon {
        if self.inactive.contains(item) {
            return TreeIcon::Inactive;
        }
        if self.symbols.get(item) == Some(&false) {
            return TreeIcon::Hidden;
        }
        let dots = if open {
            TreeIcon::DotsOpen
        } else {
            TreeIcon::Dots
        };
        match item {
            TreeItem::Mesh => TreeIcon::Mesh,
            TreeItem::Group("Parts") if self.view == TreeView::Geometry => TreeIcon::Geometry,
            TreeItem::Group("Parts") => TreeIcon::Part,
            TreeItem::Group("Mesh Setup") => TreeIcon::MeshSetup,
            TreeItem::Group("Node Sets") => TreeIcon::NodeSet,
            TreeItem::Group("Element Sets") => TreeIcon::ElementSet,
            TreeItem::Group("Surfaces") => TreeIcon::Surface,
            TreeItem::Group("Features") => TreeIcon::Features,
            TreeItem::Group("Reference Points") => TreeIcon::ReferencePoint,
            TreeItem::Group("Coordinate Systems") => TreeIcon::CoordinateSystem,
            TreeItem::Group(PLANES) => TreeIcon::Plane,
            TreeItem::Group(PATHS) => TreeIcon::Path,
            TreeItem::Group(PLANE_RESULTS) => TreeIcon::PlaneResult,
            TreeItem::Group("Materials") => TreeIcon::Material,
            TreeItem::Group("Sections") | TreeItem::Group(CROSS_SECTIONS) => TreeIcon::Section,
            TreeItem::Group("Constraints") => TreeIcon::Constraints,
            TreeItem::Group("Contacts") => TreeIcon::Contacts,
            TreeItem::Group("Surface Interactions") => TreeIcon::SurfaceInteraction,
            TreeItem::Group("Contact Pairs") => TreeIcon::ContactPair,
            TreeItem::Group(CONTACT_REGIONS) => TreeIcon::ContactPair,
            TreeItem::Group(BOLTS) => TreeIcon::Load,
            TreeItem::ContactDetection(_) => TreeIcon::Contacts,
            TreeItem::Group("Distributions") => TreeIcon::Distribution,
            TreeItem::Group("Amplitudes") => TreeIcon::Amplitude,
            TreeItem::Group("Initial Conditions") => TreeIcon::InitialConditions,
            TreeItem::Group("Steps") => TreeIcon::Steps,
            TreeItem::Step(s) => match self.step_types.get(*s) {
                Some(glyph) => TreeIcon::StepType(*glyph),
                None => TreeIcon::Step,
            },
            TreeItem::StepGroup(_, "Field Outputs") | TreeItem::FieldOutputs => {
                TreeIcon::FieldOutput
            }
            TreeItem::StepGroup(_, "History Outputs") | TreeItem::Group("History Outputs") => {
                TreeIcon::HistoryOutput
            }
            TreeItem::StepGroup(_, "BCs") => TreeIcon::BoundaryCondition,
            TreeItem::StepGroup(_, "Loads") => TreeIcon::Load,
            TreeItem::StepGroup(_, "Defined Fields") => TreeIcon::DefinedField,
            TreeItem::Group(HOT_SPOTS) => TreeIcon::HotSpot,
            _ => dots,
        }
    }

    /// Whether `item` is left out because another analysis owns it. Categories and the mesh
    /// stay, with their link sign.
    fn hidden(&self, item: &TreeItem) -> bool {
        self.state.hide_linked
            && self.view == TreeView::FeModel
            && self.state.linked.contains_key(item)
            && !matches!(item, TreeItem::Group(_) | TreeItem::Mesh)
    }

    /// A row: plus and minus box (or its space), icon and label. `check` adds a check box in
    /// front of the icon. Records the row for the connector lines of its branch.
    fn row(
        &mut self,
        ui: &mut Ui,
        state: Option<&mut CollapsingState>,
        check: Option<&mut bool>,
        icon: TreeIcon,
        item: TreeItem,
        text: impl Into<WidgetText>,
    ) -> (Row, Response, bool) {
        if self.hidden(&item) {
            self.doubled = false;
            let response = ui.interact(
                egui::Rect::NOTHING,
                ui.id().with("hidden"),
                egui::Sense::hover(),
            );
            return (Row { y: 0.0, left: 0.0 }, response, false);
        }
        ui.horizontal(|ui| {
            let left = ui.cursor().left();
            ui.spacing_mut().item_spacing.x = 0.0;
            match state {
                Some(state) => {
                    state.show_toggle_button(ui, expander);
                }
                None => ui.add_space(ui.spacing().indent),
            }
            let changed = check.is_some_and(|checked| {
                let changed = ui.checkbox(checked, "").changed();
                ui.add_space(2.0);
                changed
            });
            // Clicking the sign of a deactivated item activates it.
            let inactive = self.inactive.contains(&item);
            let sense = if inactive {
                egui::Sense::click()
            } else {
                egui::Sense::hover()
            };
            let (rect, icon_response) =
                ui.allocate_exact_size(Vec2::splat(tree_icons::SIZE), sense);
            if ui.is_rect_visible(rect) {
                tree_icons::paint(ui.painter(), rect.min, icon);
            }
            let icon_response = match self.step_type_hint(&item) {
                Some(hint) if !inactive => icon_response.on_hover_text(hint),
                _ => icon_response,
            };
            if inactive && icon_response.on_hover_text("Activate").clicked() {
                self.response.toggle_active = Some(item.clone());
            }
            ui.add_space(3.0);
            let response = self.label(ui, item, text);
            let row = Row {
                y: rect.top() + LINE_OFFSET,
                left,
            };
            if let Some(level) = self.levels.last_mut() {
                level.push(row);
            }
            (row, response, changed)
        })
        .inner
    }

    /// Node without children, aligned with the labels of sibling branches.
    /// Whether `item` lies in the branch `branch`; detected contact regions lie in their
    /// detection, not among the regions made by hand.
    fn contains(&self, branch: &TreeItem, item: &TreeItem) -> bool {
        match (branch, item) {
            (TreeItem::ContactDetection(d), TreeItem::ContactRegion(r)) => {
                self.detected.get(r) == Some(d)
            }
            (TreeItem::Group(CONTACT_REGIONS), TreeItem::ContactRegion(r)) => {
                !self.detected.contains_key(r)
            }
            _ => contains(branch, item),
        }
    }

    fn leaf(&mut self, ui: &mut Ui, item: TreeItem, text: impl Into<WidgetText>) -> Response {
        let icon = self.icon(&item, false);
        self.row(ui, None, None, icon, item, text).1
    }

    fn branch(
        &mut self,
        ui: &mut Ui,
        item: TreeItem,
        text: impl Into<WidgetText>,
        default_open: bool,
        body: impl FnOnce(&mut Self, &mut Ui),
    ) {
        if self.hidden(&item) {
            return;
        }
        let id = ui.make_persistent_id((self.view, &item));
        let mut state = CollapsingState::load_with_default_open(ui.ctx(), id, default_open);
        let outer = self.forced_open;
        if let Some((view, target, open)) = &self.state.expand
            && *view == self.view
            && *target == item
        {
            self.forced_open = Some(*open);
        }
        let revealing = (self.state.reveal)
            .then_some(self.state.selected.as_ref())
            .flatten()
            .filter(|(view, _)| *view == self.view);
        if let Some((_, selected)) = revealing
            && self.contains(&item, selected)
        {
            state.set_open(true);
        }
        if self.state.reveal_acting
            && self.view == TreeView::FeModel
            && self
                .state
                .acting
                .iter()
                .any(|acting| self.contains(&item, acting))
        {
            state.set_open(true);
        }
        if let Some(open) = self.forced_open {
            // While a branch closes, egui still draws its body for the animation, so the
            // descendants are collapsed as well.
            state.set_open(open);
        }
        let toggles = !has_properties(&item);
        let icon = self.icon(&item, state.is_open());
        let (row, header, _) = self.row(ui, Some(&mut state), None, icon, item, text);
        let doubled = self.doubled;
        // Lines go below the rows of the body, so that the boxes of sub-branches cover them.
        let lines = ui.painter().add(Shape::Noop);
        self.levels.push(Vec::new());
        let body = state.show_body_indented(&header, ui, |ui| body(self, ui));
        let rows = self.levels.pop().unwrap_or_default();
        if let Some(body) = body {
            let x = row.left + ui.spacing().indent / 2.0;
            let shapes = self.connectors(ui, x, row.y + 5.0, &rows, body.response.rect.bottom());
            ui.painter().set(lines, shapes);
        }
        self.forced_open = outer;
        if toggles && doubled {
            let mut state = CollapsingState::load_with_default_open(ui.ctx(), id, default_open);
            state.toggle(ui);
            state.store(ui.ctx());
        }
    }

    /// Dotted lines from a vertical at `x`, starting at `top`, to the icons of `rows`; rows
    /// below `bottom` are hidden by the opening animation.
    fn connectors(&self, ui: &Ui, x: f32, top: f32, rows: &[Row], bottom: f32) -> Shape {
        let ppp = ui.pixels_per_point();
        let indent = ui.spacing().indent;
        let mut mesh = Mesh::default();
        let rows: Vec<Row> = rows.iter().copied().filter(|r| r.y <= bottom).collect();
        if let Some(last) = rows.last() {
            dotted(&mut mesh, ppp, pos2(x, top), pos2(x, last.y));
        }
        for row in rows {
            dotted(
                &mut mesh,
                ppp,
                pos2(x, row.y),
                pos2(row.left + indent - 1.0, row.y),
            );
        }
        Shape::mesh(mesh)
    }

    /// Parts with visibility and colour; of the mesh, or of the geometry.
    fn parts(&mut self, ui: &mut Ui, model: &mut Model) {
        let shown = (0..model.parts.len())
            .filter(|i| !self.hidden(&TreeItem::Part(*i)))
            .count();
        let parts = counted("Parts", shown);
        self.part_indices = (0..model.parts.len())
            .filter(|i| !self.hidden(&TreeItem::Part(*i)))
            .collect();
        // Before the mesh exists, Parts stays as the place the mesh will go, but without
        // a box to expand.
        if shown == 0 {
            self.leaf(ui, TreeItem::Group("Parts"), parts);
            return;
        }
        self.branch(ui, TreeItem::Group("Parts"), parts, true, |tree, ui| {
            for (index, part) in model.parts.iter_mut().enumerate() {
                if tree.hidden(&TreeItem::Part(index)) {
                    continue;
                }
                let icon = part_icon(part);
                let item = TreeItem::Part(index);
                let (_, response, changed) =
                    tree.row(ui, None, Some(&mut part.visible), icon, item, &part.name);
                if changed {
                    tree.response.visibility.push((index, part.visible));
                }
                let view = tree.view;
                let selected = tree.state.selected_parts(view);
                response.context_menu(|ui| {
                    part_menu(
                        ui,
                        index,
                        part.visible,
                        part.contours,
                        view,
                        &selected,
                        &mut tree.response,
                    );
                });
            }
        });
    }

    /// Mesh with parts and sets; shared by the FE Model and Results trees.
    fn mesh(&mut self, ui: &mut Ui, model: Option<&mut Model>) {
        let Some(model) = model else {
            self.branch(ui, TreeItem::Mesh, "Mesh", true, |tree, ui| {
                tree.leaf(ui, TreeItem::Group("Parts"), "Parts");
            });
            return;
        };
        self.branch(ui, TreeItem::Mesh, "Mesh", true, |tree, ui| {
            tree.parts(ui, model);
            let mesh = &model.mesh;
            // The user's sets are listed even when their selection finds nothing on the mesh.
            let names = |kind: SetKind| -> Vec<String> {
                let mut names: BTreeSet<String> =
                    kind.names(mesh).into_iter().map(str::to_owned).collect();
                let user = model.fe.mesh_sets.iter().filter(|s| s.kind == kind);
                names.extend(user.map(|s| s.name.clone()));
                names.into_iter().collect()
            };
            let sets: [(&'static str, Vec<TreeItem>); 3] = [
                (
                    "Node Sets",
                    names(SetKind::Nodes)
                        .into_iter()
                        .map(TreeItem::NodeSet)
                        .collect(),
                ),
                (
                    "Element Sets",
                    (names(SetKind::Elements).into_iter())
                        .map(TreeItem::ElementSet)
                        .collect(),
                ),
                (
                    "Surfaces",
                    names(SetKind::Surface)
                        .into_iter()
                        .map(TreeItem::Surface)
                        .collect(),
                ),
            ];
            for (name, items) in sets {
                // Like the other containers, an empty one is not shown.
                if items.is_empty() {
                    continue;
                }
                let text = counted(name, items.len());
                tree.branch(ui, TreeItem::Group(name), text, false, |tree, ui| {
                    for item in items {
                        let label = match &item {
                            TreeItem::NodeSet(n)
                            | TreeItem::ElementSet(n)
                            | TreeItem::Surface(n) => n.clone(),
                            _ => String::new(),
                        };
                        tree.leaf(ui, item, label);
                    }
                });
            }
        });
    }

    /// Container of model items, e.g. "Materials (2)".
    /// Surface interactions, contact pairs, ties and node ties, under Connections > Advanced.
    fn advanced_contacts(&mut self, ui: &mut Ui, fe: &FeModel) {
        let interactions = (fe.surface_interactions.iter().enumerate())
            .map(|(i, s)| (TreeItem::SurfaceInteraction(i), s.name.as_str()))
            .collect();
        self.container(ui, "Surface Interactions", interactions);
        let pairs = (fe.contact_pairs.iter().enumerate())
            .map(|(i, c)| (TreeItem::ContactPair(i), c.name.as_str()))
            .chain((fe.ties.iter().enumerate()).map(|(i, t)| (TreeItem::Tie(i), t.name.as_str())))
            .chain(
                (fe.node_ties.iter().enumerate())
                    .map(|(i, t)| (TreeItem::NodeTie(i), t.name.as_str())),
            )
            .collect();
        self.container(ui, "Contact Pairs", pairs);
    }

    fn container(&mut self, ui: &mut Ui, name: &'static str, mut items: Vec<(TreeItem, &str)>) {
        items.retain(|(item, _)| !self.hidden(item));
        // An empty category is left out: items are created on the ribbon, not in the tree.
        if items.is_empty() {
            return;
        }
        let text = counted(name, items.len());
        self.branch(ui, TreeItem::Group(name), text, true, |tree, ui| {
            tree.siblings = items.iter().map(|(item, _)| item.clone()).collect();
            for (item, label) in items {
                tree.leaf(ui, item, label);
            }
            tree.siblings.clear();
        });
    }

    /// A contact detection with the regions it found.
    fn detection(&mut self, ui: &mut Ui, index: usize, name: &str, items: Vec<(TreeItem, &str)>) {
        let item = TreeItem::ContactDetection(index);
        if items.is_empty() {
            self.leaf(ui, item, name);
            return;
        }
        let text = counted(name, items.len());
        self.branch(ui, item, text, true, |tree, ui| {
            tree.siblings = items.iter().map(|(item, _)| item.clone()).collect();
            for (item, label) in items {
                tree.leaf(ui, item, label);
            }
            tree.siblings.clear();
        });
    }

    fn step_container(
        &mut self,
        ui: &mut Ui,
        step: usize,
        name: &'static str,
        mut items: Vec<(TreeItem, &str)>,
    ) {
        items.retain(|(item, _)| !self.hidden(item));
        let group = TreeItem::StepGroup(step, name);
        if items.is_empty() {
            return;
        }
        let text = counted(name, items.len());
        self.branch(ui, group, text, true, |tree, ui| {
            tree.siblings = items.iter().map(|(item, _)| item.clone()).collect();
            for (item, label) in items {
                tree.leaf(ui, item, label);
            }
            tree.siblings.clear();
        });
    }

    /// The features: reference points and coordinate systems, and planes.
    fn features(&mut self, ui: &mut Ui, fe: &FeModel) {
        let open = !fe.reference_points.is_empty()
            || !fe.coordinate_systems.is_empty()
            || !fe.planes.is_empty();
        if !open {
            return;
        }
        self.branch(
            ui,
            TreeItem::Group("Features"),
            "Features",
            open,
            |tree, ui| {
                let points = (fe.reference_points.iter().enumerate())
                    .map(|(i, r)| (TreeItem::ReferencePoint(i), r.name.as_str()))
                    .collect();
                tree.container(ui, REFERENCE_POINTS, points);
                let systems = (fe.coordinate_systems.iter().enumerate())
                    .map(|(i, c)| (TreeItem::CoordinateSystem(i), c.name.as_str()))
                    .collect();
                tree.container(ui, COORDINATE_SYSTEMS, systems);
                let planes = (fe.planes.iter().enumerate())
                    .map(|(i, p)| (TreeItem::Plane(i), p.name.as_str()))
                    .collect();
                tree.container(ui, PLANES, planes);
            },
        );
    }
}

/// `mesh_items` names the items of the geometry's mesh setup, for the Geometry tree;
/// `solver_findings` are the problems the solver reported in the last run.
pub fn show(
    ui: &mut Ui,
    view: TreeView,
    model: Option<&mut Model>,
    mesh_items: &[String],
    solver_findings: &[Finding],
    state: &mut TreeState,
) -> TreeResponse {
    let mut tree = Tree {
        view,
        state,
        doubled: false,
        response: TreeResponse::default(),
        levels: vec![Vec::new()],
        forced_open: None,
        findings: HashMap::new(),
        holds_invalid: HashSet::new(),
        closed: HashMap::new(),
        inactive: HashSet::new(),
        symbols: HashMap::new(),
        master_slave: HashSet::new(),
        detected: HashMap::new(),
        replaced: Default::default(),
        step_count: 0,
        step_names: Vec::new(),
        part_indices: Vec::new(),
        step_types: Vec::new(),
        siblings: Vec::new(),
    };
    let expanding = tree.state.expand.clone();
    egui::ScrollArea::both()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            // The dotted connectors replace egui's indent line.
            ui.visuals_mut().indent_has_left_vline = false;
            // The root nodes are connected too, as with the root lines.
            let lines = ui.painter().add(Shape::Noop);
            match view {
                TreeView::Geometry => {
                    match model {
                        Some(model) => tree.parts(ui, model),
                        None => {
                            tree.leaf(ui, TreeItem::Group("Parts"), "Parts");
                        }
                    }
                    let items = (mesh_items.iter().enumerate())
                        .map(|(i, name)| (TreeItem::MeshItem(i), name.as_str()))
                        .collect();
                    tree.container(ui, "Mesh Setup", items);
                }
                TreeView::FeModel => fe_model(&mut tree, ui, model, solver_findings),
                TreeView::Results => results(&mut tree, ui, model),
            }
            let roots = tree.levels.pop().unwrap_or_default();
            if let Some(first) = roots.first() {
                let x = first.left + ui.spacing().indent / 2.0;
                let shapes = tree.connectors(ui, x, first.y, &roots, f32::INFINITY);
                ui.painter().set(lines, shapes);
            }
        });
    // Applied for one frame; a request made in this frame's context menu waits for the next.
    if tree.state.expand == expanding {
        tree.state.expand = None;
    }
    tree.state.reveal = false;
    tree.state.reveal_acting = false;
    // The space bar switches the selected item on or off; here only while the
    // pointer is over the tree, so that typing elsewhere never does.
    if let Some((selected_view, item)) = &tree.state.selected
        && *selected_view == view
        && can_deactivate(item)
        && ui.rect_contains_pointer(ui.max_rect())
        && !ui.ctx().text_edit_focused()
        && ui.input(|i| i.key_pressed(egui::Key::Space))
    {
        tree.response.toggle_active = Some(item.clone());
    }
    // The Delete key deletes the selected item, after the same question as the
    // context menu. The selection may come from the 3D view, so the pointer can be anywhere.
    if let Some((selected_view, item)) = &tree.state.selected
        && *selected_view == view
        && deletable(view, item)
        && !ui.ctx().text_edit_focused()
        && ui.input(|i| i.key_pressed(egui::Key::Delete))
    {
        tree.response.delete = Some(item.clone());
    }
    tree.response
}

fn fe_model(tree: &mut Tree, ui: &mut Ui, model: Option<&mut Model>, solver: &[Finding]) {
    // A results file has no FE model; its mesh lives in the Results tree.
    let model = model.filter(|m| !m.is_results());
    if let Some(model) = &model {
        let fe = &model.fe;
        // Before the contacts, whose menus create outputs in the steps.
        tree.step_names = fe.steps.iter().map(|s| s.name.clone()).collect();
        for (r, region) in fe.contact_regions.iter().enumerate() {
            let detection = (region.detected_by)
                .and_then(|id| fe.contact_detections.iter().position(|d| d.id == id));
            if let Some(d) = detection {
                tree.detected.insert(r, d);
            }
        }
        tree.replaced = model.replaced_regions();
        for finding in model.findings().into_iter().chain(solver.iter().cloned()) {
            let (item, mut containers) = tree_items(finding.item);
            if let TreeItem::ContactRegion(r) = item
                && let Some(&d) = tree.detected.get(&r)
            {
                containers[0] = TreeItem::ContactDetection(d);
            }
            if finding.severity() == Severity::Error {
                tree.holds_invalid.extend(containers);
            }
            tree.findings.entry(item).or_default().push(finding);
        }
    }
    let fe = model.as_ref().map(|m| m.fe.clone()).unwrap_or_default();
    tree.branch(ui, TreeItem::Model, "Model", true, |tree, ui| {
        tree.mesh(ui, model);
        tree.features(ui, &fe);
        let materials: Vec<(TreeItem, &str)> = (fe.materials.iter().enumerate())
            .map(|(i, m)| (TreeItem::Material(i), m.name.as_str()))
            .collect();
        tree.container(ui, "Materials", materials);
        let sections: Vec<(TreeItem, &str)> = (fe.sections.iter().enumerate())
            .map(|(i, s)| (TreeItem::Section(i), s.name.as_str()))
            .filter(|(item, _)| !tree.hidden(item))
            .collect();
        let cross_sections: Vec<(TreeItem, &str)> = (fe.cross_sections.iter().enumerate())
            .map(|(i, c)| (TreeItem::CrossSection(i), c.name.as_str()))
            .collect();
        // The cross-sections first, as the beam sections pick from them.
        let text = counted("Sections", sections.len());
        if !sections.is_empty() || !cross_sections.is_empty() {
            tree.branch(ui, TreeItem::Group("Sections"), text, true, |tree, ui| {
                tree.container(ui, CROSS_SECTIONS, cross_sections);
                tree.siblings = sections.iter().map(|(item, _)| item.clone()).collect();
                for (item, label) in sections {
                    tree.leaf(ui, item, label);
                }
                tree.siblings.clear();
            });
        }
        for (i, constraint) in fe.constraints.iter().enumerate() {
            if !constraint.active() {
                tree.inactive.insert(TreeItem::Constraint(i));
            }
            if constraint.master_slave().is_some() {
                tree.master_slave.insert(TreeItem::Constraint(i));
            }
            if crate::symbols::Kind::of_constraint(constraint).is_some() {
                tree.symbols
                    .insert(TreeItem::Constraint(i), constraint.visible());
            }
        }
        for (i, pair) in fe.contact_pairs.iter().enumerate() {
            if !pair.active {
                tree.inactive.insert(TreeItem::ContactPair(i));
            }
            tree.master_slave.insert(TreeItem::ContactPair(i));
        }
        for (i, tie) in fe.ties.iter().enumerate() {
            if !tie.active {
                tree.inactive.insert(TreeItem::Tie(i));
            }
            tree.master_slave.insert(TreeItem::Tie(i));
        }
        for (i, _) in (fe.node_ties.iter().enumerate()).filter(|(_, t)| !t.active) {
            tree.inactive.insert(TreeItem::NodeTie(i));
        }
        for (i, region) in fe.contact_regions.iter().enumerate() {
            if !region.active || tree.replaced.contains_key(&i) {
                tree.inactive.insert(TreeItem::ContactRegion(i));
            }
            tree.master_slave.insert(TreeItem::ContactRegion(i));
        }
        for (i, _) in (fe.bolts.iter().enumerate()).filter(|(_, b)| !b.active) {
            tree.inactive.insert(TreeItem::Bolt(i));
        }
        let constraints = (fe.constraints.iter().enumerate())
            .map(|(i, c)| (TreeItem::Constraint(i), c.name()))
            .collect();
        tree.container(ui, "Constraints", constraints);
        let contacts = TreeItem::Group("Contacts");
        let advanced = !fe.surface_interactions.is_empty()
            || !fe.contact_pairs.is_empty()
            || !fe.ties.is_empty()
            || !fe.node_ties.is_empty();
        let open = advanced
            || !fe.contact_regions.is_empty()
            || !fe.contact_detections.is_empty()
            || !fe.bolts.is_empty();
        // A region a bolt replaces says so after its name.
        let region_names: Vec<String> = (fe.contact_regions.iter().enumerate())
            .map(
                |(r, region)| match tree.replaced.get(&r).and_then(|&b| fe.bolts.get(b)) {
                    Some(bolt) => format!("{} (replaced by {})", region.name, bolt.name),
                    None => region.name.clone(),
                },
            )
            .collect();
        // The group keeps its name "Contacts", which links of the project schematic use.
        if open {
            tree.branch(ui, contacts, "Connections", open, |tree, ui| {
                for (d, detection) in fe.contact_detections.iter().enumerate() {
                    let regions = (0..fe.contact_regions.len())
                        .filter(|r| tree.detected.get(r) == Some(&d))
                        .map(|r| (TreeItem::ContactRegion(r), region_names[r].as_str()))
                        .collect();
                    tree.detection(ui, d, &detection.name, regions);
                }
                let regions = (0..fe.contact_regions.len())
                    .filter(|r| !tree.detected.contains_key(r))
                    .map(|r| (TreeItem::ContactRegion(r), region_names[r].as_str()))
                    .collect();
                tree.container(ui, CONTACT_REGIONS, regions);
                let bolts = (fe.bolts.iter().enumerate())
                    .map(|(i, b)| (TreeItem::Bolt(i), b.name.as_str()))
                    .collect();
                tree.container(ui, BOLTS, bolts);
                let group = TreeItem::Group(ADVANCED_CONTACTS);
                if advanced {
                    tree.branch(ui, group, ADVANCED_CONTACTS, advanced, |tree, ui| {
                        tree.advanced_contacts(ui, &fe);
                    });
                }
            });
        }
        let distributions = (fe.distributions.iter().enumerate())
            .map(|(i, d)| (TreeItem::Distribution(i), d.name.as_str()))
            .collect();
        tree.container(ui, "Distributions", distributions);
        let amplitudes = (fe.amplitudes.iter().enumerate())
            .map(|(i, a)| (TreeItem::Amplitude(i), a.name.as_str()))
            .collect();
        tree.container(ui, "Amplitudes", amplitudes);
        for (i, _) in (fe.initial_conditions.iter().enumerate()).filter(|(_, c)| !c.active) {
            tree.inactive.insert(TreeItem::InitialCondition(i));
        }
        let initial = (fe.initial_conditions.iter().enumerate())
            .map(|(i, c)| (TreeItem::InitialCondition(i), c.name.as_str()))
            .collect();
        tree.container(ui, "Initial Conditions", initial);
        // The numbers exposed as parameters, each under the item it belongs to, which a
        // double-click opens.
        let exposed: Vec<(TreeItem, &str)> = (fe.exposed.iter())
            .filter_map(|e| Some((item_of(&fe, &e.path.item)?, e.name.as_str())))
            .collect();
        if !exposed.is_empty() {
            tree.container(ui, EXPOSED, exposed);
        }
        let steps = TreeItem::Group("Steps");
        tree.step_count = fe.steps.len();
        tree.step_types = fe.steps.iter().map(|s| StepGlyph::from(&s.kind)).collect();
        let shown = (0..fe.steps.len())
            .filter(|s| !tree.hidden(&TreeItem::Step(*s)))
            .count();
        if shown == 0 {
            return;
        }
        let text = counted("Steps", shown);
        tree.branch(ui, steps, text, true, |tree, ui| {
            for (s, step) in fe.steps.iter().enumerate() {
                if !step.active {
                    tree.inactive.insert(TreeItem::Step(s));
                }
                for (i, _) in
                    (step.boundary_conditions.iter().enumerate()).filter(|(_, b)| !b.active)
                {
                    tree.inactive.insert(TreeItem::BoundaryCondition(s, i));
                }
                for (i, _) in (step.loads.iter().enumerate()).filter(|(_, l)| !l.active) {
                    tree.inactive.insert(TreeItem::Load(s, i));
                }
                for (i, bc) in step.boundary_conditions.iter().enumerate() {
                    tree.symbols
                        .insert(TreeItem::BoundaryCondition(s, i), bc.visible);
                }
                for (i, load) in step.loads.iter().enumerate() {
                    tree.symbols.insert(TreeItem::Load(s, i), load.visible);
                }
                for (i, _) in (step.field_outputs.iter().enumerate()).filter(|(_, f)| !f.active) {
                    tree.inactive.insert(TreeItem::FieldOutput(s, i));
                }
                for (i, _) in (step.history_outputs.iter().enumerate()).filter(|(_, h)| !h.active) {
                    tree.inactive.insert(TreeItem::HistoryOutput(s, i));
                }
                for (i, _) in (step.defined_fields.iter().enumerate()).filter(|(_, f)| !f.active) {
                    tree.inactive.insert(TreeItem::DefinedField(s, i));
                }
                if !step.kind.supports_loads() {
                    tree.closed.insert(
                        TreeItem::StepGroup(s, "Loads"),
                        "A frequency step has no loads.",
                    );
                }
                if !step.kind.supports_defined_fields() {
                    tree.closed.insert(
                        TreeItem::StepGroup(s, "Defined Fields"),
                        "A thermal step solves for the temperatures and takes no defined field.",
                    );
                }
                let active = tree.state.active_step == Some(s) && fe.steps.len() > 1;
                let label = if active {
                    format!("{} (active)", step.name)
                } else {
                    step.name.clone()
                };
                tree.branch(ui, TreeItem::Step(s), &label, true, |tree, ui| {
                    // An eigenvalue step leaves out the field outputs it has (from a
                    // project of an earlier version): the solver keeps its mode shapes.
                    let outputs = (step.field_outputs.iter().enumerate())
                        .filter(|_| !step.kind.computes_eigenmodes())
                        .map(|(i, f)| (TreeItem::FieldOutput(s, i), f.name.as_str()))
                        .collect();
                    tree.step_container(ui, s, "Field Outputs", outputs);
                    let outputs = (step.history_outputs.iter().enumerate())
                        .map(|(i, h)| (TreeItem::HistoryOutput(s, i), h.name.as_str()))
                        .collect();
                    tree.step_container(ui, s, "History Outputs", outputs);
                    let bcs = (step.boundary_conditions.iter().enumerate())
                        .map(|(i, b)| (TreeItem::BoundaryCondition(s, i), b.name.as_str()))
                        .collect();
                    tree.step_container(ui, s, "BCs", bcs);
                    let loads = (step.loads.iter().enumerate())
                        .map(|(i, l)| (TreeItem::Load(s, i), l.name.as_str()))
                        .collect();
                    tree.step_container(ui, s, "Loads", loads);
                    let fields = (step.defined_fields.iter().enumerate())
                        .map(|(i, f)| (TreeItem::DefinedField(s, i), f.name.as_str()))
                        .collect();
                    tree.step_container(ui, s, "Defined Fields", fields);
                });
            }
        });
    });
}

/// Container of the hot spot definitions.
pub const HOT_SPOTS: &str = "Hot Spot Stresses";
pub const REFERENCE_POINTS: &str = "Reference Points";
pub const COORDINATE_SYSTEMS: &str = "Coordinate Systems";
pub const PLANES: &str = "Planes";
/// Container of the result paths in the Results tree.
pub const PATHS: &str = "Paths";
/// Container of the results on planes in the Results tree.
pub const PLANE_RESULTS: &str = "Plane Results";

/// The feature a tree item stands for.
pub fn feature(item: &TreeItem) -> Option<FeatureItem> {
    let (kind, index) = match *item {
        TreeItem::ReferencePoint(i) => (FeatureKind::ReferencePoint, i),
        TreeItem::CoordinateSystem(i) => (FeatureKind::CoordinateSystem, i),
        TreeItem::Plane(i) => (FeatureKind::Plane, i),
        TreeItem::ResultPath(i) => (FeatureKind::ResultPath, i),
        TreeItem::ResultPlane(i) => (FeatureKind::ResultPlane, i),
        _ => return None,
    };
    Some(FeatureItem { kind, index })
}

/// A name with the names of its children, e.g. a field with its components.
type NamedList = (String, Vec<String>);

fn results(tree: &mut Tree, ui: &mut Ui, model: Option<&mut Model>) {
    let model = model.filter(|m| m.results.is_some());
    let mut fields = Vec::new();
    let mut active = None;
    let mut shown_plane = None;
    let mut history: Vec<(String, Vec<NamedList>)> = Vec::new();
    let hot_spots: Vec<String> = (model.iter())
        .flat_map(|m| &m.hot_spots.definitions)
        .map(|h| h.name.clone())
        .collect();
    if let Some(view) = model.as_ref().and_then(|m| m.results.as_ref()) {
        history = (view.history.iter())
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
        if let Some(increment) = view.current_increment() {
            fields = increment
                .fields
                .iter()
                .map(|f| {
                    // Components out of the plane of a 2D model are zero and not listed;
                    // the rest keep their index in the field.
                    let components: Vec<(usize, String)> = (f.components.iter().enumerate())
                        .filter(|(_, c)| !c.out_of_plane)
                        .map(|(c, component)| (c, component.name.clone()))
                        .collect();
                    let derived = view.field_outputs.iter().any(|o| o.name == f.name);
                    (f.name.clone(), components, derived)
                })
                .collect();
        }
        active = Some((view.field, view.component));
        shown_plane = view.plane_result;
    }
    let fe = model.as_ref().map(|m| m.fe.clone()).unwrap_or_default();
    tree.branch(ui, TreeItem::Model, "Model", true, |tree, ui| {
        tree.mesh(ui, model);
        tree.features(ui, &fe);
    });
    tree.branch(
        ui,
        TreeItem::Group("Results"),
        "Results",
        true,
        |tree, ui| {
            let text = counted("Field Outputs", fields.len());
            if fields.is_empty() {
                tree.leaf(ui, TreeItem::FieldOutputs, text);
            } else {
                tree.branch(ui, TreeItem::FieldOutputs, text, true, |tree, ui| {
                    for (f, (name, components, derived)) in fields.into_iter().enumerate() {
                        let item = if derived {
                            TreeItem::ResultFieldOutput(f)
                        } else {
                            TreeItem::Field(f)
                        };
                        // Nodalis opens the first two fields.
                        tree.branch(ui, item, name, f < 2, |tree, ui| {
                            for (c, component) in components {
                                let item = TreeItem::Component(f, c);
                                if tree.leaf(ui, item, component).clicked()
                                    && active != Some((f, c))
                                {
                                    tree.response.component = Some((f, c));
                                }
                            }
                        });
                    }
                });
            }
            // Empty containers are not shown; Field Outputs stays as the place the results
            // go, like Parts in the mesh.
            let group = TreeItem::Group("History Outputs");
            if !history.is_empty() {
                let text = counted("History Outputs", history.len());
                tree.branch(ui, group, text, true, |tree, ui| {
                    for (s, (name, fields)) in history.into_iter().enumerate() {
                        tree.branch(ui, TreeItem::HistorySet(s), name, true, |tree, ui| {
                            // Sets with many fields, as CalculiX prints them, start closed.
                            let open = fields.len() <= 2;
                            for (f, (name, components)) in fields.into_iter().enumerate() {
                                let item = TreeItem::HistoryField(s, f);
                                tree.branch(ui, item, name, open, |tree, ui| {
                                    for (c, component) in components.into_iter().enumerate() {
                                        let item = TreeItem::HistoryComponent(s, f, c);
                                        tree.leaf(ui, item, component);
                                    }
                                });
                            }
                        });
                    }
                });
            }
            // Hot spot stresses, defined on the results.
            let hot_spots = hot_spots.iter().enumerate();
            let items = hot_spots.map(|(i, name)| (TreeItem::HotSpot(i), name.as_str()));
            tree.container(ui, HOT_SPOTS, items.collect());
            // Results along straight lines through the model.
            let paths = (fe.result_paths.iter().enumerate())
                .map(|(i, p)| (TreeItem::ResultPath(i), p.name.as_str()))
                .collect();
            tree.container(ui, PATHS, paths);
            // The values where a plane cuts the model, shown alone
            // while checked.
            let group = TreeItem::Group(PLANE_RESULTS);
            if !fe.result_planes.is_empty() {
                let text = counted(PLANE_RESULTS, fe.result_planes.len());
                tree.branch(ui, group, text, true, |tree, ui| {
                    for (i, result) in fe.result_planes.iter().enumerate() {
                        let item = TreeItem::ResultPlane(i);
                        let icon = tree.icon(&item, false);
                        let mut shown = shown_plane == Some(i);
                        let (_, response, changed) =
                            tree.row(ui, None, Some(&mut shown), icon, item, &result.name);
                        let _ = response.on_hover_text(
                            "When checked, the 3D view shows only the section plane with the \
                             legend of the values on it.",
                        );
                        if changed {
                            tree.response.plane_result = Some(shown.then_some(i));
                        }
                    }
                });
            }
        },
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parts(state: &TreeState, view: TreeView) -> Vec<usize> {
        state.selected_parts(view).into_iter().collect()
    }

    #[test]
    fn ctrl_and_shift_select_several_parts() {
        let view = TreeView::Geometry;
        let mut state = TreeState::default();
        state.select_part(view, 1);
        state.toggle_part(view, 4);
        assert_eq!(parts(&state, view), [1, 4]);
        assert_eq!(state.selected, Some((view, TreeItem::Part(4))));
        // Shift selects from the part last clicked, Ctrl+Shift adds the range.
        state.select_range(view, 2, false);
        assert_eq!(parts(&state, view), [2, 3, 4]);
        state.select_part(view, 0);
        state.select_range(view, 1, true);
        assert_eq!(parts(&state, view), [0, 1]);
        state.toggle_part(view, 5);
        state.select_range(view, 7, true);
        assert_eq!(parts(&state, view), [0, 1, 5, 6, 7]);
        // Ctrl takes a part out again; the last one leaves nothing selected.
        state.toggle_part(view, 7);
        assert_eq!(parts(&state, view), [0, 1, 5, 6]);
        assert!(matches!(state.selected, Some((_, TreeItem::Part(_)))));
        state.select_part(view, 3);
        state.toggle_part(view, 3);
        assert_eq!(state.selected, None);
        assert!(parts(&state, view).is_empty());
    }

    #[test]
    fn ctrl_and_shift_select_several_contact_pairs() {
        let view = TreeView::FeModel;
        let siblings = [
            TreeItem::ContactPair(0),
            TreeItem::ContactPair(1),
            TreeItem::Tie(0),
            TreeItem::Tie(1),
            TreeItem::NodeTie(0),
        ];
        let mut state = TreeState::default();
        state.select_item(view, siblings[1].clone());
        // Shift selects from the item last clicked; the clicked one leads.
        state.select_sibling(view, siblings[3].clone(), &siblings, false, true);
        assert_eq!(
            state.selected_items(view),
            [
                siblings[3].clone(),
                siblings[1].clone(),
                siblings[2].clone()
            ]
        );
        // Ctrl adds one and takes one out.
        state.select_sibling(view, siblings[4].clone(), &siblings, true, false);
        state.select_sibling(view, siblings[2].clone(), &siblings, true, false);
        let mut items = state.selected_items(view);
        items.sort();
        assert_eq!(
            items,
            [
                siblings[1].clone(),
                siblings[3].clone(),
                siblings[4].clone()
            ]
        );
        assert!(state.selected_items(TreeView::Geometry).is_empty());
        // Another selection drops them; a plain click selects one alone.
        state.selected = Some((view, TreeItem::Material(0)));
        assert_eq!(state.selected_items(view), [TreeItem::Material(0)]);
        state.select_sibling(view, siblings[0].clone(), &siblings, true, false);
        assert_eq!(state.selected_items(view), [siblings[0].clone()]);
        state.select_sibling(view, siblings[0].clone(), &siblings, true, false);
        assert_eq!(state.selected, None);
    }

    #[test]
    fn other_selections_drop_the_parts() {
        let mut state = TreeState::default();
        state.select_part(TreeView::FeModel, 0);
        state.toggle_part(TreeView::FeModel, 2);
        assert!(parts(&state, TreeView::Geometry).is_empty());
        state.selected = Some((TreeView::FeModel, TreeItem::Material(0)));
        assert!(parts(&state, TreeView::FeModel).is_empty());
        // A part selected anew comes alone.
        state.toggle_part(TreeView::FeModel, 1);
        assert_eq!(parts(&state, TreeView::FeModel), [1]);
        // Shift without a part selected starts at the clicked one.
        state.selected = None;
        state.select_range(TreeView::FeModel, 3, false);
        assert_eq!(parts(&state, TreeView::FeModel), [3]);
    }

    #[test]
    fn revealed_items_open_their_branches() {
        let pair = TreeItem::ContactPair(0);
        for branch in [
            TreeItem::Model,
            TreeItem::Group("Contacts"),
            TreeItem::Group(ADVANCED_CONTACTS),
            TreeItem::Group("Contact Pairs"),
        ] {
            assert!(contains(&branch, &pair), "{branch:?}");
        }
        assert!(!contains(&TreeItem::Group("Constraints"), &pair));
        assert!(contains(
            &TreeItem::Group("Constraints"),
            &TreeItem::Constraint(1)
        ));
        assert!(contains(&TreeItem::Group("Parts"), &TreeItem::Part(2)));
    }

    #[test]
    fn hot_spots_can_be_deleted() {
        assert!(deletable(TreeView::Results, &TreeItem::HotSpot(0)));
        assert!(!deletable(TreeView::Results, &TreeItem::Part(0)));
    }

    #[test]
    fn containers_have_no_properties_dialog() {
        for item in [
            TreeItem::Mesh,
            TreeItem::Group("Parts"),
            TreeItem::StepGroup(0, "BCs"),
            TreeItem::FieldOutputs,
        ] {
            assert!(!has_properties(&item), "{item:?}");
        }
        for item in [TreeItem::Model, TreeItem::Part(0), TreeItem::Material(0)] {
            assert!(has_properties(&item), "{item:?}");
        }
    }

    /// Drives the tree with synthetic pointer events, one frame per call.
    struct Harness {
        ctx: egui::Context,
        model: Model,
        state: TreeState,
        time: f64,
        opened: Vec<TreeItem>,
    }

    impl Harness {
        fn new() -> Self {
            let mut model = Model::new(std::path::Path::new("t.inp"), plx_mesh::FeMesh::default());
            model.fe.materials = vec![
                plx_model::Material {
                    name: "A".into(),
                    ..Default::default()
                },
                plx_model::Material {
                    name: "B".into(),
                    ..Default::default()
                },
            ];
            let mut h = Self {
                ctx: egui::Context::default(),
                model,
                state: TreeState::default(),
                time: 1.0,
                opened: Vec::new(),
            };
            h.frame(vec![]);
            h.frame(vec![]);
            h
        }

        fn frame(&mut self, events: Vec<egui::Event>) {
            self.time += 0.03;
            let input = egui::RawInput {
                time: Some(self.time),
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(400.0, 600.0),
                )),
                events,
                ..Default::default()
            };
            let (model, state, opened) = (&mut self.model, &mut self.state, &mut self.opened);
            let mut out = self.ctx.run_ui(input, |ui| {
                let r = show(ui, TreeView::FeModel, Some(model), &[], &[], state);
                opened.extend(r.open);
            });
            out.textures_delta.clear();
        }

        fn button(&mut self, pos: egui::Pos2, pressed: bool) -> egui::Event {
            let _ = self;
            egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed,
                modifiers: Default::default(),
            }
        }

        /// A click: press and release in separate frames, `gap` frames between the clicks.
        fn click(&mut self, pos: egui::Pos2) {
            self.frame(vec![egui::Event::PointerMoved(pos)]);
            let e = self.button(pos, true);
            self.frame(vec![e]);
            let e = self.button(pos, false);
            self.frame(vec![e]);
        }
    }

    #[test]
    fn double_click_on_a_material_opens_it_every_time() {
        // Find the row of the first material by clicking down the tree.
        let mut found = None;
        for (x, y) in (20..300)
            .step_by(4)
            .flat_map(|y| (30..140).step_by(6).map(move |x| (x, y)))
        {
            let pos = egui::pos2(x as f32, y as f32);
            let mut h = Harness::new();
            h.click(pos);
            eprintln!("{y} {:?}", h.state.selected);
            if h.state.selected == Some((TreeView::FeModel, TreeItem::Material(0))) {
                found = Some(pos);
                break;
            }
        }
        let pos = found.expect("material row found");
        for round in 0..20 {
            let mut h = Harness::new();
            h.click(pos);
            // Frames between the clicks, as at a slower frame rate.
            for _ in 0..(round % 4) {
                h.frame(vec![]);
            }
            h.opened.clear();
            h.click(pos);
            assert_eq!(h.opened, [TreeItem::Material(0)], "round {round}");
        }
    }

    #[test]
    fn double_click_survives_a_slow_frame_after_the_first_click() {
        let mut found = None;
        for (x, y) in (20..300)
            .step_by(4)
            .flat_map(|y| (30..140).step_by(6).map(move |x| (x, y)))
        {
            let pos = egui::pos2(x as f32, y as f32);
            let mut h = Harness::new();
            h.click(pos);
            if h.state.selected == Some((TreeView::FeModel, TreeItem::Material(0))) {
                found = Some(pos);
                break;
            }
        }
        let pos = found.expect("material row found");
        // Selecting rebuilds the 3D view, which makes the frame after the first click slow;
        // egui measures the time between clicks in frame time, which then exceeds its 0.3 s.
        for slow in [0.1, 0.25, 0.35, 0.45] {
            let mut h = Harness::new();
            h.click(pos);
            h.time += slow;
            h.frame(vec![]);
            h.opened.clear();
            h.click(pos);
            assert_eq!(h.opened, [TreeItem::Material(0)], "slow frame of {slow} s");
        }
    }
}
