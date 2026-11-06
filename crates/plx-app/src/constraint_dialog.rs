//! The constraint dialog: a list of constraint types above the
//! properties of the chosen one.

use egui::Ui;
use plx_mesh::FeMesh;
use plx_model::{
    CompressionOnly, Constraint, FeModel, Quantity, Region, RigidBody, Spring, SpringEnd,
    SpringLayout, UnitSystem, next_name,
};

use crate::contacts::{MasterSlave, Side};
use crate::model::{Highlight, Model};
use crate::numeric;
use crate::selection::Target;
use crate::setup::{
    FACE_SOURCES, RegionDraft, SPRING_END_SOURCES, Source, dropped_note, face_target,
    show_dropped_note,
};

/// The fixed width of the dialog's content, so it does not change with the type.
pub(crate) const WIDTH: f32 = 440.0;

/// The list of constraint types, its point and surface springs merged into one spring
/// that also connects two surfaces.
const TYPES: [(&str, Type); 3] = [
    ("Spring", Type::Spring),
    ("Compression Only", Type::CompressionOnly),
    ("Rigid Body", Type::RigidBody),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Type {
    Spring,
    CompressionOnly,
    RigidBody,
}

impl Type {
    fn of(constraint: &Constraint) -> Self {
        match constraint {
            Constraint::Spring(_) => Type::Spring,
            Constraint::CompressionOnly(_) => Type::CompressionOnly,
            Constraint::RigidBody(_) => Type::RigidBody,
            // Moved to the ties and node ties when the project was read; never edited here.
            Constraint::Tie(_) => unreachable!("ties are not constraints"),
            Constraint::NodeTie(_) => unreachable!("node ties are not constraints"),
        }
    }

    /// The default name without the number.
    fn prefix(self) -> &'static str {
        match self {
            Type::Spring => "Spring",
            Type::CompressionOnly => "Compression_Only",
            Type::RigidBody => "Rigid_Body",
        }
    }

    /// The constraint with the default values and empty regions.
    fn create(self, name: String) -> Constraint {
        let faces = || Region::Faces(Vec::new());
        match self {
            Type::Spring => Constraint::Spring(Spring::new(name)),
            Type::CompressionOnly => Constraint::CompressionOnly(CompressionOnly {
                name,
                active: true,
                visible: true,
                region: faces(),
                clearance: 0.0,
                spring_stiffness: None,
                tensile_force: None,
                each_node: false,
                flipped: false,
                offset: 0.0,
                nonlinear: false,
            }),
            Type::RigidBody => Constraint::RigidBody(RigidBody::new(name, "")),
        }
    }
}

/// The two ends of a spring, the first as the slave and the second as the master side of
/// the pair: picks in the 3D view go to the one whose "..." button was pressed. The first
/// end picks nodes, the second element faces, as with selections on both ends the second
/// is the surface the springs end on; picked faces and nodes stay what they are.
fn spring_ends(faces: Target, mesh: &FeMesh, spring: Option<&Spring>) -> MasterSlave {
    let draft = |end: &SpringEnd, target: Target| match end {
        SpringEnd::World => RegionDraft::new(SPRING_END_SOURCES, target),
        SpringEnd::Region(region) => {
            let target = match region {
                Region::Faces(_) => faces,
                Region::Nodes(_) => Target::Nodes,
                _ => target,
            };
            RegionDraft::from_region(region, SPRING_END_SOURCES, target, mesh)
        }
    };
    let (first, second) = match spring {
        Some(s) => (draft(&s.ends[0], Target::Nodes), draft(&s.ends[1], faces)),
        None => {
            let mut first = RegionDraft::new(SPRING_END_SOURCES, Target::Nodes);
            first.source = Source::Selection;
            (first, RegionDraft::new(SPRING_END_SOURCES, faces))
        }
    };
    MasterSlave {
        master: second,
        slave: first,
        side: Side::Slave,
    }
}

/// The end of a spring a draft stands for.
fn spring_end(draft: &RegionDraft) -> SpringEnd {
    if draft.is_world() {
        SpringEnd::World
    } else {
        SpringEnd::Region(draft.region())
    }
}

/// A constraint while its dialog is open.
pub(crate) struct ConstraintDraft {
    pub(crate) constraint: Constraint,
    /// The region of a support or rigid body.
    region: RegionDraft,
    /// The first (slave) and second end (master) of a spring.
    pair: MasterSlave,
}

impl ConstraintDraft {
    pub(crate) fn new(fe: &FeModel) -> Self {
        let kind = Type::Spring;
        let faces = face_target(fe);
        Self {
            constraint: kind.create(next_name(kind.prefix(), names(fe))),
            region: RegionDraft::new(FACE_SOURCES, faces),
            pair: spring_ends(faces, &FeMesh::default(), None),
        }
    }

    /// `faces` is what the faces of the model are, see [`face_target`].
    pub(crate) fn edit(constraint: &Constraint, faces: Target, mesh: &FeMesh) -> Self {
        let (region, pair) = match constraint {
            Constraint::Spring(spring) => (
                RegionDraft::new(FACE_SOURCES, faces),
                spring_ends(faces, mesh, Some(spring)),
            ),
            _ => (
                RegionDraft::from_region(constraint.regions()[0], FACE_SOURCES, faces, mesh),
                spring_ends(faces, mesh, None),
            ),
        };
        Self {
            constraint: constraint.clone(),
            region,
            pair,
        }
    }

    fn kind(&self) -> Type {
        Type::of(&self.constraint)
    }

    /// The picks of a constraint of the type, which stay when another type is chosen.
    fn region_of(&self, kind: Type) -> &RegionDraft {
        if kind == Type::Spring {
            self.pair.current()
        } else {
            &self.region
        }
    }

    pub(crate) fn name(&self) -> &str {
        self.constraint.name()
    }

    /// The region that clicks in the 3D view pick.
    pub(crate) fn region(&self) -> &RegionDraft {
        if self.kind() == Type::Spring {
            self.pair.current()
        } else {
            &self.region
        }
    }

    pub(crate) fn region_mut(&mut self) -> &mut RegionDraft {
        if self.kind() == Type::Spring {
            self.pair.current_mut()
        } else {
            &mut self.region
        }
    }

    pub(crate) fn validate(&self) -> Result<(), String> {
        if self.kind() == Type::Spring {
            return self.validate_spring();
        }
        if self.region.is_empty() {
            Err("The region is empty.".into())
        } else if matches!(&self.constraint, Constraint::RigidBody(b) if b.reference_point.is_empty())
        {
            Err("Choose the reference point that drives the rigid body.".into())
        } else {
            Ok(())
        }
    }

    fn validate_spring(&self) -> Result<(), String> {
        let ends = [&self.pair.slave, &self.pair.master];
        if ends.iter().all(|end| end.is_world()) {
            return Err("Choose a selection or a point for at least one end.".into());
        }
        for (number, end) in (1..).zip(ends) {
            if end.is_empty() {
                return Err(format!("End {number} is empty."));
            }
        }
        let [first, second] = ends.map(RegionDraft::region);
        if self.pair.slave.is_point() && self.pair.master.is_point() && first == second {
            return Err("Both ends are the same point.".into());
        }
        Ok(())
    }

    /// The region in the primary highlight colour; for a spring its second end in the
    /// primary and its first end in the secondary colour.
    pub(crate) fn highlight(&self, model: &Model) -> Highlight {
        match self.kind() {
            Type::Spring => self.pair.highlight(model),
            _ => self.region.highlight(model),
        }
    }

    /// The symbol of the constraint as entered, for the 3D view.
    pub(crate) fn symbol(&self) -> Option<crate::symbols::Item> {
        let (kind, region) = crate::symbols::Kind::of_constraint(&self.current())?;
        Some(crate::symbols::Item {
            kind,
            region,
            selected: true,
        })
    }

    /// The constraint with the regions as entered.
    pub(crate) fn finish(self) -> Constraint {
        self.current()
    }

    fn current(&self) -> Constraint {
        let mut constraint = self.constraint.clone();
        match &mut constraint {
            Constraint::Spring(spring) => {
                spring.ends = [spring_end(&self.pair.slave), spring_end(&self.pair.master)];
            }
            _ => {
                for slot in constraint.regions_mut() {
                    *slot = self.region.region();
                }
            }
        }
        constraint
    }

    pub(crate) fn form(&mut self, ui: &mut Ui, model: &Model, taken: &[&str], creating: bool) {
        let current = self.kind();
        ui.label("Type");
        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.set_width(200.0);
            ui.vertical(|ui| {
                for (label, kind) in TYPES {
                    let selected = kind == current;
                    let response =
                        ui.add_enabled(creating, egui::Button::selectable(selected, label));
                    let response = if selected {
                        response
                    } else {
                        response.on_disabled_hover_text(
                            "The type of a constraint cannot be changed when editing.",
                        )
                    };
                    if response.clicked() && !selected {
                        let dropped = self.switch(kind, taken, model);
                        dropped_note(ui, dropped);
                    }
                }
            });
        });
        ui.end_row();
        show_dropped_note(ui);
        hint(
            ui,
            "Ties are not constraints: create them with FE Model > Connections > Tie, they are \
             listed under Contact Pairs.",
        );
        name_row(ui, &mut self.constraint);
        let units = model.fe.properties.units;
        // A 2D model has no z direction, and ccx crashes on springs in it.
        let dofs = if model.fe.properties.space.is_2d() {
            2
        } else {
            3
        };
        match &mut self.constraint {
            Constraint::Spring(spring) => spring_rows(ui, model, spring, &mut self.pair, dofs),
            Constraint::CompressionOnly(support) => {
                self.region.ui(ui, model);
                compression_only_rows(ui, support, units);
            }
            Constraint::RigidBody(body) => {
                self.region.ui(ui, model);
                ui.label("Reference point");
                let points = &model.fe.reference_points;
                if body.reference_point.is_empty()
                    && let Some(first) = points.first()
                {
                    body.reference_point = first.name.clone();
                }
                if points.is_empty() {
                    ui.weak("The model has no reference points (Features).");
                } else {
                    egui::ComboBox::from_id_salt("rigid body point")
                        .selected_text(body.reference_point.as_str())
                        .width(crate::dock::fit_width(ui, 200.0))
                        .show_ui(ui, |ui| {
                            for point in points {
                                ui.selectable_value(
                                    &mut body.reference_point,
                                    point.name.clone(),
                                    &point.name,
                                );
                            }
                        });
                }
                ui.end_row();
                hint(
                    ui,
                    "The nodes of the region move as one rigid body with the reference \
                     point. Boundary conditions, forces and moments on the point drive the \
                     body.",
                );
            }
            Constraint::Tie(_) => unreachable!("ties are not constraints"),
            Constraint::NodeTie(_) => unreachable!("node ties are not constraints"),
        }
    }

    /// Another type was picked in the list: its defaults, keeping a name the user chose and
    /// the picks the new type takes.
    fn switch(&mut self, kind: Type, taken: &[&str], model: &Model) -> bool {
        let old = self.kind();
        let default_name = (self.constraint.name().strip_prefix(old.prefix()))
            .and_then(|n| n.strip_prefix('-'))
            .is_some_and(|n| n.parse::<u32>().is_ok());
        let name = if default_name {
            next_name(kind.prefix(), taken.iter().copied())
        } else {
            self.constraint.name().to_owned()
        };
        self.constraint = kind.create(name);
        let picks = self.region_of(old).clone();
        if picks.is_empty() || picks.is_world() {
            return false;
        }
        let mesh = &model.mesh;
        let faces = face_target(&model.fe);
        let (draft, dropped) = if kind == Type::Spring {
            // A spring's first end takes nodes, or faces where the picks are some.
            let (mut draft, dropped) =
                match picks.converted(SPRING_END_SOURCES, Target::Nodes, mesh) {
                    (_, true) => picks.converted(SPRING_END_SOURCES, faces, mesh),
                    kept => kept,
                };
            if draft.is_world() {
                draft.source = Source::Selection;
            }
            (draft, dropped)
        } else {
            picks.converted(FACE_SOURCES, faces, mesh)
        };
        if kind == Type::Spring {
            self.pair.slave = draft;
        } else {
            self.region = draft;
        }
        dropped
    }
}

/// The two ends of a spring, each ground ("World"), a selection or a point, and its
/// stiffness: translational between selections, translational and rotational between
/// points and ground.
fn spring_rows(
    ui: &mut Ui,
    model: &Model,
    spring: &mut Spring,
    pair: &mut MasterSlave,
    dofs: usize,
) {
    let units = model.fe.properties.units;
    let first = pair.side == Side::Slave;
    if pair.slave.ui_labeled(ui, model, "End 1", "slave", first) {
        pair.side = Side::Slave;
    }
    if pair.master.ui_labeled(ui, model, "End 2", "master", !first) {
        pair.side = Side::Master;
    }
    spring.ends = [spring_end(&pair.slave), spring_end(&pair.master)];
    // Which hint applies, without holding on to the spring.
    let shape = spring.layout().map(|layout| match layout {
        SpringLayout::SelectionToWorld { .. } => 0,
        SpringLayout::SelectionToSurface { .. } => 1,
        SpringLayout::SelectionToPoint { .. } => 2,
        SpringLayout::PointToWorld { .. } => 3,
        SpringLayout::PointToPoint { .. } => 4,
    });
    let selection = spring.selection().is_some();
    if selection {
        ui.label("Stiffness");
        ui.add_enabled_ui(!spring.each_node, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.radio_value(&mut spring.per_area, false, "Total");
                ui.radio_value(&mut spring.per_area, true, "Per area")
                    .on_hover_text("Stiffness per area of the faces of the selection.");
            });
        });
        ui.end_row();
    }
    let quantity = stiffness(spring.is_per_area());
    stiffness_rows(ui, &mut spring.stiffness, units, quantity, dofs);
    // A plane model has no rotations at points.
    if spring.takes_rotations() && dofs == 3 {
        rotational_rows(ui, &mut spring.rotational, units);
    }
    if selection {
        ui.label("");
        ui.checkbox(&mut spring.each_node, "Apply to all nodes individually");
        ui.end_row();
    }
    let distribution = if spring.each_node {
        "Every node of the selection gets the whole stiffness."
    } else if spring.is_per_area() {
        "The stiffness per area is spread over the nodes of the faces by area."
    } else {
        "The stiffness of the whole selection is spread over its nodes like a force: on \
         faces by area, on edges by length, on points in equal shares."
    };
    let text = match shape {
        None => "Choose a selection or a point for at least one end.".to_string(),
        Some(0) => {
            format!("{distribution} The springs act in the global directions to ground.")
        }
        Some(1) => format!(
            "{distribution} Each node of End 1 is connected by springs in the global \
             directions to the closest point of the faces of End 2."
        ),
        Some(2) => format!(
            "{distribution} Each node is connected by springs in the global directions to \
             the point. Rotational springs need a point or World on both ends."
        ),
        Some(3) => "Springs in the global directions hold \
             the translations (K) and rotations (KR) of the point to ground. Points are \
             reference points of rigid bodies and the points of Displacement/Rotation \
             conditions and Remote Forces on faces: with a rotation left free such a \
             condition and this spring make a hinge with a torsion spring."
            .to_string(),
        Some(_) => "Springs in the global directions connect the translations (K) and \
             rotations (KR) of the two points."
            .to_string(),
    };
    hint(ui, &text);
}

/// Rotational stiffness about the global axes, in moment per radian.
fn rotational_rows(ui: &mut Ui, rotational: &mut [f64; 3], units: UnitSystem) {
    let unit = format!("{}/rad", units.unit(Quantity::Moment));
    for (value, label) in rotational.iter_mut().zip(["KR1", "KR2", "KR3"]) {
        ui.label(label)
            .on_hover_text("Rotational stiffness about the global axis.");
        ui.horizontal(|ui| {
            ui.add(
                numeric::without_unit(value, units, Quantity::Moment)
                    .speed(1.0)
                    .range(0.0..=f64::MAX),
            );
            ui.label(unit.as_str());
        });
        ui.end_row();
    }
}

fn names(fe: &FeModel) -> impl Iterator<Item = &str> {
    fe.constraints.iter().map(Constraint::name)
}

fn name_row(ui: &mut Ui, constraint: &mut Constraint) {
    let name = match constraint {
        Constraint::Spring(c) => &mut c.name,
        Constraint::CompressionOnly(c) => &mut c.name,
        Constraint::RigidBody(c) => &mut c.name,
        Constraint::Tie(_) => unreachable!("ties are not constraints"),
        Constraint::NodeTie(_) => unreachable!("node ties are not constraints"),
    };
    ui.label("Name");
    ui.add(egui::TextEdit::singleline(name).desired_width(200.0));
    ui.end_row();
}

/// Springs have a stiffness per length, or per length and area.
fn stiffness(per_area: bool) -> Quantity {
    if per_area {
        Quantity::ForcePerVolume
    } else {
        Quantity::ForcePerLength
    }
}

fn stiffness_rows(
    ui: &mut Ui,
    stiffness: &mut [f64; 3],
    units: UnitSystem,
    of: Quantity,
    dofs: usize,
) {
    for (value, label) in stiffness.iter_mut().zip(["K1", "K2", "K3"]).take(dofs) {
        ui.label(label);
        ui.add(
            numeric::quantity(value, units, of)
                .speed(1.0)
                .range(0.0..=f64::MAX),
        );
        ui.end_row();
    }
}

fn compression_only_rows(ui: &mut Ui, support: &mut CompressionOnly, units: UnitSystem) {
    ui.label("Direction");
    ui.horizontal(|ui| {
        if ui
            .button("Flip")
            .on_hover_text("Swaps the side the support pushes from.")
            .clicked()
        {
            support.flipped = !support.flipped;
        }
        ui.label(if support.flipped {
            "Pushes outward, from inside the material"
        } else {
            "Pushes onto the surface, from outside"
        });
    });
    ui.end_row();
    length_row(ui, "Clearance", &mut support.clearance, units);
    optional_row(
        ui,
        "Spring stiffness",
        &mut support.spring_stiffness,
        CompressionOnly::DEFAULT_STIFFNESS,
        (units, Quantity::ForcePerLength),
    );
    optional_row(
        ui,
        "Tensile force",
        &mut support.tensile_force,
        CompressionOnly::DEFAULT_TENSILE_FORCE,
        (units, Quantity::Force),
    );
    ui.label("");
    ui.checkbox(&mut support.each_node, "Apply to all nodes individually");
    ui.end_row();
    length_row(ui, "Offset", &mut support.offset, units);
    ui.label("Nonlinear");
    ui.checkbox(&mut support.nonlinear, "")
        .on_hover_text("Otherwise the support is linearized in a linear step.");
    ui.end_row();
    hint(
        ui,
        if support.each_node {
            "Gap elements carry compression only; the small tensile force in the open state \
             keeps the model solvable. Stiffness and tensile force act whole at every node."
        } else {
            "Gap elements carry compression only; the small tensile force in the open state \
             keeps the model solvable. Stiffness and tensile force are those of the whole \
             surface, spread over its nodes by area. Without a value every gap gets \
             the solver's default, practically rigid."
        },
    );
}

fn length_row(ui: &mut Ui, label: &str, value: &mut f64, units: UnitSystem) {
    ui.label(label);
    ui.add(numeric::quantity(value, units, Quantity::Length).speed(0.01));
    ui.end_row();
}

/// A value left at its default until the user enters one.
fn optional_row(
    ui: &mut Ui,
    label: &str,
    value: &mut Option<f64>,
    default: f64,
    (units, quantity): (UnitSystem, Quantity),
) {
    let mut set = value.is_some();
    ui.checkbox(&mut set, label);
    ui.horizontal(|ui| {
        let mut number = value.unwrap_or(default);
        let unit = units.unit(quantity);
        ui.add_enabled(
            set,
            numeric::without_unit(&mut number, units, quantity)
                .speed(0.0)
                .custom_formatter(|v, _| numeric::format_physical(v)),
        );
        ui.label(if set { unit } else { "Default" });
        *value = set.then_some(number);
    });
    ui.end_row();
}

fn hint(ui: &mut Ui, text: &str) {
    ui.label("");
    ui.add(egui::Label::new(egui::RichText::new(text).weak()).wrap());
    ui.end_row();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::selection::{Items, Operation};

    fn empty_model() -> Model {
        Model::new(std::path::Path::new("test.inp"), FeMesh::default())
    }

    #[test]
    fn switching_the_type_keeps_the_picks_the_new_type_takes() {
        let model = empty_model();
        let mut draft = ConstraintDraft::new(&model.fe);
        draft
            .pair
            .slave
            .take(&model.mesh, Items::Nodes([1, 2].into()), Operation::Replace);
        // Nodes are no faces: the compression only support starts empty.
        assert!(draft.switch(Type::CompressionOnly, &[], &model));
        assert!(draft.region.is_empty());
        draft.region.take(
            &model.mesh,
            Items::Faces([(1, 2)].into()),
            Operation::Replace,
        );
        // Faces are the spring's region now, and back.
        assert!(!draft.switch(Type::Spring, &[], &model));
        assert_eq!(draft.pair.slave.region(), Region::Faces(vec![(1, 2)]));
        assert!(!draft.switch(Type::RigidBody, &[], &model));
        assert_eq!(draft.region.region(), Region::Faces(vec![(1, 2)]));
    }

    #[test]
    fn switching_the_type_proposes_its_default_name() {
        let mut fe = FeModel::default();
        fe.constraints.push(Type::Spring.create("Spring-1".into()));
        let mut draft = ConstraintDraft::new(&fe);
        assert_eq!(draft.name(), "Spring-2");
        assert_eq!(draft.pair.slave.target, Target::Nodes);
        draft.switch(Type::CompressionOnly, &["Spring-1"], &empty_model());
        assert_eq!(draft.name(), "Compression_Only-1");
        assert_eq!(draft.region.target, Target::Faces);
        *draft.constraint.name_mut() = "Lager".into();
        draft.switch(Type::RigidBody, &[], &empty_model());
        assert_eq!(draft.name(), "Lager");
    }

    #[test]
    fn faces_of_2d_models_are_picked_as_edges() {
        let mut fe = FeModel::default();
        fe.properties.space = plx_model::ModelSpace::PlaneStrain;
        let mut draft = ConstraintDraft::new(&fe);
        assert_eq!(draft.pair.master.target, Target::Edges);
        draft.switch(Type::CompressionOnly, &[], &empty_model());
        assert_eq!(draft.region.target, Target::Edges);
    }

    #[test]
    fn regions_survive_the_dialog() {
        let mesh = FeMesh::default();
        let connection = Constraint::Spring(Spring {
            active: false,
            ends: [
                SpringEnd::Region(Region::Faces(vec![(1, 2)])),
                SpringEnd::Region(Region::Surface("BORE".into())),
            ],
            stiffness: [1.0, 2.0, 3.0],
            per_area: true,
            ..Spring::new("Lager")
        });
        let spring = Constraint::Spring(Spring {
            stiffness: [1.0, 0.0, 0.0],
            each_node: true,
            ..Spring::to_world("Spring-1", Region::NodeSet("FIX".into()), [0.0; 3])
        });
        // A torsion spring from the point of a boundary condition to ground, given with
        // World first.
        let torsion = Constraint::Spring(Spring {
            ends: [
                SpringEnd::World,
                SpringEnd::Region(Region::RemotePoint("Hinge".into())),
            ],
            rotational: [0.0, 0.0, 5.0],
            ..Spring::new("Torsion")
        });
        let mut support = Type::CompressionOnly.create("Compression_Only-1".into());
        *support.regions_mut().remove(0) = Region::Surface("FLOOR".into());
        for constraint in [connection, spring, torsion, support] {
            let draft = ConstraintDraft::edit(&constraint, Target::Faces, &mesh);
            assert_eq!(draft.validate(), Ok(()));
            assert_eq!(draft.finish(), constraint);
        }
    }
}
