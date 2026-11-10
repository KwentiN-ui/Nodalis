//! The constraint dialog: a list of constraint types above the
//! properties of the chosen one.

use egui::Ui;
use plx_mesh::FeMesh;
use plx_model::{
    CompressionOnly, Constraint, FeModel, Joint, JointAxis, JointKind, ModelSpace, PointDefinition,
    Quantity, ReferencePoint, Region, RemoteBehavior, RigidBody, Spring, SpringEnd, SpringLayout,
    UnitSystem, next_name,
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
const TYPES: [(&str, Type); 4] = [
    ("Spring", Type::Spring),
    ("Joint", Type::Joint),
    ("Compression Only", Type::CompressionOnly),
    ("Rigid Body", Type::RigidBody),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Type {
    Spring,
    Joint,
    CompressionOnly,
    RigidBody,
}

impl Type {
    fn of(constraint: &Constraint) -> Self {
        match constraint {
            Constraint::Spring(_) => Type::Spring,
            Constraint::Joint(_) => Type::Joint,
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
            Type::Joint => "Joint",
            Type::CompressionOnly => "Compression_Only",
            Type::RigidBody => "Rigid_Body",
        }
    }

    /// The constraint with the default values and empty regions.
    fn create(self, name: String) -> Constraint {
        let faces = || Region::Faces(Vec::new());
        match self {
            Type::Spring => Constraint::Spring(Spring::new(name)),
            Type::Joint => Constraint::Joint(Joint::new(name)),
            Type::CompressionOnly => Constraint::CompressionOnly(CompressionOnly {
                id: plx_model::Id::new(),
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
            // The dialog chooses the point, see `ConstraintDraft::point_exists`.
            Type::RigidBody => Constraint::RigidBody(RigidBody::new(name, plx_model::Id::new())),
        }
    }

    /// Springs and joints connect two ends.
    fn has_ends(self) -> bool {
        matches!(self, Type::Spring | Type::Joint)
    }
}

/// The two ends of a spring, the first as the slave and the second as the master side of
/// the pair: picks in the 3D view go to the one whose "..." button was pressed. The first
/// end picks nodes, the second element faces, as with selections on both ends the second
/// is the surface the springs end on; picked faces and nodes stay what they are.
fn spring_ends(faces: Target, mesh: &FeMesh, spring: Option<&Spring>) -> MasterSlave {
    ends(faces, mesh, spring.map(|s| &s.ends), Target::Nodes)
}

/// The two ends of a spring or joint; `first` is what End 1 picks by default.
fn ends(faces: Target, mesh: &FeMesh, ends: Option<&[SpringEnd; 2]>, first: Target) -> MasterSlave {
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
    let (first, second) = match ends {
        Some(e) => (draft(&e[0], first), draft(&e[1], faces)),
        None => {
            let mut first = RegionDraft::new(SPRING_END_SOURCES, first);
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
    /// The reference point a rigid body creates with it; `None` drives the body by an
    /// existing one.
    new_point: Option<Box<NewPoint>>,
}

/// Where a reference point created with a rigid body lies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Location {
    /// The area-weighted centre of the faces of the region (edges in 2D).
    Centroid,
    /// The centre of the bounding box of the region.
    BoundingBox,
    /// Global coordinates.
    Coordinates,
}

impl Location {
    const ALL: [Location; 3] = [Self::Centroid, Self::BoundingBox, Self::Coordinates];

    fn label(self) -> &'static str {
        match self {
            Self::Centroid => "Centroid of the region",
            Self::BoundingBox => "Center of the bounding box",
            Self::Coordinates => "Coordinates",
        }
    }
}

/// A reference point the rigid body dialog creates together with the body.
#[derive(Clone, Debug)]
pub(crate) struct NewPoint {
    /// The id the point gets, which the body refers to.
    id: plx_model::Id,
    name: String,
    location: Location,
    /// Global coordinates in the stored units, for [`Location::Coordinates`].
    coordinates: [f64; 3],
    /// The point as last worked out from the region and its global position, or why it has
    /// none; the form updates it when the region or the location changes.
    resolved: Option<(ReferencePoint, Result<[f64; 3], String>)>,
}

impl NewPoint {
    fn new(fe: &FeModel) -> Self {
        Self {
            id: plx_model::Id::new(),
            name: next_name("RP", fe.reference_points.iter().map(|p| p.name.as_str())),
            location: Location::Centroid,
            coordinates: [0.0; 3],
            resolved: None,
        }
    }

    /// The reference point for the region, placed on the mesh of the model.
    fn point(&self, region: &Region) -> ReferencePoint {
        let definition = match self.location {
            Location::Centroid => PointDefinition::CenterOfGravity(region.clone()),
            Location::BoundingBox => PointDefinition::BoundingBoxCenter(region.clone()),
            Location::Coordinates => PointDefinition::Global,
        };
        ReferencePoint {
            id: self.id,
            name: self.name.trim().to_string(),
            position: self.coordinates,
            definition,
        }
    }

    /// Works the point out anew when the region or the location changed.
    fn update(&mut self, region: &Region, model: &Model) {
        let mut point = self.point(region);
        let current = self.resolved.as_ref().is_some_and(|(old, _)| {
            old.definition == point.definition && old.position == point.position
        });
        if !current {
            let position = point.resolve(&model.fe, &model.mesh);
            if let Ok(p) = position {
                point.position = p;
            }
            self.resolved = Some((point, position));
        }
    }

    /// Why the point cannot be created, if it cannot.
    fn problem(&self, fe: &FeModel) -> Option<String> {
        let name = self.name.trim();
        if name.is_empty() {
            return Some("Enter a name for the new reference point.".into());
        }
        if (fe.reference_points.iter()).any(|p| p.name.eq_ignore_ascii_case(name)) {
            return Some(format!("The reference point {name} already exists."));
        }
        match &self.resolved {
            Some((_, Err(error))) => Some(error.clone()),
            _ => None,
        }
    }
}

impl ConstraintDraft {
    pub(crate) fn new(fe: &FeModel) -> Self {
        let kind = Type::Spring;
        let faces = face_target(fe);
        Self {
            constraint: kind.create(next_name(kind.prefix(), names(fe))),
            region: RegionDraft::new(FACE_SOURCES, faces),
            pair: spring_ends(faces, &FeMesh::default(), None),
            new_point: Some(Box::new(NewPoint::new(fe))),
        }
    }

    /// `faces` is what the faces of the model are, see [`face_target`].
    pub(crate) fn edit(constraint: &Constraint, faces: Target, mesh: &FeMesh) -> Self {
        let (region, pair) = match constraint {
            Constraint::Spring(spring) => (
                RegionDraft::new(FACE_SOURCES, faces),
                spring_ends(faces, mesh, Some(spring)),
            ),
            Constraint::Joint(joint) => (
                RegionDraft::new(FACE_SOURCES, faces),
                ends(faces, mesh, Some(&joint.ends), faces),
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
            new_point: None,
        }
    }

    fn kind(&self) -> Type {
        Type::of(&self.constraint)
    }

    /// The picks of a constraint of the type, which stay when another type is chosen.
    fn region_of(&self, kind: Type) -> &RegionDraft {
        if kind.has_ends() {
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
        if self.kind().has_ends() {
            self.pair.current()
        } else {
            &self.region
        }
    }

    pub(crate) fn region_mut(&mut self) -> &mut RegionDraft {
        if self.kind().has_ends() {
            self.pair.current_mut()
        } else {
            &mut self.region
        }
    }

    pub(crate) fn validate(&self, fe: &FeModel) -> Result<(), String> {
        if self.kind().has_ends() {
            return self.validate_spring();
        }
        if self.region.is_empty() {
            Err("The region is empty.".into())
        } else if let Some(point) = self.creates_point()
            && let Some(problem) = point.problem(fe)
        {
            Err(problem)
        } else if self.creates_point().is_none()
            && matches!(&self.current(), Constraint::RigidBody(b) if fe.reference_point(b.reference_point).is_none())
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
            Type::Spring | Type::Joint => self.pair.highlight(model),
            _ => self.region.highlight(model),
        }
    }

    /// The symbol of the constraint as entered, for the 3D view.
    pub(crate) fn symbol(&self) -> Option<crate::symbols::Item> {
        let (kind, region) = crate::symbols::Kind::of_constraint(&self.current())?;
        Some(crate::symbols::Item {
            kind,
            region,
            system: None,
            selected: true,
        })
    }

    /// The constraint with the regions as entered.
    pub(crate) fn finish(self) -> Constraint {
        self.current()
    }

    /// The new point of a rigid body that creates one.
    fn creates_point(&self) -> Option<&NewPoint> {
        self.new_point
            .as_deref()
            .filter(|_| self.kind() == Type::RigidBody)
    }

    /// The reference point the rigid body creates with it, to add to the model together
    /// with the constraint; it lies where the dialog showed it.
    pub(crate) fn new_point(&self) -> Option<ReferencePoint> {
        let point = self.creates_point()?;
        let mut created = point.point(&self.region.region());
        if let Some((resolved, Ok(position))) = &point.resolved
            && resolved.definition == created.definition
        {
            created.position = *position;
        }
        Some(created)
    }

    fn current(&self) -> Constraint {
        let mut constraint = self.constraint.clone();
        match &mut constraint {
            Constraint::Spring(spring) => {
                spring.ends = [spring_end(&self.pair.slave), spring_end(&self.pair.master)];
            }
            Constraint::Joint(joint) => {
                joint.ends = [spring_end(&self.pair.slave), spring_end(&self.pair.master)];
            }
            _ => {
                for slot in constraint.regions_mut() {
                    *slot = self.region.region();
                }
                if let (Constraint::RigidBody(body), Some(point)) =
                    (&mut constraint, self.creates_point())
                {
                    body.reference_point = point.id;
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
            Constraint::Joint(joint) => joint_rows(ui, model, joint, &mut self.pair),
            Constraint::CompressionOnly(support) => {
                self.region.ui(ui, model);
                compression_only_rows(ui, support, units);
            }
            Constraint::RigidBody(body) => {
                self.region.ui(ui, model);
                let region = (!self.region.is_empty()).then(|| self.region.region());
                rigid_body_point_rows(ui, model, body, &mut self.new_point, region.as_ref());
                coupling_behavior_row(ui, &mut body.behavior);
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
        let (draft, dropped) = if kind.has_ends() {
            // A spring's first end takes nodes, or faces where the picks are some; a
            // joint's faces.
            let first = if kind == Type::Joint {
                faces
            } else {
                Target::Nodes
            };
            let (mut draft, dropped) = match picks.converted(SPRING_END_SOURCES, first, mesh) {
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
        if kind.has_ends() {
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
    // A plane model turns about z only, an axisymmetric one not at all.
    let rotations: &[usize] = match model.fe.properties.space {
        ModelSpace::ThreeD => &[0, 1, 2],
        ModelSpace::PlaneStress | ModelSpace::PlaneStrain => &[2],
        ModelSpace::Axisymmetric => &[],
    };
    if spring.takes_rotations() {
        rotational_rows(ui, &mut spring.rotational, units, rotations);
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

/// The relative degrees of freedom of a joint, in its frame.
const JOINT_DOFS: [&str; 6] = ["Ux", "Uy", "Uz", "URx", "URy", "URz"];

/// The two ends of a joint, its kind and, for a general joint, the free degrees of
/// freedom, how a selection follows its point, and the joint frame.
fn joint_rows(ui: &mut Ui, model: &Model, joint: &mut Joint, pair: &mut MasterSlave) {
    let fe = &model.fe;
    let units = fe.properties.units;
    let first = pair.side == Side::Slave;
    if pair.slave.ui_labeled(ui, model, "End 1", "slave", first) {
        pair.side = Side::Slave;
    }
    if pair.master.ui_labeled(ui, model, "End 2", "master", !first) {
        pair.side = Side::Master;
    }
    joint.ends = [spring_end(&pair.slave), spring_end(&pair.master)];
    ui.label("Joint type");
    egui::ComboBox::from_id_salt("joint kind")
        .selected_text(joint.kind.label())
        .width(crate::dock::fit_width(ui, 200.0))
        .show_ui(ui, |ui| {
            for kind in JointKind::ALL {
                ui.selectable_value(&mut joint.kind, kind, kind.label());
            }
        });
    ui.end_row();
    let dofs = fe.properties.space.remote_dofs();
    if joint.kind == JointKind::General {
        for &d in dofs {
            ui.label(JOINT_DOFS[d]);
            ui.horizontal(|ui| {
                ui.radio_value(&mut joint.free[d], false, "Fixed");
                ui.radio_value(&mut joint.free[d], true, "Free");
            });
            ui.end_row();
        }
    } else {
        let free = joint.free_dofs();
        let names: Vec<&str> = (dofs.iter())
            .filter(|&&d| free[d])
            .map(|&d| JOINT_DOFS[d])
            .collect();
        ui.label("Free");
        ui.label(if names.is_empty() {
            "None".to_string()
        } else {
            names.join(", ")
        });
        ui.end_row();
    }
    if joint.ends.iter().any(|end| end.selection().is_some()) {
        ui.label("Behavior");
        ui.horizontal(|ui| {
            ui.radio_value(&mut joint.behavior, RemoteBehavior::Rigid, "Rigid")
                .on_hover_text("The selection keeps its shape at the joint.");
            ui.radio_value(
                &mut joint.behavior,
                RemoteBehavior::Deformable,
                "Deformable",
            )
            .on_hover_text("The joint follows the mean motion of the selection.");
        });
        ui.end_row();
    }
    crate::setup::system_row(ui, &mut joint.system, fe);
    let system = fe.coordinate_system_or_global(joint.system);
    let cylindrical =
        (system.as_ref()).is_some_and(|s| s.kind == plx_model::CoordinateSystemKind::Cylindrical);
    ui.label("Location");
    ui.horizontal_wrapped(|ui| {
        let centre = if joint.ends[0].region().is_some() {
            "Centre of End 1"
        } else {
            "Centre of End 2"
        };
        if ui.radio(joint.location.is_none(), centre).clicked() {
            joint.location = None;
        }
        if ui.radio(joint.location.is_some(), "Coordinates").clicked() && joint.location.is_none() {
            // Starts from the centre, in the coordinates of the system.
            let centre = (fe.joint_origin(&model.mesh, joint)).unwrap_or([0.0; 3]);
            let local = (system.as_ref())
                .and_then(|s| s.local(centre).ok())
                .unwrap_or(centre);
            joint.location = Some(if cylindrical {
                [local[0], local[1].to_degrees(), local[2]]
            } else {
                local
            });
        }
    });
    ui.end_row();
    if let Some(location) = &mut joint.location {
        let labels = if cylindrical {
            ["R", "Theta", "Z"]
        } else {
            ["X", "Y", "Z"]
        };
        let count = if fe.properties.space.is_2d() { 2 } else { 3 };
        for (k, (value, label)) in location.iter_mut().zip(labels).take(count).enumerate() {
            ui.label(label);
            if cylindrical && k == 1 {
                ui.add(numeric::drag_value(value).speed(1.0).suffix(" °"));
            } else {
                ui.add(numeric::quantity(value, units, Quantity::Length).speed(0.1));
            }
            ui.end_row();
        }
    }
    ui.label("Joint");
    match fe.joint_origin(&model.mesh, joint) {
        Ok(origin) => {
            let [x, y, z] = origin
                .map(|v| crate::results::format_value(units.shown(v, Quantity::Length) as f32));
            let unit = units.unit(Quantity::Length);
            ui.label(format!("({x}, {y}, {z}) {unit}").trim_end())
                .on_hover_text("Global position of the joint");
        }
        Err(_) if pair.slave.is_empty() && pair.master.is_empty() => {
            ui.weak("Choose the ends");
        }
        Err(error) => {
            ui.add(
                egui::Label::new(egui::RichText::new(error).color(crate::style::error())).wrap(),
            );
        }
    }
    ui.end_row();
    if !fe.properties.space.is_2d() {
        axis_rows(ui, model, joint);
    }
    let axis = if fe.properties.space.is_2d() {
        "The joint turns about the axis normal to the model plane."
    } else {
        "The axis is the axis of a revolute, cylindrical or slider joint and the normal of a \
         planar joint (z of the joint frame). Face normal and Cylinder axis take the faces of \
         End 1 (of End 2 when End 1 has none)."
    };
    hint(
        ui,
        &format!(
            "End 2 moves against End 1 in the free degrees of freedom only. {axis} The joint \
             frame turns with End 1."
        ),
    );
}

/// The joint axis: where it comes from and the direction it has.
fn axis_rows(ui: &mut Ui, model: &Model, joint: &mut Joint) {
    let fe = &model.fe;
    ui.label("Axis");
    egui::ComboBox::from_id_salt("joint axis")
        .selected_text(joint.axis.label())
        .width(crate::dock::fit_width(ui, 200.0))
        .show_ui(ui, |ui| {
            for choice in JointAxis::CHOICES {
                if ui
                    .selectable_label(joint.axis.same_kind(choice), choice.label())
                    .clicked()
                    && !joint.axis.same_kind(choice)
                {
                    joint.axis = choice;
                }
            }
        });
    ui.end_row();
    if let JointAxis::Direction(direction) = &mut joint.axis {
        for (value, label) in direction.iter_mut().zip(["X", "Y", "Z"]) {
            ui.label(format!("Axis {label}"));
            ui.add(numeric::drag_value(value).speed(0.01));
            ui.end_row();
        }
    }
    ui.label("Axis direction");
    match fe.joint_frame(&model.mesh, joint) {
        Ok((_, axes)) => {
            let [x, y, z] = axes[2].map(|v| crate::results::format_value(v as f32));
            ui.label(format!("({x}, {y}, {z})"))
                .on_hover_text("Global direction of the joint axis");
        }
        // The origin's error shows in the Joint row.
        Err(_) if fe.joint_origin(&model.mesh, joint).is_err() => {
            ui.weak("Choose the ends");
        }
        Err(error) => {
            ui.add(
                egui::Label::new(egui::RichText::new(error).color(crate::style::error())).wrap(),
            );
        }
    }
    ui.end_row();
}

/// Rotational stiffness about the global axes, in moment per radian.
fn rotational_rows(ui: &mut Ui, rotational: &mut [f64; 3], units: UnitSystem, axes: &[usize]) {
    let unit = format!(" {}/rad", units.unit(Quantity::Moment));
    for &axis in axes {
        let (value, label) = (&mut rotational[axis], ["KR1", "KR2", "KR3"][axis]);
        ui.label(label)
            .on_hover_text("Rotational stiffness about the global axis.");
        ui.add(
            numeric::without_unit(value, units, Quantity::Moment)
                .suffix(unit.as_str())
                .speed(1.0)
                .range(0.0..=f64::MAX),
        );
        ui.end_row();
    }
}

/// How the selection of a rigid body follows its reference point.
pub(crate) fn coupling_behavior_row(ui: &mut Ui, behavior: &mut RemoteBehavior) {
    ui.label("Behavior");
    ui.horizontal(|ui| {
        ui.radio_value(behavior, RemoteBehavior::Rigid, "Rigid")
            .on_hover_text("The selection keeps its shape and moves with the point.");
        ui.radio_value(behavior, RemoteBehavior::Deformable, "Deformable")
            .on_hover_text(
                "The point follows the mean motion of the selection, which keeps its own \
                 stiffness. Suits a point that tracks where a face goes.",
            );
    });
    ui.end_row();
}

/// The reference point that drives a rigid body: an existing one, or a new one created with
/// the body at the centroid of its region, the centre of its bounding box or coordinates.
fn rigid_body_point_rows(
    ui: &mut Ui,
    model: &Model,
    body: &mut RigidBody,
    new_point: &mut Option<Box<NewPoint>>,
    region: Option<&Region>,
) {
    let points = &model.fe.reference_points;
    if new_point.is_none() && !points.iter().any(|p| p.id == body.reference_point) {
        match points.first() {
            Some(first) => body.reference_point = first.id,
            None => *new_point = Some(Box::new(NewPoint::new(&model.fe))),
        }
    }
    ui.label("Reference point");
    let selected = match new_point {
        Some(_) => "New point".to_string(),
        None => model.fe.name_or_missing(Some(body.reference_point)),
    };
    egui::ComboBox::from_id_salt("rigid body point")
        .selected_text(selected)
        .width(crate::dock::fit_width(ui, 200.0))
        .show_ui(ui, |ui| {
            if ui
                .selectable_label(new_point.is_some(), "New point")
                .clicked()
                && new_point.is_none()
            {
                *new_point = Some(Box::new(NewPoint::new(&model.fe)));
            }
            for point in points {
                let chosen = new_point.is_none() && body.reference_point == point.id;
                if ui.selectable_label(chosen, &point.name).clicked() {
                    *new_point = None;
                    body.reference_point = point.id;
                }
            }
        });
    ui.end_row();
    if let Some(point) = new_point {
        ui.label("Point name");
        ui.add(egui::TextEdit::singleline(&mut point.name).desired_width(200.0));
        ui.end_row();
        ui.label("Location");
        egui::ComboBox::from_id_salt("rigid body point location")
            .selected_text(point.location.label())
            .width(crate::dock::fit_width(ui, 200.0))
            .show_ui(ui, |ui| {
                for location in Location::ALL {
                    ui.selectable_value(&mut point.location, location, location.label());
                }
            });
        ui.end_row();
        let (stored, shown) = (model.stored_units(), model.units());
        if point.location == Location::Coordinates {
            let axes = if model.fe.properties.space.is_2d() {
                &["X", "Y"][..]
            } else {
                &["X", "Y", "Z"][..]
            };
            for (k, axis) in axes.iter().enumerate() {
                ui.label(format!("    {axis}"));
                ui.add(
                    numeric::converted(&mut point.coordinates[k], stored, shown, Quantity::Length)
                        .suffix(format!(" {}", shown.unit(Quantity::Length)))
                        .speed(1.0)
                        .max_decimals(6),
                );
                ui.end_row();
            }
        } else {
            ui.label("Position");
            let Some(region) = region else {
                ui.weak("Select the region first.");
                ui.end_row();
                return;
            };
            point.update(region, model);
            match point.resolved.as_ref().map(|(_, position)| position) {
                Some(Ok(p)) => {
                    let p = p.map(|c| model.shown(c, Quantity::Length));
                    ui.weak(format!(
                        "({}, {}, {}) {}",
                        crate::results::format_value(p[0] as f32),
                        crate::results::format_value(p[1] as f32),
                        crate::results::format_value(p[2] as f32),
                        shown.unit(Quantity::Length)
                    ));
                }
                Some(Err(error)) => {
                    ui.colored_label(crate::style::error(), error);
                }
                None => {}
            }
            ui.end_row();
        }
        hint(
            ui,
            "The point is created under Features > Reference Points when you click OK, \
             where it can be edited later. Boundary conditions, forces and moments on it \
             drive the body.",
        );
    } else {
        hint(
            ui,
            "The region is coupled to the reference point as Behavior says. Boundary \
             conditions, forces and moments on the point drive the region.",
        );
    }
}

fn names(fe: &FeModel) -> impl Iterator<Item = &str> {
    fe.constraints.iter().map(Constraint::name)
}

fn name_row(ui: &mut Ui, constraint: &mut Constraint) {
    let name = match constraint {
        Constraint::Spring(c) => &mut c.name,
        Constraint::Joint(c) => &mut c.name,
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
    fn a_rigid_body_creates_its_reference_point() {
        let model = empty_model();
        let mut fe = FeModel::default();
        fe.reference_points
            .push(ReferencePoint::new("RP-1", [0.0; 3]));
        let mut draft = ConstraintDraft::new(&fe);
        draft.switch(Type::RigidBody, &[], &model);
        draft.region.take(
            &model.mesh,
            Items::Faces([(1, 2)].into()),
            Operation::Replace,
        );
        let point = draft.new_point.as_mut().unwrap();
        assert_eq!(point.name, "RP-2");
        point.location = Location::Coordinates;
        point.coordinates = [1.0, 2.0, 3.0];
        assert_eq!(draft.validate(&fe), Ok(()));
        let created = draft.new_point().unwrap();
        assert_eq!(created.name, "RP-2");
        assert_eq!(created.position, [1.0, 2.0, 3.0]);
        assert_eq!(created.definition, PointDefinition::Global);
        // A centroid follows the region of the body.
        draft.new_point.as_mut().unwrap().location = Location::Centroid;
        assert_eq!(
            draft.new_point().unwrap().definition,
            PointDefinition::CenterOfGravity(Region::Faces(vec![(1, 2)]))
        );
        // The name of an existing point is taken.
        draft.new_point.as_mut().unwrap().name = "rp-1".into();
        assert!(draft.validate(&fe).is_err());
        draft.new_point.as_mut().unwrap().name = "Hub".into();
        let created = draft.new_point().unwrap();
        assert_eq!(created.name, "Hub");
        let Constraint::RigidBody(body) = draft.finish() else {
            unreachable!()
        };
        assert_eq!(body.reference_point, created.id);
    }

    #[test]
    fn an_edited_rigid_body_keeps_its_point() {
        let mut body = RigidBody::new("Rigid_Body-1", crate::test_ids::named("RP-1"));
        body.region = Region::Faces(vec![(1, 2)]);
        let draft = ConstraintDraft::edit(
            &Constraint::RigidBody(body.clone()),
            Target::Faces,
            &FeMesh::default(),
        );
        assert!(draft.new_point().is_none());
        assert_eq!(draft.finish(), Constraint::RigidBody(body));
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
                SpringEnd::Region(Region::RemotePoint(crate::test_ids::named("Hinge"))),
            ],
            rotational: [0.0, 0.0, 5.0],
            ..Spring::new("Torsion")
        });
        let mut support = Type::CompressionOnly.create("Compression_Only-1".into());
        *support.regions_mut().remove(0) = Region::Surface("FLOOR".into());
        for constraint in [connection, spring, torsion, support] {
            let draft = ConstraintDraft::edit(&constraint, Target::Faces, &mesh);
            assert_eq!(draft.validate(&FeModel::default()), Ok(()));
            assert_eq!(draft.finish(), constraint);
        }
    }
}
