//! Exposed parameters: numbers of the model that the project schematic sets through value
//! links, like the "P" box next to a property in Ansys Workbench. Each names its field by a
//! [`ValuePath`], the item by its name and the field of it; the value stays in the field.

use serde::{Deserialize, Serialize};

use crate::{
    BoundaryKind, DefinedFieldKind, FeModel, InitialConditionKind, LoadKind, Quantity, SectionKind,
    StepKind, ValueKind,
};

/// The item of the model a field belongs to, by its name, and of the step it is in.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ItemRef {
    Load { step: String, name: String },
    BoundaryCondition { step: String, name: String },
    DefinedField { step: String, name: String },
    Material(String),
    Section(String),
    CrossSection(String),
    Step(String),
    InitialCondition(String),
}

impl ItemRef {
    pub fn name(&self) -> &str {
        match self {
            Self::Load { name, .. }
            | Self::BoundaryCondition { name, .. }
            | Self::DefinedField { name, .. }
            | Self::Material(name)
            | Self::Section(name)
            | Self::CrossSection(name)
            | Self::Step(name)
            | Self::InitialCondition(name) => name,
        }
    }

    /// What the item is, for messages: "Load", "Material".
    pub fn noun(&self) -> &'static str {
        match self {
            Self::Load { .. } => "Load",
            Self::BoundaryCondition { .. } => "Boundary condition",
            Self::DefinedField { .. } => "Defined field",
            Self::Material(_) => "Material",
            Self::Section(_) => "Section",
            Self::CrossSection(_) => "Cross-Section",
            Self::Step(_) => "Step",
            Self::InitialCondition(_) => "Initial condition",
        }
    }

    /// The same item after it was renamed.
    pub fn renamed(&self, new: &str) -> Self {
        let mut item = self.clone();
        match &mut item {
            Self::Load { name, .. }
            | Self::BoundaryCondition { name, .. }
            | Self::DefinedField { name, .. }
            | Self::Material(name)
            | Self::Section(name)
            | Self::CrossSection(name)
            | Self::Step(name)
            | Self::InitialCondition(name) => new.clone_into(name),
        }
        item
    }

    /// The step the item is in, or the step itself.
    pub fn step(&self) -> Option<&str> {
        match self {
            Self::Load { step, .. }
            | Self::BoundaryCondition { step, .. }
            | Self::DefinedField { step, .. } => Some(step),
            Self::Step(name) => Some(name),
            _ => None,
        }
    }

    /// The same item in the step renamed from `old` to `new`.
    fn in_renamed_step(&mut self, old: &str, new: &str) {
        match self {
            Self::Load { step, .. }
            | Self::BoundaryCondition { step, .. }
            | Self::DefinedField { step, .. }
            | Self::Step(step)
                if step == old =>
            {
                new.clone_into(step)
            }
            _ => {}
        }
    }
}

/// Which number of an item a path names.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Field {
    /// The one value of the item: a pressure, a heat flux, a temperature, a preload.
    Value,
    /// A component of a vector: of a force, a moment, gravity, a velocity, a prescribed
    /// displacement.
    X,
    Y,
    Z,
    /// A prescribed rotation about an axis.
    RotationX,
    RotationY,
    RotationZ,
    /// The rotational speed of a centrifugal load or an angular velocity.
    Speed,
    /// The sink temperature of a film or radiation.
    Sink,
    /// The film coefficient.
    FilmCoefficient,
    Emissivity,
    Density,
    Young,
    Poisson,
    Conductivity,
    SpecificHeat,
    Expansion,
    /// The thickness of a plane, shell or membrane section, the wall of a pipe.
    Thickness,
    /// The cross-section of a truss.
    Area,
    /// The dimensions of a beam profile: the sides of a rectangle or box, the radius of a
    /// circle or pipe.
    BeamA,
    BeamB,
    Radius,
    /// The walls of a box profile at +1, +2, -1 and -2.
    Wall1,
    Wall2,
    Wall3,
    Wall4,
    /// The height, flange width and thicknesses of an I, T, U or L profile.
    Height,
    Width,
    WebThickness,
    FlangeThickness,
    /// The second moments of area and the shear factor of a general profile.
    I11,
    I22,
    ShearFactor,
    TimePeriod,
    InitialIncrement,
    /// The duration of a Dynamic step.
    Duration,
    /// The result frames of a Dynamic step.
    Frames,
    /// The damping ratio of a Dynamic step.
    DampingRatio,
    /// The number of eigenfrequencies of a frequency step.
    Modes,
    /// The coordinates of the point of a remote displacement, in its coordinate system.
    LocationX,
    LocationY,
    LocationZ,
    /// The pinball radius of a remote displacement.
    Pinball,
    /// The direction of the axis of a centrifugal load or an angular velocity.
    AxisX,
    AxisY,
    AxisZ,
}

impl Field {
    /// Its name after the item's: "Force-1 X".
    pub fn label(self) -> &'static str {
        match self {
            Self::Value => "",
            Self::X => "X",
            Self::Y => "Y",
            Self::Z => "Z",
            Self::RotationX => "RX",
            Self::RotationY => "RY",
            Self::RotationZ => "RZ",
            Self::Speed => "Speed",
            Self::Sink => "Sink temperature",
            Self::FilmCoefficient => "Film coefficient",
            Self::Emissivity => "Emissivity",
            Self::Density => "Density",
            Self::Young => "Young's modulus",
            Self::Poisson => "Poisson's ratio",
            Self::Conductivity => "Conductivity",
            Self::SpecificHeat => "Specific heat",
            Self::Expansion => "Expansion",
            Self::Thickness => "Thickness",
            Self::Area => "Area",
            Self::BeamA => "a",
            Self::BeamB => "b",
            Self::Radius => "Radius",
            Self::Wall1 => "t1",
            Self::Wall2 => "t2",
            Self::Wall3 => "t3",
            Self::Wall4 => "t4",
            Self::Height => "h",
            Self::Width => "b",
            Self::WebThickness => "tw",
            Self::FlangeThickness => "tf",
            Self::I11 => "I11",
            Self::I22 => "I22",
            Self::ShearFactor => "k",
            Self::TimePeriod => "Time period",
            Self::InitialIncrement => "Initial increment",
            Self::Duration => "Duration",
            Self::Frames => "Frames",
            Self::DampingRatio => "Damping ratio",
            Self::Modes => "Modes",
            Self::LocationX => "Location X",
            Self::LocationY => "Location Y",
            Self::LocationZ => "Location Z",
            Self::Pinball => "Pinball",
            Self::AxisX => "Axis X",
            Self::AxisY => "Axis Y",
            Self::AxisZ => "Axis Z",
        }
    }

    pub const VECTOR: [Self; 3] = [Self::X, Self::Y, Self::Z];
    pub const ROTATION: [Self; 3] = [Self::RotationX, Self::RotationY, Self::RotationZ];
    pub const LOCATION: [Self; 3] = [Self::LocationX, Self::LocationY, Self::LocationZ];
    pub const AXIS: [Self; 3] = [Self::AxisX, Self::AxisY, Self::AxisZ];
    pub const ALL: [Self; 46] = [
        Self::Value,
        Self::X,
        Self::Y,
        Self::Z,
        Self::RotationX,
        Self::RotationY,
        Self::RotationZ,
        Self::Speed,
        Self::Sink,
        Self::FilmCoefficient,
        Self::Emissivity,
        Self::Density,
        Self::Young,
        Self::Poisson,
        Self::Conductivity,
        Self::SpecificHeat,
        Self::Expansion,
        Self::Thickness,
        Self::Area,
        Self::BeamA,
        Self::BeamB,
        Self::Radius,
        Self::Wall1,
        Self::Wall2,
        Self::Wall3,
        Self::Wall4,
        Self::Height,
        Self::Width,
        Self::WebThickness,
        Self::FlangeThickness,
        Self::I11,
        Self::I22,
        Self::ShearFactor,
        Self::TimePeriod,
        Self::InitialIncrement,
        Self::Duration,
        Self::Frames,
        Self::DampingRatio,
        Self::Modes,
        Self::LocationX,
        Self::LocationY,
        Self::LocationZ,
        Self::Pinball,
        Self::AxisX,
        Self::AxisY,
        Self::AxisZ,
    ];

    /// The component of a vector it names.
    fn axis(self) -> Option<usize> {
        Self::VECTOR.iter().position(|f| *f == self)
    }
}

/// A number of the model: a field of an item.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ValuePath {
    pub item: ItemRef,
    pub field: Field,
}

impl ValuePath {
    pub fn new(item: ItemRef, field: Field) -> Self {
        Self { item, field }
    }

    /// The name a new exposed parameter of it gets: "Force-1 X", "Pressure-1".
    pub fn default_name(&self) -> String {
        match self.field.label() {
            "" => self.item.name().to_owned(),
            field => format!("{} {field}", self.item.name()),
        }
    }
}

/// A field of the model exposed as a parameter: a row of the analysis' block with a pin
/// that value links set it through.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Exposed {
    /// Stays the same when the parameter is renamed, for links to it.
    pub key: u32,
    pub name: String,
    pub path: ValuePath,
}

/// The number at a place of the model, typed by what it is.
enum Slot<'a> {
    Real(&'a mut f64),
    Count(&'a mut u32),
}

/// The field of the model a path names, with its kind.
fn slot_mut<'a>(fe: &'a mut FeModel, path: &ValuePath) -> Option<(Slot<'a>, ValueKind)> {
    let field = path.field;
    let real = |value: &'a mut f64, quantity: Quantity| {
        Some((Slot::Real(value), ValueKind::Quantity(quantity)))
    };
    let step_named = |fe: &'a mut FeModel, step: &str| fe.steps.iter_mut().find(|s| s.name == step);
    match &path.item {
        ItemRef::Load { step, name } => {
            let load = (step_named(fe, step)?.loads.iter_mut()).find(|l| &l.name == name)?;
            load_slot(&mut load.kind, field)
        }
        ItemRef::BoundaryCondition { step, name } => {
            // The angle of a location in a cylindrical system is in degrees, no quantity.
            let cylindrical: Vec<String> = (fe.coordinate_systems.iter())
                .filter(|c| c.kind == crate::CoordinateSystemKind::Cylindrical)
                .map(|c| c.name.clone())
                .collect();
            let bc =
                (step_named(fe, step)?.boundary_conditions.iter_mut()).find(|b| &b.name == name)?;
            match (&mut bc.kind, field) {
                (BoundaryKind::Temperature(t), Field::Value) => real(t, Quantity::Temperature),
                (BoundaryKind::Displacement(remote), field) => match field {
                    Field::Pinball => real(remote.pinball.as_mut()?, Quantity::Length),
                    Field::LocationX | Field::LocationY | Field::LocationZ => {
                        let axis = Field::LOCATION.iter().position(|f| *f == field)?;
                        if axis == 1 && cylindrical.contains(&remote.system) {
                            return None;
                        }
                        real(&mut remote.location.as_mut()?[axis], Quantity::Length)
                    }
                    field => dof_slot(&mut remote.values, field),
                },
                _ => None,
            }
        }
        ItemRef::DefinedField { step, name } => {
            let defined =
                (step_named(fe, step)?.defined_fields.iter_mut()).find(|d| &d.name == name)?;
            match (&mut defined.kind, field) {
                (DefinedFieldKind::Temperature(t), Field::Value) => real(t, Quantity::Temperature),
                _ => None,
            }
        }
        ItemRef::Material(name) => {
            let material = fe.materials.iter_mut().find(|m| &m.name == name)?;
            match field {
                Field::Density => real(material.density.as_mut()?, Quantity::Density),
                Field::Young => real(&mut material.elastic.as_mut()?.young, Quantity::Pressure),
                Field::Poisson => Some((
                    Slot::Real(&mut material.elastic.as_mut()?.poisson),
                    ValueKind::Factor,
                )),
                Field::Conductivity => real(
                    material.conductivity.as_mut()?,
                    Quantity::ThermalConductivity,
                ),
                Field::SpecificHeat => {
                    real(material.specific_heat.as_mut()?, Quantity::SpecificHeat)
                }
                Field::Expansion => real(
                    &mut material.expansion.as_mut()?.coefficient,
                    Quantity::ThermalExpansion,
                ),
                _ => None,
            }
        }
        ItemRef::Section(name) => {
            let section = fe.sections.iter_mut().find(|s| &s.name == name)?;
            match (&mut section.kind, field) {
                (SectionKind::Solid, Field::Thickness) => {
                    real(&mut section.thickness, Quantity::Length)
                }
                (SectionKind::Truss { area }, Field::Area) => real(area, Quantity::Area),
                (
                    SectionKind::Shell { thickness, .. } | SectionKind::Membrane { thickness, .. },
                    Field::Thickness,
                ) => real(thickness, Quantity::Length),
                _ => None,
            }
        }
        ItemRef::CrossSection(name) => {
            let cross_section = fe.cross_sections.iter_mut().find(|c| &c.name == name)?;
            let dimension =
                (cross_section.profile.dimensions_mut().into_iter()).find(|d| d.field == field)?;
            match dimension.quantity {
                Some(quantity) => real(dimension.value, quantity),
                None => Some((Slot::Real(dimension.value), ValueKind::Factor)),
            }
        }
        ItemRef::Step(name) => {
            let step = step_named(fe, name)?;
            let increments = match &mut step.kind {
                StepKind::Frequency(frequency) if field == Field::Modes => {
                    return Some((
                        Slot::Count(&mut frequency.num_frequencies),
                        ValueKind::Count,
                    ));
                }
                StepKind::ComplexFrequency(complex) => {
                    return rotation_slot(
                        &mut complex.point,
                        &mut complex.axis,
                        &mut complex.speed,
                        field,
                    );
                }
                StepKind::Static(s) => s,
                StepKind::HeatTransfer(h)
                | StepKind::CoupledTempDisp(h)
                | StepKind::UncoupledTempDisp(h) => &mut h.increments,
                StepKind::Dynamic(d) => {
                    if matches!(field, Field::Modes) {
                        return match &mut d.method {
                            crate::DynamicMethod::Modal { modes } => {
                                Some((Slot::Count(modes), ValueKind::Count))
                            }
                            crate::DynamicMethod::Direct => None,
                        };
                    }
                    return match (field, &mut d.damping) {
                        (Field::Duration, _) => real(&mut d.duration, Quantity::Time),
                        (Field::Frames, _) => Some((Slot::Count(&mut d.frames), ValueKind::Count)),
                        (Field::DampingRatio, crate::Damping::Ratio { zeta, .. }) => {
                            Some((Slot::Real(zeta), ValueKind::Factor))
                        }
                        _ => None,
                    };
                }
                _ => return None,
            };
            match field {
                Field::TimePeriod => real(&mut increments.time_period, Quantity::Time),
                Field::InitialIncrement => real(&mut increments.initial_increment, Quantity::Time),
                _ => None,
            }
        }
        ItemRef::InitialCondition(name) => {
            let condition = fe.initial_conditions.iter_mut().find(|c| &c.name == name)?;
            match (&mut condition.kind, field) {
                (InitialConditionKind::Temperature(t), Field::Value) => {
                    real(t, Quantity::Temperature)
                }
                (InitialConditionKind::Velocity(v), field) => {
                    real(&mut v[field.axis()?], Quantity::Velocity)
                }
                (InitialConditionKind::AngularVelocity { point, axis, speed }, field) => {
                    rotation_slot(point, axis, speed, field)
                }
                _ => None,
            }
        }
    }
}

/// A prescribed displacement (U1..U3) or rotation (UR1..UR3); `None` when it is free.
fn dof_slot(values: &mut [Option<f64>; 6], field: Field) -> Option<(Slot<'_>, ValueKind)> {
    let (index, quantity) = match field.axis() {
        Some(axis) => (axis, Quantity::Length),
        None => {
            let axis = Field::ROTATION.iter().position(|f| *f == field)?;
            (3 + axis, Quantity::Angle)
        }
    };
    Some((
        Slot::Real(values[index].as_mut()?),
        ValueKind::Quantity(quantity),
    ))
}

/// The numbers of a rotation about an axis: the speed, a point on the axis, its direction.
fn rotation_slot<'a>(
    point: &'a mut [f64; 3],
    axis: &'a mut [f64; 3],
    speed: &'a mut f64,
    field: Field,
) -> Option<(Slot<'a>, ValueKind)> {
    let slot = |value: &'a mut f64, kind| Some((Slot::Real(value), kind));
    if field == Field::Speed {
        return slot(speed, ValueKind::Quantity(Quantity::RotationalSpeed));
    }
    if let Some(i) = Field::LOCATION.iter().position(|f| *f == field) {
        return slot(&mut point[i], ValueKind::Quantity(Quantity::Length));
    }
    let i = Field::AXIS.iter().position(|f| *f == field)?;
    slot(&mut axis[i], ValueKind::Factor)
}

fn load_slot(kind: &mut LoadKind, field: Field) -> Option<(Slot<'_>, ValueKind)> {
    fn real(value: &mut f64, quantity: Quantity) -> (Slot<'_>, ValueKind) {
        (Slot::Real(value), ValueKind::Quantity(quantity))
    }
    Some(match (kind, field) {
        (LoadKind::Pressure(p), Field::Value) => real(p, Quantity::Pressure),
        (LoadKind::Force { value, .. }, field) => real(&mut value[field.axis()?], Quantity::Force),
        (LoadKind::Moment(m), field) => real(&mut m[field.axis()?], Quantity::Moment),
        // The force along X, Y and Z, the moment about RX, RY and RZ.
        (LoadKind::RemoteForce(remote), field) => match field.axis() {
            Some(axis) => real(&mut remote.force[axis], Quantity::Force),
            None => {
                let axis = Field::ROTATION.iter().position(|f| *f == field)?;
                real(&mut remote.moment[axis], Quantity::Moment)
            }
        },
        (LoadKind::Gravity(g), field) => real(&mut g[field.axis()?], Quantity::Acceleration),
        (
            LoadKind::PreTension {
                value,
                by_displacement,
                ..
            },
            Field::Value,
        ) => {
            let quantity = if *by_displacement {
                Quantity::Length
            } else {
                Quantity::Force
            };
            real(value, quantity)
        }
        (LoadKind::Centrifugal { point, axis, speed }, field) => {
            rotation_slot(point, axis, speed, field)?
        }
        (LoadKind::ConcentratedFlux(q), Field::Value) => real(q, Quantity::Power),
        (LoadKind::SurfaceFlux(q), Field::Value) => real(q, Quantity::HeatFlux),
        (LoadKind::BodyFlux(q), Field::Value) => real(q, Quantity::PowerPerVolume),
        (LoadKind::Film { sink, .. } | LoadKind::Radiation { sink, .. }, Field::Sink) => {
            real(sink, Quantity::Temperature)
        }
        (LoadKind::Film { coefficient, .. }, Field::FilmCoefficient) => {
            real(coefficient, Quantity::HeatTransferCoefficient)
        }
        (LoadKind::Radiation { emissivity, .. }, Field::Emissivity) => {
            (Slot::Real(emissivity), ValueKind::Factor)
        }
        _ => return None,
    })
}

impl FeModel {
    /// A model of only the item `path` names, to reach its field through the one accessor
    /// without copying the whole model.
    fn item_copy(&self, item: &ItemRef) -> Option<FeModel> {
        let mut fe = FeModel::default();
        match item {
            ItemRef::BoundaryCondition { step, .. } => {
                fe.steps
                    .push(self.steps.iter().find(|s| &s.name == step)?.clone());
                // A remote displacement's location depends on the kind of its system.
                fe.coordinate_systems = self.coordinate_systems.clone();
            }
            ItemRef::Load { step, .. }
            | ItemRef::DefinedField { step, .. }
            | ItemRef::Step(step) => {
                fe.steps
                    .push(self.steps.iter().find(|s| &s.name == step)?.clone());
            }
            ItemRef::Material(name) => fe
                .materials
                .push(self.materials.iter().find(|m| &m.name == name)?.clone()),
            ItemRef::Section(name) => fe
                .sections
                .push(self.sections.iter().find(|s| &s.name == name)?.clone()),
            ItemRef::CrossSection(name) => {
                fe.cross_sections.push(self.cross_section(name)?.clone())
            }
            ItemRef::InitialCondition(name) => fe.initial_conditions.push(
                (self.initial_conditions.iter())
                    .find(|c| &c.name == name)?
                    .clone(),
            ),
        }
        Some(fe)
    }

    /// The number at `path`, in SI units; `None` when the item or field does not exist.
    pub fn value_at(&self, path: &ValuePath) -> Option<f64> {
        let mut fe = self.item_copy(&path.item)?;
        match slot_mut(&mut fe, path)?.0 {
            Slot::Real(value) => Some(*value),
            Slot::Count(value) => Some(f64::from(*value)),
        }
    }

    /// What the number at `path` is; `None` when it does not exist.
    pub fn value_kind(&self, path: &ValuePath) -> Option<ValueKind> {
        let mut fe = self.item_copy(&path.item)?;
        slot_mut(&mut fe, path).map(|(_, kind)| kind)
    }

    /// Sets the number at `path`, given in SI units; a count is rounded to a whole number of
    /// at least one. Returns whether it changed.
    pub fn set_value(&mut self, path: &ValuePath, value: f64) -> bool {
        if !value.is_finite() {
            return false;
        }
        let Some((slot, kind)) = slot_mut(self, path) else {
            return false;
        };
        let value = kind.clean(value);
        match slot {
            Slot::Real(field) if *field != value => *field = value,
            Slot::Count(field) if f64::from(*field) != value => {
                *field = value.min(f64::from(u32::MAX)) as u32
            }
            _ => return false,
        }
        true
    }

    /// The exposed parameter of `key`.
    pub fn exposed(&self, key: u32) -> Option<&Exposed> {
        self.exposed.iter().find(|e| e.key == key)
    }

    /// The fields of `item` that can be exposed: those it has now.
    pub fn exposable(&self, item: &ItemRef) -> Vec<Field> {
        let Some(mut fe) = self.item_copy(item) else {
            return Vec::new();
        };
        (Field::ALL.into_iter())
            .filter(|&field| slot_mut(&mut fe, &ValuePath::new(item.clone(), field)).is_some())
            .collect()
    }

    /// Exposes the field at `path` as a parameter; returns its key, also when it was exposed
    /// already.
    pub fn expose(&mut self, path: ValuePath) -> u32 {
        if let Some(exposed) = self.exposed.iter().find(|e| e.path == path) {
            return exposed.key;
        }
        let key = self.exposed.iter().map(|e| e.key + 1).max().unwrap_or(1);
        let base = path.default_name();
        let mut name = base.clone();
        let mut n = 2;
        while self.exposed.iter().any(|e| e.name == name) {
            name = format!("{base} ({n})");
            n += 1;
        }
        self.exposed.push(Exposed { key, name, path });
        key
    }

    /// Lets the exposed parameters follow an item that was renamed.
    pub fn rename_exposed(&mut self, old: &ItemRef, new: &str) {
        if old.name() == new {
            return;
        }
        let renamed = old.renamed(new);
        let steps = matches!(old, ItemRef::Step(_));
        for exposed in &mut self.exposed {
            if &exposed.path.item == old {
                exposed.path.item = renamed.clone();
            } else if steps {
                exposed.path.item.in_renamed_step(old.name(), new);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Load, Material, Region, Step};

    fn model() -> FeModel {
        let mut step = Step::new_static("Step-1");
        step.loads.push(Load {
            distribution: None,
            name: "Force-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![1]),
            kind: LoadKind::force([0.0, -100.0, 0.0]),
            amplitude: None,
            factor_amplitude: None,
            tables: Default::default(),
        });
        FeModel {
            materials: vec![Material {
                name: "Steel".into(),
                density: Some(7850.0),
                ..Default::default()
            }],
            steps: vec![step],
            ..Default::default()
        }
    }

    #[test]
    fn a_path_reads_and_sets_its_field() {
        let mut fe = model();
        let force = ValuePath::new(
            ItemRef::Load {
                step: "Step-1".into(),
                name: "Force-1".into(),
            },
            Field::Y,
        );
        assert_eq!(fe.value_at(&force), Some(-100.0));
        assert_eq!(
            fe.value_kind(&force),
            Some(ValueKind::Quantity(Quantity::Force))
        );
        assert!(fe.set_value(&force, -250.0));
        assert!(!fe.set_value(&force, -250.0));
        assert_eq!(fe.value_at(&force), Some(-250.0));
        let young = ValuePath::new(ItemRef::Material("Steel".into()), Field::Young);
        // Steel has no elasticity: nothing to expose there.
        assert_eq!(fe.value_at(&young), None);
        assert!(!fe.set_value(&young, 2e11));
        let missing = ValuePath::new(ItemRef::Material("Copper".into()), Field::Density);
        assert_eq!(fe.value_at(&missing), None);
    }

    #[test]
    fn rotations_expose_point_and_axis() {
        let mut fe = model();
        fe.steps[0].loads[0].kind = LoadKind::Centrifugal {
            point: [1.0, 2.0, 3.0],
            axis: [0.0, 0.0, 1.0],
            speed: 5.0,
        };
        fe.initial_conditions.push(crate::InitialCondition {
            distribution: None,
            name: "Spin".into(),
            active: true,
            region: Region::Nodes(vec![1]),
            kind: InitialConditionKind::AngularVelocity {
                point: [0.0; 3],
                axis: [1.0, 0.0, 0.0],
                speed: 1.0,
            },
        });
        let load = ItemRef::Load {
            step: "Step-1".into(),
            name: "Force-1".into(),
        };
        let spin = ItemRef::InitialCondition("Spin".into());
        for item in [&load, &spin] {
            let exposable = fe.exposable(item);
            for field in [Field::Speed, Field::LocationY, Field::AxisZ] {
                assert!(exposable.contains(&field), "{field:?}");
            }
            assert!(!exposable.contains(&Field::X));
        }
        let path = ValuePath::new(load, Field::AxisZ);
        assert_eq!(fe.value_at(&path), Some(1.0));
        assert!(fe.set_value(&path, 0.5));
        assert_eq!(fe.value_kind(&path), Some(ValueKind::Factor));
        let path = ValuePath::new(spin, Field::LocationX);
        assert!(fe.set_value(&path, 4.0));
    }

    #[test]
    fn remote_displacements_expose_their_numbers() {
        let mut fe = model();
        let mut remote =
            crate::Displacement::new([Some(0.5), None, Some(0.0), None, None, Some(0.1)]);
        remote.location = Some([1.0, 2.0, 3.0]);
        fe.steps[0]
            .boundary_conditions
            .push(crate::BoundaryCondition {
                distribution: None,
                name: "Remote-1".into(),
                active: true,
                visible: true,
                region: Region::Nodes(Vec::new()),
                kind: BoundaryKind::Displacement(remote),
                amplitude: None,
            });
        let item = ItemRef::BoundaryCondition {
            step: "Step-1".into(),
            name: "Remote-1".into(),
        };
        let exposable = fe.exposable(&item);
        for field in [
            Field::X,
            Field::Z,
            Field::RotationZ,
            Field::LocationX,
            Field::LocationY,
        ] {
            assert!(exposable.contains(&field), "{field:?}");
        }
        // Free directions and a pinball region that is off have no number to expose.
        for field in [Field::Y, Field::RotationX, Field::Pinball] {
            assert!(!exposable.contains(&field), "{field:?}");
        }
        let path = |field| ValuePath::new(item.clone(), field);
        assert_eq!(fe.value_at(&path(Field::X)), Some(0.5));
        assert!(fe.set_value(&path(Field::LocationY), 4.0));
        assert_eq!(fe.value_at(&path(Field::LocationY)), Some(4.0));
        assert_eq!(
            fe.value_kind(&path(Field::RotationZ)),
            Some(ValueKind::Quantity(Quantity::Angle))
        );
    }

    #[test]
    fn exposed_parameters_follow_renamed_items() {
        let mut fe = model();
        let item = ItemRef::Load {
            step: "Step-1".into(),
            name: "Force-1".into(),
        };
        let key = fe.expose(ValuePath::new(item.clone(), Field::Y));
        assert_eq!(fe.expose(ValuePath::new(item.clone(), Field::Y)), key);
        assert_eq!(fe.exposed(key).unwrap().name, "Force-1 Y");
        fe.rename_exposed(&item, "Load-A");
        fe.rename_exposed(&ItemRef::Step("Step-1".into()), "Pull");
        assert_eq!(
            fe.exposed(key).unwrap().path.item,
            ItemRef::Load {
                step: "Pull".into(),
                name: "Load-A".into()
            }
        );
    }
}
