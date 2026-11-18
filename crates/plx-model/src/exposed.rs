//! Exposed parameters: numbers of the model that the project schematic sets through value
//! links, like the "P" box next to a property in Ansys Workbench. Each names its field by a
//! [`ValuePath`], the item by its name and the field of it; the value stays in the field.

use serde::{Deserialize, Serialize};

use plx_mesh::{FeMesh, MeshTransform};

use crate::{
    BoundaryKind, DefinedFieldKind, FeModel, Geometry, InitialConditionKind, LoadKind,
    MeshOperationKind, MeshSetupKind, MeshingParameters, Quantity, SectionKind, StepKind,
    ValueKind,
};

/// The item of the model a field belongs to, by its name, and of the step it is in.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ItemRef {
    Load {
        step: String,
        name: String,
    },
    BoundaryCondition {
        step: String,
        name: String,
    },
    DefinedField {
        step: String,
        name: String,
    },
    Material(String),
    Section(String),
    CrossSection(String),
    Step(String),
    InitialCondition(String),
    /// A mesh operation of Mesh > Mesh Operations, such as a transform.
    MeshOperation(String),
    /// A distribution, whose variables can be exposed.
    Distribution(String),
    /// A mesh setup item of the geometry, such as Meshing Parameters or a Local Mesh Size.
    MeshSetup(String),
    /// The default meshing parameters of the geometry, for the parts no Meshing Parameters
    /// item covers.
    DefaultMeshing,
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
            | Self::InitialCondition(name)
            | Self::MeshOperation(name)
            | Self::Distribution(name)
            | Self::MeshSetup(name) => name,
            Self::DefaultMeshing => "Default meshing",
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
            Self::MeshOperation(_) => "Mesh operation",
            Self::Distribution(_) => "Distribution",
            Self::MeshSetup(_) => "Mesh setup item",
            Self::DefaultMeshing => "Meshing",
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
            | Self::InitialCondition(name)
            | Self::MeshOperation(name)
            | Self::Distribution(name)
            | Self::MeshSetup(name) => new.clone_into(name),
            Self::DefaultMeshing => {}
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

    /// Whether its numbers are of the geometry's mesh setup rather than of the FE model.
    pub fn is_meshing(&self) -> bool {
        matches!(self, Self::MeshSetup(_) | Self::DefaultMeshing)
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
    /// The diameters, fill factor and bending stiffness of a rope.
    Diameter,
    WireDiameter,
    FillFactor,
    RopeBending,
    /// The duration of a static, heat transfer, visco or Dynamic step.
    Duration,
    /// The result frames of a static, heat transfer, visco or Dynamic step.
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
    /// The angle of a rotation of mesh parts, in degrees.
    Angle,
    /// The normal of the plane a mirror of mesh parts reflects at.
    NormalX,
    NormalY,
    NormalZ,
    /// The factors of a scaling of mesh parts along X, Y and Z.
    ScaleX,
    ScaleY,
    ScaleZ,
    /// The infill density of an infill section, as a fraction.
    InfillDensity,
    /// The top and bottom thickness of an infill section (its wall thickness is
    /// [`Field::Thickness`]).
    TopBottomThickness,
    /// The values of the flow of a material in Particle Flow steps: Herschel-Bulkley,
    /// Newtonian and granular. The angles are in degrees, numbers without quantity.
    YieldStress,
    Consistency,
    FlowIndex,
    Viscosity,
    FrictionAngle,
    LimitAngle,
    Grain,
    FineGrain,
    FineFraction,
    InertialNumber,
    /// The meshing parameters of a Meshing Parameters item or of the default ones: the
    /// element sizes, the elements per curvature radius and the options, 1 for on and 0 for
    /// off.
    MaxElementSize,
    MinElementSize,
    ElementsPerCurvature,
    SecondOrder,
    MidsideNodes,
    Optimize,
    Quadrilaterals,
    Hexahedra,
    /// The size and depth of a Local Mesh Size.
    ElementSize,
    Depth,
    /// A named variable of a distribution, by its key ([`crate::DistributionVariable::key`]).
    Variable(u32),
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
            Self::Diameter => "d",
            Self::WireDiameter => "Wire diameter",
            Self::FillFactor => "Fill factor",
            Self::RopeBending => "Bending",
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
            Self::Angle => "Angle",
            Self::NormalX => "Normal X",
            Self::NormalY => "Normal Y",
            Self::NormalZ => "Normal Z",
            Self::ScaleX => "Factor X",
            Self::ScaleY => "Factor Y",
            Self::ScaleZ => "Factor Z",
            Self::InfillDensity => "Infill density",
            Self::TopBottomThickness => "Top/bottom thickness",
            Self::YieldStress => "Yield stress",
            Self::Consistency => "Consistency",
            Self::FlowIndex => "Flow index",
            Self::Viscosity => "Viscosity",
            Self::FrictionAngle => "Friction angle",
            Self::LimitAngle => "Limit angle",
            Self::Grain => "Grain diameter",
            Self::FineGrain => "Fine grain diameter",
            Self::FineFraction => "Fine fraction",
            Self::InertialNumber => "I0",
            Self::MaxElementSize => "Max. element size",
            Self::MinElementSize => "Min. element size",
            Self::ElementsPerCurvature => "Elements per curvature radius",
            Self::SecondOrder => "Second order",
            Self::MidsideNodes => "Midside nodes on geometry",
            Self::Optimize => "Optimize",
            Self::Quadrilaterals => "Quadrilaterals",
            Self::Hexahedra => "Hexahedra",
            Self::ElementSize => "Size",
            Self::Depth => "Depth",
            Self::Variable(_) => "Variable",
        }
    }

    pub const VECTOR: [Self; 3] = [Self::X, Self::Y, Self::Z];
    pub const ROTATION: [Self; 3] = [Self::RotationX, Self::RotationY, Self::RotationZ];
    pub const LOCATION: [Self; 3] = [Self::LocationX, Self::LocationY, Self::LocationZ];
    pub const AXIS: [Self; 3] = [Self::AxisX, Self::AxisY, Self::AxisZ];
    pub const NORMAL: [Self; 3] = [Self::NormalX, Self::NormalY, Self::NormalZ];
    pub const SCALE: [Self; 3] = [Self::ScaleX, Self::ScaleY, Self::ScaleZ];
    pub const ALL: [Self; 77] = [
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
        Self::Diameter,
        Self::WireDiameter,
        Self::FillFactor,
        Self::RopeBending,
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
        Self::Angle,
        Self::NormalX,
        Self::NormalY,
        Self::NormalZ,
        Self::ScaleX,
        Self::ScaleY,
        Self::ScaleZ,
        Self::InfillDensity,
        Self::TopBottomThickness,
        Self::YieldStress,
        Self::Consistency,
        Self::FlowIndex,
        Self::Viscosity,
        Self::FrictionAngle,
        Self::LimitAngle,
        Self::Grain,
        Self::FineGrain,
        Self::FineFraction,
        Self::InertialNumber,
        Self::MaxElementSize,
        Self::MinElementSize,
        Self::ElementsPerCurvature,
        Self::SecondOrder,
        Self::MidsideNodes,
        Self::Optimize,
        Self::Quadrilaterals,
        Self::Hexahedra,
        Self::ElementSize,
        Self::Depth,
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
    /// An option, 1 for on and 0 for off.
    Flag(&'a mut bool),
}

impl Slot<'_> {
    fn get(&self) -> f64 {
        match self {
            Slot::Real(value) => **value,
            Slot::Count(value) => f64::from(**value),
            Slot::Flag(value) => f64::from(u8::from(**value)),
        }
    }

    /// Sets it to `value`, cleaned for `kind`; returns whether it changed. An option is on
    /// from one half.
    fn set(self, kind: ValueKind, value: f64) -> bool {
        let value = kind.clean(value);
        match self {
            Slot::Real(field) if *field != value => *field = value,
            Slot::Count(field) if f64::from(*field) != value => {
                *field = value.min(f64::from(u32::MAX)) as u32
            }
            Slot::Flag(field) if *field != (value >= 0.5) => *field = value >= 0.5,
            _ => return false,
        }
        true
    }
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
            let cylindrical: Vec<Option<crate::Id>> = (fe.coordinate_systems.iter())
                .filter(|c| c.kind == crate::CoordinateSystemKind::Cylindrical)
                .map(|c| Some(c.id))
                .collect();
            let bc =
                (step_named(fe, step)?.boundary_conditions.iter_mut()).find(|b| &b.name == name)?;
            match (&mut bc.kind, field) {
                (BoundaryKind::Temperature(t), Field::Value) => real(t, Quantity::Temperature),
                (BoundaryKind::Flow(flow), Field::Value) => match &mut flow.kind {
                    crate::particle_flow::FlowKind::Pressure(p) => real(p, Quantity::Pressure),
                    crate::particle_flow::FlowKind::Velocity(v) => real(v, Quantity::Velocity),
                    crate::particle_flow::FlowKind::Side(_) => None,
                },
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
                field => flow_slot(material, field),
            }
        }
        ItemRef::Section(name) => {
            let section = fe.sections.iter_mut().find(|s| &s.name == name)?;
            match (&mut section.kind, field) {
                (SectionKind::Solid, Field::Thickness) => {
                    real(&mut section.thickness, Quantity::Length)
                }
                (SectionKind::Truss { area }, Field::Area) => real(area, Quantity::Area),
                (SectionKind::Infill(infill), Field::Thickness) => {
                    real(&mut infill.wall_thickness, Quantity::Length)
                }
                (SectionKind::Infill(infill), Field::TopBottomThickness) => {
                    real(&mut infill.top_bottom_thickness, Quantity::Length)
                }
                (SectionKind::Infill(infill), Field::InfillDensity) => {
                    Some((Slot::Real(&mut infill.density), ValueKind::Factor))
                }
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
                StepKind::Static(s) | StepKind::Visco(crate::ViscoStep { increments: s, .. }) => s,
                StepKind::HeatTransfer(h)
                | StepKind::CoupledTempDisp(h)
                | StepKind::UncoupledTempDisp(h) => &mut h.increments,
                StepKind::Dynamic(d) => {
                    if matches!(field, Field::Modes) {
                        return match &mut d.method {
                            crate::DynamicMethod::Modal { modes, .. } => {
                                Some((Slot::Count(modes), ValueKind::Count))
                            }
                            crate::DynamicMethod::Direct
                            | crate::DynamicMethod::Explicit
                            | crate::DynamicMethod::Automatic => None,
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
                Field::Duration => real(&mut increments.duration, Quantity::Time),
                Field::Frames => Some((Slot::Count(&mut increments.frames), ValueKind::Count)),
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
        ItemRef::MeshOperation(name) => {
            let operation = fe.mesh_operations.iter_mut().find(|o| &o.name == name)?;
            match &mut operation.kind {
                MeshOperationKind::Transform(transform) => transform_slot(transform, field),
            }
        }
        // The variables are numbers of the equation, in the units the equation takes them.
        ItemRef::Distribution(name) => {
            let Field::Variable(key) = field else {
                return None;
            };
            let distribution = fe.distributions.iter_mut().find(|d| &d.name == name)?;
            let variable = distribution.variables.iter_mut().find(|v| v.key == key)?;
            Some((Slot::Real(&mut variable.value), ValueKind::Factor))
        }
        // Numbers of the geometry's mesh setup, see [`Geometry::value_at`].
        ItemRef::MeshSetup(_) | ItemRef::DefaultMeshing => None,
    }
}

/// The field of the geometry's mesh setup a path names, with its kind.
fn meshing_slot_mut<'a>(
    geometry: &'a mut Geometry,
    path: &ValuePath,
) -> Option<(Slot<'a>, ValueKind)> {
    let field = path.field;
    match &path.item {
        ItemRef::DefaultMeshing => parameters_slot(&mut geometry.meshing, field),
        ItemRef::MeshSetup(name) => {
            let item = geometry.mesh_items.iter_mut().find(|i| &i.name == name)?;
            match &mut item.kind {
                MeshSetupKind::MeshingParameters { parameters, .. } => {
                    parameters_slot(parameters, field)
                }
                MeshSetupKind::LocalMeshSize { size, depth, .. } => {
                    let length = ValueKind::Quantity(Quantity::Length);
                    match field {
                        Field::ElementSize => Some((Slot::Real(size), length)),
                        Field::Depth => Some((Slot::Real(depth), length)),
                        _ => None,
                    }
                }
                _ => None,
            }
        }
        _ => None,
    }
}

/// The numbers of meshing parameters: sizes, elements per curvature radius and options.
fn parameters_slot(
    parameters: &mut MeshingParameters,
    field: Field,
) -> Option<(Slot<'_>, ValueKind)> {
    let length = ValueKind::Quantity(Quantity::Length);
    let flag = |value| Some((Slot::Flag(value), ValueKind::Factor));
    match field {
        Field::MaxElementSize => Some((Slot::Real(&mut parameters.max_size), length)),
        Field::MinElementSize => Some((Slot::Real(&mut parameters.min_size), length)),
        Field::ElementsPerCurvature => Some((
            Slot::Real(&mut parameters.elements_per_curvature),
            ValueKind::Factor,
        )),
        Field::SecondOrder => flag(&mut parameters.second_order),
        Field::MidsideNodes => flag(&mut parameters.midside_nodes_on_geometry),
        Field::Optimize => flag(&mut parameters.optimize),
        Field::Quadrilaterals => flag(&mut parameters.quad_dominated),
        Field::Hexahedra => flag(&mut parameters.hexahedra_where_possible),
        _ => None,
    }
}

/// Whether a number of the mesh setup is one Gmsh can mesh with: element sizes greater than
/// zero, no negative minimum size, depth or elements per curvature radius.
fn meshing_value_valid(field: Field, value: f64) -> bool {
    match field {
        Field::MaxElementSize | Field::ElementSize => value > 0.0,
        Field::MinElementSize | Field::Depth | Field::ElementsPerCurvature => value >= 0.0,
        _ => true,
    }
}

impl Geometry {
    /// The number of its mesh setup at `path`, in SI units; `None` when the item or field
    /// does not exist, or the path names a number of the FE model.
    pub fn value_at(&self, path: &ValuePath) -> Option<f64> {
        let mut copy = self.setup_only();
        meshing_slot_mut(&mut copy, path).map(|(slot, _)| slot.get())
    }

    /// What the number of its mesh setup at `path` is.
    pub fn value_kind(&self, path: &ValuePath) -> Option<ValueKind> {
        let mut copy = self.setup_only();
        meshing_slot_mut(&mut copy, path).map(|(_, kind)| kind)
    }

    /// Sets the number of its mesh setup at `path`, given in SI units; returns whether it
    /// changed. A size Gmsh cannot mesh with, such as zero, is not taken. The parts it
    /// meshes are out of date after a change, see [`crate::fingerprint::mesh_input`].
    pub fn set_value(&mut self, path: &ValuePath, value: f64) -> bool {
        if !value.is_finite() || !meshing_value_valid(path.field, value) {
            return false;
        }
        match meshing_slot_mut(self, path) {
            Some((slot, kind)) => slot.set(kind, value),
            None => false,
        }
    }

    /// The fields of a mesh setup item, or of the default meshing parameters, that can be
    /// exposed.
    pub fn exposable(&self, item: &ItemRef) -> Vec<Field> {
        let mut copy = self.setup_only();
        (Field::ALL.into_iter())
            .filter(|&field| {
                meshing_slot_mut(&mut copy, &ValuePath::new(item.clone(), field)).is_some()
            })
            .collect()
    }

    /// A geometry of its mesh setup alone, without the shapes, to reach a field through the
    /// one accessor or to compare setups without copying the shapes.
    pub fn setup_only(&self) -> Geometry {
        Geometry {
            source: String::new(),
            brep: String::new(),
            meshing: self.meshing,
            mesh_items: self.mesh_items.clone(),
            part_names: Vec::new(),
            refinement: self.refinement,
            signatures: Vec::new(),
            compounds: Vec::new(),
            suppressed: Vec::new(),
            own_meshes: Vec::new(),
            ids: Vec::new(),
            from_link: false,
        }
    }
}

/// The numbers of the flow of a material; a value that follows a temperature table has no
/// number of its own.
fn flow_slot(material: &mut crate::Material, field: Field) -> Option<(Slot<'_>, ValueKind)> {
    use crate::particle_flow::Flow;
    let real = |value, quantity| Some((Slot::Real(value), ValueKind::Quantity(quantity)));
    let factor = |value| Some((Slot::Real(value), ValueKind::Factor));
    let tables = &material.flow_tables;
    let mixture =
        matches!(material.flow, Some(Flow::Granular { fine_grain, .. }) if fine_grain > 0.0);
    match (material.flow.as_mut()?, field) {
        (Flow::HerschelBulkley { yield_stress, .. }, Field::YieldStress)
            if tables.yield_stress.is_none() =>
        {
            real(yield_stress, Quantity::Pressure)
        }
        (Flow::HerschelBulkley { consistency, .. }, Field::Consistency)
            if tables.consistency.is_none() =>
        {
            real(consistency, Quantity::Consistency)
        }
        (Flow::HerschelBulkley { exponent, .. }, Field::FlowIndex) => factor(exponent),
        (Flow::Newtonian { viscosity }, Field::Viscosity) if tables.viscosity.is_none() => {
            real(viscosity, Quantity::Viscosity)
        }
        (Flow::Granular { friction_angle, .. }, Field::FrictionAngle) => factor(friction_angle),
        (Flow::Granular { limit_angle, .. }, Field::LimitAngle) => factor(limit_angle),
        (Flow::Granular { grain, .. }, Field::Grain) => real(grain, Quantity::Length),
        (Flow::Granular { fine_grain, .. }, Field::FineGrain) if mixture => {
            real(fine_grain, Quantity::Length)
        }
        (Flow::Granular { fine_fraction, .. }, Field::FineFraction) if mixture => {
            factor(fine_fraction)
        }
        (Flow::Granular { inertial, .. }, Field::InertialNumber) => factor(inertial),
        _ => None,
    }
}

/// The numbers of a transform of mesh parts. Its angle is in degrees as the dialog shows it,
/// a number without quantity like the directions and the scale factors.
fn transform_slot(transform: &mut MeshTransform, field: Field) -> Option<(Slot<'_>, ValueKind)> {
    let index = |fields: [Field; 3]| fields.iter().position(|f| *f == field);
    let length = |value| Some((Slot::Real(value), ValueKind::Quantity(Quantity::Length)));
    let factor = |value| Some((Slot::Real(value), ValueKind::Factor));
    match transform {
        MeshTransform::Translate(d) => length(&mut d[index(Field::VECTOR)?]),
        MeshTransform::Rotate { point, axis, angle } => match field {
            Field::Angle => factor(angle),
            _ => match index(Field::LOCATION) {
                Some(i) => length(&mut point[i]),
                None => factor(&mut axis[index(Field::AXIS)?]),
            },
        },
        MeshTransform::Mirror { point, normal } => match index(Field::LOCATION) {
            Some(i) => length(&mut point[i]),
            None => factor(&mut normal[index(Field::NORMAL)?]),
        },
        MeshTransform::Scale { center, factors } => match index(Field::LOCATION) {
            Some(i) => length(&mut center[i]),
            None => factor(&mut factors[index(Field::SCALE)?]),
        },
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
        (LoadKind::Force { value, .. }, field)
        | (LoadKind::Excitation(crate::Excitation { value, .. }), field) => {
            real(&mut value[field.axis()?], Quantity::Force)
        }
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
        (LoadKind::Flux { value, .. }, Field::Value) => real(value, Quantity::Power),
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
            ItemRef::CrossSection(name) => fe
                .cross_sections
                .push(self.cross_section_named(name)?.clone()),
            ItemRef::InitialCondition(name) => fe.initial_conditions.push(
                (self.initial_conditions.iter())
                    .find(|c| &c.name == name)?
                    .clone(),
            ),
            ItemRef::MeshOperation(name) => fe.mesh_operations.push(
                (self.mesh_operations.iter())
                    .find(|o| &o.name == name)?
                    .clone(),
            ),
            ItemRef::Distribution(name) => fe
                .distributions
                .push(self.distribution_named(name)?.clone()),
            ItemRef::MeshSetup(_) | ItemRef::DefaultMeshing => return None,
        }
        Some(fe)
    }

    /// The number at `path`, in SI units; `None` when the item or field does not exist.
    pub fn value_at(&self, path: &ValuePath) -> Option<f64> {
        let mut fe = self.item_copy(&path.item)?;
        slot_mut(&mut fe, path).map(|(slot, _)| slot.get())
    }

    /// The number at `path` of this model or, for a number of the mesh setup, of
    /// `geometry`, the geometry it meshes.
    pub fn value_in(&self, geometry: Option<&Geometry>, path: &ValuePath) -> Option<f64> {
        match path.item.is_meshing() {
            true => geometry?.value_at(path),
            false => self.value_at(path),
        }
    }

    /// What the number at `path` of this model or of `geometry` is, see [`Self::value_in`].
    pub fn value_kind_in(
        &self,
        geometry: Option<&Geometry>,
        path: &ValuePath,
    ) -> Option<ValueKind> {
        match path.item.is_meshing() {
            true => geometry?.value_kind(path),
            false => self.value_kind(path),
        }
    }

    /// What the number at `path` is; `None` when it does not exist.
    pub fn value_kind(&self, path: &ValuePath) -> Option<ValueKind> {
        let mut fe = self.item_copy(&path.item)?;
        slot_mut(&mut fe, path).map(|(_, kind)| kind)
    }

    /// Sets the number at `path`, given in SI units; a count is rounded to a whole number of
    /// at least one. Returns whether it changed. A number of a mesh operation changes only
    /// the operation here, not the mesh it was applied to: see [`Self::set_value_meshed`].
    pub fn set_value(&mut self, path: &ValuePath, value: f64) -> bool {
        if !value.is_finite() {
            return false;
        }
        let Some((slot, kind)) = slot_mut(self, path) else {
            return false;
        };
        slot.set(kind, value)
    }

    /// Sets the number at `path` like [`Self::set_value`]; a number of a mesh operation
    /// changes `mesh`, which has the operations applied, along with it: the operations from
    /// that one on are undone and applied again. A value that makes the operation invalid,
    /// such as a scale factor of zero, is not taken.
    pub fn set_value_meshed(&mut self, mesh: &mut FeMesh, path: &ValuePath, value: f64) -> bool {
        let ItemRef::MeshOperation(name) = &path.item else {
            return self.set_value(path, value);
        };
        let mut copy = FeModel {
            mesh_operations: self.mesh_operations.clone(),
            ..Default::default()
        };
        if !copy.set_value(path, value) {
            return false;
        }
        let valid =
            (copy.mesh_operations.iter().find(|o| &o.name == name)).is_some_and(|o| {
                match &o.kind {
                    MeshOperationKind::Transform(t) => t.problem().is_none(),
                }
            });
        if valid {
            self.set_mesh_operations(mesh, copy.mesh_operations);
        }
        valid
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
        let variables = (fe.distributions.iter())
            .flat_map(|d| d.variables.iter().map(|v| Field::Variable(v.key)))
            .collect::<Vec<_>>();
        (Field::ALL.into_iter().chain(variables))
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
        let base = match (&path.item, path.field) {
            // "Pressure map p1"
            (ItemRef::Distribution(name), Field::Variable(key)) => {
                match self.distribution_named(name).and_then(|d| d.variable(key)) {
                    Some(variable) => format!("{name} {}", variable.name),
                    None => path.default_name(),
                }
            }
            _ => path.default_name(),
        };
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

    /// The values of a flow are numbers of the material, except one that follows a table
    /// over the temperature.
    #[test]
    fn flow_values_are_exposed_unless_they_follow_a_table() {
        use crate::particle_flow::Flow;
        let mut fe = FeModel {
            materials: vec![Material {
                name: "Oil".into(),
                flow: Some(Flow::Newtonian { viscosity: 0.08 }),
                ..Material::default()
            }],
            ..FeModel::default()
        };
        let item = ItemRef::Material("Oil".into());
        assert_eq!(fe.exposable(&item), vec![Field::Viscosity]);
        let path = ValuePath::new(item.clone(), Field::Viscosity);
        assert_eq!(
            fe.value_kind(&path),
            Some(ValueKind::Quantity(Quantity::Viscosity))
        );
        assert!(fe.set_value(&path, 0.1));
        assert_eq!(fe.value_at(&path), Some(0.1));
        fe.materials[0].flow_tables.viscosity = Some(crate::ValueTable {
            points: vec![[20.0, 0.1]],
        });
        assert!(fe.exposable(&item).is_empty());
        fe.materials[0].flow = Some(Flow::KINDS[2]);
        assert_eq!(
            fe.exposable(&item),
            vec![
                Field::FrictionAngle,
                Field::LimitAngle,
                Field::Grain,
                Field::InertialNumber
            ]
        );
    }

    fn model() -> FeModel {
        let mut step = Step::new_static("Step-1");
        step.loads.push(Load {
            id: crate::Id::new(),
            distribution: None,
            name: "Force-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![1]),
            kind: LoadKind::force([0.0, -100.0, 0.0]),
            amplitude: None,
            factor_amplitude: None,
            tables: Default::default(),
            system: None,
            follower: false,
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
                id: crate::Id::new(),
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
    fn a_transform_exposes_its_numbers_and_moves_the_mesh() {
        use plx_mesh::{Element, ElementShape, Part};
        let mut mesh = FeMesh::default();
        mesh.set_node(1, [0.0; 3]);
        mesh.set_node(2, [1.0, 0.0, 0.0]);
        mesh.add_element(Element {
            id: 1,
            type_name: "T3D2".into(),
            shape: ElementShape::Line2,
            nodes: vec![1, 2],
        })
        .unwrap();
        mesh.parts.push(Part {
            bodies: Vec::new(),
            name: "A".into(),
            elements: vec![1],
        });
        let mut fe = FeModel::default();
        let operation = |transform| crate::MeshOperation {
            name: "Transform-1".into(),
            parts: vec!["A".into()],
            kind: MeshOperationKind::Transform(transform),
        };
        let shift = operation(MeshTransform::Translate([0.5, 0.0, 0.0]));
        fe.set_mesh_operations(&mut mesh, vec![shift]);
        assert_eq!(mesh.node(2), Some([1.5, 0.0, 0.0]));
        let item = ItemRef::MeshOperation("Transform-1".into());
        assert_eq!(fe.exposable(&item), Field::VECTOR);
        let x = ValuePath::new(item.clone(), Field::X);
        assert_eq!(fe.value_at(&x), Some(0.5));
        assert_eq!(fe.value_kind(&x), Some(ValueKind::LENGTH));
        // The mesh moves by the new distance, not by the sum.
        assert!(fe.set_value_meshed(&mut mesh, &x, 2.0));
        assert_eq!(mesh.node(2), Some([3.0, 0.0, 0.0]));
        assert!(!fe.set_value_meshed(&mut mesh, &x, 2.0));

        // A scaling by zero would lose the mesh: it is not taken.
        let scale = operation(MeshTransform::Scale {
            center: [0.0; 3],
            factors: [2.0, 1.0, 1.0],
        });
        fe.set_mesh_operations(&mut mesh, vec![scale]);
        assert_eq!(mesh.node(2), Some([2.0, 0.0, 0.0]));
        let factor = ValuePath::new(item.clone(), Field::ScaleX);
        assert_eq!(fe.value_kind(&factor), Some(ValueKind::Factor));
        assert!(!fe.set_value_meshed(&mut mesh, &factor, 0.0));
        assert_eq!(fe.value_at(&factor), Some(2.0));
        assert!(fe.set_value_meshed(&mut mesh, &factor, 3.0));
        assert_eq!(mesh.node(2), Some([3.0, 0.0, 0.0]));

        let rotate = operation(MeshTransform::Rotate {
            point: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            angle: 0.0,
        });
        fe.set_mesh_operations(&mut mesh, vec![rotate]);
        let exposable = fe.exposable(&item);
        for field in [Field::Angle, Field::LocationX, Field::AxisZ] {
            assert!(exposable.contains(&field), "{field:?}");
        }
        assert!(!exposable.contains(&Field::X));
        let angle = ValuePath::new(item, Field::Angle);
        assert!(fe.set_value_meshed(&mut mesh, &angle, 90.0));
        let [x, y, _] = mesh.node(2).unwrap();
        assert!(x.abs() < 1e-12 && (y - 1.0).abs() < 1e-12);
    }

    /// The numbers of the mesh setup are those of the geometry: sizes in metres, options 1
    /// or 0. A size Gmsh cannot mesh with is not taken, and a changed number makes the parts
    /// it meshes out of date.
    #[test]
    fn mesh_setup_numbers_are_exposed_through_the_geometry() {
        use crate::{MeshSetupItem, MeshingParameters, ModelSpace};
        let mut geometry = Geometry {
            source: "plate.step".into(),
            brep: "shapes".into(),
            meshing: MeshingParameters::default(),
            mesh_items: vec![
                MeshSetupItem {
                    name: "Meshing_Parameters-1".into(),
                    kind: MeshSetupKind::MeshingParameters {
                        parts: vec!["PART_1".into()],
                        parameters: MeshingParameters {
                            max_size: 0.03,
                            ..MeshingParameters::default()
                        },
                    },
                },
                MeshSetupItem {
                    name: "Local_Mesh_Size-1".into(),
                    kind: MeshSetupKind::LocalMeshSize {
                        faces: vec![1],
                        edges: Vec::new(),
                        size: 0.002,
                        depth: 0.0,
                    },
                },
            ],
            part_names: Vec::new(),
            refinement: 1.0,
            signatures: Vec::new(),
            compounds: Vec::new(),
            suppressed: Vec::new(),
            own_meshes: Vec::new(),
            ids: Vec::new(),
            from_link: false,
        };
        let item = ItemRef::MeshSetup("Meshing_Parameters-1".into());
        let exposable = geometry.exposable(&item);
        assert_eq!(exposable.len(), 8);
        assert!(
            exposable.contains(&Field::MaxElementSize) && exposable.contains(&Field::Hexahedra)
        );
        let local = ItemRef::MeshSetup("Local_Mesh_Size-1".into());
        assert_eq!(
            geometry.exposable(&local),
            [Field::ElementSize, Field::Depth]
        );
        assert_eq!(geometry.exposable(&ItemRef::DefaultMeshing).len(), 8);

        let max = ValuePath::new(item.clone(), Field::MaxElementSize);
        let kind = ValueKind::Quantity(Quantity::Length);
        assert_eq!(geometry.value_at(&max), Some(0.03));
        assert_eq!(geometry.value_kind(&max), Some(kind));
        let fe = FeModel::default();
        assert_eq!(fe.value_in(Some(&geometry), &max), Some(0.03));
        assert_eq!(fe.value_kind_in(Some(&geometry), &max), Some(kind));
        assert_eq!(fe.value_at(&max), None);
        assert_eq!(fe.value_in(None, &max), None);

        let input =
            |g: &Geometry| crate::fingerprint::mesh_input(g, &[], ModelSpace::default(), "PART_1");
        let before = input(&geometry);
        assert!(geometry.set_value(&max, 0.01));
        assert!(!geometry.set_value(&max, 0.01));
        assert_ne!(input(&geometry), before);
        // Gmsh needs a size greater than zero.
        assert!(!geometry.set_value(&max, 0.0));
        assert!(!geometry.set_value(&max, f64::NAN));
        assert_eq!(geometry.value_at(&max), Some(0.01));

        let second = ValuePath::new(item, Field::SecondOrder);
        assert_eq!(geometry.value_at(&second), Some(1.0));
        assert!(geometry.set_value(&second, 0.0));
        assert_eq!(geometry.value_at(&second), Some(0.0));
        assert!(geometry.set_value(&second, 1.0));
        assert!(!geometry.set_value(&second, 0.8));

        let size = ValuePath::new(local, Field::ElementSize);
        assert!(geometry.set_value(&size, 0.001));
        assert_eq!(geometry.value_at(&size), Some(0.001));
        let default = ValuePath::new(ItemRef::DefaultMeshing, Field::ElementsPerCurvature);
        assert!(geometry.set_value(&default, 3.0));
        assert_eq!(geometry.meshing.elements_per_curvature, 3.0);
        // Gone with the item.
        let gone = ValuePath::new(ItemRef::MeshSetup("Other".into()), Field::MaxElementSize);
        assert_eq!(geometry.value_at(&gone), None);
        assert!(!geometry.set_value(&gone, 1.0));
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
