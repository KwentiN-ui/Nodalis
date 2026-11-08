//! Converting the values of a model into another unit system, so that the model stays the
//! same physically when its unit system changes.

use crate::units::{Quantity, UnitSystem};
use crate::{
    BoundaryKind, Constraint, FeModel, GapConductance, Geometry, InitialConditionKind,
    InteractionProperty, LoadKind, Material, MeshSetupKind, MeshingParameters, StaticStep,
    StepKind, SurfaceBehavior,
};

/// Converts values from one unit system into another.
#[derive(Clone, Copy, Debug)]
pub struct Conversion {
    pub from: UnitSystem,
    pub to: UnitSystem,
}

impl Conversion {
    pub fn new(from: UnitSystem, to: UnitSystem) -> Self {
        Self { from, to }
    }

    pub fn value(&self, value: &mut f64, quantity: Quantity) {
        *value = self.from.convert(*value, quantity, self.to);
    }

    pub fn option(&self, value: &mut Option<f64>, quantity: Quantity) {
        if let Some(value) = value {
            self.value(value, quantity);
        }
    }

    pub fn all(&self, values: &mut [f64], quantity: Quantity) {
        for value in values {
            self.value(value, quantity);
        }
    }

    /// Factor of lengths, by which the mesh and the geometry are scaled.
    pub fn length_factor(&self) -> f64 {
        self.from.factor_to(Quantity::Length, self.to)
    }
}

impl FeModel {
    /// Converts every value of the model into the unit system `to` and makes it the model's.
    /// The mesh and the geometry are not part of the FE model; they scale by
    /// [`Conversion::length_factor`]. Returns what could not be converted.
    pub fn convert_units(&mut self, to: UnitSystem) -> Vec<String> {
        let c = Conversion::new(self.properties.units, to);
        self.properties.units = to;
        let mut notes = Vec::new();
        if !c.from.has_units() || !c.to.has_units() {
            return notes;
        }
        let cylindrical: Vec<String> = (self.coordinate_systems.iter())
            .filter(|s| s.kind == crate::CoordinateSystemKind::Cylindrical)
            .map(|s| s.name.clone())
            .collect();
        for material in &mut self.materials {
            material.convert_units(&c);
        }
        for section in &mut self.sections {
            c.value(&mut section.thickness, Quantity::Length);
            match &mut section.kind {
                crate::SectionKind::Shell { thickness, .. }
                | crate::SectionKind::Membrane { thickness, .. } => {
                    c.value(thickness, Quantity::Length)
                }
                crate::SectionKind::Truss { area } => c.value(area, Quantity::Area),
                crate::SectionKind::Beam(_) => {}
                crate::SectionKind::Solid => {}
            }
        }
        for cross_section in &mut self.cross_sections {
            cross_section.profile.convert_units(&c);
        }
        self.convert_features(&c);
        for constraint in &mut self.constraints {
            constraint.convert_units(&c);
            if let Constraint::Joint(joint) = constraint
                && let Some(location) = &mut joint.location
            {
                // The angle of cylindrical coordinates stays.
                if cylindrical.contains(&joint.system) {
                    c.value(&mut location[0], Quantity::Length);
                    c.value(&mut location[2], Quantity::Length);
                } else {
                    c.all(location, Quantity::Length);
                }
            }
        }
        for interaction in &mut self.surface_interactions {
            for property in &mut interaction.properties {
                property.convert_units(&c);
            }
        }
        for pair in &mut self.contact_pairs {
            c.option(&mut pair.adjustment_size, Quantity::Length);
        }
        for tie in &mut self.ties {
            c.option(&mut tie.position_tolerance, Quantity::Length);
        }
        for region in &mut self.contact_regions {
            let settings = &mut region.settings;
            c.option(&mut settings.tolerance, Quantity::Length);
            for stiffness in [
                &mut settings.normal_stiffness,
                &mut settings.tangential_stiffness,
            ] {
                if let Some(crate::Stiffness::Value(k)) = stiffness {
                    c.value(k, Quantity::ForcePerVolume);
                }
            }
        }
        c.option(&mut self.properties.absolute_zero, Quantity::Temperature);
        c.option(
            &mut self.properties.stefan_boltzmann,
            Quantity::StefanBoltzmann,
        );
        for condition in &mut self.initial_conditions {
            match &mut condition.kind {
                InitialConditionKind::Temperature(t) => c.value(t, Quantity::Temperature),
                InitialConditionKind::Velocity(v) => {
                    v.iter_mut().for_each(|v| c.value(v, Quantity::Velocity));
                }
                InitialConditionKind::AngularVelocity { point, speed, .. } => {
                    point.iter_mut().for_each(|p| c.value(p, Quantity::Length));
                    c.value(speed, Quantity::RotationalSpeed);
                }
            }
        }
        for step in &mut self.steps {
            match &mut step.kind {
                StepKind::Static(s) => s.convert_units(&c),
                StepKind::HeatTransfer(h)
                | StepKind::CoupledTempDisp(h)
                | StepKind::UncoupledTempDisp(h) => {
                    h.increments.convert_units(&c);
                    c.option(&mut h.deltmx, Quantity::TemperatureDifference);
                }
                StepKind::Frequency(f) => {
                    c.option(&mut f.lower_frequency, Quantity::Frequency);
                    c.option(&mut f.upper_frequency, Quantity::Frequency);
                }
                StepKind::Visco(v) => v.increments.convert_units(&c),
                StepKind::Dynamic(d) => {
                    c.value(&mut d.duration, Quantity::Time);
                    if let crate::TimeStep::Fixed(dt) = &mut d.time_step {
                        c.value(dt, Quantity::Time);
                    }
                    c.option(&mut d.frequency_limit, Quantity::Frequency);
                    match &mut d.damping {
                        crate::Damping::None => {}
                        crate::Damping::Ratio { range, .. } => {
                            if let crate::DampingRange::Between(f1, f2) = range {
                                c.value(f1, Quantity::Frequency);
                                c.value(f2, Quantity::Frequency);
                            }
                        }
                        crate::Damping::Rayleigh(damping) => {
                            c.value(&mut damping.alpha, Quantity::Frequency);
                            c.value(&mut damping.beta, Quantity::Time);
                        }
                    }
                }
                StepKind::ModalDynamics(m) => {
                    c.value(&mut m.increment, Quantity::Time);
                    c.value(&mut m.time_period, Quantity::Time);
                    convert_modal_damping(&mut m.damping, &c);
                }
                StepKind::SteadyStateDynamics(s) => {
                    c.value(&mut s.lower_frequency, Quantity::Frequency);
                    c.value(&mut s.upper_frequency, Quantity::Frequency);
                    c.value(&mut s.time_lower, Quantity::Time);
                    c.value(&mut s.time_upper, Quantity::Time);
                    convert_modal_damping(&mut s.damping, &c);
                }
                StepKind::ComplexFrequency(complex) => {
                    c.all(&mut complex.point, Quantity::Length);
                    c.value(&mut complex.speed, Quantity::RotationalSpeed);
                }
                // Buckling factors and the accuracy have no unit.
                StepKind::Buckle(_) => {}
                StepKind::Sensitivity(s) => c.option(&mut s.filter_radius, Quantity::Length),
            }
            for field in &mut step.defined_fields {
                if let crate::DefinedFieldKind::Temperature(t) = &mut field.kind {
                    c.value(t, Quantity::Temperature);
                }
            }
            for bc in &mut step.boundary_conditions {
                match &mut bc.kind {
                    BoundaryKind::Submodel { .. } | BoundaryKind::Symmetry { .. } => {}
                    BoundaryKind::Displacement(remote) => {
                        convert_displacements(&mut remote.values, &c);
                        for (i, table) in remote.tables.iter_mut().enumerate() {
                            let quantity = if i < 3 {
                                Quantity::Length
                            } else {
                                Quantity::Angle
                            };
                            convert_table(table, quantity, &c);
                        }
                        c.option(&mut remote.pinball, Quantity::Length);
                        if let Some(location) = &mut remote.location {
                            // The angle of cylindrical coordinates stays.
                            if cylindrical.contains(&remote.system) {
                                c.value(&mut location[0], Quantity::Length);
                                c.value(&mut location[2], Quantity::Length);
                            } else {
                                c.all(location, Quantity::Length);
                            }
                        }
                    }
                    BoundaryKind::Temperature(t) => c.value(t, Quantity::Temperature),
                }
            }
            for load in &mut step.loads {
                if let Some(quantity) = load.kind.table_quantity() {
                    for table in &mut load.tables {
                        convert_table(table, quantity, &c);
                    }
                }
                match &mut load.kind {
                    LoadKind::Moment(moment) => c.all(moment, Quantity::Moment),
                    LoadKind::PreTension {
                        value,
                        by_displacement,
                        ..
                    } => {
                        let quantity = if *by_displacement {
                            Quantity::Length
                        } else {
                            Quantity::Force
                        };
                        c.value(value, quantity);
                    }
                    LoadKind::Force { value: force, .. } => c.all(force, Quantity::Force),
                    LoadKind::RemoteForce(remote) => {
                        c.all(&mut remote.force, Quantity::Force);
                        c.all(&mut remote.moment, Quantity::Moment);
                        c.option(&mut remote.pinball, Quantity::Length);
                        if let Some(location) = &mut remote.location {
                            if cylindrical.contains(&remote.system) {
                                c.value(&mut location[0], Quantity::Length);
                                c.value(&mut location[2], Quantity::Length);
                            } else {
                                c.all(location, Quantity::Length);
                            }
                        }
                    }
                    LoadKind::Pressure(pressure) => c.value(pressure, Quantity::Pressure),
                    LoadKind::ConcentratedFlux(flux) => c.value(flux, Quantity::Power),
                    LoadKind::SurfaceFlux(flux) => c.value(flux, Quantity::HeatFlux),
                    LoadKind::BodyFlux(flux) => c.value(flux, Quantity::PowerPerVolume),
                    LoadKind::Film { sink, coefficient } => {
                        c.value(sink, Quantity::Temperature);
                        c.value(coefficient, Quantity::HeatTransferCoefficient);
                    }
                    LoadKind::Radiation { sink, .. } => c.value(sink, Quantity::Temperature),
                    LoadKind::Gravity(acceleration) => c.all(acceleration, Quantity::Acceleration),
                    // Standard gravity has no unit of its own.
                    LoadKind::StandardGravity(_) => {}
                    LoadKind::Centrifugal { point, speed, .. } => {
                        c.all(point, Quantity::Length);
                        c.value(speed, Quantity::RotationalSpeed);
                    }
                    LoadKind::Moving(moving) => {
                        for length in [
                            &mut moving.start,
                            &mut moving.end,
                            &mut moving.patch_length,
                            &mut moving.patch_width,
                        ] {
                            c.value(length, Quantity::Length);
                        }
                        for wheel in &mut moving.wheels {
                            c.value(&mut wheel.along, Quantity::Length);
                            c.value(&mut wheel.across, Quantity::Length);
                            c.all(&mut wheel.force, Quantity::Force);
                        }
                    }
                }
            }
        }
        for amplitude in &mut self.amplitudes {
            c.value(&mut amplitude.shift_time, Quantity::Time);
            for point in &mut amplitude.points {
                c.value(&mut point[0], Quantity::Time);
            }
        }
        // An amplitude scales a temperature from the zero of its unit, which moves with
        // another temperature unit such as Kelvin instead of degrees Celsius.
        let zero_moves = c.from.convert(0.0, Quantity::Temperature, c.to) != 0.0;
        let scaled_temperature = self.steps.iter().any(|step| {
            (step.boundary_conditions.iter()).any(|b| b.amplitude.is_some() && b.kind.is_thermal())
                || (step.loads.iter()).any(|l| {
                    l.amplitude.is_some()
                        && matches!(l.kind, LoadKind::Film { .. } | LoadKind::Radiation { .. })
                })
        });
        if zero_moves && scaled_temperature {
            notes.push(
                "Temperatures with an amplitude now refer to a different zero point; \
                 please check the amplitudes."
                    .into(),
            );
        }
        notes
    }

    /// Converts the positions of the features: reference points, coordinate systems, planes
    /// and result paths, and the axis of cyclic symmetry.
    pub fn convert_features(&mut self, c: &Conversion) {
        let length = |p: &mut [f64; 3]| c.all(p, Quantity::Length);
        let cylindrical: Vec<String> = (self.coordinate_systems.iter())
            .filter(|s| s.kind == crate::CoordinateSystemKind::Cylindrical)
            .map(|s| s.name.clone())
            .collect();
        for point in &mut self.reference_points {
            length(&mut point.position);
            if let crate::PointDefinition::Local {
                system,
                coordinates,
            } = &mut point.definition
            {
                // The angle of cylindrical coordinates stays.
                if cylindrical.contains(system) {
                    c.value(&mut coordinates[0], Quantity::Length);
                    c.value(&mut coordinates[2], Quantity::Length);
                } else {
                    length(coordinates);
                }
            }
        }
        for system in &mut self.coordinate_systems {
            length(&mut system.origin);
            length(&mut system.point_x);
            length(&mut system.point_xy);
        }
        let point = |p: &mut crate::PointRef| {
            if let crate::PointRef::Coordinates(p) = p {
                c.all(p, Quantity::Length);
            }
        };
        for plane in &mut self.planes {
            match &mut plane.source {
                crate::PlaneSource::PointNormal { point: p, .. } => point(p),
                crate::PlaneSource::ThreePoints { points } => points.iter_mut().for_each(point),
                crate::PlaneSource::CoordinateSystem { offset, .. } => {
                    c.value(offset, Quantity::Length)
                }
            }
        }
        for path in &mut self.result_paths {
            point(&mut path.start);
            point(&mut path.end);
        }
        for tie in &mut self.ties {
            if let Some(cyclic) = &mut tie.cyclic {
                length(&mut cyclic.axis_point);
            }
        }
    }
}

impl StaticStep {
    fn convert_units(&mut self, c: &Conversion) {
        c.value(&mut self.duration, Quantity::Time);
        let control = &mut self.control;
        if let crate::IncrementSize::Fixed(dt) = &mut control.size {
            c.value(dt, Quantity::Time);
        }
        for time in [
            &mut control.initial,
            &mut control.minimum,
            &mut control.maximum,
        ] {
            c.option(time, Quantity::Time);
        }
    }
}

impl Material {
    pub fn convert_units(&mut self, c: &Conversion) {
        c.option(&mut self.density, Quantity::Density);
        if let Some(elastic) = &mut self.elastic {
            c.value(&mut elastic.young, Quantity::Pressure);
        }
        if let Some(hyper) = &mut self.hyperelastic {
            let kinds = hyper.model.parameters();
            for (value, parameter) in hyper.constants.iter_mut().zip(kinds) {
                if parameter.kind == crate::hyperelastic::ParameterKind::Stress {
                    c.value(value, Quantity::Pressure);
                }
            }
            for point in hyper.test_data.iter_mut() {
                c.value(&mut point.stress, Quantity::Pressure);
            }
        }
        c.option(&mut self.conductivity, Quantity::ThermalConductivity);
        c.option(&mut self.specific_heat, Quantity::SpecificHeat);
        if let Some(expansion) = &mut self.expansion {
            c.value(&mut expansion.coefficient, Quantity::ThermalExpansion);
            c.value(&mut expansion.zero_temperature, Quantity::Temperature);
        }
        if let Some(plastic) = &mut self.plastic {
            for point in &mut plastic.points {
                c.value(&mut point.stress, Quantity::Pressure);
                c.value(&mut point.temperature, Quantity::Temperature);
            }
        }
        // The strain rate a * stress^n * time^m is per time: a scales with the stress to
        // the power -n and the time to the power -(m + 1).
        if let Some(creep) = &mut self.creep {
            let stress = c.from.factor_to(Quantity::Pressure, c.to);
            let time = c.from.factor_to(Quantity::Time, c.to);
            creep.a *= stress.powf(-creep.n) * time.powf(-(creep.m + 1.0));
        }
    }
}

impl Constraint {
    fn convert_units(&mut self, c: &Conversion) {
        let stiffness = |per_area: bool| {
            if per_area {
                Quantity::ForcePerVolume
            } else {
                Quantity::ForcePerLength
            }
        };
        match self {
            Constraint::Spring(s) => {
                let quantity = stiffness(s.is_per_area());
                c.all(&mut s.stiffness, quantity);
                // Moment per radian.
                c.all(&mut s.rotational, Quantity::Moment)
            }
            Constraint::CompressionOnly(s) => {
                c.value(&mut s.clearance, Quantity::Length);
                c.value(&mut s.offset, Quantity::Length);
                c.option(&mut s.spring_stiffness, Quantity::ForcePerLength);
                c.option(&mut s.tensile_force, Quantity::Force);
            }
            Constraint::Tie(t) => c.option(&mut t.position_tolerance, Quantity::Length),
            Constraint::NodeTie(_) | Constraint::RigidBody(_) | Constraint::Joint(_) => {}
        }
    }
}

impl InteractionProperty {
    pub fn convert_units(&mut self, c: &Conversion) {
        match self {
            InteractionProperty::SurfaceBehavior(behavior) => match behavior {
                SurfaceBehavior::Hard => {}
                SurfaceBehavior::Linear { k, sigma_inf, c0 } => {
                    c.value(k, Quantity::ForcePerVolume);
                    c.value(sigma_inf, Quantity::Pressure);
                    c.option(c0, Quantity::Length);
                }
                SurfaceBehavior::Exponential { c0, p0 } => {
                    c.value(c0, Quantity::Length);
                    c.value(p0, Quantity::Pressure);
                }
                SurfaceBehavior::Tabular(rows) => {
                    for [pressure, overclosure] in rows {
                        c.value(pressure, Quantity::Pressure);
                        c.value(overclosure, Quantity::Length);
                    }
                }
                SurfaceBehavior::Tied { k } => c.value(k, Quantity::ForcePerVolume),
            },
            InteractionProperty::Friction(friction) => {
                c.option(&mut friction.stick_slope, Quantity::ForcePerVolume)
            }
            InteractionProperty::GapConductance(conductance) => match conductance {
                GapConductance::Constant(value) => {
                    c.value(value, Quantity::HeatTransferCoefficient)
                }
                GapConductance::Tabular(rows) => {
                    for [value, pressure, temperature] in rows {
                        c.value(value, Quantity::HeatTransferCoefficient);
                        c.value(pressure, Quantity::Pressure);
                        c.value(temperature, Quantity::Temperature);
                    }
                }
            },
        }
    }
}

impl Geometry {
    /// Converts the mesh sizes; the shapes themselves are scaled by the mesher, which can
    /// read them.
    pub fn convert_sizes(&mut self, c: &Conversion) {
        self.meshing.convert_units(c);
        for item in &mut self.mesh_items {
            match &mut item.kind {
                MeshSetupKind::MeshingParameters { parameters, .. } => parameters.convert_units(c),
                MeshSetupKind::LocalMeshSize { size, .. } => c.value(size, Quantity::Length),
                MeshSetupKind::TetrahedralGmsh { .. } | MeshSetupKind::ExtrudeMesh(_) => {}
            }
        }
    }
}

impl MeshingParameters {
    fn convert_units(&mut self, c: &Conversion) {
        c.value(&mut self.max_size, Quantity::Length);
        c.value(&mut self.min_size, Quantity::Length);
    }
}

/// Only Rayleigh damping has units: its mass coefficient is a frequency, its stiffness
/// coefficient a time.
fn convert_modal_damping(damping: &mut Option<crate::ModalDamping>, c: &Conversion) {
    if let Some(crate::ModalDamping::Rayleigh(r)) = damping {
        c.value(&mut r.alpha, Quantity::Frequency);
        c.value(&mut r.beta, Quantity::Time);
    }
}

/// The times and values of a time table.
fn convert_table(table: &mut Option<crate::ValueTable>, quantity: Quantity, c: &Conversion) {
    for point in table.iter_mut().flat_map(|t| t.points.iter_mut()) {
        c.value(&mut point[0], Quantity::Time);
        c.value(&mut point[1], quantity);
    }
}

/// Displacements (lengths) and rotations (angles) of a boundary condition.
fn convert_displacements(values: &mut [Option<f64>; 6], c: &Conversion) {
    for (i, value) in values.iter_mut().enumerate() {
        let quantity = if i < 3 {
            Quantity::Length
        } else {
            Quantity::Angle
        };
        c.option(value, quantity);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Elastic, Load, Region, Section, SectionKind, Step};

    #[test]
    fn a_converted_model_stays_the_same_physically() {
        let mut model = FeModel::default();
        model.materials.push(Material {
            name: "Steel".into(),
            density: Some(7.85e-9),
            elastic: Some(Elastic {
                young: 210_000.0,
                poisson: 0.3,
            }),
            conductivity: Some(50.0),
            creep: Some(crate::Creep {
                a: 1e-15,
                n: 5.0,
                m: 0.0,
            }),
            ..Material::default()
        });
        model.sections.push(Section {
            name: "Section-1".into(),
            material: "Steel".into(),
            region: Region::Parts(Vec::new()),
            thickness: 5.0,
            kind: SectionKind::Solid,
        });
        let mut step = Step::new_static("Step-1");
        step.loads.push(Load {
            distribution: None,
            name: "Pressure-1".into(),
            active: true,
            visible: true,
            region: Region::Surface("A".into()),
            kind: LoadKind::Pressure(2.0),
            amplitude: None,
            factor_amplitude: None,
            tables: Default::default(),
        });
        step.loads.push(Load {
            distribution: None,
            name: "Force-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(Vec::new()),
            kind: LoadKind::force([1000.0, 0.0, 0.0]),
            amplitude: None,
            factor_amplitude: None,
            tables: Default::default(),
        });
        model.steps.push(step);
        assert!(model.convert_units(UnitSystem::MTonSC).is_empty());
        assert_eq!(model.properties.units, UnitSystem::MTonSC);
        let close = |a: f64, b: f64| (a - b).abs() <= 1e-12 * b.abs();
        let material = &model.materials[0];
        assert!(close(material.density.unwrap(), 7.85));
        assert!(close(material.elastic.unwrap().young, 2.1e8));
        // 50 W/(m·K) is 50 mW/(mm·K) and 0.05 kW/(m·K).
        assert!(close(material.conductivity.unwrap(), 0.05));
        // The same creep rate at 100 MPa = 1e5 kPa: 1e-15 * 100^5 = a * (1e5)^5.
        assert!(close(material.creep.unwrap().a, 1e-30));
        assert!(close(model.sections[0].thickness, 0.005));
        let loads = &model.steps[0].loads;
        assert!(matches!(loads[0].kind, LoadKind::Pressure(p) if close(p, 2000.0)));
        assert!(matches!(loads[1].kind, LoadKind::Force { value: [f, _, _], .. } if close(f, 1.0)));
        // Without units, the numbers are only reinterpreted.
        model.convert_units(UnitSystem::Unitless);
        assert!(close(model.sections[0].thickness, 0.005));
    }
}
