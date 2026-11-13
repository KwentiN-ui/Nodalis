//! Items whose references are gone: a section whose material
//! was deleted, or a load on a node set the mesh no longer has. Validity is derived from the
//! model each time, so an item becomes valid again as soon as its reference is back.

use plx_mesh::FeMesh;

use crate::{Constraint, FeModel, Id, LoadKind, Region, Section, SectionKind, line_tangent};

/// An item of the model that can refer to something else.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ModelItem {
    Section(usize),
    CrossSection(usize),
    Constraint(usize),
    ContactPair(usize),
    Tie(usize),
    NodeTie(usize),
    ContactRegion(usize),
    Bolt(usize),
    /// Boundary condition by step and index.
    BoundaryCondition(usize, usize),
    /// Load by step and index.
    Load(usize, usize),
    /// History output by step and index.
    HistoryOutput(usize, usize),
    /// Defined field by step and index.
    DefinedField(usize, usize),
    Material(usize),
    /// Part of the mesh by index.
    Part(usize),
    Step(usize),
    /// The boundary conditions of a step as a whole.
    BoundaryConditions(usize),
    /// The analysis, for problems the solver reported.
    Analysis,
    InitialCondition(usize),
    Amplitude(usize),
    Distribution(usize),
}

/// An item with a missing reference and why it is invalid.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Invalid {
    pub item: ModelItem,
    pub reason: String,
}

impl FeModel {
    /// The item named `name`, for an error of the lowering that names the item it was
    /// lowering: the items of `step` first (boundary conditions and loads of different steps
    /// may share names), then those of the model, then those of any step.
    pub fn item_named(&self, name: &str, step: Option<usize>) -> Option<ModelItem> {
        let in_step = |s: usize| -> Option<ModelItem> {
            let step = self.steps.get(s)?;
            let position = |names: &mut dyn Iterator<Item = &String>| {
                Iterator::position(&mut &mut *names, |n| n == name)
            };
            if let Some(i) = position(&mut step.boundary_conditions.iter().map(|b| &b.name)) {
                return Some(ModelItem::BoundaryCondition(s, i));
            }
            if let Some(i) = position(&mut step.loads.iter().map(|l| &l.name)) {
                return Some(ModelItem::Load(s, i));
            }
            let history = self.step_history_outputs(s);
            if let Some(i) = position(&mut history.iter().map(|o| &o.name)) {
                return Some(ModelItem::HistoryOutput(s, i));
            }
            if let Some(i) = position(&mut step.defined_fields.iter().map(|f| &f.name)) {
                return Some(ModelItem::DefinedField(s, i));
            }
            None
        };
        if let Some(item) = step.and_then(in_step) {
            return Some(item);
        }
        let find = |names: &mut dyn Iterator<Item = &str>, item: fn(usize) -> ModelItem| {
            Iterator::position(&mut &mut *names, |n| n == name).map(item)
        };
        find(
            &mut self.sections.iter().map(|x| x.name.as_str()),
            ModelItem::Section,
        )
        .or_else(|| {
            find(
                &mut self.materials.iter().map(|x| x.name.as_str()),
                ModelItem::Material,
            )
        })
        .or_else(|| {
            find(
                &mut self.constraints.iter().map(|x| x.name()),
                ModelItem::Constraint,
            )
        })
        .or_else(|| {
            find(
                &mut self.ties.iter().map(|x| x.name.as_str()),
                ModelItem::Tie,
            )
        })
        .or_else(|| {
            find(
                &mut self.node_ties.iter().map(|x| x.name.as_str()),
                ModelItem::NodeTie,
            )
        })
        .or_else(|| {
            find(
                &mut self.contact_pairs.iter().map(|x| x.name.as_str()),
                ModelItem::ContactPair,
            )
        })
        .or_else(|| {
            find(
                &mut self.contact_regions.iter().map(|x| x.name.as_str()),
                ModelItem::ContactRegion,
            )
        })
        .or_else(|| {
            find(
                &mut self.bolts.iter().map(|x| x.name.as_str()),
                ModelItem::Bolt,
            )
        })
        .or_else(|| {
            find(
                &mut self.amplitudes.iter().map(|x| x.name.as_str()),
                ModelItem::Amplitude,
            )
        })
        .or_else(|| {
            find(
                &mut self.distributions.iter().map(|x| x.name.as_str()),
                ModelItem::Distribution,
            )
        })
        .or_else(|| {
            find(
                &mut self.initial_conditions.iter().map(|x| x.name.as_str()),
                ModelItem::InitialCondition,
            )
        })
        .or_else(|| {
            find(
                &mut self.cross_sections.iter().map(|x| x.name.as_str()),
                ModelItem::CrossSection,
            )
        })
        .or_else(|| (0..self.steps.len()).find_map(in_step))
        .or_else(|| {
            find(
                &mut self.steps.iter().map(|x| x.name.as_str()),
                ModelItem::Step,
            )
        })
    }

    /// Items that refer to materials, parts, sets or mesh entities that do not exist.
    pub fn invalid_items(&self, mesh: &FeMesh) -> Vec<Invalid> {
        let mut invalid = Vec::new();
        for (i, section) in self.sections.iter().enumerate() {
            let reason = if section.material.and_then(|m| self.material(m)).is_none() {
                Some(match section.material {
                    None => "The section has no material".into(),
                    Some(_) => "The material of the section does not exist".into(),
                })
            } else if let SectionKind::Beam(beam) = &section.kind
                && beam
                    .cross_section
                    .and_then(|c| self.cross_section(c))
                    .is_none()
            {
                Some(match beam.cross_section {
                    None => "The beam section has no cross-section".into(),
                    Some(_) => "The cross-section of the beam section does not exist".into(),
                })
            } else {
                section.region.missing_reference(mesh)
            };
            if let Some(reason) = reason {
                invalid.push(Invalid {
                    item: ModelItem::Section(i),
                    reason,
                });
            }
        }
        for (i, cross_section) in self.cross_sections.iter().enumerate() {
            if !cross_section.profile.is_valid() {
                invalid.push(Invalid {
                    item: ModelItem::CrossSection(i),
                    reason: "The profile dimensions are invalid".into(),
                });
            }
        }
        for (i, constraint) in self.constraints.iter().enumerate() {
            let reason = match constraint.master_slave() {
                Some([master, slave]) => master_slave_reference(master, slave, mesh),
                None => (constraint.regions().into_iter())
                    .find_map(|region| region.missing_reference(mesh))
                    .or_else(|| match constraint {
                        Constraint::RigidBody(body) => {
                            self.missing_reference_point(body.reference_point)
                        }
                        Constraint::Spring(spring) => (spring.ends.iter())
                            .filter_map(crate::SpringEnd::point)
                            .find_map(|point| {
                                (self.missing_region_point(point))
                                    .or_else(|| self.missing_remote_point(mesh, point))
                            })
                            .or_else(|| {
                                (spring.layout().is_none())
                                    .then(|| "Both ends are World".to_string())
                            }),
                        Constraint::Joint(joint) => (joint.ends.iter())
                            .filter_map(crate::SpringEnd::point)
                            .find_map(|point| {
                                (self.missing_region_point(point))
                                    .or_else(|| self.missing_remote_point(mesh, point))
                            })
                            .or_else(|| {
                                (joint.locating_end().is_none())
                                    .then(|| "Both ends are World".to_string())
                            })
                            .or_else(|| {
                                (self.coordinate_system_or_global(joint.system).is_none())
                                    .then(|| "The coordinate system does not exist".to_string())
                            })
                            .or_else(|| {
                                // The axis from faces that have none (an origin that cannot
                                // be found is the selection's problem).
                                (self.joint_origin(mesh, joint).is_ok())
                                    .then(|| self.joint_frame(mesh, joint).err())
                                    .flatten()
                            }),
                        Constraint::PointMass(mass) => self.point_mass_problem(mass),
                        _ => None,
                    }),
            };
            if let Some(reason) = reason {
                invalid.push(Invalid {
                    item: ModelItem::Constraint(i),
                    reason,
                });
            }
        }
        for (i, pair) in self.contact_pairs.iter().enumerate() {
            let reason = if (pair.interaction)
                .and_then(|i| self.surface_interaction(i))
                .is_none()
            {
                Some("The surface interaction of the contact pair does not exist".into())
            } else {
                master_slave_reference(&pair.master, &pair.slave, mesh)
            };
            if let Some(reason) = reason {
                invalid.push(Invalid {
                    item: ModelItem::ContactPair(i),
                    reason,
                });
            }
        }
        for (i, tie) in self.ties.iter().enumerate() {
            if let Some(reason) = master_slave_reference(&tie.master, &tie.slave, mesh) {
                invalid.push(Invalid {
                    item: ModelItem::Tie(i),
                    reason,
                });
            }
        }
        for (i, tie) in self.node_ties.iter().enumerate() {
            if let Some(reason) = tie.region.missing_reference(mesh) {
                invalid.push(Invalid {
                    item: ModelItem::NodeTie(i),
                    reason,
                });
            }
        }
        for (i, region) in self.contact_regions.iter().enumerate() {
            let reason = (region.side_a.missing_reference(mesh))
                .map(|r| format!("Side A: {r}"))
                .or_else(|| {
                    (region.side_b.missing_reference(mesh)).map(|r| format!("Side B: {r}"))
                });
            if let Some(reason) = reason {
                invalid.push(Invalid {
                    item: ModelItem::ContactRegion(i),
                    reason,
                });
            }
        }
        for (i, bolt) in self.bolts.iter().enumerate() {
            let reason = (bolt.surface.missing_reference(mesh))
                .map(|r| format!("Bolt surface: {r}"))
                .or_else(|| (bolt.thread.missing_reference(mesh)).map(|r| format!("Thread: {r}")));
            if let Some(reason) = reason {
                invalid.push(Invalid {
                    item: ModelItem::Bolt(i),
                    reason,
                });
            }
        }
        for (s, step) in self.steps.iter().enumerate() {
            for (i, bc) in step.boundary_conditions.iter().enumerate() {
                let reason = (bc.region.missing_reference(mesh))
                    .or_else(|| self.missing_region_point(&bc.region))
                    .or_else(|| self.missing_amplitude([&bc.amplitude]))
                    .or_else(|| self.missing_distribution(&bc.distribution))
                    .or_else(|| match &bc.kind {
                        crate::BoundaryKind::Displacement(displacement) => {
                            table_problem((0..6).map(|d| displacement.table(d)))
                        }
                        _ => None,
                    })
                    .or_else(|| match &bc.kind {
                        // An unknown coordinate system, or no node in the pinball region.
                        crate::BoundaryKind::Displacement(displacement) => {
                            match self.formulation(mesh, s, i) {
                                Some(crate::Formulation::Point(_)) => (displacement.resolve(
                                    self,
                                    mesh,
                                    &bc.region,
                                    &Default::default(),
                                ))
                                .err(),
                                _ => (self.coordinate_system_or_global(displacement.system))
                                    .is_none()
                                    .then(|| "The coordinate system does not exist".into()),
                            }
                        }
                        _ => None,
                    });
                if let Some(reason) = reason {
                    invalid.push(Invalid {
                        item: ModelItem::BoundaryCondition(s, i),
                        reason,
                    });
                }
            }
            for (i, load) in step.loads.iter().enumerate() {
                let radiation = matches!(load.kind, LoadKind::Radiation { .. });
                let constants = self.properties.absolute_zero.is_some()
                    && self.properties.stefan_boltzmann.is_some();
                let reason = if radiation && !constants {
                    Some(
                        "Radiation needs the absolute zero and the \
                         Stefan-Boltzmann constant (model properties)"
                            .into(),
                    )
                } else if let LoadKind::Moving(moving) = &load.kind
                    && let Some(problem) = moving.problem(self, self.properties.space.is_2d())
                {
                    Some(problem)
                } else {
                    (load.region.missing_reference(mesh))
                        .or_else(|| self.missing_region_point(&load.region))
                        .or_else(|| {
                            self.missing_amplitude([&load.amplitude, &load.factor_amplitude])
                        })
                        .or_else(|| self.missing_distribution(&load.distribution))
                        .or_else(|| table_problem((0..3).map(|c| load.table(c))))
                        .or_else(|| match (&load.kind, &load.region) {
                            // A body load on a reference point pulls the point masses there.
                            (
                                LoadKind::Gravity(_)
                                | LoadKind::StandardGravity(_)
                                | LoadKind::Centrifugal { .. },
                                Region::ReferencePoint(_),
                            ) => (self.point_masses_pulled(&load.region, mesh).is_empty())
                                .then(|| "No active point mass sits on the reference point".into()),
                            _ => None,
                        })
                        .or_else(|| match &load.kind {
                            // An unknown coordinate system, or no node in the pinball region.
                            LoadKind::RemoteForce(remote)
                                if load.region.reference_point().is_none() =>
                            {
                                (remote.point())
                                    .resolve(self, mesh, &load.region, &Default::default())
                                    .and_then(|point| remote.global(self, point.position))
                                    .err()
                            }
                            _ => None,
                        })
                        .or_else(|| load_system_problem(self, load))
                };
                if let Some(reason) = reason {
                    invalid.push(Invalid {
                        item: ModelItem::Load(s, i),
                        reason,
                    });
                }
            }
        }
        // Each output once, in the first step that prints it.
        for output in &self.history_outputs {
            let place = (self.steps.iter().enumerate()).find_map(|(s, step)| {
                let i = step.history_outputs.iter().position(|&o| o == output.id)?;
                Some((s, i))
            });
            if let Some((s, i)) = place {
                let reason = match &output.kind {
                    crate::HistoryKind::Contact { pair } => (pair)
                        .and_then(|p| self.contact_name(p))
                        .is_none()
                        .then(|| "The contact pair does not exist".into()),
                    kind => kind.region().and_then(|r| {
                        r.missing_reference(mesh)
                            .or_else(|| self.missing_region_point(r))
                            .or_else(|| self.missing_remote_point(mesh, r))
                    }),
                };
                if let Some(reason) = reason {
                    invalid.push(Invalid {
                        item: ModelItem::HistoryOutput(s, i),
                        reason,
                    });
                }
            }
        }
        for (s, step) in self.steps.iter().enumerate() {
            for (i, field) in step.defined_fields.iter().enumerate() {
                let reason = match &field.kind {
                    crate::DefinedFieldKind::Temperature(_) => (field.region)
                        .missing_reference(mesh)
                        .or_else(|| self.missing_amplitude([&field.amplitude]))
                        .or_else(|| self.missing_distribution(&field.distribution)),
                    crate::DefinedFieldKind::TemperatureFromFile { file, .. } => (!file.is_file())
                        .then(|| format!("Result file {} does not exist", file.display())),
                };
                if let Some(reason) = reason {
                    invalid.push(Invalid {
                        item: ModelItem::DefinedField(s, i),
                        reason,
                    });
                }
            }
        }
        for (i, condition) in self.initial_conditions.iter().enumerate() {
            let reason = (condition.region.missing_reference(mesh))
                .or_else(|| self.missing_distribution(&condition.distribution));
            if let Some(reason) = reason {
                invalid.push(Invalid {
                    item: ModelItem::InitialCondition(i),
                    reason,
                });
            }
        }
        for (i, amplitude) in self.amplitudes.iter().enumerate() {
            if let Some(reason) = amplitude.points_problem() {
                invalid.push(Invalid {
                    item: ModelItem::Amplitude(i),
                    reason,
                });
            }
        }
        for (i, distribution) in self.distributions.iter().enumerate() {
            if let Some(reason) = distribution.problem() {
                invalid.push(Invalid {
                    item: ModelItem::Distribution(i),
                    reason,
                });
            }
        }
        invalid
    }

    /// The reference point, if the model does not have it.
    fn missing_reference_point(&self, id: Id) -> Option<String> {
        (self.reference_point(id).is_none()).then(|| "The reference point does not exist".into())
    }

    /// The rigid body that couples a reference point, if one does.
    pub fn rigid_body_of(&self, point: Id) -> Option<&crate::RigidBody> {
        self.constraints.iter().find_map(|c| match c {
            Constraint::RigidBody(body) if body.active && body.reference_point == point => {
                Some(body)
            }
            _ => None,
        })
    }

    /// The elements that carry a point mass: those touching the nodes of its region, or of
    /// the rigid body on its point when that body carries it. A gravity or centrifugal load
    /// pulls the mass when its region holds one of them.
    pub fn point_mass_carriers(
        &self,
        mass: &crate::PointMass,
        mesh: &FeMesh,
    ) -> Vec<plx_mesh::ElementId> {
        let point = self.reference_point(mass.reference_point);
        let carrier = match point.and_then(|_| self.rigid_body_of(mass.reference_point)) {
            Some(body) => &body.region,
            None => &mass.region,
        };
        let carried: std::collections::BTreeSet<plx_mesh::NodeId> =
            carrier.nodes(mesh).into_iter().collect();
        (mesh.elements().iter())
            .filter(|e| e.nodes.iter().any(|n| carried.contains(n)))
            .map(|e| e.id)
            .collect()
    }

    /// The active point masses that a gravity or centrifugal load on `region` pulls: those
    /// carried by elements of the region, or on a reference point those on the point alone.
    pub fn point_masses_pulled(&self, region: &Region, mesh: &FeMesh) -> Vec<&crate::PointMass> {
        let masses = (self.constraints.iter()).filter_map(|c| match c {
            Constraint::PointMass(mass) if mass.active => Some(mass),
            _ => None,
        });
        if let Region::ReferencePoint(point) = region {
            return masses.filter(|m| m.reference_point == *point).collect();
        }
        let elements: std::collections::BTreeSet<plx_mesh::ElementId> =
            region.elements(mesh).into_iter().collect();
        masses
            .filter(|mass| {
                (self.point_mass_carriers(mass, mesh).iter()).any(|e| elements.contains(e))
            })
            .collect()
    }

    /// Why a point mass cannot be written, if it cannot.
    pub fn point_mass_problem(&self, mass: &crate::PointMass) -> Option<String> {
        if self.properties.space == crate::ModelSpace::Axisymmetric {
            return Some("Point masses are not available in axisymmetric models".into());
        }
        if let Some(problem) = self.missing_reference_point(mass.reference_point) {
            return Some(problem);
        }
        let body = self.rigid_body_of(mass.reference_point);
        match (mass.region.is_empty(), body) {
            (true, None) => {
                return Some(
                    "Select the region that carries the mass, or choose a reference point a \
                     rigid body couples"
                        .into(),
                );
            }
            (false, Some(body)) => {
                return Some(format!(
                    "The rigid body {} couples the reference point already; the mass needs \
                     no region of its own",
                    body.name
                ));
            }
            _ => {}
        }
        if !mass.mass.is_finite() || mass.mass < 0.0 {
            return Some("The mass is negative".into());
        }
        match &mass.inertia {
            None if mass.mass == 0.0 => Some("The mass is zero".into()),
            None => None,
            Some(inertia) => {
                if self.coordinate_system_or_global(inertia.system).is_none() {
                    return Some("The coordinate system does not exist".into());
                }
                if self.properties.space.is_2d() {
                    // About the axis normal to the plane only.
                    if inertia.tensor[2] < 0.0 || !inertia.tensor[2].is_finite() {
                        return Some("The rotary inertia is negative".into());
                    }
                } else if let Some(problem) = inertia.problem() {
                    return Some(problem);
                }
                let tensor = if self.properties.space.is_2d() {
                    &inertia.tensor[2..3]
                } else {
                    &inertia.tensor[..]
                };
                (mass.mass == 0.0 && tensor.iter().all(|v| *v == 0.0))
                    .then(|| "The mass and the rotary inertia are zero".into())
            }
        }
    }

    /// The reference point of a region on one, if the model does not have it.
    fn missing_region_point(&self, region: &Region) -> Option<String> {
        region
            .reference_point()
            .and_then(|id| self.missing_reference_point(id))
    }

    /// The displacement of a region on its point, if the model does not have it written
    /// with a point.
    fn missing_remote_point(&self, mesh: &FeMesh, region: &Region) -> Option<String> {
        let id = region.remote_point()?;
        (self.remote_point_of(mesh, id).is_none()).then(|| match self.remote_item_name(id) {
            Some(name) => format!("Displacement or Remote Force {name} has no point"),
            None => "The displacement or remote force does not exist".into(),
        })
    }

    /// The distribution reference, if the model does not have it.
    fn missing_distribution(&self, reference: &Option<Id>) -> Option<String> {
        (*reference)
            .filter(|&id| self.distribution(id).is_none())
            .map(|_| "The distribution does not exist".into())
    }

    /// The first of the amplitude references the model does not have.
    fn missing_amplitude<'a>(
        &self,
        references: impl IntoIterator<Item = &'a Option<Id>>,
    ) -> Option<String> {
        (references.into_iter().flatten())
            .find(|&&id| self.amplitude(id).is_none())
            .map(|_| "The amplitude does not exist".into())
    }
}

/// Why the solver would reject the first of the tables, if it would.
fn table_problem<'a>(
    tables: impl Iterator<Item = Option<&'a crate::ValueTable>>,
) -> Option<String> {
    tables.flatten().find_map(crate::ValueTable::points_problem)
}

impl Section {
    /// Why the section's kind does not fit its elements or its own values, if it does not:
    /// a beam section on solids, a solid section on lines, a pipe on 2-node lines, a normal
    /// parallel to a beam, or dimensions the solver would reject.
    pub fn kind_problem(&self, fe: &FeModel, mesh: &FeMesh) -> Option<String> {
        let profile = fe.section_profile(&self.kind);
        if let Err(reason) = fe.section_axes(self) {
            return Some(reason);
        }
        let material = self.material.and_then(|m| fe.material(m));
        match &self.kind {
            SectionKind::Solid => {}
            SectionKind::Infill(infill) => {
                if let Some(problem) = infill.problem() {
                    return Some(problem.into());
                }
                if let Some(material) = material
                    && (material.elastic.is_none()
                        || material.anisotropic.is_some()
                        || material.hyperelastic.is_some())
                {
                    return Some(format!(
                        "The printed material {} needs isotropic linear elasticity; the \
                         section makes it anisotropic itself",
                        material.name
                    ));
                }
                if infill.pattern == crate::InfillPattern::Custom {
                    match infill.core_material.and_then(|m| fe.material(m)) {
                        None => return Some("The custom infill needs a core material".into()),
                        Some(core) if !core.has_elasticity() || core.hyperelastic.is_some() => {
                            return Some(format!(
                                "The core material {} needs linear elasticity",
                                core.name
                            ));
                        }
                        Some(_) => {}
                    }
                }
            }
            SectionKind::Truss { area } => {
                if !(area.is_finite() && *area > 0.0) {
                    return Some("The cross-section area must be greater than 0".into());
                }
            }
            SectionKind::Beam(beam) => match profile {
                None => {
                    return Some("The cross-section does not exist".into());
                }
                Some(profile) if !profile.is_valid() => {
                    let name = fe.name_or_missing(beam.cross_section);
                    return Some(format!(
                        "The profile dimensions of Cross-Section {name} are invalid"
                    ));
                }
                // The axis of CalculiX's general beam runs through the centroid.
                Some(profile) if !profile.is_native() && beam.offset != [0.0; 2] => {
                    return Some(format!(
                        "{} profiles take no offset: the beam axis runs through the centroid",
                        profile.label()
                    ));
                }
                Some(_) => {}
            },
            SectionKind::Shell { thickness, .. } | SectionKind::Membrane { thickness, .. } => {
                if !(thickness.is_finite() && *thickness > 0.0) {
                    return Some(format!(
                        "The {} thickness must be greater than 0",
                        self.kind.label().to_ascii_lowercase()
                    ));
                }
            }
        }
        let elements = self.region.elements(mesh);
        for element in elements.iter().filter_map(|&id| mesh.element(id)) {
            if let Some(reason) = self.kind.rejects(element, profile) {
                return Some(reason);
            }
            if let SectionKind::Beam(beam) = &self.kind
                && beam
                    .orientation
                    .normal_for(line_tangent(mesh, element))
                    .is_none()
            {
                return Some(format!(
                    "The normal is parallel to the axis of element {}",
                    element.id
                ));
            }
        }
        None
    }
}

/// What the master or slave region of a contact or tie refers to that the mesh lacks.
fn master_slave_reference(master: &Region, slave: &Region, mesh: &FeMesh) -> Option<String> {
    (master
        .missing_reference(mesh)
        .map(|r| format!("Master: {r}")))
    .or_else(|| slave.missing_reference(mesh).map(|r| format!("Slave: {r}")))
}

impl Region {
    /// What the region refers to that the mesh does not have, e.g. "Node Set FIX does not
    /// exist".
    pub fn missing_reference(&self, mesh: &FeMesh) -> Option<String> {
        match self {
            Region::Parts(names) => names
                .iter()
                .find(|name| !mesh.parts.iter().any(|p| &p.name == *name))
                .map(|name| format!("Part {name} does not exist")),
            Region::NodeSet(name) => (!mesh.node_sets.contains_key(name))
                .then(|| format!("Node Set {name} does not exist")),
            Region::ElementSet(name) => (!mesh.element_sets.contains_key(name))
                .then(|| format!("Element Set {name} does not exist")),
            Region::Surface(name) => (!mesh.surfaces.contains_key(name))
                .then(|| format!("Surface {name} does not exist")),
            Region::Nodes(nodes) => {
                let missing = nodes.iter().filter(|&&n| mesh.node(n).is_none()).count();
                (missing > 0).then(|| format!("{missing} nodes do not exist"))
            }
            Region::Faces(faces) => {
                let missing = faces
                    .iter()
                    .filter(|&&(element, face)| {
                        mesh.element(element)
                            .is_none_or(|e| face == 0 || usize::from(face) > e.faces().len())
                    })
                    .count();
                (missing > 0).then(|| format!("{missing} element faces do not exist"))
            }
            Region::Edges(edges) => {
                let missing = edges.len() - mesh.edge_lines(edges).len();
                (missing > 0).then(|| format!("{missing} element edges do not exist"))
            }
            // Checked against the model's reference points and boundary conditions by the
            // caller.
            Region::ReferencePoint(_) | Region::RemotePoint(_) => None,
            Region::Geometry(entities) => {
                if mesh.cad.is_empty() {
                    return Some("The mesh was not created from the geometry".into());
                }
                let lost = entities.iter().filter(|e| e.is_lost()).count();
                match lost {
                    0 => {}
                    1 => {
                        return Some(
                            "1 picked geometry entity was not found in the changed geometry; \
                             pick it again"
                                .into(),
                        );
                    }
                    _ => {
                        return Some(format!(
                            "{lost} picked geometry entities were not found in the changed \
                             geometry; pick them again"
                        ));
                    }
                }
                let missing = entities.iter().filter(|&&e| !mesh.cad.contains(e)).count();
                (missing > 0).then(|| format!("{missing} geometry entities are not meshed"))
            }
        }
    }
}

/// What is wrong with the coordinate system of a force, moment or excitation: it does not
/// exist or its points do not span it, or components follow time tables of their own
/// (their amplitudes belong to global directions).
fn load_system_problem(model: &FeModel, load: &crate::Load) -> Option<String> {
    let system = match load.component_system(model) {
        Ok(Some(system)) => system,
        Ok(None) => return None,
        Err(_) => return Some("The coordinate system does not exist".into()),
    };
    if let Err(reason) = system.axes() {
        return Some(reason);
    }
    (load.tables.iter().any(Option::is_some))
        .then(|| "Time tables of the components need the global coordinate system".into())
}

#[cfg(test)]
mod tests {
    use plx_mesh::{Element, ElementShape, Part};

    use super::*;
    use crate::{
        BoundaryCondition, BoundaryKind, ContactPair, Material, Section, Step, SurfaceInteraction,
    };

    fn mesh() -> FeMesh {
        let mut mesh = FeMesh::default();
        for (id, coords) in [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        .into_iter()
        .enumerate()
        {
            mesh.set_node(id as u32 + 1, coords);
        }
        mesh.add_element(Element {
            id: 1,
            type_name: "C3D4".into(),
            shape: ElementShape::Tet4,
            nodes: vec![1, 2, 3, 4],
        })
        .unwrap();
        mesh.parts.push(Part {
            bodies: Vec::new(),
            name: "PART-1".into(),
            elements: vec![1],
        });
        mesh.node_sets.insert("FIX".into(), vec![1]);
        mesh
    }

    fn model() -> FeModel {
        let mut step = Step::new_static("Step-1");
        step.boundary_conditions.push(BoundaryCondition {
            id: crate::Id::new(),
            distribution: None,
            name: "Fixed-1".into(),
            active: true,
            visible: true,
            region: Region::NodeSet("FIX".into()),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        let steel = crate::Id::new();
        FeModel {
            properties: Default::default(),
            materials: vec![Material {
                id: steel,
                name: "Steel".into(),
                density: None,
                elastic: None,
                ..Default::default()
            }],
            sections: vec![Section {
                orientation: None,
                name: "Section-1".into(),
                material: Some(steel),
                region: Region::Parts(vec!["PART-1".into()]),
                thickness: 1.0,
                kind: SectionKind::Solid,
            }],
            steps: vec![step],
            ..FeModel::default()
        }
    }

    #[test]
    fn complete_model_is_valid() {
        assert_eq!(model().invalid_items(&mesh()), []);
    }

    #[test]
    fn beam_section_without_its_cross_section_is_invalid() {
        let mut model = model();
        let tube = crate::Id::new();
        model.sections[0].kind = SectionKind::Beam(crate::BeamSection {
            cross_section: Some(tube),
            ..crate::BeamSection::DEFAULT
        });
        let invalid = model.invalid_items(&mesh());
        assert_eq!(invalid.len(), 1);
        assert_eq!(invalid[0].item, ModelItem::Section(0));
        assert!(invalid[0].reason.contains("does not exist"));
        model.cross_sections.push(crate::CrossSection {
            id: tube,
            name: "Tube".into(),
            profile: crate::BeamProfile::Pipe {
                radius: 1.0,
                thickness: 2.0,
            },
        });
        // The wall is thicker than the radius.
        let invalid = model.invalid_items(&mesh());
        assert_eq!(invalid.len(), 1);
        assert_eq!(invalid[0].item, ModelItem::CrossSection(0));
    }

    #[test]
    fn section_without_its_material_is_invalid_until_it_is_back() {
        let mut model = model();
        let steel = model.materials.remove(0);
        let invalid = model.invalid_items(&mesh());
        assert_eq!(invalid.len(), 1);
        assert_eq!(invalid[0].item, ModelItem::Section(0));
        assert!(invalid[0].reason.contains("material"));
        model.materials.push(steel);
        assert_eq!(model.invalid_items(&mesh()), []);
    }

    #[test]
    fn references_follow_amplitudes() {
        let mesh = mesh();
        let mut model = model();
        let ramp = crate::Amplitude::new("Ramp");
        model.steps[0].boundary_conditions[0].amplitude = Some(ramp.id);
        let invalid = model.invalid_items(&mesh);
        assert_eq!(invalid[0].item, ModelItem::BoundaryCondition(0, 0));
        assert!(invalid[0].reason.contains("amplitude"));
        model.amplitudes.push(ramp);
        assert_eq!(model.invalid_items(&mesh), []);
        // A renamed amplitude is still the one of the boundary condition.
        model.amplitudes[0].name = "Ramp-2".into();
        assert_eq!(model.invalid_items(&mesh), []);
        model.amplitudes[0].points.clear();
        assert_eq!(model.invalid_items(&mesh)[0].item, ModelItem::Amplitude(0));
    }

    #[test]
    fn missing_sets_and_mesh_entities_are_found() {
        let mesh = mesh();
        let mut model = model();
        model.steps[0].boundary_conditions[0].region = Region::NodeSet("GONE".into());
        assert_eq!(
            model.invalid_items(&mesh)[0].item,
            ModelItem::BoundaryCondition(0, 0)
        );
        assert!(
            Region::Nodes(vec![1, 99])
                .missing_reference(&mesh)
                .is_some()
        );
        assert!(
            Region::Faces(vec![(1, 4)])
                .missing_reference(&mesh)
                .is_none()
        );
        assert!(
            Region::Faces(vec![(1, 5)])
                .missing_reference(&mesh)
                .is_some()
        );
        assert!(
            Region::Faces(vec![(2, 1)])
                .missing_reference(&mesh)
                .is_some()
        );
        assert!(
            Region::Parts(vec!["PART-2".into()])
                .missing_reference(&mesh)
                .is_some()
        );
    }

    #[test]
    fn contact_pair_needs_its_interaction_and_surfaces() {
        let mesh = mesh();
        let mut model = model();
        let interaction = crate::Id::new();
        let mut pair = ContactPair::new("Contact_Pair-1", Some(interaction));
        pair.master = Region::Faces(vec![(1, 1)]);
        pair.slave = Region::Faces(vec![(1, 2)]);
        model.contact_pairs.push(pair);
        let invalid = model.invalid_items(&mesh);
        assert_eq!(invalid[0].item, ModelItem::ContactPair(0));
        assert!(invalid[0].reason.contains("surface interaction"));
        model.surface_interactions.push(SurfaceInteraction {
            id: interaction,
            name: "Surface_Interaction-1".into(),
            properties: Vec::new(),
        });
        assert_eq!(model.invalid_items(&mesh), []);
        model.contact_pairs[0].slave = Region::Surface("GONE".into());
        assert!(model.invalid_items(&mesh)[0].reason.starts_with("Slave"));
    }
}
