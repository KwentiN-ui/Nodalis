//! Finding the items of the model by their [`Id`], and every reference between items. Items
//! refer to each other only by id, so a renamed item keeps all its references; names are for
//! the user and the solver's input file.

use crate::{
    Amplitude, Bolt, BoundaryCondition, BoundaryKind, Constraint, ContactDetection, ContactPair,
    ContactRegion, CrossSection, Distribution, FeModel, HistoryKind, Id, InitialCondition, Load,
    LoadKind, Material, MeshOperation, MeshSet, PlaneSource, PointDefinition, PointRef, Region,
    Section, SectionKind, Step, SurfaceInteraction, Tie,
};

impl FeModel {
    pub fn amplitude(&self, id: Id<Amplitude>) -> Option<&Amplitude> {
        self.amplitudes.iter().find(|a| a.id == id)
    }

    pub fn distribution(&self, id: Id<Distribution>) -> Option<&Distribution> {
        self.distributions.iter().find(|d| d.id == id)
    }

    pub fn distribution_named(&self, name: &str) -> Option<&Distribution> {
        self.distributions.iter().find(|d| d.name == name)
    }

    pub fn cross_section(&self, id: Id<CrossSection>) -> Option<&CrossSection> {
        self.cross_sections.iter().find(|c| c.id == id)
    }

    pub fn cross_section_named(&self, name: &str) -> Option<&CrossSection> {
        self.cross_sections.iter().find(|c| c.name == name)
    }

    pub fn material(&self, id: Id<Material>) -> Option<&Material> {
        self.materials.iter().find(|m| m.id == id)
    }

    pub fn surface_interaction(&self, id: Id<SurfaceInteraction>) -> Option<&SurfaceInteraction> {
        self.surface_interactions.iter().find(|i| i.id == id)
    }

    pub fn contact_pair(&self, id: Id<ContactPair>) -> Option<&ContactPair> {
        self.contact_pairs.iter().find(|c| c.id == id)
    }

    pub fn contact_region(&self, id: Id<ContactRegion>) -> Option<&ContactRegion> {
        self.contact_regions.iter().find(|c| c.id == id)
    }

    pub fn contact_detection(&self, id: Id<ContactDetection>) -> Option<&ContactDetection> {
        self.contact_detections.iter().find(|d| d.id == id)
    }

    pub fn bolt(&self, id: Id<Bolt>) -> Option<&Bolt> {
        self.bolts.iter().find(|b| b.id == id)
    }

    pub fn step(&self, id: Id<Step>) -> Option<&Step> {
        self.steps.iter().find(|s| s.id == id)
    }

    /// The position of the step in the model's list of steps.
    pub fn step_index(&self, id: Id<Step>) -> Option<usize> {
        self.steps.iter().position(|s| s.id == id)
    }

    pub fn section(&self, id: Id<Section>) -> Option<&Section> {
        self.sections.iter().find(|s| s.id == id)
    }

    pub fn initial_condition(&self, id: Id<InitialCondition>) -> Option<&InitialCondition> {
        self.initial_conditions.iter().find(|c| c.id == id)
    }

    pub fn mesh_operation(&self, id: Id<MeshOperation>) -> Option<&MeshOperation> {
        self.mesh_operations.iter().find(|o| o.id == id)
    }

    pub fn mesh_set_by_id(&self, id: Id<MeshSet>) -> Option<&MeshSet> {
        self.mesh_sets.iter().find(|s| s.id == id)
    }

    pub fn tie(&self, id: Id<Tie>) -> Option<&Tie> {
        self.ties.iter().find(|t| t.id == id)
    }

    /// The first boundary condition with the id, of any step: the steps hold their own copies
    /// of a boundary condition that goes on, all with its id.
    pub fn boundary_condition(&self, id: Id<BoundaryCondition>) -> Option<&BoundaryCondition> {
        (self.steps.iter().flat_map(|s| &s.boundary_conditions)).find(|b| b.id == id)
    }

    /// The first load with the id, of any step.
    pub fn load(&self, id: Id<Load>) -> Option<&Load> {
        (self.steps.iter().flat_map(|s| &s.loads)).find(|l| l.id == id)
    }

    /// The name of the remote displacement or remote force a [`Region::RemotePoint`] is
    /// the point of.
    pub fn remote_item_name(&self, id: Id) -> Option<&str> {
        (self.boundary_condition(id.cast()).map(|b| b.name.as_str()))
            .or_else(|| self.load(id.cast()).map(|l| l.name.as_str()))
    }

    /// The name of the contact pair or contact region of the id, which contact history
    /// outputs and contact changes of steps refer to.
    pub fn contact_name(&self, id: Id) -> Option<&str> {
        (self.contact_pair(id.cast()).map(|c| c.name.as_str()))
            .or_else(|| self.contact_region(id.cast()).map(|c| c.name.as_str()))
    }

    /// The name of the item of the id, of whatever kind, for the GUI and messages.
    pub fn name_of<T: ?Sized>(&self, id: Id<T>) -> Option<&str> {
        let id = id.any();
        let features = (self
            .reference_points
            .iter()
            .find(|r| r.id == id.cast())
            .map(|r| &r.name))
        .or_else(|| self.coordinate_system(id.cast()).map(|c| &c.name))
        .or_else(|| self.plane(id.cast()).map(|p| &p.name));
        let items = || {
            (self.amplitude(id.cast()).map(|a| &a.name))
                .or_else(|| self.distribution(id.cast()).map(|d| &d.name))
                .or_else(|| self.cross_section(id.cast()).map(|c| &c.name))
                .or_else(|| self.material(id.cast()).map(|m| &m.name))
                .or_else(|| self.surface_interaction(id.cast()).map(|i| &i.name))
                .or_else(|| self.contact_detection(id.cast()).map(|d| &d.name))
                .or_else(|| self.bolt(id.cast()).map(|b| &b.name))
                .or_else(|| self.step(id.cast()).map(|s| &s.name))
                .or_else(|| self.section(id.cast()).map(|s| &s.name))
                .or_else(|| self.initial_condition(id.cast()).map(|c| &c.name))
                .or_else(|| self.mesh_operation(id.cast()).map(|o| &o.name))
                .or_else(|| self.mesh_set_by_id(id.cast()).map(|s| &s.name))
                .or_else(|| self.tie(id.cast()).map(|t| &t.name))
                .or_else(|| self.history_output(id.cast()).map(|h| &h.name))
        };
        (features.or_else(items).map(String::as_str))
            .or_else(|| self.contact_name(id))
            .or_else(|| self.remote_item_name(id))
            .or_else(|| {
                (self.constraints.iter())
                    .find(|c| c.id() == id)
                    .map(Constraint::name)
            })
    }

    /// The name of the item of the id, or a note that it is missing, for the GUI.
    pub fn name_or_missing<T: ?Sized>(&self, id: Option<Id<T>>) -> String {
        match id {
            None => "None".to_string(),
            Some(id) => self
                .name_of(id)
                .map_or_else(|| "(missing)".to_string(), str::to_string),
        }
    }

    /// Calls `visit` with every reference of an item to another, not the ids of the items
    /// themselves.
    pub fn for_each_reference(&mut self, visit: &mut dyn FnMut(&mut u64)) {
        fn optional<T: ?Sized>(id: &mut Option<Id<T>>, visit: &mut dyn FnMut(&mut u64)) {
            if let Some(id) = id {
                visit(id.raw_mut());
            }
        }
        let region = |region: &mut Region, visit: &mut dyn FnMut(&mut u64)| match region {
            Region::ReferencePoint(id) => visit(id.raw_mut()),
            Region::RemotePoint(id) => visit(id.raw_mut()),
            Region::Set(id) => visit(id.raw_mut()),
            _ => {}
        };
        let point = |point: &mut PointRef, visit: &mut dyn FnMut(&mut u64)| {
            if let PointRef::ReferencePoint(id) = point {
                visit(id.raw_mut());
            }
        };
        for r in self.regions_mut() {
            region(r, visit);
        }
        let filters = (self.contact_detections.iter_mut())
            .flat_map(|d| {
                std::iter::once(&mut d.scope)
                    .chain((d.rules.iter_mut()).flat_map(|r| [&mut r.side_a, &mut r.side_b]))
            })
            .chain(self.part_groups.iter_mut().map(|g| &mut g.filter));
        for filter in filters {
            for term in filter.include.iter_mut().chain(&mut filter.exclude) {
                match term {
                    crate::PartTerm::Material(id) => visit(id.raw_mut()),
                    crate::PartTerm::Group(id) => visit(id.raw_mut()),
                    _ => {}
                }
            }
        }
        for condition in &mut self.initial_conditions {
            region(&mut condition.region, visit);
            optional(&mut condition.distribution, visit);
        }
        for point in &mut self.reference_points {
            match &mut point.definition {
                PointDefinition::Local { system, .. } => optional(system, visit),
                PointDefinition::CenterOfGravity(r) | PointDefinition::BoundingBoxCenter(r) => {
                    region(r, visit)
                }
                PointDefinition::Global => {}
            }
        }
        for system in &mut self.coordinate_systems {
            optional(&mut system.base, visit);
            if let Some(r) = &mut system.attached {
                region(r, visit);
            }
        }
        for distribution in &mut self.distributions {
            optional(&mut distribution.system, visit);
        }
        for plane in &mut self.planes {
            match &mut plane.source {
                PlaneSource::PointNormal { point: p, .. } => point(p, visit),
                PlaneSource::ThreePoints { points } => {
                    for p in points {
                        point(p, visit);
                    }
                }
                PlaneSource::CoordinateSystem { system, .. } => optional(system, visit),
            }
        }
        for path in &mut self.result_paths {
            point(&mut path.start, visit);
            point(&mut path.end, visit);
        }
        for plane in &mut self.result_planes {
            visit(plane.plane.raw_mut());
        }
        for constraint in &mut self.constraints {
            match constraint {
                Constraint::RigidBody(body) => visit(body.reference_point.raw_mut()),
                Constraint::PointMass(mass) => {
                    visit(mass.reference_point.raw_mut());
                    if let Some(inertia) = &mut mass.inertia {
                        optional(&mut inertia.system, visit);
                    }
                }
                Constraint::Joint(joint) => optional(&mut joint.system, visit),
                _ => {}
            }
        }
        for section in &mut self.sections {
            optional(&mut section.material, visit);
            if let SectionKind::Beam(beam) = &mut section.kind {
                optional(&mut beam.cross_section, visit);
            }
        }
        for pair in &mut self.contact_pairs {
            optional(&mut pair.interaction, visit);
        }
        for region in &mut self.contact_regions {
            optional(&mut region.detected_by, visit);
        }
        for tie in &mut self.node_ties {
            optional(&mut tie.detected_by, visit);
        }
        for output in &mut self.history_outputs {
            if let HistoryKind::Contact { pair } = &mut output.kind {
                optional(pair, visit);
            }
        }
        for step in &mut self.steps {
            if let crate::StepKind::ComplexFrequency(complex) = &mut step.kind {
                optional(&mut complex.system, visit);
            }
            for bc in &mut step.boundary_conditions {
                optional(&mut bc.amplitude, visit);
                optional(&mut bc.distribution, visit);
                if let BoundaryKind::Displacement(displacement) = &mut bc.kind {
                    optional(&mut displacement.system, visit);
                }
            }
            for load in &mut step.loads {
                optional(&mut load.amplitude, visit);
                optional(&mut load.factor_amplitude, visit);
                optional(&mut load.distribution, visit);
                optional(&mut load.system, visit);
                match &mut load.kind {
                    LoadKind::RemoteForce(remote) => optional(&mut remote.system, visit),
                    LoadKind::Moving(moving) => optional(&mut moving.system, visit),
                    _ => {}
                }
            }
            for field in &mut step.defined_fields {
                optional(&mut field.amplitude, visit);
                optional(&mut field.distribution, visit);
            }
            for change in &mut step.contact_changes {
                visit(change.region.raw_mut());
            }
            for change in &mut step.bolt_changes {
                visit(change.bolt.raw_mut());
                optional(&mut change.amplitude, visit);
            }
        }
    }

    /// The kind, name and id of every item that others refer to.
    pub fn item_ids(&self) -> Vec<(&'static str, String, Id)> {
        let mut items = Vec::new();
        let mut add = |kind: &'static str, name: &str, id: Id| {
            items.push((kind, name.to_string(), id));
        };
        for r in &self.reference_points {
            add("RP", &r.name, r.id.any());
        }
        for c in &self.coordinate_systems {
            add("CS", &c.name, c.id.any());
        }
        for p in &self.planes {
            add("Plane", &p.name, p.id.any());
        }
        for m in &self.materials {
            add("Material", &m.name, m.id.any());
        }
        for c in &self.cross_sections {
            add("CrossSection", &c.name, c.id.any());
        }
        for i in &self.surface_interactions {
            add("Interaction", &i.name, i.id.any());
        }
        for c in &self.contact_pairs {
            add("ContactPair", &c.name, c.id.any());
        }
        for c in &self.contact_regions {
            add("ContactRegion", &c.name, c.id.any());
        }
        for d in &self.contact_detections {
            add("Detection", &d.name, d.id.any());
        }
        for g in &self.part_groups {
            add("PartGroup", &g.name, g.id.any());
        }
        for b in &self.bolts {
            add("Bolt", &b.name, b.id.any());
        }
        for a in &self.amplitudes {
            add("Amplitude", &a.name, a.id.any());
        }
        for d in &self.distributions {
            add("Distribution", &d.name, d.id.any());
        }
        for s in &self.sections {
            add("Section", &s.name, s.id.any());
        }
        for c in &self.initial_conditions {
            add("InitialCondition", &c.name, c.id.any());
        }
        for o in &self.mesh_operations {
            add("MeshOperation", &o.name, o.id.any());
        }
        for s in &self.mesh_sets {
            add("MeshSet", &s.name, s.id.any());
        }
        for t in &self.ties {
            add("Tie", &t.name, t.id.any());
        }
        for t in &self.node_ties {
            add("NodeTie", &t.name, t.id.any());
        }
        for c in &self.constraints {
            add("Constraint", c.name(), c.id().any());
        }
        for h in &self.history_outputs {
            add("HistoryOutput", &h.name, h.id.any());
        }
        for step in &self.steps {
            add("Step", &step.name, step.id.any());
            for f in &step.defined_fields {
                add("DefinedField", &f.name, f.id.any());
            }
            for b in &step.boundary_conditions {
                add("BoundaryCondition", &b.name, b.id.any());
            }
            for l in &step.loads {
                add("Load", &l.name, l.id.any());
            }
        }
        items
    }

    /// After items took the place of the model's items of the same kind and name, e.g.
    /// copied from a linked analysis, references to the items they replaced go to them;
    /// `before` is [`FeModel::item_ids`] from before the items were replaced.
    pub fn follow_replaced(&mut self, before: &[(&'static str, String, Id)]) {
        let after = self.item_ids();
        let gone = |id: Id| !after.iter().any(|(_, _, i)| *i == id);
        let moves: Vec<(Id, Id)> = (before.iter())
            .filter(|(_, _, old)| gone(*old))
            .filter_map(|(kind, name, old)| {
                let new = after.iter().find(|(k, n, _)| k == kind && n == name)?.2;
                Some((*old, new))
            })
            .collect();
        for (old, new) in moves {
            self.retarget(old, new);
        }
    }

    /// Makes every reference to the item `old` refer to `new`, e.g. when an item of the same
    /// name from another analysis takes the place of the model's own.
    pub fn retarget<T: ?Sized>(&mut self, old: Id<T>, new: Id<T>) {
        if old != new {
            self.for_each_reference(&mut |id| {
                if *id == old.raw() {
                    *id = new.raw();
                }
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{HistoryOutput, ReferencePoint, Step};

    /// The reported bug: a history output on a reference point lost it when the point was
    /// renamed, because it referred to the point by name.
    #[test]
    fn a_renamed_reference_point_keeps_its_history_output() {
        let mut model = FeModel::default();
        let point = ReferencePoint::new("RP-1", [0.0; 3]);
        let id = point.id;
        model.reference_points.push(point);
        model.steps.push(Step::new_static("Step-1"));
        let output = HistoryOutput::node("NH_Output-1", Region::ReferencePoint(id));
        model.put_history_output(0, output);

        model.reference_points[0].name = "Top".into();

        let region = model.history_outputs[0].kind.region().unwrap();
        assert_eq!(region.describe(&model, &plx_mesh::FeMesh::default()), "Top");
        assert_eq!(
            model.reference_point(id).map(|r| r.name.as_str()),
            Some("Top")
        );
    }

    #[test]
    fn references_are_retargeted() {
        let mut model = FeModel::default();
        let (old, new) = (Id::new(), Id::new());
        model.steps.push(Step::new_static("Step-1"));
        let output = HistoryOutput::node("NH_Output-1", Region::ReferencePoint(old));
        model.put_history_output(0, output);
        model.retarget(old, new);
        assert_eq!(
            model.history_outputs[0].kind.region(),
            Some(&Region::ReferencePoint(new))
        );
    }

    /// A reference point copied over one of the same name, as a link between analyses
    /// does, keeps the references to the one it replaced.
    #[test]
    fn references_follow_a_replaced_item() {
        let mut model = FeModel::default();
        model
            .reference_points
            .push(ReferencePoint::new("RP-1", [0.0; 3]));
        let old = model.reference_points[0].id;
        model.steps.push(Step::new_static("Step-1"));
        let output = HistoryOutput::node("NH_Output-1", Region::ReferencePoint(old));
        model.put_history_output(0, output);
        let before = model.item_ids();
        let other = ReferencePoint::new("RP-1", [1.0; 3]);
        let new = other.id;
        model.reference_points[0] = other;
        model.follow_replaced(&before);
        assert_eq!(
            model.history_outputs[0].kind.region(),
            Some(&Region::ReferencePoint(new))
        );
    }
}
