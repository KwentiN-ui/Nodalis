//! Validation of an analysis: every reference (set, surface, material, interaction,
//! amplitude, node, element) points at something defined, every element has a section, and
//! the sections are assigned to the elements. The reader calls it once the input is read;
//! Nodalis calls it on the model it lowers.

use crate::contact::ContactKind;
use crate::diagnostic::ItemRef;
use crate::element::ElementFamily;
use crate::error::Error;
use crate::mesh::{ElementRef, NodeRef, Surface};
use crate::model::Analysis;
use crate::section::SectionKind;
use crate::step::{ContactChange, DefinedFieldKind, Procedure};

/// Checks the analysis and assigns sections to elements. Returns every problem found, so
/// that the user sees them all at once, or the warnings of what was tolerated.
pub fn validate(analysis: &mut Analysis) -> Result<Vec<String>, Vec<Error>> {
    let mut errors = Vec::new();
    let mut warnings = Vec::new();
    let model = &mut analysis.model;

    // Element nodes.
    for element in &model.elements {
        let before = errors.len();
        if element.nodes.len() != element.kind.node_count() {
            errors.push(Error::input(
                "*ELEMENT",
                format!(
                    "element {} of type {} has {} nodes instead of {}",
                    element.id,
                    element.kind,
                    element.nodes.len(),
                    element.kind.node_count()
                ),
            ));
        }
        for &node in &element.nodes {
            if model.nodes.index(node).is_none() {
                errors.push(Error::undefined(
                    "*ELEMENT",
                    "node",
                    format!("{node} (element {})", element.id),
                ));
            }
        }
        concerning(&mut errors, before, ItemRef::Element(element.id));
    }

    // Sets and surfaces. CalculiX tolerates set members that are not defined (it allocates
    // by the largest number); they are dropped here with a warning.
    let nodes = &model.nodes;
    for set in &mut model.sets.node_sets {
        let before = set.nodes.len();
        set.nodes.retain(|&node| nodes.index(node).is_some());
        if set.nodes.len() != before {
            warnings.push(format!(
                "*WARNING in *NSET: {} undefined node(s) left out of set {}",
                before - set.nodes.len(),
                set.name
            ));
        }
    }
    let defined: std::collections::HashSet<_> = model.elements.iter().map(|e| e.id).collect();
    for set in &mut model.sets.element_sets {
        let before = set.elements.len();
        set.elements.retain(|element| defined.contains(element));
        if set.elements.len() != before {
            warnings.push(format!(
                "*WARNING in *ELSET: {} undefined element(s) left out of set {}",
                before - set.elements.len(),
                set.name
            ));
        }
    }
    for crate::mesh::NamedSurface { name, surface } in &model.sets.surfaces {
        match surface {
            Surface::Element(faces) => {
                for face in faces {
                    if model.element_index(face.element).is_none() {
                        errors.push(Error::undefined(
                            "*SURFACE",
                            "element",
                            format!("{} (surface {name})", face.element),
                        ));
                    }
                }
            }
            Surface::Node(nodes) => {
                for &node in nodes {
                    if model.nodes.index(node).is_none() {
                        errors.push(Error::undefined(
                            "*SURFACE",
                            "node",
                            format!("{node} (surface {name})"),
                        ));
                    }
                }
            }
        }
    }

    // Sections: materials and element sets, then the assignment.
    let mut assignment: Vec<Option<u32>> = vec![None; model.elements.len()];
    let mut before_section = errors.len();
    for (index, section) in model.sections.iter().enumerate() {
        concerning(
            &mut errors,
            before_section,
            ItemRef::Section(index.wrapping_sub(1)),
        );
        before_section = errors.len();
        let keyword = match &section.kind {
            SectionKind::Solid { .. } => "*SOLID SECTION",
            SectionKind::Shell { .. } => "*SHELL SECTION",
            SectionKind::Membrane { .. } => "*MEMBRANE SECTION",
            SectionKind::Beam { .. } => "*BEAM SECTION",
            SectionKind::Spring(_) => "*SPRING",
            SectionKind::Gap(_) => "*GAP",
        };
        if let Some(material) = section.kind.material()
            && model.material(material).is_none()
        {
            errors.push(Error::undefined(
                keyword,
                "material",
                format!("#{}", material.0),
            ));
        }
        let Some(set) = model.sets.element_sets.get(section.elset.index()) else {
            errors.push(Error::undefined(
                keyword,
                "element set",
                format!("#{}", section.elset.0),
            ));
            continue;
        };
        let elements = &set.elements;
        for &id in elements {
            if let Some(i) = model.element_index(id) {
                let element = &model.elements[i as usize];
                let fits = matches!(
                    (&section.kind, element.kind.family()),
                    (SectionKind::Solid { .. }, ElementFamily::Solid)
                        | (SectionKind::Solid { .. }, ElementFamily::PlaneStress)
                        | (SectionKind::Solid { .. }, ElementFamily::PlaneStrain)
                        | (SectionKind::Solid { .. }, ElementFamily::Axisymmetric)
                        | (SectionKind::Solid { .. }, ElementFamily::Truss)
                        | (SectionKind::Shell { .. }, ElementFamily::Shell)
                        | (SectionKind::Membrane { .. }, ElementFamily::Membrane)
                        | (SectionKind::Beam { .. }, ElementFamily::Beam)
                        | (SectionKind::Beam { .. }, ElementFamily::User)
                        | (SectionKind::Spring(_), ElementFamily::Spring)
                        | (SectionKind::Gap(_), ElementFamily::Gap)
                );
                if !fits {
                    errors.push(Error::input(
                        keyword,
                        format!(
                            "element {id} of type {} does not fit the section on set {}",
                            element.kind, set.name
                        ),
                    ));
                }
                assignment[i as usize] = Some(index as u32);
            }
        }
    }
    if let Some(last) = model.sections.len().checked_sub(1) {
        concerning(&mut errors, before_section, ItemRef::Section(last));
    }
    // A mesh without steps (a pure mesh file) needs no sections.
    let needs_sections = !analysis.steps.is_empty();
    for (element, section) in model.elements.iter_mut().zip(assignment) {
        element.section = section;
        if section.is_none() && needs_sections {
            errors.push(
                Error::input("*ELEMENT", format!("element {} has no section", element.id))
                    .concerning(ItemRef::Element(element.id)),
            );
        }
    }

    // Constraints.
    let node_known = |model: &crate::Model, region: &NodeRef| match region {
        NodeRef::Node(id) => model.nodes.index(*id).is_some(),
        NodeRef::Set(set) => set.index() < model.sets.node_sets.len(),
    };
    let element_known = |model: &crate::Model, region: &ElementRef| match region {
        ElementRef::Element(id) => model.element_index(*id).is_some(),
        ElementRef::Set(set) => set.index() < model.sets.element_sets.len(),
    };
    let kind_of_node_ref = |region: &NodeRef| match region {
        NodeRef::Node(_) => "node",
        NodeRef::Set(_) => "node set",
    };
    let kind_of_element_ref = |region: &ElementRef| match region {
        ElementRef::Element(_) => "element",
        ElementRef::Set(_) => "element set",
    };
    for equation in &model.constraints.equations {
        for term in &equation.terms {
            if !node_known(model, &term.node) {
                errors.push(Error::undefined(
                    "*EQUATION",
                    kind_of_node_ref(&term.node),
                    term.node.describe(&model.sets),
                ));
            }
        }
    }
    for (index, body) in model.constraints.rigid_bodies.iter().enumerate() {
        let before = errors.len();
        match &body.nodes {
            crate::constraints::RigidBodyNodes::NodeSet(set) => {
                if set.index() >= model.sets.node_sets.len() {
                    errors.push(Error::undefined("*RIGID BODY", "node set", handle(set.0)));
                }
            }
            crate::constraints::RigidBodyNodes::ElementSet(set) => {
                if set.index() >= model.sets.element_sets.len() {
                    errors.push(Error::undefined(
                        "*RIGID BODY",
                        "element set",
                        handle(set.0),
                    ));
                }
            }
        }
        for node in [body.reference_node, body.rotation_node] {
            if model.nodes.index(node).is_none() {
                errors.push(Error::undefined("*RIGID BODY", "node", node.to_string()));
            }
        }
        concerning(&mut errors, before, ItemRef::RigidBody(index));
    }
    for mass in &model.point_masses {
        let nodes = std::iter::once(mass.node).chain(mass.inertia.map(|(r, _)| r));
        for node in nodes {
            if model.nodes.index(node).is_none() {
                errors.push(Error::undefined("*POINT MASS", "node", node.to_string()));
            }
        }
        for set in mass.elset.iter().chain(&mass.name) {
            if set.index() >= model.sets.element_sets.len() {
                errors.push(Error::undefined(
                    "*POINT MASS",
                    "element set",
                    handle(set.0),
                ));
            }
        }
    }
    for joint in &model.constraints.joints {
        let before = errors.len();
        for node in joint.ends.iter().flatten().flatten() {
            if model.nodes.index(*node).is_none() {
                errors.push(Error::undefined("*JOINT", "node", node.to_string()));
            }
        }
        if joint.ends.iter().all(Option::is_none) {
            errors.push(Error::input("*JOINT", "both ends are ground"));
        }
        concerning(&mut errors, before, ItemRef::Joint(joint.name.clone()));
    }
    for tie in &model.constraints.ties {
        for surface in [tie.slave, tie.master] {
            if model.sets.surface(surface).is_none() {
                errors.push(
                    Error::undefined("*TIE", "surface", handle(surface.0))
                        .concerning(ItemRef::Tie(tie.name.clone())),
                );
            }
        }
    }
    if let Some(cyclic) = &model.constraints.cyclic_symmetry
        && cyclic.tie.index() >= model.constraints.ties.len()
    {
        errors.push(Error::undefined(
            "*CYCLIC SYMMETRY MODEL",
            "tie",
            handle(cyclic.tie.0),
        ));
    }
    for transform in &model.constraints.transforms {
        if transform.nset.index() >= model.sets.node_sets.len() {
            errors.push(Error::undefined(
                "*TRANSFORM",
                "node set",
                handle(transform.nset.0),
            ));
        }
    }
    if let Some(submodel) = &model.constraints.submodel {
        for set in &submodel.node_sets {
            if set.index() >= model.sets.node_sets.len() {
                errors.push(Error::undefined("*SUBMODEL", "node set", handle(set.0)));
            }
        }
    }

    // Contact.
    for (index, pair) in model.contact.pairs.iter().enumerate() {
        let before = errors.len();
        if model
            .contact
            .interactions
            .get(pair.interaction.index())
            .is_none()
        {
            errors.push(Error::undefined(
                "*CONTACT PAIR",
                "surface interaction",
                format!("#{}", pair.interaction.0),
            ));
        }
        for surface in [pair.slave, pair.master] {
            if model.sets.surface(surface).is_none() {
                errors.push(Error::undefined(
                    "*CONTACT PAIR",
                    "surface",
                    handle(surface.0),
                ));
            }
        }
        if let Some(Surface::Node(_)) = model.sets.surface(pair.master) {
            errors.push(Error::input(
                "*CONTACT PAIR",
                format!(
                    "master surface {} must be an element surface",
                    model.sets.surface_name(pair.master)
                ),
            ));
        }
        if pair.kind != ContactKind::NodeToSurface
            && let Some(Surface::Node(_)) = model.sets.surface(pair.slave)
        {
            errors.push(Error::input(
                "*CONTACT PAIR",
                format!(
                    "slave surface {} must be an element surface for surface to surface contact",
                    model.sets.surface_name(pair.slave)
                ),
            ));
        }
        concerning(&mut errors, before, ItemRef::ContactPair(index));
    }
    for clearance in &model.contact.clearances {
        for surface in [clearance.slave, clearance.master] {
            if model.sets.surface(surface).is_none() {
                errors.push(Error::undefined("*CLEARANCE", "surface", handle(surface.0)));
            }
        }
    }
    for support in &model.contact.compression_only {
        if model.sets.surface(support.surface).is_none() {
            errors.push(Error::undefined(
                "*COMPRESSION ONLY",
                "surface",
                handle(support.surface.0),
            ));
        }
    }

    // Initial conditions and design variables.
    for (region, _) in &model.initial.temperatures {
        if !node_known(model, region) {
            errors.push(Error::undefined(
                "*INITIAL CONDITIONS",
                kind_of_node_ref(region),
                region.describe(&model.sets),
            ));
        }
    }
    for (region, _, _) in model
        .initial
        .velocities
        .iter()
        .chain(&model.initial.displacements)
    {
        if !node_known(model, region) {
            errors.push(Error::undefined(
                "*INITIAL CONDITIONS",
                kind_of_node_ref(region),
                region.describe(&model.sets),
            ));
        }
    }
    if let Some(design) = &model.design_variables
        && design.nset.index() >= model.sets.node_sets.len()
    {
        errors.push(Error::undefined(
            "*DESIGN VARIABLES",
            "node set",
            handle(design.nset.0),
        ));
    }

    // Steps.
    let amplitude = |errors: &mut Vec<Error>, keyword: &str, id: &Option<crate::AmplitudeId>| {
        if let Some(id) = id
            && model.amplitude(*id).is_none()
        {
            errors.push(Error::undefined(keyword, "amplitude", format!("#{}", id.0)));
        }
    };
    let stored_modes_before = |steps: &[crate::Step], index: usize| {
        steps[..index]
            .iter()
            .any(|s| matches!(&s.procedure, Procedure::Frequency(f) if f.storage))
    };
    for (index, step) in analysis.steps.iter().enumerate() {
        let before = errors.len();
        let name = &step.name;
        for boundary in &step.boundaries {
            if !node_known(model, &boundary.region) {
                errors.push(Error::undefined(
                    "*BOUNDARY",
                    kind_of_node_ref(&boundary.region),
                    format!("{} (step {name})", boundary.region.describe(&model.sets)),
                ));
            }
            amplitude(&mut errors, "*BOUNDARY", &boundary.amplitude);
            if boundary.submodel_step.is_some() && model.constraints.submodel.is_none() {
                errors.push(Error::input(
                    "*BOUNDARY",
                    format!("SUBMODEL in step {name} needs a *SUBMODEL definition"),
                ));
            }
        }
        for load in &step.loads.concentrated {
            if !node_known(model, &load.region) {
                errors.push(Error::undefined(
                    "*CLOAD",
                    kind_of_node_ref(&load.region),
                    format!("{} (step {name})", load.region.describe(&model.sets)),
                ));
            }
            amplitude(&mut errors, "*CLOAD", &load.amplitude);
        }
        for load in &step.loads.distributed {
            if !element_known(model, &load.region) && !point_mass_named(model, load) {
                errors.push(Error::undefined(
                    "*DLOAD",
                    kind_of_element_ref(&load.region),
                    format!("{} (step {name})", load.region.describe(&model.sets)),
                ));
            }
            amplitude(&mut errors, "*DLOAD", &load.amplitude);
        }
        for flux in &step.loads.concentrated_flux {
            if !node_known(model, &flux.region) {
                errors.push(Error::undefined(
                    "*CFLUX",
                    kind_of_node_ref(&flux.region),
                    format!("{} (step {name})", flux.region.describe(&model.sets)),
                ));
            }
            amplitude(&mut errors, "*CFLUX", &flux.amplitude);
        }
        for flux in &step.loads.distributed_flux {
            if !element_known(model, &flux.region) {
                errors.push(Error::undefined(
                    "*DFLUX",
                    kind_of_element_ref(&flux.region),
                    format!("{} (step {name})", flux.region.describe(&model.sets)),
                ));
            }
            amplitude(&mut errors, "*DFLUX", &flux.amplitude);
        }
        for film in &step.loads.films {
            if !element_known(model, &film.region) {
                errors.push(Error::undefined(
                    "*FILM",
                    kind_of_element_ref(&film.region),
                    format!("{} (step {name})", film.region.describe(&model.sets)),
                ));
            }
            amplitude(&mut errors, "*FILM", &film.amplitude);
            amplitude(&mut errors, "*FILM", &film.film_amplitude);
        }
        for radiate in &step.loads.radiations {
            if !element_known(model, &radiate.region) {
                errors.push(Error::undefined(
                    "*RADIATE",
                    kind_of_element_ref(&radiate.region),
                    format!("{} (step {name})", radiate.region.describe(&model.sets)),
                ));
            }
            amplitude(&mut errors, "*RADIATE", &radiate.amplitude);
            amplitude(&mut errors, "*RADIATE", &radiate.radiation_amplitude);
            if model.physical.stefan_boltzmann.is_none() || model.physical.absolute_zero.is_none() {
                errors.push(Error::input(
                    "*RADIATE",
                    "radiation needs *PHYSICAL CONSTANTS with the absolute zero and the Stefan-Boltzmann constant",
                ));
            }
        }
        for field in &step.fields {
            amplitude(&mut errors, "*TEMPERATURE", &field.amplitude);
            if let DefinedFieldKind::Temperature(values) = &field.kind {
                for (region, _) in values {
                    if !node_known(model, region) {
                        errors.push(Error::undefined(
                            "*TEMPERATURE",
                            kind_of_node_ref(region),
                            format!("{} (step {name})", region.describe(&model.sets)),
                        ));
                    }
                }
            }
        }
        for change in &step.contact_changes {
            match change {
                ContactChange::ContactPair { slave, master, .. } => {
                    if !(model.contact.pairs.iter())
                        .any(|p| &p.slave == slave && &p.master == master)
                    {
                        errors.push(Error::undefined(
                            "*MODEL CHANGE",
                            "contact pair",
                            format!(
                                "{}, {}",
                                model.sets.surface_name(*slave),
                                model.sets.surface_name(*master)
                            ),
                        ));
                    }
                }
                ContactChange::Elements { elset, .. } => {
                    if elset.index() >= model.sets.element_sets.len() {
                        errors.push(Error::undefined(
                            "*MODEL CHANGE",
                            "element set",
                            handle(elset.0),
                        ));
                    }
                }
                ContactChange::Friction { interaction, .. } => {
                    if model
                        .contact
                        .interactions
                        .get(interaction.index())
                        .is_none()
                    {
                        errors.push(Error::undefined(
                            "*CHANGE FRICTION",
                            "surface interaction",
                            format!("#{}", interaction.0),
                        ));
                    }
                }
            }
        }
        for output in (step.outputs.node_file.iter()).chain(&step.outputs.el_file) {
            if let Some(nset) = output.nset
                && nset.index() >= model.sets.node_sets.len()
            {
                errors.push(Error::undefined("*NODE FILE", "node set", handle(nset.0)));
            }
        }
        for (keyword, output) in (step.outputs.node_print.iter().map(|o| ("*NODE PRINT", o)))
            .chain(step.outputs.el_print.iter().map(|o| ("*EL PRINT", o)))
        {
            match output.set {
                Some(crate::step::HistorySet::Nodes(set))
                    if set.index() >= model.sets.node_sets.len() =>
                {
                    errors.push(Error::undefined(keyword, "node set", handle(set.0)));
                }
                Some(crate::step::HistorySet::Elements(set))
                    if set.index() >= model.sets.element_sets.len() =>
                {
                    errors.push(Error::undefined(keyword, "element set", handle(set.0)));
                }
                _ => {}
            }
        }
        match &step.procedure {
            Procedure::ModalDynamics(_)
            | Procedure::SteadyStateDynamics(_)
            | Procedure::ComplexFrequency(_)
                if !stored_modes_before(&analysis.steps, index)
                    && model.restart_read.is_none()
                    && model.stored_modes_file.is_none() =>
            {
                errors.push(Error::input(
                    step.procedure.keyword(),
                    format!(
                        "step {name} needs the modes of an earlier *FREQUENCY step with STORAGE=YES"
                    ),
                ));
            }
            Procedure::Sensitivity(sensitivity) => {
                for response in &sensitivity.responses {
                    if let Some(elset) = response.elset
                        && elset.index() >= model.sets.element_sets.len()
                    {
                        errors.push(Error::undefined(
                            "*DESIGN RESPONSE",
                            "element set",
                            handle(elset.0),
                        ));
                    }
                }
            }
            _ => {}
        }
        concerning(&mut errors, before, ItemRef::Step(index));
    }

    if errors.is_empty() {
        Ok(warnings)
    } else {
        Err(errors)
    }
}

/// How a message names an item by its handle.
fn handle(index: u32) -> String {
    format!("#{index}")
}

/// Names `item` as what the errors from `from` on are about.
fn concerning(errors: &mut [Error], from: usize, item: ItemRef) {
    for error in errors.iter_mut().skip(from) {
        if error.item().is_none() {
            let plain = std::mem::replace(error, Error::Unsupported(String::new()));
            *error = plain.concerning(item.clone());
        }
    }
}

/// Whether a gravity or centrifugal load names point masses (`*POINT MASS, NAME=`), which
/// it pulls without elements.
fn point_mass_named(model: &crate::Model, load: &crate::step::DistributedLoad) -> bool {
    let crate::mesh::ElementRef::Set(name) = &load.region else {
        return false;
    };
    matches!(
        load.kind,
        crate::step::DistributedLoadKind::Gravity { .. }
            | crate::step::DistributedLoadKind::Centrifugal { .. }
    ) && (model.point_masses.iter()).any(|m| m.name.as_ref() == Some(name))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::element::{Element, ElementType};
    use crate::material::{Elastic, ElasticRow, Material};
    use crate::section::Section;
    use crate::step::{Static, Step};

    fn small_analysis() -> Analysis {
        let mut analysis = Analysis::default();
        let model = &mut analysis.model;
        for (id, c) in [
            (1, [0.0, 0.0, 0.0]),
            (2, [1.0, 0.0, 0.0]),
            (3, [0.0, 1.0, 0.0]),
            (4, [0.0, 0.0, 1.0]),
        ] {
            model.nodes.insert(id, c);
        }
        model.insert_element(Element {
            id: 1,
            kind: ElementType::C3D4,
            nodes: vec![1, 2, 3, 4],
            section: None,
        });
        model.sets.add_element_set("EALL", [1]);
        model.sets.add_node_set("FIX", [1, 2, 3]);
        model.materials.push(Material {
            name: "STEEL".into(),
            elastic: Some(Elastic {
                rows: vec![ElasticRow {
                    young: 210e3,
                    poisson: 0.3,
                    temperature: 0.0,
                }],
            }),
            ..Default::default()
        });
        model.sections.push(Section {
            elset: crate::ElementSetId(0),
            kind: SectionKind::Solid {
                material: crate::MaterialId(0),
                thickness: None,
                orientation: None,
            },
        });
        analysis
            .steps
            .push(Step::new("Step-1", Procedure::Static(Static::default())));
        analysis
    }

    #[test]
    fn a_complete_model_validates_and_gets_its_sections() {
        let mut analysis = small_analysis();
        validate(&mut analysis).unwrap();
        assert_eq!(analysis.model.elements[0].section, Some(0));
    }

    #[test]
    fn missing_references_are_reported_with_their_keyword() {
        let mut analysis = small_analysis();
        analysis.model.sections[0].kind = SectionKind::Solid {
            material: crate::MaterialId(7),
            thickness: None,
            orientation: None,
        };
        analysis.steps[0].boundaries.push(crate::step::Boundary {
            region: NodeRef::Set(crate::NodeSetId(7)),
            first_dof: 1,
            last_dof: 3,
            value: None,
            amplitude: Some(crate::AmplitudeId(3)),
            fixed: false,
            submodel_step: None,
            load_case: None,
        });
        let errors = validate(&mut analysis).unwrap_err();
        let texts: Vec<String> = errors.iter().map(|e| e.to_string()).collect();
        assert_eq!(
            texts,
            [
                "*ERROR in *SOLID SECTION: material #7 has not been defined",
                "*ERROR in *BOUNDARY: node set #7 (step Step-1) has not been defined",
                "*ERROR in *BOUNDARY: amplitude #3 has not been defined",
            ]
        );
    }

    #[test]
    fn errors_name_the_item_they_are_about() {
        let mut analysis = small_analysis();
        analysis.model.sections[0].kind = SectionKind::Solid {
            material: crate::MaterialId(7),
            thickness: None,
            orientation: None,
        };
        analysis.steps[0].boundaries.push(crate::step::Boundary {
            region: NodeRef::Set(crate::NodeSetId(7)),
            first_dof: 1,
            last_dof: 3,
            value: None,
            amplitude: None,
            fixed: false,
            submodel_step: None,
            load_case: None,
        });
        let errors = validate(&mut analysis).unwrap_err();
        let items: Vec<_> = errors.iter().map(Error::item).collect();
        assert_eq!(items, [Some(&ItemRef::Section(0)), Some(&ItemRef::Step(0))]);
        // The message stays as it was, and the diagnostic carries the item.
        assert!(
            errors[0]
                .to_string()
                .starts_with("*ERROR in *SOLID SECTION")
        );
        let diagnostic = crate::Diagnostic::from_error(&errors[1]);
        assert_eq!(diagnostic.item, Some(ItemRef::Step(0)));
        assert_eq!(diagnostic.code, crate::DiagnosticCode::Input);
    }

    #[test]
    fn an_element_without_section_is_named() {
        let mut analysis = small_analysis();
        analysis.model.sections.clear();
        let errors = validate(&mut analysis).unwrap_err();
        let id = analysis.model.elements[0].id;
        assert_eq!(errors[0].item(), Some(&ItemRef::Element(id)));
    }
}
