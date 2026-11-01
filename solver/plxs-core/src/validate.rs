//! Validation of an analysis: every reference (set, surface, material, interaction,
//! amplitude, node, element) points at something defined, every element has a section, and
//! the sections are assigned to the elements. The reader calls it once the input is read;
//! Nodalis will call it on a model it built itself.

use std::collections::{BTreeSet, HashSet};

use crate::contact::ContactKind;
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
    }

    // Sets and surfaces. CalculiX tolerates set members that are not defined (it allocates
    // by the largest number); they are dropped here with a warning.
    let nodes = &model.nodes;
    for (name, members) in &mut model.sets.node_sets {
        let before = members.len();
        members.retain(|&node| nodes.index(node).is_some());
        if members.len() != before {
            warnings.push(format!(
                "*WARNING in *NSET: {} undefined node(s) left out of set {name}",
                before - members.len()
            ));
        }
    }
    let defined: std::collections::HashSet<_> = model.elements.iter().map(|e| e.id).collect();
    for (name, members) in &mut model.sets.element_sets {
        let before = members.len();
        members.retain(|element| defined.contains(element));
        if members.len() != before {
            warnings.push(format!(
                "*WARNING in *ELSET: {} undefined element(s) left out of set {name}",
                before - members.len()
            ));
        }
    }
    for (name, surface) in &model.sets.surfaces {
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
    let material_names: HashSet<&str> = model.materials.iter().map(|m| m.name.as_str()).collect();
    let mut assignment: Vec<Option<u32>> = vec![None; model.elements.len()];
    for (index, section) in model.sections.iter().enumerate() {
        let keyword = match &section.kind {
            SectionKind::Solid { .. } => "*SOLID SECTION",
            SectionKind::Shell { .. } => "*SHELL SECTION",
            SectionKind::Membrane { .. } => "*MEMBRANE SECTION",
            SectionKind::Beam { .. } => "*BEAM SECTION",
            SectionKind::Spring(_) => "*SPRING",
            SectionKind::Gap(_) => "*GAP",
        };
        if let Some(material) = section.kind.material()
            && !material_names.contains(material)
        {
            errors.push(Error::undefined(keyword, "material", material));
        }
        let Some(elements) = model.sets.element_sets.get(&section.elset) else {
            errors.push(Error::undefined(keyword, "element set", &section.elset));
            continue;
        };
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
                            element.kind, section.elset
                        ),
                    ));
                }
                assignment[i as usize] = Some(index as u32);
            }
        }
    }
    // A mesh without steps (a pure mesh file) needs no sections.
    let needs_sections = !analysis.steps.is_empty();
    for (element, section) in model.elements.iter_mut().zip(assignment) {
        element.section = section;
        if section.is_none() && needs_sections {
            errors.push(Error::input(
                "*ELEMENT",
                format!("element {} has no section", element.id),
            ));
        }
    }

    // Constraints.
    let node_known = |model: &crate::Model, region: &NodeRef| match region {
        NodeRef::Node(id) => model.nodes.index(*id).is_some(),
        NodeRef::Set(name) => model.sets.node_sets.contains_key(name),
    };
    let element_known = |model: &crate::Model, region: &ElementRef| match region {
        ElementRef::Element(id) => model.element_index(*id).is_some(),
        ElementRef::Set(name) => model.sets.element_sets.contains_key(name),
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
                    term.node.to_string(),
                ));
            }
        }
    }
    for body in &model.constraints.rigid_bodies {
        match &body.nodes {
            crate::constraints::RigidBodyNodes::NodeSet(name) => {
                if !model.sets.node_sets.contains_key(name) {
                    errors.push(Error::undefined("*RIGID BODY", "node set", name));
                }
            }
            crate::constraints::RigidBodyNodes::ElementSet(name) => {
                if !model.sets.element_sets.contains_key(name) {
                    errors.push(Error::undefined("*RIGID BODY", "element set", name));
                }
            }
        }
        for node in [body.reference_node, body.rotation_node] {
            if model.nodes.index(node).is_none() {
                errors.push(Error::undefined("*RIGID BODY", "node", node.to_string()));
            }
        }
    }
    for tie in &model.constraints.ties {
        for surface in [&tie.slave, &tie.master] {
            if !model.sets.surfaces.contains_key(surface) {
                errors.push(Error::undefined("*TIE", "surface", surface));
            }
        }
    }
    if let Some(cyclic) = &model.constraints.cyclic_symmetry
        && !model.constraints.ties.iter().any(|t| t.name == cyclic.tie)
    {
        errors.push(Error::undefined(
            "*CYCLIC SYMMETRY MODEL",
            "tie",
            &cyclic.tie,
        ));
    }
    for transform in &model.constraints.transforms {
        if !model.sets.node_sets.contains_key(&transform.nset) {
            errors.push(Error::undefined("*TRANSFORM", "node set", &transform.nset));
        }
    }
    if let Some(submodel) = &model.constraints.submodel {
        for set in &submodel.node_sets {
            if !model.sets.node_sets.contains_key(set) {
                errors.push(Error::undefined("*SUBMODEL", "node set", set));
            }
        }
    }

    // Contact.
    let interaction_names: HashSet<&str> = (model.contact.interactions.iter())
        .map(|i| i.name.as_str())
        .collect();
    for pair in &model.contact.pairs {
        if !interaction_names.contains(pair.interaction.as_str()) {
            errors.push(Error::undefined(
                "*CONTACT PAIR",
                "surface interaction",
                &pair.interaction,
            ));
        }
        for surface in [&pair.slave, &pair.master] {
            if !model.sets.surfaces.contains_key(surface) {
                errors.push(Error::undefined("*CONTACT PAIR", "surface", surface));
            }
        }
        if let Some(Surface::Node(_)) = model.sets.surfaces.get(&pair.master) {
            errors.push(Error::input(
                "*CONTACT PAIR",
                format!("master surface {} must be an element surface", pair.master),
            ));
        }
        if pair.kind != ContactKind::NodeToSurface
            && let Some(Surface::Node(_)) = model.sets.surfaces.get(&pair.slave)
        {
            errors.push(Error::input(
                "*CONTACT PAIR",
                format!(
                    "slave surface {} must be an element surface for surface to surface contact",
                    pair.slave
                ),
            ));
        }
    }
    for clearance in &model.contact.clearances {
        for surface in [&clearance.slave, &clearance.master] {
            if !model.sets.surfaces.contains_key(surface) {
                errors.push(Error::undefined("*CLEARANCE", "surface", surface));
            }
        }
    }

    // Initial conditions and design variables.
    for (region, _) in &model.initial.temperatures {
        if !node_known(model, region) {
            errors.push(Error::undefined(
                "*INITIAL CONDITIONS",
                kind_of_node_ref(region),
                region.to_string(),
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
                region.to_string(),
            ));
        }
    }
    if let Some(design) = &model.design_variables
        && !model.sets.node_sets.contains_key(&design.nset)
    {
        errors.push(Error::undefined(
            "*DESIGN VARIABLES",
            "node set",
            &design.nset,
        ));
    }

    // Steps.
    let amplitude_names: BTreeSet<&str> =
        model.amplitudes.iter().map(|a| a.name.as_str()).collect();
    let amplitude = |errors: &mut Vec<Error>, keyword: &str, name: &Option<String>| {
        if let Some(name) = name
            && !amplitude_names.contains(name.as_str())
        {
            errors.push(Error::undefined(keyword, "amplitude", name));
        }
    };
    let stored_modes_before = |steps: &[crate::Step], index: usize| {
        steps[..index]
            .iter()
            .any(|s| matches!(&s.procedure, Procedure::Frequency(f) if f.storage))
    };
    for (index, step) in analysis.steps.iter().enumerate() {
        let name = &step.name;
        for boundary in &step.boundaries {
            if !node_known(model, &boundary.region) {
                errors.push(Error::undefined(
                    "*BOUNDARY",
                    kind_of_node_ref(&boundary.region),
                    format!("{} (step {name})", boundary.region),
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
                    format!("{} (step {name})", load.region),
                ));
            }
            amplitude(&mut errors, "*CLOAD", &load.amplitude);
        }
        for load in &step.loads.distributed {
            if !element_known(model, &load.region) {
                errors.push(Error::undefined(
                    "*DLOAD",
                    kind_of_element_ref(&load.region),
                    format!("{} (step {name})", load.region),
                ));
            }
            amplitude(&mut errors, "*DLOAD", &load.amplitude);
        }
        for flux in &step.loads.concentrated_flux {
            if !node_known(model, &flux.region) {
                errors.push(Error::undefined(
                    "*CFLUX",
                    kind_of_node_ref(&flux.region),
                    format!("{} (step {name})", flux.region),
                ));
            }
            amplitude(&mut errors, "*CFLUX", &flux.amplitude);
        }
        for flux in &step.loads.distributed_flux {
            if !element_known(model, &flux.region) {
                errors.push(Error::undefined(
                    "*DFLUX",
                    kind_of_element_ref(&flux.region),
                    format!("{} (step {name})", flux.region),
                ));
            }
            amplitude(&mut errors, "*DFLUX", &flux.amplitude);
        }
        for film in &step.loads.films {
            if !element_known(model, &film.region) {
                errors.push(Error::undefined(
                    "*FILM",
                    kind_of_element_ref(&film.region),
                    format!("{} (step {name})", film.region),
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
                    format!("{} (step {name})", radiate.region),
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
                            format!("{region} (step {name})"),
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
                            format!("{slave}, {master}"),
                        ));
                    }
                }
                ContactChange::Elements { elset, .. } => {
                    if !model.sets.element_sets.contains_key(elset) {
                        errors.push(Error::undefined("*MODEL CHANGE", "element set", elset));
                    }
                }
                ContactChange::Friction { interaction, .. } => {
                    if !interaction_names.contains(interaction.as_str()) {
                        errors.push(Error::undefined(
                            "*CHANGE FRICTION",
                            "surface interaction",
                            interaction,
                        ));
                    }
                }
            }
        }
        for output in (step.outputs.node_file.iter()).chain(&step.outputs.el_file) {
            if let Some(nset) = &output.nset
                && !model.sets.node_sets.contains_key(nset)
            {
                errors.push(Error::undefined("*NODE FILE", "node set", nset));
            }
        }
        for output in &step.outputs.node_print {
            if let Some(set) = &output.set
                && !model.sets.node_sets.contains_key(set)
            {
                errors.push(Error::undefined("*NODE PRINT", "node set", set));
            }
        }
        for output in &step.outputs.el_print {
            if let Some(set) = &output.set
                && !model.sets.element_sets.contains_key(set)
            {
                errors.push(Error::undefined("*EL PRINT", "element set", set));
            }
        }
        match &step.procedure {
            Procedure::ModalDynamics(_)
            | Procedure::SteadyStateDynamics(_)
            | Procedure::ComplexFrequency(_)
                if !stored_modes_before(&analysis.steps, index) && model.restart_read.is_none() =>
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
                    if let Some(elset) = &response.elset
                        && !model.sets.element_sets.contains_key(elset)
                    {
                        errors.push(Error::undefined("*DESIGN RESPONSE", "element set", elset));
                    }
                }
            }
            _ => {}
        }
    }

    if errors.is_empty() {
        Ok(warnings)
    } else {
        Err(errors)
    }
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
        model.sets.add_elements("EALL", [1]);
        model.sets.add_nodes("FIX", [1, 2, 3]);
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
            elset: "EALL".into(),
            kind: SectionKind::Solid {
                material: "STEEL".into(),
                thickness: None,
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
            material: "RUBBER".into(),
            thickness: None,
        };
        analysis.steps[0].boundaries.push(crate::step::Boundary {
            region: NodeRef::Set("NOWHERE".into()),
            first_dof: 1,
            last_dof: 3,
            value: None,
            amplitude: Some("A1".into()),
            fixed: false,
            submodel_step: None,
            load_case: None,
        });
        let errors = validate(&mut analysis).unwrap_err();
        let texts: Vec<String> = errors.iter().map(|e| e.to_string()).collect();
        assert_eq!(
            texts,
            [
                "*ERROR in *SOLID SECTION: material RUBBER has not been defined",
                "*ERROR in *BOUNDARY: node set NOWHERE (step Step-1) has not been defined",
                "*ERROR in *BOUNDARY: amplitude A1 has not been defined",
            ]
        );
    }
}
