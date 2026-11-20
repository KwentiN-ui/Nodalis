//! The results of an analysis whose Particle Flow step is followed by FE steps: the run of
//! the particle solver and the run of the FE solver after it in one results file. The FE
//! results keep the numbers of their nodes and elements, those of the model; the particles
//! and the walls of the flow are numbered after them. Each step shows its own parts
//! ([`crate::steps::StepRecord::elements`]): the flow its particles and walls, the FE steps the
//! parts they computed. The FE steps follow the flow in time.

use std::collections::BTreeSet;
use std::path::Path;

use plxs_core::section::SectionKind;
use plxs_output::spool::Spool;

use crate::SolverResults;

/// The material of a section of the solver's model.
fn material_of(kind: &mut SectionKind) -> Option<&mut plxs_core::MaterialId> {
    match kind {
        SectionKind::Solid { material, .. }
        | SectionKind::Shell { material, .. }
        | SectionKind::Membrane { material, .. }
        | SectionKind::Beam { material, .. } => Some(material),
        _ => None,
    }
}

/// The results of the FE steps `fe` (their increments in `fe_spool`, or in memory without
/// one) after the flow `flow` (its increments in `flow_spool`), in one: the increments go to
/// a new spool at `spool_path`, which [`SolverResults::write_spooled`] writes out.
pub fn chain(
    mut flow: SolverResults,
    flow_spool: &Spool,
    mut fe: SolverResults,
    fe_spool: Option<&Spool>,
    spool_path: &Path,
) -> std::io::Result<(SolverResults, Spool)> {
    let node_offset = fe.model.nodes.iter().map(|(id, _)| id).max().unwrap_or(0);
    let element_offset = fe.model.elements.iter().map(|e| e.id).max().unwrap_or(0);
    // The steps of the flow come first; the FE steps follow them in the numbers of the
    // solver's steps and in time.
    let flow_steps = (flow
        .steps
        .iter()
        .flat_map(|s| s.solver_steps.iter().copied()))
    .chain(flow.results.increments.iter().map(|i| i.step))
    .chain(flow_spool.increments().iter().map(|s| s.skeleton.step))
    .max()
    .unwrap_or(1);
    let flow_end = (flow.convergence.iter().map(|c| c.total_time))
        .chain(
            flow_spool
                .increments()
                .iter()
                .map(|s| s.skeleton.total_time),
        )
        .fold(0.0_f64, f64::max);

    // The parts the steps show: the results group elements into parts by material.
    let fe_parts: BTreeSet<String> = fe.model.materials.iter().map(|m| m.name.clone()).collect();
    // A material of the flow named like one of the FE steps is renamed.
    for material in &mut flow.model.materials {
        if fe_parts.contains(&material.name) {
            let old = material.name.clone();
            let mut name = format!("{old} (flow)");
            let mut k = 2;
            while fe_parts.contains(&name) {
                name = format!("{old} (flow {k})");
                k += 1;
            }
            material.name = name;
        }
    }

    let model = &mut fe.model;
    let sections = model.sections.len() as u32;
    // The flow's materials follow the FE steps' ones.
    let material_offset = model.materials.len() as u32;
    for mut section in flow.model.sections {
        if let Some(material) = material_of(&mut section.kind) {
            material.0 += material_offset;
        }
        model.sections.push(section);
    }
    model.materials.append(&mut flow.model.materials);
    for (id, x) in flow.model.nodes.iter() {
        model.nodes.insert(id + node_offset, x);
    }
    for mut element in flow.model.elements {
        element.id += element_offset;
        for node in &mut element.nodes {
            *node += node_offset;
        }
        element.section = element.section.map(|s| s + sections);
        model.elements.push(element);
    }
    model.rebuild_indices();

    let mut spool = Spool::create(spool_path)?;
    let shift_flow = |increment: &mut plxs_core::results::ResultsIncrement| {
        for field in &mut increment.fields {
            for node in &mut field.nodes {
                *node += node_offset;
            }
        }
    };
    for mut increment in std::mem::take(&mut flow.results.increments) {
        shift_flow(&mut increment);
        spool.push(increment)?;
    }
    for index in 0..flow_spool.len() {
        let mut increment = flow_spool.read(index)?;
        shift_flow(&mut increment);
        spool.push(increment)?;
    }
    let later = |step: &mut u32, time: &mut f64| {
        *step += flow_steps;
        *time += flow_end;
    };
    for mut increment in std::mem::take(&mut fe.results.increments) {
        later(&mut increment.step, &mut increment.total_time);
        spool.push(increment)?;
    }
    if let Some(fe_spool) = fe_spool {
        for spooled in fe_spool.increments() {
            let mut skeleton = spooled.skeleton.clone();
            later(&mut skeleton.step, &mut skeleton.total_time);
            let raw = (spooled.arrays.iter())
                .map(|&a| fe_spool.raw(a))
                .collect::<std::io::Result<Vec<_>>>()?;
            spool.push_raw(skeleton, &raw)?;
        }
    }

    let results = &mut fe.results;
    for block in &mut results.history {
        later(&mut block.step, &mut block.time);
    }
    let mut history = std::mem::take(&mut flow.results.history);
    history.append(&mut results.history);
    results.history = history;
    for set in &mut results.modes {
        set.step += flow_steps;
    }
    for record in &mut fe.convergence {
        later(&mut record.step, &mut record.total_time);
    }
    flow.convergence.append(&mut fe.convergence);
    fe.convergence = flow.convergence;
    for record in &mut flow.steps {
        record.elements = Some([element_offset + 1, u32::MAX]);
    }
    for record in &mut fe.steps {
        for step in &mut record.solver_steps {
            *step += flow_steps;
        }
        record.elements = Some([1, element_offset]);
    }
    flow.steps.append(&mut fe.steps);
    fe.steps = flow.steps;
    fe.unconverged_step = (fe.unconverged_step.map(|s| s + flow_steps)).or(flow.unconverged_step);
    Ok((fe, spool))
}

#[cfg(test)]
mod tests {
    use plxs_core::element::{Element, ElementType};
    use plxs_core::results::{IncrementKind, ResultField, ResultsIncrement};
    use plxs_core::section::Section;

    use super::*;
    use crate::steps::{IncrementRecord, StepRecord};

    /// A tetrahedron of material `material`, numbered from `first`.
    fn results(material: &str, first: u32, steps: &[(&str, u32)]) -> SolverResults {
        let mut model = plxs_core::Model::default();
        for k in 0..4 {
            model.nodes.insert(first + k, [f64::from(k), 0.0, 0.0]);
        }
        model.materials.push(plxs_core::material::Material {
            name: material.into(),
            ..Default::default()
        });
        model.sections.push(Section {
            elset: material.into(),
            kind: SectionKind::Solid {
                material: plxs_core::MaterialId(0),
                thickness: None,
                orientation: None,
            },
        });
        model.elements.push(Element {
            id: first,
            kind: ElementType::C3D4,
            nodes: (first..first + 4).collect(),
            section: Some(0),
        });
        model.rebuild_indices();
        let mut out = SolverResults::new(&model, Default::default(), "test");
        out.steps = (steps.iter())
            .map(|&(name, step)| {
                let mut record =
                    StepRecord::new(name, &plx_model::Step::new_static("x").kind, vec![step]);
                record.time_period = Some(1.0);
                record
            })
            .collect();
        out
    }

    fn increment(step: u32, time: f64, nodes: Vec<u32>) -> ResultsIncrement {
        ResultsIncrement {
            step,
            increment: 1,
            kind: IncrementKind::Time,
            value: time,
            total_time: time,
            nodal_diameter: None,
            fields: vec![ResultField {
                name: "DISP".into(),
                components: vec!["D1".into()],
                values: vec![vec![1.0; nodes.len()]],
                nodes,
            }],
        }
    }

    #[test]
    fn the_fe_steps_follow_the_flow_with_the_numbers_of_the_model() {
        let dir = std::env::temp_dir().join(format!("plx-chained-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        // The flow and the FE results both number from 1; the FE steps start at 0.
        let mut flow = results("Steel", 1, &[("Flow", 1)]);
        flow.convergence.push(IncrementRecord {
            step: 1,
            increment: 1,
            attempts: 1,
            iterations: 0,
            step_time: 2.0,
            total_time: 2.0,
            explicit: false,
        });
        let mut flow_spool = Spool::create(&dir.join("flow.spool")).unwrap();
        flow_spool
            .push(increment(1, 2.0, vec![1, 2, 3, 4]))
            .unwrap();
        let mut fe = results("Steel", 1, &[("Load", 1)]);
        fe.results
            .increments
            .push(increment(1, 1.0, vec![1, 2, 3, 4]));

        let (all, spool) = chain(flow, &flow_spool, fe, None, &dir.join("all.spool")).unwrap();
        // The FE nodes keep their numbers; the flow's come after them.
        let ids: Vec<u32> = all.model.nodes.iter().map(|(id, _)| id).collect();
        assert_eq!(ids, [1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(all.model.elements[1].id, 2);
        assert_eq!(all.model.elements[1].nodes, [5, 6, 7, 8]);
        assert_eq!(all.model.elements[1].section, Some(1));
        // A material of the same name is the flow's own part.
        assert_eq!(all.model.materials[1].name, "Steel (flow)");
        assert_eq!(
            all.model.sections[1].kind.material(),
            Some(plxs_core::MaterialId(1))
        );
        // The flow is step 1, the FE step 2, after the flow in time.
        assert_eq!(spool.len(), 2);
        let first = spool.read(0).unwrap();
        assert_eq!(
            (first.step, first.fields[0].nodes.clone()),
            (1, vec![5, 6, 7, 8])
        );
        let second = spool.read(1).unwrap();
        assert_eq!((second.step, second.total_time), (2, 3.0));
        assert_eq!(second.fields[0].nodes, [1, 2, 3, 4]);
        let names: Vec<_> = all
            .steps
            .iter()
            .map(|s| (s.name.as_str(), s.solver_steps.clone()))
            .collect();
        assert_eq!(names, [("Flow", vec![1]), ("Load", vec![2])]);
        // Each step shows its own parts.
        assert_eq!(all.steps[0].elements, Some([2, u32::MAX]));
        assert_eq!(all.steps[1].elements, Some([1, 1]));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
