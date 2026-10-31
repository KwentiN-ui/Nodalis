//! Concentrated loads at nodes with a local system (`*TRANSFORM`): CalculiX applies their
//! components in the local directions (manual, `*TRANSFORM`: "a local axis system ... to be
//! used for defining SPC's, MPC's and nodal forces").

use plxs_constraints::ModelConstraints;
use plxs_core::mesh::NodeRef;
use plxs_core::{Model, Step};

use crate::DofMap;
use crate::dofs::amplitude_factor;

/// Turns the concentrated loads that [`crate::load_vector`] added in global directions at
/// transformed nodes into their local directions.
#[allow(clippy::too_many_arguments)]
pub fn rotate_concentrated_loads(
    model: &Model,
    constraints: &ModelConstraints,
    dofs: &DofMap,
    step: &Step,
    time: f64,
    total: f64,
    period: f64,
    f: &mut [f64],
) {
    if constraints.rotations.is_empty() {
        return;
    }
    for load in &step.loads.concentrated {
        if !(1..=3).contains(&load.dof) {
            continue;
        }
        let factor = amplitude_factor(model, step, load.amplitude.as_deref(), time, total, period);
        let ids: Vec<u32> = match &load.region {
            NodeRef::Node(id) => vec![*id],
            NodeRef::Set(name) => model.sets.node_sets.get(name).cloned().unwrap_or_default(),
        };
        let d = load.dof as usize - 1;
        for id in ids {
            let Some(index) = model.nodes.index(id) else {
                continue;
            };
            let Some(r) = constraints.rotation(index) else {
                continue;
            };
            let value = load.value * factor;
            for (k, row) in r.iter().enumerate() {
                let delta = if k == d { 1.0 } else { 0.0 };
                if let Some(dof) = dofs.dof(index, k) {
                    f[dof as usize] += value * (row[d] - delta);
                }
            }
        }
    }
}
