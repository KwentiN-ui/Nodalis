//! The external load vector of a step: concentrated loads, pressures, gravity and
//! centrifugal loads, each scaled by its amplitude at the time asked for.

use plxs_core::mesh::{ElementRef, NodeRef};
use plxs_core::step::{DistributedLoadKind, Step};
use plxs_core::{Error, Model, Result};
use rayon::prelude::*;

use crate::ElementTable;
use crate::dofs::amplitude_factor;

/// The load vector at step time `time` (total time `total`) of a step with period `period`.
/// Returns the vector and warnings (loads on DOFs the nodes do not have).
pub fn load_vector(
    model: &Model,
    table: &ElementTable,
    step: &Step,
    time: f64,
    total: f64,
    period: f64,
) -> Result<(Vec<f64>, Vec<String>)> {
    let mut f = vec![0.0; table.dofs.len()];
    let mut warnings = Vec::new();

    for load in &step.loads.concentrated {
        let factor = amplitude_factor(model, step, load.amplitude.as_deref(), time, total, period);
        let nodes: Vec<u32> = match &load.region {
            NodeRef::Node(id) => vec![*id],
            NodeRef::Set(name) => model.sets.node_sets.get(name).cloned().unwrap_or_default(),
        };
        for node in nodes {
            let Some(index) = model.nodes.index(node) else {
                continue;
            };
            match table.dofs.dof(index, load.dof as usize - 1) {
                Some(dof) if (1..=6).contains(&load.dof) => f[dof as usize] += load.value * factor,
                _ => warnings.push(format!(
                    "*WARNING in *CLOAD: node {node} has no degree of freedom {}; the load is ignored",
                    load.dof
                )),
            }
        }
    }

    // Distributed loads per element, computed in parallel and scattered afterwards.
    let mut per_element: Vec<(u32, DistributedLoadKind, f64)> = Vec::new();
    for load in &step.loads.distributed {
        let factor = amplitude_factor(model, step, load.amplitude.as_deref(), time, total, period);
        let elements: Vec<u32> = match &load.region {
            ElementRef::Element(id) => vec![*id],
            ElementRef::Set(name) => model
                .sets
                .element_sets
                .get(name)
                .cloned()
                .unwrap_or_default(),
        };
        for id in elements {
            let Some(index) = model.element_index(id) else {
                continue;
            };
            per_element.push((index, load.kind.clone(), factor));
        }
    }
    let contributions: Vec<Result<(usize, Vec<f64>)>> = per_element
        .par_iter()
        .map(|(index, kind, factor)| {
            let entry = &table.entries[*index as usize];
            let formulation = table.formulation(entry);
            let coords = table.coordinates(model, entry);
            let mut fe = vec![0.0; formulation.dof_count()];
            match kind {
                DistributedLoadKind::Pressure { face, value } => {
                    if *face as usize == 0 || *face as usize > formulation.face_count() {
                        return Err(Error::input(
                            "*DLOAD",
                            format!(
                                "face {face} does not exist on element {} of type {}",
                                model.elements[*index as usize].id, formulation.kind
                            ),
                        ));
                    }
                    formulation.face_load(&coords, *face, value * factor, &mut fe);
                }
                DistributedLoadKind::Gravity {
                    magnitude,
                    direction,
                } => {
                    let rho = entry.density.ok_or_else(|| {
                        Error::input(
                            "*DLOAD",
                            format!(
                                "GRAV on element {} needs a *DENSITY",
                                model.elements[*index as usize].id
                            ),
                        )
                    })?;
                    let g = direction.map(|d| d * magnitude * rho * factor);
                    formulation.body_force(&coords, |_| g, &mut fe);
                }
                DistributedLoadKind::Centrifugal {
                    omega_squared,
                    point,
                    axis,
                } => {
                    let rho = entry.density.ok_or_else(|| {
                        Error::input(
                            "*DLOAD",
                            format!(
                                "CENTRIF on element {} needs a *DENSITY",
                                model.elements[*index as usize].id
                            ),
                        )
                    })?;
                    let len = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
                    let a = axis.map(|c| c / len);
                    let scale = rho * omega_squared * factor;
                    formulation.body_force(
                        &coords,
                        |x| {
                            let r = [x[0] - point[0], x[1] - point[1], x[2] - point[2]];
                            let along = r[0] * a[0] + r[1] * a[1] + r[2] * a[2];
                            [
                                scale * (r[0] - along * a[0]),
                                scale * (r[1] - along * a[1]),
                                scale * (r[2] - along * a[2]),
                            ]
                        },
                        &mut fe,
                    );
                }
                DistributedLoadKind::Other { label, .. } => {
                    return Err(Error::Unsupported(format!(
                        "*DLOAD label {label} is not implemented yet"
                    )));
                }
            }
            Ok((*index as usize, fe))
        })
        .collect();
    for contribution in contributions {
        let (index, fe) = contribution?;
        let entry = &table.entries[index];
        for (&dof, value) in entry.dofs.iter().zip(fe) {
            f[dof as usize] += value;
        }
    }
    Ok((f, warnings))
}
