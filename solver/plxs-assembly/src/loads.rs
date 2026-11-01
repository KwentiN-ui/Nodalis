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
            if table.membrane_of[index as usize] != u32::MAX {
                // Native membranes: see `membrane_pressures`.
                continue;
            }
            let index = table.entry_of[index as usize];
            if index == u32::MAX {
                continue;
            }
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
                                table.element_id(model, *index),
                                formulation.kind
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
                                table.element_id(model, *index)
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
                                table.element_id(model, *index)
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
    let pressures = membrane_pressures(model, table, step, time, total, period)?;
    add_membrane_pressures(table, &pressures, None, &mut f);
    membrane_body_loads(model, table, step, time, total, period, &mut f)?;
    Ok((f, warnings))
}

/// Gravity and centrifugal loads on native membranes, added to `f`.
fn membrane_body_loads(
    model: &Model,
    table: &ElementTable,
    step: &Step,
    time: f64,
    total: f64,
    period: f64,
    f: &mut [f64],
) -> Result<()> {
    if table.membranes.is_empty() {
        return Ok(());
    }
    for load in &step.loads.distributed {
        if !matches!(
            load.kind,
            DistributedLoadKind::Gravity { .. } | DistributedLoadKind::Centrifugal { .. }
        ) {
            continue;
        }
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
            let membrane = table.membrane_of[index as usize];
            if membrane == u32::MAX {
                continue;
            }
            let m = &table.membranes[membrane as usize];
            let rho = m.density.ok_or_else(|| {
                Error::input(
                    "*DLOAD",
                    format!("GRAV or CENTRIF on element {id} needs a *DENSITY"),
                )
            })?;
            let mut fe = vec![0.0; m.dofs.len()];
            match &load.kind {
                DistributedLoadKind::Gravity {
                    magnitude,
                    direction,
                } => {
                    let g = direction.map(|d| d * magnitude * factor);
                    m.formulation.body_force(rho, |_| g, &mut fe);
                }
                DistributedLoadKind::Centrifugal {
                    omega_squared,
                    point,
                    axis,
                } => {
                    let len = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
                    let a = axis.map(|c| c / len);
                    let scale = omega_squared * factor;
                    m.formulation.body_force(
                        rho,
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
                _ => {}
            }
            for (&dof, value) in m.dofs.iter().zip(fe) {
                f[dof as usize] += value;
            }
        }
    }
    Ok(())
}

/// The pressures on native membranes at step time `time`: (membrane index, pressure along
/// the element normal). `P` and `P1` push from the bottom side (along the normal), `P2`
/// from the top; CalculiX's edge loads `P3`.. are not supported yet.
pub fn membrane_pressures(
    model: &Model,
    table: &ElementTable,
    step: &Step,
    time: f64,
    total: f64,
    period: f64,
) -> Result<Vec<(u32, f64)>> {
    let mut out = Vec::new();
    if table.membranes.is_empty() {
        return Ok(out);
    }
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
            let membrane = table.membrane_of[index as usize];
            if membrane == u32::MAX {
                continue;
            }
            let p = match &load.kind {
                DistributedLoadKind::Gravity { .. } | DistributedLoadKind::Centrifugal { .. } => {
                    // Body loads: `membrane_body_loads`.
                    continue;
                }
                DistributedLoadKind::Pressure { face: 1, value } => *value,
                DistributedLoadKind::Pressure { face: 2, value } => -*value,
                DistributedLoadKind::Other { label, values } if label == "P" => {
                    values.first().copied().unwrap_or(0.0)
                }
                DistributedLoadKind::Pressure { face, .. } => {
                    return Err(Error::Unsupported(format!(
                        "edge load P{face} on membrane element {id} is not implemented yet"
                    )));
                }
                other => {
                    return Err(Error::Unsupported(format!(
                        "*DLOAD {other:?} on membrane element {id} is not implemented yet"
                    )));
                }
            };
            out.push((membrane, p * factor));
        }
    }
    Ok(out)
}

/// Adds the nodal forces of membrane pressures to `f` (all DOFs), on the reference surface
/// or, with `u`, on the deformed one.
pub fn add_membrane_pressures(
    table: &ElementTable,
    pressures: &[(u32, f64)],
    u: Option<&[f64]>,
    f: &mut [f64],
) {
    for &(index, p) in pressures {
        let m = &table.membranes[index as usize];
        let mut fe = vec![0.0; m.dofs.len()];
        let ue = u.map(|u| m.gather(u));
        m.formulation.pressure(ue.as_deref(), p, &mut fe);
        for (&dof, value) in m.dofs.iter().zip(fe) {
            f[dof as usize] += value;
        }
    }
}
