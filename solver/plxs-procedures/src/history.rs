//! History output: the node prints (U, V, RF with totals; NT, RFL) and element prints (S,
//! E, PEEQ, ENER, HFL at the integration points, EVOL) of an increment, as history blocks.

use plxs_core::results::{HistoryBlock, HistoryQuantity, HistorySource};
use plxs_core::state::IntegrationPointField;
use plxs_core::step::{HistoryOutput, Step, Totals};
use plxs_core::{Model, Results};

use plxs_assembly::ElementTable;

/// Appends one of the tables an eigenvalue step has: rows are keyed by mode (0 for a total
/// row).
pub fn write_table(
    results: &mut Results,
    step_number: u32,
    quantity: HistoryQuantity,
    rows: Vec<(u32, Vec<f64>)>,
) {
    results.history.push(HistoryBlock {
        step: step_number,
        increment: 0,
        quantity,
        source: HistorySource::Step,
        time: 0.0,
        rows,
    });
}

/// Appends the history blocks of an increment.
#[allow(clippy::too_many_arguments)]
pub fn write_history(
    results: &mut Results,
    model: &Model,
    table: &ElementTable,
    step: &Step,
    step_number: u32,
    increment: u32,
    // The total time, which CalculiX prints in the block headers.
    time: f64,
    u: &[f64],
    forces: &[f64],
    stresses: &IntegrationPointField,
    strains: &IntegrationPointField,
    material: Option<&IntegrationPointField>,
) {
    write_history_with_velocities(
        results,
        model,
        table,
        step,
        (step_number, increment, time),
        (u, None),
        forces,
        stresses,
        strains,
        material,
    );
}

/// [`write_history`] of a dynamic increment, which also prints the velocities (`V`).
#[allow(clippy::too_many_arguments)]
pub fn write_history_with_velocities(
    results: &mut Results,
    model: &Model,
    table: &ElementTable,
    step: &Step,
    (step_number, increment, time): (u32, u32, f64),
    (u, velocities): (&[f64], Option<&[f64]>),
    forces: &[f64],
    stresses: &IntegrationPointField,
    strains: &IntegrationPointField,
    material: Option<&IntegrationPointField>,
) {
    let peeq = material
        .filter(|m| m.per_point > 0)
        .map(|m| IntegrationPointField {
            per_point: 1,
            offsets: m.offsets.clone(),
            values: m.values.chunks(m.per_point).map(|p| p[0]).collect(),
        });
    // The energy density only when an element print asks for it (it costs a pass over
    // every integration point, which a dynamic step takes per increment).
    let wants_energy = step.outputs.el_print.iter().any(|request| {
        request.set.is_some()
            && request
                .variables
                .iter()
                .any(|v| v.trim().eq_ignore_ascii_case("ENER"))
    });
    let energy = wants_energy.then(|| crate::fields::internal_energy_density(stresses, strains));
    // The Nodalis writer puts `*EL PRINT` before `*NODE PRINT`, and CalculiX prints the
    // blocks in request order; the element blocks come first here as well.
    for request in &step.outputs.el_print {
        let Some(plxs_core::step::HistorySet::Elements(handle)) = request.set else {
            continue;
        };
        let Some(found) = model.sets.element_sets.get(handle.index()) else {
            continue;
        };
        let (set, elements) = (HistorySource::Elements(handle), &found.elements);
        for variable in &request.variables {
            if variable.trim().eq_ignore_ascii_case("EVOL") {
                write_volumes(
                    results,
                    model,
                    table,
                    request,
                    (step_number, increment, time),
                    set.clone(),
                    elements,
                );
                continue;
            }
            let (quantity, field, reorder): (HistoryQuantity, &IntegrationPointField, bool) =
                match variable.trim().to_ascii_uppercase().as_str() {
                    "S" => (HistoryQuantity::Stresses, stresses, true),
                    "E" => (HistoryQuantity::Strains, strains, true),
                    "PEEQ" => match &peeq {
                        Some(p) => (HistoryQuantity::EquivalentPlasticStrain, p, false),
                        None => continue,
                    },
                    "ENER" => match &energy {
                        Some(e) => (HistoryQuantity::InternalEnergyDensity, e, false),
                        None => continue,
                    },
                    _ => continue,
                };
            let rows = integration_point_rows(model, table, elements, field, reorder);
            results.history.push(HistoryBlock {
                step: step_number,
                increment,
                quantity,
                source: set.clone(),
                time,
                rows,
            });
        }
    }
    // The reported values only for a `*NODE PRINT`: the forces of plane and axisymmetric
    // elements cost an element stiffness each.
    if step.outputs.node_print.iter().all(|r| r.set.is_none()) {
        return;
    }
    let velocities = velocities.map(|v| table.reported_displacements(v));
    let (u, forces) = &table.reported(model, u, forces);
    for request in &step.outputs.node_print {
        let Some(plxs_core::step::HistorySet::Nodes(handle)) = request.set else {
            continue;
        };
        let Some(found) = model.sets.node_sets.get(handle.index()) else {
            continue;
        };
        let nodes = &found.nodes;
        let rows_of = |values: &[f64]| -> Vec<(u32, Vec<f64>)> {
            nodes
                .iter()
                .filter_map(|&id| {
                    let i = model.nodes.index(id)? as usize;
                    Some((id, values[i * 3..i * 3 + 3].to_vec()))
                })
                .collect()
        };
        let totals = |rows: &[(u32, Vec<f64>)]| -> Vec<f64> {
            (0..3).map(|k| rows.iter().map(|r| r.1[k]).sum()).collect()
        };
        // The rotation nodes of reference points turn: their values are rotations and
        // moments.
        let [displacements, velocities_of, forces_of, total_force] = if request.rotations {
            use HistoryQuantity as Q;
            [
                Q::Rotations,
                Q::AngularVelocities,
                Q::Moments,
                Q::TotalMoment,
            ]
        } else {
            use HistoryQuantity as Q;
            [Q::Displacements, Q::Velocities, Q::Forces, Q::TotalForce]
        };
        for variable in &request.variables {
            let block = |quantity, rows: Vec<(u32, Vec<f64>)>| HistoryBlock {
                step: step_number,
                increment,
                quantity,
                source: HistorySource::Nodes(handle),
                time,
                rows,
            };
            match variable.trim().to_ascii_uppercase().as_str() {
                "U" => {
                    if request.totals != Totals::Only {
                        results.history.push(block(displacements, rows_of(u)));
                    }
                }
                "V" => {
                    if let Some(v) = &velocities
                        && request.totals != Totals::Only
                    {
                        results.history.push(block(velocities_of, rows_of(v)));
                    }
                }
                "RF" => {
                    let rows = rows_of(forces);
                    let sum = totals(&rows);
                    if request.totals != Totals::Only {
                        results.history.push(block(forces_of, rows));
                    }
                    if request.totals != Totals::No {
                        results.history.push(block(total_force, vec![(0, sum)]));
                    }
                }
                _ => {}
            }
        }
    }
}

/// `EVOL` of an `*EL PRINT`: the volume of each element of the set (CalculiX's
/// `volume (element, volume)`) and, with `TOTALS`, their sum.
fn write_volumes(
    results: &mut Results,
    model: &Model,
    table: &ElementTable,
    request: &HistoryOutput,
    (step, increment, time): (u32, u32, f64),
    source: HistorySource,
    elements: &[u32],
) {
    let rows: Vec<(u32, Vec<f64>)> = elements
        .iter()
        .filter_map(|&id| {
            let entry = table.entries.get(model.element_index(id)? as usize)?;
            let coords = table.coordinates(model, entry);
            Some((id, vec![table.formulation(entry).volume(&coords)]))
        })
        .collect();
    let total: f64 = rows.iter().map(|r| r.1[0]).sum();
    let block = |quantity, rows| HistoryBlock {
        step,
        increment,
        quantity,
        source: source.clone(),
        time,
        rows,
    };
    if request.totals != Totals::Only {
        results.history.push(block(HistoryQuantity::Volume, rows));
    }
    if request.totals != Totals::No {
        (results.history).push(block(HistoryQuantity::TotalVolume, vec![(0, vec![total])]));
    }
}

/// Rows `(element, [point, values...])` for the elements of a set; CalculiX's `.dat` lists
/// the shear components as xy, xz, yz.
fn integration_point_rows(
    model: &Model,
    table: &ElementTable,
    elements: &[u32],
    field: &IntegrationPointField,
    reorder_shear: bool,
) -> Vec<(u32, Vec<f64>)> {
    let mut rows = Vec::new();
    for &id in elements {
        let Some(index) = model.element_index(id) else {
            continue;
        };
        let start = table.layout.offsets[index as usize] as usize;
        let end = table.layout.offsets[index as usize + 1] as usize;
        for (p, point) in (start..end).enumerate() {
            let n = field.per_point;
            let v = &field.values[point * n..point * n + n];
            let mut values = vec![(p + 1) as f64];
            if reorder_shear && n == 6 {
                values.extend_from_slice(&[v[0], v[1], v[2], v[3], v[5], v[4]]);
            } else {
                values.extend_from_slice(v);
            }
            rows.push((id, values));
        }
    }
    rows
}

/// Whether an `*EL PRINT` of the step asks for the heat flux.
pub fn wants_heat_flux(step: &Step) -> bool {
    step.outputs.el_print.iter().any(|r| {
        r.variables
            .iter()
            .any(|v| v.trim().eq_ignore_ascii_case("HFL"))
    })
}

/// Appends the thermal history blocks of an increment: `NT` (temperatures) and `RFL` (heat
/// generation, with totals) of `*NODE PRINT`, `HFL` (heat flux at the integration points)
/// of `*EL PRINT`.
#[allow(clippy::too_many_arguments)]
pub fn write_thermal_history(
    results: &mut Results,
    model: &Model,
    table: &ElementTable,
    step: &Step,
    step_number: u32,
    increment: u32,
    time: f64,
    temperatures: &[f64],
    heat_generation: &[f64],
    flux: Option<&IntegrationPointField>,
) {
    let block = |quantity, source, rows: Vec<(u32, Vec<f64>)>| HistoryBlock {
        step: step_number,
        increment,
        quantity,
        source,
        time,
        rows,
    };
    for request in &step.outputs.node_print {
        let Some(plxs_core::step::HistorySet::Nodes(handle)) = request.set else {
            continue;
        };
        let Some(found) = model.sets.node_sets.get(handle.index()) else {
            continue;
        };
        let (set, nodes) = (HistorySource::Nodes(handle), &found.nodes);
        let rows_of = |values: &[f64]| -> Vec<(u32, Vec<f64>)> {
            nodes
                .iter()
                .filter_map(|&id| Some((id, vec![values[model.nodes.index(id)? as usize]])))
                .collect()
        };
        for variable in &request.variables {
            match variable.trim().to_ascii_uppercase().as_str() {
                "NT" if request.totals != Totals::Only => {
                    let rows = rows_of(temperatures);
                    (results.history).push(block(HistoryQuantity::Temperatures, set.clone(), rows));
                }
                "RFL" => {
                    let rows = rows_of(heat_generation);
                    let sum: f64 = rows.iter().map(|r| r.1[0]).sum();
                    if request.totals != Totals::Only {
                        let quantity = HistoryQuantity::HeatGeneration;
                        results.history.push(block(quantity, set.clone(), rows));
                    }
                    if request.totals != Totals::No {
                        results.history.push(block(
                            HistoryQuantity::TotalHeatGeneration,
                            set.clone(),
                            vec![(0, vec![sum])],
                        ));
                    }
                }
                _ => {}
            }
        }
    }
    let Some(flux) = flux else {
        return;
    };
    for request in &step.outputs.el_print {
        let Some(plxs_core::step::HistorySet::Elements(handle)) = request.set else {
            continue;
        };
        let Some(found) = model.sets.element_sets.get(handle.index()) else {
            continue;
        };
        let elements = &found.elements;
        if !request
            .variables
            .iter()
            .any(|v| v.trim().eq_ignore_ascii_case("HFL"))
        {
            continue;
        }
        let rows = integration_point_rows(model, table, elements, flux, false)
            .into_iter()
            .map(|(id, v)| (id, v[..4].to_vec()))
            .collect();
        let source = HistorySource::Elements(handle);
        (results.history).push(block(HistoryQuantity::HeatFlux, source, rows));
    }
}
