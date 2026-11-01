//! History output (`.dat`): `*NODE PRINT` (U, RF with TOTALS) and `*EL PRINT` (S, E at the
//! integration points) of an increment, as blocks with CalculiX's headings.

use plxs_core::results::HistoryBlock;
use plxs_core::state::IntegrationPointField;
use plxs_core::step::{HistoryOutput, Step, Totals};
use plxs_core::{Model, Results};

use plxs_assembly::ElementTable;

/// Appends one of the tables an eigenvalue step prints (see `plxs_output::EigenTable`):
/// `quantity` is the table's name, rows are keyed by mode (0 for a total row).
pub fn write_table(
    results: &mut Results,
    step_number: u32,
    quantity: &str,
    rows: Vec<(u32, Vec<f64>)>,
) {
    results.history.push(HistoryBlock {
        step: step_number,
        increment: 0,
        quantity: quantity.to_string(),
        set: String::new(),
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
    let energy = crate::fields::internal_energy_density(stresses, strains);
    // The Nodalis writer puts `*EL PRINT` before `*NODE PRINT`, and CalculiX prints the
    // blocks in request order; the element blocks come first here as well.
    for request in &step.outputs.el_print {
        let Some(set) = request.set.as_deref() else {
            continue;
        };
        let Some(elements) = model.sets.element_sets.get(set) else {
            continue;
        };
        for variable in &request.variables {
            if variable.trim().eq_ignore_ascii_case("EVOL") {
                write_volumes(
                    results,
                    model,
                    table,
                    request,
                    (step_number, increment, time),
                    set,
                    elements,
                );
                continue;
            }
            let (quantity, field, reorder): (&str, &IntegrationPointField, bool) =
                match variable.trim().to_ascii_uppercase().as_str() {
                    "S" => (
                        "stresses (elem, integ.pnt.,sxx,syy,szz,sxy,sxz,syz)",
                        stresses,
                        true,
                    ),
                    "E" => (
                        "strains (elem, integ.pnt.,exx,eyy,ezz,exy,exz,eyz)",
                        strains,
                        true,
                    ),
                    "PEEQ" => match &peeq {
                        Some(p) => ("equivalent plastic strain (elem, integ.pnt.,pe)", p, false),
                        None => continue,
                    },
                    "ENER" => (
                        "internal energy density (elem, integ.pnt.,energy)",
                        &energy,
                        false,
                    ),
                    _ => continue,
                };
            let rows = integration_point_rows(model, table, elements, field, reorder);
            results.history.push(HistoryBlock {
                step: step_number,
                increment,
                quantity: quantity.to_string(),
                set: set.to_string(),
                time,
                rows,
            });
        }
    }
    let velocities = velocities.map(|v| table.reported(model, v, &vec![0.0; v.len()]).0);
    let (u, forces) = &table.reported(model, u, forces);
    for request in &step.outputs.node_print {
        let Some(set) = request.set.as_deref() else {
            continue;
        };
        let Some(nodes) = model.sets.node_sets.get(set) else {
            continue;
        };
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
        for variable in &request.variables {
            let block = |quantity: &str, rows: Vec<(u32, Vec<f64>)>| HistoryBlock {
                step: step_number,
                increment,
                quantity: quantity.to_string(),
                set: set.to_string(),
                time,
                rows,
            };
            match variable.trim().to_ascii_uppercase().as_str() {
                "U" => {
                    if request.totals != Totals::Only {
                        results
                            .history
                            .push(block("displacements (vx,vy,vz)", rows_of(u)));
                    }
                }
                "V" => {
                    if let Some(v) = &velocities
                        && request.totals != Totals::Only
                    {
                        results
                            .history
                            .push(block("velocities (vx,vy,vz)", rows_of(v)));
                    }
                }
                "RF" => {
                    let rows = rows_of(forces);
                    let sum = totals(&rows);
                    if request.totals != Totals::Only {
                        results.history.push(block("forces (fx,fy,fz)", rows));
                    }
                    if request.totals != Totals::No {
                        results
                            .history
                            .push(block("total force (fx,fy,fz)", vec![(0, sum)]));
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
    set: &str,
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
    let block = |quantity: &str, rows| HistoryBlock {
        step,
        increment,
        quantity: quantity.to_string(),
        set: set.to_string(),
        time,
        rows,
    };
    if request.totals != Totals::Only {
        results
            .history
            .push(block("volume (element, volume)", rows));
    }
    if request.totals != Totals::No {
        results
            .history
            .push(block("total volume", vec![(0, vec![total])]));
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
    let block = |quantity: &str, set: &str, rows: Vec<(u32, Vec<f64>)>| HistoryBlock {
        step: step_number,
        increment,
        quantity: quantity.to_string(),
        set: set.to_string(),
        time,
        rows,
    };
    for request in &step.outputs.node_print {
        let Some(set) = request.set.as_deref() else {
            continue;
        };
        let Some(nodes) = model.sets.node_sets.get(set) else {
            continue;
        };
        let rows_of = |values: &[f64]| -> Vec<(u32, Vec<f64>)> {
            nodes
                .iter()
                .filter_map(|&id| Some((id, vec![values[model.nodes.index(id)? as usize]])))
                .collect()
        };
        for variable in &request.variables {
            match variable.trim().to_ascii_uppercase().as_str() {
                "NT" if request.totals != Totals::Only => {
                    results
                        .history
                        .push(block("temperatures", set, rows_of(temperatures)));
                }
                "RFL" => {
                    let rows = rows_of(heat_generation);
                    let sum: f64 = rows.iter().map(|r| r.1[0]).sum();
                    if request.totals != Totals::Only {
                        results.history.push(block("heat generation", set, rows));
                    }
                    if request.totals != Totals::No {
                        results.history.push(block(
                            "total heat generation",
                            set,
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
        let Some(set) = request.set.as_deref() else {
            continue;
        };
        let Some(elements) = model.sets.element_sets.get(set) else {
            continue;
        };
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
        results
            .history
            .push(block("heat flux (elem, integ.pnt.,qx,qy,qz)", set, rows));
    }
}
