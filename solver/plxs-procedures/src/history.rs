//! History output (`.dat`): `*NODE PRINT` (U, RF with TOTALS) and `*EL PRINT` (S, E at the
//! integration points) of an increment, as blocks with CalculiX's headings.

use plxs_core::results::HistoryBlock;
use plxs_core::state::IntegrationPointField;
use plxs_core::step::{HistoryOutput, Step, Totals};
use plxs_core::{Model, Results};

use plxs_assembly::ElementTable;

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
    let peeq = material
        .filter(|m| m.per_point > 0)
        .map(|m| IntegrationPointField {
            per_point: 1,
            offsets: m.offsets.clone(),
            values: m.values.chunks(m.per_point).map(|p| p[0]).collect(),
        });
    let energy = crate::fields::internal_energy_density(stresses, strains);
    // The PrePoLix writer puts `*EL PRINT` before `*NODE PRINT`, and CalculiX prints the
    // blocks in request order; the element blocks come first here as well.
    for request in &step.outputs.el_print {
        let Some(set) = request.set.as_deref() else {
            continue;
        };
        let Some(elements) = model.sets.element_sets.get(set) else {
            continue;
        };
        for variable in &request.variables {
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
    let _ = HistoryOutput::clone;
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
