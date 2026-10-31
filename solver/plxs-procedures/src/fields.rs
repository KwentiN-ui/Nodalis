//! Nodal result fields of an increment: displacements, reaction forces, stresses and
//! strains extrapolated from the integration points and averaged at the nodes, and the
//! stress error estimator; appended to the results in the order CalculiX writes them.

use plxs_assembly::ElementTable;
use plxs_core::results::{IncrementKind, ResultField, ResultsIncrement};
use plxs_core::state::IntegrationPointField;
use plxs_core::{Model, ProgressEvent, Result, Step};

use crate::ProcedureContext;

/// The variables a field request asks for, upper case, from all `*NODE FILE` / `*EL FILE`
/// lines of the step.
fn requested(step: &Step) -> Vec<String> {
    step.outputs
        .node_file
        .iter()
        .chain(&step.outputs.el_file)
        .chain(&step.outputs.contact_file)
        .flat_map(|o| o.variables.iter().map(|v| v.trim().to_ascii_uppercase()))
        .collect()
}

/// Node ids in ascending order with their dense indices.
fn node_order(model: &Model) -> Vec<(u32, u32)> {
    let mut order: Vec<(u32, u32)> = model
        .nodes
        .ids
        .iter()
        .enumerate()
        .map(|(i, &id)| (id, i as u32))
        .collect();
    order.sort_unstable();
    order
}

fn vector_field(
    name: &str,
    components: [&str; 3],
    order: &[(u32, u32)],
    values: &[f64],
) -> ResultField {
    ResultField {
        name: name.to_string(),
        components: components.iter().map(|c| c.to_string()).collect(),
        nodes: order.iter().map(|&(id, _)| id).collect(),
        values: (0..3)
            .map(|k| {
                order
                    .iter()
                    .map(|&(_, i)| values[i as usize * 3 + k])
                    .collect()
            })
            .collect(),
    }
}

/// Extrapolates integration-point values (6 per point) of every element to its nodes and
/// averages them at the nodes over the elements sharing them. Returns the averages per
/// dense node (6 each) and, per element, the extrapolated nodal values for the error
/// estimator.
pub fn nodal_average(
    table: &ElementTable,
    model: &Model,
    field: &IntegrationPointField,
) -> (Vec<[f64; 6]>, Vec<Vec<[f64; 6]>>) {
    nodal_average_with(table, model, field, &[])
}

/// [`nodal_average`] with further elements of constant value (native membranes): dense
/// nodes and value.
pub fn nodal_average_with(
    table: &ElementTable,
    model: &Model,
    field: &IntegrationPointField,
    constant: &[([u32; 3], [f64; 6])],
) -> (Vec<[f64; 6]>, Vec<Vec<[f64; 6]>>) {
    let nodes = model.nodes.len();
    let mut sum = vec![[0.0; 6]; nodes];
    let mut count = vec![0u32; nodes];
    let mut per_element = Vec::with_capacity(table.entries.len());
    for (index, entry) in table.entries.iter().enumerate() {
        let formulation = table.formulation(entry);
        let ips = formulation.integration_points();
        let start = table.layout.offsets[index] as usize;
        let e = formulation.extrapolation();
        let mut nodal = vec![[0.0; 6]; entry.nodes.len()];
        for (a, node) in nodal.iter_mut().enumerate() {
            for g in 0..ips {
                let w = e[a * ips + g];
                let at = (start + g) * 6;
                for (k, v) in node.iter_mut().enumerate() {
                    *v += w * field.values[at + k];
                }
            }
        }
        for (a, &n) in entry.nodes.iter().enumerate() {
            for k in 0..6 {
                sum[n as usize][k] += nodal[a][k];
            }
            count[n as usize] += 1;
        }
        per_element.push(nodal);
    }
    for (nodes, value) in constant {
        for &n in nodes {
            for k in 0..6 {
                sum[n as usize][k] += value[k];
            }
            count[n as usize] += 1;
        }
    }
    for (s, &c) in sum.iter_mut().zip(&count) {
        if c > 0 {
            for v in s.iter_mut() {
                *v /= f64::from(c);
            }
        }
    }
    (sum, per_element)
}

fn tensor_field(
    name: &str,
    components: [&str; 6],
    order: &[(u32, u32)],
    values: &[[f64; 6]],
) -> ResultField {
    ResultField {
        name: name.to_string(),
        components: components.iter().map(|c| c.to_string()).collect(),
        nodes: order.iter().map(|&(id, _)| id).collect(),
        values: (0..6)
            .map(|k| order.iter().map(|&(_, i)| values[i as usize][k]).collect())
            .collect(),
    }
}

pub fn von_mises(s: &[f64; 6]) -> f64 {
    let (sx, sy, sz, sxy, syz, szx) = (s[0], s[1], s[2], s[3], s[4], s[5]);
    (0.5 * ((sx - sy).powi(2) + (sy - sz).powi(2) + (sz - sx).powi(2))
        + 3.0 * (sxy * sxy + syz * syz + szx * szx))
        .sqrt()
}

/// The stress error estimator `STR(%)` at every node, as the CalculiX manual describes it
/// ("gradient error estimator"): per element the difference between the largest and the
/// smallest first principal stress over its integration points, relative to the largest
/// absolute first principal stress in the element, in percent; at a node the maximum over
/// the elements sharing it.
pub fn stress_error(
    table: &ElementTable,
    model: &Model,
    stresses: &IntegrationPointField,
) -> Vec<f64> {
    let mut error = vec![0.0; model.nodes.len()];
    for (index, entry) in table.entries.iter().enumerate() {
        let ips = table.formulation(entry).integration_points();
        let start = table.layout.offsets[index] as usize;
        let (mut lo, mut hi, mut scale) = (f64::INFINITY, f64::NEG_INFINITY, 0.0_f64);
        for g in 0..ips {
            let at = (start + g) * 6;
            let s: [f64; 6] = std::array::from_fn(|k| stresses.values[at + k]);
            let first = principal_stresses(&s)[0];
            lo = lo.min(first);
            hi = hi.max(first);
            scale = scale.max(first.abs());
        }
        let relative = if scale > 0.0 {
            100.0 * (hi - lo) / scale
        } else {
            0.0
        };
        for &n in &entry.nodes {
            let e = &mut error[n as usize];
            *e = f64::max(*e, relative);
        }
    }
    error
}

/// The principal stresses of `[sxx, syy, szz, sxy, syz, szx]`, largest first (the
/// trigonometric solution of the characteristic equation of the symmetric tensor).
pub fn principal_stresses(s: &[f64; 6]) -> [f64; 3] {
    let (a11, a22, a33, a12, a23, a13) = (s[0], s[1], s[2], s[3], s[4], s[5]);
    let p1 = a12 * a12 + a13 * a13 + a23 * a23;
    let mut v = if p1 == 0.0 {
        [a11, a22, a33]
    } else {
        let q = (a11 + a22 + a33) / 3.0;
        let p2 = (a11 - q).powi(2) + (a22 - q).powi(2) + (a33 - q).powi(2) + 2.0 * p1;
        let p = (p2 / 6.0).sqrt();
        let b = |x: f64| (x - q) / p;
        let (b11, b22, b33) = (b(a11), b(a22), b(a33));
        let (b12, b23, b13) = (a12 / p, a23 / p, a13 / p);
        let det = b11 * (b22 * b33 - b23 * b23) - b12 * (b12 * b33 - b23 * b13)
            + b13 * (b12 * b23 - b22 * b13);
        let phi = (det / 2.0).clamp(-1.0, 1.0).acos() / 3.0;
        let e1 = q + 2.0 * p * phi.cos();
        let e3 = q + 2.0 * p * (phi + 2.0 * std::f64::consts::PI / 3.0).cos();
        [e1, 3.0 * q - e1 - e3, e3]
    };
    v.sort_by(|x, y| y.total_cmp(x));
    v
}

/// Appends the fields of an increment to the results.
#[allow(clippy::too_many_arguments)]
pub fn write_increment(
    ctx: &mut ProcedureContext<'_>,
    step: &Step,
    increment: u32,
    // The step time; the `.frd` is labelled with the total time, like CalculiX does.
    _time: f64,
    total: f64,
    u: &[f64],
    reactions: &[f64],
    stresses: &IntegrationPointField,
    strains: &IntegrationPointField,
) -> Result<()> {
    let variables = requested(step);
    let has = |v: &str| variables.iter().any(|x| x == v);
    let order = node_order(ctx.model);
    let mut fields = Vec::new();
    // Native membranes: constant strain and stress per element.
    let (membrane_strains, membrane_stresses): (Vec<_>, Vec<_>) = ctx
        .table
        .membranes
        .iter()
        .map(|m| {
            let (e, s) = m.strain_stress(u);
            ((m.nodes, e), (m.nodes, s))
        })
        .unzip();
    // Internal unknowns (thickness change of plane stress nodes) are no displacements.
    let (u, reactions) = &ctx.table.reported(ctx.model, u, reactions);
    if has("U") {
        fields.push(vector_field("DISP", ["D1", "D2", "D3"], &order, u));
    }
    if has("S") {
        let (average, _) = nodal_average_with(ctx.table, ctx.model, stresses, &membrane_stresses);
        fields.push(tensor_field(
            "STRESS",
            ["SXX", "SYY", "SZZ", "SXY", "SYZ", "SZX"],
            &order,
            &average,
        ));
    }
    if has("E") {
        let (average, _) = nodal_average_with(ctx.table, ctx.model, strains, &membrane_strains);
        fields.push(tensor_field(
            "TOSTRAIN",
            ["EXX", "EYY", "EZZ", "EXY", "EYZ", "EZX"],
            &order,
            &average,
        ));
    }
    if has("RF") {
        fields.push(vector_field("FORC", ["F1", "F2", "F3"], &order, reactions));
    }
    // CalculiX writes the error estimator whenever stresses are requested, unless `NOE`
    // switches it off.
    if (has("S") || has("ERR")) && !has("NOE") {
        let error = stress_error(ctx.table, ctx.model, stresses);
        fields.push(ResultField {
            name: "ERROR".into(),
            components: vec!["STR(%)".into()],
            nodes: order.iter().map(|&(id, _)| id).collect(),
            values: vec![order.iter().map(|&(_, i)| error[i as usize]).collect()],
        });
    }
    let known = ["U", "S", "E", "RF", "ERR", "NOE"];
    let mut reported = Vec::new();
    for v in &variables {
        if !known.contains(&v.as_str()) && !reported.contains(v) {
            reported.push(v.clone());
        }
    }
    ctx.results.increments.push(ResultsIncrement {
        step: ctx.step_number,
        increment,
        kind: IncrementKind::Time,
        // CalculiX labels a time increment with the total time (the step time only in the
        // first step, where both agree).
        value: total,
        total_time: total,
        nodal_diameter: None,
        fields,
    });
    let _ = &reported;
    Ok(())
}

/// The output variables of a step the solver does not produce yet, for a warning.
pub fn unsupported_outputs(step: &Step) -> Vec<String> {
    let known = ["U", "S", "E", "RF", "ERR", "NOE"];
    let mut out = Vec::new();
    for v in requested(step) {
        if !known.contains(&v.as_str()) && !out.contains(&v) {
            out.push(v);
        }
    }
    out
}

#[allow(dead_code)]
fn warn(progress: &mut dyn plxs_core::Progress, text: String) {
    progress.event(ProgressEvent::Warning(text));
}

#[cfg(test)]
mod tests {
    use super::principal_stresses;

    #[test]
    fn principal_stresses_of_known_tensors() {
        let p = principal_stresses(&[-10.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        assert_eq!(p, [0.0, 0.0, -10.0]);
        // Pure shear τ: principal stresses ±τ and 0.
        let p = principal_stresses(&[0.0, 0.0, 0.0, 5.0, 0.0, 0.0]);
        assert!((p[0] - 5.0).abs() < 1e-12 && p[1].abs() < 1e-12 && (p[2] + 5.0).abs() < 1e-12);
        // A general tensor: invariants are reproduced.
        let s = [3.0, -2.0, 7.0, 1.5, -0.5, 2.0];
        let p = principal_stresses(&s);
        let i1 = s[0] + s[1] + s[2];
        let i2 =
            s[0] * s[1] + s[1] * s[2] + s[2] * s[0] - s[3].powi(2) - s[4].powi(2) - s[5].powi(2);
        assert!((p[0] + p[1] + p[2] - i1).abs() < 1e-12);
        assert!((p[0] * p[1] + p[1] * p[2] + p[2] * p[0] - i2).abs() < 1e-10);
        assert!(p[0] >= p[1] && p[1] >= p[2]);
    }
}
