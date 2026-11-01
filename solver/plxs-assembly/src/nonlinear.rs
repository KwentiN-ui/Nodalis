//! Parallel assembly of the internal force vector and the tangent stiffness for nonlinear
//! steps: every element integrates its material law from the state at the start of the
//! increment to the given displacements; stresses, strains and material state variables
//! come back as integration-point fields the procedure stores once the increment converged.

use plxs_core::state::{IntegrationPointField, MaterialState};
use plxs_core::{Model, State};
use plxs_elements::PointKinematics;
use plxs_linalg::SparseMatrix;
use plxs_materials::{MaterialInput, STATE_SIZE};
use rayon::prelude::*;

use crate::ElementTable;

/// What an assembly evaluates the elements at.
#[derive(Clone, Copy, Debug)]
pub struct NonlinearInput<'a> {
    /// Displacements at the end of the increment (all DOFs).
    pub u: &'a [f64],
    pub nlgeom: bool,
    /// Size of the time increment and total time at its end.
    pub dt: f64,
    pub time: f64,
    /// Whether creep acts (`*VISCO`).
    pub creep_active: bool,
    pub temperature: f64,
    /// Pressures on native membranes that follow the deformed surface (`NLGEOM`):
    /// (membrane index, pressure at the end of the increment). Their nodal forces enter
    /// `f_int` with the opposite sign; their load stiffness is left out of the tangent.
    pub follower: &'a [(u32, f64)],
}

/// What an assembly produced.
#[derive(Clone, Debug)]
pub struct NonlinearOutput {
    /// Internal nodal forces (all DOFs).
    pub f_int: Vec<f64>,
    /// Cauchy stresses at the integration points (6 per point).
    pub stresses: IntegrationPointField,
    /// Strains at the integration points (6 per point, tensor shear components).
    pub strains: IntegrationPointField,
    /// Material state at the end of the increment.
    pub material: MaterialState,
    /// Sum of the absolute values of all element nodal force components and their number:
    /// CalculiX's "average force" for the convergence check is their ratio.
    pub flux_sum: f64,
    pub flux_count: u64,
    /// Largest change of the equivalent creep strain rate against the start of the
    /// increment, over all integration points (`CETOL` control).
    pub max_creep_rate_change: f64,
}

/// The material state field of a model with the layout of `table` (zeros).
pub fn material_state(table: &ElementTable) -> MaterialState {
    table.layout.field(STATE_SIZE)
}

/// Assembles the internal forces and, when `k` is given, the tangent stiffness at `input.u`
/// starting from `state` (its material variables, which must have the layout of
/// [`material_state`]).
pub fn assemble_nonlinear(
    table: &ElementTable,
    model: &Model,
    input: &NonlinearInput<'_>,
    state: &State,
    mut k: Option<&mut SparseMatrix>,
) -> NonlinearOutput {
    if let Some(k) = k.as_deref_mut() {
        k.fill_zero();
    }
    let total = table.layout.total();
    let mut out = NonlinearOutput {
        f_int: vec![0.0; table.dofs.len()],
        stresses: table.layout.field(6),
        strains: table.layout.field(6),
        material: table.layout.field(STATE_SIZE),
        flux_sum: 0.0,
        flux_count: 0,
        max_creep_rate_change: 0.0,
    };
    let old_material: &[f64] = if state.material.values.len() == total * STATE_SIZE {
        &state.material.values
    } else {
        &out.material.values
    };
    let old_material = old_material.to_vec();
    let want_k = k.is_some();

    struct ElementResult {
        fe: Vec<f64>,
        ke: Vec<f64>,
        stresses: Vec<f64>,
        strains: Vec<f64>,
        material: Vec<f64>,
        creep_change: f64,
    }

    let largest = table
        .formulations
        .iter()
        .map(|f| f.dof_count() * f.dof_count())
        .max()
        .unwrap_or(1);
    let batch = (64 * 1024 * 1024 / (8 * largest)).clamp(64, 1 << 16);
    for (chunk_index, entries) in table.entries.chunks(batch).enumerate() {
        let first = chunk_index * batch;
        let results: Vec<ElementResult> = entries
            .par_iter()
            .enumerate()
            .map(|(i, entry)| {
                let index = first + i;
                let formulation = table.formulation(entry);
                let law = table.law(entry, input.nlgeom);
                let coords = table.coordinates(model, entry);
                let ue = table.gather(entry, input.u);
                let start = table.layout.offsets[index] as usize;
                let points = formulation.integration_points();
                let mut material =
                    old_material[start * STATE_SIZE..(start + points) * STATE_SIZE].to_vec();
                let mut stresses = vec![0.0; 6 * points];
                let mut strains = vec![0.0; 6 * points];
                let mut creep_change = 0.0_f64;
                let mut fe = vec![0.0; formulation.dof_count()];
                let mut ke = Vec::new();
                let d_elastic = law.elastic_matrix();
                formulation.nonlinear(
                    &coords,
                    &ue,
                    input.nlgeom,
                    &d_elastic,
                    |ip, kinematics: &PointKinematics| {
                        let slice = &mut material[ip * STATE_SIZE..(ip + 1) * STATE_SIZE];
                        let old_rate = slice[STATE_SIZE - 1];
                        let output = law.stress(
                            &MaterialInput {
                                strain: kinematics.strain,
                                temperature: input.temperature,
                                dt: input.dt,
                                time: input.time,
                                creep_active: input.creep_active,
                            },
                            slice,
                        );
                        creep_change = creep_change.max((slice[STATE_SIZE - 1] - old_rate).abs());
                        let sigma = if input.nlgeom {
                            kinematics.cauchy(&output.stress)
                        } else {
                            output.stress
                        };
                        stresses[ip * 6..(ip + 1) * 6].copy_from_slice(&sigma);
                        let e = &kinematics.strain;
                        strains[ip * 6..(ip + 1) * 6].copy_from_slice(&[
                            e[0],
                            e[1],
                            e[2],
                            0.5 * e[3],
                            0.5 * e[4],
                            0.5 * e[5],
                        ]);
                        (output.stress, output.tangent)
                    },
                    &mut fe,
                    want_k.then_some(&mut ke),
                );
                ElementResult {
                    fe,
                    ke,
                    stresses,
                    strains,
                    material,
                    creep_change,
                }
            })
            .collect();
        for (i, (entry, result)) in entries.iter().zip(&results).enumerate() {
            let index = first + i;
            let start = table.layout.offsets[index] as usize;
            for (&dof, value) in entry.dofs.iter().zip(&result.fe) {
                out.f_int[dof as usize] += value;
                out.flux_sum += value.abs();
            }
            out.flux_count += result.fe.len() as u64;
            if let Some(k) = k.as_deref_mut() {
                k.add_block(&entry.dofs, &result.ke);
            }
            let points = result.stresses.len() / 6;
            out.stresses.values[start * 6..(start + points) * 6].copy_from_slice(&result.stresses);
            out.strains.values[start * 6..(start + points) * 6].copy_from_slice(&result.strains);
            out.material.values[start * STATE_SIZE..(start + points) * STATE_SIZE]
                .copy_from_slice(&result.material);
            out.max_creep_rate_change = out.max_creep_rate_change.max(result.creep_change);
        }
    }
    // Native membranes (total Lagrangian under NLGEOM) and the other elements of constant
    // stiffness (U1). Membranes have no integration points in the layout.
    for block in table.linear_blocks.iter().filter(|b| !b.membrane) {
        let n = block.dofs.len();
        for (i, &dof) in block.dofs.iter().enumerate() {
            let f: f64 = (0..n)
                .map(|j| block.stiffness[i * n + j] * input.u[block.dofs[j] as usize])
                .sum();
            out.f_int[dof as usize] += f;
            out.flux_sum += f.abs();
        }
        out.flux_count += n as u64;
        if let Some(k) = k.as_deref_mut() {
            k.add_block(&block.dofs, &block.stiffness);
        }
    }
    let membranes: Vec<(Vec<f64>, Vec<f64>)> = table
        .membranes
        .par_iter()
        .map(|m| {
            let mut fe = vec![0.0; m.dofs.len()];
            let mut ke = Vec::new();
            m.formulation.internal(
                &m.gather(input.u),
                input.nlgeom,
                &mut fe,
                want_k.then_some(&mut ke),
            );
            (fe, ke)
        })
        .collect();
    for (m, (fe, ke)) in table.membranes.iter().zip(&membranes) {
        for (&dof, value) in m.dofs.iter().zip(fe) {
            out.f_int[dof as usize] += value;
            out.flux_sum += value.abs();
        }
        out.flux_count += fe.len() as u64;
        if let Some(k) = k.as_deref_mut() {
            k.add_block(&m.dofs, ke);
        }
    }
    for &(index, p) in input.follower {
        let m = &table.membranes[index as usize];
        let mut fe = vec![0.0; m.dofs.len()];
        m.formulation.pressure(Some(&m.gather(input.u)), p, &mut fe);
        for (&dof, value) in m.dofs.iter().zip(fe) {
            out.f_int[dof as usize] -= value;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use plxs_core::element::ElementType;
    use plxs_core::material::{Elastic, ElasticRow, Hardening, Material, Plastic, PlasticRow};
    use plxs_elements::solid;
    use plxs_materials::law_of;

    fn steel() -> Material {
        Material {
            name: "STEEL".into(),
            elastic: Some(Elastic {
                rows: vec![ElasticRow {
                    young: 210_000.0,
                    poisson: 0.3,
                    temperature: 0.0,
                }],
            }),
            plastic: Some(Plastic {
                hardening: Hardening::Isotropic,
                rows: vec![
                    PlasticRow {
                        stress: 235.0,
                        plastic_strain: 0.0,
                        temperature: 0.0,
                    },
                    PlasticRow {
                        stress: 400.0,
                        plastic_strain: 0.2,
                        temperature: 0.0,
                    },
                ],
            }),
            ..Material::default()
        }
    }

    /// The element tangent with the finite-strain plastic law is the derivative of the
    /// internal force: a plastic state from a first stretch, then a further stretch.
    #[test]
    fn element_tangent_with_plasticity_is_the_derivative_of_the_internal_force() {
        let law = law_of(&steel(), 0.0, true).unwrap();
        let e = solid(ElementType::C3D8).unwrap();
        let coords: Vec<[f64; 3]> = e
            .family
            .node_coordinates()
            .iter()
            .map(|c| [5.0 * c[0], 5.0 * c[1], 5.0 * c[2]])
            .collect();
        let ndof = e.dof_count();
        let points = e.integration_points();
        let d = law.elastic_matrix();
        let stretch = |coords: &[[f64; 3]], a: f64| -> Vec<f64> {
            coords
                .iter()
                .flat_map(|c| [a * c[0], -0.3 * a * c[1], -0.3 * a * c[2]])
                .collect()
        };
        let input = |strain: [f64; 6]| MaterialInput {
            strain,
            temperature: 0.0,
            dt: 1.0,
            time: 1.0,
            creep_active: false,
        };
        // First increment to 0.6 % strain: plastic everywhere.
        let mut state = vec![0.0; points * STATE_SIZE];
        let u0 = stretch(&coords, 0.006);
        let mut fe = vec![0.0; ndof];
        e.nonlinear(
            &coords,
            &u0,
            true,
            &d,
            |ip, k: &PointKinematics| {
                let s = &mut state[ip * STATE_SIZE..(ip + 1) * STATE_SIZE];
                let o = law.stress(&input(k.strain), s);
                (o.stress, o.tangent)
            },
            &mut fe,
            None,
        );
        let force = |u: &[f64], ke: Option<&mut Vec<f64>>| -> Vec<f64> {
            let mut fe = vec![0.0; ndof];
            let mut st = state.clone();
            e.nonlinear(
                &coords,
                u,
                true,
                &d,
                |ip, k: &PointKinematics| {
                    let s = &mut st[ip * STATE_SIZE..(ip + 1) * STATE_SIZE];
                    let o = law.stress(&input(k.strain), s);
                    (o.stress, o.tangent)
                },
                &mut fe,
                ke,
            );
            fe
        };
        let mut u = stretch(&coords, 0.008);
        for (i, v) in u.iter_mut().enumerate() {
            *v += 0.002 * ((i as f64) * 0.7).sin();
        }
        let mut ke = Vec::new();
        force(&u, Some(&mut ke));
        let h = 1e-6;
        let scale = ke.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        let mut worst = 0.0_f64;
        for j in 0..ndof {
            let mut up = u.clone();
            up[j] += h;
            let mut um = u.clone();
            um[j] -= h;
            let (fp, fm) = (force(&up, None), force(&um, None));
            for i in 0..ndof {
                let fd = (fp[i] - fm[i]) / (2.0 * h);
                worst = worst.max((fd - ke[i * ndof + j]).abs() / scale);
            }
        }
        assert!(worst < 1e-5, "worst relative deviation {worst:e}");
    }
}
