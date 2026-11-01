//! Assembly of the matrices the eigenvalue procedures need beside `K`: the consistent
//! mass matrix, the initial stress (geometric) stiffness for stresses at the integration
//! points, and the initial displacement stiffness of a deformed state. All share the
//! pattern of the stiffness matrix.

use plxs_core::state::IntegrationPointField;
use plxs_core::{Error, Model, Result};
use plxs_linalg::SparseMatrix;
use rayon::prelude::*;

use crate::ElementTable;
use crate::elements::ElementEntry;

/// Computes the element matrices in parallel batches of bounded memory and scatters them
/// sequentially.
fn assemble_blocks<F>(table: &ElementTable, out: &mut SparseMatrix, element: F) -> Result<()>
where
    F: Fn(usize, &ElementEntry) -> Result<Vec<f64>> + Sync,
{
    out.fill_zero();
    let largest = table
        .formulations
        .iter()
        .map(|f| f.dof_count() * f.dof_count())
        .max()
        .unwrap_or(1);
    let batch = (64 * 1024 * 1024 / (8 * largest)).clamp(64, 1 << 16);
    for (chunk, entries) in table.entries.chunks(batch).enumerate() {
        let matrices: Vec<Result<Vec<f64>>> = entries
            .par_iter()
            .enumerate()
            .map(|(i, entry)| element(chunk * batch + i, entry))
            .collect();
        for (entry, matrix) in entries.iter().zip(matrices) {
            out.add_block(&entry.dofs, &matrix?);
        }
    }
    Ok(())
}

/// Which integration rule the consistent mass matrix takes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MassRule {
    /// The element's own rule, as CalculiX's eigenvalue procedures.
    Element,
    /// The full rule of the family, as CalculiX's direct time integration.
    Full,
}

/// Assembles the consistent mass matrix. Every element needs a density.
pub fn assemble_mass(table: &ElementTable, model: &Model, m: &mut SparseMatrix) -> Result<()> {
    assemble_mass_with(table, model, MassRule::Element, m)
}

/// [`assemble_mass`] with the given choice of integration rule.
pub fn assemble_mass_with(
    table: &ElementTable,
    model: &Model,
    rule: MassRule,
    m: &mut SparseMatrix,
) -> Result<()> {
    assemble_blocks(table, m, |index, entry| {
        let rho = entry.density.ok_or_else(|| {
            Error::input(
                "*DENSITY",
                format!(
                    "element {} has no density; a frequency or dynamic analysis needs *DENSITY for every material",
                    model.elements[index].id
                ),
            )
        })?;
        let formulation = table.formulation(entry);
        let coords = table.coordinates(model, entry);
        let mut me = Vec::new();
        match rule {
            MassRule::Element => formulation.mass(&coords, rho, &mut me),
            MassRule::Full => {
                formulation.mass_with(&coords, rho, &formulation.dynamic_mass_rule(), &mut me)
            }
        }
        Ok(me)
    })?;
    // Native membranes, after the solid blocks (`assemble_blocks` starts from zero).
    for membrane in &table.membranes {
        let rho = membrane.density.ok_or_else(|| {
            Error::input(
                "*DENSITY",
                format!(
                    "element {} has no density; a frequency or dynamic analysis needs *DENSITY for every material",
                    model.elements[membrane.element as usize].id
                ),
            )
        })?;
        m.add_block(&membrane.dofs, &membrane.formulation.mass(rho));
    }
    Ok(())
}

/// Assembles the initial stress stiffness `K_σ` for the stresses at the integration
/// points (6 per point, `[xx, yy, zz, xy, yz, zx]`, in the layout of the element table).
pub fn assemble_geometric_stiffness(
    table: &ElementTable,
    model: &Model,
    stresses: &IntegrationPointField,
    k_sigma: &mut SparseMatrix,
) -> Result<()> {
    assemble_blocks(table, k_sigma, |index, entry| {
        let formulation = table.formulation(entry);
        let coords = table.coordinates(model, entry);
        let start = table.layout.offsets[index] as usize;
        let end = table.layout.offsets[index + 1] as usize;
        let s: Vec<[f64; 6]> = (start..end)
            .map(|p| std::array::from_fn(|k| stresses.values[p * 6 + k]))
            .collect();
        let mut ke = Vec::new();
        formulation.geometric_stiffness(&coords, &s, &mut ke);
        Ok(ke)
    })
}

/// Assembles the tangent stiffness of a deformed state `u` with stresses `stresses`:
/// material stiffness plus initial displacement stiffness plus initial stress stiffness
/// (total Lagrangian), what a perturbation step linearizes about.
pub fn assemble_tangent_stiffness(
    table: &ElementTable,
    model: &Model,
    u: &[f64],
    stresses: &IntegrationPointField,
    k: &mut SparseMatrix,
) -> Result<()> {
    assemble_blocks(table, k, |index, entry| {
        let formulation = table.formulation(entry);
        let coords = table.coordinates(model, entry);
        let d = table.material_matrix(entry);
        let ue = table.gather(entry, u);
        let start = table.layout.offsets[index] as usize;
        let end = table.layout.offsets[index + 1] as usize;
        let s: Vec<[f64; 6]> = (start..end)
            .map(|p| std::array::from_fn(|k| stresses.values[p * 6 + k]))
            .collect();
        let mut ke = Vec::new();
        formulation.stiffness(&coords, d, &mut ke);
        formulation.initial_displacement_stiffness(&coords, d, &ue, &mut ke);
        let mut ks = Vec::new();
        formulation.geometric_stiffness(&coords, &s, &mut ks);
        for (a, b) in ke.iter_mut().zip(&ks) {
            *a += b;
        }
        Ok(ke)
    })
}
