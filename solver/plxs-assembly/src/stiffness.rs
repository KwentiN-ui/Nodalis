//! Sparsity pattern and parallel assembly of the global stiffness matrix.

use std::sync::Arc;

use plxs_core::Model;
use plxs_linalg::{Pattern, SparseMatrix};
use rayon::prelude::*;

use crate::ElementTable;

/// The pattern of the global matrix: every element couples all its DOFs.
pub fn build_pattern(table: &ElementTable) -> Arc<Pattern> {
    Arc::new(Pattern::from_blocks(
        table.dofs.len(),
        table
            .entries
            .iter()
            .map(|e| e.dofs.as_slice())
            .chain(table.discrete.iter().map(|e| e.dofs.as_slice())),
    ))
}

/// Assembles `K` from the element stiffness matrices: the elements are computed in
/// parallel batches (bounded memory), each batch is scattered into the matrix sequentially.
pub fn assemble_stiffness(table: &ElementTable, model: &Model, k: &mut SparseMatrix) {
    k.fill_zero();
    // Batches of about 64 MB of element matrices.
    let largest = table
        .formulations
        .iter()
        .map(|f| f.dof_count() * f.dof_count())
        .max()
        .unwrap_or(1);
    let batch = (64 * 1024 * 1024 / (8 * largest)).clamp(64, 1 << 16);
    for entries in table.entries.chunks(batch) {
        let matrices: Vec<Vec<f64>> = entries
            .par_iter()
            .map(|entry| {
                let formulation = table.formulation(entry);
                let coords = table.coordinates(model, entry);
                let mut ke = Vec::new();
                formulation.stiffness(&coords, table.material_matrix(entry), &mut ke);
                ke
            })
            .collect();
        for (entry, ke) in entries.iter().zip(&matrices) {
            k.add_block(&entry.dofs, ke);
        }
    }
    // Springs; gaps depend on their status and are added by the procedures.
    for element in &table.discrete {
        if let Some(ke) = element.spring_stiffness() {
            k.add_block(&element.dofs, &ke);
        }
    }
}
