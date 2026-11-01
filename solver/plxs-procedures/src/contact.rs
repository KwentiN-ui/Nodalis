//! Contact in the procedures: the matrix pattern with the possible contact springs, and the
//! contact fields of an increment (`*CONTACT FILE`).

use std::sync::Arc;

use plxs_assembly::ElementTable;
use plxs_contact::{ContactModel, Evaluation};
use plxs_core::Model;
use plxs_core::results::ResultField;
use plxs_core::step::Step;
use plxs_linalg::Pattern;

/// The pattern of the element blocks plus extra blocks (contact springs).
pub fn pattern_with(table: &ElementTable, extra: &[Vec<u32>]) -> Arc<Pattern> {
    Arc::new(Pattern::from_blocks(
        table.dofs.len(),
        table
            .entries
            .iter()
            .map(|e| e.dofs.as_slice())
            .chain(table.linear_blocks.iter().map(|b| b.dofs.as_slice()))
            .chain(table.discrete.iter().map(|e| e.dofs.as_slice()))
            .chain(extra.iter().map(Vec::as_slice)),
    ))
}

/// The `CONTACT` (CDIS, CSTR) and `CELS` fields a step asks for, over all nodes in `order`.
pub fn contact_fields(
    model: &Model,
    step: &Step,
    contact: &ContactModel,
    evaluation: &Evaluation,
    order: &[(u32, u32)],
) -> Vec<ResultField> {
    let variables: Vec<String> = step
        .outputs
        .contact_file
        .iter()
        .flat_map(|o| o.variables.iter().map(|v| v.trim().to_ascii_uppercase()))
        .collect();
    let has = |v: &str| variables.iter().any(|x| x == v);
    let mut out = Vec::new();
    if !has("CDIS") && !has("CSTR") && !has("CELS") {
        return out;
    }
    let (values, energy) = plxs_contact::output::nodal_values(contact, evaluation, model.nodes.len());
    let nodes: Vec<u32> = order.iter().map(|&(id, _)| id).collect();
    if has("CDIS") || has("CSTR") {
        let names = ["COPEN", "CSLIP1", "CSLIP2", "CPRESS", "CSHEAR1", "CSHEAR2"];
        out.push(ResultField {
            name: "CONTACT".into(),
            components: names.iter().map(|s| s.to_string()).collect(),
            nodes: nodes.clone(),
            values: (0..6)
                .map(|c| order.iter().map(|&(_, i)| values[i as usize][c]).collect())
                .collect(),
        });
    }
    if has("CELS") {
        out.push(ResultField {
            name: "CELS".into(),
            components: vec!["CELS".into()],
            nodes,
            values: vec![order.iter().map(|&(_, i)| energy[i as usize]).collect()],
        });
    }
    out
}
