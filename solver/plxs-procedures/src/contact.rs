//! Contact in the procedures: the matrix pattern with the possible contact springs, and the
//! contact fields of an increment (`*CONTACT FILE`).

use rayon::prelude::*;
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
        (table.blocks()).chain(extra.iter().map(Vec::as_slice)),
    ))
}

/// The pattern with the contact springs of `extra`. The pattern of an earlier increment
/// (or step) in `cached` is kept as long as it holds every entry of `extra`: candidate
/// master faces change from increment to increment, and a pattern that stays the same lets
/// the solver keep its symbolic analysis (ordering, elimination tree); the entries of
/// springs that are gone stay zero. Otherwise a pattern of `extra` alone replaces it (a
/// union over all increments would couple a sliding surface with every face it has passed
/// and fill the factor).
pub fn cached_pattern(
    table: &ElementTable,
    extra: &[Vec<u32>],
    cached: &mut Option<Arc<Pattern>>,
) -> Arc<Pattern> {
    if let Some(current) = cached.as_ref()
        && current.dimension() == table.dofs.len()
        && extra.par_iter().all(|block| covers(current, block))
    {
        return current.clone();
    }
    let pattern = pattern_with(table, extra);
    *cached = Some(pattern.clone());
    pattern
}

/// Whether every pair of indices of a block is an entry of the pattern.
pub(crate) fn covers(pattern: &Pattern, block: &[u32]) -> bool {
    block.iter().all(|&a| {
        block
            .iter()
            .all(|&b| a > b || pattern.position(a, b).is_some())
    })
}

/// The `CONTACT` (CDIS, CSTR) and `CELS` fields a step asks for, over all nodes in `order`:
/// those of the contact pairs (with their `evaluation`) and of the compression only
/// surfaces (COPEN and CPRESS at the displacements `u`).
pub fn contact_fields(
    model: &Model,
    step: &Step,
    contact: &ContactModel,
    evaluation: Option<&Evaluation>,
    u: &[f64],
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
    let supports = crate::support::nodal_values(model, u);
    let mut nodal = match evaluation {
        Some(evaluation) => {
            plxs_contact::output::nodal_values(contact, evaluation, model.nodes.len())
        }
        None if supports.is_some() => plxs_contact::output::NodalValues {
            values: vec![[0.0; 6]; model.nodes.len()],
            energy: vec![0.0; model.nodes.len()],
            listed: Some(vec![false; model.nodes.len()]),
        },
        None => return out,
    };
    for (i, value) in supports.iter().flatten().enumerate() {
        if let Some([opening, pressure]) = *value {
            nodal.values[i][0] = opening;
            nodal.values[i][3] += pressure;
            if let Some(listed) = nodal.listed.as_mut() {
                listed[i] = true;
            }
        }
    }
    let (values, energy) = (&nodal.values, &nodal.energy);
    let order: Vec<(u32, u32)> = order
        .iter()
        .copied()
        .filter(|&(_, i)| nodal.listed.as_ref().is_none_or(|l| l[i as usize]))
        .collect();
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_pattern_covers_the_blocks_it_was_built_from_and_their_parts() {
        let pattern = Pattern::from_blocks(6, [[0u32, 1, 2].as_slice(), &[3, 4]]);
        assert!(covers(&pattern, &[0, 1, 2]));
        assert!(covers(&pattern, &[0, 2]));
        assert!(covers(&pattern, &[4, 3]));
        assert!(covers(&pattern, &[5]));
        assert!(!covers(&pattern, &[2, 3]));
        assert!(!covers(&pattern, &[1, 5]));
    }
}
