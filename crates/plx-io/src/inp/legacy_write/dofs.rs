//! The dependent degrees of freedom of the input file, kept in one place so that the
//! constraints Nodalis writes itself never bind the same one twice. CalculiX aborts when a
//! degree of freedom is the dependent one of an `*EQUATION` or `*RIGID BODY` and a boundary
//! condition holds it ("dependent side of a MPC and a SPC"), or when two of them depend on
//! it ("dependent side of two different MPC's"); `*TIE` skips such degrees of freedom by
//! itself (tried with CalculiX 2.21 and 2.23).
//!
//! The writer claims the degrees of freedom in the order of precedence: boundary conditions
//! and cyclic symmetry first, then rigid bodies and remote points, the user's own
//! constraints, then pre-tension cuts and node ties, last the equations of contact regions.
//! As Abaqus moves a boundary condition of a tie's dependent node to its independent nodes,
//! an equation whose dependent term is taken depends on another of its terms instead: a free
//! direction of the same node, else a node on the other side. An equation all of whose terms
//! are taken binds nothing new and is left out; so is a node of a rigid body that is taken.
//! What was changed is listed as comments at the top of the constraints, and the
//! application shows it in the solver output.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

use plx_mesh::NodeId;

/// A term of an `*EQUATION`: node, degree of freedom, factor.
pub(super) type Term = (NodeId, u8, f64);

/// Below this share of the largest factor a term does not become the dependent one, which
/// would make the equation badly conditioned, while another term will do.
const PREFERRED_FACTOR: f64 = 0.1;
/// Terms below this share of the largest factor never become the dependent one.
const SMALLEST_FACTOR: f64 = 1e-6;

#[derive(Default)]
pub(super) struct Dofs {
    /// Translations held by boundary conditions in any step.
    held: BTreeMap<NodeId, [bool; 3]>,
    /// Translations an MPC depends on.
    dependent: BTreeMap<NodeId, [bool; 3]>,
    /// Per item, what was changed: equations depending on another term, equations left
    /// out, nodes left out of a rigid body.
    changes: BTreeMap<String, Changes>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Changes {
    moved: usize,
    dropped: usize,
    left_out: usize,
}

impl Dofs {
    pub fn new(held: BTreeMap<NodeId, [bool; 3]>) -> Self {
        Self {
            held,
            ..Self::default()
        }
    }

    /// Whether an MPC may not depend on the degree of freedom: a boundary condition holds
    /// it or another MPC depends on it. Rotations and temperatures are never taken here.
    pub fn taken(&self, node: NodeId, dof: u8) -> bool {
        let Some(k) = (1..=3).contains(&dof).then(|| usize::from(dof) - 1) else {
            return false;
        };
        let held = self.held.get(&node).is_some_and(|h| h[k]);
        held || self.dependent.get(&node).is_some_and(|d| d[k])
    }

    fn mark(&mut self, node: NodeId, dof: u8) {
        if (1..=3).contains(&dof) {
            self.dependent.entry(node).or_default()[usize::from(dof) - 1] = true;
        }
    }

    /// Nodes that CalculiX's own MPCs depend on in every translation, such as the slave
    /// side of a cyclic symmetry.
    pub fn mark_nodes(&mut self, nodes: impl IntoIterator<Item = NodeId>) {
        for node in nodes {
            for dof in 1..=3 {
                self.mark(node, dof);
            }
        }
    }

    /// The nodes a rigid body of `item` may take, in the order given: those with no
    /// translation taken. They are taken then.
    pub fn claim_nodes(&mut self, item: &str, nodes: Vec<NodeId>) -> Vec<NodeId> {
        let (kept, left_out): (Vec<NodeId>, Vec<NodeId>) =
            (nodes.into_iter()).partition(|&n| (1..=3).all(|dof| !self.taken(n, dof)));
        self.mark_nodes(kept.iter().copied());
        if !left_out.is_empty() {
            self.changes.entry(item.to_owned()).or_default().left_out += left_out.len();
        }
        kept
    }

    /// Takes the dependent degree of freedom of an equation of `item`, the first term. If it
    /// is taken already, a term that `movable` allows becomes the first one: in the order
    /// of the terms the first free one with a factor of at least [`PREFERRED_FACTOR`] of the
    /// largest, else any free one that is not negligible. False if no term is free; the
    /// equation is left out then.
    pub fn claim_equation(
        &mut self,
        item: &str,
        terms: &mut [Term],
        movable: impl Fn(NodeId) -> bool,
    ) -> bool {
        let Some(&(node, dof, _)) = terms.first() else {
            return false;
        };
        if !self.taken(node, dof) {
            self.mark(node, dof);
            return true;
        }
        let largest = terms.iter().fold(0.0_f64, |m, t| m.max(t.2.abs()));
        // A degree of freedom appearing twice cannot be solved for.
        let mut seen = BTreeSet::new();
        let twice: BTreeSet<(NodeId, u8)> = (terms.iter())
            .filter(|(n, d, _)| !seen.insert((*n, *d)))
            .map(|(n, d, _)| (*n, *d))
            .collect();
        let free = |t: &Term, share: f64| {
            movable(t.0)
                && (1..=3).contains(&t.1)
                && !self.taken(t.0, t.1)
                && !twice.contains(&(t.0, t.1))
                && t.2.abs() >= share * largest
                && t.2 != 0.0
        };
        let chosen = (1..terms.len())
            .find(|&i| free(&terms[i], PREFERRED_FACTOR))
            .or_else(|| (1..terms.len()).find(|&i| free(&terms[i], SMALLEST_FACTOR)));
        let changes = self.changes.entry(item.to_owned()).or_default();
        match chosen {
            Some(i) => {
                changes.moved += 1;
                terms.swap(0, i);
                let (node, dof, _) = terms[0];
                self.mark(node, dof);
                true
            }
            None => {
                changes.dropped += 1;
                false
            }
        }
    }

    /// The changes as comment lines of the input file, see [`resolved_overlaps`], or
    /// nothing if there were none.
    pub fn comments(&self) -> Option<String> {
        if self.changes.is_empty() {
            return None;
        }
        let mut out = String::new();
        for (item, changes) in &self.changes {
            let _ = writeln!(out, "{PREFIX}{item}: {}", describe(changes));
        }
        Some(out)
    }
}

/// The start of the comment lines that list the changes.
const PREFIX: &str = "** Overlapping constraints resolved: ";

/// What was changed for an item, in words.
fn describe(changes: &Changes) -> String {
    let plural = |n: usize, one: &str, many: &str| {
        if n == 1 {
            format!("1 {one}")
        } else {
            format!("{n} {many}")
        }
    };
    let mut parts = Vec::new();
    if changes.left_out > 0 {
        parts.push(format!(
            "{} left out (held already)",
            plural(changes.left_out, "node", "nodes")
        ));
    }
    if changes.moved > 0 {
        parts.push(format!(
            "{} on another degree of freedom (the first one is held already)",
            plural(changes.moved, "equation", "equations")
        ));
    }
    if changes.dropped > 0 {
        parts.push(format!(
            "{} left out (all degrees of freedom held already)",
            plural(changes.dropped, "equation", "equations")
        ));
    }
    parts.join("; ")
}

/// The overlapping constraints the writer resolved, one line each, as listed in the input
/// file `inp`: "No separation-1: 3 equations on another degree of freedom (...)".
pub fn resolved_overlaps(inp: &str) -> Vec<String> {
    (inp.lines())
        .filter_map(|line| line.strip_prefix(PREFIX))
        .map(str::to_owned)
        .collect()
}
