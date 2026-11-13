//! Degree-of-freedom numbering, single-point constraints and the reduction to free DOFs.

use plxs_core::mesh::NodeRef;
use plxs_core::step::{Boundary, DefaultAmplitude, Step};
use plxs_core::{Model, Result};

/// Global DOF numbers: `per_node` per node in dense node order, `node * per_node + d`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DofMap {
    pub per_node: usize,
    pub nodes: usize,
    /// DOFs no element works on (ascending): nodes outside every element, the out-of-plane
    /// DOF of plane strain and axisymmetric nodes. They are held at zero.
    pub unused: Vec<u32>,
}

impl DofMap {
    pub fn new(nodes: usize, per_node: usize) -> Self {
        Self {
            per_node,
            nodes,
            unused: Vec::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.nodes * self.per_node
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The global number of DOF `d` (0-based) of node `node` (dense index), `None` when the
    /// node has no such DOF (rotations of solid nodes).
    pub fn dof(&self, node: u32, d: usize) -> Option<u32> {
        (d < self.per_node).then(|| node * self.per_node as u32 + d as u32)
    }
}

/// Prescribed values on single DOFs: `dofs` ascending, `values` the prescribed
/// displacements (zero for fixed DOFs).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Spc {
    pub dofs: Vec<u32>,
    pub values: Vec<f64>,
}

/// The factor a load or boundary with amplitude `amplitude` carries at step time `time`
/// (total time `total`) of a step with period `period`.
pub fn amplitude_factor(
    model: &Model,
    step: &Step,
    amplitude: Option<&str>,
    time: f64,
    total: f64,
    period: f64,
) -> f64 {
    match amplitude.and_then(|name| model.amplitude(name)) {
        Some(a) => a.value(if a.total_time { total } else { time }),
        None => match step.default_amplitude {
            DefaultAmplitude::Ramp => {
                if period > 0.0 {
                    (time / period).clamp(0.0, 1.0)
                } else {
                    1.0
                }
            }
            DefaultAmplitude::Step => 1.0,
        },
    }
}

/// The displacement component `k` of a node at the end of the previous step, in the node's
/// local system when it has one.
fn previous_component(
    dofs: &DofMap,
    previous: &[f64],
    rotation: &dyn Fn(u32) -> Option<[[f64; 3]; 3]>,
    index: u32,
    k: usize,
) -> f64 {
    let global = |c: usize| {
        dofs.dof(index, c)
            .and_then(|g| previous.get(g as usize).copied())
            .unwrap_or(0.0)
    };
    match rotation(index) {
        Some(r) if k < 3 => (0..3).map(|c| r[c][k] * global(c)).sum(),
        _ => global(k),
    }
}

impl Spc {
    /// The single-point constraints of a step at step time `time`. DOFs a node does not
    /// have (rotations of solid nodes, DOF 11 in a mechanical step) are skipped, as CalculiX
    /// does. A DOF prescribed twice takes the last definition.
    pub fn of_step(
        model: &Model,
        step: &Step,
        dofs: &DofMap,
        time: f64,
        total: f64,
        period: f64,
    ) -> Result<Spc> {
        let mut prescribed: Vec<(u32, f64)> = Vec::new();
        for boundary in &step.boundaries {
            let factor = amplitude_factor(
                model,
                step,
                boundary.amplitude.as_deref(),
                time,
                total,
                period,
            );
            let value = boundary.value.unwrap_or(0.0) * factor;
            for node in Self::nodes(model, boundary) {
                let Some(index) = model.nodes.index(node) else {
                    continue;
                };
                for d in boundary.first_dof..=boundary.last_dof {
                    if !(1..=6).contains(&d) {
                        continue;
                    }
                    if let Some(dof) = dofs.dof(index, d as usize - 1) {
                        prescribed.push((dof, value));
                    }
                }
            }
        }
        // Unused DOFs are held at zero, whatever a boundary prescribes on them.
        prescribed.retain(|(dof, _)| dofs.unused.binary_search(dof).is_err());
        prescribed.extend(dofs.unused.iter().map(|&dof| (dof, 0.0)));
        prescribed.sort_by_key(|&(dof, _)| dof);
        let mut spc = Spc::default();
        for (dof, value) in prescribed {
            if spc.dofs.last() == Some(&dof) {
                *spc.values.last_mut().unwrap() = value;
            } else {
                spc.dofs.push(dof);
                spc.values.push(value);
            }
        }
        Ok(spc)
    }

    fn nodes(model: &Model, boundary: &Boundary) -> Vec<u32> {
        match &boundary.region {
            NodeRef::Node(id) => vec![*id],
            NodeRef::Set(name) => model.sets.node_sets.get(name).cloned().unwrap_or_default(),
        }
    }

    /// The path of the prescribed values from the end of the previous step (`previous`,
    /// global components, read like [`Spc::apply_fixed`] does) in an incremental procedure:
    /// at the start of the step every DOF still has its previous displacement, a boundary
    /// on the default `RAMP` amplitude goes linearly from there to its value, one on the
    /// `STEP` amplitude or an amplitude of its own takes the value from the first
    /// increment on. `time` is the step time the values of `self` were computed for.
    #[allow(clippy::too_many_arguments)]
    pub fn start_from_previous(
        &mut self,
        model: &Model,
        step: &Step,
        dofs: &DofMap,
        previous: &[f64],
        rotation: &dyn Fn(u32) -> Option<[[f64; 3]; 3]>,
        time: f64,
        period: f64,
    ) {
        let ramp = if period > 0.0 {
            (time / period).clamp(0.0, 1.0)
        } else {
            1.0
        };
        // (DOF, previous value, ramped); the last definition of a DOF wins, as in `of_step`.
        let mut starts: Vec<(u32, f64, bool)> = Vec::new();
        for boundary in step.boundaries.iter().filter(|b| !b.fixed) {
            let ramped =
                boundary.amplitude.is_none() && step.default_amplitude == DefaultAmplitude::Ramp;
            for node in Self::nodes(model, boundary) {
                let Some(index) = model.nodes.index(node) else {
                    continue;
                };
                for d in boundary.first_dof..=boundary.last_dof {
                    let Some(dof) = (1..=6)
                        .contains(&d)
                        .then(|| dofs.dof(index, d as usize - 1))
                        .flatten()
                    else {
                        continue;
                    };
                    let from = previous_component(dofs, previous, rotation, index, d as usize - 1);
                    starts.push((dof, from, ramped));
                }
            }
        }
        starts.reverse();
        let mut seen = std::collections::HashSet::new();
        for (dof, from, ramped) in starts {
            if !seen.insert(dof) || dofs.unused.binary_search(&dof).is_ok() {
                continue;
            }
            if let Ok(pos) = self.dofs.binary_search(&dof) {
                if time <= 0.0 {
                    self.values[pos] = from;
                } else if ramped {
                    self.values[pos] += from * (1.0 - ramp);
                }
            }
        }
    }

    /// `*BOUNDARY, FIXED`: the DOFs keep the displacement they had at the end of the previous
    /// step (`previous`, global components; at nodes with a local system the local
    /// component, `rotation` gives the system), plus the value [`Spc::of_step`] gave them
    /// (zero without one): a bolt adjusted beyond its length.
    pub fn apply_fixed(
        &mut self,
        model: &Model,
        step: &Step,
        dofs: &DofMap,
        previous: &[f64],
        rotation: &dyn Fn(u32) -> Option<[[f64; 3]; 3]>,
    ) {
        for boundary in step.boundaries.iter().filter(|b| b.fixed) {
            for node in Self::nodes(model, boundary) {
                let Some(index) = model.nodes.index(node) else {
                    continue;
                };
                for d in boundary.first_dof..=boundary.last_dof {
                    let Some(dof) = (1..=6)
                        .contains(&d)
                        .then(|| dofs.dof(index, d as usize - 1))
                        .flatten()
                    else {
                        continue;
                    };
                    let value = previous_component(dofs, previous, rotation, index, d as usize - 1);
                    if let Ok(pos) = self.dofs.binary_search(&dof) {
                        self.values[pos] += value;
                    }
                }
            }
        }
    }

    pub fn is_fixed(&self, dof: u32) -> bool {
        self.dofs.binary_search(&dof).is_ok()
    }
}

/// The split of the global DOFs into free and prescribed ones, and the maps between the
/// global and the reduced (free-only) numbering.
#[derive(Clone, Debug)]
pub struct Reduction {
    /// Global numbers of the free DOFs, ascending (reduced index -> global).
    pub free: Vec<u32>,
    /// Global -> reduced index, `u32::MAX` for prescribed DOFs.
    pub to_reduced: Vec<u32>,
    pub spc: Spc,
}

impl Reduction {
    pub fn new(total: usize, spc: Spc) -> Self {
        let mut to_reduced = vec![u32::MAX; total];
        let mut free = Vec::with_capacity(total - spc.dofs.len());
        for dof in 0..total as u32 {
            if !spc.is_fixed(dof) {
                to_reduced[dof as usize] = free.len() as u32;
                free.push(dof);
            }
        }
        Self {
            free,
            to_reduced,
            spc,
        }
    }

    /// The full displacement vector with the prescribed values filled in and zeros
    /// elsewhere.
    pub fn prescribed_vector(&self, total: usize) -> Vec<f64> {
        let mut u = vec![0.0; total];
        for (&dof, &value) in self.spc.dofs.iter().zip(&self.spc.values) {
            u[dof as usize] = value;
        }
        u
    }

    /// Gathers the free entries of a full vector.
    pub fn reduce_vector(&self, full: &[f64]) -> Vec<f64> {
        self.free.iter().map(|&dof| full[dof as usize]).collect()
    }

    /// Scatters a reduced vector into the free entries of `full`.
    pub fn expand_into(&self, reduced: &[f64], full: &mut [f64]) {
        for (&dof, &value) in self.free.iter().zip(reduced) {
            full[dof as usize] = value;
        }
    }
}
