//! How loads carried over from earlier steps meet the loads a new step defines (without
//! `OP=NEW`). CalculiX's manual only says that loads "are modified" by a new definition; what
//! that means in detail was established by running ccx 2.21 on small two- and three-step
//! models (no CalculiX source was read):
//!
//! - A `*CLOAD` of a step replaces the carried load at the same node and degree of freedom;
//!   carried loads at other nodes of a set stay. Within one step loads at the same node and
//!   degree of freedom add up (also a second card after the one that replaced the carried
//!   value).
//! - A pressure `P<n>` replaces the carried pressure on the same face of the same element.
//! - `GRAV` replaces the carried gravity of the element with the same direction; a gravity in
//!   another direction is added.
//! - `CENTRIF` replaces the carried centrifugal load of the element when point and axis are
//!   the same. With another point or axis ccx keeps the carried load and drops the new one;
//!   plxs does the same and says so in a warning.
//! - Within one step every distributed load adds to the others.
//!
//! - Boundaries and loads with an amplitude in step time are frozen at their value at the
//!   end of the step they come from; `TIME=TOTAL TIME` amplitudes go on
//!   ([`freeze_step_amplitudes`]).
//!
//! Carried set loads that are touched at some of their members are split into loads on the
//! remaining single nodes or elements, so that nothing else changes.

use std::collections::{BTreeMap, HashSet};

use plxs_core::Model;
use plxs_core::mesh::{ElementId, ElementRef, NodeId, NodeRef, Sets};
use plxs_core::step::{
    Boundary, ConcentratedLoad, DistributedLoad, DistributedLoadKind, Loads, Procedure,
};

fn nodes_of(sets: &Sets, region: &NodeRef) -> Option<Vec<NodeId>> {
    match region {
        NodeRef::Node(id) => Some(vec![*id]),
        NodeRef::Set(name) => sets.node_sets.get(name).cloned(),
    }
}

fn elements_of(sets: &Sets, region: &ElementRef) -> Option<Vec<ElementId>> {
    match region {
        ElementRef::Element(id) => Some(vec![*id]),
        ElementRef::Set(name) => sets.element_sets.get(name).cloned(),
    }
}

/// Removes from the carried concentrated loads every (node, DOF) that `new` loads.
pub(crate) fn replace_concentrated(
    carried: &mut Vec<ConcentratedLoad>,
    new: &[ConcentratedLoad],
    sets: &Sets,
) {
    let mut touched: HashSet<(NodeId, u8)> = HashSet::new();
    for load in new {
        // An unknown set is reported by the validation; it touches nothing here.
        for node in nodes_of(sets, &load.region).unwrap_or_default() {
            touched.insert((node, load.dof));
        }
    }
    if touched.is_empty() {
        return;
    }
    let mut kept = Vec::with_capacity(carried.len());
    for load in carried.drain(..) {
        let Some(nodes) = nodes_of(sets, &load.region) else {
            kept.push(load);
            continue;
        };
        let hit = nodes
            .iter()
            .filter(|&&n| touched.contains(&(n, load.dof)))
            .count();
        if hit == 0 {
            kept.push(load);
        } else if hit < nodes.len() {
            for node in nodes {
                if !touched.contains(&(node, load.dof)) {
                    kept.push(ConcentratedLoad {
                        region: NodeRef::Node(node),
                        ..load.clone()
                    });
                }
            }
        }
    }
    *carried = kept;
}

/// What a distributed load replaces of a carried one on the same element.
#[derive(Clone, Debug, PartialEq)]
enum Key {
    Pressure(u8),
    Gravity([u64; 3]),
    Centrifugal { point: [u64; 3], axis: [u64; 3] },
    Other(String),
}

fn bits(v: [f64; 3]) -> [u64; 3] {
    v.map(f64::to_bits)
}

fn key(kind: &DistributedLoadKind) -> Key {
    match kind {
        DistributedLoadKind::Pressure { face, .. } => Key::Pressure(*face),
        DistributedLoadKind::Gravity { direction, .. } => Key::Gravity(bits(*direction)),
        DistributedLoadKind::Centrifugal { point, axis, .. } => Key::Centrifugal {
            point: bits(*point),
            axis: bits(*axis),
        },
        DistributedLoadKind::Other { label, .. } => Key::Other(label.clone()),
    }
}

/// Removes from the carried distributed loads what `new` replaces, and from `new` the
/// centrifugal loads CalculiX ignores (another axis than the carried one on the element).
/// Returns warnings.
pub(crate) fn replace_distributed(
    carried: &mut Vec<DistributedLoad>,
    new: &mut Vec<DistributedLoad>,
    sets: &Sets,
) -> Vec<String> {
    let mut warnings = Vec::new();
    // The carried centrifugal loads per element, before anything is replaced.
    let mut centrifugal: BTreeMap<ElementId, Vec<Key>> = BTreeMap::new();
    for load in carried.iter() {
        if matches!(load.kind, DistributedLoadKind::Centrifugal { .. }) {
            for element in elements_of(sets, &load.region).unwrap_or_default() {
                centrifugal
                    .entry(element)
                    .or_default()
                    .push(key(&load.kind));
            }
        }
    }

    // Centrifugal loads on elements with a carried one about another axis are dropped.
    let mut adjusted = Vec::with_capacity(new.len());
    for load in new.drain(..) {
        let k = key(&load.kind);
        if !matches!(k, Key::Centrifugal { .. }) {
            adjusted.push(load);
            continue;
        }
        let Some(elements) = elements_of(sets, &load.region) else {
            adjusted.push(load);
            continue;
        };
        let other_axis = |e: &ElementId| centrifugal.get(e).is_some_and(|keys| !keys.contains(&k));
        let dropped = elements.iter().filter(|e| other_axis(e)).count();
        if dropped == 0 {
            adjusted.push(load);
            continue;
        }
        warnings.push(format!(
            "*WARNING in *DLOAD: the CENTRIF load on {} has another point or axis than the \
             one carried over from an earlier step on {dropped} element(s); like CalculiX the \
             carried one is kept there and the new one ignored (use OP=NEW to replace it)",
            region_name(&load.region)
        ));
        if dropped < elements.len() {
            for element in elements {
                if !other_axis(&element) {
                    adjusted.push(DistributedLoad {
                        region: ElementRef::Element(element),
                        ..load.clone()
                    });
                }
            }
        }
    }
    *new = adjusted;

    let mut touched: HashSet<(ElementId, KeyId)> = HashSet::new();
    let mut keys: Vec<Key> = Vec::new();
    for load in new.iter() {
        let id = key_id(key(&load.kind), &mut keys);
        for element in elements_of(sets, &load.region).unwrap_or_default() {
            touched.insert((element, id));
        }
    }
    if touched.is_empty() {
        return warnings;
    }
    let mut kept = Vec::with_capacity(carried.len());
    for load in carried.drain(..) {
        let id = key_id(key(&load.kind), &mut keys);
        let Some(elements) = elements_of(sets, &load.region) else {
            kept.push(load);
            continue;
        };
        let hit = elements
            .iter()
            .filter(|&&e| touched.contains(&(e, id)))
            .count();
        if hit == 0 {
            kept.push(load);
        } else if hit < elements.len() {
            for element in elements {
                if !touched.contains(&(element, id)) {
                    kept.push(DistributedLoad {
                        region: ElementRef::Element(element),
                        ..load.clone()
                    });
                }
            }
        }
    }
    *carried = kept;
    warnings
}

/// Keys are compared by equality only (floats by their bits), so they are numbered.
type KeyId = usize;

fn key_id(k: Key, keys: &mut Vec<Key>) -> KeyId {
    match keys.iter().position(|x| *x == k) {
        Some(i) => i,
        None => {
            keys.push(k);
            keys.len() - 1
        }
    }
}

fn region_name(region: &ElementRef) -> String {
    match region {
        ElementRef::Element(id) => format!("element {id}"),
        ElementRef::Set(name) => format!("set {name}"),
    }
}

/// What carries over into the next step: boundaries and loads that follow an amplitude in
/// step time are frozen at their value at the end of the step (the amplitude is dropped),
/// those with a `TIME=TOTAL TIME` amplitude keep following it. Observed with ccx 2.21 for
/// `*BOUNDARY`, `*CLOAD` and `*DLOAD` (P, GRAV, CENTRIF): a carried load whose step-time
/// amplitude rises further does not rise in the next step.
pub(crate) fn freeze_step_amplitudes(
    model: &Model,
    step_time: f64,
    boundaries: &mut [Boundary],
    loads: &mut Loads,
) {
    // The factor of a step-time amplitude at the end of the step, `None` for no amplitude,
    // a total-time one or an unknown name (left to the validation).
    let factor = |amplitude: &Option<String>| -> Option<f64> {
        let a = model.amplitude(amplitude.as_deref()?)?;
        (!a.total_time).then(|| a.value(step_time))
    };
    for boundary in boundaries.iter_mut() {
        if let Some(f) = factor(&boundary.amplitude) {
            boundary.value = boundary.value.map(|v| v * f);
            boundary.amplitude = None;
        }
    }
    for load in &mut loads.concentrated {
        if let Some(f) = factor(&load.amplitude) {
            load.value *= f;
            load.amplitude = None;
        }
    }
    for load in &mut loads.distributed {
        let Some(f) = factor(&load.amplitude) else {
            continue;
        };
        match &mut load.kind {
            DistributedLoadKind::Pressure { value, .. } => *value *= f,
            DistributedLoadKind::Gravity { magnitude, .. } => *magnitude *= f,
            DistributedLoadKind::Centrifugal { omega_squared, .. } => *omega_squared *= f,
            // Labels the solver does not compute keep their amplitude.
            DistributedLoadKind::Other { .. } => continue,
        }
        load.amplitude = None;
    }
}

/// The step time at the end of a step: its period.
pub(crate) fn step_period(procedure: &Procedure) -> f64 {
    match procedure {
        Procedure::Static(p) => p.increments.period(),
        Procedure::Visco(p) => p.increments.period(),
        Procedure::HeatTransfer(p)
        | Procedure::CoupledTemperatureDisplacement(p)
        | Procedure::UncoupledTemperatureDisplacement(p) => p.increments.period(),
        Procedure::Dynamic(p) => p.increments.period(),
        Procedure::ModalDynamics(p) if !p.steady_state => p.period,
        _ => 1.0,
    }
}
