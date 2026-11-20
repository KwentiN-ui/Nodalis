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
//! - The heat loads follow the same rule: a `*CFLUX` replaces the carried flux at the same
//!   node, a `*DFLUX` the carried one with the same label (`S<n>` or `BF`) on the same
//!   element, a `*FILM` the carried film on the same face, a `*RADIATE` the carried
//!   radiation on the same face. Within one step they add up.
//!
//! - Boundaries and loads with an amplitude in step time are frozen at their value at the
//!   end of the step they come from; `TIME=TOTAL TIME` amplitudes go on
//!   ([`plxs_core::step::HandedOn`]). Observed with ccx 2.21 for `*BOUNDARY`, `*CLOAD` and
//!   `*DLOAD` (P, GRAV, CENTRIF): a carried load whose step-time amplitude rises further
//!   does not rise in the next step.
//!
//! Carried set loads that are touched at some of their members are split into loads on the
//! remaining single nodes or elements, so that nothing else changes.

use std::collections::{BTreeMap, HashSet};
use std::hash::Hash;

use plxs_core::mesh::{ElementId, ElementRef, NodeId, NodeRef, Sets};
use plxs_core::step::{
    ConcentratedFlux, ConcentratedLoad, DistributedFlux, DistributedLoad, DistributedLoadKind,
    Film, FluxKind, Radiate,
};

fn nodes_of(sets: &Sets, region: &NodeRef) -> Option<Vec<NodeId>> {
    match region {
        NodeRef::Node(id) => Some(vec![*id]),
        NodeRef::Set(set) => sets.node_sets.get(set.index()).map(|s| s.nodes.clone()),
    }
}

fn elements_of(sets: &Sets, region: &ElementRef) -> Option<Vec<ElementId>> {
    match region {
        ElementRef::Element(id) => Some(vec![*id]),
        // A set without elements: the name of point masses.
        ElementRef::Set(set) => (sets.element_sets.get(set.index()))
            .map(|s| s.elements.clone())
            .filter(|elements| !elements.is_empty()),
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

/// Removes from `carried` every (member, key) that a load of `new` covers. A carried set
/// load hit at some of its members is split into single-member loads for the others.
fn replace_keyed<T: Clone, M: Copy + Eq + Hash, K: Copy + Eq + Hash>(
    carried: &mut Vec<T>,
    new: &[T],
    members: impl Fn(&T) -> Option<Vec<M>>,
    key: impl Fn(&T) -> K,
    single: impl Fn(&T, M) -> T,
) {
    let mut touched: HashSet<(M, K)> = HashSet::new();
    for load in new {
        // An unknown set is reported by the validation; it touches nothing here.
        for m in members(load).unwrap_or_default() {
            touched.insert((m, key(load)));
        }
    }
    if touched.is_empty() {
        return;
    }
    let mut kept = Vec::with_capacity(carried.len());
    for load in carried.drain(..) {
        let Some(all) = members(&load) else {
            kept.push(load);
            continue;
        };
        let k = key(&load);
        let hit = all.iter().filter(|&&m| touched.contains(&(m, k))).count();
        if hit == 0 {
            kept.push(load);
        } else if hit < all.len() {
            for m in all {
                if !touched.contains(&(m, k)) {
                    kept.push(single(&load, m));
                }
            }
        }
    }
    *carried = kept;
}

/// `*CFLUX`: replaces the carried flux at the same nodes.
pub(crate) fn replace_concentrated_flux(
    carried: &mut Vec<ConcentratedFlux>,
    new: &[ConcentratedFlux],
    sets: &Sets,
) {
    replace_keyed(
        carried,
        new,
        |l| nodes_of(sets, &l.region),
        |_| (),
        |l, node| ConcentratedFlux {
            region: NodeRef::Node(node),
            ..l.clone()
        },
    );
}

/// `*DFLUX`: replaces the carried flux with the same label on the same elements.
pub(crate) fn replace_distributed_flux(
    carried: &mut Vec<DistributedFlux>,
    new: &[DistributedFlux],
    sets: &Sets,
) {
    replace_keyed(
        carried,
        new,
        |l| elements_of(sets, &l.region),
        |l| match l.kind {
            FluxKind::Body => 0u16,
            FluxKind::Face(face) => 1 + face as u16,
        },
        |l, element| DistributedFlux {
            region: ElementRef::Element(element),
            ..l.clone()
        },
    );
}

/// `*FILM`: replaces the carried film on the same faces.
pub(crate) fn replace_films(carried: &mut Vec<Film>, new: &[Film], sets: &Sets) {
    replace_keyed(
        carried,
        new,
        |l| elements_of(sets, &l.region),
        |l| l.face,
        |l, element| Film {
            region: ElementRef::Element(element),
            ..l.clone()
        },
    );
}

/// `*RADIATE`: replaces the carried radiation on the same faces.
pub(crate) fn replace_radiations(carried: &mut Vec<Radiate>, new: &[Radiate], sets: &Sets) {
    replace_keyed(
        carried,
        new,
        |l| elements_of(sets, &l.region),
        |l| (l.face, l.cavity),
        |l, element| Radiate {
            region: ElementRef::Element(element),
            ..l.clone()
        },
    );
}

/// What a distributed load replaces of a carried one on the same element.
#[derive(Clone, Debug, PartialEq)]
enum Key {
    Pressure(u8),
    Gravity([u64; 3]),
    Centrifugal { point: [u64; 3], axis: [u64; 3] },
    ShellPressure,
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
        DistributedLoadKind::ShellPressure { .. } => Key::ShellPressure,
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
            region_name(sets, &load.region)
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
    let named = new.iter().any(|l| elements_of(sets, &l.region).is_none());
    if touched.is_empty() && !named {
        return warnings;
    }
    let mut kept = Vec::with_capacity(carried.len());
    for load in carried.drain(..) {
        let id = key_id(key(&load.kind), &mut keys);
        let Some(elements) = elements_of(sets, &load.region) else {
            // A load on the name of point masses (no element set) is replaced by a new one
            // of the same kind on the same name.
            let replaced =
                (new.iter()).any(|n| n.region == load.region && key(&n.kind) == key(&load.kind));
            if !replaced {
                kept.push(load);
            }
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

fn region_name(sets: &Sets, region: &ElementRef) -> String {
    match region {
        ElementRef::Element(id) => format!("element {id}"),
        ElementRef::Set(set) => format!("set {}", sets.element_set_name(*set)),
    }
}
