//! Penalty contact on the card ([`Contact`]): node-to-face and face-to-face pairs as
//! `plxs_contact` evaluates them in an explicit step, packed for `contact.wgsl`.

// Without the card only the types are used.
#![cfg_attr(not(feature = "gpu"), allow(dead_code))]

/// The pressure-overclosure relation of a pair (not tied).
#[derive(Clone, Debug, PartialEq)]
pub enum ContactLaw {
    /// `K`, `σ∞` (node-to-face only).
    Linear {
        k: f64,
        sigma_inf: f64,
    },
    Exponential {
        c0: f64,
        p0: f64,
    },
    /// `(overclosure, pressure)` sorted by overclosure.
    Tabular(Vec<(f64, f64)>),
}

/// A contact pair.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactPair {
    pub node_to_face: bool,
    pub law: ContactLaw,
    /// Coulomb friction: `μ` and the stick slope `λ`.
    pub friction: Option<(f64, f64)>,
    pub clearance: Option<f64>,
    /// The mean edge length of the master faces.
    pub typical: f64,
    /// Node-to-face springs carry no tension.
    pub compression_only: bool,
    /// The master faces are edges of plane or axisymmetric elements.
    pub edges: bool,
    pub active: bool,
}

/// A master face: its nodes (corners first), the number of corners (2 for an edge) and
/// whether it is quadratic; `planar` for an edge of a plane (1) or axisymmetric (2)
/// element, 0 otherwise.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactFace {
    pub nodes: Vec<u32>,
    pub corners: u8,
    pub quadratic: bool,
    pub planar: u8,
}

/// A slave point: a slave node (weight 1) or an integration point of a slave face, with
/// its area, the distance within which a spring exists and its gap in the reference
/// configuration (for a clearance).
#[derive(Clone, Debug, PartialEq)]
pub struct ContactPoint {
    pub pair: u32,
    pub nodes: Vec<u32>,
    pub weights: Vec<f64>,
    pub area: f64,
    pub cutoff: f64,
    pub initial_gap: f64,
}

/// The contact of a step: pairs, master faces (of all pairs), slave points (pair by
/// pair), the node coordinates (three per node, DOF `3 node + d`), the DOFs that move (a
/// mass and no prescribed motion) and the constants of the explicit integration.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Contact {
    pub pairs: Vec<ContactPair>,
    pub faces: Vec<ContactFace>,
    pub points: Vec<ContactPoint>,
    pub coordinates: Vec<f64>,
    pub moving: Vec<bool>,
    /// How far a contact node may move before the candidates are searched again.
    pub reach: f64,
    /// The springs' stiffness limit `stiffness / h²` on the masses of their nodes.
    pub stiffness: f64,
    /// The damping `damping h dp/dd` of a pressed spring.
    pub damping: f64,
    /// The rounded start of the linear relation, in typical edge lengths.
    pub rounding: f64,
    /// How far a face-to-face point may lie beside the closest face, in typical edge
    /// lengths.
    pub beside: f64,
    /// The nodes whose motion asks for a new search.
    pub checked: Vec<u32>,
}

/// The candidate master faces of every point from a search and the displacements of the
/// checked nodes then (three per node).
#[derive(Clone, Copy, Debug)]
pub struct Candidates<'a> {
    pub faces: &'a [Vec<u32>],
    pub prepared: &'a [f64],
}

/// The state of every point: status (0 open, 1 stick, 2 slip), slip, the factor on its
/// stiffness, gap and pressure.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ContactState {
    pub status: Vec<u8>,
    pub slip: Vec<[f64; 2]>,
    pub scale: Vec<f64>,
    pub gap: Vec<f64>,
    pub pressure: Vec<f64>,
}

/// Values per point in the state buffer.
pub(crate) const STATE: usize = 8;
/// Values per pair in the constants.
const PAIR: usize = 16;
/// Constants before the pairs.
const HEADER: usize = 8;
/// Values per slot: spring force, its damping, Gershgorin row.
pub(crate) const SLOT: usize = 9;

/// Offsets into the packed buffers, as `CDims` of `contact.wgsl`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "gpu", derive(bytemuck::Pod, bytemuck::Zeroable))]
pub(crate) struct CDims {
    pub points: u32,
    pub n: u32,
    pub events_from: u32,
    pub arg_words: u32,
    pub face_base: u32,
    pub point_base: u32,
    pub table_base: u32,
    pub width_base: u32,
    pub pdata_base: u32,
    pub cand_base: u32,
    pub slot_ptr_base: u32,
    pub slot_base: u32,
    pub gnodes: u32,
    pub gnode_base: u32,
    pub nslot_ptr_base: u32,
    pub nslot_base: u32,
    pub checks: u32,
    pub check_base: u32,
    pub pad0: u32,
    pub pad1: u32,
}

/// The constant data: the faces and points as integers, the constants, pairs, tables and
/// point data as reals.
pub(crate) fn pack(contact: &Contact, dims: &mut CDims) -> Result<(Vec<u32>, Vec<f64>), String> {
    let mut ints = Vec::with_capacity(10 * (contact.faces.len() + contact.points.len()));
    dims.face_base = 0;
    for face in &contact.faces {
        if face.nodes.len() > 8 || face.nodes.is_empty() {
            return Err(format!("a contact face of {} nodes", face.nodes.len()));
        }
        let mut row = [0u32; 10];
        row[..face.nodes.len()].copy_from_slice(&face.nodes);
        row[8] = face.nodes.len() as u32;
        row[9] = u32::from(face.corners)
            | (u32::from(face.quadratic) << 4)
            | (u32::from(face.planar) << 8);
        ints.extend_from_slice(&row);
    }
    dims.point_base = ints.len() as u32;
    for point in &contact.points {
        if point.nodes.len() > 8 || point.nodes.len() != point.weights.len() {
            return Err(format!("a contact point of {} nodes", point.nodes.len()));
        }
        let mut row = [0u32; 10];
        row[..point.nodes.len()].copy_from_slice(&point.nodes);
        row[8] = point.nodes.len() as u32;
        row[9] = point.pair;
        ints.extend_from_slice(&row);
    }
    let mut reals = vec![0.0; HEADER + PAIR * contact.pairs.len()];
    reals[0] = contact.reach;
    reals[1] = contact.stiffness;
    reals[2] = contact.damping;
    reals[3] = contact.rounding;
    reals[4] = contact.beside;
    let mut table = Vec::new();
    for (i, pair) in contact.pairs.iter().enumerate() {
        let row = &mut reals[HEADER + PAIR * i..HEADER + PAIR * (i + 1)];
        row[0] = f64::from(u8::from(pair.node_to_face));
        match &pair.law {
            ContactLaw::Linear { k, sigma_inf } => {
                row[1] = 0.0;
                row[2] = *k;
                row[3] = *sigma_inf;
            }
            ContactLaw::Exponential { c0, p0 } => {
                row[1] = 1.0;
                row[2] = *c0;
                row[3] = *p0;
            }
            ContactLaw::Tabular(points) => {
                row[1] = 2.0;
                row[4] = (table.len()) as f64;
                row[5] = points.len() as f64;
                for &(d, p) in points {
                    table.push(d);
                    table.push(p);
                }
            }
        }
        if let Some((mu, stick)) = pair.friction {
            row[6] = 1.0;
            row[7] = mu;
            row[8] = stick;
        }
        if let Some(c) = pair.clearance {
            row[9] = 1.0;
            row[10] = c;
        }
        row[11] = pair.typical;
        row[12] = f64::from(u8::from(pair.compression_only));
        row[13] = f64::from(u8::from(pair.edges));
        row[14] = f64::from(u8::from(pair.active));
    }
    dims.table_base = reals.len() as u32;
    reals.extend_from_slice(&table);
    dims.width_base = reals.len() as u32;
    dims.pdata_base = reals.len() as u32;
    for point in &contact.points {
        let mut row = [0.0; 11];
        row[..point.weights.len()].copy_from_slice(&point.weights);
        row[8] = point.area;
        row[9] = point.cutoff;
        row[10] = point.initial_gap;
        reals.extend_from_slice(&row);
    }
    dims.points = contact.points.len() as u32;
    Ok((ints, reals))
}

/// The lists of a search: the candidates per point, the slots per point (the nodes it can
/// act on), the gathered nodes with their slots in the order of the points, the checked
/// nodes. Returns the lists and the number of slots.
pub(crate) fn lists(
    contact: &Contact,
    candidates: &Candidates<'_>,
    dims: &mut CDims,
) -> Result<(Vec<u32>, usize), String> {
    let points = contact.points.len();
    if candidates.faces.len() != points {
        return Err(format!(
            "candidates for {} contact points, wanted {points}",
            candidates.faces.len()
        ));
    }
    if candidates.prepared.len() != 3 * contact.checked.len() {
        return Err("the positions of the checked contact nodes have the wrong length".into());
    }
    let mut out = Vec::new();
    dims.cand_base = 0;
    let mut at = 0u32;
    out.push(0);
    for faces in candidates.faces {
        if let Some(&f) = faces.iter().find(|&&f| f as usize >= contact.faces.len()) {
            return Err(format!("candidate face {f} of {}", contact.faces.len()));
        }
        at += faces.len() as u32;
        out.push(at);
    }
    for faces in candidates.faces {
        out.extend_from_slice(faces);
    }
    // The slots of every point: its nodes and those of its candidate faces.
    let mut slots: Vec<Vec<u32>> = Vec::with_capacity(points);
    for (point, faces) in contact.points.iter().zip(candidates.faces) {
        let mut nodes: Vec<u32> = point.nodes.clone();
        for &f in faces {
            nodes.extend_from_slice(&contact.faces[f as usize].nodes);
        }
        nodes.sort_unstable();
        nodes.dedup();
        slots.push(nodes);
    }
    dims.slot_ptr_base = out.len() as u32;
    let mut at = 0u32;
    out.push(0);
    for s in &slots {
        at += s.len() as u32;
        out.push(at);
    }
    dims.slot_base = out.len() as u32;
    let total = at as usize;
    // The slots of every node, in the order of the points.
    let mut by_node: std::collections::BTreeMap<u32, Vec<u32>> = Default::default();
    let mut slot = 0u32;
    for s in &slots {
        out.extend_from_slice(s);
        for &node in s {
            by_node.entry(node).or_default().push(slot);
            slot += 1;
        }
    }
    dims.gnodes = by_node.len() as u32;
    dims.gnode_base = out.len() as u32;
    out.extend(by_node.keys().copied());
    dims.nslot_ptr_base = out.len() as u32;
    let mut at = 0u32;
    out.push(0);
    for list in by_node.values() {
        at += list.len() as u32;
        out.push(at);
    }
    dims.nslot_base = out.len() as u32;
    for list in by_node.values() {
        out.extend_from_slice(list);
    }
    dims.checks = contact.checked.len() as u32;
    dims.check_base = out.len() as u32;
    out.extend_from_slice(&contact.checked);
    Ok((out, total))
}

/// The state buffer from a state (missing points: open, scale 1).
pub(crate) fn state_values(points: usize, state: Option<&ContactState>) -> Vec<f64> {
    let mut values = vec![0.0; STATE * points];
    for p in 0..points {
        let row = &mut values[STATE * p..STATE * (p + 1)];
        row[3] = 1.0;
        row[6] = -1.0;
        if let Some(s) = state {
            if let Some(&status) = s.status.get(p) {
                row[0] = f64::from(status);
            }
            if let Some(slip) = s.slip.get(p) {
                row[1] = slip[0];
                row[2] = slip[1];
            }
            if let Some(&scale) = s.scale.get(p) {
                row[3] = scale;
            }
            if let Some(&gap) = s.gap.get(p) {
                row[4] = gap;
            }
            if let Some(&pressure) = s.pressure.get(p) {
                row[5] = pressure;
            }
        }
    }
    values
}

/// The state from the state buffer.
pub(crate) fn state_of(values: &[f64]) -> ContactState {
    let points = values.len() / STATE;
    let mut state = ContactState::default();
    for p in 0..points {
        let row = &values[STATE * p..STATE * (p + 1)];
        state.status.push(row[0] as u8);
        state.slip.push([row[1], row[2]]);
        state.scale.push(row[3]);
        state.gap.push(row[4]);
        state.pressure.push(row[5]);
    }
    state
}
