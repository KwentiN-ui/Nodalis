//! Contact of the Nodalis solver (concept section 4.4): penalty contact between a slave
//! surface and a master surface, node-to-face (`TYPE=NODE TO SURFACE`: a spring per slave
//! node) and face-to-face (`TYPE=SURFACE TO SURFACE`: a spring per integration point of the
//! slave faces), with the pressure-overclosure relations of `*SURFACE BEHAVIOR`, Coulomb
//! friction with a stick slope (`*FRICTION`), `*CLEARANCE`, `ADJUST`, `*MODEL CHANGE` and
//! `*CHANGE FRICTION`, and the outputs `CDIS`, `CSTR`, `CELS` and `*CONTACT PRINT`.
//!
//! How it works: [`ContactModel::new`] prepares every pair once (faces, slave points with
//! their areas, candidate master faces, initial gaps; `ADJUST` moves slave nodes on to the
//! master surface). Each Newton iteration, [`ContactModel::evaluate`] projects every slave
//! point on to the closest master face in the current configuration, evaluates the
//! relation and the friction and returns the contact elements (DOFs, tangent, internal
//! force) and the trial state of every point; the procedure assembles them with the finite
//! elements and stores the trial states when the increment converged. The tangent is the
//! symmetric part of the consistent one (the coupling of sliding friction to the normal
//! pressure is left out so that the LDLᵀ solver applies; convergence is then linear in the
//! sliding points instead of quadratic).
//!
//! Sources: Wriggers, "Computational Contact Mechanics" (node-to-segment penalty elements,
//! regularised Coulomb friction with return mapping, chapters 6 and 9); the CalculiX manual
//! (`*CONTACT PAIR`, `*SURFACE BEHAVIOR`, `*FRICTION`, `*CLEARANCE`, "Node-to-Face Penalty
//! Contact", "Face-to-Face Penalty Contact"); runs of ccx 2.21 as a black box for the
//! relations and defaults ([`law`]) and the integration of slave faces ([`geometry`]).
//! No CalculiX code was consulted.

pub mod geometry;
pub mod law;
pub mod output;

use std::collections::HashSet;

use plxs_core::contact::{Adjust, ContactKind};
use plxs_core::state::{ContactPointState, ContactStatus};
use plxs_core::step::ContactChange;
use plxs_core::{Error, Model, Result, State, Step};

use geometry::{ContactFace, FaceGrid, cross, dot, normalized, scale, sub, surface_nodes};
use law::{FrictionLaw, Law};

/// How far (in typical master edge lengths) a face-to-face point may lie beside the closest
/// master face and still touch it: points over a convex edge between two faces project
/// onto the edge with a small sideways offset.
const BESIDE_TOLERANCE: f64 = 0.01;

/// A point of the slave side a spring acts on: a slave node (node-to-face) or an integration
/// point of a slave face (face-to-face).
#[derive(Clone, Debug)]
pub struct SlavePoint {
    /// Dense node indices and their shape function values at the point (a node: itself, 1).
    pub nodes: Vec<u32>,
    pub weights: Vec<f64>,
    /// Area attributed to the point: the consistent nodal area of a slave node, the
    /// integration weight times the Jacobian of a face point.
    pub area: f64,
    /// Slave face (index into [`Pair::slave_faces`]) and natural coordinates, `None` for a
    /// node.
    pub face: Option<(u32, [f64; 2])>,
}

/// A contact pair prepared for the solver.
#[derive(Clone, Debug)]
pub struct Pair {
    /// Index into `model.contact.pairs`.
    pub index: usize,
    pub kind: ContactKind,
    /// `TYPE=MORTAR`: face-to-face integration with a stiff penalty in place of the
    /// Lagrange multipliers, nodal results as mortar (dual-basis) nodal values.
    pub mortar: bool,
    pub slave_name: String,
    pub master_name: String,
    pub interaction: String,
    pub law: Law,
    /// Friction as defined with the interaction and as currently in force (`*CHANGE
    /// FRICTION`).
    pub base_friction: Option<FrictionLaw>,
    pub friction: Option<FrictionLaw>,
    pub clearance: Option<f64>,
    pub slave_faces: Vec<ContactFace>,
    pub master: Vec<ContactFace>,
    pub points: Vec<SlavePoint>,
    /// Distance within which a node-to-face spring exists, per point.
    pub cutoff: Vec<f64>,
    /// The gap of every point in the reference configuration (after `ADJUST`), `None`
    /// without a master face nearby.
    pub initial: Vec<Option<(u32, f64)>>,
    /// Mortar: the slave nodes and, per node, its integration points with the slave shape
    /// function value there (empty for other pairs and tied mortar contact).
    pub mortar_nodes: Vec<(u32, Vec<(u32, f64)>)>,
    /// Whether the pair is in force (`*MODEL CHANGE`).
    pub active: bool,
    /// Mean edge length of the master faces.
    pub typical: f64,
    /// Node-to-face springs carry compression only (set for dynamic steps by
    /// [`ContactModel::begin_step`]): the tension CalculiX's smoothed relation keeps within
    /// the cutoff would pull a body towards a surface before it arrives and hold it back
    /// as it bounces off.
    pub compression_only: bool,
    /// Candidate master faces per point, from the last [`ContactModel::prepare`].
    candidates: Vec<Vec<u32>>,
}

/// The result of one point in an evaluation.
#[derive(Clone, Debug)]
pub struct PointResult {
    /// What goes into the state: status, gap, pressure, plastic slip (local basis).
    pub state: ContactPointState,
    pub normal: [f64; 3],
    pub tangents: [[f64; 3]; 2],
    /// Total relative tangential displacement (local basis).
    pub tangential: [f64; 2],
    /// Tangential traction (local basis).
    pub shear: [f64; 2],
    /// Spring energy of the point (area included).
    pub energy: f64,
    /// The part of `energy` stored by the normal spring (without the friction's stick).
    pub normal_energy: f64,
    /// Force on the slave point.
    pub force: [f64; 3],
    pub master_face: Option<u32>,
    pub area: f64,
    /// Current position of the point.
    pub position: [f64; 3],
    /// Whether a spring element exists for the point.
    pub element: bool,
}

/// The geometry of a slave point (or a mortar node) against the master: the relative
/// displacement is `Σ c_i u_i` over `nodes`.
#[derive(Clone, Debug)]
struct Kinematics {
    nodes: Vec<u32>,
    coefficients: Vec<f64>,
    normal: [f64; 3],
    /// First tangent of the master face at the foot point (not yet orthogonalised).
    tangent: [f64; 3],
    geometric_gap: f64,
    initial_gap: f64,
    /// Gap with the clearance applied.
    gap: f64,
    relative: [f64; 3],
    position: [f64; 3],
    master_face: u32,
    area: f64,
    cutoff: f64,
}

/// A contact spring element: global DOFs, symmetric tangent (row-major) and internal force.
#[derive(Clone, Debug)]
pub struct ContactElement {
    pub dofs: Vec<u32>,
    /// The symmetric part of the tangent (row-major, `dofs.len()²`).
    pub ke: Vec<f64>,
    pub fe: Vec<f64>,
    /// The unsymmetric rest of the consistent tangent (sliding friction: the shear follows
    /// the pressure, `μ dp/dd`), row-major like `ke`; empty when there is none.
    pub unsymmetric: Vec<f64>,
    /// The part of `fe` from the damping of the spring ([`Damping`]); empty without.
    pub damping: Vec<f64>,
    /// The part of `fe` from the normal spring alone (no damping, no friction); empty for
    /// tied contact.
    pub normal: Vec<f64>,
}

/// Damping of the normal springs in a dynamic step: the pressure `p + c ḋ` with the rate of
/// overclosure `ḋ` and `c = coefficient · dp/dd`, never below zero (it does not pull a
/// surface back as it lifts off). Only points that were closed at the start of the
/// increment are damped: the force of a point that closes stays continuous, and the
/// iterations of an increment never switch the damping on or off.
#[derive(Clone, Debug, Default)]
pub struct Damping {
    /// The velocities of all DOFs at the displacements of the evaluation.
    pub velocities: Vec<f64>,
    /// `c / (dp/dd)`, a time.
    pub coefficient: f64,
    /// `dv/du` of the time integration, for the tangent.
    pub rate: f64,
}

impl ContactElement {
    /// `out += E v` for the unsymmetric rest `E` (`v`, `out` in global numbering).
    pub fn apply_unsymmetric(&self, v: &[f64], out: &mut [f64]) {
        let m = self.dofs.len();
        if self.unsymmetric.is_empty() {
            return;
        }
        for (a, &i) in self.dofs.iter().enumerate() {
            let row = &self.unsymmetric[a * m..(a + 1) * m];
            out[i as usize] += row
                .iter()
                .zip(&self.dofs)
                .map(|(e, &j)| e * v[j as usize])
                .sum::<f64>();
        }
    }
}

/// What an evaluation produced.
#[derive(Clone, Debug, Default)]
pub struct Evaluation {
    pub elements: Vec<ContactElement>,
    /// Per pair, per point.
    pub points: Vec<Vec<PointResult>>,
    /// Number of springs.
    pub active: usize,
}

impl Evaluation {
    /// The status of every point, to tell a change of the contact set between iterations.
    pub fn status_key(&self) -> Vec<u8> {
        self.points
            .iter()
            .flatten()
            .map(|p| match (p.element, p.state.status) {
                (false, _) => 0,
                (true, ContactStatus::Open) => 1,
                (true, ContactStatus::Stick) => 2,
                (true, ContactStatus::Slip) => 3,
            })
            .collect()
    }

    /// The states of all points for [`State::contact`].
    pub fn states(&self) -> Vec<Vec<ContactPointState>> {
        self.points
            .iter()
            .map(|pair| pair.iter().map(|p| p.state).collect())
            .collect()
    }
}

/// All contact pairs of a model, prepared once.
#[derive(Clone, Debug, Default)]
pub struct ContactModel {
    pub pairs: Vec<Pair>,
    /// Warnings of the preparation.
    pub warnings: Vec<String>,
    /// The last evaluation (for the outputs of a converged increment).
    pub last: Option<Evaluation>,
    /// Damping of the springs for the next evaluations (dynamic steps), `None` otherwise.
    pub damping: Option<Damping>,
}

/// Young's modulus of the first material (CalculiX's reference for defaults).
fn first_young(model: &Model) -> f64 {
    model
        .materials
        .iter()
        .filter_map(|m| plxs_materials::LinearElastic::of(m, 0.0).ok())
        .map(|e| e.young)
        .next()
        .unwrap_or(0.0)
}

/// Young's modulus of the material of an element (dense index).
fn element_young(model: &Model, element: u32) -> Option<f64> {
    let e = model.elements.get(element as usize)?;
    let section = model.sections.get(e.section? as usize)?;
    let name = match &section.kind {
        plxs_core::section::SectionKind::Solid { material, .. } => material,
        plxs_core::section::SectionKind::Beam { material, .. } => material,
        plxs_core::section::SectionKind::Shell { material, .. } => material,
        _ => return None,
    };
    let material = model.material(name)?;
    plxs_materials::LinearElastic::of(material, 0.0)
        .ok()
        .map(|e| e.young)
}

impl ContactModel {
    /// Prepares the contact pairs of a model; `ADJUST` moves slave nodes.
    pub fn new(model: &mut Model) -> Result<ContactModel> {
        let mut out = ContactModel::default();
        if model.contact.pairs.is_empty() {
            return Ok(out);
        }
        let young_default = first_young(model);
        for (index, pair) in model.contact.pairs.iter().enumerate() {
            let keyword = "*CONTACT PAIR";
            let interaction = model
                .contact
                .interactions
                .iter()
                .find(|i| i.name == pair.interaction)
                .ok_or_else(|| {
                    Error::undefined(keyword, "surface interaction", &pair.interaction)
                })?;
            let master = ContactFace::surface_faces(model, keyword, &pair.master)?;
            if master.is_empty() {
                return Err(Error::input(
                    keyword,
                    format!("master surface {} has no faces", pair.master),
                ));
            }
            let coords = model.nodes.coordinates.clone();
            let typical =
                master.iter().map(|f| f.edge_length(&coords)).sum::<f64>() / master.len() as f64;

            // Slave side.
            let is_node_surface = matches!(
                model.sets.surfaces.get(&pair.slave),
                Some(plxs_core::mesh::Surface::Node(_))
            );
            let slave_faces = if is_node_surface {
                let nodes = surface_nodes(model, keyword, &pair.slave)?;
                ContactFace::faces_of_node_set(model, &nodes)
            } else {
                ContactFace::surface_faces(model, keyword, &pair.slave)?
            };
            let young = slave_faces
                .first()
                .and_then(|f| element_young(model, f.element))
                .or_else(|| master.first().and_then(|f| element_young(model, f.element)))
                .unwrap_or(young_default);
            let mortar = matches!(pair.kind, ContactKind::Mortar | ContactKind::LinearMortar);
            let law = if mortar {
                Law::resolve_mortar(interaction.behavior.as_ref(), young, typical)
            } else {
                Law::resolve(interaction.behavior.as_ref(), young)
            };
            let friction = FrictionLaw::resolve(interaction.friction.as_ref(), young_default);
            if law.is_tied() && friction.is_none() {
                out.warnings.push(format!(
                    "*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=TIED of interaction {} has no *FRICTION; the stick slope E/2 is used",
                    interaction.name
                ));
            }
            let friction = if law.is_tied() && friction.is_none() {
                Some(FrictionLaw {
                    mu: 0.0,
                    stick: 0.5 * young_default,
                })
            } else {
                friction
            };
            let c0 = match interaction.behavior.as_ref() {
                Some(plxs_core::contact::SurfaceBehavior::Linear { c0, .. }) => *c0,
                _ => None,
            };

            let mut points = Vec::new();
            let mut cutoff = Vec::new();
            match pair.kind {
                ContactKind::NodeToSurface => {
                    let nodes = surface_nodes(model, keyword, &pair.slave)?;
                    // Consistent nodal areas from the slave faces.
                    let mut area = vec![0.0; model.nodes.len()];
                    for face in &slave_faces {
                        for (n, a) in face.nodes.iter().zip(face.nodal_areas(&coords)) {
                            area[*n as usize] += a;
                        }
                    }
                    if is_node_surface && slave_faces.is_empty() {
                        out.warnings.push(format!(
                            "node surface {} has no exterior faces; unit areas are used for its contact springs",
                            pair.slave
                        ));
                    }
                    for n in nodes {
                        let a = if area[n as usize] > 0.0 {
                            area[n as usize]
                        } else {
                            1.0
                        };
                        points.push(SlavePoint {
                            nodes: vec![n],
                            weights: vec![1.0],
                            area: a,
                            face: None,
                        });
                        cutoff.push(law.node_cutoff(c0, a));
                    }
                }
                _ => {
                    for (f, face) in slave_faces.iter().enumerate() {
                        let (natural, weights) = face.integration();
                        for (p, w) in natural.iter().zip(&weights) {
                            let fp = face.at(&coords, *p);
                            points.push(SlavePoint {
                                nodes: face.nodes.clone(),
                                weights: fp.n.clone(),
                                area: w * fp.jacobian,
                                face: Some((f as u32, *p)),
                            });
                            // Face-to-face springs act on overclosure only; the cutoff only
                            // decides which tied points are coupled.
                            cutoff.push(law.node_cutoff(c0, w * fp.jacobian));
                        }
                    }
                }
            }
            let clearance = model
                .contact
                .clearances
                .iter()
                .find(|c| c.slave == pair.slave && c.master == pair.master)
                .map(|c| c.value);
            // CalculiX applies *CLEARANCE to face-to-face contact only (observed: a
            // node-to-face pair with a clearance behaves as without one).
            let clearance = if pair.kind == ContactKind::NodeToSurface && clearance.is_some() {
                out.warnings.push(format!(
                    "*CLEARANCE of contact pair {} / {} is ignored: it applies to face-to-face contact only",
                    pair.slave, pair.master
                ));
                None
            } else {
                clearance
            };
            let mut prepared = Pair {
                index,
                kind: pair.kind,
                mortar,
                slave_name: pair.slave.clone(),
                master_name: pair.master.clone(),
                interaction: pair.interaction.clone(),
                law,
                base_friction: friction,
                friction,
                clearance,
                slave_faces,
                master,
                points,
                cutoff,
                initial: Vec::new(),
                mortar_nodes: Vec::new(),
                active: true,
                typical,
                compression_only: false,
                candidates: Vec::new(),
            };

            if mortar && !prepared.law.is_tied() {
                let mut by_node: std::collections::BTreeMap<u32, Vec<(u32, f64)>> =
                    std::collections::BTreeMap::new();
                // Dual shape functions per slave face (Wohlmuth): ψ_k = Σ_j A_kj N_j with
                // A = D M⁻¹, D = diag ∫N, M = ∫N Nᵀ, so that ∫ψ_k N_j = δ_kj ∫N_k: the slave
                // side of a node's weighted gap is the node itself.
                let mut start = 0;
                while start < prepared.points.len() {
                    let face = prepared.points[start].face.map(|(f, _)| f);
                    let end = start
                        + prepared.points[start..]
                            .iter()
                            .take_while(|q| q.face.map(|(f, _)| f) == face)
                            .count();
                    let m = prepared.points[start].nodes.len();
                    let mut mass = vec![0.0; m * m];
                    let mut diag = vec![0.0; m];
                    for q in &prepared.points[start..end] {
                        for i in 0..m {
                            diag[i] += q.weights[i] * q.area;
                            for j in 0..m {
                                mass[i * m + j] += q.weights[i] * q.weights[j] * q.area;
                            }
                        }
                    }
                    let dual = geometry::dual_coefficients(&mass, &diag, m);
                    for (p, q) in prepared.points.iter().enumerate().take(end).skip(start) {
                        for (k, &node) in q.nodes.iter().enumerate() {
                            let psi: f64 = (0..m).map(|j| dual[k * m + j] * q.weights[j]).sum();
                            by_node.entry(node).or_default().push((p as u32, psi));
                        }
                    }
                    start = end;
                }
                prepared.mortar_nodes = by_node.into_iter().collect();
            }

            // ADJUST: slave nodes within the distance are moved on to the master surface.
            let adjust_distance = match &pair.adjust {
                Some(Adjust::Distance(d)) => Some((*d, None)),
                Some(Adjust::NodeSet(set)) => Some((
                    f64::INFINITY,
                    Some(model.nodes_of(&plxs_core::mesh::NodeRef::Set(set.clone()))),
                )),
                None => None,
            };
            if let Some((distance, set)) = adjust_distance {
                let allowed: Option<HashSet<u32>> =
                    set.map(|ids| ids.iter().filter_map(|id| model.nodes.index(*id)).collect());
                let slave_nodes = surface_nodes(model, keyword, &pair.slave)?;
                let master_nodes: HashSet<u32> = prepared
                    .master
                    .iter()
                    .flat_map(|f| f.nodes.iter().copied())
                    .collect();
                let grid = FaceGrid::new(
                    &prepared.master,
                    &coords,
                    typical.max(distance.min(1e6)),
                    distance.min(1e6),
                );
                let mut moved = 0;
                for n in slave_nodes {
                    if master_nodes.contains(&n)
                        || allowed.as_ref().is_some_and(|a| !a.contains(&n))
                    {
                        continue;
                    }
                    let x = coords[n as usize];
                    let mut best: Option<(f64, [f64; 3])> = None;
                    for &f in grid.candidates(x) {
                        let (_, fp) = prepared.master[f as usize].project(&coords, x);
                        let d = geometry::norm(sub(x, fp.x));
                        if best.is_none_or(|b| d < b.0) {
                            best = Some((d, fp.x));
                        }
                    }
                    if let Some((d, y)) = best
                        && d <= distance
                        && d > 0.0
                    {
                        model.nodes.coordinates[n as usize] = y;
                        moved += 1;
                    }
                }
                if moved > 0 {
                    out.warnings.push(format!(
                        "contact pair {} / {}: {moved} slave nodes adjusted on to the master surface",
                        pair.slave, pair.master
                    ));
                }
            }
            // Initial gaps in the (adjusted) reference configuration.
            let coords = &model.nodes.coordinates;
            prepared.prepare_candidates(coords);
            let initial: Vec<Option<(u32, f64)>> = (0..prepared.points.len())
                .map(|p| {
                    prepared
                        .closest(p, coords)
                        .map(|(f, _, fp, x)| (f, dot(sub(x, fp.x), fp.normal)))
                })
                .collect();
            prepared.initial = initial;
            out.pairs.push(prepared);
        }
        Ok(out)
    }

    pub fn is_empty(&self) -> bool {
        self.pairs.is_empty()
    }

    /// Whether any pair is in force.
    pub fn is_active(&self) -> bool {
        self.pairs.iter().any(|p| p.active)
    }

    /// Applies `*MODEL CHANGE` and `*CHANGE FRICTION` of a step; the pairs switched off are
    /// kept in `state.inactive_pairs`. In a dynamic step node-to-face springs carry
    /// compression only ([`Pair::compression_only`]).
    pub fn begin_step(&mut self, step: &Step, state: &mut State) -> Vec<String> {
        let mut warnings = Vec::new();
        let dynamic = matches!(step.procedure, plxs_core::step::Procedure::Dynamic(_));
        for pair in &mut self.pairs {
            pair.active = !state.inactive_pairs.contains(&(pair.index as u32));
            pair.compression_only = dynamic;
        }
        for change in &step.contact_changes {
            match change {
                ContactChange::ContactPair { add, slave, master } => {
                    let mut found = false;
                    for pair in &mut self.pairs {
                        if &pair.slave_name == slave && &pair.master_name == master {
                            pair.active = *add;
                            found = true;
                            let index = pair.index as u32;
                            state.inactive_pairs.retain(|&i| i != index);
                            if !*add {
                                state.inactive_pairs.push(index);
                            }
                        }
                    }
                    if !found {
                        warnings.push(format!(
                            "*MODEL CHANGE: no contact pair with slave {slave} and master {master}"
                        ));
                    }
                }
                ContactChange::Friction {
                    interaction,
                    friction,
                } => {
                    let mut found = false;
                    for pair in &mut self.pairs {
                        if &pair.interaction == interaction {
                            let young = pair.base_friction.map_or(0.0, |f| 2.0 * f.stick);
                            pair.friction = FrictionLaw::resolve(Some(friction), young);
                            found = true;
                        }
                    }
                    if !found {
                        warnings.push(format!(
                            "*CHANGE FRICTION: no contact pair uses interaction {interaction}"
                        ));
                    }
                }
                ContactChange::Elements { .. } => {
                    warnings.push("*MODEL CHANGE, TYPE=ELEMENT is not implemented yet".into());
                }
            }
        }
        // The state carries one entry per point of every pair.
        if state.contact.len() != self.pairs.len()
            || state
                .contact
                .iter()
                .zip(&self.pairs)
                .any(|(s, p)| s.len() != p.points.len())
        {
            state.contact = self
                .pairs
                .iter()
                .map(|p| vec![ContactPointState::default(); p.points.len()])
                .collect();
        }
        warnings
    }

    /// Finds the candidate master faces of every point for the displacements `u` and
    /// returns the DOF blocks of all possible springs (the matrix pattern's contact part).
    pub fn prepare(&mut self, coords: &[[f64; 3]], u: &[f64]) -> Vec<Vec<u32>> {
        let current = current_configuration(coords, u);
        let mut blocks = Vec::new();
        for pair in &mut self.pairs {
            if !pair.active {
                continue;
            }
            pair.prepare_candidates(&current);
            if pair.kind == ContactKind::NodeToSurface && !pair.law.is_tied() {
                // Node-to-face: a slave node whose projection at the start of the increment
                // falls beside the master surface gets no spring in this increment
                // (observed: ccx drops a node that slid past the master's edge in an
                // earlier step, and keeps one that slides off within the increment).
                for p in 0..pair.points.len() {
                    let beside = pair.closest(p, &current).is_none_or(|(f, natural, ..)| {
                        !pair.master[f as usize].contains(natural, 1e-3)
                    });
                    if beside {
                        pair.candidates[p].clear();
                    }
                }
            }
            let point_dofs = |p: usize| {
                let mut dofs: Vec<u32> = Vec::new();
                for &n in &pair.points[p].nodes {
                    dofs.extend((0..3).map(|d| n * 3 + d));
                }
                for &f in &pair.candidates[p] {
                    for &n in &pair.master[f as usize].nodes {
                        dofs.extend((0..3).map(|d| n * 3 + d));
                    }
                }
                dofs
            };
            if pair.mortar_nodes.is_empty() {
                // A point's spring couples it with one master face: a block per candidate
                // face (not one over all candidates, which would couple the master faces
                // among each other and fill the factor for nothing).
                for p in 0..pair.points.len() {
                    for &f in &pair.candidates[p] {
                        blocks.push(pair.point_face_dofs(&[p as u32], f));
                    }
                }
            } else {
                // A mortar node couples all its integration points.
                for (_, points) in &pair.mortar_nodes {
                    let mut dofs: Vec<u32> = points
                        .iter()
                        .flat_map(|&(p, _)| point_dofs(p as usize))
                        .collect();
                    dofs.sort_unstable();
                    dofs.dedup();
                    blocks.push(dofs);
                }
            }
        }
        // The integration points of one slave face share their blocks.
        blocks.sort_unstable();
        blocks.dedup();
        blocks
    }

    /// Evaluates all springs at the displacements `u` from the states `start` (plastic
    /// slips at the start of the increment).
    pub fn evaluate(
        &self,
        coords: &[[f64; 3]],
        u: &[f64],
        start: &[Vec<ContactPointState>],
        predictor: bool,
    ) -> Evaluation {
        let mut evaluation = Evaluation::default();
        let current = current_configuration(coords, u);
        for (pi, pair) in self.pairs.iter().enumerate() {
            let count = if pair.mortar_nodes.is_empty() {
                pair.points.len()
            } else {
                pair.mortar_nodes.len()
            };
            let mut results = Vec::with_capacity(count);
            for p in 0..count {
                let previous = start
                    .get(pi)
                    .and_then(|s| s.get(p))
                    .copied()
                    .unwrap_or_default();
                let (result, element) = if pair.mortar_nodes.is_empty() {
                    pair.evaluate_point(p, &current, u, &previous, predictor, self.damping.as_ref())
                } else {
                    pair.evaluate_mortar_node(
                        p,
                        &current,
                        u,
                        &previous,
                        predictor,
                        self.damping.as_ref(),
                    )
                };
                if let Some(element) = element {
                    evaluation.elements.push(element);
                    if result.element {
                        evaluation.active += 1;
                    }
                }
                results.push(result);
            }
            evaluation.points.push(results);
        }
        evaluation
    }
}

/// The overclosure (in typical master edge lengths) over which the linear relation of a
/// dynamic step starts rounded ([`Law::rounded_pressure`]).
pub const ROUNDING: f64 = 1e-5;

/// How deep (in typical master edge lengths) one Newton correction may push a contact
/// point into the master before the correction is shortened (see
/// [`ContactModel::step_limit`]).
pub const PENETRATION_STEP_LIMIT: f64 = 0.1;

impl ContactModel {
    /// The factor (in `(0, 1]`) by which a Newton correction `du` from the displacements `u`
    /// is shortened when it drives contact points deep into the master: more than
    /// [`PENETRATION_STEP_LIMIT`] typical master edge lengths where they were not that deep
    /// already. `work` is `du · r` with the residual force `r` the correction was solved
    /// for. `start` are the contact states at the start of the increment.
    ///
    /// A body that is held only by weak springs until it touches (a ball pressed onto a
    /// plate by a force) moves through the master by its rigid-body displacement under the
    /// full load in the first iteration: the predictor stiffness only reaches the points
    /// within the cutoff, a fraction of a micrometre. All points within reach are then
    /// deeply closed and the contact set shrinks by a few points per iteration, more than
    /// the iteration limit allows, and a smaller increment changes nothing.
    ///
    /// Along such a correction the structure takes the force `(1 - α) r` (it is linear in
    /// the correction, which was solved without the contact), the contact the rest. The
    /// step stops where the contact force along `du` balances it, a line search on the
    /// contact alone: the contact then starts about at the size the load gives it. The
    /// overclosure limit bounds the step in any case.
    pub fn step_limit(
        &self,
        coords: &[[f64; 3]],
        u: &[f64],
        du: &[f64],
        start: &[Vec<ContactPointState>],
        work: f64,
    ) -> f64 {
        let at =
            |alpha: f64| -> Vec<f64> { u.iter().zip(du).map(|(a, b)| a + alpha * b).collect() };
        let current = current_configuration(coords, u);
        let mut cap = 1.0_f64;
        for pair in &self.pairs {
            if !pair.active || pair.law.is_tied() || pair.typical <= 0.0 {
                continue;
            }
            let limit = PENETRATION_STEP_LIMIT * pair.typical;
            // Cheap test first: a correction smaller than the limit cannot overclose by
            // more.
            let moves = |nodes: &[u32]| {
                nodes
                    .iter()
                    .any(|&n| (0..3).any(|d| du[n as usize * 3 + d].abs() > limit))
            };
            if !pair.points.iter().any(|point| moves(&point.nodes))
                && !pair.master.iter().any(|face| moves(&face.nodes))
            {
                continue;
            }
            for p in 0..pair.points.len() {
                // The gap after the correction, linearised at the current position against
                // the master face of now: `Σ c_i du_i` along its normal. Not the gap found
                // anew at the end of the correction: after a move of metres the point lies
                // beside every master face it could be projected on (the master moved, for
                // a curved master as the slave of a flat surface), and the deep overclosure
                // would go unseen.
                let Ok(kin) = pair.kinematics(p, &current, u) else {
                    continue;
                };
                let mut change = 0.0;
                for (&node, &c) in kin.nodes.iter().zip(&kin.coefficients) {
                    for d in 0..3 {
                        change += c * du[node as usize * 3 + d] * kin.normal[d];
                    }
                }
                let (before, after) = (kin.gap, kin.gap + change);
                if -after <= limit || -before >= limit {
                    continue;
                }
                // The gap changes about linearly with the length of the step.
                let f = (before + limit) / (before - after);
                cap = cap.min(f);
            }
        }
        if cap >= 1.0 || work <= 0.0 || !work.is_finite() {
            return cap;
        }
        // The work of the contact forces along du, relative to the start of the step.
        let contact_work = |alpha: f64| -> f64 {
            let v = at(alpha);
            let evaluation = self.evaluate(coords, &v, start, false);
            evaluation
                .elements
                .iter()
                .map(|e| {
                    e.dofs
                        .iter()
                        .zip(&e.fe)
                        .map(|(&d, f)| du[d as usize] * f)
                        .sum::<f64>()
                })
                .sum()
        };
        let base = contact_work(0.0);
        let unbalanced = |alpha: f64| (1.0 - alpha) * work - (contact_work(alpha) - base);
        if unbalanced(cap) >= 0.0 {
            return cap;
        }
        // Bisection: the contact force grows with the overclosure, the rest falls.
        let (mut low, mut high) = (0.0, cap);
        for _ in 0..40 {
            let mid = 0.5 * (low + high);
            if unbalanced(mid) > 0.0 {
                low = mid;
            } else {
                high = mid;
            }
            if high - low <= 1e-3 * high {
                break;
            }
        }
        high
    }
}

/// Heat conduction across a closed contact point: the temperature difference `Σ c_i T_i`
/// (slave point minus master foot point) over the point's area, for `*GAP CONDUCTANCE`.
#[derive(Clone, Debug)]
pub struct ThermalLink {
    /// Index into `model.contact.pairs`.
    pub pair: usize,
    pub nodes: Vec<u32>,
    pub coefficients: Vec<f64>,
    pub area: f64,
    /// Contact pressure at the point.
    pub pressure: f64,
}

impl ContactModel {
    /// The closed contact points of an evaluation as heat conduction links (tied points
    /// always conduct).
    pub fn thermal_links(
        &self,
        coords: &[[f64; 3]],
        u: &[f64],
        evaluation: &Evaluation,
    ) -> Vec<ThermalLink> {
        let current = current_configuration(coords, u);
        let mut out = Vec::new();
        for (pi, pair) in self.pairs.iter().enumerate() {
            if !pair.active {
                continue;
            }
            let results = evaluation.points.get(pi);
            for p in 0..pair.points.len() {
                let Ok(kin) = pair.kinematics(p, &current, u) else {
                    continue;
                };
                let (closed, pressure) = if pair.law.is_tied() {
                    (true, 0.0)
                } else if pair.mortar_nodes.is_empty() {
                    match results.and_then(|r| r.get(p)) {
                        Some(r) => (r.element && r.state.pressure > 0.0, r.state.pressure),
                        None => (false, 0.0),
                    }
                } else {
                    let pressure = pair.law.face_pressure(-kin.gap).0;
                    (kin.gap <= 0.0 && pressure > 0.0, pressure)
                };
                if closed {
                    out.push(ThermalLink {
                        pair: pair.index,
                        nodes: kin.nodes,
                        coefficients: kin.coefficients,
                        area: kin.area,
                        pressure,
                    });
                }
            }
        }
        out
    }
}

impl Pair {
    /// The sorted DOFs of the slave nodes of `points` and the nodes of master face `face`.
    fn point_face_dofs(&self, points: &[u32], face: u32) -> Vec<u32> {
        let mut dofs: Vec<u32> = points
            .iter()
            .flat_map(|&p| self.points[p as usize].nodes.iter())
            .chain(&self.master[face as usize].nodes)
            .flat_map(|&n| (0..3).map(move |d| n * 3 + d))
            .collect();
        dofs.sort_unstable();
        dofs.dedup();
        dofs
    }

    /// Natural coordinates of a slave point's current position and the slave face normal
    /// there (face-to-face), or the node's position.
    fn position(&self, p: usize, current: &[[f64; 3]]) -> [f64; 3] {
        let point = &self.points[p];
        let mut x = [0.0; 3];
        for (&n, &w) in point.nodes.iter().zip(&point.weights) {
            let c = current[n as usize];
            for d in 0..3 {
                x[d] += w * c[d];
            }
        }
        x
    }

    /// Candidate master faces of every point: those whose box, inflated by two typical
    /// edge lengths plus the cutoff, contains the point.
    fn prepare_candidates(&mut self, current: &[[f64; 3]]) {
        let reach = self.cutoff.iter().copied().fold(0.0, f64::max);
        let margin = 2.0 * self.typical + reach;
        let grid = FaceGrid::new(&self.master, current, self.typical.max(margin), margin);
        self.candidates = (0..self.points.len())
            .map(|p| {
                let x = self.position(p, current);
                let mut c: Vec<u32> = grid.candidates(x).to_vec();
                c.sort_unstable();
                c.dedup();
                c
            })
            .collect();
    }

    /// The closest master face of a point among its candidates: face, natural coordinates,
    /// geometry at the foot point, the point's position.
    fn closest(
        &self,
        p: usize,
        current: &[[f64; 3]],
    ) -> Option<(u32, [f64; 2], geometry::FacePoint, [f64; 3])> {
        let point = &self.points[p];
        let x = self.position(p, current);
        let candidates = self.candidates.get(p)?;
        if candidates.is_empty() {
            return None;
        }
        let slave_nodes: HashSet<u32> = point.nodes.iter().copied().collect();
        let mut best: Option<(f64, bool, u32, [f64; 2], geometry::FacePoint)> = None;
        for &f in candidates {
            let face = &self.master[f as usize];
            if face.nodes.iter().any(|n| slave_nodes.contains(n)) {
                continue;
            }
            let (natural, fp) = face.project(current, x);
            let distance = geometry::norm(sub(x, fp.x));
            let interior = face.is_interior(natural);
            // Prefer a face the point projects into; among those the closest.
            let better = match &best {
                None => true,
                Some((d, i, ..)) => (interior && !i) || (interior == *i && distance < *d),
            };
            if better {
                best = Some((distance, interior, f, natural, fp));
            }
        }
        best.map(|(_, _, f, natural, fp)| (f, natural, fp, x))
    }

    /// One point: its result and, when a spring exists, the element.
    /// One integration point or slave node: its result and, when a spring exists, the
    /// element.
    fn evaluate_point(
        &self,
        p: usize,
        current: &[[f64; 3]],
        u: &[f64],
        previous: &ContactPointState,
        predictor: bool,
        damping: Option<&Damping>,
    ) -> (PointResult, Option<ContactElement>) {
        match self.kinematics(p, current, u) {
            Ok(kin) => self.respond(&kin, previous, predictor, damping),
            Err((x, gap)) => (self.open_result(x, gap, self.points[p].area), None),
        }
    }

    /// A mortar slave node: the kinematics of its integration points weighted with the
    /// node's shape function (`a = N_k dA`), so that the relation acts on the weighted gap
    /// `ḡ_k = Σ a g / Σ a` and the node carries the force `p_k Σ a` (the dual-basis mortar
    /// condition of CalculiX for hard contact; for a penalty relation the nodal pressure is
    /// that of the weighted gap).
    fn evaluate_mortar_node(
        &self,
        k: usize,
        current: &[[f64; 3]],
        u: &[f64],
        previous: &ContactPointState,
        predictor: bool,
        damping: Option<&Damping>,
    ) -> (PointResult, Option<ContactElement>) {
        let (node, points) = &self.mortar_nodes[k];
        let x = current[*node as usize];
        let mut total = 0.0;
        let mut merged: Vec<(u32, f64)> = Vec::new();
        let mut normal = [0.0; 3];
        let mut tangent = [0.0; 3];
        let (mut geometric_gap, mut initial_gap, mut gap) = (0.0, 0.0, 0.0);
        let mut relative = [0.0; 3];
        let mut best: Option<(f64, u32)> = None;
        let mut cutoff = 0.0_f64;
        let mut all_area = 0.0;
        for &(p, w) in points {
            let p = p as usize;
            let a = w * self.points[p].area;
            all_area += a;
            let Ok(kin) = self.kinematics(p, current, u) else {
                continue;
            };
            total += a;
            for (&n, &c) in kin.nodes.iter().zip(&kin.coefficients) {
                merged.push((n, a * c));
            }
            for d in 0..3 {
                normal[d] += a * kin.normal[d];
                tangent[d] += a * kin.tangent[d];
                relative[d] += a * kin.relative[d];
            }
            geometric_gap += a * kin.geometric_gap;
            initial_gap += a * kin.initial_gap;
            gap += a * kin.gap;
            cutoff = cutoff.max(kin.cutoff);
            if best.is_none_or(|(b, _)| a > b) {
                best = Some((a, kin.master_face));
            }
        }
        let Some((_, master_face)) = best else {
            return (self.open_result(x, 0.0, all_area), None);
        };
        if total.abs() <= 1e-300 {
            return (self.open_result(x, 0.0, all_area), None);
        }
        merged.sort_by_key(|&(n, _)| n);
        let mut nodes: Vec<u32> = Vec::new();
        let mut coefficients: Vec<f64> = Vec::new();
        for (n, c) in merged {
            if nodes.last() == Some(&n) {
                *coefficients.last_mut().unwrap() += c / total;
            } else {
                nodes.push(n);
                coefficients.push(c / total);
            }
        }
        let kin = Kinematics {
            nodes,
            coefficients,
            normal: normalized(normal),
            tangent,
            geometric_gap: geometric_gap / total,
            initial_gap: initial_gap / total,
            gap: gap / total,
            relative: scale(relative, 1.0 / total),
            position: x,
            master_face,
            area: total,
            cutoff,
        };
        self.respond(&kin, previous, predictor, damping)
    }

    /// The result of a point without a spring.
    fn open_result(&self, x: [f64; 3], gap: f64, area: f64) -> PointResult {
        PointResult {
            state: ContactPointState {
                status: ContactStatus::Open,
                gap,
                pressure: 0.0,
                slip: [0.0; 2],
            },
            normal: [0.0; 3],
            tangents: [[0.0; 3]; 2],
            tangential: [0.0; 2],
            shear: [0.0; 2],
            energy: 0.0,
            normal_energy: 0.0,
            force: [0.0; 3],
            master_face: None,
            area,
            position: x,
            element: false,
        }
    }

    /// The geometry of a point against its master face: `Err((position, gap))` when it has
    /// no master face (or the pair is inactive).
    fn kinematics(
        &self,
        p: usize,
        current: &[[f64; 3]],
        u: &[f64],
    ) -> std::result::Result<Kinematics, ([f64; 3], f64)> {
        let point = &self.points[p];
        let x = self.position(p, current);
        if !self.active {
            return Err((x, 0.0));
        }
        let tied = self.law.is_tied();
        // Tied points keep the master face they started on.
        let found = if tied {
            let Some((f, _)) = self.initial[p] else {
                return Err((x, 0.0));
            };
            let (natural, fp) = self.master[f as usize].project(current, x);
            Some((f, natural, fp, x))
        } else {
            self.closest(p, current)
        };
        let Some((f, _natural, fp, x)) = found else {
            return Err((x, 0.0));
        };
        let face = &self.master[f as usize];
        let n = fp.normal;
        let geometric_gap = dot(sub(x, fp.x), n);
        // A face-to-face point beside the master surface (its foot point clamped to an edge
        // or corner of the closest face, the point itself off to the side) touches nothing:
        // the distance along the normal of the clamped foot point is no overclosure. Without
        // this a slave surface that reaches past the end of the master gets springs with a
        // false overclosure of the order of its overhang. Node-to-face points are sorted out
        // at the start of the increment instead (`prepare`).
        if !tied && self.kind != ContactKind::NodeToSurface {
            let r = sub(x, fp.x);
            let lateral = geometry::norm(sub(r, scale(n, geometric_gap)));
            if lateral > BESIDE_TOLERANCE * self.typical {
                return Err((x, 0.0));
            }
        }
        let initial_gap = self.initial[p].map_or(0.0, |(_, g)| g);
        let gap = match self.clearance {
            Some(c) => geometric_gap - initial_gap + c,
            None => geometric_gap,
        };
        // Coefficients w_i for the slave nodes, -N_j for the master nodes.
        let mut nodes: Vec<u32> = Vec::with_capacity(point.nodes.len() + face.nodes.len());
        let mut coefficients: Vec<f64> = Vec::with_capacity(nodes.capacity());
        nodes.extend(&point.nodes);
        coefficients.extend(&point.weights);
        nodes.extend(&face.nodes);
        coefficients.extend(fp.n.iter().map(|nj| -nj));
        let mut relative = [0.0; 3];
        for (&node, &c) in nodes.iter().zip(&coefficients) {
            for d in 0..3 {
                relative[d] += c * u[node as usize * 3 + d];
            }
        }
        Ok(Kinematics {
            nodes,
            coefficients,
            normal: n,
            tangent: fp.t[0],
            geometric_gap,
            initial_gap,
            gap,
            relative,
            position: x,
            master_face: f,
            area: point.area,
            cutoff: self.cutoff[p],
        })
    }

    /// The spring of a point (or mortar node) from its kinematics: the pressure relation,
    /// friction with return mapping, the element.
    fn respond(
        &self,
        kin: &Kinematics,
        previous: &ContactPointState,
        predictor: bool,
        damping: Option<&Damping>,
    ) -> (PointResult, Option<ContactElement>) {
        let tied = self.law.is_tied();
        let n = kin.normal;
        let (geometric_gap, initial_gap, gap, x, f) = (
            kin.geometric_gap,
            kin.initial_gap,
            kin.gap,
            kin.position,
            kin.master_face,
        );
        let overclosure = -gap;
        let node_to_face = self.kind == ContactKind::NodeToSurface;
        let within = gap < kin.cutoff;
        let exists = if tied {
            initial_gap.abs() <= kin.cutoff.max(self.typical)
        } else {
            within
        };
        // A face-to-face point that is open but within the cutoff keeps the stiffness of
        // the relation at zero overclosure, without a force: the first iteration of a step
        // that starts touching then sees the contact instead of a floating body.
        let open_face = !tied && !node_to_face && overclosure <= 0.0;
        // Likewise an open node-to-face spring that carries no tension (dynamic steps).
        let open_node = !tied && node_to_face && self.compression_only && overclosure <= 0.0;
        // In a dynamic step a linear relation starts rounded (see `Law::rounded_pressure`).
        let smoothing = (damping.is_some() && !open_face && !open_node && !tied)
            .then_some(ROUNDING * self.typical)
            .filter(|_| matches!(self.law, Law::Linear { .. }));
        if !exists || ((open_face || open_node) && !predictor) {
            return (self.open_result(x, gap, kin.area), None);
        }
        let (pressure, dp) = if tied {
            // Tied: the stiffness acts on the change of the geometric gap.
            self.law.node_pressure(-(geometric_gap - initial_gap))
        } else if open_node {
            (0.0, self.law.node_pressure(0.0).1)
        } else if let Some(delta) = smoothing {
            self.law.rounded_pressure(overclosure, delta)
        } else if node_to_face {
            self.law.node_pressure(overclosure)
        } else if open_face {
            (
                0.0,
                self.law.face_pressure(1e-12 * self.typical.max(1e-300)).1,
            )
        } else {
            self.law.face_pressure(overclosure)
        };

        // Tangential basis and relative displacement.
        let t1 = normalized(sub(kin.tangent, scale(n, dot(kin.tangent, n))));
        let t2 = cross(n, t1);
        let relative = kin.relative;
        // Edges of plane and axisymmetric elements slide in the plane only.
        let planar = self.master.first().is_some_and(|f| f.planar.is_some());
        let tangential = [
            dot(relative, t1),
            if planar { 0.0 } else { dot(relative, t2) },
        ];

        // Friction: return mapping on the regularised Coulomb law.
        let mut shear = [0.0; 2];
        let mut slip = previous.slip;
        let mut status = ContactStatus::Stick;
        let mut tangent = [[0.0; 2]; 2];
        let mut tangential_energy = 0.0;
        // Sliding: d(shear)/d(overclosure) = μ dp/dd along the slip direction.
        let mut coupling = [0.0; 2];
        match self.friction {
            // Node-to-face springs keep their friction in tension as well (observed: ccx
            // reports shear at nodes with a negative CPRESS), bounded by μ |p|.
            Some(friction) if tied || pressure > 0.0 || (node_to_face && pressure < 0.0) => {
                let lambda = friction.stick;
                let trial = [
                    lambda * (tangential[0] - slip[0]),
                    lambda * (tangential[1] - slip[1]),
                ];
                let size = (trial[0] * trial[0] + trial[1] * trial[1]).sqrt();
                let limit = if tied {
                    f64::INFINITY
                } else {
                    friction.mu * pressure.abs()
                };
                if size <= limit || size == 0.0 {
                    shear = trial;
                    tangent = [[lambda, 0.0], [0.0, lambda]];
                    tangential_energy = 0.5 * size * size / lambda;
                } else {
                    status = ContactStatus::Slip;
                    let dir = [trial[0] / size, trial[1] / size];
                    shear = [limit * dir[0], limit * dir[1]];
                    slip = [
                        tangential[0] - shear[0] / lambda,
                        tangential[1] - shear[1] / lambda,
                    ];
                    let a = lambda * limit / size;
                    tangent = [
                        [a * (1.0 - dir[0] * dir[0]), -a * dir[0] * dir[1]],
                        [-a * dir[0] * dir[1], a * (1.0 - dir[1] * dir[1])],
                    ];
                    tangential_energy = 0.5 * limit * limit / lambda;
                    let dlimit = friction.mu * dp * pressure.signum();
                    coupling = [dlimit * dir[0], dlimit * dir[1]];
                }
            }
            Some(friction) if (open_face || (predictor && !tied)) && damping.is_none() => {
                // Within the cutoff but not touching: the stick stiffness as a predictor,
                // without a force, like the normal stiffness above. Not in a dynamic step,
                // where the mass holds a sliding body: the stick stiffness would stop it in
                // the first iteration and leave the following ones far from the slip.
                slip = tangential;
                status = ContactStatus::Open;
                tangent = [[friction.stick, 0.0], [0.0, friction.stick]];
            }
            _ => {
                // No friction or no pressure: slides freely, the slip follows the motion.
                slip = tangential;
                if !tied && pressure <= 0.0 {
                    status = ContactStatus::Open;
                }
            }
        }
        if planar {
            shear[1] = 0.0;
            slip[1] = 0.0;
            coupling[1] = 0.0;
            tangent[0][1] = 0.0;
            tangent[1][0] = 0.0;
            tangent[1][1] = 0.0;
        }
        // CalculiX reports no slip for frictionless contact, except for mortar contact.
        let reported_tangential = if self.friction.is_some() || self.mortar {
            tangential
        } else {
            [0.0; 2]
        };
        if node_to_face && !tied && pressure <= 0.0 && self.friction.is_some() {
            status = ContactStatus::Open;
        }
        // Damping of a pressed spring: the normal pressure becomes `p + q`.
        let spring_pressure = pressure;
        let (pressure, dp, damping_pressure) = match damping {
            Some(damping)
                if !tied
                    && pressure > 0.0
                    && !open_face
                    && previous.status != ContactStatus::Open =>
            {
                let mut rate = 0.0;
                for (&node, &c) in kin.nodes.iter().zip(&kin.coefficients) {
                    for d in 0..3 {
                        rate -= c * damping.velocities[node as usize * 3 + d] * n[d];
                    }
                }
                let c = damping.coefficient * dp;
                let q = (c * rate).max(-pressure);
                // Lifting off the damping cancels the pressure; the tangent stays that of
                // the damped spring, the same on both sides of the limit.
                (pressure + q, dp + c * damping.rate, q)
            }
            _ => (pressure, dp, 0.0),
        };

        let mut dofs: Vec<u32> = Vec::with_capacity(3 * kin.nodes.len());
        for &node in &kin.nodes {
            dofs.extend((0..3).map(|d| node * 3 + d));
        }
        let coefficients = &kin.coefficients;
        let area = kin.area;
        // Traction on the slave: -p n + τ (the spring pulls the slave back along the
        // relative tangential motion); the internal force is minus the force on the node.
        let traction = [
            -pressure * n[0] + shear[0] * t1[0] + shear[1] * t2[0],
            -pressure * n[1] + shear[0] * t1[1] + shear[1] * t2[1],
            -pressure * n[2] + shear[0] * t1[2] + shear[1] * t2[2],
        ];
        let m = coefficients.len();
        let mut fe = vec![0.0; 3 * m];
        for (a, &c) in coefficients.iter().enumerate() {
            for d in 0..3 {
                fe[a * 3 + d] = c * area * traction[d];
            }
        }
        // 3x3 kernel: dp n nᵀ + Σ D_kl t_k t_lᵀ.
        let mut kernel = [[0.0; 3]; 3];
        let ts = [t1, t2];
        for (i, row) in kernel.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v = dp * n[i] * n[j];
                for k in 0..2 {
                    for l in 0..2 {
                        *v += tangent[k][l] * ts[k][i] * ts[l][j];
                    }
                }
            }
        }
        let mut ke = vec![0.0; 9 * m * m];
        let n3 = 3 * m;
        for a in 0..m {
            for b in 0..m {
                let cab = coefficients[a] * coefficients[b] * area;
                for i in 0..3 {
                    for j in 0..3 {
                        ke[(a * 3 + i) * n3 + b * 3 + j] = cab * kernel[i][j];
                    }
                }
            }
        }
        // Unsymmetric rest: d(τ_k t_k)/dΔ through the overclosure d = -Δ·n, i.e.
        // -(coupling_k t_k) nᵀ.
        let unsymmetric = if coupling != [0.0; 2] {
            let s3 = [
                coupling[0] * t1[0] + coupling[1] * t2[0],
                coupling[0] * t1[1] + coupling[1] * t2[1],
                coupling[0] * t1[2] + coupling[1] * t2[2],
            ];
            let mut e = vec![0.0; 9 * m * m];
            for a in 0..m {
                for b in 0..m {
                    let cab = coefficients[a] * coefficients[b] * area;
                    for i in 0..3 {
                        for j in 0..3 {
                            e[(a * 3 + i) * n3 + b * 3 + j] = -cab * s3[i] * n[j];
                        }
                    }
                }
            }
            e
        } else {
            Vec::new()
        };
        let normal_energy = if let Some(delta) = smoothing {
            self.law.rounded_energy(overclosure, delta)
        } else if node_to_face && !tied {
            node_energy(&self.law, overclosure)
        } else if tied {
            let d = -(geometric_gap - initial_gap);
            0.5 * spring_pressure * d
        } else {
            self.law.face_energy(overclosure)
        };
        let energy = area * (normal_energy + tangential_energy);
        let normal_energy = area * normal_energy;
        let force = [
            -area * traction[0],
            -area * traction[1],
            -area * traction[2],
        ];
        let result = PointResult {
            state: ContactPointState {
                status,
                gap,
                pressure,
                slip,
            },
            normal: n,
            tangents: [t1, t2],
            tangential: reported_tangential,
            shear,
            energy,
            normal_energy,
            force,
            master_face: Some(f),
            area,
            position: x,
            element: !open_face,
        };
        (
            result,
            Some(ContactElement {
                dofs,
                ke,
                damping: if damping_pressure != 0.0 {
                    // The normal part of the traction from the damping.
                    let mut f = vec![0.0; 3 * m];
                    for (a, &c) in coefficients.iter().enumerate() {
                        for d in 0..3 {
                            f[a * 3 + d] = -c * area * damping_pressure * n[d];
                        }
                    }
                    f
                } else {
                    Vec::new()
                },
                normal: if tied {
                    Vec::new()
                } else {
                    let mut f = vec![0.0; 3 * m];
                    for (a, &c) in coefficients.iter().enumerate() {
                        for d in 0..3 {
                            f[a * 3 + d] = -c * area * spring_pressure * n[d];
                        }
                    }
                    f
                },
                fe,
                unsymmetric,
            }),
        )
    }
}

/// Node coordinates plus displacements.
fn current_configuration(coords: &[[f64; 3]], u: &[f64]) -> Vec<[f64; 3]> {
    coords
        .iter()
        .enumerate()
        .map(|(n, c)| [c[0] + u[n * 3], c[1] + u[n * 3 + 1], c[2] + u[n * 3 + 2]])
        .collect()
}

/// `∫₀ᵈ p` of the node-to-face relation (Simpson's rule).
fn node_energy(law: &Law, d: f64) -> f64 {
    const N: usize = 16;
    let h = d / N as f64;
    let mut sum = law.node_pressure(0.0).0 + law.node_pressure(d).0;
    for i in 1..N {
        let factor = if i % 2 == 1 { 4.0 } else { 2.0 };
        sum += factor * law.node_pressure(h * i as f64).0;
    }
    sum * h / 3.0
}
