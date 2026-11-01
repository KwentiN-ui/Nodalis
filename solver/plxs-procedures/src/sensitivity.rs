//! `*SENSITIVITY` (concept section 4.5, T17): the derivatives of design responses with
//! respect to the normal displacement of the design nodes (`*DESIGN VARIABLES,
//! TYPE=COORDINATE`), as the `SEN` output and the design response table of the `.dat`.
//!
//! **Design variables.** A design node is a node of the design set that lies on an external
//! face of a solid element (a face shared by no second element). Its normal is the
//! normalized sum of the unit normals of all external faces it belongs to; the components
//! of degrees of freedom held by a single-point constraint of the step are dropped before
//! normalizing, so a node fixed in every direction has no normal and is no design variable.
//!
//! **Derivatives.** For a design node `a` with normal `n`, the derivative of a response `R`
//! is `dR/ds = Σ_k n_k dR/dx_ak`, the coordinate derivatives by central differences of the
//! element quantities of the elements around the node (semi-analytic: the node moves alone,
//! the element matrices are re-evaluated):
//!
//! - `MASS` (of the elements of the response's set): `ρ dV_e/dx`.
//! - `STRAINENERGY` (`½ uᵀ K_S u` over the set `S` for the displacements `u` of the last
//!   step): `½ uᵀ dK_S u + λᵀ (dF − dK u)` with the adjoint `K λ = K_S u`, which is `u`
//!   itself when the set covers the whole model. `dF` are the derivatives of the
//!   distributed loads the step carries (pressures on faces that move, body forces of
//!   elements whose volume changes).
//! - `EIGENFREQUENCY`: for the modes `φ_j` stored by the previous `*FREQUENCY, STORAGE=YES`
//!   step (mass normalized), `dλ_j = φ_jᵀ (dK − λ_j dM) φ_j`. Repeated eigenvalues are not
//!   differentiable in the usual sense; like CalculiX, each mode of the stored basis gets
//!   its own value, and only the sum over a cluster of repeated eigenvalues (the trace of
//!   `φ_iᵀ (dK − λ dM) φ_j`) is independent of that basis.
//!
//! **Output.** The `DFDN` written per node is the derivative divided by the node's area,
//! `A_a = Σ_f ∫_f N_a dA` over its external faces, which makes the field a sensitivity
//! density independent of the mesh; the corner nodes of quadratic elements, whose shape
//! function integrals vanish or are negative, take the mean of the midside nodes next to
//! them along the external edges. The table prints the response value and the length
//! (2-norm over the nodes) of this field. `DFDNFIL` is the filtered field normalized to unit
//! length; the linear filter (`*FILTER, TYPE=LINEAR`, radius `r`) spreads every node's value
//! to the design nodes within `r` with weights `1 − d/r` normalized per source node, and
//! without a filter `DFDNFIL` is `DFDN` normalized. All of this is inferred from CalculiX's
//! output (see `docs/solver/SCHNITTSTELLEN.md`, "Stand nach T17"); no CalculiX code was
//! consulted.

use std::collections::HashMap;

use plxs_assembly::dofs::amplitude_factor;
use plxs_assembly::{ElementTable, Spc, assemble_stiffness};
use plxs_core::mesh::ElementRef;
use plxs_core::results::{HistoryBlock, IncrementKind, ResultField, ResultsIncrement};
use plxs_core::step::{DistributedLoadKind, Procedure as ProcedureKind, Sensitivity};
use plxs_core::{Error, Model, Progress, ProgressEvent, Result, Step};
use plxs_elements::SolidElement;
use plxs_elements::shape::{face_rule, face_shape};
use plxs_linalg::{LinearSolver, SparseMatrix};
use plxs_output::StaRow;
use plxs_output::dat::DESIGN_RESPONSE_TABLE;
use rayon::prelude::*;

use crate::fields::node_order;
use crate::{Procedure, ProcedureContext, StepOutcome};

pub struct SensitivityProcedure;

/// Relative size of the coordinate perturbation of the central differences.
const RELATIVE_STEP: f64 = 1e-6;
/// A node of the design set on the surface of the model.
#[derive(Clone, Debug)]
struct DesignNode {
    /// Dense node index.
    node: u32,
    /// The design direction (unit length), zero when the node is held in every direction
    /// its faces point to.
    normal: [f64; 3],
    /// `Σ_f ∫ N_a dA` over the external faces, the area the node stands for.
    area: f64,
    /// Corner of quadratic elements: positions (in the design node list) of the midside
    /// design nodes along its external edges, whose mean it reports.
    mids: Vec<usize>,
    /// Entries of the element table that contain the node.
    elements: Vec<u32>,
}

impl DesignNode {
    fn active(&self) -> bool {
        self.normal != [0.0; 3] && (self.area > 0.0 || !self.mids.is_empty())
    }
}

/// What a response needs from the element derivatives.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ResponseKind {
    Mass,
    StrainEnergy,
    Eigenfrequency,
}

/// A distributed load on one element, as the derivative of the load vector needs it.
enum ElementLoad {
    Pressure { face: u8, value: f64 },
    Body(Box<dyn Fn([f64; 3]) -> [f64; 3] + Send + Sync>),
}

/// The elements of a `*DESIGN RESPONSE` set as flags per table entry (`None`: all).
fn response_elements(
    model: &Model,
    table: &ElementTable,
    elset: Option<&str>,
) -> Result<Option<Vec<bool>>> {
    let Some(name) = elset else {
        return Ok(None);
    };
    let Some(ids) = model.sets.element_sets.get(name) else {
        return Err(Error::undefined("*DESIGN RESPONSE", "element set", name));
    };
    let mut flags = vec![false; table.entries.len()];
    for id in ids {
        if let Some(index) = model.element_index(*id) {
            let entry = table.entry_of[index as usize];
            if entry != u32::MAX {
                flags[entry as usize] = true;
            }
        }
    }
    if flags.iter().all(|f| *f) {
        return Ok(None);
    }
    Ok(Some(flags))
}

/// The geometry of one external face at its nodes: the unit normal at the centre and the
/// shape function integrals of its nodes (element-local node indices).
fn face_geometry(
    formulation: &SolidElement,
    coords: &[[f64; 3]],
    face: usize,
) -> ([f64; 3], Vec<(usize, f64)>) {
    let f = &formulation.family.faces()[face];
    let quadratic = formulation.family.is_quadratic();
    let nodes: Vec<usize> = if quadratic {
        f.corners.iter().chain(f.mids).copied().collect()
    } else {
        f.corners.to_vec()
    };
    let (points, weights) = face_rule(f.corners.len(), quadratic);
    let mut n = vec![0.0; nodes.len()];
    let mut dn = vec![[0.0; 2]; nodes.len()];
    let mut integrals = vec![0.0; nodes.len()];
    let tangents = |dn: &[[f64; 2]]| {
        let mut t1 = [0.0; 3];
        let mut t2 = [0.0; 3];
        for (i, &node) in nodes.iter().enumerate() {
            for k in 0..3 {
                t1[k] += dn[i][0] * coords[node][k];
                t2[k] += dn[i][1] * coords[node][k];
            }
        }
        [
            t1[1] * t2[2] - t1[2] * t2[1],
            t1[2] * t2[0] - t1[0] * t2[2],
            t1[0] * t2[1] - t1[1] * t2[0],
        ]
    };
    for (p, w) in points.iter().zip(&weights) {
        face_shape(f.corners.len(), quadratic, *p, &mut n, &mut dn);
        let normal = tangents(&dn);
        let da = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
        for (i, integral) in integrals.iter_mut().enumerate() {
            *integral += n[i] * da * w;
        }
    }
    // Normal at the centre of the face.
    let centre = if f.corners.len() == 4 {
        [0.0, 0.0]
    } else {
        [1.0 / 3.0, 1.0 / 3.0]
    };
    face_shape(f.corners.len(), quadratic, centre, &mut n, &mut dn);
    let normal = tangents(&dn);
    let length = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
    let unit = if length > 0.0 {
        normal.map(|c| c / length)
    } else {
        [0.0; 3]
    };
    (unit, nodes.into_iter().zip(integrals).collect())
}

/// Builds the design nodes: surface detection, normals, areas, adjacency.
fn design_nodes(ctx: &ProcedureContext<'_>, step: &Step) -> Result<Vec<DesignNode>> {
    let model = ctx.model;
    let table = ctx.table;
    let Some(design) = &model.design_variables else {
        return Err(Error::input(
            "*SENSITIVITY",
            "the step needs *DESIGN VARIABLES, TYPE=COORDINATE before the steps",
        ));
    };
    let Some(ids) = model.sets.node_sets.get(&design.nset) else {
        return Err(Error::undefined(
            "*DESIGN VARIABLES",
            "node set",
            &design.nset,
        ));
    };
    let mut in_set = vec![false; model.nodes.len()];
    for id in ids {
        if let Some(i) = model.nodes.index(*id) {
            in_set[i as usize] = true;
        }
    }

    // External faces of the solid elements: a face key (sorted corner nodes) seen once.
    let mut seen: HashMap<Vec<u32>, Option<(u32, u8)>> = HashMap::new();
    for (e, entry) in table.entries.iter().enumerate() {
        let formulation = table.formulation(entry);
        if formulation.expansion().is_some() {
            continue;
        }
        for (f, face) in formulation.family.faces().iter().enumerate() {
            let mut key: Vec<u32> = face.corners.iter().map(|&c| entry.nodes[c]).collect();
            key.sort_unstable();
            match seen.get_mut(&key) {
                Some(slot) => *slot = None,
                None => {
                    seen.insert(key, Some((e as u32, f as u8)));
                }
            }
        }
    }
    let mut external: Vec<(u32, u8)> = seen.into_values().flatten().collect();
    external.sort_unstable();

    // Accumulate per design node.
    let mut position: HashMap<u32, usize> = HashMap::new();
    let mut nodes: Vec<DesignNode> = Vec::new();
    let mut normals: Vec<[f64; 3]> = Vec::new();
    let mut mid_nodes: Vec<Vec<u32>> = Vec::new();
    let mut quadratic_corner: Vec<bool> = Vec::new();
    let mut at = |node: u32,
                  nodes: &mut Vec<DesignNode>,
                  normals: &mut Vec<[f64; 3]>,
                  mid_nodes: &mut Vec<Vec<u32>>,
                  quadratic_corner: &mut Vec<bool>|
     -> usize {
        *position.entry(node).or_insert_with(|| {
            nodes.push(DesignNode {
                node,
                normal: [0.0; 3],
                area: 0.0,
                mids: Vec::new(),
                elements: Vec::new(),
            });
            normals.push([0.0; 3]);
            mid_nodes.push(Vec::new());
            quadratic_corner.push(false);
            nodes.len() - 1
        })
    };
    for &(e, f) in &external {
        let entry = &table.entries[e as usize];
        let formulation = table.formulation(entry);
        let coords = table.coordinates(model, entry);
        let (normal, integrals) = face_geometry(formulation, &coords, f as usize);
        let face = &formulation.family.faces()[f as usize];
        let quadratic = formulation.family.is_quadratic();
        for (local, integral) in integrals {
            let node = entry.nodes[local];
            if !in_set[node as usize] {
                continue;
            }
            let p = at(
                node,
                &mut nodes,
                &mut normals,
                &mut mid_nodes,
                &mut quadratic_corner,
            );
            for k in 0..3 {
                normals[p][k] += normal[k];
            }
            let corner = face.corners.iter().position(|&c| c == local);
            if quadratic && let Some(c) = corner {
                quadratic_corner[p] = true;
                let m = face.corners.len();
                for mid in [face.mids[c], face.mids[(c + m - 1) % m]] {
                    let mid = entry.nodes[mid];
                    if in_set[mid as usize] && !mid_nodes[p].contains(&mid) {
                        mid_nodes[p].push(mid);
                    }
                }
            } else {
                nodes[p].area += integral;
            }
        }
    }

    // Normals without the held directions.
    let spc = Spc::of_step(
        model,
        step,
        &table.dofs,
        1.0,
        ctx.state.total_time + 1.0,
        1.0,
    )?;
    let per_node = table.dofs.per_node as u32;
    for (p, node) in nodes.iter_mut().enumerate() {
        let mut n = normals[p];
        for (k, nk) in n.iter_mut().enumerate() {
            if spc
                .dofs
                .binary_search(&(node.node * per_node + k as u32))
                .is_ok()
            {
                *nk = 0.0;
            }
        }
        let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        node.normal = if length > 1e-12 {
            n.map(|c| c / length)
        } else {
            [0.0; 3]
        };
    }
    // Quadratic corners report the mean of their midside nodes; a corner with none (the
    // midside nodes are not in the design set) keeps its own derivative over the area of
    // the linear shape function.
    for p in 0..nodes.len() {
        if !quadratic_corner[p] {
            continue;
        }
        let mids: Vec<usize> = mid_nodes[p]
            .iter()
            .filter_map(|m| position.get(m).copied())
            .filter(|&m| nodes[m].normal != [0.0; 3])
            .collect();
        if mids.is_empty() {
            nodes[p].area = 0.0;
            for &(e, f) in &external {
                let entry = &table.entries[e as usize];
                let formulation = table.formulation(entry);
                let face = &formulation.family.faces()[f as usize];
                if face
                    .corners
                    .iter()
                    .any(|&l| entry.nodes[l] == nodes[p].node)
                {
                    let corners: Vec<[f64; 3]> = face
                        .corners
                        .iter()
                        .map(|&l| model.nodes.coordinates[entry.nodes[l] as usize])
                        .collect();
                    nodes[p].area += polygon_area(&corners) / corners.len() as f64;
                }
            }
        } else {
            nodes[p].mids = mids;
            nodes[p].area = 0.0;
        }
    }

    // Element adjacency of the active nodes.
    let mut adjacency: HashMap<u32, Vec<u32>> = HashMap::new();
    for (e, entry) in table.entries.iter().enumerate() {
        for &n in &entry.nodes {
            if let Some(&p) = position.get(&n)
                && nodes[p].mids.is_empty()
            {
                adjacency.entry(n).or_default().push(e as u32);
            }
        }
    }
    for node in nodes.iter_mut() {
        if let Some(list) = adjacency.remove(&node.node) {
            node.elements = list;
        }
    }
    nodes.sort_by_key(|n| n.node);
    // Positions moved by the sort: remap the midside references.
    let new_position: HashMap<u32, usize> =
        nodes.iter().enumerate().map(|(i, n)| (n.node, i)).collect();
    let old_nodes: Vec<u32> = {
        let mut v: Vec<(usize, u32)> = position.iter().map(|(&n, &p)| (p, n)).collect();
        v.sort_unstable();
        v.into_iter().map(|(_, n)| n).collect()
    };
    for node in nodes.iter_mut() {
        node.mids = node
            .mids
            .iter()
            .map(|&old| new_position[&old_nodes[old]])
            .collect();
    }
    Ok(nodes)
}

/// Area of a planar polygon given by its corners.
fn polygon_area(corners: &[[f64; 3]]) -> f64 {
    let mut s = [0.0; 3];
    for i in 0..corners.len() {
        let a = corners[i];
        let b = corners[(i + 1) % corners.len()];
        s[0] += a[1] * b[2] - a[2] * b[1];
        s[1] += a[2] * b[0] - a[0] * b[2];
        s[2] += a[0] * b[1] - a[1] * b[0];
    }
    0.5 * (s[0] * s[0] + s[1] * s[1] + s[2] * s[2]).sqrt()
}

/// The raw derivatives of all responses at one design node: per response (in the order
/// of `requests`), for `EIGENFREQUENCY` one value per mode.
struct NodeDerivatives {
    values: Vec<Vec<f64>>,
}

/// What the derivative loop needs besides the model.
struct Problem<'a> {
    model: &'a Model,
    table: &'a ElementTable,
    requests: &'a [(ResponseKind, Option<Vec<bool>>)],
    /// Displacements of the last step and the adjoint of the strain energy.
    u: Option<Vec<f64>>,
    adjoint: Option<Vec<f64>>,
    /// Distributed loads per table entry.
    loads: Vec<Vec<ElementLoad>>,
    /// Stored modes: eigenvalues and mode shapes (full DOF vectors), and the groups of
    /// modes differentiated together (single modes).
    eigenvalues: Vec<f64>,
    modes: Vec<&'a [f64]>,
    clusters: Vec<std::ops::Range<usize>>,
}

impl Problem<'_> {
    /// Derivatives of the responses with respect to the normal displacement of a node.
    fn derivatives(&self, node: &DesignNode) -> Result<NodeDerivatives> {
        let table = self.table;
        let model = self.model;
        let needs_mass = self.requests.iter().any(|(k, _)| *k == ResponseKind::Mass);
        let needs_k = self
            .requests
            .iter()
            .any(|(k, _)| matches!(k, ResponseKind::StrainEnergy | ResponseKind::Eigenfrequency));
        let needs_m = self
            .requests
            .iter()
            .any(|(k, _)| *k == ResponseKind::Eigenfrequency);
        let modes = self.modes.len();
        // Accumulators per coordinate direction.
        let mut d_mass = [0.0; 3];
        let mut d_energy = [0.0; 3];
        // Per group of modes (one mode each, see `Problem::clusters`) and direction the
        // matrix `φ_iᵀ (dK − λ dM) φ_j`, upper triangle, row major.
        let mut d_eigen: Vec<Vec<f64>> = self
            .clusters
            .iter()
            .map(|c| vec![0.0; 3 * c.len() * c.len()])
            .collect();
        let mut k_plus = Vec::new();
        let mut k_minus = Vec::new();
        let mut m_plus = Vec::new();
        let mut m_minus = Vec::new();
        for &e in &node.elements {
            let entry = &table.entries[e as usize];
            let formulation = table.formulation(entry);
            let d = table.material_matrix(entry);
            let mut coords = table.coordinates(model, entry);
            let local = entry
                .nodes
                .iter()
                .position(|&n| n == node.node)
                .expect("adjacent element contains the node");
            let size = {
                let mut lo = [f64::INFINITY; 3];
                let mut hi = [f64::NEG_INFINITY; 3];
                for c in &coords {
                    for k in 0..3 {
                        lo[k] = lo[k].min(c[k]);
                        hi[k] = hi[k].max(c[k]);
                    }
                }
                (0..3).map(|k| hi[k] - lo[k]).fold(0.0_f64, f64::max)
            };
            let h = RELATIVE_STEP * size.max(1e-300);
            let ue = self.u.as_ref().map(|u| table.gather(entry, u));
            let le = self.adjoint.as_ref().map(|l| table.gather(entry, l));
            let phi: Vec<Vec<f64>> = self.modes.iter().map(|m| table.gather(entry, m)).collect();
            let rho = entry.density;
            let ndof = entry.dofs.len();
            for k in 0..3 {
                let x0 = coords[local][k];
                coords[local][k] = x0 + h;
                let v_plus = if needs_mass {
                    formulation.volume(&coords)
                } else {
                    0.0
                };
                if needs_k {
                    formulation.stiffness(&coords, d, &mut k_plus);
                }
                if needs_m {
                    formulation.mass(&coords, 1.0, &mut m_plus);
                }
                let f_plus = self.element_load(e, formulation, &coords, ndof);
                coords[local][k] = x0 - h;
                let v_minus = if needs_mass {
                    formulation.volume(&coords)
                } else {
                    0.0
                };
                if needs_k {
                    formulation.stiffness(&coords, d, &mut k_minus);
                }
                if needs_m {
                    formulation.mass(&coords, 1.0, &mut m_minus);
                }
                let f_minus = self.element_load(e, formulation, &coords, ndof);
                coords[local][k] = x0;
                let scale = 0.5 / h;

                for (kind, set) in self.requests {
                    let in_set = set.as_ref().is_none_or(|s| s[e as usize]);
                    match kind {
                        ResponseKind::Mass => {
                            if in_set {
                                let rho = rho.ok_or_else(|| missing_density(model, table, e))?;
                                d_mass[k] += rho * (v_plus - v_minus) * scale;
                            }
                        }
                        ResponseKind::StrainEnergy => {
                            let (Some(ue), Some(le)) = (&ue, &le) else {
                                continue;
                            };
                            // dK u
                            let mut dku = vec![0.0; ndof];
                            for i in 0..ndof {
                                let mut s = 0.0;
                                for j in 0..ndof {
                                    s += (k_plus[i * ndof + j] - k_minus[i * ndof + j]) * ue[j];
                                }
                                dku[i] = s * scale;
                            }
                            let u_dk_u: f64 = ue.iter().zip(&dku).map(|(a, b)| a * b).sum();
                            let l_dk_u: f64 = le.iter().zip(&dku).map(|(a, b)| a * b).sum();
                            let l_df: f64 = match (&f_plus, &f_minus) {
                                (Some(fp), Some(fm)) => le
                                    .iter()
                                    .zip(fp.iter().zip(fm))
                                    .map(|(l, (p, m))| l * (p - m) * scale)
                                    .sum(),
                                _ => 0.0,
                            };
                            if in_set {
                                d_energy[k] += 0.5 * u_dk_u;
                            }
                            d_energy[k] += l_df - l_dk_u;
                        }
                        ResponseKind::Eigenfrequency => {
                            if modes == 0 {
                                continue;
                            }
                            let rho = rho.ok_or_else(|| missing_density(model, table, e))?;
                            for (c, range) in self.clusters.iter().enumerate() {
                                let m = range.len();
                                let lambda =
                                    self.eigenvalues[range.clone()].iter().sum::<f64>() / m as f64;
                                for (ii, i) in range.clone().enumerate() {
                                    // (dK − λ dM) φ_i
                                    let mut w = vec![0.0; ndof];
                                    for r in 0..ndof {
                                        let mut s = 0.0;
                                        for cidx in 0..ndof {
                                            let dk =
                                                k_plus[r * ndof + cidx] - k_minus[r * ndof + cidx];
                                            let dm = (m_plus[r * ndof + cidx]
                                                - m_minus[r * ndof + cidx])
                                                * rho;
                                            s += (dk - lambda * dm) * phi[i][cidx];
                                        }
                                        w[r] = s * scale;
                                    }
                                    for (jj, j) in range.clone().enumerate().skip(ii) {
                                        let v: f64 =
                                            phi[j].iter().zip(&w).map(|(a, b)| a * b).sum();
                                        d_eigen[c][k * m * m + ii * m + jj] += v;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        // Project on the normal.
        let n = node.normal;
        let mut values = Vec::with_capacity(self.requests.len());
        for (kind, _) in self.requests {
            match kind {
                ResponseKind::Mass => values.push(vec![(0..3).map(|k| n[k] * d_mass[k]).sum()]),
                ResponseKind::StrainEnergy => {
                    values.push(vec![(0..3).map(|k| n[k] * d_energy[k]).sum()])
                }
                ResponseKind::Eigenfrequency => {
                    let mut per_mode = vec![0.0; modes];
                    for (c, range) in self.clusters.iter().enumerate() {
                        let m = range.len();
                        let mut a = vec![0.0; m * m];
                        for ii in 0..m {
                            for jj in ii..m {
                                let v: f64 = (0..3)
                                    .map(|k| n[k] * d_eigen[c][k * m * m + ii * m + jj])
                                    .sum();
                                a[ii * m + jj] = v;
                                a[jj * m + ii] = v;
                            }
                        }
                        per_mode[range.start] = a[0];
                    }
                    values.push(per_mode);
                }
            }
        }
        Ok(NodeDerivatives { values })
    }

    /// The distributed loads of one element for the coordinates given, `None` without loads.
    fn element_load(
        &self,
        e: u32,
        formulation: &SolidElement,
        coords: &[[f64; 3]],
        ndof: usize,
    ) -> Option<Vec<f64>> {
        let loads = &self.loads[e as usize];
        if loads.is_empty() {
            return None;
        }
        let mut fe = vec![0.0; ndof];
        for load in loads {
            match load {
                ElementLoad::Pressure { face, value } => {
                    formulation.face_load(coords, *face, *value, &mut fe);
                }
                ElementLoad::Body(f) => formulation.body_force(coords, f, &mut fe),
            }
        }
        Some(fe)
    }
}

fn missing_density(model: &Model, table: &ElementTable, e: u32) -> Error {
    Error::input(
        "*DENSITY",
        format!(
            "element {} has no density; the sensitivity of the mass or the eigenfrequencies needs *DENSITY for every material",
            table.element_id(model, e)
        ),
    )
}

/// Distributed loads of the step per table entry, with their amplitude factors at the
/// end of the step before.
fn element_loads(ctx: &ProcedureContext<'_>, step: &Step) -> Result<Vec<Vec<ElementLoad>>> {
    let model = ctx.model;
    let table = ctx.table;
    let mut loads: Vec<Vec<ElementLoad>> = (0..table.entries.len()).map(|_| Vec::new()).collect();
    let total = ctx.state.total_time;
    for load in &step.loads.distributed {
        let factor = amplitude_factor(model, step, load.amplitude.as_deref(), 1.0, total, 1.0);
        let elements: Vec<u32> = match &load.region {
            ElementRef::Element(id) => vec![*id],
            ElementRef::Set(name) => model
                .sets
                .element_sets
                .get(name)
                .cloned()
                .unwrap_or_default(),
        };
        for id in elements {
            let Some(index) = model.element_index(id) else {
                continue;
            };
            let e = table.entry_of[index as usize];
            if e == u32::MAX {
                continue;
            }
            let entry = &table.entries[e as usize];
            let element_load = match &load.kind {
                DistributedLoadKind::Pressure { face, value } => ElementLoad::Pressure {
                    face: *face,
                    value: value * factor,
                },
                DistributedLoadKind::Gravity {
                    magnitude,
                    direction,
                } => {
                    let rho = entry
                        .density
                        .ok_or_else(|| missing_density(model, table, e))?;
                    let g = direction.map(|d| d * magnitude * rho * factor);
                    ElementLoad::Body(Box::new(move |_| g))
                }
                DistributedLoadKind::Centrifugal {
                    omega_squared,
                    point,
                    axis,
                } => {
                    let rho = entry
                        .density
                        .ok_or_else(|| missing_density(model, table, e))?;
                    let len = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
                    let a = axis.map(|c| c / len);
                    let point = *point;
                    let scale = rho * omega_squared * factor;
                    ElementLoad::Body(Box::new(move |x| {
                        let r = [x[0] - point[0], x[1] - point[1], x[2] - point[2]];
                        let along = r[0] * a[0] + r[1] * a[1] + r[2] * a[2];
                        [
                            scale * (r[0] - along * a[0]),
                            scale * (r[1] - along * a[1]),
                            scale * (r[2] - along * a[2]),
                        ]
                    }))
                }
                DistributedLoadKind::Other { label, .. } => {
                    return Err(Error::Unsupported(format!(
                        "*DLOAD label {label} is not implemented yet"
                    )));
                }
            };
            loads[e as usize].push(element_load);
        }
    }
    Ok(loads)
}

/// The adjoint `K λ = K_S u` of the strain energy of a subset `S`, solved with the
/// constraints of the step (homogeneous).
fn strain_energy_adjoint(
    ctx: &mut ProcedureContext<'_>,
    step: &Step,
    progress: &mut dyn Progress,
    u: &[f64],
    set: &[bool],
) -> Result<Vec<f64>> {
    let total_dofs = ctx.table.dofs.len();
    if ctx.stiffness.is_none() {
        let mut k = SparseMatrix::zeros(ctx.pattern.clone());
        ctx.table.require_elastic()?;
        assemble_stiffness(ctx.table, ctx.model, &mut k);
        ctx.stiffness = Some(k);
    }
    let k = ctx.stiffness.as_ref().unwrap();
    // K_S u element by element.
    let mut rhs = vec![0.0; total_dofs];
    let mut ke = Vec::new();
    for (e, entry) in ctx.table.entries.iter().enumerate() {
        if !set[e] {
            continue;
        }
        let formulation = ctx.table.formulation(entry);
        let coords = ctx.table.coordinates(ctx.model, entry);
        formulation.stiffness(&coords, ctx.table.material_matrix(entry), &mut ke);
        let ue = ctx.table.gather(entry, u);
        let n = entry.dofs.len();
        for i in 0..n {
            let mut s = 0.0;
            for j in 0..n {
                s += ke[i * n + j] * ue[j];
            }
            rhs[entry.dofs[i] as usize] += s;
        }
    }
    let spc = Spc::of_step(
        ctx.model,
        step,
        &ctx.table.dofs,
        1.0,
        ctx.state.total_time + 1.0,
        1.0,
    )?;
    let active = ctx.table.active_dofs();
    let constraint_set = ctx
        .constraints
        .step_set(ctx.model.nodes.len(), &active, &spc.dofs);
    let k_red = constraint_set.reduce(k);
    ctx.solver.analyze(k_red.pattern())?;
    ctx.solver.factorize(&k_red)?;
    let mut v = constraint_set.reduce_rhs(&rhs);
    ctx.solver.solve(&mut v)?;
    progress.event(ProgressEvent::Info(
        "adjoint of the strain energy solved".to_string(),
    ));
    let g = vec![0.0; total_dofs];
    Ok(constraint_set.expand(&v, &g))
}

/// The linear filter: every active node spreads its value to the active nodes within the
/// radius with weights `1 − d/r`, normalized per source node.
fn linear_filter(
    model: &Model,
    nodes: &[DesignNode],
    active: &[bool],
    radius: f64,
    values: &[f64],
) -> Vec<f64> {
    let coords: Vec<[f64; 3]> = nodes
        .iter()
        .map(|n| model.nodes.coordinates[n.node as usize])
        .collect();
    // Uniform grid of cell size `radius` for the neighbour search.
    let cell = |x: &[f64; 3]| -> [i64; 3] { x.map(|c| (c / radius).floor() as i64) };
    let mut grid: HashMap<[i64; 3], Vec<usize>> = HashMap::new();
    for (i, x) in coords.iter().enumerate() {
        if active[i] {
            grid.entry(cell(x)).or_default().push(i);
        }
    }
    let neighbours = |i: usize| -> Vec<(usize, f64)> {
        let c = cell(&coords[i]);
        let mut out = Vec::new();
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(list) = grid.get(&[c[0] + dx, c[1] + dy, c[2] + dz]) {
                        for &j in list {
                            let d = (0..3)
                                .map(|k| (coords[i][k] - coords[j][k]).powi(2))
                                .sum::<f64>()
                                .sqrt();
                            if d < radius {
                                out.push((j, 1.0 - d / radius));
                            }
                        }
                    }
                }
            }
        }
        out
    };
    let lists: Vec<Vec<(usize, f64)>> = (0..nodes.len())
        .into_par_iter()
        .map(|i| if active[i] { neighbours(i) } else { Vec::new() })
        .collect();
    let mut filtered = vec![0.0; nodes.len()];
    for (j, list) in lists.iter().enumerate() {
        let total: f64 = list.iter().map(|(_, w)| w).sum();
        if total <= 0.0 {
            continue;
        }
        for &(i, w) in list {
            filtered[i] += w / total * values[j];
        }
    }
    filtered
}

fn normalized(values: &[f64]) -> Vec<f64> {
    let length = values.iter().map(|v| v * v).sum::<f64>().sqrt();
    if length > 0.0 {
        values.iter().map(|v| v / length).collect()
    } else {
        vec![0.0; values.len()]
    }
}

impl Procedure for SensitivityProcedure {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        let ProcedureKind::Sensitivity(parameters) = &step.procedure else {
            return Err(Error::Unsupported(
                "SensitivityProcedure needs a *SENSITIVITY step".into(),
            ));
        };
        if parameters.nlgeom {
            return Err(Error::Unsupported(
                "*SENSITIVITY, NLGEOM is not implemented yet".into(),
            ));
        }
        let requests = parse_responses(ctx, parameters)?;
        if let Some(filter) = &parameters.filter {
            if !filter.kind.eq_ignore_ascii_case("LINEAR") {
                return Err(Error::Unsupported(format!(
                    "*FILTER, TYPE={} is not implemented yet (only LINEAR)",
                    filter.kind
                )));
            }
            if filter.radius <= 0.0 {
                return Err(Error::input("*FILTER", "the radius must be positive"));
            }
        }

        let started = std::time::Instant::now();
        let nodes = design_nodes(ctx, step)?;
        let active: Vec<bool> = nodes.iter().map(|n| n.active()).collect();
        let active_count = active.iter().filter(|a| **a).count();
        progress.event(ProgressEvent::Info(format!(
            "{} design nodes on the surface, {active_count} of them free to move ({:.2} s)",
            nodes.len(),
            started.elapsed().as_secs_f64()
        )));
        if active_count == 0 {
            return Err(Error::input(
                "*DESIGN VARIABLES",
                "no design node lies on a free surface of a solid element",
            ));
        }

        // Inputs of the responses.
        let needs_u = requests
            .iter()
            .any(|(k, _)| *k == ResponseKind::StrainEnergy);
        let needs_modes = requests
            .iter()
            .any(|(k, _)| *k == ResponseKind::Eigenfrequency);
        let u = if needs_u {
            if ctx.state.steps_done == 0
                || ctx.state.displacements.values.len() != ctx.table.dofs.len()
            {
                return Err(Error::input(
                    "*DESIGN RESPONSE",
                    "STRAINENERGY needs a *STATIC step before the sensitivity step",
                ));
            }
            Some(ctx.state.displacements.values.clone())
        } else {
            None
        };
        let adjoint = match (
            &u,
            requests
                .iter()
                .find(|(k, _)| *k == ResponseKind::StrainEnergy),
        ) {
            (Some(u), Some((_, Some(set)))) => {
                Some(strain_energy_adjoint(ctx, step, progress, u, set)?)
            }
            (Some(u), Some((_, None))) => Some(u.clone()),
            _ => None,
        };
        let stored = if needs_modes {
            match &ctx.state.stored_modes {
                Some(modes) if !modes.modes.is_empty() => Some(modes.clone()),
                _ => {
                    return Err(Error::input(
                        "*DESIGN RESPONSE",
                        "EIGENFREQUENCY needs a *FREQUENCY, STORAGE=YES step before the sensitivity step",
                    ));
                }
            }
        } else {
            None
        };
        let (eigenvalues, mode_vectors): (Vec<f64>, Vec<&[f64]>) = match &stored {
            Some(s) => (
                s.eigenvalues.clone(),
                s.modes.iter().map(|m| m.values.as_slice()).collect(),
            ),
            None => (Vec::new(), Vec::new()),
        };
        let loads = element_loads(ctx, step)?;
        let problem = Problem {
            model: ctx.model,
            table: ctx.table,
            requests: &requests,
            u,
            adjoint,
            loads,
            // One mode at a time: repeated eigenvalues keep the basis of the stored modes, as
            // CalculiX does (their sum over a cluster does not depend on that basis).
            clusters: (0..eigenvalues.len()).map(|j| j..j + 1).collect(),
            eigenvalues,
            modes: mode_vectors,
        };

        // Derivatives of the moving nodes, in parallel.
        let started = std::time::Instant::now();
        let derivatives: Vec<Result<Option<NodeDerivatives>>> = nodes
            .par_iter()
            .enumerate()
            .map(|(i, node)| {
                if active[i] && node.mids.is_empty() {
                    problem.derivatives(node).map(Some)
                } else {
                    Ok(None)
                }
            })
            .collect();
        let mut raw: Vec<Option<NodeDerivatives>> = Vec::with_capacity(nodes.len());
        for d in derivatives {
            raw.push(d?);
        }
        progress.event(ProgressEvent::Info(format!(
            "sensitivities of {} responses in {:.2} s",
            requests.len(),
            started.elapsed().as_secs_f64()
        )));

        // Per-area fields, one per response (and per mode).
        let mut fields: Vec<(ResponseKind, Option<usize>, Vec<f64>)> = Vec::new();
        for (r, (kind, _)) in requests.iter().enumerate() {
            let per_response = match kind {
                ResponseKind::Eigenfrequency => problem.modes.len(),
                _ => 1,
            };
            for j in 0..per_response {
                let mut values = vec![0.0; nodes.len()];
                for (i, node) in nodes.iter().enumerate() {
                    if let Some(d) = &raw[i]
                        && node.area > 0.0
                    {
                        values[i] = d.values[r][j] / node.area;
                    }
                }
                let corners: Vec<f64> = nodes
                    .iter()
                    .map(|node| {
                        if node.mids.is_empty() {
                            0.0
                        } else {
                            node.mids.iter().map(|&m| values[m]).sum::<f64>()
                                / node.mids.len() as f64
                        }
                    })
                    .collect();
                for (i, node) in nodes.iter().enumerate() {
                    if !node.mids.is_empty() && active[i] {
                        values[i] = corners[i];
                    }
                }
                fields.push((
                    *kind,
                    matches!(kind, ResponseKind::Eigenfrequency).then_some(j),
                    values,
                ));
            }
        }

        // Response values.
        let mass_value = |set: &Option<Vec<bool>>| -> Result<f64> {
            let mut total = 0.0;
            for (e, entry) in ctx.table.entries.iter().enumerate() {
                if set.as_ref().is_none_or(|s| s[e]) {
                    let rho = entry
                        .density
                        .ok_or_else(|| missing_density(ctx.model, ctx.table, e as u32))?;
                    let coords = ctx.table.coordinates(ctx.model, entry);
                    total += rho * ctx.table.formulation(entry).volume(&coords);
                }
            }
            Ok(total)
        };
        let energy_value = |set: &Option<Vec<bool>>| -> f64 {
            let Some(u) = &problem.u else {
                return 0.0;
            };
            let mut total = 0.0;
            let mut ke = Vec::new();
            for (e, entry) in ctx.table.entries.iter().enumerate() {
                if set.as_ref().is_none_or(|s| s[e]) {
                    let formulation = ctx.table.formulation(entry);
                    let coords = ctx.table.coordinates(ctx.model, entry);
                    formulation.stiffness(&coords, ctx.table.material_matrix(entry), &mut ke);
                    let ue = ctx.table.gather(entry, u);
                    let n = ue.len();
                    for i in 0..n {
                        for j in 0..n {
                            total += 0.5 * ue[i] * ke[i * n + j] * ue[j];
                        }
                    }
                }
            }
            total
        };

        // Output: normals, then the fields in the order of the responses.
        let order = node_order(ctx.model);
        let dense_to_position: HashMap<u32, usize> =
            nodes.iter().enumerate().map(|(i, n)| (n.node, i)).collect();
        let field_of = |name: &str, components: &[&str], per_component: &[&[f64]]| -> ResultField {
            ResultField {
                name: name.to_string(),
                components: components.iter().map(|c| c.to_string()).collect(),
                nodes: order.iter().map(|&(id, _)| id).collect(),
                values: per_component
                    .iter()
                    .map(|values| {
                        order
                            .iter()
                            .map(|&(_, dense)| {
                                dense_to_position.get(&dense).map_or(0.0, |&p| values[p])
                            })
                            .collect()
                    })
                    .collect(),
            }
        };
        let writes_sen = step.outputs.node_file.iter().any(|o| {
            o.variables
                .iter()
                .any(|v| v.trim().eq_ignore_ascii_case("SEN"))
        });
        let normals: Vec<Vec<f64>> = (0..3)
            .map(|k| nodes.iter().map(|n| n.normal[k]).collect())
            .collect();
        if writes_sen {
            let field = field_of(
                "NORM",
                &["NORMX", "NORMY", "NORMZ"],
                &[&normals[0], &normals[1], &normals[2]],
            );
            ctx.results.increments.push(ResultsIncrement {
                step: ctx.step_number,
                increment: 1,
                kind: IncrementKind::Sensitivity,
                value: 0.0,
                total_time: ctx.state.total_time,
                nodal_diameter: None,
                fields: vec![field],
            });
        }
        let mut table_rows: Vec<(u32, Vec<f64>)> = Vec::new();
        let mut table_names: Vec<String> = Vec::new();
        for (kind, mode, values) in &fields {
            let request = requests.iter().position(|(k, _)| k == kind).unwrap();
            let length = values.iter().map(|v| v * v).sum::<f64>().sqrt();
            let filtered = match &parameters.filter {
                Some(filter) => linear_filter(ctx.model, &nodes, &active, filter.radius, values),
                None => values.clone(),
            };
            let filtered = normalized(&filtered);
            let (name, frd_name, value) = match kind {
                ResponseKind::Mass => ("MASS", "SENMASS", mass_value(&requests[request].1)?),
                ResponseKind::StrainEnergy => (
                    "STRAINENERGY",
                    "SENENER",
                    energy_value(&requests[request].1),
                ),
                ResponseKind::Eigenfrequency => (
                    "EIGENFREQUENCY",
                    "SENFREQ",
                    problem.eigenvalues[mode.unwrap()],
                ),
            };
            progress.event(ProgressEvent::Info(format!(
                "{name}{}: value {value:e}, gradient length {length:e}",
                mode.map_or(String::new(), |m| format!(" (mode {})", m + 1))
            )));
            if writes_sen {
                let field = field_of(frd_name, &["DFDN", "DFDNFIL"], &[values, &filtered]);
                let frequency = match kind {
                    ResponseKind::Eigenfrequency => {
                        value.max(0.0).sqrt() / (2.0 * std::f64::consts::PI)
                    }
                    _ => 0.0,
                };
                ctx.results.increments.push(ResultsIncrement {
                    step: ctx.step_number,
                    increment: 1,
                    kind: IncrementKind::Sensitivity,
                    value: frequency,
                    total_time: ctx.state.total_time,
                    nodal_diameter: None,
                    fields: vec![field],
                });
            }
            // The eigenfrequencies get a table per mode, the other responses share one.
            if *kind == ResponseKind::Eigenfrequency {
                ctx.results.history.push(HistoryBlock {
                    step: ctx.step_number,
                    increment: 0,
                    quantity: DESIGN_RESPONSE_TABLE.to_string(),
                    set: name.to_string(),
                    time: 0.0,
                    rows: vec![(0, vec![value, length])],
                });
            } else {
                table_rows.push((table_names.len() as u32, vec![value, length]));
                table_names.push(name.to_string());
            }
        }
        if !table_rows.is_empty() {
            ctx.results.history.push(HistoryBlock {
                step: ctx.step_number,
                increment: 0,
                quantity: DESIGN_RESPONSE_TABLE.to_string(),
                set: table_names.join(","),
                time: 0.0,
                rows: table_rows,
            });
        }
        ctx.sta.push(StaRow {
            step: ctx.step_number,
            increment: 1,
            attempt: 1,
            iterations: 1,
            total_time: ctx.state.total_time,
            step_time: 0.0,
            increment_size: 0.0,
        });
        ctx.state.steps_done = ctx.step_number;
        Ok(StepOutcome::Converged)
    }
}

/// The responses of the step with their element sets.
fn parse_responses(
    ctx: &ProcedureContext<'_>,
    parameters: &Sensitivity,
) -> Result<Vec<(ResponseKind, Option<Vec<bool>>)>> {
    let mut requests = Vec::new();
    for response in &parameters.responses {
        let kind = match response.kind.as_str() {
            "MASS" => ResponseKind::Mass,
            "STRAINENERGY" => ResponseKind::StrainEnergy,
            "EIGENFREQUENCY" => ResponseKind::Eigenfrequency,
            other => {
                return Err(Error::Unsupported(format!(
                    "*DESIGN RESPONSE {other} is not implemented yet (MASS, STRAINENERGY and EIGENFREQUENCY are)"
                )));
            }
        };
        let set = match kind {
            ResponseKind::Eigenfrequency => None,
            _ => response_elements(ctx.model, ctx.table, response.elset.as_deref())?,
        };
        requests.push((kind, set));
    }
    if requests.is_empty() {
        return Err(Error::input(
            "*SENSITIVITY",
            "the step needs at least one *DESIGN RESPONSE",
        ));
    }
    Ok(requests)
}

#[cfg(test)]
mod tests {
    use super::polygon_area;

    #[test]
    fn polygon_area_of_a_square() {
        let a = polygon_area(&[[0.0; 3], [2.0, 0.0, 0.0], [2.0, 2.0, 0.0], [0.0, 2.0, 0.0]]);
        assert!((a - 4.0).abs() < 1e-12);
    }
}
