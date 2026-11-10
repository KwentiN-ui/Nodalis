//! `*TIE` (tied contact): every slave node within the position tolerance of the master
//! surface is projected on to its closest master face and coupled to the face nodes with the
//! face's shape function values at the projection, `u_s = Σ N_i(ξ, η) u_i`.
//!
//! CalculiX manual, `*TIE`: the slave surface may be element faces or nodes, the master
//! surface must be element faces; nodes farther away than the tolerance are not tied (and
//! reported). With the default `ADJUST`, the slave nodes are moved on to the master surface
//! before the analysis, which CalculiX also writes into the `.frd` mesh. Projection by
//! Gauss-Newton iteration on the face's natural coordinates (Wriggers, "Computational
//! Contact Mechanics", chapter 4.1, closest point projection).
//!
//! Plane stress, plane strain and axisymmetric elements are tied along their edges (edge k
//! from corner k to corner k + 1, midside node `corners + k`), in their plane only.
//!
//! Native membranes (`M3D3`, `M3D4`, `M3D4R`) have the faces of a shell: `S1` (`SNEG`) and
//! `S2` (`SPOS`) are the element itself, `S3`.. its edges (edge k from corner k to corner
//! k + 1). CalculiX expands a membrane to a solid layer of its thickness, whose faces lie
//! half a thickness off the reference surface; plxs computes the membrane in its reference
//! surface, so a slave node anywhere within the layer (between bottom and top, shifted by
//! the section offset) counts as lying on every face of the membrane. All three
//! displacements are tied, a membrane has no rotations to couple.
//!
//! No CalculiX code was consulted.

use std::collections::{HashMap, HashSet};

use plxs_core::constraints::Tie;
use plxs_core::mesh::Surface;
use plxs_core::section::SectionKind;
use plxs_core::{ElementFamily, ElementType};
use plxs_core::{Error, Model, Result};
use plxs_elements::membrane::is_native as is_membrane;
use plxs_elements::shape::face_shape;
use plxs_elements::solid;

/// A master face: dense node indices (corners first, then midside nodes) and its shape. An
/// edge of a plane or axisymmetric element has two corners.
#[derive(Clone, Debug)]
pub struct MasterFace {
    pub nodes: Vec<u32>,
    pub corners: usize,
    pub quadratic: bool,
    /// Faces of a membrane: the layer the membrane stands for.
    pub layer: Option<Layer>,
}

/// The layer of a membrane around its reference surface: from `bottom` to `top` along the
/// element normal (signed distances), and the corner nodes that give the normal.
#[derive(Clone, Debug)]
pub struct Layer {
    pub bottom: f64,
    pub top: f64,
    pub corners: Vec<u32>,
}

impl Layer {
    /// Unit normal of the membrane element (right hand rule over its corners).
    fn normal(&self, coords: &[[f64; 3]]) -> [f64; 3] {
        let c = |i: usize| coords[self.corners[i] as usize];
        let (a, b) = if self.corners.len() == 3 {
            (sub(c(1), c(0)), sub(c(2), c(0)))
        } else {
            (sub(c(2), c(0)), sub(c(3), c(1)))
        };
        let n = [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ];
        let l = norm(n);
        if l > 0.0 { n.map(|v| v / l) } else { n }
    }

    /// The point of the layer closest to `x` above or below `y` on the reference surface.
    fn closest(&self, coords: &[[f64; 3]], x: [f64; 3], y: [f64; 3]) -> [f64; 3] {
        let n = self.normal(coords);
        let r = sub(x, y);
        let h = (r[0] * n[0] + r[1] * n[1] + r[2] * n[2]).clamp(self.bottom, self.top);
        [y[0] + h * n[0], y[1] + h * n[1], y[2] + h * n[2]]
    }
}

/// The coupling of one slave node.
#[derive(Clone, Debug, PartialEq)]
pub struct Coupling {
    pub slave: u32,
    /// Master nodes and weights (sum 1).
    pub masters: Vec<(u32, f64)>,
    /// The projection on the master surface.
    pub point: [f64; 3],
    pub distance: f64,
    /// Tied to an edge of a plane or axisymmetric element: only the two in-plane
    /// displacements are coupled.
    pub planar: bool,
}

/// The faces of an element surface as dense node lists.
pub fn surface_faces(model: &Model, keyword: &str, name: &str) -> Result<Vec<MasterFace>> {
    let surface = model
        .sets
        .surfaces
        .get(name)
        .ok_or_else(|| Error::undefined(keyword, "surface", name))?;
    let Surface::Element(faces) = surface else {
        return Err(Error::input(
            keyword,
            format!("the master surface {name} must consist of element faces"),
        ));
    };
    let mut out = Vec::with_capacity(faces.len());
    for face in faces {
        let element = model
            .element(face.element)
            .ok_or_else(|| Error::undefined(keyword, "element", face.element.to_string()))?;
        let (table, quadratic) = face_table(element.kind).ok_or_else(|| {
            Error::Unsupported(format!(
                "{keyword}: faces of {} elements cannot be tied yet (element {})",
                element.kind, element.id
            ))
        })?;
        let (corners, mids) = table
            .get((face.face as usize).wrapping_sub(1))
            .ok_or_else(|| {
                Error::input(
                    keyword,
                    format!(
                        "face S{} does not exist on element {}",
                        face.face, element.id
                    ),
                )
            })?;
        let nodes = corners
            .iter()
            .chain(mids)
            .map(|&k| {
                let id = element.nodes[k];
                model
                    .nodes
                    .index(id)
                    .ok_or_else(|| Error::undefined(keyword, "node", id.to_string()))
            })
            .collect::<Result<Vec<u32>>>()?;
        let layer = if is_membrane(element.kind) {
            let (thickness, offset) = element
                .section
                .and_then(|s| match &model.sections.get(s as usize)?.kind {
                    SectionKind::Membrane {
                        thickness, offset, ..
                    } => Some((*thickness, *offset)),
                    _ => None,
                })
                .ok_or_else(|| {
                    Error::input(
                        keyword,
                        format!("membrane element {} has no membrane section", element.id),
                    )
                })?;
            let corners = (0..element.kind.node_count().min(4))
                .map(|k| nodes_of(model, keyword, element.nodes[k]))
                .collect::<Result<Vec<u32>>>()?;
            Some(Layer {
                bottom: (offset - 0.5) * thickness,
                top: (offset + 0.5) * thickness,
                corners,
            })
        } else {
            None
        };
        out.push(MasterFace {
            nodes,
            corners: corners.len(),
            quadratic,
            layer,
        });
    }
    Ok(out)
}

fn nodes_of(model: &Model, keyword: &str, id: plxs_core::NodeId) -> Result<u32> {
    model
        .nodes
        .index(id)
        .ok_or_else(|| Error::undefined(keyword, "node", id.to_string()))
}

/// The faces of an element kind (local corner and midside nodes of each face) and whether
/// they are quadratic; `None` for kinds without tieable faces.
#[allow(clippy::type_complexity)]
fn face_table(kind: ElementType) -> Option<(Vec<(Vec<usize>, Vec<usize>)>, bool)> {
    if let Some(family) = solid(kind).map(|s| s.family) {
        let quadratic = family.is_quadratic();
        let faces = (family.faces().iter())
            .map(|f| {
                let mids = if quadratic {
                    f.mids.to_vec()
                } else {
                    Vec::new()
                };
                (f.corners.to_vec(), mids)
            })
            .collect();
        return Some((faces, quadratic));
    }
    if is_membrane(kind) {
        // S1 (SNEG) and S2 (SPOS) are the element, S3.. its edges.
        let corners = kind.node_count();
        let element: Vec<usize> = (0..corners).collect();
        let mut faces = vec![(element.clone(), Vec::new()), (element, Vec::new())];
        faces.extend((0..corners).map(|k| (vec![k, (k + 1) % corners], Vec::new())));
        return Some((faces, false));
    }
    if !matches!(
        kind.family(),
        ElementFamily::PlaneStress | ElementFamily::PlaneStrain | ElementFamily::Axisymmetric
    ) {
        return None;
    }
    let count = kind.node_count();
    let corners = if count == 3 || count == 6 { 3 } else { 4 };
    let quadratic = count > corners;
    let faces = (0..corners)
        .map(|k| {
            let mids = if quadratic {
                vec![corners + k]
            } else {
                Vec::new()
            };
            (vec![k, (k + 1) % corners], mids)
        })
        .collect();
    Some((faces, quadratic))
}

/// The slave nodes of a surface (element faces or nodes), dense, in order, unique.
pub fn surface_nodes(model: &Model, keyword: &str, name: &str) -> Result<Vec<u32>> {
    let surface = model
        .sets
        .surfaces
        .get(name)
        .ok_or_else(|| Error::undefined(keyword, "surface", name))?;
    let mut seen = HashSet::new();
    let mut out = Vec::new();
    match surface {
        Surface::Node(ids) => {
            for &id in ids {
                if let Some(i) = model.nodes.index(id)
                    && seen.insert(i)
                {
                    out.push(i);
                }
            }
        }
        Surface::Element(_) => {
            for face in surface_faces(model, keyword, name)? {
                for n in face.nodes {
                    if seen.insert(n) {
                        out.push(n);
                    }
                }
            }
        }
    }
    Ok(out)
}

/// The layers of the membrane faces of an element surface, by dense node (the first face
/// of a node gives its layer); empty for other surfaces.
fn membrane_layers(model: &Model, name: &str) -> Result<HashMap<u32, Layer>> {
    let mut out = HashMap::new();
    if let Some(Surface::Element(_)) = model.sets.surfaces.get(name) {
        for face in surface_faces(model, "*TIE", name)? {
            if let Some(layer) = face.layer {
                for &n in &face.nodes {
                    out.entry(n).or_insert_with(|| layer.clone());
                }
            }
        }
    }
    Ok(out)
}

/// Shape functions of an edge (2 corners, then the midside node) at `xi` in [-1, 1].
fn line_shape(quadratic: bool, xi: f64, n: &mut [f64], dn: &mut [[f64; 2]]) {
    if quadratic {
        n[0] = 0.5 * xi * (xi - 1.0);
        n[1] = 0.5 * xi * (xi + 1.0);
        n[2] = 1.0 - xi * xi;
        dn[0] = [xi - 0.5, 0.0];
        dn[1] = [xi + 0.5, 0.0];
        dn[2] = [-2.0 * xi, 0.0];
    } else {
        n[0] = 0.5 * (1.0 - xi);
        n[1] = 0.5 * (1.0 + xi);
        dn[0] = [-0.5, 0.0];
        dn[1] = [0.5, 0.0];
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

impl MasterFace {
    fn clamp(&self, p: [f64; 2]) -> [f64; 2] {
        if self.corners == 2 {
            [p[0].clamp(-1.0, 1.0), 0.0]
        } else if self.corners == 4 {
            [p[0].clamp(-1.0, 1.0), p[1].clamp(-1.0, 1.0)]
        } else {
            let (mut x, mut y) = (p[0].max(0.0), p[1].max(0.0));
            let s = x + y;
            if s > 1.0 {
                x /= s;
                y /= s;
            }
            [x, y]
        }
    }

    /// Shape function values and the point at natural coordinates.
    fn evaluate(&self, coords: &[[f64; 3]], p: [f64; 2]) -> (Vec<f64>, [f64; 3], [[f64; 3]; 2]) {
        let m = self.nodes.len();
        let mut n = vec![0.0; m];
        let mut dn = vec![[0.0; 2]; m];
        if self.corners == 2 {
            line_shape(self.quadratic, p[0], &mut n, &mut dn);
        } else {
            face_shape(self.corners, self.quadratic, p, &mut n, &mut dn);
        }
        let mut x = [0.0; 3];
        let mut t = [[0.0; 3]; 2];
        for (i, &node) in self.nodes.iter().enumerate() {
            let c = coords[node as usize];
            for d in 0..3 {
                x[d] += n[i] * c[d];
                t[0][d] += dn[i][0] * c[d];
                t[1][d] += dn[i][1] * c[d];
            }
        }
        (n, x, t)
    }

    /// Closest point projection of `x` on the face: natural coordinates, weights, point.
    pub fn project(&self, coords: &[[f64; 3]], x: [f64; 3]) -> (Vec<f64>, [f64; 3]) {
        let mut p = if self.corners != 3 {
            [0.0, 0.0]
        } else {
            [1.0 / 3.0, 1.0 / 3.0]
        };
        for _ in 0..25 {
            let (_, y, t) = self.evaluate(coords, p);
            let r = sub(x, y);
            let g = [
                t[0][0] * r[0] + t[0][1] * r[1] + t[0][2] * r[2],
                t[1][0] * r[0] + t[1][1] * r[1] + t[1][2] * r[2],
            ];
            let a = t[0].iter().map(|v| v * v).sum::<f64>();
            let b = (0..3).map(|d| t[0][d] * t[1][d]).sum::<f64>();
            let c = t[1].iter().map(|v| v * v).sum::<f64>();
            let step = if self.corners == 2 {
                if a < 1e-300 {
                    break;
                }
                [g[0] / a, 0.0]
            } else {
                let det = a * c - b * b;
                if det.abs() < 1e-300 {
                    break;
                }
                [(c * g[0] - b * g[1]) / det, (a * g[1] - b * g[0]) / det]
            };
            let next = self.clamp([p[0] + step[0], p[1] + step[1]]);
            let moved = (next[0] - p[0]).abs() + (next[1] - p[1]).abs();
            p = next;
            if moved < 1e-12 {
                break;
            }
        }
        let (n, y, _) = self.evaluate(coords, p);
        match &self.layer {
            Some(layer) => (n, layer.closest(coords, x, y)),
            None => (n, y),
        }
    }

    fn bounds(&self, coords: &[[f64; 3]]) -> ([f64; 3], [f64; 3]) {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for &n in &self.nodes {
            let c = coords[n as usize];
            for d in 0..3 {
                lo[d] = lo[d].min(c[d]);
                hi[d] = hi[d].max(c[d]);
            }
        }
        // A membrane's layer reaches beyond its reference surface.
        if let Some(layer) = &self.layer {
            let h = layer.bottom.abs().max(layer.top.abs());
            lo = lo.map(|v| v - h);
            hi = hi.map(|v| v + h);
        }
        (lo, hi)
    }

    /// Mean length of the corner edges (the length of an edge face).
    fn edge_length(&self, coords: &[[f64; 3]]) -> f64 {
        if self.corners == 2 {
            return norm(sub(
                coords[self.nodes[0] as usize],
                coords[self.nodes[1] as usize],
            ));
        }
        let k = self.corners;
        (0..k)
            .map(|i| {
                norm(sub(
                    coords[self.nodes[i] as usize],
                    coords[self.nodes[(i + 1) % k] as usize],
                ))
            })
            .sum::<f64>()
            / k as f64
    }
}

/// Fraction of the typical master element size used as position tolerance when the tie
/// gives none (CalculiX manual: "a default tolerance is calculated ... of the typical
/// element size"; the fraction is not legible in the manual, 2.5 % reproduces the cases
/// Nodalis writes).
const DEFAULT_TOLERANCE: f64 = 0.025;

/// The couplings of a tie and the slave nodes that found no master face.
pub fn couplings(
    model: &Model,
    tie: &Tie,
    coords: &[[f64; 3]],
) -> Result<(Vec<Coupling>, Vec<u32>)> {
    let faces = surface_faces(model, "*TIE", &tie.master)?;
    let slaves = surface_nodes(model, "*TIE", &tie.slave)?;
    let slave_layers = membrane_layers(model, &tie.slave)?;
    if faces.is_empty() {
        return Ok((Vec::new(), slaves));
    }
    let typical = faces.iter().map(|f| f.edge_length(coords)).sum::<f64>() / faces.len() as f64;
    let tolerance = match tie.position_tolerance {
        Some(t) if t > 0.0 => t,
        _ => DEFAULT_TOLERANCE * typical,
    };
    let master_nodes: HashSet<u32> = faces.iter().flat_map(|f| f.nodes.iter().copied()).collect();

    // Uniform grid over the inflated face boxes.
    let cell = typical.max(tolerance).max(1e-12);
    let key = |x: f64| (x / cell).floor() as i64;
    let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
    // Slaves on membrane faces reach as far as their layer.
    let reach = tolerance
        + (slave_layers.values())
            .map(|l| l.bottom.abs().max(l.top.abs()))
            .fold(0.0, f64::max);
    for (index, face) in faces.iter().enumerate() {
        let (lo, hi) = face.bounds(coords);
        let lo = lo.map(|v| key(v - reach));
        let hi = hi.map(|v| key(v + reach));
        for i in lo[0]..=hi[0] {
            for j in lo[1]..=hi[1] {
                for k in lo[2]..=hi[2] {
                    grid.entry((i, j, k)).or_default().push(index as u32);
                }
            }
        }
    }

    let mut out = Vec::new();
    let mut missed = Vec::new();
    for slave in slaves {
        if master_nodes.contains(&slave) {
            continue;
        }
        let x = coords[slave as usize];
        let mut best: Option<(f64, Vec<f64>, [f64; 3], u32)> = None;
        if let Some(candidates) = grid.get(&(key(x[0]), key(x[1]), key(x[2]))) {
            for &f in candidates {
                let face = &faces[f as usize];
                let (mut n, mut y) = face.project(coords, x);
                // A slave on a membrane face stands for its layer: the closest point of
                // the layer is tied, the node keeps its place in the layer.
                if let Some(layer) = slave_layers.get(&slave) {
                    let at = layer.closest(coords, y, x);
                    if at != x {
                        let (n2, y2) = face.project(coords, at);
                        n = n2;
                        y = add(y2, sub(x, at));
                    }
                }
                let d = norm(sub(x, y));
                if best.as_ref().is_none_or(|b| d < b.0) {
                    best = Some((d, n, y, f));
                }
            }
        }
        match best {
            Some((distance, n, point, f)) if distance <= tolerance => {
                let face = &faces[f as usize];
                out.push(Coupling {
                    slave,
                    masters: face.nodes.iter().copied().zip(n).collect(),
                    point,
                    distance,
                    planar: face.corners == 2 && face.layer.is_none(),
                });
            }
            _ => missed.push(slave),
        }
    }
    Ok((out, missed))
}
