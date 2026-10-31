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
//! No CalculiX code was consulted.

use std::collections::{HashMap, HashSet};

use plxs_core::constraints::Tie;
use plxs_core::mesh::Surface;
use plxs_core::{Error, Model, Result};
use plxs_elements::shape::face_shape;
use plxs_elements::solid;

/// A master face: dense node indices (corners first, then midside nodes) and its shape.
#[derive(Clone, Debug)]
pub struct MasterFace {
    pub nodes: Vec<u32>,
    pub corners: usize,
    pub quadratic: bool,
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
        let family = solid(element.kind).map(|s| s.family).ok_or_else(|| {
            Error::Unsupported(format!(
                "{keyword}: faces of {} elements cannot be tied yet (element {})",
                element.kind, element.id
            ))
        })?;
        let table = family.faces();
        let f = table
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
        let quadratic = family.is_quadratic();
        let local = f
            .corners
            .iter()
            .chain(if quadratic { f.mids } else { &[] }.iter());
        let nodes = local
            .map(|&k| {
                let id = element.nodes[k];
                model
                    .nodes
                    .index(id)
                    .ok_or_else(|| Error::undefined(keyword, "node", id.to_string()))
            })
            .collect::<Result<Vec<u32>>>()?;
        out.push(MasterFace {
            nodes,
            corners: f.corners.len(),
            quadratic,
        });
    }
    Ok(out)
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

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

impl MasterFace {
    fn clamp(&self, p: [f64; 2]) -> [f64; 2] {
        if self.corners == 4 {
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
        face_shape(self.corners, self.quadratic, p, &mut n, &mut dn);
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
        let mut p = if self.corners == 4 {
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
            let det = a * c - b * b;
            if det.abs() < 1e-300 {
                break;
            }
            let step = [(c * g[0] - b * g[1]) / det, (a * g[1] - b * g[0]) / det];
            let next = self.clamp([p[0] + step[0], p[1] + step[1]]);
            let moved = (next[0] - p[0]).abs() + (next[1] - p[1]).abs();
            p = next;
            if moved < 1e-12 {
                break;
            }
        }
        let (n, y, _) = self.evaluate(coords, p);
        (n, y)
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
        (lo, hi)
    }

    /// Mean length of the corner edges.
    fn edge_length(&self, coords: &[[f64; 3]]) -> f64 {
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
/// PrePoLix writes).
const DEFAULT_TOLERANCE: f64 = 0.025;

/// The couplings of a tie and the slave nodes that found no master face.
pub fn couplings(
    model: &Model,
    tie: &Tie,
    coords: &[[f64; 3]],
) -> Result<(Vec<Coupling>, Vec<u32>)> {
    let faces = surface_faces(model, "*TIE", &tie.master)?;
    let slaves = surface_nodes(model, "*TIE", &tie.slave)?;
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
    for (index, face) in faces.iter().enumerate() {
        let (lo, hi) = face.bounds(coords);
        let lo = lo.map(|v| key(v - tolerance));
        let hi = hi.map(|v| key(v + tolerance));
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
                let (n, y) = faces[f as usize].project(coords, x);
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
                });
            }
            _ => missed.push(slave),
        }
    }
    Ok((out, missed))
}
