//! Faces of contact surfaces: dense node lists, shape functions, normals, closest point
//! projection, consistent nodal areas, the integration of a slave face and a grid for the
//! candidate search.
//!
//! Faces are taken from the element families of `plxs-elements` (corners counter-clockwise
//! seen from outside the element, so that `t_ξ × t_η` points out of the element). Faces of
//! plane and axisymmetric elements are their edges, extruded across the thickness (plane)
//! or around the axis (axisymmetric, full circumference): a line in natural coordinate `ξ`
//! whose second tangent is the out-of-plane axis. Closest
//! point projection by Gauss-Newton on the face's natural coordinates (Wriggers,
//! "Computational Contact Mechanics", chapter 4.1). No CalculiX code was consulted.

use std::collections::{HashMap, HashSet};

use plxs_core::mesh::Surface;
use plxs_core::section::SectionKind;
use plxs_core::{ElementFamily, Error, Model, Result};
use plxs_elements::shape::face_shape;
use plxs_elements::solid;

/// A face of an element in a contact surface.
#[derive(Clone, Debug, PartialEq)]
pub struct ContactFace {
    /// Dense element index and the user's element id.
    pub element: u32,
    pub element_id: u32,
    /// CalculiX face number (`S1` = 1).
    pub face_number: u8,
    /// Dense node indices: corners first, then midside nodes of quadratic elements.
    pub nodes: Vec<u32>,
    pub corners: usize,
    pub quadratic: bool,
    /// For an edge of a plane or axisymmetric element: how it extends out of plane.
    pub planar: Option<Planar>,
}

/// The extent of an edge face out of the plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Planar {
    /// Plane stress or strain: the section thickness.
    Thickness(f64),
    /// Axisymmetric: the full circumference `2π r` (`r` the first coordinate).
    Axisymmetric,
}

impl Planar {
    fn width(self, x: [f64; 3]) -> f64 {
        match self {
            Planar::Thickness(t) => t,
            Planar::Axisymmetric => 2.0 * std::f64::consts::PI * x[0].abs(),
        }
    }
}

/// Local corner and midside node indices of every face, whether the element is quadratic,
/// and how its faces extend out of plane.
type ElementFaces = (Vec<(Vec<usize>, Vec<usize>)>, bool, Option<Planar>);

/// The faces of an element as local corner and midside node indices, whether it is
/// quadratic and how its faces extend out of plane; `None` for elements without contact
/// faces.
fn element_faces(model: &Model, index: usize) -> Option<ElementFaces> {
    let element = &model.elements[index];
    if let Some(family) = solid(element.kind).map(|s| s.family) {
        let quadratic = family.is_quadratic();
        let faces = family
            .faces()
            .iter()
            .map(|f| {
                let mids = if quadratic {
                    f.mids.to_vec()
                } else {
                    Vec::new()
                };
                (f.corners.to_vec(), mids)
            })
            .collect();
        return Some((faces, quadratic, None));
    }
    let planar = match element.kind.family() {
        ElementFamily::PlaneStress | ElementFamily::PlaneStrain => {
            let thickness = element
                .section
                .and_then(|s| match &model.sections.get(s as usize)?.kind {
                    SectionKind::Solid { thickness, .. } => *thickness,
                    _ => None,
                })
                .filter(|t| *t > 0.0)
                .unwrap_or(1.0);
            Planar::Thickness(thickness)
        }
        ElementFamily::Axisymmetric => Planar::Axisymmetric,
        _ => return None,
    };
    // Edge k runs from corner k to corner k + 1 (counter-clockwise elements: the outward
    // normal is the edge direction × z); its midside node follows the corners.
    let count = element.kind.node_count();
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
    Some((faces, quadratic, Some(planar)))
}

/// Shape functions, point, tangents and unit normal of a face at natural coordinates.
#[derive(Clone, Debug)]
pub struct FacePoint {
    pub n: Vec<f64>,
    pub x: [f64; 3],
    /// Tangents `∂x/∂ξ`, `∂x/∂η`.
    pub t: [[f64; 3]; 2],
    /// Unit normal `t_ξ × t_η / |t_ξ × t_η|`, pointing out of the element.
    pub normal: [f64; 3],
    /// `|t_ξ × t_η|`: area per unit natural area.
    pub jacobian: f64,
}

pub fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

pub fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub fn normalized(a: [f64; 3]) -> [f64; 3] {
    let l = norm(a);
    if l > 0.0 { scale(a, 1.0 / l) } else { a }
}

impl ContactFace {
    /// The faces of an element surface, in the order of the surface definition.
    pub fn surface_faces(model: &Model, keyword: &str, name: &str) -> Result<Vec<ContactFace>> {
        let surface = model
            .sets
            .surfaces
            .get(name)
            .ok_or_else(|| Error::undefined(keyword, "surface", name))?;
        let Surface::Element(faces) = surface else {
            return Err(Error::input(
                keyword,
                format!("surface {name} must consist of element faces"),
            ));
        };
        let mut out = Vec::with_capacity(faces.len());
        for face in faces {
            let index = model
                .element_index(face.element)
                .ok_or_else(|| Error::undefined(keyword, "element", face.element.to_string()))?;
            let element = &model.elements[index as usize];
            let (table, quadratic, planar) =
                element_faces(model, index as usize).ok_or_else(|| {
                    Error::Unsupported(format!(
                        "{keyword}: contact on faces of {} elements is not implemented yet (element {})",
                        element.kind, element.id
                    ))
                })?;
            let (corners, mids) =
                table
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
            out.push(ContactFace {
                element: index,
                element_id: element.id,
                face_number: face.face,
                nodes,
                corners: corners.len(),
                quadratic,
                planar,
            });
        }
        Ok(out)
    }

    /// The exterior faces of the elements of `nodes` whose nodes all belong to the set:
    /// what a node surface stands for when areas are needed (node-to-surface slaves).
    pub fn faces_of_node_set(model: &Model, nodes: &[u32]) -> Vec<ContactFace> {
        let set: HashSet<u32> = nodes.iter().copied().collect();
        // Every face of every solid element, keyed by its sorted corner nodes.
        let mut faces: HashMap<Vec<u32>, Vec<ContactFace>> = HashMap::new();
        for (index, element) in model.elements.iter().enumerate() {
            let Some((table, quadratic, planar)) = element_faces(model, index) else {
                continue;
            };
            let dense: Option<Vec<u32>> = element
                .nodes
                .iter()
                .map(|&id| model.nodes.index(id))
                .collect();
            let Some(dense) = dense else { continue };
            if !dense.iter().any(|n| set.contains(n)) {
                continue;
            }
            for (k, (corners, mids)) in table.iter().enumerate() {
                let face_nodes: Vec<u32> = corners.iter().chain(mids).map(|&i| dense[i]).collect();
                let mut key: Vec<u32> = corners.iter().map(|&i| dense[i]).collect();
                key.sort_unstable();
                faces.entry(key).or_default().push(ContactFace {
                    element: index as u32,
                    element_id: element.id,
                    face_number: k as u8 + 1,
                    nodes: face_nodes,
                    corners: corners.len(),
                    quadratic,
                    planar,
                });
            }
        }
        let mut out: Vec<ContactFace> = faces
            .into_values()
            .filter(|list| list.len() == 1)
            .map(|mut list| list.pop().unwrap())
            .filter(|f| f.nodes.iter().all(|n| set.contains(n)))
            .collect();
        out.sort_by_key(|f| (f.element, f.face_number));
        out
    }

    /// Shape functions and geometry at natural coordinates.
    pub fn at(&self, coords: &[[f64; 3]], p: [f64; 2]) -> FacePoint {
        let m = self.nodes.len();
        let mut n = vec![0.0; m];
        let mut dn = vec![[0.0; 2]; m];
        self.shape(self.quadratic, p, &mut n, &mut dn);
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
        if let Some(planar) = self.planar {
            // An edge: the second tangent is the out-of-plane axis, the area per unit ξ
            // the edge length times the width out of plane.
            t[1] = [0.0, 0.0, 1.0];
            let c = cross(t[0], t[1]);
            let length = norm(c);
            return FacePoint {
                n,
                x,
                t,
                normal: if length > 0.0 {
                    scale(c, 1.0 / length)
                } else {
                    [0.0, 1.0, 0.0]
                },
                jacobian: length * planar.width(x),
            };
        }
        let c = cross(t[0], t[1]);
        let jacobian = norm(c);
        let normal = if jacobian > 0.0 {
            scale(c, 1.0 / jacobian)
        } else {
            [0.0, 0.0, 1.0]
        };
        FacePoint {
            n,
            x,
            t,
            normal,
            jacobian,
        }
    }

    /// Shape functions and their natural derivatives: those of `plxs-elements` for faces,
    /// the line's for edges (corners at ξ = ∓1, the midside node at 0).
    fn shape(&self, quadratic: bool, p: [f64; 2], n: &mut [f64], dn: &mut [[f64; 2]]) {
        if self.corners != 2 {
            face_shape(self.corners, quadratic, p, n, dn);
            return;
        }
        let x = p[0];
        if quadratic {
            n[0] = 0.5 * x * (x - 1.0);
            n[1] = 0.5 * x * (x + 1.0);
            n[2] = 1.0 - x * x;
            dn[0] = [x - 0.5, 0.0];
            dn[1] = [x + 0.5, 0.0];
            dn[2] = [-2.0 * x, 0.0];
        } else {
            n[0] = 0.5 * (1.0 - x);
            n[1] = 0.5 * (1.0 + x);
            dn[0] = [-0.5, 0.0];
            dn[1] = [0.5, 0.0];
        }
    }

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

    /// The centre of the face in natural coordinates.
    pub fn centre(&self) -> [f64; 2] {
        if self.corners <= 4 {
            [0.0, 0.0]
        } else {
            [1.0 / 3.0, 1.0 / 3.0]
        }
    }

    /// Closest point projection of `x` on the face (clamped to the face): natural
    /// coordinates and the geometry there.
    pub fn project(&self, coords: &[[f64; 3]], x: [f64; 3]) -> ([f64; 2], FacePoint) {
        let mut p = self.centre();
        for _ in 0..30 {
            let fp = self.at(coords, p);
            let r = sub(x, fp.x);
            let g = [dot(fp.t[0], r), dot(fp.t[1], r)];
            let a = dot(fp.t[0], fp.t[0]);
            let b = dot(fp.t[0], fp.t[1]);
            let c = dot(fp.t[1], fp.t[1]);
            let det = a * c - b * b;
            if det.abs() < 1e-300 {
                break;
            }
            let step = [(c * g[0] - b * g[1]) / det, (a * g[1] - b * g[0]) / det];
            let next = self.clamp([p[0] + step[0], p[1] + step[1]]);
            let moved = (next[0] - p[0]).abs() + (next[1] - p[1]).abs();
            p = next;
            if moved < 1e-13 {
                break;
            }
        }
        let fp = self.at(coords, p);
        (p, fp)
    }

    /// Whether natural coordinates lie strictly inside the face (not on its clamped border).
    pub fn is_interior(&self, p: [f64; 2]) -> bool {
        if self.corners == 2 {
            p[0].abs() < 1.0 - 1e-9
        } else if self.corners == 4 {
            p[0].abs() < 1.0 - 1e-9 && p[1].abs() < 1.0 - 1e-9
        } else {
            p[0] > 1e-9 && p[1] > 1e-9 && p[0] + p[1] < 1.0 - 1e-9
        }
    }

    /// Whether natural coordinates lie on the face, edges included up to `tolerance`.
    pub fn contains(&self, p: [f64; 2], tolerance: f64) -> bool {
        if self.corners == 2 {
            p[0].abs() <= 1.0 + tolerance
        } else if self.corners == 4 {
            p[0].abs() <= 1.0 + tolerance && p[1].abs() <= 1.0 + tolerance
        } else {
            p[0] >= -tolerance && p[1] >= -tolerance && p[0] + p[1] <= 1.0 + tolerance
        }
    }

    /// Bounding box of the face nodes.
    pub fn bounds(&self, coords: &[[f64; 3]]) -> ([f64; 3], [f64; 3]) {
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
    pub fn edge_length(&self, coords: &[[f64; 3]]) -> f64 {
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

    /// Integration points of the face as CalculiX's face-to-face contact uses them (observed
    /// on contact print output: a quadrilateral slave face is split into four triangles about
    /// its centre, each integrated with the 7-point rule; a triangle takes the 7-point rule
    /// as a whole). Returns natural coordinates and weights in natural area.
    pub fn integration(&self) -> (Vec<[f64; 2]>, Vec<f64>) {
        let (tp, tw) = triangle_rule_7();
        let mut points = Vec::new();
        let mut weights = Vec::new();
        if self.corners == 2 {
            // An edge stands for the quadrilateral face of the expanded element: the same
            // points, their ξ merged (the rule integrates exactly across the thickness).
            let (quad, quad_weights) = ContactFace {
                corners: 4,
                quadratic: false,
                planar: None,
                ..self.clone()
            }
            .integration();
            for (p, w) in quad.iter().zip(&quad_weights) {
                match points
                    .iter()
                    .position(|q: &[f64; 2]| (q[0] - p[0]).abs() < 1e-12)
                {
                    Some(i) => weights[i] += 0.5 * w,
                    None => {
                        points.push([p[0], 0.0]);
                        weights.push(0.5 * w);
                    }
                }
            }
        } else if self.corners == 4 {
            const C: [[f64; 2]; 4] = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
            for i in 0..4 {
                let (a, b) = (C[i], C[(i + 1) % 4]);
                // Triangle centre-a-b in natural coordinates, area 1 in natural measure.
                for (p, w) in tp.iter().zip(&tw) {
                    let (l1, l2) = (p[0], p[1]);
                    let l0 = 1.0 - l1 - l2;
                    points.push([l1 * a[0] + l2 * b[0], l1 * a[1] + l2 * b[1]]);
                    let _ = l0;
                    // The triangle's natural area is 1 (a quarter of 4); the rule's weights
                    // sum to 1/2 on the unit triangle.
                    weights.push(w * 2.0);
                }
            }
        } else {
            for (p, w) in tp.iter().zip(&tw) {
                points.push(*p);
                weights.push(*w);
            }
        }
        (points, weights)
    }

    /// Area of the face.
    pub fn area(&self, coords: &[[f64; 3]]) -> f64 {
        let (points, weights) = self.integration();
        points
            .iter()
            .zip(&weights)
            .map(|(p, w)| w * self.at(coords, *p).jacobian)
            .sum()
    }

    /// Consistent nodal areas `∫ N_i dA` of the face.
    pub fn nodal_areas(&self, coords: &[[f64; 3]]) -> Vec<f64> {
        let (points, weights) = self.integration();
        let mut out = vec![0.0; self.nodes.len()];
        for (p, w) in points.iter().zip(&weights) {
            let fp = self.at(coords, *p);
            for (o, n) in out.iter_mut().zip(&fp.n) {
                *o += w * fp.jacobian * n;
            }
        }
        out
    }

    /// Values at the face nodes from values at the integration points of [`Self::integration`]
    /// by a least-squares fit of the face's shape functions (the nodal extrapolation of
    /// contact results).
    pub fn extrapolate(&self, values: &[f64]) -> Vec<f64> {
        let (points, weights) = self.integration();
        let m = self.corners;
        // Normal equations with the corner shape functions only (bilinear/linear fit).
        let mut a = vec![0.0; m * m];
        let mut b = vec![0.0; m];
        let mut n = vec![0.0; self.nodes.len()];
        let mut dn = vec![[0.0; 2]; self.nodes.len()];
        for ((p, w), v) in points.iter().zip(&weights).zip(values) {
            self.shape(false, *p, &mut n[..m], &mut dn[..m]);
            for i in 0..m {
                b[i] += w * n[i] * v;
                for j in 0..m {
                    a[i * m + j] += w * n[i] * n[j];
                }
            }
        }
        let corners = solve_dense(m, &mut a, &mut b);
        let mut out = corners.clone();
        // Midside nodes: the mean of their corners.
        if self.quadratic && m == 2 {
            out.push(0.5 * (corners[0] + corners[1]));
        } else if self.quadratic {
            for k in 0..m {
                out.push(0.5 * (corners[k] + corners[(k + 1) % m]));
            }
        }
        out
    }
}

/// Solves a small dense symmetric system in place (Gaussian elimination with pivoting).
fn solve_dense(n: usize, a: &mut [f64], b: &mut [f64]) -> Vec<f64> {
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&i, &j| a[i * n + col].abs().total_cmp(&a[j * n + col].abs()))
            .unwrap();
        if pivot != col {
            for k in 0..n {
                a.swap(col * n + k, pivot * n + k);
            }
            b.swap(col, pivot);
        }
        let p = a[col * n + col];
        if p.abs() < 1e-300 {
            continue;
        }
        for row in col + 1..n {
            let f = a[row * n + col] / p;
            if f == 0.0 {
                continue;
            }
            for k in col..n {
                a[row * n + k] -= f * a[col * n + k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let mut s = b[row];
        for k in row + 1..n {
            s -= a[row * n + k] * x[k];
        }
        let p = a[row * n + row];
        x[row] = if p.abs() < 1e-300 { 0.0 } else { s / p };
    }
    x
}

/// The 7-point degree-5 rule on the unit triangle (weights sum to 1/2).
fn triangle_rule_7() -> (Vec<[f64; 2]>, Vec<f64>) {
    let a1 = 0.059_715_871_789_77;
    let b1 = 0.470_142_064_105_115;
    let a2 = 0.797_426_985_353_087;
    let b2 = 0.101_286_507_323_456;
    let w0 = 0.225 / 2.0;
    let w1 = 0.132_394_152_788_506 / 2.0;
    let w2 = 0.125_939_180_544_827 / 2.0;
    (
        vec![
            [1.0 / 3.0, 1.0 / 3.0],
            [a1, b1],
            [b1, a1],
            [b1, b1],
            [a2, b2],
            [b2, a2],
            [b2, b2],
        ],
        vec![w0, w1, w1, w1, w2, w2, w2],
    )
}

/// A uniform grid over the bounding boxes of faces for candidate searches.
pub struct FaceGrid {
    cell: f64,
    cells: HashMap<(i64, i64, i64), Vec<u32>>,
}

impl FaceGrid {
    /// Faces in cells; every face is registered in all cells its box inflated by `margin`
    /// touches.
    pub fn new(faces: &[ContactFace], coords: &[[f64; 3]], cell: f64, margin: f64) -> Self {
        let cell = cell.max(1e-12);
        let key = |x: f64| (x / cell).floor() as i64;
        let mut cells: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
        for (index, face) in faces.iter().enumerate() {
            let (lo, hi) = face.bounds(coords);
            let lo = lo.map(|v| key(v - margin));
            let hi = hi.map(|v| key(v + margin));
            for i in lo[0]..=hi[0] {
                for j in lo[1]..=hi[1] {
                    for k in lo[2]..=hi[2] {
                        cells.entry((i, j, k)).or_default().push(index as u32);
                    }
                }
            }
        }
        FaceGrid { cell, cells }
    }

    pub fn candidates(&self, x: [f64; 3]) -> &[u32] {
        let key = |v: f64| (v / self.cell).floor() as i64;
        self.cells
            .get(&(key(x[0]), key(x[1]), key(x[2])))
            .map_or(&[], Vec::as_slice)
    }
}

/// The nodes of a surface (element faces or nodes), dense, unique, in order.
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
            for face in ContactFace::surface_faces(model, keyword, name)? {
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

/// The coefficients `A = D M⁻¹` (row-major, `m × m`) of the dual shape functions of a face
/// from its mass matrix `M = ∫N Nᵀ` and `D = diag ∫N` (Gauss-Jordan; `M` is symmetric
/// positive definite).
pub fn dual_coefficients(mass: &[f64], diag: &[f64], m: usize) -> Vec<f64> {
    // Solve M X = I, then A = D X (M symmetric, so D M⁻¹ = D X).
    let mut a = mass.to_vec();
    let mut inv = vec![0.0; m * m];
    for i in 0..m {
        inv[i * m + i] = 1.0;
    }
    for col in 0..m {
        let pivot_row = (col..m)
            .max_by(|&x, &y| a[x * m + col].abs().total_cmp(&a[y * m + col].abs()))
            .unwrap_or(col);
        if pivot_row != col {
            for j in 0..m {
                a.swap(col * m + j, pivot_row * m + j);
                inv.swap(col * m + j, pivot_row * m + j);
            }
        }
        let pivot = a[col * m + col];
        if pivot.abs() <= 1e-300 {
            continue;
        }
        for j in 0..m {
            a[col * m + j] /= pivot;
            inv[col * m + j] /= pivot;
        }
        for r in 0..m {
            if r != col {
                let factor = a[r * m + col];
                if factor != 0.0 {
                    for j in 0..m {
                        a[r * m + j] -= factor * a[col * m + j];
                        inv[r * m + j] -= factor * inv[col * m + j];
                    }
                }
            }
        }
    }
    let mut out = vec![0.0; m * m];
    for k in 0..m {
        for j in 0..m {
            out[k * m + j] = diag[k] * inv[k * m + j];
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quad() -> (ContactFace, Vec<[f64; 3]>) {
        (
            ContactFace {
                element: 0,
                element_id: 1,
                face_number: 2,
                nodes: vec![0, 1, 2, 3],
                corners: 4,
                quadratic: false,
                planar: None,
            },
            vec![
                [0.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [2.0, 3.0, 0.0],
                [0.0, 3.0, 0.0],
            ],
        )
    }

    #[test]
    fn area_and_nodal_areas_of_a_rectangle() {
        let (f, c) = quad();
        assert!((f.area(&c) - 6.0).abs() < 1e-12);
        let a = f.nodal_areas(&c);
        for v in a {
            assert!((v - 1.5).abs() < 1e-12, "{v}");
        }
        let fp = f.at(&c, [0.0, 0.0]);
        assert!((fp.normal[2] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn projection_finds_the_foot_point() {
        let (f, c) = quad();
        let (p, fp) = f.project(&c, [0.5, 2.0, 0.7]);
        assert!((fp.x[0] - 0.5).abs() < 1e-10 && (fp.x[1] - 2.0).abs() < 1e-10);
        assert!(f.is_interior(p));
        let (p, fp) = f.project(&c, [5.0, 2.0, 0.0]);
        assert!((fp.x[0] - 2.0).abs() < 1e-10);
        assert!(!f.is_interior(p));
    }

    #[test]
    fn extrapolation_reproduces_a_bilinear_field() {
        let (f, c) = quad();
        let (points, _) = f.integration();
        let values: Vec<f64> = points
            .iter()
            .map(|p| {
                let x = f.at(&c, *p).x;
                1.0 + 2.0 * x[0] - 0.5 * x[1] + 0.25 * x[0] * x[1]
            })
            .collect();
        let nodal = f.extrapolate(&values);
        for (n, v) in nodal.iter().enumerate() {
            let x = c[n];
            let exact = 1.0 + 2.0 * x[0] - 0.5 * x[1] + 0.25 * x[0] * x[1];
            assert!((v - exact).abs() < 1e-9, "node {n}: {v} vs {exact}");
        }
    }

    #[test]
    fn dual_shape_functions_are_biorthogonal() {
        // Unit square, bilinear N, 2x2 Gauss.
        let g = 0.5 / 3f64.sqrt();
        let pts = [
            (0.5 - g, 0.5 - g),
            (0.5 + g, 0.5 - g),
            (0.5 + g, 0.5 + g),
            (0.5 - g, 0.5 + g),
        ];
        let n = |x: f64, y: f64| [(1.0 - x) * (1.0 - y), x * (1.0 - y), x * y, (1.0 - x) * y];
        let mut mass = vec![0.0; 16];
        let mut diag = vec![0.0; 4];
        for &(x, y) in &pts {
            let v = n(x, y);
            for i in 0..4 {
                diag[i] += 0.25 * v[i];
                for j in 0..4 {
                    mass[i * 4 + j] += 0.25 * v[i] * v[j];
                }
            }
        }
        let a = super::dual_coefficients(&mass, &diag, 4);
        for k in 0..4 {
            for j in 0..4 {
                let mut integral = 0.0;
                for &(x, y) in &pts {
                    let v = n(x, y);
                    let psi: f64 = (0..4).map(|l| a[k * 4 + l] * v[l]).sum();
                    integral += 0.25 * psi * v[j];
                }
                let expected = if k == j { diag[k] } else { 0.0 };
                assert!((integral - expected).abs() < 1e-12, "{k} {j}: {integral}");
            }
        }
    }
}
