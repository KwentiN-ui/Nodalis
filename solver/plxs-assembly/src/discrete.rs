//! Discrete elements: `SPRING1` (one DOF of one node to ground), `SPRING2` (between one DOF
//! of each of two nodes) with linear stiffness (`*SPRING`), and `GAPUNI` (`*GAP`), a
//! compression-only link between two nodes along a direction.
//!
//! CalculiX manual, `*SPRING` and `*GAP` and the element descriptions "SPRINGA, SPRING1,
//! SPRING2" and "GAPUNI": a spring adds `k` to its DOF (`SPRING1`) or `k [[1, -1], [-1, 1]]`
//! to its two DOFs (`SPRING2`); a gap with direction `n` and clearance `d` is closed when
//! `d + n·(u2 − u1) ≤ 0` and then acts as a penalty spring `k n nᵀ` between the nodes.
//!
//! The points of a `*COMPRESSION ONLY` surface (plxs only) are gaps too: one per integration
//! point of every face, between the point `Σ N_i u_i` of the face and fixed ground, with
//! the stiffness per area times the area of the point. A gap holds the weights `w` of its
//! nodes, its opening is `d + Σ w_i n·u_i` (`GAPUNI`: `w = [−1, 1]`).
//!
//! No CalculiX code was consulted.

use plxs_contact::geometry::ContactFace;
use plxs_core::contact::CompressionOnly;
use plxs_core::section::{SectionKind, SpringStiffness};
use plxs_core::{ElementType, Error, Model, Result};

/// A spring or gap with its global DOFs.
#[derive(Clone, Debug, PartialEq)]
pub struct DiscreteElement {
    /// Dense index of the element in the model.
    pub element: u32,
    /// Global DOFs: one or two for springs, six (3 per node) for gaps.
    pub dofs: Vec<u32>,
    pub kind: DiscreteKind,
}

#[derive(Clone, Debug, PartialEq)]
pub enum DiscreteKind {
    Spring {
        stiffness: f64,
        /// Stiffness against the displacement within the increment of a nonlinear static
        /// step (`*SPRING, STABILIZE`): no force at the start of an increment.
        stabilize: f64,
    },
    Gap {
        clearance: f64,
        /// Unit direction in which the gap closes when the weighted point moves against it
        /// (`GAPUNI`: from the first to the second node).
        direction: [f64; 3],
        stiffness: f64,
        /// Weight of every node (3 DOFs each in `dofs`) in the moving point.
        weights: Vec<f64>,
    },
}

/// Stiffness per area of a `*COMPRESSION ONLY` surface that gives none, in units of
/// `E / h` (`E` Young's modulus of the elements under the faces, `h` their mean edge
/// length): practically rigid (a closure of a tenth of the deformation of an element layer
/// under the same pressure). Stiffer, the nine gaps of a quadratic face over-constrain it
/// and the stresses at the surface stripe along the element edges (pin-loaded lug at
/// `1e3`: corner and midside nodes 3 MPa apart at 12 MPa; at `10` smooth, the peak stress
/// away from the support within 0.1 %).
pub const COMPRESSION_ONLY_STIFFNESS: f64 = 10.0;

/// Gap stiffness when `*GAP` gives none: CalculiX' manual leaves the default open; a large
/// penalty relative to typical steel models (N, mm) is used.
pub const DEFAULT_GAP_STIFFNESS: f64 = 1e12;

impl DiscreteElement {
    pub fn new(model: &Model, index: u32, nodes: &[u32]) -> Result<Self> {
        let element = &model.elements[index as usize];
        let section = element
            .section
            .and_then(|s| model.sections.get(s as usize))
            .ok_or_else(|| {
                Error::input(
                    "*ELEMENT",
                    format!("element {} has no *SPRING or *GAP", element.id),
                )
            })?;
        let dof = |node: u32, d: u8| -> Result<u32> {
            if !(1..=3).contains(&d) {
                return Err(Error::Unsupported(format!(
                    "*SPRING on degree of freedom {d} (element {}) is not implemented yet",
                    element.id
                )));
            }
            Ok(node * 3 + u32::from(d) - 1)
        };
        match (&section.kind, element.kind) {
            (SectionKind::Spring(spring), ElementType::SPRING1 | ElementType::SPRING2) => {
                let stiffness = match &spring.stiffness {
                    SpringStiffness::Linear(rows) => rows.first().map_or(0.0, |r| r.value),
                    SpringStiffness::Nonlinear(_) => {
                        return Err(Error::Unsupported(format!(
                            "nonlinear *SPRING (element {}) is not implemented yet",
                            element.id
                        )));
                    }
                };
                let dofs = if element.kind == ElementType::SPRING1 {
                    vec![dof(nodes[0], spring.dof1)?]
                } else {
                    vec![dof(nodes[0], spring.dof1)?, dof(nodes[1], spring.dof2)?]
                };
                Ok(Self {
                    element: index,
                    dofs,
                    kind: DiscreteKind::Spring {
                        stiffness,
                        stabilize: spring.stabilize * stiffness,
                    },
                })
            }
            (SectionKind::Gap(gap), ElementType::GAPUNI) => {
                let d = gap.direction;
                let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                if len == 0.0 {
                    return Err(Error::input(
                        "*GAP",
                        format!("the direction of gap element {} is zero", element.id),
                    ));
                }
                Ok(Self {
                    element: index,
                    dofs: (0..6).map(|k| nodes[k / 3] * 3 + (k % 3) as u32).collect(),
                    kind: DiscreteKind::Gap {
                        clearance: gap.clearance,
                        direction: d.map(|c| c / len),
                        stiffness: gap.stiffness.unwrap_or(DEFAULT_GAP_STIFFNESS),
                        weights: vec![-1.0, 1.0],
                    },
                })
            }
            _ => Err(Error::input(
                "*ELEMENT",
                format!(
                    "element {} of type {} needs a matching *SPRING or *GAP",
                    element.id, element.kind
                ),
            )),
        }
    }

    /// The stiffness of a spring (row-major, `dofs.len()` squared), `None` for gaps.
    pub fn spring_stiffness(&self) -> Option<Vec<f64>> {
        match self.kind {
            DiscreteKind::Spring { stiffness: k, .. } => Some(if self.dofs.len() == 1 {
                vec![k]
            } else {
                vec![k, -k, -k, k]
            }),
            DiscreteKind::Gap { .. } => None,
        }
    }

    /// The stiffness `k (w wᵀ) ⊗ (n nᵀ)` of a closed gap (`GAPUNI`: `k n nᵀ [[1, -1], [-1,
    /// 1]]`), zero for springs.
    pub fn closed_stiffness(&self) -> Vec<f64> {
        let m = self.dofs.len();
        let mut out = vec![0.0; m * m];
        if let DiscreteKind::Gap {
            direction: n,
            stiffness: k,
            ref weights,
            ..
        } = self.kind
        {
            for a in 0..m {
                for b in 0..m {
                    out[a * m + b] = k * weights[a / 3] * weights[b / 3] * n[a % 3] * n[b % 3];
                }
            }
        }
        out
    }

    /// The load-side term of a closed gap with clearance `d`: its force
    /// `k (d + Σ w_i n·u_i)` has the constant part `k d`, moved to the right-hand side.
    pub fn add_clearance_force(&self, f: &mut [f64]) {
        if let DiscreteKind::Gap {
            clearance: d,
            direction: n,
            stiffness: k,
            ref weights,
        } = self.kind
        {
            for (a, &dof) in self.dofs.iter().enumerate() {
                f[dof as usize] -= k * d * weights[a / 3] * n[a % 3];
            }
        }
    }

    /// The opening `d + Σ w_i n·u_i` of a gap (`GAPUNI`: `d + n·(u2 − u1)`), `None` for
    /// springs.
    pub fn gap_opening(&self, u: &[f64]) -> Option<f64> {
        match &self.kind {
            DiscreteKind::Gap {
                clearance,
                direction: n,
                weights,
                ..
            } => Some(
                clearance
                    + (self.dofs.iter().enumerate())
                        .map(|(a, &dof)| weights[a / 3] * n[a % 3] * u[dof as usize])
                        .sum::<f64>(),
            ),
            DiscreteKind::Spring { .. } => None,
        }
    }
}

/// An integration point of a `*COMPRESSION ONLY` surface.
#[derive(Clone, Debug)]
pub struct SupportPoint {
    /// Index of the face in [`SupportPoints::faces`].
    pub face: usize,
    /// Local coordinates of the point on its face.
    pub local: [f64; 2],
    /// Integration weight of the point on its face (without the Jacobian).
    pub weight: f64,
    /// The area the point stands for.
    pub area: f64,
    /// Direction the ground pushes the surface in.
    pub direction: [f64; 3],
    /// Shape functions of the face nodes at the point.
    pub weights: Vec<f64>,
    /// Position of the point in the reference configuration.
    pub position: [f64; 3],
}

/// The faces and integration points of a `*COMPRESSION ONLY` surface with its stiffness
/// per area.
pub struct SupportPoints {
    pub faces: Vec<ContactFace>,
    pub points: Vec<SupportPoint>,
    pub stiffness: f64,
}

impl SupportPoints {
    /// The global DOFs of the nodes of a face.
    pub fn dofs(&self, face: usize) -> Vec<u32> {
        (self.faces[face].nodes.iter())
            .flat_map(|&n| [n * 3, n * 3 + 1, n * 3 + 2])
            .collect()
    }

    /// The opening of the gap at a point (negative: closed by that much).
    pub fn opening(&self, point: &SupportPoint, clearance: f64, u: &[f64]) -> f64 {
        let nodes = &self.faces[point.face].nodes;
        clearance
            + (nodes.iter().zip(&point.weights))
                .map(|(&n, &w)| {
                    (0..3)
                        .map(|k| {
                            w * point.direction[k]
                                * u.get(n as usize * 3 + k).copied().unwrap_or(0.0)
                        })
                        .sum::<f64>()
                })
                .sum::<f64>()
    }
}

/// The integration points of a `*COMPRESSION ONLY` surface: the Gauss rule of every face
/// (3×3 on quadratic quadrilaterals, as for a pressure), the normal the face's in the
/// reference configuration.
pub fn support_points(model: &Model, support: &CompressionOnly) -> Result<SupportPoints> {
    const KEYWORD: &str = "*COMPRESSION ONLY";
    let faces = ContactFace::surface_faces(model, KEYWORD, &support.surface)?;
    let coords = &model.nodes.coordinates;
    let stiffness = match support.stiffness {
        Some(k) => k,
        None => {
            let h = faces.iter().map(|f| f.edge_length(coords)).sum::<f64>()
                / faces.len().max(1) as f64;
            let young = faces
                .iter()
                .find_map(|f| element_young(model, f.element))
                .ok_or_else(|| {
                    Error::input(
                        KEYWORD,
                        format!(
                            "surface {} lies on elements without an elastic material: give \
                             STIFFNESS",
                            support.surface
                        ),
                    )
                })?;
            COMPRESSION_ONLY_STIFFNESS * young / h.max(1e-300)
        }
    };
    let normals = smoothed_normals(&faces, coords);
    let mut points = Vec::new();
    for (f, face) in faces.iter().enumerate() {
        let (locals, weights) = face_rule(face);
        for (p, w) in locals.iter().zip(weights) {
            let fp = face.at(coords, *p);
            let area = fp.jacobian * w;
            if area <= 0.0 {
                continue;
            }
            // The normal of the surface the faces stand for, interpolated from the smoothed
            // normals at the nodes.
            let mut normal = [0.0; 3];
            for (&n, nn) in fp.n.iter().zip(&normals[f]) {
                for k in 0..3 {
                    normal[k] += n * nn[k];
                }
            }
            let length = normal.iter().map(|c| c * c).sum::<f64>().sqrt();
            let normal = if length > 0.0 {
                normal.map(|c| c / length)
            } else {
                fp.normal
            };
            // The normal points out of the material; the ground pushes against it, or along
            // it when flipped.
            let direction = if support.flipped {
                normal
            } else {
                normal.map(|c| -c)
            };
            let mut position = [0.0; 3];
            for (&n, &w) in face.nodes.iter().zip(&fp.n) {
                for (k, x) in position.iter_mut().enumerate() {
                    *x += w * coords[n as usize][k];
                }
            }
            points.push(SupportPoint {
                face: f,
                local: *p,
                weight: w,
                area,
                direction,
                weights: fp.n,
                position,
            });
        }
    }
    Ok(SupportPoints {
        faces,
        points,
        stiffness,
    })
}

/// Faces whose normals differ by less than this angle (cosine) belong to one smooth
/// surface: a cylinder meshed with straight-sided faces, not the edge of a box.
const SMOOTH_SURFACE_COS: f64 = 0.766; // 40°

/// The normal at every node of every face (outward): the area weighted mean of the normals
/// of the faces at the node that lie on the same smooth surface as the face. A support on a
/// curved surface meshed with flat faces pushes along the surface's normal then, not along
/// the kinked normals of the faces: those would hold the nodes on a kink in two directions
/// and stripe the pressure along the face edges.
fn smoothed_normals(faces: &[ContactFace], coords: &[[f64; 3]]) -> Vec<Vec<[f64; 3]>> {
    // Normal and area of each face, at its centre.
    let centre_normals: Vec<([f64; 3], f64)> = faces
        .iter()
        .map(|face| {
            // The centre and the area of the reference face.
            let (centre, reference) = match face.corners {
                3 => ([1.0 / 3.0, 1.0 / 3.0], 0.5),
                4 => ([0.0, 0.0], 4.0),
                _ => ([0.0, 0.0], 2.0),
            };
            let fp = face.at(coords, centre);
            (fp.normal, fp.jacobian * reference)
        })
        .collect();
    let mut at_node: std::collections::HashMap<u32, Vec<usize>> = Default::default();
    for (f, face) in faces.iter().enumerate() {
        for &n in &face.nodes {
            at_node.entry(n).or_default().push(f);
        }
    }
    faces
        .iter()
        .enumerate()
        .map(|(f, face)| {
            let own = centre_normals[f].0;
            face.nodes
                .iter()
                .map(|n| {
                    let mut sum = [0.0; 3];
                    for &g in &at_node[n] {
                        let (normal, area) = centre_normals[g];
                        let cos: f64 = (0..3).map(|k| normal[k] * own[k]).sum();
                        if cos >= SMOOTH_SURFACE_COS {
                            for k in 0..3 {
                                sum[k] += area * normal[k];
                            }
                        }
                    }
                    let length = sum.iter().map(|c| c * c).sum::<f64>().sqrt();
                    if length > 0.0 {
                        sum.map(|c| c / length)
                    } else {
                        own
                    }
                })
                .collect()
        })
        .collect()
}

/// The gaps of a `*COMPRESSION ONLY` surface: one per integration point of every face (see
/// [`support_points`]), between the point of the face and fixed ground, closing when the
/// surface moves against the ground (into the material from outside, or out of it with
/// `FLIP`).
pub fn surface_gaps(model: &Model, support: &CompressionOnly) -> Result<Vec<DiscreteElement>> {
    let support_points = support_points(model, support)?;
    Ok((support_points.points.iter())
        .map(|p| DiscreteElement {
            element: support_points.faces[p.face].element,
            dofs: support_points.dofs(p.face),
            kind: DiscreteKind::Gap {
                clearance: support.clearance,
                direction: p.direction,
                stiffness: support_points.stiffness * p.area,
                weights: p.weights.clone(),
            },
        })
        .collect())
}

fn face_rule(face: &ContactFace) -> (Vec<[f64; 2]>, Vec<f64>) {
    if face.corners == 2 {
        let (p, w) = plxs_elements::shape::gauss_legendre(if face.quadratic { 3 } else { 2 });
        return (p.iter().map(|&x| [x, 0.0]).collect(), w);
    }
    plxs_elements::shape::face_rule(face.corners, face.quadratic)
}

/// Young's modulus of the material of an element (dense index).
fn element_young(model: &Model, element: u32) -> Option<f64> {
    let e = model.elements.get(element as usize)?;
    let section = model.sections.get(e.section? as usize)?;
    let SectionKind::Solid { material, .. } = &section.kind else {
        return None;
    };
    plxs_materials::LinearElastic::of(model.material(material)?, 0.0)
        .ok()
        .map(|e| e.young)
}
