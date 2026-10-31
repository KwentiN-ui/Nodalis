//! Discrete elements: `SPRING1` (one DOF of one node to ground), `SPRING2` (between one DOF
//! of each of two nodes) with linear stiffness (`*SPRING`), and `GAPUNI` (`*GAP`), a
//! compression-only link between two nodes along a direction.
//!
//! CalculiX manual, `*SPRING` and `*GAP` and the element descriptions "SPRINGA, SPRING1,
//! SPRING2" and "GAPUNI": a spring adds `k` to its DOF (`SPRING1`) or `k [[1, -1], [-1, 1]]`
//! to its two DOFs (`SPRING2`); a gap with direction `n` and clearance `d` is closed when
//! `d + n·(u2 − u1) ≤ 0` and then acts as a penalty spring `k n nᵀ` between the nodes.
//!
//! No CalculiX code was consulted.

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
    },
    Gap {
        clearance: f64,
        /// Unit direction from the first to the second node.
        direction: [f64; 3],
        stiffness: f64,
    },
}

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
                    kind: DiscreteKind::Spring { stiffness },
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
            DiscreteKind::Spring { stiffness: k } => Some(if self.dofs.len() == 1 {
                vec![k]
            } else {
                vec![k, -k, -k, k]
            }),
            DiscreteKind::Gap { .. } => None,
        }
    }

    /// The stiffness `k n nᵀ [[1, -1], [-1, 1]]` of a closed gap (6 x 6), zero for springs.
    pub fn closed_stiffness(&self) -> Vec<f64> {
        let mut out = vec![0.0; self.dofs.len() * self.dofs.len()];
        if let DiscreteKind::Gap {
            direction: n,
            stiffness: k,
            ..
        } = self.kind
        {
            for a in 0..6 {
                for b in 0..6 {
                    let sign = if (a < 3) == (b < 3) { 1.0 } else { -1.0 };
                    out[a * 6 + b] = sign * k * n[a % 3] * n[b % 3];
                }
            }
        }
        out
    }

    /// The load-side term of a closed gap with clearance `d`: its force
    /// `k (d + n·(u2 − u1))` has the constant part `k d`, moved to the right-hand side.
    pub fn add_clearance_force(&self, f: &mut [f64]) {
        if let DiscreteKind::Gap {
            clearance: d,
            direction: n,
            stiffness: k,
        } = self.kind
        {
            for c in 0..3 {
                f[self.dofs[c] as usize] += k * d * n[c];
                f[self.dofs[3 + c] as usize] -= k * d * n[c];
            }
        }
    }

    /// The opening `d + n·(u2 − u1)` of a gap, `None` for springs.
    pub fn gap_opening(&self, u: &[f64]) -> Option<f64> {
        match self.kind {
            DiscreteKind::Gap {
                clearance,
                direction: n,
                ..
            } => {
                let du: f64 = (0..3)
                    .map(|d| n[d] * (u[self.dofs[3 + d] as usize] - u[self.dofs[d] as usize]))
                    .sum();
                Some(clearance + du)
            }
            DiscreteKind::Spring { .. } => None,
        }
    }
}
