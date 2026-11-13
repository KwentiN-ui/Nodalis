//! Nodes that boundary conditions hold in different coordinate systems.
//!
//! A displacement in a coordinate system of its own and a symmetry plane that is no
//! coordinate plane give their nodes a local system (`*TRANSFORM`), in which the solver reads
//! every boundary condition at the node; a node has only one such system, the last one
//! defined. Where two boundary conditions in different systems meet, as at the edge between
//! two skewed cut faces, or between such a face and a face held in global directions, the
//! held directions of one of them would be read in the system of the other.
//!
//! Such a node gets a system of its own in which every held direction is an axis, or, where
//! held directions are not perpendicular, in which their span is held: the system of one of
//! the boundary conditions where it fits, else one built from the held directions. Each
//! boundary condition then holds the axes of that system at the node: the axis along its
//! direction with its value (negated for an opposite axis), or, for a direction held at zero
//! that is no axis, every axis it has a share of.

use std::collections::BTreeMap;

use plx_mesh::{FeMesh, NodeId};
use plx_model::{BoundaryCondition, BoundaryKind, FeModel, Formulation, Frame};

/// Directions of a node's system: x, y, z as unit vectors in global coordinates.
pub(super) type Axes = [[f64; 3]; 3];

const IDENTITY: Axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
/// Directions this close to each other (1 - |cos|) count as parallel.
const PARALLEL: f64 = 1e-6;
/// Shares of a direction below this are none.
const SHARE: f64 = 1e-9;

/// What a boundary condition holds: its directions and the held degrees of freedom (0..6)
/// with their values (`None` for a submodel's values read from the global model).
pub(super) struct Hold {
    system: Option<Frame>,
    fixed: Axes,
    pub held: Vec<(usize, Option<f64>)>,
    pub submodel: bool,
}

impl Hold {
    /// The directions of the boundary condition at a point.
    pub fn axes_at(&self, point: [f64; 3]) -> Result<Axes, String> {
        match &self.system {
            Some(frame) => frame.directions_at(point),
            None => Ok(self.fixed),
        }
    }

    /// The boundary condition's values at a node with the system `frame`: the value of
    /// each degree of freedom there and the held degree of freedom of the boundary
    /// condition it comes from.
    pub fn at_node(
        &self,
        point: [f64; 3],
        frame: &Axes,
    ) -> Result<[Option<(f64, usize)>; 6], String> {
        let axes = self.axes_at(point)?;
        let mut out = [None; 6];
        for &(dof, value) in &self.held {
            let offset = dof / 3 * 3;
            let direction = axes[dof % 3];
            let value = value.unwrap_or(0.0);
            let along = (0..3).find_map(|j| {
                let cos = dot(frame[j], direction);
                (1.0 - cos.abs() < PARALLEL).then_some((j, cos.signum()))
            });
            match along {
                Some((j, sign)) => out[offset + j] = Some((value * sign, dof)),
                // Held at zero (see `fits`): every axis with a share of the direction.
                None => {
                    for j in 0..3 {
                        if dot(frame[j], direction).abs() > SHARE {
                            out[offset + j] = Some((0.0, dof));
                        }
                    }
                }
            }
        }
        Ok(out)
    }
}

/// What a boundary condition holds at its nodes; `None` for one that holds no directions
/// at nodes of the mesh (temperatures, displacements of a point).
pub(super) fn hold(
    model: &FeModel,
    mesh: &FeMesh,
    bc: &BoundaryCondition,
    formulation: Option<Formulation>,
    dofs: usize,
) -> Result<Option<Hold>, String> {
    if bc.region.reference_point().is_some() {
        return Ok(None);
    }
    let space = model.properties.space;
    let hold = match &bc.kind {
        BoundaryKind::Displacement(displacement) => {
            if formulation != Some(Formulation::Nodes) {
                return Ok(None);
            }
            let system = match displacement.system {
                Some(_) => Some(
                    model
                        .coordinate_system_or_global(displacement.system)
                        .ok_or("The coordinate system does not exist")?,
                ),
                None => None,
            };
            let held = (0..dofs)
                .filter_map(|d| {
                    let value = displacement.values[d]?;
                    // A table scales its value over time: not zero.
                    let value = if displacement.table(d).is_some() {
                        1.0
                    } else {
                        value
                    };
                    Some((d, Some(value)))
                })
                .collect();
            Hold {
                system,
                fixed: IDENTITY,
                held,
                submodel: false,
            }
        }
        BoundaryKind::Submodel { dofs: held, .. } => Hold {
            system: None,
            fixed: IDENTITY,
            held: (0..dofs.min(3))
                .filter(|&d| held[d])
                .map(|d| (d, None))
                .collect(),
            submodel: true,
        },
        BoundaryKind::Symmetry { normal, .. } => {
            let plane = plx_model::symmetry_plane(mesh, &bc.region, space, *normal)
                .map_err(|e| e.to_string())?;
            let (axis, fixed) = match plane.axis() {
                Some(axis) => (axis, IDENTITY),
                None => {
                    let (t1, _) = plane.in_plane_axes();
                    (0, [plane.normal, t1, cross(plane.normal, t1)])
                }
            };
            Hold {
                system: None,
                fixed,
                held: (plx_model::symmetry_dofs(axis, dofs).into_iter())
                    .map(|d| (d, Some(0.0)))
                    .collect(),
                submodel: false,
            }
        }
        BoundaryKind::Temperature(_) => return Ok(None),
    };
    Ok((!hold.held.is_empty()).then_some(hold))
}

/// The nodes that boundary conditions of the active steps hold in different systems, with
/// the system each gets.
pub(super) fn node_frames(
    model: &FeModel,
    mesh: &FeMesh,
    skipped: impl Fn(NodeId) -> bool,
) -> Result<BTreeMap<NodeId, Axes>, super::WriteError> {
    let dofs = if model.properties.space.is_2d() { 2 } else { 6 };
    // Per node the held directions: their system there, the degrees of freedom with their
    // values, whether a submodel holds them, and the boundary condition's name.
    type Entry<'a> = (Axes, Vec<(usize, Option<f64>)>, bool, &'a str);
    let mut at: BTreeMap<NodeId, Vec<Entry>> = BTreeMap::new();
    let positions = mesh.node_ids().iter().zip(mesh.coords());
    let positions: BTreeMap<NodeId, [f64; 3]> = positions.map(|(&id, &c)| (id, c)).collect();
    for (s, step) in (model.steps.iter().enumerate()).filter(|(_, s)| s.active) {
        for (i, bc) in step.boundary_conditions.iter().enumerate() {
            if !bc.active || !step.kind.supports_boundary(&bc.kind) {
                continue;
            }
            let formulation = model.formulation(mesh, s, i);
            let Ok(Some(hold)) = hold(model, mesh, bc, formulation, dofs) else {
                // The boundary condition reports its own errors when it is written.
                continue;
            };
            for node in bc.region.nodes(mesh) {
                let Some(&point) = positions.get(&node) else {
                    continue;
                };
                if skipped(node) {
                    continue;
                }
                let Ok(axes) = hold.axes_at(point) else {
                    continue;
                };
                (at.entry(node).or_default()).push((
                    axes,
                    hold.held.clone(),
                    hold.submodel,
                    &bc.name,
                ));
            }
        }
    }
    let mut frames = BTreeMap::new();
    for (node, entries) in at {
        let first = entries[0].0;
        if entries.iter().all(|(axes, ..)| same(axes, &first)) {
            continue;
        }
        let blocks: [Vec<([f64; 3], bool)>; 2] = std::array::from_fn(|b| {
            (entries.iter())
                .flat_map(|(axes, held, ..)| {
                    (held.iter())
                        .filter(|(d, _)| d / 3 == b)
                        .map(|&(d, v)| (axes[d % 3], v == Some(0.0)))
                })
                .collect()
        });
        let submodel = entries.iter().any(|(_, _, submodel, _)| *submodel);
        let mut candidates: Vec<Axes> = Vec::new();
        for (axes, ..) in &entries {
            if !candidates.iter().any(|c| same(c, axes)) {
                candidates.push(*axes);
            }
        }
        candidates.extend(spanned(&blocks[0]));
        let fitting = (candidates.into_iter()).find(|frame| {
            (!submodel || same(frame, &IDENTITY)) && blocks.iter().all(|b| fits(frame, b))
        });
        let Some(frame) = fitting else {
            let mut names: Vec<&str> = entries.iter().map(|(.., name)| *name).collect();
            names.dedup();
            return Err(super::WriteError::InvalidBoundary {
                item: names[0].to_owned(),
                reason: format!(
                    "Node {node} is held by {} in different coordinate systems, in directions \
                     that are not perpendicular to each other with values other than zero{}. \
                     The solver holds a node in one coordinate system only.",
                    names.join(", "),
                    if submodel {
                        ", or with a submodel boundary condition"
                    } else {
                        ""
                    }
                ),
            });
        };
        frames.insert(node, frame);
    }
    Ok(frames)
}

/// Whether the system can hold the directions of a block (translations or rotations): its
/// axes span what they span, and every direction with a value other than zero is one of the
/// axes.
fn fits(frame: &Axes, directions: &[([f64; 3], bool)]) -> bool {
    let basis = orthonormal(directions.iter().map(|(d, _)| *d));
    let spanning = (frame.iter())
        .filter(|axis| {
            let inside: f64 = basis.iter().map(|b| dot(**axis, *b).powi(2)).sum();
            inside > 1.0 - PARALLEL
        })
        .count();
    spanning == basis.len()
        && (directions.iter()).all(|(direction, zero)| {
            *zero
                || frame
                    .iter()
                    .any(|axis| 1.0 - dot(*axis, *direction).abs() < PARALLEL)
        })
}

/// A system whose first axes span the directions, starting with those of values other
/// than zero.
fn spanned(directions: &[([f64; 3], bool)]) -> Option<Axes> {
    let ordered = (directions.iter().filter(|(_, zero)| !zero))
        .chain(directions.iter().filter(|(_, zero)| *zero))
        .map(|(d, _)| *d);
    let mut basis = orthonormal(ordered);
    for axis in IDENTITY {
        if basis.len() == 3 {
            break;
        }
        basis.extend(
            orthonormal(basis.iter().copied().chain([axis]))
                .get(basis.len())
                .copied(),
        );
    }
    (basis.len() == 3).then(|| [basis[0], basis[1], cross(basis[0], basis[1])])
}

/// An orthonormal basis of the span of the directions, by Gram-Schmidt in their order.
fn orthonormal(directions: impl IntoIterator<Item = [f64; 3]>) -> Vec<[f64; 3]> {
    let mut basis: Vec<[f64; 3]> = Vec::new();
    for direction in directions {
        let mut rest = direction;
        for b in &basis {
            let share = dot(rest, *b);
            rest = std::array::from_fn(|k| rest[k] - share * b[k]);
        }
        let length = dot(rest, rest).sqrt();
        if length > 1e-6 * dot(direction, direction).sqrt() && length > 0.0 {
            basis.push(rest.map(|c| c / length));
        }
    }
    basis
}

fn same(a: &Axes, b: &Axes) -> bool {
    (0..3).all(|k| 1.0 - dot(a[k], b[k]) < PARALLEL)
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_system_fits_where_it_has_the_held_directions_as_axes() {
        let turned: Axes = [[0.0, 0.0, 1.0], [0.6, 0.8, 0.0], [0.8, -0.6, 0.0]];
        // Global z held (zero) and the turned z: the turned system has both as axes.
        let held = [([0.0, 0.0, 1.0], true), ([0.8, -0.6, 0.0], true)];
        assert!(fits(&turned, &held));
        assert!(!fits(&IDENTITY, &held));
        // All three held: any system spans them.
        let all = [
            ([1.0, 0.0, 0.0], true),
            ([0.0, 1.0, 0.0], true),
            ([0.0, 0.0, 1.0], true),
        ];
        assert!(fits(&turned, &all));
        // A value other than zero needs its direction as an axis.
        let moved = [([0.0, 0.0, 1.0], true), ([1.0, 0.0, 0.0], false)];
        assert!(!fits(&turned, &moved));
        assert!(fits(&IDENTITY, &moved));
    }

    #[test]
    fn directions_at_an_angle_span_a_system_of_their_own() {
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let held = [([1.0, 0.0, 0.0], true), ([s, s, 0.0], true)];
        let frame = spanned(&held).unwrap();
        assert!(fits(&frame, &held));
        assert!((dot(frame[2], [0.0, 0.0, 1.0]).abs() - 1.0).abs() < 1e-12);
    }
}
