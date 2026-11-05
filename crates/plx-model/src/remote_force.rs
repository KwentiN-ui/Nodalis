//! Force and moment on a point a selection follows, Ansys' remote force. The point, by
//! default at the centre of the selection, exists only in the solver's input: the selection
//! follows it as a rigid body or on average (deformable), as the point of a remote
//! displacement does, and the force and moment act on it. A moment thus reaches faces of
//! solids, whose nodes have no rotations.

use std::collections::BTreeSet;

use plx_mesh::{FeMesh, NodeId};
use serde::{Deserialize, Serialize};

use crate::features::GLOBAL;
use crate::{Behavior, Displacement, FeModel, LoadKind, ModelSpace, Region, RemoteBehavior};

/// Force and moment at a point the selection of the load follows.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RemoteForce {
    /// Force (F1..F3) in the directions of `system`.
    pub force: [f64; 3],
    /// Moment (M1..M3) about the directions of `system`, at the point.
    pub moment: [f64; 3],
    /// How the selection follows the point.
    pub behavior: RemoteBehavior,
    /// Coordinate system of the location and of the directions, by name ([`GLOBAL`]).
    pub system: String,
    /// Coordinates of the point in `system`; `None` is the centre of the selection.
    pub location: Option<[f64; 3]>,
    /// Only the nodes within this distance of the point take part, Ansys' pinball region.
    pub pinball: Option<f64>,
}

impl Default for RemoteForce {
    fn default() -> Self {
        Self {
            force: [0.0; 3],
            moment: [0.0; 3],
            behavior: RemoteBehavior::Rigid,
            system: GLOBAL.into(),
            location: None,
            pinball: None,
        }
    }
}

impl RemoteForce {
    /// The point as that of a remote displacement holding nothing, which resolves it and
    /// shares it with a remote displacement of the same region and point.
    pub fn point(&self) -> Displacement {
        Displacement {
            values: [None; 6],
            behavior: match self.behavior {
                RemoteBehavior::Rigid => Behavior::Rigid,
                RemoteBehavior::Deformable => Behavior::Deformable,
            },
            system: self.system.clone(),
            location: self.location,
            pinball: self.pinball,
        }
    }

    /// Takes the location, pinball region and coordinate system of `point` back.
    pub fn set_point(&mut self, point: &Displacement) {
        self.system = point.system.clone();
        self.location = point.location;
        self.pinball = point.pinball;
    }

    /// Force and moment in global directions at `position`, the point, with the
    /// components the model space has: 2D models move in x and y and turn about z, an
    /// axisymmetric section does not turn.
    pub fn global(
        &self,
        model: &FeModel,
        position: [f64; 3],
    ) -> Result<([f64; 3], [f64; 3]), String> {
        let system = (model.coordinate_system_or_global(&self.system))
            .ok_or_else(|| format!("Coordinate System {} does not exist", self.system))?;
        let axes = system.directions_at(position)?;
        // The global component k is sum_j axes[j][k] times the local component j.
        let rotate = |local: [f64; 3]| -> [f64; 3] {
            std::array::from_fn(|k| (0..3).map(|j| axes[j][k] * local[j]).sum())
        };
        let (mut force, mut moment) = (rotate(self.force), rotate(self.moment));
        let dofs = model.properties.space.remote_dofs();
        for k in 0..3 {
            if !dofs.contains(&k) {
                force[k] = 0.0;
            }
            if !dofs.contains(&(k + 3)) {
                moment[k] = 0.0;
            }
        }
        // Rounding of the rotation leaves no tiny components.
        let clean = |v: [f64; 3]| {
            let largest = v.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
            v.map(|x| if x.abs() <= 1e-12 * largest { 0.0 } else { x })
        };
        Ok((clean(force), clean(moment)))
    }
}

impl FeModel {
    /// The remote forces of the active steps, as step, load index, region and the load,
    /// those the step takes.
    pub fn remote_forces(&self) -> impl Iterator<Item = (usize, usize, &Region, &RemoteForce)> {
        (self.steps.iter().enumerate())
            .filter(|(_, step)| step.active && step.kind.supports_loads())
            .flat_map(|(s, step)| {
                (step.loads.iter().enumerate())
                    .filter(move |(_, load)| load.active && step.kind.supports_load(&load.kind))
                    .filter_map(move |(i, load)| match &load.kind {
                        LoadKind::RemoteForce(remote) => Some((s, i, &load.region, remote)),
                        _ => None,
                    })
            })
    }

    /// The active steps in which the point of a remote force with `region` acts: those with
    /// an active remote force on the same region and point.
    pub fn remote_force_steps(&self, region: &Region, remote: &RemoteForce) -> BTreeSet<usize> {
        let point = remote.point();
        (self.remote_forces())
            .filter(|(_, _, r, other)| *r == region && other.point().same_point(&point))
            .map(|(s, ..)| s)
            .collect()
    }

    /// The nodes of the selection that follow the point of a remote force, without those
    /// boundary conditions hold directly in the steps the point acts in.
    pub fn remote_force_nodes(
        &self,
        mesh: &FeMesh,
        region: &Region,
        remote: &RemoteForce,
    ) -> Result<(crate::RemotePoint, BTreeSet<NodeId>), String> {
        let steps = self.remote_force_steps(region, remote);
        let excluded = self.directly_held_nodes_in(mesh, |s| steps.contains(&s));
        let point = remote.point().resolve(self, mesh, region, &excluded)?;
        Ok((point, excluded))
    }
}

impl ModelSpace {
    /// Whether a remote force can have a moment in the space.
    pub fn has_remote_moment(self) -> bool {
        self.remote_dofs().iter().any(|&d| d >= 3)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CoordinateSystem, CoordinateSystemKind};

    #[test]
    fn global_components_follow_the_system() {
        let mut model = FeModel::default();
        let remote = RemoteForce {
            force: [10.0, 0.0, 0.0],
            moment: [0.0, 5.0, 0.0],
            ..Default::default()
        };
        assert_eq!(
            remote.global(&model, [0.0; 3]),
            Ok(([10.0, 0.0, 0.0], [0.0, 5.0, 0.0]))
        );
        // A system turned 90 degrees about z: local x is global y.
        model.coordinate_systems.push(CoordinateSystem {
            name: "Turned".into(),
            kind: CoordinateSystemKind::Rectangular,
            origin: [0.0; 3],
            point_x: [0.0, 1.0, 0.0],
            point_xy: [-1.0, 0.0, 0.0],
        });
        let turned = RemoteForce {
            system: "Turned".into(),
            ..remote
        };
        let (force, moment) = turned.global(&model, [0.0; 3]).unwrap();
        assert!((force[1] - 10.0).abs() < 1e-9 && force[0].abs() < 1e-9);
        assert!((moment[0] + 5.0).abs() < 1e-9 && moment[1].abs() < 1e-9);
        let missing = RemoteForce {
            system: "Missing".into(),
            ..Default::default()
        };
        assert!(missing.global(&model, [0.0; 3]).is_err());
    }

    #[test]
    fn plane_models_keep_the_in_plane_components() {
        let mut model = FeModel::default();
        model.properties.space = ModelSpace::PlaneStress;
        let remote = RemoteForce {
            force: [1.0, 2.0, 3.0],
            moment: [4.0, 5.0, 6.0],
            ..Default::default()
        };
        assert_eq!(
            remote.global(&model, [0.0; 3]),
            Ok(([1.0, 2.0, 0.0], [0.0, 0.0, 6.0]))
        );
    }
}
