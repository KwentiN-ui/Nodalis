//! Bolt pre-tension written as a cut through the mesh with `*Equation`s, instead of CalculiX's
//! `*PRE-TENSION SECTION`. CalculiX separates only the elements that have a face in the
//! section; a tetrahedron that touches the cut with just an edge or a corner keeps both sides
//! together, and most of the preload goes into it (PrePoMax users add a thin layer of wedge
//! elements at the cut for this). Here every element on the side of the selected faces gets
//! copies of the cut nodes, and the equations of CalculiX's section tie the copies to the
//! originals: the gap normal to the cut is the first degree of freedom of the pre-tension
//! node, the tangential motion is shared.

use std::collections::{BTreeMap, BTreeSet};

use plx_mesh::{ElementFamily, FeMesh, NodeId};
use plx_model::{FeModel, LoadKind};

use super::constraints::{face_node_ids, outward_normal};
use super::{WriteError, empty};

/// One cut: the original nodes with their copies, and the direction of the preload.
pub(super) struct Cut {
    pub pairs: Vec<(NodeId, NodeId)>,
    pub normal: [f64; 3],
}

/// The cut mesh with the cuts by load name.
type Split = (FeMesh, BTreeMap<String, Cut>);

/// The mesh cut at the faces of every active pre-tension load, with the cuts by load name;
/// `None` without pre-tension loads.
pub(super) fn split(mesh: &FeMesh, model: &FeModel) -> Result<Option<Split>, WriteError> {
    let loads = (model.steps.iter())
        .filter(|s| s.active && s.kind.supports_loads())
        .flat_map(|s| s.loads.iter().filter(|l| l.active));
    let mut split = None::<Split>;
    for load in loads {
        let LoadKind::PreTension { direction, .. } = load.kind else {
            continue;
        };
        let (mesh, cuts) = split.get_or_insert_with(|| (mesh.clone(), BTreeMap::new()));
        if cuts.contains_key(&load.name) {
            continue;
        }
        let faces = load.region.faces(mesh);
        // Away from the elements of the selected faces, as CalculiX takes it.
        let mut away = [0.0; 3];
        for &(element, face) in &faces {
            if let Some(n) = outward_normal(mesh, element, face) {
                (0..3).for_each(|k| away[k] += n[k]);
            }
        }
        let Some(away) = unit(away) else {
            return Err(empty(&load.name, "element faces"));
        };
        let normal = direction.and_then(unit).unwrap_or(away);
        let cut_nodes: BTreeSet<NodeId> = (faces.iter())
            .flat_map(|&(element, face)| face_node_ids(mesh, element, face))
            .collect();
        let face_elements: BTreeSet<_> = faces.iter().map(|&(element, _)| element).collect();
        // The elements on the side of the faces: theirs, and every other solid element at the
        // cut whose centre lies behind the cut nodes it shares.
        let side: BTreeSet<_> = (mesh.elements().iter())
            .filter(|e| e.shape.family() == ElementFamily::Solid)
            .filter(|e| {
                if face_elements.contains(&e.id) {
                    return true;
                }
                let shared: Vec<[f64; 3]> = (e.nodes.iter())
                    .filter(|n| cut_nodes.contains(n))
                    .filter_map(|&n| mesh.node(n))
                    .collect();
                let all: Vec<[f64; 3]> = e.nodes.iter().filter_map(|&n| mesh.node(n)).collect();
                !shared.is_empty() && dot(sub(centre(&all), centre(&shared)), away) < 0.0
            })
            .map(|e| e.id)
            .collect();
        let first = (mesh.node_ids().iter()).max().map_or(1, |&n| n + 1);
        let copies: BTreeMap<NodeId, NodeId> = (cut_nodes.iter().copied()).zip(first..).collect();
        mesh.split_nodes(&side, &copies);
        cuts.insert(
            load.name.clone(),
            Cut {
                pairs: copies.into_iter().collect(),
                normal,
            },
        );
    }
    Ok(split)
}

/// The `*Equation`s of a cut, as CalculiX generates them for a pre-tension section: per node
/// pair, the gap along the normal equals the first degree of freedom of the pre-tension node
/// (positive closes the gap, which tensions the bolt), and the motion along two tangents is
/// the same on both sides.
pub(super) fn equations(cut: &Cut, node: NodeId) -> Vec<Vec<(NodeId, u8, f64)>> {
    let n = cut.normal;
    // A tangent from the global x axis, or y when the normal is close to x.
    let axis = if n[0].abs() < 0.95 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let t = unit(sub(axis, n.map(|v| v * dot(axis, n)))).unwrap_or([0.0, 1.0, 0.0]);
    let d = cross(n, t);
    // Each equation needs its own dependent degree of freedom of the copy with a factor
    // that is not zero: a permutation of x, y, z for n, t, d.
    const PERMUTATIONS: [[usize; 3]; 6] = [
        [0, 1, 2],
        [1, 2, 0],
        [2, 0, 1],
        [0, 2, 1],
        [1, 0, 2],
        [2, 1, 0],
    ];
    let order = (PERMUTATIONS.iter())
        .find(|p| n[p[0]].abs() > 1e-3 && t[p[1]].abs() > 1e-3 && d[p[2]].abs() > 1e-3)
        .copied()
        .unwrap_or(PERMUTATIONS[0]);
    let mut out = Vec::new();
    for &(original, copy) in &cut.pairs {
        for (vector, first) in [n, t, d].into_iter().zip(order) {
            let dofs = |skip: Option<usize>| {
                (0..3).filter(move |&k| Some(k) != skip && vector[k].abs() > 1e-10)
            };
            let mut terms = vec![(copy, first as u8 + 1, vector[first])];
            terms.extend(dofs(Some(first)).map(|k| (copy, k as u8 + 1, vector[k])));
            terms.extend(dofs(None).map(|k| (original, k as u8 + 1, -vector[k])));
            if vector == n {
                terms.push((node, 1, -1.0));
            }
            out.push(terms);
        }
    }
    out
}

fn unit(v: [f64; 3]) -> Option<[f64; 3]> {
    let length = dot(v, v).sqrt();
    (length > 1e-12).then(|| v.map(|x| x / length))
}

fn centre(points: &[[f64; 3]]) -> [f64; 3] {
    std::array::from_fn(|k| points.iter().map(|p| p[k]).sum::<f64>() / points.len() as f64)
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
