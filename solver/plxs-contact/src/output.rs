//! Contact results: the nodal fields of `*CONTACT FILE` (`CDIS` = COPEN, CSLIP1, CSLIP2;
//! `CSTR` = CPRESS, CSHEAR1, CSHEAR2; `CELS`) and the blocks of `*CONTACT PRINT` (`CDIS`,
//! `CSTR`, `CELS` per contact element, `CNUM`, `CF` statistics), in the layout CalculiX
//! writes (observed on ccx 2.21 output, read by `plx-io`). Face-to-face values live at the
//! integration points of the slave faces and are extrapolated to the face nodes by a least
//! squares fit, averaged where faces share a node; node-to-face values are the nodes' own.
//! No CalculiX code was consulted.

use plxs_core::results::HistoryBlock;
use plxs_core::step::HistoryOutput;

use crate::geometry::{cross, dot, norm, normalized, scale};
use crate::{ContactModel, Evaluation};

/// The nodal contact results: the six components of the `CONTACT` field and `CELS` for
/// every node (dense index, zero at nodes without a contact element) and which nodes the
/// fields list. CalculiX lists every node when a face-to-face pair exists, else just the
/// slave nodes with a contact spring (node-to-face) and all slave nodes (mortar).
pub struct NodalValues {
    pub values: Vec<[f64; 6]>,
    pub energy: Vec<f64>,
    /// `None`: all nodes.
    pub listed: Option<Vec<bool>>,
}

pub fn nodal_values(
    contact: &ContactModel,
    evaluation: &Evaluation,
    node_count: usize,
) -> NodalValues {
    let mut sum = vec![[0.0; 6]; node_count];
    let mut energy = vec![0.0; node_count];
    let mut count = vec![0u32; node_count];
    let mut listed = vec![false; node_count];
    let mut faces = false;
    for (pair, results) in contact.pairs.iter().zip(&evaluation.points) {
        if pair.slave_faces.is_empty() || pair.points.iter().all(|p| p.face.is_none()) {
            // Node-to-face: the node's own values.
            for (point, r) in pair.points.iter().zip(results) {
                let n = point.nodes[0] as usize;
                if r.element {
                    sum[n] = values_of(r);
                    energy[n] += r.energy;
                    listed[n] = true;
                }
                count[n] = count[n].max(1);
            }
            continue;
        }
        if !pair.mortar_nodes.is_empty() {
            // Mortar: the nodes' own (weighted) values, every slave node listed.
            for ((node, _), r) in pair.mortar_nodes.iter().zip(results) {
                let n = *node as usize;
                if r.element {
                    sum[n] = values_of(r);
                    energy[n] += r.energy;
                }
                listed[n] = true;
                count[n] = count[n].max(1);
            }
            continue;
        }
        faces = true;
        // Face-to-face: per slave face, extrapolate the integration point values.
        let mut start = 0;
        for (f, face) in pair.slave_faces.iter().enumerate() {
            let end = start
                + pair.points[start..]
                    .iter()
                    .take_while(|p| p.face.is_some_and(|(k, _)| k as usize == f))
                    .count();
            let points = &results[start..end];
            if points.iter().any(|r| r.element) {
                let columns: Vec<Vec<f64>> = (0..6)
                    .map(|c| {
                        let values: Vec<f64> = points
                            .iter()
                            .map(|r| if r.element { values_of(r)[c] } else { 0.0 })
                            .collect();
                        face.extrapolate(&values)
                    })
                    .collect();
                let nodal: Vec<[f64; 6]> = (0..face.nodes.len())
                    .map(|k| std::array::from_fn(|c| columns[c][k]))
                    .collect();
                for (values, &node) in nodal.iter().zip(&face.nodes) {
                    for (s, v) in sum[node as usize].iter_mut().zip(values) {
                        *s += v;
                    }
                    count[node as usize] += 1;
                }
                for (point, r) in pair.points[start..end].iter().zip(points) {
                    for (&node, &w) in point.nodes.iter().zip(&point.weights) {
                        energy[node as usize] += w * r.energy;
                    }
                }
            } else {
                for &node in &face.nodes {
                    count[node as usize] += 1;
                }
            }
            start = end;
        }
    }
    for (s, &c) in sum.iter_mut().zip(&count) {
        if c > 1 {
            for v in s.iter_mut() {
                *v /= f64::from(c);
            }
        }
    }
    NodalValues {
        values: sum,
        energy,
        listed: (!faces).then_some(listed),
    }
}

fn values_of(r: &crate::PointResult) -> [f64; 6] {
    [
        r.state.gap,
        r.tangential[0],
        r.tangential[1],
        r.state.pressure,
        r.shear[0],
        r.shear[1],
    ]
}

/// The `.dat` blocks of a `*CONTACT PRINT` request for an increment.
pub fn history_blocks(
    contact: &ContactModel,
    evaluation: &Evaluation,
    request: &HistoryOutput,
    node_ids: &[u32],
    step: u32,
    increment: u32,
    time: f64,
) -> Vec<HistoryBlock> {
    let mut out = Vec::new();
    let block = |quantity: &str, set: &str, rows: Vec<(u32, Vec<f64>)>| HistoryBlock {
        step,
        increment,
        quantity: quantity.to_string(),
        set: set.to_string(),
        time,
        rows,
    };
    // Which pairs the request addresses: those of its surfaces, else all.
    let pairs: Vec<usize> = (0..contact.pairs.len())
        .filter(|&i| {
            request.surfaces.as_ref().is_none_or(|(slave, master)| {
                contact.pairs[i].slave_name == *slave && contact.pairs[i].master_name == *master
            })
        })
        .collect();
    // Face-to-face points are listed by slave element and face, node-to-face ones by node.
    let by_face = |i: usize| {
        let pair = &contact.pairs[i];
        pair.mortar_nodes.is_empty() && pair.points.iter().any(|p| p.face.is_some())
    };
    let slave = if pairs.iter().any(|&i| by_face(i)) {
        "slave element+face"
    } else {
        "slave node"
    };
    let elements = |f: &dyn Fn(&crate::Pair, &crate::PointResult, usize) -> Vec<f64>| {
        let mut rows = Vec::new();
        for &i in &pairs {
            let pair = &contact.pairs[i];
            for (p, r) in evaluation.points[i].iter().enumerate() {
                if !r.element {
                    continue;
                }
                let mut values = Vec::with_capacity(4);
                if let Some((node, _)) = pair.mortar_nodes.get(p) {
                    values.extend(f(pair, r, p));
                    rows.push((node_ids[*node as usize], values));
                    continue;
                }
                let id = match pair.points[p].face {
                    Some((k, _)) => {
                        let f = &pair.slave_faces[k as usize];
                        values.push(f64::from(f.face_number));
                        f.element_id
                    }
                    None => node_ids[pair.points[p].nodes[0] as usize],
                };
                values.extend(f(pair, r, p));
                rows.push((id, values));
            }
        }
        rows
    };
    for variable in &request.variables {
        match variable.trim().to_ascii_uppercase().as_str() {
            "CNUM" => {
                let n = pairs
                    .iter()
                    .map(|&i| evaluation.points[i].iter().filter(|r| r.element).count())
                    .sum::<usize>();
                out.push(block(
                    "total number of contact elements",
                    "",
                    vec![(0, vec![n as f64])],
                ));
            }
            "CDIS" => out.push(block(
                &format!("relative contact displacement ({slave},normal,tang1,tang2)"),
                "",
                elements(&|_, r, _| vec![r.state.gap, r.tangential[0], r.tangential[1]]),
            )),
            "CSTR" => out.push(block(
                &format!("contact stress ({slave},press,tang1,tang2)"),
                "",
                elements(&|_, r, _| vec![r.state.pressure, r.shear[0], r.shear[1]]),
            )),
            "CELS" => out.push(block(
                &format!("contact spring energy ({slave},energy)"),
                "",
                elements(&|_, r, _| vec![r.energy]),
            )),
            "CF" => {
                // CalculiX prints the statistics for face-to-face pairs only.
                for &i in pairs
                    .iter()
                    .filter(|&&i| contact.pairs[i].points.iter().any(|p| p.face.is_some()))
                {
                    let pair = &contact.pairs[i];
                    let results = &evaluation.points[i];
                    let mut force = [0.0; 3];
                    let mut moment = [0.0; 3];
                    let mut area = 0.0;
                    let mut centre = [0.0; 3];
                    let mut normal = [0.0; 3];
                    for r in results.iter().filter(|r| r.element) {
                        for d in 0..3 {
                            force[d] += r.force[d];
                            centre[d] += r.area * r.position[d];
                            normal[d] -= r.area * r.normal[d];
                        }
                        let m = cross(r.position, r.force);
                        for d in 0..3 {
                            moment[d] += m[d];
                        }
                        area += r.area;
                    }
                    if area > 0.0 {
                        centre = scale(centre, 1.0 / area);
                    }
                    let normal = normalized(normal);
                    let about_cg = cross(force, centre);
                    let moment_cg = [
                        moment[0] + about_cg[0],
                        moment[1] + about_cg[1],
                        moment[2] + about_cg[2],
                    ];
                    let normal_force = dot(force, normal);
                    let shear = norm([
                        force[0] - normal_force * normal[0],
                        force[1] - normal_force * normal[1],
                        force[2] - normal_force * normal[2],
                    ]);
                    out.push(block(
                        &format!(
                            "statistics for slave set {}, master set {}",
                            pair.slave_name, pair.master_name
                        ),
                        "",
                        vec![
                            (
                                0,
                                vec![
                                    force[0], force[1], force[2], moment[0], moment[1], moment[2],
                                ],
                            ),
                            (
                                0,
                                vec![
                                    centre[0], centre[1], centre[2], normal[0], normal[1],
                                    normal[2],
                                ],
                            ),
                            (0, moment_cg.to_vec()),
                            (0, vec![area, normal_force, shear]),
                        ],
                    ));
                }
            }
            _ => {}
        }
    }
    out
}
