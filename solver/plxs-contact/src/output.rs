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

/// The six components of the `CONTACT` field and `CELS` for every node (dense index): zero
/// at nodes without a contact element.
pub fn nodal_values(
    contact: &ContactModel,
    evaluation: &Evaluation,
    node_count: usize,
) -> (Vec<[f64; 6]>, Vec<f64>) {
    let mut sum = vec![[0.0; 6]; node_count];
    let mut energy = vec![0.0; node_count];
    let mut count = vec![0u32; node_count];
    for (pair, results) in contact.pairs.iter().zip(&evaluation.points) {
        if pair.slave_faces.is_empty() || pair.points.iter().all(|p| p.face.is_none()) {
            // Node-to-face: the node's own values.
            for (point, r) in pair.points.iter().zip(results) {
                let n = point.nodes[0] as usize;
                if r.element {
                    sum[n] = values_of(r);
                    energy[n] += r.energy;
                }
                count[n] = count[n].max(1);
            }
            continue;
        }
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
                let mut nodal = vec![[0.0; 6]; face.nodes.len()];
                for c in 0..6 {
                    let values: Vec<f64> = points
                        .iter()
                        .map(|r| if r.element { values_of(r)[c] } else { 0.0 })
                        .collect();
                    for (k, v) in face.extrapolate(&values).into_iter().enumerate() {
                        nodal[k][c] = v;
                    }
                }
                for (k, &node) in face.nodes.iter().enumerate() {
                    for c in 0..6 {
                        sum[node as usize][c] += nodal[k][c];
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
    (sum, energy)
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
    let elements = |f: &dyn Fn(&crate::Pair, &crate::PointResult, usize) -> Vec<f64>| {
        let mut rows = Vec::new();
        for &i in &pairs {
            let pair = &contact.pairs[i];
            for (p, r) in evaluation.points[i].iter().enumerate() {
                if !r.element {
                    continue;
                }
                let (element, face) = match pair.points[p].face {
                    Some((k, _)) => {
                        let f = &pair.slave_faces[k as usize];
                        (f.element_id, f64::from(f.face_number))
                    }
                    None => (pair.points[p].nodes[0], 0.0),
                };
                let mut values = vec![face];
                values.extend(f(pair, r, p));
                rows.push((element, values));
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
                "relative contact displacement (slave element+face,normal,tang1,tang2)",
                "",
                elements(&|_, r, _| vec![r.state.gap, r.tangential[0], r.tangential[1]]),
            )),
            "CSTR" => out.push(block(
                "contact stress (slave element+face,press,tang1,tang2)",
                "",
                elements(&|_, r, _| vec![r.state.pressure, r.shear[0], r.shear[1]]),
            )),
            "CELS" => out.push(block(
                "contact spring energy (slave element+face,energy)",
                "",
                elements(&|_, r, _| vec![r.energy]),
            )),
            "CF" => {
                for &i in &pairs {
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
