//! Results of `*COMPRESSION ONLY` surfaces: their opening and pressure at the nodes (in the
//! `CONTACT` field, as COPEN and CPRESS) and the force the ground exerts on each surface
//! (`*CONTACT PRINT` `CF`, the statistics of a contact pair with the surface as slave).

use plxs_assembly::discrete::{SupportPoints, support_points};
use plxs_core::Model;
use plxs_core::contact::CompressionOnly;
use plxs_core::results::HistoryBlock;
use plxs_core::step::HistoryOutput;

/// Opening and pressure of the gap at every point of a surface.
fn point_values(support: &CompressionOnly, points: &SupportPoints, u: &[f64]) -> Vec<(f64, f64)> {
    (points.points.iter())
        .map(|p| {
            let opening = points.opening(p, support.clearance, u);
            (opening, points.stiffness * (-opening).max(0.0))
        })
        .collect()
}

/// COPEN and CPRESS at the nodes of all compression only surfaces (dense node index), from
/// the values at the integration points by the least squares fit of the contact results,
/// averaged where faces share a node; `None` without such surfaces.
pub fn nodal_values(model: &Model, u: &[f64]) -> Option<Vec<Option<[f64; 2]>>> {
    if model.contact.compression_only.is_empty() {
        return None;
    }
    let n = model.nodes.len();
    let mut sum = vec![[0.0; 2]; n];
    let mut count = vec![0u32; n];
    for support in &model.contact.compression_only {
        let Ok(points) = support_points(model, support) else {
            continue;
        };
        let values = point_values(support, &points, u);
        let mut start = 0;
        for (f, face) in points.faces.iter().enumerate() {
            let end = start
                + points.points[start..]
                    .iter()
                    .take_while(|p| p.face == f)
                    .count();
            let range = &points.points[start..end];
            if range.is_empty() {
                continue;
            }
            let locals: Vec<[f64; 2]> = range.iter().map(|p| p.local).collect();
            let weights: Vec<f64> = range.iter().map(|p| p.weight).collect();
            let fit = |c: usize| {
                let v: Vec<f64> = values[start..end]
                    .iter()
                    .map(|&(o, p)| if c == 0 { o } else { p })
                    .collect();
                face.extrapolate_from(&locals, &weights, &v)
            };
            let (opening, pressure) = (fit(0), fit(1));
            for (k, &node) in face.nodes.iter().enumerate() {
                let node = node as usize;
                sum[node][0] += opening[k];
                // A fit may undershoot at the edge of the contact zone: no tension.
                sum[node][1] += pressure[k].max(0.0);
                count[node] += 1;
            }
            start = end;
        }
    }
    Some(
        sum.iter()
            .zip(&count)
            .map(|(s, &c)| (c > 0).then(|| s.map(|v| v / f64::from(c))))
            .collect(),
    )
}

/// The `CF` statistics of the compression only surfaces a `*CONTACT PRINT` asks for: the
/// one named as its slave without a master, or all when it names no pair.
pub fn history_blocks(
    model: &Model,
    request: &HistoryOutput,
    u: &[f64],
    step: u32,
    increment: u32,
    time: f64,
) -> Vec<HistoryBlock> {
    if !(request.variables.iter()).any(|v| v.trim().eq_ignore_ascii_case("CF")) {
        return Vec::new();
    }
    let mut out = Vec::new();
    for support in &model.contact.compression_only {
        if let Some((slave, master)) = &request.surfaces
            && !(master.is_empty() && slave.eq_ignore_ascii_case(&support.surface))
        {
            continue;
        }
        let Ok(points) = support_points(model, support) else {
            continue;
        };
        let values = point_values(support, &points, u);
        let mut force = [0.0; 3];
        let mut moment = [0.0; 3];
        let mut area = 0.0;
        let mut centre = [0.0; 3];
        let mut normal = [0.0; 3];
        for (p, &(_, pressure)) in points.points.iter().zip(&values) {
            let nodes = &points.faces[p.face].nodes;
            let mut x = p.position;
            for (&node, &w) in nodes.iter().zip(&p.weights) {
                for (k, x) in x.iter_mut().enumerate() {
                    *x += w * u.get(node as usize * 3 + k).copied().unwrap_or(0.0);
                }
            }
            let f = p.direction.map(|d| d * pressure * p.area);
            let m = cross(x, f);
            for k in 0..3 {
                force[k] += f[k];
                moment[k] += m[k];
                normal[k] += p.area * p.direction[k];
            }
            if pressure > 0.0 {
                for k in 0..3 {
                    centre[k] += p.area * x[k];
                }
                area += p.area;
            }
        }
        if area > 0.0 {
            centre = centre.map(|c| c / area);
        }
        let length = (normal.iter().map(|c| c * c).sum::<f64>()).sqrt();
        if length > 0.0 {
            normal = normal.map(|c| c / length);
        }
        let about_cg = cross(force, centre);
        let moment_cg = [0, 1, 2].map(|k| moment[k] + about_cg[k]);
        let normal_force = (0..3).map(|k| force[k] * normal[k]).sum::<f64>();
        let shear = (0..3)
            .map(|k| (force[k] - normal_force * normal[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        out.push(HistoryBlock {
            step,
            increment,
            quantity: format!(
                "statistics for slave set {}, master set {}",
                support.surface, GROUND
            ),
            set: String::new(),
            time,
            rows: vec![
                (0, [force, moment].concat()),
                (0, [centre, normal].concat()),
                (0, moment_cg.to_vec()),
                (0, vec![area, normal_force, shear]),
            ],
        });
    }
    out
}

/// The master of a compression only surface in the statistics: the ground it rests on.
pub const GROUND: &str = "GROUND";

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
