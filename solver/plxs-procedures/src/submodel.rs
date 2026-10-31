//! `*SUBMODEL, TYPE=NODE` with `*BOUNDARY, SUBMODEL, STEP=n` (concept section 4.3, M3 T15).
//!
//! The nodes of the submodel's node sets take their displacements from the results of a
//! global model (CalculiX manual, `*SUBMODEL` and "Submodel"): every node is located in an
//! element of the global mesh, its natural coordinates found by Newton's method on the
//! element's isoparametric map, and the global displacements of the last increment of step
//! `n` are interpolated with the element's shape functions. A node outside the global mesh
//! takes the element it is closest to in natural coordinates (extrapolation), with a warning.
//!
//! The global results come from the ASCII `.frd` file named by `INPUT`, relative to the
//! job's directory; its solid elements (also the expanded ones of shells, beams and plane
//! elements) carry the interpolation. Only the displacement degrees of freedom 1 to 3 are
//! supported (PrePoLix writes no thermal submodels).
//!
//! No CalculiX code was consulted.

use std::collections::HashMap;

use plxs_assembly::Spc;
use plxs_assembly::dofs::amplitude_factor;
use plxs_core::mesh::NodeRef;
use plxs_core::{Error, Model, Progress, ProgressEvent, Result, Step};
use plxs_elements::Family;

/// The mesh and displacements of a global model's `.frd`.
#[derive(Clone, Debug, Default)]
pub struct GlobalResults {
    pub nodes: HashMap<u32, [f64; 3]>,
    /// Solid elements: family and node ids in CalculiX's order.
    pub elements: Vec<(Family, Vec<u32>)>,
    /// `DISP` of the last increment of every step: step number to node values.
    pub displacements: HashMap<u32, HashMap<u32, [f64; 3]>>,
}

fn field(line: &str, range: std::ops::Range<usize>) -> Option<&str> {
    line.get(range).map(str::trim)
}

/// The frd element types this module interpolates in and the CalculiX index of every frd
/// node position.
fn frd_family(kind: u32) -> Option<(Family, &'static [usize])> {
    const HEX8: [usize; 8] = [0, 1, 2, 3, 4, 5, 6, 7];
    // frd lists the vertical midside nodes of quadratic hexahedra and wedges before the top
    // ones (the inverse of the writer's order).
    const HEX20: [usize; 20] = [
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 16, 17, 18, 19, 12, 13, 14, 15,
    ];
    const TET4: [usize; 4] = [0, 1, 2, 3];
    const TET10: [usize; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
    const WEDGE6: [usize; 6] = [0, 1, 2, 3, 4, 5];
    const WEDGE15: [usize; 15] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 13, 14, 9, 10, 11];
    Some(match kind {
        1 => (Family::Hex8, &HEX8),
        2 => (Family::Wedge6, &WEDGE6),
        3 => (Family::Tet4, &TET4),
        4 => (Family::Hex20, &HEX20),
        5 => (Family::Wedge15, &WEDGE15),
        6 => (Family::Tet10, &TET10),
        _ => return None,
    })
}

/// Reads the mesh and the displacements of an ASCII `.frd` (CalculiX's default output).
pub fn read_global_frd(text: &str) -> std::result::Result<GlobalResults, String> {
    let mut out = GlobalResults::default();
    let mut lines = text.lines().peekable();
    let mut step = 0u32;
    while let Some(line) = lines.next() {
        if line.starts_with("    2C") {
            for l in lines.by_ref() {
                if l.starts_with(" -3") {
                    break;
                }
                if !l.starts_with(" -1") {
                    continue;
                }
                let id: u32 = field(l, 3..13)
                    .and_then(|s| s.parse().ok())
                    .ok_or_else(|| format!("bad node line: {l}"))?;
                let mut x = [0.0; 3];
                for (d, v) in x.iter_mut().enumerate() {
                    *v = field(l, 13 + 12 * d..25 + 12 * d)
                        .and_then(|s| s.parse().ok())
                        .ok_or_else(|| format!("bad node line: {l}"))?;
                }
                out.nodes.insert(id, x);
            }
        } else if line.starts_with("    3C") {
            let mut current: Option<(u32, Vec<u32>)> = None;
            let finish = |current: &mut Option<(u32, Vec<u32>)>, out: &mut GlobalResults| {
                if let Some((kind, frd_nodes)) = current.take()
                    && let Some((family, order)) = frd_family(kind)
                    && frd_nodes.len() == order.len()
                {
                    let mut nodes = vec![0; order.len()];
                    for (k, &i) in order.iter().enumerate() {
                        nodes[i] = frd_nodes[k];
                    }
                    out.elements.push((family, nodes));
                }
            };
            for l in lines.by_ref() {
                if l.starts_with(" -3") {
                    break;
                }
                if l.starts_with(" -1") {
                    finish(&mut current, &mut out);
                    let kind: u32 = field(l, 13..18).and_then(|s| s.parse().ok()).unwrap_or(0);
                    current = Some((kind, Vec::new()));
                } else if l.starts_with(" -2")
                    && let Some((_, nodes)) = current.as_mut()
                {
                    let mut at = 3;
                    while let Some(v) = field(l, at..at + 10) {
                        if v.is_empty() {
                            break;
                        }
                        nodes.push(v.parse().map_err(|_| format!("bad element line: {l}"))?);
                        at += 10;
                    }
                }
            }
            finish(&mut current, &mut out);
        } else if line.starts_with("    1PSTEP") {
            if let Some(s) = line.get(48..60).and_then(|s| s.trim().parse().ok()) {
                step = s;
            }
        } else if line.starts_with(" -4") && field(line, 5..13) == Some("DISP") {
            // A later increment of the step replaces the earlier ones.
            let values = out.displacements.entry(step).or_default();
            values.clear();
            for l in lines.by_ref() {
                if l.starts_with(" -3") {
                    break;
                }
                if !l.starts_with(" -1") {
                    continue;
                }
                let Some(id) = field(l, 3..13).and_then(|s| s.parse().ok()) else {
                    continue;
                };
                let mut u = [0.0; 3];
                for (d, v) in u.iter_mut().enumerate() {
                    *v = field(l, 13 + 12 * d..25 + 12 * d)
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(0.0);
                }
                values.insert(id, u);
            }
        }
    }
    if out.elements.is_empty() {
        return Err("the file has no solid elements".into());
    }
    Ok(out)
}

/// How far natural coordinates lie outside an element's domain (zero inside).
fn outside(family: Family, p: [f64; 3]) -> f64 {
    match family {
        Family::Hex8 | Family::Hex20 => p.iter().map(|v| (v.abs() - 1.0).max(0.0)).sum(),
        Family::Tet4 | Family::Tet10 => {
            p.iter().map(|v| (-v).max(0.0)).sum::<f64>() + (p[0] + p[1] + p[2] - 1.0).max(0.0)
        }
        Family::Wedge6 | Family::Wedge15 => {
            (-p[0]).max(0.0)
                + (-p[1]).max(0.0)
                + (p[0] + p[1] - 1.0).max(0.0)
                + (p[2].abs() - 1.0).max(0.0)
        }
    }
}

/// Natural coordinates of `x` in an element (Newton on the isoparametric map).
fn natural(family: Family, coords: &[[f64; 3]], x: [f64; 3]) -> Option<[f64; 3]> {
    let m = coords.len();
    let mut n = vec![0.0; m];
    let mut dn = vec![[0.0; 3]; m];
    let mut p = match family {
        Family::Hex8 | Family::Hex20 => [0.0; 3],
        Family::Tet4 | Family::Tet10 => [0.25; 3],
        Family::Wedge6 | Family::Wedge15 => [1.0 / 3.0, 1.0 / 3.0, 0.0],
    };
    for _ in 0..30 {
        family.shape(p, &mut n, &mut dn);
        let mut r = x;
        let mut j = [[0.0; 3]; 3];
        for i in 0..m {
            for d in 0..3 {
                r[d] -= n[i] * coords[i][d];
                for k in 0..3 {
                    j[d][k] += coords[i][d] * dn[i][k];
                }
            }
        }
        let det = j[0][0] * (j[1][1] * j[2][2] - j[1][2] * j[2][1])
            - j[0][1] * (j[1][0] * j[2][2] - j[1][2] * j[2][0])
            + j[0][2] * (j[1][0] * j[2][1] - j[1][1] * j[2][0]);
        if det.abs() < 1e-300 {
            return None;
        }
        // Cramer's rule for J dp = r.
        let solve = |col: usize| {
            let mut a = j;
            for d in 0..3 {
                a[d][col] = r[d];
            }
            (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
                / det
        };
        let dp = [solve(0), solve(1), solve(2)];
        for k in 0..3 {
            p[k] += dp[k];
        }
        if dp.iter().map(|v| v.abs()).sum::<f64>() < 1e-12 {
            return Some(p);
        }
        if p.iter().any(|v| !v.is_finite() || v.abs() > 1e3) {
            return None;
        }
    }
    Some(p)
}

/// Where a submodel node lies in the global mesh: element and shape function values.
#[derive(Clone, Debug)]
struct Location {
    element: usize,
    weights: Vec<f64>,
    outside: f64,
}

impl GlobalResults {
    /// Locates points in the global mesh. Candidates come from a uniform grid of element
    /// boxes; a point without an element whose box holds it tries all elements.
    fn locate(&self, points: &[[f64; 3]]) -> Vec<Option<Location>> {
        let boxes: Vec<([f64; 3], [f64; 3])> = self
            .elements
            .iter()
            .map(|(_, nodes)| {
                let mut lo = [f64::INFINITY; 3];
                let mut hi = [f64::NEG_INFINITY; 3];
                for id in nodes {
                    if let Some(x) = self.nodes.get(id) {
                        for d in 0..3 {
                            lo[d] = lo[d].min(x[d]);
                            hi[d] = hi[d].max(x[d]);
                        }
                    }
                }
                (lo, hi)
            })
            .collect();
        let size = boxes
            .iter()
            .map(|(lo, hi)| (0..3).map(|d| hi[d] - lo[d]).fold(0.0, f64::max))
            .sum::<f64>()
            / boxes.len().max(1) as f64;
        let cell = size.max(1e-12);
        let key = |x: f64| (x / cell).floor() as i64;
        let mut grid: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
        for (e, (lo, hi)) in boxes.iter().enumerate() {
            let margin = 1e-6 * cell;
            let (a, b) = (lo.map(|v| key(v - margin)), hi.map(|v| key(v + margin)));
            for i in a[0]..=b[0] {
                for j in a[1]..=b[1] {
                    for k in a[2]..=b[2] {
                        grid.entry((i, j, k)).or_default().push(e);
                    }
                }
            }
        }
        let all: Vec<usize> = (0..self.elements.len()).collect();
        points
            .iter()
            .map(|&x| {
                let near = grid.get(&(key(x[0]), key(x[1]), key(x[2])));
                let mut best = self.best_in(near.map_or(&all[..], |v| &v[..]), x);
                if best.as_ref().is_none_or(|b| b.outside > 1e-6) {
                    let everywhere = self.best_in(&all, x);
                    if everywhere
                        .as_ref()
                        .is_some_and(|e| best.as_ref().is_none_or(|b| e.outside < b.outside))
                    {
                        best = everywhere;
                    }
                }
                best
            })
            .collect()
    }

    fn best_in(&self, candidates: &[usize], x: [f64; 3]) -> Option<Location> {
        let mut best: Option<Location> = None;
        for &e in candidates {
            let (family, nodes) = &self.elements[e];
            let Some(coords) = nodes
                .iter()
                .map(|id| self.nodes.get(id).copied())
                .collect::<Option<Vec<_>>>()
            else {
                continue;
            };
            let Some(p) = natural(*family, &coords, x) else {
                continue;
            };
            let distance = outside(*family, p);
            if best.as_ref().is_none_or(|b| distance < b.outside) {
                let mut n = vec![0.0; coords.len()];
                let mut dn = vec![[0.0; 3]; coords.len()];
                family.shape(p, &mut n, &mut dn);
                best = Some(Location {
                    element: e,
                    weights: n,
                    outside: distance,
                });
                if distance == 0.0 {
                    break;
                }
            }
        }
        best
    }
}

/// The global displacements at the nodes of the submodel boundaries of a step: for every
/// boundary with `SUBMODEL`, the dense node and its displacement vector (global
/// components). Empty when the step has none.
pub struct SubmodelValues {
    /// (boundary index, dense node) to the interpolated displacement.
    values: HashMap<(usize, u32), [f64; 3]>,
}

impl SubmodelValues {
    pub fn of_step(
        model: &Model,
        step: &Step,
        work_dir: &std::path::Path,
        progress: &mut dyn Progress,
    ) -> Result<SubmodelValues> {
        let mut values = HashMap::new();
        if !step.boundaries.iter().any(|b| b.submodel_step.is_some()) {
            return Ok(SubmodelValues { values });
        }
        let Some(submodel) = &model.constraints.submodel else {
            return Err(Error::input(
                "*BOUNDARY",
                "SUBMODEL needs a *SUBMODEL definition",
            ));
        };
        if submodel.global_elset.is_some() {
            progress.event(ProgressEvent::Warning(
                "*SUBMODEL: GLOBAL ELSET is not supported; all elements of the global model are used".into(),
            ));
        }
        let path = work_dir.join(&submodel.input);
        let text = std::fs::read(&path).map_err(|e| Error::Io {
            path: path.display().to_string(),
            message: e.to_string(),
        })?;
        let global = read_global_frd(&String::from_utf8_lossy(&text))
            .map_err(|e| Error::input("*SUBMODEL", format!("{}: {e}", path.display())))?;
        let mut outside = 0usize;
        for (index, boundary) in step.boundaries.iter().enumerate() {
            let Some(global_step) = boundary.submodel_step else {
                continue;
            };
            if boundary.first_dof > 3 {
                return Err(Error::Unsupported(format!(
                    "*BOUNDARY, SUBMODEL on degree of freedom {}: only displacements (1-3) are supported",
                    boundary.first_dof
                )));
            }
            let Some(displacements) = global.displacements.get(&global_step) else {
                return Err(Error::input(
                    "*BOUNDARY, SUBMODEL",
                    format!(
                        "the global results {} have no displacements of step {global_step}",
                        path.display()
                    ),
                ));
            };
            let ids = match &boundary.region {
                NodeRef::Node(id) => vec![*id],
                NodeRef::Set(name) => model.sets.node_sets.get(name).cloned().unwrap_or_default(),
            };
            let nodes: Vec<u32> = ids.iter().filter_map(|&id| model.nodes.index(id)).collect();
            let points: Vec<[f64; 3]> = nodes
                .iter()
                .map(|&i| model.nodes.coordinates[i as usize])
                .collect();
            for (node, location) in nodes.iter().zip(global.locate(&points)) {
                let Some(location) = location else {
                    return Err(Error::input(
                        "*SUBMODEL",
                        format!(
                            "node {} lies in no element of the global model",
                            model.nodes.ids[*node as usize]
                        ),
                    ));
                };
                if location.outside > 1e-6 {
                    outside += 1;
                }
                let element = &global.elements[location.element].1;
                let mut u = [0.0; 3];
                for (id, w) in element.iter().zip(&location.weights) {
                    let value = displacements.get(id).copied().unwrap_or([0.0; 3]);
                    for d in 0..3 {
                        u[d] += w * value[d];
                    }
                }
                values.insert((index, *node), u);
            }
        }
        if outside > 0 {
            progress.event(ProgressEvent::Warning(format!(
                "*SUBMODEL: {outside} node(s) lie outside the global mesh; their displacements are extrapolated from the nearest element"
            )));
        }
        Ok(SubmodelValues { values })
    }

    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    /// Writes the interpolated displacements into the step's prescribed values at step
    /// time `time`, scaled by the boundary's amplitude like any other prescribed
    /// displacement. `rotation(node)` gives the local system of transformed nodes (the
    /// prescribed DOFs are local there).
    #[allow(clippy::too_many_arguments)]
    pub fn apply(
        &self,
        model: &Model,
        step: &Step,
        dofs: &plxs_assembly::DofMap,
        spc: &mut Spc,
        time: f64,
        total: f64,
        period: f64,
        rotation: &dyn Fn(u32) -> Option<[[f64; 3]; 3]>,
    ) {
        if self.values.is_empty() {
            return;
        }
        for (index, boundary) in step.boundaries.iter().enumerate() {
            if boundary.submodel_step.is_none() {
                continue;
            }
            let factor = amplitude_factor(
                model,
                step,
                boundary.amplitude.as_deref(),
                time,
                total,
                period,
            );
            for (&(b, node), u) in &self.values {
                if b != index {
                    continue;
                }
                for d in boundary.first_dof..=boundary.last_dof.min(3) {
                    let k = d as usize - 1;
                    let Some(dof) = dofs.dof(node, k) else {
                        continue;
                    };
                    let value = match rotation(node) {
                        None => u[k],
                        Some(r) => (0..3).map(|g| r[g][k] * u[g]).sum(),
                    };
                    if let Ok(at) = spc.dofs.binary_search(&dof) {
                        spc.values[at] = value * factor;
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_point_in_a_distorted_hexahedron_is_found_and_interpolated() {
        let coords = [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.2, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [2.0, 0.0, 1.0],
            [2.2, 1.0, 1.3],
            [0.0, 1.0, 1.0],
        ];
        let p = natural(Family::Hex8, &coords, [1.0, 0.5, 0.5]).unwrap();
        assert_eq!(outside(Family::Hex8, p), 0.0);
        let mut n = vec![0.0; 8];
        let mut dn = vec![[0.0; 3]; 8];
        Family::Hex8.shape(p, &mut n, &mut dn);
        let x: Vec<f64> = (0..3)
            .map(|d| (0..8).map(|i| n[i] * coords[i][d]).sum())
            .collect();
        assert!(
            (x[0] - 1.0).abs() < 1e-12 && (x[1] - 0.5).abs() < 1e-12 && (x[2] - 0.5).abs() < 1e-12
        );
        assert!(
            natural(Family::Hex8, &coords, [3.0, 0.5, 0.5])
                .is_some_and(|p| outside(Family::Hex8, p) > 0.0)
        );
    }
}
