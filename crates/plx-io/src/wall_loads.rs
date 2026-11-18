//! The loads of a Particle Flow run on its walls as nodal forces on the faces of an FE
//! mesh ([`plx_model::LoadKind::WallLoads`]): each point load of the run
//! ([`plx_results::wall_loads`]) is spread over the face nodes near it whose surface faces
//! the same way, weighted by the area of each node and a bell over the distance along the
//! surface. The force is kept whole; its moment moves by less than the spread. A point with
//! no face near it is left out, and counted.
//!
//! The points lie on the wall's surface or up to two cells of the run's grid inside it (the
//! grid nodes the wall held): a face counts as near when the point lies within that depth
//! in front of or behind it. The spread is the cell of the run, or more on a coarser mesh,
//! so that every point reaches some nodes.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use plx_mesh::{ElementFamily, ElementId, FeMesh, NodeId};
use plx_model::{Region, WallLoadTime};
use plx_results::wall_loads::{Pick, PointLoads, WallLoads};

/// How deep behind and in front of a face a point of a wall may lie, in cells.
const DEPTH: f64 = 2.5;
/// The least cosine between the normal of a node and of a point that reaches it: the two
/// sides of a thin blade keep their own loads.
const FACING: f64 = 0.3;

/// The face nodes of a region with the area, the outward normal and the position of each.
#[derive(Clone, Debug, Default)]
pub struct Surface {
    pub nodes: Vec<NodeId>,
    pub points: Vec<[f64; 3]>,
    pub normals: Vec<[f64; 3]>,
    pub areas: Vec<f64>,
}

/// The element faces (element, CalculiX face number) of a region: picked faces, faces of
/// the geometry or a surface, the skin of whole parts.
pub fn region_faces(mesh: &FeMesh, region: &Region) -> Vec<(ElementId, u8)> {
    match region {
        Region::Parts(names) => {
            let elements = mesh.elements();
            let mut faces = Vec::new();
            for part in mesh.parts.iter().filter(|p| names.contains(&p.name)) {
                let skin = plx_mesh::extract_part_skin(mesh, part, 30.0);
                for face in &skin.faces {
                    let element = &elements[face.element];
                    if element.shape.family() == ElementFamily::Solid {
                        faces.push((element.id, face.face as u8 + 1));
                    }
                }
            }
            faces
        }
        _ => region.faces(mesh),
    }
}

impl Surface {
    /// The nodes of the faces: lumped shares of the face area (corners and midside nodes of
    /// quadratic faces all take a part), and the outward normal averaged by area.
    pub fn of(mesh: &FeMesh, faces: &[(ElementId, u8)]) -> Self {
        let mut at: HashMap<NodeId, usize> = HashMap::new();
        let mut out = Surface::default();
        for &(id, face) in faces {
            let Some(element) = mesh.element(id) else {
                continue;
            };
            let Some(topology) = (face as usize)
                .checked_sub(1)
                .and_then(|f| element.faces().get(f))
            else {
                continue;
            };
            let point = |local: usize| element.nodes.get(local).and_then(|&n| mesh.node(n));
            let corners: Vec<[f64; 3]> =
                topology.corners.iter().filter_map(|&l| point(l)).collect();
            if corners.len() != topology.corners.len() || corners.len() < 3 {
                continue;
            }
            let mut normal = newell(&corners);
            let area = 0.5 * length(normal);
            if area <= 0.0 {
                continue;
            }
            normal = normal.map(|v| v * 0.5 / area);
            // Outward: away from the middle of the element.
            let middle = mean(
                &(element.nodes.iter())
                    .filter_map(|&n| mesh.node(n))
                    .collect::<Vec<_>>(),
            );
            let centre = mean(&corners);
            if dot(normal, sub(centre, middle)) < 0.0 {
                normal = normal.map(|v| -v);
            }
            let quadratic = element.shape.is_quadratic() && !topology.mids.is_empty();
            let (corner, mid) = match (corners.len(), quadratic) {
                (3, true) => (1.0 / 19.0, 16.0 / 57.0),
                (4, true) => (3.0 / 76.0, 4.0 / 19.0),
                (n, _) => (1.0 / n as f64, 0.0),
            };
            let mut add = |local: usize, share: f64| {
                let Some(&node) = element.nodes.get(local) else {
                    return;
                };
                let Some(position) = mesh.node(node) else {
                    return;
                };
                let k = *at.entry(node).or_insert_with(|| {
                    out.nodes.push(node);
                    out.points.push(position);
                    out.normals.push([0.0; 3]);
                    out.areas.push(0.0);
                    out.nodes.len() - 1
                });
                out.areas[k] += share * area;
                for (sum, n) in out.normals[k].iter_mut().zip(normal) {
                    *sum += share * area * n;
                }
            };
            for &l in topology.corners {
                add(l, corner);
            }
            if quadratic {
                for &l in topology.mids {
                    add(l, mid);
                }
            }
        }
        for n in &mut out.normals {
            let l = length(*n);
            if l > 0.0 {
                *n = n.map(|v| v / l);
            }
        }
        out
    }

    /// The mean distance between neighbouring nodes.
    fn spacing(&self) -> f64 {
        let area: f64 = self.areas.iter().sum();
        (area / self.nodes.len().max(1) as f64).sqrt()
    }
}

/// Nodal forces of point loads on a surface.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mapped {
    pub forces: BTreeMap<NodeId, [f64; 3]>,
    /// The total force of the points, and of those that reached the surface.
    pub total: [f64; 3],
    pub mapped: [f64; 3],
    /// Points, and points that reached no node.
    pub points: usize,
    pub missed: usize,
}

impl Mapped {
    fn join(&mut self, other: Mapped) {
        for (node, f) in other.forces {
            let sum = self.forces.entry(node).or_default();
            (0..3).for_each(|a| sum[a] += f[a]);
        }
        (0..3).for_each(|a| {
            self.total[a] += other.total[a];
            self.mapped[a] += other.mapped[a];
        });
        self.points += other.points;
        self.missed += other.missed;
    }
}

/// Spreads `loads` onto the nodes of `surface`; `cell` is the cell of the run.
pub fn spread(surface: &Surface, loads: &PointLoads, cell: f64) -> Mapped {
    let mut out = Mapped {
        points: loads.len(),
        ..Default::default()
    };
    for (_, _, f) in loads {
        (0..3).for_each(|a| out.total[a] += f[a]);
    }
    if surface.nodes.is_empty() || loads.is_empty() {
        out.missed = loads.len();
        return out;
    }
    let reach = cell.max(1.5 * surface.spacing());
    let depth = DEPTH * cell;
    // The nodes by cells of the largest reach.
    let size = 4.0 * reach.max(depth);
    let key = |p: [f64; 3]| p.map(|v| (v / size).floor() as i64);
    let mut grid: HashMap<[i64; 3], Vec<usize>> = HashMap::new();
    for (k, &p) in surface.points.iter().enumerate() {
        if surface.areas[k] > 0.0 {
            grid.entry(key(p)).or_default().push(k);
        }
    }
    let mut weights = Vec::new();
    for &(p, n, f) in loads {
        let mut done = false;
        // Wider where the mesh is coarse near the point.
        for widen in [1.0, 2.0, 4.0] {
            let r = reach * widen;
            weights.clear();
            let base = key(p);
            for dz in -1..=1 {
                for dy in -1..=1 {
                    for dx in -1..=1 {
                        let Some(nodes) = grid.get(&[base[0] + dx, base[1] + dy, base[2] + dz])
                        else {
                            continue;
                        };
                        for &k in nodes {
                            let d = sub(surface.points[k], p);
                            let along = dot(d, n);
                            let across = length(sub(d, n.map(|v| v * along)));
                            if along.abs() > depth
                                || across >= r
                                || dot(surface.normals[k], n) < FACING
                            {
                                continue;
                            }
                            let q = 1.0 - (across / r).powi(2);
                            weights.push((k, surface.areas[k] * q * q));
                        }
                    }
                }
            }
            let sum: f64 = weights.iter().map(|(_, w)| w).sum();
            if sum > 0.0 {
                for &(k, w) in &weights {
                    let node = out.forces.entry(surface.nodes[k]).or_default();
                    (0..3).for_each(|a| node[a] += f[a] * w / sum);
                }
                (0..3).for_each(|a| out.mapped[a] += f[a]);
                done = true;
                break;
            }
        }
        if !done {
            out.missed += 1;
        }
    }
    out
}

/// What a wall load of a step needs of the run: which frames, with weights.
pub fn pick(time: WallLoadTime, loads: &WallLoads) -> Vec<(usize, f64)> {
    let end = loads.end_time();
    match time {
        WallLoadTime::Last => loads.frames(Pick::At(end)),
        WallLoadTime::At(t) => loads.frames(Pick::At(t)),
        WallLoadTime::Mean { from, to } => loads.frames(Pick::Mean(from, to.unwrap_or(end))),
        // Each frame at its time.
        WallLoadTime::OverTime => (0..loads.times.len()).map(|k| (k, 1.0)).collect(),
    }
}

/// The nodal forces of a wall load on the faces of `region` of `mesh`: one set of forces,
/// or with [`WallLoadTime::OverTime`] one per frame of the run, with its time.
pub fn nodal_forces(
    mesh: &FeMesh,
    region: &Region,
    load: &plx_model::WallLoads,
    loads: &WallLoads,
) -> Result<Vec<(Option<f64>, Mapped)>, String> {
    if loads.space != plxs_core::particles::Space::Solid {
        return Err("the Particle Flow run is 2D; its wall loads go to 3D models only".into());
    }
    let faces = region_faces(mesh, region);
    if faces.is_empty() {
        return Err("the region has no faces of solid elements".into());
    }
    let surface = Surface::of(mesh, &faces);
    let walls: Vec<usize> = (0..loads.walls.len())
        .filter(|&w| load.takes(&loads.walls[w].name))
        .collect();
    if walls.is_empty() {
        return Err("none of the walls it names is in the run".into());
    }
    let frames = pick(load.time, loads);
    if frames.is_empty() {
        return Err("the run has no frames after its start".into());
    }
    let map = |frames: &[(usize, f64)]| {
        let mut mapped = Mapped::default();
        for &w in &walls {
            mapped.join(spread(&surface, &loads.point_loads(w, frames), loads.cell));
        }
        mapped
    };
    Ok(match load.time {
        WallLoadTime::OverTime => (frames.iter())
            .map(|&(k, _)| (Some(loads.times[k]), map(&[(k, 1.0)])))
            .collect(),
        _ => vec![(None, map(&frames))],
    })
}

/// Reads the loads file of a wall load.
pub fn read(file: &Path) -> Result<WallLoads, String> {
    WallLoads::read(file)
}

fn newell(corners: &[[f64; 3]]) -> [f64; 3] {
    let mut sum = [0.0; 3];
    for (i, a) in corners.iter().enumerate() {
        let b = corners[(i + 1) % corners.len()];
        sum[0] += a[1] * b[2] - a[2] * b[1];
        sum[1] += a[2] * b[0] - a[0] * b[2];
        sum[2] += a[0] * b[1] - a[1] * b[0];
    }
    sum
}

fn mean(points: &[[f64; 3]]) -> [f64; 3] {
    let n = points.len().max(1) as f64;
    let mut m = [0.0; 3];
    for p in points {
        (0..3).for_each(|a| m[a] += p[a] / n);
    }
    m
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn length(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_mesh::{Element, ElementShape, Part};
    use plxs_core::particles::{Space, WallFrame};

    /// A block of `n`³ eight-node bricks of edge `size / n`, one part "Plate".
    fn block(n: usize, size: f64) -> FeMesh {
        let mut mesh = FeMesh::default();
        let h = size / n as f64;
        let id = |i: usize, j: usize, k: usize| (1 + i + (n + 1) * (j + (n + 1) * k)) as NodeId;
        for k in 0..=n {
            for j in 0..=n {
                for i in 0..=n {
                    mesh.set_node(id(i, j, k), [i as f64 * h, j as f64 * h, k as f64 * h]);
                }
            }
        }
        let mut elements = Vec::new();
        for k in 0..n {
            for j in 0..n {
                for i in 0..n {
                    let e = (1 + i + n * (j + n * k)) as ElementId;
                    mesh.add_element(Element {
                        id: e,
                        type_name: "C3D8".into(),
                        shape: ElementShape::Hex8,
                        nodes: vec![
                            id(i, j, k),
                            id(i + 1, j, k),
                            id(i + 1, j + 1, k),
                            id(i, j + 1, k),
                            id(i, j, k + 1),
                            id(i + 1, j, k + 1),
                            id(i + 1, j + 1, k + 1),
                            id(i, j + 1, k + 1),
                        ],
                    })
                    .unwrap();
                    elements.push(e);
                }
            }
        }
        mesh.parts.push(Part::new("Plate", elements));
        mesh
    }

    /// Loads on the top of the block: points a cell apart a little inside it.
    fn top_loads(cell: f64, size: f64, pressure: f64) -> WallLoads {
        let mut frame = WallFrame::default();
        let count = (size / cell).round() as usize;
        for i in 0..count {
            for j in 0..count {
                let (x, y) = ((i as f64 + 0.5) * cell, (j as f64 + 0.5) * cell);
                frame
                    .points
                    .push([x as f32, y as f32, (size - 0.6 * cell) as f32]);
                frame.normals.push([0.0, 0.0, 1.0]);
                frame
                    .forces
                    .push([0.1, 0.0, (-pressure * cell * cell) as f32]);
            }
        }
        let mut loads = WallLoads::new(
            cell,
            Space::Solid,
            vec![("Press".into(), vec!["Plate".into()])],
        );
        loads.push(1.0, &[frame]);
        loads
    }

    #[test]
    fn a_pressure_on_a_face_comes_whole_and_even() {
        for (n, cell) in [(10, 0.01), (2, 0.01), (40, 0.01)] {
            let mesh = block(n, 0.1);
            let loads = top_loads(cell, 0.1, 1000.0);
            let load = plx_model::WallLoads::default();
            let region = Region::Parts(vec!["Plate".into()]);
            let out = nodal_forces(&mesh, &region, &load, &loads).unwrap();
            let mapped = &out[0].1;
            assert_eq!(mapped.missed, 0, "{n}");
            // The whole force, on the top only.
            let fz: f64 = mapped.forces.values().map(|f| f[2]).sum();
            assert!((fz + 10.0).abs() < 1e-4, "{n}: {fz}");
            for (node, f) in &mapped.forces {
                let z = mesh.node(*node).unwrap()[2];
                assert!(
                    (z - 0.1).abs() < 1e-9 || f[2].abs() < 1e-12,
                    "{n}: node at {z}"
                );
            }
            // An inner node of the top takes its area times the pressure.
            if n == 10 {
                let middle = mesh
                    .node_ids()
                    .iter()
                    .copied()
                    .find(|&id| {
                        let p = mesh.node(id).unwrap();
                        (p[0] - 0.05).abs() < 1e-9 && (p[1] - 0.05).abs() < 1e-9 && p[2] > 0.099
                    })
                    .unwrap();
                let f = mapped.forces[&middle][2];
                assert!((f + 1000.0 * 0.01 * 0.01).abs() < 0.1 * 0.1, "middle {f}");
            }
        }
    }

    #[test]
    fn the_far_side_of_a_thin_plate_keeps_its_own_loads() {
        // Points under the top of a block two cells thick, pushing up from below: they
        // face down, so the top takes nothing.
        let mesh = block(4, 0.02);
        let mut frame = WallFrame::default();
        frame.points.push([0.01, 0.01, 0.019]);
        frame.normals.push([0.0, 0.0, 1.0]);
        frame.forces.push([0.0, 0.0, -1.0]);
        frame.points.push([0.01, 0.01, 0.001]);
        frame.normals.push([0.0, 0.0, -1.0]);
        frame.forces.push([0.0, 0.0, 2.0]);
        let mut loads = WallLoads::new(0.01, Space::Solid, vec![("Blade".into(), vec![])]);
        loads.push(1.0, &[frame]);
        let region = Region::Parts(vec!["Plate".into()]);
        let out = nodal_forces(&mesh, &region, &Default::default(), &loads).unwrap();
        for (node, f) in &out[0].1.forces {
            let z = mesh.node(*node).unwrap()[2];
            if z > 0.01 {
                assert!(f[2] <= 1e-12, "top pulled up: {f:?}");
            } else {
                assert!(f[2] >= -1e-12, "bottom pushed down: {f:?}");
            }
        }
        let fz: f64 = out[0].1.forces.values().map(|f| f[2]).sum();
        assert!((fz - 1.0).abs() < 1e-9);
    }
}
