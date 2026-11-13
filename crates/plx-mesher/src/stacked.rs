//! Stacked sweep: hexahedra for parts made of extrusions along one direction, such as a
//! stepped shaft, a plate with a boss and counterbored holes or a block with a pocket.
//!
//! Such a part has only two kinds of faces: caps, planar and normal to the direction, and
//! walls, which run along it. Every cap is pushed onto one plane and the caps are cut at each
//! other there, into cells. The cells are meshed together, mostly with quadrilaterals, and
//! each cell's mesh is swept through the heights at which the part has material above the
//! cell. Nodes stand in straight columns, so the elements of neighbouring cells share them.
//!
//! The geometry stays as it is; the mesh records where its faces, edges and vertices lie
//! geometrically, as for bodies of revolution.

use std::collections::{BTreeMap, BTreeSet};

use plx_model::{ExtrudeMesh, Geometry, PeriodicFaces, Recombination};

use crate::extrude::{
    Motion, Point, Topology, add, dot, norm, recombination_number, scale, source_elements, sub,
    sweep_mesh,
};
use crate::gmsh::{Gmsh, GmshError};
use crate::{
    GeneratedMesh, LocalSize, MeshOptions, generate_faces, node_coords, periodic, revolve,
};

/// How a part is a stack of extrusions.
struct Stack {
    /// Unit direction of the extrusions.
    direction: Point,
    /// The caps with their heights along the direction.
    caps: Vec<(i32, f64)>,
    /// The walls.
    walls: Vec<i32>,
}

/// Unit normals of a face at a grid of its parameters, `n` each way.
fn normals(gmsh: &Gmsh, face: i32, n: usize) -> Result<Vec<Point>, GmshError> {
    let (min, max) = gmsh.parameter_bounds(2, face)?;
    let (&[u0, v0], &[u1, v1]) = (min.as_slice(), max.as_slice()) else {
        return Err(GmshError::Other(format!(
            "surface {face} has no parameters"
        )));
    };
    let mut parameters = Vec::with_capacity(2 * n * n);
    for i in 0..n {
        for j in 0..n {
            let at = |k: usize| (k as f64 + 0.5) / n as f64;
            parameters.push(u0 + (u1 - u0) * at(i));
            parameters.push(v0 + (v1 - v0) * at(j));
        }
    }
    gmsh.normals(face, &parameters)
}

/// The direction the part is a stack of extrusions along, with its caps and walls, or why
/// there is none.
fn find_stack(gmsh: &Gmsh, topology: &Topology) -> Result<Result<Stack, String>, GmshError> {
    // Normals are unit vectors: these are angles of about a thousandth of a degree.
    const ALONG: f64 = 1.0 - 1e-9;
    const ACROSS: f64 = 2e-5;
    let mut sampled: BTreeMap<i32, (bool, Vec<Point>)> = BTreeMap::new();
    let mut candidates: Vec<Point> = Vec::new();
    for &face in &topology.faces {
        let planar = gmsh.entity_type(2, face)? == "Plane";
        let normals = normals(gmsh, face, 5)?;
        if planar
            && let Some(&normal) = normals.first()
            && !(candidates.iter()).any(|&c| dot(c, normal).abs() > ALONG)
        {
            candidates.push(normal);
        }
        sampled.insert(face, (planar, normals));
    }
    if candidates.is_empty() {
        return Ok(Err("it has no planar face".into()));
    }
    let mut best: Option<(f64, Stack)> = None;
    let mut reason = String::new();
    'directions: for direction in candidates {
        let (mut caps, mut walls) = (Vec::new(), Vec::new());
        let mut area = 0.0;
        for (&face, (planar, normals)) in &sampled {
            let along = |limit: &dyn Fn(f64) -> bool| {
                normals.iter().all(|&n| limit(dot(n, direction).abs()))
            };
            if *planar && along(&|a| a > ALONG) {
                let height = dot(gmsh.center_of_mass(2, face)?, direction);
                caps.push((face, height));
                area += gmsh.mass(2, face)?;
            } else if along(&|a| a < ACROSS) {
                walls.push(face);
            } else {
                if reason.is_empty() {
                    reason = format!("surface {face} neither ends nor runs along the extrusions");
                }
                continue 'directions;
            }
        }
        // A plain block is a stack three ways: the one with the largest ends has the
        // fewest layers.
        if best.as_ref().is_none_or(|(a, _)| area > *a) {
            best = Some((
                area,
                Stack {
                    direction,
                    caps,
                    walls,
                },
            ));
        }
    }
    Ok(best.map(|(_, stack)| stack).ok_or(reason))
}

/// A size the layers between two heights are limited to: (lowest height, highest height,
/// size).
type LayerSize = (f64, f64, f64);

/// The local mesh sizes of the part's faces and edges on what they are in the plane of the
/// cells: a cap is its cells, a wall and an edge the cell edges below them. A wall and an
/// edge that rises along the direction also limit the layers between their heights: the
/// second list, of (lowest height, highest height, size).
fn local_sizes_on_cells(
    gmsh: &Gmsh,
    geometry: &Geometry,
    topology: &Topology,
    stack: &Stack,
    base: f64,
    cap_cells: &BTreeMap<i32, Vec<i32>>,
    cells: &[i32],
) -> Result<(Vec<LocalSize>, Vec<LayerSize>), GmshError> {
    let tolerance = 1e-5 * topology.size;
    let mut layer_sizes = Vec::new();
    let mut cell_edges = BTreeSet::new();
    for &cell in cells {
        cell_edges.extend(gmsh.adjacencies(2, cell)?.1);
    }
    // A point in the middle of each cell edge.
    let mut middles = Vec::new();
    for &edge in &cell_edges {
        let (min, max) = gmsh.parameter_bounds(1, edge)?;
        if let (Some(&a), Some(&b)) = (min.first(), max.first())
            && let Some(&p) = gmsh.values(1, edge, &[0.5 * (a + b)])?.first()
        {
            middles.push((edge, p));
        }
    }
    // The point lifted from the plane of the cells to the height of a point of the entity.
    let lifted = |p: Point, on_entity: Point| {
        add(
            p,
            scale(stack.direction, dot(on_entity, stack.direction) - base),
        )
    };
    let mut sizes = Vec::new();
    // A depth below a cap holds the size in the layers that far from it; below a wall, in
    // the cells that far from its edges in the plane of the cells.
    for (faces, edges, size, depth) in geometry.local_sizes() {
        let mut on_faces = BTreeSet::new();
        let mut on_edges = BTreeSet::new();
        for face in faces.iter().filter(|f| topology.faces.contains(f)) {
            if let Some(own) = cap_cells.get(face) {
                on_faces.extend(own.iter().copied());
                if depth > 0.0
                    && let Some(&(min, max)) = topology.boxes.get(&(2, *face))
                {
                    let height = |p: [f64; 3]| dot(p, stack.direction);
                    let (a, b) = (height(min), height(max));
                    layer_sizes.push((a.min(b) - depth, a.max(b) + depth, size));
                }
            } else if stack.walls.contains(face) {
                if let Some(&(min, max)) = topology.boxes.get(&(2, *face)) {
                    let along = [0, 1].map(|side| {
                        (0..3)
                            .map(|k| {
                                let (a, b) =
                                    (min[k] * stack.direction[k], max[k] * stack.direction[k]);
                                if side == 0 { a.min(b) } else { a.max(b) }
                            })
                            .sum::<f64>()
                    });
                    layer_sizes.push((along[0], along[1], size));
                }
                let anchor = gmsh.center_of_mass(2, *face)?;
                for &(edge, p) in &middles {
                    let p = lifted(p, anchor);
                    if norm(sub(gmsh.closest_point(2, *face, p)?, p)) <= tolerance {
                        on_edges.insert(edge);
                    }
                }
            }
        }
        for edge in edges.iter().filter(|e| topology.edges.contains(e)) {
            let ends: Vec<Point> = (topology.edge_vertices.get(edge).into_iter().flatten())
                .filter_map(|v| topology.points.get(v).copied())
                .collect();
            let Some(anchor) = ends.first() else {
                continue;
            };
            if let [a, b] = ends[..] {
                let (a, b) = (dot(a, stack.direction), dot(b, stack.direction));
                if (a - b).abs() > tolerance {
                    layer_sizes.push((a.min(b), a.max(b), size));
                }
            }
            for &(cell_edge, p) in &middles {
                let p = lifted(p, *anchor);
                if norm(sub(gmsh.closest_point(1, *edge, p)?, p)) <= tolerance {
                    on_edges.insert(cell_edge);
                }
            }
        }
        if !(on_faces.is_empty() && on_edges.is_empty()) {
            sizes.push(LocalSize {
                faces: on_faces.into_iter().map(f64::from).collect(),
                edges: on_edges.into_iter().map(f64::from).collect(),
                size,
                depth,
            });
        }
    }
    Ok((sizes, layer_sizes))
}

/// Meshes a solid as a stack of extrusions. The other solids are gone from the Gmsh model
/// already. The inner error says why the part is none; the Gmsh model may then have changed.
pub(crate) fn stacked_part_mesh(
    gmsh: &Gmsh,
    geometry: &Geometry,
    part: &str,
    volume: i32,
    extrude: &ExtrudeMesh,
    matching: &[PeriodicFaces],
) -> Result<Result<GeneratedMesh, String>, GmshError> {
    let refuse = |reason: String| Ok(Err(format!("{part}: no stack of extrusions: {reason}")));
    let topology = Topology::read(gmsh, volume)?;
    // Taken while the part's own faces are there.
    let rotations = periodic::rotations(gmsh, matching)?;
    let stack = match find_stack(gmsh, &topology)? {
        Ok(stack) => stack,
        Err(reason) => return refuse(reason),
    };
    let part_volume = gmsh.mass(3, volume)?;
    let tolerance = 1e-6 * topology.size;
    // The heights the caps lie at, rising.
    let mut heights: Vec<f64> = stack.caps.iter().map(|c| c.1).collect();
    heights.sort_by(f64::total_cmp);
    heights.dedup_by(|a, b| (*a - *b).abs() <= tolerance);
    let (Some(&base), Some(&top)) = (heights.first(), heights.last()) else {
        return refuse("it has no ends".into());
    };
    if heights.len() < 2 {
        return refuse("its ends lie at one height".into());
    }

    // Every cap pushed onto the lowest height, and cut at the others there.
    let mut copies = Vec::with_capacity(stack.caps.len());
    for &(face, height) in &stack.caps {
        let copy = gmsh.copy(&[(2, face)])?;
        gmsh.translate(&copy, scale(stack.direction, base - height))?;
        copies.extend(copy);
    }
    let (_, became) = gmsh.fragment(&copies[..1], &copies[1..])?;
    let mut cap_cells: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
    let mut cells = BTreeSet::new();
    for (&(cap, _), pieces) in stack.caps.iter().zip(&became) {
        let own: Vec<i32> = (pieces.iter()).filter(|p| p.0 == 2).map(|p| p.1).collect();
        cells.extend(own.iter().copied());
        cap_cells.insert(cap, own);
    }
    let cells: Vec<i32> = cells.into_iter().collect();
    if cells.is_empty() {
        return refuse("its ends could not be laid over each other".into());
    }

    // Only the cells are meshed.
    let mut all = Vec::new();
    for dim in 0..=3 {
        all.extend(gmsh.entities(dim)?.into_iter().map(|t| (dim, t)));
    }
    gmsh.set_visibility(&all, false)?;
    let shown: Vec<(i32, i32)> = cells.iter().map(|&c| (2, c)).collect();
    gmsh.set_visibility(&shown, true)?;
    let mut options = MeshOptions::of_part(geometry, part);
    options.algorithms.0 = extrude.algorithm_2d;
    options.quads = extrude.recombination != Recombination::None;
    options.recombination = recombination_number(extrude.recombination);
    options.visible_only = true;
    let (sizes, layer_sizes) =
        local_sizes_on_cells(gmsh, geometry, &topology, &stack, base, &cap_cells, &cells)?;
    // The edges of the cut faces of a cyclic symmetry are edges of the cells: meshed alike,
    // the cut faces are.
    let mut curves = BTreeSet::new();
    for &cell in &cells {
        curves.extend(gmsh.adjacencies(2, cell)?.1);
    }
    let curves: Vec<i32> = curves.into_iter().collect();
    periodic::match_rotated(gmsh, 1, &curves, &rotations)?;
    let mut notes = Vec::new();
    generate_faces(gmsh, &mut options, &cells, &sizes, &mut notes)?;
    gmsh.set_number("Mesh.MeshOnlyVisible", 0.0)?;
    let quadratic = options.order == 2;
    let coords = node_coords(gmsh)?;
    let motion = Motion::Translate(scale(stack.direction, top - base));
    let (elements, _) = source_elements(gmsh, &cells, quadratic, &coords, &motion)?;
    if elements.is_empty() {
        return Err(GmshError::Other(format!(
            "{part}: Gmsh created no elements on the ends"
        )));
    }

    // The layers: those the item asks for over the whole height, counted in each interval
    // between two heights of caps, which gets at least one, and spread evenly there.
    let max_size = geometry.parameters(part).max_size;
    let asked = (extrude.layers.levels(max_size, top - base))
        .map_err(|reason| GmshError::Other(format!("{part}: {reason}")))?;
    let mut levels = Vec::new();
    let mut layer_interval = Vec::new();
    for (interval, bounds) in heights.windows(2).enumerate() {
        let (a, b) = (
            (bounds[0] - base) / (top - base),
            (bounds[1] - base) / (top - base),
        );
        let mut n = (asked.iter())
            .filter(|&&level| level > a + 1e-9 && level <= b + 1e-9)
            .count()
            .max(1);
        // Local sizes on walls and on edges that rise through the interval.
        let height = bounds[1] - bounds[0];
        for &(low, high, size) in &layer_sizes {
            if size > 0.0 && low < bounds[1] - tolerance && high > bounds[0] + tolerance {
                n = n.max((height / size).ceil().min(100_000.0) as usize);
            }
        }
        for k in 1..=n {
            levels.push(a + (b - a) * k as f64 / n as f64);
            layer_interval.push(interval);
        }
    }

    // Where the part has material above each cell: asked of the solid at a point above the
    // middle of one of the cell's elements, in the middle of each interval.
    let mut filled: BTreeSet<(i32, usize)> = BTreeSet::new();
    let mut expected = 0.0;
    for &cell in &cells {
        let own: Vec<_> = elements.iter().filter(|e| e.face == cell).collect();
        let Some(element) = own.get(own.len() / 2) else {
            return refuse(format!("an end piece got no elements (surface {cell})"));
        };
        let mut probe = [0.0; 3];
        for corner in &element.corners {
            probe = add(probe, coords.get(corner).copied().unwrap_or_default());
        }
        let probe = scale(probe, 1.0 / element.corners.len() as f64);
        let area = gmsh.mass(2, cell)?;
        for (interval, bounds) in heights.windows(2).enumerate() {
            let middle = 0.5 * (bounds[0] + bounds[1]) - base;
            if gmsh.contains_point(3, volume, add(probe, scale(stack.direction, middle)))? {
                filled.insert((cell, interval));
                expected += area * (bounds[1] - bounds[0]);
            }
        }
    }
    // The check of the whole: the cells times their heights are the part's volume.
    if (expected - part_volume).abs() > 1e-3 * part_volume.abs().max(f64::MIN_POSITIVE) {
        return refuse(format!(
            "its volume is {part_volume:.6e}, that of the extrusions found {expected:.6e}"
        ));
    }

    let (mut mesh, _) = sweep_mesh(
        part,
        &coords,
        &elements,
        motion,
        &levels,
        quadratic,
        &|layer, element| filled.contains(&(element.face, layer_interval[layer])),
    )?;
    mesh.cad = revolve::classify(gmsh, &mesh, &topology)?;
    let mut warnings = gmsh.warnings()?;
    warnings.extend(notes.into_iter().map(|note| format!("{part}: {note}")));
    Ok(Ok(GeneratedMesh { mesh, warnings }))
}
