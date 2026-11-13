//! Splits of the geometry with OpenCASCADE through Gmsh: parts and faces cut along a plane,
//! and planar faces cut around holes into a ring and the rest, see
//! [`plx_model::CadOperationKind::SplitPlane`] and [`plx_model::CadOperationKind::HoleRing`].

use std::collections::{BTreeMap, BTreeSet};

use plx_mesh::CadEntity;
use plx_model::{CadOperation, CompoundPart, Geometry, SplitKeep, SplitPlane};

use crate::gmsh::{Boolean, Gmsh, GmshError};
use crate::operation::{distance, outward_normal, plane_normal};
use crate::{Shape, default_parts, parts, unique_name};

/// A ring around a hole: its centre, outer radius and normal.
type Ring = ([f64; 3], f64, [f64; 3]);

/// A Gmsh entity as (dimension, tag).
type Entity = (i32, i32);

/// What a split gives on: the names of the parts as Gmsh will read the result, keyed by a
/// point of each of their entities, and the compounds.
pub(crate) struct Split {
    /// (dimension, centre of mass, part name) of every entity of a part.
    pub(crate) places: Vec<(i32, [f64; 3], String)>,
    pub(crate) compounds: Vec<CompoundPart>,
}

fn fail(message: impl Into<String>) -> GmshError {
    GmshError::Other(message.into())
}

/// Splits the parts the plane of `operation` runs through, as read into `gmsh` from `input`,
/// among the parts of the operation if it names any; `entities` holds the picked face of a
/// plane of a face.
pub(crate) fn split_by_plane(
    gmsh: &Gmsh,
    input: &Geometry,
    operation: &CadOperation,
    entities: &[CadEntity],
) -> Result<Split, GmshError> {
    let (point, normal) = cut_plane(gmsh, operation, entities)?;
    let parts = parts(gmsh, input)?;
    // Only the chosen parts are cut, or all; never the suppressed ones.
    let named = |name: &str| {
        operation.parts.is_empty() || (operation.parts.iter()).any(|p| p.eq_ignore_ascii_case(name))
    };
    let chosen = |name: &str| named(name) && !input.is_suppressed(name);
    if !parts.iter().any(|(_, name)| chosen(name)) {
        return Err(fail(if parts.iter().any(|(_, name)| named(name)) {
            "The parts to split are all suppressed"
        } else {
            "None of the parts to split is in the geometry"
        }));
    }
    let (lo, hi) = gmsh.bounding_box()?;
    let size = distance(lo, hi).max(1e-9);
    let tolerance = 1e-7 * size;
    let side = |p: [f64; 3]| (0..3).map(|k| (p[k] - point[k]) * normal[k]).sum::<f64>();
    let keeps = |s: f64| match operation.keep {
        SplitKeep::Both => true,
        SplitKeep::Positive => s > 0.0,
        SplitKeep::Negative => s < 0.0,
    };
    // A box filling the positive side over the whole geometry, and a face in the plane.
    let centre = [0, 1, 2].map(|k| 0.5 * (lo[k] + hi[k]));
    let foot = {
        let s = side(centre);
        [0, 1, 2].map(|k| centre[k] - s * normal[k])
    };
    let extent = 2.0 * size;
    let frame = plane_frame(normal, foot);
    let mut compounds = input.compounds.clone();
    let mut used: BTreeSet<String> = parts.iter().map(|(_, n)| n.clone()).collect();
    let mut places = Vec::new();
    let mut cut_any = false;
    let mut half = None;
    let mut edges: Vec<(Shape, String)> = Vec::new();
    let mut gone: Vec<Entity> = Vec::new();
    for (shape, name) in parts {
        if !chosen(&name) {
            record(gmsh, &shape, &name, &mut places)?;
            continue;
        }
        // Where the part lies: on one side, or across.
        let (min, max) = shape.bounding_box(gmsh)?;
        let corners =
            (0..8).map(|i| [0, 1, 2].map(|k| if i >> k & 1 == 0 { min[k] } else { max[k] }));
        let sides: Vec<f64> = corners.map(side).collect();
        let above = sides.iter().all(|&s| s >= -tolerance);
        let below = sides.iter().all(|&s| s <= tolerance);
        if above || below {
            let s = if above && below {
                0.0
            } else if above {
                1.0
            } else {
                -1.0
            };
            if s == 0.0 || keeps(s) {
                record(gmsh, &shape, &name, &mut places)?;
            } else {
                gone.extend(shape.entities());
                used.remove(&name);
                remove_from(&mut compounds, &name, &[]);
            }
            continue;
        }
        if shape.dim == 1 {
            edges.push((shape, name));
            continue;
        }
        let half = match half {
            Some(half) => half,
            None => {
                let tag = gmsh.add_box(
                    [-extent, -extent, 0.0],
                    [2.0 * extent, 2.0 * extent, extent],
                )?;
                gmsh.place(&[(3, tag)], &frame)?;
                half = Some(tag);
                tag
            }
        };
        // Each side made on its own, so that the pieces share nothing: a solid or a shell
        // cut in two becomes two parts.
        let objects: Vec<Entity> = shape.entities().collect();
        let positive = gmsh
            .boolean_keeping(Boolean::Intersect, &objects, &[(3, half)], false, false)
            .map_err(|e| fail(format!("OpenCASCADE could not split {name}: {e}")))?;
        let negative = gmsh
            .boolean_keeping(Boolean::Cut, &objects, &[(3, half)], false, false)
            .map_err(|e| fail(format!("OpenCASCADE could not split {name}: {e}")))?;
        let of_part = |made: &[Entity]| -> Vec<i32> {
            (made.iter())
                .filter(|e| e.0 == shape.dim)
                .map(|e| e.1)
                .collect()
        };
        let (positive, negative) = (of_part(&positive), of_part(&negative));
        // A part whose bounding box only reaches over the plane, e.g. a part with a face
        // cut along a plane before, lies on one side: OpenCASCADE gives it back whole,
        // under its own tags or new ones, and nothing on the other side.
        if positive.is_empty() || negative.is_empty() {
            let copies: Vec<Entity> = (positive.iter().chain(&negative))
                .filter(|t| !shape.tags.contains(t))
                .map(|&t| (shape.dim, t))
                .collect();
            if !copies.is_empty() {
                gmsh.remove(&copies)?;
            }
            if keeps(if negative.is_empty() { 1.0 } else { -1.0 }) {
                record(gmsh, &shape, &name, &mut places)?;
            } else {
                gone.extend(shape.entities());
                used.remove(&name);
                remove_from(&mut compounds, &name, &[]);
            }
            continue;
        }
        cut_any = true;
        // The originals, but for those given back unchanged under their own tags.
        let originals: Vec<Entity> = (shape.tags.iter())
            .filter(|t| !positive.contains(t) && !negative.contains(t))
            .map(|&t| (shape.dim, t))
            .collect();
        if !originals.is_empty() {
            gmsh.remove(&originals)?;
        }
        // The pieces: each solid, or each set of faces hanging together.
        let pieces_of = |tags: Vec<i32>| -> Result<Vec<Vec<i32>>, GmshError> {
            if shape.dim == 3 {
                Ok(tags.into_iter().map(|t| vec![t]).collect())
            } else {
                crate::shells(gmsh, &tags)
            }
        };
        let mut kept: Vec<Vec<i32>> = Vec::new();
        for (tags, sign) in [(positive, 1.0), (negative, -1.0)] {
            if keeps(sign) {
                kept.extend(pieces_of(tags)?);
            } else if !tags.is_empty() {
                gmsh.remove(&tags.iter().map(|&t| (shape.dim, t)).collect::<Vec<_>>())?;
            }
        }
        used.remove(&name);
        let names: Vec<String> = if kept.len() == 1 {
            vec![name.clone()]
        } else {
            (1..=kept.len())
                .map(|k| {
                    let piece = unique_name(&format!("{name}-{k}"), &used);
                    used.insert(piece.clone());
                    piece
                })
                .collect()
        };
        used.extend(names.iter().cloned());
        for (piece, name) in kept.iter().zip(&names) {
            for &tag in piece {
                places.push((
                    shape.dim,
                    gmsh.center_of_mass(shape.dim, tag)?,
                    name.clone(),
                ));
            }
        }
        if names.len() > 1 && operation.compound && input.compound_of(&name).is_none() {
            compounds.push(CompoundPart {
                name: unique_name(&name, &used),
                parts: names,
            });
        } else {
            remove_from(&mut compounds, &name, &names);
        }
    }
    if let Some(half) = half {
        gmsh.remove(&[(3, half)])?;
    }
    if !gone.is_empty() {
        gmsh.remove(&gone)?;
    }
    // Edges: cut by a face in the plane, the pieces sharing the cut points, so that a wire
    // stays one part.
    if !edges.is_empty() {
        let objects: Vec<Entity> = (edges.iter())
            .flat_map(|(shape, _)| shape.entities())
            .collect();
        let sheet = gmsh.add_rectangle([-extent, -extent, 0.0], [2.0 * extent; 2], 0.0)?;
        gmsh.place(&[(2, sheet)], &frame)?;
        let pieces = fragment_keeping(gmsh, &objects, &[(2, sheet)])
            .map_err(|e| fail(format!("OpenCASCADE could not split the edges: {e}")))?;
        let mut pieces = pieces.into_iter();
        for (shape, name) in &edges {
            let made: Vec<Entity> = pieces.by_ref().take(shape.tags.len()).flatten().collect();
            if made.len() > shape.tags.len() {
                cut_any = true;
            }
            let mut dropped = Vec::new();
            for (dim, tag) in made {
                let place = gmsh.center_of_mass(dim, tag)?;
                if keeps(side(place)) {
                    places.push((dim, place, name.clone()));
                } else {
                    dropped.push((dim, tag));
                }
            }
            if !dropped.is_empty() {
                gmsh.remove(&dropped)?;
            }
        }
    }
    if !cut_any && operation.keep == SplitKeep::Both {
        return Err(fail(if operation.parts.is_empty() {
            "The plane runs through no part"
        } else {
            "The plane runs through none of the parts to split"
        }));
    }
    if places.is_empty() {
        return Err(fail("Nothing is left on the side kept"));
    }
    Ok(Split { places, compounds })
}

/// Splits the planar faces around the picked circular edges of holes into a ring of the
/// width of `operation` and the rest, the solids and shells keeping all their other faces.
pub(crate) fn split_around_holes(
    gmsh: &Gmsh,
    input: &Geometry,
    operation: &CadOperation,
    entities: &[CadEntity],
) -> Result<Split, GmshError> {
    let width = operation.value;
    if width <= 0.0 {
        return Err(fail("The width of the ring must be greater than 0"));
    }
    let parts = parts(gmsh, input)?;
    let owner_of_face = |face: i32| -> Result<Option<usize>, GmshError> {
        let volumes = gmsh.adjacencies(2, face)?.0;
        Ok(parts.iter().position(|(shape, _)| match shape.dim {
            3 => volumes.contains(&shape.tags[0]),
            2 => shape.tags.contains(&face),
            _ => false,
        }))
    };
    // The rings by the part they are on, as (centre, outer radius, normal).
    let mut rings: BTreeMap<usize, Vec<Ring>> = BTreeMap::new();
    for entity in entities {
        let CadEntity::Edge(edge) = *entity else {
            return Err(fail("Split around Hole picks the circular edges of holes"));
        };
        if !gmsh.entities(1)?.contains(&edge) {
            return Err(fail(format!("The geometry has no edge {edge}")));
        }
        // Circles may come as circles, ellipses of equal axes or splines: any edge whose
        // points lie on the circle through three of them.
        let round = || fail("Split around Hole needs circular edges of holes");
        let (centre, radius, normal) = circle(gmsh, edge).map_err(|_| round())?;
        let (min, max) = gmsh.parameter_bounds(1, edge)?;
        let along: Vec<f64> = (0..=8)
            .map(|k| min[0] + f64::from(k) / 8.0 * (max[0] - min[0]))
            .collect();
        let off = |p: [f64; 3]| {
            let radial = (distance(p, centre) - radius).abs();
            let height: f64 = (0..3).map(|k| (p[k] - centre[k]) * normal[k]).sum();
            radial.max(height.abs())
        };
        if gmsh
            .values(1, edge, &along)?
            .into_iter()
            .any(|p| off(p) > 1e-5 * radius)
        {
            return Err(round());
        }
        let mut face = None;
        for f in gmsh.adjacencies(1, edge)?.0 {
            if gmsh.entity_type(2, f)? != "Plane" {
                continue;
            }
            let n = plane_normal(gmsh, f)?;
            let along: f64 = (0..3).map(|k| n[k] * normal[k]).sum();
            if along.abs() > 0.999 {
                face = Some(f);
                break;
            }
        }
        let face = face.ok_or_else(|| {
            fail("A picked hole edge bounds no planar face at right angles to the hole")
        })?;
        let owner = owner_of_face(face)?
            .ok_or_else(|| fail("A picked hole edge lies on no part of the geometry"))?;
        let list = rings.entry(owner).or_default();
        let outer = radius + width;
        let same = list.iter().any(|(c, r, _)| {
            distance(*c, centre) < 1e-6 * radius && (r - outer).abs() < 1e-6 * outer
        });
        if !same {
            list.push((centre, outer, normal));
        }
    }
    if rings.is_empty() {
        return Err(fail("No hole edges picked"));
    }
    let mut places = Vec::new();
    for (index, (shape, name)) in parts.iter().enumerate() {
        let Some(list) = rings.get(&index) else {
            record(gmsh, shape, name, &mut places)?;
            continue;
        };
        let objects: Vec<Entity> = shape.entities().collect();
        let mut disks = Vec::with_capacity(list.len());
        for &(centre, outer, normal) in list {
            disks.push((2, gmsh.add_disk(centre, outer, normal)?));
        }
        let pieces = fragment_keeping(gmsh, &objects, &disks).map_err(|e| {
            fail(format!(
                "OpenCASCADE could not split the faces of {name}: {e}"
            ))
        })?;
        let made: Vec<Entity> = pieces.into_iter().flatten().collect();
        if made.len() != objects.len() && shape.dim == 3 {
            return Err(fail(format!(
                "Splitting the faces of {name} gave no single solid"
            )));
        }
        for (dim, tag) in made {
            places.push((dim, gmsh.center_of_mass(dim, tag)?, name.clone()));
        }
    }
    Ok(Split {
        places,
        compounds: input.compounds.clone(),
    })
}

/// Fragments the objects with the tools and removes what the tools alone gave, beyond the
/// objects and their boundaries, with the edges and points it leaves loose; returns what
/// each object became.
fn fragment_keeping(
    gmsh: &Gmsh,
    objects: &[Entity],
    tools: &[Entity],
) -> Result<Vec<Vec<Entity>>, GmshError> {
    let loose_before = loose(gmsh)?;
    let (_, map) = gmsh.fragment(objects, tools)?;
    let count = objects.len().min(map.len());
    let made = map[..count].to_vec();
    // The objects' results with everything bounding them.
    let mut closure: BTreeSet<Entity> = BTreeSet::new();
    let mut stack: Vec<Entity> = made.iter().flatten().copied().collect();
    while let Some(entity) = stack.pop() {
        if closure.insert(entity) && entity.0 > 0 {
            let down = gmsh.adjacencies(entity.0, entity.1)?.1;
            stack.extend(down.into_iter().map(|t| (entity.0 - 1, t)));
        }
    }
    let tool_only: BTreeSet<Entity> = (map.iter().skip(count).flatten())
        .filter(|e| !closure.contains(e))
        .copied()
        .collect();
    if !tool_only.is_empty() {
        gmsh.remove(&tool_only.into_iter().collect::<Vec<_>>())?;
    }
    let left: Vec<Entity> = (loose(gmsh)?.into_iter())
        .filter(|e| !loose_before.contains(e) && !closure.contains(e))
        .collect();
    if !left.is_empty() {
        gmsh.remove_only(&left)?;
    }
    Ok(made)
}

/// Edges that bound no face and points that bound no edge.
fn loose(gmsh: &Gmsh) -> Result<BTreeSet<Entity>, GmshError> {
    let mut bounding = BTreeSet::new();
    for dim in [2, 1] {
        for tag in gmsh.entities(dim)? {
            bounding.extend(
                gmsh.adjacencies(dim, tag)?
                    .1
                    .into_iter()
                    .map(|t| (dim - 1, t)),
            );
        }
    }
    let mut loose = BTreeSet::new();
    for dim in [1, 0] {
        for tag in gmsh.entities(dim)? {
            if !bounding.contains(&(dim, tag)) {
                loose.insert((dim, tag));
            }
        }
    }
    Ok(loose)
}

/// Records where each entity of a part lies, with the part's name.
fn record(
    gmsh: &Gmsh,
    shape: &Shape,
    name: &str,
    places: &mut Vec<(i32, [f64; 3], String)>,
) -> Result<(), GmshError> {
    for (dim, tag) in shape.entities() {
        places.push((dim, gmsh.center_of_mass(dim, tag)?, name.to_owned()));
    }
    Ok(())
}

/// Puts the pieces of a split part in its place in the compound holding it.
fn remove_from(compounds: &mut Vec<CompoundPart>, part: &str, pieces: &[String]) {
    for compound in compounds.iter_mut() {
        if let Some(at) = compound
            .parts
            .iter()
            .position(|p| p.eq_ignore_ascii_case(part))
        {
            compound.parts.splice(at..=at, pieces.iter().cloned());
        }
    }
    compounds.retain(|c| !c.parts.is_empty());
}

/// The names of the parts Gmsh read from the result, by where their entities lie, as
/// recorded by the split.
pub(crate) fn names_after(
    gmsh: &Gmsh,
    places: &[(i32, [f64; 3], String)],
) -> Result<Vec<String>, GmshError> {
    let mut names = Vec::new();
    for (shape, default) in default_parts(gmsh)? {
        let (dim, tag) = shape.first();
        let at = gmsh.center_of_mass(dim, tag)?;
        let nearest = (places.iter())
            .filter(|(d, _, _)| *d == dim)
            .min_by(|a, b| distance(a.1, at).total_cmp(&distance(b.1, at)))
            .map(|(_, _, n)| n.clone());
        names.push(nearest.unwrap_or(default));
    }
    Ok(names)
}

/// The plane to cut along as a point on it and its unit normal.
fn cut_plane(
    gmsh: &Gmsh,
    operation: &CadOperation,
    entities: &[CadEntity],
) -> Result<([f64; 3], [f64; 3]), GmshError> {
    let offset = operation.value;
    if let Some(axis) = operation.plane.axis() {
        return Ok((axis.map(|a| a * offset), axis));
    }
    debug_assert_eq!(operation.plane, SplitPlane::Face);
    let faces: Vec<i32> = (entities.iter())
        .filter_map(|e| match e {
            CadEntity::Face(tag) => Some(*tag),
            _ => None,
        })
        .collect();
    let [face] = faces[..] else {
        return Err(fail(
            "Split by Plane needs one picked planar face for its plane",
        ));
    };
    if !gmsh.entities(2)?.contains(&face) {
        return Err(fail(format!("The geometry has no face {face}")));
    }
    if gmsh.entity_type(2, face)? != "Plane" {
        return Err(fail(
            "The plane of a split is taken from a planar face only",
        ));
    }
    // Into the solid the face bounds, else the face's own normal.
    let normal = match gmsh.adjacencies(2, face)?.0[..] {
        [volume] => outward_normal(gmsh, face, volume)?.map(|x| -x),
        _ => plane_normal(gmsh, face)?,
    };
    let centre = gmsh.center_of_mass(2, face)?;
    let on = gmsh.closest_point(2, face, centre)?;
    Ok(([0, 1, 2].map(|k| on[k] + offset * normal[k]), normal))
}

/// The affine map, the rows of a 3 x 4 matrix, that takes the global xy plane to the plane
/// through `origin` normal to `normal`, z to the normal.
fn plane_frame(normal: [f64; 3], origin: [f64; 3]) -> [f64; 12] {
    let n = normal;
    // Any direction not along the normal, made normal to it.
    let helper = if n[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let unit = |a: [f64; 3]| {
        let l = (a.iter().map(|x| x * x).sum::<f64>()).sqrt();
        a.map(|x| x / l)
    };
    let u = unit(cross(helper, n));
    let v = cross(n, u);
    [
        u[0], v[0], n[0], origin[0], //
        u[1], v[1], n[1], origin[1], //
        u[2], v[2], n[2], origin[2],
    ]
}

/// The centre, radius and unit normal of a circular edge, a whole circle or an arc, from
/// three of its points.
fn circle(gmsh: &Gmsh, edge: i32) -> Result<([f64; 3], f64, [f64; 3]), GmshError> {
    let (min, max) = gmsh.parameter_bounds(1, edge)?;
    let (a, b) = (min[0], max[0]);
    let points = gmsh.values(1, edge, &[a, a + (b - a) / 3.0, a + 2.0 * (b - a) / 3.0])?;
    let [p, q, r] = points[..] else {
        return Err(fail(format!("Edge {edge} has no points")));
    };
    let sub = |a: [f64; 3], b: [f64; 3]| [0, 1, 2].map(|k| a[k] - b[k]);
    let dot = |a: [f64; 3], b: [f64; 3]| (0..3).map(|k| a[k] * b[k]).sum::<f64>();
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let (u, v) = (sub(q, p), sub(r, p));
    let w = cross(u, v);
    let ww = dot(w, w);
    if ww <= 0.0 || !ww.is_finite() {
        return Err(fail(format!("Edge {edge} is no circle")));
    }
    // The circumcentre of the three points.
    let (uu, vv) = (dot(u, u), dot(v, v));
    let a_part = cross(w, u).map(|x| x * vv);
    let b_part = cross(v, w).map(|x| x * uu);
    let centre = [0, 1, 2].map(|k| p[k] + (a_part[k] + b_part[k]) / (2.0 * ww));
    let radius = distance(centre, p);
    let length = ww.sqrt();
    Ok((centre, radius, w.map(|x| x / length)))
}
