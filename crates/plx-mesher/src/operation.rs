//! CAD operations on the geometry with OpenCASCADE through Gmsh: fillets and chamfers of
//! edges and planar faces moved, see [`plx_model::CadOperation`].

use std::collections::BTreeSet;

use plx_mesh::CadEntity;
use plx_model::{CadOperation, CadOperationKind, EntitySignature, Geometry, match_entities};

use crate::gmsh::{Boolean, Gmsh, GmshError, with_gmsh};
use crate::{Shape, TempFile, default_parts, parts, signatures};

/// The geometry with the operation applied to the entities, which are those of `input` as
/// Gmsh reads its BREP; its length is in metres like the geometry. The parts keep their
/// names and the mesh setup stays; the faces and edges get Gmsh's new tags, with their
/// signatures to find them again. On flat geometry, fillets and chamfers act on the corners
/// of faces, picked as vertices. Fails with OpenCASCADE's reason, or when the result is not
/// a valid change of the solids or faces.
pub fn apply_cad_operation(
    input: &Geometry,
    kind: CadOperationKind,
    entities: &[CadEntity],
    value: f64,
) -> Result<Geometry, GmshError> {
    let operation = CadOperation {
        kind,
        value,
        ..CadOperation::default()
    };
    apply_operation(input, &operation, entities)
}

/// [`apply_cad_operation`] with all settings of the operation, those of a split among
/// them; its own picks are not used but `entities`, which are those of `input`.
pub fn apply_operation(
    input: &Geometry,
    operation: &CadOperation,
    entities: &[CadEntity],
) -> Result<Geometry, GmshError> {
    let fail = |message: String| GmshError::Other(message);
    let (kind, value) = (operation.kind, operation.value);
    if let Some(problem) = operation.problem() {
        return Err(fail(capitalised(&problem)));
    }
    if kind == CadOperationKind::Defeature {
        return crate::defeature::defeature(input, operation, entities).map(|d| d.geometry);
    }
    if kind.splits() && !(kind == CadOperationKind::HoleRing && value == 0.0) {
        return apply_split(input, operation, entities);
    }
    // A radius, distance or offset of zero changes nothing: the input is given on as it is,
    // so that a parameter study can switch the operation off.
    if value == 0.0 {
        return Ok(input.clone());
    }
    let file = TempFile::with_contents("brep", &input.brep)?;
    let result = TempFile::new("brep");
    let names = with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        crate::refuse_suppressed(gmsh, input, entities, kind.picked_noun())?;
        let before = named_places(gmsh, &parts(gmsh, input)?)?;
        // Flat geometry has faces as its largest shapes; their areas take the place of the
        // volumes.
        let dim = if gmsh.entities(3)?.is_empty() { 2 } else { 3 };
        let volumes_before = gmsh.entities(dim)?.len();
        let volume_before = total_mass(gmsh, dim)?;
        let corners = entities.iter().any(|e| matches!(e, CadEntity::Vertex(_)));
        let made = match kind {
            CadOperationKind::Fillet | CadOperationKind::Chamfer if corners => {
                round_corners(gmsh, kind, entities, value)?
            }
            CadOperationKind::Fillet | CadOperationKind::Chamfer => {
                round_edges(gmsh, kind, entities, value)?
            }
            CadOperationKind::PushPull => push_pull(gmsh, entities, value)?,
            CadOperationKind::SplitPlane
            | CadOperationKind::HoleRing
            | CadOperationKind::Defeature => {
                unreachable!("splits and defeaturing are applied on their own")
            }
        };
        if made.is_empty() {
            return Err(fail(format!(
                "OpenCASCADE made no {}: the {} may be too large for the faces next to the \
                 {}",
                kind.label().to_lowercase(),
                kind.value_label().to_lowercase(),
                kind.picked_noun()
            )));
        }
        // OpenCASCADE may give a broken solid instead of failing: the result must have as
        // many solids, and a volume a little different from before.
        let volumes_after = gmsh.entities(dim)?.len();
        let volume_after = total_mass(gmsh, dim)?;
        // A face moved may change the volume much; a fillet or a chamfer may not.
        let changed_much = kind != CadOperationKind::PushPull
            && (volume_after - volume_before).abs() > 0.5 * volume_before;
        if volumes_after != volumes_before
            || !(volume_after.is_finite() && volume_after > 0.0)
            || changed_much
        {
            return Err(fail(format!(
                "The {} gave no valid {}: the {} may be too large",
                kind.label().to_lowercase(),
                if dim == 3 { "solid" } else { "face" },
                kind.value_label().to_lowercase()
            )));
        }
        gmsh.write(&result.0)?;
        gmsh.clear()?;
        gmsh.import_shapes(&result.0)?;
        Ok((names_by_place(gmsh, &before)?, signatures(gmsh)?))
    })?;
    let brep = std::fs::read_to_string(&result.0)
        .map_err(|e| fail(format!("{}: {e}", result.0.display())))?;
    let (part_names, signatures) = names;
    Ok(Geometry {
        brep,
        part_names,
        signatures,
        ..input.clone()
    })
}

/// The names of the parts of the model: each keeps the name of the one of `before` it was
/// made from, the nearest of the same dimension.
pub(crate) fn names_by_place(
    gmsh: &Gmsh,
    before: &[(i32, String, [f64; 3])],
) -> Result<Vec<String>, GmshError> {
    let after = default_parts(gmsh)?;
    let mut names = Vec::with_capacity(after.len());
    for (shape, name) in after {
        let place = place_of(gmsh, &shape)?;
        let nearest = (before.iter())
            .filter(|(d, _, _)| *d == shape.dim)
            .min_by(|a, b| distance(a.2, place).total_cmp(&distance(b.2, place)))
            .map(|(_, n, _)| n.clone());
        names.push(nearest.unwrap_or(name));
    }
    Ok(names)
}

/// A split of the geometry, see [`crate::split`]: the parts keep their names, those cut in
/// pieces numbered after them.
fn apply_split(
    input: &Geometry,
    operation: &CadOperation,
    entities: &[CadEntity],
) -> Result<Geometry, GmshError> {
    let file = TempFile::with_contents("brep", &input.brep)?;
    let result = TempFile::new("brep");
    let (part_names, signatures, mut compounds) = with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        let split = match operation.kind {
            CadOperationKind::SplitPlane => {
                crate::split::split_by_plane(gmsh, input, operation, entities)?
            }
            _ => {
                crate::refuse_suppressed(gmsh, input, entities, "hole edges")?;
                crate::split::split_around_holes(gmsh, input, operation, entities)?
            }
        };
        gmsh.write(&result.0)?;
        gmsh.clear()?;
        gmsh.import_shapes(&result.0)?;
        let names = crate::split::names_after(gmsh, &split.places)?;
        Ok((names, signatures(gmsh)?, split.compounds))
    })?;
    for compound in &mut compounds {
        compound.parts.retain(|p| part_names.contains(p));
    }
    compounds.retain(|c| !c.parts.is_empty());
    let brep = std::fs::read_to_string(&result.0)
        .map_err(|e| GmshError::Other(format!("{}: {e}", result.0.display())))?;
    Ok(Geometry {
        brep,
        part_names,
        signatures,
        compounds,
        ..input.clone()
    })
}

/// Rounds or bevels the edges among `entities`; returns the new solids.
fn round_edges(
    gmsh: &Gmsh,
    kind: CadOperationKind,
    entities: &[CadEntity],
    value: f64,
) -> Result<Vec<(i32, i32)>, GmshError> {
    let fail = |message: String| GmshError::Other(message);
    let edges: Vec<i32> = (entities.iter())
        .filter_map(|e| match e {
            CadEntity::Edge(tag) => Some(*tag),
            _ => None,
        })
        .collect();
    let known: BTreeSet<i32> = gmsh.entities(1)?.into_iter().collect();
    if let Some(edge) = edges.iter().find(|e| !known.contains(e)) {
        return Err(fail(format!("The geometry has no edge {edge}")));
    }
    if edges.is_empty() {
        return Err(fail("No edges picked".into()));
    }
    // The solids the edges bound, through their faces.
    let mut volumes = BTreeSet::new();
    let mut faces = Vec::with_capacity(edges.len());
    for &edge in &edges {
        let (up, _) = gmsh.adjacencies(1, edge)?;
        faces.push(up.first().copied().unwrap_or(0));
        for face in up {
            volumes.extend(gmsh.adjacencies(2, face)?.0);
        }
    }
    if volumes.is_empty() {
        return Err(fail(format!(
            "A {} needs edges of solids",
            kind.label().to_lowercase()
        )));
    }
    let volumes: Vec<i32> = volumes.into_iter().collect();
    let made = match kind {
        CadOperationKind::Chamfer => gmsh.chamfer(&volumes, &edges, &faces, value),
        _ => gmsh.fillet(&volumes, &edges, value),
    };
    made.map_err(|e| {
        fail(format!(
            "OpenCASCADE could not make the {}: {e}",
            kind.label().to_lowercase()
        ))
    })
}

/// Rounds or bevels the corners of flat faces at the vertices among `entities`; returns the
/// faces made. OpenCASCADE rounds edges of solids only: each face is extruded to a prism,
/// the prism's edges through the corners are rounded, and its face in the plane of the
/// original is kept in its place.
fn round_corners(
    gmsh: &Gmsh,
    kind: CadOperationKind,
    entities: &[CadEntity],
    value: f64,
) -> Result<Vec<(i32, i32)>, GmshError> {
    let fail = |message: String| GmshError::Other(message);
    let known: BTreeSet<i32> = gmsh.entities(0)?.into_iter().collect();
    // The corners of each face.
    let mut faces: std::collections::BTreeMap<i32, Vec<i32>> = Default::default();
    for entity in entities {
        let CadEntity::Vertex(vertex) = *entity else {
            return Err(fail(format!(
                "A {} picks edges of solids or corners of faces, not both",
                kind.label().to_lowercase()
            )));
        };
        if !known.contains(&vertex) {
            return Err(fail(format!("The geometry has no vertex {vertex}")));
        }
        let (curves, _) = gmsh.adjacencies(0, vertex)?;
        let mut owners = BTreeSet::new();
        for &curve in &curves {
            let (up, _) = gmsh.adjacencies(1, curve)?;
            if up.len() != 1 {
                return Err(fail(
                    "A corner on an edge two faces share cannot be rounded".into(),
                ));
            }
            owners.extend(up);
        }
        match (curves.len(), owners.len()) {
            (2, 1) => {}
            _ => {
                return Err(fail(format!(
                    "A {} needs corners where two edges of one face meet",
                    kind.label().to_lowercase()
                )));
            }
        }
        let [a, b] = [curves[0], curves[1]].map(|curve| leaving(gmsh, curve, vertex));
        let (a, b) = (a?, b?);
        if (0..3).map(|k| a[k] * b[k]).sum::<f64>() < -0.9998 {
            return Err(fail(format!(
                "The edges at a picked corner meet smoothly, so there is no corner to {}",
                kind.label().to_lowercase()
            )));
        }
        faces
            .entry(owners.into_iter().next().unwrap_or_default())
            .or_default()
            .push(vertex);
    }
    let (lo, hi) = gmsh.bounding_box()?;
    let height = distance(lo, hi).max(1e-9);
    let mut made = Vec::new();
    for (face, corners) in faces {
        let normal = plane_normal(gmsh, face)?;
        let plane_point = gmsh.center_of_mass(2, face)?;
        let curves_before: BTreeSet<i32> = gmsh.entities(1)?.into_iter().collect();
        let prism = (gmsh.extrude(&[(2, face)], normal.map(|n| n * height))?)
            .into_iter()
            .find(|&(dim, _)| dim == 3)
            .ok_or_else(|| fail(format!("Face {face} could not be extruded")))?;
        // The prism's edge through each corner and one of its side faces.
        let mut edges = Vec::with_capacity(corners.len());
        let mut sides = Vec::with_capacity(corners.len());
        for &corner in &corners {
            let (up, _) = gmsh.adjacencies(0, corner)?;
            let edge = (up.into_iter())
                .find(|c| !curves_before.contains(c))
                .ok_or_else(|| fail(format!("No edge of the prism at vertex {corner}")))?;
            edges.push(edge);
            sides.push(gmsh.adjacencies(1, edge)?.0.first().copied().unwrap_or(0));
        }
        let rounded = match kind {
            CadOperationKind::Chamfer => gmsh.chamfer(&[prism.1], &edges, &sides, value),
            _ => gmsh.fillet(&[prism.1], &edges, value),
        };
        let rounded = rounded.map_err(|e| {
            fail(format!(
                "OpenCASCADE could not make the {}: {e}",
                kind.label().to_lowercase()
            ))
        })?;
        let [(3, solid)] = rounded[..] else {
            return Err(fail(format!(
                "OpenCASCADE made no {}: the {} may be too large for the edges at the corners",
                kind.label().to_lowercase(),
                kind.value_label().to_lowercase()
            )));
        };
        // The face of the solid in the plane of the original stays; the rest of the solid
        // goes, except what other faces still use.
        let (_, solid_faces) = gmsh.adjacencies(3, solid)?;
        let mut bottom = None;
        for &f in &solid_faces {
            let centre = gmsh.center_of_mass(2, f)?;
            let off: f64 = (0..3)
                .map(|k| (centre[k] - plane_point[k]) * normal[k])
                .sum();
            if off.abs() < 1e-6 * height {
                bottom = Some(f);
            }
        }
        let bottom = bottom.ok_or_else(|| fail("The rounded face was not found".into()))?;
        let closure = |faces: &[i32]| -> Result<BTreeSet<(i32, i32)>, GmshError> {
            let mut all = BTreeSet::new();
            for &f in faces {
                all.insert((2, f));
                for curve in gmsh.adjacencies(2, f)?.1 {
                    all.insert((1, curve));
                    all.extend(gmsh.adjacencies(1, curve)?.1.into_iter().map(|p| (0, p)));
                }
            }
            Ok(all)
        };
        let others: Vec<i32> = (gmsh.entities(2)?.into_iter())
            .filter(|f| !solid_faces.contains(f))
            .collect();
        let mut kept = closure(&[bottom])?;
        kept.extend(closure(&others)?);
        let gone: Vec<(i32, i32)> = (closure(&solid_faces)?.into_iter())
            .filter(|e| !kept.contains(e))
            .collect();
        gmsh.remove_only(&[(3, solid)])?;
        for dim in [2, 1, 0] {
            let these: Vec<(i32, i32)> = gone.iter().copied().filter(|e| e.0 == dim).collect();
            if !these.is_empty() {
                gmsh.remove_only(&these)?;
            }
        }
        made.push((2, bottom));
    }
    // Faces next to a remade one share their edges with it again.
    if gmsh.entities(2)?.len() > 1 {
        gmsh.remove_all_duplicates()?;
    }
    Ok(made)
}

/// The unit normal of a planar face, either way round.
pub(crate) fn plane_normal(gmsh: &Gmsh, face: i32) -> Result<[f64; 3], GmshError> {
    let (min, max) = gmsh.parameter_bounds(2, face)?;
    let [u0, v0, u1, v1] = [min[0], min[1], max[0], max[1]];
    let corners = gmsh.values(2, face, &[u0, v0, u1, v0, u0, v1])?;
    let [a, b, c] = [corners[0], corners[1], corners[2]];
    let (ab, ac) = (
        [0, 1, 2].map(|k| b[k] - a[k]),
        [0, 1, 2].map(|k| c[k] - a[k]),
    );
    let n = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    let length = (n.iter().map(|x| x * x).sum::<f64>()).sqrt();
    if length.is_nan() || length <= 0.0 {
        return Err(GmshError::Other(format!("Face {face} has no normal")));
    }
    Ok(n.map(|x| x / length))
}

/// Moves the planar faces among `entities` along their outward normal by `offset`, inwards
/// when it is negative: a prism on each face is fused with its solid or cut from it, see
/// [`CadOperationKind::PushPull`]. Returns the solids made.
fn push_pull(
    gmsh: &Gmsh,
    entities: &[CadEntity],
    offset: f64,
) -> Result<Vec<(i32, i32)>, GmshError> {
    let fail = |message: String| GmshError::Other(message);
    let faces: Vec<i32> = (entities.iter())
        .filter_map(|e| match e {
            CadEntity::Face(tag) => Some(*tag),
            _ => None,
        })
        .collect();
    if faces.is_empty() {
        return Err(fail("No faces picked".into()));
    }
    let known: BTreeSet<i32> = gmsh.entities(2)?.into_iter().collect();
    if let Some(face) = faces.iter().find(|f| !known.contains(f)) {
        return Err(fail(format!("The geometry has no face {face}")));
    }
    // One face after the other, so that faces next to each other both move as a whole:
    // the faces still to move are found again in each result by their signatures.
    let mut pending: Vec<EntitySignature> = (signatures(gmsh)?.into_iter())
        .filter(|s| faces.iter().any(|&f| s.entity == CadEntity::Face(f)))
        .collect();
    let mut made = Vec::new();
    while !pending.is_empty() {
        let CadEntity::Face(face) = pending.remove(0).entity else {
            continue;
        };
        made = move_face(gmsh, face, offset)?;
        if pending.is_empty() {
            break;
        }
        let now = signatures(gmsh)?;
        let map = match_entities(&pending, &now);
        pending = (pending.iter())
            .map(|s| {
                (map.get(&s.entity))
                    .and_then(|new| now.iter().find(|n| n.entity == *new))
                    .cloned()
                    .ok_or_else(|| fail("A picked face was lost by moving another".into()))
            })
            .collect::<Result<_, _>>()?;
    }
    Ok(made)
}

/// Moves one planar face, see [`push_pull`].
fn move_face(gmsh: &Gmsh, face: i32, offset: f64) -> Result<Vec<(i32, i32)>, GmshError> {
    let fail = |message: String| GmshError::Other(message);
    if gmsh.entity_type(2, face)? != "Plane" {
        return Err(fail("Push/Pull moves planar faces only".into()));
    }
    let (solids, _) = gmsh.adjacencies(2, face)?;
    let [volume] = solids[..] else {
        return Err(fail(
            "Push/Pull needs faces on the outside of one solid".into(),
        ));
    };
    let normal = outward_normal(gmsh, face, volume)?;
    let prisms: Vec<(i32, i32)> = (gmsh.extrude(&[(2, face)], normal.map(|n| n * offset))?)
        .into_iter()
        .filter(|&(dim, _)| dim == 3)
        .collect();
    let operation = if offset > 0.0 {
        Boolean::Fuse
    } else {
        Boolean::Cut
    };
    gmsh.boolean(operation, &[(3, volume)], &prisms, true)
        .map_err(|e| fail(format!("OpenCASCADE could not move the face: {e}")))
}

/// The unit normal of a planar face pointing out of the solid it bounds.
pub(crate) fn outward_normal(gmsh: &Gmsh, face: i32, volume: i32) -> Result<[f64; 3], GmshError> {
    let n = plane_normal(gmsh, face)?;
    // Which side the solid is on, a little off a point of the face.
    let (lo, hi) = gmsh.entity_bounding_box(2, face)?;
    let step = 1e-4 * distance(lo, hi).max(1e-9);
    let centre = gmsh.center_of_mass(2, face)?;
    let point = gmsh.closest_point(2, face, centre)?;
    let probe = |sign: f64| [0, 1, 2].map(|k| point[k] + sign * step * n[k]);
    if gmsh.contains_point(3, volume, probe(1.0))? {
        Ok(n.map(|x| -x))
    } else if gmsh.contains_point(3, volume, probe(-1.0))? {
        Ok(n)
    } else {
        Err(GmshError::Other(format!(
            "The outside of face {face} could not be told"
        )))
    }
}

fn capitalised(text: &str) -> String {
    let mut chars = text.chars();
    chars
        .next()
        .map_or_else(String::new, |c| c.to_uppercase().chain(chars).collect())
}

/// Each part as (dimension, name, centre).
pub(crate) fn named_places(
    gmsh: &Gmsh,
    parts: &[(Shape, String)],
) -> Result<Vec<(i32, String, [f64; 3])>, GmshError> {
    (parts.iter())
        .map(|(shape, name)| Ok((shape.dim, name.clone(), place_of(gmsh, shape)?)))
        .collect()
}

fn place_of(gmsh: &Gmsh, shape: &Shape) -> Result<[f64; 3], GmshError> {
    let (min, max) = shape.bounding_box(gmsh)?;
    Ok([0, 1, 2].map(|k| 0.5 * (min[k] + max[k])))
}

/// The unit direction in which a curve leaves one of its end vertices.
fn leaving(gmsh: &Gmsh, curve: i32, vertex: i32) -> Result<[f64; 3], GmshError> {
    let at = gmsh.point(vertex)?;
    let (min, max) = gmsh.parameter_bounds(1, curve)?;
    let [start, end, near_start, near_end] = gmsh.values(
        1,
        curve,
        &[
            min[0],
            max[0],
            min[0] + 1e-3 * (max[0] - min[0]),
            max[0] - 1e-3 * (max[0] - min[0]),
        ],
    )?[..] else {
        return Err(GmshError::Other(format!("Curve {curve} has no points")));
    };
    let near = if distance(start, at) <= distance(end, at) {
        near_start
    } else {
        near_end
    };
    let length = distance(near, at).max(f64::MIN_POSITIVE);
    Ok([0, 1, 2].map(|k| (near[k] - at[k]) / length))
}

pub(crate) fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|k| (a[k] - b[k]).powi(2)).sum::<f64>().sqrt()
}

/// The volume of the solids, or the area of the faces for `dim` 2.
pub(crate) fn total_mass(gmsh: &Gmsh, dim: i32) -> Result<f64, GmshError> {
    let mut mass = 0.0;
    for tag in gmsh.entities(dim)? {
        mass += gmsh.mass(dim, tag)?;
    }
    Ok(mass)
}
