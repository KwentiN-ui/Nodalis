//! Defeaturing with OpenCASCADE through Gmsh: fillets and chamfers removed from solids, the
//! faces next to them grown until they meet again, see
//! [`plx_model::CadOperationKind::Defeature`]. The faces are picked, or found by their shape:
//!
//! - A fillet is a curved face whose larger principal curvature is a radius up to the size,
//!   which ends on two or more faces it touches tangentially, across the direction it is
//!   curved in, and which turns by less than a half circle between them. So the half
//!   cylinders at the ends of a slot and the pieces of a hole stay: the faces a piece of a
//!   hole ends on are pieces of the same cylinder.
//! - A chamfer is a strip, planar or conical, of a width (its area over its longest edge) up to
//!   the size, whose long edges meet the faces next to it at an angle between 10 and 80
//!   degrees, so that steps and ribs at right angles stay.

use std::collections::BTreeSet;

use plx_mesh::CadEntity;
use plx_model::{CadOperation, DefeatureFaces, EntitySignature, Geometry, match_entities};

use crate::gmsh::{Gmsh, GmshError, with_gmsh};
use crate::operation::{named_places, names_by_place, total_mass};
use crate::{TempFile, parts, signatures};

/// The result of a defeaturing, with how many faces it removed.
#[derive(Clone, Debug)]
pub struct Defeatured {
    pub geometry: Geometry,
    /// The faces removed.
    pub removed: usize,
    /// The faces found that OpenCASCADE could not remove, which stay as they were.
    pub kept: usize,
}

/// The geometry with the faces removed: those among `entities` (of `input` as Gmsh reads
/// its BREP) when the operation removes picked faces, else the fillets and chamfers found up
/// to its size. The parts keep their names; the faces and edges get Gmsh's new tags, with
/// their signatures. Found faces that OpenCASCADE cannot remove together are removed in
/// groups of faces next to each other, and the groups it cannot remove stay; picked faces
/// are removed all or the operation fails with OpenCASCADE's reason.
pub fn defeature(
    input: &Geometry,
    operation: &CadOperation,
    entities: &[CadEntity],
) -> Result<Defeatured, GmshError> {
    let fail = |message: String| GmshError::Other(message);
    if let Some(problem) = operation.problem() {
        return Err(fail(problem));
    }
    let found = operation.faces == DefeatureFaces::Found;
    let unchanged = || Defeatured {
        geometry: input.clone(),
        removed: 0,
        kept: 0,
    };
    // A size of zero finds nothing, so that a parameter study can switch it off.
    if found && operation.value == 0.0 {
        return Ok(unchanged());
    }
    let file = TempFile::with_contents("brep", &input.brep)?;
    let result = TempFile::new("brep");
    let checkpoint = TempFile::new("brep");
    let outcome = with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        if gmsh.entities(3)?.is_empty() {
            return Err(fail(
                "Defeaturing removes faces of solids; the geometry has none".into(),
            ));
        }
        let faces = if found {
            blend_faces(gmsh, operation.value)?
        } else {
            picked_faces(gmsh, entities)?
        };
        if faces.is_empty() {
            return Ok(None);
        }
        let before = named_places(gmsh, &parts(gmsh, input)?)?;
        let solids = gmsh.entities(3)?.len();
        let volume = total_mass(gmsh, 3)?;
        let groups: Vec<Vec<EntitySignature>> = {
            let all = signatures(gmsh)?;
            (neighbour_groups(gmsh, &faces)?.into_iter())
                .map(|group| {
                    (all.iter())
                        .filter(|s| group.iter().any(|&f| s.entity == CadEntity::Face(f)))
                        .cloned()
                        .collect()
                })
                .collect()
        };
        gmsh.write(&checkpoint.0)?;
        let attempt = |faces: &[i32]| -> Result<(), String> {
            let removing: Vec<EntitySignature> = (signatures(gmsh).map_err(|e| e.to_string())?)
                .into_iter()
                .filter(|s| faces.iter().any(|&f| s.entity == CadEntity::Face(f)))
                .collect();
            let mut volumes = BTreeSet::new();
            for &face in faces {
                volumes.extend(gmsh.adjacencies(2, face).map_err(|e| e.to_string())?.0);
            }
            let volumes: Vec<i32> = volumes.into_iter().collect();
            gmsh.defeature(&volumes, faces).map_err(|e| e.to_string())?;
            // OpenCASCADE may give a broken solid instead of failing.
            let after = total_mass(gmsh, 3).map_err(|e| e.to_string())?;
            let count = gmsh.entities(3).map_err(|e| e.to_string())?.len();
            if count != solids || !(after.is_finite() && after > 0.0) {
                return Err("the result is no valid solid".into());
            }
            if (after - volume).abs() > 0.5 * volume {
                return Err("the volume changed by more than half".into());
            }
            // It may also give the solid back as it was, with a warning only.
            let now = signatures(gmsh).map_err(|e| e.to_string())?;
            if removing
                .iter()
                .any(|old| now.iter().any(|new| same_face(old, new)))
            {
                return Err("it left the faces as they were".into());
            }
            Ok(())
        };
        let restore = || -> Result<(), GmshError> {
            gmsh.clear()?;
            gmsh.import_shapes(&checkpoint.0)?;
            Ok(())
        };
        let (mut removed, mut kept) = (0, 0);
        match attempt(&faces) {
            Ok(()) => removed = faces.len(),
            Err(why) if !found => {
                return Err(fail(format!(
                    "OpenCASCADE could not remove the faces: {why}. Pick all faces of a \
                     fillet or chamfer that runs over several faces"
                )));
            }
            Err(_) => {
                // Group by group, each found again by its signatures in what the groups
                // before it left.
                restore()?;
                for group in &groups {
                    let now = signatures(gmsh)?;
                    let map = match_entities(group, &now);
                    let tags: Vec<i32> = (group.iter())
                        .filter_map(|s| match map.get(&s.entity) {
                            Some(CadEntity::Face(tag)) => Some(*tag),
                            _ => None,
                        })
                        .collect();
                    if tags.len() != group.len() {
                        kept += group.len();
                        continue;
                    }
                    match attempt(&tags) {
                        Ok(()) => {
                            removed += tags.len();
                            gmsh.write(&checkpoint.0)?;
                        }
                        Err(_) => {
                            kept += tags.len();
                            restore()?;
                        }
                    }
                }
                if removed == 0 {
                    return Err(fail(format!(
                        "OpenCASCADE could not remove any of the {} fillet and chamfer faces \
                         found",
                        faces.len()
                    )));
                }
            }
        }
        gmsh.write(&result.0)?;
        gmsh.clear()?;
        gmsh.import_shapes(&result.0)?;
        Ok(Some((
            names_by_place(gmsh, &before)?,
            signatures(gmsh)?,
            removed,
            kept,
        )))
    })?;
    let Some((part_names, signatures, removed, kept)) = outcome else {
        return Ok(unchanged());
    };
    let brep = std::fs::read_to_string(&result.0)
        .map_err(|e| fail(format!("{}: {e}", result.0.display())))?;
    Ok(Defeatured {
        geometry: Geometry {
            brep,
            part_names,
            signatures,
            ..input.clone()
        },
        removed,
        kept,
    })
}

/// The fillet and chamfer faces of the solids of a geometry up to a size, see the module;
/// their tags are those of `input` as Gmsh reads its BREP.
pub fn find_blends(input: &Geometry, max_size: f64) -> Result<Vec<CadEntity>, GmshError> {
    if max_size.is_nan() || max_size <= 0.0 {
        return Ok(Vec::new());
    }
    let file = TempFile::with_contents("brep", &input.brep)?;
    with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        Ok((blend_faces(gmsh, max_size)?.into_iter())
            .map(CadEntity::Face)
            .collect())
    })
}

/// Whether two signatures are the same face, unchanged.
fn same_face(a: &EntitySignature, b: &EntitySignature) -> bool {
    let scale = a.size.abs().sqrt().max(1e-12);
    a.entity.dim() == b.entity.dim()
        && a.kind == b.kind
        && (a.size - b.size).abs() <= 1e-6 * a.size.abs()
        && (0..3).all(|k| (a.centroid[k] - b.centroid[k]).abs() <= 1e-6 * scale)
}

/// The picked faces, which must bound solids.
fn picked_faces(gmsh: &Gmsh, entities: &[CadEntity]) -> Result<Vec<i32>, GmshError> {
    let fail = |message: String| GmshError::Other(message);
    let known: BTreeSet<i32> = gmsh.entities(2)?.into_iter().collect();
    let mut faces = Vec::new();
    for entity in entities {
        let CadEntity::Face(face) = *entity else {
            continue;
        };
        if !known.contains(&face) {
            return Err(fail(format!("The geometry has no face {face}")));
        }
        if gmsh.adjacencies(2, face)?.0.is_empty() {
            return Err(fail(
                "Defeaturing removes faces of solids, not of shells".into(),
            ));
        }
        faces.push(face);
    }
    if faces.is_empty() {
        return Err(fail("No faces picked".into()));
    }
    Ok(faces)
}

/// The faces in groups of faces next to each other, sharing edges.
fn neighbour_groups(gmsh: &Gmsh, faces: &[i32]) -> Result<Vec<Vec<i32>>, GmshError> {
    let mut edges = Vec::with_capacity(faces.len());
    for &face in faces {
        let bounding: BTreeSet<i32> = gmsh
            .adjacencies(2, face)?
            .1
            .iter()
            .map(|e| e.abs())
            .collect();
        edges.push(bounding);
    }
    let mut group_of: Vec<Option<usize>> = vec![None; faces.len()];
    let mut groups: Vec<Vec<i32>> = Vec::new();
    for start in 0..faces.len() {
        if group_of[start].is_some() {
            continue;
        }
        let index = groups.len();
        let mut group = Vec::new();
        let mut open = vec![start];
        group_of[start] = Some(index);
        while let Some(at) = open.pop() {
            group.push(faces[at]);
            for other in 0..faces.len() {
                if group_of[other].is_none() && !edges[at].is_disjoint(&edges[other]) {
                    group_of[other] = Some(index);
                    open.push(other);
                }
            }
        }
        groups.push(group);
    }
    Ok(groups)
}

/// The fillet and chamfer faces of the solids up to a size, see the module.
fn blend_faces(gmsh: &Gmsh, max_size: f64) -> Result<Vec<i32>, GmshError> {
    let mut found = Vec::new();
    for face in gmsh.entities(2)? {
        let (solids, edges) = gmsh.adjacencies(2, face)?;
        if solids.is_empty() {
            continue;
        }
        let edges: Vec<i32> = edges.iter().map(|e| e.abs()).collect();
        // Chamfers of holes are cones, or splines when the hole was made of an extruded
        // disk.
        let blend = is_chamfer(gmsh, face, &edges, max_size)?
            || (gmsh.entity_type(2, face)? != "Plane" && is_fillet(gmsh, face, &edges, max_size)?);
        if blend {
            found.push(face);
        }
    }
    Ok(found)
}

/// A little more than the size, for sizes typed as the radius of a fillet.
const SIZE_TOLERANCE: f64 = 1.0 + 1e-6;

/// Whether a face is a fillet up to the size, see the module.
fn is_fillet(gmsh: &Gmsh, face: i32, edges: &[i32], max_size: f64) -> Result<bool, GmshError> {
    let centre = gmsh.center_of_mass(2, face)?;
    let Some(here) = surface_at(gmsh, face, centre)? else {
        return Ok(false);
    };
    if here.k_max <= 0.0 || 1.0 / here.k_max > max_size * SIZE_TOLERANCE {
        return Ok(false);
    }
    let tangent = 5f64.to_radians().cos();
    // A sphere's corner of rounded edges is curved alike in all directions: every edge
    // ends it.
    let umbilic = here.k_max - here.k_min < 0.05 * here.k_max;
    // The normals of the face where it ends on faces it touches tangentially.
    let mut ends: Vec<[f64; 3]> = Vec::new();
    for &edge in edges {
        let Some((point, along)) = edge_middle(gmsh, edge)? else {
            continue;
        };
        let Some(own) = surface_at(gmsh, face, point)? else {
            continue;
        };
        // An end of the arc runs across the direction the face is curved in most.
        if !umbilic && dot(along, own.d_max).abs() > 0.3 {
            continue;
        }
        let (neighbours, _) = gmsh.adjacencies(1, edge)?;
        for other in neighbours.into_iter().filter(|&f| f != face) {
            let Some(next) = surface_at(gmsh, other, point)? else {
                continue;
            };
            let smooth = dot(own.normal, next.normal).abs() > tangent;
            let same_surface = (next.k_max - own.k_max).abs() <= 0.05 * own.k_max
                && (next.k_min - own.k_min).abs() <= 0.05 * own.k_max;
            if smooth && !same_surface {
                ends.push(own.normal);
                break;
            }
        }
    }
    if ends.len() < 2 {
        return Ok(false);
    }
    // Less than a half circle between its ends, unlike the round end of a slot.
    let widest = (ends.iter())
        .flat_map(|a| ends.iter().map(move |b| dot(*a, *b)))
        .fold(1.0f64, f64::min);
    Ok(widest > 170f64.to_radians().cos())
}

/// Whether a face is a chamfer up to the size, see the module.
fn is_chamfer(gmsh: &Gmsh, face: i32, edges: &[i32], max_size: f64) -> Result<bool, GmshError> {
    let mut lengths = Vec::with_capacity(edges.len());
    for &edge in edges {
        lengths.push(gmsh.mass(1, edge)?);
    }
    let longest = lengths.iter().copied().fold(0.0f64, f64::max);
    if longest.is_nan() || longest <= 0.0 {
        return Ok(false);
    }
    let width = gmsh.mass(2, face)? / longest;
    if width > max_size * SIZE_TOLERANCE || longest < 2.0 * width {
        return Ok(false);
    }
    let (steep, flat) = (10f64.to_radians().cos(), 80f64.to_radians().cos());
    let mut long = 0;
    for (&edge, &length) in edges.iter().zip(&lengths) {
        if length < 0.5 * longest {
            continue;
        }
        long += 1;
        let Some((point, _)) = edge_middle(gmsh, edge)? else {
            return Ok(false);
        };
        let Some(own) = surface_at(gmsh, face, point)? else {
            return Ok(false);
        };
        let (neighbours, _) = gmsh.adjacencies(1, edge)?;
        let mut bevelled = false;
        for other in neighbours.into_iter().filter(|&f| f != face) {
            if let Some(next) = surface_at(gmsh, other, point)? {
                let cos = dot(own.normal, next.normal).abs();
                bevelled |= cos > flat && cos < steep;
            }
        }
        if !bevelled {
            return Ok(false);
        }
    }
    Ok(long >= 2)
}

/// A surface at a point: its unit normal and principal curvatures.
struct SurfacePoint {
    normal: [f64; 3],
    k_max: f64,
    k_min: f64,
    /// The direction of the larger curvature.
    d_max: [f64; 3],
}

/// The surface at the point of a face nearest to `point`, if Gmsh can tell.
fn surface_at(gmsh: &Gmsh, face: i32, point: [f64; 3]) -> Result<Option<SurfacePoint>, GmshError> {
    let Ok((_, parameters)) = gmsh.projection(2, face, point) else {
        return Ok(None);
    };
    if parameters.len() < 2 {
        return Ok(None);
    }
    let parameters = &parameters[..2];
    let normal = gmsh.normals(face, parameters)?;
    let curvatures = gmsh.principal_curvatures(face, parameters)?;
    match (normal.first(), curvatures.first()) {
        (Some(&normal), Some(&(k_max, k_min, d_max))) if dot(normal, normal) > 0.5 => {
            Ok(Some(SurfacePoint {
                normal,
                k_max,
                k_min,
                d_max: unit(d_max),
            }))
        }
        _ => Ok(None),
    }
}

/// A point of a curve and the unit tangent there.
type EdgePoint = ([f64; 3], [f64; 3]);

/// The point in the middle of a curve and its unit tangent there.
fn edge_middle(gmsh: &Gmsh, edge: i32) -> Result<Option<EdgePoint>, GmshError> {
    let (min, max) = gmsh.parameter_bounds(1, edge)?;
    let (Some(&a), Some(&b)) = (min.first(), max.first()) else {
        return Ok(None);
    };
    let middle = 0.5 * (a + b);
    let step = 1e-3 * (b - a);
    let [point, before, after] = gmsh.values(1, edge, &[middle, middle - step, middle + step])?[..]
    else {
        return Ok(None);
    };
    let along = unit([0, 1, 2].map(|k| after[k] - before[k]));
    Ok((dot(along, along) > 0.5).then_some((point, along)))
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn unit(v: [f64; 3]) -> [f64; 3] {
    let length = dot(v, v).sqrt();
    if length > 0.0 && length.is_finite() {
        v.map(|x| x / length)
    } else {
        [0.0; 3]
    }
}
