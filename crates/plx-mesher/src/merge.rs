//! Several geometries put together into one, as the Geometry row of an analysis takes them
//! from several links: the untouched parts of an assembly with a part a CAD operation
//! changed, or the parts of several files.

use std::collections::{BTreeMap, BTreeSet};

use plx_model::{CompoundPart, Geometry, TagMap};

use crate::gmsh::with_gmsh;
use crate::{
    BoundingBox, Entity, GmshError, TempFile, default_parts, part_boxes, parts, same_box,
    signatures, unchanged, unique_name,
};

/// Geometries put together by [`merge_geometries`].
#[derive(Clone, Debug)]
pub struct MergedGeometry {
    pub geometry: Geometry,
    /// For each input, the new tag of each of its faces, edges and vertices by its old one,
    /// for a mesh made of it.
    pub tags: Vec<TagMap>,
    /// For each input, the new names of the parts and compound parts that had to be renamed
    /// since an earlier input has a part of the name.
    pub renamed: Vec<BTreeMap<String, String>>,
}

/// Puts the parts of `inputs` together into one geometry, in their order. Each part keeps
/// its name unless an earlier input has a part of that name: it gets "-2", "-3" … then.
/// The shapes are not joined: parts that touch stay parts of their own, as in an assembly
/// read from a file. The meshing parameters are those of the first input, with no mesh
/// setup items, since those of the inputs name their own faces.
pub fn merge_geometries(inputs: &[Geometry]) -> Result<MergedGeometry, GmshError> {
    if inputs.is_empty() {
        return Err(GmshError::Other("No geometry to put together".into()));
    }
    // Each input on its own: its parts with their names and where they lie, and its
    // entities, to find them again in the whole.
    let mut alone = Vec::with_capacity(inputs.len());
    let mut files = Vec::with_capacity(inputs.len());
    for geometry in inputs {
        let file = TempFile::with_contents("brep", &geometry.brep)?;
        let read = with_gmsh(|gmsh| {
            gmsh.import_shapes(&file.0)?;
            Ok((part_boxes(gmsh, parts(gmsh, geometry)?)?, signatures(gmsh)?))
        })?;
        alone.push(read);
        files.push(file);
    }
    let combined = TempFile::new("brep");
    with_gmsh(|gmsh| {
        for file in &files {
            gmsh.import_shapes(&file.0)?;
        }
        gmsh.write(&combined.0)
    })?;
    let brep = std::fs::read_to_string(&combined.0)
        .map_err(|e| GmshError::Other(format!("{}: {e}", combined.0.display())))?;
    let (new_parts, new_signatures, closures) = with_gmsh(|gmsh| {
        gmsh.import_shapes(&combined.0)?;
        let found = default_parts(gmsh)?;
        let mut closures = Vec::with_capacity(found.len());
        for (shape, _) in &found {
            closures.push(closure(gmsh, shape.entities())?);
        }
        Ok((part_boxes(gmsh, found)?, signatures(gmsh)?, closures))
    })?;
    // The parts of the whole are found in the inputs by where they lie, in input order.
    let mut wanted: Vec<Vec<Option<(i32, String, BoundingBox)>>> = (alone.iter())
        .map(|(parts, _)| parts.iter().cloned().map(Some).collect())
        .collect();
    let mut owner: Vec<Option<(usize, String)>> = Vec::with_capacity(new_parts.len());
    for (dim, _, bounds) in &new_parts {
        let found = wanted.iter_mut().enumerate().find_map(|(input, parts)| {
            let part = parts.iter_mut().find(|p| {
                p.as_ref()
                    .is_some_and(|(d, _, b)| d == dim && same_box(b, bounds))
            })?;
            part.take().map(|(_, name, _)| (input, name))
        });
        owner.push(found);
    }
    // Names: in input order, so that the first input keeps all of its own.
    let mut used = BTreeSet::new();
    let mut renamed = vec![BTreeMap::new(); inputs.len()];
    let mut order: Vec<usize> = (0..new_parts.len()).collect();
    order.sort_by_key(|&i| owner[i].as_ref().map_or(usize::MAX, |(input, _)| *input));
    let mut names = vec![String::new(); new_parts.len()];
    for input in 0..inputs.len() {
        let own: Vec<String> = (inputs[input].compounds.iter())
            .map(|c| c.name.clone())
            .collect();
        for &i in order
            .iter()
            .filter(|&&i| owner[i].as_ref().is_some_and(|o| o.0 == input))
        {
            let name = &owner[i].as_ref().map(|o| o.1.clone()).unwrap_or_default();
            let unique = unique_name(name, &used);
            if &unique != name {
                renamed[input].insert(name.clone(), unique.clone());
            }
            used.insert(unique.clone());
            names[i] = unique;
        }
        for name in own {
            let unique = unique_name(&name, &used);
            if unique != name {
                renamed[input].insert(name, unique.clone());
            }
            used.insert(unique);
        }
    }
    for (i, name) in names.iter_mut().enumerate() {
        if owner[i].is_none() {
            *name = unique_name(&new_parts[i].1, &used);
            used.insert(name.clone());
        }
    }
    // Each input's entities in the parts found for it, renumbered.
    let mut tags = Vec::with_capacity(inputs.len());
    for (input, (_, old)) in alone.iter().enumerate() {
        let mine: BTreeSet<_> = (0..new_parts.len())
            .filter(|&i| owner[i].as_ref().is_some_and(|o| o.0 == input))
            .flat_map(|i| closures[i].iter().copied())
            .collect();
        let new: Vec<_> = (new_signatures.iter())
            .filter(|s| mine.contains(&s.entity))
            .cloned()
            .collect();
        tags.push(unchanged(old, &new));
    }
    let rename = |input: usize, name: &str| {
        renamed[input]
            .get(name)
            .cloned()
            .unwrap_or_else(|| name.to_owned())
    };
    let compounds = (inputs.iter().enumerate())
        .flat_map(|(input, g)| g.compounds.iter().map(move |c| (input, c)))
        .map(|(input, c)| CompoundPart {
            name: rename(input, &c.name),
            parts: c.parts.iter().map(|p| rename(input, p)).collect(),
        })
        .collect();
    let mut sources: Vec<&str> = Vec::new();
    for geometry in inputs {
        for source in geometry.source.split(", ") {
            if !source.is_empty() && !sources.contains(&source) {
                sources.push(source);
            }
        }
    }
    let first = &inputs[0];
    Ok(MergedGeometry {
        geometry: Geometry {
            source: sources.join(", "),
            brep,
            meshing: first.meshing,
            mesh_items: Vec::new(),
            part_names: names,
            refinement: first.refinement,
            signatures: new_signatures,
            compounds,
        },
        tags,
        renamed,
    })
}

/// The entities and all on their boundaries, down to the vertices.
fn closure(
    gmsh: &crate::gmsh::Gmsh,
    entities: impl Iterator<Item = Entity>,
) -> Result<BTreeSet<plx_mesh::CadEntity>, GmshError> {
    let mut seen = BTreeSet::new();
    let mut open: Vec<Entity> = entities.collect();
    let mut found = BTreeSet::new();
    while let Some((dim, tag)) = open.pop() {
        if !seen.insert((dim, tag)) {
            continue;
        }
        if let Some(entity) = plx_mesh::CadEntity::of(dim, tag) {
            found.insert(entity);
        }
        if dim > 0 {
            let (_, down) = gmsh.adjacencies(dim, tag)?;
            open.extend(down.into_iter().map(|t| (dim - 1, t.abs())));
        }
    }
    Ok(found)
}
