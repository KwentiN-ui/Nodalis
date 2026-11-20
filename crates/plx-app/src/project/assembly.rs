//! The Geometry row of an analysis as a collection: the geometry objects of several links,
//! whole blocks or single parts, with their meshes, put together into one assembly, like an
//! assembly read from a file. A part that a CAD operation or a transform changed takes the place of the same part of the file
//! linked unchanged, so that the assembly does not have it twice.

use plx_mesher::MergedGeometry;

use super::*;

/// Geometries being put together on a worker thread for the Geometry row of an analysis, by analysis and row, with the fingerprint of what they are made from.
pub(super) type Merges = HashMap<(u64, String), (u64, Merge)>;

pub(super) enum Merge {
    Making(Receiver<Result<MergedGeometry, String>>),
    Made(Box<MergedGeometry>),
    Failed(String),
}

/// What one source gives the Geometry row of a block: some of its parts, through part
/// links, or all of it (`None`).
type Piece = (u64, Option<Vec<String>>);

/// The sources of the Geometry row of block `id`, in the order of their links.
fn pieces(all: &[Link], id: u64) -> Vec<Piece> {
    let mut pieces: Vec<Piece> = Vec::new();
    for link in all
        .iter()
        .filter(|l| l.target == id && l.item.first().is_some_and(|c| c == links::GEOMETRY))
    {
        let part = links::is_geometry_part(link).then(|| link.item[1].clone());
        match pieces.iter_mut().find(|(source, _)| *source == link.source) {
            Some((_, Some(parts))) => parts.extend(part),
            Some((_, None)) => {}
            None => pieces.push((link.source, part.map(|p| vec![p]))),
        }
    }
    pieces
}

impl ProjectApp {
    /// The parts of file blocks the geometry `source` gives comes from, by file block and
    /// part, and whether it changed them on the way: through a CAD operation.
    fn origins(
        &self,
        source: u64,
        parts: Option<&[String]>,
        depth: usize,
    ) -> (BTreeSet<(u64, String)>, bool) {
        let mut found = BTreeSet::new();
        let Some(node) = self.nodes.iter().find(|n| n.id == source) else {
            return (found, false);
        };
        if depth > 32 {
            return (found, false);
        }
        match node.kind() {
            Kind::File => {
                let all = node.as_file().map(|f| f.parts.clone()).unwrap_or_default();
                let parts = parts.map_or(all, <[String]>::to_vec);
                found.extend(parts.into_iter().map(|p| (source, p)));
                (found, false)
            }
            Kind::CadOperation | Kind::Transform | Kind::Analysis => {
                let mut changed = node.kind() != Kind::Analysis;
                for (from, parts) in pieces(&self.links, source) {
                    let (origins, derived) = self.origins(from, parts.as_deref(), depth + 1);
                    found.extend(origins);
                    changed |= derived;
                }
                (found, changed)
            }
            _ => (found, false),
        }
    }

    /// Reports a problem of a link once.
    fn warn_once(&mut self, line: String) {
        if self.warned.insert(line.clone()) {
            self.output.push(line);
        }
    }

    /// The geometry objects the Geometry row of analysis `link.target` takes from all its
    /// links, with the meshes they bring, carried by the first of them; `None` when they are
    /// up to date, or while a part is still being made or the geometries put together.
    pub(super) fn assembled_objects(&mut self, link: &Link) -> Option<Payload> {
        let target = link.target;
        let first = (self.links.iter())
            .find(|l| l.target == target && l.item.first().is_some_and(|c| c == links::GEOMETRY));
        if first != Some(link) {
            return None;
        }
        let mut pieces = pieces(&self.links, target);
        // An analysis loading in its window gives its objects once it has loaded; a block
        // that has none, such as a CAD operation with nothing picked, gives nothing.
        for (source, _) in &pieces {
            let node = self.nodes.iter().find(|n| n.id == *source)?;
            if node.kind() == Kind::Analysis && node.link_data().is_none() {
                return None;
            }
        }
        let path = [links::GEOMETRY.to_owned()];
        pieces.retain(|(source, _)| {
            (self.nodes.iter()).any(|n| {
                n.id == *source
                    && (n.kind() == Kind::File
                        || n.link_data().is_some_and(|d| links::exists(d, &path)))
            })
        });
        // The parts of files that come changed through another link.
        let mut changed = BTreeSet::new();
        let mut changers: BTreeMap<(u64, String), u64> = BTreeMap::new();
        for (source, parts) in &pieces {
            if self
                .nodes
                .iter()
                .any(|n| n.id == *source && n.kind() == Kind::File)
            {
                continue;
            }
            let (origins, derived) = self.origins(*source, parts.as_deref(), 0);
            if derived {
                for origin in origins {
                    changers.entry(origin.clone()).or_insert(*source);
                    changed.insert(origin);
                }
            }
        }
        let name = self.name_of(target);
        let mut copies = Vec::new();
        let mut names = Vec::new();
        let mut print = plx_mesh::Fnv::new();
        for (source, parts) in pieces {
            let from = self.name_of(source);
            // What the analysis deleted of what the link brings stays out.
            let left_out = (self.links.iter())
                .find(|l| l.source == source && l.target == target && l.item == path)
                .map(links::left_out)
                .unwrap_or_default();
            let node = self.node(source)?;
            if let Some(file) = node.as_file_mut()
                && file.model.geometry.is_some()
            {
                let read = file.read?;
                let all = parts.unwrap_or_else(|| file.parts.clone());
                let (kept, left): (Vec<String>, Vec<String>) =
                    (all.into_iter()).partition(|p| !changed.contains(&(source, p.clone())));
                let made = if kept.is_empty() {
                    Ok(None)
                } else {
                    file.subset(&kept).map(Some)
                };
                for part in &left {
                    let by = changers.get(&(source, part.clone())).copied();
                    let by = by.map(|id| self.name_of(id)).unwrap_or_default();
                    self.warn_once(format!(
                        "{name} takes {part} changed from {by} in place of {part} of {from}."
                    ));
                }
                match made {
                    Ok(None) => {}
                    Ok(Some(None)) => return None,
                    Ok(Some(Some(geometry))) => {
                        print.add(&read.to_le_bytes());
                        print.add_debug(&kept);
                        copies.push(links::MeshCopy::shapes(geometry));
                        names.push(from);
                    }
                    Err(why) => {
                        self.warn_once(format!("{from}: the parts linked are not taken: {why}."));
                        return None;
                    }
                }
                continue;
            }
            let Some(data) = node.link_data() else {
                continue;
            };
            if !links::exists(data, &path) {
                continue;
            }
            print.add(&links::fingerprint(data, &path).to_le_bytes());
            print.add_debug(&left_out);
            let mut copy = links::MeshCopy::of(data);
            let failed = copy.leave_out(&left_out).err();
            copies.push(copy);
            if let Some(why) = failed {
                self.warn_once(format!(
                    "{from}: the parts deleted in {name} are taken: {why}."
                ));
            }
            names.push(from);
        }
        if copies.is_empty() {
            return None;
        }
        // The parts the analysis meshes itself leave the link's meshes of them out.
        let own = (self.nodes.iter().find(|n| n.id == target))
            .and_then(Node::link_data)
            .and_then(|d| d.geometry.map(|g| g.own_meshes.clone()));
        print.add_debug(&own);
        let print = print.finish();
        let key = (target, path.to_vec());
        if self.applied.get(&key) == Some(&print) {
            return None;
        }
        let copy = if copies.len() == 1 {
            copies.pop()?
        } else {
            // The geometries of the objects that have one are put together; a mesh without,
            // such as one of a mesh block, joins as an imported mesh, a geometry object of
            // its own.
            let with: Vec<usize> = (copies.iter().enumerate())
                .filter(|(_, c)| c.geometry.is_some())
                .map(|(i, _)| i)
                .collect();
            let geometries: Vec<plx_model::Geometry> = (with.iter())
                .filter_map(|&i| copies[i].geometry.clone())
                .collect();
            let merged = match geometries.len() {
                0 => None,
                _ => match self.merged(target, links::GEOMETRY, geometries, print)? {
                    Ok(merged) => Some(spread(merged, &with, copies.len())),
                    Err(why) => {
                        self.warn_once(format!(
                            "{name}: the geometries are not put together: {why}."
                        ));
                        return None;
                    }
                },
            };
            let (copy, renamed) = links::merge_meshes(copies, merged);
            for (renamed, from) in renamed.iter().zip(&names) {
                for (old, new) in renamed {
                    self.warn_once(format!(
                        "{name}: {old} of {from} is called {new} there, since another object \
                         has one of the name."
                    ));
                }
            }
            copy
        };
        self.applied.insert(key, print);
        Some(Payload::Objects(Box::new(copy)))
    }

    /// The geometries put together for row `cell` of `target`, from `inputs` of fingerprint
    /// `print`; `None` while that is done on a worker thread, which starts here.
    fn merged(
        &mut self,
        target: u64,
        cell: &str,
        inputs: Vec<plx_model::Geometry>,
        print: u64,
    ) -> Option<Result<MergedGeometry, String>> {
        let key = (target, cell.to_owned());
        match self.merges.get(&key) {
            Some((made_from, merge)) if *made_from == print => {
                return match merge {
                    Merge::Making(_) => None,
                    Merge::Made(merged) => Some(Ok((**merged).clone())),
                    Merge::Failed(why) => Some(Err(why.clone())),
                };
            }
            _ => {}
        }
        let (sender, receiver) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let merged = plx_mesher::merge_geometries(&inputs).map_err(|e| e.to_string());
            let _ = sender.send(merged);
        });
        self.merges.insert(key, (print, Merge::Making(receiver)));
        None
    }

    /// Takes the geometries put together since; whether any were.
    pub(super) fn poll_merges(&mut self) -> bool {
        let mut done = false;
        for (_, merge) in self.merges.values_mut() {
            if let Merge::Making(receiver) = merge {
                *merge = match receiver.try_recv() {
                    Ok(Ok(merged)) => Merge::Made(Box::new(merged)),
                    Ok(Err(why)) => Merge::Failed(why),
                    Err(std::sync::mpsc::TryRecvError::Empty) => continue,
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                        Merge::Failed("putting them together stopped unexpectedly".into())
                    }
                };
                done = true;
            }
        }
        done
    }

    pub(super) fn merging(&self) -> bool {
        (self.merges.values()).any(|(_, m)| matches!(m, Merge::Making(_)))
    }
}

/// The geometry put together of the meshes `with` of `count`, with their new tags and names
/// ids given to each of them, by index: those without geometry keep theirs.
fn spread(merged: MergedGeometry, with: &[usize], count: usize) -> MergedGeometry {
    let MergedGeometry {
        geometry,
        tags,
        ids,
    } = merged;
    let mut all_tags = vec![Default::default(); count];
    let mut all_ids = vec![Default::default(); count];
    for ((&index, tags), ids) in with.iter().zip(tags).zip(ids) {
        all_tags[index] = tags;
        all_ids[index] = ids;
    }
    MergedGeometry {
        geometry,
        tags: all_tags,
        ids: all_ids,
    }
}
