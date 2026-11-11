//! Bolts preloaded across a cut through their shank, Ansys' bolt pretension: the user picks
//! the two faces of the cut and the preload, Nodalis derives the rest when the input file is
//! written ([`FeModel::lower_bolts`]): which side the mesh is cut on, the tie between the
//! halves of a bolt split into two bodies, the pre-tension load in every step (the preload
//! in the first step that takes loads, locked at its length after it), and the contact
//! regions on the cut faces, which the bolt replaces.

use std::collections::{BTreeMap, BTreeSet};

use plx_mesh::{ElementId, FeMesh, NodeId};
use serde::{Deserialize, Serialize};

use crate::{FeModel, Load, LoadKind, ModelItem, Region, Step, Tie};

/// A bolt cut through its shank: the faces on the two sides of the cut and its preload.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Bolt {
    /// The internal identity other items refer to; the user sees only the name.
    pub id: crate::Id,
    pub name: String,
    #[serde(default = "crate::active")]
    pub active: bool,
    /// The faces of the cut: of the two bodies of a split bolt, or both on the face inside a
    /// bolt meshed in one piece (side B may then stay empty).
    pub side_a: Region,
    pub side_b: Region,
    /// The preload in the first step that takes loads.
    pub preload: Preload,
    /// Direction of the preload; `None` takes the normal of the cut.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub axis: Option<[f64; 3]>,
    /// The faces of the internal thread the bolt is screwed into, on the nut or the part
    /// with the tapped hole; empty when the user joins the bolt there himself. The faces of
    /// the bolt that face them are tied to them where they are ([`FeModel::resolve_thread`]).
    #[serde(default = "no_faces", skip_serializing_if = "Region::is_empty")]
    pub thread: Region,
    pub color: [u8; 3],
}

fn no_faces() -> Region {
    Region::Faces(Vec::new())
}

impl Bolt {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            active: true,
            side_a: Region::Faces(Vec::new()),
            side_b: Region::Faces(Vec::new()),
            preload: Preload::Force(0.0),
            axis: None,
            thread: no_faces(),
            color: [230, 140, 0],
        }
    }

    /// The sides of the cut and the thread.
    pub fn regions(&self) -> [&Region; 3] {
        [&self.side_a, &self.side_b, &self.thread]
    }

    pub fn regions_mut(&mut self) -> [&mut Region; 3] {
        [&mut self.side_a, &mut self.side_b, &mut self.thread]
    }

    /// The nodes of both sides of the cut, of their faces too.
    pub fn nodes(&self, mesh: &FeMesh) -> BTreeSet<NodeId> {
        [&self.side_a, &self.side_b]
            .into_iter()
            .flat_map(|region| side_nodes(region, mesh))
            .collect()
    }
}

/// How a bolt is tightened.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Preload {
    /// The force pulling the two sides of the cut together.
    Force(f64),
    /// The length the bolt is shortened by.
    Shortening(f64),
}

impl Preload {
    pub fn value(self) -> f64 {
        match self {
            Preload::Force(v) | Preload::Shortening(v) => v,
        }
    }
}

/// What a bolt does in a step, Ansys' "Define By" of a bolt pretension.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum BoltAction {
    /// Pulled by a force.
    Load(f64),
    /// Shortened by a length since the start of the analysis.
    Adjust(f64),
    /// Kept at the length it had at the end of the previous step.
    Lock,
    /// Loose: no preload.
    Open,
}

impl BoltAction {
    pub fn name(self) -> &'static str {
        match self {
            BoltAction::Load(_) => "Load",
            BoltAction::Adjust(_) => "Adjustment",
            BoltAction::Lock => "Lock",
            BoltAction::Open => "Open",
        }
    }
}

impl From<Preload> for BoltAction {
    fn from(preload: Preload) -> Self {
        match preload {
            Preload::Force(f) => BoltAction::Load(f),
            Preload::Shortening(u) => BoltAction::Adjust(u),
        }
    }
}

/// What a bolt does in one step instead of the automatic action ([`FeModel::bolt_action`]),
/// and the amplitude its load or adjustment follows there.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoltChange {
    pub bolt: crate::Id,
    pub action: BoltAction,
    /// The step keeps the automatic action; the change only gives the amplitude.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub auto: bool,
    /// The amplitude of the load or adjustment; `None` is the step's default ramp.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub amplitude: Option<crate::Id>,
}

/// How the two sides of a bolt's cut are joined in the input file.
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedBolt {
    /// The element faces of the side the mesh is cut on, all on one side of the cut.
    pub cut_faces: Vec<(ElementId, u8)>,
    /// The other side, tied to the cut side; `None` when the sides share their nodes.
    pub tied_to: Option<Region>,
    /// The cut side is side A.
    pub cut_on_a: bool,
    /// Distance the tie looks for the other side within.
    pub tie_tolerance: Option<f64>,
    /// The axis of a shank picked by its cylindrical faces and cut across its middle; the
    /// direction of the preload unless the bolt gives one.
    pub shank_axis: Option<[f64; 3]>,
    /// Why, in words, for the dialog.
    pub why: String,
}

/// The bolt's side of the tie to its internal thread ([`FeModel::resolve_thread`]).
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedThread {
    /// The outer faces of the bolt tied to the thread.
    pub bolt_faces: Vec<(ElementId, u8)>,
    /// The distance of the nearest of them to the thread.
    pub gap: f64,
    /// The position tolerance of the tie: farther than every node of those faces.
    pub tolerance: f64,
}

/// What [`FeModel::lower_bolts`] added, by the bolt each item stands for.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LoweredBolts {
    pub first_tie: usize,
    pub tie_bolts: Vec<usize>,
    /// Per step: the index of the first added load and the bolt of each added load.
    pub first_loads: Vec<usize>,
    pub load_bolts: Vec<Vec<usize>>,
    /// The contact regions the bolts replace, with the bolt.
    pub replaced: BTreeMap<usize, usize>,
}

impl LoweredBolts {
    /// The bolt a tie or load of the lowered model stands for.
    pub fn bolt_of(&self, item: ModelItem) -> Option<usize> {
        match item {
            ModelItem::Tie(i) => self.tie_bolts.get(i.checked_sub(self.first_tie)?).copied(),
            ModelItem::Load(s, i) => {
                let first = *self.first_loads.get(s)?;
                self.load_bolts.get(s)?.get(i.checked_sub(first)?).copied()
            }
            _ => None,
        }
    }
}

/// Share of the nodes of both sides of a contact region on a bolt's cut that makes the bolt
/// replace the region.
const REPLACED_SHARE: f64 = 0.5;

impl FeModel {
    /// What the bolt does in the step `index`: the change of the step, else
    /// [`FeModel::auto_bolt_action`]. `None` when the step takes no pre-tension or the bolt
    /// is open.
    pub fn bolt_action(&self, bolt: &Bolt, index: usize) -> Option<BoltAction> {
        let auto = self.auto_bolt_action(bolt, index)?;
        let action = (self.steps[index].bolt_changes.iter())
            .find(|c| c.bolt == bolt.id && !c.auto)
            .map_or(auto, |c| c.action);
        (action != BoltAction::Open).then_some(action)
    }

    /// The amplitude the bolt's load or adjustment follows in the step `index`.
    pub fn bolt_amplitude(&self, bolt: &Bolt, index: usize) -> Option<crate::Id> {
        (self.steps.get(index)?.bolt_changes.iter())
            .find(|c| c.bolt == bolt.id)?
            .amplitude
    }

    /// What the bolt does in the step `index` without a change: the preload in the first
    /// active step that takes it, the lock after it and nothing before it. `None` when the
    /// step takes no pre-tension.
    pub fn auto_bolt_action(&self, bolt: &Bolt, index: usize) -> Option<BoltAction> {
        let step = self.steps.get(index)?;
        if !step.active || !takes_pre_tension(step) {
            return None;
        }
        let first = self
            .steps
            .iter()
            .position(|s| s.active && takes_pre_tension(s))?;
        Some(match index.cmp(&first) {
            std::cmp::Ordering::Less => BoltAction::Open,
            std::cmp::Ordering::Equal => bolt.preload.into(),
            std::cmp::Ordering::Greater => BoltAction::Lock,
        })
    }

    /// How the sides of the bolt are joined: the side with the finer mesh is cut, the other
    /// one tied to it; a cut inside a bolt meshed in one piece needs no tie.
    pub fn resolve_bolt(&self, bolt: &Bolt, mesh: &FeMesh) -> Result<ResolvedBolt, String> {
        let a = solid_faces(&bolt.side_a, mesh);
        let b = solid_faces(&bolt.side_b, mesh);
        if a.is_empty() && b.is_empty() {
            return Err("no faces of solid elements on the cut".into());
        }
        if b.is_empty()
            && let Some(shank) = Shank::of(&a, mesh)
        {
            // Across the middle of the free part of the shank: beside the part of it in the
            // thread, which a cut must not split from the thread.
            let engaged = match self.thread_faces(bolt, mesh) {
                Ok(Some(thread)) => (thread.bolt_faces.iter())
                    .flat_map(|&f| face_nodes(mesh, f))
                    .filter_map(|n| mesh.node(n))
                    .map(|p| dot(p, shank.axis))
                    .fold(None, |range: Option<(f64, f64)>, t| {
                        Some(range.map_or((t, t), |(lo, hi)| (lo.min(t), hi.max(t))))
                    }),
                _ => None,
            };
            let middle = match engaged {
                None => 0.5 * (shank.start + shank.end),
                Some((lo, hi)) => {
                    let below = (shank.start, lo.min(shank.end));
                    let above = (hi.max(shank.start), shank.end);
                    let (from, to) = if below.1 - below.0 >= above.1 - above.0 {
                        below
                    } else {
                        above
                    };
                    if to <= from {
                        return Err("the thread takes the whole shank: no free part to cut".into());
                    }
                    0.5 * (from + to)
                }
            };
            return shank.cut(&a, mesh, middle);
        }
        let nodes = |faces: &[(ElementId, u8)]| -> BTreeSet<NodeId> {
            faces.iter().flat_map(|&f| face_nodes(mesh, f)).collect()
        };
        let (nodes_a, nodes_b) = (nodes(&a), nodes(&b));
        let shared = b.is_empty() || nodes_a.intersection(&nodes_b).next().is_some();
        if shared {
            // One mesh through the cut: the faces of the elements on one side of it.
            let faces = if a.is_empty() { &b } else { &a };
            return Ok(ResolvedBolt {
                cut_faces: one_side(faces, mesh),
                tied_to: None,
                cut_on_a: !a.is_empty(),
                tie_tolerance: None,
                shank_axis: None,
                why: "The sides share their nodes: the mesh is cut between them.".into(),
            });
        }
        if a.is_empty() {
            return Err("side A has no faces of solid elements".into());
        }
        let (size_a, size_b) = (mean_size(&a, mesh), mean_size(&b, mesh));
        // The finer side is cut, so the tie's slave nodes are the denser ones.
        let cut_on_a = size_a <= size_b;
        let (cut, other, region, size) = if cut_on_a {
            (a, &bolt.side_b, "A", size_a)
        } else {
            (b, &bolt.side_a, "B", size_b)
        };
        Ok(ResolvedBolt {
            cut_faces: one_side(&cut, mesh),
            tied_to: Some(other.clone()),
            cut_on_a,
            tie_tolerance: (size > 0.0).then_some(0.25 * size),
            shank_axis: None,
            why: format!(
                "Two bodies: the mesh is cut on side {region}{} and tied to the other side.",
                if size_a == size_b {
                    ""
                } else {
                    " (finer mesh)"
                }
            ),
        })
    }

    /// How the bolt is joined to its internal thread: the outer faces of the bolt nearest to
    /// the thread faces, a band half an element wide beyond the nearest, on the thread's
    /// side of the cut, tied to them where they are. They are the shank in a tapped hole
    /// (the bore picked as the thread), or the end face of a bolt simplified to a cylinder
    /// that stops short of the part it is screwed into. `None` without thread faces.
    pub fn resolve_thread(
        &self,
        bolt: &Bolt,
        mesh: &FeMesh,
    ) -> Result<Option<ResolvedThread>, String> {
        let Some(mut thread) = self.thread_faces(bolt, mesh)? else {
            return Ok(None);
        };
        let Ok(cut) = self.resolve_bolt(bolt, mesh) else {
            return Ok(Some(thread));
        };
        // The plane of the cut: the centre of its faces, square to the preload.
        let points: Vec<[f64; 3]> = (cut.cut_faces.iter())
            .filter_map(|&f| face_centre(mesh, f))
            .collect();
        let normal = bolt.axis.or(cut.shank_axis).or_else(|| {
            let mut sum = [0.0; 3];
            for n in cut.cut_faces.iter().filter_map(|&f| face_normal(mesh, f)) {
                (0..3).for_each(|k| sum[k] += n[k]);
            }
            (dot(sum, sum) > 0.0).then_some(sum)
        });
        let (Some(normal), false) = (normal, points.is_empty()) else {
            return Ok(Some(thread));
        };
        let k = points.len() as f64;
        let origin = [0, 1, 2].map(|d| points.iter().map(|p| p[d]).sum::<f64>() / k);
        let side = |p: [f64; 3]| dot([0, 1, 2].map(|d| p[d] - origin[d]), normal);
        // The side of the thread: where most of the bolt faces tied to it lie.
        let sides: Vec<f64> = (thread.bolt_faces.iter())
            .map(|&f| face_centre(mesh, f).map_or(0.0, side))
            .collect();
        let thread_side = sides.iter().sum::<f64>().signum();
        let kept: Vec<(ElementId, u8)> = (thread.bolt_faces.iter().zip(&sides))
            .filter(|(_, s)| s.signum() == thread_side)
            .map(|(&f, _)| f)
            .collect();
        if kept.is_empty() {
            return Err("the thread lies on the cut".into());
        }
        thread.bolt_faces = kept;
        Ok(Some(thread))
    }

    /// [`Self::resolve_thread`] without regard to the cut.
    fn thread_faces(&self, bolt: &Bolt, mesh: &FeMesh) -> Result<Option<ResolvedThread>, String> {
        let thread = solid_faces(&bolt.thread, mesh);
        if thread.is_empty() {
            return if bolt.thread.is_empty() {
                Ok(None)
            } else {
                Err("the thread has no faces of solid elements".into())
            };
        }
        // The elements of the bolt: the bodies (or parts) of its cut, without that of the
        // thread when both are bodies of one compound part.
        let body_of = |element: ElementId| -> Option<Vec<ElementId>> {
            let part = mesh.parts.iter().find(|p| p.elements.contains(&element))?;
            Some(
                (part.bodies.iter())
                    .find(|b| b.elements.contains(&element))
                    .map_or_else(|| part.elements.clone(), |b| b.elements.clone()),
            )
        };
        let sides: BTreeSet<ElementId> = (solid_faces(&bolt.side_a, mesh).into_iter())
            .chain(solid_faces(&bolt.side_b, mesh))
            .map(|(e, _)| e)
            .collect();
        let nut: BTreeSet<ElementId> = (thread.iter())
            .filter_map(|&(e, _)| body_of(e))
            .flatten()
            .collect();
        let mut elements: BTreeSet<ElementId> = BTreeSet::new();
        for &e in &sides {
            if !elements.contains(&e)
                && let Some(body) = body_of(e)
            {
                elements.extend(body);
            }
        }
        elements.retain(|e| !nut.contains(e));
        if elements.is_empty() {
            return Err("the bolt is the part of the thread: no faces of the bolt left".into());
        }
        // The outer faces of the bolt: those no other of its elements shares.
        let mut faces: BTreeMap<Vec<NodeId>, Option<(ElementId, u8)>> = BTreeMap::new();
        for element in elements.iter().filter_map(|&e| mesh.element(e)) {
            if element.shape.family() != plx_mesh::ElementFamily::Solid {
                continue;
            }
            for (i, face) in element.faces().iter().enumerate() {
                let key = corner_key(element, face.corners);
                faces
                    .entry(key)
                    .and_modify(|f| *f = None)
                    .or_insert(Some((element.id, (i + 1) as u8)));
            }
        }
        let outer: Vec<(ElementId, u8)> = faces.into_values().flatten().collect();
        let centres: Vec<[f64; 3]> = (outer.iter())
            .map(|&f| face_centre(mesh, f).unwrap_or([f64::NAN; 3]))
            .collect();
        let distances = crate::contact_check::distances(mesh, &thread, &centres);
        let Some(nearest) = distances
            .iter()
            .copied()
            .filter(|d| d.is_finite())
            .reduce(f64::min)
        else {
            return Err("no face of the bolt near the thread".into());
        };
        let band = nearest + 0.5 * mean_size(&outer, mesh);
        let bolt_faces: Vec<(ElementId, u8)> = (outer.iter().zip(&distances))
            .filter(|(_, d)| **d <= band)
            .map(|(&f, _)| f)
            .collect();
        // Far enough for every node of those faces.
        let nodes: Vec<[f64; 3]> = (bolt_faces.iter())
            .flat_map(|&f| face_nodes(mesh, f))
            .collect::<BTreeSet<_>>()
            .into_iter()
            .filter_map(|n| mesh.node(n))
            .collect();
        let farthest = (crate::contact_check::distances(mesh, &thread, &nodes).into_iter())
            .filter(|d| d.is_finite())
            .fold(nearest, f64::max);
        let size = mean_size(&thread, mesh);
        Ok(Some(ResolvedThread {
            bolt_faces,
            gap: nearest,
            tolerance: farthest + 0.1 * size.max(farthest),
        }))
    }

    /// The nodes on the cut of a bolt: of both its sides, or of the faces the slice through
    /// a shank cuts.
    pub fn bolt_cut_nodes(&self, bolt: &Bolt, mesh: &FeMesh) -> BTreeSet<NodeId> {
        match self.resolve_bolt(bolt, mesh) {
            Ok(resolved) if resolved.shank_axis.is_some() => (resolved.cut_faces.iter())
                .flat_map(|&f| face_nodes(mesh, f))
                .collect(),
            _ => bolt.nodes(mesh),
        }
    }

    /// The contact regions the active bolts replace, with the bolt: those with both sides
    /// mostly on the faces of a bolt's cut.
    pub fn replaced_contact_regions(&self, mesh: &FeMesh) -> BTreeMap<usize, usize> {
        let mut replaced = BTreeMap::new();
        if self.contact_regions.is_empty() {
            return replaced;
        }
        let bolts: Vec<(usize, BTreeSet<NodeId>)> = (self.bolts.iter().enumerate())
            .filter(|(_, b)| b.active)
            .map(|(i, b)| (i, self.bolt_cut_nodes(b, mesh)))
            .collect();
        for (r, region) in self.contact_regions.iter().enumerate() {
            let sides = region.regions().map(|side| side_nodes(side, mesh));
            if let Some(&(b, _)) = bolts.iter().find(|(_, cut)| {
                sides.iter().all(|nodes| {
                    let on = nodes.iter().filter(|n| cut.contains(n)).count();
                    !nodes.is_empty() && on as f64 >= REPLACED_SHARE * nodes.len() as f64
                })
            }) {
                replaced.insert(r, b);
            }
        }
        replaced
    }

    /// The model with its bolts replaced by the pre-tension loads and ties they stand for,
    /// and the contact regions on their cuts switched off. Bolts whose cut cannot be
    /// resolved are left out; the model check reports them.
    pub fn lower_bolts(&self, mesh: &FeMesh) -> (FeModel, LoweredBolts) {
        let mut lowered = self.clone();
        let mut origin = LoweredBolts {
            first_tie: self.ties.len(),
            first_loads: self.steps.iter().map(|s| s.loads.len()).collect(),
            load_bolts: vec![Vec::new(); self.steps.len()],
            ..LoweredBolts::default()
        };
        if self.bolts.is_empty() {
            return (lowered, origin);
        }
        lowered.bolts.clear();
        for step in &mut lowered.steps {
            step.bolt_changes.clear();
        }
        origin.replaced = self.replaced_contact_regions(mesh);
        for &r in origin.replaced.keys() {
            let region = &mut lowered.contact_regions[r];
            region.active = false;
            let id = region.id;
            lowered.remove_contact_changes(id);
        }
        let mut taken: BTreeSet<String> = (self.steps.iter())
            .flat_map(|s| s.loads.iter().map(|l| l.name.clone()))
            .chain(self.ties.iter().map(|t| t.name.clone()))
            .collect();
        for (b, bolt) in self.bolts.iter().enumerate().filter(|(_, b)| b.active) {
            let Ok(resolved) = self.resolve_bolt(bolt, mesh) else {
                continue;
            };
            let name = unique(&bolt.name, &mut taken);
            if let Some(other) = &resolved.tied_to {
                let mut tie = Tie::new(unique(&format!("{name}_Cut"), &mut taken));
                tie.master = other.clone();
                tie.slave = Region::Faces(resolved.cut_faces.clone());
                tie.position_tolerance = resolved.tie_tolerance;
                lowered.ties.push(tie);
                origin.tie_bolts.push(b);
            }
            if let Ok(Some(thread)) = self.resolve_thread(bolt, mesh) {
                let mut tie = Tie::new(unique(&format!("{name}_Thread"), &mut taken));
                tie.master = bolt.thread.clone();
                tie.slave = Region::Faces(thread.bolt_faces);
                tie.position_tolerance = Some(thread.tolerance);
                tie.adjust = false;
                lowered.ties.push(tie);
                origin.tie_bolts.push(b);
            }
            let load_id = crate::Id::new();
            for s in 0..self.steps.len() {
                let Some(action) = self.bolt_action(bolt, s) else {
                    continue;
                };
                let amplitude = match action {
                    BoltAction::Load(_) | BoltAction::Adjust(_) => self.bolt_amplitude(bolt, s),
                    _ => None,
                };
                let (value, by_displacement, lock) = match action {
                    BoltAction::Load(f) => (f, false, false),
                    BoltAction::Adjust(u) => (u, true, false),
                    BoltAction::Lock => (0.0, false, true),
                    BoltAction::Open => continue,
                };
                lowered.steps[s].loads.push(Load {
                    id: load_id,
                    name: name.clone(),
                    active: true,
                    visible: true,
                    region: Region::Faces(resolved.cut_faces.clone()),
                    kind: LoadKind::PreTension {
                        value,
                        by_displacement,
                        lock,
                        direction: bolt.axis.or(resolved.shank_axis),
                    },
                    amplitude,
                    factor_amplitude: None,
                    tables: Default::default(),
                    distribution: None,
                    system: None,
                    follower: false,
                });
                origin.load_bolts[s].push(b);
            }
        }
        (lowered, origin)
    }

    /// Drops the step changes of a bolt that is deleted.
    pub fn remove_bolt_changes(&mut self, bolt: crate::Id) {
        for step in &mut self.steps {
            step.bolt_changes.retain(|c| c.bolt != bolt);
        }
    }
}

/// Whether a step takes a pre-tension: one that takes forces.
fn takes_pre_tension(step: &Step) -> bool {
    step.kind.supports_load(&LoadKind::PreTension {
        value: 0.0,
        by_displacement: false,
        lock: false,
        direction: None,
    })
}

fn unique(name: &str, taken: &mut BTreeSet<String>) -> String {
    let mut candidate = name.to_string();
    let mut n = 2;
    while taken.contains(&candidate) {
        candidate = format!("{name}-{n}");
        n += 1;
    }
    taken.insert(candidate.clone());
    candidate
}

/// The nodes of a region and of its faces: a side picked on the geometry may list either.
pub(crate) fn side_nodes(region: &Region, mesh: &FeMesh) -> BTreeSet<NodeId> {
    (region.nodes(mesh).into_iter())
        .chain(
            region
                .faces(mesh)
                .into_iter()
                .flat_map(|f| face_nodes(mesh, f)),
        )
        .collect()
}

/// The faces of the region that belong to solid elements.
fn solid_faces(region: &Region, mesh: &FeMesh) -> Vec<(ElementId, u8)> {
    (region.faces(mesh).into_iter())
        .filter(|&(e, _)| {
            mesh.element(e)
                .is_some_and(|e| e.shape.family() == plx_mesh::ElementFamily::Solid)
        })
        .collect()
}

/// All nodes of an element face, corners and midside nodes.
fn face_nodes(mesh: &FeMesh, (element, face): (ElementId, u8)) -> Vec<NodeId> {
    let Some(element) = mesh.element(element) else {
        return Vec::new();
    };
    let Some(topology) = element.faces().get(usize::from(face).saturating_sub(1)) else {
        return Vec::new();
    };
    (topology.corners.iter().chain(topology.mids))
        .filter_map(|&i| element.nodes.get(i).copied())
        .collect()
}

/// Normal of an element face by the order of its corners, which points into the element
/// for CalculiX's faces; its length is the area.
fn face_normal(mesh: &FeMesh, (element, face): (ElementId, u8)) -> Option<[f64; 3]> {
    let element = mesh.element(element)?;
    let topology = element.faces().get(usize::from(face).checked_sub(1)?)?;
    let corners: Vec<[f64; 3]> = (topology.corners.iter())
        .map(|&i| mesh.node(*element.nodes.get(i)?))
        .collect::<Option<_>>()?;
    // Newell's method, times the area.
    let mut n = [0.0; 3];
    for i in 0..corners.len() {
        let (p, q) = (corners[i], corners[(i + 1) % corners.len()]);
        n[0] += (p[1] - q[1]) * (p[2] + q[2]);
        n[1] += (p[2] - q[2]) * (p[0] + q[0]);
        n[2] += (p[0] - q[0]) * (p[1] + q[1]);
    }
    Some(n.map(|c| c / 2.0))
}

/// The faces whose outward normal points the way of the first one: on a face inside a mesh
/// both elements at it have a face there, and the cut takes those of one side.
fn one_side(faces: &[(ElementId, u8)], mesh: &FeMesh) -> Vec<(ElementId, u8)> {
    let Some(reference) = faces.iter().find_map(|&f| face_normal(mesh, f)) else {
        return faces.to_vec();
    };
    (faces.iter().copied())
        .filter(|&f| {
            face_normal(mesh, f).is_some_and(|n| {
                n[0] * reference[0] + n[1] * reference[1] + n[2] * reference[2] > 0.0
            })
        })
        .collect()
}

/// Square root of the mean face area: the size of the mesh on the faces.
fn mean_size(faces: &[(ElementId, u8)], mesh: &FeMesh) -> f64 {
    let areas: Vec<f64> = (faces.iter())
        .filter_map(|&f| face_normal(mesh, f))
        .map(|n| (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt())
        .collect();
    if areas.is_empty() {
        0.0
    } else {
        (areas.iter().sum::<f64>() / areas.len() as f64).sqrt()
    }
}

/// A bolt shank picked by its cylindrical faces.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Shank {
    axis: [f64; 3],
    /// A point on the axis.
    centre: [f64; 3],
    radius: f64,
    /// Where the faces end along the axis, as distances from the origin.
    start: f64,
    end: f64,
}

impl Shank {
    /// The shank whose cylindrical faces these are; `None` for faces that are not part of a
    /// cylinder, such as the flat faces of a cut. The axis is the direction the faces'
    /// normals are all square to: the eigenvector of the smallest eigenvalue of the sum of
    /// `n nᵀ` weighted by the face areas.
    fn of(faces: &[(ElementId, u8)], mesh: &FeMesh) -> Option<Shank> {
        let normals: Vec<([f64; 3], [f64; 3])> = (faces.iter())
            .filter_map(|&f| Some((face_centre(mesh, f)?, face_normal(mesh, f)?)))
            .collect();
        let mut m = [[0.0; 3]; 3];
        let mut total = 0.0;
        for (_, n) in &normals {
            let area = norm(*n);
            if area == 0.0 {
                continue;
            }
            total += area;
            for i in 0..3 {
                for j in 0..3 {
                    m[i][j] += n[i] * n[j] / area;
                }
            }
        }
        if total == 0.0 {
            return None;
        }
        let (values, vectors) = symmetric_eigen(m);
        let order = {
            let mut order = [0, 1, 2];
            order.sort_by(|&i, &j| values[i].total_cmp(&values[j]));
            order
        };
        // A cylinder: the normals turn about one axis by a quarter turn at least.
        if values[order[0]] > 0.05 * total || values[order[1]] < 0.1 * total {
            return None;
        }
        let axis = vectors[order[0]];
        // Centre and radius from the faces' centres and normals in the plane square to
        // the axis: centre + radius * normal = face centre, by least squares.
        let u = vectors[order[1]];
        let v = vectors[order[2]];
        let mut ata = [[0.0; 3]; 3];
        let mut atb = [0.0; 3];
        for (p, n) in &normals {
            let unit = n.map(|c| c / norm(*n).max(f64::MIN_POSITIVE));
            let (nu, nv) = (dot(unit, u), dot(unit, v));
            let length = (nu * nu + nv * nv).sqrt();
            if length == 0.0 {
                continue;
            }
            let (nu, nv) = (nu / length, nv / length);
            for (row, b) in [([1.0, 0.0, nu], dot(*p, u)), ([0.0, 1.0, nv], dot(*p, v))] {
                for i in 0..3 {
                    for j in 0..3 {
                        ata[i][j] += row[i] * row[j];
                    }
                    atb[i] += row[i] * b;
                }
            }
        }
        let [cu, cv, radius] = solve3(ata, atb)?;
        let (start, end) = (faces.iter())
            .flat_map(|&f| face_nodes(mesh, f))
            .filter_map(|n| mesh.node(n))
            .map(|p| dot(p, axis))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), t| {
                (lo.min(t), hi.max(t))
            });
        let centre = [0, 1, 2].map(|i| cu * u[i] + cv * v[i]);
        (radius.abs() > 0.0 && start < end).then_some(Shank {
            axis,
            centre,
            radius: radius.abs(),
            start,
            end,
        })
    }

    /// The slice across the middle of the shank: the faces between the elements inside the
    /// shank below the middle and those above it, on the elements below. The slice follows
    /// the element faces, so the mesh needs no new nodes.
    fn cut(
        &self,
        faces: &[(ElementId, u8)],
        mesh: &FeMesh,
        middle: f64,
    ) -> Result<ResolvedBolt, String> {
        let elements: BTreeSet<ElementId> = faces.iter().map(|&(e, _)| e).collect();
        let parts: Vec<&plx_mesh::Part> = (mesh.parts.iter())
            .filter(|p| p.elements.iter().any(|e| elements.contains(e)))
            .collect();
        if parts.len() > 1 {
            return Err("the faces of the shank belong to several parts".into());
        }
        let candidates: Vec<&plx_mesh::Element> = match parts.first() {
            Some(part) => (part.elements.iter())
                .filter_map(|&e| mesh.element(e))
                .collect(),
            None => mesh.elements().iter().collect(),
        };
        let reach = self.radius + 0.25 * mean_size(faces, mesh);
        let mut below = Vec::new();
        let mut above: BTreeMap<Vec<NodeId>, ()> = BTreeMap::new();
        for element in candidates {
            if element.shape.family() != plx_mesh::ElementFamily::Solid {
                continue;
            }
            let Some(centre) = element_centre(mesh, element) else {
                continue;
            };
            // Off the axis by the part of `centre - self.centre` square to it.
            let off = [0, 1, 2].map(|i| centre[i] - self.centre[i]);
            let along = dot(off, self.axis);
            let radial = [0, 1, 2].map(|i| off[i] - along * self.axis[i]);
            if norm(radial) > reach {
                continue;
            }
            if dot(centre, self.axis) < middle {
                below.push(element);
            } else {
                for face in element.faces() {
                    above.insert(corner_key(element, face.corners), ());
                }
            }
        }
        let mut cut_faces = Vec::new();
        for element in below {
            for (i, face) in element.faces().iter().enumerate() {
                if above.contains_key(&corner_key(element, face.corners)) {
                    cut_faces.push((element.id, (i + 1) as u8));
                }
            }
        }
        if cut_faces.is_empty() {
            return Err("no element faces across the middle of the shank".into());
        }
        // The cut faces are the upper faces of the elements below the middle: the preload
        // pulls along their outward normal, the axis.
        let axis = self.axis;
        Ok(ResolvedBolt {
            cut_faces,
            tied_to: None,
            cut_on_a: true,
            tie_tolerance: None,
            shank_axis: Some(axis),
            why: "The shank is picked: the mesh is cut across the middle of its cylindrical \
                  face, along the element faces."
                .into(),
        })
    }
}

/// The corner nodes of an element face in order, which identify the face on both elements
/// at it.
fn corner_key(element: &plx_mesh::Element, corners: &[usize]) -> Vec<NodeId> {
    let mut key: Vec<NodeId> = (corners.iter())
        .filter_map(|&i| element.nodes.get(i).copied())
        .collect();
    key.sort_unstable();
    key
}

/// The mean of the corner nodes of an element face.
fn face_centre(mesh: &FeMesh, (element, face): (ElementId, u8)) -> Option<[f64; 3]> {
    let element = mesh.element(element)?;
    let topology = element.faces().get(usize::from(face).checked_sub(1)?)?;
    mean_point(
        mesh,
        topology
            .corners
            .iter()
            .filter_map(|&i| element.nodes.get(i)),
    )
}

/// The mean of the nodes of an element.
fn element_centre(mesh: &FeMesh, element: &plx_mesh::Element) -> Option<[f64; 3]> {
    mean_point(mesh, element.nodes.iter())
}

fn mean_point<'a>(mesh: &FeMesh, nodes: impl Iterator<Item = &'a NodeId>) -> Option<[f64; 3]> {
    let mut sum = [0.0; 3];
    let mut count = 0.0;
    for p in nodes.filter_map(|&n| mesh.node(n)) {
        for (s, c) in sum.iter_mut().zip(p) {
            *s += c;
        }
        count += 1.0;
    }
    (count > 0.0).then(|| sum.map(|s| s / count))
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// Eigenvalues and unit eigenvectors of a symmetric 3×3 matrix, by Jacobi rotations.
fn symmetric_eigen(mut a: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let (mut p, mut q, mut largest) = (0, 1, 0.0);
        for (i, j) in [(0, 1), (0, 2), (1, 2)] {
            if a[i][j].abs() > largest {
                (p, q, largest) = (i, j, a[i][j].abs());
            }
        }
        let scale = a[0][0].abs() + a[1][1].abs() + a[2][2].abs();
        if largest <= 1e-15 * scale.max(f64::MIN_POSITIVE) {
            break;
        }
        let theta = 0.5 * (2.0 * a[p][q]).atan2(a[q][q] - a[p][p]);
        let (sin, cos) = theta.sin_cos();
        // a = Jᵀ a J with the rotation J in the plane p, q.
        for row in &mut a {
            let (akp, akq) = (row[p], row[q]);
            row[p] = cos * akp - sin * akq;
            row[q] = sin * akp + cos * akq;
        }
        let (row_p, row_q) = (a[p], a[q]);
        a[p] = [0, 1, 2].map(|k| cos * row_p[k] - sin * row_q[k]);
        a[q] = [0, 1, 2].map(|k| sin * row_p[k] + cos * row_q[k]);
        for row in &mut v {
            let (vp, vq) = (row[p], row[q]);
            row[p] = cos * vp - sin * vq;
            row[q] = sin * vp + cos * vq;
        }
    }
    let vectors = [0, 1, 2].map(|k| [v[0][k], v[1][k], v[2][k]]);
    ([a[0][0], a[1][1], a[2][2]], vectors)
}

/// The solution of a 3×3 linear system by Cramer's rule; `None` when it is singular.
fn solve3(a: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let det = |m: [[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let d = det(a);
    if d.abs() < 1e-300 {
        return None;
    }
    Some([0, 1, 2].map(|k| {
        let mut m = a;
        for i in 0..3 {
            m[i][k] = b[i];
        }
        det(m) / d
    }))
}
