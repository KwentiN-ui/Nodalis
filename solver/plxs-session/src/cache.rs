//! The step cache: a run keeps the state and the results after each step it computed, under
//! a key of everything that step's results depend on, and the next run takes over the
//! longest unchanged start of its step chain instead of computing it again.
//!
//! The key of step `k` is a hash of what the solver is given for the steps `1..=k`: the
//! model (nodes, elements, materials, sections, constraints, contact, initial conditions,
//! physical constants), the sets, surfaces and amplitudes these and the steps refer to (by
//! their definitions, only where they are referred to), the files it reads (stored modes, the global
//! results of a submodel, temperatures of a step) by their content, the steps up to `k`
//! themselves and the solver build ([`StepCache::salt`]). Nothing else is known about the
//! steps: adding a step after them, or changing a later one, leaves their keys as they
//! were; a change of anything they depend on changes them. A run continues from the state
//! of the last step it took over ([`crate::Session::resume`]), which gives the same results
//! as the run from the start.
//!
//! The cache is a folder of files `<key>.state` (the [`Snapshot`] after the step) and
//! `<key>.results` (the step's increments and history rows). It keeps the steps of the
//! last run only: what another run left there is removed when a run starts.

use std::hash::Hasher;
use std::path::{Path, PathBuf};

use plxs_core::state::Snapshot;
use plxs_core::step::DefinedFieldKind;
use plxs_core::{Analysis, Results};
use serde::Serialize;

use crate::restart::{self, Fnv};

const RESULTS_MAGIC: &[u8; 8] = b"PLXSSTP2";

/// Where the cache of a job is and which solver build it belongs to.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StepCache {
    /// The cache folder; created when the first step is kept.
    pub dir: PathBuf,
    /// The solver build: a cache of another build is not used (its results could differ).
    pub salt: u64,
}

/// What a run takes over from the cache.
pub(crate) struct Cached {
    /// The number of steps taken over.
    pub steps: usize,
    pub snapshot: Snapshot,
    /// The results of these steps.
    pub results: Results,
}

impl StepCache {
    /// The key of each step of `analysis` (before shells and beams are expanded), for a job
    /// in `work_dir`, where the files it reads are.
    pub fn keys(&self, analysis: &Analysis, work_dir: &Path) -> Vec<u64> {
        let model = &analysis.model;
        let named = Named::of(model);
        let mut h = Fnv::new();
        h.write_u64(self.salt);
        part(&mut h, &model.nodes);
        part(&mut h, &model.elements);
        // What refers to sets, surfaces and amplitudes by name; those it names count by
        // their definitions.
        let definitions = bytes(&(
            &model.materials,
            &model.sections,
            &model.constraints,
            &model.contact,
            &model.initial,
            &model.physical,
            &model.design_variables,
            &model.restart_read,
        ));
        named.referenced(&mut h, &definitions);
        file(&mut h, model.stored_modes_file.as_deref());
        if let Some(submodel) = &model.constraints.submodel {
            file(&mut h, Some(&work_dir.join(&submodel.input)));
        }
        let mut key = h.finish();
        let mut keys = Vec::with_capacity(analysis.steps.len());
        for step in &analysis.steps {
            let mut h = Fnv::new();
            h.write_u64(key);
            named.referenced(&mut h, &bytes(step));
            for field in &step.fields {
                if let DefinedFieldKind::TemperatureFile { file: name, .. } = &field.kind {
                    file(&mut h, Some(&work_dir.join(name)));
                }
            }
            key = h.finish();
            keys.push(key);
        }
        keys
    }

    fn path(&self, key: u64, extension: &str) -> PathBuf {
        self.dir.join(format!("{key:016x}.{extension}"))
    }

    /// Removes what does not belong to one of `keys` (the steps of another run).
    pub(crate) fn keep_only(&self, keys: &[u64]) {
        let Ok(entries) = std::fs::read_dir(&self.dir) else {
            return;
        };
        let wanted: Vec<String> = keys.iter().map(|k| format!("{k:016x}")).collect();
        for entry in entries.flatten() {
            let path = entry.path();
            let stem = path.file_stem().map(|s| s.to_string_lossy().into_owned());
            let ours = matches!(
                path.extension().and_then(|e| e.to_str()),
                Some("state" | "results" | "tmp")
            );
            if ours && !stem.is_some_and(|s| wanted.contains(&s)) {
                let _ = std::fs::remove_file(path);
            }
        }
    }

    /// The longest start of the step chain `keys` that is kept, with the state after its
    /// last step and the results of its steps; `None` when not even the first step is
    /// kept or a file cannot be read.
    pub(crate) fn take(&self, keys: &[u64]) -> Option<Cached> {
        let steps = keys
            .iter()
            .take_while(|&&k| self.path(k, "state").is_file() && self.path(k, "results").is_file())
            .count();
        if steps == 0 {
            return None;
        }
        let mut results = Results::default();
        for &key in &keys[..steps] {
            let bytes = std::fs::read(self.path(key, "results")).ok()?;
            let step: Results = plxs_output::packed::unpack(RESULTS_MAGIC, &bytes).ok()?;
            results.increments.extend(step.increments);
            results.history.extend(step.history);
            results.modes.extend(step.modes);
        }
        let bytes = std::fs::read(self.path(keys[steps - 1], "state")).ok()?;
        let snapshot = restart::decode_snapshot(&bytes).ok()?;
        (snapshot.state.steps_done as usize == steps).then_some(Cached {
            steps,
            snapshot,
            results,
        })
    }

    /// Keeps the state after step `step` (counted from 1) under `key`, with the results of
    /// that step out of `results`. A file that cannot be written is left out: the cache only
    /// saves time.
    pub(crate) fn keep(&self, key: u64, step: u32, snapshot: &Snapshot, results: &Results) {
        let of_step = Results {
            increments: (results.increments.iter())
                .filter(|i| i.step == step)
                .cloned()
                .collect(),
            history: (results.history.iter())
                .filter(|h| h.step == step)
                .cloned()
                .collect(),
            modes: (results.modes.iter())
                .filter(|m| m.step == step)
                .cloned()
                .collect(),
        };
        let write = |extension: &str, bytes: Vec<u8>| -> std::io::Result<()> {
            // Written beside and renamed, so that a stopped run leaves no half file.
            let path = self.path(key, extension);
            let partial = self.path(key, &format!("{extension}.tmp"));
            std::fs::write(&partial, bytes)?;
            std::fs::rename(&partial, &path)
        };
        let _ = std::fs::create_dir_all(&self.dir);
        let written = write("results", plxs_output::packed::pack(RESULTS_MAGIC, of_step))
            .ok()
            .and_then(|()| restart::encode_snapshot(snapshot).ok())
            .and_then(|bytes| write("state", bytes).ok());
        if written.is_none() {
            let _ = std::fs::remove_file(self.path(key, "results"));
            let _ = std::fs::remove_file(self.path(key, "state"));
        }
    }
}

fn part(h: &mut Fnv, value: &impl Serialize) {
    // Writing into the hasher cannot fail; a serialization error only changes the hash.
    let _ = postcard::to_io(value, &mut *h);
}

fn bytes(value: &impl Serialize) -> Vec<u8> {
    postcard::to_stdvec(value).unwrap_or_default()
}

/// The sets, surfaces and amplitudes of a model: items the model and the steps refer to by
/// name. Each counts only where it is referred to, so that the sets and amplitudes the
/// steps after a step define (the model holds them all) leave its key as it was.
struct Named {
    /// The name as it is encoded where it is referred to, and the definition.
    items: Vec<(Vec<u8>, Vec<u8>)>,
}

impl Named {
    fn of(model: &plxs_core::Model) -> Self {
        let mut items = Vec::new();
        let mut add = |kind: u8, name: &str, definition: Vec<u8>| {
            let mut definition = definition;
            definition.insert(0, kind);
            items.push((bytes(&name), definition));
        };
        let sets = &model.sets;
        for (name, ids) in &sets.node_sets {
            add(0, name, bytes(ids));
        }
        for (name, ids) in &sets.element_sets {
            add(1, name, bytes(ids));
        }
        for (name, surface) in &sets.surfaces {
            add(2, name, bytes(surface));
        }
        for amplitude in &model.amplitudes {
            add(3, &amplitude.name, bytes(amplitude));
        }
        Named { items }
    }

    /// Hashes `encoded` (an item in postcard encoding) and the definitions of the items it
    /// names. A name is found as a string is encoded; a string that only looks like a name
    /// adds an item that does not matter, which costs a step taken over, never a wrong one.
    fn referenced(&self, h: &mut Fnv, encoded: &[u8]) {
        h.write(encoded);
        for (name, definition) in &self.items {
            if encoded.windows(name.len()).any(|w| w == name.as_slice()) {
                h.write(name);
                h.write(definition);
            }
        }
    }
}

/// A file the solver reads, by its content; a missing one counts as missing.
fn file(h: &mut Fnv, path: Option<&Path>) {
    let Some(path) = path else {
        h.write_u8(0);
        return;
    };
    match std::fs::File::open(path) {
        Ok(mut file) => {
            h.write_u8(1);
            if std::io::copy(&mut file, h).is_err() {
                h.write_u8(2);
            }
        }
        Err(_) => h.write_u8(3),
    }
}
