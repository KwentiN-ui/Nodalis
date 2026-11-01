//! Restart and state links: the model fingerprint, the binary form of a [`Snapshot`], and
//! the restart file of the command line (`*RESTART, WRITE` into `<job>.rout`, `*RESTART,
//! READ` from `<job>.rin`).
//!
//! In PrePoLix (in-process, KONZEPT_CCX_ABLOESUNG 3.5) a snapshot is taken after every step
//! and kept in memory or as `<job>.s<n>.state` ([`encode_snapshot`]); the next run resumes
//! from the snapshot of the last unchanged step. The command line keeps CalculiX's form: a
//! restart input has no model definition, so the `.rout` file holds the model and the steps
//! up to the snapshots as well as the snapshots themselves.
//!
//! The format is plxs' own (postcard, behind a magic and a format number), not CalculiX's;
//! a file of another format number is refused, not converted.

use std::hash::Hasher;
use std::path::Path;

use plxs_core::state::Snapshot;
use plxs_core::{Analysis, Error, Model, Result};
use serde::{Deserialize, Serialize};

const SNAPSHOT_MAGIC: &[u8; 8] = b"PLXSSNAP";
const RESTART_MAGIC: &[u8; 8] = b"PLXSROUT";
/// Raised whenever [`plxs_core::State`], [`Snapshot`] or the IR change their layout.
const FORMAT: u32 = 1;

/// FNV-1a, 64 bit: stable across Rust versions and processes, unlike `DefaultHasher`.
struct Fnv(u64);

impl Fnv {
    fn new() -> Self {
        Fnv(0xcbf2_9ce4_8422_2325)
    }
}

impl Hasher for Fnv {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 ^= u64::from(b);
            self.0 = self.0.wrapping_mul(0x0100_0000_01b3);
        }
    }
}

impl std::io::Write for Fnv {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        Hasher::write(self, bytes);
        Ok(bytes.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

/// The fingerprint of a model a snapshot belongs to: nodes, elements, materials, sections,
/// constraints, contact, initial conditions and physical constants. Sets, surfaces,
/// amplitudes and the heading are left out, so a continuing model may add sets and
/// amplitudes for its own steps; everything the state's layout or values depend on is in.
pub fn model_fingerprint(model: &Model) -> u64 {
    fn part(h: &mut Fnv, value: &impl Serialize) {
        // Writing into the hasher cannot fail; a serialization error would be a bug of the
        // IR and only changes the hash.
        let _ = postcard::to_io(value, &mut *h);
    }
    let mut h = Fnv::new();
    part(&mut h, &model.nodes);
    part(&mut h, &model.elements);
    part(&mut h, &model.materials);
    part(&mut h, &model.sections);
    part(&mut h, &model.constraints);
    part(&mut h, &model.contact);
    part(&mut h, &model.initial);
    part(&mut h, &model.physical);
    h.finish()
}

fn format_error(what: &str, message: impl std::fmt::Display) -> Error {
    Error::Unsupported(format!("{what}: {message}"))
}

fn encode<T: Serialize>(magic: &[u8; 8], value: &T) -> Result<Vec<u8>> {
    let mut out = magic.to_vec();
    out.extend(FORMAT.to_le_bytes());
    postcard::to_extend(value, out).map_err(|e| format_error("restart data", e))
}

fn decode<T: for<'de> Deserialize<'de>>(magic: &[u8; 8], what: &str, bytes: &[u8]) -> Result<T> {
    let Some(rest) = bytes.strip_prefix(magic.as_slice()) else {
        return Err(format_error(what, "not a plxs file"));
    };
    let (format, body) = rest
        .split_at_checked(4)
        .ok_or_else(|| format_error(what, "cut off"))?;
    let format = u32::from_le_bytes(format.try_into().unwrap_or_default());
    if format != FORMAT {
        return Err(format_error(
            what,
            format!("written by another solver version (format {format}, this one reads {FORMAT})"),
        ));
    }
    postcard::from_bytes(body).map_err(|e| format_error(what, e))
}

/// A snapshot as bytes, for `<job>.s<n>.state` files and bound results.
pub fn encode_snapshot(snapshot: &Snapshot) -> Result<Vec<u8>> {
    encode(SNAPSHOT_MAGIC, snapshot)
}

pub fn decode_snapshot(bytes: &[u8]) -> Result<Snapshot> {
    decode(SNAPSHOT_MAGIC, "snapshot", bytes)
}

/// The restart file of the command line: the analysis read so far (model and the steps up
/// to the last snapshot, before shells and beams are expanded) and the states after the
/// steps that asked for `*RESTART, WRITE`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RestartFile {
    pub analysis: Analysis,
    pub snapshots: Vec<Snapshot>,
}

impl RestartFile {
    /// Adds the state after a step; with `overlay` (`*RESTART, WRITE, OVERLAY`) only the
    /// latest is kept.
    pub fn push(&mut self, snapshot: Snapshot, overlay: bool) {
        if overlay {
            self.snapshots.clear();
        }
        self.snapshots
            .retain(|s| s.state.steps_done != snapshot.state.steps_done);
        self.snapshots.push(snapshot);
    }

    /// The snapshot after step `step` (absolute number, counted from 1), or the last one.
    pub fn after(&self, step: Option<u32>) -> Option<&Snapshot> {
        match step {
            Some(step) => self.snapshots.iter().find(|s| s.state.steps_done == step),
            None => self.snapshots.iter().max_by_key(|s| s.state.steps_done),
        }
    }

    /// The analysis up to and including step `step`: what a continuing input goes on from.
    pub fn base(&self, step: u32) -> Analysis {
        let mut analysis = self.analysis.clone();
        analysis.steps.truncate(step as usize);
        analysis.model.restart_read = None;
        analysis.model.rebuild_indices();
        analysis
    }

    pub fn write(&self, path: &Path) -> Result<()> {
        let bytes = encode(RESTART_MAGIC, self)?;
        std::fs::write(path, bytes).map_err(|e| Error::Io {
            path: path.display().to_string(),
            message: e.to_string(),
        })
    }

    pub fn read(path: &Path) -> Result<RestartFile> {
        let bytes = std::fs::read(path).map_err(|e| Error::Io {
            path: path.display().to_string(),
            message: e.to_string(),
        })?;
        let mut file: RestartFile = decode(RESTART_MAGIC, "restart file", &bytes)?;
        file.analysis.model.rebuild_indices();
        Ok(file)
    }
}
