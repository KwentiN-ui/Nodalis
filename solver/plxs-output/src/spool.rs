//! The increments of a run kept on disk while it computes the next ones ([`Spool`]), so
//! that the results of a long run need not fit into memory. The increments are packed as
//! the arrays of a [`Streaming`] container that is never finished: the results file is
//! written from it at the end, copying the compressed arrays as they are.

use std::fs::File;
use std::path::{Path, PathBuf};

use std::io::{Seek, SeekFrom};

use plxs_core::results::ResultsIncrement;

use crate::packed::{Array, RawDataset, Streaming};

/// The magic of a spool file, which is never read as a container.
const MAGIC: &[u8; 8] = b"PLXSPOOL";

/// An increment in a [`Spool`]: what it is without its arrays, and the datasets of its
/// arrays.
#[derive(Clone, Debug)]
pub struct Spooled {
    /// The increment with the nodes and the columns of its fields empty (the number of
    /// columns stays).
    pub skeleton: ResultsIncrement,
    /// The datasets of its arrays, in the order of [`crate::packed::Packable::arrays`]:
    /// per field its nodes, then its columns.
    pub arrays: Vec<usize>,
}

/// Increments of a run kept in a file of their own, deleted with the spool.
pub struct Spool {
    writer: Streaming<File>,
    path: PathBuf,
    increments: Vec<Spooled>,
    /// The first error writing, which the run reports at the end.
    error: Option<String>,
}

impl std::fmt::Debug for Spool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Spool")
            .field("path", &self.path)
            .field("increments", &self.increments.len())
            .finish_non_exhaustive()
    }
}

impl Spool {
    /// A new spool in the file `path`, which it replaces.
    pub fn create(path: &Path) -> std::io::Result<Self> {
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(path)?;
        Ok(Self {
            writer: Streaming::new(MAGIC, file)?,
            path: path.to_owned(),
            increments: Vec::new(),
            error: None,
        })
    }

    /// Writes `increment` to the file and keeps only what it is without its arrays.
    pub fn push(&mut self, mut increment: ResultsIncrement) -> std::io::Result<()> {
        self.seek_end()?;
        let mut arrays = Vec::new();
        for field in &mut increment.fields {
            arrays.push(self.writer.array(Array::Ids(&mut field.nodes))?);
            field.nodes = Vec::new();
            for column in &mut field.values {
                arrays.push(self.writer.array(Array::F64(column))?);
                *column = Vec::new();
            }
        }
        self.increments.push(Spooled {
            skeleton: increment,
            arrays,
        });
        Ok(())
    }

    /// Adds an increment whose arrays are datasets of another container, `raw` in the
    /// order of [`Spooled::arrays`], copied as they are stored.
    pub fn push_raw(
        &mut self,
        skeleton: ResultsIncrement,
        raw: &[RawDataset],
    ) -> std::io::Result<()> {
        self.seek_end()?;
        let arrays = (raw.iter())
            .map(|r| self.writer.raw(r))
            .collect::<std::io::Result<Vec<usize>>>()?;
        self.increments.push(Spooled { skeleton, arrays });
        Ok(())
    }

    /// Reading back moves the cursor of the file on Windows: writing goes on at its end.
    fn seek_end(&mut self) -> std::io::Result<()> {
        self.writer.get_mut().seek(SeekFrom::End(0)).map(|_| ())
    }

    /// The increments, in the order they came.
    pub fn increments(&self) -> &[Spooled] {
        &self.increments
    }

    pub fn len(&self) -> usize {
        self.increments.len()
    }

    pub fn is_empty(&self) -> bool {
        self.increments.is_empty()
    }

    /// Keeps the increments for which `keep` is true.
    pub fn retain(&mut self, keep: impl Fn(&ResultsIncrement) -> bool) {
        self.increments.retain(|i| keep(&i.skeleton));
    }

    /// The first error writing an increment handed over by [`plxs_core::Spill`].
    pub fn error(&self) -> Option<&str> {
        self.error.as_deref()
    }

    /// Dataset `index` as it is stored.
    pub fn raw(&self, index: usize) -> std::io::Result<RawDataset> {
        let file = self.writer.get_ref();
        self.writer
            .stored(index, |at, buf| read_exact_at(file, at, buf))
    }

    /// Increment `index` with its arrays read back.
    pub fn read(&self, index: usize) -> std::io::Result<ResultsIncrement> {
        let spooled = &self.increments[index];
        let mut increment = spooled.skeleton.clone();
        let mut arrays = spooled.arrays.iter();
        let mut next = |array: Array<'_>| -> std::io::Result<()> {
            let index = *arrays
                .next()
                .ok_or_else(|| invalid("an array is missing"))?;
            let raw = self.raw(index)?;
            raw.decode(array).map_err(|e| invalid(&e.0))
        };
        for field in &mut increment.fields {
            next(Array::Ids(&mut field.nodes))?;
            for column in &mut field.values {
                next(Array::F64(column))?;
            }
        }
        Ok(increment)
    }
}

impl plxs_core::Spill for Spool {
    fn spill(&mut self, increment: ResultsIncrement) {
        if self.error.is_some() {
            return;
        }
        if let Err(e) = self.push(increment) {
            self.error = Some(format!("{}: {e}", self.path.display()));
        }
    }
}

impl Drop for Spool {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.path);
    }
}

fn invalid(message: &str) -> std::io::Error {
    std::io::Error::new(std::io::ErrorKind::InvalidData, message.to_owned())
}

#[cfg(unix)]
fn read_exact_at(file: &File, at: u64, buf: &mut [u8]) -> std::io::Result<()> {
    std::os::unix::fs::FileExt::read_exact_at(file, buf, at)
}

#[cfg(windows)]
fn read_exact_at(file: &File, mut at: u64, mut buf: &mut [u8]) -> std::io::Result<()> {
    // Moves the cursor of the file: the writer seeks back to the end before it writes.
    while !buf.is_empty() {
        match std::os::windows::fs::FileExt::seek_read(file, buf, at) {
            Ok(0) => return Err(invalid("the spool file ends early")),
            Ok(n) => {
                buf = &mut buf[n..];
                at += n as u64;
            }
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e),
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use plxs_core::Spill;
    use plxs_core::results::{IncrementKind, ResultField};

    use super::*;

    fn increment(step: u32, scale: f64) -> ResultsIncrement {
        ResultsIncrement {
            step,
            increment: 1,
            kind: IncrementKind::Time,
            value: scale,
            total_time: scale,
            nodal_diameter: None,
            fields: vec![ResultField {
                quantity: plxs_core::results::FieldQuantity::Displacements,
                imaginary: false,
                components: vec!["D1".into(), "D2".into()],
                nodes: (1..=200_000).collect(),
                values: (0..2)
                    .map(|k| (0..200_000).map(|i| (i + k) as f64 * scale).collect())
                    .collect(),
            }],
        }
    }

    #[test]
    fn spooled_increments_read_back_and_the_file_goes_with_the_spool() {
        let path = std::env::temp_dir().join(format!("plxs-spool-{}", std::process::id()));
        let mut spool = Spool::create(&path).unwrap();
        spool.spill(increment(1, 1.0));
        spool.spill(increment(2, 0.5));
        assert_eq!(spool.len(), 2);
        assert!(spool.increments()[1].skeleton.fields[0].nodes.is_empty());
        assert_eq!(spool.read(1).unwrap(), increment(2, 0.5));
        // A copy of the stored arrays of one increment, as from the step cache.
        let raw: Vec<RawDataset> = (spool.increments()[0].arrays.iter())
            .map(|&a| spool.raw(a).unwrap())
            .collect();
        let skeleton = spool.increments()[0].skeleton.clone();
        spool.push_raw(skeleton, &raw).unwrap();
        assert_eq!(spool.read(2).unwrap(), increment(1, 1.0));
        spool.retain(|i| i.step == 2);
        assert_eq!(spool.len(), 1);
        assert!(spool.error().is_none());
        drop(spool);
        assert!(!path.exists());
    }
}
