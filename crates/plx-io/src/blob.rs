//! Binary content of a project, such as the results it keeps: held in memory, in a file of
//! its own, or in the binary part of the project file it was read from, and read from there
//! when it is wanted instead of when the project is opened.
//!
//! A project file is its RON text followed by the binary content ([`crate::project`]); in the
//! text, a [`Blob`] is where it is in the binary part. [`write_project`] collects the blobs
//! while the text is written and puts them behind it, [`read_project`] gives the blobs of the
//! text their place in the file. Once a project is written, its blobs are read from the new
//! file: what was held in memory is let go.
//!
//! [`write_project`]: crate::project::save_project
//! [`read_project`]: crate::project::read_project

use std::cell::RefCell;
use std::collections::HashMap;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::Path;
use std::sync::{Arc, Mutex};

use serde::{Deserialize, Serialize};

/// Binary content, shared by its copies: a project copies its analyses to save or solve them
/// without copying their results.
#[derive(Clone, Default)]
pub struct Blob(Arc<Inner>);

#[derive(Default)]
struct Inner {
    len: u64,
    /// Where the content is; moves into the project file once it is written.
    at: Mutex<Storage>,
}

#[derive(Clone)]
enum Storage {
    Memory(Arc<[u8]>),
    /// `len` bytes from `offset` in a file, read where they are wanted.
    File {
        file: Arc<File>,
        offset: u64,
    },
}

impl Default for Storage {
    fn default() -> Self {
        Storage::Memory(Arc::from([]))
    }
}

/// Bytes read and written at a time when content is copied between files.
const COPY_CHUNK: usize = 8 << 20;

impl Blob {
    /// Content held in memory.
    pub fn from_bytes(bytes: Vec<u8>) -> Self {
        Self(Arc::new(Inner {
            len: bytes.len() as u64,
            at: Mutex::new(Storage::Memory(bytes.into())),
        }))
    }

    /// The content of the file at `path`, copied into a temporary file of its own that is
    /// gone once the content is not needed anymore: the file at `path` may change or go,
    /// and the content does not take memory.
    pub fn copy_of_file(path: &Path) -> std::io::Result<Self> {
        let mut source = File::open(path)?;
        let mut copy = anonymous_file()?;
        let len = std::io::copy(&mut source, &mut copy)?;
        Ok(Self::in_file(Arc::new(copy), 0, len))
    }

    fn in_file(file: Arc<File>, offset: u64, len: u64) -> Self {
        Self(Arc::new(Inner {
            len,
            at: Mutex::new(Storage::File { file, offset }),
        }))
    }

    pub fn len(&self) -> u64 {
        self.0.len
    }

    pub fn is_empty(&self) -> bool {
        self.0.len == 0
    }

    /// Whether the content is held in memory, not read from a file.
    pub fn in_memory(&self) -> bool {
        matches!(self.storage(), Storage::Memory(_))
    }

    fn storage(&self) -> Storage {
        self.0.at.lock().unwrap_or_else(|e| e.into_inner()).clone()
    }

    /// The whole content.
    pub fn bytes(&self) -> std::io::Result<Vec<u8>> {
        let mut bytes = vec![0; self.0.len as usize];
        self.read_at(0, &mut bytes)?;
        Ok(bytes)
    }

    /// Reads `buf.len()` bytes from `at` in the content.
    pub fn read_at(&self, at: u64, buf: &mut [u8]) -> std::io::Result<()> {
        let end = at.checked_add(buf.len() as u64);
        if end.is_none_or(|end| end > self.0.len) {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "read beyond the end of the content",
            ));
        }
        match self.storage() {
            Storage::Memory(bytes) => {
                buf.copy_from_slice(&bytes[at as usize..at as usize + buf.len()]);
                Ok(())
            }
            Storage::File { file, offset } => read_exact_at(&file, offset + at, buf),
        }
    }

    /// Reads the content from its start, independently of other readers.
    pub fn reader(&self) -> BlobReader {
        BlobReader {
            blob: self.clone(),
            at: 0,
        }
    }

    /// Writes the content to `out`, a part at a time.
    pub fn write_to(&self, out: &mut impl Write) -> std::io::Result<()> {
        match self.storage() {
            Storage::Memory(bytes) => out.write_all(&bytes),
            Storage::File { file, offset } => {
                let mut buf = vec![0; COPY_CHUNK.min(self.0.len as usize)];
                let mut done = 0;
                while done < self.0.len {
                    let n = (self.0.len - done).min(buf.len() as u64) as usize;
                    read_exact_at(&file, offset + done, &mut buf[..n])?;
                    out.write_all(&buf[..n])?;
                    done += n as u64;
                }
                Ok(())
            }
        }
    }

    /// The content is now at `offset` in `file`, which has the same content there.
    fn moved(&self, file: Arc<File>, offset: u64) {
        *self.0.at.lock().unwrap_or_else(|e| e.into_inner()) = Storage::File { file, offset };
    }

    fn same(&self, other: &Self) -> bool {
        if Arc::ptr_eq(&self.0, &other.0) {
            return true;
        }
        match (self.storage(), other.storage()) {
            (Storage::Memory(a), Storage::Memory(b)) => Arc::ptr_eq(&a, &b),
            (Storage::File { file: a, offset: x }, Storage::File { file: b, offset: y }) => {
                Arc::ptr_eq(&a, &b) && x == y
            }
            _ => false,
        }
    }
}

impl PartialEq for Blob {
    fn eq(&self, other: &Self) -> bool {
        if self.0.len != other.0.len {
            return false;
        }
        if self.same(other) {
            return true;
        }
        // Compared a part at a time; content that cannot be read is not equal.
        let chunk = COPY_CHUNK.min(self.0.len as usize);
        let (mut a, mut b) = (vec![0; chunk], vec![0; chunk]);
        let mut done = 0;
        while done < self.0.len {
            let n = (self.0.len - done).min(chunk as u64) as usize;
            let read =
                self.read_at(done, &mut a[..n]).is_ok() && other.read_at(done, &mut b[..n]).is_ok();
            if !read || a[..n] != b[..n] {
                return false;
            }
            done += n as u64;
        }
        true
    }
}

impl Eq for Blob {}

impl std::fmt::Debug for Blob {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let place = if self.in_memory() { "memory" } else { "file" };
        write!(f, "Blob({} bytes in {place})", self.0.len)
    }
}

/// Reads a [`Blob`] like a file.
pub struct BlobReader {
    blob: Blob,
    at: u64,
}

impl Read for BlobReader {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let n = (self.blob.len().saturating_sub(self.at)).min(buf.len() as u64) as usize;
        self.blob.read_at(self.at, &mut buf[..n])?;
        self.at += n as u64;
        Ok(n)
    }
}

impl Seek for BlobReader {
    fn seek(&mut self, to: SeekFrom) -> std::io::Result<u64> {
        let at = match to {
            SeekFrom::Start(at) => Some(at),
            SeekFrom::End(by) => self.blob.len().checked_add_signed(by),
            SeekFrom::Current(by) => self.at.checked_add_signed(by),
        };
        let at = at.ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::InvalidInput, "seek before the start")
        })?;
        self.at = at;
        Ok(at)
    }
}

#[cfg(unix)]
fn read_exact_at(file: &File, at: u64, buf: &mut [u8]) -> std::io::Result<()> {
    std::os::unix::fs::FileExt::read_exact_at(file, buf, at)
}

#[cfg(windows)]
fn read_exact_at(file: &File, mut at: u64, mut buf: &mut [u8]) -> std::io::Result<()> {
    // Moves the cursor of the file, which nothing else uses.
    while !buf.is_empty() {
        match std::os::windows::fs::FileExt::seek_read(file, buf, at) {
            Ok(0) => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "file ends before the content",
                ));
            }
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

/// A new temporary file without a name, deleted with its last handle.
fn anonymous_file() -> std::io::Result<File> {
    use std::sync::atomic::{AtomicU64, Ordering};
    static COUNT: AtomicU64 = AtomicU64::new(0);
    let dir = std::env::temp_dir();
    loop {
        let path = dir.join(format!(
            "nodalis-{}-{}.blob",
            std::process::id(),
            COUNT.fetch_add(1, Ordering::Relaxed)
        ));
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .open(&path);
        match file {
            Ok(file) => {
                // Gone once it is closed; open files can be removed on Windows too, as Rust
                // opens them for sharing deletion.
                let _ = std::fs::remove_file(&path);
                return Ok(file);
            }
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {}
            Err(e) => return Err(e),
        }
    }
}

/// Where a blob is in the binary part of a project file, as the text has it.
#[derive(Serialize, Deserialize)]
struct Location {
    at: u64,
    len: u64,
}

/// The blobs a project being written has, in the order of their places.
#[derive(Default)]
struct Writing {
    blobs: Vec<(Blob, u64)>,
    places: HashMap<*const Inner, u64>,
    end: u64,
}

/// The binary part of the project file being read.
struct Reading {
    file: Arc<File>,
    /// Where the binary part starts in the file, and its length.
    start: u64,
    len: u64,
}

thread_local! {
    static WRITING: RefCell<Option<Writing>> = const { RefCell::new(None) };
    static READING: RefCell<Option<Reading>> = const { RefCell::new(None) };
}

impl Serialize for Blob {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let at = WRITING.with_borrow_mut(|writing| {
            let writing = writing.as_mut()?;
            let key = Arc::as_ptr(&self.0);
            Some(*writing.places.entry(key).or_insert_with(|| {
                let at = writing.end;
                writing.blobs.push((self.clone(), at));
                writing.end += self.0.len;
                at
            }))
        });
        let at = at.ok_or_else(|| {
            serde::ser::Error::custom("binary content is written with the project file only")
        })?;
        Location {
            at,
            len: self.0.len,
        }
        .serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Blob {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let Location { at, len } = Location::deserialize(deserializer)?;
        READING.with_borrow(|reading| {
            let reading = reading.as_ref().ok_or_else(|| {
                serde::de::Error::custom("binary content is read with the project file only")
            })?;
            if at.checked_add(len).is_none_or(|end| end > reading.len) {
                return Err(serde::de::Error::custom(format!(
                    "binary content at {at} of {len} bytes is beyond the end of the file"
                )));
            }
            Ok(Blob::in_file(reading.file.clone(), reading.start + at, len))
        })
    }
}

/// Starts of a project file: the text follows, then the binary part.
pub(crate) const MAGIC: &[u8; 8] = b"NODALIS\x01";

/// Writes `value` as a project file to `out`: [`MAGIC`], the length of the text (u64, little
/// endian), the RON text, the blobs. Returns the blobs and where each one is in `out`, to
/// read them from there once the file is in place, see [`moved_into`].
pub(crate) fn write_file<T: Serialize>(
    value: &T,
    out: &mut impl Write,
) -> Result<Vec<(Blob, u64)>, String> {
    WRITING.set(Some(Writing::default()));
    let text = ron::to_string(value);
    let writing = WRITING.take().unwrap_or_default();
    let text = text.map_err(|e| e.to_string())?;
    let start = (MAGIC.len() + 8 + text.len()) as u64;
    let io = |e: std::io::Error| e.to_string();
    out.write_all(MAGIC).map_err(io)?;
    out.write_all(&(text.len() as u64).to_le_bytes())
        .map_err(io)?;
    out.write_all(text.as_bytes()).map_err(io)?;
    for (blob, _) in &writing.blobs {
        blob.write_to(out).map_err(io)?;
    }
    Ok((writing.blobs.into_iter())
        .map(|(blob, at)| (blob, start + at))
        .collect())
}

/// Reads the blobs written by [`write_file`] from `file` from now on.
pub(crate) fn moved_into(file: Arc<File>, blobs: Vec<(Blob, u64)>) {
    for (blob, offset) in blobs {
        blob.moved(file.clone(), offset);
    }
}

/// Reads a project file: its text as `T`, with its blobs read from `file` when they are
/// wanted. A file without [`MAGIC`] is RON text alone. `parse` gets the text.
pub(crate) fn read_file<T>(
    file: File,
    parse: impl FnOnce(&str) -> Result<T, String>,
) -> Result<T, String> {
    let mut file = file;
    let io = |e: std::io::Error| e.to_string();
    let len = file.seek(SeekFrom::End(0)).map_err(io)?;
    file.seek(SeekFrom::Start(0)).map_err(io)?;
    let mut magic = [0; 8];
    let has_magic = len >= 16 && {
        file.read_exact(&mut magic).map_err(io)?;
        &magic == MAGIC
    };
    if !has_magic {
        file.seek(SeekFrom::Start(0)).map_err(io)?;
        let mut text = String::new();
        file.read_to_string(&mut text).map_err(io)?;
        return parse(&text);
    }
    let mut text_len = [0; 8];
    file.read_exact(&mut text_len).map_err(io)?;
    let text_len = u64::from_le_bytes(text_len);
    let start = 16u64.checked_add(text_len).filter(|&s| s <= len);
    let start = start.ok_or("the text is longer than the file")?;
    let mut text = vec![0; text_len as usize];
    file.read_exact(&mut text).map_err(io)?;
    let text = String::from_utf8(text).map_err(|e| e.to_string())?;
    READING.set(Some(Reading {
        file: Arc::new(file),
        start,
        len: len - start,
    }));
    let parsed = parse(&text);
    READING.set(None);
    parsed
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Serialize, Deserialize, PartialEq, Debug)]
    struct Holder {
        name: String,
        blobs: Vec<Blob>,
    }

    fn temp(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("nodalis-blob-{}-{name}", std::process::id()))
    }

    #[test]
    fn blobs_go_behind_the_text_and_are_read_from_the_file() {
        let shared = Blob::from_bytes(vec![7; 100]);
        let holder = Holder {
            name: "x".into(),
            blobs: vec![
                Blob::from_bytes(b"hello".to_vec()),
                shared.clone(),
                shared.clone(),
            ],
        };
        let path = temp("roundtrip");
        let mut out = File::create(&path).unwrap();
        let blobs = write_file(&holder, &mut out).unwrap();
        // The same content once.
        assert_eq!(blobs.len(), 2);
        drop(out);
        let file = File::open(&path).unwrap();
        let read: Holder =
            read_file(file, |text| ron::from_str(text).map_err(|e| e.to_string())).unwrap();
        assert_eq!(read, holder);
        assert!(read.blobs.iter().all(|b| !b.in_memory()));
        assert_eq!(read.blobs[0].bytes().unwrap(), b"hello");
        let mut tail = [0; 3];
        read.blobs[1].read_at(97, &mut tail).unwrap();
        assert_eq!(tail, [7; 3]);
        assert!(read.blobs[1].read_at(98, &mut tail).is_err());

        // Once written, the blobs read from the file and the memory is let go.
        moved_into(Arc::new(File::open(&path).unwrap()), blobs);
        assert!(!shared.in_memory());
        assert_eq!(shared.bytes().unwrap(), vec![7; 100]);
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn plain_text_is_read_without_blobs_and_blobs_need_a_file() {
        let path = temp("plain");
        std::fs::write(&path, "(name: \"y\", blobs: [])").unwrap();
        let read: Holder = read_file(File::open(&path).unwrap(), |text| {
            ron::from_str(text).map_err(|e| e.to_string())
        })
        .unwrap();
        assert_eq!(read.name, "y");
        std::fs::remove_file(&path).unwrap();
        assert!(ron::to_string(&Blob::from_bytes(vec![1])).is_err());
    }

    #[test]
    fn a_copy_of_a_file_outlives_the_file_and_reads_like_one() {
        let path = temp("copied");
        let content: Vec<u8> = (0..=255).cycle().take(10_000).collect();
        std::fs::write(&path, &content).unwrap();
        let blob = Blob::copy_of_file(&path).unwrap();
        std::fs::write(&path, b"changed").unwrap();
        std::fs::remove_file(&path).unwrap();
        assert_eq!(blob.bytes().unwrap(), content);
        let mut reader = blob.reader();
        reader.seek(SeekFrom::Start(9_990)).unwrap();
        let mut rest = Vec::new();
        reader.read_to_end(&mut rest).unwrap();
        assert_eq!(rest, content[9_990..]);
        assert_eq!(blob, Blob::from_bytes(content));
    }
}
