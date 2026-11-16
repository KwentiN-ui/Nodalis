//! A compact binary container for results, after the layout of HDF5's chunked datasets.
//!
//! A value is stored as its large arrays (node ids, coordinates, field values, mode shapes),
//! each a dataset of its own, and the rest of it (a "skeleton" serialized with postcard,
//! with these arrays left empty) as one more dataset. Every dataset is cut into chunks of
//! [`CHUNK`] bytes, and every chunk is filtered and compressed on its own:
//!
//! - node ids are stored as the differences to the id before (ids mostly ascend by one);
//! - numbers are byte-shuffled: the first bytes of all values of a chunk, then their second
//!   bytes..., so that the sign and exponent bytes, which vary little from node to node,
//!   stand together (HDF5's shuffle filter); rows of three numbers are shuffled by
//!   component too;
//! - the chunk is compressed with Zstandard.
//!
//! Chunks are compressed and decompressed in parallel. Floating-point values are kept
//! exactly: the container is lossless.
//!
//! Layout: the caller's magic (8 bytes), the offset of the table from the start (u64,
//! little endian), the compressed chunks of every dataset back to back, then the table
//! ([`Table`], postcard) at the end, which says where each dataset is. A writer streams the
//! chunks out as it compresses them and writes the table last; the datasets need not be in
//! the order of the table ([`Streaming`] writes the arrays of a run as they come and the
//! skeleton last), and bytes no dataset of the table names are not read. All numbers are
//! little endian.

use std::io::{Read, Seek, SeekFrom, Write};

use rayon::prelude::*;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

/// The size of a chunk before compression, rounded down to whole values.
pub const CHUNK: usize = 1 << 20;

/// The Zstandard level: compresses about as well as the highest levels on results (numbers
/// whose low bytes hardly compress), at a fraction of their time.
const LEVEL: i32 = 3;

/// An array of a value that the container stores as a dataset of its own.
pub enum Array<'a> {
    /// Node (or element) ids, mostly ascending.
    Ids(&'a mut Vec<u32>),
    F64(&'a mut Vec<f64>),
    /// Rows of three numbers, such as coordinates.
    F64x3(&'a mut Vec<[f64; 3]>),
    F32x3(&'a mut Vec<[f32; 3]>),
}

/// A value the container stores: its arrays are visited by [`Packable::arrays`], always in
/// the same order, which the reader relies on.
pub trait Packable: Serialize + DeserializeOwned {
    fn arrays(&mut self, visit: &mut dyn FnMut(Array<'_>));
}

/// Why a container could not be read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PackError(pub String);

impl std::fmt::Display for PackError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for PackError {}

/// How a dataset's bytes are stored.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
enum Encoding {
    /// Plain bytes: the skeleton.
    Bytes,
    Ids,
    F64,
    F64x3,
    F32x3,
}

impl Encoding {
    /// The size of one value in bytes, by which a chunk is shuffled: rows of three numbers
    /// are shuffled as one value of their own (all first bytes of x, then of y...).
    fn width(self) -> usize {
        match self {
            Encoding::Bytes => 1,
            Encoding::Ids => 4,
            Encoding::F64 => 8,
            Encoding::F64x3 => 24,
            Encoding::F32x3 => 12,
        }
    }

    /// The size of a chunk: [`CHUNK`] rounded down to whole values.
    fn chunk(self) -> usize {
        CHUNK - CHUNK % self.width()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct Dataset {
    encoding: Encoding,
    /// Size of the dataset in bytes before compression.
    size: u64,
    /// Where its first chunk is, from the start of the container; the chunks follow back
    /// to back.
    at: u64,
    /// Sizes of its chunks after compression.
    chunks: Vec<u32>,
}

impl Dataset {
    /// Its size in the container.
    fn stored(&self) -> u64 {
        self.chunks.iter().map(|&c| u64::from(c)).sum()
    }

    /// Whether its chunks fit its size.
    fn consistent(&self) -> bool {
        let chunk = self.encoding.chunk() as u64;
        self.chunks.len() as u64 == self.size.div_ceil(chunk)
    }
}

/// The table of the datasets at the start of the container.
#[derive(Debug, Serialize, Deserialize)]
struct Table {
    datasets: Vec<Dataset>,
}

/// Stores `value` behind `magic`; its arrays are taken out of it.
pub fn pack<T: Packable>(magic: &[u8; 8], mut value: T) -> Vec<u8> {
    pack_in_place(magic, &mut value)
}

/// [`pack`] of a borrowed value, which keeps its arrays.
pub fn pack_in_place<T: Packable>(magic: &[u8; 8], value: &mut T) -> Vec<u8> {
    let mut out = std::io::Cursor::new(Vec::new());
    // Writing into memory does not fail.
    let _ = pack_to(magic, value, &mut out);
    out.into_inner()
}

/// Writes `value` behind `magic` to `out` and keeps its arrays: every array is encoded,
/// compressed and written as it is visited, so that the memory beyond the value itself is
/// one array's bytes and its compressed chunks (the results of a dynamic step with
/// hundreds of frames are gigabytes, which a copy of them, or holding the whole container,
/// would double).
pub fn pack_to<T: Packable>(
    magic: &[u8; 8],
    value: &mut T,
    out: &mut (impl Write + Seek),
) -> std::io::Result<()> {
    let start = out.stream_position()?;
    out.write_all(magic)?;
    out.write_all(&[0; 8])?;
    // The skeleton first: the value with its arrays taken out, which go back afterwards.
    let mut taken: Vec<Taken> = Vec::new();
    value.arrays(&mut |array| taken.push(Taken::take(array)));
    let skeleton = postcard::to_stdvec(&*value).unwrap_or_default();
    let mut taken = taken.into_iter();
    value.arrays(&mut |array| {
        if let Some(taken) = taken.next() {
            taken.put_back(array);
        }
    });
    let mut datasets = vec![write_dataset(out, start, Encoding::Bytes, &skeleton)?];
    drop(skeleton);
    let mut failed = None;
    value.arrays(&mut |array| {
        if failed.is_some() {
            return;
        }
        let (encoding, bytes) = encode_bytes(&array);
        match write_dataset(out, start, encoding, &bytes) {
            Ok(dataset) => datasets.push(dataset),
            Err(e) => failed = Some(e),
        }
    });
    if let Some(e) = failed {
        return Err(e);
    }
    let table_at = out.stream_position()? - start;
    let table = postcard::to_stdvec(&Table { datasets }).unwrap_or_default();
    out.write_all(&table)?;
    out.seek(SeekFrom::Start(start + 8))?;
    out.write_all(&table_at.to_le_bytes())?;
    out.seek(SeekFrom::Start(start + table_at + table.len() as u64))?;
    Ok(())
}

/// Writes the chunks of a dataset's bytes and returns its table entry; the container
/// starts at `start` in `out`.
fn write_dataset(
    out: &mut (impl Write + Seek),
    start: u64,
    encoding: Encoding,
    bytes: &[u8],
) -> std::io::Result<Dataset> {
    let mut dataset = Dataset {
        encoding,
        size: bytes.len() as u64,
        at: out.stream_position()? - start,
        chunks: Vec::new(),
    };
    // Batches of chunks, compressed in parallel, so that a large array is not held twice.
    let batch = encoding.chunk() * rayon::current_num_threads().max(1) * 4;
    for part in bytes.chunks(batch.max(1)) {
        for chunk in compress(encoding, part) {
            out.write_all(&chunk)?;
            dataset.chunks.push(chunk.len() as u32);
        }
    }
    Ok(dataset)
}

/// The chunks of a dataset's bytes, shuffled and compressed in parallel.
fn compress(encoding: Encoding, bytes: &[u8]) -> Vec<Vec<u8>> {
    bytes
        .par_chunks(encoding.chunk())
        .map_init(compressor, |compressor, chunk| {
            let shuffled = shuffle(chunk, encoding.width());
            // Compressing into memory does not fail.
            (compressor.as_mut())
                .and_then(|c| c.compress(&shuffled).ok())
                .unwrap_or_else(|| zstd::bulk::compress(&shuffled, LEVEL).unwrap_or_default())
        })
        .collect()
}

/// An array taken out of its value while the skeleton is serialized.
enum Taken {
    Ids(Vec<u32>),
    F64(Vec<f64>),
    F64x3(Vec<[f64; 3]>),
    F32x3(Vec<[f32; 3]>),
}

impl Taken {
    fn take(array: Array<'_>) -> Taken {
        match array {
            Array::Ids(ids) => Taken::Ids(std::mem::take(ids)),
            Array::F64(values) => Taken::F64(std::mem::take(values)),
            Array::F64x3(rows) => Taken::F64x3(std::mem::take(rows)),
            Array::F32x3(rows) => Taken::F32x3(std::mem::take(rows)),
        }
    }

    /// The bytes of its dataset.
    fn encode(&self) -> (Encoding, Vec<u8>) {
        match self {
            Taken::Ids(v) => (Encoding::Ids, id_bytes(v)),
            Taken::F64(v) => (Encoding::F64, number_bytes(v)),
            Taken::F64x3(v) => (Encoding::F64x3, number_bytes(v.as_flattened())),
            Taken::F32x3(v) => (
                Encoding::F32x3,
                v.iter()
                    .flatten()
                    .copied()
                    .flat_map(f32::to_le_bytes)
                    .collect(),
            ),
        }
    }

    fn put_back(self, array: Array<'_>) {
        match (self, array) {
            (Taken::Ids(v), Array::Ids(ids)) => *ids = v,
            (Taken::F64(v), Array::F64(values)) => *values = v,
            (Taken::F64x3(v), Array::F64x3(rows)) => *rows = v,
            (Taken::F32x3(v), Array::F32x3(rows)) => *rows = v,
            // `arrays` visits the same arrays in the same order every time.
            _ => {}
        }
    }
}

/// Reads a value [`pack`] stored behind `magic`.
pub fn unpack<T: Packable>(magic: &[u8; 8], bytes: &[u8]) -> Result<T, PackError> {
    let error = |message: &str| PackError(message.to_owned());
    let rest = bytes
        .strip_prefix(magic.as_slice())
        .ok_or_else(|| error("not results of this Nodalis version: solve again"))?;
    let (table_at, _) = rest
        .split_first_chunk::<8>()
        .ok_or_else(|| error("results cut short"))?;
    let table_at = usize::try_from(u64::from_le_bytes(*table_at))
        .ok()
        .filter(|at| (16..=bytes.len()).contains(at))
        .ok_or_else(|| error("results cut short"))?;
    let (data, table) = bytes.split_at(table_at);
    let table: Table =
        postcard::from_bytes(table).map_err(|e| PackError(format!("results not readable: {e}")))?;

    // The compressed chunks of every dataset, with where they go in it.
    let mut datasets: Vec<Vec<u8>> = Vec::with_capacity(table.datasets.len());
    let mut jobs: Vec<(&[u8], &mut [u8], Encoding)> = Vec::new();
    for dataset in &table.datasets {
        let size = usize::try_from(dataset.size).map_err(|_| error("results too large"))?;
        if !dataset.consistent() {
            return Err(error("results not readable: chunks do not match"));
        }
        datasets.push(vec![0; size]);
    }
    for (dataset, buffer) in table.datasets.iter().zip(&mut datasets) {
        let chunk = dataset.encoding.chunk();
        let mut rest = usize::try_from(dataset.at)
            .ok()
            .filter(|at| (16..=data.len()).contains(at))
            .map(|at| &data[at..])
            .ok_or_else(|| error("results cut short"))?;
        for (target, &length) in buffer.chunks_mut(chunk).zip(&dataset.chunks) {
            let length = length as usize;
            if rest.len() < length {
                return Err(error("results cut short"));
            }
            let (compressed, after) = rest.split_at(length);
            rest = after;
            jobs.push((compressed, target, dataset.encoding));
        }
    }
    jobs.into_par_iter()
        .try_for_each(|(compressed, target, encoding)| {
            let shuffled = zstd::bulk::decompress(compressed, target.len())
                .map_err(|e| PackError(format!("results not readable: {e}")))?;
            if shuffled.len() != target.len() {
                return Err(error("results not readable: a chunk is cut short"));
            }
            unshuffle(&shuffled, encoding.width(), target);
            Ok(())
        })?;

    let mut datasets = table.datasets.iter().map(|d| d.encoding).zip(datasets);
    let (_, skeleton) = datasets.next().ok_or_else(|| error("results are empty"))?;
    let mut value: T = postcard::from_bytes(&skeleton)
        .map_err(|e| PackError(format!("results not readable: {e}")))?;
    let mut failed = None;
    value.arrays(&mut |array| {
        if failed.is_some() {
            return;
        }
        match datasets.next() {
            Some((encoding, bytes)) => {
                if let Err(e) = put_bytes(array, encoding, &bytes) {
                    failed = Some(e);
                }
            }
            None => failed = Some(error("results not readable: an array is missing")),
        }
    });
    match failed {
        Some(e) => Err(e),
        None if datasets.next().is_some() => Err(error("results not readable: arrays left over")),
        None => Ok(value),
    }
}

/// A source of the bytes of a container that [`Archive`] reads parts of.
pub trait Source: std::io::Read + Seek + Send {}

impl<T: std::io::Read + Seek + Send> Source for T {}

/// Where a dataset is in the container.
#[derive(Debug)]
struct Located {
    encoding: Encoding,
    size: usize,
    /// Offset of its first chunk from the start of the container.
    at: u64,
    chunks: Vec<u32>,
}

/// A container opened for reading its datasets one by one, as they are wanted, instead of
/// all of it at once ([`unpack`]): the memory it takes is the table, and of a dataset read,
/// its bytes while they are decoded. The arrays are numbered from 0 in the order of
/// [`Packable::arrays`]; the skeleton is not among them.
pub struct Archive {
    source: std::sync::Mutex<Box<dyn Source>>,
    /// Where the container starts in the source.
    start: u64,
    skeleton: Located,
    arrays: Vec<Located>,
}

impl std::fmt::Debug for Archive {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Archive")
            .field("arrays", &self.arrays.len())
            .finish_non_exhaustive()
    }
}

impl Archive {
    /// Opens the container behind `magic` that starts at the current position of `source`:
    /// reads its table, not its datasets.
    pub fn open(magic: &[u8; 8], mut source: Box<dyn Source>) -> Result<Self, PackError> {
        let error = |message: &str| PackError(message.to_owned());
        let io = |e: std::io::Error| PackError(format!("results not readable: {e}"));
        let start = source.stream_position().map_err(io)?;
        let mut header = [0u8; 16];
        source
            .read_exact(&mut header)
            .map_err(|_| error("results cut short"))?;
        if &header[..8] != magic.as_slice() {
            return Err(error("not results of this Nodalis version: solve again"));
        }
        let table_at = u64::from_le_bytes(header[8..].try_into().unwrap_or_default());
        let end = source.seek(SeekFrom::End(0)).map_err(io)?;
        if table_at < 16 || start + table_at > end {
            return Err(error("results cut short"));
        }
        source.seek(SeekFrom::Start(start + table_at)).map_err(io)?;
        let mut table = Vec::new();
        source.read_to_end(&mut table).map_err(io)?;
        let table: Table = postcard::from_bytes(&table)
            .map_err(|e| PackError(format!("results not readable: {e}")))?;
        let mut located = Vec::with_capacity(table.datasets.len());
        for dataset in table.datasets {
            let size = usize::try_from(dataset.size).map_err(|_| error("results too large"))?;
            if !dataset.consistent() {
                return Err(error("results not readable: chunks do not match"));
            }
            let end = (dataset.at.checked_add(dataset.stored()))
                .filter(|&end| dataset.at >= 16 && end <= table_at);
            if end.is_none() {
                return Err(error("results cut short"));
            }
            located.push(Located {
                encoding: dataset.encoding,
                size,
                at: start + dataset.at,
                chunks: dataset.chunks,
            });
        }
        let mut datasets = located.into_iter();
        let skeleton = datasets.next().ok_or_else(|| error("results are empty"))?;
        Ok(Self {
            source: std::sync::Mutex::new(source),
            start,
            skeleton,
            arrays: datasets.collect(),
        })
    }

    /// Opens the container `bytes` in memory.
    pub fn from_bytes(magic: &[u8; 8], bytes: Vec<u8>) -> Result<Self, PackError> {
        Self::open(magic, Box::new(std::io::Cursor::new(bytes)))
    }

    /// The number of arrays.
    pub fn len(&self) -> usize {
        self.arrays.len()
    }

    pub fn is_empty(&self) -> bool {
        self.arrays.is_empty()
    }

    /// Where the container starts in its source.
    pub fn start(&self) -> u64 {
        self.start
    }

    /// The stored value with all its arrays empty; [`Archive::read`] fills them in.
    pub fn skeleton<T: Packable>(&self) -> Result<T, PackError> {
        let bytes = self.bytes(&self.skeleton)?;
        let mut value: T = postcard::from_bytes(&bytes)
            .map_err(|e| PackError(format!("results not readable: {e}")))?;
        let mut count = 0;
        value.arrays(&mut |_| count += 1);
        if count != self.arrays.len() {
            return Err(PackError(
                "results not readable: the arrays do not match".into(),
            ));
        }
        Ok(value)
    }

    /// The skeleton with all its arrays read: what [`unpack`] gives.
    pub fn read_all<T: Packable>(&self) -> Result<T, PackError> {
        let mut value: T = self.skeleton()?;
        let mut failed = None;
        let mut index = 0;
        value.arrays(&mut |array| {
            if failed.is_none()
                && let Err(e) = self.read(index, array)
            {
                failed = Some(e);
            }
            index += 1;
        });
        match failed {
            Some(e) => Err(e),
            None => Ok(value),
        }
    }

    /// Reads array `index` into `array`, which must be of its type.
    pub fn read(&self, index: usize, array: Array<'_>) -> Result<(), PackError> {
        let located = (self.arrays.get(index))
            .ok_or_else(|| PackError("results not readable: an array is missing".into()))?;
        let bytes = self.bytes(located)?;
        put_bytes(array, located.encoding, &bytes)
    }

    /// Array `index` as it is stored, compressed, to copy it into another container
    /// without decoding it ([`Streaming::raw`]).
    pub fn raw(&self, index: usize) -> Result<RawDataset, PackError> {
        let located = (self.arrays.get(index))
            .ok_or_else(|| PackError("results not readable: an array is missing".into()))?;
        Ok(RawDataset {
            encoding: located.encoding,
            size: located.size as u64,
            chunks: located.chunks.clone(),
            bytes: self.compressed(located)?,
        })
    }

    fn compressed(&self, located: &Located) -> Result<Vec<u8>, PackError> {
        let length: usize = located.chunks.iter().map(|&c| c as usize).sum();
        let mut compressed = vec![0; length];
        let mut source = self.source.lock().unwrap_or_else(|e| e.into_inner());
        (source.seek(SeekFrom::Start(located.at)))
            .and_then(|_| source.read_exact(&mut compressed))
            .map_err(|e| PackError(format!("results not readable: {e}")))?;
        Ok(compressed)
    }

    /// The bytes of a dataset: its chunks read in one go, decompressed in parallel.
    fn bytes(&self, located: &Located) -> Result<Vec<u8>, PackError> {
        decompress(located, &self.compressed(located)?)
    }
}

/// A dataset as it is stored in a container, compressed: copied between containers without
/// being decoded.
#[derive(Debug)]
pub struct RawDataset {
    encoding: Encoding,
    size: u64,
    chunks: Vec<u32>,
    bytes: Vec<u8>,
}

impl RawDataset {
    /// Decodes it into `array`, which must be of its type.
    pub fn decode(&self, array: Array<'_>) -> Result<(), PackError> {
        let located = Located {
            encoding: self.encoding,
            size: self.size as usize,
            at: 0,
            chunks: self.chunks.clone(),
        };
        let bytes = decompress(&located, &self.bytes)?;
        put_bytes(array, self.encoding, &bytes)
    }
}

/// A container written a dataset at a time, in any order: the arrays of a value as they
/// come, such as the increments of a run while it computes the next ones, and the skeleton
/// and table last ([`Streaming::finish`]). The memory it takes is the table, and of an array
/// written, its bytes while they are compressed.
pub struct Streaming<W: Write + Seek> {
    out: W,
    /// Where the container starts in `out`.
    start: u64,
    datasets: Vec<Dataset>,
}

impl<W: Write + Seek> Streaming<W> {
    /// Starts a container behind `magic` at the current position of `out`.
    pub fn new(magic: &[u8; 8], mut out: W) -> std::io::Result<Self> {
        let start = out.stream_position()?;
        out.write_all(magic)?;
        out.write_all(&[0; 8])?;
        Ok(Self {
            out,
            start,
            datasets: Vec::new(),
        })
    }

    /// Writes `array` and returns its number, by which [`Streaming::finish`] puts it into
    /// the table.
    pub fn array(&mut self, array: Array<'_>) -> std::io::Result<usize> {
        let (encoding, bytes) = encode_bytes(&array);
        let dataset = write_dataset(&mut self.out, self.start, encoding, &bytes)?;
        self.datasets.push(dataset);
        Ok(self.datasets.len() - 1)
    }

    /// Writes a dataset of another container as it is stored.
    pub fn raw(&mut self, raw: &RawDataset) -> std::io::Result<usize> {
        let at = self.out.stream_position()? - self.start;
        self.out.write_all(&raw.bytes)?;
        self.datasets.push(Dataset {
            encoding: raw.encoding,
            size: raw.size,
            at,
            chunks: raw.chunks.clone(),
        });
        Ok(self.datasets.len() - 1)
    }

    /// Dataset `index` as it is stored, read back through `read_at` (the bytes of the
    /// container from its start), which sees what was written.
    pub fn stored(
        &self,
        index: usize,
        read_at: impl FnOnce(u64, &mut [u8]) -> std::io::Result<()>,
    ) -> std::io::Result<RawDataset> {
        let dataset = (self.datasets.get(index))
            .ok_or_else(|| std::io::Error::new(std::io::ErrorKind::NotFound, "no such dataset"))?;
        let mut bytes = vec![0; dataset.stored() as usize];
        read_at(self.start + dataset.at, &mut bytes)?;
        Ok(RawDataset {
            encoding: dataset.encoding,
            size: dataset.size,
            chunks: dataset.chunks.clone(),
            bytes,
        })
    }

    /// The writer, to read back what is written; write nothing to it.
    pub fn get_ref(&self) -> &W {
        &self.out
    }

    /// The writer, to move its position back to where the container is written to after
    /// reading; write nothing to it.
    pub fn get_mut(&mut self) -> &mut W {
        &mut self.out
    }

    /// Ends the container with `value`: its skeleton and, in the order of
    /// [`Packable::arrays`], its arrays. `written[k]` is the number of the dataset already
    /// written for the `k`-th array (whose array in `value` is then left as it is), `None`
    /// writes the array of `value` now. Returns the writer, positioned after the container.
    pub fn finish<T: Packable>(
        mut self,
        value: &mut T,
        written: &[Option<usize>],
    ) -> std::io::Result<W> {
        let mut taken: Vec<Taken> = Vec::new();
        value.arrays(&mut |array| taken.push(Taken::take(array)));
        if taken.len() != written.len() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "the arrays of the value do not match the written ones",
            ));
        }
        let skeleton = postcard::to_stdvec(&*value).unwrap_or_default();
        let mut order = vec![write_dataset(
            &mut self.out,
            self.start,
            Encoding::Bytes,
            &skeleton,
        )?];
        drop(skeleton);
        let mut back = Vec::with_capacity(taken.len());
        for (taken, written) in taken.into_iter().zip(written) {
            let dataset = match written {
                Some(index) => self.datasets.get(*index).cloned().ok_or_else(|| {
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, "no such dataset")
                })?,
                None => {
                    let (encoding, bytes) = taken.encode();
                    write_dataset(&mut self.out, self.start, encoding, &bytes)?
                }
            };
            order.push(dataset);
            back.push(taken);
        }
        let mut back = back.into_iter();
        value.arrays(&mut |array| {
            if let Some(taken) = back.next() {
                taken.put_back(array);
            }
        });
        let table_at = self.out.stream_position()? - self.start;
        let table = postcard::to_stdvec(&Table { datasets: order }).unwrap_or_default();
        self.out.write_all(&table)?;
        self.out.seek(SeekFrom::Start(self.start + 8))?;
        self.out.write_all(&table_at.to_le_bytes())?;
        self.out
            .seek(SeekFrom::Start(self.start + table_at + table.len() as u64))?;
        Ok(self.out)
    }
}

/// The decoded bytes of a dataset from its compressed chunks, decompressed in parallel.
fn decompress(located: &Located, compressed: &[u8]) -> Result<Vec<u8>, PackError> {
    let mut out = vec![0; located.size];
    let mut jobs: Vec<(&[u8], &mut [u8])> = Vec::with_capacity(located.chunks.len());
    let mut rest = compressed;
    for (target, &length) in (out.chunks_mut(located.encoding.chunk())).zip(&located.chunks) {
        if rest.len() < length as usize {
            return Err(PackError("results cut short".into()));
        }
        let (chunk, after) = rest.split_at(length as usize);
        rest = after;
        jobs.push((chunk, target));
    }
    let encoding = located.encoding;
    jobs.into_par_iter().try_for_each(|(compressed, target)| {
        let shuffled = zstd::bulk::decompress(compressed, target.len())
            .map_err(|e| PackError(format!("results not readable: {e}")))?;
        if shuffled.len() != target.len() {
            return Err(PackError(
                "results not readable: a chunk is cut short".into(),
            ));
        }
        unshuffle(&shuffled, encoding.width(), target);
        Ok(())
    })?;
    Ok(out)
}

/// A Zstandard compressor at [`LEVEL`] whose frames carry a checksum, so that a damaged
/// file is noticed instead of read as other values.
fn compressor() -> Option<zstd::bulk::Compressor<'static>> {
    let mut compressor = zstd::bulk::Compressor::new(LEVEL).ok()?;
    (compressor.set_parameter(zstd::zstd_safe::CParameter::ChecksumFlag(true))).ok()?;
    Some(compressor)
}

/// The bytes of a dataset of the array.
fn encode_bytes(array: &Array<'_>) -> (Encoding, Vec<u8>) {
    match array {
        Array::Ids(ids) => (Encoding::Ids, id_bytes(ids)),
        Array::F64(values) => (Encoding::F64, number_bytes(values)),
        Array::F64x3(rows) => (Encoding::F64x3, number_bytes(rows.as_flattened())),
        Array::F32x3(rows) => (
            Encoding::F32x3,
            (rows.iter().flatten().copied())
                .flat_map(f32::to_le_bytes)
                .collect(),
        ),
    }
}

/// Ids as the differences to the id before.
fn id_bytes(ids: &[u32]) -> Vec<u8> {
    let mut previous = 0u32;
    (ids.iter())
        .flat_map(|&id| {
            let delta = id.wrapping_sub(previous);
            previous = id;
            delta.to_le_bytes()
        })
        .collect()
}

fn number_bytes(values: &[f64]) -> Vec<u8> {
    values.iter().copied().flat_map(f64::to_le_bytes).collect()
}

/// Puts a dataset's bytes back into its array.
fn put_bytes(array: Array<'_>, encoding: Encoding, bytes: &[u8]) -> Result<(), PackError> {
    let wrong = || PackError("results not readable: an array has the wrong type".into());
    if !bytes.len().is_multiple_of(encoding.width()) {
        return Err(wrong());
    }
    match (array, encoding) {
        (Array::Ids(ids), Encoding::Ids) => {
            let mut previous = 0u32;
            *ids = (bytes.as_chunks::<4>().0.iter())
                .map(|&b| {
                    previous = previous.wrapping_add(u32::from_le_bytes(b));
                    previous
                })
                .collect();
        }
        (Array::F64(values), Encoding::F64) => {
            *values = (bytes.as_chunks::<8>().0.iter())
                .map(|&b| f64::from_le_bytes(b))
                .collect();
        }
        (Array::F64x3(rows), Encoding::F64x3) => {
            *rows = (bytes.as_chunks::<24>().0.iter())
                .map(|row| {
                    let (numbers, _) = row.as_chunks::<8>();
                    std::array::from_fn(|k| f64::from_le_bytes(numbers[k]))
                })
                .collect();
        }
        (Array::F32x3(rows), Encoding::F32x3) => {
            *rows = (bytes.as_chunks::<12>().0.iter())
                .map(|row| {
                    let (numbers, _) = row.as_chunks::<4>();
                    std::array::from_fn(|k| f32::from_le_bytes(numbers[k]))
                })
                .collect();
        }
        _ => return Err(wrong()),
    }
    Ok(())
}

/// The bytes of `data` in values of `width` bytes, byte `k` of every value together; bytes
/// after the last whole value stay at the end.
fn shuffle(data: &[u8], width: usize) -> Vec<u8> {
    if width == 1 {
        return data.to_vec();
    }
    let n = data.len() / width;
    let mut out = vec![0; data.len()];
    for (k, plane) in out[..n * width].chunks_exact_mut(n.max(1)).enumerate() {
        for (i, byte) in plane.iter_mut().enumerate() {
            *byte = data[i * width + k];
        }
    }
    out[n * width..].copy_from_slice(&data[n * width..]);
    out
}

/// The inverse of [`shuffle`], into `out`.
fn unshuffle(data: &[u8], width: usize, out: &mut [u8]) {
    if width == 1 {
        out.copy_from_slice(data);
        return;
    }
    let n = data.len() / width;
    for (k, plane) in data[..n * width].chunks_exact(n.max(1)).enumerate() {
        for (i, &byte) in plane.iter().enumerate() {
            out[i * width + k] = byte;
        }
    }
    out[n * width..].copy_from_slice(&data[n * width..]);
}

/// The results' arrays: node ids and values of every field, nodes and shapes of every mode.
impl Packable for plxs_core::Results {
    fn arrays(&mut self, visit: &mut dyn FnMut(Array<'_>)) {
        for increment in &mut self.increments {
            for field in &mut increment.fields {
                visit(Array::Ids(&mut field.nodes));
                for column in &mut field.values {
                    visit(Array::F64(column));
                }
            }
        }
        for set in &mut self.modes {
            visit(Array::Ids(&mut set.nodes));
            for mode in &mut set.modes {
                visit(Array::F32x3(&mut mode.shape));
                if let Some(imaginary) = &mut mode.imaginary {
                    visit(Array::F32x3(imaginary));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use plxs_core::Results;
    use plxs_core::results::{
        HistoryBlock, IncrementKind, ModeKind, ModeSet, ModeShape, ResultField, ResultsIncrement,
    };

    use super::*;

    const MAGIC: &[u8; 8] = b"TESTPACK";

    /// Results with fields larger than a chunk, a history and modes with an imaginary part.
    fn results(nodes: u32) -> Results {
        let ids: Vec<u32> = (1..=nodes).map(|i| i * 3 / 2).collect();
        let column = |c: usize| -> Vec<f64> {
            (0..nodes)
                .map(|i| ((i as f64 * 0.37 + c as f64).sin() * 1e3).powi(3) / 7.0)
                .collect()
        };
        Results {
            increments: vec![ResultsIncrement {
                step: 1,
                increment: 1,
                kind: IncrementKind::Time,
                value: 1.0,
                total_time: 1.0,
                nodal_diameter: None,
                fields: vec![
                    ResultField {
                        name: "DISP".into(),
                        components: vec!["D1".into(), "D2".into(), "D3".into()],
                        nodes: ids.clone(),
                        values: (0..3).map(column).collect(),
                    },
                    ResultField {
                        name: "EMPTY".into(),
                        components: vec!["X".into()],
                        nodes: Vec::new(),
                        values: vec![Vec::new()],
                    },
                ],
            }],
            history: vec![HistoryBlock {
                step: 1,
                increment: 1,
                quantity: "displacements".into(),
                set: "TIP".into(),
                time: 1.0,
                rows: vec![(7, vec![1.5, -2.0, f64::MIN_POSITIVE])],
            }],
            modes: vec![ModeSet {
                step: 2,
                kind: ModeKind::Frequency,
                nodes: ids.clone(),
                modes: vec![ModeShape {
                    number: 1,
                    eigenvalue: 4.0,
                    value: 0.3,
                    nodal_diameter: Some(2),
                    shape: (0..nodes).map(|i| [i as f32, -(i as f32), 0.5]).collect(),
                    imaginary: Some((0..nodes).map(|i| [0.25, i as f32, f32::NAN]).collect()),
                }],
            }],
        }
    }

    fn same_bits(a: &Results, b: &Results) -> bool {
        // NaN is not equal to itself: compare the stored bytes.
        postcard::to_stdvec(a).unwrap() == postcard::to_stdvec(b).unwrap()
    }

    #[test]
    fn results_read_back_exactly() {
        // More than one chunk per column, and a partial chunk at the end.
        let original = results(300_001);
        let bytes = pack(MAGIC, original.clone());
        let read: Results = unpack(MAGIC, &bytes).unwrap();
        assert!(same_bits(&read, &original));
        // Smooth values and ascending ids compress.
        assert!(bytes.len() < postcard::to_stdvec(&original).unwrap().len());
    }

    #[test]
    fn packing_in_place_keeps_the_value_and_streams_the_same_bytes() {
        let original = results(300_001);
        let mut packed = original.clone();
        let bytes = pack_in_place(MAGIC, &mut packed);
        assert!(
            same_bits(&packed, &original),
            "the arrays go back into the value"
        );
        assert_eq!(bytes, pack(MAGIC, original.clone()));
        // Written behind other bytes of a file, the offsets are relative to the start.
        let mut out = std::io::Cursor::new(b"prefix".to_vec());
        out.set_position(6);
        pack_to(MAGIC, &mut packed, &mut out).unwrap();
        let written = out.into_inner();
        assert_eq!(&written[..6], b"prefix");
        assert_eq!(&written[6..], &bytes[..]);
        let read: Results = unpack(MAGIC, &bytes).unwrap();
        assert!(same_bits(&read, &original));
    }

    #[test]
    fn empty_results_read_back() {
        let bytes = pack(MAGIC, Results::default());
        let read: Results = unpack(MAGIC, &bytes).unwrap();
        assert_eq!(read, Results::default());
    }

    #[test]
    fn damaged_containers_are_refused() {
        let bytes = pack(MAGIC, results(1000));
        assert!(unpack::<Results>(b"OTHERMAG", &bytes).is_err());
        for cut in [0, 8, 11, 20, bytes.len() / 2, bytes.len() - 1] {
            assert!(
                unpack::<Results>(MAGIC, &bytes[..cut]).is_err(),
                "cut at {cut}"
            );
        }
        // A damaged chunk does not read as other results without notice.
        for at in [bytes.len() / 2, bytes.len() - 6] {
            let mut flipped = bytes.clone();
            flipped[at] ^= 0x10;
            assert!(
                unpack::<Results>(MAGIC, &flipped).is_err(),
                "flipped at {at}"
            );
        }
    }

    #[test]
    fn an_archive_reads_single_arrays() {
        let original = results(300_001);
        let bytes = pack(MAGIC, original.clone());
        let mut prefixed = b"prefix".to_vec();
        prefixed.extend(&bytes);
        let mut source = std::io::Cursor::new(prefixed);
        source.set_position(6);
        let archive = Archive::open(MAGIC, Box::new(source)).unwrap();
        assert_eq!(archive.start(), 6);
        let skeleton: Results = archive.skeleton().unwrap();
        assert!(skeleton.increments[0].fields[0].nodes.is_empty());
        assert_eq!(skeleton.increments[0].fields[0].values.len(), 3);
        // Arrays: DISP nodes, its three columns, EMPTY nodes and column, then the modes.
        let mut column = Vec::new();
        archive.read(2, Array::F64(&mut column)).unwrap();
        assert_eq!(column, original.increments[0].fields[0].values[1]);
        let mut ids = Vec::new();
        archive.read(0, Array::Ids(&mut ids)).unwrap();
        assert_eq!(ids, original.increments[0].fields[0].nodes);
        assert!(archive.read(2, Array::Ids(&mut ids)).is_err(), "wrong type");
        assert!(archive.read(99, Array::Ids(&mut ids)).is_err());
        let all: Results = archive.read_all().unwrap();
        assert!(same_bits(&all, &original));
        assert!(Archive::from_bytes(b"OTHERMAG", bytes.clone()).is_err());
        assert!(Archive::from_bytes(MAGIC, bytes[..bytes.len() / 2].to_vec()).is_err());
    }

    #[test]
    fn a_streamed_container_takes_its_arrays_in_any_order() {
        let original = results(300_001);
        // The columns of the field first, in reverse, its nodes and a column nobody keeps
        // after them; the rest of the arrays (EMPTY, the modes) from the value at the end.
        let field = &original.increments[0].fields[0];
        let mut out = Streaming::new(MAGIC, std::io::Cursor::new(Vec::new())).unwrap();
        let columns: Vec<usize> = (0..3)
            .rev()
            .map(|k| out.array(Array::F64(&mut field.values[k].clone())).unwrap())
            .collect();
        let nodes = out.array(Array::Ids(&mut field.nodes.clone())).unwrap();
        out.array(Array::F64(&mut vec![1.0; 10])).unwrap();
        // A dataset copied from another container as it is stored.
        let other = pack(MAGIC, original.clone());
        let other = Archive::from_bytes(MAGIC, other).unwrap();
        let copied = out.raw(&other.raw(1).unwrap()).unwrap();
        let mut value = original.clone();
        value.increments[0].fields[0].nodes.clear();
        value.increments[0].fields[0]
            .values
            .iter_mut()
            .for_each(Vec::clear);
        let mut written = vec![
            Some(nodes),
            Some(copied),
            Some(columns[1]),
            Some(columns[0]),
        ];
        written.resize(4 + 2 + 3, None);
        let bytes = out.finish(&mut value, &written).unwrap().into_inner();
        let read: Results = unpack(MAGIC, &bytes).unwrap();
        assert!(same_bits(&read, &original));
        let archive = Archive::from_bytes(MAGIC, bytes).unwrap();
        assert!(same_bits(
            &archive.read_all::<Results>().unwrap(),
            &original
        ));
        // A raw dataset decodes as the array.
        let mut column = Vec::new();
        archive
            .raw(3)
            .unwrap()
            .decode(Array::F64(&mut column))
            .unwrap();
        assert_eq!(column, field.values[2]);
        // The written arrays must match those of the value.
        let out = Streaming::new(MAGIC, std::io::Cursor::new(Vec::new())).unwrap();
        assert!(out.finish(&mut original.clone(), &[None]).is_err());
    }

    #[test]
    fn shuffle_is_undone() {
        for width in [1, 4, 8, 12, 24] {
            for len in [0, 1, 7, 24, 100, 1001] {
                let data: Vec<u8> = (0..len).map(|i| (i * 31 % 251) as u8).collect();
                let shuffled = shuffle(&data, width);
                let mut back = vec![0; len];
                unshuffle(&shuffled, width, &mut back);
                assert_eq!(back, data, "width {width}, length {len}");
            }
        }
    }
}
