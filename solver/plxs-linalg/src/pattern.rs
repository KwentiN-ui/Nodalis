//! Sparsity pattern and values of a symmetric sparse matrix (upper triangle, CSC).

use std::sync::Arc;

use rayon::prelude::*;

/// Matrices with at least this many stored entries are multiplied in parallel.
const PARALLEL_MULTIPLY_FROM: usize = 1 << 17;
/// At most this many column ranges in a parallel product (each holds a buffer of the rows
/// before it).
const MULTIPLY_CHUNKS: usize = 16;

/// The structure of a symmetric sparse matrix: for every column the sorted row indices of
/// the entries in the upper triangle (`row <= column`), the diagonal always present.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Pattern {
    n: usize,
    col_ptr: Vec<u32>,
    row_idx: Vec<u32>,
}

impl Pattern {
    /// Builds the pattern of a matrix of dimension `n` from the index lists of the dense
    /// blocks that will be added to it (the global DOF numbers of each element). Every pair
    /// of indices in a block is an entry; the diagonal is added for every row.
    pub fn from_blocks<'a>(n: usize, blocks: impl IntoIterator<Item = &'a [u32]>) -> Pattern {
        let blocks: Vec<&[u32]> = blocks.into_iter().collect();
        // The blocks every index is in (compressed by index), then every column on its
        // own, in parallel: the rows of the blocks of the column up to it, marked once.
        let mut in_ptr = vec![0u32; n + 1];
        for block in &blocks {
            for &a in *block {
                in_ptr[a as usize + 1] += 1;
            }
        }
        for i in 0..n {
            in_ptr[i + 1] += in_ptr[i];
        }
        let mut next = in_ptr.clone();
        let mut in_block = vec![0u32; in_ptr[n] as usize];
        for (b, block) in blocks.iter().enumerate() {
            for &a in *block {
                let at = &mut next[a as usize];
                // An index twice in a block is listed once.
                if *at > in_ptr[a as usize] && in_block[*at as usize - 1] == b as u32 {
                    continue;
                }
                in_block[*at as usize] = b as u32;
                *at += 1;
            }
        }
        let in_end = next;
        let columns = |range: std::ops::Range<usize>, mark: &mut Vec<u32>| {
            let mut counts = Vec::with_capacity(range.len());
            let mut rows = Vec::new();
            for col in range {
                let first = rows.len();
                rows.push(col as u32);
                mark[col] = col as u32;
                for &b in &in_block[in_ptr[col] as usize..in_end[col] as usize] {
                    for &a in blocks[b as usize] {
                        if (a as usize) < col && mark[a as usize] != col as u32 {
                            mark[a as usize] = col as u32;
                            rows.push(a);
                        }
                    }
                }
                rows[first..].sort_unstable();
                counts.push((rows.len() - first) as u32);
            }
            (counts, rows)
        };
        const RANGE: usize = 4096;
        let ranges: Vec<std::ops::Range<usize>> = (0..n.div_ceil(RANGE))
            .map(|r| r * RANGE..((r + 1) * RANGE).min(n))
            .collect();
        let parts: Vec<(Vec<u32>, Vec<u32>)> = if in_block.len() < 1 << 16 {
            let mut mark = vec![u32::MAX; n];
            ranges.into_iter().map(|r| columns(r, &mut mark)).collect()
        } else {
            ranges
                .into_par_iter()
                .map_init(|| vec![u32::MAX; n], |mark, r| columns(r, mark))
                .collect()
        };
        let mut col_ptr = Vec::with_capacity(n + 1);
        col_ptr.push(0u32);
        for (counts, _) in &parts {
            for &c in counts {
                col_ptr.push(col_ptr.last().copied().unwrap_or(0) + c);
            }
        }
        let mut row_idx = vec![0u32; col_ptr[n] as usize];
        let mut rest = row_idx.as_mut_slice();
        let mut slices = Vec::with_capacity(parts.len());
        for (_, rows) in &parts {
            let (piece, tail) = rest.split_at_mut(rows.len());
            slices.push(piece);
            rest = tail;
        }
        slices
            .into_par_iter()
            .zip(parts.par_iter())
            .for_each(|(piece, (_, rows))| piece.copy_from_slice(rows));
        Pattern {
            n,
            col_ptr,
            row_idx,
        }
    }

    /// Builds a pattern from the row indices of the upper triangle per column (`rows[col]`,
    /// any order, duplicates allowed, entries below the diagonal ignored). The diagonal is
    /// always added.
    pub fn from_columns(n: usize, mut rows: Vec<Vec<u32>>) -> Pattern {
        assert_eq!(rows.len(), n);
        let mut col_ptr = Vec::with_capacity(n + 1);
        let mut row_idx = Vec::new();
        col_ptr.push(0);
        for (col, list) in rows.iter_mut().enumerate() {
            list.retain(|&r| (r as usize) < col);
            list.push(col as u32);
            list.sort_unstable();
            list.dedup();
            row_idx.extend_from_slice(list);
            col_ptr.push(row_idx.len() as u32);
        }
        Pattern {
            n,
            col_ptr,
            row_idx,
        }
    }

    /// A pattern from its compressed columns: `col_ptr` (`n + 1` offsets) and per column
    /// the row indices in `row_idx`, ascending, within the upper triangle and with the
    /// diagonal.
    pub fn from_compressed(n: usize, col_ptr: Vec<u32>, row_idx: Vec<u32>) -> Pattern {
        assert_eq!(col_ptr.len(), n + 1);
        assert_eq!(col_ptr[n] as usize, row_idx.len());
        debug_assert!((0..n).all(|c| {
            let rows = &row_idx[col_ptr[c] as usize..col_ptr[c + 1] as usize];
            rows.windows(2).all(|w| w[0] < w[1]) && rows.last() == Some(&(c as u32))
        }));
        Pattern {
            n,
            col_ptr,
            row_idx,
        }
    }

    pub fn dimension(&self) -> usize {
        self.n
    }

    /// Number of stored entries (upper triangle with diagonal).
    pub fn nnz(&self) -> usize {
        self.row_idx.len()
    }

    pub fn col_ptr(&self) -> &[u32] {
        &self.col_ptr
    }

    pub fn row_idx(&self) -> &[u32] {
        &self.row_idx
    }

    /// Row indices of a column.
    pub fn column(&self, col: usize) -> &[u32] {
        &self.row_idx[self.col_ptr[col] as usize..self.col_ptr[col + 1] as usize]
    }

    /// Position of the entry `(row, col)` with `row <= col` in the value vector.
    pub fn position(&self, row: u32, col: u32) -> Option<usize> {
        let start = self.col_ptr[col as usize] as usize;
        let rows = self.column(col as usize);
        rows.binary_search(&row).ok().map(|k| start + k)
    }
}

/// A symmetric sparse matrix on a shared pattern.
#[derive(Clone, Debug)]
pub struct SparseMatrix {
    pattern: Arc<Pattern>,
    values: Vec<f64>,
}

impl SparseMatrix {
    pub fn zeros(pattern: Arc<Pattern>) -> Self {
        let values = vec![0.0; pattern.nnz()];
        Self { pattern, values }
    }

    pub fn pattern(&self) -> &Arc<Pattern> {
        &self.pattern
    }

    pub fn dimension(&self) -> usize {
        self.pattern.n
    }

    pub fn values(&self) -> &[f64] {
        &self.values
    }

    pub fn values_mut(&mut self) -> &mut [f64] {
        &mut self.values
    }

    pub fn fill_zero(&mut self) {
        // In parallel for the large matrices of an assembly in every Newton iteration.
        self.values
            .par_chunks_mut(1 << 16)
            .for_each(|chunk| chunk.fill(0.0));
    }

    /// Adds a value to the entry `(i, j)`; `(j, i)` is the same entry. Panics when the
    /// pattern has no such entry.
    pub fn add(&mut self, i: u32, j: u32, value: f64) {
        let (row, col) = if i <= j { (i, j) } else { (j, i) };
        let k = self
            .pattern
            .position(row, col)
            .unwrap_or_else(|| panic!("entry ({i}, {j}) is not in the sparsity pattern"));
        self.values[k] += value;
    }

    /// Where the upper triangle of a block at the global indices `dofs` lands in the values,
    /// in the order [`Self::add_block`] adds it: row `a`, columns `b >= a`. Looking the
    /// positions up is most of the cost of [`Self::add_block`]; an assembly can look them up
    /// in parallel and add the blocks after. Panics when the pattern lacks an entry.
    pub fn block_positions(&self, dofs: &[u32], out: &mut Vec<u32>) {
        let m = dofs.len();
        out.clear();
        out.reserve(m * (m + 1) / 2);
        for a in 0..m {
            for b in a..m {
                let (i, j) = (dofs[a], dofs[b]);
                let (row, col) = if i <= j { (i, j) } else { (j, i) };
                let k = self
                    .pattern
                    .position(row, col)
                    .unwrap_or_else(|| panic!("entry ({i}, {j}) is not in the sparsity pattern"));
                out.push(k as u32);
            }
        }
    }

    /// Adds a dense symmetric block (row-major `m x m`) at the global indices `dofs`; only
    /// the upper triangle of the block is read.
    pub fn add_block(&mut self, dofs: &[u32], block: &[f64]) {
        let m = dofs.len();
        debug_assert_eq!(block.len(), m * m);
        for a in 0..m {
            for b in a..m {
                let (i, j) = (dofs[a], dofs[b]);
                let (row, col) = if i <= j { (i, j) } else { (j, i) };
                let k = self
                    .pattern
                    .position(row, col)
                    .unwrap_or_else(|| panic!("entry ({i}, {j}) is not in the sparsity pattern"));
                self.values[k] += block[a * m + b];
            }
        }
    }

    /// Adds many dense symmetric blocks (`(dofs, row-major block)`, upper triangles read,
    /// as [`Self::add_block`]) in parallel: the positions are looked up per block, then
    /// ranges of the values take the blocks in their order, so every entry gets the same
    /// sums in the same order as adding the blocks one after the other.
    pub fn add_blocks(&mut self, blocks: &[(&[u32], &[f64])]) {
        let total: usize = blocks.iter().map(|(d, _)| d.len() * d.len()).sum();
        if total < 1 << 14 || rayon::current_num_threads() < 2 {
            for (dofs, block) in blocks {
                self.add_block(dofs, block);
            }
            return;
        }
        let positions: Vec<(Vec<u32>, u32, u32)> = blocks
            .par_iter()
            .map(|(dofs, _)| {
                let mut out = Vec::new();
                self.block_positions(dofs, &mut out);
                let (lo, hi) = out
                    .iter()
                    .fold((u32::MAX, 0), |(lo, hi), &p| (lo.min(p), hi.max(p)));
                (out, lo, hi)
            })
            .collect();
        let parts = rayon::current_num_threads();
        let size = self.values.len().div_ceil(parts).max(1 << 12);
        self.values
            .par_chunks_mut(size)
            .enumerate()
            .for_each(|(part, chunk)| {
                let lo = part * size;
                let hi = lo + chunk.len();
                for ((dofs, block), (at, first, last)) in blocks.iter().zip(&positions) {
                    if at.is_empty() || (*last as usize) < lo || *first as usize >= hi {
                        continue;
                    }
                    let m = dofs.len();
                    let mut at = at.iter();
                    for a in 0..m {
                        for &value in &block[a * m + a..(a + 1) * m] {
                            let p = *at.next().expect("a position per entry") as usize;
                            if (lo..hi).contains(&p) {
                                chunk[p - lo] += value;
                            }
                        }
                    }
                }
            });
    }

    /// `y = A x` for the full symmetric matrix; in parallel when the matrix is large.
    pub fn multiply(&self, x: &[f64], y: &mut [f64]) {
        let n = self.pattern.n;
        assert_eq!(x.len(), n);
        assert_eq!(y.len(), n);
        let chunks = rayon::current_num_threads().min(MULTIPLY_CHUNKS);
        if self.values.len() < PARALLEL_MULTIPLY_FROM || chunks < 2 {
            y.iter_mut().for_each(|v| *v = 0.0);
            self.multiply_columns(x, 0, y, &mut []);
            return;
        }
        // Column ranges of about equal numbers of entries. Every range writes the rows of
        // its own columns (the lower triangle, gathered, and the upper triangle within the
        // range) and collects the contributions of its upper triangle to earlier rows in a
        // buffer of its own, added afterwards.
        let nnz = self.values.len();
        let col_ptr = &self.pattern.col_ptr;
        let mut bounds = vec![0usize];
        for c in 1..chunks {
            let target = (nnz * c / chunks) as u32;
            let at = col_ptr.partition_point(|&p| p < target).min(n);
            bounds.push(at.max(*bounds.last().unwrap()));
        }
        bounds.push(n);
        let mut pieces: Vec<&mut [f64]> = Vec::with_capacity(chunks);
        let mut rest = &mut y[..];
        for c in 0..chunks {
            let (piece, tail) = rest.split_at_mut(bounds[c + 1] - bounds[c]);
            pieces.push(piece);
            rest = tail;
        }
        let earlier: Vec<Vec<f64>> = pieces
            .into_par_iter()
            .enumerate()
            .map(|(c, piece)| {
                piece.iter_mut().for_each(|v| *v = 0.0);
                let mut earlier = vec![0.0; bounds[c]];
                self.multiply_columns(x, bounds[c], piece, &mut earlier);
                earlier
            })
            .collect();
        const ROWS: usize = 1 << 14;
        y.par_chunks_mut(ROWS).enumerate().for_each(|(b, rows)| {
            let first = b * ROWS;
            for buffer in &earlier {
                if buffer.len() <= first {
                    continue;
                }
                let end = (first + rows.len()).min(buffer.len());
                for (yi, e) in rows.iter_mut().zip(&buffer[first..end]) {
                    *yi += e;
                }
            }
        });
    }

    /// The product of the columns `first..first + own.len()`: their own rows go to `own`,
    /// the rows before `first` to `earlier` (which must hold them).
    fn multiply_columns(&self, x: &[f64], first: usize, own: &mut [f64], earlier: &mut [f64]) {
        for (local, col) in (first..first + own.len()).enumerate() {
            let start = self.pattern.col_ptr[col] as usize;
            let end = self.pattern.col_ptr[col + 1] as usize;
            let xc = x[col];
            let mut sum = 0.0;
            for k in start..end {
                let row = self.pattern.row_idx[k] as usize;
                let v = self.values[k];
                if row >= first {
                    own[row - first] += v * xc;
                } else {
                    earlier[row] += v * xc;
                }
                if row != col {
                    sum += v * x[row];
                }
            }
            own[local] += sum;
        }
    }

    /// `self += alpha · other`; both matrices must share the pattern (the same `Arc` or an
    /// equal one, as two matrices assembled from the same element table do).
    pub fn add_scaled(&mut self, other: &SparseMatrix, alpha: f64) {
        assert!(
            Arc::ptr_eq(&self.pattern, &other.pattern) || *self.pattern == *other.pattern,
            "add_scaled needs matrices with the same sparsity pattern"
        );
        for (v, o) in self.values.iter_mut().zip(&other.values) {
            *v += alpha * o;
        }
    }

    /// The same matrix on a larger pattern that holds every entry of this one (a pattern
    /// with extra contact springs); the new entries are zero. Returns a clone when the
    /// pattern is the same.
    pub fn embed(&self, pattern: Arc<Pattern>) -> SparseMatrix {
        if Arc::ptr_eq(&self.pattern, &pattern) || *self.pattern == *pattern {
            return SparseMatrix {
                pattern,
                values: self.values.clone(),
            };
        }
        assert_eq!(self.pattern.n, pattern.n, "embed needs the same dimension");
        let mut out = SparseMatrix::zeros(pattern);
        for col in 0..self.pattern.n {
            let start = self.pattern.col_ptr[col] as usize;
            for (k, &row) in self.pattern.column(col).iter().enumerate() {
                let at = out
                    .pattern
                    .position(row, col as u32)
                    .unwrap_or_else(|| panic!("entry ({row}, {col}) is not in the larger pattern"));
                out.values[at] = self.values[start + k];
            }
        }
        out
    }

    /// `alpha · a + beta · b` on the pattern of `a` (`b` must share it).
    pub fn linear_combination(
        a: &SparseMatrix,
        alpha: f64,
        b: &SparseMatrix,
        beta: f64,
    ) -> SparseMatrix {
        let mut out = a.clone();
        for v in out.values.iter_mut() {
            *v *= alpha;
        }
        out.add_scaled(b, beta);
        out
    }

    /// The entry `(i, j)`, zero when it is not in the pattern.
    pub fn get(&self, i: u32, j: u32) -> f64 {
        let (row, col) = if i <= j { (i, j) } else { (j, i) };
        self.pattern
            .position(row, col)
            .map_or(0.0, |k| self.values[k])
    }

    /// Diagonal entries.
    pub fn diagonal(&self) -> Vec<f64> {
        (0..self.pattern.n as u32).map(|i| self.get(i, i)).collect()
    }

    /// The submatrix of the rows and columns `keep` (ascending global indices), on a new
    /// pattern of dimension `keep.len()`.
    pub fn restrict(&self, keep: &[u32]) -> SparseMatrix {
        let n = self.pattern.n;
        let mut map = vec![u32::MAX; n];
        for (new, &old) in keep.iter().enumerate() {
            map[old as usize] = new as u32;
        }
        let mut col_ptr = Vec::with_capacity(keep.len() + 1);
        let mut row_idx = Vec::new();
        let mut values = Vec::new();
        col_ptr.push(0u32);
        for &old_col in keep {
            let start = self.pattern.col_ptr[old_col as usize] as usize;
            let end = self.pattern.col_ptr[old_col as usize + 1] as usize;
            for k in start..end {
                let new_row = map[self.pattern.row_idx[k] as usize];
                if new_row != u32::MAX {
                    row_idx.push(new_row);
                    values.push(self.values[k]);
                }
            }
            col_ptr.push(row_idx.len() as u32);
        }
        SparseMatrix {
            pattern: Arc::new(Pattern {
                n: keep.len(),
                col_ptr,
                row_idx,
            }),
            values,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_matrix_embedded_in_a_larger_pattern_keeps_its_entries() {
        let small = Arc::new(Pattern::from_blocks(4, [[0u32, 1].as_slice(), &[2, 3]]));
        let large = Arc::new(Pattern::from_blocks(4, [[0u32, 1, 2, 3].as_slice()]));
        let mut a = SparseMatrix::zeros(small);
        a.add_block(&[0, 1], &[4.0, 1.0, 1.0, 3.0]);
        a.add_block(&[2, 3], &[2.0, -1.0, -1.0, 5.0]);
        let b = a.embed(large.clone());
        assert!(Arc::ptr_eq(b.pattern(), &large));
        for i in 0..4 {
            for j in 0..4 {
                assert_eq!(a.get(i, j), b.get(i, j), "({i}, {j})");
            }
        }
        assert_eq!(b.values().iter().filter(|v| **v != 0.0).count(), 6);
    }

    #[test]
    fn the_parallel_product_equals_the_sequential_one() {
        // A banded matrix large enough for the parallel product.
        let n = 20_000;
        let width = 12u32;
        let blocks: Vec<Vec<u32>> = (0..n as u32 - width)
            .map(|i| (i..i + width).collect())
            .collect();
        let pattern = Arc::new(Pattern::from_blocks(n, blocks.iter().map(Vec::as_slice)));
        let mut a = SparseMatrix::zeros(pattern.clone());
        for (k, v) in a.values_mut().iter_mut().enumerate() {
            *v = ((k * 7919) % 1000) as f64 / 1000.0 - 0.3;
        }
        assert!(a.values().len() >= PARALLEL_MULTIPLY_FROM);
        let x: Vec<f64> = (0..n).map(|i| ((i * 31) % 17) as f64 - 8.0).collect();
        let mut expected = vec![0.0; n];
        for col in 0..n {
            for (k, &row) in pattern.column(col).iter().enumerate() {
                let v = a.values()[pattern.col_ptr()[col] as usize + k];
                expected[row as usize] += v * x[col];
                if row as usize != col {
                    expected[col] += v * x[row as usize];
                }
            }
        }
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(4)
            .build()
            .unwrap();
        let mut y = vec![1.0; n];
        pool.install(|| a.multiply(&x, &mut y));
        for (y, e) in y.iter().zip(&expected) {
            assert!((y - e).abs() <= 1e-12 * e.abs().max(1.0), "{y} vs {e}");
        }
    }

    /// Pseudo-random blocks over `n` indices, some with an index twice.
    fn random_blocks(n: usize, count: usize, size: usize) -> Vec<Vec<u32>> {
        let mut h = 0x9E37_79B9_7F4A_7C15u64;
        let mut next = move || {
            h ^= h << 13;
            h ^= h >> 7;
            h ^= h << 17;
            h
        };
        (0..count)
            .map(|b| {
                let base = (next() % n as u64) as u32;
                let mut block: Vec<u32> = (0..size)
                    .map(|_| (base + (next() % 200) as u32) % n as u32)
                    .collect();
                if b % 7 == 0 {
                    block.push(block[0]);
                }
                block
            })
            .collect()
    }

    #[test]
    fn the_parallel_pattern_equals_the_columns_of_the_blocks() {
        let n = 30_000;
        let blocks = random_blocks(n, 6000, 30);
        let mut rows: Vec<Vec<u32>> = vec![Vec::new(); n];
        for block in &blocks {
            for &a in block {
                for &b in block {
                    rows[b as usize].push(a);
                }
            }
        }
        let expected = Pattern::from_columns(n, rows);
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(4)
            .build()
            .unwrap();
        let got = pool.install(|| Pattern::from_blocks(n, blocks.iter().map(Vec::as_slice)));
        assert_eq!(got, expected);
    }

    #[test]
    fn blocks_added_in_parallel_sum_as_one_after_the_other() {
        let n = 30_000;
        let blocks = random_blocks(n, 6000, 30);
        let pattern = Arc::new(Pattern::from_blocks(n, blocks.iter().map(Vec::as_slice)));
        let matrices: Vec<Vec<f64>> = (blocks.iter().enumerate())
            .map(|(b, block)| {
                let m = block.len();
                (0..m * m)
                    .map(|e| ((b * 31 + e * 17) % 1009) as f64 / 7.0 - 50.0)
                    .collect()
            })
            .collect();
        let mut one_by_one = SparseMatrix::zeros(pattern.clone());
        for (block, ke) in blocks.iter().zip(&matrices) {
            one_by_one.add_block(block, ke);
        }
        let pairs: Vec<(&[u32], &[f64])> = (blocks.iter())
            .zip(&matrices)
            .map(|(b, m)| (b.as_slice(), m.as_slice()))
            .collect();
        let mut parallel = SparseMatrix::zeros(pattern);
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(4)
            .build()
            .unwrap();
        pool.install(|| parallel.add_blocks(&pairs));
        // The same sums in the same order: equal to the last bit.
        assert_eq!(parallel.values(), one_by_one.values());
    }

    #[test]
    fn pattern_holds_blocks_and_diagonal() {
        let p = Pattern::from_blocks(4, [&[0u32, 2][..], &[2, 3][..]]);
        assert_eq!(p.column(0), &[0]);
        assert_eq!(p.column(2), &[0, 2]);
        assert_eq!(p.column(3), &[2, 3]);
        assert_eq!(p.position(0, 2), Some(2));
        assert_eq!(p.position(1, 2), None);
    }

    #[test]
    fn blocks_add_symmetrically_and_multiply() {
        let p = Arc::new(Pattern::from_blocks(3, [&[2u32, 0][..]]));
        let mut a = SparseMatrix::zeros(p);
        // Block on dofs (2, 0): [[4, 1], [1, 2]] -> A[2][2] = 4, A[0][2] = 1, A[0][0] = 2.
        a.add_block(&[2, 0], &[4.0, 1.0, 1.0, 2.0]);
        a.add(1, 1, 5.0);
        assert_eq!(a.get(2, 2), 4.0);
        assert_eq!(a.get(0, 2), 1.0);
        assert_eq!(a.get(2, 0), 1.0);
        assert_eq!(a.get(0, 0), 2.0);
        let mut y = vec![0.0; 3];
        a.multiply(&[1.0, 1.0, 1.0], &mut y);
        assert_eq!(y, vec![3.0, 5.0, 5.0]);
        let r = a.restrict(&[0, 2]);
        assert_eq!(r.dimension(), 2);
        assert_eq!(r.get(0, 1), 1.0);
        assert_eq!(r.get(1, 1), 4.0);
    }
}
