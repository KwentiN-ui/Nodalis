//! Sparse matrices of dense blocks in compressed block rows (BSR), the working format of
//! the iterative solver and its multigrid hierarchy: the unknowns of a node (or of an
//! aggregate on the coarse levels) form a block, so that the index traffic of a product is
//! a small fraction of the value traffic and the setup of the hierarchy works on blocks
//! instead of scalars. Block dimensions are chosen at run time (`3 × 3` for a mechanical
//! model, `1 × 1` for a scalar field, `3 × 6` for the prolongator to the rigid body modes).
//!
//! Products with a vector run over the block rows in parallel.

use rayon::prelude::*;

/// Matrices with fewer stored values than this are multiplied by one thread.
const PARALLEL_FROM: usize = 1 << 15;

#[derive(Clone, Debug, Default)]
pub struct Bsr {
    /// Number of block rows and block columns.
    pub brows: usize,
    pub bcols: usize,
    /// Rows and columns of a block.
    pub br: usize,
    pub bc: usize,
    pub ptr: Vec<usize>,
    pub col: Vec<u32>,
    /// `br · bc` values per block, row major.
    pub val: Vec<f64>,
}

impl Bsr {
    /// Number of rows (unknowns).
    pub fn rows(&self) -> usize {
        self.brows * self.br
    }

    pub fn cols(&self) -> usize {
        self.bcols * self.bc
    }

    /// Stored values.
    pub fn nnz(&self) -> usize {
        self.val.len()
    }

    pub fn blocks(&self) -> usize {
        self.col.len()
    }

    /// The block columns of block row `i` and their values.
    pub fn block_row(&self, i: usize) -> (&[u32], &[f64]) {
        let (s, e) = (self.ptr[i], self.ptr[i + 1]);
        let size = self.br * self.bc;
        (&self.col[s..e], &self.val[s * size..e * size])
    }

    /// `y = A x` (`sign = 1`) or `y = y₀ − A x` with `y₀` given in `y` (`sign = -1`,
    /// `accumulate`).
    fn product(&self, x: &[f64], y: &mut [f64], accumulate: bool) {
        debug_assert_eq!(x.len(), self.cols());
        debug_assert_eq!(y.len(), self.rows());
        let (br, bc) = (self.br, self.bc);
        let size = br * bc;
        let block_row = |(i, y): (usize, &mut [f64])| {
            let (s, e) = (self.ptr[i], self.ptr[i + 1]);
            if !accumulate {
                y.iter_mut().for_each(|v| *v = 0.0);
            }
            match (br, bc) {
                (3, 3) => {
                    let (mut s0, mut s1, mut s2) = (0.0, 0.0, 0.0);
                    for k in s..e {
                        let c = 3 * self.col[k] as usize;
                        let b = &self.val[9 * k..9 * k + 9];
                        let (x0, x1, x2) = (x[c], x[c + 1], x[c + 2]);
                        s0 += b[0] * x0 + b[1] * x1 + b[2] * x2;
                        s1 += b[3] * x0 + b[4] * x1 + b[5] * x2;
                        s2 += b[6] * x0 + b[7] * x1 + b[8] * x2;
                    }
                    if accumulate {
                        y[0] -= s0;
                        y[1] -= s1;
                        y[2] -= s2;
                    } else {
                        y[0] = s0;
                        y[1] = s1;
                        y[2] = s2;
                    }
                }
                (1, 1) => {
                    let mut sum = 0.0;
                    for k in s..e {
                        sum += self.val[k] * x[self.col[k] as usize];
                    }
                    if accumulate {
                        y[0] -= sum;
                    } else {
                        y[0] = sum;
                    }
                }
                _ => {
                    let sign = if accumulate { -1.0 } else { 1.0 };
                    for k in s..e {
                        let c = bc * self.col[k] as usize;
                        let b = &self.val[size * k..size * (k + 1)];
                        let xc = &x[c..c + bc];
                        for (r, y) in y.iter_mut().enumerate() {
                            let row = &b[r * bc..(r + 1) * bc];
                            let dot: f64 = row.iter().zip(xc).map(|(a, x)| a * x).sum();
                            *y += sign * dot;
                        }
                    }
                }
            }
        };
        if self.nnz() < PARALLEL_FROM {
            y.chunks_exact_mut(br).enumerate().for_each(block_row);
        } else {
            y.par_chunks_exact_mut(br)
                .enumerate()
                .with_min_len(64)
                .for_each(block_row);
        }
    }

    /// `y = A x`.
    pub fn multiply(&self, x: &[f64], y: &mut [f64]) {
        self.product(x, y, false);
    }

    /// `y = b − A x`.
    pub fn residual(&self, b: &[f64], x: &[f64], y: &mut [f64]) {
        y.copy_from_slice(b);
        self.product(x, y, true);
    }

    /// The diagonal (zero where there is no diagonal entry); square blocks only.
    pub fn diagonal(&self) -> Vec<f64> {
        debug_assert_eq!(self.br, self.bc);
        let size = self.br * self.bc;
        let mut d = vec![0.0; self.rows()];
        for i in 0..self.brows {
            for k in self.ptr[i]..self.ptr[i + 1] {
                if self.col[k] as usize == i {
                    for r in 0..self.br {
                        d[i * self.br + r] += self.val[size * k + r * self.bc + r];
                    }
                }
            }
        }
        d
    }

    /// The position of the diagonal block of block row `i`, when it exists.
    pub fn diagonal_block(&self, i: usize) -> Option<usize> {
        let (s, e) = (self.ptr[i], self.ptr[i + 1]);
        (s..e).find(|&k| self.col[k] as usize == i)
    }

    /// Calls `f(column, value)` for every entry of the scalar row `i`.
    pub fn for_row(&self, i: usize, mut f: impl FnMut(u32, f64)) {
        let (block, r) = (i / self.br, i % self.br);
        let size = self.br * self.bc;
        for k in self.ptr[block]..self.ptr[block + 1] {
            let c = self.bc as u32 * self.col[k];
            let row = &self.val[size * k + r * self.bc..size * k + (r + 1) * self.bc];
            for (j, &v) in row.iter().enumerate() {
                f(c + j as u32, v);
            }
        }
    }

    /// `Aᵀ`.
    pub fn transpose(&self) -> Bsr {
        let (br, bc) = (self.br, self.bc);
        let size = br * bc;
        let mut count = vec![0usize; self.bcols + 1];
        for &c in &self.col {
            count[c as usize + 1] += 1;
        }
        for i in 0..self.bcols {
            count[i + 1] += count[i];
        }
        let mut next = count.clone();
        let mut col = vec![0u32; self.blocks()];
        let mut val = vec![0.0; self.nnz()];
        for i in 0..self.brows {
            for k in self.ptr[i]..self.ptr[i + 1] {
                let c = self.col[k] as usize;
                let at = next[c];
                next[c] += 1;
                col[at] = i as u32;
                let block = &self.val[size * k..size * (k + 1)];
                let out = &mut val[size * at..size * (at + 1)];
                for r in 0..br {
                    for j in 0..bc {
                        out[j * br + r] = block[r * bc + j];
                    }
                }
            }
        }
        Bsr {
            brows: self.bcols,
            bcols: self.brows,
            br: bc,
            bc: br,
            ptr: count,
            col,
            val,
        }
    }

    /// Builds a matrix of `brows × bcols` blocks of `br × bc` whose block row `i` is
    /// whatever `fill(i, state, acc)` puts into the accumulator (blocks of the same
    /// column add up). Block rows are built in parallel in chunks, each with its own
    /// accumulator and a `state` from `init`.
    pub fn build<S: Send>(
        brows: usize,
        bcols: usize,
        br: usize,
        bc: usize,
        init: impl Fn() -> S + Sync,
        fill: impl Fn(usize, &mut S, &mut BlockAccumulator) + Sync,
    ) -> Bsr {
        let size = br * bc;
        // Small enough chunks for every thread to get several, even on a coarse level.
        let chunk = (brows / 64).clamp(1, 256);
        let ranges: Vec<(usize, usize)> = (0..brows)
            .step_by(chunk)
            .map(|s| (s, (s + chunk).min(brows)))
            .collect();
        let pieces: Vec<(Vec<usize>, Vec<u32>, Vec<f64>)> = ranges
            .par_iter()
            .map_init(
                || (init(), BlockAccumulator::new(bcols, size)),
                |(state, acc), &(start, end)| {
                    let mut ptr = Vec::with_capacity(end - start);
                    let mut col = Vec::new();
                    let mut val = Vec::new();
                    for i in start..end {
                        fill(i, state, acc);
                        ptr.push(col.len());
                        acc.drain_into(&mut col, &mut val);
                    }
                    (ptr, col, val)
                },
            )
            .collect();
        let blocks: usize = pieces.iter().map(|p| p.1.len()).sum();
        let mut ptr = Vec::with_capacity(brows + 1);
        let mut col = Vec::with_capacity(blocks);
        let mut val = Vec::with_capacity(blocks * size);
        for (p, c, v) in pieces {
            let base = col.len();
            ptr.extend(p.iter().map(|&x| x + base));
            col.extend_from_slice(&c);
            val.extend_from_slice(&v);
        }
        ptr.push(col.len());
        Bsr {
            brows,
            bcols,
            br,
            bc,
            ptr,
            col,
            val,
        }
    }
}

/// A sparse accumulator of blocks over `cols` block columns: dense values with a list of
/// the touched columns (in the order of first touch).
pub struct BlockAccumulator {
    size: usize,
    value: Vec<f64>,
    touched: Vec<bool>,
    list: Vec<u32>,
}

impl BlockAccumulator {
    pub fn new(cols: usize, size: usize) -> Self {
        Self {
            size,
            value: vec![0.0; cols * size],
            touched: vec![false; cols],
            list: Vec::new(),
        }
    }

    /// The block of column `col`, for adding to it.
    #[inline]
    pub fn block(&mut self, col: u32) -> &mut [f64] {
        let c = col as usize;
        if !self.touched[c] {
            self.touched[c] = true;
            self.list.push(col);
        }
        &mut self.value[c * self.size..(c + 1) * self.size]
    }

    /// The touched columns and their blocks.
    pub fn entries(&self) -> impl Iterator<Item = (u32, &[f64])> + '_ {
        self.list.iter().map(|&c| {
            (
                c,
                &self.value[c as usize * self.size..(c as usize + 1) * self.size],
            )
        })
    }

    pub fn is_empty(&self) -> bool {
        self.list.is_empty()
    }

    /// Appends the blocks to `col`/`val` and clears the accumulator.
    pub fn drain_into(&mut self, col: &mut Vec<u32>, val: &mut Vec<f64>) {
        for &c in &self.list {
            col.push(c);
            let block = &mut self.value[c as usize * self.size..(c as usize + 1) * self.size];
            val.extend_from_slice(block);
            block.iter_mut().for_each(|v| *v = 0.0);
            self.touched[c as usize] = false;
        }
        self.list.clear();
    }

    pub fn clear(&mut self) {
        for &c in &self.list {
            self.value[c as usize * self.size..(c as usize + 1) * self.size]
                .iter_mut()
                .for_each(|v| *v = 0.0);
            self.touched[c as usize] = false;
        }
        self.list.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `[[1, 2, 0], [0, 3, 4]]` as `1 × 1` blocks.
    fn small() -> Bsr {
        Bsr {
            brows: 2,
            bcols: 3,
            br: 1,
            bc: 1,
            ptr: vec![0, 2, 4],
            col: vec![0, 1, 1, 2],
            val: vec![1.0, 2.0, 3.0, 4.0],
        }
    }

    #[test]
    fn multiplies_and_transposes() {
        let a = small();
        let mut y = vec![0.0; 2];
        a.multiply(&[1.0, 1.0, 1.0], &mut y);
        assert_eq!(y, vec![3.0, 7.0]);
        let mut r = vec![0.0; 2];
        a.residual(&[3.0, 7.0], &[1.0, 1.0, 1.0], &mut r);
        assert_eq!(r, vec![0.0, 0.0]);
        let t = a.transpose();
        assert_eq!((t.brows, t.bcols), (3, 2));
        let mut z = vec![0.0; 3];
        t.multiply(&[1.0, 1.0], &mut z);
        assert_eq!(z, vec![1.0, 5.0, 4.0]);
    }

    #[test]
    fn blocks_of_three_by_two() {
        // One block row, two block columns: [[1 2 | 0 1], [0 1 | 2 0], [1 0 | 0 3]].
        let a = Bsr {
            brows: 1,
            bcols: 2,
            br: 3,
            bc: 2,
            ptr: vec![0, 2],
            col: vec![0, 1],
            val: vec![1.0, 2.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 3.0],
        };
        let mut y = vec![0.0; 3];
        a.multiply(&[1.0, 2.0, 3.0, 4.0], &mut y);
        assert_eq!(y, vec![5.0 + 4.0, 2.0 + 6.0, 1.0 + 12.0]);
        let t = a.transpose();
        let mut z = vec![0.0; 4];
        t.multiply(&[1.0, 1.0, 1.0], &mut z);
        assert_eq!(z, vec![2.0, 3.0, 2.0, 4.0]);
        let mut row = Vec::new();
        a.for_row(2, |c, v| row.push((c, v)));
        assert_eq!(row, vec![(0, 1.0), (1, 0.0), (2, 0.0), (3, 3.0)]);
    }

    #[test]
    fn builds_from_an_accumulator() {
        let a = small();
        let b = Bsr::build(
            2,
            3,
            1,
            1,
            || (),
            |i, (), acc| {
                let (col, val) = a.block_row(i);
                for (&c, &v) in col.iter().zip(val) {
                    acc.block(c)[0] += v;
                    acc.block(c)[0] += v;
                }
            },
        );
        let mut y = vec![0.0; 2];
        b.multiply(&[1.0, 1.0, 1.0], &mut y);
        assert_eq!(y, vec![6.0, 14.0]);
        assert_eq!(b.diagonal(), vec![2.0, 6.0]);
    }
}
