//! Sparsity pattern and values of a symmetric sparse matrix (upper triangle, CSC).

use std::sync::Arc;

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
        // Rows per column, collected then sorted and deduplicated.
        let mut rows: Vec<Vec<u32>> = (0..n as u32).map(|i| vec![i]).collect();
        for block in blocks {
            for &a in block {
                for &b in block {
                    if a < b {
                        rows[b as usize].push(a);
                    }
                }
            }
        }
        let mut col_ptr = Vec::with_capacity(n + 1);
        let mut row_idx = Vec::new();
        col_ptr.push(0);
        for list in &mut rows {
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
        self.values.iter_mut().for_each(|v| *v = 0.0);
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

    /// `y = A x` for the full symmetric matrix.
    pub fn multiply(&self, x: &[f64], y: &mut [f64]) {
        let n = self.pattern.n;
        assert_eq!(x.len(), n);
        assert_eq!(y.len(), n);
        y.iter_mut().for_each(|v| *v = 0.0);
        for col in 0..n {
            let start = self.pattern.col_ptr[col] as usize;
            let end = self.pattern.col_ptr[col + 1] as usize;
            let xc = x[col];
            for k in start..end {
                let row = self.pattern.row_idx[k] as usize;
                let v = self.values[k];
                y[row] += v * xc;
                if row != col {
                    y[col] += v * x[row];
                }
            }
        }
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
