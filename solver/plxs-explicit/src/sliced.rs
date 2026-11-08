//! The stiffness and the viscous matrix in the sliced storage of the product on the card
//! (`product.wgsl`): both on their common pattern, `SLICE` rows per slice, entry `j` of the
//! rows of a slice next to each other. Rows are sorted by length within windows of
//! [`WINDOW`] rows, so that the rows of a slice have about the same length and little
//! padding is read.

use crate::Csr;

/// Rows per slice: one subgroup of threads.
pub(crate) const SLICE: usize = 32;
/// Rows sorted by length together.
pub(crate) const WINDOW: usize = 256;

#[derive(Debug, Default)]
pub(crate) struct Sliced {
    /// The first entry of each slice, and the end.
    pub(crate) start: Vec<u32>,
    /// The row of each position (`u32::MAX` for the padding behind the last row).
    pub(crate) rows: Vec<u32>,
    pub(crate) col: Vec<u32>,
    /// `(K, C)` of each entry.
    pub(crate) val: Vec<[f32; 2]>,
}

impl Sliced {
    pub(crate) fn new(stiffness: &Csr, viscous: &Csr) -> Self {
        let n = stiffness.ptr.len() - 1;
        // The common pattern, row by row.
        let mut merged: Vec<Vec<(u32, [f32; 2])>> = Vec::with_capacity(n);
        for i in 0..n {
            let ks = stiffness.ptr[i] as usize..stiffness.ptr[i + 1] as usize;
            let cs = viscous
                .ptr
                .get(i + 1)
                .map_or(0..0, |&end| viscous.ptr[i] as usize..end as usize);
            let (mut a, mut b) = (ks.start, cs.start);
            let mut row = Vec::with_capacity(ks.len().max(cs.len()));
            while a < ks.end || b < cs.end {
                let ca = if a < ks.end {
                    stiffness.col[a]
                } else {
                    u32::MAX
                };
                let cb = if b < cs.end { viscous.col[b] } else { u32::MAX };
                if ca == cb {
                    row.push((ca, [stiffness.val[a], viscous.val[b]]));
                    a += 1;
                    b += 1;
                } else if ca < cb {
                    row.push((ca, [stiffness.val[a], 0.0]));
                    a += 1;
                } else {
                    row.push((cb, [0.0, viscous.val[b]]));
                    b += 1;
                }
            }
            merged.push(row);
        }
        let mut order: Vec<u32> = (0..n as u32).collect();
        for window in order.chunks_mut(WINDOW) {
            window.sort_by_key(|&r| std::cmp::Reverse(merged[r as usize].len()));
        }
        let slices = n.div_ceil(SLICE);
        let mut out = Sliced {
            start: Vec::with_capacity(slices + 1),
            rows: vec![u32::MAX; slices * SLICE],
            ..Default::default()
        };
        out.rows[..n].copy_from_slice(&order);
        out.start.push(0);
        for s in 0..slices {
            let members = &out.rows[s * SLICE..(s + 1) * SLICE];
            let width = members
                .iter()
                .filter(|&&r| r != u32::MAX)
                .map(|&r| merged[r as usize].len())
                .max()
                .unwrap_or(0);
            for j in 0..width {
                for &r in members {
                    // Padding: a zero entry on a column that exists.
                    let (c, v) = if r == u32::MAX {
                        (0, [0.0; 2])
                    } else {
                        merged[r as usize].get(j).copied().unwrap_or((r, [0.0; 2]))
                    };
                    out.col.push(c);
                    out.val.push(v);
                }
            }
            out.start.push(out.col.len() as u32);
        }
        out
    }

    /// `(K x, C y)` as the card computes it, in double precision, for tests.
    #[cfg(test)]
    fn multiply(&self, x: &[f64], y: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = x.len();
        let (mut fk, mut fc) = (vec![0.0; n], vec![0.0; n]);
        for (t, &row) in self.rows.iter().enumerate() {
            if row == u32::MAX {
                continue;
            }
            let (slice, lane) = (t / SLICE, t % SLICE);
            let first = self.start[slice] as usize;
            let width = (self.start[slice + 1] as usize - first) / SLICE;
            for j in 0..width {
                let k = first + j * SLICE + lane;
                let c = self.col[k] as usize;
                fk[row as usize] += f64::from(self.val[k][0]) * x[c];
                fc[row as usize] += f64::from(self.val[k][1]) * y[c];
            }
        }
        (fk, fc)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_sliced_products_are_the_products_of_the_matrices() {
        // Rows of different lengths and different patterns of K and C, more rows than a
        // window and a last slice that is not full.
        let n = 300;
        let mut ks = Vec::new();
        let mut cs = Vec::new();
        for i in 0..n as u32 {
            for d in 0..(i % 7) {
                let j = (i + 3 * d) % n as u32;
                ks.push((i, j, f64::from(i + j) * 0.5 + 1.0));
                if d % 2 == 0 {
                    cs.push((i, (j + 1) % n as u32, f64::from(d) + 0.25));
                }
            }
        }
        let k = Csr::from_entries(n, ks);
        let c = Csr::from_entries(n, cs);
        let sliced = Sliced::new(&k, &c);
        assert_eq!(sliced.rows.len() % SLICE, 0);
        let x: Vec<f64> = (0..n).map(|i| (i as f64 * 0.37).sin()).collect();
        let y: Vec<f64> = (0..n).map(|i| (i as f64 * 0.11).cos()).collect();
        let (fk, fc) = sliced.multiply(&x, &y);
        for (a, b) in fk
            .iter()
            .zip(k.multiply(&x))
            .chain(fc.iter().zip(c.multiply(&y)))
        {
            assert!((a - b).abs() <= 1e-12 * b.abs().max(1.0), "{a} {b}");
        }
    }
}
