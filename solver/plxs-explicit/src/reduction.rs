//! The reduction `u = T q + g` of the constraints and the inverse of the reduced mass
//! `M_q = Tᵀ M T` in clusters: the connected groups of reduced unknowns that the mass
//! couples (the fibre of a shell, a few DOFs a constraint ties together), each with its
//! dense inverse. A lumped mass without coupling constraints has clusters of one unknown.

/// The most reduced unknowns of one cluster of the mass.
pub const MAX_CLUSTER: usize = 48;

/// A sparse matrix by rows, in double precision.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Sparse {
    /// Row `i` holds the entries `ptr[i]..ptr[i + 1]`.
    pub ptr: Vec<u32>,
    pub col: Vec<u32>,
    pub val: Vec<f64>,
}

impl Sparse {
    /// The matrix of the entries `(row, col, value)` (repeated entries add up) with `rows`
    /// rows.
    pub fn from_entries(rows: usize, mut entries: Vec<(u32, u32, f64)>) -> Self {
        entries.sort_unstable_by_key(|(r, c, _)| (*r, *c));
        let mut out = Sparse {
            ptr: vec![0; rows + 1],
            ..Default::default()
        };
        for (r, c, v) in entries {
            // Sorted: the last entry is of this row when the row counts one already.
            let same = out.ptr[r as usize + 1] > 0 && out.col.last() == Some(&c);
            if same {
                *out.val.last_mut().expect("an entry") += v;
                continue;
            }
            out.ptr[r as usize + 1] += 1;
            out.col.push(c);
            out.val.push(v);
        }
        for i in 0..rows {
            out.ptr[i + 1] += out.ptr[i];
        }
        out
    }

    pub fn rows(&self) -> usize {
        self.ptr.len().saturating_sub(1)
    }

    /// The entries of row `i`.
    pub fn row(&self, i: usize) -> impl Iterator<Item = (u32, f64)> + '_ {
        let range = self.ptr[i] as usize..self.ptr[i + 1] as usize;
        self.col[range.clone()]
            .iter()
            .copied()
            .zip(self.val[range].iter().copied())
    }

    /// The transpose, with `columns` rows.
    pub fn transpose(&self, columns: usize) -> Self {
        let mut entries = Vec::with_capacity(self.col.len());
        for i in 0..self.rows() {
            for (c, v) in self.row(i) {
                entries.push((c, i as u32, v));
            }
        }
        Self::from_entries(columns, entries)
    }

    /// `y = A x`.
    pub fn multiply(&self, x: &[f64]) -> Vec<f64> {
        (0..self.rows())
            .map(|i| self.row(i).map(|(c, v)| v * x[c as usize]).sum())
            .collect()
    }
}

/// The constraints and the inverse reduced mass the card steps with.
#[derive(Clone, Debug, Default)]
pub struct Reduction {
    /// `T`, one row per DOF, one column per reduced unknown; a held DOF has no entries.
    pub transform: Sparse,
    /// The inverse of `M_q`: block diagonal by clusters, both triangles.
    pub inverse_mass: Sparse,
}

/// The sizes of the clusters of a reduced mass.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Clusters {
    /// Clusters of more than one unknown.
    pub coupled: usize,
    /// Unknowns in the largest cluster.
    pub largest: usize,
}

impl Reduction {
    /// Every DOF with a positive inverse mass is a reduced unknown of its own, the others
    /// are held.
    pub fn selection(inverse_mass: &[f64]) -> Self {
        let mut t = Vec::new();
        let mut inverse = Vec::new();
        for (i, &m) in inverse_mass.iter().enumerate() {
            if m > 0.0 {
                let j = inverse.len() as u32;
                t.push((i as u32, j, 1.0));
                inverse.push((j, j, m));
            }
        }
        let r = inverse.len();
        Self {
            transform: Sparse::from_entries(inverse_mass.len(), t),
            inverse_mass: Sparse::from_entries(r, inverse),
        }
    }

    /// The reduction of `transform` with the reduced mass `mass` (`r × r`, both
    /// triangles): its clusters are inverted one by one. `Err` names a cluster larger
    /// than [`MAX_CLUSTER`] or one whose mass is not positive definite.
    pub fn new(transform: Sparse, mass: &Sparse) -> Result<(Self, Clusters), String> {
        let r = mass.rows();
        // The connected components of the graph of the mass.
        let mut parent: Vec<u32> = (0..r as u32).collect();
        fn root(parent: &mut [u32], mut i: u32) -> u32 {
            while parent[i as usize] != i {
                let up = parent[parent[i as usize] as usize];
                parent[i as usize] = up;
                i = up;
            }
            i
        }
        for i in 0..r {
            for (j, v) in mass.row(i) {
                if v != 0.0 {
                    let (a, b) = (root(&mut parent, i as u32), root(&mut parent, j));
                    if a != b {
                        parent[a.max(b) as usize] = a.min(b);
                    }
                }
            }
        }
        let mut members: Vec<Vec<u32>> = vec![Vec::new(); r];
        for i in 0..r as u32 {
            let k = root(&mut parent, i);
            members[k as usize].push(i);
        }
        let mut entries = Vec::new();
        let mut clusters = Clusters::default();
        for list in members.iter().filter(|l| !l.is_empty()) {
            let size = list.len();
            if size > MAX_CLUSTER {
                return Err(format!(
                    "a constraint couples the mass of {size} unknowns, the card takes at most \
                     {MAX_CLUSTER}"
                ));
            }
            if size > 1 {
                clusters.coupled += 1;
            }
            clusters.largest = clusters.largest.max(size);
            let mut block = vec![0.0; size * size];
            for (a, &i) in list.iter().enumerate() {
                for (j, v) in mass.row(i as usize) {
                    if let Some(b) = list.iter().position(|&x| x == j) {
                        block[a * size + b] += v;
                    }
                }
            }
            let inverse = invert(&block, size).ok_or_else(|| {
                format!(
                    "the reduced mass of unknown {} is not positive definite",
                    list[0]
                )
            })?;
            for (a, &i) in list.iter().enumerate() {
                for (b, &j) in list.iter().enumerate() {
                    entries.push((i, j, inverse[a * size + b]));
                }
            }
        }
        Ok((
            Self {
                transform,
                inverse_mass: Sparse::from_entries(r, entries),
            },
            clusters,
        ))
    }

    /// Reduced unknowns.
    pub fn len(&self) -> usize {
        self.inverse_mass.rows()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// The inverse of the symmetric positive definite `n × n` matrix `a` by Cholesky's
/// factorization, `None` when it is not positive definite.
fn invert(a: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut l = vec![0.0; n * n];
    for j in 0..n {
        let d = a[j * n + j] - (0..j).map(|k| l[j * n + k] * l[j * n + k]).sum::<f64>();
        if d <= 0.0 || !d.is_finite() {
            return None;
        }
        let d = d.sqrt();
        l[j * n + j] = d;
        for i in j + 1..n {
            let s = a[i * n + j] - (0..j).map(|k| l[i * n + k] * l[j * n + k]).sum::<f64>();
            l[i * n + j] = s / d;
        }
    }
    let mut inverse = vec![0.0; n * n];
    for c in 0..n {
        // L y = e_c, then Lᵀ x = y.
        let mut y = vec![0.0; n];
        for i in 0..n {
            let e = if i == c { 1.0 } else { 0.0 };
            y[i] = (e - (0..i).map(|k| l[i * n + k] * y[k]).sum::<f64>()) / l[i * n + i];
        }
        for i in (0..n).rev() {
            let s = y[i]
                - (i + 1..n)
                    .map(|k| l[k * n + i] * inverse[k * n + c])
                    .sum::<f64>();
            inverse[i * n + c] = s / l[i * n + i];
        }
    }
    Some(inverse)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clusters_invert_the_coupled_mass() {
        // Unknowns 0 and 2 coupled, 1 alone, 3-4-5 a chain.
        let entries = vec![
            (0, 0, 2.0),
            (0, 2, 0.5),
            (2, 0, 0.5),
            (2, 2, 3.0),
            (1, 1, 4.0),
            (3, 3, 1.0),
            (3, 4, 0.2),
            (4, 3, 0.2),
            (4, 4, 1.5),
            (4, 5, -0.3),
            (5, 4, -0.3),
            (5, 5, 2.0),
        ];
        let mass = Sparse::from_entries(6, entries);
        let (reduction, clusters) = Reduction::new(Sparse::default(), &mass).unwrap();
        assert_eq!(
            clusters,
            Clusters {
                coupled: 2,
                largest: 3
            }
        );
        for c in 0..6 {
            let e: Vec<f64> = (0..6).map(|i| if i == c { 1.0 } else { 0.0 }).collect();
            let x = reduction.inverse_mass.multiply(&mass.multiply(&e));
            for (i, xi) in x.iter().enumerate() {
                let want = if i == c { 1.0 } else { 0.0 };
                assert!((xi - want).abs() < 1e-12, "{c} {i}: {xi}");
            }
        }
    }

    #[test]
    fn repeated_entries_add_up_and_transpose() {
        let a = Sparse::from_entries(3, vec![(2, 1, 1.0), (0, 0, 2.0), (2, 1, 3.0), (1, 2, -1.0)]);
        assert_eq!(a.ptr, vec![0, 1, 2, 3]);
        assert_eq!(a.val, vec![2.0, -1.0, 4.0]);
        let t = a.transpose(3);
        assert_eq!(t.multiply(&[1.0, 2.0, 3.0]), vec![2.0, 12.0, -2.0]);
    }

    #[test]
    fn a_large_cluster_is_refused() {
        let n = MAX_CLUSTER + 1;
        let mut entries = Vec::new();
        for i in 0..n as u32 {
            entries.push((i, i, 2.0));
            if i + 1 < n as u32 {
                entries.push((i, i + 1, 0.1));
                entries.push((i + 1, i, 0.1));
            }
        }
        let mass = Sparse::from_entries(n, entries);
        assert!(Reduction::new(Sparse::default(), &mass).is_err());
    }
}
