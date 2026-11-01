//! Fill-reducing ordering by nested dissection for the direct solvers.
//!
//! `faer` orders with approximate minimum degree (AMD), which fills the factor of a
//! three-dimensional mesh far more than a nested dissection does. Measured on a 39³-node
//! hexahedral block (173k equations, 4 threads): AMD gives a factor with 840 million
//! nonzeros and a factorization of 37 s; the geometric dissection of this module 239 million
//! and 10.6 s; CalculiX with PaStiX (Scotch ordering) 213 million and 4.5 s for the
//! factorization alone, 10.7 s for the whole run against 12.5 s for `plxs`. This module
//! computes the ordering from the sparsity pattern and, when known, the node positions, and
//! hands it to `faer` as a custom permutation.
//!
//! Method (George 1973, Lipton/Rose/Tarjan 1979, as in textbooks on sparse direct methods):
//!
//! 1. The pattern's graph is compressed to supervariables: vertices with the same closed
//!    neighborhood (the three displacement DOFs of a node share theirs) become one weighted
//!    vertex, which keeps the DOFs of a node together as a dense block of the factor.
//! 2. A subgraph is bisected. With node coordinates (the solver knows them, the DOFs of a
//!    node share them) at the median of the axis of largest extent, which cuts a mesh along
//!    a nearly flat surface; without coordinates along a level structure rooted at a
//!    pseudo-peripheral vertex, taking among the levels whose cumulative weight lies between
//!    35 % and 65 % of the subgraph the cut with the smallest separator. In both cases the
//!    vertex separator is the smaller of the two boundaries of the cut (the vertices of one
//!    side with a neighbor on the other), so that only vertices touching both sides are in it.
//! 3. Both sides are ordered recursively and the separator is numbered after them; small
//!    subgraphs are ordered breadth first (Cuthill-McKee), which keeps their small profile.
//!
//! Disconnected subgraphs are ordered component by component without a separator. Nothing of
//! CalculiX was consulted; PaStiX/Scotch are only the measuring stick.

use std::collections::HashMap;
use std::hash::{Hash, Hasher};

use crate::Pattern;

/// Subgraphs with at most this many DOFs are ordered breadth first instead of dissected.
const LEAF_WEIGHT: usize = 240;

/// The nested dissection ordering of `pattern` as the forward permutation `faer` takes:
/// `perm[new] = old`, the old index eliminated at position `new`. `coordinates`, when given,
/// are the positions of the nodes behind the unknowns (one per unknown) and make the
/// bisections geometric, which gives much smaller separators on meshes.
pub fn nested_dissection(pattern: &Pattern, coordinates: Option<&[[f64; 3]]>) -> Vec<u32> {
    let n = pattern.dimension();
    if n == 0 {
        return Vec::new();
    }
    let graph = Graph::from_pattern(pattern);
    let (quotient, members) = graph.compress();
    let coordinates = coordinates.filter(|c| c.len() == n).map(|c| {
        (members.iter())
            .map(|m| c[m[0] as usize])
            .collect::<Vec<[f64; 3]>>()
    });
    let mut order = Vec::with_capacity(quotient.len());
    let mut dissection = Dissection::new(&quotient, coordinates);
    let all: Vec<u32> = (0..quotient.len() as u32).collect();
    dissection.order(all, &mut order);
    debug_assert_eq!(order.len(), quotient.len());
    let mut perm = Vec::with_capacity(n);
    for v in order {
        perm.extend_from_slice(&members[v as usize]);
    }
    debug_assert_eq!(perm.len(), n);
    perm
}

/// The inverse of a forward permutation: `inv[old] = new`.
pub fn inverse(perm: &[u32]) -> Vec<u32> {
    let mut inv = vec![0u32; perm.len()];
    for (new, &old) in perm.iter().enumerate() {
        inv[old as usize] = new as u32;
    }
    inv
}

/// An undirected graph in compressed adjacency form with vertex weights; neighbor lists are
/// sorted and hold no self loops.
struct Graph {
    ptr: Vec<u32>,
    adj: Vec<u32>,
    weight: Vec<u32>,
}

impl Graph {
    fn from_pattern(pattern: &Pattern) -> Graph {
        let n = pattern.dimension();
        let mut degree = vec![0u32; n];
        for col in 0..n {
            for &row in pattern.column(col) {
                if (row as usize) != col {
                    degree[row as usize] += 1;
                    degree[col] += 1;
                }
            }
        }
        let mut ptr = Vec::with_capacity(n + 1);
        ptr.push(0u32);
        for d in &degree {
            ptr.push(ptr.last().unwrap() + d);
        }
        let mut fill = ptr.clone();
        let mut adj = vec![0u32; ptr[n] as usize];
        // Columns in increasing order and rows sorted within a column: every neighbor list
        // comes out sorted (the rows r < c first, then the columns c' > r as they come).
        for col in 0..n {
            for &row in pattern.column(col) {
                let row = row as usize;
                if row != col {
                    adj[fill[row] as usize] = col as u32;
                    fill[row] += 1;
                    adj[fill[col] as usize] = row as u32;
                    fill[col] += 1;
                }
            }
        }
        Graph {
            ptr,
            adj,
            weight: vec![1; n],
        }
    }

    fn len(&self) -> usize {
        self.weight.len()
    }

    fn neighbors(&self, v: u32) -> &[u32] {
        &self.adj[self.ptr[v as usize] as usize..self.ptr[v as usize + 1] as usize]
    }

    /// Merges vertices with the same closed neighborhood into one weighted vertex; returns the
    /// quotient graph and the members of each of its vertices (sorted).
    fn compress(&self) -> (Graph, Vec<Vec<u32>>) {
        let n = self.len();
        let mut groups: HashMap<u64, Vec<u32>> = HashMap::new();
        let mut representative: Vec<u32> = Vec::new();
        let mut group_of = vec![u32::MAX; n];
        let mut members: Vec<Vec<u32>> = Vec::new();
        for v in 0..n as u32 {
            let mut hasher = std::collections::hash_map::DefaultHasher::new();
            let mut closed = Vec::with_capacity(self.neighbors(v).len() + 1);
            closed.extend_from_slice(self.neighbors(v));
            closed.push(v);
            closed.sort_unstable();
            closed.hash(&mut hasher);
            let key = hasher.finish();
            let candidates = groups.entry(key).or_default();
            let found = candidates.iter().copied().find(|&g| {
                let r = representative[g as usize];
                let mut other = Vec::with_capacity(self.neighbors(r).len() + 1);
                other.extend_from_slice(self.neighbors(r));
                other.push(r);
                other.sort_unstable();
                other == closed
            });
            let g = match found {
                Some(g) => g,
                None => {
                    let g = representative.len() as u32;
                    representative.push(v);
                    members.push(Vec::new());
                    candidates.push(g);
                    g
                }
            };
            group_of[v as usize] = g;
            members[g as usize].push(v);
        }
        let m = representative.len();
        let mut ptr = Vec::with_capacity(m + 1);
        ptr.push(0u32);
        let mut adj = Vec::with_capacity(self.adj.len());
        let mut weight = Vec::with_capacity(m);
        for g in 0..m {
            let r = representative[g];
            let mut list: Vec<u32> = (self.neighbors(r).iter())
                .map(|&u| group_of[u as usize])
                .filter(|&h| h != g as u32)
                .collect();
            list.sort_unstable();
            list.dedup();
            adj.extend_from_slice(&list);
            ptr.push(adj.len() as u32);
            weight.push(members[g].iter().map(|&v| self.weight[v as usize]).sum());
        }
        (Graph { ptr, adj, weight }, members)
    }
}

/// Where a vertex is during one bisection.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Side {
    Out,
    A,
    B,
}

struct Dissection<'a> {
    graph: &'a Graph,
    /// Positions of the (quotient) vertices, when known.
    coordinates: Option<Vec<[f64; 3]>>,
    /// Generation stamp of the subgraph a vertex belongs to during a search.
    stamp: Vec<u32>,
    generation: u32,
    level: Vec<u32>,
    side: Vec<Side>,
}

impl<'a> Dissection<'a> {
    fn new(graph: &'a Graph, coordinates: Option<Vec<[f64; 3]>>) -> Self {
        Self {
            graph,
            coordinates,
            stamp: vec![0; graph.len()],
            generation: 0,
            level: vec![0; graph.len()],
            side: vec![Side::Out; graph.len()],
        }
    }

    /// Splits the subgraph at the median of the axis of largest extent; `None` when the
    /// vertices have no coordinates or all lie at one point. Returns the vertices of the two
    /// sides, each nonempty.
    fn geometric_bisection(&self, vertices: &[u32]) -> Option<(Vec<u32>, Vec<u32>)> {
        let coordinates = self.coordinates.as_ref()?;
        let mut low = [f64::INFINITY; 3];
        let mut high = [f64::NEG_INFINITY; 3];
        for &v in vertices {
            for (axis, &c) in coordinates[v as usize].iter().enumerate() {
                low[axis] = low[axis].min(c);
                high[axis] = high[axis].max(c);
            }
        }
        let (axis, extent) = (0..3)
            .map(|a| (a, high[a] - low[a]))
            .max_by(|a, b| a.1.total_cmp(&b.1))?;
        if extent <= 0.0 || extent.is_nan() {
            return None;
        }
        // Weighted median: sort by coordinate and cut where half the weight is reached.
        let mut sorted: Vec<u32> = vertices.to_vec();
        sorted.sort_unstable_by(|&a, &b| {
            coordinates[a as usize][axis].total_cmp(&coordinates[b as usize][axis])
        });
        let total: usize = (sorted.iter())
            .map(|&v| self.graph.weight[v as usize] as usize)
            .sum();
        let mut cumulative = 0usize;
        let mut split = sorted.len();
        for (i, &v) in sorted.iter().enumerate() {
            cumulative += self.graph.weight[v as usize] as usize;
            if cumulative * 2 >= total {
                split = i + 1;
                break;
            }
        }
        // Vertices at exactly the cutting coordinate stay on the lower side, so that the cut
        // runs between coordinate values (a mesh plane is not split in two).
        let cut_value = coordinates[sorted[split - 1] as usize][axis];
        while split < sorted.len() && coordinates[sorted[split] as usize][axis] == cut_value {
            split += 1;
        }
        if split == sorted.len() {
            // Everything up to the median has one coordinate: cut below it instead.
            split = sorted
                .iter()
                .position(|&v| coordinates[v as usize][axis] == cut_value)?;
            if split == 0 {
                return None;
            }
        }
        let b = sorted.split_off(split);
        Some((sorted, b))
    }

    /// Orders a subgraph split into the sides `a` and `b`: the separator is the smaller
    /// boundary, both sides without it are ordered recursively, the separator last.
    fn order_split(&mut self, a: Vec<u32>, b: Vec<u32>, out: &mut Vec<u32>) {
        for &v in &a {
            self.side[v as usize] = Side::A;
        }
        for &v in &b {
            self.side[v as usize] = Side::B;
        }
        let boundary = |this: &Self, own: &[u32], other: Side| -> (Vec<bool>, usize) {
            let mut mark = vec![false; own.len()];
            let mut weight = 0usize;
            for (i, &v) in own.iter().enumerate() {
                if (this.graph.neighbors(v).iter())
                    .any(|&u| this.selected(u) && this.side[u as usize] == other)
                {
                    mark[i] = true;
                    weight += this.graph.weight[v as usize] as usize;
                }
            }
            (mark, weight)
        };
        let (mark_a, weight_a) = boundary(self, &a, Side::B);
        let (mark_b, weight_b) = boundary(self, &b, Side::A);
        for &v in a.iter().chain(&b) {
            self.side[v as usize] = Side::Out;
        }
        let take_a = weight_a <= weight_b;
        let mut separator = Vec::new();
        let mut rest_a = Vec::with_capacity(a.len());
        let mut rest_b = Vec::with_capacity(b.len());
        for (i, v) in a.into_iter().enumerate() {
            if take_a && mark_a[i] {
                separator.push(v);
            } else {
                rest_a.push(v);
            }
        }
        for (i, v) in b.into_iter().enumerate() {
            if !take_a && mark_b[i] {
                separator.push(v);
            } else {
                rest_b.push(v);
            }
        }
        // The generation changes in the recursion; the separator needs no further searches.
        self.order(rest_a, out);
        self.order(rest_b, out);
        out.extend(separator);
    }

    /// Marks `vertices` as the current subgraph.
    fn select(&mut self, vertices: &[u32]) {
        self.generation += 1;
        for &v in vertices {
            self.stamp[v as usize] = self.generation;
        }
    }

    fn selected(&self, v: u32) -> bool {
        self.stamp[v as usize] == self.generation
    }

    /// Breadth-first search from `root` inside the current subgraph; returns the vertices in
    /// the order reached and fills `level` with their distance from the root.
    fn bfs(&mut self, root: u32, visited: &mut [bool]) -> Vec<u32> {
        let mut order = vec![root];
        visited[root as usize] = true;
        self.level[root as usize] = 0;
        let mut head = 0;
        while head < order.len() {
            let v = order[head];
            head += 1;
            let next = self.level[v as usize] + 1;
            for &u in self.graph.neighbors(v) {
                if self.selected(u) && !visited[u as usize] {
                    visited[u as usize] = true;
                    self.level[u as usize] = next;
                    order.push(u);
                }
            }
        }
        order
    }

    /// Orders the subgraph `vertices` into `out`.
    fn order(&mut self, vertices: Vec<u32>, out: &mut Vec<u32>) {
        if vertices.is_empty() {
            return;
        }
        let weight: usize = vertices
            .iter()
            .map(|&v| self.graph.weight[v as usize] as usize)
            .sum();
        self.select(&vertices);
        let mut visited = vec![false; self.graph.len()];
        // Level structure from a pseudo-peripheral vertex: the farthest vertex of a search
        // from the farthest vertex of a first search.
        let start = vertices
            .iter()
            .copied()
            .min_by_key(|&v| self.graph.neighbors(v).len())
            .unwrap();
        let first = self.bfs(start, &mut visited);
        if first.len() < vertices.len() {
            // Disconnected: the reached component and the rest, no separator between them.
            let rest: Vec<u32> = (vertices.iter().copied())
                .filter(|&v| !visited[v as usize])
                .collect();
            self.order(first, out);
            self.order(rest, out);
            return;
        }
        if weight <= LEAF_WEIGHT || vertices.len() <= 2 {
            // Cuthill-McKee order from the far end of the first search.
            let far = *first.last().unwrap();
            visited.iter_mut().for_each(|v| *v = false);
            let order = self.bfs(far, &mut visited);
            out.extend(order);
            return;
        }
        if let Some((a, b)) = self.geometric_bisection(&vertices) {
            self.order_split(a, b, out);
            return;
        }
        let far = *first.last().unwrap();
        visited.iter_mut().for_each(|v| *v = false);
        let structure = self.bfs(far, &mut visited);
        let depth = self.level[*structure.last().unwrap() as usize] as usize + 1;
        if depth < 3 {
            // Nothing to cut: the subgraph is one or two levels deep (a clique-like set).
            out.extend(structure);
            return;
        }
        // Weight of each level and its boundary toward the next level.
        let mut level_weight = vec![0usize; depth];
        for &v in &structure {
            level_weight[self.level[v as usize] as usize] += self.graph.weight[v as usize] as usize;
        }
        // Candidate cuts: A = levels 0..=cut, B = the rest, with balance in [0.35, 0.65]; the
        // separator is the lighter of the two boundaries. Choose the lightest separator.
        let mut best: Option<(usize, usize, bool)> = None; // (cut, separator weight, take A side)
        let mut cumulative = 0usize;
        for (cut, &cut_weight) in level_weight.iter().enumerate().take(depth - 1) {
            cumulative += cut_weight;
            let fraction = cumulative as f64 / weight as f64;
            if fraction < 0.35 && cut + 2 < depth {
                continue;
            }
            if fraction > 0.65 && best.is_some() {
                break;
            }
            let (a_boundary, b_boundary) = self.boundary_weights(&structure, cut as u32);
            let (sep, take_a) = if a_boundary <= b_boundary {
                (a_boundary, true)
            } else {
                (b_boundary, false)
            };
            if best.is_none_or(|(_, w, _)| sep < w) {
                best = Some((cut, sep, take_a));
            }
            if fraction > 0.65 {
                break;
            }
        }
        let Some((cut, _, _)) = best else {
            out.extend(structure);
            return;
        };
        let (a, b): (Vec<u32>, Vec<u32>) =
            (structure.iter().copied()).partition(|&v| self.level[v as usize] as usize <= cut);
        if a.is_empty() || b.is_empty() {
            out.extend(structure);
            return;
        }
        self.order_split(a, b, out);
    }

    /// Weights of the vertices of level `cut` with a neighbor in level `cut + 1`, and of the
    /// vertices of level `cut + 1` with a neighbor in level `cut`.
    fn boundary_weights(&self, structure: &[u32], cut: u32) -> (usize, usize) {
        let mut a = 0usize;
        let mut b = 0usize;
        for &v in structure {
            let lv = self.level[v as usize];
            if lv == cut {
                if (self.graph.neighbors(v).iter())
                    .any(|&u| self.selected(u) && self.level[u as usize] == cut + 1)
                {
                    a += self.graph.weight[v as usize] as usize;
                }
            } else if lv == cut + 1
                && (self.graph.neighbors(v).iter())
                    .any(|&u| self.selected(u) && self.level[u as usize] == cut)
            {
                b += self.graph.weight[v as usize] as usize;
            }
        }
        (a, b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The pattern of a structured grid of `nx * ny * nz` nodes with `dofs` DOFs each, every
    /// node coupled to its 26 neighbors (what a hexahedral mesh gives).
    fn grid_pattern(nx: usize, ny: usize, nz: usize, dofs: usize) -> Pattern {
        let id = |x: usize, y: usize, z: usize| (x * ny + y) * nz + z;
        let mut blocks = Vec::new();
        for x in 0..nx - 1 {
            for y in 0..ny - 1 {
                for z in 0..nz - 1 {
                    let mut block = Vec::new();
                    for (dx, dy, dz) in [
                        (0, 0, 0),
                        (1, 0, 0),
                        (0, 1, 0),
                        (1, 1, 0),
                        (0, 0, 1),
                        (1, 0, 1),
                        (0, 1, 1),
                        (1, 1, 1),
                    ] {
                        let node = id(x + dx, y + dy, z + dz);
                        for d in 0..dofs {
                            block.push((node * dofs + d) as u32);
                        }
                    }
                    blocks.push(block);
                }
            }
        }
        Pattern::from_blocks(nx * ny * nz * dofs, blocks.iter().map(Vec::as_slice))
    }

    /// Coordinates of the DOFs of `grid_pattern`.
    fn grid_coordinates(nx: usize, ny: usize, nz: usize, dofs: usize) -> Vec<[f64; 3]> {
        let mut coordinates = Vec::new();
        for x in 0..nx {
            for y in 0..ny {
                for z in 0..nz {
                    for _ in 0..dofs {
                        coordinates.push([x as f64, y as f64, z as f64]);
                    }
                }
            }
        }
        coordinates
    }

    fn is_permutation(perm: &[u32]) -> bool {
        let mut seen = vec![false; perm.len()];
        perm.iter().all(|&p| {
            let was = seen[p as usize];
            seen[p as usize] = true;
            !was
        })
    }

    #[test]
    fn orders_every_dof_once() {
        for (nx, ny, nz, dofs) in [(1, 1, 1, 1), (2, 2, 2, 3), (5, 4, 3, 3), (12, 12, 12, 3)] {
            let pattern = grid_pattern(nx, ny, nz, dofs);
            let coordinates = grid_coordinates(nx, ny, nz, dofs);
            for coords in [None, Some(coordinates.as_slice())] {
                let perm = nested_dissection(&pattern, coords);
                assert_eq!(perm.len(), pattern.dimension());
                assert!(is_permutation(&perm), "{nx}x{ny}x{nz}");
                let inv = inverse(&perm);
                for (new, &old) in perm.iter().enumerate() {
                    assert_eq!(inv[old as usize] as usize, new);
                }
            }
        }
    }

    #[test]
    fn keeps_the_dofs_of_a_node_together() {
        let pattern = grid_pattern(6, 6, 6, 3);
        let perm = nested_dissection(&pattern, Some(&grid_coordinates(6, 6, 6, 3)));
        for chunk in perm.chunks(3) {
            let node = chunk[0] / 3;
            assert!(chunk.iter().all(|&d| d / 3 == node), "{chunk:?}");
        }
    }

    #[test]
    fn disconnected_graphs_are_ordered_whole() {
        let a = grid_pattern(4, 4, 4, 1);
        // Two copies of the grid, unconnected.
        let n = a.dimension();
        let mut rows: Vec<Vec<u32>> = Vec::new();
        for col in 0..n {
            rows.push(a.column(col).to_vec());
        }
        for col in 0..n {
            rows.push(a.column(col).iter().map(|&r| r + n as u32).collect());
        }
        let pattern = Pattern::from_columns(2 * n, rows);
        let perm = nested_dissection(&pattern, None);
        assert!(is_permutation(&perm));
        assert_eq!(perm.len(), 2 * n);
    }

    /// The fill of the dissection ordering is far below that of the natural ordering on a
    /// three-dimensional grid; the test counts the nonzeros of the factor with the
    /// elimination-tree algorithm through faer's symbolic analysis.
    #[test]
    fn dissection_fills_less_than_the_identity_ordering() {
        use faer::sparse::SymbolicSparseColMatRef;
        use faer::sparse::linalg::cholesky::{
            CholeskySymbolicParams, SymmetricOrdering, factorize_symbolic_cholesky,
        };
        let pattern = grid_pattern(16, 16, 16, 3);
        let matrix = SymbolicSparseColMatRef::new_checked(
            pattern.dimension(),
            pattern.dimension(),
            pattern.col_ptr(),
            None,
            pattern.row_idx(),
        );
        let nnz = |ordering| {
            factorize_symbolic_cholesky(
                matrix,
                faer::Side::Upper,
                ordering,
                CholeskySymbolicParams::default(),
            )
            .unwrap()
            .len_val()
        };
        let coordinates = grid_coordinates(16, 16, 16, 3);
        let perm = nested_dissection(&pattern, Some(&coordinates));
        let inv = inverse(&perm);
        let custom = faer::perm::PermRef::new_checked(&perm, &inv, pattern.dimension());
        let identity = nnz(SymmetricOrdering::Identity);
        let dissection = nnz(SymmetricOrdering::Custom(custom));
        let amd = nnz(SymmetricOrdering::Amd);
        assert!(
            dissection * 3 < identity * 2,
            "dissection {dissection}, identity {identity}, amd {amd}"
        );
        // Better than AMD from a few thousand unknowns on (on the 39³ block of the benchmark
        // AMD gives 3.5 times the nonzeros, see the module documentation).
        assert!(dissection < amd, "dissection {dissection}, amd {amd}");
        // Without coordinates the level structure still beats the natural order.
        let perm = nested_dissection(&pattern, None);
        let inv = inverse(&perm);
        let custom = faer::perm::PermRef::new_checked(&perm, &inv, pattern.dimension());
        let graph_only = nnz(SymmetricOrdering::Custom(custom));
        assert!(
            graph_only < identity,
            "graph only {graph_only}, identity {identity}"
        );
    }
}
