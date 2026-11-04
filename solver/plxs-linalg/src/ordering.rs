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
//!    node share them) by a plane normal to one of the axes, which cuts a mesh along a
//!    nearly flat surface: every axis and several positions (30 % to 70 % of the weight) are
//!    tried and the cut with the smallest separator wins, which matters on parts with holes
//!    and ribs. Without coordinates along a level structure rooted at a pseudo-peripheral
//!    vertex, taking among the levels whose cumulative weight lies between 35 % and 65 % of
//!    the subgraph the cut with the smallest separator. In both cases the vertex separator
//!    is a minimum vertex cover of the edges between the sides (König's theorem on a maximum
//!    matching of the boundary graph), never larger than the smaller boundary, and is then
//!    refined by moving its vertices into a side where fewer vertices of the other side have
//!    to take their place (the greedy part of Fiduccia-Mattheyses).
//! 3. Both sides are ordered recursively and the separator is numbered after them; small
//!    subgraphs are ordered breadth first (Cuthill-McKee), which keeps their small profile.
//!
//! On Quentin's 591k-equation bracket (C3D10) the median cut on the longest axis with the
//! smaller boundary as separator gave a factor of 2.35 billion nonzeros (18.8 GB, beyond
//! the memory); the choice of axis and position and the vertex cover 740 million, the
//! refinement 690 million, leaves of 60 instead of 240 DOFs 650 million (5.2 GB, factorized
//! in 12 s on 4 threads); PaStiX with Scotch 387 million.
//!
//! Disconnected subgraphs are ordered component by component without a separator. The
//! dissection keeps its pending subgraphs in a work list instead of recursing: a graph that
//! falls apart into many thousands of pieces overflowed the stack of a solver thread. Nothing of
//! CalculiX was consulted; PaStiX/Scotch are only the measuring stick.

use std::collections::HashMap;
use std::hash::{Hash, Hasher};

use crate::Pattern;

/// Subgraphs with at most this many DOFs are ordered breadth first instead of dissected.
const LEAF_WEIGHT: usize = 60;

/// Where along an axis a geometric cut is tried (fraction of the weight on the lower
/// side); the cut with the smallest separator wins.
const CUT_FRACTIONS: [f64; 5] = [0.5, 0.4, 0.6, 0.3, 0.7];

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
    /// In the separator (during its refinement).
    S,
}

/// A candidate bisection: the two sides, the separator (a subset of them) and its weight.
struct Split {
    a: Vec<u32>,
    b: Vec<u32>,
    separator: Vec<u32>,
    weight: usize,
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
    /// Search stamp of the vertices a breadth-first search reached: a vertex is visited in
    /// the current search when its stamp equals `search`. Saves clearing a flag per vertex
    /// of the whole graph for every subgraph.
    seen: Vec<u32>,
    search: u32,
}

/// Work left for [`Dissection::order`], done last in, first out.
enum Task {
    /// Order this subgraph.
    Order(Vec<u32>),
    /// Append these vertices (a separator, after both of its sides).
    Emit(Vec<u32>),
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
            seen: vec![0; graph.len()],
            search: 0,
        }
    }

    /// The axes along which the subgraph extends, the largest extent first.
    fn axes_by_extent(&self, vertices: &[u32]) -> Vec<usize> {
        let Some(coordinates) = self.coordinates.as_ref() else {
            return Vec::new();
        };
        let mut low = [f64::INFINITY; 3];
        let mut high = [f64::NEG_INFINITY; 3];
        for &v in vertices {
            for (axis, &c) in coordinates[v as usize].iter().enumerate() {
                low[axis] = low[axis].min(c);
                high[axis] = high[axis].max(c);
            }
        }
        let mut axes: Vec<(usize, f64)> = (0..3)
            .map(|a| (a, high[a] - low[a]))
            .filter(|&(_, e)| e > 0.0 && e.is_finite())
            .collect();
        axes.sort_by(|a, b| b.1.total_cmp(&a.1));
        axes.into_iter().map(|(a, _)| a).collect()
    }

    /// Splits the subgraph along `axis` where `fraction` of its weight is reached; `None`
    /// when the vertices all lie at one coordinate. Returns the vertices of the two sides,
    /// each nonempty.
    fn geometric_bisection(
        &self,
        sorted: &[u32],
        axis: usize,
        fraction: f64,
    ) -> Option<(Vec<u32>, Vec<u32>)> {
        let coordinates = self.coordinates.as_ref()?;
        let total: usize = (sorted.iter())
            .map(|&v| self.graph.weight[v as usize] as usize)
            .sum();
        let mut cumulative = 0usize;
        let mut split = sorted.len();
        for (i, &v) in sorted.iter().enumerate() {
            cumulative += self.graph.weight[v as usize] as usize;
            if cumulative as f64 >= fraction * total as f64 {
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
            // Everything up to the cut has one coordinate: cut below it instead.
            split = sorted
                .iter()
                .position(|&v| coordinates[v as usize][axis] == cut_value)?;
            if split == 0 {
                return None;
            }
        }
        Some((sorted[..split].to_vec(), sorted[split..].to_vec()))
    }

    /// The vertices sorted by their coordinate along `axis`.
    fn sorted_along(&self, vertices: &[u32], axis: usize) -> Vec<u32> {
        let coordinates = self.coordinates.as_ref().expect("coordinates");
        let mut sorted: Vec<u32> = vertices.to_vec();
        sorted.sort_unstable_by(|&a, &b| {
            coordinates[a as usize][axis].total_cmp(&coordinates[b as usize][axis])
        });
        sorted
    }

    /// The vertex separator of a subgraph split into the sides `a` and `b`: a minimum
    /// vertex cover of the edges between the sides (König's theorem on a maximum matching
    /// of the bipartite boundary graph), which is never larger than the smaller boundary.
    /// Returns the separator and its weight.
    fn separator(&mut self, a: &[u32], b: &[u32]) -> (Vec<u32>, usize) {
        for &v in a {
            self.side[v as usize] = Side::A;
        }
        for &v in b {
            self.side[v as usize] = Side::B;
        }
        // Boundary vertices of A (left) and B (right) and the edges between them.
        let mut left: Vec<u32> = Vec::new();
        let mut right_index: HashMap<u32, u32> = HashMap::new();
        let mut right: Vec<u32> = Vec::new();
        let mut edges: Vec<Vec<u32>> = Vec::new();
        for &v in a {
            let mut list = Vec::new();
            for &u in self.graph.neighbors(v) {
                if self.selected(u) && self.side[u as usize] == Side::B {
                    let next = right.len() as u32;
                    let j = *right_index.entry(u).or_insert_with(|| {
                        right.push(u);
                        next
                    });
                    list.push(j);
                }
            }
            if !list.is_empty() {
                left.push(v);
                edges.push(list);
            }
        }
        for &v in a.iter().chain(b) {
            self.side[v as usize] = Side::Out;
        }
        let cover = minimum_vertex_cover(&edges, right.len());
        let mut separator = Vec::with_capacity(cover.0.len() + cover.1.len());
        separator.extend(cover.0.iter().map(|&i| left[i as usize]));
        separator.extend(cover.1.iter().map(|&j| right[j as usize]));
        let weight = separator
            .iter()
            .map(|&v| self.graph.weight[v as usize] as usize)
            .sum();
        (separator, weight)
    }

    /// Improves a separator by moving its vertices into a side when fewer vertices of the
    /// other side have to take their place (the greedy half of Fiduccia-Mattheyses, until
    /// no move helps), keeping both sides at least a quarter of the subgraph.
    fn refine(&mut self, a: &mut Vec<u32>, b: &mut Vec<u32>, separator: &mut Vec<u32>) {
        let weight = |this: &Self, v: u32| this.graph.weight[v as usize] as usize;
        for &v in a.iter() {
            self.side[v as usize] = Side::A;
        }
        for &v in b.iter() {
            self.side[v as usize] = Side::B;
        }
        for &v in separator.iter() {
            self.side[v as usize] = Side::S;
        }
        let mut wa: usize = a.iter().map(|&v| weight(self, v)).sum();
        let mut wb: usize = b.iter().map(|&v| weight(self, v)).sum();
        let total = wa + wb + separator.iter().map(|&v| weight(self, v)).sum::<usize>();
        let floor = total / 4;
        let mut queue: Vec<u32> = separator.clone();
        let mut passes = 0;
        while !queue.is_empty() && passes < 8 {
            passes += 1;
            let mut next = Vec::new();
            for v in queue.drain(..) {
                if self.side[v as usize] != Side::S {
                    continue;
                }
                let (mut na, mut nb) = (0usize, 0usize);
                for &u in self.graph.neighbors(v) {
                    if self.selected(u) {
                        match self.side[u as usize] {
                            Side::A => na += weight(self, u),
                            Side::B => nb += weight(self, u),
                            _ => {}
                        }
                    }
                }
                let wv = weight(self, v);
                // To A: v leaves the separator, its B neighbors enter it.
                let gain_a = wv as isize - nb as isize;
                let gain_b = wv as isize - na as isize;
                let to_a = if gain_a >= gain_b {
                    gain_a > 0 && wb >= nb + floor
                } else {
                    false
                };
                let to_b = !to_a && gain_b > gain_a && gain_b > 0 && wa >= na + floor;
                if !(to_a || to_b) {
                    continue;
                }
                let (own, other, gained) = if to_a {
                    (Side::A, Side::B, nb)
                } else {
                    (Side::B, Side::A, na)
                };
                self.side[v as usize] = own;
                if to_a {
                    wa += wv;
                    wb -= gained;
                } else {
                    wb += wv;
                    wa -= gained;
                }
                for &u in self.graph.neighbors(v) {
                    if self.selected(u) && self.side[u as usize] == other {
                        self.side[u as usize] = Side::S;
                        next.push(u);
                    }
                }
                // Neighbors already in the separator may now have a better move.
                for &u in self.graph.neighbors(v) {
                    if self.selected(u) && self.side[u as usize] == Side::S {
                        next.push(u);
                    }
                }
            }
            queue = next;
        }
        let all: Vec<u32> = a
            .drain(..)
            .chain(b.drain(..))
            .chain(separator.drain(..))
            .collect();
        // The separator came as a subset of the sides: every vertex once.
        for v in all {
            match self.side[v as usize] {
                Side::A => a.push(v),
                Side::B => b.push(v),
                Side::S => separator.push(v),
                Side::Out => continue,
            }
            self.side[v as usize] = Side::Out;
        }
    }

    /// Orders a subgraph split into the sides `a` and `b` with the given `separator`
    /// (vertices of either side): both sides without it recursively, the separator last.
    fn order_split(&mut self, a: Vec<u32>, b: Vec<u32>, separator: Vec<u32>, work: &mut Vec<Task>) {
        for &v in &separator {
            self.side[v as usize] = Side::A;
        }
        let rest_a: Vec<u32> = (a.into_iter())
            .filter(|&v| self.side[v as usize] == Side::Out)
            .collect();
        let rest_b: Vec<u32> = (b.into_iter())
            .filter(|&v| self.side[v as usize] == Side::Out)
            .collect();
        for &v in &separator {
            self.side[v as usize] = Side::Out;
        }
        // The generation changes in the later tasks; the separator needs no further
        // searches. Last in, first out: side A, then side B, then the separator.
        work.push(Task::Emit(separator));
        work.push(Task::Order(rest_b));
        work.push(Task::Order(rest_a));
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

    /// Starts a new breadth-first search: no vertex counts as visited.
    fn new_search(&mut self) {
        self.search = self.search.wrapping_add(1);
        if self.search == 0 {
            self.seen.iter_mut().for_each(|s| *s = 0);
            self.search = 1;
        }
    }

    fn visited(&self, v: u32) -> bool {
        self.seen[v as usize] == self.search
    }

    /// Breadth-first search from `root` inside the current subgraph, skipping the vertices
    /// already visited in the current search (see [`Self::new_search`]); returns the vertices
    /// in the order reached and fills `level` with their distance from the root.
    fn bfs(&mut self, root: u32) -> Vec<u32> {
        let mut order = vec![root];
        self.seen[root as usize] = self.search;
        self.level[root as usize] = 0;
        let mut head = 0;
        while head < order.len() {
            let v = order[head];
            head += 1;
            let next = self.level[v as usize] + 1;
            for &u in self.graph.neighbors(v) {
                if self.selected(u) && !self.visited(u) {
                    self.seen[u as usize] = self.search;
                    self.level[u as usize] = next;
                    order.push(u);
                }
            }
        }
        order
    }

    /// Orders the subgraph `vertices` into `out`. Without recursion: a mesh can fall apart
    /// into many thousands of pieces (and the dissection tree can be deep), which overflowed
    /// the stack of a solver thread.
    fn order(&mut self, vertices: Vec<u32>, out: &mut Vec<u32>) {
        let mut work = vec![Task::Order(vertices)];
        while let Some(task) = work.pop() {
            match task {
                Task::Order(vertices) => self.order_one(vertices, out, &mut work),
                Task::Emit(vertices) => out.extend(vertices),
            }
        }
    }

    /// Orders the subgraph `vertices`: a leaf straight into `out`, else splits it and leaves
    /// the parts as tasks in `work`.
    fn order_one(&mut self, vertices: Vec<u32>, out: &mut Vec<u32>, work: &mut Vec<Task>) {
        if vertices.is_empty() {
            return;
        }
        let weight: usize = vertices
            .iter()
            .map(|&v| self.graph.weight[v as usize] as usize)
            .sum();
        self.select(&vertices);
        self.new_search();
        // Level structure from a pseudo-peripheral vertex: the farthest vertex of a search
        // from the farthest vertex of a first search.
        let start = vertices
            .iter()
            .copied()
            .min_by_key(|&v| self.graph.neighbors(v).len())
            .unwrap();
        let first = self.bfs(start);
        if first.len() < vertices.len() {
            // Disconnected: every component on its own, no separator between them. All
            // components in one pass, so that many small pieces cost linear time.
            let mut components = vec![first];
            for &v in &vertices {
                if !self.visited(v) {
                    components.push(self.bfs(v));
                }
            }
            work.extend(components.into_iter().rev().map(Task::Order));
            return;
        }
        if weight <= LEAF_WEIGHT || vertices.len() <= 2 {
            // Cuthill-McKee order from the far end of the first search.
            let far = *first.last().unwrap();
            self.new_search();
            let order = self.bfs(far);
            out.extend(order);
            return;
        }
        // Geometric: the cut with the smallest separator among the axes (the largest
        // extent first; a cut along another axis often has a much smaller separator on
        // meshes with holes and ribs).
        let mut best: Option<Split> = None;
        for axis in self.axes_by_extent(&vertices) {
            let sorted = self.sorted_along(&vertices, axis);
            for fraction in CUT_FRACTIONS {
                let Some((a, b)) = self.geometric_bisection(&sorted, axis, fraction) else {
                    continue;
                };
                let (separator, weight) = self.separator(&a, &b);
                if best.as_ref().is_none_or(|s| weight < s.weight) {
                    best = Some(Split {
                        a,
                        b,
                        separator,
                        weight,
                    });
                }
            }
        }
        if let Some(Split {
            mut a,
            mut b,
            mut separator,
            ..
        }) = best
        {
            self.refine(&mut a, &mut b, &mut separator);
            self.order_split(a, b, separator, work);
            return;
        }
        let far = *first.last().unwrap();
        self.new_search();
        let structure = self.bfs(far);
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
        let (mut a, mut b) = (a, b);
        let (mut separator, _) = self.separator(&a, &b);
        self.refine(&mut a, &mut b, &mut separator);
        self.order_split(a, b, separator, work);
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

/// A minimum vertex cover of a bipartite graph given as the neighbor lists `edges` of the
/// left vertices (indices into the `right_count` right vertices): the maximum matching by
/// augmenting paths (Kuhn), then König's theorem: with `Z` the vertices reachable from the
/// unmatched left vertices along alternating paths, the cover is `(L \ Z) ∪ (R ∩ Z)`.
/// Returns the left and the right vertices of the cover.
fn minimum_vertex_cover(edges: &[Vec<u32>], right_count: usize) -> (Vec<u32>, Vec<u32>) {
    let left_count = edges.len();
    let mut match_right = vec![u32::MAX; right_count]; // right -> left
    let mut match_left = vec![u32::MAX; left_count]; // left -> right
    // Greedy start.
    for (i, list) in edges.iter().enumerate() {
        for &j in list {
            if match_right[j as usize] == u32::MAX {
                match_right[j as usize] = i as u32;
                match_left[i] = j;
                break;
            }
        }
    }
    // Augmenting paths, iteratively (a depth first search with an explicit stack).
    let mut visited = vec![u32::MAX; right_count];
    for start in 0..left_count {
        if match_left[start] != u32::MAX {
            continue;
        }
        // stack of (left vertex, next edge index)
        let mut stack: Vec<(u32, usize)> = vec![(start as u32, 0)];
        let mut path: Vec<(u32, u32)> = Vec::new(); // (left, right) edges taken
        let mut found = false;
        while let Some(&mut (l, ref mut k)) = stack.last_mut() {
            let list = &edges[l as usize];
            if *k >= list.len() {
                stack.pop();
                path.pop();
                continue;
            }
            let j = list[*k];
            *k += 1;
            if visited[j as usize] == start as u32 {
                continue;
            }
            visited[j as usize] = start as u32;
            path.push((l, j));
            let next = match_right[j as usize];
            if next == u32::MAX {
                found = true;
                break;
            }
            stack.push((next, 0));
        }
        if found {
            for &(l, j) in &path {
                match_left[l as usize] = j;
                match_right[j as usize] = l;
            }
        }
    }
    // Alternating reachability from the unmatched left vertices.
    let mut left_in_z = vec![false; left_count];
    let mut right_in_z = vec![false; right_count];
    let mut queue: Vec<u32> = (0..left_count as u32)
        .filter(|&i| match_left[i as usize] == u32::MAX)
        .collect();
    for &i in &queue {
        left_in_z[i as usize] = true;
    }
    let mut head = 0;
    while head < queue.len() {
        let l = queue[head] as usize;
        head += 1;
        for &j in &edges[l] {
            if match_left[l] == j || right_in_z[j as usize] {
                continue;
            }
            right_in_z[j as usize] = true;
            let m = match_right[j as usize];
            if m != u32::MAX && !left_in_z[m as usize] {
                left_in_z[m as usize] = true;
                queue.push(m);
            }
        }
    }
    let left: Vec<u32> = (0..left_count as u32)
        .filter(|&i| !left_in_z[i as usize])
        .collect();
    let right: Vec<u32> = (0..right_count as u32)
        .filter(|&j| right_in_z[j as usize])
        .collect();
    (left, right)
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
    fn vertex_cover_is_minimum() {
        // A path l0-r0-l1-r1: matching of size 2, cover of size 2.
        let edges = vec![vec![0], vec![0, 1]];
        let (l, r) = minimum_vertex_cover(&edges, 2);
        assert_eq!(l.len() + r.len(), 2, "{l:?} {r:?}");
        // A star: one left vertex to five right ones: cover = the left vertex.
        let edges = vec![vec![0, 1, 2, 3, 4]];
        let (l, r) = minimum_vertex_cover(&edges, 5);
        assert_eq!((l, r), (vec![0], vec![]));
        // Five left vertices to one right one: cover = the right vertex.
        let edges = vec![vec![0]; 5];
        let (l, r) = minimum_vertex_cover(&edges, 1);
        assert_eq!((l, r), (vec![], vec![0]));
        // Every edge is covered.
        let edges = vec![vec![0, 1], vec![1, 2], vec![2, 3], vec![0, 3], vec![1]];
        let (l, r) = minimum_vertex_cover(&edges, 4);
        for (i, list) in edges.iter().enumerate() {
            for &j in list {
                assert!(
                    l.contains(&(i as u32)) || r.contains(&j),
                    "{i}-{j} uncovered"
                );
            }
        }
        assert!(l.len() + r.len() <= 4);
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

    /// A frequency step of a 220k-node model crashed a solver thread: the dissection
    /// recursed once per piece of a graph that fell apart into many pieces and overflowed the
    /// 2 MB stack of the thread (and cleared a flag per vertex of the whole graph per piece).
    #[test]
    fn many_pieces_neither_overflow_the_stack_nor_take_quadratic_time() {
        let n = 300_000usize;
        let blocks: Vec<Vec<u32>> = (0..n / 3)
            .map(|i| vec![(3 * i) as u32, (3 * i + 1) as u32])
            .collect();
        let pattern = Pattern::from_blocks(n, blocks.iter().map(Vec::as_slice));
        let start = std::time::Instant::now();
        let perm = std::thread::Builder::new()
            .stack_size(256 << 10)
            .spawn(move || nested_dissection(&pattern, None))
            .unwrap()
            .join()
            .unwrap();
        assert!(is_permutation(&perm));
        assert_eq!(perm.len(), n);
        assert!(start.elapsed().as_secs() < 20, "took {:?}", start.elapsed());
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
