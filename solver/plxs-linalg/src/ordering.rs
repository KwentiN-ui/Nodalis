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
    let clock = std::time::Instant::now();
    let graph = Graph::from_pattern(pattern);
    crate::solver::trace(&clock, "dissection: graph");
    let (quotient, members) = graph.compress();
    crate::solver::trace(
        &clock,
        &format!(
            "dissection: {n} unknowns compressed to {} vertices{}",
            quotient.len(),
            if coordinates.is_some_and(|c| c.len() == n) {
                ""
            } else {
                " (no coordinates)"
            }
        ),
    );
    let coordinates = coordinates.filter(|c| c.len() == n).map(|c| {
        (members.iter())
            .map(|m| c[m[0] as usize])
            .collect::<Vec<[f64; 3]>>()
    });
    let all: Vec<u32> = (0..quotient.len() as u32).collect();
    let order = order_parallel(&quotient, coordinates.as_deref(), all, 0);
    crate::solver::trace(&clock, "dissection: order");
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

/// Subgraphs with at least this many vertices have their two sides ordered in parallel.
const PARALLEL_VERTICES: usize = 20_000;

/// Orders `vertices` of `graph` like [`Dissection::order`], the two sides of every large
/// split in parallel, each with its own search state; the splits below
/// [`PARALLEL_VERTICES`] (and the leaves, components and separators) on one thread.
fn order_parallel(
    graph: &Graph,
    coordinates: Option<&[[f64; 3]]>,
    vertices: Vec<u32>,
    depth: u32,
) -> Vec<u32> {
    let mut dissection = Dissection::new(graph, coordinates);
    let mut out = Vec::with_capacity(vertices.len());
    if vertices.len() < PARALLEL_VERTICES || depth > 16 || rayon::current_num_threads() < 2 {
        dissection.order(vertices, &mut out);
        return out;
    }
    let mut work = Vec::new();
    dissection.order_one(vertices, &mut out, &mut work);
    // A split leaves [separator, side B, side A] (done last in, first out).
    if let [Task::Emit(_), Task::Order(_), Task::Order(_)] = work.as_slice() {
        let (Some(Task::Order(a)), Some(Task::Order(b)), Some(Task::Emit(separator))) =
            (work.pop(), work.pop(), work.pop())
        else {
            unreachable!("matched above")
        };
        let (a, b) = rayon::join(
            || order_parallel(graph, coordinates, a, depth + 1),
            || order_parallel(graph, coordinates, b, depth + 1),
        );
        out.extend(a);
        out.extend(b);
        out.extend(separator);
        return out;
    }
    while let Some(task) = work.pop() {
        match task {
            Task::Order(vertices) => dissection.order(vertices, &mut out),
            Task::Emit(vertices) => out.extend(vertices),
            Task::Leaf(vertices) => dissection.order_leaf(vertices, &mut out),
        }
    }
    out
}

struct Dissection<'a> {
    graph: &'a Graph,
    /// Positions of the (quotient) vertices, when known.
    coordinates: Option<&'a [[f64; 3]]>,
    /// Generation stamp of the subgraph a vertex belongs to during a search.
    stamp: Vec<u32>,
    generation: u32,
    level: Vec<u32>,
    side: Vec<Side>,
    /// Index of a vertex among the right (side B) boundary vertices while a separator is
    /// computed, `u32::MAX` otherwise.
    right_slot: Vec<u32>,
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
    /// Order this (connected) subgraph without splitting it.
    Leaf(Vec<u32>),
}

impl<'a> Dissection<'a> {
    fn new(graph: &'a Graph, coordinates: Option<&'a [[f64; 3]]>) -> Self {
        Self {
            graph,
            coordinates,
            stamp: vec![0; graph.len()],
            generation: 0,
            level: vec![0; graph.len()],
            side: vec![Side::Out; graph.len()],
            right_slot: vec![u32::MAX; graph.len()],
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
        let mut right: Vec<u32> = Vec::new();
        let mut edges: Vec<Vec<u32>> = Vec::new();
        for &v in a {
            let mut list = Vec::new();
            for &u in self.graph.neighbors(v) {
                if self.selected(u) && self.side[u as usize] == Side::B {
                    let mut j = self.right_slot[u as usize];
                    if j == u32::MAX {
                        j = right.len() as u32;
                        self.right_slot[u as usize] = j;
                        right.push(u);
                    }
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
        for &u in &right {
            self.right_slot[u as usize] = u32::MAX;
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
        // The separator comes as a subset of the sides: its vertices count for neither side.
        // Counted in, the sides looked heavier than they are, the floor below let the moves
        // drain one side completely, and the subgraph came back whole, to be split again
        // forever (a free model of a ball, a can and a plate hung before its first output).
        let side_weight = |this: &Self, vertices: &[u32], side: Side| -> usize {
            (vertices.iter())
                .filter(|&&v| this.side[v as usize] == side)
                .map(|&v| weight(this, v))
                .sum()
        };
        let mut wa = side_weight(self, a, Side::A);
        let mut wb = side_weight(self, b, Side::B);
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
        if separator.is_empty() && (rest_a.is_empty() || rest_b.is_empty()) {
            // No smaller pieces: splitting the same subgraph again would never end. Ordered
            // as a leaf (Cuthill-McKee) instead.
            let mut whole = rest_a;
            whole.extend(rest_b);
            work.push(Task::Leaf(whole));
            return;
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
                Task::Leaf(vertices) => self.order_leaf(vertices, out),
            }
        }
    }

    /// Orders the connected subgraph `vertices` into `out` without splitting it: Cuthill-McKee
    /// from a pseudo-peripheral vertex.
    fn order_leaf(&mut self, vertices: Vec<u32>, out: &mut Vec<u32>) {
        let Some(start) = (vertices.iter().copied()).min_by_key(|&v| self.graph.neighbors(v).len())
        else {
            return;
        };
        self.select(&vertices);
        self.new_search();
        let first = self.bfs(start);
        let far = *first.last().unwrap();
        self.new_search();
        let mut order = self.bfs(far);
        // Pieces the searches did not reach (the subgraph is connected, but to be safe).
        for &v in &vertices {
            if !self.visited(v) {
                order.extend(self.bfs(v));
            }
        }
        out.extend(order);
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
    // Hopcroft-Karp: phases of shortest augmenting paths, found by a breadth first search
    // from all unmatched left vertices and taken vertex-disjointly by depth first searches
    // along its layers. Simple augmenting paths one at a time (Kuhn) can search the whole
    // boundary graph for every unmatched vertex: a cut along the plates of a welded sheet
    // metal girder has boundaries of fifty thousand vertices, and its ordering took 30 s.
    const FAR: u32 = u32::MAX;
    let mut distance = vec![FAR; left_count];
    let mut next_edge = vec![0usize; left_count];
    let mut queue: Vec<u32> = Vec::with_capacity(left_count);
    let mut stack: Vec<u32> = Vec::new();
    loop {
        queue.clear();
        for (i, d) in distance.iter_mut().enumerate() {
            if match_left[i] == u32::MAX {
                *d = 0;
                queue.push(i as u32);
            } else {
                *d = FAR;
            }
        }
        let mut found = false;
        let mut head = 0;
        while head < queue.len() {
            let l = queue[head] as usize;
            head += 1;
            for &j in &edges[l] {
                let m = match_right[j as usize];
                if m == u32::MAX {
                    found = true;
                } else if distance[m as usize] == FAR {
                    distance[m as usize] = distance[l] + 1;
                    queue.push(m);
                }
            }
        }
        if !found {
            break;
        }
        next_edge.iter_mut().for_each(|e| *e = 0);
        for start in 0..left_count {
            if match_left[start] != u32::MAX || distance[start] != 0 {
                continue;
            }
            stack.clear();
            stack.push(start as u32);
            while let Some(&l) = stack.last() {
                let l = l as usize;
                let list = &edges[l];
                if next_edge[l] >= list.len() {
                    // A dead end for this phase.
                    distance[l] = FAR;
                    stack.pop();
                    if let Some(&parent) = stack.last() {
                        next_edge[parent as usize] += 1;
                    }
                    continue;
                }
                let j = list[next_edge[l]];
                let m = match_right[j as usize];
                if m == u32::MAX {
                    // Augment along the stack: every left vertex takes its current edge.
                    for &l in &stack {
                        let j = edges[l as usize][next_edge[l as usize]];
                        match_left[l as usize] = j;
                        match_right[j as usize] = l;
                    }
                    break;
                }
                if distance[m as usize] != FAR && distance[m as usize] == distance[l] + 1 {
                    stack.push(m);
                } else {
                    next_edge[l] += 1;
                }
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
    fn the_parallel_dissection_orders_like_the_sequential_one() {
        // 27k vertices: the first splits are ordered in parallel.
        let (nx, ny, nz) = (30, 30, 30);
        let pattern = grid_pattern(nx, ny, nz, 1);
        let coordinates = grid_coordinates(nx, ny, nz, 1);
        assert!(pattern.dimension() >= PARALLEL_VERTICES);
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(4)
            .build()
            .unwrap();
        let parallel = pool.install(|| nested_dissection(&pattern, Some(&coordinates)));
        let graph = Graph::from_pattern(&pattern);
        let (quotient, members) = graph.compress();
        assert_eq!(quotient.len(), pattern.dimension());
        let positions: Vec<[f64; 3]> = members.iter().map(|m| coordinates[m[0] as usize]).collect();
        let mut sequential = Vec::new();
        Dissection::new(&quotient, Some(&positions))
            .order((0..quotient.len() as u32).collect(), &mut sequential);
        let sequential: Vec<u32> = (sequential.iter())
            .flat_map(|&v| members[v as usize].iter().copied())
            .collect();
        assert_eq!(parallel, sequential);
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

    /// The subgraph that hung the ordering of a free model (a ball, a shell can and a shell
    /// plate in contact): nodes of a plate with 6, 9 and 15 unknowns each. The refinement of
    /// its split counted the separator into both sides, drained side A and gave the whole
    /// subgraph back, which was split the same way again forever.
    #[test]
    fn a_split_that_drains_a_side_ends() {
        let vertices: [(u32, u32, [f64; 2], &[u32]); 10] = [
            (
                4200,
                6,
                [0.05409620985141139, -0.017032783851346545],
                &[4044, 4056, 4198, 4199, 4220],
            ),
            (
                4220,
                6,
                [0.061173798994375916, -0.016740033392685115],
                &[3915, 3928, 4044, 4056, 4199, 4200],
            ),
            (
                4056,
                9,
                [0.05734759798875184, -0.014908638213941672],
                &[3915, 3928, 4044, 4198, 4199, 4200, 4220],
            ),
            (
                3928,
                6,
                [0.065, -0.013928571428571415],
                &[3915, 4044, 4056, 4199, 4220],
            ),
            (
                4199,
                6,
                [0.05713680281343876, -0.011777208683497742],
                &[3915, 3928, 4044, 4056, 4198, 4200, 4220],
            ),
            (
                3915,
                15,
                [0.065, -0.009285714285714272],
                &[3927, 3928, 4044, 4056, 4199, 4201, 4220],
            ),
            (
                4198,
                6,
                [0.05283134753476454, -0.008918217815723259],
                &[4044, 4056, 4199, 4200, 4201],
            ),
            (
                4044,
                9,
                [0.05692600763812569, -0.008645779153053812],
                &[3915, 3927, 3928, 4056, 4198, 4199, 4200, 4201, 4220],
            ),
            (3927, 6, [0.065, -0.004642857142857129], &[3915, 4044, 4201]),
            (
                4201,
                6,
                [0.05669205553947769, -0.004189715681346892],
                &[3915, 3927, 4044, 4198],
            ),
        ];
        // The unknowns of each node, numbered one node after the other.
        let mut first = std::collections::HashMap::new();
        let mut n = 0usize;
        let mut coordinates = Vec::new();
        for (id, weight, [x, y], _) in vertices {
            first.insert(id, n);
            n += weight as usize;
            coordinates.extend(std::iter::repeat_n([x, y, -0.0005], weight as usize));
        }
        let dofs = |id: u32| {
            let (_, weight, ..) = vertices.iter().find(|v| v.0 == id).unwrap();
            (first[&id]..first[&id] + *weight as usize).map(|d| d as u32)
        };
        let mut blocks: Vec<Vec<u32>> = Vec::new();
        for (id, _, _, neighbors) in vertices {
            blocks.push(dofs(id).collect());
            for &other in neighbors {
                blocks.push(dofs(id).chain(dofs(other)).collect());
            }
        }
        let pattern = Pattern::from_blocks(n, blocks.iter().map(Vec::as_slice));
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(nested_dissection(&pattern, Some(&coordinates)));
        });
        let perm = rx
            .recv_timeout(std::time::Duration::from_secs(20))
            .expect("the ordering did not end");
        assert_eq!(perm.len(), n);
        assert!(is_permutation(&perm));
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
