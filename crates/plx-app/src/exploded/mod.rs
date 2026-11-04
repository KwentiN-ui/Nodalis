//! Exploded view: the parts of an assembly are pulled apart, so that faces
//! inside it, such as the bearing faces under a bolt head, can be seen and picked without
//! hiding anything. Parts that share nodes stay together and move as one.
//!
//! Three methods compute where each group of parts goes, all ported from PrePoMax:
//! disassembly moves every part the way it would be taken off (see [`disassembly`]),
//! assembly centre pushes the parts away from the centre until their bounding boxes clear
//! each other, and centre point moves them away from a given point. The offsets are linear
//! in the scale factor, so a layout is computed once and only scaled while the user drags.
//!
//! The exploded view is a matter of display: the mesh keeps its coordinates and the model
//! draws and picks its nodes moved by the offset of their part.

pub mod dialog;
mod disassembly;

use std::time::{Duration, Instant};

use glam::DVec3;
use plx_mesh::{FeMesh, PartSkin};

pub use dialog::{ExplodedDialog, ExplodedResult};

/// How the offsets are computed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Method {
    /// Each part moves the way it would be taken off: along the normal of its mating face or
    /// along the axis of the bore that holds it.
    Disassembly,
    /// Each part moves away from the centre of the assembly until it clears the parts that
    /// are already spaced out.
    AssemblyCenter,
    /// Each part moves away from a point, the further the further away it already is.
    CenterPoint,
}

impl Method {
    pub const ALL: [Method; 3] = [Self::Disassembly, Self::AssemblyCenter, Self::CenterPoint];

    pub fn label(self) -> &'static str {
        match self {
            Self::Disassembly => "Disassembly",
            Self::AssemblyCenter => "Assembly center",
            Self::CenterPoint => "Center point",
        }
    }

    pub fn description(self) -> &'static str {
        match self {
            Self::Disassembly => {
                "Each part is moved the way it would be taken off: along the normal of its \
                 mating face or along the axis of the bore it sits in."
            }
            Self::AssemblyCenter => {
                "Each part is moved away from the center of the assembly until it no longer \
                 overlaps the parts that are already spaced out."
            }
            Self::CenterPoint => {
                "Each part is moved away from the entered point, the further the further it \
                 already is from it."
            }
        }
    }

    /// The upper limit of the magnification.
    pub fn max_magnification(self) -> f64 {
        match self {
            Self::Disassembly => 10.0,
            Self::AssemblyCenter => 25.0,
            Self::CenterPoint => 1000.0,
        }
    }
}

/// The global directions the parts may move along; the other components are dropped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Xyz,
    X,
    Y,
    Z,
    Xy,
    Xz,
    Yz,
}

impl Direction {
    pub const ALL: [Direction; 7] = [
        Self::Xyz,
        Self::X,
        Self::Y,
        Self::Z,
        Self::Xy,
        Self::Xz,
        Self::Yz,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Xyz => "XYZ",
            Self::X => "X",
            Self::Y => "Y",
            Self::Z => "Z",
            Self::Xy => "XY",
            Self::Xz => "XZ",
            Self::Yz => "YZ",
        }
    }

    /// The vector with the components outside the allowed directions set to zero.
    pub fn filter(self, v: DVec3) -> DVec3 {
        let [x, y, z] = match self {
            Self::Xyz => [true; 3],
            Self::X => [true, false, false],
            Self::Y => [false, true, false],
            Self::Z => [false, false, true],
            Self::Xy => [true, true, false],
            Self::Xz => [true, false, true],
            Self::Yz => [false, true, true],
        };
        DVec3::new(
            if x { v.x } else { 0.0 },
            if y { v.y } else { 0.0 },
            if z { v.z } else { 0.0 },
        )
    }
}

/// The exploded view parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct Parameters {
    pub method: Method,
    /// The point the parts move away from with [`Method::CenterPoint`].
    pub center: DVec3,
    pub direction: Direction,
    /// Position between the assembled (0) and the exploded (1) state.
    pub scale_factor: f64,
    /// How far apart the parts are at scale factor 1.
    pub magnification: f64,
    /// Distance below which faces of two parts touch for the disassembly method; zero takes
    /// one thousandth of the assembly's diagonal.
    pub tolerance: f64,
    /// Takes the disassembly apart one level of the assembly after the other.
    pub sequential: bool,
}

impl Default for Parameters {
    fn default() -> Self {
        Self {
            method: Method::Disassembly,
            center: DVec3::ZERO,
            direction: Direction::Xyz,
            scale_factor: 0.5,
            magnification: 2.0,
            tolerance: 0.0,
            sequential: false,
        }
    }
}

impl Parameters {
    /// The factor the layout is scaled with.
    pub fn scale(&self) -> f64 {
        self.scale_factor * self.magnification
    }

    /// Keeps the values in the ranges.
    pub fn clamp(&mut self) {
        self.scale_factor = self.scale_factor.clamp(0.0, 1.0);
        self.magnification = self
            .magnification
            .clamp(1.0, self.method.max_magnification());
        self.tolerance = self.tolerance.max(0.0);
    }

    /// What the layout depends on; the scale factor, the magnification and the sequence
    /// only scale it.
    fn layout_key(&self) -> LayoutKey {
        LayoutKey {
            method: self.method,
            center: (self.method == Method::CenterPoint).then_some(self.center.to_array()),
            direction: self.direction,
            tolerance: (self.method == Method::Disassembly).then_some(self.tolerance),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
struct LayoutKey {
    method: Method,
    center: Option<[f64; 3]>,
    direction: Direction,
    tolerance: Option<f64>,
}

/// The offset of every part at scale 1, split into the steps of the disassembly in which the
/// part moves: entry k holds how far the part moves while the k-th level of the assembly
/// comes off. The other methods move every part in one step.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Layout {
    steps: Vec<Vec<DVec3>>,
}

impl Layout {
    /// Number of levels a sequential explosion takes; one where it is the same as a
    /// simultaneous one.
    pub fn step_count(&self) -> usize {
        self.steps.iter().map(Vec::len).max().unwrap_or(0).max(1)
    }

    /// The offset of every part. Scaled step by step, the scale range up to the full
    /// separation is divided among the levels: the first level comes off first and carries
    /// what is mounted on it along, then the next one, and so on.
    pub fn offsets(&self, scale: f64, sequential: bool) -> Vec<DVec3> {
        let count = self.step_count();
        let step_by_step = sequential && count > 1 && scale < 1.0;
        self.steps
            .iter()
            .map(|steps| {
                steps
                    .iter()
                    .enumerate()
                    .map(|(i, &step)| {
                        let factor = if step_by_step {
                            (scale * count as f64 - i as f64).clamp(0.0, 1.0)
                        } else {
                            scale
                        };
                        step * factor
                    })
                    .sum()
            })
            .collect()
    }

    /// The scale factors at which a sequential animation stops: the level boundaries passed
    /// between the two ends, then the end itself.
    pub fn sequence(&self, from: f64, to: f64) -> Vec<f64> {
        let count = self.step_count();
        let (min, max) = (from.min(to), from.max(to));
        let mut stops: Vec<f64> = (1..=count)
            .map(|i| i as f64 / count as f64)
            .filter(|&s| count > 1 && s > min && s < max)
            .collect();
        if to < from {
            stops.reverse();
        }
        stops.push(to);
        stops
    }
}

/// An axis-aligned bounding box.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Aabb {
    pub min: DVec3,
    pub max: DVec3,
}

impl Aabb {
    pub const EMPTY: Self = Self {
        min: DVec3::splat(f64::MAX),
        max: DVec3::splat(-f64::MAX),
    };

    pub fn include_point(&mut self, p: DVec3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }

    pub fn include(&mut self, other: &Aabb) {
        self.min = self.min.min(other.min);
        self.max = self.max.max(other.max);
    }

    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x
    }

    pub fn center(&self) -> DVec3 {
        (self.min + self.max) * 0.5
    }

    pub fn diagonal(&self) -> f64 {
        if self.is_empty() {
            0.0
        } else {
            (self.max - self.min).length()
        }
    }

    pub fn volume(&self) -> f64 {
        let size = self.max - self.min;
        size.x * size.y * size.z
    }

    /// Touching boxes intersect.
    pub fn intersects(&self, other: &Aabb) -> bool {
        (self.min.cmple(other.max) & self.max.cmpge(other.min)).all()
    }

    pub fn inflate(&mut self, by: f64) {
        self.min -= DVec3::splat(by);
        self.max += DVec3::splat(by);
    }

    pub fn translate(&mut self, by: DVec3) {
        self.min += by;
        self.max += by;
    }

    /// Scales the box about its centre.
    pub fn scale(&mut self, factor: f64) {
        let delta = (self.max - self.min) * 0.5 * (factor - 1.0);
        self.min -= delta;
        self.max += delta;
    }

    /// Gives boxes of flat parts some thickness, so that they can be told apart.
    pub fn inflate_if_thin(&mut self, factor: f64) {
        let diagonal = self.diagonal();
        let size = self.max - self.min;
        let thin = diagonal < 1e-12 || size.min_element() < diagonal * 1e-2;
        if thin {
            if diagonal < 1e-6 {
                self.inflate(factor);
            }
            self.inflate(diagonal * factor);
        }
    }

    /// The interval the box covers along a direction.
    pub fn projection(&self, direction: DVec3) -> (f64, f64) {
        let center = direction.dot(self.center());
        let extent = direction.abs().dot(self.max - self.min) * 0.5;
        (center - extent, center + extent)
    }
}

/// Rounds to significant digits to compare sizes, so that numerical noise does not
/// change the order.
pub(crate) fn round_significant(value: f64, digits: i32) -> f64 {
    if value == 0.0 || !value.is_finite() {
        return value;
    }
    let scale = 10f64.powi(digits - 1 - value.abs().log10().floor() as i32);
    (value * scale).round() / scale
}

/// One part as the exploded view sees it.
pub struct PartShape<'a> {
    pub name: &'a str,
    /// Indices of the part's nodes into the coordinates.
    pub nodes: &'a [usize],
    pub skin: &'a PartSkin,
}

/// The parts of a mesh.
pub struct Assembly<'a> {
    pub mesh: &'a FeMesh,
    pub parts: Vec<PartShape<'a>>,
}

impl Assembly<'_> {
    fn coords(&self) -> &[[f64; 3]] {
        self.mesh.coords()
    }

    fn part_box(&self, part: usize) -> Aabb {
        let mut bounds = Aabb::EMPTY;
        for &node in self.parts[part].nodes {
            bounds.include_point(DVec3::from(self.coords()[node]));
        }
        bounds
    }

    /// Groups of parts connected by shared nodes, which move as one; parts without nodes
    /// belong to none.
    fn connected_parts(&self) -> Vec<Vec<usize>> {
        let mut owner = vec![usize::MAX; self.coords().len()];
        let mut parent: Vec<usize> = (0..self.parts.len()).collect();
        fn find(parent: &mut [usize], mut i: usize) -> usize {
            while parent[i] != i {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            i
        }
        for (part, shape) in self.parts.iter().enumerate() {
            for &node in shape.nodes {
                match owner[node] {
                    usize::MAX => owner[node] = part,
                    other => {
                        let (a, b) = (find(&mut parent, other), find(&mut parent, part));
                        parent[a.max(b)] = a.min(b);
                    }
                }
            }
        }
        let mut groups: Vec<Vec<usize>> = Vec::new();
        let mut group_of = vec![usize::MAX; self.parts.len()];
        for part in 0..self.parts.len() {
            if self.parts[part].nodes.is_empty() {
                continue;
            }
            let root = find(&mut parent, part);
            if group_of[root] == usize::MAX {
                group_of[root] = groups.len();
                groups.push(Vec::new());
            }
            groups[group_of[root]].push(part);
        }
        groups
    }

    /// The offsets of all parts at scale 1. The disassembly method falls back to the
    /// assembly centre when the model gives it nothing to work with.
    pub fn layout(&self, parameters: &Parameters) -> Layout {
        let groups = self.connected_parts();
        let group_offsets: Vec<Vec<DVec3>> = match parameters.method {
            Method::Disassembly => {
                disassembly::layout(self, &groups, parameters.direction, parameters.tolerance)
                    .unwrap_or_else(|| self.assembly_center(&groups, parameters.direction))
            }
            Method::AssemblyCenter => self.assembly_center(&groups, parameters.direction),
            Method::CenterPoint => {
                self.center_point(&groups, parameters.center, parameters.direction)
            }
        };
        let mut steps = vec![Vec::new(); self.parts.len()];
        for (group, offsets) in groups.iter().zip(group_offsets) {
            for &part in group {
                steps[part] = offsets.clone();
            }
        }
        Layout { steps }
    }

    fn group_boxes(&self, groups: &[Vec<usize>]) -> Vec<Aabb> {
        groups
            .iter()
            .map(|group| {
                let mut bounds = Aabb::EMPTY;
                for &part in group {
                    bounds.include(&self.part_box(part));
                }
                bounds.inflate_if_thin(0.1);
                bounds
            })
            .collect()
    }

    /// The assembly centre method: the largest group stays, every further group, from
    /// the largest to the smallest, moves away from the centre of what is placed so far in
    /// small steps until its enlarged box clears all placed boxes.
    fn assembly_center(&self, groups: &[Vec<usize>], direction: Direction) -> Vec<Vec<DVec3>> {
        let mut boxes = self.group_boxes(groups);
        for b in &mut boxes {
            b.scale(1.2);
        }
        let mut order: Vec<usize> = (0..boxes.len()).collect();
        order.sort_by(|&a, &b| {
            round_significant(boxes[b].volume(), 6)
                .total_cmp(&round_significant(boxes[a].volume(), 6))
        });
        let mut offsets = vec![DVec3::ZERO; boxes.len()];
        let Some((&first, rest)) = order.split_first() else {
            return Vec::new();
        };
        let mut global = boxes[first];
        let mut placed = vec![boxes[first]];
        for &index in rest {
            let mut moving = boxes[index];
            let mut step = direction.filter(moving.center() - global.center());
            if step.length_squared() < (1e-6 * global.diagonal()).powi(2) {
                step = direction.filter(DVec3::ONE);
            }
            step = step.normalize_or_zero() * (0.01 * global.diagonal());
            let mut offset = DVec3::ZERO;
            let mut count = 0;
            while placed.iter().any(|p| moving.intersects(p)) && count < 10_000 {
                moving.translate(step);
                offset += step;
                count += 1;
            }
            placed.push(moving);
            global.include(&moving);
            offsets[index] = offset;
        }
        offsets.into_iter().map(|o| vec![o]).collect()
    }

    /// The centre point method: every group moves away from the point by twice its
    /// distance from it.
    fn center_point(
        &self,
        groups: &[Vec<usize>],
        center: DVec3,
        direction: Direction,
    ) -> Vec<Vec<DVec3>> {
        self.group_boxes(groups)
            .iter()
            .map(|b| {
                let away = b.center() - center;
                let distance = away.length();
                vec![direction.filter(away).normalize_or_zero() * distance * 2.0]
            })
            .collect()
    }
}

/// The easing of the exploded view animation: slow at both ends.
fn ease(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    0.5 + 0.5 * (t * 6.0 - 3.0).atan() / 3f64.atan()
}

/// Duration of an animated change.
pub const ANIMATION_TIME: Duration = Duration::from_millis(500);
/// Duration of each level of a sequential animation.
pub const STEP_TIME: Duration = Duration::from_millis(400);

/// The exploded view of one model: what is applied, the cached layout, the offsets shown now
/// and an animation towards new offsets.
#[derive(Default)]
pub struct Explosion {
    /// The applied exploded view; `None` when the model is shown assembled.
    pub applied: Option<Parameters>,
    cache: Option<(LayoutKey, Layout)>,
    /// Offset of every part as shown; empty when nothing is moved.
    offsets: Vec<DVec3>,
    /// Targets still to move to, each over its own time, and when the current one started
    /// from which offsets.
    animation: Vec<(Vec<DVec3>, Duration)>,
    started: Option<(Instant, Vec<DVec3>)>,
    /// Counts changes of the shown offsets, so that what is built from them is rebuilt.
    version: u64,
}

impl Explosion {
    pub fn offsets(&self) -> &[DVec3] {
        &self.offsets
    }

    pub fn version(&self) -> u64 {
        self.version
    }

    pub fn is_animating(&self) -> bool {
        !self.animation.is_empty()
    }

    /// Whether anything is moved.
    pub fn is_shown(&self) -> bool {
        self.offsets.iter().any(|o| *o != DVec3::ZERO)
    }

    /// The layout for the parameters, computed when they changed what it depends on.
    pub fn layout(&mut self, assembly: &Assembly, parameters: &Parameters) -> &Layout {
        let key = parameters.layout_key();
        if self.cache.as_ref().is_none_or(|(k, _)| *k != key) {
            self.cache = Some((key, assembly.layout(parameters)));
        }
        &self.cache.as_ref().expect("layout just computed").1
    }

    /// Forgets the layout, e.g. after the mesh changed.
    pub fn clear_layout(&mut self) {
        self.cache = None;
    }

    /// Shows offsets at once or animates towards them.
    pub fn show(&mut self, offsets: Vec<DVec3>, animate: bool) {
        self.show_sequence(vec![(offsets, ANIMATION_TIME)], animate);
    }

    /// Animates through several offsets one after the other, or shows the last at once.
    pub fn show_sequence(&mut self, targets: Vec<(Vec<DVec3>, Duration)>, animate: bool) {
        // A running animation continues from where it is.
        let current = self.offsets.clone();
        self.animation.clear();
        self.started = None;
        if animate {
            self.animation = targets;
            self.started = Some((Instant::now(), current));
        } else if let Some((last, _)) = targets.into_iter().last() {
            self.set(last);
        }
    }

    /// Advances the animation; returns whether the shown offsets changed.
    pub fn tick(&mut self, now: Instant) -> bool {
        let Some((start, from)) = self.started.take() else {
            return false;
        };
        let Some((target, duration)) = self.animation.first().cloned() else {
            return false;
        };
        let t = now.duration_since(start).as_secs_f64() / duration.as_secs_f64().max(1e-9);
        if t >= 1.0 {
            self.animation.remove(0);
            if !self.animation.is_empty() {
                self.started = Some((now, target.clone()));
            }
            self.set(target);
            return true;
        }
        let s = ease(t);
        let at = |i: usize, list: &[DVec3]| list.get(i).copied().unwrap_or(DVec3::ZERO);
        let count = target.len().max(from.len());
        let offsets = (0..count)
            .map(|i| at(i, &from).lerp(at(i, &target), s))
            .collect();
        self.started = Some((start, from));
        self.set(offsets);
        true
    }

    fn set(&mut self, offsets: Vec<DVec3>) {
        if offsets != self.offsets {
            self.offsets = offsets;
            self.version += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube_skin() -> PartSkin {
        PartSkin::default()
    }

    /// Three unit cubes in a row along x touching each other, given by their corners only.
    fn row() -> (FeMesh, Vec<Vec<usize>>) {
        let mut mesh = FeMesh::default();
        let mut parts = Vec::new();
        for i in 0..3 {
            let start = mesh.node_count();
            for c in 0..8 {
                let x = i as f64 + (c & 1) as f64;
                let id = mesh.node_count() as u32 + 1;
                mesh.set_node(id, [x, ((c >> 1) & 1) as f64, ((c >> 2) & 1) as f64]);
            }
            parts.push((start..start + 8).collect());
        }
        (mesh, parts)
    }

    #[test]
    fn assembly_center_moves_outer_parts_apart_and_keeps_the_largest() {
        let (mesh, nodes) = row();
        let skin = cube_skin();
        let names = ["A", "B", "C"];
        let assembly = Assembly {
            mesh: &mesh,
            parts: (0..3)
                .map(|i| PartShape {
                    name: names[i],
                    nodes: &nodes[i],
                    skin: &skin,
                })
                .collect(),
        };
        let parameters = Parameters {
            method: Method::AssemblyCenter,
            ..Parameters::default()
        };
        let offsets = assembly.layout(&parameters).offsets(1.0, false);
        // All cubes are equal; the first one stays, the others move away from it.
        assert_eq!(offsets[0], DVec3::ZERO);
        assert!(offsets[1].x > 0.0 && offsets[2].x > offsets[1].x - 1e-9);
        // Filtering to Y moves only along Y.
        let along_y = Parameters {
            direction: Direction::Y,
            ..parameters
        };
        let offsets = assembly.layout(&along_y).offsets(1.0, false);
        assert!(offsets.iter().all(|o| o.x == 0.0 && o.z == 0.0));
    }

    #[test]
    fn center_point_moves_twice_the_distance() {
        let (mesh, nodes) = row();
        let skin = cube_skin();
        let assembly = Assembly {
            mesh: &mesh,
            parts: (0..3)
                .map(|i| PartShape {
                    name: "P",
                    nodes: &nodes[i],
                    skin: &skin,
                })
                .collect(),
        };
        let parameters = Parameters {
            method: Method::CenterPoint,
            center: DVec3::new(1.5, 0.5, 0.5),
            ..Parameters::default()
        };
        let offsets = assembly.layout(&parameters).offsets(0.5, false);
        assert!((offsets[0] - DVec3::new(-1.0, 0.0, 0.0)).length() < 1e-9);
        assert!(offsets[1].length() < 1e-9);
        assert!((offsets[2] - DVec3::new(1.0, 0.0, 0.0)).length() < 1e-9);
    }

    #[test]
    fn parts_sharing_nodes_move_together() {
        let (mesh, mut nodes) = row();
        // The second part takes one node of the first one.
        nodes[1][0] = nodes[0][1];
        let skin = cube_skin();
        let assembly = Assembly {
            mesh: &mesh,
            parts: (0..3)
                .map(|i| PartShape {
                    name: "P",
                    nodes: &nodes[i],
                    skin: &skin,
                })
                .collect(),
        };
        assert_eq!(assembly.connected_parts(), [vec![0, 1], vec![2]]);
    }

    #[test]
    fn sequential_steps_divide_the_scale_range() {
        let layout = Layout {
            steps: vec![vec![], vec![DVec3::X], vec![DVec3::X, DVec3::Y]],
        };
        assert_eq!(layout.step_count(), 2);
        // Half way only the first level has moved.
        let half = layout.offsets(0.5, true);
        assert_eq!(half, [DVec3::ZERO, DVec3::X, DVec3::X]);
        assert_eq!(layout.offsets(0.5, false)[2], DVec3::new(0.5, 0.5, 0.0));
        assert_eq!(layout.offsets(1.0, true), layout.offsets(1.0, false));
        assert_eq!(layout.sequence(0.0, 1.0), [0.5, 1.0]);
        assert_eq!(layout.sequence(1.0, 0.0), [0.5, 0.0]);
    }

    #[test]
    fn animation_ends_at_the_target() {
        let mut explosion = Explosion::default();
        explosion.show(vec![DVec3::X], true);
        assert!(explosion.is_animating());
        let start = Instant::now();
        explosion.tick(start + ANIMATION_TIME * 2);
        assert!(!explosion.is_animating());
        assert_eq!(explosion.offsets(), [DVec3::X]);
    }

    #[test]
    fn rounding_keeps_significant_digits() {
        assert_eq!(round_significant(123.456_789, 6), 123.457);
        assert_eq!(round_significant(0.001_234_567, 3), 0.001_23);
    }
}
