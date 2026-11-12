//! Coordinate systems. A system is given in a base system, the global one or another of the
//! features: by its origin, a point on its x axis and a point in its xy plane (like
//! CalculiX's `*ORIENTATION`), or by an offset of its origin and three turns about its axes.
//! Cylindrical systems use the same definitions; their z axis is the cylinder axis and r, θ, z
//! are the local directions.
//!
//! A system is fixed in space, or it moves with a selection of the model
//! ([`CoordinateSystem::attached`]): under large deformations it follows the rigid motion that
//! fits the motion of the selection's nodes best. A system given in a moving base moves with
//! the base. Loads in a moving system turn with it in the solver, results in it are taken in
//! its moved directions.
//!
//! [`CoordinateSystem::frame`] resolves a system to its origin and axes in global coordinates,
//! in the undeformed model; [`Frame::moved`] moves them with the selection.

use plx_mesh::{FeMesh, NodeId};
use serde::{Deserialize, Serialize};

use crate::{Id, Region};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum CoordinateSystemKind {
    #[default]
    Rectangular,
    Cylindrical,
}

impl CoordinateSystemKind {
    pub const ALL: [CoordinateSystemKind; 2] = [Self::Rectangular, Self::Cylindrical];

    pub fn label(self) -> &'static str {
        match self {
            Self::Rectangular => "Rectangular",
            Self::Cylindrical => "Cylindrical",
        }
    }
}

/// How a coordinate system lies in its base system. Coordinates and directions are taken in
/// the x, y and z axes of the base, also when the base is cylindrical.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SystemDefinition {
    /// By its origin, a point on the positive x axis and a point in the xy plane on the side of
    /// the positive y axis.
    ThreePoints {
        origin: [f64; 3],
        point_x: [f64; 3],
        point_xy: [f64; 3],
    },
    /// The base system moved by `offset` and then turned by `angles` (degrees): about its x
    /// axis, then about the turned y axis, then about the turned z axis.
    OffsetRotation { offset: [f64; 3], angles: [f64; 3] },
}

impl SystemDefinition {
    pub const LABELS: [&'static str; 2] = ["Three points", "Offset and rotation"];

    /// The place of the definition in [`SystemDefinition::LABELS`].
    pub fn index(&self) -> usize {
        match self {
            Self::ThreePoints { .. } => 0,
            Self::OffsetRotation { .. } => 1,
        }
    }

    /// The origin and the unit axes x, y, z in the base system.
    fn local_axes(&self, name: &str) -> Result<([f64; 3], [[f64; 3]; 3]), String> {
        match *self {
            Self::ThreePoints {
                origin,
                point_x,
                point_xy,
            } => {
                let x = sub(point_x, origin);
                let xy = sub(point_xy, origin);
                let size = norm(x).max(norm(xy));
                let Some(x) = normalize(x) else {
                    return Err(format!(
                        "{name}: The point on the x axis lies at the origin."
                    ));
                };
                let z = cross(x, xy);
                if norm(z) <= 1e-9 * size * size {
                    return Err(format!(
                        "{name}: The point in the xy plane lies on the x axis."
                    ));
                }
                let z = normalize(z).unwrap_or([0.0, 0.0, 1.0]);
                Ok((origin, [x, cross(z, x), z]))
            }
            Self::OffsetRotation { offset, angles } => {
                let mut axes = IDENTITY;
                // Each turn is about an axis of the system turned so far.
                for (k, angle) in angles.into_iter().enumerate() {
                    let turn = rotation(axes[k], angle.to_radians());
                    axes = axes.map(|a| apply(&turn, a));
                }
                Ok((offset, axes))
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CoordinateSystem {
    pub id: Id,
    pub name: String,
    pub kind: CoordinateSystemKind,
    /// The system the definition is given in; `None` for the global one.
    pub base: Option<Id>,
    pub definition: SystemDefinition,
    /// The selection the system moves with; `None` keeps it fixed in space, unless its base
    /// moves.
    pub attached: Option<Region>,
}

/// The three planes of a coordinate system.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CoordinatePlane {
    Xy,
    Yz,
    Xz,
}

impl CoordinatePlane {
    pub const ALL: [CoordinatePlane; 3] = [Self::Xy, Self::Yz, Self::Xz];

    pub fn label(self) -> &'static str {
        match self {
            Self::Xy => "XY",
            Self::Yz => "YZ",
            Self::Xz => "XZ",
        }
    }

    /// Index of the axis normal to the plane.
    pub fn normal_axis(self) -> usize {
        match self {
            Self::Yz => 0,
            Self::Xz => 1,
            Self::Xy => 2,
        }
    }
}

/// How deep base systems may nest; deeper chains are taken for a loop.
const MAX_DEPTH: usize = 64;

impl CoordinateSystem {
    /// The default: the global system, fixed in space.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            id: Id::new(),
            name: name.into(),
            kind: CoordinateSystemKind::Rectangular,
            base: None,
            definition: SystemDefinition::ThreePoints {
                origin: [0.0; 3],
                point_x: [1.0, 0.0, 0.0],
                point_xy: [0.0, 1.0, 0.0],
            },
            attached: None,
        }
    }

    /// A system given by three points in global coordinates.
    pub fn from_points(
        name: impl Into<String>,
        origin: [f64; 3],
        point_x: [f64; 3],
        point_xy: [f64; 3],
    ) -> Self {
        Self {
            definition: SystemDefinition::ThreePoints {
                origin,
                point_x,
                point_xy,
            },
            ..Self::new(name)
        }
    }

    /// Gives the system by three points in its base.
    pub fn set_points(&mut self, origin: [f64; 3], point_x: [f64; 3], point_xy: [f64; 3]) {
        self.definition = SystemDefinition::ThreePoints {
            origin,
            point_x,
            point_xy,
        };
    }

    /// The system's origin and axes in global coordinates in the undeformed model, resolved
    /// through its base systems among `systems`. The axes are an error when the definition is
    /// degenerate, a base is missing or the bases form a loop.
    pub fn frame(&self, systems: &[CoordinateSystem]) -> Frame {
        let mut frame = Frame {
            name: self.name.clone(),
            kind: self.kind,
            origin: [0.0; 3],
            axes: Ok(IDENTITY),
        };
        let mut chain = vec![self];
        let mut base = self.base;
        while let Some(id) = base {
            if chain.len() > MAX_DEPTH || chain.iter().any(|c| c.id == id) {
                frame.axes = Err(format!("{}: The base systems form a loop.", self.name));
                return frame;
            }
            let Some(next) = systems.iter().find(|c| c.id == id) else {
                frame.axes = Err(format!("{}: The base system does not exist.", self.name));
                return frame;
            };
            chain.push(next);
            base = next.base;
        }
        // From the outermost base inwards.
        let (mut origin, mut axes) = ([0.0; 3], IDENTITY);
        for system in chain.into_iter().rev() {
            match system.definition.local_axes(&system.name) {
                Ok((local_origin, local_axes)) => {
                    origin = add(origin, to_global(&axes, local_origin));
                    axes = local_axes.map(|a| to_global(&axes, a));
                }
                Err(error) => {
                    frame.axes = Err(error);
                    return frame;
                }
            }
        }
        frame.origin = origin;
        frame.axes = Ok(axes);
        frame
    }

    /// The selection the system moves with: its own, or that of the nearest base that moves.
    /// `None` for a system fixed in space.
    pub fn attachment<'a>(&'a self, systems: &'a [CoordinateSystem]) -> Option<&'a Region> {
        let mut system = self;
        for _ in 0..=MAX_DEPTH {
            if let Some(region) = &system.attached {
                return Some(region);
            }
            system = systems.iter().find(|c| Some(c.id) == system.base)?;
        }
        None
    }

    /// Whether the system moves with the model, by a selection of its own or of a base.
    pub fn moves(&self, systems: &[CoordinateSystem]) -> bool {
        self.attachment(systems).is_some()
    }

    /// Whether `other` is this system or is given in it, directly or through further bases:
    /// it cannot be this system's base.
    pub fn is_base_of(&self, other: &CoordinateSystem, systems: &[CoordinateSystem]) -> bool {
        let mut system = Some(other);
        for _ in 0..=MAX_DEPTH {
            let Some(s) = system else {
                return false;
            };
            if s.id == self.id {
                return true;
            }
            system = systems.iter().find(|c| Some(c.id) == s.base);
        }
        true
    }

    /// The system at the place where it lies in the model, given by three points in the
    /// global system with the rest unchanged: a system that keeps its place when its base is
    /// removed.
    pub fn detached_from_base(&self, systems: &[CoordinateSystem]) -> Option<CoordinateSystem> {
        self.placed_in(None, 0, systems)
    }

    /// The system where it lies in the model given anew: in `base` and by the definition at
    /// `definition` in [`SystemDefinition::LABELS`]. `None` when the system or the base does
    /// not resolve.
    pub fn placed_in(
        &self,
        base: Option<Id>,
        definition: usize,
        systems: &[CoordinateSystem],
    ) -> Option<CoordinateSystem> {
        let frame = self.frame(systems);
        let axes = frame.axes().ok()?;
        let base_frame = match base {
            Some(id) => systems.iter().find(|c| c.id == id)?.frame(systems),
            None => Frame::global_system(),
        };
        let origin = base_frame.cartesian_local(frame.origin).ok()?;
        let base_axes = base_frame.axes().ok()?;
        // The axes in components along the base's axes.
        let local = axes.map(|a| base_axes.map(|b| dot(a, b)));
        let definition = match definition {
            0 => {
                // Points as far from the origin as before, so that typed values stay round.
                let length = match self.definition {
                    SystemDefinition::ThreePoints {
                        origin, point_x, ..
                    } => norm(sub(point_x, origin)),
                    SystemDefinition::OffsetRotation { .. } => 1.0,
                };
                let length = if length > 0.0 { length } else { 1.0 };
                SystemDefinition::ThreePoints {
                    origin,
                    point_x: add(origin, scale(local[0], length)),
                    point_xy: add(origin, scale(local[1], length)),
                }
            }
            _ => SystemDefinition::OffsetRotation {
                offset: origin,
                angles: intrinsic_angles(&local),
            },
        };
        Some(CoordinateSystem {
            base,
            definition,
            ..self.clone()
        })
    }
}

/// A coordinate system resolved to global coordinates: its origin and unit axes.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub name: String,
    pub kind: CoordinateSystemKind,
    pub origin: [f64; 3],
    axes: Result<[[f64; 3]; 3], String>,
}

impl Frame {
    /// The global system.
    pub fn global_system() -> Self {
        Self {
            name: crate::GLOBAL.to_string(),
            kind: CoordinateSystemKind::Rectangular,
            origin: [0.0; 3],
            axes: Ok(IDENTITY),
        }
    }

    /// The unit axes x, y and z; fails when the definition does not give a system.
    pub fn axes(&self) -> Result<[[f64; 3]; 3], String> {
        self.axes.clone()
    }

    /// The local unit directions at a point: x, y, z of a rectangular system, or r, θ, z of a
    /// cylindrical one. On the cylinder axis, r is the system's x axis.
    pub fn directions_at(&self, point: [f64; 3]) -> Result<[[f64; 3]; 3], String> {
        let [x, y, z] = self.axes()?;
        if self.kind == CoordinateSystemKind::Rectangular {
            return Ok([x, y, z]);
        }
        let d = sub(point, self.origin);
        let radial = sub(d, scale(z, dot(d, z)));
        let r = normalize(radial)
            .filter(|_| norm(radial) > 1e-12 * norm(d).max(1e-300))
            .unwrap_or(x);
        Ok([r, cross(z, r), z])
    }

    /// A plane of the system as a point on it and its unit normal, shifted by `offset` along
    /// the normal.
    pub fn plane(
        &self,
        plane: CoordinatePlane,
        offset: f64,
    ) -> Result<([f64; 3], [f64; 3]), String> {
        let axes = self.axes()?;
        let normal = axes[plane.normal_axis()];
        Ok((add(self.origin, scale(normal, offset)), normal))
    }

    /// Coordinates of a global point in the system: x, y, z, or r, θ (radians), z.
    pub fn local(&self, point: [f64; 3]) -> Result<[f64; 3], String> {
        let [x, y, z] = self.axes()?;
        let d = sub(point, self.origin);
        let (a, b, c) = (dot(d, x), dot(d, y), dot(d, z));
        Ok(match self.kind {
            CoordinateSystemKind::Rectangular => [a, b, c],
            CoordinateSystemKind::Cylindrical => [a.hypot(b), b.atan2(a), c],
        })
    }

    /// The global point at coordinates in the system: x, y, z, or r, θ (degrees), z.
    pub fn global(&self, local: [f64; 3]) -> Result<[f64; 3], String> {
        let [x, y, z] = self.axes()?;
        let [a, b, c] = match self.kind {
            CoordinateSystemKind::Rectangular => local,
            CoordinateSystemKind::Cylindrical => {
                let (r, theta) = (local[0], local[1].to_radians());
                [r * theta.cos(), r * theta.sin(), local[2]]
            }
        };
        Ok(add(
            self.origin,
            add(add(scale(x, a), scale(y, b)), scale(z, c)),
        ))
    }

    /// The point at coordinates along the system's x, y and z axes, also for a cylindrical
    /// system: how the definitions of systems given in this one are read.
    pub fn cartesian_point(&self, local: [f64; 3]) -> Result<[f64; 3], String> {
        Ok(add(self.origin, to_global(&self.axes()?, local)))
    }

    /// Coordinates of a global point along the system's x, y and z axes.
    pub fn cartesian_local(&self, point: [f64; 3]) -> Result<[f64; 3], String> {
        let axes = self.axes()?;
        let d = sub(point, self.origin);
        Ok(axes.map(|a| dot(a, d)))
    }

    /// The frame moved with the rigid motion that fits the motion of a group of points best:
    /// from `reference` to `current` positions, in the same order. One point only moves the
    /// frame; points on a line turn it the least that turns the line.
    pub fn moved(&self, reference: &[[f64; 3]], current: &[[f64; 3]]) -> Frame {
        let Some(motion) = RigidMotion::fit(reference, current) else {
            return self.clone();
        };
        Frame {
            name: self.name.clone(),
            kind: self.kind,
            origin: motion.point(self.origin),
            axes: (self.axes.clone()).map(|axes| axes.map(|a| apply(&motion.rotation, a))),
        }
    }

    /// The frame moved with the nodes of `region`: `coords` holds the moved positions of the
    /// nodes of `mesh`, by index. Without nodes the frame stays.
    pub fn moved_with(&self, region: &Region, mesh: &FeMesh, coords: &[[f64; 3]]) -> Frame {
        let (reference, current) = region_positions(region, mesh, coords);
        self.moved(&reference, &current)
    }
}

/// Reference and moved positions of the nodes of a region; `coords` holds the moved
/// positions of the nodes of `mesh` by index.
pub fn region_positions(
    region: &Region,
    mesh: &FeMesh,
    coords: &[[f64; 3]],
) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    let nodes: Vec<NodeId> = region.nodes(mesh);
    (nodes.iter())
        .filter_map(|&n| {
            let index = mesh.node_index(n)?;
            Some((*mesh.coords().get(index)?, *coords.get(index)?))
        })
        .unzip()
}

/// A rigid motion: a turn about `centre` by `rotation`, then a shift by `shift`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidMotion {
    pub centre: [f64; 3],
    pub shift: [f64; 3],
    /// Rotation matrix by rows.
    pub rotation: [[f64; 3]; 3],
}

impl RigidMotion {
    /// The rigid motion that fits the motion of the points best (Kabsch): the turn of the
    /// centred points by the polar decomposition of `H = Σ x_i X_iᵀ`. `None` without points.
    pub fn fit(reference: &[[f64; 3]], current: &[[f64; 3]]) -> Option<Self> {
        let n = reference.len().min(current.len());
        if n == 0 {
            return None;
        }
        let mean = |points: &[[f64; 3]]| {
            let sum = points[..n].iter().fold([0.0; 3], |s, &p| add(s, p));
            scale(sum, 1.0 / n as f64)
        };
        let (c0, c1) = (mean(reference), mean(current));
        let mut h = [[0.0; 3]; 3];
        let mut spread = [[0.0; 3]; 3];
        let mut size: f64 = 0.0;
        for (p, q) in reference[..n].iter().zip(&current[..n]) {
            let (a, b) = (sub(*p, c0), sub(*q, c1));
            size = size.max(norm(a));
            for i in 0..3 {
                for j in 0..3 {
                    h[i][j] += b[i] * a[j];
                    spread[i][j] += a[i] * a[j];
                }
            }
        }
        let rotation = if size <= 0.0 {
            IDENTITY
        } else {
            best_rotation(&h, &spread, &reference[..n], &current[..n], (c0, c1))
        };
        Some(Self {
            centre: c0,
            shift: sub(c1, c0),
            rotation,
        })
    }

    /// Where the motion takes a point.
    pub fn point(&self, p: [f64; 3]) -> [f64; 3] {
        add(
            add(self.centre, apply(&self.rotation, sub(p, self.centre))),
            self.shift,
        )
    }
}

/// The rotation of the polar decomposition of `h = R C` (`C` the spread of the reference
/// points); for points on a line, the least turn of the line.
fn best_rotation(
    h: &[[f64; 3]; 3],
    spread: &[[f64; 3]; 3],
    reference: &[[f64; 3]],
    current: &[[f64; 3]],
    centres: ([f64; 3], [f64; 3]),
) -> [[f64; 3]; 3] {
    let (c0, c1) = centres;
    let (values, vectors) = eigen_symmetric(*spread);
    let mut order = [0, 1, 2];
    order.sort_by(|&a, &b| values[b].total_cmp(&values[a]));
    let largest = values[order[0]].max(1e-300);
    if values[order[1]] <= 1e-10 * largest {
        // On a line: turn the line's direction onto the moved one.
        let far = (0..reference.len().min(current.len()))
            .max_by(|&a, &b| norm(sub(reference[a], c0)).total_cmp(&norm(sub(reference[b], c0))))
            .unwrap_or(0);
        let (Some(from), Some(to)) = (
            normalize(sub(reference[far], c0)),
            normalize(sub(current[far], c1)),
        ) else {
            return IDENTITY;
        };
        return turn_onto(from, to);
    }
    let mut h = *h;
    if values[order[2]] <= 1e-10 * largest {
        // In a plane, H has rank two: `H + s R n nᵀ = R (C + s n nᵀ)` is regular, with `R n`
        // from the images of two directions in the plane, which H turns and stretches.
        let (a, b) = (vectors[order[0]], vectors[order[1]]);
        let n = cross(a, b);
        let Some(image) = normalize(cross(apply(&h, a), apply(&h, b))) else {
            return IDENTITY;
        };
        for i in 0..3 {
            for j in 0..3 {
                h[i][j] += largest * image[i] * n[j];
            }
        }
    }
    polar_rotation(&h)
}

/// The rotation of the polar decomposition `H = R U` of a regular `H` by Higham's iteration
/// `R ← (R + R⁻ᵀ) / 2`, which converges quadratically.
fn polar_rotation(h: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let size = norm(h[0]).max(norm(h[1])).max(norm(h[2]));
    if size <= 0.0 || det(h) <= 0.0 {
        return IDENTITY;
    }
    let mut r = h.map(|row| scale(row, 1.0 / size));
    for _ in 0..100 {
        let Some(inverse_t) = inverse_transpose(&r) else {
            return IDENTITY;
        };
        let next: [[f64; 3]; 3] =
            std::array::from_fn(|i| std::array::from_fn(|j| 0.5 * (r[i][j] + inverse_t[i][j])));
        let change = (0..9)
            .map(|k| (next[k / 3][k % 3] - r[k / 3][k % 3]).abs())
            .fold(0.0, f64::max);
        r = next;
        if change < 1e-15 {
            break;
        }
    }
    r
}

/// Eigenvalues of a symmetric matrix by Jacobi rotations, with the eigenvectors as rows.
fn eigen_symmetric(mut a: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut v = IDENTITY;
    for _ in 0..50 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off < 1e-15 * (a[0][0].abs() + a[1][1].abs() + a[2][2].abs()).max(1e-300) {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-300 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for row in &mut a {
                let (akp, akq) = (row[p], row[q]);
                row[p] = c * akp - s * akq;
                row[q] = s * akp + c * akq;
            }
            let (row_p, row_q) = (a[p], a[q]);
            a[p] = std::array::from_fn(|k| c * row_p[k] - s * row_q[k]);
            a[q] = std::array::from_fn(|k| s * row_p[k] + c * row_q[k]);
            for row in &mut v {
                let (vp, vq) = (row[p], row[q]);
                row[p] = c * vp - s * vq;
                row[q] = s * vp + c * vq;
            }
        }
    }
    ([a[0][0], a[1][1], a[2][2]], transpose(&v))
}

/// The least rotation that turns the unit vector `from` onto `to`.
fn turn_onto(from: [f64; 3], to: [f64; 3]) -> [[f64; 3]; 3] {
    let axis = cross(from, to);
    let (sin, cos) = (norm(axis), dot(from, to));
    let Some(axis) = normalize(axis).filter(|_| sin > 1e-14) else {
        if cos > 0.0 {
            return IDENTITY;
        }
        // Opposite: half a turn about any normal.
        let other = if from[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let normal = normalize(cross(from, other)).unwrap_or([0.0, 0.0, 1.0]);
        return rotation(normal, std::f64::consts::PI);
    };
    rotation(axis, sin.atan2(cos))
}

/// The turns about x, the turned y and the turned z (degrees) that give the unit `axes`
/// (rows), as [`SystemDefinition::OffsetRotation`] applies them: `R = Rx(a) Ry(b) Rz(c)` with
/// the axes as columns of `R`. Angles that are nearly whole are rounded to them.
fn intrinsic_angles(axes: &[[f64; 3]; 3]) -> [f64; 3] {
    let r = |i: usize, j: usize| axes[j][i];
    let b = r(0, 2).clamp(-1.0, 1.0).asin();
    let (a, c) = if b.cos() > 1e-9 {
        ((-r(1, 2)).atan2(r(2, 2)), (-r(0, 1)).atan2(r(0, 0)))
    } else {
        // Gimbal lock: the turns about x and z act about the same axis.
        (r(2, 1).atan2(r(1, 1)), 0.0)
    };
    [a, b, c].map(|angle| {
        let degrees = angle.to_degrees();
        let whole = degrees.round();
        let degrees = if (degrees - whole).abs() < 1e-9 {
            whole
        } else {
            degrees
        };
        if degrees == 0.0 { 0.0 } else { degrees }
    })
}

const IDENTITY: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// The rotation by `angle` (radians) about the unit `axis`, by rows (Rodrigues).
fn rotation(axis: [f64; 3], angle: f64) -> [[f64; 3]; 3] {
    let (s, c) = angle.sin_cos();
    let [x, y, z] = axis;
    let t = 1.0 - c;
    [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
}

fn apply(m: &[[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    m.map(|row| dot(row, v))
}

/// A vector given along `axes` (rows) in global components.
fn to_global(axes: &[[f64; 3]; 3], local: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| (0..3).map(|k| local[k] * axes[k][i]).sum())
}

fn transpose(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
}

fn det(m: &[[f64; 3]; 3]) -> f64 {
    dot(m[0], cross(m[1], m[2]))
}

fn inverse_transpose(m: &[[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let d = det(m);
    if d.abs() < 1e-300 {
        return None;
    }
    // The cofactor matrix divided by the determinant is the inverse's transpose.
    Some([
        scale(cross(m[1], m[2]), 1.0 / d),
        scale(cross(m[2], m[0]), 1.0 / d),
        scale(cross(m[0], m[1]), 1.0 / d),
    ])
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    a.map(|v| v * s)
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn normalize(a: [f64; 3]) -> Option<[f64; 3]> {
    let n = norm(a);
    (n > 0.0 && n.is_finite()).then(|| scale(a, 1.0 / n))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        (0..3).all(|k| (a[k] - b[k]).abs() < 1e-9)
    }

    fn turned(
        name: &str,
        base: Option<Id>,
        offset: [f64; 3],
        angles: [f64; 3],
    ) -> CoordinateSystem {
        CoordinateSystem {
            base,
            definition: SystemDefinition::OffsetRotation { offset, angles },
            ..CoordinateSystem::new(name)
        }
    }

    #[test]
    fn axes_follow_the_three_points() {
        let cs =
            CoordinateSystem::from_points("CS", [1.0, 0.0, 0.0], [1.0, 2.0, 0.0], [0.0, 0.0, 0.0]);
        let frame = cs.frame(&[]);
        let [x, y, z] = frame.axes().unwrap();
        assert!(close(x, [0.0, 1.0, 0.0]));
        assert!(close(y, [-1.0, 0.0, 0.0]));
        assert!(close(z, [0.0, 0.0, 1.0]));
        assert!(close(frame.origin, [1.0, 0.0, 0.0]));
    }

    #[test]
    fn degenerate_systems_are_reported() {
        let cs = CoordinateSystem::from_points("CS", [0.0; 3], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]);
        assert!(
            cs.frame(&[])
                .axes()
                .unwrap_err()
                .contains("lies on the x axis")
        );
        let cs = CoordinateSystem::from_points("CS", [0.0; 3], [0.0; 3], [2.0, 0.0, 0.0]);
        assert!(cs.frame(&[]).axes().is_err());
    }

    #[test]
    fn offset_and_rotation_turn_about_the_turned_axes() {
        // 90° about x turns y onto z; then 90° about the turned y, global z.
        let cs = turned("CS", None, [1.0, 2.0, 3.0], [90.0, 90.0, 0.0]);
        let frame = cs.frame(&[]);
        let [x, y, z] = frame.axes().unwrap();
        assert!(close(x, [0.0, 1.0, 0.0]), "{x:?}");
        assert!(close(y, [0.0, 0.0, 1.0]), "{y:?}");
        assert!(close(z, [1.0, 0.0, 0.0]), "{z:?}");
        assert!(close(frame.origin, [1.0, 2.0, 3.0]));
    }

    #[test]
    fn a_system_is_given_in_its_base() {
        let base = turned("Base", None, [10.0, 0.0, 0.0], [0.0, 0.0, 90.0]);
        let child = CoordinateSystem {
            base: Some(base.id),
            ..CoordinateSystem::from_points(
                "Child",
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
            )
        };
        let systems = vec![base.clone(), child.clone()];
        let frame = child.frame(&systems);
        // The base's x is global y: the child's origin lies at (10, 1, 0).
        assert!(close(frame.origin, [10.0, 1.0, 0.0]));
        let [x, y, _] = frame.axes().unwrap();
        assert!(close(x, [0.0, 1.0, 0.0]));
        assert!(close(y, [-1.0, 0.0, 0.0]));
        // Detached, it keeps its place.
        let detached = child.detached_from_base(&systems).unwrap();
        assert_eq!(detached.base, None);
        let again = detached.frame(&[]);
        assert!(close(again.origin, frame.origin));
        assert!(close(again.axes().unwrap()[1], y));
    }

    #[test]
    fn a_system_keeps_its_place_when_given_anew() {
        let base = turned("Base", None, [1.0, 2.0, 3.0], [30.0, -20.0, 45.0]);
        let system = turned("CS", None, [4.0, 0.5, -1.0], [10.0, 75.0, -120.0]);
        let systems = vec![base.clone(), system.clone()];
        let before = system.frame(&systems);
        for definition in [0, 1] {
            for target in [None, Some(base.id)] {
                let placed = system.placed_in(target, definition, &systems).unwrap();
                assert_eq!(placed.definition.index(), definition);
                let after = placed.frame(&systems);
                assert!(
                    close(after.origin, before.origin),
                    "{definition} {target:?}"
                );
                let (a, b) = (after.axes().unwrap(), before.axes().unwrap());
                for k in 0..3 {
                    assert!(close(a[k], b[k]), "{definition} {target:?} {a:?} {b:?}");
                }
            }
        }
        // Whole angles come back whole.
        let placed = turned("CS", None, [0.0; 3], [90.0, 0.0, 0.0])
            .placed_in(None, 1, &[])
            .unwrap();
        assert_eq!(
            placed.definition,
            SystemDefinition::OffsetRotation {
                offset: [0.0; 3],
                angles: [90.0, 0.0, 0.0]
            }
        );
    }

    #[test]
    fn loops_and_missing_bases_are_errors() {
        let mut a = CoordinateSystem::new("A");
        let mut b = CoordinateSystem::new("B");
        a.base = Some(b.id);
        b.base = Some(a.id);
        let systems = vec![a.clone(), b.clone()];
        assert!(a.frame(&systems).axes().unwrap_err().contains("loop"));
        assert!(a.is_base_of(&b, &systems));
        assert!(
            a.frame(&[a.clone()])
                .axes()
                .unwrap_err()
                .contains("does not exist")
        );
    }

    #[test]
    fn a_system_moves_with_its_own_selection_or_its_base() {
        let mut base = CoordinateSystem::new("Base");
        base.attached = Some(Region::Nodes(vec![1]));
        let child = CoordinateSystem {
            base: Some(base.id),
            ..CoordinateSystem::new("Child")
        };
        let free = CoordinateSystem::new("Free");
        let systems = vec![base.clone(), child.clone(), free.clone()];
        assert!(base.moves(&systems));
        assert!(child.moves(&systems));
        assert!(!free.moves(&systems));
        assert_eq!(child.attachment(&systems), base.attached.as_ref());
    }

    #[test]
    fn cylindrical_directions_turn_with_the_point() {
        let mut cs = CoordinateSystem::new("CS");
        cs.kind = CoordinateSystemKind::Cylindrical;
        let frame = cs.frame(&[]);
        let [r, t, z] = frame.directions_at([0.0, 2.0, 5.0]).unwrap();
        assert!(close(r, [0.0, 1.0, 0.0]));
        assert!(close(t, [-1.0, 0.0, 0.0]));
        assert!(close(z, [0.0, 0.0, 1.0]));
        let local = frame.local([0.0, 2.0, 5.0]).unwrap();
        assert!(close(local, [2.0, std::f64::consts::FRAC_PI_2, 5.0]));
        assert!(close(
            frame.global([2.0, 90.0, 5.0]).unwrap(),
            [0.0, 2.0, 5.0]
        ));
    }

    #[test]
    fn a_frame_follows_a_rigid_motion_of_its_nodes() {
        let reference = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        // 90° about z, then shifted by (5, 0, 0).
        let turn = rotation([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2);
        let current: Vec<[f64; 3]> = reference
            .iter()
            .map(|&p| add(apply(&turn, p), [5.0, 0.0, 0.0]))
            .collect();
        let frame =
            CoordinateSystem::from_points("CS", [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 1.0, 0.0])
                .frame(&[]);
        let moved = frame.moved(&reference, &current);
        assert!(close(moved.origin, [5.0, 1.0, 0.0]), "{:?}", moved.origin);
        let [x, y, z] = moved.axes().unwrap();
        assert!(close(x, [0.0, 1.0, 0.0]));
        assert!(close(y, [-1.0, 0.0, 0.0]));
        assert!(close(z, [0.0, 0.0, 1.0]));
    }

    #[test]
    fn planar_and_linear_node_groups_turn_the_frame() {
        let turn = rotation(normalize([1.0, 2.0, 3.0]).unwrap(), 0.7);
        let planar = [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [0.0, 3.0, 0.0],
            [1.0, 1.0, 0.0],
        ];
        let current: Vec<[f64; 3]> = planar.iter().map(|&p| apply(&turn, p)).collect();
        let motion = RigidMotion::fit(&planar, &current).unwrap();
        for (row, expected) in motion.rotation.iter().zip(&turn) {
            assert!(close(*row, *expected), "{:?}", motion.rotation);
        }
        // On a line the least turn of the line.
        let line = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
        let current: Vec<[f64; 3]> = line.iter().map(|&p| [p[1], p[0], 0.0]).collect();
        let motion = RigidMotion::fit(&line, &current).unwrap();
        assert!(close(
            apply(&motion.rotation, [1.0, 0.0, 0.0]),
            [0.0, 1.0, 0.0]
        ));
        assert!(close(
            apply(&motion.rotation, [0.0, 0.0, 1.0]),
            [0.0, 0.0, 1.0]
        ));
        // A single point shifts only.
        let motion = RigidMotion::fit(&[[1.0, 1.0, 1.0]], &[[2.0, 1.0, 1.0]]).unwrap();
        assert_eq!(motion.rotation, IDENTITY);
        assert!(close(motion.point([0.0; 3]), [1.0, 0.0, 0.0]));
    }

    #[test]
    fn a_general_motion_of_a_solid_group_is_found() {
        let turn = rotation(normalize([-1.0, 0.5, 2.0]).unwrap(), 2.5);
        let points = [
            [0.0, 0.0, 0.0],
            [1.0, 0.2, 0.0],
            [0.3, 1.0, 0.1],
            [0.1, 0.4, 1.3],
            [0.7, 0.7, 0.7],
        ];
        let current: Vec<[f64; 3]> = points
            .iter()
            .map(|&p| add(apply(&turn, p), [1.0, -2.0, 0.5]))
            .collect();
        let motion = RigidMotion::fit(&points, &current).unwrap();
        for (p, q) in points.iter().zip(&current) {
            assert!(close(motion.point(*p), *q));
        }
    }
}
