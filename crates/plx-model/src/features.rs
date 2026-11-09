//! The features: reference points and coordinate systems, and planes. They describe
//! places and directions in the model that other items refer to by their [`Id`], e.g. a plane spanned
//! by two axes of a coordinate system, results on that plane, or a result path between two
//! reference points.
//!
//! A coordinate system is defined like CalculiX's `*ORIENTATION`: by its
//! origin, a point on its x axis and a point in its xy plane. Cylindrical systems use the same
//! points; their z axis is the cylinder axis and r, θ, z are the local directions.

use plx_mesh::FeMesh;
use serde::{Deserialize, Serialize};

use crate::{FeModel, Id, Region, UnitSystem};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReferencePoint {
    pub id: Id,
    pub name: String,
    /// Position in global coordinates: as entered, or computed from the definition and kept
    /// up to date by [`FeModel::update_reference_points`].
    pub position: [f64; 3],
    #[serde(default)]
    pub definition: PointDefinition,
}

/// How a reference point is given.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum PointDefinition {
    /// Global coordinates, the position itself.
    #[default]
    Global,
    /// Coordinates in a coordinate system of the features: x, y, z, or for a cylindrical
    /// one r, θ in degrees, z.
    Local {
        system: Option<Id>,
        coordinates: [f64; 3],
    },
    /// The area-weighted centre of selected faces (of edges in 2D models), e.g. the centre
    /// of a hole. It follows the mesh, also after remeshing a selection of the geometry.
    CenterOfGravity(Region),
    /// The centre of the bounding box of a selection.
    BoundingBoxCenter(Region),
}

impl PointDefinition {
    /// The selection of a point at the centre of faces.
    pub fn region(&self) -> Option<&Region> {
        match self {
            PointDefinition::CenterOfGravity(region)
            | PointDefinition::BoundingBoxCenter(region) => Some(region),
            PointDefinition::Global | PointDefinition::Local { .. } => None,
        }
    }
}

impl ReferencePoint {
    pub fn new(name: impl Into<String>, position: [f64; 3]) -> Self {
        Self {
            id: Id::new(),
            name: name.into(),
            position,
            definition: PointDefinition::Global,
        }
    }

    /// The global position after the definition, on `mesh`.
    pub fn resolve(&self, model: &FeModel, mesh: &FeMesh) -> Result<[f64; 3], String> {
        let empty = || {
            format!(
                "{}: The selection contains no faces of the mesh.",
                self.name
            )
        };
        match &self.definition {
            PointDefinition::Global => Ok(self.position),
            PointDefinition::Local {
                system,
                coordinates,
            } => {
                let system = model.coordinate_system_or_global(*system).ok_or_else(|| {
                    format!("{}: The coordinate system does not exist.", self.name)
                })?;
                system.global(*coordinates)
            }
            PointDefinition::CenterOfGravity(region) => {
                face_centroid(mesh, &region.faces(mesh)).ok_or_else(empty)
            }
            PointDefinition::BoundingBoxCenter(region) => {
                let points: Vec<[f64; 3]> = (region.nodes(mesh).into_iter())
                    .filter_map(|id| mesh.node(id))
                    .collect();
                if points.is_empty() {
                    return Err(empty());
                }
                let mut min = [f64::INFINITY; 3];
                let mut max = [f64::NEG_INFINITY; 3];
                for p in &points {
                    for k in 0..3 {
                        (min[k], max[k]) = (min[k].min(p[k]), max[k].max(p[k]));
                    }
                }
                Ok([0, 1, 2].map(|k| (min[k] + max[k]) * 0.5))
            }
        }
    }
}

/// Area-weighted centre of element faces, from their corners; of edges, length-weighted.
pub(crate) fn face_centroid(
    mesh: &FeMesh,
    faces: &[(plx_mesh::ElementId, u8)],
) -> Option<[f64; 3]> {
    let (mut weight, mut sum) = (0.0, [0.0; 3]);
    for &(element, face) in faces {
        let Some(element) = mesh.element(element) else {
            continue;
        };
        let Some(topology) = element.faces().get(usize::from(face).saturating_sub(1)) else {
            continue;
        };
        let corners: Vec<[f64; 3]> = (topology.corners.iter())
            .filter_map(|&local| mesh.node(*element.nodes.get(local)?))
            .collect();
        let mut accumulate = |w: f64, centre: [f64; 3]| {
            weight += w;
            sum = add(sum, scale(centre, w));
        };
        match corners[..] {
            [a, b] => accumulate(norm(sub(b, a)), scale(add(a, b), 0.5)),
            [a, ..] if corners.len() >= 3 => {
                for pair in corners[1..].windows(2) {
                    let (b, c) = (pair[0], pair[1]);
                    let area = norm(cross(sub(b, a), sub(c, a))) * 0.5;
                    accumulate(area, scale(add(add(a, b), c), 1.0 / 3.0));
                }
            }
            _ => {}
        }
    }
    (weight > 0.0).then(|| scale(sum, 1.0 / weight))
}

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

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CoordinateSystem {
    pub id: Id,
    pub name: String,
    pub kind: CoordinateSystemKind,
    pub origin: [f64; 3],
    /// A point on the positive x axis.
    pub point_x: [f64; 3],
    /// A point in the xy plane on the side of the positive y axis.
    pub point_xy: [f64; 3],
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

impl CoordinateSystem {
    /// The default: the global system.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            id: Id::new(),
            name: name.into(),
            kind: CoordinateSystemKind::Rectangular,
            origin: [0.0; 3],
            point_x: [1.0, 0.0, 0.0],
            point_xy: [0.0, 1.0, 0.0],
        }
    }

    /// The unit axes x, y and z; fails when the points do not span a plane.
    pub fn axes(&self) -> Result<[[f64; 3]; 3], String> {
        let x = sub(self.point_x, self.origin);
        let xy = sub(self.point_xy, self.origin);
        let scale = norm(x).max(norm(xy));
        let Some(x) = normalize(x) else {
            return Err(format!(
                "{}: The point on the x axis lies at the origin.",
                self.name
            ));
        };
        let z = cross(x, xy);
        if norm(z) <= 1e-9 * scale * scale {
            return Err(format!(
                "{}: The point in the xy plane lies on the x axis.",
                self.name
            ));
        }
        let z = normalize(z).unwrap_or([0.0, 0.0, 1.0]);
        Ok([x, cross(z, x), z])
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
}

/// A point given by coordinates or by a reference point.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum PointRef {
    Coordinates([f64; 3]),
    ReferencePoint(Id),
}

impl PointRef {
    pub fn resolve(&self, model: &FeModel) -> Result<[f64; 3], String> {
        match self {
            PointRef::Coordinates(p) => Ok(*p),
            PointRef::ReferencePoint(id) => model
                .reference_point(*id)
                .map(|r| r.position)
                .ok_or_else(|| "The reference point does not exist.".to_string()),
        }
    }
}

/// How a plane is defined. Points may be reference points, so the plane follows them.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum PlaneSource {
    /// Through a point with a normal, which need not be a unit vector.
    PointNormal { point: PointRef, normal: [f64; 3] },
    /// Through three points; the normal follows the right-hand rule.
    ThreePoints { points: [PointRef; 3] },
    /// A plane of a coordinate system, or of the global one (`None`), shifted along its
    /// normal.
    CoordinateSystem {
        system: Option<Id>,
        plane: CoordinatePlane,
        offset: f64,
    },
}

/// Name of the global coordinate system, which items refer to as `None` without it being a
/// feature.
pub const GLOBAL: &str = "Global";

/// A plane of the model, e.g. for results on a cut through it. Nothing of it goes into the
/// input file.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Plane {
    pub id: Id,
    pub name: String,
    pub source: PlaneSource,
}

impl Plane {
    /// The global XY plane.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            id: Id::new(),
            name: name.into(),
            source: PlaneSource::CoordinateSystem {
                system: None,
                plane: CoordinatePlane::Xy,
                offset: 0.0,
            },
        }
    }

    /// A point on the plane and its unit normal.
    pub fn resolve(&self, model: &FeModel) -> Result<([f64; 3], [f64; 3]), String> {
        match &self.source {
            PlaneSource::PointNormal { point, normal } => {
                let point = point.resolve(model)?;
                let normal = normalize(*normal)
                    .ok_or_else(|| format!("{}: The normal is zero.", self.name))?;
                Ok((point, normal))
            }
            PlaneSource::ThreePoints { points } => {
                let [a, b, c] = [&points[0], &points[1], &points[2]];
                let [a, b, c] = [a.resolve(model)?, b.resolve(model)?, c.resolve(model)?];
                let (u, v) = (sub(b, a), sub(c, a));
                let n = cross(u, v);
                let scale = norm(u).max(norm(v));
                if norm(n) <= 1e-9 * scale * scale {
                    return Err(format!(
                        "{}: The three points lie on a straight line.",
                        self.name
                    ));
                }
                Ok((a, normalize(n).unwrap_or([0.0, 0.0, 1.0])))
            }
            PlaneSource::CoordinateSystem {
                system,
                plane,
                offset,
            } => {
                let system = model.coordinate_system_or_global(*system).ok_or_else(|| {
                    format!("{}: The coordinate system does not exist.", self.name)
                })?;
                system.plane(*plane, *offset)
            }
        }
    }
}

/// Results on a plane: the cut through the model, shown alone with the range of the values
/// on it. Nothing of it goes into the input file.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResultPlane {
    pub name: String,
    /// The plane feature.
    pub plane: Id,
}

/// A straight path through the model on which results are read, e.g. across a wall: values
/// at `points` evenly spaced points from start to end, interpolated inside the elements.
/// Nothing of it goes into the input file.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResultPath {
    pub name: String,
    pub start: PointRef,
    pub end: PointRef,
    pub points: u32,
}

impl ResultPath {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            start: PointRef::Coordinates([0.0; 3]),
            end: PointRef::Coordinates([1.0, 0.0, 0.0]),
            points: 100,
        }
    }

    /// Start and end in model coordinates.
    pub fn ends(&self, model: &FeModel) -> Result<([f64; 3], [f64; 3]), String> {
        Ok((self.start.resolve(model)?, self.end.resolve(model)?))
    }

    /// Positions of the read-out points with their distance from the start.
    pub fn samples(&self, model: &FeModel) -> Result<Vec<(f64, [f64; 3])>, String> {
        let (start, end) = self.ends(model)?;
        let length = norm(sub(end, start));
        if length == 0.0 {
            return Err(format!("{}: Start and end coincide.", self.name));
        }
        let n = self.points.max(2);
        Ok((0..n)
            .map(|i| {
                let t = i as f64 / (n - 1) as f64;
                (t * length, add(start, scale(sub(end, start), t)))
            })
            .collect())
    }
}

impl FeModel {
    pub fn reference_point(&self, id: Id) -> Option<&ReferencePoint> {
        self.reference_points.iter().find(|r| r.id == id)
    }

    pub fn reference_point_named(&self, name: &str) -> Option<&ReferencePoint> {
        self.reference_points.iter().find(|r| r.name == name)
    }

    pub fn coordinate_system(&self, id: Id) -> Option<&CoordinateSystem> {
        self.coordinate_systems.iter().find(|c| c.id == id)
    }

    pub fn coordinate_system_named(&self, name: &str) -> Option<&CoordinateSystem> {
        self.coordinate_systems.iter().find(|c| c.name == name)
    }

    /// A coordinate system of the features, or the global one for `None`.
    pub fn coordinate_system_or_global(&self, system: Option<Id>) -> Option<CoordinateSystem> {
        match system {
            Some(id) => self.coordinate_system(id).cloned(),
            None => Some(CoordinateSystem::new(GLOBAL)),
        }
    }

    /// The name of a coordinate system for the GUI and messages: [`GLOBAL`] for `None`.
    pub fn system_name(&self, system: Option<Id>) -> String {
        match system {
            None => GLOBAL.to_string(),
            Some(id) => self.coordinate_system(id).map_or_else(
                || "(missing coordinate system)".to_string(),
                |c| c.name.clone(),
            ),
        }
    }

    /// The coordinate system of a name for items read from an input file or typed in:
    /// `None` for [`GLOBAL`] and for a name the features lack.
    pub fn system_by_name(&self, name: &str) -> Option<Id> {
        self.coordinate_system_named(name).map(|c| c.id)
    }

    pub fn plane(&self, id: Id) -> Option<&Plane> {
        self.planes.iter().find(|p| p.id == id)
    }

    pub fn plane_named(&self, name: &str) -> Option<&Plane> {
        self.planes.iter().find(|p| p.name == name)
    }

    /// The features the FE model shares with its results: reference points, coordinate
    /// systems and planes. Result paths and result planes belong to the results alone.
    pub fn shared_features(&self) -> FeModel {
        FeModel {
            reference_points: self.reference_points.clone(),
            coordinate_systems: self.coordinate_systems.clone(),
            planes: self.planes.clone(),
            ..FeModel::default()
        }
    }

    /// Whether the shared features, see [`FeModel::shared_features`], are the same.
    pub fn same_shared_features(&self, other: &FeModel) -> bool {
        self.reference_points == other.reference_points
            && self.coordinate_systems == other.coordinate_systems
            && self.planes == other.planes
    }

    /// Replaces the shared features with those of `source`, see
    /// [`FeModel::shared_features`].
    pub fn set_shared_features(&mut self, source: &FeModel) {
        self.reference_points = source.reference_points.clone();
        self.coordinate_systems = source.coordinate_systems.clone();
        self.planes = source.planes.clone();
    }

    /// Takes over an edit of the shared features made on results: `before` and `after` are
    /// the features of the results before and after the edit, in `units`, this the FE model
    /// in [`crate::STORED`] units. Items the edit did not change keep their values here, so
    /// that converting them back and forth never rounds them; a renamed item keeps the
    /// references to it, which go by its id.
    pub fn take_feature_edit(&mut self, before: &FeModel, after: &FeModel, units: UnitSystem) {
        fn merge<T: Clone + PartialEq>(
            own: &[T],
            before: &[T],
            after: &[T],
            converted: &[T],
            id: impl Fn(&T) -> Id,
        ) -> Vec<T> {
            (after.iter().zip(converted))
                .map(|(item, converted)| {
                    let kept = before
                        .contains(item)
                        .then(|| own.iter().find(|o| id(o) == id(item)));
                    kept.flatten().unwrap_or(converted).clone()
                })
                .collect()
        }
        let before = before.shared_features();
        let mut converted = after.shared_features();
        converted.convert_features(&crate::convert::Conversion::new(units, crate::STORED));
        self.reference_points = merge(
            &self.reference_points,
            &before.reference_points,
            &after.reference_points,
            &converted.reference_points,
            |r| r.id,
        );
        self.coordinate_systems = merge(
            &self.coordinate_systems,
            &before.coordinate_systems,
            &after.coordinate_systems,
            &converted.coordinate_systems,
            |c| c.id,
        );
        self.planes = merge(
            &self.planes,
            &before.planes,
            &after.planes,
            &converted.planes,
            |p| p.id,
        );
    }

    /// Recomputes the positions of reference points defined by a coordinate system or a
    /// selection, e.g. after remeshing; returns why points could not be placed.
    pub fn update_reference_points(&mut self, mesh: &FeMesh) -> Vec<String> {
        let mut errors = Vec::new();
        let positions: Vec<Result<[f64; 3], String>> = (self.reference_points.iter())
            .map(|point| point.resolve(self, mesh))
            .collect();
        for (point, position) in self.reference_points.iter_mut().zip(positions) {
            match position {
                Ok(position) => point.position = position,
                Err(error) => errors.push(error),
            }
        }
        errors
    }
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
        (0..3).all(|k| (a[k] - b[k]).abs() < 1e-12)
    }

    #[test]
    fn axes_follow_the_three_points() {
        let mut cs = CoordinateSystem::new("CS");
        cs.origin = [1.0, 1.0, 0.0];
        cs.point_x = [1.0, 3.0, 0.0];
        cs.point_xy = [-2.0, 4.0, 0.0];
        let [x, y, z] = cs.axes().unwrap();
        assert!(close(x, [0.0, 1.0, 0.0]));
        // The xy point need not be on the y axis, only on its side.
        assert!(close(y, [-1.0, 0.0, 0.0]));
        assert!(close(z, [0.0, 0.0, 1.0]));
        let (point, normal) = cs.plane(CoordinatePlane::Yz, 2.0).unwrap();
        assert!(close(point, [1.0, 3.0, 0.0]) && close(normal, x));
    }

    #[test]
    fn degenerate_systems_are_reported() {
        let mut cs = CoordinateSystem::new("CS");
        cs.point_xy = [2.0, 0.0, 0.0];
        assert!(cs.axes().is_err());
        cs.point_x = cs.origin;
        assert!(cs.axes().is_err());
    }

    #[test]
    fn cylindrical_directions_turn_with_the_point() {
        let mut cs = CoordinateSystem::new("CS");
        cs.kind = CoordinateSystemKind::Cylindrical;
        let [r, t, z] = cs.directions_at([0.0, 2.0, 7.0]).unwrap();
        assert!(close(r, [0.0, 1.0, 0.0]) && close(t, [-1.0, 0.0, 0.0]));
        assert!(close(z, [0.0, 0.0, 1.0]));
        let local = cs.local([0.0, 2.0, 7.0]).unwrap();
        assert!(close(local, [2.0, std::f64::consts::FRAC_PI_2, 7.0]));
        // On the axis r falls back to x.
        assert!(close(
            cs.directions_at([0.0, 0.0, 3.0]).unwrap()[0],
            [1.0, 0.0, 0.0]
        ));
    }

    #[test]
    fn paths_resolve_reference_points_and_follow_renames() {
        let mut model = FeModel::default();
        model
            .reference_points
            .push(ReferencePoint::new("RP-1", [0.0, 0.0, 4.0]));
        let rp = model.reference_points[0].id;
        let mut path = ResultPath::new("Path-1");
        path.start = PointRef::ReferencePoint(rp);
        path.end = PointRef::Coordinates([0.0, 0.0, 0.0]);
        path.points = 5;
        let samples = path.samples(&model).unwrap();
        assert_eq!(samples.len(), 5);
        assert_eq!(samples[1], (1.0, [0.0, 0.0, 3.0]));
        model.result_paths.push(path);
        // A renamed point is still the start of the path.
        model.reference_points[0].name = "Top".into();
        assert_eq!(model.result_paths[0].start, PointRef::ReferencePoint(rp));
        assert_eq!(
            model.result_paths[0].samples(&model).unwrap()[1].1,
            [0.0, 0.0, 3.0]
        );
        // A removed one is not.
        model.reference_points.clear();
        assert!(model.result_paths[0].samples(&model).is_err());
    }

    #[test]
    fn results_hand_their_feature_edits_to_the_fe_model() {
        use crate::UnitSystem;
        // The FE model in metres, its results in millimetres.
        let mut fe = FeModel::default();
        let mut hub = CoordinateSystem::new("Hub");
        hub.origin = [0.1, 0.0, 0.0];
        hub.point_x = [0.3, 0.0, 0.0];
        let hub_id = hub.id;
        fe.coordinate_systems.push(hub);
        let mut step = crate::Step::new_static("Rotor");
        step.kind = crate::StepKind::ComplexFrequency(crate::ComplexFrequencyStep {
            system: Some(hub_id),
            ..Default::default()
        });
        fe.steps.push(step);
        let mut results = fe.shared_features();
        let to_mm = crate::convert::Conversion::new(crate::STORED, UnitSystem::MmTonSC);
        results.convert_features(&to_mm);
        // A coordinate system made on the results arrives in metres.
        let before = results.clone();
        let mut local = CoordinateSystem::new("Local");
        local.origin = [5.0, 0.0, 0.0];
        local.point_x = [5.0, 10.0, 0.0];
        results.coordinate_systems.push(local);
        let kept = fe.coordinate_systems[0].clone();
        fe.take_feature_edit(&before, &results, UnitSystem::MmTonSC);
        assert_eq!(fe.coordinate_systems.len(), 2);
        // The unchanged one keeps its values exactly.
        assert_eq!(fe.coordinate_systems[0], kept);
        assert!(close(fe.coordinate_systems[1].origin, [0.005, 0.0, 0.0]));
        // A rename on the results keeps the step on the system.
        let before = results.clone();
        results.coordinate_systems[0].name = "Shaft".into();
        fe.take_feature_edit(&before, &results, UnitSystem::MmTonSC);
        assert_eq!(fe.coordinate_systems[0].name, "Shaft");
        let crate::StepKind::ComplexFrequency(complex) = &fe.steps[0].kind else {
            unreachable!()
        };
        assert_eq!(complex.system, Some(hub_id));
        // And a deletion goes as well.
        let before = results.clone();
        results.coordinate_systems.remove(1);
        fe.take_feature_edit(&before, &results, UnitSystem::MmTonSC);
        let names: Vec<_> = fe
            .coordinate_systems
            .iter()
            .map(|c| c.name.as_str())
            .collect();
        assert_eq!(names, ["Shaft"]);
    }

    #[test]
    fn springs_on_a_renamed_point_follow_it() {
        let mut model = FeModel::default();
        model
            .reference_points
            .push(ReferencePoint::new("RP-1", [0.0; 3]));
        let rp = model.reference_points[0].id;
        let mut spring = crate::Spring::new("Spring-1");
        spring.ends = [
            crate::SpringEnd::Region(crate::Region::ReferencePoint(rp)),
            crate::SpringEnd::World,
        ];
        model.constraints.push(crate::Constraint::Spring(spring));
        model.reference_points[0].name = "Hub".into();
        let crate::Constraint::Spring(spring) = &model.constraints[0] else {
            unreachable!()
        };
        assert_eq!(
            spring.ends[0],
            crate::SpringEnd::Region(crate::Region::ReferencePoint(rp))
        );
        assert_eq!(model.name_of(rp), Some("Hub"));
    }

    #[test]
    fn planes_resolve_their_definitions() {
        let mut model = FeModel::default();
        model
            .reference_points
            .push(ReferencePoint::new("RP-1", [0.0, 0.0, 2.0]));
        let rp = model.reference_points[0].id;
        let mut plane = Plane::new("Plane-1");
        let (point, normal) = plane.resolve(&model).unwrap();
        assert!(close(point, [0.0; 3]) && close(normal, [0.0, 0.0, 1.0]));
        plane.source = PlaneSource::ThreePoints {
            points: [
                PointRef::ReferencePoint(rp),
                PointRef::Coordinates([1.0, 0.0, 2.0]),
                PointRef::Coordinates([0.0, 0.0, 3.0]),
            ],
        };
        let (point, normal) = plane.resolve(&model).unwrap();
        assert!(close(point, [0.0, 0.0, 2.0]) && close(normal, [0.0, -1.0, 0.0]));
        plane.source = PlaneSource::PointNormal {
            point: PointRef::Coordinates([1.0, 1.0, 1.0]),
            normal: [0.0, 3.0, 4.0],
        };
        assert!(close(plane.resolve(&model).unwrap().1, [0.0, 0.6, 0.8]));
        // A plane of a coordinate system follows it and its renames.
        let mut cs = CoordinateSystem::new("CS");
        cs.origin = [0.0, 0.0, 5.0];
        cs.point_x = [0.0, 1.0, 5.0];
        cs.point_xy = [-1.0, 0.0, 5.0];
        let system = cs.id;
        model.coordinate_systems.push(cs);
        plane.source = PlaneSource::CoordinateSystem {
            system: Some(system),
            plane: CoordinatePlane::Yz,
            offset: 2.0,
        };
        let (point, normal) = plane.resolve(&model).unwrap();
        assert!(close(point, [0.0, 2.0, 5.0]) && close(normal, [0.0, 1.0, 0.0]));
        model.planes.push(plane);
        model.result_planes.push(ResultPlane {
            name: "Plane_Result-1".into(),
            plane: model.planes[0].id,
        });
        // So does the rotation axis of a complex frequency step: the z axis through the origin.
        let mut step = crate::Step::new_static("Rotor");
        step.kind = crate::StepKind::ComplexFrequency(crate::ComplexFrequencyStep {
            system: Some(system),
            ..Default::default()
        });
        model.steps.push(step);
        let complex = |model: &FeModel| match &model.steps[0].kind {
            crate::StepKind::ComplexFrequency(c) => c.clone(),
            _ => unreachable!(),
        };
        let (point, axis) = complex(&model).axis_in(&model).unwrap();
        assert!(close(point, [0.0, 0.0, 5.0]) && close(axis, [0.0, 0.0, 1.0]));
        model.coordinate_systems[0].name = "Turned".into();
        model.planes[0].name = "Cut".into();
        assert!(model.planes[0].resolve(&model).is_ok());
        assert_eq!(complex(&model).system, Some(system));
        assert!(complex(&model).axis_in(&model).is_ok());
        let mut missing = complex(&model);
        missing.system = Some(Id::new());
        assert!(missing.axis_in(&model).is_err());
        assert_eq!(model.result_planes[0].plane, model.planes[0].id);
    }

    #[test]
    fn degenerate_planes_are_reported() {
        let model = FeModel::default();
        let plane = Plane {
            id: crate::Id::new(),
            name: "P".into(),
            source: PlaneSource::ThreePoints {
                points: [
                    PointRef::Coordinates([0.0; 3]),
                    PointRef::Coordinates([1.0, 0.0, 0.0]),
                    PointRef::Coordinates([2.0, 0.0, 0.0]),
                ],
            },
        };
        assert!(plane.resolve(&model).is_err());
        let missing = Plane {
            id: crate::Id::new(),
            name: "P".into(),
            source: PlaneSource::CoordinateSystem {
                system: Some(crate::Id::new()),
                plane: CoordinatePlane::Xy,
                offset: 0.0,
            },
        };
        assert!(missing.resolve(&model).is_err());
    }

    #[test]
    fn local_coordinates_turn_into_global_ones() {
        let mut cs = CoordinateSystem::new("CS");
        cs.origin = [1.0, 0.0, 0.0];
        cs.point_x = [1.0, 1.0, 0.0];
        cs.point_xy = [0.0, 0.0, 0.0];
        assert!(close(cs.global([2.0, 0.0, 3.0]).unwrap(), [1.0, 2.0, 3.0]));
        cs.kind = CoordinateSystemKind::Cylindrical;
        let p = cs.global([2.0, 90.0, 0.0]).unwrap();
        assert!((0..3).all(|k| (p[k] - [-1.0, 0.0, 0.0][k]).abs() < 1e-12));
        let back = cs.local(p).unwrap();
        assert!((back[0] - 2.0).abs() < 1e-12 && (back[1].to_degrees() - 90.0).abs() < 1e-9);
    }

    #[test]
    fn reference_points_sit_at_the_centre_of_faces() {
        use plx_mesh::{Element, ElementShape, Part};
        // One hexahedron from 0 to 2 in x, 0 to 1 in y and z.
        let mut mesh = FeMesh::default();
        let corners = [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [2.0, 0.0, 1.0],
            [2.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ];
        for (i, c) in corners.into_iter().enumerate() {
            mesh.set_node(i as u32 + 1, c);
        }
        mesh.add_element(Element {
            id: 1,
            type_name: "C3D8".into(),
            shape: ElementShape::Hex8,
            nodes: (1..=8).collect(),
        })
        .unwrap();
        mesh.parts.push(Part {
            name: "P".into(),
            elements: vec![1],
        });
        let mut model = FeModel::default();
        let faces: Vec<(plx_mesh::ElementId, u8)> = (1..=6).map(|f| (1, f)).collect();
        let mut point = ReferencePoint::new("RP-1", [9.0; 3]);
        point.definition = PointDefinition::CenterOfGravity(Region::Faces(faces.clone()));
        model.reference_points.push(point);
        let mut corner = ReferencePoint::new("RP-2", [0.0; 3]);
        corner.definition = PointDefinition::BoundingBoxCenter(Region::Faces(faces));
        model.reference_points.push(corner);
        assert!(model.update_reference_points(&mesh).is_empty());
        assert!(close(model.reference_points[0].position, [1.0, 0.5, 0.5]));
        assert!(close(model.reference_points[1].position, [1.0, 0.5, 0.5]));
        // One face alone: its own centre.
        let top = mesh.element(1).unwrap().faces().iter().position(|f| {
            f.corners
                .iter()
                .all(|&c| mesh.node(mesh.element(1).unwrap().nodes[c]).unwrap()[0] == 2.0)
        });
        let face = top.unwrap() as u8 + 1;
        model.reference_points[0].definition =
            PointDefinition::CenterOfGravity(Region::Faces(vec![(1, face)]));
        model.update_reference_points(&mesh);
        assert!(close(model.reference_points[0].position, [2.0, 0.5, 0.5]));
        // Nothing selected: the point keeps its place and the error is reported.
        model.reference_points[0].definition =
            PointDefinition::CenterOfGravity(Region::Faces(Vec::new()));
        assert_eq!(model.update_reference_points(&mesh).len(), 1);
        assert!(close(model.reference_points[0].position, [2.0, 0.5, 0.5]));
    }
}
