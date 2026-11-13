//! Symbols of boundary conditions and loads drawn over the 3D view
//! for the step chosen to show: arrows for forces at the loaded nodes, pressure arrows
//! pointing onto the faces, one arrow of the resultant force for a surface traction, and
//! cones (translations) and plates (rotations) for held degrees of freedom. Heat loads and
//! temperatures, which have no direction, are balls at the nodes; heat flowing through a
//! surface is an arrow onto it (into the part) or off it.
//!
//! Constraints are yellow: a rigid body is a fan of lines from points spread
//! over its region to its reference point, springs are coils in the directions they act in,
//! and a compression only support is cones pushing onto its surface.
//!
//! Symbols keep their size on screen (symbol size 50) and lie
//! on top of the model. They are 3D shapes in pixel units, drawn as the outlines of their
//! convex parts.

use std::collections::HashSet;

use egui::{Color32, Painter, Pos2, Rect, Shape, Stroke, Vec2, vec2};
use glam::{DVec3, Vec3};
use plx_mesh::{ElementId, NodeId};
use plx_model::{BoundaryKind, Constraint, LoadKind, Region, symmetry_plane};
use plx_render::Camera;

use crate::model::Model;

/// Unit length of the support symbols in pixels; the default symbol size 50 draws a
/// fixed support about this large.
const SUPPORT_SIZE: f32 = 20.0;
/// Arrow length in pixels, long enough that the arrows read at a glance.
const ARROW_SIZE: f32 = 40.0;
/// The default colours: loads royal blue, boundary conditions lime, selection red.
const LOAD_COLOR: Color32 = Color32::from_rgb(65, 105, 225);
const BOUNDARY_COLOR: Color32 = Color32::from_rgb(0, 255, 0);
const SELECTED_COLOR: Color32 = Color32::from_rgb(255, 0, 0);
/// The default colour of constraints.
const CONSTRAINT_COLOR: Color32 = Color32::from_rgb(255, 255, 0);
/// Width of the lines of a rigid body in pixels.
const LINE_WIDTH: f32 = 1.5;
/// Width of the wire of a spring in pixels: the tube radius 0.03 of the symbol size.
const SPRING_WIDTH: f32 = 2.0;
/// A point spring on more nodes is drawn once at their centre.
const POINT_SPRING_NODES: usize = 10;
/// Splits of the region's extent when spreading arrows.
const DIVISIONS: f64 = 6.0;
/// Sides of the polygons standing in for circles.
const CIRCLE_SEGMENTS: usize = 16;

/// What a symbol stands for.
#[derive(Clone, Debug, PartialEq)]
pub enum Kind {
    Load(LoadKind),
    Boundary(BoundaryKind),
    /// A rigid body driven by the reference point.
    RigidBody {
        reference_point: plx_model::Id,
    },
    /// A point mass: a ball at its reference point, with the lines of a rigid body from the
    /// selection that carries it.
    PointMass {
        reference_point: plx_model::Id,
    },
    /// Springs with their stiffness K1..K3, to ground or to a surface.
    Spring {
        stiffness: [f64; 3],
        to_surface: bool,
    },
    /// Gaps that only take pressure, on the faces of the region; flipped they push outward.
    CompressionOnly {
        flipped: bool,
    },
    /// A joint: lines from both ends to the joint, a ball at it and arrows for its free
    /// degrees of freedom.
    Joint(Box<plx_model::Joint>),
}

impl Kind {
    /// The kind and region of a constraint's symbol; ties have none.
    pub fn of_constraint(constraint: &Constraint) -> Option<(Kind, Region)> {
        Some(match constraint {
            Constraint::RigidBody(body) => (
                Kind::RigidBody {
                    reference_point: body.reference_point,
                },
                body.region.clone(),
            ),
            Constraint::PointMass(mass) => (
                Kind::PointMass {
                    reference_point: mass.reference_point,
                },
                mass.region.clone(),
            ),
            // At the selection, else at the point; an axis shows translational and
            // rotational springs alike.
            Constraint::Spring(spring) => {
                let rotational = spring.acting_rotational();
                (
                    Kind::Spring {
                        stiffness: std::array::from_fn(|i| spring.stiffness[i].max(rotational[i])),
                        to_surface: spring.master_slave().is_some(),
                    },
                    (spring.selection())
                        .or_else(|| spring.ends.iter().find_map(plx_model::SpringEnd::point))?
                        .clone(),
                )
            }
            Constraint::CompressionOnly(support) => (
                Kind::CompressionOnly {
                    flipped: support.flipped,
                },
                support.region.clone(),
            ),
            Constraint::Joint(joint) => (
                Kind::Joint(Box::new(joint.clone())),
                joint.locating_end()?.clone(),
            ),
            Constraint::Tie(_) | Constraint::NodeTie(_) => return None,
        })
    }
}

/// A boundary condition or load to draw.
#[derive(Clone, Debug, PartialEq)]
pub struct Item {
    pub kind: Kind,
    pub region: Region,
    /// The coordinate system the components of a force, moment or excitation are given in;
    /// `None` is the global one.
    pub system: Option<plx_model::Id>,
    /// The distribution a pressure or temperature is scaled by: its arrows or balls are
    /// drawn as large as the factor where they are.
    pub distribution: Option<plx_model::Id>,
    /// Selected in the tree or edited in a dialog: drawn in the highlight colour.
    pub selected: bool,
}

/// One symbol: its shape at a point of the model.
#[derive(Clone, Debug, PartialEq)]
pub struct Symbol {
    /// Point in render coordinates.
    pub position: Vec3,
    /// Unit direction the shape points along.
    pub direction: Vec3,
    pub shape: SymbolShape,
    pub color: Color32,
    /// Size relative to the usual one: 1, or the factor of a distribution over the largest
    /// one of the item.
    pub scale: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SymbolShape {
    /// Arrow starting at the point.
    Arrow,
    /// Arrow ending at the point, such as pressure pushing onto a face.
    ArrowOnto,
    /// Arrow with two heads, a moment about its axis.
    MomentArrow,
    /// Cone with its tip at the point: a held translation.
    Cone,
    /// Rod with a plate: a held rotation.
    RotationLock,
    /// Ball around the point: a temperature or heat at a node.
    Ball,
    /// A larger ball on the point: a point mass, seen around the reference point marker.
    Mass,
    /// Line from the point to another point in render coordinates.
    LineTo(Vec3),
    /// Coil spring ending at the point.
    Spring,
    /// Cone with its tip at the point and a ground plate behind its base: a support that
    /// pushes along the direction from the side of the plate.
    GapSupport,
}

/// Symbols of the items, on the visible parts of the model.
pub fn build(model: &Model, items: &[Item]) -> Vec<Symbol> {
    let visible = Visible::new(model);
    let mut symbols = Vec::new();
    for item in items {
        let color = match (&item.kind, item.selected) {
            (_, true) => SELECTED_COLOR,
            (Kind::Load(_), false) => LOAD_COLOR,
            (Kind::Boundary(_), false) => BOUNDARY_COLOR,
            (_, false) => CONSTRAINT_COLOR,
        };
        // Components of a force or moment in the load's coordinate system, in global
        // directions at a point.
        let system = (item.system).and_then(|id| model.fe.coordinate_system_or_global(Some(id)));
        let cylindrical = system
            .as_ref()
            .is_some_and(|s| s.kind == plx_model::CoordinateSystemKind::Cylindrical);
        let global = |point: DVec3, local: [f64; 3]| {
            plx_model::Load::to_global(system.as_ref(), point.into(), local)
                .map_or(DVec3::ZERO, DVec3::from)
        };
        let mut add = |position: DVec3, direction: DVec3, shape: SymbolShape| {
            if let Some(direction) = direction.try_normalize() {
                symbols.push(Symbol {
                    position: (position - model.origin()).as_vec3(),
                    direction: direction.as_vec3(),
                    shape,
                    color,
                    scale: 1.0,
                });
            }
        };
        // The factors of the item's distribution, relative to the largest one over the
        // region, and the arrows or balls it scales.
        let sampler = (item.distribution)
            .and_then(|id| model.fe.distribution(id))
            .and_then(|d| {
                (d.sampler(model.fe.properties.units, &model.fe.coordinate_systems)).ok()
            });
        let scaled = |points: &[DVec3]| -> Option<(Vec<f64>, f64, Vec<usize>)> {
            let sampler = sampler.as_ref()?;
            let factors: Vec<f64> = points.iter().map(|&p| sampler.factor(p.into())).collect();
            let largest = (factors.iter().map(|f| f.abs()))
                .filter(|f| f.is_finite())
                .fold(0.0, f64::max);
            // Denser than usual, to show how the factor varies.
            Some((factors, largest, sample(points, DIVISIONS * 2.0)))
        };
        match item.kind {
            // Lines from nodes spread over the region to the reference point, half as dense
            // as the arrows of loads.
            Kind::RigidBody { reference_point } => {
                let Some(point) = model.fe.reference_point(reference_point) else {
                    continue;
                };
                let to = (DVec3::from(point.position) - model.origin()).as_vec3();
                let points = node_points(model, &item.region, &visible);
                for index in sample(&points, DIVISIONS / 2.0) {
                    symbols.push(Symbol {
                        position: (points[index] - model.origin()).as_vec3(),
                        direction: Vec3::ZERO,
                        shape: SymbolShape::LineTo(to),
                        color,
                        scale: 1.0,
                    });
                }
            }
            Kind::PointMass { reference_point } => {
                let Some(point) = model.fe.reference_point(reference_point) else {
                    continue;
                };
                let at = DVec3::from(point.position);
                let to = (at - model.origin()).as_vec3();
                let points = node_points(model, &item.region, &visible);
                for index in sample(&points, DIVISIONS / 2.0) {
                    symbols.push(Symbol {
                        position: (points[index] - model.origin()).as_vec3(),
                        direction: Vec3::ZERO,
                        shape: SymbolShape::LineTo(to),
                        color,
                        scale: 1.0,
                    });
                }
                symbols.push(Symbol {
                    position: to,
                    direction: Vec3::Z,
                    shape: SymbolShape::Mass,
                    color,
                    scale: 1.0,
                });
            }
            // A spring for each direction with stiffness. To a surface at points spread over
            // the region; to ground at the centre of faces, or at each node and at the
            // centre of many nodes.
            Kind::Spring {
                stiffness,
                to_surface,
            } => {
                let faces = face_geometry(model, &item.region, &visible);
                let centers: Vec<DVec3> = faces.iter().map(|f| f.center).collect();
                let mut points = if centers.is_empty() {
                    node_points(model, &item.region, &visible)
                } else if to_surface {
                    centers
                } else {
                    region_center(model, &item.region, &visible)
                        .into_iter()
                        .collect()
                };
                if to_surface {
                    let picked = sample(&points, DIVISIONS / 2.0);
                    points = picked.into_iter().map(|i| points[i]).collect();
                } else if points.len() >= POINT_SPRING_NODES {
                    points = vec![points.iter().sum::<DVec3>() / points.len() as f64];
                }
                for point in points {
                    for axis in acting(stiffness) {
                        add(point, axis, SymbolShape::Spring);
                    }
                }
            }
            // Lines from nodes spread over each selection end, or from a point end, to the
            // joint, a ball at it, and arrows along the frame axes for free translations
            // and moment arrows about them for free rotations.
            Kind::Joint(ref joint) => {
                let Ok((origin, axes)) = model.fe.joint_frame(&model.mesh, joint) else {
                    continue;
                };
                let origin = DVec3::from(origin);
                add(origin, DVec3::Z, SymbolShape::Ball);
                let free = joint.free_dofs();
                for &d in model.fe.properties.space.remote_dofs() {
                    if free[d] {
                        let shape = if d < 3 {
                            SymbolShape::Arrow
                        } else {
                            SymbolShape::MomentArrow
                        };
                        add(origin, DVec3::from(axes[d % 3]), shape);
                    }
                }
                let to = (origin - model.origin()).as_vec3();
                for end in &joint.ends {
                    let Some(region) = end.region() else {
                        continue;
                    };
                    let points = match model.fe.point_position(&model.mesh, region) {
                        Some(point) => vec![DVec3::from(point)],
                        None => node_points(model, region, &visible),
                    };
                    for index in sample(&points, DIVISIONS / 2.0) {
                        if points[index].distance(origin) > 0.0 {
                            symbols.push(Symbol {
                                position: (points[index] - model.origin()).as_vec3(),
                                direction: Vec3::ZERO,
                                shape: SymbolShape::LineTo(to),
                                color,
                                scale: 1.0,
                            });
                        }
                    }
                }
            }
            // Cones with their tips on the faces and a ground plate behind them, on the side
            // the support pushes from: outside, or inside the material when flipped.
            Kind::CompressionOnly { flipped } => {
                let faces = face_geometry(model, &item.region, &visible);
                let centers: Vec<DVec3> = faces.iter().map(|f| f.center).collect();
                let sign = if flipped { 1.0 } else { -1.0 };
                for index in sample(&centers, DIVISIONS) {
                    let face = &faces[index];
                    add(face.center, face.normal * sign, SymbolShape::GapSupport);
                }
            }
            Kind::Load(LoadKind::Moment(moment)) => {
                let points = node_points(model, &item.region, &visible);
                for index in sample(&points, DIVISIONS) {
                    let moment = global(points[index], moment);
                    add(points[index], moment, SymbolShape::MomentArrow);
                }
            }
            Kind::Load(LoadKind::Pressure(pressure)) => {
                let faces = face_geometry(model, &item.region, &visible);
                let centers: Vec<DVec3> = faces.iter().map(|f| f.center).collect();
                if let Some((factors, largest, picked)) = scaled(&centers) {
                    for index in picked {
                        let face = &faces[index];
                        let value = pressure * factors[index];
                        let (direction, shape) = if value >= 0.0 {
                            (-face.normal, SymbolShape::ArrowOnto)
                        } else {
                            (face.normal, SymbolShape::Arrow)
                        };
                        symbols.extend(scaled_symbol(
                            model,
                            face.center,
                            direction,
                            shape,
                            color,
                            factors[index].abs() / largest,
                        ));
                    }
                    continue;
                }
                for index in sample(&centers, DIVISIONS) {
                    let face = &faces[index];
                    if pressure >= 0.0 {
                        add(face.center, -face.normal, SymbolShape::ArrowOnto);
                    } else {
                        add(face.center, face.normal, SymbolShape::Arrow);
                    }
                }
            }
            // Applied to all nodes, an arrow at the nodes; else one for the total force.
            Kind::Load(
                LoadKind::Force {
                    value: force,
                    each_node: true,
                }
                | LoadKind::Excitation(plx_model::Excitation {
                    value: force,
                    each_node: true,
                    ..
                }),
            ) => {
                let points = node_points(model, &item.region, &visible);
                for index in sample(&points, DIVISIONS) {
                    add(
                        points[index],
                        global(points[index], force),
                        SymbolShape::Arrow,
                    );
                }
            }
            Kind::Load(
                LoadKind::Force { value: force, .. }
                | LoadKind::Excitation(plx_model::Excitation { value: force, .. }),
            ) => {
                // In a cylindrical system the direction changes over the region: arrows
                // spread over its nodes.
                if cylindrical {
                    let points = node_points(model, &item.region, &visible);
                    for index in sample(&points, DIVISIONS) {
                        add(
                            points[index],
                            global(points[index], force),
                            SymbolShape::Arrow,
                        );
                    }
                } else if let Some(center) = region_center(model, &item.region, &visible) {
                    add(center, global(center, force), SymbolShape::Arrow);
                }
            }
            // A remote force: an arrow for the force and one for the moment at its point,
            // with lines to it from nodes spread over the region; at a reference point the
            // arrows alone.
            Kind::Load(LoadKind::RemoteForce(ref remote)) => {
                let (centre, nodes) = match item.region.reference_point() {
                    Some(name) => match model.fe.reference_point(name) {
                        Some(point) => (DVec3::from(point.position), None),
                        None => continue,
                    },
                    None => {
                        let none = Default::default();
                        let point = remote.point();
                        let Ok(point) = point.resolve(&model.fe, &model.mesh, &item.region, &none)
                        else {
                            continue;
                        };
                        (DVec3::from(point.position), Some(point.nodes))
                    }
                };
                let Ok((force, moment)) = remote.global(&model.fe, centre.into()) else {
                    continue;
                };
                add(centre, DVec3::from(force), SymbolShape::Arrow);
                add(centre, DVec3::from(moment), SymbolShape::MomentArrow);
                let to = (centre - model.origin()).as_vec3();
                let points = nodes.map_or_else(Vec::new, |nodes| {
                    node_points(model, &Region::Nodes(nodes), &visible)
                });
                for index in sample(&points, DIVISIONS / 2.0) {
                    symbols.push(Symbol {
                        position: (points[index] - model.origin()).as_vec3(),
                        direction: Vec3::ZERO,
                        shape: SymbolShape::LineTo(to),
                        color,
                        scale: 1.0,
                    });
                }
            }
            Kind::Load(LoadKind::PreTension {
                value, direction, ..
            }) => {
                // The preload pulls the cut together: arrows onto the faces, out of them
                // for a negative value.
                let faces = face_geometry(model, &item.region, &visible);
                let centers: Vec<DVec3> = faces.iter().map(|f| f.center).collect();
                for index in sample(&centers, DIVISIONS) {
                    let face = &faces[index];
                    let along = direction.map_or(face.normal, DVec3::from);
                    if value >= 0.0 {
                        add(face.center, -along, SymbolShape::ArrowOnto);
                    } else {
                        add(face.center, along, SymbolShape::Arrow);
                    }
                }
            }
            // Gravity as an arrow at the centre of the loaded elements.
            Kind::Load(ref kind @ (LoadKind::Gravity(_) | LoadKind::StandardGravity(_))) => {
                let acceleration = kind.gravity_vector().unwrap_or_default();
                if let Some(center) = region_center(model, &item.region, &visible) {
                    add(center, DVec3::from(acceleration), SymbolShape::Arrow);
                }
            }
            // The rotation axis through the loaded elements: the point of the axis closest
            // to their centre, with the direction of the axis.
            Kind::Load(LoadKind::Centrifugal { point, axis, .. }) => {
                if let Some(center) = region_center(model, &item.region, &visible)
                    && let Some(direction) = DVec3::from(axis).try_normalize()
                {
                    let point = DVec3::from(point);
                    let foot = point + (center - point).dot(direction) * direction;
                    add(foot, direction, SymbolShape::RotationLock);
                }
            }
            // The wheels at the first and the last position, pressing on the faces below.
            Kind::Load(LoadKind::Moving(ref moving)) => {
                let Ok((_, travel, lateral)) = moving.path_axes(&model.fe) else {
                    continue;
                };
                let (travel, lateral) = (DVec3::from(travel), DVec3::from(lateral));
                let two_d = model.fe.properties.space.is_2d();
                let centers: Vec<DVec3> = (face_geometry(model, &item.region, &visible))
                    .iter()
                    .map(|f| f.center)
                    .collect();
                let mut stations = moving.stations();
                stations.dedup_by(|a, b| a == b);
                if stations.len() > 2 {
                    stations = vec![stations[0], stations[stations.len() - 1]];
                }
                for station in stations {
                    let points = moving.wheel_points(&model.fe, station);
                    for (point, wheel) in points.into_iter().zip(&moving.wheels) {
                        let point = DVec3::from(point);
                        // Offset of a face centre from the wheel within the path's plane.
                        let in_plane = |c: DVec3| {
                            let d = c - point;
                            let across = if two_d { 0.0 } else { d.dot(lateral) };
                            d.dot(travel) * travel + across * lateral
                        };
                        let nearest = (centers.iter()).min_by(|a, b| {
                            in_plane(**a).length().total_cmp(&in_plane(**b).length())
                        });
                        let target = nearest.map_or(point, |&c| c - in_plane(c));
                        add(target, DVec3::from(wheel.force), SymbolShape::ArrowOnto);
                    }
                }
            }
            // Films and radiation leave the faces; a heat flow onto faces enters them, one
            // onto edges, points or into parts shows like a temperature.
            Kind::Load(
                ref kind @ (LoadKind::Flux { .. }
                | LoadKind::Film { .. }
                | LoadKind::Radiation { .. }),
            ) if !matches!(
                kind,
                LoadKind::Flux {
                    each_node: true,
                    ..
                }
            ) && !item.region.faces(&model.mesh).is_empty() =>
            {
                let faces = face_geometry(model, &item.region, &visible);
                let centers: Vec<DVec3> = faces.iter().map(|f| f.center).collect();
                let into = matches!(kind, LoadKind::Flux { value, .. } if *value >= 0.0);
                for index in sample(&centers, DIVISIONS) {
                    let face = &faces[index];
                    if into {
                        add(face.center, -face.normal, SymbolShape::ArrowOnto);
                    } else {
                        add(face.center, face.normal, SymbolShape::Arrow);
                    }
                }
            }
            Kind::Load(LoadKind::Film { .. } | LoadKind::Radiation { .. }) => {}
            Kind::Boundary(BoundaryKind::Temperature(_)) if sampler.is_some() => {
                let points = node_points(model, &item.region, &visible);
                let Some((factors, largest, picked)) = scaled(&points) else {
                    continue;
                };
                for index in picked {
                    let scale = factors[index].abs() / largest;
                    symbols.extend(scaled_symbol(
                        model,
                        points[index],
                        DVec3::Z,
                        SymbolShape::Ball,
                        color,
                        scale,
                    ));
                }
            }
            Kind::Load(LoadKind::Flux { .. }) | Kind::Boundary(BoundaryKind::Temperature(_)) => {
                let points = node_points(model, &item.region, &visible);
                for index in sample(&points, DIVISIONS) {
                    add(points[index], DVec3::Z, SymbolShape::Ball);
                }
            }
            // A cone along the normal of the plane and plates about its in-plane axes.
            Kind::Boundary(BoundaryKind::Symmetry { normal, .. }) => {
                let space = model.fe.properties.space;
                let plane = symmetry_plane(&model.mesh, &item.region, space, normal);
                if let (Ok(plane), Some(center)) =
                    (plane, region_center(model, &item.region, &visible))
                {
                    add(center, DVec3::from(plane.normal), SymbolShape::Cone);
                    if !space.is_2d() {
                        let (t1, t2) = plane.in_plane_axes();
                        add(center, DVec3::from(t1), SymbolShape::RotationLock);
                        add(center, DVec3::from(t2), SymbolShape::RotationLock);
                    }
                }
            }
            // A displacement on a point: the supports at the point with lines to it from
            // nodes spread over the region, like a rigid body's.
            Kind::Boundary(BoundaryKind::Displacement(ref remote))
                if matches!(
                    remote.formulation(&model.fe, &model.mesh, &item.region, "", ""),
                    plx_model::Formulation::Point(_)
                ) =>
            {
                let none = Default::default();
                let Ok(point) = remote.resolve(&model.fe, &model.mesh, &item.region, &none) else {
                    continue;
                };
                let centre = DVec3::from(point.position);
                let axes = point.axes.map_or(DVec3::AXES, |a| a.map(DVec3::from));
                let held = remote.held(model.fe.properties.space);
                for (axis, value) in held.iter().enumerate() {
                    let shape = if axis < 3 {
                        SymbolShape::Cone
                    } else {
                        SymbolShape::RotationLock
                    };
                    if value.is_some() {
                        add(centre, axes[axis % 3], shape);
                    }
                }
                let to = (centre - model.origin()).as_vec3();
                let points = node_points(model, &Region::Nodes(point.nodes), &visible);
                for index in sample(&points, DIVISIONS / 2.0) {
                    symbols.push(Symbol {
                        position: (points[index] - model.origin()).as_vec3(),
                        direction: Vec3::ZERO,
                        shape: SymbolShape::LineTo(to),
                        color,
                        scale: 1.0,
                    });
                }
            }
            // Cones and plates along the axes of the boundary condition's coordinate
            // system at the centre of the region; in a cylindrical system the directions
            // change over the region: at nodes spread over it.
            Kind::Boundary(ref kind) => {
                let (held, system): ([bool; 6], _) = match *kind {
                    BoundaryKind::Displacement(ref displacement) => (
                        displacement.values.map(|v| v.is_some()),
                        displacement.system,
                    ),
                    BoundaryKind::Temperature(_) => ([false; 6], None),
                    BoundaryKind::Submodel { dofs, .. } => (dofs, None),
                    BoundaryKind::Symmetry { .. } => unreachable!("drawn above"),
                };
                let frame = system.and_then(|id| model.fe.coordinate_system_or_global(Some(id)));
                let points = if frame
                    .as_ref()
                    .is_some_and(|f| f.kind == plx_model::CoordinateSystemKind::Cylindrical)
                {
                    let points = node_points(model, &item.region, &visible);
                    sample(&points, DIVISIONS)
                        .into_iter()
                        .map(|i| points[i])
                        .collect()
                } else {
                    Vec::from_iter(region_center(model, &item.region, &visible))
                };
                for point in points {
                    let axes = match &frame {
                        Some(frame) => match frame.directions_at(point.into()) {
                            Ok(axes) => axes.map(DVec3::from),
                            Err(_) => continue,
                        },
                        None => DVec3::AXES,
                    };
                    for (axis, &held) in held.iter().enumerate() {
                        let shape = if axis < 3 {
                            SymbolShape::Cone
                        } else {
                            SymbolShape::RotationLock
                        };
                        if held {
                            add(point, axes[axis % 3], shape);
                        }
                    }
                }
            }
        }
    }
    symbols
}

/// A symbol drawn `scale` times as large as usual; none where it would be too small to see
/// or the scale is no number.
fn scaled_symbol(
    model: &Model,
    position: DVec3,
    direction: DVec3,
    shape: SymbolShape,
    color: Color32,
    scale: f64,
) -> Option<Symbol> {
    if scale.is_nan() || scale < MIN_SCALE {
        return None;
    }
    Some(Symbol {
        position: (position - model.origin()).as_vec3(),
        direction: direction.try_normalize()?.as_vec3(),
        shape,
        color,
        scale: scale.min(1.0) as f32,
    })
}

/// Symbols of a distribution smaller than this part of the largest are left out.
const MIN_SCALE: f64 = 0.02;

/// The global directions a spring with this stiffness acts in.
fn acting(stiffness: [f64; 3]) -> impl Iterator<Item = DVec3> {
    (0..3)
        .filter(move |&i| stiffness[i] != 0.0)
        .map(|i| DVec3::AXES[i])
}

/// The parts shown; symbols on hidden parts are left out.
struct Visible {
    /// Elements of the hidden parts; empty when all parts are shown.
    hidden_elements: HashSet<ElementId>,
    /// Nodes of the shown parts, only when some part is hidden.
    nodes: Option<HashSet<NodeId>>,
}

impl Visible {
    fn new(model: &Model) -> Self {
        if model.parts.iter().all(|p| p.visible) {
            return Self {
                hidden_elements: HashSet::new(),
                nodes: None,
            };
        }
        let hidden_elements = (model.parts.iter().zip(&model.mesh.parts))
            .filter(|(info, _)| !info.visible)
            .flat_map(|(_, part)| part.elements.iter().copied())
            .collect();
        Self {
            hidden_elements,
            nodes: Some(model.visible_nodes().into_iter().collect()),
        }
    }

    fn node(&self, id: NodeId) -> bool {
        self.nodes.as_ref().is_none_or(|nodes| nodes.contains(&id))
    }

    fn element(&self, id: ElementId) -> bool {
        !self.hidden_elements.contains(&id)
    }
}

fn node_points(model: &Model, region: &Region, visible: &Visible) -> Vec<DVec3> {
    // A reference point has no node; its symbols sit on the point itself.
    if let Region::ReferencePoint(id) = region {
        return (model.fe.reference_point(*id).into_iter())
            .map(|point| DVec3::from(point.position))
            .collect();
    }
    // The point of a boundary condition or remote force, at the centre of its region.
    if let Region::RemotePoint(id) = region {
        return (model.fe.remote_point_of(&model.mesh, *id).into_iter())
            .filter_map(|(region, _)| region.centroid(&model.mesh))
            .map(DVec3::from)
            .collect();
    }
    let coords = model.exploded_coords();
    region
        .nodes(&model.mesh)
        .into_iter()
        .filter(|&id| visible.node(id))
        .filter_map(|id| model.mesh.node_index(id))
        .map(|index| DVec3::from(coords[index]))
        .collect()
}

struct FaceGeometry {
    center: DVec3,
    /// Unit normal pointing out of the element.
    normal: DVec3,
    area: f64,
}

/// Centre, outward normal and area of each element face of the region.
fn face_geometry(model: &Model, region: &Region, visible: &Visible) -> Vec<FaceGeometry> {
    let mesh = &model.mesh;
    let coords = model.exploded_coords();
    let position = |id: NodeId| mesh.node_index(id).map(|i| DVec3::from(coords[i]));
    let mut faces = Vec::new();
    for (element_id, face) in region.faces(mesh) {
        if !visible.element(element_id) {
            continue;
        }
        let Some(element) = mesh.element(element_id) else {
            continue;
        };
        let Some(topology) =
            (usize::from(face).checked_sub(1)).and_then(|f| element.faces().get(f))
        else {
            continue;
        };
        let corners: Option<Vec<DVec3>> = (topology.corners.iter())
            .map(|&local| element.nodes.get(local).copied().and_then(position))
            .collect();
        let Some(corners) = corners.filter(|c| c.len() >= 2) else {
            continue;
        };
        let center = corners.iter().sum::<DVec3>() / corners.len() as f64;
        let nodes: Vec<DVec3> = element.nodes.iter().filter_map(|&n| position(n)).collect();
        let centroid = nodes.iter().sum::<DVec3>() / nodes.len().max(1) as f64;
        let (area, normal) = if let [a, b] = corners[..] {
            // An edge of a 2D element: its length, and the normal in the element's plane.
            let along = b - a;
            let away = center - centroid;
            (
                along.length(),
                away - along * away.dot(along) / along.length_squared(),
            )
        } else {
            // Newell's method: twice the area along the normal, for any planar-ish polygon.
            let mut normal = DVec3::ZERO;
            for (i, a) in corners.iter().enumerate() {
                normal += a.cross(corners[(i + 1) % corners.len()]);
            }
            (normal.length() / 2.0, normal)
        };
        let Some(mut normal) = normal.try_normalize() else {
            continue;
        };
        if normal.dot(center - centroid) < 0.0 {
            normal = -normal;
        }
        faces.push(FaceGeometry {
            center,
            normal,
            area,
        });
    }
    faces
}

/// Area-weighted centre of the region's faces, or the mean of its nodes when it has none.
fn region_center(model: &Model, region: &Region, visible: &Visible) -> Option<DVec3> {
    let faces = face_geometry(model, region, visible);
    let area: f64 = faces.iter().map(|f| f.area).sum();
    if area > 0.0 {
        return Some(faces.iter().map(|f| f.center * f.area).sum::<DVec3>() / area);
    }
    let points = node_points(model, region, visible);
    (!points.is_empty()).then(|| points.iter().sum::<DVec3>() / points.len() as f64)
}

/// Indices of points spread evenly over their extent: clusters are halved across their
/// largest extent until none spans more than the whole split into `divisions`, then the
/// point nearest each cluster's mean stands for it. A simpler take on the spatial
/// point sampler.
fn sample(points: &[DVec3], divisions: f64) -> Vec<usize> {
    let span = |indices: &[usize]| {
        let (min, max) = indices.iter().fold(
            (DVec3::splat(f64::MAX), DVec3::splat(f64::MIN)),
            |(min, max), &i| (min.min(points[i]), max.max(points[i])),
        );
        (min, max - min)
    };
    let all: Vec<usize> = (0..points.len()).collect();
    if all.is_empty() {
        return all;
    }
    let limit = span(&all).1.max_element() / divisions;
    let mut pending = vec![all];
    let mut done = Vec::new();
    while let Some(cluster) = pending.pop() {
        let (min, extent) = span(&cluster);
        let largest = extent.max_element();
        if cluster.len() <= 1 || largest <= limit {
            done.push(cluster);
            continue;
        }
        let axis = if extent.x == largest {
            0
        } else if extent.y == largest {
            1
        } else {
            2
        };
        let middle = min[axis] + largest / 2.0;
        let (low, high): (Vec<usize>, Vec<usize>) =
            cluster.iter().partition(|&&i| points[i][axis] < middle);
        if low.is_empty() || high.is_empty() {
            done.push(cluster);
            continue;
        }
        pending.push(low);
        pending.push(high);
    }
    let mut picked: Vec<usize> = done
        .iter()
        .map(|cluster| {
            let mean = cluster.iter().map(|&i| points[i]).sum::<DVec3>() / cluster.len() as f64;
            *cluster
                .iter()
                .min_by(|&&a, &&b| {
                    (points[a].distance_squared(mean)).total_cmp(&points[b].distance_squared(mean))
                })
                .expect("clusters are not empty")
        })
        .collect();
    picked.sort_unstable();
    picked
}

/// Draws the symbols: the lines below the shapes, the shapes farthest first.
pub fn draw(painter: &Painter, rect: Rect, camera: &Camera, symbols: &[Symbol]) {
    let (right, up, forward) = (camera.right(), camera.up(), camera.forward());
    // Offsets in pixels on screen for a vector in pixel units of the scene.
    let screen = |v: Vec3| vec2(v.dot(right), -v.dot(up));
    for symbol in symbols {
        if let SymbolShape::LineTo(to) = symbol.shape {
            let from = crate::overlay::project(camera, rect, symbol.position);
            let to = crate::overlay::project(camera, rect, to);
            painter.line_segment([from, to], Stroke::new(LINE_WIDTH, symbol.color));
        }
    }
    let mut order: Vec<&Symbol> = (symbols.iter())
        .filter(|s| !matches!(s.shape, SymbolShape::LineTo(_)))
        .collect();
    order.sort_by(|a, b| (b.position.dot(forward)).total_cmp(&a.position.dot(forward)));
    for symbol in order {
        let anchor = crate::overlay::project(camera, rect, symbol.position);
        if !rect.expand(2.0 * ARROW_SIZE).contains(anchor) {
            continue;
        }
        if symbol.shape == SymbolShape::Spring {
            let points: Vec<Pos2> = (spring_wire(symbol.direction).into_iter())
                .map(|p| anchor + screen(p))
                .collect();
            let wire =
                |width: f32, color: Color32| Shape::line(points.clone(), Stroke::new(width, color));
            painter.add(wire(SPRING_WIDTH + 2.0, edge(symbol.color)));
            painter.add(wire(SPRING_WIDTH, symbol.color));
            continue;
        }
        let mut solids = solids(symbol.shape, symbol.direction, symbol.scale);
        solids.sort_by(|a, b| (b.depth.dot(forward)).total_cmp(&a.depth.dot(forward)));
        for solid in solids {
            let points: Vec<Pos2> = solid.points.iter().map(|&p| anchor + screen(p)).collect();
            let hull = convex_hull(points);
            if hull.len() >= 3 {
                painter.add(Shape::convex_polygon(
                    hull,
                    symbol.color,
                    Stroke::new(1.0, edge(symbol.color)),
                ));
            }
        }
    }
}

/// Dark symbol edges; they keep red symbols visible on the red
/// highlight of the selected region.
fn edge(color: Color32) -> Color32 {
    Color32::from_rgb(color.r() / 2, color.g() / 2, color.b() / 2)
}

/// A convex part of a symbol in pixel units relative to its point: its outline is the hull
/// of the points.
struct Solid {
    points: Vec<Vec3>,
    /// A point inside, to draw the nearer parts last.
    depth: Vec3,
}

/// The parts of a shape along a direction, after VTK glyphs (unit length is the
/// symbol size): the arrow source with tip length 0.3, tip radius 0.1 and shaft radius 0.03,
/// shifted 0.05 off the point; the cone of height 1 and radius 0.5; the rod of length 1.3
/// with a 0.4 thick plate of width 0.9.
fn solids(shape: SymbolShape, direction: Vec3, scale: f32) -> Vec<Solid> {
    let size = scale
        * match shape {
            SymbolShape::Arrow | SymbolShape::ArrowOnto | SymbolShape::MomentArrow => ARROW_SIZE,
            SymbolShape::Cone
            | SymbolShape::RotationLock
            | SymbolShape::Ball
            | SymbolShape::Mass
            | SymbolShape::GapSupport => SUPPORT_SIZE,
            SymbolShape::LineTo(_) | SymbolShape::Spring => return Vec::new(),
        };
    let axis = direction * size;
    let (u, v) = direction.any_orthonormal_pair();
    let disc = |at: f32, radius: f32| -> Vec<Vec3> {
        (0..CIRCLE_SEGMENTS)
            .map(|i| {
                let angle = i as f32 / CIRCLE_SEGMENTS as f32 * std::f32::consts::TAU;
                axis * at + (u * angle.cos() + v * angle.sin()) * radius * size
            })
            .collect()
    };
    let cylinder = |from: f32, to: f32, radius: f32| Solid {
        points: [disc(from, radius), disc(to, radius)].concat(),
        depth: axis * (from + to) / 2.0,
    };
    let cone = |base: f32, tip: f32, radius: f32| Solid {
        points: [disc(base, radius), vec![axis * tip]].concat(),
        depth: axis * (base + tip) / 2.0,
    };
    match shape {
        SymbolShape::Arrow | SymbolShape::ArrowOnto => {
            let start = if shape == SymbolShape::Arrow {
                0.05
            } else {
                -1.05
            };
            vec![
                cylinder(start, start + 0.7, 0.03),
                cone(start + 0.7, start + 1.0, 0.1),
            ]
        }
        SymbolShape::MomentArrow => vec![
            cylinder(0.05, 0.55, 0.03),
            cone(0.55, 0.8, 0.1),
            cone(0.8, 1.05, 0.1),
        ],
        SymbolShape::Cone => vec![cone(-1.0, 0.0, 0.5)],
        SymbolShape::GapSupport => {
            let mut plate = Vec::new();
            for along in [-1.2, -1.45] {
                for (a, b) in [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)] {
                    plate.push(axis * along + (u * a + v * b) * 0.6 * size);
                }
            }
            vec![
                cone(-1.0, 0.0, 0.4),
                Solid {
                    points: plate,
                    depth: axis * -1.325,
                },
            ]
        }
        SymbolShape::Ball | SymbolShape::Mass => {
            // Three great circles; their outline is round from every side.
            let radius = if shape == SymbolShape::Mass { 0.6 } else { 0.3 } * size;
            let circle = |a: Vec3, b: Vec3| -> Vec<Vec3> {
                (0..CIRCLE_SEGMENTS)
                    .map(|i| {
                        let angle = i as f32 / CIRCLE_SEGMENTS as f32 * std::f32::consts::TAU;
                        (a * angle.cos() + b * angle.sin()) * radius
                    })
                    .collect()
            };
            vec![Solid {
                points: [circle(u, v), circle(v, direction), circle(direction, u)].concat(),
                depth: Vec3::ZERO,
            }]
        }
        SymbolShape::RotationLock => {
            let mut plate = Vec::new();
            for along in [-1.35, -1.75] {
                for (a, b) in [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)] {
                    plate.push(axis * along + (u * a + v * b) * 0.45 * size);
                }
            }
            vec![
                cylinder(0.0, -1.3, 0.02),
                Solid {
                    points: plate,
                    depth: axis * -1.55,
                },
            ]
        }
        SymbolShape::LineTo(_) | SymbolShape::Spring => Vec::new(),
    }
}

/// The wire of a spring along a direction in pixel units relative to its point, with
/// these proportions: 1 long with straight ends of 0.2 and 4 coils of diameter 0.2,
/// ending 0.05 before the point.
fn spring_wire(direction: Vec3) -> Vec<Vec3> {
    // Half turns, and steps per half turn.
    const TURNS: usize = 8;
    const STEPS: usize = 10;
    let size = ARROW_SIZE;
    let (u, v) = direction.any_orthonormal_pair();
    let at = |along: f32, radial: Vec3| (direction * (along - 1.05) + radial) * size;
    let (straight, radius) = (0.2, 0.1);
    let mut points = vec![at(0.0, Vec3::ZERO), at(straight, Vec3::ZERO)];
    let rise = (1.0 - 2.0 * straight) / (TURNS * STEPS) as f32;
    for i in 0..=TURNS * STEPS {
        let angle = i as f32 / STEPS as f32 * std::f32::consts::PI;
        let radial = (u * angle.cos() + v * angle.sin()) * radius;
        points.push(at(straight + rise * i as f32, radial));
    }
    points.push(at(1.0 - straight, Vec3::ZERO));
    points.push(at(1.0, Vec3::ZERO));
    points
}

/// Convex hull in counter-clockwise order (Andrew's monotone chain).
fn convex_hull(mut points: Vec<Pos2>) -> Vec<Pos2> {
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    points.dedup_by(|a, b| (*a - *b).length_sq() < 1e-6);
    if points.len() < 3 {
        return points;
    }
    let cross = |o: Pos2, a: Pos2, b: Pos2| {
        let (oa, ob): (Vec2, Vec2) = (a - o, b - o);
        oa.x * ob.y - oa.y * ob.x
    };
    let mut hull: Vec<Pos2> = Vec::with_capacity(points.len() * 2);
    for pass in 0..2 {
        let start = hull.len();
        let iter: Box<dyn Iterator<Item = &Pos2>> = if pass == 0 {
            Box::new(points.iter())
        } else {
            Box::new(points.iter().rev())
        };
        for &p in iter {
            while hull.len() >= start + 2
                && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0
            {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    hull
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hull_drops_inner_points() {
        let points = vec![
            Pos2::new(0.0, 0.0),
            Pos2::new(2.0, 0.0),
            Pos2::new(1.0, 1.0),
            Pos2::new(2.0, 2.0),
            Pos2::new(0.0, 2.0),
        ];
        let hull = convex_hull(points);
        assert_eq!(hull.len(), 4);
        assert!(!hull.contains(&Pos2::new(1.0, 1.0)));
    }

    #[test]
    fn sampling_spreads_over_the_extent() {
        // 101 points on a line: about one per sixth of its length.
        let points: Vec<DVec3> = (0..=100).map(|i| DVec3::new(i as f64, 0.0, 0.0)).collect();
        let picked = sample(&points, DIVISIONS);
        assert!((6..=12).contains(&picked.len()), "{picked:?}");
        assert_eq!(sample(&points[..1], DIVISIONS), vec![0]);
        assert!(sample(&[], DIVISIONS).is_empty());
    }

    #[test]
    fn rigid_bodies_are_lines_to_their_reference_point() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata");
        let path = root.join("kragbalken_c3d8.inp");
        let mesh = plx_io::inp::read_inp(&path).unwrap().mesh;
        let mut model = Model::new(&path, mesh);
        model.fe.reference_points.push(plx_model::ReferencePoint {
            id: crate::test_ids::named("RP-1"),
            name: "RP-1".into(),
            position: [130.0, 5.0, 5.0],
            definition: Default::default(),
        });
        let item = |reference_point: &str| Item {
            kind: Kind::RigidBody {
                reference_point: crate::test_ids::named(reference_point),
            },
            region: Region::Surface("TIP".into()),
            system: None,
            distribution: None,
            selected: false,
        };
        let symbols = build(&model, &[item("RP-1")]);
        let to = (DVec3::new(130.0, 5.0, 5.0) - model.origin()).as_vec3();
        assert!(!symbols.is_empty());
        for symbol in &symbols {
            assert_eq!(symbol.shape, SymbolShape::LineTo(to));
            assert_eq!(symbol.color, CONSTRAINT_COLOR);
            // From nodes of the end face at x = 100.
            let from = symbol.position + model.origin().as_vec3();
            assert!((from.x - 100.0).abs() < 1e-3, "{from}");
        }
        assert!(build(&model, &[item("RP-2")]).is_empty());
    }

    #[test]
    fn pressure_arrows_follow_their_distribution() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/kragbalken_c3d8.inp");
        let mesh = plx_io::inp::read_inp(&path).unwrap().mesh;
        let mut model = Model::new(&path, mesh);
        let mut distribution = plx_model::Distribution::new("Rising");
        let id = distribution.id;
        let mut symbols_with = |model: &mut Model, equation: &str| {
            distribution.kind = plx_model::DistributionKind::Equation(equation.into());
            model.fe.distributions = vec![distribution.clone()];
            let item = Item {
                kind: Kind::Load(LoadKind::Pressure(2.0)),
                region: Region::Surface("TIP".into()),
                system: None,
                distribution: Some(id),
                selected: false,
            };
            build(model, &[item])
        };
        // Arrows as long as the factor where they are, the longest of full size; the unit of
        // y cancels out.
        let arrows = symbols_with(&mut model, "y");
        assert!(!arrows.is_empty());
        let largest = arrows.iter().map(|a| a.scale).fold(0.0, f32::max);
        assert!((largest - 1.0).abs() < 1e-6);
        for arrow in &arrows {
            assert_eq!(arrow.shape, SymbolShape::ArrowOnto);
            let y = arrow.position.y as f64 + model.origin().y;
            let expected = y / arrows
                .iter()
                .map(|a| a.position.y as f64 + model.origin().y)
                .fold(0.0, f64::max);
            assert!(
                (arrow.scale as f64 - expected).abs() < 1e-4,
                "{y}: {}",
                arrow.scale
            );
        }
        // A negative factor pulls, a factor of zero has no arrows.
        let pulled = symbols_with(&mut model, "-1");
        assert!(
            pulled
                .iter()
                .all(|a| a.shape == SymbolShape::Arrow && a.scale == 1.0)
        );
        assert!(symbols_with(&mut model, "0").is_empty());
    }

    #[test]
    fn springs_act_where_they_have_stiffness() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/kragbalken_c3d8.inp");
        let mesh = plx_io::inp::read_inp(&path).unwrap().mesh;
        let model = Model::new(&path, mesh);
        let spring = |nodes: Vec<NodeId>| Item {
            kind: Kind::Spring {
                stiffness: [1.0, 0.0, 2.0],
                to_surface: false,
            },
            region: Region::Nodes(nodes),
            system: None,
            distribution: None,
            selected: false,
        };
        // Two directions at each of three nodes, or once at the centre of many nodes.
        let few = build(&model, &[spring(vec![1, 12, 23])]);
        assert_eq!(few.len(), 6);
        assert!(
            few.iter()
                .all(|s| s.shape == SymbolShape::Spring && s.direction.y == 0.0)
        );
        assert_eq!(build(&model, &[spring((1..=20).collect())]).len(), 2);
        // Cones onto the faces of a compression only support, from outside, or out of them
        // when flipped.
        let support = |flipped| Item {
            kind: Kind::CompressionOnly { flipped },
            region: Region::Surface("TIP".into()),
            system: None,
            distribution: None,
            selected: false,
        };
        let cones = build(&model, &[support(false)]);
        assert!(!cones.is_empty());
        assert!(
            cones
                .iter()
                .all(|s| s.shape == SymbolShape::GapSupport && s.direction.x < -0.99)
        );
        let flipped = build(&model, &[support(true)]);
        assert_eq!(flipped.len(), cones.len());
        assert!(flipped.iter().all(|s| s.direction.x > 0.99));
        // The spring's wire ends just before the point.
        let wire = spring_wire(Vec3::X);
        assert!((wire.last().unwrap().x / ARROW_SIZE + 0.05).abs() < 1e-5);
    }

    #[test]
    fn supports_follow_the_axes_of_their_coordinate_system() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/kragbalken_c3d8.inp");
        let mesh = plx_io::inp::read_inp(&path).unwrap().mesh;
        let mut model = Model::new(&path, mesh);
        // Local x along global y, local y along -x.
        let turned = plx_model::CoordinateSystem::from_points(
            "turned",
            [0.0; 3],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        );
        // The cylinder axis along the beam (global x) through its middle.
        let mut cylinder = plx_model::CoordinateSystem::from_points(
            "cylinder",
            [0.0, 5.0, 5.0],
            [0.0, 6.0, 5.0],
            [0.0, 5.0, 6.0],
        );
        cylinder.kind = plx_model::CoordinateSystemKind::Cylindrical;
        let (turned_id, cylinder_id) = (turned.id, cylinder.id);
        model.fe.coordinate_systems.extend([turned, cylinder]);
        let support = |system| {
            let mut displacement =
                plx_model::Displacement::new([Some(0.0), None, None, None, None, None]);
            displacement.system = system;
            displacement.behavior = plx_model::Behavior::EachNode;
            Item {
                kind: Kind::Boundary(BoundaryKind::Displacement(displacement)),
                region: Region::Surface("TIP".into()),
                system: None,
                distribution: None,
                selected: false,
            }
        };
        let global = build(&model, &[support(None)]);
        assert_eq!(global.len(), 1);
        assert!(global[0].direction.x > 0.99);
        let turned = build(&model, &[support(Some(turned_id))]);
        assert_eq!(turned.len(), 1);
        assert!(turned[0].direction.y > 0.99, "{}", turned[0].direction);
        // Radial cones at nodes spread over the end face, pointing away from the axis.
        let radial = build(&model, &[support(Some(cylinder_id))]);
        assert!(radial.len() > 1);
        for symbol in &radial {
            let at = symbol.position + model.origin().as_vec3();
            let out = (Vec3::new(0.0, at.y - 5.0, at.z - 5.0)).normalize_or_zero();
            if out != Vec3::ZERO {
                assert!(
                    symbol.direction.dot(out) > 0.99,
                    "{} at {at}",
                    symbol.direction
                );
            }
        }
    }

    #[test]
    fn arrows_start_or_end_at_the_point() {
        let tip = |shape| {
            let parts = solids(shape, Vec3::X, 1.0);
            let head = &parts[1].points;
            head[head.len() - 1].x / ARROW_SIZE
        };
        assert!((tip(SymbolShape::Arrow) - 1.05).abs() < 1e-5);
        assert!((tip(SymbolShape::ArrowOnto) + 0.05).abs() < 1e-5);
    }
}
