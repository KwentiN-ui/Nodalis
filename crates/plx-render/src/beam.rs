//! Beam and truss elements drawn as the solid their section gives them: the profile swept
//! along the line, turned by the section's normal and moved off the nodes by its offset, as
//! the solver expands the beam.

use std::collections::HashMap;

use glam::DVec3;
use plx_mesh::PartSkin;
use plx_model::{BeamProfile, Outline};

use crate::contour::NO_VALUE;
use crate::fe::{EDGE_COLOR, FEATURE_EDGE_OPACITY};
use crate::mesh::{NodeBlend, RenderMesh, Vertex};

/// Largest angle in degrees between two segments of a line whose profiles are joined
/// without an end face in between.
const JOIN_ANGLE_DEG: f64 = 45.0;

/// The cross-section of a beam element as CalculiX places it: direction 1 is `normal`
/// projected onto the plane normal to the beam, direction 2 the beam axis crossed with
/// direction 1. `offset` moves the beam axis off the nodes in the 1- and 2-direction in
/// multiples of the profile's thickness there; 0.5 puts the nodes on the positive face.
#[derive(Clone, Debug, PartialEq)]
pub struct BeamShape {
    pub profile: BeamProfile,
    pub normal: DVec3,
    pub offset: [f64; 2],
}

impl BeamShape {
    /// A truss of cross-section `area` drawn as a round bar of the same area.
    pub fn truss(area: f64) -> Option<Self> {
        (area > 0.0 && area.is_finite()).then(|| Self {
            profile: BeamProfile::Circ {
                radius: (area / std::f64::consts::PI).sqrt(),
            },
            normal: DVec3::Z,
            offset: [0.0; 2],
        })
    }

    /// The thickness of the profile in the 1- and 2-direction, which the offsets refer to.
    fn size(&self) -> [f64; 2] {
        self.profile.size()
    }

    /// The outlines of the profile about its centre in the 1-2 plane, counter-clockwise: the
    /// outer one first, then the hole of a hollow profile with as many points.
    fn loops(&self) -> Vec<Outline> {
        self.profile.outlines()
    }
}

/// Where a profile sits at one end of a segment.
#[derive(Clone, Copy)]
struct Frame {
    centre: DVec3,
    n1: DVec3,
    n2: DVec3,
    tangent: DVec3,
}

impl Frame {
    fn at(&self, [u, v]: [f64; 2]) -> DVec3 {
        self.centre + self.n1 * u + self.n2 * v
    }

    fn direction(&self, [u, v]: [f64; 2]) -> DVec3 {
        (self.n1 * u + self.n2 * v).normalize_or_zero()
    }
}

/// The solid of the line segments of `skin` that `shape` gives a profile, by their index in
/// [`PartSkin::lines`]. Two segments meeting at a node at a small angle share the profile
/// there; it ends with a face where a line ends, branches or kinks. The outline of the
/// profile is a mesh edge where elements meet, and the corners of a flat profile run along
/// the beam as feature edges.
///
/// `scalars` holds a normalized contour value per node, as [`crate::part_render_mesh`]
/// takes it. `rest` are the undeformed coordinates, where the section's normal places the
/// profile; on the deformed line `coords` the profile turns along with the line, so that a
/// beam bent until its axis runs along the normal does not twist.
#[allow(clippy::too_many_arguments)]
pub fn beam_mesh(
    coords: &[[f64; 3]],
    rest: &[[f64; 3]],
    skin: &PartSkin,
    origin: DVec3,
    color: [f32; 3],
    segment_color: impl Fn(usize) -> Option<[f32; 3]>,
    shape: impl Fn(usize) -> Option<BeamShape>,
    scalars: Option<&[f32]>,
) -> RenderMesh {
    let mut render = RenderMesh::default();
    let shapes: Vec<Option<BeamShape>> = (0..skin.lines.len())
        .map(|index| shape(index).filter(|s| s.profile.is_valid()))
        .collect();
    if shapes.iter().all(Option::is_none) {
        return render;
    }
    let point = |node: usize| DVec3::from(coords[node]);
    let rest_point = |node: usize| DVec3::from(rest.get(node).copied().unwrap_or(coords[node]));
    let direction_in = |point: &dyn Fn(usize) -> DVec3, index: usize| {
        let [a, b] = skin.lines[index];
        (point(b) - point(a)).normalize_or_zero()
    };
    let direction = |index: usize| direction_in(&point, index);
    // The ends of the segments with a profile at each node: (segment, end).
    let mut ends: HashMap<usize, Vec<(usize, usize)>> = HashMap::new();
    for (index, line) in skin.lines.iter().enumerate() {
        if shapes[index].is_some() {
            for (end, &node) in line.iter().enumerate() {
                ends.entry(node).or_default().push((index, end));
            }
        }
    }
    // The segment end joined to each segment end without an end face, if any.
    let joined = |index: usize, end: usize| -> Option<(usize, usize)> {
        let at = &ends[&skin.lines[index][end]];
        let &[first, second] = at.as_slice() else {
            return None;
        };
        let other = if first == (index, end) { second } else { first };
        // Along the line the other segment runs the same way when it starts where this
        // one ends.
        let sign = if other.1 == end { -1.0 } else { 1.0 };
        let cos = direction(index).dot(direction(other.0) * sign);
        (other.0 != index && cos >= JOIN_ANGLE_DEG.to_radians().cos()).then_some(other)
    };
    // The tangent at a segment end, the mean of both segments where two join.
    let tangent_in = |point: &dyn Fn(usize) -> DVec3, index: usize, end: usize| {
        let own = direction_in(point, index);
        match joined(index, end) {
            Some((other, other_end)) => {
                let sign = if other_end == end { -1.0 } else { 1.0 };
                (own + direction_in(point, other) * sign).normalize_or(own)
            }
            None => own,
        }
    };
    // Direction 1 at a segment end on the undeformed line: the normal projected.
    let rest_n1 = |index: usize, end: usize| {
        let normal = shapes[index]
            .as_ref()
            .expect("segments with a profile only")
            .normal;
        let tangent = tangent_in(&rest_point, index, end);
        (normal - tangent * normal.dot(tangent))
            .try_normalize()
            .unwrap_or_else(|| tangent.any_orthonormal_vector())
    };
    // On the deformed line direction 1 is carried along each chain of joined segments, from
    // the end that turned least, by the rotation between neighbouring tangents and the
    // twist the undeformed line has there.
    let mut n1s: HashMap<(usize, usize), DVec3> = HashMap::new();
    let mut seeds: Vec<(f64, usize)> = (0..skin.lines.len())
        .filter(|&i| shapes[i].is_some())
        .map(|i| {
            let turn = tangent_in(&rest_point, i, 0).dot(tangent_in(&point, i, 0));
            (-turn, i)
        })
        .collect();
    seeds.sort_by(|a, b| a.0.total_cmp(&b.0));
    for (_, seed) in seeds {
        if n1s.contains_key(&(seed, 0)) {
            continue;
        }
        let start = turned(
            rest_n1(seed, 0),
            tangent_in(&rest_point, seed, 0),
            tangent_in(&point, seed, 0),
        );
        // Both ways from the seed's first end: along the seed, and back into the segment
        // joined before it.
        let mut walks = vec![(seed, 0, start)];
        if let Some((other, other_end)) = joined(seed, 0) {
            walks.push((other, other_end, start));
        }
        for (mut index, mut from, mut n1) in walks {
            while !n1s.contains_key(&(index, from)) {
                n1s.insert((index, from), n1);
                let to = 1 - from;
                let (t0, t1) = (
                    tangent_in(&point, index, from),
                    tangent_in(&point, index, to),
                );
                let (r0, r1) = (
                    tangent_in(&rest_point, index, from),
                    tangent_in(&rest_point, index, to),
                );
                // The twist of the undeformed line between the two ends, about its tangent.
                let carried = turned(rest_n1(index, from), r0, r1);
                let target = rest_n1(index, to);
                let twist = carried.cross(target).dot(r1).atan2(carried.dot(target));
                n1 = rotate(turned(n1, t0, t1), t1, twist);
                n1s.insert((index, to), n1);
                match joined(index, to) {
                    Some((other, other_end)) if !n1s.contains_key(&(other, other_end)) => {
                        (index, from) = (other, other_end);
                    }
                    _ => break,
                }
            }
        }
    }
    let frame = |index: usize, end: usize| -> Frame {
        let shape = shapes[index]
            .as_ref()
            .expect("segments with a profile only");
        let tangent = tangent_in(&point, index, end);
        let n1 = (n1s[&(index, end)] - tangent * n1s[&(index, end)].dot(tangent))
            .try_normalize()
            .unwrap_or_else(|| tangent.any_orthonormal_vector());
        let n2 = tangent.cross(n1);
        let size = shape.size();
        let centre = point(skin.lines[index][end])
            - n1 * (shape.offset[0] * size[0])
            - n2 * (shape.offset[1] * size[1]);
        Frame {
            centre,
            n1,
            n2,
            tangent,
        }
    };
    let render_position = |p: DVec3| (p - origin).as_vec3().to_array();
    let scalar = |node: usize| scalars.map_or(NO_VALUE, |s| s[node]);
    let line = |p: DVec3| Vertex {
        position: render_position(p),
        normal: [0.0; 3],
        color: EDGE_COLOR,
        scalar: FEATURE_EDGE_OPACITY,
    };

    for (index, &[a, b]) in skin.lines.iter().enumerate() {
        let Some(shape) = &shapes[index] else {
            continue;
        };
        let color = segment_color(index).unwrap_or(color);
        let frames = [frame(index, 0), frame(index, 1)];
        let nodes = [a, b];
        let vertex = |end: usize, position: DVec3, normal: DVec3| Vertex {
            position: render_position(position),
            normal: normal.as_vec3().to_array(),
            color,
            scalar: scalar(nodes[end]),
        };
        let loops = shape.loops();
        // The walls along the segment; a hole faces inwards.
        for (k, outline) in loops.iter().enumerate() {
            let hole = k > 0;
            let n = outline.points.len();
            for i in 0..n {
                let (p, q) = (outline.points[i], outline.points[(i + 1) % n]);
                let flat = DVec3::new(q[1] - p[1], p[0] - q[0], 0.0).normalize_or_zero();
                let normal = |frame: &Frame, at: [f64; 2]| {
                    let outward = if outline.round {
                        frame.direction(at)
                    } else {
                        (frame.n1 * flat.x + frame.n2 * flat.y).normalize_or_zero()
                    };
                    if hole { -outward } else { outward }
                };
                let base = render.vertices.len() as u32;
                for (end, at) in [(0, p), (0, q), (1, q), (1, p)] {
                    let frame = &frames[end];
                    render
                        .vertices
                        .push(vertex(end, frame.at(at), normal(frame, at)));
                    render.origins.push(NodeBlend::at(nodes[end]));
                }
                let quad = if hole {
                    [2, 1, 0, 3, 2, 0]
                } else {
                    [0, 1, 2, 0, 2, 3]
                };
                render.triangles.extend(quad.map(|j| base + j));
                // The outline where the segment ends at another element, drawn by one of
                // the two.
                for end in 0..2 {
                    let other = joined(index, end);
                    if other.is_some_and(|(o, other_end)| {
                        skin.line_elements[o] != skin.line_elements[index]
                            && (index, end) < (o, other_end)
                    }) {
                        let pair = if end == 0 { [0, 1] } else { [3, 2] };
                        render.mesh_edges.extend(pair.map(|j| base + j));
                    }
                }
                if !outline.round {
                    render
                        .feature_edges
                        .extend([line(frames[0].at(p)), line(frames[1].at(p))]);
                }
            }
        }
        // End faces where the profile does not continue into another segment.
        for end in 0..2 {
            if joined(index, end).is_some() {
                continue;
            }
            let frame = &frames[end];
            // The face looks along the beam at the far end, against it at the near one.
            let facing = if end == 1 { 1.0 } else { -1.0 };
            let normal = frame.tangent * facing;
            let outer = &loops[0].points;
            let n = outer.len();
            let base = render.vertices.len() as u32;
            let mut push = |at: [f64; 2]| {
                render.vertices.push(vertex(end, frame.at(at), normal));
                render.origins.push(NodeBlend::at(nodes[end]));
            };
            let mut triangles: Vec<[u32; 3]> = Vec::new();
            match loops.get(1) {
                // A pipe or box: the wall between two outlines of as many points.
                Some(hole) if loops.len() == 2 && hole.points.len() == n => {
                    for &at in outer.iter().chain(&hole.points) {
                        push(at);
                    }
                    let n = n as u32;
                    for i in 0..n {
                        let j = (i + 1) % n;
                        triangles.push([i, j, n + j]);
                        triangles.push([i, n + j, n + i]);
                    }
                }
                None if loops[0].round => {
                    push([0.0, 0.0]);
                    for &at in outer {
                        push(at);
                    }
                    let n = n as u32;
                    for i in 0..n {
                        triangles.push([0, 1 + i, 1 + (i + 1) % n]);
                    }
                }
                // A sketch with holes of their own shape.
                Some(_) => {
                    let rings: Vec<Vec<[f64; 2]>> =
                        loops.iter().map(|o| o.points.clone()).collect();
                    for &at in rings.iter().flatten() {
                        push(at);
                    }
                    triangles.extend(
                        (plx_model::triangulate_with_holes(&rings).into_iter())
                            .map(|t| t.map(|k| k as u32)),
                    );
                }
                // I, T, U and L profiles are concave: a fan from the centre would not fill
                // them.
                None => {
                    for &at in outer {
                        push(at);
                    }
                    triangles.extend(
                        (plx_model::triangulate(outer).into_iter()).map(|t| t.map(|k| k as u32)),
                    );
                }
            }
            for [p, q, r] in triangles {
                let triangle = if end == 1 { [p, q, r] } else { [r, q, p] };
                render.triangles.extend(triangle.map(|j| base + j));
            }
            for outline in &loops {
                let m = outline.points.len();
                for i in 0..m {
                    let (p, q) = (outline.points[i], outline.points[(i + 1) % m]);
                    render
                        .feature_edges
                        .extend([line(frame.at(p)), line(frame.at(q))]);
                }
            }
        }
    }
    render
}

/// `v` turned by the smallest rotation that takes the unit vector `from` to `to`; about any
/// axis normal to them when they point opposite ways.
fn turned(v: DVec3, from: DVec3, to: DVec3) -> DVec3 {
    let axis = from.cross(to);
    let (sin, cos) = (axis.length(), from.dot(to));
    if sin < 1e-12 {
        return if cos > 0.0 {
            v
        } else {
            rotate(v, from.any_orthonormal_vector(), std::f64::consts::PI)
        };
    }
    rotate(v, axis / sin, sin.atan2(cos))
}

/// `v` turned by `angle` about the unit `axis` (Rodrigues).
fn rotate(v: DVec3, axis: DVec3, angle: f64) -> DVec3 {
    let (sin, cos) = angle.sin_cos();
    v * cos + axis.cross(v) * sin + axis * axis.dot(v) * (1.0 - cos)
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_mesh::{Element, ElementShape, FeMesh, Part, extract_part_skin};

    /// A straight beam along x from 0 to 2 of two `Line2` elements.
    fn beam() -> FeMesh {
        let mut mesh = FeMesh::default();
        for i in 0..3 {
            mesh.set_node(i + 1, [i as f64, 0.0, 0.0]);
        }
        for id in 1..3 {
            mesh.add_element(Element {
                id,
                type_name: "B31".into(),
                shape: ElementShape::Line2,
                nodes: vec![id, id + 1],
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "P".into(),
            elements: vec![1, 2],
        });
        mesh
    }

    fn swept(mesh: &FeMesh, shape: Option<BeamShape>) -> RenderMesh {
        let skin = extract_part_skin(mesh, &mesh.parts[0], 30.0);
        beam_mesh(
            mesh.coords(),
            mesh.coords(),
            &skin,
            DVec3::ZERO,
            [1.0; 3],
            |_| None,
            |_| shape.clone(),
            None,
        )
    }

    fn range(render: &RenderMesh, axis: usize) -> (f32, f32) {
        (render.vertices.iter().map(|v| v.position[axis]))
            .fold((f32::MAX, f32::MIN), |(lo, hi), x| (lo.min(x), hi.max(x)))
    }

    /// The enclosed volume of a closed triangle mesh.
    fn volume(render: &RenderMesh) -> f32 {
        (render.triangles.chunks(3))
            .map(|t| {
                let [a, b, c] =
                    [0, 1, 2].map(|k| glam::Vec3::from(render.vertices[t[k] as usize].position));
                a.dot(b.cross(c)) / 6.0
            })
            .sum()
    }

    fn rect(a: f64, b: f64) -> BeamShape {
        BeamShape {
            profile: BeamProfile::Rect { a, b },
            normal: DVec3::Z,
            offset: [0.0; 2],
        }
    }

    #[test]
    fn the_rectangle_lies_along_the_normal_and_the_axis_crossed_with_it() {
        // Direction 1 is z, direction 2 is x cross z = -y.
        let render = swept(&beam(), Some(rect(0.4, 0.2)));
        let (lo, hi) = range(&render, 2);
        assert!(
            (lo + 0.2).abs() < 1e-6 && (hi - 0.2).abs() < 1e-6,
            "{lo} {hi}"
        );
        let (lo, hi) = range(&render, 1);
        assert!(
            (lo + 0.1).abs() < 1e-6 && (hi - 0.1).abs() < 1e-6,
            "{lo} {hi}"
        );
        // Closed and facing outwards: the volume is positive.
        assert!(
            (volume(&render) - 2.0 * 0.4 * 0.2).abs() < 1e-5,
            "{}",
            volume(&render)
        );
        // One outline where the two elements meet, four corner lines along each element and
        // the outlines of the two end faces.
        assert_eq!(render.mesh_edges.len(), 4 * 2);
        assert_eq!(render.feature_edges.len(), (4 * 2 + 4 * 2) * 2);
    }

    #[test]
    fn offset_moves_the_profile_off_the_nodes() {
        let shape = BeamShape {
            offset: [0.5, -0.5],
            ..rect(0.4, 0.2)
        };
        let render = swept(&beam(), Some(shape));
        // The nodes on the positive face in 1 (z): the profile below them.
        let (lo, hi) = range(&render, 2);
        assert!((lo + 0.4).abs() < 1e-6 && hi.abs() < 1e-6, "{lo} {hi}");
        // On the negative face in 2 (-y): the profile in 2, along -y.
        let (lo, hi) = range(&render, 1);
        assert!((lo + 0.2).abs() < 1e-6 && hi.abs() < 1e-6, "{lo} {hi}");
    }

    #[test]
    fn hollow_profiles_enclose_their_walls_only() {
        let pipe = BeamShape {
            profile: BeamProfile::Pipe {
                radius: 0.2,
                thickness: 0.05,
            },
            normal: DVec3::Z,
            offset: [0.0; 2],
        };
        let polygon = |r: f64| {
            let n = plx_model::ROUND_POINTS as f64;
            0.5 * n * r * r * (std::f64::consts::TAU / n).sin()
        };
        let wall = 2.0 * (polygon(0.2) - polygon(0.15)) as f32;
        let render = swept(&beam(), Some(pipe.clone()));
        assert!(
            (volume(&render) - wall).abs() < 1e-5,
            "{} {wall}",
            volume(&render)
        );
        let boxed = BeamShape {
            profile: BeamProfile::Box {
                a: 0.4,
                b: 0.2,
                t: [0.01, 0.02, 0.03, 0.04],
            },
            ..pipe
        };
        let wall = 2.0 * (0.4 * 0.2 - (0.4 - 0.04) * (0.2 - 0.06)) as f32;
        let render = swept(&beam(), Some(boxed));
        assert!(
            (volume(&render) - wall).abs() < 1e-5,
            "{} {wall}",
            volume(&render)
        );
        // The thicker wall at +1 (z) is t1, at -1 t3.
        let inner_top = (render.vertices.iter().map(|v| v.position[2]))
            .filter(|&z| z < 0.199)
            .fold(f32::MIN, f32::max);
        assert!((inner_top - 0.19).abs() < 1e-6, "{inner_top}");
    }

    #[test]
    fn a_kink_ends_both_profiles_with_a_face() {
        let mut mesh = beam();
        mesh.set_node(3, [1.0, 1.0, 0.0]);
        let render = swept(&mesh, Some(rect(0.1, 0.1)));
        // Four walls of two triangles along each element and four end faces of two.
        assert_eq!(render.triangles.len(), (2 * 4 * 2 + 4 * 2) * 3);
        assert!(render.mesh_edges.is_empty());
    }

    #[test]
    fn concave_profiles_close_their_ends() {
        let shape = BeamShape {
            profile: BeamProfile::I {
                h: 0.2,
                b: 0.1,
                tw: 0.01,
                tf: 0.02,
            },
            normal: DVec3::Z,
            offset: [0.0; 2],
        };
        let area = shape.profile.properties().area as f32;
        let render = swept(&beam(), Some(shape));
        assert!(
            (volume(&render) - 2.0 * area).abs() < 1e-5,
            "{} {}",
            volume(&render),
            2.0 * area
        );
    }

    #[test]
    fn sketches_close_their_ends_around_their_holes() {
        let shape = BeamShape {
            profile: BeamProfile::Sketch(plx_model::Sketch {
                loops: vec![
                    vec![[0.0, 0.0], [0.2, 0.0], [0.2, 0.1], [0.1, 0.15], [0.0, 0.1]],
                    vec![[0.02, 0.02], [0.05, 0.02], [0.05, 0.05]],
                    vec![[0.12, 0.03], [0.17, 0.03], [0.17, 0.07], [0.12, 0.07]],
                ],
            }),
            normal: DVec3::Z,
            offset: [0.0; 2],
        };
        let area = shape.profile.properties().area as f32;
        let render = swept(&beam(), Some(shape));
        assert!(
            (volume(&render) - 2.0 * area).abs() < 1e-5,
            "{} {}",
            volume(&render),
            2.0 * area
        );
    }

    #[test]
    fn segments_without_a_profile_stay_lines() {
        assert!(swept(&beam(), None).vertices.is_empty());
        assert!(swept(&beam(), Some(rect(0.0, 0.1))).vertices.is_empty());
    }

    /// The profile lies where the solver puts the nodes of the expanded beam: a rectangle with
    /// offsets on a beam askew to the axes, its normal not perpendicular to it.
    #[test]
    fn rectangles_lie_where_calculix_expands_the_beam() {
        let direction = DVec3::new(1.0, 2.0, 0.5).normalize();
        let mut mesh = FeMesh::default();
        let mut text = String::from("*Node\n");
        for i in 0..3 {
            let p = direction * (i as f64 * 10.0) + DVec3::new(3.0, -2.0, 1.0);
            mesh.set_node(i + 1, p.to_array());
            text.push_str(&format!("{}, {}, {}, {}\n", i + 1, p.x, p.y, p.z));
        }
        for id in 1..3 {
            mesh.add_element(Element {
                id,
                type_name: "B31".into(),
                shape: ElementShape::Line2,
                nodes: vec![id, id + 1],
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "P".into(),
            elements: vec![1, 2],
        });
        let shape = BeamShape {
            profile: BeamProfile::Rect { a: 2.0, b: 1.0 },
            normal: DVec3::new(0.2, 0.0, 1.0),
            offset: [0.5, -0.3],
        };
        text.push_str(
            "*Element, Type=B31, Elset=Beam\n1, 1, 2\n2, 2, 3\n\
             *Material, Name=Steel\n*Elastic\n210000, 0.3\n\
             *Beam section, Elset=Beam, Material=Steel, Section=RECT, Offset1=0.5, Offset2=-0.3\n\
             2, 1\n0.2, 0, 1\n\
             *Step\n*Static\n*Boundary\n1, 1, 6\n*Node file\nU\n*End step\n",
        );
        let dir = std::env::temp_dir().join(format!("plx-beam-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("beam.inp"), text).unwrap();
        plxs_session::run_input_file(&dir, "beam").unwrap_or_else(|e| panic!("{e}"));
        let frd = plx_io::frd::read_frd(&dir.join("beam.frd")).unwrap();
        let _ = std::fs::remove_dir_all(&dir);
        let render = swept(&mesh, Some(shape));
        let near =
            |p: glam::Vec3, points: &[glam::Vec3]| points.iter().any(|q| (p - *q).length() < 1e-4);
        let expanded: Vec<glam::Vec3> = (frd.mesh.coords().iter())
            .map(|c| DVec3::from(*c).as_vec3())
            .collect();
        // Every corner of the drawn profile is a node of the expanded beam; the centres of
        // the end faces are not.
        let axis = direction.as_vec3();
        let corners: Vec<glam::Vec3> = (render.vertices.iter())
            .filter(|v| glam::Vec3::from(v.normal).dot(axis).abs() < 0.5)
            .map(|v| glam::Vec3::from(v.position))
            .collect();
        for &p in &corners {
            assert!(near(p, &expanded), "{p} not expanded");
        }
        // And the expanded nodes at the beam's nodes are corners of the profile.
        let ends = [0.0f32, 10.0, 20.0];
        let along =
            |p: glam::Vec3| (p - DVec3::new(3.0, -2.0, 1.0).as_vec3()).dot(direction.as_vec3());
        let at_ends = |p: &&glam::Vec3| ends.iter().any(|e| (along(**p) - e).abs() < 1e-4);
        let checked: Vec<_> = expanded.iter().filter(at_ends).collect();
        assert_eq!(checked.len(), 3 * 4);
        for &p in checked {
            assert!(near(p, &corners), "{p} not drawn");
        }
    }

    #[test]
    fn trusses_are_round_bars_of_their_area() {
        let shape = BeamShape::truss(std::f64::consts::PI * 0.04).unwrap();
        let BeamProfile::Circ { radius } = shape.profile else {
            panic!("{shape:?}");
        };
        assert!((radius - 0.2).abs() < 1e-12, "{radius}");
        assert!(BeamShape::truss(0.0).is_none());
    }

    /// A rope along x with its normal along y, bent into a half circle in the x-y plane: its
    /// axis runs along the normal half way. The profile follows the bend without twisting.
    #[test]
    fn a_beam_bent_along_its_normal_does_not_twist() {
        let n = 36;
        let mut mesh = FeMesh::default();
        let length = std::f64::consts::PI * 100.0;
        for i in 0..=n {
            mesh.set_node(i + 1, [length * f64::from(i) / f64::from(n), 0.0, 0.0]);
        }
        for id in 1..=n {
            mesh.add_element(Element {
                id,
                type_name: "B31".into(),
                shape: ElementShape::Line2,
                nodes: vec![id, id + 1],
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "P".into(),
            elements: (1..=n).collect(),
        });
        let rest = mesh.coords().to_vec();
        let bent: Vec<[f64; 3]> = (0..=n)
            .map(|i| {
                let angle = std::f64::consts::PI * f64::from(i) / f64::from(n);
                [100.0 * angle.sin(), 100.0 * (1.0 - angle.cos()), 0.0]
            })
            .collect();
        let skin = extract_part_skin(&mesh, &mesh.parts[0], 30.0);
        let shape = BeamShape {
            profile: BeamProfile::Circ { radius: 5.0 },
            normal: DVec3::Y,
            offset: [0.0; 2],
        };
        let render = beam_mesh(
            &bent,
            &rest,
            &skin,
            DVec3::ZERO,
            [1.0; 3],
            |_| None,
            |_| Some(shape.clone()),
            None,
        );
        // Each wall quad runs from a point of the outline at one end to the same point at
        // the other: no further than the segment's length plus the bend.
        let segment = length / f64::from(n);
        let node = |k: usize| render.origins[k].a;
        let mut k = 0;
        let mut quads = 0;
        while k + 3 < render.vertices.len() {
            if node(k) == node(k + 1) && node(k + 2) == node(k + 3) && node(k) != node(k + 2) {
                let at = |j: usize| DVec3::from(render.vertices[j].position.map(f64::from));
                let gap = (at(k) - at(k + 3)).length();
                assert!(
                    gap < 1.2 * segment,
                    "{gap} vs {segment} at node {}",
                    node(k)
                );
                quads += 1;
                k += 4;
            } else {
                k += 1;
            }
        }
        assert!(quads >= n as usize * 8, "{quads}");
    }
}
