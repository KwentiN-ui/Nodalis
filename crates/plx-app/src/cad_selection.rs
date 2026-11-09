//! Selections on a mesh generated from CAD geometry by its faces, edges and vertices.
//! They are stored as CAD entities and found on
//! the mesh again after remeshing, see [`plx_model::Region::Geometry`].

use std::collections::{BTreeMap, BTreeSet, HashMap};

use glam::Vec3;
use plx_mesh::{CadEntity, ElementId, FeMesh, NodeId};

use crate::model::{Highlight, Hit, Model};
use crate::selection::Target;
use crate::viewport::Preview;

/// Lookups from the mesh to the CAD entities, built once per mesh.
#[derive(Debug, Default)]
pub struct CadIndex {
    /// The CAD face of each element face; shell and 2D elements with face 1.
    face_of: HashMap<(ElementId, u8), i32>,
    /// The CAD edges bounding each CAD face.
    face_edges: BTreeMap<i32, Vec<i32>>,
    /// The CAD vertices at the ends of each CAD edge.
    edge_vertices: BTreeMap<i32, Vec<i32>>,
    /// The CAD edge of each mesh segment along one by its end nodes, the smaller first.
    segment_edge: HashMap<[NodeId; 2], i32>,
}

impl CadIndex {
    pub fn new(mesh: &FeMesh) -> Self {
        let cad = &mesh.cad;
        let nodes_of = |entity| -> BTreeSet<NodeId> {
            cad.nodes
                .get(&entity)
                .into_iter()
                .flatten()
                .copied()
                .collect()
        };
        let mut face_edges: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
        let mut edge_vertices: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
        let vertices: Vec<(i32, NodeId)> = (cad.nodes.iter())
            .filter_map(|(entity, nodes)| match entity {
                CadEntity::Vertex(tag) => Some((*tag, *nodes.first()?)),
                _ => None,
            })
            .collect();
        for &face in cad.faces.keys() {
            let on_face = nodes_of(CadEntity::Face(face));
            for (&edge, segments) in &cad.segments {
                if segments
                    .first()
                    .is_some_and(|s| s.iter().all(|n| on_face.contains(n)))
                {
                    face_edges.entry(face).or_default().push(edge);
                }
            }
        }
        for &edge in cad.segments.keys() {
            let on_edge = nodes_of(CadEntity::Edge(edge));
            edge_vertices.insert(
                edge,
                (vertices.iter())
                    .filter(|(_, node)| on_edge.contains(node))
                    .map(|&(tag, _)| tag)
                    .collect(),
            );
        }
        let segment_edge = (cad.segments.iter())
            .flat_map(|(&edge, segments)| {
                segments
                    .iter()
                    .map(move |&[a, b]| ([a.min(b), a.max(b)], edge))
            })
            .collect();
        Self {
            face_of: cad.face_of().into_iter().collect(),
            face_edges,
            edge_vertices,
            segment_edge,
        }
    }

    /// The vertex at an end of a CAD edge within `precision` of `point`, the nearest if both.
    fn nearest_vertex(&self, model: &Model, edge: i32, point: Vec3, precision: f32) -> Option<i32> {
        (self.edge_vertices.get(&edge)?.iter())
            .filter_map(|&v| {
                let node = *model.mesh.cad.nodes.get(&CadEntity::Vertex(v))?.first()?;
                let position = model.render_position(model.mesh.node_index(node)?);
                Some((v, position.distance(point)))
            })
            .filter(|&(_, d)| d <= precision)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(v, _)| v)
    }

    /// The CAD face an element face of the skin lies on.
    fn face(&self, mesh: &FeMesh, (element, face): (ElementId, u8)) -> Option<i32> {
        if let Some(&tag) = self.face_of.get(&(element, face)) {
            return Some(tag);
        }
        // Shell and 2D elements lie on their CAD face as a whole.
        let element = mesh.element(element)?;
        (element.shape.faces().len() == 1)
            .then(|| self.face_of.get(&(element.id, 1)).copied())
            .flatten()
    }
}

/// What a click selects by geometry on a mesh generated from CAD geometry: the vertex or
/// edge near the click for regions of nodes, the edge of a shell near the click for contact
/// surfaces, else the face under it; for the edges of 2D
/// models the nearest edge of the face. `None` where the mesh has no CAD entities.
pub fn pick(model: &Model, hit: &Hit, target: Target, precision: f32) -> Option<CadEntity> {
    let index = model.cad_index();
    if hit.line {
        // A line part is the mesh of a free CAD edge: the hit names one of its elements. A
        // click near an end of the edge picks the vertex there, as on a solid's edge.
        let edge = line_edge(model, hit)?;
        if target == Target::Nodes
            && let Some(vertex) = index.nearest_vertex(model, edge, hit.point, precision)
        {
            return Some(CadEntity::Vertex(vertex));
        }
        return (!matches!(target, Target::Faces | Target::Surfaces | Target::Lines))
            .then_some(CadEntity::Edge(edge));
    }
    let skin_face = model.skin(hit.part).faces.get(hit.face)?;
    let element = model.mesh.elements()[skin_face.element].id;
    let face = index.face(&model.mesh, (element, skin_face.face as u8 + 1))?;
    let edges = index.face_edges.get(&face).map_or(&[][..], Vec::as_slice);
    let nearest_edge = || {
        (edges.iter())
            .filter_map(|&edge| Some((edge, edge_distance(model, edge, hit.point)?)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
    };
    match target {
        Target::Faces => Some(CadEntity::Face(face)),
        // An edge of a shell near the click, else the face.
        Target::Surfaces => match nearest_edge() {
            Some((edge, d))
                if d <= precision && !model.mesh.cad_faces(&[CadEntity::Edge(edge)]).is_empty() =>
            {
                Some(CadEntity::Edge(edge))
            }
            _ => Some(CadEntity::Face(face)),
        },
        Target::Edges | Target::Lines => nearest_edge().map(|(edge, _)| CadEntity::Edge(edge)),
        Target::Nodes | Target::Point => {
            let vertices: BTreeSet<i32> = (edges.iter())
                .filter_map(|e| index.edge_vertices.get(e))
                .flatten()
                .copied()
                .collect();
            let vertex = (vertices.into_iter())
                .filter_map(|v| {
                    let node = *model.mesh.cad.nodes.get(&CadEntity::Vertex(v))?.first()?;
                    let position = model.render_position(model.mesh.node_index(node)?);
                    Some((v, position.distance(hit.point)))
                })
                .filter(|&(_, d)| d <= precision)
                .min_by(|a, b| a.1.total_cmp(&b.1));
            if let Some((vertex, _)) = vertex {
                return Some(CadEntity::Vertex(vertex));
            }
            match nearest_edge() {
                Some((edge, d)) if d <= precision => Some(CadEntity::Edge(edge)),
                _ => Some(CadEntity::Face(face)),
            }
        }
    }
}

/// The CAD face an element face lies on.
pub fn face_of(model: &Model, face: (ElementId, u8)) -> Option<i32> {
    model.cad_index().face(&model.mesh, face)
}

/// The CAD vertex nearest to a click, whatever its distance: of the edge for a line part,
/// else of the edges bounding the face under the click. `None` where they have none.
pub fn nearest_corner(model: &Model, hit: &Hit) -> Option<i32> {
    let index = model.cad_index();
    let edges: Vec<i32> = if hit.line {
        vec![line_edge(model, hit)?]
    } else {
        let skin_face = model.skin(hit.part).faces.get(hit.face)?;
        let element = model.mesh.elements()[skin_face.element].id;
        let face = index.face(&model.mesh, (element, skin_face.face as u8 + 1))?;
        index.face_edges.get(&face)?.clone()
    };
    let vertices: BTreeSet<i32> = (edges.iter())
        .filter_map(|e| index.edge_vertices.get(e))
        .flatten()
        .copied()
        .collect();
    (vertices.into_iter())
        .filter_map(|v| {
            let node = *model.mesh.cad.nodes.get(&CadEntity::Vertex(v))?.first()?;
            let position = model.render_position(model.mesh.node_index(node)?);
            Some((v, position.distance(hit.point)))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(v, _)| v)
}

/// The CAD edge of a hit line segment: the edge an element of the geometry display shows,
/// else the edge of the mesh the element's segment lies along, by the CAD map.
fn line_edge(model: &Model, hit: &Hit) -> Option<i32> {
    let element = &model.mesh.elements()[*model.skin(hit.part).line_elements.get(hit.face)?];
    if let Some(entity) = model.cad_entity(element.id) {
        return match entity {
            CadEntity::Edge(edge) => Some(edge),
            _ => None,
        };
    }
    // Quadratic lines have their midside node between the ends.
    let (a, b) = (*element.nodes.first()?, *element.nodes.last()?);
    model
        .cad_index()
        .segment_edge
        .get(&[a.min(b), a.max(b)])
        .copied()
}

/// Distance of a point to the mesh segments along a CAD edge, in render coordinates.
fn edge_distance(model: &Model, edge: i32, point: Vec3) -> Option<f32> {
    segment_positions(model, &[edge])
        .map(|[a, b]| {
            let ab = b - a;
            let t =
                ((point - a).dot(ab) / ab.length_squared().max(f32::MIN_POSITIVE)).clamp(0.0, 1.0);
            (a + ab * t).distance(point)
        })
        .min_by(f32::total_cmp)
}

fn segment_positions<'a>(
    model: &'a Model,
    edges: &'a [i32],
) -> impl Iterator<Item = [Vec3; 2]> + 'a {
    let position = |id| Some(model.render_position(model.mesh.node_index(id)?));
    (edges.iter())
        .filter_map(|e| model.mesh.cad.segments.get(e))
        .flatten()
        .filter_map(move |&[a, b]| Some([position(a)?, position(b)?]))
}

/// The CAD edges of faces and the edges themselves.
fn outline(model: &Model, entities: &BTreeSet<CadEntity>) -> Vec<i32> {
    let index = model.cad_index();
    let mut edges = BTreeSet::new();
    for entity in entities {
        match *entity {
            CadEntity::Face(face) => {
                edges.extend(index.face_edges.get(&face).into_iter().flatten().copied());
            }
            CadEntity::Edge(edge) => {
                edges.insert(edge);
            }
            CadEntity::Vertex(_) => {}
        }
    }
    edges.into_iter().collect()
}

/// The hover preview: faces by their outline, edges as lines and vertices as points.
pub fn preview(model: &Model, entities: &BTreeSet<CadEntity>) -> Preview {
    let edges = outline(model, entities);
    Preview {
        lines: segment_positions(model, &edges).collect(),
        points: vertex_nodes(model, entities)
            .filter_map(|n| model.mesh.node_index(n))
            .map(|n| model.render_position(n))
            .collect(),
    }
}

fn vertex_nodes<'a>(
    model: &'a Model,
    entities: &'a BTreeSet<CadEntity>,
) -> impl Iterator<Item = NodeId> + 'a {
    (entities.iter())
        .filter(|e| matches!(e, CadEntity::Vertex(_)))
        .filter_map(|e| model.mesh.cad.nodes.get(e))
        .flatten()
        .copied()
}

/// How a region of CAD entities shows selected: faces shaded, edges as lines, vertices as
/// points.
pub fn highlight(model: &Model, entities: &[CadEntity]) -> Highlight {
    let entities: BTreeSet<CadEntity> = entities.iter().copied().collect();
    let mut highlight = Highlight::default();
    let faces: Vec<CadEntity> = (entities.iter())
        .filter(|e| matches!(e, CadEntity::Face(_)))
        .copied()
        .collect();
    if !faces.is_empty() {
        let on_faces: BTreeSet<(ElementId, u8)> = (faces.iter())
            .filter_map(|e| match e {
                CadEntity::Face(tag) => model.mesh.cad.faces.get(tag),
                _ => None,
            })
            .flatten()
            .copied()
            .collect();
        let elements: BTreeSet<ElementId> = on_faces.iter().map(|(e, _)| *e).collect();
        // Solid element faces show as they are; shell and 2D elements with all their faces.
        highlight.faces = model
            .skin_faces()
            .filter(|f| {
                on_faces.contains(f)
                    || (elements.contains(&f.0)
                        && model
                            .mesh
                            .element(f.0)
                            .is_some_and(|e| e.shape.faces().len() == 1))
            })
            .collect();
    }
    let edges: Vec<i32> = (entities.iter())
        .filter_map(|e| match e {
            CadEntity::Edge(tag) => Some(*tag),
            _ => None,
        })
        .collect();
    highlight.lines = (edges.iter())
        .filter_map(|e| model.mesh.cad.segments.get(e))
        .flatten()
        .copied()
        .collect();
    highlight.nodes = vertex_nodes(model, &entities).collect();
    highlight
}

#[cfg(test)]
mod tests {
    use plx_io::frd::read_frd;
    use plx_model::{
        BoundaryCondition, BoundaryKind, Elastic, FeModel, Load, LoadKind, Material, Region,
        Section, SectionKind, Step,
    };

    use super::*;
    use crate::model::load;
    use crate::selection::{Operation, Picker};
    use crate::setup::{FACE_SOURCES, NODE_SOURCES, RegionDraft};

    fn testdata(name: &str) -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name)
    }

    fn gmsh_available() -> bool {
        match plx_mesher::self_test() {
            Ok(_) => true,
            Err(error) if std::env::var_os("NODALIS_REQUIRE_GMSH").is_none() => {
                eprintln!("Gmsh not available, test skipped: {error}");
                false
            }
            Err(error) => panic!("Gmsh is required: {error}"),
        }
    }

    /// The region a click into the 3D view picks, from `from` along `direction` in model
    /// coordinates.
    fn click(model: &Model, target: Target, from: [f64; 3], direction: Vec3) -> Region {
        let sources = if target == Target::Nodes {
            NODE_SOURCES
        } else {
            FACE_SOURCES
        };
        let mut draft = RegionDraft::new(sources, target);
        let hit = model.pick(model.to_render(from), direction).unwrap();
        draft.click(
            model,
            &Picker::default(),
            Some((&hit, 0.1)),
            Operation::Replace,
        );
        draft.region()
    }

    fn coords(mesh: &FeMesh, nodes: &[NodeId]) -> Vec<[f64; 3]> {
        nodes.iter().map(|&n| mesh.node(n).unwrap()).collect()
    }

    /// The plate with a hole (100 x 40 x 10) fixed at x = 0 and pulled by a pressure on its
    /// end at x = 100, both picked on the geometry: meshed coarse, then the part remeshed
    /// fine. The regions find the end faces on both meshes, and the solver gives about the
    /// same elongation.
    #[test]
    fn selections_on_the_geometry_survive_remeshing() {
        if !gmsh_available() {
            return;
        }
        let mut model = load(
            &testdata("platte_mit_loch.step"),
            plx_model::UnitSystem::Unitless,
        )
        .unwrap()
        .model;
        let mut geometry = model.geometry.clone().unwrap();
        geometry.meshing.max_size = 10.0;
        model.set_mesh(plx_mesher::generate_mesh(&geometry).unwrap().mesh);

        let fixed = click(&model, Target::Nodes, [-50.0, 5.0, 5.0], Vec3::X);
        let pulled = click(&model, Target::Faces, [150.0, 5.0, 5.0], Vec3::NEG_X);
        assert!(
            matches!(&fixed, Region::Geometry(e) if e.len() == 1),
            "{fixed:?}"
        );
        assert!(
            matches!(&pulled, Region::Geometry(e) if e.len() == 1),
            "{pulled:?}"
        );
        // Near an edge of the end the click picks the edge, near a corner the vertex.
        let edge = click(&model, Target::Nodes, [-50.0, 5.0, 9.95], Vec3::X);
        assert!(matches!(&edge, Region::Geometry(e) if matches!(e[..], [CadEntity::Edge(_)])));
        let corner = click(&model, Target::Nodes, [-50.0, 0.05, 9.95], Vec3::X);
        assert_eq!(corner.nodes(&model.mesh).len(), 1, "{corner:?}");
        assert_eq!(
            coords(&model.mesh, &corner.nodes(&model.mesh)),
            [[0.0, 0.0, 10.0]]
        );

        // A point is the vertex nearest to the click, also in the middle of the end face.
        let hit = model
            .pick(model.to_render([-50.0, 2.0, 7.0]), Vec3::X)
            .unwrap();
        let point = Picker::default().pick(&model, &hit, Target::Point, 0.1);
        assert!(
            matches!(&point, crate::selection::Items::Geometry(e) if matches!(e.first(), Some(CadEntity::Vertex(_)))),
            "{point:?}"
        );
        let crate::selection::Items::Nodes(point) = point.resolved(&model.mesh, Target::Point)
        else {
            panic!("a point resolves to nodes");
        };
        let point: Vec<NodeId> = point.into_iter().collect();
        assert_eq!(coords(&model.mesh, &point), [[0.0, 0.0, 10.0]]);

        // The surface angle mode picks CAD faces too, not the nodes it grows over; the end
        // face meets its neighbours at right angles, so it is the only one.
        let hit = model
            .pick(model.to_render([-50.0, 2.0, 7.0]), Vec3::X)
            .unwrap();
        let mut by_angle = Picker::default();
        by_angle.select_by = crate::selection::SelectBy::GeometrySurfaceAngle;
        let items = by_angle.pick(&model, &hit, Target::Nodes, 0.1);
        assert!(
            matches!(&items, crate::selection::Items::Geometry(e) if e.len() == 1 && matches!(e.first(), Some(CadEntity::Face(_)))),
            "{items:?}"
        );

        // Mixed with picked nodes the geometry turns into nodes; undo goes back to it.
        let mut draft = RegionDraft::from_region(&fixed, NODE_SOURCES, Target::Nodes, &model.mesh);
        let extra = model.mesh.node_ids()[0];
        let items = crate::selection::Items::Nodes(std::collections::BTreeSet::from([extra]));
        draft.take(&model.mesh, items, Operation::Add);
        let mut expected = fixed.nodes(&model.mesh);
        expected.push(extra);
        expected.sort_unstable();
        expected.dedup();
        assert_eq!(draft.region(), Region::Nodes(expected));
        let mut draft = RegionDraft::from_region(&fixed, NODE_SOURCES, Target::Nodes, &model.mesh);
        let hit = model
            .pick(model.to_render([150.0, 5.0, 5.0]), Vec3::NEG_X)
            .unwrap();
        draft.click(
            &model,
            &Picker::default(),
            Some((&hit, 0.1)),
            Operation::Add,
        );
        assert!(matches!(draft.region(), Region::Geometry(e) if e.len() == 2));
        draft.action(&model, crate::selection::PickerAction::Undo);
        assert_eq!(draft.region(), fixed);

        model.fe = FeModel {
            materials: vec![Material {
                id: crate::test_ids::named("Steel"),
                name: "Steel".into(),
                density: None,
                elastic: Some(Elastic {
                    young: 210000.0,
                    poisson: 0.3,
                }),
                conductivity: None,
                specific_heat: None,
                expansion: None,
                plastic: None,
                creep: None,
            }],
            sections: vec![Section {
                name: "Section-1".into(),
                material: Some(crate::test_ids::named("Steel")),
                region: Region::Parts(vec!["SOLID-1".into()]),
                thickness: 1.0,
                kind: SectionKind::Solid,
            }],
            steps: vec![Step::new_static("Step-1")],
            ..FeModel::default()
        };
        let step = &mut model.fe.steps[0];
        step.boundary_conditions.push(BoundaryCondition {
            id: crate::test_ids::named("Fix"),
            distribution: None,
            name: "Fix".into(),
            active: true,
            visible: true,
            region: fixed.clone(),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        step.loads.push(Load {
            id: crate::test_ids::named("Pull"),
            distribution: None,
            name: "Pull".into(),
            active: true,
            visible: true,
            region: pulled.clone(),
            kind: LoadKind::Pressure(-100.0),
            amplitude: None,
            factor_amplitude: None,
        });

        let check = |model: &Model| {
            let mesh = &model.mesh;
            let nodes = fixed.nodes(mesh);
            assert!(fixed.missing_reference(mesh).is_none());
            assert!(coords(mesh, &nodes).iter().all(|c| c[0].abs() < 1e-9));
            let faces = pulled.faces(mesh);
            let on_end = Region::Faces(faces.clone()).nodes(mesh);
            assert!(
                coords(mesh, &on_end)
                    .iter()
                    .all(|c| (c[0] - 100.0).abs() < 1e-9)
            );
            (nodes.len(), faces.len())
        };
        let coarse = check(&model);
        let coarse_elongation = elongation(&model);

        // Finer, as after a first run, with the part's own meshing parameters.
        geometry.mesh_items.push(plx_model::MeshSetupItem {
            name: "Meshing_Parameters-1".into(),
            kind: plx_model::MeshSetupKind::MeshingParameters {
                parts: vec!["SOLID-1".into()],
                parameters: plx_model::MeshingParameters {
                    max_size: 3.0,
                    ..geometry.meshing
                },
            },
        });
        let part = plx_mesher::generate_part_mesh(&geometry, "SOLID-1").unwrap();
        let mesh = plx_mesher::merge_part(&model.mesh, part.mesh);
        model.set_mesh(mesh);
        let fine = check(&model);
        assert!(
            fine.0 > 2 * coarse.0 && fine.1 > 2 * coarse.1,
            "{coarse:?} -> {fine:?}"
        );
        let fine_elongation = elongation(&model);

        if let (Some(coarse), Some(fine)) = (coarse_elongation, fine_elongation) {
            // 100 MPa over 100 mm of steel stretch it by 0.048 mm; the hole adds a little.
            let plain = 100.0 / 210000.0 * 100.0;
            for value in [coarse, fine] {
                assert!(value > plain && value < 1.3 * plain, "{value} vs. {plain}");
            }
            assert!((fine - coarse).abs() < 0.03 * fine, "{coarse} -> {fine}");
        }
    }

    /// A cantilever of two CAD lines along x, 0..60 and 60..100, meshed as beams: fixed at
    /// x = 0, the lines tied at x = 60 and pulled down at x = 100, all picked on the lines
    /// in the geometry mode. Meshed coarse, then both lines remeshed fine: the picks are CAD
    /// vertices and edges, which find their nodes on both meshes, and the solver gives the
    /// beam theory's tip deflection both times.
    #[test]
    fn selections_on_lines_survive_remeshing() {
        if !gmsh_available() {
            return;
        }
        let dir = std::env::temp_dir().join(format!("nodalis-lines-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("kragarm.brep");
        plx_mesher::gmsh::with_gmsh(|gmsh| {
            let a = gmsh.add_point([0.0, 0.0, 0.0])?;
            let b = gmsh.add_point([60.0, 0.0, 0.0])?;
            let c = gmsh.add_point([100.0, 0.0, 0.0])?;
            gmsh.add_line(a, b)?;
            gmsh.add_line(b, c)?;
            gmsh.write(&file)
        })
        .unwrap();
        let mut model = load(&file, plx_model::UnitSystem::Unitless).unwrap().model;
        std::fs::remove_dir_all(&dir).unwrap();
        let mut geometry = model.geometry.clone().unwrap();
        geometry.meshing.max_size = 20.0;
        model.set_mesh(plx_mesher::generate_mesh(&geometry).unwrap().mesh);
        let lines: Vec<String> = model.mesh.parts.iter().map(|p| p.name.clone()).collect();
        assert_eq!(lines.len(), 2, "{lines:?}");

        // Looking down onto the lines at x, within 1 of them.
        let hit_at = |model: &Model, x: f64| {
            let origin = model.to_render([x, 0.5, 50.0]);
            model.pick_line(origin, Vec3::NEG_Z, |_| 1.0).unwrap().1
        };
        let click = |model: &Model, target: Target, x: f64| {
            let sources = if target == Target::Nodes {
                NODE_SOURCES
            } else {
                FACE_SOURCES
            };
            let mut draft = RegionDraft::new(sources, target);
            let hit = hit_at(model, x);
            draft.click(
                model,
                &Picker::default(),
                Some((&hit, 1.0)),
                Operation::Replace,
            );
            draft.region()
        };
        let vertex = |region: &Region| matches!(region, Region::Geometry(e) if matches!(e[..], [CadEntity::Vertex(_)]));
        let fixed = click(&model, Target::Nodes, 0.5);
        let tied = click(&model, Target::Nodes, 60.5);
        let pulled = click(&model, Target::Nodes, 99.5);
        for region in [&fixed, &tied, &pulled] {
            assert!(vertex(region), "{region:?}");
        }
        // Away from its ends a click picks the whole line.
        let line = click(&model, Target::Nodes, 30.0);
        assert!(
            matches!(&line, Region::Geometry(e) if matches!(e[..], [CadEntity::Edge(_)])),
            "{line:?}"
        );
        // A point, e.g. of a remote displacement, is the nearest end.
        let point = Picker::default().pick(&model, &hit_at(&model, 80.0), Target::Point, 1.0);
        let crate::selection::Items::Geometry(point) = point else {
            panic!("{point:?}");
        };
        assert_eq!(Region::Geometry(point.into_iter().collect()), pulled);

        model.fe = FeModel {
            materials: vec![Material {
                id: crate::test_ids::named("Steel"),
                name: "Steel".into(),
                density: None,
                elastic: Some(Elastic {
                    young: 210000.0,
                    poisson: 0.3,
                }),
                conductivity: None,
                specific_heat: None,
                expansion: None,
                plastic: None,
                creep: None,
            }],
            sections: vec![Section {
                name: "Section-1".into(),
                material: Some(crate::test_ids::named("Steel")),
                region: line.clone(),
                thickness: 1.0,
                kind: SectionKind::Beam(plx_model::BeamSection {
                    cross_section: Some(crate::test_ids::named("Rect")),
                    ..plx_model::BeamSection::DEFAULT
                }),
            }],
            cross_sections: vec![plx_model::CrossSection {
                id: crate::test_ids::named("Rect"),
                name: "Rect".into(),
                profile: plx_model::BeamProfile::Rect { a: 10.0, b: 10.0 },
            }],
            node_ties: vec![plx_model::NodeTie {
                region: tied.clone(),
                ..plx_model::NodeTie::new("Node_Tie-1")
            }],
            steps: vec![Step::new_static("Step-1")],
            ..FeModel::default()
        };
        // The other line by its part.
        let other = (lines.iter())
            .find(|name| {
                let part = Region::Parts(vec![(*name).clone()]);
                part.elements(&model.mesh) != line.elements(&model.mesh)
            })
            .unwrap();
        let mut second = model.fe.sections[0].clone();
        second.name = "Section-2".into();
        second.region = Region::Parts(vec![other.clone()]);
        model.fe.sections.push(second);
        let step = &mut model.fe.steps[0];
        step.boundary_conditions.push(BoundaryCondition {
            id: crate::test_ids::named("Fix"),
            distribution: None,
            name: "Fix".into(),
            active: true,
            visible: true,
            region: fixed.clone(),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        step.loads.push(Load {
            id: crate::test_ids::named("Pull"),
            distribution: None,
            name: "Pull".into(),
            active: true,
            visible: true,
            region: pulled.clone(),
            kind: LoadKind::Force {
                value: [0.0, 0.0, -100.0],
                each_node: false,
            },
            amplitude: None,
            factor_amplitude: None,
        });

        let check = |model: &Model| {
            let mesh = &model.mesh;
            for (region, x, count) in [(&fixed, 0.0, 1), (&tied, 60.0, 2), (&pulled, 100.0, 1)] {
                assert!(region.missing_reference(mesh).is_none(), "{region:?}");
                let nodes = region.nodes(mesh);
                assert_eq!(nodes.len(), count, "{region:?}");
                assert!(coords(mesh, &nodes).iter().all(|c| (c[0] - x).abs() < 1e-9));
            }
            let elements = line.elements(mesh);
            assert!(!elements.is_empty());
            assert_eq!(
                elements,
                Region::Parts(vec![lines.iter().find(|n| *n != other).unwrap().clone()])
                    .elements(mesh)
            );
            elements.len()
        };
        let coarse = check(&model);
        let coarse_deflection = tip_deflection(&model);

        for part in &lines {
            geometry.mesh_items.push(plx_model::MeshSetupItem {
                name: format!("Meshing_Parameters-{part}"),
                kind: plx_model::MeshSetupKind::MeshingParameters {
                    parts: vec![part.clone()],
                    parameters: plx_model::MeshingParameters {
                        max_size: 4.0,
                        ..geometry.meshing
                    },
                },
            });
        }
        for part in &lines {
            let fine = plx_mesher::generate_part_mesh(&geometry, part).unwrap();
            let mesh = plx_mesher::merge_part(&model.mesh, fine.mesh);
            model.set_mesh(mesh);
        }
        let fine = check(&model);
        assert!(fine > 3 * coarse, "{coarse} -> {fine}");
        let fine_deflection = tip_deflection(&model);

        if let (Some(coarse), Some(fine)) = (coarse_deflection, fine_deflection) {
            // F L^3 / (3 E I) with I = 10^4 / 12.
            let theory = 100.0 * 100f32.powi(3) / (3.0 * 210000.0 * 1e4 / 12.0);
            for value in [coarse, fine] {
                assert!(
                    (value - theory).abs() < 0.03 * theory,
                    "{value} vs. {theory}"
                );
            }
        }
    }

    /// The largest downward displacement CalculiX computes, if ccx is installed.
    fn tip_deflection(model: &Model) -> Option<f32> {
        let inp = plx_io::inp::write_inp(&model.mesh, &model.fe, "Kragarm").unwrap();
        let dir = std::env::temp_dir().join(format!(
            "nodalis-lines-remesh-{}-{}",
            std::process::id(),
            model.mesh.element_count()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("kragarm.inp"), inp).unwrap();
        plxs_session::run_input_file(&dir, "kragarm").unwrap_or_else(|e| panic!("{e}"));
        let results = read_frd(&dir.join("kragarm.frd")).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        let disp = results.increments.last().unwrap().field("DISP").unwrap();
        let values = &disp.component("U3").unwrap().values;
        Some(-values.iter().copied().fold(f32::MAX, f32::min))
    }

    /// The largest displacement along x CalculiX computes, if ccx is installed.
    fn elongation(model: &Model) -> Option<f32> {
        let inp = plx_io::inp::write_inp(&model.mesh, &model.fe, "Platte").unwrap();
        let dir = std::env::temp_dir().join(format!(
            "nodalis-remesh-{}-{}",
            std::process::id(),
            model.mesh.element_count()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("platte.inp"), inp).unwrap();
        plxs_session::run_input_file(&dir, "platte").unwrap_or_else(|e| panic!("{e}"));
        let results = read_frd(&dir.join("platte.frd")).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        let disp = results.increments.last().unwrap().field("DISP").unwrap();
        let values = &disp.component("U1").unwrap().values;
        Some(values.iter().copied().fold(f32::MIN, f32::max))
    }
}
