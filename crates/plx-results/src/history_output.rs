//! History outputs the user derives from the results:
//! values of field components at nodes over all increments, an equation of other history
//! outputs, the size of elements or element faces on the deformed mesh, or the values of a
//! hot spot, optionally beside the position of a moving load.
//!
//! A history output gives a set of fields with components; every component has one entry
//! per node, element or face, with one value per increment.

use plx_mesh::{ElementId, FeMesh};
use plx_model::Region;
use serde::{Deserialize, Serialize};

use crate::equation::Equation;
use crate::hot_spot::{HotSpotReport, IncrementValues};
use crate::{Increment, field_output};

/// Field and component names of computed history outputs.
pub const EQUATION_FIELD: &str = "EQUATION";
pub const SIZE_FIELD: &str = "ELEMENT_SIZE";
pub const VOLUME: &str = "VOLUME";
pub const AREA: &str = "AREA";
/// Field of a hot spot's values: a component per toe node with the hot spot stress, the
/// stress at each read-out point and the position of a moving load as its entries, so that
/// one table holds all of them.
pub const HOT_SPOT_FIELD: &str = "HOT_SPOT";
/// Component of the largest hot spot stress of all toe nodes, for a hot spot with several.
pub const HOT_SPOT_MAX: &str = "MAX";
/// Entries of the components.
pub const POSITION: &str = "POSITION";
pub const HOT_SPOT_STRESS: &str = "S_HS";
pub const MAX_NODE: &str = "NODE";

/// Name of the component of a toe node.
pub fn node_component(node: plx_mesh::NodeId) -> String {
    format!("NODE_{node}")
}

/// Name of the entry of the stress at the read-out point `index` (from 0).
pub fn readout_entry(index: usize) -> String {
    format!("S_{}", index + 1)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SizeKind {
    /// Volume of solid elements.
    Volume,
    /// Area of element faces.
    Area,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum HistoryOutputKind {
    /// The components of a field at the nodes of a region.
    FromField {
        region: Region,
        field: String,
        components: Vec<String>,
    },
    /// An equation of components of other history outputs, entry by entry.
    FromEquation { equation: String, unit: String },
    /// Volume of the elements or area of the faces of a region, on the deformed mesh.
    ElementSize { region: Region, kind: SizeKind },
    /// The values of a hot spot of the results by name; with a moving load of the FE model
    /// also the load's position in each step, to plot the stress over it.
    FromHotSpot {
        hot_spot: String,
        moving_load: Option<String>,
    },
}

/// What history outputs are computed from besides the increments: the history computed so
/// far, which an equation may use, the evaluated hot spots and the moving loads of the FE
/// model.
#[derive(Clone, Copy, Debug, Default)]
pub struct Sources<'a> {
    pub sets: &'a [HistorySet],
    pub hot_spots: &'a [HotSpotReport],
    pub moving_loads: &'a [plx_model::LoadStations],
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HistoryOutput {
    pub name: String,
    pub kind: HistoryOutputKind,
}

impl HistoryOutput {
    pub fn unit(&self) -> Option<&str> {
        match &self.kind {
            HistoryOutputKind::FromEquation { unit, .. } => Some(unit),
            _ => None,
        }
    }
}

/// One column: a node, element or face, with a value per increment.
#[derive(Clone, Debug, PartialEq)]
pub struct HistoryEntry {
    pub name: String,
    pub values: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct HistoryComponent {
    pub name: String,
    pub entries: Vec<HistoryEntry>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct HistoryField {
    pub name: String,
    pub components: Vec<HistoryComponent>,
}

impl HistoryField {
    pub fn component(&self, name: &str) -> Option<&HistoryComponent> {
        self.components.iter().find(|c| c.name == name)
    }
}

/// The computed data of a history output.
#[derive(Clone, Debug, PartialEq)]
pub struct HistorySet {
    pub name: String,
    /// Step, increment and time (or frequency, buckling factor) of every row.
    pub rows: Vec<(u32, u32, f64)>,
    pub fields: Vec<HistoryField>,
}

impl HistorySet {
    pub fn field(&self, name: &str) -> Option<&HistoryField> {
        self.fields.iter().find(|f| f.name == name)
    }
}

fn rows(increments: &[Increment]) -> Vec<(u32, u32, f64)> {
    increments
        .iter()
        .map(|i| (i.step, i.increment, i.value))
        .collect()
}

/// Computes a history output.
pub fn compute(
    output: &HistoryOutput,
    increments: &[Increment],
    mesh: &FeMesh,
    sources: Sources,
) -> Result<HistorySet, String> {
    let fields = match &output.kind {
        HistoryOutputKind::FromField {
            region,
            field,
            components,
        } => vec![from_field(region, field, components, increments, mesh)?],
        HistoryOutputKind::FromEquation { equation, .. } => {
            vec![from_equation(&output.name, equation, sources.sets)?]
        }
        HistoryOutputKind::ElementSize { region, kind } => {
            vec![element_size(region, *kind, increments, mesh)?]
        }
        HistoryOutputKind::FromHotSpot {
            hot_spot,
            moving_load,
        } => {
            let report = (sources.hot_spots.iter().find(|r| r.name == *hot_spot))
                .ok_or_else(|| format!("The hot spot {hot_spot} does not exist."))?;
            let position = match moving_load {
                Some(load) => {
                    let stations = (sources.moving_loads.iter().find(|s| s.load == *load))
                        .ok_or_else(|| {
                            format!("The moving load {load} is in no active step of the FE model.")
                        })?;
                    Some(moving_load_position(stations, increments))
                }
                None => None,
            };
            vec![from_hot_spot(report, increments, position)?]
        }
    };
    Ok(HistorySet {
        name: output.name.clone(),
        rows: rows(increments),
        fields,
    })
}

fn from_field(
    region: &Region,
    field: &str,
    components: &[String],
    increments: &[Increment],
    mesh: &FeMesh,
) -> Result<HistoryField, String> {
    let nodes = region.nodes(mesh);
    if nodes.is_empty() {
        return Err("The region contains no nodes.".into());
    }
    if components.is_empty() {
        return Err("Please select at least one component.".into());
    }
    let indices: Vec<Option<usize>> = nodes.iter().map(|&n| mesh.node_index(n)).collect();
    let mut result = Vec::new();
    for name in components {
        if !increments
            .iter()
            .any(|i| i.field(field).and_then(|f| f.component(name)).is_some())
        {
            return Err(format!("The result {field}.{name} exists in no increment."));
        }
        let entries = nodes
            .iter()
            .zip(&indices)
            .map(|(id, index)| HistoryEntry {
                name: id.to_string(),
                values: increments
                    .iter()
                    .map(|inc| {
                        let values = inc.field(field).and_then(|f| f.component(name));
                        match (values, index) {
                            (Some(c), Some(i)) => c.values.get(*i).map_or(f64::NAN, |&v| v as f64),
                            _ => f64::NAN,
                        }
                    })
                    .collect(),
            })
            .collect();
        result.push(HistoryComponent {
            name: name.clone(),
            entries,
        });
    }
    Ok(HistoryField {
        name: field.to_string(),
        components: result,
    })
}

fn from_equation(own: &str, equation: &str, sets: &[HistorySet]) -> Result<HistoryField, String> {
    let parsed = Equation::parse(equation)?;
    let mut sources: Vec<&HistoryComponent> = Vec::new();
    for variable in parsed.variables() {
        let parts: Vec<&str> = variable.splitn(3, '.').collect();
        let [set, field, component] = parts[..] else {
            return Err(format!(
                "{variable}: History outputs are given as Name.Field.Component, \
                 e.g. History-1.STRESS.MISES."
            ));
        };
        if set == own {
            return Err(format!("{own} cannot use itself."));
        }
        let set = (sets.iter().find(|s| s.name == set))
            .ok_or_else(|| format!("The history output {set} does not exist."))?;
        let component = (set.field(field).and_then(|f| f.component(component)))
            .ok_or_else(|| format!("{variable} does not exist."))?;
        sources.push(component);
    }
    let (columns, rows) = match sources.first() {
        Some(first) => (
            first.entries.len(),
            first.entries.first().map_or(0, |e| e.values.len()),
        ),
        None => (1, 1),
    };
    for source in &sources {
        if source.entries.len() != columns {
            return Err(
                "All components of the equation must have the same number of entries (columns)."
                    .into(),
            );
        }
        if source.entries.iter().any(|e| e.values.len() != rows) {
            return Err(
                "All components of the equation must have the same number of increments (rows)."
                    .into(),
            );
        }
    }
    // Nodalis keeps the entry names when all components share them, else numbers them.
    let names: Vec<String> = match sources.first() {
        Some(first)
            if sources.iter().all(|s| {
                s.entries
                    .iter()
                    .zip(&first.entries)
                    .all(|(a, b)| a.name == b.name)
            }) =>
        {
            first.entries.iter().map(|e| e.name.clone()).collect()
        }
        _ => (1..=columns).map(|i| i.to_string()).collect(),
    };
    let mut args = vec![0.0; sources.len()];
    let entries = names
        .into_iter()
        .enumerate()
        .map(|(column, name)| HistoryEntry {
            name,
            values: (0..rows)
                .map(|row| {
                    for (arg, source) in args.iter_mut().zip(&sources) {
                        *arg = source.entries[column].values[row];
                    }
                    parsed.evaluate(&args)
                })
                .collect(),
        })
        .collect();
    Ok(HistoryField {
        name: EQUATION_FIELD.into(),
        components: vec![HistoryComponent {
            name: field_output::VALUE.into(),
            entries,
        }],
    })
}

/// The values of a hot spot per increment: for each toe node the hot spot stress and the
/// stress at its read-out points, after the position of the moving load if one is given;
/// increments without stresses have none.
fn from_hot_spot(
    report: &HotSpotReport,
    increments: &[Increment],
    position: Option<Vec<f64>>,
) -> Result<HistoryField, String> {
    if report.paths.is_empty() {
        return Err(match report.warnings.first() {
            Some(warning) => format!("The hot spot has no path: {warning}"),
            None => "The hot spot has no path.".into(),
        });
    }
    if report.increments.is_empty() {
        return Err("The results contain no stresses (S).".into());
    }
    // The evaluated increments are those with stresses, in file order.
    let mut evaluated = report.increments.iter().peekable();
    let rows: Vec<Option<&IncrementValues>> = increments
        .iter()
        .map(|increment| {
            evaluated.next_if(|v| {
                v.step == increment.step
                    && v.increment == increment.increment
                    && v.value == increment.value
            })
        })
        .collect();
    let column = |value: &dyn Fn(&IncrementValues) -> Option<f64>| -> Vec<f64> {
        (rows.iter())
            .map(|row| row.and_then(value).unwrap_or(f64::NAN))
            .collect()
    };
    let leading = || {
        (position.iter())
            .map(|values| HistoryEntry {
                name: POSITION.into(),
                values: values.clone(),
            })
            .collect::<Vec<_>>()
    };
    let mut components = Vec::new();
    if report.paths.len() > 1 {
        let mut entries = leading();
        entries.push(HistoryEntry {
            name: HOT_SPOT_STRESS.into(),
            values: column(&|r| Some(r.maximum()?.hot_spot)),
        });
        entries.push(HistoryEntry {
            name: MAX_NODE.into(),
            values: column(&|r| Some(f64::from(r.maximum()?.node))),
        });
        components.push(HistoryComponent {
            name: HOT_SPOT_MAX.into(),
            entries,
        });
    }
    for (path, toe) in report.paths.iter().enumerate() {
        let value = |path: usize| move |r: &IncrementValues| r.values.get(path).cloned();
        let mut entries = leading();
        entries.push(HistoryEntry {
            name: HOT_SPOT_STRESS.into(),
            values: column(&|r| Some(value(path)(r)?.hot_spot)),
        });
        for index in 0..report.distances.len() {
            entries.push(HistoryEntry {
                name: readout_entry(index),
                values: column(&|r| value(path)(r)?.readings.get(index).copied()),
            });
        }
        components.push(HistoryComponent {
            name: node_component(toe.node),
            entries,
        });
    }
    Ok(HistoryField {
        name: HOT_SPOT_FIELD.into(),
        components,
    })
}

/// The position of a moving load in the step of every increment; steps without the load
/// have none.
fn moving_load_position(stations: &plx_model::LoadStations, increments: &[Increment]) -> Vec<f64> {
    increments
        .iter()
        .map(|increment| {
            (stations.steps.iter())
                .find(|(step, _)| *step == increment.step)
                .map_or(f64::NAN, |(_, station)| *station)
        })
        .collect()
}

/// Area of a polygon given as a closed ring of points, fanned around its centroid; also the
/// signed volume of the cone from `apex` over it.
fn ring_area_and_volume(ring: &[glam::DVec3], apex: glam::DVec3) -> (f64, f64) {
    let center = ring.iter().copied().sum::<glam::DVec3>() / ring.len() as f64;
    let mut area = 0.0;
    let mut volume = 0.0;
    for (i, &a) in ring.iter().enumerate() {
        let b = ring[(i + 1) % ring.len()];
        let cross = (a - center).cross(b - center);
        area += cross.length() / 2.0;
        volume += (center - apex).dot((a - apex).cross(b - apex)) / 6.0;
    }
    (area, volume)
}

/// Corners and midside nodes of a face as a ring: c0, m0, c1, m1, ...
fn face_ring(
    element: &plx_mesh::Element,
    face: &plx_mesh::FaceTopology,
    position: &dyn Fn(u32) -> Option<glam::DVec3>,
) -> Option<Vec<glam::DVec3>> {
    let quadratic = element.shape.is_quadratic();
    let mut ring = Vec::new();
    for (i, &corner) in face.corners.iter().enumerate() {
        ring.push(position(*element.nodes.get(corner)?)?);
        if quadratic && let Some(&mid) = face.mids.get(i) {
            ring.push(position(*element.nodes.get(mid)?)?);
        }
    }
    Some(ring)
}

fn element_size(
    region: &Region,
    kind: SizeKind,
    increments: &[Increment],
    mesh: &FeMesh,
) -> Result<HistoryField, String> {
    // Entries: whole elements for volumes, faces for areas.
    let items: Vec<(ElementId, Option<u8>)> = match kind {
        SizeKind::Volume => region
            .elements(mesh)
            .into_iter()
            .filter(|&e| {
                mesh.element(e)
                    .is_some_and(|e| e.shape.family() == plx_mesh::ElementFamily::Solid)
            })
            .map(|e| (e, None))
            .collect(),
        SizeKind::Area => {
            let faces = region.faces(mesh);
            if faces.is_empty() {
                // Shell elements are their own face.
                region
                    .elements(mesh)
                    .into_iter()
                    .filter(|&e| {
                        mesh.element(e)
                            .is_some_and(|e| e.shape.family() == plx_mesh::ElementFamily::Surface)
                    })
                    .map(|e| (e, Some(1)))
                    .collect()
            } else {
                faces.into_iter().map(|(e, f)| (e, Some(f))).collect()
            }
        }
    };
    if items.is_empty() {
        return Err(match kind {
            SizeKind::Volume => "The region contains no solid elements.".into(),
            SizeKind::Area => "The region contains no element faces.".into(),
        });
    }
    let coords = mesh.coords();
    let mut entries: Vec<HistoryEntry> = items
        .iter()
        .map(|(element, face)| HistoryEntry {
            name: match face {
                Some(face) if kind == SizeKind::Area => format!("{element}-S{face}"),
                _ => element.to_string(),
            },
            values: Vec::with_capacity(increments.len()),
        })
        .collect();
    for increment in increments {
        let displacements = increment.displacements();
        let position = |id: u32| {
            let index = mesh.node_index(id)?;
            let mut p = glam::DVec3::from(coords[index]);
            if let Some(d) = displacements.as_ref().and_then(|d| d.get(index)) {
                p += glam::DVec3::new(d[0] as f64, d[1] as f64, d[2] as f64);
            }
            Some(p)
        };
        for ((element, face), entry) in items.iter().zip(&mut entries) {
            let value = mesh.element(*element).and_then(|element| {
                let faces = element.shape.faces();
                match face {
                    Some(face) => {
                        let topology = faces.get(usize::from(*face).checked_sub(1)?)?;
                        let ring = face_ring(element, topology, &position)?;
                        Some(ring_area_and_volume(&ring, glam::DVec3::ZERO).0)
                    }
                    None => {
                        let rings: Vec<Vec<glam::DVec3>> = (faces.iter())
                            .map(|f| face_ring(element, f, &position))
                            .collect::<Option<_>>()?;
                        let apex = rings.iter().flatten().copied().sum::<glam::DVec3>()
                            / rings.iter().map(Vec::len).sum::<usize>() as f64;
                        let volume: f64 = (rings.iter())
                            .map(|ring| ring_area_and_volume(ring, apex).1)
                            .sum();
                        Some(volume.abs())
                    }
                }
            });
            entry.values.push(value.unwrap_or(f64::NAN));
        }
    }
    Ok(HistoryField {
        name: SIZE_FIELD.into(),
        components: vec![HistoryComponent {
            name: match kind {
                SizeKind::Volume => VOLUME,
                SizeKind::Area => AREA,
            }
            .into(),
            entries,
        }],
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{AnalysisKind, Component, Field};
    use plx_mesh::{Element, ElementShape, Part};

    /// A unit cube C3D8 whose top moves up by `top` in the second increment.
    fn cube() -> (FeMesh, Vec<Increment>) {
        let mut mesh = FeMesh::default();
        let corners = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ];
        for (i, c) in corners.iter().enumerate() {
            mesh.set_node(i as u32 + 1, *c);
        }
        mesh.add_element(Element {
            id: 1,
            type_name: "C3D8".into(),
            shape: ElementShape::Hex8,
            nodes: (1..=8).collect(),
        })
        .unwrap();
        mesh.parts.push(Part {
            bodies: Vec::new(),
            name: "CUBE".into(),
            elements: vec![1],
        });
        let disp = |top: f32| {
            let column = |k: usize, name: &str| Component {
                name: name.into(),
                values: corners
                    .iter()
                    .map(|c| if k == 2 && c[2] > 0.5 { top } else { 0.0 })
                    .collect(),
                derived: false,
                out_of_plane: false,
            };
            Field::new(
                "DISP",
                vec![column(0, "U1"), column(1, "U2"), column(2, "U3")],
            )
        };
        let increments = [0.0, 1.0]
            .iter()
            .enumerate()
            .map(|(i, &top)| Increment {
                nodal_diameter: None,
                step: 1,
                increment: i as u32 + 1,
                kind: AnalysisKind::Static,
                value: i as f64 + 1.0,
                fields: vec![disp(top)],
            })
            .collect();
        (mesh, increments)
    }

    #[test]
    fn field_values_at_nodes_over_increments() {
        let (mesh, increments) = cube();
        let output = HistoryOutput {
            name: "History-1".into(),
            kind: HistoryOutputKind::FromField {
                region: Region::Nodes(vec![7, 1]),
                field: "DISP".into(),
                components: vec!["U3".into()],
            },
        };
        let set = compute(&output, &increments, &mesh, Sources::default()).unwrap();
        let u3 = set.field("DISP").unwrap().component("U3").unwrap();
        let names: Vec<&str> = u3.entries.iter().map(|e| e.name.as_str()).collect();
        assert_eq!(names, ["1", "7"]);
        assert_eq!(u3.entries[1].values, [0.0, 1.0]);
        assert_eq!(set.rows, [(1, 1, 1.0), (1, 2, 2.0)]);
    }

    #[test]
    fn element_volume_and_face_area_follow_the_deformation() {
        let (mesh, increments) = cube();
        let volume = HistoryOutput {
            name: "Size-1".into(),
            kind: HistoryOutputKind::ElementSize {
                region: Region::Parts(vec!["CUBE".into()]),
                kind: SizeKind::Volume,
            },
        };
        let set = compute(&volume, &increments, &mesh, Sources::default()).unwrap();
        let values = &set.field(SIZE_FIELD).unwrap().components[0].entries[0].values;
        assert!((values[0] - 1.0).abs() < 1e-12 && (values[1] - 2.0).abs() < 1e-12);
        // Face S3 (nodes 1, 5, 6, 2) is the side y = 0, stretched to height 2.
        let area = HistoryOutput {
            name: "Size-2".into(),
            kind: HistoryOutputKind::ElementSize {
                region: Region::Faces(vec![(1, 3)]),
                kind: SizeKind::Area,
            },
        };
        let set = compute(&area, &increments, &mesh, Sources::default()).unwrap();
        let entry = &set.field(SIZE_FIELD).unwrap().components[0].entries[0];
        assert_eq!(entry.name, "1-S3");
        assert!((entry.values[0] - 1.0).abs() < 1e-12 && (entry.values[1] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn equation_combines_history_outputs_entry_by_entry() {
        let (mesh, increments) = cube();
        let first = HistoryOutput {
            name: "History-1".into(),
            kind: HistoryOutputKind::FromField {
                region: Region::Nodes(vec![5, 6]),
                field: "DISP".into(),
                components: vec!["U3".into()],
            },
        };
        let sets = vec![compute(&first, &increments, &mesh, Sources::default()).unwrap()];
        let equation = HistoryOutput {
            name: "Equation-1".into(),
            kind: HistoryOutputKind::FromEquation {
                equation: "=[History-1.DISP.U3] * 10 + 1".into(),
                unit: "/".into(),
            },
        };
        let set = compute(
            &equation,
            &increments,
            &mesh,
            Sources {
                sets: &sets,
                ..Default::default()
            },
        )
        .unwrap();
        let value = set
            .field(EQUATION_FIELD)
            .unwrap()
            .component("VALUE")
            .unwrap();
        assert_eq!(value.entries[0].name, "5");
        assert_eq!(value.entries[1].values, [1.0, 11.0]);
        for wrong in ["=History-2.DISP.U3", "=DISP.U3", "=[History-1.DISP.U1]"] {
            let output = HistoryOutput {
                name: "Equation-1".into(),
                kind: HistoryOutputKind::FromEquation {
                    equation: wrong.into(),
                    unit: "/".into(),
                },
            };
            assert!(
                compute(
                    &output,
                    &increments,
                    &mesh,
                    Sources {
                        sets: &sets,
                        ..Default::default()
                    }
                )
                .is_err(),
                "{wrong}"
            );
        }
    }

    #[test]
    fn hot_spot_values_beside_the_moving_load_position() {
        use crate::hot_spot::{HotSpotComponent, HotSpotPath, HotSpotValue, IncrementValues};
        let (mesh, mut increments) = cube();
        // Three steps; the first has no stresses and so no hot spot values.
        (increments[1].step, increments[1].increment) = (2, 1);
        increments.push(Increment {
            step: 3,
            increment: 1,
            value: 3.0,
            ..increments[1].clone()
        });
        let path = |node| HotSpotPath {
            node,
            index: 0,
            position: [0.0; 3],
            direction: [1.0, 0.0, 0.0],
            points: Vec::new(),
        };
        let values = |step: u32, value: f64, hot: [f64; 2]| IncrementValues {
            step,
            increment: 1,
            value,
            values: vec![
                HotSpotValue {
                    node: 5,
                    readings: vec![hot[0] - 1.0, hot[0] - 2.0],
                    hot_spot: hot[0],
                },
                HotSpotValue {
                    node: 6,
                    readings: vec![hot[1] - 1.0, hot[1] - 2.0],
                    hot_spot: hot[1],
                },
            ],
        };
        let report = HotSpotReport {
            name: "Hot_Spot-1".into(),
            component: HotSpotComponent::Perpendicular,
            method: String::new(),
            distances: vec![4.0, 10.0],
            paths: vec![path(5), path(6)],
            increments: vec![values(2, 2.0, [10.0, 30.0]), values(3, 3.0, [40.0, 20.0])],
            warnings: Vec::new(),
        };
        let stations = plx_model::LoadStations {
            load: "Moving-1".into(),
            steps: vec![(2, 100.0), (3, 200.0)],
        };
        let output = HistoryOutput {
            name: "Hot_Spot_History".into(),
            kind: HistoryOutputKind::FromHotSpot {
                hot_spot: "Hot_Spot-1".into(),
                moving_load: Some("Moving-1".into()),
            },
        };
        let sources = Sources {
            hot_spots: std::slice::from_ref(&report),
            moving_loads: std::slice::from_ref(&stations),
            ..Default::default()
        };
        let set = compute(&output, &increments, &mesh, sources).unwrap();
        let hot = set.field(HOT_SPOT_FIELD).unwrap();
        let values = |component: &str, entry: &str| {
            let component = hot.component(component).unwrap();
            let entry = component.entries.iter().find(|e| e.name == entry).unwrap();
            entry.values.clone()
        };
        let names: Vec<&str> = hot.components.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["MAX", "NODE_5", "NODE_6"]);
        let entries: Vec<&str> = (hot.components[1].entries.iter())
            .map(|e| e.name.as_str())
            .collect();
        assert_eq!(entries, ["POSITION", "S_HS", "S_1", "S_2"]);
        assert!(values("NODE_5", "S_HS")[0].is_nan());
        assert_eq!(values("NODE_5", "S_HS")[1..], [10.0, 40.0]);
        assert_eq!(values("NODE_6", "S_HS")[1..], [30.0, 20.0]);
        assert_eq!(values("NODE_6", "S_2")[1..], [28.0, 18.0]);
        assert_eq!(values("MAX", "S_HS")[1..], [30.0, 40.0]);
        assert_eq!(values("MAX", "NODE")[1..], [6.0, 5.0]);
        let position = values("NODE_6", "POSITION");
        assert!(position[0].is_nan());
        assert_eq!(position[1..], [100.0, 200.0]);
        assert_eq!(values("MAX", "POSITION")[1..], [100.0, 200.0]);

        // Without a moving load the stresses stand alone.
        let mut plain = output.clone();
        plain.kind = HistoryOutputKind::FromHotSpot {
            hot_spot: "Hot_Spot-1".into(),
            moving_load: None,
        };
        let set = compute(&plain, &increments, &mesh, sources).unwrap();
        assert_eq!(set.fields[0].components[1].entries[0].name, "S_HS");

        // A hot spot or moving load that is gone is an error.
        let mut missing = output.clone();
        missing.kind = HistoryOutputKind::FromHotSpot {
            hot_spot: "Hot_Spot-1".into(),
            moving_load: Some("Moving-2".into()),
        };
        assert!(compute(&missing, &increments, &mesh, sources).is_err());
        assert!(compute(&output, &increments, &mesh, Sources::default()).is_err());
    }
}
