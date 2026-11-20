//! The operations of the Geometry tree, such as Split by Plane: a pipeline of CAD
//! operations from the shapes as imported or linked ([`Geometry::input`]) to the shapes the
//! analysis meshes, as CAD Operation blocks before the analysis in the project would be.
//! They act anew on new shapes a link brings, and when one of them is changed or deleted.

use plx_model::{Geometry, GeometryOperation};

/// The shapes `input` with `operations` applied in order, holding the operations and the
/// input; `suppressed` are the parts the operations leave alone. An operation that fails on
/// the shapes is skipped, with a line on why. Without operations, the input as it is.
pub fn run(
    input: Geometry,
    mut operations: Vec<GeometryOperation>,
    suppressed: &[plx_model::PartRef],
) -> (Geometry, Vec<String>) {
    let input = input.without_operations();
    if operations.is_empty() {
        return (input, Vec::new());
    }
    let mut notes = Vec::new();
    let mut shapes = Geometry {
        suppressed: suppressed.to_vec(),
        ..input.clone()
    };
    for step in &mut operations {
        if shapes.signatures.is_empty() {
            shapes = plx_mesher::with_signatures(shapes.clone()).unwrap_or(shapes);
        }
        step.operation.follow(&shapes);
        match apply(&shapes, &step.operation) {
            Ok(made) => shapes = made,
            Err(why) => notes.push(format!("{} skipped: {why}", step.name)),
        }
    }
    let shapes = Geometry {
        operations,
        input: Some(Box::new(input)),
        ..shapes.without_operations()
    };
    (shapes, notes)
}

/// The operation applied to the shapes, or why it cannot be.
pub fn apply(shapes: &Geometry, operation: &plx_model::CadOperation) -> Result<Geometry, String> {
    if let Some(problem) = operation.problem() {
        return Err(problem);
    }
    let entities = operation.resolve(shapes)?;
    plx_mesher::apply_operation(shapes, operation, &entities).map_err(|e| e.to_string())
}

/// New shapes `source`, as a link brings them, through the operations of the geometry
/// `own` of the analysis taking them: what the analysis is to take, see
/// [`crate::links::take_geometry`]. Lines on operations that failed go to `notes`.
pub fn through(
    own: Option<&Geometry>,
    source: Option<Geometry>,
    notes: &mut Vec<String>,
) -> Option<Geometry> {
    let source = source?.without_operations();
    let Some(own) = own.filter(|g| !g.operations.is_empty()) else {
        return Some(source);
    };
    // The same shapes again: what the operations made of them stays.
    if (own.input.as_deref()).is_some_and(|input| same_shapes(input, &source)) {
        return Some(Geometry {
            source: source.source,
            refinement: source.refinement,
            ids: own.ids.clone(),
            ..own.clone()
        });
    }
    let (made, failed) = run(source, own.operations.clone(), &own.suppressed);
    notes.extend(failed);
    Some(made)
}

/// The geometry with its operations replaced by `operations`, run anew from its input; the
/// lines say which failed. Without input, as when there were no operations, the shapes are
/// the input.
pub fn rerun(geometry: &Geometry, operations: Vec<GeometryOperation>) -> (Geometry, Vec<String>) {
    let input = match &geometry.input {
        Some(input) => (**input).clone(),
        None => geometry.clone().without_operations(),
    };
    let (made, notes) = run(input, operations, &geometry.suppressed);
    (made, notes)
}

/// The shapes the operation at `index` acts on: those the operations before it make of the
/// input, with the signatures of their entities.
pub fn input_of(geometry: &Geometry, index: usize) -> Geometry {
    let before = geometry.operations[..index.min(geometry.operations.len())].to_vec();
    let (shapes, _) = rerun(geometry, before);
    let shapes = Geometry {
        suppressed: geometry.suppressed.clone(),
        ..shapes.without_operations()
    };
    if shapes.signatures.is_empty() {
        plx_mesher::with_signatures(shapes.clone()).unwrap_or(shapes)
    } else {
        shapes
    }
}

fn same_shapes(a: &Geometry, b: &Geometry) -> bool {
    a.brep == b.brep && a.part_names == b.part_names && a.compounds == b.compounds
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_model::{CadOperation, CadOperationKind, UnitSystem};

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

    /// Two blocks, with the first one's name and the middle of its x extent.
    fn blocks() -> (Geometry, String, f64) {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/zwei_bloecke.step");
        let loaded = crate::model::load(&path, UnitSystem::Unitless).unwrap();
        let geometry = loaded.model.geometry.clone().unwrap();
        let geometry = plx_mesher::with_signatures(geometry).unwrap();
        let view = loaded.geometry_view.unwrap();
        let first = &view.parts[0];
        let (min, max) = first.bounds.unwrap();
        let (min, max) = (view.model_point(min), view.model_point(max));
        (geometry, first.name.clone(), 0.5 * (min.x + max.x))
    }

    /// The second block's name and the middle of its x extent.
    fn second_block() -> (String, f64) {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/zwei_bloecke.step");
        let view = crate::model::load(&path, UnitSystem::Unitless)
            .unwrap()
            .geometry_view
            .unwrap();
        let second = &view.parts[1];
        let (min, max) = second.bounds.unwrap();
        let (min, max) = (view.model_point(min), view.model_point(max));
        (second.name.clone(), 0.5 * (min.x + max.x))
    }

    fn split(name: &str, geometry: &Geometry, part: &str, x: f64) -> GeometryOperation {
        GeometryOperation {
            name: name.into(),
            operation: CadOperation {
                kind: CadOperationKind::SplitPlane,
                value: x,
                // A part the geometry does not have is one that went.
                parts: vec![geometry.part_id(part).unwrap_or_default()],
                ..CadOperation::default()
            },
        }
    }

    fn part_count(geometry: &Geometry) -> usize {
        plx_mesher::tessellate(geometry).unwrap().solids
    }

    #[test]
    fn operations_act_on_new_shapes_and_go_again() {
        if !gmsh_available() {
            return;
        }
        let (input, first, x) = blocks();
        assert_eq!(part_count(&input), 2);
        let (made, notes) = run(
            input.clone(),
            vec![split("Split-1", &input, &first, x)],
            &[],
        );
        assert!(notes.is_empty(), "{notes:?}");
        assert_eq!(part_count(&made), 3);
        assert_eq!(made.operations.len(), 1);
        assert_eq!(made.input.as_deref().map(|i| &i.brep), Some(&input.brep));
        // A link bringing the same shapes keeps what was made.
        let again = through(Some(&made), Some(input.clone()), &mut Vec::new()).unwrap();
        assert_eq!(again.brep, made.brep);
        assert_eq!(again.operations, made.operations);
        // Deleting the operation brings back the shapes as they came.
        let (back, _) = rerun(&made, Vec::new());
        assert_eq!(back.brep, input.brep);
        assert!(back.operations.is_empty() && back.input.is_none());
        // The input of the operation is the shapes before it.
        assert_eq!(input_of(&made, 0).brep, input.brep);
        // What another analysis takes is the shapes alone.
        assert!(made.clone().without_operations().input.is_none());
    }

    #[test]
    fn an_operation_that_fails_is_skipped() {
        if !gmsh_available() {
            return;
        }
        let (input, _, x) = blocks();
        let (made, notes) = run(
            input.clone(),
            vec![split("Split-1", &input, "NO SUCH PART", x)],
            &[],
        );
        assert_eq!(notes.len(), 1);
        assert!(notes[0].starts_with("Split-1 skipped"), "{notes:?}");
        assert_eq!(made.brep, input.brep);
        assert_eq!(made.operations.len(), 1);
    }

    #[test]
    fn a_geometry_link_brings_shapes_through_the_operations() {
        if !gmsh_available() {
            return;
        }
        let (input, first, x) = blocks();
        let (made, _) = run(
            input.clone(),
            vec![split("Split-1", &input, &first, x)],
            &[],
        );
        let mut own = Some(made.clone());
        let mut fe = plx_model::FeModel::default();
        // New shapes: the same blocks under another source name.
        let source = Geometry {
            source: "new.step".into(),
            ..input
        };
        let copy = crate::links::MeshCopy::shapes(source);
        crate::links::take_objects(
            &mut own,
            &plx_mesh::FeMesh::default(),
            &Default::default(),
            &mut fe,
            copy,
        );
        let own = own.unwrap();
        assert_eq!(own.operations.len(), 1);
        assert_eq!(part_count(&own), 3);
        assert_eq!(own.source, "new.step");
        // A new version of the shapes, the second block cut in the CAD program: the
        // operation cuts the first block of it again.
        let (second, middle) = second_block();
        let (version, _) = run(
            input_of(&own, 0),
            vec![split("CAD", &input_of(&own, 0), &second, middle)],
            &[],
        );
        let mut own = Some(own);
        let copy = crate::links::MeshCopy::shapes(version.clone().without_operations());
        crate::links::take_objects(
            &mut own,
            &plx_mesh::FeMesh::default(),
            &Default::default(),
            &mut fe,
            copy,
        );
        let own = own.unwrap();
        assert_eq!(part_count(&own), 4);
        assert_eq!(own.operations.len(), 1);
        assert_eq!(own.input.as_deref().map(|i| &i.brep), Some(&version.brep));
    }
}
