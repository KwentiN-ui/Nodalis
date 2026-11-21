//! The CAD tools of the Geometry workspace: Fillet, Chamfer, Push/Pull, Defeature, Split by
//! Plane and Split by Offset Circle, set up in the editor of the CAD Operation block, each a tool
//! of its own under Modify. Each becomes an operation of the
//! Geometry tree ([`plx_model::Geometry::operations`]), a step of a pipeline from the shapes
//! as imported or linked, as a CAD Operation block before the analysis would be: it can be
//! changed or deleted later, and acts anew on new shapes a link brings. The selections on the
//! shapes follow the changes. Edit > Undo takes a change back.

use super::*;
use crate::cad_editor::{CadEditorResult, CadOperationEditor};
use plx_model::{CadOperation, CadOperationKind, Geometry, GeometryOperation};

impl AnalysisWindow {
    /// Opens the editor of a CAD tool on the geometry of the analysis.
    pub(super) fn open_cad_tool(&mut self, kind: CadOperationKind) {
        let w = &mut self.workbench;
        let Some(model) = &w.model else {
            return;
        };
        // The operations of the Geometry tree so far make what it acts on.
        let Some(geometry) = model.geometry.clone().map(Geometry::without_operations) else {
            return;
        };
        let geometry = if geometry.signatures.is_empty() {
            plx_mesher::with_signatures(geometry.clone()).unwrap_or(geometry)
        } else {
            geometry
        };
        // A split cuts the parts selected in the Geometry tree, or all when none is.
        // Suppressed parts are never cut.
        let chosen = if kind == CadOperationKind::SplitPlane {
            let shown = w.geometry.as_ref();
            (w.selected_geometry_parts().into_iter())
                .filter(|&i| {
                    !shown
                        .and_then(|v| v.parts.get(i))
                        .is_some_and(|p| p.suppressed)
                })
                .collect()
        } else {
            Vec::new()
        };
        // The YZ plane through the middle of the geometry, or of the parts selected; a
        // width or size of a few millimetres.
        let value = if kind == CadOperationKind::SplitPlane {
            let shown = w.geometry.as_ref();
            let bounds: Vec<(f64, f64)> = (chosen.iter())
                .filter_map(|&i| {
                    let view = shown?;
                    let (min, max) = view.parts.get(i)?.bounds?;
                    Some((view.model_point(min).x, view.model_point(max).x))
                })
                .collect();
            let (lo, hi) = if bounds.is_empty() {
                (geometry.signatures.iter())
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), s| {
                        (lo.min(s.min[0]), hi.max(s.max[0]))
                    })
            } else {
                (bounds.iter()).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), b| {
                    (lo.min(b.0), hi.max(b.1))
                })
            };
            if lo.is_finite() && hi.is_finite() {
                0.5 * (lo + hi)
            } else {
                0.0
            }
        } else {
            3e-3
        };
        let parts = (chosen.iter())
            .filter_map(|&i| w.geometry.as_ref()?.parts.get(i))
            .map(|p| p.id)
            .filter(|&id| geometry.part(id).is_some())
            .collect();
        let operation = CadOperation {
            kind,
            value,
            parts,
            ..CadOperation::default()
        };
        let mut editor = CadOperationEditor::new(
            0,
            kind.label(),
            geometry,
            &operation,
            model.fe.properties.units,
            false,
        );
        editor.round_offset();
        let viewport = Viewport::new(w.viewport.render_state());
        w.cad_tool = Some((editor, viewport, None));
    }

    /// Opens the editor of the operation at `index` of the Geometry tree, on the shapes the
    /// operations before it make.
    pub(super) fn edit_geometry_operation(&mut self, index: usize) {
        let w = &mut self.workbench;
        let Some(model) = &w.model else {
            return;
        };
        let Some(geometry) = &model.geometry else {
            return;
        };
        let Some(step) = geometry.operations.get(index) else {
            return;
        };
        let input = crate::geometry_operations::input_of(geometry, index);
        let editor = CadOperationEditor::new(
            0,
            &step.name,
            input,
            &step.operation,
            model.fe.properties.units,
            false,
        );
        let viewport = Viewport::new(w.viewport.render_state());
        w.cad_tool = Some((editor, viewport, Some(index)));
    }

    /// Deletes the operation at `index` of the Geometry tree: the others act anew on the
    /// input.
    pub(super) fn delete_geometry_operation(&mut self, index: usize) {
        let Some(geometry) = (self.workbench.model.as_ref()).and_then(|m| m.geometry.as_ref())
        else {
            return;
        };
        let Some(name) = geometry.operations.get(index).map(|o| o.name.clone()) else {
            return;
        };
        let mut operations = geometry.operations.clone();
        operations.remove(index);
        let (made, notes) = crate::geometry_operations::rerun(geometry, operations);
        self.take_operations(made, notes);
        self.workbench.tree.selected = None;
        let line = format!("{name} deleted; mesh the geometry again.");
        self.workbench.output.push(line);
    }

    /// Takes the shapes the operations of the Geometry tree made, as an undo step.
    fn take_operations(&mut self, made: Geometry, notes: Vec<String>) {
        if let Some(model) = &mut self.workbench.model {
            crate::undo::before_shape_change(model);
        }
        self.take_geometry(Some(made), true);
        self.workbench.output.extend(notes);
    }
}

impl Workbench {
    /// The parts selected in the Geometry tree, those of selected compound parts included,
    /// as indices of the parts of the geometry view, in ascending order.
    fn selected_geometry_parts(&self) -> Vec<usize> {
        let mut parts = self.tree.selected_parts(TreeView::Geometry);
        for item in self.tree.selected_items(TreeView::Geometry) {
            if let TreeItem::Compound(c) = item
                && let Some((_, members)) = self.tree.compounds.get(c)
            {
                parts.extend(members.iter().copied());
            }
        }
        parts.into_iter().collect()
    }
}

impl AnalysisWindow {
    /// Shows the editor of a CAD tool; OK adds the operation to the Geometry tree, or
    /// changes the one edited, and the operations act anew.
    pub(super) fn cad_tool_window(&mut self, ctx: &egui::Context) {
        match self.workbench.geometry_operation_request.take() {
            Some((index, true)) => self.delete_geometry_operation(index),
            Some((index, false)) => self.edit_geometry_operation(index),
            None => {}
        }
        let Some((editor, viewport, edited)) = &mut self.workbench.cad_tool else {
            return;
        };
        let edited = *edited;
        let (operation, made) = match editor.show(ctx, viewport) {
            CadEditorResult::Open => return,
            CadEditorResult::Cancel => {
                self.workbench.cad_tool = None;
                return;
            }
            CadEditorResult::Ok(operation, made) => (operation, made),
        };
        let input = editor.input().clone();
        self.workbench.cad_tool = None;
        let Some(geometry) = (self.workbench.model.as_ref()).and_then(|m| m.geometry.clone())
        else {
            return;
        };
        let label = operation.kind.label();
        if let Some(index) = edited {
            let mut operations = geometry.operations.clone();
            let Some(step) = operations.get_mut(index) else {
                return;
            };
            step.operation = operation;
            let name = step.name.clone();
            let (made, notes) = crate::geometry_operations::rerun(&geometry, operations);
            self.take_operations(made, notes);
            let line = format!("{name} changed; mesh the geometry again.");
            self.workbench.output.push(line);
            return;
        }
        let made = match made {
            Some(made) => Ok(*made),
            None => crate::geometry_operations::apply(&input, &operation),
        };
        let made = match made {
            Ok(made) => made,
            Err(why) => {
                let line = format!("{label} failed: {why}");
                self.workbench.output.push(line);
                return;
            }
        };
        if made.brep == input.brep {
            let line = format!("{label}: nothing to change in the geometry.");
            self.workbench.output.push(line);
            return;
        }
        // The operation goes after the others, on the shapes they made.
        let name = geometry.operation_name(label);
        let mut operations = geometry.operations.clone();
        operations.push(GeometryOperation {
            name: name.clone(),
            operation: operation.clone(),
        });
        let start = match geometry.input.clone() {
            Some(start) => *start,
            None => geometry.clone().without_operations(),
        };
        let made = Geometry {
            operations,
            input: Some(Box::new(start)),
            ..made.without_operations()
        };
        self.take_operations(made, Vec::new());
        let line = format!(
            "{name} ({}) added to the Geometry tree; mesh the geometry again.",
            operation.describe_picks()
        );
        self.workbench.output.push(line);
    }
}
