//! The CAD tools of the Geometry workspace: Defeature, Split by Plane and Split around Hole
//! applied to the geometry of the analysis right away, in the editor of the CAD Operation
//! block. Unlike the block they keep no history of their own: the analysis takes the changed
//! geometry as if it had been imported so, and the selections on it follow the changed
//! shapes. Edit > Undo takes the change back.

use super::*;
use crate::cad_editor::{CadEditorResult, CadOperationEditor};
use plx_model::{CadOperation, CadOperationKind};

impl AnalysisWindow {
    /// Opens the editor of a CAD tool on the geometry of the analysis.
    pub(super) fn open_cad_tool(&mut self, kind: CadOperationKind) {
        let w = &mut self.workbench;
        // A link would bring its geometry back over the changed one.
        let linked = (w.incoming.paths.iter()).find(|(path, _)| {
            path.first()
                .is_some_and(|p| p == crate::links::GEOMETRY || p == crate::links::MESH)
        });
        if let Some((_, source)) = linked {
            let line = format!(
                "The geometry comes from {source}: change it there, or put a CAD Operation \
                 block between the two in the project."
            );
            w.output.push(line);
            return;
        }
        let Some(model) = &w.model else {
            return;
        };
        let Some(geometry) = model.geometry.clone() else {
            return;
        };
        let geometry = if geometry.signatures.is_empty() {
            plx_mesher::with_signatures(geometry.clone()).unwrap_or(geometry)
        } else {
            geometry
        };
        // A split cuts the parts selected in the Geometry tree, or all when none is.
        let chosen = if kind == CadOperationKind::SplitPlane {
            w.selected_geometry_parts()
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
            .filter_map(|&i| Some(w.geometry.as_ref()?.parts.get(i)?.name.clone()))
            .collect();
        let operation = CadOperation {
            kind,
            value,
            parts,
            ..CadOperation::default()
        };
        let editor = CadOperationEditor::new(
            0,
            kind.label(),
            geometry,
            &operation,
            model.fe.properties.units,
            false,
        );
        let viewport = Viewport::new(w.viewport.render_state());
        w.cad_tool = Some((editor, viewport));
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
    /// Shows the editor of a CAD tool; OK applies the operation to the geometry.
    pub(super) fn cad_tool_window(&mut self, ctx: &egui::Context) {
        let Some((editor, viewport)) = &mut self.workbench.cad_tool else {
            return;
        };
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
        let label = operation.kind.label();
        let made = match made {
            Some(made) => Ok(*made),
            None => (operation.problem().map_or(Ok(()), Err))
                .and_then(|()| operation.resolve(&input))
                .and_then(|entities| {
                    plx_mesher::apply_operation(&input, &operation, &entities)
                        .map_err(|e| e.to_string())
                }),
        };
        let geometry = match made {
            Ok(geometry) => geometry,
            Err(why) => {
                let line = format!("{label} failed: {why}");
                self.workbench.output.push(line);
                return;
            }
        };
        if geometry.brep == input.brep {
            let line = format!("{label}: nothing to change in the geometry.");
            self.workbench.output.push(line);
            return;
        }
        // The selections follow the shapes by their signatures, which the input has.
        if let Some(model) = &mut self.workbench.model {
            crate::undo::before_shape_change(model);
            model.geometry = Some(input);
        }
        self.take_geometry(Some(geometry), true);
        let line = format!(
            "{label} applied to the geometry ({}); mesh it again.",
            operation.describe_picks()
        );
        self.workbench.output.push(line);
    }
}
