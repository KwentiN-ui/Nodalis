//! How the window adapts to the room it has, after chapter 6 of the GUI concept. From wide
//! to narrow:
//!
//! - from 1500 points on, everything as designed;
//! - below that the ribbon folds its groups into menus, from the right, until the row fits
//!   (in [`crate::ribbon`]);
//! - below 1200 points the properties panel lies over the right edge of the 3D view instead
//!   of narrowing it;
//! - below 900 points the ribbon shows icons only, the tree folds away behind a button of
//!   the app bar and the output pane folds to its tab row.
//!
//! Below 700 points of height the ribbon folds to its tab row and the output pane to its
//! tabs. Folds the user undoes stay undone while the size stays in the same range.

use egui::Vec2;

/// Below this width the properties panel lies over the 3D view.
pub const OVERLAY_WIDTH: f32 = 1200.0;
/// Below this width the ribbon shows icons only and the side panels fold.
pub const NARROW_WIDTH: f32 = 900.0;
/// Below this height the ribbon and the output pane fold.
pub const LOW_HEIGHT: f32 = 700.0;

/// The layout for a window of `size` points.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Fit {
    /// The properties panel lies over the 3D view.
    pub overlay: bool,
    /// Ribbon tiles show their icon only, the label in the tooltip.
    pub icons_only: bool,
    /// The tree is folded away.
    pub fold_tree: bool,
    /// The ribbon shows only its tabs.
    pub fold_ribbon: bool,
}

pub fn fit(size: Vec2) -> Fit {
    let narrow = size.x < NARROW_WIDTH;
    let low = size.y < LOW_HEIGHT;
    Fit {
        overlay: size.x < OVERLAY_WIDTH,
        icons_only: narrow,
        fold_tree: narrow,
        fold_ribbon: low,
    }
}

/// The fit of the window `ctx` draws in.
pub fn current(ctx: &egui::Context) -> Fit {
    fit(ctx.content_rect().size())
}

/// A fold the layout asks for, which the user can undo: the user's choice holds until the
/// layout asks for something else.
#[derive(Clone, Copy, Debug, Default)]
pub struct Fold {
    /// What the layout asked for when the user last chose.
    asked: Option<bool>,
    /// The user's choice.
    chosen: bool,
}

impl Fold {
    /// Whether it is folded now, when the layout asks for `asked`.
    pub fn folded(&mut self, asked: bool) -> bool {
        match self.asked {
            Some(when) if when == asked => self.chosen,
            _ => {
                self.asked = None;
                asked
            }
        }
    }

    /// The user folds or unfolds it.
    pub fn set(&mut self, asked: bool, folded: bool) {
        self.asked = Some(asked);
        self.chosen = folded;
    }
}

/// Where the fold `what` of a window is kept.
fn fold_id(ctx: &egui::Context, what: &str) -> egui::Id {
    egui::Id::new(("fold", what, ctx.viewport_id()))
}

/// Whether `what` of the window `ctx` draws in is folded, when the layout asks for `asked`.
pub fn folded(ctx: &egui::Context, what: &str, asked: bool) -> bool {
    let id = fold_id(ctx, what);
    ctx.data_mut(|d| d.get_temp_mut_or_default::<Fold>(id).folded(asked))
}

/// The user folds `what` or unfolds it.
pub fn set_folded(ctx: &egui::Context, what: &str, asked: bool, folded: bool) {
    let id = fold_id(ctx, what);
    ctx.data_mut(|d| d.get_temp_mut_or_default::<Fold>(id).set(asked, folded));
}

/// Whether the tree of the window `ctx` draws in is folded away.
pub fn tree_folded(ctx: &egui::Context) -> bool {
    folded(ctx, "tree", current(ctx).fold_tree)
}

/// The user folds the tree away or shows it again.
pub fn fold_tree(ctx: &egui::Context, folded: bool) {
    set_folded(ctx, "tree", current(ctx).fold_tree, folded);
}

/// Whether the output pane of the window `ctx` draws in is hidden. It is, until one of its
/// tabs in the status bar is clicked, a run starts or a problem is asked for; whatever the
/// size of the window.
pub fn output_folded(ctx: &egui::Context) -> bool {
    folded(ctx, "output", true)
}

/// The user hides the output pane or shows it.
pub fn fold_output(ctx: &egui::Context, folded: bool) {
    set_folded(ctx, "output", true, folded);
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::vec2;

    #[test]
    fn the_layout_follows_the_size_of_the_window() {
        assert_eq!(fit(vec2(1600.0, 1000.0)), Fit::default());
        let middle = fit(vec2(1100.0, 900.0));
        assert!(middle.overlay && !middle.icons_only && !middle.fold_tree);
        let narrow = fit(vec2(800.0, 900.0));
        assert!(narrow.overlay && narrow.icons_only && narrow.fold_tree);
        assert!(!narrow.fold_ribbon);
        let low = fit(vec2(1600.0, 650.0));
        assert!(low.fold_ribbon && !low.overlay);
    }

    #[test]
    fn a_fold_the_user_undid_holds_until_the_size_changes() {
        let mut fold = Fold::default();
        assert!(fold.folded(true));
        fold.set(true, false);
        assert!(!fold.folded(true));
        // Wide again: the layout no longer folds, and the choice is forgotten.
        assert!(!fold.folded(false));
        assert!(fold.folded(true));
    }
}

/// A row of widgets that wraps when it is too wide. Inside a `Grid` cell a plain
/// `horizontal_wrapped` does not report the height of the wrapped lines, so the next row
/// would be drawn over them. The vertical wrapper makes the cell take that height.
pub fn wrapped_row<R>(
    ui: &mut egui::Ui,
    add: impl FnOnce(&mut egui::Ui) -> R,
) -> egui::InnerResponse<R> {
    ui.vertical(|ui| ui.horizontal_wrapped(add).inner)
}
