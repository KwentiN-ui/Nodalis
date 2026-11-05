//! What the 3D view showed of results bound to a project, saved with them so that they open
//! as they were left. Plain data: the program turns it into its own types.

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SavedView {
    pub camera: Option<SavedCamera>,
    /// Element edges drawn.
    pub mesh_edges: Option<bool>,
    pub deformation: Option<SavedDeformation>,
    /// Factor of the user-defined deformation.
    pub user_scale: Option<f64>,
    pub show_undeformed: Option<bool>,
    /// Colour levels of the legend; also in `color_scale`, kept for older projects.
    pub levels: Option<u32>,
    /// Colour spectrum, colour levels and limits of the legend.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color_scale: Option<plx_model::ColorScale>,
    /// Parts hidden, by name.
    pub hidden_parts: Vec<String>,
    /// What the query tool left in the view.
    pub annotations: Vec<SavedAnnotation>,
    /// Plots of result paths, hot spots and history tables in the view.
    pub path_plots: Vec<SavedPathPlot>,
    /// Arrows or contours, and their options.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vector_display: Option<crate::display::VectorDisplay>,
}

/// The camera of the results view.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SavedCamera {
    pub target: [f32; 3],
    pub distance: f32,
    /// Quaternion x, y, z, w.
    pub rotation: [f32; 4],
    pub scene_radius: f32,
}

/// How the deformed shape is scaled.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum SavedDeformation {
    Undeformed,
    TrueScale,
    Automatic(f32),
    UserDefined,
}

/// A query annotation; nodes by id, elements and parts by index.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SavedAnnotation {
    Nodes(Vec<u32>),
    Elements(Vec<usize>),
    /// The segments along the picked edges.
    Edge {
        segments: Vec<[u32; 2]>,
    },
    /// The faces of the picked surfaces by their corner nodes.
    Surface {
        faces: Vec<Vec<u32>>,
    },
    Parts(Vec<usize>),
    Distance([u32; 2]),
    Angle([u32; 3]),
    Circle([u32; 3]),
    BoundingBox,
}

/// The plot of a result path and where it is drawn.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SavedPathPlot {
    pub path: String,
    /// Top left corner relative to that of the view, in points, once moved.
    pub offset: Option<[f32; 2]>,
    pub size: [f32; 2],
    pub on_path: bool,
    pub height: f32,
    pub title: String,
    pub x_label: String,
    pub y_label: String,
    pub y_range: Option<[f64; 2]>,
    /// The plot of a hot spot of that name rather than of a path.
    #[serde(default)]
    pub hot_spot: bool,
    /// The plot of a history output table rather than of a path, with its values.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub table: Option<SavedTablePlot>,
}

/// Curves plotted from a history output table: x values and the y values of each named
/// curve, row by row. Missing values are NaN.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SavedTablePlot {
    pub x_label: String,
    pub x: Vec<f64>,
    pub curves: Vec<(String, Vec<f64>)>,
}
