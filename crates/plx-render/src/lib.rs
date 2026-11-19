//! wgpu-Renderer für FE-Netze und Ergebnisse.

mod beam;
mod camera;
pub mod contour;
mod fe;
pub mod glyphs;
mod mesh;
mod renderer;
mod section;
mod shell;

pub use beam::{BeamShape, beam_mesh};
pub use camera::{Camera, StandardView};
pub use fe::{MESH_EDGE_OPACITY, mesh_wireframe, part_color, part_render_mesh, wireframe_edges};
pub use mesh::{Directions, NodeBlend, RenderMesh, Vertex};
pub use renderer::{
    COLOR_FORMAT, ContourPalette, DisplayOptions, LicDisplay, LicSource, ViewportRenderer,
};
pub use section::{
    ClipPlane, SectionCells, SectionValues, cap_mesh, clip_plane, lighten, section_mesh,
    section_values,
};
pub use shell::{ShellThickness, thick_shell_mesh};
pub use wgpu;
