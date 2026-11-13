//! The state of the cells of an analysis, as the blocks of Ansys Workbench's project
//! schematic show it: whether geometry, mesh, setup, solution and results are there and up to
//! date. A cell is out of date when the fingerprint of its inputs differs from the one it
//! was made from, or when a cell it is made from is out of date; Solve then makes only those
//! cells again.

use std::collections::BTreeMap;

use egui::{Color32, Painter, Rect, Shape, Stroke, pos2};
use plx_io::project::BoundResults;
use plx_mesh::FeMesh;
use plx_model::{FeModel, Geometry, fingerprint};

use crate::tree_icons::{self, TreeIcon};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CellState {
    /// Not there yet: nothing imported, meshed, set up or solved.
    Missing,
    UpToDate,
    /// Made from inputs that changed since: Solve makes it again.
    OutOfDate,
    Running,
    /// The solver reported errors; results may still be there.
    Failed,
    /// Not part of this analysis, such as the geometry of a mesh read from an input file.
    NotUsed,
}

impl CellState {
    pub fn text(self) -> &'static str {
        match self {
            CellState::Missing => "not there yet",
            CellState::UpToDate => "up to date",
            CellState::OutOfDate => "out of date: Solve makes it again",
            CellState::Running => "running",
            CellState::Failed => "The solver reported errors",
            CellState::NotUsed => "not used: the mesh comes from an input file",
        }
    }
}

/// The cells of an analysis, top to bottom as the project schematic shows them.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AnalysisStatus {
    pub geometry: CellState,
    pub mesh: CellState,
    pub setup: CellState,
    pub solution: CellState,
    pub results: CellState,
    /// Parts of the geometry whose mesh is out of date, to be meshed again.
    pub stale_parts: Vec<String>,
    /// Mesh blocks the mesh is taken from whose mesh is out of date: the project window
    /// makes them anew before the analysis is solved.
    pub stale_sources: Vec<String>,
    /// Analyses whose results it takes that are out of date: the project window solves them
    /// again first.
    pub stale_results: Vec<String>,
    /// What the results of each step are computed from now, see
    /// [`crate::solve_fingerprint::step_inputs`]; empty without results. Kept here so that
    /// what is drawn in every frame does not fingerprint the whole model again.
    pub step_inputs: Vec<u64>,
    /// Whether the solution and results states are those found before the last change, while
    /// the fingerprints of the changed model are made in the background.
    pub pending: bool,
}

impl AnalysisStatus {
    /// Whether there is something to solve: a mesh, or geometry to mesh, and a step.
    pub fn solvable(&self) -> bool {
        self.setup != CellState::Missing
            && (self.mesh != CellState::Missing || self.geometry == CellState::UpToDate)
    }

    /// Whether Solve has anything to do.
    pub fn needs_solve(&self) -> bool {
        matches!(self.mesh, CellState::OutOfDate)
            || matches!(self.solution, CellState::Missing | CellState::OutOfDate)
    }
}

/// What an analysis takes from other blocks of the project that is not ready yet, as the
/// project window finds it.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Upstream {
    /// Analyses whose results it needs, e.g. as global model, that have none yet.
    pub missing_results: Vec<String>,
    /// Mesh blocks it takes its mesh from, directly or through other analyses, whose mesh
    /// is out of date: the mesh it holds is not the one it is meant to be solved with.
    pub stale_meshes: Vec<String>,
    /// Analyses whose results it takes, e.g. as global model, that are out of date: their
    /// results no longer fit their model, so the analysis' solution is out of date too.
    pub stale_results: Vec<String>,
}

impl Upstream {
    /// Whether the project has to make or solve something before the analysis is solved.
    pub fn is_ready(&self) -> bool {
        self.missing_results.is_empty()
            && self.stale_meshes.is_empty()
            && self.stale_results.is_empty()
    }
}

/// The state of each cell of an analysis with this data; `running` while the solver solves it.
/// `upstream` is what it takes from other blocks that is not ready. With `previous`, the state
/// found before, the fingerprints of the solution are made in the background if they take a
/// while, and its solution and results states stand meanwhile ([`AnalysisStatus::pending`]).
#[allow(clippy::too_many_arguments)]
pub fn analysis_status(
    geometry: Option<&Geometry>,
    mesh: &FeMesh,
    fe: &FeModel,
    mesh_inputs: &BTreeMap<String, u64>,
    results: Option<&BoundResults>,
    upstream: &Upstream,
    running: bool,
    previous: Option<&AnalysisStatus>,
) -> AnalysisStatus {
    let space = fe.properties.space;
    let meshed = mesh.element_count() > 0;
    let geometry_state = match geometry {
        Some(_) => CellState::UpToDate,
        None if meshed => CellState::NotUsed,
        None => CellState::Missing,
    };
    // Only parts meshed from the geometry know their inputs; parts read from input files
    // have none and stay as they are.
    let stale_parts: Vec<String> = match geometry {
        Some(geometry) => (mesh.parts.iter())
            .filter_map(|part| {
                // A compound part is out of date with any of its bodies.
                let bodies: Vec<&str> = if part.bodies.is_empty() {
                    vec![part.name.as_str()]
                } else {
                    part.bodies.iter().map(|b| b.name.as_str()).collect()
                };
                // Suppressed parts keep the mesh they have.
                let stale = bodies.into_iter().any(|body| {
                    !geometry.is_suppressed(body)
                        && mesh_inputs.get(body).is_some_and(|made_from| {
                            fingerprint::mesh_input(geometry, space, body) != *made_from
                        })
                });
                stale.then(|| part.name.clone())
            })
            .collect(),
        None => Vec::new(),
    };
    // A mesh taken from a mesh block is out of date with the block.
    let mesh_state = if !meshed {
        CellState::Missing
    } else if stale_parts.is_empty() && upstream.stale_meshes.is_empty() {
        CellState::UpToDate
    } else {
        CellState::OutOfDate
    };
    let setup = if fe.steps.is_empty() {
        CellState::Missing
    } else {
        CellState::UpToDate
    };
    // Lowering a large model for its fingerprints takes seconds: after a change it runs in
    // the background, and the states found before stand until it is done. Without states
    // found before there is nothing to show meanwhile.
    let prints = results.map(|_| match previous {
        Some(_) => crate::solve_fingerprint::fingerprints_soon(mesh, fe),
        None => Some(crate::solve_fingerprint::fingerprints(mesh, fe)),
    });
    if let (Some(None), Some(previous)) = (&prints, previous) {
        return AnalysisStatus {
            geometry: geometry_state,
            mesh: mesh_state,
            setup,
            solution: if running {
                CellState::Running
            } else {
                previous.solution
            },
            results: previous.results,
            stale_parts,
            stale_sources: upstream.stale_meshes.clone(),
            stale_results: upstream.stale_results.clone(),
            step_inputs: previous.step_inputs.clone(),
            pending: true,
        };
    }
    let prints = prints.flatten();
    let solution = match (results, &prints) {
        _ if running => CellState::Running,
        (Some(results), Some(prints)) => {
            let solved = results.input == Some(prints.input);
            // Results taken from analyses whose own results are out of date are too.
            if !solved || mesh_state != CellState::UpToDate || !upstream.stale_results.is_empty() {
                CellState::OutOfDate
            } else if results.with_errors {
                CellState::Failed
            } else {
                CellState::UpToDate
            }
        }
        _ => CellState::Missing,
    };
    let results_state = match (results, solution) {
        (None, _) => CellState::Missing,
        (Some(_), CellState::OutOfDate | CellState::Running) => CellState::OutOfDate,
        (Some(_), _) => CellState::UpToDate,
    };
    AnalysisStatus {
        geometry: geometry_state,
        mesh: mesh_state,
        setup,
        solution,
        results: results_state,
        stale_parts,
        stale_sources: upstream.stale_meshes.clone(),
        stale_results: upstream.stale_results.clone(),
        step_inputs: prints.map(|p| p.steps).unwrap_or_default(),
        pending: false,
    }
}

/// Paints the symbol of a state into a 16 × 16 square at `min`, as Ansys Workbench marks its
/// cells, with the icons where it has one: OK, an arrow for out of date, a question mark
/// for missing.
pub fn paint(painter: &Painter, min: egui::Pos2, state: CellState) {
    let p = |x: f32, y: f32| pos2(min.x + x, min.y + y);
    let adapt = crate::style::adapt;
    match state {
        CellState::UpToDate => {
            // A plain tick.
            painter.add(Shape::line(
                vec![p(3.0, 8.5), p(6.5, 12.0), p(13.0, 4.5)],
                Stroke::new(2.0, crate::style::ok()),
            ));
        }
        CellState::OutOfDate => {
            // A circular arrow: Update.
            let amber = adapt(Color32::from_rgb(220, 150, 0));
            let center = p(8.0, 8.5);
            let points: Vec<egui::Pos2> = (0..=12)
                .map(|i| {
                    let angle = (40.0 + i as f32 * 25.0_f32).to_radians();
                    center + 5.0 * egui::vec2(angle.cos(), -angle.sin())
                })
                .collect();
            let end = *points.last().unwrap_or(&center);
            painter.add(Shape::line(points, Stroke::new(2.0, amber)));
            painter.add(Shape::convex_polygon(
                vec![
                    end + egui::vec2(-3.0, -1.5),
                    end + egui::vec2(3.0, -2.5),
                    end + egui::vec2(0.5, 2.5),
                ],
                amber,
                Stroke::NONE,
            ));
        }
        CellState::Missing => {
            painter.text(
                p(8.0, 8.5),
                egui::Align2::CENTER_CENTER,
                "?",
                egui::FontId::proportional(13.0),
                adapt(Color32::from_rgb(60, 110, 200)),
            );
        }
        CellState::Running => tree_icons::paint(painter, min, TreeIcon::Running),
        CellState::Failed => {
            // A plain cross.
            let stroke = Stroke::new(2.0, crate::style::error());
            painter.line_segment([p(3.5, 3.5), p(12.5, 12.5)], stroke);
            painter.line_segment([p(12.5, 3.5), p(3.5, 12.5)], stroke);
        }
        CellState::NotUsed => {
            painter.line_segment(
                [p(4.0, 8.5), p(12.0, 8.5)],
                Stroke::new(1.5, adapt(Color32::GRAY)),
            );
        }
    }
}

/// A 16 × 16 rect for a state symbol, centred in `rect`.
pub fn symbol_rect(rect: Rect) -> Rect {
    Rect::from_center_size(rect.center(), egui::Vec2::splat(tree_icons::SIZE))
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_model::{MeshingParameters, Step};

    fn block() -> FeMesh {
        let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        plx_io::inp::read_inp(&root.join("testdata/block_c3d20r.inp"))
            .unwrap()
            .mesh
    }

    fn results(input: u64) -> BoundResults {
        BoundResults {
            analysis: "Analysis-1".into(),
            input: Some(input),
            ..Default::default()
        }
    }

    #[test]
    fn a_change_keeps_the_states_found_before_until_the_background_fingerprints_are_ready() {
        let mesh = block();
        let mut fe = FeModel::default();
        fe.steps.push(Step::new_static("Step-1"));
        let solved = results(crate::solve_fingerprint::solve_input(&mesh, &fe));
        let status = |fe: &FeModel, previous: Option<&AnalysisStatus>| {
            let inputs = BTreeMap::new();
            let upstream = Upstream::default();
            analysis_status(
                None,
                &mesh,
                fe,
                &inputs,
                Some(&solved),
                &upstream,
                false,
                previous,
            )
        };
        let before = status(&fe, None);
        assert_eq!(before.solution, CellState::UpToDate);
        assert!(!before.pending);
        // A second step changes what the solver computes; the model is lowered for its
        // fingerprints in the background, and the states found before stand meanwhile.
        fe.steps.push(Step::new_static("Step-2"));
        let mut now = status(&fe, Some(&before));
        let start = std::time::Instant::now();
        while now.pending {
            assert_eq!(now.solution, CellState::UpToDate);
            assert_eq!(now.step_inputs, before.step_inputs);
            assert!(
                start.elapsed().as_secs() < 30,
                "the fingerprints never came"
            );
            let generation = crate::solve_fingerprint::generation();
            while crate::solve_fingerprint::generation() == generation
                && start.elapsed().as_secs() < 30
            {
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
            now = status(&fe, Some(&now));
        }
        assert_eq!(now.solution, CellState::OutOfDate);
        assert_eq!(
            now.step_inputs,
            crate::solve_fingerprint::step_inputs(&mesh, &fe)
        );
    }

    #[test]
    fn a_changed_load_makes_only_the_solution_out_of_date() {
        let mesh = block();
        let mut fe = FeModel::default();
        fe.steps.push(Step::new_static("Step-1"));
        let solved = results(crate::solve_fingerprint::solve_input(&mesh, &fe));
        let status = analysis_status(
            None,
            &mesh,
            &fe,
            &BTreeMap::new(),
            Some(&solved),
            &Upstream::default(),
            false,
            None,
        );
        assert_eq!(status.geometry, CellState::NotUsed);
        assert_eq!(status.mesh, CellState::UpToDate);
        assert_eq!(status.solution, CellState::UpToDate);
        assert!(!status.needs_solve());
        // It keeps what the results of each step are made from, for the blocks drawn in
        // every frame.
        assert_eq!(
            status.step_inputs,
            crate::solve_fingerprint::step_inputs(&mesh, &fe)
        );
        // A result path changes nothing that the solver computes.
        fe.result_paths.push(plx_model::ResultPath::new("Path-1"));
        let status = analysis_status(
            None,
            &mesh,
            &fe,
            &BTreeMap::new(),
            Some(&solved),
            &Upstream::default(),
            false,
            None,
        );
        assert_eq!(status.solution, CellState::UpToDate);
        // Nor does a name: the solver computes the same.
        fe.steps[0].name = "Loading".into();
        let status = analysis_status(
            None,
            &mesh,
            &fe,
            &BTreeMap::new(),
            Some(&solved),
            &Upstream::default(),
            false,
            None,
        );
        assert_eq!(status.solution, CellState::UpToDate);
        fe.steps[0].active = false;
        let status = analysis_status(
            None,
            &mesh,
            &fe,
            &BTreeMap::new(),
            Some(&solved),
            &Upstream::default(),
            false,
            None,
        );
        assert_eq!(status.mesh, CellState::UpToDate);
        assert_eq!(status.solution, CellState::OutOfDate);
        assert_eq!(status.results, CellState::OutOfDate);
        assert!(status.needs_solve());
    }

    #[test]
    fn new_meshing_settings_make_the_mesh_of_that_part_out_of_date() {
        let mesh = block();
        let part = mesh.parts[0].name.clone();
        let mut geometry = Geometry {
            compounds: Vec::new(),
            suppressed: Vec::new(),
            source: "block.step".into(),
            brep: "shapes".into(),
            meshing: MeshingParameters::default(),
            mesh_items: Vec::new(),
            part_names: Vec::new(),
            refinement: 1.0,
            signatures: Vec::new(),
        };
        let fe = FeModel::default();
        let space = fe.properties.space;
        let inputs = BTreeMap::from([(
            part.clone(),
            fingerprint::mesh_input(&geometry, space, &part),
        )]);
        let solved = results(crate::solve_fingerprint::solve_input(&mesh, &fe));
        let status = analysis_status(
            Some(&geometry),
            &mesh,
            &fe,
            &inputs,
            Some(&solved),
            &Upstream::default(),
            false,
            None,
        );
        assert_eq!(status.mesh, CellState::UpToDate);
        assert_eq!(status.setup, CellState::Missing);
        geometry.meshing.max_size *= 0.5;
        let status = analysis_status(
            Some(&geometry),
            &mesh,
            &fe,
            &inputs,
            Some(&solved),
            &Upstream::default(),
            false,
            None,
        );
        assert_eq!(status.mesh, CellState::OutOfDate);
        assert_eq!(status.stale_parts, [part]);
        // The solution follows the mesh.
        assert_eq!(status.solution, CellState::OutOfDate);
    }

    #[test]
    fn a_changed_mesh_block_makes_the_mesh_taken_from_it_out_of_date() {
        let mesh = block();
        let mut fe = FeModel::default();
        fe.steps.push(Step::new_static("Step-1"));
        let solved = results(crate::solve_fingerprint::solve_input(&mesh, &fe));
        let inputs = BTreeMap::new();
        let status = analysis_status(
            None,
            &mesh,
            &fe,
            &inputs,
            Some(&solved),
            &Upstream::default(),
            false,
            None,
        );
        assert!(!status.needs_solve());
        // The block's divisions changed; the analysis still holds the block's old mesh.
        let stale = ["Box-1".to_string()];
        let upstream = Upstream {
            stale_meshes: stale.to_vec(),
            ..Default::default()
        };
        let status = analysis_status(
            None,
            &mesh,
            &fe,
            &inputs,
            Some(&solved),
            &upstream,
            false,
            None,
        );
        assert_eq!(status.mesh, CellState::OutOfDate);
        assert_eq!(status.solution, CellState::OutOfDate);
        assert_eq!(status.results, CellState::OutOfDate);
        assert_eq!(status.stale_sources, stale);
        assert!(status.stale_parts.is_empty());
        assert!(status.needs_solve());
    }

    #[test]
    fn results_taken_from_an_out_of_date_analysis_make_the_solution_out_of_date() {
        let mesh = block();
        let mut fe = FeModel::default();
        fe.steps.push(Step::new_static("Step-1"));
        let solved = results(crate::solve_fingerprint::solve_input(&mesh, &fe));
        let upstream = Upstream {
            stale_results: vec!["Global".into()],
            ..Default::default()
        };
        assert!(!upstream.is_ready());
        let inputs = BTreeMap::new();
        let status = analysis_status(
            None,
            &mesh,
            &fe,
            &inputs,
            Some(&solved),
            &upstream,
            false,
            None,
        );
        // The mesh is its own; the solution used results that no longer fit their model.
        assert_eq!(status.mesh, CellState::UpToDate);
        assert_eq!(status.solution, CellState::OutOfDate);
        assert_eq!(status.stale_results, ["Global"]);
        assert!(status.needs_solve());
    }

    #[test]
    fn analyses_without_results_are_not_solved() {
        let status = analysis_status(
            None,
            &FeMesh::default(),
            &FeModel::default(),
            &BTreeMap::new(),
            None,
            &Upstream::default(),
            false,
            None,
        );
        assert_eq!(status.geometry, CellState::Missing);
        assert_eq!(status.mesh, CellState::Missing);
        assert_eq!(status.solution, CellState::Missing);
        let running = analysis_status(
            None,
            &FeMesh::default(),
            &FeModel::default(),
            &BTreeMap::new(),
            None,
            &Upstream::default(),
            true,
            None,
        );
        assert_eq!(running.solution, CellState::Running);
    }
}
