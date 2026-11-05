use plx_render::contour::{band_colors, normalize, rgb};

use crate::animation::{Animation, AnimationKind, ColorLimits};
use crate::sound::ModeSound;
use glam::{DAffine3, DMat3};
use plx_mesh::FeMesh;
use plx_model::{ColorScale, Quantity, UnitSystem};
use plx_results::display::Representation;
use plx_results::field_output::{self, FieldOutput};
use plx_results::history_output::{self, HistoryOutput, HistorySet};
use plx_results::transformation::{self, Transformation};
use plx_results::{AnalysisKind, Component, Field, Increment};

/// How the deformed shape is scaled, as in the results toolbar.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Deformation {
    Undeformed,
    TrueScale,
    /// Largest displacement drawn as a quarter of the model size, times this factor.
    Automatic(f32),
    UserDefined,
}

impl From<Deformation> for plx_results::saved_view::SavedDeformation {
    fn from(deformation: Deformation) -> Self {
        match deformation {
            Deformation::Undeformed => Self::Undeformed,
            Deformation::TrueScale => Self::TrueScale,
            Deformation::Automatic(factor) => Self::Automatic(factor),
            Deformation::UserDefined => Self::UserDefined,
        }
    }
}

impl From<plx_results::saved_view::SavedDeformation> for Deformation {
    fn from(saved: plx_results::saved_view::SavedDeformation) -> Self {
        use plx_results::saved_view::SavedDeformation as Saved;
        match saved {
            Saved::Undeformed => Self::Undeformed,
            Saved::TrueScale => Self::TrueScale,
            Saved::Automatic(factor) => Self::Automatic(factor),
            Saved::UserDefined => Self::UserDefined,
        }
    }
}

impl Deformation {
    pub const CHOICES: [Deformation; 8] = [
        Deformation::Undeformed,
        Deformation::TrueScale,
        Deformation::Automatic(0.25),
        Deformation::Automatic(0.5),
        Deformation::Automatic(1.0),
        Deformation::Automatic(2.0),
        Deformation::Automatic(5.0),
        Deformation::UserDefined,
    ];

    pub fn label(self) -> String {
        match self {
            Deformation::Undeformed => "Undeformed".into(),
            Deformation::TrueScale => "True scale".into(),
            Deformation::Automatic(1.0) => "Automatic".into(),
            Deformation::Automatic(f) => format!("Automatic × {f}"),
            Deformation::UserDefined => "User defined".into(),
        }
    }
}

/// Selection and display settings for the results of one `.frd` file.
pub struct ResultsView {
    pub increments: Vec<Increment>,
    /// The steps the increments belong to, see [`ResultsView::set_step_records`].
    pub steps: plx_results::steps::ResultSteps,
    pub increment: usize,
    pub field: usize,
    pub component: usize,
    pub deformation: Deformation,
    /// The user picked the deformation, so it stays when another step is shown; otherwise it
    /// follows [`ResultsView::default_deformation`].
    pub deformation_chosen: bool,
    /// Whether surfaces of the analysed model are in contact, from a `CONTACT` field in the
    /// results or a contact pair in the input next to them.
    pub contact: bool,
    pub user_scale: f64,
    /// Colour spectrum, number of colours and limits of the legend.
    pub color_scale: ColorScale,
    /// Draw the undeformed outline behind the deformed shape.
    pub show_undeformed: bool,
    /// Draw the edges of the global model of a submodel, deformed alike, around the results.
    pub show_global_contours: bool,
    /// When CalculiX ran the analysis, as written into the file.
    pub date: Option<String>,
    pub time: Option<String>,
    /// Running animation, if the animation window is open.
    pub animation: Option<Animation>,
    /// Settings of the sound window, if it is open.
    pub sound: Option<ModeSound>,
    /// Mode shapes overlaid while their sound plays; shown instead of the current increment.
    pub superposition: Option<Superposition>,
    /// Field outputs the user derived from the results, in the order they are computed.
    pub field_outputs: Vec<FieldOutput>,
    /// History outputs the user derived, in the order they are computed.
    pub history_outputs: Vec<HistoryOutput>,
    /// History CalculiX printed into the `.dat` file next to the results.
    pub file_history: Vec<HistorySet>,
    /// The evaluated hot spots of the results, which history outputs may take values from.
    pub hot_spot_reports: Vec<plx_results::hot_spot::HotSpotReport>,
    /// Positions of the moving loads of the FE model by step, which a history output may
    /// put beside its values.
    pub moving_loads: Vec<plx_model::LoadStations>,
    /// The sets of the `.dat` file followed by the data of the derived history outputs that
    /// could be computed, by name.
    pub history: Vec<HistorySet>,
    /// Mirrored and patterned copies drawn besides the results, the
    /// transformations; the legend and the extremes include them.
    pub transformations: Vec<Transformation>,
    /// Coordinate systems of the results' features, for transformed field outputs.
    pub coordinate_systems: Vec<plx_model::CoordinateSystem>,
    /// Range of the shown values on the section plane, which replaces the model's range in
    /// the legend while only the section is shown.
    pub section_range: Option<(f32, f32)>,
    /// Nodes that belong to plain parts only (colour contours off): drawn grey, left out of
    /// the legend range and the extremes. Empty while every part has contours.
    pub plain_nodes: Vec<bool>,
    /// The plane result shown alone, by index in the features of the results.
    pub plane_result: Option<usize>,
    /// Arrows over the contours and their options.
    pub vector_display: plx_results::display::VectorDisplay,
    /// Unit system of the values: that of the FE model the
    /// results belong to; the legend shows units after it.
    pub units: UnitSystem,
    /// Characteristic model size for the automatic scale (cube root of the bounding
    /// box volume, square root of the area for flat models).
    model_size: f64,
    /// [`max_deformation`] of each increment, once asked for: the toolbar wants it every frame
    /// and an increment animation for all its frames.
    deformation_peaks: Vec<std::cell::OnceCell<Option<f32>>>,
}

/// Several mode shapes overlaid at one moment, each scaled by its level and swinging at its
/// own (slowed down) frequency.
pub struct Superposition {
    pub displacements: Vec<[f32; 3]>,
    /// Shown component, superposed; `None` when it could not be built.
    pub values: Option<Vec<f32>>,
    /// Legend range, the same for all moments so the colours stay comparable.
    pub range: Option<(f32, f32)>,
    /// Largest displacement for the automatic deformation scale, the same for all moments.
    pub peak: f32,
    /// Numbers of the overlaid modes.
    pub modes: Vec<u32>,
}

/// What the viewport legend shows.
pub struct Legend {
    pub title: String,
    /// Limits of the colour bands.
    pub min: f32,
    pub max: f32,
    /// Colours of the bands, minimum first.
    pub colors: Vec<[f32; 3]>,
    /// Values below the minimum of a manual scale, if there are any.
    pub below: Option<OutOfRange>,
    /// Values above the maximum of a manual scale, if there are any.
    pub above: Option<OutOfRange>,
}

/// Values beyond the limits of a manual colour scale, drawn in a colour of their own.
pub struct OutOfRange {
    pub color: [f32; 3],
    /// The extreme value beyond the limit.
    pub value: f32,
    /// The legend shows a box for them.
    pub shown: bool,
}

impl Legend {
    pub fn levels(&self) -> u32 {
        self.colors.len() as u32
    }

    /// Range the values are normalized over for colouring: the bands', widened by one band
    /// for each out-of-range colour.
    pub fn color_range(&self) -> (f32, f32) {
        let band = (self.max - self.min) / self.levels().max(1) as f32;
        let below = if self.below.is_some() { band } else { 0.0 };
        let above = if self.above.is_some() { band } else { 0.0 };
        (self.min - below, self.max + above)
    }

    /// Colours of the 3D view, minimum first: the bands between the out-of-range colours.
    pub fn palette(&self) -> Vec<[f32; 3]> {
        (self.below.iter().map(|o| o.color))
            .chain(self.colors.iter().copied())
            .chain(self.above.iter().map(|o| o.color))
            .collect()
    }

    /// Values normalized for the palette.
    pub fn normalize(&self, values: &[f32]) -> Vec<f32> {
        let (min, max) = self.color_range();
        normalize(values, min, max)
    }
}

impl ResultsView {
    pub fn new(increments: Vec<Increment>, bounds: Option<([f64; 3], [f64; 3])>) -> Self {
        let steps = plx_results::steps::ResultSteps::new(&increments, &[], &[], None);
        let mut view = Self {
            increments,
            steps,
            increment: 0,
            field: 0,
            component: 0,
            deformation: Deformation::Automatic(1.0),
            deformation_chosen: false,
            contact: false,
            user_scale: 10.0,
            color_scale: ColorScale::default(),
            show_undeformed: true,
            show_global_contours: false,
            date: None,
            time: None,
            animation: None,
            sound: None,
            superposition: None,
            field_outputs: Vec::new(),
            history_outputs: Vec::new(),
            file_history: Vec::new(),
            hot_spot_reports: Vec::new(),
            moving_loads: Vec::new(),
            history: Vec::new(),
            transformations: Vec::new(),
            coordinate_systems: Vec::new(),
            section_range: None,
            plain_nodes: Vec::new(),
            plane_result: None,
            vector_display: Default::default(),
            units: UnitSystem::default(),
            model_size: bounds.map_or(1.0, model_size),
            deformation_peaks: Vec::new(),
        };
        view.deformation_peaks = (view.increments.iter())
            .map(|_| Default::default())
            .collect();
        view.increment = view.default_increment();
        view.contact =
            (view.increments.iter()).any(|i| i.fields.iter().any(|f| f.name == "CONTACT"));
        view.follow_default_deformation();
        view
    }

    /// The automatic scale, except for contact outside eigenmode steps: exaggerated,
    /// the touching parts appear to penetrate each other, which confuses new users. Mode
    /// shapes are scaled arbitrarily, so they always get the automatic scale.
    pub fn default_deformation(&self) -> Deformation {
        let modal = self.current_increment().is_some_and(|i| i.kind.is_modal());
        if self.contact && !modal {
            Deformation::TrueScale
        } else {
            Deformation::Automatic(1.0)
        }
    }

    /// Sets the default deformation for the shown increment unless the user picked one.
    pub fn follow_default_deformation(&mut self) {
        if !self.deformation_chosen {
            self.deformation = self.default_deformation();
        }
    }

    /// Nodalis opens the last increment of the last step, or the first mode of a frequency or
    /// buckle step; the reference state of a buckle step (increment 0) is skipped.
    fn default_increment(&self) -> usize {
        let Some(last) = self.increments.last() else {
            return 0;
        };
        let step: Vec<usize> = (0..self.increments.len())
            .filter(|&i| self.increments[i].step == last.step)
            .collect();
        if last.kind.is_frequency() {
            step[0]
        } else if last.kind == AnalysisKind::Buckling {
            let first_mode = step.iter().find(|&&i| self.increments[i].increment > 0);
            *first_mode.unwrap_or(&step[0])
        } else {
            *step.last().unwrap()
        }
    }

    /// A typical length of the model, as the edge of a cube of its bounding volume.
    pub fn model_size(&self) -> f64 {
        self.model_size
    }

    pub fn current_increment(&self) -> Option<&Increment> {
        self.increments.get(self.increment)
    }

    pub fn current(&self) -> Option<(&Field, &Component)> {
        let field = self.current_increment()?.fields.get(self.field)?;
        Some((field, field.components.get(self.component)?))
    }

    /// The field whose directions arrows or streaks follow: the chosen one, else the shown
    /// field.
    pub fn directions_field(&self) -> Option<&Field> {
        let (shown, _) = self.current()?;
        Some(
            self.vector_display
                .directions(shown, self.current_increment()?),
        )
    }

    /// The representation drawn: contours alone where the directions come from a scalar
    /// field.
    pub fn representation(&self) -> Representation {
        match self.directions_field() {
            Some(field) if self.superposition.is_none() => self.vector_display.shown(field),
            _ => Representation::Contour,
        }
    }

    /// Where arrows or streaks take their directions from, while the representation needs
    /// them.
    pub fn vector_source(&self) -> Option<plx_results::VectorSource> {
        if self.representation() == Representation::Contour {
            return None;
        }
        self.vector_display.source(self.directions_field()?)
    }

    /// Switches to another increment, keeping field and component by name where possible.
    pub fn select_increment(&mut self, index: usize) {
        let names = self
            .current()
            .map(|(f, c)| (f.name.clone(), c.name.clone()));
        self.increment = index;
        self.field = 0;
        self.component = 0;
        let found = names.and_then(|(field, component)| {
            let fields = &self.current_increment()?.fields;
            let f = fields.iter().position(|f| f.name == field)?;
            let c = fields[f]
                .components
                .iter()
                .position(|c| c.name == component);
            Some((f, c.unwrap_or(0)))
        });
        if let Some((field, component)) = found {
            self.field = field;
            self.component = component;
        }
        self.follow_default_deformation();
    }

    /// What the user defined on these results and what is shown, to be saved with results
    /// bound to a project; the model's features, paths and hot spots are added by the caller.
    pub fn settings(&self) -> plx_results::ResultsSettings {
        plx_results::ResultsSettings {
            field_outputs: self.field_outputs.clone(),
            history_outputs: self.history_outputs.clone(),
            transformations: self.transformations.clone(),
            increment: self.increment,
            shown: self.shown_names(),
            view: plx_results::saved_view::SavedView {
                deformation: self.deformation_chosen.then_some(self.deformation.into()),
                user_scale: Some(self.user_scale),
                show_undeformed: Some(self.show_undeformed),
                levels: Some(self.color_scale.levels),
                color_scale: Some(self.color_scale.clone()),
                vector_display: Some(self.vector_display.clone()),
                ..Default::default()
            },
            ..Default::default()
        }
    }

    /// Takes the derived outputs, transformations and shown field of saved settings and
    /// computes the outputs; returns their failures.
    pub fn restore(
        &mut self,
        settings: &plx_results::ResultsSettings,
        mesh: &FeMesh,
    ) -> Vec<String> {
        self.coordinate_systems = settings.coordinate_systems.clone();
        self.field_outputs = settings.field_outputs.clone();
        self.history_outputs = settings.history_outputs.clone();
        self.transformations = settings.transformations.clone();
        let mut warnings = self.recompute_from(0, mesh);
        warnings.extend(self.recompute_history(mesh));
        if settings.increment < self.increments.len() {
            self.increment = settings.increment;
        }
        self.restore_selection(settings.shown.clone());
        let view = &settings.view;
        if let Some(deformation) = view.deformation {
            self.deformation = deformation.into();
            self.deformation_chosen = true;
        }
        self.follow_default_deformation();
        self.user_scale = view.user_scale.unwrap_or(self.user_scale);
        self.show_undeformed = view.show_undeformed.unwrap_or(self.show_undeformed);
        if let Some(display) = &view.vector_display {
            self.vector_display = display.clone();
        }
        match &view.color_scale {
            Some(scale) => self.color_scale = scale.clone(),
            None => self.color_scale.levels = view.levels.unwrap_or(self.color_scale.levels),
        }
        warnings
    }

    /// Index of the derived field output that computes the given field.
    pub fn field_output_index(&self, field: usize) -> Option<usize> {
        let name = &self.current_increment()?.fields.get(field)?.name;
        self.field_outputs.iter().position(|o| o.name == *name)
    }

    /// Creates (`index` is `None`) or replaces a derived field output and computes it. The
    /// outputs after it are computed again, as they may use it; their failures are returned.
    pub fn set_field_output(
        &mut self,
        index: Option<usize>,
        output: FieldOutput,
        mesh: &FeMesh,
    ) -> Result<Vec<String>, String> {
        let shown = self.shown_names();
        field_output::compute(
            &output,
            &mut self.increments,
            mesh,
            &self.coordinate_systems,
        )?;
        let at = match index.filter(|&i| i < self.field_outputs.len()) {
            Some(i) => {
                let old = std::mem::replace(&mut self.field_outputs[i], output);
                if old.name != self.field_outputs[i].name {
                    field_output::remove(&old.name, &mut self.increments);
                }
                i
            }
            None => {
                self.field_outputs.push(output);
                self.field_outputs.len() - 1
            }
        };
        let mut warnings = self.recompute_from(at + 1, mesh);
        warnings.extend(self.recompute_history(mesh));
        self.restore_selection(shown);
        Ok(warnings)
    }

    /// Deletes a derived field output and its field.
    pub fn remove_field_output(&mut self, index: usize, mesh: &FeMesh) -> Vec<String> {
        if index >= self.field_outputs.len() {
            return Vec::new();
        }
        let shown = self.shown_names();
        let output = self.field_outputs.remove(index);
        field_output::remove(&output.name, &mut self.increments);
        let mut warnings = self.recompute_from(index, mesh);
        warnings.extend(self.recompute_history(mesh));
        self.restore_selection(shown);
        warnings
    }

    /// Takes the coordinate systems of the results' features and computes the outputs that
    /// depend on them again; returns their failures.
    pub fn set_coordinate_systems(
        &mut self,
        systems: Vec<plx_model::CoordinateSystem>,
        mesh: &FeMesh,
    ) -> Vec<String> {
        if systems == self.coordinate_systems {
            return Vec::new();
        }
        self.coordinate_systems = systems;
        let first = (self.field_outputs.iter()).position(|o| {
            matches!(
                o.kind,
                plx_results::FieldOutputKind::CoordinateSystemTransform { .. }
            )
        });
        let Some(first) = first else {
            return Vec::new();
        };
        let shown = self.shown_names();
        let mut warnings = self.recompute_from(first, mesh);
        warnings.extend(self.recompute_history(mesh));
        self.restore_selection(shown);
        warnings
    }

    /// Computes the outputs from `start` on again; one that fails loses its field.
    fn recompute_from(&mut self, start: usize, mesh: &FeMesh) -> Vec<String> {
        let mut warnings = Vec::new();
        for output in self.field_outputs.iter().skip(start) {
            let systems = &self.coordinate_systems;
            if let Err(error) = field_output::compute(output, &mut self.increments, mesh, systems) {
                field_output::remove(&output.name, &mut self.increments);
                warnings.push(format!("{}: {error}", output.name));
            }
        }
        warnings
    }

    fn shown_names(&self) -> Option<(String, String)> {
        self.current()
            .map(|(f, c)| (f.name.clone(), c.name.clone()))
    }

    /// Shows the field and component by name again after fields were added or removed.
    fn restore_selection(&mut self, names: Option<(String, String)>) {
        let found = names.and_then(|(field, component)| {
            let fields = &self.current_increment()?.fields;
            let f = fields.iter().position(|f| f.name == field)?;
            let c = fields[f]
                .components
                .iter()
                .position(|c| c.name == component);
            Some((f, c.unwrap_or(0)))
        });
        (self.field, self.component) = found.unwrap_or((0, 0));
    }

    /// Creates (`index` is `None`) or replaces a history output and computes it; the history
    /// outputs after it are computed again. Their failures are returned.
    pub fn set_history_output(
        &mut self,
        index: Option<usize>,
        output: HistoryOutput,
        mesh: &FeMesh,
    ) -> Result<Vec<String>, String> {
        let at = index.filter(|&i| i < self.history_outputs.len());
        // An equation sees the outputs before it.
        let before = at.unwrap_or(self.history_outputs.len());
        let derived = &self.history[self.file_history.len().min(self.history.len())..];
        let earlier: Vec<HistorySet> = (self.file_history.iter().cloned())
            .chain(
                (derived.iter())
                    .filter(|set| {
                        (self.history_outputs[..before].iter()).any(|o| o.name == set.name)
                    })
                    .cloned(),
            )
            .collect();
        let sources = history_output::Sources {
            sets: &earlier,
            ..self.history_sources()
        };
        history_output::compute(&output, &self.increments, mesh, sources)?;
        match at {
            Some(i) => self.history_outputs[i] = output,
            None => self.history_outputs.push(output),
        }
        Ok(self.recompute_history(mesh))
    }

    pub fn remove_history_output(&mut self, index: usize, mesh: &FeMesh) -> Vec<String> {
        if index >= self.history_outputs.len() {
            return Vec::new();
        }
        self.history_outputs.remove(index);
        self.recompute_history(mesh)
    }

    /// Computes all history outputs again, in order, after the sets of the `.dat` file, which
    /// their equations may use; failures leave an output without data.
    pub fn recompute_history(&mut self, mesh: &FeMesh) -> Vec<String> {
        let mut warnings = Vec::new();
        let mut sets = self.file_history.clone();
        for output in &self.history_outputs {
            let sources = history_output::Sources {
                sets: &sets,
                ..self.history_sources()
            };
            match history_output::compute(output, &self.increments, mesh, sources) {
                Ok(set) => sets.push(set),
                Err(error) => warnings.push(format!("{}: {error}", output.name)),
            }
        }
        self.history = sets;
        warnings
    }

    fn history_sources(&self) -> history_output::Sources<'_> {
        history_output::Sources {
            sets: &[],
            hot_spots: &self.hot_spot_reports,
            moving_loads: &self.moving_loads,
        }
    }

    /// Takes the evaluated hot spots and the moving loads of the FE model and computes the
    /// history outputs again if they changed; returns their failures, `None` if nothing
    /// changed.
    pub fn set_history_sources(
        &mut self,
        hot_spots: Vec<plx_results::hot_spot::HotSpotReport>,
        moving_loads: Vec<plx_model::LoadStations>,
        mesh: &FeMesh,
    ) -> Option<Vec<String>> {
        if hot_spots == self.hot_spot_reports && moving_loads == self.moving_loads {
            return None;
        }
        self.hot_spot_reports = hot_spots;
        self.moving_loads = moving_loads;
        Some(self.recompute_history(mesh))
    }

    /// Index of the history output of a computed set; the sets of the `.dat` file have none.
    pub fn history_output_index(&self, set: usize) -> Option<usize> {
        if set < self.file_history.len() {
            return None;
        }
        let name = &self.history.get(set)?.name;
        self.history_outputs.iter().position(|o| o.name == *name)
    }

    /// Sets the history of the `.dat` file, before the derived history outputs.
    pub fn set_file_history(&mut self, sets: Vec<HistorySet>, mesh: &FeMesh) -> Vec<String> {
        self.file_history = sets;
        self.recompute_history(mesh)
    }

    /// The step of the shown increment.
    pub fn current_step(&self) -> Option<&plx_results::steps::ResultStep> {
        self.steps.step_of(self.increment)
    }

    /// Index of the step of the shown increment in [`ResultsView::steps`].
    pub fn current_step_index(&self) -> Option<usize> {
        self.steps.place(self.increment).map(|p| p.step)
    }

    /// Shows the step `step` at its last increment, the state at its end.
    pub fn switch_step(&mut self, step: usize) {
        if let Some(index) = self.steps.entry_increment(step) {
            self.switch_increment(index);
        }
    }

    /// The steps before and after the shown one that have results, for Ctrl+Page Up and
    /// Ctrl+Page Down.
    pub fn neighbour_step(&self, forward: bool) -> Option<usize> {
        let current = self.current_step_index()?;
        let has = |s: &usize| !self.steps.steps[*s].increments.is_empty();
        if forward {
            (current + 1..self.steps.steps.len()).find(has)
        } else {
            (0..current).rev().find(has)
        }
    }

    /// A step as the Step tile and list name it: its number and name.
    pub fn step_title(&self, step: usize) -> String {
        self.steps
            .steps
            .get(step)
            .map_or_else(String::new, |s| format!("{} {}", step + 1, s.name))
    }

    /// The number of the increment `index` within its step: the increment or mode number
    /// of the solver, or the position of a moving load; `None` for the reference state of a
    /// buckle step.
    pub fn entry_number(&self, index: usize) -> Option<u32> {
        let inc = self.increments.get(index)?;
        let step = self.steps.step_of(index)?;
        if step.has_positions() {
            let position = step.solver_steps.iter().position(|&s| s == inc.step)?;
            return Some(position as u32 + 1);
        }
        (inc.kind != AnalysisKind::Buckling || inc.increment > 0).then_some(inc.increment)
    }

    /// The last number of the entries of the step `step`, see [`ResultsView::entry_number`].
    pub fn entry_count(&self, step: usize) -> Option<u32> {
        let step_data = self.steps.steps.get(step)?;
        if step_data.has_positions() {
            return Some(step_data.solver_steps.len() as u32);
        }
        let last = *step_data.increments.last()?;
        self.increments.get(last).map(|i| i.increment)
    }

    /// The shown increment in its step, "12 of 20", or "Reference" for the reference state
    /// of a buckle step.
    pub fn entry_value(&self) -> Option<String> {
        let step = self.current_step_index()?;
        Some(match self.entry_number(self.increment) {
            Some(number) => format!("{number} of {}", self.entry_count(step)?),
            None => "Reference".into(),
        })
    }

    /// The entry `index` named with its kind, "Mode 2" or "Reference state".
    pub fn entry_label(&self, index: usize) -> String {
        let noun = self
            .steps
            .step_of(index)
            .map_or("Increment", |s| s.entry_noun());
        let nd = (self.increments.get(index))
            .and_then(|i| i.nodal_diameter)
            .map_or_else(String::new, |nd| format!(" (ND {nd})"));
        match self.entry_number(index) {
            Some(number) => format!("{noun} {number}{nd}"),
            None => "Reference state".into(),
        }
    }

    /// The shown increment with its step, for the headers of plots and tables:
    /// "Operating load, increment 12".
    pub fn shown_label(&self) -> String {
        let step = self
            .current_step()
            .map_or_else(String::new, |s| s.name.clone());
        let entry = self.entry_label(self.increment).to_lowercase();
        format!("{step}, {entry}")
    }

    /// A time with the unit of the results.
    fn time_text(&self, value: f64) -> String {
        let unit = self.units.unit(plx_model::Quantity::Time);
        format!("{} {unit}", format_value(value as f32))
            .trim()
            .to_owned()
    }

    /// The information block in the top right corner of the 3D view.
    pub fn status_lines(&self, file_name: &str) -> Vec<String> {
        let mut lines = vec![format!(
            "Name: {file_name}   Date: {}   Time: {}",
            self.date.as_deref().unwrap_or("-"),
            self.time.as_deref().unwrap_or("-")
        )];
        if let (Some(index), Some(step)) = (self.current_step_index(), self.current_step()) {
            let mut line = format!(
                "Step {} of {}: {} ({})",
                index + 1,
                self.steps.steps.len(),
                step.name,
                step.description()
            );
            if let Some(source) = &step.inherited_from {
                line.push_str(&format!(", from {source}"));
            }
            if !step.converged {
                line.push_str(", not converged");
            }
            lines.push(line);
        }
        let count = (self.current_step_index())
            .and_then(|s| self.entry_count(s))
            .map_or_else(String::new, |n| format!(" of {n}"));
        let entry = format!("{}{count}", self.entry_label(self.increment));
        // A single swinging mode keeps the usual mode line.
        let superposed = self.superposition.as_ref().filter(|s| s.modes.len() > 1);
        let category = self.current_step().map(|s| s.category);
        if let Some(superposition) = superposed {
            let modes: Vec<String> = superposition.modes.iter().map(u32::to_string).collect();
            lines.push(format!("Superposition of modes: {}", modes.join(", ")));
        } else if let Some(inc) = self.current_increment() {
            let value = format_value(inc.value as f32);
            let hertz = self.units.unit(plx_model::Quantity::Frequency);
            let place = self.steps.place(self.increment);
            lines.push(match inc.kind {
                AnalysisKind::Frequency => format!("{entry}   Frequency: {value} {hertz}"),
                AnalysisKind::ComplexFrequency => {
                    let phase = match self.whirl() {
                        Some((phase, _)) => format!("   Phase: {:.0} deg", phase.to_degrees()),
                        None => String::new(),
                    };
                    format!("{entry}   Complex frequency (real part): {value} {hertz}{phase}")
                }
                AnalysisKind::Buckling if inc.increment == 0 => {
                    "Reference state of the buckling loads".to_owned()
                }
                AnalysisKind::Buckling => format!("{entry}   Buckling factor: {value}"),
                AnalysisKind::Sensitivity if inc.value != 0.0 => {
                    format!(
                        "Sensitivity   Mode {}   Frequency: {value} {hertz}",
                        inc.increment
                    )
                }
                AnalysisKind::Sensitivity => "Sensitivity".to_owned(),
                _ if category == Some(plx_results::steps::StepCategory::SteadyStateDynamics) => {
                    format!("{entry}   Frequency: {value} {hertz}")
                }
                _ => {
                    let mut line = entry.clone();
                    let step = self.current_step();
                    if let Some(time) = place.and_then(|p| p.step_time) {
                        line.push_str(&format!("   Step time: {}", self.time_text(time)));
                        if let Some(period) = step.and_then(|s| s.time_period) {
                            line.push_str(&format!(" of {}", self.time_text(period)));
                        }
                    }
                    if let Some(total) = place.and_then(|p| p.total_time) {
                        line.push_str(&format!("   Total time: {}", self.time_text(total)));
                    }
                    line
                }
            });
        }
        lines.push(format!(
            "Deformation variable: Displacements   Deformation scale factor: {}",
            format_value(self.scale())
        ));
        lines
    }

    /// Node index, value and drawn item ([`ResultsView::instances`]) of the largest value on
    /// screen, animation frame included.
    pub fn maximum(&self) -> Option<(usize, f32, usize)> {
        self.extreme(|a, b| b > a)
    }

    /// Displacement scale factor for the current increment and deformation setting.
    /// The largest displacement of an increment.
    fn deformation_peak(&self, index: usize) -> Option<f32> {
        let increment = self.increments.get(index)?;
        match self.deformation_peaks.get(index) {
            Some(peak) => *peak.get_or_init(|| max_deformation(increment)),
            None => max_deformation(increment),
        }
    }

    pub fn scale(&self) -> f32 {
        match self.deformation {
            Deformation::Undeformed => 0.0,
            Deformation::TrueScale => 1.0,
            Deformation::UserDefined => self.user_scale as f32,
            Deformation::Automatic(factor) => {
                // An increment animation keeps one scale for all frames: that of the largest
                // displacement among them.
                let animated = self
                    .animation
                    .as_ref()
                    .filter(|a| a.kind.plays_increments())
                    .map(|a| a.increments.as_slice());
                let max = match animated.filter(|i| !i.is_empty()) {
                    _ if self.superposition.is_some() => {
                        self.superposition.as_ref().map(|s| s.peak)
                    }
                    Some(indices) => indices
                        .iter()
                        .filter_map(|&i| self.deformation_peak(i))
                        .reduce(f32::max),
                    None => self.deformation_peak(self.increment),
                };
                let Some(max) = max else {
                    return 0.0;
                };
                if max <= 0.0 {
                    return 1.0;
                }
                round_significant(factor as f64 * 0.25 * self.model_size / max as f64, 2) as f32
            }
        }
    }

    /// Factor on the deformation of the shown animation frame; 1 without animation, and 1
    /// while a complex mode whirls, as its displacements already carry the phase.
    pub fn amplitude(&self) -> f32 {
        if self.whirl().is_some() {
            return 1.0;
        }
        self.animation.as_ref().map_or(1.0, Animation::amplitude)
    }

    /// Factor on the shown values: the deformation factor, without its sign for magnitudes and
    /// equivalent values, which stay positive while a mode shape swings.
    /// Values other than the displacements of a whirling complex mode follow its real part.
    pub fn value_amplitude(&self) -> f32 {
        let amplitude = match self.whirl() {
            Some((phase, _)) => phase.cos(),
            None => self.amplitude(),
        };
        match self.current() {
            Some((_, component)) if component.is_invariant() => amplitude.abs(),
            _ => amplitude,
        }
    }

    /// The whirl of a complex mode during a scale factor animation: the phase angle of the
    /// shown frame, one turn over the frames, and the `PDISP` field with the magnitudes and
    /// phases of the displacements the solver writes for `PU` output.
    pub fn whirl(&self) -> Option<(f32, &Field)> {
        let animation = self.animation.as_ref()?;
        if animation.kind != AnimationKind::ScaleFactor || self.superposition.is_some() {
            return None;
        }
        let increment = self.current_increment()?;
        if increment.kind != AnalysisKind::ComplexFrequency {
            return None;
        }
        let field = increment.field("PDISP")?;
        let complete = ["MAG1", "MAG2", "MAG3", "PHA1", "PHA2", "PHA3"]
            .iter()
            .all(|n| field.component(n).is_some());
        complete.then(|| (animation.progress() * std::f32::consts::TAU, field))
    }

    /// Displacements of a whirling complex mode at the phase of the shown frame:
    /// `u_k = MAG_k cos(phase + PHA_k)` per node.
    fn whirl_displacements(phase: f32, field: &Field) -> Vec<[f32; 3]> {
        let nodes = field.component("MAG1").map_or(0, |c| c.values.len());
        (0..nodes)
            .map(|i| Self::whirl_displacement(phase, field, i))
            .collect()
    }

    /// [`Self::whirl_displacements`] of one node.
    fn whirl_displacement(phase: f32, field: &Field, node: usize) -> [f32; 3] {
        let value = |name: &str| field.component(name).map_or(f32::NAN, |c| c.values[node]);
        [("MAG1", "PHA1"), ("MAG2", "PHA2"), ("MAG3", "PHA3")].map(|(mag, pha)| {
            let (m, p) = (value(mag), value(pha));
            if m.is_finite() && p.is_finite() {
                m * (phase + p.to_radians()).cos()
            } else {
                0.0
            }
        })
    }

    /// Values of the shown displacement component of a whirling complex mode.
    fn whirl_values(&self, phase: f32, field: &Field) -> Option<Vec<f32>> {
        let (shown, component) = self.current()?;
        if shown.name != "DISP" {
            return None;
        }
        let displacements = Self::whirl_displacements(phase, field);
        let values = match component.name.as_str() {
            "U1" => displacements.iter().map(|d| d[0]).collect(),
            "U2" => displacements.iter().map(|d| d[1]).collect(),
            "U3" => displacements.iter().map(|d| d[2]).collect(),
            "ALL" => displacements
                .iter()
                .map(|d| (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt())
                .collect(),
            _ => return None,
        };
        Some(values)
    }

    /// Range of the displacement component over a whole turn of a whirling complex mode.
    fn whirl_range(&self, field: &Field) -> Option<(f32, f32)> {
        let (shown, component) = self.current()?;
        if shown.name != "DISP" {
            return None;
        }
        let magnitude = ["MAG1", "MAG2", "MAG3"].map(|n| &field.component(n).unwrap().values);
        let peak = |values: &[f32]| {
            (values.iter().copied().enumerate())
                .filter(|&(node, v)| v.is_finite() && !self.is_plain(node))
                .fold(0.0, |peak, (_, v)| f32::max(peak, v))
        };
        match component.name.as_str() {
            "U1" => Some((-peak(magnitude[0]), peak(magnitude[0]))),
            "U2" => Some((-peak(magnitude[1]), peak(magnitude[1]))),
            "U3" => Some((-peak(magnitude[2]), peak(magnitude[2]))),
            "ALL" => {
                let norm = (0..magnitude[0].len())
                    .filter(|&i| !self.is_plain(i))
                    .map(|i| {
                        [0, 1, 2]
                            .iter()
                            .map(|&k| magnitude[k][i].powi(2))
                            .sum::<f32>()
                    })
                    .filter(|v| v.is_finite())
                    .fold(0.0, f32::max);
                Some((0.0, norm.sqrt()))
            }
            _ => None,
        }
    }

    /// Ties the increments to the steps of the FE model the solver recorded with them, see
    /// [`plx_results::steps`].
    pub fn set_step_records(
        &mut self,
        records: &[plx_results::steps::StepRecord],
        convergence: &[plx_results::steps::IncrementRecord],
        unconverged: Option<u32>,
    ) {
        self.steps = plx_results::steps::ResultSteps::new(
            &self.increments,
            records,
            convergence,
            unconverged,
        );
    }

    /// Shows another increment, also while the animation window is open: the animation then
    /// continues with the new increment (and its step) instead of being restarted.
    pub fn switch_increment(&mut self, target: usize) {
        let Some(mut animation) = self.animation.take() else {
            self.select_increment(target);
            return;
        };
        self.select_increment(target);
        let modal = self.current_increment().is_some_and(|i| i.kind.is_modal());
        let increments = self.animated_increments(animation.kind);
        animation.retarget(increments, target, modal);
        self.animation = Some(animation);
        self.show_animation_frame();
    }

    /// Opens an animation of the given kind over the step of the shown increment.
    pub fn start_animation(&mut self, kind: AnimationKind) {
        let start = self
            .animation
            .take()
            .map_or(self.increment, |a| a.start_increment);
        self.select_increment(start);
        let modal = self.current_increment().is_some_and(|i| i.kind.is_modal());
        let increments = self.animated_increments(kind);
        self.animation = Some(Animation::new(kind, increments, start, modal));
        self.show_animation_frame();
    }

    /// The increments an animation plays from the current increment: those of its step, of
    /// all steps of its kind, or the last increment of each position of its moving load.
    fn animated_increments(&self, kind: AnimationKind) -> Vec<usize> {
        let Some(current) = self.current_step() else {
            return Vec::new();
        };
        match kind {
            AnimationKind::ScaleFactor | AnimationKind::Step => current.increments.clone(),
            // Timed steps play one after another; modal steps show their modes, apart from
            // the timed ones.
            AnimationKind::AllSteps => (self.steps.steps.iter())
                .filter(|s| {
                    if current.is_timed() {
                        s.is_timed()
                    } else {
                        s.category == current.category
                    }
                })
                .flat_map(|s| s.increments.iter().copied())
                .collect(),
            AnimationKind::Positions => {
                let mut last: Vec<usize> = Vec::new();
                for &i in &current.increments {
                    match last.last_mut() {
                        Some(previous)
                            if self.increments[*previous].step == self.increments[i].step =>
                        {
                            *previous = i
                        }
                        _ => last.push(i),
                    }
                }
                last
            }
        }
    }

    /// Displacements as shown: those of the overlaid modes, or of the current increment.
    pub fn shown_displacements(&self) -> Option<std::borrow::Cow<'_, [[f32; 3]]>> {
        if let Some((phase, field)) = self.whirl() {
            return Some(std::borrow::Cow::Owned(Self::whirl_displacements(
                phase, field,
            )));
        }
        match &self.superposition {
            Some(superposition) => Some(std::borrow::Cow::Borrowed(&superposition.displacements)),
            None => self
                .current_increment()?
                .displacements()
                .map(std::borrow::Cow::Owned),
        }
    }

    /// Displacement of one node as shown.
    pub fn shown_displacement(&self, node: usize) -> Option<[f32; 3]> {
        if let Some((phase, field)) = self.whirl() {
            let nodes = field.component("MAG1").map_or(0, |c| c.values.len());
            return (node < nodes).then(|| Self::whirl_displacement(phase, field, node));
        }
        if let Some(superposition) = &self.superposition {
            return superposition.displacements.get(node).copied();
        }
        let field = self.current_increment()?.field("DISP")?;
        Some(["U1", "U2", "U3"].map(|n| field.component(n).map_or(0.0, |c| c.values[node])))
    }

    /// Values of the shown component as drawn: overlaid, or scaled by the animation frame.
    pub fn shown_values(&self) -> Option<std::borrow::Cow<'_, [f32]>> {
        self.without_plain(self.all_shown_values()?)
    }

    fn all_shown_values(&self) -> Option<std::borrow::Cow<'_, [f32]>> {
        let (_, component) = self.current()?;
        if let Some(superposition) = &self.superposition {
            return superposition
                .values
                .as_deref()
                .map(std::borrow::Cow::Borrowed);
        }
        if let Some((phase, field)) = self.whirl()
            && let Some(values) = self.whirl_values(phase, field)
        {
            return Some(std::borrow::Cow::Owned(values));
        }
        let amplitude = self.value_amplitude();
        Some(std::borrow::Cow::Owned(
            component.values.iter().map(|v| v * amplitude).collect(),
        ))
    }

    /// Values with those of the plain parts' nodes made missing, so that they are drawn grey
    /// and counted nowhere.
    fn without_plain<'a>(
        &self,
        values: std::borrow::Cow<'a, [f32]>,
    ) -> Option<std::borrow::Cow<'a, [f32]>> {
        if self.plain_nodes.iter().all(|&plain| !plain) {
            return Some(values);
        }
        let mut values = values.into_owned();
        for (value, _) in values
            .iter_mut()
            .zip(&self.plain_nodes)
            .filter(|(_, p)| **p)
        {
            *value = f32::NAN;
        }
        Some(std::borrow::Cow::Owned(values))
    }

    /// Whether the node is drawn without contours.
    fn is_plain(&self, node: usize) -> bool {
        self.plain_nodes.get(node).copied().unwrap_or(false)
    }

    /// Every drawn item: the results themselves first, then the transformed copies.
    pub fn instances(&self) -> Vec<DAffine3> {
        transformation::instances(&self.transformations)
    }

    /// Values of the shown component as drawn on the item transformed by `instance`:
    /// directional components turn and mirror with it. Overlaid modes keep their values.
    pub fn shown_values_on(&self, instance: &DAffine3) -> Option<std::borrow::Cow<'_, [f32]>> {
        if self.superposition.is_none()
            && self.whirl().is_none()
            && let Some((field, _)) = self.current()
            && let Some(values) =
                transformation::transformed_values(field, self.component, instance.matrix3)
        {
            let amplitude = self.value_amplitude();
            return self.without_plain(std::borrow::Cow::Owned(
                values.iter().map(|v| v * amplitude).collect(),
            ));
        }
        self.shown_values()
    }

    /// Smallest and largest finite value outside the plain parts.
    fn range_of(&self, values: &[f32]) -> Option<(f32, f32)> {
        let finite = (values.iter().copied().enumerate())
            .filter(|&(node, v)| v.is_finite() && !self.is_plain(node));
        finite.fold(None, |range, (_, v)| match range {
            None => Some((v, v)),
            Some((min, max)) => Some((min.min(v), max.max(v))),
        })
    }

    /// Range of a component over the results and all transformed copies.
    fn component_range(&self, field: &Field, component: usize) -> Option<(f32, f32)> {
        let mut range = self.range_of(&field.components.get(component)?.values);
        let mut rotations: Vec<DMat3> = Vec::new();
        for instance in self.instances().iter().skip(1) {
            let q = instance.matrix3;
            if rotations.iter().any(|r| r.abs_diff_eq(q, 1e-12)) {
                continue;
            }
            rotations.push(q);
            let Some(values) = transformation::transformed_values(field, component, q) else {
                continue;
            };
            range = match (range, self.range_of(&values)) {
                (Some((a, b)), Some((c, d))) => Some((a.min(c), b.max(d))),
                (a, b) => a.or(b),
            };
        }
        range
    }

    /// Ends the animation and shows the increment from before it again.
    pub fn stop_animation(&mut self) {
        if let Some(animation) = self.animation.take() {
            self.select_increment(animation.start_increment);
        }
    }

    /// Selects the increment of the current frame of an increment animation.
    pub fn show_animation_frame(&mut self) {
        if let Some(increment) = self.animation.as_ref().and_then(Animation::increment)
            && increment != self.increment
        {
            self.select_increment(increment);
        }
    }

    /// Value range of the legend: that of the shown values, or over all animation frames.
    fn value_range(&self) -> Option<(f32, f32)> {
        let (field, component) = self.current()?;
        if let Some(superposition) = &self.superposition {
            return superposition.range;
        }
        let (min, max) = self.component_range(field, self.component)?;
        let Some(animation) = &self.animation else {
            return Some((min, max));
        };
        if let Some((_, pdisp)) = self.whirl()
            && let Some(range) = self.whirl_range(pdisp)
        {
            return match animation.limits {
                ColorLimits::AllFrames => Some(range),
                ColorLimits::CurrentFrame => {
                    let values = self.shown_values()?;
                    let finite = values.iter().copied().filter(|v| v.is_finite());
                    finite
                        .map(|v| (v, v))
                        .reduce(|(a, b), (c, d)| (a.min(c), b.max(d)))
                }
            };
        }
        match (animation.kind, animation.limits) {
            (AnimationKind::ScaleFactor, ColorLimits::CurrentFrame) => {
                // A negative factor swaps the ends.
                let (a, b) = (self.value_amplitude() * min, self.value_amplitude() * max);
                Some((a.min(b), a.max(b)))
            }
            // A mode shape swings every value between minus and plus its full size.
            (AnimationKind::ScaleFactor, ColorLimits::AllFrames)
                if animation.modal && !component.is_invariant() =>
            {
                Some((min.min(-max), max.max(-min)))
            }
            // Scaling runs every value from zero to its full size.
            (AnimationKind::ScaleFactor, ColorLimits::AllFrames) => {
                Some((min.min(0.0), max.max(0.0)))
            }
            (
                AnimationKind::Step | AnimationKind::AllSteps | AnimationKind::Positions,
                ColorLimits::CurrentFrame,
            ) => Some((min, max)),
            (
                AnimationKind::Step | AnimationKind::AllSteps | AnimationKind::Positions,
                ColorLimits::AllFrames,
            ) => animation
                .increments
                .iter()
                .filter_map(|&i| {
                    let field = self.increments.get(i)?.field(&field.name)?;
                    let index = (field.components.iter()).position(|c| c.name == component.name)?;
                    self.component_range(field, index)
                })
                .reduce(|(a, b), (c, d)| (a.min(c), b.max(d))),
        }
    }

    /// Like [`ResultsView::maximum`] for the smallest value.
    pub fn minimum(&self) -> Option<(usize, f32, usize)> {
        self.extreme(|a, b| b < a)
    }

    fn extreme(&self, better: impl Fn(f32, f32) -> bool) -> Option<(usize, f32, usize)> {
        let mut best: Option<(usize, f32, usize)> = None;
        for (item, instance) in self.instances().iter().enumerate() {
            let values = self.shown_values_on(instance)?;
            let found = (values.iter().copied().enumerate())
                .filter(|(_, v)| v.is_finite())
                .reduce(|a, b| if better(a.1, b.1) { b } else { a });
            if let Some((node, value)) = found
                && best.is_none_or(|b| better(b.1, value))
            {
                best = Some((node, value, item));
            }
        }
        best
    }

    pub fn legend(&self) -> Option<Legend> {
        let (field, component) = self.current()?;
        let (low, high) = match self.section_range {
            Some(range) => range,
            None => self.value_range()?,
        };
        let scale = &self.color_scale;
        let beyond = |value: f32, color: [u8; 3], shown: bool| OutOfRange {
            color: rgb(color),
            value,
            shown,
        };
        let (min, max, below, above) = match scale.manual_limits() {
            Some((min, max)) => {
                let (min, max) = (min as f32, max as f32);
                let below = (low < min).then(|| beyond(low, scale.min_color, scale.show_min_color));
                let above =
                    (high > max).then(|| beyond(high, scale.max_color, scale.show_max_color));
                (min, max, below, above)
            }
            None => (low, high, None, None),
        };
        let scope = if scale.manual_limits().is_some() {
            "Manual"
        } else if self.section_range.is_some() {
            "Section plane"
        } else {
            "Automatic"
        };
        // Names are shown with blanks instead of underscores and dashes.
        let name = |n: &str| n.replace(['_', '-'], " ");
        let derived = self.field_outputs.iter().find(|o| o.name == field.name);
        let unit = match derived.map(|o| &o.kind) {
            // The principal value of a direction is in the unit of its tensor.
            Some(plx_results::FieldOutputKind::PrincipalDirection { field, .. })
                if matches!(component.name.as_str(), "VALUE" | "ALL") =>
            {
                field_unit(field, &component.name, self.units)
            }
            Some(_) => derived.and_then(|o| o.unit()).map(|u| u.trim().to_string()),
            None => field_unit(&field.name, &component.name, self.units),
        };
        let unit = unit
            .filter(|u| !u.is_empty() && u != "/")
            .map(|u| format!("\nUnit: {u}"))
            .unwrap_or_default();
        // What the arrows or streaks show, as their colours come from the component.
        let arrows = self
            .vector_source()
            .map(|source| {
                let kind = match self.representation() {
                    Representation::Lic => "LIC",
                    _ => "Arrows",
                };
                format!("\n{kind}: {}", source.label())
            })
            .unwrap_or_default();
        Some(Legend {
            title: format!(
                "{}: {}{unit}\n{scope}{arrows}",
                name(&field.name),
                name(&component.name)
            ),
            min,
            max,
            colors: band_colors(scale),
            below,
            above,
        })
    }
}

/// What a value of a field of CalculiX's results measures.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Measure {
    Of(Quantity),
    Per(Quantity, Quantity),
}

/// What the values of a component of a field of CalculiX's results measure, in the `.frd`
/// file or, by the names of its blocks, in the `.dat` file; `None` for values without a
/// unit, such as strains and phase angles, and for unknown fields.
fn field_measure(field: &str, component: &str) -> Option<Measure> {
    use Measure::{Of, Per};
    use Quantity::*;
    let component = component.to_ascii_uppercase();
    match field.to_ascii_uppercase().as_str() {
        "DISP" | "DISPR" | "DISPI" | "DISPLACEMENTS" | "RELATIVE_CONTACT_DISPLACEMENT" => {
            Some(Of(Length))
        }
        "STRESS" | "STRESSR" | "STRESSI" | "ZZSTR" | "ZZSTRR" | "ZZSTRI" | "STRESSES"
        | "CONTACT_STRESS" => Some(Of(Pressure)),
        // Magnitudes and phase angles of complex results.
        "PDISP" => (component.starts_with("MAG")).then_some(Of(Length)),
        "PSTRESS" => (component.starts_with("MAG")).then_some(Of(Pressure)),
        "FORC" | "FORCR" | "FORCI" | "FORCES" | "TOTAL_FORCE" => Some(Of(Force)),
        "ROTATIONS" => Some(Of(Angle)),
        "MOMENTS" | "TOTAL_MOMENT" => Some(Of(Moment)),
        "ANGULAR_VELOCITIES" => Some(Of(RotationalSpeed)),
        "VELO" | "VELOCITIES" => Some(Of(Velocity)),
        "ENER" | "INTERNAL_ENERGY_DENSITY" => Some(Per(Energy, Volume)),
        "CELS"
        | "INTERNAL_ENERGY"
        | "TOTAL_INTERNAL_ENERGY"
        | "KINETIC_ENERGY"
        | "TOTAL_KINETIC_ENERGY"
        | "CONTACT_PRINT_ENERGY"
        | "CONTACT_SPRING_ENERGY"
        | "TOTAL_CONTACT_SPRING_ENERGY" => Some(Of(Energy)),
        "NDTEMP" | "TEMPERATURES" => Some(Of(Temperature)),
        "FLUX" | "HEAT_FLUX" => Some(Per(Power, Area)),
        "RFL"
        | "HEAT_GENERATION"
        | "TOTAL_HEAT_GENERATION"
        | "BODY_HEATING"
        | "TOTAL_BODY_HEATING" => Some(Of(Power)),
        "VOLUME" | "TOTAL_VOLUME" => Some(Of(Volume)),
        "CONTACT" => match component.as_str() {
            "COPEN" | "CSLIP1" | "CSLIP2" => Some(Of(Length)),
            "CPRESS" | "CSHEAR1" | "CSHEAR2" => Some(Of(Pressure)),
            _ => None,
        },
        _ => None,
    }
}

/// The unit of a field of CalculiX's results in `units`, as the legend shows it;
/// `None` for fields without a unit, such as strains, and for unknown fields.
pub fn field_unit(field: &str, component: &str, units: UnitSystem) -> Option<String> {
    let unit = |quantity| Some(units.unit(quantity).to_string()).filter(|u| !u.is_empty());
    let errors = ["ERROR", "ERRORR", "ERRORI", "HERROR", "HERRORR", "HERRORI"];
    if errors.iter().any(|e| field.eq_ignore_ascii_case(e)) {
        return Some("%".into());
    }
    match field_measure(field, component)? {
        Measure::Of(quantity) => unit(quantity),
        Measure::Per(a, b) => Some(format!("{}/{}", unit(a)?, unit(b)?)),
    }
}

/// The kind of number a component of a field is, for a value link: a factor for values
/// without a unit; `None` for a unit that is no quantity of a parameter, such as W/m².
pub fn field_value_kind(field: &str, component: &str) -> Option<plx_model::ValueKind> {
    match field_measure(field, component) {
        Some(Measure::Of(quantity)) => Some(plx_model::ValueKind::Quantity(quantity)),
        Some(Measure::Per(..)) => None,
        None => Some(plx_model::ValueKind::Factor),
    }
}

/// A value of a component of a field CalculiX wrote, in the stored SI units, in `units`;
/// unchanged for values without a unit.
pub fn shown_field_value(value: f64, field: &str, component: &str, units: UnitSystem) -> f64 {
    match field_measure(field, component) {
        Some(measure) => shown_value(value, measure, units),
        None => value,
    }
}

/// Converts a value CalculiX wrote, in the stored SI units ([`plx_model::STORED`]), into
/// `units`.
fn shown_value(value: f64, measure: Measure, units: UnitSystem) -> f64 {
    match measure {
        Measure::Of(quantity) => units.shown(value, quantity),
        Measure::Per(a, b) => {
            let stored = plx_model::STORED;
            value * stored.factor_to(a, units) / stored.factor_to(b, units)
        }
    }
}

/// Converts the results of a run, which CalculiX wrote in the stored SI units as it got the
/// model, into the units the analysis shows: the mesh they were computed on and the values
/// of every field.
pub fn convert_results(mesh: &mut FeMesh, increments: &mut [Increment], units: UnitSystem) {
    let length = plx_model::STORED.factor_to(Quantity::Length, units);
    if length != 1.0 {
        mesh.scale(length);
    }
    for field in increments.iter_mut().flat_map(|i| i.fields.iter_mut()) {
        for component in &mut field.components {
            if let Some(measure) = field_measure(&field.name, &component.name) {
                for value in &mut component.values {
                    *value = shown_value(f64::from(*value), measure, units) as f32;
                }
            }
        }
    }
}

/// Converts history output CalculiX printed in the stored SI units into `units`.
pub fn convert_history(sets: &mut [HistorySet], units: UnitSystem) {
    for field in sets.iter_mut().flat_map(|s| s.fields.iter_mut()) {
        for component in &mut field.components {
            if let Some(measure) = field_measure(&field.name, &component.name) {
                for value in component
                    .entries
                    .iter_mut()
                    .flat_map(|e| e.values.iter_mut())
                {
                    *value = shown_value(*value, measure, units);
                }
            }
        }
    }
}

/// The measure of the largest displacement: the root of the summed squares of the
/// largest absolute value of every `DISP` component, including `ALL`.
fn max_deformation(increment: &Increment) -> Option<f32> {
    // A complex mode reaches its magnitudes over a turn, not in its real part alone.
    let field = match increment.field("PDISP") {
        Some(pdisp)
            if ["MAG1", "MAG2", "MAG3"]
                .iter()
                .all(|n| pdisp.component(n).is_some()) =>
        {
            let magnitudes = Field::new(
                String::new(),
                ["MAG1", "MAG2", "MAG3"]
                    .iter()
                    .map(|n| pdisp.component(n).unwrap().clone())
                    .collect(),
            );
            return max_deformation_of(&magnitudes);
        }
        _ => increment.field("DISP")?,
    };
    max_deformation_of(field)
}

fn max_deformation_of(field: &Field) -> Option<f32> {
    let sum: f32 = field
        .components
        .iter()
        .filter_map(Component::range)
        .map(|(min, max)| min.abs().max(max.abs()).powi(2))
        .sum();
    Some(sum.sqrt())
}

pub(crate) fn model_size((min, max): ([f64; 3], [f64; 3])) -> f64 {
    let d = [0, 1, 2].map(|k| (max[k] - min[k]).abs());
    let diagonal = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    // Flat models, in any plane, and straight lines have no volume: their area or length
    // stands in for it.
    let spans: Vec<f64> = d.into_iter().filter(|&s| s >= 1e-6 * diagonal).collect();
    match spans.as_slice() {
        [a, b, c] => (a * b * c).cbrt(),
        [a, b] => (a * b).sqrt(),
        _ => diagonal,
    }
}

pub(crate) fn round_significant(value: f64, digits: i32) -> f64 {
    if value == 0.0 || !value.is_finite() {
        return value;
    }
    let magnitude = 10f64.powi(digits - 1 - value.abs().log10().floor() as i32);
    (value * magnitude).round() / magnitude
}

/// Number with four significant digits, the "G4" format.
pub fn format_value(value: f32) -> String {
    let abs = value.abs();
    if value == 0.0 {
        "0".into()
    } else if (1e-3..1e4).contains(&abs) {
        let decimals = (3 - abs.log10().floor() as i32).max(0) as usize;
        let text = format!("{value:.decimals$}");
        if text.contains('.') {
            text.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            text
        }
    } else {
        format!("{value:.3E}")
    }
}

/// Legend label: like [`format_value`] with a sign on non-negative values.
pub fn format_legend_value(value: f32) -> String {
    let text = format_value(value);
    if value >= 0.0 {
        format!("+{text}")
    } else {
        text
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn increment(step: u32, number: u32, kind: AnalysisKind, disp: &[[f32; 3]]) -> Increment {
        let column = |k: usize, name: &str| Component {
            name: name.into(),
            values: disp.iter().map(|d| d[k]).collect(),
            derived: false,
        };
        let mut field = Field::new(
            "DISP",
            vec![column(0, "U1"), column(1, "U2"), column(2, "U3")],
        );
        plx_results::add_derived_components(&mut field);
        Increment {
            nodal_diameter: None,
            step,
            increment: number,
            kind,
            value: number as f64,
            fields: vec![field],
        }
    }

    #[test]
    fn results_in_si_are_shown_in_the_model_units() {
        let mut mesh = FeMesh::default();
        mesh.set_node(1, [1.0, 0.5, 0.0]);
        let mut increments = vec![increment(1, 1, AnalysisKind::Static, &[[0.001, 0.0, 0.0]])];
        increments[0].fields.push(Field::new(
            "STRESS",
            vec![Component {
                name: "SXX".into(),
                values: vec![2.5e8],
                derived: false,
            }],
        ));
        convert_results(&mut mesh, &mut increments, UnitSystem::MmTonSC);
        assert_eq!(mesh.coords()[0], [1000.0, 500.0, 0.0]);
        let disp = increments[0].field("DISP").unwrap();
        assert!((disp.components[0].values[0] - 1.0).abs() < 1e-5);
        let stress = increments[0].field("STRESS").unwrap();
        assert!((stress.components[0].values[0] - 250.0).abs() < 1e-3);
    }

    #[test]
    fn flat_models_in_any_plane_and_lines_have_a_size() {
        assert_eq!(model_size(([0.0; 3], [2.0, 8.0, 4.0])), 4.0);
        // A membrane in the y-z plane, as in the x-y plane.
        assert_eq!(
            model_size(([0.0, -15.0, -40.0], [0.0, 15.0, 40.0])),
            2400f64.sqrt()
        );
        assert_eq!(
            model_size(([-15.0, -40.0, 0.0], [15.0, 40.0, 0.0])),
            2400f64.sqrt()
        );
        assert_eq!(model_size(([0.0; 3], [0.0, 0.0, 5.0])), 5.0);
    }

    #[test]
    fn automatic_scale_follows_model_size() {
        // Cube of size 10, largest displacement 0.1 in z: ALL and U3 both reach 0.1.
        let view = ResultsView::new(
            vec![increment(
                1,
                1,
                AnalysisKind::Static,
                &[[0.0, 0.0, 0.0], [0.0, 0.0, -0.1]],
            )],
            Some(([0.0; 3], [10.0; 3])),
        );
        assert_eq!(view.deformation, Deformation::Automatic(1.0));
        let max = (2.0f32 * 0.01).sqrt();
        let expected = round_significant(0.25 * 10.0 / max as f64, 2) as f32;
        assert_eq!(view.scale(), expected);
        assert_eq!(view.scale(), 18.0);
    }

    #[test]
    fn contact_opens_in_true_scale_except_for_mode_shapes() {
        let disp = [[0.0, 0.0, 1.0]];
        let mut contact = increment(1, 1, AnalysisKind::Static, &disp);
        let mut field = contact.fields[0].clone();
        field.name = "CONTACT".into();
        contact.fields.push(field);
        let mut view = ResultsView::new(
            vec![
                contact,
                increment(2, 1, AnalysisKind::Frequency, &disp),
                increment(3, 1, AnalysisKind::Static, &disp),
            ],
            None,
        );
        assert!(view.contact);
        assert_eq!(view.deformation, Deformation::TrueScale);
        view.select_increment(1);
        assert_eq!(view.deformation, Deformation::Automatic(1.0));
        view.select_increment(0);
        assert_eq!(view.deformation, Deformation::TrueScale);
        // A deformation the user picked stays for every step.
        (view.deformation, view.deformation_chosen) = (Deformation::Automatic(2.0), true);
        view.select_increment(1);
        assert_eq!(view.deformation, Deformation::Automatic(2.0));

        let without = ResultsView::new(vec![increment(1, 1, AnalysisKind::Static, &disp)], None);
        assert!(!without.contact);
        assert_eq!(without.deformation, Deformation::Automatic(1.0));
    }

    #[test]
    fn the_legend_shows_the_unit_of_the_results() {
        let mut view = ResultsView::new(
            vec![increment(1, 1, AnalysisKind::Static, &[[0.0, 0.0, 1.0]])],
            None,
        );
        assert!(view.legend().unwrap().title.contains("\nUnit: mm\n"));
        view.units = UnitSystem::InLbSF;
        assert!(view.legend().unwrap().title.contains("Unit: in"));
        view.units = UnitSystem::Unitless;
        assert!(!view.legend().unwrap().title.contains("Unit"));
        let units = UnitSystem::MTonSC;
        assert_eq!(field_unit("STRESS", "MISES", units).as_deref(), Some("kPa"));
        assert_eq!(field_unit("FORC", "F1", units).as_deref(), Some("kN"));
        assert_eq!(field_unit("ENER", "ENER", units).as_deref(), Some("kJ/m³"));
        assert_eq!(
            field_unit("CONTACT", "CPRESS", units).as_deref(),
            Some("kPa")
        );
        assert_eq!(field_unit("TOSTRAIN", "EXX", units), None);
    }

    /// The Step and Increment tiles: each increment in its step, named after the step's
    /// kind, and the steps before and after.
    #[test]
    fn increments_are_shown_within_their_steps() {
        use plx_results::steps::{StepCategory, StepRecord};
        let still = [[0.0; 3]];
        let mut view = ResultsView::new(
            vec![
                increment(1, 1, AnalysisKind::Static, &still),
                increment(1, 2, AnalysisKind::Static, &still),
                increment(2, 1, AnalysisKind::Frequency, &still),
                increment(2, 2, AnalysisKind::Frequency, &still),
                increment(2, 3, AnalysisKind::Frequency, &still),
            ],
            None,
        );
        let record = |name: &str, category, steps: &[u32]| StepRecord {
            name: name.into(),
            category,
            nlgeom: false,
            perturbation: false,
            time_period: None,
            solver_steps: steps.to_vec(),
            inherited_from: None,
        };
        view.set_step_records(
            &[
                record("Preload", StepCategory::Static, &[1]),
                record("Modes", StepCategory::Frequency, &[2]),
            ],
            &[],
            None,
        );
        view.select_increment(3);
        assert_eq!(view.step_title(1), "2 Modes");
        assert_eq!(view.entry_value().as_deref(), Some("2 of 3"));
        assert_eq!(view.entry_label(3), "Mode 2");
        assert_eq!(view.shown_label(), "Modes, mode 2");
        assert_eq!(view.neighbour_step(true), None);
        assert_eq!(view.neighbour_step(false), Some(0));
        // A step opens at its last increment.
        view.switch_step(0);
        assert_eq!(view.increment, 1);
        assert_eq!(view.entry_value().as_deref(), Some("2 of 2"));
        let lines = view.status_lines("r.ndlr");
        assert_eq!(lines[1], "Step 1 of 2: Preload (Static)");
        assert!(lines[2].starts_with("Increment 2 of 2"), "{lines:?}");
    }

    #[test]
    fn opens_last_increment_but_first_mode() {
        let still = [[0.0; 3]];
        let static_view = ResultsView::new(
            vec![
                increment(1, 1, AnalysisKind::Static, &still),
                increment(1, 2, AnalysisKind::Static, &still),
            ],
            None,
        );
        assert_eq!(static_view.increment, 1);
        let modes = ResultsView::new(
            vec![
                increment(1, 1, AnalysisKind::Static, &still),
                increment(2, 1, AnalysisKind::Frequency, &still),
                increment(2, 2, AnalysisKind::Frequency, &still),
            ],
            None,
        );
        assert_eq!(modes.increment, 1);
        assert_eq!(modes.current().unwrap().1.name, "ALL");
        // A buckle step opens at its first mode, after the reference state.
        let buckling = ResultsView::new(
            vec![
                increment(1, 0, AnalysisKind::Buckling, &still),
                increment(1, 1, AnalysisKind::Buckling, &still),
                increment(1, 2, AnalysisKind::Buckling, &still),
            ],
            None,
        );
        assert_eq!(buckling.increment, 1);
        let lines = buckling.status_lines("beulen.frd");
        assert_eq!(lines[1], "Step 1 of 1: Step 1 (Buckle)");
        assert!(
            lines[2].contains("Mode 1 of 2   Buckling factor: 1"),
            "{lines:?}"
        );
    }

    /// A complex mode: one node whirling in the y-z plane with magnitude 2, the real part
    /// (phase 0) pointing in y.
    fn complex_mode() -> Increment {
        let mut increment = increment(3, 1, AnalysisKind::ComplexFrequency, &[[0.0, 2.0, 0.0]]);
        let column = |name: &str, value: f32| Component {
            name: name.into(),
            values: vec![value],
            derived: false,
        };
        increment.fields.push(Field::new(
            "PDISP",
            vec![
                column("MAG1", 0.0),
                column("MAG2", 2.0),
                column("MAG3", 2.0),
                column("PHA1", 0.0),
                column("PHA2", 0.0),
                column("PHA3", -90.0),
            ],
        ));
        increment
    }

    #[test]
    fn plain_parts_stay_out_of_the_legend_range_and_the_extremes() {
        let disp = [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [9.0, 0.0, 0.0]];
        let mut view = ResultsView::new(
            vec![increment(1, 1, AnalysisKind::Static, &disp)],
            Some(([0.0; 3], [10.0; 3])),
        );
        view.component = 0; // U1
        let range = |v: &ResultsView| v.legend().map(|l| (l.min, l.max));
        assert_eq!(range(&view), Some((1.0, 9.0)));
        view.plain_nodes = vec![false, false, true];
        assert_eq!(range(&view), Some((1.0, 2.0)));
        assert_eq!(view.maximum().map(|m| m.1), Some(2.0));
        assert!(view.shown_values().unwrap()[2].is_nan(), "drawn grey");
    }

    #[test]
    fn complex_modes_whirl_with_magnitude_and_phase() {
        let mut view = ResultsView::new(vec![complex_mode()], Some(([0.0; 3], [10.0; 3])));
        assert_eq!(view.increment, 0, "the first mode opens");
        view.field = 0;
        view.component = 0; // ALL
        assert!(view.whirl().is_none());
        view.start_animation(AnimationKind::ScaleFactor);
        let animation = view.animation.as_mut().unwrap();
        animation.frames = 5;
        animation.go_to(0);
        let close = |a: [f32; 3], b: [f32; 3]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-5);
        assert!(close(
            view.shown_displacements().unwrap()[0],
            [0.0, 2.0, 0.0]
        ));
        assert_eq!(view.amplitude(), 1.0);
        // A quarter turn later the node has moved to z: PHA3 = -90 degrees.
        view.animation.as_mut().unwrap().go_to(1);
        assert!(close(
            view.shown_displacements().unwrap()[0],
            [0.0, 0.0, 2.0]
        ));
        assert_eq!(
            view.shown_values().unwrap()[0],
            2.0,
            "ALL stays the magnitude"
        );
        view.animation.as_mut().unwrap().limits = ColorLimits::AllFrames;
        let legend = view.legend().unwrap();
        assert!((legend.max - 8.0_f32.sqrt()).abs() < 1e-5, "{}", legend.max);
        view.component = 2; // U2
        assert!((view.shown_values().unwrap()[0]).abs() < 1e-5);
        assert_eq!(view.legend().map(|l| (l.min, l.max)), Some((-2.0, 2.0)));
        // The automatic scale takes the magnitudes, not the real part alone.
        view.deformation = Deformation::Automatic(1.0);
        assert_eq!(
            view.scale(),
            round_significant(0.25 * 10.0 / 8.0_f64.sqrt(), 2) as f32
        );
    }

    #[test]
    fn switching_increment_keeps_the_animation_running() {
        let still = [[0.0; 3]];
        let mut view = ResultsView::new(
            vec![
                increment(1, 1, AnalysisKind::Static, &still),
                increment(1, 2, AnalysisKind::Static, &still),
                increment(2, 1, AnalysisKind::Static, &still),
            ],
            Some(([0.0; 3], [10.0; 3])),
        );
        view.select_increment(0);
        view.start_animation(AnimationKind::Step);
        view.animation.as_mut().unwrap().fps = 7.0;
        view.animation.as_mut().unwrap().play();
        view.switch_increment(2);
        let animation = view.animation.as_ref().unwrap();
        assert_eq!(animation.increments, [2]);
        assert_eq!(animation.start_increment, 2);
        assert_eq!((animation.fps, animation.playing), (7.0, true));
        assert_eq!(view.increment, 2);
        view.switch_increment(0);
        assert_eq!(view.animation.as_ref().unwrap().increments, [0, 1]);
        assert_eq!(view.animation.as_ref().unwrap().frame, 0);
    }

    #[test]
    fn animations_play_a_step_all_steps_or_the_positions_of_a_moving_load() {
        let still = [[0.0; 3]];
        let mut view = ResultsView::new(
            vec![
                increment(1, 1, AnalysisKind::Static, &still),
                increment(1, 2, AnalysisKind::Static, &still),
                increment(2, 1, AnalysisKind::Frequency, &still),
                increment(3, 1, AnalysisKind::Static, &still),
                increment(4, 1, AnalysisKind::Static, &still),
            ],
            Some(([0.0; 3], [10.0; 3])),
        );
        view.select_increment(3);
        // All timed steps, without the modes of the frequency step.
        view.start_animation(AnimationKind::AllSteps);
        let animation = view.animation.as_ref().unwrap();
        assert_eq!(animation.increments, [0, 1, 3, 4]);
        assert_eq!(animation.frame, 2);
        view.start_animation(AnimationKind::Step);
        assert_eq!(view.animation.as_ref().unwrap().increments, [3]);
        // A moving load: one step of three solver steps, one position each.
        use plx_results::steps::{StepCategory, StepRecord};
        let record = |name: &str, category, steps: &[u32]| StepRecord {
            name: name.into(),
            category,
            nlgeom: false,
            perturbation: false,
            time_period: None,
            solver_steps: steps.to_vec(),
            inherited_from: None,
        };
        view.set_step_records(
            &[
                record("Preload", StepCategory::Static, &[1]),
                record("Modes", StepCategory::Frequency, &[2]),
                record("Rolling", StepCategory::Static, &[3, 4]),
            ],
            &[],
            None,
        );
        view.select_increment(3);
        view.start_animation(AnimationKind::Positions);
        assert_eq!(view.animation.as_ref().unwrap().increments, [3, 4]);
    }

    #[test]
    fn manual_limits_colour_values_beyond_them_apart() {
        let mut view = ResultsView::new(
            vec![increment(
                1,
                1,
                AnalysisKind::Static,
                &[[0.0, 0.0, 1.0], [0.0, 0.0, 2.0], [0.0, 0.0, 3.0]],
            )],
            Some(([0.0; 3], [10.0; 3])),
        );
        view.component = 3; // U3
        let legend = view.legend().unwrap();
        assert!(legend.title.ends_with("Automatic"));
        assert!(legend.below.is_none() && legend.above.is_none());
        assert_eq!(legend.normalize(&[1.0, 3.0]), [0.0, 1.0]);

        view.color_scale.levels = 4;
        view.color_scale.manual = Some([1.5, 2.5]);
        let legend = view.legend().unwrap();
        assert!(legend.title.ends_with("Manual"));
        assert_eq!((legend.min, legend.max), (1.5, 2.5));
        assert_eq!(legend.below.as_ref().map(|o| o.value), Some(1.0));
        assert_eq!(legend.above.as_ref().map(|o| o.value), Some(3.0));
        // One band of 0.25 for each out-of-range colour: six bands from 1.25 to 2.75.
        assert_eq!(legend.color_range(), (1.25, 2.75));
        let palette = legend.palette();
        assert_eq!(palette.len(), 6);
        assert_eq!(
            palette[0],
            plx_render::contour::rgb(view.color_scale.min_color)
        );
        assert_eq!(
            palette[5],
            plx_render::contour::rgb(view.color_scale.max_color)
        );
        let band = |v: f32| (legend.normalize(&[v])[0] * 6.0).floor().min(5.0) as usize;
        assert_eq!([band(1.0), band(1.6), band(2.4), band(3.0)], [0, 1, 4, 5]);

        // A range covering all values needs no out-of-range colours.
        view.color_scale.manual = Some([0.0, 5.0]);
        let legend = view.legend().unwrap();
        assert!(legend.below.is_none() && legend.above.is_none());
        assert_eq!(legend.color_range(), (0.0, 5.0));
        // Invalid limits fall back to the automatic ones.
        view.color_scale.manual = Some([2.0, 2.0]);
        assert_eq!(view.legend().map(|l| (l.min, l.max)), Some((1.0, 3.0)));
    }

    #[test]
    fn animation_scales_values_and_sets_legend_limits() {
        let still = [[0.0; 3]];
        let mut view = ResultsView::new(
            vec![
                increment(1, 1, AnalysisKind::Static, &[[0.0, 0.0, 1.0]]),
                increment(1, 2, AnalysisKind::Static, &[[0.0, 0.0, 3.0]]),
                increment(2, 1, AnalysisKind::Static, &still),
            ],
            Some(([0.0; 3], [10.0; 3])),
        );
        view.select_increment(0);
        view.field = 0;
        view.component = 3; // U3
        view.start_animation(AnimationKind::Step);
        let animation = view.animation.as_mut().unwrap();
        assert_eq!(animation.increments, [0, 1]);
        animation.limits = ColorLimits::AllFrames;
        assert_eq!(view.legend().map(|l| (l.min, l.max)), Some((1.0, 3.0)));

        view.start_animation(AnimationKind::ScaleFactor);
        let animation = view.animation.as_mut().unwrap();
        animation.frames = 3;
        animation.go_to(1);
        assert_eq!(view.amplitude(), 0.5);
        assert_eq!(view.legend().map(|l| (l.min, l.max)), Some((0.5, 0.5)));
        view.animation.as_mut().unwrap().limits = ColorLimits::AllFrames;
        assert_eq!(view.legend().map(|l| (l.min, l.max)), Some((0.0, 1.0)));

        view.stop_animation();
        assert_eq!(view.increment, 0);
        assert_eq!(view.amplitude(), 1.0);
    }

    #[test]
    fn mode_shape_animation_swings_signed_values_but_not_magnitudes() {
        let mode = [[0.0, 0.0, 1.0], [0.0, 0.0, 3.0]];
        let mut view = ResultsView::new(
            vec![increment(1, 1, AnalysisKind::Frequency, &mode)],
            Some(([0.0; 3], [10.0; 3])),
        );
        view.component = 3; // U3
        view.start_animation(AnimationKind::ScaleFactor);
        let animation = view.animation.as_mut().unwrap();
        assert!(animation.modal);
        animation.go_to(0);
        assert_eq!(view.amplitude(), -1.0);
        assert_eq!(view.legend().map(|l| (l.min, l.max)), Some((-3.0, -1.0)));
        assert_eq!(view.maximum(), Some((0, -1.0, 0)));
        assert_eq!(view.minimum(), Some((1, -3.0, 0)));
        view.animation.as_mut().unwrap().limits = ColorLimits::AllFrames;
        assert_eq!(view.legend().map(|l| (l.min, l.max)), Some((-3.0, 3.0)));

        view.component = 0; // ALL stays positive
        assert_eq!(view.value_amplitude(), 1.0);
        assert_eq!(view.legend().map(|l| (l.min, l.max)), Some((0.0, 3.0)));
        view.animation.as_mut().unwrap().limits = ColorLimits::CurrentFrame;
        assert_eq!(view.legend().map(|l| (l.min, l.max)), Some((1.0, 3.0)));
    }

    #[test]
    fn values_are_formatted_with_four_significant_digits() {
        assert_eq!(format_value(123.456), "123.5");
        assert_eq!(format_value(0.012345), "0.01235");
        assert_eq!(format_value(-2.0), "-2");
        assert_eq!(format_value(123456.0), "1.235E5");
        assert_eq!(format_legend_value(1.5), "+1.5");
    }
}
