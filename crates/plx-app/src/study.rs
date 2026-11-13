//! The editor of a Parameter Study block of the project schematic: its columns and the table
//! of design points, like the parameter set of Ansys Workbench. It edits the block through
//! [`StudyEdit`]s, which the project window applies, so that the table follows a run while
//! the editor is open.

use crate::clicks::TolerantDoubleClick as _;
use plx_model::{FillMode, FillRule, PointState, UnitSystem, ValueKind};

use crate::numeric::format_physical;
use crate::table_export;

/// A change the editor asks for.
pub enum StudyEdit {
    SetInput {
        point: usize,
        column: usize,
        value: f64,
    },
    AddPoint,
    Duplicate(usize),
    RemovePoint(usize),
    SetCurrent(usize),
    RenameInput(usize, String),
    RenameOutput(usize, String),
    RemoveInput(usize),
    RemoveOutput(usize),
    /// Points with these inputs, in SI units, from the fill helper or an import; `replace`
    /// removes all others but the current one first.
    AddRows {
        rows: Vec<Vec<f64>>,
        replace: bool,
    },
    Run,
    Reset,
    Close,
}

/// A design point as the editor shows it.
pub struct PointView {
    /// The inputs in SI units, edited in place.
    pub inputs: Vec<f64>,
    /// The outputs in the units their sources show them in.
    pub outputs: Vec<Option<f64>>,
    pub state: PointState,
    /// How long it took, once solved.
    pub seconds: Option<f64>,
    pub elements: Option<u64>,
}

/// What the editor shows of a study, made by the project window, which knows the units of
/// the outputs.
#[derive(Default)]
pub struct StudyView {
    pub units: UnitSystem,
    /// Name and kind of each input.
    pub inputs: Vec<(String, Option<ValueKind>)>,
    /// Name and unit of each output.
    pub outputs: Vec<(String, String)>,
    pub points: Vec<PointView>,
    pub current: usize,
    /// The point being solved while the study runs.
    pub running: Option<usize>,
    /// The study or the project is being updated: the table cannot be changed.
    pub busy: bool,
}

impl StudyView {
    /// The inputs of the current point, in SI units.
    fn current_inputs(&self) -> Vec<f64> {
        (self.points.get(self.current).map(|p| p.inputs.clone()))
            .unwrap_or_else(|| vec![0.0; self.inputs.len()])
    }

    /// The unit of input `column` in the units shown, empty for a number without one.
    fn input_unit(&self, column: usize) -> &'static str {
        (self.inputs.get(column))
            .and_then(|(_, kind)| kind.and_then(ValueKind::quantity))
            .map_or("", |q| self.units.unit(q))
    }

    /// Input `column` of a point in the units shown.
    fn input_shown(&self, column: usize, value: f64) -> f64 {
        (self.inputs.get(column))
            .and_then(|(_, kind)| kind.and_then(ValueKind::quantity))
            .map_or(value, |q| self.units.shown(value, q))
    }

    /// A value typed for input `column` in the units shown (or with its own unit), in SI.
    fn input_parsed(&self, column: usize, text: &str) -> Result<f64, String> {
        let quantity =
            (self.inputs.get(column)).and_then(|(_, kind)| kind.and_then(ValueKind::quantity));
        match quantity {
            Some(q) => Ok(self.units.stored(self.units.parse_value(text, q)?, q)),
            None => crate::numeric::evaluate(text).ok_or_else(|| format!("Not a number: {text}")),
        }
    }
}

/// What the editor shows below its columns.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Tab {
    Table,
    Plot,
}

/// The horizontal axis of the plot.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Axis {
    Input(usize),
    Output(usize),
    Elements,
}

/// How the fill helper makes the values of one input, as typed.
#[derive(Clone, Copy, PartialEq, Eq)]
enum RuleKind {
    Current,
    List,
    Range,
}

struct RuleEdit {
    kind: RuleKind,
    /// The values of a list as typed, separated by `;`.
    list: String,
    /// The range, its ends in SI units.
    from: f64,
    to: f64,
    count: usize,
}

/// The fill helper: a rule per input and how to combine them.
struct Fill {
    rules: Vec<RuleEdit>,
    mode: FillMode,
    replace: bool,
}

impl Fill {
    fn new(view: &StudyView) -> Self {
        let current = view.current_inputs();
        let rules = (current.iter().enumerate())
            .map(|(column, &value)| RuleEdit {
                kind: RuleKind::Current,
                list: format_physical(view.input_shown(column, value)),
                from: value,
                to: if value == 0.0 { 1.0 } else { 2.0 * value },
                count: 3,
            })
            .collect();
        Self {
            rules,
            mode: FillMode::Grid,
            replace: false,
        }
    }

    /// The rules, or why a list cannot be read.
    fn rules(&self, view: &StudyView) -> Result<Vec<FillRule>, String> {
        (self.rules.iter().enumerate())
            .map(|(column, rule)| match rule.kind {
                RuleKind::Current => Ok(FillRule::Current),
                RuleKind::Range => Ok(FillRule::Range {
                    from: rule.from,
                    to: rule.to,
                    count: rule.count,
                }),
                RuleKind::List => (rule.list.split(';'))
                    .map(str::trim)
                    .filter(|t| !t.is_empty())
                    .map(|t| view.input_parsed(column, t))
                    .collect::<Result<Vec<_>, _>>()
                    .map(FillRule::List),
            })
            .collect()
    }
}

/// The import helper: points pasted or read from a file.
struct Import {
    text: String,
    replace: bool,
}

/// The editor of a Parameter Study block.
pub struct StudyEditor {
    pub id: u64,
    name: String,
    /// The column being renamed: input or output, its index and the name typed so far.
    renaming: Option<(bool, usize, String)>,
    tab: Tab,
    fill: Option<Fill>,
    import: Option<Import>,
    /// The axes of the plot: the horizontal one, the outputs shown and the input that
    /// splits the points into curves.
    x: Axis,
    shown: Vec<bool>,
    split: Option<usize>,
    /// What the last export or import did.
    message: Option<String>,
    /// A helper was just opened: it goes on top of the editor in the next frames, as the
    /// click that opened it raises the editor.
    raise: u8,
}

impl StudyEditor {
    pub fn new(id: u64, name: &str) -> Self {
        Self {
            id,
            name: name.to_owned(),
            renaming: None,
            tab: Tab::Table,
            fill: None,
            import: None,
            x: Axis::Input(0),
            shown: Vec::new(),
            split: None,
            message: None,
            raise: 0,
        }
    }

    pub fn rename(&mut self, name: &str) {
        name.clone_into(&mut self.name);
    }

    pub fn show(&mut self, ctx: &egui::Context, view: &StudyView) -> Vec<StudyEdit> {
        let mut edits = Vec::new();
        let mut open = true;
        egui::Window::new(format!("Parameter Study: {}", self.name))
            .id(crate::ids::window(ctx, ("study", self.id)))
            .open(&mut open)
            .collapsible(false)
            .resizable(true)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .default_width(640.0)
            .show(ctx, |ui| {
                if view.inputs.is_empty() || view.outputs.is_empty() {
                    ui.weak(
                        "Drag the Inputs pin of the block to the exposed parameters of \
                         analyses, the numbers of mesh blocks or the Factor of a Refine Mesh \
                         block, and the value of a Measure block to its Outputs pin: each \
                         link is a column of the table.",
                    );
                }
                self.columns(ui, view, &mut edits);
                ui.separator();
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.tab, Tab::Table, "Table");
                    ui.selectable_value(&mut self.tab, Tab::Plot, "Plot");
                    ui.separator();
                    self.table_tools(ui, view);
                });
                if let Some(message) = &self.message {
                    ui.weak(message);
                }
                match self.tab {
                    Tab::Table => self.table(ui, view, &mut edits),
                    Tab::Plot => self.plot(ui, view),
                }
                ui.separator();
                ui.horizontal(|ui| {
                    let idle = !view.busy;
                    if ui
                        .add_enabled(idle, egui::Button::new("Add Point"))
                        .on_hover_text("Adds a point with the inputs of the current one.")
                        .clicked()
                    {
                        edits.push(StudyEdit::AddPoint);
                    }
                    let run = ui.add_enabled(idle, egui::Button::new("Run"));
                    if run
                        .on_hover_text(
                            "Solves the points that are not up to date, one after the other, \
                             and the current one last.",
                        )
                        .clicked()
                    {
                        edits.push(StudyEdit::Run);
                    }
                    let reset = ui.add_enabled(idle, egui::Button::new("Reset"));
                    if reset
                        .on_hover_text("Forgets the outputs of all points; the inputs stay.")
                        .clicked()
                    {
                        edits.push(StudyEdit::Reset);
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("Close").clicked() {
                            edits.push(StudyEdit::Close);
                        }
                    });
                });
            });
        if !open {
            edits.push(StudyEdit::Close);
        }
        self.fill_window(ctx, view, &mut edits);
        self.import_window(ctx, view, &mut edits);
        edits
    }

    /// Fill, import, copy and export of the table.
    fn table_tools(&mut self, ui: &mut egui::Ui, view: &StudyView) {
        let idle = !view.busy && !view.inputs.is_empty();
        if ui
            .add_enabled(idle, egui::Button::new("Fill …"))
            .on_hover_text("Adds points from a list or a range of values per input.")
            .clicked()
        {
            self.fill = Some(Fill::new(view));
            self.raise = 3;
        }
        if ui
            .add_enabled(idle, egui::Button::new("Import …"))
            .on_hover_text("Adds points from a table pasted or read from a CSV file.")
            .clicked()
        {
            self.raise = 3;
            self.import = Some(Import {
                text: String::new(),
                replace: false,
            });
        }
        let some = !view.points.is_empty();
        if ui
            .add_enabled(some, egui::Button::new("Copy Table"))
            .on_hover_text("Copies the table with tabs, for a spreadsheet.")
            .clicked()
        {
            ui.ctx().copy_text(table_text(view, '\t'));
            self.message = Some("Table copied.".into());
        }
        for (label, format) in [
            ("Export CSV …", table_export::Format::Csv),
            ("Export JSON …", table_export::Format::Json),
        ] {
            if ui.add_enabled(some, egui::Button::new(label)).clicked() {
                self.export(view, format);
            }
        }
    }

    fn export(&mut self, view: &StudyView, format: table_export::Format) {
        let Some(path) = table_export::save_dialog("Export Parameter Study", &self.name, format)
        else {
            return;
        };
        let text = table_text(view, ',');
        self.message = Some(match table_export::write(&path, &text, ',', format) {
            Ok(()) => format!("Exported: {}", path.display()),
            Err(err) => format!("Export failed ({}): {err}", path.display()),
        });
    }

    /// The fill helper, while open.
    fn fill_window(&mut self, ctx: &egui::Context, view: &StudyView, edits: &mut Vec<StudyEdit>) {
        let Some(fill) = &mut self.fill else {
            return;
        };
        let id = crate::ids::window(ctx, ("study fill", self.id));
        if self.raise > 0 {
            self.raise -= 1;
            ctx.request_repaint();
            ctx.move_to_top(egui::LayerId::new(egui::Order::Middle, id));
        }
        if fill.rules.len() != view.inputs.len() {
            *fill = Fill::new(view);
        }
        let mut open = true;
        let mut done = false;
        egui::Window::new(format!("Fill: {}", self.name))
            .id(id)
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                ui.weak(
                    "Per input: the value of the current point, a list of values separated \
                     by ; or a range.",
                );
                egui::Grid::new("study fill rules")
                    .num_columns(3)
                    .spacing([10.0, 4.0])
                    .show(ui, |ui| {
                        for (column, rule) in fill.rules.iter_mut().enumerate() {
                            let (name, kind) = &view.inputs[column];
                            ui.label(format!("P{} {name}", column + 1));
                            egui::ComboBox::from_id_salt(("fill rule", column))
                                .selected_text(match rule.kind {
                                    RuleKind::Current => "Current",
                                    RuleKind::List => "List",
                                    RuleKind::Range => "Range",
                                })
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(
                                        &mut rule.kind,
                                        RuleKind::Current,
                                        "Current",
                                    );
                                    ui.selectable_value(&mut rule.kind, RuleKind::List, "List");
                                    ui.selectable_value(&mut rule.kind, RuleKind::Range, "Range");
                                });
                            ui.horizontal(|ui| match rule.kind {
                                RuleKind::Current => {
                                    let value = view.current_inputs()[column];
                                    let unit = view.input_unit(column);
                                    ui.weak(format!(
                                        "{} {unit}",
                                        format_physical(view.input_shown(column, value))
                                    ));
                                }
                                RuleKind::List => {
                                    ui.add(
                                        egui::TextEdit::singleline(&mut rule.list)
                                            .desired_width(220.0)
                                            .hint_text("1; 1.5; 2"),
                                    );
                                    ui.weak(view.input_unit(column));
                                }
                                RuleKind::Range => {
                                    ui.label("from");
                                    ui.add(number_field(&mut rule.from, *kind, view.units));
                                    ui.label("to");
                                    ui.add(number_field(&mut rule.to, *kind, view.units));
                                    ui.weak(view.input_unit(column));
                                    ui.label("values");
                                    ui.add(crate::numeric::count(&mut rule.count).range(1..=1000));
                                }
                            });
                            ui.end_row();
                        }
                    });
                ui.separator();
                crate::layout::wrapped_row(ui, |ui| {
                    ui.radio_value(&mut fill.mode, FillMode::Grid, "All combinations")
                        .on_hover_text("A point for every combination of the values.");
                    ui.radio_value(&mut fill.mode, FillMode::OneAtATime, "One at a time")
                        .on_hover_text(
                            "Each input varied by itself, the others at the current point.",
                        );
                });
                ui.checkbox(&mut fill.replace, "Replace the points")
                    .on_hover_text("Removes all points but the current one first.");
                let mut new = 0;
                let rows = fill
                    .rules(view)
                    .map(|rules| fill_rows(view, &rules, fill.mode));
                match &rows {
                    Ok(rows) => {
                        let (count, text) = new_points(view, rows, fill.replace);
                        new = count;
                        ui.label(text);
                    }
                    Err(why) => {
                        ui.colored_label(crate::style::error(), why);
                    }
                }
                let ready = new > 0 && !view.busy;
                match crate::dialog::work_footer(ui, Some(("Add Points", ready))) {
                    crate::dialog::Work::Verb => {
                        if let Ok(rows) = rows {
                            edits.push(StudyEdit::AddRows {
                                rows,
                                replace: fill.replace,
                            });
                        }
                        done = true;
                    }
                    crate::dialog::Work::Close => done = true,
                    crate::dialog::Work::None => {}
                }
            });
        if !open || done {
            self.fill = None;
        }
    }

    /// The import helper, while open.
    fn import_window(&mut self, ctx: &egui::Context, view: &StudyView, edits: &mut Vec<StudyEdit>) {
        let Some(import) = &mut self.import else {
            return;
        };
        let id = crate::ids::window(ctx, ("study import", self.id));
        if self.raise > 0 {
            self.raise -= 1;
            ctx.request_repaint();
            ctx.move_to_top(egui::LayerId::new(egui::Order::Middle, id));
        }
        let mut open = true;
        let mut done = false;
        egui::Window::new(format!("Import: {}", self.name))
            .id(id)
            .open(&mut open)
            .collapsible(false)
            .resizable(true)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .default_width(480.0)
            .show(ctx, |ui| {
                ui.weak(
                    "Paste a table, for example from a spreadsheet, or open a CSV file. A first \
                     row of names takes the columns named after inputs; without one, the \
                     columns are the inputs in order. Values are in the units shown.",
                );
                if ui.button("Open File …").clicked()
                    && let Some(path) = rfd::FileDialog::new()
                        .set_title("Import Parameter Study")
                        .add_filter("CSV file", &["csv", "txt", "tsv"])
                        .pick_file()
                {
                    match std::fs::read_to_string(&path) {
                        Ok(text) => import.text = text,
                        Err(err) => import.text = format!("# {err}"),
                    }
                }
                egui::ScrollArea::vertical()
                    .max_height(240.0)
                    .show(ui, |ui| {
                        ui.add(
                            egui::TextEdit::multiline(&mut import.text)
                                .code_editor()
                                .desired_rows(8)
                                .desired_width(f32::INFINITY),
                        );
                    });
                ui.checkbox(&mut import.replace, "Replace the points")
                    .on_hover_text("Removes all points but the current one first.");
                let mut new = 0;
                let rows = parse_table(view, &import.text);
                match &rows {
                    Ok(rows) => {
                        let (count, text) = new_points(view, rows, import.replace);
                        new = count;
                        ui.label(text);
                    }
                    Err(why) => {
                        ui.colored_label(crate::style::error(), why);
                    }
                }
                let ready = new > 0 && !view.busy;
                match crate::dialog::work_footer(ui, Some(("Add Points", ready))) {
                    crate::dialog::Work::Verb => {
                        if let Ok(rows) = rows {
                            edits.push(StudyEdit::AddRows {
                                rows,
                                replace: import.replace,
                            });
                        }
                        done = true;
                    }
                    crate::dialog::Work::Close => done = true,
                    crate::dialog::Work::None => {}
                }
            });
        if !open || done {
            self.import = None;
        }
    }

    /// The outputs over an input, another output or the elements, a curve per value of an
    /// input that splits the points.
    fn plot(&mut self, ui: &mut egui::Ui, view: &StudyView) {
        use egui_plot::{Legend, Line, Plot, Points};
        if view.outputs.is_empty() {
            ui.weak("No outputs to plot yet.");
            return;
        }
        self.shown.resize(view.outputs.len(), false);
        if !self.shown.contains(&true) {
            self.shown[0] = true;
        }
        let axis_name = |axis: Axis| match axis {
            Axis::Input(i) => view
                .inputs
                .get(i)
                .map_or(String::new(), |(n, _)| format!("P{} {n}", i + 1)),
            Axis::Output(i) => view
                .outputs
                .get(i)
                .map_or(String::new(), |(n, _)| format!("O{} {n}", i + 1)),
            Axis::Elements => "Elements".into(),
        };
        let valid = |axis: Axis| match axis {
            Axis::Input(i) => i < view.inputs.len(),
            Axis::Output(i) => i < view.outputs.len(),
            Axis::Elements => true,
        };
        if !valid(self.x) {
            self.x = if view.inputs.is_empty() {
                Axis::Elements
            } else {
                Axis::Input(0)
            };
        }
        if self.split.is_some_and(|s| s >= view.inputs.len()) {
            self.split = None;
        }
        ui.horizontal(|ui| {
            ui.label("x");
            egui::ComboBox::from_id_salt("study plot x")
                .selected_text(axis_name(self.x))
                .show_ui(ui, |ui| {
                    let axes = (0..view.inputs.len())
                        .map(Axis::Input)
                        .chain((0..view.outputs.len()).map(Axis::Output))
                        .chain([Axis::Elements]);
                    for axis in axes {
                        ui.selectable_value(&mut self.x, axis, axis_name(axis));
                    }
                });
            ui.label("y");
            for (i, shown) in self.shown.iter_mut().enumerate() {
                ui.checkbox(shown, format!("O{}", i + 1))
                    .on_hover_text(&view.outputs[i].0);
            }
            if view.inputs.len() > 1 {
                ui.label("Curve per");
                egui::ComboBox::from_id_salt("study plot split")
                    .selected_text(
                        self.split
                            .map_or("-".to_owned(), |i| axis_name(Axis::Input(i))),
                    )
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.split, None, "-");
                        for i in 0..view.inputs.len() {
                            ui.selectable_value(
                                &mut self.split,
                                Some(i),
                                axis_name(Axis::Input(i)),
                            );
                        }
                    });
            }
        });
        let unit = |axis: Axis| match axis {
            Axis::Input(i) => view.input_unit(i).to_owned(),
            Axis::Output(i) => view.outputs[i].1.clone(),
            Axis::Elements => String::new(),
        };
        let label = |axis: Axis| match unit(axis) {
            u if u.is_empty() => axis_name(axis),
            u => format!("{} [{u}]", axis_name(axis)),
        };
        let y_label = match (0..view.outputs.len())
            .filter(|&i| self.shown[i])
            .collect::<Vec<_>>()[..]
        {
            [one] => label(Axis::Output(one)),
            _ => String::new(),
        };
        let curves = plot_curves(view, self.x, &self.shown, self.split);
        Plot::new(("study plot", self.id))
            .height(300.0)
            .legend(Legend::default().follow_insertion_order(true))
            .x_axis_label(label(self.x))
            .y_axis_label(y_label)
            .grid_color(crate::style::plot_grid())
            .show(ui, |plot_ui| {
                for (index, (name, points)) in curves.iter().enumerate() {
                    let color = crate::xy_plot::color(index);
                    plot_ui.line(
                        Line::new(name.as_str(), points.clone())
                            .color(color)
                            .width(2.0),
                    );
                    plot_ui.points(
                        Points::new(name.as_str(), points.clone())
                            .color(color)
                            .radius(3.0),
                    );
                }
            });
    }

    /// The inputs and outputs, each with its kind or unit, to rename or remove.
    fn columns(&mut self, ui: &mut egui::Ui, view: &StudyView, edits: &mut Vec<StudyEdit>) {
        let rows = (view.inputs.iter().map(|(name, kind)| {
            let kind = kind.map_or("number".to_owned(), |k| match k.quantity() {
                Some(q) => format!("{} [{}]", k.label(), view.units.unit(q)),
                None => k.label().to_owned(),
            });
            (true, name, kind)
        }))
        .chain(
            view.outputs
                .iter()
                .map(|(name, unit)| (false, name, unit.clone())),
        );
        egui::Grid::new("study columns")
            .num_columns(4)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                let (mut input, mut output) = (0, 0);
                for (is_input, name, kind) in rows {
                    let index = if is_input { &mut input } else { &mut output };
                    let at = *index;
                    *index += 1;
                    let id = if is_input {
                        format!("P{}", at + 1)
                    } else {
                        format!("O{}", at + 1)
                    };
                    ui.label(&id).on_hover_text(if is_input {
                        "An input: the study gives its value through the links of its row."
                    } else {
                        "An output: the study reads it from the block linked to its row."
                    });
                    match &mut self.renaming {
                        Some((i, a, text)) if *i == is_input && *a == at => {
                            let response =
                                ui.add(egui::TextEdit::singleline(text).desired_width(180.0));
                            if response.lost_focus() {
                                let text = text.trim().to_owned();
                                if !text.is_empty() && &text != name {
                                    edits.push(if is_input {
                                        StudyEdit::RenameInput(at, text)
                                    } else {
                                        StudyEdit::RenameOutput(at, text)
                                    });
                                }
                                self.renaming = None;
                            } else {
                                response.request_focus();
                            }
                        }
                        _ => {
                            let label = ui.add(egui::Label::new(name).sense(egui::Sense::click()));
                            if label
                                .on_hover_text("Double-click to rename")
                                .double_clicked_tolerant()
                            {
                                self.renaming = Some((is_input, at, name.clone()));
                            }
                        }
                    }
                    ui.weak(kind);
                    let remove = ui.add_enabled(!view.busy, egui::Button::new("Remove").small());
                    if remove
                        .on_hover_text("Removes the column and its links.")
                        .clicked()
                    {
                        edits.push(if is_input {
                            StudyEdit::RemoveInput(at)
                        } else {
                            StudyEdit::RemoveOutput(at)
                        });
                    }
                    ui.end_row();
                }
            });
    }

    /// The design points: inputs to edit, outputs, state, time and elements.
    fn table(&mut self, ui: &mut egui::Ui, view: &StudyView, edits: &mut Vec<StudyEdit>) {
        if view.points.is_empty() {
            ui.weak("No design points yet: link an input or an output first.");
            return;
        }
        egui::ScrollArea::both()
            .max_height(320.0)
            .auto_shrink([false, true])
            .show(ui, |ui| {
                egui::Grid::new("study points")
                    .striped(true)
                    .spacing([12.0, 3.0])
                    .show(ui, |ui| {
                        ui.strong("Point");
                        ui.strong("State");
                        for (i, (name, _)) in view.inputs.iter().enumerate() {
                            ui.strong(with_unit(&format!("P{} {name}", i + 1), view.input_unit(i)));
                        }
                        for (i, (name, unit)) in view.outputs.iter().enumerate() {
                            ui.strong(with_unit(&format!("O{} {name}", i + 1), unit));
                        }
                        ui.strong("Time");
                        ui.strong("Elements");
                        ui.label("");
                        ui.end_row();
                        for (index, point) in view.points.iter().enumerate() {
                            self.point_row(ui, view, index, point, edits);
                            ui.end_row();
                        }
                    });
            });
    }

    fn point_row(
        &mut self,
        ui: &mut egui::Ui,
        view: &StudyView,
        index: usize,
        point: &PointView,
        edits: &mut Vec<StudyEdit>,
    ) {
        let current = index == view.current;
        let label = if current {
            format!("DP {index} (current)")
        } else {
            format!("DP {index}")
        };
        let label = if current {
            egui::RichText::new(label).strong()
        } else {
            egui::RichText::new(label)
        };
        ui.label(label).on_hover_text(if current {
            "The current point: the blocks stand at its inputs."
        } else {
            "A design point; right-click its state to make it the current one."
        });
        let state = if view.running == Some(index) {
            "running".to_owned()
        } else {
            point.state.label().to_owned()
        };
        let color = match point.state {
            _ if view.running == Some(index) => ui.visuals().text_color(),
            PointState::UpToDate => crate::style::adapt(egui::Color32::from_rgb(40, 150, 60)),
            PointState::Failed => crate::style::error(),
            PointState::OutOfDate | PointState::NotSolved => {
                crate::style::adapt(egui::Color32::from_rgb(200, 140, 0))
            }
        };
        let state = ui.add(
            egui::Label::new(egui::RichText::new(state).color(color)).sense(egui::Sense::click()),
        );
        let tip = match point.state {
            PointState::Failed => {
                "A solve or a measure gave no value: the solution monitor and the output tell why."
            }
            PointState::OutOfDate => {
                "Solved, but the blocks it runs, its inputs or the outputs changed since."
            }
            _ => "Right-click for more.",
        };
        state.on_hover_text(tip).context_menu(|ui| {
            if ui
                .add_enabled(!view.busy && !current, egui::Button::new("Set as Current"))
                .on_hover_text(
                    "The blocks take its inputs; Update Project then solves them, so the \
                     analyses show its results.",
                )
                .clicked()
            {
                edits.push(StudyEdit::SetCurrent(index));
                ui.close();
            }
            if ui
                .add_enabled(!view.busy, egui::Button::new("Duplicate"))
                .clicked()
            {
                edits.push(StudyEdit::Duplicate(index));
                ui.close();
            }
            if ui
                .add_enabled(!view.busy && !current, egui::Button::new("Delete"))
                .clicked()
            {
                edits.push(StudyEdit::RemovePoint(index));
                ui.close();
            }
        });
        for (column, (&value, (_, kind))) in point.inputs.iter().zip(&view.inputs).enumerate() {
            let mut value = value;
            let field = number_field(&mut value, *kind, view.units);
            let changed = ui.add_enabled(!view.busy, field).changed();
            if changed {
                edits.push(StudyEdit::SetInput {
                    point: index,
                    column,
                    value,
                });
            }
        }
        for output in &point.outputs {
            ui.label(output.map_or(String::new(), shown_number));
        }
        ui.label(point.seconds.map_or(String::new(), |s| format!("{s:.1} s")));
        ui.label(point.elements.map_or(String::new(), |e| e.to_string()));
        ui.label("");
    }
}

/// A value of a study as text, in `units` for a quantity.
pub fn shown(kind: ValueKind, value: f64, units: UnitSystem) -> String {
    match kind {
        ValueKind::Quantity(quantity) => {
            let shown = units.shown(value, quantity);
            let unit = units.unit(quantity);
            format!("{} {unit}", crate::results::format_value(shown as f32))
        }
        ValueKind::Count => format!("{value:.0}"),
        ValueKind::Factor => crate::results::format_value(value as f32),
        ValueKind::Derived(dimension) => {
            format!(
                "{} {}",
                crate::results::format_value(value as f32),
                dimension.si()
            )
        }
    }
}

/// The field of an input, stored in SI units and shown without its unit, which the column
/// title names.
fn number_field(
    value: &mut f64,
    kind: Option<ValueKind>,
    units: UnitSystem,
) -> egui::DragValue<'_> {
    match kind {
        Some(kind) => match kind.quantity() {
            Some(quantity) => crate::numeric::without_unit(value, units, quantity),
            None => crate::numeric::of_kind(value, kind, units),
        },
        None => crate::numeric::drag_value(value),
    }
}

/// How many of `rows` are new points: the others are there already, or are the current
/// point when the points are replaced.
fn new_points(view: &StudyView, rows: &[Vec<f64>], replace: bool) -> (usize, String) {
    let current = view.current_inputs();
    let new = (rows.iter())
        .filter(|row| match replace {
            true => **row != current,
            false => !view.points.iter().any(|p| p.inputs == **row),
        })
        .count();
    let points = |n: usize| match n {
        1 => "1 new point".to_owned(),
        n => format!("{n} new points"),
    };
    let text = match rows.len() - new {
        0 => points(new),
        there => format!("{}, {there} there already", points(new)),
    };
    (new, text)
}

/// `name [unit]`, or the name alone.
fn with_unit(name: &str, unit: &str) -> String {
    if unit.is_empty() {
        name.to_owned()
    } else {
        format!("{name} [{unit}]")
    }
}

/// A number of the table with four significant digits.
fn shown_number(value: f64) -> String {
    crate::results::format_value(value as f32)
}

/// The points the fill helper makes, in SI units.
fn fill_rows(view: &StudyView, rules: &[FillRule], mode: FillMode) -> Vec<Vec<f64>> {
    let mut table = plx_model::StudyTable::default();
    for (name, kind) in &view.inputs {
        table.inputs.push(plx_model::StudyColumn {
            name: name.clone(),
            kind: *kind,
        });
    }
    table.points.push(plx_model::DesignPoint {
        inputs: view.current_inputs(),
        ..Default::default()
    });
    table.fill_rows(rules, mode)
}

/// The table as text, its cells separated by `separator`: a row of names with units, then a
/// row per point, in the units shown.
pub fn table_text(view: &StudyView, separator: char) -> String {
    let mut cells = vec!["Point".to_owned(), "State".to_owned()];
    for (i, (name, _)) in view.inputs.iter().enumerate() {
        cells.push(with_unit(name, view.input_unit(i)));
    }
    for (name, unit) in &view.outputs {
        cells.push(with_unit(name, unit));
    }
    cells.push("Time [s]".into());
    cells.push("Elements".into());
    let mut lines = vec![cells.join(&separator.to_string())];
    for (index, point) in view.points.iter().enumerate() {
        let mut cells = vec![format!("DP {index}"), point.state.label().to_owned()];
        for (column, &value) in point.inputs.iter().enumerate() {
            cells.push(format_physical(view.input_shown(column, value)));
        }
        for output in &point.outputs {
            cells.push(output.map_or(String::new(), format_physical));
        }
        cells.push(point.seconds.map_or(String::new(), |s| format!("{s:.1}")));
        cells.push(point.elements.map_or(String::new(), |e| e.to_string()));
        lines.push(cells.join(&separator.to_string()));
    }
    lines.join("\n") + "\n"
}

/// The inputs of the points of a pasted table or CSV file, in SI units: a first row of names
/// takes the columns named after inputs, the others the value of the current point; without
/// names the columns are the inputs in order.
pub fn parse_table(view: &StudyView, text: &str) -> Result<Vec<Vec<f64>>, String> {
    let lines: Vec<&str> = (text.lines())
        .filter(|l| !l.trim().is_empty() && !l.starts_with('#'))
        .collect();
    let Some(first) = lines.first() else {
        return Ok(Vec::new());
    };
    let separator = if first.contains('\t') {
        '\t'
    } else if first.contains(';') {
        ';'
    } else {
        ','
    };
    let split = |line: &str| -> Vec<String> {
        line.split(separator)
            .map(|c| c.trim().trim_matches('"').to_owned())
            .collect()
    };
    let head = split(first);
    let base = |cell: &str| match cell.rfind(" [") {
        Some(at) if cell.ends_with(']') => cell[..at].trim().to_owned(),
        _ => cell.to_owned(),
    };
    // A first row that names an input is the row of names.
    let named_columns: Vec<Option<usize>> = (view.inputs.iter())
        .map(|(name, _)| head.iter().position(|c| base(c) == *name))
        .collect();
    let named = named_columns.iter().any(Option::is_some);
    let columns: Vec<Option<usize>> = if named {
        named_columns
    } else {
        if head.len() < view.inputs.len() {
            return Err(format!(
                "A row has {} values for {} inputs.",
                head.len(),
                view.inputs.len()
            ));
        }
        (0..view.inputs.len()).map(Some).collect()
    };
    let current = view.current_inputs();
    let rows = if named { &lines[1..] } else { &lines[..] };
    (rows.iter().enumerate())
        .map(|(row, line)| {
            let cells = split(line);
            (columns.iter().enumerate())
                .map(|(input, column)| match column {
                    None => Ok(current[input]),
                    Some(at) => {
                        let cell = cells.get(*at).map_or("", String::as_str);
                        view.input_parsed(input, cell)
                            .map_err(|why| format!("Row {}: {why}", row + 1 + usize::from(named)))
                    }
                })
                .collect()
        })
        .collect()
}

/// The curves of the plot: each output shown over `x`, and per value of input `split` if
/// any, the points sorted along `x`; points without the values are left out.
fn plot_curves(
    view: &StudyView,
    x: Axis,
    shown: &[bool],
    split: Option<usize>,
) -> Vec<(String, Vec<[f64; 2]>)> {
    let x_of = |point: &PointView| match x {
        Axis::Input(i) => point.inputs.get(i).map(|&v| view.input_shown(i, v)),
        Axis::Output(i) => point.outputs.get(i).copied().flatten(),
        Axis::Elements => point.elements.map(|e| e as f64),
    };
    let mut groups: Vec<Option<f64>> = vec![None];
    if let Some(s) = split {
        groups = Vec::new();
        for point in &view.points {
            let value = Some(point.inputs[s]);
            if !groups.contains(&value) {
                groups.push(value);
            }
        }
        groups.sort_by(|a, b| a.unwrap_or(0.0).total_cmp(&b.unwrap_or(0.0)));
    }
    let mut curves = Vec::new();
    for (output, (name, _)) in view.outputs.iter().enumerate() {
        if !shown.get(output).copied().unwrap_or(false) {
            continue;
        }
        for group in &groups {
            let mut points: Vec<[f64; 2]> = (view.points.iter())
                .filter(|p| match (split, group) {
                    (Some(s), Some(value)) => p.inputs[s] == *value,
                    _ => true,
                })
                .filter_map(|p| Some([x_of(p)?, p.outputs.get(output).copied().flatten()?]))
                .collect();
            points.sort_by(|a, b| a[0].total_cmp(&b[0]));
            if points.is_empty() {
                continue;
            }
            let name = match (split, group) {
                (Some(s), Some(value)) => {
                    let unit = view.input_unit(s);
                    format!(
                        "O{} {name}, P{} = {} {unit}",
                        output + 1,
                        s + 1,
                        format_physical(view.input_shown(s, *value))
                    )
                    .trim()
                    .to_owned()
                }
                _ => format!("O{} {name}", output + 1),
            };
            curves.push((name, points));
        }
    }
    curves
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_model::Quantity;

    fn view() -> StudyView {
        let point = |force: f64, count: f64, stress: Option<f64>| PointView {
            inputs: vec![force, count],
            outputs: vec![stress],
            state: PointState::UpToDate,
            seconds: Some(1.0),
            elements: Some(40),
        };
        StudyView {
            units: UnitSystem::MmTonSC,
            inputs: vec![
                ("Force".into(), Some(ValueKind::Quantity(Quantity::Force))),
                ("Divisions".into(), Some(ValueKind::Count)),
            ],
            outputs: vec![("Stress".into(), "MPa".into())],
            points: vec![
                point(1000.0, 4.0, Some(10.0)),
                point(2000.0, 4.0, Some(20.0)),
                point(1000.0, 8.0, None),
            ],
            current: 0,
            running: None,
            busy: false,
        }
    }

    #[test]
    fn the_table_is_copied_and_read_back() {
        let view = view();
        let text = table_text(&view, '\t');
        assert!(
            text.starts_with("Point\tState\tForce [N]\tDivisions\tStress [MPa]"),
            "{text}"
        );
        assert!(
            text.contains("DP 1\tup to date\t2000\t4\t20\t1.0\t40"),
            "{text}"
        );
        let rows = parse_table(&view, &text).unwrap();
        assert_eq!(rows[1], [2000.0, 4.0]);
        // Without names, in the order of the inputs; a unit converts.
        let rows = parse_table(&view, "3 kN; 5\n4000;6").unwrap();
        assert_eq!(rows, [vec![3000.0, 5.0], vec![4000.0, 6.0]]);
        // A missing input takes the current value.
        assert_eq!(
            parse_table(&view, "Divisions\n9").unwrap(),
            [vec![1000.0, 9.0]]
        );
        assert!(
            parse_table(&view, "Other;1\n1;2")
                .unwrap_err()
                .contains("Row 1")
        );
        assert!(
            parse_table(&view, "Force,Divisions\nx,1")
                .unwrap_err()
                .contains("Row 2")
        );
    }

    #[test]
    fn the_plot_has_a_curve_per_value_of_an_input() {
        let view = view();
        let curves = plot_curves(&view, Axis::Input(0), &[true], None);
        assert_eq!(curves.len(), 1);
        assert_eq!(curves[0].1, [[1000.0, 10.0], [2000.0, 20.0]]);
        let curves = plot_curves(&view, Axis::Input(0), &[true], Some(1));
        assert_eq!(curves.len(), 1, "a group without values has no curve");
        assert_eq!(curves[0].0, "O1 Stress, P2 = 4");
    }
}
