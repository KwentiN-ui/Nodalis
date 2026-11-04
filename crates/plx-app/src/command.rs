//! The command register: every command of the ribbon, in the workspace, tab and group it is
//! shown in, with its icon, shortcut and tooltip. The ribbon, the command search (Ctrl+K)
//! and the tooltips are made from it; the project and the analyses carry the commands out.

use std::sync::LazyLock;

use plx_render::StandardView;

use crate::icons::Icon;
use crate::node_icons::NodeIcon;
use crate::tree_icons::TreeIcon;
use crate::viewport::Axis;

/// A workspace of the window: one step of the work, with its own ribbon tabs and tree.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Workspace {
    /// The schematic of the project: analyses and the blocks they take data from.
    Project,
    Geometry,
    FeModel,
    Results,
}

impl Workspace {
    pub const ANALYSIS: [Workspace; 3] = [Self::Geometry, Self::FeModel, Self::Results];

    pub fn title(self) -> &'static str {
        match self {
            Self::Project => "Project",
            Self::Geometry => "Geometry",
            Self::FeModel => "FE Model",
            Self::Results => "Results",
        }
    }

    /// The tree of an analysis this workspace shows; none for the project.
    pub fn view(self) -> Option<crate::tree::TreeView> {
        use crate::tree::TreeView;
        match self {
            Self::Project => None,
            Self::Geometry => Some(TreeView::Geometry),
            Self::FeModel => Some(TreeView::FeModel),
            Self::Results => Some(TreeView::Results),
        }
    }

    pub fn of(view: crate::tree::TreeView) -> Self {
        use crate::tree::TreeView;
        match view {
            TreeView::Geometry => Self::Geometry,
            TreeView::FeModel => Self::FeModel,
            TreeView::Results => Self::Results,
        }
    }

    /// The ribbon tabs of the workspace, in the order of the work.
    pub fn tabs(self) -> &'static [&'static str] {
        match self {
            Self::Project => &[
                "File",
                "Sources",
                "Parameters",
                "Evaluate",
                "Annotate",
                "Arrange",
            ],
            Self::Geometry => &["Geometry", "View"],
            Self::FeModel => &[
                "Mesh",
                "Material",
                "Features",
                "Connections",
                "Steps",
                "View",
            ],
            Self::Results => &["File", "Evaluate", "Display"],
        }
    }
}

/// A kind of block a tile of the project's ribbon places in the schematic.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BlockTile {
    Analysis,
    Materials,
    CrossSections,
    BoxMesh,
    RectangleMesh,
    RefineMesh,
    CadOperation,
    File,
    Parameters,
    Math,
    Switch,
    Measure,
    Study,
    Campbell,
    MeshExport,
    Note,
}

/// A command. The project carries out those of the Project workspace, an analysis the
/// others.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cmd {
    // Project workspace.
    NewProject,
    OpenProject,
    SaveProject,
    SaveProjectAs,
    SaveWithoutResults,
    NewAnalysis,
    OpenBlock,
    OpenInNewWindow,
    SolveBlock,
    Duplicate,
    Delete,
    Rename,
    UpdateProject,
    StopUpdate,
    ProjectMonitor,
    /// Place a block: it hangs at the pointer until a click into the schematic.
    Place(BlockTile),
    FrameBlocks,
    Unframe,
    Compact,
    ShowAll,
    Minimap,
    // Geometry workspace.
    Import,
    DefaultMeshParameters,
    LocalMeshSize,
    ExtrudeMesh,
    TetrahedralGmsh,
    MeshAll,
    MeshSelected,
    CancelMeshing,
    PartProperties,
    HideShow,
    DeleteParts,
    Query,
    RemoveAnnotations,
    // Solve, pinned in Geometry and FE Model.
    Solve,
    SolveAgain,
    CheckModel,
    Stop,
    Monitor,
    Campbell,
    // FE Model workspace.
    NodeSet,
    ElementSet,
    Surface,
    ElementType,
    Transform,
    MergeParts,
    MergeNodes,
    Renumber,
    ImportMesh,
    Material,
    MaterialLibrary,
    Section,
    CrossSection,
    ModelProperties,
    ReferencePoint,
    CoordinateSystem,
    Plane,
    DetectContacts,
    ContactRegion,
    Bolt,
    Constraint,
    SurfaceInteraction,
    ContactPair,
    Tie,
    NodeTie,
    ContactWorksheet,
    ConnectionSettings,
    Step,
    BoundaryCondition,
    Load,
    InitialCondition,
    DefinedField,
    Amplitude,
    Distribution,
    FieldOutput,
    HistoryOutput,
    /// The combo box of the step new items go into.
    ActiveStep,
    // Results workspace.
    /// The deformation of the shown results and its factor.
    Deformation,
    ShowUndeformed,
    /// The edges of the global model around the results of a submodel.
    GlobalContours,
    /// Contours, arrows or streaks, with the field and principal direction they follow.
    Representation,
    ColorLevels,
    Transformations,
    /// The step and increment shown.
    Increment,
    FirstIncrement,
    PreviousIncrement,
    NextIncrement,
    LastIncrement,
    Animate,
    ModeSound,
    OpenResults,
    RemoveResults,
    ExportVtu,
    ExportVtk,
    ExportFrd,
    ExportDeformedMesh,
    ColorScale,
    HotSpot,
    Path,
    PlaneResult,
    ResultHistoryOutput,
    DerivedFieldOutput,
    // View tab of Geometry, FE Model and Results.
    ZoomToFit,
    StandardViews,
    View(StandardView),
    AxisView(Axis),
    IsometricAxis(Axis),
    VerticalView,
    EdgesOnly,
    MeshEdges,
    ShellThickness,
    BeamProfiles,
    ShowLinked,
    SectionView,
    SectionOff,
    ExplodedView,
    ExplodedOff,
    Screenshot,
    ScreenshotClipboard,
    ScreenshotFile,
    ExportInp,
    /// Shows the checklist in the properties panel again.
    Checklist,
    // Everywhere.
    Settings,
    /// The Solver page of the settings.
    SolverSettings,
    UserGuide,
    OpenLogFolder,
    CheckForUpdates,
}

/// How a command shows in the ribbon.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tile {
    /// A tile that runs the command.
    Button,
    /// A tile whose upper part runs the command and whose label opens a menu of the
    /// commands that name it as their menu.
    Split,
    /// A tile that opens a menu of the commands that name it as their menu.
    Menu,
    /// An entry of the menu of this command; not a tile of its own.
    In(Cmd),
    /// A tile whose label shows a value of the workspace, such as the deformation, and
    /// which opens a panel the workspace fills to change it.
    Panel,
    /// Drawn by the workspace itself, such as the active step combo.
    Custom,
    /// Only in the command search.
    Hidden,
}

/// The icon of a tile.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TileIcon {
    Tool(Icon),
    Node(NodeIcon),
    Tree(TreeIcon),
    Shape(Glyph),
    None,
}

/// Icons only the ribbon and the app bar need, drawn with strokes in [`crate::ribbon`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Glyph {
    Window,
    Duplicate,
    Rename,
    Frame,
    Unframe,
    Compact,
    Minimap,
    Merge,
    Renumber,
    Library,
    Check,
    Hide,
    Worksheet,
    Settings,
    Help,
    Undo,
    Redo,
    Search,
    Analysis,
    Save,
    /// The button of the app bar that shows the folded tree.
    Tree,
}

pub struct Command {
    pub cmd: Cmd,
    /// The text of the tile; a line break splits it into its two lines.
    pub label: &'static str,
    /// The workspaces it is shown in; the first is where the search takes it.
    pub workspaces: &'static [Workspace],
    /// The ribbon tab, or [`PINNED`] for the group shown at the right of every tab.
    pub tab: &'static str,
    pub group: &'static str,
    pub icon: TileIcon,
    pub shortcut: &'static str,
    pub tip: &'static str,
    pub tile: Tile,
}

impl Command {
    /// The tab the command is on in `workspace`: the view commands join the controls of the
    /// results on the Display tab of the Results workspace.
    pub fn tab_in(&self, workspace: Workspace) -> &'static str {
        if self.tab == "View" && workspace == Workspace::Results {
            "Display"
        } else {
            self.tab
        }
    }

    /// The label on one line, for menus, the search and tooltips.
    pub fn name(&self) -> String {
        // A word split over the lines loses its hyphen ("Screen-\nshot"), a hyphenated word
        // keeps it ("Cross-\nSections").
        let mut name = String::new();
        let mut lines = self.label.lines().peekable();
        while let Some(line) = lines.next() {
            name.push_str(line);
            match lines.peek() {
                Some(next) if line.ends_with('-') => {
                    if next.starts_with(char::is_lowercase) {
                        name.pop();
                    }
                }
                Some(_) => name.push(' '),
                None => {}
            }
        }
        name
    }
}

/// The tab of the groups shown at the right of every tab of a workspace.
pub const PINNED: &str = "*";

const PROJECT: &[Workspace] = &[Workspace::Project];
const GEOMETRY: &[Workspace] = &[Workspace::Geometry];
const FE_MODEL: &[Workspace] = &[Workspace::FeModel];
const RESULTS: &[Workspace] = &[Workspace::Results];
const SOLVING: &[Workspace] = &[Workspace::Geometry, Workspace::FeModel];
const VIEWING: &[Workspace] = &[Workspace::Geometry, Workspace::FeModel, Workspace::Results];
const ALL: &[Workspace] = &[
    Workspace::Project,
    Workspace::Geometry,
    Workspace::FeModel,
    Workspace::Results,
];

pub const STANDARD_VIEWS: [(StandardView, &str); 7] = [
    (StandardView::Front, "Front View"),
    (StandardView::Back, "Back View"),
    (StandardView::Top, "Top View"),
    (StandardView::Bottom, "Bottom View"),
    (StandardView::Left, "Left View"),
    (StandardView::Right, "Right View"),
    (StandardView::Isometric, "Isometric View"),
];

/// All commands, in the order of their tabs and groups.
pub fn registry() -> &'static [Command] {
    static REGISTRY: LazyLock<Vec<Command>> = LazyLock::new(build);
    &REGISTRY
}

/// The command `cmd` as registered.
pub fn find(cmd: Cmd) -> Option<&'static Command> {
    registry().iter().find(|c| c.cmd == cmd)
}

#[allow(clippy::too_many_arguments)]
fn command(
    cmd: Cmd,
    label: &'static str,
    workspaces: &'static [Workspace],
    tab: &'static str,
    group: &'static str,
    icon: TileIcon,
    shortcut: &'static str,
    tip: &'static str,
) -> Command {
    Command {
        cmd,
        label,
        workspaces,
        tab,
        group,
        icon,
        shortcut,
        tip,
        tile: Tile::Button,
    }
}

fn build() -> Vec<Command> {
    use Cmd as C;
    use TileIcon::{Node, Shape, Tool, Tree};
    let mut list = Vec::new();
    let mut add = |c: Command| list.push(c);
    let tile = |mut c: Command, tile: Tile| {
        c.tile = tile;
        c
    };

    // Project workspace, tab Project.
    let (p, t) = (PROJECT, "File");
    add(command(
        C::NewProject,
        "New\nProject",
        p,
        t,
        "File",
        Tool(Icon::New),
        "",
        "Starts a new, empty project with one analysis.",
    ));
    add(command(
        C::OpenProject,
        "Open",
        p,
        t,
        "File",
        Tool(Icon::Open),
        "Ctrl+O",
        "Opens a project file.",
    ));
    add(command(
        C::SaveProject,
        "Save",
        p,
        t,
        "File",
        Tool(Icon::Save),
        "Ctrl+S",
        "Saves the project.",
    ));
    add(tile(
        command(
            C::SaveProjectAs,
            "Save As",
            ALL,
            t,
            "File",
            TileIcon::None,
            "Ctrl+Shift+S",
            "Saves the project to a file you pick.",
        ),
        Tile::Hidden,
    ));
    add(tile(
        command(
            C::SaveWithoutResults,
            "Save Without Results",
            ALL,
            t,
            "File",
            TileIcon::None,
            "",
            "Saves a copy with only the meshes and the model definitions, without the results the analyses keep.",
        ),
        Tile::Hidden,
    ));
    add(command(
        C::NewAnalysis,
        "New\nAnalysis",
        p,
        t,
        "Analysis",
        Shape(Glyph::Analysis),
        "",
        "Adds an analysis: asks for its model space and unit system, then hangs its block at the pointer.",
    ));
    add(command(
        C::OpenBlock,
        "Open",
        p,
        t,
        "Selected",
        Tool(Icon::Arrow(egui::vec2(1.0, 0.0))),
        "",
        "Opens the selected analysis in this window, or the editor of the selected block.",
    ));
    add(command(
        C::OpenInNewWindow,
        "Open in\nNew Window",
        p,
        t,
        "Selected",
        Shape(Glyph::Window),
        "",
        "Opens the selected analysis in a window of its own.",
    ));
    add(command(
        C::SolveBlock,
        "Solve",
        p,
        t,
        "Selected",
        Tool(Icon::Solve),
        "",
        "Solves the selected analysis and what it takes data from.",
    ));
    add(command(
        C::Duplicate,
        "Duplicate",
        p,
        t,
        "Selected",
        Shape(Glyph::Duplicate),
        "Shift+D",
        "Duplicates the selected blocks.",
    ));
    add(command(
        C::Delete,
        "Delete",
        p,
        t,
        "Selected",
        Tool(Icon::Remove),
        "X",
        "Deletes the selected blocks.",
    ));
    add(command(
        C::Rename,
        "Rename",
        p,
        t,
        "Selected",
        Shape(Glyph::Rename),
        "F2",
        "Renames the selected block.",
    ));
    add(command(
        C::UpdateProject,
        "Update\nProject",
        p,
        PINNED,
        "Update",
        Tool(Icon::Solve),
        "F5",
        "Brings all analyses and blocks of the project up to date.",
    ));
    add(command(
        C::StopUpdate,
        "Stop",
        p,
        PINNED,
        "Update",
        Tool(Icon::Stop),
        "",
        "Stops Update Project.",
    ));
    add(command(
        C::ProjectMonitor,
        "Monitor",
        p,
        PINNED,
        "Update",
        Tool(Icon::Monitor),
        "",
        "Shows the solution monitor below the schematic.",
    ));

    // Project workspace, a tab per kind of block.
    let place = |block, label, tab, group, icon, tip| {
        command(C::Place(block), label, PROJECT, tab, group, icon, "", tip)
    };
    add(place(
        BlockTile::Materials,
        "Materials",
        "Sources",
        "Model",
        Node(NodeIcon::Materials),
        "Places a block of materials that analyses take.",
    ));
    add(place(
        BlockTile::CrossSections,
        "Cross-\nSections",
        "Sources",
        "Model",
        Node(NodeIcon::CrossSections),
        "Places a block of beam cross-sections that analyses take.",
    ));
    add(place(
        BlockTile::BoxMesh,
        "Box\nMesh",
        "Sources",
        "Mesh",
        Node(NodeIcon::Mesh),
        "Places a block that meshes a box with hexahedra, for 3D analyses.",
    ));
    add(place(
        BlockTile::RectangleMesh,
        "Rectangle\nMesh",
        "Sources",
        "Mesh",
        Node(NodeIcon::Mesh),
        "Places a block that meshes a rectangle with quadrilaterals, for 2D analyses.",
    ));
    add(place(
        BlockTile::RefineMesh,
        "Refine\nMesh",
        "Sources",
        "Mesh",
        Node(NodeIcon::Mesh),
        "Places a block that makes a mesh finer or coarser by a factor.",
    ));
    add(place(
        BlockTile::CadOperation,
        "CAD\nOperation",
        "Sources",
        "Geometry",
        Node(NodeIcon::CadOperation),
        "Places a block that fillets, chamfers or offsets the geometry linked to it.",
    ));
    add(place(
        BlockTile::File,
        "File",
        "Sources",
        "Geometry",
        Tool(Icon::Import),
        "Places a block that reads results, a mesh or geometry from a file outside the project.",
    ));
    add(place(
        BlockTile::Parameters,
        "Parameters",
        "Parameters",
        "Values",
        Node(NodeIcon::Setup),
        "Places a block of named numbers that other blocks take.",
    ));
    add(place(
        BlockTile::Math,
        "Math",
        "Parameters",
        "Operations",
        Node(NodeIcon::Math),
        "Places a block that computes a number from two others.",
    ));
    add(place(
        BlockTile::Switch,
        "Switch",
        "Parameters",
        "Operations",
        Node(NodeIcon::Switch),
        "Places a block that gives one of several numbers, picked by its index.",
    ));
    add(place(
        BlockTile::Measure,
        "Measure",
        "Evaluate",
        "Measure",
        Node(NodeIcon::Results),
        "Places a block that takes a value from results, such as the largest stress.",
    ));
    add(place(
        BlockTile::Study,
        "Parameter\nStudy",
        "Evaluate",
        "Study",
        Node(NodeIcon::Steps),
        "Places a parameter study: a table of design points.",
    ));
    add(place(
        BlockTile::Campbell,
        "Campbell",
        "Evaluate",
        "Rotor",
        Node(NodeIcon::Solution),
        "Places a block that sweeps a rotor over its speed for the Campbell diagram.",
    ));
    add(place(
        BlockTile::MeshExport,
        "Mesh\nExport",
        "Evaluate",
        "Export",
        Node(NodeIcon::Export),
        "Places a block that writes the mesh linked to it to an input file.",
    ));
    add(place(
        BlockTile::Note,
        "Note",
        "Annotate",
        "Notes",
        Node(NodeIcon::Note),
        "Places a note: a text in the schematic.",
    ));
    add(command(
        C::FrameBlocks,
        "Frame",
        p,
        "Annotate",
        "Frames",
        Shape(Glyph::Frame),
        "Ctrl+J",
        "Puts the selected blocks in a frame.",
    ));
    add(command(
        C::Unframe,
        "Remove\nfrom Frame",
        p,
        "Annotate",
        "Frames",
        Shape(Glyph::Unframe),
        "Alt+P",
        "Takes the selected blocks out of their frame.",
    ));
    // Commands shown on a second tab; the search lists the first.
    add(command(
        C::Duplicate,
        "Duplicate",
        p,
        "Arrange",
        "Selection",
        Shape(Glyph::Duplicate),
        "Shift+D",
        "Duplicates the selected blocks.",
    ));
    add(command(
        C::Delete,
        "Delete",
        p,
        "Arrange",
        "Selection",
        Tool(Icon::Remove),
        "X",
        "Deletes the selected blocks.",
    ));
    add(command(
        C::Compact,
        "Compact\nView",
        p,
        "Arrange",
        "Selection",
        Shape(Glyph::Compact),
        "H",
        "Shows the selected blocks with only their linked rows, or all rows again.",
    ));
    add(command(
        C::ShowAll,
        "Show All",
        p,
        "Arrange",
        "View",
        Tool(Icon::Fit),
        "Home",
        "Fits all blocks into the view.",
    ));
    add(command(
        C::Minimap,
        "Minimap",
        p,
        "Arrange",
        "View",
        Shape(Glyph::Minimap),
        "",
        "Shows all blocks in small in the corner of the schematic.",
    ));

    // Geometry workspace.
    let (g, t) = (GEOMETRY, "Geometry");
    add(command(
        C::Import,
        "Import\nGeometry",
        g,
        t,
        "Import",
        Tool(Icon::Import),
        "",
        "Imports geometry (STEP, IGES, BREP) or a mesh (INP) into the analysis.",
    ));
    add(command(
        C::DefaultMeshParameters,
        "Default\nParameters",
        g,
        t,
        "Mesh Setup",
        Tool(Icon::MeshDefaults),
        "",
        "The default element sizes and settings of the mesher.",
    ));
    add(command(
        C::LocalMeshSize,
        "Local\nMesh Size",
        g,
        t,
        "Mesh Setup",
        Tool(Icon::LocalMeshSize),
        "",
        "A smaller element size on picked faces or edges.",
    ));
    add(command(
        C::ExtrudeMesh,
        "Extrude\nMesh",
        g,
        t,
        "Mesh Setup",
        Tool(Icon::ExtrudeMesh),
        "",
        "Hexahedra swept from a face mesh through a part.",
    ));
    add(command(
        C::TetrahedralGmsh,
        "Tetra.\nGmsh",
        g,
        t,
        "Mesh Setup",
        Tool(Icon::TetMesh),
        "",
        "The Gmsh algorithms used for picked parts.",
    ));
    add(command(
        C::MeshAll,
        "Mesh All\nParts",
        g,
        t,
        "Mesh",
        Node(NodeIcon::Mesh),
        "",
        "Meshes all parts of the geometry.",
    ));
    add(command(
        C::MeshSelected,
        "Mesh\nSelected",
        g,
        t,
        "Mesh",
        Node(NodeIcon::Mesh),
        "",
        "Meshes the parts selected in the tree.",
    ));
    add(command(
        C::CancelMeshing,
        "Cancel",
        g,
        t,
        "Mesh",
        Tool(Icon::Stop),
        "",
        "Stops the meshing; the parts keep their old mesh.",
    ));
    add(command(
        C::PartProperties,
        "Properties",
        g,
        t,
        "Part",
        Tree(TreeIcon::Part),
        "",
        "The name and properties of the part selected in the tree.",
    ));
    add(command(
        C::HideShow,
        "Hide /\nShow",
        g,
        t,
        "Part",
        Shape(Glyph::Hide),
        "Space",
        "Hides the parts selected in the tree, or shows them again.",
    ));
    add(command(
        C::DeleteParts,
        "Delete",
        g,
        t,
        "Part",
        Tool(Icon::Remove),
        "Del",
        "Deletes the parts selected in the tree.",
    ));
    add(command(
        C::Query,
        "Query",
        &[Workspace::Geometry, Workspace::FeModel],
        "View",
        "Inspect",
        Tool(Icon::Query),
        "",
        "Picks values from the model: points, distances, angles, results.",
    ));
    add(command(
        C::RemoveAnnotations,
        "Remove\nAnnotations",
        &[Workspace::Geometry, Workspace::FeModel],
        "View",
        "Inspect",
        Tool(Icon::RemoveAnnotations),
        "",
        "Removes all annotations from the 3D view.",
    ));
    add(tile(
        command(
            C::Solve,
            "Solve",
            SOLVING,
            PINNED,
            "Solve",
            Tool(Icon::Solve),
            "F5",
            "Solves the analysis, with the analyses it takes results from first.",
        ),
        Tile::Split,
    ));
    add(tile(
        command(
            C::SolveAgain,
            "Solve Again",
            SOLVING,
            PINNED,
            "Solve",
            TileIcon::None,
            "",
            "Solves even if the results are up to date.",
        ),
        Tile::In(C::Solve),
    ));
    add(tile(
        command(
            C::Campbell,
            "Campbell Diagram",
            SOLVING,
            PINNED,
            "Solve",
            TileIcon::None,
            "",
            "Opens the Campbell block of the analysis in the project; makes it first if there is none.",
        ),
        Tile::In(C::Solve),
    ));
    add(command(
        C::CheckModel,
        "Check\nModel",
        FE_MODEL,
        PINNED,
        "Solve",
        Shape(Glyph::Check),
        "",
        "Checks the model for what keeps it from being solved.",
    ));
    add(command(
        C::Stop,
        "Stop",
        SOLVING,
        PINNED,
        "Solve",
        Tool(Icon::Stop),
        "",
        "Stops the solver.",
    ));
    add(command(
        C::Monitor,
        "Monitor",
        SOLVING,
        PINNED,
        "Solve",
        Tool(Icon::Monitor),
        "",
        "Shows the output of the solver below the 3D view.",
    ));

    // FE Model workspace.
    let (f, t) = (FE_MODEL, "Mesh");
    add(command(
        C::NodeSet,
        "Node\nSet",
        f,
        t,
        "Sets",
        Tree(TreeIcon::NodeSet),
        "",
        "A named set of nodes, picked in the 3D view.",
    ));
    add(command(
        C::ElementSet,
        "Element\nSet",
        f,
        t,
        "Sets",
        Tree(TreeIcon::ElementSet),
        "",
        "A named set of elements, picked in the 3D view.",
    ));
    add(command(
        C::Surface,
        "Surface",
        f,
        t,
        "Sets",
        Tree(TreeIcon::Surface),
        "",
        "A named surface of element faces, picked in the 3D view.",
    ));
    add(command(
        C::ElementType,
        "Element\nType",
        f,
        t,
        "Elements",
        Tree(TreeIcon::Part),
        "",
        "The element types of the part selected in the tree.",
    ));
    add(command(
        C::Transform,
        "Transform",
        f,
        t,
        "Transform",
        Tool(Icon::Transformation),
        "",
        "Translates, rotates, mirrors or scales the parts selected in the tree.",
    ));
    add(command(
        C::MergeParts,
        "Merge\nParts",
        f,
        t,
        "Merge",
        Shape(Glyph::Merge),
        "",
        "Merges the parts selected in the tree into one.",
    ));
    add(command(
        C::MergeNodes,
        "Merge\nNodes",
        f,
        t,
        "Merge",
        Shape(Glyph::Merge),
        "",
        "Merges coincident nodes.",
    ));
    add(command(
        C::Renumber,
        "Renumber",
        f,
        t,
        "Merge",
        Shape(Glyph::Renumber),
        "",
        "Renumbers nodes and elements.",
    ));
    add(command(
        C::ImportMesh,
        "Import\nMesh (.inp)",
        f,
        t,
        "Import",
        Tool(Icon::Import),
        "",
        "Imports the mesh of an input file as parts.",
    ));
    let t = "Material";
    add(command(
        C::Material,
        "Material",
        f,
        t,
        "Material",
        Tree(TreeIcon::Material),
        "",
        "Creates a material.",
    ));
    add(command(
        C::MaterialLibrary,
        "Material\nLibrary",
        f,
        t,
        "Material",
        Shape(Glyph::Library),
        "",
        "Takes materials from the library or adds them to it.",
    ));
    add(command(
        C::Section,
        "Section",
        f,
        t,
        "Section",
        Tree(TreeIcon::Section),
        "",
        "Gives parts a material and, for shells and beams, a thickness or profile.",
    ));
    add(command(
        C::CrossSection,
        "Cross-\nSection",
        f,
        t,
        "Section",
        Node(NodeIcon::CrossSections),
        "",
        "A beam profile that beam sections pick.",
    ));
    add(command(
        C::ModelProperties,
        "Model\nProperties",
        f,
        t,
        "Model",
        Node(NodeIcon::Setup),
        "",
        "Model space, unit system and constants of the model.",
    ));
    let t = "Features";
    add(command(
        C::ReferencePoint,
        "Reference\nPoint",
        f,
        t,
        "Features",
        Tree(TreeIcon::ReferencePoint),
        "",
        "A point that rigid bodies and remote loads refer to.",
    ));
    add(command(
        C::CoordinateSystem,
        "Coordinate\nSystem",
        f,
        t,
        "Features",
        Tree(TreeIcon::CoordinateSystem),
        "",
        "A local coordinate system.",
    ));
    add(command(
        C::Plane,
        "Plane",
        f,
        t,
        "Features",
        Tree(TreeIcon::Plane),
        "",
        "A plane, for example for a section.",
    ));
    let t = "Connections";
    add(command(
        C::DetectContacts,
        "Detect\nContacts",
        f,
        t,
        "Detect",
        Tree(TreeIcon::Contacts),
        "",
        "Finds the faces of parts that touch and makes contact regions.",
    ));
    add(command(
        C::ContactRegion,
        "Contact\nRegion",
        f,
        t,
        "Create",
        Tree(TreeIcon::Contacts),
        "",
        "Two surfaces in contact by the behaviour you pick.",
    ));
    add(command(
        C::Bolt,
        "Bolt",
        f,
        t,
        "Create",
        Tree(TreeIcon::Constraints),
        "",
        "A bolt preloaded across a cut through its shank.",
    ));
    add(command(
        C::Constraint,
        "Constraint",
        f,
        t,
        "Create",
        Tree(TreeIcon::Constraints),
        "",
        "A spring, support or rigid body.",
    ));
    add(command(
        C::SurfaceInteraction,
        "Surface\nInteraction",
        f,
        t,
        "Advanced",
        Tree(TreeIcon::SurfaceInteraction),
        "",
        "The contact behaviour contact pairs refer to.",
    ));
    add(command(
        C::ContactPair,
        "Contact\nPair",
        f,
        t,
        "Advanced",
        Tree(TreeIcon::ContactPair),
        "",
        "A contact between a master and a slave surface.",
    ));
    add(command(
        C::Tie,
        "Tie",
        f,
        t,
        "Advanced",
        Tree(TreeIcon::ContactPair),
        "",
        "Surfaces glued to each other.",
    ));
    add(command(
        C::NodeTie,
        "Node\nTie",
        f,
        t,
        "Advanced",
        Tree(TreeIcon::ContactPair),
        "",
        "Nodes tied to the nearest nodes of other parts.",
    ));
    add(command(
        C::ContactWorksheet,
        "Contact\nWorksheet",
        f,
        t,
        "Review",
        Shape(Glyph::Worksheet),
        "",
        "The table of all contact regions.",
    ));
    add(command(
        C::ConnectionSettings,
        "Connection\nSettings",
        f,
        t,
        "Review",
        Tree(TreeIcon::Contacts),
        "",
        "Settings of all contacts.",
    ));
    let t = "Steps";
    add(tile(
        command(
            C::ActiveStep,
            "",
            f,
            t,
            "Active Step",
            TileIcon::None,
            "",
            "",
        ),
        Tile::Custom,
    ));
    add(command(
        C::Step,
        "Step",
        f,
        t,
        "Create",
        Tree(TreeIcon::Step),
        "",
        "A step of the analysis: static, frequency, buckle, dynamic, heat transfer.",
    ));
    add(command(
        C::BoundaryCondition,
        "Boundary\nCondition",
        f,
        t,
        "Create",
        Tree(TreeIcon::BoundaryCondition),
        "",
        "A support or prescribed displacement in the last step.",
    ));
    add(command(
        C::Load,
        "Load",
        f,
        t,
        "Create",
        Tree(TreeIcon::Load),
        "",
        "A force, pressure or other load in the last step.",
    ));
    add(command(
        C::InitialCondition,
        "Initial\nCondition",
        f,
        t,
        "Create",
        Tree(TreeIcon::InitialConditions),
        "",
        "A temperature or velocity at the start.",
    ));
    add(command(
        C::DefinedField,
        "Defined\nField",
        f,
        t,
        "Create",
        Tree(TreeIcon::DefinedField),
        "",
        "A temperature field given in the last step.",
    ));
    add(command(
        C::Amplitude,
        "Amplitude",
        f,
        t,
        "Create",
        Tree(TreeIcon::Amplitude),
        "",
        "A time curve for boundary conditions and loads.",
    ));
    add(command(
        C::Distribution,
        "Distribution",
        f,
        t,
        "Create",
        Tree(TreeIcon::Distribution),
        "",
        "A factor over the model for pressures and temperatures.",
    ));
    add(command(
        C::FieldOutput,
        "Field\nOutput",
        f,
        t,
        "Output",
        Tree(TreeIcon::FieldOutput),
        "",
        "Results written over the whole model in the last step.",
    ));
    add(command(
        C::HistoryOutput,
        "History\nOutput",
        f,
        t,
        "Output",
        Tree(TreeIcon::HistoryOutput),
        "",
        "Values printed for a region in the last step.",
    ));

    // Results workspace: File holds what reads and writes results.
    let (r, t) = (RESULTS, "File");
    add(command(
        C::OpenResults,
        "Open\nResults",
        r,
        t,
        "Import",
        Tool(Icon::Open),
        "",
        "Opens the results the analysis keeps.",
    ));
    add(command(
        C::RemoveResults,
        "Remove",
        r,
        t,
        "Import",
        Tool(Icon::Remove),
        "",
        "Drops the results the analysis keeps; saving the project leaves them out.",
    ));
    add(command(
        C::ExportVtu,
        "VTU",
        r,
        t,
        "Export",
        Tool(Icon::Arrow(egui::vec2(1.0, 0.0))),
        "",
        "Writes the results for ParaView (VTU).",
    ));
    add(command(
        C::ExportVtk,
        "VTK",
        r,
        t,
        "Export",
        Tool(Icon::Arrow(egui::vec2(1.0, 0.0))),
        "",
        "Writes the results for ParaView (legacy VTK).",
    ));
    add(command(
        C::ExportFrd,
        "FRD",
        r,
        t,
        "Export",
        Tool(Icon::Arrow(egui::vec2(1.0, 0.0))),
        "",
        "Writes the results as CalculiX's .frd and .dat files.",
    ));
    add(command(
        C::ExportDeformedMesh,
        "Deformed\nMesh",
        r,
        t,
        "Export",
        Tool(Icon::Arrow(egui::vec2(1.0, 0.0))),
        "",
        "Writes the deformed mesh of the current results as an input file.",
    ));
    let t = "Evaluate";
    add(command(
        C::Query,
        "Query",
        r,
        t,
        "Inspect",
        Tool(Icon::Query),
        "",
        "Picks values from the model: points, distances, angles, results.",
    ));
    add(command(
        C::RemoveAnnotations,
        "Remove\nAnnotations",
        r,
        t,
        "Inspect",
        Tool(Icon::RemoveAnnotations),
        "",
        "Removes all annotations from the 3D view.",
    ));
    add(command(
        C::HotSpot,
        "Hot Spot",
        r,
        t,
        "Create",
        Tree(TreeIcon::HotSpot),
        "",
        "Structural hot spot stresses by extrapolation.",
    ));
    add(command(
        C::Path,
        "Path",
        r,
        t,
        "Create",
        Tree(TreeIcon::Path),
        "",
        "Results along a line through the model.",
    ));
    add(command(
        C::PlaneResult,
        "Plane\nResult",
        r,
        t,
        "Create",
        Tree(TreeIcon::PlaneResult),
        "",
        "Results on a plane through the model.",
    ));
    add(command(
        C::ResultHistoryOutput,
        "History\nOutput",
        r,
        t,
        "Create",
        Tree(TreeIcon::HistoryOutput),
        "",
        "Values over the increments for a region.",
    ));
    add(command(
        C::DerivedFieldOutput,
        "Derived\nField",
        r,
        t,
        "Create",
        Tree(TreeIcon::FieldOutput),
        "",
        "A field computed from the results by an equation.",
    ));
    add(command(
        C::Campbell,
        "Campbell",
        r,
        t,
        "Rotor",
        Node(NodeIcon::Solution),
        "",
        "Opens the Campbell block of the analysis in the project.",
    ));

    // View tab of the analysis workspaces.
    let (v, t) = (VIEWING, "View");
    add(command(
        C::ZoomToFit,
        "Zoom\nto Fit",
        v,
        t,
        "Camera",
        Tool(Icon::Fit),
        "",
        "Fits the model into the view.",
    ));
    add(tile(
        command(
            C::StandardViews,
            "Standard\nViews",
            v,
            t,
            "Camera",
            Tool(Icon::View(StandardView::Isometric)),
            "",
            "Looks at the model from a side.",
        ),
        Tile::Menu,
    ));
    for (view, label) in STANDARD_VIEWS {
        add(tile(
            command(
                C::View(view),
                label,
                v,
                t,
                "Camera",
                Tool(Icon::View(view)),
                "",
                "Looks at the model from this side.",
            ),
            Tile::In(C::StandardViews),
        ));
    }
    for axis in Axis::ALL {
        let label = match axis {
            Axis::X => "View Normal to X",
            Axis::Y => "View Normal to Y",
            Axis::Z => "View Normal to Z",
        };
        add(tile(
            command(
                C::AxisView(axis),
                label,
                v,
                t,
                "Camera",
                TileIcon::None,
                "",
                "Looks along the axis.",
            ),
            Tile::In(C::StandardViews),
        ));
    }
    for axis in Axis::ALL {
        let label = match axis {
            Axis::X => "Isometric, X Up",
            Axis::Y => "Isometric, Y Up",
            Axis::Z => "Isometric, Z Up",
        };
        add(tile(
            command(
                C::IsometricAxis(axis),
                label,
                v,
                t,
                "Camera",
                TileIcon::None,
                "",
                "An isometric view with the axis pointing up.",
            ),
            Tile::In(C::StandardViews),
        ));
    }
    add(command(
        C::VerticalView,
        "Vertical\nView",
        v,
        t,
        "Camera",
        Tool(Icon::Vertical),
        "",
        "Turns the view so that the vertical axis points up.",
    ));
    add(command(
        C::EdgesOnly,
        "Edges\nOnly",
        v,
        t,
        "Show",
        Tool(Icon::FeatureEdges),
        "",
        "Shows only the feature edges.",
    ));
    add(command(
        C::MeshEdges,
        "Mesh\nEdges",
        v,
        t,
        "Show",
        Tool(Icon::MeshEdges),
        "",
        "Shows the edges of all elements.",
    ));
    add(command(
        C::ShellThickness,
        "Shell\nThickness",
        v,
        t,
        "Show",
        Tree(TreeIcon::Shell),
        "",
        crate::settings::SHELL_THICKNESS_TIP,
    ));
    add(command(
        C::BeamProfiles,
        "Beam\nProfiles",
        v,
        t,
        "Show",
        Node(NodeIcon::CrossSections),
        "",
        crate::settings::BEAM_PROFILES_TIP,
    ));
    add(command(
        C::ShowLinked,
        "Show\nLinked",
        v,
        t,
        "Show",
        Node(NodeIcon::Link),
        "",
        crate::settings::SHOW_LINKED_TIP,
    ));
    add(command(
        C::Checklist,
        "Check-\nlist",
        v,
        t,
        "Show",
        Shape(Glyph::Check),
        "",
        "Shows the checklist in the properties panel while nothing is edited.",
    ));
    add(command(
        C::SectionView,
        "Section\nView",
        v,
        t,
        "Cut",
        Tool(Icon::SectionView),
        "",
        "Cuts the model with a plane.",
    ));
    add(command(
        C::SectionOff,
        "Section\nOff",
        v,
        t,
        "Cut",
        Tool(Icon::Remove),
        "",
        "Turns the section view off.",
    ));
    add(command(
        C::ExplodedView,
        "Exploded\nView",
        v,
        t,
        "Cut",
        Tool(Icon::ExplodedView),
        "",
        "Pulls the parts apart.",
    ));
    add(command(
        C::ExplodedOff,
        "Exploded\nOff",
        v,
        t,
        "Cut",
        Tool(Icon::Remove),
        "",
        "Puts the parts back together.",
    ));
    add(tile(
        command(
            C::Screenshot,
            "Screen-\nshot",
            v,
            t,
            "Capture",
            Tool(Icon::Screenshot),
            "",
            "A picture of the 3D view.",
        ),
        Tile::Menu,
    ));
    add(tile(
        command(
            C::ScreenshotClipboard,
            "Copy to Clipboard",
            v,
            t,
            "Capture",
            TileIcon::None,
            "",
            "Copies a picture of the 3D view.",
        ),
        Tile::In(C::Screenshot),
    ));
    add(tile(
        command(
            C::ScreenshotFile,
            "Save As …",
            v,
            t,
            "Capture",
            TileIcon::None,
            "",
            "Saves a picture of the 3D view.",
        ),
        Tile::In(C::Screenshot),
    ));

    // The display of the shown results, on the Display tab of the Results workspace after
    // the view commands.
    let (r, t) = (RESULTS, "Display");
    add(tile(
        command(
            C::Deformation,
            "Deformation",
            r,
            t,
            "Deformation",
            Tool(Icon::Deformation),
            "",
            "How much the shape is deformed: undeformed, true scale, an automatic or your own \
             factor.",
        ),
        Tile::Panel,
    ));
    add(command(
        C::ShowUndeformed,
        "Show\nUndeformed",
        r,
        t,
        "Deformation",
        Tool(Icon::Undeformed),
        "",
        "Draws the undeformed outline behind the deformed shape.",
    ));
    add(command(
        C::GlobalContours,
        "Global\nContours",
        r,
        t,
        "Deformation",
        Tool(Icon::GlobalContours),
        "",
        "Draws the edges of the global model the submodel reads, deformed by the same scale, \
         around the results.",
    ));
    add(tile(
        command(
            C::Representation,
            "Display",
            r,
            t,
            "Contours",
            Tool(Icon::Contours),
            "",
            "Contours, or arrows or LIC streaks along the directions of a vector or tensor \
             field.",
        ),
        Tile::Panel,
    ));
    add(tile(
        command(
            C::ColorLevels,
            "Levels",
            r,
            t,
            "Contours",
            Tool(Icon::Levels),
            "",
            "The number of colour bands of the contours.",
        ),
        Tile::Panel,
    ));
    add(command(
        C::ColorScale,
        "Color\nScale",
        r,
        t,
        "Contours",
        Tool(Icon::ColorScale),
        "",
        "The spectrum and the limits of the contours.",
    ));
    add(command(
        C::Transformations,
        "Transfor-\nmations",
        r,
        t,
        "Contours",
        Tool(Icon::Transformation),
        "",
        "Symmetries and patterns: the results copied around the model.",
    ));
    add(tile(
        command(
            C::Increment,
            "Step, Increment",
            r,
            t,
            "Increment",
            Tool(Icon::Increments),
            "",
            "The step and increment shown.",
        ),
        Tile::Panel,
    ));
    for (cmd, label, icon, tip) in [
        (
            C::FirstIncrement,
            "First",
            Icon::First,
            "Shows the first increment.",
        ),
        (
            C::PreviousIncrement,
            "Previous",
            Icon::Previous,
            "Shows the increment before.",
        ),
        (
            C::NextIncrement,
            "Next",
            Icon::Next,
            "Shows the next increment.",
        ),
        (
            C::LastIncrement,
            "Last",
            Icon::Last,
            "Shows the last increment.",
        ),
    ] {
        add(command(cmd, label, r, t, "Increment", Tool(icon), "", tip));
    }
    add(command(
        C::Animate,
        "Animate",
        r,
        t,
        "Increment",
        Tool(Icon::Animate),
        "",
        "Animates the deformation, the increments or the steps; the animation window sets \
         how.",
    ));
    add(command(
        C::ModeSound,
        "Sound",
        r,
        t,
        "Increment",
        Tool(Icon::Sound),
        "",
        "Plays the mode shapes of a frequency step as sound.",
    ));
    add(tile(
        command(
            C::ExportInp,
            "Export CalculiX Input File",
            SOLVING,
            "",
            "File",
            TileIcon::None,
            "",
            "Writes the model as a CalculiX input file.",
        ),
        Tile::Hidden,
    ));
    add(tile(
        command(
            C::Settings,
            "Settings",
            ALL,
            "",
            "Edit",
            TileIcon::None,
            "",
            "The settings of Nodalis.",
        ),
        Tile::Hidden,
    ));
    add(tile(
        command(
            C::SolverSettings,
            "Solver Settings",
            ALL,
            "",
            "Edit",
            TileIcon::None,
            "",
            "The settings of the solver: threads, work folder and parallel runs.",
        ),
        Tile::Hidden,
    ));
    add(tile(
        command(
            C::UserGuide,
            "User Guide",
            ALL,
            "",
            "Help",
            TileIcon::None,
            "",
            "Opens the user guide in the web browser.",
        ),
        Tile::Hidden,
    ));
    add(tile(
        command(
            C::OpenLogFolder,
            "Open Log Folder",
            ALL,
            "",
            "Help",
            TileIcon::None,
            "",
            "Opens the folder with the session log and the crash reports.",
        ),
        Tile::Hidden,
    ));
    add(tile(
        command(
            C::CheckForUpdates,
            "Check for Updates",
            PROJECT,
            "",
            "Help",
            TileIcon::None,
            "",
            "Asks GitHub whether a newer version is available.",
        ),
        Tile::Hidden,
    ));
    list
}

/// The entries of the command search: each command once, by the workspace it runs in.
pub struct SearchEntry {
    pub cmd: Cmd,
    pub workspace: Workspace,
    pub entry: crate::node_picker::Entry,
}

/// What the command search offers in a window that has these workspaces.
pub fn search_entries(workspaces: &[Workspace]) -> Vec<SearchEntry> {
    let mut seen = Vec::new();
    let mut entries = Vec::new();
    for c in registry() {
        if matches!(c.tile, Tile::Custom | Tile::Menu | Tile::Panel) || seen.contains(&c.cmd) {
            continue;
        }
        let Some(&workspace) = c.workspaces.iter().find(|w| workspaces.contains(w)) else {
            continue;
        };
        seen.push(c.cmd);
        // The search keeps the strings for the life of the program; it lists a fixed set.
        let label: &'static str = Box::leak(c.name().into_boxed_str());
        let keywords: &'static str =
            Box::leak(format!("{} {} {}", c.group, c.tab, c.tip).into_boxed_str());
        entries.push(SearchEntry {
            cmd: c.cmd,
            workspace,
            entry: crate::node_picker::Entry {
                category: workspace.title(),
                label,
                keywords,
                tip: c.tip,
            },
        });
    }
    entries
}

/// The entries of the command search, made once per set of workspaces.
pub fn cached_search(project: bool) -> &'static [SearchEntry] {
    static WITH: LazyLock<Vec<SearchEntry>> = LazyLock::new(|| {
        search_entries(&[
            Workspace::Project,
            Workspace::Geometry,
            Workspace::FeModel,
            Workspace::Results,
        ])
    });
    static WITHOUT: LazyLock<Vec<SearchEntry>> =
        LazyLock::new(|| search_entries(&Workspace::ANALYSIS));
    if project { &WITH } else { &WITHOUT }
}

/// The entries of the command search with those of `workspace` first, which an empty
/// search lists.
pub fn search_order(project: bool, workspace: Workspace) -> Vec<&'static SearchEntry> {
    let all = cached_search(project);
    let (mut first, rest): (Vec<_>, Vec<_>) = all.iter().partition(|e| e.workspace == workspace);
    first.extend(rest);
    first
}

/// The most commands the search lists, and the text of its empty field.
pub const SEARCH_ROWS: usize = 14;
pub const SEARCH_HINT: &str = "Type a command, such as Load or Mesh …";

/// A shortcut as the tooltip shows it.
pub fn tooltip(c: &Command) -> String {
    if c.shortcut.is_empty() {
        format!("{}\n{}", c.name(), c.tip)
    } else {
        format!("{} ({})\n{}", c.name(), c.shortcut, c.tip)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_tile_is_on_a_tab_of_its_workspaces() {
        for c in registry() {
            if matches!(c.tile, Tile::Hidden) {
                continue;
            }
            for w in c.workspaces {
                assert!(
                    c.tab == PINNED || w.tabs().contains(&c.tab_in(*w)),
                    "{:?} is on tab {} that {} lacks",
                    c.cmd,
                    c.tab,
                    w.title()
                );
            }
        }
    }

    #[test]
    fn menu_entries_name_a_menu_tile() {
        for c in registry() {
            if let Tile::In(parent) = c.tile {
                let parent = find(parent).expect("menu is registered");
                assert!(matches!(parent.tile, Tile::Menu | Tile::Split));
            }
        }
    }

    #[test]
    fn search_lists_each_command_once() {
        let entries = cached_search(true);
        let mut cmds: Vec<String> = entries.iter().map(|e| format!("{:?}", e.cmd)).collect();
        let all = cmds.len();
        cmds.sort();
        cmds.dedup();
        assert_eq!(all, cmds.len());
        assert!(entries.iter().any(|e| e.cmd == Cmd::Load));
        // Without the project, its commands are not offered.
        assert!(
            !cached_search(false)
                .iter()
                .any(|e| e.cmd == Cmd::UpdateProject)
        );
    }

    #[test]
    fn every_block_kind_has_a_tile() {
        for block in [
            BlockTile::Materials,
            BlockTile::CrossSections,
            BlockTile::BoxMesh,
            BlockTile::RectangleMesh,
            BlockTile::RefineMesh,
            BlockTile::CadOperation,
            BlockTile::File,
            BlockTile::Parameters,
            BlockTile::Math,
            BlockTile::Switch,
            BlockTile::Measure,
            BlockTile::Study,
            BlockTile::Campbell,
            BlockTile::MeshExport,
            BlockTile::Note,
        ] {
            assert!(find(Cmd::Place(block)).is_some(), "{block:?}");
        }
    }

    #[test]
    fn names_join_the_lines_of_labels() {
        let name = |cmd| find(cmd).map(Command::name);
        assert_eq!(name(Cmd::CrossSection).as_deref(), Some("Cross-Section"));
        assert_eq!(name(Cmd::Screenshot).as_deref(), Some("Screenshot"));
        assert_eq!(name(Cmd::MeshAll).as_deref(), Some("Mesh All Parts"));
    }

    #[test]
    fn labels_have_at_most_two_lines() {
        for c in registry() {
            assert!(c.label.lines().count() <= 2, "{:?}", c.cmd);
        }
    }
}
