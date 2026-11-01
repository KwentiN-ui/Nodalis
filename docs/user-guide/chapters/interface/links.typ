#import "../../template.typ": *

== Linking analyses <links>

Analyses of a project share data through *links* in the project schematic: drag from a pin on
the right of a row to the pin on the left of a row of another analysis. Links are of two kinds:

- A *reference* makes a row of the target the same as the row of the same name in the source:
  the mesh is the mesh, the material is the material. It goes from like to like, and the
  target cannot change what it takes.
- An *operation* hands on what a step of the source produced: its results go into a
  calculation of the target. It starts at a row below #ui("Results"), which only gives, and
  ends where the target uses it: an input of one of its steps under #ui("Setup") >
  #ui("Steps"), its initial conditions, or its #ui("State").

The colour of a wire tells its kind, the row tells where it acts:

#table(
  columns: (30%, 1fr),
  stroke: (x, y) => if y == 0 { (bottom: 0.7pt) } else { (bottom: 0.3pt + luma(200)) },
  inset: (x: 5pt, y: 4pt),
  table.header([*Link*], [*Meaning*]),
  [Reference (blue)], [The target uses the geometry, the mesh, a category of the setup or a
    single item of the source, for example one material (@data-links).],
  [Results (orange)], [A step of the target takes the results of a step of the source: the
    temperatures of a thermal step, the eigenmodes of a frequency step, or the displacements of
    the global model of a submodel (@results-links). A Measure block takes the results of a
    step the same way, to compute a key figure of them (@measure-block).],
  [State (green)], [The target continues the source from the state after one of its steps, as
    further steps of the same CalculiX run (@state-links).],
  [Value (violet)], [A number of the target, such as the edge length of a mesh block, takes
    the value of a parameter (@parameters-block). Like a reference, it is then the same
    number. An input of a parameter study goes back to the blocks before it as a dashed
    wire (@parameter-study).],
)

A block thus reads from top to bottom: where it starts (#ui("State")), what it is (the blue
rows of the definition), what it uses (the orange inputs of its steps in the setup) and what it
produces (#ui("Results"), with pins on the right only).

Each pin has a type: geometry, mesh, setup, results, a number, an input of a parameter
study, or a whole analysis. Pins of model data also have the model space of their block (3D, 2D
plane stress, 2D plane strain or axisymmetric); the tooltip of a row names both. While a wire is
dragged, the pins it can end at get a ring and all others fade. Hovering over a faded pin says
why the link cannot be made there.

#screenshot("typed-pins.png", [The mesh of a 3D analysis dragged to a 2D analysis: every pin
  fades, and the tooltip says why])

The output pane of the project window says what each link does ("Axial-load takes Mesh from
Cantilever") and why a link cannot be made:

- the pins are of different types, for example a mesh dropped on the setup;
- the two blocks have different model spaces ("linked blocks need the same model space"). A
  different unit system is fine, since every block keeps its values in SI units. Numbers have
  no model space;
- the link would go back to an analysis that already comes before the source, and form a
  cycle;
- the item does not fit the slot, for example a material dropped on the sections, or results
  dropped on a step that cannot use them;
- the target continues another analysis and takes its whole model from there (@state-links).

Right-click a pin to remove its links, or use #ui("Unlink") in the analysis window. The target
keeps a copy of what it took, which it can edit from then on.

=== Linking data <data-links>

A data link makes the target use what the source has, live: a change in the source reaches the
target at once. Whole cells can be linked (#ui("Geometry"), #ui("Mesh")), a whole category of
the setup (open #ui("Setup") with the arrow in front of it: #ui("Materials"), #ui("Sections"),
#ui("Constraints") and so on) or a single item of a category (open the category: one material,
one contact pair, one step). A linked category also passes on items added to it later.

A linked #ui("Geometry") takes the shapes only: the parts are read-only, but the mesh setup on
the #ui("Geometry") tab (#ui("Default Mesh Parameters ..."), the #ui("Mesh Setup") items) stays
the target's own, and the target meshes the geometry itself. Several analyses can so mesh the
same geometry, from another analysis or from a file block (@file-block), each its own way. A
target without geometry starts from the mesh setup of the source. A linked #ui("Mesh") takes the
mesh with the setup it was made with; both are read-only then.

#screenshot("linked-geometry-mesh-setup.png", [Geometry linked from a file block: the parts carry
  the link sign, the Meshing Parameters item belongs to the analysis])

The FE Model tree of the target shows only what is set up in that analysis: linked items are
hidden by default, so that the tree lists the inputs that matter for it. Categories stay in the
tree, also when everything in them is linked, so that items of the analysis itself can be added;
a linked mesh or step leaves just its category row with the link sign. Tick #ui("Show linked
items") above the tree, in the #ui("View") menu or in the settings (#ui("Graphics")) to list
them; the choice is stored in the settings. Hidden items are still part of the model: dialogs
offer the linked sets, materials and so on as before, and the solver gets them.

#screenshot("linked-hidden.png", [The FE Model tree of the target with linked items hidden: only
  the own material and the own step are listed])

#screenshot("linked-shown.png", [With #ui("Show linked items") ticked, the linked ones appear with
  their link sign])

In the window of the target, linked items are read-only. When shown, a link sign marks them in the tree,
and their tooltip names the analysis they come from and says that they change there. The
commands that would change them are refused with the same hint; a double-click opens a small
dialog instead of the item's own:

#screenshot("linked-material.png", [A material taken from another analysis in the tree of the
  target])

#screenshot("linked-item-dialog.png", [Double-clicking a linked item])

#fields(
  [#ui("Open <analysis>")], [Opens the window of the source analysis to edit the item there.],
  [Unlink], [Removes the link. The target keeps a copy of the item and can edit it.],
)

A linked mesh follows the source: when the source is meshed again, the target's solution is out
of date.

==== Pins: single values and collections

The pins on the left of the rows have two shapes, like the sockets of a node editor:

#table(
  columns: (22%, 1fr),
  stroke: (x, y) => if y == 0 { (bottom: 0.7pt) } else { (bottom: 0.3pt + luma(200)) },
  inset: (x: 5pt, y: 4pt),
  table.header([*Pin*], [*Takes*]),
  [Round], [One value: the geometry, the mesh, the whole setup, a single item or the results
    of a step. A new wire replaces the old one, and the output pane says so ("Modal no longer
    takes Setup > Materials > Steel from Static").],
  [Tall, rounded], [A collection: the categories of the setup (#ui("Materials"),
    #ui("Sections"), #ui("Steps") and so on). It takes any number of wires, from one analysis
    or from several, and the items of the target that are its own stay.],
  [Green square], [The state after a step of another analysis (@state-links).],
)

#screenshot("project-pins.png", [The materials of Static linked to Modal as a whole: Steel and
  Aluminium come through, Copper is left out and Rubber is Modal's own])

A linked category is a bundle of links of its items. Open the category in both blocks to see a
wire per item. Items the source gets later come through as well. To leave one item out,
right-click its pin in the target, or double-click it in the tree of the target's window and
choose #ui("Unlink"): the target keeps its copy and can edit it, and the pin of the category
turns lighter, with a dot. Its tooltip counts the items that come through ("2 of 3 items from
Static"). Drag the item's own wire again to take it back.

When two links bring an item of the same name, the one made later wins. An own item of the target with the name of a linked one is covered by it, and its copy
of the linked item stays once that item is left out or the link is removed. Linked steps come
before the target's own steps, in the order of the links. A link of the whole #ui("Setup") is a
single value: it takes all categories of the source in place of the target's.

=== Taking results of a step <results-links>

The rows below #ui("Results") of a block are its steps: what each step produces. They have a pin
on the right only. What a step of another analysis uses is an *input* of that step: open
#ui("Setup") > #ui("Steps") of the target and then the step with the arrow in front of it, and
its inputs are listed below it, each with an orange pin:

#table(
  columns: (22%, 1fr, 1fr),
  stroke: (x, y) => if y == 0 { (bottom: 0.7pt) } else { (bottom: 0.3pt + luma(200)) },
  inset: (x: 5pt, y: 4pt),
  table.header([*Input*], [*Takes the results of*], [*Used as*]),
  [#ui("Temperatures") (every step that takes defined fields)],
  [a heat transfer, coupled or uncoupled temperature-displacement step],
  [the temperatures of that step: a defined field named #ui("Linked Temperatures")
    (`*TEMPERATURE, FILE=..., BSTEP=n`), see @defined-fields],
  [#ui("Eigenmodes") (modal dynamics and steady state dynamics steps)],
  [a frequency step with #ui("Storage") on],
  [the eigenmodes these steps work on: the `.eig` file of the other run (@modal-dynamics)],
  [#ui("Global results") (every step, and a row of the block of a submodel)],
  [any step of the global model],
  [the displacements on the cut boundary of a submodel (@submodel)],
)

Results dropped on the row of the step itself, while it is closed, go to the input that fits: eigenmodes to #ui("Eigenmodes"), temperatures to #ui("Temperatures"), anything
else to #ui("Global results"). Once the step is opened, the wire is seen to end at that input.
Results dropped on an input they do not fit are refused, and the output pane says why
("Heat is no frequency step").

The temperatures of a thermal step can also be dropped on #ui("Initial Conditions") under
#ui("Setup"): the analysis then starts at them (`*INITIAL CONDITIONS, TYPE=TEMPERATURE` on every
node).

#screenshot("temperature-links.png", [A heat transfer analysis giving its temperatures to a
  structural analysis: one link to its initial conditions ("starts at the temperatures of
  ..."), one to the #ui("Temperatures") input of its step ("takes the temperatures of ... in
  Step-1")])

- Temperatures and eigenmodes are transferred node by node, so both analyses need the same mesh:
  link the mesh, or continue from a state (@state-links). Otherwise the output pane says why
  nothing is taken. A submodel has its own, finer mesh.
- A frequency step without #ui("Storage") cannot give its eigenmodes; the link is refused with
  a hint to turn it on.
- A step that works on the eigenmodes of another analysis needs no frequency step before it.
  Its boundary conditions are written before the first step, because CalculiX takes no new
  ones in a modal or steady state dynamics step.
- Defined temperatures need initial temperatures: a link to #ui("Initial Conditions") provides
  them, and the model check counts it (@model-check).
- When the source has no results yet, #ui("Solve") and #ui("Update Project") solve it first.
  When the source is solved again, the analyses that use its results are solved after it. A
  source whose results are out of date is not solved again on its own; its block shows the
  circular arrow, and #ui("Update Project") brings both up to date.
- Before the run, Nodalis copies the result files it needs next to the input file, because
  CalculiX reads them from there.
- #ui("Unlink") takes back what the link set: the defined field, the eigenmodes or the initial
  temperatures.

=== Continuing an analysis after a step <state-links>

CalculiX solves the steps of a run one after the other, each starting from the state the step
before it left. A *state link* lets an analysis take that state from a step of another analysis
and continue with steps of its own, as if they stood behind that step in one input file. Where
Ansys Workbench links a static analysis to a pre-stressed modal analysis, Nodalis links any
step to any following steps: a load history that branches after a pre-load into several load
cases, a frequency step after a static step, or a modal dynamics step after the frequency step
of another analysis.

Drop the results of a step on the #ui("State") row at the top of another block. The target then
*continues* the source:

- It takes the geometry, the mesh, the whole setup and the steps up to and including that step
  from the source. All of it is read-only in its window and marked with the link sign like
  other linked items; the steps it took are listed before its own. New steps are added as
  usual and are its own; as always, a new step starts with the boundary conditions and loads
  of the step before it.
- Its #ui("State") row reads #ui("After <step> of <source>"). In its block, #ui("Results") lists
  only the steps of its own. The steps it took over are folded into one row right below
  #ui("Results"), #ui("2 steps from <source>"), in the link colour and with the link sign; it has
  no pins, and a click on its arrow lists the steps, greyed, as a reminder of what the state
  contains. Their results are in the block of the source. Every step shows the symbol of its type, like in
  the tree.
- A state link replaces every other link of the target: while it continues another analysis,
  nothing else can be linked to it.
- #ui("Unlink") on the #ui("State") pin, or on a taken item in the window, makes the analysis
  independent again. It keeps copies of the model and of the steps it took, so it still solves
  the same problem, now on its own.

#screenshot("project-inherited-steps.png", [The block of an analysis that continues Static after
  Step-1: #ui("Results") lists only its own Step-2, the step it took over is folded into one row
  and shown greyed once opened])

#screenshot("continued-analysis.png", [The tree of an analysis that continues another one: the
  model and the first two steps come from there, the third is its own])

*Continue from here.* The quickest way to a continuation: drag the pin of a step to empty space
and pick #ui("Continue from here") from the menu that opens. A new analysis named
#ui("<source> continued") appears at that place, already continuing from that step, with the model
space and unit system of the source. Open it and add its steps.

#screenshot("continue-from-here-menu.png", [Dragging the results of a step to empty space])

*Solving.* CalculiX solves only the own steps of the target: the run reads the state CalculiX
wrote after the step in the source's run (`*RESTART, READ`) and continues. The monitor says so
("Restart after step 2 of 3: the steps up to it are unchanged; CalculiX reads their state from
Cantilever and solves the rest", or "from the earlier run" once the target has run before). The
results of the target contain all steps, those of the source and its own, as if solved in one
go.

- When the source has no results yet, #ui("Solve") and #ui("Update Project") solve it first.
- CalculiX takes no new sets after reading a state. Nodalis therefore writes the sets the
  target's steps need into the input file of the source as well. If they are missing because
  the source was solved before the steps were added, the target is solved in full instead,
  with the source's steps repeated in its own run; the results are the same, it only takes
  longer. The monitor says #ui("Full run") and why.
- CalculiX writes a state only after steps over time (static, dynamic, heat transfer, coupled
  and uncoupled temperature-displacement, visco), not after a frequency, buckling or
  perturbation step. A target that continues after such a step is solved in full as well.
- The state files are kept in the work directory, not in the project file (@restart). On a
  computer without them the target is solved in full.

=== Split and merge <split-merge>

The context menu of a block turns steps into continuations and back, without changing what is
solved:

#fields(
  [#ui("Split After") > step], [The steps after the chosen step move to a new analysis that
    continues this one from the state after it. Analyses that took results of those steps now
    take them from the new analysis. The context menu offers every step but the last.],
  [#ui("Merge into <analysis>")], [Only for an analysis that continues another one from its last
    step. Its own steps go back to that analysis, links to its steps move along, and it is
    removed from the project. An analysis that continues from an earlier step cannot be merged,
    because its steps would have to go between the other's.],
)

#screenshot("split-after-menu.png", [Splitting an analysis after its first step])

A split is useful when one of several load cases after a common pre-load should be computed
without the others, or to show the branches of a load history in the schematic; a merge brings
the steps back into one block when the branch is no longer needed.
