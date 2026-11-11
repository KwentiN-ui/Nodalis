#import "../../template.typ": *

== Projects and analyses <project-window>

A Nodalis project (`.ndl`) holds several analyses, like a project of Ansys Workbench. The
Project workspace (@workspaces), also called the *project window* in this guide, shows each
analysis as a block in the project schematic. A double-click on a block opens the analysis in
the Geometry, FE Model or Results workspace of the same window (@main-window); with
#ui("Open in New Window") it opens in a window of its own. A new project starts with one empty
analysis.

#screenshot("project-window.png", [Project window with three analyses: #ui("Cantilever
  continued") continues #ui("Cantilever") after its last step (green wire), #ui("Axial-load")
  takes its mesh and materials (blue wires)])

New analyses are empty and independent of the others. To start from an existing one, use
#ui("Duplicate"): the copy takes over geometry, mesh, model and results. To share parts of an
analysis, link them in the schematic (@links): a linked item changes in one place and is used in
both.

=== Creating an analysis <new-analysis>

#menu("File", "New Analysis"), a double-click or right-click on the background of the
schematic or #ui("Analysis") in the block search (@block-search) open #ui("Model Properties") for the new analysis
(@model-properties). Choose its *model space* (3D, plane stress, plane strain or axisymmetric)
and its *unit system* there.

#screenshot("new-analysis-dialog.png", [Model Properties for a new analysis])

#note[The model space and the unit system of an analysis are fixed once it is created. In the
  analysis window, #menu("Material", "Model Properties") shows them, but they cannot be
  changed; create a new analysis instead. Analyses of one project may use different unit systems,
  because all values are stored in SI units and only shown in the units of their analysis
  (@unit-systems).]

=== The blocks of the schematic

An icon in the top left corner of each block shows its model space: a cube for 3D, a thin plate
for plane stress, a long prism for plane strain and a profile turned about its axis for
axisymmetric models. Hover over it for the name. Below the header, each block lists the cells of
its analysis from top to bottom, and below them the steps of the analysis as they are defined
at the moment:

#fields(
  [State], [Where the analysis starts: by itself, or from the state after a step of another
    analysis whose results are dropped here (@state-links). The row then says
    #ui("State of <analysis>").],
  [Geometry], [The imported CAD geometry. A dash means the analysis has no geometry because
    its mesh was imported from an input file.],
  [Mesh], [The mesh of the parts.],
  [Setup], [The model definition; it is there as soon as the analysis has a step. The arrow in
    front of it opens its categories (#ui("Features"), #ui("Materials"), #ui("Sections"),
    #ui("Constraints"), #ui("Contacts"), #ui("Amplitudes"), #ui("Initial Conditions"),
    #ui("Steps")), and those open to their single items, so that each can be linked on its
    own. A step opens to its inputs for results of other analyses (@results-links).],
  [Solution], [The last run whose results the analysis keeps.],
  [Results], [The results of that run, followed by one row per step. The row of a step gives
    the results of that step to other analyses (@results-links, @state-links); it takes
    nothing, so it has a pin on the right only.],
)

The symbol on the right of a cell shows its state; its tooltip says what it means:

#fields(
  [Green tick], [Up to date.],
  [Question mark], [Not there yet: nothing imported, meshed, set up or solved.],
  [Circular arrow], [Out of date: the cell was made from inputs that have changed since.
    #ui("Solve") makes it again.],
  [Red cross], [CalculiX reported errors; results may still be there.],
)

Nodalis keeps a fingerprint of the inputs each cell was made from, and only cells whose inputs
changed are out of date. New meshing settings of one part make only the mesh of that part out
of date, and with it the solution. A new load makes the solution out of date but not the mesh.
Changes that the solver does not see, such as result paths or planes, leave the solution up to
date. Within the solution, Nodalis also remembers which steps are unchanged (see below).

A mesh taken from a mesh block is out of date as soon as the block's parameters change, also in
analyses that take it through another analysis: the analysis still holds the block's old mesh.
#ui("Solve") in the analysis window then has the project window make the block's mesh anew and
take it over before the solver runs, so the results always belong to the block's current mesh.

The same holds for results and states taken from another analysis: when the source analysis is
out of date, so is the analysis that takes its results or continues from its state, and
#ui("Solve") in its window solves the source again first instead of taking its old results.

This is decided per step. The solver computes the steps one after the other, so the results of a step
depend only on the model and the steps up to it. A link from the results or the state of a step
therefore only turns out of date when the model, that step or a step before it changes: adding
a frequency step after the static step a submodel takes its global results from makes the global
analysis out of date, but the submodel stays up to date, and #ui("Update Project") solves only
the global analysis. Eigenmodes are the exception: the solver writes them anew with every frequency
step, so an analysis that takes them is out of date with any change of the source.

#screenshot("later-step-keeps-submodel.png", [#ui("Step-2") was added to #ui("Global") after
the submodel #ui("Local") was solved from #ui("Step-1"): only #ui("Global") is out of date])

#screenshot("mesh-block-out-of-date.png", [After #ui("nx") of the box changed from 10 to 20, the
mesh, solution and results of the analysis that takes the box's mesh are out of date])

A green trace at the left of the rows of the steps, under #ui("Setup") > #ui("Steps") and under
#ui("Results"), shows how the state flows through them (@step-handover). A filled dot is a step
that passes its end state on; the line joins it to the next one. A ring is a step that leaves the
state as it found it: a frequency, buckle, complex frequency or steady state dynamics step. When
it is computed about the end of the step before, a short branch joins it to the line; computed
about the unloaded model, it stands apart. The tooltip of each step row says where the step starts
and what it passes on.

#screenshot("chain-trace.png", [The trace of the steps: Modes branches off after Preload, Unload
  continues Preload, Free is computed about the unloaded model])

The pins on both sides of a row take and give links. Blue wires are references (geometry, mesh,
setup items: the target has the same as the source), orange wires carry results into the inputs
of steps and green wires carry the state after a step (@links). A round pin on the left takes
one wire, a tall one (the categories of the setup) takes any number, and the green square takes
the state of another analysis (@data-links).

=== Working with the schematic

- *Click* an analysis block to make it the active analysis of the app bar (@app-bar).
- *Double-click* a block to open its analysis in the main window. If it is open in a window
  of its own, that window comes to the front, also when it is minimized. Wayland does not let
  a program bring its own window to the front; there the taskbar marks the window instead.
  While the analysis is solved in the background, it opens once the solve is done. The header
  of an open analysis says #ui("(open)").
  Double-clicking a row opens the analysis in the matching workspace: #ui("Geometry") and
  #ui("Mesh") in the Geometry workspace, #ui("Setup") and its categories in the FE Model
  workspace, the rows of the steps likewise with the step selected in the tree, and #ui("Results") in the Results workspace.
- *Drag* a block by its header to move it. Drag the background to select the blocks in a
  rectangle, and click it to clear the selection. Drag with the middle mouse button to pan,
  scroll to pan up and down, and scroll with #key("Ctrl") held to zoom, from a tenth of the
  true size up to four times it; the text is drawn again for the zoom, so it stays sharp when
  magnified. #menu("Arrange", "Show All") (#key("Home")) fits all blocks into the view.
- The *minimap* in the lower right corner shows all blocks and frames in small, with the part
  in view outlined; click or drag in it to move the view there.
  #menu("Arrange", "Minimap") shows or hides it.
- The *floating bar* at the bottom of the schematic holds #ui("Show All"), #ui("Minimap"),
  #ui("Compact View"), #ui("Frame") and #ui("Duplicate"), the same as on the #ui("Arrange") tab.

#screenshot("schematic-minimap.png", [The minimap: the frames and blocks of the project in
  small, the outline is the part in view])

- *Drag* from a pin on the right of a row to a pin on the left of a row of another block to
  link them (@links). Right-click a pin to remove its links.
- *Right-click* a block for its context menu.
- *Double-click* or *right-click* the background to search the blocks to add there
  (@block-search, @source-blocks).

#screenshot("project-node-menu.png", [Context menu of an analysis])

#fields(
  [Open], [Opens the analysis window.],
  [Solve], [Solves the analysis, like #ui("Solve") in its window (@analysis): in the background
    with the solution monitor if its window is closed (@solution-monitor), else in its window.],
  [#ui("Split After") > step], [Moves the steps after the chosen step into a new analysis that
    continues this one from the state after it (@split-merge). Only for analyses with more than
    one step.],
  [#ui("Merge into <analysis>")], [Only for an analysis that continues another one from its last step:
    gives its steps back to that analysis and removes itself (@split-merge).],
  [Compact View (#key("H"))], [Shows only the header and the rows with a wire
    (@frames-notes); #ui("Show All Rows") shows the rest again.],
  [Group Blocks (#key("Ctrl+G"))], [Puts the block, or the selected blocks it is one of, into
    a new node group (@node-groups).],
  [Frame Blocks (#key("Ctrl+J"))], [Puts the block, or the selected blocks it is one of, in a
    new frame; #ui("Remove from Frame") (#key("Alt+P")) takes it out.],
  [Rename ... (#key("F2"))], [Gives the analysis a new name. Names are unique within a project.],
  [Duplicate (#key("Shift+D"))], [Adds a copy of the analysis, named "Copy of ...". A copy
    of a block in a frame goes into the same frame.],
  [Delete (#key("X"))], [Removes the analysis from the project at once; the note at the
    bottom of the schematic offers #ui("Undo"). An open window closes.
    Links to and from it are removed; analyses that took items from it keep their copies.],
)

==== Adding blocks by search <block-search>

Like the add menu of Blender, a double-click or right-click into empty space of the schematic shows every kind
of block Nodalis can add, grouped as on the tabs of the ribbon. Just start typing: the search
field has the focus and finds blocks by fuzzy matching, so letters in order are enough
(#ui("rct") finds #ui("Rectangle")). It also knows other words for a block, such as
#ui("hexahedra") for #ui("Box") or #ui("library") for #ui("Materials"). Search results show
their group on the right.

#key("Enter") adds the highlighted block, the best match while you type, where you
clicked; #key("Up") and #key("Down") move the highlight, a click adds any block, and
#key("Esc") or a click elsewhere closes the search. #ui("Analysis") asks for the model
properties first (@new-analysis).

#screenshot("node-picker.png", [All blocks after a double-click or right-click (left) and the search for
  "mesh" (right)])

==== Frames, notes and compact blocks <frames-notes>

Large schematics stay readable with three things borrowed from Blender's node editor:

- A *frame* is a named, tinted rectangle around a group of blocks, such as "Load cases" or
  "Thermal chain". Select the blocks and press #key("Ctrl+J"), or choose
  #ui("Frame Blocks") in the context menu of a block: the frame is drawn around them and its
  label is ready to be typed. The frame follows its blocks and grows when one moves out;
  drag the frame itself to move it with all its blocks. A block dragged and dropped onto a
  frame joins it; #key("Alt+P") or #ui("Remove from Frame") takes it out again. Double-click
  the label to rename the frame, and right-click the frame for its #ui("Color") or to
  #ui("Remove Frame"), which leaves its blocks where they are. A double-click into the frame
  below its label searches the blocks to add, as on the empty background.
- A *note* is a block with a text and no pins (#menu("Annotate", "Note"), or "note" in
  the block search). Click its text to write; it is kept once you click elsewhere, and
  #key("Ctrl+Z") undoes the whole edit.
- A *compact* block shows only its header and the rows a wire ends at, so the wires stay
  but the rows in between are gone. Click the triangle at the left of its header or press
  #key("H"); a double-click on the header of a compact block opens it again.

Frames, notes and compact blocks are saved with the project, and undo and redo cover them.

#screenshot("schematic-zoom.png", [A block zoomed in: the text stays sharp])

#screenshot("schematic-frames.png", [Two frames, a note and a compact analysis that shows only
  its linked rows])

#screenshot("frame-menu.png", [The context menu of a frame with its colours])

==== Node groups <node-groups>

A *node group* puts blocks together into one block, as in Blender: a mesh with its analysis
and the Measure blocks of its results, say, becomes one block #ui("Group") with the pins of
what goes in and out. Select the blocks and press #key("Ctrl+G"), or choose
#ui("Group Blocks") in the context menu of a block. The group block takes their place; every
wire that ran into or out of the blocks now runs to a pin of the group, an input on the left
or an output on the right, named after the pin it comes from.

Double-click the header of the group, press #key("Tab") with the pointer on it, or choose
#ui("Open Group") in its context menu to open it: the schematic then shows only the blocks
inside, with a *Group Input* block on the left that gives the inputs of the group and a
*Group Output* block on the right that takes its outputs. The path at the top left of the
schematic tells where you are, such as #ui("Bracket › Group › Mesh study"); click a part
of it to go back there. #key("Tab") on the background or #key("Ctrl+Tab") goes out one
level. Groups can be put into groups, as deep as you like.

#screenshot("node-groups.png", [Inside a group: the Group Input block gives the thickness to
  the mesh, the Group Output block takes the stress the group gives out. The path at the top
  left goes back to the project.])

The Group Input and Group Output blocks belong to their group and cannot be deleted. Each has
an *empty pin* at the bottom, a ring without fill. Drag a wire from the empty pin of the
Group Input block to a block inside, or from a block inside to the empty pin of the Group
Output block: the group gets a new input or output of that type, and the next empty pin
comes below. Outside, link the new pin of the group like any other. Double-click the row of
an input or output to rename it; once it has no wires left on either side it goes.

The same empty pin marks every block that takes any number of inputs: the choices of a
Switch block (@math-switch) and the outputs and inputs of a parameter study (@parameter-study).
A wire dropped on it adds one more.

Groups only arrange the schematic. A link through a group is the link between the blocks at
its ends, which Update Project follows as before; it is made once the wires reach from one
block to the other through the pins of the groups, and removed once one of them goes.
#key("Ctrl+Alt+G") or #ui("Ungroup") puts the blocks of a group back where the group is,
with the wires between them as before. #key("Shift+D") duplicates a group with the blocks
inside and the links between them; #key("X") deletes it with all its blocks, and undo
brings them back.

==== Keys of the schematic <schematic-keys>

While the mouse pointer is over the schematic and no text field is being edited, these keys
work as in Blender. They act on the selected blocks, or on the block under the pointer when
none is selected. Drag a rectangle on the background to select the blocks in it, or click a
block with #key("Shift") held; with #key("Shift") a rectangle adds to the selection, and
#key("Ctrl") and a click removes a block from it. A click on the background clears it. The
#ui("Arrange") and #ui("Annotate") tabs of the ribbon offer them for the selected blocks.

#fields(
  [#key("Shift+A")], [Searches the blocks to add at the pointer (@block-search).],
  [#key("X"), #key("Delete")], [Deletes the blocks at once; #ui("Undo") in the note at the
    bottom, or #key("Ctrl+Z"), brings them back.],
  [#key("Shift+D")], [Duplicates the blocks.],
  [#key("H")], [Shows the blocks compact, or in full if they all are compact.],
  [#key("Ctrl+G")], [Puts the blocks into a new node group (@node-groups).],
  [#key("Ctrl+Alt+G")], [Puts the blocks of the groups back, without the groups.],
  [#key("Tab")], [Opens the group under the pointer or selected; on the background, goes out
    of the group shown.],
  [#key("Ctrl+Tab")], [Goes out of the group shown, to the level around it.],
  [#key("Ctrl+J")], [Puts the blocks in a new frame.],
  [#key("Alt+P")], [Takes the blocks out of their frame.],
  [#key("F2")], [Renames the block.],
  [#key("Home")], [Fits all blocks and frames into the view.],
)

=== Menus and ribbon of the project window

#screenshot("project-file-menu.png", [The #ui("File") menu of the Project workspace])

The menus of the Project workspace hold only what concerns the whole project; the blocks are
placed and worked on with the ribbon (@ribbon) and the keys above.

#fields(
  [File > New Project], [Closes the project and starts a new one with #ui("Analysis-1").],
  [File > Open ... (#key("Ctrl+O"))], [Opens a project. Its analyses open when you
    double-click them.],
  [File > Open Recent], [Lists the projects opened or saved last that still exist.],
  [File > Save (#key("Ctrl+S"))], [Saves the project with all analyses, also those whose
    windows are open (@files).],
  [File > Save As ... (#key("Ctrl+Shift+S"))], [Saves the project under a new name.],
  [File > Save Without Results ...], [Saves a copy without the results the analyses keep.],
  [Edit > Undo, Redo], [Undo and redo in the schematic (@undo).],
  [Edit > Settings ...], [The settings (@settings); they apply to all windows.],
  [Help], [#ui("User Guide") opens this guide, #ui("Open Log Folder") the folder with the
    log and the crash reports (@crash-logs), #ui("Check for Updates ...") looks for a new
    version.],
)

The ribbon tabs of the Project workspace:

#fields(
  [Project], [File commands, #ui("New Analysis") (@new-analysis), and #ui("Open"),
    #ui("Open in New Window"), #ui("Solve"), #ui("Duplicate"), #ui("Delete") and
    #ui("Rename") for the selected block.],
  [Sources], [Materials (@materials-block), cross-sections, box, rectangle and refined
    meshes (@mesh-block), CAD operations and files.],
  [Parameters], [Parameters, Math and Switch blocks.],
  [Evaluate], [Measure, parameter study, Campbell and mesh export blocks.],
  [Annotate], [Notes and frames (@frames-notes).],
  [Arrange], [#ui("Duplicate"), #ui("Delete") and #ui("Compact View") for the selected
    blocks, as their keys do (@schematic-keys); #ui("Show All") (#key("Home")) fits all
    blocks into the view, #ui("Minimap") shows or hides the minimap.],
)

#key("F5") or #ui("Update Project") solves every analysis that is not up to date
(@update-project); #key("F5") in an analysis solves only that analysis. #ui("Monitor") shows
the #ui("Analyses") tab of the output pane, with the analyses being solved and the output of
the solver (@solution-monitor).

#ui("Update Project"), #ui("Stop") (stops the update and kills the running analyses) and
#ui("Monitor") stay on the right edge of the ribbon on every tab of the Project workspace.

Closing the main window closes Nodalis with all analysis windows. #menu("File", "Close
Analysis") in an analysis, or closing its own window, keeps the analysis in the project, with
all changes; save the project to write them to the file. An analysis window that is meshing or
solving cannot be closed.

=== Update Project <update-project>

#ui("Update Project") solves every analysis whose mesh or solution is out of
date or missing, like #ui("Update Project") in Ansys Workbench. Analyses without a mesh,
geometry or step are skipped and listed in the output pane. Mesh blocks whose mesh is out of date make it anew
first (@mesh-block).

- Analyses are solved in the order of their links: an analysis that takes results or a state
  from another one waits until that one is solved. An analysis upstream is solved only when it
  is not up to date itself; when it is solved again, the analyses that use its results are
  solved after it.
- Independent analyses are solved at the same time, up to #ui("Parallel analyses") of the
  CalculiX settings (default 2), each with the number of threads set there (@settings). With
  #ui("Automatic") threads the analyses solved at the same time share the processor cores; with
  a fixed number, set #ui("Threads") times #ui("Parallel analyses") to at most the number of
  processor cores.
- An analysis whose window is closed is solved in the background, without opening its window:
  Nodalis meshes its geometry if the mesh is missing or out of date, writes the input file,
  runs the solver and keeps the results in the project, as #ui("Solve") in the window does. The
  cells of its block show the progress. An analysis whose window is open is solved in its
  window, which shows the new results when it is done.
- If CalculiX fails, only the analyses that depend on the failed one are skipped, independent
  ones go on. The output pane sums up with #ui("Project updated") or
  #ui("Update finished with errors"); the solution monitor tells what went wrong.

==== Updating one block <update-block>

When the schematic holds independent branches, one of them can be brought up to date without
the others: click the amber circular arrow of a block (or the blue question mark of a
#ui("Solution") or #ui("Results") row that was never solved). Nodalis updates that block and
every block it takes data from, directly or through others, the way #ui("Update Project") does:
mesh blocks make their meshes anew, analyses are solved in the order of their links, Campbell
blocks sweep and mesh export blocks write their files. Blocks that are neither the clicked one
nor upstream of it stay as they are. Clicking the arrow of a parameter study solves its design points
(@parameter-study). The symbols cannot be clicked while an update is under way. The output pane ends
with #ui("<block> updated").

#screenshot("update-block-hover.png", [Hovering the out-of-date symbol of
  #ui("Beam-A-Variant"); #ui("Beam-B") is a branch of its own])

#screenshot("update-block-done.png", [After the click: #ui("Beam-A-Variant") and #ui("Beam-A"),
  whose setup it takes, are solved; #ui("Beam-B") is still out of date])

=== Solution monitor <solution-monitor>

The project window has its own monitor, like the monitor of an analysis window (@monitor) but
for all analyses of the project. It is the #ui("Analyses") tab of the output pane below the
schematic (@output-pane) and opens when an update or #ui("Solve") on a block starts;
#ui("Monitor") or #ui("Monitor") on the ribbon switches between it
and the #ui("Output") tab.

#screenshot("solution-monitor.png", [Solution monitor after Update Project solved three
  analyses in the background])

The list shows each analysis being solved, queued or done, with its status (preparing,
meshing, running, keeping results, solved, failed or stopped), its running time and the current
step and increment of CalculiX. Analyses solved in their open window are marked
#ui("(window)"). Right of the list is the output of the analysis picked in it: the lines of
Nodalis (meshing, model checks, restart, keeping the results) around those of CalculiX, which
#ui("Filter") narrows down to the lines containing a text.

#fields(
  [Stop], [Stops the solver at once for the picked analysis, or takes it out of the queue. It
    is not solved; the update goes on with independent analyses.],
  [Stop All], [Stops the update and every solve.],
  [Open Analysis], [Opens the window of the picked analysis in the Results workspace; a
    double-click on its name does the same. An analysis still being solved in the background
    opens once it is done.],
  [Clear Finished], [Removes the analyses that are done from the list.],
  [Profile], [Shows where the solver spent its time for the picked analysis, left of the
    output, once it is done (@solver-profile).],
  [Copy Report], [Copies the version, the system, the size of the model and the profile of the
    picked analysis as text, e.g. for an issue on GitHub.],
)
