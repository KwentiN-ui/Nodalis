#import "../../template.typ": *

= Solving <analysis>

== Solving an analysis

Each analysis of the project is solved in its window. The solver uses the settings under
#menu("Edit", "Settings ...", "Solver") (@settings) and runs in the folder
`<work directory>/<project name>/<analysis name>`, so the analyses of a project never overwrite
each other's files.

#screenshot("solve-toolbar.png", [The state of the solution in the app bar, and Solve, Check Model, Stop and
  Monitor on the right edge of the ribbon])

#fields(
  [Solve (#key("F5"))], [Makes the analysis up to date, like #ui("Update") in Ansys
    Workbench: first it meshes the parts whose mesh is missing or out of date, then it runs
    the solver if the results are missing or out of date, and keeps the new results. If nothing
    changed since the last run, it only says so.],
  [Solve Again], [Runs the solver even if the results are up to date.],
  [Check Model], [Lets the solver only read and check the model and its steps, without
    solving it.],
  [Stop], [Stops meshing for a solve, or the running solver.],
  [Monitor], [Shows the #ui("Analysis") tab of the output pane (@monitor).],
  [Open Results], [On the #ui("Results") tab: opens the results the analysis keeps.],
  [Remove], [On the #ui("Results") tab: drops the results the analysis keeps, after asking
    (@bind-result).],
)

#ui("Solve"), #ui("Stop") and #ui("Monitor") are on the right edge of the ribbon on every tab; the state of the
solution is in the app bar. Which cells are out of date is decided
as in the project schematic (@project-window): changing a load makes only the solution out of
date, changing the meshing settings of a part makes its mesh out of date, and result paths or
planes change nothing that the solver needs.

The input file is named after the analysis, for example `Cantilever.inp`. Old result files of
the analysis in its folder are deleted before the solver starts. Nodalis refuses to run a model
without mesh, without step or with all steps deactivated, and it stops if the input file cannot
be written (for example because a region is empty).

=== Monitor <monitor>

The monitor is the #ui("Analysis") tab of the output pane below the 3D view; it comes to the
front when a run starts, so the model and its results stay in view while the solver runs.
Its top row has #ui("Kill") to stop the solver, #ui("Results") to open the results once the run is
finished, the #ui("Filter") field and the status with the running time and the current step,
increment and iteration; below it is the output of the solver. The #ui("Output") tab goes back to
the messages, and the pane can be made taller by dragging its upper edge.

#screenshot("monitor.png", [The Analysis tab after a completed run])

The progress next to the running time follows the solver from its first iteration
on: the step, the increment, the attempt once the solver has cut the increment back (attempt 2
and later), the iteration, the size of the increment and the time reached in the step. So a
nonlinear step that needs several cutbacks before its first increment converges still shows
where it is, long before the status file has a line. After the run, the progress shows the
last converged increment from the status file (`.sta`).

#screenshot("monitor-progress.png", [A nonlinear step on its third attempt of the first
  increment, with the increment size cut back to 0.0625])

The #ui("Filter") field shows only the lines that contain the text you type,
ignoring upper and lower case. With "average force", for example, you see the average force of
every increment one below the other. The filter applies while the solver is still running, and
#ui("Clear") or an empty field shows all lines again.

#screenshot("monitor-filter.png", [The Analysis tab showing only the lines with "cpu"])

The monitor keeps the last 20000 lines of a run, so a long run with many increments stays fast.
Once there are more, the oldest lines go and the top row tells how many, for example
#ui("10085 earlier lines not kept"); the new lines keep coming in at the bottom.

#screenshot("monitor-long-output.png", [The end of a long run, its first lines dropped])

The status at the end is #ui("completed"), #ui("finished with errors, results available"),
#ui("failed") or #ui("killed"). If the solver failed, it reports the causes it found
and Nodalis adds a line "Possible cause: ..." with a remedy, for example for distorted
elements, missing material data, rigid body motion or missing convergence.

== Analyses that continue others <restart>

One analysis can continue another one after a step, through a state link in the project
schematic (@state-links): it starts from the displacements, stresses and contact state the other
analysis reached. When it runs, the solver computes the steps of the other analysis up to the
linked one and then its own steps. Changing a later step of an analysis solves all its steps
again.

== Results kept with the project <bind-result>

When a run is finished, its analysis keeps the results: the input file and the results of the
run are saved in the project file with the next #menu("File", "Save Project"), together with a
fingerprint of what the run was made from. The results open in the Results workspace. The
output pane shows how much they add to the project file; they are stored compressed. Keeping a
large result takes a few seconds and runs in the background. A new run replaces the results the
analysis keeps.

The solver hands its results to Nodalis in memory. They are kept as one binary results file
(`.ndlr`, written next to the input file in the work directory) that holds the mesh the results
are on and every field and history output of the run. Opening them reads the numbers directly,
without a text file to parse, and with their full precision. No `.frd` or `.dat` file is
written.

When an analysis with kept results is opened, Nodalis writes them to `Kept_Results` in the
folder of the analysis and opens the results in the Results workspace. The project schematic
and the app bar of the analysis show whether they are still up to date. Measure blocks,
the optimizer and the Campbell diagram take the results straight from memory as well. Where a CalculiX file is still needed, for example the global results of a
submodel or the temperatures a step takes from a heat transfer analysis, Nodalis writes the
`.frd` file from the kept results when it needs it.

Results of the Nodalis solver kept by an earlier version of Nodalis in another format are not
read: solve the analysis again.

#menu("File", "Export", "CalculiX Results (.frd) ...") writes the current results as CalculiX's
files, the `.frd` file and, if the run has history output, the `.dat` file next to it, for
CalculiX GraphiX, PrePoMax or other programs that read them.

#screenshot("export-calculix-results.png", [File menu of an analysis window with open results:
  Export CalculiX Results writes them as `.frd` and `.dat`])

What you defined on the kept results is saved with them and comes back as you left it: derived
field and history outputs, transformations, hot spots, the reference points, coordinate systems
and planes of the results, result paths and plane results, and the increment, field and component
shown. The view comes back too: the camera of the Results workspace, #menu("View", "Mesh Edges"),
the deformation and its factor, #ui("Show Undeformed"), the color levels, hidden parts, the
annotations of the query tool and the plots of result paths. Nodalis takes all of this from the
open result each time the project is saved.

#screenshot("bound-result-restored.png", [Kept results reopened with their symmetry
  transformation and the component shown when the project was saved])

When the analysis runs again, its new results take over all of this from the results before, so the view, transformations, paths and plots stay as they
were. Annotations of the query tool and hot spots on picked nodes or faces refer to nodes and
elements of the mesh; they are kept only if the mesh did not change. After remeshing they are
dropped, reference points placed on picked nodes or faces stay at their last position, and the
output pane lists what was dropped.

#screenshot("rerun-keeps-view.png", [The results of a new run, shown from the front without mesh
  edges like the results before])

#menu("Results", "Remove") drops the kept results after asking, since Undo cannot bring them
back; save the project to remove them from the file.

To send only the meshes and the model definitions, use
#menu("File", "Save Project Without Results ..."). It saves a copy without the kept results under
a new name (proposed as `<project name>-without-results.ndl`); the open project stays on its own
file and keeps its results.

== Model check <model-check>

Nodalis checks the model continuously while you work. Items with a problem get a warning sign in
the tree; items with an error and their containers are shown in red. Click the warning sign to see
what is wrong and how to fix it. The #ui("Problems") tab below the 3D view lists all of them
(@problems).

#screenshot("model-check.png", [Model check: parts without a section])

Errors include elements without material, materials without the data the step needs (elasticity,
density, conductivity, specific heat), invalid material constants, distorted elements, a missing
initial temperature in a transient heat transfer step, an increment larger than the step, rigid
body motion of a part in a static step, truss structures that are mechanisms, radiation without
the physical constants (@model-properties) and missing references (a deleted material, part or set that an item still uses). Warnings include parts held
only by contact (contact pairs without tied surface behavior; tied contact holds like a tie)
when #ui("Weak springs") is turned off in the connection settings (@weak-springs), conflicting boundary conditions, loads on fixed nodes, rotations without effect,
beam nodes with only some of their rotations given (the solver holds them all, as CalculiX
does, and nonlinear steps mostly fail to converge), beams held by pins only, which are free to turn about the line
through the pins (the solver solves them all the same), rigid bodies on nodes that a boundary
condition holds (@overlapping-constraints), steps without load and amplitudes that
nothing uses.

#screenshot("beam-rotations-partly-held.png", [A beam end that holds only the rotation about Z])

The solver checks the whole model again when a run starts (and with #ui("Check Model")) and
reports what it finds on the items of the model: elements it cannot compute, missing material
data, rigid body motion, a step type or feature it does not compute yet. When the solver
aborts, Nodalis explains the messages it knows in the same way, for example radiation without
physical constants as "Physical constants missing"; other messages appear as "Solver error"
with the text of the monitor.

The checks do not stop a run; their errors are listed at the top of the monitor. Missing
references do stop it, because the input file cannot be written.

=== Check by the Nodalis solver <solver-model-check>

With the Nodalis solver (#ui("Solver") under #menu("Edit", "Settings ...") > #ui("Solver"),
@settings), #ui("Check Model") in the context menu of the analysis lets the solver check the
model itself instead of estimating from the geometry. It reads the whole model with its steps
and factorizes the stiffness matrix once for every set of supports (a check takes about as long
as a linear static run, without results). Every direction in which the model can move without
resistance is found exactly, and the nodes that move with it are named:

- *Rigid body motion possible* on the boundary conditions of the step and on each part that
  moves, with the free motions, for example "displacement in Y, displacement in Z, rotation
  about X free". Parts that move apart from each other are reported separately.
- *Truss structure is a mechanism* when the free motion deforms the structure instead of moving
  it as a whole.
- *Held only by contact* instead, when the model has contact: the check does not see what
  contact will hold once it closes.
- *Nodes of a bonded contact not tied* with the parts of the slave nodes that are farther from the
  master side than the tolerance.

Free motions in a frequency step are no problem (they are the modes with frequency zero) and
are not reported. The monitor lists the findings with the first node numbers; the warning
symbols in the tree mark the boundary conditions and the parts.

When a run with the Nodalis solver stops at a singular matrix, the solver runs the same check
by itself and explains the cause in the monitor and the tree.

#screenshot("solver-model-check.png", [Check Model by the Nodalis solver: the beam, held in X
  only at its end, can still slide and turn])

=== Overlapping constraints <overlapping-constraints>

Several items of a model are written as constraint equations (MPCs) that make some degrees of
freedom follow others: regions of no separation, rigid bodies, displacements on a point, bolt
pre-tension cuts, node ties between beams and springs to a surface. The solver, like CalculiX, aborts when a
boundary condition holds such a dependent degree of freedom ("dependent side of a MPC and a
SPC") or when two equations depend on it. Nodalis keeps them apart when it writes the input
file, so the items can overlap freely, for example a fixed support on a face that also lies in a
region of no separation.

The degrees of freedom are handed out in this order: boundary conditions and cyclic symmetry
first, then rigid bodies and displacements on a point, pre-tension cuts, node ties, springs and
last the regions of no separation. An equation whose dependent degree of freedom is taken
already depends on another of its terms instead: a free direction of the same node, or else the
node on the other side, which then follows the held node, as Abaqus moves the constraint to
the independent side. An equation whose terms are all held binds nothing new and is left out.
A rigid body leaves out the nodes that are held already; the model check warns when a boundary
condition holds nodes of a rigid body. Ties need nothing of this: the solver skips the held
directions of their slave nodes by itself.

The input file lists the changes as comments at the top of the constraints, and the monitor
shows them when the run starts.

#screenshot("overlapping-constraints-monitor.png", [A support holds the edge of a region of no
  separation in Z: its three equations there depend on the nodes of the master side])

A slave node of a bonded region (a tie) that is further from the master side than the
tolerance stays untied: the solver only warns about it. After every run the analysis gets the
warning "Nodes of a bonded contact not tied" with the number of nodes and the largest distance;
raise the #ui("Tolerance") of the region above it (@contact-regions) or close the gap.

== Exporting the input file <export-inp>

#menu("File", "Export", "CalculiX Input File ...") writes the same input file as a run, for example
to run it with CalculiX or with the command line solver `plxs` on another computer. Deactivated items appear as comments.

== Submodels <submodel>

A submodel computes a detail of a larger model more precisely. Its cut boundaries follow the
displacements of the global model. Global model and submodel are two analyses of the project,
linked in the schematic (@results-links):

+ Set up the global model in one analysis and the submodel, with its own finer mesh, in
  another.
+ In the submodel, create a boundary condition of type #ui("Submodel") on the cut faces in a
  static step, and choose the displacement components.
+ Set #ui("Model Type") to #ui("Submodel") in the submodel's #ui("Model Properties"). Its block
  in the project schematic gets a #ui("Global results") row below #ui("State"), with an orange
  input pin and a question mark while nothing is linked.
+ Drag the results of the global step (the row of the step below #ui("Results") of the global
  analysis) to #ui("Global results"). The output pane confirms "takes the displacements of
  Step-1 of Global in Step-1 (submodel)", and the row names the global analysis. The step with
  submodel boundary conditions takes them, or the first step; the link also sets the global
  step of the boundary condition. Each step can take the results of another global step at its
  own #ui("Global results") input under #ui("Setup") > #ui("Steps"). Dropping results there
  makes an analysis a submodel too. Global results computed elsewhere come from a file block
  (@file-block) that reads their `.frd` file; its steps link the same way.

  #screenshot("submodel-global-results.png", [The results of Step-1 of the global analysis
    linked to the Global results row of the submodel])
+ #ui("Solve") the submodel or #ui("Update Project"): the global model is solved first if it has
  no results yet. Nodalis copies the global results next to the input file and writes
  `*SUBMODEL` and `*BOUNDARY, SUBMODEL`.

In the Results workspace of the submodel, #ui("Show Global Contours") in the result controls,
next to #ui("Show Undeformed"), draws the edges of the global model around the results, read from
the global results the submodel took: at the global increment the submodel follows and deformed by
the same scale. They show where the submodel sits in the global model and how the two deform; the
global model is neither a part in the tree nor coloured, and the legend leaves its values out. The
switch shows only for a submodel whose global results file is there.

#screenshot("submodel-global-contours.png", [Results of a submodel at the clamped end of a beam,
  with the edges of the global model around it])

Removing the link leaves the analysis a submodel whose #ui("Global results") row waits for a new
one. #ui("Model Properties") makes it a general model again.

#limitation[Only displacements can be transferred. The global result file must not be the result
file of the submodel's own analysis.]

== Exporting a deformed mesh <export-deformed>

#menu("File", "Export", "Deformed Mesh (.inp) ...") writes the mesh of the displayed result with its
nodes moved by the displacements times the current deformation scale factor. A typical use is an
imperfection for a buckling analysis: show the buckling mode, set #ui("Deformation") to #ui("User
defined") with the imperfection amplitude as factor, export, and import the file into the model with
#ui("Import Geometry") (@mesh-import). The file contains only the mesh.
