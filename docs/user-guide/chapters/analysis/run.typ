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
  [Solve Again], [Runs the solver even if the results are up to date, and computes every step
    again (@step-cache).],
  [Check Model], [Lets the solver only read and check the model and its steps, without
    solving it.],
  [Stop], [Stops meshing for a solve, or the running solver. If the solver has finished
    increments, Nodalis first asks whether their results are kept (@stop-keep-results).
    Otherwise it does not wait for the solver: it stops at once, also in the middle of a long
    factorization or eigenvalue extraction, and what the stopped run computed is discarded.],
  [Monitor], [Shows the #ui("Analysis") tab of the output pane (@monitor).],
  [Open Results], [On the #ui("File") tab: opens the results the analysis keeps.],
  [Remove], [On the #ui("File") tab: drops the results the analysis keeps, after asking
    (@bind-result).],
)

#ui("Solve"), #ui("Stop") and #ui("Monitor") are on the right edge of the ribbon on every tab; the state of the
solution is in the app bar. Which cells are out of date is decided
as in the project schematic (@project-window): changing a load makes only the solution out of
date, changing the meshing settings of a part makes its mesh out of date, and result paths or
planes change nothing that the solver needs.

The built-in solver takes the analysis directly and writes no input file; for CalculiX the
input file is named after the analysis, for example `Cantilever.inp`. To look at the model as
an input file, export it (@export-inp). Old result files of the
analysis in its folder are deleted before the solver starts. Nodalis refuses to run a model
without mesh, without step or with all steps deactivated, and it stops if the analysis cannot
be made (for example because a region is empty).

=== Keeping the results of a stopped run <stop-keep-results>

A nonlinear step with many increments may already show what you wanted to know long before it
ends, or you may see that it goes wrong and want to look at where. When you stop a run that has
finished increments (#ui("Stop") on the ribbon, #ui("Kill") in the monitor, #ui("Stop") or
#ui("Stop All") in the solution monitor, or #ui("Stop") of #ui("Update Project")), Nodalis asks what happens to them:

#fields(
  [Stop and Keep Results], [Stops the solver and keeps the results of the increments done, as
    the results of a stopped run. The step stopped halfway is marked as not finished, as after a
    step that did not converge. If that step asked for fewer result frames than it finished
    increments, the last increment done is written as an additional frame, so that the results
    end where the run stopped.],
  [Stop and Discard], [Stops the solver at once and keeps nothing, as before.],
  [Cancel], [Lets the solver go on (#key("Escape")). #key("Enter") presses no button, so that no
    run is stopped by habit.],
)

#screenshot("stop-keep-results.png", [The question when a nonlinear static step is stopped after
  some increments])

With #ui("Stop and Keep Results") the solver still finishes what it computes at that moment (an
iteration, a factorization) and then writes the results; the monitor shows
#ui("stopping, writing the results so far") meanwhile, and the status at the end is
#ui("stopped, results so far kept"). Pressing #ui("Stop") again while it writes stops at once and
discards them. Analyses of #ui("Update Project") that take data from the stopped one are not
solved. A run without finished increments, such as a linear static step, stops at once without
asking. Only nonlinear static steps write the last increment done as an additional frame; other
steps keep the frames they wrote until the stop.

=== Steps that did not change <step-cache>

A new run does not compute again the steps that did not change since the last run. After every
step the solver keeps the state it reached and the results of the step in the folder
`<analysis name>.steps` next to the input file. When the analysis is solved again, the solver
compares each step with what it kept: the first steps that are still the same are taken over,
and the run continues from the state after the last of them. Add a frequency step after a
static step that is already solved, for example, and only the frequency step is computed. The
monitor tells it in a line like "steps 1 to 1 did not change since the last run".

A step counts as unchanged only when everything its results depend on is the same: the mesh,
the materials, sections, constraints and contact, the step itself and every step before it,
the files it reads (global results of a submodel, temperatures, stored eigenmodes) and the
version of Nodalis. So changing the second step computes the second step and those after it,
while changing a material computes every step. Steps that a stopped or failed run finished are
kept too, so the next run continues where it stopped. The results are the same as those of a
run of all steps from the start.

The folder holds the steps of the last run only and takes about as much space as the results
of the run once more. #ui("Solve Again") does not take steps over: it computes every step.

#screenshot("monitor-steps-taken-over.png", [The monitor after a frequency step was added to a
  solved static step: step 1 is taken over, only step 2 is computed])

=== Standby during a run

While a run is active, Nodalis asks the operating system not to go to standby on its own, so a long
analysis is not interrupted when nobody touches the computer. On Windows this is a request
for the system to stay awake, on Linux an idle inhibitor lock of systemd-logind, the same as
`systemd-inhibit` takes. The request is released as soon as the run has finished, failed or
been stopped; with several runs at once it is released after the last one. The screen is not
kept on: it can still switch off and lock. Closing the lid or choosing Suspend by hand still
puts the computer to sleep. If the system does not offer the request (no D-Bus session on
Linux, for instance), the run starts as usual and the computer may sleep.

=== Monitor <monitor>

The monitor is the #ui("Analysis") tab of the output pane below the 3D view (@output-pane); it
opens when a run starts, so the model and its results stay in view while the solver runs.
Its top row has #ui("Kill") to stop the solver, #ui("Results") to open the results once the run is
finished, the #ui("Filter") field and the status with the running time and the current step,
increment and iteration; below it is the output of the solver. The #ui("Output") tab goes back to
the messages, and the pane can be made taller by dragging its upper edge.

#screenshot("monitor.png", [The Analysis tab after a completed run])

While the output pane is hidden, or shows another tab, the status bar tells that a run is going
on: next to the tabs a spinner, the analysis, how long it has been running and the step and
increment the solver is at, such as #ui("Beam 1: running 5 s | Step 1, increment 6"). Hovering
it shows the whole line of the monitor, a click opens the #ui("Analysis") tab. A Check Model run
shows there as well.

#figure(
  image("../../images/solve-status-bar.png", width: 70%),
  caption: [A running solve in the status bar while the output pane is hidden],
)

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
#ui("failed"), #ui("killed") or #ui("stopped, results so far kept") (@stop-keep-results). If the solver failed, it reports the causes it found
and Nodalis adds a line "Possible cause: ..." with a remedy, for example for distorted
elements, missing material data, rigid body motion or missing convergence.

=== Solver profile <solver-profile>

The solver measures on every run where its time goes, at no noticeable cost. Once a run is
finished, the monitor shows the profile left of the output: the solver time, the size of the
model as solved (nodes, elements and degrees of freedom, with shells and beams expanded) and a
table of the phases, the longest first, with their time, their share of the solver time and how
often they ran.

#fields(
  [Assembly], [Element matrices and internal forces into the global system.],
  [Constraint reduction], [Constraints (ties, couplings, rigid bodies, equations) eliminated
    from the system.],
  [Symbolic analysis], [Ordering and structure of the direct solver; reused while the
    pattern of the matrix stays the same.],
  [Factorization], [The numeric factorization of the direct solver, or the setup of the
    preconditioner of the iterative solver.],
  [Solve], [Forward and back substitution, or the iterations of the iterative solver.],
  [Vector arithmetic], [Predictors, right-hand sides, reactions and energies of the time
    increments.],
  [Results], [Stresses, strains and the result fields of the increments.],
  [Loads and constraints], [Loads and prescribed displacements of each increment.],
  [Contact], [Contact search and evaluation of the contact springs.],
  [Writing results], [Handing the results to Nodalis at the end of the run.],
  [Other], [The time no phase measured: preparing the model and the steps, and work of a
    procedure that has no phase of its own.],
)

#ui("Profile") in the top row hides and shows the table in every monitor. #ui("Copy Report")
copies a plain text report to the clipboard: the versions of Nodalis and the solver, the
operating system, the number of cores and threads, the graphics card and whether the solver
uses it, the analysis with its status and time, the size of the model, the steps with their
kind and the profile. For a model that solves suspiciously slowly, paste it into an issue on
GitHub; it is already formatted as a code block.

#screenshot("solver-profile.png", [The profile of a linear static run in the solution monitor; the
  iterative solve takes the largest share])

The profile covers the solver itself. Meshing and preparing the model before the run are not
in it; the running time in the top row includes them. A stopped run and a run with a step that
did not converge show the profile up to where they ended.

=== Convergence plot <convergence-plot>

While the solver runs, the monitor plots how it converges, right of the output, so a hard
nonlinear run shows where it struggles long before it ends; like the solution information of
Ansys Mechanical or the job monitor of Abaqus. The plot fills as the solver reports, from the
first iteration on, and stays after the run until the next one. #ui("Convergence") in the top row
of every monitor hides and shows it; the choice is kept with the window layout.

Above the plot are the views to pick from and a line on where the solver is: the step with its
name, the increment, the attempt once an increment was cut back, the iteration and the size of
the increment. For a step that goes through time, a bar shows how far the step got through its
time period.

#fields(
  [Residuals], [The largest residual force and the largest correction of the displacements of
    every Newton iteration, each divided by the criterion it has to reach, over the iterations
    of the whole run. An increment converges when both lines are below 1, the dashed
    #ui("Criterion") line; the residual criterion follows from the average force of the step,
    the correction criterion from the change of the displacements in the increment. Thin green
    lines mark the converged increments, red dashed lines the cutbacks, grey lines the start of
    a step. Hovering over a point tells the step, increment, attempt and iteration with the
    values themselves and their criteria.],
  [Time], [The total time each converged increment reached, over the increments. Red crosses
    are attempts that did not converge and were cut back.],
  [Increment Size], [The size of each increment, on a logarithmic axis, with the cut back
    attempts as red crosses: the automatic incrementation shrinking and growing the increments
    shows here.],
  [Linear Solver], [For the iterative equation solver: the relative residual of each of its
    iterations in the solve going on, live, with the one before dashed and its tolerance;
    above it the number of solves and their iterations. The direct solver needs no iterations,
    so the view stays empty.],
)

Linear steps check no convergence and add no points to #ui("Residuals"); their increments still
show under #ui("Time"). Drag to pan, drag along an axis to zoom and double-click to fit the plot
to its values again. Click an entry of the legend to hide or show its line. A long run keeps the
last 200000 iterations and increments.

#screenshot("convergence-plot.png", [The residuals of an elastic-plastic cantilever loaded beyond
  its limit load (NLGEOM): once it yields, the increments need more iterations and three are cut
  back])

== Analyses that continue others <restart>

One analysis can continue another one after a step, through a state link in the project
schematic (@state-links): it starts from the displacements, stresses and contact state the other
analysis reached. When it runs, the solver computes the steps of the other analysis up to the
linked one and then its own steps. Like any run it takes over the steps that did not change
since its last run (@step-cache), so changing one of its own steps does not compute the steps of
the other analysis again.

== Results kept with the project <bind-result>

When a run is finished, its analysis keeps the results: the input file and the results of the
run are saved in the project file with the next #menu("File", "Save Project"), together with a
fingerprint of what the run was made from. The results open in the Results workspace. The
output pane shows how much they add to the project file; they are stored compressed. Keeping
the results takes a moment and runs in the background. A new run replaces the results the
analysis keeps.

The solver keeps its results as one binary results file (`.ndlr`, written next to the input
file in the work directory) that holds the mesh the results are on and every field and history
output of the run. Opening them reads the numbers directly,
without a text file to parse, and with their full precision. No `.frd` or `.dat` file is
written.

The results file is compressed the way formats like HDF5 store large arrays: the node ids,
coordinates and every component of every field are kept as arrays of their own, cut into chunks
of 1 MB, and each chunk is shuffled byte by byte (the sign and exponent bytes of all numbers
together) and compressed with Zstandard. All chunks are compressed and read back in parallel.
The compression is lossless: every number reads back exactly as the solver computed it.
For a static analysis with 200,000 nodes and stresses, strains, displacements and forces, the
file is about 30 % smaller than without compression and opens just as fast. The project file
keeps this file as it is, without compressing it again, so keeping large results in the project
takes a fraction of a second.

A project file is the text of the project followed by a binary part with the files it keeps:
kept results and files embedded by file blocks. Opening a project reads the text only; the
kept files stay in the project file and are read from there when they are wanted, so neither
an open project nor its kept results take memory for them. Results a run has just finished are
copied aside until the project is saved. A project with 2.8 GB of kept results opens in a
seventh of a second with 20 MB of memory and is saved again in less than 2 seconds.

Results need not fit into memory. Opening them reads the mesh and the list of increments and
fields only; the values of a field are read from the file when it is shown or used, a field
at a time and with all its components. Fields read recently stay in memory for going back and
forth between increments, up to the #ui("Memory for results read") of the settings (by
default a quarter of the free memory, at most 2 GB); beyond that the ones used longest ago are
dropped and read again when they are wanted, so that clicking through every field of every
increment never holds more. The values are read in the background: while they are, the
status bar shows #ui("Reading results …") and the view keeps showing what it showed before. An
increment animation first reads the displacements of all its frames, for the scale of the
deformation.

While an increment is shown, the same field and the displacements of the increments shown next
are read ahead in the background: during an animation the frames it plays next, in the
direction it plays; otherwise the increments after the shown one and a few before it. They take
at most a quarter of the memory for results, and no more than 64 increments. Stepping or
scrolling through time thus finds them at hand. A frame that is not read yet when its turn
comes holds the animation until it is, so that no frame is skipped for reading. Animating
von Mises stresses through the 400 frames below, the view waited for the file 556 times in
30 seconds before and once now, and shows about twice as many frames per second.

Drawing an increment also happens in the background: the surfaces of the parts are built on
another thread while the status bar shows #ui("Drawing results …"), and the window keeps
answering meanwhile. For a mixer with 185,000 particles and 1.5 million nodes, changing the
increment held the window for 360 ms before and for 125 ms now; the other 350 ms run beside it.

Results of a dynamic step with 400 frames and 44,000 nodes (a 2.8 GB results file)
open in a tenth of a second with less than 30 MB of memory, where reading them as a whole took
12 seconds and 6.8 GB. Animating their 400 frames through six fields one after the other
holds 2.7 GB with the default memory for results and 0.8 GB with 512 MB.

Results need not fit into memory while the analysis runs either. Each increment the solver
finishes is written to a temporary file next to the results file (`.ndlr.spool`) and leaves
memory; only the increment it works on stays there. When the run has ended, the results file
is put together from the temporary file, without decompressing the increments again, and the
temporary file is removed. The increments of a step that did not converge are kept as well;
those of a stopped run are discarded with the temporary file. The beam with 44,000 nodes
solved for 400 frames needs 1.6 GB of memory from the first increment to the last, where the
memory grew with every increment before (3.3 GB for only 100 frames), and the run is faster
(125 instead of 160 seconds).

#figure(
  image("../../images/results-reading.png", width: 70%),
  caption: [The status bar while the displacements of an animation are read],
)

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
shown. The view comes back too: the camera, #menu("View", "Mesh Edges"),
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

#menu("File", "Remove") drops the kept results after asking, since Undo cannot bring them
back; save the project to remove them from the file.

To send only the meshes and the model definitions, use
#menu("File", "Save Project Without Results ..."). It saves a copy without the kept results under
a new name (proposed as `<project name>-without-results.ndl`); the open project stays on its own
file and keeps its results.

== Model check <model-check>

Nodalis checks the model continuously while you work. Items with a problem get a warning sign in
the tree; items with an error and their containers are shown in red. Click the warning sign to see
what is wrong and how to fix it. The #ui("Problems") tab of the output pane lists all of them
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
data, rigid body motion, a step type or feature it does not compute yet.

=== When the solver stops <solver-failures>

When a run with the Nodalis solver stops, the solver tells Nodalis what failed, not only in
words, and Nodalis explains it like the other problems: in the monitor as "Cause: ...", in the
#ui("Problems") tab, at the analysis and at the step the run stopped in. Each diagnosis names
the cause and the remedy:

- *No convergence*: the increments of a nonlinear step did not converge. The detail says why
  the last attempt was given up (the residual force grew, the iterations ran out or converged
  too slowly, the solution grew without bound) and why no smaller increment was tried. When
  the model has contact whose status had already settled, it says so: the load, the material
  or the supports are then the more likely cause.
- *Contact does not converge*: the same, while the contact status was still changing in the
  last iterations (points opening and closing again), with the remedies for contact.
- *Rigid body motion possible* instead of either, when one part kept moving by at least
  5 % of its own size in each of the last three iterations: it is held only by contact, or
  not at all, along that motion, as a screw whose head was given no contact with its seat.
  The message names a node and an element of the part and the direction it moved in, and
  the part itself is marked in the #ui("Problems") tab and the model tree. Give it a support
  or the contact it rests on.
- *Temperatures do not converge* in a heat transfer step.
- *More increments needed than allowed*: the step needs more increments than the
  #ui("Increment limit") set under #ui("Advanced settings") of the step (off by default).
- *Rigid body motion possible* for a singular stiffness matrix, with the model check of the
  solver that names the parts that move (@solver-model-check).
- *Stiffness not positive definite*, *Not enough memory*, *Iterative equation solver failed*
  (each with the equation solver to choose instead), *Eigenvalues not found* and *Undamped
  resonance*.
- *Value rejected by the solver*, *Not supported by the solver* and *Solver files not written*
  for the checks of the solver before it computes, at the item it names.

Only a failure inside the solver itself appears as "Internal solver error"; it is a bug to
report, not a problem of the model. Runs of input files with another solver are read from their
output text instead, for example radiation without physical constants as "Physical constants
missing".

#screenshot("solver-failure-diagnosis.png", [A step that needs more increments than allowed:
  the diagnosis at the step, the cause in the monitor])

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

#menu("File", "Export", "CalculiX Input File ...") writes the analysis as an input file, for example
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

The dialog of a #ui("Submodel") boundary condition shows where its displacements come from.
#ui("Global results") names the step and the analysis linked to the #ui("Global results") input
of its step. #ui("Displacements") says whether that global step writes displacements: every
step that deforms the model under loads does. The displacements at the end of the global step
are read. The dialog shows in red what keeps the submodel from running:

- nothing is linked to the step of the boundary condition,
- the global step computes mode shapes, temperatures or sensitivities, not the displacements of
  a load,
- steps of the submodel take results of different global analyses; a submodel reads one global
  results file.

For the results of a file block the dialog cannot tell what they hold; a file
without displacements of the step stops the run with a message.

#screenshot("submodel-bc-source.png", [The boundary condition reads the displacements of Step-1
  of the analysis Global])

In the Results workspace of the submodel, #ui("Global Contours") on the #ui("Display") tab,
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
nodes moved by the displacements times the current deformation scale factor. The file contains
only the mesh, for other programs or for another project. To start an analysis of the same project
from a buckling mode as imperfection, use a Deformed Mesh block in the project schematic instead
(@deformed-mesh-block): it keeps the sets of the mesh and is made again when the results change.
