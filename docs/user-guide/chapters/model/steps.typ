#import "../../template.typ": *

== Steps <steps>

A step is one analysis of CalculiX with its own boundary conditions, loads and outputs. Create it
with #ui("Step") on the #ui("Steps") tab of the ribbon. A new step copies the
boundary conditions and loads of the last step, as far as its type accepts them. The type of a
step cannot be changed after it was created.

#note[Every step writes `OP=NEW` for its boundary conditions and loads. What applies in a step is
exactly what is listed under it in the tree.]

#screenshot("step.png", [A new static step with its defaults; the note below appears in a model
  with hyperelastic materials])

In a model with contact regions the dialog also lists #ui("Contacts in this step"): regions
switched off or on and friction coefficients changed from this step on, see
@contact-steps.

=== Step types in the tree

Every step shows a pictogram of its type in front of its name, so that you can tell the steps
apart whatever you named them. Hover over the pictogram or the name to read the type.

#screenshot("step-types.png", [Static, frequency, complex frequency, buckle, heat transfer,
  coupled, dynamic, modal superposition, steady state dynamics and sensitivity steps])

#screenshot("step-type-tooltip.png", [Tooltip of a step])

A deactivated step shows the red no-entry sign instead; its tooltip still names the type.

=== Order of the steps

The solver runs the steps in the order of the list, and a step starts from the state at the end of
the previous one. To change the order, right-click a step in the #ui("Steps") list and choose
#ui("Move Up") or #ui("Move Down"). The step keeps its boundary conditions, loads, defined
fields and outputs, and stays selected. #ui("Move Up") is disabled on the first step and
#ui("Move Down") on the last.

#screenshot("step-move-menu.png", [Context menu of a step])

Steps that build on the previous one, such as a frequency step with the base state
#ui("End of ..."), take the state of whatever step now precedes them, so check the order after
moving: the #ui("Start") section of each step names the step it builds on. The step numbers of
existing results refer to the order at the time of the run.

=== What a step takes from the steps before <step-handover>

The steps of an analysis form a chain. Two kinds of steps make it up:

- Static, visco, dynamic and heat transfer steps *continue* the chain: each starts from the state
  at the end of the active step before it (displacements, stresses, plastic and creep strains,
  contact status, velocities, temperatures) and passes its own end state on. The first step
  starts from the unloaded model with the initial conditions (@initial-conditions).
- Frequency, buckle, complex frequency, steady state dynamics and sensitivity steps *probe* the
  model: they are computed about a base state and leave the state as they found it. The step
  after them starts where the step before them ended.

Every step dialog begins with the section #ui("Start"), which says this for the step with the
names of the steps involved. The model tree repeats its first line as the first, italic row
under each step; double-click it to open the step.

#screenshot("step-start.png", [The Start section of a frequency step about the state at the end
  of the static step before it])

#screenshot("step-start-static.png", [A static step after it continues the static step before the
  frequency step; the load it no longer has ramps down to zero])

#screenshot("step-start-tree.png", [The first row under each step in the model tree])

The project window shows the same chain as a trace at the left of the step rows of the analysis
block (@project-window), and the results timeline marks the steps that leave the state as they
found it (@results-timeline).

#fields(
  [Base state], [Frequency, buckle, complex frequency and steady state dynamics steps, and the
    eigenmodes of a Dynamic step with modal superposition: what they are computed about. #ui("Unloaded model") (default) uses the
    linear stiffness of the model without stresses. #ui("End of <step>") uses the state the
    named step leaves, with the stiffening by its stresses and its contact status
    (`*STEP, PERTURBATION`): a tensioned string, a bolted joint or a spinning rotor. Only the
    last continuing step before can be the base; to build on an earlier one, move the step.],
  [Starts from], [The state the step starts from: #ui("Unloaded model"), #ui("End of <step>")
    with the method of a dynamic step, and initial conditions where they act (temperatures in
    the first step, velocities in the first dynamic step). The line below lists what carries
    over. A deactivated step in between is named and left out.],
  [Uses], [What the step reads or computes for itself: the eigenmodes of a complex frequency or
    steady state dynamics step (or those an Eigenmodes link gives), the solution of the step
    before a sensitivity step.],
  [Loads], [In a static or visco step after one: every load moves linearly from its value at
    the end of that step to the value of this step. A load the step no longer has ramps down to
    zero; they are listed.],
  [Temperatures], [Temperatures a step before was given and this one is not go back to the
    initial temperatures over the step (@defined-fields-steps).],
  [Model-wide], [What holds in every step: constraints, contact, bolts, and the coupling of a
    rigid remote force of another step, which keeps its face rigid here too.],
  [Passes on], [What the next active step gets: the end state, or nothing for a probing step.],
)

==== Step overview <step-overview>

#menu("Steps", "Step Overview") (also in the context menu of #ui("Steps") in the tree) shows
every boundary condition, load and defined field, and every model-wide item, against the
steps. The second row says where each step starts. Each cell compares the item with the step
the step starts from, for a probing step its base:

#fields(
  [Created], [The step has the item and the step it starts from does not.],
  [Same, Changed], [Both have it, with the same or with other values, region or amplitude.],
  [Off], [The step has the item, deactivated.],
  [Removed], [The step it starts from has the item and this one does not.],
  [Ramps to 0], [A load a static or visco step no longer has: it moves down to zero over the
    step.],
  [Model-wide], [Constraints, ties, contact and bolts act in every step. A contact switched off
    from a step on reads #ui("Off from here"), a bolt names its action in the step.],
)

An empty cell is an item the step does not have and did not have before, or one a step of that
type cannot take (loads in a frequency step). Click a cell to open the item in that step, or a
step name to open the step. The context menu of a cell copies the item to all following steps
that can take it, in place of the one there (#ui("Copy to Following Steps")), or removes it
from them (#ui("Remove from Following Steps")). Undo takes either back.

#screenshot("step-overview.png", [Step overview: the pressure of Preload ramps down to zero in
  Unload, the frequency step Modes takes no loads, and the contact region Flange is switched
  off from Unload on])

=== Equation solver <equation-solver>

Static, visco, dynamic, heat transfer, frequency and buckle steps have an
#ui("Equation solver") field that chooses how the systems of equations of the step are
solved:

#fields(
  [Automatic], [(default) The solver chooses. A static or heat transfer step with
    50 000 equations and more whose unknowns are translations (or temperatures) only, i.e.
    a solid model, solves iteratively: the multigrid converges in a few dozen iterations
    there and beats the factorization by far. Models with shells, membranes, beams or
    trusses (the solver expands them into solids, but the multigrid does not fit the bending
    of such thin elements and hardly converges on them), the
    eigenvalue iteration of frequency and buckle steps and implicit dynamic steps (one
    factorization serves many systems) and smaller models solve directly while the factor
    fits: the solver first estimates its memory, and if that exceeds 60 % of the memory the
    computer has free, the step solves iteratively. If the chosen solver fails, the other
    one takes over once: a factorization that does not fit into the memory continues
    iteratively, an iteration that does not converge within 2000 iterations, or that stalls
    on the way (after 200 iterations, and every 100 after that, the residual has to be at
    most half of what it was 200 iterations before), continues directly. Leave it here
    unless you have a reason not to.],
  [Direct], [A sparse direct factorization (LDLᵀ). It solves every system, including
    tangents near buckling and parts held only by contact, but needs memory for the factor,
    which grows quickly with the model: about 2.4 GB for a solid block of 170 000 equations.],
  [Iterative], [Conjugate gradients with an algebraic multigrid preconditioner (smoothed
    aggregation on the rigid body modes of the nodes). It needs a fraction of the memory
    (about 20 times less on that block) and is usually faster than the direct solver on large
    solid models, also on thin ones and on quadratic tetrahedra: the number of iterations
    hardly grows with the model. It solves linear and nonlinear static steps (also with
    plasticity, large deformations, gaps and contact; sliding friction adds a few GMRES
    iterations around it),
    visco, implicit and explicit dynamic and heat transfer steps, and the eigenvalue
    iteration of frequency and buckle steps (see below). It only solves positive
    definite systems: the model must be held against every rigid body motion, and a tangent
    that loses its stiffness (close to buckling) stops it. Then the analysis stops with a
    message that names the direct solver.],
)

#screenshot("equation-solver.png", [Equation solver of a static step])

In frequency and buckle steps the eigenvalue iteration (shift-invert Lanczos) solves one
system per iteration step. With #ui("Iterative") these are solved by conjugate gradients,
which takes longer than with a factorization but fits large models into the memory. A
frequency step needs no #ui("Lower frequency") bound for it; with one it solves directly.
Without a factorization there is no Sturm sequence check that no eigenvalue was missed;
instead the iteration restarts with a fresh start vector until no new eigenvalue turns up
among the wanted ones, which also finds the second copies of double eigenvalues (symmetric
parts). Complex frequency and steady state dynamics steps and Dynamic steps by modal
superposition work on the modes of a frequency step, so the choice there applies to them as well; sensitivity steps always solve
directly and do not offer it. The monitor reports
the automatic choice, e.g. "automatic equation solver: 173394 equations of translations
only; solving iteratively" or, with the estimated memory, "the factor of 173394 equations
needs 2.4 GB, 15.8 GB available; solving directly". The input file writes
nothing for #ui("Automatic"), `SOLVER=PASTIX` for #ui("Direct") and `SOLVER=ITERATIVE
CHOLESKY` for #ui("Iterative").

The monitor shows how a solve went: for the direct solver the size of the factor and the
times of ordering, factorization and solution; for the iterative one the levels of the
multigrid hierarchy with their sizes, its memory relative to the matrix, the number of
iterations and the remaining relative residual (below 1e-12). The results of both solvers
agree to that accuracy. In a nonlinear step the hierarchy is kept over the Newton
iterations while that is cheaper than building it again (the monitor reports the
rebuilds). A node held by a stiff spring in one direction only (a gap or contact spring)
is scaled by its whole stiffness block, so that its other directions still converge.
Bodies that touch only through contact, such as a pin in a bore, are coarsened
one by one, so that each keeps its own rigid body motion on the coarse levels; and the
first solve with a hierarchy learns the shapes that converge slowest (the bending of thin
walls, for example) and adds them to the coarse space of the next one, which cuts the
iterations of the following solves by a fifth to a third on such models.

On a computer with a discrete graphics card, the iterative solver runs on the card for
models of 100 000 equations and more. It detects this by itself, and there is nothing to
set. The multigrid cycle runs in single precision as on the processor; the conjugate
gradients around it, including the product with the matrix, run in double precision on the
card as well when it supports that (otherwise they stay on the processor), so the results
are the same as without the card. The card has many times the
memory bandwidth of the processor, which is what limits the cycle: on a 591 000 equation
model of quadratic tetrahedra the solve drops from 15.9 s to 1.4 s on an RTX 4070 (the
setup of the multigrid, about 3 s, stays on the processor). Integrated graphics share the memory
of the processor and gain nothing, so they are not used. The monitor line of the
preconditioner ends with "conjugate gradients and V-cycle on the graphics card" (or only
"V-cycle on the graphics card", with the reason) and its name. If a card was found
but cannot be used, for example because the model does not fit into its memory, the line
says "V-cycle on the processor" with the reason, and the solve continues on the processor.
The solver takes at most 60 % of the graphics memory the driver grants, so that the window
and the desktop, which draw on the same card, keep enough of it.
For measurements the environment variable `PLXS_GPU=off` keeps the cycle on the processor
and `PLXS_GPU=on` uses any graphics adapter, including software renderers, for models of
any size.

=== Static

A static step needs three settings: how long it lasts, how many results it keeps, and whether
the deformations are large. How the solver divides the step into increments follows from
them, so a step runs to its end without further settings.

#screenshot("static-step.png", [A static step with large deformations and ten frames; the
  advanced settings open])

#fields(
  [Duration], [Length of the step (default 1). Without an amplitude the loads ramp up
    linearly over it. For materials without creep only its ratio to the amplitudes matters,
    so it is a pseudo time.],
  [Frames], [Result frames kept over the duration, evenly spaced, the last one at the end
    (default 1). The solver ends an increment on every frame, so the frames fall on exact
    times. A linear step is solved once per frame with one factorization of the stiffness.],
  [Large deformations (Nlgeom)], [The equilibrium is found in the deformed shape (`NLGEOM`):
    large displacements and rotations, loads that follow the surface, stiffening, snap
    through. Each increment iterates. Switching it on raises 1 frame to 10, to show the
    load history.],
)

Under #ui("Advanced settings") the automatic choices can be overridden; the box opens by
itself when something there differs from the defaults. A ticked box next to a value sets it,
an unticked one shows the automatic value greyed out.

#fields(
  [Increments], [#ui("Automatic") (default): an increment grows by half after two that
    converged easily and is cut back after one that failed. #ui("Fixed"): every increment has
    the given #ui("Size"), shortened only to end on the frames; an increment that does not
    converge fails the step.],
  [Initial], [Size of the first increment. Automatic: one frame interval, with large
    deformations at most a tenth of the duration. It never exceeds #ui("Largest").],
  [Smallest], [The step fails when a cutback would make an increment shorter. Automatic:
    1e-5 of the duration.],
  [Largest], [No increment gets longer. Automatic: the duration, as the frames already end
    increments.],
  [Increment limit], [Stops the step after this many increments. Off by default: the step
    takes as many increments as it needs, and #ui("Smallest") still stops one that gets
    stuck.],
  [Equation solver], [How the systems of equations are solved (@equation-solver).],
)

Other programs differ mainly in names: Abaqus and CalculiX ask for the initial, minimum and
maximum increment and stop after 100 increments unless told otherwise; Ansys Mechanical asks
for substeps and keeps results at evenly spaced points. Nodalis asks for the frames and
derives the rest.

=== Frequency

Computes eigenfrequencies and mode shapes (`*FREQUENCY`). A frequency step takes no loads.

#fields(
  [Base state], [#ui("Unloaded model") (default), or the state at the end of the step before
    with its prestress (@step-handover).],
  [Number of eigenfrequencies], [Default 10.],
  [Lower / Upper frequency bound], [Optional range of frequencies to compute.],
  [Store matrices and eigenmodes], [Writes the `.eig` file (`STORAGE=YES`).],
  [Nodal diameter range], [Only in cyclic symmetric models (@cyclic-symmetry): the nodal
    diameters to compute.],
  [Equation solver], [How the systems of the eigenvalue iteration are solved
    (@equation-solver).],
)

Materials need a density in a frequency step.

=== Buckle

Computes buckling factors and buckling modes (`*BUCKLE`). The loads of the step are the reference
load: the critical load is the buckling factor times these loads.

#fields(
  [Base state], [#ui("Unloaded model") (default), or the state at the end of the step before
    as a preload that the buckling factor does not scale (@step-handover).],
  [Number of buckling factors], [Default 1.],
  [Accuracy], [Convergence tolerance of the eigenvalue solver (default 1e-4).],
  [Equation solver], [How the reference solution and the systems of the eigenvalue
    iteration are solved (@equation-solver).],
)

=== Heat Transfer, Coupled and Uncoupled Temperature-Displacement

#ui("Heat Transfer") computes temperatures only (`*HEAT TRANSFER`). #ui("Coupled
Temperature-Displacement") computes temperatures and displacements together in one system of
equations (`*COUPLED TEMPERATURE-DISPLACEMENT`). #ui("Uncoupled Temperature-Displacement")
(`*UNCOUPLED TEMPERATURE-DISPLACEMENT`) solves the temperatures first and then the displacements
from them in each increment. It is faster and suits thermal stresses as long as the deformation
does not change the heat flow, for example through a contact that opens. The step takes the same
boundary conditions and loads as the coupled step: temperatures, heat fluxes, films and radiation
together with supports and mechanical loads.

#screenshot("uncoupled-step.png", [Creating an uncoupled temperature-displacement step])

#fields(
  [Steady state], [Steady-state solution (default on). Switch it off for a transient
    analysis; materials then need density and specific heat, and the model an initial
    temperature.],
  [Max. temperature change], [Largest temperature change per increment in a transient analysis
    (`DELTMX`).],
  [Nonlinear geometry], [Coupled steps only.],
)

Duration, frames, the advanced settings and the equation solver are the same as for a static
step. In a transient step the duration is real time.

=== Visco

A static step over a creep time in which materials with a creep law creep (`*VISCO`), see
@creep.

=== Field outputs <step-field-outputs>

Field outputs control what the solver writes to the results as `*NODE FILE`,
`*EL FILE` and `*CONTACT FILE`. A new step gets a node and an element field output whose
variables depend on the step type. Frequency, complex frequency and buckle steps take none:
their results are the normalized mode shapes (@mode-shapes), so the tree shows no
#ui("Field Outputs") under such a step and #ui("Field Output") on the #ui("Steps") tab does not
add one to it. Create more with #ui("Field Output") on the #ui("Steps") tab; it
goes into the active step (@active-step). Edit one with a double-click. Hovering over
a variable shows what it is.

#fields(
  [Type], [#ui("Node Output"), #ui("Element Output") or #ui("Contact Output"); chosen when the
    output is created.],
  [Variables], [Node: `RF U PU V NT PNT RFL SEN` (`SEN` in sensitivity steps only). Element: `S PHS E ME PEEQ ENER HFL ERR HER ZZS SDV`;
    `ERR` and `HER` exclude `ZZS`. Contact: `CDIS CSTR CELS PCON`.],
  [Last iterations], [#ui("On") writes every iteration of the last increment, to find out why a
    solution diverges.],
  [Contact elements], [#ui("On") writes the contact elements to a `.cel` file.],
  [Output (2D/3D)], [Element outputs; the node outputs of the step follow them. #ui("Default")
    and #ui("3D") write shells and beams in the 3D form CalculiX expands them to, #ui("2D") at
    their own nodes, where the stresses of shells lack the bending.],
  [Global], [Node and element outputs. #ui("No") writes the results in the local coordinate
    systems.],
)

#screenshot("field-output-dialog.png", [Creating an element field output])

Like boundary conditions, field outputs can be deactivated and deleted from their context menu.
A deactivated field output is left out of the input file.

#screenshot("field-output-tree.png", [A deactivated contact field output])

#note[Field outputs of a step are what the solver computes. The field outputs on the #ui("Results")
tab derive new fields from opened results instead (@derived-fields).]

=== History outputs

History outputs write values of a region to the `.dat` file in every increment (`*NODE PRINT`,
`*EL PRINT`, `*CONTACT PRINT`). Create them with #ui("History Output") on the #ui("Steps") tab, in the
active step. Hovering over a variable shows what it is.

#fields(
  [Type], [#ui("Node Output") (region of nodes), #ui("Element Output") (parts, element set or
    selection), #ui("Contact Output") (a contact pair) or #ui("Energies") (the whole model,
    Dynamic steps only, see @dynamic-step).],
  [Variables], [Node: `RF U V NT RFL`. Element: `S E ME PEEQ HFL ENER ELSE ELKE EVOL EBHE`.
    Contact: `CDIS CSTR CELS CNUM CF`. Energies: `ALLIE ALLKE ALLWK ALLVD`.],
  [Totals], [#ui("Yes") adds the sum over the region, #ui("Only") writes only the sum.],
)

#screenshot("history-output-dialog.png", [Element history output of picked faces])

CalculiX prints a history output at every increment, so a step gives one row per increment:
a linear static step one row, a nonlinear step (Nlgeom, contact, plasticity) as many rows as it
takes increments, a frequency or buckle step one row per mode. A history output also stays in
effect in the following steps until a step defines history outputs of the same type (node, element or contact) of its own.

When results are opened, the `.dat` file is read too, and the history outputs appear on the
Results workspace (@history-outputs).

==== Forces and moments of a point <history-point>

For the forces, moments and rotations of a rigid body, of a displacement on a point or of a
remote force, choose #ui("Reference Point") as the region of a node output. The list offers the
reference points under #ui("Features") (@features), which a rigid body constraint drives
(@rigid-body), the displacements of all steps that are written with a point
(@displacement-behavior) and, under #ui("Remote forces"), the remote forces of all steps
(@remote-force), whose points exist only in the input file. A remote force that acts on a
reference point is listed with that reference point. The point's values appear under the name of the history output: `RF` gives the forces
(`FORCES`) and the moments (`MOMENTS`, `RM1` to `RM3`), `U` the displacements and the rotations
(`ROTATIONS`, `UR1` to `UR3`, in radians), `V` the velocities and the angular velocities.

#screenshot("history-output-point.png", [Node history output of the point of a
  displacement])
#screenshot("history-output-remote-force.png", [The point of a remote force as the region of a
  node history output])

The solver computes reaction forces and moments only at the point of a rigid body: a rigid body
constraint or a displacement on a #ui("Rigid") point. At a #ui("Deformable") point only the
displacements and rotations are printed.
