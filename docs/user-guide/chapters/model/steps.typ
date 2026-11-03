#import "../../template.typ": *

== Steps <steps>

A step is one analysis of CalculiX with its own boundary conditions, loads and outputs. Create it
with #menu("Model", "Create Step ...") or #ui("Create ...") on #ui("Steps"). A new step copies the
boundary conditions and loads of the last step, as far as its type accepts them. The type of a
step cannot be changed after it was created.

#note[Every step writes `OP=NEW` for its boundary conditions and loads. What applies in a step is
exactly what is listed under it in the tree.]

#screenshot("step.png", [Static step])

In a model with contact regions the dialog also lists #ui("Contacts in this step"): regions
switched off or on and friction coefficients changed from this step on, see
@contact-steps.

=== Step types in the tree

Every step shows a pictogram of its type in front of its name, so that you can tell the steps
apart whatever you named them. Hover over the pictogram or the name to read the type.

#screenshot("step-types.png", [Static, frequency, complex frequency, buckle, heat transfer,
  coupled, dynamic, modal dynamics, steady state dynamics and sensitivity steps])

#screenshot("step-type-tooltip.png", [Tooltip of a step])

A deactivated step shows the red no-entry sign instead; its tooltip still names the type.

=== Order of the steps

The solver runs the steps in the order of the list, and a step starts from the state at the end of
the previous one. To change the order, right-click a step in the #ui("Steps") list and choose
#ui("Move Up") or #ui("Move Down"). The step keeps its boundary conditions, loads, defined
fields and outputs, and stays selected. #ui("Move Up") is disabled on the first step and
#ui("Move Down") on the last.

#screenshot("step-move-menu.png", [Context menu of a step])

Steps that build on the previous one, such as a step with #ui("Perturbation"), take the state of
whatever step now precedes them, so check the order after moving. The step numbers of existing
results refer to the order at the time of the run.

=== Equation solver <equation-solver>

Static, visco, dynamic, heat transfer, frequency and buckle steps have an
#ui("Equation solver") field that chooses how the systems of equations of the step are
solved:

#fields(
  [Automatic], [(default) The solver chooses. For a system of 50 000 equations and more it
    first estimates the memory of the direct factorization; if that exceeds 60 % of the
    memory the computer has free, the step solves iteratively, otherwise directly. If the
    chosen solver fails, the other one takes over once: a factorization that does not fit
    into the memory continues iteratively, an iteration that does not converge continues
    directly. Leave it here unless you have a reason not to.],
  [Direct], [A sparse direct factorization (LDLᵀ). It solves every system, including
    tangents near buckling and parts held only by contact, but needs memory for the factor,
    which grows quickly with the model: about 2.4 GB for a solid block of 170 000 equations.],
  [Iterative], [Conjugate gradients with an incomplete Cholesky preconditioner. It needs a
    fraction of the memory (about 35 times less on that block) and is often faster on large
    solid models. It solves linear and nonlinear static steps (also with plasticity, large
    deformations, gaps and contact; sliding friction adds a few GMRES iterations around it),
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
parts). Complex frequency, modal dynamics and steady state dynamics steps work on the modes of
a frequency step, so the choice there applies to them as well; sensitivity steps always solve
directly and do not offer it. The monitor reports
the automatic choice with the estimated memory, e.g. "automatic equation solver: the factor
of 173394 equations needs 2.4 GB, 15.8 GB available; solving directly". The input file writes
nothing for #ui("Automatic"), `SOLVER=PASTIX` for #ui("Direct") and `SOLVER=ITERATIVE
CHOLESKY` for #ui("Iterative").

The monitor shows how a solve went: for the direct solver the size of the factor and the
times of ordering, factorization and solution; for the iterative one the size of the
preconditioner, the number of iterations and the remaining relative residual (below 1e-12).

=== Static

#fields(
  [Nonlinear geometry (Nlgeom)], [Large deformations (`NLGEOM`).],
  [Incrementation], [#ui("Default") leaves the incrementation to CalculiX. #ui("Automatic") uses
    the time and increment values below, #ui("Direct") uses fixed increments.],
  [Max. increments], [Largest number of increments (`INC`).],
  [Time period], [Length of the step (default 1).],
  [Initial / Min. / Max. increment], [Increment sizes for automatic incrementation.],
)

=== Frequency

Computes eigenfrequencies and mode shapes (`*FREQUENCY`). A frequency step takes no loads.

#fields(
  [Perturbation], [Uses the stress state of the previous step (prestress).],
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
  [Preload from previous step (Perturbation)], [Takes the state of the previous step as
    preload.],
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

Incrementation and equation solver are the same as for a static step.

=== Visco

A static step over a creep time in which materials with a creep law creep (`*VISCO`), see
@creep.

=== Field outputs <step-field-outputs>

Field outputs control what the solver writes to the results as `*NODE FILE`,
`*EL FILE` and `*CONTACT FILE`. A new step gets a node and an element field output whose
variables depend on the step type. Create more with #ui("Create ...") on #ui("Field Outputs") of a
step or with #menu("Model", "Create Field Output ..."). Edit one with a double-click. Hovering over
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
`*EL PRINT`, `*CONTACT PRINT`). Create them with #ui("Create ...") on #ui("History Outputs") of a
step. Hovering over a variable shows what it is.

#fields(
  [Type], [#ui("Node Output") (region of nodes), #ui("Element Output") (parts, element set or
    selection) or #ui("Contact Output") (a contact pair).],
  [Variables], [Node: `RF U V NT RFL`. Element: `S E ME PEEQ HFL ENER ELSE ELKE EVOL EBHE`.
    Contact: `CDIS CSTR CELS CNUM CF`.],
  [Totals], [#ui("Yes") adds the sum over the region, #ui("Only") writes only the sum.],
)

#screenshot("history-output-dialog.png", [Element history output of picked faces])

CalculiX prints a history output at every increment, so a step gives one row per increment:
a linear static step one row, a nonlinear step (Nlgeom, contact, plasticity) as many rows as it
takes increments, a frequency or buckle step one row per mode. A history output also stays in
effect in the following steps until a step defines history outputs of the same type (node, element or contact) of its own.

When results are opened, the `.dat` file is read too, and the history outputs appear on the
#ui("Results") tab (@history-outputs).

==== Forces and moments of a point <history-point>

For the forces, moments and rotations of a rigid body or of a displacement on a point, choose
#ui("Reference Point") as the region of a node output. The list offers the reference points
under #ui("Features") (@features), which a rigid body constraint drives (@rigid-body), and the
displacements of all steps that are written with a point (@displacement-behavior), whose point
exists only in the input file. The point's values appear under the name of the history output: `RF` gives the forces
(`FORCES`) and the moments (`MOMENTS`, `RM1` to `RM3`), `U` the displacements and the rotations
(`ROTATIONS`, `UR1` to `UR3`, in radians), `V` the velocities and the angular velocities.

#screenshot("history-output-point.png", [Node history output of the point of a
  displacement])

The solver computes reaction forces and moments only at the point of a rigid body: a rigid body
constraint or a displacement on a #ui("Rigid") point. At a #ui("Deformable") point only the
displacements and rotations are printed.
