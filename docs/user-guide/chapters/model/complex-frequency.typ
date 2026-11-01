#import "../../template.typ": *

== Complex Frequency step <complex-frequency>

A #ui("Complex Frequency (rotating, Coriolis)") step computes the eigenfrequencies of a rotating
structure with its Coriolis forces (`*COMPLEX FREQUENCY`), for example the whirl modes of a
shaft. Create it with #menu("Model", "Create Step ...") and choose the type in the dialog.

#screenshot("complex-frequency-step.png", [Complex Frequency step])

#fields(
  [Coriolis forces of the rotation (CORIOLIS)], [Adds the Coriolis matrix of the rotation
    (default on). Without it the step repeats the frequencies of the frequency step.],
  [Number of complex frequencies], [How many complex eigenfrequencies the solver computes
    (default 10).],
)

The step needs two steps before it, in this order:

+ A #ui("Static") step with a #ui("Centrifugal") load (@loads). Its rotation axis and speed
  define the rotation; the Coriolis forces come from this load.
+ A #ui("Frequency") step with #ui("Store matrices and eigenmodes") and #ui("Perturbation")
  switched on. The complex frequency step is solved on the eigenmodes this step writes to the
  `.eig` file. Its number of eigenfrequencies limits how many complex frequencies make sense.

The complex frequency step writes `*STEP, PERTURBATION` exactly when this frequency step has
#ui("Perturbation") switched on, since CalculiX stops when the two steps differ ("the .eig-file
was created without perturbation info"). Without perturbation the modes ignore the stiffening by
the centrifugal preload of the static step.

The complex frequency step takes the boundary conditions of the step before it and no loads. The
model check (@model-check) reports a missing stored frequency step as an error ("No stored
eigenmodes") and a Coriolis step without a centrifugal load as a warning ("No rotation").

Join bonded parts of a rotor with a Tie constraint (#menu("Interaction", "Advanced", "Create Tie ...")), not
with a contact pair with tied surface behavior. CalculiX leaves contact pairs out of a frequency
step without perturbation, so the parts they hold float; and even with perturbation the Coriolis
coupling of the complex frequency step misses the contact, so whirl pairs do not split and modes
go missing. The model check warns about both ("Contact ignored", "Contact with Coriolis").

The node field output of a new complex frequency step is `U, PU`: `PU` writes the magnitude and
phase of the displacements of every mode (`PDISP`), which the whirl animation needs.

=== Results of a complex frequency step

Every complex mode is its own increment of the step; the increment list shows its real part
(the frequency in Hz) as #ui("Complex frequency (real part)"). The deformed shape and the legend
show the magnitudes of the displacements.

With the #ui("Scale factor") animation (@animation) a complex mode whirls instead of swinging:
the displacement of every node is $u_k = "MAG"_k cos(phi + "PHA"_k)$, and the phase $phi$ runs
through a full turn over the frames. The status line shows the phase of the frame. Forward whirl
(with the rotation) and backward whirl (against it) are told apart by the direction the shape
turns in.

#screenshot("complex-frequency-whirl.png", [A whirling complex mode of a spinning cantilever])

The `.dat` file of the step is read into the history outputs (@history-outputs). For every mode,
`EIGENVALUE_OUTPUT` holds `OMEGA` and `FREQUENCY` (the real part) and `OMEGA_IM` (the imaginary
part of the eigenvalue, the damping of the whirl), and `TURNING_DIRECTION` is +1 for a forward
and −1 for a backward whirling mode, as CalculiX prints it (F or B).

#note[A Campbell diagram over the speed of rotation needs one analysis per speed; Nodalis
runs this sweep for you, see the Campbell diagram section of the results chapter.]
