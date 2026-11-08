#import "../../template.typ": *

== Complex Frequency step <complex-frequency>

A #ui("Complex Frequency (rotating, Coriolis)") step computes the eigenfrequencies of a rotating
structure with its Coriolis forces, for example the whirl modes of a shaft. Create it with
#menu("Steps", "Step") and choose the type in the dialog. The step is complete on its own: it
holds the rotation, and no static or frequency step has to come before it.

#screenshot("complex-frequency-step.png", [Complex Frequency step])

#fields(
  [Rotational speed], [Speed of the rotation in radians per time; the line below shows it in
    rpm.],
  [Rotating parts], [The parts (or an element set) that rotate. With none picked the whole
    model rotates; otherwise only the picked parts carry the centrifugal load and the Coriolis
    forces, and the rest of the model stands still.],
  [Rotation axis], [#ui("Point and direction") types the axis below. #ui("z axis of ...")
    takes the z axis of a coordinate system (#menu("Features", "Coordinate System")) through
    its origin, for example one placed at the hub by picking on the geometry; the line below
    shows the axis it gives. The axis follows the coordinate system when it is moved or
    renamed.],
  [Axis point X, Y, Z], [A point on the rotation axis (with #ui("Point and direction")).],
  [Axis direction X, Y, Z], [Direction of the rotation axis (default z).],
  [Stress stiffening: Centrifugal prestress], [Stress stiffening and spin softening. Computes the eigenmodes of the structure
    preloaded by its centrifugal forces (default on): the stresses of the preload stiffen it,
    the centrifugal forces soften it (spin softening). Off, the eigenmodes are those of the
    structure at standstill, as some other programs compute them by default; switch it off to
    compare with them.],
  [Coriolis forces: Gyroscopic coupling], [Couples the eigenmodes by the
    Coriolis forces (default on). Without it the step gives the eigenfrequencies of the
    (preloaded) structure.],
  [Complex frequencies], [How many complex eigenfrequencies to compute (default 10).],
  [Eigenmodes used], [How many eigenmodes the complex modes are combined from (default 20,
    at least the number of complex frequencies). More eigenmodes give more accurate complex
    frequencies and take longer.],
  [Equation solver], [As in a Frequency step.],
)

The rotating parts need a density. The solver computes
the step in up to three steps of its own: with stress stiffening a static preload by the
centrifugal forces, then the eigenmodes (with stress stiffening as a perturbation of the
preload), then the complex modes. Only the complex modes are kept as results. The step starts
from the state of the step before it: a bolt preload of an earlier Static step stays in place
and stiffens the rotor as well. Loads of the step itself have no effect; the boundary
conditions hold in all three solver steps.

=== A rotor on a standing structure

With #ui("Rotating parts") a rotor turns on a structure that stands still, e.g. the blades and
hub of a wind turbine on its nacelle and tower, or a disk on a shaft in a housing. The rotor is
described in its rotating frame (centrifugal prestress, spin softening and Coriolis forces), the
standing parts in the fixed frame, and both are joined at their common nodes or ties. This is
exact for a rotor that is axisymmetric (a disk, or a bladed wheel with at least three blades on a
stiff support): its stiffness and mass seen from the standing side do not change while it turns.

For a rotor that is not axisymmetric on a flexible support, such as three blades on a tower, the
coupled system changes with the rotor position and really is time-periodic: the standing
structure sees the blades pass. The step then computes the modes for the rotor frozen in the
position it is modeled in. This is the usual approach of commercial codes, and it is good for
the frequencies of rotor and tower modes as long as they lie apart. Where a rotor mode at
frequency f meets the tower, the standing side sees it at f ± n·Ω (n the nodal diameter,
Ω the speed); resonances near these crossings and the 3P excitation of the tower need a
multi-blade (Coleman) or Floquet analysis, which the step does not do. The Campbell diagram
(see the results chapter) shows the stationary frame frequencies f ± n·Ω.

The model check (@model-check) warns about a Coriolis step at speed 0 ("No rotation").

Join bonded parts of a rotor with a Tie constraint (#menu("Connections", "Tie")), not
with a contact pair with tied surface behavior: the Coriolis coupling of the complex frequency
step misses the contact, so whirl pairs do not split and modes go missing. The model check warns
about it ("Contact with Coriolis").

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
