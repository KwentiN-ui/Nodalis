#import "../../template.typ": *

== Cyclic symmetry <cyclic-symmetry>

Disks, impellers, gears and flanges repeat one sector around an axis. With cyclic symmetry only
that sector is meshed, and a #ui("Frequency") step still computes the eigenfrequencies of the
whole part: the solver solves the sector once for every nodal diameter, the number of lines
through the axis along which a mode shape does not move. The results are expanded to the whole
part. For a static step the tie makes both cut faces move alike (nodal diameter 0).

=== The cyclic symmetry tie

Create a tie (@contacts) between the two cut faces of the sector and tick
#ui("Cyclic symmetry") in its dialog. CalculiX rotates the master face by one sector about the
axis onto the slave face (`*TIE, CYCLIC SYMMETRY` and `*CYCLIC SYMMETRY MODEL`); which face is
which does not matter.

The number of sectors is not entered: Nodalis measures the angle between the two cut faces
about the axis and divides 360° by it, 12 sectors for a 30° sector. The solver solves only sectors
that close the full circle, so an angle that does not divide 360° (such as 25°) is an error in
the dialog and in the model check; it usually means a wrong axis or a wrong face. While the
dialog is open, the 3D view draws the other sectors as a wireframe around the meshed one, so the
axis and the faces can be checked at a glance.

#screenshot("cyclic-symmetry-tie.png", [Tie dialog with the cyclic symmetry of a 30° disk
sector; the wireframe previews the other eleven sectors])

#fields(
  [Axis point / Axis direction], [A point on the axis of rotation and its direction.],
  [Sectors], [The number of sectors found from the cut faces (`N`), or why none fits.],
  [Results show], [#ui("All sectors") expands the results to the whole part
    (`NGRAPH=N`), #ui("Single sector") keeps only the meshed one, which keeps the results of
    large models small.],
)

Only 3D models can be cyclic symmetric. The nodes of the slave face follow the master face, so
Nodalis leaves them out of the boundary conditions: CalculiX refuses a node that is both bound
and fixed ("dependent side of a MPC and a SPC").

=== Frequency steps by nodal diameter

As soon as an active tie has a cyclic symmetry, the #ui("Frequency") step dialog shows a
#ui("Nodal diameter range") (`*SELECT CYCLIC SYMMETRY MODES`). A sector of $N$ sectors carries
the nodal diameters 0 to $N\/2$; unticked, all of them are computed. The
#ui("Number of eigenfrequencies") counts per nodal diameter.

#screenshot("cyclic-symmetry-step.png", [Frequency step of a cyclic symmetric model, nodal
diameters 0 to 3])

The model check (@model-check) reports cut faces that do not divide the full circle, a cyclic
symmetry tie in a 2D model and nodal diameters beyond $N\/2$.

=== Results

The modes are numbered on through the step, first those of nodal diameter 0, then 1 and so on.
The #ui("Step, Increment") list shows the nodal diameter of every mode (`1, 7 (ND 2)`), as does
the information block of the 3D view. Each mode of a nodal diameter above 0 has a twin of the
same frequency, turned by a quarter wave; the solver writes the real part (`DISP`, `STRESS`) and
the imaginary part (`DISPI`, `STRESSI`) of the complex mode. The eigenvalue table under
#ui("History Outputs") lists the nodal diameter of every mode in `NODAL_DIAMETER`.

#screenshot("cyclic-symmetry-result.png", [A mode with two nodal diameters of the disk, computed
on one 30° sector])
