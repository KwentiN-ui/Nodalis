#import "../../template.typ": *

== Sections <sections>

A section assigns a material to elements. Every element needs a section, otherwise the model check
reports "Elements without material". Likewise an element may be in only one section: if the same
part (or overlapping element sets) appears in two sections, the model check reports
"Part in more than one section" on the later section. Create sections with #menu("Model", "Create Section ...").

#screenshot("section.png", [Section dialog])

#fields(
  [Type], [#ui("Solid"), #ui("Truss") or #ui("Beam").],
  [Material], [One of the model's materials.],
  [Region], [#ui("Parts") (picked in the 3D view) or an #ui("Element Set") (@mesh-sets).],
)

=== Solid section

For solid elements and the plane and axisymmetric elements of 2D models. In plane stress and plane
strain models the section has a #ui("Thickness") (default 1). Written as `*SOLID SECTION`.

=== Truss section

For line parts that carry only axial force. #ui("Cross-section area") must be positive. The lines
are written as `T3D2` truss elements.

#note[Trusses must form a stable truss structure. If they can move without resistance, the model
check reports "Truss structure is a mechanism".]

=== Cross-sections <cross-sections>

The profile of a beam is a cross-section of its own, kept under #ui("Sections") >
#ui("Cross-Sections") in the FE Model tree, so that several beam sections share one definition.
Double-click #ui("Cross-Sections"), right-click it and choose #ui("Create ..."), or use
#menu("Model", "Create Cross-Section ...").

#fields(
  [Profile], [#ui("Rectangular") (thicknesses a and b in the 1- and 2-direction),
    #ui("Circular") (radius), #ui("Pipe") (outer radius and wall thickness) or #ui("Box") (widths a
    and b and the wall thicknesses t1 to t4 at +1, +2, -1 and -2); or one of the general profiles
    below.],
  [I-Beam, T-Beam, Channel (U)], [Height h in the 2-direction, flange width b in the 1-direction,
    web thickness tw and flange thickness tf. The T has its flange at +2, the channel its web at
    -1.],
  [Angle (L)], [Legs h along +2 and b along +1 from the corner, thickness t.],
  [General (A, I)], [A profile given by its values: area A, the second moments of area I11 and
    I22 about the 1- and 2-axis through the centroid and the factor k of CalculiX described
    below. It is drawn as the rectangle with the same moments. Switching another profile to
    #ui("General") takes over its values.],
  [Sketch], [A profile of your own drawn as polygons: an outline and any number of holes
    (@sketched-profiles).],
)

The sketch below the fields shows the profile with the 1-direction to the right and the
2-direction up, starting at the beam's nodes. The red dashed lines are the principal axes through
the centroid (circle): #ui("I") has the largest second moment of area, #ui("II") the smallest.
Below the sketch are the area A, the position of the centroid, the second moments of area I11
(about the 1-axis) and I22 (about the 2-axis) and the principal moments with the angle of axis I
from the 1-direction, in the units of the model.

#screenshot("cross-section-dialog.png", [The cross-sections under Sections and the dialog of a box
profile whose thicker wall at +1 moves the centroid in the 1-direction])

==== General profiles <general-profiles>

CalculiX has beam sections of its own for rectangles, circles, pipes and boxes only, which it
expands into solids. I, T, channel, angle and general profiles are written as a general section
(`SECTION=GENERAL`) of its user element `U1`, a linear beam that knows the profile by its area
and second moments of area. So:

- The beam axis runs through the centroid: the profile lies with its centroid on the nodes, and
  the section takes no offsets.
- The beams need lines with 2 nodes; mesh the lines linear.
- An unsymmetric profile such as an angle is written about its principal axes: Nodalis turns
  the normal about each beam by the angle of axis I, so that it bends askew as it should.
- The solver computes Static steps without large deformations (#ui("Nlgeom")) and Frequency steps
  (from CalculiX 2.23, which Nodalis ships), and the modal and steady state dynamics built on
  them. The model check refuses other steps with "Step not possible with general beam profiles".
- The shear stiffness is k G A with k the share of the area in the web (a leg of an angle), and
  the torsion stiffness k G (I11 + I22). For open profiles the torsion is far too stiff, CalculiX
  knows no warping.
- The results draw the beams as lines with their profile swept along them. CalculiX gives
  section forces instead of stresses for them; the results show no stresses on these beams.

#screenshot("cross-section-angle.png", [An angle: its principal axes are turned against the
  1-direction, and CalculiX takes it as a general section of U1])

#screenshot("general-beam-results.png", [A portal frame of I-beam columns and a channel
  girder, solved as U1 beams, with the profiles in the results])

==== Sketched profiles <sketched-profiles>

#ui("Sketch") draws a profile of any shape. Switching a profile to #ui("Sketch") starts from its
outline, so an I-beam becomes the polygon of the I-beam to adjust; a round profile becomes a
polygon of 48 points. The dialog shows the sketch on a grid instead of the fields:

- Drag a point to move it. It snaps to the grid, whose step is a round number in the model's
  units; hold Shift to place it freely.
- Double-click an edge to add a point there, right-click a point to delete it. An outline or
  hole keeps at least 3 points.
- #ui("Polygon") picks the outline or a hole for the table below it; #ui("Add Hole") puts a small
  square hole inside the outline to drag into place, #ui("Delete Hole") removes the hole picked.
- #ui("Points") lists the 1- and 2-coordinates of the polygon picked, to type exact values;
  #ui("Add Point") adds one halfway to the next point, #ui("Delete Point") removes the one
  picked.

The centroid and the principal axes follow every change, with the area and second moments of
area below. The sketch needs no particular origin: the profile lies with its centroid on the
beam's nodes, as all general profiles do (@general-profiles). Edges must not cross, and every
hole must lie inside the outline and outside the other holes; otherwise the dialog names the
problem, draws the polygons red and does not accept the cross-section. The shear factor k is
5/6. A sketch has no dimensions to expose as parameters.

#screenshot("cross-section-sketch.png", [A sketched girder: a trapezoid with a hole, its centroid
  above the middle of its height])

Every dimension can be exposed as a parameter with the #ui("P") button next to it, so that a
parameter study can vary the profile. Renaming a cross-section renames it in the beam sections
that use it. In the project schematic the cross-sections are a category of the setup that other
analyses can link to, like the materials, and a Cross-Sections block (@cross-sections-block)
keeps them in one place for several analyses.

=== Beam section

#fields(
  [Cross-section], [One of the model's cross-sections (@cross-sections); the dialog shows its
    sketch. A new beam section takes the first one.],
  [Normal (1-direction)], [#ui("Automatic") uses the global z axis, or x for beams parallel to z.
    #ui("Vector") takes the direction from x, y and z; it must not be parallel to a beam.],
  [Offset in 1-/2-direction], [Shifts the beam axis by a multiple of the section dimension.],
)

Beams are written as `*BEAM SECTION` with `B31` or `B32` elements. Pipe and box sections need
quadratic lines and are written as `B32R`, as CalculiX requires; general profiles need linear
lines and are written as `U1` (@general-profiles).

==== Beam profiles in the FE model and the results <beam-profiles>

The FE model draws beams as their profile swept along the line, as CalculiX expands them: the
1-direction is the #ui("Normal") turned perpendicular to the beam, the 2-direction the beam axis
crossed with it, and the offsets move the profile off the nodes (0.5 puts the nodes on the
positive face). Pipes and boxes show their hollow walls. Where two elements of a line meet
the profile runs on and its outline marks the element border; at the ends of a line, at kinks
of more than 45 degrees and at branches each element ends with a face. Trusses are drawn as round
bars of their cross-section area.

#screenshot("beam-profiles.png", [A frame with a rectangular beam (left), a pipe frame (right),
truss bracing and a roof of shells, drawn with their sections.])

In the Results workspace CalculiX hands the beams back as the solids it expanded them to
(pipes and boxes without their hollow). Trusses stay lines, and so do beams in results written with
`OUTPUT=2D`; Nodalis then sweeps the profiles of the input file CalculiX ran along the deformed
lines, coloured by the shown result. Such a profile is moved with its nodes, not turned by
their rotations.

Turn the profiles off with #menu("View", "Beam Profiles") or in the #link(<settings>)[settings]
under #ui("Graphics") to see the lines only. Picking still selects the lines.

#limitation[Line elements can only be used in 3D models.]
