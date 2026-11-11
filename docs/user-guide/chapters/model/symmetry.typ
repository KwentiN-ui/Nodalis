#import "../../template.typ": *

== Symmetry boundary condition <symmetry>

A #ui("Symmetry") boundary condition cuts a symmetric model in half: select the faces that lie in
the cut plane and Nodalis holds them as the mirror image would. You do not have to work out
which components of a #ui("Displacement/Rotation") boundary condition to fix. Create it with
#ui("Boundary Condition") on the #ui("Steps") tab and choose the type #ui("Symmetry").

#screenshot("symmetry-dialog.png", [Symmetry boundary condition on the front face of a block])

#fields(
  [Plane normal], [#ui("From selection") (default) fits the plane to the selected nodes;
    #ui("Normal") shows the normal found. #ui("Entered") takes the components #ui("N1"),
    #ui("N2"), #ui("N3") instead, e.g. for a single node of a beam.],
  [Region], [The faces, edges or nodes in the cut plane, a node set or a surface. On shells,
    select the edge in the cut: the plane goes through the edge, perpendicular to the shells. In
    2D models select the edges on the symmetry line.],
)

The boundary condition holds the displacement normal to the plane and, at nodes of beams and
shells, the rotations about the two axes in the plane. Solids have no rotations, so for them
only the normal displacement counts. The plane is stored as your selection, not as numbers, so it
follows the mesh after remeshing.

#screenshot("symmetry-symbol.png", [Symbol of a symmetry plane: a cone along the normal and
plates for the held rotations])

In the input file, a plane normal to a coordinate axis becomes a plain `*BOUNDARY`, e.g. for the
plane $z = "const"$ the components 3, 4 and 5. Any other plane gets a local coordinate system for
its nodes (`*TRANSFORM, TYPE=R`) with the local x axis along the normal, and `*BOUNDARY` holds
the local components 1, 5 and 6.

=== Beams on the plane <symmetry-beams>

A beam can lie on the symmetry plane itself, such as a rope running in the groove of a pulley
cut in half along its axis. Select its nodes together with the cut faces, or in a boundary
condition of its own (a straight beam alone fixes no plane: enter the normal). The plane holds
the beam's displacement normal to it and its rotations about the two in-plane axes; the
rotation about the normal stays free, so the beam bends in the plane.

The half model holds only half of such a beam. With #ui("Halve their section") (on by
default), Nodalis gives every beam element whose nodes all lie in the region half its section
when it writes the input file: half the area, second moments of area and torsion stiffness,
and so half the mass. The dialog says how many beam elements that are. Stresses stay those of
the full beam; section forces and reactions are those of the half model. Loads on the beam are
not halved: apply half of a point load such as the rope tension yourself, as for any load on
the cut plane. A rope keeps its full contact radius and touches the halved pulley with the
half of its circumference that lies in the model.

#screenshot("symmetry-beams.png", [A rope on the symmetry plane of a half pulley: the
rope's 80 elements get half their section])

#limitation[Beams with a rectangular, circular, pipe or box profile are expanded into solid
elements by the solver. On a symmetry plane their cross sections are held against turning, so
they hardly bend, and they cannot be halved; the model check warns (*Expanded beams on a
symmetry plane*). Give them a general profile with the same values instead. General, I, T, U,
L, sketched and rope profiles are computed as beams and work on the plane.]

=== Warnings <symmetry-warnings>

If the selection makes no sense as a symmetry plane, the dialog says so in red, and the model
check marks the boundary condition with a warning sign (@model-check):

- *Not planar*: the nodes are not all on one plane, e.g. a curved face or faces of two
  different planes. The message gives the largest distance from the best-fitting plane. The
  input file is still written, with that plane.
- *No plane*: a single node, or nodes on a line that is not a shell edge, do not fix a plane.
  Enter the normal, or the input file cannot be written.
- *Axisymmetric*: in an axisymmetric model only a line of constant y (perpendicular to the axis)
  is a symmetry plane.
- *Expanded beams*: beams with a rectangular, circular, pipe or box profile on the plane, see
  @symmetry-beams.
- *Shared nodes*: CalculiX reads every boundary condition and point load at the nodes of a
  skewed plane in its local system. Zero displacements in all directions, such as a fixed
  support at an edge of the plane, are the same in any system and give no warning.

#screenshot("symmetry-warning.png", [Warning for a selection on two faces at right angles])

#limitation[A node can only have one local system. Where two skewed symmetry planes meet, keep
the line between them out of one of the selections and support it with a
#ui("Displacement/Rotation") boundary condition. Shells with a skewed plane are a little softer
next to the plane than with a plane normal to an axis, because CalculiX couples the expanded
shell nodes there (about 0.5 % in a plate one element wide).]
