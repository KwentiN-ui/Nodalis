#import "../../template.typ": *

== Point masses <point-mass>

A #ui("Point Mass") constraint adds a mass, and if needed its rotary inertia, at a reference
point that a region of the model carries: a motor on a bracket, a lumped component on a beam,
a counterweight. Create it with #menu("Connections", "Constraint") and choose
#ui("Point Mass"). As with a rigid body (@rigid-body), the dialog can create the reference
point at the centre of the region.

#screenshot("point-mass.png", [Point mass with its rotary inertia in a local coordinate system])

#fields(
  [Region], [The nodes that carry the mass: a selection of faces, edges or nodes, parts, a
    node set or a surface. Not shown when a rigid body couples the chosen point already: the
    mass then rides on that body (#ui("Carried by")).],
  [Reference point], [Where the mass sits: #ui("New point") creates it with the constraint
    (#ui("Point name"), #ui("Location") as for a rigid body), or choose one of the reference
    points under #ui("Features") (@features).],
  [Behavior], [How the region carries the point. #ui("Rigid"): the region moves with the
    point as a rigid body and stiffens it. #ui("Deformable"): the point follows the weighted
    mean motion of the region, which keeps its own stiffness (as Nastran's RBE3).],
  [Mass], [The mass in the mass unit of the model (`t` in mm-t-s).],
  [Rotary inertia], [Adds the moments of inertia of the body about the point.],
  [Coordinate system], [3D: the axes the inertia tensor is given in; #ui("Global") or a
    coordinate system of the features. The tensor is turned into the global axes when the
    input file is written.],
  [I11, I22, I33], [3D: the moments of inertia about the three axes through the point.],
  [I12, I13, I23], [3D: the products of inertia, the off-diagonal entries of the inertia
    tensor ($I_(12) = -integral x y dif m$). Zero when the axes are principal axes.],
  [Iz], [2D: the moment of inertia about the axis normal to the plane through the point.],
)

The mass takes part wherever the solver works with masses: frequency steps, implicit and
explicit dynamic steps, and the modal procedures built on the frequencies. A static step is
affected only by gravity and centrifugal loads (@loads), in one of two ways:

- *With the parts.* A load on parts or element sets pulls the point mass too when the region
  holds elements the mass is coupled to (the elements of its region, or of the rigid body on
  its point); the dialog lists the point masses the load pulls along.
- *Alone.* With #ui("Region") #ui("Point Mass") the load pulls only the chosen point mass, not
  the elements that carry it, e.g. to give a motor its weight without that of the bracket.

The centrifugal force acts at the undeformed position of the point.

#screenshot("gravity-point-mass.png", [Gravity on the point mass alone])

The tensor must have no negative principal moment, the model check reports it otherwise. The
inertia of a body about one axis only (e.g. a flywheel about its shaft) may be given alone.
Point masses are not available in axisymmetric models. In the 3D view the point mass is a
ball at its reference point, with lines from its region as for a rigid body.

The point couples its region like a rigid body: boundary conditions, loads and history outputs
on the reference point work as described in @rigid-body. Written as
`*POINT MASS, NODE=..., ROTATION NODE=..., ELSET=..., NAME=...` with the mass and the tensor about the
global axes, `I11, I22, I33, I12, I13, I23`, on the reference and the rotation node of the
coupling (a plxs keyword, CalculiX has none for the rotary inertia); `ELSET` holds the elements
of the coupled nodes, for the gravity and centrifugal loads on them. `NAME` names the point
masses of a reference point for a gravity or centrifugal load on them alone.
