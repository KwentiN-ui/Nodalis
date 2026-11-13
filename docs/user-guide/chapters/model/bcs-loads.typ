#import "../../template.typ": *

== Boundary conditions <boundary-conditions>

Boundary conditions belong to a step. Create them with #ui("Boundary Condition") on the
#ui("Steps") tab of the ribbon; they go into the active step (@active-step). The #ui("Step")
row at the top of the dialog moves a boundary condition to another step. The dialog lists only
the types the step accepts. In the 3D view, supports are drawn as cones (translations) and
plates (rotations).

#screenshot("boundary-condition.png", [Support holding a face in all directions])
#screenshot("bc-2d-reference-point.png", [`RZ` (`UR3`) on the reference point of a plane model])

#fields(
  [Displacement/Rotation], [Prescribed values for the ticked components `X`, `Y`, `Z`
    (length) and `RX`, `RY`, `RZ` (angle in radians). In a cylindrical
    #ui("Coordinate system") they are named #ui("Radial"), #ui("Tangential"), #ui("Axial") and #ui("Rot. radial"), #ui("Rot. tangential"), #ui("Rot. axial")
    instead (radial, tangential, axial); the names follow the chosen system at once. In the
    input file and the results they are the components 1, 2, 3 (`U1` to `UR3`). Unticked components are free. A new
    one holds all components at zero, a fixed support. Leave the rotations of a face free and
    it turns like a hinge, also on solids, whose nodes have no rotations of their own
    (@displacement-behavior). In plane stress and plane strain models `X`, `Y` and `RZ`
    are shown, in axisymmetric models `X` and `Y`. On a beam node give all three rotations
    (a clamp) or none (a pin): CalculiX holds all of them as soon as one is given, and in
    Nlgeom steps such a node mostly stops the step from converging, so the model check warns
    "Beam rotations partly held". On a reference point of a rigid body (@rigid-body) the
    components move the point and so the rigid body.],
  [Temperature], [Prescribed temperature (heat transfer, coupled and uncoupled steps); a
    #ui("Distribution") (@distributions) makes it vary over the region.],
  [Submodel], [Displacements taken from the global model (static steps; offered only when the model type in the
    model properties is #ui("Submodel"),
    @submodel). The check boxes select the global displacements X to Z; the step of the global
    results is the one linked to the #ui("Global results") row. The dialog names that step,
    its analysis and the field outputs that write its displacements, and shows in red when
    nothing usable is linked (@submodel). The global results hold no
    rotations. Each node takes the displacement interpolated in the global element it lies
    in; a node a little outside the global mesh (a curved surface meshed differently) takes
    that of the nearest point of the global mesh, and the solver warns how many nodes do and
    how far away the farthest is.],
  [Symmetry], [Holds the selected faces in their plane like a mirror image (@symmetry).],
)

The region is a selection of faces, edges or nodes, a node set or a surface. Displacement and
temperature boundary conditions can follow an #ui("Amplitude") (@amplitudes). Each component of
a displacement can also follow a table of its own over the step time (@value-tables).

=== Behavior: nodes or a point <displacement-behavior>

What a #ui("Displacement/Rotation") does depends on what is selected. Nodes of solid and 2D
elements have displacements only. Holding all nodes of a face in `X` to `Z` therefore also
keeps the face from turning, whatever the rotations say. To let a face turn, Nodalis holds a
point at its centre instead, and the face follows the point, as Ansys' remote displacement does:
a beam end on a hinge, a shaft end turned by an angle, a bolt hole held in place while the part
turns about it. The line below #ui("Behavior") always says which of the two the input file gets.

#screenshot("bc-cylindrical-directions.png", [Components of a support in a cylindrical
  system: Radial, Tangential, Axial and the rotations about these directions])

#screenshot("displacement-hinge.png", [End face of a beam held in `X` to `Z` and in the twist
  `RX`, free to turn about `RY` and `RZ`: a rigid point at its centre, about which it turns])

#fields(
  [Behavior], [#ui("Automatic") (default) chooses as described below. #ui("Each node") holds
    every node directly. #ui("Rigid") and #ui("Deformable") always use a point.],
  [Coordinate system], [The directions of the components (and of the coordinates below):
    #ui("Global") or a coordinate system under #ui("Features") (@features). In a cylindrical
    system the components are radial, tangential and axial, and the dialog
    names them so as soon as the system is chosen. Held at each node, the directions
    are those at the node (a `*TRANSFORM` of the nodes); held at a point, those at the point.
    The cones and plates in the 3D view point along the axes of the system, in a cylindrical
    one at nodes spread over the selection.],
  [Location], [Only with a point. #ui("Centroid") puts the point at the area-weighted centre of
    the selected faces (of the edges in 2D, the mean of the nodes when no faces are selected).
    #ui("Coordinates") puts it anywhere, given in the coordinate system; switching starts from
    the centroid. #ui("Point") shows where the point is, in global coordinates.],
  [Pinball region], [Only with a point. When ticked, only the selected nodes within this
    distance of the point follow it. Ticking it starts with the distance of the farthest node.],
)

#screenshot("displacement-local-system.png", [The two cut faces of a pipe segment, each held
  normal to itself (`U3`) and in its rotations in a coordinate system turned with the face:
  the cones and plates follow the axes of the systems])

#ui("Automatic") holds each node directly

- on a reference point (its rigid body follows it), on a single node, and in axisymmetric
  models, which have no rotations,
- when all nodes have rotations of their own (beams and shells): an edge of a shell held in
  `X` to `Z` with free rotations is a line hinge,
- when all rotations are ticked and zero (`UR3` in plane models): a clamp, or a roller with
  only `Z` ticked.

Otherwise, for a face or edge of solids or 2D elements whose rotations are free or turned, it
uses a #ui("Rigid") point at the centre. So a face with `X` to `Z` ticked and the rotations
free is a hinge about its centre, and a face with only `Z` ticked and free rotations may also
tilt. A rotation exposed as a parameter counts as turned, so a study never switches between the
two. Setting a location or a pinball region asks for a point as well.

The two behaviors of a point match those of Ansys:

- #ui("Rigid"): the selection keeps its shape and moves with the point as a rigid body. This
  stiffens the selection, like a pin glued into the bore. Reaction forces and moments are
  printed at the point.
- #ui("Deformable"): the point moves with the area-weighted mean of the selection, and the
  selection may deform. Its mean displacement equals that of the point and its mean rotation
  about the point that of the point, so a bore can ovalize while it is held. This is the
  distributing coupling (RBE3) of other programs.

You define no reference point, rigid body or equation for it. When the input file is written,
Nodalis adds a reference node and a rotation node at the point. #ui("Rigid") writes a
`*RIGID BODY` on the selection, with nodes of beams and shells coupled by `*EQUATION`s
(@rigid-body); #ui("Deformable") writes `*EQUATION`s between the nodes and the point. The values
go on these two nodes, with a `*TRANSFORM` when a coordinate system is chosen. The point is
computed on the current mesh, so a selection of geometry faces keeps working after remeshing.
In the 3D view the supports are drawn at the point with lines to the selection.

The ticked components, the coordinates of the location and the pinball radius have a #ui("P")
button, so a parameter study can vary them (@exposed-parameters). Only numbers that are set can
be exposed: tick the component, choose #ui("Coordinates") or tick the pinball region first. The
angle Theta of a cylindrical system is the one number that cannot be exposed.

#screenshot("displacement-expose.png", [The #ui("P") buttons of a displacement on a
  point: the location, the pinball radius and the prescribed components can be exposed])

- Nodes of the selection that another boundary condition holds directly in the same step (a
  support of each node, a symmetry plane, a submodel boundary) are left out of the point: a
  node cannot be both held and coupled there. A hinge face whose edge lies on a symmetry plane
  just works. Supports of other steps do not count: a face clamped node by node in Step-1 can
  turn about a point in Step-2.
- Displacements of several steps with the same name and selection share one point; when one
  step needs a point, all of them get it. Displacements with the same selection, behavior,
  coordinate system, location and pinball region share one point too. A study that varies the
  location or the pinball radius in one step only gives that step a point of its own on the
  same nodes, which the model check reports; expose the number in every step and link all of
  them to the same value to keep one point. The rigid body or the equations belong to the
  whole model, so they still act in steps without the boundary condition.
- A node can belong to only one rigid body or point: do not let the selection overlap that of
  another displacement on a point or a rigid body constraint (@rigid-body). The model check
  (@model-check) reports "Nodes in several rigid bodies or remote points" otherwise.
- The solver reads every support of a node in one coordinate system only. Where supports in
  different systems meet, such as at the edge between two cut faces of a pipe segment held
  normal to themselves, or between such a face and one held in global directions, the input
  file gives these nodes a system of their own in which every held direction is an axis, and
  each support holds the matching axis there. Directions that are not perpendicular to each
  other can only be combined when they are held at zero; otherwise writing the input file
  stops with a message naming the boundary conditions. Point loads on nodes held in a
  coordinate system of their own act in the directions of that system, as on a skewed
  symmetry plane (@symmetry): keep them off such nodes.
- Ansys features that are left out: the #ui("Beam") behavior and the augmented Lagrange
  formulation (Nodalis always writes exact constraints). Tabular values per component are
  available (@value-tables).

=== Motion preview <motion-preview>

When a displacement boundary condition follows an amplitude or a value table (@value-tables), a
play button appears right of the
#ui("Amplitude") field. It previews the prescribed motion in the 3D view: the nodes of the region
move by the prescribed displacements times the amplitude, passing through the amplitude's time
range in three seconds and starting again after a short rest. The button turns into a stop
button; the preview also ends when the dialog closes or the amplitude is removed. Changes to the
values, the amplitude or the region show while it plays.

#screenshot("motion-preview.png", [Tip of a cantilever moved up and down by an amplitude])

Only the nodes of the region move; the rest of the mesh stays in place, so the elements next to
the region stretch. The motion is exaggerated so that the largest displacement is a quarter of
the model size; the dialog shows the current time of the amplitude and this scale. Rotations
(`RX` to `RZ`) are not shown. Components with a table (@value-tables) move along their table,
so the preview also shows a motion in several directions at once.

=== Value tables <value-tables>

Instead of a fixed value, every component of a displacement (`X` to `RZ`) and of a force,
moment or pressure can follow a table of values over the step time. The small #ui("T") button
right of the value opens the table in a window of its own. The first click starts a table that
ramps from zero to the value at the end of the step; edit, add or paste rows there (two columns,
time and value, for example from a spreadsheet). The value field then shows the number of
points, and a click on it opens the table again. #ui("Remove Table") goes back to the fixed value.

#screenshot("value-table.png", [`X` and `Y` of one boundary condition, each with its own table])

This describes a combined motion with one boundary condition: for example the tip of a lever
pressed down by 0.8 mm in the first second and then moved 4 mm to the side while it rises to
0.8 mm, with the tables `X`: (0, 0), (1, 0), (2, 4) and `Y`: (0, 0), (1, -0.8), (2, 0.8). In
CalculiX a boundary condition has one amplitude for all its components, so such a motion needs a
boundary condition per direction; the Nodalis solver takes a curve per component.

- The values are the displacements themselves (in the unit of the field), not factors. They are
  linear between the points and constant before the first and after the last one.
- A component with a table ignores the #ui("Amplitude") of the boundary condition or load; the
  other components still follow it.
- Times count from the start of the step. Set the #ui("Duration") of the step to the last
  time of the table (a new static step lasts 1), otherwise the model check warns "Table goes
  beyond the step". Choose the #ui("Frames") so that the results show the points of interest,
  e.g. 2 frames for the times 1 and 2 above.
- The increments of a nonlinear static step also end at every point of the tables and
  amplitudes, between the frames too, so a motion that changes direction is applied along the
  given path even with large increments. A linear step does not depend on the path and is
  solved at the frames only. A dynamic step chooses its time step fine enough for the shortest
  segment of the tables.

== Loads <loads>

Loads are created like boundary conditions, with #ui("Load") on the #ui("Steps") tab, into the active
step. A frequency step takes no loads.

#screenshot("selection.png", [Force on the end face of a plate])

#fields(
  [Force], [The default load: a *total* force `FX`, `FY`, `FZ` on whatever the region holds,
    faces, edges or points (@force-region). When the input file is written it is distributed
    to the nodes (`*CLOAD`). With #ui("Apply to all nodes individually") the whole force acts
    at every node of the region instead, as a concentrated force in CalculiX; it is off by
    default.],
  [Excitation], [A force `FX` to `FZ` that follows a signal (impulse, sine, chirp) to shake
    the structure in a Dynamic step (@excitation).],
  [Remote Force], [Force `FX` to `FZ` and moment `MX` to `MZ` at a point that the selected
    faces, edges or points follow, rigidly or on average, as Ansys' remote force does
    (@remote-force). The way to put a moment on faces of solids.],
  [Moment], [`MX`, `MY`, `MZ` at the reference point of a rigid body (@rigid-body) or at every
    node of beams and shells, which have rotations. 3D models only.],
  [Pressure], [Pressure on faces; positive values push into the material (`*DLOAD`). A
    #ui("Distribution") (@distributions) makes it vary over the faces, e.g. with depth.],
  [Gravity], [Body force on the mass of whole parts or element sets (`*DLOAD`, `GRAV`); the materials need
    a density. Point masses carried by the region are pulled too, the dialog names them; the
    region #ui("Point Mass") pulls one point mass alone (@point-mass). #ui("Definition") #ui("Standard Gravity") (default) takes only a
    #ui("Direction") along an axis (`+X` to `-Z`, in plane models only `X` and `Y`; default
    `-Z`, or `-Y` in plane models) with the standard acceleration of 9.80665 m/s² in every unit
    system. #ui("User Defined") takes the acceleration vector `g1` to `g3` (`g1`, `g2` in plane
    models). Switching keeps the nearest axis or the standard vector.],
  [Moving Load], [Wheel forces that travel over faces, one static step per position
    (@moving-load).],
  [Flux], [A *total* heat flow into whatever the region holds, faces, edges, points or whole
    parts (@flux-region); positive flows in. It covers CalculiX' concentrated, surface and
    body flux. With #ui("Apply to all nodes individually") the whole heat flow enters every
    node of the region instead (a concentrated flux); it is off by default.],
  [Film (Convection)], [Convection to the surroundings with #ui("Sink temperature") and
    #ui("Film coefficient") (`*FILM`).],
  [Radiation], [Radiation to the surroundings with #ui("Sink temperature") and
    #ui("Emissivity") (`*RADIATE`). Needs the physical constants of the model
    (@model-properties).],
)

#screenshot("gravity-dialog.png", [Gravity dialog: standard gravity along `-Z`])

Every load can follow an #ui("Amplitude"). Film and radiation have a second amplitude for the film
coefficient or the emissivity. The components of a force or moment and a pressure can also
follow a table of their own over the step time (#ui("T") button, @value-tables). Forces,
excitations and moments can be given in a coordinate system of their own and follow the
deformation (@load-directions).

In 2D models face loads act on element edges. In axisymmetric models forces act on the full
circumference.

#limitation[Line loads (force per length) are not available in the dialogs; a
#ui("Force") along edges takes the total force (@force-region). Bolt preloads: a
#ui("Bolt") under #ui("Connections") (@bolts). Defined temperature fields: @defined-fields.]

=== What a force acts on <force-region>

A #ui("Force") picks its region like a boundary condition: in the #ui("Selection") window
#ui("Faces, edges and points") takes the face, edge or corner point under the click, and
#ui("Region") can also be a node set, a surface or the #ui("Reference Point") of a rigid body
(@rigid-body). The force is always the total and is distributed to the nodes when the input file
is written:

#fields(
  [Faces], [By the face areas and the element shape functions. In 2D models the faces are the
    edges of the outline.],
  [Edges], [By the edge lengths, as the consistent nodal forces of a constant line load: on
    quadratic elements an edge's end nodes take 1/6 and its midside node 4/6 of the edge's
    share. Useful on the free edge of a shell or a membrane.],
  [Points, nodes, node sets], [In equal shares, so a single node, such as the tip of a beam,
    takes all of it. Nodes picked with the mesh based modes count as points.],
  [Reference point], [The whole force acts on the rigid body.],
)

When a selection mixes them, faces come before edges and edges before points: only the faces of a
selection of faces and edges carry the force.

#screenshot("surface-traction-edge.png", [Force of 100 N along the free edge of a shell plate])

#screenshot("force-dialog.png", [Force dialog with the Selection window])

=== What a heat flow acts on <flux-region>

A #ui("Flux") in a heat transfer or coupled step picks its region like a force: faces, edges and
points in the #ui("Selection") window, or a node set, a surface, whole #ui("Parts") or an element
set as #ui("Region"). The value is always the total heat flow (power, W in SI) and is distributed
to the nodes when the input file is written (`*CFLUX`), with the same rules as a force:

#fields(
  [Faces], [By the face areas: a constant heat flux density, the surface flux. 10 W on faces of
    0.01 m² are 1000 W/m².],
  [Edges], [By the edge lengths, e.g. along the edge of a shell.],
  [Parts, element sets], [By the volume of the elements and their shape functions: a constant
    heat source per volume, the body flux. In plane models by the area of the elements, in
    axisymmetric models by the volume of the revolution.],
  [Points, nodes, node sets], [In equal shares.],
)

Faces come before edges and edges before points. A flux density or a heat source per volume is
entered as the total: multiply it by the area or the volume of the region. In axisymmetric models the heat flow is the one through the full circumference.

#screenshot("flux-dialog.png", [Flux of 50 W on the end face of a bar in a heat transfer step])

=== Coordinate system and follower loads <load-directions>

A #ui("Force"), an #ui("Excitation") and a #ui("Moment") take their components in a
#ui("Coordinate system") of the features (@features), like a displacement does; the
default is the global one. In a rectangular system `FX` to `FZ` act along its x, y and z axes;
in a cylindrical one the dialog names them #ui("F radial"), #ui("F tangential"), #ui("F axial") (and the moments
#ui("M radial") to #ui("M axial")): radial, tangential and axial at every node, so a force on a ring of
faces can push outward all around or twist it. The arrows in the 3D view point the way the
force acts.

#screenshot("load-follower.png", [Force in a cylindrical system that follows the deformation])

#fields(
  [Coordinate system], [The directions of the components. The time tables of the components
    (#ui("T")) need the global system.],
  [Follow the deformation], [A follower load: under large deformations (#ui("Nlgeom") on the
    step, also in explicit dynamics) its direction turns with the region. The region turns by the
    rigid rotation that fits the motion of its nodes best; on the reference point of a rigid
    body the force turns with the body, at a single node of a beam or shell with its
    rotation. The size of the load stays. A force normal to a lever arm stays normal to it,
    and a force along a slender beam stays along it however far the beam turns. Without
    #ui("Nlgeom") it acts like a fixed load. In a coordinate system that moves with a
    selection (@coordinate-systems) the load turns with that system instead, and the box is
    replaced by a note.],
)

The rotation of a region comes from the motion of all its nodes, so a region that deforms a lot
in itself, such as a long edge that bends, turns by its mean rotation. Nodes on a straight line
do not show a turn about that line: the load then turns with the line only. A single node of a
solid has no rotation, so a follower load there keeps its direction.

=== Remote force <remote-force>

Nodes of solids have no rotations, so a #ui("Moment") cannot act on a face of a solid directly,
and a #ui("Force") on a face acts at the face, not at a point away from it. A
#ui("Remote Force") puts both on a point that the selection follows, the same point as that of a
displacement on a point (@displacement-behavior): a torque on a shaft end, a bending moment on a
flange, the force of a lever acting at its pin, a load on a bracket at the position of a part
that is not modelled. Pick faces, edges or points like for a #ui("Force"); Nodalis adds the point
and the coupling when the input file is written.

#screenshot("remote-force.png", [Remote force with `FY` and a torque `MX` on the end face of
  a solid beam: the arrows at the point with lines to the face])

#fields(
  [FX, FY, FZ], [Force at the point, in the directions of the coordinate system (#ui("F radial"),
    #ui("F tangential"), #ui("F axial") in a cylindrical one). In plane models `FX` and `FY`, in axisymmetric models on the full circumference.],
  [MX, MY, MZ], [Moment about the axes of the coordinate system through the point. Plane
    models take `MZ`, axisymmetric models none.],
  [Behavior], [#ui("Rigid") (default): the selection keeps its shape and follows the point as a
    rigid body (`*RIGID BODY`), which stiffens it. #ui("Deformable"): the point moves with the
    area-weighted mean of the selection, which may deform (`*EQUATION`s, RBE3); the face takes
    the load spread by area without being stiffened.],
  [Coordinate system], [The directions of the components and of the coordinates below.],
  [Location, Pinball region], [Where the point is and which nodes follow it, as for a
    displacement on a point (@displacement-behavior). #ui("Point") shows the position in global
    coordinates.],
)

A remote force with the same selection, behavior, coordinate system, location and pinball region
as another remote force, also of another step, or a displacement on a point shares that point:
a remote force and a remote displacement on one face act on the same rigid body. Like that of a
displacement, the coupling belongs to the whole model and stays in steps without the load. So a
#ui("Rigid") point keeps its face rigid in every step: a #ui("Deformable") force on the same
face in another step still loads the face, but as a rigid one, and the model check (@model-check)
warns "Deformable coupling on a rigid selection". To compare both behaviors, use one analysis for
each.
Nodes held directly by a support in a step with the load are left out of the coupling. On the
reference point of a rigid body the force and moment act on the rigid body itself, without a
point of their own. The force and moment components have a #ui("P") button for parameter
studies (@exposed-parameters).

== Hiding symbols <hide-symbols>

#ui("Hide") in the context menu of a boundary condition or load removes its symbol from the 3D
view, #ui("Show") brings it back; the tree marks a hidden item with an eye. #ui("Hide All") and
#ui("Show All") in the context menu of #ui("BCs") or #ui("Loads") of a step do this for all of
them. Constraints are hidden the same way (@constraints). A hidden item is still
drawn while it is selected, and the setting is saved with the project. Hiding changes only the
view; #ui("Deactivate") leaves an item out of the input file.

#screenshot("hide-loads.png", [Hidden pressure load and the context menu of Loads])
