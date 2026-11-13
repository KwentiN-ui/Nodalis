#import "../../template.typ": *

== Rigid bodies, moments and reference points <rigid-body>

A #ui("Rigid Body") constraint makes the nodes of a region move as one rigid body, driven by a
reference point: a bolt head, a bearing seat or a face a lever acts on. Create it with
#menu("Connections", "Constraint") and choose #ui("Rigid Body"). To only hold or move
a face as a rigid body about its centre, a #ui("Displacement/Rotation") boundary condition with
free or turned rotations (@displacement-behavior) needs no reference point.

#screenshot("rigid-body.png", [Rigid body constraint])

#fields(
  [Region], [The nodes that form the rigid body: a selection of faces, edges or nodes, parts, a
    node set or a surface.],
  [Reference point], [The point the body is driven at: #ui("New point") creates it with the
    constraint, or choose one of the reference points under #ui("Features") (@features).],
  [Point name], [Name of the new point, by default the next free #ui("RP-n").],
  [Behavior], [#ui("Rigid"): the region keeps its shape and moves with the point.
    #ui("Deformable"): the point follows the weighted mean motion of the region (as Nastran's
    RBE3), which keeps its own stiffness; the right choice for a point that only tracks a face,
    or that spreads a load without stiffening the face.],
  [Location], [Where the new point lies: #ui("Centroid of the region") (the area-weighted
    centre of its faces, of its edges in 2D, e.g. the centre of a hole),
    #ui("Center of the bounding box") of the region, or global #ui("Coordinates").
    #ui("Position") shows where it ends up.],
)

A new point is added under #ui("Features") > #ui("Reference Points") when you click
#ui("OK"), and one undo removes it together with the constraint. A point at the centroid or the
bounding box centre keeps that definition: it follows the mesh after remeshing. Edit it there to
move it; changing the region of the rigid body later does not move the point.

In the 3D view of the FE model, a rigid body is drawn as yellow lines from nodes spread over its
region to its reference point, with the symbols of any step. A selected or edited
rigid body is drawn in red; a deactivated one has no lines.

#screenshot("rigid-body-symbol.png", [Rigid body on the end face of a cantilever, driven by RP-1])

Written as `*RIGID BODY, NSET=..., REF NODE=..., ROT NODE=...`: Nodalis adds a reference node at
the point for the translations and a rotation node for the rotations, as CalculiX requires. A
deformable one is written as `*EQUATION`s between the two nodes and the region instead, as a
deformable remote displacement (@displacement-behavior); its point has no reaction forces to
print, so a history output of it leaves `RF` out.

Nodes of beams, trusses, shells and membranes in the region are an exception: CalculiX expands
these elements into solids internally and stops on such a node in a rigid body ("nodes
belonging to beam elements must not be subject to a RIGID MPC"). Nodalis couples these nodes
to the reference point with `*EQUATION`s instead, $u = u_"RP" + theta times (x - x_"RP")$, and
keeps the other nodes in the `*RIGID BODY`. In a linear analysis this is the same as a rigid
body. With large deformations (NLGEOM) the equations only hold for small rotations of the
point, and the model check (@model-check) warns "Beam or shell nodes in a rigid body". As in a rigid body, the rotations of beam and shell
nodes are not coupled: to join beams rigidly, give them a shared node with a #ui("Node Tie").

=== Loads and supports on a reference point

Boundary conditions and loads can act on a reference point instead of on nodes: choose
#ui("Reference Point") as the region in their dialogs. The point must drive an active rigid
body or point mass (@point-mass), otherwise the model check (@model-check) reports "Reference point without rigid body".

- A #ui("Displacement/Rotation") boundary condition on the point holds or moves the whole body; `UR1`, `UR2`, `UR3` turn it about the point.
- A #ui("Force") on the point pulls the whole body with the given force.
- A #ui("Moment") load (`M1`, `M2`, `M3`, written as `*CLOAD` on the rotational degrees of
  freedom 4 to 6) turns the body about the point. Moments can also act on nodes of beams or
  shells, which have rotations of their own, and need a 3D model.

#screenshot("moment-load.png", [Moment on the reference point of a rigid body])

The reaction forces and moments of a held reference point can be requested with a history
output (@history-outputs) of `RF` on the point.
