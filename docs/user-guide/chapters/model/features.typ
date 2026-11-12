#import "../../template.typ": *

== Features <features>

Under #ui("Features") in the tree you define reference points, coordinate systems and planes.
They are helpers for other functions (section views, plane results, paths, result
transformations) and are not written to the input file.

The FE Model and the Results workspace share the features of an analysis: both show the same
list, and a feature created, edited, renamed or deleted in one of them is changed in the other as
well. In the Results workspace the tiles are on the #ui("Evaluate") tab in the group
#ui("Features"). A coordinate system defined in the Results, e.g. to look at the stresses of a hub in
cylindrical coordinates, can then be used for a load or a remote displacement in the FE Model.
Adding or changing a feature that nothing in the model uses keeps the results up to date; only
features the model refers to, directly or through another feature, call for a new solution.
Result paths and plane results belong to the Results alone.

Points can be typed in or picked with #ui("..."). In the geometry mode #ui("Points"), the default
of the selection window, a click on a face or edge takes the corner point of the geometry nearest
to it, also in 2D models. With #ui("More") the mesh based modes pick a node, or the centre of the
nodes of a picked edge, element or surface, for example the centre of a hole.

#screenshot("point-picks-corner.png", [Picking a point: the corner nearest to the mouse is shown])

/ Reference Point: a named point. #ui("Create by") offers:
  - #ui("Coordinates"): typed in or picked, in the global system or in a coordinate system of
    the features chosen under #ui("Coordinate system"). In a cylindrical system the coordinates
    are R, Theta in degrees and Z. The point follows the coordinate system when it changes.
  - #ui("Center of gravity"): the area-weighted centre of the faces selected in the 3D view (in
    2D models of the edges), e.g. the centre of a hole or of an end face.
  - #ui("Bounding box center"): the centre of the box around the selected faces.
  A point at the centre of a selection moves with the mesh: after remeshing, a selection of
  geometry faces gives the new centre. #ui("Global position") shows where the point lies.
  #ui("Couple faces") adds a #ui("Rigid Body") constraint (@rigid-body) on the same faces with
  the point, so that loads, supports and history outputs on the point act on them:
  #ui("Deformable") (the default for a new point) lets the point follow the mean motion of the
  faces without stiffening them, #ui("Rigid") makes the faces keep their shape, #ui("None")
  adds no constraint, or removes the one the point already drives. The constraint follows a
  changed selection of the point. To track where a face goes independently of node numbers,
  create a point at its #ui("Center of gravity") with #ui("Deformable") coupling and request
  `U` on the point in a history output (@history-outputs).

  #screenshot("reference-point-coupling.png", [Reference point at the centre of an end face,
    coupled deformably to it])
/ Coordinate System: #ui("Rectangular") or #ui("Cylindrical"), see @coordinate-systems.
/ Plane: by #ui("Coordinate system") (a plane XY, YZ or XZ of a system with an offset), by
  #ui("Three points") (normal by the right-hand rule) or by #ui("Point and normal"). A plane
  built on reference points follows them when they change.

#screenshot("reference-point.png", [Reference point at the centre of gravity of an end face])

#screenshot("shared-coordinate-system.png", [A coordinate system created in the Results, shown in the FE Model])

=== Coordinate systems <coordinate-systems>

A coordinate system is given in a base system and either stays fixed in space or moves with a
selection of the model.

#fields(
  [Type], [#ui("Rectangular") or #ui("Cylindrical"). For a cylindrical system the z axis is the
    cylinder axis and the directions are r, θ and z.],
  [Base system], [The system the definition is given in: #ui("Global") or another coordinate
    system of the features, for example a system on a hub and one turned against it by a few
    degrees. Coordinates and turns are taken along the x, y and z axes of the base, also when the
    base is cylindrical. A system changes with its base; a base cannot be given in the system
    itself. Choosing another base keeps the system where it is and gives it anew in the new base.
    Deleting a base does the same for the systems given in it.],
  [Define by], [#ui("Three points"): #ui("Origin"), #ui("Point on x axis") and #ui("Point in xy
    plane"), typed in or picked with #ui("..."). #ui("Offset and rotation"): the base moved by
    #ui("Offset") and turned about its x axis, then about the turned y axis, then about the
    turned z axis, in degrees, as Ansys does. Switching between the two keeps the system in
    place.],
  [Motion], [#ui("Fixed in space") keeps the system where it is while the model deforms.
    #ui("Moves with selection") binds it to faces, edges or nodes picked in the 3D view, whole
    parts or a set: under large deformations it moves and turns with the rigid motion that fits
    the motion of their nodes best, for example with the faces of a bore, a flange or the end of
    a lever arm. A system given in a moving base moves with the base.],
)

#screenshot("moving-coordinate-system.png", [A coordinate system bound to the free end of a
  cantilever, given in a base system by offset and rotation])

A moving system is marked everywhere it appears: the tree shows it with an orange turning arrow
and #ui("(moving)") after its name, the 3D view with an orange ring about its origin, and the
lists that offer coordinate systems (loads, remote points, field outputs) name it with
#ui("(moving)"). In the Results the system is drawn where the shown deformation takes it.

What moves with the system under large deformations (#ui("Nlgeom")):

- A #ui("Force"), #ui("Moment"), #ui("Excitation") or #ui("Remote Force") in a moving system turns
  with it, as a follower load turning with the system's selection rather than with its own
  region (@load-directions).
- A #ui("Coordinate System Transformation") of the results (@derived-fields) takes the
  directions the system has in each increment. Displacements in a moving system are taken
  relative to it: the field less the rigid motion of the system, so that a rotating part shows
  only how it deforms.
- Displacement boundary conditions and remote displacements keep the directions the system has
  in the undeformed model; the model check warns about them in steps with #ui("Nlgeom").
- Materials, joints, planes, moving loads and reference points use the system as it lies in the
  undeformed model. Material axes turn with the material under #ui("Nlgeom") anyway.

Without #ui("Nlgeom") a moving system acts like a fixed one in the solver.
