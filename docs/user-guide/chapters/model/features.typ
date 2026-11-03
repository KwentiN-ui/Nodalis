#import "../../template.typ": *

== Features <features>

Under #ui("Features") in the tree you define reference points, coordinate systems and planes.
They are helpers for other functions (section views, plane results, paths, result
transformations) and are not written to the input file. The results have their own copy of the
features, which can be edited in the Results workspace.

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
/ Coordinate System: #ui("Rectangular") or #ui("Cylindrical"), defined by #ui("Origin"),
  #ui("Point on x-axis") and #ui("Point in xy-plane"). For a cylindrical system the z axis is the
  cylinder axis and the directions are r, θ and z.
/ Plane: by #ui("Coordinate system") (a plane XY, YZ or XZ of a system with an offset), by
  #ui("Three points") (normal by the right-hand rule) or by #ui("Point and normal"). A plane
  built on reference points follows them when they change.

#screenshot("reference-point.png", [Reference point at the centre of gravity of an end face])
