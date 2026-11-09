#import "../../template.typ": *

== Features <features>

Under #ui("Features") in the tree you define reference points, coordinate systems and planes.
They are helpers for other functions (section views, plane results, paths, result
transformations) and are not written to the input file.

The FE Model and the Results workspace share the features of an analysis: both show the same
list, and a feature created, edited, renamed or deleted in one of them is changed in the other as
well. A coordinate system defined in the Results, e.g. to look at the stresses of a hub in
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
/ Coordinate System: #ui("Rectangular") or #ui("Cylindrical"), defined by #ui("Origin"),
  #ui("Point on x-axis") and #ui("Point in xy-plane"). For a cylindrical system the z axis is the
  cylinder axis and the directions are r, θ and z.
/ Plane: by #ui("Coordinate system") (a plane XY, YZ or XZ of a system with an offset), by
  #ui("Three points") (normal by the right-hand rule) or by #ui("Point and normal"). A plane
  built on reference points follows them when they change.

#screenshot("reference-point.png", [Reference point at the centre of gravity of an end face])

#screenshot("shared-coordinate-system.png", [A coordinate system created in the Results, shown in the FE Model])
