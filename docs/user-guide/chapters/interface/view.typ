#import "../../template.typ": *

== The 3D view <view>

=== Mouse and keyboard

#table(
  columns: (auto, 1fr),
  stroke: (x, y) => if y == 0 { (bottom: 0.7pt) } else { (bottom: 0.3pt + luma(200)) },
  inset: (x: 5pt, y: 4pt),
  table.header([*Input*], [*Action*]),
  [Middle button drag], [Rotate about the point of the model under the cursor, marked by a
    red dot while turning; over empty space about the centre of the view],
  [#key("Shift") + middle button drag], [Pan],
  [Mouse wheel], [Zoom about the cursor, with #key("Ctrl") in finer steps],
  [#key("Ctrl") + middle button drag], [Zoom],
  [Double-click (left)], [Zoom to fit],
  [Left click on a part], [Select the part in the tree; #key("Ctrl") toggles, #key("Shift")
    adds. In the FE Model workspace the tree marks what acts on the part (@acting).
    A click into empty space clears the selection.],
  [Left drag], [Box selection, only while a dialog is picking (@selection)],
  [Right click], [Context menu: the menu of the part under the cursor followed by the view
    commands],
)

Rotating keeps the point you grabbed where it is, so you can look at a small detail at the end
of a long structure from all sides without it leaving the screen.

#screenshot("rotate-pivot.png", [Turning about the end of a beam: the red dot marks the point
  the view rotates about])

Keyboard shortcuts of the main window: #key("Ctrl+N") new, #key("Ctrl+O") open,
#key("Ctrl+S") save, #key("Ctrl+Shift+S") save as, #key("F5") run analysis, #key("Del")
delete the selected tree item, #key("Space") activate or deactivate it. In dialogs
#key("Enter") means OK and #key("Esc") Cancel; a confirmation takes both as Cancel
(@tool-windows).

=== Floating bar <floating-bar>

At the bottom of the 3D view floats a bar with the view commands needed most:
#ui("Zoom to Fit"), #ui("Standard Views"), #ui("Vertical View"), #ui("Edges Only"),
#ui("Mesh Edges"), #ui("Section View"), #ui("Exploded View") and #ui("Screenshot"). They are
the same as on the #ui("View") tab of the ribbon; a pressed tile shows what is on. The bar is
left out of screenshots.

#figure(
  image("../../images/view-floating-bar.png", width: 45%),
  caption: [The floating bar of the 3D view.],
)

=== Views

The projection is orthographic. Y points up in the front view.

- #ui("Front"), #ui("Back"), #ui("Left"), #ui("Right"), #ui("Top"), #ui("Bottom") and
  #ui("Isometric") are in #ui("Standard Views") on the #ui("View") tab of the ribbon.
- #ui("Vertical View") rolls the view about the viewing direction so that the global axis
  closest to screen-up points exactly up.
- #menu("View", "Standard Views", "View Normal to X/Y/Z") looks along an axis onto the plane normal to it.
- #menu("View", "Standard Views", "Isometric, X/Y/Z Up") shows the isometric view with the chosen axis
  pointing up, for models built with Z up.
- The *coordinate triad* in the lower right corner can be clicked: a click on an axis tip (or on
  its grey negative half) looks along that axis.

A 2D model always opens in the front view. The legend sits in the upper left corner, the info
block in the upper right corner, the scale bar above the floating bar. Legend, info block, scale bar and
min/max labels can be dragged to another position in the view.

=== Showing and hiding parts

Each part in the tree has a visibility check box. #ui("Hide") and #ui("Show") in the context
menu of a part act on all selected parts. Hidden parts cannot be picked and are ignored by box
selection and by #ui("Zoom to Fit").

#ui("Hide All") and #ui("Show All") in the context menu of the #ui("Parts") node switch all parts
of the tree at once, in the geometry, FE model and results trees alike.

#figure(
  image("../../images/parts-hide-all.png", width: 60%),
  caption: [Context menu of the #ui("Parts") node.],
)

=== Screenshots <screenshot>

The #ui("Screenshot") tile on the #ui("View") tab of the ribbon captures the 3D view including legend and labels:
#ui("Copy to Clipboard") or #ui("Save As ...") (PNG).

=== Section view <section-view>

#menu("View", "Section View") or #ui("Section View") on the ribbon cuts the model with a plane. Volume
elements show their cut faces with mesh edges and, in the Results workspace, with the result
colours.

#screenshot("section-view.png", [Section view of a result])

#fields(
  [Base plane], [XY, YZ or XZ at the coordinate given in #ui("Position").],
  [Plane feature], [A plane defined under #ui("Features") (@features), shifted by
    #ui("Distance") along its normal.],
  [Point and normal], [A point on the plane and its normal. #ui("From selection \"...\"") takes
    the centre of the clicked nodes, edge or face (for example the centre of a hole).
    #ui("From two points") takes the normal from two clicked points, the #ui("X")/#ui("Y")/#ui("Z")
    buttons set it to an axis.],
  [Position slider], [Moves the plane through the model.],
  [Flip], [Shows the other half.],
  [Lighten cut surfaces], [Draws the cut faces lighter (default on).],
)

In the 3D view the plane has a manipulator: drag the arrow to move the plane, drag the arcs to
tilt it. Changes are shown at once, as in every tool window (@tool-windows): #ui("Close")
keeps the section, #ui("Off") removes it and closes the window. #menu("View", "Section Off")
removes the section as well.

=== Exploded view <exploded-view>

#menu("View", "Exploded View") pulls the parts of an assembly apart for display. The mesh
itself is not changed; parts that share nodes move together. A left click on the ribbon tile
switches the exploded view on and off, a right click opens the settings.

#screenshot("exploded-view.png", [Exploded view dialog])

#fields(
  [Explosion method], [#ui("Disassembly"): each part moves the way it would be taken off, along
    the normal of its mating face or the axis of its bore. #ui("Assembly center"): the parts move
    away from the centre of the assembly. #ui("Center point"): the parts move away from the
    given point (#ui("Model Center") fills in the centre).],
  [Contact tolerance], [Disassembly only. Faces closer than this count as touching. 0 means
    1/1000 of the assembly diagonal.],
  [Sequential], [Disassembly only. Takes the assembly apart level by level.],
  [Direction], [XYZ, X, Y, Z, XY, XZ or YZ. Movements along the other axes are suppressed.],
  [Magnification], [How far the parts move at scale factor 1 (default 2).],
  [Scale factor], [0 = assembled, 1 = fully exploded (default 0.5). Also set with the
    #ui("Assembled - Exploded") slider.],
)

The parts follow the parameters at once. #ui("Close") keeps the exploded view, #ui("Off")
shows the model assembled and closes the window.
