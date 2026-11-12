#import "../../template.typ": *

== Selecting regions <selection>

Boundary conditions, loads, constraints, contacts, mesh settings and many result tools act on a
_region_ of the model. You define it by clicking in the 3D view while the item's dialog is open.

=== Region field

Each dialog with a region has a #ui("Region") field. Its combo box chooses where the region
comes from; the choices depend on the item:

/ Selection in the 3D view: picked nodes, element faces, edges, parts or CAD faces. The field shows
  what is selected ("12 element faces") and has a #ui("Clear Selection") button.
/ Parts: whole parts, picked in the 3D view.
/ Node Set, Element Set, Surface: sets and surfaces of an imported `.inp` mesh, or ones you
  defined yourself (@mesh-sets).

The #ui("...") button next to a field makes it the target of clicks in the 3D view; it is shown
pressed while it is active. Dialogs with two regions (master and slave) use one #ui("...")
button per region to switch between them.

Next to #ui("...") a check mark button takes over the selection: it closes the #ui("Selection")
window and clicks in the 3D view no longer change the region. Click #ui("...") again to pick on.
The window stays open when the dialog opens for the first time. All dialogs with a region field
behave this way, for parts as well as for picked nodes, faces and edges.

#screenshot("region-confirm.png", [A region with the confirm button next to #ui("..."), while the selection window is open])

=== Selection window <selection-window>

While a dialog picks, the #ui("Selection") window opens next to it and decides what a click
selects. Drag it by its title bar to another place; it opens there from then on.

#screenshot("selection.png", [Picking faces for a load with the selection window])

/ Geometry based: #ui("Faces, edges and points") selects the CAD face under the cursor, split
  faces included (@geometry-import), or on a mesh without geometry the smooth patch of faces;
  near a feature edge or corner, within a few pixels, it selects that. #ui("Part") selects whole parts;
  for a region of nodes (an initial velocity or temperature, a node set) these are all nodes of the
  part, those inside a solid too.
  #ui("Edge angle") and #ui("Surface angle") grow the selection across edges or faces as long as
  the angle between neighbours stays below the given value (default 30°). On a mesh generated
  from geometry, #ui("Edge angle") and #ui("Surface angle") select and highlight whole CAD edges
  and faces, like #ui("Faces, edges and points"), so the selection survives remeshing. The
  mesh based modes of the same names select the nodes or element faces they grow over.
  #ui("Surface angle") does not stop at the border of a part: where the outline edges of two
  visible parts, or of two bodies of a compound part (@compound-parts), lie on each other, it
  grows on into the neighbour, whether their meshes share nodes or not. One click thus takes
  the whole cut face of a part split into many pieces, e.g. for a symmetry boundary condition.
  Hidden parts are left out; hide a neighbour to keep the selection on one part.
/ Mesh based (#ui("More")): #ui("Node"), #ui("Element"), #ui("Edge"), #ui("Surface"),
  #ui("Part"), #ui("Edge angle"), #ui("Surface angle") and #ui("ID"). #ui("ID") takes a list such
  as `1, 5, 10-20` with #ui("Add") and #ui("Remove"); for face and edge regions the numbers are
  element numbers.


#screenshot("edge-angle.png", [Edge angle on the rim of a hole selects the CAD edge, shown as "1 edge", not its nodes])
#screenshot("surface-angle.png", [Surface angle on a shaft selects the three cylindrical faces])
#screenshot("surface-angle-across-parts.png", [A block cut into twelve parts with meshes that
  do not match: one click with #ui("Surface angle") selects its front face on all six parts])

Modes that do not fit the region are disabled, for example #ui("Node") for a pressure load.
On the Geometry tab, for mesh setup items such as a Local Mesh Size, the window offers only the
geometry based modes, as the geometry has no mesh yet; #ui("All") and #ui("Invert") act on the
faces of the visible parts.
#ui("Undo") takes back the last pick, #ui("Clear") empties the selection, #ui("All") and
#ui("Invert") act on the visible items.

#note[On a mesh created from CAD geometry, geometry-based picks are stored as CAD faces, edges
and vertices. They survive remeshing. Picks of single nodes or element faces refer to the mesh
and have to be repeated after remeshing; the output pane says how many selections are affected.]

Picks on geometry also follow the geometry when its shapes change: a STEP file that a file
block reads again (@file-block) or a geometry link from another analysis brings new shapes,
and Gmsh numbers their faces and edges anew. Each face, edge and vertex is recorded with what
it looks like (its type, centre, extent and size), and a pick takes the entity of the new
shapes that is unchanged or clearly the nearest of the same type, for example the top face of
a plate that became thicker. Local mesh sizes follow in the same way. A pick that cannot be
found, because its face went away or two faces fit it equally well, is kept as
not found: the item turns red with a warning sign, its selection reads e.g.
#ui("2 faces (1 not found)"), and the output pane says how many there are; select those faces
again.

#screenshot("selection-not-found.png", [The STEP file of the plate changed to a thicker plate
  without its hole: the boundary conditions on the top and bottom face follow, the one on the
  hole is not found])

=== Modifier keys and box selection

- A plain click replaces the selection, #key("Shift") adds, #key("Ctrl") removes,
  #key("Shift")+#key("Ctrl") keeps only the intersection. A plain click into empty space
  clears the selection.
- Dragging with the left button draws a box. Dragged from left to right (blue) it takes what lies
  completely inside; dragged from right to left (green) it also takes what crosses the box.
- When the mouse rests for a moment, what a click would select is previewed in orange.

Selected regions are drawn red. Line parts such as beams, trusses and CAD edges have no faces
to colour: once picked they are drawn as wide red lines, and the preview shows their lines in
orange.

#screenshot("selection-lines.png", [The beam RIEGEL picked as the region of a section])
