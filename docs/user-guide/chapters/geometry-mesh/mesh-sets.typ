#import "../../template.typ": *

== Node sets, element sets and surfaces <mesh-sets>

You do not need sets to set up a model: every dialog with a region takes a selection directly.
Named sets are useful when several items share the same region, or when you want to keep a
selection under a name. They are listed under #ui("Mesh") in the #ui("FE Model") tree, together
with the sets and surfaces of an imported `.inp` mesh.

#ui("Node Set"), #ui("Element Set") and #ui("Surface") on the #ui("Mesh") tab of the FE Model
workspace define a new one. Double-click a set, or choose
#ui("Edit ...") in its context menu, to change it; #ui("Delete") or the #key("Delete") key
removes it. Selecting a set in the tree highlights it in the 3D view.

#screenshot("node-set.png", [Node set dialog with its selection])

#fields(
  [Name], [The name of the set, as it is written to the input file. Letters, digits, `_`, `-`
    and `.` are allowed. An element set cannot take the name of a part.],
  [Region], [#ui("Selection in the 3D view") or #ui("Parts") for node and element sets;
    surfaces take a selection of element faces (edges in 2D models). Click #ui("...") and pick
    in the 3D view as for any region.],
)

/ Node Set: the picked nodes, or all nodes of the picked faces, edges or parts.
/ Element Set: the elements of the picked faces or of whole parts.
/ Surface: the picked element faces.

The set remembers the selection, not only the node and element numbers: a set picked on the
geometry is found again on a new mesh after meshing again. A set whose selection is no longer on
the mesh (picked nodes of an old mesh) is still listed, but stays empty until you pick it
again; items using it show a missing reference.

Sets are used through the region combo box of the dialogs (#ui("Node Set"), #ui("Element Set"),
#ui("Surface")). Renaming a set renames it in every item that uses it. The input file contains
the sets as `*NSET`, `*ELSET` and `*SURFACE` with your names.

Editing a node set or an element surface of an imported mesh turns it into a set of your own
with the same nodes or faces, which you can then pick anew.

#limitation[Element sets and node-based surfaces of an imported mesh cannot be edited, only
deleted. Element sets are picked through element faces, so in 2D models only the elements at the
outline can be picked one by one; use #ui("Parts") for whole parts.]
