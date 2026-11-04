#import "../../template.typ": *

== Mesh tools <mesh-tools>

The mesh tools change an existing mesh without meshing again: they move or mirror parts, join
parts, merge nodes that lie on top of each other and renumber the mesh. They work on the
FE Model workspace, on meshed geometry as well as on imported meshes.

=== Transform parts

#ui("Transform ...") in the context menu of one or more parts under #ui("Mesh") moves their
nodes.

#screenshot("transform-mesh.png", [Transform mesh parts])

#fields(
  [Kind], [#ui("Translate") by #ui("dX"), #ui("dY"), #ui("dZ"); #ui("Rotate") by #ui("Angle")
    (in degrees) about an axis through the point #ui("X"), #ui("Y"), #ui("Z") with the
    direction #ui("Axis"); #ui("Mirror") at the plane through the point with the #ui("Normal");
    #ui("Scale") about a centre with #ui("Factor X"), #ui("Factor Y"), #ui("Factor Z").],
)

Mirrored elements are turned inside out again, so they stay valid; the surfaces, sets and
regions of the model follow the elements. Only the mesh moves: meshing the geometry again
restores its position. In 2D models the rotation axis is the z axis and the transformations have
two components.

=== Merge parts

Select several parts under #ui("Mesh") (#key("Ctrl") adds to the selection) and choose
#ui("Merge") in their context menu. The elements of the other parts join the first selected
part, which keeps its name; element sets of the same name are joined too, and sections, loads
and other items that named a merged part now name the kept part. The nodes are not merged:
use #ui("Merge coincident nodes") afterwards where the parts touch.

=== Merge coincident nodes

#menu("Mesh", "Merge Nodes") joins nodes of all parts that lie within the
#ui("Tolerance") (default 0.1 mm) of each other into one node, the one with the lowest number.
This connects parts that were meshed separately but share their boundary nodes exactly, for
example halves of an imported mesh. The output pane reports how many nodes were merged.

#screenshot("merge-nodes.png", [Merge coincident nodes])

#note[Parts meshed from geometry in Nodalis do not share nodes and their nodes rarely coincide
exactly; connect them with ties or contacts (@constraints) instead.]

=== Renumber nodes and elements

#menu("Mesh", "Renumber") numbers all nodes and elements from the
#ui("First node number") and #ui("First element number") on (default 1), in their current
order, without gaps. Sets, surfaces and the regions of the model follow the new numbers.
