#import "../../template.typ": *

== Meshing <meshing>

Nodalis meshes each geometry part separately with Gmsh. Parts do not share nodes; connect them
with ties, contacts or node ties (@constraints).

- #menu("Geometry", "Mesh All Parts") meshes every part.
- #ui("Create Mesh") in the context menu of a part meshes only this part. Its old mesh is
  replaced; the other parts keep their node and element numbers.

The output pane lists each part with its element count and size range. Afterwards Nodalis
switches to the FE Model workspace.

=== Cancelling the meshing <cancel-meshing>

Gmsh runs in a separate process, so a meshing that takes too long, for example because the
element size is too small, can be cancelled at any time. While it runs, the status bar shows
#ui("Meshing ...") with a #ui("Cancel") button; #menu("Geometry", "Cancel") and the
#ui("Stop") tile of the ribbon do the same. The mesher stops at once and the parts keep
the mesh they had before. During #ui("Update Project") the #ui("Stop") buttons of the
Solution Monitor and #ui("Stop") in the Project workspace cancel a meshing in the same way.

#screenshot("meshing-cancel.png", [The status bar while meshing, with the Cancel button])

=== Element types

#table(
  columns: (auto, 1fr),
  stroke: (x, y) => if y == 0 { (bottom: 0.7pt) } else { (bottom: 0.3pt + luma(200)) },
  inset: (x: 5pt, y: 4pt),
  table.header([*Part*], [*Elements*]),
  [Solid (3D models)], [Hexahedra `C3D8` (linear) or `C3D20` (quadratic) where the solid can be
    swept (@stacked-sweep); otherwise tetrahedra `C3D4` or `C3D10`],
  [Face (3D models)], [Shells: quadrilaterals `S4`, `S8`, or triangles `S3`, `S6` if the
    quadrilaterals are switched off; the thickness comes from a shell section (@shell-section)],
  [Face (2D models)], [Quadrilaterals, or triangles if they are switched off; written as `CPS`,
    `CPE` or `CAX` elements depending on the model space],
  [Line (3D models)], [`B31` or `B32` lines; the section decides whether they are written as
    beams or trusses (@sections)],
)

Solids and lines can only be meshed in 3D models. Faces are meshed in both: in a 3D model they
become shells, which get their thickness from a #ui("Shell") section; a solid
section on shells is reported as an invalid section.

#screenshot("shell-mesh-3d.png", [A disk imported into a 3D model and meshed with `S6` shells])

=== Changing the element type <element-type>

A part in the FE Model workspace can be given another element type with the
same nodes: double-click the part and choose it in the #ui("Element type") box, then #ui("OK").
A part with several element types has one box per type.

#screenshot("part-element-type.png", [Choosing the element type of a part])

#table(
  columns: (auto, 1fr),
  stroke: (x, y) => if y == 0 { (bottom: 0.7pt) } else { (bottom: 0.3pt + luma(200)) },
  inset: (x: 5pt, y: 4pt),
  table.header([*Elements*], [*Element types*]),
  [Hexahedra], [`C3D8`, `C3D8R` (reduced integration), `C3D8I` (incompatible modes);
    `C3D20`, `C3D20R`],
  [Tetrahedra], [`C3D4`; `C3D10`, `C3D10T`],
  [Quadrilaterals], [`S4`, `S4R`, the membranes `M3D4`, `M3D4R`; `S8`, `S8R` in 3D models, the
    same as `CPS`, `CPE` or `CAX` elements in 2D models],
  [Triangles], [`S3` or the membrane `M3D3`; `S6` in 3D models],
)

Membranes need a membrane section (@membrane-section). Quadratic triangles and wedges have
one type only, which is shown as text, as do linear triangles in 2D models. Lines get their type from
their section (@sections). The choice is kept with the part: meshing the part again gives the
new elements the same type, and in a 2D model a new model space keeps the reduced integration
(`CPS8R` becomes `CAX8R`).

=== Meshing errors

If Gmsh fails, the output pane shows "Meshing failed: ..." with the message of Gmsh. The faces,
edges and points the message names are marked red in the Geometry workspace, so you can see
where the problem is. The marking disappears with the next meshing.

== Mesh setup <mesh-setup>

The mesh is controlled by the #ui("Mesh Setup") items in the Geometry workspace. Create them
with #ui("Local Mesh Size"), #ui("Extrude Mesh"), #ui("Thicken Shell") or #ui("Tetra. Gmsh") on
the #ui("Geometry") tab of the ribbon.

=== Default mesh parameters

#menu("Geometry", "Default Parameters") sets the parameters for every part that is not
covered by a #ui("Meshing Parameters") item. #ui("Mesh All Parts") in this dialog applies the
values and meshes at once.

#screenshot("mesh-parameters.png", [Default mesh parameters])

#fields(
  [Max. element size], [Largest element edge length.],
  [Min. element size], [Smallest element edge length.],
  [Elements per curvature radius], [Refines the mesh on curved faces: about this many elements
    per radius (default 2). 0 switches curvature refinement off.],
  [Second order], [Quadratic elements with midside nodes (default on).],
  [Midside nodes on geometry], [Places midside nodes on the curved geometry instead of on the
    straight edge between the corner nodes (only with second order).],
  [Optimize mesh (Netgen)], [Improves the element quality after meshing (default on).],
  [Quadrilaterals on faces], [Meshes faces with quadrilaterals (default on): plane elements of
    2D models and shells of 3D models; off gives triangles. Solids are not affected: their
    tetrahedra need triangles on the faces. The option is greyed out when the parameters reach
    only solids. See @quad-faces.],
  [Hexahedra where possible], [Meshes every solid that can be swept with hexahedra (default
    on): a stack of extrusions along one direction (@stacked-sweep), a straight extrusion of
    one of its faces (@extrude-mesh) or a body of revolution (@revolve-mesh). The other solids
    get tetrahedra, with a warning that says why; off gives tetrahedra everywhere.],
)

#screenshot("quad-dominated-solids.png", [Quadrilaterals on faces greyed out for a geometry of
  solids only])

=== Quadrilaterals on faces <quad-faces>

Faces are meshed with Gmsh's quasi-structured algorithm. It lays the quadrilaterals in a grid
that follows the outline and the holes, so that nearly every inner node joins four elements,
and it leaves no triangles. The element sizes are those of the meshing parameters and the
local mesh sizes; next to a rounded outline the grid may come out finer than asked for.

#screenshot("quad-faces.png", [A flange meshed with the quasi-structured algorithm])

Nodalis falls back to the Frontal-Delaunay algorithm, whose triangles are joined to
quadrilaterals, and says so in the output pane:

- if a face would get more than about 20 000 quadrilaterals, where the quasi-structured
  algorithm takes minutes;
- if the quasi-structured algorithm fails or leaves a badly distorted quadrilateral and the
  other algorithm does better.

A #ui("Tetrahedral Gmsh") item that names the part chooses the algorithm instead: its
triangles are then joined to quadrilaterals, which is faster and less regular.

=== Mesh setup items

/ Meshing Parameters: the fields above for selected parts. If several items name the same part,
  the last one counts. Create it with #ui("Mesh Parameters") in the #ui("Mesh Setup") group of
  the #ui("Geometry") ribbon, next to #ui("Default Parameters"), which sets the defaults of all
  parts.
/ Local Mesh Size: an element size on selected CAD faces and edges. Click them in the 3D view;
  #key("Shift") adds, #key("Ctrl") removes. The #ui("Selection") window next to the dialog
  (@selection-window) offers the geometry based modes: #ui("Faces and edges") takes the face
  under the cursor or the edge near it, #ui("Part") all faces of a part, #ui("Edge angle")
  the clicked edge and the edges that continue it, also in touching parts, #ui("Surface angle")
  the clicked face and the neighbouring faces that meet it at less than the angle, such as the
  pieces of a split cylinder, on into the visible parts that touch it along an edge. A box
  takes the faces inside it. The default size is a quarter of the maximum element size. #ui("Depth") keeps
  the size inside the solid up to that distance below the selected faces (and around the
  selected edges), so that a layer of that thickness gets elements of the size, such as the
  walls of an infill section (@infill-section); beyond it the elements grow to the size of the
  part. 0 (the default) refines the faces alone. In a swept hexahedral mesh a depth below a cap
  refines the layers along the sweep, a depth below a wall the face mesh next to it.
/ Tetrahedral Gmsh: chooses the Gmsh algorithms for selected parts. #ui("Surface algorithm"):
  Frontal-Delaunay (default), Delaunay, MeshAdapt or Automatic; on shells and 2D models it
  replaces the quasi-structured algorithm (@quad-faces). #ui("Volume algorithm"):
  Delaunay (default), Frontal or HXT.
/ Extrude Mesh: hexahedra and wedges for parts that are extrusions of a face or stacks of
  extrusions (@extrude-mesh), with the layers and the surface algorithm of your choice.
/ Thicken Shell Mesh: solid elements for shell parts, the face mesh offset through a thickness
  (@thicken-shell-mesh).

#screenshot("mesh-parameters-item.png", [The #ui("Mesh Parameters") tile in the Geometry ribbon and the
  dialog that creates a Meshing Parameters item for picked parts])

#screenshot("mesh-setup-item.png", [Creating a mesh setup item])

#screenshot("local-mesh-size-surface-angle.png", [A Local Mesh Size on the three faces of a
  split cylinder, picked with one click in the #ui("Surface angle") mode])

#ui("OK") stores the item. #ui("OK & Remesh") stores it and meshes again the parts it names.
A Local Mesh Size item is picked on faces and edges: #ui("OK & Remesh") meshes again only the parts that hold them,
and the other parts keep their mesh, still up to date.

=== Extrude mesh <extrude-mesh>

A part that is a straight extrusion of a face, such as a plate with holes, a profile or a
gear blank, can be meshed with hexahedra instead of tetrahedra. The #ui("Extrude Mesh") item
meshes the source face with Gmsh, mostly with quadrilaterals, and sweeps that face mesh through
the part in layers: every quadrilateral becomes a hexahedron (`C3D8`, or `C3D20` with second
order), every triangle a wedge (`C3D6`, `C3D15`). The size of the face mesh comes from the part's
meshing parameters and local mesh sizes, as for tetrahedra.

#fields(
  [Source faces], [#ui("Automatic") finds how each selected part is swept: as a stack of
    extrusions (@stacked-sweep) or, failing that, as the extrusion of one of its faces.
    #ui("Selected faces") uses the faces you click at one end of the parts; the
    dialog shows which parts they belong to. A source may consist of several faces; the
    #ui("Surface angle") mode of the selection window takes the faces that continue the
    clicked one with one click.],
  [Surface algorithm], [The Gmsh algorithm for the source face. #ui("Quasi-structured quad")
    (default) gives a regular grid of quadrilaterals only (@quad-faces), so the part gets
    hexahedra only. #ui("Frontal-Delaunay for quads") gives triangles that join well to
    quadrilaterals and is faster on very fine faces.],
  [Recombine algorithm], [Greyed out for #ui("Quasi-structured quad"), which joins no
    triangles. How the other algorithms' triangles are joined to quadrilaterals: #ui("Blossom") (default)
    or #ui("Simple") leave a few triangles, which become wedges. #ui("Simple full-quad") and
    #ui("Blossom full-quad") give quadrilaterals only, so the part gets hexahedra only; they
    split the mesh once more, which halves the element size. #ui("None") keeps the triangles.],
  [Layers defined by], [The element size in the extrude direction. #ui("Scale factor"): layers
    as thick as the part's max. element size times the factor. #ui("Number of elements"): this
    many layers of the same thickness. #ui("Multi-layered"): relative layer sizes such as
    `0.2; 0.6; 0.2;` and the elements per layer such as `2; 4; 2;`. In a stack of extrusions
    the layers are counted over the whole height; every step gets a layer boundary and at
    least one layer.],
)

#screenshot("extrude-mesh.png", [Extrude Mesh item with the top face of a plate as source,
  and the hexahedra swept from it in four layers])

Before meshing, nodalis checks that the part really is an extrusion of the source: every
vertex at the boundary of the source has a straight edge of the same length and direction to
its copy at the other end, and the part's volume is the source's area times the height. If not,
meshing fails with a message naming the part; mesh it with tetrahedra or select the faces at one
end. The element type can be changed afterwards in the part's properties (@element-type), for
example to `C3D8I`, which bends without the locking of `C3D8`. A later #ui("Tetrahedral Gmsh")
item for the same part meshes it with tetrahedra again.

Without an item, #ui("Hexahedra where possible") in the meshing parameters runs the same
automatic search for every part they cover. A part that cannot be swept is meshed with
tetrahedra, and a warning names what stands against the sweep; a #ui("Tetrahedral Gmsh") item
naming a part keeps it tetrahedral without a warning.

=== Stacks of extrusions <stacked-sweep>

Most machined parts are no extrusion of one face, but several extrusions along one direction on
top of each other: a stepped shaft, a plate with a boss, a pocket or counterbored holes, a
flange with a hub. nodalis recognises them by their faces: every face is either planar and
normal to one common direction, an end or a step, or runs along that direction, such as the
wall of a hole or the outside of a boss. Such a part is meshed with hexahedra only, without
any input (@fig-stacked-sweep):

+ All ends and steps are laid over each other in one plane and cut at each other, into cells:
  under a counterbored hole, for example, a disk for the hole and a ring for the counterbore.
+ The cells are meshed together with quadrilaterals, with the surface algorithm of the
  Extrude Mesh item or the quasi-structured algorithm (@quad-faces).
+ Each cell's mesh is swept through the heights where the part has material above the cell.
  Every step is a layer boundary, so the hexahedra of neighbouring cells share their nodes.

#figure(
  image("../../images/stacked-sweep.png", width: 80%),
  caption: [A plate with a boss, a bore and four counterbored holes, meshed with hexahedra by
    the default parameters.],
) <fig-stacked-sweep>

The layers between two steps are as thick as the max. element size at most, or as an Extrude
Mesh item with automatic source faces sets them. A Local Mesh Size on an end or a step refines
the cells below it, on a wall or an edge the quadrilaterals along it and the layers through
its height. Faces, edges and vertices of the geometry stay selectable on the mesh.

A part is no stack as soon as one face is neither an end nor a wall: a chamfer, a fillet
between a step and a wall, a hole across the direction or a conical seat. Such parts are tried
as a plain extrusion and as a body of revolution and get tetrahedra otherwise. Splitting the
geometry (@split-by-plane) so that the feature lies in a part of its own keeps hexahedra in
the rest.

=== Thicken shell mesh <thicken-shell-mesh>

A thin part drawn as faces, such as a sheet metal bracket, is meshed with shell elements. When
the model needs solid elements instead, for example for contact on both sides of the sheet or
for a material that shells do not take, the #ui("Thicken Shell Mesh") item turns the shell into
a solid. Create it with #ui("Thicken Shell") in the #ui("Mesh Setup") group and click the shell
parts. The faces are meshed as usual, after the part's meshing parameters and local mesh sizes,
and the face mesh is then offset along its normals in layers: every quadrilateral becomes a
hexahedron (`C3D8`, or `C3D20` with second order), every triangle a wedge (`C3D6`, `C3D15`).
#ui("Quad-dominated surface mesh") in the meshing parameters gives mostly hexahedra.

#fields(
  [Parts], [The shell parts to thicken. #ui("OK") refuses solids and lines.],
  [Thickness], [The thickness of the solid (default a tenth of the max. element size).],
  [Number of layers], [Layers of elements through the thickness (default 2). Linear elements
    need several layers to bend correctly.],
  [Offset], [Where the solid lies relative to the shell's faces, along the face normals: 0
    (default) centres it on the faces, half the thickness puts the faces on one side of the
    solid, minus half the thickness on the other.],
)

#screenshot("thicken-shell-mesh.png", [A bent sheet with a hole thickened to 2 mm in three
  layers of hexahedra])

All faces of a part are turned to one side first, so that the solid is one piece. Where faces
meet at an angle, as at a bend or a kink, the nodes move so that the solid keeps its thickness
on both faces. A thickness so large that the elements fold over, at a sharp inner corner or a
tight curvature, is refused with a message giving the place. Faces where more than two faces
meet at one edge cannot be thickened.

Selections made on the geometry stay valid: a face of the shell takes the side of the solid
nearest to the shell's faces, at offset 0 the side its normals point to; an edge or a vertex
takes all nodes through the thickness, so that a support on an edge holds the whole side of the
solid. After meshing, the part is a solid: give it a solid section (@sections) instead of a
shell section.

=== Bodies of revolution <revolve-mesh>

#ui("Hexahedra where possible") also finds parts that are a section turned about an axis:
rings, flanges, hubs and segments of them. Their edges are circles about one common axis and
profile curves in planes through it. nodalis meshes the section with quadrilaterals and turns
that mesh about the axis in steps of about the max element size at the largest radius, at least
16 steps for a full turn. A segment is turned from one of its end faces to the other; a closed
ring gets a section cut through it and ends where it started. The result is hexahedra, with
wedges where the section mesh has triangles (@fig-revolve-mesh). Faces, edges and vertices of
the geometry stay selectable on the mesh.

#figure(
  image("../../images/revolve-mesh.png", width: 70%),
  caption: [A 270° segment of a ring with a rounded section, meshed with #ui("Hexahedra where
    possible").],
) <fig-revolve-mesh>

A part counts as a body of revolution only if its volume is that of its section turned about
the axis. Parts that reach the axis, such as solid shafts, keep tetrahedra: their elements at the
axis would have to be pyramids, which CalculiX lacks. A part that is both an extrusion and a body
of revolution, a plain ring for example, is meshed as an extrusion.

== Using an existing mesh

A CalculiX mesh can be imported with #menu("Geometry", "Import Geometry") (@mesh-import). Its node sets, element
sets and surfaces appear under #ui("Mesh") in the #ui("FE Model") tree and can be used as
regions. Such a mesh has no geometry, so regions are picked on the mesh.
