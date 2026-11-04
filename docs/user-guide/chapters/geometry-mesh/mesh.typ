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
  [Solid (3D models)], [Tetrahedra `C3D4` (linear) or `C3D10` (quadratic)],
  [Face (3D models)], [Shells: triangles `S3`, `S6` or quad-dominated with `S4`, `S8`; the
    thickness comes from a shell section (@shell-section)],
  [Face (2D models)], [Triangles, or quad-dominated with quadrilaterals; written as `CPS`,
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
with #ui("Local Mesh Size"), #ui("Extrude Mesh") or #ui("Tetra. Gmsh") on the #ui("Geometry")
tab of the ribbon.

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
  [Quad-dominated surface mesh], [Creates mainly quadrilaterals on faces: plane elements of 2D
    models and shells of 3D models. Solids are not affected: their tetrahedra need triangles on
    the faces. The option is greyed out when the parameters reach only solids.],
  [Hexahedra where possible], [Meshes every solid that is a straight extrusion of one of its
    faces, as an Extrude Mesh item with its defaults would (@extrude-mesh), or a body of
    revolution with hexahedra (@revolve-mesh); the other solids keep tetrahedra (default
    off).],
)

#screenshot("quad-dominated-solids.png", [Quad-dominated surface mesh greyed out for a geometry of
  solids only])

=== Mesh setup items

/ Meshing Parameters: the fields above for selected parts. If several items name the same part,
  the last one counts. Create it with #ui("Mesh Parameters") in the #ui("Mesh Setup") group of
  the #ui("Geometry") ribbon, next to #ui("Default Parameters"), which sets the defaults of all
  parts.
/ Local Mesh Size: an element size on selected CAD faces and edges. Click them in the 3D view;
  #key("Shift") adds, #key("Ctrl") removes. The #ui("Selection") window next to the dialog
  (@selection-window) offers the geometry based modes: #ui("Faces and edges") takes the face
  under the cursor or the edge near it, #ui("Part") all faces of a part, #ui("Edge angle")
  the clicked edge and the edges that continue it, #ui("Surface angle") the clicked face and
  the neighbouring faces that meet it at less than the angle, such as the pieces of a split
  cylinder. A box takes the faces inside it. The default size is a quarter of the maximum
  element size.
/ Tetrahedral Gmsh: chooses the Gmsh algorithms for selected parts. #ui("Surface algorithm"):
  Frontal-Delaunay (default), Delaunay, MeshAdapt or Automatic. #ui("Volume algorithm"):
  Delaunay (default), Frontal or HXT.
/ Extrude Mesh: hexahedra and wedges for parts that are extrusions of a face (@extrude-mesh).

#screenshot("mesh-parameters-item.png", [The #ui("Mesh Parameters") tile in the Geometry ribbon and the
  dialog that creates a Meshing Parameters item for picked parts])

#screenshot("mesh-setup-item.png", [Creating a mesh setup item])

#screenshot("local-mesh-size-surface-angle.png", [A Local Mesh Size on the three faces of a
  split cylinder, picked with one click in the #ui("Surface angle") mode])

#ui("OK") stores the item. #ui("OK & Remesh") stores it and meshes again the parts it names.
A Local Mesh Size item is picked on faces and edges, so it meshes all parts again.

=== Extrude mesh <extrude-mesh>

A part that is a straight extrusion of a face, such as a plate with holes, a profile or a
gear blank, can be meshed with hexahedra instead of tetrahedra. The #ui("Extrude Mesh") item
meshes the source face with Gmsh, mostly with quadrilaterals, and sweeps that face mesh through
the part in layers: every quadrilateral becomes a hexahedron (`C3D8`, or `C3D20` with second
order), every triangle a wedge (`C3D6`, `C3D15`). The size of the face mesh comes from the part's
meshing parameters and local mesh sizes, as for tetrahedra.

#fields(
  [Source faces], [#ui("Automatic") finds a face of each selected part that the part is an
    extrusion of. #ui("Selected faces") uses the faces you click at one end of the parts; the
    dialog shows which parts they belong to. A source may consist of several faces; the
    #ui("Surface angle") mode of the selection window takes the faces that continue the
    clicked one with one click.],
  [Surface algorithm], [The Gmsh algorithm for the source face. #ui("Frontal-Delaunay for
    quads") (default) gives triangles that join well to quadrilaterals.],
  [Recombine algorithm], [How triangles are joined to quadrilaterals: #ui("Blossom") (default)
    or #ui("Simple") leave a few triangles, which become wedges. #ui("Simple full-quad") and
    #ui("Blossom full-quad") give quadrilaterals only, so the part gets hexahedra only; they
    split the mesh once more, which halves the element size. #ui("None") keeps the triangles.],
  [Layers defined by], [The element size in the extrude direction. #ui("Scale factor"): layers
    as thick as the part's max. element size times the factor. #ui("Number of elements"): this
    many layers of the same thickness. #ui("Multi-layered"): relative layer sizes such as
    `0.2; 0.6; 0.2;` and the elements per layer such as `2; 4; 2;`.],
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
automatic search for every part they cover. A part where no source face is found is meshed with
tetrahedra without an error; a #ui("Tetrahedral Gmsh") item naming a part keeps it tetrahedral.

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
