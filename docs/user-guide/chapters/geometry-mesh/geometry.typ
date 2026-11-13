#import "../../template.typ": *

= Geometry and Mesh <geometry>

== Importing geometry <geometry-import>

#menu("Geometry", "Import Geometry") or the ribbon tile #ui("Import Geometry") reads STEP
(`.step`, `.stp`), IGES (`.iges`, `.igs`) and BREP (`.brep`) files. Several files can be
selected at once. Gmsh with OpenCASCADE reads the files, so Gmsh must be available
(@settings).

- The geometry is read into the open analysis, in its model space and length unit (both were
  chosen when the analysis was created, @new-analysis).
- If the model already has geometry or a mesh, the imported parts are *added*. Existing parts keep
  their names, mesh settings and selections.
- STEP and IGES files are converted to the length unit of the model. BREP files have no unit and
  are read as they are.
- Default element sizes are derived from the size of the geometry: maximum 5 % and minimum 0.1 %
  of the bounding box diagonal.
- Parts keep the names they have in the CAD program, as STEP files from Onshape, SolidWorks or
  FreeCAD carry them, for solids and for surface parts alike. The names are made fit for
  CalculiX: upper case, letters, digits, `-` and `_`, German umlauts written out (`Stütze`
  becomes `STUETZE`). A name that is already taken gets a number appended (`OBEN-2`). Parts
  without a name in the file are called `SOLID-1`, `SHELL-1` or `LINE-1`.
- Curves that lie outside any face become line parts. The pieces of a composite curve, a
  straight piece that runs on into an arc for example, stay one line part: pieces that meet end
  to end without a kink, and where no third curve ends, belong together, and the line is meshed
  as a whole with a shared node at the joint. Curves that meet at a corner, as the members of a
  frame do, stay parts of their own.

#screenshot("geometry.png", [Imported geometry on the Geometry tab])

#screenshot("step-part-names.png", [Parts of a STEP file with the names from the CAD program])

=== Importing a mesh <mesh-import>

The same dialog also takes input files (`.inp`). Their mesh is added to the model as *mesh
parts*, for example a mesh written by #menu("File", "Export", "Deformed Mesh (.inp) ...")
(@export-deformed) to use a buckling mode as imperfection. Select the files in the file dialog
(the filter #ui("Input file") shows only those) and the parts appear in the FE Model workspace.

- A model is never replaced. If it already has parts, the imported nodes and elements are
  numbered after the highest numbers it has, so nothing that refers to the existing parts changes.
- A part, set or surface whose name is already taken gets a number appended: the second `PART-1`
  becomes `PART-1-2`.
- Only the mesh is read: nodes, elements, sets and surfaces. Materials, steps and loads of the
  file are skipped and listed in the output window.
- The mesh must fit the model space of the analysis: a mesh of plane stress elements goes into a
  2D plane stress analysis, solids into a 3D one. A 2D mesh cannot be added to a 3D analysis or
  the other way round.

#screenshot("mesh-import.png", [A deformed mesh imported next to the original part])

=== Parts <geometry-parts>

Every solid becomes a part. Faces that do not belong to a solid become shell parts, edges that do
not belong to a face become line parts (for beams and trusses). Faces that share an edge form one
shell part, as a surface split into several faces in the CAD program. Part names are taken from the
CAD file where it has names, otherwise they are `SOLID-1`, `SHELL-1`, `LINE-1` and so on.

The context menu of a geometry part has #ui("Create Mesh"), the entries of the compound parts
(@compound-parts), #ui("Properties ..."), #ui("Hide")/#ui("Show") and #ui("Delete").
Deleting a geometry part keeps a mesh that was already
created from it.

=== Split faces

It is common to split faces in the CAD program, for example a ring on a shaft where a bearing
sits, so that a boundary condition can be applied to exactly that area. Nodalis keeps such
faces: the mesh follows their borders, and in the FE Model workspace their outline is drawn
like any other edge although the surface does not bend there. Pick them with the geometry based
selection (@selection) like any other face.

#screenshot("split-face.png", [A ring split off the shaft in the CAD program, outlined and picked on the mesh])

The same holds for surfaces: a plate with areas split out of it for supports stays one shell part.
Its faces are meshed together, so the nodes on their borders are shared and no tie or contact is
needed between them, and each face can still be picked on its own.

#screenshot("split-shell.png", [A plate with discs split out of it: one part, one common mesh, one disc picked])

=== Modifying the geometry <geometry-modify>

The #ui("Modify") group of the #ui("Geometry") tab changes the imported geometry of the analysis
right away: #ui("Defeature") removes fillets and chamfers (@defeature), #ui("Split by Plane")
splits parts along a plane (@split-by-plane) and #ui("Split around Hole") splits the face around
a hole into a ring (@split-around-hole). Each opens the editor of the CAD Operation block with
that operation chosen; the other operations of the block are offered there too. #ui("OK")
replaces the geometry by the result, as if it had been imported so: the parts keep their names,
the mesh setup and the picks of the FE model follow the faces and edges by where they lie, and
the geometry has to be meshed again. A split by plane starts from the YZ plane through the middle
of the geometry, the other tools from a size of 3 mm.

Unlike the CAD Operation block, the tools keep no history and cannot be undone: change the
geometry with them once, for example to remove cosmetic fillets before meshing. Use the block in
the project when the change is to stay editable, to be varied by a parameter study or to follow a
new version of the CAD file. An analysis whose geometry comes from a link refuses the tools,
since the link would bring its geometry back; put a CAD Operation block between the two instead.

#screenshot("geometry-defeature-tool.png", [Defeature from the Modify group of the Geometry tab:
  the fillets and chamfers up to 3 mm are found on the imported bracket])

=== Compound parts <compound-parts>

A CAD model is often cut into several solids so that the mesher copes with it, for example a
block cut into pieces that can each be meshed with hexahedra. A *compound part* joins such pieces
into one part of the FE model again:

1. Select the parts in the Geometry tree (#key("Ctrl") or #key("Shift") adds to the selection).
2. Choose #ui("Create Compound Part") in their context menu.

The compound part appears in the Geometry tree like a folder, named `COMPOUND-1` and so on, with
the parts it joins under it. The context menu of a part offers #ui("Add to Compound") with a
submenu of the compound parts, and #ui("Remove from Compound") for a part that is in one. The
context menu of the compound part has #ui("Create Mesh"), #ui("Rename..."), which asks for the new name in a small dialog (Enter
confirms, Escape cancels), and #ui("Dissolve"), which makes its parts parts of their own again.

- Each part of a compound is meshed on its own, with its own mesh setup, as before. In the FE
  Model workspace the compound is *one* part: sections, loads, boundary conditions and the
  contact detection see one part.
- Where the pieces touch, the input file ties them, as a bonded contact region does
  (@contact-regions), shells edge to edge: the touching faces are found when the input file is
  written, with the tolerance of a contact detection on the mesh, and written as `*TIE`. The pieces are tied where
  they are: a gap between them within the tolerance, such as the play of a pin in its hole, is
  bridged, not closed, so the nodes stay in place and no element is pulled inside out. These
  ties do not appear in the tree; problems the model check finds on them are shown on the
  compound part.
- Changing a compound joins the existing mesh anew at once, without meshing again. Sections and
  other items on a part that joins a compound move to the compound; when a compound is dissolved,
  they apply to each of its parts.
- Renaming the compound part in the FE Model workspace (#ui("Properties ...")) renames the
  compound too, so the name stays when it is meshed again.
- A contact detection never creates contact regions between pieces of the same compound.

#note[The pieces keep their own nodes on the faces they share, as with a bonded contact. Where
the meshes of the pieces are very different in size, the tie interpolates between them; the
stresses right at the cut are then less accurate than inside a piece.]

#screenshot("compound-part.png", [A bracket cut into three solids for the mesher, joined into one compound part])

=== Geometry for 2D models

In a 2D model the geometry must consist of faces only and lie in the plane z = 0. For an
axisymmetric model it must lie at x ≥ 0, because the y axis is the axis of rotation. Otherwise
the import is refused with a message. Faces whose normal points in −z direction are flipped
when they are meshed.

Many CAD programs draw a sketch or a face on a plane away from the origin. A file whose geometry
lies flat in a plane parallel to the x-y plane, at a constant z, is therefore moved into the plane
z = 0 when it is imported, and the #ui("Output") names the file and the height it came from. This
is checked file by file, also when files are added to a geometry. Geometry that is tilted or has
any thickness in z stays where it was drawn.

#screenshot("flat-geometry-moved.png", [A plate drawn at z = 25 mm, moved into the x-y plane on import])
