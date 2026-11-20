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

Every mesh belongs to an object of the geometry. A part read from an input file has no CAD
geometry, so it becomes an *imported mesh*: it is listed in the Geometry workspace with the mesh
icon and drawn there as its outline (its feature edges, or its bounding box when it has none).
It can be renamed, suppressed and deleted like a CAD part, but not meshed again. Several imported
meshes can be merged into one; merging an imported mesh with CAD parts is refused.

#screenshot("imported-mesh-outline.png", [An imported beam mesh (EALL, selected) drawn as its outline
  beside the parts of a STEP file, one of them suppressed])

#screenshot("imported-mesh-outline.png", [The imported mesh EALL in the Geometry workspace, drawn
  as its outline next to the parts of a STEP file])

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

The context menu of a geometry part has #ui("Create Mesh"), #ui("Clear Mesh"), #ui("Suppress") (@suppressed-parts),
the entries of the compound parts (@compound-parts), #ui("Properties ..."),
#ui("Hide")/#ui("Show"), #ui("Isolate") and #ui("Delete").
A mesh always belongs to its geometry part: deleting the part, in the Geometry or in the FE
Model workspace, deletes its mesh too, and renaming it renames the mesh. The items on the deleted
part alone, such as a contact found with it or a load on it, are deleted with it. #ui("Clear Mesh") deletes
only the mesh and keeps the part. A part without a mesh makes the mesh out of date, unless it is
suppressed. #ui("Edit") > #ui("Undo") brings a deleted part back with its mesh.

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

The #ui("Modify") group of the #ui("Geometry") tab changes the geometry of the analysis, one
tool for each operation of the CAD Operation block (@cad-operation-block): #ui("Fillet") rounds
and #ui("Chamfer") bevels picked edges, #ui("Push/Pull") moves planar faces (@push-pull),
#ui("Defeature") removes fillets and chamfers (@defeature), #ui("Split by Plane") splits parts
along a plane (@split-by-plane) and #ui("Split by Offset Circle") splits a face along a circle
offset from a circular edge, a ring around a hole or along the rim of a disc
(@split-offset-circle). Each opens the editor of the CAD Operation block with that operation
chosen; the other operations of the block are offered there too. A split by plane starts from
the YZ plane through the middle of the geometry, the other tools from a size of 3 mm.

#screenshot("geometry-modify-tools.png", [The Modify group of the Geometry tab: a tool for each
  CAD operation])

#ui("OK") adds the operation to #ui("Operations") in the Geometry tree, named like
#ui("Split by Plane-1"). The operations there are a pipeline from the geometry as imported or
linked to the geometry the analysis meshes, in the order of the tree, as if CAD Operation blocks
stood before the analysis in the project (@cad-operation-block): each acts on what the ones before it
made. The parts keep their names, the mesh setup and the picks of the FE model follow the faces
and edges by where they lie, and the geometry has to be meshed again.

- Double-click an operation to change it: its editor opens on the geometry the operations before
  it make, and with #ui("OK") the operations act anew.
- #ui("Delete") in its context menu, or the Delete key, removes it; the others act anew on the
  geometry as imported or linked, and what it cut is whole again.
- Geometry that comes from a link, such as a File block or another analysis, takes the
  operations too: when the link brings new shapes, for example after the CAD file changed, the
  operations act on them again. Such an analysis meshes the parts the link brings itself rather
  than taking the link's mesh of them.
- An operation that fails on new shapes, for example when a picked face is gone, is skipped;
  the output says why. Change it or delete it.

#ui("Edit") > #ui("Undo") takes back adding, changing or deleting an operation (@undo).

#ui("Split by Plane") cuts only the parts selected in the tree when it is opened, compound parts
selected standing for their members; with nothing selected it cuts every part the plane runs
through. With parts selected, the editor shows only them, starts from the YZ
plane through their middle and draws the outline of the plane over them; the other parts stay as
they are, even those the plane runs through or that lie wholly on a side that is not kept.
The plane carries the manipulator of the section view (@section-view): drag the arrow to shift
it, an arc to tilt it about a global axis in it (@split-by-plane).

#screenshot("geometry-split-selected.png", [Split by Plane with SOLID-3 selected in the tree: the
  editor shows only that cylinder, not highlighted, with the manipulator on the plane])

#screenshot("geometry-operations.png", [A split by plane under Operations in the Geometry tree:
  double-click to change it, Delete to take it back])

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

- Shells of a compound are meshed together if they share their meshing parameters and
  Gmsh algorithms and none is thickened: an edge that two of them have, lying exactly on each
  other, is meshed once, and the shells share its nodes as one shell would. Selections made
  on such an edge or its vertices still work. Shells with a gap between them keep their own
  nodes there and are tied as below. Dissolving the compound, or taking a shell out of it,
  gives each shell nodes of its own again.
- Each other part of a compound is meshed on its own, with its own mesh setup, as before. In the FE
  Model workspace the compound is *one* part: sections, loads, boundary conditions and the
  contact detection see one part.
- Where the pieces touch, the input file ties them, as a bonded contact region does
  (@contact-regions), shells edge to edge: the touching faces are found when the input file is
  written, with the tolerance of a contact detection on the mesh, and written as `*TIE`. The pieces are tied where
  they are: a gap between them within the tolerance, such as the play of a pin in its hole, is
  bridged, not closed, so the nodes stay in place and no element is pulled inside out. Only the
  nodes lying on the other piece are tied: where one piece reaches beyond the other, its nodes
  beside the common area stay free. Two pieces with a third between them, such as the blocks on
  both sides of a thin sheet, are tied to that piece and not to each other through it. These
  ties do not appear in the tree; problems the model check finds on them are shown on the
  compound part.
- Changing a compound joins the existing mesh anew at once, without meshing again; the
  meshes of its parts are then out of date (like after a change of their mesh setup), and
  #ui("Solve") or the project's #ui("Update") meshes them again, shells together. Until
  then the parts keep their own nodes and are tied. Sections and
  other items on a part that joins a compound move to the compound; when a compound is dissolved,
  they apply to each of its parts.
- Renaming the compound part in the FE Model workspace (#ui("Properties ...")) renames the
  compound too, so the name stays when it is meshed again.
- A contact detection never creates contact regions between pieces of the same compound.
- Compound parts can be made of linked parts too (@links): they join meshes and change no
  shape, so they are part of the analysis' own setup like the mesh setup. They stay when the
  source changes, as long as their parts are still in the geometry; a part that is gone or
  renamed there drops out of the compound. When the link brings the parts meshed, they are
  read-only, because the mesh comes already joined.

#screenshot("linked-compound-part.png", [Two linked parts joined into a compound part in an analysis whose geometry comes from a file block])

#note[Solids, and shells meshed on their own, keep their own nodes on the faces they share,
as with a bonded contact. Where
the meshes of the pieces are very different in size, the tie interpolates between them; the
stresses right at the cut are then less accurate than inside a piece.]

#screenshot("compound-part.png", [A bracket cut into three solids for the mesher, joined into one compound part])

=== Suppressed parts <suppressed-parts>

A suppressed part takes part in nothing of the analysis, as if it were not there, until it
is unsuppressed. This helps with an assembly where only some parts are to be
meshed or changed now, for example a screw that is meshed later or not at all.

- #ui("Suppress") in the context menu of a part suppresses it, of several selected parts all of
  them; on a compound part it suppresses its members. #ui("Unsuppress") takes it back, as does a
  click on the no-entry sign in front of the part, and #ui("Unsuppress All") in the menu of
  #ui("Parts") unsuppresses every part.
- In the tree a suppressed part is gray with the no-entry sign instead of its icon, like a
  deactivated item of the FE model; a compound whose members are all suppressed too. They are
  listed apart under #ui("Suppressed") at the end of #ui("Parts"), which folds them away; its
  menu has #ui("Unsuppress All"). In the
  Geometry workspace it is drawn as its edges only; the FE Model and the results do not show it.
  A click or a selection box in the 3D view goes through it: only the tree selects it, to
  unsuppress it. Its check box still shows or hides it.
- #ui("Mesh All Parts") (also in the mesh setup dialog), #ui("Create Mesh") and the meshing
  before a solve leave suppressed parts out. A mesh the part already has is kept but left out of
  the solve and the input file export, and it does not count as out of date. Suppressing is the
  only way to leave a part out without deleting it.
- The solver gets nothing of a suppressed part: neither its mesh nor the items on it alone. A
  contact found with the part before it was suppressed, a boundary condition or a load only on
  it stay in the model for when it is unsuppressed, but are left out of the solve and the input
  file. An item also on other parts keeps those.
- A contact detection never searches a suppressed part, also not one it names, and part filters
  and part groups do not match it.
- #ui("Suppress") is also in the context menu of a part in the FE Model workspace and works on
  imported meshes too.
- #ui("Split by Plane") never cuts a suppressed part, also not when it is selected.
  #ui("Defeature") finds no fillets on it, and edges or faces of a suppressed part cannot be
  picked for a fillet, chamfer, push/pull, hole ring or defeaturing.
- Suppressing is part of the mesh setup of the analysis: #ui("Edit") > #ui("Undo") takes it
  back, it is saved with the project, and an analysis that takes its geometry from a link keeps
  its own suppressed parts.

#screenshot("suppressed-parts.png", [SCHRAUBE suppressed: gray with the no-entry sign in the
  tree and drawn as its edges only, Mesh All Parts meshes the other parts])

#screenshot("suppressed-parts-fe-model.png", [SOLID-3 suppressed after meshing: listed under
  Suppressed, not drawn in the FE Model, and the contact detection finds only the contact of the
  other two parts])

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
