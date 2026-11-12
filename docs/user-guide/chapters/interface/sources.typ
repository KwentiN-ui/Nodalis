#import "../../template.typ": *

== Source blocks <source-blocks>

Besides analyses, the project schematic holds *source blocks*: blocks that only give data to
analyses through links. They have no steps and are not solved, and #ui("Update Project")
passes them by; the mesh export block at the end of this section takes data instead of giving
it. Add one with its tile on the #ui("Sources"), #ui("Modify"), #ui("Parameters") or #ui("Evaluate") tab or by double-clicking or right-clicking the background of the schematic;
its header has its own tint, so that it stands apart from the analyses. Its context menu has
#ui("Edit ..."), #ui("Rename ..."), #ui("Duplicate") and #ui("Delete"); a double-click
opens its editor in the project window. Deleting a source block removes its links; the
analyses keep copies of what they took.

=== Materials <materials-block>

A materials block holds materials in the project file, so that several analyses use the same
definition: change it once in the block and every analysis that links it follows. Unlike the
material library (@materials), which belongs to you and every project you open, the block
belongs to the project and travels with it.

#screenshot("materials-block.png", [A materials block: Static takes S235, Modal takes all of
  them])

Its rows are #ui("Materials"), which gives all of its materials, and below it one row per
material; the arrow in front of #ui("Materials") hides and shows them. Drag a row to the
#ui("Setup") row of an analysis, or to its #ui("Materials") row or a material once
#ui("Setup") is open, as with links between analyses (@data-links). A link of all materials
also passes on materials added to the block later; while the block shows its materials, it is
drawn as a wire per material. In the analysis window the linked
materials carry the link sign and are read-only; #ui("Open Materials") in their dialog opens
the block's editor.

#screenshot("materials-block-editor.png", [The editor of a materials block])

#fields(
  [Shown in], [The unit system the values are shown and typed in. Like the values of an
    analysis they are stored in SI units, so each analysis shows them in its own.],
  [New ...], [Defines a new material in the dialog of the FE Model tree (@materials).],
  [Edit ...], [Edits the selected material; a double-click on it does the same.],
  [Delete], [Removes the selected material.],
  [Add from Library ...], [Opens the material library (@material-library) with the block's
    materials in place of the model's: copy materials from the library into the block, or from
    the block into the library.],
  [Import from Analysis], [Copies the materials of an analysis into the block, so that a
    project can gather them in one place afterwards. A material of the same name is replaced.],
)

#ui("OK") keeps the changes, and the analyses that link the block take them at once;
#ui("Cancel") drops them.

=== Cross-Sections <cross-sections-block>

A cross-sections block holds beam cross-sections (@cross-sections) in the project file, like a
materials block holds materials. Add it with #menu("Sources", "Cross-Sections"). Its rows
are #ui("Cross-Sections") and one row per cross-section, linked to the #ui("Setup") of an
analysis, or to its #ui("Cross-Sections") row, in the same way; the beam sections of the
analysis then pick the linked cross-sections. Its editor has #ui("Shown in"), #ui("New ..."),
#ui("Edit ..."), #ui("Delete") and #ui("Import from Analysis") as for materials; the dialog of a
cross-section is that of the FE Model tree, with the sketch and the second moments of area.

#screenshot("cross-sections-block.png", [The width a of the cross-section Girder_Rect is
  exposed in the block Profiles and set by a Parameters block; the analysis links both
  cross-sections])

Every dimension of a cross-section can be exposed with the #ui("P") button in its dialog. The
block then shows it under #ui("Exposed Parameters") with a violet pin, so that a Parameters
block (@parameters-block) or a parameter study sets it, and every analysis that links the
cross-section takes the new profile. Exposed dimensions belong to the kind of profile: after a
change of kind, expose the new dimensions again.

=== Box and Rectangle meshes <mesh-block>

A mesh block makes a structured mesh from a few numbers, without geometry and without Gmsh:
#menu("Sources", "Box Mesh") meshes a box with hexahedra for 3D analyses,
#menu("Sources", "Rectangle Mesh") a rectangle in the X-Y plane with quadrilaterals for
2D analyses. Link its #ui("Mesh") row to the #ui("Mesh") row of analyses of the same model
space; several analyses can share one mesh block.

#screenshot("mesh-block.png", [A box linked to an analysis, its parameters in the block])

Its parameters are rows of the block under #ui("Mesh"), like the inputs of a Blender node:
#ui("X"), #ui("Y"), #ui("Z") of the corner it starts at and the edge lengths #ui("Lx"),
#ui("Ly"), #ui("Lz") in the block's units, the numbers of elements #ui("nx"), #ui("ny"),
#ui("nz") as whole numbers and the #ui("Element") type. Click a field to type a value, or drag
it; a rectangle has no Z rows. Once a parameter changes, the #ui("Mesh") row shows the mesh out
of date. The violet pins on the left of the numbers take a value from a Parameters block
(@parameters-block). To make the mesh finer or coarser as a whole, put a Refine Mesh block
after it (@refine-mesh).

Its sides are node sets and surfaces at once, named after the axis and the side they lie on:
#ui("X-") at the smallest X, #ui("X+") at the largest, then #ui("Y-"), #ui("Y+") and, for a
box, #ui("Z-") and #ui("Z+"). Boundary conditions and loads pick them under #ui("Node set") or
#ui("Surface") without selecting anything. The elements form the part #ui("BOX") or
#ui("RECTANGLE"), which a section takes.

The sides, edges and corners are also faces, edges and points of geometry. A selection in
the 3D view with #ui("Faces, edges and points") picks a whole side of the box, and it stays on
that side when the mesh is made anew with other divisions, other lengths or another
refinement, as on imported geometry. A selection of single nodes or element faces holds only
for the mesh it was made on.

#screenshot("box-face-remeshed.png", [A boundary condition picked on the end face of a box with
  12 x 2 x 2 elements, after the box was made anew with 24 x 4 x 2 elements])

A double-click on the block's name opens all its parameters in a dialog, with what the rows
leave out: the model space of a rectangle and the units the lengths are shown in.

#screenshot("mesh-block-editor.png", [The parameters of a box])

#fields(
  [Model space], [Rectangle only: plane stress, plane strain or axisymmetric. It is fixed while
    analyses link the mesh, which have the same model space. An axisymmetric rectangle lies at
    X >= 0, as X is the radius.],
  [Shown in], [The unit system the lengths are shown and typed in; they are stored in SI
    units.],
  [Origin], [The corner at the smallest coordinates.],
  [Edge lengths], [The size along each axis.],
  [Elements along], [The number of elements along each axis.],
  [Element type], [Box: C3D8, C3D8R, C3D8I, C3D20 or C3D20R. Rectangle: #ui("Quadratic")
    gives 8-node instead of 4-node elements, #ui("Reduced integration") the R types; the family
    (CPS, CPE or CAX) follows from the model space. The #ui("Element") row of a rectangle
    lists its four types.],
)

The line below the fields counts the elements and names the sides. #ui("OK") keeps the
parameters, and the #ui("Mesh") row of the block shows that its mesh is out of date.
#ui("Update Project") makes it anew before it solves (@update-project), and so does
#ui("Generate Mesh") in the block's context menu; the analyses that link it take the new mesh,
and their solutions are out of date.

#screenshot("mesh-block-result.png", [The box held at X- and pressed on Z+, solved])

=== Refine Mesh <refine-mesh>

A Refine Mesh block makes the mesh linked to it again, finer or coarser by a factor, and gives
the new mesh on. Add it with #menu("Modify", "Refine Mesh"), drag the
#ui("Mesh") of a mesh block or of an analysis with imported geometry to the pin on the left of
its #ui("Mesh") row, and drag the pin on the right of that row to the #ui("Mesh") row of the
analyses that are to use the refined mesh. The mesh it refines stays as it is.

#screenshot("refine-mesh.png", [A box refined by 1.5 for the analysis Fine: the Factor takes the
  dashed wire of a parameter study, which varies it])

#fields(
  [Mesh], [Left pin: the mesh to refine. Right pin: the refined mesh, with its state.],
  [Factor], [How much finer the mesh is: 2 twice as fine, 0.5 half as fine, 1 as it is. Type
    or drag it in place, or drop the value of a Parameters block or an input of a parameter study on its pin
    (@parameter-study).],
)

What the factor does depends on where the mesh comes from:

- From a mesh block (Box, Rectangle): the numbers of elements along every edge are multiplied
  by the factor and rounded, so a box of 10 x 2 x 2 elements refined by 1.5 has 15 x 3 x 3. The
  block makes this mesh itself: once the factor or the mesh block changes, its #ui("Mesh") row
  is out of date and #ui("Update Project") or #ui("Generate Mesh") in its context menu makes it
  again.
- From an analysis with imported geometry: it gives the geometry with its mesh setup on, every
  element size divided by the factor (the max. and min. element size of the meshing
  parameters, those of Meshing Parameters items and the local mesh sizes). The analysis that
  takes it meshes the geometry with these sizes when it is solved; the sizes as set in the mesh
  setup stay. Its mesh can therefore not go to a Mesh Export block: export the mesh of the
  analysis that meshed it.

Refine Mesh blocks can follow each other; their factors multiply. A mesh read from an input
file has no parameters or geometry to mesh it again with, and the link is refused.

=== Deformed Mesh <deformed-mesh-block>

A Deformed Mesh block gives the mesh of an analysis with every node moved by its displacement in
one mode or increment of a step, times a factor. Its main use is an imperfection: the buckling
mode of a buckle step, scaled to a small amplitude, as the starting geometry of a geometrically
nonlinear static analysis, which then shows how the imperfect structure deflects and fails below
the ideal buckling load. Add it with #menu("Project", "Add Mesh", "Deformed Mesh"), drag the pin of
a step under #ui("Results") of an analysis to its #ui("Step Result") row, and drag the pin on the
right of its #ui("Mesh") row to the #ui("Mesh") row of the analysis that starts from the deformed
mesh. The pin is there before anything is solved, so the whole chain can be linked first.

#screenshot("deformed-mesh-block.png", [The first buckling mode of the analysis Buckle, at half
  its size, is the mesh of the analysis Nonlinear, which takes its materials and sections from
  Buckle too])

#fields(
  [Step Result], [The results of a step whose displacements move the nodes. Only the results of
    an analysis can be linked.],
  [Mode], [The mode of a buckle or frequency step, or the increment of any other step, counted
    from 1.],
  [Factor], [What the displacements are multiplied by; negative values turn the mode around.
    Mode shapes are normalized to a largest displacement of 1 (@mode-shapes), so the factor is
    the amplitude of the imperfection in the length unit of the analysis.],
  [Mesh], [The deformed mesh, with its state. Its row shows the largest displacement of a node
    times the factor, the amplitude of the imperfection, in the units of the analysis.],
)

#ui("Mode") and #ui("Factor") take value links like other numbers (@parameters-block), so a
parameter study can vary the amplitude or the mode of the imperfection (@parameter-study).

The deformed mesh keeps the parts, node sets, element sets, surfaces and the faces of the
geometry of the analysis it comes from, but not the geometry itself. What the analysis after it
defines on that mesh, such as a section on a part, a boundary condition on a node set or a load
on a picked face, therefore finds its region on the deformed mesh too. Link the materials and
sections, or the whole setup, of the source analysis to it as usual (@links), and give it a
static step with #ui("Nonlinear geometry (Nlgeom)") on.

#ui("Update Project") solves the analysis the results come from first, then reads the
displacements and makes the mesh, and only then solves the analyses that take it. The block makes
its mesh again whenever the results, the mesh of the source analysis, the mode or the factor
changed; until then its #ui("Mesh") row and the analyses after it are out of date. The mesh is
kept in the project file. #menu("File", "Export Deformed Mesh (.inp) ...") in an analysis window
still writes the mesh as displayed to a file (@export-deformed); a Mesh Export block after the
Deformed Mesh block does the same from the schematic (@mesh-export).

=== File <file-block>

A file block reads a file from outside the project and gives what it holds to analyses: the
results of a CalculiX run (`.frd`), the mesh of a CalculiX input file (`.inp`) or geometry
(STEP, IGES, BREP, STL). Add it with #menu("Sources", "File") or from the picker; a
dialog asks for the file at once. Its rows depend on the file:

- results: #ui("Results") with a row per step of the file, whose pins link like the steps of
  an analysis, for example to the #ui("Global results") row of a submodel (@submodel);
- an input file: #ui("Mesh"), to link to the #ui("Mesh") row of analyses;
- geometry: #ui("Geometry"), to link to the #ui("Geometry") row of analyses.

==== Single parts of an assembly <file-block-parts>

A STEP file often holds a whole assembly while an analysis needs only some of its parts. When
the geometry has more than one part, the #ui("Geometry") row opens with its arrow, like the
#ui("Setup") of an analysis, and lists the parts by their names in the file, each with a pin
of its own. Linking the #ui("Geometry") row gives the whole file; linking a part gives only
that part. Link several parts to the #ui("Geometry") row of the same analysis and it takes
them together, the other parts left out. A part link replaces a link of the whole geometry
of the same file and the other way round; geometry of other blocks stays (@geometry-assembly).

#screenshot("file-block-parts.png", [Two of the three parts of an assembly go to the analysis])

The geometry without the other parts is made in the background, in the same way as
#ui("Delete") of geometry parts (@geometry-parts): the parts keep their names, and the
analysis meshes and sets up its parts as with any geometry it takes. When the file changes,
the parts are taken again by their names; a part the file no longer has is reported in the
output pane, and the analysis keeps the geometry it had.

==== Assemblies from several links <geometry-assembly>

The #ui("Geometry") and #ui("Mesh") rows of an analysis have the tall pin of a collection
(@links): they take any number of wires and put what comes through them together into one
assembly. Link the parts of a file that stay as they are and the output of a
#ui("CAD Operation") that changes one of them, or the geometry of several files, and the
analysis meshes all parts together, each a part of its own as in an assembly read from one
file.

#screenshot("geometry-assembly.png", [The file gives its parts, the CAD operation the part it
  split: the analysis takes the split part in place of the original])

A part that reaches the analysis changed, through a #ui("CAD Operation"), a #ui("Transform")
(@transform-block) or an analysis it went through, takes the place of the same part of the file linked unchanged: link the whole
file and the operation on one of its parts, and the part is there once, changed. The output
pane says which part is replaced ("Assembly takes SOLID-2 changed from CAD Operation in place
of SOLID-2 of File").

#screenshot("geometry-assembly-parts.png", [The geometry of the analysis: SOLID-1 as in the
  file, SOLID-2 split in two pieces, tied as a compound part])
 Two parts of the same name from different blocks are kept
both, the second named with "-2" appended, which the output pane reports as well. Putting the
geometries together is done in the background; the analysis is out of date until it is done.

Several meshes linked to the #ui("Mesh") row are put together in the same way, each with its
own nodes and elements, numbered after those before it. Their parts, sets and surfaces keep
their names unless an earlier mesh has one of the name. The geometries of the meshes are put
together too, so that picks on the geometry find their nodes. An analysis takes either
geometry or meshes: a geometry link replaces its mesh links and the other way round.

#screenshot("file-block.png", [The results of a CalculiX run made elsewhere drive a submodel])

The #ui("File") row holds the path: type it, or double-click the block or use
#ui("Choose File ...") in its context menu. A relative path starts at the folder of the project
file. The block reads the file in the background and reads it again whenever it changes on
disk; #ui("Read Again") in the context menu reads it at once. The symbol of the first row shows
the state: a green tick once the file is read, a dash while no file is chosen or it is missing,
and an error when it could not be read, with the reason in the tooltip and the output pane.
A linked analysis is out of date when the file changes, and #ui("Update Project") solves it
again with the new data. The file is never written to; deleting the block leaves it as it is.

#limitation[Results of a file serve as global results of submodels only. Eigenmodes of a
frequency step cannot be linked, as the `.frd` file does not hold them.]

=== CAD Operation <cad-operation-block>

A CAD Operation block changes geometry before it is meshed: it rounds (#ui("Fillet")) or bevels
(#ui("Chamfer")) picked edges of solids or corners of flat faces, moves picked planar faces
(#ui("Push/Pull")), splits parts and faces along a plane (#ui("Split by Plane"),
@split-by-plane), splits the face around a hole into a ring (#ui("Split around Hole"),
@split-around-hole) or removes fillets and chamfers (#ui("Defeature"), @defeature), of the
geometry linked to it and gives the changed geometry on.
Add it with #menu("Modify", "CAD Operation") or from the picker, drag the
#ui("Geometry") of a file block, of an analysis with imported geometry or of another CAD
Operation block to the pin on the left of its #ui("Geometry") row, and drag the pin on the
right of that row to the #ui("Geometry") row of the analyses that are to use the changed
geometry. Blocks in a row apply their operations one after the other.

Double-click the block to open its editor. It shows the geometry linked to it in its own 3D
view: click the edges or faces to act on, a click on a picked one removes it again. Picked
ones are drawn red, the one under the resting mouse orange. Choose the operation and type its
radius, distance or offset; #ui("Preview") applies it and shows the result, #ui("Input") goes back to picking.
#ui("OK") keeps the operation, and the result of the preview with it if nothing changed since.

#screenshot("cad-operation.png", [Two edges of a plate picked for a fillet of 1 mm])

#fields(
  [Geometry], [Left pin: the geometry to change. Right pin: the changed geometry, with its
    state.],
  [Fillet / Chamfer / Push/Pull / Split], [What it does and to how many edges, corners or
    faces, or along which plane.],
  [Radius / Distance / Offset / Width / Max Size], [The radius of the fillet, the distance of
    the chamfer from the edge on both faces, how far the faces move (outwards when positive,
    inwards when negative), where the plane of a split lies, the width of the ring around a
    hole, or up to which size a defeaturing removes fillets and chamfers. Zero
    leaves the geometry as it is (except for a split by plane, where it is the plane through
    the origin or the face), so that a parameter study can
    switch the operation off in some design points. Type or drag it in place, or drop the value of a Parameters block or an
    input of a parameter study on its pin (@parameter-study): every design point then makes the
    geometry again, and the analyses after it mesh and solve it anew.],
)

Once the input geometry or the operation changes, the #ui("Geometry") row is out of date, and
#ui("Update Project") or #ui("Update Geometry") in the context menu makes it again; the
analyses linked to it mesh the new geometry. The picked edges are found again when the input
changes, for example when the file is edited in the CAD program, by where they lie, their kind
and their size; one that is no longer found has to be picked again. When OpenCASCADE cannot
make the operation, for example because the radius is larger than the faces next to an edge,
the row shows an error with the reason in its tooltip and the output pane, and the analyses
after the block are not solved; in a parameter study the design point fails.

==== Corners of 2D geometry <cad-corners-2d>

When the linked geometry has no solids, only faces, as for a plane stress, plane strain or
axisymmetric analysis, #ui("Fillet") and #ui("Chamfer") act on the corners of the faces: the
editor looks straight onto their plane and a click picks the corner nearest to it, drawn as a red
point. The fillet replaces the corner by an arc of the radius tangent to both edges; the chamfer
cuts it with a straight line at the distance from the corner on both edges. A corner where the
edges meet smoothly, such as the end of a fillet already in the sketch, has nothing to round and
the operation fails with that reason. A corner on an edge two faces share cannot be rounded either.
#ui("Push/Pull") is not offered for 2D geometry.

#screenshot("cad-corners-2d.png", [Two corners of a flat shoulder picked (left) and rounded with
  a radius of 4 mm (right)])

==== Push/Pull <push-pull>

#ui("Push/Pull") moves planar faces along their normal, for example to make a plate thicker,
a bracket longer or a shoulder higher. It adds a prism on the face to the solid, or cuts it
away for a negative offset, so neighbouring faces at right angles grow or shrink with it and
stay one face each. A sloped or rounded neighbour does not follow: it gets a step at the old
edge, which #ui("Preview") shows. Change the main sizes first and round the edges after, with
a second CAD Operation block behind the first. Curved faces cannot be picked. Several picked
faces move one after the other, so two faces that meet both move as a whole.

#screenshot("cad-push-pull.png", [The top face of a stepped bar picked to make it thicker])

==== Split by Plane <split-by-plane>

#ui("Split by Plane") cuts every part a plane runs through, so that the faces on either side
can be picked on their own, a symmetry model keeps only one half, or a region gets its own
mesh size. Choose the #ui("Plane") in the editor:

- #ui("YZ plane"), #ui("XZ plane") or #ui("XY plane"): a plane of the global coordinate system,
  moved along its axis by the #ui("Offset"); the editor draws its outline over the geometry.
- #ui("Plane of a face"): click a planar face; the cut lies parallel to it, the #ui("Offset")
  into the part.

#ui("Keep") chooses what stays: #ui("Both sides"), or only the side along the positive axis
(#ui("+X side")) or the negative one, or for the plane of a face the part #ui("Beyond the cut") or
the slice #ui("Between face and cut"). Parts wholly on the side that goes are removed.

A solid cut in two becomes two solids, a shell two shells that share nothing along the cut: in
the tree they are parts of their own. With #ui("Keep the pieces as one part") on, the default,
they form a compound part (@compound-parts) named like the solid or shell: the FE model still
has one part, with its section and material, and the pieces are tied where they were cut, a
shell's edge to edge. Turned off, each piece is a part of its own, named after the part with -1,
-2. Line parts are cut into edges that share the cut point and stay one part. When the plane
runs through no part and both sides are kept, the block reports so.

#screenshot("cad-split-plane.png", [A plate split by the YZ plane at an offset of 70 mm: the
  preview shows the two solids, which stay one part of the FE model])

#screenshot("cad-split-shell.png", [An angle of sheet metal split by the XZ plane at 20 mm: two
  shells, parts of their own in the tree under the compound part SHELL-1])

==== Split around Hole <split-around-hole>

#ui("Split around Hole") cuts a ring around a hole into the planar face it goes through, as wide
as the #ui("Width"): the ring is then a face of its own to pick, for a bearing load, a bolt
head's contact area or a finer mesh next to the hole. Click the circular edges of the holes on
the faces to split; a hole whose edge is two half circles needs only one of them, and several
holes, also on top and bottom, are split in one go. A ring wider than the face is cut off at the
face's edges. Edges that are no circles, and holes in curved faces, are refused with the reason.
This works on solids and on shell parts alike; the parts keep their names.

#screenshot("cad-hole-ring.png", [The upper edge of a hole picked and the preview: a ring of
  8 mm around the hole is a face of its own])

==== Defeature <defeature>

#ui("Defeature") removes fillets and chamfers from solids, typically of a STEP file (also a
tool in the Geometry workspace, @geometry-modify): the faces
next to each one grow until they meet again in a sharp edge. That serves two ends. A fillet of
the file can be replaced by a #ui("Fillet") of a second CAD Operation block behind this one,
whose radius a parameter study varies (@cad-fillet-study). And small cosmetic fillets and
chamfers no longer force a fine mesh where they do not matter for the result. Choose what to
#ui("Remove") in the editor:

- #ui("Found by size"), the default: every fillet whose radius is at most the #ui("Max Size")
  and every chamfer at most that wide. The editor draws the faces found in red and says how many
  there are; they are found again whenever the input or the size changes, so a study can vary the
  size too. A fillet is a curved face that ends tangentially on the faces it joins and turns by
  less than a half circle between them, so holes, pins and the round ends of slots stay, even when
  they are as small. A chamfer is a narrow planar or conical strip meeting its neighbours at an
  angle between 10 and 80 degrees, so steps and ribs at right angles stay.
- #ui("Picked faces"): the faces clicked, for example only the one fillet to replace. Switching
  to it starts from the faces found by size; a click on a face adds or removes it. A fillet that
  runs over several faces needs all of them picked.

#ui("Preview") removes the faces and says how many. OpenCASCADE cannot remove every face, for
example a fillet whose neighbours would not meet again, or one whose hole is a spline surface.
Faces found by size that it cannot remove stay as they were, group by group of faces next to each
other, and the editor says how many stayed; picked faces are removed all or the block fails with
OpenCASCADE's reason. A size of zero leaves the geometry as it is. The parts keep their names.
Defeaturing needs solids and is not offered for 2D geometry.

#screenshot("cad-defeature.png", [Fillets up to 3.5 mm and the chamfers of the holes found on a
  bracket (left) and removed (right); the inner fillet of 5 mm stays])

==== Example: a fillet radius study <cad-fillet-study>

How much does a larger fillet at the shoulder of a stepped bar lower the stress? The project
for the answer has five blocks:

+ A file block reads the bar from a STEP file.
+ A CAD Operation block, #ui("Shoulder Fillet"), rounds the two edges in the inner corners of
  the shoulder.
+ The analysis #ui("Tension") takes its geometry from the fillet block, holds the wide end
  and pulls the narrow end by 0.04 mm.
+ A Measure block (@measure-block) reads the von Mises stresses of its step, and a Math block
  (@math-switch) takes their maximum.
+ A parameter study takes that value as its output and gives its input to the
  #ui("Radius") of the fillet block: drag the pin of the study's #ui("Inputs") row onto the pin
  of the #ui("Radius") row.

#screenshot("cad-fillet-study.png", [The study varies the fillet radius (dashed wire) and reads
  the largest stress of the analysis])

With five design points from 2 to 6 mm (#ui("Fill ...") in the study editor, @study-fill),
#ui("Run") makes the geometry again for each point, meshes it and solves it. The peak stress at
the shoulder falls from about 300 to 221 MPa, steeply up to 3 mm and less and less after
that:

#screenshot("cad-fillet-study-plot.png", [The plot of the study: the largest stress over the
  fillet radius])

=== Transform <transform-block>

A Transform block moves the geometry or the mesh linked to it as a whole and gives it on:
#ui("Translate") by a vector, #ui("Rotate") by an angle about an axis through a point,
#ui("Mirror") at a plane through a point, or #ui("Scale") about a center by a factor per axis,
the same transforms as the Transform of mesh parts under FE Model (@mesh-tools). Add it
with #menu("Modify", "Transform") or from the picker, drag the #ui("Geometry") or the
#ui("Mesh") of a file block, an analysis, a CAD Operation or another block to the pin on the
left of the row of that kind, and drag the pin on the right of the same row on to analyses.
Double-click the block to choose the transform and its numbers; the last row of the block
says what it does.

#screenshot("ribbon-modify.png", [The #ui("Modify") tab holds the blocks that change the
  geometry or the mesh linked to them])

Linked to the #ui("Geometry") row of an analysis together with the unmoved geometry
(@geometry-assembly), it places a second copy of a part, or moves one part of an assembly: a
part the transform moved takes the place of the same part of the file linked unchanged. With
meshes it works the same way: the mesh of an analysis and its moved copy linked to the
#ui("Mesh") row of another analysis give that one both, each with its own nodes.

#screenshot("transform-block.png", [The mesh of Cube taken twice by Together, once moved
  by 20 mm])

#screenshot("transform-mesh.png", [The mesh of Together: the cube and its moved copy, the
  second part named SOLID-1-2])

A geometry is moved by OpenCASCADE: translated, rotated and mirrored shapes stay exact, and
picks on them stay where they were. A moved mesh carries its sets and the geometry it was
meshed from along; its parts stay as meshed, like parts read from a file. The result is made
again whenever the input or the transform changes, and when the project is opened.

=== Parameters <parameters-block>

A Parameters block holds named numbers that other blocks take through *value links*, so that
one number drives several places: the length of two meshes, or a size that is changed in one
spot. Add it with #menu("Parameters", "Parameters"); it starts with one length.

#screenshot("value-links.png", [The length of a Parameters block gives the edge length Lx of a
  box: the row of the box is read-only, in violet, and follows the parameter])

Each parameter is a row with its value, typed or dragged in place, and a violet pin on
either side. Drag the pin on the right to the pin on the left of a number of a mesh block
(#ui("X") to #ui("nz")), to the #ui("Factor") of a Refine Mesh block or to an exposed parameter
of an analysis, a Materials block or a Cross-Sections block (@exposed-parameters). The number then takes the value of
the parameter: it is shown in violet and read-only, and when the parameter changes the blocks
after it are out of date, so #ui("Update Project") meshes and solves again. The pin on the left
takes the value of another parameter. A value link joins like with like: a length goes to a
length, a force to a force, a count to a number of elements, a factor to #ui("Factor");
otherwise the output pane says why it was refused. Right-click the pin to remove the link; the
number keeps its last value.

A double-click on the block opens its editor:

#fields(
  [Shown in], [The unit system the values are shown and typed in; they are stored in SI
    units.],
  [Name], [The name of the parameter, unique in the block. Renaming it keeps its links.],
  [Kind], [#ui("Count") (a whole number of at least one), #ui("Factor") (a number without
    unit) or a physical quantity such as #ui("Length"), #ui("Force"), #ui("Pressure") or
    #ui("Temperature").],
  [Value], [The number.],
  [Remove], [Removes the parameter and its links.],
  [Add Parameter], [Adds a length.],
)

=== Math and Switch <math-switch>

A Math block computes a number from others, like the Math node of Blender: a height that is
twice a width, a thickness that never drops below a minimum. A Switch block picks one of
several numbers by an index, for variants such as three sheet thicknesses. Add them with
#menu("Parameters", "Math") and #menu("Parameters", "Switch"), or
search them with #key("Shift+A").

#screenshot("schematic-math-switch.png", [The Math block Height doubles the width, the Switch
  block Thickness picks the thickness of the variant, and the Math block Gap subtracts one from
  the other. Gap takes millimetres from the height, so the thickness is a length too])

Their inputs are rows with a violet pin on the left: type a number, or drag a value pin to it,
from a Parameters block, from another Math or Switch block, from the #ui("Inputs") of a
parameter study (@parameter-study) or from the value of a Measure block (@measure-block), such as the force
at a node, to compute a key figure from results. The #ui("Result") row at the bottom shows the number the
block gives; drag its pin on the right to a number of its kind, to another Math or Switch
block or to the #ui("Outputs") of a study. Blocks that take the result are out of date when it
changes, as with a parameter.

#fields(
  [Operation], [Math only: #ui("Add") (A + B), #ui("Subtract") (A - B), #ui("Multiply")
    (A × B), #ui("Divide") (A / B), #ui("Minimum"), #ui("Maximum") or #ui("Clamp"), which
    keeps #ui("Value") between #ui("Min") and #ui("Max"). A product or quotient gives no
    result when dividing by zero. Under #ui("Of a series"): #ui("Max"), #ui("Min"),
    #ui("Max Abs") (the value of largest magnitude, with its sign) and #ui("Mean") make one
    number of the series at their single input #ui("Values") (@series).],
  [Index], [Switch only: which choice the block gives, counted from 1. Out of range, the block
    gives no result.],
  [Choice 1, 2, ...], [Switch only: the numbers to choose from. A wire dropped on the empty
    pin below the last choice adds a choice with that link, as in Blender (@node-groups).
    #ui("Add") adds one to type, #ui("Remove") removes the last one with its link; a switch
    holds up to 20.],
)

==== Units of the numbers

A Math or Switch block has no unit of its own: its links give it one. A sum, a difference, a
minimum, a maximum, a clamp and a switch take numbers of one kind, the kind of the first number
linked to them or of the number their result goes to; a link that brings another kind is
refused, and the output pane says why. A product and a quotient combine the units of A and B
instead, so a force over a length is a stiffness, for instance the reaction force over the
displacement that two Measure blocks read from the results of a step:

#screenshot("schematic-math-units.png", [The stiffness of a spring from a force and the travel
  it causes: 4912 N over 10 mm gives 491.2 N/mm])

The kind of the result follows from those of A and B: a length times a length is an area, a
force times a length a moment, a length over a time a velocity and a length over a length a
factor without unit. It also works the other way round: with a force at A and the result linked
to a stiffness, B is a length and its field shows millimetres. An input of a product that no
link gives, and that the other numbers do not make anything else, is a factor, so a length
times 2 stays a length. A link is refused only when its unit does not fit, such as a length at
a sum of forces, or a moment from a product whose result goes to a stiffness.

An intermediate result may have any unit. When no quantity has its unit, such as a force times
a force, the block shows it in SI base units, and it goes on to other Math blocks only. Once a
later block brings it back to a unit a quantity has, it is that quantity again and goes to any
number of it. Numbers of the same unit fit together whatever their name: a moment goes to an
energy and then takes that name.

#screenshot("schematic-math-derived.png", [A force squared has no quantity of its own and is
  shown in kg² m²/s^4; divided by the force again, it is a force of 4912 N])

Changing the operation removes links to inputs the block no longer has, and links whose kinds
no longer fit, such as the length at B when a quotient of a force and a length becomes a sum.

Every number field of a block, in a Math, Switch or Parameters block, a mesh block or an
exposed parameter, works the same way. A number with more digits than the field holds shows
as many of them as fit, and the unit always in full; hover over it for the whole value, which
the block keeps: only the display is shortened. Click it to type, and the field starts from
the whole value. A field a value link gives is read-only and its label is violet; the tooltip
names the block it comes from.

#screenshot("schematic-number-fields.png", [A of the Math block comes from the parameter
  Length: the field shows 91.000273 mm, the tooltip the whole value 91.0002734375 mm])

==== Series <series>

A value link carries either one number or a *series*: a list of numbers, like an array of
NumPy. Two Measure blocks give series (@measure-block): the values of a field at the nodes of
a region, and the whole history of a step. A series goes only to a Math or Formula block
(@formula-block) or to a Plot block (@plot-block); a link of a series to a number that takes one, such as a parameter or a
column of a study, is refused, and the output pane says so.

A Math block computes with series value by value. A single number goes with every value of the
series, so A × 2 doubles every value, and two series go pairwise, so they must have the same
number of values; otherwise the block has no result and its tooltip says why. A value without a
result, such as one divided by zero, is a gap in a plot. The #ui("Result") row then shows the
number of values and their range, such as "21 values, 0 N to 80000 N", and gives a series.
#ui("Max"), #ui("Min"), #ui("Max Abs") and #ui("Mean") turn a series back into one number: the
largest von Mises stress is a Measure block of the field values of STRESS MISES followed by a
Math block #ui("Max"). Units go with the series as with numbers.

=== Formula <formula-block>

A Formula block computes a number or a series by a formula you type, of as many inputs as it
needs: the bending stress 6 F L / (b h²) of a cantilever, the magnitude of a vector from its
components, the utilization of the largest stress of a region. Add it with
#menu("Parameters", "Formula"), or search it with #key("Shift+A").

#screenshot("schematic-formula.png", [The Formula block Bending Stress computes 6 a b / (c d²)
  from the force, the length, the width and the height of a cantilever, a stress in MPa. The
  Formula block Utilization divides it by 235 MPa, a factor without unit])

A new block has no inputs, only an empty pin. Drag the pin of a number or a series to it, and
it becomes the input #ui("a"); the next one #ui("b"), then #ui("c"), up to #ui("z"), as with
the inputs of a group (@node-groups). Every input comes from its link: remove the link (right-click
its pin) and the input goes. The inputs after it move up a letter, in the formula too, and
where the formula used the removed input it now shows `?`, so it gives no result until you
link a new input or change the formula. The #ui("Result") row gives the number or the series the formula computes; drag
its pin on to any number of its kind, another Math or Formula block, a Plot or a Preview block.

The block shows its formula read-only. Click it, or double-click the block, to edit it in its
window: the formula, the result it gives as you type, the inputs with their kinds and where
they come from, and below the list of the functions; a click on a function adds it to the
formula. #ui("OK") applies the formula.

#screenshot("formula-editor.png", [The editor of the Formula block: the formula, what it gives,
  its inputs and the functions it can call])

#fields(
  [Operators], [`+ - * /` and `^` (or `**`) for powers, with parentheses. A power binds more
    than a sign: `-a^2` is −(a²).],
  [Numbers], [`2.5`, `1e-3`, `pi`, and numbers with a unit in brackets: `5[mm]`, `210[GPa]`,
    `3[N/mm]`. A number without unit is a factor. Values are in SI units, so `a / 1[mm]` is
    the number of millimetres in the length a.],
  [Functions], [`sqrt`, `cbrt`, `abs`, `sign`, `exp`, `ln`, `log10`, `sin`, `cos`, `tan`,
    `asin`, `acos`, `atan`, `atan2(y, x)`, `floor`, `ceil`, `round`, `clamp(x, low, high)`,
    and `min`, `max`, `mean`, `sum`, `std` and `norm` of one series or of several values.],
)

*Units* go through the formula as through a Math block: the inputs take the kinds of the
numbers linked to them, and the result follows. A product or a quotient combines the units, so
a force over a length is a stiffness; a power to a number written in the formula raises them,
`sqrt` halves them (m² gives m), and the result of `asin` or `atan2` is an angle. A sum, a
difference, `min`, `max` and `clamp` take numbers of one unit, and `exp`, `ln` or `floor` a
number without unit. A formula whose units do not go together, such as a force plus a length,
gives no result; the formula in the block turns red and its tooltip says why. So does a formula
whose result goes to a number of another unit: the link stays while you correct the formula.

*Series* go element by element, as in NumPy (@series): a number goes with every value of a
series, and series of the same length go value by value. `max`, `min`, `mean`, `sum`, `std`
and `norm` of a single series make one number of it, so `max(abs(a)) / 235[MPa]` is the
utilization of the largest stress of a region. Of several values they work element by
element: `max(a, 0)` keeps the positive values of a series, `norm(a, b, c)` is the magnitude
of the components a, b and c at every node.

=== Exposed parameters <exposed-parameters>

Any number of an analysis that a study or a Parameters block should set, such as a force, a
thickness or the Young's modulus of a material, is first *exposed*. In the dialog of the item,
a small #ui("P") button follows each number that can be exposed; click it and close the dialog
with #ui("OK"). The number then is a row of the analysis' block in the project window, under
#ui("Exposed Parameters"), with the value it has and a violet pin on the left. It also appears
in the FE Model tree under #ui("Exposed Parameters"), where a double-click opens the item.

#screenshot("expose-button.png", [The #ui("P") button next to the components of a force: the Y
  component is exposed])

#screenshot("exposed-parameters.png", [The force of the analysis is exposed and given by the
  Parameters block: its row under Exposed Parameters is read-only and follows the parameter])

The row is named after the item and the number, such as #ui("Force-1 Y"); right-click it to
rename it or to remove the parameter, which leaves the number as it is. Type a value in the row,
or drag a value pin of a Parameters block to its pin: the number then follows the parameter,
and the field in the dialog is read-only while the link is there. Renaming the item keeps the
parameter. The numbers that can be exposed are those of loads, boundary conditions, defined
fields and initial conditions, of materials and sections, the duration and the number of frames
of static, heat transfer, visco and Dynamic steps, the number of modes of a frequency step and
the numbers of the transforms under #ui("Mesh Operations") (@mesh-tools). A Materials block (@materials-block)
exposes the numbers of its own materials in the same way, a Cross-Sections block
(@cross-sections-block) the dimensions of its cross-sections.

Exposing a number does not change the analysis: only a new value makes it out of date.

=== Measure <measure-block>

A Measure block reads from the results of a step: the values of a field at the nodes of a
region, the displacement of a node or of any point of the model, a history output such as a
total force, or an eigenfrequency. One number is what a parameter study reads, and a quick way
to see the figure that matters without opening the results; the values of a field and a whole
history are series (@series) for Math and Plot blocks. Add it with #menu("Evaluate",
"Measure"); it starts with the von Mises stresses at all nodes.

#screenshot("measure-blocks.png", [Two Measure blocks: the lowest temperature of the thermal
  step Heat and the largest von Mises stress of the step Load that uses those temperatures])

Drag the pin of a step under #ui("Results") of an analysis to the #ui("Results") row of the
block, with an orange wire like every link of results. A Measure block measures one step;
another wire to it takes the place of the first. The block shows what it measures and, in its
last row, the value in its units with the state of the value:

- a green tick when it was computed from the results as they are and those are up to date;
- the orange arrows when the results are out of date, after a change of the analysis or of a
  block it links to; #ui("Update Project") solves the analysis again and the value follows;
- a dash while there are no results, or when the results lack what it measures. The output
  pane then says why, for example "step 1 has no field STRESS in the results".

The value is computed anew in the background whenever the results or its definition change,
and the output pane prints it ("Tip load: TOTAL_FORCE RF2 of SUPPORT = -80000 N"). The project
file keeps the value, so a reopened project shows it at once. The pin on the right of the
value row gives it to a parameter study (@parameter-study). A series shows how many values it
has and their range.

#screenshot("measure-editor.png", [The editor of a Measure block])

A double-click on the block opens its editor:

#fields(
  [Measure], [What it computes: #ui("Field values"), #ui("Value at a node"),
    #ui("Value at a point"), #ui("History output") or #ui("Eigenfrequency").],
  [Shown in], [The unit system of the value; it is stored in SI units.],
  [Field, Component], [The field and its component as the Results workspace names them:
    STRESS MISES, DISP ALL, NDTEMP T, TOTAL_FORCE RF3 for a history output. With results
    linked, both are lists of what those results have (a name they lack is shown in the
    warning color); without, they are typed. The value is that of the last increment of the
    step.],
  [Take], [For a history output: #ui("Last value"), the value of the last increment of the
    step, or #ui("Whole history"), the values of all its increments as a series. A whole
    history gives a second output #ui("Time") above the values, the time of each value, to
    plot the values over it.],
  [Region], [For field values: all nodes, or a region of the analysis whose results are
    linked: a node set of its mesh or model, or the region of a boundary condition or load,
    such as #ui("Displacement_Rotation-1 (boundary condition)"). The region is found on the mesh the results
    were solved on, so a region picked on geometry or on the sides of a mesh block follows
    every new mesh of a parameter study.],
  [Node], [For a value at a node: its number.],
  [X, Y, Z], [For a value at a point: rows of the block itself, see below.],
  [Set], [For a history output: the set of the history output, as written to the `.dat` file;
    a list of the sets of the linked results.],
  [Mode], [For an eigenfrequency: the mode, counted from 1, of a frequency step.],
)

==== Value at a point <measure-point>

#ui("Value at a point") reads a field where no node may be: at coordinates you give. The
block gets three rows #ui("X"), #ui("Y") and #ui("Z") between what it measures and its value,
each a length in the units of the block with an input pin like the inputs of a Math block.
Type a coordinate in its field, or drag a number to its pin: a parameter of a Parameters
block, the result of a Math block or an input of a parameter study. A linked coordinate is
shown in the color of value links and follows its source, so a study that makes a part longer
can move the point along with the end of the part.

#screenshot("measure-point.png", [The tip deflection of a beam read at a point: the length of
  the beam is a parameter that sets both the size of the mesh block and the X of the point])

The point is located in the mesh the results were solved on, and the field is interpolated
there with the shape functions of the element that contains it, the same functions the
element computes with, so a curved quadratic element counts with its true shape. In a 2D
model the point lies in the plane of the model (Z is 0). Where the point lies on a node or on
the boundary between elements, every element gives the same value, as CalculiX writes nodal
values averaged between elements.

A point just outside the mesh, within 5 % of the size of the nearest element, takes the value
of the nearest point of that element: a point on the round surface of a part lies a little
outside a mesh of straight-edged elements. Farther out the block has no value, and the output
pane says how far the point is from the nearest node of the mesh. Beams and trusses that
CalculiX writes as lines are left out; those written as solids (the default) count.

=== Mesh Export <mesh-export>

A mesh export block is the opposite of a source: it takes a mesh and writes it to a file, as
a CalculiX input file (`.inp`) with the nodes, the elements, and the node sets and surfaces
of the mesh. Add it with #menu("Evaluate", "Mesh Export") or from the picker under
#ui("Export"), and drag the #ui("Mesh") row of a mesh block or an analysis to its #ui("Mesh")
row. Several export blocks may take the same mesh.

#screenshot("mesh-export.png", [A box whose mesh is written to box.inp])

The block has two rows. #ui("Mesh") is the input; its symbol shows whether the file is up to
date. #ui("File") is the file to write: type its path, or double-click the block's name to
pick it in a dialog. A relative path starts at the folder of the project file, so a project
moved together with its export folder keeps working; while the project is unsaved it starts
at the folder the program was started in.

#ui("Update Project") writes the file once everything before the block is done: the mesh of
a mesh block is made first, an analysis is solved first, and the file is written from the mesh
as it is then. It writes only when something changed, that is when the mesh or the path is
new or the file is gone, so an update of a project that did not change leaves the file alone.
If an analysis before the block fails, the block does not write. #ui("Export Now") in the
block's context menu writes the file at once, whatever the state.

The file holds the mesh only, not the loads or steps of an analysis that takes the same mesh,
so it reads back as a mesh when an analysis imports it. A path the program cannot write
shows as an error in the #ui("Mesh") row's tooltip and in the output window.

=== Plot and Preview <plot-block>

The blocks under #ui("Visualize") show results in the schematic itself, without opening an
analysis. Add them with #menu("Evaluate", "Plot") and #menu("Evaluate", "Preview"), or
search them with #key("Shift+A").

#screenshot("visualize-blocks.png", [A cantilever bent far by a force at its tip. Force over
  Deflection plots the reaction force, turned positive by a Math block, over the deflection of
  the tip, both whole histories of the step; Stress Histogram counts the von Mises stresses of
  all nodes, whose maximum a Math block Max gives. Beam shows the mesh of the analysis])

A *Plot block* draws the numbers and series (@series) linked to it in the block, and draws
again whenever they change. Its #ui("Type") row decides what it draws and which inputs it has:

#fields(
  [Line], [#ui("Y") over #ui("X"), joined by lines. Without a link to #ui("X") the values of
    #ui("Y") are drawn over their number, 1, 2, 3 and so on. The #ui("Time") of a Measure
    block of a whole history at #ui("X") draws the history over the time of the step.],
  [Scatter], [The same as points without lines.],
  [Histogram], [How many of the #ui("Values") fall into each of #ui("Bins") bins of equal
    width between the smallest and the largest value, for instance how much of a part sees
    which stress. #ui("Bins") is a count, typed in the block or linked.],
)

The axes show the smallest and the largest value and are named after the blocks that give
them, with their units, which those blocks set. #ui("X") and #ui("Y") of different lengths are
drawn as far as both go, with a warning in the corner. A single number counts as a series of
one value.

A *Preview block* shows whatever is linked to its one input, so that a glance at the
schematic shows it without opening anything. Before a link the row is called #ui("Input");
it takes the name of what it shows:

#fields(
  [Mesh], [A small picture of the mesh: drag the #ui("Mesh") of an analysis or of a mesh
    block there. The parts have the colors of the analysis, seen along the diagonal like the
    isometric view; a 2D mesh is seen from the front. The picture is drawn anew in the
    background whenever the mesh changes.],
  [Value], [A number, drawn large with its unit and the name of the block that gives it.
    A series (@series) is drawn as a line over its index, with its count, mean, smallest and
    largest value and standard deviation below.],
)

A new link replaces the one before, whatever it brought: a Preview block shows one thing.
Until the analysis is solved, a linked number shows #ui("No value yet").

#screenshot("preview-block.png", [Two Preview blocks: Deflection shows the series of the tip
  deflection over the step with its statistics, Peak Stress the maximum stress that the Max
  block gives])
