#import "../../template.typ": *

== Exporting results to ParaView <export-vtk>

#menu("File", "Export", "VTU ...") and #menu("File", "Export", "VTK ...") write the current results
for ParaView and other programs built on VTK. Both are enabled when results are open; they
export the result selected in the #ui("Results") workspace.

#screenshot("export-vtu.png", [The Export VTU dialog with a series of increments])

#fields(
  [Increments], [#ui("Current increment") writes the increment shown (default).
    #ui("All increments of step ...") writes every increment of its step, and
    #ui("All increments of all steps") every increment of the file. The numbers in brackets
    count the increments.],
  [Encoding], [#ui("Binary") (default) is smaller and faster to read; the VTU format also
    compresses it. #ui("ASCII") writes readable text.],
)

#ui("Export ...") asks for the file name. Each file holds the undeformed mesh and, as point
data, every field of the increment: the components CalculiX wrote form one array per field
(`DISP` as a vector, `STRESS` and `TOSTRAIN` as symmetric tensors in VTK's component order
XX, YY, ZZ, XY, YZ, XZ), and computed components such as `MISES` or the principal values are
scalar arrays named `STRESS_MISES` and so on. Field outputs defined under #ui("Results") are
included. `NodeId`, `ElementId` and `PartId` give the CalculiX node and element numbers and the
part of each element; a VTU file also lists the part names in its field data (`PartNames`).
Each file also stores `STEP`, `INCREMENT` and the increment's time, frequency or load factor as
field data.

To see the deformation in ParaView, apply #ui("Warp By Vector") with `DISP`.

=== VTU or VTK

/ VTU: VTK's XML format (`.vtu`). Use it for ParaView; it is compressed, stores names of the
  components and of the parts, and every VTK program of the last years reads it.
/ VTK: the legacy VTK format (`.vtk`, version 4.2) for older or simpler tools that do not read
  XML files.

=== Several increments as a time series

Several increments become one file per increment and a collection file that ParaView opens as a
time series, the way the ParaView documentation recommends for transient data:

- VTU: a ParaView Data file `name.pvd` and the files `name/name_0000.vtu`,
  `name/name_0001.vtu`, ... in a folder `name` next to it.
- VTK: a file series `name.vtk.series` (a small JSON file, read by ParaView 5.5 and later) and
  the files `name/name_0000.vtk`, ... The PVD format only holds XML files, so legacy files use
  this file series instead.

Open the `.pvd` or `.vtk.series` file, not the single files, and step through the increments
with ParaView's time controls. The collection gives every file its time: the increments' own
values (time, eigenfrequency or load factor) if they increase from file to file, as within one
static or dynamic step. Otherwise, for example when several steps start their time again or
mix a static and a frequency step, the files are numbered 0, 1, 2, ... in their order in the
results file, and the output pane says so. The real values stay in the field data of each file.
Keep the collection file and its folder together when moving them.

#limitation[The export writes the mesh and the fields of the results file. Transformations
(symmetry and patterns), the deformation scale and the results' features and paths are not
exported; build them in ParaView instead.]
