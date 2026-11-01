#import "../../template.typ": *

== Files and projects <files>

=== File menu of an analysis window

The #ui("File") menu of the project window is described in @project-window. The analysis
window has these entries:

#fields(
  [Import ...], [Imports CAD geometry or the mesh of input files (@geometry-import,
    @mesh-import).],
  [Save Project (#key("Ctrl+S"))], [Saves the project with all its analyses. A project that
    has no file yet is saved with #ui("Save Project As").],
  [Save Project As ... (#key("Ctrl+Shift+S"))], [Saves the project under a new name.],
  [Save Project Without Results ...], [Saves a copy without the results the analyses keep
    (@bind-result).],
  [Export CalculiX Input File ...], [Writes the input file exactly as #ui("Solve") would
    (@export-inp).],
  [Export Deformed Mesh (.inp) ...], [Writes the displayed deformed mesh of the current result
    as a mesh-only input file (@export-deformed).],
  [Export CalculiX Results (.frd) ...], [Writes the current results as CalculiX's `.frd` file,
    with the `.dat` file of the history output beside it (@bind-result).],
  [Export VTU ... / Export VTK ...], [Write the current results, one increment or a time
    series, for ParaView and other VTK programs (@export-vtk).],
  [Close Analysis], [Closes the window; the analysis stays in the project.],
)

#note[Nodalis does not ask about unsaved changes. Save regularly. Changes of the model can be
  undone (@undo).]

=== What each file type does

/ `.ndl` (Nodalis project): all analyses of the project, each with its geometry, mesh setup,
  mesh, FE model and the results it keeps (@bind-result), and the place of its block in the
  schematic. Saving writes a temporary file first and then replaces the old one, so a crash
  during saving cannot destroy the project, together with the links between the analyses.
  A project written by a newer version of Nodalis is refused, and so are projects saved by
  Nodalis 0.2.6 and older, which held one model only. Projects of the first multi-analysis
  format (one model space for all analyses) are converted on opening: their results are dropped
  and have to be solved again, and once saved, the project no longer opens in older versions.
/ `.inp` (CalculiX input): imported with #menu("Geometry", "Import ...") (@mesh-import). *Only
  the mesh* is read: nodes, elements, `*NSET`, `*ELSET` (also `GENERATE`), `*SURFACE` and files
  referenced with `*INCLUDE`. Every element set named in `*ELEMENT, ELSET=...` becomes a part.
  All other keywords (materials, steps, ...) are skipped and listed in the output pane.
/ `.frd` (CalculiX results): the results of the analysis' own runs with CalculiX. Result files
  from elsewhere are not opened.
/ `.ndlr` (Nodalis solver results): the results of the analysis' own runs with the built-in
  solver, a compact binary file instead of `.frd` and `.dat` (@bind-result).
/ CAD files: imported with #menu("Geometry", "Import ...") (@geometry-import).

Supported element types of `.inp` meshes: `C3D4`, `C3D6`, `C3D8`, `C3D10`, `C3D15`, `C3D20` with
their `R` and `I` variants, `S3`, `S4`, `S6`, `S8`, `M3D` membranes, `CPS`/`CPE`/`CAX` plane and
axisymmetric elements, `B31`, `B32`, `T3D2` and `T3D3`.

=== Drag and drop and command line

A project file dropped onto the project window or an analysis window opens in the project
window. CAD files and input files dropped onto an analysis window are imported into it; several
CAD files dropped together are combined into one geometry.

On the command line, `nodalis project.ndl` opens the project. Other files, for example
`nodalis plate.step`, start a new project with one analysis named after the first file, into
which the files are imported. Without files, Nodalis reopens the last saved project that was
open at exit.
