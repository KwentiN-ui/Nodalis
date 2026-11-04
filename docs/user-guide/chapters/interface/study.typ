#import "../../template.typ": *

== Parameter studies <parameter-study>

A *Parameter Study* block solves the project for several sets of numbers and lists what came
out, like the parameter set of Ansys Workbench. Its table has a column for each number it
varies (its inputs, P1, P2 ...) and for each number it reads back (its outputs, O1, O2 ...);
each row is a *design point*, a set of input values with the outputs they gave.

Add one with #menu("Evaluate", "Parameter Study"), then link its columns:

+ Drag the pin of its #ui("Inputs") row to a number the study varies: a parameter exposed by
  an analysis (@exposed-parameters), a number of a mesh block, the #ui("Factor") of a Refine
  Mesh block (@refine-mesh) or an input of a Math or Switch block (@math-switch). The link makes an input column named after that number, of its
  kind (a force, a length ...), with its value as it is now. The wire is violet and dashed and
  runs below the blocks, as it leads back to the blocks before the study. One column can give
  its value to several numbers: drag the pin of its row to each. Quicker for an exposed parameter:
  right-click its row and pick the study under #ui("Add to Study").
+ Drag the value pin of a Measure block (@measure-block) to its #ui("Outputs") row. The link
  makes an output column named after the measure. The #ui("Result") of a Math or Switch block
  is an output in the same way, named after the block.

The first column makes the first design point, *DP 0*, which is the *current* one: the blocks
stand at its inputs, and its outputs are those the project shows. Removing a link leaves the
column; #ui("Remove") in the editor removes a column and its links.

#screenshot("add-to-study.png", [#ui("Add to Study") in the context menu of an exposed
  parameter])

#screenshot("study-block.png", [The study varies the exposed force of the analysis Static
  (dashed wire) and reads the largest stress of its results through Max Stress])

A double-click on the block (or #ui("Edit") in its context menu) opens its editor, with the
columns and the table of the design points:

#screenshot("study-editor.png", [Three design points solved: the stress grows with the force])

#fields(
  [Shown in], [The unit system the inputs are shown and entered in. The outputs are shown in
    the units of their Measure blocks.],
  [P1, O1 ...], [The columns, with the kind of each input. A double-click on a name renames
    it; #ui("Remove") removes the column and its links.],
  [Point], [The design points; the current one is marked.],
  [State], [#ui("not solved"), #ui("up to date"), #ui("out of date") once the blocks the
    study runs, the inputs of the point or the outputs changed since it was solved, or
    #ui("failed") when a solve or a measure gave no value. A right-click on it offers
    #ui("Set as Current"), #ui("Duplicate") and #ui("Delete").],
  [P1 ...], [The input values of each point, entered in place.],
  [O1 ...], [The outputs each point gave.],
  [Time, Elements], [How long the point took and the elements of its largest mesh.],
  [Table, Plot], [The table of the points, or a diagram of their outputs
    (@study-plot).],
  [Fill ..., Import ...], [Add many points at once (@study-fill).],
  [Copy Table], [Copies the table with tabs, for a spreadsheet.],
  [Export CSV ..., Export JSON ...], [Save the table as a CSV or JSON file (@table-json).],
  [Add Point], [Adds a point with the inputs of the current one.],
  [Run], [Solves the points that are not up to date, one after the other.],
  [Reset], [Forgets the outputs of all points; the inputs stay.],
)

#ui("Run"), the circular arrow of the block or #ui("Update Project") solve the points that
are not up to date. Each point is a whole update of the blocks between the study's inputs and
its outputs: meshes are made anew, the analyses are solved, the measures are computed. The
current point is solved last, so that the analyses end with its results; the results of the
other points are not kept, only their outputs. The output pane prints each point;
#ui("Stop") of the project window ends the run after the running solves stopped. A point stays
up to date as long as the blocks it runs are what it was solved with: change a load or a
material of the analysis and every point is out of date, and the next #ui("Update Project")
solves them again. #ui("Set as Current") gives the inputs of a point to the blocks; as the point
is up to date, #ui("Update Project") then solves the analyses for its results without running
the study.

=== Filling and importing points <study-fill>

#ui("Fill ...") makes points from values per input. Each input takes the value of the current
point (#ui("Current")), a #ui("List") of values separated by `;`, such as `1; 1.5; 2`, or a
#ui("Range") of evenly spaced values from one value to another. #ui("All combinations") makes a
point for every combination of the values of the inputs; #ui("One at a time") varies each input
by itself, the others at the current point. A point with the inputs of a point already in the
table is not added again. #ui("Replace the points") removes all points but the current one
first. The dialog says how many points it adds before #ui("Add Points").

#screenshot("study-fill.png", [Five forces from -1000 to -5000 N: three of them are in the
  table already])

#ui("Import ...") adds the points of a table pasted from a spreadsheet, or read with
#ui("Open File ...") from a CSV file. Cells are separated by tabs, `;` or `,`. A first row of
names takes the columns named after inputs, with or without the unit in brackets, as
#ui("Copy Table") and #ui("Export CSV ...") write them; an input without a column takes the
value of the current point. Without such a row, the columns are the inputs in order. Values
are in the units shown; a value with its own unit, such as `7 kN`, is converted.

=== Plot <study-plot>

The #ui("Plot") tab draws the outputs over an input, another output or the elements (#ui("x")).
The boxes #ui("O1"), #ui("O2") ... choose the outputs shown, and #ui("Curve per") makes a curve
for each value of another input, for a study over two inputs. Points without the values are
left out. Drag to move the view, scroll to zoom, double-click to fit.

#screenshot("study-plot.png", [The largest stress over the force: five points of a linear
  model on a straight line])

=== Mesh convergence

A study checks whether the results depend on the mesh: put a Refine Mesh block between the
mesh and the analyses that solve on it, link the Inputs row to its #ui("Factor") and a Measure
block of the results to the Outputs row, and add points with factors such as 1, 1.5, 2 and 3.
The #ui("Elements") column shows how many elements each one solved; in the plot, the measure
over #ui("Elements") is the convergence curve. When the output hardly
changes from one point to the next, the mesh is fine enough. When it grows with every
refinement, it is a singularity: a stress at a sharp corner or under a point load grows without
end as the mesh gets finer. Measure a displacement there, or a region away from it.
