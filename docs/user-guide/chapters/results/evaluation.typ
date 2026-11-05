#import "../../template.typ": *

== Transformations <transformations>

The transformation button among the result controls of the ribbon shows the results of a symmetric or periodic
model as the full structure. Transformations only change the display.

#figure(
  grid(
    columns: (40%, 60%),
    gutter: 8pt,
    image("../../images/transformation.png"), image("../../images/symmetry.png"),
  ),
  caption: [Symmetry about the YZ plane and its result],
)

Move a type from #ui("Available") to #ui("Active") and set its properties:

/ Symmetry X/Y/Z: mirrors the model at the plane normal to the axis through the #ui("Symmetry
  point").
/ Linear pattern: copies the model #ui("Number of items") times along the line from the start to
  the end point.
/ Circular pattern: copies the model #ui("Number of items") times by #ui("Angle") about the axis
  through two points.

Several transformations are applied one after the other. Vector and tensor results are mirrored
or rotated with the copies, and legend and labels include them. #ui("Apply") shows the
transformations, #ui("Close") applies them and closes the window, and #ui("Off") shows the
results without transformations again.

== Derived field outputs <derived-fields>

#ui("Derived Field") on the #ui("Evaluate") tab of the Results workspace computes new fields
from the results.

#screenshot("field-output.png", [Create Field Output])

#fields(
  [Limit], [Divides a component by a limit value per part, per element set or for all elements.
    Gives `RATIO` (value / limit) and `SAFETY_FACTOR` (limit / value).],
  [Envelope], [`MAX`, `MIN` and `AVERAGE` of a component over all increments of all steps.],
  [Equation], [A formula of components, for example `=Abs([STRESS.S11]) / 235`, with the
    functions (`Abs`, `Sqrt`, `Max`, `Min`, `Pow`, `Sin`, `If`, ...). #ui("Insert
    Component...") inserts a component name. #ui("Unit") is shown in the legend.],
  [Coordinate System Transformation], [Rotates a vector or tensor field into a coordinate system
    defined under #ui("Features"); cylindrical systems give radial, tangential and axial
    components.],
  [Principal Direction], [The direction of the largest (#ui("Max")), middle (#ui("Mid")) or
    smallest (#ui("Min")) principal value of a tensor field such as `STRESS` or `TOSTRAIN`, or of
    the one largest in magnitude (#ui("Abs Max"): #ui("Max") where the part is pulled,
    #ui("Min") where it is pushed, like `SGN_MAX_ABS_PRI`), as a vector field: `X`, `Y`, `Z` of the unit vector, `VALUE` the principal value with its sign
    and `ALL` its magnitude.],
  [Anisotropy], [How clearly that principal direction is defined: the gap between its principal
    value and the nearest other one, divided by the largest magnitude; for #ui("Abs Max") also
    the gap between the magnitudes of #ui("Max") and #ui("Min"). 0 means the direction is
    undefined, for example under hydrostatic pressure or where the material carries no stress;
    uniaxial tension and pure shear give 1.],
)

#limitation[Derived field outputs are lost when the result file is reloaded.]

=== Principal directions <principal-directions>

The direction of the largest principal stress shows the path the load takes through a part,
its force flow; the direction of the smallest one shows the compression paths. A principal
direction has no sign: the vector and its opposite are the same direction. Nodalis turns each
vector so that its largest component is positive, which keeps the contours of `X`, `Y` and `Z`
from jumping between neighbouring nodes. Where the principal value is repeated, as the two zero
values of uniaxial tension, any direction in their plane is one; Nodalis picks one normal to
the others. Where all three are equal, the direction is undefined and the node has no value.

#screenshot("principal-direction.png", [The Y component of the Max principal direction of
  `STRESS` in a plate with a hole under tension])

Principal directions are vector fields, so result transformations mirror and rotate them
like displacements.

== History outputs and tables <history-outputs>

#ui("History Outputs") in the Results workspace lists the history outputs from the `.dat` file
(for example the eigenfrequencies of a frequency step) and the ones you create with
#ui("History Output") on the #ui("Evaluate") tab:

/ From Field Output: values of a field at selected nodes, one row per increment.
/ From History Output by Equation: a formula of other history outputs.
/ From Element Size: volume of elements or area of faces in every increment.
/ From Hot Spot: the values of a hot spot (@hot-spots) in every increment, optionally beside the
  position of a moving load (@hot-spot-history).

A double-click on a component opens its table. Click column headers to select them
(#key("Ctrl") adds, #key("Shift") selects a range) or drag over cells. #ui("Copy")
(#key("Ctrl+C")) copies the selection as tab-separated text, #ui("Plot") (#key("Ctrl+P")) draws
the selected columns over the first one.

#screenshot("history-table.png", [Eigenfrequencies from the `.dat` file])

The #ui("Step") column names the step of each row, as the FE model names it; a darker line
separates the rows of one step from the next. Rows of steps that run over time have the
#ui("Time") column.

#screenshot("history-table-steps.png", [The displacements of two nodes over three static steps])

The small arrow at the right edge of a column header sorts the rows by that column: the first
click sorts ascending, the next one descending. It appears when the pointer is over it and stays
on the sorted column. Empty cells go last either way. #ui("Sort Ascending"), #ui("Sort
Descending") and #ui("Original Order") in the context menu do the same for the column you
right-clicked. Copy and Plot take the rows in the order shown, so a plot over a sorted column
draws its curves from the smallest x value to the largest instead of in increment order. A new
order keeps the selected columns but clears a selection of rows.

#screenshot("history-table-sorted.png", [Rows sorted by the displacement of node 1, and the
  plot of the time over it])

=== Steps in the plot <history-plot-steps>

A plot over the step, increment or time column shows the steps as bands behind the curves, with
their names at the top and dashed lines where one step ends and the next begins. #ui("X axis")
at the bottom of the plot window switches between #ui("Total time"), which runs on through the
steps, #ui("Step time"), which starts again at zero in each step (without bands, as the steps
then lie over each other), and #ui("Increment"), which counts the rows. A blue line marks the
increment the 3D view shows, and follows it when another increment is shown, by the timeline
(@results-timeline), the #ui("Increment") list or an animation. A click into the plot shows the
increment nearest to it in the 3D view; drag to move through the increments.

#screenshot("history-plot-steps.png", [Displacement of a node over three static steps; the 3D
  view shows the increment at 1.5 s])

#ui("Export") in the plot window copies the plot to the clipboard as an image (#ui("Copy to
Clipboard")) or saves it as a PNG or SVG file. The SVG file is vector graphics with the curves,
grid and legend, for reports in good print quality.

#screenshot("table-plot-export.png", [The Export menu of a plot window])

#ui("Show in View") in the plot window puts the plot into the 3D view as an annotation, like a
path plot (see @paths): it stays after the window is closed and appears in screenshots. Drag the box
to move it and its bottom right corner to resize it. #ui("Edit ...") in its context menu, or a
double-click, opens #ui("Edit Annotation") for the title, the axis labels and a fixed value range;
#ui("Delete"), or #ui("Remove from View") in the plot window, removes it. The plot keeps the values
it was made from; with results kept with the project (@bind-result) it is saved with them.

#screenshot("table-plot-in-view.png", [Displacements of two nodes over time, plotted from their
  table and shown in the 3D view])

== Paths <paths>

#ui("Path") on the #ui("Evaluate") tab shows the displayed component along a straight line.

#screenshot("path.png", [Stress along a path through the hole])

#fields(
  [Start, End], [Coordinates (typed or picked with #ui("...")) or reference points.],
  [Points], [Number of points along the path (default 100).],
)

The dialog shows minimum, maximum and a plot of the value over the distance. #ui("Copy to
Clipboard"), #ui("Save as CSV...") and #ui("Save as JSON...") export the values
(@table-json); #ui("Table") lists them. Points outside the mesh have no value. The dialog's height can be changed by dragging its bottom right corner; the
plot takes the extra height.

#ui("Show in View") puts the plot into the 3D view as an annotation, where it stays after the
dialog is closed and appears in screenshots. It shows the displayed component of the current
increment and follows when another one is selected. Drag the box to move it and its bottom right
corner to resize it; #ui("Delete") in its context menu, or #ui("Remove from View") in the dialog,
removes it.

#screenshot("path-plot-in-view.png", [A path plot as an annotation in the 3D view])

#ui("Show on Path") in the context menu draws the plot onto the path itself instead: the path is
the distance axis, the values stand off it perpendicular on the screen over a translucent area,
with the largest and smallest value labelled. Drag the curve away from the path or towards it to
change its height. Drag it across the path and the height becomes negative: the plot flips to
the other side of the path. #ui("Show in Box") puts it back into its box.

#screenshot("path-plot-on-path.png", [The same plot drawn onto its path])

#screenshot("path-plot-flipped.png", [The plot dragged across the path: negative height, other side])

#ui("Edit ...") in the context menu, or a double-click on the plot, opens #ui("Edit Annotation").
Changes show at once; #ui("Cancel") restores the plot.

#screenshot("path-plot-edit.png", [Editing a path plot])

#fields(
  [Display], [#ui("In Box") or #ui("On Path").],
  [Height on path], [Distance of the largest value from the path, in screen pixels; a negative value puts the plot on the other side of the path.],
  [Title], [Empty for the path's name and the displayed component.],
  [Distance axis label, Value axis label], [Labels of the box's axes; the value axis label is
    written above the axis, empty for none.],
  [Value range], [#ui("Automatic") fits the axis to the values; otherwise #ui("Minimum") and
    #ui("Maximum") fix it, e.g. to compare increments. On the path, the larger end is drawn at
    the height.],
)

== Plane results

#ui("Plane Result") on the #ui("Evaluate") tab evaluates the displayed component on a plane defined
under #ui("Features"): minimum and maximum with their position, the area-weighted mean, and the
cut area. With its check box ticked, the 3D view shows only the cut, and the legend uses the
values on it. #ui("Save as CSV...") and #ui("Save as JSON...") save the cut points with their
values.

== Hot spot stresses <hot-spots>

Hot spot stresses evaluate welded joints as recommended by the IIW: the stress at the weld toe is
extrapolated from read-out points on the plate surface. Double-click #ui("Hot Spot Stresses") in
the Results tree, or use #menu("Evaluate", "Hot Spot"), and pick the weld toe.

#screenshot("hot-spot.png", [Create Hot Spot with the stress along the path, the read-out points
  and the extrapolation to the weld toe])

#fields(
  [Weld toe], [A node at the weld toe. An edge can be picked as well: of its nodes, the one with
    the highest hot spot stress in the displayed increment is evaluated and plotted.],
  [Extrapolation], [IIW type a, fine mesh, linear (0.4t / 1.0t, default) or quadratic
    (0.4t / 0.9t / 1.4t); type a, coarse mesh (0.5t / 1.5t); type b, fine (4 / 8 / 12 mm) or
    coarse (5 / 15 mm); or #ui("Custom read-out points").],
  [Plate thickness t], [For type a methods.],
  [Read-out points], [Shows the extrapolation formula.],
  [Stress], [#ui("Perpendicular to weld toe"), #ui("Max principal stress") or #ui("Signed max
    abs principal stress").],
  [Path direction], [Points away from the weld toe; Nodalis turns it into the plate surface.
    #ui("Flip") reverses it.],
)

Below the inputs the dialog shows the result for the displayed increment while you edit: the hot
spot stress `S_hs` with its node, the stresses at the read-out points and a plot of the stress
along the path from the toe to a quarter beyond the last read-out point. The read-out points and
the hot spot stress are blue dots, the extrapolation through them is dashed, so you can check at a
glance whether the read-out points lie on the smooth part of the stress rise. The value axis
leaves out a sharp notch peak right at the toe, which would otherwise flatten the plot; hover the
curve to read values. #ui("Copy to Clipboard"), #ui("Save as CSV...") and #ui("Save as JSON...")
export the curve, the read-out points and the hot spot stress; in JSON these are three tables in
an array.

#ui("Show in View") puts the plot into the 3D view as an annotation that stays after the dialog is
closed and follows the displayed increment, like the plot of a path (@paths): drag it to move it,
drag its corner to resize it. Its context menu offers #ui("Edit ..."), #ui("Show on Path"), which
draws the curve, the read-out points and the extrapolation onto the path in the model, and
#ui("Delete").

#screenshot("hot-spot-on-path.png", [The plot of a hot spot shown on its path])

The evaluation runs for all increments and writes `<result>_hot_spots.csv` next to the result
file, with one row per toe node and increment; the output pane lists the highest value of each hot
spot. A hot spot is deleted with the #key("Del") key or #ui("Delete") in its context menu.

=== Hot spot stress over all steps <hot-spot-history>

#ui("Create History Output") with the type #ui("From Hot Spot") writes the values of a hot spot
into a history output, one row per increment, so the hot spot stress can be followed through all
steps, for example while a crane drives over a girder with a moving load (@moving-load).

#screenshot("hot-spot-history-dialog.png", [A history output of a hot spot beside the position
  of the crane])

#fields(
  [Hot spot], [The hot spot of the results whose values are written.],
  [Moving load], [#ui("None"), or a moving load of the FE model. Its position along the path
    (the position of the vehicle's reference point, between #ui("Start") and #ui("End")) is
    written as the first column, one value per step. Increments of steps without the load have
    no position.],
)

The history output has the field `HOT_SPOT` with one component per toe node, `NODE_<id>`. Its
table holds all values of the node side by side: the position of the moving load (`POSITION`, if
one is chosen), the hot spot stress `S_HS` and the stresses at the read-out points `S_1`, `S_2`,
... (nearest first). A hot spot on an edge with several toe nodes also gets the component `MAX`
with the largest hot spot stress of all toe nodes in each increment and its `NODE`. Increments
without stresses have no values.

To plot the stress over the position, double-click the component, select the column `POSITION`
and then `S_HS` (#key("Ctrl")-click), and choose #ui("Plot") (#key("Ctrl+P")).

#screenshot("hot-spot-history-plot.png", [The table of a toe node near the fixed end of a
  cantilever and its hot spot stress plotted over the position of a wheel load driving out])

The history output follows its hot spot: when the hot spot is changed, it is computed again; when
the hot spot is deleted or renamed, or the moving load is no longer in an active step of the FE
model, it reports an error in the output pane and has no data until it is edited.

#note[The position is that of the step. Within a step with several increments the loads are
ramped from the previous step, so the intermediate increments show the same position as the end
of their step.]

== Tables as JSON <table-json>

Wherever a table can be saved as CSV (paths, plane results, hot spot plots, parameter studies and
the Campbell diagram), it can also be saved as JSON, which other programs read without parsing
text. The file holds the column names and the rows; numbers are numbers, empty cells `null`, and
text such as a whirl direction stays text:

```json
{
  "columns": ["Distance", "X", "Y", "Z", "S Mises [MPa]"],
  "rows": [[0, 0, 5, 0, 212.4], [1.25, 1.25, 5, 0, 208.9]]
}
```

A file with several tables, such as that of a hot spot plot, holds an array of them. In a Typst
document, for example, `json("path.json")` reads the file, and its `rows` go straight into a
table or a plot.

#screenshot("path-json.png", [#ui("Save as CSV…") and #ui("Save as JSON…") in the path dialog])

== Query <query>

#ui("Query") on the #ui("View") or #ui("Evaluate") tab or the #ui("Query") tile of the ribbon (the
pipette) reads values from the model: #ui("Vertex/Node"), #ui("Facet/Element"), #ui("Edge")
(length), #ui("Surface") (area), #ui("Part"), #ui("Assembly"), #ui("Bounding box size"),
#ui("Distance") (two points), #ui("Angle") (three points) and #ui("Circle") (centre and radius
through three points). The values appear in the output pane and as labels in the 3D view. On
results, node queries also show the deformed position, the displacement and the value of the
displayed component; edges and surfaces also have the deformed length or area and the change.

The queries that pick in the 3D view open the #ui("Selection") window beside the query window,
the same as every region of the model (@selection), and pick the way it says: in the geometry
based mode a click takes the CAD face or edge, or the smooth patch bounded by feature edges, the
angle modes grow it across smooth edges, and #ui("More") offers the mesh based modes (single
nodes, elements, faces, IDs). Hovering shows what a click would pick.

- A plain click starts a new query and leaves its label; the labels of earlier queries stay.
- #key("Shift") + click adds to the query just made, #key("Ctrl") + click removes from it; its
  label and the output pane follow, e.g. the total area of several faces or the length of a chain
  of edges.
- Dragging a box picks everything inside it; #ui("All"), #ui("Invert"), #ui("Undo") and
  #ui("Clear") of the selection window work on the query just made.
- Several nodes give their number, their centre and the smallest and largest value of the
  displayed component; several elements their number by element type; several parts one
  block each.
- #ui("Distance"), #ui("Angle") and #ui("Circle") take one point per click, in order. The
  geometry based mode takes the corner point nearest to the click, the #ui("Node") mode under
  #ui("More") any node.

On the Geometry tab, #ui("Edge") and #ui("Surface") pick CAD edges and faces with the geometry
based modes only, as for a local mesh size.

#screenshot("query.png", [Three holes queried as one surface: the first by a click, the others
  with Shift + click; the fourth hole shows what the next click would pick])

#screenshot("query-geometry.png", [Edge and surface queries on the Geometry tab])

The labels stay in the view when the query window is closed and can be dragged
aside. A right click on a label offers #ui("Delete") for this label and #ui("Delete All
Annotations"); #ui("Remove Annotations") in the query window and on the
ribbon deletes all of them. Each model keeps its own labels: the geometry, the FE model and every
result file. Meshing again removes those of the FE model, since they refer to node and element
ids.

#screenshot("query-annotation-menu.png", [Query labels after closing the window, with the
  context menu of a label])
