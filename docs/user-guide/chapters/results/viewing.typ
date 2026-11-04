#import "../../template.typ": *

= Results <results>

== Opening results

The Results workspace shows the results of its analysis: they open when a run is finished,
with #ui("Results") in the monitor, with #menu("File", "Open Results") and when an analysis
with kept results is opened (@bind-result). The `.dat` file of the run is read as well. Result
files from elsewhere are not opened.

When results are opened, Nodalis shows the Results workspace with the last increment.

#screenshot("results.png", [von Mises stress of a static step])

== Results tree

/ Model: the mesh of the results with its parts and sets, and the features (reference points,
  coordinate systems, planes) used for result evaluation.
/ Field Outputs: the fields of the result file with their components. Click a component to show
  it. Names follow the CalculiX result file (`U1`, `S11`, ...). Nodalis adds the displacement magnitude `ALL`,
  `MISES`, `TRESCA`, the principal stresses and the signed maximum absolute principal stress.
/ History Outputs: values from the `.dat` file and history outputs you create (@history-outputs).
/ Hot Spot Stresses, Paths, Plane Results: evaluations described in the following sections.

Results list the parts of the model they were computed from, as in the FE Model workspace, also
when several parts share a section. A `.frd` file itself only groups the elements by material;
Nodalis reads the parts from the input file CalculiX ran, the `.inp` file of the same name next
to the results. Results of an analysis run from Nodalis always have it. The spring and gap
elements that constraints become in the input file, such as the gaps of a compression only
support (@compression-only), are left out of the results: CalculiX would list each gap as a
material of its own, and they carry no results of their own.

#screenshot("results-parts.png", [Three parts sharing one section keep their names in the results])

#screenshot("results-without-gaps.png", [A plate on compression only supports: its results show
  the plate as the only part])

== Result controls <results-toolbar>

#screenshot("results-display-tab.png", [The #ui("Display") tab: the view tools, then the groups
  #ui("Deformation"), #ui("Contours") and #ui("Increment"); a narrow window folds a group into a
  menu])

The result controls are tiles on the #ui("Display") tab of the Results workspace, in three
groups. A tile with an arrow shows its current value under its name and opens a panel to change
it; the other tiles act at once and show pressed while their option is on.

#fields(
  [Deformation], [#ui("Undeformed"), #ui("True scale"), #ui("Automatic") with factors from
    0.25 to 5, or #ui("User defined"), with the #ui("Factor") below them; it is typed for
    #ui("User defined"). Results open in #ui("Automatic"); static and dynamic
    steps of a model with contact open in #ui("True scale"), so touching parts do not appear to
    penetrate each other, while mode shapes keep #ui("Automatic"). Contact is recognised by a
    contact pair in the input file next to the results (same name, as CalculiX leaves it) or
    contact results in the file. Until you pick a deformation yourself, it follows the shown
    step.],
  [Show Undeformed], [Draws the undeformed outline behind the deformed shape.],
  [Global Contours], [For a submodel: the edges of the global model around the results.],
  [Display], [#ui("Contour"), #ui("Arrows") (@arrows) or #ui("LIC") streaks (@lic). Below them,
    #ui("Directions of") picks the field whose directions they follow: #ui("Shown field") or any
    vector or tensor field of the increment, so that, for example, the principal stresses run
    over the displacement contours. For a tensor field such as `STRESS`, #ui("Principal
    direction") picks #ui("Max"), #ui("Mid"), #ui("Min") or #ui("Abs Max"). The options of the
    arrows or streaks follow; what does not apply stays greyed out.],
  [Levels], [Number of colour bands (2 to 24).],
  [Color Scale], [The spectrum and limits of the colours (@color-scale).],
  [Transformations], [Symmetries and patterns (@transformations).],
  [Step, Increment], [The displayed increment, picked from the list of all of them.],
  [First, Previous, Next, Last], [Show another increment.],
  [Animate], [Opens the animation window (@animation).],
  [Sound], [Opens the sound window for modes of a frequency step (@sound).],
)

== Contour plot, legend and labels

The legend shows the field, the unit and the range in nine colour bands from blue (minimum) to red
(maximum) by default. The info block in the upper right shows file, date, step, increment or mode
with its frequency or buckling factor, and the deformation scale factor. Labels mark the node with
the maximum (and optionally the minimum) value. Legend, info block and labels can be dragged.
Which of them appear is set under #menu("Edit", "Settings ...", "Post-processing").

== Arrows <arrows>

#ui("Display") > #ui("Arrows") on the #ui("Display") tab draws arrows along the directions of the
shown field, or of the field picked next to it, over the surface: a vector field such as `DISP`, `FORC` or `FLUX` gives its own
direction, a tensor field such as `STRESS` or `TOSTRAIN` the principal direction picked next to
it. The arrows take the colours of the shown component and its legend, so `STRESS` with
#ui("PRINCIPAL_MAX") and the #ui("Max") direction shows how the load runs through the part, its
force flow, coloured by the tension it carries. While the arrows are coloured, the surfaces
turn grey. A principal direction has no sign, so it is drawn as a stroke without a head. The
legend names what the arrows show. A scalar field has no directions and is drawn as contours;
the choice comes back with the next vector or tensor field.

#screenshot("arrows-principal.png", [Arrows along the Max principal direction of `STRESS` in a
  plate with a hole under tension, coloured by `PRINCIPAL_MAX`])

#fields(
  [Number of arrows], [About this many arrows, spread evenly over the surface whatever the mesh
    density (default 2000). Both sides of a thin wall get their own.],
  [Length], [Length of the longest arrow relative to the spacing of the arrows (default 0.8).],
  [Length by magnitude], [Arrows are as long as the magnitude of the vector or the principal
    value, the largest one at full length; otherwise all are equally long.],
  [Arrows in contour colors], [On by default; off, the arrows are black over the colour
    contours.],
)

Mirrored and patterned copies (@transformations) show the arrows too, turned with the copy.
Parts without colour contours (@color-contours) get no arrows. Arrows and their options are
kept with the results of a project (@bind-result).

#limitation[Arrows sit on the outer surface; section faces (@section-view) have none.]

== LIC <lic>

#ui("Display") > #ui("LIC") draws the directions of the shown field as fine streaks over the
colour contours, by line integral convolution: a noise pattern fixed on the surface is smeared
along the directions, so that the streaks follow them everywhere at once. The directions come
from the same fields as the arrows (@arrows); with `STRESS` and the #ui("Max") direction the
streaks are the force flow through the part, the trajectories of its largest principal
stress. The colours still show the shown component, the streaks only change their
brightness. The legend names what the streaks show.

#screenshot("lic-force-flow.png", [Force flow in a bracket with a hole, clamped on the left and
  loaded downwards on the right: LIC along the Max principal direction of `STRESS`, coloured by
  `PRINCIPAL_MAX`])

Where the direction is not defined, the streaks fade out and the contours remain: where a
vector is close to zero, where the picked principal value is close to another one (unloaded or
evenly loaded regions), and where the direction points out of the surface or the surface is
seen edge on. #ui("Max") follows the tension paths and fades where the part is only pushed,
since there its largest principal value is close to zero like another one; #ui("Min") follows
the compression paths and fades where the part is only pulled. #ui("Abs Max") follows whichever
of the two is larger in magnitude, so one picture shows the load path through the pulled and
the pushed fibres of a bent part alike. Where tension and compression are equally large, as
near the neutral axis of a bent beam, its direction turns by 90 degrees; the streaks fade
along that line.

The streaks are drawn for the view on the screen, so they keep their fineness when zooming;
the noise stays on the model when the view turns. Section faces (@section-view) show the
streaks inside the part: move the plane through it to follow the force flow in the volume.

#screenshot("lic-section.png", [The bracket cut by a section plane: the streaks continue on the
  section face])

#fields(
  [Streak length], [Length of a streak to either side of each pixel, in pixels (default 15).
    Longer streaks are smoother but blur small details.],
  [Grain size], [Size of the noise grains in percent of the model size (default 0.1), never
    finer than a pixel. Coarser grains give wider streaks.],
  [Contrast], [How strongly the streaks stand out from each other (default 4).],
  [Mix], [How much of the brightness the streaks set, from 0 (contours alone) to 1 (default
    0.7).],
  [Defaults], [Restores the values above.],
)

Mirrored and patterned copies (@transformations) show the streaks too, turned with the copy.
Parts without colour contours (@color-contours) get none. The options are kept with the
results of a project (@bind-result).

== Color scale <color-scale>

Each result file has its own colour scale. Double-click the legend, choose #ui("Color Scale ...")
in its context menu, or click #ui("Color Scale") on the #ui("Display") tab. Changes show at once and stay when #ui("Close") closes the
window; #ui("Defaults") returns to the rainbow with automatic limits.

#screenshot("color-scale-dialog.png", [Color Scale dialog with manual limits: values above 40 MPa
and below 5 MPa are grey])

#fields(
  [Spectrum], [Cool-warm, Rainbow (default), Rainbow desaturated, Turbo, Jet, Warm, Cool,
    Cividis, Viridis, Plasma, Black body, Inferno, Kindlmann or Grayscale.],
  [Brightness], [Lightens the colours towards white, from 0 to 1 (default 0.2).],
  [Reverse colors], [Puts the maximum colour at the minimum and vice versa.],
  [Number of colors], [The colour bands, the same as #ui("Levels") on the #ui("Display") tab.],
  [Min/max limit type], [#ui("Automatic") spans the smallest to the largest shown value (over
    all frames of an animation if it is set so). #ui("Manual") uses #ui("Minimum") and
    #ui("Maximum"); the values are kept exactly as typed. The minimum must be smaller than the
    maximum.],
  [Above maximum, Below minimum], [Colour of the values beyond the manual limits (dark and light
    grey by default). The legend shows them as an extra box labelled with the largest or
    smallest value unless #ui("Show in legend") is cleared.],
)

The last line of the legend title says #ui("Automatic") or #ui("Manual"). When the results are
kept with the project (@bind-result), its colour scale is saved in the project file and restored
with it; a result file that is read again, for example after another run, keeps its scale.

#screenshot("color-scale-cool-warm.png", [The same stress in the Cool-warm spectrum with
automatic limits])

#screenshot("results-mode.png", [First mode shape of a frequency step])

=== Parts without colour contours <color-contours>

A stiff or irrelevant part can dominate the colour scale. Right-click a part under
#ui("Model") in the results tree, or in the 3D view, and choose #ui("Color Contours Off"). The
part turns plain grey and no longer counts for the legend range, the minimum and maximum labels
or the animation limits; the colours of the other parts then use the full scale.
#ui("Color Contours On") brings the contours back. With several parts selected, the command
applies to all of them. Results kept with the project (@bind-result) remember which parts have
contours off. Nodes that a grey part shares with a part that still has contours keep
their colour.

#screenshot("color-contours-off.png", [The left part without colour contours])

Increment 0 of a buckle step is the reference state; the modes follow as increments 1, 2, ...
