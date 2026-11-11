#import "../../template.typ": *

== Campbell diagram <campbell>

A Campbell diagram shows the eigenfrequencies of a rotor over its speed of rotation. CalculiX
computes the whirling modes for one speed per run, so Nodalis runs the analysis once per speed
and collects the results.

The sweep lives in a #ui("Campbell") block of the project schematic (@project-window), which
takes the analysis to sweep and keeps the diagram. The analysis needs an active
#ui("Complex Frequency") step (@complex-frequency), whose axis tells how the rotor turns; the
speed of the step does not matter, 0 included, as the sweep sets it. #menu("Solve", "Campbell Diagram") in the window of such
an analysis opens its Campbell block, and makes one next to the analysis and links it first if
there is none. A block can also be added from the block picker or the context menu of the
schematic (category #ui("Campbell")); then drag the #ui("Setup") pin of the analysis to the
#ui("Analysis") row of the block.

#screenshot("campbell-block.png", [A Campbell block linked to the analysis it sweeps, with the
speeds of its sweep and the critical speeds of its diagram])

The block shows the state of its diagram in its #ui("Analysis") row, how many speeds it has and
up to five critical speeds. The speeds and the settings of the sweep are saved in the project
file, so the diagram is there again when the project is opened. The diagram stays up to date
while the analysis is unchanged, apart from the speed of its Complex Frequency step; once the mesh or
the model of the analysis changes, the block turns out of date. #ui("Update Project") starts the
sweep of every Campbell block that is out of date or has not swept yet, beside the solves of the
analyses; the sweep runs the model of its analysis by itself and needs no results of it.

Double-clicking the block, or #ui("Open Diagram ...") in its context menu, opens the window of
the sweep. Its toolbar holds #ui("Start Sweep"), #ui("Pause"), #ui("Continue") and #ui("Stop")
with the progress of the sweep, and #ui("Copy Table") and the exports on the right. Below, the
diagram fills the left; a panel on the right holds the settings of the sweep, the critical
speeds, the selected speed and the output of the runs. Drag the vertical separator between
diagram and panel to give either more room, and the line above #ui("Output") to share the panel
between the output and the rest. The sweep does not run the analysis itself and adds nothing to
its results. The diagram appears as soon as two speeds are done. The window can be closed while
the sweep runs: the block goes on and shows its progress.

#fields(
  [Speed range \[rpm\]], [The lowest and the highest speed of the diagram; the tooltip of the
    label shows them in rad/s. It can be changed after a sweep: #ui("Continue") then sweeps
    the new part and keeps the speeds done. Default: from 0 to the speed of the Complex
    Frequency step, or to 10000 rpm when the
    analysis stands still. Standstill itself is not
    computed, as CalculiX merges the coinciding modes there; a range from 0 starts at a fiftieth
    of the highest speed.],
  [Maximum runs], [Number of runs after which the sweep stops by itself (default 40);
    0 runs until #ui("Stop") is pressed.],
  [Engine orders], [The excitation lines $f = "order" times "speed"$ drawn into the diagram, as
    a comma-separated list such as `1, 2, 3`. They can be changed after the sweep.],
  [Frequency axis \[Hz\]], [Upper end of the frequency axis; 0 (default) fits the axis to
    the modes. A limit zooms in on the low frequencies, where the order lines of a stiff rotor
    run close to the speed axis. It can be changed at any time, also during the sweep.],
  [Frame of reference], [#ui("Rotating") (default) shows the frequencies as the solver
    computes them, in the frame turning with the rotor. #ui("Stationary") shows what an
    observer at rest sees, see @campbell-frame. It is a view of the same sweep and can be
    changed at any time.],
)

#ui("Start Sweep") sets the Complex Frequency step to every speed, runs the solver in
a folder named after the block in that of the project in the work directory (@settings) and takes the complex frequencies of the
last Complex Frequency step and the whirl direction of every mode from its results. As many speeds run at the same time as #ui("Parallel analyses") of
the solver settings allows. The files of a speed are deleted once its frequencies are read; those
of a failed speed stay for a look.

The sweep needs no step size. It starts with eight speeds spread evenly over the range and then
keeps adding speeds where the curves bend most: between the two speeds whose curves deviate most
from a straight line, weighted with the width of the gap, so that a resolved curve lets the
speeds spread over the rest of the range. The diagram grows with every speed. The sweep ends at
#ui("Maximum runs"), at #ui("Stop"), which cancels the running speeds and keeps the finished ones,
or when no gap is wider than a five-hundredth of the range. #ui("Pause") lets the running speeds
finish and starts no new ones; #ui("Continue") goes on from there. #ui("Continue") also goes on
with a finished or stopped sweep: another #ui("Maximum runs") runs, as long as the model has not
changed since; after a change, #ui("Start Sweep") begins anew.

The speed range can change between the runs, also after the sweep finished or while it is
paused. When the new range reaches beyond the speeds done, as from 30000 to 40000 rpm, the panel
says so and #ui("Continue") first spreads a coarse grid over the new part, in proportion to its
width, and then refines between all speeds as before. The speeds done stay, also those outside a
narrower range; the order lines grow with the range. A narrower range within the speeds done
only limits where new speeds go, like a marked range.

To look closer at part of the diagram, hold #key("Shift") and drag across it: the marked speed
range is shaded and named #ui("Refine range"), and new speeds go only there, down to a
five-hundredth of the marked range. Mark a range while the sweep runs or is paused, or after it
finished, and press #ui("Continue"); the sweep again picks the speeds where the curves bend most
within the range. #ui("Clear Range") under the diagram lets new speeds go anywhere again. Gaps
next to a speed the solver could not solve are not refined further. The messages of the runs
appear under #ui("Output") in the panel. When fewer than two speeds give complex frequencies, the
window says so in place of the diagram; the output names the error of each failed speed.

The diagram has one curve per mode and one dashed grey line per engine order, which starts at
0 rpm even though the sweep leaves out standstill. The modes are followed from speed to speed by
their course, not by their rank among the frequencies: a mode joins the curve it continues best,
so curves that cross keep their colour, and a curve that falls to zero and rises again (in the
rotating frame of reference, the forward whirl at its critical speed) stays one curve. Roots
CalculiX finds at fewer than three neighbouring speeds, such as a spurious root near 0 Hz, are
drawn as grey crosses named #ui("Unmatched roots") instead of tearing the curves; growing modes,
whose eigenvalue has an imaginary part of weight, are drawn as red diamonds named
#ui("Unstable"). A mode that whirls the same way at every speed carries its direction in the
legend, F (forward) or B (backward) as the solver reports it, and its number of nodal
diameters, such as ND 2, where the solver finds a clear one in the mode shape.

The diagram can be moved and zoomed like the plots of egui: drag it to pan, scroll to zoom,
drag along an axis to zoom only that axis, and double-click to fit the view to the curves again.
A click on a legend entry hides or shows that mode or order line. Hovering a point shows its
curve, speed and frequency.

#screenshot("campbell.png", [Campbell diagram of a spinning cantilever: a sweep up to 30000 rpm,
continued after the speed range was widened to 40000 rpm, which brought the 1x critical speed
into the diagram; the speed of 30000 rpm is selected])

Where a mode crosses an order line, the rotor is
excited at one of its eigenfrequencies: these crossings are listed under #ui("Critical speeds")
in the panel with the order, the mode, its whirl and nodal diameter, the speed in rpm and the
frequency. For a disc, mind the frame of reference (@campbell-frame): the excitation by a
load at rest is the crossing of a mode with the order line of its own number of nodal
diameters, not with 1x.

A click on a point of the diagram selects the computed speed nearest to it, a click on the speed
of a critical speed selects that speed. A vertical line marks it in the diagram, and the points of
the modes there are ringed. #ui("Selected speed") in the panel shows it in rpm and rad/s with the
frequency of every mode at it (none for a critical speed between the computed ones).
#ui("Set Analysis Speed") sets the speed of the Complex Frequency step of the analysis to that
speed (with several such steps the fastest to exactly it and the others in proportion) and notes
it in the output: solve the analysis then to see all its results at that speed, such as the
whirling mode shapes, which the sweep does not keep. The diagram stays up to date, as it leaves the speed of the
analysis out. #ui("Clear") drops the selection. #ui("Copy Table")
copies the frequencies of all modes at all speeds as tab-separated text. #ui("Export CSV ...")
saves the same table as a CSV file, with a Save As dialog: one row per speed with the speed in
rad/s and rpm, the frequency in Hz and the whirl (F, B or empty) of every curve, the other roots
CalculiX found at that speed (unmatched and unstable, separated by spaces), and the frequency of
every order line. Numbers use a decimal point and the cells are separated by commas.
#ui("Export JSON ...") saves the same table as JSON (@table-json).

=== Rotating and stationary frame <campbell-frame>

The solver computes the Coriolis modes in the frame of reference that turns with the rotor.
The frequencies of the diagram are therefore those an observer rotating with the shaft would
see: a pair of bending modes of a shaft splits into a rising and a falling branch with the
speed, while the bending modes of a disc, such as a saw blade or a CD, hardly split at all,
because the Coriolis forces do not act on a motion along the axis. A microphone, a sensor on
the housing or a camera sees something else. A mode whose shape has $n$ nodal diameters about
the axis (two for the saddle of a disc, one for the bending of a shaft, none for an umbrella
mode) is in the stationary frame a pair of travelling waves: the forward wave at
$f + n dot "speed"$, running with the rotation, and the backward wave at $|f - n dot "speed"|$,
running against it. Where the backward wave comes to rest, at $f = n dot "speed"$, the rotor
carries a standing wave that any load at rest excites, such as the air around a disc: the
classic critical speed of a disc, which in the rotating frame is the crossing of the mode
with the order line of its own $n$.

The solver finds the number of nodal diameters of every complex mode from its shape, as the
circumferential harmonic about the rotation axis that carries most of the mode, and writes
it next to the whirl direction. A rotor that is not axisymmetric, or a mode spread over
several harmonics, gets none; such a mode is named without ND and stays out of the
stationary frame, which the line under the diagram says.

#ui("Frame of reference") set to #ui("Stationary") draws every mode with a nodal diameter as
its forward wave (solid) and, for $n > 0$, its backward wave (dashed) in the colour of the
mode, and the critical speeds become the crossings of the waves with the order lines plus
the speeds where a backward wave comes to rest, listed with the order #ui("standing").
#ui("Selected speed") then shows both waves of every mode. The table and the exports keep
the solver's frequencies with a column of the nodal diameters.

#screenshot("campbell-stationary.png", [The diagram of a spinning disc in the stationary
frame: the saddle mode (ND 2) as its forward wave rising steeply and its backward wave coming
to rest at the critical speed, the umbrella mode (ND 0) unsplit])

#note[The frequencies and whirl directions the solver reports, in the results of the analysis
and in the rotating frame of the diagram, are those of the rotating frame of reference; only
the diagram converts them.]
