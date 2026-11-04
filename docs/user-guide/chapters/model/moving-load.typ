#import "../../template.typ": *

== Moving load <moving-load>

A #ui("Moving Load") drives a group of wheels over a structure: a crane over its runway beam, a
trolley over a crane bridge, a vehicle over a bridge deck. CalculiX has no such load, so Nodalis
writes one static step per position, each with the wheel forces at that position. No script is
needed. Create it with #ui("Load") on the #ui("Steps") tab, with a static step
active, and pick the type #ui("Moving Load").

#screenshot("moving-load-dialog.png", [A crane with two wheels 3000 apart driving over a box
  girder from -1500 to 13500 in 31 positions])

#fields(
  [Region], [The faces the wheels run on, e.g. the top of the rail or the deck. Only these faces
    take the wheel forces.],
  [Coordinate system], [The system the path is measured in (@features). Its origin is the zero
    of the path, typically the start of the beam on the rail axis. #ui("Global") uses the global
    axes.],
  [Path along], [The axis of the system the wheels travel along.],
  [Across the path], [The axis across the path in the running surface. The contact patch width
    and the wheel offsets #ui("Across") are measured along it. The remaining axis is the normal of
    the running surface; its height comes from the selected faces. Example: a crane rail with the
    local x axis along the rail and y pointing across it to the trolley: #ui("Path along") X,
    #ui("Across the path") Y. 3D models only.],
  [Start, End], [Position of the vehicle's reference point along the path in the first and in
    the last step.],
  [Positions], [Number of positions, spread evenly from start to end. Each one is a static step
    of its own.],
  [Contact patch length, width], [Size of the rectangle each wheel presses on. The wheel force
    is spread evenly over the faces inside it.],
  [Wheels], [One row per wheel: its offset #ui("Along") and #ui("Across") the path from the
    reference point and its force `F1`, `F2`, `F3` in *global* directions. #ui("Add wheel")
    appends a row, #ui("Mirror") adds the wheels mirrored about the reference point, which
    turns one wheel at +2100 into a symmetric pair at ±2100.],
)

Below the table the dialog shows the total force of all wheels. In the 3D view the wheels are
drawn at the first and at the last position; here one wheel is off the girder at the start and at
the end.

#screenshot("moving-load-view.png", [The wheels of the selected moving load at its first and last
  position])

#heading(level: 3, outlined: false)[Wheels at the ends of the structure]

A wheel whose contact patch is off the selected faces carries no force; one that is partly off
carries the part of its force that falls on the faces. So a crane can drive over the runway beam
from one end to the other and off it: put #ui("Start") and #ui("End") before and behind the
beam. For a trolley that stays on its bridge choose them so that every wheel stays on the faces.
The model check warns with "Wheel off the surface" when a wheel leaves the faces at some
positions; this is a reminder, not an error.

#heading(level: 3, outlined: false)[Input file and results]

The step is written once per position (`** Moving load position 3 of 31, Crane at -500`),
everything else in the step stays the same in all of them. The wheel forces become `*CLOAD` on
the nodes of the faces under each patch, with the shares of a uniform pressure on the covered
part of each face (as for a #ui("Force"), @loads). The results have one step per
position: step through them, or animate them with the animation type #ui("Steps")
(@animation) to see the load travel. The animation keeps one deformation scale for all
positions.

#screenshot("moving-load-result.png", [Deflection with the crane in the middle of the girder,
  played as an animation of the steps])

#note[Each position is a separate linear static analysis, as in the usual script of load steps.
Dynamic effects of the moving mass are not included.]
