#import "../../template.typ": *

== Amplitudes <amplitudes>

An amplitude describes how a load or a prescribed value changes with time. Create it with
#menu("Steps", "Amplitude") and choose it in the #ui("Amplitude") field of a boundary
condition or load.

#screenshot("amplitude.png", [Amplitude with its curve])

#fields(
  [Time span], [#ui("Step time") (default) or #ui("Total time") (time since the start of the
    analysis).],
  [Shift time, Shift amplitude], [Shift the curve along the time or the amplitude axis.],
  [Data Points], [Pairs of time and factor. #ui("Add Row"), #ui("Remove"), #ui("Paste from
    Clipboard") (two columns, for example from a spreadsheet) and #ui("Sort by Time").],
)

The factor is interpolated linearly between the points and stays constant before the first and
after the last point. Times must not decrease. The amplitude is written as `*AMPLITUDE`.

An amplitude that no boundary condition, load or defined field uses has no effect. The model check
(@model-check) warns of it in the tree, so that you do not wonder why nothing moves.

#screenshot("model-check-amplitude.png", [Warning for an amplitude that nothing refers to])

With #ui("Default") instead of an amplitude, CalculiX ramps the value up linearly over a static
step and applies it at once in a heat transfer step.
