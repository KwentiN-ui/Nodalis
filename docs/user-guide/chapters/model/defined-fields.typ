#import "../../template.typ": *

== Defined fields <defined-fields>

A defined field prescribes the temperature of the model in a mechanical step, for thermal
expansion or temperature-dependent materials. Create
it with #ui("Defined Field") on the #ui("Steps") tab, in the active step. Heat transfer, coupled and uncoupled steps solve for the temperatures and take no defined
fields.

#screenshot("defined-field.png", [Defined temperature field by value])

#fields(
  [Type], [#ui("Temperature") is the only type.],
  [Source], [#ui("By value"): one temperature on the nodes of a region, with an optional
    #ui("Amplitude") (@amplitudes) that scales it over the step and an optional
    #ui("Distribution") (@distributions) that makes it vary over the region. #ui("From result file"): the
    temperatures of all nodes from the `.frd` file of a heat transfer analysis on the same mesh,
    with the #ui("Step") of that file to read (counted from 1).],
  [Temperature, Region], [The value (default 20) and the nodes it applies to: a selection,
    parts, a node set or a surface.],
  [Result file], [Picked with #ui("..."); the file name must be one CalculiX can read (letters,
    digits, `_` and `-`).],
)

Written as `*TEMPERATURE` with the node set and the value, or as `*TEMPERATURE, FILE=...,
BSTEP=...`. When the analysis starts, a result file is copied next to the input file, where
CalculiX looks for it; the monitor lists the copied files.

A model with a defined field needs an #ui("Initial Condition") of type #ui("Temperature")
(@initial-conditions), otherwise CalculiX refuses the `*TEMPERATURE` keyword; the model check
(@model-check) reports a missing one. The temperatures act through the #ui("Thermal expansion")
of the materials (@materials) with their reference temperature.

#note[The usual thermo-mechanical workflow: run a heat transfer analysis, then a static analysis
with a defined field that reads its `.frd` file, in the same model or a copy of it.]
