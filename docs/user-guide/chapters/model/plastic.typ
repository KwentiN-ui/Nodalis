#import "../../template.typ": *

=== Plasticity <plasticity>

#ui("Plasticity") in the material dialog adds von Mises plasticity with a tabular hardening
curve. It needs #ui("Elasticity") on the same
material.

#screenshot("material-plastic.png", [Material with a hardening curve])

#fields(
  [Hardening], [#ui("Isotropic") (default), #ui("Kinematic") or #ui("Combined"), written as
    `*PLASTIC, HARDENING=`.],
  [Hardening curve], [One row per point of the curve: the yield stress, the plastic strain at
    which it is reached, and the temperature. The first row has plastic strain 0 and the
    initial yield stress; the strain grows from row to row. Rows at a higher temperature start
    at plastic strain 0 again. #ui("Add row") continues the curve, #ui("Remove") deletes a
    row.],
)

A single row gives an ideally plastic material. Beyond the last point CalculiX keeps the last
yield stress. The model check (@model-check) reports a curve that does not start at strain 0, a
strain that does not grow, or plasticity without elasticity as "Invalid plasticity".

Plastic strains need a nonlinear analysis: switch on #ui("Nonlinear geometry") or use
automatic incrementation with several increments in the static step (@steps). With a plastic
material in the model the steps write the equivalent plastic strain `PE` on their own, which
shows where the material yields.
