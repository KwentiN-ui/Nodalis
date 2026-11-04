#import "../../template.typ": *

== Dynamic step <dynamic-step>

A #ui("Dynamic (time integration)") step computes the motion of the model over time
(`*DYNAMIC`): the response to a suddenly applied load, an impact or an initial velocity
(@initial-conditions). Create it with #menu("Steps", "Step") and choose the type in
the dialog.

#screenshot("dynamic-step.png", [Dynamic step])

#fields(
  [Procedure], [Implicit or explicit integration for the structure and for a fluid, written as
    `EXPLICIT=`: #ui("Implicit / Implicit") (default), #ui("Implicit / Explicit"),
    #ui("Explicit / Implicit") or #ui("Explicit / Explicit").],
  [Alpha (HHT)], [Numerical damping of the Hilber-Hughes-Taylor integration (`ALPHA=`), between
    −1/3 and 0; −0.05 is the default of CalculiX and damps high frequencies a little.],
  [Nonlinear geometry, Incrementation, Equation solver], [As in a static step (@steps,
    @equation-solver); the effective matrix of the time integration is positive definite,
    so the iterative solver takes it as well. A new dynamic
    step starts with #ui("Automatic") incrementation and an initial increment of 0.01; choose
    the increment from the highest frequency you want to resolve, about a tenth to a twentieth
    of its period.],
  [Rayleigh damping], [Damping of the whole model, written as `*DAMPING, ALPHA=, BETA=` in the
    step. #ui("Alpha (mass)") has the unit of a frequency, #ui("Beta
    (stiffness)") that of a time. For a damping ratio ζ at the circular frequency ω,
    α = 2ζω damps the low modes and β = 2ζ/ω the high ones.],
)

A dynamic step takes the same boundary conditions and loads as a static step, and materials
need a density. Loads act from the start of the step with the amplitude of the step; a load that
should be applied at once needs a stepped #ui("Amplitude") (@amplitudes) with the value 1 from
time 0, otherwise CalculiX ramps it over the step. The results have one increment per written
time step; the #ui("Step increments") animation (@animation) plays them, and history outputs
(@history-outputs) show the motion of selected nodes over time.
