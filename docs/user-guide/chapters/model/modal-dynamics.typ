#import "../../template.typ": *

== Modal Dynamics and Steady State Dynamics steps <modal-dynamics>

Both steps superpose the eigenmodes of the structure instead of integrating the full model:
#ui("Modal Dynamics (from stored modes)") computes the response over time (`*MODAL DYNAMIC`),
#ui("Steady State Dynamics (frequency response)") the amplitude at every frequency of a
harmonic excitation (`*STEADY STATE DYNAMICS`). Both need a #ui("Frequency") step with
#ui("Store matrices and eigenmodes") before them, with the same boundary conditions; the model
check (@model-check) reports a missing one as "No stored eigenmodes". Create the steps with
#menu("Model", "Create Step ...").

#screenshot("modal-dynamics-step.png", [Modal Dynamics step with a constant damping ratio])

=== Modal Dynamics

#fields(
  [Steady state (until the response repeats)], [Integrates until the response repeats itself
    within #ui("Relative error") (`STEADY STATE`), instead of over the time period.],
  [Time increment], [Fixed increment of the response (default 0.1).],
  [Time period], [Length of the step (default 1).],
  [Max. increments], [Largest number of increments (default 100). Nodalis raises it so that the
    time period fits.],
)

Loads of a modal dynamics step need an #ui("Amplitude") (@amplitudes); a stepped amplitude
applies the load at once. Only loads and the boundary conditions of the frequency step act;
CalculiX cannot change supports in a modal step.

=== Steady State Dynamics

#screenshot("steady-state-step.png", [Steady State Dynamics step])

#fields(
  [Harmonic excitation], [The loads vary as sine waves with the frequency swept over the range
    (default on). Switched off, the loads are periodic between #ui("Period start") and
    #ui("Period end") and expanded into #ui("Fourier terms") (`HARMONIC=NO`).],
  [Lower / Upper frequency], [The frequency range of the sweep (default 0 to 10 Hz).],
  [Data points], [Frequencies evaluated between two eigenfrequencies (default 20).],
  [Bias], [Crowds the frequencies towards the eigenfrequencies (default 3); 1 spreads them
    evenly.],
)

The results have one increment per frequency. The displacements are written as real part (`DISP`)
and imaginary part (`DISPI`); the magnitude of a node is the length of both together.

=== Modal damping

Both steps have a #ui("Modal damping") setting, written as `*MODAL DAMPING`:

#fields(
  [Off], [No damping.],
  [Constant ratio], [One viscous #ui("Damping ratio") (damping over critical damping) for all
    modes, for example 0.02.],
  [Ratio per mode range], [A damping ratio for each range of modes (#ui("Modes, ratio"), from
    the lowest to the highest mode number). #ui("Add range") adds a row, #ui("x") removes
    one.],
  [Rayleigh], [Damping from the mass and stiffness matrices with #ui("Alpha (mass)") and
    #ui("Beta (stiffness)"), as for a dynamic step (@dynamic-step).],
)
