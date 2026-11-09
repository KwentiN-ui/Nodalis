#import "../../template.typ": *

== Steady State Dynamics step <modal-dynamics>

#ui("Steady State Dynamics (frequency response)") superposes the eigenmodes of the structure
instead of integrating the full model and computes the amplitude at every frequency of a
harmonic excitation (`*STEADY STATE DYNAMICS`). The step computes the eigenmodes itself, with
its own boundary conditions, in a frequency step the solver runs right before it; only the
response is kept. #ui("Base state") in its #ui("Start") section (@step-handover) chooses
whether the modes are those of the unloaded model or of the state the step before leaves
(a preloaded structure). The step leaves the state as it found it. An #ui("Eigenmodes") link
from a frequency step of another analysis gives it those modes instead (@results-links).
Create the step with #menu("Steps", "Step").

The response over time from the eigenmodes is a #ui("Dynamic") step with the method
#ui("Modal superposition") (@dynamic-modal), which computes its eigenmodes itself.

#screenshot("steady-state-step.png", [Steady State Dynamics step])

#screenshot("ssd-start.png", [Start section of a Steady State Dynamics step on the preloaded state: the step computes its own eigenmodes and passes nothing on])

#fields(
  [Modes], [The number of eigenmodes the response is superposed from (default 20). The highest
    of them should lie well above the upper frequency. Not shown when an Eigenmodes link gives
    the modes.],
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

The step has a #ui("Modal damping") setting, written as `*MODAL DAMPING`:

#fields(
  [Off], [No damping.],
  [Constant ratio], [One viscous #ui("Damping ratio") (damping over critical damping) for all
    modes, for example 0.02.],
  [Ratio per mode range], [A damping ratio for each range of modes (#ui("Modes, ratio"), from
    the lowest to the highest mode number). #ui("Add range") adds a row, #ui("x") removes
    one.],
  [Rayleigh], [Damping from the mass and stiffness matrices with #ui("Alpha (mass)") and
    #ui("Beta (stiffness)"), as for a Dynamic step (@dynamic-step).],
)
