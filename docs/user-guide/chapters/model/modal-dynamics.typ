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

=== Results of a Steady State Dynamics step <steady-state-results>

The results have one increment per frequency: #ui("Frequency") in the ribbon and the timeline
step through them. At each frequency every point of the structure vibrates harmonically,
`x(t) = A cos(Ωt − θ)`, with an amplitude `A` and a phase lag `θ` behind the load. The results
tree shows this vibration:

#fields(
  [`DISP_AMPLITUDE`, `STRESS_AMPLITUDE`, ...], [The amplitude `A` of every component, the
    largest value over the period. `ALL` of the displacements and `MISES` of the stresses
    (`EQUIVALENT` of the strains) are the largest magnitude and equivalent value the period
    reaches, not the magnitude of the component amplitudes: these peak at different
    moments. Plotted over the frequencies (@history-outputs), the amplitude of a node gives
    the frequency response curve.],
  [`DISP_PHASE`, `STRESS_PHASE`, ...], [The phase lag `θ` of every component behind the load,
    in degrees from −180 to 180. Below a resonance it is close to 0, above it close to 180;
    in between, damping makes it pass through 90. Components that hardly move get 0.],
  [`DISP`, `STRESS`, `TOSTRAIN`, `FORC`], [The values at the moment of the period that
    #ui("Phase angle") picks: `Ωt` in degrees, 0 is the moment the load peaks. The deformed
    shape follows it as well.],
)

#ui("Phase angle") is the slider at the top of the #ui("Frequency") panel. #ui("Animate")
plays one period of the vibration in a loop: the deformation and the values at the moment
of the period swing as the structure does; the amplitude and phase fields stay as they are.

#screenshot("steady-state-results.png", [Results of a Steady State Dynamics step: the displacement amplitude near the first bending mode, with the phase angle in the Frequency panel])

The solver writes the response as complex numbers, as CalculiX does: the real part in `DISP`,
`STRESS`, ... and the imaginary part in `DISPI`, `STRESSI`, `TOSTRAII` and `FORCI`
(`DISP` is the value at the phase angle 0, `DISPI` the negative of that at 90 degrees). The
results tree leaves the imaginary parts out, since the amplitudes, phases and the phase angle
show the same. Without #ui("Modal damping") the response is in phase or in antiphase with the
load, the imaginary parts and the phase lags other than 0 and 180 are zero, and the amplitude
at an eigenfrequency is unbounded: give the step damping for a realistic response.

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
