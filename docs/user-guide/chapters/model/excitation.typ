#import "../../template.typ": *

== Excitation <excitation>

An #ui("Excitation") shakes the structure in a Dynamic step (@dynamic-step): a force that
follows a signal over the step time, to find its eigenfrequencies from the response to a
hammer blow, to sweep through its resonances or to drive it at one frequency. It is one load
with a choice of signal; create it with #ui("Load") on the #ui("Steps") tab, with a Dynamic
step active, and pick the type #ui("Excitation (impulse, sine, chirp)").

#screenshot("excitation-dialog.png", [A chirp from 5 to 60 Hz at the tip of a cantilever,
  with the plot of the signal over the step])

#fields(
  [FX, FY, FZ], [The peak force (#ui("F radial"), #ui("F tangential"), #ui("F axial") in a cylindrical system). The signal swings between it and its negative (an impulse
    only reaches it). It acts on faces, edges or points like a #ui("Force")
    (@force-region), with #ui("Apply to all nodes individually") at every node.],
  [Signal], [#ui("Impulse"), #ui("Sine") or #ui("Chirp"), see below. Switching keeps the
    start and the end.],
  [Start, End], [Step times the sine or sweep runs between; before and after, the force is
    zero. Without an #ui("End") it runs to the end of the step.],
)

The signals:

#fields(
  [Impulse], [The hammer blow, a Dirac impulse made smooth: a half sine of #ui("Width")
    from #ui("Time"). The impulse it carries is 2/π × peak force × width; the dialog shows
    it. The shorter the width, the higher the frequencies it excites (about up to 1.5/width).
    After it the structure swings freely in its eigenmodes.],
  [Sine], [One #ui("Frequency") with a #ui("Phase") in degrees (90° starts at the peak).
    Starting from rest, a sine also sets off the free vibration of the structure, which
    damping lets die away.],
  [Chirp], [A sine whose frequency sweeps from #ui("Start frequency") to #ui("End
    frequency") between start and end, #ui("Linear") (the same hertz per second) or
    #ui("Logarithmic") (the same factor per second: as long per octave at low as at high
    frequencies). Sweep slowly enough that each resonance has a few periods to build up.],
)

The solver evaluates the signal exactly at every time step, in implicit and explicit
integration. The automatic time step (@dynamic-step) takes 20 time steps per period of the
highest frequency of a sine or sweep and 10 per width of an impulse; the dialog of the step
shows the time step it leads to. The plot in the dialog shows the factor of the peak force
over the step.

An excitation has no amplitude: the signal is its time behaviour. It acts only in a Dynamic
step; in other steps the model check reports #ui("Excitation outside a Dynamic step") and
the step leaves it out. A signal the solver cannot evaluate (no width, no frequency, an end
before the start, a start after the end of the step) is reported as #ui("Invalid excitation
signal").

To see the response over time, add a history output of the displacement of a node and plot
it over the time (@history-outputs); during a sweep the eigenfrequencies show as the peaks of the response.
