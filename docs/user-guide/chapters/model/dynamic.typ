#import "../../template.typ": *

== Dynamic step <dynamic-step>

A #ui("Dynamic (time integration)") step computes the motion of the model over time: the
response to a suddenly applied load, a pulse, a vibration or an initial velocity
(@initial-conditions). Create it with #menu("Steps", "Step") and choose the type in the
dialog. You decide how long the step lasts, how many results you want to see and how the
structure is damped; Nodalis chooses the time step and the integration from that.

#screenshot("dynamic-step.png", [Dynamic step])

#fields(
  [Duration], [The time the step covers, from 0 (step time).],
  [Frames], [The number of result frames kept, evenly spaced over the duration; the last one
    at its end. The dialog shows the time between two frames. 200 frames are plenty for an
    animation; for the history of a fast vibration use at least two frames per period.],
  [Damping], [#ui("None"): undamped, only the numerical damping of the integration removes
    energy. #ui("Damping ratio"): the viscous damping ratio of the structure (0.02 = 2 % of
    critical damping, typical for steel structures), applied as Rayleigh damping
    C = α M + β K that meets the ratio exactly at both ends of a #ui("Frequency range")
    and is lower between them. #ui("Automatic") fits it from the lowest eigenfrequency of
    the model to the highest frequency the time step resolves; #ui("Between") takes two
    frequencies. #ui("Rayleigh coefficients"): α (mass, unit of a frequency) and β
    (stiffness, unit of a time) directly.],
  [Starts from], [What the step starts from, for information: rest, the state at the end of
    the step before it, or an initial velocity (@initial-conditions).],
)

#ui("Advanced settings") are rarely needed:

#fields(
  [Time step], [#ui("Automatic") takes the shortest of: the time between two frames, a tenth
    of the shortest segment of an amplitude the loads follow (@amplitudes), and the
    frequency limit below. #ui("At most") takes your value. Either way the time step is
    shortened so that the frames fall exactly on time steps; the line under the settings
    shows the time step used and the number of increments.],
  [Max. frequency], [The highest frequency the integration must follow, with 20
    time steps per period. Set it when the structure vibrates faster than the frames show,
    e.g. to see the stresses of an impact correctly in each frame.],
  [Numerical damping], [Damping of the time integration (Hilber-Hughes-Taylor) that removes
    the high frequencies the time step cannot resolve: #ui("Normal") (slight),
    #ui("None") (conserves the energy) or #ui("Strong").],
  [Equation solver], [As in a static step (@equation-solver).],
)

Loads act at once from the start of the step. Give a load an #ui("Amplitude")
(@amplitudes) for a ramp, a pulse or a vibration; the automatic time step follows its
points. Materials need a density. The integration is linear: no large deformations, no
contact and no plasticity in a dynamic step.

The model check warns when the frequency limit is higher than the frames can show
(#ui("Too few frames for the frequency limit")) and when all points of an amplitude the
step follows lie after its end (#ui("Amplitude starts after the step")).

The results have one increment per frame; the #ui("Step") animation (@animation) plays
them, and history outputs (@history-outputs) show the motion of selected nodes at every
time step.
