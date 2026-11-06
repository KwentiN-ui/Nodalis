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
    frequencies. A model that can move freely as a whole or in parts (a pendulum on a
    joint, a free body) gets no mass-proportional part from #ui("Automatic"), since that
    would brake the free motion like a viscous fluid. With #ui("Large deformations") the
    stiffness part takes the current stiffness, so a part that turns far is damped only
    where it deforms. #ui("Rayleigh coefficients"): α (mass, unit of a frequency) and β
    (stiffness, unit of a time) directly.],
  [Large deformations], [Off, the motion is linear: displacements and rotations stay
    small. On, every time step iterates on the deformed geometry (see below).],
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
points. Materials need a density. Contact is not available in a dynamic step.

The model check warns when the frequency limit is higher than the frames can show
(#ui("Too few frames for the frequency limit")) and when all points of an amplitude the
step follows lie after its end (#ui("Amplitude starts after the step")).

=== Large deformations

Without #ui("Large deformations") the step is linear, which is right for everything that
vibrates with small amplitudes: impacts on parts, load jumps, pulses, decaying vibrations.
A body that turns far is wrong there: a rotation is taken as a displacement along the
tangent, so a swinging part grows longer and longer, and gravity has no lever that
changes with the angle. A pendulum, a mechanism, a flap that folds over or a beam that
bends far need #ui("Large deformations").

With it the solver iterates in every time step on the deformed geometry, and rigid bodies
(@rigid-body) turn by finite angles, e.g. the reference point a #ui("Displacement/Rotation")
condition on an edge or face creates. Where the motion changes faster than the time step
resolves, or the iterations converge slowly, the solver computes a time step in up to 64
parts and says so in the output ("increment 3 is computed in 8 parts"); the frames stay
where they are. Materials with plasticity (@plasticity) make the step nonlinear in the
same way, with or without the switch.

#screenshot("dynamic-pendulum.png", [A plate hinged at its left edge, swinging under its own weight with Large deformations])

=== What the solver reports

At the start of the step the solver output shows the time step and the highest frequency it
resolves ("time step 2.5e-5 s resolves vibrations up to 4000 Hz"). With a damping ratio over
the automatic range it first computes the lowest eigenfrequency of the supported model and
reports the fit: "damping 2 % fitted between 812 and 4000 Hz (alpha = ..., beta = ...)". A
free structure is fitted at its first elastic mode. If the lowest eigenfrequency is not below
the resolved frequency (a very stiff model or a coarse time step), the range starts a decade
below it and the output warns.

=== Results

The results have one increment per frame; the #ui("Step") animation (@animation) plays them.
A new Dynamic step writes the displacements, velocities `V`, accelerations `A` and reaction
forces at the nodes and stresses and strains in the elements.

It also gets a history output #ui("Energies") of the whole model at every time step:

#fields(
  [ALLIE], [Internal (strain) energy ½ uᵀKu, including a preload of earlier steps.],
  [ALLKE], [Kinetic energy ½ vᵀMv.],
  [ALLWK], [Work of the loads since the start of the step.],
  [ALLVD], [Energy the damping dissipated since the start of the step.],
)

#screenshot("history-energies.png", [Energies history output of a Dynamic step])

With large deformations or plasticity ALLIE is the work the internal forces did, which
includes the energy plastic deformation dissipated.

ALLIE + ALLKE + ALLVD − ALLWK stays at its starting value up to the small loss of the
numerical damping: the energy balance shows whether the time step resolves the motion.
Prescribed displacements that change over time do work that ALLWK does not count. Other
history outputs (@history-outputs) show the motion of selected nodes at every time step.
