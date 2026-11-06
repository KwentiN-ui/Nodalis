#import "../../template.typ": *

== Dynamic step <dynamic-step>

A #ui("Dynamic (time integration)") step computes the motion of the model over time: the
response to a suddenly applied load, a pulse, a vibration or an initial velocity
(@initial-conditions). Create it with #menu("Steps", "Step") and choose the type in the
dialog. You decide how long the step lasts, how many results you want to see and how the
structure is damped; Nodalis chooses the time step and the integration from that.

#screenshot("dynamic-step.png", [Dynamic step])

#fields(
  [Method], [#ui("Direct integration") solves all equations of the model in every time step
    and takes contact, large deformations and plasticity. #ui("Modal superposition")
    computes the lowest eigenmodes of the supported model first and integrates each of
    them exactly; every time step then costs almost nothing. For linear models only, see
    below.],
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
points. Materials need a density. Contact regions and contact pairs (@contacts) act in a
dynamic step too, see below.

The model check warns when the frequency limit is higher than the frames can show
(#ui("Too few frames for the frequency limit")) and when all points of an amplitude the
step follows lie after its end (#ui("Amplitude starts after the step")).

=== Modal superposition <dynamic-modal>

With #ui("Modal superposition") the solver first computes the lowest #ui("Modes")
eigenmodes of the model with the supports of the step (as a Frequency step with
#ui("Storage") would), then follows the motion in these modes: every
mode is a single oscillator that is integrated exactly, with the loads linear between the
time steps. A time step costs a few microseconds instead of the solution of the whole
equation system, so long durations, fine time steps and large linear models come cheap,
and the memory stays that of the frames.

#screenshot("dynamic-modal.png", [Dynamic step with modal superposition])

The response is exact in the modes it has and misses what lies above them. Choose the
number of modes so that the highest one lies well above the frequencies the loads excite
(a pulse of 1 ms excites up to about 1 kHz); the solver output names the highest mode
("the highest mode lies at 4812 Hz"). 30 modes are plenty for a part under a load jump or a
slow pulse; a short impact or stresses near a load point need more. Local stresses right
at a concentrated load converge slowly with the number of modes; use direct integration
where they matter.

The method is linear: contact, #ui("Large deformations") and plasticity are not
followed (the model check warns). A #ui("Damping ratio") applies to every mode directly
(no Rayleigh fit, no frequency range); #ui("Rayleigh coefficients") give each mode the
ratio α/(2ω) + βω/2. Initial velocities and the state of the step before are taken
up through the modes. The energies and the velocities, accelerations and reaction forces
are written as with direct integration. Prescribed displacements must be zero (base
motion is not supported yet).

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

=== Contact <dynamic-contact>

Contact (node to surface, surface to surface or mortar, frictionless or with friction, in 3D
and in 2D) works in a Dynamic step as in a static one: a part that falls onto another, a bar that
hits a wall, a block that slides on a plate and stops by friction. Like #ui("Large
deformations"), contact makes the step iterate in every time step.

The time integration treats the contact so that impacts stay physical:

- An impact lasts far shorter than a usual time step (a steel part bouncing off a plate stays
  in contact for microseconds). When a contact closes or opens within a time step, the solver
  checks the energy the contact springs take and return; if the time step is too coarse for
  the impact it is halved around the impact, down to a few nanoseconds where needed, and
  the frames stay where they are. A dropped part therefore never bounces higher than it
  fell from.
- The contact springs are damped with a fraction of the time step. This removes the
  chatter of the nodes on the stiff contact springs (the contact force jumping between zero
  and several times its value from one time step to the next) and leaves the slower motion
  alone; the energy it takes is part of ALLVD and stays at a few percent of an impact.
- Contact carries pressure only: node to surface contact has no tension before the parts
  touch (unlike in a static step, where it follows CalculiX).

#screenshot("dynamic-contact.svg", [Left: a steel block dropped from 1 mm onto a plate with
  time steps of 1 ms; the solver refines the impact of a few microseconds, and the block
  bounces without gaining energy (part of it stays in the vibrations of the block). Right: a
  steel bar hitting a wall at 1 m/s presses with ρ c v₀ A for the time 2L/c a wave takes
  through the bar and back, then leaves at 1 m/s])

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
includes the energy plastic deformation dissipated. With contact it also holds the energy of the contact
springs and the work of the friction; the damping of the contact springs is part of ALLVD.

ALLIE + ALLKE + ALLVD − ALLWK stays at its starting value up to the small loss of the
numerical damping: the energy balance shows whether the time step resolves the motion.
Prescribed displacements that change over time do work that ALLWK does not count. Other
history outputs (@history-outputs) show the motion of selected nodes at every time step.
