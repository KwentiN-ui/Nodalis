#import "../../template.typ": *

== Dynamic step <dynamic-step>

A #ui("Dynamic (time integration)") step computes the motion of the model over time: the
response to a suddenly applied load, a pulse, a vibration or an initial velocity
(@initial-conditions). Create it with #menu("Steps", "Step") and choose the type in the
dialog. You decide how long the step lasts, how many results you want to see and how the
structure is damped; Nodalis chooses the time step and the integration from that.

#screenshot("dynamic-step.png", [Dynamic step])

#fields(
  [Method], [#ui("Implicit (direct integration)") solves all equations of the model in every
    time step and takes contact, large deformations and plasticity; its time steps can be
    long. #ui("Explicit") takes thousands of tiny time steps without solving equations: for
    impacts, crashes and drop tests, see @dynamic-explicit. #ui("Automatic
    (implicit/explicit)") switches between the two within the step, see
    @dynamic-automatic. #ui("Modal superposition")
    computes the lowest eigenmodes of the supported model first and integrates each of
    them exactly; every time step then costs almost nothing. For linear models only, see
    below.],
  [Duration], [The time the step covers, from 0 (step time).],
  [Frames], [The number of result frames kept, evenly spaced over the duration; the last one
    at its end. The dialog shows the time between two frames. 200 frames are plenty for an
    animation; for the history of a fast vibration use at least two frames per period.],
  [Frames at contact events], [On, the results also get a frame whenever a contact closes
    or opens and at the sub-steps the solver takes through an impact, between the evenly
    spaced frames (at most 32 per time step). The impact force and the moment of touch
    then show in the history and on the timeline even when the time steps are far longer
    than the impact. With explicit integration a frame follows each closing or opening
    contact within a few time steps (at most 32 between two evenly spaced frames).],
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

=== Explicit integration <dynamic-explicit>

With #ui("Explicit") the solver integrates the motion with central differences and a lumped
(diagonal) mass matrix. A time step then costs the internal forces of the elements and a
division per node: there is no equation system to solve and nothing that can fail to
converge. The price is the length of the time step: it must stay below the time a
compression wave needs to cross the smallest element, typically 10⁻⁷ to 10⁻⁶ s for steel
parts meshed in millimetres. An explicit step therefore suits short, violent events (an
impact, a crash, a drop with buckling and plasticity), which the implicit integration can
only follow with tiny time steps anyway, and not slow motions or long vibrations.

#screenshot("dynamic-explicit.png", [Dynamic step with explicit integration and the estimate of
  its stable time step])

The dialog shows an estimate of the stable time step from the mesh and the materials (the
shortest distance between two nodes of an element over the wave speed √(E(1−ν)/((1+ν)(1−2ν)ρ)),
with the node spacing through the thickness of shells) and the number of time steps it means
for the duration. Above a million steps it suggests a shorter duration. The solver computes
the exact stable time step at the start of the step from the highest eigenfrequency of every
element and takes 90 % of it; the output names the element that limits it:

```
explicit: stable time step 4.31e-7 s (element 1534), about 6.96e+5 time steps for 0.3 s
```

#ui("Frames") work as with implicit integration: the time steps within a frame are equal, so
that each frame falls exactly on a time step. The solver reports its progress per frame, with
the time steps done, the remaining time and the energy balance.

What is different from implicit integration:

- #ui("Damping ratio") damps only the mass-proportional part, met at the lowest
  eigenfrequency (stiffness-proportional damping would shrink the time step to nothing). A
  model that moves freely as a whole is not damped. #ui("Rayleigh coefficients") are taken as
  given; a β above zero shortens the time step.
- A small bulk viscosity (6 % of critical damping in the highest mode of every element)
  smooths the ringing of the sharpest waves; its energy is part of ALLVD.
- #ui("Advanced settings"): #ui("Time step") #ui("At most") caps the time step below the
  stable one (it is never taken longer). #ui("Max. frequency"), #ui("Numerical damping") and
  the equation solver have no meaning and are hidden.
- Elements with reduced integration (C3D8R, C3D20R, S4R, S8R) compute several times faster
  than fully integrated ones; the model check points this out
  (#ui("Full integration in an explicit step")).
- Plates and shells are expanded into solid elements: the time step of a thin shell follows
  from its thickness, which can make it very short (a 0.4 mm steel sheet: about 3·10⁻⁸ s).
- The energies are those of the lumped mass; ALLWK includes the work of prescribed
  displacements, so ALLIE + ALLKE + ALLVD − ALLWK stays at its starting value. The solver
  warns when the balance is off by more than 5 %.

- *Contact* works with all contact pairs (node to surface, surface to surface, mortar,
  friction). Every time step evaluates the contact springs once, without iterations: the
  time step is so short that closing and opening within it is the motion itself. A penalty
  stiffness that would be too stiff for the time step is lowered for the points concerned
  (the output reports how many and by how much), and the time step is 10 % shorter while
  contact pairs are in force. A lowered stiffness lets the surfaces overlap a little more
  under the same pressure.
- *Large deformations* (NLGEOM) take finite strains and rotations; plasticity and
  hyperelastic materials work with and without it. The stable time step is found anew every
  100 time steps, since squeezed elements get stiffer; the bulk viscosity acts in the
  deformed configuration, so that a spinning part is not braked.
- C3D8R elements get a viscous hourglass control in addition to their hourglass stiffness;
  its work is part of ALLVD.

- *Rigid bodies and joints* follow finite rotations under #ui("Large deformations"): in
  every time step the constraints are linearized at the current rotation and the members
  are put where the rotation takes them; the inertia of the turning members (their
  centripetal acceleration) is part of the motion.
- Where constraints couple the masses of several nodes (rigid bodies, ties, equations,
  joints), the solver checks the highest frequency of the constrained system once and
  shortens the time step if the constraints raise it above that of the elements.

==== On the graphics card <dynamic-gpu>

The time steps of a linear model (no contact, no large deformations, elastic materials, no
prescribed motion that changes during the step, supports without equations or local
systems) run on a discrete graphics card from 30 000 DOFs, when the setting
#ui("Graphics card") > #ui("Use for the solver") is on (@settings). Each time step is then a
product with the stiffness matrix and one with the viscous matrix (bulk viscosity and
hourglass control) and a few vector updates, which the card does many times faster than the
processor.

Most cards compute double precision far slower than single precision (a sixty-fourth on an
RTX 4070), and Metal on macOS has none. The displacements, velocities, accelerations, loads
and energies therefore stay in double precision; only the two products run in single
precision, on the motion since the start of the frame. Their rounding error is relative to
that motion and does not add up from step to step, and the energy balance the solver
reports checks it: on test models the card agrees with the processor to 1e-4 of the largest
value. The last time step of every frame, which computes the stresses of the frame, runs on
the processor. Loads follow their amplitudes linearly over at most 256 time steps.

The solver log names the card ("explicit: time steps on ..."). A card without double
precision, a model too large for its memory or any error leaves the time steps on the
processor, with the reason in the log. `PLXS_GPU=off` keeps them on the processor, and
`PLXS_GPU=on` uses any adapter (software renderers included) for models of any size.

==== Mass scaling

A few small or thin elements can dictate a time step that the rest of the mesh does not
need. #ui("Advanced settings") > #ui("Mass scaling") > #ui("To time step") gives every
element whose own stable time step is shorter than the value just the mass that lifts it
there, (target / own time step)² times its mass; all other elements keep theirs. The dialog
then counts the steps with the new time step.

#screenshot("dynamic-explicit-mass-scaling.png", [Mass scaling to a target time step in the
  advanced settings of an explicit Dynamic step])

The solver reports the mass it added, absolute and as a share of the model, the number of
elements that got some and the element that got the most. On a cantilever of 16 000 C3D10
elements with a stable time step of 3.2·10⁻⁸ s, mass scaling to 10⁻⁷ s added 3.7 % of the
mass and the step ran three times faster.

Added mass is inertia that is not there: it slows waves and lowers eigenfrequencies where it
sits. A few per cent spread over small elements leave the overall response almost
unchanged; above 5 % the solver warns. Loads from gravity keep the real mass.

==== Implicit and explicit steps in a row

Steps of either method can follow each other: a static step settles the model, an explicit
step computes a short impact, an implicit step follows the slow motion afterwards. Each step
takes over the displacements, velocities and stresses at the end of the step before it, and
#ui("Starts from") names the method of that step. The accelerations are not taken over:
every step computes them again from its own mass, the implicit step from the consistent mass,
the explicit step from the lumped mass, so the motion continues without a jolt. Velocities
at the end of an explicit step are those at the end time, not the half step before it.

An explicit step after a contact phase starts with the reduced contact stiffness it needs for
its time step; an implicit step after it uses the full stiffness again. Bodies that rest on
each other at the switch can then settle slightly into the stiffer contact.

=== Automatic switching <dynamic-automatic>

A drop test is mostly flight: smooth motion that an implicit step covers in a few long time
steps. Only the impact needs the tiny steps of explicit integration. With #ui("Automatic
(implicit/explicit)") the solver integrates implicitly and goes on explicitly where the
implicit time steps would have to become tiny:

- an increment would have to be split, for an impact or for convergence, below the
  #ui("Switch factor") (Advanced, default 30) times the stable explicit time step,
- or an increment does not converge at all.

The step then continues explicitly from the last converged increment to the end of that
frame and tries implicit integration again. When the implicit attempt stops again before it
got as far as the explicit part before it, the explicit parts grow to 2, 4, 8 and at most 16
frames before the next attempt. Each switch hands over the state as between two steps (see
above); the same model switches at the same times.

#screenshot("dynamic-automatic.png", [Dynamic step with automatic switching between implicit
  and explicit integration])

The dialog shows the stable explicit time step and the implicit time step below which the
step switches. A switch factor of 0 keeps the step implicit; a large one switches at the
first impact. Mass scaling (Advanced) applies to the explicit parts. A linear model without
contact has nothing that would shorten the implicit time steps and stays implicit.

The timeline of the results shows the explicit parts in orange (@results-timeline). The solver
log names each switch and why, and sums up how much of the step ran explicitly:

```
automatic: implicit while its increments stay above 2.109e-5 s (30 times the stable explicit time step of 7.031e-7 s), explicit below
automatic: explicit from 0.001097 s: an impact needs implicit increments shorter than 2.109e-5 s
automatic: 2.175e-4 of 0.001933 s explicit, 1 switch to explicit
```

The vibrations the implicit parts do not resolve finely enough are damped by the numerical
damping, as in an implicit step: the switch is for time steps that must be short, not for
the accuracy of a vibration.

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
- A fast part does not pass through another unseen. The contact springs of a time step only
  look for the faces near where their points start, so a projectile that reaches a can wall
  within one time step, or a lever that swings into a wall, would fly through it. The solver
  follows every contact point and master face on its straight path through the time step
  (continuous collision detection) and finds the moment the first point would touch a face:
  - before trying the time step, along the motion the step predicts (the velocities and
    accelerations continued): a point that starts more than an element edge away from the
    face it reaches is the start of an impact; the time step is cut just before the touch
    and the impact computed from there. A point that would move on more than an element edge
    after its touch within the time step cuts it as well, so that the contact can follow;
  - after a time step converged: a point without contact that went through a face on its
    way cuts the time step just before it met the face.
  The cuts are internal sub-steps like the halvings; with #ui("Frames at contact events")
  the approach gets no frame of its own, the touch and the impact do.
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

#screenshot("dynamic-ccd.svg", [A steel block thrown at a plate in a single time step of 20 ms.
  Dashed: where the motion of the time step takes it without the collision detection (the
  springs of the step never see the plate, it ends below it). Red: computed; the time step is
  cut just before the impact at 2 ms, the impact is resolved in microseconds and the block
  bounces off])

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
