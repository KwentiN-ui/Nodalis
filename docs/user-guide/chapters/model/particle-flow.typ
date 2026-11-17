#import "../../template.typ": *

== Particle Flow step <particle-flow>

A Particle Flow step computes material that flows far beyond what a mesh can follow: a paste
such as dough or minced meat, a fluid, or a soft solid at large deformations, moving under
gravity between rigid walls. It uses the material point method (MPM): the material is
carried by particles, which move through a fixed background grid of cubic cells; each time
step the particles hand their mass, momentum and stress to the grid, the grid computes the
motion, and the particles move with it. The particles never tangle, however far the
material flows.

The step is a step of a normal analysis and takes the model as it is, with the Geometry,
FE Model and Results workspaces:

#fields(
  [Material], [Every meshed solid part with a section that is not part of a rigid body is
    filled with particles at the start, two to a cell in each direction, with the material
    of its section. The mesh only describes the shape: its elements are not computed.],
  [Walls], [The parts of a rigid body (@rigid-body) are walls: tools, dies, vessels. They
    are solid parts; the material cannot enter them.],
  [Motion of a wall], [A displacement boundary condition on the reference point of a rigid
    body moves the wall: the displacements U1 to U3 shift it, the rotations UR1 to UR3 (in
    rad) turn it about the reference point, which moves along. Without an amplitude the
    motion runs over the step at constant speed, so 50 rad over a step of 5 s turn a screw
    at 10 rad/s; an amplitude shapes it over time, e.g. a short start and then a constant
    speed. A temperature boundary condition on the same reference point gives the wall its
    temperature, see @particle-flow-heat. Other boundary conditions are left out.],
  [Wall friction], [A contact region (@contacts) between a wall and a filled part gives
    the wall its law: #ui("Bonded") and #ui("Rough") let the material stick,
    #ui("No Separation") and #ui("Frictionless") let it slide freely, #ui("Frictional")
    with Coulomb friction of its coefficient. Without a contact region the material sticks.],
  [Gravity], [The gravity loads of the step, each on the filled parts it selects; a load
    that selects none is reported. Other loads are left out.],
)

A Particle Flow step is computed alone: the other steps of its analysis must be
deactivated. Create it with #ui("Steps") > #ui("Step"), type #ui("Particle Flow").

#screenshot("particle-flow-step.png", [A paste flows out of a tube through the holes of a die
plate under gravity, coloured by its speed and cut in half by a section view. Each particle
is drawn as a small cube.])

#screenshot("particle-flow-tool.png", [A blade turns in a paste, seen from above: a rotation
UR3 of one turn on the reference point of the rigid body, with an amplitude that starts it
slowly. A node output RF on its reference point gives the torque RMZ and the work the blade
put in.])

=== Flow of a material

The material dialog (@materials) gives a material its #ui("Flow (Particle Flow)"); other
steps ignore it. The elastic part comes from its Young's modulus and Poisson's ratio, the
mass from its density. Without a flow the material stays elastic, a hyperelastic solid at
large strains.

#fields(
  [Herschel-Bulkley], [A paste with a yield stress: below the #ui("Yield stress") it
    deforms elastically, above it flows with the shear stress
    $tau = tau_y + K dot(gamma)^n$, $K$ the #ui("Consistency K") and $n$ the
    #ui("Flow index n"). $n = 1$ is a Bingham material, $n < 1$ thins with the shear rate
    as most food pastes do.],
  [Newtonian], [A fluid with a #ui("Viscosity"): the bulk modulus of the elastic constants
    gives its pressure; it has no shear stiffness and takes no tension, so a free surface
    opens instead.],
  [Granular], [Dry sand, gravel, powder or grain, see @particle-flow-granular.],
  [Temperature table], [Rows of temperature and a factor on the yield stress and the
    consistency (or the viscosity), interpolated between the rows and held beyond them: a
    paste that turns soft as it warms has factors below 1 at higher temperatures. It acts in
    steps that follow the temperature; empty, the flow is the same at every temperature.],
)

=== Granular material <particle-flow-granular>

The flow #ui("Granular") is a bulk solid of dry grains after the $mu(I)$ rheology (Jop,
Forterre and Pouliquen 2006), the law most used for granular flows in a continuum. The
grains carry no tension; under pressure they hold a shear stress up to the friction
$mu p$, beyond it they flow. The friction rises with the inertial number
$I = dot(gamma) d sqrt(rho slash p)$, the shear rate against the time a grain needs to fall
into the gap under the pressure:

$ mu(I) = mu_s + (mu_2 - mu_s) I / (I_0 + I) $

#fields(
  [Friction angle], [$mu_s = tan$ of it: the friction at rest, about the angle of repose
    of the material (sand 30° to 35°, glass beads 21°).],
  [Limit angle], [$mu_2 = tan$ of it: the friction in fast flow. Equal to the friction
    angle, the friction is the same at every speed (Drucker-Prager).],
  [Grain diameter], [The mean diameter $d$ of the grains.],
  [I0], [The inertial number $I_0$ halfway between the two frictions, about 0.2 to 0.3;
    $rho$ in $I$ is the density of the material, the bulk density of the heap.],
)

The particles are points of the computation, not grains: a cell holds many grains, and the
grain diameter enters only through $I$. Young's modulus and Poisson's ratio give the
stiffness of the heap under pressure; a soft value (some MPa) keeps the time step large and
changes little. Two materials of different grains can be mixed by giving their parts
different sections; they then move together and do not separate by size.

The law is checked against measurements and the exact solution: a column of sand that
collapses onto a rough floor runs out as far as Lube et al. (2005) measured within 5 %,
a layer on an incline rests below its friction angle, and above it flows with the speed
profile of Bagnold that the law gives.

#screenshot("particle-flow-sand.png", [A column of sand, 50 mm across and 30 mm high, has
collapsed on a floor with friction into a flat heap, coloured by the plastic shear the
grains went through: most at the rim, which ran out furthest.])

=== Settings of the step

#fields(
  [Duration], [The time computed.],
  [Frames], [The number of results over the time, besides the start.],
  [Cell size], [The size of a cell of the background grid: the resolution, like the element
    size of a mesh, and what decides the computing time. A part should be several cells
    thick, a gap the material flows through eight cells wide or more.],
  [Wave speed], [The speed the elastic waves are slowed to. The time step of the explicit
    method follows the time a wave needs to cross a cell, so a stiff material takes very
    many steps. Where the material flows slowly compared with its waves, slower waves change
    nothing but the time to solve, as mass scaling does in an explicit dynamic step. The
    output names the largest ratio of the speed of the material to that of its waves: it
    should stay below about 0.1.],
  [Box], [The space the background grid covers and the material moves in: around all
    parts with a #ui("Margin"), or #ui("Fixed") by its corners. Material that leaves the box
    is gone. The FE Model view draws the box as orange edges. In a closed vessel the box
    around the parts is right; where the material can leave the walls, such as a heap that
    spreads or a jet out of a die, the box must leave it room.],
  [Sides], [Each of the six sides of the box is a resting wall (#ui("Frictionless"),
    #ui("Rough") or #ui("Frictional")) or #ui("Open"): material that passes an open side
    leaves the model, e.g. at the end of a die, and is counted as outflow.],
  [Smooth the volume change], [Averages the volume change over the cells (F-bar), against
    locking of nearly incompressible material and noise of the pressure. Keep it on.],
  [Temperature], [Follows the temperature of the material, see @particle-flow-heat.],
  [Wall heat transfer], [The heat transfer coefficient between the material and the walls
    with a temperature.],
)

=== Temperature <particle-flow-heat>

With #ui("Temperature") on, every particle carries a temperature. The flow heats the
material by what it dissipates, friction at the walls heats it where it slides, the heat is
conducted through the material over the grid, and walls with a temperature heat or cool it
with the #ui("Wall heat transfer") coefficient. The temperature table of a material's flow
makes it softer or stiffer as it warms.

#fields(
  [Materials], [Each filled material needs its #ui("Conductivity") and
    #ui("Specific heat").],
  [Start], [A temperature initial condition (@initial-conditions) on each filled part gives
    its temperature at the start.],
  [Walls], [A temperature boundary condition on the reference point of a rigid body holds
    its walls at that temperature. Walls without one take no heat but what friction makes.],
)

#screenshot("particle-flow-heat.png", [The mixer seen from above after one turn: the paste
starts at 4 °C, the blade is held at 40 °C. The warmed paste lies in a thin layer along the
blade and in its wake; a node output NT on the paste gives its mean and highest temperature
over the time, RFL on the reference point the heat the blade gave.])

The time step stays below the time heat needs to cross a cell; with food pastes, which
conduct little, that is far longer than the time step of the flow and costs nothing.

=== On the graphics card <particle-flow-gpu>

From 20 000 particles the time steps run on a discrete graphics card, when the setting
#ui("Graphics card") > #ui("Use for the solver") is on (@settings). The whole time step
runs on the card, in single precision: the particles are sorted by the cell they are in,
every node of the grid gathers what the particles around it give, the walls, the sides of
the box and the temperature act there, and the particles take their motion and their
stress back. The processor only writes the frames and the points of the history output.
The sums do not depend on the order the card's threads run in, so a run on the same card
gives the same results every time, and they agree with the processor's: a paste stirred by
a warm blade with friction gives the torque, the work, the heat and the temperatures within
0.01 % of the processor.

The output says so as the run starts; its report at the end names the card
(#ui("Particles computed on …")) or why the
processor computes them: the card is switched off, there is no discrete card, or the model
needs more of the card's memory than the solver may take (@settings).

#screenshot("particle-flow-gpu.png", [The end of the output of a Particle Flow step computed
on the graphics card (here a software renderer), filtered for "particles": the progress,
the report with the particles, cells and time steps, and the card.])

=== Results

The run writes results like any other analysis, shown in the Results workspace with the
legend, the increment bar and the animation. Each particle is a small cube of the particle
spacing, drawn rounded and without edges; a frame moves the cubes to where the particles are
(true scale). A particle that
left the model through an open side is no longer drawn. Each wall is a part of its own,
drawn as its surface where it is. The fields are:

#fields(
  [U, V], [Displacement from the start and velocity of the particles.],
  [PFLOW], [#ui("PRESS") the pressure, #ui("SHEAR") the equivalent shear stress
    ($sqrt(J_2)$), #ui("PEEQ") the accumulated plastic shear, #ui("RATE") the shear rate
    in 1/s, #ui("J") the volume ratio.],
  [NT], [The temperature, in steps that follow it.],
)

The history output holds what the history outputs of the step (@history-output-step) ask for,
nothing else. A new Particle Flow step gets an #ui("Energies") output, which can be deleted
like any other:

#fields(
  [Energies], [The energies of the model under the set PARTICLES: kinetic as ALLKE, elastic
    as ALLIE, dissipated by flow and at the walls as ALLVD, the work of gravity and of the
    moving walls as ALLWK.],
  [Node output on a rigid body], [On the reference point of a rigid body or on its parts:
    #ui("RF") gives the force of the material on its walls, the moment of that force about
    the reference point (RM), which is the torque a drive must give a turning tool, and
    the work the wall did on the material as ALLWK, the energy the drive puts in.
    #ui("RFL") gives the heat the wall gave the material.],
  [Node output on filled parts], [#ui("NT") gives the mean and the highest temperature of
    their material (TMEAN, TMAX).],
)

Element and contact outputs and other variables are left out with a warning. The output of
the run names the number of particles and cells, the time steps and the error of the energy
balance.

The method is validated against a resting basin (hydrostatic pressure), the dam break of
Martin and Moyce, the slump test of Pashias, the collapse of a Bingham column, the
Buckingham-Reiner flow of a Bingham paste through a pipe, the flow out of a slot, the
torque on a cylinder turning in a viscous fluid (Couette flow) and the heat it dissipates
there, and the conduction between a warm and a cold block (the error function of the
analytic solution).
