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
  [Material], [Every meshed solid part (in 2D every plane part, see @particle-flow-2d) with
    a section that is not part of a rigid body is filled with particles at the start, two
    to a cell in each direction, with the material of its section. The mesh only describes the shape: its elements are not computed.],
  [Walls], [The parts of a rigid body (@rigid-body) are walls: tools, dies, vessels. They
    are solid parts; the material cannot enter them. A wall that the box cuts, such as a
    drum in a thin slice, should reach a few cells beyond the box: otherwise its end faces
    lie closer to the material than its inner face, and material slides into it.],
  [Motion of a wall], [A displacement boundary condition on the reference point of a rigid
    body moves the wall: the displacements U1 to U3 shift it, the rotations UR1 to UR3 (in
    rad) turn it about the reference point, which moves along. Without an amplitude the
    motion runs over the step at constant speed, so 50 rad over a step of 5 s turn a screw
    at 10 rad/s; an amplitude shapes it over time, e.g. a short start and then a constant
    speed. A temperature boundary condition on the same reference point gives the wall its
    temperature, see @particle-flow-heat. Other boundary conditions are left out, and the
    model check warns of them.],
  [Wall friction], [A contact region (@contacts) between a wall and a filled part gives
    the wall its law: #ui("Bonded") and #ui("Rough") let the material stick,
    #ui("No Separation") and #ui("Frictionless") let it slide freely, #ui("Frictional")
    with Coulomb friction of its coefficient. Without a contact region the material sticks.],
  [Gravity], [The gravity loads of the step, each on the filled parts it selects; a load
    that selects none is reported. The step offers no other loads; one copied into it is
    left out, and the model check warns of it.],
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
large strains. The group #ui("Flow_Models") of the material library
(@material-library) has pastes, liquids and granular materials with rough values to start
from.

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
)

The yield stress, the consistency and the viscosity can each change with the temperature:
the #ui("T") button next to the value opens a table of temperature and value for it,
interpolated linearly between the rows and held below the first and above the last row.
The table takes the place of the fixed value, whose field becomes a button with the number
of rows; #ui("Remove Table") brings the fixed value back. Each value has a table of its own,
so a paste may soften in its yield stress faster than in its consistency. The tables act in
steps that follow the temperature; elsewhere the material keeps the value at the
temperature it starts with. Like the other numbers of a material, every fixed value of the
flow has a #ui("P") button that exposes it as a parameter of the project (@exposed-parameters).

#screenshot("particle-flow-temperature-table.png", [The flow of minced meat: the yield
stress and the consistency each follow a table over the temperature (the buttons with the
number of points open them); the window shows the yield stress, which falls as the fat
softens.])

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

==== Mixtures of two grain sizes <particle-flow-segregation>

With #ui("Two grain sizes") the granular material is a mixture of coarse grains, the
#ui("Grain diameter"), and fine ones, the #ui("Fine grain diameter"); the
#ui("Fine fraction") is the volume fraction of the fine grains at the start. Where the
mixture flows, the fine grains fall into the gaps between the coarse ones and sink, and the
coarse ones rise: a heap, a chute or a drum sorts itself (segregation), while the shear
mixes the grains again by diffusion. Each particle carries its fine fraction $phi$, which
changes after the model of Gray and Thornton (2005) in the form of Fan et al. (2014):

#fields(
  [Segregation], [The fine grains move along the gravity, relative to the mixture, at
    $w = S dot(gamma) (1 - phi)$ with $S = 0.26 thin d_f ln(d_c slash d_f)$ (Schlick et
    al. 2015), $d_f$ and $d_c$ the fine and coarse diameter: the larger the difference
    of the sizes and the faster the shear, the faster the mixture sorts.],
  [Diffusion], [$D = 0.042 thin dot(gamma) thin overline(d)^2$ with the mean diameter
    $overline(d) = phi d_f + (1 - phi) d_c$ (Utter and Behringer 2004).],
  [Friction], [The inertial number $I$ takes the mean diameter, so a fine layer flows
    a little more easily.],
)

The results show the fine fraction as the field #ui("FINE"). A mixture at rest does not
sort; only the shear moves the grains past each other. Parts of different materials with
the same grains but other fine fractions start sorted, for example a layer of fine grains
under one of coarse ones. The transport is checked against the exact solution of Gray and
Thornton for a column at uniform shear: the layers of fine and coarse grains grow at the
speeds it gives, and the amount of fine grains is kept.

A mixture is computed on the processor; the graphics card does not compute the
segregation yet.

#screenshot("particle-flow-segregation.png", [Glass beads of 3 mm and 1.5 mm, half and
half, in a drum that turns four times in 4 s: the fine beads (red) gather in the core of the
flowing heap, the coarse ones (blue) at its rim, as in experiments with rotating drums.])

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

=== 2D models <particle-flow-2d>

A model in plane strain or an axisymmetric model (@model-properties) flows in the plane:
the filled parts and the walls are plane parts, meshed with plane elements, and the
particles move in x and y only. The step computes a slab one cell thick along z that
repeats, with one layer of particles; it costs a fraction of the same flow in 3D.

#fields(
  [Plane strain], [A long body across which the material flows, such as a dam, a slot
    die or a heap along a wall. The masses, the loads of the walls and the energies are per
    unit length along z.],
  [Axisymmetric], [A body of revolution about the y axis, x the radius, such as a nozzle,
    a cup or a pile: every particle stands for the ring it lies on, so the hoop stress
    holds the material together around the axis and a flow towards the axis narrows as it
    should. The box starts at the axis, where its side #ui("X min") is the axis itself,
    frictionless. The masses, the energies and the force of a wall are of the whole ring;
    of a wall's force only the part along the axis remains, the rest cancels around the
    ring. Gravity acts along the axis.],
)

The 2D models are computed on the processor. Against the flow in 3D they agree closely: a
viscous fluid flows down a pipe as Poiseuille's parabola within 3 %, a cylinder of sand
collapses to the same heap as a quarter of it computed in 3D (height within 8 %, runout
within a cell) and a cylinder of water spreads as fast; plane strain is the same as a 3D
slab periodic in z. A plane stress model takes no Particle Flow step.

As material of a body of revolution flows towards the axis its rings shrink, and since a
particle keeps the mass of its ring, the particles move apart in the plane: material that
narrows to a third of its radius covers nine times its area with the same particles. Keep
a narrowing such as a nozzle within about 1:3 in radius, or the cell small enough that the
particles of the narrow part stay less than a cell apart; beyond that the material tears
into separate particles.

#screenshot("particle-flow-2d.png", [An axisymmetric nozzle of a paste printer: a piston
pushes toothpaste through a cone toward an orifice of 3 mm, coloured by its speed, which rises
tenfold toward the orifice. The plane model shows half of the section; the axis is its left
edge.])

=== Temperature <particle-flow-heat>

With #ui("Temperature") on, every particle carries a temperature. The flow heats the
material by what it dissipates, friction at the walls heats it where it slides, the heat is
conducted through the material over the grid, and walls with a temperature heat or cool it
with the #ui("Wall heat transfer") coefficient. The temperature tables of a material's flow
values make it softer or stiffer as it warms.

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
spacing (in 2D a square), drawn rounded and without edges; a frame moves the cubes to where the particles are
(true scale). A particle that
left the model through an open side is no longer drawn. Each wall is a part of its own,
drawn as its surface where it is. The fields are:

#fields(
  [U, V], [Displacement from the start and velocity of the particles.],
  [PFLOW], [#ui("PRESS") the pressure, #ui("SHEAR") the equivalent shear stress
    ($sqrt(J_2)$), #ui("PEEQ") the accumulated plastic shear, #ui("RATE") the shear rate
    in 1/s, #ui("J") the volume ratio.],
  [NT], [The temperature, in steps that follow it.],
  [FINE], [The fine fraction of grain mixtures (@particle-flow-segregation), when a
    material is one.],
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
