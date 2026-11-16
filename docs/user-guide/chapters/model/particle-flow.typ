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
    speed. Other boundary conditions are left out.],
  [Wall friction], [A contact region (@contacts) between a wall and a filled part gives
    the wall its law: #ui("Bonded") and #ui("Rough") let the material stick,
    #ui("No Separation") and #ui("Frictionless") let it slide freely, #ui("Frictional")
    with Coulomb friction of its coefficient. Without a contact region the material sticks.],
  [Gravity], [The gravity loads of the step. Other loads are left out.],
)

A Particle Flow step is computed alone: the other steps of its analysis must be
deactivated. Create it with #ui("Steps") > #ui("Step"), type #ui("Particle Flow").

#screenshot("particle-flow-step.png", [A paste flows out of a tube through the holes of a die
plate under gravity, coloured by its speed and cut in half by a section view. Each particle
is drawn as a small cube.])

#screenshot("particle-flow-tool.png", [A blade turns in a paste, seen from above: a rotation
UR3 of one turn on the reference point of the rigid body, with an amplitude that starts it
slowly. The history of the set BLADE holds the torque RMZ and the work the blade put in.])

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
)

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
  [Box], [The space the material moves in: around all parts with a #ui("Margin"), or
    #ui("Fixed") by its corners. Material that leaves the box is gone.],
  [Sides], [Each of the six sides of the box is a resting wall (#ui("Frictionless"),
    #ui("Rough") or #ui("Frictional")) or #ui("Open"): material that passes an open side
    leaves the model, e.g. at the end of a die, and is counted as outflow.],
  [Smooth the volume change], [Averages the volume change over the cells (F-bar), against
    locking of nearly incompressible material and noise of the pressure. Keep it on.],
)

=== Results

The run writes results like any other analysis, shown in the Results workspace with the
legend, the increment bar and the animation. Each particle is a small cube of the particle
spacing; a frame moves the cubes to where the particles are (true scale). A particle that
left the model through an open side is no longer drawn. Each wall is a part of its own,
drawn as its surface where it is. The fields are:

#fields(
  [U, V], [Displacement from the start and velocity of the particles.],
  [PFLOW], [#ui("PRESS") the pressure, #ui("SHEAR") the equivalent shear stress
    ($sqrt(J_2)$), #ui("PEEQ") the accumulated plastic shear, #ui("RATE") the shear rate
    in 1/s, #ui("J") the volume ratio.],
)

The history output holds the energies of the model (kinetic as ALLKE, elastic as ALLIE,
dissipated by flow and at the walls as ALLVD, the work of gravity and of the moving walls as
ALLWK) under the set PARTICLES. Per wall it holds the force of the material on it, the
moment of that force about the reference point of the wall, which is the torque a drive
must give a turning tool, and the work the wall did on the material as ALLWK, the energy
the drive puts in. The sides of the box have their force and moment about the origin. The output of the run names the number of particles and cells, the
time steps and the error of the energy balance.

The method is validated against a resting basin (hydrostatic pressure), the dam break of
Martin and Moyce, the slump test of Pashias, the collapse of a Bingham column, the
Buckingham-Reiner flow of a Bingham paste through a pipe, the flow out of a slot and the
torque on a cylinder turning in a viscous fluid (Couette flow).
