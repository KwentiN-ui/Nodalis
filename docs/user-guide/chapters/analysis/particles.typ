#import "../../template.typ": *

== Particle analysis <particles>

A particle analysis computes material that flows: a paste such as dough or minced meat, a
fluid, or a soft solid at large deformations, which moves under gravity between rigid walls.
It uses the material point method (MPM): the material is carried by particles, which move
through a fixed background grid of cubic cells; each time step the particles hand their
mass, momentum and stress to the grid, the grid computes the motion, and the particles move
with it. Unlike a mesh, the particles never tangle, however far the material flows.

A particle analysis is a block of its own in the project schematic (@project-window), not a
step of an analysis. Add it from the block picker or the context menu (category
#ui("Analysis"), #ui("Particle Analysis")) and link two things to it:

- the #ui("Geometry") of a block, e.g. a file block with a STEP file, to its #ui("Geometry")
  row. Its solid parts are filled with material or are walls; faces and edges are not used.
- the materials of a materials block, all of them or one by one, to its #ui("Materials") row.
  The density and the elastic constants come from there.

The third row of the block shows how many parts are filled and how many are walls, and
whether the results are up to date. #ui("Update Project") solves every particle analysis
whose setup, geometry or materials changed since its last run, beside the analyses.

#screenshot("particles-window.png", [The window of a particle analysis: a paste flows out of
its corner of a tank and around a block on the floor, coloured by its speed])

Double-clicking the block opens its window. The panel on the left holds the setup, the
viewport on the right the particles, with the timeline below it.

=== Parts

Every solid part of the linked geometry is listed with its role:

#fields(
  [Not used], [The part takes no part in the analysis (default).],
  [Material], [The part is filled with particles at the start, two to a cell and direction,
    with the material and the flow given below it and an initial velocity.],
  [Wall], [The part is a rigid wall that does not move. Its #ui("Wall law") is #ui("Stick")
    (the material sticks to it), #ui("Slip") (it slides freely) or #ui("Friction") with a
    Coulomb coefficient.],
)

The flow of a material part is one of these:

#fields(
  [Elastic], [A solid that does not flow: hyperelastic at large strains (Hencky), with the
    Young's modulus and Poisson's ratio of the material.],
  [Paste (Herschel-Bulkley)], [A paste with a yield stress: below the #ui("Yield stress") it
    deforms elastically, above it flows with the shear stress
    $tau = tau_y + k dot(gamma)^n$, $k$ the #ui("Consistency") and $n$ the
    #ui("Exponent n"). $n = 1$ is a Bingham material, $n < 1$ thins with the shear rate as
    most food pastes do. Its elastic part takes the moduli of the material.],
  [Fluid], [A Newtonian fluid with a #ui("Viscosity"): the bulk modulus of the material
    gives its pressure; it has no shear stiffness and takes no tension, so a free surface
    opens instead.],
)

=== Resolution and time

#fields(
  [Cell size], [The size of a cell of the background grid: the resolution, like the element
    size of a mesh. A part should be several cells thick, a gap the material flows through at
    least six to eight cells wide.],
  [End time], [The time computed.],
  [Frames], [The number of results over the time, besides the start.],
  [Gravity], [The acceleration of gravity.],
  [Slow waves to], [The speed the elastic waves are slowed to. The time step of the explicit
    method follows the time a wave needs to cross a cell, so a stiff material takes very many
    steps. Where the material flows slowly compared with its wave speed, slower waves change
    nothing but the time to solve, as mass scaling does in an explicit dynamic step. The
    output names the factor the waves were slowed by and the largest Mach number, the speed
    of the material over that of its waves: it should stay below about 0.1.],
)

=== Box

The material moves in a box: by default the box around all filled parts and walls, or one
given by its #ui("Min") and #ui("Max") corners when #ui("Around the bodies and walls") is off. Each of its six sides is a wall with slip, a wall without slip, or
#ui("Open"): material that passes an open side leaves the analysis, e.g. at the end of a die,
and is counted as outflow.

=== Solving and results

#ui("Fill") only fills the parts with particles, to check the setup; #ui("Solve") runs the
analysis in the background, #ui("Stop") ends it early with the frames computed so far. The
window can be closed meanwhile. The results are kept in the project file.

The viewport shows the particles of a frame, coloured by the speed or by one of the fields
#ui("Pressure"), #ui("Shear stress") (the equivalent shear stress, $sqrt(J_2)$),
#ui("Plastic strain") (the accumulated plastic shear), #ui("Shear rate"),
#ui("Volume ratio") or #ui("Body"). The walls are drawn as their edges. Drag to turn the
view, right-drag to move it, scroll to zoom and double-click to fit it; #ui("Play") runs through the frames.

Under #ui("History") the window plots the energies over the time: kinetic, potential,
stored elastic, dissipated, lost at the walls and carried out through open sides. Their sum
stays constant up to the error of the method, which the output names at the end. A second
plot shows the force of the material on the walls.

The method is validated against a resting basin (hydrostatic pressure), the dam break of
Martin and Moyce, the slump test of Pashias, the collapse of a Bingham column, the
Buckingham-Reiner flow of a Bingham paste through a pipe and the flow out of a slot.
