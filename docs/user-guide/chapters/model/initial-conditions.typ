#import "../../template.typ": *

== Initial conditions <initial-conditions>

#menu("Steps", "Initial Condition") sets a state of the model before the first step.
The region can be a selection, parts, a node set or a surface.

#fields(
  [Temperature], [A temperature (default 20), for example as reference for thermal expansion or
    as start value of a transient heat transfer analysis. Written as `*INITIAL CONDITIONS,
    TYPE=TEMPERATURE`. A #ui("Distribution") (@distributions) makes it vary over the region.],
  [Velocity], [The velocity `V1`, `V2`, `V3` of every node of the region at the start of a
    dynamic step (@dynamic-step). Written as `*INITIAL CONDITIONS, TYPE=VELOCITY`.],
  [Angular velocity], [A rotation of the region about an axis: #ui("X"), #ui("Y"), #ui("Z") is a
    point on the axis, #ui("Axis") its direction and #ui("Rotational speed") the speed in
    radians per time (the dialog shows it in rpm). Every node gets the velocity
    $v = omega thin n times (x - p)$ of this rotation; a positive speed turns
    counter-clockwise about the axis.],
)

#screenshot("initial-velocity.png", [Initial angular velocity of a part])

Velocities only act in a dynamic step; static, frequency and thermal steps ignore them without
a message, and the model check (@model-check) reports a velocity without a dynamic step. In 2D
models a velocity has two components and the axis of a rotation is the z axis.
