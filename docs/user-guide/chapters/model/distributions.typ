#import "../../template.typ": *

== Distributions <distributions>

A distribution is a factor that varies over the model. A pressure, a temperature boundary
condition, an initial temperature or a defined temperature that uses it takes its value times
the factor at every element face or node instead of one value for the whole region. Typical uses
are a hydrostatic pressure that grows with depth, a pressure map from a CFD analysis or a
temperature gradient for thermal expansion. Create a distribution with
#menu("Steps", "Distribution") or a double-click on #ui("Distributions") in the tree.

#screenshot("distribution-dialog.png", [Distribution by an equation in a cylindrical coordinate
  system with two variables; the help below lists the names, operators, functions and examples])

#fields(
  [Type], [#ui("Equation") or #ui("Points from file").],
  [Coordinate system], [The system the equation takes its coordinates in and the points of a
    file are given in: #ui("Global") or one of the coordinate systems of the model
    (@coordinate-systems). A system that moves with a selection counts where it is in the
    undeformed model.],
  [Factor =], [The equation. It can use `x`, `y` and `z` along the axes of the coordinate system,
    `r`, the distance from its z axis, and `theta`, the angle about its z axis from its x axis in
    radians (-pi to pi); lengths are in the length unit of the project. With a cylindrical system
    `r`, `theta` and `z` are its own coordinates; they work in a rectangular system too. Below the
    field the dialog shows the smallest and largest factor at the nodes of the mesh, or, in red,
    what is wrong with the equation, while you type.],
  [Variables], [Named numbers the equation uses, for example `p1` and `h` in `p1 * (1 - z/h)`.
    #ui("Add variable") adds one; a name starts with a letter and holds letters, digits and `_`.
    The #ui("P") button next to a value exposes it as a parameter of the analysis' block in the
    project window (@exposed-parameters), so that a Parameters block, a formula or a parameter study
    can set it from outside.],
)

The equation is written like a formula in a spreadsheet:

- `+ - * /` and `^` (or `**`) for powers, `%` for the remainder, parentheses, numbers such as
  `2.5` or `1e-3`, and `pi`.
- Comparisons `< <= > >= == !=` give 1 where they hold and 0 where not; `and`, `or` and `not`
  (or `&& || !`) combine them.
- `If(c, a, b)` is `a` where the condition `c` holds and `b` elsewhere, for example
  `If(r < 20, 1, 0.5)`; `c ? a : b` is the same.
- Functions such as `Min`, `Max`, `Abs`, `Sqrt`, `Exp`, `Ln`, `Sin`, `Cos` (radians) or `Round`;
  names of functions are not case sensitive. A leading `=` is allowed.

The #ui("Help") of the dialog lists all names, operators and functions with what they give, and a
few examples such as a linear gradient, a bearing load `Max(0, Cos(theta))` or a bell
`Exp(-(r/10)^2)`. A click on a name or function puts it into the equation at the cursor, a click on
an example replaces the equation with it.

#ui("Show in colors on the mesh") below the factor colours the mesh by the factor at the nodes
while the dialog is open, with a legend in the 3D view; the colours follow the equation, the
variables and the coordinate system as you change them. The symbols of a pressure or a
temperature boundary condition that uses a distribution show it too: each arrow or ball is as
large as the factor where it is, relative to the largest one over the region; a negative factor
turns a pressure arrow around, where the factor is zero there is none.

#screenshot("distribution-preview.png", [The factor in colours over the mesh and the arrows of a
  pressure that uses the distribution, scaled by it])

With #ui("Points from file") the factors come from a text file with one point per line:
`X Y Z factor`, the coordinates in the length unit of the project along the axes of the
coordinate system (also of a cylindrical one), separated by blanks, tabs,
commas or semicolons. Lines starting with `#` and lines that hold no four numbers, such as a
header, are skipped. #ui("Import ...") reads the file into the model, so the project no longer
needs it.

#screenshot("distribution-points.png", [Distribution from a point file, for example a pressure
  map exported from a CFD analysis])

#fields(
  [Interpolation], [#ui("Closest point"): the factor of the nearest point. #ui("Shepard (inverse
    distance)"): a smooth mean of the nearest 8 points, weighted by one over the squared distance;
    exact at the points.],
)

In the dialog of a pressure, a temperature boundary condition, an initial temperature or a
defined temperature (by value), choose the distribution in the #ui("Distribution") field;
#ui("Uniform") is the same value everywhere.

#screenshot("pressure-distribution.png", [Pressure of 0.1 MPa scaled by the distribution])

The input file then holds one line per element face (`*DLOAD` with the element number) or per
node (`*BOUNDARY`, `*INITIAL CONDITIONS`, `*TEMPERATURE`) with the scaled value; no sets are
written for them. A pressure takes the factor at the center of each face. Temperatures are scaled
in the unit shown in the dialog, so 100 °F times 0.5 stays 50 °F.

#note[With a value of 1 the factors of the distribution are the values themselves, for example
the pressures of a CFD map in the unit of the analysis.]

A distribution that nothing uses has no effect; the model check (@model-check) warns of it in
the tree. Renaming a distribution keeps it chosen in the items that use it; deleting it makes
them invalid until another one is chosen.
