#import "../../template.typ": *

== Distributions <distributions>

A distribution is a factor that varies over the model. A pressure, a temperature boundary
condition, an initial temperature or a defined temperature that uses it takes its value times
the factor at every element face or node instead of one value for the whole region. Typical uses
are a hydrostatic pressure that grows with depth, a pressure map from a CFD analysis or a
temperature gradient for thermal expansion. Create a distribution with
#menu("Steps", "Distribution") or a double-click on #ui("Distributions") in the tree.

#screenshot("distribution-dialog.png", [Distribution by an equation: the factor falls from 1 to 0
  along the beam])

#fields(
  [Type], [#ui("Equation") or #ui("Points from file").],
  [Factor], [The equation in the coordinates `x`, `y` and `z`, in the length unit of the analysis.
    It uses the syntax of the derived field outputs: `+ - * / %`, comparisons, `? :` and functions
    such as `Sqrt`, `Pow`, `Sin`, `Exp`, `Max` or `If`, for example `=1 - z/200` or
    `=Max(0, x/50)`.],
  [Over the mesh], [The smallest and largest factor at the nodes of the mesh, to check the
    distribution before it is used.],
)

With #ui("Points from file") the factors come from a text file with one point per line:
`X Y Z factor`, the coordinates in the length unit of the analysis, separated by blanks, tabs,
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
