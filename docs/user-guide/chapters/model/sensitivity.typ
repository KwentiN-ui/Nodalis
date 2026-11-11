#import "../../template.typ": *

== Sensitivity step <sensitivity>

A #ui("Sensitivity (shape)") step (`*SENSITIVITY`) shows where a change of the shape pays off
most. For every node of a design region the solver computes how much a design response, such as
the strain energy or an eigenfrequency, changes when the node moves a little along the surface
normal. The step does not change the model: it is a guide for the next design change, for
example whether a larger fillet or a thicker rib raises the stiffness more. Create it with
#menu("Steps", "Step") and choose the type in the dialog.

#screenshot("sensitivity-step.png", [Sensitivity step])

#fields(
  [Mass], [Mass of all elements (`MASS`). Needs a density.],
  [Strain energy], [Strain energy of all elements under the loads of the previous static step
    (`STRAINENERGY`). Less strain energy means a stiffer part.],
  [Eigenfrequencies], [All eigenfrequencies of the previous frequency step (`EIGENFREQUENCY`).
    CalculiX takes it only as the single design response of a step.],
  [Filter radius], [Smooths the sensitivities over this distance (`*FILTER, TYPE=LINEAR`).
    Without a filter the raw values follow the mesh closely.],
  [Design region], [The nodes that may move, usually picked faces or a node set. CalculiX keeps
    only the nodes on the surface and lists the rejected ones in a `_WarnNodeDesignReject.nam`
    file next to the input file.],
)

The step evaluates the solution of the step directly before it, so it has no boundary
conditions, loads or defined fields of its own:

- Mass and strain energy need a #ui("Static") step before it.
- Eigenfrequencies need a #ui("Frequency") step with #ui("Store matrices and eigenmodes") before
  it; CalculiX reads the modes from the `.eig` file.

The model check (@model-check) reports a sensitivity step without the fitting step. CalculiX
defines the design region once for the whole model: Nodalis writes the region of the first
sensitivity step (`*DESIGN VARIABLES, TYPE=COORDINATE`), and the check warns when a later
sensitivity step has another one.

=== Results

The step writes the surface normals `NORM` and one field per design response: `SENMASS`,
`SENENER` (strain energy) or `SENFREQ` (eigenfrequencies, one increment per mode with its
frequency). `DFDN` is the derivative of the response with respect to a shift of the node along
its outward normal, `DFDNFIL` the same after the filter, scaled by the solver. A negative value
of `SENENER` means that moving the surface outward there lowers the strain energy, that is,
material added there stiffens the part most. Values near zero mark surfaces where material can
be removed at little cost.

#screenshot("sensitivity-result.png", [Strain energy sensitivity of a cantilever: material at
the fixed end stiffens it most])

The history output `STEP_n` of the step lists the value of every response (`DESIGN_RESPONSE`)
and the length of its gradient (`GRADIENT_LENGTH`).

#limitation[The step exists for 3D models only. The design nodes lie on the free faces of solid
elements, which plane stress, plane strain and axisymmetric models do not have, so the dialog
does not offer the type there. Only the coordinates of the design nodes are design variables; the step does not
move them. Optimization with constraints (`*FEASIBLE DIRECTION`), displacement and stress
responses are not available yet.]
