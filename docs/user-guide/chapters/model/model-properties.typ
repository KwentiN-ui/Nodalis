#import "../../template.typ": *

= The FE Model

The FE Model workspace holds everything CalculiX needs besides the mesh: materials, sections,
constraints, contacts, amplitudes, initial conditions and steps with their boundary conditions,
loads and outputs. Items are created with the tiles of the ribbon tabs #ui("Mesh"), #ui("Material"),
#ui("Features"), #ui("Connections") and #ui("Steps"), or with the command search
(@command-search); the tree no longer creates items.

#screenshot("fe-model-tree.png", [FE Model tree])

== Model properties <model-properties>

#menu("Material", "Model Properties") (or a double-click on #ui("Model") in the tree) shows
the basic properties of the model. The same dialog opens when an analysis is created
(@new-analysis); the model space and the unit system are chosen there and fixed from then on.
The dialog of an existing analysis shows them, but only the model type and the physical
constants can still be changed. For another model space or unit system, create a new analysis.

#screenshot("model-properties.png", [Model properties])

=== Model space

/ 3D: solids, shells and lines.
/ 2D plane stress: plane elements (`CPS`), the section has a thickness.
/ 2D plane strain: plane elements (`CPE`), the section has a thickness.
/ 2D axisymmetric: axisymmetric elements (`CAX`). The y axis is the axis of rotation, the model
  must lie at x ≥ 0.

All 2D models must lie in the global X-Y plane. Hovering over a model space shows a short hint
on when to use it, with a sketch of the principle: plane stress for thin parts loaded in their
plane (the plate may grow thinner, `S33` = 0), plane strain for long prismatic parts with the
same section and load all along (one slice of the body is modelled, `E33` = 0), axisymmetric
for bodies of revolution (one profile at x ≥ 0 is turned about the y axis). The part drawn in
blue is what is modelled. The same hints show in the #ui("Rectangle") mesh block.

#screenshot("model-space-sketches.png", [Hover hints of the model spaces])

Geometry and meshes must fit the model space of their analysis: a 2D mesh from an input file
cannot be imported into a 3D analysis, nor solids into a 2D one.

#note(title: "Axisymmetric models")[As in CalculiX, forces in an axisymmetric model act on the
full 360° circumference. Forces are distributed according to the radius.]

=== Model type

#ui("General model") or #ui("Submodel"). A submodel takes the displacements at its cut
boundaries from a global model. With #ui("Submodel"), the block of the analysis in the project
schematic gets a #ui("Global results") row: the results of a step of the global analysis are
dragged there (@submodel). No result file is picked in the dialog.

=== Unit system <unit-systems>

#fields(
  [Unitless], [No units. Numbers are taken as they are.],
  [m, kg, s, °C], [SI units.],
  [mm, ton, s, °C], [The default. Force in N, stress in MPa, density in t/mm³.],
  [m, ton, s, °C], [Force in kN, stress in kPa.],
  [in, lbf·s²/in, s, °F], [US units. Force in lbf, stress in psi.],
)

The dialog lists the base and derived units of the chosen system.

*How values are stored.* Nodalis stores every value of the mesh, the geometry and the model in
SI units (m, kg, s, N, Pa; temperatures in °C), whatever the unit system of the analysis. The
unit system only decides how values are *shown and typed*: a dialog of an analysis in mm, ton,
s shows lengths in mm and stresses in MPa, and converts what you type. A value typed with
another unit is converted too, for example `3 cm` in a length field becomes 30 mm. The input
file CalculiX gets is therefore always in SI units; results are shown in the unit system of the analysis (legend, scale bar,
queries, tables). Because of this, analyses of one project may use different unit systems and
still share meshes, materials and results through links (@links).

=== Physical constants

#ui("Absolute zero") and #ui("Stefan-Boltzmann") are only needed for thermal radiation.
#ui("Defaults") fills them in for the selected unit system. They are written as
`*PHYSICAL CONSTANTS`.
