#import "../../template.typ": *

=== Infill section <infill-section>

An #ui("Infill (3D print)") section models a solid part printed by fused deposition (FDM): the
part stays one solid in the mesh, and every element gets the stiffness of what the printer lays
down there. There is no need to model the part hollow with the wall thickness of the slicer.

- The *walls*: the perimeters along the sides and the solid layers at top and bottom are the
  printed material. Across the layers (along the build direction) it is softer by the
  #ui("Layer factor"), since the layers bond less well than the strands hold together.
- The *core*: the infill as an orthotropic solid of the same stiffness (homogenization). Its
  walls or lines are thin plates or strands of the printed material, averaged at equal strain by
  their share of the volume, plus the bending stiffness of the cells for the deformations the
  walls alone do not resist (after Gibson & Ashby, _Cellular Solids_, 1997).
- Elements reaching through the wall get the volume average of wall and core by the share of
  their depth below the surface that lies in the wall, in eight steps.

Create it with #menu("Material", "Section") and choose #ui("Infill (3D print)"). It applies to
the solid elements of 3D parts.

#screenshot("infill-section.png", [Infill section])

#fields(
  [Material], [The printed material (the filament), with isotropic linear elasticity and,
    for dynamics and gravity, its density. The library has PLA, PETG, ABS and other plastics.],
  [Orientation], [The print axes: z of the coordinate system is the build direction, x the
    direction the infill angle counts from. Global: printed along z.],
  [Wall thickness], [The perimeters on the sides: their number times the extrusion width
    (default 0.8 mm, two perimeters of 0.4 mm).],
  [Top/bottom thickness], [The solid layers under faces that face up or down (within 45° of
    the build direction): their number times the layer height (default 0.8 mm).],
  [Infill pattern], [The pattern of the slicer, see the table below, or
    #ui("Custom core material").],
  [Infill density], [The share of material in the core, 1 to 100 % (default 20 %). The density
    of the core follows from it.],
  [Infill angle], [The direction of the infill lines in the layer from x (default 45°, as the
    slicers start).],
  [Core material], [#ui("Custom core material"): a material of its own for the core, for
    example orthotropic constants measured on printed specimens, in the print axes.],
  [Layer factor], [Young's modulus and shear moduli across the layers as a share of those in
    the layer (default 0.8; measurements on PLA and PETG report about 0.6 to 0.9).],
)

Wall thickness, top/bottom thickness and infill density can be linked to a parameter, so a
study can vary them.

The patterns and the stiffness of their core at 20 % in the print axes, as a share of the
printed material's Young's modulus (layer factor 0.8, angle 45°):

#table(
  columns: 5,
  [*Pattern*], [*E#sub[x] = E#sub[y]*], [*E#sub[z]*], [*Model*], [*Slicer names*],
  [Rectilinear / Lines], [0.042], [0.039], [two families of strands at 90°, bending like an
    open-cell foam ($rho^2$)], [Rectilinear, Lines, Zig Zag],
  [Grid], [0.009], [0.167], [walls in two directions, along z prismatic; at 45° to the walls
    the square cells bend ($rho^3$)], [Grid],
  [Triangles], [0.076], [0.167], [walls in three directions, stiff in the whole layer],
    [Triangles, Tri-Hexagon],
  [Cubic], [0.110], [0.087], [three families of walls tilted like the faces of a cube on its
    corner], [Cubic, Adaptive Cubic, Support Cubic],
  [Gyroid], [0.078], [0.062], [nearly isotropic, fit of Gonabadi et al., Applied Sciences 12
    (2022) 2180, to simulations and tests of printed gyroids], [Gyroid],
  [Honeycomb], [0.010], [0.170], [hexagonal prisms: stiff along z, bending in the layer
    ($1.5 rho^3$)], [Honeycomb],
)

Grid and honeycomb are soft in the layer: their cells fold like a parallelogram, and in a real
part the walls hold them. Along the lines of a grid (infill angle 0° to the load) it carries
like half its material.

Nodalis writes each infill section as up to nine element sets with an anisotropic material
each (`*ELASTIC, TYPE=ANISO`, in the global axes), the mix of wall and core of their wall share.
The sets and materials are internal and do not show in the model.

#note[The walls are resolved by the mesh: an element at the surface that is much thicker than
the wall gets the wall spread over its depth, which stiffens it less in bending than a thin
wall at the surface would. Give the outer faces a #ui("Local Mesh Size") with an element size
and a #ui("Depth") of about the wall thickness (@mesh-setup): the elements are then as thin as
the wall at the surface and grow towards the core. The model check warns when the elements of
an infill section are more than twice as large as its thinnest wall. The infill is a smeared
continuum: the stress in the core is the mean stress over a cell, not the stress in the infill
walls.]

Faces on a plane of a #ui("Symmetry") boundary condition are no printed surface: the part goes
on beyond them, so the elements there keep the core. Contacts, ties and symmetry act on the
mesh as with any other section.

#limitation[Every other face of the region is taken as a printed surface, also faces where the
region meets another part through a tie or contact; split such parts only where the printed part
has a surface. The core's stiffness is an estimate from the geometry of the
pattern; for a component that matters, check it against a printed specimen and enter measured
values as a #ui("Custom core material").]
