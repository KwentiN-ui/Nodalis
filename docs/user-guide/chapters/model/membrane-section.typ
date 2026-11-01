#import "../../template.typ": *

=== Membrane section <membrane-section>

A #ui("Membrane") section is the shell section's sibling for structures without bending
stiffness: foils, fabrics, balloons, pretensioned roofs or a thin skin on a solid. A membrane
carries forces in its plane only, like a tensioned cloth. Its nodes have no rotations, and a
flat membrane can take a load across its plane only once it is pretensioned or deforms, which
needs a geometrically nonlinear step (#ui("Nlgeom")). For a plate under a lateral load the
shell section (@shell-section) stays the right choice.

Create it with #menu("Model", "Create Section ...") and choose #ui("Membrane"). The mesh must
be linear: mesh the part with #ui("Second order") off (@mesh-setup). Linear shells of the region
(`S3`, `S4`, `S4R`) are written as the membranes `M3D3`, `M3D4` and `M3D4R`, so the part keeps
its element type. A membrane type can also be chosen for the part itself: double-click it on
the #ui("FE Model") tab and pick it in the #ui("Element type") box (@element-type).

#screenshot("membrane-element-type.png", [Choosing a membrane element type for a part])

#screenshot("membrane-section.png", [Membrane section])

#fields(
  [Thickness], [The thickness of the membrane (default 1); it must be greater than 0.],
  [Offset], [Position of the mesh within the thickness as a fraction of the thickness, as for
    shells. Written as `OFFSET=`.],
  [Material, Region], [As for the other sections.],
)

Written as `*MEMBRANE SECTION, ELSET=..., MATERIAL=...` with the thickness on the next line.
CalculiX reports membranes on the solids it expands them to, as shells.

Membranes have to be linear: CalculiX 2.23 computes the quadratic membranes `M3D6` and `M3D8`
wrongly (in a simple tension test `M3D8` is 1.7 times too soft and gets softer with a thicker
section). The model check reports a membrane section on quadratic or solid elements, and a
shell or solid section on membranes, as an error #ui("Section does not fit its elements").

A membrane strip of 100 × 10 × 2 stretched by 0.1 in a first step and loaded with a pressure
in a second one sags at mid-span as a taut string does, $p L^2 / (8 N)$ with the tension $N$
per width; Nodalis's tests check this with CalculiX.
