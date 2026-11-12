#import "../../template.typ": *

=== Anisotropic elasticity <anisotropic>

Materials that are stiffer in one direction than in another, such as fibre composites, wood,
rolled sheet or printed plastics, get their linear elasticity as #ui("Orthotropic") or
#ui("Anisotropic") in the #ui("Model") box of #ui("Elasticity") in the material dialog. Both
replace Young's modulus and Poisson's ratio; switching keeps the stiffness the material had
(the nearest isotropic one when going back to #ui("Linear elastic")).

#screenshot("material-orthotropic.png", [An orthotropic material])

#fields(
  [E1, E2, E3], [#ui("Orthotropic"): Young's moduli along the material axes 1, 2 and 3.],
  [nu12, nu13, nu23], [The Poisson's ratios: $nu_(i j)$ is the contraction along $j$ under a
    stress along $i$. The reverse ratios follow from symmetry,
    $nu_(j i) = nu_(i j) E_j / E_i$.],
  [G12, G13, G23], [The shear moduli of the planes of the axes.],
  [D1111 ... D2323], [#ui("Anisotropic"): the 21 entries of the upper triangle of the
    symmetric 6 × 6 stiffness matrix in the material axes, acting on the strains 11, 22, 33
    and the engineering shear strains 12, 13, 23 (so D1212 is the shear modulus G12). It starts
    as the stiffness the material had.],
)

The dialog refuses constants that are not stable, for which some strain would give energy
back (for example $nu_(12)^2 > E_1 / E_2$); the reason is shown below the constants.

*Material axes.* The axes 1, 2, 3 are the axes x, y, z of the #ui("Orientation") of the solid
section (@sections), a rectangular coordinate system of the features (@features). Without an
orientation they are the global axes. The axes stay with the material: they are the axes of
the undeformed part, and under #ui("Nlgeom") they turn with it.

Written as `*ELASTIC, TYPE=ENGINEERING CONSTANTS` (orthotropic) or `TYPE=ANISO` (anisotropic)
and, with an orientation, `*ORIENTATION, SYSTEM=RECTANGULAR` named on the
`*SOLID SECTION`. Stresses and strains in the results are in the global axes.

#limitation[Anisotropic materials work in solid sections of 3D models, with linear
elasticity only: no plasticity, creep or hyperelasticity, and no shells, membranes, beams or 2D
elements. Cylindrical orientations (axes that turn around an axis) are not supported yet. The
model check reports these cases. Contact stiffnesses and the stable time increment of explicit
steps take the Voigt average of the moduli.]

A cantilever with the stiffness along its axis doubled bends by nearly half; turned by an
orientation so that axis 1 lies across the beam, it bends exactly like the same material with
the constants of 1 and 2 swapped. Nodalis's tests check both with the solver.
