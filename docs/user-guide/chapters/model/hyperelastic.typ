#import "../../template.typ": *

=== Hyperelasticity (rubber) <hyperelasticity>

Rubber, elastomers and soft tissue stretch far and return without permanent deformation, but
their stress does not grow in proportion to the strain. A hyperelastic material describes them
with a strain energy model. Pick it in the material dialog: #ui("Elasticity") on, then
#ui("Model") instead of #ui("Linear elastic").

#screenshot("material-hyperelastic.png", [Hyperelastic material with the Yeoh model])

#fields(
  [Neo-Hooke], [One constant `C10`, half the initial shear modulus. Enough up to some 50 %
    strain, and the choice when only a hardness or modulus is known.],
  [Mooney-Rivlin], [`C10` and `C01`; initial shear modulus $2 (C_10 + C_01)$. Moderate strains
    up to some 100 %.],
  [Yeoh], [`C10`, `C20`, `C30` in the first invariant. Follows the stiffening at large
    strains and fits well to uniaxial tests alone.],
  [Polynomial (order 2)], [`C10`, `C01`, `C20`, `C11`, `C02`. Flexible, but needs test data in
    several modes to stay stable.],
  [Ogden (1 to 3 terms)], [Pairs `mu`, `alpha`; initial shear modulus $sum mu_p$. Very flexible
    up to large strains.],
  [Arruda-Boyce], [Shear modulus parameter `mu` and the #ui("Locking stretch") at which the
    molecular chains are fully stretched (typically 5 to 10).],
  [Poisson's ratio], [How compressible the material is at small strains, 0.49 by default.
    Rubber is nearly incompressible (0.49 to 0.499). Nodalis turns it into the bulk modulus
    $K_0 = 2 mu_0 (1 + nu) / (3 (1 - 2 nu))$ and writes `D1` $= 2 / K_0$.],
  [Shore A hardness], [#ui("Estimate") sets the constants from the hardness: Young's modulus
    after Gent, $E = 0.0981 (56 + 7.62 H) / (0.1375 (254 - 2.54 H))$ MPa, and the model's
    constants for the same initial shear modulus. A rough estimate for small strains.],
)

The constants mean the same as in the CalculiX and Abaqus manuals, so published values carry
over: with the isochoric invariants $overline(I)_1$, $overline(I)_2$, the isochoric principal
stretches $overline(lambda)_a$ and the volume ratio $J$, the strain energy is

$ W = sum_(i+j>=1) C_(i j) (overline(I)_1 - 3)^i (overline(I)_2 - 3)^j + (J - 1)^2 / D_1 $

for the polynomial models (Neo-Hooke, Mooney-Rivlin and Yeoh are special cases) and

$ W = sum_p (2 mu_p) / alpha_p^2 (overline(lambda)_1^(alpha_p) + overline(lambda)_2^(alpha_p)
  + overline(lambda)_3^(alpha_p) - 3) + (J - 1)^2 / D_1 $

for Ogden. Below the constants the dialog shows the shear, Young's and bulk modulus at small
strains. Switching the model keeps the initial shear modulus; switching back to
#ui("Linear elastic") keeps Young's modulus and Poisson's ratio at small strains.

The library (@material-library) has generic rubbers of 40, 60 and 70 Shore A in the category
#ui("Elastomers") (Neo-Hooke from the hardness, density 1100 kg/m³).

==== Fit to test data <hyperelastic-fit>

The best constants come from tests of the rubber. #ui("Fit to Test Data...") in the material
dialog opens a window for the test curves: nominal stress (force per original cross-section)
over nominal strain (change of length per original length) of uniaxial tension or compression,
equibiaxial tension and planar tension (pure shear).

#screenshot("hyperelastic-fit.png", [Ogden model fitted to uniaxial and equibiaxial test data])

- Pick the test in the combo box, paste two columns (strain and stress in the unit shown, one
  point per line, separated by blanks, tabs, commas or semicolons, e.g. copied from a
  spreadsheet) and press #ui("Add points"). Lines that are not two numbers, such as a header,
  are skipped. The table below lists all points; each can be edited, moved to another test or
  removed, #ui("Add row") adds one.
- #ui("Fit") (with the name of the model) sets the constants of the selected model to those that match the points best:
  least squares of the relative stress error, with the material taken as incompressible.
  Poisson's ratio stays. The constants of the polynomial models follow directly; the Ogden
  exponents and the locking stretch of Arruda-Boyce are searched from several starting values,
  and a stable fit is preferred. The window shows the remaining error (RMS of the relative
  error) and warns if the fitted material is unstable somewhere.
- The plot shows the nominal stress of the model in each test against the points, and
  updates while you edit constants or points. Without points it shows the three tests of the
  current constants.

Data of one test only (usually uniaxial) can be met by many constants, which may behave
badly in the other directions: prefer Neo-Hooke, Yeoh or Arruda-Boyce then, and add
equibiaxial or planar data for Mooney-Rivlin, the polynomial or Ogden. The points are saved
with the material.

==== Steps with hyperelastic materials

- #ui("Static"), #ui("Visco") and #ui("Dynamic") steps (direct integration, also with contact)
  compute hyperelastic materials with large deformations: Nonlinear geometry is switched on
  for them automatically, and the step dialog says so.
- #ui("Frequency"), #ui("Buckle"), #ui("Complex frequency") and steady-state
  dynamics are small vibrations about a state: without a preload they use the stiffness at
  small strains; with #ui("Perturbation") after a static step they use the tangent stiffness
  of the deformed rubber.
- Coupled temperature-displacement steps, sensitivities and Dynamic steps with modal
  superposition use the stiffness at small strains only. So do trusses, membranes and beams
  with a general profile. The model check warns about these ("Hyperelastic material computed
  linearly").

A hyperelastic material cannot be plastic or creep at the same time. The model check reports
constants without a positive initial shear modulus or a Poisson's ratio of 0.5 or more as
"Invalid hyperelastic material", and warns when the nominal stress of the material stops
growing in a uniaxial, equibiaxial or planar test between the stretches 0.1 and 10
("Hyperelastic material unstable at large strains"); the dialog shows the same warning below
the constants.

#limitation[Nodalis has no hybrid elements: fully incompressible materials (`D1 = 0`) are not
  possible, and nearly incompressible rubber makes linear elements too stiff (locking). Use
  quadratic elements (C3D10, C3D20R) for rubber parts, and keep Poisson's ratio at 0.49 unless
  the volume change matters.]
