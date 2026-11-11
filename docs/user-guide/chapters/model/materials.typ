#import "../../template.typ": *

== Materials <materials>

#menu("Material", "Material") creates a material. Each property is switched on with its
check box. A new material starts with #ui("Elasticity") switched on.

#screenshot("material.png", [Material dialog])

#fields(
  [Density], [Needed for frequency steps and transient heat transfer.
    Written as `*DENSITY`.],
  [Elasticity], [#ui("Model") #ui("Linear elastic") with #ui("Young's modulus") and
    #ui("Poisson's ratio") (0 to 0.5), written as `*ELASTIC`; or a hyperelastic model for
    rubber (@hyperelasticity), written as `*HYPERELASTIC`.],
  [Thermal conductivity], [Written as `*CONDUCTIVITY`.],
  [Specific heat], [Needed for transient heat transfer. Written as `*SPECIFIC HEAT`.],
  [Thermal expansion], [#ui("Expansion coefficient") and #ui("Reference temperature") (default
    20). Written as `*EXPANSION`.],
)

Values can be typed in any unit of the right kind, for example `210 GPa` or `7.85 g/cm³`
(@numeric-input).

Selecting a material in the model tree shows where it is used: the sections that have it are
marked in the tree, and the parts of those sections are highlighted in the 3D view.

#screenshot("material-usage.png", [Selected material with its sections and parts])

=== Material library <material-library>

#menu("Material", "Material Library") opens the material library.
On the left are the library materials in categories, on the right the materials of the model.

#screenshot("material-library.png", [Material library])

- #ui("Copy to Model") and #ui("Copy to Library") copy the selected material. Values are
  converted between the unit system of the library and that of the model. A name that already
  exists gets a suffix.
- #ui("Add Category"), #ui("Rename"), #ui("Delete"), #ui("Move Up") and #ui("Move Down")
  organise the library.
- #ui("Preview Material Properties") shows the values of the selected material.

A new library has two groups. #ui("Elastic_Models") holds materials with elastic data only:

- #ui("Steel"): #ui("Generic Steel").
- #ui("Aluminium"): the wrought alloys (#ui("Wrought")) EN AW-1050A, 3003, 5083, 5754, 6061,
  6082, 2024 and 7075, and the cast alloys (#ui("Cast")) EN AC-AlSi7Mg0.3, AlSi10Mg, AlSi12
  and AlSi9Cu3(Fe).
- #ui("Plastics"): PLA, PETG, ABS, PA6, PA12, PET, PC, PP, POM and PMMA.
- #ui("Elastomers"): hyperelastic generic rubbers (@hyperelasticity).

#ui("Plastic_Models") holds steels with a hardening curve for plastic analyses
(#ui("Plastic") switched on, isotropic hardening): #ui("S235"), #ui("S355") and #ui("42CrMo4").

All entries have density, Young's modulus, Poisson's ratio, thermal expansion, thermal
conductivity and specific heat. The values are typical room-temperature means from datasheets.
Plastics vary a lot with grade, moisture, temperature and, for 3D printing, with print
orientation and infill; aluminium alloys vary with the temper and the casting process. Treat
them as starting points and replace them with the values of your material.

The hardening curves are true stress over plastic strain and end at the uniform strain; the
solver keeps the last stress beyond it.

- #ui("S235") and #ui("S355") use the nominal strengths of EN 10025-2 (yield 235 and 355 MPa,
  tensile strength 360 and 470 MPa). The curve has a yield plateau and a strain hardening range
  after the model of Yun and Gardner (_J. Constr. Steel Res._ 133, 2017, pp. 36-46): plateau
  end strain $0.1 f_y/f_u - 0.055$ (between 0.015 and 0.03), uniform strain
  $0.6 (1 - f_y/f_u)$, and a cubic rise to the tensile strength with zero slope at the uniform
  strain. Real steel from a certain heat is usually stronger than the nominal values; take
  the values of the material certificate if you have it.
- #ui("42CrMo4") is the quenched (830 °C, oil) and tempered (630 °C) state with yield stress
  850 MPa and tensile strength 900 MPa. The curve is the power law $sigma = 1277.3 "MPa" dot
  epsilon_p^0.0979$ fitted by Basan, Kunc and Franulović (_Engineering Review_ 28, 2008,
  pp. 23-30, Table 2) to tests of this state. Other tempering temperatures give very different
  curves.

Libraries saved by an older version are updated when they are opened: missing generic
materials are added, and the former S235 is renamed to Generic Steel if you have not changed
it. The category #ui("Structural") below #ui("Steel") of earlier versions is dissolved; its
materials move up into #ui("Steel"). Materials you changed or deleted are not touched afterwards.

The library is stored per user in `materials.ron` in the settings folder (@settings). When the
dialog is closed, Nodalis asks whether to save changes to the library.
