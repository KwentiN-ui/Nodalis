#import "../../template.typ": *

== Materials <materials>

#menu("Material", "Material") creates a material. Each property is switched on with its
check box. A new material starts with #ui("Elasticity") switched on.

#screenshot("material.png", [Material dialog])

#fields(
  [Density], [Needed for frequency steps and transient heat transfer.
    Written as `*DENSITY`.],
  [Elasticity], [#ui("Young's modulus") and #ui("Poisson's ratio") (0 to 0.5). Written as
    `*ELASTIC`.],
  [Thermal conductivity], [Written as `*CONDUCTIVITY`.],
  [Specific heat], [Needed for transient heat transfer. Written as `*SPECIFIC HEAT`.],
  [Thermal expansion], [#ui("Expansion coefficient") and #ui("Reference temperature") (default
    20). Written as `*EXPANSION`.],
)

Values can be typed in any unit of the right kind, for example `210 GPa` or `7.85 g/cm³`
(@numeric-input).

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

A new library contains #ui("Generic Steel") (category #ui("Steel / Structural")) and generic
plastics in the category #ui("Plastics"): PLA, PETG, ABS, PA6, PA12, PET, PC, PP, POM and PMMA.
Each has density, Young's modulus, Poisson's ratio, thermal expansion, thermal conductivity and
specific heat. The values are typical room-temperature means from datasheets. Plastics vary a
lot with grade, moisture, temperature and, for 3D printing, with print orientation and infill,
so treat them as starting points and replace them with the values of your material.

Libraries saved by an older version are updated when they are opened: missing generic
materials are added, and the former S235 is renamed to Generic Steel if you have not changed
it. Materials you changed or deleted are not touched afterwards.

The library is stored per user in `materials.ron` in the settings folder (@settings). When the
dialog is closed, Nodalis asks whether to save changes to the library.
