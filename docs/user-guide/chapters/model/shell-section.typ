#import "../../template.typ": *

=== Shell section <shell-section>

A #ui("Shell") section gives surface elements of a 3D model a thickness, so that a thin plate
or a sheet-metal part can be computed with shell elements (`S3`, `S4`, `S6`, `S8`) instead of
solids. Create it with #menu("Material", "Section") and choose #ui("Shell").

#screenshot("shell-section.png", [Shell section])

#fields(
  [Thickness], [The thickness of the shell (default 1); it must be greater than 0.],
  [Offset], [Position of the mesh within the thickness as a fraction of the thickness, from −1
    to 1: 0 (default) puts the mesh on the mid-surface, 0.5 on the top face, −0.5 on the bottom
    face. Written as `OFFSET=`.],
  [Material, Region], [As for the other sections. The region must contain surface elements
    only; a shell section on solid or line elements is reported as an invalid section.],
)

Written as `*SHELL SECTION, ELSET=..., MATERIAL=...` with the thickness on the next line.
CalculiX expands shells into solid elements and reports the results on these solids, with new
node numbers; their stresses differ on the top and the bottom face, as bending makes them. A
Measure block on shell nodes takes the values of the nodes they were expanded to: a node set
the extremes over the thickness, a single node the mean (forces their sum).

#note[A field output with #ui("Output (2D/3D)") set to #ui("2D") reports the shells at their
own nodes instead. CalculiX then gives the stresses of the mid-surface only: the bending
stresses are missing.]

Shell nodes have rotations: #ui("Displacement/Rotation") boundary conditions can hold `UR1`,
`UR2`, `UR3`, and #ui("Moment") loads can act on them.

==== Shell thickness in the FE model and the results

The FE model shows the elements of shell and membrane sections as the solid their section
gives them: the surface thickened by the #ui("Thickness"), with the mesh lying where the
#ui("Offset") puts it. The top face is the side the element normal points to, so an offset of
0.5 draws the solid below the mesh and −0.5 above it. Neighbouring elements whose normals point
to opposite sides show a step, which reveals a flipped normal before the analysis is run.

#screenshot("shell-thickness.png", [Shell sections drawn with their thickness: the plate with
offset 0.5 hangs below its mesh, the half tube with offset 0.5 lies inside it.])

The Results workspace shows the solids CalculiX expanded the shells to, so a section view
cuts through the thickness and shows how the stresses change across it.

#screenshot("shell-section-cut.png", [A section view through a shell roof and a beam in the
results: the stress S11 runs from the top to the bottom face of the shell.])

Results with `OUTPUT=2D` hold the shells as surfaces; the Results workspace then draws them
thickened like the FE model, with thickness and offset from the input file CalculiX ran, and
the value of each node across the whole thickness.

Turn the display off and on with #menu("View", "Shell Thickness") or in the
#link(<settings>)[settings] under #ui("Graphics"); the setting applies to both workspaces.
Picking still selects the elements on the mesh.
