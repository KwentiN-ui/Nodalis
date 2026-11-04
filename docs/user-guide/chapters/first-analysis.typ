#import "../template.typ": *

= A First Analysis

This chapter walks through a complete static analysis of a plate with a hole that is fixed at one
end and pulled at the other. Each step links to the chapter with the details.

+ *Create the analysis.* A new project starts with one empty analysis, #ui("Analysis-1"),
  already picked in the app bar; click #ui("Geometry") there to work on it. Its model space and unit system come from the settings;
  keep #ui("3D") and #ui("mm, ton, s, °C"). Further analyses are added with
  #ui("New Analysis") in the Project workspace (@new-analysis, @workspaces).
+ *Import the geometry.* #ui("Import Geometry") on the ribbon (or #menu("Geometry", "Import
  ...")) reads the STEP file of the plate. It
  appears as part `SOLID-1` in the Geometry workspace (@geometry-import).
+ *Mesh it.* #ui("Default Parameters") on the ribbon sets the element size, #ui("Mesh All
  Parts") creates quadratic tetrahedra. Nodalis switches to the FE Model workspace
  (@meshing).
+ *Material and section.* #ui("Material") on the #ui("Material") tab of the ribbon with density, Young's modulus
  210000 MPa and Poisson's ratio 0.3, or copy steel from the material library (@materials). Then
  #ui("Section") assigns it to the part (@sections).
+ *Step.* #ui("Step") on the #ui("Steps") tab with type #ui("Static") (@steps).
+ *Support.* #ui("Boundary Condition"), type #ui("Displacement/Rotation"), which
  holds all components at zero; click the end
  face of the plate in the 3D view (@boundary-conditions, @selection).
+ *Load.* #ui("Load"), type #ui("Force") with `F1` = 20000 N on
  the opposite end face (@loads).
+ *Solve.* #ui("Solve") on the right of the ribbon or #key("F5"). The #ui("Analysis") tab below the 3D view shows
  the output of the solver (@monitor). If the tree shows warning signs, click them first (@model-check).
+ *Look at the results.* The results open when the run is done; otherwise
  #ui("Results") in the #ui("Analysis") tab opens them. Select #ui("STRESS") >
  #ui("MISES") in the tree to see the von Mises stress around the hole (@results).
+ *Save the project.* The save button of the app bar or #key("Ctrl+S") writes a `.ndl` file with all analyses of
  the project: geometry, mesh, model and results. The schematic of the Project workspace now shows the analysis
  as up to date; change a load, and its solution turns out of date until you solve again. Renaming or hiding
  an item, or adding a result path, changes nothing the solver computes, so the results stay up to date.

#screenshot("main-window.png", [The plate with support (green) and load (blue arrow) in the FE Model workspace])
