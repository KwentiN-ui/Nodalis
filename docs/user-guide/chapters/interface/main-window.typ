#import "../../template.typ": *

== Analysis workspaces <main-window>

The Geometry, FE Model and Results workspaces show the active analysis (@workspaces). Their
layout is fixed; only the separators between the panes can be moved.

#screenshot("analysis-window.png", [An analysis in the FE Model workspace])

From top to bottom the window contains:

/ App bar and ribbon: the workspaces, the active analysis, the menus and the commands of the
  workspace (@app-bar, @ribbon). #ui("Edit") holds Undo, Redo (@undo) and the settings.
  #menu("Help", "User Guide") opens this guide in the web browser (@offline-guide).
/ Model tree: on the left, with the items of the workspace (@model-tree).
/ 3D view: in the centre (@view).
/ Properties panel: on the right, while an item is created or edited (@properties-panel).
/ Output pane: above the status bar, hidden until one of its tabs at the left of the status
  bar is clicked (@output-pane), with three tabs. #ui("Problems") lists what the model check
  finds (@problems). #ui("Output") collects messages: what was loaded
  or saved, warnings, keywords of an imported `.inp` that were skipped ("Not yet evaluated:
  ..."), unit conversions and the output of the query tool. #ui("Analysis") is the monitor of
  the solver (@monitor); it opens when a run starts.
/ Status bar: on the left what is loaded ("bracket.ndl: 12345 nodes, 6789 elements, 3 parts") or
  what is running ("Meshing ..."), on the right the unit system and the model space of the
  analysis.

=== Commands of the ribbon

The commands that used to be the toolbar of the analysis window are on these tabs of the
ribbon:

#fields(
  [Import Geometry], [#ui("Geometry") tab of the Geometry workspace. Imports STEP, IGES or
    BREP files and adds them to the geometry (@geometry-import). #ui("Import Mesh (.inp)")
    on the #ui("Mesh") tab of the FE Model workspace adds the mesh of input files as mesh
    parts (@mesh-import).],
  [Zoom to Fit, Standard Views, Vertical View], [#ui("View") tab, group #ui("Camera"). The
    #ui("Standard Views") buttons hold front, back, top, bottom, left, right, isometric and
    the views normal to an axis (@view); a 2D model has #ui("Rotate View") instead
    (@view-2d). #ui("Vertical View") rolls the view so that the axis
    closest to screen-up points straight up.],
  [Edges Only, Mesh Edges], [#ui("View") tab, group #ui("Show"). Show only the feature edges
    of the model, or all element edges. Exactly one of the two is active.],
  [Section View, Exploded View], [#ui("View") tab, group #ui("Cut"). They stay pressed while
    a section or an exploded view is active (@section-view, @exploded-view); #ui("Section
    Off") and #ui("Exploded Off") end them.],
  [Screenshot], [#ui("View") tab: copies the 3D view to the clipboard or saves it as PNG
    (@screenshot).],
  [Query, Remove Annotations], [#ui("View") tab in Geometry and FE Model, #ui("Evaluate") tab
    in Results. #ui("Query") opens and closes the query window (@query) and stays pressed while
    it is open; #ui("Remove Annotations") deletes all query labels of the model shown.],
  [Solve, Stop, Monitor], [On the right edge of the ribbon in Geometry and FE Model. Solve the
    analysis, stop meshing or the solver, and show the #ui("Analysis") tab of the output pane
    (@monitor). The state of the solution (#ui("Not solved"), #ui("Solution up to date"),
    #ui("Solution out of date") and so on) is shown in the app bar; with problems in the last
    run it names their number, and a click lists them.],
)

== Model tree <model-tree>

The tree on the left shows the items of the workspace:

/ Geometry: imported CAD parts and the mesh setup (@geometry).
/ FE Model: mesh parts, materials, sections, constraints, contacts and steps.
/ Results: the opened result files with their fields, history outputs, paths, planes and hot
  spots (@results).

Geometry and FE Model share one camera, Results has its own. A model that has no mesh yet
opens in the Geometry workspace, opening a result file switches to Results.

=== Working with tree items

- *Double-click* an item to open it in the properties panel (@properties-panel). A
  double-click on a container only opens or closes it; new items come from the ribbon.
- *Right-click* an item for its context menu (see below).
- *Del* deletes the selected items of the FE Model at once; a note at the bottom of the 3D view
  says what was deleted and offers #ui("Undo") for a few seconds (@delete-undo). #key("Ctrl+Z")
  brings them back as well (@undo). Parts and mesh setup items are deleted after asking.
- *Space* activates or deactivates the selected item while the mouse is over the tree.
- Selecting an item highlights its region in the 3D view. Master and slave regions of contacts and
  ties get different colours.
- Parts can be selected together: #key("Ctrl")+click toggles a part, #key("Shift")+click
  selects a range. The same works for the items of one container, for example many contact
  pairs: the 3D view highlights them all, and #ui("Delete Selected"), #ui("Activate Selected") /
  #ui("Deactivate Selected") (or #key("Del") and #key("Space")) and #ui("Swap Master/Slave of
  Selected") in the context menu act on all of them.

=== Context menus

/ Containers: #ui("Expand All"), #ui("Collapse All"), and #ui("Show All")/#ui("Hide All")
  where their items have symbols. Containers create no items and open no tools such as the
  material library: that is the job of the ribbon (@ribbon). An
  empty container is not shown; it appears with its first item. A container that cannot
  take items explains why in a tooltip, for example "A frequency step has no loads."
/ Items: #ui("Edit ..."), #ui("Activate")/#ui("Deactivate"), #ui("Swap Master/Slave") (ties,
  spring connections and contact pairs), #ui("Delete"). Items that come from another analysis
  through a link are shown in blue with the name of that analysis; their commands are greyed
  out, and a double-click offers to open that analysis or to unlink the item (@data-links).
/ Parts: #ui("Properties ..."), #ui("Hide")/#ui("Show"), #ui("Delete"); on the
  Geometry workspace also #ui("Create Mesh").

=== Deactivating items

Steps, boundary conditions, loads, constraints, contact pairs, node ties and initial conditions
can be deactivated with #ui("Deactivate") or #key("Space"). A deactivated item is shown grey
with a red no-entry sign; clicking the sign activates it again. Deactivated items stay in the
model but are written to the input file only as a comment, so CalculiX ignores them.

=== Renaming

Tree items are renamed in the #ui("Name") field of their dialog. Mesh parts are renamed in their
#ui("Properties") dialog. Names are converted to upper case and may contain letters (no umlauts),
digits, `_` and `-`, up to 80 characters. Items that refer to a renamed part, material or
amplitude follow the new name. Geometry parts cannot be renamed, because the mesh setup refers to
them by name.

=== What acts on a part <acting>

A click on a part in the 3D view selects it in the #ui("FE Model") tree, where the tree stays
where it is. Instead it marks with a blue frame the contact pairs and ties, constraints,
boundary conditions and loads whose region lies on the part, and opens their branches. A load
on a reference point counts for the parts of the rigid body the point drives. With several
parts selected the tree marks what acts on any of them.

#screenshot("part-acting.png", [BLOCK4 clicked in the 3D view: its two ties and the force on it
  are marked])

=== Properties dialogs

A double-click on a part opens its properties: name, number of elements and nodes, element types,
colour and visibility. In the FE Model workspace the element types can be changed
(@element-type). In the Results workspace, a double-click on #ui("Model") shows file,
load time, node and element counts, element types, the dimensions of the model and its sets and
surfaces. A double-click on a result component shows its maximum and minimum with the node where
they occur.
