#import "../../template.typ": *

= The User Interface

== Workspaces, app bar and ribbon <workspaces>

Nodalis works in one window. Its top line, the *app bar*, switches between four *workspaces*:

/ Project: the schematic with all analyses and blocks of the project (@project-window).
/ Geometry: the CAD parts and the mesh setup of the active analysis (@geometry).
/ FE Model: mesh parts, materials, connections and steps of the active analysis.
/ Results: the results of the active analysis (@results).

#screenshot("project-workspace.png", [The Project workspace: app bar, ribbon with the
  #ui("Sources") tab, and the schematic with two analyses and a materials block])

=== App bar <app-bar>

From left to right the app bar holds:

#fields(
  [Workspaces], [#ui("Project"), then #ui("Geometry"), #ui("FE Model") and #ui("Results").
    The last three show the analysis picked right of them.],
  [Analysis], [The active analysis. Picking another one here, or selecting its block in the
    schematic, makes it active; the Geometry, FE Model and Results workspaces then show it.
    The analysis shown before stays open in the background with everything it holds, so going
    back to it is immediate.],
  [Menus], [The menus of the workspace: those of the project window in the Project workspace,
    those of the analysis in the other three. In this version the menus still hold all
    commands; the analysis menus #ui("View"), #ui("Geometry"), #ui("Mesh"), #ui("Model"),
    #ui("Interaction"), #ui("Analysis"), #ui("Results") and #ui("Tools") are gathered under
    #ui("More"). The menu paths in this guide name them without #ui("More").],
  [Save, Undo, Redo], [Save the project (#key("Ctrl+S")), undo and redo (@undo). In the
    Project workspace undo and redo act on the schematic, otherwise on the analysis.],
  [Title], [The name of the project or the analysis.],
  [State], [A short summary: in the Project workspace how many analyses there are and how
    many are out of date, in the others the state of the solution. A click shows the
    solution monitor (@solution-monitor) or the #ui("Analysis") tab (@monitor).],
  [Search], [Finds any command by its name (@command-search).],
  [Settings, Help], [The settings (@settings) and this guide.],
)

=== Ribbon <ribbon>

Below the app bar, the *ribbon* shows the commands of the workspace in tabs. Each tab groups
its commands, with the name of each group below it. A click on a tile runs its command; a
tile with a small arrow opens a menu, for example #ui("Standard Views") on the #ui("View")
tab. #ui("Solve") is split: its icon solves, its label opens #ui("Solve Again") and
#ui("Campbell Diagram"). Tiles that cannot be used at the moment are greyed out, and the
tooltip of a tile names its shortcut and what it does.

#screenshot("ribbon-fe-model.png", [The ribbon of the FE Model workspace, #ui("Mesh") tab;
  #ui("Solve") on the right stays in place on every tab])

The group on the right edge stays the same on every tab of a workspace: #ui("Update Project"),
#ui("Stop") and #ui("Monitor") in the Project workspace, #ui("Solve"), #ui("Stop") and
#ui("Monitor") in Geometry and FE Model, where #ui("Check Model") joins them. A double-click on a tab folds the
ribbon to its tab row and back; so does #ui("Fold Ribbon") in its context menu.

#fields(
  [Project], [#ui("Project"): file commands, #ui("New Analysis"), and #ui("Open"),
    #ui("Open in New Window"), #ui("Solve"), #ui("Duplicate"), #ui("Delete"), #ui("Rename")
    for the selected block. #ui("Sources"), #ui("Parameters"), #ui("Evaluate") and
    #ui("Annotate") add blocks (@place-block). #ui("Arrange") works on the selected blocks and
    the view of the schematic.],
  [Geometry], [#ui("Geometry"): import, mesh setup, meshing and the parts. #ui("View"):
    camera, display, section and exploded view, screenshot, query.],
  [FE Model], [#ui("Mesh"), #ui("Material"), #ui("Features"), #ui("Connections"),
    #ui("Steps") and #ui("View").],
  [Results], [#ui("Results"): open and remove result files, the display controls of the
    results (@results-toolbar) and the exports. #ui("Evaluate"): query, hot spots, paths,
    planes, history outputs, derived fields and the Campbell diagram. #ui("View").],
)

=== Properties panel <properties-panel>

Items of an analysis are created and edited in the *properties panel* on the right of the
Geometry, FE Model and Results workspaces: materials, sections, steps, boundary conditions,
loads, constraints, contacts, amplitudes, outputs, features, mesh setup items, hot spots, the
model properties and the properties of a part. A tile of the ribbon or a double-click on an
item in the tree opens the panel; #ui("OK") or #ui("Cancel") at its foot, or the cross in
its head, close it. The 3D view shrinks while the panel is open, and its left edge can be
dragged to make it wider.

The panel holds one editor at a time. Opening another item replaces the one shown; changes to
it that were not confirmed with #ui("OK") are dropped. While a region is picked, the
#ui("Selection") window stands left of the panel, over the 3D view.

#screenshot("properties-panel.png", [A new boundary condition in the properties panel; it
  goes into #ui("Step-1"), the active step])

=== Active step <active-step>

Boundary conditions, loads, field and history outputs and defined fields belong to a step. The
*active step* decides which: the combo box #ui("Active Step") at the left of the
#ui("Steps") tab shows it, and the tiles of that tab create their items in it. In the tree
the active step is marked #ui("(active)") when there are several steps, and the 3D view shows
its symbols.

#screenshot("active-step.png", [The #ui("Steps") tab with the active step])

- Picking a step in the combo box makes it active; #ui("New Step ...") at its end creates one.
- Selecting a step, or an item of a step, in the tree makes that step active.
- A new step becomes active once it is created.
- Without a step the combo box says #ui("No step"), and only #ui("Step") can be used.
- The #ui("Step") row at the top of a boundary condition or load moves it to another step.

=== Checklist <checklist>

While nothing is edited, the properties panel shows the *checklist* of the workspace: what the
model still needs, ticked off once it is there. Geometry asks for parts and a mesh, FE Model for
a material, a section, a step, supports and loads, Results for a solution. A button next to an
open step runs the command that does it. When the model check finds problems, a link below the
list shows them in the #ui("Problems") tab.

#screenshot("checklist.png", [The checklist of the FE Model workspace with a material defined])

The cross in the head of the panel hides the checklist, and the 3D view takes the room;
#ui("Checklist") on the #ui("View") tab shows it again.

=== Problems <problems>

The #ui("Problems") tab below the 3D view lists what the model check (@model-check) and the
last check by the solver found, errors first; its title counts them. The tooltip of an entry
says why it is a problem and how to fix it. A click selects the item in the tree and the 3D
view, a double-click opens it in the properties panel. A click on the state in the app bar
shows the tab when there are problems.

#screenshot("problems.png", [The #ui("Problems") tab with the tooltip of an error])

=== Deleting and undo <delete-undo>

#key("Delete") or #ui("Delete") deletes the selected items of the FE Model and the selected
blocks of the schematic without asking. A note at the bottom of the 3D view or the schematic
names what was deleted; its #ui("Undo") brings it back, as does #key("Ctrl+Z") at any time
(@undo). The note goes away after a few seconds.

#screenshot("delete-undo.png", [The note after deleting a material])

=== Placing blocks <place-block>

The tiles on the #ui("Sources"), #ui("Parameters"), #ui("Evaluate") and #ui("Annotate") tabs
of the Project workspace add blocks to the schematic. After a click on the tile the block
hangs at the mouse pointer; a click into the schematic places it there. #key("Esc") or a
second click on the tile cancels. #ui("New Analysis") works the same way.

#screenshot("place-block.png", [A materials block hanging at the pointer, after a click on
  #ui("Materials")])

=== Command search <command-search>

#key("Ctrl+K") or a click into the search field of the app bar opens the command search. It
finds the commands of all workspaces by their name, group, tab and tooltip; those of the
current workspace come first. #key("Enter") or a click runs the command and switches to its
workspace when needed.

#screenshot("command-search.png", [Command search for "contact"])

=== Analyses in a window of their own

An analysis can also be opened in a separate window, for example on a second screen:
#ui("Open in New Window") on the #ui("Project") tab or in the context menu of its block. That
window has its own app bar with the workspaces #ui("Geometry"), #ui("FE Model") and
#ui("Results"). Closing it keeps the analysis in the project; it then opens in the main window again.
