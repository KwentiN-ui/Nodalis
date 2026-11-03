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
