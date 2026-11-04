#import "../../template.typ": *

== Undo and redo <undo>

#menu("Edit", "Undo") (#key("Ctrl+Z")) takes back the last change of the model,
#menu("Edit", "Redo") (#key("Ctrl+Shift+Z")) makes the last undone change again. Every change
counts once, however it was made: an item created, edited in its dialog, deleted, moved,
activated or deactivated, a part renamed or deleted, a mesh setup item changed, the mesh
generated anew or edited with a mesh tool. A new change drops what could be redone. Undo
returns to the state at each change.

#screenshot("undo.png", [The Edit menu after a deleted material])

The shortcuts act on the model when no text field has the focus; in a text field,
#key("Ctrl+Z") undoes the typing there. Undo and Redo are greyed out while a dialog is open,
while meshing runs and in the Results workspace; the tooltip of the greyed-out entry says why.
The output pane notes each undo and redo.

The last 100 changes can be undone, of which at most 10 that replaced the mesh, as each keeps a
copy of the mesh. The history starts empty for each model opened or created.

#limitation[Changes of the geometry's shapes cannot be undone: an import into an existing model,
deleted geometry parts and the conversion to another unit system clear the history. Results,
their settings and the view are not part of the history either.]

=== In the project window <project-undo>

The project window has its own #menu("Edit", "Undo") (#key("Ctrl+Z")) and #menu("Edit", "Redo")
(#key("Ctrl+Shift+Z") or #key("Ctrl+Y")) for the schematic: blocks added, copied, deleted or
renamed, blocks moved, links drawn or removed, and what is entered in the dialogs of the blocks
(parameters, measures, parameter studies, mesh and material blocks, new analyses). A deleted block comes
back with its links and everything it held. The output pane notes each undo and redo; the last
100 changes can be undone, and the history starts empty for each project opened or created.

#screenshot("project-undo.png", [Edit > Undo in the project window after a block was moved])

Changes made inside an analysis are undone in its window (above). A change that would replace
the data of an analysis whose window is solving, meshing or loading waits: the output pane says
so, and the change can be undone once the window is idle. A solve is no change that can be
undone.
