#import "../../template.typ": *

== Bolt pre-tension <pre-tension>

A #ui("Pre-tension (bolt preload)") load tightens a bolt: it pulls the two sides of a cut through
the shank together with a force, or shortens the bolt by a given length. Create it with #ui("Load") on the #ui("Steps") tab.

#screenshot("pre-tension-load.png", [Pre-tension load on the cut through a bolt shank])

#fields(
  [Region], [The element faces on *one* side of a cut through the bolt shank, picked in the 3D
    view or as a surface. The cut must lie inside the bolt mesh: split the shank with a plane
    in the CAD geometry or pick the faces of one element layer.],
  [Preload by], [#ui("Force") (default), #ui("Displacement") or #ui("Lock").],
  [Force], [The force pulling the two sides together; a positive value tightens the bolt.],
  [Shortening], [The length the bolt is shortened by since the start of the analysis, when
    preloading by displacement. A shortening of 0 in a later step loosens the bolt again.],
  [Lock], [Keeps the length the bolt has at the end of the previous step (`*BOUNDARY, FIXED`):
    the bolt then works like a stiff part of the joint, and its force follows the other loads.],
  [Direction], [The direction of the bolt axis. Unticked, Nodalis uses the normal of the
    selected faces; #ui("Given") takes the components #ui("X"), #ui("Y"), #ui("Z").],
)

Nodalis cuts the mesh apart at the selected faces when it writes the input file: every
element on their side, also one that only touches the cut with an edge or a corner, gets copies
of the nodes on the cut. `*Equation`s tie the copies to the other side the way CalculiX's
`*PRE-TENSION SECTION` does: the gap normal to the cut is the first degree of freedom of a new
node per load, the sides cannot slide against each other. The preload is a `*CLOAD` with the
force or a `*BOUNDARY` with the shortening on this node. The same load name in a later step
reuses the node.

For the usual "preload, then load" sequence of a bolted joint, tighten the bolt by a force in the
first step and copy the load into the following steps with #ui("Lock"). A step without the load
leaves the bolt loose. The model check warns of a bolt locked in the first step it acts in: there
is no previous length to keep then, and the bolt carries no preload.

#note[CalculiX's `*PRE-TENSION SECTION` separates only the elements with a
face on the cut. In a tetrahedral mesh, elements that touch the cut with just an edge or a
corner then hold both sides together and take most of the preload, which is why a thin layer of wedge elements is often
added at the cut (in PrePoMax #menu("Model", "Tools", "Boundary Layer")).
Nodalis needs no such layer: its own cut separates all of them.]

#note[A bolt split into two parts in the CAD geometry has two meshes that do not share nodes at
the cut: connect the halves with a #ui("Tie") (@ties) between the two cut faces and put the
pre-tension load on the faces of one of them. Nodalis writes the nodes of the cut side as the
slave of the tie, whichever side you made the master, so the preload passes through it.]

A pre-tension load can follow an #ui("Amplitude") (@amplitudes). The reaction force at the
pre-tension node is the bolt force: Nodalis prints it (`RF1`) and the shortening (`U1`) into the
history results under the name of the load in every step that has the load (@history-outputs).

#note[A #ui("Bolt") under #ui("Connections") (@bolts) is the simpler way for most joints: pick
the faces on both sides of the cut and give the preload. It ties two bolt halves itself, locks
the bolt after the preload step and replaces the contact region found on the cut.]

#note[The preload acts across the cut only. Hold the bolt against rigid body motion with the
contact or tie to the clamped parts, not with a support on the cut faces.]
