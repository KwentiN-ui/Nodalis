#import "../../template.typ": *

== Constraints <constraints>

Parts meshed separately share no nodes. Constraints connect them or attach springs to the model.
Create them with #ui("Constraint") on the #ui("Connections") tab of the ribbon.

#screenshot("constraint.png", [Spring dialog: springs from a surface to ground])

#fields(
  [Spring], [Springs in the global directions on the nodes of the region. #ui("K1"),
    #ui("K2"), #ui("K3") are the stiffnesses; 0 means no spring in that direction. Like a
    force, the stiffness is that of the whole region and is spread over its nodes: on faces
    by area, on edges by length, on points in equal shares. #ui("Per area") gives a
    stiffness per area of the faces instead (force per volume, a bedding modulus). With
    #ui("Apply to all nodes individually") every node gets the whole stiffness. A single
    node or point is a point spring. In a 2D model (plane stress, plane strain,
    axisymmetric) there is no #ui("K3"): the dialog offers the first two directions only
    (@spring-2d).],
  [Compression Only], [A support that only takes pressure, like a part resting on a rigid floor.
    By default it pushes onto the surface from outside; #ui("Flip") under #ui("Direction")
    makes it push the surface outward from inside the material. #ui("Clearance") is the
    initial gap. #ui("Spring stiffness") and #ui("Tensile force") are optional and belong to
    the whole surface, spread over its nodes by area; with
    #ui("Apply to all nodes individually") every node gets the whole value. Without a value
    every gap gets CalculiX's default, practically rigid. With #ui("Nonlinear") the contact
    is solved nonlinearly, otherwise it is linearised in a linear step. Written as `GAPUNI`
    elements to fixed ground nodes, one per node.],
)

Changing the #ui("Type") while creating a constraint keeps the picks the new type can use. A
spring that you picked faces for becomes a Compression Only support with the same faces; picks
that do not fit, such as nodes for a support, are cleared with a short note. Loads and history
outputs behave the same when you change their type.

#screenshot("constraint-type-switch.png", [The faces picked for a spring are still selected after switching to Compression Only])

#ui("Connection") at the bottom of the spring dialog decides where the springs end.
#ui("Surface to World") connects them to ground (`SPRING1` elements). #ui("Surface to Surface")
adds a #ui("Target surface"): every node of the region is connected to the closest point of
the target surface (`SPRING2` elements and `*EQUATION`s), for example an elastic bearing
between two parts. The region is the slave and the target surface the master side: choose the
finer mesh as region. #ui("Swap Master/Slave") in the context menu exchanges the two.

#screenshot("constraint-spring-surface.png", [Spring between two surfaces: the target surface
  in red, the region in purple])

#screenshot("constraint-spring-2d.png", [Spring in a plane stress model: only #ui("K1") and
  #ui("K2")]) <spring-2d>

The 3D view of the FE model shows active constraints in yellow, together with
the symbols of any step: a coil spring for each direction with stiffness, at the centre of
the faces of a spring to ground or at each of its nodes (at their centre for ten nodes or
more), and at points spread over the region of a spring to a surface; cones with a ground plate
on the side a compression only support pushes from (@compression-only); and lines to the
reference point of a rigid body (@rigid-body). The selected or edited constraint is red. Ties
have no symbol.

#screenshot("constraint-symbols.png", [Springs on nodes and on a surface, compression only support
  and rigid body on a cantilever])

#ui("Hide") in the context menu of a constraint removes its symbol from the 3D view, #ui("Show")
brings it back; the tree marks a hidden constraint with an eye. #ui("Hide All") and
#ui("Show All") in the context menu of #ui("Constraints") do this for all constraints at once.
A hidden constraint is still drawn while it is selected, and the setting is saved
with the project. Hiding changes only the view, not the analysis; #ui("Deactivate") leaves a
constraint out of the input file.

#screenshot("compression-only-dialog.png", [Compression only support with flipped direction])
  <compression-only>

#screenshot("compression-only.png", [Front view of a cantilever: a compression only support
  pushing onto the end face from outside (right) and a flipped one pushing the top face
  outward from inside the material])

#screenshot("constraint-hide.png", [Hidden spring and the context menu of Constraints])

#limitation[User-defined equations are not available yet.]

== Contacts <contacts>

Contacts and glued joints between parts are listed under #ui("Connections") in the tree of the FE
model. The usual way is a contact region: you say which two regions touch and how they behave,
and Nodalis works out how CalculiX should model it. The manual objects of CalculiX (surface
interactions, contact pairs, ties) are still there under #ui("Connections") #sym.arrow.r
#ui("Advanced") and in the #ui("Advanced") group of the #ui("Connections") tab.

=== Contact regions <contact-regions>

#menu("Connections", "Contact Region") connects two regions, #ui("Side A") and #ui("Side B"). Which
side is A does not matter: Nodalis chooses the master side itself.

#screenshot("contact-region-dialog.png", [A frictional contact region with its advanced settings.
  Side A is red in the 3D view, side B blue; #ui("Exported as") shows what Nodalis writes for
  CalculiX and why])

#fields(
  [Type], [#ui("Bonded"): the sides are glued together, no gap and no sliding.
    #ui("No Separation"): no gap opens, but the sides slide freely along each other.
    #ui("Frictionless"): the sides may separate and slide freely, but not penetrate.
    #ui("Frictional"): like frictionless, but the sides stick until the shear exceeds the
    #ui("Friction coefficient") times the pressure. #ui("Rough"): the sides may separate but
    never slide. The grey text below the type says what it means for the analysis.],
  [Side A / Side B], [The two regions, picked like the master and slave of a contact pair. A
    region usually is the touching faces of one part.],
  [From definition], [Names the region by its type and parts, e.g.
    #ui("Frictional - Block to Cap"). A name that starts with the type follows it when the
    type changes, here or in the worksheet.],
  [Exported as], [What Nodalis writes for CalculiX, with the derived stiffness and the reasons
    below in grey.],
)

#ui("Advanced settings") shows what Nodalis derived. Every setting starts at #ui("Auto"), which
shows the derived value in brackets; choosing something else overrides it for this region only.

#fields(
  [Master], [The master side. Auto takes the side with the coarser mesh, then the stiffer
    material, then the larger area.],
  [Bonded as], [For #ui("Bonded"): #ui("Constraint (tie)") ties the slave nodes to the master
    faces (`*TIE`), linear and exact. #ui("Contact (tied contact)") glues them with a stiff
    tied penalty contact. Auto uses a tie.],
  [Normal stiffness], [The penalty stiffness of a contact. Auto is 10 #sym.times E / h, E the
    Young's modulus of the softer side and h the mesh size of the slave side, 100 #sym.times E / h
    for a tied contact. As a factor of E / h or as a value (pressure per length).],
  [Tangential stiffness], [The stiffness while the sides stick (stick slope). Auto is the normal
    stiffness / 100 for a frictional contact and the normal stiffness for rough and tied
    contacts.],
  [Initial gap], [#ui("Adjust to touch") moves the slave nodes onto the master side at the start,
    so that small gaps from meshing are closed. #ui("As is") keeps the mesh. #ui("Offset")
    sets the distance of the sides wherever they face each other, whatever the mesh: negative
    for an interference fit, such as a shaft pressed into a hub (`*CLEARANCE`, surface to
    surface contacts only). Auto adjusts bonded regions and keeps the others as they are.],
  [Tolerance], [Distance within which the sides count as touching: tied by a tie, held by no
    separation, closed by the adjustment. Auto is a quarter of the slave mesh size, for a
    region a detection found at least the distance the detection searched within.],
  [Heat conductance], [Shown when a step solves temperatures and displacements together: the
    heat flow across the closed contact per area and temperature difference. Auto is
    100 #sym.times k / h, k the thermal conductivity of the less conducting side: so much that
    the contact hardly shows in the temperatures. A bonded tie conducts heat anyway.],
  [Small sliding], [Only for the node to surface formulation, see below.],
)

#screenshot("contact-offset-dialog.png", [The advanced settings of a frictional region with an
  offset of -0.01 mm: the sides press into each other by 0.01 mm like an interference fit])

#ui("No Separation") is written without contact iterations: one equation per slave node within
the tolerance holds it to the master side normal to the surface, while it slides freely along
it. The equations stay linear, so the region needs no contact pair and no stiffness, but they
hold for small displacements only: the model check warns about a region of no separation in a
step with large deformations. A part held only by no separation can still slide, so it needs
other supports along the surface.

#screenshot("contact-no-separation.png", [A region of no separation in the worksheet, written
  as constraint equations])

The stiffness is derived from E / h, so a model gives the same penetration whether its units are
millimetres or metres. Bonded regions whose slave sides share nodes are written as one tie.

==== Contact detection <contact-detection>

Instead of picking every pair of sides by hand, let Nodalis find them:
#menu("Connections", "Detect Contacts") creates a contact detection. #ui("OK") searches the parts for surfaces that
touch and adds a contact region for every pair it finds, listed under the detection in the tree.

#screenshot("contact-detection-dialog.png", [A contact detection of all visible parts with the
  derived tolerance])

#fields(
  [Parts], [#ui("All visible parts"), or untick it and choose the parts to search, e.g. only the
    bolts and the plates they hold.],
  [Tolerance], [Surfaces closer than this touch. Auto is 0.5 % of the size of the parts
    searched, but at least half their element size, so that two meshes on a curved face still
    meet (on the geometry, see @contact-detection-geometry, just the 0.5 %). The regions found get it as their tolerance, so that a tie catches every node the
    detection saw touching.],
  [Angle], [Largest angle between the surfaces facing each other (default 35°).],
  [Surfaces], [What is searched in 3D: outer faces of #ui("Solid") parts, the top and bottom of
    #ui("Shell") and membrane parts, and their free edges (#ui("Shell edge")).],
  [Group by], [#ui("Parts") (default): one region per pair of touching parts. #ui("Faces"): one
    per pair of touching faces. #ui("Graph"): touching faces sharing nodes are merged, so that
    every node is on the slave side of one region only.],
  [Type], [The type of the regions found, #ui("Bonded") by default.],
)

The regions found are named from their definition, e.g. #ui("Bonded - Base to Block"), and work
like regions made by hand. #ui("Detect Again") in the context menu of the detection searches
again, after the geometry or the mesh changed, and replaces the regions it found before. Regions
you changed since (in their dialog or in the worksheet) stay as they are; the worksheet shows
them as #ui("changed"), and the output window names those that are no longer found. Several
detections can live side by side, e.g. one for bonded housings and one for frictional bolt
heads. Deleting a detection deletes the regions it found, except changed ones, which stay
under #ui("Contact Regions").

#screenshot("contact-detection-tree.png", [The regions found by the detection #ui("Contacts");
  the frictional one was changed after the detection and stays when detecting again])

==== Contact detection on the geometry <contact-detection-geometry>

A model made from CAD geometry is searched on the geometry itself, meshed or not, as in Ansys:
you can detect the contacts right after the import, before meshing. A region found this way
takes the whole CAD faces that touch, or the whole CAD edges for the free edges of shells and
in 2D models, also where only part of a face is in contact, e.g. the top of a plate a small
block stands on. The regions keep these faces when the parts are meshed again, finer or with
other settings, and #ui("Detect Again") finds the same regions. The dialog says so below the
type.

#screenshot("contact-detection-geometry.png", [A detection of a model that is not meshed yet:
  it searches the CAD geometry])

On the geometry, #ui("Auto") for the tolerance is 0.5 % of the size of the parts searched;
the triangles that show the faces say nothing about the mesh to come. The regions found keep
#ui("Auto") as their own tolerance: a quarter of the slave side's element size (see
@contact-regions), but at least the distance the detection searched within, so that a tie
catches every node of faces the detection found touching, also across a small gap. A model with parts that do not come from the
geometry, such as an imported mesh, is searched on its mesh as before.

#screenshot("contact-detection-geometry-region.png", [The region found before meshing, shown
  on the mesh: the whole top face of the plate (side A) against the bottom of the block])

==== Connection settings

#menu("Connections", "Connection Settings") or the context menu of #ui("Connections") sets what
all contact regions share. CalculiX allows only one contact #ui("Formulation") per model (see
"Which method?" below). Auto takes surface to surface, or the method of your manual contact
pairs if you have any. The #ui("Stiffness factor") replaces the 10 of the normal stiffness.
The model check reports a manual contact pair whose method differs from the formulation, and
mortar contact in a step other than static.

#screenshot("connection-settings-weak-springs.png", [Connection settings with weak springs for
  parts held only by contact])

===== Weak springs <weak-springs>

A part that only contacts hold, such as a block pressed onto another one with nothing else
fixing it, can move freely until its contacts close, and frictionless contact never holds it
along the surfaces. CalculiX often gives up on such a model with "too many cutbacks": with an
initial gap, with frictionless contact in a step with Nlgeom, and node to surface contact
lets the part drift sideways by arbitrary amounts. #ui("Weak springs") (on by default) gives
every such part very soft springs to ground in the input file: in each direction 10#super[-6]
E L in total, spread over all nodes of the part, E its Young's modulus and L its size (the
geometric mean of its extents). The springs are not shown in the tree, and the model check no
longer reports these parts. They count for contact pairs you define by hand too.

The springs carry about a millionth of the load once the contacts are closed, and the force
it takes to close an initial gap g, of the order 10#super[-6] E g L. Two things to keep in mind:

- A part that really is a mechanism, e.g. pushed sideways on a frictionless contact, no longer
  aborts but moves very far: the springs alone hold it. Huge displacements of a part held only
  by contact mean that something is missing, not that the result is right.
- In a frequency step after a static step the free motions of these parts are no longer at
  0 Hz but at a few thousandths of the first eigenfrequency of the part.

Turn #ui("Weak springs") off to see the warning again and hold the parts yourself, e.g. with a
#ui("Spring") constraint (@constraints).

==== Contact worksheet

#menu("Connections", "Contact Worksheet") or #ui("Contact Worksheet ...") in the context menu
of #ui("Connections") lists all contact regions in a table: type, sides, master and what each
is exported as, and which detection found it. A click on a row selects the region in the tree, #key("Ctrl")+click and
#key("Shift")+click select more. The buttons change all selected regions at once: set the type
or the friction coefficient, flip the sides, activate or deactivate, name them from their
definition, or delete them.

#screenshot("contact-worksheet.png", [The contact worksheet after a run: the bonded region
  is closed, the frictional one partly closed, and after solving it sticks and slides])

==== Contact check <contact-check>

Nodalis checks every contact region before you solve: for each node of the slave side it
measures the distance to the master side, along the outward normal of solid faces (negative
means the sides overlap). A node counts as touching when it lies within the tolerance of the
region: its tie tolerance or adjustment distance, or else a quarter of the mesh size. The
region then is

- #ui("Closed") when all slave nodes touch,
- #ui("Partly closed") when some do,
- #ui("Near open") when none does, but the gap is less than ten times the tolerance,
- #ui("Far open") when the sides are further apart.

The worksheet shows the status in #ui("Initial status"), the share of touching slave nodes in
#ui("Touching") and the smallest and largest gap in #ui("Gap"); the dialog of a region shows
the same in its #ui("Contact check") row. An open region carries no load until the parts move
onto each other, so the model check warns about every active region that is near or far open:
a bonded region that is open ties nothing, and a frictional one leaves the parts free to fly
apart unless supports hold them.

#screenshot("contact-check-dialog.png", [The contact check in the dialog of a frictional
  region: the larger block top is the slave side, so only part of it touches the cap])

Every region exported as a contact pair also writes contact results (#ui("CDIS") and
#ui("CSTR")) in static, visco and dynamic steps that do not ask for them already. After a run
the worksheet adds the column #ui("After solving"): whether the region stays open, sticks,
slides or does both, and on hover how many slave nodes are in contact, how many slide, the
largest pressure and the largest opening. A node slides when its shear stress reaches the
friction coefficient times its pressure.

==== Contacts in steps <contact-steps>

Contact regions belong to the model, but a step can switch one off or on, or give a frictional
one another friction coefficient: #ui("Contacts in this step") in the dialog of a static,
visco, dynamic or temperature-displacement step lists the active regions as they are in that
step. A change holds from its step on, in the following steps too, until another step changes
the region again; #ui("changed here") marks what the step changes. Renaming a region keeps its
changes, deleting it removes them.

#screenshot("contact-step-changes.png", [Step-2 switches the bonded region off and raises the
  friction of the frictional one to 0.3])

CalculiX switches contact pairs only (`*MODEL CHANGE`, `*CHANGE FRICTION`). A bonded region a
step switches is therefore written as a tied contact instead of a tie, and a region whose
friction a step changes gets a surface interaction of its own. A bonded region chosen as a
tie, a region of no separation and a region deactivated as a whole cannot change in steps;
the model check reports them.

==== Contact outputs <contact-outputs>

Right-click a contact region or a contact pair in the tree to create its outputs directly:
#ui("Create Contact History Output") and #ui("Create Contact Field Output"). With one step the
entry opens the dialog right away, with several steps a submenu asks for the step.

#screenshot("contact-output-menu.png", [Contact outputs in the context menu of a contact region])

The history output dialog opens prefilled: type #ui("Contact Output"), the variables `CDIS`,
`CSTR` and `CF` and the contact as its pair. After the run the `.dat` file holds the relative
displacements and stresses of the contact and, with `CF`, the total contact force and moment
in every increment (@history-outputs). Like every history output it stays in effect in the
following steps (see History outputs in the step chapter).

#screenshot("contact-output-dialog.png", [Prefilled contact history output of a bonded region])

The contact field output (`*CONTACT FILE`) writes the pressure, slip and gap of all contacts into
the `.frd` file; CalculiX knows no field output for one contact only. A step that writes contact
field outputs already opens its output instead of creating a second one (@step-field-outputs).

Only contact pairs have contact values. A bonded region with a contact history output is
therefore written as a tied contact instead of a tie when its formulation allows it. A bonded
region chosen as a tie, a region of no separation and a region deactivated as a whole have no
contact elements; the model check warns that their history output prints nothing, and the
export leaves it out.

=== Bolts <bolts>

A bolt joins the two sides of a cut through a bolt shank and tightens them with a preload.
Create it with #menu("Connections", "Bolt") or #ui("Create Bolt ...") in the context
menu of #ui("Connections"); it is listed under #ui("Connections") #sym.arrow.r #ui("Bolts").
You pick the faces on the two sides of the cut and give the preload, Nodalis works out the
rest.

#screenshot("bolt-dialog.png", [A bolt between two parts. The bonded contact region the contact
  detection found on the cut is replaced by the bolt and shown greyed out in the tree])

#fields(
  [Side A, Side B], [The faces on the two sides of the cut. For a bolt split into two parts in
    the CAD geometry these are the two cut faces, for a bolt meshed in one piece the element
    faces on one side of the cut (side B stays empty). For a bolt that is not cut at all, pick
    only the cylindrical face of its shank as side A, see below.],
  [Preload by], [#ui("Force") pulls the two sides together with the preload, #ui("Shortening")
    shortens the bolt by a length.],
  [Axis], [The direction of the bolt axis. Unticked, Nodalis uses the normal of the cut;
    #ui("Given") takes the components #ui("X"), #ui("Y"), #ui("Z").],
  [Steps], [What the bolt does in each step that takes loads, see below.],
  [Exported as], [How Nodalis writes the bolt for CalculiX: where it cuts the mesh, whether it
    ties the two halves and which contact regions the bolt replaces.],
)

==== Steps of a bolt

#ui("Auto") does what a bolted joint usually needs: the bolt is tightened with the preload in
the first step that takes loads and locked at its length in all steps after it, so its force
then follows the other loads. Steps before the preload step leave it loose. A step can choose
an action of its own instead:

#fields(
  [Load], [Pulled together by a force, by default the preload.],
  [Adjustment], [Shortened by a length since the start of the analysis.],
  [Lock], [Kept at its length at the end of the previous step (`*BOUNDARY, FIXED`).],
  [Open], [Loose: no preload, the cut carries no force.],
)

==== Picking the shank

A bolt does not need to be cut in the CAD geometry: pick the cylindrical face of the free
shank, between head and nut, as #ui("Side A") and leave #ui("Side B") empty. Nodalis finds
the axis of the shank from the face and cuts the bolt across the middle of the face. The cut
follows the element faces of the mesh, like a staircase in a tetrahedral mesh, so the bolt
needs no new mesh; the preload acts along the axis. #ui("Exported as") says when the shank was
recognised. Hide the clamped parts (@view) to reach the shank in the 3D view.

#screenshot("bolt-shank.png", [Only the shank of an uncut bolt picked: Nodalis cuts it across
  its middle])

The bolt must be one part of its own, at least where the shank is. Its contact with the hole
must let the shank slide: the contact detection usually finds the head, the nut and the shank
as one bonded region with the clamped parts. The model check then warns at the bolt that the
region holds it at its cut; make that region frictional or frictionless.

==== What Nodalis writes

When the two sides share their nodes (one mesh through the cut), Nodalis cuts the mesh between
them. For two parts it cuts the side with the finer mesh and ties its nodes to the other side;
the sides cannot slide against each other. The preload is the first degree of freedom of a new
node, as with a pre-tension load (@pre-tension).

The contact detection (@contact-detection) usually finds the two cut faces as a bonded contact
region. A tie or contact on the cut next to the bolt would hold the halves together and keep
CalculiX from converging, so the bolt replaces every contact region that lies on its cut: such
a region is greyed out in the tree with #ui("(replaced by ...)") and left out of the input
file. Contacts of the head and the nut with the clamped parts stay as they are; the model check
warns when a bonded or rough region holds the bolt at its cut, and when two bolts or a bolt and
a tie share faces.

After the run the bolt force (`RF1`) and the shortening (`U1`) of every bolt are in the history
results under the bolt's name in capitals, spaces written as underscores (`BOLT_M10`), in each
step in which the bolt is tightened or locked (@history-outputs). An open bolt carries no force.

#screenshot("bolt-force.png", [The force of a bolt in the preload step and, locked, in the
  operation step])

=== Surface interactions

#menu("Connections", "Surface Interaction") defines the contact behaviour
that contact pairs refer to. Move the models you need from #ui("Available") to #ui("Selected").

#screenshot("surface-interaction.png", [Surface interaction with surface behaviour])

#fields(
  [Surface Behavior], [Pressure-overclosure relation, the contact pressure over the overclosure
    (penetration) of the surfaces. #ui("Hard") is the usual rigid contact (see below). The others are soft
    contacts: #ui("Linear") (stiffness K, about 5 to 50 times Young's modulus per length, and
    tension at large clearance σ#sub[∞]), #ui("Exponential") (pressure p#sub[0] at zero gap,
    dropping to 1 % at gap c#sub[0]; helps convergence from open contact), #ui("Tabular") (your
    own curve) or #ui("Tied") (the surfaces stay connected with stiffness K, see below).],
  [Friction], [Coulomb friction: the tangential force is limited to #ui("Friction coefficient")
    times the contact pressure. The optional #ui("Stick slope") is the stiffness while sticking;
    if empty, Nodalis writes 1/100 of the normal contact stiffness (see below). Friction makes
    the analysis more nonlinear, so start with
    small coefficients and enough increments.],
  [Gap Conductance], [Heat transfer across the contact, constant or as a table over pressure
    and temperature, for thermal and coupled steps. Without it the contact is thermally
    insulating.],
)

A #ui("Tied") surface behavior glues the surfaces together, like a Tie, but as a penalty contact.
CalculiX needs #ui("Friction") with it in the same surface interaction: the #ui("Stick slope")
is the stiffness of the tie along the surfaces (the tie's own #ui("K") if empty), the friction
coefficient is irrelevant. Without friction the run
aborts. So choosing #ui("Tied") adds #ui("Friction") to the selected models at once, and the
dialog does not accept a tied interaction without it. Tied contact works only with the method
#ui("Surface to surface"). The model check reports contact pairs that use it otherwise.

#screenshot("surface-interaction-tied.png", [Choosing Tied adds Friction])

==== Contact stiffness

With the methods #ui("Node to surface") and #ui("Surface to surface") CalculiX enforces a
#ui("Hard") contact with a stiff spring (penalty contact), so the surfaces penetrate a little.
CalculiX's own default stiffness, 50 times Young's modulus, ignores the element size and
therefore depends on the unit system: Nodalis writes the input file in SI units, where it would
be about 1000 times softer than in mm. Nodalis therefore writes the spring itself, the same in
every unit system:

- Normal stiffness K = 10 E / h, with E the smaller Young's modulus of the two sides and h the
  mean edge length of the slave faces. Under 100 MPa a steel contact with 1 mm elements
  penetrates about 0.05 µm. If several contact pairs share a surface interaction, they all get
  the softest of their stiffnesses.
- An empty #ui("Stick slope") is K / 100, as the CalculiX manual recommends. For a #ui("Linear")
  behavior K is its own #ui("K"), and for #ui("Tied") the stick slope is the whole #ui("K"), so
  the tied surfaces do not slide.
- With #ui("Mortar") a #ui("Hard") contact is truly hard: Nodalis leaves the surface behavior
  out, which is how CalculiX's mortar contact enforces it without penetration.

For a #ui("Hard") contact the input file names E and h in a comment. A #ui("Linear") or
#ui("Tied") behavior and an entered #ui("Stick slope") are written as you entered them.

=== Contact pairs

#menu("Connections", "Contact Pair") creates a contact between a master and a slave
surface.

#screenshot("contact-pair.png", [Create Contact Pair; the grey text below #ui("Method") explains
  the chosen method])

#fields(
  [Surface Interaction], [The contact behaviour (see above).],
  [Method], [How CalculiX enforces the contact, see the table below. All contact pairs of a
    model must use the same method.],
  [Small sliding], [Slave nodes are paired with master faces only at the start of each
    increment and the pairing is kept during the iterations. This is faster and more stable when
    the surfaces slide little compared with the element size. Only for #ui("Node to surface").],
  [Adjust], [Moves slave nodes onto the master surface before the analysis, so that small gaps
    or overlaps from meshing do not start the contact with a jump. With #ui("Adjust distance")
    all slave nodes within that distance are moved; without it only nodes that penetrate the
    master surface. Only the start geometry changes, not the real geometry.],
  [Master / Slave], [The two surfaces, picked with their #ui("...") buttons. Choose the finer
    mesh as slave.],
)

In 3D models the master and slave regions can also be edges of shells and membranes, where a
shell meets a solid face edge on: a click near a free edge of a shell picks the edge, and so do
#ui("Edge angle") and, under #ui("More"), #ui("Edge"). CalculiX takes such an edge as a face of
the shell (S3 and up). For a contact on a shell edge use #ui("Surface to surface") with a
#ui("Tied") surface behavior (with its #ui("Friction"), see above).

==== Which method?

#fields(
  [Node to surface], [Each slave node is kept outside the master surface. Works with nodes or
    faces as slave and in all step types, including dynamics. Master nodes may poke through the
    slave surface and the pressure is noisy, so the finer mesh must be the slave. Best with linear
    elements and for explicit dynamics.],
  [Surface to surface], [Face-to-face penalty contact (default). The contact condition is
    integrated over the slave faces, so the slave must consist of element faces. Smoother
    pressures and better convergence than node to surface, especially with quadratic elements.
    Slower per iteration. A good choice for static contact.],
  [Mortar], [Face-to-face mortar contact. The most accurate contact pressure and little
    dependence on which side is master, which matters when you read contact pressures. Static
    steps only, element-face surfaces required, and the slowest method. Try it if surface to
    surface gives oscillating pressures; it is not meant for dynamics.],
  [Massless], [Massless node-to-face contact for explicit dynamics. Use it only in explicit
    dynamic steps.],
)

#note[The method is a CalculiX feature. See the sections on contact in the CalculiX manual
  (node-to-face, face-to-face penalty, face-to-face mortar and massless contact) for the exact
  formulations and limits.]

Hover over a method in the list to see a short description.

=== Ties <ties>

#menu("Connections", "Tie") or #ui("Create Tie ...") in the context menu of
#ui("Contact Pairs") glues a slave surface to a master surface (`*TIE`). Ties are listed under
#ui("Contact Pairs"), with the contact pairs and node ties; projects that kept them under
#ui("Constraints") open with them moved there.

#fields(
  [Position tolerance], [The largest distance at which slave nodes are tied.],
  [Adjust], [#ui("Move slave nodes onto master (Adjust)") projects the slave nodes onto the
    master surface.],
  [Master / Slave], [The two surfaces, picked with their #ui("...") buttons, also edges of
    shells as for contact pairs. Choose the finer mesh as slave, and an edge of a shell as slave
    of the face it stands on.],
  [Cyclic symmetry], [Ties the two cut faces of a sector of a rotationally repeating part, see
    @cyclic-symmetry.],
)

CalculiX ties the edge faces of membranes to nothing (the solver stops with a singular stiffness
matrix), so Nodalis writes the slave side of a tie on membrane edges as a node surface of the
edge's nodes; a membrane has no rotations that would need more.

=== Node ties

#ui("Create Node Tie ...") in the context menu of #ui("Contact Pairs") makes all nodes of a region
move together. It connects the ends of beams and trusses of different parts. With
#ui("Rotations: rigid") the connection also transmits moments; without it, beams are connected
by a hinge.

=== Many pairs at once

To delete, deactivate or swap many contact pairs or ties at once, select
them in the tree with #key("Shift")+click (a range) and #key("Ctrl")+click (one more or one less)
and use the context menu of one of them (@model-tree).

#screenshot("contact-pairs-selected.png", [Four of five ties selected and deactivated together;
  the context menu acts on all of them])
