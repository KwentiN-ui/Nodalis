#import "../../template.typ": *

== Creep and the Visco step <creep>

Metals at high temperature, plastics and solders keep deforming under a constant load. This
creep is modelled with Norton's law and computed over time in a #ui("Visco (creep)") step
(`*VISCO`).

=== Creep law of a material

#ui("Creep (Norton)") in the material dialog (@materials) adds Norton's creep law
(`*CREEP, LAW=NORTON`): the equivalent creep strain rate is

$ dot(epsilon)_c = A sigma^n t^m $

with the von Mises stress $sigma$ and the time $t$, both in the units of the model (MPa and s
in mm, ton, s). The material also needs #ui("Elasticity"); it can be plastic as well.

#screenshot("creep-material.png", [Material with Norton's creep law])

#fields(
  [A], [Creep constant, positive. Its unit depends on $n$ and $m$; Nodalis converts it with
    the model when the unit system changes.],
  [Stress exponent n], [Positive, often 3 to 8 for metals.],
  [Time exponent m], [0 for steady (secondary) creep; a negative value between -1 and 0 lets
    the rate drop over time (primary creep).],
)

Only Visco steps let the material creep; a Static step treats it as elastic (and plastic).

=== Visco step

Create it with #menu("Steps", "Step") and the type #ui("Visco (creep)"). It has the
settings of a static step (@steps); the #ui("Time period") is the creep time, 1000 s by
default. The loads of the step are held over that time. Typically a Static step applies the
loads first and the Visco step keeps them, so the creep starts from the loaded state.

#screenshot("visco-step.png", [Visco step over 1000 s])

#fields(
  [Creep strain tolerance], [Largest error of the creep strain in an increment (`CETOL`),
    which shortens the automatic increments where the strain rate changes quickly. CalculiX
    requires a positive value; 1e-4 is the default.],
)

The new step requests `PEEQ` in the element field output, which the solver writes as the
equivalent creep strain `PE`. The model check (@model-check) warns about a Visco step whose
materials have no creep law, since it then computes the same as a static step, and reports a
creep law without elasticity or with a constant that is not positive.

#screenshot("visco-result.png", [Equivalent creep strain of the cantilever after 1000 s of
creep])
