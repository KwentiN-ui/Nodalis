#import "../../template.typ": *

== Entering values <numeric-input>

Numeric fields accept a decimal point or a comma, scientific notation (`2.1e5`, `2,1e5`) and the
typographic minus sign. Thousands separators are not allowed. You can also drag a field with the
mouse to change its value.

Every numeric field also takes simple arithmetic: `+`, `-`, `*`, `/` (or `×` and `÷`), signs and
parentheses, e.g. `1/30`, `2*(3.5-1)` or `-(0,5+1)`. The field evaluates the expression when you
leave it and keeps only the result, as if you had typed it directly (@arithmetic-input). Fields
of whole numbers, such as counts, accept an expression only when its result is whole. An
expression that cannot be evaluated, such as `1/0`, is not accepted, like any other invalid input.

Fields of physical quantities show the unit of the model's unit system behind the value
(@unit-systems). You may type a value in another unit and Nodalis converts it: in a model in
mm, `3 cm` becomes 30 mm; other examples are `200 GPa`, `7.85 g/cm³`, `kN·m`, `N/mm²`,
`W/(m²·K)`, `5°`, `rpm`, `psi` and `°F`. The unit must belong to the quantity of the field. In a
unitless model, enter numbers only.

A unit after an expression applies to its result: `100/3 mm` is a third of 100 mm, and `(2+3) cm`
is 50 mm in a model in mm. Units inside an expression, as in `2 mm + 3 cm`, are not supported.

#figure(
  image("../../images/arithmetic-input.png", width: 60%),
  caption: [Typing `1/30 m` into a length field of a model in mm (left); on leaving it, the
    field holds the result (right).],
) <arithmetic-input>

Nodalis never rounds what you typed: values are shown with all significant digits, very small or
very large values in scientific notation.
