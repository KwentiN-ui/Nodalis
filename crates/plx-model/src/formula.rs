//! The formula of a Formula block of the project schematic: an expression in its inputs
//! a, b, c, ... with the operators `+ - * / ^`, numbers with units such as `2mm`,
//! `10*N/mm^2` or `2[mm]` and functions such as `sqrt` and `max`. Numbers and series go together like the arrays of
//! NumPy, element by element, a single number with every value of a series. Units go
//! through the formula: a force over a length is a force per length, a square root halves
//! the powers of the units, and a sum takes numbers of one unit.

use std::fmt;

use crate::ValueKind;
use crate::units::{Dimension, UnitSystem, parse_unit};

/// The most inputs a Formula block has: one for each letter.
pub const MAX_INPUTS: usize = 26;

/// The letter of input `index`: a, b, c, ...
pub fn input_letter(index: usize) -> char {
    (b'a' + index.min(MAX_INPUTS - 1) as u8) as char
}

/// The index of the input of `letter`.
pub fn input_index(letter: &str) -> Option<usize> {
    let mut chars = letter.chars();
    let (Some(c), None) = (chars.next(), chars.next()) else {
        return None;
    };
    c.is_ascii_lowercase().then(|| (c as u8 - b'a') as usize)
}

/// The formula `text` with its inputs named anew: the input of index `i` becomes
/// `rename(i)`, as inputs of a Formula block move up when one before them goes, and an input
/// that went (`None`) becomes `?`, which the formula then reports. Function names, units in
/// brackets and everything else stay as typed.
pub fn rename_inputs(text: &str, rename: &dyn Fn(usize) -> Option<usize>) -> String {
    let chars: Vec<char> = text.chars().collect();
    let mut out = String::with_capacity(text.len());
    let mut i = 0;
    // As the parser reads units after a number: whether a name now is a unit, such as the m
    // of 2m, whether a unit came last, after which / and ^ go on with units, and whether an
    // exponent of a unit comes.
    let mut after_number = false;
    let mut in_unit = false;
    let mut unit_power = false;
    while i < chars.len() {
        let c = chars[i];
        if c.is_whitespace() {
            out.push(c);
            i += 1;
            continue;
        }
        if c == '[' {
            (after_number, in_unit, unit_power) = (false, false, false);
            // A unit such as [mm] keeps its letters.
            let end = (i..chars.len())
                .find(|&j| chars[j] == ']')
                .unwrap_or(chars.len() - 1);
            out.extend(&chars[i..=end]);
            i = end + 1;
            continue;
        }
        if c.is_alphabetic() || c == '_' {
            let start = i;
            while i < chars.len()
                && (chars[i].is_alphabetic() || chars[i].is_ascii_digit() || chars[i] == '_')
            {
                i += 1;
            }
            let name: String = chars[start..i].iter().collect();
            let call = chars[i..].iter().find(|c| !c.is_whitespace()) == Some(&'(');
            let unit = after_number && !call && unit_name(&name).is_some();
            (after_number, in_unit, unit_power) = (unit, unit, false);
            match input_index(&name) {
                Some(index) if !call && !unit => match rename(index) {
                    Some(new) => out.push(input_letter(new)),
                    None => out.push('?'),
                },
                _ => out.push_str(&name),
            }
            continue;
        }
        if c.is_ascii_digit() || c == '.' {
            // A number, with its exponent: 1e5 is no input e.
            let start = i;
            while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.') {
                i += 1;
            }
            if i < chars.len() && (chars[i] == 'e' || chars[i] == 'E') {
                let mut j = i + 1;
                if j < chars.len() && (chars[j] == '+' || chars[j] == '-') {
                    j += 1;
                }
                if j < chars.len() && chars[j].is_ascii_digit() {
                    i = j;
                    while i < chars.len() && chars[i].is_ascii_digit() {
                        i += 1;
                    }
                }
            }
            out.extend(&chars[start..i]);
            (after_number, in_unit) = (true, unit_power);
            unit_power = false;
            continue;
        }
        match c {
            '/' if in_unit => (after_number, in_unit, unit_power) = (true, false, false),
            '^' if in_unit => (after_number, in_unit, unit_power) = (false, false, true),
            '-' | '+' if unit_power => {}
            _ => (after_number, in_unit, unit_power) = (false, false, false),
        }
        out.push(c);
        i += 1;
    }
    out
}

/// A function a formula can call.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Function {
    Sqrt,
    Cbrt,
    Abs,
    Sign,
    Exp,
    Ln,
    Log10,
    Sin,
    Cos,
    Tan,
    Asin,
    Acos,
    Atan,
    Atan2,
    Floor,
    Ceil,
    Round,
    Min,
    Max,
    Mean,
    Sum,
    Std,
    Norm,
    Clamp,
}

/// How many arguments a function takes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Arity {
    Exactly(usize),
    /// One series, or several numbers or series element by element.
    AnyNumber,
}

impl Function {
    pub const ALL: [Self; 24] = [
        Self::Sqrt,
        Self::Cbrt,
        Self::Abs,
        Self::Sign,
        Self::Exp,
        Self::Ln,
        Self::Log10,
        Self::Sin,
        Self::Cos,
        Self::Tan,
        Self::Asin,
        Self::Acos,
        Self::Atan,
        Self::Atan2,
        Self::Floor,
        Self::Ceil,
        Self::Round,
        Self::Min,
        Self::Max,
        Self::Mean,
        Self::Sum,
        Self::Std,
        Self::Norm,
        Self::Clamp,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Self::Sqrt => "sqrt",
            Self::Cbrt => "cbrt",
            Self::Abs => "abs",
            Self::Sign => "sign",
            Self::Exp => "exp",
            Self::Ln => "ln",
            Self::Log10 => "log10",
            Self::Sin => "sin",
            Self::Cos => "cos",
            Self::Tan => "tan",
            Self::Asin => "asin",
            Self::Acos => "acos",
            Self::Atan => "atan",
            Self::Atan2 => "atan2",
            Self::Floor => "floor",
            Self::Ceil => "ceil",
            Self::Round => "round",
            Self::Min => "min",
            Self::Max => "max",
            Self::Mean => "mean",
            Self::Sum => "sum",
            Self::Std => "std",
            Self::Norm => "norm",
            Self::Clamp => "clamp",
        }
    }

    /// How it is called, for the list of functions.
    pub fn signature(self) -> &'static str {
        match self {
            Self::Atan2 => "atan2(y, x)",
            Self::Clamp => "clamp(x, low, high)",
            Self::Min => "min(x, ...)",
            Self::Max => "max(x, ...)",
            Self::Mean => "mean(x, ...)",
            Self::Sum => "sum(x, ...)",
            Self::Std => "std(x, ...)",
            Self::Norm => "norm(x, ...)",
            Self::Sqrt => "sqrt(x)",
            Self::Cbrt => "cbrt(x)",
            Self::Abs => "abs(x)",
            Self::Sign => "sign(x)",
            Self::Exp => "exp(x)",
            Self::Ln => "ln(x)",
            Self::Log10 => "log10(x)",
            Self::Sin => "sin(x)",
            Self::Cos => "cos(x)",
            Self::Tan => "tan(x)",
            Self::Asin => "asin(x)",
            Self::Acos => "acos(x)",
            Self::Atan => "atan(x)",
            Self::Floor => "floor(x)",
            Self::Ceil => "ceil(x)",
            Self::Round => "round(x)",
        }
    }

    /// What it computes and what it does with units, for the list of functions.
    pub fn description(self) -> &'static str {
        match self {
            Self::Sqrt => "Square root; halves the powers of the unit (m² gives m)",
            Self::Cbrt => "Cube root; a third of the powers of the unit",
            Self::Abs => "Magnitude, in the unit of x",
            Self::Sign => "-1, 0 or 1 by the sign of x, without unit",
            Self::Exp => "e to the power x; x without unit",
            Self::Ln => "Natural logarithm; x without unit",
            Self::Log10 => "Logarithm to base 10; x without unit",
            Self::Sin => "Sine of an angle, or of a number without unit in radians",
            Self::Cos => "Cosine of an angle, or of a number without unit in radians",
            Self::Tan => "Tangent of an angle, or of a number without unit in radians",
            Self::Asin => "Arc sine, an angle; x without unit",
            Self::Acos => "Arc cosine, an angle; x without unit",
            Self::Atan => "Arc tangent, an angle; x without unit",
            Self::Atan2 => "Angle of the point (x, y); y and x of one unit",
            Self::Floor => "Next whole number below; x without unit",
            Self::Ceil => "Next whole number above; x without unit",
            Self::Round => "Nearest whole number; x without unit",
            Self::Min => "Smallest value of a series, or of several element by element",
            Self::Max => "Largest value of a series, or of several element by element",
            Self::Mean => "Mean of a series, or of several element by element",
            Self::Sum => "Sum of a series, or of several element by element",
            Self::Std => "Standard deviation of a series, or of several",
            Self::Norm => "Root of the sum of squares, e.g. the magnitude norm(x, y, z)",
            Self::Clamp => "x, but at least low and at most high; all of one unit",
        }
    }

    fn of_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|f| f.name() == name)
    }

    fn arity(self) -> Arity {
        match self {
            Self::Atan2 => Arity::Exactly(2),
            Self::Clamp => Arity::Exactly(3),
            Self::Min | Self::Max | Self::Mean | Self::Sum | Self::Std | Self::Norm => {
                Arity::AnyNumber
            }
            _ => Arity::Exactly(1),
        }
    }

    /// Whether, called with one argument, it makes one number of a series.
    fn reduces(self) -> bool {
        self.arity() == Arity::AnyNumber
    }

    /// The result for the values of its arguments at one element.
    fn apply(self, x: &[f64]) -> f64 {
        let first = x.first().copied().unwrap_or(f64::NAN);
        match self {
            Self::Sqrt => first.sqrt(),
            Self::Cbrt => first.cbrt(),
            Self::Abs => first.abs(),
            Self::Sign if first == 0.0 => 0.0,
            Self::Sign if first.is_nan() => f64::NAN,
            Self::Sign => first.signum(),
            Self::Exp => first.exp(),
            Self::Ln => first.ln(),
            Self::Log10 => first.log10(),
            Self::Sin => first.sin(),
            Self::Cos => first.cos(),
            Self::Tan => first.tan(),
            Self::Asin => first.asin(),
            Self::Acos => first.acos(),
            Self::Atan => first.atan(),
            Self::Atan2 => first.atan2(x.get(1).copied().unwrap_or(f64::NAN)),
            Self::Floor => first.floor(),
            Self::Ceil => first.ceil(),
            Self::Round => first.round(),
            Self::Clamp => {
                let (low, high) = (x[1], x[2]);
                // Like Blender's Math node: the upper limit wins when the limits cross.
                first.max(low).min(high)
            }
            Self::Min | Self::Max | Self::Mean | Self::Sum | Self::Std | Self::Norm => {
                self.reduce(x)
            }
        }
    }

    /// The one number of a reducing function of `values`, NaN left out; NaN without
    /// values.
    fn reduce(self, values: &[f64]) -> f64 {
        let finite: Vec<f64> = values.iter().copied().filter(|v| v.is_finite()).collect();
        if finite.is_empty() {
            return f64::NAN;
        }
        let n = finite.len() as f64;
        let sum: f64 = finite.iter().sum();
        match self {
            Self::Min => finite.iter().copied().fold(f64::INFINITY, f64::min),
            Self::Max => finite.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            Self::Mean => sum / n,
            Self::Sum => sum,
            Self::Std => {
                let mean = sum / n;
                (finite.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n).sqrt()
            }
            Self::Norm => finite.iter().map(|v| v * v).sum::<f64>().sqrt(),
            _ => f64::NAN,
        }
    }

    /// The unit of its result for the units of its arguments, or why they do not fit.
    fn dimension(self, args: &[Dimension], call: &Expr) -> Result<Dimension, String> {
        let none = Dimension::default();
        let angle = Dimension::new(0, 0, 0, 0, 1);
        let first = args.first().copied().unwrap_or_default();
        let without_unit = || {
            if first.is_none() {
                Ok(none)
            } else {
                Err(format!(
                    "{call} takes a number without unit, not {}",
                    describe(first)
                ))
            }
        };
        let all_alike = || match args.iter().find(|d| **d != first) {
            Some(other) => Err(format!(
                "{call} takes numbers of one unit, not {} and {}",
                describe(first),
                describe(*other)
            )),
            None => Ok(first),
        };
        match self {
            Self::Sqrt => root(first, 2, call),
            Self::Cbrt => root(first, 3, call),
            Self::Abs => Ok(first),
            Self::Sign => Ok(none),
            Self::Exp | Self::Ln | Self::Log10 | Self::Floor | Self::Ceil | Self::Round => {
                without_unit()
            }
            Self::Sin | Self::Cos | Self::Tan => {
                if first.is_none() || first == angle {
                    Ok(none)
                } else {
                    Err(format!(
                        "{call} takes an angle or a number without unit, not {}",
                        describe(first)
                    ))
                }
            }
            Self::Asin | Self::Acos | Self::Atan => without_unit().map(|_| angle),
            Self::Atan2 => all_alike().map(|_| angle),
            Self::Min | Self::Max | Self::Mean | Self::Sum | Self::Std | Self::Norm => all_alike(),
            Self::Clamp => all_alike(),
        }
    }
}

/// The `n`-th root of a unit: every power divided by `n`, which must go evenly.
fn root(dimension: Dimension, n: i8, call: &Expr) -> Result<Dimension, String> {
    let powers = [
        dimension.length,
        dimension.mass,
        dimension.time,
        dimension.temperature,
        dimension.angle,
    ];
    if powers.iter().any(|p| p % n != 0) {
        return Err(format!(
            "{call} has no unit: {} has no {} root",
            dimension.si(),
            if n == 2 { "square" } else { "cube" }
        ));
    }
    let [l, m, t, k, a] = powers.map(|p| p / n);
    Ok(Dimension::new(l, m, t, k, a))
}

/// What numbers of a unit are, for messages: "a force", "a number without unit" or "a
/// number in kg m³".
fn describe(dimension: Dimension) -> String {
    match ValueKind::of_dimension(dimension) {
        ValueKind::Factor => "a number without unit".to_owned(),
        ValueKind::Derived(_) => format!("a number in {}", dimension.si()),
        kind => format!("a {}", kind.describe()),
    }
}

/// A parsed formula.
#[derive(Clone, Debug, PartialEq)]
pub enum Expr {
    /// A number in SI units with its unit, such as `2[mm]`, 0.002 m.
    Number(f64, Dimension),
    /// A unit as written, such as `mm`, `N` or `[N/mm^2]`, which is its size in SI units:
    /// 0.001 m. `10mm` is the product of 10 and the unit, as is `10*mm`.
    Unit(String, f64, Dimension),
    /// The input of a letter, by index.
    Input(usize),
    Negate(Box<Expr>),
    /// `+ - * / ^` of two expressions; ` ` is the product of a number and the unit written
    /// right after it, as in `10mm`, which goes together before any other operator.
    Binary(char, Box<Expr>, Box<Expr>),
    Call(Function, Vec<Expr>),
}

impl fmt::Display for Expr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // A sum goes in parentheses, but not a number with its unit in a sum.
        let operand = |e: &Expr, in_sum: bool, f: &mut fmt::Formatter<'_>| match e {
            _ if e.is_unit_power() => write!(f, "{e}"),
            _ if in_sum && e.is_quantity() => write!(f, "{e}"),
            Expr::Binary(..) | Expr::Negate(_) => write!(f, "({e})"),
            _ => write!(f, "{e}"),
        };
        match self {
            Self::Number(value, dimension) if dimension.is_none() => write!(f, "{value}"),
            Self::Number(value, dimension) => write!(f, "{value}[{}]", dimension.si()),
            Self::Unit(name, ..) if name.chars().all(char::is_alphabetic) => write!(f, "{name}"),
            Self::Unit(name, ..) => write!(f, "[{name}]"),
            Self::Input(index) => write!(f, "{}", input_letter(*index)),
            Self::Binary(' ', a, b) => {
                operand(a, a.is_quantity(), f)?;
                write!(f, " ")?;
                operand(b, false, f)
            }
            Self::Binary('^', a, b) if self.is_unit_power() => write!(f, "{a}^{b}"),
            Self::Negate(e) => {
                write!(f, "-")?;
                operand(e, false, f)
            }
            Self::Binary(op, a, b) => {
                let in_sum = matches!(op, '+' | '-');
                // 10 N / mm^3 is one quantity and needs no parentheses.
                operand(a, in_sum || self.is_quantity(), f)?;
                write!(f, " {op} ")?;
                operand(b, in_sum, f)
            }
            Self::Call(function, args) => {
                write!(f, "{}(", function.name())?;
                for (i, arg) in args.iter().enumerate() {
                    if i > 0 {
                        write!(f, ", ")?;
                    }
                    write!(f, "{arg}")?;
                }
                write!(f, ")")
            }
        }
    }
}

/// Values of a formula: a single number, or a series of numbers.
#[derive(Clone, Debug, PartialEq)]
struct Values {
    values: Vec<f64>,
    series: bool,
}

impl Values {
    fn number(value: f64) -> Self {
        Self {
            values: vec![value],
            series: false,
        }
    }

    fn at(&self, i: usize) -> f64 {
        if self.series {
            self.values.get(i).copied().unwrap_or(f64::NAN)
        } else {
            self.values[0]
        }
    }
}

/// The length of the series of `values` go together in, element by element, or why not:
/// series must be of one length. `None` when all are single numbers.
fn common_length(values: &[&Values], what: &dyn Fn() -> String) -> Result<Option<usize>, String> {
    let mut length = None;
    for v in values.iter().filter(|v| v.series) {
        match length {
            None => length = Some(v.values.len()),
            Some(n) if n == v.values.len() => {}
            Some(n) => {
                return Err(format!(
                    "{}: the series have {n} and {} values, and go element by element",
                    what(),
                    v.values.len()
                ));
            }
        }
    }
    Ok(length)
}

/// The values of `f` of `args` element by element.
fn elementwise(
    args: &[&Values],
    what: &dyn Fn() -> String,
    f: impl Fn(&[f64]) -> f64,
) -> Result<Values, String> {
    Ok(match common_length(args, what)? {
        None => Values::number(f(&args.iter().map(|a| a.values[0]).collect::<Vec<_>>())),
        Some(n) => Values {
            values: (0..n)
                .map(|i| f(&args.iter().map(|a| a.at(i)).collect::<Vec<_>>()))
                .collect(),
            series: true,
        },
    })
}

impl Expr {
    /// Reads a formula; an error says what is wrong with it.
    pub fn parse(text: &str) -> Result<Self, String> {
        let tokens = tokenize(text)?;
        if tokens.is_empty() {
            return Err("the formula is empty".into());
        }
        let mut parser = Parser { tokens, at: 0 };
        let expr = parser.sum()?;
        match parser.tokens.get(parser.at) {
            None => Ok(expr),
            Some(Token::Close) => Err("there is a ) without (".into()),
            Some(token) => Err(format!("{token} is not expected after {expr}")),
        }
    }

    /// Whether it is a unit or a power of one, such as `mm^2`.
    fn is_unit_power(&self) -> bool {
        match self {
            Self::Unit(..) => true,
            Self::Binary('^', a, b) => {
                matches!(**a, Self::Unit(..)) && matches!(**b, Self::Number(..) | Self::Negate(_))
            }
            _ => false,
        }
    }

    /// Whether it is a number with units written after it, such as `10 N / mm^3`.
    fn is_quantity(&self) -> bool {
        match self {
            Self::Binary(' ', ..) => true,
            Self::Binary('*' | '/', a, b) => a.is_quantity() && b.is_unit_power(),
            _ => false,
        }
    }

    /// The indices of the inputs it uses, in order and each once.
    pub fn inputs(&self) -> Vec<usize> {
        let mut used = Vec::new();
        self.visit_inputs(&mut |i| {
            if !used.contains(&i) {
                used.push(i);
            }
        });
        used.sort_unstable();
        used
    }

    fn visit_inputs(&self, f: &mut dyn FnMut(usize)) {
        match self {
            Self::Number(..) | Self::Unit(..) => {}
            Self::Input(i) => f(*i),
            Self::Negate(e) => e.visit_inputs(f),
            Self::Binary(_, a, b) => {
                a.visit_inputs(f);
                b.visit_inputs(f);
            }
            Self::Call(_, args) => args.iter().for_each(|a| a.visit_inputs(f)),
        }
    }

    /// The value of a formula without inputs, such as an exponent; `None` with inputs.
    fn constant(&self) -> Option<f64> {
        if !self.inputs().is_empty() {
            return None;
        }
        let values = self.evaluate(&[]).ok()?;
        (!values.is_empty()).then(|| values[0])
    }

    /// The unit of its result for the units of its inputs, by index, or why the units do
    /// not go together: a force plus a length.
    pub fn dimension(&self, inputs: &[Dimension]) -> Result<Dimension, String> {
        match self {
            Self::Number(_, dimension) | Self::Unit(_, _, dimension) => Ok(*dimension),
            Self::Input(i) => inputs
                .get(*i)
                .copied()
                .ok_or_else(|| format!("there is no input {}", input_letter(*i))),
            Self::Negate(e) => e.dimension(inputs),
            Self::Binary(op, a, b) => {
                let da = a.dimension(inputs)?;
                if *op == '^' {
                    return power(da, b, inputs, self);
                }
                let db = b.dimension(inputs)?;
                match op {
                    '*' | ' ' => checked(da.times(db), self),
                    '/' => checked(da.over(db), self),
                    _ if da == db => Ok(da),
                    _ => {
                        let mut message = format!(
                            "{self} {} {} and {}",
                            if *op == '+' { "adds" } else { "subtracts" },
                            describe(da),
                            describe(db)
                        );
                        // A number typed without its unit: say how to give it one.
                        let bare = [(a, db), (b, da)].into_iter().find(|(e, other)| {
                            matches!(***e, Self::Number(_, d) if d.is_none()) && !other.is_none()
                        });
                        if let Some((number, unit)) = bare {
                            message.push_str(&format!(
                                ": write the unit after the number, such as {}",
                                with_unit(number, unit)
                            ));
                        }
                        Err(message)
                    }
                }
            }
            Self::Call(function, args) => {
                let dims = (args.iter())
                    .map(|a| a.dimension(inputs))
                    .collect::<Result<Vec<_>, _>>()?;
                function.dimension(&dims, self)
            }
        }
    }

    /// Whether its result is a series, with the inputs of `series` series and the others
    /// single numbers: a function of one series that makes one number of it is not.
    pub fn gives_series(&self, series: &[bool]) -> bool {
        match self {
            Self::Number(..) | Self::Unit(..) => false,
            Self::Input(i) => series.get(*i).copied().unwrap_or(false),
            Self::Negate(e) => e.gives_series(series),
            Self::Binary(_, a, b) => a.gives_series(series) || b.gives_series(series),
            Self::Call(function, args) if function.reduces() && args.len() == 1 => false,
            Self::Call(_, args) => args.iter().any(|a| a.gives_series(series)),
        }
    }

    /// Its values for the values of its inputs, by index: one value a single number,
    /// several a series, which goes with others element by element. A value that has no
    /// result, such as one divided by zero, is NaN.
    pub fn evaluate(&self, inputs: &[(&[f64], bool)]) -> Result<Vec<f64>, String> {
        self.values(inputs).map(|v| v.values)
    }

    fn values(&self, inputs: &[(&[f64], bool)]) -> Result<Values, String> {
        let what = || self.to_string();
        match self {
            Self::Number(value, _) | Self::Unit(_, value, _) => Ok(Values::number(*value)),
            Self::Input(i) => {
                let (values, series) = inputs
                    .get(*i)
                    .copied()
                    .ok_or_else(|| format!("there is no input {}", input_letter(*i)))?;
                if values.is_empty() {
                    return Err(format!("{} has no values", input_letter(*i)));
                }
                Ok(Values {
                    values: values.to_vec(),
                    series,
                })
            }
            Self::Negate(e) => {
                let mut v = e.values(inputs)?;
                v.values.iter_mut().for_each(|x| *x = -*x);
                Ok(v)
            }
            Self::Binary(op, a, b) => {
                let (a, b) = (a.values(inputs)?, b.values(inputs)?);
                let op = *op;
                elementwise(&[&a, &b], &what, |x| match op {
                    '+' => x[0] + x[1],
                    '-' => x[0] - x[1],
                    '*' | ' ' => x[0] * x[1],
                    '/' if x[1] == 0.0 => f64::NAN,
                    '/' => x[0] / x[1],
                    _ => x[0].powf(x[1]),
                })
            }
            Self::Call(function, args) => {
                let args = (args.iter())
                    .map(|a| a.values(inputs))
                    .collect::<Result<Vec<_>, _>>()?;
                if function.reduces() && args.len() == 1 {
                    return Ok(Values::number(function.reduce(&args[0].values)));
                }
                let refs: Vec<&Values> = args.iter().collect();
                elementwise(&refs, &what, |x| function.apply(x))
            }
        }
    }
}

/// A unit that is still one: its powers small enough to count with.
fn checked(dimension: Dimension, expr: &Expr) -> Result<Dimension, String> {
    let powers = [
        dimension.length,
        dimension.mass,
        dimension.time,
        dimension.temperature,
        dimension.angle,
    ];
    if powers.iter().any(|p| p.abs() > 30) {
        return Err(format!(
            "{expr} has powers of units too large to count with"
        ));
    }
    Ok(dimension)
}

/// The unit of `base` to the power `exponent`: any power of a number without unit; a
/// number with a unit only to a power fixed in the formula, which gives whole powers of its
/// units.
fn power(
    base: Dimension,
    exponent: &Expr,
    inputs: &[Dimension],
    whole: &Expr,
) -> Result<Dimension, String> {
    let de = exponent.dimension(inputs)?;
    if !de.is_none() {
        return Err(format!(
            "{whole}: the exponent must be without unit, not {}",
            describe(de)
        ));
    }
    if base.is_none() {
        return Ok(base);
    }
    let Some(p) = exponent.constant() else {
        return Err(format!(
            "{whole}: a number with a unit goes only to a power written in the formula, such \
             as ^2"
        ));
    };
    let powers = [
        base.length,
        base.mass,
        base.time,
        base.temperature,
        base.angle,
    ];
    let mut raised = [0i8; 5];
    for (r, &b) in raised.iter_mut().zip(&powers) {
        let x = f64::from(b) * p;
        if (x - x.round()).abs() > 1e-9 || x.abs() > 30.0 {
            return Err(format!(
                "{whole} has no unit: {} to the power {p} is not a whole power",
                base.si()
            ));
        }
        *r = x.round() as i8;
    }
    let [l, m, t, k, a] = raised;
    Ok(Dimension::new(l, m, t, k, a))
}

#[derive(Clone, Debug, PartialEq)]
enum Token {
    Number(f64),
    /// A unit in brackets, such as `[mm]`.
    Unit(String),
    Name(String),
    Operator(char),
    Open,
    Close,
    Comma,
}

impl fmt::Display for Token {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Number(v) => write!(f, "{v}"),
            Self::Unit(u) => write!(f, "[{u}]"),
            Self::Name(n) => write!(f, "{n}"),
            Self::Operator(c) => write!(f, "{c}"),
            Self::Open => write!(f, "("),
            Self::Close => write!(f, ")"),
            Self::Comma => write!(f, ","),
        }
    }
}

fn tokenize(text: &str) -> Result<Vec<Token>, String> {
    let chars: Vec<char> = text.chars().collect();
    let mut tokens = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        match c {
            c if c.is_whitespace() => i += 1,
            '+' | '/' | '^' => {
                tokens.push(Token::Operator(c));
                i += 1;
            }
            '*' if chars.get(i + 1) == Some(&'*') => {
                tokens.push(Token::Operator('^'));
                i += 2;
            }
            '*' | '×' | '·' => {
                tokens.push(Token::Operator('*'));
                i += 1;
            }
            '÷' => {
                tokens.push(Token::Operator('/'));
                i += 1;
            }
            '-' | '\u{2212}' => {
                tokens.push(Token::Operator('-'));
                i += 1;
            }
            '²' | '³' => {
                tokens.push(Token::Operator('^'));
                tokens.push(Token::Number(if c == '²' { 2.0 } else { 3.0 }));
                i += 1;
            }
            '(' => {
                tokens.push(Token::Open);
                i += 1;
            }
            ')' => {
                tokens.push(Token::Close);
                i += 1;
            }
            ',' | ';' => {
                tokens.push(Token::Comma);
                i += 1;
            }
            '[' => {
                let end = (i + 1..chars.len())
                    .find(|&j| chars[j] == ']')
                    .ok_or("a [ of a unit has no ]")?;
                let unit: String = chars[i + 1..end].iter().collect();
                tokens.push(Token::Unit(unit.trim().to_owned()));
                i = end + 1;
            }
            c if c.is_ascii_digit() || c == '.' => {
                let start = i;
                while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.') {
                    i += 1;
                }
                // An exponent only when digits follow, possibly after a sign.
                if i < chars.len() && (chars[i] == 'e' || chars[i] == 'E') {
                    let mut j = i + 1;
                    if j < chars.len() && (chars[j] == '+' || chars[j] == '-') {
                        j += 1;
                    }
                    if j < chars.len() && chars[j].is_ascii_digit() {
                        i = j;
                        while i < chars.len() && chars[i].is_ascii_digit() {
                            i += 1;
                        }
                    }
                }
                let text: String = chars[start..i].iter().collect();
                let value = text
                    .parse::<f64>()
                    .map_err(|_| format!("{text} is not a number"))?;
                tokens.push(Token::Number(value));
            }
            c if c.is_alphabetic() || c == '_' => {
                let start = i;
                while i < chars.len()
                    && (chars[i].is_alphabetic() || chars[i].is_ascii_digit() || chars[i] == '_')
                {
                    i += 1;
                }
                tokens.push(Token::Name(chars[start..i].iter().collect()));
            }
            '?' => {
                return Err(
                    "? stands for an input whose link was removed: link a new input \
                            or change the formula"
                        .into(),
                );
            }
            c => return Err(format!("{c} is not part of a formula")),
        }
    }
    Ok(tokens)
}

/// A recursive descent over the tokens of a formula.
struct Parser {
    tokens: Vec<Token>,
    at: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.at)
    }

    fn operator(&mut self, of: &str) -> Option<char> {
        match self.peek() {
            Some(&Token::Operator(c)) if of.contains(c) => {
                self.at += 1;
                Some(c)
            }
            _ => None,
        }
    }

    fn sum(&mut self) -> Result<Expr, String> {
        let mut expr = self.product()?;
        while let Some(op) = self.operator("+-") {
            let term = self.product()?;
            expr = Expr::Binary(op, Box::new(expr), Box::new(term));
        }
        Ok(expr)
    }

    fn product(&mut self) -> Result<Expr, String> {
        let mut expr = self.unary()?;
        while let Some(op) = self.operator("*/") {
            let factor = self.unary()?;
            expr = Expr::Binary(op, Box::new(expr), Box::new(factor));
        }
        Ok(expr)
    }

    /// A sign binds less than a power, as in Python: -a^2 is -(a^2).
    fn unary(&mut self) -> Result<Expr, String> {
        match self.operator("+-") {
            Some('-') => Ok(Expr::Negate(Box::new(self.unary()?))),
            Some(_) => self.unary(),
            None => self.power(),
        }
    }

    /// Powers go from the right: a^b^c is a^(b^c).
    fn power(&mut self) -> Result<Expr, String> {
        let base = self.atom()?;
        if self.operator("^").is_some() {
            let exponent = self.unary()?;
            return Ok(Expr::Binary('^', Box::new(base), Box::new(exponent)));
        }
        Ok(base)
    }

    fn atom(&mut self) -> Result<Expr, String> {
        let Some(token) = self.peek().cloned() else {
            return Err("the formula ends too early".into());
        };
        self.at += 1;
        match token {
            Token::Number(value) => {
                let number = match self.peek() {
                    // A unit in brackets goes with the number: 10[mm]^2 is (10 mm)^2.
                    Some(Token::Unit(unit)) => {
                        let unit = unit.clone();
                        self.at += 1;
                        let (factor, dimension) = read_unit(&unit)?;
                        return Ok(Expr::Binary(
                            ' ',
                            Box::new(Expr::Number(value, Dimension::default())),
                            Box::new(Expr::Unit(unit, factor, dimension)),
                        ));
                    }
                    _ => Expr::Number(value, Dimension::default()),
                };
                // Units written right after the number, each with its power, go with it:
                // 10mm^2 is ten square millimetres, 5 kN m five kilonewton metres and
                // a / 10N/mm^2 divides by a pressure.
                let mut expr = number;
                while let Some(unit) = self.unit_after_number() {
                    expr = Expr::Binary(' ', Box::new(expr), Box::new(unit?));
                }
                if matches!(expr, Expr::Binary(' ', ..)) {
                    loop {
                        if let Some(unit) = self.unit_after_number() {
                            expr = Expr::Binary('*', Box::new(expr), Box::new(unit?));
                            continue;
                        }
                        let slash = self.peek() == Some(&Token::Operator('/'));
                        let unit_follows = matches!(self.tokens.get(self.at + 1),
                            Some(Token::Name(n)) if unit_name(n).is_some()
                                && self.tokens.get(self.at + 2) != Some(&Token::Open));
                        if !(slash && unit_follows) {
                            break;
                        }
                        self.at += 1;
                        let unit = self.unit_after_number().expect("a unit follows")?;
                        expr = Expr::Binary('/', Box::new(expr), Box::new(unit));
                    }
                }
                if let Some(Token::Name(name)) = self.peek() {
                    return Err(if input_index(name).is_some() {
                        format!(
                            "{value}{name}: write {value}*{name} for a product with input \
                             {name}"
                        )
                    } else {
                        format!(
                            "{value}{name}: {name} is not a unit; units are written like \
                             {value}mm, {value}N/mm^2 or {value}[N/mm^2]"
                        )
                    });
                }
                Ok(expr)
            }
            Token::Unit(unit) => {
                let (factor, dimension) = read_unit(&unit)?;
                Ok(Expr::Unit(unit, factor, dimension))
            }
            Token::Open => {
                let expr = self.sum()?;
                if self.peek() != Some(&Token::Close) {
                    return Err("a ( has no )".into());
                }
                self.at += 1;
                Ok(expr)
            }
            Token::Name(name) => self.name(&name),
            Token::Close => Err("a ) comes where a value is expected".into()),
            Token::Comma => Err("a , comes where a value is expected".into()),
            Token::Operator(c) => Err(format!("{c} comes where a value is expected")),
        }
    }

    /// The unit written right after a number, with its power, if one comes: a name that is a
    /// unit and no function call.
    fn unit_after_number(&mut self) -> Option<Result<Expr, String>> {
        let Some(Token::Name(name)) = self.peek() else {
            return None;
        };
        if self.tokens.get(self.at + 1) == Some(&Token::Open) {
            return None;
        }
        let (factor, dimension) = unit_name(name)?;
        let unit = Expr::Unit(name.clone(), factor, dimension);
        self.at += 1;
        if self.operator("^").is_none() {
            return Some(Ok(unit));
        }
        Some(
            self.unary()
                .map(|exponent| Expr::Binary('^', Box::new(unit), Box::new(exponent))),
        )
    }

    fn name(&mut self, name: &str) -> Result<Expr, String> {
        if self.peek() == Some(&Token::Open) {
            let function = Function::of_name(name).ok_or_else(|| {
                format!("there is no function {name}; the list below the formula has them")
            })?;
            self.at += 1;
            let mut args = Vec::new();
            if self.peek() != Some(&Token::Close) {
                loop {
                    args.push(self.sum()?);
                    match self.peek() {
                        Some(Token::Comma) => self.at += 1,
                        Some(Token::Close) => break,
                        _ => return Err(format!("{name}( has no )")),
                    }
                }
            }
            self.at += 1;
            let ok = match function.arity() {
                Arity::Exactly(n) => args.len() == n,
                Arity::AnyNumber => !args.is_empty(),
            };
            if !ok {
                return Err(format!(
                    "{name} takes {}, not {}",
                    match function.arity() {
                        Arity::Exactly(1) => "one value".to_owned(),
                        Arity::Exactly(n) => format!("{n} values"),
                        Arity::AnyNumber => "one series or several values".to_owned(),
                    },
                    args.len()
                ));
            }
            return Ok(Expr::Call(function, args));
        }
        if name == "pi" {
            return Ok(Expr::Number(std::f64::consts::PI, Dimension::default()));
        }
        if let Some(index) = input_index(name) {
            return Ok(Expr::Input(index));
        }
        if let Some((factor, dimension)) = unit_name(name) {
            return Ok(Expr::Unit(name.to_owned(), factor, dimension));
        }
        if Function::of_name(name).is_some() {
            return Err(format!("{name} is a function: write {name}(...)"));
        }
        Err(format!(
            "{name} is neither an input nor a unit: the inputs are single letters a, b, c, \
             ...; units are written like 10mm, 10*N/mm^2 or 10[N/mm^2]"
        ))
    }
}

/// The size in SI units and the dimension of a unit written as a name, such as `mm`, `kN`
/// or `MPa`; `None` when the name is no unit.
fn unit_name(name: &str) -> Option<(f64, Dimension)> {
    let parsed = parse_unit(name).ok()?;
    parsed
        .zero
        .is_none()
        .then_some((parsed.factor, parsed.dimension))
}

/// `number` written with a common unit of `dimension`, for messages: 10mm, 10N/mm²,
/// 10[N·mm].
fn with_unit(number: &Expr, dimension: Dimension) -> String {
    let unit = match dimension.quantity() {
        Some(quantity) => UnitSystem::MmTonSC.unit(quantity).to_owned(),
        None => dimension.si(),
    };
    let bare = !unit.is_empty()
        && unit.chars().next().is_some_and(char::is_alphabetic)
        && unit.chars().all(|c| c.is_alphabetic() || "/²³".contains(c))
        && parse_unit(&unit).is_ok();
    if bare {
        format!("{number}{unit}")
    } else {
        format!("{number}[{unit}]")
    }
}

/// The size in SI units and the dimension of a unit in brackets.
fn read_unit(unit: &str) -> Result<(f64, Dimension), String> {
    let parsed = parse_unit(unit).map_err(|_| format!("[{unit}] is not a unit"))?;
    Ok((parsed.factor, parsed.dimension))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Quantity;

    fn dim(q: Quantity) -> Dimension {
        q.dimension()
    }

    fn eval(text: &str, inputs: &[(&[f64], bool)]) -> Vec<f64> {
        Expr::parse(text).unwrap().evaluate(inputs).unwrap()
    }

    #[test]
    fn formulas_compute_with_numbers_and_follow_precedence() {
        let a: &[f64] = &[2.0];
        let b: &[f64] = &[3.0];
        let inputs = [(a, false), (b, false)];
        assert_eq!(eval("a + b * 2", &inputs), vec![8.0]);
        assert_eq!(eval("(a + b) * 2", &inputs), vec![10.0]);
        assert_eq!(eval("-a^2", &inputs), vec![-4.0]);
        assert_eq!(eval("a^b^0", &inputs), vec![2.0]);
        assert_eq!(eval("a**2 + b²", &inputs), vec![13.0]);
        assert_eq!(eval("2 * pi", &inputs), vec![std::f64::consts::TAU]);
        assert_eq!(eval("1.5e3 / 3", &inputs), vec![500.0]);
        assert_eq!(eval("sqrt(a*8)", &inputs), vec![4.0]);
        assert_eq!(eval("max(a, b, 1)", &inputs), vec![3.0]);
        assert_eq!(eval("min(a, b)", &inputs), vec![2.0]);
        assert_eq!(eval("mean(a, b)", &inputs), vec![2.5]);
        assert_eq!(eval("clamp(5, a, b)", &inputs), vec![3.0]);
        assert_eq!(eval("norm(3, 4)", &inputs), vec![5.0]);
        assert_eq!(eval("5[mm]", &inputs), vec![0.005]);
        assert!(eval("a / 0", &inputs)[0].is_nan());
    }

    #[test]
    fn series_go_element_by_element_and_reductions_make_one_number() {
        let v: &[f64] = &[1.0, -4.0, 3.0];
        let w: &[f64] = &[10.0, 20.0, 30.0];
        let two: &[f64] = &[2.0];
        let inputs = [(v, true), (w, true), (two, false)];
        assert_eq!(eval("a * c", &inputs), vec![2.0, -8.0, 6.0]);
        assert_eq!(eval("a + b", &inputs), vec![11.0, 16.0, 33.0]);
        assert_eq!(eval("max(a)", &inputs), vec![3.0]);
        assert_eq!(eval("min(a)", &inputs), vec![-4.0]);
        assert_eq!(eval("sum(a)", &inputs), vec![0.0]);
        assert_eq!(eval("max(a, 0)", &inputs), vec![1.0, 0.0, 3.0]);
        assert_eq!(eval("abs(a)", &inputs), vec![1.0, 4.0, 3.0]);
        assert_eq!(eval("max(abs(a)) / c", &inputs), vec![2.0]);
        let e = Expr::parse("max(a) + b").unwrap();
        assert!(e.gives_series(&[true, true, false]));
        assert!(
            !Expr::parse("mean(a) * c")
                .unwrap()
                .gives_series(&[true, true, false])
        );
        assert!(
            Expr::parse("max(a, c)")
                .unwrap()
                .gives_series(&[true, true, false])
        );
        let short: &[f64] = &[1.0, 2.0];
        let err = Expr::parse("a + b")
            .unwrap()
            .evaluate(&[(v, true), (short, true)])
            .unwrap_err();
        assert!(err.contains("3 and 2 values"), "{err}");
        // A series of one value is still a series.
        let one: &[f64] = &[5.0];
        assert!(Expr::parse("a").unwrap().gives_series(&[true]));
        assert_eq!(eval("a * 2", &[(one, true)]), vec![10.0]);
        // NaN is left out of a reduction.
        let gap: &[f64] = &[1.0, f64::NAN, 3.0];
        assert_eq!(eval("mean(a)", &[(gap, true)]), vec![2.0]);
    }

    #[test]
    fn units_go_through_the_formula() {
        use Quantity::*;
        let d = |text: &str, inputs: &[Dimension]| Expr::parse(text).unwrap().dimension(inputs);
        let force_length = [dim(Force), dim(Length)];
        assert_eq!(d("a / b", &force_length), Ok(dim(ForcePerLength)));
        assert_eq!(d("a * b", &force_length), Ok(dim(Moment)));
        assert_eq!(d("2 * a", &force_length), Ok(dim(Force)));
        assert_eq!(d("sqrt(b^2 + b^2)", &force_length), Ok(dim(Length)));
        assert_eq!(d("norm(b, b, b)", &force_length), Ok(dim(Length)));
        assert_eq!(d("b + 5[mm]", &force_length), Ok(dim(Length)));
        assert_eq!(d("b^-1", &force_length), Ok(dim(Length).powi(-1)));
        assert_eq!(d("(b^2)^0.5", &force_length), Ok(dim(Length)));
        assert_eq!(d("cbrt(b^3)", &force_length), Ok(dim(Length)));
        assert_eq!(d("a / b / b", &force_length), Ok(dim(Pressure)));
        assert_eq!(d("sin(pi/2)", &force_length), Ok(Dimension::default()));
        assert_eq!(d("sin(30[deg])", &force_length), Ok(Dimension::default()));
        assert_eq!(d("atan2(b, b)", &force_length), Ok(dim(Angle)));
        let err = d("a + b", &force_length).unwrap_err();
        assert_eq!(err, "a + b adds a force and a length");
        let err = d("sqrt(b)", &force_length).unwrap_err();
        assert!(err.contains("no square root"), "{err}");
        let err = d("max(a, b)", &force_length).unwrap_err();
        assert!(err.contains("one unit"), "{err}");
        let err = d("exp(b)", &force_length).unwrap_err();
        assert!(err.contains("without unit"), "{err}");
        let err = d("b^a", &force_length).unwrap_err();
        assert!(err.contains("exponent"), "{err}");
        let err = d("c", &force_length).unwrap_err();
        assert_eq!(err, "there is no input c");
    }

    #[test]
    fn units_can_be_written_after_numbers_and_as_names() {
        use Quantity::*;
        let a: &[f64] = &[0.05];
        let inputs = [(a, false)];
        let near = |text: &str, expected: f64| {
            let got = eval(text, &inputs)[0];
            assert!(
                (got - expected).abs() <= 1e-12 * expected.abs().max(1.0),
                "{text}: {got}"
            );
        };
        let d = |text: &str| Expr::parse(text).unwrap().dimension(&[dim(Length)]);
        near("a - 10mm", 0.04);
        near("a - 10 mm", 0.04);
        near("a - 10*mm", 0.04);
        near("a - 10[mm]", 0.04);
        near("a - 1.5e1mm", 0.035);
        near("2m", 2.0);
        near("10N/mm^2", 1e7);
        near("10*N/mm^2", 1e7);
        near("10[N/mm^2]", 1e7);
        near("10N/mm²", 1e7);
        near("210GPa", 210e9);
        near("10mm^2", 1e-5);
        near("10[mm]^2", 1e-4);
        near("(10mm)^2", 1e-4);
        near("5 kN m", 5e3);
        near("a / 10mm", 5.0);
        near("1 / 10N/mm^2", 1e-7);
        near("2 / 4mm/s", 500.0);
        near("2[N]/mm", 2e3);
        near("2 min", 120.0);
        near("sin(30deg)", 0.5);
        assert_eq!(d("a - 10mm"), Ok(dim(Length)));
        assert_eq!(d("10N/mm^2"), Ok(dim(Pressure)));
        assert_eq!(d("a * 10N"), Ok(dim(Moment)));
        assert_eq!(d("a / 10mm"), Ok(Dimension::default()));
        assert_eq!(d("10mm^2"), Ok(dim(Area)));
        assert_eq!(
            d("a / 10N/mm^2"),
            Ok(dim(Pressure).powi(-1).times(dim(Length)))
        );
        assert_eq!(d("10mm/s"), Ok(dim(Velocity)));
        assert_eq!(d("5 kN m"), Ok(dim(Moment)));
        assert_eq!(Expr::parse("10mm/b").unwrap().inputs(), vec![1]);
        assert_eq!(Expr::parse("5 kN m").unwrap().to_string(), "5 kN m");
        assert_eq!(
            Expr::parse("a/10N/mm^2").unwrap().to_string(),
            "a / (10 N / mm^2)"
        );
        assert_eq!(d("min(a, 10mm)"), Ok(dim(Length)));
        // A single lowercase letter alone is an input; right after a number, a unit.
        assert_eq!(Expr::parse("2*m").unwrap().inputs(), vec![12]);
        assert!(Expr::parse("2m").unwrap().inputs().is_empty());
        // Messages show the units as written.
        let err = |text: &str| {
            Expr::parse(text)
                .unwrap()
                .dimension(&[dim(Length)])
                .unwrap_err()
        };
        assert_eq!(
            err("a - 10"),
            "a - 10 subtracts a length and a number without unit: write the unit after the \
             number, such as 10mm"
        );
        assert_eq!(err("a + 10N"), "a + 10 N adds a length and a force");
        assert_eq!(
            err("a + 10[N]^2"),
            "a + ((10 N) ^ 2) adds a length and a number in kg² m²/s^4"
        );
        assert_eq!(
            err("a + 2[N/mm^2]"),
            "a + 2 [N/mm^2] adds a length and a pressure"
        );
        assert_eq!(
            Expr::parse("a")
                .unwrap()
                .dimension(&[dim(Pressure)])
                .map_err(|_| ()),
            Ok(dim(Pressure))
        );
        let pressure = |text: &str| {
            Expr::parse(text)
                .unwrap()
                .dimension(&[dim(Pressure)])
                .unwrap_err()
        };
        assert!(
            pressure("a + 3").ends_with("such as 3MPa"),
            "{}",
            pressure("a + 3")
        );
        assert!(pressure("2 - a").ends_with("such as 2MPa"));
        assert_eq!(
            pressure("a - 10N/mm^3"),
            "a - 10 N / mm^3 subtracts a pressure and a force per volume"
        );
        let moment = Expr::parse("a + 1")
            .unwrap()
            .dimension(&[dim(Moment)])
            .unwrap_err();
        assert!(moment.ends_with("such as 1[N·mm]"), "{moment}");
        // Names that are neither inputs nor units.
        let parse = |text: &str| Expr::parse(text).unwrap_err();
        assert!(
            parse("2 foo").contains("foo is not a unit"),
            "{}",
            parse("2 foo")
        );
        assert!(parse("a + foo").contains("neither an input nor a unit"));
    }

    #[test]
    fn units_after_numbers_are_not_renamed_as_inputs() {
        let down = |i: usize| Some(i + 1);
        assert_eq!(rename_inputs("a - 2m + 3 s + m", &down), "b - 2m + 3 s + n");
        assert_eq!(rename_inputs("2 * m - 1e3 g", &down), "2 * n - 1e3 g");
        assert_eq!(rename_inputs("2N/m + 4/m", &down), "2N/m + 4/n");
        assert_eq!(rename_inputs("2mm^-2/s * s", &down), "2mm^-2/s * t");
    }

    #[test]
    fn inputs_are_renamed_but_not_functions_units_or_exponents() {
        let down = |i: usize| match i {
            1 => None,
            0 => Some(0),
            _ => Some(i - 1),
        };
        assert_eq!(
            rename_inputs("max(a, c) * 2e3 + d[mm] / 5[mm] + e", &down),
            "max(a, b) * 2e3 + c[mm] / 5[mm] + d"
        );
        assert_eq!(rename_inputs("sqrt (c) + b", &down), "sqrt (b) + ?");
        assert!(
            Expr::parse("a + ?")
                .unwrap_err()
                .contains("link was removed")
        );
    }

    #[test]
    fn mistakes_are_explained() {
        let err = |text: &str| Expr::parse(text).unwrap_err();
        assert_eq!(err(""), "the formula is empty");
        assert_eq!(err("a +"), "the formula ends too early");
        assert_eq!(err("(a + b"), "a ( has no )");
        assert_eq!(err("a + b)"), "there is a ) without (");
        assert!(err("foo(a)").contains("no function foo"));
        assert!(err("ab").contains("neither an input nor a unit"));
        assert!(err("2a").contains("2*a"));
        assert!(err("sqrt(a, b)").contains("takes one value"));
        assert!(err("max()").contains("one series or several"));
        assert!(err("a + [furlong]").contains("not a unit"));
        assert!(err("sqrt").contains("is a function"));
        assert_eq!(
            Expr::parse("c * a + a").unwrap().inputs(),
            vec![0, 2],
            "the inputs used, each once"
        );
    }
}
