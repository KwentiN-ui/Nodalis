//! Equations of derived outputs and distributions in NCalc's syntax:
//! `=STRESS.MISES / 235`, with the arithmetic and comparison operators, `? :`, and functions
//! such as `Sqrt`, `Max` or `If`. Results are referred to as `FIELD.COMPONENT`; a name
//! containing characters an identifier cannot hold, like `Limit-1.RATIO`, is written in
//! brackets: `[Limit-1.RATIO]`.

/// A parsed equation; its variables are numbered in order of appearance.
#[derive(Clone, Debug, PartialEq)]
pub struct Equation {
    expr: Expr,
    variables: Vec<String>,
}

#[derive(Clone, Debug, PartialEq)]
enum Expr {
    Number(f64),
    Variable(usize),
    Neg(Box<Expr>),
    Not(Box<Expr>),
    Binary(Op, Box<Expr>, Box<Expr>),
    Conditional(Box<Expr>, Box<Expr>, Box<Expr>),
    Call(Function, Vec<Expr>),
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Op {
    Add,
    Sub,
    Mul,
    Div,
    Rem,
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
    And,
    Or,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Function {
    Abs,
    Acos,
    Asin,
    Atan,
    Ceiling,
    Cos,
    Exp,
    Floor,
    IeeeRemainder,
    Ln,
    Log,
    Log10,
    Max,
    Min,
    Pow,
    Round,
    Sign,
    Sin,
    Sqrt,
    Tan,
    Truncate,
    If,
}

impl Function {
    /// NCalc's functions plus `Ln`, with their number of arguments.
    const ALL: [(&'static str, Function, std::ops::RangeInclusive<usize>); 22] = [
        ("Abs", Function::Abs, 1..=1),
        ("Acos", Function::Acos, 1..=1),
        ("Asin", Function::Asin, 1..=1),
        ("Atan", Function::Atan, 1..=1),
        ("Ceiling", Function::Ceiling, 1..=1),
        ("Cos", Function::Cos, 1..=1),
        ("Exp", Function::Exp, 1..=1),
        ("Floor", Function::Floor, 1..=1),
        ("IEEERemainder", Function::IeeeRemainder, 2..=2),
        ("Ln", Function::Ln, 1..=1),
        ("Log", Function::Log, 2..=2),
        ("Log10", Function::Log10, 1..=1),
        ("Max", Function::Max, 2..=2),
        ("Min", Function::Min, 2..=2),
        ("Pow", Function::Pow, 2..=2),
        ("Round", Function::Round, 1..=2),
        ("Sign", Function::Sign, 1..=1),
        ("Sin", Function::Sin, 1..=1),
        ("Sqrt", Function::Sqrt, 1..=1),
        ("Tan", Function::Tan, 1..=1),
        ("Truncate", Function::Truncate, 1..=1),
        ("If", Function::If, 3..=3),
    ];
}

fn truth(value: f64) -> bool {
    value != 0.0 && !value.is_nan()
}

fn flag(value: bool) -> f64 {
    if value { 1.0 } else { 0.0 }
}

impl Equation {
    /// Parses an equation; a leading `=` is optional.
    pub fn parse(text: &str) -> Result<Self, String> {
        let text = text.trim();
        let text = text.strip_prefix('=').unwrap_or(text);
        if text.trim().is_empty() {
            return Err("The equation is empty.".into());
        }
        let mut parser = Parser {
            chars: text.chars().collect(),
            pos: 0,
            variables: Vec::new(),
        };
        let expr = parser.conditional()?;
        parser.skip_blanks();
        if parser.pos < parser.chars.len() {
            return Err(format!(
                "Unexpected character '{}' at position {}.",
                parser.chars[parser.pos],
                parser.pos + 1
            ));
        }
        Ok(Self {
            expr,
            variables: parser.variables,
        })
    }

    /// Names used in the equation, e.g. `STRESS.MISES`, in order of appearance.
    pub fn variables(&self) -> &[String] {
        &self.variables
    }

    /// Value for one set of variable values, given in the order of [`Equation::variables`].
    pub fn evaluate(&self, values: &[f64]) -> f64 {
        eval(&self.expr, values)
    }
}

fn eval(expr: &Expr, values: &[f64]) -> f64 {
    match expr {
        Expr::Number(value) => *value,
        Expr::Variable(index) => values[*index],
        Expr::Neg(inner) => -eval(inner, values),
        Expr::Not(inner) => flag(!truth(eval(inner, values))),
        Expr::Binary(op, a, b) => {
            let a = eval(a, values);
            // Short-circuit like NCalc.
            match op {
                Op::And if !truth(a) => return 0.0,
                Op::Or if truth(a) => return 1.0,
                _ => {}
            }
            let b = eval(b, values);
            match op {
                Op::Add => a + b,
                Op::Sub => a - b,
                Op::Mul => a * b,
                Op::Div => a / b,
                Op::Rem => a % b,
                Op::Lt => flag(a < b),
                Op::Le => flag(a <= b),
                Op::Gt => flag(a > b),
                Op::Ge => flag(a >= b),
                Op::Eq => flag(a == b),
                Op::Ne => flag(a != b),
                Op::And | Op::Or => flag(truth(b)),
            }
        }
        Expr::Conditional(condition, a, b) => {
            if truth(eval(condition, values)) {
                eval(a, values)
            } else {
                eval(b, values)
            }
        }
        Expr::Call(function, args) => {
            if *function == Function::If {
                let branch = if truth(eval(&args[0], values)) { 1 } else { 2 };
                return eval(&args[branch], values);
            }
            let x = eval(&args[0], values);
            let y = || eval(&args[1], values);
            match function {
                Function::Abs => x.abs(),
                Function::Acos => x.acos(),
                Function::Asin => x.asin(),
                Function::Atan => x.atan(),
                Function::Ceiling => x.ceil(),
                Function::Cos => x.cos(),
                Function::Exp => x.exp(),
                Function::Floor => x.floor(),
                Function::IeeeRemainder => {
                    let y = y();
                    x - y * (x / y).round_ties_even()
                }
                Function::Ln => x.ln(),
                Function::Log => x.log(y()),
                Function::Log10 => x.log10(),
                Function::Max => x.max(y()),
                Function::Min => x.min(y()),
                Function::Pow => x.powf(y()),
                Function::Round => {
                    let digits = if args.len() > 1 { y() as i32 } else { 0 };
                    let scale = 10f64.powi(digits);
                    // .NET's Math.Round rounds half to even.
                    (x * scale).round_ties_even() / scale
                }
                Function::Sign => {
                    if x > 0.0 {
                        1.0
                    } else if x < 0.0 {
                        -1.0
                    } else {
                        x * 0.0
                    }
                }
                Function::Sin => x.sin(),
                Function::Sqrt => x.sqrt(),
                Function::Tan => x.tan(),
                Function::Truncate => x.trunc(),
                Function::If => unreachable!(),
            }
        }
    }
}

struct Parser {
    chars: Vec<char>,
    pos: usize,
    variables: Vec<String>,
}

impl Parser {
    fn skip_blanks(&mut self) {
        while self.chars.get(self.pos).is_some_and(|c| c.is_whitespace()) {
            self.pos += 1;
        }
    }

    fn peek(&mut self) -> Option<char> {
        self.skip_blanks();
        self.chars.get(self.pos).copied()
    }

    /// Consumes `token` if it comes next.
    fn eat(&mut self, token: &str) -> bool {
        self.skip_blanks();
        let end = self.pos + token.chars().count();
        let matches =
            end <= self.chars.len() && self.chars[self.pos..end].iter().copied().eq(token.chars());
        if matches {
            self.pos = end;
        }
        matches
    }

    fn eat_word(&mut self, word: &str) -> bool {
        let start = self.pos;
        if self.eat(word) {
            let next = self.chars.get(self.pos);
            if !next.is_some_and(|c| c.is_alphanumeric() || *c == '_') {
                return true;
            }
        }
        self.pos = start;
        false
    }

    fn conditional(&mut self) -> Result<Expr, String> {
        let condition = self.or()?;
        if !self.eat("?") {
            return Ok(condition);
        }
        let a = self.conditional()?;
        if !self.eat(":") {
            return Err("':' is missing in the expression 'condition ? a : b'.".into());
        }
        let b = self.conditional()?;
        Ok(Expr::Conditional(
            Box::new(condition),
            Box::new(a),
            Box::new(b),
        ))
    }

    fn or(&mut self) -> Result<Expr, String> {
        let mut left = self.and()?;
        while self.eat("||") || self.eat_word("or") {
            left = Expr::Binary(Op::Or, Box::new(left), Box::new(self.and()?));
        }
        Ok(left)
    }

    fn and(&mut self) -> Result<Expr, String> {
        let mut left = self.comparison()?;
        while self.eat("&&") || self.eat_word("and") {
            left = Expr::Binary(Op::And, Box::new(left), Box::new(self.comparison()?));
        }
        Ok(left)
    }

    fn comparison(&mut self) -> Result<Expr, String> {
        let mut left = self.sum()?;
        loop {
            let op = if self.eat("==") {
                Op::Eq
            } else if self.eat("!=") || self.eat("<>") {
                Op::Ne
            } else if self.eat("<=") {
                Op::Le
            } else if self.eat(">=") {
                Op::Ge
            } else if self.eat("<") {
                Op::Lt
            } else if self.eat(">") {
                Op::Gt
            } else if self.eat("=") {
                Op::Eq
            } else {
                return Ok(left);
            };
            left = Expr::Binary(op, Box::new(left), Box::new(self.sum()?));
        }
    }

    fn sum(&mut self) -> Result<Expr, String> {
        let mut left = self.product()?;
        loop {
            let op = if self.eat("+") {
                Op::Add
            } else if self.eat("-") {
                Op::Sub
            } else {
                return Ok(left);
            };
            left = Expr::Binary(op, Box::new(left), Box::new(self.product()?));
        }
    }

    fn product(&mut self) -> Result<Expr, String> {
        let mut left = self.unary()?;
        loop {
            let op = if self.eat("*") {
                Op::Mul
            } else if self.eat("/") {
                Op::Div
            } else if self.eat("%") {
                Op::Rem
            } else {
                return Ok(left);
            };
            left = Expr::Binary(op, Box::new(left), Box::new(self.unary()?));
        }
    }

    fn unary(&mut self) -> Result<Expr, String> {
        if self.eat("-") {
            return Ok(Expr::Neg(Box::new(self.unary()?)));
        }
        if self.eat("+") {
            return self.unary();
        }
        if self.eat("!") || self.eat_word("not") {
            return Ok(Expr::Not(Box::new(self.unary()?)));
        }
        self.primary()
    }

    fn variable(&mut self, name: String) -> Expr {
        let index = match self.variables.iter().position(|v| *v == name) {
            Some(index) => index,
            None => {
                self.variables.push(name);
                self.variables.len() - 1
            }
        };
        Expr::Variable(index)
    }

    fn primary(&mut self) -> Result<Expr, String> {
        let Some(c) = self.peek() else {
            return Err("The equation ends unexpectedly.".into());
        };
        if self.eat("(") {
            let inner = self.conditional()?;
            if !self.eat(")") {
                return Err("Closing bracket ')' is missing.".into());
            }
            return Ok(inner);
        }
        if self.eat("[") {
            let start = self.pos;
            while self.chars.get(self.pos).is_some_and(|&c| c != ']') {
                self.pos += 1;
            }
            if self.pos >= self.chars.len() {
                return Err("Closing bracket ']' is missing.".into());
            }
            let name: String = self.chars[start..self.pos].iter().collect();
            self.pos += 1;
            return Ok(self.variable(name.trim().to_string()));
        }
        if c.is_ascii_digit() || c == '.' {
            return self.number();
        }
        if c.is_alphabetic() || c == '_' {
            let start = self.pos;
            while self
                .chars
                .get(self.pos)
                .is_some_and(|&c| c.is_alphanumeric() || c == '_' || c == '.')
            {
                self.pos += 1;
            }
            let name: String = self.chars[start..self.pos].iter().collect();
            if self.eat("(") {
                return self.call(&name);
            }
            return Ok(match name.as_str() {
                "pi" | "Pi" => Expr::Number(std::f64::consts::PI),
                "true" => Expr::Number(1.0),
                "false" => Expr::Number(0.0),
                _ => self.variable(name),
            });
        }
        Err(format!(
            "Unexpected character '{c}' at position {}.",
            self.pos + 1
        ))
    }

    fn number(&mut self) -> Result<Expr, String> {
        let start = self.pos;
        let digits = |p: &mut Self| {
            while p.chars.get(p.pos).is_some_and(char::is_ascii_digit) {
                p.pos += 1;
            }
        };
        digits(self);
        if self.chars.get(self.pos) == Some(&'.') {
            self.pos += 1;
            digits(self);
        }
        if self
            .chars
            .get(self.pos)
            .is_some_and(|c| *c == 'e' || *c == 'E')
        {
            let mark = self.pos;
            self.pos += 1;
            if self
                .chars
                .get(self.pos)
                .is_some_and(|c| *c == '+' || *c == '-')
            {
                self.pos += 1;
            }
            if self.chars.get(self.pos).is_some_and(char::is_ascii_digit) {
                digits(self);
            } else {
                self.pos = mark;
            }
        }
        let text: String = self.chars[start..self.pos].iter().collect();
        text.parse()
            .map(Expr::Number)
            .map_err(|_| format!("Invalid number '{text}'."))
    }

    fn call(&mut self, name: &str) -> Result<Expr, String> {
        let Some((canonical, function, arity)) = Function::ALL
            .iter()
            .find(|(n, ..)| n.eq_ignore_ascii_case(name))
        else {
            return Err(format!("Unknown function '{name}'."));
        };
        let mut args = Vec::new();
        if !self.eat(")") {
            loop {
                args.push(self.conditional()?);
                if self.eat(")") {
                    break;
                }
                if !self.eat(",") {
                    return Err(format!("',' or ')' is missing in the call of {canonical}."));
                }
            }
        }
        if !arity.contains(&args.len()) {
            return Err(format!(
                "{canonical} expects {} argument(s), not {}.",
                arity.start(),
                args.len()
            ));
        }
        Ok(Expr::Call(*function, args))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn value(text: &str) -> f64 {
        Equation::parse(text).unwrap().evaluate(&[])
    }

    #[test]
    fn arithmetic_follows_precedence() {
        assert_eq!(value("=1 + 2 * 3"), 7.0);
        assert_eq!(value("(1 + 2) * 3"), 9.0);
        assert_eq!(value("-2 * -3"), 6.0);
        assert_eq!(value("7 % 4"), 3.0);
        assert_eq!(value("1.5e2 / 3"), 50.0);
        assert_eq!(value("10 - 4 - 3"), 3.0);
    }

    #[test]
    fn functions_and_conditions() {
        assert_eq!(value("Sqrt(16) + Max(2, 5)"), 9.0);
        assert_eq!(value("pow(2, 10)"), 1024.0);
        assert_eq!(value("If(3 > 2, 1, 2)"), 1.0);
        assert_eq!(value("2 >= 3 ? 1 : 2"), 2.0);
        assert_eq!(value("1 < 2 && 2 < 1"), 0.0);
        assert_eq!(value("Round(2.5)"), 2.0);
        assert_eq!(value("Round(1.2345, 2)"), 1.23);
        assert!((value("Pi") - std::f64::consts::PI).abs() < 1e-12);
    }

    #[test]
    fn variables_are_numbered_once() {
        let equation =
            Equation::parse("=STRESS.MISES / 235 + STRESS.MISES * [Limit-1.RATIO]").unwrap();
        assert_eq!(equation.variables(), ["STRESS.MISES", "Limit-1.RATIO"]);
        assert_eq!(equation.evaluate(&[470.0, 2.0]), 942.0);
    }

    #[test]
    fn errors_are_reported() {
        assert!(Equation::parse("=").is_err());
        assert!(Equation::parse("1 +").is_err());
        assert!(Equation::parse("Foo(1)").is_err());
        assert!(Equation::parse("Max(1)").is_err());
        assert!(Equation::parse("(1 + 2").is_err());
        assert!(Equation::parse("1 2").is_err());
    }
}
