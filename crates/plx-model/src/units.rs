//! Units of the model's values.
//!
//! Values are stored as plain numbers in the units of the model's [`UnitSystem`]. What a
//! number means is told by its [`Quantity`]: the GUI shows the unit next to it, takes values
//! typed in other units ("3 cm" becomes 30 in a millimetre model) and converts the model when
//! the unit system changes.

use serde::{Deserialize, Serialize};

/// The units the model's values are given in.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum UnitSystem {
    Unitless,
    MKgSC,
    #[default]
    MmTonSC,
    MTonSC,
    InLbSF,
}

/// Kinds of quantities the unit system names a unit for.
pub const BASE_QUANTITIES: [Quantity; 5] = [
    Quantity::Length,
    Quantity::Angle,
    Quantity::Mass,
    Quantity::Time,
    Quantity::Temperature,
];
pub const DERIVED_QUANTITIES: [Quantity; 13] = [
    Quantity::Area,
    Quantity::Volume,
    Quantity::Velocity,
    Quantity::RotationalSpeed,
    Quantity::Acceleration,
    Quantity::Force,
    Quantity::ForcePerLength,
    Quantity::Moment,
    Quantity::Pressure,
    Quantity::Density,
    Quantity::Energy,
    Quantity::Power,
    Quantity::Frequency,
];

/// What a value of the model measures; decides its unit in every unit system.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Quantity {
    Length,
    Angle,
    Mass,
    Time,
    /// Absolute temperature, converted with the offset of its scale.
    Temperature,
    Area,
    /// Second moment of area of a beam profile.
    SecondMomentOfArea,
    Volume,
    Velocity,
    RotationalSpeed,
    Acceleration,
    Force,
    /// Also the stiffness of a spring.
    ForcePerLength,
    Moment,
    /// Also stress and the elastic modulus.
    Pressure,
    Density,
    /// Mass moment of inertia, the rotary inertia of a point mass.
    MomentOfInertia,
    Energy,
    Power,
    Frequency,
    /// Stiffness per area of a spring or of a contact, pressure per length.
    ForcePerVolume,
    /// Heat flow per area and temperature difference, the gap conductance.
    HeatTransferCoefficient,
    /// A change of temperature, converted without the offset of the scale.
    TemperatureDifference,
    /// Heat flow per length and temperature difference.
    ThermalConductivity,
    /// Energy per mass and temperature difference.
    SpecificHeat,
    /// Strain per temperature difference.
    ThermalExpansion,
    /// Heat flow per area.
    HeatFlux,
    /// Heat generated per volume.
    PowerPerVolume,
    /// Radiated power per area and fourth power of the absolute temperature.
    StefanBoltzmann,
}

impl Quantity {
    /// Every quantity, in the order a list offers them.
    pub const ALL: [Quantity; 29] = [
        Quantity::Length,
        Quantity::Angle,
        Quantity::Mass,
        Quantity::Time,
        Quantity::Temperature,
        Quantity::Area,
        Quantity::SecondMomentOfArea,
        Quantity::Volume,
        Quantity::Velocity,
        Quantity::RotationalSpeed,
        Quantity::Acceleration,
        Quantity::Force,
        Quantity::ForcePerLength,
        Quantity::Moment,
        Quantity::Pressure,
        Quantity::Density,
        Quantity::MomentOfInertia,
        Quantity::Energy,
        Quantity::Power,
        Quantity::Frequency,
        Quantity::ForcePerVolume,
        Quantity::HeatTransferCoefficient,
        Quantity::TemperatureDifference,
        Quantity::ThermalConductivity,
        Quantity::SpecificHeat,
        Quantity::ThermalExpansion,
        Quantity::HeatFlux,
        Quantity::PowerPerVolume,
        Quantity::StefanBoltzmann,
    ];

    /// Name in the GUI.
    pub fn label(self) -> &'static str {
        match self {
            Quantity::Length => "Length",
            Quantity::Angle => "Angle",
            Quantity::Mass => "Mass",
            Quantity::Time => "Time",
            Quantity::Temperature => "Temperature",
            Quantity::Area => "Area",
            Quantity::SecondMomentOfArea => "Second moment of area",
            Quantity::Volume => "Volume",
            Quantity::Velocity => "Velocity",
            Quantity::RotationalSpeed => "Rotational speed",
            Quantity::Acceleration => "Acceleration",
            Quantity::Force => "Force",
            Quantity::ForcePerLength => "Force per length",
            Quantity::Moment => "Moment",
            Quantity::Pressure => "Pressure",
            Quantity::Density => "Density",
            Quantity::MomentOfInertia => "Moment of inertia",
            Quantity::Energy => "Energy",
            Quantity::Power => "Power",
            Quantity::Frequency => "Frequency",
            Quantity::ForcePerVolume => "Force per volume",
            Quantity::HeatTransferCoefficient => "Heat transfer coefficient",
            Quantity::TemperatureDifference => "Temperature difference",
            Quantity::ThermalConductivity => "Thermal conductivity",
            Quantity::SpecificHeat => "Specific heat",
            Quantity::ThermalExpansion => "Thermal expansion coefficient",
            Quantity::HeatFlux => "Heat flux",
            Quantity::PowerPerVolume => "Power per volume",
            Quantity::StefanBoltzmann => "Stefan-Boltzmann constant",
        }
    }

    pub fn dimension(self) -> Dimension {
        let d = Dimension::new;
        match self {
            Quantity::Length => d(1, 0, 0, 0, 0),
            Quantity::Angle => d(0, 0, 0, 0, 1),
            Quantity::Mass => d(0, 1, 0, 0, 0),
            Quantity::Time => d(0, 0, 1, 0, 0),
            Quantity::Temperature => d(0, 0, 0, 1, 0),
            Quantity::Area => d(2, 0, 0, 0, 0),
            Quantity::SecondMomentOfArea => d(4, 0, 0, 0, 0),
            Quantity::Volume => d(3, 0, 0, 0, 0),
            Quantity::Velocity => d(1, 0, -1, 0, 0),
            Quantity::RotationalSpeed => d(0, 0, -1, 0, 1),
            Quantity::Acceleration => d(1, 0, -2, 0, 0),
            Quantity::Force => d(1, 1, -2, 0, 0),
            Quantity::ForcePerLength => d(0, 1, -2, 0, 0),
            Quantity::Moment | Quantity::Energy => d(2, 1, -2, 0, 0),
            Quantity::Pressure => d(-1, 1, -2, 0, 0),
            Quantity::Density => d(-3, 1, 0, 0, 0),
            Quantity::MomentOfInertia => d(2, 1, 0, 0, 0),
            Quantity::Power => d(2, 1, -3, 0, 0),
            Quantity::Frequency => d(0, 0, -1, 0, 0),
            Quantity::ForcePerVolume => d(-2, 1, -2, 0, 0),
            Quantity::HeatTransferCoefficient => d(0, 1, -3, -1, 0),
            Quantity::TemperatureDifference => d(0, 0, 0, 1, 0),
            Quantity::ThermalConductivity => d(1, 1, -3, -1, 0),
            Quantity::SpecificHeat => d(2, 0, -2, -1, 0),
            Quantity::ThermalExpansion => d(0, 0, 0, -1, 0),
            Quantity::HeatFlux => d(0, 1, -3, 0, 0),
            Quantity::PowerPerVolume => d(-1, 1, -3, 0, 0),
            Quantity::StefanBoltzmann => d(0, 1, -3, -4, 0),
        }
    }
}

/// Powers of length, mass, time, temperature and angle a quantity is made of.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Dimension {
    pub length: i8,
    pub mass: i8,
    pub time: i8,
    pub temperature: i8,
    pub angle: i8,
}

impl Dimension {
    pub const fn new(length: i8, mass: i8, time: i8, temperature: i8, angle: i8) -> Self {
        Self {
            length,
            mass,
            time,
            temperature,
            angle,
        }
    }

    pub fn powi(self, n: i8) -> Self {
        Self::new(
            self.length * n,
            self.mass * n,
            self.time * n,
            self.temperature * n,
            self.angle * n,
        )
    }

    pub fn times(self, other: Self) -> Self {
        Self::new(
            self.length + other.length,
            self.mass + other.mass,
            self.time + other.time,
            self.temperature + other.temperature,
            self.angle + other.angle,
        )
    }

    /// This dimension divided by `other`.
    pub fn over(self, other: Self) -> Self {
        self.times(other.powi(-1))
    }

    /// Whether it is a pure number, without unit.
    pub fn is_none(self) -> bool {
        self == Self::default()
    }

    /// The quantity of this dimension, `None` for a pure number or one no quantity has. Of
    /// those that share one, a moment goes before an energy, and a temperature is a change
    /// of temperature: a product or quotient of temperatures has no zero of its scale.
    pub fn quantity(self) -> Option<Quantity> {
        Quantity::ALL
            .into_iter()
            .filter(|q| *q != Quantity::Temperature)
            .find(|q| q.dimension() == self)
    }

    /// The dimension in SI base units, such as "kg/(m s²)", short for the narrow fields of
    /// the blocks.
    pub fn si(self) -> String {
        let units = [
            ("kg", self.mass),
            ("m", self.length),
            ("s", self.time),
            ("K", self.temperature),
            ("rad", self.angle),
        ];
        let part = |(unit, power): (&str, i8)| match power {
            1 => unit.to_owned(),
            2 => format!("{unit}²"),
            3 => format!("{unit}³"),
            _ => format!("{unit}^{power}"),
        };
        let over: Vec<String> = (units.into_iter())
            .filter(|(_, power)| *power > 0)
            .map(part)
            .collect();
        let under: Vec<String> = (units.into_iter())
            .filter(|(_, power)| *power < 0)
            .map(|(unit, power)| part((unit, -power)))
            .collect();
        let over = if over.is_empty() {
            "1".to_owned()
        } else {
            over.join(" ")
        };
        match under.len() {
            0 if over == "1" => "no unit".to_owned(),
            0 => over,
            1 => format!("{over}/{}", under[0]),
            _ => format!("{over}/({})", under.join(" ")),
        }
    }
}

/// Pound-force in newton.
const POUND_FORCE: f64 = 4.448_221_615_260_5;
const INCH: f64 = 0.0254;

/// The unit system the values of every model are stored in: metre, kilogram, second and
/// degree Celsius. An analysis' own unit system only says in which units its dialogs and
/// views show the values, and in which units files without units are read, so analyses of
/// different unit systems can take values and results from each other as they are.
/// The solver gets the values in these units, too.
pub const STORED: UnitSystem = UnitSystem::MKgSC;

impl UnitSystem {
    /// In the order of the list.
    pub const ALL: [UnitSystem; 5] = [
        UnitSystem::Unitless,
        UnitSystem::MKgSC,
        UnitSystem::MmTonSC,
        UnitSystem::MTonSC,
        UnitSystem::InLbSF,
    ];

    /// The name of the unit system.
    pub fn label(self) -> &'static str {
        match self {
            UnitSystem::Unitless => "Unitless",
            UnitSystem::MKgSC => "m, kg, s, °C",
            UnitSystem::MmTonSC => "mm, ton, s, °C",
            UnitSystem::MTonSC => "m, ton, s, °C",
            UnitSystem::InLbSF => "in, lbf·s²/in, s, °F",
        }
    }

    pub fn has_units(self) -> bool {
        self != UnitSystem::Unitless
    }

    /// Units of [`BASE_QUANTITIES`], empty without units.
    pub fn base_units(self) -> [&'static str; 5] {
        BASE_QUANTITIES.map(|q| self.unit(q))
    }

    /// Units of [`DERIVED_QUANTITIES`], empty without units.
    pub fn derived_units(self) -> [&'static str; 13] {
        DERIVED_QUANTITIES.map(|q| self.unit(q))
    }

    /// The unit of a quantity, as shown next to values; empty without units.
    pub fn unit(self, quantity: Quantity) -> &'static str {
        use Quantity::*;
        // Units of m-kg, mm-t, m-t and in-lbf.
        let [si, mm, m_t, inch] = match quantity {
            Length => ["m", "mm", "m", "in"],
            Angle => ["rad"; 4],
            Mass => ["kg", "t", "t", "lbf·s²/in"],
            Time => ["s"; 4],
            Temperature => ["°C", "°C", "°C", "°F"],
            Area => ["m²", "mm²", "m²", "in²"],
            SecondMomentOfArea => ["m^4", "mm^4", "m^4", "in^4"],
            Volume => ["m³", "mm³", "m³", "in³"],
            Velocity => ["m/s", "mm/s", "m/s", "in/s"],
            RotationalSpeed => ["rad/s"; 4],
            Acceleration => ["m/s²", "mm/s²", "m/s²", "in/s²"],
            Force => ["N", "N", "kN", "lbf"],
            ForcePerLength => ["N/m", "N/mm", "kN/m", "lbf/in"],
            Moment => ["N·m", "N·mm", "kN·m", "lbf·in"],
            Pressure => ["Pa", "MPa", "kPa", "psi"],
            // The fourth power has no Latin-1 character, which the GUI font sticks to.
            Density => ["kg/m³", "t/mm³", "t/m³", "lbf·s²/in^4"],
            MomentOfInertia => ["kg·m²", "t·mm²", "t·m²", "lbf·s²·in"],
            Energy => ["J", "mJ", "kJ", "lbf·in"],
            Power => ["W", "mW", "kW", "lbf·in/s"],
            Frequency => ["Hz"; 4],
            ForcePerVolume => ["N/m³", "N/mm³", "kN/m³", "lbf/in³"],
            HeatTransferCoefficient => ["W/(m²·°C)", "mW/(mm²·°C)", "kW/(m²·°C)", "lbf/(in·s·°F)"],
            TemperatureDifference => ["°C", "°C", "°C", "°F"],
            ThermalConductivity => ["W/(m·°C)", "mW/(mm·°C)", "kW/(m·°C)", "lbf/(s·°F)"],
            SpecificHeat => ["J/(kg·°C)", "mJ/(t·°C)", "kJ/(t·°C)", "in²/(s²·°F)"],
            ThermalExpansion => ["1/°C", "1/°C", "1/°C", "1/°F"],
            HeatFlux => ["W/m²", "mW/mm²", "kW/m²", "lbf/(in·s)"],
            PowerPerVolume => ["W/m³", "mW/mm³", "kW/m³", "lbf/(in²·s)"],
            StefanBoltzmann => [
                "W/(m²·K^4)",
                "mW/(mm²·K^4)",
                "kW/(m²·K^4)",
                "lbf/(in·s·°F^4)",
            ],
        };
        match self {
            UnitSystem::Unitless => "",
            UnitSystem::MKgSC => si,
            UnitSystem::MmTonSC => mm,
            UnitSystem::MTonSC => m_t,
            UnitSystem::InLbSF => inch,
        }
    }

    /// Length, mass, time and temperature difference of the system's units in SI units;
    /// `None` without units.
    fn base(self) -> Option<[f64; 4]> {
        match self {
            UnitSystem::Unitless => None,
            UnitSystem::MKgSC => Some([1.0, 1.0, 1.0, 1.0]),
            UnitSystem::MmTonSC => Some([1e-3, 1e3, 1.0, 1.0]),
            UnitSystem::MTonSC => Some([1.0, 1e3, 1.0, 1.0]),
            UnitSystem::InLbSF => Some([INCH, POUND_FORCE / INCH, 1.0, 5.0 / 9.0]),
        }
    }

    /// Kelvin at zero of the system's temperature scale.
    fn temperature_zero(self) -> f64 {
        match self {
            UnitSystem::InLbSF => 459.67 * 5.0 / 9.0,
            _ => 273.15,
        }
    }

    /// One unit of the quantity in SI units (angles in radian, temperature differences in
    /// kelvin); `None` without units.
    pub fn si_factor(self, quantity: Quantity) -> Option<f64> {
        let [length, mass, time, temperature] = self.base()?;
        let d = quantity.dimension();
        Some(
            length.powi(d.length.into())
                * mass.powi(d.mass.into())
                * time.powi(d.time.into())
                * temperature.powi(d.temperature.into()),
        )
    }

    /// A value of this system in SI units; temperatures in kelvin.
    fn in_si(self, value: f64, quantity: Quantity) -> Option<f64> {
        let si = value * self.si_factor(quantity)?;
        Some(if quantity == Quantity::Temperature {
            si + self.temperature_zero()
        } else {
            si
        })
    }

    fn of_si(self, si: f64, quantity: Quantity) -> Option<f64> {
        let si = if quantity == Quantity::Temperature {
            si - self.temperature_zero()
        } else {
            si
        };
        Some(si / self.si_factor(quantity)?)
    }

    /// A value as it is stored ([`STORED`]) in this system's units, to be shown. Without
    /// units, it stays.
    pub fn shown(self, stored: f64, quantity: Quantity) -> f64 {
        STORED.convert(stored, quantity, self)
    }

    /// A value in this system's units as it is stored ([`STORED`]).
    pub fn stored(self, value: f64, quantity: Quantity) -> f64 {
        self.convert(value, quantity, STORED)
    }

    /// The value in the units of `target`. Without units on either side, it stays.
    pub fn convert(self, value: f64, quantity: Quantity, target: UnitSystem) -> f64 {
        self.in_si(value, quantity)
            .and_then(|si| target.of_si(si, quantity))
            .unwrap_or(value)
    }

    /// Factor that turns values of this system into values of `target`, 1 without units
    /// on either side. Absolute temperatures also shift; [`Self::convert`] handles them.
    pub fn factor_to(self, quantity: Quantity, target: UnitSystem) -> f64 {
        match (self.si_factor(quantity), target.si_factor(quantity)) {
            (Some(from), Some(to)) => from / to,
            _ => 1.0,
        }
    }

    /// Reads a value typed by the user, in this system's units unless the text names other
    /// ones: "3 cm", "3cm" and "30" all give 30 in a millimetre model. The number takes `.`
    /// or `,` as decimal separator and e-notation ([`parse_number`]) and may be a calculation
    /// ([`evaluate`]) whose result the unit applies to, e.g. "100/3 mm"; the unit may combine
    /// units with prefixes, products, quotients and powers, e.g. "kN·m", "Nmm", "N/mm²",
    /// "200 GPa", "7,85 g/cm³", "5°".
    pub fn parse_value(self, text: &str, quantity: Quantity) -> Result<f64, String> {
        let (number, unit) =
            split_expression(text).ok_or_else(|| format!("Not a number: {text}"))?;
        if unit.is_empty() {
            return Ok(number);
        }
        if !self.has_units() {
            return Err("The model has no unit system; please enter only the number.".into());
        }
        let parsed = parse_unit(unit)?;
        if parsed.dimension != quantity.dimension() {
            return Err(format!(
                "{unit} is not a unit of {} ({})",
                quantity.label(),
                self.unit(quantity)
            ));
        }
        // A temperature in °C or °F counts from its scale's zero; one in K is absolute. Other
        // quantities only measure differences ("1/°C").
        let zero = if quantity == Quantity::Temperature {
            parsed.zero.unwrap_or(0.0)
        } else {
            0.0
        };
        let si = number * parsed.factor + zero;
        let value = if quantity == Quantity::Temperature {
            self.of_si(si, quantity)
        } else {
            self.si_factor(quantity).map(|f| si / f)
        };
        value
            .filter(|v| v.is_finite())
            .ok_or_else(|| format!("Value out of range: {text}"))
    }
}

/// Parses a number typed by the user.
///
/// Both `.` and `,` are accepted as decimal separator, as is e-notation
/// (`2,1e5`, `2.1E-3`). There are no thousands separators, so `1.000,5` and
/// `1 000` are rejected. Non-finite values are rejected as well.
pub fn parse_number(text: &str) -> Option<f64> {
    let text: String = text
        .trim()
        .chars()
        .map(|c| match c {
            ',' => '.',
            // Typographic minus, as egui's own parser accepts it.
            '\u{2212}' => '-',
            c => c,
        })
        .collect();
    if !text
        .chars()
        .all(|c| c.is_ascii_digit() || "+-.eE".contains(c))
    {
        return None;
    }
    text.parse::<f64>().ok().filter(|v| v.is_finite())
}

/// Evaluates simple arithmetic typed by the user, e.g. "1/30" or "2*(3,5-1)": numbers as
/// [`parse_number`] reads them, `+ - * /` (also `−`, `×`, `÷`), signs and parentheses. Only
/// the result counts; it must be finite, so "1/0" is rejected.
pub fn evaluate(text: &str) -> Option<f64> {
    let tokens = tokenize(text)?;
    let mut parser = Calculation {
        tokens: &tokens,
        at: 0,
    };
    let value = parser.sum()?;
    (parser.at == tokens.len() && value.is_finite()).then_some(value)
}

#[derive(Clone, Copy, PartialEq)]
enum Token {
    Number(f64),
    Operator(char),
}

fn tokenize(text: &str) -> Option<Vec<Token>> {
    let mut tokens = Vec::new();
    let mut chars = text.char_indices().peekable();
    while let Some((start, c)) = chars.next() {
        match c {
            c if c.is_whitespace() => {}
            '+' | '(' | ')' | '*' | '/' => tokens.push(Token::Operator(c)),
            '-' | '\u{2212}' => tokens.push(Token::Operator('-')),
            '×' | '·' => tokens.push(Token::Operator('*')),
            '÷' => tokens.push(Token::Operator('/')),
            c if c.is_ascii_digit() || c == '.' || c == ',' => {
                let mut end = start + c.len_utf8();
                while let Some(&(i, c)) = chars.peek() {
                    let exponent = (c == 'e' || c == 'E') && {
                        // An exponent only when digits follow, possibly after a sign.
                        let rest = &text[i + 1..];
                        let rest = rest.strip_prefix(['+', '-']).unwrap_or(rest);
                        rest.starts_with(|c: char| c.is_ascii_digit())
                    };
                    if c.is_ascii_digit() || c == '.' || c == ',' {
                        end = i + 1;
                        chars.next();
                    } else if exponent {
                        chars.next();
                        if let Some(&(j, '+' | '-')) = chars.peek() {
                            end = j + 1;
                            chars.next();
                        }
                        while let Some(&(j, c)) = chars.peek()
                            && c.is_ascii_digit()
                        {
                            end = j + 1;
                            chars.next();
                        }
                        break;
                    } else {
                        break;
                    }
                }
                tokens.push(Token::Number(parse_number(&text[start..end])?));
            }
            _ => return None,
        }
    }
    Some(tokens)
}

/// A recursive descent over the tokens of [`evaluate`].
struct Calculation<'a> {
    tokens: &'a [Token],
    at: usize,
}

impl Calculation<'_> {
    fn next_operator(&mut self, of: &str) -> Option<char> {
        match self.tokens.get(self.at) {
            Some(&Token::Operator(c)) if of.contains(c) => {
                self.at += 1;
                Some(c)
            }
            _ => None,
        }
    }

    fn sum(&mut self) -> Option<f64> {
        let mut value = self.product()?;
        while let Some(op) = self.next_operator("+-") {
            let term = self.product()?;
            value = if op == '+' {
                value + term
            } else {
                value - term
            };
        }
        Some(value)
    }

    fn product(&mut self) -> Option<f64> {
        let mut value = self.factor()?;
        while let Some(op) = self.next_operator("*/") {
            let factor = self.factor()?;
            value = if op == '*' {
                value * factor
            } else {
                value / factor
            };
        }
        Some(value)
    }

    fn factor(&mut self) -> Option<f64> {
        if let Some(sign) = self.next_operator("+-") {
            let value = self.factor()?;
            return Some(if sign == '-' { -value } else { value });
        }
        if self.next_operator("(").is_some() {
            let value = self.sum()?;
            self.next_operator(")")?;
            return Some(value);
        }
        match self.tokens.get(self.at) {
            Some(&Token::Number(value)) => {
                self.at += 1;
                Some(value)
            }
            _ => None,
        }
    }
}

/// The calculation at the start of the text and the unit after it: the longest start that
/// [`evaluate`]s, so that "2e5Pa" is 2e5 Pa and "100/3 mm" is 33.3 mm.
fn split_expression(text: &str) -> Option<(f64, &str)> {
    let text = text.trim();
    let ends: Vec<usize> = (text.char_indices().map(|(i, _)| i))
        .skip(1)
        .chain([text.len()])
        .collect();
    ends.into_iter()
        .rev()
        .find_map(|end| Some((evaluate(&text[..end])?, text[end..].trim())))
}

/// A unit read from text: its size in SI units and what it measures.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParsedUnit {
    pub factor: f64,
    pub dimension: Dimension,
    /// Kelvin at zero of a temperature scale, when the text is just that scale's unit
    /// (°C, °F); a temperature in it is absolute.
    pub zero: Option<f64>,
}

/// Units that can be typed. The bool marks units that take an SI prefix.
const UNITS: &[(&str, f64, Dimension, bool)] = {
    const L: Dimension = Dimension::new(1, 0, 0, 0, 0);
    const M: Dimension = Dimension::new(0, 1, 0, 0, 0);
    const T: Dimension = Dimension::new(0, 0, 1, 0, 0);
    const K: Dimension = Dimension::new(0, 0, 0, 1, 0);
    const A: Dimension = Dimension::new(0, 0, 0, 0, 1);
    const F: Dimension = Dimension::new(1, 1, -2, 0, 0);
    const P: Dimension = Dimension::new(-1, 1, -2, 0, 0);
    const E: Dimension = Dimension::new(2, 1, -2, 0, 0);
    const W: Dimension = Dimension::new(2, 1, -3, 0, 0);
    const HZ: Dimension = Dimension::new(0, 0, -1, 0, 0);
    const RPM: Dimension = Dimension::new(0, 0, -1, 0, 1);
    const DEG: f64 = std::f64::consts::PI / 180.0;
    &[
        ("m", 1.0, L, true),
        ("in", INCH, L, false),
        ("inch", INCH, L, false),
        ("ft", 0.3048, L, false),
        ("g", 1e-3, M, true),
        ("t", 1e3, M, false),
        ("ton", 1e3, M, false),
        ("lb", 0.453_592_37, M, false),
        ("lbm", 0.453_592_37, M, false),
        ("s", 1.0, T, true),
        ("min", 60.0, T, false),
        ("h", 3600.0, T, false),
        ("K", 1.0, K, false),
        ("°C", 1.0, K, false),
        ("degC", 1.0, K, false),
        ("°F", 5.0 / 9.0, K, false),
        ("degF", 5.0 / 9.0, K, false),
        ("rad", 1.0, A, true),
        ("°", DEG, A, false),
        ("deg", DEG, A, false),
        ("N", 1.0, F, true),
        ("lbf", POUND_FORCE, F, false),
        ("kip", 1e3 * POUND_FORCE, F, false),
        ("Pa", 1.0, P, true),
        ("bar", 1e5, P, true),
        ("psi", POUND_FORCE / (INCH * INCH), P, false),
        ("ksi", 1e3 * POUND_FORCE / (INCH * INCH), P, false),
        ("J", 1.0, E, true),
        ("W", 1.0, W, true),
        ("Hz", 1.0, HZ, true),
        ("rpm", 2.0 * std::f64::consts::PI / 60.0, RPM, false),
    ]
};

const PREFIXES: &[(&str, f64)] = &[
    ("p", 1e-12),
    ("n", 1e-9),
    ("µ", 1e-6),
    ("u", 1e-6),
    ("m", 1e-3),
    ("c", 1e-2),
    ("d", 1e-1),
    ("k", 1e3),
    ("M", 1e6),
    ("G", 1e9),
    ("T", 1e12),
];

/// A unit symbol read: its size in SI units, what it measures and its name without prefix.
type Symbol = (f64, Dimension, &'static str);

/// One unit symbol, possibly with a prefix: "mm", "kN", "°C".
fn symbol(text: &str) -> Option<Symbol> {
    if let Some(&(name, factor, dimension, _)) = UNITS.iter().find(|u| u.0 == text) {
        return Some((factor, dimension, name));
    }
    PREFIXES.iter().find_map(|&(prefix, scale)| {
        let rest = text.strip_prefix(prefix)?;
        let &(name, factor, dimension, _) = UNITS.iter().find(|u| u.0 == rest && u.3)?;
        Some((scale * factor, dimension, name))
    })
}

/// A run of letters as a product of unit symbols, with as few symbols as possible, so that
/// "Nmm" is N·mm and "kNm" is kN·m.
fn word(text: &str) -> Option<Vec<Symbol>> {
    let ends: Vec<usize> = (text.char_indices().map(|(i, _)| i))
        .skip(1)
        .chain([text.len()])
        .collect();
    let starts: Vec<usize> = text.char_indices().map(|(i, _)| i).collect();
    // best[k]: the fewest symbols that make up the text up to the k-th end.
    let mut best: Vec<Option<Vec<Symbol>>> = vec![None; ends.len()];
    for (k, &end) in ends.iter().enumerate() {
        for (j, &start) in starts.iter().enumerate().take(k + 1) {
            let before = if j == 0 {
                Some(Vec::new())
            } else {
                best[j - 1].clone()
            };
            let (Some(mut before), Some(unit)) = (before, symbol(&text[start..end])) else {
                continue;
            };
            before.push(unit);
            if best[k].as_ref().is_none_or(|b| before.len() < b.len()) {
                best[k] = Some(before);
            }
        }
    }
    best.pop().flatten()
}

/// Reads a unit such as "N/mm²", "kN·m", "W/(m²·K)" or "mm^-1".
pub fn parse_unit(text: &str) -> Result<ParsedUnit, String> {
    let text = text.trim();
    let mut parser = UnitParser {
        chars: text.chars().collect(),
        at: 0,
    };
    let (factor, dimension, symbols) = parser
        .product()
        .filter(|_| parser.at == parser.chars.len())
        .ok_or_else(|| format!("Unknown unit: {text}"))?;
    let zero = match symbols.as_slice() {
        ["°C" | "degC"] => Some(273.15),
        ["°F" | "degF"] => Some(459.67 * 5.0 / 9.0),
        _ => None,
    };
    Ok(ParsedUnit {
        factor,
        dimension,
        zero,
    })
}

struct UnitParser {
    chars: Vec<char>,
    at: usize,
}

type Term = (f64, Dimension, Vec<&'static str>);

impl UnitParser {
    fn peek(&self) -> Option<char> {
        self.chars.get(self.at).copied()
    }

    fn skip_blanks(&mut self) {
        while self.peek().is_some_and(char::is_whitespace) {
            self.at += 1;
        }
    }

    /// Factors joined by `*`, `·`, blanks or `/`.
    fn product(&mut self) -> Option<Term> {
        self.skip_blanks();
        let (mut factor, mut dimension, mut symbols) = self.power()?;
        loop {
            self.skip_blanks();
            let divide = match self.peek() {
                Some('/') => true,
                Some('*' | '·' | '×') => false,
                Some(c) if is_unit_char(c) || c == '(' || c == '1' => {
                    // Juxtaposed: "N m".
                    let (f, d, s) = self.power()?;
                    factor *= f;
                    dimension = dimension.times(d);
                    symbols.extend(s);
                    continue;
                }
                _ => return Some((factor, dimension, symbols)),
            };
            self.at += 1;
            self.skip_blanks();
            let (f, d, s) = self.power()?;
            if divide {
                factor /= f;
                dimension = dimension.times(d.powi(-1));
            } else {
                factor *= f;
                dimension = dimension.times(d);
            }
            symbols.extend(s);
        }
    }

    /// A unit or a bracketed product with an optional exponent: `mm²`, `mm2`, `s^-1`.
    fn power(&mut self) -> Option<Term> {
        let (factor, dimension, symbols) = self.atom()?;
        let exponent = self.exponent()?;
        Some((
            factor.powi(exponent.into()),
            dimension.powi(exponent),
            symbols,
        ))
    }

    fn atom(&mut self) -> Option<Term> {
        match self.peek()? {
            '(' => {
                self.at += 1;
                let term = self.product()?;
                self.skip_blanks();
                (self.peek()? == ')').then(|| self.at += 1)?;
                Some(term)
            }
            // "1/s"
            '1' => {
                self.at += 1;
                Some((1.0, Dimension::default(), Vec::new()))
            }
            c if is_unit_char(c) => {
                let start = self.at;
                while self.peek().is_some_and(is_unit_char) {
                    self.at += 1;
                }
                let text: String = self.chars[start..self.at].iter().collect();
                let units = word(&text)?;
                let factor = units.iter().map(|u| u.0).product();
                let dimension = (units.iter()).fold(Dimension::default(), |d, u| d.times(u.1));
                Some((factor, dimension, units.iter().map(|u| u.2).collect()))
            }
            _ => None,
        }
    }

    /// The exponent after a unit, 1 if there is none.
    fn exponent(&mut self) -> Option<i8> {
        match self.peek() {
            Some('²') => {
                self.at += 1;
                Some(2)
            }
            Some('³') => {
                self.at += 1;
                Some(3)
            }
            Some('^') => {
                self.at += 1;
                let negative = self.peek() == Some('-');
                if negative {
                    self.at += 1;
                }
                let n = self.digits()?;
                Some(if negative { -n } else { n })
            }
            Some(c) if c.is_ascii_digit() => self.digits(),
            _ => Some(1),
        }
    }

    fn digits(&mut self) -> Option<i8> {
        let start = self.at;
        while self.peek().is_some_and(|c| c.is_ascii_digit()) {
            self.at += 1;
        }
        let text: String = self.chars[start..self.at].iter().collect();
        text.parse().ok().filter(|&n: &i8| n.abs() <= 9)
    }
}

fn is_unit_char(c: char) -> bool {
    c.is_alphabetic() || c == '°'
}

#[cfg(test)]
mod tests {
    use super::*;
    use Quantity::*;
    use UnitSystem::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1e-300)
    }

    #[track_caller]
    fn parses(units: UnitSystem, text: &str, quantity: Quantity, expected: f64) {
        let value = units.parse_value(text, quantity).unwrap();
        assert!(close(value, expected), "{text}: {value} != {expected}");
    }

    #[test]
    fn evaluates_arithmetic() {
        assert_eq!(evaluate("1/30"), Some(1.0 / 30.0));
        assert_eq!(evaluate("2*(3,5-1)"), Some(5.0));
        assert_eq!(evaluate(" 1 + 2 * 3 "), Some(7.0));
        assert_eq!(evaluate("(1+2)*3"), Some(9.0));
        assert_eq!(evaluate("8/4/2"), Some(1.0));
        assert_eq!(evaluate("10-4-3"), Some(3.0));
        assert_eq!(evaluate("-2*-3"), Some(6.0));
        assert_eq!(evaluate("-(1,5+0.5)"), Some(-2.0));
        assert_eq!(evaluate("\u{2212}3 × 2 ÷ 4"), Some(-1.5));
        assert_eq!(evaluate("2e5/2"), Some(1e5));
        assert_eq!(evaluate("2,1E-3*1e+3"), Some(2.1));
        // A plain number is its own result, unchanged.
        assert_eq!(evaluate("7,85e-9"), Some(7.85e-9));
        assert_eq!(evaluate("0.1"), Some(0.1));
        for bad in [
            "", "1/0", "0/0", "1+", "*2", "(1+2", "1+2)", "()", "1 2", "2(3)", "abc", "1e999",
            "1.000,5", "e5",
        ] {
            assert_eq!(evaluate(bad), None, "{bad:?}");
        }
    }

    #[test]
    fn units_apply_to_a_calculation() {
        let value = MmTonSC.parse_value("100/4 cm", Length).unwrap();
        assert!(close(value, 250.0), "{value}");
        assert_eq!(MmTonSC.parse_value("1/30", Length), Ok(1.0 / 30.0));
        assert_eq!(MmTonSC.parse_value("(2+3)mm", Length), Ok(5.0));
        let value = MKgSC.parse_value("2e5/2Pa", Pressure).unwrap();
        assert!(close(value, 1e5), "{value}");
        assert!(MmTonSC.parse_value("1/0 mm", Length).is_err());
        assert!(MmTonSC.parse_value("2 mm + 3", Length).is_err());
    }

    #[test]
    fn lengths_in_other_units_are_converted() {
        parses(MmTonSC, "3cm", Length, 30.0);
        parses(MmTonSC, "3 cm", Length, 30.0);
        parses(MmTonSC, "30", Length, 30.0);
        parses(MmTonSC, "2,5 m", Length, 2500.0);
        parses(MmTonSC, "1in", Length, 25.4);
        parses(MKgSC, "3cm", Length, 0.03);
        parses(InLbSF, "25.4mm", Length, 1.0);
        parses(MmTonSC, "-1e-3 m", Length, -1.0);
        parses(MmTonSC, "5 µm", Length, 0.005);
    }

    #[test]
    fn derived_units_are_converted() {
        parses(MmTonSC, "210 GPa", Pressure, 210_000.0);
        parses(MmTonSC, "2,1e5MPa", Pressure, 2.1e5);
        parses(MKgSC, "1 MPa", Pressure, 1e6);
        parses(MTonSC, "1 MPa", Pressure, 1e3);
        parses(MmTonSC, "1 N/mm²", Pressure, 1.0);
        parses(MmTonSC, "1 N/mm2", Pressure, 1.0);
        parses(MmTonSC, "1 bar", Pressure, 0.1);
        parses(MmTonSC, "7850 kg/m³", Density, 7.85e-9);
        parses(MmTonSC, "7,85 g/cm^3", Density, 7.85e-9);
        parses(MmTonSC, "1 kN", Force, 1000.0);
        parses(MTonSC, "1 kN", Force, 1.0);
        parses(InLbSF, "1 lbf", Force, 1.0);
        parses(MmTonSC, "1 Nm", Moment, 1000.0);
        parses(MmTonSC, "1 kNm", Moment, 1e6);
        parses(MmTonSC, "1 N·m", Moment, 1000.0);
        parses(MmTonSC, "1 N*m", Moment, 1000.0);
        parses(MmTonSC, "1 kN/m", ForcePerLength, 1.0);
        parses(MmTonSC, "1 N/m³", ForcePerVolume, 1e-9);
        parses(MmTonSC, "1 W/(m²·K)", HeatTransferCoefficient, 1e-3);
        parses(MmTonSC, "1 kHz", Frequency, 1000.0);
        parses(MmTonSC, "1 min", Time, 60.0);
        parses(MmTonSC, "2 ms", Time, 0.002);
        parses(
            MmTonSC,
            "60 rpm",
            RotationalSpeed,
            2.0 * std::f64::consts::PI,
        );
        parses(InLbSF, "1 psi", Pressure, 1.0);
    }

    #[test]
    fn angles_and_temperatures() {
        parses(MmTonSC, "180°", Angle, std::f64::consts::PI);
        parses(MmTonSC, "90 deg", Angle, std::f64::consts::FRAC_PI_2);
        parses(MmTonSC, "0.5 rad", Angle, 0.5);
        parses(MmTonSC, "20 °C", Temperature, 20.0);
        parses(MmTonSC, "300 K", Temperature, 300.0 - 273.15);
        parses(MmTonSC, "212 °F", Temperature, 100.0);
        parses(InLbSF, "100 °C", Temperature, 212.0);
    }

    const THERMAL_QUANTITIES: [Quantity; 7] = [
        TemperatureDifference,
        ThermalConductivity,
        SpecificHeat,
        ThermalExpansion,
        HeatFlux,
        PowerPerVolume,
        StefanBoltzmann,
    ];

    #[test]
    fn thermal_units_are_converted() {
        parses(MmTonSC, "50 W/(m·K)", ThermalConductivity, 50.0);
        parses(MmTonSC, "460 J/(kg·K)", SpecificHeat, 4.6e8);
        parses(MmTonSC, "1.2e-5 1/K", ThermalExpansion, 1.2e-5);
        parses(MmTonSC, "1.2e-5 1/°C", ThermalExpansion, 1.2e-5);
        parses(InLbSF, "1.8 1/°C", ThermalExpansion, 1.0);
        parses(MmTonSC, "1 W/m²", HeatFlux, 1e-3);
        parses(MmTonSC, "1 W/m³", PowerPerVolume, 1e-6);
        parses(MmTonSC, "5.67e-8 W/(m²·K^4)", StefanBoltzmann, 5.67e-11);
        assert!(close(
            MKgSC.convert(1.0, ThermalExpansion, InLbSF),
            5.0 / 9.0
        ));
        parses(MmTonSC, "5 K", TemperatureDifference, 5.0);
        assert!(close(
            MKgSC.convert(5.0, TemperatureDifference, InLbSF),
            9.0
        ));
    }

    #[test]
    fn wrong_or_unknown_units_are_refused() {
        assert!(MmTonSC.parse_value("3 kg", Length).is_err());
        assert!(MmTonSC.parse_value("3 xyz", Length).is_err());
        assert!(MmTonSC.parse_value("3 Hz", RotationalSpeed).is_err());
        assert!(MmTonSC.parse_value("cm", Length).is_err());
        assert!(MmTonSC.parse_value("", Length).is_err());
        // Without a unit system only plain numbers are taken.
        assert!(Unitless.parse_value("3 cm", Length).is_err());
        assert_eq!(Unitless.parse_value("3", Length), Ok(3.0));
    }

    #[test]
    fn units_with_prefixes_and_products_are_told_apart() {
        // "mN" is a millinewton, "Nm" a newton metre, "min" a minute, "nm" a nanometre.
        assert_eq!(parse_unit("mN").unwrap().dimension, Force.dimension());
        assert_eq!(parse_unit("Nm").unwrap().dimension, Moment.dimension());
        assert_eq!(parse_unit("min").unwrap().factor, 60.0);
        assert!(close(parse_unit("nm").unwrap().factor, 1e-9));
        assert!(close(parse_unit("Nmm").unwrap().factor, 1e-3));
    }

    #[test]
    fn systems_convert_into_each_other() {
        assert!(close(MmTonSC.convert(1.0, Length, MKgSC), 1e-3));
        assert!(close(MmTonSC.convert(210_000.0, Pressure, MKgSC), 2.1e11));
        assert!(close(MmTonSC.convert(7.85e-9, Density, MKgSC), 7850.0));
        assert!(close(MKgSC.convert(1.0, Force, MTonSC), 1e-3));
        assert!(close(MmTonSC.convert(1.0, Energy, MKgSC), 1e-3));
        assert!(close(MmTonSC.convert(25.4, Length, InLbSF), 1.0));
        assert!(close(MmTonSC.convert(100.0, Temperature, InLbSF), 212.0));
        assert!(close(
            InLbSF.convert(1.0, Pressure, MKgSC),
            6_894.757_293_168
        ));
        assert_eq!(MmTonSC.convert(5.0, Length, Unitless), 5.0);
        assert_eq!(MmTonSC.factor_to(Pressure, MKgSC), 1e6);
    }

    #[test]
    fn derived_units_match_the_base_units() {
        // The named unit of each quantity is the system's unit: one of it parses to 1.
        for units in [MKgSC, MmTonSC, MTonSC, InLbSF] {
            for quantity in BASE_QUANTITIES
                .into_iter()
                .chain(DERIVED_QUANTITIES)
                .chain([ForcePerVolume, HeatTransferCoefficient])
                .chain(THERMAL_QUANTITIES)
            {
                let unit = units.unit(quantity);
                let text = format!("1 {unit}");
                parses(units, &text, quantity, 1.0);
            }
        }
    }
}
