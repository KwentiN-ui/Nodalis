//! Materials (concept section 1.2): elastic, plastic, creep, thermal expansion, conduction,
//! specific heat, density. Tables depend on temperature where CalculiX allows it: a row per
//! temperature, interpolated linearly, constant beyond the ends.

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Material {
    /// Upper case name.
    pub name: String,
    pub elastic: Option<Elastic>,
    /// Hyperelasticity (`*HYPERELASTIC`); replaces `elastic` where both are given.
    #[serde(default)]
    pub hyperelastic: Option<Hyperelastic>,
    /// Anisotropic linear elasticity (`*ELASTIC, TYPE=ENGINEERING CONSTANTS`, `ORTHO` or
    /// `ANISO`); replaces `elastic`, the two are never both given.
    #[serde(default)]
    pub anisotropic: Option<AnisotropicElastic>,
    pub plastic: Option<Plastic>,
    pub creep: Option<Creep>,
    pub expansion: Option<Expansion>,
    pub conductivity: Option<Conductivity>,
    pub specific_heat: Option<SpecificHeat>,
    pub density: Option<Density>,
}

/// Isotropic linear elasticity (`*ELASTIC`, `TYPE=ISO`), rows of (E, nu, temperature).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Elastic {
    pub rows: Vec<ElasticRow>,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ElasticRow {
    pub young: f64,
    pub poisson: f64,
    pub temperature: f64,
}

/// Anisotropic linear elasticity of one temperature, in the axes of the orientation of the
/// section (the global axes without one). The constants follow CalculiX's data lines, with
/// the indices 1, 2, 3 of the material axes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum AnisotropicElastic {
    /// `TYPE=ENGINEERING CONSTANTS`: `E1, E2, E3, ν12, ν13, ν23, G12, G13, G23`.
    EngineeringConstants([f64; 9]),
    /// `TYPE=ORTHO`: `D1111, D1122, D2222, D1133, D2233, D3333, D1212, D1313, D2323`.
    Orthotropic([f64; 9]),
    /// `TYPE=ANISO`: `D1111, D1122, D2222, D1133, D2233, D3333, D1112, D2212, D3312, D1212,
    /// D1113, D2213, D3313, D1213, D1313, D1123, D2223, D3323, D1223, D1323, D2323`.
    Anisotropic([f64; 21]),
}

/// An isotropic hyperelastic material (`*HYPERELASTIC`): the strain energy model and its
/// constants in the order of CalculiX's data lines, without the temperature.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Hyperelastic {
    pub model: HyperelasticModel,
    pub constants: Vec<f64>,
}

/// The strain energy potentials of `*HYPERELASTIC`. With the isochoric invariants `Ī1`,
/// `Ī2`, the isochoric principal stretches `λ̄` and the volume ratio `J`:
///
/// - polynomial: `W = Σ C_ij (Ī1 - 3)^i (Ī2 - 3)^j + Σ (J - 1)^(2k) / D_k`, of which
///   Neo-Hooke (`C10`), Mooney-Rivlin (`C10`, `C01`), the reduced polynomial (`j = 0`) and
///   Yeoh (the reduced polynomial of order 3) are special cases;
/// - Ogden: `W = Σ 2 μ_p / α_p² (λ̄1^α_p + λ̄2^α_p + λ̄3^α_p - 3) + Σ (J - 1)^(2k) / D_k`;
/// - Arruda-Boyce: the first five terms of the inverse Langevin series in `Ī1` with the
///   initial shear modulus parameter `μ` and the locking stretch `λm`, plus
///   `((J² - 1) / 2 - ln J) / D`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum HyperelasticModel {
    /// `C10, D1`.
    NeoHooke,
    /// `C10, C01, D1`.
    MooneyRivlin,
    /// `C10, C01, C20, C11, C02, C30, C21, C12, C03` up to order `N`, then `D1 .. DN`.
    Polynomial(u8),
    /// `C10, C20, C30` up to order `N`, then `D1 .. DN`.
    ReducedPolynomial(u8),
    /// `C10, C20, C30, D1, D2, D3`.
    Yeoh,
    /// `μ1, α1, .. μN, αN`, then `D1 .. DN`.
    Ogden(u8),
    /// `μ, λm, D`.
    ArrudaBoyce,
}

impl HyperelasticModel {
    /// Number of constants on the data lines (without the temperature).
    pub fn constant_count(self) -> usize {
        match self {
            Self::NeoHooke => 2,
            Self::MooneyRivlin => 3,
            Self::Polynomial(n) => {
                let n = n as usize;
                n * (n + 3) / 2 + n
            }
            Self::ReducedPolynomial(n) => 2 * n as usize,
            Self::Yeoh => 6,
            Self::Ogden(n) => 3 * n as usize,
            Self::ArrudaBoyce => 3,
        }
    }
}

/// Rate-independent plasticity (`*PLASTIC`) with a stress / plastic strain table per
/// temperature.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Plastic {
    pub hardening: Hardening,
    pub rows: Vec<PlasticRow>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Hardening {
    Isotropic,
    Kinematic,
    Combined,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlasticRow {
    pub stress: f64,
    pub plastic_strain: f64,
    pub temperature: f64,
}

/// Norton creep (`*CREEP, LAW=NORTON`): rate = A * sigma^n * t^m, rows per temperature.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Creep {
    pub rows: Vec<CreepRow>,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CreepRow {
    pub a: f64,
    pub n: f64,
    pub m: f64,
    pub temperature: f64,
}

/// Isotropic thermal expansion (`*EXPANSION`), coefficient per temperature, referred to
/// `zero` (the temperature at which the expansion is zero).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Expansion {
    pub zero: f64,
    pub rows: Vec<TemperatureValue>,
}

/// Isotropic conductivity (`*CONDUCTIVITY`) per temperature.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Conductivity {
    pub rows: Vec<TemperatureValue>,
}

/// `*SPECIFIC HEAT` per temperature.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpecificHeat {
    pub rows: Vec<TemperatureValue>,
}

/// `*DENSITY` per temperature.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Density {
    pub rows: Vec<TemperatureValue>,
}

/// A value at a temperature; a table with a single row at temperature 0 is constant.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TemperatureValue {
    pub value: f64,
    pub temperature: f64,
}

impl TemperatureValue {
    pub const fn constant(value: f64) -> Self {
        Self {
            value,
            temperature: 0.0,
        }
    }
}

/// Linear interpolation in a temperature table, constant beyond its ends (as CalculiX does).
pub fn interpolate(rows: &[TemperatureValue], temperature: f64) -> Option<f64> {
    let (first, last) = (rows.first()?, rows.last()?);
    if rows.len() == 1 || temperature <= first.temperature {
        return Some(first.value);
    }
    if temperature >= last.temperature {
        return Some(last.value);
    }
    let upper = rows.iter().position(|r| r.temperature > temperature)?;
    let (a, b) = (rows[upper - 1], rows[upper]);
    let t = (temperature - a.temperature) / (b.temperature - a.temperature);
    Some(a.value + t * (b.value - a.value))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tables_interpolate_and_hold_their_ends() {
        let rows = [
            TemperatureValue {
                value: 10.0,
                temperature: 0.0,
            },
            TemperatureValue {
                value: 20.0,
                temperature: 100.0,
            },
        ];
        assert_eq!(interpolate(&rows, -5.0), Some(10.0));
        assert_eq!(interpolate(&rows, 50.0), Some(15.0));
        assert_eq!(interpolate(&rows, 500.0), Some(20.0));
        assert_eq!(interpolate(&rows[..1], 500.0), Some(10.0));
        assert_eq!(interpolate(&[], 0.0), None);
    }
}
