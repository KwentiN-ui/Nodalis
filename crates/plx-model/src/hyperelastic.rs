//! Hyperelastic materials: rubber-like materials whose stresses follow a strain energy model
//! at large strains. The user picks a model and its constants (or fits them to test data);
//! the compressibility is given as Poisson's ratio for small strains, which the lowering
//! turns into the solver's `D1`.
//!
//! The constants mean what they mean in the CalculiX and Abaqus manuals, so published
//! values carry over: with the isochoric invariants `Ī1`, `Ī2` and principal stretches `λ̄`
//!
//! - Neo-Hooke `W = C10 (Ī1 - 3)`,
//! - Mooney-Rivlin `W = C10 (Ī1 - 3) + C01 (Ī2 - 3)`,
//! - Yeoh `W = C10 (Ī1 - 3) + C20 (Ī1 - 3)² + C30 (Ī1 - 3)³`,
//! - polynomial of order 2 `W = Σ_(i+j≤2) Cij (Ī1 - 3)^i (Ī2 - 3)^j`,
//! - Ogden `W = Σ 2 μ_p / α_p² (λ̄1^α_p + λ̄2^α_p + λ̄3^α_p - 3)`,
//! - Arruda-Boyce with the shear modulus parameter `μ` and the locking stretch `λm`.

use serde::{Deserialize, Serialize};

/// The strain energy models offered for hyperelastic materials.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum HyperelasticModel {
    #[default]
    NeoHooke,
    MooneyRivlin,
    Yeoh,
    /// The full polynomial of order 2.
    Polynomial2,
    /// The Ogden model with 1 to 3 terms.
    Ogden(u8),
    ArrudaBoyce,
}

/// What a constant of a model measures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParameterKind {
    /// A stress (shown and converted in the unit of pressure).
    Stress,
    /// A plain number (Ogden exponents, the locking stretch).
    Number,
}

/// A constant of a model, for the material form.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Parameter {
    pub name: &'static str,
    pub kind: ParameterKind,
}

const fn stress(name: &'static str) -> Parameter {
    Parameter {
        name,
        kind: ParameterKind::Stress,
    }
}

const fn number(name: &'static str) -> Parameter {
    Parameter {
        name,
        kind: ParameterKind::Number,
    }
}

/// The coefficients of the first five terms of the Arruda-Boyce series: with them
/// `W = μ Σ c_k / λm^(2k-2) (Ī1^k - 3^k)`.
pub const ARRUDA_BOYCE: [f64; 5] = [
    0.5,
    1.0 / 20.0,
    11.0 / 1050.0,
    19.0 / 7000.0,
    519.0 / 673_750.0,
];

impl HyperelasticModel {
    pub const ALL: [Self; 8] = [
        Self::NeoHooke,
        Self::MooneyRivlin,
        Self::Yeoh,
        Self::Polynomial2,
        Self::Ogden(1),
        Self::Ogden(2),
        Self::Ogden(3),
        Self::ArrudaBoyce,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::NeoHooke => "Neo-Hooke",
            Self::MooneyRivlin => "Mooney-Rivlin",
            Self::Yeoh => "Yeoh",
            Self::Polynomial2 => "Polynomial (order 2)",
            Self::Ogden(1) => "Ogden (1 term)",
            Self::Ogden(2) => "Ogden (2 terms)",
            Self::Ogden(_) => "Ogden (3 terms)",
            Self::ArrudaBoyce => "Arruda-Boyce",
        }
    }

    /// A short description for the material form.
    pub fn description(self) -> &'static str {
        match self {
            Self::NeoHooke => {
                "One constant, C10 = initial shear modulus / 2. Good up to some 50 % strain \
                 and when only the hardness or modulus is known."
            }
            Self::MooneyRivlin => {
                "Two constants. Fits moderate strains up to some 100 %; initial shear \
                 modulus = 2 (C10 + C01)."
            }
            Self::Yeoh => {
                "Three constants in the first invariant. Captures the stiffening at large \
                 strains and fits well to uniaxial tests alone."
            }
            Self::Polynomial2 => {
                "Five constants. Flexible, but needs test data in several modes (uniaxial, \
                 biaxial, planar) to stay stable."
            }
            Self::Ogden(_) => {
                "Pairs of a modulus mu and an exponent alpha. Very flexible up to large \
                 strains; initial shear modulus = sum of mu."
            }
            Self::ArrudaBoyce => {
                "Physically based on chain networks: the shear modulus parameter mu and the \
                 locking stretch at which the chains are fully extended."
            }
        }
    }

    /// The constants of the model in the order the material stores them.
    pub fn parameters(self) -> Vec<Parameter> {
        match self {
            Self::NeoHooke => vec![stress("C10")],
            Self::MooneyRivlin => vec![stress("C10"), stress("C01")],
            Self::Yeoh => vec![stress("C10"), stress("C20"), stress("C30")],
            Self::Polynomial2 => vec![
                stress("C10"),
                stress("C01"),
                stress("C20"),
                stress("C11"),
                stress("C02"),
            ],
            Self::Ogden(n) => {
                const NAMES: [(&str, &str); 3] =
                    [("mu1", "alpha1"), ("mu2", "alpha2"), ("mu3", "alpha3")];
                (NAMES.iter().take(n.clamp(1, 3) as usize))
                    .flat_map(|&(mu, alpha)| [stress(mu), number(alpha)])
                    .collect()
            }
            Self::ArrudaBoyce => vec![stress("mu"), number("Locking stretch")],
        }
    }

    /// Constants of the model with the initial shear modulus `mu`, close to a Neo-Hooke
    /// material: what a model starts with when the user switches to it.
    pub fn equivalent(self, mu: f64) -> Vec<f64> {
        match self {
            Self::NeoHooke => vec![mu / 2.0],
            Self::MooneyRivlin => vec![mu / 2.0, 0.0],
            Self::Yeoh => vec![mu / 2.0, 0.0, 0.0],
            Self::Polynomial2 => vec![mu / 2.0, 0.0, 0.0, 0.0, 0.0],
            Self::Ogden(n) => {
                let mut c = vec![mu, 2.0];
                let exponents = [-2.0, 4.0];
                for alpha in exponents.iter().take(n.clamp(1, 3) as usize - 1) {
                    c.extend([0.0, *alpha]);
                }
                c
            }
            Self::ArrudaBoyce => {
                let locking: f64 = 7.0;
                vec![mu / arruda_boyce_factor(locking), locking]
            }
        }
    }
}

/// `μ0 / μ` of the Arruda-Boyce model with the locking stretch `λm`.
fn arruda_boyce_factor(locking: f64) -> f64 {
    (ARRUDA_BOYCE.iter().enumerate())
        .map(|(k, c)| {
            let k = k as i32;
            2.0 * c * f64::from(k + 1) * 3.0_f64.powi(k) / locking.powi(2 * k)
        })
        .sum()
}

/// A hyperelastic material: the strain energy model, its constants and the compressibility.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Hyperelastic {
    pub model: HyperelasticModel,
    /// The constants of the model in the order of [`HyperelasticModel::parameters`];
    /// stresses in the unit of pressure of the model.
    pub constants: Vec<f64>,
    /// Poisson's ratio at small strains: how compressible the material is. Rubber is
    /// nearly incompressible (0.49 to 0.499).
    pub poisson: f64,
    /// Test data the constants were or can be fitted to.
    #[serde(default)]
    pub test_data: Vec<crate::hyperelastic_fit::TestPoint>,
}

impl Default for Hyperelastic {
    /// A soft rubber of 1 MPa initial shear modulus (stored in Pa).
    fn default() -> Self {
        Self::neo_hooke(1e6, 0.49)
    }
}

/// How a test specimen is deformed: the three homogeneous modes of rubber tests.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TestMode {
    /// Uniaxial tension or compression.
    #[default]
    Uniaxial,
    /// Equibiaxial tension.
    Biaxial,
    /// Planar tension (pure shear).
    Planar,
}

impl TestMode {
    pub const ALL: [Self; 3] = [Self::Uniaxial, Self::Biaxial, Self::Planar];

    pub fn label(self) -> &'static str {
        match self {
            Self::Uniaxial => "Uniaxial",
            Self::Biaxial => "Equibiaxial",
            Self::Planar => "Planar (pure shear)",
        }
    }
}

impl Hyperelastic {
    /// A Neo-Hooke material with the initial shear modulus `mu`.
    pub fn neo_hooke(mu: f64, poisson: f64) -> Self {
        Self {
            model: HyperelasticModel::NeoHooke,
            constants: vec![mu / 2.0],
            poisson,
            test_data: Vec::new(),
        }
    }

    /// The material switched to another model with the same initial moduli.
    pub fn with_model(&self, model: HyperelasticModel) -> Self {
        if model == self.model {
            return self.clone();
        }
        let mu = self.initial_shear_modulus();
        Self {
            model,
            constants: model.equivalent(if mu > 0.0 { mu } else { 1e6 }),
            poisson: self.poisson,
            test_data: self.test_data.clone(),
        }
    }

    /// The constant `i` (0 when missing).
    fn c(&self, i: usize) -> f64 {
        self.constants.get(i).copied().unwrap_or(0.0)
    }

    /// The initial shear modulus.
    pub fn initial_shear_modulus(&self) -> f64 {
        match self.model {
            HyperelasticModel::NeoHooke => 2.0 * self.c(0),
            HyperelasticModel::MooneyRivlin | HyperelasticModel::Polynomial2 => {
                2.0 * (self.c(0) + self.c(1))
            }
            HyperelasticModel::Yeoh => 2.0 * self.c(0),
            HyperelasticModel::Ogden(n) => (0..n.clamp(1, 3) as usize).map(|p| self.c(2 * p)).sum(),
            HyperelasticModel::ArrudaBoyce => self.c(0) * arruda_boyce_factor(self.c(1)),
        }
    }

    /// The initial bulk modulus from Poisson's ratio.
    pub fn initial_bulk_modulus(&self) -> f64 {
        let nu = self.poisson;
        2.0 * self.initial_shear_modulus() * (1.0 + nu) / (3.0 * (1.0 - 2.0 * nu))
    }

    /// The solver's compressibility `D1 = 2 / K0`.
    pub fn d1(&self) -> f64 {
        2.0 / self.initial_bulk_modulus()
    }

    /// Linear elasticity with the initial moduli (what small strains see).
    pub fn small_strain(&self) -> crate::Elastic {
        crate::Elastic {
            young: 2.0 * self.initial_shear_modulus() * (1.0 + self.poisson),
            poisson: self.poisson,
        }
    }

    /// The constants as the solver takes them: the model's constants, then `D1` and zeros
    /// for the higher volumetric terms.
    pub fn solver_constants(&self) -> Vec<f64> {
        let mut c: Vec<f64> = (0..self.model.parameters().len())
            .map(|i| self.c(i))
            .collect();
        let d_terms = match self.model {
            HyperelasticModel::Yeoh => 3,
            HyperelasticModel::Polynomial2 => 2,
            HyperelasticModel::Ogden(n) => n.clamp(1, 3) as usize,
            _ => 1,
        };
        c.push(self.d1());
        c.extend(std::iter::repeat_n(0.0, d_terms - 1));
        c
    }

    /// Why the solver would refuse the material, if it would.
    pub fn problem(&self) -> Option<String> {
        if self.constants.len() != self.model.parameters().len() {
            return Some(format!(
                "{} needs {} constants",
                self.model.label(),
                self.model.parameters().len()
            ));
        }
        if self.constants.iter().any(|c| !c.is_finite()) {
            return Some("A constant is not a number.".into());
        }
        let mu = self.initial_shear_modulus();
        if mu.is_nan() || mu <= 0.0 {
            return Some("The initial shear modulus must be positive.".into());
        }
        if !(self.poisson > -1.0 && self.poisson < 0.5) {
            return Some(
                "Poisson's ratio must be below 0.5: Nodalis has no hybrid elements for \
                 fully incompressible materials."
                    .into(),
            );
        }
        match self.model {
            HyperelasticModel::Ogden(n)
                if (0..n.clamp(1, 3) as usize).any(|p| self.c(2 * p + 1) == 0.0) =>
            {
                Some("An Ogden exponent alpha is zero.".into())
            }
            HyperelasticModel::ArrudaBoyce if self.c(1) <= 1.0 => {
                Some("The locking stretch must be greater than 1.".into())
            }
            _ => None,
        }
    }

    /// The nominal stress (force per undeformed area) of an incompressible specimen
    /// stretched by `stretch` in the test mode: from the principal stretches
    /// `(λ, λ2, λ3)` and the Kirchhoff stresses with the lateral stress zero.
    pub fn nominal_stress(&self, mode: TestMode, stretch: f64) -> f64 {
        let l = stretch;
        let stretches = match mode {
            TestMode::Uniaxial => [l, 1.0 / l.sqrt(), 1.0 / l.sqrt()],
            TestMode::Biaxial => [l, l, 1.0 / (l * l)],
            TestMode::Planar => [l, 1.0, 1.0 / l],
        };
        // The deviatoric Kirchhoff stresses τ_a = λ_a ∂W/∂λ_a minus the free stress τ_3.
        let tau = self.principal_kirchhoff(stretches);
        (tau[0] - tau[2]) / l
    }

    /// The (isochoric) Kirchhoff principal stresses `λ_a ∂W/∂λ_a` of an incompressible
    /// deformation, up to a common pressure.
    fn principal_kirchhoff(&self, l: [f64; 3]) -> [f64; 3] {
        match self.model {
            HyperelasticModel::Ogden(n) => {
                let mut tau = [0.0; 3];
                for p in 0..n.clamp(1, 3) as usize {
                    let (mu, alpha) = (self.c(2 * p), self.c(2 * p + 1));
                    if alpha == 0.0 {
                        continue;
                    }
                    for (t, s) in tau.iter_mut().zip(l) {
                        *t += 2.0 * mu / alpha * s.powf(alpha);
                    }
                }
                tau
            }
            _ => {
                let squares = l.map(|s| s * s);
                let i1 = squares.iter().sum::<f64>();
                let i2 =
                    squares[0] * squares[1] + squares[1] * squares[2] + squares[2] * squares[0];
                let (w1, w2) = self.invariant_derivatives(i1, i2);
                // λ_a ∂W/∂λ_a = 2 λ_a² (W1 + W2 (I1 - λ_a²)).
                squares.map(|s| 2.0 * s * (w1 + w2 * (i1 - s)))
            }
        }
    }

    /// `∂W/∂I1` and `∂W/∂I2` of an invariant based model.
    fn invariant_derivatives(&self, i1: f64, i2: f64) -> (f64, f64) {
        let (a, b) = (i1 - 3.0, i2 - 3.0);
        match self.model {
            HyperelasticModel::NeoHooke => (self.c(0), 0.0),
            HyperelasticModel::MooneyRivlin => (self.c(0), self.c(1)),
            HyperelasticModel::Yeoh => (
                self.c(0) + 2.0 * self.c(1) * a + 3.0 * self.c(2) * a * a,
                0.0,
            ),
            HyperelasticModel::Polynomial2 => (
                self.c(0) + 2.0 * self.c(2) * a + self.c(3) * b,
                self.c(1) + self.c(3) * a + 2.0 * self.c(4) * b,
            ),
            HyperelasticModel::ArrudaBoyce => {
                let (mu, locking) = (self.c(0), self.c(1));
                let w1 = (ARRUDA_BOYCE.iter().enumerate())
                    .map(|(k, c)| {
                        let k = k as i32;
                        mu * c * f64::from(k + 1) * i1.powi(k) / locking.powi(2 * k)
                    })
                    .sum();
                (w1, 0.0)
            }
            HyperelasticModel::Ogden(_) => (0.0, 0.0),
        }
    }

    /// The stretches up to which the material is stable (Drucker) in all three test modes,
    /// checked in tension and compression between the stretches 0.1 and 10: the first
    /// stretch at which the nominal stress stops growing, if there is one.
    pub fn unstable_stretch(&self) -> Option<(TestMode, f64)> {
        for mode in TestMode::ALL {
            let mut previous: Option<(f64, f64)> = None;
            // Logarithmically spaced stretches from 0.1 to 10.
            for i in 0..=400 {
                let stretch = 10f64.powf(-1.0 + 2.0 * f64::from(i) / 400.0);
                // Biaxial compression below 0.3 compresses the thickness absurdly.
                if mode == TestMode::Biaxial && stretch < 0.3 {
                    continue;
                }
                let stress = self.nominal_stress(mode, stretch);
                if let Some((s0, p0)) = previous
                    && stress <= p0
                {
                    // Report the stretch on the side away from 1.
                    let at = if stretch > 1.0 { stretch } else { s0 };
                    return Some((mode, at));
                }
                previous = Some((stretch, stress));
            }
        }
        None
    }

    /// Young's modulus of a rubber from its Shore A hardness after Gent (1958), in MPa.
    pub fn shore_a_young_mpa(hardness: f64) -> f64 {
        0.0981 * (56.0 + 7.62336 * hardness) / (0.137505 * (254.0 - 2.54 * hardness))
    }
}

impl crate::FeModel {
    /// Whether a section of the model has a hyperelastic material.
    pub fn has_hyperelastic(&self) -> bool {
        (self.sections.iter()).any(|s| {
            (s.material.and_then(|m| self.material(m))).is_some_and(|m| m.hyperelastic.is_some())
        })
    }

    /// Whether the step is computed with large deformations: its own setting, or switched
    /// on because hyperelastic materials deform in it (static, visco and dynamic steps; the
    /// solver does the same).
    pub fn step_nlgeom(&self, kind: &crate::StepKind) -> bool {
        kind.nlgeom() || (switched_on_by_hyperelasticity(kind) && self.has_hyperelastic())
    }
}

/// Whether a step takes NLGEOM for hyperelastic materials.
pub fn switched_on_by_hyperelasticity(kind: &crate::StepKind) -> bool {
    match kind {
        crate::StepKind::Static(_) | crate::StepKind::Visco(_) => true,
        // Modal superposition stays linear.
        crate::StepKind::Dynamic(settings) => {
            matches!(settings.method, crate::DynamicMethod::Direct)
        }
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn neo_hooke_uniaxial_tension_follows_the_textbook() {
        let neo = Hyperelastic::neo_hooke(1.0, 0.49);
        for stretch in [0.5, 1.5, 3.0] {
            let expected = stretch - 1.0 / (stretch * stretch);
            assert!((neo.nominal_stress(TestMode::Uniaxial, stretch) - expected).abs() < 1e-12);
        }
        assert!(neo.unstable_stretch().is_none());
    }

    #[test]
    fn switching_the_model_keeps_the_initial_moduli_and_the_curve() {
        let neo = Hyperelastic::neo_hooke(2.0, 0.49);
        for model in HyperelasticModel::ALL {
            let other = neo.with_model(model);
            assert_eq!(other.constants.len(), model.parameters().len(), "{model:?}");
            assert!(
                (other.initial_shear_modulus() - 2.0).abs() < 1e-9,
                "{model:?}"
            );
            assert!(other.problem().is_none(), "{model:?}");
            // All but Arruda-Boyce are exactly Neo-Hooke.
            if model != HyperelasticModel::ArrudaBoyce {
                for mode in TestMode::ALL {
                    let (a, b) = (
                        neo.nominal_stress(mode, 1.7),
                        other.nominal_stress(mode, 1.7),
                    );
                    assert!((a - b).abs() < 1e-9, "{model:?} {mode:?}: {a} vs {b}");
                }
            }
        }
    }

    #[test]
    fn the_solver_constants_end_with_the_compressibility() {
        let neo = Hyperelastic::neo_hooke(1.0, 0.49);
        let c = neo.solver_constants();
        let bulk = 2.0 * 1.49 / (3.0 * 0.02);
        assert_eq!(c.len(), 2);
        assert!((c[1] - 2.0 / bulk).abs() < 1e-12);
        let yeoh = neo.with_model(HyperelasticModel::Yeoh);
        assert_eq!(yeoh.solver_constants().len(), 6);
        let ogden = neo.with_model(HyperelasticModel::Ogden(3));
        assert_eq!(ogden.solver_constants().len(), 9);
    }

    #[test]
    fn an_unstable_mooney_rivlin_material_is_found() {
        let unstable = Hyperelastic {
            model: HyperelasticModel::MooneyRivlin,
            constants: vec![-0.2, 0.7],
            poisson: 0.49,
            test_data: Vec::new(),
        };
        assert!(unstable.unstable_stretch().is_some());
    }

    #[test]
    fn shore_a_60_is_a_few_megapascal() {
        let e = Hyperelastic::shore_a_young_mpa(60.0);
        assert!((3.0..6.0).contains(&e), "{e}");
    }
}
