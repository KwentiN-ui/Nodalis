//! Material laws of the Nodalis solver: isotropic linear elasticity (Hooke), rate-independent
//! J2 plasticity with isotropic, kinematic and combined hardening (`*PLASTIC`), and Norton
//! creep (`*CREEP`) as the viscous part of the same law (concept section 4.2).
//!
//! Sources: any textbook on elasticity (e.g. Timoshenko & Goodier) for Hooke's law in the
//! engineering-strain convention; Simo & Hughes, *Computational Inelasticity* (1998),
//! chapter 3 (radial return, consistent tangent, Prager kinematic hardening) and chapter 2.7
//! (viscoplastic regularisation of Perzyna type) for the plastic and creep algorithms. The
//! CalculiX manual describes which quantities its tables mean (von Mises stress against
//! equivalent plastic strain; Norton's law on the von Mises stress with the total time).
//! No CalculiX code was consulted.
//!
//! Conventions: strains are engineering strains `[xx, yy, zz, xy, yz, zx]` (γ for the shear
//! components), stresses `[xx, yy, zz, xy, yz, zx]`; tangents act on engineering strains.

pub mod finite;
pub mod hyper;
pub mod plane_stress;
pub mod plastic;

use plxs_core::material::{Material, interpolate};
use plxs_core::{Error, Result};

pub use finite::FiniteStrainJ2;
pub use hyper::Hyperelastic;
pub use plane_stress::PlaneStress;
pub use plastic::{J2Plastic, STATE_SIZE};

/// The 6 × 6 matrix of isotropic linear elasticity acting on engineering strains
/// `[xx, yy, zz, xy, yz, zx]` (γ for the shear components).
pub fn isotropic_matrix(young: f64, poisson: f64) -> [[f64; 6]; 6] {
    let lambda = young * poisson / ((1.0 + poisson) * (1.0 - 2.0 * poisson));
    let mu = young / (2.0 * (1.0 + poisson));
    let mut d = [[0.0; 6]; 6];
    for i in 0..3 {
        d[i][..3].fill(lambda);
        d[i][i] = lambda + 2.0 * mu;
        d[i + 3][i + 3] = mu;
    }
    d
}

/// Isotropic linear elasticity of a material at a temperature.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearElastic {
    pub young: f64,
    pub poisson: f64,
}

impl LinearElastic {
    /// The elastic constants of a material at `temperature`, interpolated in its
    /// `*ELASTIC` table; for a hyperelastic material the initial moduli of its strain
    /// energy (what linear steps and small vibrations see).
    pub fn of(material: &Material, temperature: f64) -> Result<Self> {
        if let Some(hyper) = Hyperelastic::of(material)? {
            return Ok(hyper.initial());
        }
        let elastic = material.elastic.as_ref().ok_or_else(|| {
            Error::input(
                "*MATERIAL",
                format!("material {} has no *ELASTIC definition", material.name),
            )
        })?;
        let young: Vec<_> = elastic
            .rows
            .iter()
            .map(|r| plxs_core::material::TemperatureValue {
                value: r.young,
                temperature: r.temperature,
            })
            .collect();
        let poisson: Vec<_> = elastic
            .rows
            .iter()
            .map(|r| plxs_core::material::TemperatureValue {
                value: r.poisson,
                temperature: r.temperature,
            })
            .collect();
        let (young, poisson) = match (
            interpolate(&young, temperature),
            interpolate(&poisson, temperature),
        ) {
            (Some(e), Some(nu)) => (e, nu),
            _ => {
                return Err(Error::input(
                    "*ELASTIC",
                    format!("material {} has an empty *ELASTIC table", material.name),
                ));
            }
        };
        Ok(Self { young, poisson })
    }

    pub fn matrix(&self) -> [[f64; 6]; 6] {
        isotropic_matrix(self.young, self.poisson)
    }

    pub fn shear_modulus(&self) -> f64 {
        self.young / (2.0 * (1.0 + self.poisson))
    }

    pub fn bulk_modulus(&self) -> f64 {
        self.young / (3.0 * (1.0 - 2.0 * self.poisson))
    }
}

/// Density of a material at a temperature, if it has one.
pub fn density(material: &Material, temperature: f64) -> Option<f64> {
    material
        .density
        .as_ref()
        .and_then(|d| interpolate(&d.rows, temperature))
}

/// What a material law is given at an integration point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MaterialInput {
    /// Total engineering strain at the end of the increment (the small strain, or the
    /// Green-Lagrange strain under NLGEOM).
    pub strain: [f64; 6],
    pub temperature: f64,
    /// Size of the time increment.
    pub dt: f64,
    /// Total time at the end of the increment (Norton's `t^m`).
    pub time: f64,
    /// Whether creep acts (`*VISCO`); `*STATIC` switches it off.
    pub creep_active: bool,
}

/// What a material law returns.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MaterialOutput {
    pub stress: [f64; 6],
    /// Consistent tangent acting on engineering strains.
    pub tangent: [[f64; 6]; 6],
}

/// A material law: stress and consistent tangent for a total strain, integrating its state
/// variables from the start of the increment (`state` holds the values at the start on
/// entry and at the end on return).
pub trait MaterialLaw: Send + Sync {
    /// Number of state variables per integration point.
    fn state_size(&self) -> usize;
    /// Whether the law is nonlinear (plastic, creeping or hyperelastic); linear laws never
    /// iterate.
    fn is_nonlinear(&self) -> bool;
    fn stress(&self, input: &MaterialInput, state: &mut [f64]) -> MaterialOutput;
    /// [`MaterialLaw::stress`] together with the strain the stress belongs to: the input
    /// strain, except under plane stress ([`PlaneStress`]), where the law finds the
    /// thickness strain itself.
    fn stress_and_strain(
        &self,
        input: &MaterialInput,
        state: &mut [f64],
    ) -> (MaterialOutput, [f64; 6]) {
        (self.stress(input, state), input.strain)
    }
    /// [`MaterialLaw::stress_and_strain`] without the tangent: the stress and its strain,
    /// for an explicit integration, which never needs a tangent. Laws whose tangent costs
    /// more than their stress give a cheaper version.
    fn stress_without_tangent(
        &self,
        input: &MaterialInput,
        state: &mut [f64],
    ) -> ([f64; 6], [f64; 6]) {
        let (output, strain) = self.stress_and_strain(input, state);
        (output.stress, strain)
    }
    /// The elastic matrix (for hourglass control and incompatible modes).
    fn elastic_matrix(&self) -> [[f64; 6]; 6];
    /// The law as the kernels on the graphics card compute it, `None` where they have
    /// none.
    fn kernel_law(&self) -> Option<KernelLaw> {
        None
    }
}

/// A material law in the form the kernels on the graphics card take.
#[derive(Clone, Debug, PartialEq)]
pub enum KernelLaw {
    /// `σ = D ε` with [`MaterialLaw::elastic_matrix`] (Saint Venant-Kirchhoff on the
    /// Green-Lagrange strain under NLGEOM).
    Elastic,
    /// J2 plasticity without creep ([`J2Plastic`], [`FiniteStrainJ2`]): the return mapping
    /// on the state variables of [`plastic::STATE_SIZE`].
    J2 {
        shear: f64,
        bulk: f64,
        /// The yield stress against the equivalent plastic strain: piecewise linear,
        /// constant beyond its ends.
        isotropic: Vec<(f64, f64)>,
        /// The stress of the back stress against the equivalent plastic strain (empty
        /// without kinematic hardening).
        kinematic: Vec<(f64, f64)>,
        /// At finite strains ([`FiniteStrainJ2`]): the return mapping on the logarithmic
        /// elastic strain, the state's plastic slots holding `C_p⁻¹ − 1`.
        finite: bool,
    },
}

impl MaterialLaw for LinearElastic {
    fn state_size(&self) -> usize {
        0
    }

    fn is_nonlinear(&self) -> bool {
        false
    }

    fn stress(&self, input: &MaterialInput, _state: &mut [f64]) -> MaterialOutput {
        let d = self.matrix();
        let mut stress = [0.0; 6];
        for (r, s) in stress.iter_mut().enumerate() {
            *s = (0..6).map(|c| d[r][c] * input.strain[c]).sum();
        }
        MaterialOutput { stress, tangent: d }
    }

    fn elastic_matrix(&self) -> [[f64; 6]; 6] {
        self.matrix()
    }

    fn kernel_law(&self) -> Option<KernelLaw> {
        Some(KernelLaw::Elastic)
    }
}

/// The law of a material at a temperature: hyperelastic with a `*HYPERELASTIC`
/// definition, else elastic, or J2 plastic / creeping when the
/// material has a `*PLASTIC` or `*CREEP` definition. With `finite_strain` (NLGEOM steps)
/// the plastic law works on logarithmic strains in the multiplicative framework
/// ([`FiniteStrainJ2`]) and the elastic law is Saint Venant-Kirchhoff on the
/// Green-Lagrange strain the elements hand in.
pub fn law_of(
    material: &Material,
    temperature: f64,
    finite_strain: bool,
) -> Result<Box<dyn MaterialLaw>> {
    if let Some(hyper) = Hyperelastic::of(material)? {
        if material.plastic.is_some() || material.creep.is_some() {
            return Err(Error::input(
                "*HYPERELASTIC",
                format!(
                    "material {}: hyperelasticity cannot be combined with *PLASTIC or *CREEP",
                    material.name
                ),
            ));
        }
        return Ok(Box::new(hyper));
    }
    let elastic = LinearElastic::of(material, temperature)?;
    if material.plastic.is_some() || material.creep.is_some() {
        let plastic = J2Plastic::of(material, elastic, temperature)?;
        if finite_strain {
            Ok(Box::new(FiniteStrainJ2::new(plastic)))
        } else {
            Ok(Box::new(plastic))
        }
    } else {
        Ok(Box::new(elastic))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hooke_matrix_matches_the_textbook() {
        let d = isotropic_matrix(210_000.0, 0.3);
        let e = 210_000.0;
        let nu = 0.3;
        let factor = e / ((1.0 + nu) * (1.0 - 2.0 * nu));
        assert!((d[0][0] - factor * (1.0 - nu)).abs() < 1e-6);
        assert!((d[0][1] - factor * nu).abs() < 1e-6);
        assert!((d[3][3] - e / (2.0 * (1.0 + nu))).abs() < 1e-6);
        assert_eq!(d[0][3], 0.0);
    }
}
