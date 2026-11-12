//! Anisotropic linear elasticity of materials: orthotropic engineering constants or a
//! general stiffness matrix, in the material axes 1, 2, 3. A solid section's orientation
//! (a coordinate system) puts the axes into the model; without one they are the global
//! x, y, z.

use serde::{Deserialize, Serialize};

use crate::Elastic;

/// The nine engineering constants of an orthotropic material: Young's moduli along the
/// material axes, Poisson's ratios `νij` (the contraction along `j` under a stress along
/// `i`) and shear moduli of the planes. Moduli in Pa, as values are stored.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct EngineeringConstants {
    pub e1: f64,
    pub e2: f64,
    pub e3: f64,
    pub nu12: f64,
    pub nu13: f64,
    pub nu23: f64,
    pub g12: f64,
    pub g13: f64,
    pub g23: f64,
}

impl EngineeringConstants {
    /// The constants of an isotropic material.
    pub fn isotropic(elastic: Elastic) -> Self {
        let g = elastic.young / (2.0 * (1.0 + elastic.poisson));
        Self {
            e1: elastic.young,
            e2: elastic.young,
            e3: elastic.young,
            nu12: elastic.poisson,
            nu13: elastic.poisson,
            nu23: elastic.poisson,
            g12: g,
            g13: g,
            g23: g,
        }
    }

    /// The constants in the order of the solver's data line.
    pub fn values(&self) -> [f64; 9] {
        [
            self.e1, self.e2, self.e3, self.nu12, self.nu13, self.nu23, self.g12, self.g13,
            self.g23,
        ]
    }
}

/// Anisotropic linear elasticity.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Anisotropic {
    /// Orthotropic by its engineering constants.
    Orthotropic(EngineeringConstants),
    /// The general stiffness: the upper triangle of the 6 × 6 matrix acting on the
    /// engineering strains `[11, 22, 33, 12, 13, 23]`, row by row (`D1111, D1122, D1133,
    /// D1112, D1113, D1123, D2222, ...`), in Pa.
    General(Box<[f64; 21]>),
}

/// The order of the strain components of [`Anisotropic::General`], for the dialog.
pub const GENERAL_COMPONENTS: [&str; 6] = ["11", "22", "33", "12", "13", "23"];

impl Anisotropic {
    /// The stiffness of an isotropic material as a general matrix.
    pub fn general_of(elastic: Elastic) -> [f64; 21] {
        let d = plxs_materials::isotropic_matrix(elastic.young, elastic.poisson);
        from_matrix(&d)
    }

    /// The constants as the solver takes them.
    pub fn solver(&self) -> plxs_core::material::AnisotropicElastic {
        use plxs_core::material::AnisotropicElastic as A;
        match self {
            Anisotropic::Orthotropic(c) => A::EngineeringConstants(c.values()),
            Anisotropic::General(upper) => {
                // The solver's ANISO order runs column by column over 11, 22, 33, 12, 13, 23.
                let mut out = [0.0; 21];
                let mut k = 0;
                for col in 0..6 {
                    for row in 0..=col {
                        out[k] = upper[upper_index(row, col)];
                        k += 1;
                    }
                }
                A::Anisotropic(out)
            }
        }
    }

    /// The 6 × 6 stiffness in the material axes, in the solver's strain order
    /// `[11, 22, 33, 12, 23, 31]`; `Err` with the reason when it is not stable.
    pub fn matrix(&self) -> Result<[[f64; 6]; 6], String> {
        plxs_materials::anisotropic::material_matrix("", &self.solver()).map_err(|e| {
            let text = e.to_string();
            // Leave out the solver's "material : " prefix.
            match text.split_once("material : ") {
                Some((_, rest)) => rest.to_owned(),
                None => text,
            }
        })
    }

    /// Why the constants cannot be used, if they cannot.
    pub fn problem(&self) -> Option<String> {
        self.matrix().err()
    }

    /// The isotropic constants nearest to the material (the Voigt average of its moduli),
    /// for estimates that want one modulus; `None` when it is not stable.
    pub fn isotropic_equivalent(&self) -> Option<Elastic> {
        let d = self.matrix().ok()?;
        let e = plxs_materials::anisotropic::isotropic_equivalent(&d);
        Some(Elastic {
            young: e.young,
            poisson: e.poisson,
        })
    }
}

/// Index into the row-wise upper triangle of a symmetric 6 × 6 matrix in the order of
/// [`GENERAL_COMPONENTS`].
pub fn upper_index(row: usize, col: usize) -> usize {
    let (r, c) = if row <= col { (row, col) } else { (col, row) };
    // Rows before `r` hold 6, 5, ... entries.
    6 * r - r * r.saturating_sub(1) / 2 + c - r
}

/// The solver's index (`[11, 22, 33, 12, 23, 31]`) of a component of
/// [`GENERAL_COMPONENTS`].
const fn solver_index(i: usize) -> usize {
    match i {
        4 => 5,
        5 => 4,
        i => i,
    }
}

/// A solver-order 6 × 6 matrix as the row-wise upper triangle of [`Anisotropic::General`].
pub fn from_matrix(d: &[[f64; 6]; 6]) -> [f64; 21] {
    let mut out = [0.0; 21];
    for row in 0..6 {
        for col in row..6 {
            out[upper_index(row, col)] = d[solver_index(row)][solver_index(col)];
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_upper_triangle_is_numbered_row_by_row() {
        assert_eq!(upper_index(0, 0), 0);
        assert_eq!(upper_index(0, 5), 5);
        assert_eq!(upper_index(1, 1), 6);
        assert_eq!(upper_index(2, 2), 11);
        assert_eq!(upper_index(5, 5), 20);
        assert_eq!(upper_index(4, 1), upper_index(1, 4));
    }

    #[test]
    fn a_general_matrix_of_hooke_matches_its_engineering_constants() {
        let elastic = Elastic {
            young: 3.5e9,
            poisson: 0.35,
        };
        let general = Anisotropic::General(Box::new(Anisotropic::general_of(elastic)))
            .matrix()
            .unwrap();
        let ortho = Anisotropic::Orthotropic(EngineeringConstants::isotropic(elastic))
            .matrix()
            .unwrap();
        for (a, b) in general.iter().flatten().zip(ortho.iter().flatten()) {
            assert!((a - b).abs() < 1e-3 * 3.5e9, "{a} {b}");
        }
        let back = Anisotropic::General(Box::new(Anisotropic::general_of(elastic)))
            .isotropic_equivalent()
            .unwrap();
        assert!((back.young - 3.5e9).abs() < 1.0);
    }

    #[test]
    fn a_shear_coupling_lands_where_the_dialog_shows_it() {
        // D1123: row 11, column 23 of the dialog.
        let mut upper = Anisotropic::general_of(Elastic {
            young: 1000.0,
            poisson: 0.3,
        });
        upper[upper_index(0, 5)] = 10.0;
        let d = Anisotropic::General(Box::new(upper)).matrix().unwrap();
        // The solver's 23 is its index 4.
        assert_eq!(d[0][4], 10.0);
    }
}
