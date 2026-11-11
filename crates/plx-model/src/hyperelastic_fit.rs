//! Fitting the constants of a hyperelastic model to test data: nominal stress over nominal
//! strain from uniaxial, equibiaxial and planar tests of an incompressible specimen.
//!
//! The nominal stress is linear in the constants of the polynomial models (Neo-Hooke,
//! Mooney-Rivlin, Yeoh, polynomial), in the moduli `μ_p` of the Ogden model for fixed
//! exponents and in `μ` of the Arruda-Boyce model for a fixed locking stretch. The linear
//! constants come from weighted linear least squares; the exponents and the locking stretch
//! from a Nelder-Mead search over them with the linear constants solved inside (variable
//! projection), started from several points. The error is relative (each point weighted by
//! its own stress), so that small and large strains count alike, as in most fitting tools.
//! Among the fits of the several starts the best stable one wins unless an unstable one is
//! clearly better.

use serde::{Deserialize, Serialize};

use crate::hyperelastic::{Hyperelastic, HyperelasticModel, TestMode};

/// A point of a test curve: the nominal strain `λ - 1` and the nominal stress (force per
/// undeformed area, in the unit of pressure of the model).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TestPoint {
    pub mode: TestMode,
    pub strain: f64,
    pub stress: f64,
}

/// The outcome of a fit.
#[derive(Clone, Debug, PartialEq)]
pub struct Fit {
    pub constants: Vec<f64>,
    /// Root mean square of the relative stress error over the points.
    pub relative_error: f64,
    /// Where the fitted material becomes unstable, if it does.
    pub unstable: Option<(TestMode, f64)>,
}

/// Why a fit failed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FitError {
    /// Fewer usable points than constants.
    TooFewPoints { needed: usize, given: usize },
    /// The points do not determine the constants (e.g. all at the same strain).
    Singular,
}

impl std::fmt::Display for FitError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FitError::TooFewPoints { needed, given } => write!(
                f,
                "{needed} test points needed (away from zero strain), {given} given"
            ),
            FitError::Singular => {
                f.write_str("The test points do not determine the constants of this model")
            }
        }
    }
}

/// The linear constants of a model with the nonlinear ones fixed: the indices of the linear
/// constants in the constant vector and a template with the nonlinear ones set.
struct Linear {
    template: Vec<f64>,
    unknowns: Vec<usize>,
}

fn linear_part(model: HyperelasticModel, nonlinear: &[f64]) -> Linear {
    let count = model.parameters().len();
    match model {
        HyperelasticModel::Ogden(n) => {
            let n = n.clamp(1, 3) as usize;
            let mut template = vec![0.0; count];
            for p in 0..n {
                template[2 * p + 1] = nonlinear[p];
            }
            Linear {
                template,
                unknowns: (0..n).map(|p| 2 * p).collect(),
            }
        }
        HyperelasticModel::ArrudaBoyce => Linear {
            template: vec![0.0, nonlinear[0]],
            unknowns: vec![0],
        },
        _ => Linear {
            template: vec![0.0; count],
            unknowns: (0..count).collect(),
        },
    }
}

/// Weighted least squares for the linear constants; returns the constants and the sum of
/// the squared relative errors.
fn solve_linear(
    model: HyperelasticModel,
    linear: &Linear,
    points: &[TestPoint],
) -> Option<(Vec<f64>, f64)> {
    let m = linear.unknowns.len();
    let material = |constants: Vec<f64>| Hyperelastic {
        model,
        constants,
        poisson: 0.49,
        test_data: Vec::new(),
    };
    // Basis functions: the nominal stress with one linear constant 1 and the others 0.
    let rows: Vec<(Vec<f64>, f64, f64)> = (points.iter())
        .map(|p| {
            let stretch = 1.0 + p.strain;
            let basis = (linear.unknowns.iter())
                .map(|&k| {
                    let mut c = linear.template.clone();
                    c[k] = 1.0;
                    material(c).nominal_stress(p.mode, stretch)
                })
                .collect();
            (basis, p.stress, 1.0 / p.stress.abs())
        })
        .collect();
    let mut a = vec![vec![0.0; m]; m];
    let mut b = vec![0.0; m];
    for (basis, stress, weight) in &rows {
        let w2 = weight * weight;
        for i in 0..m {
            b[i] += w2 * basis[i] * stress;
            for j in 0..m {
                a[i][j] += w2 * basis[i] * basis[j];
            }
        }
    }
    let x = solve(a, b)?;
    let mut constants = linear.template.clone();
    for (&k, value) in linear.unknowns.iter().zip(&x) {
        constants[k] = *value;
    }
    let error = (rows.iter())
        .map(|(basis, stress, weight)| {
            let model: f64 = basis.iter().zip(&x).map(|(g, c)| g * c).sum();
            ((model - stress) * weight).powi(2)
        })
        .sum();
    Some((constants, error))
}

/// Gaussian elimination with partial pivoting; `None` when the matrix is singular.
#[allow(clippy::needless_range_loop)]
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    let scale = (0..n).map(|i| a[i][i].abs()).fold(0.0, f64::max);
    if scale.is_nan() || scale <= 0.0 {
        return None;
    }
    for col in 0..n {
        let pivot = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() <= 1e-12 * scale {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..n {
            let factor = a[row][col] / a[col][col];
            for k in col..n {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let sum: f64 = (row + 1..n).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - sum) / a[row][row];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// Minimizes `f` by the Nelder-Mead simplex method from `start` with initial steps `step`.
fn nelder_mead(f: &dyn Fn(&[f64]) -> f64, start: &[f64], step: f64) -> (Vec<f64>, f64) {
    let n = start.len();
    let mut simplex: Vec<(Vec<f64>, f64)> = (0..=n)
        .map(|i| {
            let mut x = start.to_vec();
            if i > 0 {
                x[i - 1] += step;
            }
            let value = f(&x);
            (x, value)
        })
        .collect();
    for _ in 0..400 * n {
        simplex.sort_by(|a, b| a.1.total_cmp(&b.1));
        let (best, worst) = (simplex[0].1, simplex[n].1);
        if (worst - best).abs() <= 1e-12 * (best.abs() + 1e-30) {
            break;
        }
        let centroid: Vec<f64> = (0..n)
            .map(|k| simplex[..n].iter().map(|(x, _)| x[k]).sum::<f64>() / n as f64)
            .collect();
        let along = |t: f64| -> Vec<f64> {
            (0..n)
                .map(|k| centroid[k] + t * (simplex[n].0[k] - centroid[k]))
                .collect()
        };
        let reflected = along(-1.0);
        let fr = f(&reflected);
        if fr < simplex[0].1 {
            let expanded = along(-2.0);
            let fe = f(&expanded);
            simplex[n] = if fe < fr {
                (expanded, fe)
            } else {
                (reflected, fr)
            };
        } else if fr < simplex[n - 1].1 {
            simplex[n] = (reflected, fr);
        } else {
            let contracted = along(0.5);
            let fc = f(&contracted);
            if fc < worst {
                simplex[n] = (contracted, fc);
            } else {
                let best_point = simplex[0].0.clone();
                for (x, value) in simplex.iter_mut().skip(1) {
                    for k in 0..n {
                        x[k] = best_point[k] + 0.5 * (x[k] - best_point[k]);
                    }
                    *value = f(x);
                }
            }
        }
    }
    simplex.sort_by(|a, b| a.1.total_cmp(&b.1));
    simplex.swap_remove(0)
}

/// Starting values of the nonlinear constants.
fn starts(model: HyperelasticModel) -> Vec<Vec<f64>> {
    match model {
        HyperelasticModel::Ogden(1) => vec![vec![2.0], vec![-2.0], vec![5.0], vec![0.5]],
        HyperelasticModel::Ogden(2) => vec![
            vec![2.0, -2.0],
            vec![1.5, 5.0],
            vec![3.0, -1.0],
            vec![1.0, 8.0],
        ],
        HyperelasticModel::Ogden(_) => vec![
            vec![1.3, 5.0, -2.0],
            vec![2.0, -2.0, 6.0],
            vec![0.5, 3.0, -1.0],
        ],
        // The logarithm of `λm - 1`.
        HyperelasticModel::ArrudaBoyce => vec![vec![2.0f64.ln()], vec![6.0f64.ln()]],
        _ => Vec::new(),
    }
}

/// Fits the constants of `model` to the test points (the material's Poisson's ratio does not
/// take part: the tests are taken as incompressible).
pub fn fit(model: HyperelasticModel, points: &[TestPoint]) -> Result<Fit, FitError> {
    let usable: Vec<TestPoint> = (points.iter())
        .filter(|p| p.strain.abs() > 1e-9 && p.strain > -1.0 && p.stress.abs() > 0.0)
        .filter(|p| p.strain.is_finite() && p.stress.is_finite())
        .copied()
        .collect();
    let needed = model.parameters().len();
    if usable.len() < needed {
        return Err(FitError::TooFewPoints {
            needed,
            given: usable.len(),
        });
    }
    let finish = |constants: Vec<f64>, error: f64| {
        let material = Hyperelastic {
            model,
            constants: constants.clone(),
            poisson: 0.49,
            test_data: Vec::new(),
        };
        Fit {
            constants,
            relative_error: (error / usable.len() as f64).sqrt(),
            unstable: material.unstable_stretch(),
        }
    };
    let starts = starts(model);
    if starts.is_empty() {
        let (constants, error) =
            solve_linear(model, &linear_part(model, &[]), &usable).ok_or(FitError::Singular)?;
        return Ok(finish(constants, error));
    }
    // The nonlinear constants as the search sees them: Ogden exponents directly, the
    // locking stretch as ln(λm - 1) so that it stays above 1.
    let decode = |x: &[f64]| -> Vec<f64> {
        match model {
            HyperelasticModel::ArrudaBoyce => vec![1.0 + x[0].exp()],
            _ => x.to_vec(),
        }
    };
    let objective = |x: &[f64]| -> f64 {
        let nonlinear = decode(x);
        if nonlinear.iter().any(|v| v.abs() < 1e-3 || v.abs() > 50.0) {
            return f64::MAX;
        }
        solve_linear(model, &linear_part(model, &nonlinear), &usable)
            .map_or(f64::MAX, |(_, error)| error)
    };
    let mut fits: Vec<Fit> = Vec::new();
    for start in starts {
        let (x, value) = nelder_mead(&objective, &start, 0.5);
        if value == f64::MAX {
            continue;
        }
        if let Some((constants, error)) =
            solve_linear(model, &linear_part(model, &decode(&x)), &usable)
        {
            let fit = finish(constants, error);
            let material = Hyperelastic {
                model,
                constants: fit.constants.clone(),
                poisson: 0.49,
                test_data: Vec::new(),
            };
            if material.initial_shear_modulus() > 0.0 {
                fits.push(fit);
            }
        }
    }
    let best = |candidates: &mut dyn Iterator<Item = &Fit>| {
        candidates
            .min_by(|a, b| a.relative_error.total_cmp(&b.relative_error))
            .cloned()
    };
    let overall = best(&mut fits.iter()).ok_or(FitError::Singular)?;
    match best(&mut fits.iter().filter(|f| f.unstable.is_none())) {
        Some(stable) if stable.relative_error <= 1.5 * overall.relative_error + 1e-3 => Ok(stable),
        _ => Ok(overall),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test points of a known material in all three modes.
    fn data(material: &Hyperelastic) -> Vec<TestPoint> {
        let mut points = Vec::new();
        for mode in TestMode::ALL {
            for i in 1..=12 {
                let strain =
                    0.1 * f64::from(i) * if mode == TestMode::Uniaxial { 2.0 } else { 1.0 };
                points.push(TestPoint {
                    mode,
                    strain,
                    stress: material.nominal_stress(mode, 1.0 + strain),
                });
            }
        }
        points
    }

    #[test]
    fn a_fit_recovers_the_constants_of_the_data() {
        for (model, constants) in [
            (HyperelasticModel::NeoHooke, vec![0.4e6]),
            (HyperelasticModel::MooneyRivlin, vec![0.3e6, 0.08e6]),
            (HyperelasticModel::Yeoh, vec![0.4e6, -0.02e6, 0.003e6]),
            (HyperelasticModel::Ogden(1), vec![0.8e6, 3.0]),
            (HyperelasticModel::Ogden(2), vec![0.6e6, 1.5, 0.01e6, 6.0]),
            (HyperelasticModel::ArrudaBoyce, vec![0.5e6, 3.0]),
        ] {
            let material = Hyperelastic {
                model,
                constants: constants.clone(),
                poisson: 0.49,
                test_data: Vec::new(),
            };
            let fit = fit(model, &data(&material)).unwrap();
            assert!(fit.relative_error < 1e-4, "{model:?}: {fit:?}");
            for (a, b) in fit.constants.iter().zip(&constants) {
                assert!(
                    (a - b).abs() < 1e-3 * b.abs().max(1.0),
                    "{model:?}: {fit:?}"
                );
            }
        }
    }

    #[test]
    fn too_few_points_are_refused() {
        let points = [TestPoint {
            mode: TestMode::Uniaxial,
            strain: 0.5,
            stress: 1e6,
        }];
        assert_eq!(
            fit(HyperelasticModel::MooneyRivlin, &points),
            Err(FitError::TooFewPoints {
                needed: 2,
                given: 1
            })
        );
        assert!(fit(HyperelasticModel::NeoHooke, &points).is_ok());
    }
}
