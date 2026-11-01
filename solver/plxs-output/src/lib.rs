//! Writers of the PrePoLix solver's result files in CalculiX's formats, so that PrePoLix
//! reads them unchanged: `.frd` (field results with the mesh), `.dat` (history output),
//! `.sta` (increment table) and `.cvg` (convergence table).
//!
//! The formats are taken from the files CalculiX writes (column layout observed on the
//! golden files of the fixture corpus) and from PrePoLix's own readers in `plx-io`; no
//! CalculiX code was consulted. The ASCII variant with 12-column fixed-width numbers is
//! written; the binary variant follows later.

pub mod dat;
pub mod frd;
pub mod sta;

pub use dat::{EigenTable, write_dat};
pub use frd::write_frd;
pub use sta::{CvgRow, StaRow, write_cvg, write_sta};

/// A number in Fortran's `E12.5` layout: `-1.77481E-03`, `0.00000E+00`, three-digit
/// exponents when needed (`1.00000E+100` is 12 columns too).
pub fn fortran_e12(value: f64) -> String {
    if value == 0.0 || !value.is_finite() {
        return " 0.00000E+00".to_string();
    }
    let text = format!("{:.5E}", value.abs());
    let (mantissa, exponent) = text.split_once('E').unwrap();
    let exponent: i32 = exponent.parse().unwrap();
    let sign = if value < 0.0 { "-" } else { " " };
    let exp_sign = if exponent < 0 { "-" } else { "+" };
    format!("{sign}{mantissa}E{exp_sign}{:02}", exponent.abs())
}

/// A number as the `.dat` file prints values: `-7.105427E-15`, 13 columns.
pub fn fortran_e13(value: f64) -> String {
    if value == 0.0 || !value.is_finite() {
        return " 0.000000E+00".to_string();
    }
    let text = format!("{:.6E}", value.abs());
    let (mantissa, exponent) = text.split_once('E').unwrap();
    let exponent: i32 = exponent.parse().unwrap();
    let sign = if value < 0.0 { "-" } else { " " };
    let exp_sign = if exponent < 0 { "-" } else { "+" };
    format!("{sign}{mantissa}E{exp_sign}{:02}", exponent.abs())
}

/// A time as the `.dat` headings print it: `0.1000000E+01` (mantissa below one).
pub fn fortran_time(value: f64) -> String {
    if value == 0.0 || !value.is_finite() {
        return "0.0000000E+00".to_string();
    }
    let text = format!("{:.6E}", value.abs());
    let (mantissa, exponent) = text.split_once('E').unwrap();
    let exponent: i32 = exponent.parse::<i32>().unwrap() + 1;
    let digits: String = mantissa.chars().filter(|c| c.is_ascii_digit()).collect();
    let sign = if value < 0.0 { "-" } else { "" };
    let exp_sign = if exponent < 0 { "-" } else { "+" };
    format!("{sign}0.{digits}E{exp_sign}{:02}", exponent.abs())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn numbers_take_fortran_layout() {
        assert_eq!(fortran_e12(-1.77481e-3), "-1.77481E-03");
        assert_eq!(fortran_e12(0.0), " 0.00000E+00");
        assert_eq!(fortran_e12(257.543), " 2.57543E+02");
        assert_eq!(fortran_e12(1e100), " 1.00000E+100");
        assert_eq!(fortran_e13(-7.105427e-15), "-7.105427E-15");
        assert_eq!(fortran_e13(4.673366e-3), " 4.673366E-03");
        assert_eq!(fortran_time(1.0), "0.1000000E+01");
        assert_eq!(fortran_time(2.0), "0.2000000E+01");
        assert_eq!(fortran_time(0.05), "0.5000000E-01");
    }
}
