//! Infill sections: solid parts printed by fused deposition (FDM), with solid walls under
//! the surface and a sparse infill inside. The model keeps the part as one solid; the
//! section gives every element the stiffness of what is printed there:
//!
//! - The **walls** (perimeters along the sides, solid layers at top and bottom) are the
//!   printed material: transversely isotropic, softer across the layers (along the build
//!   direction) by the layer factor.
//! - The **core** is the infill homogenized into an orthotropic solid: its walls or lines
//!   are thin plates or bars of the printed material, averaged at equal strain (Voigt) by
//!   their volume fractions, plus the bending stiffness of the cells (Gibson & Ashby,
//!   *Cellular Solids*, 1997) for the deformations the walls alone do not resist. Gyroid
//!   infill follows the fit of Gonabadi, Hosseini & Bull, *Numerical Modeling and
//!   Experimental Investigation of Effective Elastic Properties of the 3D Printed Gyroid
//!   Infill*, Applied Sciences 12 (2022) 2180.
//! - Elements reaching through the wall get the volume average of both by the share of
//!   their depth below the surface that lies in the wall.
//!
//! All stiffnesses here are in the print axes: x and y in the layer (x the reference of the
//! infill angle), z the build direction. The section's orientation (a coordinate system)
//! places them in the model.

use serde::{Deserialize, Serialize};

use crate::Elastic;
use plxs_materials::anisotropic::{invert, rotate};

/// A 6 × 6 stiffness matrix on the solver's engineering strains.
pub type Stiffness = [[f64; 6]; 6];

/// The infill patterns of the slicers, by the way they carry load.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum InfillPattern {
    /// Parallel lines, turned by 90° from layer to layer (PrusaSlicer Rectilinear, Cura
    /// Lines).
    Rectilinear,
    /// Lines in two directions in every layer, stacked into square cells (Grid).
    #[default]
    Grid,
    /// Lines in three directions in every layer, stacked into triangular prisms
    /// (Triangles, Cura Tri-Hexagon).
    Triangles,
    /// Three families of walls tilted like the faces of a cube on its corner (Cubic,
    /// Adaptive Cubic, Support Cubic).
    Cubic,
    /// The gyroid minimal surface (Gyroid).
    Gyroid,
    /// Hexagonal prisms (Honeycomb).
    Honeycomb,
    /// The core is a material of its own, e.g. one measured on printed specimens.
    Custom,
}

impl InfillPattern {
    pub const ALL: [InfillPattern; 7] = [
        Self::Rectilinear,
        Self::Grid,
        Self::Triangles,
        Self::Cubic,
        Self::Gyroid,
        Self::Honeycomb,
        Self::Custom,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Rectilinear => "Rectilinear / Lines",
            Self::Grid => "Grid",
            Self::Triangles => "Triangles",
            Self::Cubic => "Cubic",
            Self::Gyroid => "Gyroid",
            Self::Honeycomb => "Honeycomb",
            Self::Custom => "Custom core material",
        }
    }
}

/// An infill section: the printed material is the section's material, the build direction
/// the z axis of its orientation.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct InfillSection {
    pub pattern: InfillPattern,
    /// Infill density, the volume fraction of material in the core, 0 to 1.
    pub density: f64,
    /// Thickness of the perimeters along the sides (number of perimeters times the
    /// extrusion width).
    pub wall_thickness: f64,
    /// Thickness of the solid layers at the top and bottom (number of layers times the
    /// layer height).
    pub top_bottom_thickness: f64,
    /// Direction of the infill lines in the layer, in degrees from the x axis of the
    /// orientation.
    pub angle: f64,
    /// Stiffness of the printed material across the layers (Young's modulus along the
    /// build direction, the shear moduli of the planes containing it) as a fraction of the
    /// stiffness in the layer.
    pub layer_factor: f64,
    /// The core material of [`InfillPattern::Custom`], in the print axes.
    pub core_material: Option<crate::Id>,
}

impl InfillSection {
    /// 20 % grid at 45°, two perimeters and four layers of 0.2 mm at top and bottom, as
    /// the slicers start.
    pub const DEFAULT: Self = Self {
        pattern: InfillPattern::Grid,
        density: 0.2,
        wall_thickness: 0.8e-3,
        top_bottom_thickness: 0.8e-3,
        angle: 45.0,
        layer_factor: 0.8,
        core_material: None,
    };

    /// Why the values cannot be used, if they cannot.
    pub fn problem(&self) -> Option<&'static str> {
        if self.pattern != InfillPattern::Custom && !(self.density > 0.0 && self.density <= 1.0) {
            Some("The infill density must be above 0 % and at most 100 %.")
        } else if self.wall_thickness < 0.0 || self.top_bottom_thickness < 0.0 {
            Some("The wall thicknesses must not be negative.")
        } else if !(self.layer_factor > 0.0 && self.layer_factor <= 1.0) {
            Some("The layer factor must be above 0 and at most 1.")
        } else {
            None
        }
    }

    /// The stiffness of the walls: the printed material, softer along z.
    pub fn wall_matrix(&self, base: Elastic) -> Result<[[f64; 6]; 6], String> {
        printed(base, self.layer_factor)
    }

    /// The stiffness of the core in the print axes. `core` is the matrix of the custom
    /// core material (in the print axes), needed for [`InfillPattern::Custom`] only.
    pub fn core_matrix(
        &self,
        base: Elastic,
        core: Option<[[f64; 6]; 6]>,
    ) -> Result<[[f64; 6]; 6], String> {
        let solid = printed(base, self.layer_factor)?;
        let rho = self.density.clamp(1e-3, 1.0);
        let a = self.angle.to_radians();
        // Directions in the layer at the infill angle plus `deg`.
        let dir = |deg: f64| {
            let t = a + deg.to_radians();
            [t.cos(), t.sin(), 0.0]
        };
        // A vertical wall along `t` has the normal in the layer at a right angle to it.
        let wall = |deg: f64| dir(deg + 90.0);
        let mut d = [[0.0; 6]; 6];
        let mut add = |part: [[f64; 6]; 6], fraction: f64| {
            for (row, part_row) in d.iter_mut().zip(part) {
                for (v, p) in row.iter_mut().zip(part_row) {
                    *v += fraction * p;
                }
            }
        };
        // The bending stiffness of the cells: `C ρ^n (1 - ρ)` times the printed material.
        let (bending_c, bending_n) = match self.pattern {
            InfillPattern::Rectilinear => {
                for deg in [0.0, 90.0] {
                    add(bar(&solid, dir(deg))?, rho / 2.0);
                }
                // Lines crossing from layer to layer, bending like an open-cell foam.
                (1.0, 2.0)
            }
            InfillPattern::Grid => {
                for deg in [0.0, 90.0] {
                    add(plate(&solid, wall(deg), false)?, rho / 2.0);
                }
                (1.0, 3.0)
            }
            InfillPattern::Triangles => {
                for deg in [0.0, 60.0, 120.0] {
                    add(plate(&solid, wall(deg), false)?, rho / 3.0);
                }
                (1.0, 3.0)
            }
            InfillPattern::Honeycomb => {
                // The walls stretch along z and shear in their planes; in the layer the
                // cells bend (Gibson & Ashby: E/Es = 2.3 (t/l)³ ≈ 1.5 ρ³).
                for deg in [0.0, 60.0, 120.0] {
                    add(plate(&solid, wall(deg), true)?, rho / 3.0);
                }
                (1.5, 3.0)
            }
            InfillPattern::Cubic => {
                // The faces of a cube standing on its corner: normals 54.7° from z, 120°
                // apart around it.
                let tilt = (1.0_f64 / 3.0).sqrt().acos();
                for deg in [0.0_f64, 120.0, 240.0] {
                    let t = a + deg.to_radians();
                    let normal = [tilt.sin() * t.cos(), tilt.sin() * t.sin(), tilt.cos()];
                    add(plate(&solid, normal, false)?, rho / 3.0);
                }
                (1.0, 2.0)
            }
            InfillPattern::Gyroid => {
                // Nearly isotropic; the fit of Gonabadi et al. (2022) to the printed
                // material.
                let scale = (rho - (1.0 - rho).powf(0.77) * rho.powf(1.2)).max(0.0);
                add(solid, scale);
                (0.0, 2.0)
            }
            InfillPattern::Custom => {
                return core.ok_or_else(|| "The infill section has no core material.".into());
            }
        };
        // The bending part fades out towards a full infill, where the walls touch and
        // carry everything by themselves.
        if bending_c > 0.0 {
            add(
                plxs_materials::isotropic_matrix(base.young, 0.3),
                bending_c * rho.powf(bending_n) * (1.0 - rho),
            );
        }
        Ok(d)
    }

    /// The stiffness of the core and the walls in the model's axes, `axes` the rows of
    /// the print axes in global coordinates.
    pub fn matrices_in(
        &self,
        base: Elastic,
        core: Option<[[f64; 6]; 6]>,
        axes: &[[f64; 3]; 3],
    ) -> Result<(Stiffness, Stiffness), String> {
        Ok((
            rotate(&self.wall_matrix(base)?, axes),
            rotate(&self.core_matrix(base, core)?, axes),
        ))
    }
}

/// The printed material: isotropic in the layer, `factor` times as stiff along z and in
/// shear across the layers.
fn printed(base: Elastic, factor: f64) -> Result<[[f64; 6]; 6], String> {
    let g = base.young / (2.0 * (1.0 + base.poisson));
    crate::Anisotropic::Orthotropic(crate::EngineeringConstants {
        e1: base.young,
        e2: base.young,
        e3: factor * base.young,
        nu12: base.poisson,
        // The contraction in the layer under a stress across it is the material's; the
        // reverse one follows from symmetry.
        nu13: base.poisson * factor,
        nu23: base.poisson * factor,
        g12: g,
        g13: factor * g,
        g23: factor * g,
    })
    .matrix()
}

/// The local axes (rows) with the third along `n`.
fn frame(n: [f64; 3]) -> [[f64; 3]; 3] {
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    let n = n.map(|v| v / len);
    let helper = if n[2].abs() < 0.9 {
        [0.0, 0.0, 1.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let t = cross(helper, n);
    let lt = (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]).sqrt();
    let t = t.map(|v| v / lt);
    let s = cross(n, t);
    [t, s, n]
}

fn transpose(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
}

/// A thin plate of normal `n` of the material `solid`: the stress normal to it and its
/// transverse shears are zero (plane stress in the plate). With `no_axial` the plate
/// carries no stress along its horizontal direction either: the cell walls of a honeycomb
/// bend there instead of stretching.
fn plate(solid: &[[f64; 6]; 6], n: [f64; 3], no_axial: bool) -> Result<[[f64; 6]; 6], String> {
    let mut axes = frame(n);
    if no_axial {
        // The horizontal direction of a vertical wall as the first local axis.
        let horizontal = [-n[1], n[0], 0.0];
        let len = (horizontal[0].powi(2) + horizontal[1].powi(2)).sqrt();
        let t = horizontal.map(|v| v / len);
        axes = [t, [0.0, 0.0, 1.0], n];
        let s = [
            t[1] * n[2] - t[2] * n[1],
            t[2] * n[0] - t[0] * n[2],
            t[0] * n[1] - t[1] * n[0],
        ];
        axes[1] = s;
    }
    // The material in the plate's axes: the print axes seen from the plate frame.
    let local = rotate(solid, &transpose(&axes));
    let s = invert(&local).ok_or("The printed material is singular.")?;
    // In the plate (local 11, 22, 12; solver indices 0, 1, 3) only those stresses act: the
    // stiffness is the inverse of their compliance block.
    let keep: &[usize] = if no_axial { &[1, 3] } else { &[0, 1, 3] };
    let mut block = [[0.0; 6]; 6];
    for (i, &a) in keep.iter().enumerate() {
        for (j, &b) in keep.iter().enumerate() {
            block[i][j] = s[a][b];
        }
    }
    // Invert the kept block, padded with the identity.
    for (i, row) in block.iter_mut().enumerate().skip(keep.len()) {
        row[i] = 1.0;
    }
    let q = invert(&block).ok_or("The printed material is singular.")?;
    let mut d_local = [[0.0; 6]; 6];
    for (i, &a) in keep.iter().enumerate() {
        for (j, &b) in keep.iter().enumerate() {
            d_local[a][b] = q[i][j];
        }
    }
    Ok(rotate(&d_local, &axes))
}

/// A line along `t` of the material `solid`: stiff along itself only.
fn bar(solid: &[[f64; 6]; 6], t: [f64; 3]) -> Result<[[f64; 6]; 6], String> {
    // Local axes with the first along the bar.
    let f = frame(t);
    let axes = [f[2], f[0], f[1]];
    let local = rotate(solid, &transpose(&axes));
    let s = invert(&local).ok_or("The printed material is singular.")?;
    let mut d_local = [[0.0; 6]; 6];
    d_local[0][0] = 1.0 / s[0][0];
    Ok(rotate(&d_local, &axes))
}

/// The share of an element in a wall of thickness `t`, from the depths of its nodes below
/// the surface: the part of its depth range above `t`.
pub fn wall_share(depths: impl IntoIterator<Item = f64>, t: f64) -> f64 {
    let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
    for d in depths {
        low = low.min(d);
        high = high.max(d);
    }
    if t.is_nan() || t <= 0.0 || !low.is_finite() || low >= t {
        return 0.0;
    }
    if high <= t || high - low <= 0.0 {
        return 1.0;
    }
    ((t - low) / (high - low)).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    const PLA: Elastic = Elastic {
        young: 3.5e9,
        poisson: 0.35,
    };

    fn moduli(d: &[[f64; 6]; 6]) -> [f64; 3] {
        let s = invert(d).unwrap();
        [1.0 / s[0][0], 1.0 / s[1][1], 1.0 / s[2][2]]
    }

    fn section(pattern: InfillPattern, density: f64) -> InfillSection {
        InfillSection {
            pattern,
            density,
            angle: 0.0,
            layer_factor: 1.0,
            ..InfillSection::DEFAULT
        }
    }

    #[test]
    fn triangles_are_isotropic_in_the_layer_with_a_third_of_the_stiffness() {
        // The triangular lattice of Gibson & Ashby: E = ρ Es / 3 in the layer. The walls
        // are plates here, a little stiffer than bars by 1 / (1 - ν²) in their planes.
        let rho = 0.2;
        let d = section(InfillPattern::Triangles, rho)
            .core_matrix(PLA, None)
            .unwrap();
        let [ex, ey, ez] = moduli(&d);
        assert!((ex - ey).abs() < 1e-6 * ex, "{ex} {ey}");
        let lattice = rho * PLA.young / 3.0;
        assert!(ex > lattice && ex < 1.25 * lattice, "{ex} {lattice}");
        // Along z the prisms carry like a solid of their density.
        assert!((ez - rho * PLA.young).abs() < 0.1 * rho * PLA.young, "{ez}");
    }

    #[test]
    fn grid_is_stiff_along_its_lines_and_soft_at_45_degrees() {
        let d = section(InfillPattern::Grid, 0.2)
            .core_matrix(PLA, None)
            .unwrap();
        let [ex, ey, _] = moduli(&d);
        assert!((ex - ey).abs() < 1e-6 * ex);
        // Half of the material along x.
        // The walls along y hold the contraction along z a little.
        assert!((ex / (0.1 * PLA.young) - 1.0).abs() < 0.2, "{ex}");
        let turned = InfillSection {
            angle: 45.0,
            ..section(InfillPattern::Grid, 0.2)
        }
        .core_matrix(PLA, None)
        .unwrap();
        let [ex45, ..] = moduli(&turned);
        assert!(ex45 < 0.1 * ex, "{ex45} against {ex}");
    }

    #[test]
    fn honeycomb_bends_in_the_layer_and_carries_along_z() {
        let rho = 0.2;
        let d = section(InfillPattern::Honeycomb, rho)
            .core_matrix(PLA, None)
            .unwrap();
        let [ex, ey, ez] = moduli(&d);
        let bending = 1.5 * rho.powi(3) * PLA.young;
        assert!(ex > 0.5 * bending && ex < 3.0 * bending, "{ex} {bending}");
        assert!((ex - ey).abs() < 1e-3 * ex);
        assert!((ez / (rho * PLA.young) - 1.0).abs() < 0.1, "{ez}");
    }

    #[test]
    fn every_pattern_gives_a_stable_core_that_grows_with_the_density() {
        for pattern in InfillPattern::ALL {
            if pattern == InfillPattern::Custom {
                continue;
            }
            let mut last = [0.0; 3];
            for rho in [0.1, 0.2, 0.4, 0.8] {
                let s = InfillSection {
                    layer_factor: 0.8,
                    angle: 45.0,
                    ..section(pattern, rho)
                };
                let d = s.core_matrix(PLA, None).unwrap();
                assert!(
                    plxs_materials::anisotropic::positive_definite(&d),
                    "{pattern:?} {rho}"
                );
                let m = moduli(&d);
                for k in 0..3 {
                    assert!(m[k] > last[k], "{pattern:?} {rho} {k}");
                    assert!(m[k] < PLA.young, "{pattern:?} {rho} {k}");
                }
                last = m;
            }
        }
    }

    #[test]
    fn gyroid_follows_the_fit() {
        let d = section(InfillPattern::Gyroid, 0.5)
            .core_matrix(PLA, None)
            .unwrap();
        let [ex, _, ez] = moduli(&d);
        let fit = 0.5 - 0.5_f64.powf(0.77) * 0.5_f64.powf(1.2);
        assert!((ex / PLA.young - fit).abs() < 1e-9, "{}", ex / PLA.young);
        assert!((ez - ex).abs() < 1e-6 * ex);
    }

    #[test]
    fn walls_are_softer_across_the_layers() {
        let s = InfillSection::DEFAULT;
        let [ex, ey, ez] = moduli(&s.wall_matrix(PLA).unwrap());
        assert!((ex - PLA.young).abs() < 1.0 && (ey - PLA.young).abs() < 1.0);
        assert!((ez - 0.8 * PLA.young).abs() < 1.0);
    }

    #[test]
    fn the_wall_share_is_the_depth_range_inside_the_wall() {
        assert_eq!(wall_share([0.0, 0.5, 1.0], 2.0), 1.0);
        assert_eq!(wall_share([3.0, 4.0], 2.0), 0.0);
        assert!((wall_share([0.0, 4.0], 1.0) - 0.25).abs() < 1e-12);
        assert!((wall_share([1.0, 3.0], 2.0) - 0.5).abs() < 1e-12);
    }
}
