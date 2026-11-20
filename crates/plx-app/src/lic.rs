//! Line integral convolution of the shown vector source: its values per node for the
//! renderer, scaled so that the largest magnitude is 1, and the settings of the LIC pass.

use glam::{DAffine3, DMat3, DVec3};
use plx_render::{Directions, LicDisplay, LicSource};
use plx_results::display::Representation;
use plx_results::{Principal, VectorSource};

use crate::results::ResultsView;

/// How the renderer draws the streaks of the view, if it shows LIC.
pub fn display(view: &ResultsView) -> Option<LicDisplay> {
    if view.representation() != Representation::Lic {
        return None;
    }
    let source = match view.vector_source()? {
        VectorSource::Vector { .. } => LicSource::Vector,
        VectorSource::Principal { which, .. } => LicSource::Principal(match which {
            Principal::Max => 0,
            Principal::Mid => 1,
            Principal::Min => 2,
            Principal::AbsMax => 3,
        }),
    };
    let options = &view.vector_display.lic;
    Some(LicDisplay {
        source,
        length: options.length,
        grain: options.grain as f32,
        contrast: options.contrast as f32,
        mix: options.mix as f32,
    })
}

/// The vector source of the view at every node, if it shows LIC: x, y, z of a vector or the
/// six components of a tensor, divided by the largest magnitude; zero where a value is
/// missing.
pub fn node_values(view: &ResultsView) -> Option<Vec<Directions>> {
    display(view)?;
    let source = view.vector_source()?;
    let raw = source.raw_values(view.current_increment()?)?;
    let count = raw.iter().map(|r| r.len()).min().unwrap_or(0);
    let mut values: Vec<Directions> = (0..count)
        .map(|n| {
            let mut value = [0.0; 6];
            for (slot, component) in value.iter_mut().zip(&raw) {
                *slot = component[n];
            }
            if value.iter().all(|v| v.is_finite()) {
                value
            } else {
                [0.0; 6]
            }
        })
        .collect();
    let magnitude = |v: &Directions| match source {
        VectorSource::Vector { .. } => DVec3::new(v[0] as f64, v[1] as f64, v[2] as f64).length(),
        VectorSource::Principal { .. } => {
            let values = plx_results::principal_values(v.map(f64::from));
            values.iter().fold(0.0f64, |m, p| m.max(p.abs()))
        }
    };
    let largest = values.iter().map(magnitude).fold(0.0f64, f64::max);
    if largest > 0.0 {
        let scale = (1.0 / largest) as f32;
        for value in &mut values {
            *value = value.map(|v| v * scale);
        }
    }
    Some(values)
}

/// The values on a transformed copy: vectors turn with it, tensors as R T Rᵀ.
pub fn turned(values: &[Directions], instance: &DAffine3, tensor: bool) -> Vec<Directions> {
    let r = instance.matrix3;
    if r == DMat3::IDENTITY {
        return values.to_vec();
    }
    (values.iter())
        .map(|v| {
            let v = v.map(f64::from);
            if !tensor {
                let t = r * DVec3::new(v[0], v[1], v[2]);
                return [t.x as f32, t.y as f32, t.z as f32, 0.0, 0.0, 0.0];
            }
            let [xx, yy, zz, xy, yz, zx] = v;
            let t = DMat3::from_cols(
                DVec3::new(xx, xy, zx),
                DVec3::new(xy, yy, yz),
                DVec3::new(zx, yz, zz),
            );
            let t = r * t * r.transpose();
            [
                t.x_axis.x, t.y_axis.y, t.z_axis.z, t.y_axis.x, t.z_axis.y, t.x_axis.z,
            ]
            .map(|c| c as f32)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cantilever() -> (ResultsView, plx_mesh::FeMesh) {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/kragbalken_c3d8.frd");
        let import = plx_io::frd::read_frd(&path).unwrap();
        let view = ResultsView::new(import.increments, import.mesh.bounds());
        (view, import.mesh)
    }

    fn show(view: &mut ResultsView, field: &str) {
        let fields = &view.current_increment().unwrap().fields;
        view.field = fields.iter().position(|f| f.name == field).unwrap();
        view.component = 0;
    }

    #[test]
    fn lic_takes_the_shown_source_scaled_to_one() {
        let (mut view, _) = cantilever();
        show(&mut view, "STRESS");
        assert!(display(&view).is_none() && node_values(&view).is_none());
        view.vector_display.representation = Representation::Lic;
        view.vector_display.principal = Principal::Min;
        let lic = display(&view).unwrap();
        assert_eq!(lic.source, LicSource::Principal(2));
        assert!(lic.grain > 0.0);
        let values = node_values(&view).unwrap();
        let largest = (values.iter())
            .map(|v| {
                let p = plx_results::principal_values(v.map(f64::from));
                p[0].abs().max(p[2].abs())
            })
            .fold(0.0, f64::max);
        assert!((largest - 1.0).abs() < 1e-5, "{largest}");
        assert!(
            view.legend()
                .unwrap()
                .title
                .contains("LIC: Principal Min of STRESS")
        );

        show(&mut view, "DISP");
        assert_eq!(display(&view).unwrap().source, LicSource::Vector);
        let values = node_values(&view).unwrap();
        let largest = (values.iter())
            .map(|v| DVec3::new(v[0] as f64, v[1] as f64, v[2] as f64).length())
            .fold(0.0, f64::max);
        assert!((largest - 1.0).abs() < 1e-5);
    }

    #[test]
    fn copies_turn_vectors_and_tensors() {
        let quarter = DAffine3::from_rotation_z(std::f64::consts::FRAC_PI_2);
        let vector = turned(&[[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]], &quarter, false)[0];
        assert!((vector[1] - 1.0).abs() < 1e-6 && vector[0].abs() < 1e-6);
        // Uniaxial tension along x turns into tension along y.
        let tensor = turned(&[[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]], &quarter, true)[0];
        assert!((tensor[1] - 1.0).abs() < 1e-6 && tensor[0].abs() < 1e-6);
        // Shear xy changes its sign under the quarter turn.
        let shear = turned(&[[0.0, 0.0, 0.0, 1.0, 0.0, 0.0]], &quarter, true)[0];
        assert!((shear[3] + 1.0).abs() < 1e-6);
    }
}
