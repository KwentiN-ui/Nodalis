use crate::{Component, Field, FieldRank, TensorKind};

/// Splits the rotations of beam nodes (`D4` to `D6` of `DISP`) and the moments on them
/// (`F4` to `F6` of `FORC`), which the solver writes besides the translations and forces,
/// into fields of their own, `ROTATION` (`UR1` to `UR3`) and `MOMENT` (`M1` to `M3`): the
/// displacements stay a vector of three, so that their magnitude and the deformation scale
/// are those of the translations, and every field converts into its own unit.
pub fn split_rotations(mut field: Field) -> Vec<Field> {
    let (raw, name, renamed): ([&str; 3], &str, [&str; 3]) = match field.name.as_str() {
        "DISP" => (["D4", "D5", "D6"], "ROTATION", ["UR1", "UR2", "UR3"]),
        "FORC" => (["F4", "F5", "F6"], "MOMENT", ["M1", "M2", "M3"]),
        _ => return vec![field],
    };
    if !raw
        .iter()
        .all(|r| field.components.iter().any(|c| c.name == *r))
    {
        return vec![field];
    }
    let mut moved = Vec::new();
    for (r, n) in raw.iter().zip(renamed) {
        let at = (field.components.iter())
            .position(|c| c.name == *r)
            .unwrap();
        let mut component = field.components.remove(at);
        component.name = n.to_string();
        moved.push(component);
    }
    let mut rebuilt = Field::new(field.name.clone(), std::mem::take(&mut field.components));
    add_derived_components(&mut rebuilt);
    let mut other = Field::new(name, moved);
    add_derived_components(&mut other);
    vec![rebuilt, other]
}

/// Adds the quantities shown besides the raw components, in this order: the
/// magnitude `ALL` of vector fields first; for tensor fields the equivalent value(s) first and
/// the principal values last. Which ones follows from the field's rank.
pub fn add_derived_components(field: &mut Field) {
    field.components.retain(|c| !c.derived && c.name != "ALL");
    let base: Vec<&[f32]> = field
        .components
        .iter()
        .map(|c| c.values.as_slice())
        .collect();
    let count = base.first().map_or(0, |c| c.len());
    let derived = |name: &str, values: Vec<f32>| Component {
        name: name.to_string(),
        values,
        derived: true,
        out_of_plane: false,
    };

    if field.rank == FieldRank::Vector && base.len() == 3 {
        let all = (0..count)
            .map(|i| (base[0][i].powi(2) + base[1][i].powi(2) + base[2][i].powi(2)).sqrt())
            .collect();
        field.components.insert(0, derived("ALL", all));
        return;
    }
    let strain = match field.rank {
        FieldRank::Tensor(kind) if base.len() == 6 => kind == TensorKind::Strain,
        _ => return,
    };
    let mut equivalent = Vec::with_capacity(count);
    let mut tresca = Vec::with_capacity(count);
    let mut signed_max_abs = Vec::with_capacity(count);
    let mut principal = [(); 3].map(|_| Vec::with_capacity(count));
    for t in (0..count).map(|i| [0, 1, 2, 3, 4, 5].map(|k| base[k][i] as f64)) {
        let [xx, yy, zz, xy, yz, zx] = t;
        let mises = (0.5
            * ((xx - yy).powi(2)
                + (yy - zz).powi(2)
                + (zz - xx).powi(2)
                + 6.0 * (xy * xy + yz * yz + zx * zx)))
            .sqrt();
        let [p1, p2, p3] = principal_values(t);
        equivalent.push(if strain { mises * 2.0 / 3.0 } else { mises } as f32);
        tresca.push((p1 - p3) as f32);
        signed_max_abs.push(if p1.abs() > p3.abs() { p1 } else { p3 } as f32);
        for (column, value) in principal.iter_mut().zip([p1, p2, p3]) {
            column.push(value as f32);
        }
    }
    let [p1, p2, p3] = principal;
    if strain {
        field
            .components
            .insert(0, derived("EQUIVALENT", equivalent));
    } else {
        field.components.insert(0, derived("MISES", equivalent));
        field.components.insert(1, derived("TRESCA", tresca));
    }
    field.components.extend([
        derived("SGN_MAX_ABS_PRI", signed_max_abs),
        derived("PRINCIPAL_MAX", p1),
        derived("PRINCIPAL_MID", p2),
        derived("PRINCIPAL_MIN", p3),
    ]);
}

/// Eigenvalues of a symmetric 3 × 3 tensor given as (xx, yy, zz, xy, yz, zx), largest first.
pub fn principal_values([xx, yy, zz, xy, yz, zx]: [f64; 6]) -> [f64; 3] {
    let off = xy * xy + yz * yz + zx * zx;
    if !(xx.is_finite() && yy.is_finite() && zz.is_finite() && off.is_finite()) {
        return [f64::NAN; 3];
    }
    let mean = (xx + yy + zz) / 3.0;
    let (a, b, c) = (xx - mean, yy - mean, zz - mean);
    let p = ((a * a + b * b + c * c + 2.0 * off) / 6.0).sqrt();
    if p <= f64::EPSILON * mean.abs().max(1.0) {
        return [mean; 3];
    }
    // det((T - mean I) / p) / 2 = cos(3 phi)
    let det = a * (b * c - yz * yz) - xy * (xy * c - yz * zx) + zx * (xy * yz - b * zx);
    let r = (det / (2.0 * p * p * p)).clamp(-1.0, 1.0);
    let phi = r.acos() / 3.0;
    let third = 2.0 * std::f64::consts::PI / 3.0;
    let p1 = mean + 2.0 * p * phi.cos();
    let p3 = mean + 2.0 * p * (phi + third).cos();
    [p1, 3.0 * mean - p1 - p3, p3]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn field(name: &str, components: &[(&str, f32)]) -> Field {
        Field::new(
            name,
            components
                .iter()
                .map(|&(n, v)| Component {
                    name: n.into(),
                    values: vec![v],
                    derived: false,
                    out_of_plane: false,
                })
                .collect(),
        )
    }

    fn value(field: &Field, name: &str) -> f32 {
        field.component(name).unwrap().values[0]
    }

    #[test]
    fn displacement_magnitude() {
        let mut disp = field("DISP", &[("U1", 3.0), ("U2", 0.0), ("U3", 4.0)]);
        add_derived_components(&mut disp);
        assert_eq!(value(&disp, "ALL"), 5.0);
        assert_eq!(disp.components[0].name, "ALL");
    }

    #[test]
    fn uniaxial_stress_invariants() {
        let mut stress = field(
            "STRESS",
            &[
                ("S11", 100.0),
                ("S22", 0.0),
                ("S33", 0.0),
                ("S12", 0.0),
                ("S23", 0.0),
                ("S13", 0.0),
            ],
        );
        add_derived_components(&mut stress);
        assert!((value(&stress, "MISES") - 100.0).abs() < 1e-4);
        assert!((value(&stress, "TRESCA") - 100.0).abs() < 1e-4);
        assert!((value(&stress, "PRINCIPAL_MAX") - 100.0).abs() < 1e-4);
        assert!(value(&stress, "PRINCIPAL_MIN").abs() < 1e-4);
    }

    #[test]
    fn strain_gets_equivalent_strain_but_no_tresca() {
        let mut strain = field(
            "TOSTRAIN",
            &[
                ("E11", 3.0),
                ("E22", 0.0),
                ("E33", 0.0),
                ("E12", 0.0),
                ("E23", 0.0),
                ("E13", 0.0),
            ],
        );
        add_derived_components(&mut strain);
        assert!((value(&strain, "EQUIVALENT") - 2.0).abs() < 1e-6);
        assert!(strain.component("TRESCA").is_none());
    }

    #[test]
    fn deriving_twice_does_not_duplicate() {
        let mut disp = field("DISP", &[("U1", 1.0), ("U2", 0.0), ("U3", 0.0)]);
        add_derived_components(&mut disp);
        add_derived_components(&mut disp);
        assert_eq!(disp.components.len(), 4);
    }

    #[test]
    fn a_renamed_tensor_keeps_its_rank() {
        let mut stress = field("STRESS", &[("S11", 1.0), ("S22", 0.0), ("S33", 0.0)]);
        assert_eq!(
            stress.rank,
            FieldRank::Scalar,
            "three components are no tensor"
        );
        stress = field("STRESS", &[("S11", 1.0); 6]);
        stress.name = "S_CS".into();
        add_derived_components(&mut stress);
        assert!(stress.component("MISES").is_some());
    }

    #[test]
    fn pure_shear_principal_values() {
        let [p1, p2, p3] = principal_values([0.0, 0.0, 0.0, 50.0, 0.0, 0.0]);
        assert!((p1 - 50.0).abs() < 1e-9 && p2.abs() < 1e-9 && (p3 + 50.0).abs() < 1e-9);
    }

    #[test]
    fn principal_values_of_a_general_tensor() {
        // Eigenvalues 3, 2, 1 rotated: T = R diag(3,2,1) R^T with a 45° turn about z.
        let [p1, p2, p3] = principal_values([2.5, 2.5, 1.0, 0.5, 0.0, 0.0]);
        assert!((p1 - 3.0).abs() < 1e-9 && (p2 - 2.0).abs() < 1e-9 && (p3 - 1.0).abs() < 1e-9);
    }

    #[test]
    fn hydrostatic_state_has_equal_principal_values() {
        assert_eq!(principal_values([7.0, 7.0, 7.0, 0.0, 0.0, 0.0]), [7.0; 3]);
    }
}
