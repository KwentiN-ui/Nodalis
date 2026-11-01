//! Temperatures in mechanical analyses: initial temperatures, `*TEMPERATURE` fields (values
//! or a previous result file), thermal strains of `*EXPANSION`, their equivalent nodal
//! forces and the thermal part of the stresses. Used by the static procedure and by the
//! coupled temperature-displacement procedures.
//!
//! Thermal strain (isotropic, CalculiX manual, `*EXPANSION`):
//! `ε_th = α(T) (T - T_zero) - α(T_0) (T_0 - T_zero)` with the initial temperature `T_0`
//! and the `ZERO` temperature of the material, both at the integration point (temperatures
//! interpolated with the shape functions). Stresses are `D (ε - ε_th)`, the equivalent
//! nodal forces `∫ Bᵀ D ε_th dV` (any textbook on thermoelasticity, e.g. Zienkiewicz &
//! Taylor vol. 1, chapter 6).
//!
//! No CalculiX code was consulted.

use std::path::Path;

use plxs_assembly::{ElementTable, HeatTable};
use plxs_core::material::interpolate;
use plxs_core::mesh::NodeRef;
use plxs_core::state::IntegrationPointField;
use plxs_core::step::{DefinedFieldKind, Step};
use plxs_core::{Error, Model, Result};
use plxs_elements::SolidElement;
use rayon::prelude::*;

/// Temperatures from `*INITIAL CONDITIONS, TYPE=TEMPERATURE` (zero elsewhere), per dense
/// node.
pub fn initial_temperatures(model: &Model) -> Vec<f64> {
    let mut t = vec![0.0; model.nodes.len()];
    for (region, value) in &model.initial.temperatures {
        for id in nodes_of(model, region) {
            if let Some(i) = model.nodes.index(id) {
                t[i as usize] = *value;
            }
        }
    }
    t
}

fn nodes_of(model: &Model, region: &NodeRef) -> Vec<u32> {
    match region {
        NodeRef::Node(id) => vec![*id],
        NodeRef::Set(name) => model.sets.node_sets.get(name).cloned().unwrap_or_default(),
    }
}

/// Nodal temperatures and, per node, the index of the `*TEMPERATURE` field that set it.
pub type DefinedTemperatures = (Vec<f64>, Vec<Option<usize>>);

/// The temperatures a mechanical step prescribes with `*TEMPERATURE` at its end: the
/// initial temperatures overwritten by the fields in force, in order, with the index of
/// the field that set each node (for its amplitude). `None` when the step has no field
/// (the temperatures stay as they are).
pub fn defined_temperatures(
    model: &Model,
    step: &Step,
    initial: &[f64],
    work_dir: &Path,
) -> Result<Option<DefinedTemperatures>> {
    if step.fields.is_empty() {
        return Ok(None);
    }
    let mut t = initial.to_vec();
    let mut source = vec![None; t.len()];
    for (index, field) in step.fields.iter().enumerate() {
        let mut set = |i: u32, value: f64| {
            t[i as usize] = value;
            source[i as usize] = Some(index);
        };
        match &field.kind {
            DefinedFieldKind::Temperature(values) => {
                for (region, value) in values {
                    for id in nodes_of(model, region) {
                        if let Some(i) = model.nodes.index(id) {
                            set(i, *value);
                        }
                    }
                }
            }
            DefinedFieldKind::TemperatureFile { file, bstep, .. } => {
                let path = work_dir.join(file);
                let text = std::fs::read(&path).map_err(|e| {
                    Error::input(
                        "*TEMPERATURE",
                        format!("cannot read the result file {}: {e}", path.display()),
                    )
                })?;
                let step_number = bstep.unwrap_or(1);
                let values = frd_temperatures(&text, step_number).ok_or_else(|| {
                    Error::input(
                        "*TEMPERATURE",
                        format!(
                            "the result file {} has no temperatures (NDTEMP) for step {step_number}",
                            path.display()
                        ),
                    )
                })?;
                for (id, value) in values {
                    if let Some(i) = model.nodes.index(id) {
                        set(i, value);
                    }
                }
            }
        }
    }
    Ok(Some((t, source)))
}

/// The temperatures of a mechanical step over its time: from those at the start of the
/// step to the defined ones, ramped (CalculiX' default for `*STATIC`) or at once, or the
/// field values times their amplitude.
pub struct StepTemperatures {
    start: Vec<f64>,
    target: Option<(Vec<f64>, Vec<Option<usize>>)>,
    ramp: bool,
}

impl StepTemperatures {
    pub fn of_step(
        model: &Model,
        step: &Step,
        state: &plxs_core::State,
        work_dir: &Path,
    ) -> Result<Self> {
        let initial = initial_temperatures(model);
        let start = match &state.temperatures {
            Some(field) if field.values.len() == model.nodes.len() => field.values.clone(),
            _ => initial.clone(),
        };
        Ok(Self {
            target: defined_temperatures(model, step, &initial, work_dir)?,
            start,
            ramp: step.default_amplitude == plxs_core::step::DefaultAmplitude::Ramp,
        })
    }

    /// Whether the step changes or carries temperatures at all.
    pub fn is_active(&self) -> bool {
        self.target.is_some() || self.start.iter().any(|&t| t != 0.0)
    }

    /// The nodal temperatures at step time `time`.
    pub fn at(&self, model: &Model, step: &Step, time: f64, total: f64, period: f64) -> Vec<f64> {
        let Some((target, source)) = &self.target else {
            return self.start.clone();
        };
        let ramp = if self.ramp && period > 0.0 {
            (time / period).clamp(0.0, 1.0)
        } else {
            1.0
        };
        target
            .iter()
            .zip(source)
            .zip(&self.start)
            .map(|((&value, field), &from)| {
                let amplitude = field
                    .and_then(|f| step.fields[f].amplitude.as_deref())
                    .and_then(|name| model.amplitude(name));
                match amplitude {
                    Some(a) => value * a.value(if a.total_time { total } else { time }),
                    None => from + (value - from) * ramp,
                }
            })
            .collect()
    }
}

/// Inserts the temperatures (`NDTEMP`, requested by `NT`) after the displacements of the
/// last written increment, where CalculiX puts them; the initial temperatures when the
/// step has none.
pub fn insert_temperatures(ctx: &mut crate::ProcedureContext<'_>, step: &Step, t: Option<&[f64]>) {
    if !crate::fields::requested_variables(step)
        .iter()
        .any(|v| v == "NT")
    {
        return;
    }
    let initial;
    let t = match t {
        Some(t) => t,
        None => {
            initial = initial_temperatures(ctx.model);
            &initial
        }
    };
    let order = crate::fields::node_order(ctx.model);
    let field = plxs_core::results::ResultField {
        name: "NDTEMP".into(),
        components: vec!["T".into()],
        nodes: order.iter().map(|&(id, _)| id).collect(),
        values: vec![order.iter().map(|&(_, i)| t[i as usize]).collect()],
    };
    let Some(last) = ctx.results.increments.last_mut() else {
        return;
    };
    let at = last
        .fields
        .iter()
        .position(|f| f.name == "DISP")
        .map_or(0, |p| p + 1);
    last.fields.insert(at, field);
}

/// The nodal temperatures of the last `NDTEMP` block of step `step` in an ASCII `.frd`
/// file (CalculiX' format, which Nodalis and plxs write). Binary files are not read.
pub fn frd_temperatures(bytes: &[u8], step: u32) -> Option<Vec<(u32, f64)>> {
    let text = std::str::from_utf8(bytes).ok()?;
    let mut block_step = 0u32;
    let mut in_block = false;
    let mut current: Vec<(u32, f64)> = Vec::new();
    let mut found: Option<Vec<(u32, f64)>> = None;
    for line in text.lines() {
        if let Some(rest) = line.strip_prefix("    1PSTEP") {
            // `1PSTEP`, set number, increment, step.
            let fields: Vec<&str> = rest.split_whitespace().collect();
            block_step = fields.get(2).and_then(|s| s.parse().ok()).unwrap_or(0);
        } else if line.starts_with(" -4") {
            in_block = line.get(5..13).map(str::trim) == Some("NDTEMP") && block_step == step;
            current.clear();
        } else if in_block && line.starts_with(" -1") {
            let id = line.get(3..13)?.trim().parse().ok()?;
            let value = line.get(13..25)?.trim().parse().ok()?;
            current.push((id, value));
        } else if in_block && line.starts_with(" -3") {
            found = Some(std::mem::take(&mut current));
            in_block = false;
        }
    }
    found
}

/// Thermal strains and their mechanical effect for the elements of a table.
pub struct ThermalStrains<'a> {
    model: &'a Model,
    table: &'a ElementTable,
    heat: &'a HeatTable,
    initial: Vec<f64>,
}

impl<'a> ThermalStrains<'a> {
    /// `None` when no material of the model expands (nothing to do).
    pub fn new(model: &'a Model, table: &'a ElementTable, heat: &'a HeatTable) -> Option<Self> {
        model
            .materials
            .iter()
            .any(|m| m.expansion.is_some())
            .then(|| Self {
                model,
                table,
                heat,
                initial: initial_temperatures(model),
            })
    }

    /// The thermal strain at the integration points of `points` (an element formulation
    /// whose rule is used) for element `index` with nodal temperatures `te` and initial
    /// temperatures `t0e`, at the element's material temperatures (the mean temperature
    /// in linear hexahedra, see `SolidElement::material_temperatures`).
    fn point_strains(
        &self,
        index: usize,
        points: &SolidElement,
        te: &[f64],
        t0e: &[f64],
    ) -> Vec<f64> {
        let material = &self.model.materials[self.heat.material[index] as usize];
        let count = points.integration_points();
        let Some(expansion) = &material.expansion else {
            return vec![0.0; count];
        };
        let at = |v: &[f64]| points.material_temperatures(&points.rule, v);
        let alpha = |temperature: f64| interpolate(&expansion.rows, temperature).unwrap_or(0.0);
        at(te)
            .into_iter()
            .zip(at(t0e))
            .map(|(tp, t0p)| {
                alpha(tp) * (tp - expansion.zero) - alpha(t0p) * (t0p - expansion.zero)
            })
            .collect()
    }

    fn element_temperatures(&self, nodes: &[u32], t: &[f64]) -> (Vec<f64>, Vec<f64>) {
        (
            nodes.iter().map(|&n| t[n as usize]).collect(),
            nodes.iter().map(|&n| self.initial[n as usize]).collect(),
        )
    }

    /// The (isotropic) thermal strain at every integration point for nodal temperatures
    /// `t`, laid out like the table's integration points.
    pub fn strains(&self, t: &[f64]) -> Vec<f64> {
        let per_element: Vec<Vec<f64>> = self
            .table
            .entries
            .par_iter()
            .enumerate()
            .map(|(index, entry)| {
                let (te, t0e) = self.element_temperatures(&entry.nodes, t);
                self.point_strains(index, self.table.formulation(entry), &te, &t0e)
            })
            .collect();
        per_element.into_iter().flatten().collect()
    }

    /// `D ε_th` at an integration point of an entry.
    fn stress(d: &[[f64; 6]; 6], e: f64) -> [f64; 6] {
        std::array::from_fn(|r| (d[r][0] + d[r][1] + d[r][2]) * e)
    }

    /// The equivalent nodal forces `∫ Bᵀ D ε_th dV` (3 per node, global DOFs) for nodal
    /// temperatures `t`. C3D20 (and CPS8, CPE8, CAX8, which expand into it) integrates them
    /// with 2×2×2 points like C3D20R (its stiffness uses 27): that is what CalculiX'
    /// reactions to a prescribed temperature show.
    pub fn load(&self, t: &[f64]) -> Vec<f64> {
        // The reduced twin of each fully integrated 20-node formulation (C3D20 and the
        // expansions of CPS8, CPE8, CAX8), built once per formulation.
        let reduced20: Vec<Option<SolidElement>> = self
            .table
            .formulations
            .iter()
            .map(|f| {
                use plxs_core::ElementType::*;
                let thickness = f.expansion().map(|x| x.thickness);
                match f.kind {
                    C3D20 => plxs_elements::solid(C3D20R),
                    CPS8 => plxs_elements::plane(CPS8R, thickness),
                    CPE8 => plxs_elements::plane(CPE8R, thickness),
                    CAX8 => plxs_elements::plane(CAX8R, thickness),
                    _ => None,
                }
            })
            .collect();
        let contributions: Vec<Vec<f64>> = self
            .table
            .entries
            .par_iter()
            .enumerate()
            .map(|(index, entry)| {
                let formulation = reduced20[entry.formulation as usize]
                    .as_ref()
                    .unwrap_or_else(|| self.table.formulation(entry));
                let (te, t0e) = self.element_temperatures(&entry.nodes, t);
                let d = self.table.material_matrix(entry);
                let sigma: Vec<[f64; 6]> = self
                    .point_strains(index, formulation, &te, &t0e)
                    .iter()
                    .map(|&e| Self::stress(d, e))
                    .collect();
                let mut fe = vec![0.0; formulation.dof_count()];
                if sigma.iter().any(|s| s.iter().any(|v| *v != 0.0)) {
                    let coords = self.table.coordinates(self.model, entry);
                    formulation.internal_force(&coords, &sigma, &mut fe);
                }
                fe
            })
            .collect();
        let mut f = vec![0.0; self.table.dofs.len()];
        for (entry, fe) in self.table.entries.iter().zip(contributions) {
            for (&dof, v) in entry.dofs.iter().zip(fe) {
                f[dof as usize] += v;
            }
        }
        f
    }

    /// Subtracts `D ε_th` from stresses computed from the total strains.
    pub fn correct_stresses(&self, strains: &[f64], stresses: &mut IntegrationPointField) {
        for (index, entry) in self.table.entries.iter().enumerate() {
            let d = self.table.material_matrix(entry);
            let start = self.table.layout.offsets[index] as usize;
            let end = self.table.layout.offsets[index + 1] as usize;
            for (p, &e) in strains.iter().enumerate().take(end).skip(start) {
                if e == 0.0 {
                    continue;
                }
                let s = Self::stress(d, e);
                for (k, v) in s.iter().enumerate() {
                    stresses.values[p * 6 + k] -= v;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::frd_temperatures;

    #[test]
    fn temperatures_of_the_last_increment_of_a_step() {
        let frd = "\
    1PSTEP                         1           1           1
 -4  NDTEMP      1    1
 -5  T           1    1    0    0
 -1         1 1.00000E+01
 -1         2 2.00000E+01
 -3
    1PSTEP                         2           2           1
 -4  NDTEMP      1    1
 -5  T           1    1    0    0
 -1         1 1.50000E+01
 -1         2 2.50000E+01
 -3
    1PSTEP                         3           1           2
 -4  NDTEMP      1    1
 -5  T           1    1    0    0
 -1         1 9.00000E+01
 -1         2 9.00000E+01
 -3
";
        assert_eq!(
            frd_temperatures(frd.as_bytes(), 1),
            Some(vec![(1, 15.0), (2, 25.0)])
        );
        assert_eq!(
            frd_temperatures(frd.as_bytes(), 2),
            Some(vec![(1, 90.0), (2, 90.0)])
        );
        assert_eq!(frd_temperatures(frd.as_bytes(), 3), None);
    }
}
