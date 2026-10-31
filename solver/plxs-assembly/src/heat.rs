//! Assembly of the heat transfer problem: one temperature per node (DOF 11 of CalculiX,
//! numbered by the dense node index), the thermal material of every element, the
//! conductivity and capacity matrices, prescribed temperatures, and the heat loads
//! (`*CFLUX`, `*DFLUX` on faces and bodies, `*FILM`, `*RADIATE`) with their tangents.
//!
//! Conventions CalculiX follows (found by comparing with ccx on single elements, documented
//! in `docs/solver/SCHNITTSTELLEN.md`):
//! - films are integrated consistently over the face (`h ∫ N Nᵀ dA`), radiation with the
//!   temperature interpolated to the face's integration points;
//! - in steady-state steps without an amplitude, prescribed temperatures ramp from their
//!   value at the start of the step, fluxes and film coefficients ramp from zero, the sink
//!   temperature acts in full; transient steps apply everything at once (`STEP`).
//!
//! No CalculiX code was consulted.

use std::sync::Arc;

use plxs_core::material::{Material, interpolate};
use plxs_core::mesh::NodeRef;
use plxs_core::step::{FluxKind, Step};
use plxs_core::{Error, Model, Result};
use plxs_linalg::{Pattern, SparseMatrix};
use rayon::prelude::*;

use crate::{ElementTable, Spc};

/// Thermal data of the elements of an [`ElementTable`].
pub struct HeatTable {
    /// Per element (in table order), the index into `model.materials`.
    pub material: Vec<u32>,
    /// The temperature pattern: one DOF per node.
    pub pattern: Arc<Pattern>,
    /// Whether any conductivity depends on temperature (then the problem is nonlinear).
    pub temperature_dependent: bool,
}

impl HeatTable {
    pub fn new(model: &Model, table: &ElementTable) -> Result<Self> {
        let mut material = Vec::with_capacity(table.entries.len());
        let mut temperature_dependent = false;
        for element in &model.elements {
            let section = element
                .section
                .and_then(|s| model.sections.get(s as usize))
                .ok_or_else(|| {
                    Error::input("*ELEMENT", format!("element {} has no section", element.id))
                })?;
            let name = section.kind.material().ok_or_else(|| {
                Error::input(
                    "*SOLID SECTION",
                    format!("element {} needs a material", element.id),
                )
            })?;
            let index = model
                .materials
                .iter()
                .position(|m| m.name == name)
                .ok_or_else(|| Error::undefined("*SOLID SECTION", "material", name))?;
            temperature_dependent |= model.materials[index]
                .conductivity
                .as_ref()
                .is_some_and(|c| c.rows.len() > 1);
            material.push(index as u32);
        }
        let pattern = Arc::new(Pattern::from_blocks(
            model.nodes.len(),
            table.entries.iter().map(|e| e.nodes.as_slice()),
        ));
        Ok(Self {
            material,
            pattern,
            temperature_dependent,
        })
    }

    fn material<'m>(&self, model: &'m Model, element: usize) -> &'m Material {
        &model.materials[self.material[element] as usize]
    }

    /// Checks that every element has what a heat transfer step needs: a conductivity, and
    /// for transient steps a density and a specific heat.
    pub fn check(&self, model: &Model, transient: bool) -> Result<()> {
        for (index, element) in model.elements.iter().enumerate() {
            let m = self.material(model, index);
            if m.conductivity.is_none() {
                return Err(Error::input(
                    "*HEAT TRANSFER",
                    format!(
                        "material {} of element {} has no *CONDUCTIVITY",
                        m.name, element.id
                    ),
                ));
            }
            if transient && (m.density.is_none() || m.specific_heat.is_none()) {
                return Err(Error::input(
                    "*HEAT TRANSFER",
                    format!(
                        "material {} of element {} needs *DENSITY and *SPECIFIC HEAT for a transient analysis",
                        m.name, element.id
                    ),
                ));
            }
        }
        Ok(())
    }
}

/// The amplitude factor of a heat load: the named amplitude, otherwise a ramp over the step
/// (`ramp`) or the full value.
pub fn heat_factor(
    model: &Model,
    amplitude: Option<&str>,
    time: f64,
    total: f64,
    period: f64,
    ramp: bool,
) -> Option<f64> {
    match amplitude.and_then(|name| model.amplitude(name)) {
        Some(a) => Some(a.value(if a.total_time { total } else { time })),
        None if ramp && period > 0.0 => Some((time / period).clamp(0.0, 1.0)),
        None => None,
    }
}

/// Prescribed temperatures (DOF 11) at step time `time`. `start` are the temperatures at the
/// start of the step: without an amplitude, a ramped boundary goes from there to its value.
#[allow(clippy::too_many_arguments)]
pub fn temperature_spc(
    model: &Model,
    step: &Step,
    start: &[f64],
    time: f64,
    total: f64,
    period: f64,
    ramp: bool,
) -> Spc {
    let mut prescribed: Vec<(u32, f64)> = Vec::new();
    for boundary in &step.boundaries {
        if !(boundary.first_dof..=boundary.last_dof).contains(&11) {
            continue;
        }
        let amplitude = model
            .amplitude(boundary.amplitude.as_deref().unwrap_or(""))
            .map(|a| a.value(if a.total_time { total } else { time }));
        let ramp_factor = (ramp && period > 0.0).then(|| (time / period).clamp(0.0, 1.0));
        let target = boundary.value.unwrap_or(0.0);
        let nodes = match &boundary.region {
            NodeRef::Node(id) => vec![*id],
            NodeRef::Set(name) => model.sets.node_sets.get(name).cloned().unwrap_or_default(),
        };
        for node in nodes {
            let Some(index) = model.nodes.index(node) else {
                continue;
            };
            let from = start[index as usize];
            let value = match (amplitude, ramp_factor) {
                (Some(a), _) => target * a,
                (None, Some(f)) => from + (target - from) * f,
                (None, None) => target,
            };
            prescribed.push((index, value));
        }
    }
    // The last definition of a node wins.
    prescribed.reverse();
    prescribed.sort_by_key(|&(dof, _)| dof);
    prescribed.dedup_by_key(|&mut (dof, _)| dof);
    let mut spc = Spc::default();
    for (dof, value) in prescribed {
        spc.dofs.push(dof);
        spc.values.push(value);
    }
    spc
}

/// Values of a temperature table of the material at the integration points of an element.
fn at_points(
    rows: &[plxs_core::material::TemperatureValue],
    temperatures: &[f64],
) -> Option<Vec<f64>> {
    temperatures.iter().map(|&t| interpolate(rows, t)).collect()
}

/// Assembles the conductivity matrix `K(T)` and, when `capacity` is given, the consistent
/// heat capacity matrix `C(T)` for nodal temperatures `t`.
pub fn assemble_heat(
    model: &Model,
    table: &ElementTable,
    heat: &HeatTable,
    t: &[f64],
    k: &mut SparseMatrix,
    mut capacity: Option<&mut SparseMatrix>,
) -> Result<()> {
    k.fill_zero();
    if let Some(c) = capacity.as_deref_mut() {
        c.fill_zero();
    }
    let with_capacity = capacity.is_some();
    let blocks: Vec<Result<(Vec<f64>, Vec<f64>)>> = table
        .entries
        .par_iter()
        .enumerate()
        .map(|(index, entry)| {
            let formulation = table.formulation(entry);
            let coords = table.coordinates(model, entry);
            let te: Vec<f64> = entry.nodes.iter().map(|&n| t[n as usize]).collect();
            let rule = formulation.thermal_rule();
            let temperatures = formulation.material_temperatures(&rule, &te);
            let m = heat.material(model, index);
            let missing = |what: &str| {
                Error::input(
                    "*HEAT TRANSFER",
                    format!(
                        "material {} of element {} has no {what}",
                        m.name, model.elements[index].id
                    ),
                )
            };
            let conductivity = m
                .conductivity
                .as_ref()
                .and_then(|c| at_points(&c.rows, &temperatures))
                .ok_or_else(|| missing("*CONDUCTIVITY"))?;
            let rho_c = if with_capacity {
                let rho = m
                    .density
                    .as_ref()
                    .and_then(|d| at_points(&d.rows, &temperatures))
                    .ok_or_else(|| missing("*DENSITY"))?;
                let c = m
                    .specific_heat
                    .as_ref()
                    .and_then(|c| at_points(&c.rows, &temperatures))
                    .ok_or_else(|| missing("*SPECIFIC HEAT"))?;
                Some(rho.iter().zip(&c).map(|(a, b)| a * b).collect::<Vec<f64>>())
            } else {
                None
            };
            let (mut ke, mut ce) = (Vec::new(), Vec::new());
            formulation.heat_matrices(&coords, &conductivity, rho_c.as_deref(), &mut ke, &mut ce);
            Ok((ke, ce))
        })
        .collect();
    for (entry, block) in table.entries.iter().zip(blocks) {
        let (ke, ce) = block?;
        k.add_block(&entry.nodes, &ke);
        if let Some(c) = capacity.as_deref_mut() {
            c.add_block(&entry.nodes, &ce);
        }
    }
    Ok(())
}

/// The heat loads of a step at step time `time`: the external nodal heat `q(T)` for the
/// current temperatures `t` and, in `tangent`, `-∂q/∂T` of the temperature dependent loads
/// (films and radiation), which joins the conductivity matrix in Newton's method.
#[allow(clippy::too_many_arguments)]
pub fn heat_loads(
    model: &Model,
    table: &ElementTable,
    step: &Step,
    t: &[f64],
    time: f64,
    total: f64,
    period: f64,
    ramp: bool,
    tangent: &mut SparseMatrix,
) -> Result<Vec<f64>> {
    let nodes = model.nodes.len();
    let mut q = vec![0.0; nodes];
    tangent.fill_zero();
    let full = |f: Option<f64>| f.unwrap_or(1.0);

    for load in &step.loads.concentrated_flux {
        let factor = full(heat_factor(
            model,
            load.amplitude.as_deref(),
            time,
            total,
            period,
            ramp,
        ));
        let ids = match &load.region {
            NodeRef::Node(id) => vec![*id],
            NodeRef::Set(name) => model.sets.node_sets.get(name).cloned().unwrap_or_default(),
        };
        for id in ids {
            if let Some(index) = model.nodes.index(id) {
                q[index as usize] += load.value * factor;
            }
        }
    }

    // Element and face contributions: (element index, nodal heat, tangent block).
    enum Face {
        Flux(f64),
        Film { h: f64, sink: f64 },
        Radiation { coefficient: f64, sink: f64 },
    }
    let mut work: Vec<(u32, Option<u8>, Face)> = Vec::new();
    let element_list = |region: &plxs_core::mesh::ElementRef| -> Vec<u32> {
        model
            .elements_of(region)
            .into_iter()
            .filter_map(|id| model.element_index(id))
            .collect()
    };
    for load in &step.loads.distributed_flux {
        let factor = full(heat_factor(
            model,
            load.amplitude.as_deref(),
            time,
            total,
            period,
            ramp,
        ));
        for index in element_list(&load.region) {
            let face = match load.kind {
                FluxKind::Face(f) => Some(f),
                FluxKind::Body => None,
            };
            work.push((index, face, Face::Flux(load.value * factor)));
        }
    }
    for film in &step.loads.films {
        // The sink temperature acts in full unless it has an amplitude; the coefficient
        // ramps like a load.
        let sink = film.sink_temperature
            * heat_factor(model, film.amplitude.as_deref(), time, total, period, false)
                .unwrap_or(1.0);
        let h = film.coefficient
            * full(heat_factor(
                model,
                film.film_amplitude.as_deref(),
                time,
                total,
                period,
                ramp,
            ));
        for index in element_list(&film.region) {
            work.push((index, Some(film.face), Face::Film { h, sink }));
        }
    }
    if !step.loads.radiations.is_empty() {
        let (Some(zero), Some(sigma)) = (
            model.physical.absolute_zero,
            model.physical.stefan_boltzmann,
        ) else {
            return Err(Error::input(
                "*RADIATE",
                "radiation needs *PHYSICAL CONSTANTS with ABSOLUTE ZERO and STEFAN BOLTZMANN",
            ));
        };
        for radiation in &step.loads.radiations {
            if radiation.cavity {
                return Err(Error::Unsupported(
                    "*RADIATE with cavity radiation (R<n>CR) is not implemented".into(),
                ));
            }
            let sink = radiation.sink_temperature
                * heat_factor(
                    model,
                    radiation.amplitude.as_deref(),
                    time,
                    total,
                    period,
                    false,
                )
                .unwrap_or(1.0);
            let emissivity = radiation.emissivity
                * full(heat_factor(
                    model,
                    radiation.radiation_amplitude.as_deref(),
                    time,
                    total,
                    period,
                    ramp,
                ));
            for index in element_list(&radiation.region) {
                work.push((
                    index,
                    Some(radiation.face),
                    Face::Radiation {
                        coefficient: emissivity * sigma,
                        sink: sink - zero,
                    },
                ));
            }
        }
    }
    let zero = model.physical.absolute_zero.unwrap_or(0.0);

    type Contribution = (u32, Vec<f64>, Option<Vec<f64>>);
    let contributions: Vec<Result<Contribution>> = work
        .par_iter()
        .map(|(index, face, load)| {
            let entry = &table.entries[*index as usize];
            let formulation = table.formulation(entry);
            let coords = table.coordinates(model, entry);
            let n = entry.nodes.len();
            let mut qe = vec![0.0; n];
            let Some(face) = face else {
                // Body heat.
                if let Face::Flux(r) = load {
                    formulation.body_heat(&coords, *r, &mut qe);
                }
                return Ok((*index, qe, None));
            };
            if *face == 0 || *face as usize > formulation.family.faces().len() {
                return Err(Error::input(
                    "*DFLUX",
                    format!(
                        "face {face} does not exist on element {} of type {}",
                        model.elements[*index as usize].id, formulation.kind
                    ),
                ));
            }
            let mut ke = vec![0.0; n * n];
            let mut has_tangent = false;
            for p in formulation.face_points(&coords, *face) {
                let tp: f64 = p
                    .nodes
                    .iter()
                    .zip(&p.n)
                    .map(|(&a, w)| w * t[entry.nodes[a] as usize])
                    .sum();
                let (flux, slope) = match load {
                    Face::Flux(value) => (*value, 0.0),
                    Face::Film { h, sink } => (h * (sink - tp), *h),
                    Face::Radiation { coefficient, sink } => {
                        let theta = tp - zero;
                        (
                            coefficient * (sink.powi(4) - theta.powi(4)),
                            4.0 * coefficient * theta.powi(3),
                        )
                    }
                };
                for (a, &na) in p.nodes.iter().zip(&p.n) {
                    qe[*a] += na * flux * p.area;
                }
                if slope != 0.0 {
                    has_tangent = true;
                    for (a, &na) in p.nodes.iter().zip(&p.n) {
                        for (b, &nb) in p.nodes.iter().zip(&p.n) {
                            ke[a * n + b] += slope * na * nb * p.area;
                        }
                    }
                }
            }
            Ok((*index, qe, has_tangent.then_some(ke)))
        })
        .collect();
    for contribution in contributions {
        let (index, qe, ke) = contribution?;
        let entry = &table.entries[index as usize];
        for (&node, value) in entry.nodes.iter().zip(qe) {
            q[node as usize] += value;
        }
        if let Some(ke) = ke {
            tangent.add_block(&entry.nodes, &ke);
        }
    }
    Ok(q)
}
