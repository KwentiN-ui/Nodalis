//! The steps of an analysis from the cards that define them, with CalculiX's carry-over
//! between steps resolved: every [`Step`] holds what is in force during it.
//!
//! The reader feeds it the cards it reads; Nodalis's direct lowering of its model feeds it
//! the same cards without text in between, so that both give the same steps.
//!
//! The rules (CalculiX manual 2.23 for `OP=`, the rest observed with ccx 2.21): a step starts
//! with the boundary conditions of the step before, the loads and defined fields of the last
//! step that was no perturbation, and the output requests of the step before. The first card
//! of a kind in a step with `OP=NEW` drops what was carried of that kind. A load on a node
//! and degree of freedom (or face) that a carried load has replaces it; the step's own loads
//! add up. Output requests are replaced by the first request of their kind in a step.
//! Boundary conditions before the first step belong to the first step.

use plxs_core::step::{
    Boundary, ConcentratedFlux, ConcentratedLoad, DefinedField, DistributedFlux, DistributedLoad,
    FieldOutput, Film, HistoryOutput, Loads, Outputs, Radiate,
};
use plxs_core::{Model, Step};

/// One card of a step: a keyword with its data, typed.
#[derive(Clone, Debug, PartialEq)]
pub enum Card {
    Boundary {
        op_new: bool,
        items: Vec<Boundary>,
    },
    Cload {
        op_new: bool,
        items: Vec<ConcentratedLoad>,
    },
    Dload {
        op_new: bool,
        items: Vec<DistributedLoad>,
    },
    Cflux {
        op_new: bool,
        items: Vec<ConcentratedFlux>,
    },
    Dflux {
        op_new: bool,
        items: Vec<DistributedFlux>,
    },
    Film {
        op_new: bool,
        items: Vec<Film>,
    },
    Radiate {
        op_new: bool,
        items: Vec<Radiate>,
    },
    Temperature {
        op_new: bool,
        field: DefinedField,
    },
    NodeFile(FieldOutput),
    ElFile(FieldOutput),
    ContactFile(FieldOutput),
    NodePrint(HistoryOutput),
    ElPrint(HistoryOutput),
    ContactPrint(HistoryOutput),
}

impl Card {
    /// The keyword of the card, for messages.
    pub fn keyword(&self) -> &'static str {
        match self {
            Card::Boundary { .. } => "*BOUNDARY",
            Card::Cload { .. } => "*CLOAD",
            Card::Dload { .. } => "*DLOAD",
            Card::Cflux { .. } => "*CFLUX",
            Card::Dflux { .. } => "*DFLUX",
            Card::Film { .. } => "*FILM",
            Card::Radiate { .. } => "*RADIATE",
            Card::Temperature { .. } => "*TEMPERATURE",
            Card::NodeFile(_) => "*NODE FILE",
            Card::ElFile(_) => "*EL FILE",
            Card::ContactFile(_) => "*CONTACT FILE",
            Card::NodePrint(_) => "*NODE PRINT",
            Card::ElPrint(_) => "*EL PRINT",
            Card::ContactPrint(_) => "*CONTACT PRINT",
        }
    }
}

/// What a step starts with, see the module documentation.
#[derive(Clone, Debug, Default)]
struct Carried {
    boundaries: Vec<Boundary>,
    loads: Loads,
    fields: Vec<DefinedField>,
    outputs: Outputs,
}

/// Which kinds the current step has defined already, so that only the first card of a kind
/// decides between keeping and dropping the carried ones.
#[derive(Clone, Debug, Default)]
struct Seen {
    boundary: bool,
    cload: bool,
    dload: bool,
    cflux: bool,
    dflux: bool,
    film: bool,
    radiate: bool,
    temperature: bool,
    node_file: bool,
    el_file: bool,
    contact_file: bool,
    node_print: bool,
    el_print: bool,
    contact_print: bool,
}

/// Builds steps card by card.
#[derive(Clone, Debug, Default)]
pub struct Assembler {
    /// The step being defined.
    pub step: Option<Step>,
    /// How many steps are done.
    count: usize,
    seen: Seen,
    /// `*BOUNDARY` before the first step.
    model_boundaries: Vec<Boundary>,
    carried: Carried,
    /// How many of the current step's concentrated and distributed loads (at the front of
    /// the lists) were carried over from the previous step; a new definition replaces those
    /// at the same node and DOF or face, the step's own add up.
    carried_loads: (usize, usize),
    /// The same for `*CFLUX`, `*DFLUX`, `*FILM` and `*RADIATE`.
    carried_heat: [usize; 4],
    warnings: Vec<String>,
}

/// A card that belongs into a step came outside of one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OutsideStep;

impl Assembler {
    pub fn new() -> Self {
        Self::default()
    }

    /// Goes on after `steps` of an analysis of `model` (`*RESTART, READ`): the next step
    /// starts with what the last of them carries over.
    pub fn continuing(model: &Model, steps: &[Step]) -> Self {
        let mut assembler = Self::new();
        assembler.count = steps.len();
        if let Some(last) = steps.last() {
            assembler.carry_on(model, last);
        }
        assembler
    }

    /// Starts a step, `step` with its settings (procedure, NLGEOM, ...); it gets what the
    /// steps before carry into it.
    pub fn begin(&mut self, mut step: Step) {
        if self.count == 0 {
            self.carried.boundaries = self.model_boundaries.clone();
        }
        step.boundaries = self.carried.boundaries.clone();
        step.loads = self.carried.loads.clone();
        step.fields = self.carried.fields.clone();
        step.outputs = self.carried.outputs.clone();
        self.carried_loads = (step.loads.concentrated.len(), step.loads.distributed.len());
        self.carried_heat = [
            step.loads.concentrated_flux.len(),
            step.loads.distributed_flux.len(),
            step.loads.films.len(),
            step.loads.radiations.len(),
        ];
        self.step = Some(step);
        self.seen = Seen::default();
    }

    /// Ends the current step and returns it.
    pub fn end(&mut self, model: &Model) -> Option<Step> {
        let step = self.step.take()?;
        self.carry_on(model, &step);
        self.count += 1;
        Some(step)
    }

    /// What the assembler noticed but did not stop for (`*WARNING` lines).
    pub fn take_warnings(&mut self) -> Vec<String> {
        std::mem::take(&mut self.warnings)
    }

    /// What the next step starts with after `step`. A perturbation step does not carry its
    /// loads on; its boundaries it does. Step-time amplitudes end with the step: their loads
    /// go on with their final value.
    fn carry_on(&mut self, model: &Model, step: &Step) {
        let step_time = crate::carry::step_period(&step.procedure);
        let mut boundaries = step.boundaries.clone();
        let mut loads = step.loads.clone();
        crate::carry::freeze_step_amplitudes(model, step_time, &mut boundaries, &mut loads);
        self.carried.boundaries = boundaries;
        if !step.perturbation {
            self.carried.loads = loads;
            self.carried.fields = step.fields.clone();
        }
        self.carried.outputs = step.outputs.clone();
    }

    /// Applies a card to the current step; `model` holds the sets the loads refer to. A
    /// `*BOUNDARY` outside of a step belongs to the first step; every other card needs one.
    pub fn apply(&mut self, model: &Model, card: Card) -> Result<(), OutsideStep> {
        let sets = &model.sets;
        if let Card::Boundary { op_new, items } = card {
            match &mut self.step {
                None => {
                    if op_new {
                        self.model_boundaries.clear();
                    }
                    self.model_boundaries.extend(items);
                }
                Some(step) => {
                    if op_new && !self.seen.boundary {
                        step.boundaries.clear();
                    }
                    self.seen.boundary = true;
                    step.boundaries.extend(items);
                }
            }
            return Ok(());
        }
        let Some(step) = self.step.as_mut() else {
            return Err(OutsideStep);
        };
        let first = |seen: &mut bool| !std::mem::replace(seen, true);
        let loads = &mut step.loads;
        match card {
            Card::Boundary { .. } => unreachable!("handled above"),
            Card::Cload { op_new, items } => {
                if op_new && first(&mut self.seen.cload) {
                    self.carried_loads.0 = 0;
                    loads.concentrated.clear();
                }
                self.seen.cload = true;
                let own = loads.concentrated.split_off(self.carried_loads.0);
                crate::carry::replace_concentrated(&mut loads.concentrated, &items, sets);
                self.carried_loads.0 = loads.concentrated.len();
                loads.concentrated.extend(own);
                loads.concentrated.extend(items);
            }
            Card::Dload { op_new, mut items } => {
                if op_new && first(&mut self.seen.dload) {
                    self.carried_loads.1 = 0;
                    loads.distributed.clear();
                }
                self.seen.dload = true;
                let own = loads.distributed.split_off(self.carried_loads.1);
                let warnings =
                    crate::carry::replace_distributed(&mut loads.distributed, &mut items, sets);
                self.carried_loads.1 = loads.distributed.len();
                loads.distributed.extend(own);
                loads.distributed.extend(items);
                self.warnings.extend(warnings);
            }
            Card::Cflux { op_new, items } => {
                if op_new && first(&mut self.seen.cflux) {
                    self.carried_heat[0] = 0;
                    loads.concentrated_flux.clear();
                }
                self.seen.cflux = true;
                let own = loads.concentrated_flux.split_off(self.carried_heat[0]);
                crate::carry::replace_concentrated_flux(&mut loads.concentrated_flux, &items, sets);
                self.carried_heat[0] = loads.concentrated_flux.len();
                loads.concentrated_flux.extend(own);
                loads.concentrated_flux.extend(items);
            }
            Card::Dflux { op_new, items } => {
                if op_new && first(&mut self.seen.dflux) {
                    self.carried_heat[1] = 0;
                    loads.distributed_flux.clear();
                }
                self.seen.dflux = true;
                let own = loads.distributed_flux.split_off(self.carried_heat[1]);
                crate::carry::replace_distributed_flux(&mut loads.distributed_flux, &items, sets);
                self.carried_heat[1] = loads.distributed_flux.len();
                loads.distributed_flux.extend(own);
                loads.distributed_flux.extend(items);
            }
            Card::Film { op_new, items } => {
                if op_new && first(&mut self.seen.film) {
                    self.carried_heat[2] = 0;
                    loads.films.clear();
                }
                self.seen.film = true;
                let own = loads.films.split_off(self.carried_heat[2]);
                crate::carry::replace_films(&mut loads.films, &items, sets);
                self.carried_heat[2] = loads.films.len();
                loads.films.extend(own);
                loads.films.extend(items);
            }
            Card::Radiate { op_new, items } => {
                if op_new && first(&mut self.seen.radiate) {
                    self.carried_heat[3] = 0;
                    loads.radiations.clear();
                }
                self.seen.radiate = true;
                let own = loads.radiations.split_off(self.carried_heat[3]);
                crate::carry::replace_radiations(&mut loads.radiations, &items, sets);
                self.carried_heat[3] = loads.radiations.len();
                loads.radiations.extend(own);
                loads.radiations.extend(items);
            }
            Card::Temperature { op_new, field } => {
                if op_new && first(&mut self.seen.temperature) {
                    step.fields.clear();
                }
                self.seen.temperature = true;
                step.fields.push(field);
            }
            Card::NodeFile(output) => replace_first(
                &mut self.seen.node_file,
                &mut step.outputs.node_file,
                output,
            ),
            Card::ElFile(output) => {
                replace_first(&mut self.seen.el_file, &mut step.outputs.el_file, output)
            }
            Card::ContactFile(output) => replace_first(
                &mut self.seen.contact_file,
                &mut step.outputs.contact_file,
                output,
            ),
            Card::NodePrint(output) => replace_first(
                &mut self.seen.node_print,
                &mut step.outputs.node_print,
                output,
            ),
            Card::ElPrint(output) => {
                replace_first(&mut self.seen.el_print, &mut step.outputs.el_print, output)
            }
            Card::ContactPrint(output) => replace_first(
                &mut self.seen.contact_print,
                &mut step.outputs.contact_print,
                output,
            ),
        }
        Ok(())
    }
}

/// An output request: the first of its kind in a step replaces the carried ones.
fn replace_first<T>(seen: &mut bool, list: &mut Vec<T>, output: T) {
    if !std::mem::replace(seen, true) {
        list.clear();
    }
    list.push(output);
}
