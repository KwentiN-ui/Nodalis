//! The reader proper: keyword by keyword into the IR, with the step carry-over resolved.

use std::collections::HashSet;

use plxs_core::amplitude::{Amplitude, Signal};
use plxs_core::constraints::{
    CyclicSymmetryModel, Equation, EquationTerm, Joint, RigidBody, RigidBodyNodes, Submodel, Tie,
    Transform, TransformKind,
};
use plxs_core::contact::{
    Adjust, Clearance, ContactKind, ContactPair, Friction, GapConductance, SurfaceBehavior,
    SurfaceInteraction,
};
use plxs_core::element::{Element, ElementType};
use plxs_core::material::{
    Conductivity, Creep, CreepRow, Density, Elastic, ElasticRow, Expansion, Hardening,
    Hyperelastic, HyperelasticModel, Material, Plastic, PlasticRow, SpecificHeat, TemperatureValue,
};
use plxs_core::mesh::{ElementId, ElementRef, NodeId, NodeRef, Surface, SurfaceFace};
use plxs_core::model::{Analysis, DesignVariables, RestartRead};
use plxs_core::section::{BeamProfile, Gap, Section, SectionKind, Spring, SpringStiffness};
use plxs_core::step::{
    Boundary, Buckle, ComplexFrequency, ConcentratedFlux, ConcentratedLoad, ContactChange,
    Controls, CyclicModes, DampingRatio, DefaultAmplitude, DefinedField, DefinedFieldKind,
    DesignResponse, DistributedFlux, DistributedLoad, DistributedLoadKind, Dynamic, EquationSolver,
    ExplicitControls, FieldOutput, Film, Filter, FluxKind, Follower, Frequency, HeatTransfer,
    HistoryOutput, Incrementation, Integration, ModalDamping, ModalDynamics, OutputDimension,
    Procedure, Radiate, Rayleigh, RestartWrite, Sensitivity, Static, SteadyStateDynamics, Step,
    Totals, Visco,
};
use plxs_core::{Error, normalized_name};

use crate::Reading;
use crate::lexer::{Keyword, Line, split_fields};
use crate::steps::{Assembler, Card};

/// A data line with its origin.
#[derive(Clone, Debug)]
pub(crate) struct Data<'a> {
    pub text: &'a str,
    pub file: &'a str,
    pub line: usize,
}

impl Data<'_> {
    fn fields(&self) -> Vec<&str> {
        split_fields(self.text)
    }
}

/// Keywords CalculiX knows that Nodalis does not write; they get a clear message instead
/// of `cannot be interpreted`.
const KNOWN_UNSUPPORTED: &[&str] = &[
    "*BASEMOTION",
    "*CFD",
    "*CONDUCTIVITY",
    "*CONSTRAINT",
    "*CONTACTDAMPING",
    "*CORRELATEDCONDITIONS",
    "*COUPLING",
    "*CRACKPROPAGATION",
    "*DASHPOT",
    "*DEFORMATIONPLASTICITY",
    "*DEPVAR",
    "*DISTRIBUTING",
    "*DISTRIBUTINGCOUPLING",
    "*DISTRIBUTION",
    "*DSLOAD",
    "*ELECTRICALCONDUCTIVITY",
    "*ELECTROMAGNETICS",
    "*ELEMENTOUTPUT",
    "*EQUATIONOUTPUT",
    "*FEASIBLEDIRECTION",
    "*FLUIDCONSTANTS",
    "*FLUIDSECTION",
    "*GEOMETRICCONSTRAINT",
    "*GEOMETRICTOLERANCE",
    "*GREEN",
    "*HCF",
    "*HYPERFOAM",
    "*INTERPOLATION",
    "*KINEMATIC",
    "*MAGNETICPERMEABILITY",
    "*MASS",
    "*MASSFLOW",
    "*MEMBRANESECTION",
    "*MPC",
    "*NETWORKMPC",
    "*NODALTHICKNESS",
    "*NODEOUTPUT",
    "*NODEPRINT",
    "*OBJECTIVE",
    "*ORIENTATION",
    "*OUTPUT",
    "*PRE-TENSIONSECTION",
    "*RATEDEPENDENT",
    "*REFINEMESH",
    "*RETAINEDNODALDOFS",
    "*ROBUSTDESIGN",
    "*SECTIONPRINT",
    "*SHELLSECTION",
    "*SPECIFICGASCONSTANT",
    "*SUBSTRUCTUREGENERATE",
    "*SUBSTRUCTUREMATRIXOUTPUT",
    "*TIMEPOINTS",
    "*USERMATERIAL",
    "*USERSECTION",
    "*VALUESATINFINITY",
    "*VIEWFACTOR",
];

/// Which `*FRICTION` card is expected next: one of a surface interaction or one of a
/// `*CHANGE FRICTION` in a step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FrictionTarget {
    Interaction(usize),
    Change(usize),
    None,
}

pub(crate) struct Reader {
    reading: Reading,
    material: Option<usize>,
    interaction: Option<usize>,
    friction_target: FrictionTarget,
    user_elements: HashSet<String>,
    /// The steps, with the one being read, and whether it has a procedure yet.
    steps: Assembler,
    procedure_seen: bool,
    /// Whether the current `*STEP` names its default amplitude (`AMPLITUDE=`); without it
    /// dynamic procedures apply their loads at once (CalculiX's default `STEP` for them).
    step_amplitude_given: bool,
    last_comment: Option<String>,
}

impl Reader {
    pub(crate) fn new() -> Self {
        Self {
            reading: Reading::default(),
            material: None,
            interaction: None,
            friction_target: FrictionTarget::None,
            user_elements: HashSet::new(),
            steps: Assembler::new(),
            procedure_seen: false,
            step_amplitude_given: false,
            last_comment: None,
        }
    }

    /// A reader that goes on after the steps of `base` (`*RESTART, READ`): the model is
    /// `base`'s, the steps it reads are numbered on and start with what the last step of
    /// `base` carries over, as if they followed it in one input file.
    pub(crate) fn continuing(base: Analysis) -> Self {
        let mut reader = Self::new();
        reader.reading.analysis = base;
        reader.reading.analysis.model.rebuild_indices();
        let analysis = &reader.reading.analysis;
        reader.steps = Assembler::continuing(&analysis.model, &analysis.steps);
        reader
    }

    pub(crate) fn read(mut self, lines: &[Line]) -> Result<Reading, Error> {
        let mut i = 0;
        while i < lines.len() {
            match &lines[i] {
                Line::Comment { text, .. } => {
                    if !text.is_empty() {
                        self.last_comment = Some(text.clone());
                    }
                    i += 1;
                }
                Line::Data { text, file, line } => {
                    return Err(Error::input(
                        "input",
                        format!("data line outside of a keyword: {text}"),
                    )
                    .at(file, *line));
                }
                Line::Keyword(keyword) => {
                    let mut data = Vec::new();
                    let mut j = i + 1;
                    while j < lines.len() {
                        match &lines[j] {
                            Line::Data { text, file, line } => data.push(Data {
                                text,
                                file,
                                line: *line,
                            }),
                            // A comment after the data names what follows, e.g. the
                            // step (`** Step-1 ++`).
                            Line::Comment { text, .. } => {
                                if !text.is_empty() {
                                    self.last_comment = Some(text.clone());
                                }
                            }
                            Line::Keyword(_) => break,
                        }
                        j += 1;
                    }
                    self.keyword(keyword, &data)?;
                    i = j;
                }
            }
        }
        if let Some(step) = &self.steps.step {
            return Err(Error::input(
                "*STEP",
                format!("step {} has no *END STEP", step.name),
            ));
        }
        Ok(self.reading)
    }

    fn model(&mut self) -> &mut plxs_core::Model {
        &mut self.reading.analysis.model
    }

    fn warn(&mut self, message: impl Into<String>) {
        self.reading.warnings.push(message.into());
    }

    fn keyword(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        // Cards that only make sense inside a step, and the ones only outside.
        let in_step = self.steps.step.is_some();
        match k.name.as_str() {
            "*HEADING" => {
                k.check_parameters(&[])?;
                self.model().heading = data.iter().map(|d| d.text).collect::<Vec<_>>().join("\n");
            }
            "*NODE" => self.node(k, data)?,
            "*ELEMENT" => self.element(k, data)?,
            "*NSET" => self.nset(k, data)?,
            "*ELSET" => self.elset(k, data)?,
            "*SURFACE" => self.surface(k, data)?,
            "*MATERIAL" => self.material(k)?,
            "*ELASTIC" => self.elastic(k, data)?,
            "*PLASTIC" => self.plastic(k, data)?,
            "*CREEP" => self.creep(k, data)?,
            "*HYPERELASTIC" => self.hyperelastic(k, data)?,
            "*EXPANSION" => self.expansion(k, data)?,
            "*CONDUCTIVITY" => {
                let rows = self.temperature_table(k, data)?;
                self.current_material(k)?.conductivity = Some(Conductivity { rows });
            }
            "*SPECIFICHEAT" => {
                let rows = self.temperature_table(k, data)?;
                self.current_material(k)?.specific_heat = Some(SpecificHeat { rows });
            }
            "*DENSITY" => {
                let rows = self.temperature_table(k, data)?;
                self.current_material(k)?.density = Some(Density { rows });
            }
            "*SOLIDSECTION" => self.solid_section(k, data)?,
            "*SHELLSECTION" => self.shell_section(k, data, false)?,
            "*MEMBRANESECTION" => self.shell_section(k, data, true)?,
            "*BEAMSECTION" => self.beam_section(k, data)?,
            "*USERELEMENT" => self.user_element(k)?,
            "*SPRING" => self.spring(k, data)?,
            "*GAP" => self.gap(k, data)?,
            "*EQUATION" => self.equation(k, data)?,
            "*RIGIDBODY" => self.rigid_body(k)?,
            "*JOINT" => self.joint(k, data)?,
            "*TIE" => self.tie(k, data)?,
            "*CYCLICSYMMETRYMODEL" => self.cyclic_symmetry_model(k, data)?,
            "*TRANSFORM" => self.transform(k, data)?,
            "*SUBMODEL" => self.submodel(k, data)?,
            "*NORMAL" => self.normal(k, data)?,
            "*SURFACEINTERACTION" => self.surface_interaction(k)?,
            "*SURFACEBEHAVIOR" => self.surface_behavior(k, data)?,
            "*FRICTION" => self.friction(k, data)?,
            "*GAPCONDUCTANCE" => self.gap_conductance(k, data)?,
            "*CONTACTPAIR" => self.contact_pair(k, data)?,
            "*CLEARANCE" => self.clearance(k)?,
            "*COMPRESSIONONLY" => self.compression_only(k)?,
            "*AMPLITUDE" => self.amplitude(k, data)?,
            "*INITIALCONDITIONS" => self.initial_conditions(k, data)?,
            "*PHYSICALCONSTANTS" => self.physical_constants(k)?,
            "*DESIGNVARIABLES" => self.design_variables(k, data)?,
            "*RESTART" => self.restart(k)?,
            "*STEP" => self.step_start(k)?,
            "*ENDSTEP" => self.step_end(k)?,
            "*NOANALYSIS" => self.procedure(k, Procedure::NoAnalysis)?,
            "*STATIC" => self.static_(k, data)?,
            "*VISCO" => self.visco(k, data)?,
            "*FREQUENCY" => self.frequency(k, data)?,
            "*SELECTCYCLICSYMMETRYMODES" => self.select_cyclic_modes(k)?,
            "*COMPLEXFREQUENCY" => self.complex_frequency(k, data)?,
            "*BUCKLE" => self.buckle(k, data)?,
            "*HEATTRANSFER" => self.heat_transfer(k, data, HeatKind::Heat)?,
            "*COUPLEDTEMPERATURE-DISPLACEMENT" => self.heat_transfer(k, data, HeatKind::Coupled)?,
            "*UNCOUPLEDTEMPERATURE-DISPLACEMENT" => {
                self.heat_transfer(k, data, HeatKind::Uncoupled)?
            }
            "*DYNAMIC" => self.dynamic(k, data)?,
            "*DAMPING" => self.damping(k)?,
            "*MODALDYNAMICS" => self.modal_dynamics(k, data)?,
            "*MODALDAMPING" => self.modal_damping(k, data)?,
            "*STEADYSTATEDYNAMICS" => self.steady_state_dynamics(k, data)?,
            "*SENSITIVITY" => self.sensitivity(k)?,
            "*DESIGNRESPONSE" => self.design_response(k, data)?,
            "*FILTER" => self.filter(k, data)?,
            "*OBJECTIVE" => self.objective(k, data)?,
            "*CONTROLS" => self.controls(k, data)?,
            "*BOUNDARY" => self.boundary(k, data)?,
            "*CLOAD" => self.cload(k, data)?,
            "*DLOAD" => self.dload(k, data)?,
            "*CFLUX" => self.cflux(k, data)?,
            "*DFLUX" => self.dflux(k, data)?,
            "*FILM" => self.film(k, data)?,
            "*RADIATE" => self.radiate(k, data)?,
            "*TEMPERATURE" => self.temperature(k, data)?,
            "*MODELCHANGE" => self.model_change(k, data)?,
            "*CHANGEFRICTION" => self.change_friction(k)?,
            "*NODEFILE" => self.field_output(k, data, FileKind::Node)?,
            "*ELFILE" => self.field_output(k, data, FileKind::Element)?,
            "*CONTACTFILE" => self.field_output(k, data, FileKind::Contact)?,
            "*NODEPRINT" => self.history_output(k, data, FileKind::Node)?,
            "*ELPRINT" => self.history_output(k, data, FileKind::Element)?,
            "*CONTACTPRINT" => self.history_output(k, data, FileKind::Contact)?,
            name if KNOWN_UNSUPPORTED.contains(&name) => {
                return Err(Error::Unsupported(format!(
                    "keyword {} is not supported by the solver yet",
                    k.display_name()
                )));
            }
            _ => {
                return Err(Error::UnknownKeyword {
                    keyword: k.display_name(),
                    location: None,
                }
                .at(&k.file, k.line));
            }
        }
        let _ = in_step;
        Ok(())
    }

    // ----- Mesh -------------------------------------------------------------------------

    fn node(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["NSET"])?;
        let nset = k.value("NSET").map(normalized_name);
        let mut ids = Vec::new();
        for d in data {
            let f = d.fields();
            if f.len() < 2 {
                return Err(data_error(k, d, "a node needs an id and coordinates"));
            }
            let id = parse_id(k, d, f[0])?;
            let mut c = [0.0; 3];
            for (i, v) in f[1..].iter().take(3).enumerate() {
                c[i] = parse_number(k, d, v)?;
            }
            self.model().nodes.insert(id, c);
            ids.push(id);
        }
        if let Some(nset) = nset {
            self.model().sets.add_nodes(&nset, ids);
        }
        Ok(())
    }

    fn element(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["TYPE", "ELSET"])?;
        let Some(type_name) = k.value("TYPE") else {
            return Err(k.error("no TYPE given"));
        };
        let Some(kind) = ElementType::parse(type_name) else {
            return Err(k.error(format!("element type {type_name} is not supported")));
        };
        if kind == ElementType::U1 && !self.user_elements.contains("U1") {
            return Err(k.error("element type U1 needs a *USER ELEMENT definition first"));
        }
        let elset = k.value("ELSET").map(normalized_name);
        let needed = kind.node_count() + 1;
        let mut ids = Vec::new();
        let mut pending: Vec<u32> = Vec::new();
        for d in data {
            for field in d.fields() {
                pending.push(parse_id(k, d, field)?);
            }
            if pending.len() >= needed {
                let values: Vec<u32> = pending.drain(..needed).collect();
                if !pending.is_empty() {
                    return Err(data_error(
                        k,
                        d,
                        format!(
                            "element {} of type {kind} takes {} nodes",
                            values[0],
                            kind.node_count()
                        ),
                    ));
                }
                let id = values[0];
                self.model().insert_element(Element {
                    id,
                    kind,
                    nodes: values[1..].to_vec(),
                    section: None,
                });
                ids.push(id);
            }
        }
        if !pending.is_empty() {
            return Err(k.error(format!(
                "element {} of type {kind} has {} nodes instead of {}",
                pending[0],
                pending.len() - 1,
                kind.node_count()
            )));
        }
        if let Some(elset) = elset {
            self.model().sets.add_elements(&elset, ids);
        }
        Ok(())
    }

    fn nset(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["NSET", "GENERATE"])?;
        let Some(name) = k.value("NSET").map(normalized_name) else {
            return Err(k.error("no NSET given"));
        };
        let mut ids = Vec::new();
        for d in data {
            let f = d.fields();
            if k.has("GENERATE") {
                ids.extend(generated(k, d, &f)?);
                continue;
            }
            for field in f {
                match field.parse::<NodeId>() {
                    Ok(id) => ids.push(id),
                    Err(_) => {
                        let other = normalized_name(field);
                        let Some(members) = self.model().sets.node_sets.get(&other).cloned() else {
                            return Err(Error::undefined("*NSET", "node set", other));
                        };
                        ids.extend(members);
                    }
                }
            }
        }
        self.model().sets.add_nodes(&name, ids);
        Ok(())
    }

    fn elset(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["ELSET", "GENERATE"])?;
        let Some(name) = k.value("ELSET").map(normalized_name) else {
            return Err(k.error("no ELSET given"));
        };
        let mut ids = Vec::new();
        for d in data {
            let f = d.fields();
            if k.has("GENERATE") {
                ids.extend(generated(k, d, &f)?);
                continue;
            }
            for field in f {
                match field.parse::<ElementId>() {
                    Ok(id) => ids.push(id),
                    Err(_) => {
                        let other = normalized_name(field);
                        let Some(members) = self.model().sets.element_sets.get(&other).cloned()
                        else {
                            return Err(Error::undefined("*ELSET", "element set", other));
                        };
                        ids.extend(members);
                    }
                }
            }
        }
        self.model().sets.add_elements(&name, ids);
        Ok(())
    }

    fn surface(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["NAME", "TYPE"])?;
        let Some(name) = k.value("NAME").map(normalized_name) else {
            return Err(k.error("no NAME given"));
        };
        let surface = match k.value("TYPE").unwrap_or("ELEMENT") {
            "ELEMENT" => {
                let mut faces = Vec::new();
                for d in data {
                    let f = d.fields();
                    if f.len() != 2 {
                        return Err(data_error(
                            k,
                            d,
                            "an element face needs an element or set and a face label",
                        ));
                    }
                    let face = parse_face(k, d, f[1])?;
                    match f[0].parse::<ElementId>() {
                        Ok(element) => faces.push(SurfaceFace { element, face }),
                        Err(_) => {
                            let set = normalized_name(f[0]);
                            let Some(members) = self.model().sets.element_sets.get(&set).cloned()
                            else {
                                return Err(Error::undefined("*SURFACE", "element set", set));
                            };
                            faces.extend(
                                members
                                    .into_iter()
                                    .map(|element| SurfaceFace { element, face }),
                            );
                        }
                    }
                }
                Surface::Element(faces)
            }
            "NODE" => {
                let mut nodes = Vec::new();
                for d in data {
                    for field in d.fields() {
                        match field.parse::<NodeId>() {
                            Ok(id) => nodes.push(id),
                            Err(_) => {
                                let set = normalized_name(field);
                                let Some(members) = self.model().sets.node_sets.get(&set).cloned()
                                else {
                                    return Err(Error::undefined("*SURFACE", "node set", set));
                                };
                                nodes.extend(members);
                            }
                        }
                    }
                }
                Surface::Node(nodes)
            }
            other => return Err(k.error(format!("TYPE={other} is not supported"))),
        };
        self.model().sets.surfaces.insert(name, surface);
        Ok(())
    }

    // ----- Materials --------------------------------------------------------------------

    fn material(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&["NAME"])?;
        let Some(name) = k.value("NAME").map(normalized_name) else {
            return Err(k.error("no NAME given"));
        };
        let model = self.model();
        let index = match model.materials.iter().position(|m| m.name == name) {
            Some(i) => i,
            None => {
                model.materials.push(Material {
                    name,
                    ..Default::default()
                });
                model.materials.len() - 1
            }
        };
        self.material = Some(index);
        Ok(())
    }

    fn current_material(&mut self, k: &Keyword) -> Result<&mut Material, Error> {
        let Some(index) = self.material else {
            return Err(k.error("no *MATERIAL card before it"));
        };
        Ok(&mut self.reading.analysis.model.materials[index])
    }

    fn elastic(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["TYPE"])?;
        if let Some(t) = k.value("TYPE")
            && t != "ISO"
            && t != "ISOTROPIC"
        {
            return Err(k.error(format!("TYPE={t} is not supported, only ISO")));
        }
        let mut rows = Vec::new();
        for d in data {
            let v = numbers(k, d)?;
            if v.len() < 2 {
                return Err(data_error(
                    k,
                    d,
                    "Young's modulus and Poisson's ratio needed",
                ));
            }
            rows.push(ElasticRow {
                young: v[0],
                poisson: v[1],
                temperature: v.get(2).copied().unwrap_or(0.0),
            });
        }
        if rows.is_empty() {
            return Err(k.error("no data"));
        }
        self.current_material(k)?.elastic = Some(Elastic { rows });
        Ok(())
    }

    fn plastic(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["HARDENING"])?;
        let hardening = match k.value("HARDENING").unwrap_or("ISOTROPIC") {
            "ISOTROPIC" => Hardening::Isotropic,
            "KINEMATIC" => Hardening::Kinematic,
            "COMBINED" => Hardening::Combined,
            other => return Err(k.error(format!("HARDENING={other} is not supported"))),
        };
        let mut rows = Vec::new();
        for d in data {
            let v = numbers(k, d)?;
            if v.len() < 2 {
                return Err(data_error(k, d, "stress and plastic strain needed"));
            }
            rows.push(PlasticRow {
                stress: v[0],
                plastic_strain: v[1],
                temperature: v.get(2).copied().unwrap_or(0.0),
            });
        }
        self.current_material(k)?.plastic = Some(Plastic { hardening, rows });
        Ok(())
    }

    fn creep(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["LAW"])?;
        if let Some(law) = k.value("LAW")
            && law != "NORTON"
        {
            return Err(k.error(format!("LAW={law} is not supported, only NORTON")));
        }
        let mut rows = Vec::new();
        for d in data {
            let v = numbers(k, d)?;
            if v.len() < 3 {
                return Err(data_error(k, d, "A, n and m of Norton's law needed"));
            }
            rows.push(CreepRow {
                a: v[0],
                n: v[1],
                m: v[2],
                temperature: v.get(3).copied().unwrap_or(0.0),
            });
        }
        self.current_material(k)?.creep = Some(Creep { rows });
        Ok(())
    }

    /// `*HYPERELASTIC` with one of the strain energy models; the constants of the first
    /// temperature (temperature dependent constants are not supported).
    fn hyperelastic(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[
            "NEO HOOKE",
            "MOONEY-RIVLIN",
            "POLYNOMIAL",
            "REDUCED POLYNOMIAL",
            "YEOH",
            "OGDEN",
            "ARRUDA-BOYCE",
            "N",
        ])?;
        let order = || -> Result<u8, Error> {
            match k.integer("N")?.unwrap_or(1) {
                n @ 1..=3 => Ok(n as u8),
                n => Err(k.error(format!("N={n} is not supported, only 1 to 3"))),
            }
        };
        let model = if k.has("NEO HOOKE") {
            HyperelasticModel::NeoHooke
        } else if k.has("MOONEY-RIVLIN") {
            HyperelasticModel::MooneyRivlin
        } else if k.has("REDUCED POLYNOMIAL") {
            HyperelasticModel::ReducedPolynomial(order()?)
        } else if k.has("POLYNOMIAL") {
            HyperelasticModel::Polynomial(order()?)
        } else if k.has("YEOH") {
            HyperelasticModel::Yeoh
        } else if k.has("OGDEN") {
            HyperelasticModel::Ogden(order()?)
        } else if k.has("ARRUDA-BOYCE") {
            HyperelasticModel::ArrudaBoyce
        } else {
            return Err(k.error(
                "no strain energy model given (NEO HOOKE, MOONEY-RIVLIN, POLYNOMIAL, \
                 REDUCED POLYNOMIAL, YEOH, OGDEN or ARRUDA-BOYCE)",
            ));
        };
        let mut values = Vec::new();
        for d in data {
            values.extend(numbers(k, d)?);
        }
        let count = model.constant_count();
        if values.len() < count {
            return Err(k.error(format!(
                "{count} constants expected, {} given",
                values.len()
            )));
        }
        // One temperature: the constants and an optional temperature.
        if values.len() > count + 1 {
            return Err(k.error("temperature dependent constants are not supported"));
        }
        values.truncate(count);
        self.current_material(k)?.hyperelastic = Some(Hyperelastic {
            model,
            constants: values,
        });
        Ok(())
    }

    fn expansion(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["TYPE", "ZERO"])?;
        if let Some(t) = k.value("TYPE")
            && t != "ISO"
        {
            return Err(k.error(format!("TYPE={t} is not supported, only ISO")));
        }
        let zero = k.number("ZERO")?.unwrap_or(0.0);
        let rows = self.temperature_table(k, data)?;
        self.current_material(k)?.expansion = Some(Expansion { zero, rows });
        Ok(())
    }

    /// Rows of `value, temperature` (temperature optional).
    fn temperature_table(
        &mut self,
        k: &Keyword,
        data: &[Data<'_>],
    ) -> Result<Vec<TemperatureValue>, Error> {
        k.check_parameters(&["TYPE", "ZERO"])?;
        let mut rows = Vec::new();
        for d in data {
            let v = numbers(k, d)?;
            if v.is_empty() {
                return Err(data_error(k, d, "a value is needed"));
            }
            rows.push(TemperatureValue {
                value: v[0],
                temperature: v.get(1).copied().unwrap_or(0.0),
            });
        }
        if rows.is_empty() {
            return Err(k.error("no data"));
        }
        Ok(rows)
    }

    // ----- Sections ---------------------------------------------------------------------

    fn section_names(&self, k: &Keyword) -> Result<(String, String), Error> {
        let Some(elset) = k.value("ELSET").map(normalized_name) else {
            return Err(k.error("no ELSET given"));
        };
        let Some(material) = k.value("MATERIAL").map(normalized_name) else {
            return Err(k.error("no MATERIAL given"));
        };
        Ok((elset, material))
    }

    fn solid_section(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["ELSET", "MATERIAL", "ORIENTATION"])?;
        if k.has("ORIENTATION") {
            return Err(k.error("ORIENTATION is not supported"));
        }
        let (elset, material) = self.section_names(k)?;
        let thickness = match data.first() {
            Some(d) => numbers(k, d)?.first().copied(),
            None => None,
        };
        self.model().sections.push(Section {
            elset,
            kind: SectionKind::Solid {
                material,
                thickness,
            },
        });
        Ok(())
    }

    fn shell_section(
        &mut self,
        k: &Keyword,
        data: &[Data<'_>],
        membrane: bool,
    ) -> Result<(), Error> {
        k.check_parameters(&[
            "ELSET",
            "MATERIAL",
            "OFFSET",
            "NODAL THICKNESS",
            "ORIENTATION",
            "COMPOSITE",
        ])?;
        if k.has("ORIENTATION") || k.has("COMPOSITE") {
            return Err(k.error("ORIENTATION and COMPOSITE are not supported"));
        }
        let (elset, material) = self.section_names(k)?;
        let offset = k.number("OFFSET")?.unwrap_or(0.0);
        let nodal_thickness = k.has("NODAL THICKNESS");
        let thickness = match data.first() {
            Some(d) => numbers(k, d)?.first().copied(),
            None => None,
        };
        let Some(thickness) = thickness else {
            if !nodal_thickness {
                return Err(k.error("no thickness given"));
            }
            self.model().sections.push(Section {
                elset,
                kind: SectionKind::Shell {
                    material,
                    thickness: 0.0,
                    offset,
                    nodal_thickness,
                },
            });
            return Ok(());
        };
        let kind = if membrane {
            SectionKind::Membrane {
                material,
                thickness,
                offset,
            }
        } else {
            SectionKind::Shell {
                material,
                thickness,
                offset,
                nodal_thickness,
            }
        };
        self.model().sections.push(Section { elset, kind });
        Ok(())
    }

    fn beam_section(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[
            "ELSET",
            "MATERIAL",
            "SECTION",
            "OFFSET1",
            "OFFSET2",
            "ORIENTATION",
            "NODAL THICKNESS",
        ])?;
        if k.has("ORIENTATION") || k.has("NODAL THICKNESS") {
            return Err(k.error("ORIENTATION and NODAL THICKNESS are not supported"));
        }
        let (elset, material) = self.section_names(k)?;
        let section = k.value("SECTION").unwrap_or("RECT");
        let Some(dims) = data.first() else {
            return Err(k.error("no dimensions given"));
        };
        let v = numbers(k, dims)?;
        let need = |n: usize| -> Result<(), Error> {
            if v.len() < n {
                Err(data_error(
                    k,
                    dims,
                    format!("SECTION={section} needs {n} values"),
                ))
            } else {
                Ok(())
            }
        };
        let profile = match section {
            "RECT" => {
                need(2)?;
                BeamProfile::Rect { a: v[0], b: v[1] }
            }
            "CIRC" => {
                need(1)?;
                BeamProfile::Circ {
                    a: v[0],
                    b: v.get(1).copied().unwrap_or(v[0]),
                }
            }
            "PIPE" => {
                need(2)?;
                BeamProfile::Pipe {
                    radius: v[0],
                    thickness: v[1],
                }
            }
            "BOX" => {
                need(6)?;
                BeamProfile::Box {
                    a: v[0],
                    b: v[1],
                    t1: v[2],
                    t2: v[3],
                    t3: v[4],
                    t4: v[5],
                }
            }
            "GENERAL" => {
                need(5)?;
                BeamProfile::General {
                    area: v[0],
                    i11: v[1],
                    i12: v[2],
                    i22: v[3],
                    torsion: v[4],
                    shear_area_1: v.get(5).copied(),
                    shear_area_2: v.get(6).copied(),
                }
            }
            other => return Err(k.error(format!("SECTION={other} is not supported"))),
        };
        let normal = match data.get(1) {
            Some(d) => {
                let n = numbers(k, d)?;
                if n.len() < 3 {
                    return Err(data_error(k, d, "the normal needs three components"));
                }
                [n[0], n[1], n[2]]
            }
            None => [0.0, 0.0, 1.0],
        };
        self.model().sections.push(Section {
            elset,
            kind: SectionKind::Beam {
                material,
                profile,
                normal,
                offset1: k.number("OFFSET1")?.unwrap_or(0.0),
                offset2: k.number("OFFSET2")?.unwrap_or(0.0),
            },
        });
        Ok(())
    }

    fn user_element(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&["TYPE", "NODES", "INTEGRATION POINTS", "MAXDOF"])?;
        let Some(t) = k.value("TYPE") else {
            return Err(k.error("no TYPE given"));
        };
        if t != "U1" {
            return Err(k.error(format!(
                "user element {t} is not supported, only U1 (general beam)"
            )));
        }
        if k.integer("NODES")? != Some(2) || k.integer("MAXDOF")? != Some(6) {
            return Err(k.error("U1 is a beam with NODES=2 and MAXDOF=6"));
        }
        self.user_elements.insert(t.to_string());
        Ok(())
    }

    fn spring(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["ELSET", "NONLINEAR", "ORIENTATION"])?;
        if k.has("ORIENTATION") {
            return Err(k.error("ORIENTATION is not supported"));
        }
        let Some(elset) = k.value("ELSET").map(normalized_name) else {
            return Err(k.error("no ELSET given"));
        };
        let Some(first) = data.first() else {
            return Err(k.error("no data"));
        };
        let dofs = numbers(k, first)?;
        if dofs.is_empty() || dofs.len() > 2 {
            return Err(data_error(
                k,
                first,
                "the first line gives the degree(s) of freedom",
            ));
        }
        let dof1 = dofs[0] as u8;
        let dof2 = dofs.get(1).map(|d| *d as u8).unwrap_or(dof1);
        let stiffness = if k.has("NONLINEAR") {
            let mut rows = Vec::new();
            for d in &data[1..] {
                let v = numbers(k, d)?;
                if v.len() < 2 {
                    return Err(data_error(k, d, "force and displacement needed"));
                }
                rows.push((v[0], v[1], v.get(2).copied().unwrap_or(0.0)));
            }
            SpringStiffness::Nonlinear(rows)
        } else {
            let mut rows = Vec::new();
            for d in &data[1..] {
                let v = numbers(k, d)?;
                if v.is_empty() {
                    return Err(data_error(k, d, "a stiffness is needed"));
                }
                rows.push(TemperatureValue {
                    value: v[0],
                    temperature: v.get(1).copied().unwrap_or(0.0),
                });
            }
            if rows.is_empty() {
                return Err(k.error("no stiffness given"));
            }
            SpringStiffness::Linear(rows)
        };
        self.model().sections.push(Section {
            elset,
            kind: SectionKind::Spring(Spring {
                dof1,
                dof2,
                stiffness,
                orientation: None,
            }),
        });
        Ok(())
    }

    fn gap(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["ELSET"])?;
        let Some(elset) = k.value("ELSET").map(normalized_name) else {
            return Err(k.error("no ELSET given"));
        };
        let Some(d) = data.first() else {
            return Err(k.error("no data"));
        };
        let v = numbers(k, d)?;
        if v.len() < 4 {
            return Err(data_error(k, d, "clearance and direction needed"));
        }
        self.model().sections.push(Section {
            elset,
            kind: SectionKind::Gap(Gap {
                clearance: v[0],
                direction: [v[1], v[2], v[3]],
                // Manual: field 5 is not used, field 6 the spring stiffness, field 7 the
                // tensile force at infinite clearance (not kept, it is tiny by design).
                stiffness: v.get(5).copied().filter(|&k| k > 0.0),
                friction: None,
            }),
        });
        Ok(())
    }

    // ----- Constraints ------------------------------------------------------------------

    fn equation(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[])?;
        let mut rest: &[Data<'_>] = data;
        while let Some(first) = rest.first() {
            let count = numbers(k, first)?;
            let Some(&count) = count.first() else {
                return Err(data_error(k, first, "the number of terms is needed"));
            };
            let count = count as usize;
            let mut terms = Vec::new();
            let mut i = 1;
            while terms.len() < count {
                let Some(d) = rest.get(i) else {
                    return Err(k.error(format!("{count} terms announced, {} given", terms.len())));
                };
                let f = d.fields();
                if f.len() % 3 != 0 {
                    return Err(data_error(
                        k,
                        d,
                        "terms are node, degree of freedom, coefficient",
                    ));
                }
                for t in f.chunks(3) {
                    terms.push(EquationTerm {
                        node: node_ref(t[0]),
                        dof: parse_number(k, d, t[1])? as u8,
                        coefficient: parse_number(k, d, t[2])?,
                    });
                }
                i += 1;
            }
            if terms.len() != count {
                return Err(k.error(format!("{count} terms announced, {} given", terms.len())));
            }
            self.model().constraints.equations.push(Equation { terms });
            rest = &rest[i..];
        }
        Ok(())
    }

    fn rigid_body(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&["NSET", "ELSET", "REF NODE", "ROT NODE"])?;
        let nodes = match (k.value("NSET"), k.value("ELSET")) {
            (Some(n), None) => RigidBodyNodes::NodeSet(normalized_name(n)),
            (None, Some(e)) => RigidBodyNodes::ElementSet(normalized_name(e)),
            _ => return Err(k.error("either NSET or ELSET is needed")),
        };
        let Some(reference_node) = k.integer("REF NODE")? else {
            return Err(k.error("REF NODE is needed"));
        };
        let rotation_node = k.integer("ROT NODE")?.unwrap_or(reference_node);
        self.model().constraints.rigid_bodies.push(RigidBody {
            nodes,
            reference_node,
            rotation_node,
        });
        Ok(())
    }

    /// `*JOINT, NAME=..`: the reference and rotation nodes of both ends (0 for ground), the
    /// origin with the local x and y directions, and the held relative degrees of freedom
    /// (1-3 translations, 4-6 rotations, local).
    fn joint(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["NAME"])?;
        let name = k.value("NAME").map(normalized_name).unwrap_or_default();
        let (nodes, frame, dofs) = match data {
            [nodes, frame] => (nodes, frame, None),
            [nodes, frame, dofs] => (nodes, frame, Some(dofs)),
            _ => return Err(k.error("data lines are nodes, frame and held DOFs")),
        };
        let n = numbers(k, nodes)?;
        if n.len() != 4 {
            return Err(data_error(k, nodes, "four nodes are needed (0 for ground)"));
        }
        let end = |r: f64, q: f64| (r > 0.0).then_some([r as NodeId, q as NodeId]);
        let f = numbers(k, frame)?;
        if f.len() != 9 {
            return Err(data_error(k, frame, "origin, x and y direction are needed"));
        }
        let unit = |v: [f64; 3]| {
            let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
            (l > 0.0).then(|| [v[0] / l, v[1] / l, v[2] / l])
        };
        let (Some(x), Some(y)) = (unit([f[3], f[4], f[5]]), unit([f[6], f[7], f[8]])) else {
            return Err(data_error(k, frame, "the directions must not be zero"));
        };
        let Some(z) = unit([
            x[1] * y[2] - x[2] * y[1],
            x[2] * y[0] - x[0] * y[2],
            x[0] * y[1] - x[1] * y[0],
        ]) else {
            return Err(data_error(k, frame, "x and y must not be parallel"));
        };
        let y = [
            z[1] * x[2] - z[2] * x[1],
            z[2] * x[0] - z[0] * x[2],
            z[0] * x[1] - z[1] * x[0],
        ];
        let mut fixed = [false; 6];
        if let Some(line) = dofs {
            for d in numbers(k, line)? {
                if !(1.0..=6.0).contains(&d) {
                    return Err(data_error(k, line, "degrees of freedom are 1 to 6"));
                }
                fixed[d as usize - 1] = true;
            }
        }
        self.model().constraints.joints.push(Joint {
            name,
            ends: [end(n[0], n[1]), end(n[2], n[3])],
            origin: [f[0], f[1], f[2]],
            axes: [x, y, z],
            fixed,
        });
        Ok(())
    }

    fn tie(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[
            "NAME",
            "POSITION TOLERANCE",
            "ADJUST",
            "CYCLIC SYMMETRY",
            "MULTISTAGE",
            "TIED CONTACT",
        ])?;
        let name = k.value("NAME").map(normalized_name).unwrap_or_default();
        let Some(d) = data.first() else {
            return Err(k.error("the slave and master surface are needed"));
        };
        let f = d.fields();
        if f.len() != 2 {
            return Err(data_error(k, d, "the slave and master surface are needed"));
        }
        self.model().constraints.ties.push(Tie {
            name,
            slave: normalized_name(f[0]),
            master: normalized_name(f[1]),
            position_tolerance: k.number("POSITION TOLERANCE")?,
            adjust: k.value("ADJUST") != Some("NO"),
            cyclic_symmetry: k.has("CYCLIC SYMMETRY"),
            multistage: k.has("MULTISTAGE"),
        });
        Ok(())
    }

    fn cyclic_symmetry_model(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["N", "NGRAPH", "TIE", "ELSET", "CHECK"])?;
        let Some(sectors) = k.integer("N")? else {
            return Err(k.error("N is needed"));
        };
        let Some(tie) = k.value("TIE").map(normalized_name) else {
            return Err(k.error("TIE is needed"));
        };
        let Some(d) = data.first() else {
            return Err(k.error("two points on the axis are needed"));
        };
        let v = numbers(k, d)?;
        if v.len() < 6 {
            return Err(data_error(k, d, "two points on the axis are needed"));
        }
        self.model().constraints.cyclic_symmetry = Some(CyclicSymmetryModel {
            sectors,
            ngraph: k.integer("NGRAPH")?.unwrap_or(1),
            tie,
            axis: [[v[0], v[1], v[2]], [v[3], v[4], v[5]]],
            elset: k.value("ELSET").map(normalized_name),
            check: k.value("CHECK") != Some("NO"),
        });
        Ok(())
    }

    fn transform(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["NSET", "TYPE"])?;
        let Some(nset) = k.value("NSET").map(normalized_name) else {
            return Err(k.error("no NSET given"));
        };
        let kind = match k.value("TYPE").unwrap_or("R") {
            "R" => TransformKind::Rectangular,
            "C" => TransformKind::Cylindrical,
            other => return Err(k.error(format!("TYPE={other} is not supported"))),
        };
        let Some(d) = data.first() else {
            return Err(k.error("two points are needed"));
        };
        let v = numbers(k, d)?;
        if v.len() < 6 {
            return Err(data_error(k, d, "two points are needed"));
        }
        self.model().constraints.transforms.push(Transform {
            nset,
            kind,
            a: [v[0], v[1], v[2]],
            b: [v[3], v[4], v[5]],
        });
        Ok(())
    }

    fn submodel(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["TYPE", "INPUT", "GLOBAL ELSET"])?;
        if k.value("TYPE").unwrap_or("NODE") != "NODE" {
            return Err(k.error("only TYPE=NODE is supported"));
        }
        let Some(input) = k.value("INPUT") else {
            return Err(k.error("INPUT is needed"));
        };
        let mut node_sets = Vec::new();
        for d in data {
            node_sets.extend(d.fields().into_iter().map(normalized_name));
        }
        self.model().constraints.submodel = Some(Submodel {
            input: input.to_string(),
            node_sets,
            global_elset: k.value("GLOBAL ELSET").map(normalized_name),
        });
        Ok(())
    }

    fn normal(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[])?;
        for d in data {
            let v = numbers(k, d)?;
            if v.len() < 5 {
                return Err(data_error(k, d, "element, node and normal needed"));
            }
            self.model()
                .constraints
                .normals
                .push(plxs_core::section::NodeNormal {
                    element: v[0] as ElementId,
                    node: v[1] as NodeId,
                    normal: [v[2], v[3], v[4]],
                });
        }
        Ok(())
    }

    // ----- Contact ----------------------------------------------------------------------

    fn surface_interaction(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&["NAME"])?;
        let Some(name) = k.value("NAME").map(normalized_name) else {
            return Err(k.error("no NAME given"));
        };
        let contact = &mut self.model().contact;
        let index = match contact.interactions.iter().position(|i| i.name == name) {
            Some(i) => i,
            None => {
                contact.interactions.push(SurfaceInteraction {
                    name,
                    behavior: None,
                    friction: None,
                    gap_conductance: None,
                });
                contact.interactions.len() - 1
            }
        };
        self.interaction = Some(index);
        self.friction_target = FrictionTarget::Interaction(index);
        Ok(())
    }

    fn current_interaction(&mut self, k: &Keyword) -> Result<&mut SurfaceInteraction, Error> {
        let Some(index) = self.interaction else {
            return Err(k.error("no *SURFACE INTERACTION card before it"));
        };
        Ok(&mut self.reading.analysis.model.contact.interactions[index])
    }

    fn surface_behavior(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["PRESSURE-OVERCLOSURE"])?;
        let kind = k.value("PRESSURE-OVERCLOSURE").unwrap_or("HARD");
        let first = data
            .first()
            .map(|d| numbers(k, d))
            .transpose()?
            .unwrap_or_default();
        let behavior = match kind {
            "HARD" => SurfaceBehavior::Hard,
            "LINEAR" => {
                let Some(&kk) = first.first() else {
                    return Err(k.error("the stiffness is needed"));
                };
                SurfaceBehavior::Linear {
                    k: kk,
                    sigma_inf: first.get(1).copied(),
                    c0: first.get(2).copied(),
                }
            }
            "EXPONENTIAL" => {
                if first.len() < 2 {
                    return Err(k.error("c0 and p0 are needed"));
                }
                SurfaceBehavior::Exponential {
                    c0: first[0],
                    p0: first[1],
                }
            }
            "TABULAR" => {
                let mut points = Vec::new();
                for d in data {
                    let v = numbers(k, d)?;
                    if v.len() < 2 {
                        return Err(data_error(k, d, "pressure and overclosure needed"));
                    }
                    points.push((v[0], v[1]));
                }
                SurfaceBehavior::Tabular(points)
            }
            "TIED" => {
                let Some(&kk) = first.first() else {
                    return Err(k.error("the stiffness is needed"));
                };
                SurfaceBehavior::Tied { k: kk }
            }
            other => {
                return Err(k.error(format!("PRESSURE-OVERCLOSURE={other} is not supported")));
            }
        };
        self.current_interaction(k)?.behavior = Some(behavior);
        Ok(())
    }

    fn friction(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[])?;
        let Some(d) = data.first() else {
            return Err(k.error("the friction coefficient is needed"));
        };
        let v = numbers(k, d)?;
        let Some(&coefficient) = v.first() else {
            return Err(data_error(k, d, "the friction coefficient is needed"));
        };
        let friction = Friction {
            coefficient,
            stick_slope: v.get(1).copied(),
        };
        match self.friction_target {
            FrictionTarget::Interaction(i) => {
                self.model().contact.interactions[i].friction = Some(friction);
            }
            FrictionTarget::Change(i) => {
                let Some(step) = &mut self.steps.step else {
                    return Err(k.error("*CHANGE FRICTION outside of a step"));
                };
                if let ContactChange::Friction { friction: f, .. } = &mut step.contact_changes[i] {
                    *f = friction;
                }
                self.friction_target = FrictionTarget::None;
            }
            FrictionTarget::None => {
                return Err(k.error("no *SURFACE INTERACTION or *CHANGE FRICTION card before it"));
            }
        }
        Ok(())
    }

    fn gap_conductance(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["USER"])?;
        if k.has("USER") {
            return Err(k.error("USER is not supported"));
        }
        let mut rows = Vec::new();
        for d in data {
            let v = numbers(k, d)?;
            if v.is_empty() {
                return Err(data_error(k, d, "a conductance is needed"));
            }
            rows.push((
                v[0],
                v.get(1).copied().unwrap_or(0.0),
                v.get(2).copied().unwrap_or(0.0),
            ));
        }
        let conductance = match rows.as_slice() {
            [] => return Err(k.error("no data")),
            [(c, p, t)] if *p == 0.0 && *t == 0.0 => GapConductance::Constant(*c),
            _ => GapConductance::Table(rows),
        };
        self.current_interaction(k)?.gap_conductance = Some(conductance);
        Ok(())
    }

    fn contact_pair(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["INTERACTION", "TYPE", "SMALL SLIDING", "ADJUST"])?;
        let Some(interaction) = k.value("INTERACTION").map(normalized_name) else {
            return Err(k.error("INTERACTION is needed"));
        };
        let kind = match k.value("TYPE").unwrap_or("NODETOSURFACE") {
            "NODETOSURFACE" => ContactKind::NodeToSurface,
            "SURFACETOSURFACE" => ContactKind::SurfaceToSurface,
            "MORTAR" => ContactKind::Mortar,
            "LINMORTAR" | "PGLINMORTAR" => ContactKind::LinearMortar,
            other => return Err(k.error(format!("TYPE={other} is not supported"))),
        };
        let adjust = k.value("ADJUST").map(|v| match v.parse::<f64>() {
            Ok(distance) => Adjust::Distance(distance),
            Err(_) => Adjust::NodeSet(normalized_name(v)),
        });
        let Some(d) = data.first() else {
            return Err(k.error("the slave and master surface are needed"));
        };
        let f = d.fields();
        if f.len() != 2 {
            return Err(data_error(k, d, "the slave and master surface are needed"));
        }
        self.model().contact.pairs.push(ContactPair {
            interaction,
            kind,
            small_sliding: k.has("SMALL SLIDING"),
            adjust,
            slave: normalized_name(f[0]),
            master: normalized_name(f[1]),
        });
        Ok(())
    }

    /// `*COMPRESSION ONLY, SURFACE=name[, CLEARANCE=d][, STIFFNESS=k][, FLIP]` (plxs only):
    /// see [`plxs_core::contact::CompressionOnly`].
    fn compression_only(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&["SURFACE", "CLEARANCE", "STIFFNESS", "FLIP"])?;
        let Some(surface) = k.value("SURFACE").map(normalized_name) else {
            return Err(k.error("SURFACE is needed"));
        };
        let stiffness = k.number("STIFFNESS")?;
        if stiffness.is_some_and(|s| s <= 0.0 || s.is_nan()) {
            return Err(k.error("STIFFNESS must be positive"));
        }
        self.model()
            .contact
            .compression_only
            .push(plxs_core::contact::CompressionOnly {
                surface,
                clearance: k.number("CLEARANCE")?.unwrap_or(0.0),
                stiffness,
                flipped: k.has("FLIP"),
            });
        Ok(())
    }

    fn clearance(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&["MASTER", "SLAVE", "VALUE"])?;
        let (Some(master), Some(slave), Some(value)) =
            (k.value("MASTER"), k.value("SLAVE"), k.number("VALUE")?)
        else {
            return Err(k.error("MASTER, SLAVE and VALUE are needed"));
        };
        self.model().contact.clearances.push(Clearance {
            master: normalized_name(master),
            slave: normalized_name(slave),
            value,
        });
        Ok(())
    }

    // ----- Amplitudes, initial conditions, constants ------------------------------------

    fn amplitude(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["NAME", "TIME", "SHIFTX", "SHIFTY", "USER", "SHAPE", "SWEEP"])?;
        let Some(name) = k.value("NAME").map(normalized_name) else {
            return Err(k.error("no NAME given"));
        };
        if k.has("USER") {
            return Err(k.error("USER is not supported"));
        }
        let mut values = Vec::new();
        for d in data {
            values.extend(numbers(k, d)?);
        }
        let signal = match k.value("SHAPE") {
            None | Some("TABLE") => None,
            Some(shape) => Some(signal(k, shape, &values)?),
        };
        if signal.is_none() && k.has("SWEEP") {
            return Err(k.error("SWEEP needs SHAPE=CHIRP"));
        }
        let points: Vec<(f64, f64)> = if signal.is_some() {
            Vec::new()
        } else {
            if values.len() % 2 != 0 {
                return Err(k.error("time and value come in pairs"));
            }
            values.chunks(2).map(|p| (p[0], p[1])).collect()
        };
        self.model().amplitudes.push(Amplitude {
            name,
            total_time: k.value("TIME") == Some("TOTALTIME"),
            shift_x: k.number("SHIFTX")?.unwrap_or(0.0),
            shift_y: k.number("SHIFTY")?.unwrap_or(0.0),
            points,
            signal,
        });
        Ok(())
    }

    fn initial_conditions(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["TYPE", "USER", "FILE", "STEP", "INC"])?;
        if k.has("USER") || k.has("FILE") {
            return Err(k.error("USER and FILE are not supported"));
        }
        match k.value("TYPE") {
            Some("TEMPERATURE") => {
                for d in data {
                    let f = d.fields();
                    if f.len() < 2 {
                        return Err(data_error(k, d, "node or set and temperature needed"));
                    }
                    let value = parse_number(k, d, f[1])?;
                    self.model()
                        .initial
                        .temperatures
                        .push((node_ref(f[0]), value));
                }
            }
            Some(kind @ ("VELOCITY" | "DISPLACEMENT")) => {
                for d in data {
                    let f = d.fields();
                    if f.len() < 3 {
                        return Err(data_error(
                            k,
                            d,
                            "node or set, degree of freedom and value needed",
                        ));
                    }
                    let dof = parse_number(k, d, f[1])? as u8;
                    let value = parse_number(k, d, f[2])?;
                    let entry = (node_ref(f[0]), dof, value);
                    if kind == "VELOCITY" {
                        self.model().initial.velocities.push(entry);
                    } else {
                        self.model().initial.displacements.push(entry);
                    }
                }
            }
            Some(other) => return Err(k.error(format!("TYPE={other} is not supported"))),
            None => return Err(k.error("TYPE is needed")),
        }
        Ok(())
    }

    fn physical_constants(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&["ABSOLUTE ZERO", "STEFAN BOLTZMANN", "NEWTON GRAVITY"])?;
        let physical = plxs_core::PhysicalConstants {
            absolute_zero: k.number("ABSOLUTE ZERO")?,
            stefan_boltzmann: k.number("STEFAN BOLTZMANN")?,
            newton_gravity: k.number("NEWTON GRAVITY")?,
        };
        self.model().physical = physical;
        Ok(())
    }

    fn design_variables(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["TYPE"])?;
        if k.value("TYPE") != Some("COORDINATE") {
            return Err(k.error("only TYPE=COORDINATE is supported"));
        }
        let Some(d) = data.first() else {
            return Err(k.error("the node set is needed"));
        };
        let nset = normalized_name(d.fields().first().copied().unwrap_or_default());
        self.model().design_variables = Some(DesignVariables { nset });
        Ok(())
    }

    fn restart(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&["READ", "WRITE", "STEP", "FREQUENCY", "OVERLAY"])?;
        if k.has("READ") {
            self.model().restart_read = Some(RestartRead {
                step: k.integer("STEP")?,
            });
        } else if k.has("WRITE") {
            let Some(step) = &mut self.steps.step else {
                return Err(k.error("WRITE belongs into a step"));
            };
            step.restart_write = Some(RestartWrite {
                frequency: k.integer("FREQUENCY")?.unwrap_or(1),
                overlay: k.has("OVERLAY"),
            });
        } else {
            return Err(k.error("READ or WRITE is needed"));
        }
        Ok(())
    }

    // ----- Steps ------------------------------------------------------------------------

    fn step_start(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&[
            "NLGEOM",
            "INC",
            "PERTURBATION",
            "AMPLITUDE",
            "INCF",
            "THERMAL NETWORK",
            "SHOCK SMOOTHING",
            "TURBULENCE MODEL",
        ])?;
        if self.steps.step.is_some() {
            return Err(k.error("a step inside a step"));
        }
        let number = self.reading.analysis.steps.len() as u32 + 1;
        let name = self
            .last_comment
            .take()
            .filter(|c| c.ends_with('+'))
            .map(|c| c.trim_end_matches('+').trim().to_string())
            .filter(|c| !c.is_empty())
            .unwrap_or_else(|| format!("Step-{number}"));
        let mut step = Step::new(name, Procedure::NoAnalysis);
        step.nlgeom = match k.value("NLGEOM") {
            None => k.has("NLGEOM"),
            Some("NO") => false,
            Some(_) => true,
        };
        step.perturbation = k.has("PERTURBATION");
        step.max_increments = k.integer("INC")?.unwrap_or(100);
        self.step_amplitude_given = k.has("AMPLITUDE");
        step.default_amplitude = match k.value("AMPLITUDE") {
            Some("STEP") => DefaultAmplitude::Step,
            _ => DefaultAmplitude::Ramp,
        };
        // What the step starts with.
        self.steps.begin(step);
        self.procedure_seen = false;
        self.friction_target = FrictionTarget::None;
        Ok(())
    }

    fn step_end(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&[])?;
        let Some(step) = &self.steps.step else {
            return Err(k.error("*END STEP without a step"));
        };
        if !self.procedure_seen {
            return Err(Error::input(
                "*STEP",
                format!(
                    "step {} has no procedure (*STATIC, *FREQUENCY, ...)",
                    step.name
                ),
            )
            .at(&k.file, k.line));
        }
        let analysis = &mut self.reading.analysis;
        if let Some(step) = self.steps.end(&analysis.model) {
            analysis.steps.push(step);
        }
        self.friction_target = FrictionTarget::None;
        Ok(())
    }

    /// Applies a card of a step to the step being read.
    fn apply(&mut self, k: &Keyword, card: Card) -> Result<(), Error> {
        (self.steps.apply(&self.reading.analysis.model, card))
            .map_err(|_| k.error("outside of a step"))?;
        let warnings = self.steps.take_warnings();
        self.reading.warnings.extend(warnings);
        Ok(())
    }

    fn current_step(&mut self, k: &Keyword) -> Result<&mut Step, Error> {
        match &mut self.steps.step {
            Some(step) => Ok(step),
            None => Err(k.error("outside of a step")),
        }
    }

    fn procedure(&mut self, k: &Keyword, procedure: Procedure) -> Result<(), Error> {
        if self.procedure_seen {
            return Err(k.error("the step has a procedure already"));
        }
        self.procedure_seen = true;
        self.current_step(k)?.procedure = procedure;
        Ok(())
    }

    fn static_(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[
            "SOLVER",
            "DIRECT",
            "FRAMES",
            "EXPLICIT",
            "TIME RESET",
            "TOTAL TIME AT START",
        ])?;
        if k.has("EXPLICIT") {
            return Err(k.error("EXPLICIT is not supported"));
        }
        let procedure = Procedure::Static(Static {
            solver: solver(k)?,
            increments: incrementation(k, data)?,
            time_reset: k.has("TIME RESET"),
            total_time_at_start: k.number("TOTAL TIME AT START")?,
        });
        self.procedure(k, procedure)
    }

    fn visco(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[
            "SOLVER",
            "DIRECT",
            "FRAMES",
            "CETOL",
            "TIME RESET",
            "TOTAL TIME AT START",
        ])?;
        let Some(cetol) = k.number("CETOL")? else {
            return Err(k.error("CETOL is needed"));
        };
        let procedure = Procedure::Visco(Visco {
            solver: solver(k)?,
            increments: incrementation(k, data)?,
            cetol,
        });
        self.procedure(k, procedure)
    }

    fn frequency(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[
            "SOLVER",
            "STORAGE",
            "GLOBAL",
            "CYCMPC",
            "MATRIXSTORAGE",
            "NUMBEROFEIGENVALUES",
        ])?;
        if k.has("MATRIXSTORAGE") {
            return Err(k.error("MATRIXSTORAGE is not supported"));
        }
        let v = data
            .first()
            .map(|d| numbers(k, d))
            .transpose()?
            .unwrap_or_default();
        let procedure = Procedure::Frequency(Frequency {
            solver: solver(k)?,
            storage: k.value("STORAGE") == Some("YES"),
            global: k.value("GLOBAL") != Some("NO"),
            cycmpc_active: k.value("CYCMPC") != Some("INACTIVE"),
            count: v.first().map(|c| *c as u32).unwrap_or(1),
            lower: v.get(1).copied(),
            upper: v.get(2).copied(),
            cyclic_modes: None,
        });
        self.procedure(k, procedure)
    }

    fn select_cyclic_modes(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&["NMIN", "NMAX"])?;
        let modes = CyclicModes {
            nmin: k.integer("NMIN")?,
            nmax: k.integer("NMAX")?,
        };
        match &mut self.current_step(k)?.procedure {
            Procedure::Frequency(f) => f.cyclic_modes = Some(modes),
            _ => return Err(k.error("belongs into a *FREQUENCY step")),
        }
        Ok(())
    }

    fn complex_frequency(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["CORIOLIS", "FLUID"])?;
        let v = data
            .first()
            .map(|d| numbers(k, d))
            .transpose()?
            .unwrap_or_default();
        let procedure = Procedure::ComplexFrequency(ComplexFrequency {
            coriolis: k.has("CORIOLIS"),
            fluid: k.has("FLUID"),
            count: v.first().map(|c| *c as u32).unwrap_or(1),
        });
        self.procedure(k, procedure)
    }

    fn buckle(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["SOLVER"])?;
        let v = data
            .first()
            .map(|d| numbers(k, d))
            .transpose()?
            .unwrap_or_default();
        let procedure = Procedure::Buckle(Buckle {
            solver: solver(k)?,
            count: v.first().map(|c| *c as u32).unwrap_or(1),
            accuracy: v.get(1).copied(),
        });
        self.procedure(k, procedure)
    }

    fn heat_transfer(
        &mut self,
        k: &Keyword,
        data: &[Data<'_>],
        kind: HeatKind,
    ) -> Result<(), Error> {
        k.check_parameters(&[
            "SOLVER",
            "DIRECT",
            "FRAMES",
            "STEADY STATE",
            "DELTMX",
            "FREQUENCY",
            "MODAL DYNAMIC",
            "TIME RESET",
            "TOTAL TIME AT START",
            "COMPRESSIBLE",
            "SHOCK SMOOTHING",
            "TURBULENCE MODEL",
        ])?;
        if k.has("COMPRESSIBLE") || k.has("TURBULENCE MODEL") || k.has("MODAL DYNAMIC") {
            return Err(k.error("fluid parameters are not supported"));
        }
        let settings = HeatTransfer {
            solver: solver(k)?,
            steady_state: k.has("STEADY STATE"),
            increments: incrementation(k, data)?,
            deltmx: k.number("DELTMX")?,
            frequency_count: k.integer("FREQUENCY")?,
            direct: k.has("DIRECT"),
        };
        let procedure = match kind {
            HeatKind::Heat => Procedure::HeatTransfer(settings),
            HeatKind::Coupled => Procedure::CoupledTemperatureDisplacement(settings),
            HeatKind::Uncoupled => Procedure::UncoupledTemperatureDisplacement(settings),
        };
        self.procedure(k, procedure)
    }

    fn dynamic(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[
            "SOLVER",
            "DIRECT",
            "ALPHA",
            "EXPLICIT",
            "RELATIVE TO ABSOLUTE",
            "TIME RESET",
            "TOTAL TIME AT START",
            "EVENTS",
            "INTEGRATION",
            "MASS SCALING",
            "TIME STEP",
            "SWITCH",
            "SHELL INERTIA",
        ])?;
        let full_shell_inertia = match k.value("SHELL INERTIA").map(|t| t.to_ascii_uppercase()) {
            None => false,
            Some(t) if t == "FULL" => true,
            Some(t) if t == "SCALED" => false,
            Some(other) => {
                return Err(Error::input(
                    "*DYNAMIC",
                    format!("SHELL INERTIA={other}: expected FULL or SCALED"),
                ));
            }
        };
        let controls = ExplicitControls {
            mass_scaling: k.number("MASS SCALING")?,
            time_step: k.number("TIME STEP")?,
            full_shell_inertia,
        };
        let integration = match k.value("INTEGRATION").map(|t| t.to_ascii_uppercase()) {
            None if k.has("EXPLICIT") => Integration::Explicit(controls),
            None => Integration::Implicit,
            Some(t) if t == "IMPLICIT" => Integration::Implicit,
            Some(t) if t == "EXPLICIT" => Integration::Explicit(controls),
            Some(t) if t == "AUTOMATIC" => Integration::Automatic {
                explicit: controls,
                switch_factor: k
                    .number("SWITCH")?
                    .unwrap_or(Integration::DEFAULT_SWITCH_FACTOR),
            },
            Some(other) => {
                return Err(k.error(format!(
                    "INTEGRATION={other}: expected IMPLICIT, EXPLICIT or AUTOMATIC"
                )));
            }
        };
        let procedure = Procedure::Dynamic(Dynamic {
            solver: solver(k)?,
            alpha: k.number("ALPHA")?,
            integration,
            increments: incrementation(k, data)?,
            rayleigh: None,
            damping_ratio: None,
            event_frames: k.has("EVENTS"),
        });
        // CalculiX applies the loads of a dynamic step at once unless the step asks for a
        // ramp (observed with ccx 2.21: `*STEP` without `AMPLITUDE=` gives the same
        // results as `AMPLITUDE=STEP`).
        if !self.step_amplitude_given {
            self.current_step(k)?.default_amplitude = DefaultAmplitude::Step;
        }
        self.procedure(k, procedure)
    }

    fn damping(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&["ALPHA", "BETA", "STRUCTURAL", "RATIO", "LOWER", "UPPER"])?;
        if let Some(ratio) = k.number("RATIO")? {
            let upper = (k.number("UPPER")?).ok_or_else(|| k.error("RATIO needs UPPER"))?;
            let damping = DampingRatio {
                ratio,
                lower: k.number("LOWER")?,
                upper,
            };
            match &mut self.current_step(k)?.procedure {
                Procedure::Dynamic(d) => d.damping_ratio = Some(damping),
                _ => self.warn("*DAMPING outside of a *DYNAMIC step is ignored".to_string()),
            }
            return Ok(());
        }
        let rayleigh = Rayleigh {
            alpha: k.number("ALPHA")?.unwrap_or(0.0),
            beta: k.number("BETA")?.unwrap_or(0.0),
        };
        match &mut self.current_step(k)?.procedure {
            Procedure::Dynamic(d) => d.rayleigh = Some(rayleigh),
            _ => self.warn("*DAMPING outside of a *DYNAMIC step is ignored".to_string()),
        }
        Ok(())
    }

    fn modal_dynamics(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["SOLVER", "STEADY STATE", "DELTMX"])?;
        let v = data
            .first()
            .map(|d| numbers(k, d))
            .transpose()?
            .unwrap_or_default();
        if v.len() < 2 {
            return Err(k.error("time increment and period are needed"));
        }
        let procedure = Procedure::ModalDynamics(ModalDynamics {
            solver: solver(k)?,
            steady_state: k.has("STEADY STATE"),
            increment: v[0],
            period: v[1],
            damping: None,
        });
        // Like *DYNAMIC: the loads act at once without AMPLITUDE= (ccx 2.21).
        if !self.step_amplitude_given {
            self.current_step(k)?.default_amplitude = DefaultAmplitude::Step;
        }
        self.procedure(k, procedure)
    }

    fn modal_damping(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["RAYLEIGH", "MODAL"])?;
        let damping = if k.has("RAYLEIGH") {
            let Some(d) = data.first() else {
                return Err(k.error("alpha and beta are needed"));
            };
            let f = d.fields();
            if f.len() < 4 {
                return Err(data_error(k, d, "the data line is ` , , alpha, beta`"));
            }
            ModalDamping::Rayleigh {
                alpha: parse_number(k, d, f[2])?,
                beta: parse_number(k, d, f[3])?,
            }
        } else {
            let mut rows = Vec::new();
            for d in data {
                let v = numbers(k, d)?;
                if v.len() < 3 {
                    return Err(data_error(k, d, "first mode, last mode and ratio needed"));
                }
                rows.push((v[0] as u32, v[1] as u32, v[2]));
            }
            ModalDamping::Direct(rows)
        };
        match &mut self.current_step(k)?.procedure {
            Procedure::ModalDynamics(m) => m.damping = Some(damping),
            Procedure::SteadyStateDynamics(s) => s.damping = Some(damping),
            _ => {
                return Err(
                    k.error("belongs into a *MODAL DYNAMICS or *STEADY STATE DYNAMICS step")
                );
            }
        }
        Ok(())
    }

    fn steady_state_dynamics(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["SOLVER", "HARMONIC"])?;
        let v = data
            .first()
            .map(|d| numbers(k, d))
            .transpose()?
            .unwrap_or_default();
        if v.len() < 2 {
            return Err(k.error("the frequency range is needed"));
        }
        let procedure = Procedure::SteadyStateDynamics(SteadyStateDynamics {
            solver: solver(k)?,
            harmonic: k.value("HARMONIC") != Some("NO"),
            lower: v[0],
            upper: v[1],
            points: v.get(2).map(|p| *p as u32).unwrap_or(20),
            bias: v.get(3).copied().unwrap_or(3.0),
            fourier_terms: v.get(4).map(|n| *n as u32),
            damping: None,
        });
        self.procedure(k, procedure)
    }

    fn sensitivity(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&["NLGEOM", "READ", "WRITE"])?;
        let procedure = Procedure::Sensitivity(Sensitivity {
            nlgeom: k.has("NLGEOM"),
            responses: Vec::new(),
            filter: None,
            objectives: Vec::new(),
        });
        self.procedure(k, procedure)
    }

    fn current_sensitivity(&mut self, k: &Keyword) -> Result<&mut Sensitivity, Error> {
        match &mut self.current_step(k)?.procedure {
            Procedure::Sensitivity(s) => Ok(s),
            _ => Err(k.error("belongs into a *SENSITIVITY step")),
        }
    }

    fn design_response(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["NAME"])?;
        let name = k.value("NAME").map(normalized_name).unwrap_or_default();
        let Some(d) = data.first() else {
            return Err(k.error("the response is needed"));
        };
        let f = d.fields();
        let kind = normalized_name(f[0]);
        let elset = f
            .get(1)
            .filter(|s| !s.is_empty())
            .map(|s| normalized_name(s));
        let mut values = Vec::new();
        for v in f.iter().skip(2) {
            values.push(parse_number(k, d, v)?);
        }
        self.current_sensitivity(k)?.responses.push(DesignResponse {
            name,
            kind,
            elset,
            data: values,
        });
        Ok(())
    }

    fn filter(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[
            "TYPE",
            "BOUNDARY WEIGHTING",
            "EDGE PRESERVATION",
            "DIRECTION WEIGHTING",
        ])?;
        let kind = k.value("TYPE").unwrap_or("LINEAR").to_string();
        let Some(d) = data.first() else {
            return Err(k.error("the radius is needed"));
        };
        let radius = numbers(k, d)?.first().copied().unwrap_or(0.0);
        self.current_sensitivity(k)?.filter = Some(Filter { kind, radius });
        Ok(())
    }

    fn objective(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["TARGET"])?;
        let names: Vec<String> = data
            .iter()
            .flat_map(|d| {
                d.fields()
                    .into_iter()
                    .map(normalized_name)
                    .collect::<Vec<_>>()
            })
            .collect();
        self.current_sensitivity(k)?.objectives.extend(names);
        Ok(())
    }

    fn controls(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["PARAMETERS", "RESET"])?;
        let parameters = k.value("PARAMETERS").unwrap_or("RESET").to_string();
        let mut values = Vec::new();
        for d in data {
            for f in d.fields() {
                values.push(if f.is_empty() {
                    f64::NAN
                } else {
                    parse_number(k, d, f)?
                });
            }
        }
        self.current_step(k)?.controls = Some(Controls { parameters, values });
        Ok(())
    }

    // ----- Boundary conditions and loads --------------------------------------------------

    fn boundary(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[
            "OP",
            "AMPLITUDE",
            "FIXED",
            "SUBMODEL",
            "STEP",
            "LOAD CASE",
            "MASS FLOW",
            "USER",
            "TIME DELAY",
        ])?;
        if k.has("MASS FLOW") || k.has("USER") || k.has("TIME DELAY") {
            return Err(k.error("MASS FLOW, USER and TIME DELAY are not supported"));
        }
        let amplitude = k.value("AMPLITUDE").map(normalized_name);
        let fixed = k.has("FIXED");
        let submodel_step = if k.has("SUBMODEL") {
            Some(k.integer("STEP")?.unwrap_or(1))
        } else {
            None
        };
        let load_case = k.integer("LOAD CASE")?.map(|c| c as u8);
        let mut new = Vec::new();
        for d in data {
            let f = d.fields();
            if f.len() < 2 {
                return Err(data_error(k, d, "node or set and degree of freedom needed"));
            }
            let first_dof = parse_number(k, d, f[1])? as u8;
            let last_dof = match f.get(2) {
                Some(v) if !v.is_empty() => parse_number(k, d, v)? as u8,
                _ => first_dof,
            };
            let value = match f.get(3) {
                Some(v) if !v.is_empty() => Some(parse_number(k, d, v)?),
                _ => None,
            };
            new.push(Boundary {
                region: node_ref(f[0]),
                first_dof,
                last_dof: last_dof.max(first_dof),
                value,
                amplitude: amplitude.clone(),
                fixed,
                submodel_step,
                load_case,
            });
        }
        let op_new = k.value("OP") == Some("NEW");
        self.apply(k, Card::Boundary { op_new, items: new })
    }

    fn cload(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[
            "OP",
            "AMPLITUDE",
            "LOAD CASE",
            "SECTOR",
            "TIME DELAY",
            "USER",
            "OMEGA0",
            "FOLLOWER",
            "FOLLOWER ROTATION",
        ])?;
        if k.has("USER") || k.has("TIME DELAY") || k.has("OMEGA0") {
            return Err(k.error("USER, TIME DELAY and OMEGA0 are not supported"));
        }
        let follower = match (k.value("FOLLOWER"), k.value("FOLLOWER ROTATION")) {
            (Some(_), Some(_)) => {
                return Err(k.error("FOLLOWER and FOLLOWER ROTATION exclude each other"));
            }
            (Some(nodes), None) => Some(Follower::Nodes(node_ref(nodes))),
            (None, Some(node)) => match node.trim().parse::<NodeId>() {
                Ok(id) => Some(Follower::Rotation(id)),
                Err(_) => return Err(k.error("FOLLOWER ROTATION needs a node number")),
            },
            (None, None) if k.has("FOLLOWER") || k.has("FOLLOWER ROTATION") => {
                return Err(k.error("FOLLOWER needs the nodes the load turns with"));
            }
            (None, None) => None,
        };
        let amplitude = k.value("AMPLITUDE").map(normalized_name);
        let load_case = k.integer("LOAD CASE")?.map(|c| c as u8);
        let sector = k.integer("SECTOR")?;
        let mut new = Vec::new();
        for d in data {
            let f = d.fields();
            if f.len() < 3 {
                return Err(data_error(
                    k,
                    d,
                    "node or set, degree of freedom and value needed",
                ));
            }
            new.push(ConcentratedLoad {
                region: node_ref(f[0]),
                dof: parse_number(k, d, f[1])? as u8,
                value: parse_number(k, d, f[2])?,
                amplitude: amplitude.clone(),
                load_case,
                sector,
                follower: follower.clone(),
            });
        }
        let op_new = k.value("OP") == Some("NEW");
        self.apply(k, Card::Cload { op_new, items: new })
    }

    fn dload(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["OP", "AMPLITUDE", "LOAD CASE", "SECTOR", "TIME DELAY"])?;
        if k.has("TIME DELAY") {
            return Err(k.error("TIME DELAY is not supported"));
        }
        let amplitude = k.value("AMPLITUDE").map(normalized_name);
        let load_case = k.integer("LOAD CASE")?.map(|c| c as u8);
        let sector = k.integer("SECTOR")?;
        let mut new = Vec::new();
        for d in data {
            let f = d.fields();
            if f.len() < 3 {
                return Err(data_error(k, d, "element or set, label and value needed"));
            }
            let label = f[1].to_ascii_uppercase();
            let mut values = Vec::new();
            for v in &f[2..] {
                values.push(parse_number(k, d, v)?);
            }
            let kind =
                if let Some(face) = label.strip_prefix('P').and_then(|n| n.parse::<u8>().ok()) {
                    DistributedLoadKind::Pressure {
                        face,
                        value: values[0],
                    }
                } else if label == "GRAV" {
                    if values.len() < 4 {
                        return Err(data_error(
                            k,
                            d,
                            "GRAV needs the magnitude and the direction",
                        ));
                    }
                    DistributedLoadKind::Gravity {
                        magnitude: values[0],
                        direction: [values[1], values[2], values[3]],
                    }
                } else if label == "CENTRIF" {
                    if values.len() < 7 {
                        return Err(data_error(
                            k,
                            d,
                            "CENTRIF needs omega squared, a point and the axis",
                        ));
                    }
                    DistributedLoadKind::Centrifugal {
                        omega_squared: values[0],
                        point: [values[1], values[2], values[3]],
                        axis: [values[4], values[5], values[6]],
                    }
                } else {
                    DistributedLoadKind::Other { label, values }
                };
            new.push(DistributedLoad {
                region: element_ref(f[0]),
                kind,
                amplitude: amplitude.clone(),
                load_case,
                sector,
            });
        }
        let op_new = k.value("OP") == Some("NEW");
        self.apply(k, Card::Dload { op_new, items: new })
    }

    fn cflux(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["OP", "AMPLITUDE", "TIME DELAY", "USER"])?;
        let amplitude = k.value("AMPLITUDE").map(normalized_name);
        let mut new = Vec::new();
        for d in data {
            let f = d.fields();
            if f.len() < 3 {
                return Err(data_error(k, d, "node or set, 11 and value needed"));
            }
            new.push(ConcentratedFlux {
                region: node_ref(f[0]),
                value: parse_number(k, d, f[2])?,
                amplitude: amplitude.clone(),
            });
        }
        let op_new = k.value("OP") == Some("NEW");
        self.apply(k, Card::Cflux { op_new, items: new })
    }

    fn dflux(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["OP", "AMPLITUDE", "TIME DELAY"])?;
        let amplitude = k.value("AMPLITUDE").map(normalized_name);
        let mut new = Vec::new();
        for d in data {
            let f = d.fields();
            if f.len() < 3 {
                return Err(data_error(k, d, "element or set, label and value needed"));
            }
            let label = f[1].to_ascii_uppercase();
            let kind = if label == "BF" {
                FluxKind::Body
            } else if let Some(face) = label.strip_prefix('S').and_then(|n| n.parse::<u8>().ok()) {
                FluxKind::Face(face)
            } else {
                return Err(data_error(k, d, format!("label {label} is not supported")));
            };
            new.push(DistributedFlux {
                region: element_ref(f[0]),
                kind,
                value: parse_number(k, d, f[2])?,
                amplitude: amplitude.clone(),
            });
        }
        let op_new = k.value("OP") == Some("NEW");
        self.apply(k, Card::Dflux { op_new, items: new })
    }

    fn film(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[
            "OP",
            "AMPLITUDE",
            "FILM AMPLITUDE",
            "TIME DELAY",
            "FILM TIME DELAY",
        ])?;
        let amplitude = k.value("AMPLITUDE").map(normalized_name);
        let film_amplitude = k.value("FILM AMPLITUDE").map(normalized_name);
        let mut new = Vec::new();
        for d in data {
            let f = d.fields();
            if f.len() < 4 {
                return Err(data_error(
                    k,
                    d,
                    "element or set, label, sink temperature and coefficient needed",
                ));
            }
            let face = parse_face_label(k, d, f[1], 'F')?;
            new.push(Film {
                region: element_ref(f[0]),
                face,
                sink_temperature: parse_number(k, d, f[2])?,
                coefficient: parse_number(k, d, f[3])?,
                amplitude: amplitude.clone(),
                film_amplitude: film_amplitude.clone(),
            });
        }
        let op_new = k.value("OP") == Some("NEW");
        self.apply(k, Card::Film { op_new, items: new })
    }

    fn radiate(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[
            "OP",
            "AMPLITUDE",
            "RADIATION AMPLITUDE",
            "ENVNODE",
            "CAVITY",
            "TIME DELAY",
            "RADIATION TIME DELAY",
        ])?;
        if k.has("ENVNODE") {
            return Err(k.error("ENVNODE is not supported"));
        }
        let amplitude = k.value("AMPLITUDE").map(normalized_name);
        let radiation_amplitude = k.value("RADIATION AMPLITUDE").map(normalized_name);
        let mut new = Vec::new();
        for d in data {
            let f = d.fields();
            if f.len() < 4 {
                return Err(data_error(
                    k,
                    d,
                    "element or set, label, sink temperature and emissivity needed",
                ));
            }
            let label = f[1].to_ascii_uppercase();
            let cavity = label.ends_with("CR");
            let face = parse_face_label(k, d, label.trim_end_matches("CR"), 'R')?;
            new.push(Radiate {
                region: element_ref(f[0]),
                face,
                cavity,
                sink_temperature: parse_number(k, d, f[2])?,
                emissivity: parse_number(k, d, f[3])?,
                amplitude: amplitude.clone(),
                radiation_amplitude: radiation_amplitude.clone(),
            });
        }
        let op_new = k.value("OP") == Some("NEW");
        self.apply(k, Card::Radiate { op_new, items: new })
    }

    fn temperature(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&[
            "OP",
            "AMPLITUDE",
            "FILE",
            "BSTEP",
            "ESTEP",
            "BINC",
            "EINC",
            "TIME DELAY",
            "USER",
        ])?;
        let amplitude = k.value("AMPLITUDE").map(normalized_name);
        let kind = match k.value("FILE") {
            Some(file) => DefinedFieldKind::TemperatureFile {
                file: file.to_string(),
                bstep: k.integer("BSTEP")?,
                estep: k.integer("ESTEP")?,
                binc: k.integer("BINC")?,
                einc: k.integer("EINC")?,
            },
            None => {
                let mut values = Vec::new();
                for d in data {
                    let f = d.fields();
                    if f.len() < 2 {
                        return Err(data_error(k, d, "node or set and temperature needed"));
                    }
                    values.push((node_ref(f[0]), parse_number(k, d, f[1])?));
                }
                DefinedFieldKind::Temperature(values)
            }
        };
        let op_new = k.value("OP") == Some("NEW");
        let field = DefinedField { kind, amplitude };
        self.apply(k, Card::Temperature { op_new, field })
    }

    fn model_change(&mut self, k: &Keyword, data: &[Data<'_>]) -> Result<(), Error> {
        k.check_parameters(&["TYPE", "ADD", "REMOVE", "MECHSTRAINTORESIDUAL"])?;
        let add = match (k.has("ADD"), k.has("REMOVE")) {
            (true, false) => true,
            (false, true) => false,
            _ => return Err(k.error("ADD or REMOVE is needed")),
        };
        let change = match k.value("TYPE") {
            Some("CONTACTPAIR") => {
                let Some(d) = data.first() else {
                    return Err(k.error("the slave and master surface are needed"));
                };
                let f = d.fields();
                if f.len() != 2 {
                    return Err(data_error(k, d, "the slave and master surface are needed"));
                }
                ContactChange::ContactPair {
                    add,
                    slave: normalized_name(f[0]),
                    master: normalized_name(f[1]),
                }
            }
            Some("ELEMENT") => {
                let Some(d) = data.first() else {
                    return Err(k.error("the element set is needed"));
                };
                ContactChange::Elements {
                    add,
                    elset: normalized_name(d.fields().first().copied().unwrap_or_default()),
                }
            }
            _ => return Err(k.error("TYPE=CONTACT PAIR or TYPE=ELEMENT is needed")),
        };
        self.current_step(k)?.contact_changes.push(change);
        Ok(())
    }

    fn change_friction(&mut self, k: &Keyword) -> Result<(), Error> {
        k.check_parameters(&["INTERACTION"])?;
        let Some(interaction) = k.value("INTERACTION").map(normalized_name) else {
            return Err(k.error("INTERACTION is needed"));
        };
        let step = self.current_step(k)?;
        step.contact_changes.push(ContactChange::Friction {
            interaction,
            friction: Friction {
                coefficient: 0.0,
                stick_slope: None,
            },
        });
        self.friction_target = FrictionTarget::Change(step.contact_changes.len() - 1);
        Ok(())
    }

    // ----- Outputs ----------------------------------------------------------------------

    fn field_output(
        &mut self,
        k: &Keyword,
        data: &[Data<'_>],
        kind: FileKind,
    ) -> Result<(), Error> {
        k.check_parameters(&[
            "FREQUENCY",
            "FREQUENCYF",
            "NSET",
            "LAST ITERATIONS",
            "CONTACT ELEMENTS",
            "OUTPUT",
            "GLOBAL",
            "TIME POINTS",
            "OUTPUT ALL",
            "SECTION FORCES",
        ])?;
        let mut variables = Vec::new();
        for d in data {
            variables.extend(d.fields().into_iter().map(|v| v.to_ascii_uppercase()));
        }
        let output = FieldOutput {
            variables,
            frequency: k.integer("FREQUENCY")?.unwrap_or(1),
            nset: k.value("NSET").map(normalized_name),
            last_iterations: k.has("LAST ITERATIONS"),
            contact_elements: k.has("CONTACT ELEMENTS"),
            output: match k.value("OUTPUT") {
                Some("2D") => OutputDimension::TwoD,
                Some("3D") => OutputDimension::ThreeD,
                _ => OutputDimension::Expanded,
            },
            global: k.value("GLOBAL") != Some("NO"),
            section_forces: k.has("SECTION FORCES"),
            time_points: k.value("TIME POINTS").map(normalized_name),
        };
        let card = match kind {
            FileKind::Node => Card::NodeFile(output),
            FileKind::Element => Card::ElFile(output),
            FileKind::Contact => Card::ContactFile(output),
        };
        self.apply(k, card)
    }

    fn history_output(
        &mut self,
        k: &Keyword,
        data: &[Data<'_>],
        kind: FileKind,
    ) -> Result<(), Error> {
        k.check_parameters(&[
            "NSET",
            "ELSET",
            "FREQUENCY",
            "FREQUENCYF",
            "TOTALS",
            "GLOBAL",
            "TIME POINTS",
            "SLAVE",
            "MASTER",
        ])?;
        let set = match kind {
            FileKind::Node => k.value("NSET"),
            FileKind::Element => k.value("ELSET"),
            FileKind::Contact => None,
        }
        .map(normalized_name);
        if set.is_none() && kind != FileKind::Contact {
            return Err(k.error("the set is needed"));
        }
        let mut variables = Vec::new();
        for d in data {
            variables.extend(d.fields().into_iter().map(|v| v.to_ascii_uppercase()));
        }
        let output = HistoryOutput {
            set,
            variables,
            frequency: k.integer("FREQUENCY")?.unwrap_or(1),
            totals: match k.value("TOTALS") {
                Some("YES") => Totals::Yes,
                Some("ONLY") => Totals::Only,
                _ => Totals::No,
            },
            global: k.value("GLOBAL") != Some("NO"),
            time_points: k.value("TIME POINTS").map(normalized_name),
            surfaces: match (kind, k.value("SLAVE"), k.value("MASTER")) {
                (FileKind::Contact, Some(slave), Some(master)) => {
                    Some((normalized_name(slave), normalized_name(master)))
                }
                _ => None,
            },
        };
        let card = match kind {
            FileKind::Node => Card::NodePrint(output),
            FileKind::Element => Card::ElPrint(output),
            FileKind::Contact => Card::ContactPrint(output),
        };
        self.apply(k, card)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum HeatKind {
    Heat,
    Coupled,
    Uncoupled,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum FileKind {
    Node,
    Element,
    Contact,
}

// ----- Helpers ------------------------------------------------------------------------------

fn data_error(k: &Keyword, d: &Data<'_>, message: impl Into<String>) -> Error {
    Error::input(
        k.display_name(),
        format!("{} (line: {})", message.into(), d.text),
    )
    .at(d.file, d.line)
}

fn parse_number(k: &Keyword, d: &Data<'_>, text: &str) -> Result<f64, Error> {
    let text = text.trim();
    if text.is_empty() {
        return Ok(0.0);
    }
    // Fortran-style exponents without `E` (`1.0+3`) are rare; `1.0D3` occurs.
    text.replace(['D', 'd'], "E")
        .parse::<f64>()
        .map_err(|_| data_error(k, d, format!("{text} is not a number")))
}

fn parse_id(k: &Keyword, d: &Data<'_>, text: &str) -> Result<u32, Error> {
    text.trim()
        .parse::<u32>()
        .map_err(|_| data_error(k, d, format!("{text} is not a node or element number")))
}

/// The signal of an `*AMPLITUDE, SHAPE=...`: its numbers on the data lines are
/// `start, width` of a pulse, `start, end, frequency, phase` of a sine (phase in degrees) and
/// `start, end, start frequency, end frequency` of a chirp.
fn signal(k: &Keyword, shape: &str, values: &[f64]) -> Result<Signal, Error> {
    let take = |n: usize, what: &str| -> Result<&[f64], Error> {
        if values.len() != n {
            return Err(k.error(format!("SHAPE={shape} needs {what} on its data line")));
        }
        Ok(values)
    };
    let signal = match shape {
        "PULSE" => {
            let v = take(2, "start and width")?;
            if v[1] <= 0.0 {
                return Err(k.error("the width of a pulse must be positive"));
            }
            Signal::Pulse {
                start: v[0],
                width: v[1],
            }
        }
        "SINE" => {
            let v = take(4, "start, end, frequency and phase")?;
            Signal::Sine {
                start: v[0],
                end: v[1],
                frequency: v[2],
                phase: v[3].to_radians(),
            }
        }
        "CHIRP" => {
            let v = take(4, "start, end, start frequency and end frequency")?;
            let logarithmic = match k.value("SWEEP") {
                None | Some("LINEAR") => false,
                Some("LOG" | "LOGARITHMIC") => true,
                Some(other) => return Err(k.error(format!("unknown SWEEP={other}"))),
            };
            if logarithmic && (v[2] <= 0.0 || v[3] <= 0.0) {
                return Err(k.error("a logarithmic sweep needs positive frequencies"));
            }
            Signal::Chirp {
                start: v[0],
                end: v[1],
                from: v[2],
                to: v[3],
                logarithmic,
            }
        }
        other => return Err(k.error(format!("unknown SHAPE={other}"))),
    };
    if let Signal::Sine { frequency, .. } = signal
        && frequency <= 0.0
    {
        return Err(k.error("the frequency must be positive"));
    }
    if signal.end().is_nan() || signal.end() <= signal.start() {
        return Err(k.error("the signal must end after it starts"));
    }
    Ok(signal)
}

fn numbers(k: &Keyword, d: &Data<'_>) -> Result<Vec<f64>, Error> {
    d.fields()
        .into_iter()
        .map(|f| parse_number(k, d, f))
        .collect()
}

/// `first, last, step` of a `GENERATE` line.
fn generated(k: &Keyword, d: &Data<'_>, f: &[&str]) -> Result<Vec<u32>, Error> {
    if f.len() < 2 {
        return Err(data_error(k, d, "GENERATE needs first and last id"));
    }
    let first = parse_id(k, d, f[0])?;
    let last = parse_id(k, d, f[1])?;
    let step = match f.get(2) {
        Some(s) if !s.is_empty() => parse_id(k, d, s)?.max(1),
        _ => 1,
    };
    Ok((first..=last).step_by(step as usize).collect())
}

fn node_ref(text: &str) -> NodeRef {
    match text.trim().parse::<NodeId>() {
        Ok(id) => NodeRef::Node(id),
        Err(_) => NodeRef::Set(normalized_name(text)),
    }
}

fn element_ref(text: &str) -> ElementRef {
    match text.trim().parse::<ElementId>() {
        Ok(id) => ElementRef::Element(id),
        Err(_) => ElementRef::Set(normalized_name(text)),
    }
}

/// `S4` -> 4, `SPOS` -> 2, `SNEG` -> 1 of a surface definition.
fn parse_face(k: &Keyword, d: &Data<'_>, text: &str) -> Result<u8, Error> {
    let label = text.trim().to_ascii_uppercase();
    match label.as_str() {
        "SPOS" => Ok(2),
        "SNEG" => Ok(1),
        _ => parse_face_label(k, d, &label, 'S'),
    }
}

/// `<prefix><n>` -> n.
fn parse_face_label(k: &Keyword, d: &Data<'_>, text: &str, prefix: char) -> Result<u8, Error> {
    let label = text.trim().to_ascii_uppercase();
    label
        .strip_prefix(prefix)
        .and_then(|n| n.parse::<u8>().ok())
        .ok_or_else(|| data_error(k, d, format!("{text} is not a face label")))
}

fn solver(k: &Keyword) -> Result<EquationSolver, Error> {
    Ok(match k.value("SOLVER") {
        None => EquationSolver::Default,
        Some("SPOOLES") => EquationSolver::Spooles,
        Some("PASTIX") => EquationSolver::Pastix,
        Some("PARDISO") => EquationSolver::Pardiso,
        Some("ITERATIVESCALING") => EquationSolver::IterativeScaling,
        Some("ITERATIVECHOLESKY") => EquationSolver::IterativeCholesky,
        Some(other) => return Err(k.error(format!("SOLVER={other} is not supported"))),
    })
}

/// The incrementation line of a procedure: initial, period, minimum, maximum.
fn incrementation(k: &Keyword, data: &[Data<'_>]) -> Result<Incrementation, Error> {
    let v = match data.first() {
        Some(d) => d
            .fields()
            .into_iter()
            .map(|f| {
                if f.is_empty() {
                    Ok(None)
                } else {
                    parse_number(k, d, f).map(Some)
                }
            })
            .collect::<Result<Vec<_>, _>>()?,
        None => Vec::new(),
    };
    let at = |i: usize| v.get(i).copied().flatten();
    Ok(Incrementation {
        initial: at(0),
        period: at(1),
        minimum: at(2),
        maximum: at(3),
        direct: k.has("DIRECT"),
        frames: match k.integer("FRAMES")? {
            Some(0) => return Err(k.error("FRAMES must be at least 1")),
            frames => frames,
        },
    })
}
