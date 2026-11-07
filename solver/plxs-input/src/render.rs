//! The other direction of the reader: an [`Analysis`] as the text of an input file.
//!
//! Nodalis computes its analyses from the IR directly; the text is only for the export
//! ("Export > Solver input") and for the command line `plxs job.inp`. Reading the text back
//! gives the same analysis: every number is written so that it reads back exactly, every set
//! by its members, and every step complete (`OP=NEW` for each kind it or the step before it
//! has), so that nothing depends on the carry-over between steps. What the format cannot say
//! is left out: a step without output requests after one with them (the requests carry on),
//! and orientations of springs (not read).
//!
//! The keywords are spelled as the CalculiX manual (2.23) documents them, so that the file
//! also runs in CalculiX for the features both have.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

use plxs_core::constraints::{RigidBodyNodes, TransformKind};
use plxs_core::contact::{Adjust, ContactKind, GapConductance, SurfaceBehavior};
use plxs_core::element::ElementType;
use plxs_core::material::TemperatureValue;
use plxs_core::mesh::Surface;
use plxs_core::section::{BeamProfile, SectionKind, SpringStiffness};
use plxs_core::step::{
    ContactChange, DefaultAmplitude, DefinedFieldKind, DistributedLoadKind, EquationSolver,
    FieldOutput, FluxKind, HistoryOutput, Incrementation, Loads, ModalDamping, OutputDimension,
    Procedure, Totals,
};
use plxs_core::{Analysis, Model, Step};

/// The text of an input file that reads back as `analysis`.
pub fn render(analysis: &Analysis) -> String {
    render_with_parts(analysis, &[])
}

/// [`render`] with the elements of the element sets `parts` in element blocks of those
/// names (`*ELEMENT, ELSET=`), as far as the order of the elements allows: a mesh reader
/// makes parts of them. The names are written as given, in any case.
pub fn render_with_parts(analysis: &Analysis, parts: &[String]) -> String {
    let mut out = Out::default();
    let model = &analysis.model;
    heading(&mut out, model);
    if let Some(submodel) = &model.constraints.submodel {
        let mut header = format!("*Submodel, Type=Node, Input=\"{}\"", submodel.input);
        if let Some(elset) = &submodel.global_elset {
            let _ = write!(header, ", Global elset={elset}");
        }
        out.keyword(&header);
        out.list(submodel.node_sets.iter().map(String::as_str));
    }
    mesh(&mut out, model, parts);
    materials(&mut out, model);
    sections(&mut out, model);
    constraints(&mut out, model);
    contact(&mut out, model);
    amplitudes(&mut out, model);
    initial_conditions(&mut out, model);
    if let Some(design) = &model.design_variables {
        out.keyword("*Design variables, Type=Coordinate");
        out.line(&design.nset);
    }
    if let Some(restart) = &model.restart_read {
        match restart.step {
            Some(step) => out.keyword(&format!("*Restart, Read, Step={step}")),
            None => out.keyword("*Restart, Read"),
        }
    }
    steps(&mut out, analysis);
    out.text
}

#[derive(Default)]
struct Out {
    text: String,
}

impl Out {
    fn keyword(&mut self, line: &str) {
        self.text.push_str(line);
        self.text.push('\n');
    }

    fn comment(&mut self, text: &str) {
        let _ = writeln!(self.text, "** {text}");
    }

    fn line(&mut self, line: &str) {
        self.text.push_str(line);
        self.text.push('\n');
    }

    /// Comma-separated entries, at most `per_line` on a line; a line that goes on ends in a
    /// comma, which the reader joins with the next.
    fn entries(&mut self, entries: &[String], per_line: usize) {
        let lines: Vec<String> = entries.chunks(per_line).map(|c| c.join(", ")).collect();
        if !lines.is_empty() {
            self.text.push_str(&lines.join(",\n"));
            self.text.push('\n');
        }
    }

    /// Ids or names, 16 to a line.
    fn list<'a>(&mut self, items: impl IntoIterator<Item = &'a str>) {
        let items: Vec<String> = items.into_iter().map(str::to_owned).collect();
        self.entries(&items, 16);
    }

    fn ids(&mut self, ids: &[u32]) {
        let items: Vec<String> = ids.iter().map(u32::to_string).collect();
        self.entries(&items, 16);
    }

    fn numbers(&mut self, values: &[f64]) {
        let items: Vec<String> = values.iter().map(|&v| number(v)).collect();
        self.entries(&items, 8);
    }
}

/// A number that reads back as the same `f64`: the shortest form that does, plain when that
/// is short, else with an exponent.
pub(crate) fn number(value: f64) -> String {
    if value.is_nan() {
        return String::new();
    }
    let plain = value.to_string();
    let exponent = format!("{value:e}");
    if plain.len() <= exponent.len() || plain.len() <= 12 {
        plain
    } else {
        exponent
    }
}

fn heading(out: &mut Out, model: &Model) {
    out.keyword("*Heading");
    for line in model.heading.lines() {
        // A heading line that looked like a keyword or a comment would not read back.
        out.line(line.trim_start_matches('*'));
    }
}

fn mesh(out: &mut Out, model: &Model, parts: &[String]) {
    if !model.nodes.is_empty() {
        out.keyword("*Node");
        for (id, [x, y, z]) in model.nodes.iter() {
            let _ = writeln!(
                out.text,
                "{id}, {}, {}, {}",
                number(x),
                number(y),
                number(z)
            );
        }
    }
    // An element block takes the name of the part whose members go on with its elements;
    // what such blocks leave of a set follows as `*ELSET`.
    let sets = &model.sets.element_sets;
    let part_name = |set: &str| parts.iter().find(|p| plxs_core::normalized_name(p) == set);
    let mut covered: BTreeMap<&str, usize> = sets.keys().map(|k| (k.as_str(), 0)).collect();
    let mut user_element = false;
    let elements = &model.elements;
    let mut i = 0;
    while i < elements.len() {
        let kind = elements[i].kind;
        let run = |ids: &[u32]| {
            (ids.iter().zip(&elements[i..]))
                .take_while(|(id, e)| **id == e.id && e.kind == kind)
                .count()
        };
        let mut best: Option<(&str, usize)> = None;
        for (name, ids) in sets.iter().filter(|(name, _)| part_name(name).is_some()) {
            let n = run(&ids[covered[name.as_str()]..]);
            if n > 0 && best.is_none_or(|(_, m)| n > m) {
                best = Some((name, n));
            }
        }
        let (set, count) = match best {
            Some((set, n)) => (Some(set), n),
            None => {
                // Up to the next element a set goes on with.
                let heads: BTreeSet<u32> = (sets.iter())
                    .filter(|(name, _)| part_name(name).is_some())
                    .filter_map(|(name, ids)| ids.get(covered[name.as_str()]).copied())
                    .collect();
                let n = (elements[i..].iter().enumerate())
                    .take_while(|(k, e)| e.kind == kind && (*k == 0 || !heads.contains(&e.id)))
                    .count();
                (None, n)
            }
        };
        if kind == ElementType::U1 && !std::mem::replace(&mut user_element, true) {
            out.keyword("*User element, Type=U1, Nodes=2, Integration points=2, Maxdof=6");
        }
        match set {
            Some(set) => {
                let written = part_name(set).map_or(set, String::as_str);
                out.keyword(&format!("*Element, Type={kind}, Elset={written}"));
                *covered.get_mut(set).unwrap() += count;
            }
            None => out.keyword(&format!("*Element, Type={kind}")),
        }
        for element in &elements[i..i + count] {
            let mut entries = vec![element.id.to_string()];
            entries.extend(element.nodes.iter().map(u32::to_string));
            out.entries(&entries, 16);
        }
        i += count;
    }
    for (name, ids) in &model.sets.node_sets {
        out.keyword(&format!("*Nset, Nset={name}"));
        out.ids(ids);
    }
    for (name, ids) in sets {
        let rest = &ids[covered[name.as_str()]..];
        if ids.is_empty() || !rest.is_empty() {
            out.keyword(&format!("*Elset, Elset={name}"));
            out.ids(rest);
        }
    }
    for (name, surface) in &model.sets.surfaces {
        match surface {
            Surface::Element(faces) => {
                out.keyword(&format!("*Surface, Name={name}, Type=Element"));
                for face in faces {
                    let _ = writeln!(out.text, "{}, S{}", face.element, face.face);
                }
            }
            Surface::Node(nodes) => {
                out.keyword(&format!("*Surface, Name={name}, Type=Node"));
                out.ids(nodes);
            }
        }
    }
}

fn table(out: &mut Out, rows: &[TemperatureValue]) {
    for row in rows {
        let _ = writeln!(
            out.text,
            "{}, {}",
            number(row.value),
            number(row.temperature)
        );
    }
}

fn materials(out: &mut Out, model: &Model) {
    for material in &model.materials {
        out.keyword(&format!("*Material, Name={}", material.name));
        if let Some(elastic) = &material.elastic {
            out.keyword("*Elastic");
            for row in &elastic.rows {
                out.numbers(&[row.young, row.poisson, row.temperature]);
            }
        }
        if let Some(hyper) = &material.hyperelastic {
            use plxs_core::material::HyperelasticModel as H;
            let model = match hyper.model {
                H::NeoHooke => "Neo Hooke".to_string(),
                H::MooneyRivlin => "Mooney-Rivlin".to_string(),
                H::Polynomial(n) => format!("Polynomial, N={n}"),
                H::ReducedPolynomial(n) => format!("Reduced polynomial, N={n}"),
                H::Yeoh => "Yeoh".to_string(),
                H::Ogden(n) => format!("Ogden, N={n}"),
                H::ArrudaBoyce => "Arruda-Boyce".to_string(),
            };
            out.keyword(&format!("*Hyperelastic, {model}"));
            let mut values = hyper.constants.clone();
            values.push(0.0);
            out.numbers(&values);
        }
        if let Some(plastic) = &material.plastic {
            let hardening = match plastic.hardening {
                plxs_core::material::Hardening::Isotropic => "Isotropic",
                plxs_core::material::Hardening::Kinematic => "Kinematic",
                plxs_core::material::Hardening::Combined => "Combined",
            };
            out.keyword(&format!("*Plastic, Hardening={hardening}"));
            for row in &plastic.rows {
                out.numbers(&[row.stress, row.plastic_strain, row.temperature]);
            }
        }
        if let Some(creep) = &material.creep {
            out.keyword("*Creep, Law=Norton");
            for row in &creep.rows {
                out.numbers(&[row.a, row.n, row.m, row.temperature]);
            }
        }
        if let Some(expansion) = &material.expansion {
            out.keyword(&format!("*Expansion, Zero={}", number(expansion.zero)));
            table(out, &expansion.rows);
        }
        if let Some(conductivity) = &material.conductivity {
            out.keyword("*Conductivity");
            table(out, &conductivity.rows);
        }
        if let Some(specific_heat) = &material.specific_heat {
            out.keyword("*Specific heat");
            table(out, &specific_heat.rows);
        }
        if let Some(density) = &material.density {
            out.keyword("*Density");
            table(out, &density.rows);
        }
    }
}

fn sections(out: &mut Out, model: &Model) {
    for section in &model.sections {
        let elset = &section.elset;
        match &section.kind {
            SectionKind::Solid {
                material,
                thickness,
            } => {
                out.keyword(&format!(
                    "*Solid section, Elset={elset}, Material={material}"
                ));
                if let Some(thickness) = thickness {
                    out.line(&number(*thickness));
                }
            }
            SectionKind::Shell {
                material,
                thickness,
                offset,
                nodal_thickness,
            } => {
                let mut header = format!("*Shell section, Elset={elset}, Material={material}");
                if *offset != 0.0 {
                    let _ = write!(header, ", Offset={}", number(*offset));
                }
                if *nodal_thickness {
                    header.push_str(", Nodal thickness");
                }
                out.keyword(&header);
                out.line(&number(*thickness));
            }
            SectionKind::Membrane {
                material,
                thickness,
                offset,
            } => {
                let mut header = format!("*Membrane section, Elset={elset}, Material={material}");
                if *offset != 0.0 {
                    let _ = write!(header, ", Offset={}", number(*offset));
                }
                out.keyword(&header);
                out.line(&number(*thickness));
            }
            SectionKind::Beam {
                material,
                profile,
                normal,
                offset1,
                offset2,
            } => {
                let (name, values) = match profile {
                    BeamProfile::Rect { a, b } => ("Rect", vec![*a, *b]),
                    BeamProfile::Circ { a, b } => ("Circ", vec![*a, *b]),
                    BeamProfile::Pipe { radius, thickness } => ("Pipe", vec![*radius, *thickness]),
                    BeamProfile::Box {
                        a,
                        b,
                        t1,
                        t2,
                        t3,
                        t4,
                    } => ("Box", vec![*a, *b, *t1, *t2, *t3, *t4]),
                    BeamProfile::General {
                        area,
                        i11,
                        i12,
                        i22,
                        torsion,
                        shear_area_1,
                        shear_area_2,
                    } => {
                        let mut values = vec![*area, *i11, *i12, *i22, *torsion];
                        match (shear_area_1, shear_area_2) {
                            (Some(a1), Some(a2)) => values.extend([*a1, *a2]),
                            (Some(a1), None) => values.push(*a1),
                            // The second shear area alone cannot be written; the first
                            // is then CalculiX's default, the area.
                            (None, Some(a2)) => values.extend([*area, *a2]),
                            (None, None) => {}
                        }
                        ("General", values)
                    }
                };
                let mut header =
                    format!("*Beam section, Elset={elset}, Material={material}, Section={name}");
                if *offset1 != 0.0 {
                    let _ = write!(header, ", Offset1={}", number(*offset1));
                }
                if *offset2 != 0.0 {
                    let _ = write!(header, ", Offset2={}", number(*offset2));
                }
                out.keyword(&header);
                out.numbers(&values);
                out.numbers(normal);
            }
            SectionKind::Spring(spring) => {
                let nonlinear = matches!(spring.stiffness, SpringStiffness::Nonlinear(_));
                out.keyword(&if nonlinear {
                    format!("*Spring, Elset={elset}, Nonlinear")
                } else {
                    format!("*Spring, Elset={elset}")
                });
                // SPRING2 takes both degrees of freedom, SPRING1 one.
                let two = spring.dof1 != spring.dof2
                    || (model.sets.element_sets.get(elset)).is_some_and(|ids| {
                        ids.iter().any(|&id| {
                            model
                                .element(id)
                                .is_some_and(|e| e.kind == ElementType::SPRING2)
                        })
                    });
                if two {
                    let _ = writeln!(out.text, "{}, {}", spring.dof1, spring.dof2);
                } else {
                    let _ = writeln!(out.text, "{}", spring.dof1);
                }
                match &spring.stiffness {
                    SpringStiffness::Linear(rows) => table(out, rows),
                    SpringStiffness::Nonlinear(rows) => {
                        for &(force, displacement, temperature) in rows {
                            out.numbers(&[force, displacement, temperature]);
                        }
                    }
                }
            }
            SectionKind::Gap(gap) => {
                out.keyword(&format!("*Gap, Elset={elset}"));
                let [x, y, z] = gap.direction;
                let mut line = format!(
                    "{}, {}, {}, {}",
                    number(gap.clearance),
                    number(x),
                    number(y),
                    number(z)
                );
                if let Some(stiffness) = gap.stiffness {
                    let _ = write!(line, ", , {}", number(stiffness));
                }
                out.line(&line);
            }
        }
    }
}

fn constraints(out: &mut Out, model: &Model) {
    let constraints = &model.constraints;
    for transform in &constraints.transforms {
        let kind = match transform.kind {
            TransformKind::Rectangular => "R",
            TransformKind::Cylindrical => "C",
        };
        out.keyword(&format!("*Transform, Nset={}, Type={kind}", transform.nset));
        let [a, b] = [transform.a, transform.b];
        out.numbers(&[a[0], a[1], a[2], b[0], b[1], b[2]]);
    }
    for normal in &constraints.normals {
        out.keyword("*Normal");
        let [x, y, z] = normal.normal;
        let _ = writeln!(
            out.text,
            "{}, {}, {}, {}, {}",
            normal.element,
            normal.node,
            number(x),
            number(y),
            number(z)
        );
    }
    for body in &constraints.rigid_bodies {
        let nodes = match &body.nodes {
            RigidBodyNodes::NodeSet(set) => format!("Nset={set}"),
            RigidBodyNodes::ElementSet(set) => format!("Elset={set}"),
        };
        out.keyword(&format!(
            "*Rigid body, {nodes}, Ref node={}, Rot node={}",
            body.reference_node, body.rotation_node
        ));
    }
    for joint in &constraints.joints {
        out.keyword(&format!("*Joint, Name={}", joint.name));
        let [first, second] = joint.ends.map(|end| end.unwrap_or([0, 0]));
        out.line(&format!(
            "{}, {}, {}, {}",
            first[0], first[1], second[0], second[1]
        ));
        let [x, y, _] = joint.axes;
        let o = joint.origin;
        let frame: Vec<String> = [o, x, y].iter().flatten().map(|&v| number(v)).collect();
        out.line(&frame.join(", "));
        let held: Vec<String> = (1..=6)
            .filter(|&d| joint.fixed[d - 1])
            .map(|d| d.to_string())
            .collect();
        if !held.is_empty() {
            out.line(&held.join(", "));
        }
    }
    if !constraints.equations.is_empty() {
        out.keyword("*Equation");
        for equation in &constraints.equations {
            let _ = writeln!(out.text, "{}", equation.terms.len());
            // Four terms to a line, as CalculiX reads them.
            for chunk in equation.terms.chunks(4) {
                let terms: Vec<String> = (chunk.iter())
                    .map(|t| format!("{}, {}, {}", t.node, t.dof, number(t.coefficient)))
                    .collect();
                out.line(&terms.join(", "));
            }
        }
    }
    for tie in &constraints.ties {
        let mut header = String::from("*Tie");
        if !tie.name.is_empty() {
            let _ = write!(header, ", Name={}", tie.name);
        }
        if let Some(tolerance) = tie.position_tolerance {
            let _ = write!(header, ", Position tolerance={}", number(tolerance));
        }
        if !tie.adjust {
            header.push_str(", Adjust=No");
        }
        if tie.cyclic_symmetry {
            header.push_str(", Cyclic symmetry");
        }
        if tie.multistage {
            header.push_str(", Multistage");
        }
        out.keyword(&header);
        let _ = writeln!(out.text, "{}, {}", tie.slave, tie.master);
    }
    if let Some(cyclic) = &constraints.cyclic_symmetry {
        let mut header = format!(
            "*Cyclic symmetry model, N={}, Ngraph={}, Tie={}",
            cyclic.sectors, cyclic.ngraph, cyclic.tie
        );
        if let Some(elset) = &cyclic.elset {
            let _ = write!(header, ", Elset={elset}");
        }
        if !cyclic.check {
            header.push_str(", Check=No");
        }
        out.keyword(&header);
        let [a, b] = cyclic.axis;
        out.numbers(&[a[0], a[1], a[2], b[0], b[1], b[2]]);
    }
}

fn contact(out: &mut Out, model: &Model) {
    let contact = &model.contact;
    for interaction in &contact.interactions {
        out.keyword(&format!("*Surface interaction, Name={}", interaction.name));
        match &interaction.behavior {
            None => {}
            Some(SurfaceBehavior::Hard) => {
                out.keyword("*Surface behavior, Pressure-overclosure=Hard");
            }
            Some(SurfaceBehavior::Linear { k, sigma_inf, c0 }) => {
                out.keyword("*Surface behavior, Pressure-overclosure=Linear");
                let mut values = vec![*k];
                match (sigma_inf, c0) {
                    (Some(s), Some(c)) => values.extend([*s, *c]),
                    (Some(s), None) => values.push(*s),
                    (None, Some(c)) => values.extend([0.0, *c]),
                    (None, None) => {}
                }
                out.numbers(&values);
            }
            Some(SurfaceBehavior::Exponential { c0, p0 }) => {
                out.keyword("*Surface behavior, Pressure-overclosure=Exponential");
                out.numbers(&[*c0, *p0]);
            }
            Some(SurfaceBehavior::Tabular(points)) => {
                out.keyword("*Surface behavior, Pressure-overclosure=Tabular");
                for &(pressure, overclosure) in points {
                    out.numbers(&[pressure, overclosure]);
                }
            }
            Some(SurfaceBehavior::Tied { k }) => {
                out.keyword("*Surface behavior, Pressure-overclosure=Tied");
                out.numbers(&[*k]);
            }
        }
        if let Some(friction) = &interaction.friction {
            out.keyword("*Friction");
            friction_line(out, friction);
        }
        match &interaction.gap_conductance {
            None => {}
            Some(GapConductance::Constant(value)) => {
                out.keyword("*Gap conductance");
                out.numbers(&[*value]);
            }
            Some(GapConductance::Table(rows)) => {
                out.keyword("*Gap conductance");
                for &(conductance, pressure, temperature) in rows {
                    out.numbers(&[conductance, pressure, temperature]);
                }
            }
        }
    }
    for pair in &contact.pairs {
        let kind = match pair.kind {
            ContactKind::NodeToSurface => "Node to surface",
            ContactKind::SurfaceToSurface => "Surface to surface",
            ContactKind::Mortar => "Mortar",
            ContactKind::LinearMortar => "Linmortar",
        };
        let mut header = format!(
            "*Contact pair, Interaction={}, Type={kind}",
            pair.interaction
        );
        if pair.small_sliding {
            header.push_str(", Small sliding");
        }
        match &pair.adjust {
            None => {}
            Some(Adjust::Distance(distance)) => {
                let _ = write!(header, ", Adjust={}", number(*distance));
            }
            Some(Adjust::NodeSet(set)) => {
                let _ = write!(header, ", Adjust={set}");
            }
        }
        out.keyword(&header);
        let _ = writeln!(out.text, "{}, {}", pair.slave, pair.master);
    }
    for clearance in &contact.clearances {
        out.keyword(&format!(
            "*Clearance, Master={}, Slave={}, Value={}",
            clearance.master,
            clearance.slave,
            number(clearance.value)
        ));
    }
}

fn friction_line(out: &mut Out, friction: &plxs_core::contact::Friction) {
    match friction.stick_slope {
        Some(slope) => out.numbers(&[friction.coefficient, slope]),
        None => out.numbers(&[friction.coefficient]),
    }
}

fn amplitudes(out: &mut Out, model: &Model) {
    for amplitude in &model.amplitudes {
        let mut header = format!("*Amplitude, Name={}", amplitude.name);
        if amplitude.total_time {
            header.push_str(", Time=Total time");
        }
        if amplitude.shift_x != 0.0 {
            let _ = write!(header, ", Shiftx={}", number(amplitude.shift_x));
        }
        if amplitude.shift_y != 0.0 {
            let _ = write!(header, ", Shifty={}", number(amplitude.shift_y));
        }
        out.keyword(&header);
        let values: Vec<f64> = (amplitude.points.iter())
            .flat_map(|&(t, v)| [t, v])
            .collect();
        out.numbers(&values);
    }
}

fn initial_conditions(out: &mut Out, model: &Model) {
    let initial = &model.initial;
    if !initial.temperatures.is_empty() {
        out.keyword("*Initial conditions, Type=Temperature");
        for (region, value) in &initial.temperatures {
            let _ = writeln!(out.text, "{region}, {}", number(*value));
        }
    }
    for (kind, entries) in [
        ("Velocity", &initial.velocities),
        ("Displacement", &initial.displacements),
    ] {
        if entries.is_empty() {
            continue;
        }
        out.keyword(&format!("*Initial conditions, Type={kind}"));
        for (region, dof, value) in entries {
            let _ = writeln!(out.text, "{region}, {dof}, {}", number(*value));
        }
    }
    let physical = &model.physical;
    let mut header = String::from("*Physical constants");
    for (name, value) in [
        ("Absolute zero", physical.absolute_zero),
        ("Stefan Boltzmann", physical.stefan_boltzmann),
        ("Newton gravity", physical.newton_gravity),
    ] {
        if let Some(value) = value {
            let _ = write!(header, ", {name}={}", number(value));
        }
    }
    if header.contains('=') {
        out.keyword(&header);
    }
}

/// Boundary conditions CalculiX does not take inside a step of these procedures: they go
/// before the first step, where the reader gives them to it.
fn boundaries_before_steps(procedure: &Procedure) -> bool {
    matches!(
        procedure,
        Procedure::ModalDynamics(_) | Procedure::SteadyStateDynamics(_)
    )
}

fn steps(out: &mut Out, analysis: &Analysis) {
    // What each step starts with when read: the boundaries of the step before, the loads
    // and fields of the last step that was no perturbation, the outputs of the step before.
    let mut carried_boundaries = false;
    let mut carried_loads = Loads::default();
    let mut carried_fields = false;
    for (index, step) in analysis.steps.iter().enumerate() {
        let before = index == 0 && boundaries_before_steps(&step.procedure);
        if before && !step.boundaries.is_empty() {
            boundaries(out, step, false);
        }
        out.comment(&format!("{} ++++++++++++++++++++", step.name));
        let mut header = String::from("*Step");
        if step.nlgeom {
            header.push_str(", Nlgeom");
        }
        if step.perturbation {
            header.push_str(", Perturbation");
        }
        let _ = write!(header, ", Inc={}", step.max_increments);
        header.push_str(match step.default_amplitude {
            DefaultAmplitude::Ramp => ", Amplitude=Ramp",
            DefaultAmplitude::Step => ", Amplitude=Step",
        });
        out.keyword(&header);
        procedure(out, &step.procedure);
        if let Some(controls) = &step.controls {
            out.keyword(&format!("*Controls, Parameters={}", controls.parameters));
            if !controls.values.is_empty() {
                out.numbers(&controls.values);
            }
        }
        if !before && (carried_boundaries || !step.boundaries.is_empty()) {
            boundaries(out, step, index > 0);
        }
        loads(out, &step.loads, &carried_loads);
        if carried_fields || !step.fields.is_empty() {
            fields(out, step);
        }
        for change in &step.contact_changes {
            contact_change(out, change);
        }
        outputs(out, step);
        if let Some(restart) = &step.restart_write {
            let mut header = format!("*Restart, Write, Frequency={}", restart.frequency);
            if restart.overlay {
                header.push_str(", Overlay");
            }
            out.keyword(&header);
        }
        out.keyword("*End step");
        carried_boundaries = !step.boundaries.is_empty();
        if !step.perturbation {
            carried_loads = step.loads.clone();
            carried_fields = !step.fields.is_empty();
        }
    }
}

fn solver_parameter(header: &mut String, solver: EquationSolver) {
    let name = match solver {
        EquationSolver::Default => return,
        EquationSolver::Spooles => "Spooles",
        EquationSolver::Pastix => "PaStiX",
        EquationSolver::Pardiso => "Pardiso",
        EquationSolver::IterativeScaling => "Iterative scaling",
        EquationSolver::IterativeCholesky => "Iterative Cholesky",
    };
    let _ = write!(header, ", Solver={name}");
}

/// The incrementation line: initial, period, minimum, maximum, missing ones empty.
fn increments_line(out: &mut Out, increments: &Incrementation) {
    let values = [
        increments.initial,
        increments.period,
        increments.minimum,
        increments.maximum,
    ];
    let Some(last) = values.iter().rposition(Option::is_some) else {
        return;
    };
    let fields: Vec<String> = values[..=last]
        .iter()
        .map(|v| v.map(number).unwrap_or_default())
        .collect();
    out.line(&fields.join(", "));
}

fn optional_numbers(out: &mut Out, first: String, rest: &[Option<f64>]) {
    let mut fields = vec![first];
    if let Some(last) = rest.iter().rposition(Option::is_some) {
        fields.extend(
            rest[..=last]
                .iter()
                .map(|v| v.map(number).unwrap_or_default()),
        );
    }
    out.line(&fields.join(", "));
}

fn modal_damping(out: &mut Out, damping: &Option<ModalDamping>) {
    match damping {
        None => {}
        Some(ModalDamping::Direct(rows)) => {
            out.keyword("*Modal damping");
            for &(first, last, ratio) in rows {
                let _ = writeln!(out.text, "{first}, {last}, {}", number(ratio));
            }
        }
        Some(ModalDamping::Rayleigh { alpha, beta }) => {
            out.keyword("*Modal damping, Rayleigh");
            let _ = writeln!(out.text, ", , {}, {}", number(*alpha), number(*beta));
        }
    }
}

fn procedure(out: &mut Out, procedure: &Procedure) {
    match procedure {
        Procedure::NoAnalysis => out.keyword("*No analysis"),
        Procedure::Static(settings) => {
            let mut header = String::from("*Static");
            solver_parameter(&mut header, settings.solver);
            if settings.increments.direct {
                header.push_str(", Direct");
            }
            if settings.time_reset {
                header.push_str(", Time reset");
            }
            if let Some(start) = settings.total_time_at_start {
                let _ = write!(header, ", Total time at start={}", number(start));
            }
            out.keyword(&header);
            increments_line(out, &settings.increments);
        }
        Procedure::Visco(settings) => {
            let mut header = format!("*Visco, Cetol={}", number(settings.cetol));
            solver_parameter(&mut header, settings.solver);
            if settings.increments.direct {
                header.push_str(", Direct");
            }
            out.keyword(&header);
            increments_line(out, &settings.increments);
        }
        Procedure::Frequency(settings) => {
            let mut header = String::from("*Frequency");
            solver_parameter(&mut header, settings.solver);
            if settings.storage {
                header.push_str(", Storage=Yes");
            }
            if !settings.global {
                header.push_str(", Global=No");
            }
            if !settings.cycmpc_active {
                header.push_str(", Cycmpc=Inactive");
            }
            out.keyword(&header);
            optional_numbers(
                out,
                settings.count.to_string(),
                &[settings.lower, settings.upper],
            );
            if let Some(modes) = &settings.cyclic_modes {
                let mut header = String::from("*Select cyclic symmetry modes");
                if let Some(nmin) = modes.nmin {
                    let _ = write!(header, ", Nmin={nmin}");
                }
                if let Some(nmax) = modes.nmax {
                    let _ = write!(header, ", Nmax={nmax}");
                }
                out.keyword(&header);
            }
        }
        Procedure::ComplexFrequency(settings) => {
            let mut header = String::from("*Complex frequency");
            if settings.coriolis {
                header.push_str(", Coriolis");
            }
            if settings.fluid {
                header.push_str(", Fluid");
            }
            out.keyword(&header);
            let _ = writeln!(out.text, "{}", settings.count);
        }
        Procedure::Buckle(settings) => {
            let mut header = String::from("*Buckle");
            solver_parameter(&mut header, settings.solver);
            out.keyword(&header);
            optional_numbers(out, settings.count.to_string(), &[settings.accuracy]);
        }
        Procedure::HeatTransfer(settings)
        | Procedure::CoupledTemperatureDisplacement(settings)
        | Procedure::UncoupledTemperatureDisplacement(settings) => {
            let mut header = String::from(match procedure {
                Procedure::HeatTransfer(_) => "*Heat transfer",
                Procedure::CoupledTemperatureDisplacement(_) => "*Coupled temperature-displacement",
                _ => "*Uncoupled temperature-displacement",
            });
            solver_parameter(&mut header, settings.solver);
            if settings.direct || settings.increments.direct {
                header.push_str(", Direct");
            }
            if settings.steady_state {
                header.push_str(", Steady state");
            }
            if let Some(deltmx) = settings.deltmx {
                let _ = write!(header, ", Deltmx={}", number(deltmx));
            }
            if let Some(count) = settings.frequency_count {
                let _ = write!(header, ", Frequency={count}");
            }
            out.keyword(&header);
            increments_line(out, &settings.increments);
        }
        Procedure::Dynamic(settings) => {
            let mut header = String::from("*Dynamic");
            solver_parameter(&mut header, settings.solver);
            if let Some(alpha) = settings.alpha {
                let _ = write!(header, ", Alpha={}", number(alpha));
            }
            if settings.explicit {
                header.push_str(", Explicit");
            }
            if settings.increments.direct {
                header.push_str(", Direct");
            }
            out.keyword(&header);
            increments_line(out, &settings.increments);
            if let Some(rayleigh) = &settings.rayleigh {
                out.keyword(&format!(
                    "*Damping, Alpha={}, Beta={}",
                    number(rayleigh.alpha),
                    number(rayleigh.beta)
                ));
            }
            if let Some(damping) = &settings.damping_ratio {
                let mut line = format!("*Damping, Ratio={}", number(damping.ratio));
                if let Some(lower) = damping.lower {
                    let _ = write!(line, ", Lower={}", number(lower));
                }
                let _ = write!(line, ", Upper={}", number(damping.upper));
                out.keyword(&line);
            }
        }
        Procedure::ModalDynamics(settings) => {
            let mut header = String::from("*Modal dynamics");
            solver_parameter(&mut header, settings.solver);
            if settings.steady_state {
                header.push_str(", Steady state");
            }
            out.keyword(&header);
            out.numbers(&[settings.increment, settings.period]);
            modal_damping(out, &settings.damping);
        }
        Procedure::SteadyStateDynamics(settings) => {
            let mut header = String::from("*Steady state dynamics");
            solver_parameter(&mut header, settings.solver);
            if !settings.harmonic {
                header.push_str(", Harmonic=No");
            }
            out.keyword(&header);
            let mut line = format!(
                "{}, {}, {}, {}",
                number(settings.lower),
                number(settings.upper),
                settings.points,
                number(settings.bias)
            );
            if let Some(terms) = settings.fourier_terms {
                let _ = write!(line, ", {terms}");
            }
            out.line(&line);
            modal_damping(out, &settings.damping);
        }
        Procedure::Sensitivity(settings) => {
            out.keyword(if settings.nlgeom {
                "*Sensitivity, Nlgeom"
            } else {
                "*Sensitivity"
            });
            for response in &settings.responses {
                if response.name.is_empty() {
                    out.keyword("*Design response");
                } else {
                    out.keyword(&format!("*Design response, Name={}", response.name));
                }
                let mut fields = vec![
                    response.kind.clone(),
                    response.elset.clone().unwrap_or_default(),
                ];
                fields.extend(response.data.iter().map(|&v| number(v)));
                while fields.last().is_some_and(String::is_empty) {
                    fields.pop();
                }
                out.line(&fields.join(", "));
            }
            if let Some(filter) = &settings.filter {
                out.keyword(&format!("*Filter, Type={}", filter.kind));
                out.line(&number(filter.radius));
            }
            if !settings.objectives.is_empty() {
                out.keyword("*Objective");
                out.list(settings.objectives.iter().map(String::as_str));
            }
        }
    }
}

/// `*BOUNDARY` cards, one per run of boundaries with the same parameters; `op_new` on the
/// first.
fn boundaries(out: &mut Out, step: &Step, op_new: bool) {
    let mut first = true;
    if step.boundaries.is_empty() && op_new {
        out.keyword("*Boundary, op=New");
        return;
    }
    let mut current: Option<String> = None;
    for boundary in &step.boundaries {
        let mut parameters = String::new();
        if let Some(amplitude) = &boundary.amplitude {
            let _ = write!(parameters, ", Amplitude={amplitude}");
        }
        if boundary.fixed {
            parameters.push_str(", Fixed");
        }
        if let Some(step) = boundary.submodel_step {
            let _ = write!(parameters, ", Submodel, Step={step}");
        }
        if let Some(case) = boundary.load_case {
            let _ = write!(parameters, ", Load case={case}");
        }
        if current.as_ref() != Some(&parameters) {
            let new = if op_new && std::mem::take(&mut first) {
                ", op=New"
            } else {
                ""
            };
            out.keyword(&format!("*Boundary{new}{parameters}"));
            current = Some(parameters);
        }
        let mut line = format!(
            "{}, {}, {}",
            boundary.region, boundary.first_dof, boundary.last_dof
        );
        if let Some(value) = boundary.value {
            let _ = write!(line, ", {}", number(value));
        }
        out.line(&line);
    }
}

/// Cards of one load kind: `OP=NEW` on the first when the step carries some of the kind in,
/// one card per run of loads with the same parameters.
fn cards<T>(
    out: &mut Out,
    keyword: &str,
    items: &[T],
    carried: bool,
    parameters: impl Fn(&T) -> String,
    line: impl Fn(&T) -> String,
) {
    if items.is_empty() {
        if carried {
            out.keyword(&format!("{keyword}, op=New"));
        }
        return;
    }
    let mut current: Option<String> = None;
    let mut first = true;
    for item in items {
        let params = parameters(item);
        if current.as_ref() != Some(&params) {
            let new = if std::mem::take(&mut first) {
                ", op=New"
            } else {
                ""
            };
            out.keyword(&format!("{keyword}{new}{params}"));
            current = Some(params);
        }
        out.line(&line(item));
    }
}

fn amplitude_parameter(name: &str, amplitude: &Option<String>) -> String {
    amplitude
        .as_ref()
        .map(|a| format!(", {name}={a}"))
        .unwrap_or_default()
}

fn case_and_sector(load_case: Option<u8>, sector: Option<u32>) -> String {
    let mut out = String::new();
    if let Some(case) = load_case {
        let _ = write!(out, ", Load case={case}");
    }
    if let Some(sector) = sector {
        let _ = write!(out, ", Sector={sector}");
    }
    out
}

fn loads(out: &mut Out, loads: &Loads, carried: &Loads) {
    cards(
        out,
        "*Cload",
        &loads.concentrated,
        !carried.concentrated.is_empty(),
        |l| {
            amplitude_parameter("Amplitude", &l.amplitude) + &case_and_sector(l.load_case, l.sector)
        },
        |l| format!("{}, {}, {}", l.region, l.dof, number(l.value)),
    );
    cards(
        out,
        "*Dload",
        &loads.distributed,
        !carried.distributed.is_empty(),
        |l| {
            amplitude_parameter("Amplitude", &l.amplitude) + &case_and_sector(l.load_case, l.sector)
        },
        |l| {
            let values: Vec<String> = match &l.kind {
                DistributedLoadKind::Pressure { face, value } => {
                    vec![format!("P{face}"), number(*value)]
                }
                DistributedLoadKind::Gravity {
                    magnitude,
                    direction,
                } => [String::from("GRAV"), number(*magnitude)]
                    .into_iter()
                    .chain(direction.iter().map(|&v| number(v)))
                    .collect(),
                DistributedLoadKind::Centrifugal {
                    omega_squared,
                    point,
                    axis,
                } => [String::from("CENTRIF"), number(*omega_squared)]
                    .into_iter()
                    .chain(point.iter().chain(axis).map(|&v| number(v)))
                    .collect(),
                DistributedLoadKind::Other { label, values } => std::iter::once(label.clone())
                    .chain(values.iter().map(|&v| number(v)))
                    .collect(),
            };
            format!("{}, {}", l.region, values.join(", "))
        },
    );
    cards(
        out,
        "*Cflux",
        &loads.concentrated_flux,
        !carried.concentrated_flux.is_empty(),
        |l| amplitude_parameter("Amplitude", &l.amplitude),
        |l| format!("{}, 11, {}", l.region, number(l.value)),
    );
    cards(
        out,
        "*Dflux",
        &loads.distributed_flux,
        !carried.distributed_flux.is_empty(),
        |l| amplitude_parameter("Amplitude", &l.amplitude),
        |l| {
            let label = match l.kind {
                FluxKind::Face(face) => format!("S{face}"),
                FluxKind::Body => "BF".into(),
            };
            format!("{}, {label}, {}", l.region, number(l.value))
        },
    );
    cards(
        out,
        "*Film",
        &loads.films,
        !carried.films.is_empty(),
        |l| {
            amplitude_parameter("Amplitude", &l.amplitude)
                + &amplitude_parameter("Film amplitude", &l.film_amplitude)
        },
        |l| {
            format!(
                "{}, F{}, {}, {}",
                l.region,
                l.face,
                number(l.sink_temperature),
                number(l.coefficient)
            )
        },
    );
    cards(
        out,
        "*Radiate",
        &loads.radiations,
        !carried.radiations.is_empty(),
        |l| {
            amplitude_parameter("Amplitude", &l.amplitude)
                + &amplitude_parameter("Radiation amplitude", &l.radiation_amplitude)
        },
        |l| {
            let cavity = if l.cavity { "CR" } else { "" };
            format!(
                "{}, R{}{cavity}, {}, {}",
                l.region,
                l.face,
                number(l.sink_temperature),
                number(l.emissivity)
            )
        },
    );
}

fn fields(out: &mut Out, step: &Step) {
    if step.fields.is_empty() {
        // Nothing to say "no fields" with: an empty card would be a field without values.
        return;
    }
    for (i, field) in step.fields.iter().enumerate() {
        let new = if i == 0 { ", op=New" } else { "" };
        let amplitude = amplitude_parameter("Amplitude", &field.amplitude);
        match &field.kind {
            DefinedFieldKind::Temperature(values) => {
                out.keyword(&format!("*Temperature{new}{amplitude}"));
                for (region, value) in values {
                    let _ = writeln!(out.text, "{region}, {}", number(*value));
                }
            }
            DefinedFieldKind::TemperatureFile {
                file,
                bstep,
                estep,
                binc,
                einc,
            } => {
                let mut header = format!("*Temperature{new}{amplitude}, File=\"{file}\"");
                for (name, value) in [
                    ("Bstep", bstep),
                    ("Estep", estep),
                    ("Binc", binc),
                    ("Einc", einc),
                ] {
                    if let Some(value) = value {
                        let _ = write!(header, ", {name}={value}");
                    }
                }
                out.keyword(&header);
            }
        }
    }
}

fn contact_change(out: &mut Out, change: &ContactChange) {
    let add = |add: bool| if add { "Add" } else { "Remove" };
    match change {
        ContactChange::ContactPair {
            add: a,
            slave,
            master,
        } => {
            out.keyword(&format!("*Model change, Type=Contact pair, {}", add(*a)));
            let _ = writeln!(out.text, "{slave}, {master}");
        }
        ContactChange::Elements { add: a, elset } => {
            out.keyword(&format!("*Model change, Type=Element, {}", add(*a)));
            out.line(elset);
        }
        ContactChange::Friction {
            interaction,
            friction,
        } => {
            out.keyword(&format!("*Change friction, Interaction={interaction}"));
            out.keyword("*Friction");
            friction_line(out, friction);
        }
    }
}

fn field_output_header(keyword: &str, output: &FieldOutput) -> String {
    let mut header = String::from(keyword);
    if output.frequency != 1 {
        let _ = write!(header, ", Frequency={}", output.frequency);
    }
    if let Some(nset) = &output.nset {
        let _ = write!(header, ", Nset={nset}");
    }
    if output.last_iterations {
        header.push_str(", Last iterations");
    }
    if output.contact_elements {
        header.push_str(", Contact elements");
    }
    match output.output {
        OutputDimension::Expanded => {}
        OutputDimension::TwoD => header.push_str(", Output=2D"),
        OutputDimension::ThreeD => header.push_str(", Output=3D"),
    }
    if !output.global {
        header.push_str(", Global=No");
    }
    if output.section_forces {
        header.push_str(", Section forces");
    }
    if let Some(points) = &output.time_points {
        let _ = write!(header, ", Time points={points}");
    }
    header
}

fn history_output_header(keyword: &str, set: &str, output: &HistoryOutput) -> String {
    let mut header = String::from(keyword);
    if let Some(name) = &output.set {
        let _ = write!(header, ", {set}={name}");
    }
    if let Some((slave, master)) = &output.surfaces {
        let _ = write!(header, ", Slave={slave}, Master={master}");
    }
    if output.frequency != 1 {
        let _ = write!(header, ", Frequency={}", output.frequency);
    }
    match output.totals {
        Totals::No => {}
        Totals::Yes => header.push_str(", Totals=Yes"),
        Totals::Only => header.push_str(", Totals=Only"),
    }
    if !output.global {
        header.push_str(", Global=No");
    }
    if let Some(points) = &output.time_points {
        let _ = write!(header, ", Time points={points}");
    }
    header
}

fn outputs(out: &mut Out, step: &Step) {
    let outputs = &step.outputs;
    for (keyword, list) in [
        ("*Node file", &outputs.node_file),
        ("*El file", &outputs.el_file),
        ("*Contact file", &outputs.contact_file),
    ] {
        for output in list {
            out.keyword(&field_output_header(keyword, output));
            out.list(output.variables.iter().map(String::as_str));
        }
    }
    for (keyword, set, list) in [
        ("*Node print", "Nset", &outputs.node_print),
        ("*El print", "Elset", &outputs.el_print),
        ("*Contact print", "", &outputs.contact_print),
    ] {
        for output in list {
            out.keyword(&history_output_header(keyword, set, output));
            out.list(output.variables.iter().map(String::as_str));
        }
    }
}

/// Names the analysis uses that the text cannot hold as they are, e.g. with a comma or a
/// blank: the reader would split or join them. Empty when the analysis renders faithfully.
pub fn unwritable_names(analysis: &Analysis) -> BTreeSet<String> {
    let sets = &analysis.model.sets;
    let names = (sets.node_sets.keys())
        .chain(sets.element_sets.keys())
        .chain(sets.surfaces.keys())
        .chain(analysis.model.materials.iter().map(|m| &m.name))
        .chain(analysis.model.amplitudes.iter().map(|a| &a.name));
    names
        .filter(|n| {
            n.is_empty()
                || n.contains([',', ' ', '"', '='])
                || n.starts_with('*')
                || n.parse::<u32>().is_ok()
        })
        .cloned()
        .collect()
}
