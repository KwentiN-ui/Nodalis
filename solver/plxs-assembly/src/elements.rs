//! The element table: for every element of the model its formulation, dense node indices,
//! global DOFs and material matrix, resolved once per model.

use std::collections::HashMap;

use plxs_core::section::SectionKind;
use plxs_core::state::IntegrationPointField;
use plxs_core::{Element, ElementFamily, ElementType, Error, Model, Result};
use plxs_elements::{SolidElement, plane, solid};
use plxs_materials::{LinearElastic, MaterialLaw, law_of};

use rayon::prelude::*;

use crate::DofMap;

/// One element of the table.
#[derive(Clone, Debug)]
pub struct ElementEntry {
    /// Dense index of the element in the model.
    pub element: u32,
    /// Index into [`ElementTable::formulations`].
    pub formulation: u32,
    /// Index into [`ElementTable::materials`].
    pub material: u32,
    /// Dense node indices in element order.
    pub nodes: Vec<u32>,
    /// Global DOFs in element order (3 per node).
    pub dofs: Vec<u32>,
    /// Density, when the material has one.
    pub density: Option<f64>,
}

/// Where the integration points of every element sit in a flat per-point array.
#[derive(Clone, Debug, Default)]
pub struct IntegrationPointLayout {
    /// `offsets[e]..offsets[e+1]` are the points of element `e`.
    pub offsets: Vec<u32>,
}

impl IntegrationPointLayout {
    pub fn total(&self) -> usize {
        self.offsets.last().copied().unwrap_or(0) as usize
    }

    pub fn field(&self, per_point: usize) -> IntegrationPointField {
        IntegrationPointField {
            per_point,
            offsets: self.offsets.clone(),
            values: vec![0.0; self.total() * per_point],
        }
    }
}

/// The resolved elements of a model.
pub struct ElementTable {
    /// The continuum elements (springs and gaps are in [`ElementTable::discrete`]).
    pub entries: Vec<ElementEntry>,
    /// For every model element (dense index) its entry, `u32::MAX` for discrete elements.
    pub entry_of: Vec<u32>,
    /// Springs and gaps.
    pub discrete: Vec<crate::discrete::DiscreteElement>,
    pub formulations: Vec<SolidElement>,
    /// Elastic constants and matrix per (section, temperature) combination.
    pub materials: Vec<(LinearElastic, [[f64; 6]; 6])>,
    /// The material law of every entry of `materials` (elastic, plastic, creeping), for
    /// small strains and for NLGEOM steps (finite strains).
    pub laws: Vec<Box<dyn MaterialLaw>>,
    pub laws_finite: Vec<Box<dyn MaterialLaw>>,
    pub dofs: DofMap,
    pub layout: IntegrationPointLayout,
    /// Elements computed natively with a constant stiffness and no integration points
    /// (the general beam U1): they enter `K` only.
    pub linear_blocks: Vec<LinearBlock>,
    /// Native membranes (their linear stiffness is among `linear_blocks`, which nonlinear
    /// steps skip for them).
    pub membranes: Vec<Membrane>,
    /// For every model element (dense index) its membrane, `u32::MAX` for the others.
    pub membrane_of: Vec<u32>,
    /// A material without `*ELASTIC`: allowed for pure heat transfer, where it gets zero
    /// elastic constants; [`Self::require_elastic`] refuses it for mechanical steps.
    pub missing_elastic: Option<String>,
}

/// A native membrane (`M3D3`, `M3D4`; `plxs_elements::membrane`).
#[derive(Clone, Debug)]
pub struct Membrane {
    /// Dense index of the element in the model.
    pub element: u32,
    /// Dense node indices.
    pub nodes: Vec<u32>,
    /// Global DOFs (3 per node).
    pub dofs: Vec<u32>,
    pub formulation: plxs_elements::membrane::Membrane,
    /// Density, when the material has one.
    pub density: Option<f64>,
    /// Its stiffness among [`ElementTable::linear_blocks`].
    pub block: usize,
}

impl Membrane {
    /// The element's displacements out of the global ones.
    pub fn gather(&self, u: &[f64]) -> Vec<f64> {
        self.dofs
            .iter()
            .map(|&d| u.get(d as usize).copied().unwrap_or(0.0))
            .collect()
    }

    /// Strain and stress (global, tensor shear) at the nodes for the displacements `u`
    /// (dense DOFs); `nlgeom` gives the Green-Lagrange strain and the Cauchy stress.
    pub fn nodal_values(&self, u: &[f64], nlgeom: bool) -> Vec<([f64; 6], [f64; 6])> {
        self.formulation.nodal_values(&self.gather(u), nlgeom)
    }
}

/// A constant element stiffness on global DOFs.
#[derive(Clone, Debug)]
pub struct LinearBlock {
    pub dofs: Vec<u32>,
    /// Whether the block is the linear stiffness of a native membrane.
    pub membrane: bool,
    /// Row-major, `dofs.len()` squared.
    pub stiffness: Vec<f64>,
}

impl ElementTable {
    /// Fails when a material lacks `*ELASTIC`: mechanical steps need it, heat transfer not.
    pub fn require_elastic(&self) -> Result<()> {
        match &self.missing_elastic {
            Some(name) => Err(Error::input(
                "*MATERIAL",
                format!("material {name} has no *ELASTIC definition"),
            )),
            None => Ok(()),
        }
    }

    /// Resolves the elements of a model at a uniform reference temperature. Elements the
    /// solver cannot compute yet are an error naming the type.
    pub fn new(model: &Model, temperature: f64) -> Result<Self> {
        let mut dofs = DofMap::new(model.nodes.len(), 3);
        // Formulations per type and, for plane elements, thickness.
        let mut formulation_index: HashMap<(ElementType, u64), u32> = HashMap::new();
        // Per node: whether some element uses its third DOF, whether it is internal.
        let mut uses_z = vec![false; model.nodes.len()];
        let mut connected = vec![false; model.nodes.len()];

        let mut formulations = Vec::new();
        let mut material_index: HashMap<u32, u32> = HashMap::new();
        let mut materials = Vec::new();
        let mut laws: Vec<Box<dyn MaterialLaw>> = Vec::new();
        let mut laws_finite: Vec<Box<dyn MaterialLaw>> = Vec::new();
        let mut missing_elastic: Option<String> = None;
        let mut entries = Vec::with_capacity(model.elements.len());
        let mut offsets = Vec::with_capacity(model.elements.len() + 1);
        offsets.push(0u32);
        let mut linear_blocks = Vec::new();
        let mut membranes = Vec::new();
        let mut membrane_of = vec![u32::MAX; model.elements.len()];
        let mut entry_of = Vec::with_capacity(model.elements.len());
        let mut discrete = Vec::new();
        for (index, element) in model.elements.iter().enumerate() {
            let family = element.kind.family();
            if matches!(family, ElementFamily::Spring | ElementFamily::Gap) {
                let nodes = Self::node_indices(model, element)?;
                let made = crate::discrete::DiscreteElement::new(model, index as u32, &nodes)?;
                // A spring on an in-plane DOF of a plane model leaves the out-of-plane DOF
                // unused; a gap works along its direction.
                let along_z = match &made.kind {
                    crate::discrete::DiscreteKind::Gap { direction, .. } => direction[2] != 0.0,
                    _ => made.dofs.iter().any(|d| d % 3 == 2),
                };
                for &n in &nodes {
                    connected[n as usize] = true;
                    uses_z[n as usize] |= along_z;
                }
                discrete.push(made);
                entry_of.push(u32::MAX);
                continue;
            }
            if element.kind == ElementType::U1 {
                let block = Self::u1_block(model, element, temperature)?;
                for &d in &block.dofs {
                    connected[d as usize / 3] = true;
                    uses_z[d as usize / 3] = true;
                }
                linear_blocks.push(block);
                entry_of.push(u32::MAX);
                continue;
            }
            if plxs_elements::membrane::is_native(element.kind) {
                let mut membrane = Self::membrane(model, element, index as u32, temperature)?;
                for &n in &membrane.nodes {
                    connected[n as usize] = true;
                    uses_z[n as usize] = true;
                }
                membrane_of[index] = membranes.len() as u32;
                linear_blocks.push(LinearBlock {
                    dofs: membrane.dofs.clone(),
                    membrane: true,
                    stiffness: membrane.formulation.stiffness(),
                });
                membrane.block = linear_blocks.len() - 1;
                membranes.push(membrane);
                entry_of.push(u32::MAX);
                continue;
            }
            entry_of.push(entries.len() as u32);
            let is_plane = matches!(
                family,
                ElementFamily::PlaneStress
                    | ElementFamily::PlaneStrain
                    | ElementFamily::Axisymmetric
            );
            if family != ElementFamily::Solid && !is_plane {
                return Err(Error::Unsupported(format!(
                    "element type {} is not implemented yet (element {})",
                    element.kind, element.id
                )));
            }
            let thickness = if is_plane {
                element
                    .section
                    .and_then(|s| match &model.sections.get(s as usize)?.kind {
                        SectionKind::Solid { thickness, .. } => *thickness,
                        _ => None,
                    })
            } else {
                None
            };
            // Expanded beams with a round section keep their beam section, whose profile
            // gives the integration points.
            let section_rule =
                element
                    .section
                    .and_then(|s| match &model.sections.get(s as usize)?.kind {
                        SectionKind::Beam { profile, .. } => {
                            plxs_elements::expand::section_rule(element.kind, profile)
                        }
                        _ => None,
                    });
            let key = match (&section_rule, element.section) {
                (Some(_), Some(s)) => (element.kind, u64::MAX - u64::from(s)),
                _ => (element.kind, thickness.map_or(0, f64::to_bits)),
            };
            let formulation = match formulation_index.get(&key) {
                Some(&i) => i,
                None => {
                    let made = if is_plane {
                        plane(element.kind, thickness)
                    } else {
                        solid(element.kind).map(|f| match section_rule.clone() {
                            Some((rule, extrapolation)) => f.with_rule(rule, extrapolation),
                            None => f,
                        })
                    };
                    let f = made.ok_or_else(|| {
                        Error::Unsupported(format!(
                            "element type {} is not implemented yet",
                            element.kind
                        ))
                    })?;
                    formulations.push(f);
                    let i = formulations.len() as u32 - 1;
                    formulation_index.insert(key, i);
                    i
                }
            };
            let section_index = element.section.ok_or_else(|| {
                Error::input("*ELEMENT", format!("element {} has no section", element.id))
            })?;
            let (material, density) = match material_index.get(&section_index) {
                Some(&i) => (i, section_density(model, section_index)),
                None => {
                    let section = &model.sections[section_index as usize];
                    let name = match &section.kind {
                        SectionKind::Solid { material, .. } => material.as_str(),
                        SectionKind::Beam { material, .. } if section_rule.is_some() => {
                            material.as_str()
                        }
                        _ => {
                            return Err(Error::input(
                                "*SOLID SECTION",
                                format!("element {} needs a solid section", element.id),
                            ));
                        }
                    };
                    let mat = model
                        .material(name)
                        .ok_or_else(|| Error::undefined("*SOLID SECTION", "material", name))?;
                    let elastic = if mat.elastic.is_none() {
                        missing_elastic.get_or_insert_with(|| mat.name.clone());
                        LinearElastic {
                            young: 0.0,
                            poisson: 0.0,
                        }
                    } else {
                        LinearElastic::of(mat, temperature)?
                    };
                    materials.push((elastic, elastic.matrix()));
                    if mat.elastic.is_none() {
                        laws.push(Box::new(elastic));
                        laws_finite.push(Box::new(elastic));
                    } else {
                        laws.push(law_of(mat, temperature, false)?);
                        laws_finite.push(law_of(mat, temperature, true)?);
                    }
                    let i = materials.len() as u32 - 1;
                    material_index.insert(section_index, i);
                    (i, plxs_materials::density(mat, temperature))
                }
            };
            let nodes = Self::node_indices(model, element)?;
            let f = &formulations[formulation as usize];
            for &n in &nodes {
                connected[n as usize] = true;
                uses_z[n as usize] |= f.uses_out_of_plane();
            }
            let dof_list = nodes
                .iter()
                .flat_map(|&n| (0..3).map(move |d| n * 3 + d as u32))
                .collect();
            let points = formulations[formulation as usize].integration_points() as u32;
            offsets.push(offsets.last().unwrap() + points);
            entries.push(ElementEntry {
                element: index as u32,
                formulation,
                material,
                nodes,
                dofs: dof_list,
                density,
            });
        }
        // Nodes that multi-point constraints tie in (the original nodes and helper nodes of
        // expanded shells and beams, reference nodes) work through those constraints.
        for equation in &model.constraints.equations {
            for term in &equation.terms {
                for id in model.nodes_of(&term.node) {
                    if let Some(n) = model.nodes.index(id) {
                        connected[n as usize] = true;
                        uses_z[n as usize] = true;
                    }
                }
            }
        }
        for body in &model.constraints.rigid_bodies {
            for id in [body.reference_node, body.rotation_node] {
                if let Some(n) = model.nodes.index(id) {
                    connected[n as usize] = true;
                    uses_z[n as usize] = true;
                }
            }
        }
        // DOFs no element works on are held at zero: nodes outside every element, and the
        // out-of-plane DOF of nodes that only plane strain or axisymmetric elements use.
        for n in 0..model.nodes.len() {
            let base = n as u32 * 3;
            if !connected[n] {
                dofs.unused.extend([base, base + 1, base + 2]);
            } else if !uses_z[n] {
                dofs.unused.push(base + 2);
            }
        }
        Ok(Self {
            entries,
            entry_of,
            discrete,
            formulations,
            materials,
            laws,
            laws_finite,
            dofs,
            layout: IntegrationPointLayout { offsets },
            linear_blocks,
            membranes,
            membrane_of,
            missing_elastic,
        })
    }

    /// The stiffness of a U1 beam. After the expansion its nodes are the first node, the
    /// helper node carrying its rotations, the second node and its rotation helper.
    fn u1_block(model: &Model, element: &Element, temperature: f64) -> Result<LinearBlock> {
        use plxs_core::section::BeamProfile;
        let bad =
            |what: &str| Error::input("*ELEMENT", format!("U1 element {}: {what}", element.id));
        if element.nodes.len() != 4 {
            return Err(bad("expected two nodes with rotation helpers"));
        }
        let section = element
            .section
            .and_then(|s| model.sections.get(s as usize))
            .ok_or_else(|| bad("no section"))?;
        let SectionKind::Beam {
            material,
            profile:
                BeamProfile::General {
                    area,
                    i11,
                    i22,
                    torsion,
                    ..
                },
            normal,
            ..
        } = &section.kind
        else {
            return Err(bad("needs *BEAM SECTION, SECTION=GENERAL"));
        };
        let mat = model
            .material(material)
            .ok_or_else(|| Error::undefined("*BEAM SECTION", "material", material))?;
        let elastic = LinearElastic::of(mat, temperature)?;
        let nodes = Self::node_indices(model, element)?;
        let x = |i: usize| model.nodes.coordinates[nodes[i] as usize];
        // The fifth value of the section is CalculiX's shear factor k (Nodalis writes it
        // so; the IR calls the field `torsion`).
        let u1 = plxs_elements::u1::U1Section {
            area: *area,
            i11: *i11,
            i22: *i22,
            shear_factor: *torsion,
            young: elastic.young,
            poisson: elastic.poisson,
        };
        let stiffness = plxs_elements::u1::stiffness(x(0), x(2), *normal, &u1)
            .ok_or_else(|| bad("no length, or the section vector is parallel to the axis"))?;
        let dofs = nodes
            .iter()
            .flat_map(|&n| (0..3).map(move |d| n * 3 + d))
            .collect();
        Ok(LinearBlock {
            dofs,
            membrane: false,
            stiffness,
        })
    }

    /// A native M3D3 membrane and its stiffness.
    fn membrane(
        model: &Model,
        element: &Element,
        index: u32,
        temperature: f64,
    ) -> Result<Membrane> {
        let bad = |what: &str| {
            Error::input(
                "*ELEMENT",
                format!("{} element {}: {what}", element.kind, element.id),
            )
        };
        let section = element
            .section
            .and_then(|s| model.sections.get(s as usize))
            .ok_or_else(|| bad("no section"))?;
        let (material, thickness) = match &section.kind {
            SectionKind::Membrane {
                material,
                thickness,
                ..
            } => (material, *thickness),
            SectionKind::Shell {
                material,
                thickness,
                ..
            } => (material, *thickness),
            _ => return Err(bad("needs *MEMBRANE SECTION or *SHELL SECTION")),
        };
        let mat = model
            .material(material)
            .ok_or_else(|| Error::undefined("*MEMBRANE SECTION", "material", material))?;
        let elastic = LinearElastic::of(mat, temperature)?;
        let nodes = Self::node_indices(model, element)?;
        let coordinates: Vec<[f64; 3]> = nodes
            .iter()
            .map(|&n| model.nodes.coordinates[n as usize])
            .collect();
        let formulation = plxs_elements::membrane::Membrane::new(
            coordinates,
            elastic.young,
            elastic.poisson,
            thickness,
            element.kind == ElementType::M3D4R,
        )
        .ok_or_else(|| bad("degenerate element"))?;
        let dofs = nodes
            .iter()
            .flat_map(|&n| (0..3).map(move |d| n * 3 + d))
            .collect();
        Ok(Membrane {
            element: index,
            nodes,
            dofs,
            formulation,
            density: plxs_materials::density(mat, temperature),
            block: usize::MAX,
        })
    }

    fn node_indices(model: &Model, element: &Element) -> Result<Vec<u32>> {
        element
            .nodes
            .iter()
            .map(|&id| {
                model.nodes.index(id).ok_or_else(|| {
                    Error::undefined("*ELEMENT", "node", format!("{id} (element {})", element.id))
                })
            })
            .collect()
    }

    /// The user id of the element of an entry.
    pub fn element_id(&self, model: &Model, entry: u32) -> u32 {
        model.elements[self.entries[entry as usize].element as usize].id
    }

    /// Marks the DOFs that some element (continuum, discrete or native block) uses.
    pub fn active_dofs(&self) -> Vec<bool> {
        let mut active = vec![false; self.dofs.len()];
        for dof in self
            .entries
            .iter()
            .flat_map(|e| &e.dofs)
            .chain(self.discrete.iter().flat_map(|e| &e.dofs))
            .chain(self.linear_blocks.iter().flat_map(|b| &b.dofs))
        {
            active[*dof as usize] = true;
        }
        active
    }

    pub fn formulation(&self, entry: &ElementEntry) -> &SolidElement {
        &self.formulations[entry.formulation as usize]
    }

    pub fn material_matrix(&self, entry: &ElementEntry) -> &[[f64; 6]; 6] {
        &self.materials[entry.material as usize].1
    }

    /// The material law of an element, in its finite-strain form for NLGEOM steps.
    pub fn law(&self, entry: &ElementEntry, finite_strain: bool) -> &dyn MaterialLaw {
        if finite_strain {
            self.laws_finite[entry.material as usize].as_ref()
        } else {
            self.laws[entry.material as usize].as_ref()
        }
    }

    /// Whether any element has a plastic or creeping material (the step then iterates even
    /// without NLGEOM, as CalculiX does).
    pub fn has_nonlinear_material(&self) -> bool {
        self.laws.iter().any(|l| l.is_nonlinear())
    }

    /// Coordinates of the element's nodes.
    pub fn coordinates(&self, model: &Model, entry: &ElementEntry) -> Vec<[f64; 3]> {
        entry
            .nodes
            .iter()
            .map(|&n| model.nodes.coordinates[n as usize])
            .collect()
    }

    /// Element displacements gathered from the global vector.
    pub fn gather(&self, entry: &ElementEntry, u: &[f64]) -> Vec<f64> {
        entry.dofs.iter().map(|&d| u[d as usize]).collect()
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Displacements and nodal forces as they are reported (`.frd`, `.dat`): what the
    /// solver computes, except at the nodes of plane and axisymmetric elements, which report
    /// the thickness average of their expanded nodes' displacements and the sum of their
    /// forces, as CalculiX does (see `plxs_elements::plane`). The internal out-of-plane
    /// unknown of plane stress nodes is no displacement and reported as zero.
    pub fn reported(&self, model: &Model, u: &[f64], forces: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let mut u_out = u.to_vec();
        let mut f_out = forces.to_vec();
        if self.formulations.iter().all(|f| f.expansion().is_none()) {
            return (u_out, f_out);
        }
        let reported: Vec<Option<plxs_elements::ReportedNodal>> = self
            .entries
            .par_iter()
            .map(|entry| {
                let f = self.formulation(entry);
                f.expansion()?;
                let coords = self.coordinates(model, entry);
                f.reported_nodal(&coords, self.material_matrix(entry), &self.gather(entry, u))
            })
            .collect();
        // Plane nodes: swap the element's share of the solver's forces for the reported one.
        for (entry, r) in self.entries.iter().zip(&reported) {
            let Some(r) = r else { continue };
            for a in 0..entry.nodes.len() {
                for k in 0..3 {
                    let dof = entry.dofs[3 * a + k] as usize;
                    u_out[dof] = r.displacements[a][k];
                    f_out[dof] += r.forces[a][k] - r.condensed[3 * a + k];
                }
            }
        }
        (u_out, f_out)
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// The density of the material of a section, if it has one.
fn section_density(model: &Model, section_index: u32) -> Option<f64> {
    let section = &model.sections[section_index as usize];
    let name = section.kind.material()?;
    model
        .material(name)
        .and_then(|m| plxs_materials::density(m, 0.0))
}
