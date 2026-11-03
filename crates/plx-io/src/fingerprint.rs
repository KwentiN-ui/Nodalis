//! Fingerprints of what the solver computes with, taken from the analysis a model is lowered
//! to ([`crate::inp::Lowered`]) rather than from the model: only what reaches the solver
//! counts. Renaming an item, hiding it, a deactivated item that adds nothing, or items only
//! used to look at results change nothing, so the results stay up to date.
//!
//! Names do not count either. Every name of the analysis (sets, surfaces, materials,
//! amplitudes, interactions, ties, steps) is replaced by what it stands for: a set by its
//! members, a material by its values. A set nothing refers to does not count at all, so the
//! extra sets an analysis writes for the analyses continuing from it change nothing.
//!
//! Each step gets the fingerprint of the model and the steps up to it: the solver computes
//! them one after the other, so a change in step 2 leaves the results of step 1 up to date.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use plx_mesh::Fnv;
use plx_model::{FeModel, ModelItem};
use plxs_core as ir;
use serde_json::Value;

use crate::inp::Lowered;

/// The fingerprint of the whole solution and of the results of each step of the model.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Fingerprints {
    /// What the whole solution is computed from.
    pub input: u64,
    /// Per step of the model, in its order: the model and the steps up to it. A deactivated
    /// step has the fingerprint of the steps before it.
    pub steps: Vec<u64>,
}

/// The fingerprints of the analysis lowered from `fe`. The files of other analyses the model
/// reads (global results of a submodel, stored eigenmodes, initial and step temperatures) come
/// from `fe` and count by their folder and file name, which change with the results they are,
/// not with where the work directory is.
pub fn fingerprints(lowered: &Lowered, fe: &FeModel) -> Fingerprints {
    let analysis = &lowered.analysis;
    let names = Names::of(&analysis.model);
    let mut model = Fnv::new();
    mesh(&mut model, &analysis.model);
    let m = &analysis.model;
    for part in [
        names.value(&m.materials),
        names.value(&m.sections),
        names.value(&m.constraints),
        names.value(&m.contact),
        names.value(&m.amplitudes),
        names.value(&m.initial),
        names.value(&m.physical),
        names.value(&m.design_variables),
        names.value(&m.restart_read),
    ] {
        model.add_str(&part.to_string());
    }
    let properties = &fe.properties;
    for file in [
        properties.global_results.as_deref(),
        properties.stored_modes.as_deref(),
        properties
            .initial_temperatures
            .as_ref()
            .map(|(f, _)| f.as_path()),
    ] {
        model.add_str(&file.map(short).unwrap_or_default());
    }
    if let Some((_, step)) = &properties.initial_temperatures {
        model.add(&step.to_le_bytes());
    }
    let model = model.finish();

    // The steps of the solver one after the other, each with those before it.
    let mut after = Vec::with_capacity(analysis.steps.len());
    let mut hash = model;
    for step in &analysis.steps {
        let mut unnamed = step.clone();
        unnamed.name.clear();
        let mut next = Fnv::new();
        next.add(&hash.to_le_bytes());
        next.add_str(&names.value(&unnamed).to_string());
        hash = next.finish();
        after.push(hash);
    }
    // The steps of the model: the last solver step it became (a moving load becomes
    // several), and the files its temperatures are read from.
    let mut last_of = vec![None; fe.steps.len()];
    for (index, _) in analysis.steps.iter().enumerate() {
        if let Some(ModelItem::Step(s)) = lowered.origins.get(&ir::ItemRef::Step(index))
            && let Some(last) = last_of.get_mut(s)
        {
            *last = Some(index);
        }
    }
    let mut steps = Vec::with_capacity(fe.steps.len());
    let mut done = model;
    let mut files = 0u64;
    for (s, step) in fe.steps.iter().enumerate() {
        if let Some(index) = last_of[s] {
            done = after[index];
        }
        for field in step.defined_fields.iter().filter(|f| f.active) {
            if let plx_model::DefinedFieldKind::TemperatureFromFile { file, step } = &field.kind {
                let mut next = Fnv::new();
                next.add(&files.to_le_bytes());
                next.add_str(&short(file));
                next.add(&step.to_le_bytes());
                files = next.finish();
            }
        }
        steps.push(combined(done, files));
    }
    Fingerprints {
        input: combined(after.last().copied().unwrap_or(model), files),
        steps,
    }
}

fn combined(a: u64, b: u64) -> u64 {
    let mut hash = Fnv::new();
    hash.add(&a.to_le_bytes());
    hash.add(&b.to_le_bytes());
    hash.finish()
}

/// A path by its folder and file name.
fn short(path: &Path) -> String {
    let parts: Vec<_> = path.components().rev().take(2).collect();
    parts
        .into_iter()
        .rev()
        .collect::<PathBuf>()
        .display()
        .to_string()
}

/// The nodes and elements, by their ids, coordinates, types and nodes.
fn mesh(hash: &mut Fnv, model: &ir::Model) {
    for (id, coordinates) in model.nodes.ids.iter().zip(&model.nodes.coordinates) {
        hash.add(&id.to_le_bytes());
        for c in coordinates {
            hash.add(&c.to_le_bytes());
        }
    }
    for element in &model.elements {
        hash.add(&element.id.to_le_bytes());
        hash.add_str(&format!("{:?}", element.kind));
        for node in &element.nodes {
            hash.add(&node.to_le_bytes());
        }
    }
}

/// What each name of the analysis stands for.
struct Names(HashMap<String, String>);

impl Names {
    fn of(model: &ir::Model) -> Self {
        let mut names = Names(HashMap::new());
        let members = |prefix: &str, ids: &[u32]| {
            let mut ids = ids.to_vec();
            ids.sort_unstable();
            ids.dedup();
            let mut hash = Fnv::new();
            for id in ids {
                hash.add(&id.to_le_bytes());
            }
            format!("{prefix}{:016x}", hash.finish())
        };
        for (name, ids) in &model.sets.node_sets {
            names.add(name, members("N", ids));
        }
        for (name, ids) in &model.sets.element_sets {
            names.add(name, members("E", ids));
        }
        for (name, surface) in &model.sets.surfaces {
            let mut hash = Fnv::new();
            hash.add_debug(surface);
            names.add(name, format!("S{:016x}", hash.finish()));
        }
        // Then what refers to sets and surfaces itself, by its definition without its name.
        let by_value = |prefix: &str, name: &str, value: Value| {
            let mut hash = Fnv::new();
            hash.add_str(&blank(value, name).to_string());
            (name.to_owned(), format!("{prefix}{:016x}", hash.finish()))
        };
        let mut defined = Vec::new();
        for material in &model.materials {
            defined.push(by_value("M", &material.name, names.value(material)));
        }
        for amplitude in &model.amplitudes {
            defined.push(by_value("A", &amplitude.name, names.value(amplitude)));
        }
        for interaction in &model.contact.interactions {
            defined.push(by_value("I", &interaction.name, names.value(interaction)));
        }
        for tie in &model.constraints.ties {
            defined.push(by_value("T", &tie.name, names.value(tie)));
        }
        for (name, canonical) in defined {
            names.add(&name, canonical);
        }
        names
    }

    /// Adds what `name` stands for; a name of a node set and an element set both stands for
    /// both.
    fn add(&mut self, name: &str, canonical: String) {
        self.0
            .entry(name.to_owned())
            .and_modify(|c| {
                c.push('|');
                c.push_str(&canonical);
            })
            .or_insert(canonical);
    }

    /// `value` with every name replaced by what it stands for.
    fn value(&self, value: &impl serde::Serialize) -> Value {
        let value = serde_json::to_value(value).unwrap_or(Value::Null);
        self.replace(value)
    }

    fn replace(&self, value: Value) -> Value {
        match value {
            Value::String(text) => Value::String(self.0.get(&text).cloned().unwrap_or(text)),
            Value::Array(items) => {
                Value::Array(items.into_iter().map(|v| self.replace(v)).collect())
            }
            Value::Object(map) => Value::Object(
                map.into_iter()
                    .map(|(k, v)| (self.0.get(&k).cloned().unwrap_or(k), self.replace(v)))
                    .collect(),
            ),
            other => other,
        }
    }
}

/// `value` without the string `name` where it is the item's own name.
fn blank(value: Value, name: &str) -> Value {
    match value {
        Value::Object(map) => Value::Object(
            map.into_iter()
                .map(|(k, v)| match v {
                    Value::String(s) if k == "name" && s == name => (k, Value::Null),
                    v => (k, v),
                })
                .collect(),
        ),
        other => other,
    }
}
