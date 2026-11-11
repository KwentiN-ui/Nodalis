//! History outputs of a step, the nodal, element and contact history outputs: values
//! the solver prints into the `.dat` file at every increment (`*NODE PRINT`, `*EL PRINT`,
//! `*CONTACT PRINT`). The model keeps what the user picked; the sets and keywords are made
//! when the input file is written.

use serde::{Deserialize, Serialize};

use crate::Region;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HistoryOutput {
    /// The internal identity the steps that print the output refer to: one output of the
    /// model, printed in every step that lists it.
    pub id: crate::Id,
    pub name: String,
    /// A deactivated history output is left out of the input file.
    #[serde(default = "crate::active")]
    pub active: bool,
    pub kind: HistoryKind,
    /// CalculiX's variable names, such as `U` or `RF`, in a fixed order.
    pub variables: Vec<String>,
    pub totals: Totals,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum HistoryKind {
    /// Values at the nodes of a region (`*NODE PRINT`).
    Node { region: Region },
    /// Values at the integration points or of whole elements of a region (`*EL PRINT`).
    Element { region: Region },
    /// Values of all contact elements (`*CONTACT PRINT`); the contact pair gives master and
    /// slave of the contact forces `CF`.
    Contact { pair: Option<crate::Id> },
    /// Energies of the whole model over the time of a Dynamic step: internal, kinetic,
    /// external work and damping dissipation (`*EL PRINT` of all elements, `TOTALS=ONLY`).
    Energies,
}

/// Whether sums over the region are printed, CalculiX's `TOTALS` parameter.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Totals {
    /// Only the values of every node or element.
    #[default]
    No,
    /// The sum in addition to the values.
    Yes,
    /// Only the sum.
    Only,
}

impl Totals {
    pub const ALL: [Totals; 3] = [Totals::No, Totals::Yes, Totals::Only];

    pub fn label(self) -> &'static str {
        match self {
            Totals::No => "No",
            Totals::Yes => "Yes",
            Totals::Only => "Only",
        }
    }
}

impl HistoryKind {
    /// Variables the kind can print, in a fixed order.
    pub fn choices(&self) -> &'static [&'static str] {
        match self {
            HistoryKind::Node { .. } => &["RF", "U", "V", "NT", "RFL"],
            HistoryKind::Element { .. } => &[
                "S", "E", "ME", "PEEQ", "HFL", "ENER", "ELSE", "ELKE", "EVOL", "EBHE",
            ],
            HistoryKind::Contact { .. } => &["CDIS", "CSTR", "CELS", "CNUM", "CF"],
            HistoryKind::Energies => &["ALLIE", "ALLKE", "ALLWK", "ALLVD"],
        }
    }

    /// The tree's and the dialog's name of the kind.
    pub fn label(&self) -> &'static str {
        match self {
            HistoryKind::Node { .. } => "Node Output",
            HistoryKind::Element { .. } => "Element Output",
            HistoryKind::Contact { .. } => "Contact Output",
            HistoryKind::Energies => "Energies",
        }
    }

    /// Prefix of default names, the `NH_Output`, `EH_Output` and `CH_Output`.
    pub fn prefix(&self) -> &'static str {
        match self {
            HistoryKind::Node { .. } => "NH_Output",
            HistoryKind::Element { .. } => "EH_Output",
            HistoryKind::Contact { .. } => "CH_Output",
            HistoryKind::Energies => "Energies",
        }
    }

    pub fn region(&self) -> Option<&Region> {
        match self {
            HistoryKind::Node { region } | HistoryKind::Element { region } => Some(region),
            HistoryKind::Contact { .. } | HistoryKind::Energies => None,
        }
    }

    pub fn region_mut(&mut self) -> Option<&mut Region> {
        match self {
            HistoryKind::Node { region } | HistoryKind::Element { region } => Some(region),
            HistoryKind::Contact { .. } | HistoryKind::Energies => None,
        }
    }
}

impl HistoryOutput {
    /// The default nodal history output: reaction forces and displacements.
    pub fn node(name: impl Into<String>, region: Region) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            active: true,
            kind: HistoryKind::Node { region },
            variables: vec!["RF".into(), "U".into()],
            totals: Totals::No,
        }
    }

    /// The default element history output: stresses and strains.
    pub fn element(name: impl Into<String>, region: Region) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            active: true,
            kind: HistoryKind::Element { region },
            variables: vec!["S".into(), "E".into()],
            totals: Totals::No,
        }
    }

    /// The default contact history output: contact displacements and stresses.
    pub fn contact(name: impl Into<String>, pair: Option<crate::Id>) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            active: true,
            kind: HistoryKind::Contact { pair },
            variables: vec!["CDIS".into(), "CSTR".into()],
            totals: Totals::No,
        }
    }

    /// The energies of the whole model, which a new Dynamic step prints.
    pub fn energies(name: impl Into<String>) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            active: true,
            kind: HistoryKind::Energies,
            variables: HistoryKind::Energies
                .choices()
                .iter()
                .map(|v| v.to_string())
                .collect(),
            totals: Totals::Only,
        }
    }

    /// Sorts the variables into their fixed order and drops those the kind cannot print.
    pub fn normalize_variables(&mut self) {
        let choices = self.kind.choices();
        self.variables.retain(|v| choices.contains(&v.as_str()));
        self.variables
            .sort_by_key(|v| choices.iter().position(|c| c == v));
        self.variables.dedup();
    }
}

impl crate::FeModel {
    /// The history output of the model with the id.
    pub fn history_output(&self, id: crate::Id) -> Option<&HistoryOutput> {
        self.history_outputs.iter().find(|o| o.id == id)
    }

    pub fn history_output_mut(&mut self, id: crate::Id) -> Option<&mut HistoryOutput> {
        self.history_outputs.iter_mut().find(|o| o.id == id)
    }

    /// The history outputs step `s` prints, in its order; the tree's index of each is its
    /// place in this list.
    pub fn step_history_outputs(&self, s: usize) -> Vec<&HistoryOutput> {
        let ids = self
            .steps
            .get(s)
            .map(|step| step.history_outputs.as_slice());
        (ids.unwrap_or_default().iter())
            .filter_map(|&id| self.history_output(id))
            .collect()
    }

    /// The `i`th history output of step `s`.
    pub fn step_history_output(&self, s: usize, i: usize) -> Option<&HistoryOutput> {
        let id = *self.steps.get(s)?.history_outputs.get(i)?;
        self.history_output(id)
    }

    pub fn step_history_output_mut(&mut self, s: usize, i: usize) -> Option<&mut HistoryOutput> {
        let id = *self.steps.get(s)?.history_outputs.get(i)?;
        self.history_output_mut(id)
    }

    /// The steps that print the history output.
    pub fn history_output_steps(&self, id: crate::Id) -> Vec<usize> {
        (self.steps.iter().enumerate())
            .filter(|(_, step)| step.history_outputs.contains(&id))
            .map(|(s, _)| s)
            .collect()
    }

    /// Adds `output` to the model and to step `s`, in place of the output of the model
    /// with its id.
    pub fn put_history_output(&mut self, s: usize, output: HistoryOutput) {
        let id = output.id;
        match self.history_output_mut(id) {
            Some(existing) => *existing = output,
            None => self.history_outputs.push(output),
        }
        if let Some(step) = self.steps.get_mut(s)
            && !step.history_outputs.contains(&id)
        {
            step.history_outputs.push(id);
        }
    }

    /// Takes the history output out of the model and out of every step.
    pub fn delete_history_output(&mut self, id: crate::Id) {
        self.history_outputs.retain(|o| o.id != id);
        for step in &mut self.steps {
            step.history_outputs.retain(|&o| o != id);
        }
    }

    /// Takes over from `from` the history outputs the steps print, in place of those of
    /// the same id, for steps taken from another analysis; drops those no step prints any
    /// more.
    pub fn adopt_history_outputs(&mut self, from: &crate::FeModel) {
        let printed: Vec<crate::Id> = (self.steps.iter())
            .flat_map(|s| s.history_outputs.iter().copied())
            .collect();
        for output in (from.history_outputs.iter()).filter(|o| printed.contains(&o.id)) {
            match self.history_output_mut(output.id) {
                Some(existing) => *existing = output.clone(),
                None => self.history_outputs.push(output.clone()),
            }
        }
        self.drop_unused_history_outputs();
    }

    /// The history outputs `steps` print, in the order of the model.
    pub fn printed_history_outputs(&self, steps: &[crate::Step]) -> Vec<&HistoryOutput> {
        (self.history_outputs.iter())
            .filter(|o| steps.iter().any(|s| s.history_outputs.contains(&o.id)))
            .collect()
    }

    /// Takes the history outputs no step prints out of the model.
    pub fn drop_unused_history_outputs(&mut self) {
        let used: std::collections::BTreeSet<crate::Id> = (self.steps.iter())
            .flat_map(|s| s.history_outputs.iter().copied())
            .collect();
        self.history_outputs.retain(|o| used.contains(&o.id));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steps_share_the_outputs_of_the_model() {
        let mut fe = crate::FeModel {
            steps: vec![crate::Step::new_static("A"), crate::Step::new_static("B")],
            ..Default::default()
        };
        let output = HistoryOutput::node("NH_Output-1", Region::Nodes(vec![1]));
        let id = output.id;
        fe.put_history_output(0, output);
        fe.steps[1].history_outputs.push(id);
        fe.step_history_output_mut(1, 0).unwrap().active = false;
        assert!(!fe.step_history_outputs(0)[0].active);
        assert_eq!(fe.history_output_steps(id), [0, 1]);
        fe.steps[0].history_outputs.clear();
        fe.drop_unused_history_outputs();
        assert_eq!(fe.history_outputs.len(), 1);
        fe.delete_history_output(id);
        assert!(fe.history_outputs.is_empty() && fe.steps[1].history_outputs.is_empty());
    }

    #[test]
    fn variables_keep_their_order() {
        let mut output = HistoryOutput::node("NH_Output-1", Region::Nodes(vec![1]));
        output.variables = vec!["U".into(), "XX".into(), "RF".into(), "U".into()];
        output.normalize_variables();
        assert_eq!(output.variables, ["RF", "U"]);
    }
}
