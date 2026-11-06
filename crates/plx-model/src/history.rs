//! History outputs of a step, the nodal, element and contact history outputs: values
//! the solver prints into the `.dat` file at every increment (`*NODE PRINT`, `*EL PRINT`,
//! `*CONTACT PRINT`). The model keeps what the user picked; the sets and keywords are made
//! when the input file is written.

use serde::{Deserialize, Serialize};

use crate::Region;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HistoryOutput {
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
    Contact { pair: String },
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
            name: name.into(),
            active: true,
            kind: HistoryKind::Element { region },
            variables: vec!["S".into(), "E".into()],
            totals: Totals::No,
        }
    }

    /// The default contact history output: contact displacements and stresses.
    pub fn contact(name: impl Into<String>, pair: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            kind: HistoryKind::Contact { pair: pair.into() },
            variables: vec!["CDIS".into(), "CSTR".into()],
            totals: Totals::No,
        }
    }

    /// The energies of the whole model, which a new Dynamic step prints.
    pub fn energies(name: impl Into<String>) -> Self {
        Self {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn variables_keep_their_order() {
        let mut output = HistoryOutput::node("NH_Output-1", Region::Nodes(vec![1]));
        output.variables = vec!["U".into(), "XX".into(), "RF".into(), "U".into()];
        output.normalize_variables();
        assert_eq!(output.variables, ["RF", "U"]);
    }

    #[test]
    fn old_projects_have_no_history_outputs() {
        // A step saved before history outputs existed still loads.
        let text = ron::to_string(&crate::Step::new_static("Step-1")).unwrap();
        let old = text.replace(",history_outputs:[]", "");
        assert_ne!(old, text);
        let step: crate::Step = ron::from_str(&old).unwrap();
        assert!(step.history_outputs.is_empty());
    }
}
