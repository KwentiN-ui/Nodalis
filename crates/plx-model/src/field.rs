//! Field outputs of a step, the nodal, element and contact field outputs: the
//! variables the solver writes into the `.frd` file (`*NODE FILE`, `*EL FILE`,
//! `*CONTACT FILE`). The model keeps what the user picked; the keywords are made when the
//! input file is written.

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FieldOutput {
    pub name: String,
    /// A deactivated field output is left out of the input file.
    #[serde(default = "crate::active")]
    pub active: bool,
    pub kind: OutputKind,
    /// CalculiX's variable names, such as `U` or `S`, in a fixed order.
    pub variables: Vec<String>,
    /// Writes the results of every iteration of the last increment, for finding out why a
    /// solution diverges (`LAST ITERATIONS`).
    #[serde(default)]
    pub last_iterations: bool,
    /// Writes the contact elements into a `.cel` file (`CONTACT ELEMENTS`).
    #[serde(default)]
    pub contact_elements: bool,
    /// Results in the global coordinate system; no writes them in the local systems of the
    /// nodes or elements (`GLOBAL=NO`). Contact outputs have no such option.
    #[serde(default = "crate::active")]
    pub global: bool,
    /// How 1D and 2D elements are written; only element outputs have the option.
    #[serde(default)]
    pub dimension: OutputDimension,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum OutputKind {
    /// `*NODE FILE`
    Node,
    /// `*EL FILE`
    Element,
    /// `*CONTACT FILE`
    Contact,
}

/// CalculiX's `OUTPUT` parameter of `*EL FILE`, the "Output (2D/3D)".
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum OutputDimension {
    /// Left to CalculiX, which writes shells and beams as the solids it expands them to.
    #[default]
    Default,
    /// `OUTPUT=2D`
    TwoD,
    /// `OUTPUT=3D`: 1D and 2D elements in their expanded 3D form.
    ThreeD,
}

impl OutputDimension {
    pub const ALL: [OutputDimension; 3] = [Self::Default, Self::TwoD, Self::ThreeD];

    pub fn label(self) -> &'static str {
        match self {
            Self::Default => "Default",
            Self::TwoD => "2D",
            Self::ThreeD => "3D",
        }
    }
}

impl OutputKind {
    pub const ALL: [OutputKind; 3] = [Self::Node, Self::Element, Self::Contact];

    /// Variables the kind can write, in a fixed order.
    pub fn choices(self) -> &'static [&'static str] {
        match self {
            Self::Node => &["RF", "U", "PU", "V", "A", "NT", "PNT", "RFL", "SEN"],
            Self::Element => &[
                "S", "PHS", "E", "ME", "PEEQ", "ENER", "HFL", "ERR", "HER", "ZZS", "SDV",
            ],
            Self::Contact => &["CDIS", "CSTR", "CELS", "PCON"],
        }
    }

    /// The tree's and the dialog's name of the kind.
    pub fn label(self) -> &'static str {
        match self {
            Self::Node => "Node Output",
            Self::Element => "Element Output",
            Self::Contact => "Contact Output",
        }
    }

    /// Prefix of default names, the `NF_Output`, `EF_Output` and `CF_Output`.
    pub fn prefix(self) -> &'static str {
        match self {
            Self::Node => "NF_Output",
            Self::Element => "EF_Output",
            Self::Contact => "CF_Output",
        }
    }

    /// The variables picked for a new field output of the kind.
    pub fn default_variables(self) -> &'static [&'static str] {
        match self {
            Self::Node => &["U"],
            Self::Element => &["S", "E"],
            Self::Contact => &["CDIS", "CSTR"],
        }
    }
}

/// The description of a field or history output variable, shown when hovering over its
/// check box; the history-only variables are described as in the CalculiX manual.
pub fn output_variable_description(variable: &str) -> &'static str {
    match variable {
        "RF" => "Reaction forces.",
        "U" => "Displacements.",
        "PU" => "Displacements magnitude and phase.",
        "V" => "Velocities.",
        "A" => "Accelerations (Dynamic steps).",
        "ALLIE" => "Internal (strain) energy of the whole model.",
        "ALLKE" => "Kinetic energy of the whole model.",
        "ALLWK" => "Work of the external loads since the start of the step.",
        "ALLVD" => "Energy dissipated by the damping since the start of the step.",
        "NT" => "Temperatures.",
        "PNT" => "Temperatures magnitude and phase.",
        "RFL" => "External concentrated heat sources.",
        "SEN" => "Sensitivities of the design responses (sensitivity steps only).",
        "S" => "Stresses.",
        "PHS" => "Stresses magnitude and phase.",
        "E" => "Total strains.",
        "ME" => "Mechanical strains.",
        "PEEQ" => "Equivalent plastic strain.",
        "ENER" => "Energy density.",
        "HFL" => "Heat flux.",
        "ERR" => "Extrapolation error estimator for stress calculations.",
        "HER" => "Extrapolation error estimator for heat calculations.",
        "ZZS" => "Zienkiewicz-Zhu improved stress.",
        "SDV" => "Internal state variables.",
        "CDIS" => "Relative contact displacements.",
        "CSTR" => "Contact stresses.",
        "CELS" => "Contact energy.",
        "PCON" => "Contact states for frequency and steady state dynamics calculations.",
        "ELSE" => "Internal energy of whole elements.",
        "ELKE" => "Kinetic energy of whole elements.",
        "EVOL" => "Volume of whole elements.",
        "EBHE" => "Heat generation of whole elements (body heat flux).",
        "CNUM" => "Number of contact elements.",
        "CF" => "Contact forces summed over the slave surface of the contact pair.",
        _ => "",
    }
}

impl FieldOutput {
    /// A field output of the kind with the given variables and the default options.
    pub fn new(name: impl Into<String>, kind: OutputKind, variables: &[&str]) -> Self {
        Self {
            name: name.into(),
            active: true,
            kind,
            variables: variables.iter().map(|v| v.to_string()).collect(),
            last_iterations: false,
            contact_elements: false,
            global: true,
            dimension: OutputDimension::Default,
        }
    }

    /// Sorts the variables into their fixed order and drops those the kind cannot write.
    pub fn normalize_variables(&mut self) {
        let choices = self.kind.choices();
        self.variables.retain(|v| choices.contains(&v.as_str()));
        self.variables
            .sort_by_key(|v| choices.iter().position(|c| c == v));
        self.variables.dedup();
    }

    /// Why the solver would reject the variables.
    pub fn check_variables(&self) -> Result<(), String> {
        if self.variables.is_empty() {
            return Err("Please select at least one variable.".into());
        }
        let has = |v: &str| self.variables.iter().any(|x| x == v);
        for other in ["ERR", "HER"] {
            if has(other) && has("ZZS") {
                return Err(format!("{other} and ZZS are mutually exclusive."));
            }
        }
        Ok(())
    }

    /// Field outputs added to a new static step. `NOE` is added when writing.
    pub fn defaults() -> Vec<Self> {
        vec![
            Self::new("NF-Output-1", OutputKind::Node, &["RF", "U"]),
            Self::new("EF-Output-1", OutputKind::Element, &["S", "E"]),
        ]
    }

    /// Field outputs of a new visco step: those of a static step and the equivalent creep
    /// strain, which the solver writes as `PEEQ`.
    pub fn visco_defaults() -> Vec<Self> {
        let mut outputs = Self::defaults();
        outputs[1].variables.push("PEEQ".into());
        outputs
    }

    /// Field outputs added to a new heat transfer step: temperatures, reaction heat
    /// flows and heat fluxes.
    pub fn heat_transfer_defaults() -> Vec<Self> {
        let mut outputs = Self::defaults();
        outputs[0].variables = vec!["NT".into(), "RFL".into()];
        outputs[1].variables = vec!["HFL".into()];
        outputs
    }

    /// Field outputs added to a new dynamic step: velocities and accelerations too.
    pub fn dynamic_defaults() -> Vec<Self> {
        let mut outputs = Self::defaults();
        outputs[0].variables = ["RF", "U", "V", "A"].map(String::from).to_vec();
        outputs[1].variables = ["S", "E"].map(String::from).to_vec();
        outputs
    }

    /// Field outputs of a new sensitivity step: the sensitivities only, as the step
    /// computes no displacements or stresses of its own.
    pub fn sensitivity_defaults() -> Vec<Self> {
        vec![Self::new("NF-Output-1", OutputKind::Node, &["SEN"])]
    }

    /// Field outputs added to a new coupled temperature-displacement step.
    pub fn coupled_defaults() -> Vec<Self> {
        let mut outputs = Self::defaults();
        outputs[0].variables = ["RF", "U", "NT", "RFL"].map(String::from).to_vec();
        outputs[1].variables = ["S", "E", "HFL"].map(String::from).to_vec();
        outputs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn projects_without_the_options_load_with_the_defaults() {
        let saved = r#"(name: "NF-Output-1", kind: Node, variables: ["RF", "U"])"#;
        let output: FieldOutput = ron::from_str(saved).unwrap();
        assert_eq!(
            output,
            FieldOutput::new("NF-Output-1", OutputKind::Node, &["RF", "U"])
        );
    }

    #[test]
    fn error_estimators_exclude_improved_stresses() {
        let mut output = FieldOutput::new("EF_Output-1", OutputKind::Element, &["S", "ERR"]);
        assert!(output.check_variables().is_ok());
        output.variables.push("ZZS".into());
        assert!(output.check_variables().is_err());
        output.variables.clear();
        assert!(output.check_variables().is_err());
    }

    #[test]
    fn every_output_variable_has_a_description() {
        let history = [
            crate::HistoryKind::Node {
                region: crate::Region::Nodes(Vec::new()),
            },
            crate::HistoryKind::Element {
                region: crate::Region::Nodes(Vec::new()),
            },
            crate::HistoryKind::Contact { pair: None },
        ];
        let field = OutputKind::ALL.iter().flat_map(|k| k.choices());
        for variable in field.chain(history.iter().flat_map(|k| k.choices())) {
            assert!(
                !output_variable_description(variable).is_empty(),
                "{variable}"
            );
        }
    }

    #[test]
    fn variables_keep_their_order() {
        let mut output = FieldOutput::new("x", OutputKind::Node, &["V", "U", "S", "U"]);
        output.normalize_variables();
        assert_eq!(output.variables, ["U", "V"]);
    }
}
