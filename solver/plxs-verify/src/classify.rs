//! What a fixture tests, derived from the keywords and element types of its input: the area
//! (which milestone thread owns it), the tolerance class and the features it covers.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

/// Tolerance classes of the concept (section 6.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Class {
    /// Linear: displacements, reactions, temperatures, linear stresses and strains,
    /// eigenfrequencies.
    A,
    /// Smooth nonlinear: NLGEOM, plasticity, creep, transient heat; compared at the end of the
    /// step, the number of increments only by order of magnitude.
    B,
    /// Contact: displacements, contact stresses and forces at the end of the step; the
    /// contact status (open/closed) of every node must agree.
    C,
    /// Dynamics: time histories by their norms, peaks within one increment.
    E,
}

impl Class {
    /// Relative tolerance of a value with respect to the largest absolute value of its
    /// component. Class A is 1e-5 rather than the concept's 1e-6: CalculiX writes the ASCII
    /// `.frd` with six significant digits (`%12.5E`), so a bit-exact solver can differ from
    /// the golden file by up to 5e-6 of the value already.
    pub fn tolerance(self) -> f64 {
        match self {
            Class::A => 1e-5,
            Class::B => 1e-3,
            Class::C => 1e-2,
            Class::E => 1e-3,
        }
    }

    pub fn letter(self) -> &'static str {
        match self {
            Class::A => "A",
            Class::B => "B",
            Class::C => "C",
            Class::E => "E",
        }
    }
}

/// The milestone whose threads make a fixture pass.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Stage {
    M1,
    M2,
    M3,
    /// Not computed by plxs in phase 1 (sensitivity); CalculiX stays in charge.
    Ccx,
}

impl Stage {
    pub fn name(self) -> &'static str {
        match self {
            Stage::M1 => "M1",
            Stage::M2 => "M2",
            Stage::M3 => "M3",
            Stage::Ccx => "ccx",
        }
    }
}

/// The `<name>.toml` beside a fixture.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Meta {
    pub name: String,
    /// Directory under `solver/fixtures`.
    pub area: String,
    pub class: Class,
    pub stage: Stage,
    /// Keywords (`*STATIC`) and element types (`C3D8`) the input uses, sorted.
    pub features: Vec<String>,
    /// Where the input came from: `plx-io writer test` or `testdata`.
    pub source: String,
    /// Which CalculiX computed the golden files, `none` when it could not (the fixture
    /// then checks parsing only).
    pub reference: String,
    /// Why there is no reference, when `reference` is `none`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
    /// Documented deviations from CalculiX: `.frd` fields (`FORC`) that are not compared,
    /// each with the reason in `note` and docs/solver/SCHNITTSTELLEN.md.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub deviations: Vec<String>,
}

/// The areas, in the order of precedence a fixture is assigned by: the first area whose
/// marker the input carries wins.
pub const AREAS: &[(&str, &str)] = &[
    ("sensitivity", "*SENSITIVITY (not computed by plxs)"),
    ("cyclic", "cyclic symmetry and submodels"),
    (
        "modal",
        "modal dynamics, steady state dynamics, complex frequency",
    ),
    ("dynamic", "*DYNAMIC"),
    ("contact", "contact pairs"),
    ("heat", "heat transfer, coupled and uncoupled"),
    ("eigen", "*FREQUENCY, *BUCKLE"),
    ("nonlinear", "NLGEOM, plasticity, creep"),
    ("structural", "shells, membranes, beams, trusses, U1"),
    (
        "constraints",
        "equations, rigid bodies, ties, transforms, springs, gaps",
    ),
    ("plane", "plane stress, plane strain, axisymmetric"),
    (
        "loads",
        "amplitudes, gravity, centrifugal, temperatures, several steps",
    ),
    ("static", "linear static on solids"),
];

/// Keywords and element types of an input, upper case, in order of appearance without
/// repetition.
pub fn features_of(input: &str) -> Vec<String> {
    let mut features: Vec<String> = Vec::new();
    let mut seen = BTreeSet::new();
    let mut push = |f: String| {
        if seen.insert(f.clone()) {
            features.push(f);
        }
    };
    let mut nlgeom = false;
    for line in input.lines().map(str::trim) {
        if !line.starts_with('*') || line.starts_with("**") {
            continue;
        }
        let upper = line.to_ascii_uppercase();
        let keyword = upper.split(',').next().unwrap_or_default().trim();
        let keyword = keyword.split_whitespace().collect::<Vec<_>>().join(" ");
        push(keyword.clone());
        if keyword == "*STEP" && upper.contains("NLGEOM") && !upper.contains("NLGEOM=NO") {
            nlgeom = true;
        }
        if keyword == "*ELEMENT"
            && let Some(t) = upper
                .split(',')
                .find_map(|p| p.trim().strip_prefix("TYPE="))
        {
            push(t.trim().to_string());
        }
    }
    if nlgeom {
        push("NLGEOM".into());
    }
    features
}

/// Classifies an input: area, class and stage from its features.
pub fn classify(name: &str, input: &str, source: &str) -> Meta {
    let features = features_of(input);
    let has = |f: &str| features.iter().any(|x| x == f);
    let any = |fs: &[&str]| fs.iter().any(|f| has(f));
    let plane = features
        .iter()
        .any(|f| f.starts_with("CPS") || f.starts_with("CPE") || f.starts_with("CAX"));
    let structural = features.iter().any(|f| {
        matches!(
            f.as_str(),
            "S3" | "S4"
                | "S4R"
                | "S6"
                | "S8"
                | "S8R"
                | "M3D3"
                | "M3D4"
                | "M3D4R"
                | "M3D6"
                | "B31"
                | "B32"
                | "B32R"
                | "T3D2"
                | "U1"
        )
    });
    let steps = input
        .lines()
        .filter(|l| l.trim().to_ascii_uppercase().starts_with("*STEP"))
        .count();
    let (area, stage) = if has("*SENSITIVITY") {
        ("sensitivity", Stage::Ccx)
    } else if any(&["*CYCLIC SYMMETRY MODEL", "*SUBMODEL"]) {
        ("cyclic", Stage::M3)
    } else if any(&[
        "*MODAL DYNAMICS",
        "*STEADY STATE DYNAMICS",
        "*COMPLEX FREQUENCY",
    ]) {
        ("modal", Stage::M3)
    } else if has("*DYNAMIC") {
        ("dynamic", Stage::M3)
    } else if has("*CONTACT PAIR") {
        ("contact", Stage::M3)
    } else if any(&[
        "*HEAT TRANSFER",
        "*COUPLED TEMPERATURE-DISPLACEMENT",
        "*UNCOUPLED TEMPERATURE-DISPLACEMENT",
    ]) {
        ("heat", Stage::M2)
    } else if any(&["*FREQUENCY", "*BUCKLE"]) {
        ("eigen", Stage::M2)
    } else if any(&["NLGEOM", "*PLASTIC", "*VISCO", "*CREEP"]) {
        ("nonlinear", Stage::M2)
    } else if structural {
        ("structural", Stage::M2)
    } else if any(&[
        "*EQUATION",
        "*RIGID BODY",
        "*TIE",
        "*TRANSFORM",
        "*SPRING",
        "*GAP",
    ]) {
        ("constraints", Stage::M2)
    } else if plane {
        ("plane", Stage::M2)
    } else if any(&["*AMPLITUDE", "*TEMPERATURE", "*INITIAL CONDITIONS"])
        || features.iter().any(|f| f == "*DLOAD") && input.to_ascii_uppercase().contains(", GRAV,")
        || input.to_ascii_uppercase().contains(", CENTRIF,")
        || steps > 1
    {
        ("loads", Stage::M2)
    } else {
        ("static", Stage::M1)
    };
    let class = if has("*CONTACT PAIR") {
        Class::C
    } else if any(&["*DYNAMIC", "*MODAL DYNAMICS", "*STEADY STATE DYNAMICS"]) {
        Class::E
    } else if any(&["NLGEOM", "*PLASTIC", "*VISCO", "*CREEP"])
        || (any(&[
            "*HEAT TRANSFER",
            "*COUPLED TEMPERATURE-DISPLACEMENT",
            "*UNCOUPLED TEMPERATURE-DISPLACEMENT",
        ]) && !input.to_ascii_uppercase().contains("STEADY STATE"))
        || has("*RADIATE")
        || has("*GAP")
    {
        Class::B
    } else {
        Class::A
    };
    Meta {
        name: name.to_string(),
        area: area.to_string(),
        class,
        stage,
        features,
        source: source.to_string(),
        reference: "none".into(),
        note: None,
        deviations: Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_contact_input_lands_in_contact_with_class_c() {
        let input = "*Node\n*Element, Type=C3D8, Elset=E\n*Contact pair, Interaction=I, Type=Surface to surface\n*Step, Nlgeom\n*Static\n*End step\n";
        let meta = classify("x", input, "test");
        assert_eq!(meta.area, "contact");
        assert_eq!(meta.class, Class::C);
        assert_eq!(meta.stage, Stage::M3);
        assert!(meta.features.contains(&"C3D8".to_string()));
        assert!(meta.features.contains(&"NLGEOM".to_string()));
    }

    #[test]
    fn a_plain_static_input_is_m1() {
        let input =
            "*Node\n*Element, Type=C3D20R, Elset=E\n*Step\n*Static\n*Cload\n1, 3, -1\n*End step\n";
        let meta = classify("x", input, "test");
        assert_eq!(
            (meta.area.as_str(), meta.class, meta.stage),
            ("static", Class::A, Stage::M1)
        );
    }
}
