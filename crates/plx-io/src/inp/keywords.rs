//! The CalculiX keywords Nodalis writes, each with the feature that needs it, so that the
//! self test can say which features an older or reduced CalculiX build cannot compute.
//!
//! A new step type or feature that writes a new keyword adds it to [`WRITTEN_KEYWORDS`]; a
//! test checks that every keyword in the input file writer is listed.

/// Keywords the input file writer emits, spelled as CalculiX documents them, with the
/// Nodalis feature that writes them. Keywords of the basic static analysis come first.
pub const WRITTEN_KEYWORDS: &[(&str, &str)] = &[
    ("*HEADING", "Every analysis"),
    ("*NODE", "Every analysis"),
    ("*ELEMENT", "Every analysis"),
    ("*NSET", "Every analysis"),
    ("*ELSET", "Every analysis"),
    ("*SURFACE", "Every analysis"),
    ("*MATERIAL", "Every analysis"),
    ("*ELASTIC", "Every analysis"),
    ("*SOLID SECTION", "Every analysis"),
    ("*STEP", "Every analysis"),
    ("*STATIC", "Static step"),
    ("*BOUNDARY", "Boundary conditions"),
    ("*CLOAD", "Loads"),
    ("*DLOAD", "Loads"),
    ("*NODE FILE", "Field outputs"),
    ("*EL FILE", "Field outputs"),
    ("*END STEP", "Every analysis"),
    ("*NO ANALYSIS", "Model check"),
    ("*DENSITY", "Material density"),
    ("*PLASTIC", "Plastic material"),
    ("*EXPANSION", "Thermal expansion"),
    ("*CONDUCTIVITY", "Thermal material"),
    ("*SPECIFIC HEAT", "Thermal material"),
    ("*PHYSICAL CONSTANTS", "Radiation"),
    ("*AMPLITUDE", "Amplitudes"),
    ("*INITIAL CONDITIONS", "Initial conditions"),
    (
        "*TRANSFORM",
        "Boundary conditions in a local coordinate system",
    ),
    ("*SUBMODEL", "Submodel"),
    ("*SHELL SECTION", "Shell sections"),
    ("*MEMBRANE SECTION", "Membrane sections"),
    ("*BEAM SECTION", "Beam sections"),
    (
        "*USER ELEMENT",
        "General beam profiles (I, T, U, L, General)",
    ),
    ("*SPRING", "Springs"),
    ("*GAP", "Compression only support"),
    ("*EQUATION", "Node ties and constraints"),
    ("*RIGID BODY", "Rigid body"),
    ("*TIE", "Tie"),
    ("*PRE-TENSION SECTION", "Bolt pre-tension"),
    ("*SURFACE INTERACTION", "Contact"),
    ("*SURFACE BEHAVIOR", "Contact"),
    ("*CONTACT PAIR", "Contact"),
    ("*CLEARANCE", "Contact offset"),
    ("*MODEL CHANGE", "Contacts switched in steps"),
    ("*CHANGE FRICTION", "Friction changed in steps"),
    ("*FRICTION", "Contact with friction"),
    ("*GAP CONDUCTANCE", "Thermal contact"),
    ("*CONTACT FILE", "Contact field outputs"),
    ("*CONTACT PRINT", "Contact history outputs"),
    ("*NODE PRINT", "History outputs"),
    ("*EL PRINT", "History outputs"),
    ("*TEMPERATURE", "Defined temperature fields"),
    ("*FREQUENCY", "Frequency step"),
    ("*COMPLEX FREQUENCY", "Complex frequency step"),
    ("*BUCKLE", "Buckle step"),
    ("*HEAT TRANSFER", "Heat transfer step"),
    (
        "*COUPLED TEMPERATURE-DISPLACEMENT",
        "Coupled temperature-displacement step",
    ),
    (
        "*UNCOUPLED TEMPERATURE-DISPLACEMENT",
        "Uncoupled temperature-displacement step",
    ),
    ("*CFLUX", "Thermal loads"),
    ("*DFLUX", "Thermal loads"),
    ("*FILM", "Thermal loads"),
    ("*RADIATE", "Thermal loads"),
    ("*DYNAMIC", "Dynamic step"),
    ("*DAMPING", "Dynamic step with damping"),
    ("*MODAL DYNAMICS", "Modal dynamics step"),
    ("*STEADY STATE DYNAMICS", "Steady state dynamics step"),
    ("*MODAL DAMPING", "Modal damping"),
    ("*VISCO", "Visco step"),
    ("*CREEP", "Creep material"),
    ("*CYCLIC SYMMETRY MODEL", "Cyclic symmetry"),
    ("*SELECT CYCLIC SYMMETRY MODES", "Cyclic symmetry"),
    ("*SENSITIVITY", "Sensitivity step"),
    ("*DESIGN VARIABLES", "Sensitivity step"),
    ("*DESIGN RESPONSE", "Sensitivity step"),
    ("*FILTER", "Sensitivity step"),
    ("*OBJECTIVE", "Sensitivity step"),
];

/// A keyword as CalculiX compares it: upper case without blanks (`*Node file` → `*NODEFILE`).
pub fn normalized(keyword: &str) -> String {
    let name = keyword.split(',').next().unwrap_or_default();
    name.split_whitespace()
        .collect::<String>()
        .to_ascii_uppercase()
}

/// The feature that writes `keyword`, if it is one of [`WRITTEN_KEYWORDS`].
pub fn feature(keyword: &str) -> Option<&'static str> {
    let keyword = normalized(keyword);
    (WRITTEN_KEYWORDS.iter())
        .find(|(k, _)| normalized(k) == keyword)
        .map(|&(_, feature)| feature)
}

/// The distinct keywords of an input file in the order they first appear, in upper case with
/// single blanks (`*NODE FILE`); comment lines (`**`) are skipped.
pub fn input_keywords(input: &str) -> Vec<String> {
    let mut keywords: Vec<String> = Vec::new();
    for line in input.lines().map(str::trim_start) {
        if !line.starts_with('*') || line.starts_with("**") {
            continue;
        }
        let name = line.split(',').next().unwrap_or_default();
        let keyword = name.split_whitespace().collect::<Vec<_>>().join(" ");
        let keyword = keyword.to_ascii_uppercase();
        if keyword.len() > 1
            && !keywords
                .iter()
                .any(|k| normalized(k) == normalized(&keyword))
        {
            keywords.push(keyword);
        }
    }
    keywords
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keywords_of_an_input_file() {
        let input =
            "** comment\n*Heading\nTest\n*Node file\nU\n*NODE FILE, Nset=A\n*node   print\n";
        assert_eq!(
            input_keywords(input),
            ["*HEADING", "*NODE FILE", "*NODE PRINT"]
        );
        assert_eq!(normalized("*Node print, Nset=A"), "*NODEPRINT");
        assert_eq!(
            feature("*Complex frequency, Coriolis"),
            Some("Complex frequency step")
        );
        assert_eq!(feature("*UNKNOWN"), None);
    }

    /// Every keyword literal in the writer's source needs a feature, so that new features
    /// cannot forget the table.
    #[test]
    fn every_written_keyword_is_listed() {
        let sources = [
            include_str!("write.rs"),
            include_str!("write/constraints.rs"),
        ];
        let mut missing = Vec::new();
        for source in sources {
            for (at, _) in source.match_indices("\"*") {
                let rest = &source[at + 1..];
                if rest.starts_with("**") {
                    continue;
                }
                let end = rest[1..]
                    .find(|c: char| !(c.is_ascii_alphabetic() || c == ' ' || c == '-'))
                    .map_or(rest.len(), |e| e + 1);
                let keyword = rest[..end].trim_end();
                // `format!("*{keyword}, op=New")` and the like.
                if keyword.len() > 1 && feature(keyword).is_none() {
                    missing.push(keyword.to_string());
                }
            }
        }
        assert!(missing.is_empty(), "not in WRITTEN_KEYWORDS: {missing:?}");
    }

    #[test]
    fn listed_once() {
        for (i, (keyword, _)) in WRITTEN_KEYWORDS.iter().enumerate() {
            assert!(
                !WRITTEN_KEYWORDS[..i].iter().any(|(k, _)| k == keyword),
                "{keyword}"
            );
        }
    }
}
