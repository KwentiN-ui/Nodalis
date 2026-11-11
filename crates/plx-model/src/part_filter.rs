//! Choosing parts by rule rather than one by one: by name, a name pattern, the material of
//! their section or a part group. A large assembly has hundreds of parts, which no list of
//! checkboxes keeps apart; a filter says which of them a contact detection searches and
//! which pairs its rules apply to, and keeps doing so when parts are added or renamed.

use std::collections::{BTreeSet, HashMap};

use plx_mesh::FeMesh;
use serde::{Deserialize, Serialize};

use crate::{FeModel, Region};

/// One way of naming parts.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PartTerm {
    /// The part of this name.
    Part(String),
    /// Parts whose name matches the pattern, ignoring case: `*` stands for any text and `?`
    /// for one character; a pattern without them matches names containing it.
    Pattern(String),
    /// Parts with elements of a section of this material.
    Material(crate::Id),
    /// The parts of this part group.
    Group(crate::Id),
}

/// Parts chosen by terms: those any of `include` names, all parts if it names none, less
/// those any of `exclude` names.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PartFilter {
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub include: Vec<PartTerm>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub exclude: Vec<PartTerm>,
}

/// A named set of parts, kept as a filter so that it follows the parts of the model.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PartGroup {
    /// The internal identity other items refer to; the user sees only the name.
    pub id: crate::Id,
    pub name: String,
    pub filter: PartFilter,
}

impl PartGroup {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            filter: PartFilter::default(),
        }
    }
}

/// What a filter looks at of each part of a mesh: its name and the materials of its
/// sections, gathered once for the many filters of a detection.
#[derive(Clone, Debug, Default)]
pub struct PartContext {
    pub names: Vec<String>,
    pub materials: Vec<BTreeSet<crate::Id>>,
    groups: HashMap<crate::Id, PartFilter>,
}

impl PartContext {
    pub fn len(&self) -> usize {
        self.names.len()
    }

    pub fn is_empty(&self) -> bool {
        self.names.is_empty()
    }
}

/// How deep part groups may refer to each other; deeper, a group matches nothing, which
/// keeps a group naming itself from looping.
const GROUP_DEPTH: usize = 8;

impl PartFilter {
    /// The filter of all parts.
    pub fn all() -> Self {
        Self::default()
    }

    /// The filter of these parts by name.
    pub fn parts<S: Into<String>>(names: impl IntoIterator<Item = S>) -> Self {
        Self {
            include: names
                .into_iter()
                .map(|n| PartTerm::Part(n.into()))
                .collect(),
            exclude: Vec::new(),
        }
    }

    /// Whether it takes every part.
    pub fn is_all(&self) -> bool {
        self.include.is_empty() && self.exclude.is_empty()
    }

    /// Whether the part `part` of the context passes the filter.
    pub fn matches(&self, part: usize, context: &PartContext) -> bool {
        self.matches_at(part, context, 0)
    }

    fn matches_at(&self, part: usize, context: &PartContext, depth: usize) -> bool {
        if depth > GROUP_DEPTH {
            return false;
        }
        let named = |term: &PartTerm| term.matches(part, context, depth);
        (self.include.is_empty() || self.include.iter().any(named))
            && !self.exclude.iter().any(named)
    }

    /// Of each part of the context, whether it passes the filter.
    pub fn select(&self, context: &PartContext) -> Vec<bool> {
        (0..context.len())
            .map(|p| self.matches(p, context))
            .collect()
    }

    /// Of each part of the context, whether a search of the filter takes it: the parts it
    /// matches, of the visible ones if it names none to include.
    pub fn searched(&self, context: &PartContext, visible: &[bool]) -> Vec<bool> {
        (0..context.len())
            .map(|i| {
                (!self.include.is_empty() || visible.get(i).copied().unwrap_or(true))
                    && self.matches(i, context)
            })
            .collect()
    }

    /// Whether the filter refers to the material or part group of this id.
    pub fn refers_to(&self, id: crate::Id) -> bool {
        (self.include.iter().chain(&self.exclude)).any(|t| match t {
            PartTerm::Material(m) | PartTerm::Group(m) => *m == id,
            _ => false,
        })
    }

    /// Drops the terms naming the material or part group `id`, which was deleted.
    pub fn forget(&mut self, id: crate::Id) {
        let keep =
            |t: &PartTerm| !matches!(t, PartTerm::Material(m) | PartTerm::Group(m) if *m == id);
        self.include.retain(keep);
        self.exclude.retain(keep);
    }

    /// Renames the part `old` to `new` in its terms.
    pub fn rename_part(&mut self, old: &str, new: &str) {
        for term in self.include.iter_mut().chain(&mut self.exclude) {
            if let PartTerm::Part(name) = term
                && name == old
            {
                *name = new.to_string();
            }
        }
    }

    /// Replaces the part `old` by the parts `new` in its terms.
    pub fn split_part(&mut self, old: &str, new: &[String]) {
        for terms in [&mut self.include, &mut self.exclude] {
            if let Some(at) = terms.iter().position(|t| *t == PartTerm::Part(old.into())) {
                terms.splice(at..=at, new.iter().map(|n| PartTerm::Part(n.clone())));
            }
        }
    }

    /// A short text for lists, such as "Bolt*, Steel except Group A".
    pub fn describe(&self, fe: &FeModel) -> String {
        let terms = |terms: &[PartTerm]| {
            (terms.iter())
                .map(|t| t.describe(fe))
                .collect::<Vec<_>>()
                .join(", ")
        };
        let mut text = if self.include.is_empty() {
            "Any".to_string()
        } else {
            terms(&self.include)
        };
        if !self.exclude.is_empty() {
            text += " except ";
            text += &terms(&self.exclude);
        }
        text
    }
}

impl PartTerm {
    fn matches(&self, part: usize, context: &PartContext, depth: usize) -> bool {
        match self {
            PartTerm::Part(name) => context.names.get(part).is_some_and(|n| n == name),
            PartTerm::Pattern(pattern) => context
                .names
                .get(part)
                .is_some_and(|n| name_matches(pattern, n)),
            PartTerm::Material(id) => context.materials.get(part).is_some_and(|m| m.contains(id)),
            PartTerm::Group(id) => (context.groups.get(id))
                .is_some_and(|filter| filter.matches_at(part, context, depth + 1)),
        }
    }

    /// The term as the user reads it.
    pub fn describe(&self, fe: &FeModel) -> String {
        match self {
            PartTerm::Part(name) => name.clone(),
            PartTerm::Pattern(pattern) => format!("Name: {pattern}"),
            PartTerm::Material(id) => match fe.material(*id) {
                Some(m) => format!("Material: {}", m.name),
                None => "Material: (deleted)".into(),
            },
            PartTerm::Group(id) => match fe.part_groups.iter().find(|g| g.id == *id) {
                Some(g) => format!("Group: {}", g.name),
                None => "Group: (deleted)".into(),
            },
        }
    }
}

/// Whether `name` matches `pattern`, ignoring case: `*` any text, `?` one character; a
/// pattern without them matches names containing it.
pub fn name_matches(pattern: &str, name: &str) -> bool {
    let pattern = pattern.trim().to_lowercase();
    let name = name.to_lowercase();
    if pattern.is_empty() {
        return false;
    }
    if !pattern.contains(['*', '?']) {
        return name.contains(&pattern);
    }
    let (p, n): (Vec<char>, Vec<char>) = (pattern.chars().collect(), name.chars().collect());
    // Greedy match with backtracking to the last star.
    let (mut i, mut j) = (0, 0);
    let (mut star, mut mark) = (None, 0);
    while j < n.len() {
        if i < p.len() && (p[i] == '?' || p[i] == n[j]) {
            i += 1;
            j += 1;
        } else if i < p.len() && p[i] == '*' {
            star = Some(i);
            mark = j;
            i += 1;
        } else if let Some(s) = star {
            i = s + 1;
            mark += 1;
            j = mark;
        } else {
            return false;
        }
    }
    p[i..].iter().all(|&c| c == '*')
}

impl FeModel {
    /// The parts of `mesh` as filters see them: their names, the materials of their
    /// sections and the part groups of the model.
    pub fn part_context(&self, mesh: &FeMesh) -> PartContext {
        let index: HashMap<&str, usize> = (mesh.parts.iter().enumerate())
            .map(|(i, p)| (p.name.as_str(), i))
            .collect();
        let mut materials = vec![BTreeSet::new(); mesh.parts.len()];
        let mut part_of = HashMap::new();
        for section in &self.sections {
            let Some(material) = section.material else {
                continue;
            };
            // Sections on whole parts are found by name, so that the display of the CAD
            // geometry, which has the parts but none of their elements, sees them too.
            if let Region::Parts(names) = &section.region {
                for name in names {
                    if let Some(&p) = index.get(name.as_str()) {
                        materials[p].insert(material);
                    }
                }
                continue;
            }
            if part_of.is_empty() {
                for (p, part) in mesh.parts.iter().enumerate() {
                    for &e in &part.elements {
                        part_of.insert(e, p);
                    }
                }
            }
            for element in section.region.elements(mesh) {
                if let Some(&p) = part_of.get(&element) {
                    materials[p].insert(material);
                }
            }
        }
        PartContext {
            names: mesh.parts.iter().map(|p| p.name.clone()).collect(),
            materials,
            groups: (self.part_groups.iter())
                .map(|g| (g.id, g.filter.clone()))
                .collect(),
        }
    }

    /// Part filters naming the part `old` name the parts `new`.
    pub(crate) fn split_part_in_filters(&mut self, old: &str, new: &[String]) {
        let filters = (self.contact_detections.iter_mut())
            .flat_map(|d| {
                std::iter::once(&mut d.scope)
                    .chain((d.rules.iter_mut()).flat_map(|r| [&mut r.side_a, &mut r.side_b]))
            })
            .chain(self.part_groups.iter_mut().map(|g| &mut g.filter));
        for filter in filters {
            filter.split_part(old, new);
        }
    }

    /// Drops the filter terms naming the material or part group `id`, which was deleted.
    pub fn forget_in_filters(&mut self, id: crate::Id) {
        for detection in &mut self.contact_detections {
            detection.forget(id);
        }
        for group in &mut self.part_groups {
            group.filter.forget(id);
        }
    }

    pub fn part_group(&self, id: crate::Id) -> Option<&PartGroup> {
        self.part_groups.iter().find(|g| g.id == id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn context(names: &[&str], groups: &[PartGroup]) -> PartContext {
        PartContext {
            names: names.iter().map(|n| n.to_string()).collect(),
            materials: vec![BTreeSet::new(); names.len()],
            groups: groups.iter().map(|g| (g.id, g.filter.clone())).collect(),
        }
    }

    #[test]
    fn patterns_match_ignoring_case() {
        assert!(name_matches("bolt*", "Bolt_M12_1"));
        assert!(name_matches("*M12*", "bolt_m12_1"));
        assert!(name_matches("Bolt_?", "Bolt_3"));
        assert!(!name_matches("Bolt_?", "Bolt_13"));
        assert!(name_matches("m12", "Bolt_M12_1"));
        assert!(!name_matches("*nut", "Bolt_M12_1"));
        assert!(!name_matches("", "Bolt"));
    }

    #[test]
    fn filters_include_then_exclude_and_follow_groups() {
        let mut bolts = PartGroup::new("Bolts");
        bolts.filter.include.push(PartTerm::Pattern("bolt*".into()));
        let mut not_long = PartGroup::new("Short");
        not_long.filter.include.push(PartTerm::Group(bolts.id));
        not_long
            .filter
            .exclude
            .push(PartTerm::Pattern("*long".into()));
        let mut myself = PartGroup::new("Loop");
        myself.filter.include.push(PartTerm::Group(myself.id));
        let groups = [bolts.clone(), not_long.clone(), myself.clone()];
        let ctx = context(&["Bolt1", "Bolt2long", "Plate"], &groups);
        assert_eq!(PartFilter::all().select(&ctx), [true, true, true]);
        let filter = PartFilter {
            include: vec![PartTerm::Group(not_long.id)],
            exclude: Vec::new(),
        };
        assert_eq!(filter.select(&ctx), [true, false, false]);
        let filter = PartFilter {
            include: Vec::new(),
            exclude: vec![PartTerm::Group(bolts.id)],
        };
        assert_eq!(filter.select(&ctx), [false, false, true]);
        // A group naming itself matches nothing rather than looping.
        let filter = PartFilter {
            include: vec![PartTerm::Group(myself.id)],
            exclude: Vec::new(),
        };
        assert_eq!(filter.select(&ctx), [false, false, false]);
    }
}
