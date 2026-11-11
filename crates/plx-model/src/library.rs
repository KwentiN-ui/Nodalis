//! Material library: materials sorted into categories, kept outside of any project so that
//! they can be copied into every model.

use serde::{Deserialize, Serialize};

use crate::{Elastic, Expansion, Material, Plastic, PlasticPoint, UnitSystem};

/// Version of the library file format written by this build.
pub const LIBRARY_FORMAT: u32 = 1;

/// Name of the root category.
pub const ROOT_NAME: &str = "Materials";

/// Default name of a new category.
pub const NEW_CATEGORY: &str = "NewCategory";

/// Place of a node in the library: the index of each node on the way down from the root's
/// children. The empty path is the root category.
pub type LibraryPath = Vec<usize>;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MaterialLibrary {
    pub format: u32,
    /// The units the library's values are in; materials are converted when they are copied
    /// between the library and a model. Libraries saved before this was stored are in
    /// the default "mm, ton, s, °C".
    #[serde(default)]
    pub units: UnitSystem,
    /// Version of the built-in materials this library has been updated to; 0 for libraries
    /// saved before the field existed.
    #[serde(default)]
    pub defaults_version: u32,
    pub root: Category,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Category {
    pub name: String,
    /// Whether the category is shown expanded; remembered.
    #[serde(default)]
    pub expanded: bool,
    pub items: Vec<LibraryNode>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum LibraryNode {
    Category(Category),
    Material(Material),
}

impl LibraryNode {
    pub fn name(&self) -> &str {
        match self {
            LibraryNode::Category(c) => &c.name,
            LibraryNode::Material(m) => &m.name,
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            LibraryNode::Category(c) => c.name = name,
            LibraryNode::Material(m) => m.name = name,
        }
    }
}

impl Category {
    fn new(name: &str, items: Vec<LibraryNode>) -> Self {
        Self {
            name: name.into(),
            expanded: true,
            items,
        }
    }

    fn contains(&self, name: &str) -> bool {
        self.items
            .iter()
            .any(|n| n.name().eq_ignore_ascii_case(name))
    }
}

/// Version of the built-in materials. Raise it when `MaterialLibrary::default` gains or changes
/// materials; libraries saved with an older version are updated by
/// [`MaterialLibrary::update_defaults`].
pub const DEFAULTS_VERSION: u32 = 3;

/// Generic material in the library's unit system (mm, t, s, degC) from datasheet values.
struct Generic {
    name: &'static str,
    /// Density in kg/m^3.
    density: f64,
    /// Young's modulus in MPa.
    young: f64,
    poisson: f64,
    /// Coefficient of thermal expansion in 1/K.
    expansion: f64,
    /// Thermal conductivity in W/(m K).
    conductivity: f64,
    /// Specific heat in J/(kg K).
    specific_heat: f64,
}

impl Generic {
    fn material(&self) -> Material {
        Material {
            name: self.name.into(),
            // kg/m^3 -> t/mm^3
            density: Some(self.density / 1e12),
            elastic: Some(Elastic {
                young: self.young,
                poisson: self.poisson,
            }),
            // W/(m K) = t mm/(s^3 K): the same number
            conductivity: Some(self.conductivity),
            // J/(kg K) = m^2/(s^2 K) -> mm^2/(s^2 K)
            specific_heat: Some(self.specific_heat * 1e6),
            expansion: Some(Expansion {
                coefficient: self.expansion,
                ..Expansion::default()
            }),
            ..Material::default()
        }
    }
}

/// A generic rubber of a Shore A hardness: Neo-Hooke with Young's modulus after Gent and
/// Poisson's ratio 0.49, in the library's unit system.
fn rubber(hardness: f64) -> Material {
    let young = crate::Hyperelastic::shore_a_young_mpa(hardness);
    // Two significant digits: datasheet precision.
    let digits = 1 - young.log10().floor() as i32;
    let scale = 10f64.powi(digits);
    let young = (young * scale).round() / scale;
    let poisson = 0.49;
    Material {
        name: format!("Generic Rubber {hardness} Shore A"),
        density: Some(1100.0 / 1e12),
        hyperelastic: Some(crate::Hyperelastic::neo_hooke(
            young / (2.0 * (1.0 + poisson)),
            poisson,
        )),
        conductivity: Some(0.15),
        specific_heat: Some(1900.0 * 1e6),
        expansion: Some(Expansion {
            coefficient: 200e-6,
            ..Expansion::default()
        }),
        ..Material::default()
    }
}

/// Name of the steel the library started with; it is renamed to [`GENERIC_STEEL`].
const OLD_STEEL: &str = "S235";
const GENERIC_STEEL: &str = "Generic Steel";

/// Typical room-temperature mean values from manufacturer datasheets and handbooks. Plastics
/// vary a lot with grade, moisture, temperature and (for 3D printing) print orientation and
/// infill, so these are starting points to be replaced with the values of the actual material.
const STEEL: Generic = Generic {
    name: GENERIC_STEEL,
    density: 7850.0,
    young: 210_000.0,
    poisson: 0.3,
    expansion: 12e-6,
    conductivity: 50.0,
    specific_heat: 470.0,
};

const PLASTICS: [Generic; 10] = [
    Generic {
        name: "Generic PLA",
        density: 1240.0,
        young: 3500.0,
        poisson: 0.36,
        expansion: 68e-6,
        conductivity: 0.13,
        specific_heat: 1800.0,
    },
    Generic {
        name: "Generic PETG",
        density: 1270.0,
        young: 2100.0,
        poisson: 0.38,
        expansion: 60e-6,
        conductivity: 0.20,
        specific_heat: 1200.0,
    },
    Generic {
        name: "Generic ABS",
        density: 1050.0,
        young: 2200.0,
        poisson: 0.35,
        expansion: 90e-6,
        conductivity: 0.17,
        specific_heat: 1400.0,
    },
    Generic {
        name: "Generic PA6",
        density: 1140.0,
        young: 3000.0,
        poisson: 0.39,
        expansion: 80e-6,
        conductivity: 0.25,
        specific_heat: 1700.0,
    },
    Generic {
        name: "Generic PA12",
        density: 1020.0,
        young: 1700.0,
        poisson: 0.40,
        expansion: 100e-6,
        conductivity: 0.23,
        specific_heat: 1700.0,
    },
    Generic {
        name: "Generic PET",
        density: 1380.0,
        young: 2800.0,
        poisson: 0.37,
        expansion: 70e-6,
        conductivity: 0.24,
        specific_heat: 1100.0,
    },
    Generic {
        name: "Generic PC",
        density: 1200.0,
        young: 2300.0,
        poisson: 0.37,
        expansion: 65e-6,
        conductivity: 0.21,
        specific_heat: 1200.0,
    },
    Generic {
        name: "Generic PP",
        density: 905.0,
        young: 1400.0,
        poisson: 0.42,
        expansion: 100e-6,
        conductivity: 0.22,
        specific_heat: 1900.0,
    },
    Generic {
        name: "Generic POM",
        density: 1410.0,
        young: 2900.0,
        poisson: 0.35,
        expansion: 110e-6,
        conductivity: 0.31,
        specific_heat: 1500.0,
    },
    Generic {
        name: "Generic PMMA",
        density: 1180.0,
        young: 3200.0,
        poisson: 0.37,
        expansion: 70e-6,
        conductivity: 0.19,
        specific_heat: 1470.0,
    },
];

/// Wrought aluminium alloys (EN AW), typical values of the tempers in use (6061-T6, 6082-T6,
/// 5083-H111, 7075-T6, 2024-T3) from ASM and manufacturer datasheets. The elastic constants
/// do not depend much on the temper.
const ALUMINIUM_WROUGHT: [Generic; 8] = [
    Generic {
        name: "EN AW-1050A",
        density: 2710.0,
        young: 69_000.0,
        poisson: 0.33,
        expansion: 23.6e-6,
        conductivity: 229.0,
        specific_heat: 900.0,
    },
    Generic {
        name: "EN AW-3003",
        density: 2730.0,
        young: 69_000.0,
        poisson: 0.33,
        expansion: 23.2e-6,
        conductivity: 193.0,
        specific_heat: 893.0,
    },
    Generic {
        name: "EN AW-5083",
        density: 2660.0,
        young: 71_000.0,
        poisson: 0.33,
        expansion: 23.8e-6,
        conductivity: 117.0,
        specific_heat: 900.0,
    },
    Generic {
        name: "EN AW-5754",
        density: 2670.0,
        young: 70_000.0,
        poisson: 0.33,
        expansion: 23.8e-6,
        conductivity: 147.0,
        specific_heat: 900.0,
    },
    Generic {
        name: "EN AW-6061",
        density: 2700.0,
        young: 68_900.0,
        poisson: 0.33,
        expansion: 23.6e-6,
        conductivity: 167.0,
        specific_heat: 896.0,
    },
    Generic {
        name: "EN AW-6082",
        density: 2700.0,
        young: 70_000.0,
        poisson: 0.33,
        expansion: 23.4e-6,
        conductivity: 170.0,
        specific_heat: 900.0,
    },
    Generic {
        name: "EN AW-2024",
        density: 2780.0,
        young: 73_100.0,
        poisson: 0.33,
        expansion: 23.2e-6,
        conductivity: 121.0,
        specific_heat: 875.0,
    },
    Generic {
        name: "EN AW-7075",
        density: 2810.0,
        young: 71_700.0,
        poisson: 0.33,
        expansion: 23.4e-6,
        conductivity: 130.0,
        specific_heat: 960.0,
    },
];

/// Cast aluminium alloys (EN AC) as used in sand, die and permanent mould casting. Conductivity
/// varies with the heat treatment and the casting process, so the mean of the datasheet range
/// is used.
const ALUMINIUM_CAST: [Generic; 4] = [
    Generic {
        name: "EN AC-AlSi7Mg0.3 (42100)",
        density: 2680.0,
        young: 72_500.0,
        poisson: 0.33,
        expansion: 21.5e-6,
        conductivity: 150.0,
        specific_heat: 880.0,
    },
    Generic {
        name: "EN AC-AlSi10Mg (43000)",
        density: 2670.0,
        young: 75_000.0,
        poisson: 0.33,
        expansion: 21.0e-6,
        conductivity: 150.0,
        specific_heat: 880.0,
    },
    Generic {
        name: "EN AC-AlSi12 (44200)",
        density: 2650.0,
        young: 75_000.0,
        poisson: 0.33,
        expansion: 20.0e-6,
        conductivity: 140.0,
        specific_heat: 880.0,
    },
    Generic {
        name: "EN AC-AlSi9Cu3(Fe) (46000)",
        density: 2760.0,
        young: 74_000.0,
        poisson: 0.33,
        expansion: 21.0e-6,
        conductivity: 110.0,
        specific_heat: 880.0,
    },
];

/// A steel with a plastic hardening curve: `curve` holds (true stress in MPa, plastic strain)
/// from the yield point on, in the order the solver wants them.
struct PlasticSteel {
    base: Generic,
    curve: &'static [(f64, f64)],
}

impl PlasticSteel {
    fn material(&self) -> Material {
        Material {
            plastic: Some(Plastic {
                hardening: crate::Hardening::Isotropic,
                points: (self.curve.iter())
                    .map(|&(stress, plastic_strain)| PlasticPoint {
                        stress,
                        plastic_strain,
                        temperature: 0.0,
                    })
                    .collect(),
            }),
            ..self.base.material()
        }
    }
}

/// Hot-rolled structural steels (EN 10025-2) with the nominal yield and tensile strength of the
/// standard (S235: 235 / 360 MPa, S355: 355 / 470 MPa). The curve follows the strain hardening
/// model of Yun and Gardner (J. Constr. Steel Res. 133, 2017): a yield plateau up to
/// `esh = 0.1 fy/fu - 0.055` (limited to 0.015 .. 0.03), then hardening to the tensile
/// strength at the uniform strain `eu = 0.6 (1 - fy/fu)`, a cubic that ends with zero slope
/// (maximum load). Engineering values are converted to true stress and plastic strain.
const PLASTIC_STEELS: [PlasticSteel; 3] = [
    PlasticSteel {
        base: Generic {
            name: "S235",
            ..STEEL
        },
        curve: &[
            (235.0, 0.0000),
            (239.0, 0.0138),
            (267.0, 0.0271),
            (293.0, 0.0403),
            (316.0, 0.0534),
            (337.0, 0.0663),
            (354.0, 0.0790),
            (370.0, 0.0916),
            (383.0, 0.1040),
            (394.0, 0.1163),
            (404.0, 0.1285),
            (412.0, 0.1405),
            (419.0, 0.1524),
            (425.0, 0.1641),
            (430.0, 0.1757),
            (435.0, 0.1872),
        ],
    },
    PlasticSteel {
        base: Generic {
            name: "S355",
            ..STEEL
        },
        curve: &[
            (356.0, 0.0000),
            (362.0, 0.0186),
            (389.0, 0.0273),
            (413.0, 0.0359),
            (434.0, 0.0444),
            (452.0, 0.0529),
            (468.0, 0.0613),
            (482.0, 0.0697),
            (494.0, 0.0780),
            (504.0, 0.0862),
            (512.0, 0.0944),
            (519.0, 0.1025),
            (525.0, 0.1106),
            (530.0, 0.1186),
            (535.0, 0.1265),
            (539.0, 0.1344),
        ],
    },
    // Quenched (830 degC, oil) and tempered (630 degC) 42CrMo4: yield 850 MPa, tensile
    // strength 900 MPa. Beyond the yield the true stress follows the power law
    // 1277.3 MPa * ep^0.0979 that Basan, Kunc and Franulovic fitted (Engineering Review 28,
    // 2008, Table 2); it holds up to the uniform strain of about 0.1.
    PlasticSteel {
        base: Generic {
            name: "42CrMo4",
            conductivity: 42.0,
            specific_heat: 460.0,
            expansion: 12.3e-6,
            ..STEEL
        },
        curve: &[
            (850.0, 0.0),
            (871.0, 0.02),
            (906.0, 0.03),
            (932.0, 0.04),
            (953.0, 0.05),
            (970.0, 0.06),
            (985.0, 0.07),
            (997.0, 0.08),
            (1009.0, 0.09),
            (1020.0, 0.10),
        ],
    },
];

/// The S235 of libraries created before the generic materials; renamed only while unchanged.
fn old_steel() -> Material {
    Material {
        name: OLD_STEEL.into(),
        density: Some(7.85e-9),
        elastic: Some(Elastic {
            young: 210_000.0,
            poisson: 0.3,
        }),
        ..Material::default()
    }
}

fn category_node(name: &str, items: Vec<LibraryNode>) -> LibraryNode {
    LibraryNode::Category(Category::new(name, items))
}

impl Default for MaterialLibrary {
    /// The library a new installation starts with. Values in the unit system mm, t, s, as
    /// the default "mm, ton, s, °C".
    fn default() -> Self {
        let materials = |list: &[Generic]| {
            (list.iter())
                .map(|g| LibraryNode::Material(g.material()))
                .collect()
        };
        Self {
            format: LIBRARY_FORMAT,
            units: UnitSystem::MmTonSC,
            defaults_version: DEFAULTS_VERSION,
            root: Category::new(
                ROOT_NAME,
                vec![
                    category_node(
                        "Elastic_Models",
                        vec![
                            category_node("Steel", materials(&[STEEL])),
                            category_node(
                                "Aluminium",
                                vec![
                                    category_node("Wrought", materials(&ALUMINIUM_WROUGHT)),
                                    category_node("Cast", materials(&ALUMINIUM_CAST)),
                                ],
                            ),
                            category_node("Plastics", materials(&PLASTICS)),
                            category_node(
                                "Elastomers",
                                [40.0, 60.0, 70.0]
                                    .map(|h| LibraryNode::Material(rubber(h)))
                                    .to_vec(),
                            ),
                        ],
                    ),
                    category_node(
                        "Plastic_Models",
                        vec![category_node(
                            "Steel",
                            (PLASTIC_STEELS.iter())
                                .map(|s| LibraryNode::Material(s.material()))
                                .collect(),
                        )],
                    ),
                ],
            ),
        }
    }
}

impl MaterialLibrary {
    /// The node at a path; `None` for the root and for paths that lead nowhere.
    pub fn node(&self, path: &[usize]) -> Option<&LibraryNode> {
        let (&last, parent) = path.split_last()?;
        self.category(parent)?.items.get(last)
    }

    fn node_mut(&mut self, path: &[usize]) -> Option<&mut LibraryNode> {
        let (&last, parent) = path.split_last()?;
        self.category_mut(parent)?.items.get_mut(last)
    }

    /// The category at a path, the root for the empty path.
    pub fn category(&self, path: &[usize]) -> Option<&Category> {
        let mut category = &self.root;
        for &index in path {
            match category.items.get(index)? {
                LibraryNode::Category(c) => category = c,
                LibraryNode::Material(_) => return None,
            }
        }
        Some(category)
    }

    pub fn category_mut(&mut self, path: &[usize]) -> Option<&mut Category> {
        let mut category = &mut self.root;
        for &index in path {
            match category.items.get_mut(index)? {
                LibraryNode::Category(c) => category = c,
                LibraryNode::Material(_) => return None,
            }
        }
        Some(category)
    }

    pub fn material(&self, path: &[usize]) -> Option<&Material> {
        match self.node(path)? {
            LibraryNode::Material(m) => Some(m),
            LibraryNode::Category(_) => None,
        }
    }

    /// The category a new node goes into when `path` is selected: the selected category, or
    /// the category of the selected material.
    fn target_category(&self, path: &[usize]) -> Option<LibraryPath> {
        if self.category(path).is_some() {
            Some(path.to_vec())
        } else {
            self.material(path)?;
            Some(path[..path.len() - 1].to_vec())
        }
    }

    /// Adds an empty category to the selected category (none below a
    /// material) and returns its path.
    pub fn add_category(&mut self, selected: &[usize]) -> Option<LibraryPath> {
        let category = self.category_mut(selected)?;
        let name = free_name(category, NEW_CATEGORY, "-");
        category.expanded = true;
        category
            .items
            .push(LibraryNode::Category(Category::new(&name, Vec::new())));
        let mut path = selected.to_vec();
        path.push(category.items.len() - 1);
        Some(path)
    }

    /// Copies a material of the model into the selected category, or next to the selected
    /// material. A taken name gets the suffix `_Model-n`. Returns the new path.
    pub fn add_material(&mut self, selected: &[usize], material: &Material) -> Option<LibraryPath> {
        let target = self.target_category(selected)?;
        let category = self.category_mut(&target)?;
        let mut material = material.clone();
        material.name = free_name(category, &material.name, "_Model-");
        category.expanded = true;
        category.items.push(LibraryNode::Material(material));
        let mut path = target;
        path.push(category.items.len() - 1);
        Some(path)
    }

    /// Deletes a node with everything in it. Returns the node to select afterwards: the one
    /// that took its place, the one before it, or the parent.
    pub fn delete(&mut self, path: &[usize]) -> Option<LibraryPath> {
        let (&last, parent) = path.split_last()?;
        let category = self.category_mut(parent)?;
        if last >= category.items.len() {
            return None;
        }
        category.items.remove(last);
        let mut next = parent.to_vec();
        if last < category.items.len() {
            next.push(last);
        } else if last > 0 {
            next.push(last - 1);
        }
        Some(next)
    }

    /// Renames a category or material; the name must be free among its siblings.
    pub fn rename(&mut self, path: &[usize], name: &str) -> Result<(), String> {
        let name = name.trim();
        let (&last, parent) = path
            .split_last()
            .ok_or("The root of the library cannot be renamed.")?;
        if name.is_empty() {
            return Err("Please enter a name.".into());
        }
        let category = self.category(parent).ok_or("Entry not found.")?;
        let taken = (category.items.iter().enumerate())
            .any(|(i, n)| i != last && n.name().eq_ignore_ascii_case(name));
        if taken {
            return Err(format!(
                "{} already contains an entry named {name}.",
                category.name
            ));
        }
        self.node_mut(path)
            .ok_or("Entry not found.")?
            .set_name(name.into());
        Ok(())
    }

    /// Index a material would move to, one place up or down among its siblings. Categories
    /// keep their place.
    pub fn move_target(&self, path: &[usize], up: bool) -> Option<usize> {
        self.material(path)?;
        let (&last, parent) = path.split_last()?;
        let count = self.category(parent)?.items.len();
        if up {
            last.checked_sub(1)
        } else {
            Some(last + 1).filter(|&t| t < count)
        }
    }

    /// Moves a material one place up or down and returns its new path.
    pub fn move_material(&mut self, path: &[usize], up: bool) -> Option<LibraryPath> {
        let target = self.move_target(path, up)?;
        let (&last, parent) = path.split_last()?;
        self.category_mut(parent)?.items.swap(last, target);
        let mut moved = parent.to_vec();
        moved.push(target);
        Some(moved)
    }

    /// Brings a library saved by an older version up to the current built-in materials: the
    /// old "S235" becomes "Generic Steel" if the user has not changed it, and built-in
    /// materials that are missing are added to their category. Materials the user has
    /// deleted come back only once, when the version rises.
    pub fn update_defaults(&mut self) {
        if self.defaults_version >= DEFAULTS_VERSION {
            return;
        }
        self.defaults_version = DEFAULTS_VERSION;
        if self.units != UnitSystem::MmTonSC {
            return;
        }
        // The category "Structural" below "Steel" is gone: its materials move up.
        let steel = ensure_category(&mut self.root, &["Elastic_Models", "Steel"]);
        let structural = steel.items.iter().position(
            |n| matches!(n, LibraryNode::Category(c) if c.name.eq_ignore_ascii_case("Structural")),
        );
        if let Some(index) = structural {
            let LibraryNode::Category(structural) = steel.items.remove(index) else {
                unreachable!("index points to a category")
            };
            for node in structural.items {
                if !steel.contains(node.name()) {
                    steel.items.insert(index.min(steel.items.len()), node);
                }
            }
        }
        let old = old_steel();
        let reference = MaterialLibrary::default();
        merge_defaults(&mut self.root, &reference.root, &old);
    }

    /// Path of the first material in tree order, selected on opening.
    pub fn first_material(&self) -> Option<LibraryPath> {
        fn find(category: &Category, path: &mut LibraryPath) -> bool {
            for (i, node) in category.items.iter().enumerate() {
                path.push(i);
                match node {
                    LibraryNode::Material(_) => return true,
                    LibraryNode::Category(c) if find(c, path) => return true,
                    LibraryNode::Category(_) => {}
                }
                path.pop();
            }
            false
        }
        let mut path = Vec::new();
        find(&self.root, &mut path).then_some(path)
    }
}

/// Adds the built-in categories and materials of `reference` that `category` lacks, recursively.
fn merge_defaults(category: &mut Category, reference: &Category, old: &Material) {
    for node in &reference.items {
        match node {
            LibraryNode::Material(m) => add_default(category, m, old),
            LibraryNode::Category(sub) => {
                let target = ensure_category(category, &[&sub.name]);
                merge_defaults(target, sub, old);
            }
        }
    }
}

/// The category reached by names from `category`, created where missing.
fn ensure_category<'a>(category: &'a mut Category, names: &[&str]) -> &'a mut Category {
    let Some((first, rest)) = names.split_first() else {
        return category;
    };
    let index = category
        .items
        .iter()
        .position(|n| matches!(n, LibraryNode::Category(c) if c.name.eq_ignore_ascii_case(first)));
    let index = index.unwrap_or_else(|| {
        category.items.push(category_node(first, Vec::new()));
        category.items.len() - 1
    });
    match &mut category.items[index] {
        LibraryNode::Category(c) => ensure_category(c, rest),
        LibraryNode::Material(_) => unreachable!("index points to a category"),
    }
}

/// Adds a built-in material unless the category has one of that name; an unchanged old
/// S235 is replaced by Generic Steel instead.
fn add_default(category: &mut Category, material: &Material, old: &Material) {
    if material.name == GENERIC_STEEL {
        let unchanged = category
            .items
            .iter_mut()
            // The same material by its values: ids are made anew with each library.
            .find(|n| matches!(n, LibraryNode::Material(m) if Material { id: old.id, ..m.clone() } == *old));
        if let Some(node) = unchanged {
            *node = LibraryNode::Material(material.clone());
            return;
        }
    }
    if !category.contains(&material.name) {
        category.items.push(LibraryNode::Material(material.clone()));
    }
}

/// `name` if no item of the category has it, else `name` with `separator` and the first
/// free number.
fn free_name(category: &Category, name: &str, separator: &str) -> String {
    if !category.contains(name) {
        return name.into();
    }
    (1..)
        .map(|n| format!("{name}{separator}{n}"))
        .find(|candidate| !category.contains(candidate))
        .expect("unbounded range")
}

/// Name for a material copied into the model: `_Library-n` is appended when the name
/// is taken.
pub fn name_for_model<'a>(name: &str, existing: impl IntoIterator<Item = &'a str>) -> String {
    let existing: Vec<&str> = existing.into_iter().collect();
    let taken = |candidate: &str| existing.iter().any(|e| e.eq_ignore_ascii_case(candidate));
    if !taken(name) {
        return name.into();
    }
    (1..)
        .map(|n| format!("{name}_Library-{n}"))
        .find(|candidate| !taken(candidate))
        .expect("unbounded range")
}

#[cfg(test)]
mod tests {
    use super::*;

    const S235: [usize; 3] = [0, 0, 0];

    #[test]
    fn default_library_holds_s235_in_default_categories() {
        let library = MaterialLibrary::default();
        assert_eq!(library.first_material(), Some(S235.to_vec()));
        let s235 = library.material(&S235).unwrap();
        assert_eq!(s235.name, "Generic Steel");
        assert_eq!(s235.density, Some(7.85e-9));
        let names: Vec<&str> = (1..3)
            .map(|n| library.node(&S235[..n]).unwrap().name())
            .collect();
        assert_eq!(names, ["Elastic_Models", "Steel"]);
    }

    #[test]
    fn categories_are_added_with_free_names() {
        let mut library = MaterialLibrary::default();
        assert_eq!(library.add_category(&[]), Some(vec![2]));
        assert_eq!(library.add_category(&[]), Some(vec![3]));
        assert_eq!(library.node(&[3]).unwrap().name(), "NewCategory-1");
        // Not below a material.
        assert_eq!(library.add_category(&S235), None);
    }

    #[test]
    fn model_materials_go_next_to_the_selected_material() {
        let mut library = MaterialLibrary::default();
        let s235 = library.material(&S235).unwrap().clone();
        let path = library.add_material(&S235, &s235).unwrap();
        assert_eq!(path, [0, 0, 1]);
        assert_eq!(
            library.material(&path).unwrap().name,
            "Generic Steel_Model-1"
        );
        let path = library.add_material(&[], &s235).unwrap();
        assert_eq!(path, [2]);
        assert_eq!(library.material(&path).unwrap().name, "Generic Steel");
    }

    #[test]
    fn rename_rejects_names_of_siblings() {
        let mut library = MaterialLibrary::default();
        let new = library.add_category(&[0, 0]).unwrap();
        assert!(library.rename(&new, "generic steel").is_err());
        assert!(library.rename(&new, " ").is_err());
        assert!(library.rename(&[], "Root").is_err());
        library.rename(&new, "Custom").unwrap();
        assert_eq!(library.node(&new).unwrap().name(), "Custom");
    }

    #[test]
    fn delete_selects_the_neighbour_or_the_parent() {
        let mut library = MaterialLibrary::default();
        let s235 = library.material(&S235).unwrap().clone();
        library.add_material(&S235, &s235);
        assert_eq!(library.delete(&[0, 0, 1]), Some(vec![0, 0, 0]));
        assert_eq!(library.delete(&S235), Some(vec![0, 0]));
        // The aluminium alloys come first now.
        assert_eq!(library.first_material(), Some(vec![0, 1, 0, 0]));
        assert_eq!(library.delete(&[]), None);
    }

    #[test]
    fn only_materials_move() {
        let mut library = MaterialLibrary::default();
        let s235 = library.material(&S235).unwrap().clone();
        library.add_material(&S235, &s235);
        assert_eq!(library.move_material(&S235, true), None);
        assert_eq!(library.move_material(&S235, false), Some(vec![0, 0, 1]));
        assert_eq!(
            library.material(&S235).unwrap().name,
            "Generic Steel_Model-1"
        );
        assert_eq!(library.move_material(&[0, 0, 1], false), None);
        assert_eq!(library.move_material(&[0], false), None);
    }

    #[test]
    fn copies_into_the_model_get_free_names() {
        assert_eq!(name_for_model("S235", ["Steel"]), "S235");
        assert_eq!(
            name_for_model("S235", ["s235", "S235_Library-1"]),
            "S235_Library-2"
        );
    }

    #[test]
    fn default_library_has_generic_plastics() {
        let library = MaterialLibrary::default();
        let plastics = library.category(&[0, 2]).unwrap();
        assert_eq!(plastics.name, "Plastics");
        let pla = (plastics.items.iter())
            .find_map(|n| match n {
                LibraryNode::Material(m) if m.name == "Generic PLA" => Some(m),
                _ => None,
            })
            .unwrap();
        assert_eq!(pla.density, Some(1.24e-9));
        assert_eq!(pla.elastic.unwrap().young, 3500.0);
        assert_eq!(pla.specific_heat, Some(1.8e9));
    }

    #[test]
    fn old_libraries_get_the_generic_materials() {
        // A library as saved before the generic materials: S235 only, no version.
        let mut old = MaterialLibrary {
            defaults_version: 0,
            ..MaterialLibrary::default()
        };
        old.root.items.clear();
        let steel = ensure_category(&mut old.root, &["Elastic_Models", "Steel", "Structural"]);
        steel.items.push(LibraryNode::Material(old_steel()));
        let mut mine = old.clone();
        mine.update_defaults();
        assert_eq!(mine.defaults_version, DEFAULTS_VERSION);
        assert_eq!(mine.material(&[0, 0, 0]).unwrap().name, "Generic Steel");
        assert!(mine.category(&[0, 2]).unwrap().items.len() >= 10);
        assert!(mine.category(&[1, 0]).is_some(), "plastic steels added");
        let again = mine.clone();
        mine.update_defaults();
        assert_eq!(mine, again);

        // A changed S235 stays; Generic Steel is added next to it.
        let mut changed = old;
        let LibraryNode::Material(m) = &mut ensure_category(
            &mut changed.root,
            &["Elastic_Models", "Steel", "Structural"],
        )
        .items[0] else {
            unreachable!()
        };
        m.elastic.as_mut().unwrap().young = 200_000.0;
        changed.update_defaults();
        let names: Vec<&str> = (changed.category(&[0, 0]).unwrap().items.iter())
            .map(LibraryNode::name)
            .collect();
        assert_eq!(names, ["S235", "Generic Steel"]);
    }

    #[test]
    fn default_library_has_aluminium_and_plastic_steels() {
        let library = MaterialLibrary::default();
        let aluminium = library.category(&[0, 1]).unwrap();
        assert_eq!(aluminium.name, "Aluminium");
        let names: Vec<&str> = aluminium.items.iter().map(LibraryNode::name).collect();
        assert_eq!(names, ["Wrought", "Cast"]);
        let plastic = library.category(&[1, 0]).unwrap();
        assert_eq!(plastic.name, "Steel");
        assert_eq!(plastic.items.len(), 3);
        for node in &plastic.items {
            let LibraryNode::Material(m) = node else {
                panic!("category in the plastic steels")
            };
            let curve = m.plastic.as_ref().unwrap();
            assert_eq!(curve.invalid_row(), None, "{}", m.name);
            assert!(m.elastic.is_some() && m.density.is_some());
        }
        let LibraryNode::Material(s355) = &plastic.items[1] else {
            unreachable!()
        };
        assert_eq!(s355.name, "S355");
        assert_eq!(s355.plastic.as_ref().unwrap().points[0].stress, 356.0);
    }
}
