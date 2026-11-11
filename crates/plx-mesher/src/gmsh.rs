//! Safe access to the Gmsh library: finding and loading it, and one wrapper per C function.
//!
//! Gmsh keeps one global model per process, so the library is loaded once and every use goes
//! through [`with_gmsh`], which serialises them.

// Calling into a C library needs `unsafe`; it is confined to this module and `ffi`.
#![allow(unsafe_code)]

use std::ffi::{CStr, CString, c_char, c_int};
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use crate::ffi::Api;

/// The Gmsh version nodalis is built and tested against; `scripts/fetch_gmsh.py` fetches it.
pub const TESTED_VERSION: (u32, u32) = (4, 15);

/// Environment variable naming the Gmsh library, used when no path is configured. Tests and
/// the CI use it.
pub const LIBRARY_ENV: &str = "NODALIS_GMSH";

#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum GmshError {
    #[error(
        "Gmsh library not found (searched: {}). Set the path under Settings > Gmsh.",
        .0.join(", ")
    )]
    NotFound(Vec<String>),
    #[error("{path}: cannot be loaded: {message}")]
    Load { path: PathBuf, message: String },
    #[error("{path}: not a matching Gmsh library, missing {}", .missing.join(", "))]
    MissingFunctions {
        path: PathBuf,
        missing: Vec<&'static str>,
    },
    #[error("Gmsh {version} is not supported, Gmsh 4 is required")]
    UnsupportedVersion { version: String },
    #[error("Gmsh: {0}")]
    Call(String),
    #[error("{0}")]
    Other(String),
}

/// What the settings show about the loaded library.
#[derive(Clone, Debug, PartialEq)]
pub struct LibraryInfo {
    pub path: PathBuf,
    pub version: String,
    /// The version differs from [`TESTED_VERSION`]; it may work, but is not tested.
    pub untested: bool,
}

/// A loaded and initialised Gmsh.
pub struct Gmsh {
    api: Api,
    info: LibraryInfo,
}

struct State {
    /// Library from the settings; `None` searches the default places.
    configured: Option<PathBuf>,
    loaded: Option<Gmsh>,
}

static STATE: Mutex<State> = Mutex::new(State {
    configured: None,
    loaded: None,
});

/// A boolean operation of OpenCASCADE.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Boolean {
    Intersect,
    Fuse,
    Cut,
}

/// What a boolean operation made, and for each input what it became.
pub type Fragments = (Vec<(i32, i32)>, Vec<Vec<(i32, i32)>>);

/// The C function of a boolean operation of OpenCASCADE.
type BooleanFn = unsafe extern "C" fn(
    *const c_int,
    usize,
    *const c_int,
    usize,
    *mut *mut c_int,
    *mut usize,
    *mut *mut *mut c_int,
    *mut *mut usize,
    *mut usize,
    c_int,
    c_int,
    c_int,
    *mut c_int,
);

impl Boolean {
    fn function(self, api: &crate::ffi::Api) -> BooleanFn {
        match self {
            Self::Intersect => api.occ_intersect,
            Self::Fuse => api.occ_fuse,
            Self::Cut => api.occ_cut,
        }
    }
}

/// Sets where the library is loaded from. Gmsh cannot be unloaded, so a change takes effect
/// only if the library has not been loaded yet; returns whether that is the case.
pub fn set_library_path(path: Option<PathBuf>) -> bool {
    let mut state = STATE.lock().unwrap_or_else(|e| e.into_inner());
    state.configured = path;
    state.loaded.is_none()
}

/// The loaded library, if any use loaded it already.
pub fn loaded_library() -> Option<LibraryInfo> {
    let state = STATE.lock().unwrap_or_else(|e| e.into_inner());
    state.loaded.as_ref().map(|g| g.info.clone())
}

/// The library another process should load to mesh like this one: the loaded one, else the
/// configured one; `None` searches the default places there too.
pub fn library_path() -> Option<PathBuf> {
    let state = STATE.lock().unwrap_or_else(|e| e.into_inner());
    match &state.loaded {
        Some(gmsh) => Some(gmsh.info.path.clone()),
        None => state.configured.clone(),
    }
}

/// Runs `f` with Gmsh, loading the library on first use. Calls from several threads wait for
/// each other, because Gmsh has one global model.
pub fn with_gmsh<T>(f: impl FnOnce(&Gmsh) -> Result<T, GmshError>) -> Result<T, GmshError> {
    let mut state = STATE.lock().unwrap_or_else(|e| e.into_inner());
    if state.loaded.is_none() {
        let gmsh = Gmsh::load(state.configured.as_deref())?;
        state.loaded = Some(gmsh);
    }
    let gmsh = state.loaded.as_ref().expect("loaded above");
    gmsh.clear()?;
    gmsh.start_log()?;
    let result = f(gmsh);
    // The model is not needed any more; clearing frees its memory right away.
    let _ = gmsh.clear();
    result
}

/// Places the library is looked for, in order: the configured path, [`LIBRARY_ENV`], the
/// program's folder (where releases ship it) with its `gmsh` and `../gmsh` subfolders, in
/// development builds `target/gmsh` of the workspace (where `scripts/fetch_gmsh.py` puts it),
/// then the system's search path.
pub fn candidates(configured: Option<&Path>) -> Vec<PathBuf> {
    if let Some(path) = configured {
        return vec![path.to_path_buf()];
    }
    if let Some(path) = std::env::var_os(LIBRARY_ENV).filter(|p| !p.is_empty()) {
        return vec![PathBuf::from(path)];
    }
    let mut found = Vec::new();
    if let Some(dir) = std::env::current_exe()
        .ok()
        .and_then(|e| Some(e.parent()?.to_path_buf()))
    {
        for dir in [dir.clone(), dir.join("gmsh"), dir.join("..").join("gmsh")] {
            found.extend(libraries_in(&dir));
        }
    }
    // Development builds and tests also find the library fetched into the workspace.
    if cfg!(debug_assertions) {
        found.extend(libraries_in(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/gmsh"),
        ));
    }
    // Bare names are resolved by the system's loader.
    let (major, minor) = TESTED_VERSION;
    found.extend(
        if cfg!(windows) {
            vec![format!("gmsh-{major}.{minor}.dll"), "gmsh.dll".into()]
        } else if cfg!(target_os = "macos") {
            vec!["libgmsh.dylib".into()]
        } else {
            vec![format!("libgmsh.so.{major}.{minor}"), "libgmsh.so".into()]
        }
        .into_iter()
        .map(PathBuf::from),
    );
    found
}

/// Gmsh libraries in a folder, newest version first.
fn libraries_in(dir: &Path) -> Vec<PathBuf> {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut found: Vec<PathBuf> = entries
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.is_file())
        .filter(|p| {
            let name = p
                .file_name()
                .map_or(String::new(), |n| n.to_string_lossy().into());
            if cfg!(windows) {
                name.starts_with("gmsh") && name.ends_with(".dll")
            } else if cfg!(target_os = "macos") {
                name.starts_with("libgmsh") && name.ends_with(".dylib")
            } else {
                name.starts_with("libgmsh.so")
            }
        })
        .collect();
    found.sort();
    found.reverse();
    found
}

/// "4.15.2" as (4, 15).
fn parse_version(version: &str) -> Option<(u32, u32)> {
    let mut parts = version.trim().split('.');
    Some((parts.next()?.parse().ok()?, parts.next()?.parse().ok()?))
}

impl Gmsh {
    fn load(configured: Option<&Path>) -> Result<Self, GmshError> {
        let candidates = candidates(configured);
        let mut first_error = None;
        for path in &candidates {
            let explicit = path.components().count() > 1;
            if explicit && !path.is_file() {
                continue;
            }
            match Self::load_from(path) {
                Ok(gmsh) => return Ok(gmsh),
                // A bare name the loader does not know is just not installed.
                Err(GmshError::Load { .. }) if !explicit => {}
                Err(error) => {
                    first_error.get_or_insert(error);
                }
            }
        }
        Err(first_error.unwrap_or_else(|| {
            GmshError::NotFound(candidates.iter().map(|p| p.display().to_string()).collect())
        }))
    }

    fn load_from(path: &Path) -> Result<Self, GmshError> {
        // SAFETY: loading runs the library's initialisers; Gmsh's only set up its globals.
        let library = unsafe { libloading::Library::new(path) }.map_err(|e| GmshError::Load {
            path: path.to_path_buf(),
            message: e.to_string(),
        })?;
        // SAFETY: a library exporting Gmsh's function names is Gmsh.
        let api = unsafe { Api::load(library) }.map_err(|missing| GmshError::MissingFunctions {
            path: path.to_path_buf(),
            missing,
        })?;
        let mut gmsh = Self {
            api,
            info: LibraryInfo {
                path: std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf()),
                version: String::new(),
                untested: false,
            },
        };
        gmsh.initialize()?;
        let version = gmsh.option_string("General.Version")?;
        match parse_version(&version) {
            Some((4, minor)) => gmsh.info.untested = (4, minor) != TESTED_VERSION,
            _ => return Err(GmshError::UnsupportedVersion { version }),
        }
        gmsh.info.version = version;
        // Messages go to the log, not to the terminal; STEP files are read in millimetres.
        gmsh.set_number("General.Terminal", 0.0)?;
        gmsh.set_string("Geometry.OCCTargetUnit", "MM")?;
        Ok(gmsh)
    }

    pub fn info(&self) -> &LibraryInfo {
        &self.info
    }

    /// Calls a C function with an error flag and turns a set flag into Gmsh's last error.
    fn call(&self, f: impl FnOnce(*mut c_int)) -> Result<(), GmshError> {
        let mut ierr: c_int = 0;
        f(&mut ierr);
        if ierr == 0 {
            return Ok(());
        }
        let mut message: *mut c_char = std::ptr::null_mut();
        let mut ignored: c_int = 0;
        // SAFETY: valid out pointers; the returned string is copied and freed.
        unsafe {
            (self.api.logger_get_last_error)(&mut message, &mut ignored);
        }
        let text = self.take_string(message);
        Err(GmshError::Call(if text.is_empty() {
            format!("Error {ierr}")
        } else {
            text
        }))
    }

    /// Copies a string returned by Gmsh and releases it.
    fn take_string(&self, pointer: *mut c_char) -> String {
        if pointer.is_null() {
            return String::new();
        }
        // SAFETY: Gmsh returns NUL-terminated strings allocated with its own allocator.
        unsafe {
            let text = CStr::from_ptr(pointer).to_string_lossy().into_owned();
            (self.api.free)(pointer.cast());
            text
        }
    }

    /// Copies an array returned by Gmsh and releases it.
    fn take_vec<T: Copy>(&self, pointer: *mut T, len: usize) -> Vec<T> {
        if pointer.is_null() {
            return Vec::new();
        }
        // SAFETY: Gmsh returns `len` initialised values allocated with its own allocator.
        unsafe {
            let values = std::slice::from_raw_parts(pointer, len).to_vec();
            (self.api.free)(pointer.cast());
            values
        }
    }

    fn initialize(&self) -> Result<(), GmshError> {
        // No command line, no configuration files of a Gmsh installation, no GUI.
        self.call(|e| unsafe { (self.api.initialize)(0, std::ptr::null_mut(), 0, 0, e) })
    }

    pub fn clear(&self) -> Result<(), GmshError> {
        self.call(|e| unsafe { (self.api.clear)(e) })?;
        let name = c_string("nodalis")?;
        self.call(|e| unsafe { (self.api.model_add)(name.as_ptr(), e) })
    }

    pub fn set_number(&self, name: &str, value: f64) -> Result<(), GmshError> {
        let name = c_string(name)?;
        self.call(|e| unsafe { (self.api.option_set_number)(name.as_ptr(), value, e) })
    }

    pub fn set_string(&self, name: &str, value: &str) -> Result<(), GmshError> {
        let (name, value) = (c_string(name)?, c_string(value)?);
        self.call(|e| unsafe {
            (self.api.option_set_string)(name.as_ptr(), value.as_ptr(), e);
        })
    }

    pub fn option_string(&self, name: &str) -> Result<String, GmshError> {
        let name = c_string(name)?;
        let mut value = std::ptr::null_mut();
        self.call(|e| unsafe { (self.api.option_get_string)(name.as_ptr(), &mut value, e) })?;
        Ok(self.take_string(value))
    }

    /// Writes the model in the format given by the extension, e.g. `.brep` or `.step`.
    pub fn write(&self, path: &Path) -> Result<(), GmshError> {
        let path = c_path(path)?;
        self.call(|e| unsafe { (self.api.write)(path.as_ptr(), e) })
    }

    /// Reads a STEP, IGES or BREP file into the OpenCASCADE kernel and synchronises the model.
    /// Shapes of every dimension are kept, so that free faces and edges, the shell and line
    /// parts, survive next to the solids; the returned (dimension, tag) pairs are the shapes
    /// of the file.
    pub fn import_shapes(&self, path: &Path) -> Result<Vec<(i32, i32)>, GmshError> {
        let file = c_path(path)?;
        let empty = c_string("")?;
        let (mut tags, mut len) = (std::ptr::null_mut(), 0);
        self.call(|e| unsafe {
            (self.api.occ_import_shapes)(file.as_ptr(), &mut tags, &mut len, 0, empty.as_ptr(), e);
        })?;
        let tags = self.take_vec(tags, len);
        self.synchronize()?;
        Ok(pairs(&tags))
    }

    /// Adds an axis-parallel box from a corner by its size; returns its tag.
    pub fn add_box(&self, origin: [f64; 3], size: [f64; 3]) -> Result<i32, GmshError> {
        let ([x, y, z], [dx, dy, dz]) = (origin, size);
        let mut tag = 0;
        self.call(|e| tag = unsafe { (self.api.occ_add_box)(x, y, z, dx, dy, dz, -1, e) })?;
        self.synchronize()?;
        Ok(tag)
    }

    /// Adds a flat disk of a radius about a centre, normal to `axis`; returns its tag.
    pub fn add_disk(
        &self,
        centre: [f64; 3],
        radius: f64,
        axis: [f64; 3],
    ) -> Result<i32, GmshError> {
        let [x, y, z] = centre;
        let mut tag = 0;
        self.call(|e| {
            tag = unsafe {
                (self.api.occ_add_disk)(
                    x,
                    y,
                    z,
                    radius,
                    radius,
                    -1,
                    axis.as_ptr(),
                    3,
                    std::ptr::null(),
                    0,
                    e,
                )
            };
        })?;
        self.synchronize()?;
        Ok(tag)
    }

    /// A rectangle in a plane z = const, with corners rounded by `radius` if it is positive.
    pub fn add_rectangle(
        &self,
        origin: [f64; 3],
        size: [f64; 2],
        radius: f64,
    ) -> Result<i32, GmshError> {
        let ([x, y, z], [dx, dy]) = (origin, size);
        let mut tag = 0;
        self.call(|e| {
            tag = unsafe { (self.api.occ_add_rectangle)(x, y, z, dx, dy, -1, radius, e) };
        })?;
        self.synchronize()?;
        Ok(tag)
    }

    /// Adds a point; returns its tag. Points are the ends of lines, see [`Self::add_line`].
    pub fn add_point(&self, [x, y, z]: [f64; 3]) -> Result<i32, GmshError> {
        let mut tag = 0;
        self.call(|e| tag = unsafe { (self.api.occ_add_point)(x, y, z, 0.0, -1, e) })?;
        self.synchronize()?;
        Ok(tag)
    }

    /// Adds a straight line between two points; returns its tag.
    pub fn add_line(&self, start: i32, end: i32) -> Result<i32, GmshError> {
        let mut tag = 0;
        self.call(|e| tag = unsafe { (self.api.occ_add_line)(start, end, -1, e) })?;
        self.synchronize()?;
        Ok(tag)
    }

    /// Removes entities given as (dimension, tag) with what bounds them and nothing else
    /// uses.
    pub fn remove(&self, entities: &[(i32, i32)]) -> Result<(), GmshError> {
        let flat: Vec<c_int> = entities.iter().flat_map(|&(d, t)| [d, t]).collect();
        self.call(|e| unsafe { (self.api.occ_remove)(flat.as_ptr(), flat.len(), 1, e) })?;
        self.synchronize()
    }

    /// Merges entities that lie on each other, e.g. the edge two faces share once one of them
    /// was made anew, so that they are joined again.
    pub fn remove_all_duplicates(&self) -> Result<(), GmshError> {
        self.call(|e| unsafe { (self.api.occ_remove_all_duplicates)(e) })?;
        self.synchronize()
    }

    /// Removes entities given as (dimension, tag) and nothing else, so that the faces of a
    /// removed solid stay.
    pub fn remove_only(&self, entities: &[(i32, i32)]) -> Result<(), GmshError> {
        let flat: Vec<c_int> = entities.iter().flat_map(|&(d, t)| [d, t]).collect();
        self.call(|e| unsafe { (self.api.occ_remove)(flat.as_ptr(), flat.len(), 0, e) })?;
        self.synchronize()
    }

    /// Coordinates of a point entity.
    pub fn point(&self, tag: i32) -> Result<[f64; 3], GmshError> {
        let (mut coords, mut len) = (std::ptr::null_mut(), 0);
        self.call(|e| unsafe {
            (self.api.get_value)(0, tag, std::ptr::null(), 0, &mut coords, &mut len, e);
        })?;
        match self.take_vec(coords, len)[..] {
            [x, y, z] => Ok([x, y, z]),
            _ => Err(GmshError::Other(format!("Point {tag} has no coordinates"))),
        }
    }

    /// Length of a curve, area of a surface or volume of a solid of the OpenCASCADE model.
    pub fn mass(&self, dim: i32, tag: i32) -> Result<f64, GmshError> {
        let mut mass = 0.0;
        self.call(|e| unsafe { (self.api.occ_get_mass)(dim, tag, &mut mass, e) })?;
        Ok(mass)
    }

    /// Centre of mass of an entity of the OpenCASCADE model.
    pub fn center_of_mass(&self, dim: i32, tag: i32) -> Result<[f64; 3], GmshError> {
        let [mut x, mut y, mut z] = [0.0; 3];
        self.call(|e| unsafe {
            (self.api.occ_get_center_of_mass)(dim, tag, &mut x, &mut y, &mut z, e);
        })?;
        Ok([x, y, z])
    }

    /// Bounds of the parameters of a curve (one) or surface (two).
    pub fn parameter_bounds(&self, dim: i32, tag: i32) -> Result<(Vec<f64>, Vec<f64>), GmshError> {
        let (mut min, mut min_len) = (std::ptr::null_mut(), 0);
        let (mut max, mut max_len) = (std::ptr::null_mut(), 0);
        self.call(|e| unsafe {
            (self.api.get_parametrization_bounds)(
                dim,
                tag,
                &mut min,
                &mut min_len,
                &mut max,
                &mut max_len,
                e,
            );
        })?;
        Ok((self.take_vec(min, min_len), self.take_vec(max, max_len)))
    }

    /// Points of a curve or surface at parameters, flat (one or two values per point).
    pub fn values(
        &self,
        dim: i32,
        tag: i32,
        parameters: &[f64],
    ) -> Result<Vec<[f64; 3]>, GmshError> {
        let (mut coords, mut len) = (std::ptr::null_mut(), 0);
        self.call(|e| unsafe {
            (self.api.get_value)(
                dim,
                tag,
                parameters.as_ptr(),
                parameters.len(),
                &mut coords,
                &mut len,
                e,
            );
        })?;
        Ok(self.take_vec(coords, len).as_chunks::<3>().0.to_vec())
    }

    /// The point of a curve or surface closest to a point.
    pub fn closest_point(
        &self,
        dim: i32,
        tag: i32,
        point: [f64; 3],
    ) -> Result<[f64; 3], GmshError> {
        Ok(self.projection(dim, tag, point)?.0)
    }

    /// The closest point on an entity, with its parametric coordinates.
    pub fn projection(
        &self,
        dim: i32,
        tag: i32,
        point: [f64; 3],
    ) -> Result<([f64; 3], Vec<f64>), GmshError> {
        let (mut coords, mut len) = (std::ptr::null_mut(), 0);
        let (mut parametric, mut parametric_len) = (std::ptr::null_mut(), 0);
        self.call(|e| unsafe {
            (self.api.get_closest_point)(
                dim,
                tag,
                point.as_ptr(),
                3,
                &mut coords,
                &mut len,
                &mut parametric,
                &mut parametric_len,
                e,
            );
        })?;
        let parametric = self.take_vec(parametric, parametric_len);
        match self.take_vec(coords, len)[..] {
            [x, y, z] => Ok(([x, y, z], parametric)),
            _ => Err(GmshError::Other(format!("No closest point on {dim}/{tag}"))),
        }
    }

    /// Whether parametric coordinates lie inside an entity, within its trimming bounds.
    pub fn is_inside(&self, dim: i32, tag: i32, parametric: &[f64]) -> Result<bool, GmshError> {
        let mut inside = 0;
        self.call(|e| unsafe {
            inside = (self.api.is_inside)(dim, tag, parametric.as_ptr(), parametric.len(), 1, e);
        })?;
        Ok(inside > 0)
    }

    /// Whether a point given by its coordinates lies inside an entity, e.g. a solid.
    pub fn contains_point(&self, dim: i32, tag: i32, point: [f64; 3]) -> Result<bool, GmshError> {
        let mut inside = 0;
        self.call(|e| unsafe {
            inside = (self.api.is_inside)(dim, tag, point.as_ptr(), 3, 0, e);
        })?;
        Ok(inside > 0)
    }

    /// Shows or hides entities with what bounds them; meshing can be limited to the shown
    /// ones with the option `Mesh.MeshOnlyVisible`.
    pub fn set_visibility(&self, entities: &[(i32, i32)], visible: bool) -> Result<(), GmshError> {
        let flat: Vec<c_int> = entities.iter().flat_map(|&(d, t)| [d, t]).collect();
        self.call(|e| unsafe {
            (self.api.set_visibility)(flat.as_ptr(), flat.len(), c_int::from(visible), 1, e);
        })
    }

    /// Moves entities by an affine transformation, the rows of a 3 x 4 matrix.
    pub fn affine_transform(
        &self,
        entities: &[(i32, i32)],
        matrix: &[f64; 12],
    ) -> Result<(), GmshError> {
        let flat: Vec<c_int> = entities.iter().flat_map(|&(d, t)| [d, t]).collect();
        self.call(|e| unsafe {
            (self.api.occ_affine_transform)(flat.as_ptr(), flat.len(), matrix.as_ptr(), 12, e);
        })?;
        self.synchronize()
    }

    /// The common part of the objects and the tools; the tools stay.
    pub fn intersect(
        &self,
        objects: &[(i32, i32)],
        tools: &[(i32, i32)],
    ) -> Result<Vec<(i32, i32)>, GmshError> {
        self.boolean(Boolean::Intersect, objects, tools, false)
    }

    /// A boolean operation of objects and tools; the objects are removed, the tools too if
    /// `remove_tools`. Returns the new entities.
    pub fn boolean(
        &self,
        operation: Boolean,
        objects: &[(i32, i32)],
        tools: &[(i32, i32)],
        remove_tools: bool,
    ) -> Result<Vec<(i32, i32)>, GmshError> {
        let function = match operation {
            Boolean::Intersect => self.api.occ_intersect,
            Boolean::Fuse => self.api.occ_fuse,
            Boolean::Cut => self.api.occ_cut,
        };
        let objects: Vec<c_int> = objects.iter().flat_map(|&(d, t)| [d, t]).collect();
        let tools: Vec<c_int> = tools.iter().flat_map(|&(d, t)| [d, t]).collect();
        let (mut out, mut out_len) = (std::ptr::null_mut(), 0);
        let (mut map, mut map_lens, mut map_len) =
            (std::ptr::null_mut(), std::ptr::null_mut(), 0usize);
        self.call(|e| unsafe {
            function(
                objects.as_ptr(),
                objects.len(),
                tools.as_ptr(),
                tools.len(),
                &mut out,
                &mut out_len,
                &mut map,
                &mut map_lens,
                &mut map_len,
                -1,
                1,
                c_int::from(remove_tools),
                e,
            );
        })?;
        // The map of which input gave which output is not needed.
        let maps: Vec<*mut c_int> = self.take_vec(map, map_len);
        for pointer in maps {
            self.take_vec(pointer, 0);
        }
        self.take_vec(map_lens, map_len);
        let out = self.take_vec(out, out_len);
        self.synchronize()?;
        Ok(pairs(&out))
    }

    /// A boolean operation as [`Self::boolean`] that may keep the objects.
    pub fn boolean_keeping(
        &self,
        operation: Boolean,
        objects: &[(i32, i32)],
        tools: &[(i32, i32)],
        remove_object: bool,
        remove_tool: bool,
    ) -> Result<Vec<(i32, i32)>, GmshError> {
        self.boolean_with_map(
            operation.function(&self.api),
            objects,
            tools,
            remove_object,
            remove_tool,
        )
        .map(|(out, _)| out)
    }

    /// Cuts objects and tools at each other where they meet, so that they share what lies
    /// on both, objects and tools removed; returns the new entities and, for each object and
    /// then each tool, the entities it became.
    pub fn fragment(
        &self,
        objects: &[(i32, i32)],
        tools: &[(i32, i32)],
    ) -> Result<Fragments, GmshError> {
        self.boolean_with_map(self.api.occ_fragment, objects, tools, true, true)
    }

    fn boolean_with_map(
        &self,
        function: BooleanFn,
        objects: &[(i32, i32)],
        tools: &[(i32, i32)],
        remove_object: bool,
        remove_tool: bool,
    ) -> Result<Fragments, GmshError> {
        let objects: Vec<c_int> = objects.iter().flat_map(|&(d, t)| [d, t]).collect();
        let tools: Vec<c_int> = tools.iter().flat_map(|&(d, t)| [d, t]).collect();
        let (mut out, mut out_len) = (std::ptr::null_mut(), 0);
        let (mut map, mut map_lens, mut map_len) =
            (std::ptr::null_mut(), std::ptr::null_mut(), 0usize);
        self.call(|e| unsafe {
            function(
                objects.as_ptr(),
                objects.len(),
                tools.as_ptr(),
                tools.len(),
                &mut out,
                &mut out_len,
                &mut map,
                &mut map_lens,
                &mut map_len,
                -1,
                c_int::from(remove_object),
                c_int::from(remove_tool),
                e,
            );
        })?;
        let lens: Vec<usize> = self.take_vec(map_lens, map_len);
        let maps: Vec<*mut c_int> = self.take_vec(map, map_len);
        let maps = (maps.into_iter().zip(lens))
            .map(|(pointer, len)| pairs(&self.take_vec(pointer, len)))
            .collect();
        let out = self.take_vec(out, out_len);
        self.synchronize()?;
        Ok((pairs(&out), maps))
    }

    /// Rounds edges of solids with a radius, the solids replaced by the rounded ones; returns
    /// the new solids.
    pub fn fillet(
        &self,
        volumes: &[i32],
        curves: &[i32],
        radius: f64,
    ) -> Result<Vec<(i32, i32)>, GmshError> {
        let (mut out, mut out_len) = (std::ptr::null_mut(), 0);
        self.call(|e| unsafe {
            (self.api.occ_fillet)(
                volumes.as_ptr(),
                volumes.len(),
                curves.as_ptr(),
                curves.len(),
                &radius,
                1,
                &mut out,
                &mut out_len,
                1,
                e,
            );
        })?;
        let out = self.take_vec(out, out_len);
        self.synchronize()?;
        Ok(pairs(&out))
    }

    /// Bevels edges of solids by a distance measured on the given face of each edge, the
    /// solids replaced by the bevelled ones; returns the new solids.
    pub fn chamfer(
        &self,
        volumes: &[i32],
        curves: &[i32],
        surfaces: &[i32],
        distance: f64,
    ) -> Result<Vec<(i32, i32)>, GmshError> {
        let (mut out, mut out_len) = (std::ptr::null_mut(), 0);
        self.call(|e| unsafe {
            (self.api.occ_chamfer)(
                volumes.as_ptr(),
                volumes.len(),
                curves.as_ptr(),
                curves.len(),
                surfaces.as_ptr(),
                surfaces.len(),
                &distance,
                1,
                &mut out,
                &mut out_len,
                1,
                e,
            );
        })?;
        let out = self.take_vec(out, out_len);
        self.synchronize()?;
        Ok(pairs(&out))
    }

    /// Extrudes entities along a vector; returns the new entities, among them the solids
    /// swept by faces.
    pub fn extrude(
        &self,
        entities: &[(i32, i32)],
        along: [f64; 3],
    ) -> Result<Vec<(i32, i32)>, GmshError> {
        let entities: Vec<c_int> = entities.iter().flat_map(|&(d, t)| [d, t]).collect();
        let (mut out, mut out_len) = (std::ptr::null_mut(), 0);
        let [dx, dy, dz] = along;
        self.call(|e| unsafe {
            (self.api.occ_extrude)(
                entities.as_ptr(),
                entities.len(),
                dx,
                dy,
                dz,
                &mut out,
                &mut out_len,
                std::ptr::null(),
                0,
                std::ptr::null(),
                0,
                0,
                e,
            );
        })?;
        let out = self.take_vec(out, out_len);
        self.synchronize()?;
        Ok(pairs(&out))
    }

    /// Revolves entities about an axis through a point by an angle in radians; returns the
    /// new entities.
    pub fn revolve(
        &self,
        entities: &[(i32, i32)],
        point: [f64; 3],
        axis: [f64; 3],
        angle: f64,
    ) -> Result<Vec<(i32, i32)>, GmshError> {
        let flat: Vec<c_int> = entities.iter().flat_map(|&(d, t)| [d, t]).collect();
        let (mut out, mut out_len) = (std::ptr::null_mut(), 0);
        let [x, y, z] = point;
        let [ax, ay, az] = axis;
        self.call(|e| unsafe {
            (self.api.occ_revolve)(
                flat.as_ptr(),
                flat.len(),
                x,
                y,
                z,
                ax,
                ay,
                az,
                angle,
                &mut out,
                &mut out_len,
                std::ptr::null(),
                0,
                std::ptr::null(),
                0,
                0,
                e,
            );
        })?;
        let out = self.take_vec(out, out_len);
        self.synchronize()?;
        Ok(pairs(&out))
    }

    /// Adds a mesh size field of a type such as `Constant` or `Min`; returns its tag.
    pub fn add_field(&self, kind: &str) -> Result<i32, GmshError> {
        let kind = c_string(kind)?;
        let mut tag = 0;
        self.call(|e| tag = unsafe { (self.api.field_add)(kind.as_ptr(), -1, e) })?;
        Ok(tag)
    }

    pub fn set_field_number(&self, field: i32, option: &str, value: f64) -> Result<(), GmshError> {
        let option = c_string(option)?;
        self.call(|e| unsafe { (self.api.field_set_number)(field, option.as_ptr(), value, e) })
    }

    pub fn set_field_numbers(
        &self,
        field: i32,
        option: &str,
        values: &[f64],
    ) -> Result<(), GmshError> {
        let option = c_string(option)?;
        self.call(|e| unsafe {
            (self.api.field_set_numbers)(field, option.as_ptr(), values.as_ptr(), values.len(), e);
        })
    }

    /// Makes the field the element size everywhere, combined with the other size limits.
    pub fn set_background_field(&self, field: i32) -> Result<(), GmshError> {
        self.call(|e| unsafe { (self.api.field_set_as_background)(field, e) })
    }

    fn synchronize(&self) -> Result<(), GmshError> {
        self.call(|e| unsafe { (self.api.occ_synchronize)(e) })
    }

    /// Tags of all entities of a dimension (0 points, 1 curves, 2 surfaces, 3 volumes).
    pub fn entities(&self, dim: i32) -> Result<Vec<i32>, GmshError> {
        let (mut tags, mut len) = (std::ptr::null_mut(), 0);
        self.call(|e| unsafe { (self.api.get_entities)(&mut tags, &mut len, dim, e) })?;
        Ok(pairs(&self.take_vec(tags, len))
            .into_iter()
            .map(|(_, tag)| tag)
            .collect())
    }

    pub fn entity_name(&self, dim: i32, tag: i32) -> Result<String, GmshError> {
        let mut name = std::ptr::null_mut();
        self.call(|e| unsafe { (self.api.get_entity_name)(dim, tag, &mut name, e) })?;
        Ok(self.take_string(name))
    }

    /// Gmsh's type of an entity, e.g. "Plane", "Cylinder", "Line", "Circle", "Point".
    pub fn entity_type(&self, dim: i32, tag: i32) -> Result<String, GmshError> {
        let mut kind = std::ptr::null_mut();
        self.call(|e| unsafe { (self.api.get_type)(dim, tag, &mut kind, e) })?;
        Ok(self.take_string(kind))
    }

    pub fn set_entity_name(&self, dim: i32, tag: i32, name: &str) -> Result<(), GmshError> {
        let name = c_string(name)?;
        self.call(|e| unsafe { (self.api.set_entity_name)(dim, tag, name.as_ptr(), e) })
    }

    /// Entities of the next higher and the next lower dimension touching an entity.
    pub fn adjacencies(&self, dim: i32, tag: i32) -> Result<(Vec<i32>, Vec<i32>), GmshError> {
        let (mut up, mut up_len) = (std::ptr::null_mut(), 0);
        let (mut down, mut down_len) = (std::ptr::null_mut(), 0);
        self.call(|e| unsafe {
            (self.api.get_adjacencies)(dim, tag, &mut up, &mut up_len, &mut down, &mut down_len, e);
        })?;
        Ok((self.take_vec(up, up_len), self.take_vec(down, down_len)))
    }

    /// Bounding box of the whole model.
    pub fn bounding_box(&self) -> Result<([f64; 3], [f64; 3]), GmshError> {
        self.entity_bounding_box(-1, -1)
    }

    /// Bounding box of one entity.
    pub fn entity_bounding_box(
        &self,
        dim: i32,
        tag: i32,
    ) -> Result<([f64; 3], [f64; 3]), GmshError> {
        let (mut min, mut max) = ([0.0; 3], [0.0; 3]);
        let [x0, y0, z0] = &mut min;
        let [x1, y1, z1] = &mut max;
        self.call(|e| unsafe { (self.api.get_bounding_box)(dim, tag, x0, y0, z0, x1, y1, z1, e) })?;
        Ok((min, max))
    }

    pub fn generate(&self, dim: i32) -> Result<(), GmshError> {
        self.call(|e| unsafe { (self.api.mesh_generate)(dim, e) })
    }

    /// Takes the triangulation OpenCASCADE makes of each face, with the deflections of the
    /// options `Mesh.StlLinearDeflection` and `Mesh.StlAngularDeflection`, as the mesh of the
    /// faces. Its triangles may be long and thin, so it is for display only.
    pub fn import_stl(&self) -> Result<(), GmshError> {
        self.call(|e| unsafe { (self.api.mesh_import_stl)(e) })
    }

    pub fn set_order(&self, order: i32) -> Result<(), GmshError> {
        self.call(|e| unsafe { (self.api.mesh_set_order)(order, e) })
    }

    /// Tags and coordinates of all mesh nodes.
    pub fn nodes(&self) -> Result<(Vec<usize>, Vec<f64>), GmshError> {
        let (mut tags, mut tags_len) = (std::ptr::null_mut(), 0);
        let (mut coords, mut coords_len) = (std::ptr::null_mut(), 0);
        let (mut parametric, mut parametric_len) = (std::ptr::null_mut(), 0);
        self.call(|e| unsafe {
            (self.api.mesh_get_nodes)(
                &mut tags,
                &mut tags_len,
                &mut coords,
                &mut coords_len,
                &mut parametric,
                &mut parametric_len,
                -1,
                -1,
                0,
                0,
                e,
            );
        })?;
        self.take_vec(parametric, parametric_len);
        Ok((
            self.take_vec(tags, tags_len),
            self.take_vec(coords, coords_len),
        ))
    }

    /// Elements of a Gmsh element type on one entity: their tags and node tags, `n` per
    /// element.
    pub fn elements(
        &self,
        element_type: i32,
        entity: i32,
    ) -> Result<(Vec<usize>, Vec<usize>), GmshError> {
        let (mut tags, mut tags_len) = (std::ptr::null_mut(), 0);
        let (mut nodes, mut nodes_len) = (std::ptr::null_mut(), 0);
        self.call(|e| unsafe {
            (self.api.mesh_get_elements_by_type)(
                element_type,
                &mut tags,
                &mut tags_len,
                &mut nodes,
                &mut nodes_len,
                entity,
                0,
                1,
                e,
            );
        })?;
        Ok((
            self.take_vec(tags, tags_len),
            self.take_vec(nodes, nodes_len),
        ))
    }

    fn start_log(&self) -> Result<(), GmshError> {
        // Restarting empties the log of the previous use.
        self.call(|e| unsafe { (self.api.logger_stop)(e) })?;
        self.call(|e| unsafe { (self.api.logger_start)(e) })
    }

    /// Warnings and errors Gmsh logged since the current use started.
    pub fn warnings(&self) -> Result<Vec<String>, GmshError> {
        let (mut lines, mut len) = (std::ptr::null_mut(), 0);
        self.call(|e| unsafe { (self.api.logger_get)(&mut lines, &mut len, e) })?;
        let lines = self.take_vec(lines, len);
        Ok(lines
            .into_iter()
            .map(|line| self.take_string(line))
            .filter_map(|line| {
                let rest = line
                    .strip_prefix("Warning : ")
                    .or_else(|| line.strip_prefix("Error   : "))?;
                Some(rest.trim().to_string())
            })
            .collect())
    }
}

/// Gmsh's flat (dim, tag) lists as pairs.
fn pairs(flat: &[c_int]) -> Vec<(i32, i32)> {
    flat.as_chunks::<2>()
        .0
        .iter()
        .map(|&[a, b]| (a, b))
        .collect()
}

fn c_string(text: &str) -> Result<CString, GmshError> {
    CString::new(text).map_err(|_| GmshError::Other(format!("invalid text: {text:?}")))
}

fn c_path(path: &Path) -> Result<CString, GmshError> {
    // Gmsh expects UTF-8 file names on every platform.
    let text = path
        .to_str()
        .ok_or_else(|| GmshError::Other(format!("{}: path is not UTF-8", path.display())))?;
    c_string(text)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn versions_are_parsed() {
        assert_eq!(parse_version("4.15.2"), Some((4, 15)));
        assert_eq!(parse_version("4.13"), Some((4, 13)));
        assert_eq!(parse_version("x"), None);
    }

    #[test]
    fn a_configured_path_is_the_only_candidate() {
        let path = PathBuf::from("/opt/gmsh/libgmsh.so");
        assert_eq!(candidates(Some(&path)), vec![path]);
        // Without one, the system loader is asked last.
        if std::env::var_os(LIBRARY_ENV).is_none() {
            let names = candidates(None);
            assert!(names.last().is_some_and(|p| p.components().count() == 1));
        }
    }

    #[test]
    fn a_missing_configured_library_is_reported() {
        let missing = std::env::temp_dir()
            .join("plx-gibt-es-nicht")
            .join("libgmsh.so");
        assert!(matches!(
            Gmsh::load(Some(&missing)),
            Err(GmshError::NotFound(_))
        ));
    }
}
