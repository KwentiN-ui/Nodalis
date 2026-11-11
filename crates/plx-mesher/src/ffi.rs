//! The part of Gmsh's C API (`gmshc.h`) nodalis uses, loaded from the shared library at run
//! time. Every function reports errors through its last argument `ierr`; arrays and strings it
//! returns are allocated by Gmsh and released with `gmshFree`.

#![allow(unsafe_code)]

use std::ffi::{c_char, c_double, c_int, c_void};

use libloading::Library;

macro_rules! gmsh_api {
    ($($field:ident = $symbol:literal: fn($($arg:ty),* $(,)?) $(-> $ret:ty)?;)*) => {
        /// Function pointers into the loaded library, which they keep alive.
        pub(crate) struct Api {
            $(pub $field: unsafe extern "C" fn($($arg),*) $(-> $ret)?,)*
            _library: Library,
        }

        impl Api {
            /// Looks up every function; fails with the names the library lacks, e.g. because
            /// it is an older Gmsh.
            ///
            /// # Safety
            /// The library must be Gmsh, so that the symbols have the declared signatures.
            pub unsafe fn load(library: Library) -> Result<Self, Vec<&'static str>> {
                let mut missing = Vec::new();
                $(
                    // SAFETY: the caller guarantees that the symbol has this signature; the
                    // pointer stays valid because the library is stored next to it.
                    let $field = unsafe {
                        library.get::<unsafe extern "C" fn($($arg),*) $(-> $ret)?>(
                            concat!($symbol, "\0").as_bytes(),
                        )
                    }
                    .map(|symbol| *symbol)
                    .map_err(|_| missing.push($symbol))
                    .ok();
                )*
                match ($($field,)*) {
                    ($(Some($field),)*) => Ok(Self { $($field,)* _library: library }),
                    _ => Err(missing),
                }
            }
        }
    };
}

type Ierr = *mut c_int;

gmsh_api! {
    initialize = "gmshInitialize": fn(c_int, *mut *mut c_char, c_int, c_int, Ierr);
    clear = "gmshClear": fn(Ierr);
    free = "gmshFree": fn(*mut c_void);
    write = "gmshWrite": fn(*const c_char, Ierr);
    option_set_number = "gmshOptionSetNumber": fn(*const c_char, c_double, Ierr);
    option_set_string = "gmshOptionSetString": fn(*const c_char, *const c_char, Ierr);
    option_get_string = "gmshOptionGetString": fn(*const c_char, *mut *mut c_char, Ierr);
    model_add = "gmshModelAdd": fn(*const c_char, Ierr);
    get_entities = "gmshModelGetEntities": fn(*mut *mut c_int, *mut usize, c_int, Ierr);
    get_entity_name = "gmshModelGetEntityName": fn(c_int, c_int, *mut *mut c_char, Ierr);
    set_entity_name = "gmshModelSetEntityName": fn(c_int, c_int, *const c_char, Ierr);
    get_type = "gmshModelGetType": fn(c_int, c_int, *mut *mut c_char, Ierr);
    get_adjacencies = "gmshModelGetAdjacencies":
        fn(c_int, c_int, *mut *mut c_int, *mut usize, *mut *mut c_int, *mut usize, Ierr);
    get_bounding_box = "gmshModelGetBoundingBox": fn(
        c_int, c_int,
        *mut c_double, *mut c_double, *mut c_double,
        *mut c_double, *mut c_double, *mut c_double,
        Ierr,
    );
    get_value = "gmshModelGetValue":
        fn(c_int, c_int, *const c_double, usize, *mut *mut c_double, *mut usize, Ierr);
    occ_get_mass = "gmshModelOccGetMass": fn(c_int, c_int, *mut c_double, Ierr);
    occ_get_center_of_mass = "gmshModelOccGetCenterOfMass":
        fn(c_int, c_int, *mut c_double, *mut c_double, *mut c_double, Ierr);
    get_parametrization_bounds = "gmshModelGetParametrizationBounds":
        fn(c_int, c_int, *mut *mut c_double, *mut usize, *mut *mut c_double, *mut usize, Ierr);
    get_closest_point = "gmshModelGetClosestPoint": fn(
        c_int, c_int,
        *const c_double, usize,
        *mut *mut c_double, *mut usize,
        *mut *mut c_double, *mut usize,
        Ierr,
    );
    get_normal = "gmshModelGetNormal":
        fn(c_int, *const c_double, usize, *mut *mut c_double, *mut usize, Ierr);
    get_principal_curvatures = "gmshModelGetPrincipalCurvatures": fn(
        c_int,
        *const c_double, usize,
        *mut *mut c_double, *mut usize,
        *mut *mut c_double, *mut usize,
        *mut *mut c_double, *mut usize,
        *mut *mut c_double, *mut usize,
        Ierr,
    );
    occ_defeature = "gmshModelOccDefeature": fn(
        *const c_int, usize,
        *const c_int, usize,
        *mut *mut c_int, *mut usize,
        c_int,
        Ierr,
    );
    occ_copy = "gmshModelOccCopy": fn(*const c_int, usize, *mut *mut c_int, *mut usize, Ierr);
    is_inside = "gmshModelIsInside": fn(c_int, c_int, *const c_double, usize, c_int, Ierr) -> c_int;
    set_visibility = "gmshModelSetVisibility": fn(*const c_int, usize, c_int, c_int, Ierr);
    occ_translate = "gmshModelOccTranslate":
        fn(*const c_int, usize, c_double, c_double, c_double, Ierr);
    occ_affine_transform = "gmshModelOccAffineTransform":
        fn(*const c_int, usize, *const c_double, usize, Ierr);
    occ_intersect = "gmshModelOccIntersect": fn(
        *const c_int, usize,
        *const c_int, usize,
        *mut *mut c_int, *mut usize,
        *mut *mut *mut c_int, *mut *mut usize, *mut usize,
        c_int, c_int, c_int,
        Ierr,
    );
    occ_fillet = "gmshModelOccFillet": fn(
        *const c_int, usize,
        *const c_int, usize,
        *const c_double, usize,
        *mut *mut c_int, *mut usize,
        c_int,
        Ierr,
    );
    occ_chamfer = "gmshModelOccChamfer": fn(
        *const c_int, usize,
        *const c_int, usize,
        *const c_int, usize,
        *const c_double, usize,
        *mut *mut c_int, *mut usize,
        c_int,
        Ierr,
    );
    occ_extrude = "gmshModelOccExtrude": fn(
        *const c_int, usize,
        c_double, c_double, c_double,
        *mut *mut c_int, *mut usize,
        *const c_int, usize,
        *const c_double, usize,
        c_int,
        Ierr,
    );
    occ_fuse = "gmshModelOccFuse": fn(
        *const c_int, usize,
        *const c_int, usize,
        *mut *mut c_int, *mut usize,
        *mut *mut *mut c_int, *mut *mut usize, *mut usize,
        c_int, c_int, c_int,
        Ierr,
    );
    occ_cut = "gmshModelOccCut": fn(
        *const c_int, usize,
        *const c_int, usize,
        *mut *mut c_int, *mut usize,
        *mut *mut *mut c_int, *mut *mut usize, *mut usize,
        c_int, c_int, c_int,
        Ierr,
    );
    occ_fragment = "gmshModelOccFragment": fn(
        *const c_int, usize,
        *const c_int, usize,
        *mut *mut c_int, *mut usize,
        *mut *mut *mut c_int, *mut *mut usize, *mut usize,
        c_int, c_int, c_int,
        Ierr,
    );
    occ_revolve = "gmshModelOccRevolve": fn(
        *const c_int, usize,
        c_double, c_double, c_double,
        c_double, c_double, c_double,
        c_double,
        *mut *mut c_int, *mut usize,
        *const c_int, usize,
        *const c_double, usize,
        c_int,
        Ierr,
    );
    occ_import_shapes = "gmshModelOccImportShapes":
        fn(*const c_char, *mut *mut c_int, *mut usize, c_int, *const c_char, Ierr);
    occ_add_box = "gmshModelOccAddBox":
        fn(c_double, c_double, c_double, c_double, c_double, c_double, c_int, Ierr) -> c_int;
    occ_add_cylinder = "gmshModelOccAddCylinder": fn(
        c_double, c_double, c_double, c_double, c_double, c_double, c_double, c_int, c_double,
        Ierr,
    ) -> c_int;
    occ_add_sphere = "gmshModelOccAddSphere": fn(
        c_double, c_double, c_double, c_double, c_int, c_double, c_double, c_double, Ierr,
    ) -> c_int;
    occ_add_disk = "gmshModelOccAddDisk": fn(
        c_double, c_double, c_double, c_double, c_double, c_int,
        *const c_double, usize,
        *const c_double, usize,
        Ierr,
    ) -> c_int;
    occ_add_rectangle = "gmshModelOccAddRectangle":
        fn(c_double, c_double, c_double, c_double, c_double, c_int, c_double, Ierr) -> c_int;
    occ_add_point = "gmshModelOccAddPoint":
        fn(c_double, c_double, c_double, c_double, c_int, Ierr) -> c_int;
    occ_add_line = "gmshModelOccAddLine": fn(c_int, c_int, c_int, Ierr) -> c_int;
    occ_synchronize = "gmshModelOccSynchronize": fn(Ierr);
    occ_remove = "gmshModelOccRemove": fn(*const c_int, usize, c_int, Ierr);
    occ_remove_all_duplicates = "gmshModelOccRemoveAllDuplicates": fn(Ierr);
    field_add = "gmshModelMeshFieldAdd": fn(*const c_char, c_int, Ierr) -> c_int;
    field_set_number = "gmshModelMeshFieldSetNumber": fn(c_int, *const c_char, c_double, Ierr);
    field_set_numbers = "gmshModelMeshFieldSetNumbers":
        fn(c_int, *const c_char, *const c_double, usize, Ierr);
    field_set_as_background = "gmshModelMeshFieldSetAsBackgroundMesh": fn(c_int, Ierr);
    field_remove = "gmshModelMeshFieldRemove": fn(c_int, Ierr);
    mesh_clear = "gmshModelMeshClear": fn(*const c_int, usize, Ierr);
    mesh_generate = "gmshModelMeshGenerate": fn(c_int, Ierr);
    mesh_set_order = "gmshModelMeshSetOrder": fn(c_int, Ierr);
    mesh_import_stl = "gmshModelMeshImportStl": fn(Ierr);
    mesh_get_nodes = "gmshModelMeshGetNodes": fn(
        *mut *mut usize, *mut usize,
        *mut *mut c_double, *mut usize,
        *mut *mut c_double, *mut usize,
        c_int, c_int, c_int, c_int,
        Ierr,
    );
    mesh_get_elements_by_type = "gmshModelMeshGetElementsByType": fn(
        c_int,
        *mut *mut usize, *mut usize,
        *mut *mut usize, *mut usize,
        c_int, usize, usize,
        Ierr,
    );
    logger_start = "gmshLoggerStart": fn(Ierr);
    logger_get = "gmshLoggerGet": fn(*mut *mut *mut c_char, *mut usize, Ierr);
    logger_stop = "gmshLoggerStop": fn(Ierr);
    logger_get_last_error = "gmshLoggerGetLastError": fn(*mut *mut c_char, Ierr);
}
