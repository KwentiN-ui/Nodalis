//! Explicit time steps on the graphics card: central differences with a lumped mass for a
//! model without contact (with the element kernels: elastic materials under small strains
//! or large deformations, small-strain J2 plasticity; with the matrices: linear models), by compute shaders through
//! wgpu (Vulkan, DirectX 12 or Metal).
//!
//! **Precision.** Consumer cards compute double precision at a fraction of the single
//! precision rate (1/64 on an RTX 4070) and Metal has none. The state stays in double
//! precision: displacements, velocities, accelerations, the loads and the work of the
//! forces. The expensive part, the forces, runs in single precision on *relative*
//! quantities: the element forces on the displacements relative to the element's first
//! node; the products of the stiffness and the viscous matrix on the displacements since a
//! reference state (the start of the frame, whose internal forces the processor computed
//! in double precision) and the velocities. The rounding error is then relative to the
//! motion within an element or a frame, not to the displacements as a whole, and it does not accumulate:
//! the forces are formed anew from the state in every step. The energy balance the solver
//! reports checks the result; cards without double precision leave the step to the
//! processor.
//!
//! **One step** is a few dispatches: [`advance`](../advance.wgsl) (velocities,
//! displacements), the forces, and [`accelerate`](../accelerate.wgsl) (forces, work,
//! accelerations). The forces come either from the elements ([`Elements`],
//! [`elements`](../elements.wgsl) per family of elements, one thread per element, then
//! [`gather`](../gather.wgsl) sums them per DOF in double precision; no matrix is built), or
//! as the products `K Δu` and `C v` of the matrices in one pass over the common pattern of
//! both, stored in slices of rows so that neighbouring threads read neighbouring values
//! ([`product`](../product.wgsl)). A chunk of up to [`MAX_CHUNK`] steps goes to the card as
//! one command buffer; the loads are interpolated linearly between the chunk's ends, so the
//! caller chooses chunks short against the amplitudes.
//!
//! Without the feature `gpu` the crate only says that there is no card ([`Engine::new`]
//! fails).
//!
//! No CalculiX code was consulted.

#[cfg(feature = "gpu")]
mod gpu;
#[cfg(any(feature = "gpu", test))]
mod sliced;

mod elements;
pub use elements::{Element, Elements, Family, Hourglass, MAX_NODES, Material, STATE_SIZE};

/// The most time steps one [`Engine::run`] takes.
pub const MAX_CHUNK: usize = 256;

/// A sparse matrix by rows (both triangles), in single precision.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Csr {
    pub ptr: Vec<u32>,
    pub col: Vec<u32>,
    pub val: Vec<f32>,
}

impl Csr {
    /// The matrix of the entries `(row, col, value)` (repeated entries add up) of an
    /// `n × n` matrix.
    pub fn from_entries(n: usize, mut entries: Vec<(u32, u32, f64)>) -> Self {
        entries.sort_unstable_by_key(|(r, c, _)| (*r, *c));
        let mut out = Csr {
            ptr: vec![0; n + 1],
            ..Default::default()
        };
        let mut sums: Vec<(u32, u32, f64)> = Vec::with_capacity(entries.len());
        for (r, c, v) in entries {
            match sums.last_mut() {
                Some(last) if last.0 == r && last.1 == c => last.2 += v,
                _ => sums.push((r, c, v)),
            }
        }
        for (r, c, v) in sums {
            out.ptr[r as usize + 1] += 1;
            out.col.push(c);
            out.val.push(v as f32);
        }
        for i in 0..n {
            out.ptr[i + 1] += out.ptr[i];
        }
        out
    }

    /// `y = A x` in double precision, for tests.
    pub fn multiply(&self, x: &[f64]) -> Vec<f64> {
        (0..self.ptr.len() - 1)
            .map(|i| {
                (self.ptr[i] as usize..self.ptr[i + 1] as usize)
                    .map(|k| f64::from(self.val[k]) * x[self.col[k] as usize])
                    .sum()
            })
            .collect()
    }
}

/// How the card forms the internal and viscous forces.
pub enum ForceModel<'a> {
    /// The products of the stiffness and the viscous matrix with the motion since the
    /// reference state.
    Matrices {
        stiffness: &'a Csr,
        viscous: &'a Csr,
    },
    /// Element by element from the displacements and velocities ([`Elements`]).
    Elements(&'a Elements),
}

/// What the card needs once per step: the forces, the inverse lumped mass (0 on the DOFs
/// that do not move: held, or no element uses them) and the mass-proportional damping.
pub struct Setup<'a> {
    pub forces: ForceModel<'a>,
    pub inverse_mass: &'a [f64],
    /// `α` of the damping force `α m v`.
    pub alpha: f64,
}

/// The state at the start of a run of steps: displacements, half-step velocities,
/// accelerations, and the internal, viscous (with damping) and external forces at it.
pub struct Start<'a> {
    pub u: &'a [f64],
    pub v: &'a [f64],
    pub a: &'a [f64],
    pub internal: &'a [f64],
    pub viscous: &'a [f64],
    pub loads: &'a [f64],
    /// The material state ([`STATE_SIZE`] per integration point; see [`Element::state`]),
    /// empty when no element has state variables.
    pub material: &'a [f64],
}

/// The state the card reached: displacements, half-step velocities, accelerations and the
/// work of the internal, external and viscous forces since [`Engine::load`].
#[derive(Clone, Debug, Default)]
pub struct Reached {
    pub u: Vec<f64>,
    pub v: Vec<f64>,
    pub a: Vec<f64>,
    pub internal_work: f64,
    pub external_work: f64,
    pub dissipation: f64,
}

/// The explicit time steps of one step on the graphics card.
pub struct Engine {
    #[cfg(feature = "gpu")]
    inner: gpu::Engine,
}

impl Engine {
    /// Opens the card and uploads the matrices. `any_adapter` takes any adapter, software
    /// renderers included (tests, measurements); otherwise only a discrete card. `Err`
    /// tells why the steps stay on the processor.
    pub fn new(setup: &Setup<'_>, any_adapter: bool) -> Result<Self, String> {
        #[cfg(feature = "gpu")]
        {
            gpu::Engine::new(setup, any_adapter).map(|inner| Self { inner })
        }
        #[cfg(not(feature = "gpu"))]
        {
            let _ = (setup, any_adapter);
            Err("built without graphics card support".to_owned())
        }
    }

    /// The card, e.g. "NVIDIA GeForce RTX 4070 (discrete)".
    pub fn device(&self) -> &str {
        #[cfg(feature = "gpu")]
        {
            &self.inner.gpu.name
        }
        #[cfg(not(feature = "gpu"))]
        {
            ""
        }
    }

    /// Takes the state to start from; it is also the reference state of the single
    /// precision products. The work starts again at zero.
    pub fn load(&mut self, start: &Start<'_>) -> Result<(), String> {
        #[cfg(feature = "gpu")]
        {
            self.inner.load(start)
        }
        #[cfg(not(feature = "gpu"))]
        {
            let _ = start;
            Err("built without graphics card support".to_owned())
        }
    }

    /// Queues the time steps `steps` (`(h, c)`: the step and the factor of the velocity
    /// update, `½ (h_before + h)`), at most [`MAX_CHUNK`], with the loads going linearly
    /// from `from` to `to` over them.
    pub fn run(&mut self, steps: &[(f64, f64)], from: &[f64], to: &[f64]) -> Result<(), String> {
        if steps.len() > MAX_CHUNK {
            return Err(format!("at most {MAX_CHUNK} time steps per run"));
        }
        #[cfg(feature = "gpu")]
        {
            self.inner.run(steps, from, to)
        }
        #[cfg(not(feature = "gpu"))]
        {
            let _ = (steps, from, to);
            Err("built without graphics card support".to_owned())
        }
    }

    /// Writes the material state on the card into `material` (the vector of
    /// [`Start::material`]).
    pub fn read_state(&mut self, material: &mut [f64]) -> Result<(), String> {
        #[cfg(feature = "gpu")]
        {
            self.inner.read_state(material)
        }
        #[cfg(not(feature = "gpu"))]
        {
            let _ = material;
            Err("built without graphics card support".to_owned())
        }
    }

    /// Waits for the queued steps and reads the state back.
    pub fn read(&mut self) -> Result<Reached, String> {
        #[cfg(feature = "gpu")]
        {
            self.inner.read()
        }
        #[cfg(not(feature = "gpu"))]
        {
            Err("built without graphics card support".to_owned())
        }
    }
}
