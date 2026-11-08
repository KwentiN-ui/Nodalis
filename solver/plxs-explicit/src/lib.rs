//! Explicit time steps on the graphics card: central differences with a lumped mass for a
//! model without contact (with the element kernels: elastic materials under small strains
//! or large deformations, J2 plasticity at small or finite strains, hyperelasticity, and
//! constant stiffness blocks), by compute shaders through wgpu (Vulkan, DirectX 12 or
//! Metal).
//!
//! **Precision.** Consumer cards compute double precision at a fraction of the single
//! precision rate (1/64 on an RTX 4070) and Metal has none. The state stays in double
//! precision: displacements, velocities, accelerations, the loads and the work of the
//! forces. The expensive part, the element forces, runs in single precision on
//! *relative* quantities: the displacements relative to the element's first node. The
//! rounding error is then relative to the motion within an element, not to the
//! displacements as a whole, and it does not accumulate: the forces are formed anew from
//! the state in every step. The energy balance the solver reports checks the result; cards
//! without double precision leave the step to the processor.
//!
//! **One step** is a few dispatches: [`advance`](../advance.wgsl) (velocities,
//! displacements), the forces, and [`accelerate`](../accelerate.wgsl) (forces, work,
//! accelerations). The forces come from the elements ([`Elements`],
//! [`elements`](../elements.wgsl) per family of elements, one thread per element, and
//! [`blocks`](../blocks.wgsl) for the constant blocks, one thread per block row), then
//! [`gather`](../gather.wgsl) sums them per DOF in double precision; no matrix is built. A
//! chunk of up to [`MAX_CHUNK`] steps goes to the card as one command buffer; the loads are
//! interpolated linearly between the chunk's ends, so the caller chooses chunks short
//! against the amplitudes.
//!
//! Without the feature `gpu` the crate only says that there is no card ([`Engine::new`]
//! fails).
//!
//! No CalculiX code was consulted.

#[cfg(feature = "gpu")]
mod gpu;

mod elements;
pub use elements::{
    Block, Element, Elements, Family, Hourglass, Isochoric, MAX_NODES, Material, Points,
    STATE_SIZE, Volumetric,
};

/// The most time steps one [`Engine::run`] takes.
pub const MAX_CHUNK: usize = 256;

/// What the card needs once per step: the elements, the inverse lumped mass (0 on the DOFs
/// that do not move: held, or no element uses them) and the mass-proportional damping.
pub struct Setup<'a> {
    pub elements: &'a Elements,
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
