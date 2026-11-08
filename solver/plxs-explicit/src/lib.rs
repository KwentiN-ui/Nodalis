//! Explicit time steps on the graphics card: central differences with a lumped mass,
//! constraints `u = T q + g` and the reduced mass inverted in clusters ([`Reduction`]),
//! for a model with or without contact (with the element kernels: elastic materials under small strains
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
//! **One step** is a few dispatches: [`advance`](../advance.wgsl) (reduced velocities and
//! displacements), [`expand`](../expand.wgsl) (`u = T q + g`, `v = T q̇`), the forces,
//! [`accelerate`](../accelerate.wgsl) (forces, work, residual) and twice
//! [`sparse`](../sparse.wgsl) (`Tᵀ r`, then the reduced accelerations). The forces come from the elements ([`Elements`],
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

mod contact;
mod elements;
mod guard;
mod reduction;
pub use contact::{
    Candidates, Contact, ContactFace, ContactLaw, ContactPair, ContactPoint, ContactState,
};
pub use elements::{
    Block, Element, Elements, Family, Hourglass, Isochoric, MAX_NODES, Material, Points,
    STATE_SIZE, Volumetric,
};
pub use guard::{GUARD_RATIO, Guard};
pub use reduction::{Clusters, MAX_CLUSTER, Reduction, Sparse};

/// The most time steps one [`Engine::run`] takes.
pub const MAX_CHUNK: usize = 256;

/// What the card needs once per step: the elements, the lumped mass per DOF (for the
/// damping), the constraints with the inverse reduced mass, and the mass-proportional
/// damping.
pub struct Setup<'a> {
    pub elements: &'a Elements,
    pub mass: &'a [f64],
    pub reduction: &'a Reduction,
    /// `α` of the damping force `α m v`.
    pub alpha: f64,
    /// Whether the prescribed displacements change: the card then also adds up the work
    /// of their reactions.
    pub prescribed_motion: bool,
    /// The contact pairs with the candidates of their last search, `None` without
    /// contact.
    pub contact: Option<(&'a Contact, Candidates<'a>)>,
    /// The elements whose shortest distances watch the time step under large
    /// deformations ([`Engine::departure`]), `None` under small ones.
    pub guard: Option<&'a Guard>,
}

/// The state at the start of a run of steps: the reduced displacements, half-step
/// velocities and accelerations, the prescribed displacements `g` and their rate over the
/// last step (over the first step when there was none, so that the first step sees no
/// prescribed acceleration), the displacements and velocities of the DOFs (`T q + g`,
/// `T q̇ + ġ`), the internal, viscous (with damping) and external forces at it, and the
/// reactions of the prescribed DOFs (empty: zero) for the work of the prescribed motion.
pub struct Start<'a> {
    pub q: &'a [f64],
    pub qdot: &'a [f64],
    pub a: &'a [f64],
    pub prescribed: &'a [f64],
    pub prescribed_rate: &'a [f64],
    pub reactions: &'a [f64],
    pub u: &'a [f64],
    pub v: &'a [f64],
    pub internal: &'a [f64],
    pub viscous: &'a [f64],
    pub loads: &'a [f64],
    /// The material state ([`STATE_SIZE`] per integration point; see [`Element::state`]),
    /// empty when no element has state variables.
    pub material: &'a [f64],
    /// The state of the contact points (`None`: open, at full stiffness).
    pub contact: Option<&'a ContactState>,
}

/// The state the card reached: the reduced displacements, half-step velocities and
/// accelerations, the displacements and velocities of the DOFs, and the work of the
/// internal, external and viscous forces since [`Engine::load`].
#[derive(Clone, Debug, Default)]
pub struct Reached {
    pub q: Vec<f64>,
    pub qdot: Vec<f64>,
    pub a: Vec<f64>,
    pub u: Vec<f64>,
    pub v: Vec<f64>,
    pub internal_work: f64,
    /// With the work of the prescribed displacements.
    pub external_work: f64,
    pub dissipation: f64,
    /// The rate of the prescribed displacements over the last step.
    pub prescribed_rate: Vec<f64>,
    /// The reactions of the last step (empty without prescribed motion).
    pub reactions: Vec<f64>,
    /// The internal and the viscous forces (with the damping `α m v`) at the end.
    pub internal: Vec<f64>,
    pub viscous: Vec<f64>,
    /// The state of the contact points (`None` without contact).
    pub contact: Option<ContactState>,
}

/// How a run of time steps ended: the time steps taken and, when it stopped early, why.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Ran {
    pub steps: usize,
    /// A contact node moved farther than the candidates cover: search again.
    pub search: bool,
    /// A contact opened or closed in the last time step taken (from the step
    /// [`Engine::events_from`] allows on, the run stops there).
    pub event: bool,
}

/// The explicit time steps of one step on the graphics card.
pub struct Engine {
    #[cfg(feature = "gpu")]
    inner: gpu::Engine,
}

impl Engine {
    /// Opens the card and uploads the elements and the constraints. `any_adapter` takes any adapter, software
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

    /// Takes the state to start from. The work starts again at zero.
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

    /// Uploads the candidates of a new search.
    pub fn candidates(&mut self, candidates: &Candidates<'_>) -> Result<(), String> {
        #[cfg(feature = "gpu")]
        {
            self.inner.candidates(candidates)
        }
        #[cfg(not(feature = "gpu"))]
        {
            let _ = candidates;
            Err("built without graphics card support".to_owned())
        }
    }

    /// From which time step of the next runs (counted from the first of the run) a contact
    /// that opens or closes stops the run, for an event frame; `None`: never.
    pub fn events_from(&mut self, from: Option<usize>) {
        #[cfg(feature = "gpu")]
        {
            self.inner.events_from = from.map_or(u32::MAX, |f| f.min(u32::MAX as usize) as u32);
        }
        #[cfg(not(feature = "gpu"))]
        {
            let _ = from;
        }
    }

    /// Runs the time steps `steps` (`(h, c)`: the step and the factor of the velocity
    /// update, `½ (h_before + h)`), at most [`MAX_CHUNK`], with the loads and the
    /// prescribed displacements going linearly from the first to the second over them.
    /// Waits for them; with contact the run can stop early ([`Ran`]).
    pub fn run(
        &mut self,
        steps: &[(f64, f64)],
        loads: [&[f64]; 2],
        prescribed: [&[f64]; 2],
    ) -> Result<Ran, String> {
        if steps.len() > MAX_CHUNK {
            return Err(format!("at most {MAX_CHUNK} time steps per run"));
        }
        #[cfg(feature = "gpu")]
        {
            self.inner.run(steps, loads, prescribed)
        }
        #[cfg(not(feature = "gpu"))]
        {
            let _ = (steps, loads, prescribed);
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

    /// Takes the squared lengths ([`Guard::lengths`]) that [`Self::departure`] compares
    /// with.
    pub fn guard_reference(&mut self, reference: &[f32]) -> Result<(), String> {
        #[cfg(feature = "gpu")]
        {
            self.inner.guard_reference(reference)
        }
        #[cfg(not(feature = "gpu"))]
        {
            let _ = reference;
            Err("built without graphics card support".to_owned())
        }
    }

    /// [`Guard::departure`] at the displacements on the card.
    pub fn departure(&mut self) -> Result<f32, String> {
        #[cfg(feature = "gpu")]
        {
            self.inner.departure()
        }
        #[cfg(not(feature = "gpu"))]
        {
            Err("built without graphics card support".to_owned())
        }
    }

    /// Waits for the queued steps and reads the displacements of the DOFs `dofs`.
    pub fn read_values(&mut self, dofs: &[u32]) -> Result<Vec<f64>, String> {
        #[cfg(feature = "gpu")]
        {
            self.inner.read_values(dofs)
        }
        #[cfg(not(feature = "gpu"))]
        {
            let _ = dofs;
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
