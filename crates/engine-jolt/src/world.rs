//! The deterministic `World` -- a thin RAII handle around JoltC's
//! `JPC_PhysicsSystem` + the temp allocator + the single-threaded job
//! system + the three filter interfaces (broad-phase layer interface,
//! object-vs-broadphase filter, object-pair filter).
//!
//! v0.22's headline constraint is determinism. The contract:
//!
//! * **Single-threaded JobSystem.** `JPC_JobSystemSingleThreaded` is
//!   wired by default. Multi-threaded with pinned worker count is
//!   v0.23+ work; the single-threaded baseline removes thread-pool
//!   ordering from the determinism equation for v0.22.
//! * **Two-layer broadphase.** STATIC + MOVING per `crate::layers`.
//!   The filter callbacks are `static extern "C"` functions over
//!   immutable layer ids -- no Rust state, no captures, no ordering
//!   surprises.
//! * **`drive_step(dt, substeps)` is the ONE entrypoint.** No
//!   internal accumulator. Caller supplies `dt = 1/64s` from
//!   `Time<Fixed>` and a constant `substeps` (default 1).
//!
//! See `xtreme-game-engine/docs/v0.22-design.md` section "Determinism
//! contract" for the verbatim guarantee.

use glam::Vec3;

#[cfg(feature = "native")]
use std::ptr;
#[cfg(feature = "native")]
use crate::body_interface::BodyInterface;
#[cfg(feature = "native")]
use crate::layers::BroadPhaseLayer;

/// World configuration. Defaults match the v0.22 design doc:
/// gravity = (0, -9.81, 0), `num_velocity_steps = 10`,
/// `num_position_steps = 2` (Jolt defaults, set explicitly so future
/// flips show in git blame).
#[derive(Debug, Clone, Copy)]
pub struct WorldConfig {
    pub max_bodies: u32,
    pub max_body_pairs: u32,
    pub max_contact_constraints: u32,
    pub num_body_mutexes: u32,
    pub gravity: Vec3,
    pub num_velocity_steps: u32,
    pub num_position_steps: u32,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            // Match the framework's tested defaults; larger caps
            // require a bigger temp allocator (size empirically
            // linked to body-pair count during the solver pass).
            // Consumers needing more capacity should construct an
            // explicit WorldConfig.
            max_bodies: 1024,
            max_body_pairs: 1024,
            max_contact_constraints: 1024,
            // Zero requests Jolt's default per-body mutex pool sizing.
            num_body_mutexes: 0,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            num_velocity_steps: 10,
            num_position_steps: 2,
        }
    }
}

/// RAII-managed JoltPhysics world. Drops every JoltC handle in the
/// correct reverse order so leaks don't accumulate across test runs.
#[cfg(feature = "native")]
pub struct World {
    physics_system: *mut joltc_sys::JPC_PhysicsSystem,
    broad_phase_layer_interface: *mut joltc_sys::JPC_BroadPhaseLayerInterface,
    object_vs_broad_phase_layer_filter:
        *mut joltc_sys::JPC_ObjectVsBroadPhaseLayerFilter,
    object_vs_object_layer_filter: *mut joltc_sys::JPC_ObjectLayerPairFilter,
    job_system: *mut joltc_sys::JPC_JobSystemSingleThreaded,
    temp_allocator: *mut joltc_sys::JPC_TempAllocatorImpl,
    /// Leaked Box pointer; reclaimed in Drop AFTER listener delete.
    contact_state: *mut crate::contact::ContactState,
    contact_listener: *mut joltc_sys::JPC_ContactListener,
}

#[cfg(not(feature = "native"))]
pub struct World {
    _marker: std::marker::PhantomData<()>,
}

#[cfg(feature = "native")]
impl World {
    /// Construct a new world with the given config.
    ///
    /// Performs the one-time JoltC global init (alloc + factory +
    /// type registry) the first time it's called per process; later
    /// `World::new` calls reuse the registered types.
    pub fn new(config: WorldConfig) -> Self {
        use joltc_sys::*;
        global_init();
        unsafe {
            let temp_allocator = JPC_TempAllocatorImpl_new(10 * 1024 * 1024);
            let job_system =
                JPC_JobSystemSingleThreaded_new(JPC_MAX_PHYSICS_JOBS as u32);

            let broad_phase_layer_interface =
                JPC_BroadPhaseLayerInterface_new(ptr::null(), BPL_FNS);
            let object_vs_broad_phase_layer_filter =
                JPC_ObjectVsBroadPhaseLayerFilter_new(ptr::null_mut(), OVB_FNS);
            let object_vs_object_layer_filter =
                JPC_ObjectLayerPairFilter_new(ptr::null_mut(), OVO_FNS);

            let physics_system = JPC_PhysicsSystem_new();
            JPC_PhysicsSystem_Init(
                physics_system,
                config.max_bodies,
                config.num_body_mutexes,
                config.max_body_pairs,
                config.max_contact_constraints,
                broad_phase_layer_interface,
                object_vs_broad_phase_layer_filter,
                object_vs_object_layer_filter,
            );
            JPC_PhysicsSystem_SetGravity(
                physics_system,
                crate::math::to_jpc_vec3(config.gravity),
            );

            // Allocate the contact-event buffer on the heap; pass its
            // pointer as the listener `this` so OnContact callbacks
            // can push into it deterministically. The buffer lives
            // until World::Drop, where we delete the listener first
            // and then reclaim the Box.
            let contact_state = crate::contact::ContactState::new();
            let contact_listener = joltc_sys::JPC_ContactListener_new(
                contact_state as *mut std::ffi::c_void,
                crate::contact::CONTACT_LISTENER_FNS,
            );
            joltc_sys::JPC_PhysicsSystem_SetContactListener(
                physics_system,
                contact_listener,
            );

            // JoltC v0.3.1 does not expose JPC_PhysicsSystem_*PhysicsSettings;
            // the global `mNumVelocitySteps = 10`, `mNumPositionSteps = 2`,
            // and `mDeterministicSimulation = true` Jolt defaults stand. Per-body
            // overrides on JPC_BodyCreationSettings.NumVelocityStepsOverride /
            // .NumPositionStepsOverride are available if engine-plugin-physics
            // needs them. WorldConfig keeps the fields as a documentation
            // surface; wiring them through requires a JoltC patch (queued as
            // v0.23+ engine-jolt work). Reads from `config` here keep the
            // values un-warned:
            let _ = (
                config.num_velocity_steps,
                config.num_position_steps,
            );

            Self {
                physics_system,
                broad_phase_layer_interface,
                object_vs_broad_phase_layer_filter,
                object_vs_object_layer_filter,
                job_system,
                temp_allocator,
                contact_state,
                contact_listener,
            }
        }
    }

    /// The single entrypoint for advancing physics. Caller supplies
    /// `dt` (typically `1.0 / 64.0` from `Time<Fixed>`) and
    /// `collision_steps` (default 1; constant per the determinism
    /// contract, NEVER derived from `dt`).
    pub fn drive_step(&mut self, dt: f32, collision_steps: i32) {
        unsafe {
            joltc_sys::JPC_PhysicsSystem_Update(
                self.physics_system,
                dt,
                collision_steps,
                self.temp_allocator,
                self.job_system.cast::<joltc_sys::JPC_JobSystem>(),
            );
        }
    }

    /// Run broadphase optimization. Called once after the initial
    /// burst of static body spawns so the BVH is built deterministi-
    /// cally before stepping.
    pub fn optimize_broad_phase(&mut self) {
        unsafe {
            joltc_sys::JPC_PhysicsSystem_OptimizeBroadPhase(self.physics_system);
        }
    }

    /// Borrow a `BodyInterface` for spawning, removing, and inspecting
    /// bodies. The interface holds a shared reference to `self`; only
    /// one may exist at a time per Rust borrow rules.
    pub fn body_interface(&mut self) -> BodyInterface<'_> {
        // Safety: GetBodyInterface returns a long-lived pointer owned
        // by the PhysicsSystem; the interface is valid for the
        // lifetime of `self`.
        let raw = unsafe {
            joltc_sys::JPC_PhysicsSystem_GetBodyInterface(self.physics_system)
        };
        BodyInterface::from_raw(raw)
    }

    /// Borrow the `NarrowPhaseQuery` for raycast / shape-cast / collide
    /// queries. Read-only -- a `&self` borrow suffices because Jolt's
    /// narrow phase reads the broadphase tree without mutating it.
    pub fn narrow_phase(&self) -> crate::narrow_phase::NarrowPhaseQuery<'_> {
        let raw = unsafe {
            joltc_sys::JPC_PhysicsSystem_GetNarrowPhaseQuery(self.physics_system)
        };
        crate::narrow_phase::NarrowPhaseQuery::from_raw(raw)
    }

    /// Drain and clear the per-tick contact event buffer. Events are
    /// returned sorted by `(body_a, body_b, kind)` so iteration order
    /// is deterministic across runs. Call this after `drive_step` in
    /// the same tick to consume contacts; events not drained persist
    /// across ticks (the buffer accumulates).
    pub fn drain_contacts(&mut self) -> Vec<crate::contact::ContactEvent> {
        crate::contact::drain_sorted(self.contact_state)
    }
}

#[cfg(feature = "native")]
impl Drop for World {
    fn drop(&mut self) {
        unsafe {
            // Tear down in reverse construction order. The contact
            // listener must die BEFORE the physics system stops
            // referencing it; the ContactState Box must outlive the
            // listener so any in-flight callback completes against
            // valid memory.
            joltc_sys::JPC_PhysicsSystem_delete(self.physics_system);
            joltc_sys::JPC_ContactListener_delete(self.contact_listener);
            crate::contact::ContactState::destroy(self.contact_state);
            joltc_sys::JPC_BroadPhaseLayerInterface_delete(
                self.broad_phase_layer_interface,
            );
            joltc_sys::JPC_ObjectVsBroadPhaseLayerFilter_delete(
                self.object_vs_broad_phase_layer_filter,
            );
            joltc_sys::JPC_ObjectLayerPairFilter_delete(
                self.object_vs_object_layer_filter,
            );
            joltc_sys::JPC_JobSystemSingleThreaded_delete(self.job_system);
            joltc_sys::JPC_TempAllocatorImpl_delete(self.temp_allocator);
        }
    }
}

#[cfg(not(feature = "native"))]
impl World {
    pub fn new(_config: WorldConfig) -> Self {
        panic!(
            "engine-jolt: World::new called without the `native` feature -- \
             enable `native` (and ideally `cross_deterministic`) to link JoltPhysics."
        )
    }
}

// ---------------------------------------------------------------------
// Filter callbacks
//
// These are `extern "C"` functions over the canonical 2-layer scheme.
// Stateless by construction: no captures, no Rust storage, no atomic
// ordering. The whole filter interface is layer-id math on
// `crate::layers` constants.
// ---------------------------------------------------------------------

#[cfg(feature = "native")]
unsafe extern "C" fn bpl_get_num(_this: *const std::ffi::c_void) -> std::ffi::c_uint {
    BroadPhaseLayer::COUNT
}

#[cfg(feature = "native")]
unsafe extern "C" fn bpl_get(
    _this: *const std::ffi::c_void,
    layer: joltc_sys::JPC_ObjectLayer,
) -> joltc_sys::JPC_BroadPhaseLayer {
    if layer == crate::layers::ObjectLayer::STATIC.0 as joltc_sys::JPC_ObjectLayer {
        BroadPhaseLayer::STATIC.into()
    } else {
        BroadPhaseLayer::MOVING.into()
    }
}

#[cfg(feature = "native")]
const BPL_FNS: joltc_sys::JPC_BroadPhaseLayerInterfaceFns =
    joltc_sys::JPC_BroadPhaseLayerInterfaceFns {
        GetNumBroadPhaseLayers: Some(bpl_get_num),
        GetBroadPhaseLayer: Some(bpl_get),
    };

#[cfg(feature = "native")]
unsafe extern "C" fn ovb_should_collide(
    _this: *const std::ffi::c_void,
    obj: joltc_sys::JPC_ObjectLayer,
    bp: joltc_sys::JPC_BroadPhaseLayer,
) -> bool {
    let static_bp: joltc_sys::JPC_BroadPhaseLayer = BroadPhaseLayer::STATIC.into();
    let moving_bp: joltc_sys::JPC_BroadPhaseLayer = BroadPhaseLayer::MOVING.into();
    let static_ol: joltc_sys::JPC_ObjectLayer =
        crate::layers::ObjectLayer::STATIC.0 as joltc_sys::JPC_ObjectLayer;
    let _ = (static_bp, moving_bp, static_ol);
    if obj == static_ol {
        // Static colliders only need to test against moving objects.
        bp == BroadPhaseLayer::MOVING.into()
    } else {
        // Moving objects collide against both broadphase layers.
        true
    }
}

#[cfg(feature = "native")]
const OVB_FNS: joltc_sys::JPC_ObjectVsBroadPhaseLayerFilterFns =
    joltc_sys::JPC_ObjectVsBroadPhaseLayerFilterFns {
        ShouldCollide: Some(ovb_should_collide),
    };

#[cfg(feature = "native")]
unsafe extern "C" fn ovo_should_collide(
    _this: *const std::ffi::c_void,
    a: joltc_sys::JPC_ObjectLayer,
    b: joltc_sys::JPC_ObjectLayer,
) -> bool {
    let static_ol: joltc_sys::JPC_ObjectLayer =
        crate::layers::ObjectLayer::STATIC.0 as joltc_sys::JPC_ObjectLayer;
    if a == static_ol {
        // Static-vs-static skipped; static-vs-moving collides.
        b != static_ol
    } else {
        // Moving collides with anything.
        true
    }
}

#[cfg(feature = "native")]
const OVO_FNS: joltc_sys::JPC_ObjectLayerPairFilterFns =
    joltc_sys::JPC_ObjectLayerPairFilterFns {
        ShouldCollide: Some(ovo_should_collide),
    };

// ---------------------------------------------------------------------
// One-shot global init
// ---------------------------------------------------------------------

#[cfg(feature = "native")]
fn global_init() {
    use std::sync::OnceLock;
    static INIT: OnceLock<()> = OnceLock::new();
    INIT.get_or_init(|| unsafe {
        joltc_sys::JPC_RegisterDefaultAllocator();
        joltc_sys::JPC_FactoryInit();
        joltc_sys::JPC_RegisterTypes();
    });
}
