//! Deterministic contact event buffering.
//!
//! Jolt's raw `ContactListener::OnContactAdded` fires from worker
//! threads in unpredictable order; even on a single-threaded job
//! system the firing order within a step depends on broadphase
//! ordering. To deliver bit-identical events across runs the
//! engine-jolt bridge:
//!
//!   1. Owns a `ContactState` heap-allocated per `World` (the listener
//!      `this`-pointer in JoltC's ABI).
//!   2. Inside the C ABI callbacks, BUFFERS `ContactEvent` into a Vec
//!      via a `Mutex<Vec<ContactEvent>>` (Mutex chosen over RefCell so
//!      a future multi-threaded JobSystem still works -- the lock is
//!      uncontended in single-threaded mode).
//!   3. Exposes `World::drain_contacts()` which sorts the buffer by
//!      `(body_a, body_b, kind)` before returning, GUARANTEEING the
//!      Bevy plugin sees deterministic event order.
//!
//! Listener-bridge pattern (paste! + extern-C boxing) is the same
//! design rolt uses; lifted under dual MIT/Apache-2.0 attribution.
//! engine-jolt owns the public surface (sorted ContactSink, BodyId
//! safe types) so determinism-first contracts are encoded here.

use std::sync::Mutex;

use crate::body::BodyId;

/// What kind of contact event this is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ContactKind {
    /// A new contact appeared between the two bodies this step.
    Added,
    /// A contact present in a previous step persisted this step.
    /// Off by default in the v0.22 sink (added/ended only). Voxelith
    /// would opt in via the Bevy plugin policy if it needs per-step
    /// contact callbacks (rare).
    Persisted,
    /// A previously-present contact ended this step.
    Removed,
}

/// One sorted, deterministic contact event delivered to the Bevy
/// plugin. The pair is canonicalized so `body_a < body_b` always.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ContactEvent {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub kind: ContactKind,
}

/// Per-World contact state. Held inside a Box, the raw pointer
/// passed to JoltC as the listener `this`. Cleaned up in
/// `World::Drop` via Box::from_raw.
pub(crate) struct ContactState {
    pub(crate) buffer: Mutex<Vec<ContactEvent>>,
}

impl ContactState {
    pub(crate) fn new() -> *mut ContactState {
        Box::into_raw(Box::new(ContactState {
            buffer: Mutex::new(Vec::with_capacity(256)),
        }))
    }

    /// Reclaim the box; called from World::Drop AFTER the listener
    /// has been unregistered.
    pub(crate) unsafe fn destroy(ptr: *mut ContactState) {
        if !ptr.is_null() {
            drop(Box::from_raw(ptr));
        }
    }
}

/// Sort + drain the buffer. Returns events in `(body_a, body_b, kind)`
/// order regardless of how Jolt fired the underlying callbacks.
#[cfg(feature = "native")]
pub(crate) fn drain_sorted(state: *mut ContactState) -> Vec<ContactEvent> {
    if state.is_null() {
        return Vec::new();
    }
    // Safety: state was allocated by ContactState::new() and is alive
    // for the lifetime of the owning World (destroyed in Drop).
    let state_ref = unsafe { &*state };
    let mut events = match state_ref.buffer.lock() {
        Ok(mut g) => std::mem::take(&mut *g),
        Err(p) => {
            // Lock poisoning means a previous OnContact callback
            // panicked. Recover the inner data and clear.
            let mut g = p.into_inner();
            std::mem::take(&mut *g)
        }
    };
    events.sort();
    events
}

#[cfg(not(feature = "native"))]
pub(crate) fn drain_sorted(_state: *mut ContactState) -> Vec<ContactEvent> {
    Vec::new()
}

// ---------------------------------------------------------------------
// JoltC extern "C" callback bridge
//
// JPC_ContactListenerFns has 4 callbacks:
//   - OnContactValidate (called BEFORE the collision -- return true to
//     allow). We always return true (no filtering at engine-jolt; the
//     Bevy plugin filters via CollisionEventsEnabled at emit time).
//   - OnContactAdded
//   - OnContactPersisted
//   - OnContactRemoved
//
// The `this` pointer is the *mut ContactState we registered. We
// downcast it back, lock the buffer, and push a ContactEvent.
// ---------------------------------------------------------------------

#[cfg(feature = "native")]
unsafe fn push_event(
    this: *mut std::ffi::c_void,
    body_a: joltc_sys::JPC_BodyID,
    body_b: joltc_sys::JPC_BodyID,
    kind: ContactKind,
) {
    if this.is_null() {
        return;
    }
    let state = this as *const ContactState;
    let state_ref = &*state;
    let (a, b) = if body_a <= body_b {
        (BodyId(body_a), BodyId(body_b))
    } else {
        (BodyId(body_b), BodyId(body_a))
    };
    let event = ContactEvent {
        body_a: a,
        body_b: b,
        kind,
    };
    if let Ok(mut g) = state_ref.buffer.lock() {
        g.push(event);
    }
    // Lock poisoning is silently dropped: a panicking listener is
    // unrecoverable in unsafe land. drain_sorted() handles poisoning
    // on the next call.
}

#[cfg(feature = "native")]
unsafe extern "C" fn on_contact_validate(
    _this: *mut std::ffi::c_void,
    _body_a: *const joltc_sys::JPC_Body,
    _body_b: *const joltc_sys::JPC_Body,
    _base_offset: joltc_sys::JPC_RVec3,
    _collision_result: *const joltc_sys::JPC_CollideShapeResult,
) -> joltc_sys::JPC_ValidateResult {
    joltc_sys::JPC_VALIDATE_RESULT_ACCEPT_ALL_CONTACTS
}

#[cfg(feature = "native")]
unsafe extern "C" fn on_contact_added(
    this: *mut std::ffi::c_void,
    body_a: *const joltc_sys::JPC_Body,
    body_b: *const joltc_sys::JPC_Body,
    _manifold: *const joltc_sys::JPC_ContactManifold,
    _settings: *mut joltc_sys::JPC_ContactSettings,
) {
    let a = joltc_sys::JPC_Body_GetID(body_a);
    let b = joltc_sys::JPC_Body_GetID(body_b);
    push_event(this, a, b, ContactKind::Added);
}

#[cfg(feature = "native")]
unsafe extern "C" fn on_contact_persisted(
    this: *mut std::ffi::c_void,
    body_a: *const joltc_sys::JPC_Body,
    body_b: *const joltc_sys::JPC_Body,
    _manifold: *const joltc_sys::JPC_ContactManifold,
    _settings: *mut joltc_sys::JPC_ContactSettings,
) {
    let a = joltc_sys::JPC_Body_GetID(body_a);
    let b = joltc_sys::JPC_Body_GetID(body_b);
    push_event(this, a, b, ContactKind::Persisted);
}

#[cfg(feature = "native")]
unsafe extern "C" fn on_contact_removed(
    this: *mut std::ffi::c_void,
    sub_shape_pair: *const joltc_sys::JPC_SubShapeIDPair,
) {
    // SubShapeIDPair carries the body ids directly. Read them out.
    if sub_shape_pair.is_null() {
        return;
    }
    let pair = &*sub_shape_pair;
    push_event(this, pair.Body1ID, pair.Body2ID, ContactKind::Removed);
}

#[cfg(feature = "native")]
pub(crate) const CONTACT_LISTENER_FNS: joltc_sys::JPC_ContactListenerFns =
    joltc_sys::JPC_ContactListenerFns {
        OnContactValidate: Some(on_contact_validate),
        OnContactAdded: Some(on_contact_added),
        OnContactPersisted: Some(on_contact_persisted),
        OnContactRemoved: Some(on_contact_removed),
    };
