//! `BodyInterface` -- borrowed handle for spawning, removing, and
//! inspecting bodies inside a `World`.
//!
//! The interface mirrors JoltC's `JPC_BodyInterface` API but
//! consumes/produces safe types: `BodyDef`, `BodyId`, glam vectors.
//! All operations are O(1); the underlying JoltC BodyManager is the
//! source of truth for body storage and lifecycle.
//!
//! Add/Remove semantics: `spawn` inserts the body into the broadphase
//! (active if Dynamic/Kinematic, inactive if Static). `remove`
//! takes the body OUT of the broadphase but keeps the body data
//! around. `destroy` removes-and-frees in one shot. Step 2.0 ships
//! `spawn` + `destroy` only; explicit remove-keep is v0.23+ if a
//! consumer surfaces a need.

use glam::Vec3;

use crate::body::{BodyDef, BodyId};

#[cfg(feature = "native")]
pub struct BodyInterface<'world> {
    raw: *mut joltc_sys::JPC_BodyInterface,
    _marker: std::marker::PhantomData<&'world mut crate::world::World>,
}

#[cfg(not(feature = "native"))]
pub struct BodyInterface<'world> {
    _marker: std::marker::PhantomData<&'world mut crate::world::World>,
}

#[cfg(feature = "native")]
impl<'world> BodyInterface<'world> {
    pub(crate) fn from_raw(raw: *mut joltc_sys::JPC_BodyInterface) -> Self {
        Self {
            raw,
            _marker: std::marker::PhantomData,
        }
    }

    /// Spawn a body from the given definition. The body's shape is
    /// added to the broadphase; the returned `BodyId` is stable for
    /// the lifetime of the body.
    pub fn spawn(&mut self, def: &BodyDef) -> BodyId {
        use joltc_sys::*;
        unsafe {
            let settings = JPC_BodyCreationSettings {
                Position: crate::math::to_jpc_rvec3(def.position),
                Rotation: crate::math::to_jpc_quat(def.rotation),
                LinearVelocity: crate::math::to_jpc_vec3(def.linear_velocity),
                AngularVelocity: crate::math::to_jpc_vec3(def.angular_velocity),
                ObjectLayer: def.object_layer.into(),
                MotionType: def.body_type.to_motion_type(),
                Shape: def.shape.ptr as *const _,
                Friction: def.friction,
                Restitution: def.restitution,
                ..Default::default()
            };
            let body = JPC_BodyInterface_CreateBody(self.raw, &settings);
            let id = JPC_Body_GetID(body);
            JPC_BodyInterface_AddBody(self.raw, id, def.body_type.activation());
            BodyId::from(id)
        }
    }

    /// Remove the body from the broadphase AND free its storage. The
    /// `BodyId` becomes invalid after this call.
    pub fn destroy(&mut self, id: BodyId) {
        unsafe {
            joltc_sys::JPC_BodyInterface_RemoveBody(self.raw, id.into());
            joltc_sys::JPC_BodyInterface_DestroyBody(self.raw, id.into());
        }
    }

    /// Read the body's world-space position.
    pub fn position(&self, id: BodyId) -> Vec3 {
        let pos = unsafe {
            joltc_sys::JPC_BodyInterface_GetPosition(
                self.raw as *const joltc_sys::JPC_BodyInterface,
                id.into(),
            )
        };
        crate::math::from_jpc_rvec3(pos)
    }

    /// Read the body's world-space rotation.
    pub fn rotation(&self, id: BodyId) -> glam::Quat {
        let rot = unsafe {
            joltc_sys::JPC_BodyInterface_GetRotation(
                self.raw as *const joltc_sys::JPC_BodyInterface,
                id.into(),
            )
        };
        crate::math::from_jpc_quat(rot)
    }

    /// Read the body's linear velocity.
    pub fn linear_velocity(&self, id: BodyId) -> Vec3 {
        let v = unsafe {
            joltc_sys::JPC_BodyInterface_GetLinearVelocity(
                self.raw as *const joltc_sys::JPC_BodyInterface,
                id.into(),
            )
        };
        crate::math::from_jpc_vec3(v)
    }

    /// True while the body is awake (in a simulating island). Static
    /// bodies and slept dynamic bodies report `false`. This is the one
    /// primitive the Bevy plugin's `Sleeping` marker keys off; Jolt's
    /// island/sleep heuristic is deterministic under the single-threaded
    /// `cross_deterministic` job system, so the awake->asleep transition
    /// tick is bit-identical across runs and platforms.
    pub fn is_active(&self, id: BodyId) -> bool {
        unsafe {
            joltc_sys::JPC_BodyInterface_IsActive(
                self.raw as *const joltc_sys::JPC_BodyInterface,
                id.into(),
            )
        }
    }

    /// Set the body's linear velocity.
    pub fn set_linear_velocity(&mut self, id: BodyId, v: Vec3) {
        unsafe {
            joltc_sys::JPC_BodyInterface_SetLinearVelocity(
                self.raw,
                id.into(),
                crate::math::to_jpc_vec3(v),
            );
        }
    }

    /// Wake a sleeping body -- re-insert it into a simulating island.
    /// Jolt does NOT auto-wake a slept body when the static geometry
    /// beneath it is removed (e.g. terrain dug out from under settled
    /// debris); call this to make it fall. No-op on an already-active
    /// body. Deterministic under the single-threaded job system.
    pub fn activate(&mut self, id: BodyId) {
        unsafe {
            joltc_sys::JPC_BodyInterface_ActivateBody(self.raw, id.into());
        }
    }
}

#[cfg(not(feature = "native"))]
impl<'world> BodyInterface<'world> {
    pub fn spawn(&mut self, _def: &BodyDef) -> BodyId {
        panic!("engine-jolt: BodyInterface called without the `native` feature")
    }
    pub fn destroy(&mut self, _id: BodyId) {
        panic!("engine-jolt: BodyInterface called without the `native` feature")
    }
    pub fn position(&self, _id: BodyId) -> Vec3 {
        panic!("engine-jolt: BodyInterface called without the `native` feature")
    }
    pub fn rotation(&self, _id: BodyId) -> glam::Quat {
        panic!("engine-jolt: BodyInterface called without the `native` feature")
    }
    pub fn linear_velocity(&self, _id: BodyId) -> Vec3 {
        panic!("engine-jolt: BodyInterface called without the `native` feature")
    }
    pub fn is_active(&self, _id: BodyId) -> bool {
        panic!("engine-jolt: BodyInterface called without the `native` feature")
    }
    pub fn set_linear_velocity(&mut self, _id: BodyId, _v: Vec3) {
        panic!("engine-jolt: BodyInterface called without the `native` feature")
    }
    pub fn activate(&mut self, _id: BodyId) {
        panic!("engine-jolt: BodyInterface called without the `native` feature")
    }
}
