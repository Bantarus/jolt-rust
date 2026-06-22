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

    /// Read the body's angular velocity (rad/s, world space). The
    /// companion to `linear_velocity`; together they are the complete
    /// velocity state a predicted-physics rollback captures + restores
    /// (v0.38). Closes the readback gap the Bevy plugin's `sync_out`
    /// flagged ("angular velocity getter queued for engine-jolt").
    pub fn angular_velocity(&self, id: BodyId) -> Vec3 {
        let v = unsafe {
            joltc_sys::JPC_BodyInterface_GetAngularVelocity(
                self.raw as *const joltc_sys::JPC_BodyInterface,
                id.into(),
            )
        };
        crate::math::from_jpc_vec3(v)
    }

    /// Set the body's angular velocity (rad/s, world space).
    pub fn set_angular_velocity(&mut self, id: BodyId, v: Vec3) {
        unsafe {
            joltc_sys::JPC_BodyInterface_SetAngularVelocity(
                self.raw,
                id.into(),
                crate::math::to_jpc_vec3(v),
            );
        }
    }

    /// Teleport the body to a world-space pose, activating it. Pairs
    /// with `set_pose_and_velocity` when velocity must also be restored.
    pub fn set_position_and_rotation(
        &mut self,
        id: BodyId,
        position: Vec3,
        rotation: glam::Quat,
    ) {
        unsafe {
            joltc_sys::JPC_BodyInterface_SetPositionAndRotation(
                self.raw,
                id.into(),
                crate::math::to_jpc_rvec3(position),
                crate::math::to_jpc_quat(rotation),
                joltc_sys::JPC_ACTIVATION_ACTIVATE,
            );
        }
    }

    /// Atomically restore a body's full kinematic state -- pose AND
    /// linear+angular velocity -- in a single FFI call, IN PLACE (the
    /// body stays alive; NOT destroy/respawn). This is the
    /// predicted-physics B2 restore primitive (v0.38): on a
    /// misprediction the rollback pushes the captured authoritative
    /// `(Transform, Velocity)` back into the live Jolt body, then
    /// replays the unacked inputs.
    ///
    /// B2 fidelity ceiling (load-bearing): this restores the PERSISTENT
    /// rigid-body motion state only. Jolt's per-step solver caches
    /// (contact warm-start lambdas, manifolds, island assignment,
    /// sleep-timer phase) are NOT captured, so a resting/stacked-contact
    /// body diverges on the resim frame. Bit-exact resume awaits the B1
    /// `JPC_StateRecorder` binding; v0.38 scopes prediction to
    /// free-flight / shallow-contact bodies.
    pub fn set_pose_and_velocity(
        &mut self,
        id: BodyId,
        position: Vec3,
        rotation: glam::Quat,
        linear: Vec3,
        angular: Vec3,
    ) {
        unsafe {
            joltc_sys::JPC_BodyInterface_SetPositionRotationAndVelocity(
                self.raw,
                id.into(),
                crate::math::to_jpc_rvec3(position),
                crate::math::to_jpc_quat(rotation),
                crate::math::to_jpc_vec3(linear),
                crate::math::to_jpc_vec3(angular),
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
    pub fn angular_velocity(&self, _id: BodyId) -> Vec3 {
        panic!("engine-jolt: BodyInterface called without the `native` feature")
    }
    pub fn set_angular_velocity(&mut self, _id: BodyId, _v: Vec3) {
        panic!("engine-jolt: BodyInterface called without the `native` feature")
    }
    pub fn set_position_and_rotation(&mut self, _id: BodyId, _position: Vec3, _rotation: glam::Quat) {
        panic!("engine-jolt: BodyInterface called without the `native` feature")
    }
    pub fn set_pose_and_velocity(
        &mut self,
        _id: BodyId,
        _position: Vec3,
        _rotation: glam::Quat,
        _linear: Vec3,
        _angular: Vec3,
    ) {
        panic!("engine-jolt: BodyInterface called without the `native` feature")
    }
    pub fn activate(&mut self, _id: BodyId) {
        panic!("engine-jolt: BodyInterface called without the `native` feature")
    }
}
