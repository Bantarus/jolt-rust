//! Body identifiers, types, and creation descriptors.
//!
//! `BodyId` is a transparent newtype around JoltC's `JPC_BodyID` (a
//! plain u32). It's `Copy` + `Ord` so engine-plugin-physics can sort
//! its `BodyIdMap` deterministically.
//!
//! `BodyDef` is the value-typed body creation descriptor; the
//! `BodyInterface::spawn` method consumes it and returns a `BodyId`.

use glam::{Quat, Vec3};

use crate::layers::ObjectLayer;
use crate::shape::ShapeHandle;

/// Body identifier. Stable for the lifetime of the body inside the
/// owning `World`. Internally a u32 (the JoltC representation).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BodyId(pub u32);

#[cfg(feature = "native")]
impl From<joltc_sys::JPC_BodyID> for BodyId {
    fn from(id: joltc_sys::JPC_BodyID) -> Self {
        BodyId(id as u32)
    }
}

#[cfg(feature = "native")]
impl From<BodyId> for joltc_sys::JPC_BodyID {
    fn from(id: BodyId) -> Self {
        id.0 as joltc_sys::JPC_BodyID
    }
}

/// Motion type. Maps to the JoltC `JPC_MOTION_TYPE_*` constants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyType {
    /// Never moves; participates in collision as an immovable obstacle.
    Static,
    /// Moves under physics (gravity, contacts, impulses).
    Dynamic,
    /// Moves via explicit `set_transform` / `set_velocity` calls;
    /// participates in collision but is not affected by forces.
    Kinematic,
}

#[cfg(feature = "native")]
impl BodyType {
    pub(crate) fn to_motion_type(self) -> joltc_sys::JPC_MotionType {
        match self {
            BodyType::Static => joltc_sys::JPC_MOTION_TYPE_STATIC,
            BodyType::Dynamic => joltc_sys::JPC_MOTION_TYPE_DYNAMIC,
            BodyType::Kinematic => joltc_sys::JPC_MOTION_TYPE_KINEMATIC,
        }
    }

    pub(crate) fn activation(self) -> joltc_sys::JPC_Activation {
        match self {
            BodyType::Static => joltc_sys::JPC_ACTIVATION_DONT_ACTIVATE,
            BodyType::Dynamic | BodyType::Kinematic => {
                joltc_sys::JPC_ACTIVATION_ACTIVATE
            }
        }
    }
}

/// Body creation descriptor consumed by `BodyInterface::spawn`. The
/// `shape` is cloned (refcount bumped) when the body is created, so
/// the caller retains independent ownership of the handle.
#[derive(Debug, Clone)]
pub struct BodyDef {
    pub shape: ShapeHandle,
    pub position: Vec3,
    pub rotation: Quat,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub body_type: BodyType,
    pub object_layer: ObjectLayer,
    /// Friction (Coulomb coefficient). Default 0.2.
    pub friction: f32,
    /// Restitution coefficient. Default 0.0.
    pub restitution: f32,
}

impl BodyDef {
    /// Convenience builder for a default dynamic body at the given
    /// position. Caller fills in remaining fields with builder-style
    /// chained assignments.
    pub fn dynamic(shape: ShapeHandle, position: Vec3) -> Self {
        Self {
            shape,
            position,
            rotation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            body_type: BodyType::Dynamic,
            object_layer: ObjectLayer::MOVING,
            friction: 0.2,
            restitution: 0.0,
        }
    }

    /// Convenience builder for a static body at the given position.
    pub fn static_body(shape: ShapeHandle, position: Vec3) -> Self {
        Self {
            shape,
            position,
            rotation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            body_type: BodyType::Static,
            object_layer: ObjectLayer::STATIC,
            friction: 0.2,
            restitution: 0.0,
        }
    }
}
