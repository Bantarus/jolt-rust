//! glam <-> JoltC math conversions.
//!
//! engine-jolt's public API speaks in `glam::Vec3` / `glam::Quat`
//! (matching Bevy 0.18's math types so engine-plugin-physics has a
//! zero-conversion path in `sync_in` / `sync_out`). The conversions
//! here are inlinable and never panic.
//!
//! The `Real` type alias matches joltc-sys (f32 by default, f64 with
//! the `double-precision` feature). Most engine-jolt users stay in
//! f32 land; the conversions happily lossy-cast at the boundary.

#[cfg(feature = "native")]
use glam::{Quat, Vec3};

#[cfg(feature = "double_precision")]
pub type Real = f64;
#[cfg(not(feature = "double_precision"))]
pub type Real = f32;

/// Convert a glam `Vec3` into a JoltC `JPC_Vec3`. The fourth lane is
/// duplicated from `z` to match the JPC convention (JPC_Vec3 stores
/// `_w` as padding that is sometimes read).
#[cfg(feature = "native")]
#[inline]
pub(crate) fn to_jpc_vec3(v: Vec3) -> joltc_sys::JPC_Vec3 {
    joltc_sys::JPC_Vec3 {
        x: v.x,
        y: v.y,
        z: v.z,
        _w: v.z,
    }
}

/// Convert a JoltC `JPC_Vec3` back into glam `Vec3`.
#[cfg(feature = "native")]
#[inline]
pub(crate) fn from_jpc_vec3(v: joltc_sys::JPC_Vec3) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

/// Convert a glam `Vec3` into a JoltC `JPC_RVec3` (which uses `Real`
/// per the joltc-sys feature flags). Casts through f32/f64 as needed.
#[cfg(feature = "native")]
#[inline]
pub(crate) fn to_jpc_rvec3(v: Vec3) -> joltc_sys::JPC_RVec3 {
    let x = v.x as Real;
    let y = v.y as Real;
    let z = v.z as Real;
    joltc_sys::JPC_RVec3 { x, y, z, _w: z }
}

/// Convert a JoltC `JPC_RVec3` back into glam `Vec3` (lossy when the
/// `double-precision` feature is on -- engine-jolt's public API is
/// f32 unless v0.23+ ships a typed f64 surface).
#[cfg(feature = "native")]
#[inline]
pub(crate) fn from_jpc_rvec3(v: joltc_sys::JPC_RVec3) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

/// Convert a glam `Quat` into a JoltC `JPC_Quat`. JoltC quaternions
/// match glam's convention (x, y, z, w).
#[cfg(feature = "native")]
#[inline]
pub(crate) fn to_jpc_quat(q: Quat) -> joltc_sys::JPC_Quat {
    let [x, y, z, w] = q.to_array();
    joltc_sys::JPC_Quat { x, y, z, w }
}

/// Convert a JoltC `JPC_Quat` back into glam `Quat`.
#[cfg(feature = "native")]
#[inline]
pub(crate) fn from_jpc_quat(q: joltc_sys::JPC_Quat) -> Quat {
    Quat::from_xyzw(q.x, q.y, q.z, q.w)
}
