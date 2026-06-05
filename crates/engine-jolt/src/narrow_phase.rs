//! Narrow-phase queries (raycast, shape-cast, collide-shape).
//!
//! v0.22 ships `cast_ray` only -- the surface the engine-plugin-physics
//! v0.22 `physics.raycast` XGEP method consumes. `cast_shape` and
//! `collide_shape` defer to v0.23.
//!
//! JoltC's `JPC_RayCastResult` returns BodyId + Fraction + SubShapeID
//! only. The hit point is computed locally as `origin + fraction *
//! direction`. The surface normal is NOT in v0.22 -- it requires a
//! follow-up call to fetch the body's shape and call its sub-shape
//! normal function; queued as v0.23 if Voxelith's surface needs it.

use glam::Vec3;

use crate::body::BodyId;

#[cfg(feature = "native")]
pub struct NarrowPhaseQuery<'world> {
    raw: *const joltc_sys::JPC_NarrowPhaseQuery,
    _marker: std::marker::PhantomData<&'world crate::world::World>,
}

#[cfg(not(feature = "native"))]
pub struct NarrowPhaseQuery<'world> {
    _marker: std::marker::PhantomData<&'world crate::world::World>,
}

/// Result of a successful raycast. Hit body id, distance from ray
/// origin, and computed hit point. Surface normal is not in v0.22.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    pub body: BodyId,
    pub distance: f32,
    pub point: Vec3,
}

#[cfg(feature = "native")]
impl<'world> NarrowPhaseQuery<'world> {
    pub(crate) fn from_raw(raw: *const joltc_sys::JPC_NarrowPhaseQuery) -> Self {
        Self {
            raw,
            _marker: std::marker::PhantomData,
        }
    }

    /// Cast a ray of length `max_distance` from `origin` in direction
    /// `direction` (auto-normalized internally so callers don't need
    /// to). Returns the closest hit, if any. Filters are passed as
    /// null -- v0.22 ships unfiltered raycasts; layer-mask filtering
    /// surfaces in v0.23 alongside `physics.query_bodies`.
    pub fn cast_ray(
        &self,
        origin: Vec3,
        direction: Vec3,
        max_distance: f32,
    ) -> Option<RayHit> {
        use joltc_sys::*;
        let dir = direction.normalize_or_zero();
        if dir == Vec3::ZERO {
            return None;
        }
        let ray = JPC_RRayCast {
            Origin: crate::math::to_jpc_rvec3(origin),
            Direction: crate::math::to_jpc_vec3(dir * max_distance),
        };
        let mut args = JPC_NarrowPhaseQuery_CastRayArgs {
            Ray: ray,
            Result: JPC_RayCastResult {
                BodyID: 0,
                Fraction: 1.001, // Sentinel: Jolt overwrites only if hit found
                SubShapeID2: 0,
            },
            BroadPhaseLayerFilter: std::ptr::null(),
            ObjectLayerFilter: std::ptr::null(),
            BodyFilter: std::ptr::null(),
            ShapeFilter: std::ptr::null(),
        };
        let hit = unsafe { JPC_NarrowPhaseQuery_CastRay(self.raw, &mut args) };
        if !hit {
            return None;
        }
        let fraction = args.Result.Fraction;
        let distance = fraction * max_distance;
        let point = origin + dir * distance;
        Some(RayHit {
            body: BodyId(args.Result.BodyID),
            distance,
            point,
        })
    }
}

#[cfg(not(feature = "native"))]
impl<'world> NarrowPhaseQuery<'world> {
    pub fn cast_ray(
        &self,
        _origin: Vec3,
        _direction: Vec3,
        _max_distance: f32,
    ) -> Option<RayHit> {
        panic!("engine-jolt: NarrowPhaseQuery without the `native` feature")
    }
}
