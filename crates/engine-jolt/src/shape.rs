//! Safe shape construction.
//!
//! `ShapeDef` is the value-typed description of a collider; calling
//! `build()` constructs the underlying `JPC_Shape*` and returns a
//! `ShapeHandle` RAII wrapper. v0.22 ships `Box`, `Sphere`, `Capsule`
//! per the design doc (Step 2.0 minimum); `ConvexHull`, `Mesh`, and
//! `Compound` follow in Step 2.1.
//!
//! Shape handles are reference-counted at the JoltC layer (Jolt's
//! `RefTarget<Shape>`); the engine-jolt `ShapeHandle` increments the
//! refcount on `Clone` and decrements on `Drop`. Bodies hold their
//! own ref independently, so dropping the handle after spawning a
//! body is safe and idiomatic.

use glam::Vec3;

use crate::error::JoltError;
#[cfg(feature = "native")]
use crate::error::build_error;
#[cfg(feature = "native")]
use std::ptr;

/// Value-typed shape description. The `build()` method materializes
/// the underlying JoltC `*mut JPC_Shape` and returns a refcounted
/// handle.
#[derive(Debug, Clone, PartialEq)]
pub enum ShapeDef {
    Box { half_extents: Vec3 },
    Sphere { radius: f32 },
    Capsule { half_height: f32, radius: f32 },
    // ConvexHull(Vec<Vec3>), Mesh(...), Compound(...) defer to Step 2.1.
}

impl ShapeDef {
    /// Build the underlying JoltC shape. Requires the `native`
    /// feature; without it the call panics with a clear message
    /// (engine-jolt is a stub crate without the C++ runtime).
    #[cfg(feature = "native")]
    pub fn build(&self) -> Result<ShapeHandle, JoltError> {
        use joltc_sys::*;
        unsafe {
            let mut shape: *mut JPC_Shape = ptr::null_mut();
            let mut err: *mut JPC_String = ptr::null_mut();
            let kind: &'static str;
            let ok = match *self {
                ShapeDef::Box { half_extents } => {
                    kind = "Box";
                    let settings = JPC_BoxShapeSettings {
                        HalfExtent: crate::math::to_jpc_vec3(half_extents),
                        ..Default::default()
                    };
                    JPC_BoxShapeSettings_Create(&settings, &mut shape, &mut err)
                }
                ShapeDef::Sphere { radius } => {
                    kind = "Sphere";
                    let settings = JPC_SphereShapeSettings {
                        Radius: radius,
                        ..Default::default()
                    };
                    JPC_SphereShapeSettings_Create(&settings, &mut shape, &mut err)
                }
                ShapeDef::Capsule { half_height, radius } => {
                    kind = "Capsule";
                    let settings = JPC_CapsuleShapeSettings {
                        HalfHeightOfCylinder: half_height,
                        Radius: radius,
                        ..Default::default()
                    };
                    JPC_CapsuleShapeSettings_Create(&settings, &mut shape, &mut err)
                }
            };
            if ok {
                Ok(ShapeHandle { ptr: shape })
            } else {
                Err(build_error(kind, err))
            }
        }
    }

    #[cfg(not(feature = "native"))]
    pub fn build(&self) -> Result<ShapeHandle, JoltError> {
        panic!(
            "engine-jolt: ShapeDef::build called without the `native` feature -- \
             enable `native` (and ideally `cross_deterministic`) to link JoltPhysics."
        )
    }
}

/// RAII wrapper around a `*mut JPC_Shape`. The shape is reference-
/// counted at the JoltC layer; clone increments, drop decrements. A
/// freshly-built shape has refcount 1.
#[cfg(feature = "native")]
pub struct ShapeHandle {
    pub(crate) ptr: *mut joltc_sys::JPC_Shape,
}

#[cfg(not(feature = "native"))]
#[derive(Clone)]
pub struct ShapeHandle {
    _marker: std::marker::PhantomData<()>,
}

#[cfg(feature = "native")]
impl std::fmt::Debug for ShapeHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ShapeHandle({:p})", self.ptr)
    }
}

#[cfg(not(feature = "native"))]
impl std::fmt::Debug for ShapeHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ShapeHandle(<no-native>)")
    }
}

#[cfg(feature = "native")]
impl Clone for ShapeHandle {
    fn clone(&self) -> Self {
        unsafe {
            joltc_sys::JPC_Shape_AddRef(self.ptr as *const joltc_sys::JPC_Shape);
        }
        Self { ptr: self.ptr }
    }
}

#[cfg(feature = "native")]
impl Drop for ShapeHandle {
    fn drop(&mut self) {
        unsafe {
            joltc_sys::JPC_Shape_Release(self.ptr as *const joltc_sys::JPC_Shape);
        }
    }
}
