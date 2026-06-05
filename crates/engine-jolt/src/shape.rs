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

use glam::{Quat, Vec3};

use crate::error::JoltError;
#[cfg(feature = "native")]
use crate::error::build_error;
#[cfg(feature = "native")]
use std::ptr;

/// Value-typed shape description. The `build()` method materializes
/// the underlying JoltC `*mut JPC_Shape` and returns a refcounted
/// handle.
///
/// `Compound` recursively contains its sub-shapes by value -- `build()`
/// constructs each sub-shape, then assembles the static-compound shape
/// from the resulting handles. Failure in any sub-shape propagates as
/// a `JoltError::Build` whose `kind` identifies the offending variant.
#[derive(Debug, Clone, PartialEq)]
pub enum ShapeDef {
    Box {
        half_extents: Vec3,
    },
    Sphere {
        radius: f32,
    },
    Capsule {
        half_height: f32,
        radius: f32,
    },
    /// Convex hull from a point cloud. Jolt computes the actual hull
    /// (Quickhull); degenerate / coplanar inputs produce
    /// `JoltError::Build`.
    ConvexHull {
        points: Vec<Vec3>,
    },
    /// Triangle mesh. Used for static colliders (the engine v0.22
    /// design doc forbids dynamic mesh colliders -- they are not
    /// physics-correct).
    Mesh {
        vertices: Vec<Vec3>,
        triangles: Vec<[u32; 3]>,
    },
    /// Static compound of sub-shapes. Each tuple is `(offset, rotation,
    /// child)` relative to the compound's origin. v0.22 ships
    /// StaticCompound only (immutable post-build); MutableCompound
    /// (for chunk-streaming edits) is v0.23+ if Voxelith asks.
    Compound {
        children: Vec<(Vec3, Quat, ShapeDef)>,
    },
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
            let ok = match self {
                ShapeDef::Box { half_extents } => {
                    kind = "Box";
                    let settings = JPC_BoxShapeSettings {
                        HalfExtent: crate::math::to_jpc_vec3(*half_extents),
                        ..Default::default()
                    };
                    JPC_BoxShapeSettings_Create(&settings, &mut shape, &mut err)
                }
                ShapeDef::Sphere { radius } => {
                    kind = "Sphere";
                    let settings = JPC_SphereShapeSettings {
                        Radius: *radius,
                        ..Default::default()
                    };
                    JPC_SphereShapeSettings_Create(&settings, &mut shape, &mut err)
                }
                ShapeDef::Capsule {
                    half_height,
                    radius,
                } => {
                    kind = "Capsule";
                    let settings = JPC_CapsuleShapeSettings {
                        HalfHeightOfCylinder: *half_height,
                        Radius: *radius,
                        ..Default::default()
                    };
                    JPC_CapsuleShapeSettings_Create(&settings, &mut shape, &mut err)
                }
                ShapeDef::ConvexHull { points } => {
                    kind = "ConvexHull";
                    // Convert glam Vec3 -> JPC_Vec3; storage must live
                    // until the Create call returns.
                    let pts: Vec<JPC_Vec3> = points
                        .iter()
                        .map(|p| crate::math::to_jpc_vec3(*p))
                        .collect();
                    let settings = JPC_ConvexHullShapeSettings {
                        Density: 1000.0,
                        Points: pts.as_ptr(),
                        PointsLen: pts.len(),
                        MaxConvexRadius: 0.05,
                        MaxErrorConvexRadius: 0.05,
                        HullTolerance: 1.0e-3,
                        ..Default::default()
                    };
                    JPC_ConvexHullShapeSettings_Create(&settings, &mut shape, &mut err)
                }
                ShapeDef::Mesh {
                    vertices,
                    triangles,
                } => {
                    kind = "Mesh";
                    // Mesh shapes are static-collider only per design
                    // doc; we don't validate that here -- caller passes
                    // them to a Static body.
                    let mut verts: Vec<JPC_Float3> = vertices
                        .iter()
                        .map(|v| JPC_Float3 {
                            x: v.x,
                            y: v.y,
                            z: v.z,
                        })
                        .collect();
                    let mut tris: Vec<JPC_IndexedTriangle> = triangles
                        .iter()
                        .map(|t| JPC_IndexedTriangle {
                            idx: *t,
                            materialIndex: 0,
                            userData: 0,
                        })
                        .collect();
                    let settings = JPC_MeshShapeSettings {
                        TriangleVertices: verts.as_mut_ptr(),
                        TriangleVerticesLen: verts.len(),
                        IndexedTriangles: tris.as_mut_ptr(),
                        IndexedTrianglesLen: tris.len(),
                        ..Default::default()
                    };
                    JPC_MeshShapeSettings_Create(&settings, &mut shape, &mut err)
                }
                ShapeDef::Compound { children } => {
                    kind = "Compound";
                    // Build each child shape first; failure short-circuits.
                    let child_handles: Vec<ShapeHandle> = children
                        .iter()
                        .map(|(_, _, def)| def.build())
                        .collect::<Result<Vec<_>, _>>()?;
                    let subs: Vec<JPC_SubShapeSettings> = children
                        .iter()
                        .zip(child_handles.iter())
                        .map(|((pos, rot, _), handle)| JPC_SubShapeSettings {
                            Position: crate::math::to_jpc_vec3(*pos),
                            Rotation: crate::math::to_jpc_quat(*rot),
                            Shape: handle.ptr as *const _,
                            ..Default::default()
                        })
                        .collect();
                    let settings = JPC_StaticCompoundShapeSettings {
                        SubShapes: subs.as_ptr(),
                        SubShapesLen: subs.len(),
                        ..Default::default()
                    };
                    JPC_StaticCompoundShapeSettings_Create(
                        &settings,
                        &mut shape,
                        &mut err,
                    )
                    // child_handles drop here -- each holds a refcount
                    // that the compound shape now also references;
                    // dropping releases our ref and the compound's
                    // internal AddRef keeps the children alive.
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
