//! Object + broad-phase layers.
//!
//! engine-jolt v0.22 ships the canonical two-layer scheme
//! (`STATIC`, `MOVING`) per the design doc decision 5 (broadphase
//! layer scheme: ObjectLayer u16 + 2 broadphase layers STATIC, MOVING).
//! Future v0.23+ work can expose a runtime `LayerConfig` resource
//! for consumer override (Voxelith's debris/character/projectile
//! split) but until then the discipline is: static colliders go in
//! STATIC, dynamic + kinematic bodies go in MOVING.
//!
//! ObjectLayer is a u16 (joltc-sys default; the `object_layer_u32`
//! feature widens to u32 but engine-jolt does not flip its own
//! `ObjectLayer` typedef on that feature -- consumers who need u32
//! reach through joltc-sys directly for now).

/// Object layer newtype around the JoltC default `JPC_ObjectLayer`
/// (u16). Layer 0 = STATIC (non-moving colliders), layer 1 = MOVING
/// (dynamic + kinematic bodies). Larger ids reserved for v0.23+.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ObjectLayer(pub u16);

impl ObjectLayer {
    pub const STATIC: ObjectLayer = ObjectLayer(0);
    pub const MOVING: ObjectLayer = ObjectLayer(1);
}

#[cfg(feature = "native")]
impl From<ObjectLayer> for joltc_sys::JPC_ObjectLayer {
    fn from(l: ObjectLayer) -> Self {
        l.0 as joltc_sys::JPC_ObjectLayer
    }
}

/// Broad-phase layer newtype. Same mapping as ObjectLayer for v0.22:
/// layer 0 = STATIC (skipped during broadphase moves), layer 1 =
/// MOVING.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BroadPhaseLayer(pub u8);

impl BroadPhaseLayer {
    pub const STATIC: BroadPhaseLayer = BroadPhaseLayer(0);
    pub const MOVING: BroadPhaseLayer = BroadPhaseLayer(1);
    pub const COUNT: u32 = 2;
}

#[cfg(feature = "native")]
impl From<BroadPhaseLayer> for joltc_sys::JPC_BroadPhaseLayer {
    fn from(l: BroadPhaseLayer) -> Self {
        l.0 as joltc_sys::JPC_BroadPhaseLayer
    }
}
