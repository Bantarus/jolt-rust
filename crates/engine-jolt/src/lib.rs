//! engine-jolt -- Bevy-agnostic safe wrapper over JoltC.
//!
//! v0.22 Step 2.0 surface. Owns the deterministic `World` +
//! `BodyInterface` + safe `ShapeDef` builders (Box / Sphere /
//! Capsule). Headless `tests/determinism_two_runs.rs` proves the
//! safe API produces bit-identical body positions across two
//! independent runs given identical inputs.
//!
//! Step 2.1 (immediate follow-on) lands `ShapeDef::ConvexHull / Mesh
//! / Compound`, the deterministic `ContactSink` (sorted buffer +
//! listener-bridge macro lifted from rolt under dual MIT/Apache-2.0
//! attribution), `NarrowPhaseQuery::cast_ray`, and `WorldSnapshot`
//! save/restore.
//!
//! Design reference: `xtreme-game-engine/docs/v0.22-design.md`
//! section "Crate layout" -> `engine-jolt (sibling repo, Bevy-AGNOSTIC)`.

pub mod body;
pub mod body_interface;
pub mod error;
pub mod layers;
pub mod math;
pub mod shape;
pub mod world;

pub use body::{BodyDef, BodyId, BodyType};
pub use body_interface::BodyInterface;
pub use error::JoltError;
pub use layers::{BroadPhaseLayer, ObjectLayer};
pub use shape::{ShapeDef, ShapeHandle};
pub use world::{World, WorldConfig};

/// One-stop import for the common surface. Mirrors the Bevy
/// `prelude::*` convention so `use engine_jolt::prelude::*;` brings
/// in the value types most engine-plugin-physics methods need.
pub mod prelude {
    pub use crate::body::{BodyDef, BodyId, BodyType};
    pub use crate::body_interface::BodyInterface;
    pub use crate::error::JoltError;
    pub use crate::layers::{BroadPhaseLayer, ObjectLayer};
    pub use crate::shape::{ShapeDef, ShapeHandle};
    pub use crate::world::{World, WorldConfig};
}

/// Returns `true` if this crate was built with the `native` feature
/// (i.e. joltc-sys is linked and the Jolt C++ runtime is available).
pub const fn native_enabled() -> bool {
    cfg!(feature = "native")
}

/// Returns `true` if this crate was built with `cross_deterministic`,
/// guaranteeing bit-identical physics across Win-MSVC / Linux-Clang /
/// macOS-ARM64 / Linux-ARM64 same-binary same-input runs at ~8%
/// perf cost. Implies `native_enabled()`.
pub const fn cross_deterministic_enabled() -> bool {
    cfg!(feature = "cross_deterministic")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn feature_flags_consistent() {
        if cross_deterministic_enabled() {
            assert!(native_enabled(), "cross_deterministic must imply native");
        }
    }
}
