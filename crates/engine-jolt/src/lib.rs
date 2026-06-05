//! engine-jolt -- Bevy-agnostic safe wrapper over JoltC.
//!
//! This is a placeholder skeleton committed at v0.22 Step 1 so the
//! engine-side path-dep from `xtreme-game-engine/crates/engine-plugin-physics`
//! resolves during the Step 3 plugin-skeleton work. The actual safe
//! wrapper surface (World / WorldConfig / drive_step / BodyInterface /
//! ShapeDef / NarrowPhaseQuery / deterministic ContactSink /
//! WorldSnapshot) is authored in v0.22 Step 2.
//!
//! Until Step 2 lands, this crate exports nothing public beyond a
//! single feature-flag identification helper. The native feature is
//! still wired to joltc-sys so feature unification across the engine
//! workspace stays honest.
//!
//! Design reference: `xtreme-game-engine/docs/v0.22-design.md`
//! section "Crate layout" -> `engine-jolt (sibling repo, Bevy-AGNOSTIC)`.
//! Rule reference: forthcoming `xtreme-game-engine/AGENTS.md` Rule 28
//! (landing at v0.22 milestone tag).

/// Returns `true` if this crate was built with the `native` feature
/// (i.e. joltc-sys is linked and the Jolt C++ runtime is available).
/// Returns `false` for the no-op stub build downstream consumers see
/// via `cargo check --workspace` without `--features physics-native`.
pub const fn native_enabled() -> bool {
    cfg!(feature = "native")
}

/// Returns `true` if this crate was built with `cross_deterministic`,
/// guaranteeing bit-identical physics across {Win-MSVC, Linux-Clang,
/// macOS-ARM64, Linux-ARM64} same-binary same-input runs at ~8% perf
/// cost. Implies `native_enabled()`.
pub const fn cross_deterministic_enabled() -> bool {
    cfg!(feature = "cross_deterministic")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn feature_flags_consistent() {
        // If cross_deterministic is on, native must also be on
        // (the feature graph enforces this).
        if cross_deterministic_enabled() {
            assert!(native_enabled(), "cross_deterministic must imply native");
        }
    }
}
