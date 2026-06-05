# engine-jolt

Bevy-agnostic safe Rust wrapper over JoltC for `xtreme-game-engine`
v0.22+ (`engine-plugin-physics`).

**Status: v0.22 Step 1 placeholder skeleton.** The public API surface
is authored in v0.22 Step 2; this crate currently exports two
const-fn feature-flag introspection helpers and nothing else. The
feature graph is fully wired so engine workspace feature unification
behaves correctly.

## Why this crate exists

The engine ships physics as two layers per `xtreme-game-engine/docs/v0.22-design.md`
section "Crate layout":

1. **engine-jolt (THIS crate, Bevy-AGNOSTIC).** Safe Rust wrapper.
   Owns `World`, `WorldConfig`, `drive_step(dt, substeps)`,
   `BodyInterface`, `ShapeDef` builders, deterministic `ContactSink`,
   `WorldSnapshot` save/restore, listener-bridge macro pattern.
   Lives in `/home/bantarus/DEV/jolt-rust/` (this sibling repo).
   ZERO bevy / bevy_ecs / engine-* deps -- enforceable by grep.
   This crate exists so the safety + determinism contract is testable
   without booting Bevy (headless `determinism_two_runs.rs` test).

2. **engine-plugin-physics (downstream, Bevy plugin).** Lives in
   `xtreme-game-engine/crates/engine-plugin-physics/`. Thin Bevy
   plugin: Reflect components (`RigidBody`, `Collider`, `Velocity`,
   ...), `NonSend<JoltWorld>` resource, `FixedPostUpdate` step
   pipeline, `physics.raycast` XGEP method. Depends on engine-jolt.

Games depend on `engine_plugin_physics::prelude` only -- never on
this crate directly. Rule 3 / Rule 28 ownership.

## Features

| Feature | Default | Pulls | Notes |
|---|---|---|---|
| `native` | off | `joltc-sys` | Compiles JoltPhysics C++ runtime. Without this, the crate is a no-op stub so `cargo check --workspace` stays light. |
| `cross_deterministic` | off | `native` + `joltc-sys/cross-platform-deterministic` | Bit-identical physics across Win-MSVC / Linux-Clang / macOS-ARM64 / Linux-ARM64. ~8% perf cost. Implies `native`. |
| `double_precision` | off | `native` + `joltc-sys/double-precision` | f64 world coordinates. v0.23+ axis. |
| `object_layer_u32` | off | `native` + `joltc-sys/object-layer-u32` | 32-bit object layers (4B layers). v0.23+ if Voxelith asks. |
| `asserts` | off | `native` + `joltc-sys/asserts` | Jolt internal debug asserts. |

The downstream `engine-plugin-physics`'s `native` feature pulls
`cross_deterministic` by default -- determinism is the engine's
headline value.

## Build

```bash
# Stub-only (no C++ toolchain needed):
cargo check -p engine-jolt

# Native (requires cmake + clang + libclang + build-essential):
cargo build -p engine-jolt --release --features native

# Native + cross-platform-deterministic (what the engine uses):
cargo build -p engine-jolt --release --features cross_deterministic
```

## Determinism floor

Verified in Step 1 via `crates/joltc-sys/examples/hello_jolt.rs`:
two independent 64-tick simulations with identical inputs produce
bit-identical body positions on WSL2 Linux Clang 18. Cross-platform
validation (Win-MSVC / macOS-ARM64) pending native-Windows 4090
relay per engine `AGENTS.md` Rule 16.

The Step 2 deliverable `tests/determinism_two_runs.rs` ports this
smoke onto the safe wrapper surface -- same proof, safe API.

## License

MIT OR Apache-2.0 (matches the rest of the jolt-rust workspace).

The Step 2 listener-bridge macro pattern in `src/traits.rs` (lifted
from `crates/rolt/src/traits.rs`) carries dual MIT/Apache-2.0
attribution to SecondHalfGames per the rolt source headers; rolt
itself stays a reference, not a runtime dep.
