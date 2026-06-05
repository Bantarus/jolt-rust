# vendor.diff.md -- jolt-rust local patches vs upstream

This file documents every local modification to this fork of
`SecondHalfGames/jolt-rust`. It is the contract between the
`xtreme-game-engine` v0.22 milestone (engine-plugin-physics) and the
upstream physics stack. Each patch is named, dated, justified, and
diff-stat'd. When upstream subtree-pulls happen (quarterly), this
file is the checklist for re-applying / dropping each patch.

The pattern mirrors `rive-rust`'s sibling-repo precedent
(`/home/bantarus/DEV/rive-rust/`), where day-one patches against
upstream `rive-app/rive-runtime` are tracked similarly.

## Upstream baseline

| Component | Source | Pinned SHA | Version |
|---|---|---|---|
| jolt-rust | `SecondHalfGames/jolt-rust` | `40c3aac7ac58df1fbfcf23ccda1e60b17fd47238` | crates: joltc-sys 0.3.1, rolt 0.3.1 |
| JoltC | `SecondHalfGames/JoltC` (submodule) | `2982004387a9e36ca89525a87d983709d3666da7` | -- |
| JoltPhysics | `jrouwe/JoltPhysics` (nested submodule) | `0373ec0dd762e4bc2f6acdb08371ee84fa23c6db` | **v5.3.0** |

The joltc-sys 0.3.1 `version = "0.3.1+Jolt-5.0.0"` build-metadata
string is stale -- the submodule resolves to JoltPhysics v5.3.0
(verified via `Jolt/Core/Core.h`: `JPH_VERSION_MAJOR 5`,
`JPH_VERSION_MINOR 3`, `JPH_VERSION_PATCH 0`).

## Local patches

### Patch 1 -- `cross-platform-deterministic` cargo feature

Date: 2026-06-05 (v0.22 Step 1)
Files: `crates/joltc-sys/Cargo.toml`, `crates/joltc-sys/build.rs`
Lines added: ~25

**Why.** Engine v0.22's headline value is bit-identical physics
across Win-MSVC / Linux-Clang / macOS-ARM64 / Linux-ARM64.
JoltPhysics already supports this via its CMake variable
`CROSS_PLATFORM_DETERMINISTIC=ON` -- when enabled it (a) sets the
`JPH_CROSS_PLATFORM_DETERMINISTIC` compile def on the `Jolt` target,
(b) suppresses `JPH_USE_FMADD` (FMA contraction would otherwise
produce platform-specific float results), (c) the flag propagates
into JoltC via `JoltC/CMakeLists.txt:44` (`target_link_libraries(joltc
PUBLIC Jolt)`) so both libraries build in lockstep.

Upstream joltc-sys does NOT expose this flag as a cargo feature
(only `asserts`, `double-precision`, `object-layer-u32`). This
patch plumbs it through.

**What.** Adds the `cross-platform-deterministic` cargo feature to
`Cargo.toml`. In `build.rs`:
- Sets `config.define("CROSS_PLATFORM_DETERMINISTIC", "ON")` on the
  cmake invocation when the feature is enabled. The variable
  propagates into JoltPhysics via CMake subdirectory scope
  inheritance (`JoltC/CMakeLists.txt:83` `add_subdirectory(JoltPhysics/Build)`),
  and JoltPhysics's `CMakeLists.txt:547` reads it.
- Mirrors the flag into bindgen with `-DJPH_CROSS_PLATFORM_DETERMINISTIC=1`
  so header parsing matches compiled library layout. (No JPC-prefixed
  twin: the flag only gates internal Jolt float ops; JoltC's C ABI
  surface is unaffected.)

**Verification (Step 1).** Verified by inspecting
`target/release/build/joltc-sys-<HASH>/out/build/CMakeCache.txt`:
`CROSS_PLATFORM_DETERMINISTIC:BOOL=ON`. Verified end-to-end by
`examples/hello_jolt`: two independent 64-tick simulations produce
bit-identical body positions across all 10 dynamic bodies, on
WSL2 Linux (Clang 18 / GCC 13).

**Cross-platform validation pending.** Win-MSVC + macOS-ARM64 +
Linux-ARM64 verification deferred to the 4090 relay session per
engine AGENTS.md Rule 16. Titan engine's CI already validates the
upstream JoltPhysics flag across that matrix.

**Re-apply on upstream pull.** Trivial -- the patch only adds; it
doesn't modify any line upstream might rewrite. If upstream ever
adds this feature themselves, drop this patch and use theirs.

### Patch 2 -- `-ffast-math` / `-march=native` build guard

Date: 2026-06-05 (v0.22 Step 1)
File: `crates/joltc-sys/build.rs`
Lines added: ~30

**Why.** A consumer who sets `RUSTFLAGS="-march=native"` or
`CXXFLAGS="-ffast-math"` in their CI / shell env will silently
re-introduce FMA contraction even when our
`cross-platform-deterministic` feature is on. The build succeeds,
local tests pass, then physics hashes diverge on a different
consumer's machine. Jolt's docs warn about this; no other layer
in the stack catches it.

**What.** Adds `check_no_fast_math()` called at the top of `main()`
in `build.rs`. Inspects `CFLAGS`, `CXXFLAGS`, `RUSTFLAGS`, and
`CARGO_ENCODED_RUSTFLAGS` env vars; panics with an actionable
message if any contains `-ffast-math` or `-march=native`.

**Caveat.** The guard catches the env-var path but cannot catch
fast-math enabled via a `.cargo/config.toml`'s `rustflags = [...]`
TOML key -- cargo doesn't expose that as a build-script env var.
This is a limitation accepted for v0.22; a v0.23+ enhancement could
read `.cargo/config.toml` directly. Documented in the build.rs
comment.

**Re-apply on upstream pull.** Trivial -- the patch only adds an
early panic-guard function. Drop if upstream ships its own guard.

## Notes on patches we intentionally did NOT apply

The engine v0.22 synthesis (cf. `xtreme-game-engine/docs/v0.22-design.md`
section "Build / CI plan") proposed cherry-picking three upstream
PRs for Linux undefined-references fixes (joltc-sys issues #5 / #9 /
#10). **Skipped -- the baseline build succeeded cleanly on this
WSL2 Linux Clang 18 host** with no patches applied. Those issues
appear to have landed upstream between when the synthesis ran and
this Step 1 spike. If a clean WSL2 build ever regresses, revisit.

The synthesis also proposed pinning JoltPhysics to 5.3.0 as a
deliberate version bump from upstream's 5.0.0. **No bump needed --
the joltc-sys 0.3.1 submodule already resolves to JoltPhysics v5.3.0**;
the `+Jolt-5.0.0` semver build-metadata string is stale.

## Quarterly subtree pull checklist

When pulling upstream main into our fork:

1. `git fetch upstream main` then `git merge upstream/main` (or
   subtree pull). Resolve conflicts in `Cargo.toml` and `build.rs`
   in favor of upstream changes plus re-applying our patches.
2. Re-init submodules: `git submodule update --init --recursive`.
3. Re-verify each patch is still load-bearing: read the updated
   joltc-sys `Cargo.toml` features list to see if `cross-platform-deterministic`
   has been adopted upstream (drop our patch if so).
4. Re-run `cargo build -p joltc-sys --release --features cross-platform-deterministic`
   and `cargo run -p joltc-sys --release --features cross-platform-deterministic --example hello_jolt`.
5. Hash the determinism smoke output and compare against the
   previous SHA; document any divergence as a determinism-relevant
   upstream change.
6. Re-tag this file's "Upstream baseline" table with new SHAs.
7. Bump engine-jolt's reference pin in
   `xtreme-game-engine/crates/engine-plugin-physics/Cargo.toml` if
   the API surface changed.
8. Update this file with the new baseline SHAs and any new patches.

Quarterly cadence is Bantarus-owned engine maintenance debt,
explicitly named per engine AGENTS Rule 28.
