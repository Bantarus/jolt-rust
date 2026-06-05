//! Headless determinism floor for engine-jolt v0.22.
//!
//! Ports the Step 1 `joltc-sys/examples/hello_jolt.rs` smoke onto the
//! safe API. Two independent simulations with identical inputs MUST
//! produce bit-identical body positions. Runs without booting Bevy.
//!
//! This test is the "(a) wrapper" half of the v0.22 determinism
//! contract (the design doc's section "Determinism contract"). The
//! Bevy-side mirror is `crates/engine-plugin-physics/tests/
//! determinism.rs` at Step 4.

#![cfg(feature = "native")]

use engine_jolt::prelude::*;
use glam::Vec3;

const TICK_COUNT: u32 = 64;
const DELTA_TIME: f32 = 1.0 / 64.0;
const COLLISION_STEPS: i32 = 1;
const BOX_COUNT: usize = 10;

/// One complete simulation. Returns each body's final position
/// packed as u32 bit patterns so equality detects ANY float
/// divergence (including signed zero / NaN).
fn simulate() -> Vec<u32> {
    let mut world = World::new(WorldConfig::default());

    let floor_shape = ShapeDef::Box {
        half_extents: Vec3::new(50.0, 1.0, 50.0),
    }
    .build()
    .expect("floor shape");

    let mut body_ids = Vec::with_capacity(BOX_COUNT);

    {
        let mut bi = world.body_interface();

        let floor_def = BodyDef::static_body(floor_shape, Vec3::new(0.0, -1.0, 0.0));
        let _floor = bi.spawn(&floor_def);

        let box_shape = ShapeDef::Box {
            half_extents: Vec3::splat(0.5),
        }
        .build()
        .expect("box shape");

        for i in 0..BOX_COUNT {
            // Same deterministic stagger pattern as hello_jolt.rs.
            let xi = i as f32;
            let x = (xi - (BOX_COUNT as f32) * 0.5) * 0.4;
            let y = 5.0 + xi * 0.25;
            let z = (xi % 3.0 - 1.0) * 0.2;
            let def = BodyDef::dynamic(box_shape.clone(), Vec3::new(x, y, z));
            body_ids.push(bi.spawn(&def));
        }
    }

    world.optimize_broad_phase();

    for _ in 0..TICK_COUNT {
        world.drive_step(DELTA_TIME, COLLISION_STEPS);
    }

    let mut positions: Vec<(BodyId, Vec3)> = body_ids
        .iter()
        .map(|id| (*id, world.body_interface().position(*id)))
        .collect();
    positions.sort_by_key(|(id, _)| *id);

    let mut out = Vec::with_capacity(BOX_COUNT * 4);
    for (id, pos) in &positions {
        out.push(id.0);
        out.push(pos.x.to_bits());
        out.push(pos.y.to_bits());
        out.push(pos.z.to_bits());
    }
    out
}

#[test]
fn two_independent_runs_are_bit_identical() {
    let run_a = simulate();
    let run_b = simulate();

    assert_eq!(run_a.len(), BOX_COUNT * 4);
    assert_eq!(run_b.len(), BOX_COUNT * 4);
    assert_eq!(
        run_a, run_b,
        "engine-jolt determinism violation: run A and run B diverged. \
         The first divergence at byte index N implies the bridge between \
         JoltC and engine-jolt is leaking non-determinism (HashMap iteration, \
         pointer-address dependence, etc). Investigate the BodyId ordering \
         in body_interface.rs and the broadphase callbacks in world.rs first."
    );
}

#[test]
fn cross_deterministic_feature_is_enabled() {
    // The test is meaningful only when the cross_deterministic feature
    // is on -- without it, the determinism guarantee weakens to
    // same-platform only. This test asserts the test harness was run
    // with the right feature combination so a false-pass on a
    // misconfigured CI invocation is loud.
    assert!(
        engine_jolt::cross_deterministic_enabled(),
        "tests/determinism_two_runs.rs must be run with --features \
         cross_deterministic; bit-identicality across platforms is not \
         guaranteed without it. Run: \
         `cargo test -p engine-jolt --features cross_deterministic`"
    );
}
