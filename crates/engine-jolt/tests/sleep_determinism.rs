//! Headless determinism floor for the `Sleeping` marker primitive
//! (engine-plugin-physics v0.22.2 Task A).
//!
//! `BodyInterface::is_active()` is the one primitive the Bevy plugin's
//! `Sleeping` marker keys off. This test proves the awake->asleep
//! transition is DETERMINISTIC on the safe API with no Bevy boot: two
//! independent `World`s drop one dynamic box onto a static floor and
//! step until Jolt's island heuristic deactivates the body. The tick
//! at which `is_active()` flips to `false` MUST be identical across the
//! two runs. Mirrors `determinism_two_runs.rs`.

#![cfg(feature = "native")]

use engine_jolt::prelude::*;
use glam::Vec3;

const DELTA_TIME: f32 = 1.0 / 64.0;
const COLLISION_STEPS: i32 = 1;
/// ~9.4s at 64Hz -- ample for a single box to fall, settle, and trip
/// Jolt's ~0.5s sleep timer. If a run reaches this cap the body never
/// slept, which the test treats as a failure (see assertion).
const MAX_TICKS: u32 = 600;

/// Drop one dynamic box onto a static floor and return the tick index
/// at which `is_active()` first reports `false` (the sleep tick), or
/// `None` if the body never slept within `MAX_TICKS`.
fn sleep_tick() -> Option<u32> {
    let mut world = World::new(WorldConfig::default());

    let floor = ShapeDef::Box {
        half_extents: Vec3::new(50.0, 1.0, 50.0),
    }
    .build()
    .expect("floor shape");
    let box_shape = ShapeDef::Box {
        half_extents: Vec3::splat(0.5),
    }
    .build()
    .expect("box shape");

    let body_id = {
        let mut bi = world.body_interface();
        let floor_def = BodyDef::static_body(floor, Vec3::new(0.0, -1.0, 0.0));
        let _floor = bi.spawn(&floor_def);
        let box_def = BodyDef::dynamic(box_shape, Vec3::new(0.0, 2.0, 0.0));
        bi.spawn(&box_def)
    };

    world.optimize_broad_phase();

    for tick in 0..MAX_TICKS {
        world.drive_step(DELTA_TIME, COLLISION_STEPS);
        if !world.body_interface().is_active(body_id) {
            return Some(tick);
        }
    }
    None
}

#[test]
fn sleep_tick_is_identical_across_runs() {
    let run_a = sleep_tick();
    let run_b = sleep_tick();

    assert!(
        run_a.is_some(),
        "the dynamic box never slept within {MAX_TICKS} ticks; either \
         Jolt sleeping is disabled or the box never settled -- the \
         `Sleeping` marker would never fire for the consumer"
    );
    assert_eq!(
        run_a, run_b,
        "engine-jolt sleep determinism violation: the box slept at tick \
         {run_a:?} in run A but {run_b:?} in run B. is_active() must be \
         bit-deterministic across runs for the Bevy `Sleeping` marker to \
         appear at the same FixedPostUpdate tick across two Apps."
    );
}
