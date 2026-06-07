//! Headless proof of the `Wake` primitive (engine-plugin-physics
//! v0.22.3 Task A).
//!
//! Demonstrates BOTH the bug and the fix with no Bevy boot:
//!   1. A Dynamic body settles on a static floor and SLEEPS.
//!   2. The floor is destroyed -- and the body STAYS ASLEEP (Jolt does
//!      not auto-wake on static-body removal: this is the bug that makes
//!      debris float when terrain is dug out from under it).
//!   3. `activate()` wakes it; one step later it is active and falling.
//!
//! Running the identical script in two independent `World`s under
//! `cross_deterministic` must yield bit-identical post-wake positions --
//! proving activation is deterministic. Mirrors determinism_two_runs.rs.

#![cfg(feature = "native")]

use engine_jolt::prelude::*;
use glam::Vec3;

const DELTA_TIME: f32 = 1.0 / 64.0;
const COLLISION_STEPS: i32 = 1;
const MAX_TICKS: u32 = 600; // ample for a single box to settle + sleep
const SETTLE_TICKS: u32 = 8; // post-floor-removal ticks to prove no auto-wake
const FALL_TICKS: u32 = 16; // post-activate ticks to let it fall

struct WakeOutcome {
    slept: bool,
    still_asleep_after_floor_removed: bool,
    active_after_wake: bool,
    y_rest: f32,
    y_after_fall: f32,
    pos_bits_after_fall: [u32; 3],
}

fn run_wake_scenario() -> WakeOutcome {
    let mut world = World::new(WorldConfig::default());

    let floor_shape = ShapeDef::Box {
        half_extents: Vec3::new(50.0, 1.0, 50.0),
    }
    .build()
    .expect("floor shape");
    let box_shape = ShapeDef::Box {
        half_extents: Vec3::splat(0.5),
    }
    .build()
    .expect("box shape");

    let (floor_id, box_id) = {
        let mut bi = world.body_interface();
        let floor = bi.spawn(&BodyDef::static_body(floor_shape, Vec3::new(0.0, -1.0, 0.0)));
        let b = bi.spawn(&BodyDef::dynamic(box_shape, Vec3::new(0.0, 2.0, 0.0)));
        (floor, b)
    };
    world.optimize_broad_phase();

    // 1. Step until the box sleeps.
    let mut slept = false;
    for _ in 0..MAX_TICKS {
        world.drive_step(DELTA_TIME, COLLISION_STEPS);
        if !world.body_interface().is_active(box_id) {
            slept = true;
            break;
        }
    }
    let y_rest = world.body_interface().position(box_id).y;

    // 2. Remove the floor; the box must STAY asleep (the bug).
    world.body_interface().destroy(floor_id);
    for _ in 0..SETTLE_TICKS {
        world.drive_step(DELTA_TIME, COLLISION_STEPS);
    }
    let still_asleep_after_floor_removed = !world.body_interface().is_active(box_id);

    // 3. Wake it explicitly; one step later it must be active.
    world.body_interface().activate(box_id);
    world.drive_step(DELTA_TIME, COLLISION_STEPS);
    let active_after_wake = world.body_interface().is_active(box_id);

    // Let it fall, then capture position.
    for _ in 0..FALL_TICKS {
        world.drive_step(DELTA_TIME, COLLISION_STEPS);
    }
    let p = world.body_interface().position(box_id);

    WakeOutcome {
        slept,
        still_asleep_after_floor_removed,
        active_after_wake,
        y_rest,
        y_after_fall: p.y,
        pos_bits_after_fall: [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()],
    }
}

#[test]
fn slept_body_needs_activate_to_fall_after_support_removed() {
    let o = run_wake_scenario();

    assert!(o.slept, "the box never slept within {MAX_TICKS} ticks");
    assert!(
        o.still_asleep_after_floor_removed,
        "the box auto-woke when the floor was destroyed -- this test \
         encodes Jolt's documented behavior (NO auto-wake on static-body \
         removal); if this fails, the wake primitive's premise changed"
    );
    assert!(
        o.active_after_wake,
        "activate() did not wake the body -- BodyInterface::activate is \
         not re-inserting it into a simulating island"
    );
    assert!(
        o.y_after_fall < o.y_rest - 0.05,
        "after wake the box did not fall: y_rest={}, y_after_fall={} \
         (expected a clear decrease once support was gone)",
        o.y_rest,
        o.y_after_fall
    );
}

#[test]
fn wake_is_deterministic() {
    let a = run_wake_scenario();
    let b = run_wake_scenario();

    assert_eq!(
        a.pos_bits_after_fall, b.pos_bits_after_fall,
        "engine-jolt wake determinism violation: post-wake positions \
         diverged across two runs. activate() must be bit-deterministic \
         for the Bevy `Wake` marker to behave identically across Apps."
    );
}

#[test]
fn cross_deterministic_feature_is_enabled() {
    assert!(
        engine_jolt::cross_deterministic_enabled(),
        "run with --features cross_deterministic; cross-platform \
         bit-identicality is not guaranteed without it"
    );
}
