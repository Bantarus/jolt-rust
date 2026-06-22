//! Headless proof of the v0.38 predicted-physics restore primitives
//! (engine-plugin-physics Model B / B2).
//!
//! The four new `BodyInterface` setters exist so a networked rollback can
//! push a captured authoritative `(pose, velocity)` back into a live Jolt
//! body IN PLACE (not destroy/respawn) before replaying unacked inputs:
//!   - `angular_velocity()` getter (closes the `sync_out` readback gap),
//!   - `set_angular_velocity()`,
//!   - `set_position_and_rotation()` (teleport, activating),
//!   - `set_pose_and_velocity()` (the atomic full-kinematic-state restore).
//!
//! The contract this test pins: a set-then-get with NO intervening step is
//! BIT-EXACT (Jolt stores the values directly), and the same restore script
//! in two independent `World`s under `cross_deterministic` reads back
//! bit-identically -- the restore is deterministic, the precondition the
//! predicted-physics resim relies on.
//!
//! Native-only; EXECUTION is Windows-deferred (the WSL2 native-test-build
//! OOM, Rule 36) -- compile-verified in-env, run on the playtest box.

#![cfg(feature = "native")]

use engine_jolt::prelude::*;
use glam::{Quat, Vec3};

const DELTA_TIME: f32 = 1.0 / 64.0;
const COLLISION_STEPS: i32 = 1;

fn vec_bits(v: Vec3) -> [u32; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}

fn quat_bits(q: Quat) -> [u32; 4] {
    [q.x.to_bits(), q.y.to_bits(), q.z.to_bits(), q.w.to_bits()]
}

/// Spawn a free-flight dynamic box, let it move a few ticks, then exercise
/// each restore setter and read the body's state back. Returns the
/// post-restore readback bits for the two-run determinism check.
fn run_restore_scenario() -> ([u32; 3], [u32; 4], [u32; 3], [u32; 3]) {
    let mut world = World::new(WorldConfig::default());

    let box_shape = ShapeDef::Box {
        half_extents: Vec3::splat(0.5),
    }
    .build()
    .expect("box shape");

    let box_id = {
        let mut bi = world.body_interface();
        bi.spawn(&BodyDef::dynamic(box_shape, Vec3::new(0.0, 5.0, 0.0)))
    };
    world.optimize_broad_phase();

    // Let it fall a few ticks so it carries a non-trivial pose + velocity.
    for _ in 0..8 {
        world.drive_step(DELTA_TIME, COLLISION_STEPS);
    }

    // The authoritative state we want to restore the body TO.
    let auth_pos = Vec3::new(1.5, 3.25, -2.0);
    let auth_rot = Quat::from_rotation_y(0.6).normalize();
    let auth_lin = Vec3::new(0.5, -1.25, 0.75);
    let auth_ang = Vec3::new(0.1, -0.2, 0.3);

    let mut bi = world.body_interface();

    // (1) The atomic full-kinematic-state restore is bit-exact set-then-get.
    bi.set_pose_and_velocity(box_id, auth_pos, auth_rot, auth_lin, auth_ang);
    assert_eq!(
        vec_bits(bi.position(box_id)),
        vec_bits(auth_pos),
        "set_pose_and_velocity position must read back bit-exact"
    );
    assert_eq!(
        quat_bits(bi.rotation(box_id)),
        quat_bits(auth_rot),
        "set_pose_and_velocity rotation must read back bit-exact"
    );
    assert_eq!(
        vec_bits(bi.linear_velocity(box_id)),
        vec_bits(auth_lin),
        "set_pose_and_velocity linear velocity must read back bit-exact"
    );
    assert_eq!(
        vec_bits(bi.angular_velocity(box_id)),
        vec_bits(auth_ang),
        "set_pose_and_velocity angular velocity must read back bit-exact"
    );

    // (2) set_position_and_rotation teleports pose, leaves velocity intact.
    let tp_pos = Vec3::new(-4.0, 9.0, 1.0);
    let tp_rot = Quat::from_rotation_x(-0.3).normalize();
    bi.set_position_and_rotation(box_id, tp_pos, tp_rot);
    assert_eq!(vec_bits(bi.position(box_id)), vec_bits(tp_pos));
    assert_eq!(quat_bits(bi.rotation(box_id)), quat_bits(tp_rot));
    assert_eq!(
        vec_bits(bi.linear_velocity(box_id)),
        vec_bits(auth_lin),
        "set_position_and_rotation must not disturb linear velocity"
    );

    // (3) set_angular_velocity + the new getter round-trip.
    let new_ang = Vec3::new(-0.4, 0.55, -0.05);
    bi.set_angular_velocity(box_id, new_ang);
    assert_eq!(vec_bits(bi.angular_velocity(box_id)), vec_bits(new_ang));

    (
        vec_bits(bi.position(box_id)),
        quat_bits(bi.rotation(box_id)),
        vec_bits(bi.linear_velocity(box_id)),
        vec_bits(bi.angular_velocity(box_id)),
    )
}

#[test]
fn restore_setters_round_trip_bit_exact_and_deterministic() {
    let run_a = run_restore_scenario();
    let run_b = run_restore_scenario();
    assert_eq!(
        run_a, run_b,
        "pose/velocity restore must be bit-identical across two independent Worlds \
         (the determinism precondition the predicted-physics resim relies on)"
    );
}
