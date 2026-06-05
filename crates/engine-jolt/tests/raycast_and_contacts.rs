//! Coverage for the Step 2.1 additions: raycast, contact events,
//! compound shapes. Each test is independent and uses the smallest
//! world that exercises the feature.

#![cfg(feature = "native")]

use engine_jolt::prelude::*;
use glam::{Quat, Vec3};

/// Cast a ray at a single static box; verify the hit returns the
/// box's BodyId and a sensible distance.
#[test]
fn raycast_hits_static_box() {
    let mut world = World::new(WorldConfig::default());

    let box_shape = ShapeDef::Box {
        half_extents: Vec3::splat(1.0),
    }
    .build()
    .unwrap();

    let id = {
        let mut bi = world.body_interface();
        bi.spawn(&BodyDef::static_body(box_shape, Vec3::new(5.0, 0.0, 0.0)))
    };
    world.optimize_broad_phase();

    // Ray from origin pointing +X should hit the box at distance ~4.0
    // (box centered at x=5 with half-extent 1, so its left face is at x=4).
    let hit = world
        .narrow_phase()
        .cast_ray(Vec3::ZERO, Vec3::X, 10.0)
        .expect("ray should hit");
    assert_eq!(hit.body, id, "hit body should be the static box");
    assert!(
        (hit.distance - 4.0).abs() < 0.01,
        "hit distance should be ~4.0, got {}",
        hit.distance
    );
    assert!(
        hit.point.x > 3.9 && hit.point.x < 4.1,
        "hit point x should be ~4.0, got {}",
        hit.point.x
    );
}

/// Cast a ray into empty space; verify no hit returns None.
#[test]
fn raycast_misses_empty_space() {
    let world = World::new(WorldConfig::default());
    let hit = world
        .narrow_phase()
        .cast_ray(Vec3::ZERO, Vec3::Y, 100.0);
    assert!(hit.is_none(), "ray into empty space should not hit");
}

/// Compound shape: assemble a 4-box static collider, verify it builds
/// and a body spawn against it succeeds.
#[test]
fn compound_shape_builds_and_spawns() {
    let mut world = World::new(WorldConfig::default());

    let child = ShapeDef::Box {
        half_extents: Vec3::splat(0.5),
    };
    let compound = ShapeDef::Compound {
        children: vec![
            (Vec3::new(-1.0, 0.0, 0.0), Quat::IDENTITY, child.clone()),
            (Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY, child.clone()),
            (Vec3::new(0.0, 0.0, -1.0), Quat::IDENTITY, child.clone()),
            (Vec3::new(0.0, 0.0, 1.0), Quat::IDENTITY, child),
        ],
    };
    let handle = compound.build().expect("compound build");

    let id = {
        let mut bi = world.body_interface();
        bi.spawn(&BodyDef::static_body(handle, Vec3::ZERO))
    };
    // The compound's AABB extends well past the spawn point; ray from
    // above must hit one of the children.
    world.optimize_broad_phase();
    let hit = world
        .narrow_phase()
        .cast_ray(Vec3::new(1.0, 5.0, 0.0), -Vec3::Y, 10.0)
        .expect("ray should hit one of the compound children");
    assert_eq!(hit.body, id, "hit must be the compound body");
}

/// Convex hull from a 4-point tetrahedron.
#[test]
fn convex_hull_shape_builds() {
    let _world = World::new(WorldConfig::default());
    let hull = ShapeDef::ConvexHull {
        points: vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ],
    };
    let _handle = hull.build().expect("hull build");
}

/// Contact events: drop a dynamic box onto a static floor, step until
/// they touch, verify drain_contacts returns at least one Added event
/// and the events are sorted by body id pair.
#[test]
fn contacts_emit_added_event_sorted() {
    let mut world = World::new(WorldConfig::default());

    let floor_id;
    let box_id;
    {
        let floor_shape = ShapeDef::Box {
            half_extents: Vec3::new(50.0, 1.0, 50.0),
        }
        .build()
        .unwrap();
        let box_shape = ShapeDef::Box {
            half_extents: Vec3::splat(0.5),
        }
        .build()
        .unwrap();
        let mut bi = world.body_interface();
        floor_id = bi.spawn(&BodyDef::static_body(floor_shape, Vec3::new(0.0, -1.0, 0.0)));
        box_id = bi.spawn(&BodyDef::dynamic(box_shape, Vec3::new(0.0, 2.0, 0.0)));
    }
    world.optimize_broad_phase();

    // Step until the box has time to fall onto the floor. With
    // gravity 9.81 m/s^2 falling 1.5m takes ~0.55s = ~36 ticks at 64Hz.
    let mut saw_added = false;
    let mut sample_event = None;
    for _ in 0..120 {
        world.drive_step(1.0 / 64.0, 1);
        let events = world.drain_contacts();
        if !events.is_empty() {
            // Verify sorted property: each event has body_a <= body_b
            // and the Vec itself is in (body_a, body_b, kind) order.
            for window in events.windows(2) {
                assert!(window[0] <= window[1], "events must be sorted");
            }
            for e in &events {
                assert!(e.body_a <= e.body_b, "body pair must be canonicalized");
                if e.kind == ContactKind::Added {
                    saw_added = true;
                    sample_event = Some(*e);
                }
            }
        }
        if saw_added {
            break;
        }
    }
    assert!(saw_added, "expected at least one Added contact event within 120 ticks");
    let e = sample_event.unwrap();
    // The expected pair is (floor, box) sorted -- floor was spawned
    // first so floor_id < box_id.
    assert!(
        (e.body_a == floor_id && e.body_b == box_id)
            || (e.body_a == box_id && e.body_b == floor_id),
        "Added event must involve floor + box; got {:?}",
        e
    );
}
