//! hello_jolt -- Step 1 deterministic smoke for engine v0.22 (E13).
//!
//! Drops 10 boxes onto a static floor under JoltPhysics, runs 64 steps
//! at dt=1/64 substeps=1 on a single-threaded JobSystem, then runs the
//! same simulation a second time and asserts bit-identical body
//! positions. This is the FLOOR of the v0.22 determinism contract:
//! same seed (none here -- input is deterministic by construction),
//! same binary, same platform -> bit-identical floats.
//!
//! Run with:
//!     cargo run -p joltc-sys --release --example hello_jolt \
//!         --features cross-platform-deterministic
//!
//! NOTE: This example links joltc-sys raw FFI directly. The safe
//! wrapper (engine-jolt) lands in Step 2; until then, every Jolt call
//! is unsafe and manually paired with its delete.

use std::ffi::{c_uint, c_void, CStr, CString};
use std::ptr;

use joltc_sys::*;

const OL_NON_MOVING: JPC_ObjectLayer = 0;
const OL_MOVING: JPC_ObjectLayer = 1;
const BPL_NON_MOVING: JPC_BroadPhaseLayer = 0;
const BPL_MOVING: JPC_BroadPhaseLayer = 1;
const BPL_COUNT: u32 = 2;

const TICK_COUNT: u32 = 64;
const DELTA_TIME: f32 = 1.0 / 64.0;
const COLLISION_STEPS: i32 = 1;
const BOX_COUNT: usize = 10;

fn vec3(x: f32, y: f32, z: f32) -> JPC_Vec3 {
    JPC_Vec3 { x, y, z, _w: z }
}

fn rvec3(x: Real, y: Real, z: Real) -> JPC_RVec3 {
    JPC_RVec3 { x, y, z, _w: z }
}

fn create_box(settings: &JPC_BoxShapeSettings) -> Result<*mut JPC_Shape, CString> {
    let mut shape: *mut JPC_Shape = ptr::null_mut();
    let mut err: *mut JPC_String = ptr::null_mut();
    unsafe {
        if JPC_BoxShapeSettings_Create(settings, &mut shape, &mut err) {
            Ok(shape)
        } else {
            Err(CStr::from_ptr(JPC_String_c_str(err)).to_owned())
        }
    }
}

unsafe extern "C" fn bpl_get_num_broad_phase_layers(_this: *const c_void) -> c_uint {
    BPL_COUNT
}

unsafe extern "C" fn bpl_get_broad_phase_layer(
    _this: *const c_void,
    layer: JPC_ObjectLayer,
) -> JPC_BroadPhaseLayer {
    match layer {
        OL_NON_MOVING => BPL_NON_MOVING,
        OL_MOVING => BPL_MOVING,
        _ => panic!("unknown object layer {layer}"),
    }
}

const BPL: JPC_BroadPhaseLayerInterfaceFns = JPC_BroadPhaseLayerInterfaceFns {
    GetNumBroadPhaseLayers: Some(bpl_get_num_broad_phase_layers),
    GetBroadPhaseLayer: Some(bpl_get_broad_phase_layer),
};

unsafe extern "C" fn ovb_should_collide(
    _this: *const c_void,
    layer1: JPC_ObjectLayer,
    layer2: JPC_BroadPhaseLayer,
) -> bool {
    match layer1 {
        OL_NON_MOVING => layer2 == BPL_MOVING,
        OL_MOVING => true,
        _ => panic!("unknown object layer {layer1}"),
    }
}

const OVB: JPC_ObjectVsBroadPhaseLayerFilterFns = JPC_ObjectVsBroadPhaseLayerFilterFns {
    ShouldCollide: Some(ovb_should_collide),
};

unsafe extern "C" fn ovo_should_collide(
    _this: *const c_void,
    layer1: JPC_ObjectLayer,
    layer2: JPC_ObjectLayer,
) -> bool {
    match layer1 {
        OL_NON_MOVING => layer2 == OL_MOVING,
        OL_MOVING => true,
        _ => panic!("unknown object layer {layer1}"),
    }
}

const OVO: JPC_ObjectLayerPairFilterFns = JPC_ObjectLayerPairFilterFns {
    ShouldCollide: Some(ovo_should_collide),
};

/// One self-contained physics simulation. Spawns floor + BOX_COUNT
/// dynamic boxes at deterministic positions, steps TICK_COUNT times,
/// returns each body's final translation packed as u32 bit-patterns
/// (so equality comparison detects ANY floating divergence, including
/// signed zero).
fn simulate() -> Vec<u32> {
    unsafe {
        JPC_RegisterDefaultAllocator();
        JPC_FactoryInit();
        JPC_RegisterTypes();

        let temp_allocator = JPC_TempAllocatorImpl_new(10 * 1024 * 1024);
        // Single-threaded job system: no inter-tick thread ordering
        // non-determinism for the spike. Production engine-jolt will
        // wire this off WorldConfig.thread_count.
        let job_system = JPC_JobSystemSingleThreaded_new(JPC_MAX_PHYSICS_JOBS as _);

        let broad_phase_layer_interface =
            JPC_BroadPhaseLayerInterface_new(ptr::null(), BPL);
        let object_vs_broad_phase_layer_filter =
            JPC_ObjectVsBroadPhaseLayerFilter_new(ptr::null_mut(), OVB);
        let object_vs_object_layer_filter =
            JPC_ObjectLayerPairFilter_new(ptr::null_mut(), OVO);

        let physics_system = JPC_PhysicsSystem_new();
        JPC_PhysicsSystem_Init(
            physics_system,
            /* max_bodies */ 1024,
            /* num_body_mutexes */ 0,
            /* max_body_pairs */ 1024,
            /* max_contact_constraints */ 1024,
            broad_phase_layer_interface,
            object_vs_broad_phase_layer_filter,
            object_vs_object_layer_filter,
        );

        let body_interface = JPC_PhysicsSystem_GetBodyInterface(physics_system);

        let floor_shape = create_box(&JPC_BoxShapeSettings {
            HalfExtent: vec3(50.0, 1.0, 50.0),
            ..Default::default()
        })
        .expect("floor shape");
        let floor_settings = JPC_BodyCreationSettings {
            Position: rvec3(0.0, -1.0, 0.0),
            MotionType: JPC_MOTION_TYPE_STATIC,
            ObjectLayer: OL_NON_MOVING,
            Shape: floor_shape,
            ..Default::default()
        };
        let floor = JPC_BodyInterface_CreateBody(body_interface, &floor_settings);
        let floor_id = JPC_Body_GetID(floor);
        JPC_BodyInterface_AddBody(body_interface, floor_id, JPC_ACTIVATION_DONT_ACTIVATE);

        // 10 dynamic boxes on a deterministic stagger pattern.
        let box_shape = create_box(&JPC_BoxShapeSettings {
            HalfExtent: vec3(0.5, 0.5, 0.5),
            ..Default::default()
        })
        .expect("box shape");

        let mut box_ids = Vec::with_capacity(BOX_COUNT);
        for i in 0..BOX_COUNT {
            // Stagger X so they don't stack perfectly (deterministic
            // input but exercises the solver against non-trivial
            // contact patterns).
            let xi = i as f32;
            let x = (xi - (BOX_COUNT as f32) * 0.5) * 0.4;
            let y = 5.0 + xi * 0.25;
            let z = (xi % 3.0 - 1.0) * 0.2;
            let body_settings = JPC_BodyCreationSettings {
                Position: rvec3(x as Real, y as Real, z as Real),
                MotionType: JPC_MOTION_TYPE_DYNAMIC,
                ObjectLayer: OL_MOVING,
                Shape: box_shape,
                ..Default::default()
            };
            let body = JPC_BodyInterface_CreateBody(body_interface, &body_settings);
            let id = JPC_Body_GetID(body);
            JPC_BodyInterface_AddBody(body_interface, id, JPC_ACTIVATION_ACTIVATE);
            box_ids.push(id);
        }

        // Optimize broadphase after the initial spawn so the static
        // floor's tree is built deterministically before stepping.
        JPC_PhysicsSystem_OptimizeBroadPhase(physics_system);

        for _ in 0..TICK_COUNT {
            JPC_PhysicsSystem_Update(
                physics_system,
                DELTA_TIME,
                COLLISION_STEPS,
                temp_allocator,
                job_system.cast::<JPC_JobSystem>(),
            );
        }

        // Collect (BodyId, position) sorted by BodyId. The body_ids
        // were assigned at insert order; we never iterate Jolt's
        // internal active-body list (warning: non-deterministic order
        // upstream).
        let mut positions: Vec<(u32, JPC_RVec3)> = Vec::with_capacity(BOX_COUNT);
        for id in &box_ids {
            // GetPosition takes *const; GetBodyInterface gave us *mut.
            let pos = JPC_BodyInterface_GetPosition(
                body_interface as *const JPC_BodyInterface,
                *id,
            );
            // JPC_BodyID is a plain `pub type JPC_BodyID = u32;` alias.
            positions.push((*id, pos));
        }
        positions.sort_by_key(|(id, _)| *id);

        let mut out = Vec::with_capacity(BOX_COUNT * 4);
        for (id, pos) in &positions {
            out.push(*id);
            // Real is f32 by default, f64 with `double-precision`.
            // Pack each component as its native bit-width by casting
            // through f64 -> f32 lossily so we always emit u32 bits.
            // (This matches the design doc's "f32 transform" surface;
            // double-precision is a v0.23+ axis.)
            out.push((pos.x as f32).to_bits());
            out.push((pos.y as f32).to_bits());
            out.push((pos.z as f32).to_bits());
        }

        // Teardown in reverse order of construction.
        JPC_PhysicsSystem_delete(physics_system);
        JPC_BroadPhaseLayerInterface_delete(broad_phase_layer_interface);
        JPC_ObjectVsBroadPhaseLayerFilter_delete(object_vs_broad_phase_layer_filter);
        JPC_ObjectLayerPairFilter_delete(object_vs_object_layer_filter);
        JPC_JobSystemSingleThreaded_delete(job_system);
        JPC_TempAllocatorImpl_delete(temp_allocator);

        out
    }
}

fn main() {
    println!("hello_jolt -- v0.22 Step 1 determinism smoke");
    println!("  TICK_COUNT={TICK_COUNT} DELTA_TIME=1/64 COLLISION_STEPS={COLLISION_STEPS} BOX_COUNT={BOX_COUNT}");
    #[cfg(feature = "cross-platform-deterministic")]
    println!("  cross-platform-deterministic: ON");
    #[cfg(not(feature = "cross-platform-deterministic"))]
    println!("  cross-platform-deterministic: OFF");

    let run_a = simulate();
    let run_b = simulate();

    // run_a and run_b each pack BOX_COUNT * 4 u32s: [id, x.bits, y.bits, z.bits] per body.
    assert_eq!(run_a.len(), BOX_COUNT * 4);
    assert_eq!(run_b.len(), BOX_COUNT * 4);

    println!();
    println!("  sorted body positions after {TICK_COUNT} ticks (run A):");
    for chunk in run_a.chunks(4) {
        let id = chunk[0];
        let x = f32::from_bits(chunk[1]);
        let y = f32::from_bits(chunk[2]);
        let z = f32::from_bits(chunk[3]);
        println!("    body {id:>5}: ({x:>10.6}, {y:>10.6}, {z:>10.6})");
    }

    if run_a == run_b {
        println!();
        println!("  determinism OK -- run A and run B are bit-identical across all {BOX_COUNT} bodies");
    } else {
        println!();
        eprintln!("  DETERMINISM FAILURE -- run A and run B diverged");
        for (i, (a, b)) in run_a.iter().zip(run_b.iter()).enumerate() {
            if a != b {
                eprintln!("    word {i}: A=0x{a:08x} B=0x{b:08x}");
            }
        }
        std::process::exit(1);
    }
}
