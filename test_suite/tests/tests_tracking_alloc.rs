use rsruckig::prelude::*;

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// Counting allocator to verify the allocation-free steady state of Trackig
struct CountingAllocator;

static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
static COUNTING: AtomicBool = AtomicBool::new(false);

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if COUNTING.load(Ordering::Relaxed) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        System.alloc(layout)
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout)
    }
}

#[global_allocator]
static GLOBAL: CountingAllocator = CountingAllocator;

const DT: f64 = 0.001;

#[test]
fn test_tracking_steady_state_is_allocation_free() {
    let mut otg = Trackig::<1, IgnoreErrorHandler>::new(None, DT);
    let mut input = InputParameter::<1>::new(None);
    let mut output = OutputParameter::<1>::new(None);
    input.synchronization = Synchronization::None;
    input.max_velocity = daov_stack![4.0];
    input.max_acceleration = daov_stack![20.0];
    input.max_jerk = daov_stack![200.0];
    input.max_position = Some(daov_stack![2.0]);
    input.min_position = Some(daov_stack![-2.0]);

    let mut target = TargetState::<1>::new(None);

    // Warm-up
    for i in 0..100 {
        let t = i as f64 * DT;
        target.position[0] = (2.0 * t).sin();
        target.velocity[0] = 2.0 * (2.0 * t).cos();
        otg.update(&target, &input, &mut output).unwrap();
        output.pass_to_input(&mut input);
    }

    // Steady state must not allocate
    ALLOCATIONS.store(0, Ordering::Relaxed);
    COUNTING.store(true, Ordering::Relaxed);
    for i in 100..1100 {
        let t = i as f64 * DT;
        target.position[0] = (2.0 * t).sin();
        target.velocity[0] = 2.0 * (2.0 * t).cos();
        otg.update(&target, &input, &mut output).unwrap();
        output.pass_to_input(&mut input);
    }
    COUNTING.store(false, Ordering::Relaxed);

    let allocations = ALLOCATIONS.load(Ordering::Relaxed);
    assert_eq!(
        allocations, 0,
        "steady-state tracking made {allocations} heap allocations"
    );
}

/// Guards `InputParameter::copy_from`: in heap mode (`DOF == 0`) a retarget must not
/// reallocate `current_input`. Before that change, `Ruckig::update` cloned the input
/// on every recalculation, allocating a fresh `Vec` per array field each cycle.
#[test]
fn test_heap_mode_retarget_is_allocation_free() {
    let mut otg = Ruckig::<0, IgnoreErrorHandler>::new(Some(3), DT);
    let mut output = OutputParameter::<0>::new(Some(3));

    let mut input_a = InputParameter::<0>::new(Some(3));
    input_a.synchronization = Synchronization::None;
    input_a.current_position = daov_heap![0.0, 0.0, 0.0];
    input_a.target_position = daov_heap![1.0, 2.0, 3.0];
    input_a.max_velocity = daov_heap![3.0, 3.0, 3.0];
    input_a.max_acceleration = daov_heap![3.0, 3.0, 3.0];
    input_a.max_jerk = daov_heap![4.0, 4.0, 4.0];
    input_a.min_velocity = Some(daov_heap![-3.0, -3.0, -3.0]);
    input_a.min_acceleration = Some(daov_heap![-3.0, -3.0, -3.0]);

    let mut input_b = input_a.clone();
    input_b.target_position = daov_heap![-1.0, -2.0, -3.0];

    // Warm-up: let one-time allocations (trajectory profiles, first Some(..) copies,
    // internal scratch buffers) settle before counting.
    for i in 0..50 {
        let input = if i % 2 == 0 { &input_a } else { &input_b };
        otg.update(input, &mut output).unwrap();
    }

    // Each iteration alternates the target, so `update` recalculates every cycle and
    // copies the input into `current_input`. That copy must reuse existing storage.
    ALLOCATIONS.store(0, Ordering::Relaxed);
    COUNTING.store(true, Ordering::Relaxed);
    for i in 0..1000 {
        let input = if i % 2 == 0 { &input_a } else { &input_b };
        otg.update(input, &mut output).unwrap();
    }
    COUNTING.store(false, Ordering::Relaxed);

    let allocations = ALLOCATIONS.load(Ordering::Relaxed);
    assert_eq!(
        allocations, 0,
        "heap-mode retarget made {allocations} heap allocations"
    );
}
