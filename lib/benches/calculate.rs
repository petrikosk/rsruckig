//! Microbenchmarks for the trajectory generator hot paths.
//!
//! These guard `calculate()` (the retarget / recalculation path, which dominates
//! CPU) and `Trajectory::at_time` (the allocation-free steady-state sampling path)
//! against regressions. Inputs are fixed / seeded so runs are comparable across
//! commits: establish a baseline with
//! `cargo bench --bench calculate -- --save-baseline main`, then compare later
//! commits with `cargo bench --bench calculate -- --baseline main`.

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use rsruckig::prelude::*;

/// Small deterministic PRNG (xorshift64*) so the corpus is reproducible without
/// pulling extra dev-dependencies into the workspace.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        // xorshift64* must not be seeded with 0.
        Rng(seed | 1)
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform f64 in [lo, hi).
    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        let f = (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64;
        lo + f * (hi - lo)
    }
}

/// A non-trivial 3-DoF state-to-state input that exercises the full third-order
/// Step-1/Step-2 path (non-zero start/target velocity and acceleration, asymmetric
/// negative-direction limits).
fn make_3dof_input() -> InputParameter<3> {
    let mut input = InputParameter::new(None);
    input.current_position = daov_stack![0.0, 0.0, 0.5];
    input.current_velocity = daov_stack![0.0, -2.2, -0.5];
    input.current_acceleration = daov_stack![0.0, 2.5, -0.5];
    input.target_position = daov_stack![5.0, -2.0, -3.5];
    input.target_velocity = daov_stack![0.0, -0.5, -2.0];
    input.target_acceleration = daov_stack![0.0, 0.0, 0.5];
    input.max_velocity = daov_stack![3.0, 1.0, 3.0];
    input.max_acceleration = daov_stack![3.0, 2.0, 1.0];
    input.max_jerk = daov_stack![4.0, 3.0, 2.0];
    input.min_velocity = Some(daov_stack![-2.0, -0.5, -3.0]);
    input.min_acceleration = Some(daov_stack![-2.0, -2.0, -2.0]);
    input
}

fn make_1dof_input() -> InputParameter<1> {
    let mut input = InputParameter::new(None);
    input.current_position = daov_stack![0.0];
    input.current_velocity = daov_stack![0.7];
    input.current_acceleration = daov_stack![0.5];
    input.target_position = daov_stack![10.0];
    input.target_velocity = daov_stack![-1.0];
    input.target_acceleration = daov_stack![0.0];
    input.max_velocity = daov_stack![5.0];
    input.max_acceleration = daov_stack![10.0];
    input.max_jerk = daov_stack![20.0];
    input
}

/// Generate a fixed corpus of valid 3-DoF inputs for a stable aggregate across
/// profile families.
fn make_corpus(n: usize) -> Vec<InputParameter<3>> {
    let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.005);
    let mut rng = Rng::new(0x9E37_79B9_7F4A_7C15);
    let mut corpus = Vec::with_capacity(n);
    while corpus.len() < n {
        let mut input = InputParameter::new(None);
        for d in 0..3 {
            input.current_position[d] = rng.uniform(-8.0, 8.0);
            input.current_velocity[d] = rng.uniform(-1.5, 1.5);
            input.current_acceleration[d] = rng.uniform(-1.5, 1.5);
            input.target_position[d] = rng.uniform(-8.0, 8.0);
            input.target_velocity[d] = rng.uniform(-1.5, 1.5);
            input.target_acceleration[d] = rng.uniform(-1.5, 1.5);
            input.max_velocity[d] = rng.uniform(3.0, 12.0);
            input.max_acceleration[d] = rng.uniform(3.0, 12.0);
            input.max_jerk[d] = rng.uniform(3.0, 12.0);
        }
        // Only keep inputs that yield a valid trajectory so every bench iteration
        // measures a full successful calculation.
        if otg.validate_input(&input, false, false).is_err() {
            continue;
        }
        let mut traj = Trajectory::new(None);
        if otg.calculate(&input, &mut traj).is_ok() {
            corpus.push(input);
        }
    }
    corpus
}

fn bench_calculate(c: &mut Criterion) {
    // 1-DoF retarget: no synchronization, isolates the per-DoF solver.
    {
        let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.005);
        let input = make_1dof_input();
        let mut traj = Trajectory::new(None);
        c.bench_function("calculate_retarget_1dof", |b| {
            b.iter(|| {
                otg.calculate(black_box(&input), black_box(&mut traj))
                    .unwrap();
                black_box(&traj);
            })
        });
    }

    // 3-DoF, time synchronization (default): exercises `synchronize`.
    {
        let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.005);
        let input = make_3dof_input();
        let mut traj = Trajectory::new(None);
        c.bench_function("calculate_retarget_3dof", |b| {
            b.iter(|| {
                otg.calculate(black_box(&input), black_box(&mut traj))
                    .unwrap();
                black_box(&traj);
            })
        });
    }

    // 3-DoF, phase synchronization.
    {
        let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.005);
        let mut input = make_3dof_input();
        input.synchronization = Synchronization::Phase;
        let mut traj = Trajectory::new(None);
        c.bench_function("calculate_retarget_3dof_phase_sync", |b| {
            b.iter(|| {
                otg.calculate(black_box(&input), black_box(&mut traj))
                    .unwrap();
                black_box(&traj);
            })
        });
    }

    // 3-DoF, no synchronization: each DoF time-optimal, no common-time search.
    {
        let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.005);
        let mut input = make_3dof_input();
        input.synchronization = Synchronization::None;
        let mut traj = Trajectory::new(None);
        c.bench_function("calculate_retarget_3dof_no_sync", |b| {
            b.iter(|| {
                otg.calculate(black_box(&input), black_box(&mut traj))
                    .unwrap();
                black_box(&traj);
            })
        });
    }
}

fn bench_sample_steady_state(c: &mut Criterion) {
    // Pure `at_time` sampling on an already-computed trajectory — the
    // allocation-free steady-state path taken when the input is unchanged.
    let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.005);
    let input = make_3dof_input();
    let mut traj = Trajectory::new(None);
    otg.calculate(&input, &mut traj).unwrap();
    let duration = traj.get_duration();

    let mut pos = DataArrayOrVec::new(None, 0.0);
    let mut vel = DataArrayOrVec::new(None, 0.0);
    let mut acc = DataArrayOrVec::new(None, 0.0);
    let mut jerk = DataArrayOrVec::new(None, 0.0);
    let section = 0usize;

    c.bench_function("sample_at_time_3dof", |b| {
        let mut t = 0.0;
        b.iter(|| {
            t += 0.005;
            if t > duration {
                t = 0.0;
            }
            traj.at_time(
                black_box(t),
                &mut Some(&mut pos),
                &mut Some(&mut vel),
                &mut Some(&mut acc),
                &mut Some(&mut jerk),
                &mut Some(section),
            );
            black_box((&pos, &vel, &acc, &jerk, section));
        })
    });
}

/// Heap-mode (runtime-DoF) retarget via `update`. Each iteration alternates the
/// target so `update` takes the recalculation branch, which clones the input into
/// `current_input` — the allocation this benchmark exists to measure.
fn bench_update_retarget_heap(c: &mut Criterion) {
    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(3), 0.005);
    let mut output = OutputParameter::new(Some(3));

    let mut input_a = InputParameter::<0>::new(Some(3));
    input_a.current_position = daov_heap![0.0, 0.0, 0.0];
    input_a.target_position = daov_heap![1.0, 2.0, 3.0];
    input_a.max_velocity = daov_heap![3.0, 3.0, 3.0];
    input_a.max_acceleration = daov_heap![3.0, 3.0, 3.0];
    input_a.max_jerk = daov_heap![4.0, 4.0, 4.0];

    let mut input_b = input_a.clone();
    input_b.target_position = daov_heap![-1.0, -2.0, -3.0];

    c.bench_function("update_retarget_heap_3dof", |b| {
        let mut toggle = false;
        b.iter(|| {
            toggle = !toggle;
            let input = if toggle { &input_a } else { &input_b };
            let _ = otg.update(black_box(input), &mut output);
            black_box(&output);
        })
    });
}

fn bench_corpus(c: &mut Criterion) {
    let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.005);
    let corpus = make_corpus(1000);
    let mut traj = Trajectory::new(None);
    c.bench_function("calculate_corpus_3dof_1000", |b| {
        b.iter(|| {
            let mut acc = 0.0;
            for input in &corpus {
                otg.calculate(black_box(input), &mut traj).unwrap();
                acc += traj.get_duration();
            }
            black_box(acc)
        })
    });
}

criterion_group!(
    benches,
    bench_calculate,
    bench_sample_steady_state,
    bench_update_retarget_heap,
    bench_corpus
);
criterion_main!(benches);
