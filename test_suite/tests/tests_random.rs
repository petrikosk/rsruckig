//! Randomized trajectory tests, modelled after the random test cases of the
//! upstream Ruckig test suite (`test/test_target.cpp`).
//!
//! Every case draws kinematic states and limits from seeded distributions (so
//! failures are reproducible), skips inputs the validation rejects, and checks
//! the invariants of the calculated trajectory.
//!
//! The number of trajectories per test scales with the environment variable
//! `RSRUCKIG_RANDOM_TRAJECTORIES` (default 16384), e.g.
//! `RSRUCKIG_RANDOM_TRAJECTORIES=1000000 cargo test --release --test tests_random`.
//!
//! With `RSRUCKIG_ORACLE_DUMP=<file>`, every checked input is appended to
//! `<file>` together with the calculated result, for a bit-exact comparison
//! with upstream Ruckig by `test_suite/oracle/oracle.cpp` (see there).

use rand_core::RngCore;
use rand_distr::{Distribution, Normal, Uniform};
use rand_pcg::Pcg64Mcg;
use rsruckig::prelude::*;

use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::{Mutex, OnceLock};

const DELTA_TIME: f64 = 0.005;

/// Append `input` and its calculated result to the oracle dump, if requested
fn dump_for_oracle<const DOF: usize>(input: &InputParameter<DOF>) {
    static DUMP: OnceLock<Option<Mutex<BufWriter<File>>>> = OnceLock::new();
    let Some(dump) = DUMP.get_or_init(|| {
        let path = std::env::var_os("RSRUCKIG_ORACLE_DUMP")?;
        let file = File::options().create(true).append(true).open(path).ok()?;
        Some(Mutex::new(BufWriter::new(file)))
    }) else {
        return;
    };

    // The result code instead of an error
    let mut otg = Ruckig::<DOF, IgnoreErrorHandler>::new(None, DELTA_TIME);
    let mut traj = Trajectory::new(None);
    let result = otg.calculate(input, &mut traj).unwrap_or(RuckigResult::Error);

    let interface = |dof| (control_interface(input, dof) == ControlInterface::Velocity) as u8;
    let synchronization = match input.synchronization {
        Synchronization::Time => 0,
        Synchronization::TimeIfNecessary => 1,
        Synchronization::Phase => 2,
        Synchronization::None => 3,
    };
    // A negative value stands for no minimum duration
    let minimum_duration = input.minimum_duration.unwrap_or(-1.0);
    let mut line = format!(
        "{DOF} {} {synchronization} {:016x}",
        interface(0),
        minimum_duration.to_bits()
    );
    let mut push = |value: f64| line.push_str(&format!(" {:016x}", value.to_bits()));
    for dof in 0..DOF {
        push(input.current_position[dof]);
        push(input.current_velocity[dof]);
        push(input.current_acceleration[dof]);
        push(input.target_position[dof]);
        push(input.target_velocity[dof]);
        push(input.target_acceleration[dof]);
        push(input.max_velocity[dof]);
        push(input.max_acceleration[dof]);
        push(input.max_jerk[dof]);
        push(input.min_velocity.as_ref().map_or(-input.max_velocity[dof], |v| v[dof]));
        push(input.min_acceleration.as_ref().map_or(-input.max_acceleration[dof], |v| v[dof]));
    }
    let duration = if result == RuckigResult::Working { traj.get_duration() } else { 0.0 };
    let mut dump = dump.lock().unwrap();
    writeln!(dump, "{line} {} {:016x}", result as i32, duration.to_bits()).unwrap();
    dump.flush().unwrap();
}

/// Number of random trajectories of a test with relative weight `share`
fn number_of_trajectories(share: f64) -> usize {
    let base = std::env::var("RSRUCKIG_RANDOM_TRAJECTORIES")
        .ok()
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(16_384);
    ((base as f64 * share) as usize).max(1)
}

/// Seeded source of random per-DoF values (cf. upstream `Randomizer`)
struct Randomizer<D: Distribution<f64>> {
    rng: Pcg64Mcg,
    dist: D,
    uniform: Uniform<f64>,
}

impl<D: Distribution<f64>> Randomizer<D> {
    fn new(dist: D, seed: u64) -> Self {
        let mut rng = Pcg64Mcg::new(seed as u128 | 1);
        rng.next_u64();
        Self {
            rng,
            dist,
            uniform: Uniform::new(0.0, 1.0),
        }
    }

    fn fill<const DOF: usize>(&mut self, values: &mut DataArrayOrVec<f64, DOF>) {
        for value in values.iter_mut() {
            *value = self.dist.sample(&mut self.rng);
        }
    }

    /// Sample with probability `p`, zero otherwise
    fn fill_or_zero<const DOF: usize>(&mut self, values: &mut DataArrayOrVec<f64, DOF>, p: f64) {
        for value in values.iter_mut() {
            *value = if self.uniform.sample(&mut self.rng) < p {
                self.dist.sample(&mut self.rng)
            } else {
                0.0
            };
        }
    }

    /// Sample a limit that leaves room for the magnitude of `offset`
    fn fill_above<const DOF: usize>(
        &mut self,
        values: &mut DataArrayOrVec<f64, DOF>,
        offset: &DataArrayOrVec<f64, DOF>,
    ) {
        for (value, offset) in values.iter_mut().zip(offset.iter()) {
            *value = self.dist.sample(&mut self.rng) + offset.abs();
        }
    }

    /// Sample a negative limit that leaves room for the magnitude of `offset`
    fn fill_below<const DOF: usize>(
        &mut self,
        values: &mut DataArrayOrVec<f64, DOF>,
        offset: &DataArrayOrVec<f64, DOF>,
    ) {
        for (value, offset) in values.iter_mut().zip(offset.iter()) {
            *value = self.dist.sample(&mut self.rng) - offset.abs();
        }
    }
}

fn position_dist() -> Normal<f64> {
    Normal::new(0.0, 4.0).unwrap()
}

fn dynamic_dist() -> Normal<f64> {
    Normal::new(0.0, 0.8).unwrap()
}

fn limit_dist() -> Uniform<f64> {
    Uniform::new(0.08, 16.0)
}

fn limit_dist_high() -> Uniform<f64> {
    Uniform::new(10.0, 1_000_000.0)
}

fn min_limit_dist() -> Uniform<f64> {
    Uniform::new(-16.0, -0.08)
}

/// Kinematic state of all DoFs at `time`
fn sample<const DOF: usize>(traj: &Trajectory<DOF>, time: f64) -> [DataArrayOrVec<f64, DOF>; 3] {
    let mut position = DataArrayOrVec::new(None, 0.0);
    let mut velocity = DataArrayOrVec::new(None, 0.0);
    let mut acceleration = DataArrayOrVec::new(None, 0.0);
    traj.at_time(
        time,
        &mut Some(&mut position),
        &mut Some(&mut velocity),
        &mut Some(&mut acceleration),
        &mut None,
        &mut None,
    );
    [position, velocity, acceleration]
}

fn control_interface<const DOF: usize>(input: &InputParameter<DOF>, dof: usize) -> ControlInterface {
    input
        .per_dof_control_interface
        .as_ref()
        .map_or(input.control_interface, |interfaces| interfaces[dof])
}

/// Calculate the trajectory for `input` and check that it is valid and ends
/// in the target state. Returns false if the input was skipped (invalid, or a
/// duration beyond the supported range).
fn check_calculation<const DOF: usize>(
    otg: &mut Ruckig<DOF, ThrowErrorHandler>,
    input: &InputParameter<DOF>,
    traj: &mut Trajectory<DOF>,
) -> bool {
    if otg.validate_input(input, false, true).is_err() {
        return false;
    }
    dump_for_oracle(input);

    let result = otg
        .calculate(input, traj)
        .unwrap_or_else(|error| panic!("calculation failed: {error}\ninput: {input}"));
    if result == RuckigResult::ErrorTrajectoryDuration {
        return false;
    }
    assert_eq!(result, RuckigResult::Working, "input: {input}");

    let duration = traj.get_duration();
    assert!(duration.is_finite() && duration >= 0.0, "duration {duration}, input: {input}");

    for (dof, &t_min) in traj.get_independent_min_durations().iter().enumerate() {
        assert!(
            t_min <= duration + 1e-9,
            "independent minimum duration {t_min} of DoF {dof} exceeds the duration {duration}, input: {input}"
        );
    }

    let synchronization = |dof: usize| {
        input
            .per_dof_synchronization
            .as_ref()
            .map_or(input.synchronization, |synchronizations| synchronizations[dof])
    };

    for dof in 0..DOF {
        // An unsynchronized DoF reaches its target at its own minimum
        // duration (and keeps moving with the target velocity afterwards)
        let t_end = if synchronization(dof) == Synchronization::None {
            traj.get_independent_min_durations()[dof]
        } else {
            duration
        };
        let [position, velocity, acceleration] = sample(traj, t_end);
        assert!(
            position[dof].is_finite() && velocity[dof].is_finite() && acceleration[dof].is_finite(),
            "non-finite final state of DoF {dof}, input: {input}"
        );
        if control_interface(input, dof) == ControlInterface::Position {
            assert!(
                (position[dof] - input.target_position[dof]).abs() < 1e-8,
                "final position {} of DoF {dof} misses the target {}, input: {input}",
                position[dof],
                input.target_position[dof]
            );
        }
        assert!(
            (velocity[dof] - input.target_velocity[dof]).abs() < 1e-8,
            "final velocity {} of DoF {dof} misses the target {}, input: {input}",
            velocity[dof],
            input.target_velocity[dof]
        );
        assert!(
            (acceleration[dof] - input.target_acceleration[dof]).abs() < 1e-8,
            "final acceleration {} of DoF {dof} misses the target {}, input: {input}",
            acceleration[dof],
            input.target_acceleration[dof]
        );
    }

    let [position, velocity, acceleration] = sample(traj, 0.0);
    for dof in 0..DOF {
        // Without a jerk (acceleration) limit, the acceleration (velocity)
        // may jump at the start
        let continuous_acceleration = input.max_jerk[dof].is_finite();
        let continuous_velocity =
            continuous_acceleration || input.max_acceleration[dof].is_finite();
        assert!(
            (position[dof] - input.current_position[dof]).abs() < 1e-8
                && (!continuous_velocity
                    || (velocity[dof] - input.current_velocity[dof]).abs() < 1e-8)
                && (!continuous_acceleration
                    || (acceleration[dof] - input.current_acceleration[dof]).abs() < 1e-8),
            "initial state of DoF {dof} does not match the current state, input: {input}"
        );
    }

    true
}

/// Run `count` valid random cases produced by `generate`
fn run_random<const DOF: usize>(
    count: usize,
    mut generate: impl FnMut(&mut InputParameter<DOF>),
    mut check: impl FnMut(&mut Ruckig<DOF, ThrowErrorHandler>, &InputParameter<DOF>, &Trajectory<DOF>),
) {
    let mut otg = Ruckig::<DOF, ThrowErrorHandler>::new(None, DELTA_TIME);
    let mut traj = Trajectory::new(None);
    let mut input = InputParameter::new(None);

    let mut checked = 0;
    // Bounded, so that a generator producing only invalid inputs terminates
    for _ in 0..64 * count {
        if checked == count {
            break;
        }
        generate(&mut input);
        if check_calculation(&mut otg, &input, &mut traj) {
            check(&mut otg, &input, &traj);
            checked += 1;
        }
    }
    assert_eq!(checked, count, "too few valid random inputs");
}

/// Position interface with random limits (cf. upstream `random_3`)
fn generate_position<const DOF: usize>(
    seed: u64,
) -> impl FnMut(&mut InputParameter<DOF>) {
    let mut p = Randomizer::new(position_dist(), seed);
    let mut d = Randomizer::new(dynamic_dist(), seed + 1);
    let mut l = Randomizer::new(limit_dist(), seed + 2);

    move |input| {
        p.fill(&mut input.current_position);
        d.fill_or_zero(&mut input.current_velocity, 0.9);
        d.fill_or_zero(&mut input.current_acceleration, 0.8);
        p.fill(&mut input.target_position);
        d.fill_or_zero(&mut input.target_velocity, 0.7);
        d.fill_or_zero(&mut input.target_acceleration, 0.6);
        l.fill_above(&mut input.max_velocity, &input.target_velocity);
        l.fill_above(&mut input.max_acceleration, &input.target_acceleration);
        l.fill(&mut input.max_jerk);
    }
}

#[test]
fn random_position_1() {
    run_random::<1>(number_of_trajectories(1.0), generate_position(42), |_, _, _| {});
}

#[test]
fn random_position_3() {
    run_random::<3>(number_of_trajectories(1.0), generate_position(43), |_, _, _| {});
}

#[test]
fn random_position_3_phase_synchronization() {
    let mut generate = generate_position(44);
    run_random::<3>(
        number_of_trajectories(0.5),
        |input| {
            generate(input);
            input.synchronization = Synchronization::Phase;
        },
        |_, _, _| {},
    );
}

#[test]
fn random_position_3_no_synchronization() {
    let mut generate = generate_position(45);
    run_random::<3>(
        number_of_trajectories(0.5),
        |input| {
            generate(input);
            input.synchronization = Synchronization::None;
        },
        |_, _, _| {},
    );
}

/// A minimum duration forces the time synchronization (step 2) of every DoF,
/// including the otherwise limiting one
#[test]
fn random_position_3_minimum_duration() {
    let mut generate = generate_position(67);
    let mut duration = Randomizer::new(Uniform::new(0.0, 12.0), 68);

    run_random::<3>(
        number_of_trajectories(1.0),
        |input| {
            generate(input);
            let mut minimum_duration = DataArrayOrVec::<f64, 1>::new(None, 0.0);
            duration.fill(&mut minimum_duration);
            input.minimum_duration = Some(minimum_duration[0]);
        },
        |_, input, traj| {
            assert!(
                traj.get_duration() >= input.minimum_duration.unwrap() - 1e-12,
                "duration {} below the minimum, input: {input}",
                traj.get_duration()
            );
        },
    );
}

/// Discrete states provoke degenerate profiles (cf. upstream `random_discrete_3`)
#[test]
fn random_discrete_3() {
    let mut rng = Pcg64Mcg::new(46);
    let position = Uniform::new_inclusive(-1_i32, 1);
    let dynamic = Uniform::new_inclusive(-1_i32, 1);
    let limit = Uniform::new_inclusive(1_i32, 2);

    run_random::<3>(
        number_of_trajectories(1.0),
        |input| {
            for dof in 0..3 {
                input.current_position[dof] = position.sample(&mut rng) as f64;
                input.current_velocity[dof] = dynamic.sample(&mut rng) as f64;
                input.current_acceleration[dof] = dynamic.sample(&mut rng) as f64;
                input.target_position[dof] = position.sample(&mut rng) as f64;
                input.target_velocity[dof] = dynamic.sample(&mut rng) as f64;
                input.target_acceleration[dof] = dynamic.sample(&mut rng) as f64;
                input.max_velocity[dof] = limit.sample(&mut rng) as f64;
                input.max_acceleration[dof] = limit.sample(&mut rng) as f64;
                input.max_jerk[dof] = limit.sample(&mut rng) as f64;
            }
        },
        |_, _, _| {},
    );
}

/// Very high limits (cf. upstream `random_3_high`)
#[test]
fn random_position_3_high_limits() {
    let mut p = Randomizer::new(position_dist(), 47);
    let mut d = Randomizer::new(dynamic_dist(), 48);
    let mut l = Randomizer::new(limit_dist_high(), 49);

    run_random::<3>(
        number_of_trajectories(0.5),
        |input| {
            p.fill(&mut input.current_position);
            d.fill(&mut input.current_velocity);
            d.fill(&mut input.current_acceleration);
            p.fill(&mut input.target_position);
            d.fill(&mut input.target_velocity);
            d.fill(&mut input.target_acceleration);
            l.fill_above(&mut input.max_velocity, &input.target_velocity);
            l.fill_above(&mut input.max_acceleration, &input.target_acceleration);
            l.fill(&mut input.max_jerk);
        },
        |_, _, _| {},
    );
}

/// Asymmetric limits (cf. upstream `random_direction_3`)
#[test]
fn random_position_3_directional_limits() {
    let mut generate = generate_position(50);
    let mut min_l = Randomizer::new(min_limit_dist(), 51);

    run_random::<3>(
        number_of_trajectories(1.0),
        |input| {
            generate(input);
            let mut min_velocity = DataArrayOrVec::new(None, 0.0);
            let mut min_acceleration = DataArrayOrVec::new(None, 0.0);
            min_l.fill_below(&mut min_velocity, &input.target_velocity);
            min_l.fill_below(&mut min_acceleration, &input.target_acceleration);
            input.min_velocity = Some(min_velocity);
            input.min_acceleration = Some(min_acceleration);
        },
        |_, _, _| {},
    );
}

/// Velocity interface (cf. upstream `velocity_random_3`)
#[test]
fn random_velocity_3() {
    let mut p = Randomizer::new(position_dist(), 52);
    let mut d = Randomizer::new(dynamic_dist(), 53);
    let mut l = Randomizer::new(limit_dist(), 54);

    run_random::<3>(
        number_of_trajectories(1.0),
        |input| {
            input.control_interface = ControlInterface::Velocity;
            p.fill(&mut input.current_position);
            d.fill_or_zero(&mut input.current_velocity, 0.9);
            d.fill_or_zero(&mut input.current_acceleration, 0.8);
            d.fill_or_zero(&mut input.target_velocity, 0.7);
            d.fill_or_zero(&mut input.target_acceleration, 0.6);
            l.fill_above(&mut input.max_acceleration, &input.target_acceleration);
            l.fill(&mut input.max_jerk);
        },
        |_, _, _| {},
    );
}

/// Second-order profiles: infinite jerk (cf. upstream `random_3` with
/// acceleration limits only)
#[test]
fn random_position_3_second_order() {
    let mut p = Randomizer::new(position_dist(), 55);
    let mut d = Randomizer::new(dynamic_dist(), 56);
    let mut l = Randomizer::new(limit_dist(), 57);

    run_random::<3>(
        number_of_trajectories(1.0),
        |input| {
            p.fill(&mut input.current_position);
            d.fill_or_zero(&mut input.current_velocity, 0.9);
            p.fill(&mut input.target_position);
            d.fill_or_zero(&mut input.target_velocity, 0.7);
            l.fill_above(&mut input.max_velocity, &input.target_velocity);
            l.fill(&mut input.max_acceleration);
            input.max_jerk = DataArrayOrVec::new(None, f64::INFINITY);
        },
        |_, _, _| {},
    );
}

/// First-order profiles: infinite acceleration and jerk
#[test]
fn random_position_3_first_order() {
    let mut p = Randomizer::new(position_dist(), 58);
    let mut l = Randomizer::new(limit_dist(), 59);

    run_random::<3>(
        number_of_trajectories(0.5),
        |input| {
            p.fill(&mut input.current_position);
            p.fill(&mut input.target_position);
            l.fill(&mut input.max_velocity);
            input.max_acceleration = DataArrayOrVec::new(None, f64::INFINITY);
            input.max_jerk = DataArrayOrVec::new(None, f64::INFINITY);
        },
        |_, _, _| {},
    );
}

/// Second-order velocity interface: infinite jerk
#[test]
fn random_velocity_3_second_order() {
    let mut p = Randomizer::new(position_dist(), 60);
    let mut d = Randomizer::new(dynamic_dist(), 61);
    let mut l = Randomizer::new(limit_dist(), 62);

    run_random::<3>(
        number_of_trajectories(0.5),
        |input| {
            input.control_interface = ControlInterface::Velocity;
            p.fill(&mut input.current_position);
            d.fill_or_zero(&mut input.current_velocity, 0.9);
            d.fill_or_zero(&mut input.target_velocity, 0.7);
            l.fill(&mut input.max_acceleration);
            input.max_jerk = DataArrayOrVec::new(None, f64::INFINITY);
        },
        |_, _, _| {},
    );
}

/// A trajectory that starts within its limits stays within them, and the
/// reported position extrema bound every sampled position
#[test]
fn random_position_3_limits_and_extrema() {
    let mut generate = generate_position::<3>(63);

    let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, DELTA_TIME);
    let mut traj = Trajectory::new(None);
    let mut input = InputParameter::new(None);

    let count = number_of_trajectories(0.125);
    let mut checked = 0;
    for _ in 0..256 * count {
        if checked == count {
            break;
        }
        generate(&mut input);
        // Only inputs whose current state is within the limits never exceed them
        if otg.validate_input(&input, true, true).is_err()
            || !check_calculation(&mut otg, &input, &mut traj)
        {
            continue;
        }
        checked += 1;

        let duration = traj.get_duration();
        let extrema = traj.get_position_extrema().clone();
        const STEPS: usize = 400;
        for step in 0..=STEPS {
            let time = duration * step as f64 / STEPS as f64;
            let [position, velocity, acceleration] = sample(&traj, time);
            for dof in 0..3 {
                assert!(
                    velocity[dof].abs() <= input.max_velocity[dof] + 1e-8,
                    "velocity {} of DoF {dof} at t = {time} exceeds {}, input: {input}",
                    velocity[dof],
                    input.max_velocity[dof]
                );
                assert!(
                    acceleration[dof].abs() <= input.max_acceleration[dof] + 1e-8,
                    "acceleration {} of DoF {dof} at t = {time} exceeds {}, input: {input}",
                    acceleration[dof],
                    input.max_acceleration[dof]
                );
                assert!(
                    extrema[dof].min - 1e-8 <= position[dof] && position[dof] <= extrema[dof].max + 1e-8,
                    "position {} of DoF {dof} at t = {time} outside the extrema [{}, {}], input: {input}",
                    position[dof],
                    extrema[dof].min,
                    extrema[dof].max
                );
            }
        }

        // The extrema are attained at their reported times
        for dof in 0..3 {
            let [at_min, _, _] = sample(&traj, extrema[dof].t_min);
            let [at_max, _, _] = sample(&traj, extrema[dof].t_max);
            assert!(
                (at_min[dof] - extrema[dof].min).abs() < 1e-8
                    && (at_max[dof] - extrema[dof].max).abs() < 1e-8,
                "extrema of DoF {dof} are not attained at their times, input: {input}"
            );
        }
    }
    assert_eq!(checked, count, "too few valid random inputs");
}

/// `get_first_time_at_position` returns a time at which the position is
/// actually reached, and no earlier sample has crossed it
#[test]
fn random_position_1_first_time_at_position() {
    let mut fraction = Randomizer::new(Uniform::new(0.05, 0.95), 64);

    run_random::<1>(
        number_of_trajectories(0.25),
        generate_position(65),
        |_, input, traj| {
            let mut ratio = DataArrayOrVec::<f64, 1>::new(None, 0.0);
            fraction.fill(&mut ratio);
            let (p0, pf) = (input.current_position[0], input.target_position[0]);
            let position = p0 + ratio[0] * (pf - p0);

            // The trajectory is continuous from p0 to pf, so it crosses
            let time = traj
                .get_first_time_at_position(0, position)
                .unwrap_or_else(|| panic!("position {position} is never reached, input: {input}"));
            assert!(
                (0.0..=traj.get_duration() + 1e-9).contains(&time),
                "time {time} outside the trajectory, input: {input}"
            );
            let [reached, _, _] = sample(traj, time);
            assert!(
                (reached[0] - position).abs() < 1e-7,
                "position {} at t = {time} instead of {position}, input: {input}",
                reached[0]
            );

            const STEPS: usize = 200;
            let side = (p0 - position).signum();
            for step in 0..STEPS {
                let earlier = time * step as f64 / STEPS as f64;
                let [p, _, _] = sample(traj, earlier);
                assert!(
                    (p[0] - position) * side >= -1e-7,
                    "position {position} is already crossed at t = {earlier} < {time}, input: {input}"
                );
            }
        },
    );
}

/// Limits of machine-tool axes: short moves with high acceleration and jerk
/// limits, whose phases last milliseconds
fn machine_limit_dists() -> [Uniform<f64>; 3] {
    [Uniform::new(0.05, 2.0), Uniform::new(10.0, 5000.0), Uniform::new(1000.0, 200_000.0)]
}

/// The velocity (position interface only), acceleration and jerk of DoF 0
/// stay within their limits for the whole duration
fn check_limits_1(input: &InputParameter<1>, traj: &Trajectory<1>) {
    let min_velocity = input.min_velocity.as_ref().map_or(-input.max_velocity[0], |v| v[0]);
    let min_acceleration =
        input.min_acceleration.as_ref().map_or(-input.max_acceleration[0], |a| a[0]);
    let check_velocity = input.control_interface == ControlInterface::Position;

    // A velocity that is beyond its limits at the start, or driven beyond
    // them by the initial acceleration, is first brought within them: by the
    // brake profile and the beginning of the profile following it
    let brake_duration = traj.get_profiles()[0][0].brake.duration;
    let mut velocity_reached_limits = false;

    const STEPS: usize = 256;
    let duration = traj.get_duration();
    for step in 0..=STEPS {
        let state = traj.state_at(duration * step as f64 / STEPS as f64);
        let (v, a, j) = (state.velocity[0], state.acceleration[0], state.jerk[0]);
        let velocity_within_limits = min_velocity - 1e-9 <= v && v <= input.max_velocity[0] + 1e-9;
        velocity_reached_limits |= velocity_within_limits && state.time >= brake_duration;
        assert!(
            !check_velocity || !velocity_reached_limits || velocity_within_limits,
            "velocity {v} at t = {} exceeds its limits, input: {input}",
            state.time
        );
        assert!(
            min_acceleration - 1e-9 <= a && a <= input.max_acceleration[0] + 1e-9,
            "acceleration {a} at t = {} exceeds its limits, input: {input}",
            state.time
        );
        assert!(
            j.abs() <= input.max_jerk[0] * (1.0 + 1e-12),
            "jerk {j} at t = {} exceeds its limit, input: {input}",
            state.time
        );
    }
}

/// A relative perturbation: none, or of a magnitude between 1e-12 and 1e-6
/// with a random sign
fn perturbation(rng: &mut Pcg64Mcg) -> f64 {
    let uniform = Uniform::new(0.0, 1.0);
    if uniform.sample(rng) < 0.25 {
        return 0.0;
    }
    let magnitude = 10f64.powf(-12.0 + 6.0 * uniform.sample(rng));
    if uniform.sample(rng) < 0.5 { magnitude } else { -magnitude }
}

/// Re-planning to the same target from a state on the final jerk phase of a
/// running profile, exactly or with a small perturbation of the state or the
/// target. The remaining profile is short there, which is numerically
/// demanding for the root finding of step 1, in particular with the limits
/// of a machine tool.
#[test]
fn random_position_1_retarget_in_final_jerk_phase() {
    let mut p = Randomizer::new(Normal::new(0.0, 0.5).unwrap(), 70);
    let mut d = Randomizer::new(dynamic_dist(), 71);
    let mut l = Randomizer::new(limit_dist(), 72);
    let [v_dist, a_dist, j_dist] = machine_limit_dists();
    let mut rng = Pcg64Mcg::new(73);
    let uniform = Uniform::new(0.0, 1.0);

    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, DELTA_TIME);
    let mut traj = Trajectory::new(None);
    let mut input = InputParameter::new(None);
    let mut replanned = Trajectory::new(None);

    let count = number_of_trajectories(2.0);
    let mut checked = 0;
    for case in 0..64 * count {
        if checked == count {
            break;
        }

        p.fill(&mut input.current_position);
        p.fill(&mut input.target_position);
        if case % 2 == 0 {
            // Machine tool limits, starting at rest in most cases
            d.fill_or_zero(&mut input.current_velocity, 0.2);
            d.fill_or_zero(&mut input.current_acceleration, 0.2);
            input.max_velocity[0] = v_dist.sample(&mut rng) + input.current_velocity[0].abs();
            input.max_acceleration[0] = a_dist.sample(&mut rng);
            input.max_jerk[0] = j_dist.sample(&mut rng);
        } else {
            d.fill_or_zero(&mut input.current_velocity, 0.9);
            d.fill_or_zero(&mut input.current_acceleration, 0.8);
            l.fill_above(&mut input.max_velocity, &input.current_velocity);
            l.fill_above(&mut input.max_acceleration, &input.current_acceleration);
            l.fill(&mut input.max_jerk);
        }
        if !check_calculation(&mut otg, &input, &mut traj) {
            continue;
        }

        // The last phase with a jerk
        let profile = &traj.get_profiles()[0][0];
        let Some(phase) = (0..7).rev().find(|&i| profile.t[i] > 0.0 && profile.j[i] != 0.0) else {
            continue;
        };
        let phase_end = profile.brake.duration + profile.accel.duration + profile.t_sum[phase];
        let phase_start = phase_end - profile.t[phase];

        for _ in 0..4 {
            // Anywhere within the phase, or at one of the cycles before its end
            let time = if uniform.sample(&mut rng) < 0.5 {
                phase_start + uniform.sample(&mut rng) * profile.t[phase]
            } else {
                let cycles = (profile.t[phase] / 0.0005).floor();
                phase_end - 0.0005 * (uniform.sample(&mut rng) * cycles).ceil()
            };
            let state = traj.state_at(time.clamp(phase_start, phase_end));

            let mut retarget = input.clone();
            retarget.current_position[0] = state.position[0];
            retarget.current_velocity[0] = state.velocity[0];
            retarget.current_acceleration[0] = state.acceleration[0];

            let factor = 1.0 + perturbation(&mut rng);
            match (uniform.sample(&mut rng) * 4.0) as usize {
                0 => retarget.current_position[0] *= factor,
                1 => retarget.current_velocity[0] *= factor,
                2 => retarget.current_acceleration[0] *= factor,
                _ => retarget.target_position[0] *= factor,
            }

            if check_calculation(&mut otg, &retarget, &mut replanned) {
                check_limits_1(&retarget, &replanned);
            }
        }
        checked += 1;
    }
    assert_eq!(checked, count, "too few valid random inputs");
}

/// Moves from rest to rest over a distance exactly at (or a few doubles or a
/// small perturbation next to) the border between two profile types: where
/// the acceleration limit or the velocity limit is just reached, so that a
/// phase of constant acceleration or velocity vanishes.
#[test]
fn random_position_1_at_profile_borders() {
    let mut p = Randomizer::new(position_dist(), 77);
    let mut l = Randomizer::new(limit_dist(), 78);
    let [v_dist, a_dist, j_dist] = machine_limit_dists();
    let mut rng = Pcg64Mcg::new(79);
    let uniform = Uniform::new(0.0, 1.0);
    let mut case = 0_usize;

    run_random::<1>(
        number_of_trajectories(4.0),
        |input| {
            case += 1;
            if case.is_multiple_of(2) {
                input.max_velocity[0] = v_dist.sample(&mut rng);
                input.max_acceleration[0] = a_dist.sample(&mut rng);
                input.max_jerk[0] = j_dist.sample(&mut rng);
            } else {
                l.fill(&mut input.max_velocity);
                l.fill(&mut input.max_acceleration);
                l.fill(&mut input.max_jerk);
            }
            if uniform.sample(&mut rng) < 0.3 {
                // Round limits, as set by hand
                for limit in [&mut input.max_velocity[0], &mut input.max_acceleration[0], &mut input.max_jerk[0]] {
                    let digit = 10f64.powf(limit.log10().floor());
                    *limit = (*limit / digit).round() * digit;
                }
            }
            let (a, j) = (input.max_acceleration[0], input.max_jerk[0]);
            if uniform.sample(&mut rng) < 0.2 {
                // Both limits are reached at once
                input.max_velocity[0] = a * a / j;
            }
            let v = input.max_velocity[0];

            let distance = if uniform.sample(&mut rng) < 0.5 || a * a / j > v {
                // The velocity limit is just reached, with or without a
                // phase of constant acceleration
                if a * a / j > v { 2.0 * v * (v / j).sqrt() } else { v * (v / a + a / j) }
            } else {
                // The acceleration limit is just reached
                2.0 * a * a * a / (j * j)
            };
            let distance = match (uniform.sample(&mut rng) * 3.0) as usize {
                0 => distance,
                1 => {
                    let ulps = (uniform.sample(&mut rng) * 9.0) as i64 - 4;
                    f64::from_bits((distance.to_bits() as i64 + ulps) as u64)
                }
                _ => distance * (1.0 + perturbation(&mut rng)),
            };
            let direction = if uniform.sample(&mut rng) < 0.5 { 1.0 } else { -1.0 };

            if uniform.sample(&mut rng) < 0.5 {
                input.current_position[0] = 0.0;
            } else {
                p.fill(&mut input.current_position);
            }
            input.target_position[0] = input.current_position[0] + direction * distance;
        },
        |_, input, traj| check_limits_1(input, traj),
    );
}

/// A value with a magnitude drawn uniformly between the powers of ten `low`
/// and `high`
fn log_uniform(rng: &mut Pcg64Mcg, low: f64, high: f64) -> f64 {
    10f64.powf(Uniform::new(low, high).sample(rng))
}

/// `value` rounded to seven significant digits, like a programmed coordinate
fn round_to_7_digits(value: f64) -> f64 {
    format!("{value:.6e}").parse().unwrap()
}

/// Acceleration limits of 10 to 20 000 that differ by a factor of 0.2 to 5
/// in half of the cases, and a jerk limit of 2 to 1000 times the acceleration
fn fill_asymmetric_limits(rng: &mut Pcg64Mcg, input: &mut InputParameter<1>) {
    let uniform = Uniform::new(0.0, 1.0);
    let mut a_max = log_uniform(rng, 1.0, 20_000f64.log10());
    let mut j_max = a_max * log_uniform(rng, 2f64.log10(), 3.0);
    if uniform.sample(rng) < 0.5 {
        // Round limits, as set by hand
        a_max = format!("{a_max:.0e}").parse().unwrap();
        j_max = format!("{j_max:.0e}").parse().unwrap();
    }
    input.max_acceleration[0] = a_max;
    input.max_jerk[0] = j_max;
    input.min_acceleration = if uniform.sample(rng) < 0.5 {
        None
    } else {
        let ratio = log_uniform(rng, 0.2f64.log10(), 5f64.log10());
        let ratio = if uniform.sample(rng) < 0.5 { (ratio * 10.0).round().max(2.0) / 10.0 } else { ratio };
        Some(daov_stack![-a_max * ratio])
    };
}

/// The distances of a move from rest to rest without a velocity limit at
/// which (0) the lower of the acceleration limits `a_max` and `-a_min` and
/// (1) both of them are just reached
fn acceleration_border_distances(a_max: f64, a_min: f64, j_max: f64) -> [f64; 2] {
    let (low, high) = (f64::min(a_max, -a_min), f64::max(a_max, -a_min));
    // The profile to the higher limit and back to the lower one, which is
    // held until the velocity has vanished (or the reverse with equal length)
    let phases = [
        (high / j_max, j_max),
        ((high + low) / j_max, -j_max),
        ((high * high - low * low) / (2.0 * j_max * low), 0.0),
        (low / j_max, j_max),
    ];
    let (mut p, mut v, mut a) = (0.0, 0.0, 0.0);
    for (t, j) in phases {
        p += t * (v + t * (a / 2.0 + t * j / 6.0));
        v += t * (a + t * j / 2.0);
        a += t * j;
    }
    [2.0 * low * low * low / (j_max * j_max), p]
}

/// The longest move of the tests with limits of a machine tool
const MAX_DISTANCE: f64 = 1e5;

/// Moves from rest to rest with different limits for acceleration and
/// deceleration, over distances at and around those at which an acceleration
/// limit is just reached, and from a wide range. At these borders a phase of
/// constant acceleration vanishes, and limits of some thousands have a
/// resolution coarser than the absolute tolerances of the profile check.
#[test]
fn random_position_1_asymmetric_acceleration_limits() {
    let mut p = Randomizer::new(position_dist(), 80);
    let mut rng = Pcg64Mcg::new(81);
    let uniform = Uniform::new(0.0, 1.0);

    run_random::<1>(
        number_of_trajectories(4.0),
        |input| {
            fill_asymmetric_limits(&mut rng, input);
            let (a_max, j_max) = (input.max_acceleration[0], input.max_jerk[0]);
            let a_min = input.min_acceleration.as_ref().map_or(-a_max, |a| a[0]);

            let borders = acceleration_border_distances(a_max, a_min, j_max);
            let border = borders[(uniform.sample(&mut rng) * 2.0) as usize];
            let distance = match (uniform.sample(&mut rng) * 6.0) as usize {
                0 => border,
                1 => {
                    let ulps = (uniform.sample(&mut rng) * 9.0) as i64 - 4;
                    f64::from_bits((border.to_bits() as i64 + ulps) as u64)
                }
                2 => border * (1.0 + perturbation(&mut rng)),
                3 => round_to_7_digits(border),
                4 => round_to_7_digits(borders[0] * (0.1 + 2.9 * uniform.sample(&mut rng))),
                _ => round_to_7_digits(borders[0] * log_uniform(&mut rng, -4.0, 4.0)),
            };
            // The final position is checked to an absolute precision of 1e-8,
            // which positions far beyond this cannot resolve
            let distance = distance.min(MAX_DISTANCE);

            input.max_velocity[0] = match (uniform.sample(&mut rng) * 3.0) as usize {
                // Never reached, reached at once with the acceleration limit, or any
                0 => 1e6,
                1 => f64::min(a_max, -a_min).powi(2) / j_max,
                _ => log_uniform(&mut rng, -1.0, 3.0),
            };

            if uniform.sample(&mut rng) < 0.5 {
                input.current_position[0] = 0.0;
            } else {
                p.fill(&mut input.current_position);
            }
            let direction = if uniform.sample(&mut rng) < 0.5 { 1.0 } else { -1.0 };
            input.target_position[0] = input.current_position[0] + direction * distance;
        },
        |_, input, traj| check_limits_1(input, traj),
    );
}

/// Moves from rest to rest with limits of a machine tool, over a dense series
/// of distances around the one at which the acceleration limit is just
/// reached (2 a^3 / j^2 for the upper limit), without reaching the velocity
/// limit
#[test]
fn position_1_distance_scan_with_asymmetric_acceleration_limits() {
    // a_max, a_min, j_max
    let limits = [
        [20000.0, -8000.0, 100000.0],
        [10000.0, -4000.0, 50000.0],
        [10000.0, -4000.0, 1e6],
        [8000.0, -4000.0, 50000.0],
        [6000.0, -3000.0, 30000.0],
        [5000.0, -2000.0, 50000.0],
        [4000.0, -2000.0, 20000.0],
        [1000.0, -400.0, 5000.0],
        [4000.0, -10000.0, 50000.0],
    ];
    let count = number_of_trajectories(0.25);

    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, DELTA_TIME);
    let mut traj = Trajectory::new(None);
    let mut input = InputParameter::new(None);
    input.max_velocity[0] = 1e6;

    for [a_max, a_min, j_max] in limits {
        input.max_acceleration[0] = a_max;
        input.min_acceleration = Some(daov_stack![a_min]);
        input.max_jerk[0] = j_max;

        let border = 2.0 * a_max * a_max * a_max / (j_max * j_max);
        for step in 0..count {
            let factor = 0.1 + 2.9 * step as f64 / count as f64;
            input.target_position[0] = round_to_7_digits(factor * border);
            assert!(check_calculation(&mut otg, &input, &mut traj), "input: {input}");
            check_limits_1(&input, &traj);
        }
    }
}

/// Re-planning from a state of a running profile, in particular from its
/// first phase with a rising acceleration and from the cycle at which the
/// acceleration limit is reached: to the same or a new target, to the
/// position of the fastest stop (a feed hold), and with a lowered velocity
/// limit. The profile then ends in or begins with a vanishing phase, and
/// reaches the acceleration limit of some thousands by construction.
#[test]
fn random_position_1_retarget_from_running_profile() {
    let mut rng = Pcg64Mcg::new(83);
    let uniform = Uniform::new(0.0, 1.0);

    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, DELTA_TIME);
    let mut traj = Trajectory::new(None);
    let mut input = InputParameter::new(None);
    let mut stop = Trajectory::new(None);
    let mut replanned = Trajectory::new(None);

    let count = number_of_trajectories(2.0);
    let mut checked = 0;
    for _ in 0..64 * count {
        if checked == count {
            break;
        }

        fill_asymmetric_limits(&mut rng, &mut input);
        input.max_velocity[0] = log_uniform(&mut rng, -1.0, 3.0);
        input.current_position[0] = if uniform.sample(&mut rng) < 0.5 {
            0.0
        } else {
            log_uniform(&mut rng, -1.0, 2.0) * if uniform.sample(&mut rng) < 0.5 { 1.0 } else { -1.0 }
        };
        let direction = if uniform.sample(&mut rng) < 0.5 { 1.0 } else { -1.0 };
        let distance = round_to_7_digits(log_uniform(&mut rng, -3.0, 3.0));
        input.target_position[0] = input.current_position[0] + direction * distance;
        if !check_calculation(&mut otg, &input, &mut traj) {
            continue;
        }
        let duration = traj.get_duration();

        // The first phase with a jerk
        let profile = &traj.get_profiles()[0][0];
        let Some(phase) = (0..7).find(|&i| profile.t[i] > 0.0 && profile.j[i] != 0.0) else {
            continue;
        };
        let phase_start = profile.t_sum[phase] - profile.t[phase];
        let phase_end = profile.t_sum[phase];

        for _ in 0..4 {
            let cycle = if uniform.sample(&mut rng) < 0.5 { 0.001 } else { 0.0005 };
            let time = match (uniform.sample(&mut rng) * 5.0) as usize {
                0 => phase_start + uniform.sample(&mut rng) * profile.t[phase],
                1 => phase_end,
                // The cycles of the phase and the one following it
                2 => cycle * (uniform.sample(&mut rng) * (phase_end / cycle + 1.0)).ceil(),
                3 => cycle * (phase_end / cycle).ceil(),
                _ => uniform.sample(&mut rng) * duration,
            };
            let state = traj.state_at(time.min(duration));

            let mut retarget = input.clone();
            retarget.current_position[0] = state.position[0];
            retarget.current_velocity[0] = state.velocity[0];
            retarget.current_acceleration[0] = state.acceleration[0];

            let variant = (uniform.sample(&mut rng) * 6.0) as usize;
            if variant == 1 || variant == 2 {
                // The position of the fastest stop
                let mut stop_input = retarget.clone();
                stop_input.control_interface = ControlInterface::Velocity;
                if !check_calculation(&mut otg, &stop_input, &mut stop) {
                    continue;
                }
                let stop_position = stop.state_at(stop.get_duration()).position[0];
                retarget.target_position[0] = if variant == 1 {
                    stop_position
                } else {
                    stop_position + (stop_position - retarget.current_position[0]) * perturbation(&mut rng)
                };
            } else if variant == 3 {
                // A new target, ahead or behind
                let distance = round_to_7_digits(log_uniform(&mut rng, -3.0, 3.0));
                let direction = if uniform.sample(&mut rng) < 0.7 { direction } else { -direction };
                retarget.target_position[0] = retarget.current_position[0] + direction * distance;
            }
            if variant >= 4 || uniform.sample(&mut rng) < 0.2 {
                // A lowered velocity limit, also below the current velocity
                retarget.max_velocity[0] *= 0.05 + 0.95 * uniform.sample(&mut rng);
            }

            if check_calculation(&mut otg, &retarget, &mut replanned) {
                check_limits_1(&retarget, &replanned);
            }
        }
        checked += 1;
    }
    assert_eq!(checked, count, "too few valid random inputs");
}

/// Velocity interface starting exactly at an acceleration limit or just
/// inside of it, also for asymmetric limits: the boundary between the
/// profiles with and without an acceleration plateau.
#[test]
fn random_velocity_1_at_acceleration_limit() {
    let mut d = Randomizer::new(dynamic_dist(), 74);
    let mut l = Randomizer::new(limit_dist(), 75);
    let [_, a_dist, j_dist] = machine_limit_dists();
    let mut rng = Pcg64Mcg::new(76);
    let uniform = Uniform::new(0.0, 1.0);
    let velocity_scale = Uniform::new(-1.5, 1.5);
    let mut case = 0_usize;

    run_random::<1>(
        number_of_trajectories(4.0),
        |input| {
            case += 1;
            input.control_interface = ControlInterface::Velocity;
            d.fill_or_zero(&mut input.target_velocity, 0.5);
            d.fill_or_zero(&mut input.target_acceleration, 0.3);
            if case.is_multiple_of(2) {
                input.max_acceleration[0] = a_dist.sample(&mut rng) + input.target_acceleration[0].abs();
                input.max_jerk[0] = j_dist.sample(&mut rng);
            } else {
                l.fill_above(&mut input.max_acceleration, &input.target_acceleration);
                l.fill(&mut input.max_jerk);
            }
            let (a_max, j_max) = (input.max_acceleration[0], input.max_jerk[0]);

            let a_min = if uniform.sample(&mut rng) < 0.5 {
                let a_min = -(0.1 + 1.9 * uniform.sample(&mut rng)) * a_max
                    - input.target_acceleration[0].abs();
                input.min_acceleration = Some(daov_stack![a_min]);
                a_min
            } else {
                input.min_acceleration = None;
                -a_max
            };

            // Exactly at a limit, the adjacent double or slightly inside
            let limit = if uniform.sample(&mut rng) < 0.5 { a_max } else { a_min };
            input.current_acceleration[0] = match (uniform.sample(&mut rng) * 3.0) as usize {
                0 => limit,
                1 => f64::from_bits(limit.to_bits() - 1),
                _ => limit * (1.0 - perturbation(&mut rng).abs()),
            };

            // A random velocity difference, or the one of the profile that
            // just touches the opposite acceleration limit, where the
            // acceleration plateau vanishes
            let other = if limit == a_max { a_min } else { a_max };
            let af = input.target_acceleration[0];
            let ramp = (a_max - a_min).powi(2) / (2.0 * j_max);
            let difference = match (uniform.sample(&mut rng) * 3.0) as usize {
                0 => velocity_scale.sample(&mut rng) * ramp,
                1 => limit.signum() * (limit * limit - af * af) / (2.0 * j_max),
                _ => limit.signum() * (limit * limit - 2.0 * other * other + af * af) / (2.0 * j_max),
            };
            input.current_velocity[0] =
                input.target_velocity[0] - difference * (1.0 + perturbation(&mut rng));
        },
        |_, input, traj| check_limits_1(input, traj),
    );
}

/// Stepping through a trajectory with `update` reaches the target, and
/// re-planning from the intermediate states succeeds (cf. upstream
/// `step_through_and_check_calculation`).
///
/// A state sampled from a profile lies exactly on a switching surface of the
/// time-optimal control, which makes re-planning to the same target
/// numerically degenerate: rounding decides whether the solver finds the
/// remainder of the original profile, a longer profile or - rarely - none at
/// all. Upstream Ruckig behaves the same way in these cases. The re-planned
/// duration is therefore not compared with the remainder of the original
/// plan, and a small rate of failed re-plans is tolerated.
#[test]
fn random_step_through_3() {
    let mut generate = generate_position::<3>(66);

    let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, DELTA_TIME);
    let mut traj = Trajectory::new(None);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);
    let mut replanner = Ruckig::<3, ThrowErrorHandler>::new(None, DELTA_TIME);
    let mut replanned = Trajectory::new(None);

    let (mut replans, mut failed_replans) = (0_usize, 0_usize);
    let mut last_failure = String::new();

    let count = number_of_trajectories(1.0 / 64.0);
    let mut checked = 0;
    for _ in 0..256 * count {
        if checked == count {
            break;
        }
        generate(&mut input);
        if !check_calculation(&mut otg, &input, &mut traj) || traj.get_duration() > 60.0 {
            continue;
        }
        checked += 1;

        let duration = traj.get_duration();
        let max_steps = (duration / DELTA_TIME).ceil() as usize + 2;
        let mut stepped = input.clone();
        otg.reset();

        let mut steps = 0;
        loop {
            let result = otg
                .update(&stepped, &mut output)
                .unwrap_or_else(|error| panic!("update failed: {error}\ninput: {stepped}"));
            if result == RuckigResult::Finished {
                break;
            }
            assert_eq!(result, RuckigResult::Working, "input: {stepped}");
            output.pass_to_input(&mut stepped);

            steps += 1;
            assert!(
                steps <= max_steps,
                "not finished after {steps} steps for a duration of {duration}, input: {input}"
            );

            if steps % 8 == 0 && replanner.validate_input(&stepped, false, true).is_ok() {
                dump_for_oracle(&stepped);
                replans += 1;
                match replanner.calculate(&stepped, &mut replanned) {
                    Ok(RuckigResult::Working) => assert!(
                        replanned.get_duration().is_finite() && replanned.get_duration() >= 0.0,
                        "re-planned duration {}, input: {stepped}",
                        replanned.get_duration()
                    ),
                    Ok(RuckigResult::ErrorTrajectoryDuration) => {}
                    failure => {
                        failed_replans += 1;
                        last_failure = format!("{failure:?}");
                    }
                }
            }
        }

        // The final cycle samples past the end of the trajectory, where the
        // target state is extrapolated with constant acceleration
        let t_past = output.time - duration;
        assert!((0.0..=DELTA_TIME).contains(&t_past), "finished at {}", output.time);
        for dof in 0..3 {
            let expected = input.target_position[dof]
                + t_past * (input.target_velocity[dof] + 0.5 * t_past * input.target_acceleration[dof]);
            assert!(
                (output.new_position[dof] - expected).abs() < 1e-6,
                "DoF {dof} finished at {} instead of {expected}, input: {input}",
                output.new_position[dof]
            );
        }
    }
    assert_eq!(checked, count, "too few valid random inputs");

    // At most one failure per thousand re-plans
    eprintln!("{failed_replans} of {replans} re-plans failed");
    assert!(
        1000 * failed_replans <= replans,
        "{failed_replans} of {replans} re-plans failed, e.g. with {last_failure}"
    );
}
