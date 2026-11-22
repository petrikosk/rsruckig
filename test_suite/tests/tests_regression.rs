//! Regression tests for fixed defects. The expected durations of the
//! trajectory cases were calculated with upstream Ruckig.

// The inputs are printed with 17 significant digits to reproduce their bits
#![allow(clippy::excessive_precision)]

use float_eq::assert_float_eq;
use rsruckig::prelude::*;
use rsruckig::profile::{ControlSigns, Profile, ReachedLimits};
use rsruckig::roots;

fn sample<const DOF: usize>(traj: &Trajectory<DOF>, time: f64) -> DataArrayOrVec<f64, DOF> {
    let mut position = DataArrayOrVec::new(None, 0.0);
    traj.at_time(time, &mut Some(&mut position), &mut None, &mut None, &mut None, &mut None);
    position
}

fn calculate<const DOF: usize>(input: &InputParameter<DOF>) -> Trajectory<DOF> {
    let mut otg = Ruckig::<DOF, ThrowErrorHandler>::new(None, 0.005);
    let mut traj = Trajectory::new(None);
    let result = otg.calculate(input, &mut traj).unwrap();
    assert_eq!(result, RuckigResult::Working);
    traj
}

/// A cubic without constant term has the root 0 and the roots of the deflated
/// quadratic (the constant term was dropped instead of the leading one)
#[test]
fn test_solve_cub_without_constant_term() {
    // x^3 - 3 x^2 + 2 x = x (x - 1) (x - 2)
    let roots = roots::solve_cub(1.0, -3.0, 2.0, 0.0);
    let mut positive: Vec<f64> = roots.get_data().iter().copied().filter(|&x| x > 0.0).collect();
    positive.sort_by(f64::total_cmp);
    assert_eq!(positive.len(), 2, "roots: {positive:?}");
    assert_float_eq!(positive[0], 1.0, abs <= 1e-12);
    assert_float_eq!(positive[1], 2.0, abs <= 1e-12);

    // 2 x^2 - 4 x = 2 x (x - 2): deflates to a linear equation
    let roots = roots::solve_cub(0.0, 2.0, -4.0, 0.0);
    assert!(roots.get_data().iter().any(|&x| (x - 2.0).abs() < 1e-12));
}

/// The last jerk phase of the UDUD pattern is negative, also for the velocity interface
#[test]
fn test_velocity_check_udud_jerk_signs() {
    let mut profile = Profile {
        t: [1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0],
        ..Default::default()
    };
    profile.check_for_velocity(ControlSigns::UDUD, ReachedLimits::None, 2.0, 10.0, -10.0);
    assert_eq!(profile.j, [2.0, 0.0, -2.0, 0.0, 2.0, 0.0, -2.0]);

    profile.check_for_velocity(ControlSigns::UDDU, ReachedLimits::None, 2.0, 10.0, -10.0);
    assert_eq!(profile.j, [2.0, 0.0, -2.0, 0.0, -2.0, 0.0, 2.0]);
}

/// Velocity interface with an initial acceleration beyond its limit: the
/// brake pre-trajectory must end safely inside the limit
#[test]
fn test_velocity_interface_acceleration_brake() {
    let mut input = InputParameter::<3>::new(None);
    input.control_interface = ControlInterface::Velocity;
    input.current_position = daov_stack![-2.866560865882922, 0.48480073665477647, 1.0506647296170006];
    input.current_velocity = daov_stack![0.0, -0.312701250823189, -0.08897432526917481];
    input.current_acceleration = daov_stack![0.0, -0.04797992785420745, 1.252510306574836];
    input.target_velocity = daov_stack![0.315370701768807, -0.4992401679937798, 1.4001686306280658];
    input.max_acceleration = daov_stack![1.1276702889714223, 11.27413561925475, 0.1659010059929173];
    input.max_jerk = daov_stack![0.6039483695753376, 0.47951187642586424, 0.6390225402682774];

    assert_float_eq!(calculate(&input).get_duration(), 3.5372299356177779, abs <= 1e-12);
}

/// Second-order profile with an initial velocity beyond its limit
#[test]
fn test_second_order_velocity_brake() {
    let mut input = InputParameter::<3>::new(None);
    input.current_position = daov_stack![0.08347974172923422, -6.558826224335791, -6.7418385465902];
    input.current_velocity = daov_stack![-0.2411799879386312, -0.034116570296812664, 2.855580160997743];
    input.target_position = daov_stack![-1.1266003934530344, -1.3145055626223274, -3.0585178158457333];
    input.target_velocity = daov_stack![0.32842739635578494, 0.0, 0.2720498083784748];
    input.max_velocity = daov_stack![1.9074400463443275, 0.525065477770936, 0.7079045491504192];
    input.max_acceleration = daov_stack![2.3767554525860652, 12.337676829261184, 1.357061170704446];
    input.max_jerk = daov_stack![f64::INFINITY, f64::INFINITY, f64::INFINITY];

    assert_float_eq!(calculate(&input).get_duration(), 10.033349481712822, abs <= 1e-12);
}

/// Two DoFs whose minimum durations differ only in the last bits: the
/// synchronization of the faster one is degenerate for step 2
#[test]
fn test_synchronization_of_nearly_equal_durations() {
    let mut input = InputParameter::<3>::new(None);
    input.current_position = daov_stack![-1.0, -1.0, 1.0];
    input.current_velocity = daov_stack![-1.0, 0.0, -1.0];
    input.current_acceleration = daov_stack![0.0, 1.0, 1.0];
    input.target_position = daov_stack![-1.0, 1.0, -1.0];
    input.target_velocity = daov_stack![1.0, 1.0, 0.0];
    input.target_acceleration = daov_stack![0.0, 1.0, 1.0];
    input.max_velocity = daov_stack![2.0, 1.0, 2.0];
    input.max_acceleration = daov_stack![1.0, 1.0, 1.0];
    input.max_jerk = daov_stack![1.0, 1.0, 1.0];

    let traj = calculate(&input);
    assert_float_eq!(traj.get_duration(), 3.3416366446808046, abs <= 1e-12);
    let position = sample(&traj, traj.get_duration());
    for dof in 0..3 {
        assert_float_eq!(position[dof], input.target_position[dof], abs <= 1e-8);
    }
}

/// A state sampled while cruising exceeds the velocity limit by rounding;
/// the brake pre-trajectory of a few ulps must not break the synchronization
#[test]
fn test_synchronization_with_infinitesimal_brake() {
    let mut input = InputParameter::<3>::new(None);
    input.current_position = daov_stack![1.2233536992094485, 6.4876340599369877, -3.4913269757813739];
    input.current_velocity = daov_stack![-2.2338053424365643, 1.9606462295823273, -3.8344946599819454];
    input.current_acceleration = daov_stack![0.0, -0.5252970855026478, 0.0];
    input.target_position = daov_stack![0.1779117856458829, 6.5990856786088523, -5.8251910550157380];
    input.target_velocity = daov_stack![0.9583851588189554, -1.6147488204718876, -0.3497565078213620];
    input.target_acceleration = daov_stack![0.0, 0.0, -0.3152793345248101];
    input.max_velocity = daov_stack![2.2338053424365638, 16.8954568878107061, 7.9178969277331328];
    input.max_acceleration = daov_stack![6.2156242406609357, 14.4029744923713281, 8.9232215979430602];
    input.max_jerk = daov_stack![15.4543550104327156, 10.4385065718700343, 12.0359022234274029];

    let traj = calculate(&input);
    assert_float_eq!(traj.get_duration(), 1.1223423116295679, abs <= 1e-12);
    let position = sample(&traj, traj.get_duration());
    for dof in 0..3 {
        assert_float_eq!(position[dof], input.target_position[dof], abs <= 1e-8);
    }
}

/// Position extrema inside phases of constant acceleration (no jerk) are found
#[test]
fn test_position_extremum_in_constant_acceleration_phase() {
    let mut input = InputParameter::<1>::new(None);
    input.current_velocity = daov_stack![1.0];
    input.target_velocity = daov_stack![-1.0];
    input.max_velocity = daov_stack![2.0];
    input.max_acceleration = daov_stack![1.0];
    input.max_jerk = daov_stack![f64::INFINITY];

    // Decelerates with -1 for 2 s: turns around at t = 1, p = 0.5
    let mut traj = calculate(&input);
    assert_float_eq!(traj.get_duration(), 2.0, abs <= 1e-12);
    let extrema = traj.get_position_extrema()[0];
    assert_float_eq!(extrema.max, 0.5, abs <= 1e-12);
    assert_float_eq!(extrema.t_max, 1.0, abs <= 1e-12);
    assert_float_eq!(extrema.min, 0.0, abs <= 1e-12);
}

/// The times of position extrema in later sections are trajectory times
#[test]
fn test_position_extrema_times_across_sections() {
    let mut input = InputParameter::<1>::with_waypoints(None, 2);
    input.intermediate_positions.push(daov_stack![1.0]);
    input.intermediate_positions.push(daov_stack![-1.0]);
    input.max_velocity = daov_stack![1.0];
    input.max_acceleration = daov_stack![2.0];
    input.max_jerk = daov_stack![10.0];

    let mut otg = Ruckig::<1, ThrowErrorHandler>::with_waypoints(None, 0.005, 2);
    let mut traj = Trajectory::new(None);
    assert_eq!(otg.calculate(&input, &mut traj).unwrap(), RuckigResult::Working);

    let extrema = traj.get_position_extrema()[0];
    assert!(extrema.max >= 1.0 - 1e-9 && extrema.min <= -1.0 + 1e-9);
    assert!(extrema.t_min > extrema.t_max, "the minimum follows the maximum");
    assert_float_eq!(sample(&traj, extrema.t_max)[0], extrema.max, abs <= 1e-9);
    assert_float_eq!(sample(&traj, extrema.t_min)[0], extrema.min, abs <= 1e-9);
}

/// The first time at a position accounts for the brake pre-trajectory
#[test]
fn test_first_time_at_position_with_brake() {
    let mut input = InputParameter::<1>::new(None);
    input.current_velocity = daov_stack![3.0];
    input.target_position = daov_stack![10.0];
    input.max_velocity = daov_stack![1.0];
    input.max_acceleration = daov_stack![1.0];
    input.max_jerk = daov_stack![1.0];

    let traj = calculate(&input);
    // Inside the brake pre-trajectory, and after it
    for position in [0.5, 2.0, 5.0, 9.5] {
        let time = traj
            .get_first_time_at_position(0, position)
            .unwrap_or_else(|| panic!("position {position} is not reached"));
        assert_float_eq!(sample(&traj, time)[0], position, abs <= 1e-9);
    }
    assert!(traj.get_first_time_at_position(0, 0.5).unwrap() < 0.5);
    assert_eq!(traj.get_first_time_at_position(0, -1.0), None);
}

/// The initial state of the second DoF lies exactly on a switching curve: a
/// single jerk phase of 0.3215 s reaches the target, the next feasible
/// duration is 0.6056 s. Synchronizing with the slightly slower third DoF
/// must skip the infeasible range between the two.
#[test]
fn test_synchronization_with_isolated_extremal_profile() {
    let mut input = InputParameter::<3>::new(None);
    input.current_position = daov_stack![1.4131319213654534, 1.7791465567116844, -2.886982545477548];
    input.current_velocity = daov_stack![0.38227470307136846, 0.33584809590456144, 0.1992866547691332];
    input.current_acceleration = daov_stack![-1.6801503532462185, -1.769259717756475, -1.2398678463518704];
    input.target_position = daov_stack![1.4572689120656497, 1.820649664751985, -2.8656280272596435];
    input.target_acceleration = daov_stack![0.3798149601467334, -0.3202291938465538, 0.0];
    input.max_velocity = daov_stack![14.462860289946967, 4.039598426155517, 10.393540541347303];
    input.max_acceleration = daov_stack![1.6801503532462185, 14.226168465481363, 6.274828194600925];
    input.max_jerk = daov_stack![13.442807625093192, 4.507593238141367, 3.856937330294652];

    let traj = calculate(&input);
    assert_float_eq!(traj.get_duration(), 0.60563301856886054, abs <= 1e-12);
    let position = sample(&traj, traj.get_duration());
    for dof in 0..3 {
        assert_float_eq!(position[dof], input.target_position[dof], abs <= 1e-8);
    }
}

/// Batch sampling agrees with `at_time`, also across sections, past the end
/// and for a runtime number of DoFs
#[test]
fn test_at_times_matches_at_time() {
    let mut input = InputParameter::<0>::with_waypoints(Some(2), 1);
    input.intermediate_positions.push(daov_heap![1.0, -0.5]);
    input.target_position = daov_heap![0.0, 2.0];
    input.max_velocity = daov_heap![1.0, 1.5];
    input.max_acceleration = daov_heap![2.0, 2.0];
    input.max_jerk = daov_heap![10.0, 8.0];

    let mut otg = Ruckig::<0, ThrowErrorHandler>::with_waypoints(Some(2), 0.005, 1);
    let mut traj = Trajectory::new(Some(2));
    assert_eq!(otg.calculate(&input, &mut traj).unwrap(), RuckigResult::Working);

    let duration = traj.get_duration();
    let times: Vec<f64> = (0..=24).map(|i| duration * f64::from(i) / 20.0).collect();
    let states: Vec<KinematicState<0>> = traj.at_times(&times).collect();
    assert_eq!(states.len(), times.len());

    for (state, &time) in states.iter().zip(&times) {
        let mut position = DataArrayOrVec::new(Some(2), 0.0);
        let mut velocity = DataArrayOrVec::new(Some(2), 0.0);
        let mut acceleration = DataArrayOrVec::new(Some(2), 0.0);
        let mut jerk = DataArrayOrVec::new(Some(2), 0.0);
        let mut section = None;
        traj.at_time(
            time,
            &mut Some(&mut position),
            &mut Some(&mut velocity),
            &mut Some(&mut acceleration),
            &mut Some(&mut jerk),
            &mut section,
        );
        assert_eq!(state.time, time);
        assert_eq!(state.position, position);
        assert_eq!(state.velocity, velocity);
        assert_eq!(state.acceleration, acceleration);
        assert_eq!(state.jerk, jerk);
        assert_eq!(Some(state.section), section);
    }
    assert_eq!(states[0].section, 0);
    assert_eq!(states[24].section, 2, "past the end");
    assert_eq!(traj.state_at(duration).position, states[20].position);
}

/// The velocity (position interface only), acceleration and jerk of DoF 0
/// stay within their limits for the whole duration
fn assert_within_limits(input: &InputParameter<1>, traj: &Trajectory<1>) {
    const STEPS: usize = 4096;
    let duration = traj.get_duration();
    for step in 0..=STEPS {
        let state = traj.state_at(duration * step as f64 / STEPS as f64);
        let (t, v, a, j) = (state.time, state.velocity[0], state.acceleration[0], state.jerk[0]);
        if input.control_interface == ControlInterface::Position {
            assert!(v.abs() <= input.max_velocity[0] + 1e-9, "velocity {v} at t = {t}");
        }
        assert!(a.abs() <= input.max_acceleration[0] + 1e-9, "acceleration {a} at t = {t}");
        assert!(j.abs() <= input.max_jerk[0] * (1.0 + 1e-12), "jerk {j} at t = {t}");
    }
}

/// A quartic with roots far below one has coefficients below the machine
/// epsilon that are not negligible: x^4 + b x^2 + c x + d with a root at 5e-5
#[test]
fn test_solve_quart_with_small_roots() {
    let polynom = [0.0, 4.235308885508857e-8, 1.63463932024377e-16, -1.1211150847294935e-16];
    let value = |x: f64| ((x * x + polynom[1]) * x + polynom[2]) * x + polynom[3];

    let roots = roots::solve_quart_monic_scaled(&polynom);
    let positive: Vec<f64> = roots.get_data().iter().copied().filter(|&x| x > 0.0).collect();
    assert_eq!(positive.len(), 1, "roots: {positive:?}");
    assert_float_eq!(positive[0], 5.0e-5, abs <= 1e-6);
    assert!(value(positive[0]).abs() < 1e-28, "residual {}", value(positive[0]));

    // Roots of order one are not rescaled: x (x - 1) (x - 2) (x - 3)
    let polynom = [-6.0, 11.0, -6.0, 0.0];
    let mut scaled: Vec<f64> = roots::solve_quart_monic_scaled(&polynom).get_data().to_vec();
    let mut plain: Vec<f64> = roots::solve_quart_monic_arr(&polynom).get_data().to_vec();
    scaled.sort_by(f64::total_cmp);
    plain.sort_by(f64::total_cmp);
    assert_eq!(scaled, plain);
}

/// Re-planning from a state close to the final jerk phase of a running
/// profile with high limits: the remaining profile takes 4.4 ms, so that the
/// coefficients of the quartic for its durations fall below the machine
/// epsilon and its root was dropped (step 1 failed, as in upstream Ruckig)
#[test]
fn test_position_retarget_in_final_jerk_phase() {
    let mut input = InputParameter::<1>::new(None);
    input.current_position = daov_stack![0.1976052267223661];
    input.current_velocity = daov_stack![0.1886795880329236];
    input.current_acceleration = daov_stack![-86.8257617504501127];
    input.target_position = daov_stack![0.1978788770940445];
    input.max_velocity = daov_stack![0.4089533144682651];
    input.max_acceleration = daov_stack![1000.0];
    input.max_jerk = daov_stack![20000.0];

    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.0005);
    let mut traj = Trajectory::new(None);
    assert_eq!(otg.calculate(&input, &mut traj).unwrap(), RuckigResult::Working);

    let duration = traj.get_duration();
    assert_float_eq!(duration, 0.004441275675214965, abs <= 1e-12);

    let start = traj.state_at(0.0);
    assert_float_eq!(start.position[0], input.current_position[0], abs <= 1e-12);
    assert_float_eq!(start.velocity[0], input.current_velocity[0], abs <= 1e-12);
    assert_float_eq!(start.acceleration[0], input.current_acceleration[0], abs <= 1e-9);

    let end = traj.state_at(duration);
    assert_float_eq!(end.position[0], input.target_position[0], abs <= 1e-9);
    assert_float_eq!(end.velocity[0], 0.0, abs <= 1e-9);
    assert_float_eq!(end.acceleration[0], 0.0, abs <= 1e-9);

    assert_within_limits(&input, &traj);
}

/// Velocity interface starting exactly at the acceleration limit, with a
/// velocity difference for which the acceleration just touches the opposite
/// limit: the peak of the profile without an acceleration plateau exceeded
/// the limit by rounding, while the plateau was too short to count (step 1
/// failed, as in upstream Ruckig)
#[test]
fn test_velocity_at_acceleration_limit() {
    let mut input = InputParameter::<1>::new(None);
    input.control_interface = ControlInterface::Velocity;
    input.current_velocity = daov_stack![45.0000000000000639];
    input.current_acceleration = daov_stack![3000.0];
    input.max_velocity = daov_stack![200.0];
    input.max_acceleration = daov_stack![3000.0];
    input.max_jerk = daov_stack![100000.0];

    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.001);
    let mut traj = Trajectory::new(None);
    assert_eq!(otg.calculate(&input, &mut traj).unwrap(), RuckigResult::Working);

    // From 3000 to -3000 and back to zero with the maximal jerk
    let duration = traj.get_duration();
    assert_float_eq!(duration, 0.09, abs <= 1e-12);

    let start = traj.state_at(0.0);
    assert_float_eq!(start.velocity[0], input.current_velocity[0], abs <= 1e-12);
    assert_float_eq!(start.acceleration[0], input.current_acceleration[0], abs <= 1e-9);

    let end = traj.state_at(duration);
    assert_float_eq!(end.velocity[0], 0.0, abs <= 1e-9);
    assert_float_eq!(end.acceleration[0], 0.0, abs <= 1e-9);

    assert_within_limits(&input, &traj);
}

/// Velocity interface with asymmetric acceleration limits of some thousands:
/// integrating the first phase missed the opposite limit by more than the
/// absolute tolerance (step 1 failed, as in upstream Ruckig)
#[test]
fn test_velocity_at_asymmetric_acceleration_limit() {
    let mut input = InputParameter::<1>::new(None);
    input.control_interface = ControlInterface::Velocity;
    input.current_velocity = daov_stack![820.8709466237777406];
    input.current_acceleration = daov_stack![2669.4558889757258839];
    input.target_velocity = daov_stack![-0.4575900641514823];
    input.max_acceleration = daov_stack![2669.4558889757258839];
    input.min_acceleration = Some(daov_stack![-4575.9864112724844745]);
    input.max_jerk = daov_stack![21156.7643019730021479];

    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.001);
    let mut traj = Trajectory::new(None);
    assert_eq!(otg.calculate(&input, &mut traj).unwrap(), RuckigResult::Working);

    let duration = traj.get_duration();
    let end = traj.state_at(duration);
    assert_float_eq!(end.velocity[0], input.target_velocity[0], abs <= 1e-9);
    assert_float_eq!(end.acceleration[0], 0.0, abs <= 1e-9);

    const STEPS: usize = 4096;
    for step in 0..=STEPS {
        let state = traj.state_at(duration * step as f64 / STEPS as f64);
        let (a, j) = (state.acceleration[0], state.jerk[0]);
        assert!((-4575.9864112724844745 - 1e-9..=2669.4558889757258839 + 1e-9).contains(&a), "acceleration {a}");
        assert!(j.abs() <= input.max_jerk[0] * (1.0 + 1e-12), "jerk {j}");
    }
}
