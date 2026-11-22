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

/// Moves from rest to rest over a distance at which the acceleration just
/// touches its limits (2 a^3 / j^2), partly together with the velocity limit
/// (a^2 / j): the peak of the profile without phases of constant acceleration
/// exceeded the limit by rounding, while these phases were too short to
/// count (step 1 failed, as in upstream Ruckig)
#[test]
fn test_position_touching_acceleration_limits() {
    // p0, pf, v_max, a_max, j_max, duration
    let cases = [
        [0.0, 5.4, 200.0, 3000.0, 100000.0, 0.12],
        [0.0, 5.400000000000001, 200.0, 3000.0, 100000.0, 0.12],
        [0.0, 5.4000000000000066, 200.0, 3000.0, 100000.0, 0.12],
        [0.0, 5.4, 111.94384625378346, 3000.0, 100000.0, 0.12],
        [1.4605632494715692, 6.8605632494715758, 111.94384625378346, 3000.0, 100000.0, 0.12],
        [0.0, 10.000000000000004, 50.0, 500.0, 5000.0, 0.4],
        [4.2315545023694767, 14.2315545023694803, 50.0, 500.0, 5000.0, 0.4],
        [0.0, 10.0, 50.0, 500.0, 5000.0, 0.4],
        [0.0, 9.999999999999998, 50.0, 500.0, 5000.0, 0.4],
        // Limits whose resolution is coarser than the absolute tolerances
        [-4.0485948169797803, 35480.053123373138, 7738.8155345466857, 3375.5548528957256, 1472.3662185824835, 9.170421897197647],
    ];

    for [p0, pf, v_max, a_max, j_max, expected] in cases {
        let mut input = InputParameter::<1>::new(None);
        input.current_position = daov_stack![p0];
        input.target_position = daov_stack![pf];
        input.max_velocity = daov_stack![v_max];
        input.max_acceleration = daov_stack![a_max];
        input.max_jerk = daov_stack![j_max];

        let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.001);
        let mut traj = Trajectory::new(None);
        let result = otg.calculate(&input, &mut traj);
        assert_eq!(result.map_err(|e| e.to_string()), Ok(RuckigResult::Working), "to {pf}");

        let duration = traj.get_duration();
        assert_float_eq!(duration, expected, abs <= 1e-12);

        let end = traj.state_at(duration);
        assert_float_eq!(end.position[0], pf, abs <= 1e-9);
        assert_float_eq!(end.velocity[0], 0.0, abs <= 1e-9);
        assert_float_eq!(end.acceleration[0], 0.0, abs <= 1e-9);

        assert_within_limits(&input, &traj);
    }
}

/// Moves of rscnc that failed in step 1 by rounding only (most of them fail
/// in upstream Ruckig as well; the moves from rest with a deceleration limit
/// of some thousands succeed there):
/// - from rest to rest with different limits for acceleration and
///   deceleration, over a distance at which the lower one is just reached,
/// - from rest to rest with limits of some thousands, whose resolution is
///   coarser than the absolute tolerance of the profile check, and
/// - re-planned from a state of a running profile, where the acceleration
///   limit is reached by construction, a phase vanishes at the end, or the
///   state after the brake profile is at the velocity limit.
#[test]
fn test_position_at_profile_borders_and_high_limits() {
    // p0, v0, a0, pf, v_max, a_max, a_min, j_max
    let none = f64::NAN;
    #[rustfmt::skip]
    let cases = [
        [0.3305072073372264, 0.0, 0.0, 0.7564272073372305, 69.8679531783741936, 440.0, -1000.0, 20000.0],
        [25.2038415752106921, 0.0, 0.0, 25.2646765752106930, 88.0207976759676569, 350.0, -230.0, 20000.0],
        [9.5139315860761542, 0.0, 0.0, 9.7139315860761588, 19.2694229919122115, 3000.0, -1000.0, 100000.0],
        [0.0, 0.0, 0.0, 80.464, 1.0e6, 10000.0, -4000.0, 50000.0],
        [0.0, 0.0, 0.0, 86.264, 1.0e6, 10000.0, -4000.0, 50000.0],
        [0.0, 0.0, 0.0, 217.9632, 1.0e6, 6000.0, -3000.0, 30000.0],
        [0.0, 0.0, 0.0, 0.20783, 1.0e6, 10000.0, -4000.0, 1.0e6],
        [0.0, 0.0, 0.0, 160.464, 1.0e6, 20000.0, -8000.0, 100000.0],
        [0.0, 0.0, 0.0, 7.91698694742703879e2, 1.0e6, 9.89623368428380309e3, -4.01476453880622148e3, 4.94811684214190172e4],
        [7.73054920674879185e1, 5.35077798200030514e1, 2.47940484937907922e3, 7.88471555846658561e1, 3.81425860694441212e2, 3000.0, none, 2.88248514812506794e5],
        [6.38870424613712129e-1, 1.00401113581114032e2, 6.94178172774930044e3, 3.20118436424864639e0, 5.27093707286114295e2, 8.62353945700734766e3, none, 8.59584798416446080e5],
        [-6.83219815346825641e1, 1.43556680302960249e1, 2.51982894007541790e3, -6.81582805614554985e1, 2.82127499787189180e1, 3000.0, none, 6.94619435714184423e5],
        [1.08423981130202520e1, 2.80275207050567133e2, 2.26127515896611612e3, 3.68902395574895223e1, 3.38999777145342080e2, 4.45273967541855200e3, none, 8.90547935083710472e4],
        [-4.81533508037750906e1, 1.85906600029214189e2, 2.38305795589645959e3, -2.92294522034591147e1, 2.70741950166248387e2, 3000.0, none, 7.80503145413719176e4],
        [2.16097688053005186e1, 5.19199488476061333e2, 4.98998444383786955e3, 1.05775623905558263e2, 7.22087227342823439e2, 6.31431394819386605e3, -2.02067634152031110e3, 3.75650673819755262e5],
        [3.88939999999999984e1, 4.83000003706214954e2, 2.99999999999999454e3, 1.43257495115174351e2, 1.45152183511143136e2, 3000.0, -1.41999574570260065e4, 300000.0],
        [3.53306418237799136e1, 1.09314683272240899e2, 3.56148235358225804e2, 4.28234187790264968e1, 7.66170213745982398e0, 9.77638229270576289e2, -4.15581256893435511e3, 1.64007709662116074e5],
        // Found by the random tests: at the velocity limit after the brake
        // profile, and a target just short of the position of the fastest stop
        [0.0043014788621616, 1.3537405614441522, 261.9098265161763948, 905.2205000000000155, 2.4929678283002787, 349.0815542034002874, -139.6326216813601206, 30106.7044396843375580],
        [0.1601521184462225, 68.4752438270576107, 18243.2052084032184212, 3.0645662451547011, 127.8696505915645361, 19350.7829459830936685, -3870.1565891966188246, 2801732.9779510037042201],
    ];

    for (case, [p0, v0, a0, pf, v_max, a_max, a_min, j_max]) in cases.into_iter().enumerate() {
        let mut input = InputParameter::<1>::new(None);
        input.current_position = daov_stack![p0];
        input.current_velocity = daov_stack![v0];
        input.current_acceleration = daov_stack![a0];
        input.target_position = daov_stack![pf];
        input.max_velocity = daov_stack![v_max];
        input.max_acceleration = daov_stack![a_max];
        input.min_acceleration = (!a_min.is_nan()).then(|| daov_stack![a_min]);
        input.max_jerk = daov_stack![j_max];
        let a_min = if a_min.is_nan() { -a_max } else { a_min };

        let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.001);
        let mut traj = Trajectory::new(None);
        let result = otg.calculate(&input, &mut traj);
        assert_eq!(result.map_err(|e| e.to_string()), Ok(RuckigResult::Working), "case {case}");

        let duration = traj.get_duration();
        let end = traj.state_at(duration);
        assert_float_eq!(end.position[0], pf, abs <= 1e-8);
        assert_float_eq!(end.velocity[0], 0.0, abs <= 1e-8);
        assert_float_eq!(end.acceleration[0], 0.0, abs <= 1e-8);

        // A velocity beyond its limit at the start (after lowering the
        // limit) is first brought within it
        let mut velocity_reached_limit = false;
        const STEPS: usize = 4096;
        for step in 0..=STEPS {
            let state = traj.state_at(duration * step as f64 / STEPS as f64);
            let (t, v, a, j) = (state.time, state.velocity[0], state.acceleration[0], state.jerk[0]);
            velocity_reached_limit |= v.abs() <= v_max + 1e-9;
            assert!(!velocity_reached_limit || v.abs() <= v_max + 1e-9, "case {case}: velocity {v} at t = {t}");
            assert!(a_min - 1e-9 <= a && a <= a_max + 1e-9, "case {case}: acceleration {a} at t = {t}");
            assert!(j.abs() <= j_max * (1.0 + 1e-12), "case {case}: jerk {j} at t = {t}");
        }
    }
}
