use rsruckig::prelude::*;
use rsruckig::trajectory::Trajectory;

use rand_distr::{Distribution, Uniform};
use rand_pcg::Pcg64Mcg;

const CONTINUITY_TOL: f64 = 1e-6;
const LIMIT_TOL: f64 = 1e-6;

/// Sample position, velocity and acceleration of a heap-mode trajectory
fn sample(
    traj: &Trajectory<0>,
    dofs: usize,
    t: f64,
) -> (
    DataArrayOrVec<f64, 0>,
    DataArrayOrVec<f64, 0>,
    DataArrayOrVec<f64, 0>,
) {
    let mut p = DataArrayOrVec::new(Some(dofs), 0.0);
    let mut v = DataArrayOrVec::new(Some(dofs), 0.0);
    let mut a = DataArrayOrVec::new(Some(dofs), 0.0);
    let mut j = DataArrayOrVec::new(Some(dofs), 0.0);
    let mut section = Some(0_usize);
    traj.at_time(
        t,
        &mut Some(&mut p),
        &mut Some(&mut v),
        &mut Some(&mut a),
        &mut Some(&mut j),
        &mut section,
    );
    (p, v, a)
}

/// Basic 3-DoF heap-mode input with generous limits
fn basic_input(dofs: usize) -> InputParameter<0> {
    let mut input = InputParameter::<0>::new(Some(dofs));
    for dof in 0..dofs {
        input.max_velocity[dof] = 2.0;
        input.max_acceleration[dof] = 5.0;
        input.max_jerk[dof] = 20.0;
    }
    input
}

fn push_waypoint(input: &mut InputParameter<0>, values: &[f64]) {
    let mut waypoint = DataArrayOrVec::new(Some(values.len()), 0.0);
    for (dof, value) in values.iter().enumerate() {
        waypoint[dof] = *value;
    }
    input.intermediate_positions.push(waypoint);
}

/// Assert position/velocity/acceleration continuity at every section boundary
fn assert_boundary_continuity(traj: &Trajectory<0>, dofs: usize, context: &str) {
    let eps = 1e-9;
    let boundaries = traj.get_intermediate_durations();
    for (s, &t_b) in boundaries[..boundaries.len() - 1].iter().enumerate() {
        let (p_before, v_before, a_before) = sample(traj, dofs, t_b - eps);
        let (p_after, v_after, a_after) = sample(traj, dofs, t_b + eps);
        for dof in 0..dofs {
            assert!(
                (p_before[dof] - p_after[dof]).abs() < CONTINUITY_TOL,
                "{context}: position of DoF {dof} discontinuous at section boundary {s} (t = {t_b}): {} vs {}",
                p_before[dof],
                p_after[dof]
            );
            assert!(
                (v_before[dof] - v_after[dof]).abs() < CONTINUITY_TOL,
                "{context}: velocity of DoF {dof} discontinuous at section boundary {s} (t = {t_b}): {} vs {}",
                v_before[dof],
                v_after[dof]
            );
            assert!(
                (a_before[dof] - a_after[dof]).abs() < 1e-4,
                "{context}: acceleration of DoF {dof} discontinuous at section boundary {s} (t = {t_b}): {} vs {}",
                a_before[dof],
                a_after[dof]
            );
        }
    }
}

/// Assert that dense samples respect the effective per-section velocity and
/// acceleration limits
fn assert_limits_respected(traj: &Trajectory<0>, input: &InputParameter<0>, context: &str) {
    let dofs = input.degrees_of_freedom;
    let boundaries = traj.get_intermediate_durations().to_vec();
    let duration = traj.get_duration();
    let steps = 2000;
    for i in 0..=steps {
        let t = duration * (i as f64) / (steps as f64);
        let section = boundaries.iter().take_while(|&&b| b < t).count();
        let (_, v, a) = sample(traj, dofs, t);
        for dof in 0..dofs {
            let v_max = input
                .per_section_max_velocity
                .as_ref()
                .map_or(input.max_velocity[dof], |lim| lim[section][dof]);
            let a_max = input
                .per_section_max_acceleration
                .as_ref()
                .map_or(input.max_acceleration[dof], |lim| lim[section][dof]);
            assert!(
                v[dof].abs() <= v_max * (1.0 + LIMIT_TOL) + LIMIT_TOL,
                "{context}: velocity {} of DoF {dof} at t = {t} (section {section}) exceeds limit {v_max}",
                v[dof]
            );
            assert!(
                a[dof].abs() <= a_max * (1.0 + LIMIT_TOL) + LIMIT_TOL,
                "{context}: acceleration {} of DoF {dof} at t = {t} (section {section}) exceeds limit {a_max}",
                a[dof]
            );
        }
    }
}

#[test]
fn test_waypoints_hit_exactly() {
    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(3), 0.01);
    let mut input = basic_input(3);
    input.target_position[0] = 1.0;
    input.target_position[1] = -1.0;
    input.target_position[2] = 0.5;
    push_waypoint(&mut input, &[0.4, -0.6, 1.0]);
    push_waypoint(&mut input, &[0.8, -0.8, 0.7]);

    let mut traj = Trajectory::<0>::new(Some(3));
    otg.calculate(&input, &mut traj).unwrap();

    assert_eq!(traj.profiles.len(), 3);
    let boundaries = traj.get_intermediate_durations();
    assert_eq!(boundaries.len(), 3);
    assert!((boundaries[2] - traj.get_duration()).abs() < 1e-12);
    for s in 0..boundaries.len() - 1 {
        assert!(boundaries[s] < boundaries[s + 1]);
    }

    // Sampling exactly at a boundary yields the waypoint state (the
    // pass-through acceleration is chosen by the refinement, but limited)
    for (s, waypoint) in input.intermediate_positions.iter().enumerate() {
        let (p, _, a) = sample(&traj, 3, boundaries[s]);
        for dof in 0..3 {
            assert!(
                (p[dof] - waypoint[dof]).abs() < 1e-8,
                "waypoint {s} missed for DoF {dof}: {} instead of {}",
                p[dof],
                waypoint[dof]
            );
            assert!(
                a[dof].abs() <= input.max_acceleration[dof] * (1.0 + LIMIT_TOL),
                "waypoint {s} passed with out-of-limit acceleration {}",
                a[dof]
            );
        }
    }

    // Final state is the target
    let (p, v, _) = sample(&traj, 3, traj.get_duration());
    for dof in 0..3 {
        assert!((p[dof] - input.target_position[dof]).abs() < 1e-8);
        assert!(v[dof].abs() < 1e-8);
    }

    assert_boundary_continuity(&traj, 3, "waypoints_hit_exactly");
    assert_limits_respected(&traj, &input, "waypoints_hit_exactly");
}

#[test]
fn test_monotone_chain_passes_with_nonzero_velocity() {
    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(1), 0.01);
    let mut input = basic_input(1);
    input.target_position[0] = 3.0;
    push_waypoint(&mut input, &[1.0]);
    push_waypoint(&mut input, &[2.0]);

    let mut traj = Trajectory::<0>::new(Some(1));
    otg.calculate(&input, &mut traj).unwrap();

    let boundaries = traj.get_intermediate_durations();
    for s in 0..2 {
        let (_, v, _) = sample(&traj, 1, boundaries[s]);
        assert!(
            v[0] > 0.1,
            "monotone chain should pass waypoint {s} with positive velocity, got {}",
            v[0]
        );
        assert!(v[0] <= input.max_velocity[0] * (1.0 + LIMIT_TOL));
    }

    // A stop-at-waypoint trajectory over the same chain must not be faster
    let mut stop_input = basic_input(1);
    stop_input.target_position[0] = 3.0;
    let mut stop_duration = 0.0;
    let mut section_traj = Trajectory::<0>::new(Some(1));
    for (from, to) in [(0.0, 1.0), (1.0, 2.0), (2.0, 3.0)] {
        stop_input.current_position[0] = from;
        stop_input.target_position[0] = to;
        otg.calculate(&stop_input, &mut section_traj).unwrap();
        stop_duration += section_traj.get_duration();
    }
    assert!(
        traj.get_duration() < stop_duration,
        "pass-through ({}) should be faster than stopping at every waypoint ({})",
        traj.get_duration(),
        stop_duration
    );
}

#[test]
fn test_direction_reversal() {
    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(1), 0.01);
    let mut input = basic_input(1);
    input.target_position[0] = 0.5;
    push_waypoint(&mut input, &[1.0]); // reversal: 0 -> 1 -> 0.5

    // Without refinement, the initial heuristic passes a reversal at rest
    otg.waypoint_refinement_sweeps = 0;
    let mut traj = Trajectory::<0>::new(Some(1));
    otg.calculate(&input, &mut traj).unwrap();
    let boundaries = traj.get_intermediate_durations().to_vec();
    let (p, v, _) = sample(&traj, 1, boundaries[0]);
    assert!((p[0] - 1.0).abs() < 1e-8);
    assert!(
        v[0].abs() < 1e-8,
        "unrefined direction reversal must be passed with zero velocity, got {}",
        v[0]
    );
    assert_boundary_continuity(&traj, 1, "direction_reversal_raw");
    let raw_duration = traj.get_duration();

    // With refinement, the reversal may be passed with velocity (overshoot
    // and return) — the waypoint must still be hit exactly and the result
    // must not be slower
    otg.waypoint_refinement_sweeps = 4;
    otg.calculate(&input, &mut traj).unwrap();
    let boundaries = traj.get_intermediate_durations().to_vec();
    let (p, v, _) = sample(&traj, 1, boundaries[0]);
    assert!((p[0] - 1.0).abs() < 1e-8, "refined reversal missed the waypoint");
    assert!(v[0].abs() <= input.max_velocity[0] * (1.0 + LIMIT_TOL));
    assert!(traj.get_duration() <= raw_duration + 1e-9);
    assert_boundary_continuity(&traj, 1, "direction_reversal_refined");
    assert_limits_respected(&traj, &input, "direction_reversal_refined");
}

#[test]
fn test_per_section_limits_respected() {
    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(1), 0.01);
    let mut input = basic_input(1);
    input.target_position[0] = 3.0;
    push_waypoint(&mut input, &[1.0]);
    push_waypoint(&mut input, &[2.0]);

    // Middle section is much slower
    input.per_section_max_velocity = Some(vec![
        DataArrayOrVec::new(Some(1), 2.0),
        DataArrayOrVec::new(Some(1), 0.5),
        DataArrayOrVec::new(Some(1), 2.0),
    ]);
    input.per_section_max_acceleration = Some(vec![
        DataArrayOrVec::new(Some(1), 5.0),
        DataArrayOrVec::new(Some(1), 2.0),
        DataArrayOrVec::new(Some(1), 5.0),
    ]);

    let mut traj = Trajectory::<0>::new(Some(1));
    otg.calculate(&input, &mut traj).unwrap();

    assert_boundary_continuity(&traj, 1, "per_section_limits");
    assert_limits_respected(&traj, &input, "per_section_limits");

    // The waypoint velocities must respect the tighter neighboring section
    let boundaries = traj.get_intermediate_durations();
    for s in 0..2 {
        let (_, v, _) = sample(&traj, 1, boundaries[s]);
        assert!(
            v[0].abs() <= 0.5 * (1.0 + LIMIT_TOL),
            "waypoint {s} velocity {} exceeds the middle section limit 0.5",
            v[0]
        );
    }
}

#[test]
fn test_per_section_minimum_duration() {
    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(1), 0.01);
    let mut input = basic_input(1);
    input.target_position[0] = 3.0;
    push_waypoint(&mut input, &[1.0]);
    push_waypoint(&mut input, &[2.0]);
    input.per_section_minimum_duration = Some(vec![0.5, 2.0, 0.5]);

    let mut traj = Trajectory::<0>::new(Some(1));
    otg.calculate(&input, &mut traj).unwrap();

    let boundaries = traj.get_intermediate_durations();
    let mut previous = 0.0;
    for (s, &min_duration) in [0.5, 2.0, 0.5].iter().enumerate() {
        let section_duration = boundaries[s] - previous;
        assert!(
            section_duration >= min_duration - 1e-9,
            "section {s} duration {section_duration} undercuts its minimum {min_duration}"
        );
        previous = boundaries[s];
    }
    assert_boundary_continuity(&traj, 1, "per_section_minimum_duration");
}

#[test]
fn test_position_limits_with_waypoints() {
    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(1), 0.01);
    let mut input = basic_input(1);
    input.target_position[0] = 0.0;
    push_waypoint(&mut input, &[1.0]); // waypoint exactly on the position limit
    input.max_position = Some(DataArrayOrVec::new(Some(1), 1.0));
    input.min_position = Some(DataArrayOrVec::new(Some(1), -1.0));

    let mut traj = Trajectory::<0>::new(Some(1));
    otg.calculate(&input, &mut traj).unwrap();

    let extrema = traj.get_position_extrema();
    assert!(extrema[0].max <= 1.0 + LIMIT_TOL);
    assert!(extrema[0].min >= -1.0 - LIMIT_TOL);
    assert_boundary_continuity(&traj, 1, "position_limits");
}

#[test]
fn test_validation_rejections() {
    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(1), 0.01);
    let mut traj = Trajectory::<0>::new(Some(1));

    // Infinite jerk (the default) is not supported with waypoints
    let mut input = InputParameter::<0>::new(Some(1));
    input.max_velocity[0] = 1.0;
    input.max_acceleration[0] = 1.0;
    input.target_position[0] = 2.0;
    push_waypoint(&mut input, &[1.0]);
    assert!(otg.calculate(&input, &mut traj).is_err());

    // Velocity control interface
    let mut input = basic_input(1);
    input.target_position[0] = 2.0;
    push_waypoint(&mut input, &[1.0]);
    input.control_interface = ControlInterface::Velocity;
    assert!(otg.calculate(&input, &mut traj).is_err());
    input.control_interface = ControlInterface::Position;
    assert!(otg.calculate(&input, &mut traj).is_ok());

    // Per-DoF settings
    input.per_dof_synchronization = Some(DataArrayOrVec::new(Some(1), Synchronization::Time));
    assert!(otg.calculate(&input, &mut traj).is_err());
    input.per_dof_synchronization = None;

    // Global minimum duration
    input.minimum_duration = Some(1.0);
    assert!(otg.calculate(&input, &mut traj).is_err());
    input.minimum_duration = None;

    // Discrete durations
    input.duration_discretization = DurationDiscretization::Discrete;
    assert!(otg.calculate(&input, &mut traj).is_err());
    input.duration_discretization = DurationDiscretization::Continuous;

    // NaN waypoint
    input.intermediate_positions[0][0] = f64::NAN;
    assert!(otg.calculate(&input, &mut traj).is_err());
    input.intermediate_positions[0][0] = 1.0;

    // Waypoint with the wrong number of DoFs
    input.intermediate_positions.push(DataArrayOrVec::new(Some(2), 0.0));
    assert!(otg.calculate(&input, &mut traj).is_err());
    input.intermediate_positions.pop();

    // Per-section array with the wrong number of sections
    input.per_section_max_velocity = Some(vec![DataArrayOrVec::new(Some(1), 1.0)]);
    assert!(otg.calculate(&input, &mut traj).is_err());
    input.per_section_max_velocity = None;

    // Wrong-length per-section minimum duration
    input.per_section_minimum_duration = Some(vec![0.5]);
    assert!(otg.calculate(&input, &mut traj).is_err());
    input.per_section_minimum_duration = None;

    // Waypoint outside the global position limits
    input.max_position = Some(DataArrayOrVec::new(Some(1), 0.5));
    assert!(otg.calculate(&input, &mut traj).is_err());
    input.max_position = None;

    assert!(otg.calculate(&input, &mut traj).is_ok());
}

#[test]
fn test_filter_intermediate_positions() {
    let otg = Ruckig::<0, ThrowErrorHandler>::new(Some(2), 0.01);
    let mut input = InputParameter::<0>::new(Some(2));
    for dof in 0..2 {
        input.max_velocity[dof] = 1.0;
        input.max_acceleration[dof] = 1.0;
        input.max_jerk[dof] = 1.0;
    }
    input.target_position[0] = 3.0;
    input.target_position[1] = 3.0;
    let threshold = DataArrayOrVec::new(Some(2), 0.1);

    // Empty input stays empty
    assert!(otg.filter_intermediate_positions(&input, &threshold).is_empty());

    // Collinear waypoints are filtered out
    push_waypoint(&mut input, &[1.0, 1.0]);
    push_waypoint(&mut input, &[2.0, 2.0]);
    let filtered = otg.filter_intermediate_positions(&input, &threshold);
    assert!(
        filtered.is_empty(),
        "collinear waypoints should be dropped, {} remain",
        filtered.len()
    );

    // A waypoint clearly off the line is kept
    input.intermediate_positions.clear();
    push_waypoint(&mut input, &[1.0, 1.0]);
    push_waypoint(&mut input, &[1.5, 0.0]); // off the diagonal
    let filtered = otg.filter_intermediate_positions(&input, &threshold);
    assert!(filtered.iter().any(|w| w[1] == 0.0), "off-line waypoint dropped");

    // Zero-extent DoF between the anchors: the remaining DoF decides
    let mut flat = InputParameter::<0>::new(Some(2));
    flat.target_position[0] = 2.0;
    flat.target_position[1] = 0.0; // no motion in DoF 1
    push_waypoint(&mut flat, &[1.0, 0.0]);
    let filtered = otg.filter_intermediate_positions(&flat, &threshold);
    assert!(filtered.is_empty(), "collinear waypoint with zero-extent DoF kept");
}

#[test]
fn test_update_section_outputs() {
    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(1), 0.01);
    let mut input = basic_input(1);
    input.target_position[0] = 3.0;
    push_waypoint(&mut input, &[1.0]);
    push_waypoint(&mut input, &[2.0]);
    let mut output = OutputParameter::<0>::new(Some(1));

    let mut section_changes = 0;
    let mut last_section = 0;
    let mut first = true;
    while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
        if first {
            assert!(output.new_calculation);
            assert_eq!(output.new_section, 0);
            first = false;
        } else {
            assert!(!output.new_calculation);
        }
        assert!(
            output.new_section >= last_section,
            "new_section went backwards: {} -> {}",
            last_section,
            output.new_section
        );
        if output.did_section_change {
            assert!(output.new_section > last_section);
            section_changes += 1;
        }
        last_section = output.new_section;
        output.pass_to_input(&mut input);
    }
    assert!(
        section_changes >= 2,
        "expected at least two section changes, got {section_changes}"
    );
    assert!(last_section >= 2);

    // Mutating a waypoint mid-run triggers a recalculation and resets the section
    let mut input = basic_input(1);
    input.target_position[0] = 3.0;
    push_waypoint(&mut input, &[1.0]);
    push_waypoint(&mut input, &[2.0]);
    let mut output = OutputParameter::<0>::new(Some(1));
    while output.new_section == 0 {
        assert_eq!(otg.update(&input, &mut output).unwrap(), RuckigResult::Working);
        output.pass_to_input(&mut input);
        assert!(output.time < 5.0, "expected to pass the first waypoint");
    }
    input.intermediate_positions[1][0] = 2.5;
    assert_eq!(otg.update(&input, &mut output).unwrap(), RuckigResult::Working);
    assert!(output.new_calculation, "waypoint change must trigger a recalculation");
    assert_eq!(output.new_section, 0, "section must reset on recalculation");
    assert!(!output.did_section_change);
}

#[test]
fn test_brake_only_in_first_section() {
    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(1), 0.01);
    let mut input = basic_input(1);
    input.current_velocity[0] = 3.0; // above max_velocity: forces a brake pre-trajectory
    input.target_position[0] = 4.0;
    push_waypoint(&mut input, &[2.0]);
    push_waypoint(&mut input, &[3.0]);

    let mut traj = Trajectory::<0>::new(Some(1));
    otg.calculate(&input, &mut traj).unwrap();

    assert!(
        traj.profiles[0][0].brake.duration > 0.0,
        "expected a brake pre-trajectory in the first section"
    );
    for section in 1..traj.profiles.len() {
        assert_eq!(
            traj.profiles[section][0].brake.duration, 0.0,
            "unexpected brake pre-trajectory in section {section}"
        );
    }
    assert_boundary_continuity(&traj, 1, "brake_only_in_first_section");
}

#[test]
fn test_refinement_improves_duration() {
    // The refined trajectory must never be slower than the raw heuristic
    // chain, and all soundness invariants must hold for both
    let mut input = basic_input(3);
    input.target_position[0] = 2.0;
    input.target_position[1] = -1.0;
    input.target_position[2] = 1.0;
    push_waypoint(&mut input, &[0.5, -0.3, 0.4]);
    push_waypoint(&mut input, &[1.2, -0.7, 0.9]);
    push_waypoint(&mut input, &[1.7, -0.9, 0.2]);

    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(3), 0.01);
    let mut traj_refined = Trajectory::<0>::new(Some(3));
    otg.calculate(&input, &mut traj_refined).unwrap();

    otg.waypoint_refinement_sweeps = 0;
    let mut traj_raw = Trajectory::<0>::new(Some(3));
    otg.calculate(&input, &mut traj_raw).unwrap();

    assert!(
        traj_refined.get_duration() <= traj_raw.get_duration() + 1e-9,
        "refinement made the trajectory slower: {} vs {}",
        traj_refined.get_duration(),
        traj_raw.get_duration()
    );
    for traj in [&traj_refined, &traj_raw] {
        assert_boundary_continuity(traj, 3, "refinement_improves_duration");
        assert_limits_respected(traj, &input, "refinement_improves_duration");
    }
    // Waypoints still hit exactly after refinement
    let boundaries = traj_refined.get_intermediate_durations();
    for (s, waypoint) in input.intermediate_positions.iter().enumerate() {
        let (p, _, _) = sample(&traj_refined, 3, boundaries[s]);
        for dof in 0..3 {
            assert!((p[dof] - waypoint[dof]).abs() < 1e-8);
        }
    }
}

#[test]
fn test_trackig_rejects_waypoints() {
    let mut otg = Trackig::<0, IgnoreErrorHandler>::new(Some(1), 0.01);
    let mut input = basic_input(1);
    push_waypoint(&mut input, &[1.0]);
    let mut output = OutputParameter::<0>::new(Some(1));
    let target = TargetState::new(Some(1));
    assert_eq!(
        otg.update(&target, &input, &mut output).unwrap(),
        RuckigResult::ErrorInvalidInput
    );

    let mut otg = Trackig::<0, ThrowErrorHandler>::new(Some(1), 0.01);
    assert!(otg.update(&target, &input, &mut output).is_err());
}

#[test]
fn test_randomized_waypoint_chains() {
    let mut rng = Pcg64Mcg::new(0x77a1);
    let position_dist = Uniform::new(-10.0, 10.0);
    let velocity_dist = Uniform::new(0.5, 10.0);
    let acceleration_dist = Uniform::new(1.0, 20.0);
    let jerk_dist = Uniform::new(2.0, 100.0);
    let count_dist = Uniform::new(1_usize, 6);

    let dofs = 3;
    let mut otg = Ruckig::<0, ThrowErrorHandler>::with_waypoints(Some(dofs), 0.01, 5);
    let mut traj = Trajectory::<0>::new(Some(dofs));

    for iteration in 0..200 {
        let mut input = InputParameter::<0>::with_waypoints(Some(dofs), 5);
        for dof in 0..dofs {
            input.max_velocity[dof] = velocity_dist.sample(&mut rng);
            input.max_acceleration[dof] = acceleration_dist.sample(&mut rng);
            input.max_jerk[dof] = jerk_dist.sample(&mut rng);
            input.current_position[dof] = position_dist.sample(&mut rng);
            input.target_position[dof] = position_dist.sample(&mut rng);
        }
        let n_waypoints = count_dist.sample(&mut rng);
        for _ in 0..n_waypoints {
            let mut waypoint = DataArrayOrVec::new(Some(dofs), 0.0);
            for dof in 0..dofs {
                waypoint[dof] = position_dist.sample(&mut rng);
            }
            input.intermediate_positions.push(waypoint);
        }

        let result = otg.calculate(&input, &mut traj);
        assert!(
            result.is_ok(),
            "iteration {iteration}: calculation failed: {result:?}\ninput: {input}"
        );

        assert_eq!(traj.profiles.len(), n_waypoints + 1);
        assert_boundary_continuity(&traj, dofs, &format!("random iteration {iteration}"));
        assert_limits_respected(&traj, &input, &format!("random iteration {iteration}"));

        // All waypoints are hit
        let boundaries = traj.get_intermediate_durations();
        for (s, waypoint) in input.intermediate_positions.iter().enumerate() {
            let (p, _, _) = sample(&traj, dofs, boundaries[s]);
            for dof in 0..dofs {
                assert!(
                    (p[dof] - waypoint[dof]).abs() < 1e-6,
                    "iteration {iteration}: waypoint {s} missed for DoF {dof}: {} instead of {}",
                    p[dof],
                    waypoint[dof]
                );
            }
        }
    }
}
