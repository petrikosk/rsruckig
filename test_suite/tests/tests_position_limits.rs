use rsruckig::brake::{
    stop_distance_second_order, stop_distance_third_order, velocity_cap_second_order,
    velocity_cap_third_order,
};
use rsruckig::prelude::*;
use rsruckig::trajectory::Trajectory;

use rand_distr::{Distribution, Uniform};
use rand_pcg::Pcg64Mcg;

const LIMIT_TOL: f64 = 1e-6;
/// A profile reaches its target position within the precision of the profile
/// check (1e-8), which the extrema do not account for
const EXTREMA_TOL: f64 = 1e-7;

/// Assert that the trajectory extrema of every DoF stay inside the given limits
fn assert_extrema_within(
    traj: &mut Trajectory<0>,
    min_pos: &[f64],
    max_pos: &[f64],
    context: &str,
) {
    let ext = traj.get_position_extrema();
    for dof in 0..min_pos.len() {
        assert!(
            ext[dof].max <= max_pos[dof] + LIMIT_TOL,
            "{context}: DoF {dof} max extremum {} exceeds limit {}",
            ext[dof].max,
            max_pos[dof]
        );
        assert!(
            ext[dof].min >= min_pos[dof] - LIMIT_TOL,
            "{context}: DoF {dof} min extremum {} undercuts limit {}",
            ext[dof].min,
            min_pos[dof]
        );
    }
}

#[test]
fn test_stop_distance_velocity_cap_roundtrip() {
    let mut rng = Pcg64Mcg::new(0x5b1);
    let d_dist = Uniform::new(1e-6, 100.0);
    let a_dist = Uniform::new(0.1, 50.0);
    let j_dist = Uniform::new(0.5, 500.0);

    for _ in 0..10_000 {
        let d = d_dist.sample(&mut rng);
        let a_min = -a_dist.sample(&mut rng);
        let j_max = j_dist.sample(&mut rng);

        // Third order: a full stop from the capped velocity must fit within d
        let v_cap = velocity_cap_third_order(d, a_min, j_max);
        assert!(v_cap >= 0.0);
        let stop = stop_distance_third_order(v_cap, 0.0, a_min, j_max);
        assert!(
            stop <= d * (1.0 + 1e-9) + 1e-12,
            "third order: stop {stop} > d {d} (v_cap {v_cap}, a_min {a_min}, j_max {j_max})"
        );
        // ... and it should not be needlessly conservative
        assert!(
            stop >= d * (1.0 - 1e-6) - 1e-12,
            "third order: stop {stop} far below d {d}"
        );

        // Second order
        let v_cap2 = velocity_cap_second_order(d, a_min);
        let stop2 = stop_distance_second_order(v_cap2, a_min);
        assert!((stop2 - d).abs() <= d * 1e-9 + 1e-12);
    }

    // Trivial cases
    assert_eq!(velocity_cap_third_order(0.0, -1.0, 1.0), 0.0);
    assert_eq!(velocity_cap_third_order(-1.0, -1.0, 1.0), 0.0);
    assert_eq!(stop_distance_third_order(-1.0, 0.0, -1.0, 1.0), 0.0);
    assert_eq!(stop_distance_second_order(-1.0, -1.0), 0.0);
}

#[test]
fn test_validation() {
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    let mut input = InputParameter::<1>::new(None);
    input.max_velocity[0] = 10.0;
    input.max_acceleration[0] = 10.0;
    input.max_jerk[0] = 30.0;
    input.min_position = Some(daov_stack![-1.0]);
    input.max_position = Some(daov_stack![1.0]);
    let mut traj = Trajectory::<1>::new(None);

    // Target outside the limits
    input.target_position[0] = 2.0;
    assert!(otg.calculate(&input, &mut traj).is_err());
    input.target_position[0] = -2.0;
    assert!(otg.calculate(&input, &mut traj).is_err());

    // Target on the limit with velocity into the wall
    input.target_position[0] = 1.0;
    input.target_velocity[0] = 1.0;
    assert!(otg.calculate(&input, &mut traj).is_err());

    // Target on the limit at rest is fine
    input.target_velocity[0] = 0.0;
    assert!(otg.calculate(&input, &mut traj).is_ok());

    // Inverted limits
    input.min_position = Some(daov_stack![2.0]);
    assert!(otg.calculate(&input, &mut traj).is_err());
}

#[test]
fn test_unavoidable_crossing_is_detected() {
    // Stopping distance from v0 = 10 (a_max = 10, j_max = 30) is ~6.67:
    // a limit at 3.0 cannot be respected from this state
    let mut otg = Ruckig::<1, IgnoreErrorHandler>::new(None, 0.01);
    let mut input = InputParameter::<1>::new(None);
    let mut output = OutputParameter::<1>::new(None);
    input.current_velocity[0] = 10.0;
    input.target_position[0] = 1.0;
    input.max_velocity[0] = 10.0;
    input.max_acceleration[0] = 10.0;
    input.max_jerk[0] = 30.0;
    input.min_position = Some(daov_stack![-10.0]);
    input.max_position = Some(daov_stack![3.0]);

    let result = otg.update(&input, &mut output).unwrap();
    assert_eq!(result, RuckigResult::ErrorPositionalLimits);

    // With ThrowErrorHandler the same input produces an error
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    let mut traj = Trajectory::<1>::new(None);
    assert!(otg.calculate(&input, &mut traj).is_err());
}

#[test]
fn test_unbound_input_is_unchanged() {
    // Limits far away must not alter the time-optimal trajectory
    let mut input = InputParameter::<1>::new(None);
    input.current_velocity[0] = 7.0;
    input.target_position[0] = 10.0;
    input.max_velocity[0] = 10.0;
    input.max_acceleration[0] = 10.0;
    input.max_jerk[0] = 30.0;

    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    let mut traj = Trajectory::<1>::new(None);
    otg.calculate(&input, &mut traj).unwrap();
    let free_duration = traj.get_duration();

    input.min_position = Some(daov_stack![-100.0]);
    input.max_position = Some(daov_stack![100.0]);
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    otg.calculate(&input, &mut traj).unwrap();
    assert_eq!(traj.get_duration(), free_duration);

    // Same for infinite limits on individual DoFs
    input.min_position = Some(daov_stack![f64::NEG_INFINITY]);
    input.max_position = Some(daov_stack![f64::INFINITY]);
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    otg.calculate(&input, &mut traj).unwrap();
    assert_eq!(traj.get_duration(), free_duration);
}

#[test]
fn test_boundary_start() {
    let mut input = InputParameter::<1>::new(None);
    input.max_velocity[0] = 5.0;
    input.max_acceleration[0] = 10.0;
    input.max_jerk[0] = 100.0;
    input.min_position = Some(daov_stack![-1.0]);
    input.max_position = Some(daov_stack![1.0]);

    // Start exactly on the limit, at rest, moving inward
    input.current_position[0] = 1.0;
    input.target_position[0] = 0.0;
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    let mut traj1 = Trajectory::<1>::new(None);
    otg.calculate(&input, &mut traj1).unwrap();
    let ext = traj1.get_position_extrema();
    assert!(ext[0].max <= 1.0 + LIMIT_TOL);
    assert!(ext[0].min >= -1.0 - LIMIT_TOL);

    // Start on the limit moving away from it
    input.current_velocity[0] = -1.0;
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    otg.calculate(&input, &mut traj1).unwrap();
    let ext = traj1.get_position_extrema();
    assert!(ext[0].max <= 1.0 + LIMIT_TOL);

    // Start near the limit moving toward it slowly: brake + return
    input.current_position[0] = 0.9;
    input.current_velocity[0] = 1.0;
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    otg.calculate(&input, &mut traj1).unwrap();
    let ext = traj1.get_position_extrema();
    assert!(ext[0].max <= 1.0 + LIMIT_TOL, "extremum {}", ext[0].max);
}

#[test]
fn test_velocity_interface_with_position_limits() {
    // Command full speed toward a wall: position must saturate below the wall
    let mut otg = Ruckig::<1, IgnoreErrorHandler>::new(None, 0.001);
    let mut input = InputParameter::<1>::new(None);
    let mut output = OutputParameter::<1>::new(None);
    input.control_interface = ControlInterface::Velocity;
    input.target_velocity[0] = 10.0;
    input.max_velocity[0] = 10.0;
    input.max_acceleration[0] = 10.0;
    input.max_jerk[0] = 30.0;
    input.min_position = Some(daov_stack![-2.0]);
    input.max_position = Some(daov_stack![2.0]);

    let mut max_p: f64 = 0.0;
    for _ in 0..20_000 {
        let res = otg.update(&input, &mut output).unwrap();
        output.pass_to_input(&mut input);
        max_p = max_p.max(output.new_position[0]);
        if res == RuckigResult::Finished {
            break;
        }
    }
    assert!(max_p <= 2.0 + LIMIT_TOL, "max position {max_p}");
}

fn random_fuzz(order: u8, cases: usize, seed: u64) {
    let mut rng = Pcg64Mcg::new(seed as u128);
    let pos_dist = Uniform::new(-5.0, 5.0);
    let vel_scale = Uniform::new(0.5, 10.0);
    let acc_scale = Uniform::new(1.0, 20.0);
    let jerk_scale = Uniform::new(5.0, 100.0);
    let margin_dist = Uniform::new(0.0, 3.0);
    let frac = Uniform::new(-1.0, 1.0);

    let mut working = 0usize;
    let mut rejected = 0usize;

    for case in 0..cases {
        let mut input = InputParameter::<0>::new(Some(1));
        let p0 = pos_dist.sample(&mut rng);
        let pf = pos_dist.sample(&mut rng);
        let v_max = vel_scale.sample(&mut rng);
        input.current_position[0] = p0;
        input.target_position[0] = pf;
        input.max_velocity[0] = v_max;
        input.max_acceleration[0] = if order >= 2 {
            acc_scale.sample(&mut rng)
        } else {
            f64::INFINITY
        };
        input.max_jerk[0] = if order >= 3 {
            jerk_scale.sample(&mut rng)
        } else {
            f64::INFINITY
        };
        if order >= 2 {
            input.current_velocity[0] = frac.sample(&mut rng) * v_max;
        }
        if order >= 3 {
            input.current_acceleration[0] =
                frac.sample(&mut rng) * input.max_acceleration[0] * 0.5;
        }

        let min_pos = p0.min(pf) - margin_dist.sample(&mut rng);
        let max_pos = p0.max(pf) + margin_dist.sample(&mut rng);
        input.min_position = Some(daov_heap![min_pos]);
        input.max_position = Some(daov_heap![max_pos]);

        let mut otg = Ruckig::<0, IgnoreErrorHandler>::new(Some(1), 0.01);
        let mut traj = Trajectory::<0>::new(Some(1));
        let result = otg.calculate(&input, &mut traj).unwrap();

        match result {
            RuckigResult::Working => {
                working += 1;
                assert_extrema_within(
                    &mut traj,
                    &[min_pos],
                    &[max_pos],
                    &format!("order {order} case {case} ({input})"),
                );
                // The target state must still be reached
                let mut p = daov_heap![0.0];
                let mut v = daov_heap![0.0];
                let mut a = daov_heap![0.0];
                let duration = traj.get_duration();
                traj.at_time(
                    duration,
                    &mut Some(&mut p),
                    &mut Some(&mut v),
                    &mut Some(&mut a),
                    &mut None,
                    &mut None,
                );
                assert!(
                    (p[0] - pf).abs() < 1e-6,
                    "order {order} case {case}: end position {} != target {}",
                    p[0],
                    pf
                );
            }
            _ => {
                // An error code is acceptable (e.g. unavoidable crossing from
                // the random initial state) - a silent violation is not
                rejected += 1;
            }
        }
    }

    // The vast majority of sampled cases must be solvable
    assert!(
        working > cases / 2,
        "order {order}: only {working}/{cases} solvable ({rejected} rejected)"
    );
}

#[test]
fn test_fuzz_third_order() {
    random_fuzz(3, 2000, 0xf00d);
}

#[test]
fn test_fuzz_second_order() {
    random_fuzz(2, 1000, 0xbeef);
}

#[test]
fn test_fuzz_first_order() {
    random_fuzz(1, 500, 0xcafe);
}

#[test]
fn test_time_synchronization_with_position_limits() {
    // DoF 0 is slow and far, DoF 1 is fast, close to its upper limit.
    // Time synchronization must not push DoF 1 beyond its limit.
    // (Note: the stopping distance from v0 = 2.5 is ~0.40 < 0.5, so the
    // limit of DoF 1 is respectable - v0 = 3.0 would make it unavoidable.)
    let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.01);
    let mut input = InputParameter::<3>::new(None);
    input.current_position = daov_stack![0.0, 0.0, 0.0];
    input.current_velocity = daov_stack![0.0, 2.5, 0.0];
    input.target_position = daov_stack![10.0, 0.4, -5.0];
    input.max_velocity = daov_stack![1.0, 5.0, 5.0];
    input.max_acceleration = daov_stack![5.0, 20.0, 10.0];
    input.max_jerk = daov_stack![20.0, 100.0, 50.0];
    input.min_position = Some(daov_stack![-20.0, -0.5, -20.0]);
    input.max_position = Some(daov_stack![20.0, 0.5, 20.0]);

    let mut traj = Trajectory::<3>::new(None);
    otg.calculate(&input, &mut traj).unwrap();

    let duration = traj.get_duration();
    let ext = traj.get_position_extrema();
    assert!(ext[1].max <= 0.5 + LIMIT_TOL, "extremum {}", ext[1].max);
    assert!(ext[1].min >= -0.5 - LIMIT_TOL);

    // All DoFs arrive simultaneously
    let mut p = daov_stack![0.0, 0.0, 0.0];
    traj.at_time(duration, &mut Some(&mut p), &mut None, &mut None, &mut None, &mut None);
    for dof in 0..3 {
        assert!(
            (p[dof] - input.target_position[dof]).abs() < 1e-6,
            "DoF {dof}: {} != {}",
            p[dof],
            input.target_position[dof]
        );
    }
}

#[test]
fn test_phase_synchronization_with_position_limits() {
    let mut otg = Ruckig::<2, ThrowErrorHandler>::new(None, 0.01);
    let mut input = InputParameter::<2>::new(None);
    input.synchronization = Synchronization::Phase;
    input.current_position = daov_stack![0.0, 0.0];
    input.target_position = daov_stack![1.0, 2.0];
    input.max_velocity = daov_stack![5.0, 5.0];
    input.max_acceleration = daov_stack![10.0, 10.0];
    input.max_jerk = daov_stack![50.0, 50.0];
    input.min_position = Some(daov_stack![-1.0, -1.0]);
    input.max_position = Some(daov_stack![1.5, 2.5]);

    let mut traj = Trajectory::<2>::new(None);
    otg.calculate(&input, &mut traj).unwrap();
    let ext = traj.get_position_extrema();
    assert!(ext[0].max <= 1.5 + LIMIT_TOL);
    assert!(ext[1].max <= 2.5 + LIMIT_TOL);
}

#[test]
fn test_stepwise_update_respects_limits() {
    // Step through a whole braked + limited motion with update() and check
    // every sampled position
    let mut otg = Ruckig::<1, IgnoreErrorHandler>::new(None, 0.001);
    let mut input = InputParameter::<1>::new(None);
    let mut output = OutputParameter::<1>::new(None);
    input.current_velocity[0] = 4.0;
    input.current_position[0] = 0.0;
    input.target_position[0] = 0.5;
    input.max_velocity[0] = 5.0;
    input.max_acceleration[0] = 10.0;
    input.max_jerk[0] = 100.0;
    input.min_position = Some(daov_stack![-1.0]);
    input.max_position = Some(daov_stack![1.0]);

    let mut steps = 0;
    loop {
        let res = otg.update(&input, &mut output).unwrap();
        assert!(
            output.new_position[0] <= 1.0 + LIMIT_TOL,
            "step {steps}: position {} beyond limit",
            output.new_position[0]
        );
        assert!(output.new_position[0] >= -1.0 - LIMIT_TOL);
        output.pass_to_input(&mut input);
        steps += 1;
        if res == RuckigResult::Finished || steps > 20_000 {
            break;
        }
    }
    assert!(steps < 20_000, "did not finish");
    assert!((output.new_position[0] - 0.5).abs() < 1e-6);
}

/// Sample the whole trajectory and assert that every position of `dof` stays
/// inside the limits and inside the reported extrema
fn assert_samples_within<const DOF: usize>(
    traj: &mut Trajectory<DOF>,
    dof: usize,
    min_pos: f64,
    max_pos: f64,
) {
    let duration = traj.get_duration();
    let ext = traj.get_position_extrema()[dof];
    assert!(ext.max <= max_pos + LIMIT_TOL, "extremum {}", ext.max);
    assert!(ext.min >= min_pos - LIMIT_TOL, "extremum {}", ext.min);

    for i in 0..=2000 {
        let t = duration * f64::from(i) / 2000.0;
        let p = traj.state_at(t).position[dof];
        assert!(p <= max_pos + LIMIT_TOL, "position {p} at t = {t}");
        assert!(p >= min_pos - LIMIT_TOL, "position {p} at t = {t}");
        assert!(
            p <= ext.max + EXTREMA_TOL && p >= ext.min - EXTREMA_TOL,
            "position {p} at t = {t} outside of the extrema [{}, {}]",
            ext.min,
            ext.max
        );
    }
}

#[test]
fn test_no_synchronization_respects_limits_until_the_end() {
    // Issue #17: DoF 0 reaches its target with a velocity long before DoF 1
    // finishes, and must not drift beyond its limit in the meantime
    let mut otg = Ruckig::<2, ThrowErrorHandler>::new(None, 0.01);
    let mut input = InputParameter::<2>::new(None);
    let mut traj = Trajectory::<2>::new(None);

    input.synchronization = Synchronization::None;
    input.target_position = daov_stack![0.5, 10.0];
    input.target_velocity = daov_stack![0.5, 0.0];
    input.max_velocity = daov_stack![1.0, 1.0];
    input.max_acceleration = daov_stack![1.0, 1.0];
    input.max_jerk = daov_stack![1.0, 1.0];
    input.min_position = Some(daov_stack![-1.0, -1.0]);
    input.max_position = Some(daov_stack![1.5, 11.0]);

    let result = otg.calculate(&input, &mut traj).unwrap();
    assert_eq!(result, RuckigResult::Working);
    assert_samples_within(&mut traj, 0, -1.0, 1.5);
    assert_samples_within(&mut traj, 1, -1.0, 11.0);

    // Both DoFs reach their target state at the end
    let end = traj.state_at(traj.get_duration());
    assert!((end.position[0] - 0.5).abs() < 1e-6);
    assert!((end.velocity[0] - 0.5).abs() < 1e-6);
    assert!((end.position[1] - 10.0).abs() < 1e-6);
}

#[test]
fn test_no_synchronization_is_unchanged_inside_the_limits() {
    // The same motion with enough room: DoF 0 stays independent and keeps
    // its target velocity after its own (shorter) profile
    let mut otg = Ruckig::<2, ThrowErrorHandler>::new(None, 0.01);
    let mut input = InputParameter::<2>::new(None);
    let mut traj = Trajectory::<2>::new(None);

    input.synchronization = Synchronization::None;
    input.target_position = daov_stack![0.5, 10.0];
    input.target_velocity = daov_stack![0.5, 0.0];
    input.max_velocity = daov_stack![1.0, 1.0];
    input.max_acceleration = daov_stack![1.0, 1.0];
    input.max_jerk = daov_stack![1.0, 1.0];
    input.min_position = Some(daov_stack![-1.0, -1.0]);
    input.max_position = Some(daov_stack![20.0, 11.0]);

    otg.calculate(&input, &mut traj).unwrap();
    let duration = traj.get_duration();
    let t_dof0 = traj.get_independent_min_durations()[0];
    assert!(t_dof0 < duration - 1.0);
    assert!(traj.get_profiles()[0][0].total_duration() < duration - 1.0);

    // The extrema include the motion after the profile of DoF 0
    let end = traj.state_at(duration);
    let expected = 0.5 + 0.5 * (duration - t_dof0);
    assert!((end.position[0] - expected).abs() < 1e-9);
    let ext = traj.get_position_extrema()[0];
    assert!((ext.max - expected).abs() < 1e-9, "extremum {}", ext.max);
    assert!((ext.t_max - duration).abs() < 1e-9);
    assert_samples_within(&mut traj, 0, -1.0, 20.0);
}

#[test]
fn test_fuzz_no_synchronization() {
    let mut rng = Pcg64Mcg::new(0x17);
    let pos_dist = Uniform::new(-5.0, 5.0);
    let vel_scale = Uniform::new(0.5, 10.0);
    let acc_scale = Uniform::new(1.0, 20.0);
    let jerk_scale = Uniform::new(5.0, 100.0);
    let margin_dist = Uniform::new(0.0, 3.0);
    let frac = Uniform::new(-1.0, 1.0);

    let cases = 2000;
    let mut working = 0usize;
    for case in 0..cases {
        let mut input = InputParameter::<3>::new(None);
        input.synchronization = Synchronization::None;
        let mut min_pos = [0.0; 3];
        let mut max_pos = [0.0; 3];
        for dof in 0..3 {
            let p0 = pos_dist.sample(&mut rng);
            let pf = pos_dist.sample(&mut rng);
            let v_max = vel_scale.sample(&mut rng);
            input.current_position[dof] = p0;
            input.target_position[dof] = pf;
            input.max_velocity[dof] = v_max;
            input.max_acceleration[dof] = acc_scale.sample(&mut rng);
            input.max_jerk[dof] = jerk_scale.sample(&mut rng);
            input.current_velocity[dof] = frac.sample(&mut rng) * v_max;
            input.target_velocity[dof] = 0.3 * frac.sample(&mut rng) * v_max;
            min_pos[dof] = p0.min(pf) - margin_dist.sample(&mut rng);
            max_pos[dof] = p0.max(pf) + margin_dist.sample(&mut rng);
        }
        input.min_position = Some(daov_stack![min_pos[0], min_pos[1], min_pos[2]]);
        input.max_position = Some(daov_stack![max_pos[0], max_pos[1], max_pos[2]]);

        let mut otg = Ruckig::<3, IgnoreErrorHandler>::new(None, 0.01);
        let mut traj = Trajectory::<3>::new(None);
        // An error code is acceptable, a silent violation is not
        if !matches!(otg.calculate(&input, &mut traj), Ok(RuckigResult::Working)) {
            continue;
        }
        working += 1;

        let duration = traj.get_duration();
        let ext = traj.get_position_extrema().clone();
        for dof in 0..3 {
            for i in 0..=200 {
                let p = traj.state_at(duration * f64::from(i) / 200.0).position[dof];
                assert!(
                    p <= max_pos[dof] + LIMIT_TOL && p >= min_pos[dof] - LIMIT_TOL,
                    "case {case} DoF {dof}: position {p} outside [{}, {}] ({input})",
                    min_pos[dof],
                    max_pos[dof]
                );
                assert!(
                    p <= ext[dof].max + EXTREMA_TOL && p >= ext[dof].min - EXTREMA_TOL,
                    "case {case} DoF {dof}: position {p} outside of the extrema [{}, {}] at i = {i}, duration {duration}, profile {:?} ({input})",
                    ext[dof].min,
                    ext[dof].max,
                    traj.get_profiles()[0][dof]
                );
            }
        }
    }

    assert!(working > cases / 4, "only {working}/{cases} solvable");
}
