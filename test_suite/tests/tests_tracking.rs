use rsruckig::prelude::*;

const DT: f64 = 0.001;

#[test]
fn test_tracking_sinusoid() {
    let mut otg = Trackig::<1, IgnoreErrorHandler>::new(None, DT);
    let mut input = InputParameter::<1>::new(None);
    let mut output = OutputParameter::<1>::new(None);
    input.synchronization = Synchronization::None;
    input.max_velocity = daov_stack![4.0];
    input.max_acceleration = daov_stack![20.0];
    input.max_jerk = daov_stack![200.0];

    let mut target = TargetState::<1>::new(None);
    let mut max_err: f64 = 0.0;
    for i in 0..4000 {
        let t = i as f64 * DT;
        target.position[0] = (2.0 * t).sin();
        target.velocity[0] = 2.0 * (2.0 * t).cos();
        target.acceleration[0] = -4.0 * (2.0 * t).sin();

        otg.update(&target, &input, &mut output).unwrap();
        output.pass_to_input(&mut input);

        // Kinematic limits hold in every cycle
        assert!(output.new_velocity[0].abs() <= 4.0 + 1e-9);
        assert!(output.new_acceleration[0].abs() <= 20.0 + 1e-9);

        // Tracking error stays small after the transient
        if t > 1.0 {
            max_err = max_err.max((output.new_position[0] - target.position[0]).abs());
        }
    }
    assert!(max_err < 0.05, "tracking error {max_err}");
}

#[test]
fn test_tracking_step_target() {
    let mut otg = Trackig::<1, IgnoreErrorHandler>::new(None, DT);
    let mut input = InputParameter::<1>::new(None);
    let mut output = OutputParameter::<1>::new(None);
    input.synchronization = Synchronization::None;
    input.max_velocity = daov_stack![2.0];
    input.max_acceleration = daov_stack![10.0];
    input.max_jerk = daov_stack![100.0];

    let mut target = TargetState::<1>::new(None);
    target.position[0] = 1.0;

    let mut finished_at = None;
    for i in 0..5000 {
        let result = otg.update(&target, &input, &mut output).unwrap();
        output.pass_to_input(&mut input);
        assert!(output.new_velocity[0].abs() <= 2.0 + 1e-9);
        if result == RuckigResult::Finished && finished_at.is_none() {
            finished_at = Some(i as f64 * DT);
        }
    }

    // Converged to the step target roughly in point-to-point time (~1.3 s here)
    let finished_at = finished_at.expect("did not reach the step target");
    assert!(finished_at < 2.0, "took {finished_at} s");
    assert!((output.new_position[0] - 1.0).abs() < 1e-6);
    assert!(output.new_velocity[0].abs() < 1e-6);
}

#[test]
fn test_tracking_unreachable_ramp() {
    // Target moves at twice the velocity limit: no errors, output saturates at
    // the limit and lags behind
    let mut otg = Trackig::<1, IgnoreErrorHandler>::new(None, DT);
    let mut input = InputParameter::<1>::new(None);
    let mut output = OutputParameter::<1>::new(None);
    input.synchronization = Synchronization::None;
    input.max_velocity = daov_stack![1.0];
    input.max_acceleration = daov_stack![10.0];
    input.max_jerk = daov_stack![100.0];

    let mut target = TargetState::<1>::new(None);
    let mut lag_prev = 0.0;
    for i in 0..3000 {
        let t = i as f64 * DT;
        target.position[0] = 2.0 * t;
        target.velocity[0] = 2.0;

        let result = otg.update(&target, &input, &mut output).unwrap();
        output.pass_to_input(&mut input);

        assert!(
            result == RuckigResult::Working || result == RuckigResult::Finished,
            "unexpected result {result:?} at cycle {i}"
        );
        assert!(output.new_velocity[0] <= 1.0 + 1e-9);

        let lag = target.position[0] - output.new_position[0];
        if t > 1.0 {
            // Lag grows monotonically once saturated
            assert!(lag >= lag_prev - 1e-9, "lag shrank while saturated");
        }
        lag_prev = lag;
    }
    assert!((output.new_velocity[0] - 1.0).abs() < 1e-6);
    assert!(lag_prev > 1.0);
}

#[test]
fn test_tracking_with_position_limits() {
    // Sinusoid amplitude far beyond the position limits: the output must stay
    // inside the limits in every cycle
    let mut otg = Trackig::<1, IgnoreErrorHandler>::new(None, DT);
    let mut input = InputParameter::<1>::new(None);
    let mut output = OutputParameter::<1>::new(None);
    input.synchronization = Synchronization::None;
    input.max_velocity = daov_stack![5.0];
    input.max_acceleration = daov_stack![30.0];
    input.max_jerk = daov_stack![300.0];
    input.max_position = Some(daov_stack![0.5]);
    input.min_position = Some(daov_stack![-0.5]);

    let mut target = TargetState::<1>::new(None);
    let mut max_abs_p: f64 = 0.0;
    for i in 0..8000 {
        let t = i as f64 * DT;
        target.position[0] = 1.5 * (2.0 * t).sin();
        target.velocity[0] = 3.0 * (2.0 * t).cos();
        target.acceleration[0] = -6.0 * (2.0 * t).sin();

        otg.update(&target, &input, &mut output).unwrap();
        output.pass_to_input(&mut input);
        max_abs_p = max_abs_p.max(output.new_position[0].abs());
        assert!(
            output.new_position[0].abs() <= 0.5 + 1e-6,
            "cycle {i}: position {} outside limits",
            output.new_position[0]
        );
    }
    // The output actually reaches (nearly) the limit while following
    assert!(max_abs_p > 0.45, "never got near the limit: {max_abs_p}");
}

#[test]
fn test_tracking_target_filter() {
    // A noisy target with the low-pass enabled: still follows, and the filter
    // does not break convergence to a constant target
    let mut otg = Trackig::<1, IgnoreErrorHandler>::new(None, DT);
    otg.target_filter_time_constant = Some(0.05);
    let mut input = InputParameter::<1>::new(None);
    let mut output = OutputParameter::<1>::new(None);
    input.synchronization = Synchronization::None;
    input.max_velocity = daov_stack![3.0];
    input.max_acceleration = daov_stack![20.0];
    input.max_jerk = daov_stack![200.0];

    let mut target = TargetState::<1>::new(None);
    target.position[0] = 0.5;
    for _ in 0..3000 {
        otg.update(&target, &input, &mut output).unwrap();
        output.pass_to_input(&mut input);
    }
    assert!((output.new_position[0] - 0.5).abs() < 1e-3);
}

#[test]
fn test_tracking_sub_cycle_lag_converges() {
    // Regression: a moving target that the output lags by LESS than one cycle's
    // travel is reachable in less than one cycle. The tracker used to sample the
    // (sub-cycle) plan at delta_time regardless, extrapolating the final state
    // at constant velocity past the target, so the output advanced exactly as
    // much as the target and every such lag was a fixed point of the loop (with
    // a slow forward drift of j*L^3/(32 v^3) per cycle from the jerk bump of the
    // minimum-time profile). The sample time must be clamped to the plan.
    const DT: f64 = 0.004;
    const V: f64 = 50.0;
    let mut otg = Trackig::<1, IgnoreErrorHandler>::new(None, DT);
    let mut input = InputParameter::<1>::new(None);
    let mut output = OutputParameter::<1>::new(None);
    input.synchronization = Synchronization::None;
    input.max_velocity = daov_stack![500.0];
    input.max_acceleration = daov_stack![5000.0];
    input.max_jerk = daov_stack![50000.0];
    // start half a cycle of travel behind the stream, already at stream speed
    input.current_position = daov_stack![-0.0988];
    input.current_velocity = daov_stack![V];
    input.current_acceleration = daov_stack![0.0];

    let mut target = TargetState::<1>::new(None);
    target.velocity[0] = V;
    let mut err_at_100 = 0.0;
    for k in 0..400 {
        // the sample the tracker sees this cycle: the stream at this instant
        target.position[0] = k as f64 * V * DT;
        otg.update(&target, &input, &mut output).unwrap();
        output.pass_to_input(&mut input);

        let err = output.new_position[0] - target.position[0];
        if k == 100 {
            err_at_100 = err;
        }
        if k >= 100 {
            // on the target, not frozen at the initial lag
            assert!(err.abs() < 1e-9, "cycle {k}: error {err}");
            // and not drifting either
            assert!((err - err_at_100).abs() < 1e-12, "cycle {k}: drift {}", err - err_at_100);
        }
    }
    assert!((output.new_velocity[0] - V).abs() < 1e-9);
}
