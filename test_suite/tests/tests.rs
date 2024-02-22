use rsruckig::prelude::*;

use float_eq::assert_float_eq;
use rsruckig::input_parameter::{ControlInterface, DurationDiscretization, Synchronization};
use rsruckig::trajectory::Trajectory;

fn almost_equal_vecs(a: &[f64], b: &[f64], epsilon: f64) -> bool {
    if a.len() != b.len() {
        panic!(
            "Length mismatch: left vector has length {}, right vector has length {}",
            a.len(),
            b.len()
        );
    }

    for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
        if (x - y).abs() > epsilon {
            panic!(
                "Values at index {} differ: left = {}, right = {}, difference = {}",
                i,
                x,
                y,
                (x - y).abs()
            );
        }
    }

    true
}

#[test]
// Single DOF
fn test_at_time() {
    // Setup
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.005);
    let mut input = InputParameter::new(None);
    input.current_position = DataArrayOrVec::Stack([0.0]);
    input.target_position = DataArrayOrVec::Stack([1.0]);
    input.max_velocity = DataArrayOrVec::Stack([1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0]);
    input.max_jerk = DataArrayOrVec::Stack([1.0]);

    let mut traj = Trajectory::new(None);
    let _result = otg.calculate(&input, &mut traj);

    let mut output = OutputParameter::new(None);

    // Call the method you want to test
    let result = otg.update(&input, &mut output);

    // Assertions
    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(traj.get_duration(), 3.1748, abs <= 0.000_1);

    let mut new_position = DataArrayOrVec::Stack([0.0; 1]);
    let mut new_velocity = DataArrayOrVec::Stack([0.0; 1]);
    let mut new_acceleration = DataArrayOrVec::Stack([0.0; 1]);
    traj.at_time(
        0.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );
    assert_float_eq!(new_position[0], input.current_position[0], abs <= 0.000_1);

    traj.at_time(
        3.1748 / 2.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );
    assert_float_eq!(new_position[0], 0.5, abs <= 0.000_1);
}

// Secondary features
#[test]
fn test_secondary() {
    // Setup
    let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.005);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);

    input.current_position = DataArrayOrVec::Stack([0.0, -2.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([1.0, -3.0, 2.0]);
    input.target_velocity = DataArrayOrVec::Stack([0.0, 0.3, 0.0]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.max_velocity = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_jerk = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);

    let mut traj = Trajectory::new(None);

    let result = otg.calculate(&input, &mut traj);
    assert_eq!(result.unwrap_or(RuckigResult::Error), RuckigResult::Working);
    assert_float_eq!(traj.get_duration(), 4.0, abs <= 0.000_1);

    let result = otg.update(&input, &mut output);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(output.trajectory.get_duration(), 4.0, abs <= 0.000_1);

    let mut new_position = DataArrayOrVec::Stack([0.0; 3]);
    let mut new_velocity = DataArrayOrVec::Stack([0.0; 3]);
    let mut new_acceleration = DataArrayOrVec::Stack([0.0; 3]);
    let mut new_jerk = DataArrayOrVec::Stack([0.0; 3]);

    output.trajectory.at_time(
        0.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );
    assert_eq!(new_position, input.current_position);
    assert_eq!(new_velocity, input.current_velocity);
    assert_eq!(new_acceleration, input.current_acceleration);

    let mut new_section: Option<usize> = None;

    output.trajectory.at_time(
        output.trajectory.get_duration(),
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );
    assert!(almost_equal_vecs(
        &new_position,
        &input.target_position,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &new_velocity,
        &input.target_velocity,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &new_acceleration,
        &input.target_acceleration,
        0.000_1
    ));

    output.trajectory.at_time(
        2.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut Some(&mut new_jerk),
        &mut new_section,
    );

    assert!(almost_equal_vecs(
        &new_position,
        &[0.5, -2.6871268303003437, 1.0],
        0.000_1
    ));
    assert_eq!(new_jerk, DataArrayOrVec::Stack([0.0, 0.0, -1.0]));
    assert_eq!(new_section, Some(0));

    output.trajectory.at_time(
        5.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut Some(&mut new_jerk),
        &mut new_section,
    );
    assert_eq!(new_jerk, DataArrayOrVec::Stack([0.0, 0.0, 0.0]));
    assert_eq!(new_section, Some(1));

    let independent_min_durations = output.trajectory.get_independent_min_durations();
    assert_float_eq!(independent_min_durations[0], 3.1748021039, abs <= 0.000_1);
    assert_float_eq!(independent_min_durations[1], 3.6860977315, abs <= 0.000_1);
    assert_float_eq!(
        independent_min_durations[2],
        output.trajectory.get_duration(),
        abs <= 0.000_1
    );

    let position_extrema = output.trajectory.get_position_extrema();
    assert_float_eq!(position_extrema[0].t_max, 4.0, abs <= 0.000_1);
    assert_float_eq!(position_extrema[0].max, 1.0, abs <= 0.000_1);
    assert_float_eq!(position_extrema[0].t_min, 0.0, abs <= 0.000_1);
    assert_float_eq!(position_extrema[0].min, 0.0, abs <= 0.000_1);

    assert_float_eq!(position_extrema[1].t_max, 0.0, abs <= 0.000_1);
    assert_float_eq!(position_extrema[1].max, -2.0, abs <= 0.000_1);
    assert_float_eq!(position_extrema[1].t_min, 3.2254033308, abs <= 0.000_1);
    assert_float_eq!(position_extrema[1].min, -3.1549193338, abs <= 0.000_1);

    assert_float_eq!(position_extrema[2].t_max, 4.0, abs <= 0.000_1);
    assert_float_eq!(position_extrema[2].max, 2.0, abs <= 0.000_1);
    assert_float_eq!(position_extrema[2].t_min, 0.0, abs <= 0.000_1);
    assert_float_eq!(position_extrema[2].min, 0.0, abs <= 0.000_1);

    let time = output.trajectory.get_first_time_at_position(0, 0.0);
    assert_ne!(time, None);
    assert_float_eq!(time.unwrap(), 0.0, abs <= 0.000_1);

    let time = output.trajectory.get_first_time_at_position(0, 0.5);
    assert_ne!(time, None);
    assert_float_eq!(time.unwrap(), 2.0, abs <= 0.000_1);

    let time = output.trajectory.get_first_time_at_position(0, 1.0);
    assert_ne!(time, None);
    assert_float_eq!(time.unwrap(), 4.0, abs <= 0.000_1);

    let time = output.trajectory.get_first_time_at_position(1, -3.0);
    assert_ne!(time, None);
    assert_float_eq!(time.unwrap(), 2.6004877902, abs <= 0.000_1);

    let time = output.trajectory.get_first_time_at_position(1, -3.1);
    assert_ne!(time, None);
    assert_float_eq!(time.unwrap(), 2.8644154489, abs <= 0.000_1);

    let time = output.trajectory.get_first_time_at_position(2, 0.05);
    assert_ne!(time, None);
    assert_float_eq!(time.unwrap(), 0.6694329501, abs <= 0.000_1);

    let time = output.trajectory.get_first_time_at_position(0, -1.0);
    assert_eq!(time, None);

    let time = output.trajectory.get_first_time_at_position(1, -3.4);
    assert_eq!(time, None);

    let time = output.trajectory.get_first_time_at_position(6, 0.0);
    assert_eq!(time, None);
    input.current_position = DataArrayOrVec::Stack([0.0, -2.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([1.0, -3.0, 2.0]);
    input.target_velocity = DataArrayOrVec::Stack([2.0, 0.3, 0.0]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.max_velocity = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_jerk = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);

    let result = otg.update(&input, &mut output);

    match result {
        Ok(_) => panic!("Expected an error but got a successful result."),
        Err(e) => {
            let error_message = e.to_string();
            assert!(
                error_message.contains("exceeds its maximum velocity limit"),
                "Unexpected error message: {}",
                error_message
            );
        }
    }
    assert!(!output.new_calculation);

    input.target_velocity = DataArrayOrVec::Stack([0.2, -0.3, 0.8]);
    let result = otg.update(&input, &mut output);
    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert!(output.new_calculation);

    input.minimum_duration = Some(12.0);
    let result = otg.update(&input, &mut output);
    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(output.trajectory.get_duration(), 12.0, abs <= 0.000_1);

    input.current_position = DataArrayOrVec::Stack([1300.0, 0.0, 0.02]);
    input.current_velocity = DataArrayOrVec::Stack([1200.0, 0.0, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([1400.0, 0.0, 0.02]);
    input.target_velocity = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.max_velocity = DataArrayOrVec::Stack([800.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([40000.0, 1.0, 1.0]);
    input.max_jerk = DataArrayOrVec::Stack([200000.0, 1.0, 1.0]);

    input.minimum_duration = None;

    let result = otg.update(&input, &mut output);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(output.trajectory.get_duration(), 0.167347, abs <= 0.000_1);

    let independent_min_durations = output.trajectory.get_independent_min_durations();
    assert_float_eq!(
        independent_min_durations[0],
        output.trajectory.get_duration(),
        abs <= 0.000_1
    );
    assert_float_eq!(independent_min_durations[1], 0.0, abs <= 0.000_1);
    assert_float_eq!(independent_min_durations[2], 0.0, abs <= 0.000_1);
}

#[test]
fn test_enabled() {
    // Setup
    let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.005);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);

    input.enabled = DataArrayOrVec::Stack([true, false, false]);
    input.current_position = DataArrayOrVec::Stack([0.0, -2.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([0.0, 0.1, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([0.0, 0.0, -0.2]);

    input.target_position = DataArrayOrVec::Stack([1.0, -3.0, 2.0]);

    input.max_velocity = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_jerk = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);

    let result = otg.update(&input, &mut output);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(
        output.trajectory.get_duration(),
        3.1748021039,
        abs <= 0.000_1
    );

    let mut new_position = DataArrayOrVec::Stack([0.0; 3]);
    let mut new_velocity = DataArrayOrVec::Stack([0.0; 3]);
    let mut new_acceleration = DataArrayOrVec::Stack([0.0; 3]);

    output.trajectory.at_time(
        0.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );

    assert!(almost_equal_vecs(
        &new_position,
        &input.current_position,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &new_velocity,
        &input.current_velocity,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &new_acceleration,
        &input.current_acceleration,
        0.000_1
    ));

    output.trajectory.at_time(
        output.trajectory.get_duration(),
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );
    assert!(almost_equal_vecs(
        &new_position,
        &[input.target_position[0], -1.6825197896, -1.0079368399],
        0.000_1
    ));

    // Make sure that disabled DoFs overwrite prior blocks
    input.enabled = DataArrayOrVec::Stack([true, true, true]);
    input.current_position = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.target_position = DataArrayOrVec::Stack([100.0, -3000.0, 2000.0]);
    input.target_velocity = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);

    let _result = otg.update(&input, &mut output);

    input.enabled = DataArrayOrVec::Stack([false, false, true]);
    input.current_position = DataArrayOrVec::Stack([0.0, -2.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([0.0, 0.2, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([0.0, 0.2, 0.0]);

    input.target_position = DataArrayOrVec::Stack([1.0, -3.0, 2.0]);
    input.target_velocity = DataArrayOrVec::Stack([0.0, 0.0, 0.2]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, -0.1]);

    input.max_velocity = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_jerk = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);

    let result = otg.update(&input, &mut output);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(
        output.trajectory.get_duration(),
        3.6578610221,
        abs <= 0.000_1
    );
}

#[test]
fn test_phase_synchronization() {
    // Setup
    let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.005);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);
    let mut traj = Trajectory::new(None);

    let mut new_position = DataArrayOrVec::Stack([0.0; 3]);
    let mut new_velocity = DataArrayOrVec::Stack([0.0; 3]);
    let mut new_acceleration = DataArrayOrVec::Stack([0.0; 3]);

    input.current_position = DataArrayOrVec::Stack([0.0, -2.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([1.0, -3.0, 2.0]);

    input.max_velocity = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_jerk = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);

    input.synchronization = Synchronization::Phase;

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert!(almost_equal_vecs(
        &traj.get_profiles()[0][0].t,
        &traj.get_profiles()[0][1].t,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &traj.get_profiles()[0][0].t,
        &traj.get_profiles()[0][2].t,
        0.000_1
    ));

    let result = otg.update(&input, &mut output);

    output.trajectory.at_time(
        1.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );
    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(output.trajectory.get_duration(), 4.0, abs <= 0.000_1);
    assert!(almost_equal_vecs(
        &new_position,
        &[0.0833333333, -2.0833333333, 0.1666666667],
        0.000_1
    ));

    assert!(almost_equal_vecs(
        &output.trajectory.get_profiles()[0][0].t,
        &output.trajectory.get_profiles()[0][1].t,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &output.trajectory.get_profiles()[0][0].t,
        &output.trajectory.get_profiles()[0][2].t,
        0.000_1
    ));

    input.current_position = DataArrayOrVec::Stack([0.0, -2.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([10.0, -3.0, 2.0]);

    input.max_velocity = DataArrayOrVec::Stack([10.0, 2.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([10.0, 2.0, 1.0]);
    input.max_jerk = DataArrayOrVec::Stack([10.0, 2.0, 1.0]);

    let result = otg.update(&input, &mut output);

    output.trajectory.at_time(
        1.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(output.trajectory.get_duration(), 4.0, abs <= 0.000_1);
    assert!(almost_equal_vecs(
        &new_position,
        &[0.8333333333, -2.0833333333, 0.1666666667],
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &output.trajectory.get_profiles()[0][0].t,
        &output.trajectory.get_profiles()[0][1].t,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &output.trajectory.get_profiles()[0][0].t,
        &output.trajectory.get_profiles()[0][2].t,
        0.000_1
    ));

    // Test equal start and target state
    input.current_position = DataArrayOrVec::Stack([1.0, -2.0, 3.0]);
    input.target_position = DataArrayOrVec::Stack([1.0, -2.0, 3.0]);

    let result = otg.update(&input, &mut output);
    output.trajectory.at_time(
        0.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );

    assert_eq!(result.unwrap(), RuckigResult::Finished);
    assert_float_eq!(output.trajectory.get_duration(), 0.0, abs <= 0.000_1);
    assert!(almost_equal_vecs(&new_position, &[1.0, -2.0, 3.0], 0.000_1));
    assert!(almost_equal_vecs(
        &output.trajectory.get_profiles()[0][0].t,
        &output.trajectory.get_profiles()[0][1].t,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &output.trajectory.get_profiles()[0][0].t,
        &output.trajectory.get_profiles()[0][2].t,
        0.000_1
    ));

    input.current_position = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.target_velocity = DataArrayOrVec::Stack([0.2, 0.3, 0.4]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.max_velocity = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_jerk = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);

    let result = otg.calculate(&input, &mut traj);
    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert!(almost_equal_vecs(
        &traj.get_profiles()[0][0].t,
        &traj.get_profiles()[0][1].t,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &traj.get_profiles()[0][0].t,
        &traj.get_profiles()[0][2].t,
        0.000_1
    ));

    input.current_position = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([0.0, 0.0, 0.01]);
    input.target_velocity = DataArrayOrVec::Stack([0.2, 0.3, 0.4]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_ne!(traj.get_profiles()[0][0].t, traj.get_profiles()[0][1].t);
    assert_ne!(traj.get_profiles()[0][0].t, traj.get_profiles()[0][2].t);

    input.current_position = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([0.4, 0.15, 0.2]);
    input.current_acceleration = DataArrayOrVec::Stack([0.8, 0.3, 0.4]);

    input.target_position = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.target_velocity = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert!(almost_equal_vecs(
        &traj.get_profiles()[0][0].t,
        &traj.get_profiles()[0][1].t,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &traj.get_profiles()[0][0].t,
        &traj.get_profiles()[0][2].t,
        0.000_1
    ));

    input.max_velocity = DataArrayOrVec::Stack([1.0, 0.2, 1.0]);

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_ne!(traj.get_profiles()[0][0].t, traj.get_profiles()[0][1].t);
    assert_ne!(traj.get_profiles()[0][0].t, traj.get_profiles()[0][2].t);

    input.current_position = DataArrayOrVec::Stack([0.0, 0.02, 1.0]);
    input.current_velocity = DataArrayOrVec::Stack([-0.2, 0.15, 0.2]);
    input.current_acceleration = DataArrayOrVec::Stack([-0.4, 0.3, 0.4]);

    input.target_position = DataArrayOrVec::Stack([0.03, 0.0, 0.0]);
    input.target_velocity = DataArrayOrVec::Stack([-0.02, 0.015, 0.02]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.max_velocity = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_jerk = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.control_interface = ControlInterface::Velocity;

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert!(almost_equal_vecs(
        &traj.get_profiles()[0][0].t,
        &traj.get_profiles()[0][1].t,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &traj.get_profiles()[0][0].t,
        &traj.get_profiles()[0][2].t,
        0.000_1
    ));

    input.max_jerk = DataArrayOrVec::Stack([1.0, 0.1, 1.0]);

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert!(almost_equal_vecs(
        &traj.get_profiles()[0][0].t,
        &traj.get_profiles()[0][1].t,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &traj.get_profiles()[0][0].t,
        &traj.get_profiles()[0][2].t,
        0.000_1
    ));

    input.target_acceleration = DataArrayOrVec::Stack([0.01, 0.0, 0.0]);

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_ne!(traj.get_profiles()[0][0].t, traj.get_profiles()[0][1].t);
    assert_ne!(traj.get_profiles()[0][0].t, traj.get_profiles()[0][2].t);

    input.current_position = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.target_velocity = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
}

#[test]
fn test_discretion() {
    // Setup
    let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.01);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);
    let mut traj = Trajectory::new(None);

    let mut new_position = DataArrayOrVec::Stack([0.0; 3]);
    let mut new_velocity = DataArrayOrVec::Stack([0.0; 3]);
    let mut new_acceleration = DataArrayOrVec::Stack([0.0; 3]);

    input.current_position = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([1.0, -3.0, 2.0]);
    input.target_velocity = DataArrayOrVec::Stack([0.2, 0.2, 0.2]);

    input.max_velocity = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([2.0, 2.0, 2.0]);
    input.max_jerk = DataArrayOrVec::Stack([1.8, 2.4, 2.0]);

    input.duration_discretization = DurationDiscretization::Discrete;

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(traj.get_duration(), 4.5, abs <= 0.000_1);

    let _result = otg.update(&input, &mut output);
    output.trajectory.at_time(
        4.5,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );
    assert!(almost_equal_vecs(&new_position, &[1.0, -3.0, 2.0], 0.000_1));
}

#[test]
fn test_per_dof_setting() {
    let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.005);
    let mut input = InputParameter::new(None);
    let mut traj = Trajectory::new(None);

    // Test case 1
    input.current_position = DataArrayOrVec::Stack([0.0, -2.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([1.0, -3.0, 2.0]);
    input.target_velocity = DataArrayOrVec::Stack([0.0, 0.3, 0.0]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.max_velocity = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_jerk = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);

    let result = otg.calculate(&input, &mut traj);
    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(traj.get_duration(), 4.0, abs <= 0.000_1);

    let mut new_position = DataArrayOrVec::Stack([0.0; 3]);
    let mut new_velocity = DataArrayOrVec::Stack([0.0; 3]);
    let mut new_acceleration = DataArrayOrVec::Stack([0.0; 3]);

    traj.at_time(
        2.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );
    assert!(almost_equal_vecs(
        &new_position,
        &[0.5, -2.6871268303, 1.0],
        0.000_1
    ));

    input.control_interface = ControlInterface::Velocity;

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(traj.get_duration(), 1.095445115, abs <= 0.000_1);

    traj.at_time(
        1.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );
    assert!(almost_equal_vecs(
        &new_position,
        &[0.0, -1.8641718534, 0.0],
        0.000_1
    ));

    input.per_dof_control_interface = Some(DataArrayOrVec::Stack([
        ControlInterface::Position,
        ControlInterface::Velocity,
        ControlInterface::Position,
    ]));

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(traj.get_duration(), 4.0, abs <= 0.000_1);

    input.per_dof_synchronization = Some(DataArrayOrVec::Stack([
        Synchronization::Time,
        Synchronization::None,
        Synchronization::Time,
    ]));

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(traj.get_duration(), 4.0, abs <= 0.000_1);

    traj.at_time(
        2.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );

    assert!(almost_equal_vecs(
        &new_position,
        &[0.5, -1.5643167673, 1.0],
        0.000_1
    ));

    input.control_interface = ControlInterface::Position;
    input.per_dof_control_interface = None;
    input.per_dof_synchronization = Some(DataArrayOrVec::Stack([
        Synchronization::None,
        Synchronization::Time,
        Synchronization::Time,
    ]));

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(traj.get_duration(), 4.0, abs <= 0.000_1);

    traj.at_time(
        2.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );

    assert!(almost_equal_vecs(
        &new_position,
        &[0.7482143874, -2.6871268303, 1.0],
        0.000_1
    ));

    let independent_min_durations = traj.get_independent_min_durations();

    traj.at_time(
        independent_min_durations[0],
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );

    assert_float_eq!(new_position[0], input.target_position[0], abs <= 0.000_1);

    traj.at_time(
        independent_min_durations[1],
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );

    assert_float_eq!(new_position[1], -3.0890156397, abs <= 0.000_1);

    traj.at_time(
        independent_min_durations[2],
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );

    assert_float_eq!(new_position[2], input.target_position[2], abs <= 0.000_1);

    input.current_position = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([35.0, 35.0, 35.0]);
    input.target_velocity = DataArrayOrVec::Stack([125.0, 125.0, 100.0]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.max_velocity = DataArrayOrVec::Stack([125.0, 125.0, 100.0]);
    input.max_acceleration = DataArrayOrVec::Stack([2000.0, 2000.0, 2000.0]);
    input.max_jerk = DataArrayOrVec::Stack([20000.0, 20000.0, 20000.0]);

    input.per_dof_synchronization = Some(DataArrayOrVec::Stack([
        Synchronization::Time,
        Synchronization::Time,
        Synchronization::None,
    ]));

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(traj.get_duration(), 0.4207106781, abs <= 0.000_1);

    input.current_position = DataArrayOrVec::Stack([0.0, -2.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([0.0, 0.2, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([0.0, 0.2, 0.0]);

    input.target_position = DataArrayOrVec::Stack([1.0, -3.0, 2.0]);
    input.target_velocity = DataArrayOrVec::Stack([0.0, 0.0, 0.2]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, -0.1]);

    input.max_velocity = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_jerk = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);

    input.per_dof_synchronization = Some(DataArrayOrVec::Stack([
        Synchronization::None,
        Synchronization::None,
        Synchronization::Time,
    ]));

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(traj.get_duration(), 3.7885667284, abs <= 0.000_1);

    input.per_dof_synchronization = Some(DataArrayOrVec::Stack([
        Synchronization::None,
        Synchronization::Time,
        Synchronization::None,
    ]));

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(traj.get_duration(), 3.7885667284, abs <= 0.000_1);

    input.enabled = DataArrayOrVec::Stack([true, false, true]);

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(traj.get_duration(), 3.6578610221, abs <= 0.000_1);

    input.current_position = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([0.2, 0.0, -0.1]);
    input.current_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([1.0, -0.2, -0.5]);
    input.target_velocity = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.max_velocity = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_jerk = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);

    input.per_dof_synchronization = Some(DataArrayOrVec::Stack([
        Synchronization::Phase,
        Synchronization::None,
        Synchronization::Phase,
    ]));

    input.enabled = DataArrayOrVec::Stack([true, true, true]);

    let result = otg.calculate(&input, &mut traj);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(traj.get_duration(), 2.848387279, abs <= 0.000_1);
    assert_ne!(traj.get_profiles()[0][0].t, traj.get_profiles()[0][1].t);
    assert!(almost_equal_vecs(
        &traj.get_profiles()[0][0].t,
        &traj.get_profiles()[0][2].t,
        0.000_1
    ));
}

#[test]
fn test_dynamic_dofs() {
    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(3), 0.005);
    let mut input = InputParameter::new(Some(3));
    let mut output = OutputParameter::new(Some(3));

    input.current_position = DataArrayOrVec::Heap(vec![0.0, -2.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Heap(vec![0.0, 0.0, 0.0]);
    input.current_acceleration = DataArrayOrVec::Heap(vec![0.0, 0.0, 0.0]);

    input.target_position = DataArrayOrVec::Heap(vec![1.0, -3.0, 2.0]);
    input.target_velocity = DataArrayOrVec::Heap(vec![0.0, 0.3, 0.0]);
    input.target_acceleration = DataArrayOrVec::Heap(vec![0.0, 0.0, 0.0]);

    input.max_velocity = DataArrayOrVec::Heap(vec![1.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Heap(vec![1.0, 1.0, 1.0]);
    input.max_jerk = DataArrayOrVec::Heap(vec![1.0, 1.0, 1.0]);

    let result = otg.update(&input, &mut output);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(output.trajectory.get_duration(), 4.0, abs <= 0.000_1);

    let mut new_position = DataArrayOrVec::Heap(vec![0.0; 3]);
    let mut new_velocity = DataArrayOrVec::Heap(vec![0.0; 3]);
    let mut new_acceleration = DataArrayOrVec::Heap(vec![0.0; 3]);

    output.trajectory.at_time(
        0.0,
        &mut Some(&mut new_position),
        &mut Some(&mut new_velocity),
        &mut Some(&mut new_acceleration),
        &mut None,
        &mut None,
    );
    assert!(almost_equal_vecs(
        &new_position,
        &input.current_position,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &new_velocity,
        &input.current_velocity,
        0.000_1
    ));
    assert!(almost_equal_vecs(
        &new_acceleration,
        &input.current_acceleration,
        0.000_1
    ));
}

#[test]
fn test_zero_limits() {
    let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.005);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);

    input.current_position = DataArrayOrVec::Stack([0.0, -2.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([0.2, 0.0, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([1.0, -3.0, 0.0]);
    input.target_velocity = DataArrayOrVec::Stack([0.2, 0.0, 0.0]);
    input.target_acceleration = DataArrayOrVec::Stack([0.0, 0.0, 0.0]);

    input.max_velocity = DataArrayOrVec::Stack([1.0, 1.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([0.0, 1.0, 0.0]);
    input.max_jerk = DataArrayOrVec::Stack([0.0, 1.0, 0.0]);

    let result = otg.update(&input, &mut output);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(output.trajectory.get_duration(), 5.0, abs <= 0.000_1);

    input.current_position = DataArrayOrVec::Stack([0.0, -2.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([-0.2, 0.0, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([1.0, 0.0, 0.0]);

    input.target_position = DataArrayOrVec::Stack([0.4, -3.0, 0.0]);
    input.target_velocity = DataArrayOrVec::Stack([0.8, 0.0, 0.0]);
    input.target_acceleration = DataArrayOrVec::Stack([1.0, 0.0, 0.0]);

    input.max_velocity = DataArrayOrVec::Stack([1.0, 200.0, 0.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0, 200.0, 0.0]);
    input.max_jerk = DataArrayOrVec::Stack([0.0, 200.0, 0.0]);

    let result = otg.update(&input, &mut output);

    match result {
        Ok(_) => panic!("Expected an error but got a successful result."),
        Err(e) => {
            let error_message = e.to_string();
            assert!(
                error_message.contains("zero limits conflict in step 1"),
                "Unexpected error message: {}",
                error_message
            );
        }
    }

    input.target_position = DataArrayOrVec::Stack([0.3, -3.0, 0.0]);
    input.max_velocity = DataArrayOrVec::Stack([1.0, 2.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0, 2.0, 0.0]);
    input.max_jerk = DataArrayOrVec::Stack([0.0, 2.0, 0.0]);

    let result = otg.update(&input, &mut output);

    match result {
        Ok(_) => panic!("Expected an error but got a successful result."),
        Err(e) => {
            let error_message = e.to_string();
            assert!(
                error_message.contains("zero limits conflict with other"),
                "Unexpected error message: {}",
                error_message
            );
        }
    }

    input.control_interface = ControlInterface::Velocity;
    input.current_position = DataArrayOrVec::Stack([0.0, -2.0, 0.0]);
    input.current_velocity = DataArrayOrVec::Stack([-0.2, 0.0, 0.0]);
    input.current_acceleration = DataArrayOrVec::Stack([1.0, 0.0, 0.2]);
    input.target_position = DataArrayOrVec::Stack([0.4, -3.0, 0.0]);
    input.target_velocity = DataArrayOrVec::Stack([0.9, 0.5, 0.4]);
    input.target_acceleration = DataArrayOrVec::Stack([1.0, 0.0, 0.2]);
    input.max_velocity = DataArrayOrVec::Stack([1.0, 2.0, 1.0]);
    input.max_acceleration = DataArrayOrVec::Stack([1.0, 2.0, 6.0]);
    input.max_jerk = DataArrayOrVec::Stack([0.0, 2.0, 0.0]);

    let result = otg.update(&input, &mut output);

    match result {
        Ok(_) => panic!("Expected an error but got a successful result."),
        Err(e) => {
            let error_message = e.to_string();
            assert!(
                error_message.contains("zero limits conflict with other"),
                "Unexpected error message: {}",
                error_message
            );
        }
    }

    input.max_jerk = DataArrayOrVec::Stack([1.0, 2.0, 0.0]);

    let result = otg.update(&input, &mut output);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(output.trajectory.get_duration(), 2.0, abs <= 0.000_1);

    input.max_jerk = DataArrayOrVec::Stack([0.0, 2.0, 20.0]);

    let result = otg.update(&input, &mut output);

    assert_eq!(result.unwrap(), RuckigResult::Working);
    assert_float_eq!(output.trajectory.get_duration(), 1.1, abs <= 0.000_1);
}

#[test]
fn test_min_duration() -> Result<(), RuckigError> {
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    let mut input = InputParameter::new(None);

    input.current_position[0] = 0.0;
    input.current_velocity[0] = 0.0;
    input.current_acceleration[0] = 0.0;

    input.target_position[0] = 1.0;
    input.target_velocity[0] = 1.0;
    input.target_acceleration[0] = 1.0;

    input.max_velocity[0] = 1.0;
    input.max_acceleration[0] = 2.0;
    input.max_jerk[0] = 3.0;

    input.minimum_duration = None;
    input.synchronization = Synchronization::None;
    let mut trajectory = Trajectory::new(None);
    otg.calculate(&input, &mut trajectory)?;

    let duration = trajectory.get_duration();

    let mut trajectory_min_duration = Trajectory::new(None);
    input.minimum_duration = Some(5.0);
    otg.calculate(&input, &mut trajectory_min_duration)?;
    let new_duration = trajectory_min_duration.duration.clone();
    dbg!(duration, new_duration);
    assert!(new_duration > duration);
    assert!(new_duration >= 5.0);

    Ok(())
}
