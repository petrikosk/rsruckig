//! Tracking example: follow a moving target signal (sinusoid + step) with the
//! Trackig interface under kinematic constraints.

use gnuplot::Coordinate::Graph;
use gnuplot::{AxesCommon, Caption, Color, Figure};
use rsruckig::prelude::*;

fn main() {
    let delta_time = 0.001;
    let mut otg = Trackig::<1, IgnoreErrorHandler>::new(None, delta_time);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);

    input.synchronization = Synchronization::None;
    input.max_velocity[0] = 3.0;
    input.max_acceleration[0] = 15.0;
    input.max_jerk[0] = 150.0;

    let mut target = TargetState::new(None);

    let mut x_time = Vec::new();
    let mut y_target = Vec::new();
    let mut y_tracked = Vec::new();
    let mut y_vel = Vec::new();

    let mut max_calculation_duration: f64 = 0.0;
    for i in 0..8000 {
        let t = i as f64 * delta_time;

        // Sinusoid for 4 seconds, then a step target
        if t < 4.0 {
            target.position[0] = (2.0 * t).sin();
            target.velocity[0] = 2.0 * (2.0 * t).cos();
            target.acceleration[0] = -4.0 * (2.0 * t).sin();
        } else {
            target.position[0] = -1.5;
            target.velocity[0] = 0.0;
            target.acceleration[0] = 0.0;
        }

        let _ = otg.update(&target, &input, &mut output).unwrap();
        output.pass_to_input(&mut input);
        max_calculation_duration = max_calculation_duration.max(output.calculation_duration);

        x_time.push(t);
        y_target.push(target.position[0]);
        y_tracked.push(output.new_position[0]);
        y_vel.push(output.new_velocity[0]);
    }
    println!("Max calculation duration: {max_calculation_duration} µs");

    let mut fg = Figure::new();
    let chart_title = format!(
        "Tracking a moving target. Max. calc duration {max_calculation_duration:.1} µs"
    );
    fg.axes2d()
        .set_title(&chart_title, &[])
        .set_legend(Graph(0.7), Graph(0.9), &[], &[])
        .set_x_label("time in seconds", &[])
        .set_y_label("position / velocity", &[])
        .lines(
            x_time.clone(),
            y_target,
            &[Caption("Target position"), Color("red")],
        )
        .lines(
            x_time.clone(),
            y_tracked,
            &[Caption("Tracked position"), Color("blue")],
        )
        .lines(x_time, y_vel, &[Caption("Velocity"), Color("gray")]);
    fg.show().unwrap();
}
