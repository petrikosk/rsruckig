//! Trajectory through intermediate waypoints (mirrors upstream example 03),
//! calculated locally by the pass-through waypoint solver.

use gnuplot::Coordinate::Graph;
use gnuplot::{AxesCommon, Caption, Figure};
use rsruckig::prelude::*;

fn main() {
    let max_number_of_waypoints = 10; // for pre-allocation
    let mut otg = Ruckig::<3, ThrowErrorHandler>::with_waypoints(None, 0.01, max_number_of_waypoints);
    let mut input = InputParameter::<3>::with_waypoints(None, max_number_of_waypoints);
    let mut output = OutputParameter::<3>::with_waypoints(None, max_number_of_waypoints);

    input.current_position = daov_stack![0.2, 0.0, -0.3];
    input.current_velocity = daov_stack![0.0, 0.2, 0.0];
    input.current_acceleration = daov_stack![0.0, 0.6, 0.0];

    input.intermediate_positions.push(daov_stack![1.4, -1.6, 1.0]);
    input.intermediate_positions.push(daov_stack![-0.6, -0.5, 0.4]);
    input.intermediate_positions.push(daov_stack![-0.4, -0.35, 0.0]);
    input.intermediate_positions.push(daov_stack![0.8, 1.8, -0.1]);

    input.target_position = daov_stack![0.5, 1.2, 0.0];
    input.target_velocity = daov_stack![0.0, 0.0, 0.0];
    input.target_acceleration = daov_stack![0.0, 0.0, 0.0];

    input.max_velocity = daov_stack![3.0, 2.0, 3.0];
    input.max_acceleration = daov_stack![6.0, 4.0, 4.0];
    input.max_jerk = daov_stack![16.0, 10.0, 20.0];

    let mut max_calculation_duration = 0.0;

    let mut x_time: Vec<f64> = Vec::new();
    let mut y_pos1: Vec<f64> = Vec::new();
    let mut y_pos2: Vec<f64> = Vec::new();
    let mut y_pos3: Vec<f64> = Vec::new();

    while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
        if output.calculation_duration > max_calculation_duration {
            max_calculation_duration = output.calculation_duration;
        }
        if output.did_section_change {
            println!(
                "t = {:.3}: reached section {} (waypoint {} passed)",
                output.time, output.new_section, output.new_section
            );
        }
        x_time.push(output.time);

        y_pos1.push(output.new_position[0]);
        y_pos2.push(output.new_position[1]);
        y_pos3.push(output.new_position[2]);

        output.pass_to_input(&mut input);
    }
    println!(
        "Trajectory duration: {:.4} s over {} sections",
        output.trajectory.get_duration(),
        output.trajectory.get_profiles().len()
    );
    println!("Max calculation duration: {} µs", max_calculation_duration);

    let mut fg = Figure::new();
    // gnuplot 6 on Windows defaults to the qt terminal, which cannot draw over
    // a pipe (gnuplot bug #1426: black/empty window) - use the native windows
    // terminal instead, unless the user picked one explicitly via GNUTERM
    #[cfg(windows)]
    if std::env::var_os("GNUTERM").is_none() {
        fg.set_terminal("windows", "");
    }
    let chart_title = format!(
        "Trajectory through 4 intermediate waypoints. Max. calc duration {} µs",
        max_calculation_duration
    );
    fg.axes2d()
        .set_title(chart_title.as_str(), &[])
        .set_legend(Graph(0.5), Graph(0.9), &[], &[])
        .set_x_label("time in seconds", &[])
        .set_y_label("Position 3 DoF", &[])
        .lines(x_time.clone(), y_pos1.clone(), &[Caption("Position 1")])
        .lines(x_time.clone(), y_pos2.clone(), &[Caption("Position 2")])
        .lines(x_time.clone(), y_pos3.clone(), &[Caption("Position 3")]);
    fg.show().unwrap();
}
