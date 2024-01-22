use gnuplot::Coordinate::Graph;
use gnuplot::{AxesCommon, Caption, Figure};
use rsruckig::prelude::*;

fn main() {
    let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.01);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);

    input.current_position = daov_stack![0.0, 0.0, 0.5];
    input.current_velocity = daov_stack![0.0, -2.2, -0.5];
    input.current_acceleration = daov_stack![0.0, 2.5, -0.5];

    input.target_position = daov_stack![5.0, -2.0, -3.5];
    input.target_velocity = daov_stack![0.0, -0.5, -2.0];
    input.target_acceleration = daov_stack![0.0, 0.0, 0.5];

    input.max_velocity = daov_stack![3.0, 1.0, 3.0];
    input.max_acceleration = daov_stack![3.0, 2.0, 1.0];
    input.max_jerk = daov_stack![4.0, 3.0, 2.0];

    // Set different constraints for negative direction
    input.min_velocity = Some(daov_stack![-2.0, -0.5, -3.0]);
    input.min_acceleration = Some(daov_stack![-2.0, -2.0, -2.0]);

    let mut max_calculation_duration = 0.0;

    let mut x_time: Vec<f64> = Vec::new();
    let mut y_pos1: Vec<f64> = Vec::new();
    let mut y_pos2: Vec<f64> = Vec::new();
    let mut y_pos3: Vec<f64> = Vec::new();

    while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
        if output.calculation_duration > max_calculation_duration {
            max_calculation_duration = output.calculation_duration;
        }
        x_time.push(output.time);

        y_pos1.push(output.new_position[0]);
        y_pos2.push(output.new_position[1]);
        y_pos3.push(output.new_position[2]);

        output.pass_to_input(&mut input);
    }
    println!("Max calculation duration: {} µs", max_calculation_duration);
    println!("InputParameter: {}", input);

    let mut fg = Figure::new();
    let chart_title = format!(
        "S-Curve Position Motion Profile. Max. calc duration {} µs",
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
