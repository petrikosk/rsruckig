use rsruckig::prelude::*;

mod plot_util;
use plot_util::Series;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(3), 0.01);
    let mut input = InputParameter::new(Some(3));
    let mut output = OutputParameter::new(Some(3));

    input.current_position = daov_heap![0.0, 0.0, 0.5];
    input.current_velocity = daov_heap![0.0, -2.2, -0.5];
    input.current_acceleration = daov_heap![0.0, 2.5, -0.5];

    input.target_position = daov_heap![5.0, -2.0, -3.5];
    input.target_velocity = daov_heap![0.0, -0.5, -2.0];
    input.target_acceleration = daov_heap![0.0, 0.0, 0.5];

    input.max_velocity = daov_heap![3.0, 1.0, 3.0];
    input.max_acceleration = daov_heap![3.0, 2.0, 1.0];
    input.max_jerk = daov_heap![4.0, 3.0, 2.0];

    // Set different constraints for negative direction
    input.min_velocity = Some(daov_heap![-2.0, -0.5, -3.0]);
    input.min_acceleration = Some(daov_heap![-2.0, -2.0, -2.0]);

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

    let chart_title = format!(
        "S-Curve Position Motion Profile. Max. calc duration {} µs",
        max_calculation_duration
    );
    plot_util::plot(
        &chart_title,
        "Position 3 DoF",
        &[
            Series::new("Position 1", &x_time, &y_pos1),
            Series::new("Position 2", &x_time, &y_pos2),
            Series::new("Position 3", &x_time, &y_pos3),
        ],
    )
}
