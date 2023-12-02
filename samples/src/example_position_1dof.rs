use gnuplot::Coordinate::Graph;
use gnuplot::{AxesCommon, Caption, Figure};
use rsruckig::input_parameter::Synchronization;
use rsruckig::{
    input_parameter::InputParameter, output_parameter::OutputParameter, result::RuckigResult,
    ruckig::Ruckig,
};

fn main() {
    let mut otg = Ruckig::new(1, 0.01, true);
    let mut input = InputParameter::new(1);
    let mut output: OutputParameter = OutputParameter::new(1);

    input.current_position = vec![0.0];
    input.current_velocity = vec![7.0];
    input.current_acceleration = vec![0.0];

    input.target_position = vec![10.0];
    input.target_velocity = vec![0.0];
    input.target_acceleration = vec![0.0];

    input.max_velocity = vec![10.0];
    input.max_acceleration = vec![10.0];
    input.max_jerk = vec![30.0];

    input.synchronization = Synchronization::None;

    let mut max_calculation_duration = 0.0;

    let mut x_time: Vec<f64> = Vec::new();
    let mut y_pos: Vec<f64> = Vec::new();
    let mut y_vel: Vec<f64> = Vec::new();
    let mut y_acc: Vec<f64> = Vec::new();
    let mut y_jerk: Vec<f64> = Vec::new();

    while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
        if output.calculation_duration > max_calculation_duration {
            max_calculation_duration = output.calculation_duration;
        }
        x_time.push(output.time);

        y_pos.push(output.new_position[0]);
        y_vel.push(output.new_velocity[0]);
        y_acc.push(output.new_acceleration[0]);
        y_jerk.push(output.new_jerk[0]);

        output.pass_to_input(&mut input);
    }
    println!("Max calculation duration: {} µs", max_calculation_duration);

    let mut fg = Figure::new();
    let chart_title = format!(
        "S-Curve Position Motion Profile. Max. calc duration {} µs",
        max_calculation_duration
    );
    fg.axes2d()
        .set_title(chart_title.as_str(), &[])
        .set_legend(Graph(0.5), Graph(0.9), &[], &[])
        .set_x_label("time in seconds", &[])
        .set_y_label("Position derivatives u, u/s, u/s², u/s³", &[])
        .lines(x_time.clone(), y_pos.clone(), &[Caption("Position")])
        .lines(x_time.clone(), y_vel.clone(), &[Caption("Velocity")])
        .lines(x_time.clone(), y_acc.clone(), &[Caption("Acceleration")])
        .lines(x_time.clone(), y_jerk.clone(), &[Caption("Jerk")]);
    fg.show().unwrap();
}
