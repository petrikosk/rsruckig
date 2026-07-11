//! Position-limit example: an overshooting motion is constrained by max_position.
//!
//! Without limits, the time-optimal trajectory from (p=0, v=8) to p=10 overshoots
//! beyond the target before coming back. With max_position, the same motion is
//! kept inside the allowed workspace.

use rsruckig::prelude::*;

mod plot_util;
use plot_util::Series;

fn generate(with_limits: bool) -> (Vec<f64>, Vec<f64>) {
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.001);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);

    input.current_position[0] = 0.0;
    input.current_velocity[0] = 8.0;
    input.target_position[0] = 8.0;
    input.max_velocity[0] = 10.0;
    input.max_acceleration[0] = 5.0;
    input.max_jerk[0] = 30.0;

    if with_limits {
        input.min_position = Some(daov_stack![-0.5]);
        input.max_position = Some(daov_stack![10.0]);
    }

    let mut x_time = Vec::new();
    let mut y_pos = Vec::new();
    while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
        x_time.push(output.time);
        y_pos.push(output.new_position[0]);
        output.pass_to_input(&mut input);
    }
    (x_time, y_pos)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (x_free, y_free) = generate(false);
    let (x_lim, y_lim) = generate(true);

    let limit_line_x = [0.0, x_free.last().copied().unwrap_or(1.0).max(
        x_lim.last().copied().unwrap_or(1.0),
    )];
    let limit_line_y = [10.0, 10.0];

    plot_util::plot(
        "Position limits: overshoot constrained by max_position",
        "Position",
        &[
            Series::new("Without position limits", &x_free, &y_free),
            Series::new("With max_position = 10", &x_lim, &y_lim),
            Series::dashed("max_position", &limit_line_x, &limit_line_y),
        ],
    )
}
