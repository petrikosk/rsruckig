//! Position-limit example: an overshooting motion is constrained by max_position.
//!
//! Without limits, the time-optimal trajectory from (p=0, v=8) to p=10 overshoots
//! beyond the target before coming back. With max_position, the same motion is
//! kept inside the allowed workspace.

use gnuplot::Coordinate::Graph;
use gnuplot::{AxesCommon, Caption, Color, DashType, Figure, LineStyle};
use rsruckig::prelude::*;

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

fn main() {
    let (x_free, y_free) = generate(false);
    let (x_lim, y_lim) = generate(true);

    let limit_line_x = [0.0, x_free.last().copied().unwrap_or(1.0).max(
        x_lim.last().copied().unwrap_or(1.0),
    )];
    let limit_line_y = [10.0, 10.0];

    let mut fg = Figure::new();
    // gnuplot 6 on Windows defaults to the qt terminal, which cannot draw over
    // a pipe (gnuplot bug #1426: black/empty window) - use the native windows
    // terminal instead, unless the user picked one explicitly via GNUTERM
    #[cfg(windows)]
    if std::env::var_os("GNUTERM").is_none() {
        fg.set_terminal("windows", "");
    }
    fg.axes2d()
        .set_title("Position limits: overshoot constrained by max\\_position", &[])
        .set_legend(Graph(0.6), Graph(0.3), &[], &[])
        .set_x_label("time in seconds", &[])
        .set_y_label("position", &[])
        .lines(
            x_free,
            y_free,
            &[Caption("Without position limits"), Color("red")],
        )
        .lines(
            x_lim,
            y_lim,
            &[Caption("With max\\_position = 10"), Color("blue")],
        )
        .lines(
            limit_line_x.to_vec(),
            limit_line_y.to_vec(),
            &[
                Caption("max\\_position"),
                Color("black"),
                LineStyle(DashType::Dash),
            ],
        );
    fg.show().unwrap();
}
