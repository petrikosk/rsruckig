use plotters::prelude::*;
use rsruckig::prelude::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);

    input.current_position[0] = 0.0;
    input.current_velocity[0] = 7.0;
    input.current_acceleration[0] = 0.0;

    input.target_position[0] = 10.0;
    input.target_velocity[0] = 0.0;
    input.target_acceleration[0] = 0.0;

    input.max_velocity[0] = 10.0;
    input.max_acceleration[0] = 10.0;
    input.max_jerk[0] = 30.0;

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

    // ---- Plotters Section ----

    let chart_title = format!(
        "S-Curve Position Motion Profile (max calc {:.2} µs)",
        max_calculation_duration
    );

    let plot_path = "example_position_1dof.svg";
    let root = SVGBackend::new(plot_path, (1280, 768)).into_drawing_area();
    root.fill(&WHITE)?;

    let x_min = *x_time.first().unwrap_or(&0.0);
    let x_max = *x_time.last().unwrap_or(&1.0);

    // Find y-range across all datasets
    let all_y = y_pos
        .iter()
        .chain(&y_vel)
        .chain(&y_acc)
        .chain(&y_jerk)
        .cloned()
        .collect::<Vec<f64>>();
    let y_min = all_y.iter().cloned().fold(f64::INFINITY, f64::min);
    let y_max = all_y.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    let mut chart = ChartBuilder::on(&root)
        .caption(chart_title, ("sans-serif", 25))
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(60)
        .build_cartesian_2d(x_min..x_max, y_min..y_max)?;

    chart
        .configure_mesh()
        .x_desc("Time [s]")
        .y_desc("Position / Velocity / Acceleration / Jerk")
        .label_style(("sans-serif", 15))
        .draw()?;

    // Plot each line
    chart
        .draw_series(LineSeries::new(
            x_time.iter().zip(y_pos.iter()).map(|(&x, &y)| (x, y)),
            &RED,
        ))?
        .label("Position")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    chart
        .draw_series(LineSeries::new(
            x_time.iter().zip(y_vel.iter()).map(|(&x, &y)| (x, y)),
            &BLUE,
        ))?
        .label("Velocity")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    chart
        .draw_series(LineSeries::new(
            x_time.iter().zip(y_acc.iter()).map(|(&x, &y)| (x, y)),
            &GREEN,
        ))?
        .label("Acceleration")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &GREEN));

    chart
        .draw_series(LineSeries::new(
            x_time.iter().zip(y_jerk.iter()).map(|(&x, &y)| (x, y)),
            &MAGENTA,
        ))?
        .label("Jerk")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &MAGENTA));

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    root.present()?;

    println!("Plot saved to '{}'", plot_path);
    Ok(())
}
