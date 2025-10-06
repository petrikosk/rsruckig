use plotters::prelude::*;
use rsruckig::prelude::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
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

    // ---- Plotters Section ----

    let chart_title = format!(
        "S-Curve Position Motion Profile (max calc {:.2} µs)",
        max_calculation_duration
    );

    let plot_path = "example_position_3dof.svg";
    let root = SVGBackend::new(plot_path, (1280, 768)).into_drawing_area();
    root.fill(&WHITE)?;

    let x_min = *x_time.first().unwrap_or(&0.0);
    let x_max = *x_time.last().unwrap_or(&1.0);

    // Find y-range across all datasets
    let all_y = y_pos1
        .iter()
        .chain(&y_pos2)
        .chain(&y_pos3)
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
        .y_desc("Position 3 DoF")
        .label_style(("sans-serif", 15))
        .draw()?;

    // Plot each line
    chart
        .draw_series(LineSeries::new(
            x_time.iter().zip(y_pos1.iter()).map(|(&x, &y)| (x, y)),
            &RED,
        ))?
        .label("Position 1")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    chart
        .draw_series(LineSeries::new(
            x_time.iter().zip(y_pos2.iter()).map(|(&x, &y)| (x, y)),
            &BLUE,
        ))?
        .label("Position 2")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    chart
        .draw_series(LineSeries::new(
            x_time.iter().zip(y_pos3.iter()).map(|(&x, &y)| (x, y)),
            &GREEN,
        ))?
        .label("Position 3")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &GREEN));

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    root.present()?;

    println!("Plot saved to '{}'", plot_path);
    Ok(())
}
