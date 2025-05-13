use log::{error, info};
use log4rs;
use rsruckig::error::RuckigErrorHandler;
use rsruckig::prelude::*;

// This example shows how to use a custom error handler.
// This error handler uses log4rs to log errors.
// Input data is intentionally wrong to trigger an error.
#[derive(Default, Debug)]
pub struct LogErrorHandler;

impl RuckigErrorHandler for LogErrorHandler {
    fn handle_validation_error(message: &str) -> Result<(), RuckigError> {
        error!("{}", message);
        Ok(())
    }
    fn handle_calculator_error(message: &str) -> Result<(), RuckigError> {
        error!("{}", message);
        Ok(())
    }
}

fn main() {
    log4rs::init_file("logging_config.yaml", Default::default()).unwrap();
    let mut otg = Ruckig::<1, LogErrorHandler>::new(None, 0.01);
    info!("Ruckig initialised.");
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);

    input.current_position[0] = 0.0;
    input.current_velocity[0] = 7.0;
    input.current_acceleration[0] = 0.0;

    input.target_position[0] = 10.0;
    // Below value is intentionally wrong to trigger an error.
    input.target_velocity[0] = 20.0;
    input.target_acceleration[0] = 0.0;

    input.max_velocity[0] = 10.0;
    input.max_acceleration[0] = 10.0;
    input.max_jerk[0] = 30.0;

    input.synchronization = Synchronization::None;

    let mut max_calculation_duration = 0.0;

    while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
        if output.calculation_duration > max_calculation_duration {
            max_calculation_duration = output.calculation_duration;
        }
        output.pass_to_input(&mut input);
    }
    println!("Max calculation duration: {} µs", max_calculation_duration);
}
