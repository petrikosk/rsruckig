//! Main implementation for the Ruckig algorithm.

use crate::calculator_target::TargetCalculator;
use crate::error::{RuckigError, RuckigErrorHandler};
use crate::input_parameter::{DurationDiscretization, InputParameter};
use crate::output_parameter::OutputParameter;
use crate::result::RuckigResult;
use crate::trajectory::Trajectory;
use std::marker::PhantomData;
use std::time::Instant;

pub struct Ruckig<E: RuckigErrorHandler> {
    current_input: InputParameter,
    current_input_initialized: bool,
    pub calculator: TargetCalculator,
    pub degrees_of_freedom: usize,
    pub delta_time: f64,
    _error_handler: PhantomData<E>,
}

impl<E: RuckigErrorHandler> Default for Ruckig<E> {
    fn default() -> Self {
        Self::new(1, 0.01)
    }
}

impl<E: RuckigErrorHandler> Ruckig<E> {
    pub fn new(degrees_of_freedom: usize, delta_time: f64) -> Self {
        Self {
            current_input: InputParameter::new(degrees_of_freedom),
            current_input_initialized: false,
            calculator: TargetCalculator::new(degrees_of_freedom),
            degrees_of_freedom,
            delta_time,
            _error_handler: PhantomData,
        }
    }

    pub fn reset(&mut self) {
        self.current_input_initialized = false;
    }

    /// Validate the input as well as the Ruckig instance for trajectory calculation
    pub fn validate_input(
        &self,
        input: &InputParameter,
        check_current_state_within_limits: bool,
        check_target_state_within_limits: bool,
    ) -> Result<bool, RuckigError> {
        if !input.validate::<E>(
            check_current_state_within_limits,
            check_target_state_within_limits,
        )? {
            return Ok(false);
        }

        if self.delta_time <= 0.0
            && input.duration_discretization != DurationDiscretization::Continuous
        {
            return E::handle_validation_error(&format!(
                "delta time (control rate) parameter {} should be larger than zero.",
                self.delta_time
            ));
        }

        Ok(true)
    }

    pub fn calculate(
        &mut self,
        input: &InputParameter,
        traj: &mut Trajectory,
    ) -> Result<RuckigResult, RuckigError> {
        self.validate_input(input, false, true)?;

        self.calculator.calculate::<E>(input, traj, self.delta_time)
    }

    pub fn update(
        &mut self,
        input: &InputParameter,
        output: &mut OutputParameter,
    ) -> Result<RuckigResult, RuckigError> {
        let start = Instant::now();

        if self.degrees_of_freedom == 0
            && (self.degrees_of_freedom != input.degrees_of_freedom
                || self.degrees_of_freedom != output.degrees_of_freedom)
        {
            return E::handle_calculator_error(
                "mismatch in degrees of freedom (vector size).",
                RuckigResult::Error,
            );
        }

        output.new_calculation = false;

        let result = Ok(RuckigResult::Working);
        if !self.current_input_initialized || *input != self.current_input {
            self.calculate(input, &mut output.trajectory)?;

            self.current_input = input.clone();
            self.current_input_initialized = true;
            output.time = 0.0;
            output.new_calculation = true;
        }

        let old_section = output.new_section;
        output.time += self.delta_time;
        output.trajectory.at_time(
            output.time,
            &mut Some(&mut output.new_position),
            &mut Some(&mut output.new_velocity),
            &mut Some(&mut output.new_acceleration),
            &mut Some(&mut output.new_jerk),
            &mut Some(output.new_section),
        );
        output.did_section_change = output.new_section > old_section; // Report only forward section changes

        let stop = Instant::now();
        output.calculation_duration = (stop.duration_since(start).as_nanos() as f64) / 1000.0;

        output.pass_to_input(&mut self.current_input);

        if output.time > output.trajectory.get_duration() {
            return Ok(RuckigResult::Finished);
        }

        result
    }
}
