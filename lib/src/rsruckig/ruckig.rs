//! Main implementation for the Ruckig algorithm.
//!
//! The `Ruckig` struct is the central component of the library. It coordinates:
//! - Input validation
//! - Trajectory calculation
//! - Step-by-step trajectory updates
//!
//! The struct uses generic parameters to define:
//! - The number of degrees of freedom (DoF) for compile-time optimization
//! - The error handler strategy for customized error handling

use crate::calculator_target::TargetCalculator;
use crate::error::{RuckigError, RuckigErrorHandler};
use crate::input_parameter::{DurationDiscretization, InputParameter};
use crate::output_parameter::OutputParameter;
use crate::result::RuckigResult;
use crate::trajectory::Trajectory;
use std::marker::PhantomData;
use std::time::Instant;

/// Main trajectory generation class
///
/// The Ruckig class is responsible for generating time-optimal trajectories
/// with velocity, acceleration, and jerk constraints. It supports both:
/// - Online trajectory generation (step-by-step)
/// - Offline trajectory calculation (all at once)
///
/// # Key Features
///
/// - Jerk-limited trajectories for smooth motion
/// - Time-optimal solutions for point-to-point motions
/// - Support for arbitrary initial and target states
/// - Customizable kinematic constraints (velocity, acceleration, jerk)
/// - Different synchronization behaviors between multiple DoFs
///
/// # Type Parameters
///
/// * `DOF` - Number of degrees of freedom. Use a specific number (>0) for stack allocation or 0 for heap allocation.
/// * `E` - Error handler type that implements RuckigErrorHandler trait.
///
/// # Examples
///
/// Stack allocation (3 DoF):
/// ```
/// use rsruckig::prelude::*;
///
/// // Create a Ruckig instance with 3 DoF and 0.01s control cycle
/// let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.01);
/// ```
///
/// Heap allocation (dynamic DoF):
/// ```
/// use rsruckig::prelude::*;
///
/// // Create a Ruckig instance with 6 DoF and 0.01s control cycle
/// let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(6), 0.01);
/// ```
///
/// # Example
///
/// ```
/// use rsruckig::prelude::*;
///
/// // Stack allocation (3 DoF)
/// let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.01);
///
/// // Heap allocation (3 DoF)
/// let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(3), 0.01);
/// ```
#[derive(Debug)]
pub struct Ruckig<const DOF: usize, E: RuckigErrorHandler> {
    current_input: InputParameter<DOF>,
    current_input_initialized: bool,
    pub calculator: TargetCalculator<DOF>,
    pub degrees_of_freedom: usize,
    pub delta_time: f64,
    _error_handler: PhantomData<E>,
}

impl<const DOF: usize, E: RuckigErrorHandler> Default for Ruckig<DOF, E> {
    fn default() -> Self {
        Self::new(None, 0.01)
    }
}

impl<const DOF: usize, E: RuckigErrorHandler> Ruckig<DOF, E> {
    /// Create a new Ruckig instance
    ///
    /// # Parameters
    ///
    /// * `degrees_of_freedom` - The number of DoF when using heap allocation (DOF=0).
    ///                          For stack allocation (DOF>0), this parameter is ignored.
    /// * `delta_time` - The control cycle duration in seconds.
    ///
    /// # Examples
    ///
    /// ```
    /// use rsruckig::prelude::*;
    /// // Stack allocation with 3 DoF
    /// let mut otg = Ruckig::<3, ThrowErrorHandler>::new(None, 0.01);
    ///
    /// // Heap allocation with 6 DoF
    /// let mut otg = Ruckig::<0, ThrowErrorHandler>::new(Some(6), 0.01);
    /// ```
    pub fn new(degrees_of_freedom: Option<usize>, delta_time: f64) -> Self {
        Self {
            current_input: InputParameter::new(degrees_of_freedom),
            current_input_initialized: false,
            calculator: TargetCalculator::new(degrees_of_freedom),
            degrees_of_freedom: degrees_of_freedom.unwrap_or(DOF),
            delta_time,
            _error_handler: PhantomData,
        }
    }

    /// Reset the internal state of the Ruckig instance
    ///
    /// This method resets the internal state, causing the next call to
    /// `update` to recalculate the trajectory.
    pub fn reset(&mut self) {
        self.current_input_initialized = false;
    }

    /// Validate the input parameters and Ruckig instance for trajectory calculation
    ///
    /// This method checks:
    /// - Input parameter validity through InputParameter::validate
    /// - Compatibility between the duration discretization mode and delta time
    ///
    /// # Parameters
    ///
    /// * `input` - The input parameters to validate
    /// * `check_current_state_within_limits` - Whether to check if the current state is within kinematic limits
    /// * `check_target_state_within_limits` - Whether to check if the target state is within kinematic limits
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If validation passes
    /// * `Err(RuckigError)` - If validation fails (when using ThrowErrorHandler)
    ///
    /// # Note
    ///
    /// When `check_current_state_within_limits` and `check_target_state_within_limits` are both true,
    /// the calculated trajectory is guaranteed to be within the kinematic limits throughout its duration.
    pub fn validate_input(
        &self,
        input: &InputParameter<DOF>,
        check_current_state_within_limits: bool,
        check_target_state_within_limits: bool,
    ) -> Result<(), RuckigError> {
        input.validate::<E>(
            check_current_state_within_limits,
            check_target_state_within_limits,
        )?;

        if self.delta_time <= 0.0
            && input.duration_discretization != DurationDiscretization::Continuous
        {
            return E::handle_validation_error(&format!(
                "delta time (control rate) parameter {} should be larger than zero.",
                self.delta_time
            ));
        }

        Ok(())
    }

    /// Calculate a complete trajectory offline
    ///
    /// This method calculates a complete trajectory without stepping through it.
    /// Use this method for offline trajectory generation when you need the full
    /// trajectory all at once instead of step by step.
    ///
    /// # Arguments
    ///
    /// * `input` - The input parameters defining the trajectory
    /// * `traj` - The trajectory object to store the calculated trajectory
    ///
    /// # Returns
    ///
    /// * `Ok(RuckigResult::Working)` - If the calculation was successful and needs more iterations
    /// * `Ok(RuckigResult::Finished)` - If the calculation was successful and the trajectory is complete
    /// * `Err(RuckigError)` - If an error occurred during calculation (when using ThrowErrorHandler)
    ///
    /// # Example
    ///
    /// ```
    /// use rsruckig::prelude::*;
    ///
    /// let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    /// let mut input = InputParameter::new(None);
    /// let mut trajectory = Trajectory::new(None);
    ///
    /// // Set input parameters
    /// input.current_position[0] = 0.0;
    /// input.target_position[0] = 10.0;
    /// input.max_velocity[0] = 5.0;
    /// input.max_acceleration[0] = 10.0;
    /// input.max_jerk[0] = 20.0;
    ///
    /// // Calculate the entire trajectory at once
    /// let result = otg.calculate(&input, &mut trajectory).unwrap();
    /// println!("Trajectory duration: {}", trajectory.get_duration());
    /// ```
    pub fn calculate(
        &mut self,
        input: &InputParameter<DOF>,
        traj: &mut Trajectory<DOF>,
    ) -> Result<RuckigResult, RuckigError> {
        self.validate_input(input, false, true)?;

        self.calculator.calculate::<E>(input, traj, self.delta_time)
    }

    /// Update the trajectory generation
    ///
    /// This is the main method for step-by-step trajectory generation. It should be called
    /// in each control cycle to get the next state along the trajectory.
    ///
    /// The method performs the following steps:
    /// 1. If the input has changed since the last call, recalculate the trajectory
    /// 2. Step forward in time by `delta_time`
    /// 3. Calculate the new kinematic state at the current time
    /// 4. Update the internal state for the next iteration
    ///
    /// # Arguments
    ///
    /// * `input` - The input parameters defining the trajectory
    /// * `output` - The output parameters to store the results
    ///
    /// # Returns
    ///
    /// * `Ok(RuckigResult::Working)` - If the trajectory is still being executed
    /// * `Ok(RuckigResult::Finished)` - If the trajectory has reached the target
    /// * `Err(RuckigError)` - If an error occurred during calculation (when using ThrowErrorHandler)
    ///
    /// # Example
    ///
    /// ```
    /// use rsruckig::prelude::*;
    ///
    /// let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    /// let mut input = InputParameter::new(None);
    /// let mut output = OutputParameter::new(None);
    ///
    /// // Set input parameters
    /// input.current_position[0] = 0.0;
    /// input.target_position[0] = 10.0;
    /// input.max_velocity[0] = 5.0;
    /// input.max_acceleration[0] = 10.0;
    /// input.max_jerk[0] = 20.0;
    ///
    /// // Generate trajectory step by step
    /// while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
    ///     // Use the new state (output.new_position, output.new_velocity, etc.)
    ///     
    ///     // Pass the new state to the input for the next iteration
    ///     output.pass_to_input(&mut input);
    /// }
    /// ```
    pub fn update(
        &mut self,
        input: &InputParameter<DOF>,
        output: &mut OutputParameter<DOF>,
    ) -> Result<RuckigResult, RuckigError> {
        let start = Instant::now();

        if self.degrees_of_freedom == 0
            && (self.degrees_of_freedom != input.degrees_of_freedom
                || self.degrees_of_freedom != output.degrees_of_freedom)
        {
            E::handle_calculator_error("mismatch in degrees of freedom (vector size).")?;
            return Ok(RuckigResult::Error);
        }

        output.new_calculation = false;

        if !self.current_input_initialized || *input != self.current_input {
            self.calculate(input, &mut output.trajectory)?;

            output.new_calculation = true;
            self.current_input = input.clone();
            self.current_input_initialized = true;
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

        Ok(RuckigResult::Working)
    }
}
