//! Output parameters containing the generated trajectory and current state
//!
//! This module provides structures to hold the results of trajectory generation,
//! including the full trajectory and the current kinematic state.

use core::fmt;
use core::ops::Deref;

use crate::input_parameter::InputParameter;
use crate::trajectory::Trajectory;
use crate::util::{join, DataArrayOrVec};

/// Output parameters from trajectory generator
///
/// This structure contains the results of trajectory generation:
/// - The calculated trajectory
/// - The current kinematic state at the given time step
/// - Status information about the calculation
///
/// # Type Parameters
///
/// * `DOF` - Number of degrees of freedom. Use a specific number for stack allocation or 0 for heap allocation.
///
/// # Example
///
/// ```
/// use rsruckig::prelude::*;
///
/// // Stack allocation (3 DoF)
/// let mut output = OutputParameter::<3>::new(None);
///
/// // Heap allocation (dynamic DoF)
/// let mut output = OutputParameter::<0>::new(Some(3));
/// ```
#[derive(Debug, Clone)]
pub struct OutputParameter<const DOF: usize> {
    /// Number of degrees of freedom
    pub degrees_of_freedom: usize,
    /// The calculated trajectory
    pub trajectory: Trajectory<DOF>,
    /// Current position for each DoF at the current time step
    pub new_position: DataArrayOrVec<f64, DOF>,
    /// Current velocity for each DoF at the current time step
    pub new_velocity: DataArrayOrVec<f64, DOF>,
    /// Current acceleration for each DoF at the current time step
    pub new_acceleration: DataArrayOrVec<f64, DOF>,
    /// Current jerk for each DoF at the current time step
    pub new_jerk: DataArrayOrVec<f64, DOF>,
    /// Current time along the trajectory
    pub time: f64,
    /// Index of the current section between waypoints
    pub new_section: usize,
    /// Whether the trajectory crossed into a new section in the last update
    pub did_section_change: bool,
    /// Whether a new trajectory was calculated in the last update
    pub new_calculation: bool,
    /// Whether the trajectory calculation was interrupted (time limit reached)
    pub was_calculation_interrupted: bool,
    /// Duration of the calculation in microseconds
    pub calculation_duration: f64,
}

impl<const DOF: usize> Default for OutputParameter<DOF> {
    fn default() -> Self {
        Self::new(None)
    }
}

impl<const DOF: usize> OutputParameter<DOF> {
    pub fn new(dofs: Option<usize>) -> Self {
        Self {
            degrees_of_freedom: dofs.unwrap_or(DOF),
            trajectory: Trajectory::new(dofs),
            new_position: DataArrayOrVec::new(dofs, 0.0),
            new_velocity: DataArrayOrVec::new(dofs, 0.0),
            new_acceleration: DataArrayOrVec::new(dofs, 0.0),
            new_jerk: DataArrayOrVec::new(dofs, 0.0),
            time: 0.0,
            new_section: 0,
            did_section_change: false,
            new_calculation: false,
            was_calculation_interrupted: false,
            calculation_duration: 0.0,
        }
    }
    /// Updates the current kinematic state in the input parameter with the new state from output
    ///
    /// This is a crucial method for continuous trajectory generation. After each update step,
    /// this method should be called to transfer the new kinematic state to the input parameter
    /// for the next calculation.
    ///
    /// # Arguments
    ///
    /// * `input` - The input parameter to update with the current state
    ///
    /// # Example
    ///
    /// ```ignore
    /// use rsruckig::prelude::*;
    ///
    /// // Assume otg, input, and output are properly initialized
    /// while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
    ///     // Use output.new_position, output.new_velocity, etc.
    ///
    ///     // Prepare for the next iteration
    ///     output.pass_to_input(&mut input);
    /// }
    /// ```
    pub fn pass_to_input(&self, input: &mut InputParameter<DOF>) {
        input.current_position = self.new_position.clone();
        input.current_velocity = self.new_velocity.clone();
        input.current_acceleration = self.new_acceleration.clone();
    }
}

impl<const DOF: usize> fmt::Display for OutputParameter<DOF> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(
            f,
            "\nout.new_position = [{}]",
            join::<DOF>(self.new_position.deref(), true)
        )?;
        writeln!(
            f,
            "out.new_velocity = [{}]",
            join::<DOF>(self.new_velocity.deref(), true)
        )?;
        writeln!(
            f,
            "out.new_acceleration = [{}]",
            join::<DOF>(self.new_acceleration.deref(), true)
        )?;
        writeln!(
            f,
            "out.new_jerk = [{}]",
            join::<DOF>(self.new_jerk.deref(), true)
        )?;
        writeln!(f, "out.time = [{}]", self.time)?;
        writeln!(
            f,
            "out.calculation_duration = [{}]",
            self.calculation_duration
        )?;

        Ok(())
    }
}
