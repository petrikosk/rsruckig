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
use crate::calculator_waypoints::{WaypointsCalculator, DEFAULT_REFINEMENT_SWEEPS};
use crate::error::{RuckigError, RuckigErrorHandler};
use crate::input_parameter::{DurationDiscretization, InputParameter};
use crate::output_parameter::OutputParameter;
use crate::result::RuckigResult;
use crate::trajectory::Trajectory;
use crate::util::DataArrayOrVec;
use core::marker::PhantomData;

use crate::alloc::vec::Vec;

// Instant is not available in core
#[cfg(feature = "std")]
use std::time::Instant;

use crate::alloc::format;

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
#[derive(Debug, Clone)]
pub struct Ruckig<const DOF: usize, E: RuckigErrorHandler> {
    current_input: InputParameter<DOF>,
    current_input_initialized: bool,
    pub calculator: TargetCalculator<DOF>,
    /// Calculator for trajectories through intermediate waypoints, created
    /// lazily on the first waypoint calculation (or eagerly by
    /// [`with_waypoints`](Ruckig::with_waypoints))
    waypoints_calculator: Option<WaypointsCalculator<DOF>>,
    /// Number of local refinement sweeps over the waypoint pass-through states
    /// per waypoint calculation. More sweeps yield trajectories closer to
    /// time-optimal at the cost of calculation time; `0` disables refinement
    /// and uses the raw (conservative) heuristic chain.
    pub waypoint_refinement_sweeps: usize,
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
    ///   For stack allocation (DOF>0), this parameter is ignored.
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
            waypoints_calculator: None,
            waypoint_refinement_sweeps: DEFAULT_REFINEMENT_SWEEPS,
            degrees_of_freedom: degrees_of_freedom.unwrap_or(DOF),
            delta_time,
            _error_handler: PhantomData,
        }
    }

    /// Like [`new`](Ruckig::new), but pre-allocates storage for trajectories
    /// with up to `max_number_of_waypoints` intermediate positions, so that
    /// waypoint calculations do not allocate on the first call
    ///
    /// # Example
    ///
    /// ```
    /// use rsruckig::prelude::*;
    ///
    /// let mut otg = Ruckig::<3, ThrowErrorHandler>::with_waypoints(None, 0.01, 8);
    /// let mut input = InputParameter::<3>::with_waypoints(None, 8);
    /// let mut output = OutputParameter::<3>::with_waypoints(None, 8);
    /// ```
    pub fn with_waypoints(
        degrees_of_freedom: Option<usize>,
        delta_time: f64,
        max_number_of_waypoints: usize,
    ) -> Self {
        let mut otg = Self::new(degrees_of_freedom, delta_time);
        otg.current_input
            .intermediate_positions
            .reserve(max_number_of_waypoints);
        otg.waypoints_calculator = Some(WaypointsCalculator::with_waypoints(
            degrees_of_freedom,
            max_number_of_waypoints,
        ));
        otg
    }

    /// Filter intermediate positions that lie within a per-DoF
    /// `threshold_distance` of the linear interpolation between their
    /// neighboring (kept) positions, including the current and target position
    /// as outer anchors. Returns the filtered list, which can be assigned back
    /// to `input.intermediate_positions`.
    ///
    /// Faithful port of the upstream ruckig algorithm: a running intersection
    /// of the per-DoF line-parameter intervals decides whether all skipped
    /// waypoints stay within the threshold corridor. Note that a DoF with zero
    /// extent between the anchors (`pos_end == pos_start`) yields NaN interval
    /// bounds, which `f64::max`/`f64::min` ignore — such a DoF is effectively
    /// unconstrained, matching the upstream behavior.
    pub fn filter_intermediate_positions(
        &self,
        input: &InputParameter<DOF>,
        threshold_distance: &DataArrayOrVec<f64, DOF>,
    ) -> Vec<DataArrayOrVec<f64, DOF>> {
        if input.intermediate_positions.is_empty() {
            return input.intermediate_positions.clone();
        }

        let n_waypoints = input.intermediate_positions.len();
        let mut is_active = crate::alloc::vec![true; n_waypoints];

        let mut start = 0_usize;
        for end in 2..n_waypoints + 2 {
            let pos_start = if start == 0 {
                &input.current_position
            } else {
                &input.intermediate_positions[start - 1]
            };
            let pos_end = if end == n_waypoints + 1 {
                &input.target_position
            } else {
                &input.intermediate_positions[end - 1]
            };

            let mut are_all_below = true;
            'current: for current in start + 1..end {
                let pos_current = &input.intermediate_positions[current - 1];

                let mut t_start_max = 0.0_f64;
                let mut t_end_min = 1.0_f64;
                for dof in 0..self.degrees_of_freedom {
                    let extent = pos_end[dof] - pos_start[dof];
                    let h0 = (pos_current[dof] - pos_start[dof]) / extent;
                    let t_start = h0 - threshold_distance[dof] / extent.abs();
                    let t_end = h0 + threshold_distance[dof] / extent.abs();

                    t_start_max = t_start.max(t_start_max);
                    t_end_min = t_end.min(t_end_min);

                    if t_start_max > t_end_min {
                        are_all_below = false;
                        break 'current;
                    }
                }
            }

            is_active[end - 2] = !are_all_below;
            if !are_all_below {
                start = end - 1;
            }
        }

        let mut filtered_positions = Vec::with_capacity(n_waypoints);
        for (waypoint, active) in input.intermediate_positions.iter().zip(&is_active) {
            if *active {
                filtered_positions.push(waypoint.clone());
            }
        }
        filtered_positions
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

        if !input.intermediate_positions.is_empty() {
            let dofs = if DOF == 0 {
                Some(self.degrees_of_freedom)
            } else {
                None
            };
            let wc = self
                .waypoints_calculator
                .get_or_insert_with(|| WaypointsCalculator::new(dofs));
            wc.refinement_sweeps = self.waypoint_refinement_sweeps;
            return wc.calculate::<E>(input, traj, self.delta_time);
        }

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
        #[cfg(feature = "std")]
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
            let result = self.calculate(input, &mut output.trajectory)?;
            if result != RuckigResult::Working {
                return Ok(result);
            }

            output.time = 0.0; // reset sampling time for the newly calculated trajectory
            output.new_section = 0; // the new trajectory starts in its first section
            output.did_section_change = false;
            output.new_calculation = true;
            // In-place copy reuses `current_input`'s storage instead of allocating fresh
            // Vecs on every retarget (heap mode). Must stay in sync with InputParameter's
            // PartialEq, which drives the `input != current_input` check above.
            self.current_input.copy_from(input);
            self.current_input_initialized = true;
        }

        let old_section = output.new_section;
        output.time += self.delta_time;
        let mut new_section = Some(output.new_section);
        output.trajectory.at_time(
            output.time,
            &mut Some(&mut output.new_position),
            &mut Some(&mut output.new_velocity),
            &mut Some(&mut output.new_acceleration),
            &mut Some(&mut output.new_jerk),
            &mut new_section,
        );
        output.new_section = new_section.unwrap_or(old_section);
        output.did_section_change = output.new_section > old_section; // Report only forward section changes

        #[cfg(feature = "std")]
        {
            let stop = Instant::now();
            output.calculation_duration = (stop.duration_since(start).as_nanos() as f64) / 1000.0;
        }
        #[cfg(not(feature = "std"))]
        {
            output.calculation_duration = 0.0;
        }

        output.pass_to_input(&mut self.current_input);

        if output.time > output.trajectory.get_duration() {
            return Ok(RuckigResult::Finished);
        }

        Ok(RuckigResult::Working)
    }
}
