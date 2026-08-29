//! Tracking interface for following a moving target signal under kinematic constraints
//!
//! While [`Ruckig`](crate::ruckig::Ruckig) generates a trajectory to a fixed target
//! state, [`Trackig`] follows a *moving* target signal (position, velocity,
//! acceleration per DoF) as closely as the kinematic limits allow. Every control
//! cycle the state-to-state calculation is re-run from the current state toward
//! the instantaneous target (reactive tracking).
//!
//! Unreachable targets are handled gracefully: the target state is clamped into
//! the reachable set (kinematic limits, optional position limits), so a target
//! moving faster than the limits allow makes the output lag behind at maximum
//! velocity instead of producing errors. If a single cycle still fails
//! numerically, the last successfully calculated trajectory is followed instead.
//!
//! For tracking, per-DoF time synchronization is usually not wanted: set
//! `input.synchronization = Synchronization::None` (or `Phase`) so that every
//! DoF follows its own time-optimal profile toward the moving target.
//!
//! # Example
//!
//! ```
//! use rsruckig::prelude::*;
//!
//! let mut otg = Trackig::<1, IgnoreErrorHandler>::new(None, 0.01);
//! let mut input = InputParameter::new(None);
//! let mut output = OutputParameter::new(None);
//! input.synchronization = Synchronization::None;
//! input.max_velocity[0] = 3.0;
//! input.max_acceleration[0] = 10.0;
//! input.max_jerk[0] = 100.0;
//!
//! let mut target = TargetState::new(None);
//! for i in 0..100 {
//!     let t = i as f64 * 0.01;
//!     target.position[0] = (2.0 * t).sin();
//!     target.velocity[0] = 2.0 * (2.0 * t).cos();
//!     target.acceleration[0] = -4.0 * (2.0 * t).sin();
//!
//!     let _ = otg.update(&target, &input, &mut output).unwrap();
//!     output.pass_to_input(&mut input);
//! }
//! ```

use crate::brake::{velocity_cap_second_order, velocity_cap_third_order};
use crate::calculator_target::TargetCalculator;
use crate::error::{RuckigError, RuckigErrorHandler};
use crate::input_parameter::InputParameter;
use crate::output_parameter::OutputParameter;
use crate::result::RuckigResult;
use crate::trajectory::Trajectory;
use crate::util::DataArrayOrVec;
use core::marker::PhantomData;

#[cfg(feature = "std")]
use std::time::Instant;

/// Kinematic target state of a tracked signal (per DoF)
#[derive(Debug, Clone, Default)]
pub struct TargetState<const DOF: usize> {
    pub position: DataArrayOrVec<f64, DOF>,
    pub velocity: DataArrayOrVec<f64, DOF>,
    pub acceleration: DataArrayOrVec<f64, DOF>,
}

impl<const DOF: usize> TargetState<DOF> {
    pub fn new(dofs: Option<usize>) -> Self {
        Self {
            position: DataArrayOrVec::new(dofs, 0.0),
            velocity: DataArrayOrVec::new(dofs, 0.0),
            acceleration: DataArrayOrVec::new(dofs, 0.0),
        }
    }
}

/// Copy an optional per-DoF array without allocating when the shape is unchanged
fn assign_optional<T, const DOF: usize>(
    dst: &mut Option<DataArrayOrVec<T, DOF>>,
    src: &Option<DataArrayOrVec<T, DOF>>,
) where
    T: Copy + Clone + Default + core::fmt::Debug,
{
    match (dst.as_mut(), src) {
        (Some(d), Some(s)) if d.len() == s.len() => d.copy_from_slice(s),
        (None, None) => {}
        _ => *dst = src.clone(),
    }
}

/// Online trajectory generator for tracking a moving target signal
///
/// Reactive tracking: every call to [`update`](Trackig::update) recalculates a
/// time-optimal trajectory from the current state toward the instantaneous
/// (clamped) target and steps `delta_time` along it. The steady-state cycle is
/// allocation-free for stack-allocated DoFs (`DOF > 0`).
///
/// For control loops, `Trackig<DOF, IgnoreErrorHandler>` is recommended so that
/// a numerically failing cycle falls back to the last good trajectory instead
/// of returning an error.
#[derive(Debug)]
pub struct Trackig<const DOF: usize, E: RuckigErrorHandler> {
    pub calculator: TargetCalculator<DOF>,
    pub degrees_of_freedom: usize,
    pub delta_time: f64,
    /// Optional first-order low-pass filter time constant (in seconds) applied
    /// to the raw target signal. `None` (default) disables filtering.
    pub target_filter_time_constant: Option<f64>,
    scratch_input: InputParameter<DOF>,
    fallback_trajectory: Trajectory<DOF>,
    have_fallback: bool,
    time_along_trajectory: f64,
    filtered_target: TargetState<DOF>,
    filter_initialized: bool,
    _error_handler: PhantomData<E>,
}

impl<const DOF: usize, E: RuckigErrorHandler> Trackig<DOF, E> {
    pub fn new(dofs: Option<usize>, delta_time: f64) -> Self {
        Self {
            calculator: TargetCalculator::new(dofs),
            degrees_of_freedom: dofs.unwrap_or(DOF),
            delta_time,
            target_filter_time_constant: None,
            scratch_input: InputParameter::new(dofs),
            fallback_trajectory: Trajectory::new(dofs),
            have_fallback: false,
            time_along_trajectory: 0.0,
            filtered_target: TargetState::new(dofs),
            filter_initialized: false,
            _error_handler: PhantomData,
        }
    }

    /// Reset the tracking state (fallback trajectory and target filter)
    pub fn reset(&mut self) {
        self.have_fallback = false;
        self.time_along_trajectory = 0.0;
        self.filter_initialized = false;
    }

    /// Copy the user input into the internal scratch input without heap
    /// allocations in the steady state
    fn copy_input(&mut self, input: &InputParameter<DOF>) {
        let s = &mut self.scratch_input;
        s.degrees_of_freedom = input.degrees_of_freedom;
        s.control_interface = input.control_interface;
        s.synchronization = input.synchronization;
        s.duration_discretization = input.duration_discretization.clone();
        s.current_position.copy_from_slice(&input.current_position);
        s.current_velocity.copy_from_slice(&input.current_velocity);
        s.current_acceleration
            .copy_from_slice(&input.current_acceleration);
        s.max_velocity.copy_from_slice(&input.max_velocity);
        s.max_acceleration.copy_from_slice(&input.max_acceleration);
        s.max_jerk.copy_from_slice(&input.max_jerk);
        s.enabled.copy_from_slice(&input.enabled);
        s.minimum_duration = input.minimum_duration;
        s.interrupt_calculation_duration = input.interrupt_calculation_duration;
        assign_optional(&mut s.min_velocity, &input.min_velocity);
        assign_optional(&mut s.min_acceleration, &input.min_acceleration);
        assign_optional(&mut s.max_position, &input.max_position);
        assign_optional(&mut s.min_position, &input.min_position);
        assign_optional(
            &mut s.per_dof_control_interface,
            &input.per_dof_control_interface,
        );
        assign_optional(
            &mut s.per_dof_synchronization,
            &input.per_dof_synchronization,
        );
        // Deliberate exception to the faithful copy: intermediate waypoints are
        // not supported by the tracking interface (update() rejects them), so
        // the scratch input always stays single-section
        s.intermediate_positions.clear();
        s.per_section_max_velocity = None;
        s.per_section_max_acceleration = None;
        s.per_section_max_jerk = None;
        s.per_section_min_velocity = None;
        s.per_section_min_acceleration = None;
        s.per_section_max_position = None;
        s.per_section_min_position = None;
        s.per_section_minimum_duration = None;
    }

    /// Optionally low-pass filter the raw target signal. Returns whether the
    /// filtered state (true) or the raw target (false) should be used.
    fn filter_target(&mut self, target: &TargetState<DOF>) -> bool {
        let Some(tc) = self.target_filter_time_constant else {
            self.filter_initialized = false;
            return false;
        };

        if !self.filter_initialized {
            self.filtered_target
                .position
                .copy_from_slice(&target.position);
            self.filtered_target
                .velocity
                .copy_from_slice(&target.velocity);
            self.filtered_target
                .acceleration
                .copy_from_slice(&target.acceleration);
            self.filter_initialized = true;
            return true;
        }

        let alpha = self.delta_time / (tc + self.delta_time);
        for dof in 0..self.degrees_of_freedom {
            self.filtered_target.position[dof] +=
                alpha * (target.position[dof] - self.filtered_target.position[dof]);
            self.filtered_target.velocity[dof] +=
                alpha * (target.velocity[dof] - self.filtered_target.velocity[dof]);
            self.filtered_target.acceleration[dof] +=
                alpha * (target.acceleration[dof] - self.filtered_target.acceleration[dof]);
        }
        true
    }

    /// Clamp the target state per DoF into the reachable set of the kinematic
    /// limits (and position limits, if set), so that the state-to-state
    /// calculation cannot be asked for an impossible target.
    fn clamp_target_into_scratch(&mut self, use_filtered: bool, target: &TargetState<DOF>) {
        let s = &mut self.scratch_input;
        for dof in 0..self.degrees_of_freedom {
            let (raw_p, raw_v, raw_a) = if use_filtered {
                (
                    self.filtered_target.position[dof],
                    self.filtered_target.velocity[dof],
                    self.filtered_target.acceleration[dof],
                )
            } else {
                (
                    target.position[dof],
                    target.velocity[dof],
                    target.acceleration[dof],
                )
            };

            let v_max = s.max_velocity[dof];
            let v_min = s.min_velocity.as_ref().map_or(-v_max, |v| v[dof]);
            let a_max = s.max_acceleration[dof];
            let a_min = s.min_acceleration.as_ref().map_or(-a_max, |v| v[dof]);
            let j_max = s.max_jerk[dof];

            let mut af = raw_a.clamp(a_min, a_max);
            if !af.is_finite() {
                af = 0.0;
            }
            let mut vf = raw_v.clamp(v_min, v_max);
            if j_max > 0.0 && j_max.is_finite() {
                // Avoid targets from which the velocity limit would inevitably
                // be exceeded while the acceleration returns to zero
                if af < 0.0 {
                    vf = vf.min(v_max - af * af / (2.0 * j_max));
                }
                if af > 0.0 {
                    vf = vf.max(v_min + af * af / (2.0 * j_max));
                }
            }

            let mut pf = raw_p;
            let max_pos = s.max_position.as_ref().map_or(f64::INFINITY, |v| v[dof]);
            let min_pos = s
                .min_position
                .as_ref()
                .map_or(f64::NEG_INFINITY, |v| v[dof]);
            if s.max_position.is_some() || s.min_position.is_some() {
                pf = pf.clamp(min_pos, max_pos);
                // The stopping distance from the target state must fit inside
                // the limits as well
                let (cap_up, cap_down) = if j_max > 0.0 && j_max.is_finite() {
                    (
                        velocity_cap_third_order(max_pos - pf, a_min, j_max),
                        velocity_cap_third_order(pf - min_pos, a_max, j_max),
                    )
                } else if a_max.is_finite() || a_min.is_finite() {
                    (
                        velocity_cap_second_order(max_pos - pf, a_min),
                        velocity_cap_second_order(pf - min_pos, a_max),
                    )
                } else {
                    (f64::INFINITY, f64::INFINITY)
                };
                // Both signs are bounded by both caps: arriving at the target
                // position against the arrival direction forces a turnaround
                // excursion beyond it (e.g. reaching a point at the upper limit
                // with negative velocity requires approaching from above)
                let cap = cap_up.min(cap_down);
                vf = vf.clamp(-cap, cap);
                if vf == 0.0 {
                    // A resting target pinned at a limit must not accelerate
                    // into the wall
                    if max_pos - pf <= 0.0 {
                        af = af.min(0.0);
                    }
                    if pf - min_pos <= 0.0 {
                        af = af.max(0.0);
                    }
                }
            }

            s.target_position[dof] = pf;
            s.target_velocity[dof] = vf;
            s.target_acceleration[dof] = af;
        }
    }

    /// Copy the single-section trajectory into the fallback slot without
    /// allocating
    fn store_fallback(&mut self, traj: &Trajectory<DOF>) {
        self.fallback_trajectory.duration = traj.duration;
        self.fallback_trajectory.cumulative_times.clear();
        self.fallback_trajectory
            .cumulative_times
            .extend_from_slice(&traj.cumulative_times);
        self.fallback_trajectory
            .independent_min_durations
            .copy_from_slice(&traj.independent_min_durations);
        for dof in 0..self.degrees_of_freedom {
            self.fallback_trajectory.profiles[0][dof] = traj.profiles[0][dof];
        }
    }

    fn restore_fallback(&self, traj: &mut Trajectory<DOF>) {
        traj.duration = self.fallback_trajectory.duration;
        traj.cumulative_times.clear();
        traj.cumulative_times
            .extend_from_slice(&self.fallback_trajectory.cumulative_times);
        traj.independent_min_durations
            .copy_from_slice(&self.fallback_trajectory.independent_min_durations);
        for dof in 0..self.degrees_of_freedom {
            traj.profiles[0][dof] = self.fallback_trajectory.profiles[0][dof];
        }
    }

    /// Follow the target signal for one control cycle
    ///
    /// Recalculates the trajectory from `input`'s current state toward the
    /// (clamped) `target` and steps `delta_time` along it into `output`.
    /// As with [`Ruckig::update`](crate::ruckig::Ruckig::update), pass the new
    /// state back with `output.pass_to_input(&mut input)` in each cycle.
    ///
    /// Returns `Working` while the output still moves toward the current
    /// target, and `Finished` when the target state has been reached (until
    /// the target moves again).
    pub fn update(
        &mut self,
        target: &TargetState<DOF>,
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

        if !input.intermediate_positions.is_empty() {
            E::handle_validation_error(
                "intermediate positions are not supported by the tracking interface.",
            )?;
            return Ok(RuckigResult::ErrorInvalidInput);
        }

        self.copy_input(input);
        let use_filtered = self.filter_target(target);
        self.clamp_target_into_scratch(use_filtered, target);

        let result = self.calculator.calculate::<E>(
            &self.scratch_input,
            &mut output.trajectory,
            self.delta_time,
        )?;

        output.new_calculation = false;
        let mut fallback_exhausted = false;
        if result == RuckigResult::Working {
            self.store_fallback(&output.trajectory);
            self.have_fallback = true;
            // Never sample past the end of the plan. A target that is reachable
            // in less than one cycle (e.g. a moving target the output lags by
            // less than one cycle's travel) yields a trajectory shorter than
            // delta_time; sampling at delta_time would extrapolate the final
            // state at constant velocity past the target, so every such lag
            // would be a fixed point of the tracking loop.
            self.time_along_trajectory = self.delta_time.min(output.trajectory.get_duration());
            output.new_calculation = true;
        } else {
            if !self.have_fallback {
                return Ok(result);
            }
            // Keep following the last successfully calculated trajectory. Never
            // step past its end: hold the final state instead of extrapolating.
            self.restore_fallback(&mut output.trajectory);
            let duration = self.fallback_trajectory.get_duration();
            self.time_along_trajectory = (self.time_along_trajectory + self.delta_time).min(duration);
            fallback_exhausted = self.time_along_trajectory >= duration;
        }

        let old_section = output.new_section;
        output.time = self.time_along_trajectory;
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
        output.did_section_change = output.new_section > old_section;

        #[cfg(feature = "std")]
        {
            output.calculation_duration =
                (Instant::now().duration_since(start).as_nanos() as f64) / 1000.0;
        }
        #[cfg(not(feature = "std"))]
        {
            output.calculation_duration = 0.0;
        }

        if fallback_exhausted {
            // The fallback trajectory has been ridden to its end and the
            // calculation keeps failing: report the actual result code
            return Ok(result);
        }

        // >= : the sample time is clamped to the plan duration above, so a plan
        // that completes within this cycle is sampled exactly at its end
        if output.time >= output.trajectory.get_duration() {
            return Ok(RuckigResult::Finished);
        }

        Ok(RuckigResult::Working)
    }
}
