//! Input parameters for trajectory generation, including constraints and settings
//!
//! This module defines the parameters for trajectory generation, including the current state,
//! target state, and kinematic constraints like velocity, acceleration, and jerk limits.
//! It also provides options for customizing the generation process.

use crate::brake::{stop_distance_second_order, stop_distance_third_order};
use crate::error::{RuckigError, RuckigErrorHandler};
use crate::util::{join, DataArrayOrVec};
use core::fmt;
use core::ops::Deref;

use crate::alloc::format;
use crate::alloc::vec::Vec;

/// Control interface for trajectory generation
///
/// Specifies which kinematic level is being controlled directly:
/// - Position: Full position, velocity, and acceleration control (default)
/// - Velocity: Direct velocity control (useful for visual servoing and stop motions)
/// - Acceleration: Direct acceleration control
///
/// # Example
///
/// ```
/// use rsruckig::prelude::*;
///
/// let mut input = InputParameter::<1>::new(None);
///
/// // For position control (default)
/// input.control_interface = ControlInterface::Position;
///
/// // For velocity control (e.g., for visual servoing)
/// input.control_interface = ControlInterface::Velocity;
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum ControlInterface {
    /// Full kinematic state control (position, velocity, acceleration)
    #[default]
    Position,

    /// Direct velocity control (useful for visual servoing, stop trajectories)
    Velocity,

    /// Direct acceleration control
    Acceleration,
}

/// Synchronization behavior for multi-DoF trajectories
///
/// Controls how multiple degrees of freedom are synchronized during trajectory generation:
/// - Time: All DoFs reach the target at the same time (default)
/// - TimeIfNecessary: Only synchronize if required by other constraints
/// - Phase: All DoFs follow the same phase profile (results in straight-line motions)
/// - None: Each DoF follows its own independent time-optimal profile
///
/// # Example
///
/// ```
/// use rsruckig::prelude::*;
///
/// let mut input = InputParameter::<3>::new(None);
///
/// // For time synchronization (default)
/// input.synchronization = Synchronization::Time;
///
/// // For straight-line motions
/// input.synchronization = Synchronization::Phase;
///
/// // For independent, time-optimal profiles for each DoF
/// input.synchronization = Synchronization::None;
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum Synchronization {
    /// All DoFs reach their target at the same time (default)
    #[default]
    Time,

    /// Only synchronize DoFs if required by other constraints
    TimeIfNecessary,

    /// All DoFs follow the same phase profile (results in straight-line motions)
    Phase,

    /// Each DoF follows its own time-optimal profile independently
    None,
}

/// Duration discretization mode for trajectory timing
///
/// Controls whether the trajectory duration should be:
/// - Continuous: Any duration is allowed (default)
/// - Discrete: Duration must be a multiple of the control cycle
///
/// The discrete option ensures that the exact target state can be reached
/// precisely at a control cycle execution.
///
/// # Example
///
/// ```
/// use rsruckig::prelude::*;
///
/// let mut input = InputParameter::<1>::new(None);
///
/// // Allow any trajectory duration (default)
/// input.duration_discretization = DurationDiscretization::Continuous;
///
/// // Force duration to be a multiple of the control cycle
/// input.duration_discretization = DurationDiscretization::Discrete;
/// ```
#[derive(Debug, Default, Clone, PartialEq)]
pub enum DurationDiscretization {
    /// Any trajectory duration is allowed (default)
    #[default]
    Continuous,

    /// Trajectory duration must be a multiple of the control cycle
    Discrete,
}

/// Input parameters for trajectory generation
///
/// This structure contains all the parameters needed for trajectory generation:
/// - Current kinematic state (position, velocity, acceleration)
/// - Target kinematic state
/// - Kinematic limits (velocity, acceleration, jerk)
/// - Control settings (interface type, synchronization, etc.)
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
/// let mut input = InputParameter::<3>::new(None);
/// input.current_position = daov_stack![0.0, 0.0, 0.0];
/// input.target_position = daov_stack![1.0, 2.0, 3.0];
///
/// // Heap allocation (dynamic DoF)
/// let mut input = InputParameter::<0>::new(Some(3));
/// input.current_position = daov_heap![0.0, 0.0, 0.0];
/// input.target_position = daov_heap![1.0, 2.0, 3.0];
/// ```
#[derive(Debug, Clone)]
pub struct InputParameter<const DOF: usize> {
    /// Number of degrees of freedom
    pub degrees_of_freedom: usize,
    /// Control interface type (Position, Velocity, or Acceleration)
    pub control_interface: ControlInterface,
    /// Synchronization behavior between multiple DoFs
    pub synchronization: Synchronization,
    /// Whether the duration should be a discrete multiple of the control cycle
    pub duration_discretization: DurationDiscretization,
    /// Current position for each DoF
    pub current_position: DataArrayOrVec<f64, DOF>,
    /// Current velocity for each DoF
    pub current_velocity: DataArrayOrVec<f64, DOF>,
    /// Current acceleration for each DoF
    pub current_acceleration: DataArrayOrVec<f64, DOF>,
    /// Target position for each DoF
    pub target_position: DataArrayOrVec<f64, DOF>,
    /// Target velocity for each DoF
    pub target_velocity: DataArrayOrVec<f64, DOF>,
    /// Target acceleration for each DoF
    pub target_acceleration: DataArrayOrVec<f64, DOF>,
    /// Maximum velocity limit for each DoF
    pub max_velocity: DataArrayOrVec<f64, DOF>,
    /// Maximum acceleration limit for each DoF
    pub max_acceleration: DataArrayOrVec<f64, DOF>,
    /// Maximum jerk limit for each DoF
    pub max_jerk: DataArrayOrVec<f64, DOF>,
    /// Minimum velocity limit for each DoF (negative values). If None, negative of max_velocity is used.
    pub min_velocity: Option<DataArrayOrVec<f64, DOF>>,
    /// Minimum acceleration limit for each DoF (negative values). If None, negative of max_acceleration is used.
    pub min_acceleration: Option<DataArrayOrVec<f64, DOF>>,
    /// Maximum position limit for each DoF. If None, no upper position limit is applied.
    /// Use `f64::INFINITY` entries to disable the limit for individual DoFs.
    pub max_position: Option<DataArrayOrVec<f64, DOF>>,
    /// Minimum position limit for each DoF. If None, no lower position limit is applied.
    /// Use `f64::NEG_INFINITY` entries to disable the limit for individual DoFs.
    pub min_position: Option<DataArrayOrVec<f64, DOF>>,
    /// Intermediate waypoint positions the trajectory should pass through (per
    /// waypoint, one position per DoF). Empty by default. Requires the position
    /// control interface and a global synchronization. The trajectory then
    /// consists of `intermediate_positions.len() + 1` sections.
    pub intermediate_positions: Vec<DataArrayOrVec<f64, DOF>>,
    /// Per-section maximum velocity, overwrites the global `max_velocity` for
    /// individual sections. Length must be `intermediate_positions.len() + 1`.
    pub per_section_max_velocity: Option<Vec<DataArrayOrVec<f64, DOF>>>,
    /// Per-section maximum acceleration, overwrites the global `max_acceleration`
    pub per_section_max_acceleration: Option<Vec<DataArrayOrVec<f64, DOF>>>,
    /// Per-section maximum jerk, overwrites the global `max_jerk`
    pub per_section_max_jerk: Option<Vec<DataArrayOrVec<f64, DOF>>>,
    /// Per-section minimum velocity (negative values), overwrites the global `min_velocity`
    pub per_section_min_velocity: Option<Vec<DataArrayOrVec<f64, DOF>>>,
    /// Per-section minimum acceleration (negative values), overwrites the global `min_acceleration`
    pub per_section_min_acceleration: Option<Vec<DataArrayOrVec<f64, DOF>>>,
    /// Per-section maximum position limit, overwrites the global `max_position`
    pub per_section_max_position: Option<Vec<DataArrayOrVec<f64, DOF>>>,
    /// Per-section minimum position limit, overwrites the global `min_position`
    pub per_section_min_position: Option<Vec<DataArrayOrVec<f64, DOF>>>,
    /// Optional minimum duration per section. Length must be `intermediate_positions.len() + 1`.
    pub per_section_minimum_duration: Option<Vec<f64>>,
    /// Whether each DoF is enabled in the calculation
    pub enabled: DataArrayOrVec<bool, DOF>,
    /// Sets the control interface for each DoF individually, overwrites global control_interface
    pub per_dof_control_interface: Option<DataArrayOrVec<ControlInterface, DOF>>,
    /// Sets the synchronization for each DoF individually, overwrites global synchronization
    pub per_dof_synchronization: Option<DataArrayOrVec<Synchronization, DOF>>,
    /// Optional minimum duration of the trajectory
    pub minimum_duration: Option<f64>,
    /// Optional duration after which calculation should be interrupted (for real-time guarantees)
    pub interrupt_calculation_duration: Option<f64>,
}

impl<const DOF: usize> PartialEq for InputParameter<DOF> {
    fn eq(&self, other: &Self) -> bool {
        self.current_position == other.current_position
            && self.current_velocity == other.current_velocity
            && self.current_acceleration == other.current_acceleration
            && self.target_position == other.target_position
            && self.target_velocity == other.target_velocity
            && self.target_acceleration == other.target_acceleration
            && self.max_velocity == other.max_velocity
            && self.max_acceleration == other.max_acceleration
            && self.max_jerk == other.max_jerk
            && self.enabled == other.enabled
            && self.minimum_duration == other.minimum_duration
            && self.min_velocity == other.min_velocity
            && self.min_acceleration == other.min_acceleration
            && self.max_position == other.max_position
            && self.min_position == other.min_position
            && self.control_interface == other.control_interface
            && self.synchronization == other.synchronization
            && self.duration_discretization == other.duration_discretization
            && self.per_dof_control_interface == other.per_dof_control_interface
            && self.per_dof_synchronization == other.per_dof_synchronization
            && self.intermediate_positions == other.intermediate_positions
            && self.per_section_max_velocity == other.per_section_max_velocity
            && self.per_section_max_acceleration == other.per_section_max_acceleration
            && self.per_section_max_jerk == other.per_section_max_jerk
            && self.per_section_min_velocity == other.per_section_min_velocity
            && self.per_section_min_acceleration == other.per_section_min_acceleration
            && self.per_section_max_position == other.per_section_max_position
            && self.per_section_min_position == other.per_section_min_position
            && self.per_section_minimum_duration == other.per_section_minimum_duration
    }
}

impl<const DOF: usize> InputParameter<DOF> {
    /// Overwrite `self` with the contents of `other`, reusing existing storage.
    ///
    /// This is the allocation-free equivalent of `*self = other.clone()` used on the
    /// retarget hot path (`Ruckig::update`): in heap mode (`DOF == 0`) cloning would
    /// allocate a fresh `Vec` for every array field, whereas this copies in place.
    ///
    /// IMPORTANT: this must copy every field compared by [`PartialEq`] (see the `eq`
    /// impl above) so that the `input != current_input` retarget check stays exact.
    /// It also copies the two fields `PartialEq` ignores (`degrees_of_freedom`,
    /// `interrupt_calculation_duration`) so `self` remains a faithful copy of `other`.
    /// Keep this in sync with both the struct definition and `PartialEq`.
    pub fn copy_from(&mut self, other: &Self) {
        #[inline]
        fn copy_opt<T: Clone + Default + core::fmt::Debug, const N: usize>(
            dst: &mut Option<DataArrayOrVec<T, N>>,
            src: &Option<DataArrayOrVec<T, N>>,
        ) {
            match (dst.as_mut(), src) {
                (Some(d), Some(s)) => d.copy_from(s),
                (_, None) => *dst = None,
                (None, Some(s)) => *dst = Some(s.clone()),
            }
        }

        self.degrees_of_freedom = other.degrees_of_freedom;
        self.control_interface = other.control_interface;
        self.synchronization = other.synchronization;
        self.duration_discretization = other.duration_discretization.clone();

        self.current_position.copy_from(&other.current_position);
        self.current_velocity.copy_from(&other.current_velocity);
        self.current_acceleration
            .copy_from(&other.current_acceleration);
        self.target_position.copy_from(&other.target_position);
        self.target_velocity.copy_from(&other.target_velocity);
        self.target_acceleration
            .copy_from(&other.target_acceleration);
        self.max_velocity.copy_from(&other.max_velocity);
        self.max_acceleration.copy_from(&other.max_acceleration);
        self.max_jerk.copy_from(&other.max_jerk);
        self.enabled.copy_from(&other.enabled);

        copy_opt(&mut self.min_velocity, &other.min_velocity);
        copy_opt(&mut self.min_acceleration, &other.min_acceleration);
        copy_opt(&mut self.max_position, &other.max_position);
        copy_opt(&mut self.min_position, &other.min_position);
        copy_opt(
            &mut self.per_dof_control_interface,
            &other.per_dof_control_interface,
        );
        copy_opt(
            &mut self.per_dof_synchronization,
            &other.per_dof_synchronization,
        );

        copy_vec(&mut self.intermediate_positions, &other.intermediate_positions);
        copy_opt_vec(
            &mut self.per_section_max_velocity,
            &other.per_section_max_velocity,
        );
        copy_opt_vec(
            &mut self.per_section_max_acceleration,
            &other.per_section_max_acceleration,
        );
        copy_opt_vec(&mut self.per_section_max_jerk, &other.per_section_max_jerk);
        copy_opt_vec(
            &mut self.per_section_min_velocity,
            &other.per_section_min_velocity,
        );
        copy_opt_vec(
            &mut self.per_section_min_acceleration,
            &other.per_section_min_acceleration,
        );
        copy_opt_vec(
            &mut self.per_section_max_position,
            &other.per_section_max_position,
        );
        copy_opt_vec(
            &mut self.per_section_min_position,
            &other.per_section_min_position,
        );
        match (
            self.per_section_minimum_duration.as_mut(),
            &other.per_section_minimum_duration,
        ) {
            (Some(d), Some(s)) => {
                d.clear();
                d.extend_from_slice(s);
            }
            (_, None) => self.per_section_minimum_duration = None,
            (None, Some(s)) => self.per_section_minimum_duration = Some(s.clone()),
        }

        self.minimum_duration = other.minimum_duration;
        self.interrupt_calculation_duration = other.interrupt_calculation_duration;
    }
}

/// Copy a per-waypoint/per-section list in place, reusing existing per-DoF
/// storage when the list length is unchanged (allocation-free steady state)
fn copy_vec<T: Copy + Clone + Default + core::fmt::Debug, const N: usize>(
    dst: &mut Vec<DataArrayOrVec<T, N>>,
    src: &[DataArrayOrVec<T, N>],
) {
    dst.truncate(src.len());
    let n_reused = dst.len();
    for (d, s) in dst.iter_mut().zip(src) {
        d.copy_from(s);
    }
    for s in &src[n_reused..] {
        dst.push(s.clone());
    }
}

fn copy_opt_vec<T: Copy + Clone + Default + core::fmt::Debug, const N: usize>(
    dst: &mut Option<Vec<DataArrayOrVec<T, N>>>,
    src: &Option<Vec<DataArrayOrVec<T, N>>>,
) {
    match (dst.as_mut(), src) {
        (Some(d), Some(s)) => copy_vec(d, s),
        (_, None) => *dst = None,
        (None, Some(s)) => *dst = Some(s.clone()),
    }
}

impl<const DOF: usize> Default for InputParameter<DOF> {
    fn default() -> Self {
        Self::new(None)
    }
}

impl<const DOF: usize> InputParameter<DOF> {
    pub fn new(dofs: Option<usize>) -> Self {
        Self {
            degrees_of_freedom: dofs.unwrap_or(DOF),
            control_interface: ControlInterface::Position,
            synchronization: Synchronization::Time,
            duration_discretization: DurationDiscretization::Continuous,
            current_position: DataArrayOrVec::new(dofs, 0.0),
            current_velocity: DataArrayOrVec::new(dofs, 0.0),
            current_acceleration: DataArrayOrVec::<f64, DOF>::new(dofs, 0.0),
            target_position: DataArrayOrVec::<f64, DOF>::new(dofs, 0.0),
            target_velocity: DataArrayOrVec::<f64, DOF>::new(dofs, 0.0),
            target_acceleration: DataArrayOrVec::<f64, DOF>::new(dofs, 0.0),
            max_velocity: DataArrayOrVec::<f64, DOF>::new(dofs, 0.0),
            max_acceleration: DataArrayOrVec::<f64, DOF>::new(dofs, f64::INFINITY),
            max_jerk: DataArrayOrVec::<f64, DOF>::new(dofs, f64::INFINITY),
            enabled: DataArrayOrVec::<bool, DOF>::new(dofs, true),
            min_velocity: None,
            min_acceleration: None,
            max_position: None,
            min_position: None,
            intermediate_positions: Vec::new(),
            per_section_max_velocity: None,
            per_section_max_acceleration: None,
            per_section_max_jerk: None,
            per_section_min_velocity: None,
            per_section_min_acceleration: None,
            per_section_max_position: None,
            per_section_min_position: None,
            per_section_minimum_duration: None,
            per_dof_control_interface: None,
            per_dof_synchronization: None,
            minimum_duration: None,
            interrupt_calculation_duration: None,
        }
    }

    /// Like [`new`](InputParameter::new), but pre-allocates storage for up to
    /// `max_number_of_waypoints` intermediate positions so that later waypoint
    /// assignments do not need to grow the list
    pub fn with_waypoints(dofs: Option<usize>, max_number_of_waypoints: usize) -> Self {
        let mut input = Self::new(dofs);
        input.intermediate_positions.reserve(max_number_of_waypoints);
        input
    }

    #[inline]
    pub fn v_at_a_zero(v0: f64, a0: f64, j: f64) -> f64 {
        v0 + (a0 * a0) / (2.0 * j)
    }

    /// Validate the input for trajectory calculation
    pub fn validate<E: RuckigErrorHandler>(
        &self,
        check_current_state_within_limits: bool,
        check_target_state_within_limits: bool,
    ) -> Result<(), RuckigError> {
        self.validate_waypoints::<E>(check_target_state_within_limits)?;

        for dof in 0..self.degrees_of_freedom {
            let j_max = self.max_jerk[dof];
            if j_max.is_nan() || j_max < 0.0 {
                return E::handle_validation_error(&format!(
                    "Maximum jerk limit {} of DoF {} should be larger than or equal to zero.",
                    j_max, dof
                ));
            }

            let a_max: f64 = self.max_acceleration[dof];
            if a_max.is_nan() || a_max < 0.0 {
                return E::handle_validation_error(&format!("maximum acceleration limit {} of DoF {} should be larger than or equal to zero.", a_max, dof));
            }

            let a_min: f64 = match &self.min_acceleration {
                Some(min_acc) => min_acc.deref()[dof],
                None => -self.max_acceleration.deref()[dof],
            };
            if a_min.is_nan() || a_min > 0.0 {
                return E::handle_validation_error(&format!("minimum acceleration limit {} of DoF {} should be smaller than or equal to zero.", a_min, dof));
            }

            let max_pos = match &self.max_position {
                Some(mp) => mp[dof],
                None => f64::INFINITY,
            };
            let min_pos = match &self.min_position {
                Some(mp) => mp[dof],
                None => f64::NEG_INFINITY,
            };
            let position_limits_set = self.max_position.is_some() || self.min_position.is_some();
            if position_limits_set {
                if max_pos.is_nan() || min_pos.is_nan() {
                    return E::handle_validation_error(&format!(
                        "position limits [{}, {}] of DoF {} should be valid numbers.",
                        min_pos, max_pos, dof
                    ));
                }
                if min_pos > max_pos {
                    return E::handle_validation_error(&format!(
                        "minimum position limit {} of DoF {} should be smaller than or equal to its maximum position limit {}.",
                        min_pos, dof, max_pos
                    ));
                }
            }

            let a0: f64 = self.current_acceleration[dof];
            if a0.is_nan() {
                return E::handle_validation_error(&format!(
                    "current acceleration {} of DoF {} should be a valid number.",
                    a0, dof
                ));
            }

            let af: f64 = self.target_acceleration[dof];
            if af.is_nan() {
                return E::handle_validation_error(&format!(
                    "target acceleration {} of DoF {} should be a valid number.",
                    af, dof
                ));
            }

            if check_current_state_within_limits {
                if a0 > a_max {
                    return E::handle_validation_error(&format!("current acceleration {} of DoF {} exceeds its maximum acceleration limit {}.", a0, dof, a_max));
                }
                if a0 < a_min {
                    return E::handle_validation_error(&format!("current acceleration {} of DoF {} undercuts its minimum acceleration limit {}.", a0, dof, a_min));
                }
            }
            if check_target_state_within_limits {
                if af > a_max {
                    return E::handle_validation_error(&format!("target acceleration {} of DoF {} exceeds its maximum acceleration limit {}.", af, dof, a_max));
                }
                if af < a_min {
                    return E::handle_validation_error(&format!("target acceleration {} of DoF {} undercuts its minimum acceleration limit {}.", af, dof, a_min));
                }
            }

            let v0 = self.current_velocity[dof];
            if v0.is_nan() {
                return E::handle_validation_error(&format!(
                    "current velocity {} of DoF {} should be a valid number.",
                    v0, dof
                ));
            }
            let vf = self.target_velocity[dof];
            if vf.is_nan() {
                return E::handle_validation_error(&format!(
                    "target velocity {} of DoF {} should be a valid number.",
                    vf, dof
                ));
            }

            let control_interface_ = match &self.per_dof_control_interface {
                Some(per_dof) => match per_dof.get(dof) {
                    Some(interface) => interface, // assuming get() returns a reference; de-reference it
                    None => &self.control_interface,
                },
                None => &self.control_interface,
            };

            if let ControlInterface::Position = control_interface_ {
                let p0 = self.current_position[dof];
                if p0.is_nan() {
                    return E::handle_validation_error(&format!(
                        "current position {} of DoF {} should be a valid number.",
                        p0, dof
                    ));
                }
                let pf = self.target_position[dof];
                if pf.is_nan() {
                    return E::handle_validation_error(&format!(
                        "target position {} of DoF {} should be a valid number.",
                        pf, dof
                    ));
                }

                let v_max = self.max_velocity[dof];
                if v_max.is_nan() || v_max < 0.0 {
                    return E::handle_validation_error(&format!("maximum velocity limit {} of DoF {} should be larger than or equal to zero.", v_max, dof));
                }

                let v_min = if let Some(min_velocity) = &self.min_velocity {
                    min_velocity[dof]
                } else {
                    -v_max
                };
                if v_min.is_nan() || v_min > 0.0 {
                    return E::handle_validation_error(&format!("minimum velocity limit {} of DoF {} should be smaller than or equal to zero.", v_min, dof));
                }

                if check_current_state_within_limits {
                    if v0 > v_max {
                        return E::handle_validation_error(&format!(
                            "current velocity {} of DoF {} exceeds its maximum velocity limit {}.",
                            v0, dof, v_max
                        ));
                    }
                    if v0 < v_min {
                        return E::handle_validation_error(&format!("current velocity {} of DoF {} undercuts its minimum velocity limit {}.", v0, dof, v_min));
                    }
                }
                if check_target_state_within_limits {
                    if vf > v_max {
                        return E::handle_validation_error(&format!(
                            "target velocity {} of DoF {} exceeds its maximum velocity limit {}.",
                            vf, dof, v_max
                        ));
                    }
                    if vf < v_min {
                        return E::handle_validation_error(&format!(
                            "target velocity {} of DoF {} undercuts its minimum velocity limit {}.",
                            vf, dof, v_min
                        ));
                    }
                }
                if check_current_state_within_limits {
                    if a0 > 0.0
                        && j_max > 0.0
                        && InputParameter::<DOF>::v_at_a_zero(v0, a0, j_max) > v_max
                    {
                        return E::handle_validation_error(&format!("DoF {} will inevitably reach a velocity {} from the current kinematic state that will exceed its maximum velocity limit {}.", dof, InputParameter::<DOF>::v_at_a_zero(v0, a0, j_max), v_max));
                    }
                    if a0 < 0.0
                        && j_max > 0.0
                        && InputParameter::<DOF>::v_at_a_zero(v0, a0, -j_max) < v_min
                    {
                        return E::handle_validation_error(&format!("DoF {} will inevitably reach a velocity {} from the current kinematic state that will undercut its minimum velocity limit {}.", dof, InputParameter::<DOF>::v_at_a_zero(v0, a0, -j_max), v_min));
                    }
                }
                if check_target_state_within_limits {
                    if af < 0.0
                        && j_max > 0.0
                        && InputParameter::<DOF>::v_at_a_zero(vf, af, j_max) > v_max
                    {
                        return E::handle_validation_error(&format!("DoF {} will inevitably have reached a velocity {} from the target kinematic state that will exceed its maximum velocity limit {}.", dof, InputParameter::<DOF>::v_at_a_zero(vf, af, j_max), v_max));
                    }
                    if af > 0.0
                        && j_max > 0.0
                        && InputParameter::<DOF>::v_at_a_zero(vf, af, -j_max) < v_min
                    {
                        return E::handle_validation_error(&format!("DoF {} will inevitably have reached a velocity {} from the target kinematic state that will undercut its minimum velocity limit {}.", dof, InputParameter::<DOF>::v_at_a_zero(vf, af, -j_max), v_min));
                    }
                }

                if position_limits_set {
                    let a_min = match &self.min_acceleration {
                        Some(min_acc) => min_acc[dof],
                        None => -a_max,
                    };
                    let stop_distances = |v: f64, a: f64| -> (f64, f64) {
                        if j_max > 0.0 && j_max.is_finite() {
                            (
                                stop_distance_third_order(v, a, a_min, j_max),
                                stop_distance_third_order(-v, -a, a_max, j_max),
                            )
                        } else if a_max.is_finite() || a_min.is_finite() {
                            (
                                stop_distance_second_order(v, a_min),
                                stop_distance_second_order(-v, a_max),
                            )
                        } else {
                            // First-order profiles are monotone: no overshoot possible
                            (0.0, 0.0)
                        }
                    };

                    if check_target_state_within_limits {
                        if pf > max_pos {
                            return E::handle_validation_error(&format!("target position {} of DoF {} exceeds its maximum position limit {}.", pf, dof, max_pos));
                        }
                        if pf < min_pos {
                            return E::handle_validation_error(&format!("target position {} of DoF {} undercuts its minimum position limit {}.", pf, dof, min_pos));
                        }

                        let (stop_up, stop_down) = stop_distances(vf, af);
                        if stop_up > max_pos - pf {
                            return E::handle_validation_error(&format!("DoF {} will inevitably travel a stopping distance {} from the target kinematic state that will exceed its maximum position limit {}.", dof, stop_up, max_pos));
                        }
                        if stop_down > pf - min_pos {
                            return E::handle_validation_error(&format!("DoF {} will inevitably travel a stopping distance {} from the target kinematic state that will undercut its minimum position limit {}.", dof, stop_down, min_pos));
                        }
                    }
                    if check_current_state_within_limits {
                        if p0 > max_pos {
                            return E::handle_validation_error(&format!("current position {} of DoF {} exceeds its maximum position limit {}.", p0, dof, max_pos));
                        }
                        if p0 < min_pos {
                            return E::handle_validation_error(&format!("current position {} of DoF {} undercuts its minimum position limit {}.", p0, dof, min_pos));
                        }

                        // Inevitable-crossing guard: even the hardest brake must fit
                        // inside the remaining distance to the position limits.
                        let (stop_up, stop_down) = stop_distances(v0, a0);
                        if stop_up > max_pos - p0 {
                            return E::handle_validation_error(&format!("DoF {} will inevitably travel a stopping distance {} from the current kinematic state that will exceed its maximum position limit {}.", dof, stop_up, max_pos));
                        }
                        if stop_down > p0 - min_pos {
                            return E::handle_validation_error(&format!("DoF {} will inevitably travel a stopping distance {} from the current kinematic state that will undercut its minimum position limit {}.", dof, stop_down, min_pos));
                        }
                    }
                }
            } else if position_limits_set && check_current_state_within_limits {
                let p0 = self.current_position[dof];
                if p0 > max_pos {
                    return E::handle_validation_error(&format!("current position {} of DoF {} exceeds its maximum position limit {}.", p0, dof, max_pos));
                }
                if p0 < min_pos {
                    return E::handle_validation_error(&format!("current position {} of DoF {} undercuts its minimum position limit {}.", p0, dof, min_pos));
                }
            }
        }
        Ok(())
    }

    /// Validate the intermediate waypoints and per-section constraints
    fn validate_waypoints<E: RuckigErrorHandler>(
        &self,
        check_target_state_within_limits: bool,
    ) -> Result<(), RuckigError> {
        let n_waypoints = self.intermediate_positions.len();
        let n_sections = n_waypoints + 1;

        // Fast path: nothing waypoint-related is set (keeps the plain
        // state-to-state validation hot path free of the checks below)
        if n_waypoints == 0
            && self.per_section_max_velocity.is_none()
            && self.per_section_max_acceleration.is_none()
            && self.per_section_max_jerk.is_none()
            && self.per_section_min_velocity.is_none()
            && self.per_section_min_acceleration.is_none()
            && self.per_section_max_position.is_none()
            && self.per_section_min_position.is_none()
            && self.per_section_minimum_duration.is_none()
        {
            return Ok(());
        }

        if n_waypoints > 0 {
            if self.control_interface != ControlInterface::Position
                || self.per_dof_control_interface.is_some()
                || self.per_dof_synchronization.is_some()
            {
                return E::handle_validation_error("intermediate positions can only be used together with the position control interface and a global synchronization.");
            }
            if self.minimum_duration.is_some()
                || self.duration_discretization != DurationDiscretization::Continuous
            {
                return E::handle_validation_error("intermediate positions can not be used together with a global minimum or discrete duration.");
            }
            for dof in 0..self.degrees_of_freedom {
                if self.max_jerk[dof].is_infinite() {
                    return E::handle_validation_error(&format!("infinite jerk limit of DoF {} is currently not supported with intermediate positions.", dof));
                }
            }

            for (index, waypoint) in self.intermediate_positions.iter().enumerate() {
                if waypoint.len() != self.degrees_of_freedom {
                    return E::handle_validation_error(&format!(
                        "intermediate position {} has {} DoFs, expected {}.",
                        index,
                        waypoint.len(),
                        self.degrees_of_freedom
                    ));
                }
                for dof in 0..self.degrees_of_freedom {
                    let w = waypoint[dof];
                    if !w.is_finite() {
                        return E::handle_validation_error(&format!(
                            "intermediate position {} of DoF {} should be a valid number.",
                            index, dof
                        ));
                    }
                    // Global position limits; per-section limits are checked below
                    if let Some(max_pos) = &self.max_position {
                        if w > max_pos[dof] {
                            return E::handle_validation_error(&format!("intermediate position {} of DoF {} exceeds its maximum position limit {}.", index, dof, max_pos[dof]));
                        }
                    }
                    if let Some(min_pos) = &self.min_position {
                        if w < min_pos[dof] {
                            return E::handle_validation_error(&format!("intermediate position {} of DoF {} undercuts its minimum position limit {}.", index, dof, min_pos[dof]));
                        }
                    }
                }
            }
        }

        // Per-section constraint arrays must match the number of sections and
        // follow the same per-DoF rules as their global counterparts
        struct SectionArrayCheck {
            name: &'static str,
            is_min: bool, // min limits must be <= 0, max limits must be >= 0
            is_position: bool,
        }
        let checks = [
            ("per_section_max_velocity", &self.per_section_max_velocity, SectionArrayCheck { name: "maximum velocity", is_min: false, is_position: false }),
            ("per_section_max_acceleration", &self.per_section_max_acceleration, SectionArrayCheck { name: "maximum acceleration", is_min: false, is_position: false }),
            ("per_section_max_jerk", &self.per_section_max_jerk, SectionArrayCheck { name: "maximum jerk", is_min: false, is_position: false }),
            ("per_section_min_velocity", &self.per_section_min_velocity, SectionArrayCheck { name: "minimum velocity", is_min: true, is_position: false }),
            ("per_section_min_acceleration", &self.per_section_min_acceleration, SectionArrayCheck { name: "minimum acceleration", is_min: true, is_position: false }),
            ("per_section_max_position", &self.per_section_max_position, SectionArrayCheck { name: "maximum position", is_min: false, is_position: true }),
            ("per_section_min_position", &self.per_section_min_position, SectionArrayCheck { name: "minimum position", is_min: true, is_position: true }),
        ];
        for (field_name, array, check) in checks {
            let Some(sections) = array else { continue };
            if sections.len() != n_sections {
                return E::handle_validation_error(&format!(
                    "{} has {} entries, expected one per section ({}).",
                    field_name,
                    sections.len(),
                    n_sections
                ));
            }
            for (section, values) in sections.iter().enumerate() {
                if values.len() != self.degrees_of_freedom {
                    return E::handle_validation_error(&format!(
                        "{} of section {} has {} DoFs, expected {}.",
                        field_name,
                        section,
                        values.len(),
                        self.degrees_of_freedom
                    ));
                }
                for dof in 0..self.degrees_of_freedom {
                    let value = values[dof];
                    if value.is_nan() {
                        return E::handle_validation_error(&format!(
                            "per-section {} {} of section {}, DoF {} should be a valid number.",
                            check.name, value, section, dof
                        ));
                    }
                    if !check.is_position {
                        if !check.is_min && value < 0.0 {
                            return E::handle_validation_error(&format!("per-section {} limit {} of section {}, DoF {} should be larger than or equal to zero.", check.name, value, section, dof));
                        }
                        if check.is_min && value > 0.0 {
                            return E::handle_validation_error(&format!("per-section {} limit {} of section {}, DoF {} should be smaller than or equal to zero.", check.name, value, section, dof));
                        }
                    }
                }
            }
        }

        // Per-section position limits: consistent ranges, and each waypoint must
        // lie within the limits of both adjacent sections
        if self.per_section_max_position.is_some() || self.per_section_min_position.is_some() {
            for section in 0..n_sections {
                for dof in 0..self.degrees_of_freedom {
                    let max_pos = self
                        .per_section_max_position
                        .as_ref()
                        .map_or(f64::INFINITY, |v| v[section][dof]);
                    let min_pos = self
                        .per_section_min_position
                        .as_ref()
                        .map_or(f64::NEG_INFINITY, |v| v[section][dof]);
                    if min_pos > max_pos {
                        return E::handle_validation_error(&format!("per-section minimum position limit {} of section {}, DoF {} should be smaller than or equal to its maximum position limit {}.", min_pos, section, dof, max_pos));
                    }
                    // Waypoint k joins sections k-1 and k: it must satisfy both
                    for waypoint_index in [section.checked_sub(1), Some(section)]
                        .into_iter()
                        .flatten()
                    {
                        if waypoint_index >= n_waypoints {
                            continue;
                        }
                        let w = self.intermediate_positions[waypoint_index][dof];
                        if w > max_pos || w < min_pos {
                            return E::handle_validation_error(&format!("intermediate position {} of DoF {} violates the position limits [{}, {}] of adjacent section {}.", waypoint_index, dof, min_pos, max_pos, section));
                        }
                    }
                }
            }
        }

        // The target state must satisfy the limits of the last section
        if check_target_state_within_limits {
            let last = n_sections - 1;
            for dof in 0..self.degrees_of_freedom {
                if let Some(v) = &self.per_section_max_velocity {
                    if self.target_velocity[dof] > v[last][dof] {
                        return E::handle_validation_error(&format!("target velocity {} of DoF {} exceeds the maximum velocity limit {} of the last section.", self.target_velocity[dof], dof, v[last][dof]));
                    }
                }
                if let Some(v) = &self.per_section_min_velocity {
                    if self.target_velocity[dof] < v[last][dof] {
                        return E::handle_validation_error(&format!("target velocity {} of DoF {} undercuts the minimum velocity limit {} of the last section.", self.target_velocity[dof], dof, v[last][dof]));
                    }
                }
                if let Some(a) = &self.per_section_max_acceleration {
                    if self.target_acceleration[dof] > a[last][dof] {
                        return E::handle_validation_error(&format!("target acceleration {} of DoF {} exceeds the maximum acceleration limit {} of the last section.", self.target_acceleration[dof], dof, a[last][dof]));
                    }
                }
                if let Some(a) = &self.per_section_min_acceleration {
                    if self.target_acceleration[dof] < a[last][dof] {
                        return E::handle_validation_error(&format!("target acceleration {} of DoF {} undercuts the minimum acceleration limit {} of the last section.", self.target_acceleration[dof], dof, a[last][dof]));
                    }
                }
                if let Some(p) = &self.per_section_max_position {
                    if self.target_position[dof] > p[last][dof] {
                        return E::handle_validation_error(&format!("target position {} of DoF {} exceeds the maximum position limit {} of the last section.", self.target_position[dof], dof, p[last][dof]));
                    }
                }
                if let Some(p) = &self.per_section_min_position {
                    if self.target_position[dof] < p[last][dof] {
                        return E::handle_validation_error(&format!("target position {} of DoF {} undercuts the minimum position limit {} of the last section.", self.target_position[dof], dof, p[last][dof]));
                    }
                }
            }
        }

        if let Some(durations) = &self.per_section_minimum_duration {
            if durations.len() != n_sections {
                return E::handle_validation_error(&format!(
                    "per_section_minimum_duration has {} entries, expected one per section ({}).",
                    durations.len(),
                    n_sections
                ));
            }
            for (section, duration) in durations.iter().enumerate() {
                if !duration.is_finite() || *duration < 0.0 {
                    return E::handle_validation_error(&format!(
                        "per-section minimum duration {} of section {} should be a non-negative number.",
                        duration, section
                    ));
                }
            }
        }

        Ok(())
    }
}

impl<const DOF: usize> fmt::Display for InputParameter<DOF> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f)?;

        if self.control_interface == ControlInterface::Velocity {
            writeln!(f, "inp.control_interface = ControlInterface.Velocity")?;
        }
        if self.synchronization == Synchronization::Phase {
            writeln!(f, "inp.synchronization = Synchronization.Phase")?;
        } else if self.synchronization == Synchronization::None {
            writeln!(f, "inp.synchronization = Synchronization.No")?;
        }
        if self.duration_discretization == DurationDiscretization::Discrete {
            writeln!(
                f,
                "inp.duration_discretization = DurationDiscretization.Discrete"
            )?;
        }

        writeln!(
            f,
            "\ninp.current_position = [{}]",
            join::<DOF>(self.current_position.deref(), true)
        )?;
        writeln!(
            f,
            "inp.current_velocity = [{}]",
            join::<DOF>(self.current_velocity.deref(), true)
        )?;
        writeln!(
            f,
            "inp.current_acceleration = [{}]",
            join::<DOF>(self.current_acceleration.deref(), true)
        )?;
        writeln!(
            f,
            "inp.target_position = [{}]",
            join::<DOF>(self.target_position.deref(), true)
        )?;
        writeln!(
            f,
            "inp.target_velocity = [{}]",
            join::<DOF>(self.target_velocity.deref(), true)
        )?;
        writeln!(
            f,
            "inp.target_acceleration = [{}]",
            join::<DOF>(self.target_acceleration.deref(), true)
        )?;
        writeln!(
            f,
            "inp.max_velocity = [{}]",
            join::<DOF>(self.max_velocity.deref(), true)
        )?;
        writeln!(
            f,
            "inp.max_acceleration = [{}]",
            join::<DOF>(self.max_acceleration.deref(), true)
        )?;
        writeln!(
            f,
            "inp.max_jerk = [{}]",
            join::<DOF>(self.max_jerk.deref(), true)
        )?;

        if let Some(min_vel) = &self.min_velocity {
            writeln!(
                f,
                "inp.min_velocity = [{}]",
                join::<DOF>(min_vel.deref(), true)
            )?;
        }
        if let Some(min_acc) = &self.min_acceleration {
            writeln!(
                f,
                "inp.min_acceleration = [{}]",
                join::<DOF>(min_acc.deref(), true)
            )?;
        }
        if let Some(max_pos) = &self.max_position {
            writeln!(
                f,
                "inp.max_position = [{}]",
                join::<DOF>(max_pos.deref(), true)
            )?;
        }
        if let Some(min_pos) = &self.min_position {
            writeln!(
                f,
                "inp.min_position = [{}]",
                join::<DOF>(min_pos.deref(), true)
            )?;
        }

        fn write_sections<const DOF: usize>(
            f: &mut fmt::Formatter,
            name: &str,
            sections: &[DataArrayOrVec<f64, DOF>],
        ) -> fmt::Result {
            write!(f, "inp.{} = [", name)?;
            for (i, section) in sections.iter().enumerate() {
                let separator = if i > 0 { ", " } else { "" };
                write!(f, "{}[{}]", separator, join::<DOF>(section.deref(), true))?;
            }
            writeln!(f, "]")
        }

        if !self.intermediate_positions.is_empty() {
            write_sections(f, "intermediate_positions", &self.intermediate_positions)?;
        }
        if let Some(v) = &self.per_section_max_velocity {
            write_sections(f, "per_section_max_velocity", v)?;
        }
        if let Some(v) = &self.per_section_max_acceleration {
            write_sections(f, "per_section_max_acceleration", v)?;
        }
        if let Some(v) = &self.per_section_max_jerk {
            write_sections(f, "per_section_max_jerk", v)?;
        }
        if let Some(v) = &self.per_section_min_velocity {
            write_sections(f, "per_section_min_velocity", v)?;
        }
        if let Some(v) = &self.per_section_min_acceleration {
            write_sections(f, "per_section_min_acceleration", v)?;
        }
        if let Some(v) = &self.per_section_max_position {
            write_sections(f, "per_section_max_position", v)?;
        }
        if let Some(v) = &self.per_section_min_position {
            write_sections(f, "per_section_min_position", v)?;
        }
        if let Some(durations) = &self.per_section_minimum_duration {
            writeln!(
                f,
                "inp.per_section_minimum_duration = [{}]",
                join::<0>(durations, false)
            )?;
        }

        Ok(())
    }
}
