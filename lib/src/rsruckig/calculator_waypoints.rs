//! Calculation of multi-section trajectories through intermediate waypoints
//!
//! Upstream ruckig calculates intermediate waypoints only in the Pro version or
//! via its cloud API. This module implements a local solver instead:
//!
//! 1. A pass-through kinematic state at every waypoint is chosen heuristically
//!    (zero velocity on direction reversals, otherwise a velocity in the
//!    direction of travel bounded by the reachable/stoppable velocity over the
//!    adjacent segments; zero acceleration).
//! 2. The sections between the waypoints are chained as state-to-state
//!    calculations of [`TargetCalculator`].
//! 3. A bounded refinement pass (coordinate descent over the waypoint
//!    velocities and accelerations, see [`refinement_sweeps`]) locally
//!    minimizes the total duration. Every candidate state is validated by a
//!    full solver call, so feasibility and limit compliance are preserved by
//!    construction.
//!
//! The resulting trajectory is feasible and smooth (position, velocity and
//! acceleration are continuous across the section boundaries), but — in
//! contrast to a globally optimal joint optimization over all sections — not
//! guaranteed time-optimal (that problem is NP-hard).
//!
//! Sections between waypoints are always time-synchronized (or
//! phase-synchronized when requested), so all degrees of freedom reach each
//! waypoint simultaneously.
//!
//! [`refinement_sweeps`]: WaypointsCalculator::refinement_sweeps

use crate::alloc::format;
use crate::alloc::vec::Vec;
use crate::brake::velocity_cap_third_order;
use crate::calculator_target::TargetCalculator;
use crate::error::{IgnoreErrorHandler, RuckigError, RuckigErrorHandler};
use crate::input_parameter::{ControlInterface, InputParameter, Synchronization};
use crate::result::RuckigResult;
use crate::trajectory::Trajectory;
use crate::util::DataArrayOrVec;

#[allow(unused_imports)]
use num_traits::Float; // no_std: f64::sqrt / cbrt via libm

/// Safety factor applied to the initial heuristic pass-through velocity; the
/// refinement pass may push the velocity back up to the exact bound
const KAPPA: f64 = 0.9;

/// Default number of refinement sweeps over all waypoints
pub const DEFAULT_REFINEMENT_SWEEPS: usize = 4;

/// Multiplicative velocity probe ladder for the refinement pass
const VELOCITY_SCALES: [f64; 5] = [0.5, 0.8, 0.95, 1.1, 1.25];

/// Acceleration probes as fractions of the curvature-based estimate
const ACCELERATION_SCALES: [f64; 3] = [0.0, 0.5, 1.0];

/// Additive pattern-search probes per DoF: ±velocity and ±acceleration steps.
/// These can move a state away from zero (e.g. pass a direction reversal with
/// an overshoot-and-return), which multiplicative scaling cannot.
const N_ADDITIVE_PROBES: usize = 4;

/// Additive step factor per sweep (fraction of the limit range), shrinking for
/// convergence. Sweeps beyond the last entry keep the smallest step.
const ADDITIVE_STEPS: [f64; 4] = [0.2, 0.08, 0.03, 0.01];

/// Effective per-section limit: the per-section value if set, else the global one
#[inline]
fn eff_limit<const DOF: usize>(
    per_section: &Option<Vec<DataArrayOrVec<f64, DOF>>>,
    global: f64,
    section: usize,
    dof: usize,
) -> f64 {
    per_section.as_ref().map_or(global, |v| v[section][dof])
}

/// Effective per-section minimum limit: per-section value, else global minimum,
/// else the negative of the effective maximum of the same section
#[inline]
fn eff_min_limit<const DOF: usize>(
    per_section: &Option<Vec<DataArrayOrVec<f64, DOF>>>,
    global: &Option<DataArrayOrVec<f64, DOF>>,
    eff_max: f64,
    section: usize,
    dof: usize,
) -> f64 {
    if let Some(v) = per_section {
        v[section][dof]
    } else if let Some(g) = global {
        g[dof]
    } else {
        -eff_max
    }
}

/// Effective per-section position limit: per-section value, else the global
/// one, else unbounded
#[inline]
fn eff_min_max_position<const DOF: usize>(
    per_section: &Option<Vec<DataArrayOrVec<f64, DOF>>>,
    global: &Option<DataArrayOrVec<f64, DOF>>,
    unbounded: f64,
    section: usize,
    dof: usize,
) -> f64 {
    if let Some(v) = per_section {
        v[section][dof]
    } else if let Some(g) = global {
        g[dof]
    } else {
        unbounded
    }
}

/// Fill an optional per-DoF array in place (allocation-free once created)
#[inline]
fn fill_limit_opt<const DOF: usize>(
    dst: &mut Option<DataArrayOrVec<f64, DOF>>,
    dofs: Option<usize>,
    n_dofs: usize,
    mut value: impl FnMut(usize) -> f64,
) {
    let arr = dst.get_or_insert_with(|| DataArrayOrVec::new(dofs, 0.0));
    for dof in 0..n_dofs {
        arr[dof] = value(dof);
    }
}

/// Combined (tighter) kinematic limits of the two sections adjacent to a waypoint
struct AdjacentLimits {
    v_max: f64,
    v_min: f64,
    a_max: f64,
    a_min: f64,
    j_max: f64,
    /// Direction of monotone travel through the waypoint (0.0 on reversal/dwell)
    sign: f64,
    /// Velocity magnitude cap from the position limits (infinite when unset)
    position_cap: f64,
}

/// Calculator for trajectories through intermediate waypoints
///
/// Chains state-to-state sections calculated by [`TargetCalculator`], using
/// heuristic pass-through states at the waypoints followed by a bounded local
/// refinement. Used by [`Ruckig`](crate::ruckig::Ruckig) whenever
/// [`InputParameter::intermediate_positions`] is non-empty.
#[derive(Debug, Clone)]
pub struct WaypointsCalculator<const DOF: usize> {
    calculator: TargetCalculator<DOF>,
    /// Scratch input for a single section
    section_input: InputParameter<DOF>,
    /// Scratch trajectory, always a single section
    section_trajectory: Trajectory<DOF>,
    /// Pass-through velocities, one entry per waypoint
    waypoint_velocities: Vec<DataArrayOrVec<f64, DOF>>,
    /// Pass-through accelerations, one entry per waypoint
    waypoint_accelerations: Vec<DataArrayOrVec<f64, DOF>>,
    /// Duration of each section for the current waypoint states
    section_durations: Vec<f64>,
    /// Per-DoF time-optimal durations of each section (for `independent_min_durations`)
    section_min_durations: Vec<DataArrayOrVec<f64, DOF>>,
    /// Backup of a waypoint state while probing candidates
    backup_velocity: DataArrayOrVec<f64, DOF>,
    backup_acceleration: DataArrayOrVec<f64, DOF>,
    /// Number of coordinate-descent sweeps over all waypoints that locally
    /// minimize the total trajectory duration after the initial heuristic
    /// chain has been built. Each sweep probes a small ladder of alternative
    /// pass-through velocities and accelerations per waypoint, each validated
    /// by a full state-to-state calculation. `0` disables refinement (fastest
    /// calculation, most conservative trajectory).
    pub refinement_sweeps: usize,
    pub degrees_of_freedom: usize,
}

impl<const DOF: usize> WaypointsCalculator<DOF> {
    pub fn new(dofs: Option<usize>) -> Self {
        Self {
            calculator: TargetCalculator::new(dofs),
            section_input: InputParameter::new(dofs),
            section_trajectory: Trajectory::new(dofs),
            waypoint_velocities: Vec::new(),
            waypoint_accelerations: Vec::new(),
            section_durations: Vec::new(),
            section_min_durations: Vec::new(),
            backup_velocity: DataArrayOrVec::new(dofs, 0.0),
            backup_acceleration: DataArrayOrVec::new(dofs, 0.0),
            refinement_sweeps: DEFAULT_REFINEMENT_SWEEPS,
            degrees_of_freedom: dofs.unwrap_or(DOF),
        }
    }

    /// Like [`new`](WaypointsCalculator::new), but pre-allocates storage for up
    /// to `max_number_of_waypoints` intermediate positions
    pub fn with_waypoints(dofs: Option<usize>, max_number_of_waypoints: usize) -> Self {
        let mut calculator = Self::new(dofs);
        calculator.waypoint_velocities.reserve(max_number_of_waypoints);
        calculator
            .waypoint_accelerations
            .reserve(max_number_of_waypoints);
        calculator.section_durations.reserve(max_number_of_waypoints + 1);
        calculator
            .section_min_durations
            .reserve(max_number_of_waypoints + 1);
        calculator
    }

    #[inline]
    fn dofs_opt(&self) -> Option<usize> {
        if DOF == 0 {
            Some(self.degrees_of_freedom)
        } else {
            None
        }
    }

    /// Position of chain point `k` (0 = current, 1..=n = waypoints, n+1 = target)
    #[inline]
    fn chain_position<'a>(
        inp: &'a InputParameter<DOF>,
        k: usize,
    ) -> &'a DataArrayOrVec<f64, DOF> {
        let n = inp.intermediate_positions.len();
        if k == 0 {
            &inp.current_position
        } else if k <= n {
            &inp.intermediate_positions[k - 1]
        } else {
            &inp.target_position
        }
    }

    /// Combined limits, direction and position-limit velocity cap for waypoint `k`
    fn adjacent_limits(&self, inp: &InputParameter<DOF>, k: usize, dof: usize) -> AdjacentLimits {
        let (s_prev, s_next) = (k - 1, k);

        let vmax_prev = eff_limit(&inp.per_section_max_velocity, inp.max_velocity[dof], s_prev, dof);
        let vmax_next = eff_limit(&inp.per_section_max_velocity, inp.max_velocity[dof], s_next, dof);
        let vmin_prev = eff_min_limit(&inp.per_section_min_velocity, &inp.min_velocity, vmax_prev, s_prev, dof);
        let vmin_next = eff_min_limit(&inp.per_section_min_velocity, &inp.min_velocity, vmax_next, s_next, dof);
        let amax_prev = eff_limit(&inp.per_section_max_acceleration, inp.max_acceleration[dof], s_prev, dof);
        let amax_next = eff_limit(&inp.per_section_max_acceleration, inp.max_acceleration[dof], s_next, dof);
        let amin_prev = eff_min_limit(&inp.per_section_min_acceleration, &inp.min_acceleration, amax_prev, s_prev, dof);
        let amin_next = eff_min_limit(&inp.per_section_min_acceleration, &inp.min_acceleration, amax_next, s_next, dof);
        let jmax_prev = eff_limit(&inp.per_section_max_jerk, inp.max_jerk[dof], s_prev, dof);
        let jmax_next = eff_limit(&inp.per_section_max_jerk, inp.max_jerk[dof], s_next, dof);

        let d_prev = Self::chain_position(inp, k)[dof] - Self::chain_position(inp, k - 1)[dof];
        let d_next = Self::chain_position(inp, k + 1)[dof] - Self::chain_position(inp, k)[dof];
        let sign = if d_prev * d_next > 0.0 { d_next.signum() } else { 0.0 };

        let j_max = jmax_prev.min(jmax_next);
        let position_limits_set = inp.max_position.is_some()
            || inp.min_position.is_some()
            || inp.per_section_max_position.is_some()
            || inp.per_section_min_position.is_some();
        let position_cap = if position_limits_set && j_max > 0.0 && j_max.is_finite() {
            // The stopping distance from the waypoint state must fit inside
            // the position limits of both adjacent sections
            let w = Self::chain_position(inp, k)[dof];
            let max_pos = eff_min_max_position(&inp.per_section_max_position, &inp.max_position, f64::INFINITY, s_prev, dof)
                .min(eff_min_max_position(&inp.per_section_max_position, &inp.max_position, f64::INFINITY, s_next, dof));
            let min_pos = eff_min_max_position(&inp.per_section_min_position, &inp.min_position, f64::NEG_INFINITY, s_prev, dof)
                .max(eff_min_max_position(&inp.per_section_min_position, &inp.min_position, f64::NEG_INFINITY, s_next, dof));
            // Use the weaker deceleration capability of both sections
            let a_min_eff = amin_prev.max(amin_next);
            let a_max_eff = amax_prev.min(amax_next);
            velocity_cap_third_order(max_pos - w, a_min_eff, j_max)
                .min(velocity_cap_third_order(w - min_pos, a_max_eff, j_max))
        } else {
            f64::INFINITY
        };

        AdjacentLimits {
            v_max: vmax_prev.min(vmax_next),
            v_min: vmin_prev.max(vmin_next),
            a_max: amax_prev.min(amax_next),
            a_min: amin_prev.max(amin_next),
            j_max,
            sign,
            position_cap,
        }
    }

    /// Estimate the initial pass-through velocity at every waypoint
    /// (acceleration starts at zero; the refinement pass may change both)
    fn estimate_waypoint_states(&mut self, inp: &InputParameter<DOF>) {
        let n = inp.intermediate_positions.len();
        let dofs_opt = self.dofs_opt();
        self.waypoint_velocities
            .resize_with(n, || DataArrayOrVec::new(dofs_opt, 0.0));
        self.waypoint_accelerations
            .resize_with(n, || DataArrayOrVec::new(dofs_opt, 0.0));

        for k in 1..=n {
            for dof in 0..self.degrees_of_freedom {
                self.waypoint_accelerations[k - 1][dof] = 0.0;
                if !inp.enabled[dof] {
                    self.waypoint_velocities[k - 1][dof] = 0.0;
                    continue;
                }
                let limits = self.adjacent_limits(inp, k, dof);
                if limits.sign == 0.0 {
                    // Direction reversal (or dwell): the waypoint is a position
                    // extremum for this DoF, pass with zero velocity
                    self.waypoint_velocities[k - 1][dof] = 0.0;
                    continue;
                }

                let d_prev = Self::chain_position(inp, k)[dof] - Self::chain_position(inp, k - 1)[dof];
                let d_next = Self::chain_position(inp, k + 1)[dof] - Self::chain_position(inp, k)[dof];
                // Directional velocity limit of both adjacent sections
                let v_lim = if limits.sign > 0.0 { limits.v_max } else { -limits.v_min };
                // Acceleration capability toward the waypoint and deceleration
                // capability after it, in the direction of motion
                let a_acc = if limits.sign > 0.0 { limits.a_max } else { -limits.a_min };
                let a_dec = if limits.sign > 0.0 { -limits.a_min } else { limits.a_max };
                // Acceleration-limited reachability over the adjacent segment
                // lengths: the velocity must be reachable from rest before the
                // waypoint and reducible to rest after it
                let v_reach = (2.0 * a_acc * d_prev.abs()).sqrt();
                let v_stop = (2.0 * a_dec * d_next.abs()).sqrt();
                // Jerk-limited bound: conservative peak velocity of a
                // jerk-bang profile over the adjacent segment lengths
                let v_jerk_prev = (limits.j_max * d_prev * d_prev).cbrt();
                let v_jerk_next = (limits.j_max * d_next * d_next).cbrt();

                let v = KAPPA
                    * v_lim
                        .min(v_reach)
                        .min(v_stop)
                        .min(v_jerk_prev)
                        .min(v_jerk_next)
                        .min(limits.position_cap);
                self.waypoint_velocities[k - 1][dof] = limits.sign * v;
            }
        }
    }

    /// Write the kinematic state of chain point `k` into the scratch input's
    /// current state
    fn set_current_from_chain(&mut self, inp: &InputParameter<DOF>, k: usize) {
        let n = inp.intermediate_positions.len();
        let s = &mut self.section_input;
        if k == 0 {
            s.current_position.copy_from(&inp.current_position);
            s.current_velocity.copy_from(&inp.current_velocity);
            s.current_acceleration.copy_from(&inp.current_acceleration);
        } else {
            debug_assert!(k <= n);
            s.current_position.copy_from(&inp.intermediate_positions[k - 1]);
            s.current_velocity.copy_from(&self.waypoint_velocities[k - 1]);
            s.current_acceleration
                .copy_from(&self.waypoint_accelerations[k - 1]);
        }
    }

    /// Write the kinematic state of chain point `k` into the scratch input's
    /// target state
    fn set_target_from_chain(&mut self, inp: &InputParameter<DOF>, k: usize) {
        let n = inp.intermediate_positions.len();
        let s = &mut self.section_input;
        if k <= n {
            debug_assert!(k >= 1);
            s.target_position.copy_from(&inp.intermediate_positions[k - 1]);
            s.target_velocity.copy_from(&self.waypoint_velocities[k - 1]);
            s.target_acceleration
                .copy_from(&self.waypoint_accelerations[k - 1]);
        } else {
            s.target_position.copy_from(&inp.target_position);
            s.target_velocity.copy_from(&inp.target_velocity);
            s.target_acceleration.copy_from(&inp.target_acceleration);
        }
    }

    /// Configure the scratch input's limits and settings for section `s`
    fn set_section_limits(&mut self, inp: &InputParameter<DOF>, section: usize) {
        let n = inp.intermediate_positions.len();
        let dofs_opt = self.dofs_opt();
        let n_dofs = self.degrees_of_freedom;
        let s = &mut self.section_input;

        s.degrees_of_freedom = inp.degrees_of_freedom;
        s.control_interface = ControlInterface::Position;
        // Intermediate sections must be synchronized so that all DoFs pass the
        // waypoint simultaneously — otherwise the next section's start state
        // would not match the sampled state at the section boundary. The final
        // section is free to use the requested synchronization.
        s.synchronization = if section < n {
            match inp.synchronization {
                Synchronization::Phase => Synchronization::Phase,
                _ => Synchronization::Time,
            }
        } else {
            inp.synchronization
        };
        s.enabled.copy_from(&inp.enabled);
        s.minimum_duration = inp
            .per_section_minimum_duration
            .as_ref()
            .map(|durations| durations[section]);

        for dof in 0..n_dofs {
            s.max_velocity[dof] =
                eff_limit(&inp.per_section_max_velocity, inp.max_velocity[dof], section, dof);
            s.max_acceleration[dof] = eff_limit(
                &inp.per_section_max_acceleration,
                inp.max_acceleration[dof],
                section,
                dof,
            );
            s.max_jerk[dof] = eff_limit(&inp.per_section_max_jerk, inp.max_jerk[dof], section, dof);
        }
        if inp.per_section_min_velocity.is_some() || inp.min_velocity.is_some() {
            fill_limit_opt(&mut s.min_velocity, dofs_opt, n_dofs, |dof| {
                eff_min_limit(
                    &inp.per_section_min_velocity,
                    &inp.min_velocity,
                    eff_limit(&inp.per_section_max_velocity, inp.max_velocity[dof], section, dof),
                    section,
                    dof,
                )
            });
        } else {
            s.min_velocity = None;
        }
        if inp.per_section_min_acceleration.is_some() || inp.min_acceleration.is_some() {
            fill_limit_opt(&mut s.min_acceleration, dofs_opt, n_dofs, |dof| {
                eff_min_limit(
                    &inp.per_section_min_acceleration,
                    &inp.min_acceleration,
                    eff_limit(
                        &inp.per_section_max_acceleration,
                        inp.max_acceleration[dof],
                        section,
                        dof,
                    ),
                    section,
                    dof,
                )
            });
        } else {
            s.min_acceleration = None;
        }
        if inp.per_section_max_position.is_some() || inp.max_position.is_some() {
            fill_limit_opt(&mut s.max_position, dofs_opt, n_dofs, |dof| {
                eff_min_max_position(
                    &inp.per_section_max_position,
                    &inp.max_position,
                    f64::INFINITY,
                    section,
                    dof,
                )
            });
        } else {
            s.max_position = None;
        }
        if inp.per_section_min_position.is_some() || inp.min_position.is_some() {
            fill_limit_opt(&mut s.min_position, dofs_opt, n_dofs, |dof| {
                eff_min_max_position(
                    &inp.per_section_min_position,
                    &inp.min_position,
                    f64::NEG_INFINITY,
                    section,
                    dof,
                )
            });
        } else {
            s.min_position = None;
        }
    }

    /// Solve section `s` for the current waypoint states into the scratch
    /// trajectory (no error handling — returns the raw result code)
    fn solve_section(
        &mut self,
        inp: &InputParameter<DOF>,
        section: usize,
        delta_time: f64,
    ) -> Result<RuckigResult, RuckigError> {
        self.set_section_limits(inp, section);
        self.set_current_from_chain(inp, section);
        self.set_target_from_chain(inp, section + 1);
        self.calculator.calculate::<IgnoreErrorHandler>(
            &self.section_input,
            &mut self.section_trajectory,
            delta_time,
        )
    }

    /// Copy the scratch section trajectory into `traj` at `section` and record
    /// its durations
    fn commit_section(&mut self, section: usize, traj: &mut Trajectory<DOF>) {
        for dof in 0..self.degrees_of_freedom {
            let profile = self.section_trajectory.profiles[0][dof];
            debug_assert!(
                section == 0 || profile.brake.duration == 0.0,
                "unexpected brake pre-trajectory in section {}",
                section
            );
            traj.profiles[section][dof] = profile;
            self.section_min_durations[section][dof] =
                self.section_trajectory.independent_min_durations[dof];
        }
        self.section_durations[section] = self.section_trajectory.duration;
    }

    /// Solve both sections adjacent to waypoint `k` for the current waypoint
    /// states; returns their durations, or None if either section is infeasible
    fn eval_pair(
        &mut self,
        inp: &InputParameter<DOF>,
        k: usize,
        delta_time: f64,
    ) -> Result<Option<(f64, f64)>, RuckigError> {
        let result_prev = self.solve_section(inp, k - 1, delta_time)?;
        if result_prev != RuckigResult::Working {
            return Ok(None);
        }
        let duration_prev = self.section_trajectory.duration;
        let result_next = self.solve_section(inp, k, delta_time)?;
        if result_next != RuckigResult::Working {
            return Ok(None);
        }
        Ok(Some((duration_prev, self.section_trajectory.duration)))
    }

    /// Refine the pass-through state of waypoint `k` by hill climbing: probe a
    /// ladder of alternative velocities and accelerations (whole-vector scale
    /// moves first, then per-DoF moves) and immediately adopt every candidate
    /// that reduces the summed duration of the two adjacent sections. Returns
    /// whether any candidate improved.
    fn refine_waypoint(
        &mut self,
        inp: &InputParameter<DOF>,
        k: usize,
        delta_time: f64,
        additive_step: f64,
    ) -> Result<bool, RuckigError> {
        let n_dofs = self.degrees_of_freedom;
        // Time scale of the curvature-based acceleration estimate
        let pair_duration = (self.section_durations[k - 1] + self.section_durations[k]).max(1e-9);
        let second_difference = |dof: usize| {
            Self::chain_position(inp, k + 1)[dof] - 2.0 * Self::chain_position(inp, k)[dof]
                + Self::chain_position(inp, k - 1)[dof]
        };
        let mut improved_any = false;

        // Whole-vector moves: scale every DoF's velocity together (the bound
        // jump is per-DoF directional). These make large coupled steps that
        // per-DoF moves cannot (time synchronization couples the DoFs).
        let n_vector_candidates = VELOCITY_SCALES.len() + 1;
        for candidate in 0..n_vector_candidates {
            self.backup_velocity.copy_from(&self.waypoint_velocities[k - 1]);
            self.backup_acceleration
                .copy_from(&self.waypoint_accelerations[k - 1]);
            let mut changed = false;
            for dof in 0..n_dofs {
                if !inp.enabled[dof] {
                    continue;
                }
                let limits = self.adjacent_limits(inp, k, dof);
                let (v, a) = candidate_state(
                    candidate,
                    self.backup_velocity[dof],
                    self.backup_acceleration[dof],
                    &limits,
                    second_difference(dof),
                    pair_duration,
                    additive_step,
                );
                changed |= (v - self.backup_velocity[dof]).abs() > 1e-12
                    || (a - self.backup_acceleration[dof]).abs() > 1e-12;
                self.waypoint_velocities[k - 1][dof] = v;
                self.waypoint_accelerations[k - 1][dof] = a;
            }
            if !changed {
                continue;
            }
            let base = self.section_durations[k - 1] + self.section_durations[k];
            match self.eval_pair(inp, k, delta_time)? {
                Some((d_prev, d_next)) if d_prev + d_next < base - 1e-9 => {
                    self.section_durations[k - 1] = d_prev;
                    self.section_durations[k] = d_next;
                    improved_any = true;
                }
                _ => {
                    self.waypoint_velocities[k - 1].copy_from(&self.backup_velocity);
                    self.waypoint_accelerations[k - 1]
                        .copy_from(&self.backup_acceleration);
                }
            }
        }

        // Per-DoF moves: velocities and accelerations of a single DoF at a
        // time (fine-tuning; typically only the duration-limiting DoF matters)
        let n_candidates =
            VELOCITY_SCALES.len() + 1 + ACCELERATION_SCALES.len() + N_ADDITIVE_PROBES;
        for dof in 0..n_dofs {
            if !inp.enabled[dof] {
                continue;
            }
            let limits = self.adjacent_limits(inp, k, dof);
            for candidate in 0..n_candidates {
                let current_v = self.waypoint_velocities[k - 1][dof];
                let current_a = self.waypoint_accelerations[k - 1][dof];
                let (v, a) = candidate_state(
                    candidate,
                    current_v,
                    current_a,
                    &limits,
                    second_difference(dof),
                    pair_duration,
                    additive_step,
                );
                if (v - current_v).abs() <= 1e-12 && (a - current_a).abs() <= 1e-12 {
                    continue;
                }
                self.waypoint_velocities[k - 1][dof] = v;
                self.waypoint_accelerations[k - 1][dof] = a;
                let base = self.section_durations[k - 1] + self.section_durations[k];
                match self.eval_pair(inp, k, delta_time)? {
                    Some((d_prev, d_next)) if d_prev + d_next < base - 1e-9 => {
                        self.section_durations[k - 1] = d_prev;
                        self.section_durations[k] = d_next;
                        improved_any = true;
                    }
                    _ => {
                        self.waypoint_velocities[k - 1][dof] = current_v;
                        self.waypoint_accelerations[k - 1][dof] = current_a;
                    }
                }
            }
        }

        Ok(improved_any)
    }

    /// Calculate a multi-section trajectory through the intermediate waypoints
    pub fn calculate<E: RuckigErrorHandler>(
        &mut self,
        inp: &InputParameter<DOF>,
        traj: &mut Trajectory<DOF>,
        delta_time: f64,
    ) -> Result<RuckigResult, RuckigError> {
        let n = inp.intermediate_positions.len();
        let n_sections = n + 1;
        let dofs_opt = self.dofs_opt();

        self.estimate_waypoint_states(inp);

        traj.resize_sections(n_sections);
        self.section_durations.resize(n_sections, 0.0);
        self.section_min_durations
            .resize_with(n_sections, || DataArrayOrVec::new(dofs_opt, 0.0));

        // Phase 1: initial feasible chain. If a section fails, its waypoint's
        // pass-through velocity is degraded (halved twice, then zero) before
        // giving up. The final section's target state is user-given and is
        // never degraded.
        for section in 0..n_sections {
            let max_attempts = if section < n { 4 } else { 1 };
            let mut result = RuckigResult::Working;
            for attempt in 0..max_attempts {
                if attempt > 0 {
                    let scale = if attempt < 3 { 0.5 } else { 0.0 };
                    for dof in 0..self.degrees_of_freedom {
                        self.waypoint_velocities[section][dof] *= scale;
                    }
                }
                result = self.solve_section(inp, section, delta_time)?;
                if result == RuckigResult::Working {
                    break;
                }
            }
            if result != RuckigResult::Working {
                E::handle_calculator_error(&format!(
                    "error in section {} between waypoints, result {:?}, input: {}",
                    section, result, self.section_input
                ))?;
                return Ok(result);
            }
            self.section_durations[section] = self.section_trajectory.duration;
        }

        // Phase 2: bounded local refinement of the waypoint pass-through states
        // with shrinking additive probe steps (pattern search); stop early only
        // once the smallest step no longer improves
        if n > 0 {
            for sweep in 0..self.refinement_sweeps {
                let additive_step = ADDITIVE_STEPS[sweep.min(ADDITIVE_STEPS.len() - 1)];
                let mut improved = false;
                for k in 1..=n {
                    improved |= self.refine_waypoint(inp, k, delta_time, additive_step)?;
                }
                if !improved && sweep + 1 >= ADDITIVE_STEPS.len() {
                    break;
                }
            }
        }

        // Phase 3: final pass — re-solve every section for the final waypoint
        // states (all verified feasible above; the solver is deterministic)
        // and commit the profiles and bookkeeping
        traj.duration = 0.0;
        for section in 0..n_sections {
            let result = self.solve_section(inp, section, delta_time)?;
            debug_assert_eq!(result, RuckigResult::Working);
            if result != RuckigResult::Working {
                E::handle_calculator_error(&format!(
                    "error in section {} between waypoints, result {:?}, input: {}",
                    section, result, self.section_input
                ))?;
                return Ok(result);
            }
            self.commit_section(section, traj);
            traj.duration += self.section_durations[section];
            traj.cumulative_times[section] = traj.duration;
        }
        for dof in 0..self.degrees_of_freedom {
            traj.independent_min_durations[dof] = 0.0;
            for section in 0..n_sections {
                traj.independent_min_durations[dof] += self.section_min_durations[section][dof];
            }
        }

        Ok(RuckigResult::Working)
    }
}

/// Build one candidate pass-through state (velocity, acceleration) for the
/// refinement ladder, clamped into the combined limits of the adjacent
/// sections so that every candidate is a valid section boundary state.
///
/// Candidates `0..VELOCITY_SCALES.len()` scale the current velocity (keeping
/// the acceleration), the next candidate jumps to the directional velocity
/// bound, and the remaining ones probe curvature-based accelerations (keeping
/// the velocity).
fn candidate_state(
    candidate: usize,
    current_v: f64,
    current_a: f64,
    limits: &AdjacentLimits,
    second_difference: f64,
    pair_duration: f64,
    additive_step: f64,
) -> (f64, f64) {
    // Safety factor keeping guarded states strictly inside the brake
    // conditions of `BrakeProfile::get_position_brake_trajectory` (which use
    // exact comparisons) despite floating-point rounding
    const GUARD: f64 = 0.999;

    let n_scales = VELOCITY_SCALES.len();
    let n_acc = ACCELERATION_SCALES.len();
    let position_cap = GUARD * limits.position_cap;
    let clamp_v = |v: f64| -> f64 {
        v.clamp(limits.v_min, limits.v_max)
            .clamp(-position_cap, position_cap)
    };

    let (v, mut a) = if candidate < n_scales {
        (clamp_v(current_v * VELOCITY_SCALES[candidate]), current_a)
    } else if candidate == n_scales {
        // Jump to the directional velocity bound (0 on reversals)
        let bound = if limits.sign > 0.0 {
            limits.v_max
        } else if limits.sign < 0.0 {
            limits.v_min
        } else {
            0.0
        };
        (clamp_v(bound), current_a)
    } else if candidate < n_scales + 1 + n_acc {
        // Acceleration probe: curvature-based estimate a ~ 4 * d²p / T²
        let scale = ACCELERATION_SCALES[candidate - n_scales - 1];
        let a = scale * 4.0 * second_difference / (pair_duration * pair_duration);
        (current_v, a)
    } else {
        // Additive pattern-search probes: ±step in velocity or acceleration.
        // Unlike multiplicative scaling, these can leave a zero state — e.g.
        // pass a direction-reversal waypoint with an overshoot-and-return.
        let v_step = additive_step * 0.5 * (limits.v_max - limits.v_min);
        let a_step = additive_step * 0.5 * (limits.a_max - limits.a_min);
        match candidate - n_scales - 1 - n_acc {
            0 => (clamp_v(current_v + v_step), current_a),
            1 => (clamp_v(current_v - v_step), current_a),
            2 => (current_v, current_a + a_step),
            _ => (current_v, current_a - a_step),
        }
    };

    // Joint guards on the final (v, a) pair — applied to every candidate so
    // that a velocity probe cannot invalidate an acceleration adopted earlier
    a = a.clamp(limits.a_min, limits.a_max);
    if limits.position_cap.is_finite() {
        // Conservative: near position limits, keep the waypoint acceleration
        // at zero (the velocity cap already guards the state)
        a = 0.0;
    } else if limits.j_max > 0.0 && limits.j_max.is_finite() {
        // The velocity extremum while the acceleration ramps back to zero,
        // v_at_a_zero(v, a, j) = v + a²/(2j), must stay within the limits —
        // otherwise the section solver would emit a brake pre-trajectory
        if a > 0.0 {
            let headroom = (limits.v_max - v).max(0.0);
            a = a.min(GUARD * (2.0 * limits.j_max * headroom).sqrt());
        } else if a < 0.0 {
            let headroom = (v - limits.v_min).max(0.0);
            a = a.max(-GUARD * (2.0 * limits.j_max * headroom).sqrt());
        }
    }
    (v, a)
}
