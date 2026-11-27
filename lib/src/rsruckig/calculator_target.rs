//! Calculation of a state-to-state trajectory.
use crate::brake::{
    stop_distance_second_order, stop_distance_third_order, velocity_cap_second_order,
    velocity_cap_third_order,
};
use crate::error::{RuckigError, RuckigErrorHandler};
use crate::util::DataArrayOrVec;
use crate::{
    block::Block,
    input_parameter::{ControlInterface, DurationDiscretization, InputParameter, Synchronization},
    position_first_step1::PositionFirstOrderStep1,
    position_first_step2::PositionFirstOrderStep2,
    position_second_step1::PositionSecondOrderStep1,
    position_second_step2::PositionSecondOrderStep2,
    position_third_step1::PositionThirdOrderStep1,
    position_third_step2::PositionThirdOrderStep2,
    profile::{Direction, Profile, ReachedLimits},
    result::RuckigResult,
    trajectory::Trajectory,
    velocity_second_step1::VelocitySecondOrderStep1,
    velocity_second_step2::VelocitySecondOrderStep2,
    velocity_third_step1::VelocityThirdOrderStep1,
    velocity_third_step2::VelocityThirdOrderStep2,
};

use crate::alloc::{format, vec, vec::Vec};
use core::fmt;

/// Highest constrained derivative of a DoF, selecting the profile family
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ProfileOrder {
    /// Velocity-limited only (infinite acceleration and jerk)
    First,
    /// Acceleration-limited (infinite jerk)
    Second,
    /// Jerk-limited
    Third,
}

impl ProfileOrder {
    #[inline]
    fn of<const DOF: usize>(inp: &InputParameter<DOF>, dof: usize) -> Self {
        if !inp.max_jerk[dof].is_infinite() {
            Self::Third
        } else if !inp.max_acceleration[dof].is_infinite() {
            Self::Second
        } else {
            Self::First
        }
    }
}

/// Bisection state for shrinking a directional velocity limit until the
/// position limits hold: `lo` is known compliant (or infeasible), `hi` violates
#[derive(Debug, Clone, Copy)]
struct VelocityBisection {
    lo: f64,
    hi: f64,
    active: bool,
}

impl VelocityBisection {
    fn new(lo: f64, hi: f64, active: bool) -> Self {
        Self { lo, hi, active }
    }

    #[inline]
    fn midpoint(&self) -> f64 {
        0.5 * (self.lo + self.hi)
    }

    /// Feed back the outcome for the probed magnitude `v`
    #[inline]
    fn update(&mut self, v: f64, violated: bool) {
        if violated {
            self.active = true;
            self.hi = v;
        } else if self.active {
            self.lo = v;
        }
    }

    /// The probed magnitude `v` was too restrictive for any profile
    #[inline]
    fn enlarge(&mut self, v: f64) {
        if self.active {
            self.lo = v;
        }
    }

    #[inline]
    fn converged(&self) -> bool {
        !self.active || (self.hi - self.lo) <= 1e-9 * self.hi.max(1.0)
    }
}

/// Relative distance of a synchronized duration to an extremal profile of
/// step 1 below which the extremal profile is used if step 2 finds no profile
const STEP2_EXTREMAL_TOLERANCE: f64 = 1e-12;

/// Bound on the trajectory duration: a calculation is abandoned (before the
/// expensive step 2) once `offset + duration >= limit`
#[derive(Debug, Clone, Copy)]
pub(crate) struct DurationBudget {
    pub offset: f64,
    pub limit: f64,
}

impl DurationBudget {
    pub(crate) const UNLIMITED: Self = Self {
        offset: 0.0,
        limit: f64::INFINITY,
    };

    #[inline]
    fn is_exceeded_by(&self, duration: f64) -> bool {
        self.offset + duration >= self.limit
    }
}

/// Calculator for state-to-state trajectories without intermediate waypoints.
///
/// For every DoF it first calculates the time-optimal profile and the blocked
/// duration intervals (step 1), then determines the synchronized duration and
/// recalculates the profiles of the remaining DoFs for it (step 2). Used by
/// [`Ruckig`](crate::ruckig::Ruckig) and [`Trackig`](crate::trackig::Trackig).
#[derive(Debug, Clone)]
pub struct TargetCalculator<const DOF: usize> {
    eps: f64,
    return_error_at_maximal_duration: bool,
    new_phase_control: DataArrayOrVec<f64, DOF>,
    pd: DataArrayOrVec<f64, DOF>,
    possible_t_syncs: Vec<f64>,
    idx: Vec<usize>,
    blocks: DataArrayOrVec<Block, DOF>,
    // Reused across DoFs/retargets so the solver's [Profile; 6] buffer is
    // default-constructed once here instead of on every run_position_step1 call.
    pos_step1: PositionThirdOrderStep1,
    /// Per DoF: the step 1 result in `blocks` (and the effective velocity
    /// limits) already matches the coming input, so step 1 is skipped. Only
    /// used by callers of [`calculate_section`](Self::calculate_section) that
    /// re-solve nearly identical inputs; `calculate` always starts cleared.
    step1_valid: DataArrayOrVec<bool, DOF>,
    inp_min_velocity: DataArrayOrVec<f64, DOF>,
    inp_min_acceleration: DataArrayOrVec<f64, DOF>,
    inp_min_position: DataArrayOrVec<f64, DOF>,
    inp_max_position: DataArrayOrVec<f64, DOF>,
    eff_min_velocity: DataArrayOrVec<f64, DOF>,
    eff_max_velocity: DataArrayOrVec<f64, DOF>,
    position_limits_active: bool,
    inp_per_dof_control_interface: DataArrayOrVec<ControlInterface, DOF>,
    inp_per_dof_synchronization: DataArrayOrVec<Synchronization, DOF>,
    /// Number of degrees of freedom
    pub degrees_of_freedom: usize,
}

impl<const DOF: usize> TargetCalculator<DOF> {
    /// Create a calculator. Pass `None` to use the compile-time `DOF`, or
    /// `Some(n)` with `DOF = 0` for `n` heap-allocated degrees of freedom.
    pub fn new(dofs: Option<usize>) -> Self {
        Self {
            blocks: DataArrayOrVec::new(dofs, Block::default()),
            pos_step1: PositionThirdOrderStep1::default(),
            step1_valid: DataArrayOrVec::new(dofs, false),
            inp_min_velocity: DataArrayOrVec::new(dofs, 0.0),
            inp_min_acceleration: DataArrayOrVec::new(dofs, 0.0),
            inp_min_position: DataArrayOrVec::new(dofs, f64::NEG_INFINITY),
            inp_max_position: DataArrayOrVec::new(dofs, f64::INFINITY),
            eff_min_velocity: DataArrayOrVec::new(dofs, 0.0),
            eff_max_velocity: DataArrayOrVec::new(dofs, 0.0),
            position_limits_active: false,
            inp_per_dof_control_interface: DataArrayOrVec::new(dofs, ControlInterface::default()),
            inp_per_dof_synchronization: DataArrayOrVec::new(dofs, Synchronization::default()),
            new_phase_control: DataArrayOrVec::new(dofs, 0.0),
            pd: DataArrayOrVec::new(dofs, 0.0),
            possible_t_syncs: vec![0.0; 3 * dofs.unwrap_or(DOF) + 1],
            idx: vec![0; 3 * dofs.unwrap_or(DOF) + 1],
            eps: f64::EPSILON,
            return_error_at_maximal_duration: true,
            degrees_of_freedom: dofs.unwrap_or(DOF),
        }
    }

    // Allowing mutable reference to self for the sake of better performance.
    #[allow(clippy::wrong_self_convention)]
    fn is_input_collinear(
        &mut self,
        inp: &InputParameter<DOF>,
        limiting_direction: Direction,
        limiting_dof: usize,
    ) -> bool {
        // Check that vectors pd, v0, a0, vf, af are collinear
        for dof in 0..self.degrees_of_freedom {
            self.pd[dof] = inp.target_position[dof] - inp.current_position[dof];
        }

        let mut scale_vector: Option<&DataArrayOrVec<f64, DOF>> = None;
        let mut scale_dof: Option<usize> = None;
        for dof in 0..self.degrees_of_freedom {
            if self.inp_per_dof_synchronization[dof] != Synchronization::Phase {
                continue;
            }

            if self.inp_per_dof_control_interface[dof] == ControlInterface::Position
                && self.pd[dof].abs() > self.eps
            {
                scale_vector = Some(&self.pd);
                scale_dof = Some(dof);
                break;
            } else if inp.current_velocity[dof].abs() > self.eps {
                scale_vector = Some(&inp.current_velocity);
                scale_dof = Some(dof);
                break;
            } else if inp.current_acceleration[dof].abs() > self.eps {
                scale_vector = Some(&inp.current_acceleration);
                scale_dof = Some(dof);
                break;
            } else if inp.target_velocity[dof].abs() > self.eps {
                scale_vector = Some(&inp.target_velocity);
                scale_dof = Some(dof);
                break;
            } else if inp.target_acceleration[dof].abs() > self.eps {
                scale_vector = Some(&inp.target_acceleration);
                scale_dof = Some(dof);
                break;
            }
        }

        if scale_dof.is_none() {
            return false; // Zero everywhere is in theory collinear, but that trivial case is better handled elsewhere
        }

        let scale = scale_vector.unwrap()[scale_dof.unwrap()];
        let pd_scale = self.pd[scale_dof.unwrap()] / scale;
        let v0_scale = inp.current_velocity[scale_dof.unwrap()] / scale;
        let vf_scale = inp.target_velocity[scale_dof.unwrap()] / scale;
        let a0_scale = inp.current_acceleration[scale_dof.unwrap()] / scale;
        let af_scale = inp.target_acceleration[scale_dof.unwrap()] / scale;

        let scale_limiting = scale_vector.unwrap()[limiting_dof];
        let mut control_limiting = if limiting_direction == Direction::UP {
            inp.max_jerk[limiting_dof]
        } else {
            -inp.max_jerk[limiting_dof]
        };
        if inp.max_jerk[limiting_dof].is_infinite() {
            control_limiting = if limiting_direction == Direction::UP {
                inp.max_acceleration[limiting_dof]
            } else {
                self.inp_min_acceleration[limiting_dof]
            };
        }

        for dof in 0..self.degrees_of_freedom {
            if self.inp_per_dof_synchronization[dof] != Synchronization::Phase {
                continue;
            }

            let current_scale = scale_vector.unwrap()[dof];
            if (self.inp_per_dof_control_interface[dof] == ControlInterface::Position
                && (self.pd[dof] - pd_scale * current_scale).abs() > self.eps)
                || (inp.current_velocity[dof] - v0_scale * current_scale).abs() > self.eps
                || (inp.current_acceleration[dof] - a0_scale * current_scale).abs() > self.eps
                || (inp.target_velocity[dof] - vf_scale * current_scale).abs() > self.eps
                || (inp.target_acceleration[dof] - af_scale * current_scale).abs() > self.eps
            {
                return false;
            }

            self.new_phase_control[dof] = control_limiting * current_scale / scale_limiting;
        }

        true
    }

    fn synchronize(
        &mut self,
        t_min: Option<f64>,
        t_sync: &mut f64,
        limiting_dof: &mut Option<usize>,
        profiles: &mut DataArrayOrVec<Profile, { DOF }>,
        discrete_duration: bool,
        delta_time: f64,
    ) -> bool {
        // Check for (degrees_of_freedom == 1 && !t_min && !discrete_duration) is now outside

        // Possible t_syncs are the start times of the intervals and optional t_min
        let mut any_interval = false;
        for dof in 0..self.degrees_of_freedom {
            // Ignore DoFs without synchronization here
            if self.inp_per_dof_synchronization[dof] == Synchronization::None {
                self.possible_t_syncs[dof] = 0.0;
                self.possible_t_syncs[self.degrees_of_freedom + dof] = f64::INFINITY;
                self.possible_t_syncs[2 * self.degrees_of_freedom + dof] = f64::INFINITY;
                continue;
            }

            self.possible_t_syncs[dof] = self.blocks[dof].t_min;
            self.possible_t_syncs[self.degrees_of_freedom + dof] = self.blocks[dof]
                .a
                .as_ref()
                .map_or(f64::INFINITY, |a| a.right);
            self.possible_t_syncs[2 * self.degrees_of_freedom + dof] = self.blocks[dof]
                .b
                .as_ref()
                .map_or(f64::INFINITY, |b| b.right);
            any_interval |= self.blocks[dof].a.is_some() || self.blocks[dof].b.is_some();
        }
        self.possible_t_syncs[3 * self.degrees_of_freedom] = t_min.unwrap_or(f64::INFINITY);
        any_interval |= t_min.is_some();

        if discrete_duration {
            for possible_t_sync in &mut self.possible_t_syncs {
                if possible_t_sync.is_infinite() {
                    continue;
                }

                let remainder = *possible_t_sync % delta_time; // in [0, delta_time)
                if remainder > self.eps {
                    *possible_t_sync += delta_time - remainder;
                }
            }
        }

        // Test them in sorted order
        // Setting up the range for `idx_end`
        let idx_end = if any_interval {
            self.idx.len()
        } else {
            self.degrees_of_freedom
        };

        // Initialize the range similar to `std::iota`
        for i in 0..idx_end {
            self.idx[i] = i;
        }

        // Sort the values in the range
        let possible_t_syncs = &self.possible_t_syncs;
        self.idx[..idx_end].sort_by(|&i, &j| possible_t_syncs[i].total_cmp(&possible_t_syncs[j]));

        // Start at last tmin (or worse)
        for &i in &self.idx[self.degrees_of_freedom.saturating_sub(1)..idx_end] {
            let possible_t_sync = self.possible_t_syncs[i];
            let is_blocked = (0..self.degrees_of_freedom).any(|dof| {
                self.inp_per_dof_synchronization[dof] != Synchronization::None
                    && self.blocks[dof].is_blocked(possible_t_sync)
            });
            if is_blocked || possible_t_sync < t_min.unwrap_or(0.0) || possible_t_sync.is_infinite()
            {
                continue;
            }

            *t_sync = possible_t_sync;
            if i == 3 * self.degrees_of_freedom {
                // Optional t_min
                *limiting_dof = None;
                return true;
            }

            let div = i / self.degrees_of_freedom;
            let dof = i % self.degrees_of_freedom;
            *limiting_dof = Some(dof);
            // `Profile`/`Interval` are `Copy`, so read the single needed field directly
            // instead of cloning the whole (two-profile) `Interval`.
            let block = &self.blocks[dof];
            let interval = match div {
                1 => block.a.as_ref(),
                2 => block.b.as_ref(),
                _ => None,
            };
            profiles[dof] = interval.map_or(block.p_min, |interval| interval.profile);
            return true;
        }

        false
    }

    /// Scaled tolerance for position-limit checks. Accounts for the target
    /// precision (1e-8) of the profiles and the +EPS paddings of brake phases.
    #[inline]
    fn position_limit_tolerance(limit: f64) -> f64 {
        1e-8 * limit.abs().max(1.0)
    }

    /// Stopping distances (upwards, downwards) from the given state under the
    /// kinematic limits of `dof`. Zero for first-order (monotone) profiles.
    fn stop_distances(&self, dof: usize, inp: &InputParameter<DOF>, v: f64, a: f64) -> (f64, f64) {
        let j_max = inp.max_jerk[dof];
        let a_max = inp.max_acceleration[dof];
        let a_min = self.inp_min_acceleration[dof];
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
            (0.0, 0.0)
        }
    }

    /// Start position of a profile, including a possible brake pre-trajectory
    #[inline]
    fn profile_start_position(profile: &Profile) -> f64 {
        if profile.brake.duration > 0.0 {
            profile.brake.p[0]
        } else {
            profile.p[0]
        }
    }

    /// Check a single profile against the position limits of `dof`, over the
    /// whole trajectory `duration`: a profile that ends earlier (a DoF without
    /// synchronization) keeps moving with its final velocity until then.
    /// Returns (upper_limit_violated, lower_limit_violated). Besides the extrema
    /// of the profile itself, the end state must leave enough margin for a full
    /// stop inside the limits, so that the system can always stay in the workspace.
    fn profile_position_violations(
        &self,
        dof: usize,
        inp: &InputParameter<DOF>,
        profile: &Profile,
        duration: f64,
    ) -> (bool, bool) {
        // If the current position already lies outside the limits, the
        // trajectory back inside is still allowed - it must only never move
        // further outside than its start
        let p_start = Self::profile_start_position(profile);
        let max_pos = self.inp_max_position[dof].max(p_start);
        let min_pos = self.inp_min_position[dof].min(p_start);
        let tol_up = Self::position_limit_tolerance(max_pos);
        let tol_down = Self::position_limit_tolerance(min_pos);

        let ext = profile.get_position_extrema_until(duration);
        let mut up = ext.max > max_pos + tol_up;
        let mut down = ext.min < min_pos - tol_down;

        let (p_end, v_end) = profile.state_after(duration);
        let (stop_up, stop_down) = self.stop_distances(dof, inp, v_end, profile.a[7]);
        up |= p_end + stop_up > max_pos + tol_up;
        down |= p_end - stop_down < min_pos - tol_down;

        (up, down)
    }

    /// Check all candidate profiles of the step-1 block of `dof` against the
    /// position limits. Returns (upper_violated, lower_violated) over all of them,
    /// as the synchronization may select any of the block profiles later on.
    fn block_position_violations(&self, dof: usize) -> (bool, bool) {
        // All block profiles share the same start state (see
        // profile_position_violations for the rationale of relaxing by it)
        let p_start = Self::profile_start_position(&self.blocks[dof].p_min);
        let max_pos = self.inp_max_position[dof].max(p_start);
        let min_pos = self.inp_min_position[dof].min(p_start);
        let tol_up = Self::position_limit_tolerance(max_pos);
        let tol_down = Self::position_limit_tolerance(min_pos);

        let mut up = false;
        let mut down = false;
        let mut check = |profile: &Profile| {
            let ext = profile.get_position_extrema();
            up |= ext.max > max_pos + tol_up;
            down |= ext.min < min_pos - tol_down;
        };

        check(&self.blocks[dof].p_min);
        if let Some(a) = &self.blocks[dof].a {
            check(&a.profile);
        }
        if let Some(b) = &self.blocks[dof].b {
            check(&b.profile);
        }
        (up, down)
    }

    /// Report a calculation error to the error handler `T`; the message is
    /// only formatted when somebody listens
    #[inline]
    fn report<T: RuckigErrorHandler>(&self, message: fmt::Arguments) -> Result<(), RuckigError> {
        if T::IGNORES_ERRORS {
            return Ok(());
        }
        T::handle_calculator_error(&format!("{}", message))
    }

    /// Whether a limit of `dof` is zero, which makes some targets unreachable
    #[inline]
    fn has_zero_limits(&self, inp: &InputParameter<DOF>, dof: usize) -> bool {
        inp.max_acceleration[dof] == 0.0
            || self.inp_min_acceleration[dof] == 0.0
            || inp.max_jerk[dof] == 0.0
    }

    /// Brake pre-trajectory (if the current state exceeds or will exceed the
    /// limits) and boundary conditions of the profile of `dof`
    #[inline(always)]
    fn set_brake_and_boundary(&self, dof: usize, inp: &InputParameter<DOF>, p: &mut Profile) {
        let order = ProfileOrder::of(inp, dof);
        let (p0, v0, a0) = (
            inp.current_position[dof],
            inp.current_velocity[dof],
            inp.current_acceleration[dof],
        );
        let (v_max, v_min) = (inp.max_velocity[dof], self.inp_min_velocity[dof]);
        let (a_max, a_min) = (inp.max_acceleration[dof], self.inp_min_acceleration[dof]);
        let j_max = inp.max_jerk[dof];
        let (min_position, max_position) = (self.inp_min_position[dof], self.inp_max_position[dof]);

        match self.inp_per_dof_control_interface[dof] {
            ControlInterface::Position => {
                match (order, self.position_limits_active) {
                    (ProfileOrder::Third, true) => {
                        p.brake.get_position_brake_trajectory_with_position_limits(
                            p0,
                            v0,
                            a0,
                            v_max,
                            v_min,
                            a_max,
                            a_min,
                            j_max,
                            min_position,
                            max_position,
                        )
                    }
                    (ProfileOrder::Third, false) => p
                        .brake
                        .get_position_brake_trajectory(v0, a0, v_max, v_min, a_max, a_min, j_max),
                    (ProfileOrder::Second, true) => p
                        .brake
                        .get_second_order_position_brake_trajectory_with_position_limits(
                            p0,
                            v0,
                            v_max,
                            v_min,
                            a_max,
                            a_min,
                            min_position,
                            max_position,
                        ),
                    (ProfileOrder::Second, false) => p
                        .brake
                        .get_second_order_position_brake_trajectory(v0, v_max, v_min, a_max, a_min),
                    (ProfileOrder::First, _) => {}
                }
                p.set_boundary(
                    p0,
                    v0,
                    a0,
                    inp.target_position[dof],
                    inp.target_velocity[dof],
                    inp.target_acceleration[dof],
                );
            }
            ControlInterface::Velocity => {
                match order {
                    ProfileOrder::Third => p
                        .brake
                        .get_velocity_brake_trajectory(a0, a_max, a_min, j_max),
                    _ => p.brake.get_second_order_velocity_brake_trajectory(),
                }

                // With position limits, cap the effective target velocity so
                // that a full stop always fits inside the remaining distance
                // to the limits. The user's input is not modified.
                let mut target_velocity = inp.target_velocity[dof];
                if self.position_limits_active {
                    let (room_up, room_down) = (max_position - p0, p0 - min_position);
                    let (cap_up, cap_down) = match order {
                        ProfileOrder::Third => (
                            velocity_cap_third_order(room_up, a_min, j_max),
                            velocity_cap_third_order(room_down, a_max, j_max),
                        ),
                        ProfileOrder::Second => (
                            velocity_cap_second_order(room_up, a_min),
                            velocity_cap_second_order(room_down, a_max),
                        ),
                        ProfileOrder::First => (f64::INFINITY, f64::INFINITY),
                    };
                    target_velocity = target_velocity.min(cap_up).max(-cap_down);
                }

                p.set_boundary_for_velocity(
                    p0,
                    v0,
                    a0,
                    target_velocity,
                    inp.target_acceleration[dof],
                );
            }
            ControlInterface::Acceleration => {}
        }

        // Finalize pre & post-trajectories
        match order {
            ProfileOrder::Third => p.brake.finalize(&mut p.p[0], &mut p.v[0], &mut p.a[0]),
            ProfileOrder::Second => {
                p.brake
                    .finalize_second_order(&mut p.p[0], &mut p.v[0], &mut p.a[0])
            }
            // First-order profiles never brake; a reused profile must not
            // keep the brake of an earlier calculation
            ProfileOrder::First => p.brake.duration = 0.0,
        }
    }

    /// Step 1: extremal profiles and blocked duration intervals of `dof`, with
    /// the current effective velocity limits
    #[inline(always)]
    fn run_step1(&mut self, dof: usize, inp: &InputParameter<DOF>, p: &mut Profile) -> bool {
        let (v_max, v_min) = (self.eff_max_velocity[dof], self.eff_min_velocity[dof]);
        let (a_max, a_min) = (inp.max_acceleration[dof], self.inp_min_acceleration[dof]);
        let j_max = inp.max_jerk[dof];
        let block = &mut self.blocks[dof];

        match (
            self.inp_per_dof_control_interface[dof],
            ProfileOrder::of(inp, dof),
        ) {
            (ControlInterface::Position, ProfileOrder::Third) => {
                self.pos_step1.reset(
                    p.p[0], p.v[0], p.a[0], p.pf, p.vf, p.af, v_max, v_min, a_max, a_min, j_max,
                );
                self.pos_step1.get_profile(p, block)
            }
            (ControlInterface::Position, ProfileOrder::Second) => PositionSecondOrderStep1::new(
                p.p[0], p.v[0], p.pf, p.vf, v_max, v_min, a_max, a_min,
            )
            .get_profile(p, block),
            (ControlInterface::Position, ProfileOrder::First) => {
                PositionFirstOrderStep1::new(p.p[0], p.pf, v_max, v_min).get_profile(p, block)
            }
            (ControlInterface::Velocity, ProfileOrder::Third) => {
                VelocityThirdOrderStep1::new(p.v[0], p.a[0], p.vf, p.af, a_max, a_min, j_max)
                    .get_profile(p, block)
            }
            (ControlInterface::Velocity, _) => {
                VelocitySecondOrderStep1::new(p.v[0], p.vf, a_max, a_min).get_profile(p, block)
            }
            (ControlInterface::Acceleration, _) => false,
        }
    }

    /// Step 2: profile of `dof` with the fixed duration `t_profile`, within
    /// the velocity limits `v_max` / `v_min` (position interface)
    fn run_step2(
        &self,
        dof: usize,
        inp: &InputParameter<DOF>,
        t_profile: f64,
        v_max: f64,
        v_min: f64,
        p: &mut Profile,
    ) -> bool {
        let (a_max, a_min) = (inp.max_acceleration[dof], self.inp_min_acceleration[dof]);
        let j_max = inp.max_jerk[dof];

        match (
            self.inp_per_dof_control_interface[dof],
            ProfileOrder::of(inp, dof),
        ) {
            (ControlInterface::Position, ProfileOrder::Third) => PositionThirdOrderStep2::new(
                t_profile, p.p[0], p.v[0], p.a[0], p.pf, p.vf, p.af, v_max, v_min, a_max, a_min,
                j_max,
            )
            .get_profile(p),
            (ControlInterface::Position, ProfileOrder::Second) => PositionSecondOrderStep2::new(
                t_profile, p.p[0], p.v[0], p.pf, p.vf, v_max, v_min, a_max, a_min,
            )
            .get_profile(p),
            (ControlInterface::Position, ProfileOrder::First) => {
                PositionFirstOrderStep2::new(t_profile, p.p[0], p.pf, v_max, v_min).get_profile(p)
            }
            (ControlInterface::Velocity, ProfileOrder::Third) => VelocityThirdOrderStep2::new(
                t_profile, p.v[0], p.a[0], p.vf, p.af, a_max, a_min, j_max,
            )
            .get_profile(p),
            (ControlInterface::Velocity, _) => {
                VelocitySecondOrderStep2::new(t_profile, p.v[0], p.vf, a_max, a_min).get_profile(p)
            }
            (ControlInterface::Acceleration, _) => false,
        }
    }

    /// Phase synchronization: check the profile of `dof` that reuses the
    /// timing of the limiting DoF with the scaled control `new_phase_control`
    fn check_phase_profile(
        &self,
        dof: usize,
        inp: &InputParameter<DOF>,
        t_profile: f64,
        p: &mut Profile,
    ) -> bool {
        let signs = p.control_signs;
        let limits = ReachedLimits::None;
        let control = self.new_phase_control[dof];
        let (v_max, v_min) = (self.eff_max_velocity[dof], self.eff_min_velocity[dof]);
        let (a_max, a_min) = (inp.max_acceleration[dof], self.inp_min_acceleration[dof]);
        let j_max = inp.max_jerk[dof];

        match (
            self.inp_per_dof_control_interface[dof],
            ProfileOrder::of(inp, dof),
        ) {
            (ControlInterface::Position, ProfileOrder::Third) => p.check_with_timing_full(
                signs, limits, t_profile, control, v_max, v_min, a_max, a_min, j_max,
            ),
            (ControlInterface::Position, ProfileOrder::Second) => p
                .check_for_second_order_with_timing_full(
                    signs, limits, t_profile, control, -control, v_max, v_min, a_max, a_min,
                ),
            (ControlInterface::Position, ProfileOrder::First) => p
                .check_for_first_order_with_timing_full(
                    signs, limits, t_profile, control, v_max, v_min,
                ),
            (ControlInterface::Velocity, ProfileOrder::Third) => p
                .check_for_velocity_with_timing_full(
                    t_profile, signs, limits, control, a_max, a_min, j_max,
                ),
            (ControlInterface::Velocity, _) => p
                .check_for_second_order_velocity_with_timing_a_limits(
                    signs, limits, t_profile, control, a_max, a_min,
                ),
            (ControlInterface::Acceleration, _) => true,
        }
    }

    /// Active position-limit enforcement after step 1: if any block profile of
    /// `dof` would leave the position limits, shrink the effective velocity
    /// limits by bisection and re-run step 1, so that the whole feasible
    /// duration interval respects the limits. Returns false if no compliant
    /// profile was found within the iteration budget.
    fn enforce_position_limits_step1(
        &mut self,
        dof: usize,
        inp: &InputParameter<DOF>,
        p: &mut Profile,
    ) -> bool {
        const MAX_ITERS: usize = 24;

        let (up_viol, down_viol) = self.block_position_violations(dof);
        if !up_viol && !down_viol {
            return true;
        }

        let mut up = VelocityBisection::new(p.vf.max(0.0), self.eff_max_velocity[dof], up_viol);
        let mut down =
            VelocityBisection::new((-p.vf).max(0.0), -self.eff_min_velocity[dof], down_viol);
        let mut best: Option<(f64, f64)> = None;

        for _ in 0..MAX_ITERS {
            if up.active {
                self.eff_max_velocity[dof] = up.midpoint();
            }
            if down.active {
                self.eff_min_velocity[dof] = -down.midpoint();
            }
            let (v_up, v_down) = (self.eff_max_velocity[dof], -self.eff_min_velocity[dof]);

            if !self.run_step1(dof, inp, p) {
                // Too restrictive for any feasible profile: enlarge again
                up.enlarge(v_up);
                down.enlarge(v_down);
                continue;
            }

            let (up_viol, down_viol) = self.block_position_violations(dof);
            up.update(v_up, up_viol);
            down.update(v_down, down_viol);

            if !up_viol && !down_viol {
                best = Some((self.eff_max_velocity[dof], self.eff_min_velocity[dof]));
                if up.converged() && down.converged() {
                    break;
                }
            }
        }

        let Some((v_max, v_min)) = best else {
            return false;
        };
        self.eff_max_velocity[dof] = v_max;
        self.eff_min_velocity[dof] = v_min;
        // Leave the block in the compliant state
        self.run_step1(dof, inp, p)
    }

    /// Last-resort repair when a synchronized (step 2 / phase-sync) profile
    /// leaves the position limits: shrink the effective velocity limits (or the
    /// clamped target velocity for the velocity interface) by bisection and
    /// re-run step 2 for this DoF only, keeping the trajectory duration fixed.
    fn repair_profile_step2(
        &mut self,
        dof: usize,
        inp: &InputParameter<DOF>,
        duration: f64,
        profile: &mut Profile,
    ) -> bool {
        const MAX_ITERS: usize = 16;

        let original = *profile;
        let t_profile = duration - profile.brake.duration - profile.accel.duration;
        let mut best: Option<Profile> = None;

        match self.inp_per_dof_control_interface[dof] {
            ControlInterface::Position => {
                let (up_viol, down_viol) =
                    self.profile_position_violations(dof, inp, profile, duration);
                if !up_viol && !down_viol {
                    return true;
                }

                let mut v_up = self.eff_max_velocity[dof];
                let mut v_down = -self.eff_min_velocity[dof];
                let mut up = VelocityBisection::new(profile.vf.max(0.0), v_up, up_viol);
                let mut down = VelocityBisection::new((-profile.vf).max(0.0), v_down, down_viol);

                for _ in 0..MAX_ITERS {
                    if up.active {
                        up.hi = up.hi.min(v_up);
                        v_up = up.midpoint();
                    }
                    if down.active {
                        down.hi = down.hi.min(v_down);
                        v_down = down.midpoint();
                    }

                    *profile = original;
                    if !self.run_step2(dof, inp, t_profile, v_up, -v_down, profile) {
                        // Too restrictive for this fixed duration: enlarge again
                        up.enlarge(v_up);
                        down.enlarge(v_down);
                        continue;
                    }

                    let (up_viol, down_viol) =
                        self.profile_position_violations(dof, inp, profile, duration);
                    up.update(v_up, up_viol);
                    down.update(v_down, down_viol);

                    if !up_viol && !down_viol {
                        best = Some(*profile);
                        break;
                    }
                }
            }
            ControlInterface::Velocity => {
                let mut lo = 0.0;
                let mut hi = profile.vf.abs();
                let sign = if profile.vf < 0.0 { -1.0 } else { 1.0 };

                for _ in 0..MAX_ITERS {
                    let vf = sign * 0.5 * (lo + hi);
                    *profile = original;
                    profile.vf = vf;

                    // The velocity interface has no velocity limits
                    if !self.run_step2(dof, inp, t_profile, 0.0, 0.0, profile) {
                        lo = 0.5 * (lo + hi);
                        continue;
                    }

                    let (up_viol, down_viol) =
                        self.profile_position_violations(dof, inp, profile, duration);
                    if up_viol || down_viol {
                        hi = vf.abs();
                    } else {
                        best = Some(*profile);
                        lo = vf.abs();
                        if hi - lo <= 1e-9 * hi.max(1.0) {
                            break;
                        }
                    }
                }
            }
            ControlInterface::Acceleration => {}
        }

        *profile = best.unwrap_or(original);
        best.is_some()
    }

    /// Calculate the time-optimal state-to-state trajectory, enforcing optional
    /// position limits (`InputParameter::min_position` / `max_position`).
    pub fn calculate<T: RuckigErrorHandler>(
        &mut self,
        inp: &InputParameter<DOF>,
        traj: &mut Trajectory<DOF>,
        delta_time: f64,
    ) -> Result<RuckigResult, RuckigError> {
        self.invalidate_step1(None);
        let result =
            self.calculate_section::<T>(inp, traj, delta_time, DurationBudget::UNLIMITED)?;
        // An unlimited budget cannot be exceeded
        Ok(result.unwrap_or(RuckigResult::Working))
    }

    /// Forget the cached step 1 result of `dof` (or of all DoFs)
    #[inline]
    pub(crate) fn invalidate_step1(&mut self, dof: Option<usize>) {
        match dof {
            Some(dof) => self.step1_valid[dof] = false,
            None => self.step1_valid.iter_mut().for_each(|valid| *valid = false),
        }
    }

    /// Like [`calculate`](Self::calculate), but
    /// - returns `None` (leaving `traj` incomplete) as soon as the duration is
    ///   known to exceed `budget`, and
    /// - reuses the step 1 results of all DoFs not invalidated by
    ///   [`invalidate_step1`](Self::invalidate_step1) since the last call.
    ///   The caller guarantees that for those DoFs neither the boundary
    ///   states nor the limits of `inp` changed.
    pub(crate) fn calculate_section<T: RuckigErrorHandler>(
        &mut self,
        inp: &InputParameter<DOF>,
        traj: &mut Trajectory<DOF>,
        delta_time: f64,
        budget: DurationBudget,
    ) -> Result<Option<RuckigResult>, RuckigError> {
        let result = self.calculate_internal::<T>(inp, traj, delta_time, budget)?;
        if result != Some(RuckigResult::Working) || !self.position_limits_active {
            return Ok(result);
        }

        for dof in 0..self.degrees_of_freedom {
            if !inp.enabled[dof]
                || self.inp_per_dof_control_interface[dof] == ControlInterface::Acceleration
            {
                continue;
            }

            let duration = traj.duration;
            let (up, down) =
                self.profile_position_violations(dof, inp, &traj.profiles[0][dof], duration);
            if !up && !down {
                continue;
            }

            let mut profile = traj.profiles[0][dof];

            // A DoF without synchronization that ends before the trajectory
            // keeps moving with its target velocity until the other DoFs
            // finish. If that leaves the position limits, let it reach its
            // target state at the end of the trajectory instead.
            if profile.total_duration() < duration - self.eps {
                let t_profile = duration - profile.brake.duration - profile.accel.duration;
                let (v_max, v_min) = (self.eff_max_velocity[dof], self.eff_min_velocity[dof]);
                let mut synchronized = profile;
                if self.run_step2(dof, inp, t_profile, v_max, v_min, &mut synchronized) {
                    profile = synchronized;
                }
            }

            if self.repair_profile_step2(dof, inp, duration, &mut profile) {
                traj.profiles[0][dof] = profile;
                traj.phase_synchronized_sections[0] = false;
                continue;
            }

            self.report::<T>(format_args!(
                "trajectory of DoF {} exceeds position limits, input: {}",
                dof, inp
            ))?;
            return Ok(Some(RuckigResult::ErrorPositionalLimits));
        }

        Ok(Some(RuckigResult::Working))
    }

    /// Resolve the per-DoF settings and limits of `inp` into the calculator's
    /// scratch arrays
    #[inline(always)]
    fn resolve_input(&mut self, inp: &InputParameter<DOF>) {
        self.position_limits_active = inp.max_position.is_some() || inp.min_position.is_some();

        for dof in 0..self.degrees_of_freedom {
            self.inp_per_dof_control_interface[dof] = inp
                .per_dof_control_interface
                .as_ref()
                .and_then(|v| v.get(dof).copied())
                .unwrap_or(inp.control_interface);
            self.inp_per_dof_synchronization[dof] = inp
                .per_dof_synchronization
                .as_ref()
                .and_then(|v| v.get(dof).copied())
                .unwrap_or(inp.synchronization);

            self.inp_max_position[dof] =
                inp.max_position.as_ref().map_or(f64::INFINITY, |v| v[dof]);
            self.inp_min_position[dof] = inp
                .min_position
                .as_ref()
                .map_or(f64::NEG_INFINITY, |v| v[dof]);
            self.inp_min_velocity[dof] = inp
                .min_velocity
                .as_ref()
                .map_or(-inp.max_velocity[dof], |v| v[dof]);
            self.inp_min_acceleration[dof] = inp
                .min_acceleration
                .as_ref()
                .map_or(-inp.max_acceleration[dof], |v| v[dof]);

            // A reused step 1 result keeps its (possibly position-limited)
            // effective velocity limits
            if !self.step1_valid[dof] {
                self.eff_max_velocity[dof] = inp.max_velocity[dof];
                self.eff_min_velocity[dof] = self.inp_min_velocity[dof];
            }
        }
    }

    fn calculate_internal<T: RuckigErrorHandler>(
        &mut self,
        inp: &InputParameter<DOF>,
        traj: &mut Trajectory<DOF>,
        delta_time: f64,
        budget: DurationBudget,
    ) -> Result<Option<RuckigResult>, RuckigError> {
        // A state-to-state trajectory has exactly one section; shrink a
        // trajectory that previously held intermediate waypoints
        traj.resize_sections(1);
        traj.phase_synchronized_sections[0] = false;

        self.resolve_input(inp);

        for dof in 0..self.degrees_of_freedom {
            let p = &mut traj.profiles[0][dof];

            if !inp.enabled[dof] {
                // A disabled DoF keeps its current state, extrapolated with
                // constant acceleration when sampled
                *p = Profile::default();
                p.p = [inp.current_position[dof]; 8];
                p.v = [inp.current_velocity[dof]; 8];
                p.a = [inp.current_acceleration[dof]; 8];
                p.pf = inp.current_position[dof];
                p.vf = inp.current_velocity[dof];
                p.af = inp.current_acceleration[dof];

                self.blocks[dof].t_min = 0.0;
                self.blocks[dof].a = None;
                self.blocks[dof].b = None;
                continue;
            }

            self.set_brake_and_boundary(dof, inp, p);

            if self.step1_valid[dof] {
                traj.independent_min_durations[dof] = self.blocks[dof].t_min;
                continue;
            }

            let found_profile = self.run_step1(dof, inp, p);
            if found_profile
                && self.position_limits_active
                && self.inp_per_dof_control_interface[dof] == ControlInterface::Position
                && !self.enforce_position_limits_step1(dof, inp, p)
            {
                self.report::<T>(format_args!(
                    "no profile within position limits found in step 1, dof: {} input: {}",
                    dof, inp
                ))?;
                return Ok(Some(RuckigResult::ErrorPositionalLimits));
            }

            if !found_profile {
                if self.has_zero_limits(inp, dof) {
                    self.report::<T>(format_args!(
                        "zero limits conflict in step 1, dof: {} input: {}",
                        dof, inp
                    ))?;
                    return Ok(Some(RuckigResult::ErrorZeroLimits));
                }
                self.report::<T>(format_args!("error in step 1, dof: {} input: {}", dof, inp))?;
                return Ok(Some(RuckigResult::ErrorExecutionTimeCalculation));
            }

            self.step1_valid[dof] = true;
            traj.independent_min_durations[dof] = self.blocks[dof].t_min;
        }

        let discrete_duration = inp.duration_discretization == DurationDiscretization::Discrete;
        if self.degrees_of_freedom == 1 && inp.minimum_duration.is_none() && !discrete_duration {
            traj.duration = self.blocks[0].t_min;
            traj.profiles[0][0] = self.blocks[0].p_min;
            traj.cumulative_times[0] = traj.duration;
            if budget.is_exceeded_by(traj.duration) {
                return Ok(None);
            }
            return Ok(Some(RuckigResult::Working));
        }

        // Repeated only if step 2 disproves a duration that step 1 considered
        // feasible, with that duration blocked (at most twice per DoF)
        'synchronization: loop {
            traj.phase_synchronized_sections[0] = false;
            let mut limiting_dof: Option<usize> = None; // The DoF that doesn't need step 2
            let found_synchronization = self.synchronize(
                inp.minimum_duration,
                &mut traj.duration,
                &mut limiting_dof,
                &mut traj.profiles[0],
                discrete_duration,
                delta_time,
            );
            if !found_synchronization {
                if (0..self.degrees_of_freedom).any(|dof| self.has_zero_limits(inp, dof)) {
                    self.report::<T>(format_args!(
                        "zero limits conflict with other degrees of freedom in time synchronization {}",
                        traj.duration
                    ))?;
                    return Ok(Some(RuckigResult::ErrorZeroLimits));
                }
                self.report::<T>(format_args!(
                    "error in time synchronization: {}",
                    traj.duration
                ))?;
                return Ok(Some(RuckigResult::ErrorSynchronizationCalculation));
            }

            // None Synchronization
            for dof in 0..self.degrees_of_freedom {
                if inp.enabled[dof]
                    && self.inp_per_dof_synchronization[dof] == Synchronization::None
                {
                    traj.profiles[0][dof] = self.blocks[dof].p_min;
                    if self.blocks[dof].t_min > traj.duration {
                        traj.duration = self.blocks[dof].t_min;
                        limiting_dof = Some(dof);
                    }
                }
            }
            traj.cumulative_times[0] = traj.duration;

            if budget.is_exceeded_by(traj.duration) {
                return Ok(None);
            }

            if self.return_error_at_maximal_duration && traj.duration > 7.6e3 {
                return Ok(Some(RuckigResult::ErrorTrajectoryDuration));
            }

            if traj.duration.abs() < f64::EPSILON {
                // Copy all profiles for end state
                for dof in 0..self.degrees_of_freedom {
                    if inp.enabled[dof] {
                        traj.profiles[0][dof] = self.blocks[dof].p_min;
                    }
                }
                return Ok(Some(RuckigResult::Working));
            }

            let synchronizations = &self.inp_per_dof_synchronization;
            if !discrete_duration && synchronizations.iter().all(|s| *s == Synchronization::None) {
                return Ok(Some(RuckigResult::Working));
            }

            // Phase Synchronization
            if let Some(limiting_dof) = limiting_dof {
                if synchronizations
                    .iter()
                    .any(|s| *s == Synchronization::Phase)
                {
                    let p_limiting = traj.profiles[0][limiting_dof];
                    if self.is_input_collinear(inp, p_limiting.direction, limiting_dof) {
                        let mut found_time_synchronization = true;
                        for dof in 0..self.degrees_of_freedom {
                            if !inp.enabled[dof]
                                || dof == limiting_dof
                                || self.inp_per_dof_synchronization[dof] != Synchronization::Phase
                            {
                                continue;
                            }

                            let p = &mut traj.profiles[0][dof];
                            let t_profile = traj.duration - p.brake.duration - p.accel.duration;

                            p.t = p_limiting.t; // Copy timing information from limiting DoF
                            p.control_signs = p_limiting.control_signs;

                            found_time_synchronization &=
                                self.check_phase_profile(dof, inp, t_profile, p);

                            p.limits = p_limiting.limits; // After check method call to set correct limits
                        }

                        if found_time_synchronization
                            && self.inp_per_dof_synchronization.iter().all(|s| {
                                matches!(s, Synchronization::Phase | Synchronization::None)
                            })
                        {
                            traj.phase_synchronized_sections[0] =
                                (0..self.degrees_of_freedom).all(|dof| {
                                    !inp.enabled[dof]
                                        || (self.inp_per_dof_synchronization[dof]
                                            == Synchronization::Phase
                                            && traj.profiles[0][dof].brake.duration == 0.0
                                            && traj.profiles[0][dof].accel.duration == 0.0)
                                });
                            return Ok(Some(RuckigResult::Working));
                        }
                    }
                }
            }

            // Time Synchronization
            for dof in 0..self.degrees_of_freedom {
                let skip_synchronization = (Some(dof) == limiting_dof
                    || self.inp_per_dof_synchronization[dof] == Synchronization::None)
                    && !discrete_duration;
                if !inp.enabled[dof] || skip_synchronization {
                    continue;
                }

                let block = &self.blocks[dof];
                let p = &mut traj.profiles[0][dof];
                let t_profile = traj.duration - p.brake.duration - p.accel.duration;

                if self.inp_per_dof_synchronization[dof] == Synchronization::TimeIfNecessary
                    && inp.target_velocity[dof].abs() < self.eps
                    && inp.target_acceleration[dof].abs() < self.eps
                {
                    *p = block.p_min;
                    continue;
                }

                // Check if the final time corresponds to an extremal profile calculated in step 1
                // Use 2*eps because of numerical robustness in duration discretization
                if let Some(extremal) = block.extremal_profile_at(t_profile, 2.0 * self.eps) {
                    *p = *extremal;
                    continue;
                }

                let (v_max, v_min) = (self.eff_max_velocity[dof], self.eff_min_velocity[dof]);
                if !self.run_step2(dof, inp, t_profile, v_max, v_min, p) {
                    // Durations within rounding distance of an extremal profile
                    // are degenerate for step 2 (e.g. two DoFs whose minimum
                    // durations differ in the last bits): the extremal profile
                    // itself is the solution then. The block durations include
                    // the brake pre-trajectory, so compare the total duration.
                    let tolerance = STEP2_EXTREMAL_TOLERANCE * traj.duration.abs().max(1.0);
                    if let Some(extremal) = block.extremal_profile_at(traj.duration, tolerance) {
                        *p = *extremal;
                        continue;
                    }

                    // Otherwise the duration is not feasible for this DoF after
                    // all: block it and synchronize again at a later duration
                    if self.blocks[dof].block_until_next_extremal(traj.duration) {
                        continue 'synchronization;
                    }

                    self.report::<T>(format_args!(
                        "error in step 2 in dof: {} for t sync: {} input: {}",
                        dof, traj.duration, inp
                    ))?;
                    return Ok(Some(RuckigResult::ErrorSynchronizationCalculation));
                }
            }

            return Ok(Some(RuckigResult::Working));
        }
    }
}
