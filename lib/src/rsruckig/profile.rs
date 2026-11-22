//! Motion profiles for trajectory generation with kinematic constraints
//!
//! This module defines the motion profiles used for trajectory generation,
//! which represent mathematically optimal motion under kinematic constraints.

use crate::brake::BrakeProfile;
use crate::roots;
use crate::util::integrate;
use core::fmt;

#[cfg(not(feature = "std"))]
use num_traits::Float;

const V_EPS: f64 = 1e-12;
const A_EPS: f64 = 1e-12;
const J_EPS: f64 = 1e-12;

const P_PRECISION: f64 = 1e-8;
const V_PRECISION: f64 = 1e-8;
const A_PRECISION: f64 = 1e-10;

const T_MAX: f64 = 1e12;

/// The kinematic limits that a profile reaches.
///
/// `Acc0` is the acceleration limit in the first (acceleration) part of the
/// profile, `Acc1` the limit in the last (deceleration) part, and `Vel` the
/// velocity limit in the cruise phase.
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub enum ReachedLimits {
    /// Both acceleration limits and the velocity limit.
    Acc0Acc1Vel,
    /// Only the velocity limit.
    Vel,
    /// Only the first acceleration limit.
    Acc0,
    /// Only the second acceleration limit.
    Acc1,
    /// Both acceleration limits.
    Acc0Acc1,
    /// The first acceleration limit and the velocity limit.
    Acc0Vel,
    /// The second acceleration limit and the velocity limit.
    Acc1Vel,
    /// No limits.
    #[default]
    None,
}

/// Whether a profile uses the limits in their nominal (`UP`) or swapped (`DOWN`)
/// orientation, i.e. whether it moves mainly in positive or negative direction.
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub enum Direction {
    /// Nominal limits (`v_max`, `a_max`, ...).
    #[default]
    UP,
    /// Swapped limits (`v_min`, `a_min`, ...).
    DOWN,
}

/// The sign pattern of the jerk (or acceleration) in the four non-constant
/// phases of a profile: `U` = up (positive), `D` = down (negative).
#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub enum ControlSigns {
    /// Up, down, down, up.
    #[default]
    UDDU,
    /// Up, down, up, down.
    UDUD,
}

/// Position extrema of a profile or trajectory and the times they are reached.
#[derive(Debug, Clone, Copy, Default)]
pub struct Bound {
    /// The minimum position.
    pub min: f64,
    /// The maximum position.
    pub max: f64,
    /// The time at which the minimum position is reached.
    pub t_min: f64,
    /// The time at which the maximum position is reached.
    pub t_max: f64,
}

/// The state profile for position, velocity, acceleration and jerk for a single DoF
///
/// This structure holds the complete kinematic profile for a single degree of freedom.
/// It includes:
/// - Time intervals for each trajectory segment
/// - Jerk, acceleration, velocity, and position profiles
/// - Target kinematic state
/// - Information about which limits were reached
///
/// The profile consists of up to 7 phases, each with constant jerk, following
/// a time-optimal bang-bang approach for trajectory generation.
///
/// This is a low-level representation used internally by the algorithm, but
/// exposed for advanced use cases.
#[derive(Debug, Clone, Copy, Default)]
pub struct Profile {
    /// Duration of each phase (up to 7 phases)
    pub t: [f64; 7],

    /// Cumulative time at the end of each phase
    pub t_sum: [f64; 7],

    /// Jerk value for each phase
    pub j: [f64; 7],

    /// Acceleration at the start of each phase (`a[7]` is the final acceleration)
    pub a: [f64; 8],

    /// Velocity at the start of each phase (`v[7]` is the final velocity)
    pub v: [f64; 8],

    /// Position at the start of each phase (`p[7]` is the final position)
    pub p: [f64; 8],

    /// Brake pre-trajectory that brings an initial state violating the limits
    /// back within them before the main profile starts
    pub brake: BrakeProfile,

    /// Post-trajectory appended after the main profile (currently always of
    /// zero duration, but included in the total duration)
    pub accel: BrakeProfile,

    /// Target final position
    pub pf: f64,

    /// Target final velocity
    pub vf: f64,

    /// Target final acceleration
    pub af: f64,

    /// Information about which kinematic limits were reached
    pub limits: ReachedLimits,

    /// Direction of motion
    pub direction: Direction,

    /// Control sign configuration
    pub control_signs: ControlSigns,
}

/// The acceleration limits that a profile reaches by the construction of its
/// durations, for [`Profile::check_robust`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ConstructedLimits {
    /// The first phase ends at the upper acceleration limit (`a[1] == a_max`).
    pub acc0: bool,
    /// The third phase ends at the lower acceleration limit (`a[3] == a_min`),
    /// for profiles without a cruise phase.
    pub acc1_peak: bool,
    /// The fifth phase ends at the lower acceleration limit (`a[5] == a_min`).
    pub acc1: bool,
}

impl ConstructedLimits {
    /// No limit is reached by construction.
    pub const NONE: Self = Self { acc0: false, acc1_peak: false, acc1: false };
    /// `a[1] == a_max`
    pub const ACC0: Self = Self { acc0: true, acc1_peak: false, acc1: false };
    /// `a[3] == a_min`
    pub const ACC1_PEAK: Self = Self { acc0: false, acc1_peak: true, acc1: false };
    /// `a[5] == a_min`
    pub const ACC1: Self = Self { acc0: false, acc1_peak: false, acc1: true };
    /// `a[1] == a_max` and `a[5] == a_min`
    pub const ACC0_ACC1: Self = Self { acc0: true, acc1_peak: false, acc1: true };
}

/// Jerk of each of the seven phases for the given control signs: phases with
/// zero duration carry no jerk, the constant-acceleration and cruise phases
/// (1, 3, 5) never do.
#[inline(always)]
fn jerk_pattern(control_signs: ControlSigns, t: &[f64; 7], jf: f64) -> [f64; 7] {
    let (j4, j6) = match control_signs {
        ControlSigns::UDDU => (-jf, jf),
        ControlSigns::UDUD => (jf, -jf),
    };
    let active = |t: f64, j: f64| if t > 0.0 { j } else { 0.0 };
    [
        active(t[0], jf),
        0.0,
        active(t[2], -jf),
        0.0,
        active(t[4], j4),
        0.0,
        active(t[6], j6),
    ]
}

impl Profile {
    /// Validates a third-order velocity-control profile whose phase durations `t`
    /// have been set.
    ///
    /// Computes the cumulative times, the jerk pattern for `control_signs` with
    /// jerk `jf`, and integrates the states. Sets `control_signs`, `limits` and
    /// `direction`. Returns `true` if all durations are non-negative, the final
    /// velocity and acceleration match the target and the acceleration limits
    /// hold.
    #[inline]
    pub fn check_for_velocity(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        jf: f64,
        a_max: f64,
        a_min: f64,
    ) -> bool {
        self.check_for_velocity_impl(control_signs, limits, false, jf, a_max, a_min)
    }

    /// [`check_for_velocity`](Self::check_for_velocity) for a profile whose
    /// first phase ends at the acceleration limit `a_max` by construction:
    /// the acceleration is snapped exactly to the limit there, so that the
    /// rounding of the integration cannot exceed it.
    #[inline]
    pub fn check_for_velocity_at_acceleration_limit(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        jf: f64,
        a_max: f64,
        a_min: f64,
    ) -> bool {
        self.check_for_velocity_impl(control_signs, limits, true, jf, a_max, a_min)
    }

    #[inline(always)]
    fn check_for_velocity_impl(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        set_limits: bool,
        jf: f64,
        a_max: f64,
        a_min: f64,
    ) -> bool {
        if self.t[0] < 0.0 {
            return false;
        }

        self.t_sum[0] = self.t[0];
        for i in 0..6 {
            if self.t[i + 1] < 0.0 {
                return false;
            }

            self.t_sum[i + 1] = self.t_sum[i] + self.t[i + 1];
        }

        if (limits == ReachedLimits::Acc0) && self.t[1] < f64::EPSILON {
            return false;
        }

        if self.t_sum[6] > T_MAX {
            // Use T_PRECISION for numerical reasons.
            return false;
        }

        self.j = jerk_pattern(control_signs, &self.t, jf);

        for i in 0..7 {
            self.a[i + 1] = self.a[i] + self.t[i] * self.j[i];
            self.v[i + 1] = self.v[i] + self.t[i] * (self.a[i] + self.t[i] * self.j[i] / 2.0);
            self.p[i + 1] = self.p[i]
                + self.t[i]
                    * (self.v[i] + self.t[i] * (self.a[i] / 2.0 + self.t[i] * self.j[i] / 6.0));

            if set_limits && i == 0 {
                self.a[1] = a_max;
            }
        }

        self.control_signs = control_signs;
        self.limits = limits;

        self.direction = if a_max > 0.0 {
            Direction::UP
        } else {
            Direction::DOWN
        };
        let a_upp_lim = match self.direction {
            Direction::UP => a_max + A_EPS,
            Direction::DOWN => a_min + A_EPS,
        };
        let a_low_lim = match self.direction {
            Direction::UP => a_min - A_EPS,
            Direction::DOWN => a_max - A_EPS,
        };

        // For Velocity limit checks. Here I'm using V_PRECISION and A_PRECISION for clarity.
        (self.v[7] - self.vf).abs() < V_PRECISION
            && (self.a[7] - self.af).abs() < A_PRECISION
            && self.a[1] >= a_low_lim
            && self.a[3] >= a_low_lim
            && self.a[5] >= a_low_lim
            && self.a[1] <= a_upp_lim
            && self.a[3] <= a_upp_lim
            && self.a[5] <= a_upp_lim
    }

    /// [`check_for_velocity`](Self::check_for_velocity) for a profile with a
    /// prescribed duration `_tf`.
    ///
    /// The duration is not checked separately, since every profile with a
    /// prescribed duration is constructed from a `tf - ...` equation.
    #[inline]
    pub fn check_for_velocity_with_timing(
        &mut self,
        _tf: f64,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        jf: f64,
        a_max: f64,
        a_min: f64,
    ) -> bool {
        // Time doesn't need to be checked as every profile has a: tf - ... equation
        self.check_for_velocity(control_signs, limits, jf, a_max, a_min)
    }

    /// Like [`check_for_velocity_with_timing`](Self::check_for_velocity_with_timing),
    /// but additionally requires `|jf| <= |j_max|`.
    #[inline]
    pub fn check_for_velocity_with_timing_full(
        &mut self,
        tf: f64,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        jf: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) -> bool {
        jf.abs() < j_max.abs() + J_EPS
            && self.check_for_velocity_with_timing(tf, control_signs, limits, jf, a_max, a_min)
    }

    /// Sets the initial state and the target velocity and acceleration for
    /// velocity control. The target position is left unchanged.
    #[inline]
    pub fn set_boundary_for_velocity(
        &mut self,
        p0_new: f64,
        v0_new: f64,
        a0_new: f64,
        vf_new: f64,
        af_new: f64,
    ) {
        self.a[0] = a0_new;
        self.v[0] = v0_new;
        self.p[0] = p0_new;
        self.af = af_new;
        self.vf = vf_new;
    }

    /// Validates a second-order velocity-control profile (infinite jerk) with a
    /// single constant-acceleration phase `t[1]` of acceleration `a_up`.
    ///
    /// Returns `true` if the final velocity matches the target.
    #[inline]
    pub fn check_for_second_order_velocity(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        a_up: f64,
    ) -> bool {
        // ReachedLimits::ACC0
        if self.t[1] < 0.0 {
            return false;
        }

        self.t_sum = [
            0.0, self.t[1], self.t[1], self.t[1], self.t[1], self.t[1], self.t[1],
        ];
        if self.t_sum[6] > T_MAX {
            return false;
        }

        self.j = [0.0; 7];
        self.a = [0.0; 8];
        self.a[1] = if self.t[1] > 0.0 { a_up } else { 0.0 };
        self.a[7] = self.af; // Assuming `af` is a member of Profile

        for i in 0..7 {
            self.v[i + 1] = self.v[i] + self.t[i] * self.a[i];
            self.p[i + 1] = self.p[i] + self.t[i] * (self.v[i] + self.t[i] * self.a[i] / 2.0);
        }

        self.control_signs = control_signs;
        self.limits = limits;

        self.direction = if a_up > 0.0 {
            Direction::UP
        } else {
            Direction::DOWN
        };

        (self.v[7] - self.vf).abs() < V_PRECISION
    }

    /// [`check_for_second_order_velocity`](Self::check_for_second_order_velocity)
    /// for a profile with a prescribed duration `_tf`.
    #[inline]
    pub fn check_for_second_order_velocity_with_timing(
        &mut self,
        _tf: f64,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        a_up: f64,
    ) -> bool {
        self.check_for_second_order_velocity(control_signs, limits, a_up)
    }

    /// Like [`check_for_second_order_velocity_with_timing`](Self::check_for_second_order_velocity_with_timing),
    /// but additionally requires `a_min <= a_up <= a_max`.
    #[inline]
    pub fn check_for_second_order_velocity_with_timing_a_limits(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        _tf: f64,
        a_up: f64,
        a_max: f64,
        a_min: f64,
    ) -> bool {
        a_min - A_EPS < a_up
            && a_up < a_max + A_EPS
            && self.check_for_second_order_velocity_with_timing(_tf, control_signs, limits, a_up)
    }

    /// Validates a third-order position-control profile whose phase durations
    /// `t` have been set.
    ///
    /// Computes the cumulative times, the jerk pattern for `control_signs` with
    /// jerk `jf`, and integrates the states. If `set_limits` is `true`, the
    /// accelerations of phases that reach a limit are snapped exactly to it.
    /// Sets `control_signs`, `limits` and `direction`. Returns `true` if all
    /// durations are non-negative and consistent with `limits`, the final state
    /// matches the target, and the velocity and acceleration limits hold.
    #[inline(always)]
    pub fn check(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        set_limits: bool,
        jf: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
    ) -> bool {
        self.check_impl(control_signs, limits, set_limits, None, jf, v_max, v_min, a_max, a_min)
    }

    /// [`check`](Self::check) for numerically degenerate profiles, used when
    /// the regular check has rejected all candidates of step 1.
    ///
    /// The regular check integrates the profile and compares the result with
    /// the limits within absolute tolerances of 1e-12. Rounding exceeds these
    /// for accelerations (or velocities) of some thousands, whose resolution
    /// is of that order, and it rejects profiles at the border between two
    /// profile types. This check therefore
    /// - snaps the accelerations to the limits that the profile reaches by
    ///   construction, as given by `constructed`,
    /// - accepts phases of constant acceleration with a vanishing duration
    ///   and takes durations that are negative by rounding for zero, and
    /// - uses tolerances not below the resolution of the limits.
    ///
    /// The final state is checked like in the regular check.
    #[allow(clippy::too_many_arguments)]
    #[inline]
    pub fn check_robust(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        constructed: ConstructedLimits,
        jf: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
    ) -> bool {
        self.check_impl(
            control_signs,
            limits,
            false,
            Some(constructed),
            jf,
            v_max,
            v_min,
            a_max,
            a_min,
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    fn check_impl(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        set_limits: bool,
        robust: Option<ConstructedLimits>,
        jf: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
    ) -> bool {
        if robust.is_some() {
            // Durations that are negative by rounding
            let magnitude: f64 = self.t.iter().map(|t| t.abs()).sum();
            let tolerance = 8.0 * f64::EPSILON * f64::max(1.0, magnitude);
            for t in self.t.iter_mut() {
                if *t < 0.0 && *t > -tolerance {
                    *t = 0.0;
                }
            }
        }

        if self.t[0] < 0.0 {
            return false;
        }

        self.t_sum[0] = self.t[0];
        for i in 0..6 {
            if self.t[i + 1] < 0.0 {
                return false;
            }
            self.t_sum[i + 1] = self.t_sum[i] + self.t[i + 1];
        }

        if matches!(
            limits,
            ReachedLimits::Acc0Acc1Vel
                | ReachedLimits::Acc0Vel
                | ReachedLimits::Acc1Vel
                | ReachedLimits::Vel
        ) && self.t[3] < f64::EPSILON
        {
            return false;
        }

        if robust.is_none()
            && matches!(limits, ReachedLimits::Acc0 | ReachedLimits::Acc0Acc1)
            && self.t[1] < f64::EPSILON
        {
            return false;
        }

        if robust.is_none()
            && matches!(limits, ReachedLimits::Acc1 | ReachedLimits::Acc0Acc1)
            && self.t[5] < f64::EPSILON
        {
            return false;
        }

        if self.t_sum[6] > T_MAX {
            return false;
        }

        self.j = jerk_pattern(control_signs, &self.t, jf);

        // The absolute tolerances are below the resolution of limits beyond
        // some thousands
        let tolerance = |limit: f64, eps: f64| {
            if robust.is_some() {
                f64::max(eps, 16.0 * f64::EPSILON * limit.abs())
            } else {
                eps
            }
        };

        self.direction = if v_max > 0.0 {
            Direction::UP
        } else {
            Direction::DOWN
        };

        let (v_upp, v_low) = if self.direction == Direction::UP {
            (v_max, v_min)
        } else {
            (v_min, v_max)
        };
        let v_upp_lim = v_upp + tolerance(v_upp, V_EPS);
        let v_low_lim = v_low - tolerance(v_low, V_EPS);

        for i in 0..7 {
            self.a[i + 1] = self.a[i] + self.t[i] * self.j[i];
            self.v[i + 1] = self.v[i] + self.t[i] * (self.a[i] + self.t[i] * self.j[i] / 2.0);
            self.p[i + 1] = self.p[i]
                + self.t[i]
                    * (self.v[i] + self.t[i] * (self.a[i] / 2.0 + self.t[i] * self.j[i] / 6.0));

            if matches!(
                limits,
                ReachedLimits::Acc0Acc1Vel
                    | ReachedLimits::Acc0Acc1
                    | ReachedLimits::Acc0Vel
                    | ReachedLimits::Acc1Vel
                    | ReachedLimits::Vel
            ) && i == 2
            {
                self.a[3] = 0.0;
            }

            if let Some(constructed) = robust {
                if (i == 0 && constructed.acc0) || (i == 4 && constructed.acc1) {
                    self.a[i + 1] = if i == 0 { a_max } else { a_min };
                }
                if i == 2 && constructed.acc1_peak {
                    self.a[3] = a_min;
                }
            }

            if set_limits {
                match limits {
                    ReachedLimits::Acc1 if i == 2 => {
                        self.a[3] = a_min;
                    }
                    ReachedLimits::Acc0Acc1 => {
                        if i == 0 {
                            self.a[1] = a_max;
                        }
                        if i == 4 {
                            self.a[5] = a_min;
                        }
                    }
                    _ => {}
                }
            }

            if i > 1 && self.a[i + 1] * self.a[i] < -f64::EPSILON {
                let v_a_zero = self.v[i] - (self.a[i] * self.a[i]) / (2.0 * self.j[i]);
                if v_a_zero > v_upp_lim || v_a_zero < v_low_lim {
                    return false;
                }
            }
        }

        self.control_signs = control_signs;
        self.limits = limits;

        let (a_upp, a_low) = if self.direction == Direction::UP {
            (a_max, a_min)
        } else {
            (a_min, a_max)
        };
        let a_upp_lim = a_upp + tolerance(a_upp, A_EPS);
        let a_low_lim = a_low - tolerance(a_low, A_EPS);

        (self.p[7] - self.pf).abs() < P_PRECISION
            && (self.v[7] - self.vf).abs() < V_PRECISION
            && (self.a[7] - self.af).abs() < A_PRECISION
            && self.a[1] >= a_low_lim
            && self.a[1] <= a_upp_lim
            && self.a[3] >= a_low_lim
            && self.a[3] <= a_upp_lim
            && self.a[5] >= a_low_lim
            && self.a[5] <= a_upp_lim
            && self.v[3] <= v_upp_lim
            && self.v[3] >= v_low_lim
            && self.v[4] <= v_upp_lim
            && self.v[4] >= v_low_lim
            && self.v[5] <= v_upp_lim
            && self.v[5] >= v_low_lim
            && self.v[6] <= v_upp_lim
            && self.v[6] >= v_low_lim
    }

    /// [`check`](Self::check) (without snapping to limits) for a profile with
    /// a prescribed duration.
    ///
    /// The duration is not checked separately, since every profile with a
    /// prescribed duration is constructed from a `tf - ...` equation.
    #[inline]
    pub fn check_with_timing(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        jf: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
    ) -> bool {
        // Time doesn't need to be checked as every profile has a: tf - ... equation
        // Note: Uncomment the next part if t_precision is used later
        // && (self.t_sum.last().unwrap_or(&0.0) - tf).abs() < t_precision
        self.check(control_signs, limits, false, jf, v_max, v_min, a_max, a_min)
    }

    /// Like [`check_with_timing`](Self::check_with_timing), but additionally
    /// requires `|jf| <= |j_max|`.
    #[inline]
    pub fn check_with_timing_full(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        _tf: f64,
        jf: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) -> bool {
        (jf.abs() < j_max.abs() + J_EPS)
            && self.check_with_timing(control_signs, limits, jf, v_max, v_min, a_max, a_min)
    }

    /// Copies the initial state, the target state and the brake and
    /// acceleration sub-profiles from `profile`.
    #[inline]
    pub fn set_boundary_from_profile(&mut self, profile: &Profile) {
        self.a[0] = profile.a[0];
        self.v[0] = profile.v[0];
        self.p[0] = profile.p[0];
        self.af = profile.af;
        self.vf = profile.vf;
        self.pf = profile.pf;
        self.brake = profile.brake;
        self.accel = profile.accel;
    }

    /// Sets the initial state (`p0_new`, `v0_new`, `a0_new`) and the target
    /// state (`pf_new`, `vf_new`, `af_new`).
    #[inline]
    pub fn set_boundary(
        &mut self,
        p0_new: f64,
        v0_new: f64,
        a0_new: f64,
        pf_new: f64,
        vf_new: f64,
        af_new: f64,
    ) {
        self.a[0] = a0_new;
        self.v[0] = v0_new;
        self.p[0] = p0_new;
        self.af = af_new;
        self.vf = vf_new;
        self.pf = pf_new;
    }

    /// Validates a second-order position-control profile (infinite jerk) whose
    /// phase durations `t` have been set, using the accelerations `a_up` and
    /// `a_down` according to `control_signs`.
    ///
    /// Returns `true` if all durations are non-negative, the final position and
    /// velocity match the target and the velocity limits hold.
    #[inline]
    pub fn check_for_second_order(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        a_up: f64,
        a_down: f64,
        v_max: f64,
        v_min: f64,
    ) -> bool {
        if self.t[0] < 0.0 {
            return false;
        }

        self.t_sum[0] = self.t[0];
        for i in 0..6 {
            if self.t[i + 1] < 0.0 {
                return false;
            }

            self.t_sum[i + 1] = self.t_sum[i] + self.t[i + 1];
        }

        if self.t_sum[6] > T_MAX {
            return false;
        }

        self.j = [0.0; 7];

        match control_signs {
            ControlSigns::UDDU => {
                self.a = [
                    if self.t[0] > 0.0 { a_up } else { 0.0 },
                    0.0,
                    if self.t[2] > 0.0 { a_down } else { 0.0 },
                    0.0,
                    if self.t[4] > 0.0 { a_down } else { 0.0 },
                    0.0,
                    if self.t[6] > 0.0 { a_up } else { 0.0 },
                    self.af,
                ];
            }
            _ => {
                self.a = [
                    if self.t[0] > 0.0 { a_up } else { 0.0 },
                    0.0,
                    if self.t[2] > 0.0 { a_down } else { 0.0 },
                    0.0,
                    if self.t[4] > 0.0 { a_up } else { 0.0 },
                    0.0,
                    if self.t[6] > 0.0 { a_down } else { 0.0 },
                    self.af,
                ];
            }
        }

        self.direction = if v_max > 0.0 {
            Direction::UP
        } else {
            Direction::DOWN
        };

        let v_upp_lim = if self.direction == Direction::UP {
            v_max + V_EPS
        } else {
            v_min + V_EPS
        };
        let v_low_lim = if self.direction == Direction::UP {
            v_min - V_EPS
        } else {
            v_max - V_EPS
        };

        for i in 0..7 {
            self.v[i + 1] = self.v[i] + self.t[i] * self.a[i];
            self.p[i + 1] = self.p[i] + self.t[i] * (self.v[i] + self.t[i] * self.a[i] / 2.0);
        }

        self.control_signs = control_signs;
        self.limits = limits;

        (self.p[7] - self.pf).abs() < P_PRECISION
            && (self.v[7] - self.vf).abs() < V_PRECISION
            && self.v[2] <= v_upp_lim
            && self.v[2] >= v_low_lim
            && self.v[3] <= v_upp_lim
            && self.v[3] >= v_low_lim
            && self.v[4] <= v_upp_lim
            && self.v[4] >= v_low_lim
            && self.v[5] <= v_upp_lim
            && self.v[5] >= v_low_lim
            && self.v[6] <= v_upp_lim
            && self.v[6] >= v_low_lim
            && self.v[7] <= v_upp_lim
            && self.v[7] >= v_low_lim
    }

    /// [`check_for_second_order`](Self::check_for_second_order) for a profile
    /// with a prescribed duration.
    ///
    /// The duration is not checked separately, since every profile with a
    /// prescribed duration is constructed from a `tf - ...` equation.
    #[inline]
    pub fn check_for_second_order_with_timing(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        _: f64,
        a_up: f64,
        a_down: f64,
        v_max: f64,
        v_min: f64,
    ) -> bool {
        // Time doesn't need to be checked as every profile has a: tf - ... equation
        self.check_for_second_order(control_signs, limits, a_up, a_down, v_max, v_min)
        // && (f64::abs(self.t_sum.last().unwrap_or(&0.0) - tf) < T_PRECISION)
    }

    /// Like [`check_for_second_order_with_timing`](Self::check_for_second_order_with_timing),
    /// but additionally requires `a_up` and `a_down` to lie within
    /// `[a_min, a_max]`.
    #[inline]
    pub fn check_for_second_order_with_timing_full(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        tf: f64,
        a_up: f64,
        a_down: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
    ) -> bool {
        (a_min - A_EPS < a_up)
            && (a_up < a_max + A_EPS)
            && (a_min - A_EPS < a_down)
            && (a_down < a_max + A_EPS)
            && self.check_for_second_order_with_timing(
                control_signs,
                limits,
                tf,
                a_up,
                a_down,
                v_max,
                v_min,
            )
    }

    /// Validates a first-order position-control profile (infinite acceleration)
    /// with a single constant-velocity phase `t[3]` of velocity `v_up`.
    ///
    /// Returns `true` if the final position matches the target.
    #[inline]
    pub fn check_for_first_order(
        &mut self,
        v_up: f64,
        control_signs: ControlSigns,
        limits: ReachedLimits,
    ) -> bool {
        if self.t[3] < 0.0 {
            return false;
        }

        self.t_sum = [0.0, 0.0, 0.0, self.t[3], self.t[3], self.t[3], self.t[3]];
        if self.t_sum[6] > T_MAX {
            return false;
        }

        self.j = [0.0; 7];
        self.a = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.af];
        self.v = [
            0.0,
            0.0,
            0.0,
            if self.t[3] > 0.0 { v_up } else { 0.0 },
            0.0,
            0.0,
            0.0,
            self.vf,
        ];

        for i in 0..7 {
            self.p[i + 1] = self.p[i] + self.t[i] * (self.v[i] + self.t[i] * self.a[i] / 2.0);
        }

        self.control_signs = control_signs;
        self.limits = limits;
        self.direction = if v_up > 0.0 {
            Direction::UP
        } else {
            Direction::DOWN
        };

        (self.p[7] - self.pf).abs() < P_PRECISION
    }

    /// [`check_for_first_order`](Self::check_for_first_order) for a profile with
    /// a prescribed duration.
    #[inline]
    pub fn check_for_first_order_with_timing(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        _: f64,
        v_up: f64,
    ) -> bool {
        self.check_for_first_order(v_up, control_signs, limits)
    }

    /// Like [`check_for_first_order_with_timing`](Self::check_for_first_order_with_timing),
    /// but additionally requires `v_min <= v_up <= v_max`.
    #[inline]
    pub fn check_for_first_order_with_timing_full(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
        tf: f64,
        v_up: f64,
        v_max: f64,
        v_min: f64,
    ) -> bool {
        (v_min - V_EPS < v_up)
            && (v_up < v_max + V_EPS)
            && self.check_for_first_order_with_timing(control_signs, limits, tf, v_up)
    }

    /// Updates `ext` with the position at local time `t_ext` of a constant-jerk
    /// phase (start state `p`, `v`, `a`, jerk `j`, duration `t`, starting at
    /// `t_sum`), if `t_ext` lies strictly inside the phase. A positive
    /// acceleration there indicates a minimum, a negative one a maximum.
    pub fn check_position_extremum(
        t_ext: f64,
        t_sum: f64,
        t: f64,
        p: f64,
        v: f64,
        a: f64,
        j: f64,
        ext: &mut Bound,
    ) {
        if 0.0 < t_ext && t_ext < t {
            let (p_ext, _, a_ext) = integrate(t_ext, p, v, a, j);
            if a_ext > 0.0 && p_ext < ext.min {
                ext.min = p_ext;
                ext.t_min = t_sum + t_ext;
            } else if a_ext < 0.0 && p_ext > ext.max {
                ext.max = p_ext;
                ext.t_max = t_sum + t_ext;
            }
        }
    }

    fn check_step_for_position_extremum(
        t_sum: f64,
        t: f64,
        p: f64,
        v: f64,
        a: f64,
        j: f64,
        ext: &mut Bound,
    ) {
        if p < ext.min {
            ext.min = p;
            ext.t_min = t_sum;
        }
        if p > ext.max {
            ext.max = p;
            ext.t_max = t_sum;
        }

        if j != 0.0 {
            let d = a * a - 2.0 * j * v;
            if d.abs() < f64::EPSILON {
                Self::check_position_extremum(-a / j, t_sum, t, p, v, a, j, ext);
            } else if d > 0.0 {
                let d_sqrt = d.sqrt();
                Self::check_position_extremum((-a - d_sqrt) / j, t_sum, t, p, v, a, j, ext);
                Self::check_position_extremum((-a + d_sqrt) / j, t_sum, t, p, v, a, j, ext);
            }
        } else if a != 0.0 {
            // Constant acceleration: the velocity crosses zero at most once
            Self::check_position_extremum(-v / a, t_sum, t, p, v, a, j, ext);
        }
    }

    /// Returns the minimum and maximum positions of the profile, including the
    /// brake pre-trajectory, and the times at which they are reached.
    pub fn get_position_extrema(&self) -> Bound {
        let mut extrema = Bound {
            min: f64::INFINITY,
            max: f64::NEG_INFINITY,
            t_min: 0.0,
            t_max: 0.0,
        };

        if self.brake.duration > 0.0 && self.brake.t[0] > 0.0 {
            Self::check_step_for_position_extremum(
                0.0,
                self.brake.t[0],
                self.brake.p[0],
                self.brake.v[0],
                self.brake.a[0],
                self.brake.j[0],
                &mut extrema,
            );
            if self.brake.t[1] > 0.0 {
                Self::check_step_for_position_extremum(
                    self.brake.t[0],
                    self.brake.t[1],
                    self.brake.p[1],
                    self.brake.v[1],
                    self.brake.a[1],
                    self.brake.j[1],
                    &mut extrema,
                );
            }
        }

        let mut t_current_sum = 0.0;
        for i in 0..7 {
            if i > 0 {
                t_current_sum = self.t_sum[i - 1];
            }
            Self::check_step_for_position_extremum(
                t_current_sum + self.brake.duration,
                self.t[i],
                self.p[i],
                self.v[i],
                self.a[i],
                self.j[i],
                &mut extrema,
            );
        }

        if self.pf < extrema.min {
            extrema.min = self.pf;
            extrema.t_min = self.t_sum[6] + self.brake.duration;
        }
        if self.pf > extrema.max {
            extrema.max = self.pf;
            extrema.t_max = self.t_sum[6] + self.brake.duration;
        }

        extrema
    }

    /// Returns the first time at which the profile reaches the position `pt`,
    /// together with the velocity and acceleration at that time, as
    /// `(time, velocity, acceleration)`. `offset` is added to the returned time.
    /// The brake pre-trajectory is not searched.
    pub fn get_first_state_at_position(&self, pt: f64, offset: f64) -> Option<(f64, f64, f64)> {
        for i in 0..7 {
            if (self.p[i] - pt).abs() < f64::EPSILON {
                let time = offset + if i > 0 { self.t_sum[i - 1] } else { 0.0 };
                return Some((time, self.v[i], self.a[i]));
            }

            if self.t[i] == 0.0 {
                continue;
            }

            // The roots are unordered: the first crossing is the smallest one
            // inside the phase
            let first_crossing =
                roots::solve_cub(self.j[i] / 6.0, self.a[i] / 2.0, self.v[i], self.p[i] - pt)
                    .get_data()
                    .iter()
                    .copied()
                    .filter(|&t| 0.0 < t && t <= self.t[i])
                    .min_by(f64::total_cmp);
            if let Some(t) = first_crossing {
                let time = offset + t + if i > 0 { self.t_sum[i - 1] } else { 0.0 };
                let (_, vt, at) = integrate(t, self.p[i], self.v[i], self.a[i], self.j[i]);
                return Some((time, vt, at));
            }
        }

        if (self.pf - pt).abs() < 1e-9 {
            let time = offset + self.t_sum[6];
            return Some((time, self.vf, self.af));
        }

        None
    }
}

impl fmt::Display for Profile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.direction {
            Direction::UP => write!(f, "UP_")?,
            Direction::DOWN => write!(f, "DOWN_")?,
        }

        match self.limits {
            ReachedLimits::Acc0Acc1Vel => write!(f, "ACC0_ACC1_VEL")?,
            ReachedLimits::Vel => write!(f, "VEL")?,
            ReachedLimits::Acc0 => write!(f, "ACC0")?,
            ReachedLimits::Acc1 => write!(f, "ACC1")?,
            ReachedLimits::Acc0Acc1 => write!(f, "ACC0_ACC1")?,
            ReachedLimits::Acc0Vel => write!(f, "ACC0_VEL")?,
            ReachedLimits::Acc1Vel => write!(f, "ACC1_VEL")?,
            ReachedLimits::None => write!(f, "NONE")?,
        }

        match self.control_signs {
            ControlSigns::UDDU => write!(f, "_UDDU"),
            ControlSigns::UDUD => write!(f, "_UDUD"),
        }
    }
}
