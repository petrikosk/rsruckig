//! Emergency brake profile calculation for trajectory interruption
//!
//! This module provides functionality to create and handle emergency brake profiles
//! for safely stopping motion when a trajectory needs to be interrupted.

use crate::util::integrate;
use core::f64;

#[cfg(not(feature = "std"))]
use num_traits::Float;

const EPS: f64 = 2.2e-14;

#[inline]
fn v_at_t(v0: f64, a0: f64, j: f64, t: f64) -> f64 {
    v0 + t * (a0 + j * t / 2.0)
}

#[inline]
fn v_at_a_zero(v0: f64, a0: f64, j: f64) -> f64 {
    v0 + (a0 * a0) / (2.0 * j)
}

/// Exact distance traveled until a full stop (v = 0, a = 0) from state (v, a),
/// decelerating with peak acceleration bounded by `a_min` (<= 0) and jerk bounded
/// by `j_max` (> 0, finite). Returns 0.0 if the state never moves forward.
///
/// The stop profile is the time- and distance-optimal three-phase profile:
/// jerk -j_max until the peak deceleration, optional hold, jerk +j_max back to zero
/// so that v reaches 0 exactly when a reaches 0.
pub fn stop_distance_third_order(v: f64, a: f64, a_min: f64, j_max: f64) -> f64 {
    let a_stop = a_min.abs();
    // Peak deceleration (squared) of the triangular stop profile
    let s = j_max * v + 0.5 * a * a;
    if s <= 0.0 {
        // Velocity never becomes positive: no forward travel
        return 0.0;
    }

    if a < 0.0 && s <= a * a {
        // Already decelerating harder than the triangular peak: v hits 0 while
        // ramping a back to zero with jerk +j_max. Distance until v = 0.
        let disc = a * a - 2.0 * j_max * v;
        let t_stop = (-a - disc.max(0.0).sqrt()) / j_max;
        let t_stop = t_stop.max(0.0);
        return (v * t_stop + 0.5 * a * t_stop * t_stop + j_max * t_stop * t_stop * t_stop / 6.0)
            .max(0.0);
    }

    let a_pk_tri = s.sqrt();
    let (a_pk, t_hold) = if a_pk_tri <= a_stop {
        (a_pk_tri, 0.0)
    } else {
        let t_hold = (v + 0.5 * a * a / j_max) / a_stop - a_stop / j_max;
        (a_stop, t_hold.max(0.0))
    };

    let t_ramp_down = ((a + a_pk) / j_max).max(0.0);
    let t_ramp_up = a_pk / j_max;

    let (p1, v1, a1) = integrate(t_ramp_down, 0.0, v, a, -j_max);
    let (p2, v2, a2) = integrate(t_hold, p1, v1, a1, 0.0);
    let (p3, _, _) = integrate(t_ramp_up, p2, v2, a2, j_max);
    p3.max(0.0)
}

/// Distance traveled until a full stop from velocity `v` with acceleration bounded
/// by `a_min` (< 0), for the second-order case (infinite jerk).
pub fn stop_distance_second_order(v: f64, a_min: f64) -> f64 {
    if v <= 0.0 {
        return 0.0;
    }
    v * v / (2.0 * a_min.abs())
}

/// Maximum velocity (with a = 0) from which a full jerk-limited stop covers a
/// distance of at most `d`, with peak deceleration `|a_min|` and jerk `j_max`.
/// Inverse of `stop_distance_third_order` for a = 0.
pub fn velocity_cap_third_order(d: f64, a_min: f64, j_max: f64) -> f64 {
    if d <= 0.0 {
        return 0.0;
    }
    let a_stop = a_min.abs();
    // Triangular stop: d = v^(3/2) / sqrt(j_max)  =>  v = cbrt(d^2 * j_max)
    let v_tri = (d * d * j_max).cbrt();
    if v_tri * j_max <= a_stop * a_stop || a_stop.is_infinite() {
        // Peak deceleration sqrt(v * j_max) stays below a_stop
        return v_tri;
    }
    // Trapezoidal stop: d = v^2/(2A) + v*A/(2J)
    let half_a2_j = 0.5 * a_stop * a_stop / j_max;
    -half_a2_j + (half_a2_j * half_a2_j + 2.0 * a_stop * d).sqrt()
}

/// Maximum velocity from which a stop with acceleration `|a_min|` covers at most `d`
/// (second-order case, infinite jerk).
pub fn velocity_cap_second_order(d: f64, a_min: f64) -> f64 {
    if d <= 0.0 {
        return 0.0;
    }
    (2.0 * a_min.abs() * d).sqrt()
}

#[derive(Debug, Clone, Copy, Default)]
pub struct BrakeProfile {
    pub duration: f64,
    pub t: [f64; 2],
    pub j: [f64; 2],
    pub a: [f64; 2],
    pub v: [f64; 2],
    pub p: [f64; 2],
}

impl BrakeProfile {
    pub fn new() -> Self {
        BrakeProfile {
            duration: 0.0,
            t: [0.0; 2],
            j: [0.0; 2],
            a: [0.0; 2],
            v: [0.0; 2],
            p: [0.0; 2],
        }
    }

    fn acceleration_brake(
        &mut self,
        v0: f64,
        a0: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) {
        self.j[0] = -j_max;

        let t_to_a_max = (a0 - a_max) / j_max;
        let t_to_a_zero = a0 / j_max;

        let v_at_a_max = v_at_t(v0, a0, -j_max, t_to_a_max);
        let v_at_a_zero = v_at_t(v0, a0, -j_max, t_to_a_zero);

        if (v_at_a_zero > v_max && j_max > 0.0) || (v_at_a_zero < v_max && j_max < 0.0) {
            self.velocity_brake(v0, a0, v_max, v_min, a_max, a_min, j_max);
        } else if (v_at_a_max < v_min && j_max > 0.0) || (v_at_a_max > v_min && j_max < 0.0) {
            let t_to_v_min = -(v_at_a_max - v_min) / a_max;
            let t_to_v_max = -a_max / (2.0 * j_max) - (v_at_a_max - v_max) / a_max;

            self.t[0] = t_to_a_max + EPS;
            self.t[1] = t_to_v_min.min((t_to_v_max - EPS).max(0.0));
        } else {
            self.t[0] = t_to_a_max + EPS;
        }
    }

    fn velocity_brake(
        &mut self,
        v0: f64,
        a0: f64,
        v_max: f64,
        v_min: f64,
        _a_max: f64,
        a_min: f64,
        j_max: f64,
    ) {
        self.j[0] = -j_max;
        let t_to_a_min = (a0 - a_min) / j_max;
        let t_to_v_max = a0 / j_max + ((a0 * a0 + 2.0 * j_max * (v0 - v_max)).sqrt()) / j_max.abs();
        let t_to_v_min = a0 / j_max + ((a0 * a0 / 2.0 + j_max * (v0 - v_min)).sqrt()) / j_max.abs();
        let t_min_to_v_max = t_to_v_max.min(t_to_v_min);

        if t_to_a_min < t_min_to_v_max {
            let v_at_a_min = v_at_t(v0, a0, -j_max, t_to_a_min);
            let t_to_v_max_with_constant = -(v_at_a_min - v_max) / a_min;
            let t_to_v_min_with_constant = a_min / (2.0 * j_max) - (v_at_a_min - v_min) / a_min;

            self.t[0] = (t_to_a_min - EPS).max(0.0);
            self.t[1] = t_to_v_max_with_constant
                .min(t_to_v_min_with_constant)
                .max(0.0);
        } else {
            self.t[0] = (t_min_to_v_max - EPS).max(0.0);
        }
    }

    pub fn get_position_brake_trajectory(
        &mut self,
        v0: f64,
        a0: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) {
        self.t[0] = 0.0;
        self.t[1] = 0.0;
        self.j[0] = 0.0;
        self.j[1] = 0.0;

        if j_max == 0.0 || a_max == 0.0 || a_min == 0.0 {
            return; // Ignore braking for zero-limits
        }

        if a0 > a_max {
            self.acceleration_brake(v0, a0, v_max, v_min, a_max, a_min, j_max);
        } else if a0 < a_min {
            self.acceleration_brake(v0, a0, v_min, v_max, a_min, a_max, -j_max);
        } else if (v0 > v_max && v_at_a_zero(v0, a0, -j_max) > v_min)
            || (a0 > 0.0 && v_at_a_zero(v0, a0, j_max) > v_max)
        {
            self.velocity_brake(v0, a0, v_max, v_min, a_max, a_min, j_max);
        } else if (v0 < v_min && v_at_a_zero(v0, a0, j_max) < v_max)
            || (a0 < 0.0 && v_at_a_zero(v0, a0, -j_max) < v_min)
        {
            self.velocity_brake(v0, a0, v_min, v_max, a_min, a_max, -j_max);
        }
    }

    /// Position-limit-aware variant of `get_position_brake_trajectory`: additionally
    /// brakes when the current velocity exceeds the largest velocity from which a
    /// full stop still fits inside the position limits.
    #[allow(clippy::too_many_arguments)]
    pub fn get_position_brake_trajectory_with_position_limits(
        &mut self,
        p0: f64,
        v0: f64,
        a0: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
        min_position: f64,
        max_position: f64,
    ) {
        let v_cap_up = velocity_cap_third_order(max_position - p0, a_min, j_max);
        let v_cap_down = velocity_cap_third_order(p0 - min_position, a_max, j_max);
        self.get_position_brake_trajectory(
            v0,
            a0,
            v_max.min(v_cap_up),
            v_min.max(-v_cap_down),
            a_max,
            a_min,
            j_max,
        );
    }

    /// Position-limit-aware variant of `get_second_order_position_brake_trajectory`.
    pub fn get_second_order_position_brake_trajectory_with_position_limits(
        &mut self,
        p0: f64,
        v0: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        min_position: f64,
        max_position: f64,
    ) {
        let v_cap_up = velocity_cap_second_order(max_position - p0, a_min);
        let v_cap_down = velocity_cap_second_order(p0 - min_position, a_max);
        self.get_second_order_position_brake_trajectory(
            v0,
            v_max.min(v_cap_up),
            v_min.max(-v_cap_down),
            a_max,
            a_min,
        );
    }

    pub fn get_second_order_position_brake_trajectory(
        &mut self,
        v0: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
    ) {
        self.t[0] = 0.0;
        self.t[1] = 0.0;
        self.j[0] = 0.0;
        self.j[1] = 0.0;
        self.a[0] = 0.0;
        self.a[1] = 0.0;

        if a_max == 0.0 || a_min == 0.0 {
            return; // Ignore braking for zero-limits
        }

        if v0 > v_max {
            self.a[0] = a_min;
            self.t[0] = (v_max - v0) / a_min + f64::EPSILON;
        } else if v0 < v_min {
            self.a[0] = a_max;
            self.t[0] = (v_min - v0) / a_max + f64::EPSILON;
        }
    }

    pub fn get_velocity_brake_trajectory(&mut self, a0: f64, a_max: f64, a_min: f64, j_max: f64) {
        self.t[0] = 0.0;
        self.t[1] = 0.0;
        self.j[0] = 0.0;
        self.j[1] = 0.0;

        if j_max == 0.0 {
            return; // Ignore braking for zero-limits
        }

        if a0 > a_max {
            self.j[0] = -j_max;
            self.t[0] = (a0 - a_max) / j_max + f64::EPSILON;
        } else if a0 < a_min {
            self.j[0] = j_max;
            self.t[0] = -(a0 - a_min) / j_max + f64::EPSILON;
        }
    }

    pub fn get_second_order_velocity_brake_trajectory(&mut self) {
        self.t[0] = 0.0;
        self.t[1] = 0.0;
        self.j[0] = 0.0;
        self.j[1] = 0.0;
    }

    pub fn finalize(&mut self, ps: &mut f64, vs: &mut f64, as_: &mut f64) {
        if self.t[0] <= 0.0 && self.t[1] <= 0.0 {
            self.duration = 0.0;
            return;
        }

        self.duration = self.t[0];
        self.p[0] = *ps;
        self.v[0] = *vs;
        self.a[0] = *as_;
        let result = integrate(self.t[0], *ps, *vs, *as_, self.j[0]);
        *ps = result.0;
        *vs = result.1;
        *as_ = result.2;

        if self.t[1] > 0.0 {
            self.duration += self.t[1];
            self.p[1] = *ps;
            self.v[1] = *vs;
            self.a[1] = *as_;
            let result = integrate(self.t[1], *ps, *vs, *as_, self.j[1]);
            *ps = result.0;
            *vs = result.1;
            *as_ = result.2;
        }
    }

    pub fn finalize_second_order(&mut self, ps: &mut f64, vs: &mut f64, as_: &mut f64) {
        if self.t[0] <= 0.0 {
            self.duration = 0.0;
            return;
        }

        self.duration = self.t[0];
        self.p[0] = *ps;
        self.v[0] = *vs;
        let result = integrate(self.t[0], *ps, *vs, self.a[0], 0.0);
        *ps = result.0;
        *vs = result.1;
        *as_ = result.2;
    }
}
