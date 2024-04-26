use crate::brake::BrakeProfile;
use crate::roots;
use crate::util::integrate;
use std::fmt;

static V_EPS: f64 = 1e-12;
static A_EPS: f64 = 1e-12;
static J_EPS: f64 = 1e-12;

static P_PRECISION: f64 = 1e-8;
static V_PRECISION: f64 = 1e-8;
static A_PRECISION: f64 = 1e-10;

static T_MAX: f64 = 1e12;

#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub enum ReachedLimits {
    Acc0Acc1Vel,
    Vel,
    Acc0,
    Acc1,
    Acc0Acc1,
    Acc0Vel,
    Acc1Vel,
    #[default]
    None,
}

#[derive(Debug, Default, PartialEq, Clone)]
pub enum Direction {
    #[default]
    UP,
    DOWN,
}

#[derive(Debug, Default, PartialEq, Clone)]
pub enum ControlSigns {
    #[default]
    UDDU,
    UDUD,
}

#[derive(Debug, Clone, Default)]
pub struct Bound {
    // The extreme position
    pub min: f64,
    pub max: f64,
    // Time when the positions are reached
    pub t_min: f64,
    pub t_max: f64,
}

/// The state profile for position, velocity, acceleration and jerk for a single DoF
#[derive(Debug, Clone, Default)]
pub struct Profile {
    pub t: [f64; 7],
    pub t_sum: [f64; 7],
    pub j: [f64; 7],
    pub a: [f64; 8],
    pub v: [f64; 8],
    pub p: [f64; 8],

    /// Brake sub-profiles
    pub brake: BrakeProfile,
    pub accel: BrakeProfile,

    /// Target (final) kinematic state
    pub pf: f64,
    pub vf: f64,
    pub af: f64,

    pub limits: ReachedLimits,
    pub direction: Direction,
    pub control_signs: ControlSigns,
}

impl Profile {
    pub fn check_for_velocity(
        &mut self,
        control_signs: ControlSigns,
        limits: ReachedLimits,
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

        if (limits == ReachedLimits::Acc0) && self.t[1] < std::f64::EPSILON {
            return false;
        }

        if *self.t_sum.last().unwrap() > T_MAX {
            // Use T_PRECISION for numerical reasons.
            return false;
        }

        if control_signs == ControlSigns::UDDU {
            self.j = [
                if self.t[0] > 0.0 { jf } else { 0.0 },
                0.0,
                if self.t[2] > 0.0 { -jf } else { 0.0 },
                0.0,
                if self.t[4] > 0.0 { -jf } else { 0.0 },
                0.0,
                if self.t[6] > 0.0 { jf } else { 0.0 },
            ];
        } else {
            self.j = [
                if self.t[0] > 0.0 { jf } else { 0.0 },
                0.0,
                if self.t[2] > 0.0 { -jf } else { 0.0 },
                0.0,
                if self.t[4] > 0.0 { jf } else { 0.0 },
                0.0,
                if self.t[6] > 0.0 { jf } else { 0.0 },
            ];
        }

        for i in 0..7 {
            self.a[i + 1] = self.a[i] + self.t[i] * self.j[i];
            self.v[i + 1] = self.v[i] + self.t[i] * (self.a[i] + self.t[i] * self.j[i] / 2.0);
            self.p[i + 1] = self.p[i]
                + self.t[i]
                    * (self.v[i] + self.t[i] * (self.a[i] / 2.0 + self.t[i] * self.j[i] / 6.0));
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
        (self.v.last().unwrap() - self.vf).abs() < V_PRECISION
            && (self.a.last().unwrap() - self.af).abs() < A_PRECISION
            && self.a[1] >= a_low_lim
            && self.a[3] >= a_low_lim
            && self.a[5] >= a_low_lim
            && self.a[1] <= a_upp_lim
            && self.a[3] <= a_upp_lim
            && self.a[5] <= a_upp_lim
    }

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

    // For second-order velocity interface
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
        //self.t_sum.fill(self.t[1]);
        if *self.t_sum.last().unwrap_or(&0.0) > T_MAX {
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

        (self.v.last().unwrap_or(&0.0) - self.vf).abs() < V_PRECISION
    }

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

    #[inline]
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

        if matches!(limits, ReachedLimits::Acc0 | ReachedLimits::Acc0Acc1)
            && self.t[1] < f64::EPSILON
        {
            return false;
        }

        if matches!(limits, ReachedLimits::Acc1 | ReachedLimits::Acc0Acc1)
            && self.t[5] < f64::EPSILON
        {
            return false;
        }

        if self.t_sum.last().unwrap_or(&0.0) > &T_MAX {
            return false;
        }

        self.j = if control_signs == ControlSigns::UDDU {
            [
                if self.t[0] > 0.0 { jf } else { 0.0 },
                0.0,
                if self.t[2] > 0.0 { -jf } else { 0.0 },
                0.0,
                if self.t[4] > 0.0 { -jf } else { 0.0 },
                0.0,
                if self.t[6] > 0.0 { jf } else { 0.0 },
            ]
        } else {
            [
                if self.t[0] > 0.0 { jf } else { 0.0 },
                0.0,
                if self.t[2] > 0.0 { -jf } else { 0.0 },
                0.0,
                if self.t[4] > 0.0 { jf } else { 0.0 },
                0.0,
                if self.t[6] > 0.0 { -jf } else { 0.0 },
            ]
        };

        self.direction = if v_max > 0.0 {
            Direction::UP
        } else {
            Direction::DOWN
        };

        let v_upp_lim = if self.direction == Direction::UP {
            v_max
        } else {
            v_min
        } + V_EPS;
        let v_low_lim = if self.direction == Direction::UP {
            v_min
        } else {
            v_max
        } - V_EPS;

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

            if set_limits {
                match limits {
                    ReachedLimits::Acc1 => {
                        if i == 2 {
                            self.a[3] = a_min;
                        }
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

        let a_upp_lim = if self.direction == Direction::UP {
            a_max
        } else {
            a_min
        } + A_EPS;
        let a_low_lim = if self.direction == Direction::UP {
            a_min
        } else {
            a_max
        } - A_EPS;

        (self.p.last().unwrap_or(&0.0) - self.pf).abs() < P_PRECISION
            && (self.v.last().unwrap_or(&0.0) - self.vf).abs() < V_PRECISION
            && (self.a.last().unwrap_or(&0.0) - self.af).abs() < A_PRECISION
            && [self.a[1], self.a[3], self.a[5]]
                .iter()
                .all(|&x| x >= a_low_lim && x <= a_upp_lim)
            && [self.v[3], self.v[4], self.v[5], self.v[6]]
                .iter()
                .all(|&x| x <= v_upp_lim && x >= v_low_lim)
    }

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

    #[inline]
    pub fn set_boundary_from_profile(&mut self, profile: &Profile) {
        self.a[0] = profile.a[0];
        self.v[0] = profile.v[0];
        self.p[0] = profile.p[0];
        self.af = profile.af;
        self.vf = profile.vf;
        self.pf = profile.pf;
        self.brake = profile.brake.clone();
        self.accel = profile.accel.clone();
    }

    #[inline]
    pub fn set_boundary(
        &mut self,
        p0_new: &f64,
        v0_new: &f64,
        a0_new: &f64,
        pf_new: &f64,
        vf_new: &f64,
        af_new: &f64,
    ) {
        self.a[0] = *a0_new;
        self.v[0] = *v0_new;
        self.p[0] = *p0_new;
        self.af = *af_new;
        self.vf = *vf_new;
        self.pf = *pf_new;
    }

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

        if *self.t_sum.last().unwrap_or(&0.0) > T_MAX {
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

        (self.p.last().unwrap_or(&0.0) - self.pf).abs() < P_PRECISION
            && (self.v.last().unwrap_or(&0.0) - self.vf).abs() < P_PRECISION
            && self.v[2..=7].iter().all(|&v| v <= v_upp_lim)
            && self.v[2..=7].iter().all(|&v| v >= v_low_lim)
    }

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

    // For first-order position interface
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
        if *self.t_sum.last().unwrap_or(&0.0) > T_MAX {
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

        (self.p.last().unwrap_or(&0.0) - self.pf).abs() < P_PRECISION
    }

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
        }
    }

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
            extrema.t_min = self.t_sum.last().unwrap_or(&0.0) + self.brake.duration;
        }
        if self.pf > extrema.max {
            extrema.max = self.pf;
            extrema.t_max = self.t_sum.last().unwrap_or(&0.0) + self.brake.duration;
        }

        extrema
    }

    pub fn get_first_state_at_position(&self, pt: f64, offset: f64) -> Option<(f64, f64, f64)> {
        for i in 0..7 {
            if (self.p[i] - pt).abs() < f64::EPSILON {
                let time = offset + if i > 0 { self.t_sum[i - 1] } else { 0.0 };
                return Some((time, self.v[i], self.a[i]));
            }

            if self.t[i] == 0.0 {
                continue;
            }

            for &t in roots::solve_cub(self.j[i] / 6.0, self.a[i] / 2.0, self.v[i], self.p[i] - pt)
                .get_data()
            {
                let _t = t;

                if 0.0 < _t && _t <= self.t[i] {
                    let time = offset + _t + if i > 0 { self.t_sum[i - 1] } else { 0.0 };
                    let (_, vt, at) = integrate(_t, self.p[i], self.v[i], self.a[i], self.j[i]);
                    return Some((time, vt, at));
                }
            }
        }

        if (self.pf - pt).abs() < 1e-9 {
            let time = offset + self.t_sum.last().unwrap_or(&0.0);
            return Some((time, self.vf, self.af));
        }

        None
    }
}

impl fmt::Display for Profile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut result = String::new();

        match self.direction {
            Direction::UP => result.push_str("UP_"),
            Direction::DOWN => result.push_str("DOWN_"),
        }

        match self.limits {
            ReachedLimits::Acc0Acc1Vel => result.push_str("ACC0_ACC1_VEL"),
            ReachedLimits::Vel => result.push_str("VEL"),
            ReachedLimits::Acc0 => result.push_str("ACC0"),
            ReachedLimits::Acc1 => result.push_str("ACC1"),
            ReachedLimits::Acc0Acc1 => result.push_str("ACC0_ACC1"),
            ReachedLimits::Acc0Vel => result.push_str("ACC0_VEL"),
            ReachedLimits::Acc1Vel => result.push_str("ACC1_VEL"),
            ReachedLimits::None => result.push_str("NONE"),
        }

        match self.control_signs {
            ControlSigns::UDDU => result.push_str("_UDDU"),
            ControlSigns::UDUD => result.push_str("_UDUD"),
        }

        write!(f, "{}", result)
    }
}
