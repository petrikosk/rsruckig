//! Mathematical equations for Step 2 in third-order position interface: Time synchronization
use arrayvec::ArrayVec;

use crate::{
    profile::{ControlSigns, Profile, ReachedLimits},
    roots::*,
};

pub struct PositionThirdOrderStep2 {
    v0: f64,
    a0: f64,
    tf: f64,
    vf: f64,
    af: f64,
    _v_max: f64,
    _v_min: f64,
    _a_max: f64,
    _a_min: f64,
    _j_max: f64,

    // Pre-calculated expressions
    pd: f64,
    tf_tf: f64,
    tf_p3: f64,
    tf_p4: f64,
    vd: f64,
    vd_vd: f64,
    vf_vf: f64,
    ad: f64,
    ad_ad: f64,
    a0_a0: f64,
    af_af: f64,
    a0_p3: f64,
    a0_p4: f64,
    a0_p5: f64,
    a0_p6: f64,
    af_p3: f64,
    af_p4: f64,
    af_p5: f64,
    af_p6: f64,
    j_max_j_max: f64,
    g1: f64,
    g2: f64,
    minimize_jerk: bool,
}

impl PositionThirdOrderStep2 {
    pub fn new(
        tf: f64,
        p0: f64,
        v0: f64,
        a0: f64,
        pf: f64,
        vf: f64,
        af: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) -> Self {
        let pd = pf - p0;
        let tf_tf = tf * tf;
        let tf_p3 = tf_tf * tf;
        let tf_p4 = tf_tf * tf_tf;

        let vd = vf - v0;
        let vd_vd = vd * vd;
        let vf_vf = vf * vf;

        let ad = af - a0;
        let ad_ad = ad * ad;
        let a0_a0 = a0 * a0;
        let af_af = af * af;

        let a0_p3 = a0 * a0_a0;
        let a0_p4 = a0_a0 * a0_a0;
        let a0_p5 = a0_p3 * a0_a0;
        let a0_p6 = a0_p4 * a0_a0;
        let af_p3 = af * af_af;
        let af_p4 = af_af * af_af;
        let af_p5 = af_p3 * af_af;
        let af_p6 = af_p4 * af_af;

        // max values needs to be invariant to plus minus sign change
        let j_max_j_max = j_max * j_max;

        let g1 = -pd + tf * v0;
        let g2 = -2.0 * pd + tf * (v0 + vf);
        Self {
            v0,
            a0,
            tf,
            vf,
            af,
            _v_max: v_max,
            _v_min: v_min,
            _a_max: a_max,
            _a_min: a_min,
            _j_max: j_max,
            pd,
            tf_tf,
            tf_p3,
            tf_p4,
            vd,
            vd_vd,
            vf_vf,
            ad,
            ad_ad,
            a0_a0,
            af_af,
            a0_p3,
            a0_p4,
            a0_p5,
            a0_p6,
            af_p3,
            af_p4,
            af_p5,
            af_p6,
            j_max_j_max,
            g1,
            g2,
            minimize_jerk: false,
        }
    }

    fn time_acc0_acc1_vel(
        &mut self,
        profile: &mut Profile,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) -> bool {
        // Profile UDDU, Solution 1
        if (2.0 * (a_max - a_min) + self.ad) / j_max < self.tf {
            let h1 = f64::sqrt(
                (self.a0_p4 + self.af_p4
                    - 4.0 * self.a0_p3 * (2.0 * a_max + a_min) / 3.0
                    - 4.0 * self.af_p3 * (a_max + 2.0 * a_min) / 3.0
                    + 2.0 * (self.a0_a0 - self.af_af) * a_max * a_max
                    + (4.0 * self.a0 * a_max - 2.0 * self.a0_a0)
                    * (self.af_af - 2.0 * self.af * a_min
                    + (a_min - a_max) * a_min
                    + 2.0 * j_max * (a_min * self.tf - self.vd))
                    + 2.0
                    * self.af_af
                    * (a_min * a_min + 2.0 * j_max * (a_max * self.tf - self.vd))
                    + 4.0
                    * j_max
                    * (2.0 * a_min * (self.af * self.vd + j_max * self.g1)
                    + (a_max * a_max - a_min * a_min) * self.vd
                    + j_max * self.vd_vd)
                    + 8.0 * a_max * self.j_max_j_max * (self.pd - self.tf * self.vf))
                    / (a_max * a_min)
                    + 4.0 * self.af_af
                    + 2.0 * self.a0_a0
                    + (4.0 * self.af + a_max - a_min) * (a_max - a_min)
                    + 4.0 * j_max * (a_min - a_max + j_max * self.tf - 2.0 * self.af) * self.tf,
            ) * f64::abs(j_max)
                / j_max;
            profile.t[0] = (-self.a0 + a_max) / j_max;
            profile.t[1] = (-(self.af_af - self.a0_a0
                + 2.0 * a_max * a_max
                + a_min * (a_min - 2.0 * self.ad - 3.0 * a_max)
                + 2.0 * j_max * (a_min * self.tf - self.vd))
                + a_min * h1)
                / (2.0 * (a_max - a_min) * j_max);
            profile.t[2] = a_max / j_max;
            profile.t[3] = (a_min - a_max + h1) / (2.0 * j_max);
            profile.t[4] = -a_min / j_max;
            profile.t[5] = self.tf
                - (profile.t[0]
                + profile.t[1]
                + profile.t[2]
                + profile.t[3]
                + 2.0 * profile.t[4]
                + self.af / j_max);
            profile.t[6] = profile.t[4] + self.af / j_max;

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::Acc0Acc1Vel,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        // Profile UDUD
        if (-self.a0 + 4.0 * a_max - self.af) / j_max < self.tf {
            profile.t[0] = (-self.a0 + a_max) / j_max;
            profile.t[1] = (3.0 * (self.a0_p4 + self.af_p4)
                - 4.0 * (self.a0_p3 + self.af_p3) * a_max
                - 4.0 * self.af_p3 * a_max
                + 24.0 * (self.a0 + self.af) * a_max * a_max * a_max
                - 6.0 * (self.af_af + self.a0_a0) * (a_max * a_max - 2.0 * j_max * self.vd)
                + 6.0
                * self.a0_a0
                * (self.af_af - 2.0 * self.af * a_max - 2.0 * a_max * j_max * self.tf)
                - 12.0
                * a_max
                * a_max
                * (2.0 * a_max * a_max - 2.0 * a_max * j_max * self.tf + j_max * self.vd)
                - 24.0 * self.af * a_max * j_max * self.vd
                + 12.0 * self.j_max_j_max * (2.0 * a_max * self.g1 + self.vd_vd))
                / (12.0
                * a_max
                * j_max
                * (self.a0_a0 + self.af_af - 2.0 * (self.a0 + self.af) * a_max
                + 2.0 * (a_max * a_max - a_max * j_max * self.tf + j_max * self.vd)));
            profile.t[2] = a_max / j_max;
            profile.t[3] = (-self.a0_a0 - self.af_af
                + 2.0 * a_max * (self.a0 + self.af - 2.0 * a_max)
                - 2.0 * j_max * self.vd)
                / (2.0 * a_max * j_max)
                + self.tf;
            profile.t[4] = profile.t[2];
            profile.t[5] = self.tf
                - (profile.t[0] + profile.t[1] + profile.t[2] + profile.t[3] + 2.0 * profile.t[4]
                - self.af / j_max);
            profile.t[6] = profile.t[4] - self.af / j_max;

            if profile.check_with_timing(
                ControlSigns::UDUD,
                ReachedLimits::Acc0Acc1Vel,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }
        false
    }

    fn time_acc1_vel(
        &mut self,
        profile: &mut Profile,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) -> bool {
        // Profile UDDU
        {
            let ph1 = self.a0_a0 + self.af_af
                - a_min * (self.a0 + 2.0 * self.af - a_min)
                - 2.0 * j_max * (self.vd - a_min * self.tf);
            let ph2 = 2.0 * a_min * (j_max * self.g1 + self.af * self.vd) - a_min * a_min * self.vd
                + j_max * self.vd_vd;
            let ph3 = self.af_af + a_min * (a_min - 2.0 * self.af)
                - 2.0 * j_max * (self.vd - a_min * self.tf);
            let mut polynom = [0.0; 4];

            polynom[0] = (2.0 * (2.0 * self.a0 - a_min)) / j_max;
            polynom[1] = (4.0 * self.a0_a0 + ph1 - 3.0 * self.a0 * a_min) / self.j_max_j_max;
            polynom[2] = (2.0 * self.a0 * ph1) / (self.j_max_j_max * j_max);
            polynom[3] = (3.0 * (self.a0_p4 + self.af_p4)
                - 4.0 * (self.a0_p3 + 2.0 * self.af_p3) * a_min
                + 6.0 * self.af_af * (a_min * a_min - 2.0 * j_max * self.vd)
                + 12.0 * j_max * ph2
                + 6.0 * self.a0_a0 * ph3)
                / (12.0 * self.j_max_j_max * self.j_max_j_max);

            let t_min = -self.a0 / j_max;
            let t_max = f64::min(
                (self.tf + 2.0 * a_min / j_max - (self.a0 + self.af) / j_max) / 2.0,
                (a_max - self.a0) / j_max,
            );

            let roots = solve_quart_monic_arr(&polynom);
            for mut t in &mut roots.into_iter() {
                if t < t_min || t > t_max {
                    continue;
                }

                // Single Newton step (regarding pd)
                if f64::abs(self.a0 + j_max * t) > 16.0 * f64::EPSILON {
                    let h0 = j_max * t * t;
                    let orig = -self.pd
                        + (3.0 * (self.a0_p4 + self.af_p4)
                        - 8.0 * self.af_p3 * a_min
                        - 4.0 * self.a0_p3 * a_min
                        + 6.0 * self.af_af * (a_min * a_min + 2.0 * j_max * (h0 - self.vd))
                        + 6.0
                        * self.a0_a0
                        * (self.af_af - 2.0 * self.af * a_min
                        + a_min * a_min
                        + 2.0 * a_min * j_max * (-2.0 * t + self.tf)
                        + 2.0 * j_max * (5.0 * h0 - self.vd))
                        + 24.0
                        * self.a0
                        * j_max
                        * t
                        * (self.a0_a0 + self.af_af - 2.0 * self.af * a_min
                        + a_min * a_min
                        + 2.0 * j_max * (a_min * (-t + self.tf) + h0 - self.vd))
                        - 24.0 * self.af * a_min * j_max * (h0 - self.vd)
                        + 12.0
                        * j_max
                        * (a_min * a_min * (h0 - self.vd)
                        + j_max * (h0 - self.vd) * (h0 - self.vd)))
                        / (24.0 * a_min * self.j_max_j_max)
                        + h0 * (self.tf - t)
                        + self.tf * self.v0;
                    let deriv = (self.a0 + j_max * t)
                        * ((self.a0_a0 + self.af_af) / (a_min * j_max)
                        + (a_min - self.a0 - 2.0 * self.af) / j_max
                        + (4.0 * self.a0 * t + 2.0 * h0 - 2.0 * self.vd) / a_min
                        + 2.0 * self.tf
                        - 3.0 * t);

                    t -= orig / deriv;
                }

                let h1 = -((self.a0_a0 + self.af_af) / 2.0
                    + j_max * (-self.vd + 2.0 * self.a0 * t + j_max * t * t))
                    / a_min;

                profile.t[0] = t;
                profile.t[1] = 0.0;
                profile.t[2] = self.a0 / j_max + t;
                profile.t[3] = self.tf - (h1 - a_min + self.a0 + self.af) / j_max - 2.0 * t;
                profile.t[4] = -a_min / j_max;
                profile.t[5] = (h1 + a_min) / j_max;
                profile.t[6] = profile.t[4] + self.af / j_max;

                if profile.check_with_timing(
                    ControlSigns::UDDU,
                    ReachedLimits::Acc1Vel,
                    j_max,
                    v_max,
                    v_min,
                    a_max,
                    a_min,
                ) {
                    return true;
                }
            }
        }

        // Profile UDUD
        {
            let ph1 = self.a0_a0 - self.af_af + (2.0 * self.af - self.a0) * a_max
                - a_max * a_max
                - 2.0 * j_max * (self.vd - a_max * self.tf);
            let ph2 = a_max * a_max + 2.0 * j_max * self.vd;
            let ph3 = self.af_af + ph2 - 2.0 * a_max * (self.af + j_max * self.tf);
            let ph4 = 2.0 * a_max * j_max * self.g1 + a_max * a_max * self.vd + j_max * self.vd_vd;

            let mut polynom = [0.0; 4];
            polynom[0] = (4.0 * self.a0 - 2.0 * a_max) / j_max;
            polynom[1] = (4.0 * self.a0_a0 - 3.0 * self.a0 * a_max + ph1) / self.j_max_j_max;
            polynom[2] = (2.0 * self.a0 * ph1) / (self.j_max_j_max * j_max);
            polynom[3] = (3.0 * (self.a0_p4 + self.af_p4)
                - 4.0 * (self.a0_p3 + 2.0 * self.af_p3) * a_max
                - 24.0 * self.af * a_max * j_max * self.vd
                + 12.0 * j_max * ph4
                - 6.0 * self.a0_a0 * ph3
                + 6.0 * self.af_af * ph2)
                / (12.0 * self.j_max_j_max * self.j_max_j_max);

            let t_min = -self.a0 / j_max;
            let t_max = f64::min(
                (self.tf + self.ad / j_max - 2.0 * a_max / j_max) / 2.0,
                (a_max - self.a0) / j_max,
            );

            let roots = solve_quart_monic_arr(&polynom);
            for t in &mut roots.into_iter() {
                if t > t_max || t < t_min {
                    continue;
                }

                let h1 = ((self.a0_a0 - self.af_af) / 2.0 + self.j_max_j_max * t * t
                    - j_max * (self.vd - 2.0 * self.a0 * t))
                    / a_max;

                profile.t[0] = t;
                profile.t[1] = 0.0;
                profile.t[2] = t + self.a0 / j_max;
                profile.t[3] = self.tf + (h1 + self.ad - a_max) / j_max - 2.0 * t;
                profile.t[4] = a_max / j_max;
                profile.t[5] = -(h1 + a_max) / j_max;
                profile.t[6] = profile.t[4] - self.af / j_max;

                if profile.check_with_timing(
                    ControlSigns::UDUD,
                    ReachedLimits::Acc1Vel,
                    j_max,
                    v_max,
                    v_min,
                    a_max,
                    a_min,
                ) {
                    return true;
                }
            }
        }

        false
    }

    fn time_acc0_vel(
        &mut self,
        profile: &mut Profile,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) -> bool {
        if self.tf < f64::max((-self.a0 + a_max) / j_max, 0.0) + f64::max(a_max / j_max, 0.0) {
            return false;
        }

        let ph1 = 12.0
            * j_max
            * (-a_max * a_max * self.vd - j_max * self.vd_vd
            + 2.0 * a_max * j_max * (-self.pd + self.tf * self.vf));

        // Profile UDDU
        {
            let mut polynom = [0.0; 4];
            polynom[0] = (2.0 * a_max) / j_max;
            polynom[1] = (self.a0_a0 - self.af_af
                + 2.0 * self.ad * a_max
                + a_max * a_max
                + 2.0 * j_max * (self.vd - a_max * self.tf))
                / self.j_max_j_max;
            polynom[2] = 0.0;
            polynom[3] = -(-3.0 * (self.a0_p4 + self.af_p4)
                + 4.0 * (self.af_p3 + 2.0 * self.a0_p3) * a_max
                - 12.0 * self.a0 * a_max * (self.af_af - 2.0 * j_max * self.vd)
                + 6.0 * self.a0_a0 * (self.af_af - a_max * a_max - 2.0 * j_max * self.vd)
                + 6.0
                * self.af_af
                * (a_max * a_max - 2.0 * a_max * j_max * self.tf + 2.0 * j_max * self.vd)
                + ph1)
                / (12.0 * self.j_max_j_max * self.j_max_j_max);

            let t_min = -self.af / j_max;
            let t_max = f64::min(self.tf - (2.0 * a_max - self.a0) / j_max, -a_min / j_max);
            let roots = solve_quart_monic_arr(&polynom);
            for mut t in &mut roots.into_iter() {
                if t < t_min || t > t_max {
                    continue;
                }

                // Single Newton step (regarding self.pd)
                if t > f64::EPSILON {
                    let h1 = j_max * t * t + self.vd;
                    let orig = (-3.0 * (self.a0_p4 + self.af_p4)
                        + 4.0 * (self.af_p3 + 2.0 * self.a0_p3) * a_max
                        - 24.0 * self.af * a_max * self.j_max_j_max * t * t
                        - 12.0 * self.a0 * a_max * (self.af_af - 2.0 * j_max * h1)
                        + 6.0 * self.a0_a0 * (self.af_af - a_max * a_max - 2.0 * j_max * h1)
                        + 6.0
                        * self.af_af
                        * (a_max * a_max - 2.0 * a_max * j_max * self.tf + 2.0 * j_max * h1)
                        - 12.0
                        * j_max
                        * (a_max * a_max * h1
                        + j_max * h1 * h1
                        + 2.0
                        * a_max
                        * j_max
                        * (self.pd + j_max * t * t * (t - self.tf)
                        - self.tf * self.vf)))
                        / (24.0 * a_max * self.j_max_j_max);
                    let deriv = -t
                        * (self.a0_a0 - self.af_af
                        + 2.0 * a_max * (self.ad - j_max * self.tf)
                        + a_max * a_max
                        + 3.0 * a_max * j_max * t
                        + 2.0 * j_max * h1)
                        / a_max;

                    t -= orig / deriv;
                }

                let h1 =
                    ((self.a0_a0 - self.af_af) / 2.0 + j_max * (j_max * t * t + self.vd)) / a_max;

                profile.t[0] = (-self.a0 + a_max) / j_max;
                profile.t[1] = (h1 - a_max) / j_max;
                profile.t[2] = a_max / j_max;
                profile.t[3] = self.tf - (h1 + self.ad + a_max) / j_max - 2.0 * t;
                profile.t[4] = t;
                profile.t[5] = 0.0;
                profile.t[6] = self.af / j_max + t;

                if profile.check_with_timing(
                    ControlSigns::UDDU,
                    ReachedLimits::Acc0Vel,
                    j_max,
                    v_max,
                    v_min,
                    a_max,
                    a_min,
                ) {
                    return true;
                }
            }
        }

        // Profile UDUD
        {
            let mut polynom = [0.0; 4];
            polynom[0] = (-2.0 * a_max) / j_max;
            polynom[1] = -(self.a0_a0 + self.af_af - 2.0 * (self.a0 + self.af) * a_max
                + a_max * a_max
                + 2.0 * j_max * (self.vd - a_max * self.tf))
                / self.j_max_j_max;
            polynom[2] = 0.0;
            polynom[3] = (3.0 * (self.a0_p4 + self.af_p4)
                - 4.0 * (self.af_p3 + 2.0 * self.a0_p3) * a_max
                + 6.0 * self.a0_a0 * (self.af_af + a_max * a_max + 2.0 * j_max * self.vd)
                - 12.0 * self.a0 * a_max * (self.af_af + 2.0 * j_max * self.vd)
                + 6.0
                * self.af_af
                * (a_max * a_max - 2.0 * a_max * j_max * self.tf + 2.0 * j_max * self.vd)
                - ph1)
                / (12.0 * self.j_max_j_max * self.j_max_j_max);

            let t_min = self.af / j_max;
            let t_max = f64::min(self.tf - a_max / j_max, a_max / j_max);

            let roots = solve_quart_monic_arr(&polynom);
            for mut t in &mut roots.into_iter() {
                if t < t_min || t > t_max {
                    continue;
                }

                // Single Newton step (regarding self.pd)
                {
                    let h1 = j_max * t * t - self.vd;
                    let orig = -(3.0 * (self.a0_p4 + self.af_p4)
                        - 4.0 * (2.0 * self.a0_p3 + self.af_p3) * a_max
                        + 24.0 * self.af * a_max * self.j_max_j_max * t * t
                        - 12.0 * self.a0 * a_max * (self.af_af - 2.0 * j_max * h1)
                        + 6.0 * self.a0_a0 * (self.af_af + a_max * a_max - 2.0 * j_max * h1)
                        + 6.0
                        * self.af_af
                        * (a_max * a_max - 2.0 * j_max * (self.tf * a_max + h1))
                        + 12.0
                        * j_max
                        * (-a_max * a_max * h1 + j_max * h1 * h1
                        - 2.0
                        * a_max
                        * j_max
                        * (-self.pd
                        + j_max * t * t * (t - self.tf)
                        + self.tf * self.vf)))
                        / (24.0 * a_max * self.j_max_j_max);
                    let deriv = t
                        * (self.a0_a0 + self.af_af
                        - 2.0 * j_max * h1
                        - 2.0 * (self.a0 + self.af + j_max * self.tf) * a_max
                        + a_max * a_max
                        + 3.0 * a_max * j_max * t)
                        / a_max;

                    t -= orig / deriv;
                }

                let h1 =
                    ((self.a0_a0 + self.af_af) / 2.0 + j_max * (self.vd - j_max * t * t)) / a_max;

                profile.t[0] = (-self.a0 + a_max) / j_max;
                profile.t[1] = (h1 - a_max) / j_max;
                profile.t[2] = a_max / j_max;
                profile.t[3] = self.tf - (h1 - self.a0 - self.af + a_max) / j_max - 2.0 * t;
                profile.t[4] = t;
                profile.t[5] = 0.0;
                profile.t[6] = -(self.af / j_max) + t;

                if profile.check_with_timing(
                    ControlSigns::UDUD,
                    ReachedLimits::Acc0Vel,
                    j_max,
                    v_max,
                    v_min,
                    a_max,
                    a_min,
                ) {
                    return true;
                }
            }
        }

        false
    }

    fn time_vel(
        &mut self,
        profile: &mut Profile,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) -> bool {
        let tz_min = f64::max(0.0, -self.a0 / j_max);
        let tz_max = f64::min((self.tf - self.a0 / j_max) / 2.0, (a_max - self.a0) / j_max);

        // Profile UDDU
        if f64::abs(self.v0) < f64::EPSILON
            && f64::abs(self.a0) < f64::EPSILON
            && f64::abs(self.vf) < f64::EPSILON
            && f64::abs(self.af) < f64::EPSILON
        {
            let mut polynom = [0.0; 4];
            polynom[0] = 1.0;
            polynom[1] = -self.tf / 2.0;
            polynom[2] = 0.0;
            polynom[3] = self.pd / (2.0 * j_max);

            let roots = solve_cub(polynom[0], polynom[1], polynom[2], polynom[3]);
            for mut t in &mut roots.into_iter() {
                if t > self.tf / 4.0 {
                    continue;
                }

                // Single Newton step (regarding self.pd)
                if t > f64::EPSILON {
                    let orig = -self.pd + j_max * t * t * (self.tf - 2.0 * t);
                    let deriv = 2.0 * j_max * t * (self.tf - 3.0 * t);
                    t -= orig / deriv;
                }

                profile.t[0] = t;
                profile.t[1] = 0.0;
                profile.t[2] = t;
                profile.t[3] = self.tf - 4.0 * t;
                profile.t[4] = t;
                profile.t[5] = 0.0;
                profile.t[6] = t;

                if profile.check_with_timing(
                    ControlSigns::UDDU,
                    ReachedLimits::Vel,
                    j_max,
                    v_max,
                    v_min,
                    a_max,
                    a_min,
                ) {
                    return true;
                }
            }
        } else {
            let p1 = self.af_af
                - 2.0 * j_max * (-2.0 * self.af * self.tf + j_max * self.tf_tf + 3.0 * self.vd);
            let ph1 =
                self.af_p3 - 3.0 * self.j_max_j_max * self.g1 - 3.0 * self.af * j_max * self.vd;
            let ph2 = self.af_p4
                + 8.0 * self.af_p3 * j_max * self.tf
                + 12.0
                * j_max
                * (3.0 * j_max * self.vd_vd - self.af_af * self.vd
                + 2.0 * self.af * j_max * (self.g1 - self.tf * self.vd)
                - 2.0 * self.j_max_j_max * self.tf * self.g1);
            let ph3 = self.a0 * (self.af - j_max * self.tf);
            let ph4 = j_max * (-self.ad + j_max * self.tf);

            // Find root of 5th order polynom
            let mut polynom = ArrayVec::<f64, 6>::new();
            polynom.push(1.0);
            polynom.push((15.0 * self.a0_a0 + self.af_af + 4.0 * self.af * j_max * self.tf
                - 16.0 * ph3
                - 2.0 * j_max * (j_max * self.tf_tf + 3.0 * self.vd))
                / (4.0 * ph4));
            polynom.push((29.0 * self.a0_p3 - 2.0 * self.af_p3 - 33.0 * self.a0 * ph3
                + 6.0 * self.j_max_j_max * self.g1
                + 6.0 * self.af * j_max * self.vd
                + 6.0 * self.a0 * p1)
                / (6.0 * j_max * ph4));
            polynom.push((61.0 * self.a0_p4 - 76.0 * self.a0_a0 * ph3 - 16.0 * self.a0 * ph1
                + 30.0 * self.a0_a0 * p1
                + ph2)
                / (24.0 * self.j_max_j_max * ph4));
            polynom.push((self.a0
                * (7.0 * self.a0_p4 - 10.0 * self.a0_a0 * ph3 - 4.0 * self.a0 * ph1
                + 6.0 * self.a0_a0 * p1
                + ph2))
                / (12.0 * self.j_max_j_max * j_max * ph4));
            polynom.push((7.0 * self.a0_p6 + self.af_p6 - 12.0 * self.a0_p4 * ph3
                + 48.0 * self.af_p3 * self.j_max_j_max * self.g1
                - 8.0 * self.a0_p3 * ph1
                - 72.0
                * self.j_max_j_max
                * j_max
                * (j_max * self.g1 * self.g1
                + self.vd_vd * self.vd
                + 2.0 * self.af * self.g1 * self.vd)
                - 6.0 * self.af_p4 * j_max * self.vd
                + 36.0 * self.af_af * self.j_max_j_max * self.vd_vd
                + 9.0 * self.a0_p4 * p1
                + 3.0 * self.a0_a0 * ph2)
                / (144.0 * self.j_max_j_max * self.j_max_j_max * ph4));

            let deriv = poly_monic_deri(&polynom);
            let dderiv = poly_deri(&deriv);

            // Solve 4th order derivative analytically
            let d_extremas = solve_quart_monic_coeffs(deriv[1], deriv[2], deriv[3], deriv[4]);

            let mut tz_current = tz_min;

            let mut check_root = |mut t: f64| {
                // Single Newton step (regarding self.pd)
                {
                    let h1 = f64::sqrt(
                        (self.a0_a0 + self.af_af) / (2.0 * self.j_max_j_max)
                            + (2.0 * self.a0 * t + j_max * t * t - self.vd) / j_max,
                    );
                    let orig = -self.pd
                        - (2.0 * self.a0_p3
                        + 4.0 * self.af_p3
                        + 24.0 * self.a0 * j_max * t * (self.af + j_max * (h1 + t - self.tf))
                        + 6.0 * self.a0_a0 * (self.af + j_max * (2.0 * t - self.tf))
                        + 6.0 * (self.a0_a0 + self.af_af) * j_max * h1
                        + 12.0 * self.af * j_max * (j_max * t * t - self.vd)
                        + 12.0
                        * self.j_max_j_max
                        * (j_max * t * t * (h1 + t - self.tf)
                        - self.tf * self.v0
                        - h1 * self.vd))
                        / (12.0 * self.j_max_j_max);
                    let deriv_newton = -(self.a0 + j_max * t)
                        * (3.0 * (h1 + t) - 2.0 * self.tf + (self.a0 + 2.0 * self.af) / j_max);
                    if !orig.is_nan()
                        && !deriv_newton.is_nan()
                        && f64::abs(deriv_newton) > f64::EPSILON
                    {
                        t -= orig / deriv_newton;
                    }
                }

                if t > self.tf || t.is_nan() {
                    return false;
                }

                let h1 = f64::sqrt(
                    (self.a0_a0 + self.af_af) / (2.0 * self.j_max_j_max)
                        + (t * (2.0 * self.a0 + j_max * t) - self.vd) / j_max,
                );
                profile.t[0] = t;
                profile.t[1] = 0.0;
                profile.t[2] = t + self.a0 / j_max;
                profile.t[3] = self.tf - 2.0 * (t + h1) - (self.a0 + self.af) / j_max;
                profile.t[4] = h1;
                profile.t[5] = 0.0;
                profile.t[6] = h1 + self.af / j_max;

                profile.check_with_timing(
                    ControlSigns::UDDU,
                    ReachedLimits::Vel,
                    j_max,
                    v_max,
                    v_min,
                    a_max,
                    a_min,
                )
            };

            for mut tz in &mut d_extremas.into_iter() {
                if tz >= tz_max {
                    continue;
                }

                let orig = poly_eval(&deriv, tz);
                if f64::abs(orig) > TOLERANCE {
                    tz -= orig / poly_eval(&dderiv, tz);
                }

                let val_new = poly_eval(&polynom, tz);
                if f64::abs(val_new) < 64.0 * f64::abs(poly_eval(&dderiv, tz)) * TOLERANCE {
                    if check_root(tz) {
                        return true;
                    }
                } else if poly_eval(&polynom, tz_current) * val_new < 0.0 && check_root(shrink_interval_default::<6>(&polynom, tz_current, tz)) {
                    return true;
                }
                tz_current = tz;
            }
            let val_max = poly_eval(&polynom, tz_max);
            if poly_eval(&polynom, tz_current) * val_max < 0.0 {
                if check_root(shrink_interval_default::<6>(&polynom, tz_current, tz_max)) {
                    return true;
                }
            } else if f64::abs(val_max) < 8.0 * f64::EPSILON && check_root(tz_max) {
                return true;
            }
        }

        // Profile UDUD
        {
            let ph1 = self.af_af
                - 2.0 * j_max * (2.0 * self.af * self.tf + j_max * self.tf_tf - 3.0 * self.vd);
            let ph2 =
                self.af_p3 - 3.0 * self.j_max_j_max * self.g1 + 3.0 * self.af * j_max * self.vd;
            let ph3 = 2.0 * j_max * self.tf * self.g1 + 3.0 * self.vd_vd;
            let ph4 = self.af_p4 - 8.0 * self.af_p3 * j_max * self.tf
                + 12.0
                * j_max
                * (j_max * ph3
                + self.af_af * self.vd
                + 2.0 * self.af * j_max * (self.g1 - self.tf * self.vd));
            let ph5 = self.af + j_max * self.tf;

            // Find root of 6th order polynom
            let mut polynom = ArrayVec::<f64, 7>::new();
            polynom.push(1.0);
            polynom.push((5.0 * self.a0 - ph5) / j_max);
            polynom.push((39.0 * self.a0_a0 - ph1 - 16.0 * self.a0 * ph5) / (4.0 * self.j_max_j_max));
            polynom.push((55.0 * self.a0_p3 - 33.0 * self.a0_a0 * ph5 - 6.0 * self.a0 * ph1
                + 2.0 * ph2)
                / (6.0 * self.j_max_j_max * j_max));
            polynom.push((101.0 * self.a0_p4 + ph4 - 76.0 * self.a0_p3 * ph5 - 30.0 * self.a0_a0 * ph1
                + 16.0 * self.a0 * ph2)
                / (24.0 * self.j_max_j_max * self.j_max_j_max));
            polynom.push((self.a0
                * (11.0 * self.a0_p4 + ph4 - 10.0 * self.a0_p3 * ph5 - 6.0 * self.a0_a0 * ph1
                + 4.0 * self.a0 * ph2))
                / (12.0 * self.j_max_j_max * self.j_max_j_max * j_max));
            polynom.push((11.0 * self.a0_p6
                - self.af_p6
                - 12.0 * self.a0_p5 * ph5
                - 48.0 * self.af_p3 * self.j_max_j_max * self.g1
                - 9.0 * self.a0_p4 * ph1
                + 72.0
                * self.j_max_j_max
                * j_max
                * (j_max * self.g1 * self.g1
                - self.vd_vd * self.vd
                - 2.0 * self.af * self.g1 * self.vd)
                - 6.0 * self.af_p4 * j_max * self.vd
                - 36.0 * self.af_af * self.j_max_j_max * self.vd_vd
                + 8.0 * self.a0_p3 * ph2
                + 3.0 * self.a0_a0 * ph4)
                / (144.0 * self.j_max_j_max * self.j_max_j_max * self.j_max_j_max));

            let deriv = poly_monic_deri(&polynom);
            let dderiv = poly_monic_deri(&deriv);

            let mut dd_tz_current = tz_min;
            let mut dd_tz_intervals: Set<(f64, f64), 6> = Set::new();

            let dd_extremas =
                solve_quart_monic_coeffs(dderiv[1], dderiv[2], dderiv[3], dderiv[4]);
            for mut tz in &mut dd_extremas.into_iter() {
                if tz >= tz_max {
                    continue;
                }

                let orig = poly_eval(&dderiv, tz);
                if f64::abs(orig) > TOLERANCE {
                    tz -= orig / poly_eval(&poly_deri(&dderiv), tz);
                }

                if poly_eval(&deriv, dd_tz_current) * poly_eval(&deriv, tz) < 0.0 {
                    dd_tz_intervals.insert((dd_tz_current, tz));
                }
                dd_tz_current = tz;
            }
            if poly_eval(&deriv, dd_tz_current) * poly_eval(&deriv, tz_max) < 0.0 {
                dd_tz_intervals.insert((dd_tz_current, tz_max));
            }

            let mut tz_current = tz_min;

            let mut check_root = |mut t: f64| {
                // Double Newton step (regarding self.pd)
                {
                    let mut h1 = f64::sqrt(
                        (self.af_af - self.a0_a0) / (2.0 * self.j_max_j_max)
                            - ((2.0 * self.a0 + j_max * t) * t - self.vd) / j_max,
                    );
                    let mut orig = -self.pd
                        + (self.af_p3 - self.a0_p3
                        + 3.0 * self.a0_a0 * j_max * (self.tf - 2.0 * t))
                        / (6.0 * self.j_max_j_max)
                        + (2.0 * self.a0 + j_max * t) * t * (self.tf - t)
                        + (j_max * h1 - self.af) * h1 * h1
                        + self.tf * self.v0;
                    let mut deriv_newton = (self.a0 + j_max * t)
                        * (2.0 * (self.af + j_max * self.tf) - 3.0 * j_max * (h1 + t) - self.a0)
                        / j_max;

                    t -= orig / deriv_newton;

                    h1 = f64::sqrt(
                        (self.af_af - self.a0_a0) / (2.0 * self.j_max_j_max)
                            - ((2.0 * self.a0 + j_max * t) * t - self.vd) / j_max,
                    );
                    orig = -self.pd
                        + (self.af_p3 - self.a0_p3
                        + 3.0 * self.a0_a0 * j_max * (self.tf - 2.0 * t))
                        / (6.0 * self.j_max_j_max)
                        + (2.0 * self.a0 + j_max * t) * t * (self.tf - t)
                        + (j_max * h1 - self.af) * h1 * h1
                        + self.tf * self.v0;
                    if f64::abs(orig) > 1e-9 {
                        deriv_newton = (self.a0 + j_max * t)
                            * (2.0 * (self.af + j_max * self.tf)
                            - 3.0 * j_max * (h1 + t)
                            - self.a0)
                            / j_max;

                        t -= orig / deriv_newton;
                    }
                }

                let h1 = f64::sqrt(
                    (self.af_af - self.a0_a0) / (2.0 * self.j_max_j_max)
                        - ((2.0 * self.a0 + j_max * t) * t - self.vd) / j_max,
                );
                profile.t[0] = t;
                profile.t[1] = 0.0;
                profile.t[2] = t + self.a0 / j_max;
                profile.t[3] = self.tf - 2.0 * (t + h1) + self.ad / j_max;
                profile.t[4] = h1;
                profile.t[5] = 0.0;
                profile.t[6] = h1 - self.af / j_max;

                profile.check_with_timing(
                    ControlSigns::UDUD,
                    ReachedLimits::Vel,
                    j_max,
                    v_max,
                    v_min,
                    a_max,
                    a_min,
                )
            };
            for interval in dd_tz_intervals {
                let (first, second) = interval;
                let tz = shrink_interval_default::<7>(&deriv, first, second);

                if tz >= tz_max {
                    continue;
                }

                let p_val = poly_eval(&polynom, tz);
                if f64::abs(p_val) < 64.0 * f64::abs(poly_eval(&dderiv, tz)) * TOLERANCE {
                    if check_root(tz) {
                        return true;
                    }
                } else if poly_eval(&polynom, tz_current) * p_val < 0.0 && check_root(shrink_interval_default::<7>(&polynom, tz_current, tz)) {
                    return true;
                }
                tz_current = tz;
            }
            if poly_eval(&polynom, tz_current) * poly_eval(&polynom, tz_max) < 0.0 && check_root(shrink_interval_default::<7>(&polynom, tz_current, tz_max)) {
                return true;
            }
        }

        false
    }

    fn time_acc0_acc1(
        &mut self,
        profile: &mut Profile,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        _: f64,
    ) -> bool {
        if f64::abs(self.a0) < f64::EPSILON && f64::abs(self.af) < f64::EPSILON {
            let h1 = 2.0 * a_min * self.g1
                + self.vd_vd
                + a_max * (2.0 * self.pd + a_min * self.tf_tf - 2.0 * self.tf * self.vf);
            let h2 = (a_max - a_min) * (-a_min * self.vd + a_max * (a_min * self.tf - self.vd));

            let jf = h2 / h1;
            profile.t[0] = a_max / jf;
            profile.t[1] = (-2.0 * a_max * h1 + a_min * a_min * self.g2) / h2;
            profile.t[2] = profile.t[0];
            profile.t[3] = 0.0;
            profile.t[4] = -a_min / jf;
            profile.t[5] = self.tf - (2.0 * profile.t[0] + profile.t[1] + 2.0 * profile.t[4]);
            profile.t[6] = profile.t[4];

            return profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::Acc0Acc1,
                jf,
                v_max,
                v_min,
                a_max,
                a_min,
            );
        }
        // UDDU
        {
            let h1 = f64::sqrt(
                144.0
                    * pow2(
                    (a_max - a_min) * (-a_min * self.vd + a_max * (a_min * self.tf - self.vd))
                        - self.af_af * (a_max * self.tf - self.vd)
                        + 2.0 * self.af * a_min * (a_max * self.tf - self.vd)
                        + self.a0_a0 * (a_min * self.tf + self.v0 - self.vf)
                        - 2.0 * self.a0 * a_max * (a_min * self.tf - self.vd),
                )
                    + 48.0
                    * self.ad
                    * (3.0 * self.a0_p3 - 3.0 * self.af_p3
                    + 12.0 * a_max * a_min * (-a_max + a_min)
                    + 4.0 * self.af_af * (a_max + 2.0 * a_min)
                    + self.a0
                    * (-3.0 * self.af_af
                    + 8.0 * self.af * (a_min - a_max)
                    + 6.0 * (a_max * a_max + 2.0 * a_max * a_min - a_min * a_min))
                    + 6.0
                    * self.af
                    * (a_max * a_max - 2.0 * a_max * a_min - a_min * a_min)
                    + self.a0_a0 * (3.0 * self.af - 4.0 * (2.0 * a_max + a_min)))
                    * (2.0 * a_min * self.g1
                    + self.vd * self.vd
                    + a_max
                    * (2.0 * self.pd + a_min * self.tf * self.tf
                    - 2.0 * self.tf * self.vf)),
            );
            let jf = -(3.0 * self.af_af * a_max * self.tf
                - 3.0 * self.a0_a0 * a_min * self.tf
                - 6.0 * self.ad * a_max * a_min * self.tf
                + 3.0 * a_max * a_min * (a_min - a_max) * self.tf
                + 3.0 * (self.a0_a0 - self.af_af) * self.vd
                + 6.0 * self.vd * (self.af * a_min - self.a0 * a_max)
                + 3.0 * (a_max * a_max - a_min * a_min) * self.vd
                + h1 / 4.0)
                / (6.0
                * (2.0 * a_min * self.g1
                + self.vd * self.vd
                + a_max * (2.0 * self.pd + a_min * self.tf_tf - 2.0 * self.tf * self.vf)));
            profile.t[0] = (a_max - self.a0) / jf;
            profile.t[1] = (self.a0_a0 - self.af_af + 2.0 * self.ad * a_min
                - 2.0
                * (a_max * a_max - 2.0 * a_max * a_min + a_min * a_min + a_min * jf * self.tf
                - jf * self.vd))
                / (2.0 * (a_max - a_min) * jf);
            profile.t[2] = a_max / jf;
            profile.t[3] = 0.0;
            profile.t[4] = -a_min / jf;
            profile.t[5] = self.tf
                - (profile.t[0] + profile.t[1] + profile.t[2] + 2.0 * profile.t[4] + self.af / jf);
            profile.t[6] = profile.t[4] + self.af / jf;

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::Acc0Acc1,
                jf,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        false
    }

    fn time_acc1(
        &mut self,
        profile: &mut Profile,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) -> bool {
        // a3 != 0.0
        // Case UDDU
        {
            let h0 = f64::sqrt(
                self.j_max_j_max
                    * (self.a0_p4 + self.af_p4 - 4.0 * self.af_p3 * j_max * self.tf
                    + 6.0 * self.af_af * self.j_max_j_max * self.tf_tf
                    - 4.0 * self.a0_p3 * (self.af - j_max * self.tf)
                    + 6.0
                    * self.a0_a0
                    * (self.af - j_max * self.tf)
                    * (self.af - j_max * self.tf)
                    + 24.0 * self.af * self.j_max_j_max * self.g1
                    - 4.0
                    * self.a0
                    * (self.af_p3 - 3.0 * self.af_af * j_max * self.tf
                    + 6.0 * self.j_max_j_max * (-self.pd + self.tf * self.vf))
                    - 12.0 * self.j_max_j_max * (-self.vd_vd + j_max * self.tf * self.g2))
                    / 3.0,
            ) / j_max;
            let h1 = f64::sqrt(
                (self.a0_a0 + self.af_af
                    - 2.0 * self.a0 * self.af
                    - 2.0 * self.ad * j_max * self.tf
                    + 2.0 * h0)
                    / self.j_max_j_max
                    + self.tf_tf,
            );

            profile.t[0] = -(self.a0_a0 + self.af_af + 2.0 * self.a0 * (j_max * self.tf - self.af)
                - 2.0 * j_max * self.vd
                + h0)
                / (2.0 * j_max * (-self.ad + j_max * self.tf));
            profile.t[1] = 0.0;
            profile.t[2] = (self.tf - h1) / 2.0 - self.ad / (2.0 * j_max);
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = h1;
            profile.t[6] = self.tf - (profile.t[0] + profile.t[2] + profile.t[5]);

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::Acc1,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        // Case UDUD
        {
            let h0 = f64::sqrt(
                self.j_max_j_max
                    * (self.a0_p4
                    + self.af_p4
                    + 4.0 * (self.af_p3 - self.a0_p3) * j_max * self.tf
                    + 6.0 * self.af_af * self.j_max_j_max * self.tf_tf
                    + 6.0
                    * self.a0_a0
                    * (self.af + j_max * self.tf)
                    * (self.af + j_max * self.tf)
                    + 24.0 * self.af * self.j_max_j_max * self.g1
                    - 4.0
                    * self.a0
                    * (self.a0_a0 * self.af
                    + self.af_p3
                    + 3.0 * self.af_af * j_max * self.tf
                    + 6.0 * self.j_max_j_max * (-self.pd + self.tf * self.vf))
                    + 12.0 * self.j_max_j_max * (self.vd_vd + j_max * self.tf * self.g2))
                    / 3.0,
            ) / j_max;
            let h1 = f64::sqrt(
                (self.a0_a0 + self.af_af - 2.0 * self.a0 * self.af
                    + 2.0 * self.ad * j_max * self.tf
                    + 2.0 * h0)
                    / self.j_max_j_max
                    + self.tf_tf,
            );

            profile.t[0] = 0.0;
            profile.t[1] = 0.0;
            profile.t[2] = -(self.a0_a0 + self.af_af - 2.0 * self.a0 * self.af
                + 2.0 * j_max * (self.vd - self.a0 * self.tf)
                + h0)
                / (2.0 * j_max * (self.ad + j_max * self.tf));
            profile.t[3] = 0.0;
            profile.t[4] = self.ad / (2.0 * j_max) + (self.tf - h1) / 2.0;
            profile.t[5] = h1;
            profile.t[6] = self.tf - (profile.t[5] + profile.t[4] + profile.t[2]);

            if profile.check_with_timing(
                ControlSigns::UDUD,
                ReachedLimits::Acc1,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        // Case UDDU, Solution 2
        {
            let h0a = self.a0_p3 - self.af_p3 - 3.0 * self.a0_a0 * a_min
                + 3.0 * a_min * a_min * (self.a0 + j_max * self.tf)
                + 3.0 * self.af * a_min * (-a_min - 2.0 * j_max * self.tf)
                - 3.0 * self.af_af * (-a_min - j_max * self.tf)
                - 3.0
                * self.j_max_j_max
                * (-2.0 * self.pd - a_min * self.tf_tf + 2.0 * self.tf * self.vf);
            let h0b = self.a0_a0 + self.af_af - 2.0 * (self.a0 + self.af) * a_min
                + 2.0 * (a_min * a_min - j_max * (-a_min * self.tf + self.vd));
            let h0c = self.a0_p4 + 3.0 * self.af_p4 - 4.0 * (self.a0_p3 + 2.0 * self.af_p3) * a_min
                + 6.0 * self.a0_a0 * a_min * a_min
                + 6.0 * self.af_af * (a_min * a_min - 2.0 * j_max * self.vd)
                + 12.0
                * j_max
                * (2.0 * a_min * j_max * self.g1 - a_min * a_min * self.vd
                + j_max * self.vd_vd)
                + 24.0 * self.af * a_min * j_max * self.vd
                - 4.0
                * self.a0
                * (self.af_p3 - 3.0 * self.af * a_min * (-a_min - 2.0 * j_max * self.tf)
                + 3.0 * self.af_af * (-a_min - j_max * self.tf)
                + 3.0
                * j_max
                * (-a_min * a_min * self.tf
                + j_max
                * (-2.0 * self.pd - a_min * self.tf_tf
                + 2.0 * self.tf * self.vf)));
            let h1 = f64::abs(j_max) / j_max * f64::sqrt(4.0 * h0a * h0a - 6.0 * h0b * h0c);
            let h2 = 6.0 * j_max * h0b;

            profile.t[0] = 0.0;
            profile.t[1] = 0.0;
            profile.t[2] = (2.0 * h0a + h1) / h2;
            profile.t[3] = -(self.a0_a0 + self.af_af - 2.0 * (self.a0 + self.af) * a_min
                + 2.0 * (a_min * a_min + a_min * j_max * self.tf - j_max * self.vd))
                / (2.0 * j_max * (self.a0 - a_min - j_max * profile.t[2]));
            profile.t[4] = (self.a0 - a_min) / j_max - profile.t[2];
            profile.t[5] =
                self.tf - (profile.t[2] + profile.t[3] + profile.t[4] + (self.af - a_min) / j_max);
            profile.t[6] = (self.af - a_min) / j_max;

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::Acc1,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        // Case UDUD, Solution 1
        {
            let h0a = -self.a0_p3 + self.af_p3 + 3.0 * (self.a0_a0 - self.af_af) * a_max
                - 3.0 * self.ad * a_max * a_max
                - 6.0 * self.af * a_max * j_max * self.tf
                + 3.0 * self.af_af * j_max * self.tf
                + 3.0
                * j_max
                * (a_max * a_max * self.tf
                + j_max * (-2.0 * self.pd - a_max * self.tf_tf + 2.0 * self.tf * self.vf));
            let h0b = self.a0_a0 - self.af_af
                + 2.0 * self.ad * a_max
                + 2.0 * j_max * (a_max * self.tf - self.vd);
            let h0c = self.a0_p4 + 3.0 * self.af_p4 - 4.0 * (self.a0_p3 + 2.0 * self.af_p3) * a_max
                + 6.0 * self.a0_a0 * a_max * a_max
                - 24.0 * self.af * a_max * j_max * self.vd
                + 12.0
                * j_max
                * (2.0 * a_max * j_max * self.g1
                + j_max * self.vd_vd
                + a_max * a_max * self.vd)
                + 6.0 * self.af_af * (a_max * a_max + 2.0 * j_max * self.vd)
                - 4.0
                * self.a0
                * (self.af_p3 + 3.0 * self.af * a_max * (a_max - 2.0 * j_max * self.tf)
                - 3.0 * self.af_af * (a_max - j_max * self.tf)
                + 3.0
                * j_max
                * (a_max * a_max * self.tf
                + j_max
                * (-2.0 * self.pd - a_max * self.tf_tf
                + 2.0 * self.tf * self.vf)));
            let h1 = f64::abs(j_max) / j_max * f64::sqrt(4.0 * h0a * h0a - 6.0 * h0b * h0c);
            let h2 = 6.0 * j_max * h0b;

            profile.t[0] = 0.0;
            profile.t[1] = 0.0;
            profile.t[2] = -(2.0 * h0a + h1) / h2;
            profile.t[3] = 2.0 * h1 / h2;
            profile.t[4] = (a_max - self.a0) / j_max + profile.t[2];
            profile.t[5] =
                self.tf - (profile.t[2] + profile.t[3] + profile.t[4] + (-self.af + a_max) / j_max);
            profile.t[6] = (-self.af + a_max) / j_max;

            if profile.check_with_timing(
                ControlSigns::UDUD,
                ReachedLimits::Acc1,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }
        false
    }

    fn time_acc0(
        &mut self,
        profile: &mut Profile,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) -> bool {
        // UDUD
        {
            let h1 = f64::sqrt(
                self.ad_ad / (2.0 * self.j_max_j_max)
                    - self.ad * (a_max - self.a0) / (self.j_max_j_max)
                    + (a_max * self.tf - self.vd) / j_max,
            );

            profile.t[0] = (a_max - self.a0) / j_max;
            profile.t[1] = self.tf - self.ad / j_max - 2.0 * h1;
            profile.t[2] = h1;
            profile.t[3] = 0.0;
            profile.t[4] = (self.af - a_max) / j_max + h1;
            profile.t[5] = 0.0;
            profile.t[6] = 0.0;

            if profile.check_with_timing(
                ControlSigns::UDUD,
                ReachedLimits::None,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        // UDUD
        {
            let h0a = -self.a0_a0 + self.af_af - 2.0 * self.ad * a_max
                + 2.0 * j_max * (a_max * self.tf - self.vd);
            let h0b = self.a0_p3 + 2.0 * self.af_p3
                - 6.0 * self.af_af * a_max
                - 3.0 * self.a0_a0 * (self.af - j_max * self.tf)
                - 3.0 * self.a0 * a_max * (a_max - 2.0 * self.af + 2.0 * j_max * self.tf)
                - 3.0
                * j_max
                * (j_max * (-2.0 * self.pd + a_max * self.tf_tf + 2.0 * self.tf * self.v0)
                + a_max * (a_max * self.tf - 2.0 * self.vd))
                + 3.0
                * self.af
                * (a_max * a_max + 2.0 * a_max * j_max * self.tf - 2.0 * j_max * self.vd);
            let h0 = f64::abs(j_max) * f64::sqrt(4.0 * h0b * h0b - 18.0 * h0a * h0a * h0a);
            let h1 = 3.0 * j_max * h0a;

            profile.t[0] = (-self.a0 + a_max) / j_max;
            profile.t[1] = (-self.a0_p3
                + self.af_p3
                + self.af_af * (-6.0 * a_max + 3.0 * j_max * self.tf)
                + self.a0_a0 * (-3.0 * self.af + 6.0 * a_max + 3.0 * j_max * self.tf)
                + 6.0 * self.af * (a_max * a_max - j_max * self.vd)
                + 3.0 * self.a0 * (self.af_af - 2.0 * (a_max * a_max + j_max * self.vd))
                - 6.0 * j_max * (a_max * (a_max * self.tf - 2.0 * self.vd) + j_max * self.g2))
                / h1;
            profile.t[2] =
                -(self.ad + h0 / h1) / (2.0 * j_max) + self.tf / 2.0 - profile.t[1] / 2.0;
            profile.t[3] = h0 / (j_max * h1);
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = self.tf - (profile.t[0] + profile.t[1] + profile.t[2] + profile.t[3]);

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::None,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }
        // a3 != 0.0
        // UDDU Solution 1
        {
            let h0a = self.a0_p3 + 2.0 * self.af_p3
                - 6.0 * (self.af_af + a_max * a_max) * a_max
                - 6.0 * (self.a0 + self.af) * a_max * j_max * self.tf
                + 9.0 * a_max * a_max * (self.af + j_max * self.tf)
                + 3.0 * self.a0 * a_max * (-2.0 * self.af + 3.0 * a_max)
                + 3.0 * self.a0_a0 * (self.af - 2.0 * a_max + j_max * self.tf)
                - 6.0 * self.j_max_j_max * self.g1
                + 6.0 * (self.af - a_max) * j_max * self.vd
                - 3.0 * a_max * self.j_max_j_max * self.tf_tf;
            let h0b = self.a0_a0
                + self.af_af
                + 2.0
                * (a_max * a_max - (self.a0 + self.af) * a_max
                + j_max * (self.vd - a_max * self.tf));
            let h1 = f64::abs(j_max) / j_max * f64::sqrt(4.0 * h0a * h0a - 18.0 * h0b * h0b * h0b);
            let h2 = 6.0 * j_max * h0b;

            profile.t[0] = (-self.a0 + a_max) / j_max;
            profile.t[1] = self.ad / j_max - 2.0 * profile.t[0] - (2.0 * h0a - h1) / h2 + self.tf;
            profile.t[2] = -(2.0 * h0a + h1) / h2;
            profile.t[3] = (2.0 * h0a - h1) / h2;
            profile.t[4] = self.tf - (profile.t[0] + profile.t[1] + profile.t[2] + profile.t[3]);
            profile.t[5] = 0.0;
            profile.t[6] = 0.0;

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::Acc0,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }
        false
    }

    fn time_none(
        &mut self,
        profile: &mut Profile,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) -> bool {
        if f64::abs(self.v0) < f64::EPSILON
            && f64::abs(self.a0) < f64::EPSILON
            && f64::abs(self.af) < f64::EPSILON
        {
            let h1 = f64::sqrt(self.tf_tf * self.vf_vf + pow2(4.0 * self.pd - self.tf * self.vf));
            let jf = 4.0 * (4.0 * self.pd - 2.0 * self.tf * self.vf + h1) / self.tf_p3;

            profile.t[0] = self.tf / 4.0;
            profile.t[1] = 0.0;
            profile.t[2] = 2.0 * profile.t[0];
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = profile.t[0];

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::None,
                jf,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        if f64::abs(self.a0) < f64::EPSILON && f64::abs(self.af) < f64::EPSILON {
            // Is that really needed?
            // Profiles with a3 != 0, Solution UDDU
            {
                // First acc, then constant
                {
                    let mut polynom = [0.0; 4];
                    polynom[0] = -2.0 * self.tf;
                    polynom[1] = 2.0 * self.vd / j_max + self.tf_tf;
                    polynom[2] = 4.0 * (self.pd - self.tf * self.vf) / j_max;
                    polynom[3] = (self.vd_vd + j_max * self.tf * self.g2) / (self.j_max_j_max);

                    let roots = solve_quart_monic_arr(&polynom);
                    for mut t in &mut roots.into_iter() {
                        if t > self.tf / 2.0 || t > (a_max - self.a0) / j_max {
                            continue;
                        }

                        // Single Newton step (regarding self.pd)
                        {
                            let h1 = (j_max * t * (t - self.tf) + self.vd)
                                / (j_max * (2.0 * t - self.tf));
                            let h2 = (2.0 * j_max * t * (t - self.tf) + j_max * self.tf_tf
                                - 2.0 * self.vd)
                                / (j_max * (2.0 * t - self.tf) * (2.0 * t - self.tf));
                            let orig = (-2.0 * self.pd
                                + 2.0 * self.tf * self.v0
                                + h1 * h1 * j_max * (self.tf - 2.0 * t)
                                + j_max
                                * self.tf
                                * (2.0 * h1 * t - t * t - (h1 - t) * self.tf))
                                / 2.0;
                            let deriv = (j_max * self.tf * (2.0 * t - self.tf) * (h2 - 1.0)) / 2.0
                                + h1 * j_max * (self.tf - (2.0 * t - self.tf) * h2 - h1);

                            t -= orig / deriv;
                        }

                        profile.t[0] = t;
                        profile.t[1] = 0.0;
                        profile.t[2] = (j_max * t * (t - self.tf) + self.vd)
                            / (j_max * (2.0 * t - self.tf));
                        profile.t[3] = self.tf - 2.0 * t;
                        profile.t[4] = t - profile.t[2];
                        profile.t[5] = 0.0;
                        profile.t[6] = 0.0;

                        if profile.check_with_timing(
                            ControlSigns::UDDU,
                            ReachedLimits::None,
                            j_max,
                            v_max,
                            v_min,
                            a_max,
                            a_min,
                        ) {
                            return true;
                        }
                    }
                }
            }
        }

        // UDUD T 0246
        {
            let h0 = f64::sqrt(
                2.0 * self.j_max_j_max
                    * (2.0
                    * pow2(
                    self.a0_p3 - self.af_p3 - 3.0 * self.af_af * j_max * self.tf
                        + 9.0 * self.af * self.j_max_j_max * self.tf_tf
                        - 3.0 * self.a0_a0 * (self.af + j_max * self.tf)
                        + 3.0 * self.a0 * pow2(self.af + j_max * self.tf)
                        + 3.0
                        * self.j_max_j_max
                        * (8.0 * self.pd + j_max * self.tf_tf * self.tf
                        - 8.0 * self.tf * self.vf),
                )
                    - 3.0
                    * (self.a0_a0 + self.af_af
                    - 2.0 * self.af * j_max * self.tf
                    - 2.0 * self.a0 * (self.af + j_max * self.tf)
                    - j_max * (j_max * self.tf_tf + 4.0 * self.v0 - 4.0 * self.vf))
                    * (self.a0_p4
                    + self.af_p4
                    + 4.0 * self.af_p3 * j_max * self.tf
                    + 6.0 * self.af_af * self.j_max_j_max * self.tf_tf
                    - 3.0
                    * self.j_max_j_max
                    * self.j_max_j_max
                    * self.tf_tf
                    * self.tf_tf
                    - 4.0 * self.a0_p3 * (self.af + j_max * self.tf)
                    + 6.0 * self.a0_a0 * pow2(self.af + j_max * self.tf)
                    - 12.0
                    * self.af
                    * self.j_max_j_max
                    * (8.0 * self.pd + j_max * self.tf_tf * self.tf
                    - 8.0 * self.tf * self.v0)
                    + 48.0 * self.j_max_j_max * self.vd_vd
                    + 48.0 * self.j_max_j_max * j_max * self.tf * self.g2
                    - 4.0
                    * self.a0
                    * (self.af_p3 + 3.0 * self.af_af * j_max * self.tf
                    - 9.0 * self.af * self.j_max_j_max * self.tf_tf
                    - 3.0
                    * self.j_max_j_max
                    * (8.0 * self.pd + j_max * self.tf_tf * self.tf
                    - 8.0 * self.tf * self.vf)))),
            ) / j_max;
            let h1 = 12.0
                * j_max
                * (-self.a0_a0 - self.af_af
                + 2.0 * self.af * j_max * self.tf
                + 2.0 * self.a0 * (self.af + j_max * self.tf)
                + j_max * (j_max * self.tf_tf + 4.0 * self.v0 - 4.0 * self.vf));
            let h2 = -4.0 * self.a0_p3 + 4.0 * self.af_p3 + 12.0 * self.a0_a0 * self.af
                - 12.0 * self.a0 * self.af_af
                + 48.0 * self.j_max_j_max * self.pd
                + 12.0 * (self.a0_a0 - self.af_af) * j_max * self.tf
                - 24.0 * self.j_max_j_max * self.tf * (self.v0 + self.vf)
                + 24.0 * self.ad * j_max * self.vd;
            let h3 = 2.0 * self.a0_p3 - 2.0 * self.af_p3 - 6.0 * self.a0_a0 * self.af
                + 6.0 * self.a0 * self.af_af;

            profile.t[0] = (h3
                - 48.0 * self.j_max_j_max * (self.tf * self.vf - self.pd)
                - 6.0 * (self.a0_a0 + self.af_af) * j_max * self.tf
                + 12.0 * self.a0 * self.af * j_max * self.tf
                + 6.0
                * (self.a0 + 3.0 * self.af + j_max * self.tf)
                * self.tf_tf
                * self.j_max_j_max
                - h0)
                / h1;
            profile.t[1] = 0.0;
            profile.t[2] = (h2 + h0) / h1;
            profile.t[3] = 0.0;
            profile.t[4] = (-h2 + h0) / h1;
            profile.t[5] = 0.0;
            profile.t[6] = (-h3 + 48.0 * self.j_max_j_max * (self.tf * self.v0 - self.pd)
                - 6.0 * (self.a0_a0 + self.af_af) * j_max * self.tf
                + 12.0 * self.a0 * self.af * j_max * self.tf
                + 6.0
                * (self.af + 3.0 * self.a0 + j_max * self.tf)
                * self.tf_tf
                * self.j_max_j_max
                - h0)
                / h1;

            if profile.check_with_timing(
                ControlSigns::UDUD,
                ReachedLimits::None,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        // Profiles with a3 != 0.0, Solution UDDU
        {
            // T 0234
            {
                let ph1 = self.af + j_max * self.tf;

                let mut polynom = [0.0; 4];
                polynom[0] = -2.0 * (self.ad + j_max * self.tf) / j_max;
                polynom[1] = 2.0
                    * (self.a0_a0 + self.af_af + j_max * (self.af * self.tf + self.vd)
                    - 2.0 * self.a0 * ph1)
                    / self.j_max_j_max
                    + self.tf_tf;
                polynom[2] = 2.0
                    * (self.a0_p3 - self.af_p3 - 3.0 * self.af_af * j_max * self.tf
                    + 3.0 * self.a0 * ph1 * (ph1 - self.a0)
                    - 6.0 * self.j_max_j_max * (-self.pd + self.tf * self.vf))
                    / (3.0 * self.j_max_j_max * j_max);
                polynom[3] = (self.a0_p4 + self.af_p4 + 4.0 * self.af_p3 * j_max * self.tf
                    - 4.0 * self.a0_p3 * ph1
                    + 6.0 * self.a0_a0 * ph1 * ph1
                    + 24.0 * self.j_max_j_max * self.af * self.g1
                    - 4.0
                    * self.a0
                    * (self.af_p3
                    + 3.0 * self.af_af * j_max * self.tf
                    + 6.0 * self.j_max_j_max * (-self.pd + self.tf * self.vf))
                    + 6.0 * self.j_max_j_max * self.af_af * self.tf_tf
                    + 12.0 * self.j_max_j_max * (self.vd_vd + j_max * self.tf * self.g2))
                    / (12.0 * self.j_max_j_max * self.j_max_j_max);

                let t_min = self.ad / j_max;
                let t_max = f64::min((a_max - self.a0) / j_max, (self.ad / j_max + self.tf) / 2.0);

                let roots = solve_quart_monic_arr(&polynom);
                for mut t in &mut roots.into_iter() {
                    if t < t_min || t > t_max {
                        continue;
                    }

                    // Single Newton step (regarding self.pd)
                    {
                        let h0 = j_max * (2.0 * t - self.tf) - self.ad;
                        let h1 = (self.ad_ad - 2.0 * self.af * j_max * t
                            + 2.0 * self.a0 * j_max * (t - self.tf)
                            + 2.0 * j_max * (j_max * t * (t - self.tf) + self.vd))
                            / (2.0 * j_max * h0);
                        let h2 = (-self.ad_ad
                            + 2.0 * self.j_max_j_max * (self.tf_tf + t * (t - self.tf))
                            + (self.a0 + self.af) * j_max * self.tf
                            - self.ad * h0
                            - 2.0 * j_max * self.vd)
                            / (h0 * h0);
                        let orig = (-self.a0_p3
                            + self.af_p3
                            + 3.0 * self.ad_ad * j_max * (h1 - t)
                            + 3.0 * self.ad * self.j_max_j_max * (h1 - t) * (h1 - t)
                            - 3.0 * self.a0 * self.af * self.ad
                            + 3.0
                            * self.j_max_j_max
                            * (self.a0 * self.tf_tf - 2.0 * self.pd
                            + 2.0 * self.tf * self.v0
                            + h1 * h1 * j_max * (self.tf - 2.0 * t)
                            + j_max
                            * self.tf
                            * (2.0 * h1 * t - t * t - (h1 - t) * self.tf)))
                            / (6.0 * self.j_max_j_max);
                        let deriv = (h0 * (-self.ad + j_max * self.tf) * (h2 - 1.0))
                            / (2.0 * j_max)
                            + h1 * (-self.ad + j_max * (self.tf - h1) - h0 * h2);

                        t -= orig / deriv;
                    }

                    profile.t[0] = t;
                    profile.t[1] = 0.0;
                    profile.t[2] = (self.ad_ad
                        + 2.0
                        * j_max
                        * (-self.a0 * self.tf - self.ad * t
                        + j_max * t * (t - self.tf)
                        + self.vd))
                        / (2.0 * j_max * (-self.ad + j_max * (2.0 * t - self.tf)));
                    profile.t[3] = self.ad / j_max + self.tf - 2.0 * t;
                    profile.t[4] = self.tf - (t + profile.t[2] + profile.t[3]);
                    profile.t[5] = 0.0;
                    profile.t[6] = 0.0;

                    if profile.check_with_timing(
                        ControlSigns::UDDU,
                        ReachedLimits::None,
                        j_max,
                        v_max,
                        v_min,
                        a_max,
                        a_min,
                    ) {
                        return true;
                    }
                }
            }

            // T 3456
            {
                let h1 = 3.0 * j_max * (self.ad_ad + 2.0 * j_max * (self.a0 * self.tf - self.vd));
                let h2 = self.ad_ad + 2.0 * j_max * (self.a0 * self.tf - self.vd);
                let h0 = f64::sqrt(
                    4.0 * pow2(
                        2.0 * (self.a0_p3 - self.af_p3)
                            - 6.0 * self.a0_a0 * (self.af - j_max * self.tf)
                            + 6.0 * self.j_max_j_max * self.g1
                            + 3.0
                            * self.a0
                            * (2.0 * self.af_af - 2.0 * j_max * self.af * self.tf
                            + self.j_max_j_max * self.tf_tf)
                            + 6.0 * self.ad * j_max * self.vd,
                    ) - 18.0 * h2 * h2 * h2,
                ) / h1
                    * f64::abs(j_max)
                    / j_max;

                profile.t[0] = 0.0;
                profile.t[1] = 0.0;
                profile.t[2] = 0.0;
                profile.t[3] = (self.af_p3 - self.a0_p3
                    + 3.0 * (self.af_af - self.a0_a0) * j_max * self.tf
                    - 3.0 * self.ad * (self.a0 * self.af + 2.0 * j_max * self.vd)
                    - 6.0 * self.j_max_j_max * self.g2)
                    / h1;
                profile.t[4] = (self.tf - profile.t[3] - h0) / 2.0 - self.ad / (2.0 * j_max);
                profile.t[5] = h0;
                profile.t[6] = (self.tf - profile.t[3] + self.ad / j_max - h0) / 2.0;

                if profile.check_with_timing(
                    ControlSigns::UDDU,
                    ReachedLimits::None,
                    j_max,
                    v_max,
                    v_min,
                    a_max,
                    a_min,
                ) {
                    return true;
                }
            }

            // T 2346
            {
                let ph1 = self.ad_ad + 2.0 * (self.af + self.a0) * j_max * self.tf
                    - j_max * (j_max * self.tf_tf + 4.0 * self.vd);
                let ph2 = j_max * self.tf_tf * self.g1
                    - self.vd * (-2.0 * self.pd - self.tf * self.v0 + 3.0 * self.tf * self.vf);
                let ph3 = 5.0 * self.af_af - 8.0 * self.af * j_max * self.tf
                    + 2.0 * j_max * (2.0 * j_max * self.tf_tf - self.vd);
                let ph4 = self.j_max_j_max * self.tf_p4 - 2.0 * self.vd_vd
                    + 8.0 * j_max * self.tf * (-self.pd + self.tf * self.vf);
                let ph5 = 5.0 * self.af_p4
                    - 8.0 * self.af_p3 * j_max * self.tf
                    - 12.0 * self.af_af * j_max * (j_max * self.tf_tf + self.vd)
                    + 24.0
                    * self.af
                    * self.j_max_j_max
                    * (-2.0 * self.pd + j_max * self.tf_p3 + 2.0 * self.tf * self.vf)
                    - 6.0 * self.j_max_j_max * ph4;
                let ph6 = -self.vd_vd
                    + j_max
                    * self.tf
                    * (-2.0 * self.pd + 3.0 * self.tf * self.v0 - self.tf * self.vf)
                    - self.af * self.g2;

                let mut polynom = [0.0; 4];
                polynom[0] = -(4.0 * (self.a0_p3 - self.af_p3)
                    - 12.0 * self.a0_a0 * (self.af - j_max * self.tf)
                    + 6.0
                    * self.a0
                    * (2.0 * self.af_af - 2.0 * self.af * j_max * self.tf
                    + j_max * (j_max * self.tf_tf - 2.0 * self.vd))
                    + 6.0 * self.af * j_max * (3.0 * j_max * self.tf_tf + 2.0 * self.vd)
                    - 6.0
                    * self.j_max_j_max
                    * (-4.0 * self.pd + j_max * self.tf_p3 - 2.0 * self.tf * self.v0
                    + 6.0 * self.tf * self.vf))
                    / (3.0 * j_max * ph1);
                polynom[1] = -(-self.a0_p4 - self.af_p4
                    + 4.0 * self.a0_p3 * (self.af - j_max * self.tf)
                    + self.a0_a0
                    * (-6.0 * self.af_af + 8.0 * self.af * j_max * self.tf
                    - 4.0 * j_max * (j_max * self.tf_tf - self.vd))
                    + 2.0 * self.af_af * j_max * (j_max * self.tf_tf + 2.0 * self.vd)
                    - 4.0
                    * self.af
                    * self.j_max_j_max
                    * (-3.0 * self.pd
                    + j_max * self.tf_p3
                    + 2.0 * self.tf * self.v0
                    + self.tf * self.vf)
                    + self.j_max_j_max
                    * (self.j_max_j_max * self.tf_p4 - 8.0 * self.vd_vd
                    + 4.0
                    * j_max
                    * self.tf
                    * (-3.0 * self.pd + self.tf * self.v0 + 2.0 * self.tf * self.vf))
                    + 2.0
                    * self.a0
                    * (2.0 * self.af_p3 - 2.0 * self.af_af * j_max * self.tf
                    + self.af * j_max * (-3.0 * j_max * self.tf_tf - 4.0 * self.vd)
                    + self.j_max_j_max
                    * (-6.0 * self.pd + j_max * self.tf_p3 - 4.0 * self.tf * self.v0
                    + 10.0 * self.tf * self.vf)))
                    / (self.j_max_j_max * ph1);
                polynom[2] = -(self.a0_p5 - self.af_p5 + self.af_p4 * j_max * self.tf
                    - 5.0 * self.a0_p4 * (self.af - j_max * self.tf)
                    + 2.0 * self.a0_p3 * ph3
                    + 4.0 * self.af_p3 * j_max * (j_max * self.tf_tf + self.vd)
                    + 12.0 * self.j_max_j_max * self.af * ph6
                    - 2.0
                    * self.a0_a0
                    * (5.0 * self.af_p3
                    - 9.0 * self.af_af * j_max * self.tf
                    - 6.0 * self.af * j_max * self.vd
                    + 6.0
                    * self.j_max_j_max
                    * (-2.0 * self.pd - self.tf * self.v0 + 3.0 * self.tf * self.vf))
                    - 12.0 * self.j_max_j_max * j_max * ph2
                    + self.a0 * ph5)
                    / (3.0 * self.j_max_j_max * j_max * ph1);
                polynom[3] = -(-self.a0_p6 - self.af_p6
                    + 6.0 * self.a0_p5 * (self.af - j_max * self.tf)
                    - 48.0 * self.af_p3 * self.j_max_j_max * self.g1
                    + 72.0
                    * self.j_max_j_max
                    * j_max
                    * (j_max * self.g1 * self.g1
                    + self.vd_vd * self.vd
                    + 2.0 * self.af * self.g1 * self.vd)
                    - 3.0 * self.a0_p4 * ph3
                    - 36.0 * self.af_af * self.j_max_j_max * self.vd_vd
                    + 6.0 * self.af_p4 * j_max * self.vd
                    + 4.0
                    * self.a0_p3
                    * (5.0 * self.af_p3
                    - 9.0 * self.af_af * j_max * self.tf
                    - 6.0 * self.af * j_max * self.vd
                    + 6.0
                    * self.j_max_j_max
                    * (-2.0 * self.pd - self.tf * self.v0 + 3.0 * self.tf * self.vf))
                    - 3.0 * self.a0_a0 * ph5
                    + 6.0
                    * self.a0
                    * (self.af_p5
                    - self.af_p4 * j_max * self.tf
                    - 4.0 * self.af_p3 * j_max * (j_max * self.tf_tf + self.vd)
                    + 12.0 * self.j_max_j_max * (-self.af * ph6 + j_max * ph2)))
                    / (18.0 * self.j_max_j_max * self.j_max_j_max * ph1);

                let t_max = (self.a0 - a_min) / j_max;

                let roots = solve_quart_monic_arr(&polynom);
                for mut t in &mut roots.into_iter() {
                    if t > t_max {
                        continue;
                    }

                    // Single Newton step (regarding self.pd)
                    {
                        let h1 = self.ad_ad / 2.0
                            + j_max
                            * (self.af * t + (j_max * t - self.a0) * (t - self.tf)
                            - self.vd);
                        let h2 = -self.ad + j_max * (self.tf - 2.0 * t);
                        let h3 = f64::sqrt(h1);
                        let orig = (self.af_p3 - self.a0_p3
                            + 3.0 * self.af * j_max * t * (self.af + j_max * t)
                            + 3.0 * self.a0_a0 * (self.af + j_max * t)
                            - 3.0
                            * self.a0
                            * (self.af_af
                            + 2.0 * self.af * j_max * t
                            + self.j_max_j_max * (t * t - self.tf_tf))
                            + 3.0
                            * self.j_max_j_max
                            * (-2.0 * self.pd
                            + j_max * t * (t - self.tf) * self.tf
                            + 2.0 * self.tf * self.v0))
                            / (6.0 * self.j_max_j_max)
                            - h3 * h3 * h3 / (j_max * f64::abs(j_max))
                            + ((-self.ad - j_max * t) * h1) / (self.j_max_j_max);
                        let deriv = (6.0 * j_max * h2 * h3 / f64::abs(j_max)
                            + 2.0 * (-self.ad - j_max * self.tf) * h2
                            - 2.0
                            * (3.0 * self.ad_ad
                            + self.af * j_max * (8.0 * t - 2.0 * self.tf)
                            + 4.0 * self.a0 * j_max * (-2.0 * t + self.tf)
                            + 2.0
                            * j_max
                            * (j_max * t * (3.0 * t - 2.0 * self.tf) - self.vd)))
                            / (4.0 * j_max);

                        t -= orig / deriv;
                    }

                    let h1 = f64::sqrt(
                        2.0 * self.ad_ad
                            + 4.0
                            * j_max
                            * (self.ad * t + self.a0 * self.tf + j_max * t * (t - self.tf)
                            - self.vd),
                    ) / f64::abs(j_max);

                    // Solution 2.0 with aPlat
                    profile.t[0] = 0.0;
                    profile.t[1] = 0.0;
                    profile.t[2] = t;
                    profile.t[3] = self.tf - 2.0 * t - self.ad / j_max - h1;
                    profile.t[4] = h1 / 2.0;
                    profile.t[5] = 0.0;
                    profile.t[6] = self.tf - (t + profile.t[3] + profile.t[4]);

                    if profile.check_with_timing(
                        ControlSigns::UDDU,
                        ReachedLimits::None,
                        j_max,
                        v_max,
                        v_min,
                        a_max,
                        a_min,
                    ) {
                        return true;
                    }
                }
            }
        }

        // Profiles with a3 != 0.0, Solution UDUD
        {
            // T 0124
            {
                let ph0 = -2.0 * self.pd - self.tf * self.v0 + 3.0 * self.tf * self.vf;
                let ph1 = -self.ad + j_max * self.tf;
                let ph2 = j_max * self.tf_tf * self.g1 - self.vd * ph0;
                let ph3 = 5.0 * self.af_af
                    + 2.0 * j_max * (2.0 * j_max * self.tf_tf - self.vd - 4.0 * self.af * self.tf);
                let ph4 = self.j_max_j_max * self.tf_p4 - 2.0 * self.vd_vd
                    + 8.0 * j_max * self.tf * (-self.pd + self.tf * self.vf);
                let ph5 = 5.0 * self.af_p4
                    - 8.0 * self.af_p3 * j_max * self.tf
                    - 12.0 * self.af_af * j_max * (j_max * self.tf_tf + self.vd)
                    + 24.0
                    * self.af
                    * self.j_max_j_max
                    * (-2.0 * self.pd + j_max * self.tf_p3 + 2.0 * self.tf * self.vf)
                    - 6.0 * self.j_max_j_max * ph4;
                let ph6 = -self.vd_vd
                    + j_max
                    * self.tf
                    * (-2.0 * self.pd + 3.0 * self.tf * self.v0 - self.tf * self.vf);
                let ph7 = 3.0 * self.j_max_j_max * ph1 * ph1;

                let mut polynom = [0.0; 4];
                polynom[0] =
                    (4.0 * self.af * self.tf - 2.0 * j_max * self.tf_tf - 4.0 * self.vd) / ph1;
                polynom[1] = (-2.0 * (self.a0_p4 + self.af_p4)
                    + 8.0 * self.af_p3 * j_max * self.tf
                    + 6.0 * self.af_af * self.j_max_j_max * self.tf_tf
                    + 8.0 * self.a0_p3 * (self.af - j_max * self.tf)
                    - 12.0
                    * self.a0_a0
                    * (self.af - j_max * self.tf)
                    * (self.af - j_max * self.tf)
                    - 12.0
                    * self.af
                    * self.j_max_j_max
                    * (-self.pd + j_max * self.tf_p3 - 2.0 * self.tf * self.v0
                    + 3.0 * self.tf * self.vf)
                    + 2.0
                    * self.a0
                    * (4.0 * self.af_p3 - 12.0 * self.af_af * j_max * self.tf
                    + 9.0 * self.af * self.j_max_j_max * self.tf_tf
                    - 3.0
                    * self.j_max_j_max
                    * (2.0 * self.pd + j_max * self.tf_p3 - 2.0 * self.tf * self.vf))
                    + 3.0
                    * self.j_max_j_max
                    * (self.j_max_j_max * self.tf_p4 + 4.0 * self.vd_vd
                    - 4.0
                    * j_max
                    * self.tf
                    * (self.pd + self.tf * self.v0 - 2.0 * self.tf * self.vf)))
                    / ph7;
                polynom[2] = (-self.a0_p5 + self.af_p5 - self.af_p4 * j_max * self.tf
                    + 5.0 * self.a0_p4 * (self.af - j_max * self.tf)
                    - 2.0 * self.a0_p3 * ph3
                    - 4.0 * self.af_p3 * j_max * (j_max * self.tf_tf + self.vd)
                    + 12.0 * self.af_af * self.j_max_j_max * self.g2
                    - 12.0 * self.af * self.j_max_j_max * ph6
                    + 2.0
                    * self.a0_a0
                    * (5.0 * self.af_p3
                    - 9.0 * self.af_af * j_max * self.tf
                    - 6.0 * self.af * j_max * self.vd
                    + 6.0 * self.j_max_j_max * ph0)
                    + 12.0 * self.j_max_j_max * j_max * ph2
                    + self.a0
                    * (-5.0 * self.af_p4
                    + 8.0 * self.af_p3 * j_max * self.tf
                    + 12.0 * self.af_af * j_max * (j_max * self.tf_tf + self.vd)
                    - 24.0
                    * self.af
                    * self.j_max_j_max
                    * (-2.0 * self.pd + j_max * self.tf_p3 + 2.0 * self.tf * self.vf)
                    + 6.0 * self.j_max_j_max * ph4))
                    / (j_max * ph7);
                polynom[3] = -(self.a0_p6 + self.af_p6
                    - 6.0 * self.a0_p5 * (self.af - j_max * self.tf)
                    + 48.0 * self.af_p3 * self.j_max_j_max * self.g1
                    - 72.0
                    * self.j_max_j_max
                    * j_max
                    * (j_max * self.g1 * self.g1
                    + self.vd_vd * self.vd
                    + 2.0 * self.af * self.g1 * self.vd)
                    + 3.0 * self.a0_p4 * ph3
                    - 6.0 * self.af_p4 * j_max * self.vd
                    + 36.0 * self.af_af * self.j_max_j_max * self.vd_vd
                    - 4.0
                    * self.a0_p3
                    * (5.0 * self.af_p3
                    - 9.0 * self.af_af * j_max * self.tf
                    - 6.0 * self.af * j_max * self.vd
                    + 6.0 * self.j_max_j_max * ph0)
                    + 3.0 * self.a0_a0 * ph5
                    - 6.0
                    * self.a0
                    * (self.af_p5
                    - self.af_p4 * j_max * self.tf
                    - 4.0 * self.af_p3 * j_max * (j_max * self.tf_tf + self.vd)
                    + 12.0
                    * self.j_max_j_max
                    * (self.af_af * self.g2 - self.af * ph6 + j_max * ph2)))
                    / (6.0 * self.j_max_j_max * ph7);

                let roots = solve_quart_monic_arr(&polynom);
                for t in &mut roots.into_iter() {
                    if t > self.tf || t > (a_max - self.a0) / j_max {
                        continue;
                    }

                    let h1 = f64::sqrt(
                        self.ad_ad / (2.0 * self.j_max_j_max)
                            + (self.a0 * (t + self.tf) - self.af * t + j_max * t * self.tf
                            - self.vd)
                            / j_max,
                    );

                    profile.t[0] = t;
                    profile.t[1] = self.tf - self.ad / j_max - 2.0 * h1;
                    profile.t[2] = h1;
                    profile.t[3] = 0.0;
                    profile.t[4] = self.ad / j_max + h1 - t;
                    profile.t[5] = 0.0;
                    profile.t[6] = 0.0;

                    if profile.check_with_timing(
                        ControlSigns::UDUD,
                        ReachedLimits::None,
                        j_max,
                        v_max,
                        v_min,
                        a_max,
                        a_min,
                    ) {
                        return true;
                    }
                }
            }
        }

        // 3 step profile (ak. UZD), sometimes missed because of numerical errors T 012
        {
            let h1 = (-self.ad_ad + j_max * (2.0 * (self.a0 + self.af) * self.tf - 4.0 * self.vd + j_max * self.tf_tf)).sqrt() / j_max.abs();

            profile.t[0] = (self.tf - h1 + self.ad / j_max) / 2.0;
            profile.t[1] = h1;
            profile.t[2] = (self.tf - h1 - self.ad / j_max) / 2.0;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = 0.0;

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::None,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        // 3 step profile (ak. UZU), sometimes missed because of numerical errors
        {
            let mut polynom = [0.0; 4];
            polynom[0] = self.ad_ad;
            polynom[1] = self.ad_ad * self.tf;
            polynom[2] = (self.a0_a0 + self.af_af + 10.0 * self.a0 * self.af) * self.tf_tf + 24.0 * (self.tf * (self.af * self.v0 - self.a0 * self.vf) - self.pd * self.ad) + 12.0 * self.vd_vd;
            polynom[3] = -3.0 * self.tf * ((self.a0_a0 + self.af_af + 2.0 * self.a0 * self.af) * self.tf_tf - 4.0 * self.vd * (self.a0 + self.af) * self.tf + 4.0 * self.vd_vd);

            let roots = solve_cub(polynom[0], polynom[1], polynom[2], polynom[3]);
            for t in &mut roots.into_iter() {
                if t > self.tf {
                    continue;
                }
                let jf = self.ad / (self.tf - t);

                profile.t[0] = (2.0 * (self.vd - self.a0 * self.tf) + self.ad * (t - self.tf)) / (2.0 * jf * t);
                profile.t[1] = t;
                profile.t[2] = 0.0;
                profile.t[3] = 0.0;
                profile.t[4] = 0.0;
                profile.t[5] = 0.0;
                profile.t[6] = self.tf - (profile.t[0] + profile.t[1]);

                if profile.check_with_timing(
                    ControlSigns::UDDU,
                    ReachedLimits::None,
                    j_max,
                    v_max,
                    v_min,
                    a_max,
                    a_min,
                ) {
                    return true;
                }
            }
        }

        // 3 step profile (ak. UDU), sometimes missed because of numerical errors
        {
            profile.t[0] = (self.ad_ad / j_max + 2.0 * (self.a0 + self.af) * self.tf - j_max * self.tf_tf - 4.0 * self.vd) / (4.0 * (self.ad - j_max * self.tf));
            profile.t[1] = 0.0;
            profile.t[2] = -self.ad / (2.0 * j_max) + self.tf / 2.0;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = self.tf - (profile.t[0] + profile.t[2]);

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::None,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        false
    }

    fn time_none_smooth(
        &mut self,
        profile: &mut Profile,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) -> bool {
        {
            let h0 = self.ad_ad + 2.0 * j_max * (self.a0 * self.tf - self.vd);
            let h1a = 2.0 * (self.a0_p3 - self.af_p3)
                - 6.0 * self.a0_a0 * (self.af - j_max * self.tf)
                + 6.0 * self.j_max_j_max * (-self.pd + self.tf * self.v0)
                + 6.0 * self.a0 * self.af_af
                + 3.0 * self.a0 * j_max * (j_max * self.tf_tf - 2.0 * self.vd)
                + 6.0 * self.af * j_max * (self.vd - self.tf * self.a0);
            let h1 = f64::sqrt(4.0 * h1a * h1a - 18.0 * h0 * h0 * h0) * f64::abs(j_max) / j_max;

            profile.t[0] = 0.0;
            profile.t[1] =
                (-self.a0_p3 + self.af_p3 + 3.0 * (self.af_af - self.a0_a0) * j_max * self.tf
                    - 3.0 * self.a0 * self.af * self.ad
                    - 6.0 * j_max * self.ad * self.vd
                    - 6.0 * self.j_max_j_max * (-2.0 * self.pd + self.tf * (self.v0 + self.vf)))
                    / (3.0 * j_max * h0);
            profile.t[2] = (4.0 * (self.a0_p3 - self.af_p3)
                + 6.0 * self.j_max_j_max * self.a0 * self.tf_tf
                + 12.0 * self.a0 * self.af * self.ad
                + 12.0
                * j_max
                * (j_max * (self.tf * self.v0 - self.pd)
                + self.ad * (self.vd - self.a0 * self.tf))
                - h1)
                / (6.0 * j_max * h0);
            profile.t[3] = h1 / (3.0 * j_max * h0);
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = self.tf - (profile.t[1] + profile.t[2] + profile.t[3]);

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::None,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        {
            let h0 = self.ad_ad + 2.0 * j_max * (self.vd - self.af * self.tf);
            let h0b = self.af_p3
                - 3.0
                * self.j_max_j_max
                * (self.af * self.tf_tf + 2.0 * (self.pd - self.tf * self.vf));
            let h1a = self.a0_p3 + 3.0 * self.a0 * self.af * self.ad - h0b;
            let h1 = f64::sqrt(
                4.0 * h1a * h1a
                    - 6.0
                    * h0
                    * (self.a0_p4 + self.af_p4 - 4.0 * self.a0_p3 * self.af
                    + 6.0 * self.a0_a0 * self.af_af
                    + 12.0
                    * self.j_max_j_max
                    * (self.vd_vd - 2.0 * self.af * (self.pd - self.tf * self.v0))
                    - 4.0 * self.a0 * h0b),
            ) * f64::abs(j_max)
                / j_max;

            profile.t[0] = -(2.0 * h1a + h1) / (6.0 * j_max * h0);
            profile.t[1] = h1 / (3.0 * j_max * h0);
            profile.t[2] = profile.t[0] - (self.af - self.a0) / j_max;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = self.tf - (profile.t[0] + profile.t[1] + profile.t[2]);
            profile.t[6] = 0.0;

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::None,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        // Solution 3
        {
            let h0 = f64::sqrt(
                3.0 * (self.a0_p4 + self.af_p4 - 4.0 * self.af_p3 * j_max * self.tf
                    + 6.0 * self.af_af * self.j_max_j_max * self.tf_tf
                    - 4.0 * self.a0_p3 * (self.af - j_max * self.tf)
                    + 6.0 * self.a0_a0 * (self.af - j_max * self.tf) * (self.af - j_max * self.tf)
                    + 24.0 * self.af * self.j_max_j_max * (-self.pd + self.tf * self.v0)
                    - 4.0
                    * self.a0
                    * (self.af_p3 - 3.0 * self.af_af * j_max * self.tf
                    + 6.0 * self.j_max_j_max * (-self.pd + self.tf * self.vf))
                    - 12.0
                    * self.j_max_j_max
                    * (-self.vd_vd
                    + j_max * self.tf * (-2.0 * self.pd + self.tf * (self.v0 + self.vf)))),
            ) * f64::abs(j_max)
                / j_max;
            let h1 = f64::sqrt(
                3.0 * (3.0 * self.a0_a0 + 3.0 * self.af_af
                    - 6.0 * self.a0 * self.af
                    - 6.0 * self.ad * j_max * self.tf
                    + 3.0 * self.j_max_j_max * self.tf_tf
                    - 2.0 * h0),
            ) * f64::abs(j_max)
                / j_max;

            profile.t[0] = (-3.0 * (self.a0_a0 + self.af_af)
                + 6.0 * self.a0 * self.af
                + 6.0 * j_max * (self.vd - self.a0 * self.tf)
                + h0)
                / (6.0 * j_max * (-self.ad + j_max * self.tf));
            profile.t[1] = 0.0;
            profile.t[2] = (3.0 * j_max * self.tf - 3.0 * self.ad - h1) / (6.0 * j_max);
            profile.t[3] = h1 / (3.0 * j_max);
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = self.tf - (profile.t[0] + profile.t[2] + profile.t[3]);

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::None,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        // Solution 2
        {
            let h0 = 6.0 * (self.ad_ad + 2.0 * self.af * j_max * self.tf - 2.0 * j_max * self.vd);
            let h1a = 2.0
                * (self.a0_p3 - self.af_p3
                + 3.0 * self.a0 * self.af * self.ad
                + 6.0 * self.j_max_j_max * (self.pd - self.tf * self.vf)
                + 3.0 * self.j_max_j_max * self.af * self.tf_tf);
            let h1 = f64::sqrt(
                h1a * h1a
                    - h0 * (self.a0_p4 - 4.0 * self.a0_p3 * self.af
                    + 6.0 * self.a0_a0 * self.af_af
                    + self.af_p4
                    + 24.0 * self.af * self.j_max_j_max * (-self.pd + self.tf * self.v0)
                    + 12.0 * self.j_max_j_max * self.vd_vd
                    - 4.0
                    * self.a0
                    * (self.af_p3 - 3.0 * self.af * self.j_max_j_max * self.tf_tf
                    + 6.0 * self.j_max_j_max * (-self.pd + self.tf * self.vf))),
            ) * f64::abs(j_max)
                / j_max;
            let h2 = 4.0 * self.a0_p3 - 4.0 * self.af_p3 + 12.0 * self.a0 * self.af * self.ad
                - 12.0 * self.j_max_j_max * (self.pd - self.tf * self.vf)
                - 6.0 * self.j_max_j_max * self.af * self.tf_tf
                + 12.0 * self.ad * j_max * (self.vd - self.af * self.tf);
            let h3 = j_max * h0;

            profile.t[0] = 0.0;
            profile.t[1] = 0.0;
            profile.t[2] = (h1a + h1) / h3;
            profile.t[3] = -(h2 + h1) / h3;
            profile.t[4] = (h2 - h1) / h3;
            profile.t[5] = self.tf - (profile.t[2] + profile.t[3] + profile.t[4]);
            profile.t[6] = 0.0;

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::None,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        // Solution 1
        {
            let h0 = f64::sqrt(
                (self.a0_p4 + self.af_p4 - 4.0 * self.af_p3 * j_max * self.tf
                    + 6.0 * self.af_af * self.j_max_j_max * self.tf_tf
                    - 4.0 * self.a0_p3 * (self.af - j_max * self.tf)
                    + 6.0 * self.a0_a0 * (self.af - j_max * self.tf) * (self.af - j_max * self.tf)
                    + 24.0 * self.af * self.j_max_j_max * (-self.pd + self.tf * self.v0)
                    - 4.0
                    * self.a0
                    * (self.af_p3 - 3.0 * self.af_af * j_max * self.tf
                    + 6.0 * self.j_max_j_max * (-self.pd + self.tf * self.vf))
                    - 12.0
                    * self.j_max_j_max
                    * (-self.vd_vd
                    + j_max * self.tf * (-2.0 * self.pd + self.tf * (self.v0 + self.vf))))
                    / 3.0,
            ) * f64::abs(j_max)
                / j_max;
            let h1 = f64::sqrt(
                self.ad_ad - 2.0 * self.ad * j_max * self.tf
                    + self.j_max_j_max * self.tf_tf
                    + 2.0 * h0,
            ) * f64::abs(j_max)
                / j_max;

            profile.t[0] = -(self.ad_ad + 2.0 * j_max * (self.a0 * self.tf - self.vd) + h0)
                / (2.0 * j_max * (-self.ad + j_max * self.tf));
            profile.t[1] = 0.0;
            profile.t[2] = 0.0;
            profile.t[3] = 0.0;
            profile.t[4] = (-self.ad + j_max * self.tf - h1) / (2.0 * j_max);
            profile.t[5] = h1 / j_max;
            profile.t[6] = self.tf - (profile.t[0] + profile.t[4] + profile.t[5]);

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::None,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        }

        false
    }

    pub fn get_profile(&mut self, profile: &mut Profile) -> bool {
        // Test all cases to get ones that match
        // However we should guess which one is correct and try them first...
        let up_first = self.pd > self.tf * self.v0;
        let v_max = if up_first { self._v_max } else { self._v_min };
        let v_min = if up_first { self._v_min } else { self._v_max };
        let a_max = if up_first { self._a_max } else { self._a_min };
        let a_min = if up_first { self._a_min } else { self._a_max };
        let j_max = if up_first { self._j_max } else { -self._j_max };

        if self.minimize_jerk
            && (self.time_none_smooth(profile, v_max, v_min, a_max, a_min, j_max)
            || self.time_none_smooth(profile, v_min, v_max, a_min, a_max, -j_max))
        {
            return true;
        }

        self.time_acc0_acc1_vel(profile, v_max, v_min, a_max, a_min, j_max)
            || self.time_vel(profile, v_max, v_min, a_max, a_min, j_max)
            || self.time_acc0_vel(profile, v_max, v_min, a_max, a_min, j_max)
            || self.time_acc1_vel(profile, v_max, v_min, a_max, a_min, j_max)
            || self.time_acc0_acc1_vel(profile, v_min, v_max, a_min, a_max, -j_max)
            || self.time_vel(profile, v_min, v_max, a_min, a_max, -j_max)
            || self.time_acc0_vel(profile, v_min, v_max, a_min, a_max, -j_max)
            || self.time_acc1_vel(profile, v_min, v_max, a_min, a_max, -j_max)
            || self.time_acc0_acc1(profile, v_max, v_min, a_max, a_min, j_max)
            || self.time_acc0(profile, v_max, v_min, a_max, a_min, j_max)
            || self.time_acc1(profile, v_max, v_min, a_max, a_min, j_max)
            || self.time_none(profile, v_max, v_min, a_max, a_min, j_max)
            || self.time_acc0_acc1(profile, v_min, v_max, a_min, a_max, -j_max)
            || self.time_acc0(profile, v_min, v_max, a_min, a_max, -j_max)
            || self.time_acc1(profile, v_min, v_max, a_min, a_max, -j_max)
            || self.time_none(profile, v_min, v_max, a_min, a_max, -j_max)
    }
}
