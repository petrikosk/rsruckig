//! Mathematical equations for Step 1 in third-order position interface: Extremal profiles
use crate::block::{Block, Interval};
use crate::profile::{ControlSigns, Profile, ReachedLimits};
use crate::roots;

#[derive(Default)]
pub struct PositionThirdOrderStep1 {
    v0: f64,
    a0: f64,
    vf: f64,
    af: f64,
    _v_max: f64,
    _v_min: f64,
    _a_max: f64,
    _a_min: f64,
    _j_max: f64,

    // Pre-calculated expressions
    pd: f64,
    v0_v0: f64,
    vf_vf: f64,
    a0_a0: f64,
    a0_p3: f64,
    a0_p4: f64,
    af_af: f64,
    af_p3: f64,
    af_p4: f64,
    j_max_j_max: f64,

    // Max 5 valid profiles + 1 spare for numerical issues
    valid_profiles: [Profile; 6],
    current_index: usize,
}

impl PositionThirdOrderStep1 {
    pub fn new(
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
        let v0_v0 = v0 * v0;
        let vf_vf = vf * vf;
        let a0_a0 = a0 * a0;
        let af_af = af * af;
        let a0_p3 = a0 * a0_a0;
        let a0_p4 = a0_a0 * a0_a0;
        let af_p3 = af * af_af;
        let af_p4 = af_af * af_af;
        let j_max_j_max = j_max * j_max;

        Self {
            v0,
            a0,
            vf,
            af,
            _v_max: v_max,
            _v_min: v_min,
            _a_max: a_max,
            _a_min: a_min,
            _j_max: j_max,
            pd,
            v0_v0,
            vf_vf,
            a0_a0,
            a0_p3,
            a0_p4,
            af_af,
            af_p3,
            af_p4,
            j_max_j_max,
            valid_profiles: Default::default(),
            current_index: 0,
        }
    }

    #[inline]
    fn add_profile(&mut self) {
        if self.current_index < 5 {
            self.current_index += 1;
            let (left, right) = self.valid_profiles.split_at_mut(self.current_index);
            right[0].set_boundary_from_profile(&left[left.len() - 1]);
        }
    }

    fn time_all_vel(
        &mut self,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
        _: bool,
    ) {
        let profile = &mut self.valid_profiles[self.current_index];
        // ACC0_ACC1_VEL
        profile.t[0] = (-self.a0 + a_max) / j_max;
        profile.t[1] =
            (self.a0_a0 / 2.0 - a_max * a_max - j_max * (self.v0 - v_max)) / (a_max * j_max);
        profile.t[2] = a_max / j_max;
        profile.t[3] = (3.0 * (self.a0_p4 * a_min - self.af_p4 * a_max)
            + 8.0
            * a_max
            * a_min
            * (self.af_p3 - self.a0_p3
            + 3.0 * j_max * (self.a0 * self.v0 - self.af * self.vf))
            + 6.0 * self.a0_a0 * a_min * (a_max * a_max - 2.0 * j_max * self.v0)
            - 6.0 * self.af_af * a_max * (a_min * a_min - 2.0 * j_max * self.vf)
            - 12.0
            * j_max
            * (a_max
            * a_min
            * (a_max * (self.v0 + v_max)
            - a_min * (self.vf + v_max)
            - 2.0 * j_max * self.pd)
            + (a_min - a_max) * j_max * v_max * v_max
            + j_max * (a_max * self.vf_vf - a_min * self.v0_v0)))
            / (24.0 * a_max * a_min * self.j_max_j_max * v_max);
        profile.t[4] = -a_min / j_max;
        profile.t[5] =
            -(self.af_af / 2.0 - a_min * a_min - j_max * (self.vf - v_max)) / (a_min * j_max);
        profile.t[6] = profile.t[4] + self.af / j_max;

        if profile.check_with_timing(
            ControlSigns::UDDU,
            ReachedLimits::Vel,
            j_max,
            v_max,
            v_min,
            a_max,
            a_min,
        ) {
            self.add_profile();
            return;
        }

        // ACC1_VEL
        let profile = &mut self.valid_profiles[self.current_index];
        let t_acc0 = (self.a0_a0 / (2.0 * self.j_max_j_max) + (v_max - self.v0) / j_max).sqrt();

        profile.t[0] = t_acc0 - self.a0 / j_max;
        profile.t[1] = 0.0;
        profile.t[2] = t_acc0;
        profile.t[3] = -(3.0 * self.af_p4
            - 8.0 * a_min * (self.af_p3 - self.a0_p3)
            - 24.0 * a_min * j_max * (self.a0 * self.v0 - self.af * self.vf)
            + 6.0 * self.af_af * (a_min * a_min - 2.0 * j_max * self.vf)
            - 12.0
            * j_max
            * (2.0 * a_min * j_max * self.pd
            + a_min * a_min * (self.vf + v_max)
            + j_max * (v_max * v_max - self.vf_vf)
            + a_min * t_acc0 * (self.a0_a0 - 2.0 * j_max * (self.v0 + v_max))))
            / (24.0 * a_min * self.j_max_j_max * v_max);

        if profile.check_with_timing(
            ControlSigns::UDDU,
            ReachedLimits::Vel,
            j_max,
            v_max,
            v_min,
            a_max,
            a_min,
        ) {
            self.add_profile();
            return;
        }

        // ACC0_VEL
        let profile = &mut self.valid_profiles[self.current_index];
        let t_acc1 = (self.af_af / (2.0 * self.j_max_j_max) + (v_max - self.vf) / j_max).sqrt();

        profile.t[0] = (-self.a0 + a_max) / j_max;
        profile.t[1] =
            (self.a0_a0 / 2.0 - a_max * a_max - j_max * (self.v0 - v_max)) / (a_max * j_max);
        profile.t[2] = a_max / j_max;
        profile.t[3] = (3.0 * self.a0_p4
            + 8.0 * a_max * (self.af_p3 - self.a0_p3)
            + 24.0 * a_max * j_max * (self.a0 * self.v0 - self.af * self.vf)
            + 6.0 * self.a0_a0 * (a_max * a_max - 2.0 * j_max * self.v0)
            - 12.0
            * j_max
            * (-2.0 * a_max * j_max * self.pd
            + a_max * a_max * (self.v0 + v_max)
            + j_max * (v_max * v_max - self.v0_v0)
            + a_max * t_acc1 * (-self.af_af + 2.0 * (self.vf + v_max) * j_max)))
            / (24.0 * a_max * self.j_max_j_max * v_max);
        profile.t[4] = t_acc1;
        profile.t[5] = 0.0;
        profile.t[6] = t_acc1 + self.af / j_max;

        if profile.check_with_timing(
            ControlSigns::UDDU,
            ReachedLimits::Vel,
            j_max,
            v_max,
            v_min,
            a_max,
            a_min,
        ) {
            self.add_profile();
            return;
        }

        // VEL
        // Solution 3/4
        let profile = &mut self.valid_profiles[self.current_index];
        profile.t[0] = t_acc0 - self.a0 / j_max;
        profile.t[1] = 0.0;
        profile.t[2] = t_acc0;
        profile.t[3] = (self.af_p3 - self.a0_p3) / (3.0 * self.j_max_j_max * v_max)
            + (self.a0 * self.v0 - self.af * self.vf
            + (self.af_af * t_acc1 + self.a0_a0 * t_acc0) / 2.0)
            / (j_max * v_max)
            - (self.v0 / v_max + 1.0) * t_acc0
            - (self.vf / v_max + 1.0) * t_acc1
            + self.pd / v_max;
        if profile.check_with_timing(
            ControlSigns::UDDU,
            ReachedLimits::Vel,
            j_max,
            v_max,
            v_min,
            a_max,
            a_min,
        ) {
            self.add_profile();
        }
    }
    fn time_acc0_acc1(
        &mut self,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
        return_after_found: bool,
    ) {
        let mut h1 = (3. * (self.af_p4 * a_max - self.a0_p4 * a_min)
            + a_max
            * a_min
            * (8. * (self.a0_p3 - self.af_p3)
            + 3. * a_max * a_min * (a_max - a_min)
            + 6. * a_min * self.af_af
            - 6. * a_max * self.a0_a0)
            + 12.
            * j_max
            * (a_max
            * a_min
            * ((a_max - 2. * self.a0) * self.v0 - (a_min - 2. * self.af) * self.vf)
            + a_min * self.a0_a0 * self.v0
            - a_max * self.af_af * self.vf))
            / (3. * (a_max - a_min) * self.j_max_j_max)
            + 4. * (a_max * self.vf_vf - a_min * self.v0_v0 - 2. * a_min * a_max * self.pd)
            / (a_max - a_min);

        if h1 >= 0. {
            h1 = f64::sqrt(h1) / 2.;
            let h2 = self.a0_a0 / (2. * a_max * j_max) + (a_min - 2. * a_max) / (2. * j_max)
                - self.v0 / a_max;
            let h3 = -self.af_af / (2. * a_min * j_max) - (a_max - 2. * a_min) / (2. * j_max)
                + self.vf / a_min;

            // UDDU: Solution 2

            if h2 > h1 / a_max && h3 > -h1 / a_min {
                let profile = &mut self.valid_profiles[self.current_index];
                profile.t[0] = (-self.a0 + a_max) / j_max;
                profile.t[1] = h2 - h1 / a_max;
                profile.t[2] = a_max / j_max;
                profile.t[3] = 0.;
                profile.t[4] = -a_min / j_max;
                profile.t[5] = h3 + h1 / a_min;
                profile.t[6] = profile.t[4] + self.af / j_max;

                if profile.check_with_timing(
                    ControlSigns::UDDU,
                    ReachedLimits::Acc0Acc1,
                    j_max,
                    v_max,
                    v_min,
                    a_max,
                    a_min,
                ) {
                    self.add_profile();
                    if return_after_found {
                        return;
                    }
                }
            }

            // UDDU: Solution 1
            if h2 > -h1 / a_max && h3 > h1 / a_min {
                let profile = &mut self.valid_profiles[self.current_index];
                profile.t[0] = (-self.a0 + a_max) / j_max;
                profile.t[1] = h2 + h1 / a_max;
                profile.t[2] = a_max / j_max;
                profile.t[3] = 0.;
                profile.t[4] = -a_min / j_max;
                profile.t[5] = h3 - h1 / a_min;
                profile.t[6] = profile.t[4] + self.af / j_max;

                if profile.check_with_timing(
                    ControlSigns::UDDU,
                    ReachedLimits::Acc0Acc1,
                    j_max,
                    v_max,
                    v_min,
                    a_max,
                    a_min,
                ) {
                    self.add_profile();
                }
            }
        }
    }

    fn time_all_none_acc0_acc1(
        &mut self,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
        return_after_found: bool,
    ) {
        let j_max_j_max = j_max * j_max;
        // NONE UDDU / UDUD Strategy: t7 == 0 (equals UDDU), this one is in particular prone to numerical issues
        let h2_none = (self.a0_a0 - self.af_af) / (2.0 * j_max) + (self.vf - self.v0);
        let h2_h2 = h2_none * h2_none;
        let t_min_none = (self.a0 - self.af) / j_max;
        let t_max_none = (a_max - a_min) / j_max;

        let mut polynom_none = [0.0; 4];
        polynom_none[0] = 0.0;
        polynom_none[1] =
            -2.0 * (self.a0_a0 + self.af_af - 2.0 * j_max * (self.v0 + self.vf)) / (j_max_j_max);
        polynom_none[2] = 4.0
            * (self.a0_p3 - self.af_p3 + 3.0 * j_max * (self.af * self.vf - self.a0 * self.v0))
            / (3.0 * j_max_j_max * j_max)
            - 4.0 * self.pd / j_max;
        polynom_none[3] = -h2_h2 / (j_max_j_max);

        // ACC0
        let h3_acc0 =
            (self.a0_a0 - self.af_af) / (2.0 * a_max * j_max) + (self.vf - self.v0) / a_max;
        let t_min_acc0 = (a_max - self.af) / j_max;
        let t_max_acc0 = (a_max - a_min) / j_max;

        let h0_acc0 = 3.0 * (self.af_p4 - self.a0_p4)
            + 8.0 * (self.a0_p3 - self.af_p3) * a_max
            + 24.0 * a_max * j_max * (self.af * self.vf - self.a0 * self.v0)
            - 6.0 * self.a0_a0 * (a_max * a_max - 2.0 * j_max * self.v0)
            + 6.0 * self.af_af * (a_max * a_max - 2.0 * j_max * self.vf)
            + 12.0
            * j_max
            * (j_max * (self.vf_vf - self.v0_v0 - 2.0 * a_max * self.pd)
            - a_max * a_max * (self.vf - self.v0));
        let h2_acc0 = -self.af_af + a_max * a_max + 2.0 * j_max * self.vf;

        let mut polynom_acc0 = [0.0; 4];
        polynom_acc0[0] = -2.0 * a_max / j_max;
        polynom_acc0[1] = h2_acc0 / (j_max_j_max);
        polynom_acc0[2] = 0.0;
        polynom_acc0[3] = h0_acc0 / (12.0 * self.j_max_j_max * self.j_max_j_max);

        // ACC1
        let h3_acc1 = -(self.a0_a0 + self.af_af) / (2.0 * j_max * a_min)
            + a_min / j_max
            + (self.vf - self.v0) / a_min;
        let t_min_acc1 = (a_min - self.a0) / j_max;
        let t_max_acc1 = (a_max - self.a0) / j_max;

        let h0_acc1 = (self.a0_p4 - self.af_p4) / 4.0
            + 2.0 * (self.af_p3 - self.a0_p3) * a_min / 3.0
            + (self.a0_a0 - self.af_af) * a_min * a_min / 2.0
            + j_max
            * (self.af_af * self.vf
            + self.a0_a0 * self.v0
            + 2.0 * a_min * (j_max * self.pd - self.a0 * self.v0 - self.af * self.vf)
            + a_min * a_min * (self.v0 + self.vf)
            + j_max * (self.v0_v0 - self.vf_vf));
        let h2_acc1 = self.a0_a0 - self.a0 * a_min + 2.0 * j_max * self.v0;

        let mut polynom_acc1 = [0.0; 4];
        polynom_acc1[0] = 2.0 * (2.0 * self.a0 - a_min) / j_max;
        polynom_acc1[1] =
            (5.0 * self.a0_a0 + a_min * (a_min - 6.0 * self.a0) + 2.0 * j_max * self.v0)
                / (j_max_j_max);
        polynom_acc1[2] = 2.0 * (self.a0 - a_min) * h2_acc1 / (j_max_j_max * j_max);
        polynom_acc1[3] = h0_acc1 / (self.j_max_j_max * self.j_max_j_max);

        let mut polynom_acc0_min = polynom_acc0;
        polynom_acc0_min[0] += 4.0 * t_min_acc0;
        polynom_acc0_min[1] += (3.0 * polynom_acc0[0] + 6.0 * t_min_acc0) * t_min_acc0;
        polynom_acc0_min[2] += (2.0 * polynom_acc0[1]
            + (3.0 * polynom_acc0[0] + 4.0 * t_min_acc0) * t_min_acc0)
            * t_min_acc0;
        polynom_acc0_min[3] += (polynom_acc0[2]
            + (polynom_acc0[1] + (polynom_acc0[0] + t_min_acc0) * t_min_acc0) * t_min_acc0)
            * t_min_acc0;

        // const bool polynom_none_has_solution = (polynom_none_min[0] < 0.0) || (polynom_none_min[1] < 0.0) || (polynom_none_min[2] < 0.0) || (polynom_none_min[3] <= 0.0);
        let polynom_acc0_has_solution = (polynom_acc0_min[0] < 0.0)
            || (polynom_acc0_min[1] < 0.0)
            || (polynom_acc0_min[2] < 0.0)
            || (polynom_acc0_min[3] <= 0.0);
        let polynom_acc1_has_solution = (polynom_acc1[0] < 0.0)
            || (polynom_acc1[1] < 0.0)
            || (polynom_acc1[2] < 0.0)
            || (polynom_acc1[3] <= 0.0);

        let roots_none = roots::solve_quart_monic_arr(&polynom_none);
        let roots_acc0 = if polynom_acc0_has_solution {
            roots::solve_quart_monic_arr(&polynom_acc0)
        } else {
            roots::PositiveSet::new()
        };
        let roots_acc1 = if polynom_acc1_has_solution {
            roots::solve_quart_monic_arr(&polynom_acc1)
        } else {
            roots::PositiveSet::new()
        };

        for mut t in &mut roots_none.into_iter() {
            if t < t_min_none || t > t_max_none {
                continue;
            }

            // Single Newton-step (regarding pd)
            if t > f64::EPSILON {
                let h1 = j_max * t * t;
                let orig = -h2_h2 / (4.0 * j_max * t)
                    + h2_none * (self.af / j_max + t)
                    + (4.0 * self.a0_p3 + 2.0 * self.af_p3
                    - 6.0 * self.a0_a0 * (self.af + 2.0 * j_max * t)
                    + 12.0 * (self.af - self.a0) * j_max * self.v0
                    + 3.0 * self.j_max_j_max * (-4.0 * self.pd + (h1 + 8.0 * self.v0) * t))
                    / (12.0 * self.j_max_j_max);
                let deriv = h2_none + 2.0 * self.v0 - self.a0_a0 / j_max
                    + h2_h2 / (4.0 * h1)
                    + (3.0 * h1) / 4.0;

                let delta_t = orig / deriv;
                t -= delta_t;
            }
            let profile = &mut self.valid_profiles[self.current_index];
            let h0 = h2_none / (2.0 * j_max * t);
            profile.t[0] = h0 + t / 2.0 - self.a0 / j_max;
            profile.t[1] = 0.0;
            profile.t[2] = t;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = -h0 + t / 2.0 + self.af / j_max;

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::None,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                self.add_profile();
                if return_after_found {
                    return;
                }
            }
        }

        for mut t in &mut roots_acc0.into_iter() {
            if t < t_min_acc0 || t > t_max_acc0 {
                continue;
            }

            // Single Newton step (regarding pd)
            if t > f64::EPSILON {
                let h1 = j_max * t;
                let orig = h0_acc0 / (12.0 * self.j_max_j_max * t) + t * (h2_acc0 + h1 * (h1 - 2.0 * a_max));
                let deriv = 2.0 * (h2_acc0 + h1 * (2.0 * h1 - 3.0 * a_max));

                let delta_t = orig / deriv;
                t -= delta_t;
            }
            let profile = &mut self.valid_profiles[self.current_index];
            profile.t[0] = (-self.a0 + a_max) / j_max;
            profile.t[1] = h3_acc0 - 2.0 * t + (j_max / a_max) * t * t;
            profile.t[2] = t;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = (self.af - a_max) / j_max + t;

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::Acc0,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                self.add_profile();
                if return_after_found {
                    return;
                }
            }
        }

        for mut t in &mut roots_acc1.into_iter() {
            if t < t_min_acc1 || t > t_max_acc1 {
                continue;
            }

            // Double Newton step (regarding pd)
            if t > f64::EPSILON {
                let h5 = self.a0_p3 + 2.0 * j_max * self.a0 * self.v0;
                let mut h1 = j_max * t;
                let mut orig = -(h0_acc1 / 2.0
                    + h1 * (h5
                    + self.a0 * (a_min - 2.0 * h1) * (a_min - h1)
                    + self.a0_a0 * (5.0 * h1 / 2.0 - 2.0 * a_min)
                    + a_min * a_min * h1 / 2.0
                    + j_max * (h1 / 2.0 - a_min) * (h1 * t + 2.0 * self.v0)))
                    / j_max;
                let mut deriv =
                    (a_min - self.a0 - h1) * (h2_acc1 + h1 * (4.0 * self.a0 - a_min + 2.0 * h1));
                let mut delta_t = f64::min(orig / deriv, t);
                t -= delta_t;

                h1 = j_max * t;
                orig = -(h0_acc1 / 2.0
                    + h1 * (h5
                    + self.a0 * (a_min - 2.0 * h1) * (a_min - h1)
                    + self.a0_a0 * (5.0 * h1 / 2.0 - 2.0 * a_min)
                    + a_min * a_min * h1 / 2.0
                    + j_max * (h1 / 2.0 - a_min) * (h1 * t + 2.0 * self.v0)))
                    / j_max;

                if orig.abs() > 1e-9 {
                    deriv = (a_min - self.a0 - h1)
                        * (h2_acc1 + h1 * (4.0 * self.a0 - a_min + 2.0 * h1));
                    delta_t = orig / deriv;
                    t -= delta_t;

                    h1 = j_max * t;
                    orig = -(h0_acc1 / 2.0
                        + h1 * (h5
                        + self.a0 * (a_min - 2.0 * h1) * (a_min - h1)
                        + self.a0_a0 * (5.0 * h1 / 2.0 - 2.0 * a_min)
                        + a_min * a_min * h1 / 2.0
                        + j_max * (h1 / 2.0 - a_min) * (h1 * t + 2.0 * self.v0)))
                        / j_max;

                    if orig.abs() > 1e-9 {
                        deriv = (a_min - self.a0 - h1)
                            * (h2_acc1 + h1 * (4.0 * self.a0 - a_min + 2.0 * h1));
                        delta_t = orig / deriv;
                        t -= delta_t;
                    }
                }
            }
            let profile = &mut self.valid_profiles[self.current_index];
            profile.t[0] = t;
            profile.t[1] = 0.0;
            profile.t[2] = (self.a0 - a_min) / j_max + t;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = h3_acc1 - (2.0 * self.a0 + j_max * t) * t / a_min;
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
                self.add_profile();
                if return_after_found {
                    return;
                }
            }
        }
    }

    fn time_acc1_vel_two_step(
        &mut self,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) {
        let profile = &mut self.valid_profiles[self.current_index];
        profile.t[0] = 0.0;
        profile.t[1] = 0.0;
        profile.t[2] = self.a0 / j_max;
        profile.t[3] = -(3.0 * self.af_p4
            - 8.0 * a_min * (self.af_p3 - self.a0_p3)
            - 24.0 * a_min * j_max * (self.a0 * self.v0 - self.af * self.vf)
            + 6.0 * self.af_af * (a_min * a_min - 2.0 * j_max * self.vf)
            - 12.0
            * j_max
            * (2. * a_min * j_max * self.pd
            + a_min * a_min * (self.vf + v_max)
            + j_max * (v_max * v_max - self.vf_vf)
            + a_min * self.a0 * (self.a0_a0 - 2.0 * j_max * (self.v0 + v_max)) / j_max))
            / (24.0 * a_min * self.j_max_j_max * v_max);
        profile.t[4] = -a_min / j_max;
        profile.t[5] =
            -(self.af_af / 2.0 - a_min * a_min + j_max * (v_max - self.vf)) / (a_min * j_max);
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
            self.add_profile();
        }
    }

    fn time_acc0_two_step(&mut self, v_max: f64, v_min: f64, a_max: f64, a_min: f64, j_max: f64) {
        {
            // Two step
            let profile = &mut self.valid_profiles[self.current_index];
            profile.t[0] = 0.0;
            profile.t[1] = (self.af_af - self.a0_a0 + 2.0 * j_max * (self.vf - self.v0))
                / (2.0 * self.a0 * j_max);
            profile.t[2] = (self.a0 - self.af) / j_max;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
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
                self.add_profile();
                return;
            }
        }

        // Three step - Removed pf
        {
            let profile = &mut self.valid_profiles[self.current_index];
            profile.t[0] = (-self.a0 + a_max) / j_max;
            profile.t[1] = (self.a0_a0 + self.af_af - 2.0 * a_max * a_max
                + 2.0 * j_max * (self.vf - self.v0))
                / (2.0 * a_max * j_max);
            profile.t[2] = (-self.af + a_max) / j_max;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
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
                self.add_profile();
                return;
            }
        }

        // Three step - Removed a_max
        {
            let profile = &mut self.valid_profiles[self.current_index];
            let h0 = 3.0 * (self.af_af - self.a0_a0 + 2.0 * j_max * (self.v0 + self.vf));
            let h2 = self.a0_p3
                + 2.0 * self.af_p3
                + 6.0 * self.j_max_j_max * self.pd
                + 6.0 * (self.af - self.a0) * j_max * self.vf
                - 3.0 * self.a0 * self.af_af;
            let h1 = f64::sqrt(
                2.0 * (2.0 * h2 * h2
                    + h0 * (self.a0_p4 - 6.0 * self.a0_a0 * (self.af_af + 2.0 * j_max * self.vf)
                    + 8.0
                    * self.a0
                    * (self.af_p3
                    + 3.0 * self.j_max_j_max * self.pd
                    + 3.0 * self.af * j_max * self.vf)
                    - 3.0
                    * (self.af_p4
                    + 4.0 * self.af_af * j_max * self.vf
                    + 4.0 * self.j_max_j_max * (self.vf_vf - self.v0_v0)))),
            ) * f64::abs(j_max)
                / j_max;
            profile.t[0] = (4.0 * self.af_p3 + 2.0 * self.a0_p3 - 6.0 * self.a0 * self.af_af
                + 12.0 * self.j_max_j_max * self.pd
                + 12.0 * (self.af - self.a0) * j_max * self.vf
                + h1)
                / (2.0 * j_max * h0);
            profile.t[1] = -h1 / (j_max * h0);
            profile.t[2] = (-4.0 * self.a0_p3 - 2.0 * self.af_p3
                + 6.0 * self.a0_a0 * self.af
                + 12.0 * self.j_max_j_max * self.pd
                - 12.0 * (self.af - self.a0) * j_max * self.v0
                + h1)
                / (2.0 * j_max * h0);
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
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
                self.add_profile();
                return;
            }
        }

        // Three step - t=(a_max - a_min)/j_max
        {
            let t = (a_max - a_min) / j_max;
            let profile = &mut self.valid_profiles[self.current_index];
            profile.t[0] = (-self.a0 + a_max) / j_max;
            profile.t[1] = (self.a0_a0 - self.af_af) / (2.0 * a_max * j_max)
                + (self.vf - self.v0 + j_max * t * t) / a_max
                - 2.0 * t;
            profile.t[2] = t;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = (self.af - a_min) / j_max;

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::Acc0,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                self.add_profile();
            }
        }
    }

    fn time_vel_two_step(&mut self, v_max: f64, v_min: f64, a_max: f64, a_min: f64, j_max: f64) {
        let h1 = f64::sqrt(self.af_af / (2.0 * self.j_max_j_max) + (v_max - self.vf) / j_max);
        // Four step
        {
            // Solution 3/4
            let profile = &mut self.valid_profiles[self.current_index];
            profile.t[0] = -self.a0 / j_max;
            profile.t[1] = 0.0;
            profile.t[2] = 0.0;
            profile.t[3] = (self.af_p3 - self.a0_p3) / (3.0 * self.j_max_j_max * v_max)
                + (self.a0 * self.v0 - self.af * self.vf + (self.af_af * h1) / 2.0)
                / (j_max * v_max)
                - (self.vf / v_max + 1.0) * h1
                + self.pd / v_max;
            profile.t[4] = h1;
            profile.t[5] = 0.0;
            profile.t[6] = h1 + self.af / j_max;

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::Vel,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                self.add_profile();
                return;
            }
        }

        // Four step
        {
            let profile = &mut self.valid_profiles[self.current_index];
            profile.t[0] = 0.0;
            profile.t[1] = 0.0;
            profile.t[2] = self.a0 / j_max;
            profile.t[3] = (self.af_p3 - self.a0_p3) / (3.0 * self.j_max_j_max * v_max)
                + (self.a0 * self.v0 - self.af * self.vf
                + (self.af_af * h1 + self.a0_p3 / j_max) / 2.0)
                / (j_max * v_max)
                - (self.v0 / v_max + 1.0) * self.a0 / j_max
                - (self.vf / v_max + 1.0) * h1
                + self.pd / v_max;
            profile.t[4] = h1;
            profile.t[5] = 0.0;
            profile.t[6] = h1 + self.af / j_max;

            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::Vel,
                j_max,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                self.add_profile();
            }
        }
    }

    fn time_none_two_step(&mut self, v_max: f64, v_min: f64, a_max: f64, a_min: f64, j_max: f64) {
        // Two step
        {
            let profile = &mut self.valid_profiles[self.current_index];
            let h0 = f64::sqrt((self.a0_a0 + self.af_af) / 2.0 + j_max * (self.vf - self.v0))
                * f64::abs(j_max)
                / j_max;
            profile.t[0] = (h0 - self.a0) / j_max;
            profile.t[1] = 0.0;
            profile.t[2] = (h0 - self.af) / j_max;
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
                self.add_profile();
                return;
            }
        }

        // Single step
        {
            let profile = &mut self.valid_profiles[self.current_index];
            profile.t[0] = (self.af - self.a0) / j_max;
            profile.t[1] = 0.0;
            profile.t[2] = 0.0;
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
                self.add_profile();
            }
        }
    }

    fn time_all_single_step(
        &mut self,
        profile: &mut Profile,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        _: f64,
    ) -> bool {
        if f64::abs(self.af - self.a0) > f64::EPSILON {
            return false;
        }

        profile.t[0] = 0.0;
        profile.t[1] = 0.0;
        profile.t[2] = 0.0;
        profile.t[3] = 0.0;
        profile.t[4] = 0.0;
        profile.t[5] = 0.0;
        profile.t[6] = 0.0;

        if f64::abs(self.a0) > f64::EPSILON {
            let q = f64::sqrt(2.0 * self.a0 * self.pd + self.v0_v0);

            // Solution 1
            profile.t[3] = (-self.v0 + q) / self.a0;
            if profile.t[3] >= 0.0
                && profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::None,
                0.0,
                v_max,
                v_min,
                a_max,
                a_min,
            )
            {
                return true;
            }

            // Solution 2
            profile.t[3] = -(self.v0 + q) / self.a0;
            if profile.t[3] >= 0.0
                && profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::None,
                0.0,
                v_max,
                v_min,
                a_max,
                a_min,
            )
            {
                return true;
            }
        } else if f64::abs(self.v0) > f64::EPSILON {
            profile.t[3] = self.pd / self.v0;
            if profile.check_with_timing(
                ControlSigns::UDDU,
                ReachedLimits::None,
                0.0,
                v_max,
                v_min,
                a_max,
                a_min,
            ) {
                return true;
            }
        } else if f64::abs(self.pd) < f64::EPSILON && profile.check_with_timing(
            ControlSigns::UDDU,
            ReachedLimits::None,
            0.0,
            v_max,
            v_min,
            a_max,
            a_min,
        ) {
            return true;
        }

        false
    }

    pub fn get_profile(&mut self, input: &Profile, block: &mut Block) -> bool {
        // Zero-limits special case
        if self._j_max == 0.0 || self._a_max == 0.0 || self._a_min == 0.0 {
            let p = &mut block.p_min;
            p.set_boundary_from_profile(input);

            if self.time_all_single_step(
                p,
                self._v_max,
                self._v_min,
                self._a_max,
                self._a_min,
                self._j_max,
            ) {
                // [p.t_sum.len() - 1] instead of C++ back()
                block.t_min = p.t_sum[p.t_sum.len() - 1] + p.brake.duration + p.accel.duration;
                if f64::abs(self.v0) > f64::EPSILON || f64::abs(self.a0) > f64::EPSILON
                {
                    block.a = Some(Interval::new(block.t_min, f64::INFINITY));
                }
                return true;
            }
            return false;
        }

        self.valid_profiles[0].set_boundary_from_profile(input);
        self.current_index = 0;

        if f64::abs(self.vf) < f64::EPSILON && f64::abs(self.af) < f64::EPSILON {
            let v_max = if self.pd >= 0.0 {
                self._v_max
            } else {
                self._v_min
            };
            let v_min = if self.pd >= 0.0 {
                self._v_min
            } else {
                self._v_max
            };
            let a_max = if self.pd >= 0.0 {
                self._a_max
            } else {
                self._a_min
            };
            let a_min = if self.pd >= 0.0 {
                self._a_min
            } else {
                self._a_max
            };
            let j_max = if self.pd >= 0.0 {
                self._j_max
            } else {
                -self._j_max
            };

            if f64::abs(self.v0) < f64::EPSILON
                && f64::abs(self.a0) < f64::EPSILON
                && f64::abs(self.pd) < f64::EPSILON
            {
                self.time_all_none_acc0_acc1(v_max, v_min, a_max, a_min, j_max, true);
            } else {
                // There is no blocked interval when vf==0 && af==0, so return after first found profile
                self.time_all_vel(v_max, v_min, a_max, a_min, j_max, true);
                if self.current_index > 0 {
                    return Block::calculate_block(
                        block,
                        &mut self.valid_profiles,
                        &mut self.current_index,
                        None,
                    );
                }
                self.time_all_none_acc0_acc1(v_max, v_min, a_max, a_min, j_max, true);
                if self.current_index > 0 {
                    return Block::calculate_block(
                        block,
                        &mut self.valid_profiles,
                        &mut self.current_index,
                        None,
                    );
                }
                self.time_acc0_acc1(v_max, v_min, a_max, a_min, j_max, true);
                if self.current_index > 0 {
                    return Block::calculate_block(
                        block,
                        &mut self.valid_profiles,
                        &mut self.current_index,
                        None,
                    );
                }

                self.time_all_vel(v_min, v_max, a_min, a_max, -j_max, true);
                if self.current_index > 0 {
                    return Block::calculate_block(
                        block,
                        &mut self.valid_profiles,
                        &mut self.current_index,
                        None,
                    );
                }
                self.time_all_none_acc0_acc1(v_min, v_max, a_min, a_max, -j_max, true);
                if self.current_index > 0 {
                    return Block::calculate_block(
                        block,
                        &mut self.valid_profiles,
                        &mut self.current_index,
                        None,
                    );
                }
                self.time_acc0_acc1(v_min, v_max, a_min, a_max, -j_max, true);
            };
        } else {
            self.time_all_none_acc0_acc1(
                self._v_max,
                self._v_min,
                self._a_max,
                self._a_min,
                self._j_max,
                false,
            );
            self.time_all_none_acc0_acc1(
                self._v_min,
                self._v_max,
                self._a_min,
                self._a_max,
                -self._j_max,
                false,
            );
            self.time_acc0_acc1(
                self._v_max,
                self._v_min,
                self._a_max,
                self._a_min,
                self._j_max,
                false,
            );
            self.time_acc0_acc1(
                self._v_min,
                self._v_max,
                self._a_min,
                self._a_max,
                -self._j_max,
                false,
            );
            self.time_all_vel(
                self._v_max,
                self._v_min,
                self._a_max,
                self._a_min,
                self._j_max,
                false,
            );
            self.time_all_vel(
                self._v_min,
                self._v_max,
                self._a_min,
                self._a_max,
                -self._j_max,
                false,
            );
        }

        if self.current_index == 0 {
            self.time_none_two_step(
                self._v_max,
                self._v_min,
                self._a_max,
                self._a_min,
                self._j_max,
            );
            if self.current_index > 0 {
                return Block::calculate_block(
                    block,
                    &mut self.valid_profiles,
                    &mut self.current_index,
                    None,
                );
            }
            self.time_none_two_step(
                self._v_min,
                self._v_max,
                self._a_min,
                self._a_max,
                -self._j_max,
            );
            if self.current_index > 0 {
                return Block::calculate_block(
                    block,
                    &mut self.valid_profiles,
                    &mut self.current_index,
                    None,
                );
            }
            self.time_acc0_two_step(
                self._v_max,
                self._v_min,
                self._a_max,
                self._a_min,
                self._j_max,
            );
            if self.current_index > 0 {
                return Block::calculate_block(
                    block,
                    &mut self.valid_profiles,
                    &mut self.current_index,
                    None,
                );
            }
            self.time_acc0_two_step(
                self._v_min,
                self._v_max,
                self._a_min,
                self._a_max,
                -self._j_max,
            );
            if self.current_index > 0 {
                return Block::calculate_block(
                    block,
                    &mut self.valid_profiles,
                    &mut self.current_index,
                    None,
                );
            }
            self.time_vel_two_step(
                self._v_max,
                self._v_min,
                self._a_max,
                self._a_min,
                self._j_max,
            );
            if self.current_index > 0 {
                return Block::calculate_block(
                    block,
                    &mut self.valid_profiles,
                    &mut self.current_index,
                    None,
                );
            }
            self.time_vel_two_step(
                self._v_min,
                self._v_max,
                self._a_min,
                self._a_max,
                -self._j_max,
            );
            if self.current_index > 0 {
                return Block::calculate_block(
                    block,
                    &mut self.valid_profiles,
                    &mut self.current_index,
                    None,
                );
            }
            self.time_acc1_vel_two_step(
                self._v_max,
                self._v_min,
                self._a_max,
                self._a_min,
                self._j_max,
            );
            if self.current_index > 0 {
                return Block::calculate_block(
                    block,
                    &mut self.valid_profiles,
                    &mut self.current_index,
                    None,
                );
            }
            self.time_acc1_vel_two_step(
                self._v_min,
                self._v_max,
                self._a_min,
                self._a_max,
                -self._j_max,
            );
        }

        Block::calculate_block(
            block,
            &mut self.valid_profiles,
            &mut self.current_index,
            None,
        )
    }
}
