//! Mathematical equations for Step 2 in second-order position interface: Time synchronization
use crate::profile::{ControlSigns, Profile, ReachedLimits};

use num_traits::Float;

#[derive(Debug)]
pub struct PositionSecondOrderStep2 {
    v0: f64,
    tf: f64,
    vf: f64,
    _v_max: f64,
    _v_min: f64,
    _a_max: f64,
    _a_min: f64,
    pd: f64,
    vd: f64,
}

impl PositionSecondOrderStep2 {
    /// Create a new instance of `PositionSecondOrderStep2`
    pub fn new(
        tf: f64,
        p0: f64,
        v0: f64,
        pf: f64,
        vf: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
    ) -> Self {
        Self {
            v0,
            tf,
            vf,
            _v_max: v_max,
            _v_min: v_min,
            _a_max: a_max,
            _a_min: a_min,
            pd: pf - p0,
            vd: vf - v0,
        }
    }

    fn time_acc0(
        &mut self,
        profile: &mut Profile,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        _: bool,
    ) -> bool {
        // UD Solution 1/2
        {
            let h1 = f64::sqrt(
                (2.0 * a_max * (self.pd - self.tf * self.vf)
                    - 2.0 * a_min * (self.pd - self.tf * self.v0)
                    + self.vd * self.vd)
                    / (a_max * a_min)
                    + self.tf * self.tf,
            );

            profile.t[0] =
                (a_max * self.vd - a_max * a_min * (self.tf - h1)) / (a_max * (a_max - a_min));
            profile.t[1] = h1;
            profile.t[2] = self.tf - (profile.t[0] + h1);
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = 0.0;

            if profile.check_for_second_order(
                ControlSigns::UDDU,
                ReachedLimits::Acc0,
                a_max,
                a_min,
                v_max,
                v_min,
            ) {
                profile.pf = *profile.p.last().unwrap();
                return true;
            }
        }

        // UU Solution
        {
            let h1 = -self.vd + a_max * self.tf;

            profile.t[0] =
                -self.vd * self.vd / (2.0 * a_max * h1) + (self.pd - self.v0 * self.tf) / h1;
            profile.t[1] = -self.vd / a_max + self.tf;
            profile.t[2] = 0.0;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = self.tf - (profile.t[0] + profile.t[1]);

            if profile.check_for_second_order(
                ControlSigns::UDDU,
                ReachedLimits::Acc0,
                a_max,
                a_min,
                v_max,
                v_min,
            ) {
                profile.pf = *profile.p.last().unwrap();
                return true;
            }
        }

        // UU Solution - 2 step
        {
            profile.t[0] = 0.0;
            profile.t[1] = -self.vd / a_max + self.tf;
            profile.t[2] = 0.0;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = self.vd / a_max;

            if profile.check_for_second_order(
                ControlSigns::UDDU,
                ReachedLimits::Acc0,
                a_max,
                a_min,
                v_max,
                v_min,
            ) {
                profile.pf = *profile.p.last().unwrap();
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
        _: bool,
    ) -> bool {
        if f64::abs(self.v0) < f64::EPSILON
            && f64::abs(self.vf) < f64::EPSILON
            && f64::abs(self.pd) < f64::EPSILON
        {
            profile.t[0] = 0.0;
            profile.t[1] = self.tf;
            profile.t[2] = 0.0;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = 0.0;

            if profile.check_for_second_order(
                ControlSigns::UDDU,
                ReachedLimits::None,
                a_max,
                a_min,
                v_max,
                v_min,
            ) {
                profile.pf = *profile.p.last().unwrap();
                return true;
            }
        }

        // UD Solution 1/2
        {
            let h1 = 2.0 * (self.vf * self.tf - self.pd);

            profile.t[0] = h1 / self.vd;
            profile.t[1] = self.tf - profile.t[0];
            profile.t[2] = 0.0;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = 0.0;

            let af = self.vd * self.vd / h1;

            if (a_min - 1e-12 < af)
                && (af < a_max + 1e-12)
                && profile.check_for_second_order(
                    ControlSigns::UDDU,
                    ReachedLimits::None,
                    af,
                    -af,
                    v_max,
                    v_min,
                )
            {
                profile.pf = *profile.p.last().unwrap();
                return true;
            }
        }

        false
    }

    #[inline]
    fn check_all(
        &mut self,
        profile: &mut Profile,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
    ) -> bool {
        self.time_acc0(profile, v_max, v_min, a_max, a_min, false)
            || self.time_none(profile, v_max, v_min, a_max, a_min, false)
    }

    pub fn get_profile(&mut self, profile: &mut Profile) -> bool {
        // Test all cases to get ones that match
        // However we should guess which one is correct and try them first...
        if self.pd > 0.0 {
            return self.check_all(profile, self._v_max, self._v_min, self._a_max, self._a_min)
                || self.check_all(profile, self._v_min, self._v_max, self._a_min, self._a_max);
        }

        self.check_all(profile, self._v_min, self._v_max, self._a_min, self._a_max)
            || self.check_all(profile, self._v_max, self._v_min, self._a_max, self._a_min)
    }
}
