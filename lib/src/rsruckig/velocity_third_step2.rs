//! Mathematical equations for Step 2 in third-order velocity interface: Time synchronization

use crate::profile::{ControlSigns, Profile, ReachedLimits};

#[cfg(not(feature = "std"))]
use num_traits::Float;

pub struct VelocityThirdOrderStep2 {
    a0: f64,
    tf: f64,
    af: f64,
    _a_max: f64,
    _a_min: f64,
    _j_max: f64,
    vd: f64,
    ad: f64,
}

impl VelocityThirdOrderStep2 {
    pub fn new(
        tf: f64,
        v0: f64,
        a0: f64,
        vf: f64,
        af: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) -> Self {
        Self {
            a0,
            tf,
            af,
            _a_max: a_max,
            _a_min: a_min,
            _j_max: j_max,
            vd: vf - v0,
            ad: af - a0,
        }
    }

    fn time_acc0(&mut self, profile: &mut Profile, a_max: f64, a_min: f64, j_max: f64) -> bool {
        // UD Solution 1/2
        {
            let h1 = f64::sqrt(
                (-self.ad * self.ad
                    + 2.0 * j_max * ((self.a0 + self.af) * self.tf - 2.0 * self.vd))
                    / (j_max * j_max)
                    + self.tf * self.tf,
            );

            profile.t[0] = self.ad / (2.0 * j_max) + (self.tf - h1) / 2.0;
            profile.t[1] = h1;
            profile.t[2] = self.tf - (profile.t[0] + h1);
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = 0.0;

            if profile.check_for_velocity(
                ControlSigns::UDDU,
                ReachedLimits::Acc0,
                j_max,
                a_max,
                a_min,
            ) {
                profile.pf = *profile.p.last().unwrap();
                return true;
            }
        }

        // UU Solution
        {
            let h1 = -self.ad + j_max * self.tf;

            profile.t[0] =
                -self.ad * self.ad / (2.0 * j_max * h1) + (self.vd - self.a0 * self.tf) / h1;
            profile.t[1] = -self.ad / j_max + self.tf;
            profile.t[2] = 0.0;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = self.tf - (profile.t[0] + profile.t[1]);

            if profile.check_for_velocity(
                ControlSigns::UDDU,
                ReachedLimits::Acc0,
                j_max,
                a_max,
                a_min,
            ) {
                profile.pf = *profile.p.last().unwrap();
                return true;
            }
        }

        // UU Solution - 2 step
        {
            profile.t[0] = 0.0;
            profile.t[1] = -self.ad / j_max + self.tf;
            profile.t[2] = 0.0;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = self.ad / j_max;

            if profile.check_for_velocity(
                ControlSigns::UDDU,
                ReachedLimits::Acc0,
                j_max,
                a_max,
                a_min,
            ) {
                profile.pf = *profile.p.last().unwrap();
                return true;
            }
        }

        false
    }

    fn time_none(&mut self, profile: &mut Profile, a_max: f64, a_min: f64, j_max: f64) -> bool {
        if f64::abs(self.a0) < f64::EPSILON
            && f64::abs(self.af) < f64::EPSILON
            && f64::abs(self.vd) < f64::EPSILON
        {
            profile.t[0] = 0.0;
            profile.t[1] = self.tf;
            profile.t[2] = 0.0;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = 0.0;

            if profile.check_for_velocity(
                ControlSigns::UDDU,
                ReachedLimits::None,
                j_max,
                a_max,
                a_min,
            ) {
                profile.pf = *profile.p.last().unwrap();
                return true;
            }
        }

        // UD Solution 1/2
        {
            let h1 = 2.0 * (self.af * self.tf - self.vd);

            profile.t[0] = h1 / self.ad;
            profile.t[1] = self.tf - profile.t[0];
            profile.t[2] = 0.0;
            profile.t[3] = 0.0;
            profile.t[4] = 0.0;
            profile.t[5] = 0.0;
            profile.t[6] = 0.0;

            let jf = self.ad * self.ad / h1;

            if profile.check_for_velocity(
                ControlSigns::UDDU,
                ReachedLimits::None,
                jf,
                a_max,
                a_min,
            ) {
                profile.pf = *profile.p.last().unwrap();
                return true;
            }
        }

        false
    }

    fn check_all(&mut self, profile: &mut Profile, a_max: f64, a_min: f64, j_max: f64) -> bool {
        self.time_acc0(profile, a_max, a_min, j_max)
            || self.time_none(profile, a_max, a_min, j_max)
    }

    pub fn get_profile(&mut self, profile: &mut Profile) -> bool {
        // Test all cases to get ones that match
        // However we should guess which one is correct and try them first...
        if self.vd > 0.0 {
            return self.check_all(profile, self._a_max, self._a_min, self._j_max)
                || self.check_all(profile, self._a_min, self._a_max, -self._j_max);
        }

        self.check_all(profile, self._a_min, self._a_max, -self._j_max)
            || self.check_all(profile, self._a_max, self._a_min, self._j_max)
    }
}
