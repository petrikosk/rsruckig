//! Mathematical equations for Step 2 in second-order velocity interface: Time synchronization

use crate::{
    profile::{ControlSigns, Profile, ReachedLimits},
};

pub struct VelocitySecondOrderStep2 {
    tf: f64,
    _a_max: f64,
    _a_min: f64,
    vd: f64,
}

impl VelocitySecondOrderStep2 {
    pub fn new(tf: f64, v0: f64, vf: f64, a_max: f64, a_min: f64) -> Self {
        Self {
            tf,
            _a_max: a_max,
            _a_min: a_min,
            vd: vf - v0,
        }
    }

    pub fn get_profile(&mut self, profile: &mut Profile) -> bool {
        let af = self.vd / self.tf;
        profile.t[0] = 0.0;
        profile.t[1] = self.tf;
        profile.t[2] = 0.0;
        profile.t[3] = 0.0;
        profile.t[4] = 0.0;
        profile.t[5] = 0.0;
        profile.t[6] = 0.0;

        if profile.check_for_second_order_velocity_with_timing_a_limits(
            ControlSigns::UDDU,
            ReachedLimits::Acc0,
            self.tf,
            af,
            self._a_max,
            self._a_min,
        ) {
            profile.pf = *profile.p.last().unwrap();
            return true;
        }

        false
    }
}
