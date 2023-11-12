//! Mathematical equations for Step 2 in first-order position interface: Time synchronization
use crate::profile::{ControlSigns, Profile, ReachedLimits};

pub struct PositionFirstOrderStep2 {
    tf: f64,
    _v_max: f64,
    _v_min: f64,
    pd: f64,
}

impl PositionFirstOrderStep2 {
    pub fn new(tf: f64, p0: f64, pf: f64, v_max: f64, v_min: f64) -> Self {
        Self {
            tf,
            _v_max: v_max,
            _v_min: v_min,
            pd: pf - p0,
        }
    }

    pub fn get_profile(&mut self, profile: &mut Profile) -> bool {
        let vf = self.pd / self.tf;

        profile.t[0] = 0.0;
        profile.t[1] = 0.0;
        profile.t[2] = 0.0;
        profile.t[3] = self.tf;
        profile.t[4] = 0.0;
        profile.t[5] = 0.0;
        profile.t[6] = 0.0;

        profile.check_for_first_order(vf, ControlSigns::UDDU, ReachedLimits::None)
    }
}
