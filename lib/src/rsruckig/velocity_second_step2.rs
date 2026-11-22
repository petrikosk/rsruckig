//! Mathematical equations for Step 2 in second-order velocity interface: Time synchronization

use crate::profile::{ControlSigns, Profile, ReachedLimits};

/// Step 2 solver for the second-order velocity interface: calculates a
/// constant-acceleration profile with the synchronized duration `tf`.
#[derive(Debug)]
pub struct VelocitySecondOrderStep2 {
    tf: f64,
    _a_max: f64,
    _a_min: f64,
    vd: f64,
}

impl VelocitySecondOrderStep2 {
    /// Create a solver for changing the velocity from `v0` to `vf` in duration
    /// `tf` within the acceleration limits `[a_min, a_max]`.
    pub fn new(tf: f64, v0: f64, vf: f64, a_max: f64, a_min: f64) -> Self {
        Self {
            tf,
            _a_max: a_max,
            _a_min: a_min,
            vd: vf - v0,
        }
    }

    /// Calculate a profile that reaches the target in exactly the duration
    /// `tf` given to [`Self::new`].
    ///
    /// The phase durations are written into `profile`, whose boundary state
    /// and pre-trajectories must already be set. Returns `false` if no valid
    /// profile with that duration exists within the limits.
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
            profile.pf = profile.p[7];
            return true;
        }

        false
    }
}
