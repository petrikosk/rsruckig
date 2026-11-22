//! Mathematical equations for Step 2 in first-order position interface: Time synchronization
use crate::profile::{ControlSigns, Profile, ReachedLimits};

/// Step 2 solver for the first-order position interface: calculates a
/// constant-velocity profile with the synchronized duration `tf`.
#[derive(Debug)]
pub struct PositionFirstOrderStep2 {
    tf: f64,
    _v_max: f64,
    _v_min: f64,
    pd: f64,
}

impl PositionFirstOrderStep2 {
    /// Create a solver for moving from position `p0` to `pf` in duration `tf`
    /// within the velocity limits `[v_min, v_max]`.
    pub fn new(tf: f64, p0: f64, pf: f64, v_max: f64, v_min: f64) -> Self {
        Self {
            tf,
            _v_max: v_max,
            _v_min: v_min,
            pd: pf - p0,
        }
    }

    /// Calculate a profile that reaches the target in exactly the duration
    /// `tf` given to [`Self::new`].
    ///
    /// The phase durations are written into `profile`, whose boundary state
    /// and pre-trajectories must already be set. Returns `false` if no valid
    /// profile with that duration exists within the limits.
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
