//! Mathematical equations for Step 1 in second-order velocity interface: Extremal profiles

use crate::{
    block::Block,
    profile::{ControlSigns, Profile, ReachedLimits},
};

/// Step 1 solver for the second-order velocity interface: calculates the
/// time-optimal profile, accelerating at the limit towards the target velocity.
#[derive(Debug)]
pub struct VelocitySecondOrderStep1 {
    _a_max: f64,
    _a_min: f64,
    vd: f64,
}

impl VelocitySecondOrderStep1 {
    /// Create a solver for changing the velocity from `v0` to `vf` within the
    /// acceleration limits `[a_min, a_max]`.
    pub fn new(v0: f64, vf: f64, a_max: f64, a_min: f64) -> Self {
        Self {
            _a_max: a_max,
            _a_min: a_min,
            vd: vf - v0,
        }
    }
    /// Calculate the time-optimal profile for this DoF.
    ///
    /// Writes the minimal-duration profile into `block.p_min` (with the brake
    /// and acceleration pre-trajectories and boundary state taken from
    /// `input`) and sets `block.t_min`. Returns `false` if no valid profile was
    /// found.
    pub fn get_profile(&mut self, input: &Profile, block: &mut Block) -> bool {
        let p = &mut block.p_min;
        p.set_boundary_from_profile(input);

        let af = if self.vd > 0.0 {
            self._a_max
        } else {
            self._a_min
        };
        p.t[0] = 0.0;
        p.t[1] = self.vd / af;
        p.t[2] = 0.0;
        p.t[3] = 0.0;
        p.t[4] = 0.0;
        p.t[5] = 0.0;
        p.t[6] = 0.0;

        if p.check_for_second_order_velocity(ControlSigns::UDDU, ReachedLimits::Acc0, af) {
            block.t_min = p.t_sum[6] + p.brake.duration + p.accel.duration;
            return true;
        }

        false
    }
}
