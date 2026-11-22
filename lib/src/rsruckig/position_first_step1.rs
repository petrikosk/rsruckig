//! Mathematical equations for Step 1 in first-order position interface: Extremal profiles
use crate::block::Block;
use crate::profile::{ControlSigns, Profile, ReachedLimits};

/// Step 1 solver for the first-order position interface: calculates the
/// time-optimal profile, moving at the velocity limit in the direction of the
/// target.
#[derive(Debug)]
pub struct PositionFirstOrderStep1 {
    _v_max: f64,
    _v_min: f64,
    pd: f64,
}

impl PositionFirstOrderStep1 {
    /// Create a solver for moving from position `p0` to `pf` within the
    /// velocity limits `[v_min, v_max]`.
    pub fn new(p0: f64, pf: f64, v_max: f64, v_min: f64) -> Self {
        Self {
            _v_max: v_max,
            _v_min: v_min,
            pd: pf - p0,
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

        let vf = if self.pd > 0.0 {
            self._v_max
        } else {
            self._v_min
        };
        p.t[0] = 0.0;
        p.t[1] = 0.0;
        p.t[2] = 0.0;
        p.t[3] = self.pd / vf;
        p.t[4] = 0.0;
        p.t[5] = 0.0;
        p.t[6] = 0.0;

        if p.check_for_first_order(vf, ControlSigns::UDDU, ReachedLimits::Vel) {
            block.t_min = p.t_sum[6] + p.brake.duration + p.accel.duration;
            return true;
        }
        false
    }
}
