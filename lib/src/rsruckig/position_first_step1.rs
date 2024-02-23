//! Mathematical equations for Step 1 in first-order position interface: Extremal profiles
use crate::block::Block;
use crate::profile::{ControlSigns, Profile, ReachedLimits};

#[derive(Debug)]
pub struct PositionFirstOrderStep1 {
    _v_max: f64,
    _v_min: f64,
    pd: f64,
}

impl PositionFirstOrderStep1 {
    pub fn new(p0: f64, pf: f64, v_max: f64, v_min: f64) -> Self {
        Self {
            _v_max: v_max,
            _v_min: v_min,
            pd: pf - p0,
        }
    }
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
            block.t_min = p.t_sum.last().unwrap() + p.brake.duration + p.accel.duration;
            return true;
        }
        false
    }
}
