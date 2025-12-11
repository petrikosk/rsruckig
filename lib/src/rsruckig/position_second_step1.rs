//! Mathematical equations for Step 1 in second-order position interface: Extremal profiles
use crate::{
    block::{Block, Interval},
    profile::{ControlSigns, Profile, ReachedLimits},
};

#[cfg(not(feature = "std"))]
use num_traits::Float;

#[derive(Debug)]
pub struct PositionSecondOrderStep1 {
    v0: f64,
    vf: f64,
    _v_max: f64,
    _v_min: f64,
    _a_max: f64,
    _a_min: f64,
    pd: f64,
    valid_profiles: [Profile; 6],
    current_index: usize,
}

impl PositionSecondOrderStep1 {
    /// Create a new instance of `PositionSecondOrderStep2`
    pub fn new(
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
            vf,
            _v_max: v_max,
            _v_min: v_min,
            _a_max: a_max,
            _a_min: a_min,
            pd: pf - p0,
            valid_profiles: Default::default(),
            current_index: 0,
        }
    }

    #[inline]
    fn add_profile(&mut self, profile: &mut Profile) {
        if self.current_index < 3 {
            self.valid_profiles[self.current_index] = profile.clone();
            self.current_index += 1;
            self.valid_profiles[self.current_index].set_boundary_from_profile(profile);
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
    ) {
        profile.t[0] = (-self.v0 + v_max) / a_max;
        profile.t[1] = (a_min * self.v0 * self.v0 - a_max * self.vf * self.vf)
            / (2.0 * a_max * a_min * v_max)
            + v_max * (a_max - a_min) / (2.0 * a_max * a_min)
            + self.pd / v_max;
        profile.t[2] = (self.vf - v_max) / a_min;
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
            self.add_profile(profile);
        }
    }

    fn time_none(
        &mut self,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        return_after_found: bool,
    ) {
        let mut h1 =
            (a_max * self.vf * self.vf - a_min * self.v0 * self.v0 - 2.0 * a_max * a_min * self.pd)
                / (a_max - a_min);
        if h1 >= 0.0 {
            h1 = f64::sqrt(h1);

            // Solution 1
            {
                let mut profile = self.valid_profiles[self.current_index].clone();
                profile.t[0] = -(self.v0 + h1) / a_max;
                profile.t[1] = 0.0;
                profile.t[2] = (self.vf + h1) / a_min;
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
                    self.add_profile(&mut profile);
                    if return_after_found {
                        return;
                    }
                }
            }

            // Solution 2
            {
                let mut profile = self.valid_profiles[self.current_index].clone();
                profile.t[0] = (-self.v0 + h1) / a_max;
                profile.t[1] = 0.0;
                profile.t[2] = (self.vf - h1) / a_min;
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
                    self.add_profile(&mut profile);
                }
            }
        }
    }
    fn time_all_single_step(
        &mut self,
        profile: &mut Profile,
        v_max: f64,
        v_min: f64,
        _: f64,
        _: f64,
    ) -> bool {
        if f64::abs(self.vf - self.v0) > f64::EPSILON {
            return false;
        }

        profile.t[0] = 0.0;
        profile.t[1] = 0.0;
        profile.t[2] = 0.0;
        profile.t[3] = 0.0;
        profile.t[4] = 0.0;
        profile.t[5] = 0.0;
        profile.t[6] = 0.0;

        if profile.check_for_second_order(
            ControlSigns::UDDU,
            ReachedLimits::None,
            0.0,
            0.0,
            v_max,
            v_min,
        ) {
            return true;
        }

        if f64::abs(self.v0) > f64::EPSILON {
            profile.t[3] = self.pd / self.v0;
            if profile.check_for_second_order(
                ControlSigns::UDDU,
                ReachedLimits::None,
                0.0,
                0.0,
                v_max,
                v_min,
            ) {
                return true;
            }
        } else if f64::abs(self.pd) < f64::EPSILON
            && profile.check_for_second_order(
                ControlSigns::UDDU,
                ReachedLimits::None,
                0.0,
                0.0,
                v_max,
                v_min,
            )
        {
            return true;
        }

        false
    }

    pub fn get_profile(&mut self, input: &Profile, block: &mut Block) -> bool {
        // Zero-limits special case
        if self._v_max == 0.0 && self._v_min == 0.0 {
            let p = &mut block.p_min;
            p.set_boundary_from_profile(input);

            if self.time_all_single_step(p, self._v_max, self._v_min, self._a_max, self._a_min) {
                block.t_min = p.t_sum[6] + p.brake.duration + p.accel.duration;
                if f64::abs(self.v0) > f64::EPSILON {
                    block.a = Some(Interval::new(block.t_min, f64::INFINITY));
                }
                return true;
            }
            return false;
        }

        self.valid_profiles[0].set_boundary_from_profile(input);
        self.current_index = 0;
        let mut profile = self.valid_profiles[0].clone();

        if f64::abs(self.vf) < f64::EPSILON {
            // There is no blocked interval when self.vf==0.0, so return after first found profile
            let v_max = if self.pd >= 0.0 {
                self._v_max
            } else {
                self._v_min
            };
            // below lines to be converted to match rust
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

            self.time_none(v_max, v_min, a_max, a_min, true);
            if self.current_index > 0 {
                return Block::calculate_block(
                    block,
                    &mut self.valid_profiles,
                    &mut self.current_index,
                    None,
                );
            }
            self.time_acc0(&mut profile, v_max, v_min, a_max, a_min, true);
            if self.current_index > 0 {
                return Block::calculate_block(
                    block,
                    &mut self.valid_profiles,
                    &mut self.current_index,
                    None,
                );
            }

            self.time_none(v_min, v_max, a_min, a_max, true);
            if self.current_index > 0 {
                return Block::calculate_block(
                    block,
                    &mut self.valid_profiles,
                    &mut self.current_index,
                    None,
                );
            }
            self.time_acc0(&mut profile, v_min, v_max, a_min, a_max, true);
        } else {
            self.time_none(self._v_max, self._v_min, self._a_max, self._a_min, false);
            self.time_none(self._v_min, self._v_max, self._a_min, self._a_max, false);
            self.time_acc0(
                &mut profile,
                self._v_max,
                self._v_min,
                self._a_max,
                self._a_min,
                false,
            );
            self.time_acc0(
                &mut profile,
                self._v_min,
                self._v_max,
                self._a_min,
                self._a_max,
                false,
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
