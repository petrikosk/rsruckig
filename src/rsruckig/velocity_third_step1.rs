//! Mathematical equations for Step 1 in third-order velocity interface: Extremal profiles

use crate::{
    block::{self, Block, Interval},
    profile::{ControlSigns, Profile, ReachedLimits},
};

pub struct VelocityThirdOrderStep1 {
    a0: f64,
    af: f64,
    _a_max: f64,
    _a_min: f64,
    _j_max: f64,
    vd: f64,
    valid_profiles: [Profile; 6],
    current_index: usize,
}

impl VelocityThirdOrderStep1 {
    pub fn new(v0: f64, a0: f64, vf: f64, af: f64, a_max: f64, a_min: f64, j_max: f64) -> Self {
        Self {
            a0,
            af,
            _a_max: a_max,
            _a_min: a_min,
            _j_max: j_max,
            vd: vf - v0,
            valid_profiles: Default::default(),
            current_index: 0,
        }
    }

    #[inline]
    fn add_profile(&mut self) {
        if self.current_index < 5 {
            self.current_index += 1;
            let (left, right) = self.valid_profiles.split_at_mut(self.current_index);
            right[0].set_boundary_from_profile(&left[left.len() - 1]);
        }
    }

    fn time_acc0(&mut self, a_max: f64, a_min: f64, j_max: f64, _: bool) {
        let profile = &mut self.valid_profiles[self.current_index];
        profile.t[0] = (-self.a0 + a_max) / j_max;
        profile.t[1] = (self.a0 * self.a0 + self.af * self.af) / (2.0 * a_max * j_max)
            - a_max / j_max
            + self.vd / a_max;
        profile.t[2] = (-self.af + a_max) / j_max;
        profile.t[3] = 0.0;
        profile.t[4] = 0.0;
        profile.t[5] = 0.0;
        profile.t[6] = 0.0;

        if profile.check_for_velocity(ControlSigns::UDDU, ReachedLimits::Acc0, j_max, a_max, a_min)
        {
            self.add_profile();
        }
    }

    fn time_none(&mut self, a_max: f64, a_min: f64, j_max: f64, return_after_found: bool) {
        let mut h1 = (self.a0 * self.a0 + self.af * self.af) / 2.0 + j_max * self.vd;
        if h1 >= 0.0 {
            h1 = f64::sqrt(h1);

            // Solution 1
            {
                let profile = &mut self.valid_profiles[self.current_index];
                profile.t[0] = -(self.a0 + h1) / j_max;
                profile.t[1] = 0.0;
                profile.t[2] = -(self.af + h1) / j_max;
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
                    self.add_profile();
                    if return_after_found {
                        return;
                    }
                }
            }

            // Solution 2
            {
                let profile = &mut self.valid_profiles[self.current_index];
                profile.t[0] = (-self.a0 + h1) / j_max;
                profile.t[1] = 0.0;
                profile.t[2] = (-self.af + h1) / j_max;
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
                    self.add_profile();
                }
            }
        }
    }

    fn time_all_single_step(
        &mut self,
        profile: &mut Profile,
        a_max: f64,
        a_min: f64,
        _: f64,
    ) -> bool {
        if f64::abs(self.af - self.a0) > f64::EPSILON {
            return false;
        }

        profile.t[0] = 0.0;
        profile.t[1] = 0.0;
        profile.t[2] = 0.0;
        profile.t[3] = 0.0;
        profile.t[4] = 0.0;
        profile.t[5] = 0.0;
        profile.t[6] = 0.0;

        if f64::abs(self.a0) > f64::EPSILON {
            profile.t[3] = self.vd / self.a0;
            if profile.check_for_velocity(
                ControlSigns::UDDU,
                ReachedLimits::None,
                0.0,
                a_max,
                a_min,
            ) {
                return true;
            }
        } else if f64::abs(self.vd) < f64::EPSILON
            && profile.check_for_velocity(
                ControlSigns::UDDU,
                ReachedLimits::None,
                0.0,
                a_max,
                a_min,
            )
        {
            return true;
        }

        false
    }

    pub fn get_profile(&mut self, input: &mut Profile, block: &mut block::Block) -> bool {
        // Zero-limits special case
        if self._j_max == 0.0 {
            let p = &mut block.p_min;
            p.set_boundary_from_profile(input);

            if self.time_all_single_step(p, self._a_max, self._a_min, self._j_max) {
                block.t_min = *p.t_sum.last().unwrap_or(&0.0) + p.brake.duration + p.accel.duration;
                if f64::abs(self.a0) > f64::EPSILON {
                    block.a = Some(Interval::new(block.t_min, f64::INFINITY));
                }
                return true;
            }
            return false;
        }

        self.valid_profiles[0].set_boundary_from_profile(input);
        self.current_index = 0;

        if f64::abs(self.af) < f64::EPSILON {
            // There is no blocked interval when self.af==0.0, so return after first found profile
            let a_max = if self.vd >= 0.0 {
                self._a_max
            } else {
                self._a_min
            };
            let a_min = if self.vd >= 0.0 {
                self._a_min
            } else {
                self._a_max
            };
            let j_max = if self.vd >= 0.0 {
                self._j_max
            } else {
                -self._j_max
            };

            self.time_none(a_max, a_min, j_max, true);
            if self.current_index > 0 {
                return Block::calculate_block(
                    block,
                    &mut self.valid_profiles,
                    &mut self.current_index,
                    None,
                );
            }
            self.time_acc0(a_max, a_min, j_max, true);
            if self.current_index > 0 {
                return Block::calculate_block(
                    block,
                    &mut self.valid_profiles,
                    &mut self.current_index,
                    None,
                );
            }

            self.time_none(a_min, a_max, -j_max, true);
            if self.current_index > 0 {
                return Block::calculate_block(
                    block,
                    &mut self.valid_profiles,
                    &mut self.current_index,
                    None,
                );
            }
            self.time_acc0(a_min, a_max, -j_max, true);
        } else {
            self.time_none(self._a_max, self._a_min, self._j_max, false);
            self.time_none(self._a_min, self._a_max, -self._j_max, false);
            self.time_acc0(self._a_max, self._a_min, self._j_max, false);
            self.time_acc0(self._a_min, self._a_max, -self._j_max, false);
        }

        Block::calculate_block(
            block,
            &mut self.valid_profiles,
            &mut self.current_index,
            None,
        )
    }
}
