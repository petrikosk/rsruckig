//! Mathematical equations for Step 1 in third-order velocity interface: Extremal profiles

use crate::{
    block::{self, Block, Interval},
    profile::{ControlSigns, Profile, ReachedLimits},
};

#[cfg(not(feature = "std"))]
use num_traits::Float;

/// The limits `(a_max, a_min, j_max)` of a search direction; the opposite
/// direction swaps the bounds and negates the jerk
type DirectedLimits = (f64, f64, f64);

/// Step 1 solver for the third-order velocity interface: calculates the
/// time-optimal profile and the blocked duration intervals.
#[derive(Debug)]
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
    /// Create a solver for changing from state `(v0, a0)` to `(vf, af)` within
    /// the acceleration limits `[a_min, a_max]` and maximal jerk `j_max`.
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

    /// [`Self::time_acc0`] for numerically degenerate inputs.
    ///
    /// At the boundary between the profiles without and with an acceleration
    /// plateau, where the acceleration just touches `a_max`, both neighbours
    /// may be rejected by rounding: the peak of [`Self::time_none`] exceeds
    /// the limit by more than the tolerance, while the plateau is still
    /// shorter than the machine epsilon and counts as missing. A plateau of
    /// vanishing duration is therefore accepted here. Moreover, the
    /// acceleration is snapped to the limit that the first phase reaches by
    /// construction: integrating it may miss a limit of some thousands by
    /// more than the (absolute) tolerance, in particular when starting
    /// exactly at the opposite limit.
    fn time_acc0_degenerate(&mut self, a_max: f64, a_min: f64, j_max: f64) {
        let t0 = (-self.a0 + a_max) / j_max;
        let t2 = (-self.af + a_max) / j_max;
        let mut t1 = (self.a0 * self.a0 + self.af * self.af) / (2.0 * a_max * j_max)
            - a_max / j_max
            + self.vd / a_max;

        // The rounding error of the plateau duration
        let tolerance =
            8.0 * f64::EPSILON * f64::max(1.0, t0.abs() + t2.abs() + (self.vd / a_max).abs());
        if t1 < -tolerance {
            return;
        }
        let limits = if t1 < f64::EPSILON {
            t1 = 0.0;
            ReachedLimits::None
        } else {
            ReachedLimits::Acc0
        };

        let profile = &mut self.valid_profiles[self.current_index];
        profile.t[0] = t0;
        profile.t[1] = t1;
        profile.t[2] = t2;
        profile.t[3] = 0.0;
        profile.t[4] = 0.0;
        profile.t[5] = 0.0;
        profile.t[6] = 0.0;

        if profile.check_for_velocity_at_acceleration_limit(
            ControlSigns::UDDU,
            limits,
            j_max,
            a_max,
            a_min,
        ) {
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

    /// Calculate the time-optimal profile for this DoF.
    ///
    /// Writes the minimal-duration profile into `block.p_min` (with the brake
    /// and acceleration pre-trajectories and boundary state taken from
    /// `input`), sets `block.t_min` and, where applicable, the blocked duration
    /// intervals `block.a` / `block.b`. Returns `false` if no valid profile was
    /// found.
    pub fn get_profile(&mut self, input: &mut Profile, block: &mut block::Block) -> bool {
        // Zero-limits special case
        if self._j_max == 0.0 {
            let p = &mut block.p_min;
            p.set_boundary_from_profile(input);

            if self.time_all_single_step(p, self._a_max, self._a_min, self._j_max) {
                block.t_min = p.t_sum[6] + p.brake.duration + p.accel.duration;
                if f64::abs(self.a0) > f64::EPSILON {
                    block.a = Some(Interval::new(block.t_min, f64::INFINITY));
                }
                return true;
            }
            return false;
        }

        self.valid_profiles[0].set_boundary_from_profile(input);
        self.current_index = 0;

        let up: DirectedLimits = (self._a_max, self._a_min, self._j_max);
        let down: DirectedLimits = (self._a_min, self._a_max, -self._j_max);

        if f64::abs(self.af) < f64::EPSILON {
            // There is no blocked interval when af==0, so return after first
            // found profile, searching in the direction of the target first
            let directions = if self.vd >= 0.0 {
                [up, down]
            } else {
                [down, up]
            };
            for (a_max, a_min, j_max) in directions {
                self.time_none(a_max, a_min, j_max, true);
                if self.current_index > 0 {
                    break;
                }
                self.time_acc0(a_max, a_min, j_max, true);
                if self.current_index > 0 {
                    break;
                }
            }
        } else {
            // The order of the valid profiles matters for the block calculation
            for (a_max, a_min, j_max) in [up, down] {
                self.time_none(a_max, a_min, j_max, false);
            }
            for (a_max, a_min, j_max) in [up, down] {
                self.time_acc0(a_max, a_min, j_max, false);
            }
        }

        if self.current_index == 0 {
            // Fallback for numerically degenerate inputs, searching in the
            // direction of the target first
            let directions = if self.vd >= 0.0 {
                [up, down]
            } else {
                [down, up]
            };
            for (a_max, a_min, j_max) in directions {
                self.time_acc0_degenerate(a_max, a_min, j_max);
                if self.current_index > 0 {
                    break;
                }
            }
        }

        Block::calculate_block(
            block,
            &mut self.valid_profiles,
            &mut self.current_index,
            None,
        )
    }
}
