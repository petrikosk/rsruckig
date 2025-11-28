//! Which times are possible for synchronization?
use crate::profile::Profile;
use core::cmp::Ordering;
use core::f64;
use core::fmt;
use core::option::Option;

#[derive(Clone, Default, Debug)]
pub struct Block {
    pub p_min: Profile,
    pub t_min: f64,
    pub a: Option<Interval>,
    pub b: Option<Interval>,
}

pub fn remove_profile(
    valid_profiles: &mut [Profile; 6],
    valid_profile_counter: &mut usize,
    index: usize,
) {
    for i in index..(*valid_profile_counter - 1) {
        valid_profiles[i] = valid_profiles[i + 1].clone();
    }
    *valid_profile_counter -= 1;
}

impl Block {
    #[inline]
    pub fn set_min_profile(&mut self, profile: &Profile) {
        self.p_min = profile.clone();
        self.t_min = self.p_min.t_sum[6]
            + self.p_min.brake.duration
            + self.p_min.accel.duration;
        self.a = None;
        self.b = None;
    }

    pub fn calculate_block(
        block: &mut Block,
        valid_profiles: &mut [Profile; 6],
        valid_profile_counter: &mut usize,
        numerical_robust: Option<bool>,
    ) -> bool {
        if *valid_profile_counter == 1 {
            block.set_min_profile(&valid_profiles[0]);
            return true;
        } else if *valid_profile_counter == 2 {
            if f64::abs(
                valid_profiles[0].t_sum[6] - valid_profiles[1].t_sum[6],
            ) < 8.0 * core::f64::EPSILON
            {
                block.set_min_profile(&valid_profiles[0]);
                return true;
            }

            if numerical_robust.unwrap_or(true) {
                let idx_min = if valid_profiles[0].t_sum[6] < valid_profiles[1].t_sum[6] {
                    0
                } else {
                    1
                };

                let idx_else_1 = (idx_min + 1) % 2;

                block.set_min_profile(&valid_profiles[idx_min]);
                block.a = Some(Interval::from_profiles(
                    &valid_profiles[idx_min],
                    &valid_profiles[idx_else_1],
                ));

                return true;
            }
            // Only happens due to numerical issues
        } else if *valid_profile_counter == 4 {
            // Find "identical" profiles
            if f64::abs(
                valid_profiles[0].t_sum[6] - valid_profiles[1].t_sum[6],
            ) < 32.0 * f64::EPSILON
                && valid_profiles[0].direction != valid_profiles[1].direction
            {
                remove_profile(valid_profiles, valid_profile_counter, 1);
            } else if (f64::abs(
                valid_profiles[2].t_sum[6] - valid_profiles[3].t_sum[6],
            ) < 256.0 * f64::EPSILON
                && valid_profiles[2].direction != valid_profiles[3].direction)
                || (f64::abs(
                    valid_profiles[0].t_sum[6] - valid_profiles[3].t_sum[6],
                ) < 256.0 * f64::EPSILON
                    && valid_profiles[0].direction != valid_profiles[3].direction)
            {
                remove_profile(valid_profiles, valid_profile_counter, 3);
            } else {
                return false;
            }
        } else if *valid_profile_counter % 2 == 0 {
            return false;
        }

        // Find index of fastest profile
        let idx_min = valid_profiles
            .iter()
            .take(*valid_profile_counter)
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                a.t_sum[6]
                    .partial_cmp(&b.t_sum[6])
                    .unwrap_or(Ordering::Equal)
            })
            .map(|(idx, _)| idx)
            .unwrap_or(0);

        block.set_min_profile(&valid_profiles[idx_min]);

        if *valid_profile_counter == 3 {
            let idx_else_1 = (idx_min + 1) % 3;
            let idx_else_2 = (idx_min + 2) % 3;

            block.a = Some(Interval::from_profiles(
                &valid_profiles[idx_else_1],
                &valid_profiles[idx_else_2],
            ));
            return true;
        } else if *valid_profile_counter == 5 {
            let idx_else_1 = (idx_min + 1) % 5;
            let idx_else_2 = (idx_min + 2) % 5;
            let idx_else_3 = (idx_min + 3) % 5;
            let idx_else_4 = (idx_min + 4) % 5;

            if valid_profiles[idx_else_1].direction == valid_profiles[idx_else_2].direction {
                block.a = Some(Interval::from_profiles(
                    &valid_profiles[idx_else_1],
                    &valid_profiles[idx_else_2],
                ));
                block.b = Some(Interval::from_profiles(
                    &valid_profiles[idx_else_3],
                    &valid_profiles[idx_else_4],
                ));
            } else {
                block.a = Some(Interval::from_profiles(
                    &valid_profiles[idx_else_1],
                    &valid_profiles[idx_else_4],
                ));
                block.b = Some(Interval::from_profiles(
                    &valid_profiles[idx_else_2],
                    &valid_profiles[idx_else_3],
                ));
            }
            return true;
        }

        false
    }

    #[inline]
    pub fn is_blocked(&self, t: f64) -> bool {
        (t < self.t_min)
            || (self.a.is_some()
                && t > self.a.as_ref().unwrap().left
                && t < self.a.as_ref().unwrap().right)
            || (self.b.is_some()
                && t > self.b.as_ref().unwrap().left
                && t < self.b.as_ref().unwrap().right)
    }

    pub fn get_profile(&self, t: f64) -> &Profile {
        if self.b.is_some() && t >= self.b.as_ref().unwrap().right {
            &self.b.as_ref().unwrap().profile
        } else if self.a.is_some() && t >= self.a.as_ref().unwrap().right {
            &self.a.as_ref().unwrap().profile
        } else {
            &self.p_min
        }
    }
}

impl fmt::Display for Block {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "[{} ", self.t_min)?;
        if let Some(a) = &self.a {
            write!(f, "{}] [{} ", a.left, a.right)?;
        }
        if let Some(b) = &self.b {
            write!(f, "{}] [{} ", b.left, b.right)?;
        }
        write!(f, "-")
    }
}

#[derive(Debug, Clone, Default)]
pub struct Interval {
    pub left: f64,
    pub right: f64,
    pub profile: Profile,
}

impl Interval {
    pub fn new(left: f64, right: f64) -> Self {
        Self {
            left,
            right,
            ..Default::default()
        }
    }

    #[inline]
    pub fn from_profiles(profile_left: &Profile, profile_right: &Profile) -> Self {
        let left_duration = profile_left.t_sum[6]
            + profile_left.brake.duration
            + profile_left.accel.duration;
        let right_duration = profile_right.t_sum[6]
            + profile_right.brake.duration
            + profile_right.accel.duration;

        let (left, right, profile) = if left_duration < right_duration {
            (left_duration, right_duration, profile_right)
        } else {
            (right_duration, left_duration, profile_left)
        };

        Self {
            left,
            right,
            profile: profile.clone(),
        }
    }
}
