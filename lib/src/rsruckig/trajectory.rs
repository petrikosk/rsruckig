use crate::profile::Bound;
use crate::profile::Profile;
use crate::util::integrate;

// We'll use Vec<T> instead of CustomVector<T, DOFs>
#[derive(Clone)]
pub struct Trajectory<const DOF: usize> {
    pub profiles: Vec<[Profile; DOF]>,
    pub duration: f64,
    pub cumulative_times: [f64; DOF],
    pub independent_min_durations: [f64; DOF],
    position_extrema: [Bound; DOF],
}

impl<const DOF: usize> Default for Trajectory<DOF> {
    fn default() -> Self {
        Self {
            profiles: Default::default(),
            duration: Default::default(),
            cumulative_times: std::array::from_fn(|_| Default::default()),
            independent_min_durations: std::array::from_fn(|_| Default::default()),
            position_extrema: std::array::from_fn(|_| Default::default()),
        }
    }
}

impl<const DOF: usize> Trajectory<DOF> {
    pub fn new() -> Self {
        Self {
            profiles: vec![[Profile::default(); DOF]],
            duration: 0.0,
            cumulative_times: [0.0; DOF],
            independent_min_durations: [0.0; DOF],
            position_extrema: std::array::from_fn(|_| Bound::default()),
        }
    }
    pub fn state_to_integrate_from<F>(
        &self,
        time: f64,
        new_section: &mut usize,
        mut set_integrate: F,
    ) where
        F: FnMut(usize, f64, f64, f64, f64, f64),
    {
        let degrees_of_freedom = self.profiles[0].len();

        if time >= self.duration {
            *new_section = self.profiles.len();
            let profiles_dof = &self.profiles.last().unwrap();
            for dof in 0..degrees_of_freedom {
                let t_pre = if self.profiles.len() > 1 {
                    self.cumulative_times[self.cumulative_times.len() - 2]
                } else {
                    profiles_dof[dof].brake.duration
                };
                let t_diff = time - (t_pre + profiles_dof[dof].t_sum.last().unwrap());
                set_integrate(
                    dof,
                    t_diff,
                    *profiles_dof[dof].p.last().unwrap(),
                    *profiles_dof[dof].v.last().unwrap(),
                    *profiles_dof[dof].a.last().unwrap(),
                    0.0,
                );
            }
            return;
        }

        let new_section_index = self
            .cumulative_times
            .iter()
            .position(|&t| t > time)
            .unwrap_or(self.cumulative_times.len());
        *new_section = new_section_index;
        let mut t_diff = time;
        if new_section_index > 0 {
            t_diff -= self.cumulative_times[new_section_index - 1];
        }

        for dof in 0..degrees_of_freedom {
            let p = &self.profiles[*new_section][dof];
            let mut t_diff_dof = t_diff;

            // Brake pre-trajectory
            if *new_section == 0 && p.brake.duration > 0.0 {
                if t_diff_dof < p.brake.duration {
                    let index = if t_diff_dof < p.brake.t[0] { 0 } else { 1 };
                    if index > 0 {
                        t_diff_dof -= p.brake.t[index - 1];
                    }
                    set_integrate(
                        dof,
                        t_diff_dof,
                        p.brake.p[index],
                        p.brake.v[index],
                        p.brake.a[index],
                        p.brake.j[index],
                    );
                    continue;
                } else {
                    t_diff_dof -= p.brake.duration;
                }
            }
            if t_diff_dof >= *p.t_sum.last().unwrap_or(&0.0) {
                set_integrate(
                    dof,
                    t_diff_dof - p.t_sum.last().unwrap_or(&0.0),
                    *p.p.last().unwrap_or(&0.0),
                    *p.v.last().unwrap_or(&0.0),
                    *p.a.last().unwrap_or(&0.0),
                    0.0,
                );
                continue;
            }

            let index_dof = p
                .t_sum
                .iter()
                .position(|&t| t > t_diff_dof)
                .unwrap_or(p.t_sum.len() - 1);

            if index_dof > 0 {
                t_diff_dof -= p.t_sum[index_dof - 1];
            }

            set_integrate(
                dof,
                t_diff_dof,
                p.p[index_dof],
                p.v[index_dof],
                p.a[index_dof],
                p.j[index_dof],
            );
        }
    }

    pub fn at_time(
        &self,
        time: f64,
        new_position: &mut Option<&mut [f64; DOF]>,
        new_velocity: &mut Option<&mut [f64; DOF]>,
        new_acceleration: &mut Option<&mut [f64; DOF]>,
        new_jerk: &mut Option<&mut [f64; DOF]>,
        new_section: &mut Option<usize>,
    ) {
        new_section.get_or_insert(0);

        if let Some(ref mut section_value) = new_section {
            self.state_to_integrate_from(time, section_value, |dof, t, p, v, a, j| {
                let (pos, vel, acc) = integrate(t, p, v, a, j);
                if let Some(ref mut pos_vec) = new_position {
                    pos_vec[dof] = pos;
                }

                if let Some(ref mut vel_vec) = new_velocity {
                    vel_vec[dof] = vel;
                }

                if let Some(ref mut acc_vec) = new_acceleration {
                    acc_vec[dof] = acc;
                }

                if let Some(ref mut jerk_vec) = new_jerk {
                    jerk_vec[dof] = j;
                }
            });
        }
    }

    pub fn get_profiles(&self) -> &Vec<[Profile; DOF]> {
        &self.profiles
    }

    pub fn get_duration(&self) -> f64 {
        self.duration
    }

    pub fn get_intermediate_durations(&self) -> &[f64; DOF] {
        &self.cumulative_times
    }

    pub fn get_independent_min_durations(&self) -> &[f64; DOF] {
        &self.independent_min_durations
    }

    pub fn get_position_extrema(&mut self) -> &[Bound; DOF] {
        for dof in 0..DOF {
            self.position_extrema[dof] = self.profiles[0][dof].get_position_extrema();
        }

        for i in 1..self.profiles.len() {
            for dof in 0..DOF {
                let section_position_extrema = self.profiles[i][dof].get_position_extrema();
                if section_position_extrema.max > self.position_extrema[dof].max {
                    self.position_extrema[dof].max = section_position_extrema.max;
                    self.position_extrema[dof].t_max = section_position_extrema.t_max;
                }
                if section_position_extrema.min < self.position_extrema[dof].min {
                    self.position_extrema[dof].min = section_position_extrema.min;
                    self.position_extrema[dof].t_min = section_position_extrema.t_min;
                }
            }
        }

        &self.position_extrema
    }

    pub fn get_first_time_at_position(&self, dof: usize, position: f64) -> Option<f64> {
        if dof >= DOF {
            return None;
        }

        let time; // Or any default value
        for p in &self.profiles {
            if let Some((returned_time, _, _)) = p[dof].get_first_state_at_position(position, 0.0) {
                time = returned_time;
                return Some(time);
            }
        }
        None
    }
}
