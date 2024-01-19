use crate::profile::Bound;
use crate::profile::Profile;
use crate::util::{integrate, DataArrayOrVec};

// We'll use Vec<T> instead of CustomVector<T, DOFs>
#[derive(Clone)]
pub struct Trajectory<const DOFs: usize> {
    pub profiles: Vec<DataArrayOrVec<Profile, DOFs>>,
    pub duration: f64,
    pub cumulative_times: DataArrayOrVec<f64, DOFs>,
    pub independent_min_durations: DataArrayOrVec<f64, DOFs>,
    position_extrema: DataArrayOrVec<Bound, DOFs>,
    degrees_of_freedom: usize,
    continue_calculation_counter: usize,
}

impl<const DOFs: usize> Default for Trajectory<DOFs> {
    fn default() -> Self {
        Self {
            profiles: Default::default(),
            duration: Default::default(),
            cumulative_times: DataArrayOrVec::default(),
            independent_min_durations: DataArrayOrVec::default(),
            position_extrema: DataArrayOrVec::default(),
            degrees_of_freedom: 1,
            continue_calculation_counter: 0,
        }
    }
}

impl<const DOFs: usize> Trajectory<DOFs> {
    pub fn new(dofs: Option<usize>) -> Self {
        Self {
            profiles: vec![DataArrayOrVec::new(dofs, Profile::default())],
            duration: 0.0,
            cumulative_times: DataArrayOrVec::new(dofs, 0.0),
            independent_min_durations: vec![0.0; dofs],
            position_extrema: vec![Bound::default(); dofs],
            degrees_of_freedom: dofs,
            continue_calculation_counter: 0,
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

    // Equivalent to the C++ constructor with D == 0
    pub fn with_dofs(dofs: usize) -> Self {
        let mut traj = Trajectory {
            degrees_of_freedom: dofs,
            profiles: Vec::new(),
            duration: 0.0,
            cumulative_times: Vec::new(),
            independent_min_durations: Vec::new(),
            position_extrema: Vec::new(),
            continue_calculation_counter: 0,
        };
        traj.resize_for_dofs(dofs);
        traj
    }

    // This is a helper method for resizing based on dofs
    fn resize_for_dofs(&mut self, dofs: usize) {
        // Do the necessary resizing. Assuming `profiles` and others are Vecs:
        self.profiles[0].resize(dofs, Default::default());
        self.independent_min_durations.resize(dofs, 0.0);
        self.position_extrema.resize(dofs, Default::default());
        self.cumulative_times.resize(dofs, 0.0);
        self.continue_calculation_counter = 0;
        self.degrees_of_freedom = dofs;
        self.duration = 0.0;
    }

    pub fn at_time(
        &self,
        time: f64,
        new_position: &mut Option<&mut Vec<f64>>,
        new_velocity: &mut Option<&mut Vec<f64>>,
        new_acceleration: &mut Option<&mut Vec<f64>>,
        new_jerk: &mut Option<&mut Vec<f64>>,
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

    pub fn get_profiles(&self) -> &Vec<Vec<Profile>> {
        &self.profiles
    }

    pub fn get_duration(&self) -> f64 {
        self.duration
    }

    pub fn get_intermediate_durations(&self) -> &Vec<f64> {
        &self.cumulative_times
    }

    pub fn get_independent_min_durations(&self) -> &Vec<f64> {
        &self.independent_min_durations
    }

    pub fn get_position_extrema(&mut self) -> &Vec<Bound> {
        for dof in 0..self.degrees_of_freedom {
            self.position_extrema[dof] = self.profiles[0][dof].get_position_extrema();
        }

        for i in 1..self.profiles.len() {
            for dof in 0..self.degrees_of_freedom {
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
        if dof >= self.degrees_of_freedom {
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
