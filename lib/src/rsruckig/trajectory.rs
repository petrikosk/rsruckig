//! Trajectory representation and evaluation
//!
//! This module provides the core trajectory type that represents a complete
//! time-parameterized path with position, velocity, and acceleration profiles.

use crate::profile::Bound;
use crate::profile::Profile;
use crate::roots;
use crate::util::{integrate, DataArrayOrVec};

use crate::alloc::{vec, vec::Vec};

/// The kinematic state of all degrees of freedom at one time of a
/// [`Trajectory`], as returned by [`Trajectory::state_at`] and
/// [`Trajectory::at_times`]
#[derive(Debug, Clone, PartialEq)]
pub struct KinematicState<const DOF: usize> {
    /// The sampled time
    pub time: f64,
    /// Position of each DoF
    pub position: DataArrayOrVec<f64, DOF>,
    /// Velocity of each DoF
    pub velocity: DataArrayOrVec<f64, DOF>,
    /// Acceleration of each DoF
    pub acceleration: DataArrayOrVec<f64, DOF>,
    /// Jerk of each DoF
    pub jerk: DataArrayOrVec<f64, DOF>,
    /// Index of the section the time lies in; the number of sections for
    /// times past the end of the trajectory
    pub section: usize,
}

/// A calculated trajectory: one [`Profile`] per section and degree of freedom
#[derive(Debug, Clone)]
pub struct Trajectory<const DOF: usize> {
    /// Profiles per section (one entry per section between waypoints) and DoF
    pub profiles: Vec<DataArrayOrVec<Profile, DOF>>,
    /// Total duration of the trajectory in seconds
    pub duration: f64,
    /// Absolute end time of each section; the last entry equals `duration`
    pub cumulative_times: Vec<f64>,
    /// Minimum duration each DoF would need on its own, without
    /// synchronization to the other DoFs
    pub independent_min_durations: DataArrayOrVec<f64, DOF>,
    pub(crate) phase_synchronized_sections: Vec<bool>,
    position_extrema: DataArrayOrVec<Bound, DOF>,
    degrees_of_freedom: usize,
}

impl<const DOF: usize> Default for Trajectory<DOF> {
    fn default() -> Self {
        Self {
            profiles: vec![DataArrayOrVec::new(None, Profile::default())],
            duration: Default::default(),
            cumulative_times: vec![0.0],
            phase_synchronized_sections: vec![false],
            independent_min_durations: DataArrayOrVec::new(None, 0.0),
            position_extrema: DataArrayOrVec::new(None, Bound::default()),
            degrees_of_freedom: DOF,
        }
    }
}

impl<const DOF: usize> Trajectory<DOF> {
    /// Create an empty trajectory with a single section.
    ///
    /// `dofs` is the number of degrees of freedom for heap allocation
    /// (`DOF == 0`) and is ignored for stack allocation (`DOF > 0`).
    pub fn new(dofs: Option<usize>) -> Self {
        Self {
            profiles: vec![DataArrayOrVec::<Profile, DOF>::new(
                dofs,
                Profile::default(),
            )],
            duration: 0.0,
            cumulative_times: vec![0.0],
            phase_synchronized_sections: vec![false],
            independent_min_durations: DataArrayOrVec::new(dofs, 0.0),
            position_extrema: DataArrayOrVec::new(dofs, Bound::default()),
            degrees_of_freedom: dofs.unwrap_or(DOF),
        }
    }

    /// Resize to `sections` sections, reusing existing storage where possible
    pub(crate) fn resize_sections(&mut self, sections: usize) {
        let dofs = if DOF == 0 {
            Some(self.degrees_of_freedom)
        } else {
            None
        };
        self.profiles
            .resize_with(sections, || DataArrayOrVec::new(dofs, Profile::default()));
        self.cumulative_times.resize(sections, 0.0);
        self.phase_synchronized_sections.resize(sections, false);
    }

    /// Pre-allocate storage for a trajectory with up to `sections` sections
    /// (i.e. `sections - 1` intermediate waypoints) to avoid allocations during
    /// later calculations
    pub fn reserve_sections(&mut self, sections: usize) {
        if sections > self.profiles.len() {
            self.profiles.reserve(sections - self.profiles.len());
        }
        if sections > self.phase_synchronized_sections.len() {
            self.phase_synchronized_sections
                .reserve(sections - self.phase_synchronized_sections.len());
        }
        if sections > self.cumulative_times.len() {
            self.cumulative_times
                .reserve(sections - self.cumulative_times.len());
        }
    }

    /// Absolute start time of `section`
    #[inline]
    fn section_start(&self, section: usize) -> f64 {
        if section > 0 {
            self.cumulative_times[section - 1]
        } else {
            0.0
        }
    }

    /// Locate the phase that contains `time` and call `set_integrate` once per
    /// DoF with the starting state of that phase.
    ///
    /// The callback receives `(dof, t, p, v, a, j)`: the time `t` elapsed
    /// since the start of the phase and the phase's initial position,
    /// velocity, acceleration and constant jerk, ready to be passed to
    /// [`integrate`]. Times past the end extrapolate the final state with
    /// zero jerk. `new_section` is set to the index of the section containing
    /// `time` (the number of sections for times past the end).
    #[inline]
    pub fn state_to_integrate_from<F>(
        &self,
        time: f64,
        new_section: &mut usize,
        mut set_integrate: F,
    ) where
        F: FnMut(usize, f64, f64, f64, f64, f64),
    {
        if time >= self.duration {
            // Past the end: extrapolate the final state of the last section
            let last = self.profiles.len() - 1;
            *new_section = self.profiles.len();
            let t_section_start = self.section_start(last);
            for (dof, p) in self.profiles[last].iter().enumerate() {
                let t_diff = time - (t_section_start + p.brake.duration + p.t_sum[6]);
                set_integrate(dof, t_diff, p.p[7], p.v[7], p.a[7], 0.0);
            }
            return;
        }

        // `time < duration` guarantees a section end after `time`
        let section = self
            .cumulative_times
            .iter()
            .position(|&end| end > time)
            .unwrap_or(self.cumulative_times.len() - 1);
        *new_section = section;
        let t_diff = time - self.section_start(section);

        for (dof, p) in self.profiles[section].iter().enumerate() {
            let mut t_diff_dof = t_diff;

            // Brake pre-trajectory
            if p.brake.duration > 0.0 {
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
            if t_diff_dof >= p.t_sum[6] {
                set_integrate(dof, t_diff_dof - p.t_sum[6], p.p[7], p.v[7], p.a[7], 0.0);
                continue;
            }

            // `t_diff_dof < t_sum[6]` guarantees a phase end after it
            let index_dof = p
                .t_sum
                .iter()
                .position(|&end| end > t_diff_dof)
                .unwrap_or(6);

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

    /// Sample the kinematic state at `time` into existing storage.
    ///
    /// Each output that is `Some` is overwritten with the corresponding
    /// value per DoF; `None` outputs are skipped. `new_section` receives the
    /// section index (see [`state_to_integrate_from`](Trajectory::state_to_integrate_from)).
    /// This method does not allocate, which makes it suitable for control
    /// loops; [`state_at`](Trajectory::state_at) is the more convenient
    /// by-value alternative.
    #[inline]
    pub fn at_time(
        &self,
        time: f64,
        new_position: &mut Option<&mut DataArrayOrVec<f64, DOF>>,
        new_velocity: &mut Option<&mut DataArrayOrVec<f64, DOF>>,
        new_acceleration: &mut Option<&mut DataArrayOrVec<f64, DOF>>,
        new_jerk: &mut Option<&mut DataArrayOrVec<f64, DOF>>,
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

    /// The kinematic state at the given time.
    ///
    /// Returns the state by value, which allocates for a runtime number of
    /// DoFs (`DOF == 0`); use [`Trajectory::at_time`] to sample into existing
    /// storage in a control loop.
    ///
    /// ```
    /// use rsruckig::prelude::*;
    ///
    /// let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    /// let mut input = InputParameter::new(None);
    /// input.target_position = daov_stack![1.0];
    /// input.max_velocity = daov_stack![1.0];
    /// input.max_acceleration = daov_stack![1.0];
    /// input.max_jerk = daov_stack![1.0];
    ///
    /// let mut trajectory = Trajectory::new(None);
    /// otg.calculate(&input, &mut trajectory).unwrap();
    ///
    /// let end = trajectory.state_at(trajectory.get_duration());
    /// assert!((end.position[0] - 1.0).abs() < 1e-9);
    /// ```
    pub fn state_at(&self, time: f64) -> KinematicState<DOF> {
        let dofs = (DOF == 0).then_some(self.degrees_of_freedom);
        let mut state = KinematicState {
            time,
            position: DataArrayOrVec::new(dofs, 0.0),
            velocity: DataArrayOrVec::new(dofs, 0.0),
            acceleration: DataArrayOrVec::new(dofs, 0.0),
            jerk: DataArrayOrVec::new(dofs, 0.0),
            section: 0,
        };
        self.state_to_integrate_from(time, &mut state.section, |dof, t, p, v, a, j| {
            (
                state.position[dof],
                state.velocity[dof],
                state.acceleration[dof],
            ) = integrate(t, p, v, a, j);
            state.jerk[dof] = j;
        });
        state
    }

    /// The kinematic states at several times, sampled lazily.
    ///
    /// The times may be given in any order, by value or by reference.
    ///
    /// ```
    /// use rsruckig::prelude::*;
    ///
    /// let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    /// let mut input = InputParameter::new(None);
    /// input.target_position = daov_stack![1.0];
    /// input.max_velocity = daov_stack![1.0];
    /// input.max_acceleration = daov_stack![1.0];
    /// input.max_jerk = daov_stack![1.0];
    ///
    /// let mut trajectory = Trajectory::new(None);
    /// otg.calculate(&input, &mut trajectory).unwrap();
    ///
    /// let duration = trajectory.get_duration();
    /// let times = (0..=10).map(|i| duration * f64::from(i) / 10.0);
    /// let positions: Vec<f64> = trajectory
    ///     .at_times(times)
    ///     .map(|state| state.position[0])
    ///     .collect();
    /// assert!(positions.windows(2).all(|pair| pair[0] <= pair[1]));
    ///
    /// // Times by reference
    /// let states: Vec<_> = trajectory.at_times(&[0.0, duration]).collect();
    /// assert_eq!(states[0].position[0], 0.0);
    /// ```
    pub fn at_times<'a, I>(&'a self, times: I) -> impl Iterator<Item = KinematicState<DOF>> + 'a
    where
        I: IntoIterator,
        I::Item: core::borrow::Borrow<f64>,
        I::IntoIter: 'a,
    {
        times
            .into_iter()
            .map(move |time| self.state_at(*core::borrow::Borrow::borrow(&time)))
    }

    /// Copies `other` into this trajectory, reusing the existing storage
    pub(crate) fn copy_from(&mut self, other: &Self) {
        self.resize_sections(other.profiles.len());
        for (profiles, other_profiles) in self.profiles.iter_mut().zip(&other.profiles) {
            profiles.copy_from(other_profiles);
        }
        self.phase_synchronized_sections
            .copy_from_slice(&other.phase_synchronized_sections);
        self.duration = other.duration;
        self.cumulative_times.copy_from_slice(&other.cumulative_times);
        self.independent_min_durations
            .copy_from(&other.independent_min_durations);
        self.position_extrema.copy_from(&other.position_extrema);
    }

    /// Whether the solver retained a shared phase profile for each section.
    ///
    /// A false value means phase synchronization was not established by the
    /// solver, including when it was not requested or time synchronization was
    /// used instead. True requires every enabled DoF to request phase
    /// synchronization and have no brake or acceleration pre-trajectory. This
    /// reports the solver outcome, not a geometric collision-safety guarantee.
    /// Results are meaningful after a successful calculation.
    ///
    /// With intermediate waypoints the sections are currently never phase
    /// synchronized, as the pass-through states of the waypoints are chosen
    /// per DoF.
    pub fn phase_synchronized_sections(&self) -> &[bool] {
        &self.phase_synchronized_sections
    }

    /// The profiles of all sections, indexed as `[section][dof]`
    #[inline]
    pub fn get_profiles(&self) -> &Vec<DataArrayOrVec<Profile, { DOF }>> {
        &self.profiles
    }

    /// Total duration of the trajectory in seconds
    #[inline]
    pub fn get_duration(&self) -> f64 {
        self.duration
    }

    /// Absolute times at which the intermediate waypoints are reached (one
    /// entry per section; the last entry equals the total duration)
    #[inline]
    pub fn get_intermediate_durations(&self) -> &[f64] {
        &self.cumulative_times
    }

    /// Minimum duration each DoF would need on its own, without
    /// synchronization to the other DoFs
    #[inline]
    pub fn get_independent_min_durations(&self) -> &DataArrayOrVec<f64, { DOF }> {
        &self.independent_min_durations
    }

    /// Position extrema of each DoF over the whole trajectory, together with
    /// the (absolute) times at which they are reached.
    ///
    /// The extrema bound every state sampled between zero and the duration
    /// of the trajectory: a DoF whose profile ends earlier (no
    /// synchronization, or a disabled DoF) keeps moving with its final
    /// velocity until the trajectory ends, which is included here.
    ///
    /// The extrema are computed on each call and cached in the trajectory,
    /// hence the `&mut self`.
    pub fn get_position_extrema(&mut self) -> &DataArrayOrVec<Bound, { DOF }> {
        let section_duration = self.cumulative_times[0];
        for dof in 0..self.degrees_of_freedom {
            self.position_extrema[dof] =
                self.profiles[0][dof].get_position_extrema_until(section_duration);
        }

        for i in 1..self.profiles.len() {
            // Profile times are section-local; offset by the section start
            let t_offset = self.section_start(i);
            let section_duration = self.cumulative_times[i] - t_offset;
            for dof in 0..self.degrees_of_freedom {
                let section_position_extrema =
                    self.profiles[i][dof].get_position_extrema_until(section_duration);
                if section_position_extrema.max > self.position_extrema[dof].max {
                    self.position_extrema[dof].max = section_position_extrema.max;
                    self.position_extrema[dof].t_max = section_position_extrema.t_max + t_offset;
                }
                if section_position_extrema.min < self.position_extrema[dof].min {
                    self.position_extrema[dof].min = section_position_extrema.min;
                    self.position_extrema[dof].t_min = section_position_extrema.t_min + t_offset;
                }
            }
        }

        &self.position_extrema
    }

    /// Velocity extrema of `dof` within `section`, together with the
    /// (absolute) times at which they are reached, or `None` if `section` or
    /// `dof` is out of range.
    ///
    /// A trajectory without intermediate waypoints has the single section
    /// `0`; with waypoints, section `i` ends at
    /// [`get_intermediate_durations()[i]`](Self::get_intermediate_durations).
    /// The extrema bound every state sampled within the section, including
    /// the brake pre-trajectory and the motion of a DoF whose profile ends
    /// before the section does (no synchronization, or a disabled DoF). They
    /// are computed from the same phases the trajectory is sampled from, so
    /// they hold up to floating-point rounding. The peak absolute velocity is
    /// `min.abs().max(max.abs())`.
    ///
    /// ```
    /// use rsruckig::prelude::*;
    ///
    /// let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    /// let mut input = InputParameter::new(None);
    /// input.target_position = daov_stack![4.0];
    /// input.max_velocity = daov_stack![1.0];
    /// input.max_acceleration = daov_stack![1.0];
    /// input.max_jerk = daov_stack![1.0];
    ///
    /// let mut trajectory = Trajectory::new(None);
    /// otg.calculate(&input, &mut trajectory).unwrap();
    ///
    /// let extrema = trajectory.get_velocity_extrema(0, 0).unwrap();
    /// assert!(extrema.min.abs() < 1e-12);
    /// assert!((extrema.max - 1.0).abs() < 1e-12);
    /// assert!(trajectory.get_velocity_extrema(1, 0).is_none());
    /// ```
    pub fn get_velocity_extrema(&self, section: usize, dof: usize) -> Option<Bound> {
        if dof >= self.degrees_of_freedom {
            return None;
        }
        let profile = self.profiles.get(section)?.get(dof)?;
        let t_offset = self.section_start(section);
        let section_duration = *self.cumulative_times.get(section)? - t_offset;

        let mut extrema = profile.get_velocity_extrema_until(section_duration);
        // The durations of a profile's phases can sum up to slightly more
        // than the section's duration by rounding; keep the times inside
        extrema.t_min = extrema.t_min.min(section_duration) + t_offset;
        extrema.t_max = extrema.t_max.min(section_duration) + t_offset;
        Some(extrema)
    }

    /// The first (absolute) time at which `dof` passes through `position`,
    /// or `None` if the position is never reached or `dof` is out of range
    pub fn get_first_time_at_position(&self, dof: usize, position: f64) -> Option<f64> {
        if dof >= self.degrees_of_freedom {
            return None;
        }

        for (section, profiles) in self.profiles.iter().enumerate() {
            let p = &profiles[dof];
            // Profile times are section-local; offset by the section start
            let t_offset = self.section_start(section);

            // Brake pre-trajectory (not part of the profile's own phases)
            if p.brake.duration > 0.0 {
                let mut t_phase_start = 0.0;
                for phase in 0..2 {
                    let t_phase = p.brake.t[phase];
                    if t_phase <= 0.0 {
                        continue;
                    }
                    let first_crossing = roots::solve_cub(
                        p.brake.j[phase] / 6.0,
                        p.brake.a[phase] / 2.0,
                        p.brake.v[phase],
                        p.brake.p[phase] - position,
                    )
                    .get_data()
                    .iter()
                    .copied()
                    .filter(|&t| (0.0..=t_phase).contains(&t))
                    .min_by(f64::total_cmp);
                    if let Some(t) = first_crossing {
                        return Some(t_offset + t_phase_start + t);
                    }
                    t_phase_start += t_phase;
                }
            }

            if let Some((time, _, _)) =
                p.get_first_state_at_position(position, t_offset + p.brake.duration)
            {
                return Some(time);
            }
        }
        None
    }
}
