//! Calculation of a state-to-state trajectory.
use crate::error::{RuckigError, RuckigErrorHandler};
use crate::util::DataArrayOrVec;
use crate::{
    block::Block,
    input_parameter::{ControlInterface, DurationDiscretization, InputParameter, Synchronization},
    position_first_step1::PositionFirstOrderStep1,
    position_first_step2::PositionFirstOrderStep2,
    position_second_step1::PositionSecondOrderStep1,
    position_second_step2::PositionSecondOrderStep2,
    position_third_step1::PositionThirdOrderStep1,
    position_third_step2::PositionThirdOrderStep2,
    profile::{ControlSigns, Direction, Profile, ReachedLimits},
    result::RuckigResult,
    trajectory::Trajectory,
    velocity_second_step1::VelocitySecondOrderStep1,
    velocity_second_step2::VelocitySecondOrderStep2,
    velocity_third_step1::VelocityThirdOrderStep1,
    velocity_third_step2::VelocityThirdOrderStep2,
};

#[derive(Default, Debug)]
pub struct TargetCalculator<const DOF: usize> {
    eps: f64,
    return_error_at_maximal_duration: bool,
    new_phase_control: DataArrayOrVec<f64, DOF>,
    pd: DataArrayOrVec<f64, DOF>,
    possible_t_syncs: Vec<f64>,
    idx: Vec<usize>,
    blocks: DataArrayOrVec<Block, DOF>,
    inp_min_velocity: DataArrayOrVec<f64, DOF>,
    inp_min_acceleration: DataArrayOrVec<f64, DOF>,
    inp_per_dof_control_interface: DataArrayOrVec<ControlInterface, DOF>,
    inp_per_dof_synchronization: DataArrayOrVec<Synchronization, DOF>,
    pub degrees_of_freedom: usize,
}

impl<const DOF: usize> TargetCalculator<DOF> {
    pub fn new(dofs: Option<usize>) -> Self {
        Self {
            blocks: DataArrayOrVec::new(dofs, Block::default()),
            inp_min_velocity: DataArrayOrVec::new(dofs, 0.0),
            inp_min_acceleration: DataArrayOrVec::new(dofs, 0.0),
            inp_per_dof_control_interface: DataArrayOrVec::new(dofs, ControlInterface::default()),
            inp_per_dof_synchronization: DataArrayOrVec::new(dofs, Synchronization::default()),
            new_phase_control: DataArrayOrVec::new(dofs, 0.0),
            pd: DataArrayOrVec::new(dofs, 0.0),
            possible_t_syncs: vec![0.0; 3 * dofs.unwrap_or(DOF) + 1],
            idx: vec![0; 3 * dofs.unwrap_or(DOF) + 1],
            eps: f64::EPSILON,
            return_error_at_maximal_duration: true,
            degrees_of_freedom: dofs.unwrap_or(DOF),
        }
    }

    // Allowing mutable reference to self for the sake of better performance.
    #[allow(clippy::wrong_self_convention)]
    fn is_input_collinear(
        &mut self,
        inp: &InputParameter<DOF>,
        limiting_direction: Direction,
        limiting_dof: usize,
    ) -> bool {
        // Check that vectors pd, v0, a0, vf, af are collinear
        for dof in 0..self.degrees_of_freedom {
            self.pd[dof] = inp.target_position[dof] - inp.current_position[dof];
        }

        let mut scale_vector: Option<&DataArrayOrVec<f64, DOF>> = None;
        let mut scale_dof: Option<usize> = None;
        for dof in 0..self.degrees_of_freedom {
            if self.inp_per_dof_synchronization[dof] != Synchronization::Phase {
                continue;
            }

            if self.inp_per_dof_control_interface[dof] == ControlInterface::Position
                && self.pd[dof].abs() > self.eps
            {
                scale_vector = Some(&self.pd);
                scale_dof = Some(dof);
                break;
            } else if inp.current_velocity[dof].abs() > self.eps {
                scale_vector = Some(&inp.current_velocity);
                scale_dof = Some(dof);
                break;
            } else if inp.current_acceleration[dof].abs() > self.eps {
                scale_vector = Some(&inp.current_acceleration);
                scale_dof = Some(dof);
                break;
            } else if inp.target_velocity[dof].abs() > self.eps {
                scale_vector = Some(&inp.target_velocity);
                scale_dof = Some(dof);
                break;
            } else if inp.target_acceleration[dof].abs() > self.eps {
                scale_vector = Some(&inp.target_acceleration);
                scale_dof = Some(dof);
                break;
            }
        }

        if scale_dof.is_none() {
            return false; // Zero everywhere is in theory collinear, but that trivial case is better handled elsewhere
        }

        let scale = scale_vector.unwrap()[scale_dof.unwrap()];
        let pd_scale = self.pd[scale_dof.unwrap()] / scale;
        let v0_scale = inp.current_velocity[scale_dof.unwrap()] / scale;
        let vf_scale = inp.target_velocity[scale_dof.unwrap()] / scale;
        let a0_scale = inp.current_acceleration[scale_dof.unwrap()] / scale;
        let af_scale = inp.target_acceleration[scale_dof.unwrap()] / scale;

        let scale_limiting = scale_vector.unwrap()[limiting_dof];
        let mut control_limiting = if limiting_direction == Direction::UP {
            inp.max_jerk[limiting_dof]
        } else {
            -inp.max_jerk[limiting_dof]
        };
        if inp.max_jerk[limiting_dof].is_infinite() {
            control_limiting = if limiting_direction == Direction::UP {
                inp.max_acceleration[limiting_dof]
            } else {
                self.inp_min_acceleration[limiting_dof]
            };
        }

        for dof in 0..self.degrees_of_freedom {
            if self.inp_per_dof_synchronization[dof] != Synchronization::Phase {
                continue;
            }

            let current_scale = scale_vector.unwrap()[dof];
            if (self.inp_per_dof_control_interface[dof] == ControlInterface::Position
                && (self.pd[dof] - pd_scale * current_scale).abs() > self.eps)
                || (inp.current_velocity[dof] - v0_scale * current_scale).abs() > self.eps
                || (inp.current_acceleration[dof] - a0_scale * current_scale).abs() > self.eps
                || (inp.target_velocity[dof] - vf_scale * current_scale).abs() > self.eps
                || (inp.target_acceleration[dof] - af_scale * current_scale).abs() > self.eps
            {
                return false;
            }

            self.new_phase_control[dof] = control_limiting * current_scale / scale_limiting;
        }

        true
    }

    fn synchronize(
        &mut self,
        t_min: Option<f64>,
        t_sync: &mut f64,
        limiting_dof: &mut Option<usize>,
        profiles: &mut DataArrayOrVec<Profile, { DOF }>,
        discrete_duration: bool,
        delta_time: f64,
    ) -> bool {
        // Check for (degrees_of_freedom == 1 && !t_min && !discrete_duration) is now outside

        // Possible t_syncs are the start times of the intervals and optional t_min
        let mut any_interval = false;
        for dof in 0..self.degrees_of_freedom {
            // Ignore DoFs without synchronization here
            if self.inp_per_dof_synchronization[dof] == Synchronization::None {
                self.possible_t_syncs[dof] = 0.0;
                self.possible_t_syncs[self.degrees_of_freedom + dof] = f64::INFINITY;
                self.possible_t_syncs[2 * self.degrees_of_freedom + dof] = f64::INFINITY;
                continue;
            }

            self.possible_t_syncs[dof] = self.blocks[dof].t_min;
            self.possible_t_syncs[self.degrees_of_freedom + dof] =
                if let Some(a) = &self.blocks[dof].a {
                    a.right
                } else {
                    f64::INFINITY
                };
            self.possible_t_syncs[2 * self.degrees_of_freedom + dof] =
                if let Some(b) = &self.blocks[dof].b {
                    b.right
                } else {
                    f64::INFINITY
                };
            any_interval |= self.blocks[dof].a.is_some() || self.blocks[dof].b.is_some();
        }
        self.possible_t_syncs[3 * self.degrees_of_freedom] = t_min.unwrap_or(f64::INFINITY);
        any_interval |= t_min.is_some();

        if discrete_duration {
            for possible_t_sync in &mut self.possible_t_syncs {
                if possible_t_sync.is_infinite() {
                    continue;
                }

                let remainder = *possible_t_sync % delta_time; // in [0, delta_time)
                if remainder > self.eps {
                    *possible_t_sync += delta_time - remainder;
                }
            }
        }

        // Test them in sorted order
        // Setting up the range for `idx_end`
        let idx_end = if any_interval {
            self.idx.len()
        } else {
            self.degrees_of_freedom
        };

        // Initialize the range similar to `std::iota`
        for i in 0..idx_end {
            self.idx[i] = i;
        }

        // Sort the values in the range
        self.idx[0..idx_end].sort_by(|&i, &j| {
            self.possible_t_syncs[i]
                .partial_cmp(&self.possible_t_syncs[j])
                .unwrap()
        });

        // Start at last tmin (or worse)
        for &i in &self.idx[(self.degrees_of_freedom - 1)..] {
            let possible_t_sync = self.possible_t_syncs[i];
            let mut is_blocked = false;
            for dof in 0..self.degrees_of_freedom {
                if self.inp_per_dof_synchronization[dof] == Synchronization::None {
                    continue; // inner dof loop
                }
                if self.blocks[dof].is_blocked(possible_t_sync) {
                    is_blocked = true;
                    break; // inner dof loop
                }
            }
            if is_blocked || possible_t_sync < t_min.unwrap_or(0.0) || possible_t_sync.is_infinite()
            {
                continue;
            }

            *t_sync = possible_t_sync;
            if i == 3 * self.degrees_of_freedom {
                // Optional t_min
                *limiting_dof = None;
                return true;
            }

            let div = i / self.degrees_of_freedom;
            *limiting_dof = Some(i % self.degrees_of_freedom);
            match div {
                0 => {
                    profiles[limiting_dof.unwrap()] =
                        self.blocks[limiting_dof.unwrap()].p_min.clone();
                }
                1 => {
                    profiles[limiting_dof.unwrap()] = self.blocks[limiting_dof.unwrap()]
                        .a
                        .clone()
                        .unwrap()
                        .profile;
                }
                2 => {
                    profiles[limiting_dof.unwrap()] = self.blocks[limiting_dof.unwrap()]
                        .b
                        .clone()
                        .unwrap()
                        .profile;
                }
                _ => {}
            }
            return true;
        }

        false
    }

    /// Calculate the time-optimal waypoint-based trajectory.
    pub fn calculate<T: RuckigErrorHandler>(
        &mut self,
        inp: &InputParameter<DOF>,
        traj: &mut Trajectory<DOF>,
        delta_time: f64,
    ) -> Result<RuckigResult, RuckigError> {
        for dof in 0..self.degrees_of_freedom {
            let p = &mut traj.profiles[0][dof];

            self.inp_min_velocity[dof] = inp
                .min_velocity
                .as_ref()
                .map_or(-inp.max_velocity[dof], |v| v[dof]);

            self.inp_min_acceleration[dof] = inp
                .min_acceleration
                .as_ref()
                .map_or(-inp.max_acceleration[dof], |v| v[dof]);

            self.inp_per_dof_control_interface =
                DataArrayOrVec::new(Some(self.degrees_of_freedom), inp.control_interface.clone());
            if let Some(per_dof_control_interface) = &inp.per_dof_control_interface {
                for (dof, value) in per_dof_control_interface.iter().enumerate() {
                    *self.inp_per_dof_control_interface.get_mut(dof).unwrap() = value.clone();
                }
            }

            self.inp_per_dof_synchronization =
                DataArrayOrVec::new(Some(self.degrees_of_freedom), inp.synchronization.clone());
            if let Some(per_dof_synchronization) = &inp.per_dof_synchronization {
                for (dof, value) in per_dof_synchronization.iter().enumerate() {
                    *self.inp_per_dof_synchronization.get_mut(dof).unwrap() = value.clone();
                }
            }

            if !inp.enabled[dof] {
                if let Some(last) = p.p.last_mut() {
                    *last = inp.current_position[dof];
                }
                if let Some(last) = p.v.last_mut() {
                    *last = inp.current_velocity[dof];
                }
                if let Some(last) = p.a.last_mut() {
                    *last = inp.current_acceleration[dof];
                }
                if let Some(last) = p.t_sum.last_mut() {
                    *last = 0.0;
                }

                self.blocks[dof].t_min = 0.0;
                self.blocks[dof].a = None;
                self.blocks[dof].b = None;
                continue;
            }

            // Calculate brake (if input exceeds or will exceed limits)
            match self.inp_per_dof_control_interface[dof] {
                ControlInterface::Position => {
                    if !inp.max_jerk[dof].is_infinite() {
                        p.brake.get_position_brake_trajectory(
                            inp.current_velocity[dof],
                            inp.current_acceleration[dof],
                            inp.max_velocity[dof],
                            inp.min_velocity
                                .as_ref()
                                .and_then(|v| v.get(dof))
                                .cloned()
                                .unwrap_or(-inp.max_velocity[dof]),
                            inp.max_acceleration[dof],
                            inp.min_acceleration
                                .as_ref()
                                .and_then(|v| v.get(dof))
                                .cloned()
                                .unwrap_or(-inp.max_acceleration[dof]),
                            inp.max_jerk[dof],
                        );
                    } else if !inp.max_acceleration[dof].is_infinite() {
                        p.brake.get_second_order_position_brake_trajectory(
                            inp.current_velocity[dof],
                            inp.max_velocity[dof],
                            inp.min_velocity
                                .as_ref()
                                .and_then(|v| v.get(dof))
                                .cloned()
                                .unwrap_or(-inp.max_velocity[dof]),
                            inp.max_acceleration[dof],
                            inp.min_acceleration
                                .as_ref()
                                .and_then(|v| v.get(dof))
                                .cloned()
                                .unwrap_or(-inp.max_acceleration[dof]),
                        );
                    }
                    p.set_boundary(
                        &inp.current_position[dof],
                        &inp.current_velocity[dof],
                        &inp.current_acceleration[dof],
                        &inp.target_position[dof],
                        &inp.target_velocity[dof],
                        &inp.target_acceleration[dof],
                    );
                }
                ControlInterface::Velocity => {
                    if !inp.max_jerk[dof].is_infinite() {
                        p.brake.get_velocity_brake_trajectory(
                            inp.current_acceleration[dof],
                            inp.max_acceleration[dof],
                            inp.min_acceleration
                                .as_ref()
                                .and_then(|v| v.get(dof))
                                .cloned()
                                .unwrap_or(-inp.max_acceleration[dof]),
                            inp.max_jerk[dof],
                        );
                    } else {
                        p.brake.get_second_order_velocity_brake_trajectory();
                    }
                    p.set_boundary_for_velocity(
                        inp.current_position[dof],
                        inp.current_velocity[dof],
                        inp.current_acceleration[dof],
                        inp.target_velocity[dof],
                        inp.target_acceleration[dof],
                    );
                }
                _ => {}
            }
            // Finalize pre & post-trajectories
            if !inp.max_jerk[dof].is_infinite() {
                p.brake.finalize(&mut p.p[0], &mut p.v[0], &mut p.a[0]);
            } else if !inp.max_acceleration[dof].is_infinite() {
                p.brake
                    .finalize_second_order(&mut p.p[0], &mut p.v[0], &mut p.a[0]);
            }

            let mut found_profile = false;
            match self.inp_per_dof_control_interface[dof] {
                ControlInterface::Position => {
                    if !inp.max_jerk[dof].is_infinite() {
                        let mut step1 = PositionThirdOrderStep1::new(
                            p.p[0],
                            p.v[0],
                            p.a[0],
                            p.pf,
                            p.vf,
                            p.af,
                            inp.max_velocity[dof],
                            inp.min_velocity
                                .as_ref()
                                .map_or(-inp.max_velocity[dof], |v| v[dof]),
                            inp.max_acceleration[dof],
                            inp.min_acceleration
                                .as_ref()
                                .map_or(-inp.max_acceleration[dof], |v| v[dof]),
                            inp.max_jerk[dof],
                        );
                        found_profile = step1.get_profile(p, &mut self.blocks[dof]);
                    } else if !inp.max_acceleration[dof].is_infinite() {
                        let mut step1 = PositionSecondOrderStep1::new(
                            p.p[0],
                            p.v[0],
                            p.pf,
                            p.vf,
                            inp.max_velocity[dof],
                            inp.min_velocity
                                .as_ref()
                                .map_or(-inp.max_velocity[dof], |v| v[dof]),
                            inp.max_acceleration[dof],
                            inp.min_acceleration
                                .as_ref()
                                .map_or(-inp.max_acceleration[dof], |v| v[dof]),
                        );
                        found_profile = step1.get_profile(p, &mut self.blocks[dof]);
                    } else {
                        let mut step1 = PositionFirstOrderStep1::new(
                            p.p[0],
                            p.pf,
                            inp.max_velocity[dof],
                            inp.min_velocity
                                .as_ref()
                                .map_or(-inp.max_velocity[dof], |v| v[dof]),
                        );
                        found_profile = step1.get_profile(p, &mut self.blocks[dof]);
                    }
                }
                ControlInterface::Velocity => {
                    if !inp.max_jerk[dof].is_infinite() {
                        let mut step1 = VelocityThirdOrderStep1::new(
                            p.v[0],
                            p.a[0],
                            p.vf,
                            p.af,
                            inp.max_acceleration[dof],
                            inp.min_acceleration
                                .as_ref()
                                .map_or(-inp.max_acceleration[dof], |v| v[dof]),
                            inp.max_jerk[dof],
                        );
                        found_profile = step1.get_profile(p, &mut self.blocks[dof]);
                    } else {
                        let mut step1 = VelocitySecondOrderStep1::new(
                            p.v[0],
                            p.vf,
                            inp.max_acceleration[dof],
                            inp.min_acceleration
                                .as_ref()
                                .map_or(-inp.max_acceleration[dof], |v| v[dof]),
                        );
                        found_profile = step1.get_profile(p, &mut self.blocks[dof]);
                    }
                }
                ControlInterface::Acceleration => {}
            }

            if !found_profile {
                let has_zero_limits = inp.max_acceleration[dof] == 0.0
                    || inp
                        .min_acceleration
                        .as_ref()
                        .map_or(-inp.max_acceleration[dof], |v| v[dof])
                        == 0.0
                    || inp.max_jerk[dof] == 0.0;
                if has_zero_limits {
                    return T::handle_calculator_error(
                        &format!(
                            "zero limits conflict in step 1, dof: {} input: {}",
                            dof, inp
                        )
                        .to_owned(),
                        RuckigResult::ErrorZeroLimits,
                    );
                }
                return T::handle_calculator_error(
                    &format!("error in step 1, dof: {} input: {}", dof, inp).to_owned(),
                    RuckigResult::ErrorExecutionTimeCalculation,
                );
            }

            traj.independent_min_durations[dof] = self.blocks[dof].t_min;
        }
        let discrete_duration = inp.duration_discretization == DurationDiscretization::Discrete;
        if self.degrees_of_freedom == 1 && inp.minimum_duration.is_none() && !discrete_duration {
            traj.duration = self.blocks[0].t_min;
            traj.profiles[0][0] = self.blocks[0].p_min.clone();
            traj.cumulative_times[0] = traj.duration;
            return Ok(RuckigResult::Working);
        }

        let mut limiting_dof: Option<usize> = None; // The DoF that doesn't need step 2
        let found_synchronization = self.synchronize(
            inp.minimum_duration,
            &mut traj.duration,
            &mut limiting_dof,
            &mut traj.profiles[0],
            discrete_duration,
            delta_time,
        );
        if !found_synchronization {
            let mut has_zero_limits = false;
            for dof in 0..self.degrees_of_freedom {
                if inp.max_acceleration[dof] == 0.0
                    || inp
                        .min_acceleration
                        .as_ref()
                        .map_or(-inp.max_acceleration[dof], |v| v[dof])
                        == 0.0
                    || inp.max_jerk[dof] == 0.0
                {
                    has_zero_limits = true;
                    break;
                }
            }

            if has_zero_limits {
                return T::handle_calculator_error(
                    &format!("zero limits conflict with other degrees of freedom in time synchronization {}", traj.duration),
                    RuckigResult::ErrorZeroLimits);
            }
            return T::handle_calculator_error(
                &format!("error in time synchronization: {}", traj.duration),
                RuckigResult::ErrorSynchronizationCalculation,
            );
        }
        // None Synchronization
        for dof in 0..self.degrees_of_freedom {
            if inp.enabled[dof] && self.inp_per_dof_synchronization[dof] == Synchronization::None {
                traj.profiles[0][dof] = self.blocks[dof].p_min.clone();
                if self.blocks[dof].t_min > traj.duration {
                    traj.duration = self.blocks[dof].t_min;
                    limiting_dof = Some(dof);
                }
            }
        }
        traj.cumulative_times[0] = traj.duration;

        if self.return_error_at_maximal_duration && traj.duration > 7.6e3 {
            return Ok(RuckigResult::ErrorTrajectoryDuration);
        }

        if (traj.duration - 0.0).abs() < f64::EPSILON {
            // Copy all profiles for end state
            for dof in 0..self.degrees_of_freedom {
                traj.profiles[0][dof] = self.blocks[dof].p_min.clone();
            }
            return Ok(RuckigResult::Working);
        }

        if !discrete_duration
            && self
                .inp_per_dof_synchronization
                .iter()
                .all(|s| s == &Synchronization::None)
        {
            return Ok(RuckigResult::Working);
        }

        // Phase Synchronization
        if let Some(limiting_dof_value) = limiting_dof {
            if self
                .inp_per_dof_synchronization
                .iter()
                .any(|s| s == &Synchronization::Phase)
            {
                let p_limiting = traj.profiles[0][limiting_dof_value].clone();
                if self.is_input_collinear(inp, p_limiting.direction, limiting_dof_value) {
                    let mut found_time_synchronization = true;
                    for dof in 0..self.degrees_of_freedom {
                        if !inp.enabled[dof]
                            || dof == limiting_dof_value
                            || self.inp_per_dof_synchronization[dof] != Synchronization::Phase
                        {
                            continue;
                        }

                        let p = &mut traj.profiles[0][dof];
                        let t_profile = traj.duration - p.brake.duration - p.accel.duration;

                        p.t = p_limiting.t; // Copy timing information from limiting DoF
                        p.control_signs = p_limiting.control_signs.clone();

                        match self.inp_per_dof_control_interface[dof] {
                            ControlInterface::Position => match p.control_signs {
                                ControlSigns::UDDU => {
                                    if !inp.max_jerk[dof].is_infinite() {
                                        found_time_synchronization &= p.check_with_timing_full(
                                            ControlSigns::UDDU,
                                            ReachedLimits::None,
                                            t_profile,
                                            self.new_phase_control[dof],
                                            inp.max_velocity[dof],
                                            self.inp_min_velocity[dof],
                                            inp.max_acceleration[dof],
                                            self.inp_min_acceleration[dof],
                                            inp.max_jerk[dof],
                                        );
                                    } else if !inp.max_acceleration[dof].is_infinite() {
                                        found_time_synchronization &= p
                                            .check_for_second_order_with_timing_full(
                                                ControlSigns::UDDU,
                                                ReachedLimits::None,
                                                t_profile,
                                                self.new_phase_control[dof],
                                                -self.new_phase_control[dof],
                                                inp.max_velocity[dof],
                                                self.inp_min_velocity[dof],
                                                inp.max_acceleration[dof],
                                                self.inp_min_acceleration[dof],
                                            );
                                    } else {
                                        found_time_synchronization &= p
                                            .check_for_first_order_with_timing_full(
                                                ControlSigns::UDDU,
                                                ReachedLimits::None,
                                                t_profile,
                                                self.new_phase_control[dof],
                                                inp.max_velocity[dof],
                                                self.inp_min_velocity[dof],
                                            );
                                    }
                                }
                                ControlSigns::UDUD => {
                                    if !inp.max_jerk[dof].is_infinite() {
                                        found_time_synchronization &= p.check_with_timing_full(
                                            ControlSigns::UDUD,
                                            ReachedLimits::None,
                                            t_profile,
                                            self.new_phase_control[dof],
                                            inp.max_velocity[dof],
                                            self.inp_min_velocity[dof],
                                            inp.max_acceleration[dof],
                                            self.inp_min_acceleration[dof],
                                            inp.max_jerk[dof],
                                        );
                                    } else {
                                        found_time_synchronization &= p
                                            .check_for_second_order_with_timing_full(
                                                ControlSigns::UDUD,
                                                ReachedLimits::None,
                                                t_profile,
                                                self.new_phase_control[dof],
                                                -self.new_phase_control[dof],
                                                inp.max_velocity[dof],
                                                self.inp_min_velocity[dof],
                                                inp.max_acceleration[dof],
                                                self.inp_min_acceleration[dof],
                                            );
                                    }
                                }
                            },
                            ControlInterface::Velocity => match p.control_signs {
                                ControlSigns::UDDU => {
                                    if !inp.max_jerk[dof].is_infinite() {
                                        found_time_synchronization &= p
                                            .check_for_velocity_with_timing_full(
                                                t_profile,
                                                ControlSigns::UDDU,
                                                ReachedLimits::None,
                                                self.new_phase_control[dof],
                                                inp.max_acceleration[dof],
                                                self.inp_min_acceleration[dof],
                                                inp.max_jerk[dof],
                                            );
                                    } else {
                                        found_time_synchronization &= p
                                            .check_for_second_order_velocity_with_timing_a_limits(
                                                ControlSigns::UDDU,
                                                ReachedLimits::None,
                                                t_profile,
                                                self.new_phase_control[dof],
                                                inp.max_acceleration[dof],
                                                self.inp_min_acceleration[dof],
                                            );
                                    }
                                }
                                ControlSigns::UDUD => {
                                    if !inp.max_jerk[dof].is_infinite() {
                                        found_time_synchronization &= p
                                            .check_for_velocity_with_timing_full(
                                                t_profile,
                                                ControlSigns::UDUD,
                                                ReachedLimits::None,
                                                self.new_phase_control[dof],
                                                inp.max_acceleration[dof],
                                                self.inp_min_acceleration[dof],
                                                inp.max_jerk[dof],
                                            );
                                    } else {
                                        found_time_synchronization &= p
                                            .check_for_second_order_velocity_with_timing_a_limits(
                                                ControlSigns::UDUD,
                                                ReachedLimits::None,
                                                t_profile,
                                                self.new_phase_control[dof],
                                                inp.max_acceleration[dof],
                                                self.inp_min_acceleration[dof],
                                            );
                                    }
                                }
                            },
                            _ => {}
                        }

                        p.limits = p_limiting.limits; // After check method call to set correct limits
                    }

                    if found_time_synchronization
                        && self
                            .inp_per_dof_synchronization
                            .iter()
                            .all(|s| s == &Synchronization::Phase || s == &Synchronization::None)
                    {
                        return Ok(RuckigResult::Working);
                    }
                }
            }
        }

        // Time Synchronization
        for dof in 0..self.degrees_of_freedom {
            let skip_synchronization = (Some(dof) == limiting_dof
                || self.inp_per_dof_synchronization[dof] == Synchronization::None)
                && !discrete_duration;
            if !inp.enabled[dof] || skip_synchronization {
                continue;
            }

            let p = &mut traj.profiles[0][dof];
            let t_profile = traj.duration - p.brake.duration - p.accel.duration;

            if self.inp_per_dof_synchronization[dof] == Synchronization::TimeIfNecessary
                && inp.target_velocity[dof].abs() < self.eps
                && inp.target_acceleration[dof].abs() < self.eps
            {
                traj.profiles[0][dof] = self.blocks[dof].p_min.clone();
                continue;
            }

            // Check if the final time corresponds to an extremal profile calculated in step 1
            if (t_profile - self.blocks[dof].t_min).abs() < 2.0 * self.eps {
                traj.profiles[0][dof] = self.blocks[dof].p_min.clone();
                continue;
            } else if let Some(a) = &self.blocks[dof].a {
                if (t_profile - a.right).abs() < 2.0 * self.eps {
                    traj.profiles[0][dof] = a.profile.clone();
                    continue;
                }
            } else if let Some(b) = &self.blocks[dof].b {
                if (t_profile - b.right).abs() < 2.0 * self.eps {
                    traj.profiles[0][dof] = b.profile.clone();
                    continue;
                }
            }

            let mut found_time_synchronization = false;
            match self.inp_per_dof_control_interface[dof] {
                ControlInterface::Position => {
                    if !inp.max_jerk[dof].is_infinite() {
                        let mut step2 = PositionThirdOrderStep2::new(
                            t_profile,
                            p.p[0],
                            p.v[0],
                            p.a[0],
                            p.pf,
                            p.vf,
                            p.af,
                            inp.max_velocity[dof],
                            self.inp_min_velocity[dof],
                            inp.max_acceleration[dof],
                            self.inp_min_acceleration[dof],
                            inp.max_jerk[dof],
                        );
                        found_time_synchronization = step2.get_profile(p);
                    } else if !inp.max_acceleration[dof].is_infinite() {
                        let mut step2 = PositionSecondOrderStep2::new(
                            t_profile,
                            p.p[0],
                            p.v[0],
                            p.pf,
                            p.vf,
                            inp.max_velocity[dof],
                            self.inp_min_velocity[dof],
                            inp.max_acceleration[dof],
                            self.inp_min_acceleration[dof],
                        );
                        found_time_synchronization = step2.get_profile(p);
                    } else {
                        let mut step2 = PositionFirstOrderStep2::new(
                            t_profile,
                            p.p[0],
                            p.pf,
                            inp.max_velocity[dof],
                            self.inp_min_velocity[dof],
                        );
                        found_time_synchronization = step2.get_profile(p);
                    }
                }
                ControlInterface::Velocity => {
                    if !inp.max_jerk[dof].is_infinite() {
                        let mut step2 = VelocityThirdOrderStep2::new(
                            t_profile,
                            p.v[0],
                            p.a[0],
                            p.vf,
                            p.af,
                            inp.max_acceleration[dof],
                            self.inp_min_acceleration[dof],
                            inp.max_jerk[dof],
                        );
                        found_time_synchronization = step2.get_profile(p);
                    } else {
                        let mut step2 = VelocitySecondOrderStep2::new(
                            t_profile,
                            p.v[0],
                            p.vf,
                            inp.max_acceleration[dof],
                            self.inp_min_acceleration[dof],
                        );
                        found_time_synchronization = step2.get_profile(p);
                    }
                }
                _ => {}
            }

            if !found_time_synchronization {
                return T::handle_calculator_error(
                    &format!(
                        "error in step 2 in dof: {} for t sync: {} input: {}",
                        dof, traj.duration, inp
                    ),
                    RuckigResult::ErrorExecutionTimeCalculation,
                );
            }

            // Uncomment the following line if you want to debug
            // println!("{} profile step2: {}", dof, p.to_string());
        }

        Ok(RuckigResult::Working)
    }
}
