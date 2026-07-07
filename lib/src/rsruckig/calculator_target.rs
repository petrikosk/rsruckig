//! Calculation of a state-to-state trajectory.
use crate::brake::{
    stop_distance_second_order, stop_distance_third_order, velocity_cap_second_order,
    velocity_cap_third_order,
};
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

use crate::alloc::{vec, vec::Vec, format};

#[derive(Debug, Clone)]
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
    inp_min_position: DataArrayOrVec<f64, DOF>,
    inp_max_position: DataArrayOrVec<f64, DOF>,
    eff_min_velocity: DataArrayOrVec<f64, DOF>,
    eff_max_velocity: DataArrayOrVec<f64, DOF>,
    position_limits_active: bool,
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
            inp_min_position: DataArrayOrVec::new(dofs, f64::NEG_INFINITY),
            inp_max_position: DataArrayOrVec::new(dofs, f64::INFINITY),
            eff_min_velocity: DataArrayOrVec::new(dofs, 0.0),
            eff_max_velocity: DataArrayOrVec::new(dofs, 0.0),
            position_limits_active: false,
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

    /// Scaled tolerance for position-limit checks. Accounts for the target
    /// precision (1e-8) of the profiles and the +EPS paddings of brake phases.
    #[inline]
    fn position_limit_tolerance(limit: f64) -> f64 {
        1e-8 * limit.abs().max(1.0)
    }

    /// Stopping distances (upwards, downwards) from the given state under the
    /// kinematic limits of `dof`. Zero for first-order (monotone) profiles.
    fn stop_distances(&self, dof: usize, inp: &InputParameter<DOF>, v: f64, a: f64) -> (f64, f64) {
        let j_max = inp.max_jerk[dof];
        let a_max = inp.max_acceleration[dof];
        let a_min = self.inp_min_acceleration[dof];
        if j_max > 0.0 && j_max.is_finite() {
            (
                stop_distance_third_order(v, a, a_min, j_max),
                stop_distance_third_order(-v, -a, a_max, j_max),
            )
        } else if a_max.is_finite() || a_min.is_finite() {
            (
                stop_distance_second_order(v, a_min),
                stop_distance_second_order(-v, a_max),
            )
        } else {
            (0.0, 0.0)
        }
    }

    /// Check a single profile against the position limits of `dof`.
    /// Returns (upper_limit_violated, lower_limit_violated). Besides the extrema
    /// of the profile itself, the end state must leave enough margin for a full
    /// stop inside the limits, so that the system can always stay in the workspace.
    /// Start position of a profile, including a possible brake pre-trajectory
    #[inline]
    fn profile_start_position(profile: &Profile) -> f64 {
        if profile.brake.duration > 0.0 {
            profile.brake.p[0]
        } else {
            profile.p[0]
        }
    }

    fn profile_position_violations(
        &self,
        dof: usize,
        inp: &InputParameter<DOF>,
        profile: &Profile,
    ) -> (bool, bool) {
        // If the current position already lies outside the limits, the
        // trajectory back inside is still allowed - it must only never move
        // further outside than its start
        let p_start = Self::profile_start_position(profile);
        let max_pos = self.inp_max_position[dof].max(p_start);
        let min_pos = self.inp_min_position[dof].min(p_start);
        let tol_up = Self::position_limit_tolerance(max_pos);
        let tol_down = Self::position_limit_tolerance(min_pos);

        let ext = profile.get_position_extrema();
        let mut up = ext.max > max_pos + tol_up;
        let mut down = ext.min < min_pos - tol_down;

        let (stop_up, stop_down) = self.stop_distances(dof, inp, profile.v[7], profile.a[7]);
        up |= profile.p[7] + stop_up > max_pos + tol_up;
        down |= profile.p[7] - stop_down < min_pos - tol_down;

        (up, down)
    }

    /// Check all candidate profiles of the step-1 block of `dof` against the
    /// position limits. Returns (upper_violated, lower_violated) over all of them,
    /// as the synchronization may select any of the block profiles later on.
    fn block_position_violations(&self, dof: usize) -> (bool, bool) {
        // All block profiles share the same start state (see
        // profile_position_violations for the rationale of relaxing by it)
        let p_start = Self::profile_start_position(&self.blocks[dof].p_min);
        let max_pos = self.inp_max_position[dof].max(p_start);
        let min_pos = self.inp_min_position[dof].min(p_start);
        let tol_up = Self::position_limit_tolerance(max_pos);
        let tol_down = Self::position_limit_tolerance(min_pos);

        let mut up = false;
        let mut down = false;
        let mut check = |profile: &Profile| {
            let ext = profile.get_position_extrema();
            up |= ext.max > max_pos + tol_up;
            down |= ext.min < min_pos - tol_down;
        };

        check(&self.blocks[dof].p_min);
        if let Some(a) = &self.blocks[dof].a {
            check(&a.profile);
        }
        if let Some(b) = &self.blocks[dof].b {
            check(&b.profile);
        }
        (up, down)
    }

    /// Run step 1 for a position-interface DoF with the current effective
    /// velocity limits.
    fn run_position_step1(
        &mut self,
        dof: usize,
        inp: &InputParameter<DOF>,
        p: &mut Profile,
    ) -> bool {
        if !inp.max_jerk[dof].is_infinite() {
            let mut step1 = PositionThirdOrderStep1::new(
                p.p[0],
                p.v[0],
                p.a[0],
                p.pf,
                p.vf,
                p.af,
                self.eff_max_velocity[dof],
                self.eff_min_velocity[dof],
                inp.max_acceleration[dof],
                self.inp_min_acceleration[dof],
                inp.max_jerk[dof],
            );
            step1.get_profile(p, &mut self.blocks[dof])
        } else if !inp.max_acceleration[dof].is_infinite() {
            let mut step1 = PositionSecondOrderStep1::new(
                p.p[0],
                p.v[0],
                p.pf,
                p.vf,
                self.eff_max_velocity[dof],
                self.eff_min_velocity[dof],
                inp.max_acceleration[dof],
                self.inp_min_acceleration[dof],
            );
            step1.get_profile(p, &mut self.blocks[dof])
        } else {
            let mut step1 = PositionFirstOrderStep1::new(
                p.p[0],
                p.pf,
                self.eff_max_velocity[dof],
                self.eff_min_velocity[dof],
            );
            step1.get_profile(p, &mut self.blocks[dof])
        }
    }

    /// Active position-limit enforcement after step 1: if any block profile of
    /// `dof` would leave the position limits, shrink the effective velocity
    /// limits by bisection and re-run step 1, so that the whole feasible
    /// duration interval respects the limits. Returns false if no compliant
    /// profile was found within the iteration budget.
    fn enforce_position_limits_step1(
        &mut self,
        dof: usize,
        inp: &InputParameter<DOF>,
        p: &mut Profile,
    ) -> bool {
        const MAX_ITERS: usize = 24;

        let (up_viol, down_viol) = self.block_position_violations(dof);
        if !up_viol && !down_viol {
            return true;
        }

        let mut search_up = up_viol;
        let mut search_down = down_viol;
        let mut lo_up = p.vf.max(0.0);
        let mut hi_up = self.eff_max_velocity[dof];
        let mut lo_down = (-p.vf).max(0.0);
        let mut hi_down = -self.eff_min_velocity[dof];
        let mut best: Option<(f64, f64)> = None;

        for _ in 0..MAX_ITERS {
            if search_up {
                self.eff_max_velocity[dof] = 0.5 * (lo_up + hi_up);
            }
            if search_down {
                self.eff_min_velocity[dof] = -0.5 * (lo_down + hi_down);
            }

            if !self.run_position_step1(dof, inp, p) {
                // Too restrictive for any feasible profile: enlarge again
                if search_up {
                    lo_up = self.eff_max_velocity[dof];
                }
                if search_down {
                    lo_down = -self.eff_min_velocity[dof];
                }
                continue;
            }

            let (up, down) = self.block_position_violations(dof);
            if up {
                search_up = true;
                hi_up = self.eff_max_velocity[dof];
            } else if search_up {
                lo_up = self.eff_max_velocity[dof];
            }
            if down {
                search_down = true;
                hi_down = -self.eff_min_velocity[dof];
            } else if search_down {
                lo_down = -self.eff_min_velocity[dof];
            }

            if !up && !down {
                best = Some((self.eff_max_velocity[dof], self.eff_min_velocity[dof]));
                let converged_up = !search_up || (hi_up - lo_up) <= 1e-9 * hi_up.max(1.0);
                let converged_down =
                    !search_down || (hi_down - lo_down) <= 1e-9 * hi_down.max(1.0);
                if converged_up && converged_down {
                    break;
                }
            }
        }

        match best {
            Some((v_up, v_down)) => {
                self.eff_max_velocity[dof] = v_up;
                self.eff_min_velocity[dof] = v_down;
                // Leave the block in the compliant state
                self.run_position_step1(dof, inp, p)
            }
            None => false,
        }
    }

    /// Last-resort repair when a synchronized (step 2 / phase-sync) profile
    /// leaves the position limits: shrink the effective velocity limits (or the
    /// clamped target velocity for the velocity interface) by bisection and
    /// re-run step 2 for this DoF only, keeping the trajectory duration fixed.
    fn repair_profile_step2(
        &mut self,
        dof: usize,
        inp: &InputParameter<DOF>,
        duration: f64,
        profile: &mut Profile,
    ) -> bool {
        const MAX_ITERS: usize = 16;

        let original = *profile;
        let t_profile = duration - profile.brake.duration - profile.accel.duration;

        match self.inp_per_dof_control_interface[dof] {
            ControlInterface::Position => {
                let mut lo_up = profile.vf.max(0.0);
                let mut hi_up = self.eff_max_velocity[dof];
                let mut lo_down = (-profile.vf).max(0.0);
                let mut hi_down = -self.eff_min_velocity[dof];
                let mut search_up = false;
                let mut search_down = false;
                let mut v_up = self.eff_max_velocity[dof];
                let mut v_down = self.eff_min_velocity[dof];
                let mut best: Option<Profile> = None;

                let (up, down) = self.profile_position_violations(dof, inp, profile);
                search_up |= up;
                search_down |= down;
                if !up && !down {
                    return true;
                }

                for _ in 0..MAX_ITERS {
                    if search_up {
                        hi_up = hi_up.min(v_up);
                        v_up = 0.5 * (lo_up + hi_up);
                    }
                    if search_down {
                        hi_down = hi_down.min(-v_down);
                        v_down = -0.5 * (lo_down + hi_down);
                    }

                    *profile = original;
                    let found = if !inp.max_jerk[dof].is_infinite() {
                        let mut step2 = PositionThirdOrderStep2::new(
                            t_profile,
                            profile.p[0],
                            profile.v[0],
                            profile.a[0],
                            profile.pf,
                            profile.vf,
                            profile.af,
                            v_up,
                            v_down,
                            inp.max_acceleration[dof],
                            self.inp_min_acceleration[dof],
                            inp.max_jerk[dof],
                        );
                        step2.get_profile(profile)
                    } else if !inp.max_acceleration[dof].is_infinite() {
                        let mut step2 = PositionSecondOrderStep2::new(
                            t_profile,
                            profile.p[0],
                            profile.v[0],
                            profile.pf,
                            profile.vf,
                            v_up,
                            v_down,
                            inp.max_acceleration[dof],
                            self.inp_min_acceleration[dof],
                        );
                        step2.get_profile(profile)
                    } else {
                        let mut step2 = PositionFirstOrderStep2::new(
                            t_profile,
                            profile.p[0],
                            profile.pf,
                            v_up,
                            v_down,
                        );
                        step2.get_profile(profile)
                    };

                    if !found {
                        // Too restrictive for this fixed duration: enlarge again
                        if search_up {
                            lo_up = v_up;
                        }
                        if search_down {
                            lo_down = -v_down;
                        }
                        continue;
                    }

                    let (up, down) = self.profile_position_violations(dof, inp, profile);
                    if up {
                        search_up = true;
                        hi_up = v_up;
                    } else if search_up {
                        lo_up = v_up;
                    }
                    if down {
                        search_down = true;
                        hi_down = -v_down;
                    } else if search_down {
                        lo_down = -v_down;
                    }

                    if !up && !down {
                        best = Some(*profile);
                        break;
                    }
                }

                match best {
                    Some(repaired) => {
                        *profile = repaired;
                        true
                    }
                    None => {
                        *profile = original;
                        false
                    }
                }
            }
            ControlInterface::Velocity => {
                let mut lo = 0.0;
                let mut hi = profile.vf.abs();
                let sign = if profile.vf < 0.0 { -1.0 } else { 1.0 };
                let mut best: Option<Profile> = None;

                for _ in 0..MAX_ITERS {
                    let vf = sign * 0.5 * (lo + hi);
                    *profile = original;
                    profile.vf = vf;

                    let found = if !inp.max_jerk[dof].is_infinite() {
                        let mut step2 = VelocityThirdOrderStep2::new(
                            t_profile,
                            profile.v[0],
                            profile.a[0],
                            vf,
                            profile.af,
                            inp.max_acceleration[dof],
                            self.inp_min_acceleration[dof],
                            inp.max_jerk[dof],
                        );
                        step2.get_profile(profile)
                    } else {
                        let mut step2 = VelocitySecondOrderStep2::new(
                            t_profile,
                            profile.v[0],
                            vf,
                            inp.max_acceleration[dof],
                            self.inp_min_acceleration[dof],
                        );
                        step2.get_profile(profile)
                    };

                    if !found {
                        lo = 0.5 * (lo + hi);
                        continue;
                    }

                    let (up, down) = self.profile_position_violations(dof, inp, profile);
                    if up || down {
                        hi = vf.abs();
                    } else {
                        best = Some(*profile);
                        lo = vf.abs();
                        if hi - lo <= 1e-9 * hi.max(1.0) {
                            break;
                        }
                    }
                }

                match best {
                    Some(repaired) => {
                        *profile = repaired;
                        true
                    }
                    None => {
                        *profile = original;
                        false
                    }
                }
            }
            ControlInterface::Acceleration => false,
        }
    }

    /// Calculate the time-optimal state-to-state trajectory, enforcing optional
    /// position limits (`InputParameter::min_position` / `max_position`).
    pub fn calculate<T: RuckigErrorHandler>(
        &mut self,
        inp: &InputParameter<DOF>,
        traj: &mut Trajectory<DOF>,
        delta_time: f64,
    ) -> Result<RuckigResult, RuckigError> {
        let result = self.calculate_internal::<T>(inp, traj, delta_time)?;
        if result != RuckigResult::Working || !self.position_limits_active {
            return Ok(result);
        }

        for dof in 0..self.degrees_of_freedom {
            if !inp.enabled[dof]
                || self.inp_per_dof_control_interface[dof] == ControlInterface::Acceleration
            {
                continue;
            }

            let (up, down) = self.profile_position_violations(dof, inp, &traj.profiles[0][dof]);
            if !up && !down {
                continue;
            }

            let duration = traj.duration;
            let mut profile = traj.profiles[0][dof];
            if self.repair_profile_step2(dof, inp, duration, &mut profile) {
                traj.profiles[0][dof] = profile;
                continue;
            }

            T::handle_calculator_error(&format!(
                "trajectory of DoF {} exceeds position limits, input: {}",
                dof, inp
            ))?;
            return Ok(RuckigResult::ErrorPositionalLimits);
        }

        Ok(RuckigResult::Working)
    }

    fn calculate_internal<T: RuckigErrorHandler>(
        &mut self,
        inp: &InputParameter<DOF>,
        traj: &mut Trajectory<DOF>,
        delta_time: f64,
    ) -> Result<RuckigResult, RuckigError> {
        // Initialize per-dof control interface and synchronization once before the loop
        for i in 0..self.degrees_of_freedom {
            self.inp_per_dof_control_interface[i] = inp.control_interface;
            self.inp_per_dof_synchronization[i] = inp.synchronization;
        }
        if let Some(per_dof_control_interface) = &inp.per_dof_control_interface {
            for (i, value) in per_dof_control_interface.iter().enumerate() {
                self.inp_per_dof_control_interface[i] = *value;
            }
        }
        if let Some(per_dof_synchronization) = &inp.per_dof_synchronization {
            for (i, value) in per_dof_synchronization.iter().enumerate() {
                self.inp_per_dof_synchronization[i] = *value;
            }
        }

        self.position_limits_active = inp.max_position.is_some() || inp.min_position.is_some();
        for dof in 0..self.degrees_of_freedom {
            self.inp_max_position[dof] = inp
                .max_position
                .as_ref()
                .map_or(f64::INFINITY, |v| v[dof]);
            self.inp_min_position[dof] = inp
                .min_position
                .as_ref()
                .map_or(f64::NEG_INFINITY, |v| v[dof]);
        }

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

            self.eff_max_velocity[dof] = inp.max_velocity[dof];
            self.eff_min_velocity[dof] = self.inp_min_velocity[dof];

            if !inp.enabled[dof] {
                p.p[7] = inp.current_position[dof];
                p.v[7] = inp.current_velocity[dof];
                p.a[7] = inp.current_acceleration[dof];
                p.t_sum[6] = 0.0;

                self.blocks[dof].t_min = 0.0;
                self.blocks[dof].a = None;
                self.blocks[dof].b = None;
                continue;
            }

            // Use cached min values
            let min_vel = self.inp_min_velocity[dof];
            let min_acc = self.inp_min_acceleration[dof];

            // Calculate brake (if input exceeds or will exceed limits)
            match self.inp_per_dof_control_interface[dof] {
                ControlInterface::Position => {
                    if !inp.max_jerk[dof].is_infinite() {
                        if self.position_limits_active {
                            p.brake.get_position_brake_trajectory_with_position_limits(
                                inp.current_position[dof],
                                inp.current_velocity[dof],
                                inp.current_acceleration[dof],
                                inp.max_velocity[dof],
                                min_vel,
                                inp.max_acceleration[dof],
                                min_acc,
                                inp.max_jerk[dof],
                                self.inp_min_position[dof],
                                self.inp_max_position[dof],
                            );
                        } else {
                            p.brake.get_position_brake_trajectory(
                                inp.current_velocity[dof],
                                inp.current_acceleration[dof],
                                inp.max_velocity[dof],
                                min_vel,
                                inp.max_acceleration[dof],
                                min_acc,
                                inp.max_jerk[dof],
                            );
                        }
                    } else if !inp.max_acceleration[dof].is_infinite() {
                        if self.position_limits_active {
                            p.brake
                                .get_second_order_position_brake_trajectory_with_position_limits(
                                    inp.current_position[dof],
                                    inp.current_velocity[dof],
                                    inp.max_velocity[dof],
                                    min_vel,
                                    inp.max_acceleration[dof],
                                    min_acc,
                                    self.inp_min_position[dof],
                                    self.inp_max_position[dof],
                                );
                        } else {
                            p.brake.get_second_order_position_brake_trajectory(
                                inp.current_velocity[dof],
                                inp.max_velocity[dof],
                                min_vel,
                                inp.max_acceleration[dof],
                                min_acc,
                            );
                        }
                    }
                    p.set_boundary(
                        inp.current_position[dof],
                        inp.current_velocity[dof],
                        inp.current_acceleration[dof],
                        inp.target_position[dof],
                        inp.target_velocity[dof],
                        inp.target_acceleration[dof],
                    );
                }
                ControlInterface::Velocity => {
                    if !inp.max_jerk[dof].is_infinite() {
                        p.brake.get_velocity_brake_trajectory(
                            inp.current_acceleration[dof],
                            inp.max_acceleration[dof],
                            min_acc,
                            inp.max_jerk[dof],
                        );
                    } else {
                        p.brake.get_second_order_velocity_brake_trajectory();
                    }

                    // With position limits, cap the effective target velocity so
                    // that a full stop always fits inside the remaining distance
                    // to the limits. The user's input is not modified.
                    let mut target_velocity = inp.target_velocity[dof];
                    if self.position_limits_active {
                        let p0 = inp.current_position[dof];
                        let (cap_up, cap_down) = if !inp.max_jerk[dof].is_infinite() {
                            (
                                velocity_cap_third_order(
                                    self.inp_max_position[dof] - p0,
                                    min_acc,
                                    inp.max_jerk[dof],
                                ),
                                velocity_cap_third_order(
                                    p0 - self.inp_min_position[dof],
                                    inp.max_acceleration[dof],
                                    inp.max_jerk[dof],
                                ),
                            )
                        } else if !inp.max_acceleration[dof].is_infinite() {
                            (
                                velocity_cap_second_order(
                                    self.inp_max_position[dof] - p0,
                                    min_acc,
                                ),
                                velocity_cap_second_order(
                                    p0 - self.inp_min_position[dof],
                                    inp.max_acceleration[dof],
                                ),
                            )
                        } else {
                            (f64::INFINITY, f64::INFINITY)
                        };
                        target_velocity = target_velocity.min(cap_up).max(-cap_down);
                    }

                    p.set_boundary_for_velocity(
                        inp.current_position[dof],
                        inp.current_velocity[dof],
                        inp.current_acceleration[dof],
                        target_velocity,
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
                    found_profile = self.run_position_step1(dof, inp, p);
                    if found_profile
                        && self.position_limits_active
                        && !self.enforce_position_limits_step1(dof, inp, p)
                    {
                        T::handle_calculator_error(&format!(
                            "no profile within position limits found in step 1, dof: {} input: {}",
                            dof, inp
                        ))?;
                        return Ok(RuckigResult::ErrorPositionalLimits);
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
                            min_acc,
                            inp.max_jerk[dof],
                        );
                        found_profile = step1.get_profile(p, &mut self.blocks[dof]);
                    } else {
                        let mut step1 = VelocitySecondOrderStep1::new(
                            p.v[0],
                            p.vf,
                            inp.max_acceleration[dof],
                            min_acc,
                        );
                        found_profile = step1.get_profile(p, &mut self.blocks[dof]);
                    }
                }
                ControlInterface::Acceleration => {}
            }

            if !found_profile {
                let has_zero_limits = inp.max_acceleration[dof] == 0.0
                    || min_acc == 0.0
                    || inp.max_jerk[dof] == 0.0;
                if has_zero_limits {
                    T::handle_calculator_error(
                        &format!(
                            "zero limits conflict in step 1, dof: {} input: {}",
                            dof, inp
                        )
                    )?;
                    return Ok(RuckigResult::ErrorZeroLimits);
                }
                T::handle_calculator_error(
                    &format!("error in step 1, dof: {} input: {}", dof, inp)
                )?;
                return Ok(RuckigResult::ErrorExecutionTimeCalculation);
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
                T::handle_calculator_error(
                    &format!("zero limits conflict with other degrees of freedom in time synchronization {}", traj.duration)
                )?;
                return Ok(RuckigResult::ErrorZeroLimits);
            }
            T::handle_calculator_error(
                &format!("error in time synchronization: {}", traj.duration)
            )?;
            return Ok(RuckigResult::ErrorSynchronizationCalculation);
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
                                            self.eff_max_velocity[dof],
                                            self.eff_min_velocity[dof],
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
                                                self.eff_max_velocity[dof],
                                                self.eff_min_velocity[dof],
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
                                                self.eff_max_velocity[dof],
                                                self.eff_min_velocity[dof],
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
                                            self.eff_max_velocity[dof],
                                            self.eff_min_velocity[dof],
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
                                                self.eff_max_velocity[dof],
                                                self.eff_min_velocity[dof],
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
                            self.eff_max_velocity[dof],
                            self.eff_min_velocity[dof],
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
                            self.eff_max_velocity[dof],
                            self.eff_min_velocity[dof],
                            inp.max_acceleration[dof],
                            self.inp_min_acceleration[dof],
                        );
                        found_time_synchronization = step2.get_profile(p);
                    } else {
                        let mut step2 = PositionFirstOrderStep2::new(
                            t_profile,
                            p.p[0],
                            p.pf,
                            self.eff_max_velocity[dof],
                            self.eff_min_velocity[dof],
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
                T::handle_calculator_error(
                    &format!(
                        "error in step 2, dof: {} input: {}", dof, inp
                    )
                )?;
                return Ok(RuckigResult::ErrorExecutionTimeCalculation);
            }

            // Uncomment the following line if you want to debug
            // println!("{} profile step2: {}", dof, p.to_string());
        }

        Ok(RuckigResult::Working)
    }
}
