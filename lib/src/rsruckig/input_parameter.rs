use crate::error::{RuckigError, RuckigErrorHandler};
use crate::util::join;
use std::fmt;

#[derive(Default, Clone, PartialEq, Copy)]
pub enum ControlInterface {
    #[default]
    Position,
    Velocity,
    Acceleration,
}

#[derive(Default, Clone, PartialEq, Copy)]
pub enum Synchronization {
    #[default]
    Time,
    TimeIfNecessary,
    Phase,
    None,
}

#[derive(Default, Clone, PartialEq)]
pub enum DurationDiscretization {
    #[default]
    Continuous,
    Discrete,
}

#[derive(Clone)]
pub struct InputParameter<const DOF: usize> {
    pub control_interface: ControlInterface,
    pub synchronization: Synchronization,
    pub duration_discretization: DurationDiscretization,
    pub current_position: [f64; DOF],
    pub current_velocity: [f64; DOF],
    pub current_acceleration: [f64; DOF],
    pub target_position: [f64; DOF],
    pub target_velocity: [f64; DOF],
    pub target_acceleration: [f64; DOF],
    pub max_velocity: [f64; DOF],
    pub max_acceleration: [f64; DOF],
    pub max_jerk: [f64; DOF],
    pub min_velocity: Option<[f64; DOF]>,
    pub min_acceleration: Option<[f64; DOF]>,
    pub enabled: [bool; DOF],
    pub per_dof_control_interface: Option<[ControlInterface; DOF]>,
    pub per_dof_synchronization: Option<[Synchronization; DOF]>,
    pub minimum_duration: Option<f64>,
    pub per_section_minimum_duration: Option<[f64; DOF]>,
    pub interrupt_calculation_duration: Option<f64>,
}

impl<const DOF: usize> Default for InputParameter<DOF> {
    fn default() -> Self {
        Self {
            control_interface: Default::default(),
            synchronization: Default::default(),
            duration_discretization: Default::default(),
            current_position: [0.; DOF],
            current_velocity: [0.; DOF],
            current_acceleration: [0.; DOF],
            target_position: [0.; DOF],
            target_velocity: [0.; DOF],
            target_acceleration: [0.; DOF],
            max_velocity: [0.; DOF],
            max_acceleration: [0.; DOF],
            max_jerk: [0.; DOF],
            min_velocity: Default::default(),
            min_acceleration: Default::default(),
            enabled: [true; DOF],
            per_dof_control_interface: Default::default(),
            per_dof_synchronization: Default::default(),
            minimum_duration: Default::default(),
            per_section_minimum_duration: Default::default(),
            interrupt_calculation_duration: Default::default(),
        }
    }
}

impl<const DOF: usize> PartialEq for InputParameter<DOF> {
    fn eq(&self, other: &Self) -> bool {
        self.current_position == other.current_position
            && self.current_velocity == other.current_velocity
            && self.current_acceleration == other.current_acceleration
            && self.target_position == other.target_position
            && self.target_velocity == other.target_velocity
            && self.target_acceleration == other.target_acceleration
            && self.max_velocity == other.max_velocity
            && self.max_acceleration == other.max_acceleration
            && self.max_jerk == other.max_jerk
            && self.enabled == other.enabled
            && self.minimum_duration == other.minimum_duration
            && self.per_section_minimum_duration == other.per_section_minimum_duration
            && self.min_velocity == other.min_velocity
            && self.min_acceleration == other.min_acceleration
            && self.control_interface == other.control_interface
            && self.synchronization == other.synchronization
            && self.duration_discretization == other.duration_discretization
            && self.per_dof_control_interface == other.per_dof_control_interface
            && self.per_dof_synchronization == other.per_dof_synchronization
    }
}

impl<const DOF: usize> InputParameter<DOF> {
    pub fn new() -> Self {
        Self {
            control_interface: ControlInterface::Position,
            synchronization: Synchronization::Time,
            duration_discretization: DurationDiscretization::Continuous,
            current_position: [0.0; DOF],
            current_velocity: [0.0; DOF],
            current_acceleration: [0.0; DOF],
            target_position: [0.0; DOF],
            target_velocity: [0.0; DOF],
            target_acceleration: [0.0; DOF],
            max_velocity: [0.0; DOF],
            max_acceleration: [f64::INFINITY; DOF],
            max_jerk: [f64::INFINITY; DOF],
            min_velocity: None,
            min_acceleration: None,
            enabled: [true; DOF],
            per_dof_control_interface: None,
            per_dof_synchronization: None,
            minimum_duration: None,
            per_section_minimum_duration: None,
            interrupt_calculation_duration: None,
        }
    }

    #[inline]
    pub fn v_at_a_zero(v0: f64, a0: f64, j: f64) -> f64 {
        v0 + (a0 * a0) / (2.0 * j)
    }

    /// Validate the input for trajectory calculation
    pub fn validate<E: RuckigErrorHandler>(
        &self,
        check_current_state_within_limits: bool,
        check_target_state_within_limits: bool,
    ) -> Result<bool, RuckigError> {
        for dof in 0..DOF {
            let j_max = self.max_jerk[dof];
            if j_max.is_nan() || j_max < 0.0 {
                return E::handle_validation_error(&format!(
                    "Maximum jerk limit {} of DoF {} should be larger than or equal to zero.",
                    j_max, dof
                ));
            }

            let a_max: f64 = self.max_acceleration[dof];
            if a_max.is_nan() || a_max < 0.0 {
                return E::handle_validation_error(&format!("maximum acceleration limit {} of DoF {} should be larger than or equal to zero.", a_max, dof));
            }

            let a_min: f64 = match &self.min_acceleration {
                Some(min_acc) => min_acc[dof],
                None => -self.max_acceleration[dof],
            };
            if a_min.is_nan() || a_min > 0.0 {
                return E::handle_validation_error(&format!("minimum acceleration limit {} of DoF {} should be smaller than or equal to zero.", a_min, dof));
            }

            let a0: f64 = self.current_acceleration[dof];
            if a0.is_nan() {
                return E::handle_validation_error(&format!(
                    "current acceleration {} of DoF {} should be a valid number.",
                    a0, dof
                ));
            }

            let af: f64 = self.target_acceleration[dof];
            if af.is_nan() {
                return E::handle_validation_error(&format!(
                    "target acceleration {} of DoF {} should be a valid number.",
                    af, dof
                ));
            }

            if check_current_state_within_limits {
                if a0 > a_max {
                    return E::handle_validation_error(&format!("current acceleration {} of DoF {} exceeds its maximum acceleration limit {}.", a0, dof, a_max));
                }
                if a0 < a_min {
                    return E::handle_validation_error(&format!("current acceleration {} of DoF {} undercuts its minimum acceleration limit {}.", a0, dof, a_min));
                }
            }
            if check_target_state_within_limits {
                if af > a_max {
                    return E::handle_validation_error(&format!("target acceleration {} of DoF {} exceeds its maximum acceleration limit {}.", af, dof, a_max));
                }
                if af < a_min {
                    return E::handle_validation_error(&format!("target acceleration {} of DoF {} undercuts its minimum acceleration limit {}.", af, dof, a_min));
                }
            }

            let v0 = self.current_velocity[dof];
            if v0.is_nan() {
                return E::handle_validation_error(&format!(
                    "current velocity {} of DoF {} should be a valid number.",
                    v0, dof
                ));
            }
            let vf = self.target_velocity[dof];
            if vf.is_nan() {
                return E::handle_validation_error(&format!(
                    "target velocity {} of DoF {} should be a valid number.",
                    vf, dof
                ));
            }

            let control_interface_ = match &self.per_dof_control_interface {
                Some(per_dof) => match per_dof.get(dof) {
                    Some(interface) => interface, // assuming get() returns a reference; de-reference it
                    None => &self.control_interface,
                },
                None => &self.control_interface,
            };

            if let ControlInterface::Position = control_interface_ {
                let p0 = self.current_position[dof];
                if p0.is_nan() {
                    return E::handle_validation_error(&format!(
                        "current position {} of DoF {} should be a valid number.",
                        p0, dof
                    ));
                }
                let pf = self.target_position[dof];
                if pf.is_nan() {
                    return E::handle_validation_error(&format!(
                        "target position {} of DoF {} should be a valid number.",
                        pf, dof
                    ));
                }

                let v_max = self.max_velocity[dof];
                if v_max.is_nan() || v_max < 0.0 {
                    return E::handle_validation_error(&format!("maximum velocity limit {} of DoF {} should be larger than or equal to zero.", v_max, dof));
                }

                let v_min = if let Some(min_velocity) = &self.min_velocity {
                    min_velocity[dof]
                } else {
                    -v_max
                };
                if v_min.is_nan() || v_min > 0.0 {
                    return E::handle_validation_error(&format!("minimum velocity limit {} of DoF {} should be smaller than or equal to zero.", v_min, dof));
                }

                if check_current_state_within_limits {
                    if v0 > v_max {
                        return E::handle_validation_error(&format!(
                            "current velocity {} of DoF {} exceeds its maximum velocity limit {}.",
                            v0, dof, v_max
                        ));
                    }
                    if v0 < v_min {
                        return E::handle_validation_error(&format!("current velocity {} of DoF {} undercuts its minimum velocity limit {}.", v0, dof, v_min));
                    }
                }
                if check_target_state_within_limits {
                    if vf > v_max {
                        return E::handle_validation_error(&format!(
                            "target velocity {} of DoF {} exceeds its maximum velocity limit {}.",
                            vf, dof, v_max
                        ));
                    }
                    if vf < v_min {
                        return E::handle_validation_error(&format!(
                            "target velocity {} of DoF {} undercuts its minimum velocity limit {}.",
                            vf, dof, v_min
                        ));
                    }
                }
                if check_current_state_within_limits {
                    if a0 > 0.0
                        && j_max > 0.0
                        && InputParameter::<DOF>::v_at_a_zero(v0, a0, j_max) > v_max
                    {
                        return E::handle_validation_error(&format!("DoF {} will inevitably reach a velocity {} from the current kinematic state that will exceed its maximum velocity limit {}.", dof, InputParameter::<DOF>::v_at_a_zero(v0, a0, j_max), v_max));
                    }
                    if a0 < 0.0
                        && j_max > 0.0
                        && InputParameter::<DOF>::v_at_a_zero(v0, a0, -j_max) < v_min
                    {
                        return E::handle_validation_error(&format!("DoF {} will inevitably reach a velocity {} from the current kinematic state that will undercut its minimum velocity limit {}.", dof, InputParameter::<DOF>::v_at_a_zero(v0, a0, -j_max), v_min));
                    }
                }
                if check_target_state_within_limits {
                    if af < 0.0
                        && j_max > 0.0
                        && InputParameter::<DOF>::v_at_a_zero(vf, af, j_max) > v_max
                    {
                        return E::handle_validation_error(&format!("DoF {} will inevitably have reached a velocity {} from the target kinematic state that will exceed its maximum velocity limit {}.", dof, InputParameter::<DOF>::v_at_a_zero(vf, af, j_max), v_max));
                    }
                    if af > 0.0
                        && j_max > 0.0
                        && InputParameter::<DOF>::v_at_a_zero(vf, af, -j_max) < v_min
                    {
                        return E::handle_validation_error(&format!("DoF {} will inevitably have reached a velocity {} from the target kinematic state that will undercut its minimum velocity limit {}.", dof, InputParameter::<DOF>::v_at_a_zero(vf, af, -j_max), v_min));
                    }
                }
            }
        }
        Ok(true)
    }
}

impl<const DOF: usize> fmt::Display for InputParameter<DOF> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f)?;

        if self.control_interface == ControlInterface::Velocity {
            writeln!(f, "inp.control_interface = ControlInterface.Velocity")?;
        }
        if self.synchronization == Synchronization::Phase {
            writeln!(f, "inp.synchronization = Synchronization.Phase")?;
        } else if self.synchronization == Synchronization::None {
            writeln!(f, "inp.synchronization = Synchronization.No")?;
        }
        if self.duration_discretization == DurationDiscretization::Discrete {
            writeln!(
                f,
                "inp.duration_discretization = DurationDiscretization.Discrete"
            )?;
        }

        writeln!(
            f,
            "\ninp.current_position = [{}]",
            join(&self.current_position, true)
        )?;
        writeln!(
            f,
            "inp.current_velocity = [{}]",
            join(&self.current_velocity, true)
        )?;
        writeln!(
            f,
            "inp.current_acceleration = [{}]",
            join(&self.current_acceleration, true)
        )?;
        writeln!(
            f,
            "inp.target_position = [{}]",
            join(&self.target_position, true)
        )?;
        writeln!(
            f,
            "inp.target_velocity = [{}]",
            join(&self.target_velocity, true)
        )?;
        writeln!(
            f,
            "inp.target_acceleration = [{}]",
            join(&self.target_acceleration, true)
        )?;
        writeln!(f, "inp.max_velocity = [{}]", join(&self.max_velocity, true))?;
        writeln!(
            f,
            "inp.max_acceleration = [{}]",
            join(&self.max_acceleration, true)
        )?;
        writeln!(f, "inp.max_jerk = [{}]", join(&self.max_jerk, true))?;

        if let Some(min_vel) = &self.min_velocity {
            writeln!(f, "inp.min_velocity = [{}]", join(min_vel, true))?;
        }
        if let Some(min_acc) = &self.min_acceleration {
            writeln!(f, "inp.min_acceleration = [{}]", join(min_acc, true))?;
        }

        Ok(())
    }
}
