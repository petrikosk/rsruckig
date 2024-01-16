use std::fmt;

use crate::input_parameter::InputParameter;
use crate::trajectory::Trajectory;
use crate::util::join;

#[derive(Clone)]
pub struct OutputParameter<const DOF: usize> {
    pub trajectory: Trajectory<DOF>,
    pub new_position: [f64; DOF],
    pub new_velocity: [f64; DOF],
    pub new_acceleration: [f64; DOF],
    pub new_jerk: [f64; DOF],
    pub time: f64,
    pub new_section: usize,
    pub did_section_change: bool,
    pub new_calculation: bool,
    pub was_calculation_interrupted: bool,
    pub calculation_duration: f64,
}

impl<const DOF: usize> Default for OutputParameter<DOF> {
    fn default() -> Self {
        Self {
            trajectory: Default::default(),
            new_position: std::array::from_fn(|_| Default::default()),
            new_velocity: std::array::from_fn(|_| Default::default()),
            new_acceleration: std::array::from_fn(|_| Default::default()),
            new_jerk: std::array::from_fn(|_| Default::default()),
            time: Default::default(),
            new_section: Default::default(),
            did_section_change: Default::default(),
            new_calculation: Default::default(),
            was_calculation_interrupted: Default::default(),
            calculation_duration: Default::default(),
        }
    }
}

impl<const DOF: usize> OutputParameter<DOF> {
    pub fn new() -> Self {
        Self {
            trajectory: Trajectory::new(),
            new_position: [0.0; DOF],
            new_velocity: [0.0; DOF],
            new_acceleration: [0.0; DOF],
            new_jerk: [0.0; DOF],
            time: 0.0,
            new_section: 0,
            did_section_change: false,
            new_calculation: false,
            was_calculation_interrupted: false,
            calculation_duration: 0.0,
        }
    }
    pub fn pass_to_input(&self, input: &mut InputParameter<DOF>) {
        input.current_position = self.new_position;
        input.current_velocity = self.new_velocity;
        input.current_acceleration = self.new_acceleration;
    }
}

impl<const DOF: usize> fmt::Display for OutputParameter<DOF> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(
            f,
            "\nout.new_position = [{}]",
            join(&self.new_position, true)
        )?;
        writeln!(f, "out.new_velocity = [{}]", join(&self.new_velocity, true))?;
        writeln!(
            f,
            "out.new_acceleration = [{}]",
            join(&self.new_acceleration, true)
        )?;
        writeln!(f, "out.new_jerk = [{}]", join(&self.new_jerk, true))?;
        writeln!(f, "out.time = [{}]", self.time)?;
        writeln!(
            f,
            "out.calculation_duration = [{}]",
            self.calculation_duration
        )?;

        Ok(())
    }
}
