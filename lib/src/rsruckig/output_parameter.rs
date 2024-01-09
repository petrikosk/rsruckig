use std::fmt;
use std::ops::Deref;

use crate::input_parameter::InputParameter;
use crate::trajectory::Trajectory;
use crate::util::join;

#[derive(Clone, Default)]
pub struct OutputParameter {
    pub degrees_of_freedom: usize,
    pub trajectory: Trajectory,
    pub new_position: Vec<f64>,
    pub new_velocity: Vec<f64>,
    pub new_acceleration: Vec<f64>,
    pub new_jerk: Vec<f64>,
    pub time: f64,
    pub new_section: usize,
    pub did_section_change: bool,
    pub new_calculation: bool,
    pub was_calculation_interrupted: bool,
    pub calculation_duration: f64,
}

impl OutputParameter {
    pub fn new(dofs: usize) -> Self {
        Self {
            degrees_of_freedom: dofs,
            trajectory: Trajectory::new(dofs),
            new_position: vec![0.0; dofs],
            new_velocity: vec![0.0; dofs],
            new_acceleration: vec![0.0; dofs],
            new_jerk: vec![0.0; dofs],
            time: 0.0,
            new_section: 0,
            did_section_change: false,
            new_calculation: false,
            was_calculation_interrupted: false,
            calculation_duration: 0.0,
        }
    }
    pub fn pass_to_input(&self, input: &mut InputParameter) {
        input.current_position = self.new_position.clone();
        input.current_velocity = self.new_velocity.clone();
        input.current_acceleration = self.new_acceleration.clone();
    }
}

impl fmt::Display for OutputParameter {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(
            f,
            "\nout.new_position = [{}]",
            join(self.new_position.deref(), true)
        )?;
        writeln!(
            f,
            "out.new_velocity = [{}]",
            join(self.new_velocity.deref(), true)
        )?;
        writeln!(
            f,
            "out.new_acceleration = [{}]",
            join(self.new_acceleration.deref(), true)
        )?;
        writeln!(f, "out.new_jerk = [{}]", join(self.new_jerk.deref(), true))?;
        writeln!(f, "out.time = [{}]", self.time)?;
        writeln!(
            f,
            "out.calculation_duration = [{}]",
            self.calculation_duration
        )?;

        Ok(())
    }
}
