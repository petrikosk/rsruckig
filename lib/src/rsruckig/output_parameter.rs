use std::fmt;
use std::ops::Deref;

use crate::input_parameter::InputParameter;
use crate::trajectory::Trajectory;
use crate::util::{join, DataArrayOrVec};

#[derive(Debug, Clone)]
pub struct OutputParameter<const DOF: usize> {
    pub degrees_of_freedom: usize,
    pub trajectory: Trajectory<DOF>,
    pub new_position: DataArrayOrVec<f64, DOF>,
    pub new_velocity: DataArrayOrVec<f64, DOF>,
    pub new_acceleration: DataArrayOrVec<f64, DOF>,
    pub new_jerk: DataArrayOrVec<f64, DOF>,
    pub time: f64,
    pub new_section: usize,
    pub did_section_change: bool,
    pub new_calculation: bool,
    pub was_calculation_interrupted: bool,
    pub calculation_duration: f64,
}

impl<const DOF: usize> Default for OutputParameter<DOF> {
    fn default() -> Self {
        Self::new(None)
    }
}

impl<const DOF: usize> OutputParameter<DOF> {
    pub fn new(dofs: Option<usize>) -> Self {
        Self {
            degrees_of_freedom: dofs.unwrap_or(DOF),
            trajectory: Trajectory::new(dofs),
            new_position: DataArrayOrVec::new(dofs, 0.0),
            new_velocity: DataArrayOrVec::new(dofs, 0.0),
            new_acceleration: DataArrayOrVec::new(dofs, 0.0),
            new_jerk: DataArrayOrVec::new(dofs, 0.0),
            time: 0.0,
            new_section: 0,
            did_section_change: false,
            new_calculation: false,
            was_calculation_interrupted: false,
            calculation_duration: 0.0,
        }
    }
    pub fn pass_to_input(&self, input: &mut InputParameter<DOF>) {
        input.current_position = self.new_position.clone();
        input.current_velocity = self.new_velocity.clone();
        input.current_acceleration = self.new_acceleration.clone();
    }
}

impl<const DOF: usize> fmt::Display for OutputParameter<DOF> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(
            f,
            "\nout.new_position = [{}]",
            join::<DOF>(self.new_position.deref(), true)
        )?;
        writeln!(
            f,
            "out.new_velocity = [{}]",
            join::<DOF>(self.new_velocity.deref(), true)
        )?;
        writeln!(
            f,
            "out.new_acceleration = [{}]",
            join::<DOF>(self.new_acceleration.deref(), true)
        )?;
        writeln!(
            f,
            "out.new_jerk = [{}]",
            join::<DOF>(self.new_jerk.deref(), true)
        )?;
        writeln!(f, "out.time = [{}]", self.time)?;
        writeln!(
            f,
            "out.calculation_duration = [{}]",
            self.calculation_duration
        )?;

        Ok(())
    }
}
