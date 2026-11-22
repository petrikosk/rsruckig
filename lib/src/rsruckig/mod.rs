//! The core implementation of the Ruckig trajectory generation algorithm
//!
//! This module contains all the components needed for real-time trajectory generation:
//! - Trajectory generator [`ruckig::Ruckig`]
//! - Input parameters [`input_parameter::InputParameter`]
//! - Output parameters [`output_parameter::OutputParameter`]
//! - Error handling through the [`error::RuckigErrorHandler`] trait
//! - Collection types like [`util::DataArrayOrVec`]
//!
//! ## Quick Start
//!
//! ```
//! use rsruckig::prelude::*;
//!
//! // 1 DoF, stack allocated, 10 ms control cycle
//! let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
//! let mut input = InputParameter::new(None);
//! let mut output = OutputParameter::new(None);
//!
//! input.current_position = daov_stack![0.0];
//! input.target_position = daov_stack![1.0];
//! input.max_velocity = daov_stack![1.0];
//! input.max_acceleration = daov_stack![2.0];
//! input.max_jerk = daov_stack![5.0];
//!
//! // Online: step the trajectory once per control cycle
//! while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
//!     output.pass_to_input(&mut input);
//! }
//! assert!((output.new_position[0] - 1.0).abs() < 1e-9);
//!
//! // Offline: calculate the whole trajectory at once
//! let mut trajectory = Trajectory::new(None);
//! otg.calculate(&input, &mut trajectory).unwrap();
//! ```
//!
//! Tracking a moving target signal is provided by [`trackig::Trackig`].
//!
//! ## Main Components
//!
//! The main components you'll interact with are:
//!
//! - [`ruckig::Ruckig`]: Main class for trajectory calculation and step-by-step updating
//! - [`input_parameter::InputParameter`]: Contains the current state, target state, and limits
//! - [`output_parameter::OutputParameter`]: Contains the resulting trajectory and calculated states
//!
//! ## Memory Management
//!
//! The library supports both stack and heap allocation for degree-of-freedom (DoF) data:
//!
//! - Use template parameter for stack allocation (e.g., `Ruckig::<3, ThrowErrorHandler>`)
//! - Use runtime parameter for heap allocation (e.g., `Ruckig::<0, ThrowErrorHandler>::new(Some(3), ...)`)
//!
//! ## Error Handling
//!
//! Error handling is done through the [`error::RuckigErrorHandler`] trait with two implementations:
//!
//! - [`error::ThrowErrorHandler`]: Returns errors when validation or calculation fails
//! - [`error::IgnoreErrorHandler`]: Ignores errors and continues execution
//!
//! You can implement your own error handler by implementing the [`error::RuckigErrorHandler`] trait.
//!
//! ## Control Interfaces
//!
//! The library supports three control interfaces:
//!
//! - Position control: Control the full kinematic state (default)
//! - Velocity control: Control the velocity (useful for visual servoing)
//! - Acceleration control: Control the acceleration directly

#![allow(clippy::too_many_arguments)]
#![warn(missing_docs)]

#![no_std]

#[cfg(feature = "std")]
extern crate std;

mod alloc {
    #[cfg(feature = "alloc")]
    extern crate alloc;

    #[cfg(not(feature = "std"))]
    pub use alloc::*;
    #[cfg(feature = "std")]
    pub use std::*;
}

pub mod block;
pub mod brake;
pub mod calculator_target;
pub mod calculator_waypoints;
pub mod error;
pub mod input_parameter;
pub mod output_parameter;
pub mod position_first_step1;
pub mod position_first_step2;
pub mod position_second_step1;
pub mod position_second_step2;
pub mod position_third_step1;
pub mod position_third_step2;
pub mod profile;
pub mod result;
pub mod roots;
pub mod ruckig;
pub mod trackig;
pub mod trajectory;
pub mod util;
pub mod velocity_second_step1;
pub mod velocity_second_step2;
pub mod velocity_third_step1;
pub mod velocity_third_step2;

/// Re-exports of the most commonly used types
pub mod prelude {
    pub use super::error::RuckigError;
    pub use super::error::{IgnoreErrorHandler, RuckigErrorHandler, ThrowErrorHandler};
    pub use super::input_parameter::{
        ControlInterface, DurationDiscretization, InputParameter, Synchronization,
    };
    pub use super::output_parameter::OutputParameter;
    pub use super::profile::Profile;
    pub use super::result::RuckigResult;
    pub use super::ruckig::Ruckig;
    pub use super::trackig::{TargetState, Trackig};
    pub use super::trajectory::{KinematicState, Trajectory};
    pub use super::util::DataArrayOrVec;

    // Also re-export the macros for convenience
    pub use crate::daov_stack;
    pub use crate::daov_heap;
    pub use crate::count_exprs;
}
