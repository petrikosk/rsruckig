#![allow(clippy::too_many_arguments)]

pub mod block;
pub mod brake;
pub mod calculator_target;
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
pub mod trajectory;
pub mod util;
pub mod velocity_second_step1;
pub mod velocity_second_step2;
pub mod velocity_third_step1;
pub mod velocity_third_step2;
pub mod prelude {
    pub use super::daov_heap;
    pub use super::daov_stack;
    pub use super::error::RuckigError;
    pub use super::error::{IgnoreErrorHandler, ThrowErrorHandler};
    pub use super::input_parameter::{
        ControlInterface, DurationDiscretization, InputParameter, Synchronization,
    };
    pub use super::output_parameter::OutputParameter;
    pub use super::profile::Profile;
    pub use super::result::RuckigResult;
    pub use super::ruckig::Ruckig;
    pub use super::trajectory::Trajectory;
    pub use super::util::DataArrayOrVec;
}
