use crate::result::RuckigResult;
use std::{
    error::Error,
    fmt::{self},
};

pub struct RuckigError {
    message: String,
}

impl Error for RuckigError {}

impl RuckigError {
    pub fn new(message: String) -> RuckigError {
        RuckigError {
            message: format!("\n[rsruckig] {}\n", message),
        }
    }
}

impl fmt::Display for RuckigError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "{}", self.message)
    }
}

impl fmt::Debug for RuckigError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "RuckigError: {}", self.message)
    }
}

/// Trait for handling validation and calculator errors.
/// Types that implement this trait decide how to respond to validation and calculator errors.
pub trait RuckigErrorHandler {
    fn handle_validation_error(message: &str) -> Result<bool, RuckigError>;
    fn handle_calculator_error(
        message: &str,
        result: RuckigResult,
    ) -> Result<RuckigResult, RuckigError>;
}

#[derive(Debug, Default)]
pub struct ThrowErrorHandler;

impl RuckigErrorHandler for ThrowErrorHandler {
    fn handle_validation_error(message: &str) -> Result<bool, RuckigError> {
        Err(RuckigError::new(message.to_string()))
    }
    fn handle_calculator_error(
        message: &str,
        result: RuckigResult,
    ) -> Result<RuckigResult, RuckigError> {
        Err(RuckigError::new(format!("{}: {:?}", message, result)))
    }
}

#[derive(Debug, Default)]
pub struct IgnoreErrorHandler;

impl RuckigErrorHandler for IgnoreErrorHandler {
    fn handle_validation_error(_message: &str) -> Result<bool, RuckigError> {
        Ok(false)
    }
    fn handle_calculator_error(
        _message: &str,
        result: RuckigResult,
    ) -> Result<RuckigResult, RuckigError> {
        Ok(result)
    }
}
