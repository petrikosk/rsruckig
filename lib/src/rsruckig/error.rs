use std::fmt;

pub struct RuckigError {
    message: String,
}

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

impl std::fmt::Debug for RuckigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "RuckigError: {}", self.message)
    }
}

/// Trait for handling validation errors.
/// Types that implement this trait decide how to respond to validation errors.
pub trait ValidationErrorHandler {
    fn handle_error(message: &str) -> Result<bool, RuckigError>;
}

pub struct ThrowErrorHandler;

impl ValidationErrorHandler for ThrowErrorHandler {
    fn handle_error(message: &str) -> Result<bool, RuckigError> {
        Err(RuckigError::new(message.to_string()))
    }
}

pub struct IgnoreErrorHandler;

impl ValidationErrorHandler for IgnoreErrorHandler {
    fn handle_error(_message: &str) -> Result<bool, RuckigError> {
        Ok(false)
    }
}
