//! Error types and handling for the Ruckig algorithm
//!
//! This module provides error types and error handling mechanisms for the Ruckig algorithm.
//! It includes error enums and traits for customizing error behavior.

use thiserror::Error;

/// Errors that can occur during trajectory generation
///
/// This enum represents the different types of errors that can occur during
/// the validation and calculation phases of trajectory generation.
#[derive(Debug, Error)]
pub enum RuckigError {
    /// Error during input validation phase
    ///
    /// This error occurs when the input parameters are invalid, such as:
    /// - Kinematic limits that are out of range
    /// - Current or target states that would inevitably violate constraints
    /// - Incompatible configuration settings
    #[error("Validation error: {0}")]
    ValidationError(String),

    /// Error during trajectory calculation phase
    ///
    /// This error occurs when a problem is encountered during the actual
    /// trajectory calculation, such as:
    /// - Numerical instability issues
    /// - Failed to find a valid solution
    /// - Internal algorithm errors
    #[error("Calculator error: {0}")]
    CalculatorError(String),
}

/// Trait for customizing error handling behavior
///
/// This trait allows for customizing how errors are handled during trajectory
/// generation. By implementing this trait, you can decide whether errors should
/// be returned as Results, logged, silently ignored, or handled in any other way.
///
/// The library provides two implementations:
/// - `ThrowErrorHandler`: Returns errors as `Result::Err`
/// - `IgnoreErrorHandler`: Ignores errors and returns `Result::Ok`
///
/// # Example
///
/// Custom error handler that logs errors:
///
/// ```ignore
/// // Note: This example requires the `log` crate
/// use log::error;
/// use rsruckig::prelude::*;
///
/// #[derive(Default, Debug)]
/// pub struct LogErrorHandler;
///
/// impl RuckigErrorHandler for LogErrorHandler {
///     fn handle_validation_error(message: &str) -> Result<(), RuckigError> {
///         error!("Validation error: {}", message);
///         // Return Ok to continue execution despite the error
///         Ok(())
///     }
///     
///     fn handle_calculator_error(message: &str) -> Result<(), RuckigError> {
///         error!("Calculator error: {}", message);
///         // Return Ok to continue execution despite the error
///         Ok(())
///     }
/// }
/// ```
pub trait RuckigErrorHandler {
    /// Handle an error during input validation
    ///
    /// # Arguments
    ///
    /// * `message` - The error message describing the validation issue
    ///
    /// # Returns
    ///
    /// * `Ok(())` - To ignore the error and continue
    /// * `Err(RuckigError)` - To propagate the error
    fn handle_validation_error(message: &str) -> Result<(), RuckigError>;

    /// Handle an error during trajectory calculation
    ///
    /// # Arguments
    ///
    /// * `message` - The error message describing the calculation issue
    ///
    /// # Returns
    ///
    /// * `Ok(())` - To ignore the error and continue
    /// * `Err(RuckigError)` - To propagate the error
    fn handle_calculator_error(message: &str) -> Result<(), RuckigError>;
}

#[derive(Debug, Default)]
pub struct ThrowErrorHandler;

impl RuckigErrorHandler for ThrowErrorHandler {
    fn handle_validation_error(message: &str) -> Result<(), RuckigError> {
        Err(RuckigError::ValidationError(message.to_string()))
    }

    fn handle_calculator_error(message: &str) -> Result<(), RuckigError> {
        Err(RuckigError::CalculatorError(message.to_string()))
    }
}

#[derive(Debug, Default)]
pub struct IgnoreErrorHandler;

impl RuckigErrorHandler for IgnoreErrorHandler {
    fn handle_validation_error(_message: &str) -> Result<(), RuckigError> {
        Ok(())
    }

    fn handle_calculator_error(_message: &str) -> Result<(), RuckigError> {
        Ok(())
    }
}
