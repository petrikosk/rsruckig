//! Result types for trajectory calculation operations
//!
//! This module provides result types that indicate the status of trajectory
//! calculation operations, distinguishing between success and different error states.

/// Result type of Ruckig's trajectory calculation and update functions
///
/// This enum represents the current state of trajectory generation:
/// - Success states (`Working`, `Finished`) indicate normal operation
/// - Error states (negative values) indicate various failure conditions
///
/// The result can be compared to integer values for backward compatibility with C++ code.
///
/// # Example
///
/// ```no_run
/// use rsruckig::prelude::*;
///
/// let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
/// let mut input = InputParameter::new(None);
/// let mut output = OutputParameter::new(None);
///
/// // Configure input...
///
/// while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
///     // Trajectory still in progress
///     output.pass_to_input(&mut input);
/// }
///
/// // Trajectory has reached the target
/// assert_eq!(output.time >= output.trajectory.get_duration(), true);
/// ```
#[derive(Debug, PartialEq)]
pub enum RuckigResult {
    /// The trajectory is being calculated normally
    Working = 0,
    
    /// The trajectory has reached its final position
    Finished = 1,
    
    /// Unclassified error in calculation
    Error = -1,
    
    /// Error in the input parameters
    ErrorInvalidInput = -100,
    
    /// The trajectory duration exceeds its numerical limits
    ErrorTrajectoryDuration = -101,
    
    /// The trajectory exceeds the given positional limits (only in Ruckig Pro)
    ErrorPositionalLimits = -102,
    
    /// The trajectory is not valid due to a conflict with zero limits
    ErrorZeroLimits = -104,
    
    /// Error during the extreme time calculation (Step 1)
    ErrorExecutionTimeCalculation = -110,
    
    /// Error during the synchronization calculation (Step 2)
    ErrorSynchronizationCalculation = -111,
}
