/*!
# RSRuckig

Ruckig generates trajectories on-the-fly, allowing robots and machines to react instantaneously to sensor input.

This Rust library is a direct port of the [C++ Ruckig library](https://github.com/pantor/ruckig), with adaptations for Rust's
type system and memory management features.

## Overview

Ruckig calculates a trajectory to a *target* waypoint (with position, velocity, and acceleration) starting from *any* initial
state limited by velocity, acceleration, and jerk constraints. Besides the target state, Ruckig allows defining
intermediate positions for waypoint following. For state-to-state motions, Ruckig guarantees a time-optimal solution.
With intermediate waypoints, Ruckig calculates the path and its time parametrization jointly, resulting in significantly
faster trajectories compared to traditional methods.

## Core Components

The library provides three main components:
- `Ruckig`: Main class for trajectory generation
- `InputParameter`: Class defining the kinematic state and constraints
- `OutputParameter`: Class holding the calculated trajectory results

## Getting Started

```rust
use rsruckig::prelude::*;

fn main() {
    // Create a Ruckig instance with 1 DoF and the ThrowErrorHandler
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01); // DoF, control cycle in seconds

    // Define input parameters
    let mut input = InputParameter::new(None);
    input.current_position[0] = 0.0;
    input.current_velocity[0] = 0.0;
    input.current_acceleration[0] = 0.0;

    input.target_position[0] = 1.0;
    input.target_velocity[0] = 0.0;
    input.target_acceleration[0] = 0.0;

    input.max_velocity[0] = 0.5;
    input.max_acceleration[0] = 1.0;
    input.max_jerk[0] = 2.0;

    // Create output parameters
    let mut output = OutputParameter::new(None);

    // Generate trajectory step by step
    while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
        // Use the new state (output.new_position, output.new_velocity, etc.)

        // Pass the new state to the input for the next iteration
        output.pass_to_input(&mut input);
    }
}
```

## Features

- **Real-time Trajectory Generation**: Generate trajectories on-the-fly for robots and machines
- **Jerk-limited Motion**: Ensures smooth motion by limiting jerk
- **Waypoint-based Trajectory Generation**: Supports intermediate waypoints for complex paths
- **Customizable Error Handling**: Implement your own error handling strategies using the `RuckigErrorHandler` trait
- **Stack or Heap Allocation**: Choose between compile-time (stack) or runtime (heap) allocation for degrees of freedom

## Error Handling

The library provides two error handlers out of the box:
- `ThrowErrorHandler`: Returns errors when validation or calculation fails
- `IgnoreErrorHandler`: Ignores errors and continues execution

You can also implement custom error handlers by implementing the `RuckigErrorHandler` trait.

## Numerical Stability

The library is tested for numerical stability and guarantees correctness within the following bounds:
- Final position and velocity within `1e-8`
- Final acceleration within `1e-10`
- Velocity, acceleration, and jerk limits within `1e-12`

All kinematic limits should be below `1e12` for reliable operation, and the maximum supported trajectory duration is `7e3`.

See the documentation for individual modules and types for more details.
*/

//! # RSRuckig
//!
//! Ruckig generates trajectories on-the-fly, allowing robots and machines to react instantaneously to sensor input.
//!
//! This Rust library is a direct port of the [C++ Ruckig library](https://github.com/pantor/ruckig), with adaptations for Rust's
//! type system and memory management features.

pub mod rsruckig;

/// Re-exports of the most commonly used types
/// 
/// This module provides easy access to the most commonly used types in the RSRuckig library.
pub mod prelude {
    pub use crate::rsruckig::prelude::*;
}

// Re-export macros from rsruckig module
pub use crate::rsruckig::daov_heap;
pub use crate::rsruckig::daov_stack;
pub use crate::rsruckig::count_exprs;