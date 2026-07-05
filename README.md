<div class="centered">
  <h1 class="centered">Ruckig</h1>
  <h3 class="centered">
    Instantaneous Motion Generation for Robots and Machines.
  </h3>
</div>

This is a Rust port of the repository https://github.com/pantor/ruckig/. Cloud client and most pro-features are not
ported. However, this port implements its own equivalents of two Ruckig Pro features: **position limits**
(`min_position` / `max_position`) and a **tracking interface** (`Trackig`) for following a moving target signal. The
examples use Gnuplot to illustrate the trajectories.

Ruckig generates trajectories on-the-fly, allowing robots and machines to react instantaneously to sensor input. Ruckig
calculates a trajectory to a _target_ state (with position, velocity, and acceleration) starting from _any_ initial
state limited by velocity, acceleration, and jerk constraints. For these state-to-state motions, Ruckig guarantees a
time-optimal solution. On top of that, this port can restrict the workspace with per-DoF _position limits_ and follow a
moving target signal with the _tracking interface_. Intermediate waypoints (a Ruckig Pro / cloud feature) are not
supported.

More information can be found at [ruckig.com](https://ruckig.com) and in the corresponding
paper [Jerk-limited Real-time Trajectory Generation with Arbitrary Target States](https://arxiv.org/abs/2105.04830),
accepted for the _Robotics: Science and Systems (RSS), 2021_ conference.

## Features

- **Real-time Trajectory Generation**: Generate trajectories on-the-fly for robots and machines.
- **Jerk-limited Motion**: Ensures smooth motion by limiting jerk.
- **Position Limits**: Restrict the workspace with `min_position` / `max_position` - the generated trajectory is
  guaranteed to stay inside the limits, or a `ErrorPositionalLimits` result is returned (never a silent violation).
- **Tracking Interface**: Follow a moving target signal with the `Trackig` struct under full kinematic constraints.
- **Customizable Error Handling**: Implement your own error handling strategies using the `RuckigErrorHandler` trait.
- **no-std support**: Run this library on embedded systems without the standard library (still requires `alloc`)

## Installation

To build the project, run:

```bash
cargo build --release
```

## No std usage

This crate still requires `alloc`. Use the following in your Cargo.toml:

```
{ default-features = false, features = ["libm", "alloc"] }
```

### Dependencies

Ensure you have the following dependencies installed:

- Rust (latest stable version)
- Gnuplot (for visualizing trajectories)

On Ubuntu, you can install Gnuplot with:

```bash
sudo apt-get install gnuplot
```

## Documentation

The `rsruckig` library provides comprehensive documentation for all its components. You can generate and view the documentation locally by running:

```bash
cargo doc --open
```

This will open the documentation in your default web browser, where you can explore detailed descriptions of the library's modules, structs, enums, and methods.

## Examples

Here are some examples to get you started:

### Basic Usage

```rust
use rsruckig::prelude::*;

fn main() {
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);

    input.current_position[0] = 0.0;
    input.target_position[0] = 10.0;
    input.max_velocity[0] = 5.0;
    input.max_acceleration[0] = 10.0;
    input.max_jerk[0] = 20.0;

    while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
        println!("Position: {:?}, Velocity: {:?}", output.new_position, output.new_velocity);
        output.pass_to_input(&mut input);
    }
}
```

### Custom Error Handling

```rust
use log::error;
use rsruckig::prelude::*;

#[derive(Default, Debug)]
pub struct LogErrorHandler;

impl RuckigErrorHandler for LogErrorHandler {
    fn handle_validation_error(message: &str) -> Result<(), RuckigError> {
        error!("Validation error: {}", message);
        Ok(())
    }

    fn handle_calculator_error(message: &str) -> Result<(), RuckigError> {
        error!("Calculator error: {}", message);
        Ok(())
    }
}

fn main() {
    let mut otg = Ruckig::<1, LogErrorHandler>::new(None, 0.01);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);

    input.current_position[0] = 0.0;
    input.target_position[0] = 10.0;
    input.max_velocity[0] = 5.0;
    input.max_acceleration[0] = 10.0;
    input.max_jerk[0] = 20.0;

    while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
        println!("Position: {:?}, Velocity: {:?}", output.new_position, output.new_velocity);
        output.pass_to_input(&mut input);
    }
}
```

### Position Limits

```rust
use rsruckig::prelude::*;

fn main() {
    let mut otg = Ruckig::<1, ThrowErrorHandler>::new(None, 0.01);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);

    input.current_velocity[0] = 8.0; // Fast toward the target...
    input.target_position[0] = 8.0;
    input.max_velocity[0] = 10.0;
    input.max_acceleration[0] = 5.0;
    input.max_jerk[0] = 30.0;

    // ...but the trajectory must never leave [-0.5, 10.0]
    input.min_position = Some(daov_stack![-0.5]);
    input.max_position = Some(daov_stack![10.0]);

    while otg.update(&input, &mut output).unwrap() == RuckigResult::Working {
        output.pass_to_input(&mut input);
    }
}
```

### Tracking a Moving Target

```rust
use rsruckig::prelude::*;

fn main() {
    let delta_time = 0.001;
    let mut otg = Trackig::<1, IgnoreErrorHandler>::new(None, delta_time);
    let mut input = InputParameter::new(None);
    let mut output = OutputParameter::new(None);
    input.synchronization = Synchronization::None; // recommended for tracking
    input.max_velocity[0] = 3.0;
    input.max_acceleration[0] = 15.0;
    input.max_jerk[0] = 150.0;

    let mut target = TargetState::new(None);
    for i in 0..5000 {
        let t = i as f64 * delta_time;
        target.position[0] = (2.0 * t).sin();
        target.velocity[0] = 2.0 * (2.0 * t).cos();
        target.acceleration[0] = -4.0 * (2.0 * t).sin();

        let _ = otg.update(&target, &input, &mut output).unwrap();
        output.pass_to_input(&mut input);
        // Use output.new_position / new_velocity / new_acceleration as setpoints
    }
}
```

### Running the Bundled Examples

The `samples/` crate contains runnable examples that plot the trajectories with Gnuplot:

```bash
cargo run --bin example_position_1dof
cargo run --bin example_position_3dof
cargo run --bin example_position_3_dynamic_dof
cargo run --bin example_velocity_1dof
cargo run --bin example_position_1dof_custom_error_handler
cargo run --bin example_position_limits_1dof
cargo run --bin example_tracking_1dof
```

## Tutorial

Let's get started!

![Trajectory Profile](https://github.com/petrikosk/rsruckig/raw/master/doc/example_profile.png?raw=true)

### State-to-state Trajectory Generation

Ruckig provides three main interface classes: the _Ruckig_, the _InputParameter_, and the _OutputParameter_ class.
(For following a moving target signal, see the _Trackig_ class further below.)

First, you'll need to create a Ruckig instance with the `DOF` and `RuckigErrorHandler` as a template parameter,
and the number of DOF as an option and the control cycle (
e.g. in seconds) in the constructor.

You can opt to use template parameter to specify the number of DOF and use stack allocation, or use the constructor to
specify the number of DOF as `Option<usize>`.
If you use dynamic number of DOF, the template DOF parameter must be set to 0 to allocate dofs dynamically.
Any number greater than 0 as a template parameter, will result in omitting the dynamic DOF parameter.
In case of dynamic DOF allocation, a good practice is to set constructor DOF parameter to None.

```rust
use rsruckig::prelude::*;

// stack allocation using template parameter
let mut ruckig = Ruckig::<6, ThrowErrorHandler>::new(None, 0.01); // Number DoFs; control cycle in [s]

// dynamic allocation
let mut ruckig = Ruckig::<0, ThrowErrorHandler>::new(Some(6), 0.01); // Number DoFs; control cycle in [s]
```

Implemented error handlers are:

- `ThrowErrorHandler` - throws an error with a detailed reason if an input is not valid.
- `IgnoreErrorHandler` - ignores the error and returns `Ok(RuckigResult)`.

To implement your own error handler, you need to implement the `RuckigErrorHandler` trait:

```rust
pub trait RuckigErrorHandler {
    fn handle_validation_error(message: &str) -> Result<(), RuckigError>;
    fn handle_calculator_error(message: &str) -> Result<(), RuckigError>;
}
```

The input type has 3 blocks of data: the _current_ state, the _target_ state and the corresponding kinematic _limits_.

```rust
// Stack DOF allocation
let mut input = InputParameter::new(None); // Number DoFs
input.current_position = daov_stack![0.2, ...];
input.current_velocity = daov_stack![0.1, ...];
input.current_acceleration = daov_stack![0.1, ...];
input.target_position = daov_stack![0.5, ...];
input.target_velocity = daov_stack![-0.1, ...];
input.target_acceleration = daov_stack![0.2, ...];
input.max_velocity = daov_stack![0.4, ...];
input.max_acceleration = daov_stack![1.0, ...];
input.max_jerk = daov_stack![4.0, ...];

let mut output: OutputParameter = OutputParameter::new(None);

// Dynamic DOF allocation
let mut input = InputParameter::new(Some(6)); // Number DoFs
input.current_position = daov_heap![0.2, ...];
input.current_velocity = daov_heap![0.1, ...];
input.current_acceleration = daov_heap![0.1, ...];
input.target_position = daov_heap![0.5, ...];
input.target_velocity = daov_heap![-0.1, ...];
input.target_acceleration = daov_heap![0.2, ...];
input.max_velocity = daov_heap![0.4, ...];
input.max_acceleration = daov_heap![1.0, ...];
input.max_jerk = daov_heap![4.0, ...];

let mut output: OutputParameter = OutputParameter::new(Some(6)); // Number DoFs
```

If you only want to have a acceleration-constrained trajectory, you can also omit the `max_jerk` as well as
the `current` and `target_acceleration` value. Given all input and output resources, we can iterate over the trajectory
at each discrete time step. For most applications, this loop must run within a real-time thread and controls the actual
hardware.

```rust
while ruckig.update(&input, &mut output).unwrap() == RuckigResult::Working {
  // Make use of the new state here!
  // e.g. robo.setJointPositions(output.new_position);

  output.pass_to_input(&mut input); // Don't forget this!
}
```

Within the control loop, you need to update the _current state_ of the input parameter according to the calculated
trajectory. Therefore, the `pass_to_input` method copies the new kinematic state of the output to the current kinematic
state of the input parameter. If (in the next step) the current state is not the expected, pre-calculated trajectory,
Ruckig will calculate a new trajectory based on the novel input. When the trajectory has reached the target state,
the `update` function will return `Result::Finished`.

### DataArrayOrVec

The `DataArrayOrVec` type is a wrapper around a fixed-size array or a vector. It is mainly used to store the kinematic state.
User can choose between stack and heap allocation. The stack allocation is faster, but the number of DOF must be known at
compile time. The heap allocation is slower, but the number of DOF can be set at runtime.

```rust
pub enum DataArrayOrVec<T, const N: usize>
where T: std::fmt::Debug {
    Stack([T; N]),
    Heap(Vec<T>),
}
```

`DataArrayOrVec` has two associated macros to streamline the instantiation process with these shorthand forms:

- `daov_stack!` - Creates a stack-allocated DataArrayOrVec with fixed-size array storage
- `daov_heap!` - Creates a heap-allocated DataArrayOrVec with Vec storage

Example of usage:

```rust
// For stack allocation, using template parameter and array under the hood
let mut data = daov_stack![0.2, 0.3, 0.4];
// For heap allocation, using Vec under the hood
let mut dynamic_data = daov_heap![0.2, 0.3, 0.4];

data[0] = 0.5;
data[1] = 0.6;
data[2] = 0.7;

dynamic_data[0] = 0.5;
dynamic_data[1] = 0.6;
dynamic_data[2] = 0.7;
```

### Input Parameter

To go into more detail, the *InputParameter* type has following members:

```rust
current_position: DataArrayOrVec<f64, DOF>;
current_velocity: DataArrayOrVec<f64, DOF>; // Initialized to zero
current_acceleration: DataArrayOrVec<f64, DOF>; // Initialized to zero

target_position: DataArrayOrVec<f64, DOF>;
target_velocity: DataArrayOrVec<f64, DOF>; // Initialized to zero
target_acceleration: DataArrayOrVec<f64, DOF>; // Initialized to zero

max_velocity: DataArrayOrVec<f64, DOF>;
max_acceleration: DataArrayOrVec<f64, DOF>;
max_jerk: DataArrayOrVec<f64, DOF>; // Initialized to infinity

min_velocity: Option<DataArrayOrVec<f64, DOF>>; // If not given, the negative maximum velocity will be used.
min_acceleration: Option<DataArrayOrVec<f64, DOF>>; // If not given, the negative maximum acceleration will be used.

max_position: Option<DataArrayOrVec<f64, DOF>>; // Optional upper position limit per DoF
min_position: Option<DataArrayOrVec<f64, DOF>>; // Optional lower position limit per DoF

enabled: DataArrayOrVec<bool, DOF>; // Initialized to true
minimum_duration: Option<f64>;

control_interface: ControlInterface; // The default position interface controls the full kinematic state.
synchronization: Synchronization; // Synchronization behavior of multiple DoFs
duration_discretization: DurationDiscretization; // Whether the duration should be a discrete multiple of the control cycle (off by default)

per_dof_control_interface: Option<DataArrayOrVec<ControlInterface, DOF>>; // Sets the control interface for each DoF individually
per_dof_synchronization: Option<DataArrayOrVec<Synchronization, DOF>>; // Sets the synchronization for each DoF individually
```

On top of the current state, target state, and constraints, Ruckig allows for a few more advanced settings:

- A _minimum_ velocity and acceleration can be specified - these should be a negative number. If they are not given, the
  negative maximum velocity or acceleration will be used (similar to the jerk limit). For example, this might be useful
  in human robot collaboration settings with a different velocity limit towards a human. Or, when switching between
  different moving coordinate frames like picking from a conveyer belt.
- If a DoF is not _enabled_, it will be ignored in the calculation. Ruckig will output a trajectory with constant
  acceleration for those DoFs.
- A _minimum duration_ can be optionally given. Note that Ruckig can not guarantee an exact, but only a minimum duration
  of the trajectory.
- The control interface (position or velocity control) can be switched easily. For example, a stop trajectory or visual
  servoing can be easily implemented with the velocity interface.
- Different synchronization behaviors (i.a. phase, time, or no synchonization) are implemented. Phase synchronization
  results in straight-line motions.
- The trajectory duration might be constrained to a multiple of the control cycle. This way, the _exact_ state can be
  reached at a control loop execution.
- _Position limits_ (`min_position` / `max_position`) restrict the workspace of the system. See the section below for
  details.

### Input Validation

To check that Ruckig is able to generate a trajectory before the actual calculation step,

```rust
ruckig.validate_input::<E>(input, check_current_state_within_limits, check_target_state_within_limits);
// returns Result<(), RuckigError>. If RuckigErrorHandler is ThrowErrorHandler, it returns Err(RuckigError::ValidationError) in case of error.
// If the error handler is IgnoreErrorHandler, it returns Ok(()) even if the input is invalid.
```

throws an error with a detailed reason if an input is not valid. You must set the template parameter
via e.g. `ruckig.validate_input<ThrowErrorHandler>(...)` . The two boolean arguments check that the current or target state
are within the limits. The check includes a typical catch of jerk-limited trajectory generation: When the current state
is at maximal velocity, any positive acceleration will inevitable lead to a velocity violation _at a future timestep_.
In general, this condition is fulfilled when

```
Abs(acceleration) <= Sqrt(2 * max_jerk * (max_velocity - Abs(velocity))).
```

If both arguments are set to true, the calculated trajectory is guaranteed to be _within the kinematic limits
throughout_ its duration. Also, note that there are range constraints of the input due to numerical reasons, see below
for more details.

### `RuckigResult` type

The `update` function of the Ruckig class returns a Result type that indicates the current state of the algorithm. This
can either be **working**, **finished** if the trajectory has finished, or an **error** type if something went wrong
during calculation. The result type can be compared as a standard integer.

| State                           | Error Code |
| ------------------------------- | ---------- |
| Working                         | 0          |
| Finished                        | 1          |
| Error                           | -1         |
| ErrorInvalidInput               | -100       |
| ErrorTrajectoryDuration         | -101       |
| ErrorPositionalLimits           | -102       |
| ErrorZeroLimits                 | -104       |
| ErrorExecutionTimeCalculation   | -110       |
| ErrorSynchronizationCalculation | -111       |

### Output Parameter

The output class includes the new kinematic state and the overall trajectory.

```.rs
new_position: DataArrayOrVec<f64, DOF>;
new_velocity: DataArrayOrVec<f64, DOF>;
new_acceleration: DataArrayOrVec<f64, DOF>;

trajectory: Trajectory; // The current trajectory
time: f64; // The current, auto-incremented time. Reset to 0 at a new calculation.

new_section: usize; // Index of the trajectory section (always 0 in this port - reserved for intermediate waypoints)
did_section_change: bool; // Was a new section reached in the last cycle?

new_calculation: bool; // Whether a new calculation was performed in the last cycle
was_calculation_interrupted: bool; // Reserved - calculation interruption is not implemented in this port
calculation_duration: f64; // Duration of the calculation in the last cycle [µs]
```

Moreover, the **trajectory** struct has a range of useful parameters and methods.

```.rs
duration: f64; // Duration of the trajectory
independent_min_durations: DataArrayOrVec<f64, DOF>; // Time-optimal profile for each independent DoF

<...> pub fn at_time(
        &self,
        time: f64,
        new_position: &mut Option<&mut DataArrayOrVec<f64, DOF>>,
        new_velocity: &mut Option<&mut DataArrayOrVec<f64, DOF>>,
        new_acceleration: &mut Option<&mut DataArrayOrVec<f64, DOF>>,
        new_jerk: &mut Option<&mut DataArrayOrVec<f64, DOF>>,
        new_section: &mut Option<usize>,
    ); // Get the kinematic state of the trajectory at a given time
<...> get_position_extrema(); // Returns information about the position extrema and their times
```

Again, we refer to the [API documentation](https://docs.ruckig.com) for the exact signatures. (C++ version only)

### Offline Calculation

Ruckig also supports an offline approach for calculating a trajectory:

```rust
result = ruckig.calculate(input, trajectory); // Returns  Result<RuckigResult, RuckigError>
```

When only using this method, the `Ruckig` constructor does not need a control cycle (`delta_time`) as an argument.
However if given, Ruckig supports stepping through the trajectory with

```rust
while ruckig.update(&input, &mut output).unwrap() == RuckigResult::Working {
  // Make use of the new state here!
  // e.g. robot.setJointPositions(output.new_position);
}
```

### Position Limits

Optional per-DoF position limits restrict the workspace of the system:

```rust
input.min_position = Some(daov_stack![-1.0]);
input.max_position = Some(daov_stack![1.0]);
// Use f64::INFINITY / f64::NEG_INFINITY entries to disable the limit for individual DoFs
```

The generated trajectory is guaranteed to stay inside the limits: candidate profiles are checked against their exact
position extrema, and if needed, the effective velocity limits of the affected DoF are reduced so that a full
(jerk-limited) stop always fits inside the remaining distance to the limit. The check also requires the trajectory's
end state to leave enough stopping margin, so the system can always stay inside the workspace afterwards. With the
velocity control interface, the effective target velocity is capped accordingly.

If the current kinematic state makes crossing a limit unavoidable (e.g. too fast, too close to the limit), the best
possible braking is applied and `RuckigResult::ErrorPositionalLimits` (-102) is returned - a violation is never
silent. Input validation also catches target states outside the limits and current/target states from which a stop
inside the limits is impossible.

See `samples/src/example_position_limits_1dof.rs` for a complete example.

### Tracking a Moving Target (Trackig)

The `Trackig` struct follows a moving target signal as closely as the kinematic limits allow (the equivalent of the
Ruckig Pro tracking interface, implemented independently). Every control cycle, the state-to-state calculation is
re-run from the current state toward the instantaneous target:

```rust
use rsruckig::prelude::*;

let mut otg = Trackig::<1, IgnoreErrorHandler>::new(None, 0.001);
let mut input = InputParameter::new(None);
let mut output = OutputParameter::new(None);
input.synchronization = Synchronization::None; // recommended for tracking
input.max_velocity[0] = 3.0;
input.max_acceleration[0] = 15.0;
input.max_jerk[0] = 150.0;

let mut target = TargetState::new(None);
loop {
    // Update the target signal, e.g. from a sensor:
    // target.position[0] = ...; target.velocity[0] = ...; target.acceleration[0] = ...;
    let _ = otg.update(&target, &input, &mut output).unwrap();
    output.pass_to_input(&mut input);
    // Use output.new_position / new_velocity / new_acceleration as setpoints
}
```

Properties:

- Unreachable targets are clamped into the reachable set: a target moving faster than the limits allow makes the
  output lag behind at maximum velocity instead of producing errors.
- Position limits (see above) compose with tracking: the target is clamped into the limits, in both arrival
  directions.
- If a cycle still fails numerically, the last successfully calculated trajectory is followed (never extrapolated
  past its end).
- The steady-state cycle is allocation-free for stack-allocated DoFs.
- An optional first-order low-pass filter (`target_filter_time_constant`) smooths noisy target signals.

See `samples/src/example_tracking_1dof.rs` for a complete example.

## Tests and Numerical Stability

The upstream C++ test suite validates over 5.000.000.000 random trajectories as well as many additional edge cases;
this port ships its own test suite with known-trajectory tables, seeded random fuzzing (including position limits for
1st/2nd/3rd-order profiles), tracking scenarios (sinusoid, step, unreachable ramp), regression tests for degenerate
boundary states, and an allocation-count check for the tracking control cycle. The
numerical exactness is tested for the final position and final velocity to be within `1e-8`, for the final acceleration
to be within `1e-10`, and for the velocity, acceleration and jerk limit to be within of a numerical error of `1e-12`.
These are absolute values - we suggest to scale your input so that these correspond to your required precision of the
system. For example, for most real-world systems we suggest to use input values in `[m]` (instead of e.g. `[mm]`),
as `1e-8m` is sufficient precise for practical trajectory generation. Furthermore, all kinematic limits should be
below `1e12`. The maximal supported trajectory duration is `7e3`. Note that Ruckig will also output values outside of
this range, there is however no guarantee for correctness. The Ruckig Pro version has additional tools to increase the
numerical range and improve reliability.

## Development

Original Ruckig is written in C++17. It is continuously tested on `ubuntu-latest`, `macos-latest`, and `windows-latest`
against following versions

Rust version is a port of the original C++ community version, excluding the cloud client. Two Ruckig Pro features -
position limits and the tracking interface - are implemented independently in this port.

## Rust port TODOs

- [x] Convert the RuckigError to a Rust enum
- [x] Add more tests (position limits, tracking, degenerate-input regressions)
- [x] Add more examples
- [x] Position limits (`min_position` / `max_position`)
- [x] Tracking interface (`Trackig`)
- [ ] Intermediate waypoints
- [ ] Add more documentation
- [ ] Further optimisation of performance

## Contributing

We welcome contributions! To contribute:

1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Submit a pull request with a clear description of your changes.

## License

This project is licensed under the MIT License. See the `LICENSE.txt` file for details.

## Citation

```
@article{berscheid2021jerk,
  title={Jerk-limited Real-time Trajectory Generation with Arbitrary Target States},
  author={Berscheid, Lars and Kr{\"o}ger, Torsten},
  journal={Robotics: Science and Systems XVII},
  year={2021}
}
```

<style>
    .centered {
        text-align: center;
    }
</style>
