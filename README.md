<div class="centered">
  <h1 class="centered">Ruckig</h1>
  <h3 class="centered">
    Instantaneous Motion Generation for Robots and Machines.
  </h3>
</div>

This is a Rust port of the repository https://github.com/pantor/ruckig/. Cloud client and pro-features are not ported. The
examples use Gnuplot to illustrate the trajectories.

Ruckig generates trajectories on-the-fly, allowing robots and machines to react instantaneously to sensor input. Ruckig
calculates a trajectory to a *target* waypoint (with position, velocity, and acceleration) starting from *any* initial
state limited by velocity, acceleration, and jerk constraints. Besides the target state, Ruckig allows to define
intermediate positions for waypoint following. For state-to-state motions, Ruckig guarantees a time-optimal solution.
With intermediate waypoints, Ruckig calculates the path and its time parametrization jointly, resulting in significantly
faster trajectories compared to traditional methods.

More information can be found at [ruckig.com](https://ruckig.com) and in the corresponding
paper [Jerk-limited Real-time Trajectory Generation with Arbitrary Target States](https://arxiv.org/abs/2105.04830),
accepted for the *Robotics: Science and Systems (RSS), 2021* conference.

## Installation

```bash
cargo build --release
```

## Documentation (incomplete)
```bash
cargo doc --open
```

## Tutorial

Let's get started!

![Trajectory Profile](https://github.com/petrikosk/rsruckig/raw/master/doc/example_profile.png?raw=true)

### Waypoint-based Trajectory Generation

Ruckig provides three main interface classes: the *Ruckig*, the *InputParameter*, and the *OutputParameter* class.

First, you'll need to create a Ruckig instance with the ```DOF``` and ```RuckigErrorHandler``` as a template parameter,
and the number of DOF as an option and the control cycle (
e.g. in seconds) in the constructor.

You can opt to use template parameter to specify the number of DOF and use stack allocation, or use the constructor to
specify the number of DOF as ```Option<usize>```.
If you use dynamic number of DOF, the template DOF parameter must be set to 0 to allocate dofs dynamically.
Any number greater than 0 as a template parameter, will result in omitting the dynamic DOF parameter.
In case of dynamic DOF allocation, a good practice is to set constructor DOF parameter to None.

```.rs
use rsruckig::prelude::*;

// stack allocation using template parameter
let mut ruckig = Ruckig::<6, ThrowErrorHandler>::new(None, 0.01); // Number DoFs; control cycle in [s]

// dynamic allocation
let mut ruckig = Ruckig::<0, ThrowErrorHandler>::new(Some(6), 0.01); // Number DoFs; control cycle in [s]
```
Implemented error handlers are:
- ```ThrowErrorHandler``` - throws an error with a detailed reason if an input is not valid.
- ```IgnoreErrorHandler``` - ignores the error and returns ```Ok(RuckigResult)```.

To implement your own error handler, you need to implement the ```RuckigErrorHandler``` trait:
```.rs
pub trait RuckigErrorHandler {
    fn handle_validation_error(message: &str) -> Result<bool, RuckigError>;
    fn handle_calculator_error(
        message: &str,
        result: RuckigResult,
    ) -> Result<RuckigResult, RuckigError>;
}
```

The input type has 3 blocks of data: the *current* state, the *target* state and the corresponding kinematic *limits*.

```.rs
// Stack DOF allocation
let mut input = InputParameter::new(None); // Number DoFs
input.current_position = daov_stack![[0.2, ...];
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

```.rs
while ruckig.update(&input, &mut output).unwrap() == RuckigResult::Working {
  // Make use of the new state here!
  // e.g. robo.setJointPositions(output.new_position);

  output.pass_to_input(&mut input); // Don't forget this!
}
```

Within the control loop, you need to update the *current state* of the input parameter according to the calculated
trajectory. Therefore, the `pass_to_input` method copies the new kinematic state of the output to the current kinematic
state of the input parameter. If (in the next step) the current state is not the expected, pre-calculated trajectory,
Ruckig will calculate a new trajectory based on the novel input. When the trajectory has reached the target state,
the `update` function will return `Result::Finished`.

### DataArrayOrVec

The `DataArrayOrVec` type is a wrapper around a fixed-size array or a vector. It is mainly used to store the kinematic state.
User can choose between stack and heap allocation. The stack allocation is faster, but the number of DOF must be known at
compile time. The heap allocation is slower, but the number of DOF can be set at runtime.

```.rs
pub enum DataArrayOrVec<T, const N: usize>
where T: std::fmt::Debug {
Stack([T; N]),
Heap(Vec<T>),
}    
```

`DataArrayOrVec` has two associated macros to stramline the instantation process with these shorthand forms:

- `daov_stack!`
- `daov_heap!`

Example of usage:    

```.rs
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

```.rs
### Input Parameter

To go into more detail, the *InputParameter* type has following members:

current_position: DataArrayOrVec<f64, DOF>;
current_velocity: DataArrayOrVec<f64, DOF>; // Initialized to zero
current_acceleration: DataArrayOrVec<f64, DOF>; // Initialized to zero

target_position: DataArrayOrVec<f64, DOF>;
target_velocity: VDataArrayOrVec<f64, DOF>; // Initialized to zero
target_acceleration: DataArrayOrVec<f64, DOF>; // Initialized to zero

max_velocity: DataArrayOrVec<f64, DOF>;
max_acceleration: DataArrayOrVec<f64, DOF>;
max_jerk: DataArrayOrVec<f64, DOF>; // Initialized to infinity

min_velocity: Option<DataArrayOrVec<f64, DOF>>; // If not given, the negative maximum velocity will be used.
min_acceleration: Option<DataArrayOrVec<f64, DOF>>; // If not given, the negative maximum acceleration will be used.

enabled: Vec<bool>; // Initialized to true
minimum_duration: Option<f64>;

control_interface: ControlInterface; // The default position interface controls the full kinematic state.
synchronization: Synchronization; // Synchronization behavior of multiple DoFs
duration_discretization: DurationDiscretization; // Whether the duration should be a discrete multiple of the control cycle (off by default)

per_dof_control_interface: Option<Vec<ControlInterface>>; // Sets the control interface for each DoF individually, overwrites global control_interface
per_dof_synchronization: Option<Vec<Synchronization>>; // Sets the synchronization for each DoF individually, overwrites global synchronization
```

On top of the current state, target state, and constraints, Ruckig allows for a few more advanced settings:

- A *minimum* velocity and acceleration can be specified - these should be a negative number. If they are not given, the
  negative maximum velocity or acceleration will be used (similar to the jerk limit). For example, this might be useful
  in human robot collaboration settings with a different velocity limit towards a human. Or, when switching between
  different moving coordinate frames like picking from a conveyer belt.
- You can overwrite the global kinematic limits to specify limits for each section between two waypoints separately by
  using e.g. `per_section_max_velocity`.
- If a DoF is not *enabled*, it will be ignored in the calculation. Ruckig will output a trajectory with constant
  acceleration for those DoFs.
- A *minimum duration* can be optionally given. Note that Ruckig can not guarantee an exact, but only a minimum duration
  of the trajectory.
- The control interface (position or velocity control) can be switched easily. For example, a stop trajectory or visual
  servoing can be easily implemented with the velocity interface.
- Different synchronization behaviors (i.a. phase, time, or no synchonization) are implemented. Phase synchronization
  results in straight-line motions.
- The trajectory duration might be constrained to a multiple of the control cycle. This way, the *exact* state can be
  reached at a control loop execution.

### Input Validation

To check that Ruckig is able to generate a trajectory before the actual calculation step,

```.rs
ruckig.validate_input<E: RuckigErrorHandler>(input, check_current_state_within_limits, check_target_state_within_limits);
// returns Result<bool, RuckigError>. If RuckigErrorHandler is ThrowErrorHandler, it returns Err(RuckigError::ValidationError) in case of error.
// If the error handler is IgnoreErrorHandler, it returns Ok(true) if the input is valid, and Ok(false) if the input is invalid.
```

throws an error with a detailed reason if an input is not valid. You must set the template parameter
via e.g. `ruckig.validate_input<ThrowErrorHandler>(...)` . The two boolean arguments check that the current or target state
are within the limits. The check includes a typical catch of jerk-limited trajectory generation: When the current state
is at maximal velocity, any positive acceleration will inevitable lead to a velocity violation *at a future timestep*.
In general, this condition is fulfilled when

```
Abs(acceleration) <= Sqrt(2 * max_jerk * (max_velocity - Abs(velocity))).
```

If both arguments are set to true, the calculated trajectory is guaranteed to be *within the kinematic limits
throughout* its duration. Also, note that there are range constraints of the input due to numerical reasons, see below
for more details.

### ```RuckigResult``` type

The `update` function of the Ruckig class returns a Result type that indicates the current state of the algorithm. This
can either be **working**, **finished** if the trajectory has finished, or an **error** type if something went wrong
during calculation. The result type can be compared as a standard integer.

| State                           | Error Code |
|---------------------------------|------------|
| Working                         | 0          |
| Finished                        | 1          |
| Error                           | -1         |
| ErrorInvalidInput               | -100       |
| ErrorTrajectoryDuration         | -101       |
| ErrorPositionalLimits           | -102       |
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

new_section: usize; // Index of the section between two (possibly filtered) intermediate positions.
did_section_change: bool; // Was a new section reached in the last cycle?

new_calculation: bool; // Whether a new calculation was performed in the last cycle
was_calculation_interrupted: bool; // Was the trajectory calculation interrupted? (only in Pro Version)
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

```.rs
result = ruckig.calculate(input, trajectory); // Returns  Result<RuckigResult, RuckigError>
```

When only using this method, the `Ruckig` constructor does not need a control cycle (`delta_time`) as an argument.
However if given, Ruckig supports stepping through the trajectory with

```.rs
while (ruckig.update(&input, &mut output).unwrap() == RuckigResult::Working {
  // Make use of the new state here!
  // e.g. robot.setJointPositions(output.new_position);
}
```

## Tests and Numerical Stability

The current test suite validates over 5.000.000.000 random trajectories as well as many additional edge cases. The
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

Rust version is a port of the original C++ version, excluding Pro features and cloud client.

## Rust port TODOs

- [ ] Add more tests
- [ ] Convert the RuckigError to a Rust enum
- [ ] Add more examples
- [ ] Add more documentation
- [ ] Further optimisation of performance

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