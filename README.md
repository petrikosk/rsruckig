<div class="centered">
  <h1 class="centered">Ruckig</h1>
  <h3 class="centered">
    Instantaneous Motion Generation for Robots and Machines.
  </h3>
</div>

This is a Rust port of the repository https://github.com/pantor/ruckig/. Cloud client features are not ported. The
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

![Trajectory Profile](https://github.com/petrikosk/rsruckig/raw/master/doc/example_profile.jpg?raw=true)

### Waypoint-based Trajectory Generation

Ruckig provides three main interface classes: the *Ruckig*, the *InputParameter*, and the *OutputParameter* class.

First, you'll need to create a Ruckig instance with the number of DoFs as a template parameter, and the control cycle (
e.g. in seconds) in the constructor.

```.rs
let mut ruckig = Ruckig::new(6, 0.01, true); // Number DoFs; control cycle in [s]; throw error 
```

The input type has 3 blocks of data: the *current* state, the *target* state and the corresponding kinematic *limits*.

```.rs
let mut input = InputParameter::new(6); // Number DoFs
input.current_position = vec![0.2, ...];
input.current_velocity = vec![0.1, ...];
input.current_acceleration = vec![0.1, ...];
input.target_position = vec![0.5, ...];
input.target_velocity = vec![-0.1, ...];
input.target_acceleration = vec![0.2, ...];
input.max_velocity = vec![0.4, ...];
input.max_acceleration = vec![1.0, ...];
input.max_jerk = vec![4.0, ...];

let mut output: OutputParameter = OutputParameter::new(6); // Number DoFs
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

### Input Parameter

To go into more detail, the *InputParameter* type has following members:

```.rs

current_position: Vec<f64>;
current_velocity: Vec<f64>; // Initialized to zero
current_acceleration: Vec<f64>; // Initialized to zero

target_position: Vec<f64>;
target_velocity: Vec<f64>; // Initialized to zero
target_acceleration: Vec<f64>; // Initialized to zero

max_velocity: Vec<f64>;
max_acceleration: Vec<f64>;
max_jerk: Vec<f64>; // Initialized to infinity

min_velocity: Option<Vec<f64>>; // If not given, the negative maximum velocity will be used.
min_acceleration: Option<Vec<f64>>; // If not given, the negative maximum acceleration will be used.

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

We refer to the [API documentation](https://docs.ruckig.com/namespaceruckig.html) of the enumerations within
the `ruckig` namespace for all available options.

### Input Validation

To check that Ruckig is able to generate a trajectory before the actual calculation step,

```.rs
ruckig.validate_input<T: ValidationErrorHandler>(input, throw_validation_error, check_current_state_within_limits, check_target_state_within_limits);
// returns Result<bool, RuckigError>. If throw_validation_error is true, it returns Err(RuckigError::ValidationError) in case of error.
// Otherwise, it returns Ok(true) if the input is valid, and Ok(false) if the input is invalid.
```

throws an error with a detailed reason if an input is not valid. You can also set the default template parameter
via `ruckig.validate_input<ThrowErrorHandler>(...)` . The two boolean arguments check that the current or target state
are within the limits. The check includes a typical catch of jerk-limited trajectory generation: When the current state
is at maximal velocity, any positive acceleration will inevitable lead to a velocity violation *at a future timestep*.
In general, this condition is fulfilled when

```
Abs(acceleration) <= Sqrt(2 * max_jerk * (max_velocity - Abs(velocity))).
```

If both arguments are set to true, the calculated trajectory is guaranteed to be *within the kinematic limits
throughout* its duration. Also, note that there are range constraints of the input due to numerical reasons, see below
for more details.

### Result Type

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
new_position: Vec<f64>;
new_velocity: Vec<f64>;
new_acceleration: Vec<f64>;

trajectory: Trajectory; // The current trajectory
time: f64; // The current, auto-incremented time. Reset to 0 at a new calculation.

new_section: usize; // Index of the section between two (possibly filtered) intermediate positions.
did_section_change: bool; // Was a new section reached in the last cycle?

new_calculation: bool; // Whether a new calculation was performed in the last cycle
was_calculation_interrupted: bool; // Was the trajectory calculation interrupted? (only in Pro Version)
calculation_duration: f64; // Duration of the calculation in the last cycle [µs]
```

Moreover, the **trajectory** class has a range of useful parameters and methods.

```.rs
duration: f64; // Duration of the trajectory
independent_min_durations: Vec<f64>; // Time-optimal profile for each independent DoF

<...> pub fn at_time(
        &self,
        time: f64,
        new_position: &mut Option<&mut Vec<f64>>,
        new_velocity: &mut Option<&mut Vec<f64>>,
        new_acceleration: &mut Option<&mut Vec<f64>>,
        new_jerk: &mut Option<&mut Vec<f64>>,
        new_section: &mut Option<usize>,
    ); // Get the kinematic state of the trajectory at a given time
<...> get_position_extrema(); // Returns information about the position extrema and their times
```

Again, we refer to the [API documentation](https://docs.ruckig.com) for the exact signatures.

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

## Benchmark

We find that Ruckig is more than twice as fast as Reflexxes Type IV for state-to-state motions and well-suited for
control cycles as low as 250 microseconds. The Ruckig *Community Version* is in general a more powerful and open-source
alternative to the [Reflexxes Type IV](http://reflexxes.ws/) library. In fact, Ruckig is the first Type V trajectory
generator for arbitrary target states and even supports directional velocity and acceleration limits, while also being
faster on top.

![Benchmark](https://github.com/pantor/ruckig/raw/master/doc/benchmark.png?raw=true)

For trajectories with intermediate waypoints, we compare Ruckig to [Toppra](https://github.com/hungpham2511/toppra), a
state-of-the-art library for robotic motion planning. Ruckig is able to improve the trajectory duration on average by
around 10%, as the path planning and time parametrization are calculated jointly. Moreover, Ruckig is real-time capable
and supports jerk-constraints.

![Benchmark](https://github.com/pantor/ruckig/raw/master/doc/ruckig_toppra_example.png?raw=true)

## Development

Original Ruckig is written in C++17. It is continuously tested on `ubuntu-latest`, `macos-latest`, and `windows-latest`
against following versions

Rust version is a port of the original C++ version, excluding Pro features and cloud client.

## Used By

Ruckig is used by over hundred research labs, companies, and open-source projects worldwide, including:

- [MoveIt 2](https://moveit.ros.org) for trajectory generation.
- [CoppeliaSim](https://www.coppeliarobotics.com/) starting from version 4.3.
- [Fuzzy Logic Robotics](https://flr.io)
- [Gestalt Robotics](https://www.gestalt-robotics.com)
- [Struckig](https://github.com/stefanbesler/struckig), a port of Ruckig to Structered Text (ST - IEC61131-3) for usage
  on PLCs.
- [Scanlab](https://www.scanlab.de/de) for controlling lasers.
- [Frankx](https://github.com/pantor/frankx) for controlling the Franka Emika robot arm.
- [Wiredworks](https://wiredworks.com) made a simple
  Kivy [GUI application](https://github.com/wiredworks/ruckig-showcase)
- [rsruckig](https://github.com/petrikosk/rsruckig)
- and many others!

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