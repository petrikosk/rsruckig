use rand_core::SeedableRng;
use rand_distr::{Distribution, Normal, Uniform};
use rand_pcg::Pcg64Mcg;
use rsruckig::error::ThrowErrorHandler;
use rsruckig::input_parameter::InputParameter;
use rsruckig::output_parameter::OutputParameter;
use rsruckig::ruckig::Ruckig;
use rsruckig::trajectory::Trajectory;
use std::time::Instant;

struct Randomizer<D>
where
    D: Distribution<f64>,
{
    rng: Pcg64Mcg,
    distribution: D,
    uniform_dist: Uniform<f64>,
}

impl<D> Randomizer<D>
where
    D: Distribution<f64>,
{
    pub fn new(distribution: D, local_seed: u64) -> Self {
        let rng = Pcg64Mcg::seed_from_u64(local_seed);
        let uniform_dist = Uniform::new(0.0, 1.0);
        Self {
            rng,
            distribution,
            uniform_dist,
        }
    }

    pub fn fill(&mut self, input: &mut [f64]) {
        for val in input.iter_mut() {
            *val = self.distribution.sample(&mut self.rng);
        }
    }

    pub fn fill_or_zero(&mut self, input: &mut [f64], p: f64) {
        for val in input.iter_mut() {
            if self.uniform_dist.sample(&mut self.rng) < p {
                *val = self.distribution.sample(&mut self.rng);
            } else {
                *val = 0.0;
            }
        }
    }

    pub fn fill_with_offset(&mut self, input: &mut [f64], offset: &[f64]) {
        for (val, &off) in input.iter_mut().zip(offset.iter()) {
            *val = self.distribution.sample(&mut self.rng) + off.abs();
        }
    }

    pub fn _fill_min_with_offset(&mut self, input: &mut [f64], offset: &[f64]) {
        for (val, &off) in input.iter_mut().zip(offset.iter()) {
            *val = self.distribution.sample(&mut self.rng) - off.abs();
        }
    }
}

fn _check_update(otg: &mut Ruckig, input: &InputParameter) -> f64 {
    let mut output = OutputParameter::new(input.degrees_of_freedom);
    let _result = otg.update(input, &mut output);
    output.calculation_duration
}

fn check_calculation(otg: &mut Ruckig, input: &InputParameter) -> f64 {
    let mut traj = Trajectory::new(input.degrees_of_freedom);
    let start = Instant::now();
    let _result = otg.calculate(input, &mut traj);
    let stop = Instant::now();
    stop.duration_since(start).as_nanos() as f64 / 1000.0
}

fn analyse(v: &Vec<f64>) -> (f64, f64) {
    let sum: f64 = v.iter().sum();
    let mean = sum / v.len() as f64;

    // Calculate differences from the mean and collect into a Vec<f64>
    let diff: Vec<f64> = v.iter().map(|&x| x - mean).collect();

    // Calculate the sum of squares of the differences
    let sq_sum: f64 = diff.iter().map(|&d| d * d).sum();

    // Calculate the standard deviation
    let std_deviation = (sq_sum / v.len() as f64).sqrt();

    (mean, std_deviation)
}

fn benchmark(n: &mut usize, number_of_trajectories: i64, degrees_of_freedom: usize, verbose: bool) {
    let mut otg = Ruckig::new(degrees_of_freedom, 0.005, true);

    let position_dist = Normal::new(0.0, 4.0).unwrap();
    let dynamic_dist = Normal::new(0.0, 0.8).unwrap();
    let limit_dist = Uniform::new(0.1, 12.0);

    let mut position_randomizer = Randomizer::new(position_dist, 42);
    let mut dynamic_randomizer = Randomizer::new(dynamic_dist, 43);
    let mut limit_randomizer = Randomizer::new(limit_dist, 44);

    let mut input = InputParameter::new(degrees_of_freedom);

    let mut average: Vec<f64> = Vec::new();
    let mut worst: Vec<f64> = Vec::new();
    let mut global: Vec<f64> = Vec::new();

    for _ in 0..*n {
        let mut average_ = 0.0;
        let mut worst_: f32 = 0.0;
        let mut n: usize = 1;

        let start = Instant::now();

        for _ in 0..number_of_trajectories {
            position_randomizer.fill(&mut input.current_position);
            dynamic_randomizer.fill_or_zero(&mut input.current_velocity, 0.9);
            dynamic_randomizer.fill_or_zero(&mut input.current_acceleration, 0.8);
            position_randomizer.fill(&mut input.target_position);
            dynamic_randomizer.fill_or_zero(&mut input.target_velocity, 0.7);

            // Assuming OTGType is RuckigThrow, filling target_acceleration.
            dynamic_randomizer.fill_or_zero(&mut input.target_acceleration, 0.6);

            // Using fill_with_offset for max_velocity and max_acceleration because
            // fill_uniform is not defined in the previous Randomizer. Adjust as necessary.
            limit_randomizer.fill_with_offset(
                &mut input.max_velocity,
                &input.target_velocity, // pass a reference to target_velocity
            );
            limit_randomizer.fill_with_offset(
                &mut input.max_acceleration,
                &input.target_acceleration, // pass a reference to target_acceleration
            );

            // fill_normal is not a method in the previous Randomizer, assuming it should be fill.
            limit_randomizer.fill(&mut input.max_jerk);

            // Perform the same validation as the C++ version.
            // Assume validate_input method is correctly defined.
            if let Ok(false) = otg.validate_input::<ThrowErrorHandler>(&input, false, false, false)
            {
                continue;
            }

            let time: f32 = check_calculation(&mut otg, &input) as f32;
            average_ = average_ + (time - average_) / n as f32;
            worst_ = worst_.max(time);
            n += 1;
        }
        let stop = Instant::now();
        let global_ =
            stop.duration_since(start).as_nanos() as f64 / 1000.0 / number_of_trajectories as f64;

        average.push(average_ as f64);
        worst.push(worst_ as f64);
        global.push(global_);
    }
    let (average_mean, average_std) = analyse(&average);
    let (worst_mean, worst_std) = analyse(&worst);
    let (global_mean, global_std) = analyse(&global);

    if verbose {
        println!("--------------------------------------------------");
        println!(
            "Benchmark for {} DoFs on {} trajectories",
            degrees_of_freedom, number_of_trajectories
        );
        println!(
            "Average calculation duration {:.4} pm {:.4} [µs]",
            average_mean, average_std
        );
        println!(
            "Worst calculation duration {:.4} pm {:.4} [µs]",
            worst_mean, worst_std
        );
        println!(
            "End-to-end Calculation Duration {:.4} pm {:.4} [µs]",
            global_mean, global_std
        );
    }
}

fn main() {
    let mut n = 2 * 5;
    let number_of_trajectories = 4 * 64 * 1024;

    let dofs = 3;
    benchmark(&mut n, number_of_trajectories, dofs, true);
}
