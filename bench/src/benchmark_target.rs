use gnuplot::{AxesCommon, Caption, Color, Figure, Tick};
use rand_core::SeedableRng;
use rand_distr::{Distribution, Normal, Uniform};
use rand_pcg::Pcg64Mcg;
use rsruckig::error::RuckigErrorHandler;
use rsruckig::prelude::*;
use std::time::Instant;

struct BenchmarkResults {
    degrees_of_freedom: usize,
    number_of_trajectories: i64,
    average_mean: f64,
    average_std: f64,
    worst_mean: f64,
    worst_std: f64,
    global_mean: f64,
    global_std: f64,
}

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

fn _check_update<const DOF: usize, E: RuckigErrorHandler>(otg: &mut Ruckig<DOF, E>, input: &InputParameter<DOF>) -> f64 {
    let mut output = OutputParameter::new(None);
    let _result = otg.update(input, &mut output);
    output.calculation_duration
}

fn check_calculation<const DOF: usize, E: RuckigErrorHandler>(otg: &mut Ruckig<DOF, E>, input: &InputParameter<DOF>) -> f64 {
    let mut traj = Trajectory::new(None);
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

fn benchmark<const DOF: usize>(
    n: &mut usize,
    number_of_trajectories: i64,
    verbose: bool,
) -> BenchmarkResults {
    let mut otg = Ruckig::<DOF, ThrowErrorHandler>::new(None, 0.005);

    let position_dist = Normal::new(0.0, 4.0).unwrap();
    let dynamic_dist = Normal::new(0.0, 0.8).unwrap();
    let limit_dist = Uniform::new(0.1, 12.0);

    let mut position_randomizer = Randomizer::new(position_dist, 42);
    let mut dynamic_randomizer = Randomizer::new(dynamic_dist, 43);
    let mut limit_randomizer = Randomizer::new(limit_dist, 44);

    let mut input = InputParameter::new(None);

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
            dynamic_randomizer.fill_or_zero(&mut input.target_acceleration, 0.6);

            limit_randomizer.fill_with_offset(
                &mut input.max_velocity,
                &input.target_velocity, // pass a reference to target_velocity
            );
            limit_randomizer.fill_with_offset(
                &mut input.max_acceleration,
                &input.target_acceleration, // pass a reference to target_acceleration
            );

            limit_randomizer.fill(&mut input.max_jerk);

            if let Ok(false) = otg.validate_input(&input, false, false) {
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
            DOF, number_of_trajectories
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
    BenchmarkResults {
        degrees_of_freedom: DOF,
        number_of_trajectories,
        average_mean,
        average_std,
        worst_mean,
        worst_std,
        global_mean,
        global_std,
    }
}

fn plot_benchmark_results(benchmark_results: BenchmarkResults) {
    let mut fg = Figure::new();

    // Data for the bar chart
    let metrics = ["Average", "Worst", "Global"];
    let means = [
        benchmark_results.average_mean,
        benchmark_results.worst_mean,
        benchmark_results.global_mean,
    ];
    let std_devs = [
        benchmark_results.average_std,
        benchmark_results.worst_std,
        benchmark_results.global_std,
    ];
    let positions: Vec<f64> = (0..metrics.len()).map(|x| x as f64).collect();

    // Creating custom ticks for the x-axis
    let custom_ticks: Vec<Tick<f64, String>> = positions
        .iter()
        .zip(metrics.iter())
        .map(|(&pos, &label)| Tick::Major(pos, gnuplot::Fix(label.to_string())))
        .collect();

    fg.axes2d()
        .set_title(
            &format!(
                "Benchmark for {} DoFs on {} trajectories",
                benchmark_results.degrees_of_freedom, benchmark_results.number_of_trajectories
            ),
            &[],
        )
        .set_x_label("Metrics", &[])
        .set_y_label("Duration (µs)", &[])
        .set_x_ticks_custom(custom_ticks.iter(), &[], &[])
        .boxes(
            positions.iter(),
            means.iter(),
            &[Caption("Mean"), Color("blue")],
        )
        .y_error_lines(
            positions.iter(),
            means.iter(),
            std_devs.iter(),
            &[Caption("Std Dev"), Color("red")],
        );
    fg.show().unwrap();
}

fn main() {
    let mut n = 2 * 5;
    let number_of_trajectories = 4 * 64 * 1024;

    let results = benchmark::<3>(&mut n, number_of_trajectories, true);
    plot_benchmark_results(results);
}
