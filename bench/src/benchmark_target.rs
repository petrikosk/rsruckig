//! Benchmark of the trajectory calculation on random state-to-state inputs.
//!
//! Prints the calculation durations and writes them as a bar chart to
//! `rsruckig-benchmark_target.svg` in the current working directory. The chart
//! is drawn with the pure-Rust `plotters` crate, like the examples.

use plotters::prelude::*;
use rand_core::SeedableRng;
use rand_distr::{Distribution, Normal, Uniform};
use rand_pcg::Pcg64Mcg;
use rsruckig::prelude::*;
use std::error::Error;
use std::time::Instant;

/// Mean and standard deviation of a measurement over the benchmark runs
#[derive(Clone, Copy)]
struct Statistic {
    mean: f64,
    std: f64,
}

impl Statistic {
    fn of(values: &[f64]) -> Self {
        let count = values.len() as f64;
        let mean = values.iter().sum::<f64>() / count;
        let variance = values.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / count;
        Self {
            mean,
            std: variance.sqrt(),
        }
    }
}

/// Calculation durations in microseconds
struct BenchmarkResults {
    degrees_of_freedom: usize,
    number_of_trajectories: usize,
    /// Average duration of a single calculation
    average: Statistic,
    /// Worst duration of a single calculation
    worst: Statistic,
    /// Duration per trajectory including the input generation
    global: Statistic,
}

struct Randomizer<D: Distribution<f64>> {
    rng: Pcg64Mcg,
    distribution: D,
    uniform: Uniform<f64>,
}

impl<D: Distribution<f64>> Randomizer<D> {
    fn new(distribution: D, seed: u64) -> Self {
        Self {
            rng: Pcg64Mcg::seed_from_u64(seed),
            distribution,
            uniform: Uniform::new(0.0, 1.0),
        }
    }

    fn fill(&mut self, values: &mut [f64]) {
        for value in values {
            *value = self.distribution.sample(&mut self.rng);
        }
    }

    /// Samples each value with probability `p`, and sets it to zero otherwise
    fn fill_or_zero(&mut self, values: &mut [f64], p: f64) {
        for value in values {
            *value = if self.uniform.sample(&mut self.rng) < p {
                self.distribution.sample(&mut self.rng)
            } else {
                0.0
            };
        }
    }

    /// Samples each value above the magnitude of its offset
    fn fill_with_offset(&mut self, values: &mut [f64], offsets: &[f64]) {
        for (value, offset) in values.iter_mut().zip(offsets) {
            *value = self.distribution.sample(&mut self.rng) + offset.abs();
        }
    }
}

fn microseconds_since(start: Instant) -> f64 {
    start.elapsed().as_nanos() as f64 / 1000.0
}

fn benchmark<const DOF: usize>(runs: usize, number_of_trajectories: usize) -> BenchmarkResults {
    let mut otg = Ruckig::<DOF, ThrowErrorHandler>::new(None, 0.005);
    let mut input = InputParameter::new(None);
    let mut traj = Trajectory::new(None);

    let mut position = Randomizer::new(Normal::new(0.0, 4.0).unwrap(), 42);
    let mut dynamic = Randomizer::new(Normal::new(0.0, 0.8).unwrap(), 43);
    let mut limit = Randomizer::new(Uniform::new(0.1, 12.0), 44);

    let mut averages = Vec::with_capacity(runs);
    let mut worsts = Vec::with_capacity(runs);
    let mut globals = Vec::with_capacity(runs);

    for _ in 0..runs {
        let mut total = 0.0;
        let mut worst: f64 = 0.0;
        let mut calculations = 0_usize;

        let start = Instant::now();
        for _ in 0..number_of_trajectories {
            position.fill(&mut input.current_position);
            dynamic.fill_or_zero(&mut input.current_velocity, 0.9);
            dynamic.fill_or_zero(&mut input.current_acceleration, 0.8);
            position.fill(&mut input.target_position);
            dynamic.fill_or_zero(&mut input.target_velocity, 0.7);
            dynamic.fill_or_zero(&mut input.target_acceleration, 0.6);
            limit.fill_with_offset(&mut input.max_velocity, &input.target_velocity);
            limit.fill_with_offset(&mut input.max_acceleration, &input.target_acceleration);
            limit.fill(&mut input.max_jerk);

            if otg.validate_input(&input, false, false).is_err() {
                continue;
            }

            let calculation_start = Instant::now();
            // Failed calculations are part of the measured workload
            let _ = std::hint::black_box(otg.calculate(&input, &mut traj));
            let duration = microseconds_since(calculation_start);

            total += duration;
            worst = worst.max(duration);
            calculations += 1;
        }
        globals.push(microseconds_since(start) / number_of_trajectories as f64);
        averages.push(total / calculations.max(1) as f64);
        worsts.push(worst);
    }

    BenchmarkResults {
        degrees_of_freedom: DOF,
        number_of_trajectories,
        average: Statistic::of(&averages),
        worst: Statistic::of(&worsts),
        global: Statistic::of(&globals),
    }
}

impl BenchmarkResults {
    fn title(&self) -> String {
        format!(
            "Benchmark for {} DoFs on {} trajectories",
            self.degrees_of_freedom, self.number_of_trajectories
        )
    }

    fn metrics(&self) -> [(&'static str, Statistic); 3] {
        [
            ("Average", self.average),
            ("Worst", self.worst),
            ("End-to-end", self.global),
        ]
    }

    fn print(&self) {
        println!("--------------------------------------------------");
        println!("{}", self.title());
        for (name, statistic) in self.metrics() {
            println!(
                "{name} calculation duration {:.4} pm {:.4} [µs]",
                statistic.mean, statistic.std
            );
        }
    }

    /// Draws the mean durations as bars with their standard deviations as
    /// error bars. The duration axis is logarithmic, as the worst case is
    /// orders of magnitude above the average.
    fn plot(&self, path: &str) -> Result<(), Box<dyn Error>> {
        let metrics = self.metrics();
        let lowest = metrics
            .iter()
            .map(|(_, s)| s.mean)
            .fold(f64::INFINITY, f64::min);
        let highest = metrics
            .iter()
            .map(|(_, s)| s.mean + s.std)
            .fold(0.0, f64::max);
        // Whole decades, so that the axis gets labelled ticks
        let y_min = 10_f64.powf(lowest.log10().floor());
        let y_max = 10_f64.powf(highest.log10().ceil());

        let root = SVGBackend::new(path, (960, 640)).into_drawing_area();
        root.fill(&WHITE)?;

        let mut chart = ChartBuilder::on(&root)
            .caption(self.title(), ("sans-serif", 25))
            .margin(20)
            .x_label_area_size(45)
            .y_label_area_size(70)
            .build_cartesian_2d(
                (0..metrics.len() - 1).into_segmented(),
                (y_min..y_max).log_scale(),
            )?;

        chart
            .configure_mesh()
            .disable_x_mesh()
            .y_desc("Duration [µs]")
            .x_label_formatter(&|segment| match segment {
                SegmentValue::CenterOf(index) => metrics
                    .get(*index)
                    .map_or_else(String::new, |(name, _)| name.to_string()),
                _ => String::new(),
            })
            .draw()?;

        chart.draw_series(metrics.iter().enumerate().map(|(index, (_, statistic))| {
            let mut bar = Rectangle::new(
                [
                    (SegmentValue::Exact(index), y_min),
                    (SegmentValue::Exact(index + 1), statistic.mean),
                ],
                BLUE.mix(0.6).filled(),
            );
            bar.set_margin(0, 0, 30, 30);
            bar
        }))?;

        chart.draw_series(metrics.iter().enumerate().map(|(index, (_, statistic))| {
            let center = SegmentValue::CenterOf(index);
            PathElement::new(
                [
                    (center.clone(), (statistic.mean - statistic.std).max(y_min)),
                    (center, statistic.mean + statistic.std),
                ],
                RED.stroke_width(3),
            )
        }))?;

        root.present()?;
        Ok(())
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let runs = 10;
    let number_of_trajectories = 4 * 64 * 1024;

    let results = benchmark::<3>(runs, number_of_trajectories);
    results.print();

    let path = concat!(env!("CARGO_BIN_NAME"), ".svg");
    results.plot(path)?;
    println!("Chart written to {path}");
    Ok(())
}
