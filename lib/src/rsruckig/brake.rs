use crate::util::integrate;
use std::f64;

const EPS: f64 = 2.2e-14;

#[inline]
fn v_at_t(v0: f64, a0: f64, j: f64, t: f64) -> f64 {
    v0 + t * (a0 + j * t / 2.0)
}

#[inline]
fn v_at_a_zero(v0: f64, a0: f64, j: f64) -> f64 {
    v0 + (a0 * a0) / (2.0 * j)
}

#[derive(Debug, Clone, Default)]
pub struct BrakeProfile {
    pub duration: f64,
    pub t: [f64; 2],
    pub j: [f64; 2],
    pub a: [f64; 2],
    pub v: [f64; 2],
    pub p: [f64; 2],
}

impl BrakeProfile {
    pub fn new() -> Self {
        BrakeProfile {
            duration: 0.0,
            t: [0.0; 2],
            j: [0.0; 2],
            a: [0.0; 2],
            v: [0.0; 2],
            p: [0.0; 2],
        }
    }

    fn acceleration_brake(
        &mut self,
        v0: f64,
        a0: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) {
        self.j[0] = -j_max;

        let t_to_a_max = (a0 - a_max) / j_max;
        let t_to_a_zero = a0 / j_max;

        let v_at_a_max = v_at_t(v0, a0, -j_max, t_to_a_max);
        let v_at_a_zero = v_at_t(v0, a0, -j_max, t_to_a_zero);

        if (v_at_a_zero > v_max && j_max > 0.0) || (v_at_a_zero < v_max && j_max < 0.0) {
            self.velocity_brake(v0, a0, v_max, v_min, a_max, a_min, j_max);
        } else if (v_at_a_max < v_min && j_max > 0.0) || (v_at_a_max > v_min && j_max < 0.0) {
            let t_to_v_min = -(v_at_a_max - v_min) / a_max;
            let t_to_v_max = -a_max / (2.0 * j_max) - (v_at_a_max - v_max) / a_max;

            self.t[0] = t_to_a_max + EPS;
            self.t[1] = t_to_v_min.min((t_to_v_max - EPS).max(0.0));
        } else {
            self.t[0] = t_to_a_max + EPS;
        }
    }

    fn velocity_brake(
        &mut self,
        v0: f64,
        a0: f64,
        v_max: f64,
        v_min: f64,
        _a_max: f64,
        a_min: f64,
        j_max: f64,
    ) {
        self.j[0] = -j_max;
        let t_to_a_min = (a0 - a_min) / j_max;
        let t_to_v_max = a0 / j_max + ((a0 * a0 + 2.0 * j_max * (v0 - v_max)).sqrt()) / j_max.abs();
        let t_to_v_min = a0 / j_max + ((a0 * a0 / 2.0 + j_max * (v0 - v_min)).sqrt()) / j_max.abs();
        let t_min_to_v_max = t_to_v_max.min(t_to_v_min);

        if t_to_a_min < t_min_to_v_max {
            let v_at_a_min = v_at_t(v0, a0, -j_max, t_to_a_min);
            let t_to_v_max_with_constant = -(v_at_a_min - v_max) / a_min;
            let t_to_v_min_with_constant = a_min / (2.0 * j_max) - (v_at_a_min - v_min) / a_min;

            self.t[0] = (t_to_a_min - EPS).max(0.0);
            self.t[1] = t_to_v_max_with_constant
                .min(t_to_v_min_with_constant)
                .max(0.0);
        } else {
            self.t[0] = (t_min_to_v_max - EPS).max(0.0);
        }
    }

    pub fn get_position_brake_trajectory(
        &mut self,
        v0: f64,
        a0: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
        j_max: f64,
    ) {
        self.t[0] = 0.0;
        self.t[1] = 0.0;
        self.j[0] = 0.0;
        self.j[1] = 0.0;

        if j_max == 0.0 || a_max == 0.0 || a_min == 0.0 {
            return; // Ignore braking for zero-limits
        }

        if a0 > a_max {
            self.acceleration_brake(v0, a0, v_max, v_min, a_max, a_min, j_max);
        } else if a0 < a_min {
            self.acceleration_brake(v0, a0, v_min, v_max, a_min, a_max, -j_max);
        } else if (v0 > v_max && v_at_a_zero(v0, a0, -j_max) > v_min)
            || (a0 > 0.0 && v_at_a_zero(v0, a0, j_max) > v_max)
        {
            self.velocity_brake(v0, a0, v_max, v_min, a_max, a_min, j_max);
        } else if (v0 < v_min && v_at_a_zero(v0, a0, j_max) < v_max)
            || (a0 < 0.0 && v_at_a_zero(v0, a0, -j_max) < v_min)
        {
            self.velocity_brake(v0, a0, v_min, v_max, a_min, a_max, -j_max);
        }
    }

    pub fn get_second_order_position_brake_trajectory(
        &mut self,
        v0: f64,
        v_max: f64,
        v_min: f64,
        a_max: f64,
        a_min: f64,
    ) {
        self.t[0] = 0.0;
        self.t[1] = 0.0;
        self.j[0] = 0.0;
        self.j[1] = 0.0;
        self.a[0] = 0.0;
        self.a[1] = 0.0;

        if a_max == 0.0 || a_min == 0.0 {
            return; // Ignore braking for zero-limits
        }

        if v0 > v_max {
            self.a[0] = a_min;
            self.t[0] = (v_max - v0) / a_min + f64::EPSILON;
        } else if v0 < v_min {
            self.a[0] = a_max;
            self.t[0] = (v_min - v0) / a_max + f64::EPSILON;
        }
    }

    pub fn get_velocity_brake_trajectory(&mut self, a0: f64, a_max: f64, a_min: f64, j_max: f64) {
        self.t[0] = 0.0;
        self.t[1] = 0.0;
        self.j[0] = 0.0;
        self.j[1] = 0.0;

        if j_max == 0.0 {
            return; // Ignore braking for zero-limits
        }

        if a0 > a_max {
            self.j[0] = -j_max;
            self.t[0] = (a0 - a_max) / j_max + std::f64::EPSILON;
        } else if a0 < a_min {
            self.j[0] = j_max;
            self.t[0] = -(a0 - a_min) / j_max + std::f64::EPSILON;
        }
    }

    pub fn get_second_order_velocity_brake_trajectory(&mut self) {
        self.t[0] = 0.0;
        self.t[1] = 0.0;
        self.j[0] = 0.0;
        self.j[1] = 0.0;
    }

    pub fn finalize(&mut self, ps: &mut f64, vs: &mut f64, as_: &mut f64) {
        if self.t[0] <= 0.0 && self.t[1] <= 0.0 {
            self.duration = 0.0;
            return;
        }

        self.duration = self.t[0];
        self.p[0] = *ps;
        self.v[0] = *vs;
        self.a[0] = *as_;
        let result = integrate(self.t[0], *ps, *vs, *as_, self.j[0]);
        *ps = result.0;
        *vs = result.1;
        *as_ = result.2;

        if self.t[1] > 0.0 {
            self.duration += self.t[1];
            self.p[1] = *ps;
            self.v[1] = *vs;
            self.a[1] = *as_;
            let result = integrate(self.t[1], *ps, *vs, *as_, self.j[1]);
            *ps = result.0;
            *vs = result.1;
            *as_ = result.2;
        }
    }

    pub fn finalize_second_order(&mut self, ps: &mut f64, vs: &mut f64, as_: &mut f64) {
        if self.t[0] <= 0.0 {
            self.duration = 0.0;
            return;
        }

        self.duration = self.t[0];
        self.p[0] = *ps;
        self.v[0] = *vs;
        let result = integrate(self.t[0], *ps, *vs, self.a[0], 0.0);
        *ps = result.0;
        *vs = result.1;
        *as_ = result.2;
    }
}
