//! Root-finding utilities for trajectory calculations
//!
//! This module provides functions for finding roots of polynomials,
//! which is essential for solving the trajectory optimization problems.

use arrayvec::ArrayVec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

const COS_120: f64 = -0.50;
const SIN_120: f64 = 0.866_025_403_784_438_6;
/// Convergence tolerance of the polynomial root refinement in [`shrink_interval`].
pub const TOLERANCE: f64 = 1e-14;

/// Returns the square of `v`.
pub fn pow2<T: core::ops::Mul<Output = T> + Copy>(v: T) -> T {
    v * v
}

/// A fixed-capacity set of at most `N` unique values, stored on the stack.
#[derive(Debug)]
pub struct Set<T, const N: usize> {
    /// The stored values in insertion order.
    pub data: ArrayVec<T, N>,
}

impl<T: Copy + Default + PartialEq, const N: usize> Default for Set<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Copy + Default + PartialEq, const N: usize> Set<T, N> {
    /// Creates an empty set.
    pub fn new() -> Self {
        Set {
            data: ArrayVec::<T, N>::new(),
        }
    }

    /// Returns the stored values as a slice.
    pub fn begin(&self) -> &[T] {
        &self.data[..]
    }

    /// Inserts `value` unless an equal value is already stored.
    ///
    /// # Panics
    ///
    /// Panics if the set already holds `N` values.
    pub fn insert(&mut self, value: T) {
        if !self.data.contains(&value) {
            self.data.push(value);
        }
    }
}

impl<T, const N: usize> IntoIterator for Set<T, N> {
    type Item = T;
    type IntoIter = <ArrayVec<T, N> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        self.data.into_iter()
    }
}

impl<'a, T, const N: usize> IntoIterator for &'a Set<T, N> {
    type Item = &'a T;
    type IntoIter = core::slice::Iter<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        self.data.iter()
    }
}

impl<'a, T, const N: usize> IntoIterator for &'a mut Set<T, N> {
    type Item = &'a mut T;
    type IntoIter = core::slice::IterMut<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        self.data.iter_mut()
    }
}

/// A [`Set`] of `f64` that only accepts non-negative values, used to collect
/// the non-negative real roots of a polynomial.
///
/// Consuming iteration (and mutable iteration) yields the values in ascending order.
#[derive(Debug)]
pub struct PositiveSet<const N: usize>(pub Set<f64, N>);

impl<const N: usize> PositiveSet<N> {
    /// Creates an empty set.
    pub fn new() -> Self {
        PositiveSet(Set::new())
    }

    /// Inserts `value` if it is non-negative and not already stored.
    pub fn insert(&mut self, value: f64) {
        if value >= 0.0 {
            self.0.insert(value);
        }
    }

    /// Returns the stored values (in insertion order) as a slice.
    pub fn get_data(&self) -> &[f64] {
        self.0.begin()
    }
}

impl<const N: usize> IntoIterator for PositiveSet<N> {
    type Item = f64;
    type IntoIter = <ArrayVec<f64, N> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        let mut data = self.0.data;
        data.sort_unstable_by(f64::total_cmp);
        data.into_iter()
    }
}

impl<'a, const N: usize> IntoIterator for &'a PositiveSet<N> {
    type Item = &'a f64;
    type IntoIter = core::slice::Iter<'a, f64>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.data.iter()
    }
}

impl<'a, const N: usize> IntoIterator for &'a mut PositiveSet<N> {
    type Item = &'a mut f64;
    type IntoIter = core::slice::IterMut<'a, f64>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.data.sort_unstable_by(f64::total_cmp);
        self.0.data.iter_mut()
    }
}

impl<const N: usize> Default for PositiveSet<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Calculates the non-negative real roots of `a*x^3 + b*x^2 + c*x + d = 0`.
///
/// Degenerates to the quadratic or linear case for vanishing leading coefficients.
#[inline]
pub fn solve_cub(a: f64, b: f64, c: f64, d: f64) -> PositiveSet<3> {
    let mut roots = PositiveSet::new();

    // A vanishing constant term has the root x = 0; the remaining roots are
    // those of the deflated polynomial a*x^2 + b*x + c
    let (a, b, c, d) = if d.abs() < f64::EPSILON {
        roots.insert(0.0);
        (0.0, a, b, c)
    } else {
        (a, b, c, d)
    };

    if a.abs() < f64::EPSILON {
        if b.abs() < f64::EPSILON {
            // Linear equation
            if c.abs() > f64::EPSILON {
                roots.insert(-d / c);
            }
        } else {
            // Quadratic equation
            let discriminant = c * c - 4.0 * b * d;
            if discriminant >= 0.0 {
                let inv2b = 1.0 / (2.0 * b);
                let y = discriminant.sqrt();
                roots.insert((-c + y) * inv2b);
                roots.insert((-c - y) * inv2b);
            }
        }
    } else {
        // Cubic equation
        let inva = 1.0 / a;
        let invaa = inva * inva;
        let bb = b * b;
        let bover3a = b * inva / 3.0;
        let p = (a * c - bb / 3.0) * invaa;
        let halfq = (2.0 * bb * b - 9.0 * a * b * c + 27.0 * a * a * d) / 54.0 * invaa * inva;
        let yy = p * p * p / 27.0 + halfq * halfq;

        if yy > f64::EPSILON {
            // Sqrt is positive: one real solution
            let y = yy.sqrt();
            let uuu = -halfq + y;
            let vvv = -halfq - y;
            let www = if uuu.abs() > vvv.abs() { uuu } else { vvv };
            let w = www.cbrt();
            roots.insert(w - p / (3.0 * w) - bover3a);
        } else if yy < -f64::EPSILON {
            // Sqrt is negative: three real solutions
            let x = -halfq;
            let y = (-yy).sqrt();
            let mut theta;
            let mut r;

            // Convert to polar form
            if x.abs() > f64::EPSILON {
                theta = y.atan2(x);
                r = (x * x - yy).sqrt();
            } else {
                // Vertical line
                theta = core::f64::consts::PI / 2.0;
                r = y;
            }
            // Calculate cube root
            theta /= 3.0;
            r = 2.0 * r.cbrt();
            // Convert to complex coordinate
            let ux = theta.cos() * r;
            let uyi = theta.sin() * r;

            roots.insert(ux - bover3a);
            roots.insert(ux * COS_120 - uyi * SIN_120 - bover3a);
            roots.insert(ux * COS_120 + uyi * SIN_120 - bover3a);
        } else {
            // Sqrt is zero: two real solutions
            let www = -halfq;
            let w = 2.0 * www.cbrt();

            roots.insert(w - bover3a);
            roots.insert(w * COS_120 - bover3a);
        }
    }
    roots
}

/// Solves the resolvent cubic `x^3 + a*x^2 + b*x + c = 0` of a quartic equation.
///
/// The real roots are written to `x` and their number (1, 2 or 3) is returned.
/// If only one real root exists, `x[1]` and `x[2]` hold the real and imaginary
/// part of the complex pair.
#[inline]
pub fn solve_resolvent(x: &mut [f64; 3], a: f64, b: f64, c: f64) -> usize {
    let a = a / 3.0;
    let a2 = a * a;
    let mut q = a2 - b / 3.0;
    let r = (a * (2.0 * a2 - b) + c) / 2.0;
    let r2 = r * r;
    let q3 = q * q * q;

    if r2 < q3 {
        let q_sqrt = q.sqrt();
        // Preserve the original NaN fallback semantics of min/max.
        #[allow(clippy::manual_clamp)]
        let t = (r / (q * q_sqrt)).min(1.0).max(-1.0);
        q = -2.0 * q_sqrt;

        let theta = t.acos() / 3.0;
        let ux = theta.cos() * q;
        let uyi = (theta).sin() * q;
        x[0] = ux - a;
        x[1] = ux * COS_120 - uyi * SIN_120 - a;
        x[2] = ux * COS_120 + uyi * SIN_120 - a;
        3
    } else {
        let mut a_ = (-r.abs() - (r2 - q3).sqrt()).cbrt();
        if r < 0.0 {
            a_ = -a_;
        }
        let b_ = if a_ == 0.0 { 0.0 } else { q / a_ };

        x[0] = (a_ + b_) - a;
        x[1] = -(a_ + b_) / 2.0 - a;
        x[2] = 3.0_f64.sqrt() * (a_ - b_) / 2.0;
        if x[2].abs() < f64::EPSILON {
            x[2] = x[1];
            2
        } else {
            1
        }
    }
}

/// Calculates the non-negative real roots of the monic quartic equation
/// `x^4 + a*x^3 + b*x^2 + c*x + d = 0`.
///
/// Coefficients below the machine epsilon are treated as zero, which suits
/// roots of order one. See [`solve_quart_monic_scaled`] for small roots.
#[inline]
pub fn solve_quart_monic_coeffs(a: f64, b: f64, c: f64, d: f64) -> PositiveSet<4> {
    let mut roots = PositiveSet::new();

    let a_squared = a * a;
    let four_b = 4.0 * b;

    if d.abs() < f64::EPSILON {
        if c.abs() < f64::EPSILON {
            roots.insert(0.0);

            let d_ = a_squared - four_b;
            if d_.abs() < f64::EPSILON {
                roots.insert(-a / 2.0);
            } else if d_ > 0.0 {
                let sqrt_d = d_.sqrt();
                roots.insert((-a - sqrt_d) / 2.0);
                roots.insert((-a + sqrt_d) / 2.0);
            }
            return roots;
        }

        if a.abs() < f64::EPSILON && b.abs() < f64::EPSILON {
            roots.insert(0.0);
            roots.insert(-c.cbrt());
            return roots;
        }
    }

    let a3 = -b;
    let b3 = a * c - 4.0 * d;
    let c3 = -a_squared * d - c * c + four_b * d;

    let mut x3 = [0.0; 3];
    let number_zeroes = solve_resolvent(&mut x3, a3, b3, c3);

    let mut y = x3[0];
    // Choosing Y with maximal absolute value.
    if number_zeroes != 1 {
        if x3[1].abs() > y.abs() {
            y = x3[1];
        }
        if x3[2].abs() > y.abs() {
            y = x3[2];
        }
    }

    let q1: f64;
    let q2: f64;
    let p1: f64;
    let p2: f64;

    let mut d_ = y * y - 4.0 * d;
    if d_.abs() < f64::EPSILON {
        q2 = y / 2.0;
        q1 = q2;
        d_ = a_squared - 4.0 * (b - y);
        if d_.abs() < f64::EPSILON {
            p2 = a / 2.0;
            p1 = p2;
        } else {
            let sqrt_d = d_.sqrt();
            p1 = (a + sqrt_d) / 2.0;
            p2 = (a - sqrt_d) / 2.0;
        }
    } else {
        let sqrt_d = d_.sqrt();
        q1 = (y + sqrt_d) / 2.0;
        q2 = (y - sqrt_d) / 2.0;
        p1 = (a * q1 - c) / (q1 - q2);
        p2 = (c - a * q2) / (q1 - q2);
    }

    const EPS_M_BY_16: f64 = 16.0 * f64::EPSILON;
    d_ = p1 * p1 - 4.0 * q1;
    if d_.abs() < EPS_M_BY_16 {
        roots.insert(-p1 / 2.0);
    } else if d_ > 0.0 {
        let sqrt_d = d_.sqrt();
        roots.insert((-p1 - sqrt_d) / 2.0);
        roots.insert((-p1 + sqrt_d) / 2.0);
    }

    d_ = p2 * p2 - 4.0 * q2;
    if d_.abs() < EPS_M_BY_16 {
        roots.insert(-p2 / 2.0);
    } else if d_ > 0.0 {
        let sqrt_d = d_.sqrt();
        roots.insert((-p2 - sqrt_d) / 2.0);
        roots.insert((-p2 + sqrt_d) / 2.0);
    }

    roots
}

/// Calculates the non-negative real roots of the monic quartic
/// `x^4 + p[0]*x^3 + p[1]*x^2 + p[2]*x + p[3] = 0`.
///
/// See [`solve_quart_monic_coeffs`].
#[inline]
pub fn solve_quart_monic_arr(polynom: &[f64; 4]) -> PositiveSet<4> {
    solve_quart_monic_coeffs(polynom[0], polynom[1], polynom[2], polynom[3])
}

/// Calculates the non-negative real roots of the monic quartic equation
/// `x^4 + a*x^3 + b*x^2 + c*x + d = 0` with zero tests relative to the
/// magnitude of the roots.
///
/// [`solve_quart_monic_coeffs`] treats coefficients below the machine epsilon
/// as zero. The coefficients of a quartic with roots of the magnitude `s`
/// scale with up to `s^4` however, so that e.g. durations of some 1e-4 lead
/// to a constant term of 1e-16 that is far from negligible: dropping it loses
/// the root. This variant solves the quartic in the normalized variable
/// `x / s` instead.
pub fn solve_quart_monic_scaled(polynom: &[f64; 4]) -> PositiveSet<4> {
    let [a, b, c, d] = *polynom;

    // Upper bound of the root magnitudes up to a factor of two (Fujiwara)
    let magnitude = a
        .abs()
        .max(b.abs().sqrt())
        .max(c.abs().cbrt())
        .max(d.abs().sqrt().sqrt());
    if !(1e-60..1.0).contains(&magnitude) {
        return solve_quart_monic_coeffs(a, b, c, d);
    }

    // The power of two above the magnitude: scaling by it is exact
    let exponent = (magnitude.to_bits() >> 52) & 0x7ff;
    let s = f64::from_bits((exponent + 1) << 52);
    let s2 = s * s;

    let mut roots = solve_quart_monic_coeffs(a / s, b / s2, c / (s2 * s), d / (s2 * s2));
    for root in roots.0.data.iter_mut() {
        *root *= s;
    }
    roots
}

/// Returns the coefficients of the derivative of the polynomial `coeffs`.
///
/// Coefficients are ordered from the highest degree down to the constant term.
/// `coeffs` must not be empty.
#[inline]
pub fn poly_deri<const N: usize>(coeffs: &ArrayVec<f64, N>) -> ArrayVec<f64, N> {
    let mut deriv = ArrayVec::<f64, N>::new();
    let len = coeffs.len();
    for i in 0..len - 1 {
        deriv.push((len - 1 - i) as f64 * coeffs[i]);
    }
    deriv
}

/// Returns the derivative of the monic polynomial `monic_coeffs`, normalized
/// to be monic again (i.e. divided by the degree).
///
/// Coefficients are ordered from the highest degree down to the constant term.
#[inline]
pub fn poly_monic_deri<const N: usize>(monic_coeffs: &ArrayVec<f64, N>) -> ArrayVec<f64, N> {
    let mut deriv = ArrayVec::<f64, N>::new();
    let len = monic_coeffs.len();
    deriv.push(1.0);
    for i in 1..len - 1 {
        deriv.push((len - 1 - i) as f64 * monic_coeffs[i] / (len - 1) as f64);
    }
    deriv
}

/// Evaluates the polynomial `p` at `x`.
///
/// Coefficients are ordered from the highest degree down to the constant term.
#[inline]
pub fn poly_eval<const N: usize>(p: &ArrayVec<f64, N>, x: f64) -> f64 {
    let mut result = 0.0;
    let n = p.len();
    if x.abs() < f64::EPSILON {
        result = p[n - 1];
    } else if (x - 1.0).abs() < f64::EPSILON {
        result = p.iter().sum();
    } else {
        let mut xn = 1.0;
        for &coeff in p.iter().rev() {
            result += coeff * xn;
            xn *= x;
        }
    }
    result
}

/// [`shrink_interval`] with the default limit of 128 iterations.
#[inline]
pub fn shrink_interval_default<const N: usize>(p: &ArrayVec<f64, N>, l: f64, h: f64) -> f64 {
    shrink_interval::<N, 128>(p, l, h)
}

/// Refines a root of the polynomial `p` within the bracket `[l, h]` using a
/// safeguarded Newton–Raphson method (falling back to bisection), with at most
/// `MAX_ITS` iterations.
///
/// The polynomial values at `l` and `h` should have opposite signs.
#[inline]
pub fn shrink_interval<const N: usize, const MAX_ITS: usize>(
    p: &ArrayVec<f64, N>,
    mut l: f64,
    mut h: f64,
) -> f64 {
    let deriv = poly_deri(p);
    let fl = poly_eval(p, l);
    let fh = poly_eval(p, h);
    if fl == 0.0 {
        return l;
    }
    if fh == 0.0 {
        return h;
    }
    if fl > 0.0 {
        core::mem::swap(&mut l, &mut h);
    }

    let mut rts = (l + h) / 2.0;
    let mut dxold = (h - l).abs();
    let mut dx = dxold;
    let mut f = poly_eval(p, rts);
    let mut df = poly_eval(&deriv, rts);
    for _ in 0..MAX_ITS {
        if (((rts - h) * df - f) * ((rts - l) * df - f) > 0.0)
            || (2.0 * f).abs() > dxold.abs() * df.abs()
        {
            dxold = dx;
            dx = (h - l) / 2.0;
            rts = l + dx;
            if (l - rts).abs() < f64::EPSILON {
                break;
            }
        } else {
            dxold = dx;
            dx = f / df;
            let temp = rts;
            rts -= dx;
            if (temp - rts).abs() < f64::EPSILON {
                break;
            }
        }

        if dx.abs() < TOLERANCE {
            break;
        }

        f = poly_eval(p, rts);
        df = poly_eval(&deriv, rts);
        if f < 0.0 {
            l = rts;
        } else {
            h = rts;
        }
    }

    rts
}
