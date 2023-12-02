pub fn join<T: std::fmt::Display>(array: &[T]) -> String {
    array
        .iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join(", ")
}

#[inline]
pub fn integrate(t: f64, p0: f64, v0: f64, a0: f64, j: f64) -> (f64, f64, f64) {
    (
        p0 + t * (v0 + t * (a0 / 2.0 + t * j / 6.0)),
        v0 + t * (a0 + t * j / 2.0),
        a0 + t * j,
    )
}
