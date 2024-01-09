pub fn join(numbers: &[f64], high_precision: bool) -> String {
    if high_precision {
        numbers
            .iter()
            .map(|&num| format!("{:.16}", num))
            .collect::<Vec<String>>()
            .join(", ")
    } else {
        numbers
            .iter()
            .map(|&num| num.to_string())
            .collect::<Vec<String>>()
            .join(", ")
    }
}

#[inline]
pub fn integrate(t: f64, p0: f64, v0: f64, a0: f64, j: f64) -> (f64, f64, f64) {
    (
        p0 + t * (v0 + t * (a0 / 2.0 + t * j / 6.0)),
        v0 + t * (a0 + t * j / 2.0),
        a0 + t * j,
    )
}
