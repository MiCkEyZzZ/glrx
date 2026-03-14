use num_complex::Complex32;

#[inline]
pub fn compute_power(samples: &[Complex32]) -> f32 {
    if samples.is_empty() {
        return 0.0;
    }

    let total: f32 = samples.iter().map(|s| s.norm_sqr()).sum();

    total / samples.len() as f32
}
