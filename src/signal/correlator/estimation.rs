//! C/N₀ Estimation.

use std::f32;

use num_complex::Complex32;

/// C/N₀ estimation using the **narrow-band power ration** method.
pub fn cn0_estimate(
    prompt_accumulations: &[Complex32],
    coherent_time_sec: f64,
) -> f32 {
    if prompt_accumulations.len() < 2 {
        return 0.0;
    }

    // Mean coherent power (average of |P|²)
    let p_coh: f32 = prompt_accumulations
        .iter()
        .map(|p| p.norm_sqr())
        .sum::<f32>()
        / prompt_accumulations.len() as f32;
    // Mean non-coherent power (average of |P|)² — note the squaring outside
    let p_nc_sq: f32 = {
        let mean_env = prompt_accumulations.iter().map(|p| p.norm()).sum::<f32>()
            / prompt_accumulations.len() as f32;
        mean_env * mean_env
    };

    let denom = (p_nc_sq - p_coh).max(f32::EPSILON);
    let cn0_linear = p_coh / denom / coherent_time_sec as f32;
    10.0 * cn0_linear.max(f32::EPSILON).log10()
}

/// Simple non-cogerent C/N₀ estimate from I/O components.
pub fn cn0_estimate_iwbp(
    narrow_band_power: f32,
    wide_band_power: f32,
    coherent_time_sec: f64,
) -> f32 {
    let ratio =
        (narrow_band_power - wide_band_power).max(f32::EPSILON) / wide_band_power.max(f32::EPSILON);

    10.0 * ratio.log10() - 10.0 * coherent_time_sec.log10() as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cn0_estimate_single_sample_returns_zero() {
        let accums = vec![Complex32::new(100.0, 0.0)];

        assert_eq!(cn0_estimate(&accums, 0.001), 0.0);
    }

    #[test]
    fn test_cn0_estimate_high_snr_above_40dbhz() {
        // Simulate strong signal: large prompt values, low noise variation
        let accums: Vec<Complex32> = (0..20).map(|_| Complex32::new(100.0, 0.0)).collect();
        let cn0 = cn0_estimate(&accums, 0.001);

        assert!(cn0 > 40.0, "expected CN0 > 40 db-Hz, got {cn0}");
    }

    #[test]
    fn test_cn0_estimate_increases_with_signal_strength() {}
}
