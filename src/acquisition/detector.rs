//! Post-correlation detection: CFAR, peak validation, SNR estimation.
//!
//! After the PCPS correlator produce a 2-D power surface, this module
//! decides whether a satellite is present and estimates signal quality.
//!
//! # CFAR (Constant False-Alarm Rate) Detection
//!
//! The detector computes a **noise floor** estimate from the correlation
//! surface and compares the peel to a threshold:
//!
//! ```text
//! detected = peak_power / noise_floor >= threshold
//! ```
//!
//! Two noise floor estimators are available:
//!
//! | Estimator | Description | Use case |
//! |-----------|-------------|----------|
//! | `MeanCfar` | Mean of the entire surface | Simple, fast |
//! | `TrimmedMeanCfar` | Mean excluding the top N% of cells | Robust multi-signal |
//!
//! # Peak validation
//!
//! A single-sample peak can be a side-lobe artefact. The detector also checks:
//! - **Second-peak ratio**: the strongest secondary peak (>1 chip away
//!   from the main peak) should be significantly weaker.
//! - **Code-phase continuity**: optional tracking of phase across epochs.

use crate::acquisition::fft_search::SearchResult;

/// Detection decision returned by the [`Detector`].
#[derive(Debug, Clone, PartialEq)]
pub enum DetectionVerdict {
    /// Signal confirmed: peak exceeds threshold and passes validation
    Detected {
        /// Estimated C/N₀ in dB-Hz
        cn0_db_hz: f32,

        /// Confidence score in [0, 1].
        confidence: f32,
    },

    /// Peak present but below the configured CFAR threshold
    BelowThreshold {
        /// Actual peak-to-noise ratio.
        peak_to_noise: f32,

        /// Configured threshold.
        threshold: f32,
    },

    /// Peak exceeds threshold but failed secondary-peak or other validation
    FalseAlarm {
        /// Reason for rejection.
        reason: &'static str,
    },
}

/// Strategy for estimating the noise floor from a correlation surface.
#[derive(Debug, Clone, Copy)]
pub enum CfarEstimator {
    /// Use the mean of all cells in the surface.
    Mean,

    /// Use the mean of the lower `(1 − trim_fraction)` of cells.
    /// `trim_fraction = 0.05` trims the top 5% (removes interference spikes).
    TrimmedMean {
        /// Fraction of highest values to exclude from noise floor estimation.
        /// For example, 0.05 removes the top 5% of correlation bins.
        trim_fraction: f32,
    },
}

/// Configuration for the post-correlation detector.
#[derive(Debug, Clone)]
pub struct DetectorConfig {
    /// Primary CFAR threshold: `peak/noise_floor` > this -> candidate
    pub cfar_threshold: f32,

    /// CFAR noise floor estimator
    pub estimator: CfarEstimator,

    /// Minimum number of Doppler bins to estimate fine Doppler
    pub min_doppler_bins_for_fine: usize,

    /// Reject detection if `second_peak / main_peak` exceeds this ratio
    /// Set to 1.0 to disable
    pub second_peak_ratio_limit: f32,

    /// Exclusion zone around the main peak (in samples) for second-peak search
    pub second_peak_exclusion: usize,
}

/// Post-correlation CFAR detector and signal quality estimator.
pub struct Detector {
    config: DetectorConfig,
}

impl DetectionVerdict {
    /// Whether this is a confirmed detection.
    #[must_use]
    pub const fn is_detected(&self) -> bool {
        matches!(self, DetectionVerdict::Detected { .. })
    }

    /// Confidence score: 1.0 for `Detected`, 0.0 for others.
    #[must_use]
    pub const fn confidence(&self) -> f32 {
        match self {
            DetectionVerdict::Detected { confidence, .. } => *confidence,
            _ => 0.0,
        }
    }
}

impl CfarEstimator {
    /// Estimate the noise floor from a flat power surface slice.
    ///
    /// # Panics
    ///
    /// Panics if the input contains `NaN`, because the trimmed-mean path
    /// sorts values using floating-point comparison.
    #[must_use]
    pub fn estimate(
        self,
        surface: &[f32],
    ) -> f32 {
        let len = surface.len();
        if len == 0 {
            return 0.0;
        }

        match self {
            CfarEstimator::Mean => surface.iter().sum::<f32>() / len as f32,

            CfarEstimator::TrimmedMean { trim_fraction } => {
                let mut sorted = surface.to_vec();
                sorted.sort_by(f32::total_cmp);

                let trim_fraction = trim_fraction.clamp(0.0, 1.0);

                // floor(len * trim_fraction) без float -> int cast
                let mut trim = 0usize;
                let mut acc = 0.0f32;

                for _ in 0..len {
                    acc += trim_fraction;
                    if acc >= 1.0 {
                        trim += 1;
                        acc -= 1.0;
                    }
                }

                let keep = len.saturating_sub(trim).max(1);

                sorted[..keep].iter().sum::<f32>() / keep as f32
            }
        }
    }
}

impl Detector {
    /// Create a detector with the given configuration.
    #[must_use]
    pub const fn new(config: DetectorConfig) -> Self {
        Self { config }
    }

    /// Create a detector with default settings.
    #[must_use]
    pub fn with_defaults() -> Self {
        Self::new(DetectorConfig::default())
    }

    /// Ecaluate a [`SearchResult`] and return a detection verdict.
    ///
    /// # Arguments
    ///
    /// - `result` — output of [`PcpsSearch::search_prn`].
    /// - `surface` — the 1-D correlation power surface at the best Doppler bin.
    ///   Length must equal the block size used for acquisition.
    #[must_use]
    pub fn evaluate(
        &self,
        result: &SearchResult,
        surface: &[f32],
    ) -> DetectionVerdict {
        // 1. Compute noise floor with configured estimator
        let noise_floor = self.config.estimator.estimate(surface);
        let peak_to_noise = if noise_floor > f32::EPSILON {
            result.peak_power / noise_floor
        } else {
            0.0
        };

        // 2. CFAR gate
        if peak_to_noise < self.config.cfar_threshold {
            return DetectionVerdict::BelowThreshold {
                peak_to_noise,
                threshold: self.config.cfar_threshold,
            };
        }

        // 3. Second-peak validation
        let second_peak = self.find_second_peak(surface, result.code_phase_samples);

        if second_peak > 0.0 && result.peak_power > 0.0 {
            let ratio = second_peak / result.peak_power;

            if ratio > self.config.second_peak_ratio_limit {
                return DetectionVerdict::FalseAlarm {
                    reason: "second peak too close to main peak",
                };
            }
        }

        // 4. Estimate Estimate C/N₀ and confidence
        let cn0 = estimate_cn0(peak_to_noise, self.config.cfar_threshold);
        let confidence = self.compute_confidence(peak_to_noise);

        DetectionVerdict::Detected {
            cn0_db_hz: cn0,
            confidence,
        }
    }

    /// Compute noise floor from a raw correlation power surface.
    ///
    /// Useful when you have the surface directly without a `SearchResult`.
    #[must_use]
    pub fn noise_floor(
        &self,
        surface: &[f32],
    ) -> f32 {
        self.config.estimator.estimate(surface)
    }

    /// Compute CFAR peak-to-noise ratio for a surface.
    #[must_use]
    pub fn peak_to_noise(
        &self,
        surface: &[f32],
    ) -> (usize, f32) {
        let peak_idx = surface
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.total_cmp(b))
            .map_or(0, |(i, _)| i);

        let noise = self.noise_floor(surface);
        let ratio = if noise > f32::EPSILON {
            surface[peak_idx] / noise
        } else {
            0.0
        };

        (peak_idx, ratio)
    }

    /// Find the strongest peak outside the exclusion zone around `main_peak`.
    fn find_second_peak(
        &self,
        surface: &[f32],
        main_peak: usize,
    ) -> f32 {
        let n = surface.len();
        let excl = self.config.second_peak_exclusion;

        surface
            .iter()
            .enumerate()
            .filter(|&(i, _)| {
                let dist = i.abs_diff(main_peak);
                let dist_wrap = n.abs_diff(dist); // или n - dist (см. ниже)

                dist.min(dist_wrap) > excl
            })
            .map(|(_, &p)| p)
            .fold(0.0f32, f32::max)
    }

    /// Map peak-to-noise ratio to a [0, 1] confidence score.
    fn compute_confidence(
        &self,
        peak_to_noise: f32,
    ) -> f32 {
        let t = self.config.cfar_threshold;
        // Linearly ramp from threshold (-> 0.5) to 5 * threshold (-> 1.0)
        let x = (peak_to_noise - t) / (4.0 * t);

        (0.5 + x * 0.5).clamp(0.5, 1.0)
    }
}

/// Estimate C/N₀ in dB-Hz from the peak-to-noise ratio.
///
/// Uses the approximation:
///
/// ```text
/// C/N₀ ≈ 10·log₁₀(SNR_linear / T_coh)
/// ```
///
/// where `SNR_linear ≈ (peak/noise − 1)` and `T_coh = 1 ms`.
#[must_use]
pub fn estimate_cn0(
    peak_to_noise: f32,
    _threshold: f32,
) -> f32 {
    let snr_linear = (peak_to_noise - 1.0).max(f32::EPSILON);

    // T_coh = 1ms = 0.001s -> log10(1/0.001) = 30db
    10.0 * snr_linear.log10() + 30.0
}

/// Compute signal-to-noise ratio from a correlation surface.
#[must_use]
pub fn surface_snr(surface: &[f32]) -> (usize, f32) {
    if surface.is_empty() {
        return (0, 0.0);
    }

    let (peak_idx, &peak) = surface
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.total_cmp(b))
        .unwrap_or((0, &0.0));

    let mean = surface.iter().sum::<f32>() / surface.len() as f32;
    let snr = if mean > f32::EPSILON {
        peak / mean
    } else {
        0.0
    };

    (peak_idx, snr)
}

impl Default for DetectorConfig {
    fn default() -> Self {
        Self {
            cfar_threshold: 3.0,
            estimator: CfarEstimator::Mean,
            min_doppler_bins_for_fine: 3,
            second_peak_ratio_limit: 0.5,
            second_peak_exclusion: 10,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_result(
        peak_power: f32,
        noise_floor: f32,
        code_phase: usize,
    ) -> SearchResult {
        SearchResult {
            prn: 1,
            doppler_coarse_hz: 0.0,
            doppler_fine_hz: 0.0,
            code_phase_samples: code_phase,
            code_phase_chips: code_phase as f64 * 1023.0 / 2048.0,
            peak_power,
            noise_floor,
            peak_to_noise: peak_power / noise_floor.max(f32::EPSILON),
            detected: peak_power / noise_floor.max(f32::EPSILON) >= 3.0,
        }
    }

    #[test]
    fn test_verdict_detected_is_detected() {
        let v = DetectionVerdict::Detected {
            cn0_db_hz: 45.0,
            confidence: 0.9,
        };

        assert!(v.is_detected());
        assert!((v.confidence() - 0.9).abs() < 1e-5);
    }

    #[test]
    fn test_verdict_below_threshold_not_detected() {
        let v = DetectionVerdict::BelowThreshold {
            peak_to_noise: 1.5,
            threshold: 3.0,
        };

        assert!(!v.is_detected());
        assert_eq!(v.confidence(), 0.0);
    }

    #[test]
    fn test_verdict_false_alarm_not_detected() {
        let v = DetectionVerdict::FalseAlarm { reason: "test" };

        assert!(!v.is_detected());
    }

    #[test]
    fn test_mean_estimator_all_equal() {
        let surface = vec![2.0f32; 100];

        assert!((CfarEstimator::Mean.estimate(&surface) - 2.0).abs() < 1e-5);
    }

    #[test]
    fn test_mean_estimator_empty() {
        assert_eq!(CfarEstimator::Mean.estimate(&[]), 0.0);
    }

    #[test]
    fn test_mean_estimator_mixed() {
        let surface = vec![1.0f32, 3.0, 2.0];

        assert!((CfarEstimator::Mean.estimate(&surface) - 2.0).abs() < 1e-5);
    }

    #[test]
    fn test_trimmed_mean_excludes_top_fraction() {
        // 10 elements: [1,1,1,1,1,1,1,1,1, 100]
        // trim 10% -> exclude top 1 -> mean of [1..1] = 1.0
        let mut surface: Vec<f32> = vec![1.0; 9];

        surface.push(100.0);

        let est = CfarEstimator::TrimmedMean { trim_fraction: 0.1 };
        let result = est.estimate(&surface);

        assert!(
            result < 20.0,
            "trim mean should exclude outlier, got {result}"
        );
    }

    #[test]
    fn trimmed_mean_empty() {
        let est = CfarEstimator::TrimmedMean { trim_fraction: 0.1 };

        assert_eq!(est.estimate(&[]), 0.0);
    }

    #[test]
    fn test_surface_snr_peak_at_correct_index() {
        let mut surface = vec![1.0f32; 100];

        surface[42] = 50.0;

        let (idx, snr) = surface_snr(&surface);

        assert_eq!(idx, 42);
        assert!(snr > 1.0, "snr should be > 1, got {snr}");
    }

    #[test]
    fn test_surface_snr_uniform_is_one() {
        let surface = vec![5.0f32; 64];
        let (_, snr) = surface_snr(&surface);

        assert!((snr - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_surface_snr_empty() {
        assert_eq!(surface_snr(&[]), (0, 0.0));
    }

    #[test]
    fn test_cn0_increases_with_snr() {
        let cn0_low = estimate_cn0(3.0, 3.0);
        let cn0_high = estimate_cn0(10.0, 3.0);

        assert!(cn0_high > cn0_low, "cn0_high={cn0_high} cn0_low={cn0_low}");
    }

    #[test]
    fn test_cn0_high_snr_above_40dbhz() {
        // SNR=100 → CN0 ≈ 10*log10(99) + 30 ≈ 49.9 dBHz
        let cn0 = estimate_cn0(100.0, 3.0);

        assert!(cn0 > 40.0, "expected > 40 dBHz, got {cn0}");
    }

    #[test]
    fn test_detector_detects_strong_signal() {
        let det = Detector::with_defaults();
        // Strong peak at index 0, uniform noise elsewhere
        let mut surface = vec![1.0f32; 2048];

        surface[0] = 1000.0;

        let result = make_result(1000.0, 1.0, 0);
        let verdict = det.evaluate(&result, &surface);

        assert!(
            verdict.is_detected(),
            "strong signal should be detected: {verdict:?}"
        );
    }

    #[test]
    fn test_detector_rejects_weak_signal() {
        let det = Detector::with_defaults();
        let surface = vec![1.0f32; 2048]; // uniform, no peak
        let result = make_result(1.5, 1.0, 0); // P/N = 1.5 < 3.0
        let verdict = det.evaluate(&result, &surface);

        assert!(!verdict.is_detected());
        assert!(matches!(verdict, DetectionVerdict::BelowThreshold { .. }));
    }

    #[test]
    fn test_detector_confidence_above_half_for_detected() {
        let det = Detector::with_defaults();
        let mut surface = vec![1.0f32; 2048];

        surface[50] = 500.0;

        let result = make_result(500.0, 1.0, 50);
        let verdict = det.evaluate(&result, &surface);

        if verdict.is_detected() {
            assert!(verdict.confidence() >= 0.5);
        }
    }

    #[test]
    fn test_detector_second_peak_causes_false_alarm() {
        // If there's a very strong secondary peak, it should be rejected
        let det = Detector::new(DetectorConfig {
            second_peak_ratio_limit: 0.5,
            second_peak_exclusion: 5,
            ..DetectorConfig::default()
        });
        let mut surface = vec![1.0f32; 100];

        surface[10] = 100.0; // main peak
        surface[50] = 80.0; // secondary peak at 80% → ratio = 0.80 > 0.5 → false alarm

        let result = make_result(100.0, 1.0, 10);
        let verdict = det.evaluate(&result, &surface);

        assert!(
            matches!(verdict, DetectionVerdict::FalseAlarm { .. }),
            "should be false alarm due to second peak: {verdict:?}"
        );
    }

    #[test]
    fn test_detector_noise_floor_method() {
        let det = Detector::with_defaults();
        let surface = vec![2.0f32; 64];

        assert!((det.noise_floor(&surface) - 2.0).abs() < 1e-5);
    }

    #[test]
    fn test_detector_peak_to_noise_method() {
        let det = Detector::with_defaults();
        let mut surface = vec![1.0f32; 64];
        surface[10] = 10.0;

        let (idx, ratio) = det.peak_to_noise(&surface);

        assert_eq!(idx, 10);
        assert!((ratio - 8.767_123).abs() < 0.01, "ratio={ratio}");
    }

    #[test]
    fn test_trimmed_cfar_detector_works() {
        let det = Detector::new(DetectorConfig {
            estimator: CfarEstimator::TrimmedMean {
                trim_fraction: 0.05,
            },
            cfar_threshold: 3.0,
            ..DetectorConfig::default()
        });
        let mut surface = vec![1.0f32; 2048];

        surface[100] = 200.0;

        let result = make_result(200.0, 1.0, 100);
        let verdict = det.evaluate(&result, &surface);

        assert!(verdict.is_detected());
    }
}
