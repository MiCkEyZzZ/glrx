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
        ///
        trim_fraction: f32,
    },
}

impl DetectionVerdict {
    /// Whether this is a confirmed detection.
    pub fn is_detected(&self) -> bool {
        matches!(self, DetectionVerdict::Detected { .. })
    }

    /// Confidence score: 1.0 for `Detected`, 0.0 for others.
    pub fn confidence(&self) -> f32 {
        match self {
            DetectionVerdict::Detected { confidence, .. } => *confidence,
            _ => 0.0,
        }
    }
}

impl CfarEstimator {
    /// Estimate the noise floor from a flat power surface slice.
    pub fn estimate(
        &self,
        surface: &[f32],
    ) -> f32 {
        if surface.is_empty() {
            return 0.0;
        }

        match self {
            CfarEstimator::Mean => surface.iter().sum::<f32>() / surface.len() as f32,
            CfarEstimator::TrimmedMean { trim_fraction } => {
                let mut sorted = surface.to_vec();

                sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());

                let keep = ((1.0 - trim_fraction) * sorted.len() as f32) as usize;
                let keep = keep.max(1);

                sorted[..keep].iter().sum::<f32>() / keep as f32
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
