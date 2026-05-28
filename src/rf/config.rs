//! Конфигурация RF-тракта для источников IQ и DSP-пайплайна.

use std::time::Duration;

use crate::rf::{
    error::{RfError, RfResult},
    format::SampleFormat,
};

/// Configuration shared by all IQ sources.
#[derive(Debug, Clone)]
pub struct RfConfig {
    /// Centre frequency in Hz (e.g. `1_575_420_000` for GPS L1)
    pub center_freq_hz: f64,

    /// Sample rate in samples/second (e.g. `2_048_000` for 2.048 MHz)
    pub sample_rate_hz: f64,

    /// Optional gain in dB. `None` means use the source default / AGC
    pub gain_db: Option<f64>,

    /// Wire format of incoming samples
    pub format: SampleFormat,
}

impl RfConfig {
    /// Nyquist bandwidth in Hz
    #[must_use]
    pub fn bandwidth_hz(&self) -> f64 {
        self.sample_rate_hz / 2.0
    }

    /// Duration of a single sample in seconds
    #[must_use]
    pub fn sample_period_s(&self) -> f64 {
        1.0 / self.sample_rate_hz
    }

    /// Number of samples in `duration`.
    #[must_use]
    pub fn samples_in(
        &self,
        duration: Duration,
    ) -> usize {
        (self.sample_rate_hz * duration.as_secs_f64()) as usize
    }

    /// Validate configuration fields
    pub fn validate(&self) -> RfResult<()> {
        if self.sample_rate_hz <= 0.0 {
            return Err(RfError::Config(format!(
                "sample_rate_hz must be positive, got {}",
                self.sample_rate_hz
            )));
        }

        if self.center_freq_hz <= 0.0 {
            return Err(RfError::Config(format!(
                "center_freq_hz must be positive, got {}",
                self.center_freq_hz
            )));
        }

        Ok(())
    }
}

impl Default for RfConfig {
    /// GPS L1 C/A defaults: 1575.42 MHz, 2.048 Msps, I8.
    fn default() -> Self {
        Self {
            center_freq_hz: 1_575_420_000.0,
            sample_rate_hz: 2_048_000.0,
            gain_db: None,
            format: SampleFormat::I8,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg_with_rate(rate: f64) -> RfConfig {
        RfConfig {
            sample_rate_hz: rate,
            ..Default::default()
        }
    }

    #[test]
    fn test_rf_config_default_are_valid() {
        RfConfig::default()
            .validate()
            .expect("default config should be valid");
    }

    #[test]
    fn test_rf_config_samples_in_duration() {
        let cfg = RfConfig::default();
        let n = cfg.samples_in(Duration::from_millis(1));

        assert_eq!(n, 2048);
    }

    #[test]
    fn test_rf_config_validate_errors() {
        let cfg = cfg_with_rate(0.0);

        assert!(cfg.validate().is_err());

        let cfg = cfg_with_rate(0.0);

        assert!(cfg.validate().is_err());
    }

    #[test]
    fn test_bandwidth_and_period() {
        let cfg = RfConfig::default();

        assert_eq!(cfg.bandwidth_hz(), 1_024_000.0);
        assert!((cfg.sample_period_s() - 1.0 / 2_048_000.0).abs() < 1e-12);
    }
}
