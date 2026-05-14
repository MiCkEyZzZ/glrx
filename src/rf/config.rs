//! Конфигурация RF-тракта для источников IQ и DSP-пайплайна.

use std::time::Duration;

use crate::rf::{
    error::{RfError, RfResult},
    format::SampleFormat,
};

/// Конфигурация RF-фронтенда для захвата и воспроизведения IQ-данных.
///
/// Определяет ключевые параметры обработки сигнала: частоту, дискретизацию,
/// усиление и формат входных отсчётов.
#[derive(Debug, Clone)]
pub struct RfConfig {
    /// Центральная частота в Гц (например, `1_575_420_000.0` для GPS L1).
    pub center_freq_hz: f64,

    /// Частота дискретизации в отсчётах в секунду (например, `2_048_000.0` =
    /// 2.048 `МГц`).
    pub sample_rate_hz: f64,

    /// Усиление в дБ.
    ///
    /// `None` означает использование автоматической регулировки усиления (AGC)
    /// или значения по умолчанию устройства.
    pub gain_db: Option<f64>,

    /// Формат входных IQ-отсчётов.
    pub format: SampleFormat,
}

impl RfConfig {
    /// Оценка полосы пропускания (по Найквисту).
    #[must_use]
    pub fn bandwidth_hz(&self) -> f64 {
        self.sample_rate_hz / 2.0
    }

    /// Период одного отсчёта в секундах.
    #[must_use]
    pub fn sample_period_s(&self) -> f64 {
        1.0 / self.sample_rate_hz
    }

    /// Количество комплексных отсчётов в заданной длительности.
    #[must_use]
    pub fn samples_in(
        &self,
        duration: Duration,
    ) -> usize {
        (self.sample_rate_hz * duration.as_secs_f64()) as usize
    }

    /// Проверка корректности конфигурации.
    ///
    /// Возвращает ошибку, если параметры некорректны (например, нулевая
    /// частота).
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
