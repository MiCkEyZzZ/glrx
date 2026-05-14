//! Метрики работы источников IQ-данных.
//!
//! Содержит статистику потока, включая:
//! - количество переданных и потерянных сэмплов
//! - разрывы потока
//! - оценку скорости дискретизации
//! - оценку мощности сигнала
//!
//! Используется совместно с [`IqSource`].

/// Метрики выполнения, предоставляемые [`IqSource`].
#[derive(Debug, Clone, Default)]
pub struct SourceMetrics {
    /// Общее количество комплексных сэмплов, выданных с начала работы
    pub total_samples: u64,

    /// Количество потерянных сэмплов из-за переполнения буфера или недопоставки
    /// драйвера
    pub dropped_samples: u64,

    /// Количество обнаруженных прерываний потока (разрыв > 1 мс)
    pub interruptions: u64,

    /// Измеренная мгновенная скорость сэмплирования (Гц). `None`, если пока
    /// недоступна
    pub measured_rate_hz: Option<f64>,

    /// Оценка мощности сигнала в dBFS. `None`, если пока недоступна
    pub power_dbfs: Option<f32>,
}

impl SourceMetrics {
    /// Доля потерь сэмплов
    #[must_use]
    pub fn loss_ratio(&self) -> f64 {
        if self.total_samples == 0 {
            return 0.0;
        }

        self.dropped_samples as f64 / (self.total_samples + self.dropped_samples) as f64
    }
}

////////////////////////////////////////////////////////////////////////////////
// Тесты
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_metrics_loss_ratio() {
        let m = SourceMetrics {
            total_samples: 900,
            dropped_samples: 100,
            ..Default::default()
        };

        assert!((m.loss_ratio() - 0.1).abs() < 1e-9);
    }

    #[test]
    fn test_loss_ratio_zero_total() {
        let m = SourceMetrics::default();
        assert_eq!(m.loss_ratio(), 0.0);
    }

    #[test]
    fn test_loss_ratio_no_drops() {
        let m = SourceMetrics {
            total_samples: 1000,
            dropped_samples: 0,
            ..Default::default()
        };
        assert_eq!(m.loss_ratio(), 0.0);
    }

    #[test]
    fn test_loss_ratio_drops_more_than_total() {
        let m = SourceMetrics {
            total_samples: 50,
            dropped_samples: 150,
            ..Default::default()
        };
        assert!((m.loss_ratio() - 0.75).abs() < 1e-9);
    }
}
