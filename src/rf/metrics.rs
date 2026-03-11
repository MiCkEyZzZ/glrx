/// Runtime metrics reported by an [`IqSource`].
#[derive(Debug, Clone, Default)]
pub struct SourceMetrics {
    /// Total number of complex samples delivered since start
    pub total_samples: u64,

    /// Number of samples lost due to buffer overflow or driver underrun
    pub dropped_samples: u64,

    /// Number of stream interruptions detected (gap > 1 ms).
    pub interruptions: u64,

    /// Instantaneous measured sample rate (Hz). `None` if not yet available.
    pub measured_rate_hz: Option<f64>,

    /// Signal power estimate in dBFS. `None` if not yet available.
    pub power_dbfs: Option<f32>,
}

impl SourceMetrics {
    pub fn loss_ratio(&self) -> f64 {
        if self.total_samples == 0 {
            return 0.0;
        }

        self.dropped_samples as f64 / (self.total_samples + self.dropped_samples) as f64
    }
}

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
}
