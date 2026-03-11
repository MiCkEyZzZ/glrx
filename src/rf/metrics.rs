/// Runtime metrics reported by an [`IqSource`].
#[derive(Debug, Clone, Default)]
pub struct SourceMetrics {
    pub total_samples: u64,
    pub dropped_samples: u64,
    pub interruptions: u64,
    pub measured_rate_hz: Option<f64>,
    pub power_dbfs: Option<f32>,
}
