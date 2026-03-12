use num_complex::Complex32;

use crate::rf::{
    config::RfConfig,
    error::{RfError, RfResult},
    metrics::SourceMetrics,
};

/// Unified interface for any IQ sample source.
pub trait IqSource: Send + Sync {
    /// Return the configuration of this source.
    fn config(&self) -> &RfConfig;

    /// Read the next block of `n` samples.
    fn read_block(&mut self, n: usize) -> RfResult<IqBlock>;

    /// Seek to a sample offset (optional; file sources support this).
    fn seek(&mut self, _sample_offset: u64) -> RfResult<()> {
        Err(RfError::Sdr("seek not supported by this source".into()))
    }

    /// Return a snapshot of current metrics.
    fn metrics(&self) -> SourceMetrics;

    /// Human-readable name for logging.
    fn name(&self) -> &str;
}

#[derive(Debug, Clone)]
pub struct IqBlock {
    /// Complex baseband samples, normalised to roughly +/- 1.0.
    pub samples: Vec<Complex32>,

    /// Config that was active when this block was captured.
    pub config: RfConfig,

    /// Sample index of the first sample in this block (monotonically increasing).
    pub start_sample: u64,
}

impl IqBlock {
    pub fn duration_s(&self) -> f64 {
        self.samples.len() as f64 / self.config.sample_rate_hz
    }
}
