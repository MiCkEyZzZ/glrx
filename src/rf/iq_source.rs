//! Унифицированный интерфейс источников IQ-данных.
//!
//! Этот модуль определяет:
//! - трейты для потоков IQ-сэмплов (`IqSource`)
//! - контейнер блока данных (`IqBlock`)
//!
//! Используется как абстракция над SDR-устройствами, файлами и потоками.

use num_complex::Complex32;

use crate::rf::{
    config::RfConfig,
    error::{RfError, RfResult},
    metrics::SourceMetrics,
};

/// Unified interface for any IQ sample source.
///
/// Implementations must:
/// * Return samples normalised to approximately ±1.0 regardless of wire format.
/// * Be thread-safe (`Send + Sync`) so the pipeline can move the source across
///   threads.
/// * Report accurate metrics via [`IqSource::metrics`].
pub trait IqSource: Send + Sync {
    /// Return the configuration of this source
    fn config(&self) -> &RfConfig;

    /// Read the next block of `n` samples.
    ///
    /// May return fewer than `n` samples near the end of a file.
    /// Returns `Err(RfError::EndOfFile)` when no more samples are available.
    fn read_block(
        &mut self,
        n: usize,
    ) -> RfResult<IqBlock>;

    /// Seek to a sample offset (optional; file sources support this).
    fn seek(
        &mut self,
        _sample_offset: u64,
    ) -> RfResult<()> {
        Err(RfError::Sdr("этот источник не поддерживает seek".into()))
    }

    /// Return a snapshot of current metrics.
    fn metrics(&self) -> SourceMetrics;

    /// Human-readable name for logging.
    fn name(&self) -> &str;
}

/// A block of IQ samples with the configuration that produced them.
#[derive(Debug, Clone)]
pub struct IqBlock {
    /// Complex baseband samples, normalised to roughly ±1.0
    pub samples: Vec<Complex32>,

    /// Config that was active when this block was captured
    pub config: RfConfig,

    /// Sample index of the first sample in this block (monotonically
    /// increasing)
    pub start_sample: u64,
}

impl IqBlock {
    /// Duration of this block in seconds
    #[must_use]
    pub fn duration_s(&self) -> f64 {
        self.samples.len() as f64 / self.config.sample_rate_hz
    }
}
