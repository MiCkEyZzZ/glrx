use std::sync::Arc;

use num_complex::Complex32;

use crate::rf::{
    config::RfConfig,
    error::{RfError, RfResult},
    metrics::SourceMetrics,
};

/// Унифицированный интерфейс для любого источника IQ-сэмплов.
pub trait IqSource: Send + Sync {
    /// Возвращает конфигурацию этого источника.
    fn config(&self) -> &RfConfig;

    /// Чтение следующего блока из `n` сэмплов.
    fn read_block(
        &mut self,
        n: usize,
    ) -> RfResult<IqBlock>;

    /// Перейти к указанному смещению сэмпла (опционально; поддерживается,
    /// например, файловыми источниками).
    fn seek(
        &mut self,
        _sample_offset: u64,
    ) -> RfResult<()> {
        Err(RfError::Sdr("этот источник не поддерживает seek".into()))
    }

    /// Вернуть снимок текущих метрик.
    fn metrics(&self) -> SourceMetrics;

    /// Читаемое имя источника для логирования.
    fn name(&self) -> &str;
}

#[derive(Debug, Clone)]
pub struct IqBlock {
    /// Комплексные базовые сэмплы, нормализованные примерно в диапазон +/- 1.0.
    pub samples: Vec<Complex32>,

    /// Конфигурация, действовавшая при захвате этого блока.
    pub config: Arc<RfConfig>,

    /// Индекс сэмпла первого сэмпла в этом блоке (монотонно увеличивается).
    pub start_sample: u64,
}

impl IqBlock {
    /// Длительность блока в секундах.
    pub fn duration_s(&self) -> f64 {
        self.samples.len() as f64 / self.config.sample_rate_hz
    }
}
