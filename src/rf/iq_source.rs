//! Унифицированный интерфейс источников IQ-данных.
//!
//! Этот модуль определяет:
//! - трейты для потоков IQ-сэмплов (`IqSource`)
//! - контейнер блока данных (`IqBlock`)
//!
//! Используется как абстракция над SDR-устройствами, файлами и потоками.

use std::sync::Arc;

use num_complex::Complex32;

use crate::rf::{
    config::RfConfig,
    error::{RfError, RfResult},
    metrics::SourceMetrics,
};

/// Унифицированный интерфейс для любого источника IQ-сэмплов.
///
/// Источник может быть файловым, потоковым или аппаратным SDR-устройством.
pub trait IqSource: Send + Sync {
    /// Возвращает конфигурацию источника.
    fn config(&self) -> &RfConfig;

    /// Читает следующий блок из `n` комплексных сэмплов.
    fn read_block(
        &mut self,
        n: usize,
    ) -> RfResult<IqBlock>;

    /// Переходит к указанному смещению в сэмплах.
    ///
    /// По умолчанию источники могут не поддерживать эту операцию.
    fn seek(
        &mut self,
        _sample_offset: u64,
    ) -> RfResult<()> {
        Err(RfError::Sdr("этот источник не поддерживает seek".into()))
    }

    /// Возвращает текущий снимок метрик источника.
    fn metrics(&self) -> SourceMetrics;

    /// Возвращает человекочитаемое имя источника для логирования.
    fn name(&self) -> &str;
}

/// Блок IQ-данных, полученный от источника.
///
/// Содержит комплексные отсчёты, конфигурацию на момент захвата и индекс
/// первого сэмпла в потоке.
#[derive(Debug, Clone)]
pub struct IqBlock {
    /// Комплексные базовые сэмплы, нормализованные примерно в диапазон `[-1.0,
    /// 1.0]`.
    pub samples: Vec<Complex32>,

    /// Конфигурация, действовавшая при захвате этого блока.
    pub config: Arc<RfConfig>,

    /// Индекс первого сэмпла в общем потоке.
    pub start_sample: u64,
}

impl IqBlock {
    /// Возвращает длительность блока в секундах.
    #[must_use]
    pub fn duration_s(&self) -> f64 {
        self.samples.len() as f64 / self.config.sample_rate_hz
    }
}

////////////////////////////////////////////////////////////////////////////////
// Тесты
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use num_complex::Complex32;

    use super::*;
    use crate::rf::config::RfConfig;

    #[test]
    fn test_iqblock_duration() {
        let config = Arc::new(RfConfig::default());
        let block = IqBlock {
            samples: vec![Complex32::new(0.0, 0.0); 2048],
            config,
            start_sample: 0,
        };

        assert!((block.duration_s() - 0.001).abs() < 1e-9);
    }
}
