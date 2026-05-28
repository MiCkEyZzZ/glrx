//! Обработанный блок IQ-данных на выходе signal-слоя.
//!
//! В отличии от [`IqBlock`] из RF-слоя, [`SignalBlock`] содержит данные после
//! downconversion и фильтрации, а также метаданные о приёмной обработке.

use num_complex::Complex32;

/// Блок комплексных сэмплов после обработки в signal-слое.
///
/// Передаётся в acquisition/tracling как вход корреляции.
#[derive(Debug, Clone)]
pub struct SignalBlock {
    /// Комплексные baseband-сэмплы (после downconversion)
    pub samples: Vec<Complex32>,

    /// Частота дискретизации после всей обработки (может отличаться от исходной
    /// после децимации)
    pub sample_rate_hz: f64,

    /// Центральная частота на момент захвата (до downconversion)
    pub center_freq_hz: f64,

    /// Индекс первого сэмпла в оригинальном потоке
    pub start_sample: u64,

    /// Применённый доплеровский сдвиг при downconversion (Гц)
    pub applied_doppler_hz: f64,
}

impl SignalBlock {
    /// Создаёт новый блок.
    #[must_use]
    pub fn new(
        samples: Vec<Complex32>,
        sample_rate_hz: f64,
        center_freq_hz: f64,
        start_sample: u64,
    ) -> Self {
        Self {
            samples,
            sample_rate_hz,
            center_freq_hz,
            start_sample,
            applied_doppler_hz: 0.0,
        }
    }

    /// Длительность блока в секундах.
    #[must_use]
    pub fn duration_sec(&self) -> f64 {
        self.samples.len() as f64 / self.sample_rate_hz
    }

    /// Количество сэмплов в одном миллисекунде при текущей частоте.
    #[must_use]
    pub fn samples_per_ms(&self) -> usize {
        (self.sample_rate_hz / 1000.0) as usize
    }

    /// Возвращает срез сэмплов за один интервал интеграции (1мс).
    ///
    /// Если блок короче 1мс - возвращает весь срез.
    #[must_use]
    pub fn first_ms(&self) -> &[Complex32] {
        let n = self.samples_per_ms().min(self.samples.len());

        &self.samples[..n]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn block(
        n: usize,
        fs: f64,
    ) -> SignalBlock {
        SignalBlock::new(vec![Complex32::new(1.0, 0.0); n], fs, 1_575_420_000.0, 0)
    }

    #[test]
    fn test_duration_1ms() {
        let b = block(2048, 2_048_000.0);

        assert!((b.duration_sec() - 0.001).abs() < 1e-9);
    }

    #[test]
    fn test_samples_per_ms_at_2mhz() {
        let b = block(8192, 2_048_000.0);

        assert_eq!(b.first_ms().len(), 2048);
    }

    #[test]
    fn test_first_ms_length() {
        let b = block(8192, 2_048_000.0);

        assert_eq!(b.first_ms().len(), 2048);
    }

    #[test]
    fn test_first_ms_short_block() {
        let b = block(512, 2_048_000.0);

        assert_eq!(b.first_ms().len(), 512);
    }
}
