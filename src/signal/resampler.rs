//! Модуль ресемплинга (изменения частоты дискретизации).
//!
//! Содержит реализацию:
//!
//! - [`Decimator`] — понижение частоты дискретизации (downsampling)
//! - [`Interpolator`] — повышение частоты дискретизации (upsampling)
//!
//! ## Подход
//!
//! Используется классическая схема:
//!
//! - **Decimation**:
//!   1. Антиалиасинговый FIR-фильтр
//!   2. Выбор каждого `M`-го отсчёта
//!
//! - **Interpolation**:
//!   1. Вставка нулей (zero-stuffing)
//!   2. Сглаживающий FIR-фильтр (anti-imaging)
//!
//! ## FIR-фильтр
//!
//! Встроенный фильтр:
//!
//! - тип: sinc + окно Хэмминга
//! - длина: 63 taps
//! - частота среза: ≈ `0.45 / factor`
//! - подавление в стоп-полосе: ~ −40…−45 dB
//!
//! ## Особенности
//!
//! - фильтры сохраняют внутреннее состояние (подходят для streaming)
//! - корректно работают с комплексными сигналами (IQ)
//! - интерполятор компенсирует ослабление после zero-stuffing
//!
//! ## Применение
//!
//! - SDR / GNSS приёмники
//! - согласование частот дискретизации между блоками
//! - цифровая обработка сигналов (DSP pipelines)

use num_complex::Complex32;

use crate::signal::filter::{FirFilter, Window};

/// Дециматор (понижение частоты дискретизации).
///
/// Выполняет:
/// 1. Антиалиасинговую фильтрацию (LPF)
/// 2. Выбор каждого `factor`-го отсчёта (downsampling)
///
/// Это предотвращает aliasing при уменьшении частоты дискретизации.
///
/// # Детали реализации
///
/// - Используется FIR-фильтр (sinc + окно Хэмминга)
/// - Частота среза ≈ `0.45 / factor`
/// - Типичное подавление в стоп-полосе ≈ −40…−45 dB
///
/// # Состояние
///
/// Внутренний FIR-фильтр сохраняет линию задержки между вызовами,
/// что делает структуру пригодной для потоковой обработки.
pub struct Decimator {
    /// Антиалиасинговый FIR-фильтр.
    filter: FirFilter,

    /// Коэффициент децимации (во сколько раз уменьшается частота).
    factor: usize,
}

/// Интерполятор (увеличение частоты дискретизации).
///
/// Выполняет:
/// 1. Вставку нулей (zero-stuffing)
/// 2. Сглаживание FIR-фильтром (anti-imaging)
///
/// # Детали реализации
///
/// - После вставки нулей амплитуда сигнала уменьшается в `factor` раз
/// - FIR-фильтр масштабируется на `factor`, чтобы компенсировать это
///
/// # Состояние
///
/// FIR-фильтр сохраняет линию задержки между вызовами.
pub struct Interpolator {
    /// Сглаживающий FIR-фильтр.
    filter: FirFilter,

    /// Коэффициент интерполяции.
    factor: usize,
}

impl Decimator {
    /// Создаёт дециматор со встроенным FIR-фильтром.
    ///
    /// # Аргументы
    /// - `factor` — коэффициент децимации (>= 2)
    ///
    /// # Паника
    /// Если `factor < 2`
    #[must_use]
    pub fn new(factor: usize) -> Self {
        assert!(factor >= 2, "decimation factor must be >= 2");

        // Normalised cutoff: 0.5/factor (just below the new Nyquist frequency)
        let cutoff_norm = 0.45 / factor as f64; // slight guard band
        let coeffs = build_lp_coeffs(cutoff_norm, 63);

        Self::with_filter(factor, FirFilter::new(coeffs))
    }

    /// Создаёт дециматор с пользовательским FIR-фильтром.
    ///
    /// Позволяет использовать собственные характеристики фильтра
    /// (например, более узкую переходную полосу).
    #[must_use]
    pub fn with_filter(
        factor: usize,
        filter: FirFilter,
    ) -> Self {
        assert!(factor >= 2);

        Self { filter, factor }
    }

    /// Возвращает коэффициент децимации.
    #[must_use]
    pub const fn factor(&self) -> usize {
        self.factor
    }

    /// Вычисляет выходную частоту дискретизации.
    #[must_use]
    pub fn output_rate(
        &self,
        input_rate_hz: f64,
    ) -> f64 {
        input_rate_hz / self.factor as f64
    }

    /// Выполняет децимацию сигнала.
    ///
    /// # Возвращает
    /// Вектор длиной примерно `input.len() / factor`
    pub fn decimate(
        &mut self,
        input: &[Complex32],
    ) -> Vec<Complex32> {
        self.filter
            .apply(input)
            .into_iter()
            .step_by(self.factor)
            .collect()
    }
}

impl Interpolator {
    /// Создаёт интерполятор со встроенным FIR-фильтром.
    ///
    /// # Аргументы
    /// - `factor` — коэффициент интерполяции (>= 2)
    ///
    /// # Паника
    /// Если `factor < 2`
    #[must_use]
    pub fn new(factor: usize) -> Self {
        assert!(factor >= 2, "interpolation factor nust be >= 2");

        let cutoff_norm = 0.45 / factor as f64;

        // Масштабируем коэффициенты для компенсации zero-stuffing
        let coeffs: Vec<f32> = build_lp_coeffs(cutoff_norm, 63)
            .into_iter()
            .map(|c| c * factor as f32)
            .collect();

        Self::with_filter(factor, FirFilter::new(coeffs))
    }

    /// Создаёт интерполятор с пользовательским FIR-фильтром.
    #[must_use]
    pub fn with_filter(
        factor: usize,
        filter: FirFilter,
    ) -> Self {
        assert!(factor >= 2);

        Self { filter, factor }
    }

    /// Возвращает коэффициент интерполяции.
    #[must_use]
    pub const fn factor(&self) -> usize {
        self.factor
    }

    /// Вычисляет выходную частоту дискретизации.
    #[must_use]
    pub fn output_rate(
        &self,
        input_rate_hz: f64,
    ) -> f64 {
        input_rate_hz * self.factor as f64
    }

    /// Выполняет интерполяцию сигнала.
    ///
    /// # Алгоритм
    /// 1. Вставка `(factor - 1)` нулей между отсчётами
    /// 2. FIR-фильтрация для восстановления сигнала
    ///
    /// # Возвращает
    /// Вектор длиной `input.len() * factor`
    pub fn interpolate(
        &mut self,
        input: &[Complex32],
    ) -> Vec<Complex32> {
        let mut upsampled = vec![Complex32::default(); input.len() * self.factor];

        for (i, &s) in input.iter().enumerate() {
            upsampled[i * self.factor] = s;
        }

        self.filter.apply(&upsampled)
    }
}

/// Build a Hamming-windows sinc LPF at normalised cutoff `fc`.
fn build_lp_coeffs(
    fc: f64,
    num_taps: usize,
) -> Vec<f32> {
    FirFilter::low_pass(fc * 2_048_000.0, 2_048_000.0, num_taps, Window::Hamming)
        .coeffs()
        .to_vec()
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use core::f64;

    use super::*;

    #[test]
    fn test_decimator_output_length_exact() {
        let mut d = Decimator::new(4);
        let input = vec![Complex32::new(1.0, 0.0); 2048];
        let out = d.decimate(&input);

        assert_eq!(out.len(), 512);
    }

    #[test]
    fn test_decimator_output_length_floor() {
        let mut d = Decimator::new(4);
        let out = d.decimate(&vec![Complex32::default(); 100]);

        assert_eq!(out.len(), 25);
    }

    #[test]
    fn test_decimator_passes_dc() {
        let mut d = Decimator::new(4);
        let dc: Vec<Complex32> = vec![Complex32::new(1.0, 0.0); 512];
        let out = d.decimate(&dc);
        let skip = (d.filter.num_taps() - 1) / d.factor + 1;

        for s in out.iter().skip(skip) {
            assert!((s.re - 1.0).abs() < 0.02, "DC re={}", s.re);
        }
    }

    #[test]
    fn test_decimator_attenuates_high_freq() {
        let mut d = Decimator::new(4);

        // High-frequency tone near input Nyquist: should be attenuated
        let n = 2048;
        let fs = 2_048_000.0_f64;
        let f_alias = 700_000.0_f64; // > fs / (2 * factor), must be filtered
        let tone: Vec<Complex32> = (0..n)
            .map(|i| {
                let t = f64::from(i) / fs;
                Complex32::new((2.0 * std::f64::consts::PI * f_alias * t).cos() as f32, 0.0)
            })
            .collect();
        let out = d.decimate(&tone);
        let skip = d.filter.group_delay_samples() / d.factor + 1;
        let max_amp: f32 = out.iter().skip(skip).map(|s| s.norm()).fold(0.0, f32::max);

        assert!(max_amp < 0.1, "alias not suppressed: max_amp={max_amp}");
    }

    #[test]
    fn test_decimator_state_continuous_across_blocks() {
        let input: Vec<Complex32> = (0..256)
            .map(|n| Complex32::new((n as f32).sin(), 0.0))
            .collect();
        let mut d1 = Decimator::new(2);
        let mut d2 = Decimator::new(2);
        let full = d1.decimate(&input);
        let p1 = d2.decimate(&input[..128]);
        let p2 = d2.decimate(&input[128..]);
        let split: Vec<_> = p1.iter().chain(p2.iter()).copied().collect();

        for (a, b) in full.iter().zip(split.iter()) {
            assert!((a.re - b.re).abs() < 1e-5, "a={} b={}", a.re, b.re);
        }
    }

    #[test]
    fn test_decimator_output_rate() {
        let d = Decimator::new(4);

        assert!((d.output_rate(2_048_000.0) - 512_000.0).abs() < 1e-9);
    }

    #[test]
    fn test_interpolator_output_length() {
        let mut i = Interpolator::new(4);

        let input = vec![Complex32::default(); 128];
        let out = i.interpolate(&input);

        assert_eq!(out.len(), 512);
    }

    #[test]
    fn test_interpolator_output_rate() {
        let i = Interpolator::new(4);

        assert!((i.output_rate(512_000.0) - 2_048_000.0).abs() < 1e-9);
    }

    #[test]
    fn test_interpolator_passes_dc() {
        let mut interp = Interpolator::new(4);
        let dc: Vec<Complex32> = vec![Complex32::new(1.0, 0.0); 128];
        let out = interp.interpolate(&dc);
        // After transient, DC should be near 1.0
        let skip = interp.filter.group_delay_samples() + 8;

        for s in out.iter().skip(skip) {
            assert!((s.re - 1.0).abs() < 0.05, "DC re={}", s.re);
        }
    }

    #[test]
    fn test_decimator_then_interpolator_round_trip_dc() {
        // DC → decimate(2) → interpolate(2) should return ~DC in steady state.
        // Cascaded transient budget:
        //   dec filter (63 taps): 62 input samples → 31 dec output samples in transient
        //   These 31 dec transient samples → 62 interp input samples
        //   interp filter (63 taps): adds ~62 more output samples of transient
        //   Total: ~124 interp output samples before steady state.
        // Use 1024 input samples so there are ample steady-state output samples.
        let mut dec = Decimator::new(2);
        let mut interp = Interpolator::new(2);
        let dc: Vec<Complex32> = vec![Complex32::new(1.0, 0.0); 1024];
        let downsampled = dec.decimate(&dc);
        let restored = interp.interpolate(&downsampled);
        // Skip the full cascaded transient (use conservative estimate).
        let dec_transient_in_interp_samples = (dec.filter.num_taps() - 1) * 2;
        let interp_transient = interp.filter.num_taps() - 1;
        let skip = dec_transient_in_interp_samples + interp_transient + 4;

        for s in restored.iter().skip(skip) {
            assert!((s.re - 1.0).abs() < 0.1, "round-trip re={}", s.re);
        }
    }
}
