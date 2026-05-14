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

use crate::{FirFilter, Window};

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
    pub fn new(factor: usize) -> Self {
        assert!(factor >= 2, "decimation factor must be >= 2");

        let cutoff_norm = 0.45 / factor as f64;
        let coeffs = build_lp_coeffs(cutoff_norm, 63);

        Self::with_filter(factor, FirFilter::new(coeffs))
    }

    /// Создаёт дециматор с пользовательским FIR-фильтром.
    ///
    /// Позволяет использовать собственные характеристики фильтра
    /// (например, более узкую переходную полосу).
    pub fn with_filter(
        factor: usize,
        filter: FirFilter,
    ) -> Self {
        assert!(factor >= 2);

        Self { filter, factor }
    }

    /// Возвращает коэффициент децимации.
    pub fn factor(&self) -> usize {
        self.factor
    }

    /// Вычисляет выходную частоту дискретизации.
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
    pub fn with_filter(
        factor: usize,
        filter: FirFilter,
    ) -> Self {
        assert!(factor >= 2);

        Self { filter, factor }
    }

    /// Возвращает коэффициент интерполяции.
    pub fn factor(&self) -> usize {
        self.factor
    }

    /// Вычисляет выходную частоту дискретизации.
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

fn build_lp_coeffs(
    fc: f64,
    num_taps: usize,
) -> Vec<f32> {
    FirFilter::low_pass(fc * 2_048_000.0, 2_048_000.0, num_taps, Window::Hamming)
        .coeffs()
        .to_vec()
}

////////////////////////////////////////////////////////////////////////////////
// Тесты
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
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
}
