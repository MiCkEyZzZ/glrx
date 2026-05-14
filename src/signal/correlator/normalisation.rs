//! Утилиты обработки мощности и амплитуды комплексных сигналов.
//!
//! Модуль предоставляет базовые операции для DSP:
//!
//! * вычисление мощности и RMS
//! * масштабирование (вещественное и комплексное)
//! * нормализация сигнала по мощности
//!
//! # Основные определения
//!
//! Для комплексного сигнала `s[n] = I + jQ`:
//!
//! ```text
//! |s[n]|² = I² + Q²
//! ```
//!
//! * **Средняя мощность**:
//!
//! ```text
//! P = (1/N) Σ |s[n]|²
//! ```
//!
//! * **RMS (Root Mean Square)**:
//!
//! ```text
//! RMS = sqrt(P)
//! ```
//!
//! # Применение
//!
//! Используется в:
//!
//! * нормализации входных данных перед корреляцией
//! * AGC (automatic gain control)
//! * оценке SNR
//! * подготовке сигналов для PLL/DLL
//!
//! # Замечания
//!
//! * Все операции выполняются в базовой полосе (complex baseband)
//! * Нормализация безопасна при нулевом сигнале (без деления на ноль)
//! * Комплексное масштабирование позволяет выполнять фазовый сдвиг

use num_complex::Complex32;

/// Вычисляет **среднюю мощность** блока комплексного сигнала.
///
/// Формула:
///
/// ```text
/// P = (1/N) · Σ |s[n]|²
/// ```
///
/// где:
///
/// - `s[n]` — комплексные отсчёты сигнала
/// - `|s[n]|² = I² + Q²`
/// - `N` — число отсчётов
///
/// Если массив пустой, возвращается `0.0`.
#[inline]
#[must_use]
pub fn compute_power(samples: &[Complex32]) -> f32 {
    if samples.is_empty() {
        return 0.0;
    }

    // Сумма квадратов амплитуд
    let total: f32 = samples.iter().map(|s| s.norm_sqr()).sum();

    // Средняя мощность
    total / samples.len() as f32
}

/// Вычисляет **RMS-амплитуду** сигнала.
///
/// RMS (Root Mean Square):
///
/// ```text
/// RMS = sqrt((1/N) Σ |s[n]|²)
/// ```
///
/// Фактически это квадратный корень из средней мощности сигнала.
#[inline]
#[must_use]
pub fn compute_rms(samples: &[Complex32]) -> f32 {
    compute_power(samples).sqrt()
}

/// Масштабирует сигнал на **вещественный коэффициент**.
///
/// Каждому отсчёту применяется операция:
///
/// ```text
/// s[n] = s[n] · factor
/// ```
#[inline]
pub fn scale(
    samples: &mut [Complex32],
    factor: f32,
) {
    for s in samples.iter_mut() {
        *s *= factor;
    }
}

/// Масштабирует сигнал на **комплексный коэффициент**.
///
/// Используется, например:
///
/// • для фазового вращения сигнала
/// • для компенсации несущей
/// • для цифрового микширования
///
/// Операция:
///
/// ```text
/// s[n] = s[n] · factor
/// ```
#[inline]
pub fn scale_complex(
    samples: &mut [Complex32],
    factor: Complex32,
) {
    for s in samples.iter_mut() {
        *s *= factor;
    }
}

/// Нормирует сигнал к заданной **средней мощности** `target_power`.
///
/// После нормализации выполняется условие:
///
/// ```text
/// mean(|s|²) = target_power
/// ```
///
/// Алгоритм:
///
/// 1. вычисляется текущая мощность `p`
/// 2. находится масштаб:
///
/// ```text
/// scale = sqrt(target_power / p)
/// ```
///
/// 3. сигнал умножается на этот коэффициент
///
/// Если текущая мощность очень мала (`< f32::EPSILON`),
/// функция ничего не делает, чтобы избежать деления на ноль.
pub fn normalize_to_power(
    samples: &mut [Complex32],
    target_power: f32,
) {
    let p = compute_power(samples);

    if p < f32::EPSILON {
        return;
    }

    scale(samples, (target_power / p).sqrt());
}

/// Нормирует сигнал к **единичной средней мощности**.
///
/// После вызова:
///
/// ```text
/// mean(|s|²) = 1
/// ```
///
/// Это стандартная операция предварительной
/// нормализации сигналов в DSP.
pub fn normalize(samples: &mut [Complex32]) {
    normalize_to_power(samples, 1.0);
}

////////////////////////////////////////////////////////////////////////////////
// Тесты
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_power_empty_is_zezo() {
        assert_eq!(compute_power(&[]), 0.0);
    }

    #[test]
    fn test_compute_power_unit_signal() {
        let samples = vec![Complex32::new(1.0, 0.0); 100];
        let p = compute_power(&samples);

        assert!((p - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_power_complex_signal() {
        // |1 + 1j|^2 = 2
        let samples = vec![Complex32::new(1.0, 1.0); 50];

        let p = compute_power(&samples);

        assert!((p - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_rms_is_sqrt_of_power() {
        let samples = vec![Complex32::new(3.0, 4.0); 16]; // |s|^2 = 25
        let rms = compute_rms(&samples);

        assert!((rms - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_scale_multiplies_samples() {
        let mut samples = vec![Complex32::new(2.0, 3.0); 8];

        scale(&mut samples, 0.5);

        for s in samples {
            assert!((s.re - 1.0).abs() < 1e-6);
            assert!((s.im - 1.5).abs() < 1e-6);
        }
    }

    #[test]
    fn test_scale_complex_rotates_signal() {
        let mut samples = vec![Complex32::new(1.0, 0.0)];

        // умножение на j -> поворот на 90 градусов
        scale_complex(&mut samples, Complex32::new(0.0, 1.0));

        assert!(samples[0].re.abs() < 1e-6);
        assert!((samples[0].im - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_normalize_to_power_sets_target_power() {
        let mut samples = vec![Complex32::new(3.0, 4.0); 64]; // |s|^2 = 25

        normalize_to_power(&mut samples, 1.0);

        let p = compute_power(&samples);

        assert!((p - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_normalize_sets_unit_power() {
        let mut samples: Vec<Complex32> = (1..=16).map(|n| Complex32::new(n as f32, 0.0)).collect();

        normalize(&mut samples);

        let p = compute_power(&samples);

        assert!((p - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_normalize_handles_zero_signal() {
        let mut samples = vec![Complex32::new(0.0, 0.0); 10];

        normalize(&mut samples);

        let p = compute_power(&samples);

        assert_eq!(p, 0.0);
    }
}
