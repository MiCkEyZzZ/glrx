//! Нормализация и оценка мощности комплексных сигналов.
//!
//! Модуль предоставляет:
//!
//! * [`compute_power`] / [`compute_rms`] — вычисление мощности и RMS
//! * [`scale`] / [`scale_complex`] — вещественное и комплексное масштабирование
//! * [`normalize`] / [`normalize_to_power`] — нормализация по целевой мощности
//! * [`cn0_estimate`] — оценка C/N₀ методом moment estimator
//! * [`cn0_estimate_iwbp`] — оценка C/N₀ методом NB/WB power ratio
//!
//! # Основные определения
//!
//! Для комплексного сигнала `s[n] = I + jQ`:
//!
//! ```text
//! |s[n]|² = I² + Q²
//!
//! Средняя мощность:  P   = (1/N) Σ |s[n]|²
//! RMS-амплитуда:     RMS = √P
//! ```
//!
//! # Применение в GNSS
//!
//! | Операция | Где используется |
//! |----------|-----------------|
//! | `normalize` | Нормализация перед корреляцией |
//! | `compute_power` | AGC (Automatic Gain Control) |
//! | `cn0_estimate` | Оценка качества сигнала в tracking-loop |
//! | `scale_complex` | Компенсация начальной фазы несущей |

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

/// Оценка C/N₀ методом **moment estimator** (narrow-band power ratio).
///
/// Алгоритм:
///
/// ```text
/// P_coh  = mean(|P[k]|²)          — когерентная мощность
/// P_nc   = mean(|P[k]|)           — некогерентная амплитуда
/// P_nc²  = P_nc²                  — некогерентная мощность
///
/// CN0 = 10·log₁₀( P_coh / (P_nc² − P_coh) / T_coh )  дБ-Гц
/// ```
///
/// # Аргументы
///
/// * `prompt_accumulations` — выборка prompt-корреляций за несколько эпох
///   (типично 20 значений = 20 мс при T_coh = 1 мс)
/// * `coherent_time_s` — длительность одного интервала интеграции (секунды)
///
/// # Возвращает
///
/// C/N₀ в **дБ-Гц**. Типичные значения для GPS L1 C/A: 35–50 дБ-Гц.
///
/// Возвращает `0.0` если накоплений менее 2.
pub fn cn0_estimate(
    prompt_accumulations: &[Complex32],
    coherent_time_s: f64,
) -> f32 {
    if prompt_accumulations.len() < 2 {
        return 0.0;
    }
    let n = prompt_accumulations.len() as f32;

    // Средняя когерентная мощность: mean(|P|²)
    let p_coh: f32 = prompt_accumulations
        .iter()
        .map(|p| p.norm_sqr())
        .sum::<f32>()
        / n;

    // Средняя некогерентная амплитуда, возведённая в квадрат: mean(|P|)²
    let mean_env: f32 = prompt_accumulations.iter().map(|p| p.norm()).sum::<f32>() / n;
    let p_nc_sq = mean_env * mean_env;

    let denom = (p_nc_sq - p_coh).max(f32::EPSILON);
    let cn0_linear = p_coh / denom / coherent_time_s as f32;

    10.0 * cn0_linear.max(f32::EPSILON).log10()
}

/// Оценка C/N₀ методом **NB/WB power ratio** (Baulieu's estimator).
///
/// ```text
/// CN0 = 10·log₁₀((P_nb − P_wb) / P_wb) − 10·log₁₀(T_coh)
/// ```
///
/// * `P_nb` — мощность в узкой полосе (из коррелятора): `|I|² + |Q|²`
/// * `P_wb` — мощность в широкой полосе (оценка шумового пола)
///
/// # Возвращает
///
/// C/N₀ в дБ-Гц.
pub fn cn0_estimate_iwbp(
    narrow_band_power: f32,
    wide_band_power: f32,
    coherent_time_s: f64,
) -> f32 {
    let ratio =
        (narrow_band_power - wide_band_power).max(f32::EPSILON) / wide_band_power.max(f32::EPSILON);
    10.0 * ratio.log10() - 10.0 * (coherent_time_s as f32).log10()
}

////////////////////////////////////////////////////////////////////////////////
// Tests
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
    fn test_power_quadrature() {
        // |1+1j|² = 2
        let s = vec![Complex32::new(1.0, 1.0); 50];

        assert!((compute_power(&s) - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_power_3_4_is_25() {
        let s = vec![Complex32::new(3.0, 4.0); 32]; // |s|² = 25

        assert!((compute_power(&s) - 25.0).abs() < 1e-5);
    }

    #[test]
    fn test_power_mixed_values() {
        let s = vec![
            Complex32::new(1.0, 0.0), // |s|² = 1
            Complex32::new(0.0, 1.0), // |s|² = 1
            Complex32::new(1.0, 1.0), // |s|² = 2
        ];

        // mean = (1+1+2)/3 = 4/3
        assert!((compute_power(&s) - 4.0 / 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_rms_is_sqrt_of_power() {
        let s = vec![Complex32::new(3.0, 4.0); 16]; // |s|² = 25, rms = 5

        assert!((compute_rms(&s) - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_rms_unit_signal() {
        let s = vec![Complex32::new(1.0, 0.0); 64];

        assert!((compute_rms(&s) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_scale_halves_amplitude() {
        let mut s = vec![Complex32::new(2.0, 4.0); 8];

        scale(&mut s, 0.5);

        for x in &s {
            assert!((x.re - 1.0).abs() < 1e-6);
            assert!((x.im - 2.0).abs() < 1e-6);
        }
    }

    #[test]
    fn test_scale_zero_factor() {
        let mut s = vec![Complex32::new(5.0, 3.0); 4];

        scale(&mut s, 0.0);

        for x in &s {
            assert_eq!(*x, Complex32::new(0.0, 0.0));
        }
    }

    #[test]
    fn test_scale_negative_factor_flips_sign() {
        let mut s = vec![Complex32::new(1.0, 0.0)];

        scale(&mut s, -1.0);

        assert!((s[0].re + 1.0).abs() < 1e-6);
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
    fn test_scale_complex_rotation_90_deg() {
        let mut s = vec![Complex32::new(1.0, 0.0)];

        // Умножение на j = 90° поворот
        scale_complex(&mut s, Complex32::new(0.0, 1.0));

        assert!(s[0].re.abs() < 1e-6, "re={}", s[0].re);
        assert!((s[0].im - 1.0).abs() < 1e-6, "im={}", s[0].im);
    }

    #[test]
    fn test_scale_complex_by_one_identity() {
        let mut s = vec![Complex32::new(3.0, -2.0)];

        scale_complex(&mut s, Complex32::new(1.0, 0.0));

        assert!((s[0].re - 3.0).abs() < 1e-6);
        assert!((s[0].im + 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_normalize_unit_power_after_call() {
        let mut s: Vec<Complex32> = (1..=16).map(|n| Complex32::new(n as f32, 0.0)).collect();

        normalize(&mut s);

        assert!((compute_power(&s) - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_normalize_already_unit_unchanged() {
        let mut s = vec![Complex32::new(1.0, 0.0); 64];

        normalize(&mut s);

        assert!((compute_power(&s) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_normalize_preserves_direction() {
        let original = vec![Complex32::new(3.0, 4.0); 16];
        let mut s = original.clone();

        normalize(&mut s);

        // Проверяем, что направление (I/Q соотношение) сохранилось
        let angle_orig = original[0].im.atan2(original[0].re);
        let angle_norm = s[0].im.atan2(s[0].re);

        assert!((angle_orig - angle_norm).abs() < 1e-5);
    }

    #[test]
    fn test_normalize_zero_signal_no_panic() {
        let mut s = vec![Complex32::new(0.0, 0.0); 10];
        normalize(&mut s); // не должно быть паники
        assert_eq!(compute_power(&s), 0.0);
    }

    #[test]
    fn test_normalize_to_power_target() {
        let mut s = vec![Complex32::new(3.0, 4.0); 64]; // P = 25

        normalize_to_power(&mut s, 4.0);

        assert!((compute_power(&s) - 4.0).abs() < 1e-4);
    }

    #[test]
    fn test_normalize_to_power_high_target() {
        let mut s = vec![Complex32::new(1.0, 0.0); 32]; // P = 1

        normalize_to_power(&mut s, 100.0);

        assert!((compute_power(&s) - 100.0).abs() < 1e-3);
    }

    #[test]
    fn test_cn0_estimate_too_few_samples_returns_zero() {
        let acc = vec![Complex32::new(10.0, 0.0)];

        assert_eq!(cn0_estimate(&acc, 0.001), 0.0);
    }

    #[test]
    fn test_cn0_estimate_empty_returns_zero() {
        assert_eq!(cn0_estimate(&[], 0.001), 0.0);
    }

    #[test]
    fn test_cn0_estimate_strong_signal_above_40dbhz() {
        // Нешумный сигнал → очень высокий C/N₀
        let acc: Vec<Complex32> = vec![Complex32::new(1000.0, 0.0); 20];
        let cn0 = cn0_estimate(&acc, 0.001);

        assert!(cn0 > 40.0, "expected CN0 > 40 dBHz, got {}", cn0);
    }

    #[test]
    fn test_cn0_increases_with_snr() {
        // Более сильный сигнал → выше C/N₀
        let weak: Vec<Complex32> = (0..20)
            .map(|i| Complex32::new(1.0 + i as f32 * 0.01, i as f32 * 0.05))
            .collect();
        let strong: Vec<Complex32> = weak.iter().map(|s| s * 100.0).collect();
        let cn0_weak = cn0_estimate(&weak, 0.001);
        let cn0_strong = cn0_estimate(&strong, 0.001);

        assert!(
            cn0_strong > cn0_weak,
            "strong={} weak={}",
            cn0_strong,
            cn0_weak
        );
    }

    #[test]
    fn test_cn0_estimate_two_samples_minimum() {
        let acc = vec![Complex32::new(10.0, 0.0), Complex32::new(10.0, 0.0)];
        let cn0 = cn0_estimate(&acc, 0.001);

        // С двумя одинаковыми семплами mean_env² = p_coh → denom ≈ 0 → clamped
        assert!(cn0.is_finite());
    }

    #[test]
    fn test_cn0_iwbp_positive_result() {
        // narrow > wide → положительный C/N₀
        let cn0 = cn0_estimate_iwbp(100.0, 1.0, 0.001);

        assert!(cn0 > 0.0, "cn0={}", cn0);
    }

    #[test]
    fn test_cn0_iwbp_finite_for_equal_powers() {
        // При равных мощностях числитель = 0 → clamped
        let cn0 = cn0_estimate_iwbp(10.0, 10.0, 0.001);

        assert!(cn0.is_finite());
    }

    #[test]
    fn test_cn0_iwbp_increases_with_snr() {
        let cn0_low = cn0_estimate_iwbp(10.0, 5.0, 0.001);
        let cn0_high = cn0_estimate_iwbp(100.0, 5.0, 0.001);

        assert!(cn0_high > cn0_low, "low={} high={}", cn0_low, cn0_high);
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
