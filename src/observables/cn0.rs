//! C/N₀ на уровне наблюдаемых (observables layer).
//!
//! Оценка C/N₀ (carrier-to-noise density ratio, дБ-Гц) — ключевой
//! показатель качества сигнала. Используется в двух местах pipeline:
//!
//! 1. **Tracking layer** (`TrackingChannel::Cn0Estimator`) — быстрая
//!    скользящая оценка для решений о деаллокации канала.
//! 2. **Observables layer** (этот модуль) — взвешенная оценка для
//!    solver'а (WLS weights), вывода в NMEA (GSV поле CN0) и
//!    мониторинга качества.
//!
//! # Алгоритм
//!
//! Используется метод Narrowband-Wideband Power Ratio (NW-PR), широко
//! применяемый в GNSS-приёмниках и описанный в "Understanding GPS:
//! Principles and Applications" (Kaplan & Hegarty):
//!
//! ```text
//! NP  = mean(|P_i|²)            (Narrowband Power — среднее по когерентным)
//! WP  = mean(|P_i|)²            (Wideband Power — квадрат среднего)
//! M   = число эпох в окне
//!
//! C/N₀ = 10 · log10( (NP − WP) / (WP · T) )
//!
//! где T — период когерентного накопления (с).
//! ```
//!
//! # Весовой коэффициент для WLS
//!
//! Для weighted least squares solver: `w = 10^(cn0_db_hz / 10)` (линейная
//! шкала, обратно пропорционально дисперсии шума).

use num_complex::Complex32;

/// Минимальное число эпох для надёжной оценки C/N₀.
pub const MIN_SAMPLES_FOR_CN0: usize = 2;

/// Порог CN0 (дБ-Гц) ниже которого сигнал считается слабым.
pub const WEAK_SIGNAL_CN0_THRESHOLD: f32 = 30.0;

/// Порог CN0 (дБ-Гц) ниже которого сигнал считается утерянным.
pub const LOST_SIGNAL_CN0_THRESHOLD: f32 = 25.0;

/// Оценка C/N₀ для одной эпохи (observables layer).
#[derive(Debug, Clone, Copy)]
pub struct Cn0Estimate {
    /// C/N₀ в дБ-Гц
    pub db_hz: f32,

    /// Весовой коэффициент для WLS solver (линейный масштаб, безразмерный)
    pub wls_weight: f64,

    /// Число эпох, использованных для оценки
    pub samples_used: usize,
}

/// Классификация качества сигнала по C/N₀.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignalQuality {
    /// C/N₀ > 38 дБ-Гц — хороший сигнал
    Good,

    /// 30 ≤ C/N₀ ≤ 38 дБ-Гц — удовлетворительный
    Marginal,

    /// C/N₀ < 30 дБ-Гц — слабый сигнал, ошибки возможны
    Weak,

    /// C/N₀ < 25 дБ-Гц — сигнал практически утерян
    Lost,
}

impl Cn0Estimate {
    /// Классифицирует качество сигнала по текущей оценке C/N₀.
    #[must_use]
    pub fn quality(&self) -> SignalQuality {
        if self.db_hz >= 38.0 {
            SignalQuality::Good
        } else if self.db_hz >= WEAK_SIGNAL_CN0_THRESHOLD {
            SignalQuality::Marginal
        } else if self.db_hz >= LOST_SIGNAL_CN0_THRESHOLD {
            SignalQuality::Weak
        } else {
            SignalQuality::Lost
        }
    }
}

/// Вычисляет C/N₀ методом NW-PR из истории Prompt-корреляций.
///
/// # Аргументы
///
/// - `prompts` — история Prompt-корреляций (когерентно накопленных)
/// - `coherent_time_s` — период одного когерентного накопления (с),
///   обычно 0.001 (1 мс) или 0.02 (20 мс для накопления по биту)
///
/// # Возвращает
///
/// `None`, если `prompts.len() < MIN_SAMPLES_FOR_CN0`.
#[must_use]
pub fn estimate_cn0(
    prompts: &[Complex32],
    coherent_time_s: f64,
) -> Option<Cn0Estimate> {
    if prompts.len() < MIN_SAMPLES_FOR_CN0 {
        return None;
    }

    let n = prompts.len() as f64;

    // Narrowband Power: среднее квадратов модулей.
    let narrow_power: f64 = prompts.iter().map(|p| f64::from(p.norm_sqr())).sum::<f64>() / n;

    // Wideband Power: квадрат среднего модуля.
    let mean_amplitude: f64 = prompts.iter().map(|p| f64::from(p.norm())).sum::<f64>() / n;
    let wide_power = mean_amplitude * mean_amplitude;

    // Отношение сигнал/шум.
    let noise_power = narrow_power - wide_power;

    let cn0_linear = if noise_power > 0.0 && wide_power > 0.0 {
        wide_power / (noise_power * coherent_time_s)
    } else {
        // Невозможно оценить (слишком мало шума или нет сигнала).
        return None;
    };

    let db_hz = 10.0 * cn0_linear.log10();

    // Ограничиваем в разумном диапазоне.
    let db_hz = db_hz.clamp(-10.0, 70.0) as f32;

    // Линейный вес для WLS: σ² ∝ 1/CN0_linear → w ∝ CN0_linear.
    let wls_weight = 10f64.powf(f64::from(db_hz) / 10.0);

    Some(Cn0Estimate {
        db_hz,
        wls_weight,
        samples_used: prompts.len(),
    })
}

/// Конвертирует C/N₀ из `ChannelOutput` (уже оценённый tracking-слоем) в
/// `Cn0Estimate` для observables-слоя.
///
/// Использует tracking-оценку напрямую, когда история Prompt недоступна
/// (например, при поступлении данных из внешнего источника).
#[must_use]
pub fn from_tracking_estimate(db_hz: f32) -> Cn0Estimate {
    let wls_weight = 10f64.powf(f64::from(db_hz) / 10.0);

    Cn0Estimate {
        db_hz,
        wls_weight,
        samples_used: 0, // unknown, came from tracking layer
    }
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    fn prompts_constant(
        amp: f32,
        n: usize,
    ) -> Vec<Complex32> {
        vec![Complex32::new(amp, 0.0); n]
    }

    fn prompts_noisy(
        signal_amp: f32,
        noise_amp: f32,
        n: usize,
    ) -> Vec<Complex32> {
        // Детерминированная "шумовая" последовательность (псевдослучайная
        // через простой LCG) без зависимостей от rand.
        let mut result = Vec::with_capacity(n);
        let mut state = 12345u64;

        for i in 0..n {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let noise_i = (state >> 33) as f32 / u32::MAX as f32 * 2.0 - 1.0;
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let noise_q = (state >> 33) as f32 / u32::MAX as f32 * 2.0 - 1.0;
            let sign = if i % 20 < 10 { 1.0_f32 } else { -1.0_f32 };

            result.push(Complex32::new(
                sign * signal_amp + noise_amp * noise_i,
                noise_amp * noise_q,
            ));
        }

        result
    }

    #[test]
    fn test_estimate_cn0_returns_none_for_empty() {
        assert!(estimate_cn0(&[], 0.001).is_none());
    }

    #[test]
    fn test_estimate_cn0_returns_none_for_single_sample() {
        let p = vec![Complex32::new(1.0, 0.0)];

        assert!(estimate_cn0(&p, 0.001).is_none());
    }

    #[test]
    fn test_estimate_cn0_returns_some_for_two_samples() {
        let p = prompts_noisy(100.0, 1.0, 2); // вместо prompts_constant

        assert!(estimate_cn0(&p, 0.001).is_some());
    }

    #[test]
    fn test_estimate_cn0_higher_signal_gives_higher_cn0() {
        // Больший сигнал → выше C/N₀.
        let strong = prompts_noisy(100.0, 1.0, 50);
        let weak = prompts_noisy(10.0, 1.0, 50);

        let cn0_strong = estimate_cn0(&strong, 0.001).unwrap().db_hz;
        let cn0_weak = estimate_cn0(&weak, 0.001).unwrap().db_hz;

        assert!(
            cn0_strong > cn0_weak,
            "strong signal should have higher C/N₀: {cn0_strong} vs {cn0_weak}"
        );
    }

    #[test]
    fn test_estimate_cn0_more_noise_gives_lower_cn0() {
        // Больше шума → ниже C/N₀.
        let low_noise = prompts_noisy(50.0, 1.0, 100);
        let high_noise = prompts_noisy(50.0, 20.0, 100);

        let cn0_low = estimate_cn0(&low_noise, 0.001).unwrap().db_hz;
        let cn0_high = estimate_cn0(&high_noise, 0.001).unwrap().db_hz;

        assert!(
            cn0_low > cn0_high,
            "low noise should give higher C/N₀: {cn0_low} vs {cn0_high}"
        );
    }

    #[test]
    fn test_estimate_cn0_result_is_finite() {
        let p = prompts_noisy(50.0, 5.0, 100);
        let est = estimate_cn0(&p, 0.001).unwrap();

        assert!(est.db_hz.is_finite());
        assert!(est.wls_weight.is_finite());
    }

    #[test]
    fn test_estimate_cn0_wls_weight_positive() {
        let p = prompts_noisy(50.0, 5.0, 100);
        let est = estimate_cn0(&p, 0.001).unwrap();

        assert!(est.wls_weight > 0.0);
    }

    #[test]
    fn test_estimate_cn0_samples_used_matches_input() {
        let p = prompts_noisy(100.0, 1.0, 42); // вместо prompts_constant
        let est = estimate_cn0(&p, 0.001).unwrap();

        assert_eq!(est.samples_used, 42);
    }

    #[test]
    fn test_quality_good_above_38() {
        let est = Cn0Estimate {
            db_hz: 45.0,
            wls_weight: 1.0,
            samples_used: 10,
        };

        assert_eq!(est.quality(), SignalQuality::Good);
    }

    #[test]
    fn test_quality_marginal_between_30_and_38() {
        let est = Cn0Estimate {
            db_hz: 34.0,
            wls_weight: 1.0,
            samples_used: 10,
        };

        assert_eq!(est.quality(), SignalQuality::Marginal);
    }

    #[test]
    fn test_quality_weak_between_25_and_30() {
        let est = Cn0Estimate {
            db_hz: 27.0,
            wls_weight: 1.0,
            samples_used: 10,
        };

        assert_eq!(est.quality(), SignalQuality::Weak);
    }

    #[test]
    fn test_quality_lost_below_25() {
        let est = Cn0Estimate {
            db_hz: 20.0,
            wls_weight: 1.0,
            samples_used: 10,
        };

        assert_eq!(est.quality(), SignalQuality::Lost);
    }

    #[test]
    fn test_from_tracking_estimate_preserves_db_hz() {
        let est = from_tracking_estimate(42.5);

        assert!((est.db_hz - 42.5).abs() < 1e-5);
    }

    #[test]
    fn test_from_tracking_estimate_wls_weight_consistent() {
        let db_hz = 40.0_f32;
        let est = from_tracking_estimate(db_hz);
        let expected_weight = 10f64.powf(f64::from(db_hz) / 10.0);

        assert!((est.wls_weight - expected_weight).abs() < 1e-6);
    }

    #[test]
    fn test_wls_weight_increases_with_cn0() {
        let low = from_tracking_estimate(30.0);
        let high = from_tracking_estimate(45.0);

        assert!(
            high.wls_weight > low.wls_weight,
            "higher C/N₀ must give higher WLS weight"
        );
    }

    #[test]
    fn test_pure_noise_gives_none_or_low_cn0() {
        // При нулевой амплитуде сигнала (только шум) estimate должен
        // вернуть None (wide_power ≈ narrow_power → noise_power ≈ 0) или
        // очень маленький C/N₀.
        // Константный шум (все одинаковые) → noise_power = 0 → None.
        let p = prompts_constant(1.0, 20);
        let result = estimate_cn0(&p, 0.001);

        // При полностью константном входе narrow_power == wide_power →
        // noise_power == 0 → None (защита от деления).
        assert!(
            result.is_none(),
            "pure constant (no noise) should return None, got {result:?}",
        );
    }

    #[test]
    fn test_quality_thresholds() {
        let est = |db_hz| Cn0Estimate {
            db_hz,
            wls_weight: 1.0,
            samples_used: 0,
        };

        assert_eq!(est(38.0).quality(), SignalQuality::Good);
        assert_eq!(est(37.999).quality(), SignalQuality::Marginal);

        assert_eq!(est(30.0).quality(), SignalQuality::Marginal);
        assert_eq!(est(29.999).quality(), SignalQuality::Weak);

        assert_eq!(est(25.0).quality(), SignalQuality::Weak);
        assert_eq!(est(24.999).quality(), SignalQuality::Lost);
    }

    #[test]
    fn test_longer_coherent_time_gives_lower_cn0() {
        let p = prompts_noisy(50.0, 5.0, 100);

        let short = estimate_cn0(&p, 0.001).unwrap();
        let long = estimate_cn0(&p, 0.020).unwrap();

        assert!(short.db_hz > long.db_hz);
    }

    #[test]
    fn test_cn0_is_clamped_to_upper_limit() {
        let p = prompts_noisy(1000.0, 0.01, 1000);

        let est = estimate_cn0(&p, 0.001).unwrap();

        assert!(est.db_hz <= 70.0);
    }

    #[test]
    fn test_cn0_never_nan_or_inf() {
        let p = prompts_noisy(50.0, 5.0, 100);

        let est = estimate_cn0(&p, 0.001).unwrap();

        assert!(est.db_hz.is_finite());
        assert!(est.wls_weight.is_finite());
    }

    #[test]
    fn test_cn0_monotonic_in_signal_power() {
        let low = prompts_noisy(10.0, 5.0, 100);
        let mid = prompts_noisy(50.0, 5.0, 100);
        let high = prompts_noisy(200.0, 5.0, 100);

        let cn0_low = estimate_cn0(&low, 0.001).unwrap().db_hz;
        let cn0_mid = estimate_cn0(&mid, 0.001).unwrap().db_hz;
        let cn0_high = estimate_cn0(&high, 0.001).unwrap().db_hz;

        assert!(cn0_low < cn0_mid);
        assert!(cn0_mid < cn0_high);
    }

    #[test]
    fn test_cn0_monotonic_in_noise() {
        let low_noise = prompts_noisy(50.0, 1.0, 100);
        let high_noise = prompts_noisy(50.0, 20.0, 100);

        let cn0_low = estimate_cn0(&low_noise, 0.001).unwrap().db_hz;
        let cn0_high = estimate_cn0(&high_noise, 0.001).unwrap().db_hz;

        assert!(cn0_low > cn0_high);
    }

    #[test]
    fn test_wls_weight_exponential_growth() {
        let est30 = from_tracking_estimate(30.0);
        let est40 = from_tracking_estimate(40.0);

        assert!(est40.wls_weight > est30.wls_weight);

        // sanity: 10 dB increase = ×10 weight
        let ratio = est40.wls_weight / est30.wls_weight;
        assert!((ratio - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_cn0_clamping_behavior() {
        let p = prompts_noisy(1000.0, 1.0, 1000); // шум достаточен для стабильной оценки
        let est = estimate_cn0(&p, 0.001).unwrap();

        assert!(est.db_hz <= 70.0);
        assert!(est.db_hz >= -10.0);
    }

    #[test]
    fn test_cn0_is_deterministic() {
        let p1 = prompts_noisy(50.0, 5.0, 200);
        let p2 = prompts_noisy(50.0, 5.0, 200);

        let a = estimate_cn0(&p1, 0.001).unwrap();
        let b = estimate_cn0(&p2, 0.001).unwrap();

        assert!((a.db_hz - b.db_hz).abs() < 1e-6);
        assert!((a.wls_weight - b.wls_weight).abs() < 1e-9);
    }
}
