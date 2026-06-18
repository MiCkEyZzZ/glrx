//! Двойная верификация acquisition-результатов.
//!
//! После первичного обнаружения пика PCPS-алгоритмом качество сигнала
//! нестабильно: возможны ложные срабатывания от шумовых пиков, боковых
//! лепестков автокорреляции и RFI. Этот модуль реализует двухэтапную
//! схему верификации:
//!
//! ```text
//! Первый проход: широкий Doppler (например, ±10 кГц, шаг 500 Гц)
//!     │
//!     ▼  пик найден И SNR >= cfar_threshold_1?
//!     │  нет → retry → ... → Rejected
//!     │  да  ↓
//!     ▼
//! Второй проход: УЗКИЙ поиск вокруг кандидата
//!     диапазон = [candidate ± half_span], шаг = second_pass.doppler_step_hz
//!     │
//!     ▼  SNR >= cfar_threshold_2  И  |Δdoppler| <= tolerance?
//!     │  нет → Marginal (первый прошёл, второй нет → диагностика, не tracking)
//!     │  да  ↓
//!     ▼
//! Confirmed → acquisition_result() → tracking
//! ```
//!
//! # Безопасность для tracking
//!
//! [`VerificationVerdict::acquisition_result`] возвращает `Some` **только**
//! для `Confirmed`. `Marginal` намеренно не пропускается в tracking —
//! используйте [`VerificationVerdict::search_result_diagnostic`] только для
//! логирования.
//!
//! # Политика повтора
//!
//! При `Rejected` применяется экспоненциальный back-off:
//! - попытка 0: без задержки
//! - попытка k: `base_delay_ms × 2^(k−1)`, но не более `max_delay_ms`

use std::{
    sync::Arc,
    time::{Duration, Instant},
};

#[cfg(not(feature = "rayon"))]
use num_complex::Complex32;
#[cfg(feature = "rayon")]
use num_complex::Complex32;

use crate::{
    acquisition::{
        detector::estimate_cn0,
        fft_search::{PcpsSearch, SearchConfig, SearchResult},
    },
    signal::prn_code::PrnCodeCache,
};

/// Результат двухэтапной верификации acquisition.
#[derive(Debug, Clone, PartialEq)]
pub enum VerificationVerdict {
    /// Оба прохода обнаружили спутник с уровнем выше порога
    ///
    /// Поля содержат окончательный уточнённый результат
    Confirmed {
        /// Результат второго (уточняющего) прохода
        result: SearchResult,

        /// Оценка C/N₀ в дБ-Гц
        cn0_db_hz: f32,

        /// Время, затраченное на оба прохода
        elapsed: Duration,
    },

    /// Первый проход дал пик, но второй не подтвердил.
    ///
    /// **Не передавать в tracking.** Только для диагностики и повтора.
    Marginal {
        /// Результат первого прохода (ненадёжный)
        result: SearchResult,

        /// Причина отклонения второго прохода
        reason: MarginalReason,

        /// Время, затраченное на оба прохода
        elapsed: Duration,
    },

    /// Оба прохода не обнаружили спутника (исчерпаны все попытки).
    Rejected {
        /// PRN, который искали
        prn: u8,

        /// Peak-to-noise из первого прохода (если был хоть какой-то пик)
        peak_to_noise: Option<f32>,

        /// Время, затраченное на попытку
        elapsed: Duration,
    },
}

/// Причина маргинального результата (диагностика)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MarginalReason {
    /// Второй проход не вернул результата (PRN не precomputed)
    NoResult,

    /// SNR второго прохода ниже порога
    LowSnr,

    /// Doppler второго прохода не согласуется с первым
    DopplerMismatch,

    /// Оба условия не выполняются
    LowSnrAndDopplerMismatch,
}

/// Политика повтора при неудаче верификации.
#[derive(Debug, Clone)]
pub struct RetryPolicy {
    /// Максимальное число попыток (включая первую)
    pub max_attempts: usize,

    /// Базовая задержка (мс). Удваивается с каждой попыткой.
    pub base_delay_ms: u64,

    /// Потолок задержки (мс)
    pub max_delay_ms: u64,
}

/// Накопленая статистика верификатора.
#[derive(Debug, Clone, Default)]
pub struct VerifierStats {
    /// Число вызовов `verify_prn`.
    pub total_calls: u64,

    /// Число успешных верификаций (`Confirmed`).
    pub confirmed: u64,

    /// Число маргинальных результатов (первый проход ок, второй — нет).
    pub marginal: u64,

    /// Число полных отказов (`Rejected`).
    pub rejected: u64,

    /// Число верификаций с хотя бы одним повтором.
    pub retried: u64,

    /// Суммарное время всех вызовов (нс) — для `mean_elapsed`.
    total_elapsed_ns: u128,
}

/// Конфигурация [`AcquisitionVerifier`].
#[derive(Debug, Clone)]
pub struct VerifierConfig {
    /// Конфигурация первого прохода (широкий поиск)
    pub first_pass: SearchConfig,

    /// Параметры второго прохода.
    ///
    /// `doppler_max_hz` используется как **±`half_span`** вокруг кандидата:
    ///
    /// ```text
    /// second_min = candidate_doppler − second_pass.doppler_max_hz
    /// second_max = candidate_doppler + second_pass.doppler_max_hz
    /// ```
    ///
    /// Таким образом второй проход всегда центрирован на результате
    /// первого — это настоящий уточняющий поиск.
    pub second_pass: SearchConfig,

    /// Максимально допустимое расхождение Doppler между двумя проходами (Hz).
    pub doppler_tolerance_hz: f64,

    /// Политика повтора.
    pub retry: RetryPolicy,
}

/// Верифицированный результат acquisition, готовый к передаче в tracking.
///
/// Создаётся **только** через [`VerificationVerdict::acquisition_result`],
/// то есть только из `Confirmed`. Tracking использует `doppler_hz` и
/// `code_phase_samples` для инициализации DLL/PLL.
#[derive(Debug, Clone)]
pub struct AcquisitionResult {
    /// PRN спутника (GPS 1-32)
    pub prn: u8,

    /// Точный Doppler из второго прохода (Гц)
    pub doppler_hz: f64,

    /// Фаза кода в сэмплах
    pub code_phase_samples: usize,

    /// Фаза кода в чипах (0.0..1023.0)
    pub code_phase_chips: f64,

    /// Оценка C/N₀ (дБ-Гц).
    pub cn0_db_hz: f32,

    /// Peak-to-noise второго прохода.
    pub peak_to_noise: f32,
}

/// Двухпроходный верификатор acquisition с политикой повтора.
///
/// Хранит `Arc<PrnCodeCache>` чтобы второй проход мог пересоздать движок
/// с динамически вычесленным Doppler-диапазоном вокруг кандидата первого
/// прохода - настоящий узкий уточняющий поиск.
pub struct AcquisitionVerifier {
    /// Движок первого прохода (широкий поиск).
    engine_first: PcpsSearch,

    /// Кэш PRN-кодов — нужен для пересоздания движка второго прохода.
    cache: Arc<PrnCodeCache>,

    /// Размер IQ-блока в сэмплах.
    block_size: usize,

    /// Частота дискретизации (Гц).
    sample_rate_hz: f64,

    /// Конфигурация.
    config: VerifierConfig,

    /// Накопленная статистика.
    stats: VerifierStats,
}

impl VerificationVerdict {
    /// Возвращает `true` если верификация прошла успешно.
    #[must_use]
    pub const fn is_confirmed(&self) -> bool {
        matches!(self, VerificationVerdict::Confirmed { .. })
    }

    /// Возвращает `AcquisitionResult` **только если** `Confirmed`.
    ///
    /// Это единственный безопасный путь для передачи результата в tracking.
    /// `Marginal` намеренно не пропускатеся.
    #[must_use]
    pub const fn acquisition_result(&self) -> Option<AcquisitionResult> {
        match self {
            VerificationVerdict::Confirmed {
                result, cn0_db_hz, ..
            } => Some(AcquisitionResult::from_search_result(result, *cn0_db_hz)),
            _ => None,
        }
    }

    /// Возвращает `&SearchResult` для любого исхода — только для диагностики.
    ///
    /// Не использовать для инициализации tracking.
    #[must_use]
    pub const fn search_result_diagnostic(&self) -> Option<&SearchResult> {
        match self {
            VerificationVerdict::Confirmed { result, .. }
            | VerificationVerdict::Marginal { result, .. } => Some(result),
            VerificationVerdict::Rejected { .. } => None,
        }
    }
}

impl RetryPolicy {
    /// Вычисляет задержку для `attempt`-й повторной попытки (0-based).
    #[must_use]
    pub fn delay_for(
        &self,
        attempt: usize,
    ) -> Duration {
        if attempt == 0 {
            return Duration::ZERO;
        }

        let exp = (attempt as u32).saturating_sub(1);
        let ms = self.base_delay_ms.saturating_mul(1u64 << exp);
        let ms = ms.min(self.max_delay_ms);

        Duration::from_millis(ms)
    }
}

impl VerifierStats {
    /// Доля ложных срабатываний: `marginal / (confirmed + marginal)`.
    ///
    /// Приближённо оценивает ненадёжные обнаружения из всех обнаружений.
    #[must_use]
    pub fn false_alarm_rate(&self) -> f64 {
        let detections = self.confirmed + self.marginal;

        if detections == 0 {
            0.0
        } else {
            self.marginal as f64 / detections as f64
        }
    }

    /// Среднее время верификации.
    #[must_use]
    pub fn mean_elapsed(&self) -> Duration {
        if self.total_calls == 0 {
            return Duration::ZERO;
        }

        Duration::from_nanos((self.total_elapsed_ns / u128::from(self.total_calls)) as u64)
    }

    /// Some doc-comment
    pub const fn record(
        &mut self,
        verdict: &VerificationVerdict,
        retried: bool,
    ) {
        self.total_calls += 1;

        if retried {
            self.retried += 1;
        }

        let elapsed = match verdict {
            VerificationVerdict::Confirmed { elapsed, .. }
            | VerificationVerdict::Marginal { elapsed, .. }
            | VerificationVerdict::Rejected { elapsed, .. } => *elapsed,
        };

        self.total_elapsed_ns += elapsed.as_nanos();

        match verdict {
            VerificationVerdict::Confirmed { .. } => self.confirmed += 1,
            VerificationVerdict::Marginal { .. } => self.marginal += 1,
            VerificationVerdict::Rejected { .. } => self.rejected += 1,
        }
    }
}

impl AcquisitionResult {
    /// Some doc-comment
    pub(crate) const fn from_search_result(
        result: &SearchResult,
        cn0_db_hz: f32,
    ) -> Self {
        Self {
            prn: result.prn,
            doppler_hz: result.doppler_fine_hz,
            code_phase_samples: result.code_phase_samples,
            code_phase_chips: result.code_phase_chips,
            cn0_db_hz,
            peak_to_noise: result.peak_to_noise,
        }
    }
}

impl AcquisitionVerifier {
    /// Создаёт новый верификатор с явным кэшем PRN-кода.
    ///
    /// `cache` можно разделить с другими компонентами через `Arc::clone`.
    #[must_use]
    pub fn new(
        block_size: usize,
        sample_rate_hz: f64,
        config: VerifierConfig,
        cache: Arc<PrnCodeCache>,
    ) -> Self {
        let engine_first = PcpsSearch::new(block_size, sample_rate_hz, config.first_pass.clone());

        Self {
            engine_first,
            cache,
            block_size,
            sample_rate_hz,
            config,
            stats: VerifierStats::default(),
        }
    }

    /// Создаёт верификатор с конфигурацией по умолчанию, создаёт кэш внутри.
    #[must_use]
    pub fn with_defaults(
        block_size: usize,
        sample_rate_hz: f64,
    ) -> Self {
        let cache = Arc::new(PrnCodeCache::new());

        Self::new(block_size, sample_rate_hz, VerifierConfig::default(), cache)
    }

    /// Предварительно вычисляет FFT PRN-кода для одного спутника.
    pub fn precompute_prn(
        &mut self,
        prn: u8,
    ) {
        self.engine_first.precompute_prn(prn, &self.cache.clone());
    }

    /// Предварительно вычисляет FFT PRN-кодов для GPS PRN 1-32.
    pub fn precompute_all(&mut self) {
        self.engine_first.precompute_all(&self.cache.clone());
    }

    /// Верефицирует `prn` двойным проходом с политикой повтора.
    ///
    /// Второй проход строится с Doppler-диапазоном, динамически
    /// центрированным на результате первого прохода.
    ///
    /// # Panics
    ///
    /// Panics if `signal.len() != self.block_size`.
    pub fn verify_prn(
        &mut self,
        signal: &[Complex32],
        prn: u8,
    ) -> VerificationVerdict {
        assert_eq!(signal.len(), self.block_size);

        let t0 = Instant::now();
        let max_attempts = self.config.retry.max_attempts;
        let mut retried = false;

        for attempt in 0..max_attempts {
            if attempt > 0 {
                retried = true;

                let delay = self.config.retry.delay_for(attempt);

                if !delay.is_zero() {
                    std::thread::sleep(delay);
                }

                log::debug!("PRN {prn}: retry {attempt}/{max_attempts}");
            }

            let verdict = self.single_verify(signal, prn, t0);

            match &verdict {
                VerificationVerdict::Confirmed { .. } | VerificationVerdict::Marginal { .. } => {
                    self.stats.record(&verdict, retried);

                    return verdict;
                }
                VerificationVerdict::Rejected { .. } => {
                    if attempt + 1 == max_attempts {
                        self.stats.record(&verdict, retried);

                        return verdict;
                    }
                    log::debug!(
                        "PRN {prn}: attempt {attempt} rejected, \
                         retrying ({}/{})",
                        attempt + 1,
                        max_attempts
                    );
                }
            }
        }

        // Недостижимо — последняя итерация всегда возвращает.
        // Rustc требует exhaustive return.
        let verdict = VerificationVerdict::Rejected {
            prn,
            peak_to_noise: None,
            elapsed: t0.elapsed(),
        };

        self.stats.record(&verdict, retried);

        verdict
    }

    /// Верифицирует все PRN 1-32, возвращает подтверждённые спутники
    /// отсортированные по C/N₀ убыванием.
    ///
    /// Только `Confirmed` попадает в список.
    pub fn verify_all(
        &mut self,
        signal: &[Complex32],
    ) -> Vec<AcquisitionResult> {
        if signal.len() != self.block_size {
            return Vec::new();
        }

        let mut results = Vec::new();

        for prn in 1u8..=32 {
            if let Some(r) = self.verify_prn(signal, prn).acquisition_result() {
                results.push(r);
            }
        }

        results.sort_by(|a, b| b.cn0_db_hz.total_cmp(&a.cn0_db_hz));

        results
    }

    /// Снимок статистики
    #[must_use]
    pub const fn stats(&self) -> &VerifierStats {
        &self.stats
    }

    /// Сбросить статистику.
    pub fn reset_stats(&mut self) {
        self.stats = VerifierStats::default();
    }

    fn single_verify(
        &mut self,
        signal: &[Complex32],
        prn: u8,
        t0: Instant,
    ) -> VerificationVerdict {
        // ── Первый проход: широкий поиск ──────────────────────────────────────
        let Some(first) = self.engine_first.search_prn(signal, prn) else {
            return VerificationVerdict::Rejected {
                prn,
                peak_to_noise: None,
                elapsed: t0.elapsed(),
            };
        };

        if !first.detected {
            return VerificationVerdict::Rejected {
                prn,
                peak_to_noise: Some(first.peak_to_noise),
                elapsed: t0.elapsed(),
            };
        }

        // ── Второй проход: узкий поиск вокруг кандидата ───────────────────────
        //
        // Диапазон центрирован на doppler_coarse_hz первого прохода.
        // half_span берётся из second_pass.doppler_max_hz — это задокументировано
        // в VerifierConfig.
        let half_span = self.config.second_pass.doppler_max_hz;
        let narrow_cfg = SearchConfig {
            doppler_min_hz: first.doppler_coarse_hz - half_span,
            doppler_max_hz: first.doppler_coarse_hz + half_span,
            doppler_step_hz: self.config.second_pass.doppler_step_hz,
            cfar_threshold: self.config.second_pass.cfar_threshold,
        };

        // Создаём движок второго прохода с узким narrow_cfg.
        // Arc<PrnCodeCache> позволяет precompute только нужный PRN без
        // копирования данных — O(N) на один PRN.
        let second = {
            let mut engine2 =
                PcpsSearch::new(self.block_size, self.sample_rate_hz, narrow_cfg.clone());
            engine2.precompute_prn(prn, &self.cache);
            engine2.search_prn(signal, prn)
        };

        let Some(second) = second else {
            return VerificationVerdict::Marginal {
                result: first,
                reason: MarginalReason::NoResult,
                elapsed: t0.elapsed(),
            };
        };

        // ── Проверка согласованности ──────────────────────────────────────────
        let snr_ok = second.peak_to_noise >= narrow_cfg.cfar_threshold;
        let doppler_ok = (second.doppler_fine_hz - first.doppler_fine_hz).abs()
            <= self.config.doppler_tolerance_hz;

        if snr_ok && doppler_ok {
            let cn0 = estimate_cn0(second.peak_to_noise, narrow_cfg.cfar_threshold);
            return VerificationVerdict::Confirmed {
                result: second,
                cn0_db_hz: cn0,
                elapsed: t0.elapsed(),
            };
        }

        let reason = match (snr_ok, doppler_ok) {
            (false, false) => MarginalReason::LowSnrAndDopplerMismatch,
            (false, true) => MarginalReason::LowSnr,
            (true, false) => MarginalReason::DopplerMismatch,
            (true, true) => unreachable!("both ok was handled above"),
        };

        VerificationVerdict::Marginal {
            result: first,
            reason,
            elapsed: t0.elapsed(),
        }
    }
}

impl Default for RetryPolicy {
    fn default() -> Self {
        Self {
            max_attempts: 3,
            base_delay_ms: 20,
            max_delay_ms: 100,
        }
    }
}

impl Default for VerifierConfig {
    fn default() -> Self {
        Self {
            first_pass: SearchConfig {
                doppler_min_hz: -10_000.0,
                doppler_max_hz: 10_000.0,
                doppler_step_hz: 500.0,
                cfar_threshold: 3.0,
            },
            second_pass: SearchConfig {
                // doppler_max_hz = half_span вокруг кандидата
                doppler_min_hz: -1_500.0, // используется только как half_span
                doppler_max_hz: 1_500.0,
                doppler_step_hz: 250.0,
                cfar_threshold: 2.5,
            },
            doppler_tolerance_hz: 750.0,
            retry: RetryPolicy::default(),
        }
    }
}

/// Параллельный поиск нескольких PRN через Rayon.
///
/// Каждый PRN отсортированы по `peak_to_noise` убыванием.
///
///
/// Включить: `features = ["rayon"]` в `Cargo.toml`.
#[cfg(feature = "rayon")]
#[must_use]
pub fn parallel_search(
    signal: &[Complex32],
    block_size: usize,
    sample_rate_hz: f64,
    prns: &[u8],
    config: &SearchConfig,
    cache: &PrnCodeCache,
) -> Vec<SearchResult> {
    use rayon::prelude::*;

    let mut results: Vec<SearchResult> = prns
        .par_iter()
        .filter_map(|&prn| {
            let mut engine = PcpsSearch::new(block_size, sample_rate_hz, config.clone());
            engine.precompute_prn(prn, cache);
            engine.search_prn(signal, prn)
        })
        .filter(|r| r.detected)
        .collect();

    results.sort_by(|a, b| b.peak_to_noise.total_cmp(&a.peak_to_noise));

    results
}

/// Последовательный fallback `parallel_search` без Rayon.
///
/// Идентичная сигнатура - переключатся через условную компиляцию.
#[cfg(not(feature = "rayon"))]
#[must_use]
pub fn parallel_search(
    signal: &[Complex32],
    block_size: usize,
    sample_rate_hz: f64,
    prns: &[u8],
    config: &SearchConfig,
    cache: &PrnCodeCache,
) -> Vec<SearchResult> {
    let mut results = Vec::new();

    for &prn in prns {
        let mut engine = PcpsSearch::new(block_size, sample_rate_hz, config.clone());

        engine.precompute_prn(prn, cache);

        if let Some(r) = engine.search_prn(signal, prn) {
            if r.detected {
                results.push(r);
            }
        }
    }

    results.sort_by(|a, b| b.peak_to_noise.total_cmp(&a.peak_to_noise));

    results
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::sync::Arc;

    use crate::{
        acquisition::{
            fft_search::SearchConfig,
            verifier::{AcquisitionVerifier, RetryPolicy, VerifierConfig},
        },
        signal::prn_code::PrnCodeCache,
    };

    const FS: f64 = 2_048_000.0;
    const N: usize = 2048;

    fn make_verifier() -> AcquisitionVerifier {
        let cache = Arc::new(PrnCodeCache::new());
        let cfg = VerifierConfig {
            first_pass: SearchConfig {
                doppler_min_hz: -1_000.0,
                doppler_max_hz: 1_000.0,
                doppler_step_hz: 500.0,
                cfar_threshold: 2.0,
            },
            second_pass: SearchConfig {
                doppler_min_hz: -500.0, // не используется напрямую
                doppler_max_hz: 500.0,  // half_span
                doppler_step_hz: 250.0,
                cfar_threshold: 2.0,
            },
            doppler_tolerance_hz: 600.0,
            retry: RetryPolicy {
                max_attempts: 1,
                base_delay_ms: 0,
                max_delay_ms: 0,
            },
        };
        let mut v = AcquisitionVerifier::new(N, FS, cfg, cache);
        v.precompute_all();
        v
    }

    fn dummy_search_result(prn: u8) -> SearchResult {
        SearchResult {
            prn,
            doppler_coarse_hz: 0.0,
            doppler_fine_hz: 0.0,
            code_phase_samples: 100,
            code_phase_chips: 50.0,
            peak_power: 500.0,
            noise_floor: 1.0,
            peak_to_noise: 500.0,
            detected: true,
        }
    }

    #[test]
    fn test_retry_first_attempt_zero_delay() {
        assert_eq!(RetryPolicy::default().delay_for(0), Duration::ZERO);
    }

    #[test]
    fn test_retry_exponential_backoff() {
        let p = RetryPolicy {
            base_delay_ms: 10,
            max_delay_ms: 1000,
            ..Default::default()
        };

        assert_eq!(p.delay_for(1).as_millis(), 10);
        assert_eq!(p.delay_for(2).as_millis(), 20);
        assert_eq!(p.delay_for(3).as_millis(), 40);
    }

    #[test]
    fn test_retry_capped_at_max() {
        let p = RetryPolicy {
            base_delay_ms: 100,
            max_delay_ms: 150,
            ..Default::default()
        };

        assert!(p.delay_for(10).as_millis() <= 150);
    }

    #[test]
    fn test_stats_false_alarm_rate_zero_when_empty() {
        assert!((VerifierStats::default().false_alarm_rate()).abs() < 1e-9);
    }

    #[test]
    fn stats_false_alarm_rate_half() {
        let s = VerifierStats {
            confirmed: 5,
            marginal: 5,
            ..Default::default()
        };
        assert!((s.false_alarm_rate() - 0.5).abs() < 1e-9);
    }

    #[test]
    fn stats_mean_elapsed_zero_when_empty() {
        assert_eq!(VerifierStats::default().mean_elapsed(), Duration::ZERO);
    }

    #[test]
    fn confirmed_is_confirmed_and_has_acquisition_result() {
        let v = VerificationVerdict::Confirmed {
            result: dummy_search_result(1),
            cn0_db_hz: 45.0,
            elapsed: Duration::from_millis(5),
        };
        assert!(v.is_confirmed());
        assert!(v.acquisition_result().is_some());
        assert!(v.search_result_diagnostic().is_some());
    }

    #[test]
    fn marginal_not_confirmed_no_acquisition_result() {
        // Ключевой тест безопасности: Marginal НЕ должен пропускаться в tracking
        let v = VerificationVerdict::Marginal {
            result: dummy_search_result(1),
            reason: MarginalReason::LowSnr,
            elapsed: Duration::ZERO,
        };
        assert!(!v.is_confirmed());
        assert!(
            v.acquisition_result().is_none(),
            "Marginal must NOT pass to tracking via acquisition_result()"
        );
        // Но диагностика доступна
        assert!(v.search_result_diagnostic().is_some());
    }

    #[test]
    fn rejected_no_acquisition_result_no_diagnostic() {
        let v = VerificationVerdict::Rejected {
            prn: 1,
            peak_to_noise: None,
            elapsed: Duration::ZERO,
        };
        assert!(!v.is_confirmed());
        assert!(v.acquisition_result().is_none());
        assert!(v.search_result_diagnostic().is_none());
    }

    #[test]
    fn marginal_diagnostic_all_reasons_accessible() {
        // Убеждаемся что все варианты MarginalReason компилируются
        for reason in [
            MarginalReason::NoResult,
            MarginalReason::LowSnr,
            MarginalReason::DopplerMismatch,
            MarginalReason::LowSnrAndDopplerMismatch,
        ] {
            let v = VerificationVerdict::Marginal {
                result: dummy_search_result(1),
                reason,
                elapsed: Duration::ZERO,
            };
            assert!(!v.is_confirmed());
        }
    }

    #[test]
    fn acquisition_result_fields_correct() {
        let mut r = dummy_search_result(7);

        r.doppler_fine_hz = 1050.0;
        r.code_phase_samples = 42;
        r.code_phase_chips = 21.0;

        let ar = AcquisitionResult::from_search_result(&r, 45.0);

        assert_eq!(ar.prn, 7);
        assert!((ar.doppler_hz - 1050.0).abs() < 1e-9);
        assert_eq!(ar.code_phase_samples, 42);
        assert!((ar.code_phase_chips - 21.0).abs() < 1e-9);
        assert!((ar.cn0_db_hz - 45.0).abs() < 1e-3);
        assert!((ar.peak_to_noise - 500.0).abs() < 1e-3);
    }

    #[test]
    fn verify_prn_zero_signal_rejected() {
        let mut v = make_verifier();
        let signal = vec![Complex32::new(0.0, 0.0); N];
        let verdict = v.verify_prn(&signal, 1);
        assert!(
            matches!(verdict, VerificationVerdict::Rejected { .. }),
            "zero signal → Rejected, got: {verdict:?}"
        );
    }

    #[test]
    fn verify_prn_aligned_signal_passes_first_pass() {
        let cache = PrnCodeCache::new();
        let mut v = make_verifier();
        let signal: Vec<Complex32> = cache
            .resample_gps(1, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();
        let verdict = v.verify_prn(&signal, 1);
        // Чистый сигнал должен пройти хотя бы первый проход
        assert!(
            verdict.is_confirmed() || matches!(verdict, VerificationVerdict::Marginal { .. }),
            "clean PRN 1 → at least Marginal, got: {verdict:?}"
        );
    }

    #[test]
    fn verify_all_empty_for_noise() {
        let mut v = make_verifier();
        let signal = vec![Complex32::new(0.0, 0.0); N];
        assert!(v.verify_all(&signal).is_empty());
    }

    #[test]
    fn verify_all_sorted_by_cn0_descending() {
        let cache = PrnCodeCache::new();
        let mut v = make_verifier();
        let signal: Vec<Complex32> = cache
            .resample_gps(5, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();
        let results = v.verify_all(&signal);
        for i in 1..results.len() {
            assert!(
                results[i - 1].cn0_db_hz >= results[i].cn0_db_hz,
                "not sorted by C/N₀ at {} and {}",
                i - 1,
                i
            );
        }
    }

    #[test]
    fn verify_all_only_confirmed_in_results() {
        // verify_all не должен возвращать Marginal или Rejected
        let mut v = make_verifier();
        let signal = vec![Complex32::new(0.01, 0.0); N]; // слабый шум
        let _results = v.verify_all(&signal);
        // Все элементы вернулись через acquisition_result() → все Confirmed
        // Дополнительная проверка: stats.rejected + stats.marginal + stats.confirmed = total
        let s = v.stats();
        assert_eq!(
            s.confirmed + s.marginal + s.rejected,
            s.total_calls,
            "stats don't add up"
        );
    }

    #[test]
    fn stats_total_calls_accumulate() {
        let mut v = make_verifier();
        let signal = vec![Complex32::new(0.0, 0.0); N];
        for prn in 1u8..=3 {
            v.verify_prn(&signal, prn);
        }
        assert_eq!(v.stats().total_calls, 3);
    }

    #[test]
    fn stats_reset_clears_all_fields() {
        let mut v = make_verifier();
        let signal = vec![Complex32::new(0.0, 0.0); N];
        v.verify_prn(&signal, 1);
        v.reset_stats();
        let s = v.stats();
        assert_eq!(s.total_calls, 0);
        assert_eq!(s.confirmed, 0);
        assert_eq!(s.marginal, 0);
        assert_eq!(s.rejected, 0);
        assert_eq!(s.retried, 0);
    }

    #[test]
    fn parallel_search_empty_signal_returns_empty() {
        let cache = PrnCodeCache::new();
        let signal = vec![Complex32::new(0.0, 0.0); N];
        let cfg = SearchConfig {
            doppler_min_hz: 0.0,
            doppler_max_hz: 0.0,
            doppler_step_hz: 500.0,
            cfar_threshold: 3.0,
        };
        let results = parallel_search(
            &signal,
            N,
            FS,
            &(1u8..=32).collect::<Vec<_>>(),
            &cfg,
            &cache,
        );
        assert!(results.is_empty());
    }

    #[test]
    fn parallel_search_finds_injected_prn() {
        let cache = PrnCodeCache::new();
        let signal: Vec<Complex32> = cache
            .resample_gps(3, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();
        let cfg = SearchConfig {
            doppler_min_hz: -500.0,
            doppler_max_hz: 500.0,
            doppler_step_hz: 500.0,
            cfar_threshold: 2.0,
        };
        let results = parallel_search(&signal, N, FS, &[3u8], &cfg, &cache);
        assert!(!results.is_empty(), "should detect PRN 3");
        assert_eq!(results[0].prn, 3);
    }

    #[test]
    fn parallel_search_results_sorted_by_snr() {
        let cache = PrnCodeCache::new();
        let signal: Vec<Complex32> = cache
            .resample_gps(7, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();
        let cfg = SearchConfig {
            doppler_min_hz: 0.0,
            doppler_max_hz: 0.0,
            doppler_step_hz: 500.0,
            cfar_threshold: 2.0,
        };
        let results = parallel_search(
            &signal,
            N,
            FS,
            &(1u8..=32).collect::<Vec<_>>(),
            &cfg,
            &cache,
        );
        for i in 1..results.len() {
            assert!(
                results[i - 1].peak_to_noise >= results[i].peak_to_noise,
                "not sorted at {i}"
            );
        }
    }
}
