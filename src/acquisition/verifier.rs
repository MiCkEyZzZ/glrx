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

use std::{sync::Arc, time::Duration};

#[cfg(not(feature = "rayon"))]
use num_complex::Complex32;
#[cfg(feature = "rayon")]
use num_complex::Complex32;

use crate::{
    acquisition::fft_search::{PcpsSearch, SearchConfig, SearchResult},
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
    pub total_attempts: u64,

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
    /// `doppler_max_hz` используется как **±half_span** вокруг кандидата:
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
    pub fn is_confirmed(&self) -> bool {
        matches!(self, VerificationVerdict::Confirmed { .. })
    }

    /// Возвращает `AcquisitionResult` **только если** `Confirmed`.
    ///
    /// Это единственный безопасный путь для передачи результата в tracking.
    /// `Marginal` намеренно не пропускатеся.
    #[must_use]
    pub fn acquisition_result(&self) -> Option<AcquisitionResult> {
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
    pub fn search_result(&self) -> Option<&SearchResult> {
        match self {
            VerificationVerdict::Confirmed { result, .. } => Some(result),
            VerificationVerdict::Marginal { result, .. } => Some(result),
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
            return 0.0;
        }

        self.marginal as f64 / detections as f64
    }

    /// Среднее время верификации.
    #[must_use]
    pub fn mean_elapsed(&self) -> Duration {
        if self.total_attempts == 0 {
            return Duration::ZERO;
        }

        Duration::from_nanos((self.total_elapsed_ns / self.total_attempts as u128) as u64)
    }

    ///
    pub fn record(
        &mut self,
        verdict: &VerificationVerdict,
        retried: bool,
    ) {
        self.total_attempts += 1;

        if retried {
            self.retried += 1;
        }

        let elapsed = match verdict {
            VerificationVerdict::Confirmed { elapsed, .. } => *elapsed,
            VerificationVerdict::Marginal { elapsed, .. } => *elapsed,
            VerificationVerdict::Rejected { elapsed, .. } => *elapsed,
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
    ///
    pub(crate) fn from_search_result(
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
pub fn parallel_search(
    signal: &[Complex32],
    block_size: usize,
    sample_rate_hz: f64,
    prns: &[u8],
    config: SearchConfig,
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

    results.sort_by(|a, b| b.peak_to_noise.partial_cmp(&a.peak_to_noise).unwrap());

    results
}

/// Последовательный fallback `parallel_search` без Rayon.
///
/// Идентичная сигнатура - переключатся через условную компиляцию.
#[cfg(not(feature = "rayon"))]
pub fn parallel_search(
    signal: &[Complex32],
    block_size: usize,
    sample_rate_hz: f64,
    prns: &[u8],
    config: SearchConfig,
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

    results.sort_by(|a, b| b.peak_to_noise.partial_cmp(&a.peak_to_noise).unwrap());

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

    fn _make_verifier() -> AcquisitionVerifier {
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

    fn _dummy_search_result(prn: u8) -> SearchResult {
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
}
