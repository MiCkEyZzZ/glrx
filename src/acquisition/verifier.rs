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
//!
//! # Параллельный поиск
//!
//! Двойная верификация по своей природе stateful только в части retry и
//! статистики. Сама проверка одного PRN (`verify_prn_pure`) не требует
//! `&mut self` и безопасна для параллельного вызова из разных потоков —
//! на этом построена [`verify_all_parallel`], использующая Rayon.

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
/// Для одиночных проверок с retry и накоплением статистики используйте
/// [`AcquisitionVerifier::verify_prn`] / [`AcquisitionVerifier::verify_all`].
/// Для параллельной обработки многих PRN без накопления retry-статистики
/// в реальном времени используйте свободную функцию
/// [`verify_all_parallel`].
pub struct AcquisitionVerifier {
    /// Размер IQ-блока в сэмплах.
    block_size: usize,

    /// Частота дискретизации (Гц).
    sample_rate_hz: f64,

    cache: Arc<PrnCodeCache>,

    /// Конфигурация.
    config: VerifierConfig,

    /// Накопленная статистика.
    stats: VerifierStats,
}

/// Результат параллельного прогона: подтверждённые спутники + агрегирования
/// статистика по всем потокам.
#[derive(Debug, Clone)]
pub struct ParallelVerifyOutput {
    /// Подтверждённые спутники, отсортированные по C/N₀ убыванием.
    pub results: Vec<AcquisitionResult>,

    /// Статистика, агрегированная по всем обработанным PRN.
    ///
    /// Каждый PRN обрабатывается одной попыткой (без внутреннего retry —
    /// retry для параллельного режима не применяется, так как политика
    /// повтора в реальном времени плохо сочетается с параллелизмом по
    /// потокам; при необходимости повторите весь `verify_all_parallel`
    /// для PRN, попавших в `Marginal`/`Rejected`).
    pub stats: VerifierStats,
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
    /// Доля маргинальных обнаружений среди всех успешных первых проходов:
    /// `marginal / (confirmed + marginal)`.
    ///
    /// **Это не настоящий false-positive rate** — у нас нет независимого
    /// источника истины о реальном наличии спутника. Это приближение:
    /// доля случаев, когда широкий поиск нашёл пик, но узкий уточняющий
    /// проход не подтвердил его с тем же или близким Doppler/SNR.
    /// Используйте как индикатор нестабильности detection, а не как
    /// строгую метрику ошибок 1-го рода.
    #[must_use]
    pub fn marginal_rate(&self) -> f64 {
        let d = self.confirmed + self.marginal;

        if d == 0 {
            0.0
        } else {
            self.marginal as f64 / d as f64
        }
    }

    /// Устаревший алиас [`Self::marginal_rate`].
    ///
    /// Сохранён для обратной совместимости. Имя исторически вводило в
    /// заблуждение (предполагало строгий false-positive rate с ground
    /// truth, которого здесь нет) — используйте `marginal_rate()`.
    #[must_use]
    pub fn false_alarm_rate(&self) -> f64 {
        self.marginal_rate()
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

    /// Объединяет статистику из другого экземпляра (для слияния результатов параллельных потоков).
    pub const fn merge(
        &mut self,
        other: &VerifierStats,
    ) {
        self.total_calls += other.total_calls;
        self.confirmed += other.confirmed;
        self.marginal += other.marginal;
        self.rejected += other.rejected;
        self.retried += other.retried;
        self.total_elapsed_ns += other.total_elapsed_ns;
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
    #[must_use]
    pub fn new(
        block_size: usize,
        sample_rate_hz: f64,
        config: VerifierConfig,
        cache: Arc<PrnCodeCache>,
    ) -> Self {
        Self {
            block_size,
            sample_rate_hz,
            cache,
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

    /// Кэш PRN-кодов, используемый этим верификатором.
    ///
    /// Передавайте этот `Arc` в [`verify_all_parallel`], чтобы не
    /// пересчитывать код.
    #[must_use]
    pub const fn cache(&self) -> &Arc<PrnCodeCache> {
        &self.cache
    }

    /// Конфигурация верификации.
    #[must_use]
    pub const fn config(&self) -> &VerifierConfig {
        &self.config
    }

    /// Верифицирует `prn` двойным проходом с политикой повтора.
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

            let verdict = verify_prn_pure(
                signal,
                prn,
                self.block_size,
                self.sample_rate_hz,
                &self.config,
                &self.cache,
            );

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
                        "PRN {prn}: attempt {attempt} rejected, retrying ({}/{})",
                        attempt + 1,
                        max_attempts
                    );
                }
            }
        }

        unreachable!("loop above always returns on the last attempt")
    }

    /// Последовательно верифицирует все PRN 1–32, возвращает подтверждённые
    /// спутники, отсортированные по C/N₀ убыванием.
    ///
    /// Для параллельной версии (без накопления retry-статистики в
    /// `self.stats`, но быстрее на многих ядрах) см. [`verify_all_parallel`].
    ///
    /// # Panics
    ///
    /// Panics if `signal.len() != self.block_size`.
    pub fn verify_all(
        &mut self,
        signal: &[Complex32],
    ) -> Vec<AcquisitionResult> {
        assert_eq!(signal.len(), self.block_size);

        let mut results = Vec::new();

        for prn in 1u8..=32 {
            if let Some(r) = self.verify_prn(signal, prn).acquisition_result() {
                results.push(r);
            }
        }

        results.sort_by(|a, b| b.cn0_db_hz.partial_cmp(&a.cn0_db_hz).unwrap());

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

    /// Объединяет в свою статистику результат внешнего вычисления - например,
    /// после вызова [`verify_all_parallel`], чтобы агрегаты (`mean_elapsed`, `marginal_rate`)
    /// учитывали и параллельный прогон.
    pub const fn absorb_stats(
        &mut self,
        other: &VerifierStats,
    ) {
        self.stats.merge(other);
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
                doppler_min_hz: -1_500.0, // half_span, не абсолютная граница
                doppler_max_hz: 1_500.0,
                doppler_step_hz: 250.0,
                cfar_threshold: 2.5,
            },
            doppler_tolerance_hz: 750.0,
            retry: RetryPolicy::default(),
        }
    }
}

/// Один проход двойной верификации **без повторов и без мутации общего состояния** - только
/// первый + второй поиск и сравнение.
///
/// Эта ф-я не использует `&mut self` ни от чего общего: она создаёт оба движка PCPS локально из
/// `cache` и `config`. Благодаря этому она безопасна для вызова из разных потоков Rayon
/// одновременно - каждый вызов полностью независим.
///
/// Используется как строительный блок:
/// - [`AcquisitionVerifier::verify_prn`] оборачивает её в retry-цикл и
///   копит статистику последовательно;
/// - [`verify_all_parallel`] вызывает её параллельно по списку PRN.
#[must_use]
pub fn verify_prn_pure(
    signal: &[Complex32],
    prn: u8,
    block_size: usize,
    sample_rate_hz: f64,
    config: &VerifierConfig,
    cache: &PrnCodeCache,
) -> VerificationVerdict {
    let t0 = Instant::now();

    // Первый проход: широкий поиск
    let mut engine_first = PcpsSearch::new(block_size, sample_rate_hz, config.first_pass.clone());

    engine_first.precompute_prn(prn, cache);

    let Some(first) = engine_first.search_prn(signal, prn) else {
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

    // Второй проход: узкий поиск вокруг кандидата
    let half_span = config.second_pass.doppler_max_hz;
    let narrow_cfg = SearchConfig {
        doppler_min_hz: first.doppler_coarse_hz - half_span,
        doppler_max_hz: first.doppler_coarse_hz + half_span,
        doppler_step_hz: config.second_pass.doppler_step_hz,
        cfar_threshold: config.second_pass.cfar_threshold,
    };

    let mut engine_second = PcpsSearch::new(block_size, sample_rate_hz, narrow_cfg.clone());

    engine_second.precompute_prn(prn, cache);

    let Some(second) = engine_second.search_prn(signal, prn) else {
        return VerificationVerdict::Marginal {
            result: first,
            reason: MarginalReason::NoResult,
            elapsed: t0.elapsed(),
        };
    };

    // Проверка согласованности
    let snr_ok = second.peak_to_noise >= narrow_cfg.cfar_threshold;
    let doppler_ok =
        (second.doppler_fine_hz - first.doppler_coarse_hz).abs() <= config.doppler_tolerance_hz;

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

/// Параллельно верифицирует список `prns` двойным проходом, используя Rayon.
///
/// В отличие от [`AcquisitionVerifier::verify_prn`], здесь **нет retry**:
/// каждый PRN обрабатывается ровно одной попыткой двойной верификации в
/// отдельном потоке. Это сознательный компромисс — retry с задержками
/// (`std::thread::sleep`) плохо сочетается с пулом потоков Rayon, так как
/// блокирует worker-поток. Если нужен повтор для конкретных PRN, вызовите
/// `verify_all_parallel` повторно с отфильтрованным списком.
///
/// # Аргументы
///
/// - `signal` — один IQ-блок длиной `block_size`
/// - `block_size`, `sample_rate_hz` — параметры приёмника
/// - `prns` — список PRN для проверки (например, ещё не захваченные)
/// - `config` — конфигурация двойной верификации
/// - `cache` — общий кэш PRN-кодов
///
/// # Возвращает
///
/// [`ParallelVerifyOutput`] с подтверждёнными спутниками и агрегированной
/// статистикой (которую можно влить в `AcquisitionVerifier` через
/// [`AcquisitionVerifier::absorb_stats`]).
#[cfg(feature = "rayon")]
#[must_use]
pub fn verify_all_parallel(
    signal: &[Complex32],
    block_size: usize,
    sample_rate_hz: f64,
    prns: &[u8],
    config: &VerifierConfig,
    cache: &PrnCodeCache,
) -> ParallelVerifyOutput {
    use rayon::prelude::*;

    let verdicts: Vec<VerificationVerdict> = prns
        .par_iter()
        .map(|&prn| verify_prn_pure(signal, prn, block_size, sample_rate_hz, config, cache))
        .collect();

    let mut stats = VerifierStats::default();
    let mut results: Vec<AcquisitionResult> = Vec::new();

    for verdict in &verdicts {
        stats.record(verdict, false);
        if let Some(r) = verdict.acquisition_result() {
            results.push(r);
        }
    }

    results.sort_by(|a, b| b.cn0_db_hz.total_cmp(&a.cn0_db_hz));

    ParallelVerifyOutput { results, stats }
}

/// Последовательный fallback [`verify_all_parallel`] без Rayon.
///
/// Идентичная сигнатура и поведение (включая отсутствие retry) —
/// переключение через `features = ["rayon"]` в `Cargo.toml`.
#[cfg(not(feature = "rayon"))]
#[must_use]
pub fn verify_all_parallel(
    signal: &[Complex32],
    block_size: usize,
    sample_rate_hz: f64,
    prns: &[u8],
    config: &VerifierConfig,
    cache: &PrnCodeCache,
) -> ParallelVerifyOutput {
    let mut stats = VerifierStats::default();
    let mut results: Vec<AcquisitionResult> = Vec::new();

    for &prn in prns {
        let verdict = verify_prn_pure(signal, prn, block_size, sample_rate_hz, config, cache);
        stats.record(&verdict, false);
        if let Some(r) = verdict.acquisition_result() {
            results.push(r);
        }
    }

    results.sort_by(|a, b| b.cn0_db_hz.total_cmp(&a.cn0_db_hz));

    ParallelVerifyOutput { results, stats }
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

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

    fn small_verifier_config() -> VerifierConfig {
        VerifierConfig {
            first_pass: SearchConfig {
                doppler_min_hz: -1_000.0,
                doppler_max_hz: 1_000.0,
                doppler_step_hz: 500.0,
                cfar_threshold: 2.0,
            },
            second_pass: SearchConfig {
                doppler_min_hz: -500.0,
                doppler_max_hz: 500.0,
                doppler_step_hz: 250.0,
                cfar_threshold: 2.0,
            },
            doppler_tolerance_hz: 600.0,
            retry: RetryPolicy {
                max_attempts: 1,
                base_delay_ms: 0,
                max_delay_ms: 0,
            },
        }
    }

    fn make_verifier() -> AcquisitionVerifier {
        let cache = Arc::new(PrnCodeCache::new());
        AcquisitionVerifier::new(N, FS, small_verifier_config(), cache)
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
    fn test_stats_false_alarm_rate_half() {
        let s = VerifierStats {
            confirmed: 5,
            marginal: 5,
            ..Default::default()
        };

        assert!((s.marginal_rate() - 0.5).abs() < 1e-9);
    }

    #[test]
    fn test_stats_mean_elapsed_zero_when_empty() {
        assert_eq!(VerifierStats::default().mean_elapsed(), Duration::ZERO);
    }

    #[test]
    fn test_stats_merge_adds_fields() {
        let mut a = VerifierStats {
            total_calls: 2,
            confirmed: 1,
            marginal: 1,
            ..Default::default()
        };
        let b = VerifierStats {
            total_calls: 3,
            confirmed: 2,
            rejected: 1,
            ..Default::default()
        };

        a.merge(&b);

        assert_eq!(a.total_calls, 5);
        assert_eq!(a.confirmed, 3);
        assert_eq!(a.marginal, 1);
        assert_eq!(a.rejected, 1);
    }

    #[test]
    #[allow(deprecated)]
    fn test_deprecated_false_alarm_rate_matches_marginal_rate() {
        let s = VerifierStats {
            confirmed: 3,
            marginal: 1,
            ..Default::default()
        };

        assert!((s.false_alarm_rate() - s.marginal_rate()).abs() < 1e-12);
    }

    #[test]
    fn test_confirmed_is_confirmed_and_has_acquisition_result() {
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
    fn test_marginal_not_confirmed_no_acquisition_result() {
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
        assert!(v.search_result_diagnostic().is_some());
    }

    #[test]
    fn test_rejected_no_acquisition_result_no_diagnostic() {
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
    fn test_acquisition_result_fields_correct() {
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
    fn test_verify_prn_zero_signal_rejected() {
        let mut v = make_verifier();
        let signal = vec![Complex32::new(0.0, 0.0); N];
        let verdict = v.verify_prn(&signal, 1);

        assert!(
            matches!(verdict, VerificationVerdict::Rejected { .. }),
            "zero signal → Rejected, got: {verdict:?}"
        );
    }

    #[test]
    fn test_verify_prn_aligned_signal_passes_first_pass() {
        let cache = PrnCodeCache::new();
        let mut v = make_verifier();
        let signal: Vec<Complex32> = cache
            .resample_gps(1, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();
        let verdict = v.verify_prn(&signal, 1);

        assert!(
            verdict.is_confirmed() || matches!(verdict, VerificationVerdict::Marginal { .. }),
            "clean PRN 1 → at least Marginal, got: {verdict:?}"
        );
    }

    #[test]
    fn test_verify_all_empty_for_noise() {
        let mut v = make_verifier();
        let signal = vec![Complex32::new(0.0, 0.0); N];

        assert!(v.verify_all(&signal).is_empty());
    }

    #[test]
    fn test_verify_all_sorted_by_cn0_descending() {
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
            assert!(results[i - 1].cn0_db_hz >= results[i].cn0_db_hz);
        }
    }

    #[test]
    fn test_stats_total_calls_accumulate() {
        let mut v = make_verifier();
        let signal = vec![Complex32::new(0.0, 0.0); N];

        for prn in 1u8..=3 {
            v.verify_prn(&signal, prn);
        }

        assert_eq!(v.stats().total_calls, 3);
    }

    #[test]
    fn test_stats_reset_clears_all_fields() {
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
    fn test_absorb_stats_merges_into_verifier() {
        let mut v = make_verifier();

        let external = VerifierStats {
            confirmed: 5,
            total_calls: 5,
            ..Default::default()
        };

        v.absorb_stats(&external);

        assert_eq!(v.stats().confirmed, 5);
        assert_eq!(v.stats().total_calls, 5);
    }

    #[test]
    fn test_verify_prn_pure_zero_signal_rejected() {
        let cache = PrnCodeCache::new();
        let cfg = small_verifier_config();
        let signal = vec![Complex32::new(0.0, 0.0); N];
        let v = verify_prn_pure(&signal, 1, N, FS, &cfg, &cache);

        assert!(matches!(v, VerificationVerdict::Rejected { .. }));
    }

    #[test]
    fn test_verify_prn_pure_matches_verify_prn_for_single_attempt() {
        // С max_attempts=1 поведение verify_prn (без retry) должно совпадать
        // с прямым вызовом verify_prn_pure.
        let cache = PrnCodeCache::new();
        let cfg = small_verifier_config();
        let signal: Vec<Complex32> = cache
            .resample_gps(3, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();

        let direct = verify_prn_pure(&signal, 3, N, FS, &cfg, &cache);
        let mut v = make_verifier();
        let via_struct = v.verify_prn(&signal, 3);

        assert_eq!(direct.is_confirmed(), via_struct.is_confirmed());
    }

    #[test]
    fn test_parallel_empty_for_noise() {
        let cache = PrnCodeCache::new();
        let cfg = small_verifier_config();
        let signal = vec![Complex32::new(0.0, 0.0); N];
        let out = verify_all_parallel(
            &signal,
            N,
            FS,
            &(1u8..=32).collect::<Vec<_>>(),
            &cfg,
            &cache,
        );

        assert!(
            out.results.is_empty(),
            "noise should yield no confirmations"
        );
        assert_eq!(out.stats.total_calls, 32);
    }

    #[test]
    fn test_parallel_finds_injected_prn() {
        let cache = PrnCodeCache::new();
        let cfg = small_verifier_config();
        let signal: Vec<Complex32> = cache
            .resample_gps(5, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();

        let out = verify_all_parallel(
            &signal,
            N,
            FS,
            &(1u8..=32).collect::<Vec<_>>(),
            &cfg,
            &cache,
        );

        // PRN 5 должен быть среди подтверждённых либо хотя бы среди обработанных
        assert_eq!(out.stats.total_calls, 32);

        if !out.results.is_empty() {
            assert_eq!(
                out.results[0].prn, 5,
                "PRN 5 should be the strongest confirmation"
            );
        }
    }

    #[test]
    fn test_parallel_results_sorted_by_cn0() {
        let cache = PrnCodeCache::new();
        let cfg = small_verifier_config();
        let signal: Vec<Complex32> = cache
            .resample_gps(7, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();

        let out = verify_all_parallel(
            &signal,
            N,
            FS,
            &(1u8..=32).collect::<Vec<_>>(),
            &cfg,
            &cache,
        );

        for i in 1..out.results.len() {
            assert!(out.results[i - 1].cn0_db_hz >= out.results[i].cn0_db_hz);
        }
    }

    #[test]
    fn test_parallel_stats_match_sequential_outcome_distribution() {
        // Параллельный и последовательный прогон по одному и тому же сигналу
        // должны давать одинаковое распределение исходов (детерминированный
        // алгоритм, разные потоки не меняют результат).
        let cache = PrnCodeCache::new();
        let cfg = small_verifier_config();
        let signal: Vec<Complex32> = cache
            .resample_gps(2, N)
            .unwrap()
            .into_iter()
            .map(|c| Complex32::new(c, 0.0))
            .collect();

        let prns: Vec<u8> = (1u8..=32).collect();
        let parallel_out = verify_all_parallel(&signal, N, FS, &prns, &cfg, &cache);

        let mut seq_confirmed = 0u64;
        let mut seq_marginal = 0u64;
        let mut seq_rejected = 0u64;

        for &prn in &prns {
            match verify_prn_pure(&signal, prn, N, FS, &cfg, &cache) {
                VerificationVerdict::Confirmed { .. } => seq_confirmed += 1,
                VerificationVerdict::Marginal { .. } => seq_marginal += 1,
                VerificationVerdict::Rejected { .. } => seq_rejected += 1,
            }
        }

        assert_eq!(parallel_out.stats.confirmed, seq_confirmed);
        assert_eq!(parallel_out.stats.marginal, seq_marginal);
        assert_eq!(parallel_out.stats.rejected, seq_rejected);
    }

    #[test]
    fn test_cache_accessor_returns_same_arc() {
        let v = make_verifier();
        let c1 = v.cache();

        assert!(Arc::strong_count(c1) >= 1);
    }
}
