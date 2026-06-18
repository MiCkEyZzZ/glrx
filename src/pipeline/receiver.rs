//! High-level receiver pipeline - оркестрация IQ -> Acquisition -> Tracking.
//!
//! # State machine
//!
//! ```text
//! ColdStart
//!     │  IQ-поток доступен
//!     ▼
//! Acquiring
//!     │  ≥ 1 спутник подтверждён
//!     ▼
//! Tracking
//!     │  эфемериды декодированы (будущая фаза)
//!     ▼
//! Navigating
//!     │  ≥ 4 наблюдаемых (будущая фаза)
//!     ▼
//! Fixed
//! ```
//!
//! # Интеграция с acquisition
//!
//! `Receiver` использует [`AcquisitionVerifier`] передаётся через [`ReceiverEvent::SatelliteAcquired`] во внешний обработчик
//! (tracking-каналы, навигационный движок).
//!
//! # Пример
//!
//! ```no_run
//! use std::sync::Arc;
//! use glrx::{
//!     pipeline::receiver::{Receiver, ReceiverConfig},
//!     rf::{config::RfConfig, file::FileSource, format::SampleFormat},
//!     signal::prn_code::PrnCodeCache,
//! };
//!
//! let rf_cfg = RfConfig::default();
//! let source = FileSource::open("gps_l1.bin", rf_cfg.clone()).unwrap();
//! let cache = Arc::new(PrnCodeCache::new());
//!
//! let mut rx = Receiver::new(ReceiverConfig::default(), Box::new(source), &cache);
//!
//! rx.run_epoch(); // один цикл: читает блок, ищет спутникиЮ эмитирует события
//! ```

use std::{sync::Arc, time::Instant};

use num_complex::Complex32;

use crate::{
    acquisition::verifier::{AcquisitionResult, AcquisitionVerifier, VerifierConfig},
    pipeline,
    rf::{error::RfError, iq_source::IqSource},
    signal::prn_code::PrnCodeCache,
};

/// Состояние ресивера.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReceiverState {
    /// Начальное состояние. IQ-поток ещё не обрабатывается
    ColdStart,

    /// Идёт поиск спутников (PCPS + верификация)
    Acquiring,

    /// Хотя бы один спутник захвачен, tracking активен
    Tracking,

    /// Эфемериды декодированы, вычисляются наблюдаемые
    Navigating,

    /// Позиционное решение доступно (>= 4 спутников)
    Fixed,
}

/// Событие внутренней шины ресивера.
///
/// Обработчик событий (`event_handler` в [`ReceiverConfig`]) получает
/// эти события для инициализации tracking-каналов, логирования, телеметрии.
#[derive(Debug, Clone)]
pub enum ReceiverEvent {
    /// Состояние ресивера изменилась
    StateChanged {
        /// Предыдущее состояние
        from: ReceiverState,

        /// Новое состояние
        to: ReceiverState,
    },
    /// Спутник успешно прошёл двоёную верификацию
    SatelliteAcquired {
        /// Верифицированный результат acquisition
        result: AcquisitionResult,
    },
    /// Спутник потерян (tracking-канал закрыт)
    SatelliteLost {
        /// PRN потерянного спутника
        prn: u8,
    },
    /// Acquisition-эпоха завершена (диагностика).
    AcquisitionEpochDone {
        /// Число проверенных PRN за эпоху
        prns_searched: usize,

        /// Число новых подтверждений
        newly_confirmed: usize,

        /// Длительность эпохи
        elapsed_ms: u32,
    },
}

/// Конфигурация ресивера.
pub struct ReceiverConfig {
    /// Размер IQ-блока в сэмплах (обычно 2048 при 2.048 Мсп/с = 1мс)
    pub block_size: usize,

    /// Частота дискретизации (Гц)
    pub sample_rate_hz: f64,

    /// Конфигурация верификатора acquisition
    pub verifier: VerifierConfig,

    /// Минимум подтверждённых спутников для перехода `Acquiring -> Tracking`
    pub min_satellites_for_tracking: usize,

    /// PRN для поиска. По умолчанию GPS 1-32
    pub search_prns: Vec<u8>,
}

/// Запись о захваченном спутнике.
#[derive(Debug, Clone)]
pub struct TrackedSatellite {
    /// Верефицированный acquisition-результат
    pub acquisition: AcquisitionResult,

    /// Эпоха захвата (номер IQ-блока)
    pub acuired_at_epoch: u64,
}

/// Оркестратор pipeline: IQ-источник → acquisition → tracking.
///
/// На каждую эпоху (`run_epoch`):
/// 1. Читает один IQ-блок из источника.
/// 2. Если состояние `Acquiring` — запускает верификатор для всех PRN.
/// 3. Новые подтверждённые спутники передаются через `dispatch_event`.
/// 4. Обновляет состояние state machine.
pub struct Receiver {
    config: ReceiverConfig,
    source: Box<dyn IqSource>,
    verifier: AcquisitionVerifier,
    state: ReceiverState,
    tracked: Vec<TrackedSatellite>,
    epoch: u64,
    acq_epochs: u64,
    acq_confirmed_total: u64,
}

impl Receiver {
    /// Создаёт ресивер.
    ///
    /// # Аргументы
    ///
    /// - `config` — конфигурация ресивера
    /// - `source` — источник IQ-данных (файл, SDR, mock)
    /// - `cache` — кэш PRN-кодов (разделяется через `Arc`)
    #[must_use]
    pub fn new(
        config: ReceiverConfig,
        source: Box<dyn IqSource>,
        cache: &Arc<PrnCodeCache>,
    ) -> Self {
        let mut verifier = AcquisitionVerifier::new(
            config.block_size,
            config.sample_rate_hz,
            config.verifier.clone(),
            Arc::clone(cache),
        );

        verifier.precompute_all();

        Self {
            config,
            source,
            verifier,
            state: ReceiverState::ColdStart,
            tracked: Vec::new(),
            epoch: 0,
            acq_epochs: 0,
            acq_confirmed_total: 0,
        }
    }

    /// Выполняет одну эпоху обработки.
    ///
    /// Читает один IQ-блок, выполняет acquisition (если нужно) и возвращает события.
    /// Вызывающий код должен передать события в tracking-каналы.
    ///
    /// # Errors
    ///
    /// Возвращает ошибку если IQ-источник вернул ошибку чтения или EOF.
    pub fn run_epoch(&mut self) -> Result<Vec<ReceiverEvent>, RfError> {
        let block = self.source.read_block(self.config.block_size)?;

        Ok(self.process_block(&block.samples))
    }

    /// Обрабатывает IQ-блок переданный снаружи (без чтения из источника)
    ///
    /// Удобно для тестирования и встроенных применений, где IQ-данные
    /// уже есть в памяти.
    pub fn process_block(
        &mut self,
        signal: &[Complex32],
    ) -> std::vec::Vec<pipeline::receiver::ReceiverEvent> {
        let mut events = Vec::new();
        let epoch = self.epoch;

        self.epoch += 1;

        // Переход из ColdStart -> Acquiring при первом блоке
        if self.state == ReceiverState::ColdStart {
            let ev = self.transition_to(ReceiverState::Acquiring);

            events.push(ev);
        }

        // Acquisition epoch (только в состоянии Acquiring)
        if self.state == ReceiverState::Acquiring {
            let acq_events = self.run_acquisition_epoch(signal, epoch);

            events.extend(acq_events);

            // Проверяем условие перехода Acquiring -> Tracking
            if self.tracked.len() >= self.config.min_satellites_for_tracking {
                let ev = self.transition_to(ReceiverState::Tracking);

                events.push(ev);
            }
        }

        events
    }

    /// Возвращает текущее состояние ресивера.
    #[must_use]
    pub const fn state(&self) -> ReceiverState {
        self.state
    }

    /// Запускает один цикл acquisition по всем `search_prns`.
    ///
    /// Возвращает события [`ReceiverEvent::SatelliteAcquired`] для каждого нового подтверждённого спутника.
    fn run_acquisition_epoch(
        &mut self,
        signal: &[Complex32],
        epoch: u64,
    ) -> Vec<ReceiverEvent> {
        let t0 = Instant::now();
        let mut events = Vec::new();
        let mut newly_confirmed = 0usize;

        // Ищем только PRN, которые ещё не захвачены
        let already_tracked: Vec<u8> = self.tracked.iter().map(|s| s.acquisition.prn).collect();
        let prns_to_search: Vec<u8> = self
            .config
            .search_prns
            .iter()
            .copied()
            .filter(|prn| !already_tracked.contains(prn))
            .collect();
        let prns_searched = prns_to_search.len();

        for prn in prns_to_search {
            let verdict = self.verifier.verify_prn(signal, prn);

            if let Some(result) = verdict.acquisition_result() {
                log::info!(
                    "PRN {} acquired: doppler={:.0} Hz, \
                     code_phase={} samples, C/N₀={:.1} dBHz",
                    result.prn,
                    result.doppler_hz,
                    result.code_phase_samples,
                    result.cn0_db_hz,
                );

                self.tracked.push(TrackedSatellite {
                    acquisition: result.clone(),
                    acuired_at_epoch: epoch,
                });

                events.push(ReceiverEvent::SatelliteAcquired { result });
                newly_confirmed += 1;

                self.acq_confirmed_total += 1;
            }
        }

        self.acq_epochs += 1;

        let elapsed_ms = t0.elapsed().as_millis() as u32;

        events.push(ReceiverEvent::AcquisitionEpochDone {
            prns_searched,
            newly_confirmed,
            elapsed_ms,
        });

        events
    }

    fn transition_to(
        &mut self,
        new_state: ReceiverState,
    ) -> ReceiverEvent {
        let from = self.state;

        self.state = new_state;

        log::debug!("Receiver: {from:?} -> {new_state:?}");

        ReceiverEvent::StateChanged {
            from,
            to: new_state,
        }
    }
}

impl Default for ReceiverConfig {
    fn default() -> Self {
        Self {
            block_size: 2048,
            sample_rate_hz: 2_048_000.0,
            verifier: VerifierConfig::default(),
            min_satellites_for_tracking: 1,
            search_prns: (1u8..=32).collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use num_complex::Complex32;

    use super::*;
    use crate::{
        acquisition::verifier::VerifierConfig,
        rf::{
            config::RfConfig,
            iq_source::{IqBlock, IqSource},
            metrics::SourceMetrics,
        },
        signal::prn_code::PrnCodeCache,
    };

    // ── Mock IQ source ────────────────────────────────────────────────────────

    /// Источник который выдаёт заранее подготовленные блоки по одному.
    struct VecSource {
        blocks: Vec<Vec<Complex32>>,
        idx: usize,
        config: RfConfig,
    }

    impl VecSource {
        fn new(blocks: Vec<Vec<Complex32>>) -> Self {
            Self {
                blocks,
                idx: 0,
                config: RfConfig::default(),
            }
        }

        fn single(block: Vec<Complex32>) -> Self {
            Self::new(vec![block])
        }
    }

    impl IqSource for VecSource {
        fn config(&self) -> &RfConfig {
            &self.config
        }

        fn name(&self) -> &'static str {
            "vec_source"
        }

        fn read_block(
            &mut self,
            _n: usize,
        ) -> crate::rf::error::RfResult<IqBlock> {
            if self.idx >= self.blocks.len() {
                return Err(crate::rf::error::RfError::EndOfFile);
            }
            let samples = self.blocks[self.idx].clone();
            self.idx += 1;
            Ok(IqBlock {
                samples,
                config: self.config.clone(),
                start_sample: (self.idx as u64 - 1) * 2048,
            })
        }

        fn metrics(&self) -> SourceMetrics {
            SourceMetrics::default()
        }
    }

    const FS: f64 = 2_048_000.0;
    const N: usize = 2048;

    fn make_receiver(signal: Vec<Complex32>) -> Receiver {
        let cache = Arc::new(PrnCodeCache::new());
        let cfg = ReceiverConfig {
            block_size: N,
            sample_rate_hz: FS,
            verifier: VerifierConfig {
                first_pass: crate::acquisition::fft_search::SearchConfig {
                    doppler_min_hz: -500.0,
                    doppler_max_hz: 500.0,
                    doppler_step_hz: 500.0,
                    cfar_threshold: 2.0,
                },
                second_pass: crate::acquisition::fft_search::SearchConfig {
                    doppler_min_hz: -250.0,
                    doppler_max_hz: 250.0,
                    doppler_step_hz: 250.0,
                    cfar_threshold: 2.0,
                },
                doppler_tolerance_hz: 600.0,
                retry: crate::acquisition::verifier::RetryPolicy {
                    max_attempts: 1,
                    base_delay_ms: 0,
                    max_delay_ms: 0,
                },
            },
            min_satellites_for_tracking: 1,
            search_prns: (1u8..=32).collect(),
        };
        let source = Box::new(VecSource::single(signal));
        Receiver::new(cfg, source, &cache)
    }

    #[test]
    fn test_initial_state_is_cold_start() {
        let rx = make_receiver(vec![Complex32::new(0.0, 0.0); N]);

        assert_eq!(rx.state(), ReceiverState::ColdStart);
    }

    #[test]
    fn test_first_epoch_transitions_to_acquiring() {
        let mut rx = make_receiver(vec![Complex32::new(0.0, 0.0); N]);
        let events = rx.run_epoch().unwrap();

        assert_eq!(rx.state(), ReceiverState::Acquiring);

        // Должно быть StateChanged ColdStart → Acquiring
        let state_changed = events.iter().any(|e| {
            matches!(
                e,
                ReceiverEvent::StateChanged {
                    from: ReceiverState::ColdStart,
                    to: ReceiverState::Acquiring,
                }
            )
        });

        assert!(state_changed, "expected StateChanged ColdStart→Acquiring");
    }

    #[test]
    fn acquisition_epoch_done_event_emitted() {
        let mut rx = make_receiver(vec![Complex32::new(0.0, 0.0); N]);
        let events = rx.run_epoch().unwrap();
        let has_epoch_done = events
            .iter()
            .any(|e| matches!(e, ReceiverEvent::AcquisitionEpochDone { .. }));

        assert!(has_epoch_done, "AcquisitionEpochDone should be emitted");
    }
}
