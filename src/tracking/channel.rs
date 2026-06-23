//! Channel abstraction - единица параллельного сопровождения одного спутника.
//!
//! Receiver одновременно отслеживает несколько спутников (типично 8/16/32),
//! каждый - в своём `TrackingChannel`. Канал инкапсулирует полный tracking-стек
//! одного PRN: DLL (код), FLL (грубый частотный захват),
//! PLL (фазовая синхронизация после захвата), оценка C/N₀ и собственный
//! индикатор состояния.
//!
//! # Жизненный цикл канала
//!
//! ```text
//! AcquisitionResult { prn, doppler_hz, code_phase_chips, cn0_db_hz, .. }
//!     │
//!     ▼
//! TrackingChannel::allocate(prn, acquisition_result, config)
//!     │  ChannelState::Acquired
//!     ▼
//! TrackingChannel::update(epl)        ← вызывается каждую 1-мс эпоху
//!     │
//!     ├─ Dll::update(epl)                          → code phase / freq
//!     ├─ Fll::update(epl.prompt)  (пока не ready)   → грубая частота
//!     │      │ ready_for_pll
//!     │      ▼
//!     │  Pll::new(fll.complete_handoff())           ← один раз, handoff
//!     ├─ Pll::update(epl.prompt)  (после handoff)   → точная фаза
//!     ▼
//! ChannelState::{FrequencyLock, PhaseLock}
//!     │
//!     ▼ (PLL сигнализирует LockLost)
//! ChannelState::LockLost  →  деаллокация / повторный acquisition
//! ```
//!
//! # Многоканальность
//!
//! [`ChannelBank`] держит конфигурируемое число каналов (8/16/32),
//! аллоцирует их под новые [`AcquisitionResult`] и деаллоцирует при потере
//! lock. Обновление всех активных каналов на одну IQ-эпоху выполняется
//! через [`ChannelBank::update_all`] — последовательно или параллельно
//! (через Rayon, если включена фича `rayon`), так как каналы полностью
//! независимы друг от друга и не делят мутируемое состояние.

use num_complex::Complex32;

use crate::acquisition::verifier::AcquisitionResult;
use crate::signal::correlator::discriminators::EplOutput;
use crate::signal::correlator::normalisation::cn0_estimate;
use crate::tracking::dll::{Dll, DllConfig, DllOutput};
use crate::tracking::fll::{Fll, FllConfig, FllOutput, FllState};
use crate::tracking::pll::{Pll, PllConfig, PllOutput, PllState};

// Доплер несущей пересчитывается в поправку частоты кода:
// doppler_chp_rate = doppler_hz * (chip_rate / carrier_freq_hz)
// Точный carrier_freq_hz знает только RF-слой. Тут использую
// консервативное приближение через номинальное отношение GPS L1
// (chip_rate / L1_freq), которое канал может уточнить позже через
// `Dll::initialize` извне при необходимости.
const GPS_L1_HZ: f64 = 1_575_420_000.0;

/// Состояние каналов сопровождения.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelState {
    /// Каналов свободен (не аллоцирован под спутник)
    Idle,

    /// Канал только аллоцирован под `AcquisitionResult`, ещё ни одного
    /// обновления не произошло.
    Acquired,

    /// FLL выполняет грубый частотный захват
    FrequencyLock,

    /// PLL выполняет точное фазовое сопровождение (после handoff от FLL)
    PhaseLock,

    /// Lock потерян - каналов помечен на деалокацию
    LockLost,
}

/// Скользящая оценка C/N₀ каналов на основе истории Prompt-корреляции.
///
/// Отдельная от внутренних оценок `Pll`/`LockDetector` — это оценка
/// **уровня канала**, используемая для общих метрик и решений о
/// деаллокации (`TrackingChannel`), не привязанная к конкретной стадии
/// (FLL/PLL).
#[derive(Debug, Clone)]
pub struct Cn0Estimator {
    history: Vec<Complex32>,
    window: usize,
    coherent_time_s: f64,
}

/// Конфигурация одного [`TrackingChannel`].
#[derive(Debug, Clone)]
pub struct ChannelConfig {
    /// Конфигурация DLL
    pub dll: DllConfig,

    /// Конфигурация FLL (частотный захват)
    pub fll: FllConfig,

    /// Конфигурация PLL (фазовое сопровождение после handoff)
    pub pll: PllConfig,

    /// Рзамер окна оценки C/N₀ канала (число эпох).
    pub cn0_window: usize,
}

/// Выход одной эпохи обновления канала.
#[derive(Debug, Clone, Copy)]
pub struct ChannelOutput {
    /// PRN спутника, отслеживаемого этим каналом.
    pub prn: u8,

    /// Выход DLL (фаза/частота кода).
    pub dll: DllOutput,

    /// Выход PLL, если контур уже передан в фазовое сопровождение.
    pub pll: Option<PllOutput>,

    /// Выход FLL, если контур ещё находится на стадии частотного захвата.
    pub fll: Option<FllOutput>,

    /// Текущая оценка C/N₀ канала (дБ-Гц), если накоплено достаточно данных.
    pub cn0_db_hz: Option<f32>,

    /// Состояние канала после этого обновления.
    pub state: ChannelState,
}

/// Канал сопровождения одного спутника: DLL + FLL -> PLL + оценка C/N₀.
///
/// Создаётся через [`TrackingChannel::allocate`] из [`AcquisitionResult`];
/// каждая 1-мс эпоха подаётся через [`TrackingChannel::update`].
pub struct TrackingChannel {
    /// PRN отслеживаемого спутника
    pub prn: u8,

    /// Кодовый контур
    pub dll: Dll,

    /// Частотный контур (активен до handoff в PLL)
    pub fll: Fll,

    /// Фазовый контур; `None` до завершения FLL-захвата
    pub pll: Option<Pll>,

    /// Оценщик C/N₀ уровня канала
    pub cn0_estimator: Cn0Estimator,

    /// Текущее состояние канала
    pub state: ChannelState,

    config: ChannelConfig,
    allocated_at_epoch: u64,
    phase_locked_at_epoch: Option<u64>,
    total_epochs: u64,
}

/// Конфигурация банка каналов.
#[derive(Debug, Clone)]
pub struct ChannelBankConfig {
    /// Число одновременно отслеживаемых спутников. Issue фиксирует
    /// типичные значения: 8 / 16 / 32, но допускается любое положительное
    /// число.
    pub num_channels: usize,

    /// Конфигурация, применяемая, к каждому новому каналу при аллокации
    pub channel: ChannelConfig,
}

/// Суммарные метрики банкаканалов.
#[derive(Debug, Clone, Default)]
pub struct ChannelBankMetrics {
    /// Число сконфигурированных слотов (емкость банка)
    pub capacity: usize,

    /// Число каналов, активно отслеживающих спутник (не `Idle`)
    pub active_channels: usize,

    /// Число каналов в состоянии `PhaseLock`
    pub phase_locked_channels: usize,

    /// Время захвата (`lock_time_ms`) по каждому активному PRN, для
    /// которого фазовый lock уже достигнут
    pub lock_time_per_prn_ms: Vec<(u8, u64)>,
}

/// Пул каналов сопровождения с конфигурируемой ёмкостью (8/16/32 и т.д.)
pub struct ChannelBank {
    slots: Vec<Option<TrackingChannel>>,
    config: ChannelBankConfig,
}

impl Cn0Estimator {
    /// Создаёт оценщик со скользящим окном `window` Prompt-значений.
    ///
    /// # Panics
    ///
    /// Паникует, если `window < 2` (минимум для оценки нужно две точки)
    #[must_use]
    pub fn new(
        window: usize,
        coherent_time_s: f64,
    ) -> Self {
        assert!(window >= 2, "Cn0Estimator window must be at least 2");

        Self {
            history: Vec::with_capacity(window),
            window,
            coherent_time_s,
        }
    }

    /// Конструктор с настройками по умолчанию: окно 20 (20мс при 1 мс/эпоха).
    #[must_use]
    pub fn with_defaults() -> Self {
        Self::new(20, 0.001)
    }

    /// Добавляет отдну Prompt-корреляцию в скользящее окно.
    pub fn push(
        &mut self,
        prompt: Complex32,
    ) {
        self.history.push(prompt);

        if self.history.len() > self.window {
            self.history.remove(0);
        }
    }

    /// Текущая оценка C/N₀ (дБ-Гц), либо `None`, если накоплено меньше
    /// двух точек.
    #[must_use]
    pub fn estimate_db_hz(&self) -> Option<f32> {
        if self.history.len() < 2 {
            None
        } else {
            Some(cn0_estimate(&self.history, self.coherent_time_s))
        }
    }
}

impl TrackingChannel {
    /// Аллоцирует новый канал под результат acquisition.
    #[must_use]
    pub fn allocate(
        acquisition: &AcquisitionResult,
        config: ChannelConfig,
    ) -> Self {
        let mut dll = Dll::new(config.dll.clone());
        let chip_rate = dll.config().nominal_chip_rate_hz;
        let doppler_chip_correction = acquisition.doppler_hz * (chip_rate / GPS_L1_HZ);

        dll.initialize(acquisition.code_phase_chips, doppler_chip_correction);

        let fll = Fll::new(config.fll, acquisition.doppler_hz);
        let cn0_estimator = Cn0Estimator::new(config.cn0_window, 0.001);

        Self {
            prn: acquisition.prn,
            dll,
            fll,
            pll: None,
            cn0_estimator,
            state: ChannelState::Acquired,
            config,
            allocated_at_epoch: 0,
            phase_locked_at_epoch: None,
            total_epochs: 0,
        }
    }

    /// Обработывает одну 1-мс эпоху: обновляет DLL, и в зависимости от
    /// текущей стадии - FLL (до handoff) либо PLL (после).
    pub fn update(
        &mut self,
        epl: &EplOutput,
    ) -> ChannelOutput {
        self.total_epochs += 1;

        let dll_out = self.dll.update(epl);
        self.cn0_estimator.push(epl.prompt);

        let (fll_out, pll_out) = self.step_frequency_or_phase(epl.prompt);

        self.update_channel_state(pll_out.as_ref());

        ChannelOutput {
            prn: self.prn,
            dll: dll_out,
            pll: pll_out,
            fll: fll_out,
            cn0_db_hz: self.cn0_estimator.estimate_db_hz(),
            state: self.state,
        }
    }

    /// Время от аллокации до первого достижения `PhaseLock`, в
    /// миллисекундах (1 эпоха = 1мс). `None`, если фазовый lock ещё не
    /// достигнут.
    #[must_use]
    pub fn lock_time_ms(&self) -> Option<u64> {
        self.phase_locked_at_epoch
            .map(|e| e - self.allocated_at_epoch)
    }

    /// Число обработанных эпох с момента аллокации.
    #[must_use]
    pub const fn total_epochs(&self) -> u64 {
        self.total_epochs
    }

    /// Текущая оценка C/N₀ канала (дБ-Гц).
    #[must_use]
    pub fn cn0_db_hz(&self) -> Option<f32> {
        self.cn0_estimator.estimate_db_hz()
    }

    fn step_frequency_or_phase(
        &mut self,
        prompt: Complex32,
    ) -> (Option<FllOutput>, Option<PllOutput>) {
        if let Some(pll) = self.pll.as_mut() {
            // PLL уже активен — FLL больше не вызывается.
            let out = pll.update(prompt);

            (None, Some(out))
        } else {
            let fll_out = self.fll.update(prompt);

            if fll_out.ready_for_pll {
                let handoff_freq = self.fll.complete_handoff();
                let mut pll = Pll::new(self.config.pll.clone(), handoff_freq);
                let pll_out = pll.update(prompt);

                self.pll = Some(pll);

                (Some(fll_out), Some(pll_out))
            } else {
                (Some(fll_out), None)
            }
        }
    }

    #[allow(clippy::match_same_arms, clippy::match_wildcard_for_single_variants)]
    fn update_channel_state(
        &mut self,
        pll_out: Option<&PllOutput>,
    ) {
        self.state = match (&self.pll, pll_out) {
            (Some(_), Some(out)) => match out.state {
                PllState::Searching => ChannelState::FrequencyLock,
                PllState::PllLock => {
                    self.phase_locked_at_epoch.get_or_insert(self.total_epochs);
                    ChannelState::PhaseLock
                }
                PllState::LockLost => ChannelState::LockLost,
            },
            (None, _) => match self.fll.state() {
                FllState::Searching | FllState::FllLock => ChannelState::FrequencyLock,
                FllState::PllLock => ChannelState::PhaseLock, // переходный кадр
            },
            _ => self.state,
        };
    }

    /// Возвращает `true`, если канал считается потерявшим lock и должен быть
    /// деаллоцирован вызывающим кодом (см. [`ChannelBank::reap_lost`]).
    #[must_use]
    pub fn is_lock_lost(&self) -> bool {
        self.state == ChannelState::LockLost
    }
}

impl ChannelBank {
    /// Создаёт банк с `num_channels` свободными слотами.
    #[must_use]
    pub fn new(config: ChannelBankConfig) -> Self {
        let num_channels = config.num_channels;
        let slots = (0..num_channels).map(|_| None).collect();

        Self { slots, config }
    }

    /// Ёмкость банка (число слотов).
    #[must_use]
    pub const fn capacity(&self) -> usize {
        self.slots.len()
    }

    /// Число свободных слотов.
    #[must_use]
    pub fn free_slots(&self) -> usize {
        self.slots.iter().filter(|s| s.is_none()).count()
    }

    /// Аллоцирует свободный слот под `acquisition`.
    ///
    /// Возвращает индекс выделенного слота, либо `None`, если все слоты
    /// заняты.
    pub fn allocate(
        &mut self,
        acquisition: &AcquisitionResult,
    ) -> Option<usize> {
        let idx = self.slots.iter().position(Option::is_none)?;

        self.slots[idx] = Some(TrackingChannel::allocate(
            acquisition,
            self.config.channel.clone(),
        ));

        Some(idx)
    }

    /// Принудительно освобождает слот `idx` (например, по внешнему решению
    /// — например, при ручном переакуайре спутника).
    pub fn deallocate(
        &mut self,
        idx: usize,
    ) -> Option<TrackingChannel> {
        self.slots.get_mut(idx).and_then(Option::take)
    }

    /// Освобождает все каналы, находящиеся в состоянии `LockLost`.
    ///
    /// Возвращает список PRN деаллоцированных спутников.
    pub fn reap_lost(&mut self) -> Vec<u8> {
        let mut reaped = Vec::new();

        for slot in &mut self.slots {
            let should_reap = slot.as_ref().is_some_and(TrackingChannel::is_lock_lost);

            if should_reap && let Some(channel) = slot.take() {
                reaped.push(channel.prn);
            }
        }

        reaped
    }

    /// Обновляет все активные каналы одной 1мс эпохой EPL-данных.
    /// `epl_by_prn` - ф-я, возвращая [`EplOutput`] для заданного
    /// PRN на текущей эпохе (как правило, результат отдельной
    /// `correlator_epl` на канал, с собственным Early/Prompt/Late
    /// репликами кода данного канала).
    ///
    /// Последовательная версия: используется когда фича `rayon` отключена.
    /// Каждый канал обновляется независимо - порядок не важен.
    #[cfg(not(feature = "rayon"))]
    pub fn update_all<F>(
        &mut self,
        mut epl_by_prn: F,
    ) -> Vec<ChannelOutput>
    where
        F: FnMut(u8) -> EplOutput,
    {
        self.slots
            .iter_mut()
            .filter_map(Option::as_mut)
            .map(|ch| {
                let epl = epl_by_prn(ch.prn);

                ch.update(&epl)
            })
            .collect()
    }

    /// Параллельная версия [`ChannelBank::update_all`] через Rayon.
    ///
    /// Каналы не делят мутироемое состояние между собой, поэтому
    /// обновление каждого канала безопасно выполнять в отдельном
    /// потоке. `epl_by_prn` должна `Sync`, так как вызывается параллельно
    /// из нескольких потоков.
    #[cfg(feature = "rayon")]
    pub fn update_all<F>(
        &mut self,
        epl_by_prn: F,
    ) -> Vec<ChannelOutput>
    where
        F: Fn(u8) -> EplOutput + Sync,
    {
        use rayon::prelude::*;

        self.slots
            .par_iter_mut()
            .filter_map(Option::as_mut)
            .map(|ch| {
                let epl = epl_by_prn(ch.prn);
                ch.update(&epl)
            })
            .collect()
    }

    /// Снимок метрик банка: число активных/locked каналов, время захвата
    /// по каждому PRN с достигнутым фазовым lock.
    #[must_use]
    pub fn metrics(&self) -> ChannelBankMetrics {
        let active: Vec<&TrackingChannel> = self.slots.iter().filter_map(Option::as_ref).collect();
        let phase_locked_channels = active
            .iter()
            .filter(|ch| ch.state == ChannelState::PhaseLock)
            .count();
        let lock_time_per_prn_ms = active
            .iter()
            .filter_map(|ch| ch.lock_time_ms().map(|t| (ch.prn, t)))
            .collect();

        ChannelBankMetrics {
            capacity: self.slots.len(),
            active_channels: active.len(),
            phase_locked_channels,
            lock_time_per_prn_ms,
        }
    }

    /// Итератор по занятым каналам (только для чтения).
    pub fn channels(&self) -> impl Iterator<Item = &TrackingChannel> {
        self.slots.iter().filter_map(Option::as_ref)
    }

    /// Находит канал по PRN, если он аллоцирован.
    #[must_use]
    pub fn find_by_prn(
        &self,
        prn: u8,
    ) -> Option<&TrackingChannel> {
        self.channels().find(|ch| ch.prn == prn)
    }
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            dll: DllConfig::default(),
            fll: FllConfig::default(),
            pll: PllConfig::default(),
            cn0_window: 20,
        }
    }
}

impl Default for ChannelBankConfig {
    fn default() -> Self {
        Self {
            num_channels: 16,
            channel: ChannelConfig::default(),
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    fn dummy_acquisition(
        prn: u8,
        doppler_hz: f64,
    ) -> AcquisitionResult {
        AcquisitionResult {
            prn,
            doppler_hz,
            code_phase_samples: 0,
            code_phase_chips: 0.0,
            cn0_db_hz: 45.0,
            peak_to_noise: 100.0,
        }
    }

    fn fast_channel_config() -> ChannelConfig {
        // Быстрые пороги, чтобы FLL и handoff происходили в чситанные эпохи
        ChannelConfig {
            fll: FllConfig {
                epochs_before_narrowing: 2,
                epochs_before_handoff: 2,
                stable_threshold_hz: 1000.0,
                ..FllConfig::default()
            },
            ..ChannelConfig::default()
        }
    }

    #[test]
    fn test_cn0_estimator_none_with_fewer_than_two_samples() {
        let mut est = Cn0Estimator::with_defaults();

        assert!(est.estimate_db_hz().is_none());

        est.push(Complex32::new(1.0, 0.0));

        assert!(est.estimate_db_hz().is_none());
    }

    #[test]
    fn test_cn0_estimator_returns_value_after_two_samples() {
        let mut est = Cn0Estimator::with_defaults();

        for _ in 0..5 {
            est.push(Complex32::new(100.0, 0.0));
        }

        assert!(est.estimate_db_hz().is_some());
    }

    #[test]
    fn test_cn0_estimator_window_bounded() {
        let mut est = Cn0Estimator::new(5, 0.001);

        for i in 0..20 {
            est.push(Complex32::new(i as f32, 0.0));
        }

        assert!(est.history.len() <= 5);
    }

    #[test]
    #[should_panic(expected = "at least 2")]
    fn test_cn0_estimator_rejects_window_below_two() {
        let _ = Cn0Estimator::new(1, 0.001);
    }

    #[test]
    fn test_channel_allocate_sets_prn_and_acquired_state() {
        let acq = dummy_acquisition(7, 500.0);
        let ch = TrackingChannel::allocate(&acq, ChannelConfig::default());

        assert_eq!(ch.prn, 7);
        assert_eq!(ch.state, ChannelState::Acquired);
        assert!(ch.pll.is_none());
    }

    #[test]
    fn test_channel_allocate_initializes_dll_phase() {
        let acq = dummy_acquisition(3, 0.0);
        let ch = TrackingChannel::allocate(&acq, ChannelConfig::default());

        assert!((ch.dll.code_phase_offset_chips() - acq.code_phase_chips).abs() < 1e-6);
    }

    #[test]
    fn test_channel_update_progresses_through_frequency_lock() {
        let acq = dummy_acquisition(1, 0.0);
        let mut ch = TrackingChannel::allocate(&acq, fast_channel_config());
        let epl = EplOutput {
            early: Complex32::new(1.0, 0.0),
            prompt: Complex32::new(1.0, 0.0),
            late: Complex32::new(1.0, 0.0),
        };
        let out = ch.update(&epl);

        assert_eq!(out.prn, 1);
        assert!(matches!(
            out.state,
            ChannelState::FrequencyLock | ChannelState::PhaseLock
        ));
    }

    #[test]
    fn test_channel_eventually_reaches_phase_lock_under_stable_signal() {
        let acq = dummy_acquisition(5, 0.0);
        let mut ch = TrackingChannel::allocate(&acq, fast_channel_config());
        let epl = EplOutput {
            early: Complex32::new(1.0, 0.0),
            prompt: Complex32::new(1.0, 0.0),
            late: Complex32::new(1.0, 0.0),
        };
        let mut reached_phase_lock = false;

        for _ in 0..50 {
            if ch.update(&epl).state == ChannelState::PhaseLock {
                reached_phase_lock = true;
                break;
            }
        }

        assert!(
            reached_phase_lock,
            "channel should reach PhaseLock under stable signal"
        );
        assert!(ch.pll.is_some());
    }

    #[test]
    fn test_channel_lock_time_ms_recorded_after_phase_lock() {
        let acq = dummy_acquisition(9, 0.0);
        let mut ch = TrackingChannel::allocate(&acq, fast_channel_config());
        let epl = EplOutput {
            early: Complex32::new(1.0, 0.0),
            prompt: Complex32::new(1.0, 0.0),
            late: Complex32::new(1.0, 0.0),
        };

        for _ in 0..50 {
            ch.update(&epl);
            if ch.state == ChannelState::PhaseLock {
                break;
            }
        }

        assert!(ch.lock_time_ms().is_some());
    }

    #[test]
    fn test_channel_cn0_db_hz_available_after_enough_epochs() {
        let acq = dummy_acquisition(2, 0.0);
        let mut ch = TrackingChannel::allocate(&acq, ChannelConfig::default());
        let epl = EplOutput {
            early: Complex32::new(1.0, 0.0),
            prompt: Complex32::new(100.0, 0.0),
            late: Complex32::new(1.0, 0.0),
        };

        for _ in 0..5 {
            ch.update(&epl);
        }

        assert!(ch.cn0_db_hz().is_some());
    }

    #[test]
    fn test_channel_total_epochs_increments() {
        let acq = dummy_acquisition(4, 0.0);
        let mut ch = TrackingChannel::allocate(&acq, ChannelConfig::default());
        let epl = EplOutput {
            early: Complex32::default(),
            prompt: Complex32::new(1.0, 0.0),
            late: Complex32::default(),
        };

        for i in 1..=10u64 {
            ch.update(&epl);
            assert_eq!(ch.total_epochs(), i);
        }
    }

    #[test]
    fn test_bank_respects_configured_capacity_8() {
        let bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 8,
            ..Default::default()
        });

        assert_eq!(bank.capacity(), 8);
        assert_eq!(bank.free_slots(), 8);
    }

    #[test]
    fn test_bank_respects_configured_capacity_16() {
        let bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 16,
            ..Default::default()
        });

        assert_eq!(bank.capacity(), 16);
    }

    #[test]
    fn test_bank_respects_configured_capacity_32() {
        let bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 32,
            ..Default::default()
        });

        assert_eq!(bank.capacity(), 32);
    }

    #[test]
    fn test_bank_allocate_fills_free_slot() {
        let mut bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 4,
            ..Default::default()
        });
        let acq = dummy_acquisition(11, 100.0);
        let idx = bank.allocate(&acq);

        assert!(idx.is_some());
        assert_eq!(bank.free_slots(), 3);
    }

    #[test]
    fn test_bank_allocate_returns_none_when_full() {
        let mut bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 2,
            ..Default::default()
        });

        assert!(bank.allocate(&dummy_acquisition(1, 0.0)).is_some());
        assert!(bank.allocate(&dummy_acquisition(2, 0.0)).is_some());
        assert!(bank.allocate(&dummy_acquisition(3, 0.0)).is_none());
    }

    #[test]
    fn test_bank_deallocate_frees_slot() {
        let mut bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 2,
            ..Default::default()
        });
        let idx = bank.allocate(&dummy_acquisition(1, 0.0)).unwrap();

        assert_eq!(bank.free_slots(), 1);

        let removed = bank.deallocate(idx);

        assert!(removed.is_some());
        assert_eq!(bank.free_slots(), 2);
    }

    #[test]
    fn test_bank_reap_lost_removes_only_lock_lost_channels() {
        let mut bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 3,
            ..Default::default()
        });

        bank.allocate(&dummy_acquisition(1, 0.0));
        bank.allocate(&dummy_acquisition(2, 0.0));

        // Никто ещё не потерял lock — reap не должен ничего удалить.
        let reaped = bank.reap_lost();

        assert!(reaped.is_empty());
        assert_eq!(bank.free_slots(), 1);
    }

    #[test]
    fn test_bank_find_by_prn_locates_allocated_channel() {
        let mut bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 4,
            ..Default::default()
        });

        bank.allocate(&dummy_acquisition(21, 0.0));

        assert!(bank.find_by_prn(21).is_some());
        assert!(bank.find_by_prn(99).is_none());
    }

    #[test]
    fn test_bank_update_all_updates_every_active_channel() {
        let mut bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 4,
            ..Default::default()
        });
        bank.allocate(&dummy_acquisition(1, 0.0));
        bank.allocate(&dummy_acquisition(2, 0.0));

        let outputs = bank.update_all(|_prn| EplOutput {
            early: Complex32::new(1.0, 0.0),
            prompt: Complex32::new(1.0, 0.0),
            late: Complex32::new(1.0, 0.0),
        });

        assert_eq!(outputs.len(), 2);
    }

    #[test]
    fn test_bank_update_all_skips_idle_slots() {
        let mut bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 8,
            ..Default::default()
        });
        bank.allocate(&dummy_acquisition(1, 0.0));

        let outputs = bank.update_all(|_prn| EplOutput {
            early: Complex32::default(),
            prompt: Complex32::new(1.0, 0.0),
            late: Complex32::default(),
        });

        assert_eq!(
            outputs.len(),
            1,
            "only the allocated slot should produce output"
        );
    }

    #[test]
    fn test_bank_update_all_passes_correct_prn_to_callback() {
        let mut bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 4,
            ..Default::default()
        });
        bank.allocate(&dummy_acquisition(17, 0.0));

        let outputs = bank.update_all(|prn| {
            assert_eq!(prn, 17);
            EplOutput {
                early: Complex32::default(),
                prompt: Complex32::new(1.0, 0.0),
                late: Complex32::default(),
            }
        });

        assert_eq!(outputs[0].prn, 17);
    }

    #[test]
    fn test_bank_metrics_reports_capacity_and_active_count() {
        let mut bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 8,
            ..Default::default()
        });
        bank.allocate(&dummy_acquisition(1, 0.0));
        bank.allocate(&dummy_acquisition(2, 0.0));

        let metrics = bank.metrics();
        assert_eq!(metrics.capacity, 8);
        assert_eq!(metrics.active_channels, 2);
    }

    #[test]
    fn test_bank_metrics_phase_locked_count_increases_under_stable_signal() {
        let mut bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 2,
            channel: fast_channel_config(),
        });
        bank.allocate(&dummy_acquisition(1, 0.0));

        let epl = EplOutput {
            early: Complex32::new(1.0, 0.0),
            prompt: Complex32::new(1.0, 0.0),
            late: Complex32::new(1.0, 0.0),
        };

        let mut phase_locked_at_some_point = false;
        for _ in 0..50 {
            bank.update_all(|_| epl.clone());
            if bank.metrics().phase_locked_channels > 0 {
                phase_locked_at_some_point = true;
                break;
            }
        }

        assert!(phase_locked_at_some_point);
    }

    #[test]
    fn test_bank_metrics_lock_time_per_prn_populated_after_lock() {
        let mut bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 1,
            channel: fast_channel_config(),
        });
        bank.allocate(&dummy_acquisition(42, 0.0));

        let epl = EplOutput {
            early: Complex32::new(1.0, 0.0),
            prompt: Complex32::new(1.0, 0.0),
            late: Complex32::new(1.0, 0.0),
        };

        for _ in 0..50 {
            bank.update_all(|_| epl.clone());
        }

        let metrics = bank.metrics();
        let found = metrics
            .lock_time_per_prn_ms
            .iter()
            .any(|(prn, _)| *prn == 42);
        assert!(
            found,
            "PRN 42 should appear in lock_time_per_prn_ms once locked"
        );
    }

    #[test]
    fn test_bank_metrics_empty_bank_reports_zero_active() {
        let bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 16,
            ..Default::default()
        });
        let metrics = bank.metrics();
        assert_eq!(metrics.active_channels, 0);
        assert_eq!(metrics.phase_locked_channels, 0);
        assert!(metrics.lock_time_per_prn_ms.is_empty());
    }

    #[test]
    fn test_bank_handles_many_channels_independently() {
        let mut bank = ChannelBank::new(ChannelBankConfig {
            num_channels: 32,
            ..Default::default()
        });
        for prn in 1u8..=32 {
            assert!(
                bank.allocate(&dummy_acquisition(prn, f64::from(prn) * 10.0))
                    .is_some()
            );
        }
        assert_eq!(bank.free_slots(), 0);

        let outputs = bank.update_all(|prn| EplOutput {
            early: Complex32::new(1.0, 0.0),
            prompt: Complex32::new(f64::from(prn) as f32, 0.0),
            late: Complex32::new(1.0, 0.0),
        });

        assert_eq!(outputs.len(), 32);
        // Каждый канал должен получить именно свой PRN.
        let mut prns: Vec<u8> = outputs.iter().map(|o| o.prn).collect();
        prns.sort_unstable();
        let expected: Vec<u8> = (1u8..=32).collect();
        assert_eq!(prns, expected);
    }
}
