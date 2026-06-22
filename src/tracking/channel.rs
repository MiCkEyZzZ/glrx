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
    /// Число одновременно отслеживаемых спутников
    pub num_channels: usize,

    /// Конфигурация, применяемая, к каждому новому каналу при аллокации
    pub channel: ChannelConfig,
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
                _ => ChannelState::FrequencyLock,
            },
            (None, _) => match self.fll.state() {
                FllState::Searching | FllState::FllLock => ChannelState::FrequencyLock,
                FllState::PllLock => ChannelState::PhaseLock, // переходный кадр
            },
            _ => self.state,
        };
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
}
