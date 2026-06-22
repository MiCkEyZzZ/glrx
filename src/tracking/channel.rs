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

use crate::signal::correlator::normalisation::cn0_estimate;
use crate::tracking::dll::{Dll, DllConfig, DllOutput};
use crate::tracking::fll::{Fll, FllConfig, FllOutput};
use crate::tracking::pll::{Pll, PllConfig};

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

    /// Выход DLL (фаза/частота кода)
    pub dll: DllOutput,

    /// Выход FLL, если контур ещё находится на стадии частотного захвата
    pub fll: FllOutput,

    /// Текущая оценка C/N₀ канала (дБ-Гц), если накоплено достаточно данных
    pub cn0_db_hz: Option<f32>,

    /// Состояние канала после этого обновления
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

    _config: ChannelConfig,
    _allocated_at_epoch: u64,
    _phase_locked_at_epoch: Option<u64>,
    _total_epochs: u64,
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
    _slots: Vec<Option<TrackingChannel>>,
    _config: ChannelBankConfig,
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

#[cfg(test)]
mod tests {
    use super::*;

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
}
