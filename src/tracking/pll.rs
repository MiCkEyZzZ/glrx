//! PLL — Phase Lock Loop для отслеживания фазы несущей.
//!
//! Без PLL несущая не синхронизирована: навигационные биты не декодируются,
//! псевдодальность не может быть вычислена с требуемой точностью.
//!
//! # Контур
//!
//! ```text
//! correlator_epl(signal, early, prompt, late)
//!     │  EplOutput { E, P, L }
//!     ▼
//! CoherentAccumulator (1–20 мс)
//!     │  накопленный Prompt
//!     ▼
//! Costas (dd_atan) discriminator      ← ошибка фазы
//!     ▼
//! PllLoopFilter (3-й порядок: фаза + частота + ускорение частоты)
//!     │  поправка частоты несущей (Hz)
//!     ▼
//! Pll::carrier_freq_hz()  → carrier NCO
//! ```
//! # Связь с FLL
//!
//! `Pll` — чисто фазовый контур. Частотный захват (грубая синхронизация
//! при больших начальных ошибках Doppler) выполняется отдельным модулем
//! [`crate::tracking::fll::Fll`] **до** создания `Pll`: после того как
//! `Fll::update` сигнализирует `ready_for_pll`, его финальная оценка
//! частоты (`Fll::complete_handoff()`) передаётся в [`Pll::new`] как
//! `initial_doppler_hz`. Сам `Pll` не содержит FLL-стадии — она полностью
//! вынесена, чтобы не дублировать cross-product дискриминатор и не плодить
//! два независимых частотных контура внутри одного канала (см.
//! [`crate::tracking::channel::TrackingChannel`], который оркестрирует
//! переключение).
//!
//! # Дискриминатор Костаса
//!
//! Используется decision-directed `atan` из [`EplOutput::pll_dd_atan`]
//! (`signal::correlator::discriminators`) - устраняет 180°-неоднозначность
//! навигационного бита, что делает его классическим Costas-дискриминатором
//! для BPSK-сигналов (GPS L1 C/A).
//!
//! # Петлевой фильтр третьего порядка
//!
//! Третий порядок (фаза, частота, скорость изменения частоты) необходим
//! для удержания lock при высокой динамике платформы (ускорение/jerk).
//! Используется стандартная форма с тремя коэффициентами `a₁, a₂, a₃`,
//! полученными из шумовой полосы `Bₗ` через табличные множители Уильямса:
//!
//! ```text
//! a₁ = 1.1·ωₙ        a₂ = 2.4·ωₙ²       a₃ = 1.1·ωₙ³
//! ωₙ = Bₗ / 0.7845    (приближение для 3-го порядка, ζ ≈ 1)
//! ```
//!
//! # Детекция потери lock
//!
//! Lock считается потерянным, если:
//! - диспресия фазовой ошибки за окно превышает порог, или
//! - оценка C/N₀ (через накопленные Prompt) падает ниже порога.

use std::{collections::VecDeque, f64::consts::TAU};

use num_complex::Complex32;

use crate::signal::correlator::{discriminators::EplOutput, normalisation::cn0_estimate};

/// Состояние PLL-контура.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PllState {
    /// Захват не начат (начальное состояние)
    Searching,

    /// PLL активен - фазовая синхронизация достигнута, FLL отключён
    PllLock,

    /// Lock потерян (детектирован loss-of-lock); требует повторный захват
    LockLost,
}

/// Накопитель когерентной интеграции Prompt-корреляцией.
///
/// GPS L1 C/A навигационный бит длится 20 мс в пределах одного бита можно
/// складывать Prompt-корреляция без потери информации (конгерентное накопление),
/// повышая эффективный C/N₀ перед дискриминатором.
#[derive(Debug, Clone)]
pub struct CoherentAccumulator {
    /// Целое число эпох в одном накоплении (1-20)
    traget_epochs: usize,

    /// Накопленная сумма Prompt за текущий интервал
    sum: Complex32,

    /// Число эпох, накопленных с начала текущего интервала
    count: usize,
}

/// Конфигурация детекции потери lock.
#[derive(Debug, Clone, Copy)]
pub struct LockDetectorConfig {
    /// Размер скользящего окна фазовых ошибок (число последних обновлений)
    pub window_size: usize,

    /// Порог стандартного отклонения фазовой ошибки (рад), при превышении
    /// которого lock считается потерянным.
    pub phase_std_threshold_rad: f32,

    /// Порог C/N₀ (дБ-Гц), ниже которого lock считается потерянным.
    pub cn0_threshold_db_hz: f32,

    /// Минимальное число обновлений в окне для принятия решения
    /// (защита от ложного срабатывания сразу после захвата).
    pub min_samples: usize,
}

/// Конфигурация PLL.
#[derive(Debug, Clone)]
pub struct PllConfig {
    /// Шумовая полоса петли (Hz). Типично 10–25 Hz.
    pub bandwidth_hz: f32,
    /// Период когерентного накопления (мс), 1–20.
    pub integration_ms: usize,
    /// Конфигурация детектора потери lock.
    pub lock_detector: LockDetectorConfig,
    /// Ограничение выхода фильтра (Hz) — защита от насыщения.
    pub output_clamp_hz: f32,
}

/// Коэффициент петлевого фильтра третьего порядка.
///
/// Получены из шумовой полосы `Bₗ` приближением для критически
/// демпфированной петли третьего порядка (множители Уильямса):
///
/// ```text
/// ωₙ = Bₗ / 0.7845
/// a₁ = 1.1 · ωₙ
/// a₂ = 2.4 · ωₙ²
/// a₃ = 1.1 · ωₙ³
/// ```
#[derive(Debug, Clone, Copy)]
pub struct PllFilterCoeffs {
    /// Коэффициент пропорционального (фазового) звена
    pub a1: f32,

    /// Коэффициент звена частоты (первый интегратор)
    pub a2: f32,

    /// Коэффициент звена ускорения частоты (второй интегратор)
    pub a3: f32,
}

/// Петлевой фильтр третьего порядка для PLL.
#[derive(Debug, Clone)]
pub struct PllLoopFilter {
    coeffs: PllFilterCoeffs,

    /// Интегратор второго порядка (накопленное ускорение частоты)
    acc2: f32,

    /// Интегратор первого порядка (накопленная частота)
    acc1: f32,

    /// Период обновления (с) - соответствует периоду когерентного накопления
    update_period_s: f32,
}

/// Детектор потери lock на основе дисперсии фазовой ошибки и C/N₀.
#[derive(Debug, Clone)]
struct LockDetector {
    config: LockDetectorConfig,
    phase_errors: VecDeque<f32>,
    prompt_history: Vec<Complex32>,
}

/// Выход одного цикла обновления PLL.
#[derive(Debug, Clone, Copy)]
pub struct PllOutput {
    /// Текущая накопленная фаза несущей (рад, развёрнутая, не по модулю 2π)
    pub carrier_phase_rad: f64,

    /// Текущая оценка частоты несущей (Гц), включая Doppler.
    pub carrier_freq_hz: f64,

    /// Ошибка дискриминатора текущей эпохи (рад для PLL, Гц для FLL).
    pub discriminator_output: f32,

    /// Поправка петлевого фильтра (Гц).
    pub filter_output: f32,

    /// Состояние контура после этого обновления.
    pub state: PllState,

    /// `true`, если в этом вызове было выполнено когерентное накопление
    /// (т.е. набралось `integration_ms` эпох и дискриминатор отработал).
    pub coherent_epoch_completed: bool,
}

/// Метрики производительности PLL для бенчмарков.
///
/// `time_to_lock_ms` и `steady_state_error_rad` - ключевые показатели,
/// требуемые для оценки качества контура.
#[derive(Debug, Clone, Copy, Default)]
pub struct PllBenchmarkMetrics {
    /// Время от начала захвата (первого `update`) до перехода в `PllLock`,
    /// в миллисекундах. `None`, если PLL-lock ещё не достигнут.
    pub time_to_lock_ms: Option<f64>,

    /// Стандартное отклонение фазовой ошибки в установившемся режиме (рад),
    /// вычисленное по последнему скользящему окну детектора lock.
    pub steady_state_phase_error_rad: f32,

    /// Текущая оценка C/N₀ (дБ-Гц), если доступна.
    pub cn0_db_hz: Option<f32>,

    /// Общее число обработанных эпох (1 мс каждая) с момента создания/сброса.
    pub total_epochs: u64,
}

/// PLL (Costas Loop) tracking
#[derive(Debug, Clone)]
pub struct Pll {
    config: PllConfig,
    filter: PllLoopFilter,
    accumulator: CoherentAccumulator,
    lock_detector: LockDetector,

    carrier_phase_rad: f64,
    carrier_freq_hz: f64,

    state: PllState,
    total_epochs: u64,
    locked_at_epoch: Option<u64>,
}

impl CoherentAccumulator {
    /// Создаёт накопитель на `integration_ms` миллисекунд (1 эпоха = 1 мс).
    ///
    /// # Panics
    ///
    /// Паникует если `integration_ms` вне диапазона `1..=20`.
    #[must_use]
    pub fn new(integration_ms: usize) -> Self {
        assert!(
            (1..=20).contains(&integration_ms),
            "coherent integration must be 1-20 ms, got {integration_ms}"
        );

        Self {
            traget_epochs: integration_ms,
            sum: Complex32::default(),
            count: 0,
        }
    }

    /// Добавляет одну 1-мс эпоху Prompt-корреляции.
    ///
    /// Возвращает `Some(accumulated_prompt)`, когда накоплено
    /// `target_apochs` эпох (сумма сбрасывается), иначе `None`.
    #[must_use]
    pub fn push(
        &mut self,
        prompt: Complex32,
    ) -> Option<Complex32> {
        self.sum += prompt;
        self.count += 1;

        if self.count >= self.traget_epochs {
            let result = self.sum;

            self.sum = Complex32::default();
            self.count = 0;

            Some(result)
        } else {
            None
        }
    }

    /// Настроенный период накопления (мс).
    #[must_use]
    pub const fn integration_ms(&self) -> usize {
        self.traget_epochs
    }

    /// Сбрасывает накопленную сумму без изменения настройки периода.
    pub fn reset(&mut self) {
        self.sum = Complex32::default();
        self.count = 0;
    }
}

impl PllFilterCoeffs {
    /// Вычисляет коэффициент из шумовой полосы петли.
    ///
    /// # Panics
    ///
    /// Паникует если `bandwidth_hz <= 0.0`.
    #[must_use]
    pub fn new(bandwidth_hz: f32) -> Self {
        assert!(bandwidth_hz > 0.0, "bandwidth must be positive");

        let omega_n = bandwidth_hz / 0.7845;

        Self {
            a1: 1.1 * omega_n,
            a2: 2.4 * omega_n,
            a3: 1.1 * omega_n.powi(3),
        }
    }
}

impl PllLoopFilter {
    /// Создаёт фильтр с заданной полосой и периодом обновления.
    #[must_use]
    pub fn new(
        bandwidth_hz: f32,
        update_period_s: f32,
    ) -> Self {
        Self {
            coeffs: PllFilterCoeffs::new(bandwidth_hz),
            acc2: 0.0,
            acc1: 0.0,
            update_period_s,
        }
    }

    /// Обновляеь фильтр одной ошибкой дискриминатора (рад) и возвращает
    /// поправку к частоте несущей (Гц).
    #[must_use]
    pub fn update(
        &mut self,
        phase_error_rad: f32,
    ) -> f32 {
        let t = self.update_period_s;

        self.acc2 += phase_error_rad * self.coeffs.a3 * t;
        self.acc1 += (phase_error_rad * self.coeffs.a2 + self.acc2) * t;

        phase_error_rad * self.coeffs.a1 + self.acc1
    }

    /// Сбрасывает оба интегратора.
    pub const fn reset(&mut self) {
        self.acc1 = 0.0;
        self.acc2 = 0.0;
    }

    /// Текущее значение интегратора частоты (Гц) - для диагностики.
    #[must_use]
    pub const fn freq_integrator(&self) -> f32 {
        self.acc1
    }

    /// Принудительно устанавливает значение интегратора частоты — может
    /// использоваться при переинициализации `Pll` со внешней частотной
    /// поправкой (например, после повторного `Fll`-захвата), чтобы не
    /// терять накопленную динамику.
    pub const fn set_freq_integrator(
        &mut self,
        value: f32,
    ) {
        self.acc1 = value;
    }

    /// Текущее значение интегратора частоты (Гц) - для диагностики.
    #[must_use]
    pub const fn coeffs(&self) -> PllFilterCoeffs {
        self.coeffs
    }
}

impl LockDetector {
    pub fn new(config: LockDetectorConfig) -> Self {
        Self {
            config,
            phase_errors: VecDeque::with_capacity(config.window_size),
            prompt_history: Vec::with_capacity(20),
        }
    }

    fn push_phase_error(
        &mut self,
        error_rad: f32,
    ) {
        if self.phase_errors.len() >= self.config.window_size {
            self.phase_errors.pop_front();
        }

        self.phase_errors.push_back(error_rad);
    }

    fn push_prompt(
        &mut self,
        prompt: Complex32,
    ) {
        self.prompt_history.push(prompt);

        if self.prompt_history.len() > 20 {
            self.prompt_history.remove(0);
        }
    }

    /// `true`, если по накопленным данным lock следует считать потерянным.
    fn is_lost(&self) -> bool {
        if self.phase_errors.len() < self.config.min_samples {
            return false;
        }

        let n = self.phase_errors.len() as f32;
        let mean: f32 = self.phase_errors.iter().sum::<f32>() / n;
        let variance: f32 = self
            .phase_errors
            .iter()
            .map(|e| (e - mean).powi(2))
            .sum::<f32>()
            / n;
        let std_dev = variance.sqrt();

        if std_dev > self.config.phase_std_threshold_rad {
            return true;
        }

        if self.prompt_history.len() >= 2 {
            let cn0 = cn0_estimate(&self.prompt_history, 0.001);

            if cn0 > 0.0 && cn0 < self.config.cn0_threshold_db_hz {
                return true;
            }
        }

        false
    }

    fn current_phase_std_rad(&self) -> f32 {
        if self.phase_errors.len() < 2 {
            return 0.0;
        }

        let n = self.phase_errors.len() as f32;
        let mean: f32 = self.phase_errors.iter().sum::<f32>() / n;
        let variance: f32 = self
            .phase_errors
            .iter()
            .map(|e| (e - mean).powi(2))
            .sum::<f32>()
            / n;

        variance.sqrt()
    }

    fn current_cn0_db_hz(&self) -> Option<f32> {
        if self.prompt_history.len() < 2 {
            None
        } else {
            Some(cn0_estimate(&self.prompt_history, 0.001))
        }
    }

    fn reset(&mut self) {
        self.phase_errors.clear();
        self.prompt_history.clear();
    }
}

impl Pll {
    /// Создаёт PLL с заданной конфигурацией и начальной частотой несущей
    /// (как правило — финальной оценкой [`crate::tracking::fll::Fll`]
    /// после `complete_handoff()`, либо `doppler_hz` напрямую из
    /// acquisition, если FLL-стадия пропускается).
    #[must_use]
    pub fn new(
        config: PllConfig,
        initial_freq_hz: f64,
    ) -> Self {
        let period_s = config.integration_ms as f32 / 1000.0;
        let filter = PllLoopFilter::new(config.bandwidth_hz, period_s);
        let accumulator = CoherentAccumulator::new(config.integration_ms);
        let lock_detector = LockDetector::new(config.lock_detector);

        Self {
            config,
            filter,
            accumulator,
            lock_detector,
            carrier_phase_rad: 0.0,
            carrier_freq_hz: initial_freq_hz,
            state: PllState::Searching,
            total_epochs: 0,
            locked_at_epoch: None,
        }
    }

    /// Создаёт PLL с конфигурацией по умолчанию и заданным начальным Doppler.
    #[must_use]
    pub fn with_defaults(initial_doppler_hz: f64) -> Self {
        Self::new(PllConfig::default(), initial_doppler_hz)
    }

    /// Подаёт одну 1-мс Prompt-корреляцию в контур.
    ///
    /// Дискриминатор и петлевой фильтр срабатывают только когда
    /// когерентное накопление завершено; до этого возвращается
    /// промежуточный выход с `coherent_epoch_completed = false`.
    pub fn update(
        &mut self,
        prompt: Complex32,
    ) -> PllOutput {
        self.total_epochs += 1;

        self.carrier_phase_rad += TAU * self.carrier_freq_hz / 1000.0;

        let Some(accumulated) = self.accumulator.push(prompt) else {
            return PllOutput {
                carrier_phase_rad: self.carrier_phase_rad,
                carrier_freq_hz: self.carrier_freq_hz,
                discriminator_output: 0.0,
                filter_output: 0.0,
                state: self.state,
                coherent_epoch_completed: false,
            };
        };

        self.lock_detector.push_prompt(accumulated);

        let epl_view = EplOutput {
            early: Complex32::default(),
            prompt: accumulated,
            late: Complex32::default(),
        };
        let phase_error_rad = epl_view.pll_dd_atan();
        self.lock_detector.push_phase_error(phase_error_rad);

        if self.state != PllState::PllLock {
            self.state = PllState::PllLock;
            self.locked_at_epoch.get_or_insert(self.total_epochs);
        }

        let mut filter_output = self.filter.update(phase_error_rad);
        filter_output =
            filter_output.clamp(-self.config.output_clamp_hz, self.config.output_clamp_hz);

        self.carrier_freq_hz += f64::from(filter_output);

        if self.state == PllState::PllLock && self.lock_detector.is_lost() {
            self.state = PllState::LockLost;
            log::warn!(
                "PLL: lock lost (phase_std={:.3} rad)",
                self.lock_detector.current_phase_std_rad()
            );
        }

        PllOutput {
            carrier_phase_rad: self.carrier_phase_rad,
            carrier_freq_hz: self.carrier_freq_hz,
            discriminator_output: phase_error_rad,
            filter_output,
            state: self.state,
            coherent_epoch_completed: true,
        }
    }

    /// Текущая (развёрнутая) фаза несущей в радианах.
    #[must_use]
    pub const fn carrier_phase_rad(&self) -> f64 {
        self.carrier_phase_rad
    }

    /// Текущая оценка частоты несущей (Гц).
    #[must_use]
    pub const fn carrier_freq_hz(&self) -> f64 {
        self.carrier_freq_hz
    }

    /// Текущее состояние контура.
    #[must_use]
    pub const fn state(&self) -> PllState {
        self.state
    }

    /// Конфигурация PLL.
    #[must_use]
    pub const fn config(&self) -> &PllConfig {
        &self.config
    }

    /// Общее число обработанных 1-мс эпох.
    #[must_use]
    pub const fn total_epochs(&self) -> u64 {
        self.total_epochs
    }

    /// Снимок метрик для бенчмарка: время захвата и установившаяся ошибка.
    ///
    /// `time_to_lock_ms` вычисляется как `(pll_locked_at_epoch) × integration_ms`.
    #[must_use]
    pub fn benchmark_metrics(&self) -> PllBenchmarkMetrics {
        let time_to_lock_ms = self.locked_at_epoch.map(|epoch| epoch as f64);

        PllBenchmarkMetrics {
            time_to_lock_ms,
            steady_state_phase_error_rad: self.lock_detector.current_phase_std_rad(),
            cn0_db_hz: self.lock_detector.current_cn0_db_hz(),
            total_epochs: self.total_epochs,
        }
    }

    /// Полный сброс PLL с новой начальной (Doppler) частотой — используется
    /// после повторного acquisition при потере lock.
    pub fn reset(
        &mut self,
        initial_freq_hz: f64,
    ) {
        let period_s = self.config.integration_ms as f32 / 1000.0;
        self.filter = PllLoopFilter::new(self.config.bandwidth_hz, period_s);
        self.accumulator.reset();
        self.lock_detector.reset();

        self.carrier_phase_rad = 0.0;
        self.carrier_freq_hz = initial_freq_hz;
        self.state = PllState::Searching;
        self.total_epochs = 0;
        self.locked_at_epoch = None;
    }
}

impl Default for LockDetectorConfig {
    fn default() -> Self {
        Self {
            window_size: 50,
            phase_std_threshold_rad: 0.6, // ≈ 34°, типичный порог потери Costas-lock
            cn0_threshold_db_hz: 30.0,
            min_samples: 10,
        }
    }
}

impl Default for PllConfig {
    /// GPS L1 C/A defaults: полоса 18 Hz, 1 мс интеграция.
    fn default() -> Self {
        Self {
            bandwidth_hz: 18.0,
            integration_ms: 1,
            lock_detector: LockDetectorConfig::default(),
            output_clamp_hz: 5000.0,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    fn make_pll() -> Pll {
        Pll::with_defaults(0.0)
    }

    #[test]
    fn test_accumulator_returns_none_before_target_reached() {
        let mut acc = CoherentAccumulator::new(5);
        for _ in 0..4 {
            assert!(acc.push(Complex32::new(1.0, 0.0)).is_none());
        }
    }

    #[test]
    fn test_accumulator_returns_sum_at_target() {
        let mut acc = CoherentAccumulator::new(4);
        for _ in 0..3 {
            let _ = acc.push(Complex32::new(1.0, 0.0));
        }
        let result = acc.push(Complex32::new(1.0, 0.0)).unwrap();
        assert!((result.re - 4.0).abs() < 1e-6);
    }

    #[test]
    #[should_panic(expected = "1-20 ms")]
    fn test_accumulator_rejects_zero_integration() {
        let _ = CoherentAccumulator::new(0);
    }

    #[test]
    #[should_panic(expected = "1-20 ms")]
    fn test_accumulator_rejects_over_20ms() {
        let _ = CoherentAccumulator::new(21);
    }

    #[test]
    fn test_filter_coeffs_all_positive() {
        let c = PllFilterCoeffs::new(18.0);
        assert!(c.a1 > 0.0);
        assert!(c.a2 > 0.0);
        assert!(c.a3 > 0.0);
    }

    #[test]
    #[should_panic(expected = "bandwidth must be positive")]
    fn test_filter_coeffs_zero_bandwidth_panics() {
        let _ = PllFilterCoeffs::new(0.0);
    }

    #[test]
    fn test_loop_filter_zero_error_zero_output() {
        let mut f = PllLoopFilter::new(18.0, 0.001);
        assert!(f.update(0.0).abs() < 1e-6);
    }

    #[test]
    fn test_loop_filter_handles_constant_error_without_diverging_to_nan() {
        let mut f = PllLoopFilter::new(18.0, 0.001);
        for _ in 0..10_000 {
            assert!(f.update(0.05).is_finite());
        }
    }

    #[test]
    fn test_loop_filter_set_freq_integrator_overrides_value() {
        let mut f = PllLoopFilter::new(18.0, 0.001);
        f.set_freq_integrator(123.0);
        assert!((f.freq_integrator() - 123.0).abs() < 1e-6);
    }

    #[test]
    fn test_lock_detector_not_lost_with_few_samples() {
        let mut d = LockDetector::new(LockDetectorConfig::default());
        for _ in 0..5 {
            d.push_phase_error(2.0);
        }
        assert!(!d.is_lost());
    }

    #[test]
    fn test_lock_detector_triggers_on_high_phase_variance() {
        let cfg = LockDetectorConfig {
            window_size: 20,
            phase_std_threshold_rad: 0.3,
            min_samples: 5,
            ..LockDetectorConfig::default()
        };
        let mut d = LockDetector::new(cfg);
        for i in 0..20 {
            let v = if i % 2 == 0 { 1.0 } else { -1.0 };
            d.push_phase_error(v);
        }
        assert!(d.is_lost());
    }

    #[test]
    fn test_pll_starts_in_searching_state() {
        assert_eq!(make_pll().state(), PllState::Searching);
    }

    #[test]
    fn test_pll_enters_pll_lock_on_first_completed_epoch() {
        let mut pll = make_pll();
        let out = pll.update(Complex32::new(1.0, 0.0));
        assert!(out.coherent_epoch_completed);
        assert_eq!(out.state, PllState::PllLock);
    }

    #[test]
    fn test_pll_costas_discriminator_near_zero_for_aligned_phase() {
        let mut pll = make_pll();
        let aligned = Complex32::new(1.0, 0.0);
        pll.update(aligned);
        let out = pll.update(aligned);
        assert!(out.discriminator_output.abs() < 0.05);
    }

    #[test]
    fn test_pll_costas_removes_180_degree_bit_ambiguity() {
        let mut pll = make_pll();
        let positive_bit = Complex32::new(1.0, 0.0);
        let negative_bit = Complex32::new(-1.0, 0.0);

        pll.update(positive_bit);
        let out_pos = pll.update(positive_bit);
        let out_neg = pll.update(negative_bit);

        assert!(out_pos.discriminator_output.abs() < 0.2);
        assert!(out_neg.discriminator_output.abs() < 0.2);
    }

    #[test]
    fn test_pll_detects_loss_of_lock_under_erratic_phase() {
        let cfg = PllConfig {
            lock_detector: LockDetectorConfig {
                window_size: 10,
                phase_std_threshold_rad: 0.2,
                min_samples: 5,
                cn0_threshold_db_hz: -1000.0,
            },
            ..PllConfig::default()
        };
        let mut pll = Pll::new(cfg, 0.0);

        let stable = Complex32::new(1.0, 0.0);
        for _ in 0..5 {
            pll.update(stable);
        }
        assert_eq!(pll.state(), PllState::PllLock);

        let mut lost = false;
        for i in 0..30 {
            let angle = if i % 2 == 0 { 1.2_f32 } else { -1.2_f32 };
            let erratic = Complex32::new(angle.cos(), angle.sin());
            if pll.update(erratic).state == PllState::LockLost {
                lost = true;
                break;
            }
        }
        assert!(lost);
    }

    #[test]
    fn test_pll_benchmark_time_to_lock_recorded_immediately() {
        let mut pll = make_pll();
        pll.update(Complex32::new(1.0, 0.0));
        assert!(pll.benchmark_metrics().time_to_lock_ms.is_some());
    }

    #[test]
    fn test_pll_reset_returns_to_searching_with_new_freq() {
        let mut pll = make_pll();
        for _ in 0..10 {
            pll.update(Complex32::new(1.0, 0.0));
        }
        pll.reset(1500.0);
        assert_eq!(pll.state(), PllState::Searching);
        assert_eq!(pll.total_epochs(), 0);
        assert!((pll.carrier_freq_hz() - 1500.0).abs() < 1e-9);
    }

    #[test]
    fn test_pll_initial_freq_is_reflected_in_carrier_freq() {
        let pll = Pll::with_defaults(3500.0);
        assert!((pll.carrier_freq_hz() - 3500.0).abs() < 1e-9);
    }

    #[test]
    fn test_pll_remains_numerically_stable_over_many_epochs() {
        let mut pll = make_pll();
        let prompt = Complex32::new(1.0, 0.05);
        for _ in 0..5000 {
            let out = pll.update(prompt);
            assert!(out.carrier_freq_hz.is_finite());
            assert!(out.carrier_phase_rad.is_finite());
        }
    }

    #[test]
    fn test_pll_integration_with_correlator_epl_prompt() {
        use crate::signal::correlator::base::correlator_epl;

        let n = 64;
        let code = vec![1.0_f32; n];
        let signal = vec![Complex32::new(1.0, 0.0); n];
        let epl = correlator_epl(&signal, &code, &code, &code);

        let mut pll = make_pll();
        let out = pll.update(epl.prompt);
        assert!(out.coherent_epoch_completed);
        assert!(out.carrier_freq_hz.is_finite());
    }
}
