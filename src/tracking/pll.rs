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
//! ┌───────────────────────────────────────────────┐
//! │ FLL_LOCK: cross-product discriminator         │  ← начальный захват,
//! │     │  ошибка частоты                         │    широкая полоса
//! │     ▼                                         │
//! │ PLL_LOCK: Costas (dd_atan) discriminator      │  ← после захвата FLL,
//! │     │  ошибка фазы                            │    узкая полоса
//! └───────────────────────────────────────────────┘
//!     ▼
//! PllLoopFilter (3-й порядок: фаза + частота + ускорение частоты)
//!     │  поправка частоты несущей (Hz)
//!     ▼
//! Pll::carrier_freq_hz()  → carrier NCO
//! ```
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
//! # FLL-asusted PLL
//!
//! При первоначальном захвате частотная ошибка может быть слишком велика
//! для PLL (диапазон захвата PLL обычно ±¼ от полосы петли). FLL имеет
//! гораздо более широкий диапазон захвата и используется для грубой
//! предварительной синхронизации частоты перед переключением на PLL.
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

    /// FLL активен - грубая частотная синхронизация
    FllLock,

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
    /// Шумовая полоса петли в режиме PLL (Hz). Типично 10–25 Hz.
    pub pll_bandwidth_hz: f32,

    /// Шумовая полоса петли в режиме FLL (Hz). Шире, чем PLL — обычно 50–200 Hz,
    /// для быстрого грубого захвата.
    pub fll_bandwidth_hz: f32,

    /// Период когерентного накопления (мс), 1–20.
    pub integration_ms: usize,

    /// Число последовательных эпох FLL-lock (по критерию малой частотной
    /// ошибки), после которых контур переключается на PLL.
    pub fll_to_pll_stable_epochs: usize,

    /// Порог частотной ошибки FLL (Hz), ниже которого эпоха считается
    /// "стабильной" для перехода в PLL.
    pub fll_stable_threshold_hz: f32,

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
    pub(crate) acc1: f32,

    /// Период обновления (с) - соответствует периоду когерентного накопления
    update_period_s: f32,
}

/// Детектор потери lock на основе дисперсии фазовой ошибки и C/N₀.
#[derive(Debug, Clone)]
struct LockDetector {
    config: LockDetectorConfig,
    phase_errors: std::collections::VecDeque<f32>,
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

/// PLL (Costas Loop) tracking
#[derive(Debug, Clone)]
pub struct Pll {
    config: PllConfig,
    filter: PllLoopFilter,
    accumulator: CoherentAccumulator,
    lock_detector: LockDetector,
    carrier_phase_rad: f64,
    /// Текущая оценка частоты несущей.
    carrier_freq_hz: f64,
    state: PllState,
    fll_prev_prompt: Option<Complex32>,
    fll_stable_count: usize,
    total_epochs: u64,
    /// Эпоха (номер 1-мс такта), на которой был зафиксирован переход в
    /// `PllLock` - для вычисления `time_to_lock_ms`
    pll_locked_at_epoch: Option<u64>,
}

impl CoherentAccumulator {
    /// Создаёт накопитель на `integration_ms` миллисекунд (1 эпоха = 1 мс).
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

    /// Сбрасывает накопленную сумму без изменения настройки периода.
    pub fn reset(&mut self) {
        self.sum = Complex32::default();
        self.count = 0;
    }
}

impl PllFilterCoeffs {
    /// Вычисляет коэффициент из шумовой полосы петли.
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

    /// Текущее значение интегратора частоты (Гц) - для диагностики.
    #[must_use]
    pub const fn freq_integrator(&self) -> f32 {
        self.acc1
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
}

impl Pll {
    /// Создаёт PLL с заданной конфигурацией и начальной (Doppler) частотой
    /// несущей, полученной из acquisition.
    #[must_use]
    pub fn new(
        cfg: PllConfig,
        initial_doppler_hz: f64,
    ) -> Self {
        let period_s = cfg.integration_ms as f32 / 1000.0;
        let filter = PllLoopFilter::new(cfg.fll_bandwidth_hz, period_s);
        let accumulator = CoherentAccumulator::new(cfg.integration_ms);
        let lock_detector = LockDetector::new(cfg.lock_detector);

        Self {
            config: cfg,
            filter,
            accumulator,
            lock_detector,
            carrier_phase_rad: 0.0,
            carrier_freq_hz: initial_doppler_hz,
            state: PllState::Searching,
            fll_prev_prompt: None,
            fll_stable_count: 0,
            total_epochs: 0,
            pll_locked_at_epoch: None,
        }
    }

    /// Создаёт PLL с конфигурацией по умолчанию и заданным начальным Doppler.
    #[must_use]
    pub fn with_defaults(initial_doppler_hz: f64) -> Self {
        Self::new(PllConfig::default(), initial_doppler_hz)
    }

    /// Обновление PLL по Prompt коррелятору.
    pub fn update(
        &mut self,
        prompt: Complex32,
    ) -> PllOutput {
        self.total_epochs += 1;

        // Развёртка фазы несущей на каждой 1-мс эпохе по текущей оценке частоты
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

        if self.state == PllState::Searching {
            self.state = PllState::FllLock;
        }

        let (discriminator_output, filter_output) = match self.state {
            PllState::FllLock => self.step_fll(accumulated),
            PllState::PllLock | PllState::LockLost => self.step_pll(accumulated),
            PllState::Searching => unreachable!("handle above"),
        };

        self.carrier_freq_hz += f64::from(filter_output);

        // Детекция потери lock - только в режиме PllLock
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
            discriminator_output,
            filter_output,
            state: self.state,
            coherent_epoch_completed: true,
        }
    }

    fn step_fll(
        &mut self,
        accumulated: Complex32,
    ) -> (f32, f32) {
        let period_s = f64::from(self.config.integration_ms as f32) / 1000.0;

        let freq_error_hz = if let Some(prev) = self.fll_prev_prompt {
            fll_cross_product_discriminator(prev, accumulated, period_s)
        } else {
            0.0
        };

        self.fll_prev_prompt = Some(accumulated);

        // FLL-фильтр работает с частотной ошибкой напрямую как с фазовой
        // ошибкой укрупнённого первого порядка через тот же 3-полюсный
        // фильтр (уго широкая полоса делает это устойчивым для захвата).
        let filter_output = self.filter.update(freq_error_hz as f32);

        if freq_error_hz.abs() < f64::from(self.config.fll_stable_threshold_hz) {
            self.fll_stable_count += 1;
        } else {
            self.fll_stable_count = 0;
        }

        if self.fll_stable_count >= self.config.fll_to_pll_stable_epochs {
            log::debug!(
                "PLL: FLL stable for {} epochs, switching to PLL (bandwidth {} Hz)",
                self.fll_stable_count,
                self.config.pll_bandwidth_hz,
            );

            self.switch_to_pll();
        }

        (freq_error_hz as f32, filter_output)
    }

    fn step_pll(
        &mut self,
        accumulated: Complex32,
    ) -> (f32, f32) {
        // Costas (decision-directed atan) дискриминатор устраняет
        // 180°-неоднозначность навигационного бита.
        let epl_view = EplOutput {
            early: Complex32::default(),
            prompt: accumulated,
            late: Complex32::default(),
        };
        let phase_error_rad = epl_view.pll_dd_atan();

        self.lock_detector.push_phase_error(phase_error_rad);

        if self.state != PllState::PllLock {
            self.state = PllState::PllLock;
            self.pll_locked_at_epoch = Some(self.total_epochs);
        }

        let filter_output = self.filter.update(phase_error_rad);
        let filter_output =
            filter_output.clamp(-self.config.output_clamp_hz, self.config.output_clamp_hz);

        (phase_error_rad, filter_output)
    }

    /// Переключает контур из `FllLock` в `PllLock`, пересобирая петлевой
    /// фильтр на узкую полосу PLL и сохраняя текущую частотную поправку
    /// (черз интегратор), чтобы избежать скачка частоты при переключении.
    fn switch_to_pll(&mut self) {
        let preserved_freq_integrator = self.filter.freq_integrator();
        let period_s = self.config.integration_ms as f32 / 1000.0;

        self.filter = PllLoopFilter::new(self.config.pll_bandwidth_hz, period_s);
        self.filter.acc1 = preserved_freq_integrator;

        self.state = PllState::PllLock;
        self.pll_locked_at_epoch = Some(self.total_epochs);
        self.fll_prev_prompt = None;
    }

    /// Текущая оценка частоты несущей (Гц).
    #[must_use]
    pub const fn carrier_freq_hz(&self) -> f64 {
        self.carrier_freq_hz
    }
}

/// Cross-product дискриминатор частоты (FLL).
///
/// Использует два последовательных Prompt-накопления для оценки ошибки
/// частоты несущей:
///
/// ```text
/// cross = Im(P[k] · conj(P[k-1]))
/// dot   = Re(P[k] · conj(P[k-1]))
/// error = atan2(cross, dot) / (2π·T)
/// ```
///
/// где `T` - период между накоплениями (с). Результат - оценка частотной
/// ошибки в Гц, не зависящая от знака навигационного бита (произведение
/// двух последовательных Prompt устраняет 180°-скачки).
#[must_use]
pub fn fll_cross_product_discriminator(
    prev: Complex32,
    curr: Complex32,
    period_s: f64,
) -> f64 {
    if period_s <= 0.0 {
        return 0.0;
    }

    let cross = (curr * prev.conj()).im;
    let dot = (curr * prev.conj()).re;

    f64::from(cross.atan2(dot)) / (TAU * period_s)
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
    /// GPS L1 C/A по умолчанию: PLL 18 Гц, FLL 100 Гц, интеграция 1 мс.
    fn default() -> Self {
        Self {
            pll_bandwidth_hz: 18.0,
            fll_bandwidth_hz: 100.0,
            integration_ms: 1,
            fll_to_pll_stable_epochs: 5,
            fll_stable_threshold_hz: 25.0,
            lock_detector: LockDetectorConfig::default(),
            output_clamp_hz: 5000.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

        let result = acc.push(Complex32::new(1.0, 0.0));

        assert!(result.is_some());

        let sum = result.unwrap();

        assert!((sum.re - 4.0).abs() < 1e-6);
    }

    #[test]
    fn test_accumulator_resets_after_emitting() {
        let mut acc = CoherentAccumulator::new(2);

        let _ = acc.push(Complex32::new(1.0, 0.0));

        let first = acc.push(Complex32::new(1.0, 0.0)).unwrap();

        assert!((first.re - 2.0).abs() < 1e-6);

        // Следующий цикл начинается с нуля.
        assert!(acc.push(Complex32::new(3.0, 0.0)).is_none());

        let second = acc.push(Complex32::new(3.0, 0.0)).unwrap();

        assert!((second.re - 6.0).abs() < 1e-6);
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
    fn test_accumulator_accepts_full_issue_range() {
        for ms in [1usize, 10, 20] {
            let _ = CoherentAccumulator::new(ms);
        }
    }

    #[test]
    fn test_accumulator_reset_clears_partial_sum() {
        let mut acc = CoherentAccumulator::new(10);

        let _ = acc.push(Complex32::new(5.0, 0.0));

        acc.reset();

        // После сброса должно требоваться полных 10 эпох заново.
        for _ in 0..9 {
            assert!(acc.push(Complex32::new(1.0, 0.0)).is_none());
        }
        assert!(acc.push(Complex32::new(1.0, 0.0)).is_some());
    }

    #[test]
    fn test_fll_discriminator_zero_for_identical_consecutive_prompts() {
        let p = Complex32::new(1.0, 0.0);
        let err = fll_cross_product_discriminator(p, p, 0.001);

        assert!(err.abs() < 1e-6, "identical prompts → zero frequency error");
    }

    #[test]
    fn test_fll_discriminator_nonzero_for_rotating_phase() {
        // Симулируем частотную ошибку: фаза поворачивается между эпохами.
        let prev = Complex32::new(1.0, 0.0);
        let angle = 0.1_f32; // small phase rotation
        let curr = Complex32::new(angle.cos(), angle.sin());
        let err = fll_cross_product_discriminator(prev, curr, 0.001);

        assert!(
            err.abs() > 0.0,
            "rotating phase should produce nonzero freq error"
        );
    }

    #[test]
    fn test_fll_discriminator_sign_matches_rotation_direction() {
        let prev = Complex32::new(1.0, 0.0);
        let pos_rot = Complex32::new(0.1_f32.cos(), 0.1_f32.sin());
        let neg_rot = Complex32::new(0.1_f32.cos(), -0.1_f32.sin());

        let err_pos = fll_cross_product_discriminator(prev, pos_rot, 0.001);
        let err_neg = fll_cross_product_discriminator(prev, neg_rot, 0.001);

        assert!(err_pos > 0.0);
        assert!(err_neg < 0.0);
    }

    #[test]
    fn test_fll_discriminator_zero_period_returns_zero() {
        let p1 = Complex32::new(1.0, 0.0);
        let p2 = Complex32::new(0.0, 1.0);

        assert_eq!(fll_cross_product_discriminator(p1, p2, 0.0), 0.0);
    }
}
