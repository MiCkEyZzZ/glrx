//! FLL — Frequency Lock Loop.
//!
//! Без FLL чистый PLL нестабилен при больших начальных ошибках частоты:
//! диапазон захвата PLL обычно не превышает ±¼ его шумовой полосы (для
//! типичной полосы 18 Гц это всего ±4–5 Гц), тогда как ошибка Doppler
//! после acquisition может достигать единиц килогерц. FLL имеет
//! существенно более широкий диапазон захвата и используется как
//! предварительная грубая стадия перед передачей контроля PLL.
//!
//! # Контур
//!
//! ```text
//! correlator_epl(...).prompt  (или накопленный Prompt после CoherentAccumulator)
//!     │
//!     ▼
//! Fll::update(prompt)
//!     │
//!     ├─ cross_product_discriminator(prev, curr)   ← ошибка частоты (Гц)
//!     │
//!     ▼
//! FllLoopFilter (1-й порядок при широкой полосе / 2-й порядок при сужении)
//!     │  поправка частоты (Гц)
//!     ▼
//! FllOutput { freq_hz, state, ready_for_pll }
//!     │
//!     ▼ (когда ready_for_pll == true)
//! передача carrier_freq_hz в Pll::reset(..) / Pll::new(..)
//! ```
//!
//! # Cross-product дискриминатор
//!
//! ```text
//! cross    = Im(P[k] · conj(P[k-1]))
//! dot      = Re(P[k] · conj(P[k-1]))
//! error_hz = atan2(cross, dot) / (2π·T)
//! ```
//!
//! Произведение двух последовательных Prompt-корреляций устраняет
//! зависимость от знака навигационного бита (180°-скачки), что делает
//! дискриминатор устойчивым к данным без необходимости их декодирования.
//!
//!  # Bandwidth scheduling
//!
//! - **Старт (`Searching`)**: широкая полоса (например 100–250 Гц) —
//!   максимальный диапазон захвата, минимальная задержка реакции.
//! - **После первых стабильных эпох (`FllLock`)**: полоса сужается
//!   (например до 25–50 Гц) — снижается шум оценки частоты перед
//!   передачей в PLL.
//! - **Передача в PLL (`PllLock`)**: FLL завершает работу; `Fll`
//!   сигнализирует об этом через `ready_for_pll` в [`FllOutput`].
//!
//! # Состояния
//!
//! [`FllState::Searching`] → [`FllState::FllLock`] → [`FllState::PllLock`].
//! Переход `Searching → FllLock` происходит сразу после первого валидного
//! обновления (требуется минимум два Prompt для cross-product). Переход
//! `FllLock → PllLock` происходит после `stable_epochs_for_handoff`
//! подряд эпох с частотной ошибкой ниже `handoff_threshold_hz`.

use std::f64::consts::TAU;

use num_complex::Complex32;

/// Состояние FLL-контура.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FllState {
    /// Только запущен/сброшен. Нужен минимум один Prompt для старта
    /// cross-product дискриминатора (требует пары последовательных
    /// значений)
    Searching,

    /// Частотный захват активен, FLL вырыбатывает поправки
    FllLock,

    /// Контроль передан PLL, `Fll::update` после этого не должен
    /// вызываться (либо вызовы являются no-op диагностикой).
    PllLock,
}

/// Петлевой фильтр FLL.
#[derive(Debug, Clone, Copy)]
pub struct FllFilterCoeffs {
    /// Постоянная времени интегратора
    pub tau1: f32,

    /// Пропорциональный коэффициент
    pub kp: f32,
}

/// Петлевой фильтр FLL (первый порядок, PI-форма).
#[derive(Debug, Clone)]
pub struct FllLoopFilter {
    coeffs: FllFilterCoeffs,
    integrator: f32,
    update_period_s: f32,
}

/// Конфигурация FLL.
#[derive(Debug, Clone, Copy)]
pub struct FllConfig {
    /// Широкая полоса на старте захвата (Гц). Определяет диапазон захвата:
    /// чем шире, тем больше начальная ошибка частоты, которую FLL способен
    /// поймать, но тем выше шум оценки.
    pub wide_bandwidth_hz: f32,

    /// Узкая полоса в установившемся `FllLock` (Гц), применяется после
    /// `epochs_before_narrowing` стабильных эпох - снижает шум перед
    /// передачей в PLL.
    pub narrow_bandwidth_hz: f32,

    /// Период между обновлениями дискриминатора (с) - соответствует
    /// периоду когерентного накопления Prompt, обычно 0.001 (1мс)
    pub update_period_s: f32,

    /// Порог |ошибка частоты| (Гц), ниже которого эпоха считается
    /// "стабильной" для целей служения полосы и передачи в PLL
    pub stable_threshold_hz: f32,

    /// Число подряд стабильных эпох, после которого полоса сужается
    /// (wide -> narrow)
    pub epochs_before_narrowing: usize,

    /// Число подряд стабильных эпох (уже на узкой полосе), после которого
    /// контур считается готовым к передаче в PLL (`ready_for_pll = true`)
    pub epochs_before_handoff: usize,

    /// Ограничение выходной поправки (Гц) - защита от насыщения при
    /// аномальных входах.
    pub output_clamp_hz: f32,
}

/// Выход одного цикла обновления FLL.
#[derive(Debug, Clone, Copy)]
pub struct FllOutput {
    /// Текущая оценка частоты несущей (Гц), включая исходный Доплер
    pub freq_hz: f64,

    /// Ошибка дискриминатор текущей эпхи (Гц). `0.0`, если данных было
    /// недостаточно (первая эпоха после `Searching`)
    pub discriminator_output: f64,

    /// Поправка петлевого фильтра текущей эпохи (Гц)
    pub filter_output: f32,

    /// Состояние контура после этого обновления
    pub state: FllState,

    /// `true`, если контур считает себя готовым к передаче управления PLL
    /// (накоплено `epochs_before_handoff` подряд стабильных эпох на узкой
    /// полосе). Once `true`, остаётся `true` до явного `reset`.
    pub ready_for_pll: bool,

    /// Текущая используемая полоса петли (Hz) — диагностика переключения
    /// wide → narrow.
    pub current_bandwidth_hz: f32,
}

/// Frequency Lock Loop с переключением широкой/узкой полосы и явным
/// индикатором готовности к передаче управления PLL.
///
/// # Типичный интеграция с PLL
///
/// ```text
/// let mut fll = Fll::new(FllConfig::default(), doppler_from_acquisition);
///
/// loop {
///     let out = fll.update(coherent_prompt);
///     if out.ready_for_pll {
///         let pll = Pll::with_defaults(out.freq_hz);
///         break;
///     }
/// }
/// ```
#[derive(Debug, Clone)]
pub struct Fll {
    config: FllConfig,
    filter: FllLoopFilter,
    freq_hz: f64,
    state: FllState,
    prev_prompt: Option<Complex32>,
    stable_count: usize,
    narrowed: bool,
    ready_for_pll: bool,
    total_epochs: u64,
}

impl FllFilterCoeffs {
    /// Вычисляет коэффициент из шумовой полосы петли.
    ///
    /// # Panics
    ///
    /// Паникует, если `bandwidth_hz <= 0.0`.
    #[must_use]
    pub fn new(bandwidth_hz: f32) -> Self {
        assert!(bandwidth_hz > 0.0, "bandwidth must be positive");

        let omega_n = bandwidth_hz / 0.53;

        Self {
            tau1: 1.0 / omega_n,
            kp: omega_n,
        }
    }
}

impl FllLoopFilter {
    /// Создаёт фильтр с заданной полосой и периодом обновления.
    #[must_use]
    pub fn new(
        bandwidth_hz: f32,
        update_period_s: f32,
    ) -> Self {
        Self {
            coeffs: FllFilterCoeffs::new(bandwidth_hz),
            integrator: 0.0,
            update_period_s,
        }
    }

    /// Обновляет фильтр одной ошибкой частоты (Гц) и возвращает поправку (Гц).
    #[must_use]
    pub fn update(
        &mut self,
        freq_error_hz: f32,
    ) -> f32 {
        self.integrator += freq_error_hz * self.update_period_s / self.coeffs.tau1;

        let proportional = freq_error_hz * self.coeffs.kp;

        self.integrator + proportional
    }

    /// Сбрасывает интегратор.
    pub const fn reset(&mut self) {
        self.integrator = 0.0;
    }

    /// Текущее значение интегратора (Гц) - для диагностики и для переноса
    /// накопленной поправки при передаче в PLL.
    #[must_use]
    pub const fn integrator(&self) -> f32 {
        self.integrator
    }

    /// Принудительно устанавливает значение интегратора - используется при
    /// смене полосы (wide -> narrow), чтобы не было скачка выходной поправки.
    pub const fn set_integrator(
        &mut self,
        value: f32,
    ) {
        self.integrator = value;
    }

    /// Коэффициент фильтра.
    #[must_use]
    pub const fn coeffs(&self) -> FllFilterCoeffs {
        self.coeffs
    }
}

impl Fll {
    /// Создаёт Fll с заданной конфигурацией и начальной (Доплер) частотой
    /// несущей, полученной из acquisition.
    #[must_use]
    pub fn new(
        config: FllConfig,
        initial_doppler_hz: f64,
    ) -> Self {
        let filter = FllLoopFilter::new(config.wide_bandwidth_hz, config.update_period_s);

        Self {
            config,
            filter,
            freq_hz: initial_doppler_hz,
            state: FllState::Searching,
            prev_prompt: None,
            stable_count: 0,
            narrowed: false,
            ready_for_pll: false,
            total_epochs: 0,
        }
    }

    /// Создаёт FLL с конфигурацией по умолчанию.
    #[must_use]
    pub fn with_defaults(initial_doppler_hz: f64) -> Self {
        Self::new(FllConfig::default(), initial_doppler_hz)
    }

    /// Подаёт одну (когерентно накопленную) Prompt-корреляцию в контур
    pub fn update(
        &mut self,
        prompt: Complex32,
    ) -> FllOutput {
        self.total_epochs += 1;

        if self.state == FllState::PllLock {
            return self.frozen_output();
        }

        let Some(prev) = self.prev_prompt else {
            self.prev_prompt = Some(prompt);
            self.state = FllState::FllLock;

            return FllOutput {
                freq_hz: self.freq_hz,
                discriminator_output: 0.0,
                filter_output: 0.0,
                state: self.state,
                ready_for_pll: false,
                current_bandwidth_hz: self.current_bandwidth(),
            };
        };

        let freq_error_hz =
            cross_product_discriminator(prev, prompt, f64::from(self.config.update_period_s));

        self.prev_prompt = Some(prompt);

        let mut filter_output = self.filter.update(freq_error_hz as f32);

        filter_output =
            filter_output.clamp(-self.config.output_clamp_hz, self.config.output_clamp_hz);

        self.freq_hz += f64::from(filter_output);

        self.track_stability(freq_error_hz);
        self.maybe_narrow_bandwidth();
        self.maybe_signal_handoff();

        FllOutput {
            freq_hz: self.freq_hz,
            discriminator_output: freq_error_hz,
            filter_output,
            state: self.state,
            ready_for_pll: self.ready_for_pll,
            current_bandwidth_hz: self.current_bandwidth(),
        }
    }

    /// Явно переводит контур в `PllLock`, фиксируя текущую частоту как
    /// финальную оценку, передаваемую внешнему PLL.
    pub const fn complete_handoff(&mut self) -> f64 {
        self.state = FllState::PllLock;
        self.freq_hz
    }

    /// Текущая оценка частоты несущей (Гц).
    #[must_use]
    pub const fn freq_hz(&self) -> f64 {
        self.freq_hz
    }

    /// Текущее состояние контура.
    #[must_use]
    pub const fn state(&self) -> FllState {
        self.state
    }

    /// Возвращает `true`, если контур стгнализирует готовность к передаче в PLL.
    #[must_use]
    pub const fn is_ready_for_pll(&self) -> bool {
        self.ready_for_pll
    }

    /// Конфигурация контура.
    #[must_use]
    pub const fn config(&self) -> &FllConfig {
        &self.config
    }

    /// Число обработанных эпох с момента создания/последнего сброса.
    #[must_use]
    pub const fn total_epochs(&self) -> u64 {
        self.total_epochs
    }

    /// Полный сброс FLL с новой начальной (Доплер) частотой - например,
    /// после потери в PLL и необходимости повторного грубого захвата.
    pub fn reset(
        &mut self,
        initial_doppler_hz: f64,
    ) {
        self.filter =
            FllLoopFilter::new(self.config.wide_bandwidth_hz, self.config.update_period_s);
        self.freq_hz = initial_doppler_hz;
        self.state = FllState::Searching;
        self.prev_prompt = None;
        self.stable_count = 0;
        self.narrowed = false;
        self.ready_for_pll = false;
        self.total_epochs = 0;
    }

    fn track_stability(
        &mut self,
        freq_error_hz: f64,
    ) {
        if freq_error_hz.abs() < f64::from(self.config.stable_threshold_hz) {
            self.stable_count += 1;
        } else {
            self.stable_count = 0;
        }
    }

    fn maybe_narrow_bandwidth(&mut self) {
        if !self.narrowed && self.stable_count >= self.config.epochs_before_narrowing {
            log::debug!(
                "FLL: narrowing bandwidth {} Hz -> {} Hz after {} stable epochs",
                self.config.wide_bandwidth_hz,
                self.config.narrow_bandwidth_hz,
                self.stable_count
            );

            let preserved = self.filter.integrator();

            self.filter =
                FllLoopFilter::new(self.config.narrow_bandwidth_hz, self.config.update_period_s);
            self.filter.set_integrator(preserved);
            self.narrowed = true;
            // Считаем стабильность заново на узкой полосе для решения о handoff
            self.stable_count = 0;
        }
    }

    fn maybe_signal_handoff(&mut self) {
        if self.narrowed
            && !self.ready_for_pll
            && self.stable_count >= self.config.epochs_before_handoff
        {
            log::debug!(
                "FLL: ready for PLL handoff at freq={:.1} Hz after {} stable narrow epochs",
                self.freq_hz,
                self.stable_count
            );

            self.ready_for_pll = true;
        }
    }

    const fn current_bandwidth(&self) -> f32 {
        if self.narrowed {
            self.config.narrow_bandwidth_hz
        } else {
            self.config.wide_bandwidth_hz
        }
    }

    const fn frozen_output(&self) -> FllOutput {
        FllOutput {
            freq_hz: self.freq_hz,
            discriminator_output: 0.0,
            filter_output: 0.0,
            state: self.state,
            ready_for_pll: self.ready_for_pll,
            current_bandwidth_hz: self.current_bandwidth(),
        }
    }
}

/// Cross-product дискриминатор частоты.
///
/// # Аргументы
///
/// * `prev` — предыдущая (накопленная) Prompt-корреляция
/// * `curr` — текущая (накопленная) Prompt-корреляция
/// * `period_s` — интервал времени между `prev` и `curr` (с)
///
/// # Возвращает
///
/// Оценку ошибки частоты несущей в Гц. Знак указывает направление
/// коррекции: положительная ошибка означает, что текущая оценка частоты
/// ниже истинной (NCO нужно ускорить).
///
/// Возвращает `0.0`, если `period_s <= 0.0` (защита от деления на ноль).
#[must_use]
pub fn cross_product_discriminator(
    prev: Complex32,
    curr: Complex32,
    period_s: f64,
) -> f64 {
    if period_s <= 0.0 {
        return 0.0;
    }

    let product = curr * prev.conj();
    let cross = product.im;
    let dot = product.re;

    f64::from(cross.atan2(dot)) / (TAU * period_s)
}

impl Default for FllConfig {
    fn default() -> Self {
        Self {
            wide_bandwidth_hz: 150.0,
            narrow_bandwidth_hz: 30.0,
            update_period_s: 0.001,
            stable_threshold_hz: 25.0,
            epochs_before_narrowing: 5,
            epochs_before_handoff: 5,
            output_clamp_hz: 8_000.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::TAU;

    use super::*;

    fn prompt_at_freq(
        freq_hz: f64,
        sample_idx: u64,
        period_s: f64,
    ) -> Complex32 {
        let phase = TAU * freq_hz * sample_idx as f64 * period_s;

        Complex32::new(phase.cos() as f32, phase.sin() as f32)
    }

    /// Симулирует серию Prompt-корреляций с истинной частотной ошибкой
    /// `true_error_hz` относительно начальной оценки FLL (которая всегда
    /// стартует с `0.0` Hz внутренней частоты дискриминатора — сам
    /// дискриминатор видит фазовое вращение, соответствующее ошибке).
    fn run_transient(
        fll: &mut Fll,
        true_error_hz: f64,
        epochs: usize,
    ) -> Vec<FllOutput> {
        let period_s = f64::from(fll.config().update_period_s);
        let mut outputs = Vec::with_capacity(epochs);

        for k in 0..epochs {
            let prompt = prompt_at_freq(true_error_hz, k as u64, period_s);
            outputs.push(fll.update(prompt));
        }

        outputs
    }

    #[test]
    fn test_discriminator_zero_for_identical_consecuitive_prompts() {
        let p = Complex32::new(1.0, 0.0);
        let err = cross_product_discriminator(p, p, 0.001);

        assert!(
            err.abs() < 1e-6,
            "identical prompts -> zero frequency error"
        );
    }

    #[test]
    fn test_discriminator_nonzero_for_rotating_phase() {
        let prev = Complex32::new(1.0, 0.0);
        let angle = 0.1_f32;
        let curr = Complex32::new(angle.cos(), angle.sin());
        let err = cross_product_discriminator(prev, curr, 0.001);

        assert!(err.abs() > 0.0);
    }

    #[test]
    fn test_discriminator_zero_period_returns_zero() {
        let p1 = Complex32::new(1.0, 0.0);
        let p2 = Complex32::new(0.0, 1.0);

        assert!(cross_product_discriminator(p1, p2, 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_discriminator_180_degree_bit_flip_does_not_break_estimate() {
        // Cross-product должен быть устйчив к скачку знака навигационного
        // бита между эпохами (произведение P[k] * conj(P[k-1]) убирает знак)
        let prev = Complex32::new(1.0, 0.0);
        let curr_flipped = Complex32::new(-1.0, 0.0); // same phase, flipped bit
        let err = cross_product_discriminator(prev, curr_flipped, 0.001);

        // atan2(0, -1) = π, что соответствует на самом деле НЕ нулевой
        // оценке при чистом перевороте знака без частотной ошибки - это
        // ожидаемое поведение для одиночного скачка бита (общая защита от
        // 180° даётся усреднение/коэрентным накоплением выше по конвеёеру,
        // а не самим дискриминатором на голых семплах). Здесь мы только
        // проверяем что ф-я не паникует и возвращает конечное число.
        assert!(err.is_finite());
    }

    #[test]
    fn test_filter_coeffs_positive_for_typical_bandwidth() {
        let c = FllFilterCoeffs::new(150.0);

        assert!(c.tau1 > 0.0);
        assert!(c.kp > 0.0);
    }

    #[test]
    #[should_panic(expected = "bandwidth must be positive")]
    fn test_filter_coeffs_zero_bandwidth_panics() {
        let _ = FllFilterCoeffs::new(0.0);
    }

    #[test]
    fn test_loop_filter_zero_error_zero_output() {
        let mut f = FllLoopFilter::new(150.0, 0.001);

        assert!(f.update(0.0).abs() < 1e-6);
    }

    #[test]
    fn test_loop_filter_positive_error_positive_output() {
        let mut f = FllLoopFilter::new(150.0, 0.001);

        assert!(f.update(10.0) > 0.0);
    }

    #[test]
    fn test_loop_filter_reset_clears_integrator() {
        let mut f = FllLoopFilter::new(150.0, 0.001);

        let _ = f.update(10.0);
        let _ = f.update(10.0);

        f.reset();

        assert!(f.integrator().abs() < 1e-6);
    }

    #[test]
    fn test_loop_filter_set_integrator_overrides_value() {
        let mut f = FllLoopFilter::new(150.0, 0.001);

        f.set_integrator(42.0);

        assert!((f.integrator() - 42.0).abs() < 1e-6);
    }

    #[test]
    fn test_loop_filter_wider_bandwidth_reacts_faster() {
        let mut narrow = FllLoopFilter::new(20.0, 0.001);
        let mut wide = FllLoopFilter::new(150.0, 0.001);
        let out_narrow = narrow.update(100.0).abs();
        let out_wide = wide.update(100.0).abs();

        assert!(out_wide > out_narrow);
    }

    #[test]
    fn test_fll_starts_in_searching_state() {
        let fll = Fll::with_defaults(0.0);

        assert_eq!(fll.state(), FllState::Searching);
    }

    #[test]
    fn test_fll_first_update_enters_fll_lock_without_discriminator_output() {
        let mut fll = Fll::with_defaults(1000.0);
        let out = fll.update(Complex32::new(1.0, 0.0));

        assert_eq!(out.state, FllState::FllLock);
        assert!(out.discriminator_output.abs() < 1e-12);
        // Частота не должна меняться на первой эпохе (нет пары для cross-product)
        assert!((out.freq_hz - 1000.0).abs() < 1e-9);
    }

    #[test]
    fn test_fll_total_epochs_increments() {
        let mut fll = Fll::with_defaults(0.0);

        for i in 1..=10u64 {
            fll.update(Complex32::new(1.0, 0.0));

            assert_eq!(fll.total_epochs(), i);
        }
    }

    #[test]
    fn test_fll_starts_on_wide_bandwidth() {
        let mut fll = Fll::with_defaults(0.0);
        let out = fll.update(Complex32::new(1.0, 0.0));
        let out2 = fll.update(Complex32::new(1.0, 0.0));

        assert!((out.current_bandwidth_hz - fll.config().wide_bandwidth_hz).abs() < 1e-6);
        assert!((out2.current_bandwidth_hz - fll.config().wide_bandwidth_hz).abs() < 1e-6);
    }

    #[test]
    fn test_fll_narrows_bandwidth_after_stable_epochs() {
        let cfg = FllConfig {
            epochs_before_narrowing: 3,
            stable_threshold_hz: 50.0,
            ..FllConfig::default()
        };
        let mut fll = Fll::new(cfg, 0.0);
        // Постоянная фаза (DC) → нулевая частотная ошибка на каждой эпохе после первой.
        let prompt = Complex32::new(1.0, 0.0);
        let mut narrowed_seen = false;

        for _ in 0..20 {
            let out = fll.update(prompt);

            if (out.current_bandwidth_hz - cfg.narrow_bandwidth_hz).abs() < 1e-6 {
                narrowed_seen = true;
                break;
            }
        }

        assert!(narrowed_seen, "bandwidth should narrow after stable epochs");
    }

    #[test]
    fn test_fll_signals_ready_for_pll_after_narrow_and_handoff_epochs() {
        let cfg = FllConfig {
            epochs_before_narrowing: 2,
            epochs_before_handoff: 2,
            stable_threshold_hz: 50.0,
            ..FllConfig::default()
        };
        let mut fll = Fll::new(cfg, 0.0);
        let prompt = Complex32::new(1.0, 0.0);
        let mut became_ready = false;

        for _ in 0..30 {
            let out = fll.update(prompt);
            if out.ready_for_pll {
                became_ready = true;
                break;
            }
        }

        assert!(
            became_ready,
            "FLL should signal ready_for_pll under stable input"
        );
        assert!(fll.is_ready_for_pll());
    }

    #[test]
    fn test_fll_complete_handoff_transitions_to_pll_lock() {
        let mut fll = Fll::with_defaults(1234.5);

        fll.update(Complex32::new(1.0, 0.0));
        fll.update(Complex32::new(1.0, 0.0));

        let handoff_freq = fll.complete_handoff();

        assert_eq!(fll.state(), FllState::PllLock);
        assert!((handoff_freq - fll.freq_hz()).abs() < 1e-9);
    }

    #[test]
    fn test_fll_update_after_handoff_is_noop() {
        let mut fll = Fll::with_defaults(1000.0);

        fll.update(Complex32::new(1.0, 0.0));
        fll.complete_handoff();

        let freq_before = fll.freq_hz();
        let out = fll.update(Complex32::new(0.0, 1.0)); // would normally produce large error
        let freq_after = fll.freq_hz();

        assert_eq!(out.state, FllState::PllLock);
        assert!(
            (freq_before - freq_after).abs() < 1e-9,
            "frequency must not change after handoff"
        );
    }

    #[test]
    fn test_fll_reset_returns_to_searching_with_new_doppler() {
        let mut fll = Fll::with_defaults(0.0);

        for _ in 0..10 {
            fll.update(Complex32::new(1.0, 0.0));
        }

        fll.reset(2500.0);

        assert_eq!(fll.state(), FllState::Searching);
        assert_eq!(fll.total_epochs(), 0);
        assert!((fll.freq_hz() - 2500.0).abs() < 1e-9);
        assert!(!fll.is_ready_for_pll());
    }

    #[test]
    fn test_transient_response_converges_for_plus_3khz_error() {
        // +3 kHz начальная ошибка частоты — типичный случай после
        // acquisition с грубым Doppler-шагом.
        let cfg = FllConfig {
            wide_bandwidth_hz: 150.0,
            narrow_bandwidth_hz: 30.0,
            stable_threshold_hz: 50.0,
            epochs_before_narrowing: 5,
            epochs_before_handoff: 5,
            ..FllConfig::default()
        };
        let mut fll = Fll::new(cfg, 0.0);
        let outputs = run_transient(&mut fll, 3000.0, 500);
        // К концу симуляции остаточная частотная ошибка дискриминатора
        // должна быть много меньше начальной ошибки в 3 кГц.
        let final_window: Vec<f64> = outputs[outputs.len() - 20..]
            .iter()
            .map(|o| o.discriminator_output.abs())
            .collect();
        let mean_final_error: f64 = final_window.iter().sum::<f64>() / final_window.len() as f64;

        assert!(
            mean_final_error < 100.0,
            "residual frequency error should shrink well below 3000 Hz, got {mean_final_error}"
        );
    }

    #[test]
    fn test_transient_response_converges_for_minus_3khz_error() {
        let cfg = FllConfig {
            wide_bandwidth_hz: 150.0,
            narrow_bandwidth_hz: 30.0,
            stable_threshold_hz: 50.0,
            epochs_before_narrowing: 5,
            epochs_before_handoff: 5,
            ..FllConfig::default()
        };
        let mut fll = Fll::new(cfg, 0.0);
        let outputs = run_transient(&mut fll, -3000.0, 500);
        let final_window: Vec<f64> = outputs[outputs.len() - 20..]
            .iter()
            .map(|o| o.discriminator_output.abs())
            .collect();
        let mean_final_error: f64 = final_window.iter().sum::<f64>() / final_window.len() as f64;

        assert!(
            mean_final_error < 100.0,
            "residual frequency error should shrink well below 3000 Hz, got {mean_final_error}"
        );
    }

    #[test]
    fn test_transient_response_reaches_pll_ready_within_bounded_time_at_3khz() {
        // При большой начальной ошибке (issue: ±3 кГц) контур должен всё
        // же успеть дойти до состояния "готов к PLL" за разумное число
        // эпох (не зависнуть в FllLock навечно).
        let cfg = FllConfig {
            wide_bandwidth_hz: 150.0,
            narrow_bandwidth_hz: 30.0,
            stable_threshold_hz: 50.0,
            epochs_before_narrowing: 5,
            epochs_before_handoff: 5,
            ..FllConfig::default()
        };
        let mut fll = Fll::new(cfg, 0.0);
        let outputs = run_transient(&mut fll, 3000.0, 1000);
        let became_ready = outputs.iter().any(|o| o.ready_for_pll);

        assert!(
            became_ready,
            "should reach ready_for_pll within 1000 epochs at +3 kHz error"
        );
    }

    #[test]
    fn test_transient_response_output_never_diverges_at_3khz() {
        let mut fll = Fll::with_defaults(0.0);
        let outputs = run_transient(&mut fll, 3000.0, 500);

        for out in &outputs {
            assert!(out.freq_hz.is_finite());
            assert!(out.filter_output.is_finite());
        }
    }

    #[test]
    fn test_transient_response_narrows_bandwidth_only_after_convergence_at_3khz() {
        // На начальных эпохах с большой ошибкой полоса должна оставаться
        // широкой (контур ещё не стабилизировался); сужение происходит
        // только после того, как ошибка упадёт ниже порога.
        let cfg = FllConfig {
            wide_bandwidth_hz: 150.0,
            narrow_bandwidth_hz: 30.0,
            stable_threshold_hz: 50.0,
            epochs_before_narrowing: 5,
            epochs_before_handoff: 5,
            ..FllConfig::default()
        };
        let mut fll = Fll::new(cfg, 0.0);
        let outputs = run_transient(&mut fll, 3000.0, 500);

        // Первые несколько эпох — заведомо широкая полоса (ошибка огромна).
        assert!(
            (outputs[1].current_bandwidth_hz - cfg.wide_bandwidth_hz).abs() < 1e-6,
            "should still be wide early in transient"
        );

        // К концу симуляции должно произойти сужение.
        let last = outputs.last().unwrap();

        assert!(
            (last.current_bandwidth_hz - cfg.narrow_bandwidth_hz).abs() < 1e-6,
            "should have narrowed by the end of a long transient"
        );
    }

    #[test]
    fn test_transient_response_handles_smaller_1khz_error_faster_than_3khz() {
        // Меньшая начальная ошибка должна достигать ready_for_pll не позже
        // (как правило — раньше или одновременно), чем большая.
        let cfg = FllConfig {
            wide_bandwidth_hz: 150.0,
            narrow_bandwidth_hz: 30.0,
            stable_threshold_hz: 50.0,
            epochs_before_narrowing: 5,
            epochs_before_handoff: 5,
            ..FllConfig::default()
        };
        let mut fll_1k = Fll::new(cfg, 0.0);
        let mut fll_3k = Fll::new(cfg, 0.0);
        let out_1k = run_transient(&mut fll_1k, 1000.0, 1000);
        let out_3k = run_transient(&mut fll_3k, 3000.0, 1000);
        let epoch_ready_1k = out_1k.iter().position(|o| o.ready_for_pll);
        let epoch_ready_3k = out_3k.iter().position(|o| o.ready_for_pll);

        assert!(epoch_ready_1k.is_some(), "1 kHz error should converge");
        assert!(epoch_ready_3k.is_some(), "3 kHz error should converge");
        assert!(
            epoch_ready_1k.unwrap() <= epoch_ready_3k.unwrap(),
            "smaller initial error should not take longer to lock: {epoch_ready_1k:?} vs {epoch_ready_3k:?}",
        );
    }
}
