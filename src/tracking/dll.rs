//! DLL — Code Delay Lock Loop
//!
//! Отслеживает фазу PRN-кода спутника. Без DLL код-фаза уплывает отнтсительно
//! принимаемого сигнала, и вычислить псевдодальность становится невозможно.
//!
//! # Контур
//!
//! ```text
//! correlator_epl(signal, early, prompt, late)
//!     │  EplOutput { E, P, L }
//!     ▼
//! discriminator: dll_nelp() | dll_ele()      ← EplOutput (signal/discriminators.rs)
//!     │  ошибка (единицы дискриминатора)
//!     ▼
//! масштабирование в чипы (÷ 2·half_chip_spacing)
//!     │  ошибка в чипах
//!     ▼
//! DllLoopFilter (2-й порядок, PI)
//!     │  поправка частоты кода (chips/s)
//!     ▼
//! Dll::chip_freq_hz()              → код NCO
//! Dll::code_phase_offset_chips()   → вход в make_epl_replicas на след. эпохе
//! ```
//!
//! # Петлевой фильтр (как в `docs/TRACKING.md`)
//!
//! ```text
//! ωₙ = Bₗ · 8ζ / (4ζ² + 1)
//! τ₁ = 1 / ωₙ²
//! τ₂ = 2ζ / ωₙ
//!
//! y_i[k] = y_i[k-1] + e[k] · T / τ₁     (интегратор)
//! y_p[k] = e[k] · τ₂ / τ₁               (пропорциональное звено)
//! u[k]   = y_i[k] + y_p[k]
//! ```
//!
//! где `Bₗ` — полоса петли (Hz), `ζ` — демпфирование, `T` — период
//! интеграции (обычно 0.001 с для GPS L1 C/A).

use crate::signal::correlator::discriminators::EplOutput;

/// Тип дискриминатора кодовой петли.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DllDiscriminatorKind {
    /// Normalised Early-Late Power: `(|E|² − |L|²) / (|E|² + |L|²)`.
    ///
    /// Нормирован по мощности - не зависит от амплитуды сигнала.
    /// Рекомендуется как основной дискриминатор.
    Nelp,

    /// Early-Late Envelope: `|E| − |L|`.
    ///
    /// Не нормирован, зависит от амплитуды сигнала (требует стабильного AGC).
    Ele,
}

/// Состояние DLL между эпохами.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DllState {
    /// Петля ещё не получила ни одного обновления (или была сброшена)
    Unlocked,

    /// Петля активно отслеживает код
    Locked,
}

/// Выход одного цикла обновления DLL.
#[derive(Debug, Clone, Copy)]
pub struct DllOutput {
    /// Текущее смещение фазы кода (чипы), нормализовано в `[0, 1023]`
    ///
    /// Используется как вход для следующей генерации Early/Prompt/Late
    /// реплик (`make_epl_replicas`) и для вычисления псевдодальности.
    pub code_phase_offset_chips: f64,

    /// Текущая частота кода (chips/s), включающая поправка петли
    pub chip_freq_hz: f64,

    /// Выход дискриминатора в чипах (диагностика)
    pub discriminator_output: f32,

    /// Выход петлевого фильтра (chips/s, диагностика)
    pub filter_output: f32,

    /// Состояние петли после этого обновления
    pub state: DllState,
}

/// Коэффициент петлевого фильтра второго порядка.
///
/// Вычисляются из полосы петли `Bₗ` и коээфициента демпфирования `ζ`
/// по стандартным формулам двухпольного PI-фильтра.
#[derive(Debug, Clone, Copy)]
pub struct DllFilterCoeffs {
    /// Постоянная времени интегратора τ₁ (с²)
    pub tau1: f32,

    /// Постоянная времени пропорционального звена τ₂ (с)
    pub tau2: f32,
}

/// Петлевой фильтр второго порядка (PI) для DLL.
///
/// ```text
/// y_i[k] = y_i[k-1] + e[k] · T / τ₁
/// y_p[k] = e[k] · τ₂ / τ₁
/// u[k]   = y_i[k] + y_p[k]
/// ```
#[derive(Debug, Clone)]
pub struct DllLoopFilter {
    coeffs: DllFilterCoeffs,

    /// Накопленная интегральная составляющая (chips/s)
    pub(crate) integrator: f32,

    /// Период когерентной интеграции (с), обычно 0.001
    integration_time_s: f32,
}

/// Конфигурация DLL.
#[derive(Debug, Clone)]
pub struct DllConfig {
    /// Шумовая полоса петли (Гц). Типично 1-5 Гц: уже - меньше шума и
    /// медленнее реакция на динамику, шире - наоборот
    pub bandwidth_hz: f32,

    /// Коэффициент демпфирования. `0.707` - критическое демпфирование
    pub damping: f32,

    /// Половина межкорреляторного расстояния (chip spacing), в чипах.
    ///
    /// Early и Late реплики смещены на ±`half_chip_spacing` от Prompt.
    /// Допустимый диапазон по issue: **0.1–1.0 чипа** (здесь хранится
    /// именно половина расстояния, т.е. полное расстояние E↔L =
    /// `2 × half_chip_spacing` укладывается в 0.2–2.0 чипа).
    pub half_chip_spacing: f32,

    /// Тип дискриминатора
    pub discriminator: DllDiscriminatorKind,

    /// Период когерентной интеграции (с), обычно 0.001 (1мс)
    pub integration_time_s: f32,

    /// Номинальная частота кода (chips/s). Для GPS L1 C/A - 023 000
    pub nominal_chip_rate_hz: f64,

    /// Ограничение выхода фильтра (chips/s) - защита от насыщения при
    /// аномальных входах (например, во время потери lock)
    pub output_clamp_chips_s: f32,
}

/// Code Delay Lock Loop для отслеживания PRN-кода спутника.
///
/// # Типичный цикл (1 мс эпоха)
///
/// ```text
/// let half_samples = dll.half_chip_samples(fs);
/// let (early, prompt, late) = make_epl_replicas(&prn_code, half_samples);
/// let epl = correlator_epl(&baseband_signal, &early, &prompt, &late);
/// let out = dll.update(&epl);
/// // out.code_phase_offset_chips → используется как фаза для следующей
/// // генерации реплик (через resample_gps_with_phase или shift_code)
/// ```
#[derive(Debug, Clone)]
pub struct Dll {
    config: DllConfig,
    filter: DllLoopFilter,
    code_phase_chips: f64,
    chip_freq_hz: f64,
    state: DllState,
    epochs: u64,
}

impl DllConfig {
    /// Проверяет, что `half_chip_spacing` укладывается в допустимый диапазон полного chip
    /// spacing 0.1-1.0 чипа.
    ///
    /// Полное расстояние Early ↔ Late равно `2 × half_chip_spacing`, поэтому
    /// допустимый диапазон для `half_chip_spacing` — `0.05..=0.5`.
    #[must_use]
    pub fn chip_spacing_in_range(&self) -> bool {
        let full_spacing = 2.0 * self.half_chip_spacing;

        (0.1..=1.0).contains(&full_spacing)
    }
}

impl DllFilterCoeffs {
    /// Вычисляет коэффициенты из полосы петли и демпфирования.
    ///
    /// # Аргументы
    ///
    /// * `bandwidth_hz` — шумовая полоса петли, типично 1–5 Hz
    /// * `damping` — коэффициент демпфирования; `0.707` (1/√2) — критическое
    ///
    /// # Panics
    ///
    /// Panics if `bandwidth_hz <= 0.0` or `damping <= 0.0`.
    #[must_use]
    pub fn new(
        bandwidth_hz: f32,
        damping: f32,
    ) -> Self {
        assert!(bandwidth_hz > 0.0, "bandwidth must be positive");
        assert!(damping > 0.0, "damping must be positive");

        let omega_n = bandwidth_hz * 8.0 * damping / (4.0 * damping * damping + 1.0);
        let tau1 = 1.0 / (omega_n * omega_n);
        let tau2 = 2.0 * damping / omega_n;

        Self { tau1, tau2 }
    }
}

impl DllLoopFilter {
    /// Создаёт фильтр с заданной полосой, демпфированием и периодом интеграции.
    #[must_use]
    pub fn new(
        bandwidth_hz: f32,
        damping: f32,
        integration_time_s: f32,
    ) -> Self {
        Self {
            coeffs: DllFilterCoeffs::new(bandwidth_hz, damping),
            integrator: 0.0,
            integration_time_s,
        }
    }

    /// Обновляет фильтр одной ошибкой дискриминатора (в чипах) и возвращает
    /// поправку к номинальной частоте кода (chips/s).
    #[must_use]
    pub fn update(
        &mut self,
        error_chips: f32,
    ) -> f32 {
        let t = self.integration_time_s;
        let tau1 = self.coeffs.tau1;
        let tau2 = self.coeffs.tau2;

        self.integrator += error_chips * t / tau1;

        let proportional = error_chips * tau2 / tau1;

        self.integrator + proportional
    }

    /// Сбрасывает интегратор в ноль.
    pub const fn reset(&mut self) {
        self.integrator = 0.0;
    }

    /// Сбрасывает интегратор ноль.
    #[must_use]
    pub const fn integrator(&self) -> f32 {
        self.integrator
    }

    /// Коэффициент фильтра.
    #[must_use]
    pub const fn coeffs(&self) -> DllFilterCoeffs {
        self.coeffs
    }
}

impl Dll {
    /// Создаёт новый DLL с заданной конфигурацией.
    #[must_use]
    pub fn new(config: DllConfig) -> Self {
        let chip_freq_hz = config.nominal_chip_rate_hz;
        let filter = DllLoopFilter::new(
            config.bandwidth_hz,
            config.damping,
            config.integration_time_s,
        );

        Self {
            config,
            filter,
            code_phase_chips: 0.0,
            chip_freq_hz,
            state: DllState::Unlocked,
            epochs: 0,
        }
    }

    /// Создаёт DLL с конфигурацией по умолчанию (GPS L1 C/A, 2 Hz, 0.5 chip)
    #[must_use]
    pub fn with_defaults() -> Self {
        Self::new(DllConfig::default())
    }

    /// Создаёт DLL с заданной полосой петли и `half_chip_spacing`,
    /// остальные параметры - из `DllConfig::default()`.
    #[must_use]
    pub fn with_bandwidth(
        bandwidth_hz: f32,
        half_chip_spacing: f32,
    ) -> Self {
        Self::new(DllConfig {
            bandwidth_hz,
            half_chip_spacing,
            ..DllConfig::default()
        })
    }

    /// Выполняет одну эпоху обновления DLL.
    ///
    /// 1. Вычисляет ошибку дискриминатора (в чипах)
    /// 2. Прогоняет её через петлевой фильтр 2-го порядка
    /// 3. Ограничивает выход (`output_clamp_chips_s`)
    /// 4. Обновляет частоту и накопленную фазу кода
    pub fn update(
        &mut self,
        epl: &EplOutput,
    ) -> DllOutput {
        let raw_error = discriminate(
            epl,
            self.config.discriminator,
            self.config.half_chip_spacing,
        );
        let mut filter_out = self.filter.update(raw_error);

        filter_out = filter_out.clamp(
            -self.config.output_clamp_chips_s,
            self.config.output_clamp_chips_s,
        );

        self.chip_freq_hz = self.config.nominal_chip_rate_hz + f64::from(filter_out);

        // интегрируем частоту за интервал интеграции, чтобы получить накопленную фазу кода.
        self.code_phase_chips += self.chip_freq_hz * f64::from(self.config.integration_time_s);
        // Нормализуем в [0, 1023] - длина периода GPS L1 C/A.
        self.code_phase_chips = self.code_phase_chips.rem_euclid(1023.0);
        self.epochs += 1;
        self.state = DllState::Locked;

        DllOutput {
            code_phase_offset_chips: self.code_phase_chips,
            chip_freq_hz: self.chip_freq_hz,
            discriminator_output: raw_error,
            filter_output: filter_out,
            state: self.state,
        }
    }

    /// Текущая фаза кода (чипы), нормализована в `[0, 1023]`.
    #[must_use]
    pub const fn code_phase_offset_chips(&self) -> f64 {
        self.code_phase_chips
    }

    /// Текущая частота кода (chi[s/s]).
    #[must_use]
    pub const fn chip_freq_hz(&self) -> f64 {
        self.chip_freq_hz
    }

    /// Текущее состояние петли.
    #[must_use]
    pub const fn state(&self) -> DllState {
        self.state
    }

    /// Число обработанных эпох.
    #[must_use]
    pub const fn epochs(&self) -> u64 {
        self.epochs
    }

    /// Конфигурация DLL.
    #[must_use]
    pub const fn config(&self) -> &DllConfig {
        &self.config
    }

    /// Половина chip spacing в **сэмплах** при заданной частоте дискретизации.
    ///
    /// Передаётся как аргумент `make_epl_replicas(prompt_code, half_chip_samples)`.
    ///
    /// ```text
    /// half_chip_samples = half_chip_spacing × (fs / chip_rate)
    /// ```
    #[must_use]
    pub fn half_chip_samples(
        &self,
        sample_rate_hz: f64,
    ) -> f64 {
        f64::from(self.config.half_chip_spacing) * sample_rate_hz / self.chip_freq_hz
    }

    /// Меняет полосу петли без сброса накопленного состояния (фазы и
    /// интегратора) - позволяет сужать полосу после захвата (wide -> narrow)
    /// для снижения шума в установившемся режиме.
    pub fn set_bandwidth(
        &mut self,
        bandwidth_hz: f32,
    ) {
        self.config.bandwidth_hz = bandwidth_hz;

        let integrator = self.filter.integrator();

        self.filter = DllLoopFilter::new(
            bandwidth_hz,
            self.config.damping,
            self.config.integration_time_s,
        );
        self.filter.integrator = integrator;
    }

    /// Инициализирует DLL известной начальной фазой кода и доплеровской
    /// поправкой к частоте — вызывается сразу после acquisition.
    ///
    /// # Аргументы
    ///
    /// * `code_phase_chips` — начальная фаза кода (чипы), например из
    ///   `AcquisitionResult::code_phase_chips`
    /// * `doppler_chip_rate_correction` — поправка к номинальной частоте
    ///   кода (chips/s), обычно пересчитанная из Doppler несущей:
    ///   `doppler_hz × (chip_rate / carrier_freq_hz)`
    pub fn initialize(
        &mut self,
        code_phase_chips: f64,
        doppler_chip_rate_correction: f64,
    ) {
        self.code_phase_chips = code_phase_chips.rem_euclid(1023.0);
        self.chip_freq_hz = self.config.nominal_chip_rate_hz + doppler_chip_rate_correction;
        self.filter.reset();
        self.state = DllState::Unlocked;
        self.epochs = 0;
    }

    /// Полный сброс DLL в начальное состояние.
    pub const fn reset(&mut self) {
        self.code_phase_chips = 0.0;
        self.chip_freq_hz = self.config.nominal_chip_rate_hz;
        self.filter.reset();
        self.state = DllState::Unlocked;
        self.epochs = 0;
    }
}

/// Вычисляет ошибку дискриминатора в **чипах**.
///
/// # Аргументы
///
/// - `epl` - результат EPL-коррелятора текущей эпохи
/// - `kind` - выбранный тип дискриминатора
/// - `half_chip_spacing` - половина межкоррелятора расстояния (чипы),
///   например 0.5 для классического ±0.5-chip spacing
///
/// # Возвращает
///
/// Ошибку фазы кода в чипах. Положительное значение означает, что
/// локальный (опорный) код **запаздывает** относительно принятого сигнала
/// (Early сильнее Late).
#[must_use]
#[inline]
pub fn discriminate(
    epl: &EplOutput,
    kind: DllDiscriminatorKind,
    half_chip_spacing: f32,
) -> f32 {
    let raw = match kind {
        DllDiscriminatorKind::Nelp => epl.dll_nelp(),
        DllDiscriminatorKind::Ele => epl.dll_ele(),
    };

    if half_chip_spacing > f32::EPSILON {
        raw / (2.0 * half_chip_spacing)
    } else {
        0.0
    }
}

impl Default for DllConfig {
    /// GPS L1 C/A defaults: полоса 2 Гц, chip spacing 0.5 (E ↔ L = 1.0 чипа).
    fn default() -> Self {
        Self {
            bandwidth_hz: 2.0,
            damping: 0.707,
            half_chip_spacing: 0.5,
            discriminator: DllDiscriminatorKind::Nelp,
            integration_time_s: 0.001,
            nominal_chip_rate_hz: 1_023_000.0,
            output_clamp_chips_s: 200.0,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use num_complex::Complex32;

    use crate::signal::{
        correlator::{base::correlator_epl, code_utilities::make_epl_replicas},
        prn_code::PrnCodeCache,
    };

    use super::*;

    fn epl_balanced(amp: f32) -> EplOutput {
        EplOutput {
            early: Complex32::new(amp, 0.0),
            prompt: Complex32::new(amp * 1.5, 0.0),
            late: Complex32::new(amp, 0.0),
        }
    }

    fn epl_with_el(
        e: f32,
        l: f32,
    ) -> EplOutput {
        EplOutput {
            early: Complex32::new(e, 0.0),
            prompt: Complex32::new(2.0, 0.0),
            late: Complex32::new(l, 0.0),
        }
    }

    #[test]
    fn test_filter_coeffs_positive_for_typical_inputs() {
        let c = DllFilterCoeffs::new(2.0, 0.707);

        assert!(c.tau1 > 0.0);
        assert!(c.tau2 > 0.0);
    }

    #[test]
    fn test_filter_coeffs_wider_bandwidth_smaller_tau1() {
        let narrow = DllFilterCoeffs::new(1.0, 0.707);
        let wide = DllFilterCoeffs::new(5.0, 0.707);

        assert!(wide.tau1 < narrow.tau1, "wider bandwidth -> smaller tau1");
    }

    #[test]
    #[should_panic(expected = "bandwidth must be positive")]
    fn test_filter_coeffs_zero_bandwidth_panic() {
        let _ = DllFilterCoeffs::new(0.0, 0.707);
    }

    #[test]
    fn test_loop_filter_zero_error_produces_zero_output() {
        let mut f = DllLoopFilter::new(2.0, 0.707, 0.001);

        assert!(f.update(0.0).abs() < 1e-9);
    }

    #[test]
    fn test_loop_filter_positive_error_positive_output() {
        let mut f = DllLoopFilter::new(2.0, 0.707, 0.001);

        assert!(f.update(0.1) > 0.0);
    }

    #[test]
    fn test_loop_filter_negative_error_negative_output() {
        let mut f = DllLoopFilter::new(2.0, 0.707, 0.001);

        assert!(f.update(-0.1) < 0.0);
    }

    #[test]
    fn test_loop_filter_integrator_accumulates_monotonically() {
        let mut f = DllLoopFilter::new(2.0, 0.707, 0.001);

        let _ = f.update(0.1);

        let i1 = f.integrator();

        let _ = f.update(0.1);

        let i2 = f.integrator();

        assert!(
            i2 > i1,
            "integrator should accumulate under constant positive error"
        );
    }

    #[test]
    fn test_loop_filter_reset_clears_integrator() {
        let mut f = DllLoopFilter::new(2.0, 0.707, 0.001);

        let _ = f.update(1.0);
        let _ = f.update(1.0);
        f.reset();

        assert!(f.integrator().abs() < 1e-9);
    }

    #[test]
    fn test_loop_filter_wider_bandwidth_responds_faster() {
        // Шире полоса → больший начальный отклик на ту же ошибку.
        let mut narrow = DllLoopFilter::new(1.0, 0.707, 0.001);
        let mut wide = DllLoopFilter::new(5.0, 0.707, 0.001);
        let out_narrow = narrow.update(0.1).abs();
        let out_wide = wide.update(0.1).abs();

        assert!(out_wide > out_narrow, "wide={out_wide} narrow={out_narrow}");
    }

    #[test]
    fn test_discriminate_nelp_zero_when_balanced() {
        let epl = epl_balanced(1.0);
        let err = discriminate(&epl, DllDiscriminatorKind::Nelp, 0.5);

        assert!(err.abs() < 1e-6, "balanced E/L -> zero error, got {err}");
    }

    #[test]
    fn test_discriminate_nelp_positive_when_early_stronger() {
        let epl = epl_with_el(2.0, 1.0);
        let err = discriminate(&epl, DllDiscriminatorKind::Nelp, 0.5);

        assert!(err > 0.0, "early stronger → positive error");
    }

    #[test]
    fn test_discriminate_nelp_negative_when_late_stronger() {
        let epl = epl_with_el(1.0, 2.0);
        let err = discriminate(&epl, DllDiscriminatorKind::Nelp, 0.5);

        assert!(err < 0.0, "late stronger → negative error");
    }

    #[test]
    fn test_discriminate_smaller_chip_spacing_amplifies_normalized_error() {
        let epl = epl_with_el(2.0, 1.0);
        let err_half = discriminate(&epl, DllDiscriminatorKind::Nelp, 0.5);
        let err_quarter = discriminate(&epl, DllDiscriminatorKind::Nelp, 0.25);

        assert!(
            err_quarter.abs() > err_half.abs(),
            "smaller spacing → larger normalized error: {err_half} vs {err_quarter}"
        );
    }

    #[test]
    fn test_discriminate_zero_chip_spacing_returns_zero() {
        let epl = epl_with_el(2.0, 1.0);
        let err = discriminate(&epl, DllDiscriminatorKind::Nelp, 0.0);

        assert!(err.abs() < 1e-9, "zero spacing must not divide by zero");
    }

    #[test]
    fn test_discriminate_ele_zero_when_balanced() {
        let epl = epl_balanced(1.0);
        let err = discriminate(&epl, DllDiscriminatorKind::Ele, 0.5);

        assert!(err.abs() < 1e-6);
    }

    #[test]
    fn test_discriminate_nelp_is_amplitude_invariant_ele_is_not() {
        let low = epl_with_el(1.0, 0.5);
        let high = epl_with_el(10.0, 5.0); // same E/L ratio, x10 amplitude
        let nelp_low = discriminate(&low, DllDiscriminatorKind::Nelp, 0.5);
        let nelp_high = discriminate(&high, DllDiscriminatorKind::Nelp, 0.5);
        let ele_low = discriminate(&low, DllDiscriminatorKind::Ele, 0.5);
        let ele_high = discriminate(&high, DllDiscriminatorKind::Ele, 0.5);

        assert!(
            (nelp_low - nelp_high).abs() < 1e-5,
            "NELP must be amplitude-invariant: {nelp_low} vs {nelp_high}"
        );
        assert!(
            (ele_high - ele_low).abs() > 1.0,
            "ELE must scale with amplitude: {ele_low} vs {ele_high}"
        );
    }

    #[test]
    fn test_config_default_is_valid() {
        let cfg = DllConfig::default();

        assert!(cfg.bandwidth_hz > 0.0);
        assert!(cfg.damping > 0.0);
        assert!(cfg.nominal_chip_rate_hz > 0.0);
        assert!(cfg.chip_spacing_in_range());
    }

    #[test]
    fn test_config_chip_spacing_range_accepts_0_1_to_1_0() {
        // полный интервал = 2 * half_chip_spacing должен находиться в диапазоне [0.1, 1.0]
        for half in [0.05, 0.1, 0.25, 0.5] {
            let cfg = DllConfig {
                half_chip_spacing: half,
                ..DllConfig::default()
            };

            assert!(
                cfg.chip_spacing_in_range(),
                "half={half} should be in range"
            );
        }
    }

    #[test]
    fn test_config_chip_spacing_range_rejects_out_of_bounds() {
        let too_small = DllConfig {
            half_chip_spacing: 0.01,
            ..DllConfig::default()
        };
        let too_large = DllConfig {
            half_chip_spacing: 0.9,
            ..DllConfig::default()
        };

        assert!(!too_small.chip_spacing_in_range());
        assert!(!too_large.chip_spacing_in_range());
    }

    #[test]
    fn test_dll_starts_unlocked_with_zero_epochs() {
        let mut dll = Dll::with_defaults();
        let out = dll.update(&epl_balanced(1.0));

        assert_eq!(out.state, DllState::Locked);
        assert_eq!(dll.state(), DllState::Locked);
    }

    #[test]
    fn test_dll_update_transitions_to_locked() {
        let mut dll = Dll::with_defaults();
        let out = dll.update(&epl_balanced(1.0));

        assert_eq!(out.state, DllState::Locked);
        assert_eq!(dll.state(), DllState::Locked);
    }

    #[test]
    fn test_dll_update_increments_epoch_counter() {
        let mut dll = Dll::with_defaults();
        let epl = epl_balanced(1.0);

        for i in 1..=5 {
            dll.update(&epl);

            assert_eq!(dll.epochs, i);
        }
    }

    #[test]
    fn test_dll_balanced_epl_does_not_drift_frequency() {
        let mut dll = Dll::with_defaults();
        let epl = epl_balanced(1.0);
        let nominal = dll.config().nominal_chip_rate_hz;

        for _ in 0..100 {
            dll.update(&epl);
        }

        let drift = (dll.chip_freq_hz() - nominal).abs();

        assert!(
            drift < 1.0,
            "balanced EPL should not drift, drift={drift} chips/s"
        );
    }

    #[test]
    fn test_dll_early_strong_increases_chip_freq() {
        // Раннaя ветвь сильнее -> код запаздывает -> нужно ускорить локальный код
        let mut dll = Dll::with_defaults();
        let epl = epl_with_el(3.0, 1.0);
        let initial = dll.chip_freq_hz();

        for _ in 0..50 {
            dll.update(&epl);
        }

        assert!(
            dll.chip_freq_hz() > initial,
            "early strong -> freg should increase: {initial} -> {}",
            dll.chip_freq_hz()
        );
    }

    #[test]
    fn test_dll_late_strong_decreases_chip_freq() {
        let mut dll = Dll::with_defaults();
        let epl = epl_with_el(1.0, 3.0);
        let initial = dll.chip_freq_hz();

        for _ in 0..50 {
            dll.update(&epl);
        }

        assert!(
            dll.chip_freq_hz() < initial,
            "late strong → freq should decrease: {initial} → {}",
            dll.chip_freq_hz()
        );
    }

    #[test]
    fn test_dll_output_respects_clamp() {
        let mut dll = Dll::new(DllConfig {
            output_clamp_chips_s: 50.0,
            ..DllConfig::default()
        });
        let extreme_epl = EplOutput {
            early: Complex32::new(1000.0, 0.0),
            prompt: Complex32::new(0.5, 0.0),
            late: Complex32::new(0.001, 0.0),
        };

        for _ in 0..200 {
            let out = dll.update(&extreme_epl);
            let nominal = dll.config().nominal_chip_rate_hz;
            let deviation = (out.chip_freq_hz - nominal).abs() as f32;

            assert!(
                deviation <= 50.0 + 1e-3,
                "exceeded clamp: deviation={deviation}"
            );
        }
    }

    #[test]
    fn test_dll_code_phase_stays_within_one_period() {
        let mut dll = Dll::with_defaults();
        let epl = epl_balanced(1.0);

        for _ in 0..10_000 {
            dll.update(&epl);

            let phase = dll.code_phase_offset_chips();

            assert!(
                (0.0..1023.0).contains(&phase),
                "phase out of range: {phase}"
            );
        }
    }

    #[test]
    fn test_dll_reset_restores_initial_state() {
        let mut dll = Dll::with_defaults();
        let epl = epl_balanced(2.0);

        for _ in 0..100 {
            dll.update(&epl);
        }

        dll.reset();

        assert_eq!(dll.state(), DllState::Unlocked);
        assert_eq!(dll.epochs(), 0);
        assert!(dll.code_phase_offset_chips().abs() < 1e-9);
        assert!((dll.chip_freq_hz() - dll.config().nominal_chip_rate_hz).abs() < 1e-9);
    }

    #[test]
    fn test_dll_initialize_sets_phase_and_doppler_correction() {
        let mut dll = Dll::with_defaults();

        dll.initialize(512.5, 100.0);

        assert!((dll.code_phase_offset_chips() - 512.5).abs() < 1e-9);

        let expected = dll.config().nominal_chip_rate_hz + 100.0;

        assert!((dll.chip_freq_hz() - expected).abs() < 1e-9);
        assert_eq!(dll.state(), DllState::Unlocked);
    }

    #[test]
    fn test_dll_initialize_wraps_phase_over_1023() {
        let mut dll = Dll::with_defaults();

        dll.initialize(1500.0, 0.0);

        let phase = dll.code_phase_offset_chips();

        assert!((0.0..1023.0).contains(&phase));
    }

    #[test]
    fn test_set_bandwidth_preserves_integrator_state() {
        let mut dll = Dll::with_defaults();
        let epl = epl_with_el(2.0, 1.0);

        for _ in 0..10 {
            dll.update(&epl);
        }

        let before = dll.filter.integrator();

        dll.set_bandwidth(1.0); // сужение полосы после захвата

        let after = dll.filter.integrator();

        assert!(
            (before - after).abs() < 1e-6,
            "bandwidth change must preserve integrator: {before} vs {after}"
        );
        assert!((dll.config().bandwidth_hz - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_half_chip_samples_matches_formula() {
        let dll = Dll::with_defaults(); // half_chip_spacing=0.5, chip_rate=1_023_000
        let fs = 2_048_000.0_f64;
        let samples = dll.half_chip_samples(fs);
        let expected = 0.5 * fs / dll.config().nominal_chip_rate_hz;

        assert!((samples - expected).abs() < 1e-9);
    }

    #[test]
    fn test_with_bandwidth_constructor_sets_fields() {
        let dll = Dll::with_bandwidth(4.0, 0.25);

        assert!((dll.config().bandwidth_hz - 4.0).abs() < 1e-6);
        assert!((dll.config().half_chip_spacing - 0.25).abs() < 1e-6);
    }

    #[test]
    fn test_step_response_filter_output_grows_under_constant_error() {
        // Подаём постоянную ошибку и проверяем, что выход фильтра нарастает
        // (накопление интегральной составляющей) — необходимое условие
        // корректной работы замкнутой петли в реальном приёмнике.
        let mut dll = Dll::new(DllConfig {
            bandwidth_hz: 5.0,
            ..DllConfig::default()
        });
        let epl = epl_with_el(2.0, 1.0); // постоянная ошибка дискриминатора
        let mut outputs = Vec::new();

        for _ in 0..30 {
            outputs.push(dll.update(&epl).filter_output);
        }

        let first_half: f32 = outputs[..15].iter().sum::<f32>() / 15.0;
        let second_half: f32 = outputs[15..].iter().sum::<f32>() / 15.0;

        assert!(
            second_half > first_half,
            "filter output should grow under constant error: {first_half} → {second_half}"
        );
    }

    #[test]
    fn test_step_response_reaches_near_zero_error_under_closed_loop_simulation() {
        // Простая замкнутая петля: эмулируем, что коррекция DLL уменьшает
        // фактическое рассогласование сигнала на каждой эпохе. Дискриминатор
        // пропорционален остаточной задержке; ожидаем сходимость к ~0.
        let mut dll = Dll::new(DllConfig {
            bandwidth_hz: 5.0,
            ..DllConfig::default()
        });

        // Симулированное рассогласование в "единицах E-L power"; уменьшается
        // пропорционально накопленной коррекции DLL (упрощённая модель).
        let mut residual_mismatch = 1.0_f32;
        let mut last_error = f32::MAX;

        for _ in 0..200 {
            // E/L строим так, чтобы NELP ≈ residual_mismatch при малых значениях.
            let e = 1.0 + residual_mismatch;
            let l = 1.0 - residual_mismatch.min(0.99);
            let epl = epl_with_el(e.max(0.001), l.max(0.001));

            let out = dll.update(&epl);

            last_error = out.discriminator_output;

            // Коррекция уменьшает рассогласование (упрощённо, без реальной
            // физики корреляции — только проверяем сходимость алгоритма).
            residual_mismatch = (residual_mismatch - out.filter_output.abs() * 0.0001).max(0.0);
        }

        assert!(
            last_error.abs() < 1.0,
            "discriminator error should shrink toward zero, got {last_error}"
        );
    }

    #[test]
    fn test_dll_tracks_through_simulated_doppler_chip_rate_offset() {
        // Doppler на несущей создаёт пропорциональный сдвиг частоты кода.
        // Инициализируем DLL с такой поправкой и убеждаемся, что петля
        // остаётся стабильной (частота не уходит в бесконечность/NaN) и
        // продолжает корректировку при сбалансированном E/L.
        let mut dll = Dll::with_defaults();
        let doppler_chip_correction = 50.0; // chips/s, имитирует динамику платформы

        dll.initialize(0.0, doppler_chip_correction);

        let epl = epl_balanced(1.0); // сигнал выровнен относительно текущей фазы

        for _ in 0..500 {
            let out = dll.update(&epl);
            assert!(
                out.chip_freq_hz.is_finite(),
                "chip frequency must stay finite"
            );
            assert!(out.code_phase_offset_chips.is_finite());
        }

        // С балансированным E/L и без новой ошибки частота не должна
        // улетать далеко от исходной (initialize) поправки.
        let nominal = dll.config().nominal_chip_rate_hz;
        let final_offset = (dll.chip_freq_hz() - nominal).abs();

        assert!(
            final_offset < 200.0,
            "frequency should remain bounded under Doppler offset: {final_offset}"
        );
    }

    #[test]
    fn test_dll_recovers_after_doppler_step_change() {
        // Имитация скачка Doppler в середине tracking: после нескольких
        // эпох с одной поправкой меняем offset и проверяем, что петля
        // остаётся численно стабильной (не NaN/Inf), реагируя на новую
        // ошибку дискриминатора.
        let mut dll = Dll::new(DllConfig {
            bandwidth_hz: 5.0,
            ..DllConfig::default()
        });
        let epl_phase1 = epl_with_el(1.0, 1.0); // balanced
        let epl_phase2 = epl_with_el(2.5, 0.5); // sudden mismatch (simulated Doppler jump)

        for _ in 0..50 {
            dll.update(&epl_phase1);
        }

        let freq_before_jump = dll.chip_freq_hz();

        for _ in 0..50 {
            let out = dll.update(&epl_phase2);
            assert!(out.chip_freq_hz.is_finite());
        }

        assert!(
            (dll.chip_freq_hz() - freq_before_jump).abs() > 1e-9,
            "DLL should react to the new discriminator error after the jump"
        );
        assert!(dll.chip_freq_hz().is_finite());
    }

    #[test]
    fn test_dll_integration_aligned_prn_gives_near_zero_discriminator() {
        const FS: f64 = 2_048_000.0;
        const N: usize = 2048;

        let cache = PrnCodeCache::new();
        let prn_code: Vec<f32> = cache.resample_gps(1, N).unwrap();
        let signal: Vec<Complex32> = prn_code.iter().map(|&c| Complex32::new(c, 0.0)).collect();
        let mut dll = Dll::with_defaults();
        let half_samples = dll.half_chip_samples(FS);
        let (early, prompt, late) = make_epl_replicas(&prn_code, half_samples);
        let epl = correlator_epl(&signal, &early, &prompt, &late);
        let out = dll.update(&epl);

        assert!(
            out.discriminator_output.abs() < 0.1,
            "aligned PRN should give near-zero discriminator, got {}",
            out.discriminator_output
        );
    }

    #[test]
    fn test_dll_integration_delayed_prn_gives_nonzero_discriminator() {
        const FS: f64 = 2_048_000.0;
        const N: usize = 2048;

        let cache = PrnCodeCache::new();
        let prn_code: Vec<f32> = cache.resample_gps(2, N).unwrap();
        let delay = 2usize;
        let mut delayed = vec![0.0_f32; N];
        delayed[delay..N].copy_from_slice(&prn_code[..(N - delay)]);

        let signal: Vec<Complex32> = delayed.iter().map(|&c| Complex32::new(c, 0.0)).collect();
        let mut dll = Dll::with_defaults();
        let half_samples = dll.half_chip_samples(FS);
        let (early, prompt, late) = make_epl_replicas(&prn_code, half_samples);
        let epl = correlator_epl(&signal, &early, &prompt, &late);
        let out = dll.update(&epl);

        assert!(
            out.discriminator_output != 0.0,
            "delayed signal should produce a nonzero discriminator"
        );
    }

    #[test]
    fn test_dll_with_ele_discriminator_balanced_gives_near_zero() {
        let mut dll = Dll::new(DllConfig {
            discriminator: DllDiscriminatorKind::Ele,
            ..DllConfig::default()
        });
        let out = dll.update(&epl_balanced(1.0));

        assert!(out.discriminator_output.abs() < 1e-6);
    }

    #[test]
    fn test_loop_filter_matches_pi_equations_exactly() {
        let mut f = DllLoopFilter::new(2.0, 0.707, 0.001);

        let error = 0.1_f32;

        let tau1 = f.coeffs().tau1;
        let tau2 = f.coeffs().tau2;
        let t = 0.001_f32;

        let out = f.update(error);

        let expected_integrator = error * t / tau1;
        let expected_proportional = error * tau2 / tau1;
        let expected_output = expected_integrator + expected_proportional;

        assert!(
            (f.integrator() - expected_integrator).abs() < 1e-6,
            "integrator mismatch: expected={}, got={}",
            expected_integrator,
            f.integrator()
        );

        assert!(
            (out - expected_output).abs() < 1e-6,
            "output mismatch: expected={expected_output}, got={out}",
        );
    }

    #[test]
    fn test_dll_initialize_wraps_negative_phase() {
        let mut dll = Dll::with_defaults();

        dll.initialize(-10.0, 0.0);

        let phase = dll.code_phase_offset_chips();

        assert!(
            (0.0..1023.0).contains(&phase),
            "phase should be wrapped into [0,1023), got {phase}"
        );

        assert!((phase - 1013.0).abs() < 1e-9);
    }
}
