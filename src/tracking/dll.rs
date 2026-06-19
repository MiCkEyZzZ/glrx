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
    pub code_freq_hz: f64,

    /// Выход дискриминатора в чипах (диагностика)
    pub descriminator_output: f32,

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
    /// Постоянная времени интегратора τ₁ (с)
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
    /// Шумовая полоса петли (Гц). Типично 1-5 Гц: уже - меньше шума и медленнее реакция на динамику, шире - наоборот
    pub bandwidth_hz: f32,

    /// Коэффициент демпфирования. `0.707` - критическое демпфирование
    pub damping: f32,

    /// Половина межкорреляторного расстояния (chip spacing), в чипах
    pub half_chip_spacing: f32,

    /// Тип дискриминатора
    pub discriminator: DllDiscriminatorKind,

    /// Период когерентной интеграции (с), обычно 0.001 (1мс)
    pub integration_time_s: f32,

    /// Номинальная частота кода (chips/s). Для GPS L1 C/A - 023 000
    pub nominal_chip_rate_hz: f64,

    /// Ограничение выхода фильтра (chips/s) - защита от насыщения при аномальных входах (например, во время потери lock)
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
    #[must_use]
    pub fn chip_spacing_in_range(&self) -> bool {
        let full_spacing = 2.0 * self.half_chip_spacing;

        (0.1..=1.0).contains(&full_spacing)
    }
}

impl DllFilterCoeffs {
    /// Вычисляет коэффициенты из полосы петли и демпфирования.
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

        self.integrator += error_chips * t * tau1;

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
            code_freq_hz: self.chip_freq_hz,
            descriminator_output: raw_error,
            filter_output: filter_out,
            state: self.state,
        }
    }

    /// Текущее состояние петли.
    #[must_use]
    pub const fn state(&self) -> DllState {
        self.state
    }
}

/// Вычисляет ошибку дискриминатора в **чипах**.
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

#[cfg(test)]
mod tests {
    use num_complex::Complex32;

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
}
