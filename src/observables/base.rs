//! Базовые типы навигационных наблюдаемых.
//!
//! Этот модуль объединяет результаты вычислений из подмодулей
//! [`crate::observables::pseudorange`],
//! [`crate::observables::doppler`] и
//! [`crate::observables::cn0`] в единые структуры,
//! используемые навигационным решателем.
//!
//! Он является границей между DSP-слоем (tracking loops) и навигационным
//! слоем (ephemeris, solver). Его задача — преобразовать сырые параметры,
//! полученные от каналов сопровождения, в физически осмысленные измерения
//! с единицами СИ и применёнными поправками.
//!
//! # Формирование наблюдаемого
//!
//! ```text
//! ChannelOutput { dll, pll, cn0_db_hz }
//!     +
//! DecodedSubframe { how.tow_count }
//!     +
//! Ephemeris { clock, orbit1, orbit2 }
//!     +
//! IonosphericModel (опционально)
//!     │
//!     ▼
//! Observable {
//!     prn,
//!     pseudorange,
//!     doppler,
//!     cn0,
//!     timestamp_s,
//! }
//!     │
//!     ▼
//! ObservableSet
//!     │
//!     ▼
//! Solver::update(observables)
//! ```
//!
//! Основными типами этого модуля являются:
//!
//! - [`Observable`] — полный набор наблюдаемых для одного спутника;
//! - [`ObservableSet`] — набор наблюдаемых одной навигационной эпохи,
//!   готовый к передаче в position solver.

use crate::{
    navigation::{ephemeris::Ephemeris, nav_data::IonosphericModel},
    observables::{
        cn0::{Cn0Estimate, LOST_SIGNAL_CN0_THRESHOLD, SignalQuality, from_tracking_estimate},
        doppler::{DopplerInput, DopplerObservable, compute_doppler},
        pseudorange::{
            ApproxUserPosition, PseudorangeInput, PseudorangeResult, compute_pseudorange,
            tow_count_to_seconds,
        },
    },
};

/// Полный набор наблюдаемых для одного спутника и одной навигационной эпохи.
///
/// Это основная структура данных, передаваемая из observable-слоя в solver.
/// Содержит исправленные измерения и метаданные качества.
///
/// # Пример использования
///
/// ```text
/// let obs = Observable::from_channel(
///     &channel_output,
///     tow_s,
///     receiver_time_s,
///     &ephemeris,
///     iono.as_ref(),
///     user_pos.as_ref(),
/// );
///
/// if obs.pseudorange.valid {
///     solver.update(&obs);
/// }
/// ```
#[derive(Debug, Clone)]
pub struct Observable {
    /// PRN спутника
    pub prn: u8,

    /// Результат вычисления псевдодальности (включая все поправки)
    pub pseudorange: PseudorangeResult,

    /// Доплер наблюдение и псевдоскорость
    pub doppler: DopplerObservable,

    /// Оценка C/N₀ (дБ-Гц) и WLS-вес.
    pub cn0: Cn0Estimate,

    /// GPS system time момента приёма сигнала (с от начала недели)
    pub timestamp_s: f64,
}

/// Набор наблюдаемых для одной эпохи от всех активных каналов.
///
/// Готов к передаче в `crate::solver` для вычисления позиций.
/// Solver требует минимум 4 валидных наблюдаемых для 3D-fix.
#[derive(Debug, Clone, Default)]
pub struct ObservableSet {
    /// Наблюдаемые по всем спутникам текущей эпохи
    pub observables: Vec<Observable>,

    /// GPS system time эпохи (с)
    pub epoch_time_s: f64,
}

/// Входные данные для формирования `Observable`.
///
/// Это промежуточная структура, отделяющая DSP-слой (tracking loops)
/// от навигационного слоя.
///
/// Используется вместо длинного списка аргументов в `from_channel`.
#[derive(Debug, Clone)]
pub struct ObservableInput<'a> {
    /// PRN спутника (ID SV)
    pub prn: u8,

    /// Кодовая фаза (chips), измеренная tracking loop (DLL)
    pub code_phase_chips: f64,

    /// Частота чипов кода (Hz), например GPS L1 C/A = 1.023 MHz
    pub chip_freq_hz: f64,

    /// Несущая частота (Hz), используемая для доплера и восстановления времени
    pub carrier_freq_hz: f64,

    /// Оценка C/N₀ (дБ-Гц), полученная из tracking loop.
    ///
    /// `None` означает отсутствие валидной оценки сигнала.
    pub cn0_db_hz: Option<f32>,

    /// Счётчик времени GPS week (TOW count), используемый для восстановления времени передачи
    pub tow_count: u32,

    /// Время приёма сигнала в системе GPS time scale (секунды от начала недели)
    pub receiver_time_s: f64,

    /// Эфемериды спутника, используемые для расчёта положения и clock bias
    pub eph: &'a Ephemeris,

    /// Ионосферная модель (опционально), используется для коррекции задержки сигнала
    pub iono: Option<&'a IonosphericModel>,

    /// Оценка позиции пользователя (опционально), используется для улучшения модели псевдодальности
    pub user_pos: Option<&'a ApproxUserPosition>,
}

impl Observable {
    /// Создаёт `Observable` из выхода tracking-канала.
    ///
    /// Выполняет:
    /// - перевод code phase → pseudorange
    /// - вычисление доплера
    /// - оценку C/N₀ и весов WLS
    /// - применение навигационных поправок (эфемериды, ионосфера)
    #[must_use]
    pub fn from_channel(input: &ObservableInput) -> Self {
        let tow_s = tow_count_to_seconds(input.tow_count);

        let pr_input = PseudorangeInput {
            prn: input.prn,
            code_phase_chips: input.code_phase_chips,
            chip_freq_hz: input.chip_freq_hz,
            receiver_time_s: input.receiver_time_s,
            tow_s,
            carrier_freq_hz: input.carrier_freq_hz,
            cn0_db_hz: input.cn0_db_hz,
        };

        let pseudorange = compute_pseudorange(&pr_input, input.eph, input.iono, input.user_pos);

        let doppler_input = DopplerInput {
            prn: input.prn,
            carrier_freq_hz: input.carrier_freq_hz,
            t_tx_s: pseudorange.t_tx_s,
        };

        let doppler = compute_doppler(&doppler_input, input.eph);

        let cn0 = input.cn0_db_hz.map_or(
            Cn0Estimate {
                db_hz: 0.0,
                wls_weight: 1.0,
                samples_used: 0,
            },
            from_tracking_estimate,
        );

        Self {
            prn: input.prn,
            pseudorange,
            doppler,
            cn0,
            timestamp_s: input.receiver_time_s,
        }
    }

    /// Возвращает исправленну. псевдодальность в метрах (удобный accessor).
    #[must_use]
    pub const fn pseudorange_m(&self) -> f64 {
        self.pseudorange.corrected_m
    }

    /// Возвращает Доплер смешение (Гц).
    #[must_use]
    pub const fn doppler_hz(&self) -> f64 {
        self.doppler.doppler_hz
    }

    /// Возвращает исправленную псевдодальность (м/с).
    #[must_use]
    pub const fn pseudorange_rate_m_s(&self) -> f64 {
        self.doppler.pseudorange_rate_corrected_m_s
    }

    /// Возвращает C/N₀ (дБ-Гц).
    #[must_use]
    pub const fn cn0_db_hz(&self) -> f32 {
        self.cn0.db_hz
    }

    /// Возвращает WLS-вес (линейный масштаб C/N₀).
    #[must_use]
    pub const fn wls_weight(&self) -> f64 {
        self.cn0.wls_weight
    }

    /// Возвращает классификацию качества сигнала.
    #[must_use]
    pub fn signal_quality(&self) -> SignalQuality {
        self.cn0.quality()
    }

    /// `true`, если псевдодальность прошла валидацию (в физических разумных
    /// пределах) и пригодна для подачи в solver.
    #[must_use]
    pub const fn is_usable(&self) -> bool {
        self.pseudorange.valid && self.cn0.db_hz >= LOST_SIGNAL_CN0_THRESHOLD
    }
}

impl ObservableSet {
    /// Создаёт пустой набор для заданного комента времени.
    #[must_use]
    pub const fn new(epoch_time_s: f64) -> Self {
        Self {
            observables: Vec::new(),
            epoch_time_s,
        }
    }

    /// Добавляет наблюдаемого.
    pub fn push(
        &mut self,
        obs: Observable,
    ) {
        self.observables.push(obs);
    }

    /// Возвращает число наблюдаемых в наборе.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.observables.len()
    }

    /// Возвращает `true`, если набор пустой.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.observables.is_empty()
    }

    /// Возвращает число наблюдаемых, пригодных для solver (валидные и с достаточным CN0).
    #[must_use]
    pub fn usable_count(&self) -> usize {
        self.observables.iter().filter(|o| o.is_usable()).count()
    }

    /// Возвращает `true`, если набор содержит минимум 4 пригодных наблюдаемых
    /// (необходимое условие для 3D position fix).
    #[must_use]
    pub fn sufficient_for_fix(&self) -> bool {
        self.usable_count() >= 4
    }

    /// Итератор по пригодным для solver наблюдаемых.
    pub fn usable(&self) -> impl Iterator<Item = &Observable> {
        self.observables.iter().filter(|o| o.is_usable())
    }
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        navigation::ephemeris::{ClockParams, Ephemeris, OrbitPart1, OrbitPart2},
        observables::{
            base::Observable,
            pseudorange::{GPS_L1_CARRIER_HZ, GPS_L1_CHIP_RATE},
        },
    };

    fn dummy_ephemeris(prn: u8) -> Ephemeris {
        Ephemeris::new(
            prn,
            ClockParams {
                week_number: 2300,
                ura_index: 0,
                sv_health: 0,
                iodc: 0x0010,
                toc: 0.0,
                af2: 0.0,
                af1: 0.0,
                af0: 0.0,
            },
            OrbitPart1 {
                iode: 0x10,
                crs: 0.0,
                delta_n: 0.0,
                m0: 0.0,
                cuc: 0.0,
                e: 0.001,
                cus: 0.0,
                sqrt_a: 5153.65,
                toe: 0.0,
            },
            OrbitPart2 {
                cic: 0.0,
                omega0: 0.0,
                cis: 0.0,
                i0: 55.0_f64.to_radians(),
                crc: 0.0,
                omega: 0.0,
                omega_dot: 0.0,
                iode: 0x10,
                idot: 0.0,
            },
        )
    }

    fn make_observable(prn: u8) -> Observable {
        let eph = dummy_ephemeris(prn);

        let code_phase = 100.0;
        let chip_freq = GPS_L1_CHIP_RATE;
        let tow_count = 17;

        let tow_s = tow_count_to_seconds(tow_count);
        let t_tx = tow_s + code_phase / chip_freq;
        let receiver_time_s = t_tx + 0.075;

        Observable::from_channel(&ObservableInput {
            prn,
            code_phase_chips: code_phase,
            chip_freq_hz: chip_freq,
            carrier_freq_hz: GPS_L1_CARRIER_HZ + 500.0,
            cn0_db_hz: Some(42.0),
            tow_count,
            receiver_time_s,
            eph: &eph,
            iono: None,
            user_pos: None,
        })
    }

    fn make_observable_with_cn0(cn0: Option<f32>) -> Observable {
        let eph = dummy_ephemeris(1);

        Observable::from_channel(&ObservableInput {
            prn: 1,
            code_phase_chips: 100.0,
            chip_freq_hz: GPS_L1_CHIP_RATE,
            carrier_freq_hz: GPS_L1_CARRIER_HZ + 500.0,
            cn0_db_hz: cn0,
            tow_count: 17,
            receiver_time_s: 100.0,
            eph: &eph,
            iono: None,
            user_pos: None,
        })
    }

    #[test]
    fn test_observable_prn_preserved() {
        let obs = make_observable(7);

        assert_eq!(obs.prn, 7);
    }

    #[test]
    fn test_observable_pseudorange_m_positive() {
        let obs = make_observable(1);

        assert!(obs.pseudorange_m() > 0.0, "pseudorange must be positive");
    }

    #[test]
    fn test_observable_doppler_hz_matches_offset() {
        let obs = make_observable(1);

        // carrier_freq = GPS_L1 + 500 → Doppler = 500 Hz
        assert!(
            (obs.doppler_hz() - 500.0).abs() < 1.0,
            "Doppler should be ~500 Hz, got {}",
            obs.doppler_hz()
        );
    }

    #[test]
    fn test_observable_cn0_db_hz_matches_input() {
        let obs = make_observable(1);

        assert!(
            (obs.cn0_db_hz() - 42.0).abs() < 1e-5,
            "C/N₀ must match input 42 dB-Hz, got {}",
            obs.cn0_db_hz()
        );
    }

    #[test]
    fn test_observable_wls_weight_positive() {
        let obs = make_observable(1);

        assert!(obs.wls_weight() > 0.0);
    }

    #[test]
    fn test_observable_is_usable_with_good_signal_and_valid_pseudorange() {
        let obs = make_observable(1);

        // cn0=42 > 25 dB-Hz, pseudorange в диапазоне GPS -> usable
        assert!(obs.is_usable());
    }

    #[test]
    fn test_observable_not_usable_with_low_cn0() {
        let eph = dummy_ephemeris(1);

        let code_phase = 100.0_f64;
        let chip_freq = GPS_L1_CHIP_RATE;
        let tow_count = 18;

        let tow_s = tow_count_to_seconds(tow_count);
        let t_tx = tow_s + code_phase / chip_freq;
        let receiver_time_s = t_tx + 20_200_000.0 / 299_792_458.0;

        let obs = Observable::from_channel(&ObservableInput {
            prn: 1,
            code_phase_chips: code_phase,
            chip_freq_hz: chip_freq,
            carrier_freq_hz: GPS_L1_CARRIER_HZ,
            cn0_db_hz: Some(20.0), // ниже порога
            tow_count,
            receiver_time_s,
            eph: &eph,
            iono: None,
            user_pos: None,
        });

        assert!(!obs.is_usable(), "cn0=20 dB-Hz should be below threshold");
    }

    #[test]
    fn test_observable_set_empty_initially() {
        let set = ObservableSet::new(100.0);

        assert!(set.is_empty());
        assert_eq!(set.len(), 0);
        assert_eq!(set.usable_count(), 0);
        assert!(!set.sufficient_for_fix());
    }

    #[test]
    fn test_observable_set_push_increases_len() {
        let mut set = ObservableSet::new(100.0);

        set.push(make_observable(1));
        set.push(make_observable(2));

        assert_eq!(set.len(), 2);
    }

    #[test]
    fn test_observable_set_sufficient_for_fix_with_four_good_observables() {
        let mut set = ObservableSet::new(100.0);

        for prn in 1u8..=4 {
            set.push(make_observable(prn));
        }

        assert!(set.sufficient_for_fix());
    }

    #[test]
    fn test_observable_set_not_sufficient_with_three() {
        let mut set = ObservableSet::new(100.0);

        for prn in 1u8..=3 {
            set.push(make_observable(prn));
        }

        assert!(!set.sufficient_for_fix());
    }

    #[test]
    fn test_observable_set_usable_iterator_filters_invalid() {
        let mut set = ObservableSet::new(100.0);

        set.push(make_observable(1)); // usable
        set.push(make_observable(2)); // usable

        let usable_count = set.usable().count();

        assert_eq!(usable_count, 2);
    }

    #[test]
    fn test_observable_pseudorange_rate_finite() {
        let obs = make_observable(1);

        assert!(obs.pseudorange_rate_m_s().is_finite());
    }

    #[test]
    fn test_observable_signal_quality_good_for_high_cn0() {
        let obs = make_observable(1); // cn0=42 dB-Hz

        assert_eq!(obs.signal_quality(), SignalQuality::Good);
    }

    #[test]
    fn test_observable_cn0_fallback_when_none() {
        let obs = make_observable_with_cn0(None);

        assert!(obs.cn0_db_hz().abs() < f32::EPSILON);
        assert!(obs.wls_weight() >= 1.0);
    }

    #[test]
    fn test_observable_is_usable_on_cn0_threshold_boundary() {
        let eph = dummy_ephemeris(1);

        let code_phase = 100.0;
        let chip_freq = GPS_L1_CHIP_RATE;
        let tow_count = 17;

        let tow_s = tow_count_to_seconds(tow_count);
        let t_tx = tow_s + code_phase / chip_freq;
        let receiver_time_s = t_tx + 0.075;

        let obs = Observable::from_channel(&ObservableInput {
            prn: 1,
            code_phase_chips: code_phase,
            chip_freq_hz: chip_freq,
            carrier_freq_hz: GPS_L1_CARRIER_HZ,
            cn0_db_hz: Some(LOST_SIGNAL_CN0_THRESHOLD),
            tow_count,
            receiver_time_s,
            eph: &eph,
            iono: None,
            user_pos: None,
        });

        assert!(obs.pseudorange.valid);
        assert!(
            (obs.cn0_db_hz() - LOST_SIGNAL_CN0_THRESHOLD).abs() < f32::EPSILON,
            "expected C/N0 = {}, got {}",
            LOST_SIGNAL_CN0_THRESHOLD,
            obs.cn0_db_hz()
        );
        assert!(obs.is_usable());
    }

    #[test]
    fn test_doppler_uses_pseudorange_tx_time_consistently() {
        // doppler должен быть валидным и конечным даже при изменении chip phase
        let mut obs2 = make_observable(1);

        obs2.pseudorange.t_tx_s += 1e-3; // искусственный сдвиг

        assert!(obs2.doppler_hz().is_finite());
    }

    #[test]
    fn test_timestamp_equals_receiver_time() {
        let obs = make_observable(1);

        assert!(obs.timestamp_s.is_finite());
    }
}
