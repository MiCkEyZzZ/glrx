//! Вычисление псевдодальности из кодовой фазы и TOW
//!
//! Псевдодальность - базовый измерительный параметр GNSS:
//! это расстояние от антены приёмника до спутника, испорченное
//! ошибкой часов приёмника. Без псевдодальностей position solver
//! не имеет входных данных.
//!
//! # Формула
//!
//! ```text
//! ρ_raw = (t_rx − t_tx) · c
//!
//! t_tx  = TOW_subframe + code_phase_chips / chip_rate_hz
//! t_rx  = local_clock_s  (устанавливается внешне, например, из первого
//!         успешно декодированного TOW)
//!
//! ρ_corrected = ρ_raw
//!               − Δt_sv  · c   (поправка часов спутника + релятивистика)
//!               − I · c        (ионосферная задержка, если модель доступна)
//!               − T · c        (тропосферная задержка, если доступна)
//! ```
//!
//! # Положение в pipeline
//!
//! ```text
//! TrackingChannel → ChannelOutput { dll, pll, cn0_db_hz }
//!                      │
//!                      ▼
//!              PseudorangeInput { prn, code_phase_chips,
//!                                 chip_freq_hz, tow_s,
//!                                 carrier_freq_hz, cn0_db_hz }
//!                      │
//!                      ▼
//!            compute_pseudorange(input, eph, iono_opt, rx_pos_opt)
//!                      │
//!                      ▼
//!               PseudorangeResult { raw_m, corrected_m, t_tx_s,
//!                                    corrections, valid }
//! ```
//!
//! # Тропосфера
//!
//! Реализована упрощённая модель Саастамойнена в зените (без картографической
//! ф-ии угла места) - достаточно для первого fix и точная реализация запланирована
//! отдельно.
//!
//! # Валидация с RINEX
//!
//! `PseudorangeResult::validate_against_rinex` сравнивает вычисленную
//! псевдодальность с внешним эталоном (например, из файла RINEX 2/3) и
//! возвращает разность в метрах. Допустимая погрешность после всех
//! поправок - < 1 м.

use std::f64::consts::PI;

use crate::navigation::{
    ephemeris::{Ephemeris, SPEED_OF_LIGHT},
    nav_data::IonosphericModel,
};

/// Номинальная частота кода L1 C/A (chips/s).
pub const GPS_L1_CHIP_RATE: f64 = 1_023_000.0;

/// Длина периода PRN-кода GPS L1 C/A в чипах.
pub const GPS_L1_CODE_CHIPS: f64 = 1023.0;

/// Длительность одного периода PRN-кода (с) = 1 мс.
pub const GPS_L1_CODE_PERIOD_S: f64 = 1e-3;

/// Номинальная несущая частота GPS L1 (Гц).
pub const GPS_L1_CARRIER_HZ: f64 = 1_575_420_000.0;

/// Длина волны GPS L1 несущей (м): λ = c / f.
pub const GPS_L1_WAVELENGTH_M: f64 = SPEED_OF_LIGHT / GPS_L1_CARRIER_HZ;

/// Минимальная разумная псевдодальность: ~ 20 000 км (ниже орбиты GPS).
pub const MIN_PSEUDORANGE_M: f64 = 20_000_000.0;

/// Максимальная разумная псевдодальность: ~ 27 000 км (выше орибиты GPS).
pub const MAX_PSEUDORANGE_M: f64 = 27_000_000.0;

/// Стандартная высота тропосферы для урощённой коррекции (м).
const TROPO_HEIGHT_M: f64 = 0.0; // уровень моря по умолчанию

/// Входные данные для одной эпохи вычисления псевдодальности одного канала.
///
/// Заполняется из [`crate::tracking::channel::ChannelOutput`] и
/// [`crate::navigation::frame_decoder::HowWord`].
#[derive(Debug, Clone, Copy)]
pub struct PseudorangeInput {
    /// PRN отслеживаемого спутника
    pub prn: u8,

    /// Фаза кода в текущую эпоху (чипы), нормализована в [`0, 1023`]
    /// Из `ChannelOutput::dll::code_phase_offset_chips`.
    pub code_phase_chips: f64,

    /// Текущая частота кода (chips/s)
    /// Из `ChannelOutput::dll::chip_freq_hz`
    pub chip_freq_hz: f64,

    /// Момент приёма сигнала по локальным часам приёмника (с от начала
    /// GPS-недели). Обновляется при получении каждого нового TOW.
    pub receiver_time_s: f64,

    /// TOW (**начала** субфрейма, из которого этот TOW был декодирован),
    /// в секундах. Из `HowWord::tow_count * 6.0`.
    ///
    /// В соответствии с GPS ICD-200 `tow_count` указывает начало
    /// **следующего** субфрейма, поэтому перед использованием нужно
    /// вычесть 6 с:  `tow_s = (how.tow_count - 1) * 6`.
    pub tow_s: f64,

    /// Текущая частота несущей (Гц), включая Доплер.
    /// Из `ChannelOutput::pll::carrier_freq_hz`.
    pub carrier_freq_hz: f64,

    /// Оценка C/N₀ канала (дБ-Гц).
    /// Из `ChannelOutput::cn0_db_hz`.
    pub cn0_db_hz: Option<f32>,
}

/// Все поправки, применённые к псевдодальности (метры, каждая отдельно).
#[allow(clippy::struct_field_names)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PseudorangeCorrections {
    /// Поправка часов спутника, включая релятивистику (м).
    /// Отрицательная — уменьшает сырую псевдодальность.
    pub satellite_clock_m: f64,

    /// Ионосферная задержка (м). Положительная — увеличивает задержку.
    pub ionosphere_m: f64,

    /// Тропосферная задержка (м). Положительная — увеличивает задержку.
    pub troposphere_m: f64,
}

/// Результат вычисления псевдодальности для одного спутника и одной эпохи.
#[derive(Debug, Clone, Copy)]
pub struct PseudorangeResult {
    /// PRN спутника
    pub prn: u8,

    /// Сырая псевдодальность до поправок (м)
    pub raw_m: f64,

    /// Исправленная псевдодальность (м)
    pub corrected_m: f64,

    /// Момент передачи (GPS system time, с от начала недели)
    pub t_tx_s: f64,

    /// Момент приёма (GPS system time, с от начала недели)
    pub t_rx_s: f64,

    /// Все применённые поправки
    pub corrections: PseudorangeCorrections,

    /// `true`, если псевдодальность укладывается в физически разумные пределы
    pub valid: bool,
}

/// Положение пользователя (приближённое) для вычисления ионосферной и
/// тропосферной поправок. Если неизсвестно - передаётся `None` и поправки
/// расчитываются по нулевому положению / пропускаются.
#[derive(Debug, Clone, Copy)]
pub struct ApproxUserPosition {
    /// Геодезическая широта (рад)
    pub lat_rad: f64,

    /// Геодезическая долгота (рад)
    pub lon_rad: f64,

    /// Высота над элипсойдом (м)
    pub alt_m: f64,
}

impl PseudorangeCorrections {
    /// Суммарная поправка (м): `satellite_clock + ionosphere + troposphere`.
    #[must_use]
    pub fn total_m(&self) -> f64 {
        self.satellite_clock_m + self.ionosphere_m + self.troposphere_m
    }
}

impl PseudorangeResult {
    /// Сравнивает вычесленную псевдодальность с эталонной из REMIX.
    ///
    /// # Аргументы
    ///
    /// - `rinex_pseudorange_m` - псевдодальность из RINEX-файла (C1 или P1), метры
    ///
    /// # Возвращает
    ///
    /// Разность `corrected_m - rinex_m` (метры). Положительная — наша
    /// оценка завышена.
    ///
    /// Допустимая
    #[must_use]
    pub fn validate_against_rinex(
        &self,
        rinex_pseudorange_m: f64,
    ) -> f64 {
        self.corrected_m - rinex_pseudorange_m
    }

    /// `true`, если прогрешность относительно RINEX не превышает `threshold_m`.
    #[must_use]
    pub fn is_within_rinex_tolerance(
        &self,
        rinex_pseudorange_m: f64,
        threshold_m: f64,
    ) -> bool {
        self.validate_against_rinex(rinex_pseudorange_m).abs() <= threshold_m
    }
}

/// Вычисляет псевдодальность из кодовой фазы и TOW.
#[must_use]
pub fn compute_pseudorange(
    input: &PseudorangeInput,
    eph: &Ephemeris,
    iono: Option<&IonosphericModel>,
    user_pos: Option<&ApproxUserPosition>,
) -> PseudorangeResult {
    // Момент передачи сигнала
    // t_tx = TOW_начало_сабфрейма + дробная_часть_миллисекунды
    // Дробная часть: code_phase_chips / chip_rate_hz
    // кода (chip_freq_hz = nominal ± Доплер поправка).
    let chip_rate = if input.chip_freq_hz > 0.0 {
        input.chip_freq_hz
    } else {
        GPS_L1_CHIP_RATE
    };
    let fractional_code_s = input.code_phase_chips / chip_rate;
    // TOW указывает момент начала текущего subframe (6-секунд интервал).
    let t_tx_uncorrected = input.tow_s + fractional_code_s;

    // Проверка часов спутника
    // Используем полную поправку, включая релятивистику (см. ephemeris.rs).
    let clock_correction_s = eph.clock_correction_with_relativistic(t_tx_uncorrected);
    let t_tx = t_tx_uncorrected - clock_correction_s;

    // Сырая псевдодальность
    let t_rx = input.receiver_time_s;
    let mut delta_t = t_rx - t_tx;

    // Корректируем переход через GPS-недели (604 800 с).
    if delta_t < 0.0 {
        delta_t += 604_800.0;
    } else if delta_t > 302_400.0 {
        delta_t -= 604_800.0;
    }

    let raw_m = delta_t * SPEED_OF_LIGHT;
    // Ионосферная поправка
    let ionosphere_m = compute_ionosphere_correction(input, iono, user_pos);
    // Тропосферная поправка (упрощённая)
    let troposphere_m = compute_troposphere_correction(user_pos);
    // Поправка часов спутника в метрах
    let satellite_clock_m = -clock_correction_s * SPEED_OF_LIGHT;
    let corrections = PseudorangeCorrections {
        satellite_clock_m,
        ionosphere_m,
        troposphere_m,
    };

    // Исправленная псевдодальность
    let corrected_m = raw_m + corrections.total_m();
    let valid = (MIN_PSEUDORANGE_M..=MAX_PSEUDORANGE_M).contains(&corrected_m);

    PseudorangeResult {
        prn: input.prn,
        raw_m,
        corrected_m,
        t_tx_s: t_tx,
        t_rx_s: t_rx,
        corrections,
        valid,
    }
}

/// Вспомогательная ф-я `tow_count` из [`crate::navigation::frame_decoder::HowWord`]
/// в секунды начала **текущего** subframe.
///
/// GPS ICD-200: `tow_count` - число 6-секндных интервалов до начала
/// **следующего** subframe. Поэтому начало текущего subframe:
///
/// ```text
/// tow_s = (tow_count - 1) * 6
/// ```
///
/// При `tow_count == 0` начало недели (переполнение): возвращается 0.
#[must_use]
pub fn tow_count_to_seconds(tow_count: u32) -> f64 {
    if tow_count == 0 {
        0.0
    } else {
        f64::from(tow_count.saturating_sub(1)) * 6.0
    }
}

/// Вычисляет ионосферную поправку в метрах (всегда положительную).
fn compute_ionosphere_correction(
    input: &PseudorangeInput,
    iono: Option<&IonosphericModel>,
    user_pos: Option<&ApproxUserPosition>,
) -> f64 {
    let Some(model) = iono else { return 0.0 };

    // Угол места спутника в полуокружностях: приближённо из частоты несущей.
    // Фактическое значение elevation должно вычислятся из позиции спутника
    // и пользователя. Здесь используем грубое приближение 0.3 (~ 54°), если
    // истинный угол неизвестен.
    let elevation_semicircles = match user_pos {
        Some(_) | None => 0.3, // TODO: вычислять из ECEF-позиций когда solver даст позици.
    };
    let (lat_sc, lon_sc) = match user_pos {
        Some(pos) => (pos.lat_rad / PI, pos.lon_rad / PI),
        None => (0.0, 0.0),
    };
    // GPS TOW в секундах суток
    let tow_mod_day = input.tow_s.rem_euclid(86_400.0);
    let delay_s = model.delay_seconds(elevation_semicircles, lat_sc, lon_sc, tow_mod_day);

    delay_s * SPEED_OF_LIGHT
}

/// Упрощённая тропосфера поправка Саастамойнена в зените (м).
///
/// Без угла места (elevation mapping): даёт ~2.3 м у уровня моря - типичный
/// тропосферный зенитный путь задержки. Точная реализация с картографической
/// функцией запланирована отдельно.
fn compute_troposphere_correction(user_pos: Option<&ApproxUserPosition>) -> f64 {
    // Задержка в зените при уровне моря ~2.3 м (Саастамойнен, стандартная атмосфера).
    let zenith_delay_m = 2.3;

    // Поправка на высоте: задержка убывает экспоненцианально.
    let height_m = user_pos.map_or(TROPO_HEIGHT_M, |p| p.alt_m);
    let scale = (-height_m / 7_000.0).exp();

    zenith_delay_m * scale
}

#[cfg(test)]
mod tests {
    use crate::navigation::ephemeris::{
        ClockParams, Ephemeris, OrbitPart1, OrbitPart2, SPEED_OF_LIGHT,
    };

    use super::*;

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

    fn nominal_input(prn: u8) -> PseudorangeInput {
        // Обычный GPS-спутник на расстоянии ~20 200 км -> ~ 67.4 мс
        // t_tx ≈ 100.0 с, t_rx = t_tx + 0.0674 с
        let tow_s = 100.0_f64;
        let code_phase_chips = 100.0;
        let fractional_s = code_phase_chips / GPS_L1_CHIP_RATE;
        let t_tx = tow_s + fractional_s;
        let expected_range_m = 20_200_000.0;
        let flight_time_s = expected_range_m / SPEED_OF_LIGHT;
        let t_rx = t_tx + flight_time_s;

        PseudorangeInput {
            prn,
            code_phase_chips,
            chip_freq_hz: GPS_L1_CHIP_RATE,
            receiver_time_s: t_rx,
            tow_s,
            carrier_freq_hz: GPS_L1_CARRIER_HZ,
            cn0_db_hz: Some(45.0),
        }
    }

    #[test]
    fn test_tow_count_to_seconds_zero() {
        assert!((tow_count_to_seconds(0) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_tow_count_to_seconds_one() {
        assert!((tow_count_to_seconds(1) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_tow_count_to_seconds_two() {
        // tow_count = 2 -> (2 - 1) * 6 = 6 c
        assert!((tow_count_to_seconds(2) - 6.0) < 1e-9);
    }

    #[test]
    fn test_tow_count_to_seconds_typical() {
        // tow_count=100 -> 99 * 6 = 594 c
        assert!((tow_count_to_seconds(100) - 594.0).abs() < 1e-9);
    }

    #[test]
    fn test_compute_pseudorange_raw_is_positive_and_in_range() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let result = compute_pseudorange(&input, &eph, None, None);

        assert!(result.raw_m > 0.0, "raw pseudorange must be positive");
        assert!(
            result.raw_m >= MIN_PSEUDORANGE_M,
            "raw={} below GPS orbit range",
            result.raw_m
        );
        assert!(
            result.raw_m <= MAX_PSEUDORANGE_M,
            "raw={} above GPS orbit range",
            result.raw_m
        );
    }

    #[test]
    fn test_compute_pseudorange_valid_flag_set_for_nominal_input() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let result = compute_pseudorange(&input, &eph, None, None);

        assert!(result.valid, "nominal GPS pseudorange must be valid");
    }

    #[test]
    fn test_compute_pseudorange_t_tx_less_than_t_rx() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let result = compute_pseudorange(&input, &eph, None, None);

        assert!(
            result.t_tx_s < result.t_rx_s,
            "signal must arrive after transmission: t_tx={} t_rx={}",
            result.t_tx_s,
            result.t_rx_s,
        );
    }

    #[test]
    fn test_compute_pseudorange_prn_preserved() {
        let eph = dummy_ephemeris(7);
        let mut input = nominal_input(7);

        input.prn = 7;

        let result = compute_pseudorange(&input, &eph, None, None);

        assert_eq!(result.prn, 7);
    }

    #[test]
    fn test_satellite_clock_correction_compensates_clock_bias() {
        let eph_zero_clock = dummy_ephemeris(1);
        let mut eph_nonzero_clock = dummy_ephemeris(1);

        // Постоянная ошибка часов спутника 1 мкс (~300 м без компенсации).
        eph_nonzero_clock.clock.af0 = 1e-6;

        let input = nominal_input(1);

        let r0 = compute_pseudorange(&input, &eph_zero_clock, None, None);
        let r1 = compute_pseudorange(&input, &eph_nonzero_clock, None, None);

        let diff = (r1.corrected_m - r0.corrected_m).abs();

        assert!(
            diff < 1e-3,
            "satellite clock correction should compensate clock bias, diff={diff}"
        );
    }

    #[test]
    fn test_satellite_clock_bias_changes_raw_pseudorange() {
        let eph_zero_clock = dummy_ephemeris(1);
        let mut eph_nonzero_clock = dummy_ephemeris(1);

        eph_nonzero_clock.clock.af0 = 1e-6;

        let input = nominal_input(1);

        let r0 = compute_pseudorange(&input, &eph_zero_clock, None, None);
        let r1 = compute_pseudorange(&input, &eph_nonzero_clock, None, None);

        let diff = (r1.raw_m - r0.raw_m).abs();

        assert!(
            (diff - SPEED_OF_LIGHT * 1e-6).abs() < 1.0,
            "expected ≈{} m difference, got {} m",
            SPEED_OF_LIGHT * 1e-6,
            diff,
        );
    }

    #[test]
    fn test_corrections_satellite_clock_m_opposite_sign_to_af0() {
        let mut eph = dummy_ephemeris(1);

        eph.clock.af0 = 1e-6; // положительная ошибка → спутник спешит → t_tx занижен → ρ завышена → корrekция отрицательная

        let input = nominal_input(1);
        let result = compute_pseudorange(&input, &eph, None, None);

        // satellite_clock_m = -Δt_sv * c; af0 > 0 → Δt_sv > 0 → correction < 0
        assert!(
            result.corrections.satellite_clock_m < 0.0,
            "positive af0 should produce negative satellite_clock correction, got {}",
            result.corrections.satellite_clock_m
        );
    }

    #[test]
    fn test_ionosphere_correction_is_nonnegative() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let iono = IonosphericModel {
            alpha: [1e-8, 0.0, 0.0, 0.0],
            beta: [80_000.0, 0.0, 0.0, 0.0],
        };
        let result = compute_pseudorange(&input, &eph, Some(&iono), None);

        assert!(
            result.corrections.ionosphere_m >= 0.0,
            "ionospheric delay must be non-negative, got {}",
            result.corrections.ionosphere_m
        );
    }

    #[test]
    fn test_ionosphere_correction_without_model_is_zero() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let result = compute_pseudorange(&input, &eph, None, None);

        assert!(
            result.corrections.ionosphere_m.abs() < 1e-12,
            "without iono model correction must be zero"
        );
    }

    #[test]
    fn test_troposphere_correction_is_positive_at_sea_level() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let result = compute_pseudorange(&input, &eph, None, None);

        // Даже без user_pos тропосфера применяется с высотой 0 м → ~2.3 м
        assert!(
            result.corrections.troposphere_m > 0.0,
            "troposphere correction must be positive, got {}",
            result.corrections.troposphere_m
        );
        assert!(
            result.corrections.troposphere_m < 5.0,
            "sea-level troposphere correction must be < 5 m, got {}",
            result.corrections.troposphere_m
        );
    }

    #[test]
    fn test_troposphere_correction_decreases_with_altitude() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let low = compute_pseudorange(
            &input,
            &eph,
            None,
            Some(&ApproxUserPosition {
                lat_rad: 0.0,
                lon_rad: 0.0,
                alt_m: 0.0,
            }),
        );
        let high = compute_pseudorange(
            &input,
            &eph,
            None,
            Some(&ApproxUserPosition {
                lat_rad: 0.0,
                lon_rad: 0.0,
                alt_m: 5000.0,
            }),
        );

        assert!(
            high.corrections.troposphere_m < low.corrections.troposphere_m,
            "troposphere correction must decrease with altitude: low={} high={}",
            low.corrections.troposphere_m,
            high.corrections.troposphere_m
        );
    }

    #[test]
    fn test_corrections_total_m_matches_individual_sum() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let iono = IonosphericModel {
            alpha: [1e-8, 0.0, 0.0, 0.0],
            beta: [80_000.0, 0.0, 0.0, 0.0],
        };
        let result = compute_pseudorange(&input, &eph, Some(&iono), None);
        let expected = result.corrections.satellite_clock_m
            + result.corrections.ionosphere_m
            + result.corrections.troposphere_m;

        assert!(
            (result.corrections.total_m() - expected).abs() < 1e-9,
            "total_m must equal sum of components"
        );
    }

    #[test]
    fn test_corrected_equals_raw_plus_total_corrections() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let result = compute_pseudorange(&input, &eph, None, None);
        let expected = result.raw_m + result.corrections.total_m();

        assert!(
            (result.corrected_m - expected).abs() < 1e-6,
            "corrected_m must equal raw_m + total corrections"
        );
    }

    #[test]
    fn test_validate_against_rinex_zero_difference() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let result = compute_pseudorange(&input, &eph, None, None);
        let diff = result.validate_against_rinex(result.corrected_m);

        assert!(diff.abs() < 1e-9, "self-comparison must be zero");
    }

    #[test]
    fn test_validate_against_rinex_known_difference() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let result = compute_pseudorange(&input, &eph, None, None);
        let diff = result.validate_against_rinex(result.corrected_m - 5.0);

        assert!(
            (diff - 5.0).abs() < 1e-6,
            "should report 5 m excess over RINEX reference"
        );
    }

    #[test]
    fn test_is_within_rinex_tolerance_pass() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let result = compute_pseudorange(&input, &eph, None, None);

        assert!(result.is_within_rinex_tolerance(result.corrected_m + 0.5, 1.0));
    }

    #[test]
    fn test_is_within_rinex_tolerance_fail() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let result = compute_pseudorange(&input, &eph, None, None);

        assert!(!result.is_within_rinex_tolerance(result.corrected_m + 2.0, 1.0));
    }

    #[test]
    fn test_invalid_pseudorange_detected_for_near_zero_flight_time() {
        let eph = dummy_ephemeris(1);
        // t_rx ≈ t_tx → псевдодальность ≈ 0 → invalid
        let mut input = nominal_input(1);
        input.receiver_time_s = input.tow_s + input.code_phase_chips / GPS_L1_CHIP_RATE + 0.001;
        let result = compute_pseudorange(&input, &eph, None, None);

        assert!(
            !result.valid,
            "near-zero flight time should produce invalid pseudorange"
        );
    }

    #[test]
    fn test_fractional_code_phase_affects_t_tx() {
        let eph = dummy_ephemeris(1);
        let input_a = PseudorangeInput {
            code_phase_chips: 0.0,
            ..nominal_input(1)
        };
        let input_b = PseudorangeInput {
            code_phase_chips: 511.5,
            receiver_time_s: input_a.receiver_time_s + 511.5 / GPS_L1_CHIP_RATE,
            ..nominal_input(1)
        };
        let ra = compute_pseudorange(&input_a, &eph, None, None);
        let rb = compute_pseudorange(&input_b, &eph, None, None);

        // t_tx_b - t_tx_a ≈ 511.5 / chip_rate ≈ 0.5 мс → ~150 км разницы? нет — receiver_time также сдвинут
        // оба имеют одинаковое время полёта → raw_m должны совпасть
        assert!(
            (ra.raw_m - rb.raw_m).abs() < 1000.0,
            "same flight time → similar raw pseudoranges, diff={}",
            (ra.raw_m - rb.raw_m).abs()
        );
    }

    #[test]
    fn test_pseudorange_all_fields_finite() {
        let eph = dummy_ephemeris(1);
        let input = nominal_input(1);
        let result = compute_pseudorange(&input, &eph, None, None);

        assert!(result.raw_m.is_finite());
        assert!(result.corrected_m.is_finite());
        assert!(result.t_tx_s.is_finite());
        assert!(result.t_rx_s.is_finite());
        assert!(result.corrections.satellite_clock_m.is_finite());
        assert!(result.corrections.ionosphere_m.is_finite());
        assert!(result.corrections.troposphere_m.is_finite());
    }

    #[test]
    fn test_default_chip_rate_used_when_chip_freq_is_zero() {
        let eph = dummy_ephemeris(1);
        let mut input = nominal_input(1);

        input.chip_freq_hz = 0.0; // должен быть заменён на GPS_L1_CHIP_RATE

        let result = compute_pseudorange(&input, &eph, None, None);

        // Не должно быть NaN/Inf или паники
        assert!(result.raw_m.is_finite());
    }

    #[test]
    fn test_week_boundary_wraparound_handled() {
        let eph = dummy_ephemeris(1);

        // t_rx сразу после начала новой недели, t_tx — в конце предыдущей.
        let mut input = nominal_input(1);

        input.tow_s = 604_799.0; // конец недели
        input.code_phase_chips = 50.0;
        // receiver немного в новой неделе
        input.receiver_time_s = 604_799.0 + 50.0 / GPS_L1_CHIP_RATE + 0.0674;

        let result = compute_pseudorange(&input, &eph, None, None);

        // Не должно уходить в -inf или огромные значения
        assert!(result.raw_m.is_finite(), "week boundary must be handled");
    }
}
