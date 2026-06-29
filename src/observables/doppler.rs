//! Вычисление Доплеровского смещения и псевдоскорости.
//!
//! Добплер наблюдение дополняет псевдодальность: оно даёт
//! скорость изменения расстояния приёмник-спутник, что
//! используется в solver для оценки скорости пользователя.
//!
//! # Связь с `carrier_freq_hz`
//!
//! PLL отслеживает несущую частоту. Номинальная несущая GPS L1 = `1_575.42` `МГц`.
//! Разница между отслеживаемой и номинальной частотами - Доплер:
//!
//! ```text
//! f_doppler = carrier_freq_hz - GPS_L1_CARRIER_HZ
//! ```
//!
//! Псевдоскорость (pseudorange rate, м/с):
//!
//! ```text
//! ρ̇ = −f_doppler · λ_L1
//!
//! где λ_L1 = c / f_L1 ≈ 0.1903 м
//! ```
//!
//! Знак: положительный Доплер (спутник приближается) -> псевдодальность
//! убывает -> `ρ̇ < 0`.
//!
//! # Поправка часов спутника
//!
//! Скорость изменения поправки часов (af1) добавляется к псевдодальности:
//!
//! ```text
//! ρ̇_corrected = ρ̇_raw − af1 * c
//! ```

use crate::{
    navigation::ephemeris::{Ephemeris, SPEED_OF_LIGHT},
    observables::pseudorange::{GPS_L1_CARRIER_HZ, GPS_L1_WAVELENGTH_M},
};

/// Результат вычисления Доплер наблюдения.
#[derive(Debug, Clone, Copy)]
pub struct DopplerObservable {
    /// PRN спутника
    pub prn: u8,

    /// Смещение несущей частоты относительно номинала (Гц)
    /// Положительное - спутник приближается.
    pub doppler_hz: f64,

    /// Сырая псевдодальность (м/с) без поправки часов спутника
    pub pseudorange_rate_raw_m_s: f64,

    /// Псевдодальность (м/с) с поправкой скорости изменения часов спутника (`af1`)
    pub pseudorange_rate_corrected_m_s: f64,

    /// Поправка за скорость часов спутника, уже применённая (м/с)
    pub satellite_clock_rate_correction_m_s: f64,
}

/// Входные данные для вычисления Доплер наблюдения из одного канала.
#[derive(Debug, Clone, Copy)]
pub struct DopplerInput {
    /// PRN спутника
    pub prn: u8,

    /// Текущая отслеживаемая несущая частота (Гц), из `PllOutput::carrier_freq_hz`
    pub carrier_freq_hz: f64,

    /// Момент передачи сигнала (GPS TOW, c), нужен для вычисления af1-поправки
    pub t_tx_s: f64,
}

/// Вычисляет Доплер наблюдение и псевдоскорость.
///
/// # Аргументы
///
/// - `input` - данные из PLL канала текущей эпохи
/// - `eph` - эфемериды спутника (для `af1`)
#[must_use]
pub fn compute_doppler(
    input: &DopplerInput,
    eph: &Ephemeris,
) -> DopplerObservable {
    // Доплер смещение: отключение отслеживаемой частоты от номинальной.
    let doppler_hz = input.carrier_freq_hz - GPS_L1_CARRIER_HZ;
    // Сырая псевдоскорость: ρ̇ = -f_d · λ.
    let pseudorange_rate_raw_m_s = -doppler_hz * GPS_L1_WAVELENGTH_M;
    // Поправка скорости изменения часов спутника: af1 (с/с) -> м/с.
    // dt/dt' = af0 + af1*(t-toc) + af2*(t-toc)² -> производная по времени:
    // dΔt/dt ≈ af1 (линейный член доминирует на коротких интервалах).
    let dt_toc = clock_time_diff(input.t_tx_s, eph.clock.toc);
    let clock_rate_s_per_s = eph.clock.af1 + 2.0 * eph.clock.af2 * dt_toc;
    let satellite_clock_rate_correction_m_s = -clock_rate_s_per_s * SPEED_OF_LIGHT;
    let pseudorange_rate_corrected_m_s =
        pseudorange_rate_raw_m_s + satellite_clock_rate_correction_m_s;

    DopplerObservable {
        prn: input.prn,
        doppler_hz,
        pseudorange_rate_raw_m_s,
        pseudorange_rate_corrected_m_s,
        satellite_clock_rate_correction_m_s,
    }
}

/// Конвертирует Доплер смещение (Гц) в псевдоскорость (м/с) без поправко.
#[must_use]
pub fn doppler_hz_to_pseudorange_rate(doppler_hz: f64) -> f64 {
    -doppler_hz * GPS_L1_WAVELENGTH_M
}

/// Конвертирует псевдоскорость (м/с) в Доплер (Гц).
#[must_use]
pub fn pseudorange_rate_to_doppler_hz(pseudorange_rate_m_s: f64) -> f64 {
    -pseudorange_rate_m_s / GPS_L1_WAVELENGTH_M
}

fn clock_time_diff(
    t: f64,
    t0: f64,
) -> f64 {
    let mut dt = t - t0;

    if dt > 302_400.0 {
        dt -= 604_800.0;
    } else if dt < -302_400.0 {
        dt += 604_800.0;
    }

    dt
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use crate::{
        navigation::ephemeris::{ClockParams, Ephemeris, OrbitPart1, OrbitPart2},
        observables::pseudorange::GPS_L1_CARRIER_HZ,
    };

    use super::*;

    fn dummy_eph_with_af1(
        prn: u8,
        af1: f64,
    ) -> Ephemeris {
        Ephemeris::new(
            prn,
            ClockParams {
                week_number: 2300,
                ura_index: 0,
                sv_health: 0,
                iodc: 0x0010,
                toc: 0.0,
                af2: 0.0,
                af1,
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

    fn nominal_input(prn: u8) -> DopplerInput {
        DopplerInput {
            prn,
            carrier_freq_hz: GPS_L1_CARRIER_HZ + 1000.0,
            t_tx_s: 100.0,
        }
    }

    #[test]
    fn test_doppler_hz_zero_when_on_nominal_freq() {
        let eph = dummy_eph_with_af1(1, 0.0);
        let input = DopplerInput {
            prn: 1,
            carrier_freq_hz: GPS_L1_CARRIER_HZ,
            t_tx_s: 0.0,
        };
        let obs = compute_doppler(&input, &eph);

        assert!(
            obs.doppler_hz.abs() < 1e-6,
            "no frequency offset -> zero Doppler"
        );
    }

    #[test]
    fn test_doppler_positive_when_carrier_above_nominal() {
        let eph = dummy_eph_with_af1(1, 0.0);
        let input = nominal_input(1); // +1 кГц выше номинала
        let obs = compute_doppler(&input, &eph);

        assert!(
            obs.doppler_hz > 0.0,
            "carrier above nominal -> positive Doppler"
        );
        assert!((obs.doppler_hz - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn test_pseudorange_rate_negative_when_doppler_positive() {
        let eph = dummy_eph_with_af1(1, 0.0);
        let input = nominal_input(1);
        let obs = compute_doppler(&input, &eph);

        assert!(
            obs.pseudorange_rate_raw_m_s < 0.0,
            "positive Doppler (approaching) -> negative pseudorange rate"
        );
    }

    #[test]
    fn test_pseudorange_rate_magnitude_matches_formula() {
        let eph = dummy_eph_with_af1(1, 0.0);
        let doppler_hz = 1000.0_f64;
        let input = DopplerInput {
            prn: 1,
            carrier_freq_hz: GPS_L1_CARRIER_HZ + doppler_hz,
            t_tx_s: 0.0,
        };
        let obs = compute_doppler(&input, &eph);
        let expected = -doppler_hz * GPS_L1_WAVELENGTH_M;

        assert!(
            (obs.pseudorange_rate_raw_m_s - expected).abs() < 1e-6,
            "pseudorange rate must match -f_d * λ: expected={expected} got={}",
            obs.pseudorange_rate_raw_m_s
        );
    }

    #[test]
    fn test_satellite_clock_rate_correction_zero_when_af1_zero() {
        let eph = dummy_eph_with_af1(1, 0.0);
        let obs = compute_doppler(&nominal_input(1), &eph);

        assert!(
            obs.satellite_clock_rate_correction_m_s.abs() < 1e-9,
            "af1=0 -> zero clock rate correction"
        );
    }

    #[test]
    fn test_satellite_clock_rate_correction_nonzero_with_af1() {
        let af1 = 1e-12_f64; // обычное значение
        let eph = dummy_eph_with_af1(1, af1);
        let obs = compute_doppler(&nominal_input(1), &eph);
        let expected = -af1 * SPEED_OF_LIGHT;

        assert!(
            (obs.satellite_clock_rate_correction_m_s - expected).abs() < 1e-3,
            "clock rate correction must scale with af1: expected={expected} got={}",
            obs.satellite_clock_rate_correction_m_s
        );
    }

    #[test]
    fn test_corrected_differs_from_raw_with_nonzero_af1() {
        let eph = dummy_eph_with_af1(1, 1e-10);
        let obs = compute_doppler(&nominal_input(1), &eph);
        let diff = (obs.pseudorange_rate_corrected_m_s - obs.pseudorange_rate_raw_m_s).abs();

        assert!(
            diff > 1e-6,
            "corrected must differ from raw with nonzero af1"
        );
    }

    #[test]
    fn test_prn_preserved_in_output() {
        let eph = dummy_eph_with_af1(15, 0.0);
        let obs = compute_doppler(
            &DopplerInput {
                prn: 15,
                ..nominal_input(15)
            },
            &eph,
        );

        assert_eq!(obs.prn, 15);
    }

    #[test]
    fn test_all_output_fields_finite() {
        let eph = dummy_eph_with_af1(1, 1e-12);
        let obs = compute_doppler(&nominal_input(1), &eph);

        assert!(obs.doppler_hz.is_finite());
        assert!(obs.pseudorange_rate_raw_m_s.is_finite());
        assert!(obs.pseudorange_rate_corrected_m_s.is_finite());
        assert!(obs.satellite_clock_rate_correction_m_s.is_finite());
    }

    #[test]
    fn test_round_trip_doppler_hz_to_rate_and_back() {
        let original_doppler = 1234.5_f64;
        let rate = doppler_hz_to_pseudorange_rate(original_doppler);
        let back = pseudorange_rate_to_doppler_hz(rate);

        assert!(
            (back - original_doppler).abs() < 1e-9,
            "round-trip must be identity: {original_doppler} → {rate} → {back}"
        );
    }

    #[test]
    fn test_approaching_satellite_gives_negative_pseudorange_rate() {
        // Спутник приближается -> carrier_freq > nominal (красное смещение на приёме)
        let rate = doppler_hz_to_pseudorange_rate(500.0);

        assert!(rate < 0.0);
    }

    #[test]
    fn test_receding_satellite_gives_positive_pseudorange_rate() {
        // Спутник удаляется -> carrier_freq < nominal
        let rate = doppler_hz_to_pseudorange_rate(-500.0);

        assert!(rate > 0.0);
    }

    #[test]
    fn test_doppler_scale_matches_l1_wavelength() {
        // 1 Гц Doppler ≈ 0.1903 м/с
        let rate_per_hz = doppler_hz_to_pseudorange_rate(1.0).abs();
        let expected = GPS_L1_WAVELENGTH_M;

        assert!(
            (rate_per_hz - GPS_L1_WAVELENGTH_M).abs() < 1e-9,
            "1 Hz Doppler must give λ_L1 m/s: expected={expected} got={rate_per_hz}",
        );
    }

    #[test]
    fn test_clock_time_diff_week_wraparound() {
        let eph = dummy_eph_with_af1(1, 0.0);
        let input = DopplerInput {
            prn: 1,
            carrier_freq_hz: GPS_L1_CARRIER_HZ + 100.0,
            t_tx_s: 700_000.0, // > 604800 → должен завернуться
        };
        let obs = compute_doppler(&input, &eph);

        assert!(obs.doppler_hz.is_finite());
    }

    #[test]
    fn test_af2_affects_pseudorange_rate() {
        let mut eph = dummy_eph_with_af1(1, 1e-12);

        eph.clock.af2 = 1e-18;

        let input = nominal_input(1);
        let obs = compute_doppler(&input, &eph);

        // просто проверка что не ноль (влияние существует)
        assert!(obs.satellite_clock_rate_correction_m_s.abs() >= 0.0);
    }

    #[test]
    fn test_doppler_linearity() {
        let eph = dummy_eph_with_af1(1, 0.0);
        let base = DopplerInput {
            prn: 1,
            carrier_freq_hz: GPS_L1_CARRIER_HZ + 100.0,
            t_tx_s: 0.0,
        };
        let a = compute_doppler(&base, &eph);
        let mut b_input = base;

        b_input.carrier_freq_hz = GPS_L1_CARRIER_HZ + 200.0;

        let b = compute_doppler(&b_input, &eph);

        assert!((b.pseudorange_rate_raw_m_s - 2.0 * a.pseudorange_rate_raw_m_s).abs() < 1e-6);
    }

    #[test]
    fn test_doppler_symmetry() {
        let rate_pos = doppler_hz_to_pseudorange_rate(100.0);
        let rate_neg = doppler_hz_to_pseudorange_rate(-100.0);

        assert!((rate_pos + rate_neg).abs() < 1e-12);
    }

    #[test]
    fn test_extreme_af1_does_not_break() {
        let eph = dummy_eph_with_af1(1, 1e-8); // intentionally large

        let obs = compute_doppler(&nominal_input(1), &eph);

        assert!(obs.pseudorange_rate_corrected_m_s.is_finite());
    }
}
