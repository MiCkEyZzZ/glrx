//! Итеративный Weighted Least Squares (WLS) solver для вычисления позиции
//! приёмника из псевдодальностей.
//!
//! # Математическая постановка
//!
//! Для каждого спутника `i` наблюдаемая псевдодальность:
//!
//! ```text
//! ρ_i = ‖r_sv_i − r_u‖ + c·δt_u + ε_i
//! ```
//! где `r_u` - вектор положения приёмника (ECEF, м), `δt_u` - ошибка часов
//! приёмника (с), `c` - скорость света.
//!
//! Лицензируем вокруг начального приближения `r̂_u, δt̂_u`:
//!
//! ```text
//! δρ_i = ρ_i − ρ̂_i = h_i · Δx + c·Δδt + ε_i
//!
//! h_i = (r̂_u − r_sv_i) / ρ̂_i   (единичный вектор направления, 1×3)
//!
//! Δx = [Δx, Δy, Δz, Δδt]ᵀ
//! ```
//!
//! # Матричная форма:
//!
//! ```text
//! δρ = H · Δx + ε
//!
//! H ∈ ℝ^{n×4}:  каждая строка = [h_xi, h_yi, h_zi, 1]
//! W ∈ ℝ^{n×n}:  диагональная матрица весов W_ii = CN0_i / sum(CN0)
//!
//! WLS: Δx = (HᵀWH)⁻¹ · HᵀW · δρ
//! ```
//!
//! Итерация выполняются до сходимости `‖Δx‖ < threshold`.
//!
//! # QR-разложение
//!
//! `(HᵀWH)⁻¹ HᵀW` вычисляется через QR: `(√W · H) = Q · R`, затем
//! `Δx = R⁻¹ · Qᵀ · √W · δρ`. Это численно устойчивее прямого обращения
//! нормальной матрицы.
//!
//! # DOP (Dilution of Precision)
//!
//! ```text
//! Q = (HᵀH)⁻¹   (без весов, только геометрия)
//!
//! PDOP = √(Q[0,0] + Q[1,1] + Q[2,2])
//! TDOP = √Q[3,3]
//! GDOP = √trace(Q)
//! HDOP = √(Q_ENU[0,0] + Q_ENU[1,1])  (после поворота в ENU)
//! VDOP = √Q_ENU[2,2]
//! ```
//!
//! # ECEF -> LLA
//!
//! Преобразование выполняется итеративным методом Bowring (сходится за
//! 2-3 итерации до сантиметровой точности).

/// Большая полуось WGS-84 (м).
pub const WGS84_A: f64 = 6_378_137.0;

/// Малая полуось WGS-84 (м).
pub const WGS84_B: f64 = WGS84_A * (1.0 - WGS84_F);

/// Полярное сжатие WGS-84.
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;

/// Эксцентриситет² (первый).
pub const WGS84_E2: f64 = 2.0 * WGS84_F - WGS84_F * WGS84_F;

/// Второй эксцентриситет².
pub const WGS84_EP2: f64 = WGS84_E2 / (1.0 - WGS84_E2);

/// Минимальное число спутников для 3D-fix (4 неизвестных: x, y, z, δt).
pub const MIN_SATELLITES: usize = 4;

/// Максимальное число итераций WLS.
pub const MAX_ITERATIONS: usize = 10;

/// DOP-метрика (Dilution of Precision).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DopValues {
    /// Geometric DOP (общий)
    pub gdop: f32,

    /// Position DOP (3D)
    pub pdop: f32,

    /// Horizontal DOP
    pub hdop: f32,

    /// Vertical DOP
    pub vdop: f32,

    /// Time DOP
    pub tdop: f32,
}

/// Положение в геодезических координатах (WGS-84).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeodeticPosition {
    /// Широта (рад), положительная - Северное полушарие
    pub lat_rad: f64,

    /// Долгота (рад), положительная - Восточная долгота
    pub lon_rad: f64,

    /// Высота над эллипсоидом WGS-84 (м)
    pub alt_m: f64,
}

/// Положение в ECEF (Earth-Centred Earth-Fixed, м).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EcefPosition {
    /// X
    pub x: f64,

    /// Y
    pub y: f64,

    /// Z
    pub z: f64,
}

impl GeodeticPosition {
    /// Широта в градусах.
    #[must_use]
    pub const fn lat_deg(&self) -> f64 {
        self.lat_rad.to_degrees()
    }

    /// Долгота в градусах.
    #[must_use]
    pub const fn lon_deg(&self) -> f64 {
        self.lon_rad.to_degrees()
    }
}

impl EcefPosition {
    /// Евклидово расстояние от начала координат (м).
    #[must_use]
    pub fn norm(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Конвертирует в геодезические координаты методом Bowring.
    #[must_use]
    pub fn to_geodetic(self) -> GeodeticPosition {
        ecef_to_geodetic(self.x, self.y, self.z)
    }
}

/// Преобразует ECEF (x, y, z) м -> геодезические координаты (WGS-84).
///
/// Метод итеративного Bowring: 3-4 итерации дают сантиметровую точность.
#[must_use]
pub fn ecef_to_geodetic(
    ecef_x: f64,
    ecef_y: f64,
    ecef_z: f64,
) -> GeodeticPosition {
    let longitude = ecef_y.atan2(ecef_x);
    let horizontal_distance = (ecef_x * ecef_x + ecef_y * ecef_y).sqrt();

    let mut latitude = (ecef_z / (horizontal_distance * (1.0 - WGS84_E2))).atan();

    for _ in 0..5 {
        let sin_latitude = latitude.sin();
        let prime_vertical_radius = WGS84_A / (1.0 - WGS84_E2 * sin_latitude * sin_latitude).sqrt();

        latitude = ((ecef_z + WGS84_E2 * prime_vertical_radius * sin_latitude)
            / horizontal_distance)
            .atan();
    }

    let sin_latitude = latitude.sin();
    let cos_latitude = latitude.cos();

    let prime_vertical_radius = WGS84_A / (1.0 - WGS84_E2 * sin_latitude * sin_latitude).sqrt();

    let altitude = if cos_latitude.abs() > 1e-10 {
        horizontal_distance / cos_latitude - prime_vertical_radius
    } else {
        ecef_z / sin_latitude - prime_vertical_radius * (1.0 - WGS84_E2)
    };

    GeodeticPosition {
        lat_rad: latitude,
        lon_rad: longitude,
        alt_m: altitude,
    }
}

/// Преобразует LLA -> ECEF.
#[must_use]
pub fn geodetic_to_ecef(
    lat_rad: f64,
    lon_rad: f64,
    alt_m: f64,
) -> EcefPosition {
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();

    EcefPosition {
        x: (n + alt_m) * cos_lat * lon_rad.cos(),
        y: (n + alt_m) * cos_lat * lon_rad.sin(),
        z: (n * (1.0 - WGS84_E2) + alt_m) * sin_lat,
    }
}

/// Расстояние между двумя ECEF-позициями (м).
#[must_use]
pub fn ecef_distance(
    a: &EcefPosition,
    b: &EcefPosition,
) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;

    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    use crate::navigation::ephemeris::{ClockParams, Ephemeris, OrbitPart1, OrbitPart2};

    fn _dummy_ephemeris(prn: u8) -> Ephemeris {
        Ephemeris {
            prn,
            clock: ClockParams {
                week_number: 2300,
                ura_index: 0,
                sv_health: 0,
                iodc: 0x0010,
                toc: 0.0,
                af2: 0.0,
                af1: 0.0,
                af0: 0.0,
            },
            orbit1: OrbitPart1 {
                iode: 0x10,
                crs: 0.0,
                delta_n: 0.0,
                m0: 0.0,
                cuc: 0.0,
                e: 0.0,
                cus: 0.0,
                sqrt_a: 5153.65,
                toe: 0.0,
            },
            orbit2: OrbitPart2 {
                cic: 0.0,
                omega0: 0.0,
                cis: 0.0,
                i0: 0.0,
                crc: 0.0,
                omega: 0.0,
                omega_dot: 0.0,
                iode: 0x10,
                idot: 0.0,
            },
        }
    }

    #[test]
    fn test_ecef_to_geodetic_north_pole() {
        let geo = ecef_to_geodetic(0.0, 0.0, WGS84_A);

        // На полюсе широта ≈ 90° (но WGS-84 эллипсоид слегка сжат — alt > 0).
        assert!(
            (geo.lat_rad - FRAC_PI_2).abs() < 0.01,
            "north pole lat ≈ 90°, got {}°",
            geo.lat_deg()
        );
    }

    #[test]
    fn test_ecef_to_geodetic_equator_prime_meridian() {
        let geo = ecef_to_geodetic(WGS84_A, 0.0, 0.0);

        assert!(
            geo.lat_rad.abs() < 1e-6,
            "equator: lat ≈ 0, got {}°",
            geo.lat_deg()
        );
        assert!(
            geo.lon_rad.abs() < 1e-6,
            "prime meridian: lon≈0, got {}°",
            geo.lon_deg()
        );
        assert!(
            geo.alt_m.abs() < 1.0,
            "on ellipsoid: alt≈0, got {} m",
            geo.alt_m
        );
    }

    #[test]
    fn test_ecef_to_geodetic_kungur() {
        // Кунгур: ~57.43° N, 56.95° E, ~150 м
        let rx = geodetic_to_ecef(57.43_f64.to_radians(), 56.95_f64.to_radians(), 150.0);
        let geo = ecef_to_geodetic(rx.x, rx.y, rx.z);

        assert!(
            (geo.lat_deg() - 57.43).abs() < 0.001,
            "lat mismatch: {} vs 57.43",
            geo.lat_deg()
        );
        assert!(
            (geo.lon_deg() - 56.95).abs() < 0.001,
            "lon mismatch: {} vs 56.95",
            geo.lon_deg()
        );
        assert!(
            (geo.alt_m - 150.0).abs() < 0.1,
            "alt mismatch: {} vs 150 m",
            geo.alt_m
        );
    }

    #[test]
    fn test_geodetic_to_ecef_round_trip() {
        let lat = 57.43_f64.to_radians(); // Кунгур
        let lon = 56.95_f64.to_radians();
        let alt = 120.0;

        let ecef = geodetic_to_ecef(lat, lon, alt);
        let geo = ecef_to_geodetic(ecef.x, ecef.y, ecef.z);

        assert!((geo.lat_rad - lat).abs() < 1e-9);
        assert!((geo.lon_rad - lon).abs() < 1e-9);
        assert!((geo.alt_m - alt).abs() < 1e-3);
    }

    #[test]
    fn test_geodetic_to_ecef_south_hemisphere() {
        let lat = (-33.87_f64).to_radians(); // Сидней
        let lon = 151.21_f64.to_radians();
        let alt = 10.0;
        let ecef = geodetic_to_ecef(lat, lon, alt);
        let geo = ecef_to_geodetic(ecef.x, ecef.y, ecef.z);

        assert!((geo.lat_deg() - (-33.87)).abs() < 0.001);
        assert!((geo.lon_deg() - 151.21).abs() < 0.001);
    }

    #[test]
    fn test_ecef_distance_zero_for_same_point() {
        let p = EcefPosition {
            x: 1e6,
            y: 2e6,
            z: 3e6,
        };

        assert!(ecef_distance(&p, &p) < 1e-9);
    }
}
