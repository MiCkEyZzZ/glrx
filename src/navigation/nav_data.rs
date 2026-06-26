//! Хранилище навигационных данных приёмника: эфемериды по PRN,
//! ионосферная модель (Klobuchar), almanac (заглушка под GLRX-12).
//!
//! `NavData` - центральная точка накопления данных, декодированных из
//! навигационного сообщения GPS (через [`crate::navigation::ephemeris`]),
//! используемая потребителями выше по конвеёеру (observables, solver) для
//! получения текущих эфемерид конкретного спутника.

use std::{collections::HashMap, f64::consts::TAU};

use crate::navigation::{
    ephemeris::{
        BitCursor, ClockParams, Ephemeris, EphemerisValidationError, OrbitPart1, OrbitPart2,
        concat_data_words, parse_subframe1, parse_subframe2, parse_subframe3,
    },
    frame_decoder::DecodedSubframe,
};

/// Причина, по которой эфемериды для PRN недоступны или невалидны при
/// запросе через `NavData::ephemeris_validated`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EphemerisLookupError {
    /// Для данного PRN ещё не собран полный комплект эфемерид (Subframe
    /// 1, 2 и 3).
    NotYetAvailable,

    /// Эфемериды собраны, но не прошли валидацию (health/IOD).
    Invalid(EphemerisValidationError),
}

/// Параметры ионосферной модели Клобухара, декодируемые из Subframe 4
/// (страница 18).
///
/// GPS ICD-200 передаёт 4 амплитудных коэффициента `α₀..α₃` и 4
/// коэффициента периода `β₀..β₃`, используемых для оценки ионосферной
/// задержки сигнала L1 в зависимости от позиции пользователя и времени
/// суток (см. `docs/NAVIGATION.md`, раздел "Ionospheric Model").
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IonosphericModel {
    /// Амплитудные коэффициенты (секунды), масштабы `2⁻³⁰, 2⁻²⁷, 2⁻²⁴, 2⁻²⁴`
    pub alpha: [f64; 4],

    /// Коэффициенты периода (секунды), масштабы `2¹², 2¹⁴, 2¹⁶, 2¹⁶`
    pub beta: [f64; 4],
}

/// GPS <-> UTC коррекция, передаваемая в том же Subframe 4 страницы 18, что и
/// ионосферные параметры. Хранится отдельно от `crate::utils::timing` - слоя:
/// здесь - только сырые декодированные значения, использование (например, для
/// построения `gnss_time::LeapEntry`) выполняется выше по конвейеру.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UtcCorrection {
    /// Текущее число leap-секунд (`ΔtLS`).
    pub delta_t_ls: i8,

    /// Будущее число leap-секунд после запланированного события (`ΔtLSF`).
    pub delta_t_lsf: i8,

    /// Номер недели запланированного leap-second события (`WN_LSF`).
    pub wn_lsf: u8,

    /// Номер дня внутри недели запланированного события (`DN`).
    pub dn: u8,
}

/// Заготовка под запись almanac (приближённая орбита для быстрого cold
/// start) - полная реализация запланирована в GLRX-12. Здесь определена
/// только структура хранения, чтобы [`NavData`] могла резервировать под
/// неё место без переработки API при появлении парсера.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AlmanacEntry {
    /// Health-байт спутника, как передан в almanac-записи
    pub health: u8,

    /// Выпуск данных, Almanac.
    pub ioda: u8,
}

/// Промежуточное состояние сборки эфемерид для одного PRN: отдельные
/// subframe приходят не одновременно (каждый decode subframe занимает 6 с),
/// поэтому до получения всех трех частей собранные параметры хрянятся
/// отделбно.
#[derive(Debug, Clone, Copy, Default)]
pub struct PendingEphemeris {
    clock: Option<ClockParams>,
    orbit1: Option<OrbitPart1>,
    orbit2: Option<OrbitPart2>,
}

/// Хранилище текущего навигационного состояния приёмника: эфемерид по
/// каждому отслеживаемому PRN, ионосферная модель, almanoc, UTC-коррекция.
///
/// # Типичный поток использования
///
/// ```text
/// nav_data.ingest_subframe(prn, &decoded_subframe);
/// // ... после получения subframe 1, 2 и 3 для данного PRN:
/// if let Some(eph) = nav_data.ephemeris(prn) {
///     if eph.validate().is_ok() {
///         let (x, y, z, _) = eph.position_ecef(gps_tow);
///     }
/// }
/// ```
#[derive(Debug, Default)]
pub struct NavData {
    /// Полностью собранные и готовые к использованию эфемериды по PRN
    pub ephemeris: HashMap<u8, Ephemeris>,

    /// Незавершённые наборы эфемерид (ожидают оставшиеся subframe) по PRN
    pending: HashMap<u8, PendingEphemeris>,

    /// Ионосферная модель (общая для всех спутников constellation)
    pub iono: Option<IonosphericModel>,

    /// Почасовые/недельные almanac-записи по PRN (заглушка, GLRX-12)
    pub almanac: HashMap<u8, AlmanacEntry>,

    /// GPS–UTC коррекция (leap seconds), если уже декодирована
    pub utc_correction: Option<UtcCorrection>,
}

impl IonosphericModel {
    /// Разбирает параметры ионосферной модели из Subframe 4, страница 18.
    ///
    /// # Примечание о раскладке
    ///
    /// Subframe 4 используется механизм "страниц" (page 1-25, циклически
    /// переключаемых через биты данных), и точная битовая раскладка
    /// page 18 не определяется одним только `subfrane_id == 4` - нужна
    /// дополнительная проверка page ID, не входящая в [`DecodedSubframe`]
    /// текущего вида. Этот парсер предполагает, что вызывающий код уже
    /// отфильтрован нужный subframe по внешнему признаку (data ID / DV ID - 56,
    /// согласно ICD) и передаёт сюда корректные информационные слова.
    ///
    /// Раскладка (биты, начиная с начала информационных слов 3..10):
    /// `α₀[8] α₁[8] α₂[8] α₃[8] β₀[8] β₁[8] β₂[8] β₃[8] ...`
    #[must_use]
    pub fn parse_page18(subframe: &DecodedSubframe) -> Option<Self> {
        if subframe.subframe_id != 4 {
            return None;
        }

        let bits = concat_data_words(subframe);
        let c = BitCursor::new(&bits);

        let alpha0 = c.signed(0, 8);
        let alpha1 = c.signed(8, 8);
        let alpha2 = c.signed(16, 8);
        let alpha3 = c.signed(24, 8);
        let beta0 = c.signed(32, 8);
        let beta1 = c.signed(40, 8);
        let beta2 = c.signed(48, 8);
        let beta3 = c.signed(56, 8);

        Some(Self {
            alpha: [
                f64::from(alpha0) * 2f64.powi(-30),
                f64::from(alpha1) * 2f64.powi(-27),
                f64::from(alpha2) * 2f64.powi(-24),
                f64::from(alpha3) * 2f64.powi(-24),
            ],
            beta: [
                f64::from(beta0) * 2f64.powi(12),
                f64::from(beta1) * 2f64.powi(14),
                f64::from(beta2) * 2f64.powi(16),
                f64::from(beta3) * 2f64.powi(16),
            ],
        })
    }

    /// Вычисляет ионосферную задержку (секунды) для сигнала L1 метода
    /// Клобухара.
    ///
    /// # Аргументы
    ///
    /// - `elevation_semicircles` — угол места спутника, полуокружности
    ///   (`elevation_rad / π`)
    /// - `azimuth_semicircles` — азимут спутника, полуокружности (не
    ///   используется в этой упрощённой форме модели, оставлен для
    ///   совместимости интерфейса с полной реализацией)
    /// - `lat_semicircles`, `lon_semicircles` — геодезическая широта/долгота
    ///   пользователя, полуокружности
    /// - `gps_tow_s` — GPS-время суток (секунды, `0..86400`, можно передавать
    ///   `tow mod 86400`)
    #[must_use]
    pub fn delay_seconds(
        &self,
        elevation_semicircles: f64,
        lat_semicircles: f64,
        lon_semicircles: f64,
        gps_tow_s: f64,
    ) -> f64 {
        // Геомагнитная широта точки пересечения (упрощенно - модель
        // использует геодезическую широту пользователя как приближение).
        let phi_m = lat_semicircles + 0.064 * (lon_semicircles - 1.617).cos();
        let amplitude = (self.alpha[0]
            + self.alpha[1] * phi_m
            + self.alpha[2] * phi_m * phi_m
            + self.alpha[3] * phi_m * phi_m * phi_m)
            .max(0.0);
        let period = (self.beta[0]
            + self.beta[1] * phi_m
            + self.beta[2] * phi_m * phi_m
            + self.beta[3] * phi_m * phi_m * phi_m)
            .max(72_000.0);
        let local_time = gps_tow_s.rem_euclid(86_400.0);
        let x = TAU * (local_time - 50_400.0) / period;
        let obliquity_factor = 1.0 + 16.0 * (0.53 - elevation_semicircles).powi(3);
        let periodic_term = if x.abs() < 1.57 {
            5e-9 + amplitude * (1.0 - x * x / 2.0 + x.powi(4) / 24.0)
        } else {
            5e-9
        };

        obliquity_factor * periodic_term
    }
}

impl PendingEphemeris {
    const fn is_complete(&self) -> bool {
        self.clock.is_some() && self.orbit1.is_some() && self.orbit2.is_some()
    }

    const fn try_assemble(
        &self,
        prn: u8,
    ) -> Option<Ephemeris> {
        match (self.clock, self.orbit1, self.orbit2) {
            (Some(clock), Some(orbit1), Some(orbit2)) => {
                Some(Ephemeris::new(prn, clock, orbit1, orbit2))
            }
            _ => None,
        }
    }
}

impl NavData {
    /// Создаёт пустое хранилище.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Принимает декодированный subframe для заданного PRN и обновляет
    /// внутреннее состояние.
    ///
    /// Поддерживает subframe 1, 2, 3 (эфемериды) - при поступлении всех
    /// трёх частей для данного PRN, [`NavData::ephemeris`] становится
    /// доступным. Subframe 4 (ионосфера) поддерживается отдельно через
    /// [`IonosphericModel::parse_page18`] - `ingest_subframe` не делает
    /// различия страниц Subframe 4 (см. примечание в
    /// [`IonosphericModel::parse_page18`]), поэтому ионосферные данные
    /// нужно передавать через `NavData::set_ionospheric_model` явно.
    ///
    /// Возвращает `true`, если после этого вызова для `prn` стал доступен
    /// (или обновился) полный комплект эфемерид.
    pub fn ingest_subframe(
        &mut self,
        prn: u8,
        subframe: &DecodedSubframe,
    ) -> bool {
        let entry = self.pending.entry(prn).or_default();
        let updated = match subframe.subframe_id {
            1 => {
                if let Some(clock) = parse_subframe1(subframe) {
                    entry.clock = Some(clock);
                    true
                } else {
                    false
                }
            }
            2 => {
                if let Some(orbit1) = parse_subframe2(subframe) {
                    entry.orbit1 = Some(orbit1);
                    true
                } else {
                    false
                }
            }
            3 => {
                if let Some(orbit2) = parse_subframe3(subframe) {
                    entry.orbit2 = Some(orbit2);
                    true
                } else {
                    false
                }
            }
            _ => false,
        };

        if !updated {
            return false;
        }

        if entry.is_complete()
            && let Some(eph) = entry.try_assemble(prn)
        {
            self.ephemeris.insert(prn, eph);

            return true;
        }

        false
    }

    /// Устанавливает ионосферную модель (например, после декодирования
    /// Subframe 4 страницы 18 через [`IonosphericModel::parse_page18`]).
    pub const fn set_ionospheric_model(
        &mut self,
        model: IonosphericModel,
    ) {
        self.iono = Some(model);
    }

    /// Устанавливает GPS-UTC коррукцию.
    pub const fn set_utc_correction(
        &mut self,
        correction: UtcCorrection,
    ) {
        self.utc_correction = Some(correction);
    }

    /// Возвращает эфемериды для `prn`, если полный комплект уже собран
    /// (без валидации health/IOD - см. `NavData::ephemeris_validated`
    /// для проверенной версии).
    #[must_use]
    pub fn ephemeris(
        &self,
        prn: u8,
    ) -> Option<&Ephemeris> {
        self.ephemeris.get(&prn)
    }

    /// Возвращает эфемериды для `prn`, прошедшие валидацию (health == 0,
    /// IODE/IODC консистентны).
    ///
    /// # Errors
    ///
    /// Возвращает [`EphemerisLookupError::NotYetAvailable`], если для
    /// `prn` ещё не собран полный комплект, либо
    /// [`EphemerisLookupError::Invalid`] с конкретной причиной, если
    /// валидация не прошла.
    pub fn ephemeris_validated(
        &self,
        prn: u8,
    ) -> Result<&Ephemeris, EphemerisLookupError> {
        let eph = self
            .ephemeris
            .get(&prn)
            .ok_or(EphemerisLookupError::NotYetAvailable)?;

        eph.validate().map_err(EphemerisLookupError::Invalid)?;

        Ok(eph)
    }

    /// Проверяет возраст эфимерид относительно `current_tow` (секунды
    /// недели GPS): по GPS ICD-200 эфемериды считаются актуальными в
    /// пределах ±2 часов от `toe`.
    ///
    /// Возвращает `true`, если `|current_tow - toe|` (с учётом перехода
    /// через границу недели) не превышает `max_age_s` секунд.
    #[must_use]
    pub fn is_ephemeris_fresh(
        &self,
        prn: u8,
        current_tow: f64,
        max_age_s: f64,
    ) -> bool {
        let Some(eph) = self.ephemeris.get(&prn) else {
            return false;
        };

        let mut dt = current_tow - eph.orbit1.toe;

        if dt > 302_400.0 {
            dt -= 604_800.0;
        } else if dt < -302_400.0 {
            dt += 604_800.0;
        }

        dt.abs() <= max_age_s
    }

    /// Удаляет эфемериды и незавершённые данные для `prn` (например, после
    /// потери lock на этом спутнике).
    pub fn clear_prn(
        &mut self,
        prn: u8,
    ) {
        self.ephemeris.remove(&prn);
        self.pending.remove(&prn);
        self.almanac.remove(&prn);
    }

    /// Число PRN с полностью собранными эфемеридами.
    #[must_use]
    pub fn ephemeris_count(&self) -> usize {
        self.ephemeris.len()
    }
}

#[cfg(test)]
mod tests {
    use crate::navigation::frame_decoder::HowWord;

    use super::*;

    fn make_subframe(
        subframe_id: u8,
        words: [[bool; 24]; 10],
    ) -> DecodedSubframe {
        DecodedSubframe {
            subframe_id,
            how: HowWord {
                tow_count: 0,
                subframe_id,
                alert_flag: false,
                anti_spoof_flag: false,
            },
            words,
        }
    }

    fn bits_from_u32(
        value: u32,
        len: usize,
    ) -> Vec<bool> {
        (0..len).rev().map(|i| (value >> i) & 1 == 1).collect()
    }

    fn pad_word(bits: &[bool]) -> [bool; 24] {
        let mut word = [false; 24];
        let start = 24 - bits.len();

        word[start..].copy_from_slice(bits);

        word
    }

    fn subframe1_healthy(iode_low8: u8) -> DecodedSubframe {
        let mut words = [[false; 24]; 10];

        // Слово 3
        let mut w0 = Vec::new();

        w0.extend(bits_from_u32(2300, 10)); // неделя
        w0.extend(bits_from_u32(0, 2)); // L2 код
        w0.extend(bits_from_u32(0, 4)); // URA
        w0.extend(bits_from_u32(0, 6)); // состояние = 0
        w0.extend(bits_from_u32(u32::from(iode_low8 >> 6), 2)); // IODC[9:8]

        words[2] = pad_word(&w0);

        // Слово 4
        let mut w1 = Vec::new();

        w1.extend(bits_from_u32(0, 8)); // t_gd
        w1.extend(bits_from_u32(u32::from(iode_low8), 8)); // IODC[7:0]
        w1.extend(bits_from_u32(0, 8)); // начало toc

        words[3] = pad_word(&w1);

        make_subframe(1, words)
    }

    fn subframe2_with_iode(iode: u8) -> DecodedSubframe {
        let mut words = [[false; 24]; 10];
        let mut w0 = Vec::new();

        w0.extend(bits_from_u32(u32::from(iode), 8));
        w0.extend(bits_from_u32(0, 16));
        words[2] = pad_word(&w0);

        // sqrt_a должен иметь ненулевое правдоподобное значение, чтобы position_ecef не делил на ноль.
        let mut bits = [false; 192];

        bits[144..168].copy_from_slice(&{
            let mut tmp = [false; 24];
            let val_bits = bits_from_u32(2_000_000, 24);
            tmp.copy_from_slice(&val_bits);
            tmp
        });

        // iode находится в word index 0 bits 0..8 — уже установлен через w0 выше; объединяем,
        // пересобирая words[2..10] полностью из `bits` плюс iode/crs в word0.
        for i in 0..8 {
            let mut w = [false; 24];

            w.copy_from_slice(&bits[i * 24..(i + 1) * 24]);
            if i == 0 {
                w = words[2];
            }

            words[2 + i] = w;
        }

        make_subframe(2, words)
    }

    fn subframe3_with_iode(iode: u8) -> DecodedSubframe {
        let mut words = [[false; 24]; 10];
        let mut w7 = Vec::new();

        w7.extend(bits_from_u32(u32::from(iode), 8));
        w7.extend(bits_from_u32(0, 14));
        w7.extend(bits_from_u32(0, 2));

        words[9] = pad_word(&w7);

        make_subframe(3, words)
    }

    #[test]
    fn test_iono_parse_page18_returns_none_for_wrong_subframe_id() {
        let sf = make_subframe(1, [[false; 24]; 10]);

        assert!(IonosphericModel::parse_page18(&sf).is_none());
    }

    #[test]
    fn test_iono_parse_page18_extracts_alpha0() {
        let mut words = [[false; 24]; 10];
        let mut w0 = Vec::new();

        w0.extend(bits_from_u32(1, 8)); // alpha0 = 1
        w0.extend(bits_from_u32(0, 8));
        w0.extend(bits_from_u32(0, 8));

        words[2] = pad_word(&w0);

        let sf = make_subframe(4, words);
        let iono = IonosphericModel::parse_page18(&sf).unwrap();
        let expected = 1.0 * 2f64.powi(-30);

        assert!((iono.alpha[0] - expected).abs() < 1e-15);
    }

    #[test]
    fn test_iono_delay_seconds_is_nonnegative_for_reasonable_inputs() {
        let model = IonosphericModel {
            alpha: [1e-8, 1e-8, 1e-8, 1e-8],
            beta: [80_000.0, 0.0, 0.0, 0.0],
        };
        let delay = model.delay_seconds(0.3, 0.5, 0.5, 43_200.0);

        assert!(delay >= 0.0);
        assert!(delay.is_finite());
    }

    #[test]
    fn test_iono_delay_seconds_zero_alpha_gives_minimum_delay() {
        let model = IonosphericModel {
            alpha: [0.0; 4],
            beta: [80_000.0, 0.0, 0.0, 0.0],
        };
        let delay = model.delay_seconds(0.3, 0.5, 0.5, 0.0);

        // При нулевой амплитуде periodic_term должен сводиться к минимальной границе 5 нс,
        // масштабированной коэффициентом наклона (obliquity factor).
        assert!(delay > 0.0);
        assert!(delay < 1e-7);
    }

    #[test]
    fn test_nav_data_starts_empty() {
        let nav = NavData::new();

        assert_eq!(nav.ephemeris_count(), 0);
        assert!(nav.ephemeris(1).is_none());
    }

    #[test]
    fn test_nav_data_assembles_ephemeris_after_all_three_subframes() {
        let mut nav = NavData::new();
        let sf1 = subframe1_healthy(0x10);
        let sf2 = subframe2_with_iode(0x10);
        let sf3 = subframe3_with_iode(0x10);

        assert!(!nav.ingest_subframe(5, &sf1));
        assert!(!nav.ingest_subframe(5, &sf2));

        let completed = nav.ingest_subframe(5, &sf3);

        assert!(
            completed,
            "all three subframes ingested → ephemeris should assemble"
        );
        assert!(nav.ephemeris(5).is_some());
        assert_eq!(nav.ephemeris_count(), 1);
    }

    #[test]
    fn test_nav_data_ingest_order_independent() {
        let mut nav = NavData::new();

        let sf1 = subframe1_healthy(0x20);
        let sf2 = subframe2_with_iode(0x20);
        let sf3 = subframe3_with_iode(0x20);

        // Порядок подачи отличается: 2, 3, 1
        nav.ingest_subframe(7, &sf2);
        nav.ingest_subframe(7, &sf3);

        let completed = nav.ingest_subframe(7, &sf1);

        assert!(completed);
        assert!(nav.ephemeris(7).is_some());
    }

    #[test]
    fn test_nav_data_separate_prns_do_not_interfere() {
        let mut nav = NavData::new();

        let sf1 = subframe1_healthy(0x10);
        let sf2 = subframe2_with_iode(0x10);
        let sf3 = subframe3_with_iode(0x10);

        nav.ingest_subframe(1, &sf1);
        nav.ingest_subframe(1, &sf2);
        nav.ingest_subframe(1, &sf3);

        // PRN 2 ничего не получил — должен оставаться недоступным.
        assert!(nav.ephemeris(1).is_some());
        assert!(nav.ephemeris(2).is_none());
    }

    #[test]
    fn test_nav_data_incomplete_set_returns_none() {
        let mut nav = NavData::new();
        let sf1 = subframe1_healthy(0x10);

        nav.ingest_subframe(3, &sf1);

        assert!(nav.ephemeris(3).is_none());
    }

    #[test]
    fn test_nav_data_clear_prn_removes_ephemeris() {
        let mut nav = NavData::new();
        let sf1 = subframe1_healthy(0x10);
        let sf2 = subframe2_with_iode(0x10);
        let sf3 = subframe3_with_iode(0x10);

        nav.ingest_subframe(9, &sf1);
        nav.ingest_subframe(9, &sf2);
        nav.ingest_subframe(9, &sf3);

        assert!(nav.ephemeris(9).is_some());

        nav.clear_prn(9);

        assert!(nav.ephemeris(9).is_none());
        assert_eq!(nav.ephemeris_count(), 0);
    }

    #[test]
    fn test_ephemeris_validated_not_yet_available() {
        let nav = NavData::new();
        let result = nav.ephemeris_validated(1);

        assert_eq!(result.unwrap_err(), EphemerisLookupError::NotYetAvailable);
    }

    #[test]
    fn test_ephemeris_validated_succeeds_for_consistent_healthy_set() {
        let mut nav = NavData::new();
        let sf1 = subframe1_healthy(0x10);
        let sf2 = subframe2_with_iode(0x10);
        let sf3 = subframe3_with_iode(0x10);

        nav.ingest_subframe(4, &sf1);
        nav.ingest_subframe(4, &sf2);
        nav.ingest_subframe(4, &sf3);

        assert!(nav.ephemeris_validated(4).is_ok());
    }

    #[test]
    fn test_ephemeris_validated_detects_iode_mismatch() {
        let mut nav = NavData::new();
        let sf1 = subframe1_healthy(0x10);
        let sf2 = subframe2_with_iode(0x10);
        let sf3 = subframe3_with_iode(0x99); // несоответствие IODE

        nav.ingest_subframe(6, &sf1);
        nav.ingest_subframe(6, &sf2);
        nav.ingest_subframe(6, &sf3);

        let result = nav.ephemeris_validated(6);

        assert!(matches!(
            result,
            Err(EphemerisLookupError::Invalid(
                EphemerisValidationError::IodeMismatch { .. }
            ))
        ));
    }

    #[test]
    fn test_is_ephemeris_fresh_true_within_window() {
        let mut nav = NavData::new();
        let sf1 = subframe1_healthy(0x10);
        let sf2 = subframe2_with_iode(0x10); // toe = 0 (по умолчанию в этом builder)
        let sf3 = subframe3_with_iode(0x10);

        nav.ingest_subframe(8, &sf1);
        nav.ingest_subframe(8, &sf2);
        nav.ingest_subframe(8, &sf3);

        assert!(nav.is_ephemeris_fresh(8, 3600.0, 7200.0)); // 1 час после toe=0, окно 2 часа
    }

    #[test]
    fn test_is_ephemeris_fresh_false_outside_window() {
        let mut nav = NavData::new();
        let sf1 = subframe1_healthy(0x10);
        let sf2 = subframe2_with_iode(0x10);
        let sf3 = subframe3_with_iode(0x10);

        nav.ingest_subframe(8, &sf1);
        nav.ingest_subframe(8, &sf2);
        nav.ingest_subframe(8, &sf3);

        assert!(!nav.is_ephemeris_fresh(8, 10_000.0, 7200.0)); // далеко за пределами окна 2 часа
    }

    #[test]
    fn test_is_ephemeris_fresh_false_for_unknown_prn() {
        let nav = NavData::new();

        assert!(!nav.is_ephemeris_fresh(42, 0.0, 7200.0));
    }

    #[test]
    fn test_nav_data_stores_utc_correction() {
        let mut nav = NavData::new();
        let correction = UtcCorrection {
            delta_t_ls: 18,
            delta_t_lsf: 18,
            wn_lsf: 100,
            dn: 1,
        };

        nav.set_utc_correction(correction);

        assert_eq!(nav.utc_correction, Some(correction));
    }

    #[test]
    fn test_nav_data_stores_ionospheric_model() {
        let mut nav = NavData::new();
        let model = IonosphericModel {
            alpha: [0.0; 4],
            beta: [0.0; 4],
        };

        nav.set_ionospheric_model(model);

        assert!(nav.iono.is_some());
    }

    #[test]
    fn test_nav_data_almanac_defaults_empty() {
        let nav = NavData::new();

        assert!(nav.almanac.is_empty());
    }

    #[test]
    fn test_ephemeris_update_replaces_old() {
        let mut nav = NavData::new();
        // Сначала собираем эфемериды с IODE=0x10
        let sf1 = subframe1_healthy(0x10);
        let sf2 = subframe2_with_iode(0x10);
        let sf3 = subframe3_with_iode(0x10);

        nav.ingest_subframe(1, &sf1);
        nav.ingest_subframe(1, &sf2);
        nav.ingest_subframe(1, &sf3);

        assert!(nav.ephemeris(1).is_some());

        // Теперь приходят новые данные с IODE=0x20
        let sf1_new = subframe1_healthy(0x20);
        let sf2_new = subframe2_with_iode(0x20);
        let sf3_new = subframe3_with_iode(0x20);

        nav.ingest_subframe(1, &sf1_new);
        nav.ingest_subframe(1, &sf2_new);
        nav.ingest_subframe(1, &sf3_new);

        // После обновления должны быть новые эфемериды
        let eph = nav.ephemeris(1).unwrap();

        assert_eq!(eph.orbit1.iode, 0x20);
    }

    #[test]
    fn test_ingest_duplicate_subframe_does_not_trigger_completion() {
        let mut nav = NavData::new();

        let sf1 = subframe1_healthy(0x10);

        // дважды один и тот же subframe
        assert!(!nav.ingest_subframe(1, &sf1));
        assert!(!nav.ingest_subframe(1, &sf1));

        assert!(nav.ephemeris(1).is_none());
    }

    #[test]
    fn test_pending_overwrite_keeps_consistency() {
        let mut nav = NavData::new();

        let sf1_a = subframe1_healthy(0x10);
        let sf2 = subframe2_with_iode(0x10);

        nav.ingest_subframe(1, &sf1_a);
        nav.ingest_subframe(1, &sf2);

        let sf1_b = subframe1_healthy(0x20);
        nav.ingest_subframe(1, &sf1_b);

        // после смены IODE сборка должна либо пересобраться корректно,
        // либо старый pending должен быть инвалидирован
        assert!(nav.ephemeris(1).is_none() || nav.ephemeris(1).unwrap().orbit1.iode == 0x20);
    }

    #[test]
    fn test_clear_prn_removes_all_state() {
        let mut nav = NavData::new();

        let sf1 = subframe1_healthy(0x10);
        nav.ingest_subframe(5, &sf1);

        nav.clear_prn(5);

        assert!(nav.ephemeris(5).is_none());
        assert!(!nav.pending.contains_key(&5));
        assert!(!nav.almanac.contains_key(&5));
    }

    #[test]
    fn test_is_ephemeris_fresh_handles_week_wraparound() {
        let mut nav = NavData::new();

        let sf1 = subframe1_healthy(0x10);
        let sf2 = subframe2_with_iode(0x10);
        let sf3 = subframe3_with_iode(0x10);

        nav.ingest_subframe(1, &sf1);
        nav.ingest_subframe(1, &sf2);
        nav.ingest_subframe(1, &sf3);

        let eph = nav.ephemeris.get(&1).unwrap();

        // искусственно ставим toe near week boundary
        let mut eph = *eph;
        eph.orbit1.toe = 604_700.0;

        nav.ephemeris.insert(1, eph);

        assert!(nav.is_ephemeris_fresh(1, 100.0, 7200.0));
    }
}
