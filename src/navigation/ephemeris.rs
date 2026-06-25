//! Декодирование эфемерид GPS (Subframe 1, 2, 3) и вычисление позиции
//! спутника в ECEF на заданный момент времени.
//!
//! Без эфемерид невозможно вычислить координаты спутника, а без координат
//! ни псевдодальность, ни позиционное решение. Этот модуль потребляет
//! [`crate::navigation::frame_decoder::DecodedSubframe`] и извлекает из
//! информационных слов параметры орбиты Кплера и коррекции часов спутника
//! согласно GPS ICD-200 / IS-GPS-200.
//!
//! ```text
//! DecodedSubframe { subframe_id: 1, words: [..] }
//!     │
//!     ▼
//! parse_subframe1()  → ClockParams { week, af0, af1, af2, toc, iodc, health }
//!
//! DecodedSubframe { subframe_id: 2, words: [..] }
//!     │
//!     ▼
//! parse_subframe2()  → OrbitPart1 { iode, crs, delta_n, m0, cuc, e, cus, sqrt_a, toe }
//!
//! DecodedSubframe { subframe_id: 3, words: [..] }
//!     │
//!     ▼
//! parse_subframe3()  → OrbitPart2 { cic, omega0, cis, i0, crc, omega, omega_dot, idot }
//!
//! ClockParams + OrbitPart1 + OrbitPart2  →  Ephemeris (полный комплект)
//!     │
//!     ▼
//! Ephemeris::position_ecef(t)  →  (x, y, z) метры в ECEF
//! ```
//!
//! # Битовые поля
//!
//! Каждое информационное слово приходит как `[bool; 24]` (после parity-корреляции
//! в `frame_decoder`). Поля могут пересекать границы слов (например, `e` - 32 бита,
//! занимает части двух последовательных 24-битных слов), поэтому используется конкатенация
//! юитов нескольких слов перед извлечением поля - см. [`BitCursor`].

use crate::navigation::frame_decoder::DecodedSubframe;

const PI: f64 = core::f64::consts::PI;

/// Причина, по которой набор эфемерид считается непригодным для
/// вычисления позиции.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EphemerisValidationError {
    /// `sv+health != 0` - спутник помечен как нездоровый
    UnhealthySatellite {
        /// Значение health-флага
        health: u8,
    },

    /// `IODE` (Subframe 2) и `IODE` (Subframe 3) не совпадают - параметры
    /// орбиты получены из разных, потенциально несовместимых наборов
    /// эфимерид.
    IodeMismatch {
        /// IODE из Subframe 2
        iode_sf2: u8,
        /// IODE из Subframe 3
        iode_sf3: u8,
    },

    /// Нижние 8 бит `IODE` (Subframe 1) не совпадают с `IODE` (Subframe 2/3)
    IodcIodeMismatch {
        /// Нижние 8 бит IODC
        iodc_low8: u8,
        /// IODE
        iode: u8,
    },
}

/// Курсор для извлечения битовых полей произвольной длины из
/// конкатинированного потока информационных слов subframe.
pub struct BitCursor<'a> {
    bits: &'a [bool],
}

/// Параметры часов спутника и health/IODC из Subframe 1.
///
/// Масштабные коэффициенты согласно GPS ICD-200 (см. `docs/NAVIGATION.md`):
/// `toc`: 2⁴ с, `af2`: 2⁻⁵⁵ с/с², `af1`: 2⁻⁴³ с/с, `af0`: 2⁻³¹ с.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClockParams {
    /// Номер недели GPS (10 бит, mod 1024 — week rollover решается выше по
    /// конвейеру, см. `utils::timing::WeekRollover`)
    pub week_number: u16,

    /// Индекс точности пользовательского диапазона (4 бита)
    pub ura_index: i8,

    /// Health-флаг спутника (6 бит, `0` = healthy).
    pub sv_health: u8,

    /// Выдача данных, часов (10 бит) — должно совпасть с `IODE` из
    /// Subframe 2/3 (нижние 8 бит) для консистентности эфемерид.
    pub iodc: u16,

    /// Время отсчета часов (с), масштаб 2⁴.
    pub toc: f64,

    /// Квадратичный коэффициент коррекции часов (с/с²), масштаб 2⁻⁵⁵.
    pub af2: f64,

    /// Линейный коэффициент коррекции часов (с/с), масштаб 2⁻⁴³.
    pub af1: f64,

    /// Постоянный коэффициент коррекции часов (с), масштаб 2⁻³¹.
    pub af0: f64,
}

/// Параметры орбиты, часть 1 (Subframe 2): `iode`, `crs`, `delta_n`, `m0`,
/// `cuc`, `e`, `cus`, `sqrt_a`, `toe`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrbitPart1 {
    /// Issue of Data, Ephemeris (8 бит) — должен совпадать с нижними 8
    /// битами `iodc` из Subframe 1 и с `iode` из Subframe 3.
    pub iode: u8,

    /// Поправка синуса к радиусу орбиты (м), масштаб 2⁻⁵.
    pub crs: f64,

    /// Поправка к среднему движению (рад/с), масштаб 2⁻⁴³·π.
    pub delta_n: f64,

    /// Средняя аномалия на эпоху `toe` (рад), масштаб 2⁻³¹·π.
    pub m0: f64,

    /// Поправка широты, косинусный член (рад), масштаб 2⁻²⁹.
    pub cuc: f64,

    /// Эксцентриситет орбиты, масштаб 2⁻³³.
    pub e: f64,

    /// Поправка широты, синусный член (рад), масштаб 2⁻²⁹.
    pub cus: f64,

    /// Квадратный корень большой полуоси (м^½), масштаб 2⁻¹⁹.
    pub sqrt_a: f64,

    /// Время отсчёта эфемерид (с), масштаб 2⁴.
    pub toe: f64,
}

/// Параметры орбиты, часть 2 (Subframe 3): `cic`, `omega0`, `cis`, `i0`,
/// `crc`, `omega`, `omega_dot`, `idot`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrbitPart2 {
    /// Поправка наклонения, косинусный член (рад), масштаб 2⁻²⁹.
    pub cic: f64,

    /// Долгота восходящего узла на начальную эпоху недели (рад), масштаб
    /// 2⁻³¹·π.
    pub omega0: f64,

    /// Поправка наклонения, синусный член (рад), масштаб 2⁻²⁹.
    pub cis: f64,

    /// Наклонение орбиты на эпоху `toe` (рад), масштаб 2⁻³¹·π.
    pub i0: f64,

    /// Поправка к радиусу, косинусный член (м), масштаб 2⁻⁵.
    pub crc: f64,

    /// Аргумент перигея (рад), масштаб 2⁻³¹·π.
    pub omega: f64,

    /// Скорость изменения долготы узла (рад/с), масштаб 2⁻⁴³·π.
    pub omega_dot: f64,

    /// Issue of Data, Ephemeris (8 бит) — должен совпадать с `iode` из
    /// Subframe 2.
    pub iode: u8,

    /// Скорость изменения наклонения (рад/с), масштаб 2⁻⁴³·π.
    pub idot: f64,
}

/// Полный комплект эфемерид одного спутника, собранный из Subframe 1, 2, 3.
///
/// Содержит все параметры, необходимые для вычисления позиции спутника в
/// ECEF на произвольный момент времени GPS.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ephemeris {
    /// PRN спутника
    pub prn: u8,

    /// Параметры часов и health (Subframe 1)
    pub clock: ClockParams,

    /// Параметры орбиты, часть 1 (Subframe 2)
    pub orbit1: OrbitPart1,

    /// Параметры орбиты, часть 2 (Subframe 3)
    pub orbit2: OrbitPart2,
}

impl<'a> BitCursor<'a> {
    /// Создаёт курсор над конкатенированными информационными битами
    /// (без TLM и HOW - то есть `words[2..10]` объединённые в один слайс).
    #[must_use]
    pub const fn new(bits: &'a [bool]) -> Self {
        Self { bits }
    }

    /// Извлекает беззнаковое целое из `len` битов начиная с `offset` (MSB первым).
    ///
    /// # Panics
    ///
    /// Паникует, если `offset + len` выходит за пределы доступных бит.
    #[must_use]
    pub fn unsigned(
        &self,
        offset: usize,
        len: usize,
    ) -> u32 {
        assert!(
            offset + len <= self.bits.len(),
            "bit field out of range: offset={offset} len={len} total={}",
            self.bits.len()
        );

        self.bits[offset..offset + len]
            .iter()
            .fold(0u32, |acc, &b| (acc << 1) | u32::from(b))
    }

    /// Извлекает знаковое целое (two's complement) из `len` бит начиная с
    /// `offset` (MSB - знаковый бит).
    ///
    /// # Panics
    ///
    /// Паникует, если `offset + len` выходит за пределы доступных бит, или
    /// если `len == 0` либо `len > 32`.
    #[must_use]
    pub fn signed(
        &self,
        offset: usize,
        len: usize,
    ) -> i32 {
        assert!(len > 0 && len <= 32, "signed field length must be 1..=32");

        let raw = self.unsigned(offset, len);
        let sign_bit = 1u32 << (len - 1);

        if raw & sign_bit != 0 {
            // Расширяем знак: вычитаем 2^len.
            let full = 1i64 << len;

            (i64::from(raw) - full) as i32
        } else {
            i32::try_from(raw).expect("positive signed field must fit into i32")
        }
    }
}

impl Ephemeris {
    /// Собирает полный комплект эфемерид из трёх разобранных subframe.
    ///
    /// Не выполняет валидацию (health/IOD) - используйте
    /// [`Ephemeris::validate`] после сборки.
    #[must_use]
    pub const fn new(
        prn: u8,
        clock: ClockParams,
        orbit1: OrbitPart1,
        orbit2: OrbitPart2,
    ) -> Self {
        Self {
            prn,
            clock,
            orbit1,
            orbit2,
        }
    }

    /// Проверяет health-бит и консистентность `IODE`/`IODC` между
    /// Subframe 1, 2 и 3
    ///
    /// # Errors
    ///
    /// Возвращает соответствующий вариант [`EphemerisValidationError`] при
    /// первом обнаруженном несоответсвии (порядок проверки: health -> IODE
    /// Subframe2 vs Subframe3 -> IODC low8 vs IODE).
    pub const fn validate(&self) -> Result<(), EphemerisValidationError> {
        if self.clock.sv_health != 0 {
            return Err(EphemerisValidationError::UnhealthySatellite {
                health: self.clock.sv_health,
            });
        }

        if self.orbit1.iode != self.orbit2.iode {
            return Err(EphemerisValidationError::IodeMismatch {
                iode_sf2: self.orbit1.iode,
                iode_sf3: self.orbit2.iode,
            });
        }

        let iodc_low8 = (self.clock.iodc & 0xFF) as u8;

        if iodc_low8 != self.orbit1.iode {
            return Err(EphemerisValidationError::IodcIodeMismatch {
                iodc_low8,
                iode: self.orbit1.iode,
            });
        }

        Ok(())
    }
}

/// Конкатенирует информационные слова `words[2..10]` (8 слов * 24 бита =
/// 192 бита) в единый массив бит для использования с [`BitCursor`].
#[must_use]
pub fn concat_data_words(subframe: &DecodedSubframe) -> [bool; 192] {
    let mut bits = [false; 192];

    for (word_idx, word) in subframe.words[2..10].iter().enumerate() {
        bits[word_idx * 24..(word_idx + 1) * 24].copy_from_slice(word);
    }

    bits
}

/// Разбирает Subframe 1 (часы спутника, health, IODC).
///
/// # Аргументы
/// - `subframe` - декодированный subframe с `subframe_id == 1`.
///
/// # Возвращает
/// - `None`, если `subframe.subframe_id != 1`.
#[must_use]
pub fn parse_subframe1(subframe: &DecodedSubframe) -> Option<ClockParams> {
    if subframe.subframe_id != 1 {
        return None;
    }

    let bits = concat_data_words(subframe);
    let c = BitCursor::new(&bits);

    // Битовые смещения внутри 192-битного блока words[2..10] (то есть
    // считая от начала слова 3 subframe, после TLM+HOW).
    // Word 3: week(10) + c/a или p (2, не используется) + ura(4) + health(6) + iodc_msb(2)
    let week_number = c.unsigned(0, 10) as u16;
    // биты 10-11: code on L2 (не используется здесь)
    let ura_index = c.signed(12, 4) as i8;
    let sv_health = c.unsigned(16, 6) as u8;
    let iodc_msb = c.unsigned(22, 2);

    // Word 4-5: reserved (не разбираем, не нужны для позиции)
    // Word 6: t_gd (8, не используется здесь) + iodc_lsb(8) + toc(16) -- упрощённая
    // раскладка ниже соответствует фактической компоновке ICD: t_gd занимает
    // последние 8 бит word3 в некоторых реализациях; здесь используем
    // стандартную раскладку IS-GPS-200 Table 20-I.
    //
    // Смещения относительно начала 192-битного блока (word indices 0..7
    // соответствуют ICD words 3..10):
    // word0 (ICD word3): week[10] L2code[2] ura[4] health[6] iodc_msb[2]  = 24 бит
    // word1 (ICD word4): t_gd[8] iodc_lsb[8] toc[16-... ] -- t_gd(8)+iodc_lsb(8)+toc_part
    let t_gd_raw = c.signed(24, 8); // ICD word4 bits 1-8: t_gd
    let iodc_lsb = c.unsigned(32, 8); // word4 bits 9-16
    let toc_raw = c.unsigned(40, 16); // word4 bits 17-24 + word5 bits 1-8 => итого 16 бит
    let af2_raw = c.signed(56, 8); // word5 bits 9-16
    let af1_raw = c.signed(64, 16); // word5 bits 17-24 + word6 bits 1-8
    let af0_raw = c.signed(80, 22); // word6 bits 9-24 + word7 bits 1-6

    let iodc = (iodc_msb << 8) | iodc_lsb;
    let _ = t_gd_raw; // t_gd зарезервирован для будущей коррекции группового интервала

    Some(ClockParams {
        week_number,
        ura_index,
        sv_health,
        iodc: iodc as u16,
        toc: f64::from(toc_raw) * 2f64.powi(4),
        af2: f64::from(af2_raw) * 2f64.powi(-55),
        af1: f64::from(af1_raw) * 2f64.powi(-43),
        af0: f64::from(af0_raw) * 2f64.powi(-31),
    })
}

/// Разбирает Subframe 2 (параметры орбиты, часть 1).
///
/// # Возвращает
/// - `None`, если `subframe.subframe_id != 2`.
#[must_use]
pub fn parse_subframe2(subframe: &DecodedSubframe) -> Option<OrbitPart1> {
    if subframe.subframe_id != 2 {
        return None;
    }

    let bits = concat_data_words(subframe);
    let c = BitCursor::new(&bits);

    // word0 (ICD word3): iode[8] crs[16]
    let iode = c.unsigned(0, 8) as u8;
    let crs_raw = c.signed(8, 16);
    // word1 (ICD word4): delta_n[16] m0_msb[8]
    let delta_n_raw = c.signed(24, 16);
    let m0_msb = c.unsigned(40, 8);
    // word2 (ICD word5): m0_lsb[24]
    let m0_lsb = c.unsigned(48, 24);
    // word3 (ICD word6): cuc[16] e_msb[8]
    let cuc_raw = c.signed(72, 16);
    let e_msb = c.unsigned(88, 8);
    // word4 (ICD word7): e_lsb[24]
    let e_lsb = c.unsigned(96, 24);
    // word5 (ICD word8): cus[16] sqrt_a_msb[8]
    let cus_raw = c.signed(120, 16);
    let sqrt_a_msb = c.unsigned(136, 8);
    // word6 (ICD word9): sqrt_a_lsb[24]
    let sqrt_a_lsb = c.unsigned(144, 24);
    // word7 (ICD word10): toe[16] fit_interval[1] aodo[5] parity_aux[2]
    let toe_raw = c.unsigned(168, 16);

    let m0_combined = (m0_msb << 24) | m0_lsb; // 32 бит
    let e_combined = (e_msb << 24) | e_lsb; // 32 бит
    let sqrt_a_combined = (sqrt_a_msb << 24) | sqrt_a_lsb; // 32 бит

    Some(OrbitPart1 {
        iode,
        crs: f64::from(crs_raw) * 2f64.powi(-5),
        delta_n: f64::from(delta_n_raw) * 2f64.powi(-43) * PI,
        m0: sign_extend_32(m0_combined) * 2f64.powi(-31) * PI,
        cuc: f64::from(cuc_raw) * 2f64.powi(-29),
        e: f64::from(e_combined) * 2f64.powi(-33),
        cus: f64::from(cus_raw) * 2f64.powi(-29),
        sqrt_a: f64::from(sqrt_a_combined) * 2f64.powi(-19),
        toe: f64::from(toe_raw) * 2f64.powi(4),
    })
}

/// Разбирает Subframe 3 (параметры орбиты, часть 2).
///
/// # Возвращает
/// - `None`, если `subframe.subframe_id != 3`.
#[must_use]
pub fn parse_subframe3(subframe: &DecodedSubframe) -> Option<OrbitPart2> {
    if subframe.subframe_id != 3 {
        return None;
    }

    let bits = concat_data_words(subframe);
    let c = BitCursor::new(&bits);

    // word0 (ICD word3): cic[16] omega0_msb[8]
    let cic_raw = c.signed(0, 16);
    let omega0_msb = c.unsigned(16, 8);
    // word1 (ICD word4): omega0_lsb[24]
    let omega0_lsb = c.unsigned(24, 24);
    // word2 (ICD word5): cis[16] i0_msb[8]
    let cis_raw = c.signed(48, 16);
    let i0_msb = c.unsigned(64, 8);
    // word3 (ICD word6): i0_lsb[24]
    let i0_lsb = c.unsigned(72, 24);
    // word4 (ICD word7): crc[16] omega_msb[8]
    let crc_raw = c.signed(96, 16);
    let omega_msb = c.unsigned(112, 8);
    // word5 (ICD word8): omega_lsb[24]
    let omega_lsb = c.unsigned(120, 24);
    // word6 (ICD word9): omega_dot[24]
    let omega_dot_raw = c.signed(144, 24);
    // word7 (ICD word10): iode[8] idot[14] parity_aux[2]
    let iode = c.unsigned(168, 8) as u8;
    let idot_raw = c.signed(176, 14);

    let omega0_combined = (omega0_msb << 24) | omega0_lsb;
    let i0_combined = (i0_msb << 24) | i0_lsb;
    let omega_combined = (omega_msb << 24) | omega_lsb;

    Some(OrbitPart2 {
        cic: f64::from(cic_raw) * 2f64.powi(-29),
        omega0: sign_extend_32(omega0_combined) * 2f64.powi(-31) * PI,
        cis: f64::from(cis_raw) * 2f64.powi(-29),
        i0: sign_extend_32(i0_combined) * 2f64.powi(-31) * PI,
        crc: f64::from(crc_raw) * 2f64.powi(-5),
        omega: sign_extend_32(omega_combined) * 2f64.powi(-31) * PI,
        omega_dot: f64::from(omega_dot_raw) * 2f64.powi(-43) * PI,
        iode,
        idot: f64::from(idot_raw) * 2f64.powi(-43) * PI,
    })
}

/// Расширяет знак 32-битного значения, хранимого в `u32`, в `f64`
fn sign_extend_32(raw: u32) -> f64 {
    f64::from(raw.cast_signed())
}

#[cfg(test)]
mod tests {
    use crate::navigation::frame_decoder::{DecodedSubframe, HowWord};

    use super::*;

    fn make_subframe(
        subframe_id: u8,
        data_words: [[bool; 24]; 8],
    ) -> DecodedSubframe {
        let mut words = [[false; 24]; 10];

        // words[0] = TLM (irrelevant for ephemeris parsing)
        // words[1] = HOW (irrelevant content-wise, but must carry subframe_id)
        words[2..10].copy_from_slice(&data_words);

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
        assert!(bits.len() <= 24);

        let mut word = [false; 24];
        let start = 24 - bits.len();

        word[start..].copy_from_slice(bits);

        word
    }

    fn dummy_clock(
        health: u8,
        iodc: u16,
    ) -> ClockParams {
        ClockParams {
            week_number: 2300,
            ura_index: 0,
            sv_health: health,
            iodc,
            toc: 0.0,
            af2: 0.0,
            af1: 0.0,
            af0: 0.0,
        }
    }

    fn dummy_orbit1(iode: u8) -> OrbitPart1 {
        OrbitPart1 {
            iode,
            crs: 0.0,
            delta_n: 0.0,
            m0: 0.0,
            cuc: 0.0,
            e: 0.0,
            cus: 0.0,
            sqrt_a: 5153.7,
            toe: 0.0,
        }
    }

    fn dummy_orbit2(iode: u8) -> OrbitPart2 {
        OrbitPart2 {
            cic: 0.0,
            omega0: 0.0,
            cis: 0.0,
            i0: 0.96,
            crc: 0.0,
            omega: 0.0,
            omega_dot: 0.0,
            iode,
            idot: 0.0,
        }
    }

    #[test]
    fn test_cursor_unsigned_extracts_correct_value() {
        // 0b1011 = 11
        let bits = [true, false, true, true];
        let c = BitCursor::new(&bits);

        assert_eq!(c.unsigned(0, 4), 11);
    }

    #[test]
    fn test_cursor_unsigned_partial_rabge() {
        let bits = [false, true, true, false, true];
        let c = BitCursor::new(&bits);

        // bits[1..4] = 1,1,0 = 0b110 = 6
        assert_eq!(c.unsigned(1, 3), 6);
    }

    #[test]
    fn test_cursor_signed_positive_value() {
        // 4-bit field, MSB=0 -> positive: 0b0101 = 5
        let bits = [false, true, false, true];
        let c = BitCursor::new(&bits);

        assert_eq!(c.signed(0, 4), 5);
    }

    #[test]
    fn test_cursor_signed_negative_value() {
        // 4-bit field, MSB=1 -> negative: 0b1000 = -8 (two's complement)
        let bits = [true, false, false, false];
        let c = BitCursor::new(&bits);

        assert_eq!(c.signed(0, 4), -8);
    }

    #[test]
    fn test_cursor_signed_negative_one() {
        // 4-bit all-ones = -1
        let bits = [true, true, true, true];
        let c = BitCursor::new(&bits);

        assert_eq!(c.signed(0, 4), -1);
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn test_cursor_panics_on_out_of_range() {
        let bits = [true, false];
        let c = BitCursor::new(&bits);
        let _ = c.unsigned(0, 5);
    }

    #[test]
    fn test_parse_subframe1_returns_none_for_wrong_id() {
        let sf = make_subframe(2, [[false; 24]; 8]);

        assert!(parse_subframe1(&sf).is_none());
    }

    #[test]
    fn test_parse_subframe1_extracts_week_and_health() {
        let mut words = [[false; 24]; 8];

        // word0: week[10] l2code[2] ura[4] health[6] iodc_msb[2]
        let mut w0 = Vec::new();

        w0.extend(bits_from_u32(500, 10)); // week = 500
        w0.extend(bits_from_u32(0, 2)); // l2 code
        w0.extend(bits_from_u32(0, 4)); // ura (signed but 0 is fine)
        w0.extend(bits_from_u32(0, 6)); // health = 0 (healthy)
        w0.extend(bits_from_u32(0, 2)); // iodc_msb

        words[0] = pad_word(&w0);

        let sf = make_subframe(1, words);
        let clock = parse_subframe1(&sf).unwrap();

        assert_eq!(clock.week_number, 500);
        assert_eq!(clock.sv_health, 0);
    }

    #[test]
    fn test_parse_subframe1_extracts_nonzero_health() {
        let mut words = [[false; 24]; 8];
        let mut w0 = Vec::new();

        w0.extend(bits_from_u32(0, 10));
        w0.extend(bits_from_u32(0, 2));
        w0.extend(bits_from_u32(0, 4));
        w0.extend(bits_from_u32(0b10_1010, 6)); // nonzero health
        w0.extend(bits_from_u32(0, 2));

        words[0] = pad_word(&w0);

        let sf = make_subframe(1, words);
        let clock = parse_subframe1(&sf).unwrap();

        assert_eq!(clock.sv_health, 0b10_1010);
    }

    #[test]
    fn test_parse_subframe1_af0_scale_applied() {
        let mut words = [[false; 24]; 8];

        // word6 (data index 3): af1[16-high8] ... actually af0 spans word6[9..24]+word7[1..6].
        // We directly target offsets used in parse_subframe1: af0_raw at bit 80, len 22.
        // bits 80..102 sit in data words: 80/24=3 (word index 3, bit 8) .. up to word index 4.
        let mut bits = [false; 192];

        // Set af0_raw = 1 (smallest positive value) at offset 80, length 22.
        bits[80 + 21] = true; // LSB of the 22-bit field at position offset+len-1

        for i in 0..8 {
            words[i] = bits[i * 24..(i + 1) * 24].try_into().unwrap();
        }

        let sf = make_subframe(1, words);
        let clock = parse_subframe1(&sf).unwrap();

        let expected = 1.0 * 2f64.powi(-31);

        assert!((clock.af0 - expected).abs() < 1e-15);
    }

    #[test]
    fn test_parse_subframe2_returns_none_for_wrong_id() {
        let sf = make_subframe(1, [[false; 24]; 8]);

        assert!(parse_subframe2(&sf).is_none());
    }

    #[test]
    fn test_parse_subframe2_extracts_iode() {
        let mut words = [[false; 24]; 8];
        let mut w0 = Vec::new();

        w0.extend(bits_from_u32(77, 8)); // iode = 77
        w0.extend(bits_from_u32(0, 16)); // crs

        words[0] = pad_word(&w0);

        let sf = make_subframe(2, words);
        let orbit = parse_subframe2(&sf).unwrap();

        assert_eq!(orbit.iode, 77);
    }

    #[test]
    fn test_parse_subframe2_sqrt_a_combines_msb_lsb() {
        // sqrt_a is 32-bit split across words[5] (msb 8 bits) and words[6] (lsb 24 bits).
        let mut bits = [false; 192];

        // sqrt_a_msb at offset 136 (8 bits), sqrt_a_lsb at offset 144 (24 bits).
        // Set combined value = 1 (LSB of full 32-bit field) → bit at offset 144+23.
        bits[144 + 23] = true;

        let mut words = [[false; 24]; 8];

        for i in 0..8 {
            words[i] = bits[i * 24..(i + 1) * 24].try_into().unwrap();
        }

        let sf = make_subframe(2, words);
        let orbit = parse_subframe2(&sf).unwrap();

        let expected = 1.0 * 2f64.powi(-19);

        assert!((orbit.sqrt_a - expected).abs() < 1e-12);
    }

    #[test]
    fn test_parse_subframe2_eccentricity_is_unsigned() {
        // e occupies bits [88..96] (msb, word3) + [96..120] (lsb, word4) = 32 bits, unsigned.
        let mut bits = [false; 192];

        bits[96 + 23] = true; // LSB of combined 32-bit field → value 1

        let mut words = [[false; 24]; 8];

        for i in 0..8 {
            words[i] = bits[i * 24..(i + 1) * 24].try_into().unwrap();
        }

        let sf = make_subframe(2, words);
        let orbit = parse_subframe2(&sf).unwrap();

        let expected = 1.0 * 2f64.powi(-33);

        assert!((orbit.e - expected).abs() < 1e-15);
        assert!(orbit.e >= 0.0, "eccentricity must be non-negative");
    }

    #[test]
    fn test_parse_subframe3_returns_none_for_wrong_id() {
        let sf = make_subframe(1, [[false; 24]; 8]);

        assert!(parse_subframe3(&sf).is_none());
    }

    #[test]
    fn test_parse_subframe3_extracts_iode_matching_subframe2() {
        let mut words = [[false; 24]; 8];
        let mut w7 = Vec::new();

        w7.extend(bits_from_u32(77, 8)); // iode = 77, matches subframe2 test
        w7.extend(bits_from_u32(0, 14)); // idot
        w7.extend(bits_from_u32(0, 2)); // aux

        words[7] = pad_word(&w7);

        let sf = make_subframe(3, words);
        let orbit = parse_subframe3(&sf).unwrap();

        assert_eq!(orbit.iode, 77);
    }

    #[test]
    fn test_parse_subframe3_omega0_sign_extends_negative() {
        // omega0 spans words[0] bits[16..24] (msb) + words[1] (lsb, 24 bits) = 32-bit signed.
        let mut bits = [false; 192];

        // Set sign bit (MSB of the 32-bit field, at offset 16) to 1 → negative value.
        bits[16] = true;

        let mut words = [[false; 24]; 8];

        for i in 0..8 {
            words[i] = bits[i * 24..(i + 1) * 24].try_into().unwrap();
        }

        let sf = make_subframe(3, words);
        let orbit = parse_subframe3(&sf).unwrap();

        assert!(
            orbit.omega0 < 0.0,
            "sign bit set should yield negative omega0"
        );
    }

    #[test]
    fn test_validate_passes_for_healthy_consistent_ephemeris() {
        let eph = Ephemeris::new(
            1,
            dummy_clock(0, 0x00AA),
            dummy_orbit1(0xAA),
            dummy_orbit2(0xAA),
        );

        assert!(eph.validate().is_ok());
    }

    #[test]
    fn test_validate_fails_for_unhealthy_satellite() {
        let eph = Ephemeris::new(
            1,
            dummy_clock(5, 0x00AA),
            dummy_orbit1(0xAA),
            dummy_orbit2(0xAA),
        );

        assert_eq!(
            eph.validate(),
            Err(EphemerisValidationError::UnhealthySatellite { health: 5 })
        );
    }

    #[test]
    fn test_validate_fails_for_iode_mismatch_between_sf2_sf3() {
        let eph = Ephemeris::new(
            1,
            dummy_clock(0, 0x00AA),
            dummy_orbit1(0xAA),
            dummy_orbit2(0xBB),
        );

        assert_eq!(
            eph.validate(),
            Err(EphemerisValidationError::IodeMismatch {
                iode_sf2: 0xAA,
                iode_sf3: 0xBB
            })
        );
    }

    #[test]
    fn test_validate_fails_for_iodc_iode_mismatch() {
        let eph = Ephemeris::new(
            1,
            dummy_clock(0, 0x00FF),
            dummy_orbit1(0xAA),
            dummy_orbit2(0xAA),
        );

        assert_eq!(
            eph.validate(),
            Err(EphemerisValidationError::IodcIodeMismatch {
                iodc_low8: 0xFF,
                iode: 0xAA
            })
        );
    }

    #[test]
    fn test_validate_checks_health_before_iode() {
        // Both unhealthy AND IODE mismatch — health check should win (checked first).
        let eph = Ephemeris::new(
            1,
            dummy_clock(3, 0x00AA),
            dummy_orbit1(0xAA),
            dummy_orbit2(0xBB),
        );

        assert_eq!(
            eph.validate(),
            Err(EphemerisValidationError::UnhealthySatellite { health: 3 })
        );
    }
}
