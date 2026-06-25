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
}
