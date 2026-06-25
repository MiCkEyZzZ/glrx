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

/// Курсор для извлечения битовых полей произвольной длины из
/// конкатинированного потока информационных слов subframe.
pub struct BitCursor<'a> {
    bits: &'a [bool],
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

#[cfg(test)]
mod tests {
    use super::*;

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
}
