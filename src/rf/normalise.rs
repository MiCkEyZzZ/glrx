//! Нормализация IQ-отсчётов в комплексное представление [`Complex32`].
//!
//! Преобразует различные форматы входных данных (I8, I16, F32)
//! в единый диапазон `[-1.0, 1.0]`, используемый DSP-пайплайном.

use num_complex::Complex32;

/// Нормализует пару I8 сэмплов в Complex32 в диапазоне [-1.0, 1.0].
#[inline]
pub(crate) fn norm_i8(
    i: i8,
    q: i8,
) -> Complex32 {
    const SCALE: f32 = 1.0 / 127.0;

    Complex32::new(f32::from(i) * SCALE, f32::from(q) * SCALE)
}

/// Нормализует пару I16 сэмплов в Complex32 в диапазоне [-1.0, 1.0].
#[inline]
pub(crate) fn norm_i16(
    i: i16,
    q: i16,
) -> Complex32 {
    const SCALE: f32 = 1.0 / 32767.0;
    Complex32::new(f32::from(i) * SCALE, f32::from(q) * SCALE)
}

/// F32 пара: передаётся без изменений.
#[inline]
pub(crate) const fn norm_f32(
    i: f32,
    q: f32,
) -> Complex32 {
    Complex32::new(i, q)
}

////////////////////////////////////////////////////////////////////////////////
// Tests
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_norm_i8_extremes() {
        let max = norm_i8(127, -127);

        assert!((max.re - 1.0).abs() < 0.01);
        assert!((max.im + 1.0).abs() < 0.01);
    }

    #[test]
    fn test_norm_i16_zero() {
        let z = norm_i16(0, 0);

        assert_eq!(z, Complex32::new(0.0, 0.0));
    }

    #[test]
    fn test_norm_i16_extremes() {
        let max = norm_i16(32767, -32767);
        assert!((max.re - 1.0).abs() < 1e-6);
        assert!((max.im + 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_norm_f32_pass_through() {
        let x = norm_f32(0.5, -0.5);
        assert_eq!(x, Complex32::new(0.5, -0.5));
    }
}
