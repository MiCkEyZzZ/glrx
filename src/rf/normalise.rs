use num_complex::Complex32;

/// Normalise a raw I8 pair to Complex32 in [-1.0, 1.0].
#[inline]
pub(crate) fn norm_i8(
    i: i8,
    q: i8,
) -> Complex32 {
    const SCALE: f32 = 1.0 / 127.0;

    Complex32::new(i as f32 * SCALE, q as f32 * SCALE)
}

/// Normalise a raw I16 pair to Complex32 in [-1.0, 1.0].
#[inline]
pub(crate) fn norm_i16(
    i: i16,
    q: i16,
) -> Complex32 {
    const SCALE: f32 = 1.0 / 32767.0;
    Complex32::new(i as f32 * SCALE, q as f32 * SCALE)
}

/// F32 pair: pass through unchanged.
#[inline]
pub(crate) fn norm_f32(
    i: f32,
    q: f32,
) -> Complex32 {
    Complex32::new(i, q)
}

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
}
