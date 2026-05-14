//! Форматы представления комплексных IQ-отсчётов.
//!
//! Определяет поддерживаемые типы сэмплов (I8, I16, F32)
//! и утилиты для их парсинга и вычисления размера.

use std::str::FromStr;

use crate::rf::error::{RfError, RfResult};

/// Формат комплексных IQ-отсчётов.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleFormat {
    /// 8-битные signed IQ-отсчёты: `i8 + i8`.
    I8,

    /// 16-битные signed IQ-отсчёты: `i16 + i16`.
    I16,

    /// 32-битные floating-point IQ-отсчёты: `f32 + f32`.
    F32,
}

impl SampleFormat {
    /// Возвращает размер одного комплексного отсчёта в байтах.
    #[must_use]
    pub const fn bytes_per_complex_sample(self) -> usize {
        match self {
            SampleFormat::I8 => 2,
            SampleFormat::I16 => 4,
            SampleFormat::F32 => 8,
        }
    }
}

impl FromStr for SampleFormat {
    type Err = RfError;

    /// Парсит строковое представление формата IQ-отсчётов.
    ///
    /// Поддерживаемые значения:
    /// - `i8`, `sc8`, `int8`
    /// - `i16`, `sc16`, `int16`
    /// - `f32`, `fc32`, `float32`
    fn from_str(s: &str) -> RfResult<Self> {
        match s.to_ascii_lowercase().as_str() {
            "i8" | "sc8" | "int8" => Ok(Self::I8),
            "i16" | "sc16" | "int16" => Ok(Self::I16),
            "f32" | "fc32" | "float32" => Ok(Self::F32),
            other => Err(RfError::UnsupportedFormat(other.to_string())),
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// Тесты
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_bytes() {
        assert_eq!(SampleFormat::I8.bytes_per_complex_sample(), 2);
        assert_eq!(SampleFormat::I16.bytes_per_complex_sample(), 4);
        assert_eq!(SampleFormat::F32.bytes_per_complex_sample(), 8);
    }

    #[test]
    fn sample_format_from_str() {
        assert_eq!(SampleFormat::from_str("i8").unwrap(), SampleFormat::I8);
        assert_eq!(SampleFormat::from_str("SC8").unwrap(), SampleFormat::I8);
        assert_eq!(SampleFormat::from_str("f32").unwrap(), SampleFormat::F32);
        assert!(SampleFormat::from_str("u8").is_err());
    }
}
