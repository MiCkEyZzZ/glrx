use std::str::FromStr;

use crate::rf::error::{RfError, RfResult};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleFormat {
    I8,
    I16,
    F32,
}

impl SampleFormat {
    pub const fn bytes_per_sample(self) -> usize {
        match self {
            SampleFormat::I8 => 2,
            SampleFormat::I16 => 4,
            SampleFormat::F32 => 8,
        }
    }
}

impl FromStr for SampleFormat {
    type Err = RfError;

    fn from_str(s: &str) -> RfResult<Self> {
        match s.to_ascii_lowercase().as_str() {
            "i8" | "sc8" | "int8" => Ok(Self::I8),
            "i16" | "sc16" | "int16" => Ok(Self::I16),
            "f32" | "fc32" | "float32" => Ok(Self::F32),
            other => Err(RfError::UnsupportedFormat(other.to_string())),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_bytes() {
        assert_eq!(SampleFormat::I8.bytes_per_sample(), 2);
        assert_eq!(SampleFormat::I16.bytes_per_sample(), 4);
        assert_eq!(SampleFormat::F32.bytes_per_sample(), 8);
    }

    #[test]
    fn sample_format_from_str() {
        assert_eq!(SampleFormat::from_str("i8").unwrap(), SampleFormat::I8);
        assert_eq!(SampleFormat::from_str("SC8").unwrap(), SampleFormat::I8);
        assert_eq!(SampleFormat::from_str("f32").unwrap(), SampleFormat::F32);
        assert!(SampleFormat::from_str("u8").is_err());
    }
}
