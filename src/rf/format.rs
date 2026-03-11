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

    pub fn from_str(s: &str) -> RfResult<Self> {
        match s.to_ascii_lowercase().as_str() {
            "i8" | "sc8" | "int8" => Ok(Self::I8),
            "i16" | "sc16" | "int16" => Ok(Self::I16),
            "f32" | "fc32" | "float32" => Ok(Self::F32),
            other => Err(RfError::UnsupportedFormat(other.to_string())),
        }
    }
}
