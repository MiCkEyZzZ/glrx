use std::time::Duration;

use thiserror::Error;

pub type RfResult<T> = Result<T, RfError>;

#[derive(Debug, Error)]
pub enum RfError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("end if IQ file reached")]
    EndOfFile,

    #[error("unsupported sample format: {0}")]
    UnsupportedFormat(String),

    #[error("ring buffer overflow — {dropped} samples dropped")]
    BufferOverflow { dropped: usize },

    #[error("stream interrupted after {gap:?} gap")]
    StreamInterrupted { gap: Duration },

    #[error("SDR error: {0}")]
    Sdr(String),

    #[error("invalid configuration: {0}")]
    Config(String),
}
