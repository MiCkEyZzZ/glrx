//! RF subsystem errors and unified operation result type.

use std::{error::Error, fmt, time::Duration};

/// Unified error type for RF frontend operations (file + SDR + streaming).
#[derive(Debug)]
pub enum RfError {
    /// I/O error while reading file or stream.
    Io(std::io::Error),

    /// End of IQ file or stream reached.
    EndOfFile,

    /// Unsupported IQ sample format (e.g. i8, i16, f32 mismatch).
    UnsupportedFormat(String),

    /// Ring buffer overflow — samples were dropped.
    BufferOverflow {
        /// Number of dropped samples due to overflow.
        dropped: usize,
    },

    /// Stream interruption detected due to missing samples or timeout gap.
    StreamInterrupted {
        /// Time gap that caused stream interruption.
        gap: Duration,
    },

    /// Generic SDR backend error (`SoapySDR` / RTL-SDR).
    Sdr(String),

    /// Invalid RF or stream configuration.
    Config(String),
}

impl fmt::Display for RfError {
    fn fmt(
        &self,
        f: &mut fmt::Formatter<'_>,
    ) -> fmt::Result {
        match self {
            RfError::Io(err) => write!(f, "I/O error: {err}"),
            RfError::EndOfFile => write!(f, "end of IQ file reached"),
            RfError::UnsupportedFormat(fmt_str) => {
                write!(f, "unsupported sample format: {fmt_str}")
            }
            RfError::BufferOverflow { dropped } => {
                write!(f, "ring buffer overflow — {dropped} samples dropped")
            }
            RfError::StreamInterrupted { gap } => {
                write!(f, "stream interrupted after {gap:?} gap")
            }
            RfError::Sdr(err) => write!(f, "SDR error: {err}"),
            RfError::Config(err) => write!(f, "invalid configuration: {err}"),
        }
    }
}

impl Error for RfError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            RfError::Io(err) => Some(err),
            _ => None,
        }
    }
}

impl From<std::io::Error> for RfError {
    fn from(err: std::io::Error) -> Self {
        RfError::Io(err)
    }
}

/// Result type alias for RF operations.
pub type RfResult<T> = Result<T, RfError>;
